//! The Map tab: the vehicle's GPS position and this session's track on OpenStreetMap tiles. The position
//! comes from GPS_RAW_INT (`lat` and `lon` in degE7, `cog`, `fix_type`) and nothing is computed from it. The
//! tiles are fetched over HTTPS only for what is on the screen and kept on disk (`cache_dir`); no coordinate
//! is written anywhere and nothing is sent to the vehicle.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::time::Duration;

use eframe::egui::{
    self, Align, Color32, Label, Layout, Pos2, Rect, RichText, Shape, Stroke, UiBuilder, pos2, vec2,
};
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{MessageEntry, MessageStore};
use walkers::sources::OpenStreetMap;
use walkers::{
    HeaderValue, HttpOptions, HttpTiles, MapMemory, Position, Projector, Tiles, lat_lon,
};

use super::{View, ViewCtx};
use crate::status::GREY;

/// Sent with every tile request (OpenStreetMap's tile policy asks for one that names the application).
pub const USER_AGENT: &str = concat!(
    "MARV-GCS/",
    env!("CARGO_PKG_VERSION"),
    " (MARV V2 ground station)"
);
/// Always drawn on the map.
pub const ATTRIBUTION: &str = "© OpenStreetMap contributors";
/// The most positions the track keeps; the oldest are dropped.
pub const TRACK_MAX: usize = 10_000;
/// A GPS_RAW_INT older than this is not a position.
const GPS_STALE: Duration = Duration::from_secs(3);
/// GPS_RAW_INT `cog` when unknown.
const UNKNOWN_COURSE: f64 = 65_535.0;
/// Zoom of the world view, before any position, and of the first position.
const WORLD_ZOOM: f64 = 2.0;
const VEHICLE_ZOOM: f64 = 16.0;
const ARROW: f32 = 11.0;

pub const WAITING: &str = "waiting for a GPS fix";
pub const NO_FIX: &str = "no GPS fix";
pub const NO_POSITION: &str = "no position";

/// Where the tiles are kept: `$XDG_CACHE_HOME/marv-gcs/tiles`, else `$HOME/.cache/marv-gcs/tiles`; `None`
/// when neither is set to an absolute path (the tiles are then not kept). The folder is created when the
/// first tile is stored.
pub fn cache_dir(xdg_cache_home: Option<&str>, home: Option<&str>) -> Option<PathBuf> {
    let absolute = |value: Option<&str>| value.map(PathBuf::from).filter(|path| path.is_absolute());
    let base =
        absolute(xdg_cache_home).or_else(|| absolute(home).map(|home| home.join(".cache")))?;
    Some(base.join("marv-gcs").join("tiles"))
}

fn environment_cache_dir() -> Option<PathBuf> {
    cache_dir(
        std::env::var("XDG_CACHE_HOME").ok().as_deref(),
        std::env::var("HOME").ok().as_deref(),
    )
}

/// A position the vehicle reported with a fix.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Fix {
    /// Degrees.
    lat: f64,
    lon: f64,
    /// Degrees clockwise from north.
    course: Option<f64>,
}

/// What the latest GPS_RAW_INT was.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Latest {
    seen: Duration,
    has_fix: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Status {
    /// No position yet in this session.
    Waiting,
    /// A fix, and the message is young.
    Live,
    /// The latest message is young but has no 2D fix: the marker is the last known position.
    NoFix,
    /// The latest message is old, or there is no session.
    NoPosition,
}

/// 2D fix or better: `fix_type` 2 and up (as text `GPS_FIX_TYPE_3D_FIX`, or as a number).
fn has_fix(entry: &MessageEntry) -> bool {
    match entry.text("fix_type") {
        Some(text) => matches!(
            text.strip_prefix("GPS_FIX_TYPE_").unwrap_or(text),
            "2D_FIX" | "3D_FIX" | "DGPS" | "RTK_FLOAT" | "RTK_FIXED" | "STATIC" | "PPP"
        ),
        None => entry.number("fix_type").is_some_and(|n| n >= 2.0),
    }
}

/// The position of a GPS_RAW_INT that has a fix and coordinates on the globe.
fn read_fix(entry: &MessageEntry) -> Option<Fix> {
    if !has_fix(entry) {
        return None;
    }
    let lat = entry.number("lat")? / 1e7;
    let lon = entry.number("lon")? / 1e7;
    if !(lat.abs() <= 90.0 && lon.abs() <= 180.0) {
        return None;
    }
    let course = entry
        .number("cog")
        .filter(|c| *c != UNKNOWN_COURSE && c.is_finite())
        .map(|c| c / 100.0);
    Some(Fix { lat, lon, course })
}

/// The positions of this session and what the latest GPS_RAW_INT said.
#[derive(Debug, Default)]
struct Tracker {
    track: VecDeque<(f64, f64)>,
    /// The latest position that had a fix.
    last: Option<Fix>,
    latest: Option<Latest>,
    /// Session time of the GPS_RAW_INT already taken in.
    handled: Option<Duration>,
}

impl Tracker {
    /// Take in the latest GPS_RAW_INT, once.
    fn update(&mut self, store: &MessageStore) {
        let Some(entry) = store.entry("GPS_RAW_INT", 0) else {
            return;
        };
        if self.handled == Some(entry.last_seen) {
            return;
        }
        self.handled = Some(entry.last_seen);
        let fix = read_fix(entry);
        self.latest = Some(Latest {
            seen: entry.last_seen,
            has_fix: fix.is_some(),
        });
        if let Some(fix) = fix {
            self.last = Some(fix);
            self.track.push_back((fix.lat, fix.lon));
            while self.track.len() > TRACK_MAX {
                self.track.pop_front();
            }
        }
    }

    fn status(&self, now: Option<Duration>) -> Status {
        if self.last.is_none() {
            return Status::Waiting;
        }
        let young =
            |latest: &Latest| now.is_some_and(|now| now.saturating_sub(latest.seen) <= GPS_STALE);
        match &self.latest {
            Some(latest) if young(latest) && latest.has_fix => Status::Live,
            Some(latest) if young(latest) => Status::NoFix,
            _ => Status::NoPosition,
        }
    }
}

enum TileSource {
    /// No tiles at all (the tests, and every view list built by `views::all`).
    None,
    /// Created at the first frame, which has the egui context.
    OpenStreetMap,
    Ready(Box<HttpTiles>),
}

pub struct MapView {
    tracker: Tracker,
    memory: MapMemory,
    /// The map has been zoomed to the vehicle once, at its first position.
    zoomed: bool,
    tiles: TileSource,
}

impl MapView {
    /// `online`: fetch OpenStreetMap tiles; otherwise the map has no background and nothing is fetched.
    pub fn new(online: bool) -> Self {
        Self {
            tracker: Tracker::default(),
            memory: world_view(),
            zoomed: false,
            tiles: if online {
                TileSource::OpenStreetMap
            } else {
                TileSource::None
            },
        }
    }
}

/// The tiles, created at the first frame that wants them.
fn tiles<'a>(source: &'a mut TileSource, ctx: &egui::Context) -> Option<&'a mut dyn Tiles> {
    if matches!(source, TileSource::OpenStreetMap) {
        let options = HttpOptions {
            cache: environment_cache_dir(),
            user_agent: Some(HeaderValue::from_static(USER_AGENT)),
            ..HttpOptions::default()
        };
        *source = TileSource::Ready(Box::new(HttpTiles::with_options(
            OpenStreetMap,
            options,
            ctx.clone(),
        )));
    }
    match source {
        TileSource::Ready(tiles) => Some(&mut **tiles as &mut dyn Tiles),
        _ => None,
    }
}

fn world_view() -> MapMemory {
    let mut memory = MapMemory::default();
    let _ = memory.set_zoom(WORLD_ZOOM);
    memory
}

impl View for MapView {
    fn name(&self) -> &'static str {
        "Map"
    }

    fn session_changed(&mut self) {
        self.tracker = Tracker::default();
        self.memory = world_view();
        self.zoomed = false;
    }

    fn update(&mut self, state: &VehicleState, _now: Option<Duration>) {
        self.tracker.update(&state.messages);
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        self.tracker.update(&cx.state.messages);
        if !self.zoomed && self.tracker.last.is_some() {
            self.zoomed = true;
            let _ = self.memory.set_zoom(VEHICLE_ZOOM);
        }
        let status = self.tracker.status(cx.now);
        let vehicle = self
            .tracker
            .last
            .map_or_else(Position::default, |fix| lat_lon(fix.lat, fix.lon));

        controls(ui, &mut self.memory, &self.tracker, status, vehicle);
        ui.add_space(2.0);

        let ctx = ui.ctx().clone();
        let tiles = tiles(&mut self.tiles, &ctx);
        let tracker = &self.tracker;
        walkers::Map::new(tiles, &mut self.memory, vehicle)
            .zoom_with_ctrl(false)
            .show(ui, |ui, response, projector, _| {
                draw_track(ui, projector, tracker, status);
                overlay_status(ui, response.rect, status);
                overlay_attribution(ui, response.rect);
            });
    }
}

fn controls(
    ui: &mut egui::Ui,
    memory: &mut MapMemory,
    tracker: &Tracker,
    status: Status,
    vehicle: Position,
) {
    ui.horizontal(|ui| {
        let known = tracker.last.is_some();
        let mut follow = memory.detached().is_none();
        if ui.checkbox(&mut follow, "Follow vehicle").changed() {
            if follow {
                memory.follow_my_position();
            } else {
                memory.center_at(vehicle);
            }
        }
        let centre = ui.add_enabled(known && !follow, egui::Button::new("Centre on vehicle"));
        if centre.clicked() {
            memory.center_at(vehicle);
        }
        if ui.button("+").clicked() {
            let _ = memory.zoom_in();
        }
        if ui.button("-").clicked() {
            let _ = memory.zoom_out();
        }
        if let Some(fix) = tracker.last {
            let text = format!("{:.6}, {:.6}", fix.lat, fix.lon);
            let colour = if status == Status::Live {
                ui.visuals().text_color()
            } else {
                GREY
            };
            ui.label(RichText::new(text).color(colour));
        }
    });
}

fn draw_track(ui: &egui::Ui, projector: &Projector, tracker: &Tracker, status: Status) {
    let painter = ui.painter();
    let live = status == Status::Live;
    if tracker.track.len() > 1 {
        let points: Vec<Pos2> = tracker
            .track
            .iter()
            .map(|&(lat, lon)| to_pos2(projector, lat_lon(lat, lon)))
            .collect();
        let colour = if live {
            Color32::from_rgb(40, 110, 220)
        } else {
            GREY
        };
        painter.add(Shape::line(points, Stroke::new(2.0, colour)));
    }
    if let Some(fix) = tracker.last {
        let at = to_pos2(projector, lat_lon(fix.lat, fix.lon));
        let (fill, outline) = if live {
            (Color32::from_rgb(230, 60, 40), Color32::WHITE)
        } else {
            (GREY, Color32::DARK_GRAY)
        };
        match fix.course {
            Some(course) => {
                let angle = course.to_radians() as f32;
                let forward = vec2(angle.sin(), -angle.cos());
                let side = vec2(-forward.y, forward.x);
                painter.add(Shape::convex_polygon(
                    vec![
                        at + forward * ARROW,
                        at - forward * ARROW * 0.7 + side * ARROW * 0.6,
                        at - forward * ARROW * 0.3,
                        at - forward * ARROW * 0.7 - side * ARROW * 0.6,
                    ],
                    fill,
                    Stroke::new(1.5, outline),
                ));
            }
            None => {
                painter.circle(at, ARROW * 0.6, fill, Stroke::new(1.5, outline));
            }
        }
    }
}

fn to_pos2(projector: &Projector, position: Position) -> Pos2 {
    let v = projector.project(position);
    pos2(v.x, v.y)
}

fn text_on_map(text: &str, colour: Color32) -> Label {
    Label::new(
        RichText::new(text)
            .color(colour)
            .background_color(Color32::from_white_alpha(215)),
    )
}

fn overlay_status(ui: &mut egui::Ui, area: Rect, status: Status) {
    let (text, colour) = match status {
        Status::Live => return,
        Status::Waiting => (WAITING, Color32::BLACK),
        Status::NoFix => (NO_FIX, Color32::from_rgb(150, 100, 0)),
        Status::NoPosition => (NO_POSITION, Color32::from_rgb(150, 100, 0)),
    };
    let area = area.shrink(6.0);
    ui.new_child(
        UiBuilder::new()
            .max_rect(area)
            .layout(Layout::top_down(Align::Min)),
    )
    .add(text_on_map(text, colour));
}

fn overlay_attribution(ui: &mut egui::Ui, area: Rect) {
    let area = area.shrink(4.0);
    ui.new_child(
        UiBuilder::new()
            .max_rect(area)
            .layout(Layout::bottom_up(Align::Max)),
    )
    .add(text_on_map(ATTRIBUTION, Color32::BLACK));
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcs_mavlink::services::messages::{Field, FieldValue, MessageKey};

    const LAT: f64 = 45.0;
    const LON: f64 = 9.0;

    fn number(name: &str, v: f64) -> Field {
        Field {
            name: name.into(),
            value: FieldValue::Number(v),
        }
    }

    fn text(name: &str, v: &str) -> Field {
        Field {
            name: name.into(),
            value: FieldValue::Text(v.into()),
        }
    }

    fn gps(fix: Field, lat: f64, lon: f64, cog: f64, seen: f64) -> MessageStore {
        let mut store = MessageStore::default();
        put(&mut store, fix, lat, lon, cog, seen);
        store
    }

    fn put(store: &mut MessageStore, fix: Field, lat: f64, lon: f64, cog: f64, seen: f64) {
        store.entries.insert(
            MessageKey {
                system_id: 1,
                component_id: 1,
                message_id: 24,
                instance: 0,
            },
            MessageEntry {
                name: "GPS_RAW_INT".into(),
                label: "GPS_RAW_INT".into(),
                count: 1,
                rate_hz: 1.0,
                last_seen: Duration::from_secs_f64(seen),
                fields: vec![
                    fix,
                    number("lat", (lat * 1e7).round()),
                    number("lon", (lon * 1e7).round()),
                    number("cog", cog),
                ],
            },
        );
    }

    fn fixed(kind: &str) -> Field {
        text("fix_type", &format!("GPS_FIX_TYPE_{kind}"))
    }

    fn secs(s: f64) -> Option<Duration> {
        Some(Duration::from_secs_f64(s))
    }

    #[test]
    fn the_position_is_degrees_from_degrees_e7() {
        let store = gps(fixed("3D_FIX"), LAT, LON, 9000.0, 10.0);
        let mut tracker = Tracker::default();
        tracker.update(&store);
        let fix = tracker.last.expect("a position");
        assert!((fix.lat - LAT).abs() < 1e-9 && (fix.lon - LON).abs() < 1e-9);
        assert_eq!(fix.course, Some(90.0));
        let store = gps(fixed("3D_FIX"), -33.5, -70.25, 65_535.0, 11.0);
        tracker.update(&store);
        let fix = tracker.last.expect("a position");
        assert!((fix.lat + 33.5).abs() < 1e-9 && (fix.lon + 70.25).abs() < 1e-9);
        assert_eq!(fix.course, None);
    }

    #[test]
    fn no_marker_below_a_2d_fix() {
        for kind in ["NO_GPS", "NO_FIX"] {
            let mut tracker = Tracker::default();
            tracker.update(&gps(fixed(kind), LAT, LON, 0.0, 10.0));
            assert_eq!(tracker.last, None, "{kind}");
            assert!(tracker.track.is_empty(), "{kind}");
            assert_eq!(tracker.status(secs(10.5)), Status::Waiting, "{kind}");
        }
        for n in [0.0, 1.0] {
            let mut tracker = Tracker::default();
            tracker.update(&gps(number("fix_type", n), LAT, LON, 0.0, 10.0));
            assert_eq!(tracker.last, None, "fix_type {n}");
        }
        for kind in ["2D_FIX", "3D_FIX", "DGPS", "RTK_FLOAT", "RTK_FIXED"] {
            let mut tracker = Tracker::default();
            tracker.update(&gps(fixed(kind), LAT, LON, 0.0, 10.0));
            assert!(tracker.last.is_some(), "{kind}");
        }
        let mut tracker = Tracker::default();
        tracker.update(&gps(number("fix_type", 2.0), LAT, LON, 0.0, 10.0));
        assert!(tracker.last.is_some());
    }

    #[test]
    fn coordinates_off_the_globe_are_not_a_position() {
        let mut tracker = Tracker::default();
        tracker.update(&gps(fixed("3D_FIX"), 91.0, LON, 0.0, 10.0));
        tracker.update(&gps(fixed("3D_FIX"), LAT, 181.0, 0.0, 11.0));
        assert_eq!(tracker.last, None);
    }

    #[test]
    fn a_position_is_live_for_3_seconds_then_the_last_one_stays_grey() {
        let mut tracker = Tracker::default();
        assert_eq!(tracker.status(secs(1.0)), Status::Waiting);
        tracker.update(&gps(fixed("3D_FIX"), LAT, LON, 0.0, 10.0));
        assert_eq!(tracker.status(secs(10.0)), Status::Live);
        assert_eq!(tracker.status(secs(13.0)), Status::Live);
        assert_eq!(tracker.status(secs(13.1)), Status::NoPosition);
        assert_eq!(tracker.status(None), Status::NoPosition);
        assert!(tracker.last.is_some());
    }

    #[test]
    fn a_young_message_without_a_fix_says_no_gps_fix_and_keeps_the_last_position() {
        let mut tracker = Tracker::default();
        tracker.update(&gps(fixed("3D_FIX"), LAT, LON, 0.0, 10.0));
        tracker.update(&gps(fixed("NO_FIX"), 0.0, 0.0, 0.0, 12.0));
        assert_eq!(tracker.status(secs(12.5)), Status::NoFix);
        let fix = tracker.last.expect("the last position stays");
        assert!((fix.lat - LAT).abs() < 1e-9);
        assert_eq!(tracker.track.len(), 1);
        assert_eq!(tracker.status(secs(20.0)), Status::NoPosition);
    }

    #[test]
    fn the_track_takes_each_new_message_once_and_only_with_a_fix() {
        let mut tracker = Tracker::default();
        let mut store = gps(fixed("3D_FIX"), LAT, LON, 0.0, 10.0);
        tracker.update(&store);
        tracker.update(&store);
        assert_eq!(tracker.track.len(), 1);
        put(&mut store, fixed("NO_FIX"), 0.0, 0.0, 0.0, 11.0);
        tracker.update(&store);
        assert_eq!(tracker.track.len(), 1);
        put(&mut store, fixed("2D_FIX"), LAT + 0.001, LON, 0.0, 12.0);
        tracker.update(&store);
        assert_eq!(tracker.track.len(), 2);
        assert!((tracker.track[1].0 - (LAT + 0.001)).abs() < 1e-9);
    }

    #[test]
    fn the_track_drops_the_oldest_past_10_000() {
        let mut tracker = Tracker::default();
        let mut store = MessageStore::default();
        for i in 0..TRACK_MAX + 5 {
            put(
                &mut store,
                fixed("3D_FIX"),
                10.0 + i as f64 * 1e-5,
                LON,
                0.0,
                1.0 + i as f64,
            );
            tracker.update(&store);
        }
        assert_eq!(tracker.track.len(), TRACK_MAX);
        assert!((tracker.track[0].0 - (10.0 + 5.0 * 1e-5)).abs() < 1e-9);
        let newest = 10.0 + (TRACK_MAX + 4) as f64 * 1e-5;
        assert!((tracker.track[TRACK_MAX - 1].0 - newest).abs() < 1e-9);
    }

    #[test]
    fn a_new_session_clears_the_track_and_the_view() {
        let mut view = MapView::new(false);
        let mut state = VehicleState::default();
        put(&mut state.messages, fixed("3D_FIX"), LAT, LON, 0.0, 10.0);
        view.update(&state, secs(10.0));
        assert_eq!(view.tracker.track.len(), 1);
        view.memory.center_at(lat_lon(1.0, 2.0));
        view.session_changed();
        assert!(view.tracker.track.is_empty());
        assert_eq!(view.tracker.last, None);
        assert_eq!(view.tracker.status(secs(10.0)), Status::Waiting);
        assert!(view.memory.detached().is_none(), "follow is on again");
        assert_eq!(view.memory.zoom(), WORLD_ZOOM);
        assert!(!view.zoomed);
    }

    fn draw(view: &mut MapView, state: &VehicleState, now: f64) {
        let mut harness = egui_kittest::Harness::new_ui(|ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state,
                commands: &mut commands,
                now: secs(now),
                access: gcs_mavlink::Access::Configure,
            };
            view.ui(ui, &mut cx);
        });
        harness.run();
    }

    #[test]
    fn the_first_position_zooms_to_the_vehicle_whichever_comes_first() {
        let mut state = VehicleState::default();
        put(&mut state.messages, fixed("3D_FIX"), LAT, LON, 0.0, 10.0);

        // The position arrives (through `update`) before the tab is first drawn.
        let mut view = MapView::new(false);
        view.update(&state, secs(10.0));
        assert_eq!(view.memory.zoom(), WORLD_ZOOM);
        draw(&mut view, &state, 10.5);
        assert_eq!(view.memory.zoom(), VEHICLE_ZOOM);

        // The tab is open and shows the world when the position arrives.
        let mut view = MapView::new(false);
        draw(&mut view, &VehicleState::default(), 9.0);
        assert_eq!(view.memory.zoom(), WORLD_ZOOM);
        view.update(&state, secs(10.0));
        draw(&mut view, &state, 10.5);
        assert_eq!(view.memory.zoom(), VEHICLE_ZOOM);

        // Only the first position zooms: the user's zoom stays.
        view.memory.set_zoom(10.0).unwrap();
        put(&mut state.messages, fixed("3D_FIX"), LAT, LON, 0.0, 11.0);
        draw(&mut view, &state, 11.5);
        assert_eq!(view.memory.zoom(), 10.0);

        // A new session starts over.
        view.session_changed();
        draw(&mut view, &state, 12.0);
        assert_eq!(view.memory.zoom(), VEHICLE_ZOOM);
    }

    #[test]
    fn the_map_follows_by_default() {
        let view = MapView::new(false);
        assert!(view.memory.detached().is_none());
        assert_eq!(view.memory.zoom(), WORLD_ZOOM);
    }

    #[test]
    fn the_user_agent_names_the_application_and_has_no_personal_data() {
        assert_eq!(
            USER_AGENT,
            format!(
                "MARV-GCS/{} (MARV V2 ground station)",
                env!("CARGO_PKG_VERSION")
            )
        );
        assert!(!USER_AGENT.contains('@'));
        assert!(HeaderValue::from_str(USER_AGENT).is_ok());
    }

    #[test]
    fn the_cache_is_the_xdg_cache_folder_or_home_dot_cache() {
        let expected = |base: &str| PathBuf::from(base).join("marv-gcs").join("tiles");
        assert_eq!(
            cache_dir(Some("/x/cache"), Some("/home/u")),
            Some(expected("/x/cache"))
        );
        assert_eq!(
            cache_dir(None, Some("/home/u")),
            Some(expected("/home/u/.cache"))
        );
        assert_eq!(
            cache_dir(Some(""), Some("/home/u")),
            Some(expected("/home/u/.cache"))
        );
        assert_eq!(
            cache_dir(Some("relative"), Some("/home/u")),
            Some(expected("/home/u/.cache"))
        );
        assert_eq!(cache_dir(None, None), None);
        assert!(
            cache_dir(None, Some("/home/u"))
                .unwrap()
                .ends_with(".cache/marv-gcs/tiles")
        );
    }
}
