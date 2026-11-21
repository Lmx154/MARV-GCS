//! The Map tab: the vehicle's GPS position and this session's track on a dark map drawn from OpenFreeMap's
//! vector tiles (OpenStreetMap data), and a waypoint editor. The position comes from GPS_RAW_INT (`lat` and
//! `lon` in degE7, `cog`, `fix_type`) and nothing is computed from it. The tiles are fetched over HTTPS only
//! for what is on the screen and kept on disk (`cache_dir`); the vehicle's position is never written anywhere
//! and nothing is sent to the vehicle. The marker turns by the yaw of ATTITUDE (radians, 0 = north, clockwise)
//! while that message is fresh (the HUD's rule); otherwise by the GPS course when there is one, otherwise it is
//! a circle. The map is always north up.
//!
//! Waypoints: a left click on the map adds one to the end of the current list (while "Add on click" is on),
//! a right click within `HIT_RADIUS` of one removes it. The side panel edits the lists (new, rename,
//! duplicate, delete), the points (coordinates typed or pasted, altitude, order) and saves every change to
//! the list's file (`crate::missions`, QGC WPL 110). The waypoints are not sent to the vehicle.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::time::Duration;

use eframe::egui::{
    self, Align, Color32, Label, Layout, Pos2, Rect, RichText, Shape, Stroke, UiBuilder, pos2, vec2,
};
use gcs_mavlink::VehicleState;
use gcs_mavlink::services::messages::{MessageEntry, MessageStore};
use walkers::sources::{Attribution as SourceAttribution, TileSource as TileSourceTrait};
use walkers::{
    Drawable, HeaderValue, HttpOptions, HttpTiles, MapMemory, Position, Projector, Style,
    StyleLayer, Text as MapText, Tile, TileId, Tiles, Value, lat_lon,
};

use super::hud::ATTITUDE_STALE;
use super::{View, ViewCtx};
use crate::missions::{self, Store, Waypoint};
use crate::status::GREY;
use crate::theme;

/// Sent with every tile request (the tile policies ask for one that names the application).
pub const USER_AGENT: &str = concat!(
    "MARV-GCS/",
    env!("CARGO_PKG_VERSION"),
    " (MARV V2 ground station)"
);
/// Always drawn on the map.
pub const ATTRIBUTION: &str = "OpenFreeMap © OpenMapTiles © OpenStreetMap contributors";
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

/// OpenFreeMap's vector tiles (<https://openfreemap.org>: no key, no account, no limit on requests; the
/// OpenMapTiles schema, made from OpenStreetMap data), 512 px with data up to zoom 14. The station draws their
/// areas, lines and labels itself (`draw_base`), so they stay sharp at every zoom. The raster maps tried
/// before have no keyless dark tile past zoom 16 (Esri's Dark Gray Canvas) or need a key (CARTO since
/// 2026-09-25, Stadia).
pub struct OpenFreeMap;

/// The size of an OpenFreeMap tile, in the pixels its drawables are given in.
const TILE_SIZE: u32 = 512;
/// The last zoom with tiles; closer, the zoom 14 tile is drawn larger.
const DATA_ZOOM: u8 = 14;

impl TileSourceTrait for OpenFreeMap {
    fn tile_url(&self, tile_id: TileId) -> String {
        format!(
            "https://tiles.openfreemap.org/planet/20261115_080022_pt/{}/{}/{}.pbf",
            tile_id.zoom, tile_id.x, tile_id.y
        )
    }

    fn attribution(&self) -> SourceAttribution {
        SourceAttribution {
            text: ATTRIBUTION,
            url: "https://openfreemap.org",
            logo_light: None,
            logo_dark: None,
        }
    }

    fn tile_size(&self) -> u32 {
        TILE_SIZE
    }

    fn max_zoom(&self) -> u8 {
        DATA_ZOOM
    }
}

/// walkers' dark style for the OpenMapTiles schema, with each colour of its palette replaced by the
/// station's: grey land, lighter roads (highways the lightest), buildings a step above the land, muted water,
/// light grey labels. Stations and peaks keep walkers' colours.
const RECOLOUR: [(&str, &str); 20] = [
    ("#000000", "#2e3035"), // land, and the rail under its ties
    ("#061009", "#2b3a2f"), // parks and woods (half transparent)
    ("#000c00", "#2f3b33"), // allotments, playgrounds
    ("#0a0a0a", "#3a3c41"), // piers
    ("#101010", "#34363b"), // tunnel edges
    ("#141414", "#26282c"), // label halos
    ("#191919", "#36383d"), // hospitals, pedestrian areas, aerodromes
    ("#1f1f1f", "#55585e"), // slip roads, beaches
    ("#292929", "#44474d"), // buildings, tunnels
    ("#3a2312", "#4f4a44"), // paths and tracks
    ("#333333", "#45474c"), // runways
    ("#352121", "#7a6f62"), // motorways and trunk roads
    ("#464646", "#5d6066"), // other roads
    ("#5c5c5c", "#a3a7ad"), // minor labels
    ("#707070", "#c3c7cc"), // labels, borders
    ("#999999", "#dadde1"), // towns and cities
    ("#161e31", "#1f2d3d"), // water
    ("#134164", "#7d98b3"), // water labels
    ("#0d1526", "#1f2d3d"), // water label halos
    ("#bfbfbf", "#8a8d92"), // rail ties
];

fn dark_style() -> Style {
    let mut style = Style::openmaptiles_basemap_dark();
    for layer in &mut style.layers {
        let paint = match layer {
            StyleLayer::Background { paint }
            | StyleLayer::Fill { paint, .. }
            | StyleLayer::Line { paint, .. } => Some(paint),
            StyleLayer::Symbol { paint, .. } => paint.as_mut(),
            _ => None,
        };
        let Some(paint) = paint else { continue };
        for colour in [
            &mut paint.background_color,
            &mut paint.fill_color,
            &mut paint.line_color,
            &mut paint.text_color,
            &mut paint.text_halo_color,
        ]
        .into_iter()
        .flatten()
        {
            recolour(&mut colour.0);
        }
    }
    style
}

/// Replaces the palette's colours wherever they are in a style value, expressions included.
fn recolour(value: &mut Value) {
    match value {
        Value::String(text) => {
            if let Some((_, to)) = RECOLOUR
                .iter()
                .find(|(from, _)| text.eq_ignore_ascii_case(from))
            {
                *text = (*to).to_owned();
            }
        }
        Value::Array(items) => items.iter_mut().for_each(recolour),
        _ => {}
    }
}

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
    OpenFreeMap,
    Ready(Box<HttpTiles>),
}

pub struct MapView {
    tracker: Tracker,
    memory: MapMemory,
    /// The map has been zoomed to the vehicle once, at its first position.
    zoomed: bool,
    tiles: TileSource,
    waypoints: WaypointEditor,
}

impl MapView {
    /// `online`: fetch tiles and keep the waypoint lists in the data folder; otherwise the map has no
    /// background, nothing is fetched and the lists stay in memory (the tests).
    pub fn new(online: bool) -> Self {
        Self::with_store(online, online.then(Store::from_environment).flatten())
    }

    /// As `new`, with the waypoint lists in `store` (`None`: in memory only).
    pub fn with_store(online: bool, store: Option<Store>) -> Self {
        Self {
            waypoints: WaypointEditor::new(store),
            tracker: Tracker::default(),
            memory: world_view(),
            zoomed: false,
            tiles: if online {
                TileSource::OpenFreeMap
            } else {
                TileSource::None
            },
        }
    }
}

/// The tiles, created at the first frame that wants them.
fn tiles<'a>(source: &'a mut TileSource, ctx: &egui::Context) -> Option<&'a mut HttpTiles> {
    if matches!(source, TileSource::OpenFreeMap) {
        let options = HttpOptions {
            cache: environment_cache_dir(),
            user_agent: Some(HeaderValue::from_static(USER_AGENT)),
            ..HttpOptions::default()
        };
        *source = TileSource::Ready(Box::new(HttpTiles::with_options_and_style(
            OpenFreeMap,
            options,
            dark_style(),
            ctx.clone(),
        )));
    }
    match source {
        TileSource::Ready(tiles) => Some(&mut **tiles),
        _ => None,
    }
}

/// The tile `(x, y)` of `zoom` that holds `position`, kept on the grid.
fn tile_at(position: Position, zoom: u8) -> (u32, u32) {
    let count = f64::from(1u32 << zoom);
    let x = (position.x() + 180.0) / 360.0 * count;
    let y = (1.0 - position.y().to_radians().tan().asinh() / std::f64::consts::PI) / 2.0 * count;
    let cell = |v: f64| v.floor().clamp(0.0, count - 1.0) as u32;
    (cell(x), cell(y))
}

/// The north-west corner of the tile `(x, y)` of `zoom`.
fn tile_corner(x: u32, y: u32, zoom: u8) -> Position {
    let count = f64::from(1u32 << zoom);
    let lon = f64::from(x) / count * 360.0 - 180.0;
    let lat = (std::f64::consts::PI * (1.0 - 2.0 * f64::from(y) / count))
        .sinh()
        .atan()
        .to_degrees();
    lat_lon(lat, lon)
}

/// Draws the vector tiles on the screen part of the map, below everything else. walkers can draw them only
/// with wgpu and the window uses glow, so the station paints each tile's areas, lines and labels with egui.
/// A tile drawn larger than its 512 px (closer than zoom 15) widens its lines with it, so that a road keeps
/// its width on the ground instead of turning into a hairline.
fn draw_base(ui: &egui::Ui, projector: &Projector, tiles: &mut HttpTiles) {
    let area = projector.clip_rect();
    let painter = ui.painter().with_clip_rect(area.intersect(ui.clip_rect()));
    let zoom = projector.zoom();
    // As walkers chooses them: a 512 px tile stands for the zoom one closer.
    let tile_zoom = (zoom.round() - 1.0).clamp(0.0, f64::from(DATA_ZOOM)) as u8;
    let (min_x, min_y) = tile_at(projector.unproject(area.left_top().to_vec2()), tile_zoom);
    let (max_x, max_y) = tile_at(
        projector.unproject(area.right_bottom().to_vec2()),
        tile_zoom,
    );
    let corner = |x: u32, y: u32| projector.project(tile_corner(x, y, tile_zoom)).to_pos2();
    let mut labels = Vec::new();
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let Some(piece) = tiles.at(TileId {
                x,
                y,
                zoom: tile_zoom,
            }) else {
                continue;
            };
            let Tile::Vector { drawables, texts } = &piece.tile else {
                continue;
            };
            let rect = Rect::from_min_max(corner(x, y), corner(x + 1, y + 1));
            // A tile still loading is shown as its part (`uv`) of a farther one.
            let size = rect.size() / piece.uv.size();
            let full = Rect::from_min_size(rect.min - size * piece.uv.min.to_vec2(), size);
            let scale = full.width() / TILE_SIZE as f32;
            let place = |p: Pos2| full.min + p.to_vec2() * scale;
            let tile_painter = painter.with_clip_rect(rect.intersect(painter.clip_rect()));
            let visible = tile_painter.clip_rect();
            let local = Rect::from_min_max(
                ((visible.min - full.min) / scale).to_pos2(),
                ((visible.max - full.min) / scale).to_pos2(),
            );
            for drawable in drawables.iter() {
                match drawable {
                    Drawable::Fill(mesh) => {
                        tile_painter.add(fill_shape(mesh, place));
                    }
                    Drawable::Lines(lines) => {
                        for line in lines {
                            let bounds = Rect::from_points(&line.points).expand(line.width);
                            if bounds.intersects(local) {
                                tile_painter.add(line_shape(line, scale, place));
                            }
                        }
                    }
                }
            }
            labels.extend(
                texts
                    .iter()
                    .filter(|text| text.min_zoom.is_none_or(|min| zoom - 1.0 >= f64::from(min)))
                    .map(|text| MapText {
                        position: place(text.position),
                        max_width: text.max_width.map(|width| width * scale),
                        ..text.clone()
                    }),
            );
        }
    }
    painter.extend(walkers::place_texts(labels, ui.ctx()));
}

fn fill_shape(mesh: &walkers::Mesh, place: impl Fn(Pos2) -> Pos2) -> Shape {
    Shape::mesh(egui::Mesh {
        indices: mesh.indices.clone(),
        vertices: mesh
            .vertices
            .iter()
            .map(|vertex| egui::epaint::Vertex {
                pos: place(vertex.position),
                uv: egui::epaint::WHITE_UV,
                color: vertex.color,
            })
            .collect(),
        ..egui::Mesh::default()
    })
}

fn line_shape(line: &walkers::Line, scale: f32, place: impl Fn(Pos2) -> Pos2) -> Shape {
    let points: Vec<Pos2> = line.points.iter().map(|&p| place(p)).collect();
    let stroke = Stroke::new(line.width * scale, line.color);
    let dashes: Vec<f32> = line
        .dasharray
        .iter()
        .step_by(2)
        .map(|d| d * scale)
        .collect();
    let gaps: Vec<f32> = line
        .dasharray
        .iter()
        .skip(1)
        .step_by(2)
        .map(|g| g * scale)
        .collect();
    let dashed = !dashes.is_empty()
        && dashes.len() == gaps.len()
        && dashes.iter().chain(&gaps).all(|&length| length > 0.1);
    if dashed {
        Shape::Vec(Shape::dashed_line_with_offset(
            &points, stroke, &dashes, &gaps, 0.0,
        ))
    } else {
        Shape::line(points, stroke)
    }
}

/// The marker's heading in radians clockwise from north: a fresh ATTITUDE's yaw, else the GPS course.
fn marker_heading(store: &MessageStore, now: Option<Duration>, course: Option<f64>) -> Option<f32> {
    let yaw = store
        .entry("ATTITUDE", 0)
        .filter(|e| now.is_some_and(|now| now.saturating_sub(e.last_seen) <= ATTITUDE_STALE))
        .and_then(|e| e.number("yaw"))
        .filter(|yaw| yaw.is_finite());
    yaw.or_else(|| course.map(f64::to_radians))
        .map(|a| a as f32)
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

    fn scrolls(&self) -> bool {
        false
    }

    fn session_changed(&mut self) {
        // The waypoints are not part of a session: they stay.
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

        controls(
            ui,
            &mut self.memory,
            &self.tracker,
            status,
            vehicle,
            &mut self.waypoints.panel_open,
        );
        ui.add_space(2.0);
        self.waypoints.hovered = None;
        let area = ui.available_rect_before_wrap();
        let (map_area, panel_area) = if self.waypoints.panel_open {
            let width = PANEL_WIDTH.min(area.width() * 0.5);
            let (map_area, panel_area) = area.split_left_right_at_x(area.right() - width);
            (map_area, Some(panel_area))
        } else {
            (area, None)
        };
        if let Some(panel_area) = panel_area {
            ui.painter().rect_filled(panel_area, 0.0, theme::PANEL);
            ui.painter().vline(
                panel_area.left(),
                panel_area.y_range(),
                Stroke::new(1.0, theme::BORDER),
            );
            let mut panel_ui = ui.new_child(
                UiBuilder::new()
                    .max_rect(panel_area.shrink2(vec2(8.0, 0.0)))
                    .layout(Layout::top_down(Align::Min)),
            );
            panel_ui.set_clip_rect(panel_area.intersect(ui.clip_rect()));
            egui::ScrollArea::vertical()
                .id_salt("map_waypoint_panel")
                .auto_shrink([false, false])
                .show(&mut panel_ui, |ui| {
                    self.waypoints.panel(ui, &mut self.memory)
                });
        }

        let ctx = ui.ctx().clone();
        let tiles = tiles(&mut self.tiles, &ctx);
        let tracker = &self.tracker;
        let waypoints = &mut self.waypoints;
        let heading = marker_heading(
            &cx.state.messages,
            cx.now,
            tracker.last.and_then(|fix| fix.course),
        );
        ui.scope_builder(UiBuilder::new().max_rect(map_area), |ui| {
            walkers::Map::new(None, &mut self.memory, vehicle)
                .zoom_with_ctrl(false)
                .show(ui, |ui, response, projector, _| {
                    if let Some(tiles) = tiles {
                        draw_base(ui, projector, tiles);
                    }
                    draw_track(ui, projector, tracker, status, heading);
                    waypoints.interact_and_draw(ui, response, projector);
                    overlay_status(ui, response.rect, status);
                    let attribution = overlay_attribution(ui, response.rect);
                    overlay_compass(ui, response.rect, attribution);
                });
        });
    }
}

fn controls(
    ui: &mut egui::Ui,
    memory: &mut MapMemory,
    tracker: &Tracker,
    status: Status,
    vehicle: Position,
    waypoints_open: &mut bool,
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
        ui.toggle_value(waypoints_open, "Waypoints");
        if let Some(fix) = tracker.last {
            let text = format!("{:.6}, {:.6}", fix.lat, fix.lon);
            let colour = if status == Status::Live {
                ui.visuals().text_color()
            } else {
                GREY
            };
            theme::status_dot(
                ui,
                if status == Status::Live {
                    theme::OK
                } else {
                    GREY
                },
            );
            ui.label(theme::value(text).color(colour));
        }
    });
}

fn draw_track(
    ui: &egui::Ui,
    projector: &Projector,
    tracker: &Tracker,
    status: Status,
    heading: Option<f32>,
) {
    let painter = ui.painter();
    let live = status == Status::Live;
    if tracker.track.len() > 1 {
        let points: Vec<Pos2> = tracker
            .track
            .iter()
            .map(|&(lat, lon)| to_pos2(projector, lat_lon(lat, lon)))
            .collect();
        let colour = if live { theme::ACCENT } else { GREY };
        painter.add(Shape::line(points, Stroke::new(2.0, colour)));
    }
    if let Some(fix) = tracker.last {
        let at = to_pos2(projector, lat_lon(fix.lat, fix.lon));
        let (fill, outline) = if live {
            (theme::ERROR, theme::TEXT_STRONG)
        } else {
            (GREY, theme::INSET)
        };
        match heading {
            Some(angle) => {
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
            .background_color(Color32::from_black_alpha(190)),
    )
}

fn overlay_status(ui: &mut egui::Ui, area: Rect, status: Status) {
    let (text, colour) = match status {
        Status::Live => return,
        Status::Waiting => (WAITING, theme::TEXT),
        Status::NoFix => (NO_FIX, theme::WARN),
        Status::NoPosition => (NO_POSITION, theme::WARN),
    };
    let area = area.shrink(6.0);
    ui.new_child(
        UiBuilder::new()
            .max_rect(area)
            .layout(Layout::top_down(Align::Min)),
    )
    .add(text_on_map(text, colour));
}

/// Draws the attribution and returns the rectangle it took.
fn overlay_attribution(ui: &mut egui::Ui, area: Rect) -> Rect {
    let area = area.shrink(4.0);
    ui.new_child(
        UiBuilder::new()
            .max_rect(area)
            .layout(Layout::bottom_up(Align::Max)),
    )
    .add(text_on_map(ATTRIBUTION, theme::TEXT_SECONDARY))
    .rect
}

/// Half the side of the compass square.
const COMPASS_RADIUS: f32 = 20.0;

/// The compass square: in the bottom-right corner of the map, its bottom edge just above the attribution.
fn compass_rect(area: Rect, attribution: Rect) -> Rect {
    let right = area.right() - 6.0;
    let bottom = attribution.top().min(area.bottom() - 4.0) - 4.0;
    Rect::from_min_max(
        pos2(right - 2.0 * COMPASS_RADIUS, bottom - 2.0 * COMPASS_RADIUS),
        pos2(right, bottom),
    )
}

/// A small compass rose. The map is always north up (walkers cannot rotate it), so north is straight up.
fn overlay_compass(ui: &egui::Ui, area: Rect, attribution: Rect) {
    let square = compass_rect(area, attribution);
    let painter = ui.painter_at(square.expand(1.0));
    let centre = square.center();
    let ring = theme::TEXT_SECONDARY;
    painter.circle(
        centre,
        COMPASS_RADIUS,
        Color32::from_black_alpha(190),
        Stroke::new(1.0, ring),
    );
    let needle = |tip: Pos2, colour: Color32| {
        let side = vec2(5.0, 0.0);
        let base = centre;
        painter.add(Shape::convex_polygon(
            vec![tip, base + side, base - side],
            colour,
            Stroke::new(0.5, ring),
        ));
    };
    needle(centre + vec2(0.0, -COMPASS_RADIUS * 0.8), theme::ERROR);
    needle(centre + vec2(0.0, COMPASS_RADIUS * 0.8), ring);
    painter.text(
        centre + vec2(0.0, -COMPASS_RADIUS * 0.8) + vec2(0.0, 1.0),
        egui::Align2::CENTER_TOP,
        "N",
        egui::FontId::proportional(9.0),
        theme::TEXT_STRONG,
    );
}

const PANEL_WIDTH: f32 = 280.0;
/// Width of the latitude and longitude, and of the altitude, columns of the waypoint table.
const COORD_WIDTH: f32 = 64.0;
const ALT_WIDTH: f32 = 42.0;
/// A right click this close (screen pixels) to a waypoint's centre takes that waypoint.
pub const HIT_RADIUS: f32 = 10.0;
/// Metres above home of a waypoint added by a click, until the user changes it.
const DEFAULT_ALT_M: f32 = 30.0;
const ALT_RANGE: std::ops::RangeInclusive<f32> = -1_000.0..=100_000.0;
const MARK_RADIUS: f32 = 9.0;
const AMBER: Color32 = theme::WARN;
const NEW_LIST_NAME: &str = "mission";

/// The index of the point nearest to `pointer` and within `radius` of it, if any (the last one on a tie:
/// it is drawn on top).
pub fn hit_test(points: &[Pos2], pointer: Pos2, radius: f32) -> Option<usize> {
    points
        .iter()
        .enumerate()
        .rev()
        .map(|(i, p)| (i, p.distance(pointer)))
        .filter(|(_, d)| *d <= radius)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i)
}

/// `lat, lon` (comma, semicolon or spaces between) as degrees on the globe.
pub fn parse_pair(text: &str) -> Option<(f64, f64)> {
    let mut parts = text
        .split(|c: char| c == ',' || c == ';' || c.is_whitespace())
        .filter(|part| !part.is_empty());
    let lat = parse_coordinate(parts.next()?, 90.0)?;
    let lon = parse_coordinate(parts.next()?, 180.0)?;
    parts.next().is_none().then_some((lat, lon))
}

fn parse_coordinate(text: &str, limit: f64) -> Option<f64> {
    let value: f64 = text.trim().parse().ok()?;
    (value.is_finite() && value.abs() <= limit).then_some(value)
}

fn wrap_longitude(lon: f64) -> f64 {
    (lon + 180.0).rem_euclid(360.0) - 180.0
}

fn format_length(metres: f64) -> String {
    if metres < 1000.0 {
        format!("{metres:.0} m")
    } else {
        format!("{:.2} km", metres / 1000.0)
    }
}

#[derive(Debug, Default, PartialEq)]
enum Mode {
    #[default]
    Idle,
    New(String),
    Rename(String),
    ConfirmDelete,
    ConfirmClear,
}

enum ListAction {
    New(String),
    Rename(String),
    Delete,
    Clear,
}

/// The waypoint list being edited, the lists on disk and the state of the side panel.
struct WaypointEditor {
    store: Option<Store>,
    names: Vec<String>,
    /// The list the points belong to; `None` until one exists (the first added point creates "mission").
    current: Option<String>,
    list: Vec<Waypoint>,
    /// The line under the list buttons and whether it is an error.
    status: Option<(String, bool)>,
    add_on_click: bool,
    default_alt: f32,
    panel_open: bool,
    lat_text: String,
    lon_text: String,
    entry_alt: f32,
    mode: Mode,
    selected: Option<usize>,
    /// The row under the pointer in the panel, this frame.
    hovered: Option<usize>,
}

impl WaypointEditor {
    fn new(store: Option<Store>) -> Self {
        let mut editor = Self {
            store,
            names: Vec::new(),
            current: None,
            list: Vec::new(),
            status: None,
            add_on_click: true,
            default_alt: DEFAULT_ALT_M,
            panel_open: true,
            lat_text: String::new(),
            lon_text: String::new(),
            entry_alt: DEFAULT_ALT_M,
            mode: Mode::Idle,
            selected: None,
            hovered: None,
        };
        editor.refresh_names();
        if let Some(first) = editor.names.first().cloned() {
            editor.select(&first);
        }
        editor
    }

    fn ok(&mut self, text: impl Into<String>) {
        self.status = Some((text.into(), false));
    }

    fn fail(&mut self, text: impl Into<String>) {
        self.status = Some((text.into(), true));
    }

    fn refresh_names(&mut self) {
        if let Some(store) = &self.store {
            match store.names() {
                Ok(names) => self.names = names,
                Err(e) => self.status = Some((e, true)),
            }
        }
    }

    /// Make `name` the current list. A list that cannot be read is not made current (an autosave would
    /// overwrite it).
    fn select(&mut self, name: &str) {
        let Some(store) = &self.store else {
            return;
        };
        self.selected = None;
        self.mode = Mode::Idle;
        match store.load(name) {
            Ok(list) => {
                self.list = list;
                self.current = Some(name.to_string());
                self.status = None;
            }
            Err(e) => {
                self.list.clear();
                self.current = None;
                self.status = Some((e, true));
            }
        }
    }

    /// Save the current list after a change; creates "mission" when there is no list yet.
    fn commit(&mut self) {
        let Some(store) = self.store.clone() else {
            self.fail("not saved: no data folder");
            return;
        };
        let name = self
            .current
            .get_or_insert_with(|| store.unique_name(NEW_LIST_NAME))
            .clone();
        match store.save(&name, &self.list) {
            Ok(()) => {
                self.refresh_names();
                self.ok("saved");
            }
            Err(e) => self.fail(e),
        }
    }

    fn add(&mut self, waypoint: Waypoint) {
        self.list.push(waypoint);
        self.selected = None;
        self.commit();
    }

    fn remove(&mut self, index: usize) {
        if index < self.list.len() {
            self.list.remove(index);
            self.selected = None;
            self.commit();
        }
    }

    fn perform(&mut self, action: ListAction) -> bool {
        let Some(store) = self.store.clone() else {
            self.fail("no data folder: lists cannot be kept");
            return false;
        };
        let result = match action {
            ListAction::New(name) => store.create(&name).map(|()| {
                self.refresh_names();
                self.select(&name);
                self.ok(format!("created \"{name}\""));
            }),
            ListAction::Rename(name) => match self.current.clone() {
                Some(from) => store.rename(&from, &name).map(|()| {
                    self.current = Some(name.clone());
                    self.refresh_names();
                    self.ok(format!("renamed to \"{name}\""));
                }),
                None => Err("no list".to_string()),
            },
            ListAction::Delete => match self.current.clone() {
                Some(name) => store.delete(&name).map(|()| {
                    self.current = None;
                    self.list.clear();
                    self.selected = None;
                    self.refresh_names();
                    if let Some(first) = self.names.first().cloned() {
                        self.select(&first);
                    }
                    self.ok(format!("deleted \"{name}\""));
                }),
                None => Err("no list".to_string()),
            },
            ListAction::Clear => {
                self.list.clear();
                self.selected = None;
                self.commit();
                return true;
            }
        };
        match result {
            Ok(()) => true,
            Err(e) => {
                self.fail(e);
                false
            }
        }
    }

    fn duplicate(&mut self) {
        let (Some(store), Some(from)) = (self.store.clone(), self.current.clone()) else {
            return;
        };
        let name = store.unique_name(&format!("{from} copy"));
        match store.duplicate(&from, &name) {
            Ok(()) => {
                self.refresh_names();
                self.select(&name);
                self.ok(format!("copied to \"{name}\""));
            }
            Err(e) => self.fail(e),
        }
    }

    /// The side panel.
    fn panel(&mut self, ui: &mut egui::Ui, memory: &mut MapMemory) {
        ui.add_space(4.0);
        theme::section_title(ui, "Waypoint list");
        self.list_selector(ui);
        self.list_mode(ui);
        if let Some((text, is_error)) = &self.status {
            let colour = if *is_error { theme::ERROR } else { GREY };
            ui.label(RichText::new(text).color(colour));
        }
        ui.separator();

        ui.checkbox(&mut self.add_on_click, "Add on click");
        ui.label(
            RichText::new("left click: add; right click on a point: remove")
                .small()
                .color(GREY),
        );
        ui.horizontal(|ui| {
            let label = ui.label("Default altitude (m)");
            ui.add(egui::DragValue::new(&mut self.default_alt).range(ALT_RANGE))
                .labelled_by(label.id);
        });
        ui.add_space(2.0);
        self.add_by_coordinates(ui);
        ui.separator();

        ui.horizontal(|ui| {
            ui.label(format!(
                "{} points, {}",
                self.list.len(),
                format_length(missions::path_length_m(&self.list))
            ));
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(
                    !self.list.is_empty(),
                    egui::Button::new("Centre on waypoints"),
                )
                .clicked()
            {
                memory.center_at(centre_of(&self.list));
            }
            if ui
                .add_enabled(!self.list.is_empty(), egui::Button::new("Clear"))
                .clicked()
            {
                self.mode = Mode::ConfirmClear;
            }
        });
        ui.label(
            RichText::new("Waypoints are not sent to the vehicle yet.")
                .small()
                .color(GREY),
        );
        ui.separator();
        self.table(ui);
    }

    fn list_selector(&mut self, ui: &mut egui::Ui) {
        let mut switch = None;
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("map_waypoint_list")
                .selected_text(self.current.as_deref().unwrap_or("(none)"))
                .width(PANEL_WIDTH - 40.0)
                .show_ui(ui, |ui| {
                    for name in &self.names {
                        let is_current = self.current.as_deref() == Some(name);
                        if ui.selectable_label(is_current, name).clicked() && !is_current {
                            switch = Some(name.clone());
                        }
                    }
                })
                .response
                .accessible_name("waypoint list");
        });
        if let Some(name) = switch {
            self.select(&name);
        }
        ui.horizontal_wrapped(|ui| {
            let has_list = self.current.is_some();
            if ui.button("New").clicked() {
                let name = self.store.as_ref().map_or_else(
                    || NEW_LIST_NAME.to_string(),
                    |s| s.unique_name(NEW_LIST_NAME),
                );
                self.mode = Mode::New(name);
            }
            if ui
                .add_enabled(has_list, egui::Button::new("Rename"))
                .clicked()
            {
                self.mode = Mode::Rename(self.current.clone().unwrap_or_default());
            }
            if ui
                .add_enabled(has_list, egui::Button::new("Duplicate"))
                .clicked()
            {
                self.duplicate();
            }
            if ui
                .add_enabled(has_list, egui::Button::new("Delete"))
                .clicked()
            {
                self.mode = Mode::ConfirmDelete;
            }
        });
    }

    /// The name box or the confirmation that a list button opened.
    fn list_mode(&mut self, ui: &mut egui::Ui) {
        let mut mode = std::mem::take(&mut self.mode);
        let mut run = None;
        let mut close = false;
        let is_new = matches!(mode, Mode::New(_));
        match &mut mode {
            Mode::Idle => {}
            Mode::New(text) | Mode::Rename(text) => {
                ui.horizontal(|ui| {
                    let edit = ui.add(egui::TextEdit::singleline(text).desired_width(130.0));
                    let enter = edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if ui
                        .button(if is_new { "Create" } else { "Rename to" })
                        .clicked()
                        || enter
                    {
                        run = Some(if is_new {
                            ListAction::New(text.clone())
                        } else {
                            ListAction::Rename(text.clone())
                        });
                    }
                    close |= ui.button("Cancel").clicked();
                });
            }
            Mode::ConfirmDelete => {
                ui.horizontal(|ui| {
                    ui.label(format!(
                        "Delete \"{}\"?",
                        self.current.as_deref().unwrap_or_default()
                    ));
                    if ui.button("Delete list").clicked() {
                        run = Some(ListAction::Delete);
                    }
                    close |= ui.button("Cancel").clicked();
                });
            }
            Mode::ConfirmClear => {
                ui.horizontal(|ui| {
                    ui.label(format!("Remove all {} points?", self.list.len()));
                    if ui.button("Remove all").clicked() {
                        run = Some(ListAction::Clear);
                    }
                    close |= ui.button("Cancel").clicked();
                });
            }
        }
        if let Some(action) = run {
            close |= self.perform(action);
        }
        // `perform` and `select` may have set a new mode; otherwise keep (or close) this one.
        if self.mode == Mode::Idle {
            self.mode = if close { Mode::Idle } else { mode };
        }
    }

    fn add_by_coordinates(&mut self, ui: &mut egui::Ui) {
        ui.label(theme::muted("Add by coordinates").strong());
        ui.horizontal(|ui| {
            let lat = ui
                .add(
                    egui::TextEdit::singleline(&mut self.lat_text)
                        .hint_text("lat (or lat, lon)")
                        .desired_width(90.0),
                )
                .accessible_name("latitude of the new point");
            if lat.changed()
                && let Some((lat, lon)) = parse_pair(&self.lat_text)
            {
                self.lat_text = format!("{lat}");
                self.lon_text = format!("{lon}");
            }
            ui.add(
                egui::TextEdit::singleline(&mut self.lon_text)
                    .hint_text("lon")
                    .desired_width(70.0),
            )
            .accessible_name("longitude of the new point");
            ui.add(
                egui::DragValue::new(&mut self.entry_alt)
                    .range(ALT_RANGE)
                    .suffix(" m"),
            )
            .accessible_name("altitude of the new point");
        });
        let lat = parse_coordinate(&self.lat_text, 90.0);
        let lon = parse_coordinate(&self.lon_text, 180.0);
        ui.horizontal(|ui| {
            let add = ui.add_enabled(
                lat.is_some() && lon.is_some(),
                egui::Button::new("Add point"),
            );
            if add.clicked()
                && let (Some(lat), Some(lon)) = (lat, lon)
            {
                self.add(Waypoint {
                    lat,
                    lon,
                    alt_m: self.entry_alt,
                });
                self.lat_text.clear();
                self.lon_text.clear();
            }
            let typed = !self.lat_text.is_empty() || !self.lon_text.is_empty();
            if typed && (lat.is_none() || lon.is_none()) {
                ui.label(
                    RichText::new("lat -90 to 90, lon -180 to 180")
                        .small()
                        .color(theme::ERROR),
                );
            }
        });
    }

    fn table(&mut self, ui: &mut egui::Ui) {
        let mut list = std::mem::take(&mut self.list);
        let mut selected = self.selected;
        let mut hovered = None;
        let mut delete = None;
        let mut swap = None;
        let mut changed = false;
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.x = 3.0;
                ui.spacing_mut().button_padding.x = 3.0;
                ui.spacing_mut().interact_size.x = 16.0;
                ui.style_mut().override_text_style = Some(egui::TextStyle::Monospace);
                egui::Grid::new("map_waypoint_table")
                    .num_columns(7)
                    .spacing([3.0, 3.0])
                    .show(ui, |ui| {
                        let header_height = ui.spacing().interact_size.y;
                        for (header, width) in [
                            ("#", 0.0),
                            ("lat", COORD_WIDTH),
                            ("lon", COORD_WIDTH),
                            ("alt m", ALT_WIDTH),
                            ("", 0.0),
                            ("", 0.0),
                            ("", 0.0),
                        ] {
                            let text = RichText::new(header).small().color(GREY);
                            if width > 0.0 {
                                ui.add_sized([width, header_height], Label::new(text));
                            } else {
                                ui.label(text);
                            }
                        }
                        ui.end_row();
                        let last = list.len().saturating_sub(1);
                        for (i, w) in list.iter_mut().enumerate() {
                            let mut over = false;
                            let number =
                                ui.selectable_label(selected == Some(i), format!("{}", i + 1));
                            if number.clicked() {
                                selected = if selected == Some(i) { None } else { Some(i) };
                            }
                            over |= number.hovered();
                            let height = ui.spacing().interact_size.y;
                            let lat = ui
                                .add_sized(
                                    [COORD_WIDTH, height],
                                    egui::DragValue::new(&mut w.lat)
                                        .range(-90.0..=90.0)
                                        .speed(0.00001)
                                        .fixed_decimals(5),
                                )
                                .accessible_name(format!("latitude of point {}", i + 1));
                            let lon = ui
                                .add_sized(
                                    [COORD_WIDTH, height],
                                    egui::DragValue::new(&mut w.lon)
                                        .range(-180.0..=180.0)
                                        .speed(0.00001)
                                        .fixed_decimals(5),
                                )
                                .accessible_name(format!("longitude of point {}", i + 1));
                            let alt = ui
                                .add_sized(
                                    [ALT_WIDTH, height],
                                    egui::DragValue::new(&mut w.alt_m)
                                        .range(ALT_RANGE)
                                        .max_decimals(1),
                                )
                                .accessible_name(format!("altitude of point {}", i + 1));
                            for response in [&lat, &lon, &alt] {
                                over |= response.hovered();
                                changed |= response.changed();
                            }
                            let up = ui.add_enabled(i > 0, egui::Button::new("^").small());
                            let down = ui.add_enabled(i < last, egui::Button::new("v").small());
                            let remove = ui.add(egui::Button::new("x").small());
                            over |= up.hovered() || down.hovered() || remove.hovered();
                            if up.clicked() {
                                swap = Some((i - 1, i));
                            }
                            if down.clicked() {
                                swap = Some((i, i + 1));
                            }
                            if remove.clicked() {
                                delete = Some(i);
                            }
                            if over {
                                hovered = Some(i);
                            }
                            ui.end_row();
                        }
                    });
            });
        self.list = list;
        self.selected = selected;
        self.hovered = hovered;
        if let Some((a, b)) = swap {
            self.list.swap(a, b);
            self.selected = self.selected.map(|s| {
                if s == a {
                    b
                } else if s == b {
                    a
                } else {
                    s
                }
            });
            changed = true;
        }
        if let Some(i) = delete {
            self.remove(i);
        } else if changed {
            self.commit();
        }
    }

    /// Take the clicks on the map, then draw the waypoints.
    fn interact_and_draw(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        projector: &Projector,
    ) {
        let project = |list: &[Waypoint]| -> Vec<Pos2> {
            list.iter()
                .map(|w| to_pos2(projector, lat_lon(w.lat, w.lon)))
                .collect()
        };
        let mut points = project(&self.list);
        if self.panel_open {
            let selected_before = self.selected;
            if let Some(pointer) = response.interact_pointer_pos() {
                let hit = hit_test(&points, pointer, HIT_RADIUS);
                if response.secondary_clicked() {
                    if let Some(i) = hit {
                        self.remove(i);
                    }
                } else if response.clicked() && self.add_on_click {
                    match hit {
                        Some(i) => self.selected = Some(i),
                        None => {
                            let at = projector.unproject(pointer.to_vec2());
                            self.add(Waypoint {
                                lat: at.y(),
                                lon: wrap_longitude(at.x()),
                                alt_m: self.default_alt,
                            });
                        }
                    }
                }
                if self.list.len() != points.len() || self.selected != selected_before {
                    points = project(&self.list);
                    ui.ctx().request_repaint();
                }
            }
        }
        let painter = ui.painter_at(projector.clip_rect());
        if points.len() > 1 {
            painter.add(Shape::line(points.clone(), Stroke::new(2.0, AMBER)));
        }
        for (i, at) in points.iter().enumerate() {
            let highlighted = self.selected == Some(i) || self.hovered == Some(i);
            let (radius, ring) = if highlighted {
                (MARK_RADIUS + 2.0, Stroke::new(2.5, theme::TEXT_STRONG))
            } else {
                (MARK_RADIUS, Stroke::new(1.5, theme::INSET))
            };
            painter.circle(*at, radius, AMBER, ring);
            painter.text(
                *at,
                egui::Align2::CENTER_CENTER,
                format!("{}", i + 1),
                egui::FontId::proportional(11.0),
                theme::INSET,
            );
        }
    }
}

/// The middle of the box around the waypoints.
fn centre_of(list: &[Waypoint]) -> Position {
    let (mut lat_min, mut lat_max) = (f64::MAX, f64::MIN);
    let (mut lon_min, mut lon_max) = (f64::MAX, f64::MIN);
    for w in list {
        lat_min = lat_min.min(w.lat);
        lat_max = lat_max.max(w.lat);
        lon_min = lon_min.min(w.lon);
        lon_max = lon_max.max(w.lon);
    }
    lat_lon((lat_min + lat_max) / 2.0, (lon_min + lon_max) / 2.0)
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
        assert_eq!(TRACK_MAX, 10_000);
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
    fn the_compass_is_in_the_bottom_right_corner_above_the_attribution() {
        let area = Rect::from_min_size(pos2(0.0, 40.0), vec2(800.0, 500.0));
        let attribution = Rect::from_min_max(pos2(640.0, 520.0), pos2(796.0, 536.0));
        let compass = compass_rect(area, attribution);
        assert!(area.contains_rect(compass));
        assert!(!compass.intersects(attribution));
        assert!(compass.center().x > area.center().x && compass.center().y > area.center().y);
        assert!(
            area.right() - compass.right() < 12.0,
            "against the right edge"
        );
        assert!(compass.bottom() <= attribution.top());
    }

    #[test]
    fn the_map_draws_the_compass_and_keeps_the_attribution() {
        use egui_kittest::kittest::Queryable;
        let mut view = MapView::new(false);
        let state = VehicleState::default();
        let mut harness = egui_kittest::Harness::new_ui(|ui| {
            let mut commands = Vec::new();
            let mut cx = ViewCtx {
                state: &state,
                commands: &mut commands,
                now: secs(1.0),
                access: gcs_mavlink::Access::Configure,
            };
            view.ui(ui, &mut cx);
        });
        harness.run();
        harness.get_by_label(ATTRIBUTION);
    }

    fn attitude(yaw: f64, seen: f64) -> MessageStore {
        let mut store = MessageStore::default();
        store.entries.insert(
            MessageKey {
                system_id: 1,
                component_id: 1,
                message_id: 30,
                instance: 0,
            },
            MessageEntry {
                name: "ATTITUDE".into(),
                label: "ATTITUDE".into(),
                count: 1,
                rate_hz: 1.0,
                last_seen: Duration::from_secs_f64(seen),
                fields: vec![number("yaw", yaw)],
            },
        );
        store
    }

    #[test]
    fn the_marker_turns_by_a_fresh_attitude_yaw_before_the_gps_course() {
        let store = attitude(1.0, 10.0);
        assert_eq!(marker_heading(&store, secs(11.0), Some(90.0)), Some(1.0));
        assert_eq!(marker_heading(&store, secs(11.0), None), Some(1.0));
    }

    #[test]
    fn a_stale_attitude_falls_back_to_the_course_and_none_gives_the_circle() {
        let store = attitude(1.0, 10.0);
        let late = secs(10.0 + ATTITUDE_STALE.as_secs_f64() + 0.5);
        assert_eq!(
            marker_heading(&store, late, Some(90.0)),
            Some(90f64.to_radians() as f32)
        );
        assert_eq!(marker_heading(&store, late, None), None);
        assert_eq!(
            marker_heading(&MessageStore::default(), secs(1.0), None),
            None
        );
        assert_eq!(marker_heading(&store, None, None), None);
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

    #[test]
    fn the_tiles_are_openfreemap_vector_tiles_to_zoom_14() {
        let url = OpenFreeMap.tile_url(TileId {
            x: 1,
            y: 2,
            zoom: 5,
        });
        assert!(url.starts_with("https://tiles.openfreemap.org/planet/"));
        assert!(url.ends_with("/5/1/2.pbf"));
        assert_eq!(OpenFreeMap.attribution().text, ATTRIBUTION);
        assert!(ATTRIBUTION.contains("OpenFreeMap") && ATTRIBUTION.contains("OpenMapTiles"));
        assert!(ATTRIBUTION.contains("© OpenStreetMap contributors"));
        assert_eq!(OpenFreeMap.max_zoom(), 14);
        assert_eq!(OpenFreeMap.tile_size(), 512);
    }

    fn colours(value: &Value, out: &mut Vec<String>) {
        match value {
            Value::String(text) if text.starts_with('#') => out.push(text.to_lowercase()),
            Value::Array(items) => items.iter().for_each(|item| colours(item, out)),
            _ => {}
        }
    }

    #[test]
    fn the_style_has_only_the_station_colours_and_grey_land() {
        let style = dark_style();
        let mut found = Vec::new();
        for layer in &style.layers {
            let paint = match layer {
                StyleLayer::Background { paint }
                | StyleLayer::Fill { paint, .. }
                | StyleLayer::Line { paint, .. } => Some(paint),
                StyleLayer::Symbol { paint, .. } => paint.as_ref(),
                _ => None,
            };
            if let Some(paint) = paint {
                for colour in [
                    &paint.background_color,
                    &paint.fill_color,
                    &paint.line_color,
                    &paint.text_color,
                    &paint.text_halo_color,
                ]
                .into_iter()
                .flatten()
                {
                    colours(&colour.0, &mut found);
                }
            }
        }
        assert!(found.len() > 20);
        for colour in &found {
            let walkers_own = RECOLOUR.iter().any(|(from, _)| from == colour);
            let ours = RECOLOUR.iter().any(|(_, to)| to == colour);
            let kept = ["#7fb3d9", "#b59a6a"].contains(&colour.as_str());
            assert!(ours || kept, "{colour} is not one of the station's");
            assert!(!walkers_own || ours, "{colour} was not replaced");
        }
        let StyleLayer::Background { paint } = &style.layers[0] else {
            panic!("the first layer is the background");
        };
        assert_eq!(
            paint.background_color.as_ref().map(|c| c.0.clone()),
            Some(Value::from("#2e3035"))
        );
    }

    #[test]
    fn tiles_are_found_and_placed_on_the_standard_grid() {
        // The simulator's position, at the zoom of the last tiles.
        assert_eq!(tile_at(lat_lon(47.3977, 8.5456), 14), (8580, 5736));
        assert_eq!(tile_at(lat_lon(0.0, 0.0), 0), (0, 0));
        // Off the grid is its edge.
        assert_eq!(tile_at(lat_lon(89.0, 200.0), 3), (7, 0));
        let corner = tile_corner(8580, 5736, 14);
        assert_eq!(
            tile_at(lat_lon(corner.y() - 1e-6, corner.x() + 1e-6), 14),
            (8580, 5736)
        );
        assert_eq!(tile_corner(0, 0, 0).x(), -180.0);
        assert!((tile_corner(0, 0, 0).y() - 85.051_128_8).abs() < 1e-6);
        assert!(tile_corner(1, 1, 1).y().abs() < 1e-9);
    }

    #[test]
    fn a_tile_drawn_larger_widens_its_lines_and_dashes() {
        let line = walkers::Line {
            points: vec![pos2(0.0, 0.0), pos2(10.0, 0.0)],
            width: 0.5,
            color: Color32::WHITE,
            dasharray: Vec::new(),
        };
        let Shape::Path(path) = line_shape(&line, 16.0, |p| p) else {
            panic!("a solid line is one path");
        };
        assert_eq!(path.stroke.width, 8.0);
        let dashed = walkers::Line {
            dasharray: vec![2.0, 1.0],
            ..line.clone()
        };
        let Shape::Vec(dashes) = line_shape(&dashed, 2.0, |p| p) else {
            panic!("a dashed line is its dashes");
        };
        // Dashes of 4 and gaps of 2 along 10 px.
        assert_eq!(dashes.len(), 2);
        // An odd pattern is drawn solid rather than guessed at.
        let odd = walkers::Line {
            dasharray: vec![2.0],
            ..line
        };
        assert!(matches!(line_shape(&odd, 1.0, |p| p), Shape::Path(_)));
    }

    #[test]
    fn a_right_click_takes_the_nearest_point_within_10_pixels() {
        let points = [pos2(100.0, 100.0), pos2(108.0, 100.0), pos2(300.0, 300.0)];
        assert_eq!(hit_test(&points, pos2(107.0, 100.0), HIT_RADIUS), Some(1));
        assert_eq!(hit_test(&points, pos2(101.0, 100.0), HIT_RADIUS), Some(0));
        assert_eq!(hit_test(&points, pos2(300.0, 309.9), HIT_RADIUS), Some(2));
        assert_eq!(hit_test(&points, pos2(300.0, 310.5), HIT_RADIUS), None);
        assert_eq!(hit_test(&points, pos2(200.0, 200.0), HIT_RADIUS), None);
        assert_eq!(hit_test(&[], pos2(0.0, 0.0), HIT_RADIUS), None);
        let same = [pos2(5.0, 5.0), pos2(5.0, 5.0)];
        assert_eq!(
            hit_test(&same, pos2(5.0, 5.0), HIT_RADIUS),
            Some(1),
            "the one on top"
        );
    }

    #[test]
    fn a_pasted_pair_is_split_and_checked() {
        assert_eq!(parse_pair("45.5, 9.25"), Some((45.5, 9.25)));
        assert_eq!(parse_pair("-33.5 -70.25"), Some((-33.5, -70.25)));
        assert_eq!(parse_pair(" 1;2 "), Some((1.0, 2.0)));
        assert_eq!(parse_pair("45.5"), None);
        assert_eq!(parse_pair("1, 2, 3"), None);
        assert_eq!(parse_pair("91, 2"), None);
        assert_eq!(parse_pair("1, 181"), None);
        assert_eq!(parse_pair("a, b"), None);
        assert_eq!(parse_pair("NaN, 1"), None);
        assert_eq!(wrap_longitude(190.0), -170.0);
        assert_eq!(wrap_longitude(-190.0), 170.0);
        assert_eq!(wrap_longitude(10.0), 10.0);
        assert_eq!(format_length(950.4), "950 m");
        assert_eq!(format_length(1234.0), "1.23 km");
    }

    fn temp_store(tag: &str) -> Store {
        Store::new(std::env::temp_dir().join(format!(
            "marv-gcs-map-{tag}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        )))
    }

    #[test]
    fn the_editor_autosaves_creates_mission_and_keeps_points_across_sessions() {
        let store = temp_store("editor");
        let mut view = MapView::with_store(false, Some(store.clone()));
        let w = |lat| Waypoint {
            lat,
            lon: 2.0,
            alt_m: 10.0,
        };
        view.waypoints.add(w(1.0));
        view.waypoints.add(w(2.0));
        assert_eq!(view.waypoints.current.as_deref(), Some("mission"));
        assert_eq!(store.load("mission").unwrap(), vec![w(1.0), w(2.0)]);
        assert_eq!(view.waypoints.status, Some(("saved".to_string(), false)));
        view.session_changed();
        assert_eq!(
            view.waypoints.list.len(),
            2,
            "a new session keeps the waypoints"
        );
        view.waypoints.remove(0);
        assert_eq!(store.load("mission").unwrap(), vec![w(2.0)]);

        assert!(view.waypoints.perform(ListAction::New("second".into())));
        assert!(view.waypoints.list.is_empty());
        view.waypoints.add(w(3.0));
        assert!(
            !view.waypoints.perform(ListAction::New("second".into())),
            "taken"
        );
        assert!(view.waypoints.status.as_ref().unwrap().1);
        assert!(view.waypoints.perform(ListAction::Rename("third".into())));
        view.waypoints.duplicate();
        assert_eq!(view.waypoints.current.as_deref(), Some("third copy"));
        assert_eq!(view.waypoints.list, vec![w(3.0)]);
        assert!(view.waypoints.perform(ListAction::Clear));
        assert_eq!(store.load("third copy").unwrap(), vec![]);
        assert!(view.waypoints.perform(ListAction::Delete));
        assert_eq!(store.names().unwrap(), ["mission", "third"]);
        assert_eq!(view.waypoints.current.as_deref(), Some("mission"));

        // A fresh view finds the lists again.
        let again = MapView::with_store(false, Some(store.clone()));
        assert_eq!(again.waypoints.names, ["mission", "third"]);
        assert_eq!(again.waypoints.list, vec![w(2.0)]);
        let _ = std::fs::remove_dir_all(store.dir());
    }

    #[test]
    fn a_list_that_cannot_be_read_is_not_overwritten() {
        let store = temp_store("corrupt");
        std::fs::create_dir_all(store.dir()).unwrap();
        let file = store.dir().join("mission.waypoints");
        std::fs::write(&file, "garbage").unwrap();
        let mut view = MapView::with_store(false, Some(store.clone()));
        assert!(view.waypoints.status.as_ref().unwrap().1);
        view.waypoints.add(Waypoint {
            lat: 1.0,
            lon: 2.0,
            alt_m: 0.0,
        });
        assert_eq!(std::fs::read_to_string(&file).unwrap(), "garbage");
        assert_eq!(view.waypoints.current.as_deref(), Some("mission 2"));
        let _ = std::fs::remove_dir_all(store.dir());
    }

    #[test]
    fn without_a_data_folder_the_points_stay_in_memory_and_say_so() {
        let mut view = MapView::with_store(false, None);
        view.waypoints.add(Waypoint {
            lat: 1.0,
            lon: 2.0,
            alt_m: 0.0,
        });
        assert_eq!(view.waypoints.list.len(), 1);
        assert!(view.waypoints.status.as_ref().unwrap().1);
    }

    #[test]
    fn a_left_click_on_the_map_adds_a_waypoint_and_a_right_click_on_it_removes_it() {
        use egui_kittest::kittest::Queryable;
        let store = temp_store("click");
        let mut view = MapView::with_store(false, Some(store.clone()));
        let state = VehicleState::default();
        let mut harness = egui_kittest::Harness::builder()
            .with_size(vec2(900.0, 600.0))
            .build_ui(|ui| {
                let mut commands = Vec::new();
                let mut cx = ViewCtx {
                    state: &state,
                    commands: &mut commands,
                    now: secs(1.0),
                    access: gcs_mavlink::Access::Configure,
                };
                view.ui(ui, &mut cx);
            });
        harness.run();
        let click = |harness: &mut egui_kittest::Harness<'_>, at: Pos2, button| {
            let event = |pressed| egui::Event::PointerButton {
                pos: at,
                button,
                pressed,
                modifiers: egui::Modifiers::NONE,
            };
            harness.event(egui::Event::PointerMoved(at));
            harness.run();
            harness.event(event(true));
            harness.run();
            harness.event(event(false));
            harness.run();
        };
        let (first, second) = (pos2(250.0, 300.0), pos2(600.0, 300.0));
        click(&mut harness, first, egui::PointerButton::Primary);
        harness.get_by_label("1 points, 0 m");
        click(&mut harness, second, egui::PointerButton::Primary);
        harness.get_by_label_contains("2 points, ");
        // A right click away from a point does nothing; on a point it removes that point.
        click(
            &mut harness,
            pos2(400.0, 450.0),
            egui::PointerButton::Secondary,
        );
        harness.get_by_label_contains("2 points, ");
        click(&mut harness, first, egui::PointerButton::Secondary);
        harness.get_by_label("1 points, 0 m");
        drop(harness);
        assert_eq!(store.load("mission").unwrap().len(), 1);
        let _ = std::fs::remove_dir_all(store.dir());
    }
}
