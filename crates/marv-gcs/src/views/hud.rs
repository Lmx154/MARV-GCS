//! The side panel: a flight instrument at the top left of the window, on every tab, and the vehicle's
//! status sections under it ([`super::sidebar`]). It shows what the vehicle sends and computes nothing:
//! ATTITUDE (roll, pitch, yaw) for the horizon and the heading, GPS_RAW_INT for ground speed, altitude and
//! course over ground. Only units are converted. It draws from the state and sends nothing, so it takes no
//! command list. The drawing is a square as wide as the panel; every length in it is a share of its side.

use std::time::Duration;

use eframe::egui::{
    self, Align2, Color32, FontId, Label, Pos2, Rect, RichText, Role, Sense, Shape, Stroke, Vec2,
    WidgetInfo, pos2, vec2,
};
use gcs_mavlink::services::messages::{MessageEntry, MessageStore};
use gcs_mavlink::{LinkStatus, VehicleState};

use super::sidebar;
use crate::status::GREY;

/// The side panel's width when the window opens, and the range the user can drag it in (logical px).
pub const PANEL_DEFAULT_WIDTH: f32 = 320.0;
pub const PANEL_MIN_WIDTH: f32 = 240.0;
pub const PANEL_MAX_WIDTH: f32 = 480.0;
/// The side the lengths below were drawn for; a drawing of side `s` scales them by `s / DESIGN_SIDE`.
const DESIGN_SIDE: f32 = 240.0;
/// An ATTITUDE older than this is not shown.
pub(super) const ATTITUDE_STALE: Duration = Duration::from_secs(2);
/// A GPS_RAW_INT older than this is not shown.
pub(super) const GPS_STALE: Duration = Duration::from_secs(3);
/// GPS_RAW_INT `vel` and `cog` when unknown.
const UNKNOWN_U16: f64 = 65_535.0;
/// GPS_RAW_INT `satellites_visible` when unknown.
const UNKNOWN_SATELLITES: f64 = 255.0;
/// The heading strip shows this many degrees on each side of the heading.
const HEADING_SPAN: f32 = 45.0;
/// Height of the heading strip, at [`DESIGN_SIDE`].
const STRIP_HEIGHT: f32 = 22.0;
/// The roll arc covers -60 to +60 degrees.
const ARC_LIMIT: f32 = 60.0;
const ARC_TICKS: [f32; 5] = [10.0, 20.0, 30.0, 45.0, 60.0];

/// What the latest ATTITUDE says, in the message's units (rad).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    /// Positive: right wing down.
    pub roll: f64,
    /// Positive: nose up.
    pub pitch: f64,
    /// 0 to 2 pi or -pi to pi, as sent; north is 0.
    pub yaw: f64,
}

/// The fresh values, in the units the vehicle sends.
pub(super) struct Data {
    pub(super) attitude: Option<Attitude>,
    /// m/s.
    speed: Option<f64>,
    /// m, above mean sea level, from GPS_RAW_INT.
    altitude: Option<f64>,
    /// Degrees.
    course: Option<f64>,
    pub(super) fix: Option<String>,
    pub(super) satellites: Option<u32>,
}

pub(super) fn fresh<'a>(
    store: &'a MessageStore,
    name: &str,
    now: Option<Duration>,
    limit: Duration,
) -> Option<&'a MessageEntry> {
    let entry = store.entry(name, 0)?;
    let age = now?.saturating_sub(entry.last_seen);
    (age <= limit).then_some(entry)
}

pub(super) fn data(store: &MessageStore, now: Option<Duration>) -> Data {
    let attitude = fresh(store, "ATTITUDE", now, ATTITUDE_STALE).and_then(|e| {
        Some(Attitude {
            roll: e.number("roll")?,
            pitch: e.number("pitch")?,
            yaw: e.number("yaw")?,
        })
    });
    let gps = fresh(store, "GPS_RAW_INT", now, GPS_STALE);
    let known = |field: &str| {
        gps?.number(field)
            .filter(|v| *v != UNKNOWN_U16 && v.is_finite())
    };
    Data {
        attitude,
        speed: known("vel").map(|v| v / 100.0),
        altitude: gps
            .and_then(|g| g.number("alt"))
            .filter(|v| v.is_finite())
            .map(|v| v / 1000.0),
        course: known("cog").map(|v| v / 100.0),
        fix: gps.and_then(|g| match g.text("fix_type") {
            Some(text) => Some(fix_text(text)),
            None => g.number("fix_type").map(|n| format!("{n}")),
        }),
        satellites: gps
            .and_then(|g| g.number("satellites_visible"))
            .filter(|v| *v != UNKNOWN_SATELLITES)
            .map(|v| v as u32),
    }
}

/// `GPS_FIX_TYPE_3D_FIX` as `3D FIX`.
fn fix_text(name: &str) -> String {
    name.strip_prefix("GPS_FIX_TYPE_")
        .unwrap_or(name)
        .replace('_', " ")
}

/// A value with one decimal and no "-0.0".
pub(super) fn one_decimal(v: f64) -> String {
    format!("{:.1}", (v * 10.0).round() / 10.0 + 0.0)
}

/// Degrees in 0 to 359.
pub(super) fn heading_deg(yaw: f64) -> u32 {
    (yaw.to_degrees().round().rem_euclid(360.0) as u32) % 360
}

/// The two end points of the horizon line in `rect`, the line the sky and the ground meet at. The
/// aircraft is fixed at the centre; the scale is [`px_per_deg`]. The line is rotated by `roll` and
/// shifted by `pitch` along the direction toward the ground, `(sin roll, cos roll)` on a screen whose
/// y axis points down: right wing down (roll > 0) lifts the right end, nose up (pitch > 0) lowers the
/// line. The ends are one side length from the line's centre, which covers the square.
pub fn horizon_line(rect: Rect, roll: f32, pitch: f32) -> [Pos2; 2] {
    let side = rect.width();
    let along = vec2(roll.cos(), -roll.sin());
    let centre = rect.center() + ground_direction(roll) * (px_per_deg(side) * pitch.to_degrees());
    [centre - along * side, centre + along * side]
}

/// Unit vector toward the ground on the screen.
fn ground_direction(roll: f32) -> Vec2 {
    vec2(roll.sin(), roll.cos())
}

/// Pixels per degree of pitch: the square shows 80 degrees.
fn px_per_deg(side: f32) -> f32 {
    side / 80.0
}

pub(super) struct Colours {
    sky: Color32,
    ground: Color32,
    line: Color32,
    symbol: Color32,
    text: Color32,
}

impl Colours {
    pub(super) fn live() -> Self {
        Colours {
            sky: Color32::from_rgb(70, 130, 200),
            ground: Color32::from_rgb(130, 90, 50),
            line: Color32::WHITE,
            symbol: Color32::from_rgb(255, 200, 0),
            text: Color32::WHITE,
        }
    }

    pub(super) fn grey() -> Self {
        Colours {
            sky: Color32::from_gray(105),
            ground: Color32::from_gray(75),
            line: Color32::from_gray(190),
            symbol: Color32::from_gray(200),
            text: Color32::from_gray(210),
        }
    }
}

/// The accessible name of the drawing.
pub const DRAWING_NAME: &str = "HUD drawing";

/// A label inside `area`, over the drawing. It does not move the layout's cursor (`Ui::put` would, and the
/// lines under the drawing would start inside it).
fn place(ui: &mut egui::Ui, area: Rect, label: Label) {
    ui.new_child(egui::UiBuilder::new().max_rect(area).layout(
        egui::Layout::centered_and_justified(egui::Direction::TopDown),
    ))
    .add(label);
}

/// The scale of a drawing of side `side` against [`DESIGN_SIDE`].
fn scale(side: f32) -> f32 {
    side / DESIGN_SIDE
}

/// The side panel: the drawing, the numbers under it, then the status sections, scrolling when they
/// do not fit.
pub fn ui(ui: &mut egui::Ui, state: &VehicleState, now: Option<Duration>) {
    let max_side = ui.available_height() * MAX_HEIGHT_SHARE;
    egui::ScrollArea::vertical()
        .id_salt("side_panel")
        .auto_shrink([false, false])
        .show(ui, |ui| instrument(ui, state, now, max_side));
}

/// The drawing takes at most this share of the panel's height, so the sections under it stay in view.
pub const MAX_HEIGHT_SHARE: f32 = 0.4;

fn instrument(ui: &mut egui::Ui, state: &VehicleState, now: Option<Duration>, max_side: f32) {
    let connected = now.is_some() && state.link.status == LinkStatus::Connected;
    let data = data(&state.messages, now);
    let colours = if connected {
        Colours::live()
    } else {
        Colours::grey()
    };
    let side = ui.available_width().min(max_side).max(1.0);
    let k = scale(side);
    let (rect, response) = ui
        .vertical_centered(|ui| ui.allocate_exact_size(vec2(side, side), Sense::hover()))
        .inner;
    response.widget_info(|| WidgetInfo::labeled(Role::Image, true, DRAWING_NAME));
    let painter = ui.painter_at(rect);
    // Without an attitude the level horizon is grey, so it cannot be read as a real "level".
    let grey = Colours::grey();
    let horizon = if data.attitude.is_some() {
        &colours
    } else {
        &grey
    };
    draw_horizon(&painter, rect, horizon, data.attitude);
    draw_arc(&painter, rect, &colours, data.attitude);
    draw_strip(
        &painter,
        rect,
        &colours,
        data.attitude.map(|a| a.yaw),
        data.course,
    );
    draw_aircraft(&painter, rect, &colours);
    draw_boxes(ui, &painter, rect, &colours, &data);
    if data.attitude.is_none() {
        place(
            ui,
            Rect::from_center_size(
                rect.center() + vec2(0.0, 0.2 * side),
                vec2(side - 20.0 * k, 20.0 * k),
            ),
            Label::new(
                RichText::new("no attitude from the vehicle")
                    .size(font(13.0, k))
                    .color(colours.text),
            ),
        );
    }

    let text = |ui: &mut egui::Ui, s: String| {
        let colour = if connected {
            ui.visuals().text_color()
        } else {
            GREY
        };
        ui.label(RichText::new(s).color(colour));
    };
    ui.add_space(4.0);
    let a = data.attitude;
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing.x = 10.0 * k.max(1.0);
        text(
            ui,
            format!(
                "Roll {}°",
                a.map_or("--".into(), |a| one_decimal(a.roll.to_degrees()))
            ),
        );
        text(
            ui,
            format!(
                "Pitch {}°",
                a.map_or("--".into(), |a| one_decimal(a.pitch.to_degrees()))
            ),
        );
        text(
            ui,
            format!(
                "Heading {}",
                a.map_or("--".into(), |a| format!("{}°", heading_deg(a.yaw)))
            ),
        );
    });
    sidebar::ui(ui, state, now);
}

/// A font size of `design` points at scale `k`, never below 8.
fn font(design: f32, k: f32) -> f32 {
    (design * k).max(8.0)
}

/// A line width of `design` px at scale `k`, never below 1.
fn width(design: f32, k: f32) -> f32 {
    (design * k).max(1.0)
}

pub(super) fn draw_horizon(
    painter: &egui::Painter,
    rect: Rect,
    colours: &Colours,
    attitude: Option<Attitude>,
) {
    let (roll, pitch) = attitude.map_or((0.0, 0.0), |a| {
        (a.roll as f32, (a.pitch as f32).clamp(-1.57, 1.57))
    });
    let [a, b] = horizon_line(rect, roll, pitch);
    let down = ground_direction(roll);
    painter.rect_filled(rect, 0.0, colours.sky);
    let reach = down * (4.0 * rect.width());
    painter.add(Shape::convex_polygon(
        vec![a, b, b + reach, a + reach],
        colours.ground,
        Stroke::NONE,
    ));
    let side = rect.width();
    let k = scale(side);
    painter.line_segment([a, b], Stroke::new(width(2.0, k), colours.line));
    if attitude.is_none() {
        return;
    }
    let along = vec2(roll.cos(), -roll.sin());
    let centre = (a.to_vec2() + b.to_vec2()) / 2.0;
    for step in (-18..=18).filter(|s| *s != 0) {
        let degrees = (step * 5) as f32;
        let at = centre.to_pos2() - down * (px_per_deg(side) * degrees);
        if (at - rect.center()).length() > 0.36 * side {
            continue;
        }
        let long = step % 2 == 0;
        let half = side * if long { 0.12 } else { 0.06 };
        painter.line_segment(
            [at - along * half, at + along * half],
            Stroke::new(width(1.5, k), colours.line),
        );
        if long {
            let label = format!("{}", degrees as i32);
            let font = FontId::proportional(font(10.0, k));
            painter.text(
                at - along * (half + 3.0 * k),
                Align2::RIGHT_CENTER,
                &label,
                font.clone(),
                colours.text,
            );
            painter.text(
                at + along * (half + 3.0 * k),
                Align2::LEFT_CENTER,
                &label,
                font,
                colours.text,
            );
        }
    }
}

/// The point on a circle of radius `r` around `centre`, `degrees` clockwise from straight up.
fn on_circle(centre: Pos2, r: f32, degrees: f32) -> Pos2 {
    let rad = degrees.to_radians();
    centre + vec2(rad.sin(), -rad.cos()) * r
}

fn draw_arc(painter: &egui::Painter, rect: Rect, colours: &Colours, attitude: Option<Attitude>) {
    let centre = rect.center();
    let radius = 0.40 * rect.width();
    let k = scale(rect.width());
    let stroke = Stroke::new(width(1.5, k), colours.line);
    let points: Vec<Pos2> = (-12..=12)
        .map(|i| on_circle(centre, radius, i as f32 * 5.0))
        .collect();
    painter.add(Shape::line(points, stroke));
    for degrees in [0.0]
        .into_iter()
        .chain(ARC_TICKS.into_iter().flat_map(|t| [t, -t]))
    {
        let length = k * if degrees == 0.0 { 10.0 } else { 6.0 };
        painter.line_segment(
            [
                on_circle(centre, radius, degrees),
                on_circle(centre, radius - length, degrees),
            ],
            stroke,
        );
    }
    if let Some(attitude) = attitude {
        // The world's vertical turns the other way from the roll: right wing down, the pointer goes left.
        let at = (-(attitude.roll.to_degrees() as f32)).clamp(-ARC_LIMIT, ARC_LIMIT);
        painter.add(Shape::convex_polygon(
            vec![
                on_circle(centre, radius + k, at),
                on_circle(centre, radius + 11.0 * k, at - 4.0),
                on_circle(centre, radius + 11.0 * k, at + 4.0),
            ],
            colours.symbol,
            Stroke::NONE,
        ));
    }
}

/// The signed angle from `from` to `to` in degrees, -180 to 180.
fn wrap_180(degrees: f32) -> f32 {
    (degrees + 180.0).rem_euclid(360.0) - 180.0
}

fn draw_strip(
    painter: &egui::Painter,
    rect: Rect,
    colours: &Colours,
    yaw: Option<f64>,
    course: Option<f64>,
) {
    let k = scale(rect.width());
    let strip = Rect::from_min_size(rect.min, vec2(rect.width(), STRIP_HEIGHT * k));
    painter.rect_filled(strip, 0.0, Color32::from_black_alpha(150));
    let Some(yaw) = yaw else {
        return;
    };
    let yaw = yaw.to_degrees() as f32;
    let per_deg = rect.width() / (2.0 * HEADING_SPAN);
    let x_of = |degrees: f32| rect.center().x + wrap_180(degrees - yaw) * per_deg;
    for tick in 0..36 {
        let degrees = tick as f32 * 10.0;
        if wrap_180(degrees - yaw).abs() > HEADING_SPAN {
            continue;
        }
        let x = x_of(degrees);
        let cardinal = match tick {
            0 => Some("N"),
            9 => Some("E"),
            18 => Some("S"),
            27 => Some("W"),
            _ => None,
        };
        let length = k * if cardinal.is_some() { 4.0 } else { 7.0 };
        painter.line_segment(
            [
                pos2(x, strip.bottom()),
                pos2(x, strip.bottom() - length - 2.0 * k),
            ],
            Stroke::new(width(1.0, k), colours.line),
        );
        if let Some(letter) = cardinal {
            painter.text(
                pos2(x, strip.top() + 8.0 * k),
                Align2::CENTER_CENTER,
                letter,
                FontId::proportional(font(12.0, k)),
                colours.text,
            );
        }
    }
    painter.add(Shape::convex_polygon(
        vec![
            pos2(rect.center().x, strip.bottom()),
            pos2(rect.center().x - 4.0 * k, strip.bottom() + 6.0 * k),
            pos2(rect.center().x + 4.0 * k, strip.bottom() + 6.0 * k),
        ],
        colours.symbol,
        Stroke::NONE,
    ));
    if let Some(course) = course {
        let offset = wrap_180(course as f32 - yaw);
        if offset.abs() <= HEADING_SPAN {
            let x = x_of(course as f32);
            painter.add(Shape::convex_polygon(
                vec![
                    pos2(x, strip.top() + 8.0 * k),
                    pos2(x - 4.0 * k, strip.top()),
                    pos2(x + 4.0 * k, strip.top()),
                ],
                Color32::from_rgb(230, 80, 230),
                Stroke::NONE,
            ));
        }
    }
}

pub(super) fn draw_aircraft(painter: &egui::Painter, rect: Rect, colours: &Colours) {
    let c = rect.center();
    let k = scale(rect.width());
    let stroke = Stroke::new(width(3.0, k), colours.symbol);
    for sign in [-1.0, 1.0] {
        painter.line_segment(
            [
                c + vec2(sign * 44.0 * k, 0.0),
                c + vec2(sign * 16.0 * k, 0.0),
            ],
            stroke,
        );
        painter.line_segment(
            [
                c + vec2(sign * 16.0 * k, 0.0),
                c + vec2(sign * 16.0 * k, 7.0 * k),
            ],
            stroke,
        );
    }
    painter.circle_filled(c, 2.5 * k, colours.symbol);
}

fn draw_boxes(
    ui: &mut egui::Ui,
    painter: &egui::Painter,
    rect: Rect,
    colours: &Colours,
    data: &Data,
) {
    let k = scale(rect.width());
    let height = 26.0 * k;
    let boxes = [
        (
            Rect::from_min_size(
                pos2(rect.left() + 2.0 * k, rect.center().y - height / 2.0),
                vec2(66.0 * k, height),
            ),
            "ground speed",
            data.speed
                .map_or("--".into(), |v| format!("{} m/s", one_decimal(v))),
        ),
        (
            Rect::from_min_size(
                pos2(
                    rect.right() - 2.0 * k - 78.0 * k,
                    rect.center().y - height / 2.0,
                ),
                vec2(78.0 * k, height),
            ),
            "GPS alt MSL",
            data.altitude
                .map_or("--".into(), |v| format!("{} m", one_decimal(v))),
        ),
    ];
    for (area, caption, value) in boxes {
        painter.rect_filled(area, 2.0 * k, Color32::from_rgb(20, 20, 20));
        painter.rect_stroke(
            area,
            2.0 * k,
            Stroke::new(width(1.0, k), colours.line),
            egui::StrokeKind::Inside,
        );
        painter.text(
            pos2(area.center().x, area.top() - 2.0 * k),
            Align2::CENTER_BOTTOM,
            caption,
            FontId::proportional(font(9.0, k)),
            colours.text,
        );
        place(
            ui,
            area,
            Label::new(RichText::new(value).size(font(13.0, k)).color(colours.text))
                .wrap_mode(egui::TextWrapMode::Extend),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQUARE: f32 = 240.0;

    fn rect() -> Rect {
        Rect::from_min_size(pos2(0.0, 0.0), vec2(SQUARE, SQUARE))
    }

    fn near(p: Pos2, x: f32, y: f32) {
        assert!(
            (p.x - x).abs() < 0.01 && (p.y - y).abs() < 0.01,
            "{p:?} is not ({x}, {y})"
        );
    }

    /// The square is 240 px, its centre (120, 120), the scale 240 / 80 = 3 px per degree, and the line's
    /// ends are 240 px from its centre along `(cos roll, -sin roll)`.
    ///
    /// - roll 0, pitch 0: centre (120, 120), along (1, 0): ends (-120, 120) and (360, 120).
    /// - roll 30, pitch 0: along (0.8660, -0.5), 240 along is (207.846, -120): ends (-87.846, 240) and
    ///   (327.846, 0). The right end is higher (smaller y): right wing down lifts the right end.
    /// - roll 0, pitch 10: the line moves 10 * 3 = 30 px toward the ground (down): centre (120, 150),
    ///   ends (-120, 150) and (360, 150). Nose up lowers the horizon.
    #[test]
    fn horizon_line_for_level_rolled_and_pitched() {
        let [a, b] = horizon_line(rect(), 0.0, 0.0);
        near(a, -120.0, 120.0);
        near(b, 360.0, 120.0);
        let [a, b] = horizon_line(rect(), 30.0_f32.to_radians(), 0.0);
        near(a, -87.846, 240.0);
        near(b, 327.846, 0.0);
        let [a, b] = horizon_line(rect(), 0.0, 10.0_f32.to_radians());
        near(a, -120.0, 150.0);
        near(b, 360.0, 150.0);
    }

    #[test]
    fn heading_wraps_into_0_to_359() {
        assert_eq!(heading_deg(90.0_f64.to_radians()), 90);
        assert_eq!(heading_deg(-90.0_f64.to_radians()), 270);
        assert_eq!(heading_deg(359.7_f64.to_radians()), 0);
        assert_eq!(heading_deg(0.0), 0);
    }

    #[test]
    fn a_zero_is_never_negative() {
        assert_eq!(one_decimal(-0.01), "0.0");
        assert_eq!(one_decimal(-10.04), "-10.0");
    }
}
