//! The "Sensor presets" section of the Devices tab (`docs/link-contract.md` section 13): one card per
//! sensor (IMU1, IMU2, BARO, MAG) with a preset picker, a radar chart, the summary, the physical
//! values and one status line. The station renders the table of `marv-presets` and computes no preset
//! logic and no score. BARO and MAG: Apply sends one `PARAM_SET` of the selector and one read by
//! name of every governed field; only when the vehicle's table then equals the crate's values is the
//! preset "Applied in RAM; not saved" (Save to flash is command 245, as on the Radio tab). The IMU
//! cards show the picker and the chart; their Apply is not built yet.

use std::time::Duration;

use eframe::egui::{
    self, Align2, Color32, FontId, Mesh, Painter, Pos2, Sense, Shape, Stroke, Vec2, vec2,
};
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::params::{ParamTable, ParamValue, WriteStatus};
use gcs_mavlink::Command;
use marv_presets::{
    Axis, Kind, MARV_DEVICE_IMU1, MARV_DEVICE_IMU2, Preset, SENSORS, Sensor, Value, preset,
};

use super::ViewCtx;
use super::actions;
use super::radio::{NOT_SAVED_TEXT, saved_since};
use crate::status::{self, GREY, RED, YELLOW};

/// Shown for a selector the list does not hold yet.
pub const NOT_LOADED_TEXT: &str = "parameters not loaded";
/// Shown after an Apply whose read-back differs from the table.
pub const MISMATCH_TEXT: &str = "the vehicle's values differ from the preset table";
/// Shown (and the picker disabled) when a known preset's fields differ from the crate's.
pub const SKEW_TEXT: &str = "station preset table does not match the firmware";
/// Why an IMU card's Apply is disabled in this build.
pub const IMU_TEXT: &str = "IMU presets: next build step";
/// The name a selector of 0 is shown as.
pub const CUSTOM_TEXT: &str = "Custom (debug)";
/// The read-back is judged when it has had this long to arrive (a read is asked 3 times, 500 ms apart).
pub const READ_BACK_WAIT: Duration = Duration::from_secs(3);

const CARD_WIDTH: f32 = 260.0;
const CHART_SIZE: Vec2 = vec2(CARD_WIDTH, 160.0);
const CHART_RADIUS: f32 = 50.0;
const LABEL_GAP: f32 = 4.0;
const ORANGE: Color32 = Color32::from_rgb(235, 160, 40);
const ORANGE_FILL: Color32 = Color32::from_rgba_premultiplied(94, 64, 16, 100);
const COMPARE: Color32 = Color32::from_rgb(120, 190, 240);
/// The chart's rings, in percent of the radius.
const RINGS: [u8; 4] = [25, 50, 75, 100];

/// The four cards, in the crate's device order.
#[derive(Default)]
pub struct SensorPresets {
    cards: [SensorPreset; 4],
}

/// What one card keeps between frames.
#[derive(Default)]
pub struct SensorPreset {
    /// A choice in the picker that was not applied.
    staged: Option<i32>,
    /// The last Apply, until a save is accepted.
    applied: Option<Applied>,
}

struct Applied {
    /// Session time of the press.
    at: Duration,
    /// The preset that was written.
    id: i32,
}

/// What the vehicle's selector says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Current {
    NotLoaded,
    /// A value the crate does not know.
    Unknown(i64),
    /// 0.
    Custom,
    Preset(i32),
}

impl Current {
    pub fn of(table: &ParamTable, sensor: &Sensor) -> Current {
        let Some(value) = table.value(sensor.selector).map(ParamValue::as_f64) else {
            return Current::NotLoaded;
        };
        let id = value.round() as i64;
        if value.fract() != 0.0 {
            Current::Unknown(id)
        } else if id == 0 {
            Current::Custom
        } else if i32::try_from(id).is_ok_and(|id| preset(sensor, id).is_some()) {
            Current::Preset(id as i32)
        } else {
            Current::Unknown(id)
        }
    }
}

impl SensorPresets {
    pub fn session_changed(&mut self) {
        *self = SensorPresets::default();
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let gate = status::command_gate(cx.access, cx.state, cx.now.is_some());
        egui::CollapsingHeader::new("Sensor presets")
            .default_open(true)
            .show(ui, |ui| {
                if let Some(reason) = gate {
                    ui.colored_label(YELLOW, format!("Apply and Save to flash are off: {reason}"));
                }
                let columns = ((ui.available_width() / (CARD_WIDTH + 24.0)) as usize)
                    .clamp(1, SENSORS.len());
                let mut cards: Vec<_> = SENSORS.iter().zip(self.cards.iter_mut()).collect();
                for row in cards.chunks_mut(columns) {
                    ui.horizontal_top(|ui| {
                        for (sensor, card) in row.iter_mut() {
                            ui.group(|ui| ui.vertical(|ui| card.ui(ui, cx, sensor, gate)));
                        }
                    });
                }
            });
    }
}

/// How an Apply stands.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Flow {
    /// The selector write is not answered yet.
    Writing,
    /// The vehicle refused the selector write; the value it uses.
    Refused(String),
    NoAnswer,
    /// The reads are out and the table does not equal the preset yet.
    Reading,
    /// The table equals the preset.
    Matches,
    /// The reads had their time and the table still differs.
    Differs,
}

/// Where an Apply that was sent at `at` stands at `now`: the selector's write, then the table against
/// the crate's preset.
pub fn apply_flow(table: &ParamTable, sensor: &Sensor, id: i32, at: Duration, now: Duration) -> Flow {
    match table.writes.get(sensor.selector) {
        Some(WriteStatus::Pending { .. }) => return Flow::Writing,
        Some(WriteStatus::Refused { in_effect, .. }) => return Flow::Refused(in_effect.to_string()),
        Some(WriteStatus::Unanswered { .. }) => return Flow::NoAnswer,
        None => {}
    }
    let selected = table.value(sensor.selector).map(ParamValue::as_f64) == Some(f64::from(id));
    if selected && fields_differ(table, sensor, id) == Some(false) {
        Flow::Matches
    } else if now.saturating_sub(at) < READ_BACK_WAIT {
        Flow::Reading
    } else {
        Flow::Differs
    }
}

/// Whether a value of the table is the crate's value (a REAL32 numerically, within its precision).
pub fn same(got: ParamValue, want: Value) -> bool {
    let got = got.as_f64();
    match want {
        Value::I32(v) => got == f64::from(v),
        Value::F32(v) => (got - f64::from(v)).abs() <= 1e-5 * f64::from(v).abs().max(1.0),
    }
}

/// Whether the table's fields differ from the crate's values of preset `id`: `None` when the preset
/// is unknown or a field is not in the table (not loaded).
pub fn fields_differ(table: &ParamTable, sensor: &Sensor, id: i32) -> Option<bool> {
    let preset = preset(sensor, id)?;
    let mut differ = false;
    for (field, want) in sensor.fields.iter().zip(preset.values) {
        differ |= !same(table.value(field)?, *want);
    }
    Some(differ)
}

impl SensorPreset {
    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx, sensor: &Sensor, gate: Option<&str>) {
        let table = &cx.state.params;
        let now = cx.now.unwrap_or_default();
        let title = sensor
            .selector
            .strip_suffix("_PRESET")
            .unwrap_or(sensor.selector);
        let imu = matches!(sensor.device, MARV_DEVICE_IMU1 | MARV_DEVICE_IMU2);
        if let Some(applied) = &self.applied
            && saved_since(cx.state, applied.at)
        {
            self.applied = None;
        }
        let current = Current::of(table, sensor);
        if let Current::Preset(id) = current
            && self.staged == Some(id)
        {
            self.staged = None;
        }
        let flow = self
            .applied
            .as_ref()
            .map(|a| apply_flow(table, sensor, a.id, a.at, now));
        let skew = flow.is_none()
            && matches!(current, Current::Preset(id) if fields_differ(table, sensor, id) == Some(true));
        let enabled = matches!(current, Current::Preset(_)) && !skew;
        let chosen = match current {
            Current::Preset(id) => Some(self.staged.unwrap_or(id)),
            _ => None,
        };
        let chosen_preset = chosen.and_then(|id| preset(sensor, id));

        ui.set_width(CARD_WIDTH);
        let named = ui.strong(title);
        let shown = match current {
            Current::NotLoaded => "—".to_string(),
            Current::Custom => CUSTOM_TEXT.to_string(),
            Current::Unknown(n) => format!("unknown ({n})"),
            Current::Preset(_) => chosen_preset.map_or_else(String::new, |p| p.name.to_string()),
        };
        let mut hovered = None;
        let mut pick = None;
        let combo = ui.add_enabled_ui(enabled, |ui| {
            egui::ComboBox::from_id_salt(sensor.selector)
                .selected_text(shown)
                .width(CARD_WIDTH - 24.0)
                .show_ui(ui, |ui| {
                    for p in sensor.presets {
                        let response = ui.selectable_label(chosen == Some(p.id), p.name);
                        if response.hovered() {
                            hovered = Some(p.id);
                        }
                        if response.clicked() {
                            pick = Some(p.id);
                        }
                    }
                })
        });
        combo.inner.response.labelled_by(named.id);
        if let Some(id) = pick {
            self.staged = Some(id);
            if matches!(
                flow,
                Some(Flow::Refused(_) | Flow::NoAnswer | Flow::Differs)
            ) {
                self.applied = None;
            }
        }
        draw_chart(
            ui,
            sensor,
            chosen_preset,
            hovered.and_then(|id| preset(sensor, id)),
        );
        if let Some(p) = chosen_preset {
            ui.add(egui::Label::new(egui::RichText::new(p.summary).italics()).wrap());
            for (i, axis) in sensor.axes.iter().enumerate() {
                ui.small(axis_line(axis, p, i));
            }
        }

        let flow = self.applied.as_ref().and(flow);
        let (text, colour) = status_line(current, skew, flow.as_ref(), self.staged, sensor);
        ui.add(egui::Label::new(egui::RichText::new(text).color(colour)).wrap());
        if matches!(flow, Some(Flow::Writing | Flow::Reading)) {
            ui.ctx().request_repaint_after(Duration::from_millis(250));
        }
        if matches!(flow, Some(Flow::Matches)) {
            if actions::button(ui, "Save to flash", gate) {
                cx.commands.push(Command::SendCommand {
                    command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
                    params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                });
            }
        }
        if enabled {
            let reason = if imu { Some(IMU_TEXT) } else { gate };
            ui.horizontal(|ui| {
                if actions::button(ui, "Apply", reason)
                    && let Some(id) = chosen
                {
                    cx.commands.push(Command::SetParam {
                        name: sensor.selector.to_owned(),
                        value: ParamValue::I32(id),
                    });
                    for field in sensor.fields {
                        cx.commands.push(Command::ReadParam {
                            name: (*field).to_owned(),
                        });
                    }
                    self.applied = Some(Applied { at: now, id });
                }
                if imu {
                    ui.colored_label(YELLOW, IMU_TEXT);
                }
            });
        }
    }
}

/// The one status line of a card and its colour.
pub fn status_line(
    current: Current,
    skew: bool,
    flow: Option<&Flow>,
    staged: Option<i32>,
    sensor: &Sensor,
) -> (String, Color32) {
    if let Some(flow) = flow {
        return match flow {
            Flow::Writing => ("applying…".to_string(), GREY),
            Flow::Refused(in_effect) => (format!("refused: vehicle uses {in_effect}"), RED),
            Flow::NoAnswer => ("no answer".to_string(), RED),
            Flow::Reading => ("reading back…".to_string(), GREY),
            Flow::Matches => (NOT_SAVED_TEXT.to_string(), YELLOW),
            Flow::Differs => (MISMATCH_TEXT.to_string(), RED),
        };
    }
    match current {
        Current::NotLoaded => (NOT_LOADED_TEXT.to_string(), GREY),
        Current::Unknown(n) => (
            format!("unknown preset {n} (station older than firmware)"),
            RED,
        ),
        _ if skew => (SKEW_TEXT.to_string(), RED),
        Current::Custom => (
            "Custom (debug): the fields are set one by one".to_string(),
            YELLOW,
        ),
        Current::Preset(id) => match staged.and_then(|s| preset(sensor, s)) {
            Some(p) => (format!("{}: not applied", p.name), GREY),
            None => (
                format!(
                    "on the vehicle: {}",
                    preset(sensor, id).map_or("", |p| p.name)
                ),
                GREY,
            ),
        },
    }
}

/// A physical value as it is shown: three significant digits at most, no trailing zeros.
pub fn number_text(v: f32) -> String {
    let a = v.abs();
    let text = if a >= 100.0 {
        format!("{v:.0}")
    } else if a >= 10.0 {
        format!("{v:.1}")
    } else if a >= 1.0 {
        format!("{v:.2}")
    } else {
        format!("{v:.3}")
    };
    if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    }
}

/// One axis of a preset in words: its value with the unit, the second value, and the model / bound tag.
pub fn axis_line(axis: &Axis, preset: &Preset, i: usize) -> String {
    let mut text = format!(
        "{}: {} {}",
        axis.name,
        preset.physical.get(i).map_or_else(String::new, |v| number_text(*v)),
        axis.unit
    );
    if let (Some(unit2), Some(Some(v2))) = (axis.unit2, preset.physical2.get(i)) {
        text.push_str(&format!(", {} {unit2}", number_text(*v2)));
    }
    match axis.kind {
        Kind::Plain => {}
        Kind::Model => text.push_str(" (model)"),
        Kind::Bound => text.push_str(" (bound)"),
    }
    text
}

/// The unit direction of spoke `i` of `n`: the first at the top, then clockwise on the screen.
pub fn spoke(i: usize, n: usize) -> Vec2 {
    let angle = -std::f32::consts::FRAC_PI_2 + std::f32::consts::TAU * i as f32 / n as f32;
    vec2(angle.cos(), angle.sin())
}

/// The polygon of a score set: vertex `i` on spoke `i` at `score / 100` of `radius` from `centre`.
pub fn radar_vertices(scores: &[u8], centre: Pos2, radius: f32) -> Vec<Pos2> {
    scores
        .iter()
        .enumerate()
        .map(|(i, s)| centre + spoke(i, scores.len()) * (radius * f32::from(*s) / 100.0))
        .collect()
}

/// The fill of a radar polygon: a fan of one triangle per edge from `centre`. The polygon of a score
/// set is star-shaped from the centre but not convex in general, so `Shape::convex_polygon` would
/// fill the wrong area.
pub fn fill_shape(vertices: &[Pos2], centre: Pos2, colour: Color32) -> Shape {
    let mut mesh = Mesh::default();
    mesh.colored_vertex(centre, colour);
    for v in vertices {
        mesh.colored_vertex(*v, colour);
    }
    let n = vertices.len() as u32;
    for i in 0..n {
        mesh.add_triangle(0, 1 + i, 1 + (i + 1) % n);
    }
    Shape::mesh(mesh)
}

fn draw_chart(ui: &mut egui::Ui, sensor: &Sensor, selected: Option<&Preset>, hovered: Option<&Preset>) {
    let (response, painter) = ui.allocate_painter(CHART_SIZE, Sense::hover());
    let centre = response.rect.center();
    let n = sensor.axes.len();
    let ring_colour = ui.visuals().widgets.noninteractive.bg_stroke.color;
    let text_colour = ui.visuals().text_color();
    for ring in RINGS {
        painter.add(Shape::closed_line(
            radar_vertices(&vec![ring; n], centre, CHART_RADIUS),
            Stroke::new(1.0, ring_colour),
        ));
    }
    for (i, axis) in sensor.axes.iter().enumerate() {
        let dir = spoke(i, n);
        painter.line_segment(
            [centre, centre + dir * CHART_RADIUS],
            Stroke::new(1.0, ring_colour),
        );
        let anchor = if dir.x > 0.3 {
            Align2::LEFT_CENTER
        } else if dir.x < -0.3 {
            Align2::RIGHT_CENTER
        } else if dir.y < 0.0 {
            Align2::CENTER_BOTTOM
        } else {
            Align2::CENTER_TOP
        };
        painter.text(
            centre + dir * (CHART_RADIUS + LABEL_GAP),
            anchor,
            axis.name,
            FontId::proportional(10.0),
            text_colour,
        );
    }
    paint_polygon(&painter, centre, selected, Some((ORANGE_FILL, ORANGE)), 2.0);
    if let Some(p) = hovered
        && selected.is_none_or(|s| s.id != p.id)
    {
        paint_polygon(&painter, centre, Some(p), None, 1.5);
    }
}

fn paint_polygon(
    painter: &Painter,
    centre: Pos2,
    preset: Option<&Preset>,
    fill: Option<(Color32, Color32)>,
    width: f32,
) {
    let Some(p) = preset else { return };
    let vertices = radar_vertices(p.scores, centre, CHART_RADIUS);
    let outline = match fill {
        Some((fill, outline)) => {
            painter.add(fill_shape(&vertices, centre, fill));
            outline
        }
        None => COMPARE,
    };
    painter.add(Shape::closed_line(vertices, Stroke::new(width, outline)));
}
