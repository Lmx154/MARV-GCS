//! The "Sensor presets" section of the Devices tab (`docs/link-contract.md` section 13): one card per
//! sensor (IMU1, IMU2, BARO, MAG) with a preset picker, a radar chart, the summary, the physical
//! values and one status line. A preset is a station feature: the table is `gcs_mavlink::presets` and
//! the vehicle knows no preset. The active preset is the one whose field values all equal the vehicle's,
//! else "Custom". Apply writes the preset's fields as ordinary `PARAM_SET` writes, every field whose
//! value goes down first, then every field whose value goes up (see [`write_plan`]); only when every
//! write's echo has come back and the vehicle's values equal the table is the preset "Applied in RAM;
//! not saved" (Save to flash is command 245, as on the Radio tab). The IMUs apply after a reboot: the
//! same writes, then "Pending" and one "Save and reboot" (245, and only when it is ACCEPTED, 246). The
//! debug switch of the section header offers "Custom (debug)" in the picker, which shows the fields as
//! values to type; it is a gate of this window only.

use std::cell::Cell;
use std::collections::BTreeMap;
use std::rc::Rc;
use std::time::Duration;

use eframe::egui::{
    self, Align2, Color32, FontId, Mesh, Painter, Pos2, Rect, Sense, Shape, Stroke, Vec2, vec2,
};
use gcs_mavlink::Command;
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::presets::{
    Axis, Kind, MARV_DEVICE_IMU1, MARV_DEVICE_IMU2, Preset, SENSORS, Sensor, Value, preset,
};
use gcs_mavlink::services::params::{ParamTable, ParamValue, WriteStatus};
use gcs_mavlink::{CommandStatus, VehicleState};

use super::ViewCtx;
use super::actions::{self, Confirm};
use super::radio::{NOT_SAVED_TEXT, saved_since};
use crate::status::{self, GREY, RED, YELLOW};
use crate::theme;

/// Shown for a sensor whose fields the list does not hold yet.
pub const NOT_LOADED_TEXT: &str = "parameters not loaded";
/// Shown after an Apply whose writes all had their time and the vehicle's values still differ from the table.
pub const MISMATCH_TEXT: &str = "the vehicle's values differ from the preset table";
/// An IMU preset that is written and read back and takes effect after the next boot.
pub const PENDING_TEXT: &str = "Pending: takes effect after Save and reboot";
/// The label of the one button of a pending IMU card.
pub const SAVE_AND_REBOOT: &str = "Save and reboot";
/// The label of the debug switch in the section header.
pub const DEBUG_TEXT: &str = "Debug: raw sensor settings";
/// On the Parameters tab, beside a field a preset sets while the debug switch is off.
pub const SET_IN_DEVICES_TEXT: &str = "set in Devices > Sensor presets";
/// Beside an IMU field that is editable: the chip is configured once, at boot.
pub const AFTER_REBOOT_TEXT: &str = "applies after reboot";
/// The save of "Save and reboot" is waited for this long to appear as the vehicle's last command.
const SAVE_WAIT: Duration = Duration::from_secs(15);

/// The station-wide debug switch: off at every start, never saved. The Devices tab shows it and the
/// Parameters tab reads it. It is a gate of this window only; the vehicle knows nothing of it.
pub type DebugFlag = Rc<Cell<bool>>;

/// Whether `name` is one of the fields a preset sets.
pub fn is_governed(name: &str) -> bool {
    SENSORS.iter().any(|s| s.fields.contains(&name))
}

/// Whether a write of the governed parameter `name` takes effect only after a reboot (the IMUs).
pub fn applies_after_reboot(name: &str) -> bool {
    SENSORS
        .iter()
        .any(|s| is_imu(s) && s.fields.contains(&name))
}

fn is_imu(sensor: &Sensor) -> bool {
    matches!(sensor.device, MARV_DEVICE_IMU1 | MARV_DEVICE_IMU2)
}

/// The sensor's name on its card.
pub fn sensor_name(sensor: &Sensor) -> &'static str {
    match sensor.device {
        MARV_DEVICE_IMU1 => "IMU1",
        MARV_DEVICE_IMU2 => "IMU2",
        3 => "BARO",
        _ => "MAG",
    }
}

/// The entry of the picker that shows the fields as values to type (debug switch on).
pub const CUSTOM_TEXT: &str = "Custom (debug)";
/// What a card shows when the vehicle's values equal no preset.
pub const CUSTOM_ACTIVE_TEXT: &str = "Custom";
/// An Apply is judged when its writes have had this long to be answered (a write is sent 3 times,
/// 500 ms apart).
pub const WRITE_WAIT: Duration = Duration::from_secs(3);

const CARD_WIDTH: f32 = 260.0;
pub const CHART_SIZE: Vec2 = vec2(CARD_WIDTH, 180.0);
const CHART_RADIUS: f32 = 70.0;
const LABEL_GAP: f32 = 4.0;
const MIN_CHART_RADIUS: f32 = 30.0;
const ORANGE: Color32 = theme::WARN;
const ORANGE_FILL: Color32 = Color32::from_rgba_premultiplied(94, 72, 25, 100);
const COMPARE: Color32 = theme::ACCENT;
/// The chart's rings, in percent of the radius.
const RINGS: [u8; 4] = [25, 50, 75, 100];

/// The four cards, in the crate's device order.
pub struct SensorPresets {
    cards: [SensorPreset; 4],
    debug: DebugFlag,
}

/// What one card keeps between frames.
#[derive(Default)]
pub struct SensorPreset {
    /// A choice in the picker that was not applied (0 is Custom).
    staged: Option<i32>,
    /// The last Apply, until a save is accepted (an IMU: until the session ends).
    applied: Option<Applied>,
    /// The second click of "Save and reboot".
    confirm: Confirm,
    /// The typed text of the fields of a vehicle in Custom, by name.
    edits: BTreeMap<String, String>,
}

struct Applied {
    /// Session time of the press.
    at: Duration,
    /// The preset that was written.
    id: i32,
    /// The fields that were written, in the order they were sent.
    written: Vec<String>,
    /// An IMU preset: how its save and reboot stands.
    reboot: Reboot,
}

/// How "Save and reboot" of a pending IMU preset stands.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reboot {
    /// Not pressed.
    Idle,
    /// 245 was sent at this session time and is not ACCEPTED yet.
    Saving(Duration),
    /// The save was ACCEPTED and 246 was sent at this session time.
    Rebooting(Duration),
    /// The save or the reboot did not succeed; nothing more was sent.
    Failed(String),
}

/// Which preset the vehicle's values are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Current {
    /// The list does not hold every field yet.
    NotLoaded,
    /// The values equal no preset.
    Custom,
    Preset(i32),
}

impl Current {
    /// The preset of `sensor` whose field values all equal the table's, else `Custom`.
    pub fn of(table: &ParamTable, sensor: &Sensor) -> Current {
        if sensor.fields.iter().any(|f| table.value(f).is_none()) {
            return Current::NotLoaded;
        }
        sensor
            .presets
            .iter()
            .find(|p| fields_differ(table, sensor, p.id) == Some(false))
            .map_or(Current::Custom, |p| Current::Preset(p.id))
    }
}

impl SensorPresets {
    /// The cards, with the station's debug switch (shared with the Parameters tab).
    pub fn new(debug: DebugFlag) -> SensorPresets {
        SensorPresets {
            cards: Default::default(),
            debug,
        }
    }

    pub fn session_changed(&mut self) {
        self.cards = Default::default();
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let gate = status::command_gate(cx.access, cx.state, cx.now.is_some());
        let id = ui.make_persistent_id("sensor_presets");
        let debug = self.debug.get();
        let mut switch = debug;
        egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true)
            .show_header(ui, |ui| {
                ui.label(theme::title_text("Sensor presets"));
                ui.checkbox(&mut switch, DEBUG_TEXT);
            })
            .body(|ui| {
                if let Some(reason) = gate {
                    ui.colored_label(
                        YELLOW,
                        format!("Apply, Save to flash and Save and reboot are off: {reason}"),
                    );
                }
                let columns =
                    ((ui.available_width() / (CARD_WIDTH + 24.0)) as usize).clamp(1, SENSORS.len());
                let mut cards: Vec<_> = SENSORS.iter().zip(self.cards.iter_mut()).collect();
                for row in cards.chunks_mut(columns) {
                    ui.horizontal_top(|ui| {
                        for (sensor, card) in row.iter_mut() {
                            theme::card()
                                .fill(theme::INSET)
                                .inner_margin(egui::Margin::same(10))
                                .show(ui, |ui| {
                                    ui.vertical(|ui| card.ui(ui, cx, sensor, gate, switch));
                                });
                        }
                    });
                }
            });
        self.debug.set(switch);
    }
}

/// How an Apply stands.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Flow {
    /// Writes are not answered yet.
    Writing,
    /// The vehicle refused a write: the field and the value it uses.
    Refused { field: String, in_effect: String },
    /// A write got no answer: the field.
    NoAnswer(String),
    /// Every write was answered and the vehicle's values equal the preset.
    Matches,
    /// The writes had their time and the values still differ.
    Differs,
}

/// Where an Apply of preset `id` that wrote the fields `written` at `at` stands at `now`, from the
/// echo of each write and the table.
pub fn apply_flow(
    table: &ParamTable,
    sensor: &Sensor,
    id: i32,
    written: &[String],
    at: Duration,
    now: Duration,
) -> Flow {
    let mut writing = false;
    let mut refused = None;
    let mut unanswered = None;
    for name in written {
        match table.writes.get(name) {
            Some(WriteStatus::Pending { .. }) => writing = true,
            Some(WriteStatus::Refused { in_effect, .. }) => {
                refused.get_or_insert((name, in_effect));
            }
            Some(WriteStatus::Unanswered { .. }) => {
                unanswered.get_or_insert(name);
            }
            None => {}
        }
    }
    if writing {
        Flow::Writing
    } else if let Some((field, in_effect)) = refused {
        Flow::Refused {
            field: field.clone(),
            in_effect: in_effect.to_string(),
        }
    } else if let Some(field) = unanswered {
        Flow::NoAnswer(field.clone())
    } else if fields_differ(table, sensor, id) == Some(false) {
        Flow::Matches
    } else if now.saturating_sub(at) < WRITE_WAIT {
        Flow::Writing
    } else {
        Flow::Differs
    }
}

/// Whether a value of the table is the preset's value (a REAL32 numerically, within its precision).
pub fn same(got: ParamValue, want: Value) -> bool {
    let got = got.as_f64();
    match want {
        Value::I32(v) => got == f64::from(v),
        Value::F32(v) => (got - f64::from(v)).abs() <= 1e-5 * f64::from(v).abs().max(1.0),
    }
}

/// Whether the table's fields differ from the values of preset `id`: `None` when the preset is
/// unknown or a field is not in the table (not loaded).
pub fn fields_differ(table: &ParamTable, sensor: &Sensor, id: i32) -> Option<bool> {
    let preset = preset(sensor, id)?;
    let mut differ = false;
    for (field, want) in sensor.fields.iter().zip(preset.values) {
        differ |= !same(table.value(field)?, *want);
    }
    Some(differ)
}

/// The writes of an Apply of `chosen`, in the order they are sent: first every field whose value goes
/// down, then every field whose value goes up, each group in the table's field order; a field that
/// does not change is not written. Every intermediate combination stays valid because the limits are
/// downward closed (BMP581: the highest ODR falls as OSR rises; BMM350: the highest averaging falls
/// as ODR rises; IIR and the IMU fields are unconstrained), so lowering an ODR, an OSR or an
/// averaging never makes a valid combination invalid. `None` when a field is not in the table.
pub fn write_plan(
    table: &ParamTable,
    sensor: &Sensor,
    chosen: &Preset,
) -> Option<Vec<(&'static str, ParamValue)>> {
    let mut down = Vec::new();
    let mut up = Vec::new();
    for (field, want) in sensor.fields.iter().zip(chosen.values) {
        let param = table.params.get(*field)?;
        let have = param.value(table.encoding);
        if same(have, *want) {
            continue;
        }
        let want = match want {
            Value::I32(v) => f64::from(*v),
            Value::F32(v) => f64::from(*v),
        };
        let value = param.kind.value_from(want)?;
        if want < have.as_f64() {
            down.push((*field, value));
        } else {
            up.push((*field, value));
        }
    }
    down.extend(up);
    Some(down)
}

impl SensorPreset {
    fn ui(
        &mut self,
        ui: &mut egui::Ui,
        cx: &mut ViewCtx,
        sensor: &Sensor,
        gate: Option<&'static str>,
        debug: bool,
    ) {
        let table = &cx.state.params;
        let now = cx.now.unwrap_or_default();
        let title = sensor_name(sensor);
        let imu = is_imu(sensor);
        if !imu
            && let Some(applied) = &self.applied
            && saved_since(cx.state, applied.at)
        {
            self.applied = None;
        }
        let current = Current::of(table, sensor);
        if !debug && self.staged == Some(0) {
            self.staged = None;
        }
        if match current {
            Current::Preset(id) => self.staged == Some(id),
            Current::Custom => self.staged == Some(0),
            Current::NotLoaded => false,
        } {
            self.staged = None;
        }
        let flow = self
            .applied
            .as_ref()
            .map(|a| apply_flow(table, sensor, a.id, &a.written, a.at, now));
        if matches!(
            flow,
            Some(Flow::Refused { .. } | Flow::NoAnswer(_) | Flow::Differs)
        ) {
            // The card shows what the values now match.
            self.staged = None;
        }
        let enabled = current != Current::NotLoaded;
        let chosen = match current {
            Current::Preset(id) => Some(self.staged.unwrap_or(id)),
            Current::Custom => Some(self.staged.unwrap_or(0)),
            Current::NotLoaded => None,
        };
        let chosen_preset = chosen.and_then(|id| preset(sensor, id));

        ui.set_width(CARD_WIDTH);
        let named = ui.strong(format!("{title} preset"));
        let shown = match (current, chosen) {
            (Current::NotLoaded, _) => "—".to_string(),
            (_, Some(0)) if debug => CUSTOM_TEXT.to_string(),
            (_, Some(0)) => CUSTOM_ACTIVE_TEXT.to_string(),
            _ => chosen_preset.map_or_else(String::new, |p| p.name.to_string()),
        };
        let mut hovered = None;
        let mut pick = None;
        let combo = ui.add_enabled_ui(enabled, |ui| {
            egui::ComboBox::from_id_salt(sensor.fields[0])
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
                    if debug
                        && ui
                            .selectable_label(chosen == Some(0), CUSTOM_TEXT)
                            .clicked()
                    {
                        pick = Some(0);
                    }
                })
        });
        combo.inner.response.labelled_by(named.id);
        if let Some(id) = pick {
            self.staged = Some(id);
            if matches!(flow, Some(Flow::Differs)) {
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
                ui.label(axis_line(axis, p, i));
            }
        }

        let flow = self.applied.as_ref().and(flow);
        let (mut text, mut colour) = status_line(current, flow.as_ref(), self.staged, sensor);
        if matches!(flow, Some(Flow::Matches)) && imu {
            self.advance_reboot(cx.state, now, cx.commands);
            (text, colour) = match self.applied.as_ref().map(|a| &a.reboot) {
                Some(Reboot::Saving(_)) => ("saving…".to_string(), GREY),
                Some(Reboot::Rebooting(_)) => (
                    "saved; the reboot command is sent: the vehicle restarts".to_string(),
                    YELLOW,
                ),
                Some(Reboot::Failed(why)) => (format!("{why}; the vehicle was not rebooted"), RED),
                _ => (PENDING_TEXT.to_string(), YELLOW),
            };
        }
        ui.add(egui::Label::new(egui::RichText::new(text).color(colour)).wrap());
        if matches!(flow, Some(Flow::Writing))
            || matches!(
                self.applied.as_ref().map(|a| &a.reboot),
                Some(Reboot::Saving(_) | Reboot::Rebooting(_))
            )
        {
            ui.ctx().request_repaint_after(Duration::from_millis(250));
        }
        if matches!(flow, Some(Flow::Matches)) {
            if imu {
                if let Some(applied) = &mut self.applied
                    && matches!(applied.reboot, Reboot::Idle | Reboot::Failed(_))
                    && self.confirm.button(ui, cx.now, SAVE_AND_REBOOT, gate)
                {
                    cx.commands.push(storage_command());
                    applied.reboot = Reboot::Saving(now);
                }
            } else if actions::button(ui, "Save to flash", gate) {
                cx.commands.push(storage_command());
            }
        }
        if enabled
            && let Some(id) = chosen.filter(|id| *id != 0)
            && actions::button(ui, "Apply", gate)
            && let Some(plan) = preset(sensor, id).and_then(|p| write_plan(table, sensor, p))
        {
            let written = plan.iter().map(|(name, _)| (*name).to_owned()).collect();
            for (name, value) in plan {
                cx.commands.push(Command::SetParam {
                    name: name.to_owned(),
                    value,
                });
            }
            self.applied = Some(Applied {
                at: now,
                id,
                written,
                reboot: Reboot::Idle,
            });
        }
        if debug && chosen == Some(0) {
            self.custom_fields(ui, cx, sensor, gate);
        }
    }

    /// An IMU preset that was applied: once the save is on its way, its answer decides whether the
    /// reboot is sent (only an ACCEPTED save sends 246, once).
    fn advance_reboot(&mut self, state: &VehicleState, now: Duration, commands: &mut Vec<Command>) {
        let Some(applied) = &mut self.applied else {
            return;
        };
        match &applied.reboot {
            Reboot::Saving(at) => {
                let at = *at;
                match command_since(state, 245, at).map(|r| &r.status) {
                    Some(CommandStatus::Finished { accepted: true, .. }) => {
                        commands.push(Command::SendCommand {
                            command: MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        });
                        applied.reboot = Reboot::Rebooting(now);
                    }
                    Some(CommandStatus::Finished { text, .. }) => {
                        applied.reboot = Reboot::Failed(format!("save refused: {text}"));
                    }
                    Some(CommandStatus::NoAnswer | CommandStatus::NoResult) => {
                        applied.reboot = Reboot::Failed("save unanswered".to_string());
                    }
                    Some(CommandStatus::Refused(why)) => {
                        applied.reboot = Reboot::Failed(format!("save not sent: {why}"));
                    }
                    Some(CommandStatus::Waiting | CommandStatus::InProgress { .. }) => {}
                    None if now.saturating_sub(at) > SAVE_WAIT => {
                        applied.reboot = Reboot::Failed("save not sent".to_string());
                    }
                    None => {}
                }
            }
            Reboot::Rebooting(at) => {
                let at = *at;
                match command_since(state, 246, at).map(|r| &r.status) {
                    Some(CommandStatus::Finished {
                        text,
                        accepted: false,
                    }) => {
                        applied.reboot = Reboot::Failed(format!("reboot refused: {text}"));
                    }
                    Some(CommandStatus::NoAnswer | CommandStatus::NoResult) => {
                        applied.reboot = Reboot::Failed("reboot unanswered".to_string());
                    }
                    _ => {}
                }
            }
            Reboot::Idle | Reboot::Failed(_) => {}
        }
    }

    /// The governed fields of a vehicle in Custom, as typed values with one Apply. The vehicle
    /// validates every write; what it refuses is shown beside the field.
    fn custom_fields(
        &mut self,
        ui: &mut egui::Ui,
        cx: &mut ViewCtx,
        sensor: &Sensor,
        gate: Option<&'static str>,
    ) {
        let table = &cx.state.params;
        self.edits
            .retain(|name, text| !matches!(field_meaning(table, name, text), Some(Meaning::Same)));
        ui.separator();
        ui.strong("Raw settings (Custom)");
        for field in sensor.fields {
            let Some(current) = table.value(field) else {
                continue;
            };
            ui.horizontal(|ui| {
                ui.label(*field);
                let mut text = self
                    .edits
                    .get(*field)
                    .cloned()
                    .unwrap_or_else(|| current.to_string());
                let response = ui
                    .add(egui::TextEdit::singleline(&mut text).desired_width(70.0))
                    .accessible_name(format!("value of {field}"));
                if response.changed() {
                    if text.trim() == current.to_string() {
                        self.edits.remove(*field);
                    } else {
                        self.edits.insert((*field).to_owned(), text);
                    }
                }
                let (note, colour) = match table.writes.get(*field) {
                    Some(WriteStatus::Pending { .. }) => ("writing…".to_string(), YELLOW),
                    Some(WriteStatus::Refused { in_effect, .. }) => {
                        (format!("refused: vehicle uses {in_effect}"), RED)
                    }
                    Some(WriteStatus::Unanswered { .. }) => ("no answer".to_string(), RED),
                    None if applies_after_reboot(field) => (AFTER_REBOOT_TEXT.to_string(), GREY),
                    None => (String::new(), GREY),
                };
                ui.colored_label(colour, note);
            });
        }
        let sendable = self.sendable_fields(table);
        if actions::button(
            ui,
            "Apply fields",
            gate.or(sendable.is_empty().then_some("nothing edited")),
        ) {
            for (name, value) in sendable {
                cx.commands.push(Command::SetParam { name, value });
            }
        }
    }

    /// The valid edits that differ from the vehicle's value and are not on their way, in field order.
    fn sendable_fields(&self, table: &ParamTable) -> Vec<(String, ParamValue)> {
        self.edits
            .iter()
            .filter_map(|(name, text)| match field_meaning(table, name, text)? {
                Meaning::Valid(value) => Some((name.clone(), value)),
                Meaning::Same | Meaning::Invalid => None,
            })
            .filter(|(name, value)| {
                !matches!(table.writes.get(name), Some(WriteStatus::Pending { value: sent }) if sent == value)
            })
            .collect()
    }
}

/// What the typed text of a field means.
enum Meaning {
    Same,
    Valid(ParamValue),
    Invalid,
}

fn field_meaning(table: &ParamTable, name: &str, text: &str) -> Option<Meaning> {
    let param = table.params.get(name)?;
    let current = param.value(table.encoding);
    Some(
        match text
            .trim()
            .parse::<f64>()
            .ok()
            .and_then(|n| param.kind.value_from(n))
        {
            Some(v) if v == current => Meaning::Same,
            Some(v) => Meaning::Valid(v),
            None => Meaning::Invalid,
        },
    )
}

fn storage_command() -> Command {
    Command::SendCommand {
        command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
        params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
}

/// The vehicle's last command when it is `command` with param1 = 1 started at or after `at`.
fn command_since(
    state: &VehicleState,
    command: u32,
    at: Duration,
) -> Option<&gcs_mavlink::CommandRecord> {
    state
        .command
        .as_ref()
        .filter(|r| r.command == command && r.params[0] == 1.0 && r.started >= at)
}

/// The one status line of a card and its colour.
pub fn status_line(
    current: Current,
    flow: Option<&Flow>,
    staged: Option<i32>,
    sensor: &Sensor,
) -> (String, Color32) {
    let now_matches = || match current {
        Current::Preset(id) => preset(sensor, id).map_or("", |p| p.name),
        _ => CUSTOM_ACTIVE_TEXT,
    };
    if let Some(flow) = flow {
        return match flow {
            Flow::Writing => ("applying…".to_string(), GREY),
            Flow::Refused { field, in_effect } => (
                format!(
                    "refused: {field} (vehicle uses {in_effect}); the values now match: {}",
                    now_matches()
                ),
                RED,
            ),
            Flow::NoAnswer(field) => (
                format!(
                    "no answer: {field}; the values now match: {}",
                    now_matches()
                ),
                RED,
            ),
            Flow::Matches if is_imu(sensor) => (PENDING_TEXT.to_string(), YELLOW),
            Flow::Matches => (NOT_SAVED_TEXT.to_string(), YELLOW),
            Flow::Differs => (MISMATCH_TEXT.to_string(), RED),
        };
    }
    match current {
        Current::NotLoaded => (NOT_LOADED_TEXT.to_string(), GREY),
        Current::Custom => ("Custom: the values match no preset".to_string(), YELLOW),
        Current::Preset(_) => match staged.and_then(|s| preset(sensor, s)) {
            Some(p) => (format!("{}: not applied", p.name), GREY),
            None => (format!("on the vehicle: {}", now_matches()), GREY),
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
        preset
            .physical
            .get(i)
            .map_or_else(String::new, |v| number_text(*v)),
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

/// Where the chart of `sensor` puts its labels inside `area`: the radius is [`CHART_RADIUS`] where
/// every label fits and smaller only as far as needed.
pub struct ChartGeometry {
    pub radius: f32,
    /// Each axis' label text and the rectangle it occupies.
    pub labels: Vec<(&'static str, Rect, Align2)>,
}

const LABEL_FONT_SIZE: f32 = 10.0;

fn label_anchor(dir: Vec2) -> Align2 {
    if dir.x > 0.3 {
        Align2::LEFT_CENTER
    } else if dir.x < -0.3 {
        Align2::RIGHT_CENTER
    } else if dir.y < 0.0 {
        Align2::CENTER_BOTTOM
    } else {
        Align2::CENTER_TOP
    }
}

pub fn chart_geometry(painter: &Painter, sensor: &Sensor, area: Rect) -> ChartGeometry {
    let centre = area.center();
    let n = sensor.axes.len();
    let sizes: Vec<Vec2> = sensor
        .axes
        .iter()
        .map(|axis| {
            painter
                .layout_no_wrap(
                    axis.name.to_owned(),
                    FontId::proportional(LABEL_FONT_SIZE),
                    Color32::WHITE,
                )
                .size()
        })
        .collect();
    let place = |radius: f32| -> Vec<(&'static str, Rect, Align2)> {
        sensor
            .axes
            .iter()
            .zip(&sizes)
            .enumerate()
            .map(|(i, (axis, size))| {
                let dir = spoke(i, n);
                let anchor = label_anchor(dir);
                let rect = anchor.anchor_size(centre + dir * (radius + LABEL_GAP), *size);
                (axis.name, rect, anchor)
            })
            .collect()
    };
    let mut radius = CHART_RADIUS;
    while radius > MIN_CHART_RADIUS && !place(radius).iter().all(|(_, r, _)| area.contains_rect(*r))
    {
        radius -= 1.0;
    }
    ChartGeometry {
        radius,
        labels: place(radius),
    }
}

fn draw_chart(
    ui: &mut egui::Ui,
    sensor: &Sensor,
    selected: Option<&Preset>,
    hovered: Option<&Preset>,
) {
    let (response, painter) = ui.allocate_painter(CHART_SIZE, Sense::hover());
    let centre = response.rect.center();
    let n = sensor.axes.len();
    let ring_colour = ui.visuals().widgets.noninteractive.bg_stroke.color;
    let text_colour = ui.visuals().text_color();
    let geometry = chart_geometry(&painter, sensor, response.rect);
    let radius = geometry.radius;
    for ring in RINGS {
        painter.add(Shape::closed_line(
            radar_vertices(&vec![ring; n], centre, radius),
            Stroke::new(1.0, ring_colour),
        ));
    }
    for (i, (name, rect, anchor)) in geometry.labels.iter().enumerate() {
        let dir = spoke(i, n);
        painter.line_segment(
            [centre, centre + dir * radius],
            Stroke::new(1.0, ring_colour),
        );
        painter.text(
            anchor.pos_in_rect(rect),
            *anchor,
            name,
            FontId::proportional(LABEL_FONT_SIZE),
            text_colour,
        );
    }
    paint_polygon(
        &painter,
        centre,
        radius,
        selected,
        Some((ORANGE_FILL, ORANGE)),
        2.0,
    );
    if let Some(p) = hovered
        && selected.is_none_or(|s| s.id != p.id)
    {
        paint_polygon(&painter, centre, radius, Some(p), None, 1.5);
    }
}

fn paint_polygon(
    painter: &Painter,
    centre: Pos2,
    radius: f32,
    preset: Option<&Preset>,
    fill: Option<(Color32, Color32)>,
    width: f32,
) {
    let Some(p) = preset else { return };
    let vertices = radar_vertices(p.scores, centre, radius);
    let outline = match fill {
        Some((fill, outline)) => {
            painter.add(fill_shape(&vertices, centre, fill));
            outline
        }
        None => COMPARE,
    };
    painter.add(Shape::closed_line(vertices, Stroke::new(width, outline)));
}
