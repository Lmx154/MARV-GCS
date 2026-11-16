//! Radio view: the sixteen channels of `RC_CHANNELS` as bars, the link quality, the age of the last stick
//! report, the RC row of `MARV_DEVICE_STATUS` and `RADIO_STATUS` when it is present. The four function
//! bars (roll, pitch, yaw, throttle) show what the vehicle's `MANUAL_CONTROL` carries, and on channels 1
//! to 8 the markers `MIN`, `TRIM` and `MAX` and the function of the channel come from the vehicle's RC
//! parameters (`docs/link-contract.md` section 12). The station computes no normalised value. The
//! calibration (Configure sessions): capture the extremes of the raw channels, centre, review, then Apply
//! writes the values that differ with `PARAM_SET` and Save to flash sends command 245; nothing is saved
//! or sent before the user presses a button. No mapping editor yet.

use std::time::Duration;

use eframe::egui::{self, Color32, Rect, Sense, Stroke, StrokeKind, vec2};
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::messages::{MessageEntry, MessageStore};
use gcs_mavlink::services::params::{FetchStatus, ParamTable, ParamValue, WriteStatus};
use gcs_mavlink::{Command, CommandStatus, VehicleState};

use super::actions;
use super::devices;
use super::{View, ViewCtx};
use crate::status::{self, GREY, YELLOW};

/// The RC row's `device_id` in `MARV_DEVICE_STATUS`.
const RC_DEVICE: u8 = 7;
/// The bar's scale in microseconds, and the marks drawn on it.
pub const SCALE_US: (f64, f64) = (800.0, 2200.0);
pub const MARKS_US: [f64; 3] = [1000.0, 1500.0, 2000.0];
/// RC_CHANNELS older than this (or never seen) is "no stick data".
pub const STICK_STALE: Duration = Duration::from_secs(2);
/// The channels the message carries.
pub const CHANNELS: usize = 16;
/// The channels that have `RCn_MIN`, `RCn_TRIM` and `RCn_MAX`.
pub const MAPPED_CHANNELS: usize = 8;
/// The mark of a channel's `MIN`, `TRIM` and `MAX` on its bar.
const MARKER: Color32 = Color32::from_rgb(235, 160, 40);
/// Shown in place of the markers and tags when the vehicle has no `RC1_MIN`.
pub const NO_RC_PARAMS: &str = "this firmware has no RC mapping parameters";
const BAR_SIZE: egui::Vec2 = vec2(420.0, 16.0);
/// A channel counts as moved when its captured range reaches below the first and above the second.
pub const MOVED_BELOW_US: u16 = 1300;
pub const MOVED_ABOVE_US: u16 = 1700;
/// Shown next to "Calibrate radio".
pub const PREPARE_TEXT: &str =
    "Transmitter on in its CRSF model; battery unplugged, no propellers.";
pub const CAPTURE_TEXT: &str = "Move every stick and switch slowly to both ends and hold each end briefly, then press Done.";
pub const CENTRE_TEXT: &str = "Centre the sticks, throttle down, then OK";
pub const NOT_SAVED_TEXT: &str = "Applied in RAM; not saved to flash";
/// The drawn range of a captured channel.
const CAPTURED: Color32 = Color32::from_rgba_premultiplied(40, 110, 60, 140);

#[derive(Default)]
pub struct Radio {
    flow: Flow,
    /// The last Apply, until a save is accepted.
    applied: Option<Applied>,
}

/// Where the calibration stands.
#[derive(Default)]
enum Flow {
    #[default]
    Idle,
    Capturing(Capture),
    Centre(Capture),
    Review(Capture),
}

/// What the user has moved so far, per channel 1 to 16.
#[derive(Debug, Clone, Default)]
pub struct Capture {
    pub ranges: [Option<(u16, u16)>; CHANNELS],
    pub trims: [Option<u16>; CHANNELS],
}

impl Capture {
    /// Widen every channel's range by the latest values.
    pub fn observe(&mut self, values: &[Option<u16>]) {
        for (range, value) in self.ranges.iter_mut().zip(values) {
            if let Some(v) = *value {
                *range = Some(match *range {
                    Some((lo, hi)) => (lo.min(v), hi.max(v)),
                    None => (v, v),
                });
            }
        }
    }

    /// Take the current value of each channel as its trim.
    pub fn centre(&mut self, values: &[Option<u16>]) {
        for (trim, value) in self.trims.iter_mut().zip(values) {
            *trim = *value;
        }
    }

    /// The new (`MIN`, `TRIM`, `MAX`) of a channel (0 for channel 1) that was moved; `None` for "not moved".
    /// `TRIM` is clamped into [`MIN`, `MAX`].
    pub fn moved(&self, channel: usize) -> Option<(i64, Option<i64>, i64)> {
        let (lo, hi) = self.ranges.get(channel).copied().flatten()?;
        if !(lo < MOVED_BELOW_US && hi > MOVED_ABOVE_US) {
            return None;
        }
        let (lo, hi) = (i64::from(lo), i64::from(hi));
        let trim = self.trims[channel].map(|t| i64::from(t).clamp(lo, hi));
        Some((lo, trim, hi))
    }

    /// The writes of Apply: for channels 1 to 8 that were moved, each of `RCn_MIN`, `RCn_TRIM` and
    /// `RCn_MAX` whose value differs from the vehicle's current one. Channels 9 to 16 never.
    pub fn writes(&self, mapping: &Mapping) -> Vec<(String, i64)> {
        let mut out = Vec::new();
        for channel in 0..MAPPED_CHANNELS {
            let (Some((min, trim, max)), Some(current)) =
                (self.moved(channel), mapping.channel(channel))
            else {
                continue;
            };
            for (suffix, new, now) in [
                ("MIN", Some(min), current.min),
                ("TRIM", trim, current.trim),
                ("MAX", Some(max), current.max),
            ] {
                if let (Some(new), Some(now)) = (new, now)
                    && new != now
                {
                    out.push((format!("RC{}_{suffix}", channel + 1), new));
                }
            }
        }
        out
    }
}

/// An Apply that was sent.
struct Applied {
    /// Session time of the press.
    at: Duration,
    sent: Vec<(String, i64)>,
    /// A save of this session was accepted after `at`.
    saved: bool,
}

/// Whether the vehicle's last command is an accepted "Save to flash" started at or after `at`.
fn saved_since(state: &VehicleState, at: Duration) -> bool {
    state.command.as_ref().is_some_and(|r| {
        r.command == 245
            && r.params[0] == 1.0
            && r.started >= at
            && matches!(r.status, CommandStatus::Finished { accepted: true, .. })
    })
}

impl View for Radio {
    fn name(&self) -> &'static str {
        "Radio"
    }

    fn session_changed(&mut self) {
        *self = Radio::default();
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        let store = &cx.state.messages;
        let sticks = Sticks::of(store, cx.now);
        let (text, colour) = sticks.age_line();
        ui.colored_label(colour, text);
        ui.label(link_quality_text(sticks.rssi));
        let rc_row = devices::rows(store, cx.now)
            .into_iter()
            .find(|row| row.device == RC_DEVICE);
        match &rc_row {
            Some(row) => {
                let colour = if row.stale {
                    GREY
                } else {
                    devices::state_colour(&row.state)
                };
                ui.colored_label(
                    colour,
                    format!(
                        "RC row: {}, last error {}, {}",
                        row.state, row.last_error, row.received
                    ),
                );
            }
            None => {
                ui.label("RC row: no RC row yet");
            }
        }
        ui.label(radio_status_text(store.entry("RADIO_STATUS", 0)));
        ui.separator();
        let control = ManualControl::of(store, cx.now);
        ui.label("Sticks as the vehicle maps them (MANUAL_CONTROL)");
        for function in Function::ALL {
            let value = control.map(|c| function.value(c));
            ui.horizontal(|ui| {
                function_bar(ui, value, function.range());
                ui.label(function.text(value));
            });
        }
        ui.separator();
        let mapping = Mapping::of(&cx.state.params);
        if mapping == Mapping::Missing {
            ui.label(NO_RC_PARAMS);
        }
        self.calibration(ui, cx, &sticks, &mapping);
        ui.separator();
        let capture = match &self.flow {
            Flow::Capturing(c) | Flow::Centre(c) | Flow::Review(c) => Some(c),
            Flow::Idle => None,
        };
        marks_header(ui);
        egui::ScrollArea::vertical().show(ui, |ui| {
            for channel in 0..sticks.shown() {
                let value = sticks
                    .values
                    .get(channel)
                    .copied()
                    .flatten()
                    .filter(|_| sticks.fresh);
                let marks = mapping.channel(channel);
                let captured = capture.and_then(|c| c.ranges[channel]);
                ui.horizontal(|ui| {
                    ui.monospace(format!("CH {:>2}", channel + 1));
                    bar(ui, value, marks, captured);
                    ui.label(value_text(value));
                    if let Some((lo, hi)) = captured {
                        ui.label(format!("captured {lo} to {hi}"));
                    }
                    if let Some(marks) = marks {
                        ui.label(marks.text());
                    }
                });
            }
        });
    }
}

impl Radio {
    /// The calibration: the button and its reason, the steps, and the "not saved" line.
    fn calibration(
        &mut self,
        ui: &mut egui::Ui,
        cx: &mut ViewCtx,
        sticks: &Sticks,
        mapping: &Mapping,
    ) {
        let live: Vec<Option<u16>> = sticks
            .values
            .iter()
            .map(|v| v.filter(|_| sticks.fresh))
            .collect();
        if let Some(applied) = &mut self.applied
            && !applied.saved
            && saved_since(cx.state, applied.at)
        {
            applied.saved = true;
        }
        ui.strong("Calibration");
        let flow = std::mem::take(&mut self.flow);
        self.flow = match flow {
            Flow::Idle => {
                let reason = calibrate_gate(cx, sticks, mapping);
                ui.horizontal(|ui| {
                    let start = actions::button(ui, "Calibrate radio", reason);
                    if let Some(reason) = reason {
                        ui.colored_label(YELLOW, reason);
                    }
                    ui.label(PREPARE_TEXT);
                    if start {
                        Flow::Capturing(Capture::default())
                    } else {
                        Flow::Idle
                    }
                })
                .inner
            }
            Flow::Capturing(mut capture) => {
                capture.observe(&live);
                ui.label(CAPTURE_TEXT);
                if !sticks.fresh {
                    ui.colored_label(YELLOW, "no stick data");
                }
                let (done, cancel) = ui
                    .horizontal(|ui| {
                        (
                            actions::button(ui, "Done", None),
                            actions::button(ui, "Cancel", None),
                        )
                    })
                    .inner;
                if cancel {
                    Flow::Idle
                } else if done {
                    Flow::Centre(capture)
                } else {
                    Flow::Capturing(capture)
                }
            }
            Flow::Centre(mut capture) => {
                ui.label(CENTRE_TEXT);
                let reason = (!sticks.fresh).then_some("no stick data");
                let (ok, cancel) = ui
                    .horizontal(|ui| {
                        (
                            actions::button(ui, "OK", reason),
                            actions::button(ui, "Cancel", None),
                        )
                    })
                    .inner;
                if cancel {
                    Flow::Idle
                } else if ok {
                    capture.centre(&live);
                    Flow::Review(capture)
                } else {
                    Flow::Centre(capture)
                }
            }
            Flow::Review(capture) => {
                review_table(ui, &capture, mapping);
                let writes = capture.writes(mapping);
                let reason = apply_gate(cx, writes.len());
                let (apply, cancel) = ui
                    .horizontal(|ui| {
                        let apply = actions::button(ui, "Apply", reason);
                        if let Some(reason) = reason {
                            ui.colored_label(YELLOW, reason);
                        }
                        (apply, actions::button(ui, "Cancel", None))
                    })
                    .inner;
                if apply {
                    for (name, value) in &writes {
                        cx.commands.push(Command::SetParam {
                            name: name.clone(),
                            value: ParamValue::I32(*value as i32),
                        });
                    }
                    self.applied = Some(Applied {
                        at: cx.now.unwrap_or_default(),
                        sent: writes,
                        saved: false,
                    });
                }
                if apply || cancel {
                    Flow::Idle
                } else {
                    Flow::Review(capture)
                }
            }
        };
        if let Some(applied) = &self.applied
            && !applied.saved
        {
            applied_section(ui, cx, applied);
        }
    }
}

/// Why "Calibrate radio" is disabled; `None` when it may be pressed.
fn calibrate_gate(cx: &ViewCtx, sticks: &Sticks, mapping: &Mapping) -> Option<&'static str> {
    if let Some(reason) = status::configure_blocked(cx.access) {
        return Some(reason);
    }
    if cx.now.is_none() {
        return Some(status::NOT_CONNECTED);
    }
    if !sticks.fresh {
        return Some("no stick data");
    }
    match mapping {
        Mapping::NotReady => Some("the parameter list is not complete"),
        Mapping::Missing => Some(NO_RC_PARAMS),
        Mapping::Marks(_) => None,
    }
}

/// Why "Apply" is disabled; `None` when it may be pressed.
fn apply_gate(cx: &ViewCtx, writes: usize) -> Option<&'static str> {
    if let Some(reason) = status::configure_blocked(cx.access) {
        return Some(reason);
    }
    if cx.now.is_none() {
        return Some(status::NOT_CONNECTED);
    }
    (writes == 0).then_some("nothing to write: no channel was moved, or the vehicle already has the values")
}

/// The review: channels 1 to 8 as current -> captured, channels 9 to 16 as captured only.
fn review_table(ui: &mut egui::Ui, capture: &Capture, mapping: &Mapping) {
    ui.label("Review (current -> captured)");
    let number = |v: Option<i64>| v.map_or_else(|| "—".to_string(), |v| v.to_string());
    for channel in 0..CHANNELS {
        let text = if channel < MAPPED_CHANNELS {
            let current = mapping.channel(channel);
            let cur = |f: fn(&ChannelMarks) -> Option<i64>| number(current.and_then(f));
            match capture.moved(channel) {
                Some((min, trim, max)) => format!(
                    "CH {:>2}: MIN {} -> {min}  TRIM {} -> {}  MAX {} -> {max}",
                    channel + 1,
                    cur(|m| m.min),
                    cur(|m| m.trim),
                    number(trim),
                    cur(|m| m.max),
                ),
                None => format!(
                    "CH {:>2}: not moved (MIN {} TRIM {} MAX {} kept)",
                    channel + 1,
                    cur(|m| m.min),
                    cur(|m| m.trim),
                    cur(|m| m.max),
                ),
            }
        } else {
            match capture.ranges[channel] {
                Some((lo, hi)) => format!(
                    "CH {:>2}: captured {lo} to {hi} (not written: no parameters)",
                    channel + 1
                ),
                None => format!("CH {:>2}: no data (not written: no parameters)", channel + 1),
            }
        };
        ui.monospace(text);
    }
}

/// After Apply, until a save is accepted: the "not saved" line, what the vehicle did with the writes,
/// and "Save to flash" (the Parameters tab's command and gate).
fn applied_section(ui: &mut egui::Ui, cx: &mut ViewCtx, applied: &Applied) {
    ui.colored_label(YELLOW, NOT_SAVED_TEXT);
    let table = &cx.state.params;
    for (name, value) in &applied.sent {
        match table.writes.get(name) {
            Some(WriteStatus::Pending { .. }) => {
                ui.label(format!("{name}: writing…"));
            }
            Some(WriteStatus::Refused { in_effect, .. }) => {
                ui.colored_label(
                    status::RED,
                    format!("{name}: refused: vehicle uses {in_effect} (wanted {value})"),
                );
            }
            Some(WriteStatus::Unanswered { .. }) => {
                ui.colored_label(status::RED, format!("{name}: no answer"));
            }
            None => {}
        }
    }
    let gate = status::command_gate(cx.access, cx.state, cx.now.is_some());
    ui.horizontal(|ui| {
        if actions::button(ui, "Save to flash", gate) {
            cx.commands.push(Command::SendCommand {
                command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
                params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            });
        }
        if let Some(reason) = gate
            && cx.access == gcs_mavlink::Access::Configure
        {
            ui.colored_label(YELLOW, reason);
        }
    });
    actions::status_line(ui, cx.state);
}

/// What the latest `RC_CHANNELS` says.
pub struct Sticks {
    /// Microseconds of channel 1 to 16 as received; `None` for a channel the message marks unused (65535) or
    /// does not carry (beyond `chancount`).
    pub values: Vec<Option<u16>>,
    /// `chancount`, at most 16; 16 when nothing was received.
    pub count: usize,
    pub rssi: Option<u8>,
    /// How old the message is; `None` when never seen or no session.
    pub age: Option<Duration>,
    /// Seen within [`STICK_STALE`].
    pub fresh: bool,
}

impl Sticks {
    pub fn of(store: &MessageStore, now: Option<Duration>) -> Sticks {
        let Some(entry) = store.entry("RC_CHANNELS", 0) else {
            return Sticks::none();
        };
        let age = now.map(|now| now.saturating_sub(entry.last_seen));
        let count = entry
            .number("chancount")
            .map_or(CHANNELS, |n| (n as usize).min(CHANNELS));
        let values = (0..CHANNELS)
            .map(|i| {
                if i >= count {
                    return None;
                }
                let raw = entry.number(&format!("chan{}_raw", i + 1))? as u16;
                (raw != u16::MAX).then_some(raw)
            })
            .collect();
        Sticks {
            values,
            count,
            rssi: entry.number("rssi").map(|r| r as u8),
            age,
            fresh: age.is_some_and(|a| a <= STICK_STALE),
        }
    }

    fn none() -> Sticks {
        Sticks {
            values: vec![None; CHANNELS],
            count: CHANNELS,
            rssi: None,
            age: None,
            fresh: false,
        }
    }

    /// How many bars: `chancount` of the last message, 16 before any.
    pub fn shown(&self) -> usize {
        self.count
    }

    /// The age of the last RC_CHANNELS in words; "no stick data" when it is old or was never seen.
    pub fn age_line(&self) -> (String, Color32) {
        match (self.fresh, self.age) {
            (true, Some(age)) => (
                format!("RC_CHANNELS {:.1} s ago", age.as_secs_f64()),
                crate::status::GREEN,
            ),
            (false, Some(age)) => (
                format!(
                    "no stick data (last RC_CHANNELS {:.1} s ago)",
                    age.as_secs_f64()
                ),
                YELLOW,
            ),
            _ => ("no stick data".to_string(), YELLOW),
        }
    }
}

/// What the latest `MANUAL_CONTROL` says, as the vehicle sent it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ManualControl {
    /// `y`: -1000 to 1000, positive for stick right.
    pub roll: i32,
    /// `x`: -1000 to 1000, positive for stick forward.
    pub pitch: i32,
    /// `r`: -1000 to 1000, positive for yaw right.
    pub yaw: i32,
    /// `z`: 0 to 1000, 1000 for throttle up.
    pub throttle: i32,
}

impl ManualControl {
    /// `None` when none was received, when it is older than [`STICK_STALE`] or when there is no session.
    pub fn of(store: &MessageStore, now: Option<Duration>) -> Option<ManualControl> {
        let entry = store.entry("MANUAL_CONTROL", 0)?;
        let age = now?.saturating_sub(entry.last_seen);
        if age > STICK_STALE {
            return None;
        }
        let field = |name: &str| entry.number(name).map(|n| n as i32);
        Some(ManualControl {
            roll: field("y")?,
            pitch: field("x")?,
            yaw: field("r")?,
            throttle: field("z")?,
        })
    }
}

/// The four functions of the function bars, in the order they are drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Function {
    Roll,
    Pitch,
    Yaw,
    Throttle,
}

impl Function {
    pub const ALL: [Function; 4] = [
        Function::Roll,
        Function::Pitch,
        Function::Yaw,
        Function::Throttle,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Function::Roll => "Roll",
            Function::Pitch => "Pitch",
            Function::Yaw => "Yaw",
            Function::Throttle => "Throttle",
        }
    }

    /// The direction the user should see when they move the stick (the contract's sign convention).
    pub fn direction(self) -> &'static str {
        match self {
            Function::Roll => "stick right: +",
            Function::Pitch => "stick forward: +",
            Function::Yaw => "yaw right: +",
            Function::Throttle => "throttle up: 1000",
        }
    }

    pub fn value(self, control: ManualControl) -> i32 {
        match self {
            Function::Roll => control.roll,
            Function::Pitch => control.pitch,
            Function::Yaw => control.yaw,
            Function::Throttle => control.throttle,
        }
    }

    /// The scale of the bar.
    pub fn range(self) -> (i32, i32) {
        match self {
            Function::Throttle => (0, 1000),
            _ => (-1000, 1000),
        }
    }

    /// The line next to the bar: the name, the value or "no data", and the expected direction.
    pub fn text(self, value: Option<i32>) -> String {
        match value {
            Some(v) => format!("{}: {v} ({})", self.name(), self.direction()),
            None => format!("{}: no data ({})", self.name(), self.direction()),
        }
    }
}

/// `MIN`, `TRIM` and `MAX` of one channel and the functions mapped to it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelMarks {
    pub min: Option<i64>,
    pub trim: Option<i64>,
    pub max: Option<i64>,
    pub functions: Vec<Function>,
}

impl ChannelMarks {
    fn text(&self) -> String {
        let number = |v: Option<i64>| v.map_or_else(|| "—".to_string(), |v| v.to_string());
        let mut text = format!(
            "MIN {} TRIM {} MAX {}",
            number(self.min),
            number(self.trim),
            number(self.max)
        );
        if !self.functions.is_empty() {
            let names: Vec<_> = self.functions.iter().map(|f| f.name()).collect();
            text.push_str(&format!("  [{}]", names.join(", ")));
        }
        text
    }
}

/// The vehicle's RC parameters as the markers and tags of channels 1 to 8 need them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Mapping {
    /// The parameter list is not complete: nothing is drawn.
    NotReady,
    /// The list is complete and has no `RC1_MIN`.
    Missing,
    /// One entry per channel 1 to 8.
    Marks(Vec<ChannelMarks>),
}

impl Mapping {
    pub fn of(params: &ParamTable) -> Mapping {
        if params.fetch != FetchStatus::Complete {
            return Mapping::NotReady;
        }
        if params.value("RC1_MIN").is_none() {
            return Mapping::Missing;
        }
        let number = |name: &str| params.value(name).map(|v| v.as_f64() as i64);
        let mut marks: Vec<ChannelMarks> = (1..=MAPPED_CHANNELS)
            .map(|n| ChannelMarks {
                min: number(&format!("RC{n}_MIN")),
                trim: number(&format!("RC{n}_TRIM")),
                max: number(&format!("RC{n}_MAX")),
                functions: Vec::new(),
            })
            .collect();
        for (name, function) in [
            ("RCMAP_ROLL", Function::Roll),
            ("RCMAP_PITCH", Function::Pitch),
            ("RCMAP_THROTTLE", Function::Throttle),
            ("RCMAP_YAW", Function::Yaw),
        ] {
            let channel = number(name).and_then(|n| usize::try_from(n).ok());
            if let Some(entry) = channel
                .and_then(|c| c.checked_sub(1))
                .and_then(|i| marks.get_mut(i))
            {
                entry.functions.push(function);
            }
        }
        Mapping::Marks(marks)
    }

    /// The markers and tags of channel `index` (0 for channel 1), if there are any to draw.
    pub fn channel(&self, index: usize) -> Option<&ChannelMarks> {
        match self {
            Mapping::Marks(marks) => marks.get(index),
            _ => None,
        }
    }
}

/// `rssi` of RC_CHANNELS: 0 to 254, 255 unknown.
pub fn link_quality_text(rssi: Option<u8>) -> String {
    match rssi {
        None | Some(255) => "Link quality: unknown".to_string(),
        Some(r) => format!(
            "Link quality: rssi {r} of 254 ({} %)",
            u32::from(r) * 100 / 254
        ),
    }
}

/// The share of the bar (0 to 1) a value in microseconds fills on the 800 to 2200 scale.
pub fn fraction(us: u16) -> f32 {
    ((f64::from(us) - SCALE_US.0) / (SCALE_US.1 - SCALE_US.0)).clamp(0.0, 1.0) as f32
}

fn value_text(value: Option<u16>) -> String {
    value.map_or_else(|| "—".to_string(), |us| us.to_string())
}

/// `RADIO_STATUS` in one line, or that there is none.
pub fn radio_status_text(entry: Option<&MessageEntry>) -> String {
    let Some(entry) = entry else {
        return "RADIO_STATUS: none received".to_string();
    };
    let field = |name: &str| {
        entry
            .number(name)
            .map_or_else(|| "—".to_string(), |n| format!("{n}"))
    };
    format!(
        "RADIO_STATUS: rssi {}, remrssi {}, txbuf {} %, noise {}, remnoise {}, rxerrors {}, fixed {}",
        field("rssi"),
        field("remrssi"),
        field("txbuf"),
        field("noise"),
        field("remnoise"),
        field("rxerrors"),
        field("fixed"),
    )
}

fn marks_header(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.monospace("     ");
        let (rect, _) = ui.allocate_exact_size(vec2(BAR_SIZE.x, 14.0), Sense::hover());
        let painter = ui.painter_at(rect);
        for mark in MARKS_US {
            let x = rect.left() + fraction(mark as u16) * rect.width();
            painter.text(
                egui::pos2(x, rect.center().y),
                egui::Align2::CENTER_CENTER,
                format!("{mark}"),
                egui::FontId::proportional(11.0),
                ui.visuals().weak_text_color(),
            );
        }
    });
}

fn bar(
    ui: &mut egui::Ui,
    value: Option<u16>,
    marks: Option<&ChannelMarks>,
    captured: Option<(u16, u16)>,
) {
    let (rect, _) = ui.allocate_exact_size(BAR_SIZE, Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, ui.visuals().extreme_bg_color);
    if let Some((lo, hi)) = captured {
        let span = Rect::from_min_max(
            egui::pos2(rect.left() + fraction(lo) * rect.width(), rect.top()),
            egui::pos2(rect.left() + fraction(hi) * rect.width(), rect.bottom()),
        );
        painter.rect_filled(span, 0.0, CAPTURED);
    }
    if let Some(us) = value {
        let filled =
            Rect::from_min_size(rect.min, vec2(rect.width() * fraction(us), rect.height()));
        painter.rect_filled(filled, 2.0, Color32::from_rgb(70, 150, 220));
    }
    for mark in MARKS_US {
        let x = rect.left() + fraction(mark as u16) * rect.width();
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
            Stroke::new(1.0, ui.visuals().weak_text_color()),
        );
    }
    for us in marks
        .into_iter()
        .flat_map(|m| [m.min, m.trim, m.max])
        .flatten()
    {
        let x = rect.left() + fraction(us.clamp(0, i64::from(u16::MAX)) as u16) * rect.width();
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
            Stroke::new(2.0, MARKER),
        );
    }
    painter.rect_stroke(rect, 2.0, Stroke::new(1.0, GREY), StrokeKind::Inside);
}

/// A function bar: the value on `range`, drawn from the left edge for 0 at the bottom of the range and
/// from the centre for a signed range. Empty for `None`.
fn function_bar(ui: &mut egui::Ui, value: Option<i32>, range: (i32, i32)) {
    let (rect, _) = ui.allocate_exact_size(BAR_SIZE, Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, ui.visuals().extreme_bg_color);
    let span = (range.1 - range.0) as f32;
    let at =
        |v: i32| rect.left() + (v.clamp(range.0, range.1) - range.0) as f32 / span * rect.width();
    if let Some(v) = value {
        let (from, to) = if range.0 < 0 { (0, v) } else { (range.0, v) };
        let (a, b) = (at(from), at(to));
        let filled = Rect::from_min_max(
            egui::pos2(a.min(b), rect.top()),
            egui::pos2(a.max(b), rect.bottom()),
        );
        painter.rect_filled(filled, 2.0, Color32::from_rgb(70, 150, 220));
    }
    if range.0 < 0 {
        let x = at(0);
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
            Stroke::new(1.0, ui.visuals().weak_text_color()),
        );
    }
    painter.rect_stroke(rect, 2.0, Stroke::new(1.0, GREY), StrokeKind::Inside);
}

#[cfg(test)]
mod tests {
    use gcs_mavlink::services::messages::{Field, FieldValue, MessageKey};

    use super::*;

    fn entry(name: &str, fields: &[(&str, f64)], last_seen: f64) -> (MessageKey, MessageEntry) {
        (
            MessageKey {
                system_id: 1,
                component_id: 1,
                message_id: 65,
                instance: 0,
            },
            MessageEntry {
                name: name.into(),
                label: name.into(),
                count: 1,
                rate_hz: 5.0,
                last_seen: Duration::from_secs_f64(last_seen),
                fields: fields
                    .iter()
                    .map(|(n, v)| Field {
                        name: (*n).into(),
                        value: FieldValue::Number(*v),
                    })
                    .collect(),
            },
        )
    }

    fn store_with(fields: &[(&str, f64)], last_seen: f64) -> MessageStore {
        let mut store = MessageStore::default();
        let (key, e) = entry("RC_CHANNELS", fields, last_seen);
        store.entries.insert(key, e);
        store
    }

    fn secs(s: f64) -> Option<Duration> {
        Some(Duration::from_secs_f64(s))
    }

    #[test]
    fn the_bar_scale_is_800_to_2200_with_marks_at_1000_1500_2000() {
        assert_eq!(fraction(800), 0.0);
        assert_eq!(fraction(2200), 1.0);
        assert_eq!(fraction(1500), 0.5);
        assert_eq!(fraction(700), 0.0);
        assert_eq!(fraction(3000), 1.0);
        assert!((fraction(1000) - 200.0 / 1400.0).abs() < 1e-6);
        assert!((fraction(2000) - 1200.0 / 1400.0).abs() < 1e-6);
        assert_eq!(MARKS_US, [1000.0, 1500.0, 2000.0]);
    }

    #[test]
    fn the_channels_follow_chancount_and_an_unused_channel_is_blank() {
        let store = store_with(
            &[
                ("chan1_raw", 1000.0),
                ("chan2_raw", 1500.0),
                ("chan3_raw", 65_535.0),
                ("chan4_raw", 2000.0),
                ("chancount", 3.0),
                ("rssi", 127.0),
            ],
            9.0,
        );
        let sticks = Sticks::of(&store, secs(9.5));
        assert_eq!(sticks.shown(), 3);
        assert_eq!(&sticks.values[..4], [Some(1000), Some(1500), None, None]);
        assert_eq!(sticks.rssi, Some(127));
        assert!(sticks.fresh);
    }

    #[test]
    fn sixteen_bars_before_any_data_and_no_stick_data() {
        let sticks = Sticks::of(&MessageStore::default(), secs(5.0));
        assert_eq!(sticks.shown(), 16);
        assert!(sticks.values.iter().all(Option::is_none));
        assert_eq!(sticks.age_line().0, "no stick data");
        assert_eq!(
            Sticks::of(&MessageStore::default(), None).age_line().0,
            "no stick data"
        );
    }

    #[test]
    fn stick_data_older_than_two_seconds_is_no_stick_data_and_two_is_not() {
        let store = store_with(&[("chan1_raw", 1500.0), ("chancount", 16.0)], 10.0);
        let at = |now| Sticks::of(&store, secs(now));
        assert!(at(12.0).fresh);
        assert_eq!(at(10.25).age_line().0, "RC_CHANNELS 0.2 s ago");
        assert!(!at(12.001).fresh);
        assert_eq!(
            at(15.0).age_line().0,
            "no stick data (last RC_CHANNELS 5.0 s ago)"
        );
        assert!(!Sticks::of(&store, None).fresh);
    }

    #[test]
    fn link_quality_255_is_unknown() {
        assert_eq!(link_quality_text(Some(255)), "Link quality: unknown");
        assert_eq!(link_quality_text(None), "Link quality: unknown");
        assert_eq!(
            link_quality_text(Some(254)),
            "Link quality: rssi 254 of 254 (100 %)"
        );
        assert_eq!(
            link_quality_text(Some(0)),
            "Link quality: rssi 0 of 254 (0 %)"
        );
    }

    #[test]
    fn radio_status_is_shown_when_present() {
        assert_eq!(radio_status_text(None), "RADIO_STATUS: none received");
        let (_, e) = entry(
            "RADIO_STATUS",
            &[
                ("rssi", 200.0),
                ("remrssi", 190.0),
                ("txbuf", 90.0),
                ("noise", 10.0),
                ("remnoise", 12.0),
                ("rxerrors", 3.0),
                ("fixed", 1.0),
            ],
            1.0,
        );
        assert_eq!(
            radio_status_text(Some(&e)),
            "RADIO_STATUS: rssi 200, remrssi 190, txbuf 90 %, noise 10, remnoise 12, rxerrors 3, fixed 1"
        );
    }
}
