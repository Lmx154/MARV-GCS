//! The "Compass" panel of the Devices tab: the guided calibration of
//! `_plan/decision-compass-cal-20261120.md` (sections 2, 3, 4, 5, 6 and 8; link contract section 16).
//!
//! The numbers and the procedure are `gcs_magcal`; this file is the view and the wiring. It reads the calibration in
//! force and the GPS fix from the state, feeds a [`Session`] every HIGHRES_IMU `id` 0 message, the armed state of
//! HEARTBEAT, the changes of the watched parameters and the time (every frame: a lost link times out), and shows
//! its status. When the session ends accepted it is the commit (invariant I5): the thirteen writes of the plan one
//! at a time, each judged by its echo, then a read-back of the whole list compared with the plan; only after a good
//! read-back is the calibration shown as in use and Save offered. Nothing is written unless the session accepted.
//!
//! `Compass::update` runs every frame for every view (so samples are never lost while another tab shows);
//! `Compass::ui` runs it too, so a view driven by `ui` alone works. Commands are queued in an outbox that `ui`
//! sends, so a commit cannot send while the Devices tab is hidden; when it fails or ends the outbox is dropped, so no write leaves afterwards.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use eframe::egui::{self, Color32, Pos2, Sense, Shape, Stroke, vec2};
use gcs_magcal::accept::{ACCEPTED_MESSAGE, FITNESS_LIMIT_UT};
use gcs_magcal::heading::{self, GpsRecord, HeadingCheck, ImuSample};
use gcs_magcal::session::{
    Ending, Gps, NO_POSITION_MESSAGE, Outcome, ParamValue as PlanValue, Session, Status, Success,
    WATCHED_PARAMS, verify_read_back,
};
use gcs_magcal::turn::SECTORS;
use gcs_magcal::undo::Calibration;
use gcs_magcal::wmm;
use gcs_mavlink::dialect::MavCmd;
use gcs_mavlink::services::messages::MessageStore;
use gcs_mavlink::services::params::{FetchStatus, ParamTable, ParamValue, WriteStatus};
use gcs_mavlink::{Command, CommandStatus, VehicleState};

use super::ViewCtx;
use super::actions;
use super::compass_quad;
use super::presets::WRITE_WAIT;
use crate::status::{GREEN, GREY, RED, YELLOW};

/// The line of the unsaved-calibration bar once a guided calibration was read back good.
pub const GUIDED_UNSAVED_TEXT: &str = ACCEPTED_MESSAGE;
pub const GUIDED_BUTTON: &str = "Calibrate compass (guided)";
pub const QUICK_BUTTON: &str = "Quick calibration (best effort)";
pub const QUICK_NOTE: &str =
    "Heading errors of 15 deg and more were measured with this kind of calibration";
pub const CANCEL_BUTTON: &str = "Cancel compass calibration";
pub const ARMED_REASON: &str = "the vehicle is armed";
pub const RUNNING_REASON: &str = "a guided calibration is running";
pub const NOT_LOADED_MESSAGE: &str =
    "The compass parameters are not loaded yet: wait for the parameter list";
pub const HEADING_START: &str = "Start heading check";
pub const HEADING_STOP: &str = "Stop heading check";
pub const DISCARD_BUTTON: &str = "Discard";
/// How long the list may take to come back for the read-back.
const READ_BACK_WAIT: Duration = Duration::from_secs(30);
/// The most HIGHRES_IMU samples a heading check keeps (25 Hz: forty minutes).
const HEADING_MAX_SAMPLES: usize = 60_000;
const TILE: f32 = 78.0;
const QUAD_SIZE: egui::Vec2 = vec2(290.0, 230.0);
const QUAD_SCALE: f32 = 80.0;
const REPAINT: Duration = Duration::from_millis(100);
const IMU_FIELDS: [&str; 10] = [
    "time_usec",
    "xacc",
    "yacc",
    "zacc",
    "xgyro",
    "ygyro",
    "zgyro",
    "xmag",
    "ymag",
    "zmag",
];

/// What the vehicle's parameters say of its compass calibration (decision section 5, "State shown").
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CalState {
    NotCalibrated,
    BestEffort,
    BestEffortOlder,
    Guided,
}

impl CalState {
    pub fn word(self) -> &'static str {
        match self {
            CalState::NotCalibrated => "Not calibrated",
            CalState::BestEffort => "Best effort",
            CalState::BestEffortOlder => "best effort (older calibration)",
            CalState::Guided => "Guided",
        }
    }
}

/// The state from `MAG_CAL_TYPE`, `MAG_OFF*` and `MAG_FLD_UT`: type 2 with a field above zero is Guided; type 1 is
/// Best effort; type 0 (or a type 2 without a field) with any non-zero offset is the older best effort; else not
/// calibrated.
pub fn cal_state(cal_type: i32, offsets: [f64; 3], field_ut: f64) -> CalState {
    if cal_type == 2 && field_ut > 0.0 {
        CalState::Guided
    } else if cal_type == 1 {
        CalState::BestEffort
    } else if offsets.iter().any(|o| *o != 0.0) {
        CalState::BestEffortOlder
    } else {
        CalState::NotCalibrated
    }
}

fn number(table: &ParamTable, name: &str) -> Option<f64> {
    table.value(name).map(ParamValue::as_f64)
}

/// The line of the panel: the state, with the stored expected field and inclination when present; `None` while
/// the parameters are not loaded.
pub fn state_line(table: &ParamTable) -> Option<String> {
    let cal_type = number(table, "MAG_CAL_TYPE")?;
    let offsets = ["MAG_OFFX", "MAG_OFFY", "MAG_OFFZ"].map(|n| number(table, n).unwrap_or(0.0));
    let field = number(table, "MAG_FLD_UT").unwrap_or(0.0);
    let state = cal_state(cal_type as i32, offsets, field);
    let mut line = format!("Compass calibration: {}", state.word());
    if field > 0.0 {
        let inclination = number(table, "MAG_FLD_INC").unwrap_or(0.0);
        line.push_str(&format!(
            " (expected field {field:.1} uT, inclination {inclination:.1} deg)"
        ));
    }
    Some(line)
}

/// The decimal year of a Unix time, to a day (the model changes by nanotesla a day).
pub fn decimal_year(unix_seconds: f64) -> f64 {
    1970.0 + unix_seconds / (365.2425 * 86_400.0)
}

fn decimal_year_now() -> f64 {
    decimal_year(
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0.0, |d| d.as_secs_f64()),
    )
}

/// Whether the latest HEARTBEAT has `MAV_MODE_FLAG_SAFETY_ARMED`.
pub fn is_armed(store: &MessageStore) -> bool {
    store.entry("HEARTBEAT", 0).is_some_and(|e| {
        if let Some(mode) = e.number("base_mode") {
            (mode as u32) & 128 != 0
        } else {
            e.text("base_mode")
                .is_some_and(|t| t.contains("SAFETY_ARMED"))
        }
    })
}

/// `GPS_FIX_TYPE_3D_FIX` as 3.
fn fix_type_number(text: &str) -> Option<u8> {
    Some(match text.strip_prefix("GPS_FIX_TYPE_").unwrap_or(text) {
        "NO_GPS" => 0,
        "NO_FIX" => 1,
        "2D_FIX" => 2,
        "3D_FIX" => 3,
        "DGPS" => 4,
        "RTK_FLOAT" => 5,
        "RTK_FIXED" => 6,
        "STATIC" => 7,
        "PPP" => 8,
        _ => return None,
    })
}

/// The latest GPS_RAW_INT as the session's start condition; its age is the session time since it arrived.
fn gps_fix(store: &MessageStore, now: Duration) -> Option<Gps> {
    let e = store.entry("GPS_RAW_INT", 0)?;
    let fix_type = match e.text("fix_type") {
        Some(t) => fix_type_number(t)?,
        None => e.number("fix_type")? as u8,
    };
    Some(Gps {
        fix_type,
        age_s: now.saturating_sub(e.last_seen).as_secs_f64(),
        lat_deg: e.number("lat")? / 1e7,
        lon_deg: e.number("lon")? / 1e7,
        height_m: e.number("alt")? / 1000.0,
    })
}

/// The calibration in force, from the nine parameters; `None` when one is not in the table.
fn calibration_in_force(table: &ParamTable) -> Option<Calibration> {
    let v = |names: [&str; 3]| -> Option<[f64; 3]> {
        Some([
            number(table, names[0])?,
            number(table, names[1])?,
            number(table, names[2])?,
        ])
    };
    Some(Calibration {
        offsets: v(["MAG_OFFX", "MAG_OFFY", "MAG_OFFZ"])?,
        diag: v(["MAG_DIAGX", "MAG_DIAGY", "MAG_DIAGZ"])?,
        offdiag: v(["MAG_ODIAGXY", "MAG_ODIAGXZ", "MAG_ODIAGYZ"])?,
    })
}

/// One HIGHRES_IMU sample of `id` 0.
#[derive(Clone, Copy, Debug)]
struct Imu {
    time_usec: u64,
    accel: [f32; 3],
    gyro: [f32; 3],
    mag: [f32; 3],
}

/// The HIGHRES_IMU `id` 0 messages that arrived after the `seen` first ones, oldest first; the count is updated.
/// The series of the store keep every message for a minute, so a sample is missed only by a frame gap of that long.
fn new_imu(store: &MessageStore, seen: &mut u64) -> Vec<Imu> {
    let Some(entry) = store.entry("HIGHRES_IMU", 0) else {
        return Vec::new();
    };
    let fresh = entry.count.saturating_sub(*seen) as usize;
    *seen = entry.count;
    if fresh == 0 {
        return Vec::new();
    }
    let series: Vec<_> = IMU_FIELDS
        .iter()
        .filter_map(|f| store.series("HIGHRES_IMU", 0, f))
        .collect();
    if series.len() != IMU_FIELDS.len() {
        return Vec::new();
    }
    let available = series.iter().map(|s| s.points.len()).min().unwrap_or(0);
    let n = fresh.min(available);
    (0..n)
        .map(|k| {
            let at = |field: usize| {
                let s = &series[field].points;
                s[s.len() - n + k][1]
            };
            let v3 = |first: usize| [at(first) as f32, at(first + 1) as f32, at(first + 2) as f32];
            Imu {
                time_usec: at(0) as u64,
                accel: v3(1),
                gyro: v3(4),
                mag: v3(7),
            }
        })
        .collect()
}

/// How many PARAM_VALUE messages have been received.
fn param_values(store: &MessageStore) -> u64 {
    store.entry("PARAM_VALUE", 0).map_or(0, |e| e.count)
}

fn latest_accel(store: &MessageStore) -> Option<[f32; 3]> {
    let e = store.entry("HIGHRES_IMU", 0)?;
    Some([
        e.number("xacc")? as f32,
        e.number("yacc")? as f32,
        e.number("zacc")? as f32,
    ])
}

fn watched_values(table: &ParamTable) -> Vec<(&'static str, Option<f32>)> {
    WATCHED_PARAMS
        .iter()
        .map(|&n| (n, number(table, n).map(|v| v as f32)))
        .collect()
}

#[derive(Clone, Copy, PartialEq)]
enum Tone {
    Good,
    Bad,
    Info,
}

/// What the panel says of the last run.
struct Shown {
    tone: Tone,
    text: String,
    lines: Vec<String>,
    /// Offer Discard (the vehicle was left marked not calibrated).
    discard: bool,
}

impl Shown {
    fn new(tone: Tone, text: impl Into<String>) -> Self {
        Shown {
            tone,
            text: text.into(),
            lines: Vec::new(),
            discard: false,
        }
    }
}

struct Sampling {
    session: Session,
    snapshot: Vec<(&'static str, Option<f32>)>,
}

struct ReadBack {
    at: Duration,
    /// PARAM_VALUE messages received when the list was asked for: a list that came after it adds one per parameter.
    values_before: u64,
}

struct Commit {
    success: Success,
    /// Writes pushed so far.
    pushed: usize,
    /// Session time of the latest push.
    at: Duration,
    read_back: Option<ReadBack>,
}

/// After a read-back that differed or did not complete: `MAG_CAL_TYPE` 0 is written once more, so that type 2 stands
/// only for a set that was read back equal.
struct Reset {
    why: String,
    at: Duration,
}

enum Run {
    Idle,
    Reset(Box<Reset>),
    Sampling(Box<Sampling>),
    Commit(Box<Commit>),
}

#[derive(Default)]
struct HeadingState {
    running: bool,
    imu: Vec<ImuSample>,
    gps: Vec<GpsRecord>,
    imu_seen: u64,
    gps_seen: u64,
    position: Option<(f64, f64, f64)>,
    result: Option<String>,
}

pub struct Compass {
    run: Run,
    shown: Option<Shown>,
    imu_seen: u64,
    outbox: Vec<Command>,
    /// Session time at which a guided calibration was read back good: in use, not saved, until a save or a reload
    /// is accepted.
    unsaved: Option<Duration>,
    heading: HeadingState,
    year: f64,
}

impl Default for Compass {
    fn default() -> Self {
        Compass {
            run: Run::Idle,
            shown: None,
            imu_seen: 0,
            outbox: Vec::new(),
            unsaved: None,
            heading: HeadingState::default(),
            year: decimal_year_now(),
        }
    }
}

impl Compass {
    pub fn session_changed(&mut self) {
        *self = Compass::default();
    }

    /// Whether a guided calibration is in use on the vehicle and not saved (the bar offers Save and Discard).
    pub fn unsaved(&self) -> bool {
        self.unsaved.is_some()
    }

    pub fn update(&mut self, state: &VehicleState, now: Option<Duration>) {
        let Some(now) = now else {
            return;
        };
        self.collect_heading(state);
        self.advance_run(state, now);
        if let Some(at) = self.unsaved
            && state.command.as_ref().is_some_and(|r| {
                r.command == 245
                    && r.started >= at
                    && matches!(r.status, CommandStatus::Finished { accepted: true, .. })
            })
        {
            self.unsaved = None;
        }
    }

    fn start(&mut self, state: &VehicleState, now: Duration) {
        self.shown = None;
        let Some(calibration) = calibration_in_force(&state.params) else {
            self.shown = Some(Shown::new(Tone::Bad, NOT_LOADED_MESSAGE));
            return;
        };
        let gps = gps_fix(&state.messages, now);
        match Session::start(calibration, gps, self.year, now.as_secs_f64()) {
            Err(refusal) => self.shown = Some(Shown::new(Tone::Bad, refusal.message())),
            Ok(session) => {
                self.imu_seen = state
                    .messages
                    .entry("HIGHRES_IMU", 0)
                    .map_or(0, |e| e.count);
                self.run = Run::Sampling(Box::new(Sampling {
                    session,
                    snapshot: watched_values(&state.params),
                }));
            }
        }
    }

    fn advance_run(&mut self, state: &VehicleState, now: Duration) {
        match std::mem::replace(&mut self.run, Run::Idle) {
            Run::Idle => {}
            Run::Sampling(mut s) => {
                if is_armed(&state.messages) {
                    s.session.armed();
                }
                for (name, before) in &s.snapshot {
                    let current = number(&state.params, name).map(|v| v as f32);
                    if current.is_some() && current != *before {
                        s.session.param_changed(name);
                    }
                }
                for m in new_imu(&state.messages, &mut self.imu_seen) {
                    s.session.sample(m.time_usec, m.accel, m.gyro, m.mag);
                }
                s.session.tick(now.as_secs_f64());
                match s.session.outcome().clone() {
                    Outcome::Running => self.run = Run::Sampling(s),
                    Outcome::Accepted(success) => {
                        self.shown = None;
                        self.unsaved = None;
                        self.run = Run::Commit(Box::new(Commit {
                            success: *success,
                            pushed: 0,
                            at: now,
                            read_back: None,
                        }));
                        self.advance_run(state, now);
                    }
                    Outcome::Ended(ending) => {
                        let tone = if ending == Ending::Cancelled {
                            Tone::Info
                        } else {
                            Tone::Bad
                        };
                        self.shown = Some(Shown::new(tone, ending.message()));
                    }
                }
            }
            Run::Commit(c) => self.advance_commit(c, state, now),
            Run::Reset(r) => self.advance_reset(r, state, now),
        }
    }

    fn begin_reset(&mut self, why: String, now: Duration) {
        self.outbox.push(Command::SetParam {
            name: "MAG_CAL_TYPE".to_owned(),
            value: ParamValue::I32(0),
        });
        self.run = Run::Reset(Box::new(Reset { why, at: now }));
    }

    /// Judges the extra write of `MAG_CAL_TYPE` 0 by its echo, as the others.
    fn advance_reset(&mut self, r: Box<Reset>, state: &VehicleState, now: Duration) {
        let unverified = |this: &mut Compass, why: &str| {
            let mut shown = Shown::new(
                Tone::Bad,
                format!(
                    "Writing the calibration failed: {why}. The vehicle may be using an unverified compass calibration; Discard or a reboot restores the saved values."
                ),
            );
            shown.discard = true;
            this.shown = Some(shown);
            this.unsaved = None;
            this.outbox.clear();
            this.run = Run::Idle;
        };
        match state.params.writes.get("MAG_CAL_TYPE") {
            Some(WriteStatus::Pending { .. }) => {
                self.run = Run::Reset(r);
                return;
            }
            Some(WriteStatus::Refused { .. } | WriteStatus::Unanswered { .. }) => {
                return unverified(self, &r.why);
            }
            None => {}
        }
        if number(&state.params, "MAG_CAL_TYPE") == Some(0.0) {
            return self.fail(r.why, false);
        }
        if now.saturating_sub(r.at) > WRITE_WAIT + WRITE_WAIT {
            return unverified(self, &r.why);
        }
        self.run = Run::Reset(r);
    }

    fn fail(&mut self, why: String, first_write: bool) {
        let mut shown = if first_write {
            Shown::new(
                Tone::Bad,
                format!("The compass was not changed: {why}. Nothing was written."),
            )
        } else {
            Shown::new(
                Tone::Bad,
                format!(
                    "Writing the calibration failed: {why}. The vehicle's compass is marked not calibrated (type 0); Discard or a reboot restores the saved values."
                ),
            )
        };
        shown.discard = !first_write;
        self.shown = Some(shown);
        self.unsaved = None;
        self.outbox.clear();
        self.run = Run::Idle;
    }

    fn advance_commit(&mut self, mut c: Box<Commit>, state: &VehicleState, now: Duration) {
        let plan_len = c.success.plan.len();
        if let Some(rb) = &c.read_back {
            let table = &state.params;
            // The list was asked for after the last write, and the table is emptied when it starts: a value in the
            // table came from that list, and the PARAM_VALUE count says a whole list arrived after the request.
            let values = param_values(&state.messages).saturating_sub(rb.values_before);
            match &table.fetch {
                FetchStatus::Failed(why) => {
                    let why = format!("the read-back failed: {why}");
                    return self.begin_reset(why, now);
                }
                FetchStatus::Complete if values >= table.params.len().max(1) as u64 => {
                    let read: Vec<(&str, f32)> = c
                        .success
                        .plan
                        .iter()
                        .filter_map(|(n, _)| Some((*n, number(table, n)? as f32)))
                        .collect();
                    return match verify_read_back(&c.success.plan, &read) {
                        Err(why) => self.begin_reset(format!("the read-back differs: {why}"), now),
                        Ok(()) => self.succeed(&c.success, now),
                    };
                }
                _ => {}
            }
            if now.saturating_sub(rb.at) > READ_BACK_WAIT {
                return self.begin_reset("read-back did not complete".to_string(), now);
            }
            self.run = Run::Commit(c);
            return;
        }
        if c.pushed > 0 {
            let (name, value) = c.success.plan[c.pushed - 1];
            match state.params.writes.get(name) {
                Some(WriteStatus::Pending { .. }) => {
                    self.run = Run::Commit(c);
                    return;
                }
                Some(WriteStatus::Refused { in_effect, .. }) => {
                    let why = format!(
                        "{name} was refused (the vehicle uses {})",
                        in_effect.as_f64()
                    );
                    return self.fail(why, c.pushed == 1);
                }
                Some(WriteStatus::Unanswered { .. }) => {
                    return self.fail(format!("{name} got no answer"), c.pushed == 1);
                }
                None => {}
            }
            let confirmed = number(&state.params, name) == Some(f64::from(value.as_f32()));
            if !confirmed {
                if now.saturating_sub(c.at) > WRITE_WAIT + WRITE_WAIT {
                    return self.fail(format!("{name} got no answer"), c.pushed == 1);
                }
                self.run = Run::Commit(c);
                return;
            }
        }
        if c.pushed < plan_len {
            let (name, value) = c.success.plan[c.pushed];
            self.outbox.push(Command::SetParam {
                name: name.to_owned(),
                value: match value {
                    PlanValue::F32(v) => ParamValue::F32(v),
                    PlanValue::I32(v) => ParamValue::I32(v),
                },
            });
            c.pushed += 1;
            c.at = now;
        } else {
            self.outbox.push(Command::RefreshParams);
            c.read_back = Some(ReadBack {
                at: now,
                values_before: param_values(&state.messages),
            });
        }
        self.run = Run::Commit(c);
    }

    fn succeed(&mut self, success: &Success, now: Duration) {
        let a = &success.accepted;
        let o = a.calibration.offsets;
        let mut shown = Shown::new(Tone::Good, ACCEPTED_MESSAGE);
        shown.lines = vec![
            format!(
                "Fitness {:.2} uT (limit {FITNESS_LIMIT_UT}), {} samples",
                a.fitness_ut, success.samples
            ),
            format!("Offsets {:.1} {:.1} {:.1} uT", o[0], o[1], o[2]),
            format!(
                "Inclination measured {:.1} deg, model {:.1} deg; field {:.1} uT",
                a.measured_inclination_deg, a.inclination_deg, a.field_ut
            ),
        ];
        self.shown = Some(shown);
        self.unsaved = Some(now);
        self.run = Run::Idle;
    }

    fn running(&self) -> bool {
        !matches!(self.run, Run::Idle)
    }

    fn collect_heading(&mut self, state: &VehicleState) {
        let h = &mut self.heading;
        if !h.running {
            return;
        }
        if h.imu.len() < HEADING_MAX_SAMPLES {
            for m in new_imu(&state.messages, &mut h.imu_seen) {
                h.imu.push(ImuSample {
                    time_usec: m.time_usec,
                    accel: m.accel,
                    mag: m.mag,
                });
            }
        }
        if let Some(e) = state.messages.entry("GPS_RAW_INT", 0)
            && e.count != h.gps_seen
        {
            h.gps_seen = e.count;
            let known = |f: &str| e.number(f).filter(|v| *v != f64::from(u16::MAX));
            if let (Some(t), Some(cog), Some(vel)) =
                (e.number("time_usec"), known("cog"), known("vel"))
            {
                h.gps.push(GpsRecord {
                    time_usec: t as u64,
                    course_deg: cog / 100.0,
                    speed_mps: vel / 100.0,
                });
            }
            if let (Some(lat), Some(lon), Some(alt)) =
                (e.number("lat"), e.number("lon"), e.number("alt"))
            {
                h.position = Some((lat / 1e7, lon / 1e7, alt / 1000.0));
            }
        }
    }

    fn heading_text(&self) -> String {
        let h = &self.heading;
        let Some((lat, lon, alt)) = h.position else {
            return NO_POSITION_MESSAGE.to_string();
        };
        match wmm::field(lat, lon, alt, self.year) {
            Err(e) => format!("The Earth's field model cannot be used: {e}"),
            Ok(f) => {
                let check = heading::check(&h.imu, &h.gps, f.declination_deg);
                let detail = match &check {
                    HeadingCheck::Measured {
                        median_deg,
                        spread_deg,
                        records,
                        ..
                    } => format!(
                        " (median {median_deg:+.1} deg, spread {spread_deg:.1} deg, {records} records, declination {:+.1} deg)",
                        f.declination_deg
                    ),
                    HeadingCheck::NotEnough { .. } => String::new(),
                };
                format!("Heading check: {}{detail}", check.message())
            }
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx, gate: Option<&'static str>) {
        self.update(cx.state, cx.now);
        let now = cx.now.unwrap_or_default();
        let state = cx.state;
        let armed = is_armed(&state.messages);
        let busy = self.running();
        let guided_gate = gate
            .or(armed.then_some(ARMED_REASON))
            .or(busy.then_some(RUNNING_REASON));
        ui.separator();
        ui.strong("Compass");
        ui.label(state_line(&state.params).unwrap_or_else(|| {
            "Compass calibration: not known until the parameter list is loaded".to_string()
        }));
        ui.horizontal(|ui| {
            if actions::button(ui, GUIDED_BUTTON, guided_gate) {
                self.start(state, now);
            }
            if actions::button(ui, QUICK_BUTTON, guided_gate) {
                cx.commands.push(Command::SendCommand {
                    command: MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
                    params: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                });
            }
            if matches!(self.run, Run::Sampling(_)) && ui.button(CANCEL_BUTTON).clicked() {
                if let Run::Sampling(s) = &mut self.run {
                    s.session.cancel();
                }
                self.update(state, cx.now);
            }
        });
        ui.label(QUICK_NOTE);
        if let Some(reason) = guided_gate {
            ui.colored_label(YELLOW, format!("{GUIDED_BUTTON} is off: {reason}"));
        }

        match &self.run {
            Run::Sampling(s) => run_view(ui, &s.session.status(now.as_secs_f64()), state),
            Run::Commit(c) => {
                let text = if c.read_back.is_some() {
                    "Reading the calibration back from the vehicle".to_string()
                } else {
                    format!(
                        "Writing the calibration to the vehicle: {} of {}",
                        c.pushed,
                        c.success.plan.len()
                    )
                };
                ui.colored_label(YELLOW, text);
            }
            Run::Reset(_) => {
                ui.colored_label(YELLOW, "Marking the compass not calibrated (type 0)");
            }
            Run::Idle => {}
        }
        if let Some(shown) = &self.shown {
            let colour = match shown.tone {
                Tone::Good => GREEN,
                Tone::Bad => RED,
                Tone::Info => GREY,
            };
            if shown.tone != Tone::Good {
                ui.colored_label(colour, &shown.text);
            }
            for line in &shown.lines {
                ui.label(line);
            }
            if shown.discard && actions::button(ui, DISCARD_BUTTON, gate) {
                cx.commands.push(Command::SendCommand {
                    command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
                    params: [0.0; 7],
                });
            }
        }
        self.heading_ui(ui);
        cx.commands.append(&mut self.outbox);
        if busy || self.heading.running {
            ui.ctx().request_repaint_after(REPAINT);
        }
    }

    fn heading_ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Heading check: walk a straight line with the vehicle");
            if self.heading.running {
                if ui.button(HEADING_STOP).clicked() {
                    self.heading.running = false;
                    self.heading.result = Some(self.heading_text());
                }
            } else if ui.button(HEADING_START).clicked() {
                self.heading = HeadingState {
                    running: true,
                    ..HeadingState::default()
                };
                self.heading.imu_seen = u64::MAX;
                self.heading.gps_seen = u64::MAX;
            }
        });
        if self.heading.running {
            ui.label(self.heading_text());
        } else if let Some(text) = &self.heading.result {
            ui.label(text);
        }
    }
}

/// The picture and the status line while the session runs.
fn run_view(ui: &mut egui::Ui, status: &Status, state: &VehicleState) {
    ui.horizontal_top(|ui| {
        let (response, painter) = ui.allocate_painter(QUAD_SIZE, Sense::hover());
        if let Some(target) = status.target {
            let a = target.expected_accel();
            if let Some(down) = compass_quad::down_from_accel(a.map(|v| v as f32)) {
                compass_quad::draw(
                    &painter,
                    response.rect,
                    QUAD_SCALE,
                    &compass_quad::quad(down, true),
                    true,
                );
            }
        }
        if let Some(down) = latest_accel(&state.messages).and_then(compass_quad::down_from_accel) {
            compass_quad::draw(
                &painter,
                response.rect,
                QUAD_SCALE,
                &compass_quad::quad(down, false),
                false,
            );
        }
        egui::Grid::new("compass_tiles").show(ui, |ui| {
            for (i, p) in status.poses.iter().enumerate() {
                ui.vertical(|ui| {
                    let (rect, _) = ui.allocate_exact_size(vec2(TILE, TILE), Sense::hover());
                    let painter = ui.painter_at(rect);
                    draw_ring(&painter, rect, &p.visited);
                    let outline = if Some(p.pose) == status.target {
                        Some(Stroke::new(2.0, YELLOW))
                    } else if Some(p.pose) == status.recognised {
                        Some(Stroke::new(1.0, GREY))
                    } else {
                        None
                    };
                    if let Some(stroke) = outline {
                        painter.rect_stroke(rect, 2.0, stroke, egui::StrokeKind::Inside);
                    }
                    let done = if p.complete { " done" } else { "" };
                    ui.label(format!(
                        "{} {}/{SECTORS}{done}",
                        p.pose.name(),
                        p.visited_count
                    ));
                });
                if i % 3 == 2 {
                    ui.end_row();
                }
            }
        });
    });
    ui.label(&status.instruction);
    ui.label(format!(
        "{:.0} s left for this pose, {} samples kept",
        status.seconds_left, status.samples_kept
    ));
    if let Some(p) = status.target {
        ui.label(format!("Next: {p}"));
    }
}

/// The ring of one pose: twelve sectors from the top, clockwise, filled when visited.
fn draw_ring(painter: &egui::Painter, rect: egui::Rect, visited: &[bool; SECTORS]) {
    let centre = rect.center();
    let outer = rect.width().min(rect.height()) * 0.5 - 2.0;
    let inner = outer * 0.55;
    let step = std::f32::consts::TAU / SECTORS as f32;
    for (i, &seen) in visited.iter().enumerate() {
        let (from, to) = (
            -std::f32::consts::FRAC_PI_2 + step * i as f32,
            -std::f32::consts::FRAC_PI_2 + step * (i + 1) as f32,
        );
        let arc = |radius: f32, t: f32| {
            Pos2::new(centre.x + radius * t.cos(), centre.y + radius * t.sin())
        };
        let mut points = Vec::new();
        for k in 0..=4 {
            points.push(arc(outer, from + (to - from) * k as f32 / 4.0));
        }
        for k in (0..=4).rev() {
            points.push(arc(inner, from + (to - from) * k as f32 / 4.0));
        }
        let fill = if seen { GREEN } else { Color32::from_gray(55) };
        painter.add(Shape::convex_polygon(
            points,
            fill,
            Stroke::new(1.0, Color32::from_gray(20)),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_three_states_and_the_older_best_effort() {
        assert_eq!(cal_state(0, [0.0; 3], 0.0), CalState::NotCalibrated);
        assert_eq!(
            cal_state(0, [0.0, 0.0, 1.5], 0.0),
            CalState::BestEffortOlder
        );
        assert_eq!(cal_state(1, [0.0; 3], 0.0), CalState::BestEffort);
        assert_eq!(cal_state(1, [3.0, 0.0, 0.0], 0.0), CalState::BestEffort);
        assert_eq!(cal_state(2, [1.0, 2.0, 3.0], 48.0), CalState::Guided);
        // Type 2 without an expected field is not Guided.
        assert_eq!(cal_state(2, [0.0; 3], 0.0), CalState::NotCalibrated);
        assert_eq!(
            cal_state(2, [1.0, 0.0, 0.0], 0.0),
            CalState::BestEffortOlder
        );
    }

    #[test]
    fn the_decimal_year_of_a_unix_time() {
        // 2026-01-01T00:00:00Z.
        assert!((decimal_year(1_767_225_600.0) - 2026.0).abs() < 0.01);
        assert!((decimal_year(0.0) - 1970.0).abs() < 1e-9);
    }

    #[test]
    fn fix_types_by_name() {
        assert_eq!(fix_type_number("GPS_FIX_TYPE_3D_FIX"), Some(3));
        assert_eq!(fix_type_number("GPS_FIX_TYPE_NO_FIX"), Some(1));
        assert_eq!(fix_type_number("GPS_FIX_TYPE_RTK_FIXED"), Some(6));
        assert_eq!(fix_type_number("nonsense"), None);
    }
}
