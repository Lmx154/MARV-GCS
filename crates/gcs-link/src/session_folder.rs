//! Session folders: one `.tlog` becomes a folder that can be read with `csv`, `json`, `awk` and `grep`
//! (no MAVLink library): `session.tlog`, `session.json` and one `<MESSAGE>.csv` per message type.
//! The layout and the schema of `session.json` are in the README ("Sessions").
//!
//! This is file handling on top of the MAVLink layer, so it lives here and not in `gcs-mavlink`
//! (which only ever sees a `Read` / `Write` pair) and not in an application (both the window and
//! `marv-cli` call it). Privacy: no latitude or longitude leaves this module except inside the
//! copied `session.tlog` (see [`is_position_field`] and [`POSITION_ONLY`]).

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use gcs_mavlink::dialect::{MavAutopilot, MavCmd, MavProtocolCapability};
use gcs_mavlink::services::messages::INSTANCE_FIELDS;
use gcs_mavlink::services::params::{ParamEncoding, ParamType, ParamValue, decode};
use gcs_mavlink::tlog::TlogReader;
use gcs_mavlink::{Body, Deframer, Frame, Message, SessionConfig};
use serde_json::{Map, Number, Value, json};

/// The version of the `session.json` layout.
pub const SCHEMA_VERSION: u64 = 1;
/// A silence longer than this between two frames from the vehicle is a link loss.
pub const LINK_GAP_S: f64 = 3.0;
const TLOG_NAME: &str = "session.tlog";
const JSON_NAME: &str = "session.json";

/// Messages that are only positions; they are not written (and are listed as skipped).
/// `MISSION_ITEM*` and `COMMAND_INT` carry coordinates in `x` and `y`.
pub const POSITION_ONLY: &[&str] = &[
    "GPS_GLOBAL_ORIGIN",
    "SET_GPS_GLOBAL_ORIGIN",
    "HOME_POSITION",
    "SET_HOME_POSITION",
    "MISSION_ITEM",
    "MISSION_ITEM_INT",
    "COMMAND_INT",
];

/// Units of the fields of the messages the station uses, from the MAVLink and marv-dialect
/// definitions (the `mavlink` crate does not expose them at run time). A field that is not listed
/// has no unit in the definitions; `RC_CHANNELS.chanN_raw` is handled in [`unit_of`].
const UNITS: &[(&str, &[(&str, &str)])] = &[
    (
        "HIGHRES_IMU",
        &[
            ("time_usec", "us"),
            ("xacc", "m/s/s"),
            ("yacc", "m/s/s"),
            ("zacc", "m/s/s"),
            ("xgyro", "rad/s"),
            ("ygyro", "rad/s"),
            ("zgyro", "rad/s"),
            ("xmag", "gauss"),
            ("ymag", "gauss"),
            ("zmag", "gauss"),
            ("abs_pressure", "hPa"),
            ("diff_pressure", "hPa"),
            ("temperature", "degC"),
        ],
    ),
    (
        "SCALED_IMU2",
        &[
            ("time_boot_ms", "ms"),
            ("xacc", "mG"),
            ("yacc", "mG"),
            ("zacc", "mG"),
            ("xgyro", "mrad/s"),
            ("ygyro", "mrad/s"),
            ("zgyro", "mrad/s"),
            ("xmag", "mgauss"),
            ("ymag", "mgauss"),
            ("zmag", "mgauss"),
            ("temperature", "cdegC"),
        ],
    ),
    (
        "SCALED_PRESSURE",
        &[
            ("time_boot_ms", "ms"),
            ("press_abs", "hPa"),
            ("press_diff", "hPa"),
            ("temperature", "cdegC"),
            ("temperature_press_diff", "cdegC"),
        ],
    ),
    (
        "SYS_STATUS",
        &[
            ("load", "d%"),
            ("voltage_battery", "mV"),
            ("current_battery", "cA"),
            ("battery_remaining", "%"),
            ("drop_rate_comm", "c%"),
        ],
    ),
    (
        "GPS_RAW_INT",
        &[
            ("time_usec", "us"),
            ("alt", "mm"),
            ("vel", "cm/s"),
            ("cog", "cdeg"),
            ("alt_ellipsoid", "mm"),
            ("h_acc", "mm"),
            ("v_acc", "mm"),
            ("vel_acc", "mm/s"),
            ("hdg_acc", "degE5"),
            ("yaw", "cdeg"),
        ],
    ),
    ("RC_CHANNELS", &[("time_boot_ms", "ms")]),
    (
        "ESC_STATUS",
        &[
            ("time_usec", "us"),
            ("rpm", "rpm"),
            ("voltage", "V"),
            ("current", "A"),
        ],
    ),
    ("ESC_INFO", &[("time_usec", "us"), ("temperature", "cdegC")]),
    (
        "MARV_DEVICE_STATUS",
        &[("time_boot_ms", "ms"), ("last_sample_age_ms", "ms")],
    ),
];

/// The unit of a CSV column (an array element `name_3` has the unit of `name`), if the definitions give one.
pub fn unit_of(message: &str, column: &str) -> Option<&'static str> {
    if message == "RC_CHANNELS" && column.starts_with("chan") && column.ends_with("_raw") {
        return Some("us");
    }
    let (_, fields) = UNITS.iter().find(|(name, _)| *name == message)?;
    let find = |field: &str| fields.iter().find(|(f, _)| *f == field).map(|(_, u)| *u);
    find(column).or_else(|| {
        let (base, index) = column.rsplit_once('_')?;
        index
            .bytes()
            .all(|b| b.is_ascii_digit())
            .then(|| find(base))
            .flatten()
    })
}

/// Whether a field holds a latitude or a longitude (they never reach a CSV or the JSON).
pub fn is_position_field(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    matches!(n.as_str(), "lat" | "lon" | "latitude" | "longitude")
        || ["_lat", "_lon", "_latitude", "_longitude"]
            .iter()
            .any(|suffix| n.ends_with(suffix))
        || n.starts_with("lat_")
        || n.starts_with("lon_")
}

/// The integer value of the bit-flag fields of a message, which `serde` shows as `A | B` text. The
/// list is every field of the dialect whose type is a `bitflags` type (they are CSV cells as numbers).
macro_rules! flag_fields {
    ($($message:ident { $($field:ident),+ }),+ $(,)?) => {
        #[allow(deprecated)] // the dialect still has a few superseded messages
        fn flag_values(message: &Message) -> Vec<(&'static str, u64)> {
            match message {
                $(Message::$message(d) => vec![$((stringify!($field), u64::from(d.$field.bits()))),+],)+
                _ => Vec::new(),
            }
        }
    };
}

flag_fields! {
    ADSB_VEHICLE { flags },
    AIRSPEED { flags },
    AIS_VESSEL { flags },
    ATTITUDE_TARGET { type_mask },
    AUTOPILOT_STATE_FOR_GIMBAL_DEVICE { estimator_status },
    AUTOPILOT_VERSION { capabilities },
    AVAILABLE_MODES { properties },
    BATTERY_STATUS { fault_bitmask },
    CAMERA_IMAGE_CAPTURED { capture_result },
    CAMERA_INFORMATION { flags },
    CAMERA_TRACKING_GEO_STATUS { tracking_status },
    CAMERA_TRACKING_IMAGE_STATUS { tracking_status, target_data },
    COMPONENT_INFORMATION_BASIC { capabilities },
    CURRENT_EVENT_SEQUENCE { flags },
    ESTIMATOR_STATUS { flags },
    GENERATOR_STATUS { status },
    GIMBAL_DEVICE_ATTITUDE_STATUS { failure_flags, flags },
    GIMBAL_DEVICE_INFORMATION { cap_flags, cap_flags2 },
    GIMBAL_DEVICE_SET_ATTITUDE { flags },
    GIMBAL_MANAGER_INFORMATION { cap_flags },
    GIMBAL_MANAGER_SET_ATTITUDE { flags },
    GIMBAL_MANAGER_SET_MANUAL_CONTROL { flags },
    GIMBAL_MANAGER_SET_PITCHYAW { flags },
    GIMBAL_MANAGER_STATUS { flags },
    GLOBAL_POSITION_SENSOR { flags },
    GPS_INPUT { ignore_flags },
    HEARTBEAT { base_mode },
    HIGHRES_IMU { fields_updated },
    HIGH_LATENCY { base_mode },
    HIGH_LATENCY2 { failure_flags },
    HIL_ACTUATOR_CONTROLS { flags, mode },
    HIL_SENSOR { fields_updated },
    ILLUMINATOR_STATUS { error_status },
    LANDING_TARGET { position_valid },
    ONBOARD_COMPUTER_STATUS { status_flags },
    POSITION_TARGET_GLOBAL_INT { type_mask },
    POSITION_TARGET_LOCAL_NED { type_mask },
    POWER_STATUS { flags },
    SERIAL_CONTROL { flags },
    SET_ATTITUDE_TARGET { type_mask },
    SET_MODE { base_mode },
    SET_POSITION_TARGET_GLOBAL_INT { type_mask },
    SET_POSITION_TARGET_LOCAL_NED { type_mask },
    STORAGE_INFORMATION { storage_usage },
    SYS_STATUS { onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, onboard_control_sensors_present_extended, onboard_control_sensors_enabled_extended, onboard_control_sensors_health_extended },
    UTM_GLOBAL_POSITION { flags },
    VIDEO_STREAM_INFORMATION { flags },
    VIDEO_STREAM_STATUS { flags },
    WINCH_STATUS { status },
}

/// A mark the user made, at a Unix time.
#[derive(Debug, Clone, PartialEq)]
pub struct UserMark {
    pub time_us: u64,
    pub text: String,
}

#[derive(Debug, Clone, Default)]
pub struct SessionOptions {
    /// The name; it ends up in the folder name (made file-safe) and in `session.json` as given.
    pub name: String,
    /// The parameters at the start; without it the first `PARAM_VALUE` of each name stands for them.
    pub parameters: Option<Vec<(String, f64)>>,
    pub marks: Vec<UserMark>,
}

/// What was written.
#[derive(Debug, Clone, PartialEq)]
pub struct Exported {
    pub folder: PathBuf,
    /// Each CSV and its number of rows, in message order.
    pub files: Vec<(String, u64)>,
    pub marks: usize,
    /// Messages not written: the message and why.
    pub skipped: Vec<(String, String)>,
}

/// One line of `session list`.
#[derive(Debug, Clone, PartialEq)]
pub struct SessionInfo {
    pub folder: String,
    pub name: String,
    pub start_utc: String,
    pub duration_s: f64,
    pub marks: usize,
}

// ---- time ----

/// Year, month and day of a day count since 1970-01-01 (proleptic Gregorian calendar).
fn civil_from_days(days: u64) -> (u64, u64, u64) {
    let z = days + 719_468;
    let era = z / 146_097;
    let day_of_era = z % 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let shifted_month = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * shifted_month + 2) / 5 + 1;
    let month = if shifted_month < 10 {
        shifted_month + 3
    } else {
        shifted_month - 9
    };
    (era * 400 + year_of_era + u64::from(month <= 2), month, day)
}

/// `2026-11-17T17:47:08.123Z`
fn iso_utc(time_us: u64) -> String {
    let secs = time_us / 1_000_000;
    let (year, month, day) = civil_from_days(secs / 86_400);
    let rest = secs % 86_400;
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{:03}Z",
        rest / 3600,
        rest / 60 % 60,
        rest % 60,
        time_us % 1_000_000 / 1000
    )
}

/// `2026-11-17_17-47-08Z`, the style of the recordings' names.
fn folder_time(time_us: u64) -> String {
    let secs = time_us / 1_000_000;
    let (year, month, day) = civil_from_days(secs / 86_400);
    let rest = secs % 86_400;
    format!(
        "{year:04}-{month:02}-{day:02}_{:02}-{:02}-{:02}Z",
        rest / 3600,
        rest / 60 % 60,
        rest % 60
    )
}

fn file_safe(name: &str) -> String {
    let safe: String = name
        .chars()
        .take(64)
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if safe.is_empty() {
        "session".to_owned()
    } else {
        safe
    }
}

/// Rounded to 6 significant digits, so the JSON shows no arithmetic noise.
fn sig(value: f64) -> f64 {
    if !value.is_finite() {
        return value;
    }
    format!("{value:.5e}").parse().unwrap_or(value)
}

// ---- statistics ----

#[derive(Debug, Clone, Copy)]
struct Stat {
    n: u64,
    mean: f64,
    m2: f64,
    min: f64,
    max: f64,
}

impl Stat {
    fn new() -> Self {
        Self {
            n: 0,
            mean: 0.0,
            m2: 0.0,
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
        }
    }

    fn add(&mut self, value: f64) {
        self.n += 1;
        let delta = value - self.mean;
        self.mean += delta / self.n as f64;
        self.m2 += delta * (value - self.mean);
        self.min = self.min.min(value);
        self.max = self.max.max(value);
    }

    fn json(&self) -> Value {
        let sd = if self.n > 1 {
            (self.m2 / (self.n - 1) as f64).sqrt()
        } else {
            0.0
        };
        json!({"mean": sig(self.mean), "sd": sig(sd), "min": sig(self.min), "max": sig(self.max)})
    }
}

#[derive(Debug, Default)]
struct Group {
    count: u64,
    first_s: f64,
    last_s: f64,
    fields: Vec<(String, Stat)>,
}

impl Group {
    fn add(&mut self, t_s: f64, cells: &[(String, Cell)], time_column: Option<&str>) {
        if self.count == 0 {
            self.first_s = t_s;
        }
        self.count += 1;
        self.last_s = t_s;
        for (name, cell) in cells {
            let Some(number) = cell.number.filter(|n| n.is_finite()) else {
                continue;
            };
            if Some(name.as_str()) == time_column {
                continue;
            }
            let at = match self.fields.iter().position(|(n, _)| n == name) {
                Some(at) => at,
                None => {
                    self.fields.push((name.clone(), Stat::new()));
                    self.fields.len() - 1
                }
            };
            self.fields[at].1.add(number);
        }
    }

    fn json(&self) -> Value {
        let rate = (self.count > 1 && self.last_s > self.first_s)
            .then(|| sig((self.count - 1) as f64 / (self.last_s - self.first_s)));
        let fields: Map<String, Value> = self
            .fields
            .iter()
            .map(|(name, stat)| (name.clone(), stat.json()))
            .collect();
        json!({"count": self.count, "rate_hz": rate, "fields": fields})
    }
}

fn groups_json(groups: &BTreeMap<String, Group>) -> Value {
    Value::Object(groups.iter().map(|(k, g)| (k.clone(), g.json())).collect())
}

// ---- cells and CSV ----

#[derive(Debug, Clone)]
struct Cell {
    text: String,
    number: Option<f64>,
}

fn number_cell(n: &Number) -> Cell {
    if let Some(i) = n.as_i64() {
        return Cell {
            text: i.to_string(),
            number: Some(i as f64),
        };
    }
    if let Some(u) = n.as_u64() {
        return Cell {
            text: u.to_string(),
            number: Some(u as f64),
        };
    }
    let f = n.as_f64().unwrap_or(f64::NAN);
    let single = f as f32;
    let text = if f64::from(single) == f {
        single.to_string()
    } else {
        f.to_string()
    };
    Cell {
        text,
        number: Some(f),
    }
}

/// The columns a field gives: arrays become `name_0`, `name_1`, ...; the position fields give none.
fn flatten(name: &str, value: &Value, out: &mut Vec<(String, Cell)>) {
    match value {
        Value::Array(items) => {
            for (index, item) in items.iter().enumerate() {
                flatten(&format!("{name}_{index}"), item, out);
            }
        }
        Value::Number(n) => out.push((name.to_owned(), number_cell(n))),
        Value::Bool(b) => out.push((
            name.to_owned(),
            Cell {
                text: u8::from(*b).to_string(),
                number: Some(f64::from(u8::from(*b))),
            },
        )),
        Value::Null => out.push((
            name.to_owned(),
            Cell {
                text: "nan".to_owned(),
                number: None,
            },
        )),
        Value::String(text) => out.push((
            name.to_owned(),
            Cell {
                text: text.clone(),
                number: None,
            },
        )),
        Value::Object(map) => {
            let text = match map.get("type") {
                Some(Value::String(variant)) if map.len() == 1 => variant.clone(),
                _ => value.to_string(),
            };
            out.push((name.to_owned(), Cell { text, number: None }));
        }
    }
}

fn csv_text(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_owned()
    }
}

struct Table {
    writer: BufWriter<File>,
    header: Vec<String>,
    rows: u64,
}

impl Table {
    fn row(&mut self, t_s: f64, cells: &[(String, Cell)]) -> io::Result<()> {
        let mut line = format!("{t_s:.6}");
        let same = cells.len() == self.header.len() - 1
            && cells
                .iter()
                .zip(&self.header[1..])
                .all(|((n, _), h)| n == h);
        if same {
            for (_, cell) in cells {
                line.push(',');
                line.push_str(&csv_text(&cell.text));
            }
        } else {
            for column in &self.header[1..] {
                line.push(',');
                if let Some((_, cell)) = cells.iter().find(|(n, _)| n == column) {
                    line.push_str(&csv_text(&cell.text));
                }
            }
        }
        line.push('\n');
        self.rows += 1;
        self.writer.write_all(line.as_bytes())
    }
}

// ---- the pass over the recording ----

struct Mark {
    t_s: f64,
    kind: &'static str,
    text: String,
}

struct ParamEvent {
    t_s: f64,
    name: String,
    kind: Option<ParamType>,
    raw: [u8; 4],
}

struct Pass<'a> {
    start_us: u64,
    folder: &'a Path,
    station_id: u8,
    tables: BTreeMap<String, Table>,
    units: BTreeMap<String, BTreeMap<String, &'static str>>,
    skipped: BTreeMap<String, (u64, &'static str)>,
    overall: BTreeMap<String, Group>,
    /// Boundaries between intervals: the user marks' times, ascending.
    bounds: Vec<f64>,
    intervals: Vec<BTreeMap<String, Group>>,
    marks: Vec<Mark>,
    params: Vec<ParamEvent>,
    deframer: Deframer,
    station_frames: u64,
    /// The latest `COMMAND_LONG` the station sent, by command: the seven parameters.
    sent: BTreeMap<String, [f32; 7]>,
    vehicle_id: Option<u8>,
    first_sender: Option<u8>,
    ardupilot: bool,
    capabilities: Option<MavProtocolCapability>,
    firmware: Option<String>,
    last_rx_s: Option<f64>,
    last_t_s: f64,
}

/// The motor test the station sent (link contract section 15): param1 the motor, param3 the throttle in
/// percent (0 is a stop), param4 the duration in seconds.
fn describe_motor_test(p: &[f32; 7]) -> String {
    if p[2] == 0.0 {
        format!("motor {}, stop", p[0])
    } else {
        format!("motor {}, {:.2} %, {:.1} s", p[0], p[2], p[3])
    }
}

fn describe_calibration(p: &[f32; 7]) -> &'static str {
    if p.iter().all(|v| *v == 0.0) {
        "cancel"
    } else if p[0] == 1.0 {
        "gyro"
    } else if p[1] == 1.0 {
        "compass"
    } else if p[4] != 0.0 {
        "accelerometer"
    } else if p[2] == 1.0 {
        "ground pressure"
    } else if p[3] == 1.0 {
        "radio"
    } else if p[5] == 1.0 {
        "compass motor interference"
    } else if p[6] == 1.0 {
        "esc / battery"
    } else {
        "unknown"
    }
}

fn result_words(result: &str) -> &str {
    result.strip_prefix("MAV_RESULT_").unwrap_or(result)
}

fn result_by_number(result: u8) -> String {
    match result {
        0 => "ACCEPTED".into(),
        1 => "TEMPORARILY_REJECTED".into(),
        2 => "DENIED".into(),
        3 => "UNSUPPORTED".into(),
        4 => "FAILED".into(),
        5 => "IN_PROGRESS".into(),
        6 => "CANCELLED".into(),
        7 => "COMMAND_LONG_ONLY".into(),
        8 => "COMMAND_INT_ONLY".into(),
        9 => "UNSUPPORTED_MAV_FRAME".into(),
        other => format!("result {other}"),
    }
}

/// Command and result of a `COMMAND_ACK` whose command the dialect does not know (MAVLink 2
/// payload, zero-extended: command u16, result u8, progress u8).
fn raw_ack(frame: &Frame) -> Option<(String, String)> {
    if frame.message_id != 77 || frame.raw.len() < 12 || frame.raw[0] != 0xFD {
        return None;
    }
    let len = usize::from(frame.raw[1]);
    let mut payload = [0u8; 4];
    for (slot, byte) in payload.iter_mut().zip(&frame.raw[10..10 + len.min(4)]) {
        *slot = *byte;
    }
    let command = u16::from_le_bytes([payload[0], payload[1]]);
    Some((format!("command {command}"), result_by_number(payload[2])))
}

fn firmware_string(version: u32, custom: &[u8; 8]) -> String {
    let kind = match version & 0xFF {
        0 => "-dev",
        64 => "-alpha",
        128 => "-beta",
        192 => "-rc",
        _ => "",
    };
    let mut text = format!(
        "{}.{}.{}{kind}",
        version >> 24,
        version >> 16 & 0xFF,
        version >> 8 & 0xFF
    );
    if custom.iter().any(|b| *b != 0) {
        text.push_str(" (");
        for byte in custom {
            text.push_str(&format!("{byte:02x}"));
        }
        text.push(')');
    }
    text
}

impl<'a> Pass<'a> {
    fn time_s(&self, time_us: u64) -> f64 {
        time_us.saturating_sub(self.start_us) as f64 / 1e6
    }

    fn mark(&mut self, t_s: f64, kind: &'static str, text: String) {
        self.marks.push(Mark { t_s, kind, text });
    }

    fn record(&mut self, time_us: u64, raw: &[u8]) -> io::Result<()> {
        let mut frames = Vec::new();
        self.deframer.push(raw, &mut frames);
        let t_s = self.time_s(time_us);
        self.last_t_s = self.last_t_s.max(t_s);
        for frame in &frames {
            self.frame(t_s, frame)?;
        }
        Ok(())
    }

    fn frame(&mut self, t_s: f64, frame: &Frame) -> io::Result<()> {
        if frame.system_id == self.station_id {
            self.station_frames += 1;
            if let Body::Known(Message::COMMAND_LONG(c)) = &frame.body {
                self.sent.insert(
                    format!("{:?}", c.command),
                    [
                        c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7,
                    ],
                );
            }
            return Ok(());
        }
        self.first_sender.get_or_insert(frame.system_id);
        if let Some(last) = self.last_rx_s
            && t_s - last > LINK_GAP_S
        {
            self.mark(last, "link", "link lost".to_owned());
            self.mark(t_s, "link", "link back".to_owned());
        }
        self.last_rx_s = Some(t_s);
        let message = match &frame.body {
            Body::Known(message) => message,
            Body::Undecodable(_) => {
                if let Some((command, result)) = raw_ack(frame) {
                    self.mark(t_s, "command", format!("{command}: {result}"));
                }
                self.skip(
                    format!("ID_{}", frame.message_id),
                    "the dialect refused the payload",
                );
                return Ok(());
            }
            Body::Unknown => {
                self.skip(
                    format!("ID_{}", frame.message_id),
                    "not in the dialect (not decoded)",
                );
                return Ok(());
            }
        };
        self.on_message(t_s, frame.system_id, message);
        let Ok(Value::Object(map)) = serde_json::to_value(message) else {
            return Ok(());
        };
        let Some(name) = map.get("type").and_then(Value::as_str).map(str::to_owned) else {
            return Ok(());
        };
        if POSITION_ONLY.contains(&name.as_str()) {
            self.skip(name, "only positions (privacy)");
            return Ok(());
        }
        let mut cells = Vec::with_capacity(map.len());
        for (field, value) in &map {
            if field != "type" && !is_position_field(field) {
                flatten(field, value, &mut cells);
            }
        }
        for (field, bits) in flag_values(message) {
            if let Some((_, cell)) = cells.iter_mut().find(|(n, _)| n == field) {
                *cell = Cell {
                    text: bits.to_string(),
                    number: None,
                };
            }
        }
        let time_column = ["time_usec", "time_boot_ms"]
            .iter()
            .find(|t| cells.iter().any(|(n, _)| n == *t))
            .map(|t| (*t).to_owned());
        if let Some(time) = &time_column {
            let at = cells.iter().position(|(n, _)| n == time).unwrap_or(0);
            let cell = cells.remove(at);
            cells.insert(0, cell);
        }
        let label = INSTANCE_FIELDS
            .iter()
            .find(|(message, _)| *message == name)
            .and_then(|(_, field)| map.get(*field))
            .map_or_else(
                || name.clone(),
                |value| {
                    let instance = match value {
                        Value::Object(inner) => inner
                            .get("type")
                            .and_then(Value::as_str)
                            .unwrap_or("?")
                            .to_owned(),
                        Value::String(text) => text.clone(),
                        other => other.to_string(),
                    };
                    format!("{name}[{instance}]")
                },
            );
        self.overall
            .entry(label.clone())
            .or_default()
            .add(t_s, &cells, time_column.as_deref());
        let slot = self.bounds.partition_point(|b| *b <= t_s);
        self.intervals[slot]
            .entry(label)
            .or_default()
            .add(t_s, &cells, time_column.as_deref());
        if !self.tables.contains_key(&name) {
            let path = self.folder.join(format!("{name}.csv"));
            let mut writer = BufWriter::new(File::create(path)?);
            let mut header = vec!["t_s".to_owned()];
            header.extend(cells.iter().map(|(n, _)| n.clone()));
            writer.write_all(header.join(",").as_bytes())?;
            writer.write_all(b"\n")?;
            let units = self.units.entry(name.clone()).or_default();
            for column in &header {
                if let Some(unit) = unit_of(&name, column) {
                    units.insert(column.clone(), unit);
                }
            }
            if units.is_empty() {
                self.units.remove(&name);
            }
            self.tables.insert(
                name.clone(),
                Table {
                    writer,
                    header,
                    rows: 0,
                },
            );
        }
        if let Some(table) = self.tables.get_mut(&name) {
            table.row(t_s, &cells)?;
        }
        Ok(())
    }

    fn skip(&mut self, name: String, why: &'static str) {
        self.skipped.entry(name).or_insert((0, why)).0 += 1;
    }

    /// What the typed message gives besides its columns: marks, parameters, firmware, vehicle.
    fn on_message(&mut self, t_s: f64, sender: u8, message: &Message) {
        match message {
            Message::HEARTBEAT(h) if h.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID => {
                if self.vehicle_id.is_none() {
                    self.vehicle_id = Some(sender);
                    self.ardupilot = h.autopilot == MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA;
                }
            }
            Message::AUTOPILOT_VERSION(v) => {
                if self.firmware.is_none() {
                    self.firmware = Some(firmware_string(
                        v.flight_sw_version,
                        &v.flight_custom_version,
                    ));
                    self.capabilities = Some(v.capabilities);
                }
            }
            Message::COMMAND_ACK(ack) => {
                let command = format!("{:?}", ack.command);
                let result = format!("{:?}", ack.result);
                let mut text = command.clone();
                if ack.command == MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION
                    && let Some(p) = self.sent.get(&command)
                {
                    text = format!("{command} ({})", describe_calibration(p));
                }
                if ack.command == MavCmd::MAV_CMD_DO_MOTOR_TEST
                    && let Some(p) = self.sent.get(&command)
                {
                    text = format!("{command} ({})", describe_motor_test(p));
                }
                text = format!("{text}: {}", result_words(&result));
                if result.ends_with("IN_PROGRESS") && ack.progress <= 100 {
                    text = format!("{text} ({}%)", ack.progress);
                }
                self.mark(t_s, "command", text);
            }
            Message::STATUSTEXT(s) => {
                let severity = format!("{:?}", s.severity);
                let severity = severity.strip_prefix("MAV_SEVERITY_").unwrap_or(&severity);
                let text: String = s.text.to_str().map(str::to_owned).unwrap_or_default();
                self.mark(t_s, "text", format!("{severity}: {text}"));
            }
            Message::PARAM_VALUE(p) => {
                let name = p.param_id.to_str().map(str::to_owned).unwrap_or_default();
                let kind = match p.param_type {
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_UINT8 => Some(ParamType::U8),
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_INT8 => Some(ParamType::I8),
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_UINT16 => {
                        Some(ParamType::U16)
                    }
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_INT16 => {
                        Some(ParamType::I16)
                    }
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_UINT32 => {
                        Some(ParamType::U32)
                    }
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_INT32 => {
                        Some(ParamType::I32)
                    }
                    gcs_mavlink::dialect::MavParamType::MAV_PARAM_TYPE_REAL32 => {
                        Some(ParamType::F32)
                    }
                    _ => None,
                };
                self.params.push(ParamEvent {
                    t_s,
                    name,
                    kind,
                    raw: p.param_value.to_le_bytes(),
                });
            }
            _ => {}
        }
    }

    fn encoding(&self) -> ParamEncoding {
        match self.capabilities {
            Some(caps)
                if caps.contains(
                    MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
                ) =>
            {
                ParamEncoding::Bytewise
            }
            Some(caps)
                if caps.contains(
                    MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST,
                ) =>
            {
                ParamEncoding::CCast
            }
            _ if self.ardupilot => ParamEncoding::CCast,
            _ => ParamEncoding::Bytewise,
        }
    }

    /// `(at_start, changes)` of the parameters.
    fn parameters(&self, snapshot: Option<&[(String, f64)]>) -> (Map<String, Value>, Vec<Value>) {
        let encoding = self.encoding();
        let mut current: BTreeMap<String, f64> = BTreeMap::new();
        let mut at_start = Map::new();
        if let Some(snapshot) = snapshot {
            for (name, value) in snapshot {
                current.insert(name.clone(), *value);
                at_start.insert(name.clone(), json!(value));
            }
        }
        let mut changes = Vec::new();
        for event in &self.params {
            let Some(kind) = event.kind else { continue };
            let value = match decode(kind, event.raw, encoding) {
                ParamValue::F32(v) => v.to_string().parse().unwrap_or_else(|_| f64::from(v)),
                other => other.as_f64(),
            };
            match current.insert(event.name.clone(), value) {
                None if snapshot.is_none() => {
                    at_start.insert(event.name.clone(), json!(value));
                }
                Some(old) if old != value => changes.push(json!({
                    "t_s": sig6(event.t_s),
                    "name": event.name,
                    "old": old,
                    "new": value,
                })),
                _ => {}
            }
        }
        (at_start, changes)
    }
}

/// A time in seconds to the microsecond.
fn sig6(t_s: f64) -> f64 {
    (t_s * 1e6).round() / 1e6
}

/// Convert the `.tlog` at `tlog` into a session folder under `out_dir`.
///
/// The folder is `<out_dir>/<UTC time of the first frame>_<name>` (just the time when the name is that time); it must not exist (an existing one
/// is never touched). A truncated last record is reported in `warnings` of `session.json` and the
/// frames before it are used. On any other failure the half-written folder is removed.
pub fn export_session(
    tlog: &Path,
    out_dir: &Path,
    options: &SessionOptions,
) -> io::Result<Exported> {
    let mut reader = TlogReader::new(BufReader::new(File::open(tlog)?));
    let first = match reader.next() {
        Some(record) => record?,
        None => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("{} holds no frames", tlog.display()),
            ));
        }
    };
    fs::create_dir_all(out_dir)?;
    let stamp = folder_time(first.time_us);
    let folder = if options.name == stamp {
        out_dir.join(&stamp)
    } else {
        out_dir.join(format!("{stamp}_{}", file_safe(&options.name)))
    };
    fs::create_dir(&folder).map_err(|e| {
        if e.kind() == io::ErrorKind::AlreadyExists {
            io::Error::new(
                e.kind(),
                format!(
                    "{} already exists (a session folder is never overwritten)",
                    folder.display()
                ),
            )
        } else {
            e
        }
    })?;
    match write_session(tlog, &folder, options, first, reader) {
        Ok(exported) => Ok(exported),
        Err(error) => {
            let _ = fs::remove_dir_all(&folder);
            Err(error)
        }
    }
}

fn write_session(
    tlog: &Path,
    folder: &Path,
    options: &SessionOptions,
    first: gcs_mavlink::tlog::TlogRecord,
    reader: TlogReader<BufReader<File>>,
) -> io::Result<Exported> {
    fs::copy(tlog, folder.join(TLOG_NAME))?;
    let start_us = first.time_us;
    let mut user: Vec<(f64, &str)> = options
        .marks
        .iter()
        .map(|m| {
            (
                m.time_us.saturating_sub(start_us) as f64 / 1e6,
                m.text.as_str(),
            )
        })
        .collect();
    user.sort_by(|a, b| a.0.total_cmp(&b.0));
    let bounds: Vec<f64> = user.iter().map(|(t, _)| *t).collect();
    let mut pass = Pass {
        start_us,
        folder,
        station_id: SessionConfig::default().system_id,
        tables: BTreeMap::new(),
        units: BTreeMap::new(),
        skipped: BTreeMap::new(),
        overall: BTreeMap::new(),
        intervals: (0..=bounds.len()).map(|_| BTreeMap::new()).collect(),
        bounds,
        marks: Vec::new(),
        params: Vec::new(),
        deframer: Deframer::new(),
        station_frames: 0,
        sent: BTreeMap::new(),
        vehicle_id: None,
        first_sender: None,
        ardupilot: false,
        capabilities: None,
        firmware: None,
        last_rx_s: None,
        last_t_s: 0.0,
    };
    let mut warnings: Vec<String> = Vec::new();
    pass.record(first.time_us, &first.frame)?;
    for record in reader {
        match record {
            Ok(record) => pass.record(record.time_us, &record.frame)?,
            Err(error) => {
                warnings.push(error.to_string());
                break;
            }
        }
    }
    for (t_s, text) in &user {
        pass.mark(*t_s, "user", (*text).to_owned());
    }
    pass.marks.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));

    let mut files = Vec::new();
    for (name, table) in &mut pass.tables {
        table.writer.flush()?;
        files.push((format!("{name}.csv"), table.rows));
    }
    let duration = pass.last_t_s;
    let (at_start, changes) = pass.parameters(options.parameters.as_deref());
    let marks_json: Vec<Value> = pass
        .marks
        .iter()
        .map(|m| json!({"t_s": sig6(m.t_s), "kind": m.kind, "text": m.text}))
        .collect();
    let intervals: Vec<Value> = if user.is_empty() {
        Vec::new()
    } else {
        (0..=user.len())
            .map(|i| {
                json!({
                    "from": if i == 0 { "start" } else { user[i - 1].1 },
                    "to": if i == user.len() { "stop" } else { user[i].1 },
                    "start_s": if i == 0 { 0.0 } else { sig6(user[i - 1].0) },
                    "end_s": if i == user.len() { sig6(duration) } else { sig6(user[i].0) },
                    "messages": groups_json(&pass.intervals[i]),
                })
            })
            .collect()
    };
    let skipped: Vec<(String, String)> = pass
        .skipped
        .iter()
        .map(|(name, (count, why))| (name.clone(), format!("{why} ({count} frames)")))
        .collect();
    let files_json: Map<String, Value> = files.iter().map(|(n, r)| (n.clone(), json!(r))).collect();
    let units_json: Map<String, Value> = pass
        .units
        .iter()
        .map(|(message, fields)| (message.clone(), json!(fields)))
        .collect();
    let source = tlog
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let document = json!({
        "schema_version": SCHEMA_VERSION,
        "name": options.name,
        "start_utc": iso_utc(start_us),
        "stop_utc": iso_utc(start_us + (duration * 1e6).round() as u64),
        "duration_s": sig6(duration),
        "source": source,
        "vehicle": {
            "system_id": pass.vehicle_id.or(pass.first_sender),
            "firmware": pass.firmware,
        },
        "station_frames": pass.station_frames,
        "parameters": {"at_start": at_start, "changes": changes},
        "marks": marks_json,
        "files": files_json,
        "skipped": skipped
            .iter()
            .map(|(m, why)| json!({"message": m, "why": why}))
            .collect::<Vec<_>>(),
        "units": units_json,
        "summary": {
            "messages": groups_json(&pass.overall),
            "intervals": intervals,
        },
        "warnings": warnings,
    });
    let mut text = serde_json::to_string_pretty(&document).map_err(io::Error::other)?;
    text.push('\n');
    fs::write(folder.join(JSON_NAME), text)?;
    Ok(Exported {
        folder: folder.to_path_buf(),
        files,
        marks: pass.marks.len(),
        skipped,
    })
}

/// The sessions under `dir` (every sub-folder with a readable `session.json`), by folder name.
pub fn list_sessions(dir: &Path) -> io::Result<Vec<SessionInfo>> {
    let mut sessions = Vec::new();
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        let Ok(text) = fs::read_to_string(path.join(JSON_NAME)) else {
            continue;
        };
        let Ok(Value::Object(map)) = serde_json::from_str::<Value>(&text) else {
            continue;
        };
        sessions.push(SessionInfo {
            folder: path
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default(),
            name: map
                .get("name")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_owned(),
            start_utc: map
                .get("start_utc")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_owned(),
            duration_s: map.get("duration_s").and_then(Value::as_f64).unwrap_or(0.0),
            marks: map
                .get("marks")
                .and_then(Value::as_array)
                .map_or(0, Vec::len),
        });
    }
    sessions.sort_by(|a, b| a.folder.cmp(&b.folder));
    Ok(sessions)
}

#[cfg(test)]
#[path = "session_folder_tests.rs"]
mod tests;
