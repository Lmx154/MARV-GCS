use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};

use gcs_mavlink::Encoder;
use gcs_mavlink::dialect::{
    AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA,
    HIGHRES_IMU_DATA, HOME_POSITION_DATA, MavAutopilot, MavCmd, MavMessage as Message,
    MavParamType, MavResult, MavSeverity, PARAM_VALUE_DATA, STATUSTEXT_DATA,
};
use serde_json::Value;

use super::*;

/// 2023-11-14 22:13:20 UTC.
const T0: u64 = 1_700_000_000_000_000;

struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let path = std::env::temp_dir().join(format!(
            "gcs-link-session-{tag}-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).expect("temp dir");
        Self(path)
    }

    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

/// A recording built frame by frame: the vehicle is system 1, the station 255.
struct Recording {
    bytes: Vec<u8>,
    vehicle: Encoder,
    station: Encoder,
}

impl Recording {
    fn new() -> Self {
        Self {
            bytes: Vec::new(),
            vehicle: Encoder::new(1, 1),
            station: Encoder::new(255, 190),
        }
    }

    fn vehicle(&mut self, at_s: f64, message: Message) {
        let frame = self.vehicle.encode(&message);
        self.push(at_s, &frame);
    }

    fn station(&mut self, at_s: f64, message: Message) {
        let frame = self.station.encode(&message);
        self.push(at_s, &frame);
    }

    fn push(&mut self, at_s: f64, frame: &[u8]) {
        self.bytes
            .extend_from_slice(&(T0 + (at_s * 1e6) as u64).to_be_bytes());
        self.bytes.extend_from_slice(frame);
    }

    fn write(&self, dir: &Path, name: &str) -> PathBuf {
        let path = dir.join(name);
        fs::write(&path, &self.bytes).expect("write tlog");
        path
    }
}

fn heartbeat() -> Message {
    Message::HEARTBEAT(HEARTBEAT_DATA {
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        ..HEARTBEAT_DATA::default()
    })
}

fn imu(gyro: f32, id: u8) -> Message {
    Message::HIGHRES_IMU(HIGHRES_IMU_DATA {
        time_usec: 5_000,
        xgyro: gyro,
        zacc: -9.5,
        id,
        ..HIGHRES_IMU_DATA::default()
    })
}

fn param(name: &str, value: f32) -> Message {
    Message::PARAM_VALUE(PARAM_VALUE_DATA {
        param_value: value,
        param_count: 2,
        param_index: 0,
        param_id: name.into(),
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
    })
}

fn gps() -> Message {
    Message::GPS_RAW_INT(GPS_RAW_INT_DATA {
        lat: 0,
        lon: 0,
        alt: 1500,
        satellites_visible: 7,
        ..GPS_RAW_INT_DATA::default()
    })
}

/// 9 s: calibration at 2 s, a user mark at 2.5 s, 5.5 s of silence after 3.5 s.
fn calibration_recording() -> Recording {
    let mut rec = Recording::new();
    rec.vehicle(0.0, heartbeat());
    rec.vehicle(
        0.1,
        Message::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
            flight_sw_version: 1 << 24 | 2 << 16 | 3 << 8 | 255,
            ..AUTOPILOT_VERSION_DATA::default()
        }),
    );
    rec.vehicle(0.5, param("GYRO_GAIN", 1.0));
    rec.vehicle(0.5, param("OTHER", 5.0));
    rec.vehicle(1.0, imu(0.1, 1));
    rec.vehicle(1.0, gps());
    rec.vehicle(1.5, imu(0.1, 1));
    rec.station(
        2.0,
        Message::COMMAND_LONG(COMMAND_LONG_DATA {
            command: MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            param1: 1.0,
            ..COMMAND_LONG_DATA::default()
        }),
    );
    rec.vehicle(
        2.1,
        Message::COMMAND_ACK(COMMAND_ACK_DATA {
            command: MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            result: MavResult::MAV_RESULT_ACCEPTED,
            ..COMMAND_ACK_DATA::default()
        }),
    );
    rec.vehicle(2.4, imu(0.1, 1));
    rec.vehicle(3.0, imu(0.3, 1));
    rec.vehicle(
        3.2,
        Message::STATUSTEXT(STATUSTEXT_DATA {
            severity: MavSeverity::MAV_SEVERITY_INFO,
            text: "Calibration done".into(),
            ..STATUSTEXT_DATA::default()
        }),
    );
    rec.vehicle(3.4, param("GYRO_GAIN", 2.0));
    rec.vehicle(3.5, imu(0.3, 1));
    rec.vehicle(9.0, imu(0.3, 1));
    rec
}

fn options() -> SessionOptions {
    SessionOptions {
        name: "gyro, test".to_owned(),
        parameters: None,
        marks: vec![UserMark {
            time_us: T0 + 2_500_000,
            text: "board lying still".to_owned(),
        }],
    }
}

fn read_json(folder: &Path) -> Value {
    serde_json::from_str(&fs::read_to_string(folder.join("session.json")).expect("session.json"))
        .expect("valid json")
}

fn lines(path: &Path) -> Vec<String> {
    fs::read_to_string(path)
        .expect("csv")
        .lines()
        .map(str::to_owned)
        .collect()
}

fn export(rec: &Recording, options: &SessionOptions) -> (TempDir, Exported) {
    let dir = TempDir::new("export");
    let tlog = rec.write(dir.path(), "cal.tlog");
    let exported = export_session(&tlog, &dir.path().join("sessions"), options).expect("export");
    (dir, exported)
}

#[test]
fn the_folder_holds_the_tlog_the_json_and_one_csv_per_message_type() {
    let rec = calibration_recording();
    let (dir, exported) = export(&rec, &options());
    assert_eq!(
        exported.folder,
        dir.path().join("sessions/2023-11-14_22-13-20Z_gyro__test")
    );
    let mut names: Vec<String> = fs::read_dir(&exported.folder)
        .expect("folder")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    assert_eq!(
        names,
        [
            "AUTOPILOT_VERSION.csv",
            "COMMAND_ACK.csv",
            "GPS_RAW_INT.csv",
            "HEARTBEAT.csv",
            "HIGHRES_IMU.csv",
            "PARAM_VALUE.csv",
            "STATUSTEXT.csv",
            "session.json",
            "session.tlog",
        ]
    );
    assert_eq!(
        fs::read(exported.folder.join("session.tlog")).expect("copy"),
        rec.bytes
    );
    let json = read_json(&exported.folder);
    assert_eq!(json["files"]["HIGHRES_IMU.csv"], 6);
    assert_eq!(json["files"]["PARAM_VALUE.csv"], 3);
    assert_eq!(json["files"]["HEARTBEAT.csv"], 1);
    assert!(json["files"].get("COMMAND_LONG.csv").is_none());
    assert_eq!(json["station_frames"], 1);
    assert_eq!(
        exported.files.iter().map(|(_, rows)| rows).sum::<u64>(),
        6 + 3 + 1 + 1 + 1 + 1 + 1
    );
}

#[test]
fn csv_headers_and_times_start_at_zero_with_the_message_time_second() {
    let rec = calibration_recording();
    let (_dir, exported) = export(&rec, &options());
    let imu = lines(&exported.folder.join("HIGHRES_IMU.csv"));
    assert_eq!(imu.len(), 7);
    let header: Vec<&str> = imu[0].split(',').collect();
    assert_eq!(&header[..3], ["t_s", "time_usec", "xacc"]);
    assert!(
        header.contains(&"xgyro") && header.contains(&"id"),
        "{header:?}"
    );
    assert!(imu[1].starts_with("1.000000,5000,"), "{}", imu[1]);
    let heartbeat = lines(&exported.folder.join("HEARTBEAT.csv"));
    assert!(heartbeat[1].starts_with("0.000000,"), "{heartbeat:?}");
    let last: Vec<&str> = imu[6].split(',').collect();
    assert_eq!(last[0], "9.000000");
    let gyro = header.iter().position(|c| *c == "xgyro").expect("xgyro");
    assert_eq!(last[gyro], "0.3");
}

#[test]
fn the_json_has_the_vehicle_the_marks_and_the_units() {
    let rec = calibration_recording();
    let (_dir, exported) = export(&rec, &options());
    let json = read_json(&exported.folder);
    assert_eq!(json["schema_version"], 1);
    assert_eq!(json["name"], "gyro, test");
    assert_eq!(json["start_utc"], "2023-11-14T22:13:20.000Z");
    assert_eq!(json["stop_utc"], "2023-11-14T22:13:29.000Z");
    assert_eq!(json["duration_s"], 9.0);
    assert_eq!(json["source"], "cal.tlog");
    assert_eq!(json["vehicle"]["system_id"], 1);
    assert_eq!(json["vehicle"]["firmware"], "1.2.3");
    assert_eq!(json["units"]["HIGHRES_IMU"]["xgyro"], "rad/s");
    assert_eq!(json["units"]["HIGHRES_IMU"]["time_usec"], "us");
    let marks: Vec<(f64, &str, &str)> = json["marks"]
        .as_array()
        .expect("marks")
        .iter()
        .map(|m| {
            (
                m["t_s"].as_f64().expect("t_s"),
                m["kind"].as_str().expect("kind"),
                m["text"].as_str().expect("text"),
            )
        })
        .collect();
    assert_eq!(
        marks,
        [
            (
                2.1,
                "command",
                "MAV_CMD_PREFLIGHT_CALIBRATION (gyro): ACCEPTED"
            ),
            (2.5, "user", "board lying still"),
            (3.2, "text", "INFO: Calibration done"),
            (3.5, "link", "link lost"),
            (9.0, "link", "link back"),
        ]
    );
    assert_eq!(exported.marks, 5);
}

#[test]
fn the_summary_splits_at_the_user_mark() {
    let rec = calibration_recording();
    let (_dir, exported) = export(&rec, &options());
    let json = read_json(&exported.folder);
    let all = &json["summary"]["messages"]["HIGHRES_IMU[1]"];
    assert_eq!(all["count"], 6);
    assert_eq!(all["fields"]["zacc"]["mean"], -9.5);
    assert!(all["fields"].get("time_usec").is_none());
    let intervals = json["summary"]["intervals"].as_array().expect("intervals");
    assert_eq!(intervals.len(), 2);
    assert_eq!(intervals[0]["from"], "start");
    assert_eq!(intervals[0]["to"], "board lying still");
    assert_eq!(intervals[0]["start_s"], 0.0);
    assert_eq!(intervals[0]["end_s"], 2.5);
    assert_eq!(intervals[1]["from"], "board lying still");
    assert_eq!(intervals[1]["to"], "stop");
    assert_eq!(intervals[1]["end_s"], 9.0);
    let before = &intervals[0]["messages"]["HIGHRES_IMU[1]"];
    let after = &intervals[1]["messages"]["HIGHRES_IMU[1]"];
    assert_eq!(before["count"], 3);
    assert_eq!(after["count"], 3);
    assert_eq!(before["fields"]["xgyro"]["mean"], 0.1);
    assert_eq!(before["fields"]["xgyro"]["sd"], 0.0);
    assert_eq!(after["fields"]["xgyro"]["mean"], 0.3);
    assert_eq!(after["fields"]["xgyro"]["min"], 0.3);
    assert_eq!(before["rate_hz"], sig(2.0 / 1.4));
    let none = SessionOptions {
        marks: Vec::new(),
        ..options()
    };
    let (_dir2, exported) = export(&rec, &none);
    let json = read_json(&exported.folder);
    assert_eq!(
        json["summary"]["intervals"].as_array().expect("array").len(),
        0
    );
}

#[test]
fn a_parameter_change_is_listed_and_the_start_comes_from_the_first_value() {
    let rec = calibration_recording();
    let (_dir, exported) = export(&rec, &options());
    let json = read_json(&exported.folder);
    assert_eq!(json["parameters"]["at_start"]["GYRO_GAIN"], 1.0);
    assert_eq!(json["parameters"]["at_start"]["OTHER"], 5.0);
    let changes = json["parameters"]["changes"].as_array().expect("changes");
    assert_eq!(changes.len(), 1);
    assert_eq!(changes[0]["t_s"], 3.4);
    assert_eq!(changes[0]["name"], "GYRO_GAIN");
    assert_eq!(changes[0]["old"], 1.0);
    assert_eq!(changes[0]["new"], 2.0);
}

#[test]
fn a_snapshot_is_the_start_and_the_first_value_is_compared_to_it() {
    let rec = calibration_recording();
    let with = SessionOptions {
        parameters: Some(vec![
            ("GYRO_GAIN".to_owned(), 0.5),
            ("ONLY_HERE".to_owned(), 7.0),
        ]),
        ..options()
    };
    let (_dir, exported) = export(&rec, &with);
    let json = read_json(&exported.folder);
    let start = json["parameters"]["at_start"].as_object().expect("start");
    assert_eq!(start.len(), 2);
    assert_eq!(start["GYRO_GAIN"], 0.5);
    let changes = json["parameters"]["changes"].as_array().expect("changes");
    assert_eq!(changes.len(), 2);
    assert_eq!((&changes[0]["old"], &changes[0]["new"]), (&json!(0.5), &json!(1.0)));
    assert_eq!((&changes[1]["old"], &changes[1]["new"]), (&json!(1.0), &json!(2.0)));
}

#[test]
fn a_float_parameter_keeps_its_short_decimal_form() {
    let mut rec = Recording::new();
    rec.vehicle(0.0, param("SMALL", 0.1));
    let (_dir, exported) = export(&rec, &SessionOptions::default());
    let json = read_json(&exported.folder);
    assert_eq!(json["parameters"]["at_start"]["SMALL"], 0.1);
}

fn walk(value: &Value, keys: &mut Vec<String>) {
    match value {
        Value::Object(map) => {
            for (key, inner) in map {
                keys.push(key.clone());
                walk(inner, keys);
            }
        }
        Value::Array(items) => items.iter().for_each(|i| walk(i, keys)),
        _ => {}
    }
}

#[test]
fn no_latitude_or_longitude_reaches_the_csv_or_the_json() {
    let rec = calibration_recording();
    let (_dir, exported) = export(&rec, &options());
    let gps = lines(&exported.folder.join("GPS_RAW_INT.csv"));
    let header: Vec<&str> = gps[0].split(',').collect();
    assert!(
        header.contains(&"alt") && header.contains(&"satellites_visible"),
        "{header:?}"
    );
    for column in &header {
        assert!(!is_position_field(column), "{column}");
    }
    let text = fs::read_to_string(exported.folder.join("session.json")).expect("json");
    let mut keys = Vec::new();
    walk(&read_json(&exported.folder), &mut keys);
    assert!(
        !keys.iter().any(|k| is_position_field(k)),
        "a position key in session.json: {keys:?}"
    );
    assert!(
        !text.contains("\"lat\"") && !text.contains("\"lon\""),
        "{text}"
    );
    assert!(gps[1].contains(",1500,"), "{}", gps[1]);
}

#[test]
fn position_fields_and_position_only_messages_are_recognised() {
    for name in [
        "lat",
        "lon",
        "latitude",
        "longitude",
        "Lat",
        "home_lat",
        "target_lon",
        "lat_int",
        "lon_int",
    ] {
        assert!(is_position_field(name), "{name}");
    }
    for name in ["alt", "latency", "flat", "relative_alt", "load", "online"] {
        assert!(!is_position_field(name), "{name}");
    }
    let mut rec = Recording::new();
    rec.vehicle(
        0.0,
        Message::HOME_POSITION(HOME_POSITION_DATA::default()),
    );
    rec.vehicle(0.5, heartbeat());
    let (_dir, exported) = export(&rec, &SessionOptions::default());
    assert!(!exported.folder.join("HOME_POSITION.csv").exists());
    assert_eq!(exported.skipped.len(), 1);
    assert_eq!(exported.skipped[0].0, "HOME_POSITION");
    let json = read_json(&exported.folder);
    assert_eq!(json["skipped"][0]["message"], "HOME_POSITION");
}

#[test]
fn an_existing_folder_is_never_overwritten() {
    let rec = calibration_recording();
    let (dir, exported) = export(&rec, &options());
    let marker = exported.folder.join("keep.txt");
    fs::write(&marker, "mine").expect("marker");
    let tlog = dir.path().join("cal.tlog");
    let error =
        export_session(&tlog, &dir.path().join("sessions"), &options()).expect_err("must refuse");
    assert_eq!(error.kind(), io::ErrorKind::AlreadyExists);
    assert!(error.to_string().contains("never overwritten"), "{error}");
    assert_eq!(fs::read_to_string(&marker).expect("marker"), "mine");
}

#[test]
fn an_empty_or_missing_file_is_an_error_and_leaves_no_folder() {
    let dir = TempDir::new("empty");
    let empty = dir.path().join("empty.tlog");
    fs::write(&empty, []).expect("empty");
    let out = dir.path().join("sessions");
    let error = export_session(&empty, &out, &SessionOptions::default()).expect_err("empty");
    assert!(error.to_string().contains("no frames"), "{error}");
    assert!(!out.exists());
    assert!(
        export_session(&dir.path().join("none.tlog"), &out, &SessionOptions::default()).is_err()
    );
}

#[test]
fn a_truncated_recording_is_converted_with_a_warning() {
    let mut rec = calibration_recording();
    rec.bytes.extend_from_slice(&T0.to_be_bytes());
    rec.bytes.extend_from_slice(&[0xFD, 9, 0]);
    let (_dir, exported) = export(&rec, &options());
    let json = read_json(&exported.folder);
    assert_eq!(json["files"]["HIGHRES_IMU.csv"], 6);
    assert_eq!(json["warnings"].as_array().expect("warnings").len(), 1);
}

#[test]
fn the_list_reads_the_sessions_in_order() {
    let rec = calibration_recording();
    let (dir, _) = export(&rec, &options());
    let out = dir.path().join("sessions");
    fs::create_dir(out.join("not-a-session")).expect("dir");
    let other = SessionOptions {
        name: "a".to_owned(),
        ..SessionOptions::default()
    };
    export_session(&dir.path().join("cal.tlog"), &out, &other).expect("second");
    let list = list_sessions(&out).expect("list");
    assert_eq!(list.len(), 2);
    assert_eq!(list[0].folder, "2023-11-14_22-13-20Z_a");
    assert_eq!(list[0].marks, 4);
    assert_eq!(list[1].name, "gyro, test");
    assert_eq!(list[1].start_utc, "2023-11-14T22:13:20.000Z");
    assert_eq!(list[1].duration_s, 9.0);
    assert_eq!(list[1].marks, 5);
}

#[test]
fn units_come_from_the_table_for_arrays_and_rc_channels() {
    assert_eq!(unit_of("RC_CHANNELS", "chan7_raw"), Some("us"));
    assert_eq!(unit_of("HIGHRES_IMU", "xmag"), Some("gauss"));
    assert_eq!(unit_of("SCALED_PRESSURE", "press_abs"), Some("hPa"));
    assert_eq!(unit_of("SYS_STATUS", "voltage_battery"), Some("mV"));
    assert_eq!(
        unit_of("MARV_DEVICE_STATUS", "last_sample_age_ms"),
        Some("ms")
    );
    assert_eq!(unit_of("MANUAL_CONTROL", "x"), None);
    assert_eq!(unit_of("HIGHRES_IMU", "nothing"), None);
}
