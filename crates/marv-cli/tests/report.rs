//! `marv-cli log report`: files only, no link and no vehicle; the log is synthesised here.

mod common;

use common::{TempDir, run};

fn padded(text: &str, size: usize) -> Vec<u8> {
    let mut v = text.as_bytes().to_vec();
    v.resize(size, 0);
    v
}

fn fmt(type_id: u8, name: &str, format: &str, columns: &str, length: u8) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 128, type_id, length];
    r.extend(padded(name, 4));
    r.extend(padded(format, 16));
    r.extend(padded(columns, 64));
    r
}

/// An armed window of 6 s with a ground second and a hover (stronger in its first second, so that the
/// flight bounds survive the second round), an IMU at 100 Hz; a GPS record with a position.
fn synthetic_log() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt(
        130,
        "MOT",
        "QBBHHHHHHHH",
        "TimeUS,Arm,Mode,A0,A1,A2,A3,C0,C1,C2,C3",
        3 + 8 + 1 + 1 + 16,
    ));
    f.extend(fmt(
        131,
        "ESC",
        "QIIIIccccBBBBB",
        "TimeUS,E0,E1,E2,E3,V0,V1,V2,V3,T0,T1,T2,T3,Fl",
        3 + 8 + 16 + 8 + 5,
    ));
    f.extend(fmt(
        132,
        "GPS",
        "QBBIHBcLLeff",
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
        3 + 8 + 1 + 1 + 4 + 2 + 1 + 2 + 4 + 4 + 4 + 4 + 4,
    ));
    f.extend(fmt(
        133,
        "IMU",
        "QBffffff",
        "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
        3 + 8 + 1 + 24,
    ));
    for i in 0..600u64 {
        let t = i * 10_000;
        let armed = u8::from(i < 599);
        let level: u16 = if t < 1_000_000 { 100 } else { 500 };
        let erpm: u32 = match t {
            0..1_000_000 => 0,
            1_000_000..2_000_000 => 24_000,
            _ => 20_000,
        };
        let mut r = vec![0xA3, 0x95, 130];
        r.extend(t.to_le_bytes());
        r.extend([armed, 0]);
        for _ in 0..4 {
            r.extend(level.to_le_bytes());
        }
        r.extend([0u8; 8]);
        f.extend(r);
        let mut r = vec![0xA3, 0x95, 133];
        r.extend(t.to_le_bytes());
        r.push(0);
        for v in [0.0f32, 0.0, 0.0, 0.0, 0.0, 9.80665] {
            r.extend(v.to_le_bytes());
        }
        f.extend(r);
        let mut r = vec![0xA3, 0x95, 131];
        r.extend(t.to_le_bytes());
        for _ in 0..4 {
            r.extend(erpm.to_le_bytes());
        }
        r.extend([0u8; 8 + 5]);
        f.extend(r);
    }
    let mut g = vec![0xA3, 0x95, 132];
    g.extend(1_000_000u64.to_le_bytes());
    g.extend([0, 3]);
    g.extend(0u32.to_le_bytes());
    g.extend(0u16.to_le_bytes());
    g.push(9);
    g.extend(120u16.to_le_bytes());
    g.extend(123_456_789i32.to_le_bytes());
    g.extend((-987_654_321i32).to_le_bytes());
    g.extend(5000i32.to_le_bytes());
    g.extend(1.0f32.to_le_bytes());
    g.extend(2.0f32.to_le_bytes());
    f.extend(g);
    f
}

#[test]
fn the_report_prints_and_writes_json_and_holds_no_position() {
    let dir = TempDir::new("report");
    let log = dir.file("flight.BIN");
    std::fs::write(&log, synthetic_log()).expect("write the log");
    let json = dir.file("report.json");
    let r = run(&format!(
        "log report {} --mass-kg 1 --kf 3e-5 --poles 14 --accel-lpf-hz 5 --json {}",
        log.display(),
        json.display()
    ));
    assert_eq!(r.code, 0, "{r:?}");
    for part in [
        "Flight-log report: flight.BIN",
        "poles 14 (from --poles)",
        "window 1 flight 1",
        "Vibration",
        "window 1 flight 1: 1.000 s to 5.000 s",
        "too slow for spectra",
        "all four motors pooled",
        "round 2, flights bounded with k1",
        "Armed windows",
        "window 1 flight 1",
        "thrust constant (mass x g",
        "ESC temperature",
        "FDS not in this log",
        "valid only for the level range",
    ] {
        assert!(r.out.contains(part), "{part:?} missing in\n{}", r.out);
    }
    let text = std::fs::read_to_string(&json).expect("the JSON file");
    let value: serde_json::Value = serde_json::from_str(&text).expect("valid JSON");
    assert_eq!(value["file"], "flight.BIN");
    assert_eq!(value["inputs"]["poles_source"], "argument");
    assert_eq!(value["armed_windows"].as_array().expect("windows").len(), 1);
    assert_eq!(value["armed_windows"][0]["airborne"], true);
    assert!(
        value["not_in_this_log"]
            .as_array()
            .expect("list")
            .iter()
            .any(|n| n == "FDS")
    );
    for secret in ["123456789", "987654321"] {
        assert!(
            !r.out.contains(secret) && !text.contains(secret),
            "{secret}"
        );
    }

    assert_eq!(value["vibration"]["state"], "done");
    assert_eq!(
        value["vibration"]["intervals"][0]["too_slow_for_spectra"],
        true
    );
    assert_eq!(value["armed_windows"][0]["flights"][0]["bins"], 8);
    assert_eq!(value["motor_identification"]["pooled"]["usable_bins"], 36);

    // A kf too low for the log to fly: the report says so, with the largest r.
    let low = run(&format!(
        "log report {} --mass-kg 1 --kf 1.5e-5 --poles 14",
        log.display()
    ));
    assert_eq!(low.code, 0, "{low:?}");
    assert!(
        low.out.contains(
            "no flight found with the given kf; if the vehicle flew, kf is probably too low (the largest r seen was 0.79)"
        ),
        "{}",
        low.out
    );

    // Without a pole count the sections that need the rotor speed say so; the rest is still there.
    let json2 = dir.file("report2.json");
    let r = run(&format!(
        "log report {} --mass-kg 1 --kf 3e-5 --json {}",
        log.display(),
        json2.display()
    ));
    assert_eq!(r.code, 0, "{r:?}");
    assert!(
        r.out
            .matches("give --poles: the motor's magnet count")
            .count()
            >= 5,
        "{}",
        r.out
    );
    assert!(r.out.contains("Armed windows") && r.out.contains("Health"));
    assert!(!r.out.contains("DEFAULT"));
    let value: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&json2).expect("the JSON file"))
            .expect("JSON");
    assert_eq!(value["inputs"]["poles_source"], "missing");
    assert_eq!(value["vibration"]["state"], "needs_poles");

    // An existing JSON file is never overwritten.
    let again = run(&format!(
        "log report {} --mass-kg 1 --kf 3e-5 --json {}",
        log.display(),
        json.display()
    ));
    assert_eq!(again.code, 1, "{again:?}");
    assert!(again.err.contains("cannot write"), "{again:?}");
}

#[test]
fn the_arguments_are_checked_with_a_clear_message() {
    let r = run("log report some.BIN --kf 3e-5");
    assert_eq!(r.code, 2, "{r:?}");
    assert!(r.err.contains("log report needs --mass-kg"), "{r:?}");
    let r = run("log report some.BIN --mass-kg 1");
    assert_eq!(r.code, 2, "{r:?}");
    assert!(r.err.contains("log report needs --kf"), "{r:?}");
    let r = run("log report some.BIN --mass-kg 0 --kf 1");
    assert_eq!(r.code, 2, "{r:?}");
    assert!(r.err.contains("--mass-kg needs a number above 0"), "{r:?}");
}

#[test]
fn a_missing_file_is_exit_1() {
    let r = run("log report /nonexistent/none.BIN --mass-kg 1 --kf 1e-5");
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("log report of"), "{r:?}");
}

/// The synthetic log with `NAV` and `SETP` inside its flight (1 s to 5 s): the roll setpoint is 20 degrees
/// and the estimate 18, pitch and yaw on target; five more `SETP` records of kind 2.
fn tracking_log() -> Vec<u8> {
    let quat = |roll_deg: f32| {
        let h = roll_deg.to_radians() / 2.0;
        [h.cos(), h.sin(), 0.0f32, 0.0f32]
    };
    let mut f = synthetic_log();
    f.extend(fmt(
        134,
        "NAV",
        "QfffffffB",
        "TimeUS,Q1,Q2,Q3,Q4,GX,GY,GZ,Vld",
        3 + 8 + 28 + 1,
    ));
    f.extend(fmt(
        135,
        "SETP",
        "QBffffffff",
        "TimeUS,K,Q1,Q2,Q3,Q4,RX,RY,RZ,Thr",
        3 + 8 + 1 + 32,
    ));
    for i in 100..490u64 {
        let t = i * 10_000;
        let mut r = vec![0xA3, 0x95, 134];
        r.extend(t.to_le_bytes());
        for v in quat(18.0).into_iter().chain([0.0; 3]) {
            r.extend(v.to_le_bytes());
        }
        r.push(1);
        f.extend(r);
        let mut r = vec![0xA3, 0x95, 135];
        r.extend(t.to_le_bytes());
        r.push(if (200..205).contains(&i) { 2 } else { 1 });
        for v in quat(20.0).into_iter().chain([0.0; 4]) {
            r.extend(v.to_le_bytes());
        }
        f.extend(r);
    }
    f
}

#[test]
fn the_report_has_a_tracking_section_in_text_and_json() {
    let dir = TempDir::new("tracking");
    let log = dir.file("flight.BIN");
    std::fs::write(&log, tracking_log()).expect("write the log");
    let json = dir.file("report.json");
    let r = run(&format!(
        "log report {} --mass-kg 1 --kf 3e-5 --poles 14 --json {}",
        log.display(),
        json.display()
    ));
    assert_eq!(r.code, 0, "{r:?}");
    for part in [
        "Tracking (NAV attitude against the SETP attitude setpoint",
        "5 records with another setpoint kind, not analysed",
        "roll: setpoint 20.00 to 20.00, error mean 2.00, rms 2.00, largest 2.00",
        "yaw (error wrapped to +-180)",
        "roll excursion, largest: 2.00 deg at 1.000 s (estimate 18.00, past the setpoint bound 20.00; body rate 0.0 deg/s)",
        "pitch excursion, largest: none",
        "roll: 1 holds",
        "settled error 2.00",
    ] {
        assert!(r.out.contains(part), "{part:?} missing in\n{}", r.out);
    }
    let value: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&json).expect("the JSON file"))
            .expect("JSON");
    let t = &value["tracking"];
    assert_eq!(t["state"], "done");
    assert_eq!(t["other_setpoint_kind_records"], 5);
    let roll = &t["flights"][0]["axes"][0];
    assert_eq!(roll["axis"], "roll");
    assert!((roll["mean_error_deg"].as_f64().expect("number") - 2.0).abs() < 1e-3);
    assert!(
        (t["flights"][0]["excursion_roll"]["excursion_deg"]
            .as_f64()
            .expect("number")
            - 2.0)
            .abs()
            < 1e-3
    );
    assert!(t["flights"][0]["excursion_pitch"].is_null());
    assert_eq!(t["holds"]["roll"]["count"], 1);
    assert_eq!(t["holds"]["roll"]["holds"][0]["duration_us"], 3_890_000);
    assert!(t["definitions"]["hold_min_s"].is_number());

    // A log with neither NAV nor SETP says so.
    let plain = dir.file("plain.BIN");
    std::fs::write(&plain, synthetic_log()).expect("write the log");
    let r = run(&format!(
        "log report {} --mass-kg 1 --kf 3e-5 --poles 14",
        plain.display()
    ));
    assert!(
        r.out.contains("Tracking") && r.out.contains("not in this log (NAV, SETP)"),
        "{}",
        r.out
    );
}

/// A log with one ground run (1 s to 5.5 s, the motors well above idle, a kf too small for any flight): the
/// estimated roll drifts by 0.5 degrees per second while the gyro reads only its bias.
fn ground_log() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt(
        130,
        "MOT",
        "QBBHHHHHHHH",
        "TimeUS,Arm,Mode,A0,A1,A2,A3,C0,C1,C2,C3",
        3 + 8 + 1 + 1 + 16,
    ));
    f.extend(fmt(
        131,
        "ESC",
        "QIIIIccccBBBBB",
        "TimeUS,E0,E1,E2,E3,V0,V1,V2,V3,T0,T1,T2,T3,Fl",
        3 + 8 + 16 + 8 + 5,
    ));
    f.extend(fmt(
        134,
        "NAV",
        "QfffffffB",
        "TimeUS,Q1,Q2,Q3,Q4,GX,GY,GZ,Vld",
        3 + 8 + 28 + 1,
    ));
    f.extend(fmt(
        133,
        "IMU",
        "QBffffff",
        "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
        3 + 8 + 1 + 24,
    ));
    for i in 0..600u64 {
        let t = i * 10_000;
        let level: u16 = if t < 1_000_000 { 100 } else { 500 };
        let mut r = vec![0xA3, 0x95, 130];
        r.extend(t.to_le_bytes());
        r.extend([u8::from(i < 599), 0]);
        for _ in 0..4 {
            r.extend(level.to_le_bytes());
        }
        r.extend([0u8; 8]);
        f.extend(r);
        let mut r = vec![0xA3, 0x95, 131];
        r.extend(t.to_le_bytes());
        r.extend([0u8; 16 + 8 + 5]);
        f.extend(r);
        let roll = if t < 1_000_000 {
            0.0f32
        } else {
            0.5 * (t - 1_000_000) as f32 / 1e6
        };
        let h = roll.to_radians() / 2.0;
        let mut r = vec![0xA3, 0x95, 134];
        r.extend(t.to_le_bytes());
        for v in [h.cos(), h.sin(), 0.0, 0.0, 0.0, 0.0, 0.0] {
            r.extend(v.to_le_bytes());
        }
        r.push(1);
        f.extend(r);
        let mut r = vec![0xA3, 0x95, 133];
        r.extend(t.to_le_bytes());
        r.push(0);
        for v in [0.01f32, 0.0, 0.0, 0.0, 0.0, -9.80665] {
            r.extend(v.to_le_bytes());
        }
        f.extend(r);
    }
    f
}

#[test]
fn the_report_has_the_ground_check_and_the_closing_block_in_text_and_json() {
    let dir = TempDir::new("ground");
    let log = dir.file("ground.BIN");
    std::fs::write(&log, ground_log()).expect("write the log");
    let json = dir.file("report.json");
    let r = run(&format!(
        "log report {} --mass-kg 1 --kf 1e-9 --poles 14 --level-range 0 3000 --json {}",
        log.display(),
        json.display()
    ));
    assert_eq!(r.code, 0, "{r:?}");
    for part in [
        "Ground check of the tilt estimate (each ground run; reference: the 1 s before the run",
        "window 1 ground run 1: 1.000 s to 5.500 s",
        "reference: roll 0.00, pitch 0.00 (100 NAV, 100 IMU records); gyro bias x 0.573",
        "largest estimate shift from the reference: roll 2.1",
        "gyro-integrated rotation at the end of the run: about x 0.00, about y 0.00",
        "largest unexplained shift",
        "Values this log supports (valid only for the levels, speeds and battery voltage seen in this log)",
        "thrust constant k: no flight",
        "hover: no flight",
        "1 g window: no flight",
    ] {
        assert!(r.out.contains(part), "{part:?} missing in\n{}", r.out);
    }
    let value: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&json).expect("the JSON file"))
            .expect("JSON");
    let g = &value["ground_check"];
    assert_eq!(g["state"], "done");
    let f = &g["runs"][0]["figures"];
    assert!(
        (f["largest_estimate_shift_roll"]["deg"]
            .as_f64()
            .expect("number")
            - 2.125)
            .abs()
            < 0.05
    );
    assert_eq!(
        f["gyro_integrated_end_x_deg"]
            .as_f64()
            .expect("number")
            .abs()
            < 1e-3,
        true
    );
    assert!(f["largest_unexplained_shift_pitch"]["bin_start_us"].is_number());
    let v = &value["values_supported"];
    assert!(v["validity"].is_string());
    assert!(v["thrust_constant"].is_null());
    assert_eq!(v["hover"]["flights"], 0);
    assert!(v["motor_line_pooled"].is_object() || v["motor_line_pooled"].is_null());

    // No ground run in the other synthetic log; no NAV, no IMU in a log without them.
    let flight = dir.file("flight.BIN");
    std::fs::write(&flight, tracking_log()).expect("write the log");
    let r = run(&format!(
        "log report {} --mass-kg 1 --kf 3e-5 --poles 14",
        flight.display()
    ));
    assert!(r.out.contains("no ground run"), "{}", r.out);
    assert!(r.out.contains("thrust constant k (round 2)"), "{}", r.out);
    assert!(r.out.find("Tracking").unwrap() < r.out.find("Ground check").unwrap());
    assert!(r.out.find("Ground check").unwrap() < r.out.find("Values this log supports").unwrap());

    // Bad --level-range is a usage error.
    for bad in ["--level-range 5", "--level-range 3 3", "--level-range a b"] {
        let r = run(&format!(
            "log report {} --mass-kg 1 --kf 1e-9 {bad}",
            log.display()
        ));
        assert_eq!(r.code, 2, "{bad}: {r:?}");
    }
}
