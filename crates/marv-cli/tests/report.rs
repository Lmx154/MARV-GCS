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
