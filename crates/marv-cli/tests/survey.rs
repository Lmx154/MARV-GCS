//! `marv-cli log survey`: synthetic logs written with the firmware's RCIN and GPS layouts (made-up positions in
//! the open ocean), the command run as a process with its own `XDG_DATA_HOME`, the list read back by the crate's
//! own `.waypoints` reader. Files only: no link, no vehicle.

mod common;

use std::path::PathBuf;
use std::process::Command;

use common::TempDir;
use marv_cli::args::{Parsed, parse};
use marv_cli::waypoints;

const GPS: u8 = 134;
const RCIN: u8 = 136;

fn padded(text: &str, size: usize) -> Vec<u8> {
    let mut v = text.as_bytes().to_vec();
    v.resize(size, 0);
    v
}

fn fmt(type_id: u8, name: &str, format: &str, columns: &str, length: usize) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 128, type_id, length as u8];
    r.extend(padded(name, 4));
    r.extend(padded(format, 16));
    r.extend(padded(columns, 64));
    r
}

fn header() -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt(
        GPS,
        "GPS",
        "QBBIHBcLLeff",
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
        42,
    ));
    f.extend(fmt(
        RCIN,
        "RCIN",
        "QHHHHHHHHHHHHHH",
        "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
        39,
    ));
    f
}

fn rcin(t_us: u64, c9: u16) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, RCIN];
    r.extend(t_us.to_le_bytes());
    for c in 1..=14u16 {
        r.extend((if c == 9 { c9 } else { 1500 }).to_le_bytes());
    }
    r
}

fn gps_hdop(t_us: u64, status: u8, lat: f64, lon: f64, hdop: i16) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, GPS];
    r.extend(t_us.to_le_bytes());
    r.push(0); // I
    r.push(status);
    r.extend(3_600_000u32.to_le_bytes()); // GMS: one hour into the week
    r.extend(2400u16.to_le_bytes()); // GWk: the week of 2026-01-04
    r.push(11); // NSats
    r.extend(hdop.to_le_bytes()); // HDop, 0.01 units
    r.extend(((lat * 1e7).round() as i32).to_le_bytes());
    r.extend(((lon * 1e7).round() as i32).to_le_bytes());
    r.extend(1000i32.to_le_bytes());
    r.extend(0f32.to_le_bytes());
    r.extend(0f32.to_le_bytes());
    r
}

/// A press: the channel high from `start_s` for `samples` RCIN samples (0.1 s apart), the fix status during it, and
/// where the receiver was.
struct Press {
    start_s: f64,
    samples: u32,
    status: u8,
    lat: f64,
    lon: f64,
}

/// Where the receiver is when no press is on (not a survey point).
const DECOY: (f64, f64) = (0.9000001, -22.0000001);

/// The point of press `i` of a test: made up, in the open ocean, 0.001 degrees (about 111 m) apart.
fn spot(i: usize) -> (f64, f64) {
    (0.4173529 + 0.001 * i as f64, -23.8641927 + 0.001 * i as f64)
}

fn press(i: usize, start_s: f64, samples: u32, status: u8) -> Press {
    let (lat, lon) = spot(i);
    Press {
        start_s,
        samples,
        status,
        lat,
        lon,
    }
}

/// RCIN at 10 Hz and GPS at 5 Hz for `seconds`; channel 9 low (1000) except in the presses (2000); in the window
/// of a press (0.3 s each side) the fixes are at its place with one wild fix among them.
fn log(presses: &[Press], seconds: u64) -> Vec<u8> {
    log_hdop(presses, seconds, 120)
}

fn log_hdop(presses: &[Press], seconds: u64, hdop: i16) -> Vec<u8> {
    let mut f = header();
    let us = |s: f64| (s * 1e6).round() as u64;
    let window = |p: &Press| {
        (
            us(p.start_s) - 300_000,
            us(p.start_s) + u64::from(p.samples - 1) * 100_000 + 300_000,
        )
    };
    for k in 0..seconds * 10 {
        let t = k * 100_000;
        let high = presses
            .iter()
            .any(|p| t >= us(p.start_s) && t < us(p.start_s) + u64::from(p.samples) * 100_000);
        f.extend(rcin(t, if high { 2000 } else { 1000 }));
    }
    for k in 0..seconds * 5 {
        let t = k * 200_000;
        match presses.iter().find(|p| {
            let (a, b) = window(p);
            t >= a && t <= b
        }) {
            Some(p) => f.extend(gps_hdop(t, p.status, p.lat, p.lon, hdop)),
            None => f.extend(gps_hdop(t, 3, DECOY.0, DECOY.1, hdop)),
        }
    }
    for p in presses {
        f.extend(gps_hdop(us(p.start_s) + 50_000, p.status, 3.5, 7.5, hdop)); // the wild fix
    }
    f
}

struct Ran {
    code: i32,
    out: String,
    err: String,
    store: PathBuf,
}

fn survey(dir: &TempDir, bytes: &[u8], args: &str) -> Ran {
    let file = dir.file("00000007.BIN");
    std::fs::write(&file, bytes).unwrap();
    let store = dir.file("data");
    let output = Command::new(env!("CARGO_BIN_EXE_marv-cli"))
        .arg("log")
        .arg("survey")
        .arg(&file)
        .args(args.split_whitespace())
        .env("XDG_DATA_HOME", &store)
        .env_remove("HOME")
        .output()
        .expect("run marv-cli");
    Ran {
        code: output.status.code().unwrap_or(-1),
        out: String::from_utf8_lossy(&output.stdout).into_owned(),
        err: String::from_utf8_lossy(&output.stderr).into_owned(),
        store: store.join("marv-gcs").join("missions"),
    }
}

fn list(ran: &Ran, name: &str) -> Vec<gcs_mavlink::services::missions::MissionPoint> {
    let text = std::fs::read_to_string(ran.store.join(format!("{name}.waypoints"))).expect("list");
    waypoints::parse(&text)
        .expect("the reader accepts the file")
        .0
}

fn e7(x: f64) -> i32 {
    (x * 1e7).round() as i32
}

#[test]
fn an_hdop_that_is_not_available_prints_n_a() {
    let dir = TempDir::new("survey-hdop");
    let ran = survey(
        &dir,
        &log_hdop(&[press(0, 10.0, 6, 3)], 20, i16::MAX),
        "--name na",
    );
    assert_eq!(ran.code, 0, "{}", ran.err);
    assert!(ran.out.contains("hdop n/a"), "{}", ran.out);
    assert!(!ran.out.contains("327"), "{}", ran.out);
}

#[test]
fn three_presses_give_three_points_at_the_medians() {
    let dir = TempDir::new("survey-three");
    let presses = [
        press(0, 10.0, 6, 3),
        press(1, 20.0, 4, 4),
        press(2, 30.0, 3, 3),
    ];
    let ran = survey(&dir, &log(&presses, 40), "--name field");
    assert_eq!(ran.code, 0, "{}", ran.err);
    let points = list(&ran, "field");
    assert_eq!(points.len(), 3);
    for (i, p) in points.iter().enumerate() {
        let (lat, lon) = spot(i);
        assert_eq!((p.lat_e7, p.lon_e7), (e7(lat), e7(lon)), "point {i}");
        assert_eq!(p.z, 5.0);
    }
    // press 1: 6 high samples (10.0 to 10.5 s), widened to 9.7 to 10.8 s: 6 receiver fixes and the wild one
    assert!(
        ran.out.contains("press 1: t 10.0 s, held 0.5 s, 7 fixes"),
        "{}",
        ran.out
    );
    assert!(ran.out.contains("11 satellites at least"), "{}", ran.out);
    assert!(ran.out.contains("hdop 1.20 at most"), "{}", ran.out);
    assert!(ran.out.contains("first point"), "{}", ran.out);
    assert!(ran.out.contains("press 2:"), "{}", ran.out);
    // 0.001 degrees of latitude and of longitude at 0.4 degrees north is about 157 m
    assert!(
        ran.out.contains("157.3 m from the previous point"),
        "{}",
        ran.out
    );
    assert!(
        ran.out
            .contains("list \"field\": 3 waypoints at 5 m above take-off"),
        "{}",
        ran.out
    );
    assert!(ran.out.contains(ran.store.to_str().unwrap()), "{}", ran.out);
}

#[test]
fn a_press_without_a_3d_fix_is_skipped_and_reported() {
    let dir = TempDir::new("survey-nofix");
    let presses = [
        press(0, 10.0, 5, 3),
        press(1, 20.0, 5, 3),
        press(2, 30.0, 5, 2),
        press(3, 40.0, 5, 3),
    ];
    let ran = survey(&dir, &log(&presses, 50), "--name field --alt 7.5");
    assert_eq!(ran.code, 0, "{}", ran.err);
    assert!(ran.out.contains("press 3: no 3D fix"), "{}", ran.out);
    let points = list(&ran, "field");
    assert_eq!(points.len(), 3);
    assert_eq!(points[2].lat_e7, e7(spot(3).0));
    assert!(points.iter().all(|p| p.z == 7.5));
}

#[test]
fn a_one_sample_blip_is_not_a_press() {
    let dir = TempDir::new("survey-blip");
    let presses = [press(0, 10.0, 1, 3), press(1, 20.0, 4, 3)];
    let ran = survey(&dir, &log(&presses, 30), "--name field");
    assert_eq!(ran.code, 0, "{}", ran.err);
    let points = list(&ran, "field");
    assert_eq!(points.len(), 1);
    assert_eq!(points[0].lat_e7, e7(spot(1).0));
    assert!(ran.out.contains("1 waypoints"), "{}", ran.out);
}

#[test]
fn a_channel_that_never_moves_gives_no_file() {
    let dir = TempDir::new("survey-still");
    // channel 9 moves only 300 raw units: under the 400 needed
    let mut bytes = header();
    for k in 0..100u64 {
        bytes.extend(rcin(k * 100_000, if k % 20 < 10 { 1000 } else { 1300 }));
    }
    let ran = survey(&dir, &bytes, "--name field");
    assert_eq!(ran.code, 0, "{}", ran.err);
    assert!(
        ran.out.contains("channel 9 never moved: no survey points"),
        "{}",
        ran.out
    );
    assert!(!ran.store.exists(), "no folder, no file");
    // another channel of the same log never moves either
    let ran = survey(&dir, &bytes, "--name field --channel 3");
    assert!(ran.out.contains("channel 3 never moved"), "{}", ran.out);
}

#[test]
fn eighteen_presses_give_sixteen_points() {
    let dir = TempDir::new("survey-many");
    let presses: Vec<Press> = (0..18)
        .map(|i| press(i, 10.0 + 5.0 * i as f64, 4, 3))
        .collect();
    let ran = survey(&dir, &log(&presses, 110), "--name field");
    assert_eq!(ran.code, 0, "{}", ran.err);
    let points = list(&ran, "field");
    assert_eq!(points.len(), 16);
    assert_eq!(points[15].lat_e7, e7(spot(15).0));
    assert!(
        ran.out
            .contains("2 more presses left out: a mission holds 16 waypoints"),
        "{}",
        ran.out
    );
    assert!(ran.out.contains("16 waypoints"), "{}", ran.out);
}

#[test]
fn the_file_is_the_stations_format_and_reads_back() {
    let dir = TempDir::new("survey-format");
    let presses = [press(0, 10.0, 5, 3), press(1, 20.0, 5, 3)];
    let ran = survey(&dir, &log(&presses, 30), "--name field --alt 12");
    assert_eq!(ran.code, 0, "{}", ran.err);
    let text = std::fs::read_to_string(ran.store.join("field.waypoints")).unwrap();
    let rows: Vec<&str> = text.lines().collect();
    assert_eq!(rows[0], "QGC WPL 110");
    assert_eq!(rows.len(), 4);
    let (lat0, lon0) = spot(0);
    assert_eq!(
        rows[1],
        format!("0\t1\t0\t16\t0\t0\t0\t0\t{lat0:.8}\t{lon0:.8}\t0\t1")
    );
    let (lat1, lon1) = spot(1);
    assert_eq!(
        rows[3],
        format!("2\t0\t3\t16\t0\t0\t0\t0\t{lat1:.8}\t{lon1:.8}\t12\t1")
    );
    let points = list(&ran, "field");
    assert_eq!(points.len(), 2);
    assert_eq!(
        (points[1].lat_e7, points[1].lon_e7, points[1].z),
        (e7(lat1), e7(lon1), 12.0)
    );
    // the same text the crate's own writer gives for the points it read
    assert_eq!(waypoints::to_wpl(&points, None), text);
}

#[test]
fn nothing_printed_has_a_coordinate() {
    let dir = TempDir::new("survey-private");
    let presses = [
        press(0, 10.0, 5, 3),
        press(1, 20.0, 5, 3),
        press(2, 30.0, 5, 2),
    ];
    let ran = survey(&dir, &log(&presses, 40), "");
    assert_eq!(ran.code, 0, "{}", ran.err);
    let shown = format!("{}{}", ran.out, ran.err);
    for i in 0..2 {
        let (lat, lon) = spot(i);
        for text in [
            format!("{lat:.7}"),
            format!("{lon:.7}"),
            format!("{lat:.8}"),
            format!("{lon:.8}"),
            format!("{lat}"),
            format!("{lon}"),
            format!("{}", e7(lat)),
            format!("{}", e7(lon)),
            "4173529".to_string(),
            "8641927".to_string(),
        ] {
            assert!(!shown.contains(&text), "{text} is printed in {shown}");
        }
    }
    // the default name: the date of the log's GPS week (2026-01-04) and the file's stem
    assert!(
        ran.out
            .contains("list \"survey-20260104-00000007\": 2 waypoints"),
        "{}",
        ran.out
    );
    assert!(
        ran.store
            .join("survey-20260104-00000007.waypoints")
            .exists()
    );
}

#[test]
fn an_existing_list_is_kept_without_force() {
    let dir = TempDir::new("survey-force");
    let presses = [press(0, 10.0, 5, 3), press(1, 20.0, 5, 3)];
    let bytes = log(&presses, 30);
    assert_eq!(survey(&dir, &bytes, "--name field").code, 0);
    let path = dir.file("data/marv-gcs/missions/field.waypoints");
    let first = std::fs::read_to_string(&path).unwrap();
    let again = survey(&dir, &log(&presses[..1], 30), "--name field");
    assert_eq!(again.code, 1);
    assert!(again.err.contains("already exists"), "{}", again.err);
    assert_eq!(std::fs::read_to_string(&path).unwrap(), first);
    let forced = survey(&dir, &log(&presses[..1], 30), "--name field --force");
    assert_eq!(forced.code, 0, "{}", forced.err);
    assert_eq!(list(&forced, "field").len(), 1);
}

#[test]
fn the_arguments() {
    let words = |s: &str| s.split_whitespace().map(str::to_string).collect::<Vec<_>>();
    let Ok(Parsed::LogSurvey(c)) = parse(&words("log survey a.BIN")) else {
        panic!("parsed")
    };
    assert_eq!((c.alt_m, c.channel, c.force, c.name), (5.0, 9, false, None));
    let Ok(Parsed::LogSurvey(c)) = parse(&words(
        "log survey a.BIN --name x --alt 8 --channel 11 --force",
    )) else {
        panic!("parsed")
    };
    assert_eq!(
        (c.alt_m, c.channel, c.force, c.name.as_deref()),
        (8.0, 11, true, Some("x"))
    );
    for bad in [
        "log survey",
        "log survey a.BIN b.BIN",
        "log survey a.BIN --channel 15",
        "log survey a.BIN --channel 0",
        "log survey a.BIN --alt 0",
        "log survey a.BIN --alt",
        "log survey a.BIN --nope",
    ] {
        assert!(parse(&words(bad)).is_err(), "{bad}");
    }
    assert!(marv_cli::args::USAGE.contains("marv-cli log survey <file.BIN>"));
}
