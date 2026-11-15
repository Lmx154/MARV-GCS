//! The CLI against the simulated vehicle over a real TCP socket, in real time.

mod common;

use std::net::TcpListener;
use std::time::{Duration, Instant};

use common::{Rig, Shared, run};

fn last(lines: &[&str]) -> String {
    lines.last().map(|l| (*l).to_string()).unwrap_or_default()
}

#[test]
fn status_shows_the_vehicle() {
    let rig = Rig::start();
    let r = rig.cli("status");
    assert_eq!(r.code, 0, "{r:?}");
    let lines = r.lines();
    assert!(
        lines[0].starts_with("vehicle\tsystem=1 component=1 type="),
        "{r:?}"
    );
    assert!(lines.contains(&"role\tConfigure"), "{r:?}");
    assert!(lines.iter().any(|l| l.starts_with("health\t")), "{r:?}");
    assert!(lines.contains(&"devices\t7"), "{r:?}");
    assert!(
        lines
            .iter()
            .any(|l| l.starts_with("text\t") && l.contains("\tINFO\t")),
        "{r:?}"
    );
}

#[test]
fn devices_prints_seven_rows() {
    let rig = Rig::start();
    let r = rig.cli("devices --for 1.5");
    assert_eq!(r.code, 0, "{r:?}");
    let rows: Vec<Vec<&str>> = r.lines().iter().map(|l| l.split('\t').collect()).collect();
    assert_eq!(rows.len(), 7, "{r:?}");
    let names: Vec<&str> = rows.iter().map(|c| c[0]).collect();
    assert_eq!(names, ["IMU1", "IMU2", "BARO", "MAG", "GNSS", "BATT", "RC"]);
    for cells in &rows {
        assert_eq!(cells.len(), 8, "{cells:?}");
        assert_eq!(&cells[1..3], ["OK", "NONE"], "{cells:?}");
        assert_eq!(&cells[4..7], ["0", "0", "0"], "{cells:?}");
        assert!(
            cells[3].parse::<u64>().is_ok() && cells[7].parse::<u32>().is_ok(),
            "{cells:?}"
        );
    }
}

#[test]
fn texts_shows_the_boot_text_and_watch_shows_rows_and_texts() {
    let rig = Rig::start();
    let r = rig.cli("texts --for 1");
    assert_eq!(r.code, 0, "{r:?}");
    let line = r.lines()[0].to_string();
    let cells: Vec<&str> = line.split('\t').collect();
    assert_eq!(cells.len(), 3, "{r:?}");
    assert!(
        cells[0].parse::<f64>().is_ok() && cells[1] == "INFO",
        "{r:?}"
    );
    let r = rig.cli("watch --for 2.5");
    assert_eq!(r.code, 0, "{r:?}");
    let devices = r
        .lines()
        .iter()
        .filter(|l| l.starts_with("device\t"))
        .count();
    assert_eq!(
        devices, 7,
        "one line per device while nothing changes: {r:?}"
    );
}

#[test]
fn param_list_prints_the_simulators_parameters() {
    let rig = Rig::start();
    let r = rig.cli("param list");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.lines().len(), 13, "{r:?}");
    assert!(r.lines().contains(&"IMU_RATE_HZ\t800\ti32"), "{r:?}");
    assert!(r.lines().contains(&"BAT_LOW_V\t14\tf32"), "{r:?}");
    assert!(r.lines().contains(&"NAV_TEST_16CHARS\t1.5\tf32"), "{r:?}");
}

#[test]
fn param_get_one_value_and_an_unknown_name() {
    let rig = Rig::start();
    let r = rig.cli("param get LOG_RATE_HZ");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "LOG_RATE_HZ\t100\ti32\n"),
        "{r:?}"
    );
    let r = rig.cli("param get NOPE");
    assert_eq!(r.code, 1, "{r:?}");
    assert!(
        r.err.contains("unknown parameter NOPE") && r.out.is_empty(),
        "{r:?}"
    );
}

#[test]
fn param_set_prints_the_echo_and_the_value_stays() {
    let rig = Rig::start();
    let r = rig.cli("param set LOG_RATE_HZ 200");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "LOG_RATE_HZ\t200\ti32\n"),
        "{r:?}"
    );
    let r = rig.cli("param get LOG_RATE_HZ");
    assert_eq!(
        r.out, "LOG_RATE_HZ\t200\ti32\n",
        "a second invocation reads it back: {r:?}"
    );
    let r = rig.cli("param set BAT_LOW_V 13.5");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "BAT_LOW_V\t13.5\tf32\n"),
        "{r:?}"
    );
}

#[test]
fn param_set_refused_by_the_vehicle_prints_what_is_in_effect_and_fails() {
    let rig = Rig::start();
    let r = rig.cli("param set SYS_UID 5");
    assert_eq!(r.code, 1, "{r:?}");
    assert_eq!(
        r.out, "SYS_UID\t1592460852\ti32\n",
        "the value in effect is printed: {r:?}"
    );
    assert!(
        r.err.contains("asked 5") && r.err.contains("keeps 1592460852"),
        "{r:?}"
    );
    let r = rig.cli("param set LOG_RATE_HZ 1.5");
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("does not fit"), "{r:?}");
}

#[test]
fn param_save_ends_accepted_with_progress_lines() {
    let rig = Rig::start();
    let r = rig.cli("param save");
    assert_eq!(r.code, 0, "{r:?}");
    let lines = r.lines();
    assert_eq!(last(&lines), "save parameters: accepted", "{r:?}");
    assert!(
        lines
            .iter()
            .any(|l| l.starts_with("save parameters: ") && l.ends_with(" %")),
        "{r:?}"
    );
}

#[test]
fn calibrate_gyro_ends_accepted() {
    let rig = Rig::start();
    let r = rig.cli("calibrate gyro");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(last(&r.lines()), "gyro calibration: accepted", "{r:?}");
    assert!(r.out.contains("gyro calibration: 0 %"), "{r:?}");
}

#[test]
fn calibrate_cancel_during_a_calibration_ends_it_cancelled() {
    let rig = Rig::start();
    let shared = Shared::default();
    let calibration = {
        let (out, addr) = (shared.clone(), rig.addr.clone());
        std::thread::spawn(move || {
            let args: Vec<String> = format!("--connect tcp:{addr} calibrate gyro")
                .split_whitespace()
                .map(str::to_string)
                .collect();
            let (mut out, mut err) = (out, Vec::new());
            marv_cli::run(&args, &mut out, &mut err)
        })
    };
    let deadline = Instant::now() + Duration::from_secs(10);
    while !shared.text().contains(" %") {
        assert!(
            Instant::now() < deadline,
            "the calibration never started: {:?}",
            shared.text()
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    let cancel = rig.cli("calibrate cancel");
    assert_eq!(cancel.code, 0, "{cancel:?}");
    assert_eq!(
        last(&cancel.lines()),
        "cancel calibration: accepted",
        "{cancel:?}"
    );
    let code = calibration.join().expect("the calibration thread");
    assert_eq!(code, 1, "{:?}", shared.text());
    assert_eq!(
        last(&shared.text().lines().collect::<Vec<_>>()),
        "gyro calibration: cancelled"
    );
    // With nothing running the vehicle has nothing to cancel.
    let again = rig.cli("calibrate cancel");
    assert_eq!(again.code, 1, "{again:?}");
    assert_eq!(
        last(&again.lines()),
        "cancel calibration: denied",
        "{again:?}"
    );
}

#[test]
fn device_restart_baro_is_accepted_and_rc_is_denied() {
    let rig = Rig::start();
    let r = rig.cli("device restart BARO");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(last(&r.lines()), "restart BARO: accepted", "{r:?}");
    let r = rig.cli("device restart RC");
    assert_eq!(r.code, 1, "{r:?}");
    assert_eq!(last(&r.lines()), "restart RC: denied", "{r:?}");
    assert!(r.err.contains("restart RC: denied"), "{r:?}");
    let r = rig.cli("device selftest MAG");
    assert_eq!(r.code, 0, "{r:?}");
}

#[test]
fn reboot_without_yes_sends_nothing_and_does_not_even_connect() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    listener.set_nonblocking(true).expect("nonblocking");
    let addr = listener.local_addr().expect("address");
    for words in ["reboot", "param reset-all", "param reset-calibration"] {
        let r = run(&format!("--connect tcp:{addr} {words}"));
        assert_eq!(r.code, 2, "{r:?}");
        assert!(r.err.contains("--yes") && r.out.is_empty(), "{r:?}");
    }
    assert!(listener.accept().is_err(), "a connection was made");
}

#[test]
fn reboot_and_the_resets_with_yes_follow_to_the_result() {
    let rig = Rig::start();
    let r = rig.cli("reboot --yes");
    assert_eq!(
        (r.code, last(&r.lines())),
        (0, "reboot: accepted".to_string()),
        "{r:?}"
    );
    let r = rig.cli("param reset-calibration --yes");
    assert_eq!(r.code, 0, "{r:?}");
}

#[test]
fn no_vehicle_nothing_listening() {
    let addr = {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        listener.local_addr().expect("address")
    };
    let begun = Instant::now();
    let r = run(&format!("--connect tcp:{addr} --timeout 1 status"));
    assert_eq!(r.code, 3, "{r:?}");
    assert!(r.err.contains("cannot open") && r.out.is_empty(), "{r:?}");
    assert!(begun.elapsed() < Duration::from_secs(3));
}

#[test]
fn no_vehicle_a_silent_peer_gives_the_heartbeat_timeout() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("address");
    let begun = Instant::now();
    let r = run(&format!("--connect tcp:{addr} --timeout 1 status"));
    let took = begun.elapsed();
    assert_eq!(r.code, 3, "{r:?}");
    assert!(
        r.err.contains("no heartbeat from a vehicle within 1 s"),
        "{r:?}"
    );
    assert!(r.out.is_empty());
    assert!(
        took >= Duration::from_millis(900) && took < Duration::from_secs(3),
        "{took:?}"
    );
    drop(listener);
}

#[test]
fn record_writes_a_log_the_station_can_read_and_never_overwrites() {
    let rig = Rig::start();
    let path = std::env::temp_dir().join(format!("marv-cli-test-{}.tlog", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let r = rig.cli(&format!("record {} --for 1.5", path.display()));
    assert_eq!(r.code, 0, "{r:?}");
    let line = r.lines()[0].to_string();
    assert!(
        line.starts_with(&format!("recorded\t{}\t", path.display())),
        "{r:?}"
    );
    let file = std::fs::File::open(&path).expect("the log exists");
    let mut reader = gcs_mavlink::tlog::TlogReader::new(file);
    let mut records = 0;
    for _ in 0..100_000 {
        match reader.next() {
            Some(Ok(_)) => records += 1,
            _ => break,
        }
    }
    assert!(records > 10, "{records} records");
    let again = rig.cli(&format!("record {} --for 1", path.display()));
    assert_eq!(again.code, 1, "{again:?}");
    assert!(again.err.contains("never overwritten"), "{again:?}");
    let _ = std::fs::remove_file(&path);
}
