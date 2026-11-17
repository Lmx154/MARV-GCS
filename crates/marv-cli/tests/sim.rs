//! The CLI against the simulated vehicle over a real TCP socket, in real time.

mod common;

use std::net::TcpListener;
use std::time::{Duration, Instant};

use common::{Rig, run};

fn last(lines: &[&str]) -> String {
    lines.last().map(|l| (*l).to_string()).unwrap_or_default()
}

#[test]
fn status_shows_the_vehicle() {
    let rig = Rig::start();
    let session = connected(&rig);
    // A text that certainly arrives: the vehicle says why it refuses a write.
    let r = on(&session, "param set SYS_UID 5");
    assert_eq!(r.code, 1, "{r:?}");
    let r = on(&session, "status");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.err, "", "status shows its texts on stdout only: {r:?}");
    let lines = r.lines();
    assert!(
        lines[0].starts_with("vehicle\tsystem=1 component=1 type="),
        "{r:?}"
    );
    assert!(lines.contains(&"role\tConfigure"), "{r:?}");
    assert!(lines.iter().any(|l| l.starts_with("health\t")), "{r:?}");
    assert!(lines.contains(&"devices\t8"), "{r:?}");
    assert!(
        lines
            .iter()
            .any(|l| l.starts_with("text\t") && l.ends_with("\tWARNING\tSYS_UID: read only")),
        "{r:?}"
    );
}

#[test]
fn devices_prints_eight_rows() {
    let rig = Rig::start();
    let r = rig.cli("devices --for 1.5");
    assert_eq!(r.code, 0, "{r:?}");
    let rows: Vec<Vec<&str>> = r.lines().iter().map(|l| l.split('\t').collect()).collect();
    assert_eq!(rows.len(), 8, "{r:?}");
    let names: Vec<&str> = rows.iter().map(|c| c[0]).collect();
    assert_eq!(
        names,
        ["IMU1", "IMU2", "BARO", "MAG", "GNSS", "BATT", "RC", "SD"]
    );
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

/// A session on the rig's vehicle that has seen the heartbeat.
fn connected(rig: &Rig) -> gcs_mavlink::Session {
    let endpoint = format!("tcp:{}", rig.addr).parse().expect("endpoint");
    let session =
        gcs_link::start_session(&endpoint, gcs_mavlink::Access::Configure, Box::new(|| {}))
            .expect("open");
    let deadline = Instant::now() + Duration::from_secs(10);
    while session.state().link.vehicle.is_none() {
        assert!(Instant::now() < deadline, "never connected");
        std::thread::sleep(Duration::from_millis(10));
    }
    session
}

/// Runs the command line on a session that exists already.
fn on(session: &gcs_mavlink::Session, words: &str) -> common::Output {
    let args: Vec<String> = format!("--connect tcp:127.0.0.1:5760 {words}")
        .split_whitespace()
        .map(str::to_string)
        .collect();
    let Ok(marv_cli::args::Parsed::Run(invocation)) = marv_cli::args::parse(&args) else {
        panic!("{words}: does not parse");
    };
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let code = marv_cli::commands::execute(
        &invocation,
        session,
        gcs_mavlink::Access::Configure,
        &mut out,
        &mut err,
    );
    common::Output {
        code,
        out: String::from_utf8_lossy(&out).into_owned(),
        err: String::from_utf8_lossy(&err).into_owned(),
    }
}

#[test]
fn texts_and_watch_show_what_the_vehicle_says() {
    let rig = Rig::start();
    let session = connected(&rig);
    // A refused action makes the vehicle say why, in a STATUSTEXT.
    session.send(gcs_mavlink::Command::SendCommand {
        command: gcs_mavlink::dialect::MavCmd::MAV_CMD_MARV_DEVICE,
        params: [7.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    let r = on(&session, "texts --for 1");
    assert_eq!(r.code, 0, "{r:?}");
    // The vehicle's boot text may or may not have reached this first connection: one line or two.
    assert!(!r.lines().is_empty() && r.lines().len() <= 2, "{r:?}");
    for line in r.lines() {
        let cells: Vec<&str> = line.split('\t').collect();
        assert_eq!(cells.len(), 3, "{r:?}");
        assert!(
            cells[0].parse::<f64>().is_ok() && cells[1] == "INFO" && !cells[2].is_empty(),
            "{r:?}"
        );
    }
    let r = on(&session, "watch --for 2.5");
    assert_eq!(r.code, 0, "{r:?}");
    let lines = r.lines();
    let devices = lines.iter().filter(|l| l.starts_with("device\t")).count();
    assert_eq!(
        devices, 8,
        "one line per device while nothing changes: {r:?}"
    );
    assert!(
        lines.iter().any(|l| l.starts_with("text\t")),
        "the text is listed again by watch: {r:?}"
    );
}

#[test]
fn param_list_prints_the_simulators_parameters() {
    let rig = Rig::start();
    let r = rig.cli("param list");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.lines().len(), 70, "{r:?}");
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
fn a_refused_param_set_shows_the_vehicles_reason_as_a_text_line_on_stderr() {
    let rig = Rig::start();
    let r = rig.cli("param set SYS_UID 5");
    assert_eq!(r.code, 1, "{r:?}");
    assert_eq!(r.out, "SYS_UID\t1592460852\ti32\n", "{r:?}");
    let lines: Vec<&str> = r.err.lines().collect();
    assert!(
        lines.contains(&"text\tWARNING\tSYS_UID: read only"),
        "{r:?}"
    );
    assert_eq!(
        lines.iter().filter(|l| l.contains("SYS_UID")).count(),
        2,
        "one text line and the refusal message, no repeats: {r:?}"
    );
    assert!(
        r.err.contains("asked 5") && r.err.contains("keeps 1592460852"),
        "{r:?}"
    );
    let text_at = lines.iter().position(|l| l.starts_with("text\t")).unwrap();
    let refusal_at = lines.iter().position(|l| l.contains("asked 5")).unwrap();
    assert!(text_at < refusal_at, "the reason comes first: {r:?}");
}

/// The simulator's `stale_echo_delay_ms` makes the order: a PARAM_VALUE with the OLD value right
/// after the PARAM_SET (read by the service as a refusal), the accepting echo 50 ms later.
#[test]
fn a_stale_old_value_before_the_accepting_echo_is_not_reported_as_a_refusal() {
    let rig = Rig::with(gcs_sim::SimConfig {
        stale_echo_delay_ms: 50,
        ..gcs_sim::SimConfig::default()
    });
    let r = rig.cli("param set LOG_RATE_HZ 200");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.out, "LOG_RATE_HZ\t200\ti32\n", "{r:?}");
    assert!(!r.err.contains("keeps"), "{r:?}");
    let r = rig.cli("param get LOG_RATE_HZ");
    assert_eq!(r.out, "LOG_RATE_HZ\t200\ti32\n", "{r:?}");
    // The plain refusal is still a refusal.
    let r = rig.cli("param set SYS_UID 5");
    assert_eq!(r.code, 1, "{r:?}");
    assert_eq!(r.out, "SYS_UID\t1592460852\ti32\n", "{r:?}");
}

#[test]
fn an_accepted_param_set_has_no_text_line() {
    let rig = Rig::start();
    // The boot text belongs to the first session only.
    let r = rig.cli("status --for 0.3");
    assert_eq!(r.code, 0, "{r:?}");
    let r = rig.cli("param set LOG_RATE_HZ 200");
    assert_eq!(r.code, 0, "{r:?}");
    assert_eq!(r.out, "LOG_RATE_HZ\t200\ti32\n", "{r:?}");
    assert!(
        !r.err.lines().any(|l| l.starts_with("text\t")),
        "no text line: {r:?}"
    );
}

#[test]
fn texts_prints_on_stdout_only_and_a_text_before_the_command_is_shown() {
    let rig = Rig::start();
    let session = connected(&rig);
    let r = on(&session, "param set SYS_UID 5");
    assert_eq!(r.code, 1, "{r:?}");
    assert!(r.err.contains("text\tWARNING\tSYS_UID: read only"), "{r:?}");
    let r = on(&session, "texts --for 0.5");
    assert_eq!(r.code, 0, "{r:?}");
    assert!(
        r.out
            .lines()
            .any(|l| l.ends_with("\tWARNING\tSYS_UID: read only")),
        "{r:?}"
    );
    assert_eq!(r.err, "", "no second copy on stderr: {r:?}");
    // Another command on the same session shows the earlier text, as the session's own.
    let r = on(&session, "devices --for 0.3");
    assert_eq!(r.code, 0, "{r:?}");
    assert!(
        r.err
            .lines()
            .any(|l| l == "text\tWARNING\tSYS_UID: read only"),
        "{r:?}"
    );
    assert!(!r.out.contains("SYS_UID"), "stdout is unchanged: {r:?}");
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
fn calibrate_compass_prints_the_hint_once_and_ends_accepted_with_progress() {
    let rig = Rig::start();
    let r = rig.cli("calibrate compass");
    assert_eq!(r.code, 0, "{r:?}");
    let lines = r.lines();
    assert_eq!(last(&lines), "compass calibration: accepted", "{r:?}");
    assert_eq!(
        lines
            .iter()
            .filter(|l| **l
                == "compass calibration: rotate the vehicle through all orientations for 30 s")
            .count(),
        1,
        "{r:?}"
    );
    assert_eq!(
        lines.first().copied(),
        Some("compass calibration: rotate the vehicle through all orientations for 30 s"),
        "{r:?}"
    );
    assert!(r.out.contains("compass calibration: 0 %"), "{r:?}");
    assert!(
        lines.iter().any(|l| l.starts_with("compass calibration: ")
            && l.ends_with(" %")
            && !l.contains(": 0 %")),
        "{r:?}"
    );
}

#[test]
fn calibrate_cancel_during_a_compass_calibration_stops_it() {
    let rig = Rig::start();
    {
        let endpoint = format!("tcp:{}", rig.addr).parse().expect("endpoint");
        let session =
            gcs_link::start_session(&endpoint, gcs_mavlink::Access::Configure, Box::new(|| {}))
                .expect("open");
        let deadline = Instant::now() + Duration::from_secs(10);
        while session.state().link.vehicle.is_none() {
            assert!(Instant::now() < deadline, "never connected");
            std::thread::sleep(Duration::from_millis(10));
        }
        session.send(gcs_mavlink::Command::SendCommand {
            command: gcs_mavlink::dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            params: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
        let deadline = Instant::now() + Duration::from_secs(10);
        while !matches!(
            session.state().command.as_ref().map(|r| r.status.clone()),
            Some(gcs_mavlink::CommandStatus::InProgress { .. })
        ) {
            assert!(Instant::now() < deadline, "the calibration never started");
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    let cancel = rig.cli("calibrate cancel");
    assert_eq!(cancel.code, 0, "{cancel:?}");
    assert_eq!(
        last(&cancel.lines()),
        "cancel calibration: accepted",
        "{cancel:?}"
    );
    let again = rig.cli("calibrate cancel");
    assert_eq!(again.code, 1, "{again:?}");
    assert_eq!(
        last(&again.lines()),
        "cancel calibration: denied",
        "{again:?}"
    );
}

/// The simulator serves one client at a time, as the board's one USB port does: a calibration
/// started by one invocation can be cancelled by the next one only after the first has let go of the
/// link (Ctrl-C), and the vehicle goes on calibrating meanwhile.
#[test]
fn calibrate_cancel_during_a_calibration_stops_it_and_a_second_cancel_has_nothing_to_cancel() {
    let rig = Rig::start();
    {
        let endpoint = format!("tcp:{}", rig.addr).parse().expect("endpoint");
        let session =
            gcs_link::start_session(&endpoint, gcs_mavlink::Access::Configure, Box::new(|| {}))
                .expect("open");
        let deadline = Instant::now() + Duration::from_secs(10);
        while session.state().link.vehicle.is_none() {
            assert!(Instant::now() < deadline, "never connected");
            std::thread::sleep(Duration::from_millis(10));
        }
        session.send(gcs_mavlink::Command::SendCommand {
            command: gcs_mavlink::dialect::MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            params: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
        let deadline = Instant::now() + Duration::from_secs(10);
        while !matches!(
            session.state().command.as_ref().map(|r| r.status.clone()),
            Some(gcs_mavlink::CommandStatus::InProgress { .. })
        ) {
            assert!(Instant::now() < deadline, "the calibration never started");
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    let cancel = rig.cli("calibrate cancel");
    assert_eq!(cancel.code, 0, "{cancel:?}");
    assert_eq!(
        last(&cancel.lines()),
        "cancel calibration: accepted",
        "{cancel:?}"
    );
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
fn a_governed_sensor_write_without_debug_sends_nothing_and_does_not_even_connect() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    listener.set_nonblocking(true).expect("nonblocking");
    let addr = listener.local_addr().expect("address");
    for words in [
        "param set BARO_ODR_HZ 25",
        "param set IMU1_GYR_FS 250",
        "param set MAG_AVG 4",
    ] {
        let r = run(&format!("--connect tcp:{addr} {words}"));
        assert_eq!(r.code, 2, "{words}: {r:?}");
        assert!(
            r.err.contains("--debug") && r.err.contains("refused") && r.out.is_empty(),
            "{words}: {r:?}"
        );
        assert_eq!(r.err.lines().filter(|l| l.contains("--debug")).count(), 1);
    }
    assert!(listener.accept().is_err(), "a connection was made");
}

#[test]
fn with_debug_a_governed_field_is_written_and_read_back() {
    let rig = Rig::start();
    let r = rig.cli("param set BARO_ODR_HZ 25 --debug");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "BARO_ODR_HZ\t25\ti32\n"),
        "{r:?}"
    );
    let r = rig.cli("--debug param set BARO_IIR_T 3");
    assert_eq!(
        (r.code, r.out.as_str()),
        (0, "BARO_IIR_T\t3\ti32\n"),
        "{r:?}"
    );
    let r = rig.cli("param get BARO_ODR_HZ");
    assert_eq!(r.out, "BARO_ODR_HZ\t25\ti32\n", "{r:?}");
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
