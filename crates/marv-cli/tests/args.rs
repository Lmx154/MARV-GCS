//! The command line: every command and option, and every error.

use std::path::PathBuf;
use std::time::Duration;

use gcs_mavlink::FlightCommand;
use marv_cli::args::{
    Action, Calibration, Command, Parsed, Storage, USAGE, device_id, mav_command, parse,
};

fn args(words: &str) -> Vec<String> {
    words.split_whitespace().map(str::to_string).collect()
}

fn command(words: &str) -> Command {
    match parse(&args(&format!("--connect tcp:127.0.0.1:5760 {words}"))) {
        Ok(Parsed::Run(invocation)) => invocation.command,
        other => panic!("{words}: {other:?}"),
    }
}

fn error(words: &str) -> String {
    match parse(&args(words)) {
        Err(message) => message,
        other => panic!("{words}: expected an error, got {other:?}"),
    }
}

#[test]
fn every_command_parses() {
    assert_eq!(command("status"), Command::Status);
    assert_eq!(command("devices"), Command::Devices);
    assert_eq!(command("texts"), Command::Texts);
    assert_eq!(command("watch"), Command::Watch);
    assert_eq!(command("param list"), Command::ParamList);
    assert_eq!(
        command("param get BAT_LOW_V"),
        Command::ParamGet("BAT_LOW_V".into())
    );
    assert_eq!(
        command("param set BAT_LOW_V 13.5"),
        Command::ParamSet("BAT_LOW_V".into(), 13.5)
    );
    assert_eq!(
        command("param set SYS_TRIM -3"),
        Command::ParamSet("SYS_TRIM".into(), -3.0)
    );
    assert_eq!(command("param save"), Command::Param(Storage::Save));
    assert_eq!(command("param reload"), Command::Param(Storage::Reload));
    assert_eq!(
        command("param reset-all --yes"),
        Command::Param(Storage::ResetAll)
    );
    assert_eq!(
        command("param reset-calibration --yes"),
        Command::Param(Storage::ResetCalibration)
    );
    assert_eq!(
        command("calibrate gyro"),
        Command::Calibrate(Calibration::Gyro)
    );
    assert_eq!(
        command("calibrate accel"),
        Command::Calibrate(Calibration::Accel)
    );
    assert_eq!(
        command("calibrate compass"),
        Command::Calibrate(Calibration::Compass)
    );
    assert_eq!(
        command("calibrate cancel"),
        Command::Calibrate(Calibration::Cancel)
    );
    assert_eq!(
        command("device restart baro"),
        Command::Device {
            action: Action::Restart,
            id: 3,
            name: "BARO".into()
        }
    );
    assert_eq!(
        command("device selftest IMU2"),
        Command::Device {
            action: Action::SelfTest,
            id: 2,
            name: "IMU2".into()
        }
    );
    assert_eq!(command("reboot --yes"), Command::Reboot);
    assert_eq!(
        command("record /tmp/x.tlog"),
        Command::Record(PathBuf::from("/tmp/x.tlog"))
    );
}

#[test]
fn every_option_parses_and_may_come_anywhere() {
    let Ok(Parsed::Run(i)) = parse(&args(
        "devices --for 3.5 --timeout 1 --connect serial:/dev/ttyACM0",
    )) else {
        panic!("did not parse");
    };
    assert_eq!(i.timeout, Duration::from_secs(1));
    assert_eq!(i.duration, Some(Duration::from_secs_f64(3.5)));
    assert_eq!(i.endpoint.to_string(), "serial:/dev/ttyACM0:115200");
    let Ok(Parsed::Run(i)) = parse(&args("--connect udp:0.0.0.0:14550 status")) else {
        panic!("did not parse");
    };
    assert_eq!(
        i.timeout,
        Duration::from_secs(5),
        "the heartbeat timeout defaults to 5 s"
    );
    assert_eq!(i.duration, None);
    assert_eq!(parse(&args("--help")), Ok(Parsed::Help));
    assert_eq!(parse(&args("status --help")), Ok(Parsed::Help));
    assert!(USAGE.contains("Exit status"));
}

#[test]
fn the_errors() {
    assert!(error("--connect tcp:127.0.0.1:5760").contains("missing command"));
    assert!(error("status").contains("missing --connect"));
    assert!(error("--connect tcp:127.0.0.1:5760 frobnicate").contains("unknown command"));
    assert!(error("--connect nonsense status").contains("invalid endpoint"));
    assert!(error("--connect").contains("needs a value"));
    assert!(error("--connect tcp:127.0.0.1:5760 --colour status").contains("unknown option"));
    assert!(error("--connect tcp:127.0.0.1:5760 param").contains("param needs"));
    assert!(error("--connect tcp:127.0.0.1:5760 param get").contains("exactly one NAME"));
    assert!(error("--connect tcp:127.0.0.1:5760 param set A").contains("NAME and a VALUE"));
    assert!(error("--connect tcp:127.0.0.1:5760 param set A x").contains("needs a number"));
    assert!(error("--connect tcp:127.0.0.1:5760 param set A NaN").contains("needs a number"));
    assert!(error("--connect tcp:127.0.0.1:5760 param nothing").contains("unknown param command"));
    assert!(error("--connect tcp:127.0.0.1:5760 calibrate").contains("calibrate needs"));
    assert!(error("--connect tcp:127.0.0.1:5760 calibrate mag").contains("unknown calibration"));
    assert!(error("--connect tcp:127.0.0.1:5760 device").contains("restart or selftest"));
    assert!(error("--connect tcp:127.0.0.1:5760 device restart").contains("needs a NAME"));
    assert!(error("--connect tcp:127.0.0.1:5760 device restart WHEEL").contains("unknown device"));
    assert!(
        error("--connect tcp:127.0.0.1:5760 device explode IMU1")
            .contains("unknown device command")
    );
    assert!(error("--connect tcp:127.0.0.1:5760 record").contains("needs a FILE"));
    assert!(error("--connect tcp:127.0.0.1:5760 status extra").contains("unexpected argument"));
}

#[test]
fn a_bad_number() {
    for bad in ["0", "-1", "abc", "inf", "NaN", "100000"] {
        let message = error(&format!("--connect tcp:127.0.0.1:5760 devices --for {bad}"));
        assert!(
            message.contains("--for needs a number of seconds"),
            "{bad:?}: {message}"
        );
        let message = error(&format!(
            "--connect tcp:127.0.0.1:5760 --timeout {bad} status"
        ));
        assert!(message.contains("--timeout needs"), "{bad:?}: {message}");
    }
}

#[test]
fn the_yes_guard() {
    for words in ["param reset-all", "param reset-calibration", "reboot"] {
        let message = error(&format!("--connect tcp:127.0.0.1:5760 {words}"));
        assert!(
            message.contains("refused") && message.contains("--yes"),
            "{words}: {message}"
        );
    }
    assert!(error("--connect tcp:127.0.0.1:5760 param save --yes").contains("--yes only applies"));
    assert!(error("--connect tcp:127.0.0.1:5760 status --yes").contains("--yes only applies"));
}

#[test]
fn the_debug_guard() {
    let base = "--connect tcp:127.0.0.1:5760";
    for words in [
        "param set BARO_ODR_HZ 25",
        "param set BARO_OSR_P 8",
        "param set MAG_ODR_HZ 50",
        "param set IMU1_ACC_FS 4",
        "param set IMU2_ODR_HZ 400",
        "param set baro_iir_t 3",
    ] {
        let message = error(&format!("{base} {words}"));
        assert!(
            message.contains("refused") && message.contains("--debug") && !message.contains('\n'),
            "{words}: {message}"
        );
        assert!(matches!(
            command(&format!("{words} --debug")),
            Command::ParamSet(..)
        ));
    }
    for words in [
        "param set BARO_PRESET 0",
        "param set MAG_OFFX 1",
        "param set LOG_RATE_HZ 200",
        "param set BARO_RATE_HZ 20",
    ] {
        assert!(
            matches!(command(words), Command::ParamSet(..)),
            "{words} needs no --debug"
        );
    }
    assert!(error(&format!("{base} param save --debug")).contains("--debug only applies"));
    assert!(error(&format!("{base} status --debug")).contains("--debug only applies"));
}

#[test]
fn for_applies_only_to_collecting_commands() {
    for words in [
        "param list",
        "param save",
        "calibrate gyro",
        "reboot --yes",
        "device restart IMU1",
    ] {
        let message = error(&format!("--connect tcp:127.0.0.1:5760 {words} --for 2"));
        assert!(message.contains("--for only applies"), "{words}: {message}");
    }
    for words in ["status", "devices", "texts", "watch", "record /tmp/x"] {
        command(&format!("{words} --for 2"));
    }
}

#[test]
fn the_mavlink_commands_carry_the_contracts_parameters() {
    let of = |words: &str| mav_command(&command(words)).expect(words);
    let p = |index: usize, value: f32| {
        let mut a = [0.0f32; 7];
        a[index] = value;
        a
    };
    assert_eq!(of("calibrate gyro").2, p(0, 1.0));
    assert_eq!(of("calibrate accel").2, p(4, 4.0));
    assert_eq!(of("calibrate compass").2, p(1, 1.0));
    assert_eq!(of("calibrate cancel").2, [0.0; 7]);
    assert_eq!(of("param reload").2, p(0, 0.0));
    assert_eq!(of("param save").2, p(0, 1.0));
    assert_eq!(of("param reset-all --yes").2, p(0, 2.0));
    assert_eq!(of("param reset-calibration --yes").2, p(0, 3.0));
    assert_eq!(of("reboot --yes").2, p(0, 1.0));
    let restart = of("device restart BATT").2;
    assert_eq!(restart, [6.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert_eq!(
        of("device selftest MAG").2,
        [4.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    );
    assert!(mav_command(&Command::Status).is_none());
    assert_eq!(device_id("RC"), Some(7));
}

#[test]
fn the_logs_commands_parse() {
    assert_eq!(command("logs list"), Command::LogsList);
    assert_eq!(
        command("logs get 7"),
        Command::LogsGet {
            number: 7,
            output: None,
            force: false
        }
    );
    // The options may stand anywhere on the line.
    for words in [
        "logs get 12 -o /tmp/a.bin --force",
        "-o /tmp/a.bin logs get 12 --force",
        "--force logs get 12 --output /tmp/a.bin",
    ] {
        assert_eq!(
            command(words),
            Command::LogsGet {
                number: 12,
                output: Some(PathBuf::from("/tmp/a.bin")),
                force: true
            },
            "{words}"
        );
    }
    assert!(USAGE.contains("logs list") && USAGE.contains("logs get"));
}

#[test]
fn the_logs_errors() {
    let e = |words: &str| error(&format!("--connect tcp:127.0.0.1:5760 {words}"));
    assert!(e("logs").contains("list or get"));
    assert!(e("logs erase").contains("unknown logs command"));
    assert!(e("logs list 3").contains("unexpected argument"));
    assert!(e("logs get").contains("needs a log NUMBER"));
    assert!(e("logs get 0").contains("1 to 65535"));
    assert!(e("logs get -3").contains("1 to 65535"));
    assert!(e("logs get 70000").contains("1 to 65535"));
    assert!(e("logs get two").contains("1 to 65535"));
    assert!(e("logs get 3 4").contains("unexpected argument"));
    assert!(e("logs get 3 -o").contains("-o needs a value"));
    assert!(e("logs list -o /tmp/a").contains("only applies to logs get"));
    assert!(e("status --force").contains("only applies to logs get"));
}

fn session(words: &str) -> marv_cli::args::SessionCommand {
    match parse(&args(&format!("session {words}"))) {
        Ok(Parsed::Session(command)) => command,
        other => panic!("{words}: {other:?}"),
    }
}

#[test]
fn the_session_commands_need_no_connect() {
    use marv_cli::args::SessionCommand;
    assert_eq!(
        session("export /tmp/a.tlog"),
        SessionCommand::Export {
            file: PathBuf::from("/tmp/a.tlog"),
            name: None,
            out: None
        }
    );
    assert_eq!(
        session("export /tmp/a.tlog --name cal_1 --out /tmp/s"),
        SessionCommand::Export {
            file: PathBuf::from("/tmp/a.tlog"),
            name: Some("cal_1".into()),
            out: Some(PathBuf::from("/tmp/s"))
        }
    );
    assert_eq!(session("list"), SessionCommand::List { dir: None });
    assert_eq!(
        session("list /tmp/s"),
        SessionCommand::List {
            dir: Some(PathBuf::from("/tmp/s"))
        }
    );
    assert_eq!(parse(&args("session --help")), Ok(Parsed::Help));
    assert!(USAGE.contains("session export") && USAGE.contains("session list"));
}

#[test]
fn the_session_errors() {
    assert!(error("session").contains("export or list"));
    assert!(error("session erase").contains("unknown session command"));
    assert!(error("session export").contains("needs a FILE.tlog"));
    assert!(error("session export a.tlog b.tlog").contains("unexpected argument"));
    assert!(error("session export a.tlog --name").contains("--name needs a value"));
    assert!(error("session export a.tlog --bad").contains("unknown option"));
    assert!(error("session list --name x").contains("only apply to session export"));
    assert!(error("session list a b").contains("unexpected argument"));
    assert!(
        error("--connect tcp:127.0.0.1:5760 session export a.tlog").contains("without --connect")
    );
}

#[test]
fn the_log_report_parses_and_needs_its_inputs() {
    use marv_cli::args::LogCommand;
    match parse(&args(
        "log report a.BIN --mass-kg 1.5 --kf 2e-5 --poles 12 --accel-lpf-hz 20 --accel-lpf-hz 5.5 --json out.json",
    )) {
        Ok(Parsed::Log(LogCommand::Report {
            file,
            mass_kg,
            kf,
            poles,
            accel_lpf_hz,
            level_range,
            json,
        })) => {
            assert_eq!(level_range, None);
            assert_eq!(file, PathBuf::from("a.BIN"));
            assert_eq!((mass_kg, kf, poles), (1.5, 2e-5, Some(12.0)));
            assert_eq!(accel_lpf_hz, vec![20.0, 5.5]);
            assert_eq!(json, Some(PathBuf::from("out.json")));
        }
        other => panic!("{other:?}"),
    }
    assert!(matches!(
        parse(&args("log report a.BIN --kf 2e-5 --mass-kg 1")),
        Ok(Parsed::Log(LogCommand::Report {
            poles: None,
            accel_lpf_hz,
            json: None,
            ..
        })) if accel_lpf_hz.is_empty()
    ));
    assert!(error("log report a.BIN --kf 2e-5").contains("needs --mass-kg"));
    assert!(error("log report a.BIN --mass-kg 1").contains("needs --kf"));
    assert!(error("log report --mass-kg 1 --kf 1").contains("needs a FILE.BIN"));
    assert!(error("log report a.BIN b.BIN --mass-kg 1 --kf 1").contains("unexpected argument"));
    assert!(error("log report a.BIN --mass-kg x --kf 1").contains("--mass-kg needs a number"));
    assert!(error("log report a.BIN --mass-kg 1 --kf -2").contains("--kf needs a number above 0"));
    assert!(error("log report a.BIN --mass-kg 1 --kf 1 --poles").contains("--poles needs a value"));
    assert!(
        error("log report a.BIN --mass-kg 1 --kf 1 --accel-lpf-hz 0")
            .contains("--accel-lpf-hz needs a number above 0")
    );
    assert!(matches!(
        parse(&args("log report a.BIN --mass-kg 1 --kf 1 --level-range 0 2000")),
        Ok(Parsed::Log(LogCommand::Report {
            level_range: Some((lo, hi)),
            ..
        })) if lo == 0.0 && hi == 2000.0
    ));
    assert!(error("log report a.BIN --mass-kg 1 --kf 1 --level-range 5").contains("needs a value"));
    assert!(
        error("log report a.BIN --mass-kg 1 --kf 1 --level-range 1 x")
            .contains("--level-range needs two numbers")
    );
    assert!(
        error("log report a.BIN --mass-kg 1 --kf 1 --level-range 5 5")
            .contains("--level-range needs min below max")
    );
    assert!(error("log").contains("needs report"));
    assert!(error("log erase").contains("unknown log command"));
    assert!(USAGE.contains("log report"));
}

#[test]
fn the_mission_commands_parse() {
    assert_eq!(
        command("mission upload plan.waypoints"),
        Command::MissionUpload {
            file: PathBuf::from("plan.waypoints"),
            return_height: None
        }
    );
    assert_eq!(
        command("mission upload plan.waypoints --return-height 25.5"),
        Command::MissionUpload {
            file: PathBuf::from("plan.waypoints"),
            return_height: Some(25.5)
        }
    );
    assert!(
        error("--connect tcp:127.0.0.1:5760 mission upload plan.waypoints --return-height nan")
            .contains("--return-height")
    );
    assert!(
        error("--connect tcp:127.0.0.1:5760 mission upload plan.waypoints --return-height")
            .contains("needs a value")
    );
    assert!(
        error("--connect tcp:127.0.0.1:5760 mission clear --return-height 20")
            .contains("only applies to mission upload")
    );
    assert!(USAGE.contains("--return-height <m>"));
    assert_eq!(
        command("mission download"),
        Command::MissionDownload {
            file: None,
            force: false
        }
    );
    assert_eq!(
        command("mission download --force out.waypoints"),
        Command::MissionDownload {
            file: Some(PathBuf::from("out.waypoints")),
            force: true
        }
    );
    assert_eq!(command("mission clear"), Command::MissionClear);
    assert!(USAGE.contains("mission upload") && USAGE.contains("mission download"));
    assert_eq!(
        command("mission start"),
        Command::Flight(FlightCommand::Start)
    );
    assert_eq!(
        command("mission pause"),
        Command::Flight(FlightCommand::Pause)
    );
    assert_eq!(
        command("mission abort"),
        Command::Flight(FlightCommand::Abort)
    );
    assert_eq!(
        command("mission land"),
        Command::Flight(FlightCommand::Land)
    );
    assert_eq!(
        command("mission return"),
        Command::Flight(FlightCommand::Return)
    );
    assert!(USAGE.contains("mission start | pause | abort | land | return"));
}

#[test]
fn the_mission_errors() {
    let e = |words: &str| error(&format!("--connect tcp:127.0.0.1:5760 {words}"));
    assert!(e("mission").contains("upload, download or clear"));
    assert!(e("mission fly").contains("unknown mission command"));
    assert!(e("mission upload").contains("needs a FILE.waypoints"));
    assert!(e("mission upload a b").contains("unexpected argument"));
    assert!(e("mission download a b").contains("unexpected argument"));
    assert!(e("mission clear now").contains("unexpected argument"));
    assert!(e("mission start now").contains("unexpected argument"));
    assert!(e("mission start --force").contains("only applies to"));
    assert!(
        e("mission upload a --force").contains("only applies to logs get and mission download")
    );
    assert!(e("mission download -o a").contains("only applies to logs get"));
    assert!(e("mission clear --for 3").contains("only applies to"));
}
