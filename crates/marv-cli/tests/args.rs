//! The command line: every command and option, and every error.

use std::path::PathBuf;
use std::time::Duration;

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
