//! The command line, parsed by hand: `marv-cli --connect <endpoint> <command> [args]`.

use std::path::PathBuf;
use std::time::Duration;

use gcs_mavlink::dialect::MavCmd;
use gcs_transport::Endpoint;

pub const USAGE: &str = "\
marv-cli: the MARV ground station on the command line

Usage: marv-cli --connect <endpoint> [--timeout <s>] <command> [args]

Endpoint: serial:PATH[:BAUD], tcp:HOST:PORT or udp:ADDR:PORT. serial and tcp are
configure links; udp is operate-only (every command that writes is refused).

Commands:
  status                           one summary of the link and the vehicle
  devices [--for <s>]              the device rows (default 2 s of collecting)
  texts [--for <s>]                STATUSTEXTs as they arrive (default 10 s)
  watch [--for <s>]                device changes and texts (default 30 s)
  param list | get <NAME> | set <NAME> <VALUE>
                                   a *_PRESET selector to 0 and a sensor field a preset governs need --debug
  param save | reload              store the parameters / read them back from flash
  param reset-all --yes            all parameters to their defaults
  param reset-calibration --yes    the calibration parameters to their defaults
  calibrate gyro | accel | compass | cancel
  device restart <NAME>            NAME: IMU1 IMU2 BARO MAG GNSS BATT (RC SD PSRAM LED ESC are sent and the vehicle decides)
  device selftest <NAME>
  reboot --yes
  record <FILE> [--for <s>]        record the link to a .tlog (default 60 s)
  logs list                        the logs on the vehicle's card: NUMBER<TAB>SIZE, then last<TAB>N (USB or TCP only)
  logs get <N> [-o FILE] [--force] download log N (default file NNNNNNNN.BIN here; an existing file is kept without --force)

Options:
  --connect <endpoint>   where the vehicle is (required)
  --timeout <s>          wait this long for the vehicle's heartbeat (default 5)
  --for <s>              how long a collecting command runs
  --yes                  confirm reset-all, reset-calibration and reboot
  --debug                allow param set of a *_PRESET selector to 0 or of a field a preset governs
  -o, --output <file>    where logs get writes the log
  --force                let logs get overwrite an existing file
  --help                 this text

Exit status: 0 success; 1 the command failed or the vehicle did not accept it;
2 usage error; 3 no link or no heartbeat; 4 refused (the link is operate-only).
";

pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);
pub const DEFAULT_DEVICES: Duration = Duration::from_secs(2);
pub const DEFAULT_TEXTS: Duration = Duration::from_secs(10);
pub const DEFAULT_WATCH: Duration = Duration::from_secs(30);
pub const DEFAULT_RECORD: Duration = Duration::from_secs(60);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Calibration {
    Gyro,
    Accel,
    Compass,
    Cancel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Restart,
    SelfTest,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Storage {
    Reload,
    Save,
    ResetAll,
    ResetCalibration,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Status,
    Devices,
    Texts,
    Watch,
    ParamList,
    ParamGet(String),
    ParamSet(String, f64),
    Param(Storage),
    Calibrate(Calibration),
    Device {
        action: Action,
        id: u8,
        name: String,
    },
    Reboot,
    Record(PathBuf),
    LogsList,
    LogsGet {
        number: u16,
        /// `None`: `NNNNNNNN.BIN` in the current directory.
        output: Option<PathBuf>,
        force: bool,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Invocation {
    pub endpoint: Endpoint,
    pub timeout: Duration,
    /// How long a collecting command runs; the command's own default when `None`.
    pub duration: Option<Duration>,
    pub command: Command,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Parsed {
    Help,
    Run(Invocation),
}

/// The `MARV_DEVICE` ids by name (docs/link-contract.md, section 8).
pub fn device_id(name: &str) -> Option<u8> {
    Some(match name.to_ascii_uppercase().as_str() {
        "IMU1" => 1,
        "IMU2" => 2,
        "BARO" => 3,
        "MAG" => 4,
        "GNSS" => 5,
        "BATT" => 6,
        "RC" => 7,
        "SD" => 9,
        "PSRAM" => 10,
        "LED" => 11,
        "ESC" => 12,
        _ => return None,
    })
}

fn seconds(option: &str, text: &str) -> Result<Duration, String> {
    match text.parse::<f64>() {
        Ok(s) if s.is_finite() && s > 0.0 && s <= 86_400.0 => Ok(Duration::from_secs_f64(s)),
        _ => Err(format!(
            "{option} needs a number of seconds above 0 (and at most 86400), not {text:?}"
        )),
    }
}

/// Parses the arguments after the program name. The error text is the whole message.
pub fn parse(args: &[String]) -> Result<Parsed, String> {
    let mut connect = None;
    let mut timeout = DEFAULT_TIMEOUT;
    let mut duration = None;
    let mut yes = false;
    let mut debug = false;
    let mut output = None;
    let mut force = false;
    let mut words: Vec<&str> = Vec::new();
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut value = |option: &str| {
            iter.next()
                .map(String::as_str)
                .ok_or_else(|| format!("{option} needs a value"))
        };
        match arg.as_str() {
            "--help" | "-h" => return Ok(Parsed::Help),
            "--connect" => {
                let text = value("--connect")?;
                connect = Some(text.parse::<Endpoint>().map_err(|e| e.to_string())?);
            }
            "--timeout" => timeout = seconds("--timeout", value("--timeout")?)?,
            "--for" => duration = Some(seconds("--for", value("--for")?)?),
            "--yes" => yes = true,
            "--debug" => debug = true,
            "-o" | "--output" => output = Some(PathBuf::from(value(arg)?)),
            "--force" => force = true,
            option if option.starts_with("--") => return Err(format!("unknown option {option}")),
            word => words.push(word),
        }
    }
    let Some(endpoint) = connect else {
        return Err("missing --connect <endpoint>".to_string());
    };
    let (mut command, takes_for) = command(&words, yes)?;
    match &mut command {
        Command::LogsGet {
            output: file,
            force: forced,
            ..
        } => {
            *file = output;
            *forced = force;
        }
        _ if output.is_some() => return Err("-o only applies to logs get".to_string()),
        _ if force => return Err("--force only applies to logs get".to_string()),
        _ => {}
    }
    match &command {
        Command::ParamSet(name, number) => {
            if !debug && let Some(why) = needs_debug(name, *number) {
                return Err(format!(
                    "param set {name} refused: {why}; it needs --debug (the station's gate for raw sensor settings)"
                ));
            }
        }
        _ if debug => return Err("--debug only applies to param set".to_string()),
        _ => {}
    }
    if yes
        && !matches!(
            &command,
            Command::Param(Storage::ResetAll | Storage::ResetCalibration) | Command::Reboot
        )
    {
        return Err(
            "--yes only applies to param reset-all, param reset-calibration and reboot".to_string(),
        );
    }
    if duration.is_some() && !takes_for {
        return Err("--for only applies to status, devices, texts, watch and record".to_string());
    }
    Ok(Parsed::Run(Invocation {
        endpoint,
        timeout,
        duration,
        command,
    }))
}

/// Why a write of `number` to the parameter `name` is a raw sensor setting, if it is: the selector of
/// a sensor preset set to 0 (Custom), or a field a preset governs (the table is the crate's).
fn needs_debug(name: &str, number: f64) -> Option<&'static str> {
    marv_presets::SENSORS.iter().find_map(|sensor| {
        if sensor.selector.eq_ignore_ascii_case(name) && number == 0.0 {
            Some("Custom (0) leaves the sensor's preset")
        } else if sensor.fields.iter().any(|f| f.eq_ignore_ascii_case(name)) {
            Some("a field governed by a sensor preset")
        } else {
            None
        }
    })
}

/// The command and whether it takes `--for`.
fn command(words: &[&str], yes: bool) -> Result<(Command, bool), String> {
    let none = |command: Command, rest: &[&str]| -> Result<(Command, bool), String> {
        if rest.is_empty() {
            Ok((command, false))
        } else {
            Err(format!("unexpected argument {:?}", rest[0]))
        }
    };
    let Some((&name, rest)) = words.split_first() else {
        return Err("missing command".to_string());
    };
    match name {
        "status" | "devices" | "texts" | "watch" => {
            let (command, _) = none(
                match name {
                    "status" => Command::Status,
                    "devices" => Command::Devices,
                    "texts" => Command::Texts,
                    _ => Command::Watch,
                },
                rest,
            )?;
            Ok((command, true))
        }
        "param" => {
            let Some((&sub, rest)) = rest.split_first() else {
                return Err(
                    "param needs list, get, set, save, reload, reset-all or reset-calibration"
                        .to_string(),
                );
            };
            match (sub, rest) {
                ("list", _) => none(Command::ParamList, rest),
                ("get", [name]) => Ok((Command::ParamGet((*name).to_string()), false)),
                ("get", _) => Err("param get needs exactly one NAME".to_string()),
                ("set", [name, value]) => match value.parse::<f64>() {
                    Ok(number) if number.is_finite() => {
                        Ok((Command::ParamSet((*name).to_string(), number), false))
                    }
                    _ => Err(format!("param set needs a number as VALUE, not {value:?}")),
                },
                ("set", _) => Err("param set needs a NAME and a VALUE".to_string()),
                ("save", _) => none(Command::Param(Storage::Save), rest),
                ("reload", _) => none(Command::Param(Storage::Reload), rest),
                ("reset-all", _) | ("reset-calibration", _) => {
                    if !yes {
                        return Err(format!(
                            "param {sub} refused: it needs --yes (it discards the stored values)"
                        ));
                    }
                    let storage = if sub == "reset-all" {
                        Storage::ResetAll
                    } else {
                        Storage::ResetCalibration
                    };
                    none(Command::Param(storage), rest)
                }
                (other, _) => Err(format!("unknown param command {other:?}")),
            }
        }
        "calibrate" => match rest {
            ["gyro"] => Ok((Command::Calibrate(Calibration::Gyro), false)),
            ["accel"] => Ok((Command::Calibrate(Calibration::Accel), false)),
            ["compass"] => Ok((Command::Calibrate(Calibration::Compass), false)),
            ["cancel"] => Ok((Command::Calibrate(Calibration::Cancel), false)),
            [] => Err("calibrate needs gyro, accel, compass or cancel".to_string()),
            [other, ..] => Err(format!(
                "unknown calibration {other:?}: gyro, accel, compass or cancel"
            )),
        },
        "device" => {
            let action = match rest.first() {
                Some(&"restart") => Action::Restart,
                Some(&"selftest") => Action::SelfTest,
                Some(other) => {
                    return Err(format!(
                        "unknown device command {other:?}: restart or selftest"
                    ));
                }
                None => return Err("device needs restart or selftest".to_string()),
            };
            match &rest[1..] {
                [device] => {
                    let id = device_id(device).ok_or_else(|| {
                        format!("unknown device {device:?}: IMU1 IMU2 BARO MAG GNSS BATT RC SD PSRAM LED ESC")
                    })?;
                    Ok((
                        Command::Device {
                            action,
                            id,
                            name: device.to_ascii_uppercase(),
                        },
                        false,
                    ))
                }
                [] => Err("device needs a NAME".to_string()),
                [_, extra, ..] => Err(format!("unexpected argument {extra:?}")),
            }
        }
        "reboot" => {
            if !yes {
                return Err("reboot refused: it needs --yes (the vehicle restarts)".to_string());
            }
            none(Command::Reboot, rest)
        }
        "record" => match rest {
            [file] => Ok((Command::Record(PathBuf::from(file)), true)),
            [] => Err("record needs a FILE".to_string()),
            [_, extra, ..] => Err(format!("unexpected argument {extra:?}")),
        },
        "logs" => match rest {
            ["list"] => Ok((Command::LogsList, false)),
            ["list", extra, ..] => Err(format!("unexpected argument {extra:?}")),
            ["get", number] => match number.parse::<u16>() {
                Ok(number) if number > 0 => Ok((
                    Command::LogsGet {
                        number,
                        output: None,
                        force: false,
                    },
                    false,
                )),
                _ => Err(format!(
                    "logs get needs a log number from 1 to 65535, not {number:?}"
                )),
            },
            ["get"] => Err("logs get needs a log NUMBER".to_string()),
            ["get", _, extra, ..] => Err(format!("unexpected argument {extra:?}")),
            [] => Err("logs needs list or get".to_string()),
            [other, ..] => Err(format!("unknown logs command {other:?}: list or get")),
        },
        other => Err(format!("unknown command {other:?}")),
    }
}

/// What `Command::SendCommand` carries for the commands that send one: the words for the
/// progress lines, the `MAV_CMD` and its seven parameters (docs/link-contract.md, section 10).
pub fn mav_command(command: &Command) -> Option<(String, MavCmd, [f32; 7])> {
    let zero = [0.0f32; 7];
    Some(match command {
        Command::Calibrate(Calibration::Gyro) => (
            "gyro calibration".into(),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        Command::Calibrate(Calibration::Accel) => (
            "accelerometer calibration".into(),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0],
        ),
        Command::Calibrate(Calibration::Compass) => (
            "compass calibration".into(),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        Command::Calibrate(Calibration::Cancel) => (
            "cancel calibration".into(),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            zero,
        ),
        Command::Param(storage) => {
            let (label, p1) = match storage {
                Storage::Reload => ("reload parameters", 0.0),
                Storage::Save => ("save parameters", 1.0),
                Storage::ResetAll => ("reset all parameters", 2.0),
                Storage::ResetCalibration => ("reset calibration parameters", 3.0),
            };
            (
                label.into(),
                MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
                [p1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        }
        Command::Device { action, id, name } => {
            let (word, p2) = match action {
                Action::Restart => ("restart", 1.0),
                Action::SelfTest => ("self-test", 2.0),
            };
            (
                format!("{word} {name}"),
                MavCmd::MAV_CMD_MARV_DEVICE,
                [f32::from(*id), p2, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        }
        Command::Reboot => (
            "reboot".into(),
            MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        _ => return None,
    })
}
