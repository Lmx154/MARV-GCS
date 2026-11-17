//! Command line: hand-written options.

use std::path::PathBuf;

use gcs_mavlink::tlog::ReplaySpeed;
use gcs_transport::Endpoint;

use crate::views;

pub const USAGE: &str = "usage: marv-gcs [--connect <endpoint>] [--log-dir <dir>] [--no-log]
                [--view <name>] [--review <file.BIN>] [--screenshot <file.png>]
                [--after <seconds>]

  --connect <endpoint>      connect at start: tcp:HOST:PORT, serial:PATH[:BAUD] or
                            udp:ADDR:PORT (the local address to listen on), or replay a log:
                            tlog:PATH (real time) or tlogfast:PATH (as fast as read)
  --log-dir <dir>           directory for the automatic .tlog recordings (default: logs)
  --no-log                  do not record sessions automatically
  --view <name>             tab shown at start: a tab title, in any case
  --review <file.BIN>       open a DataFlash log in the Log tab's review section
                            and start on that tab (unless --view says otherwise)
  --screenshot <file.png>   save a picture of the window and exit
  --after <seconds>         seconds from the start to the picture (default 3)
  --help                    print this text";

/// Directory for the automatic recordings when `--log-dir` is not given.
pub const DEFAULT_LOG_DIR: &str = "logs";

/// Seconds from the start to the screenshot when `--after` is not given.
pub const DEFAULT_AFTER_SECS: f64 = 3.0;

#[derive(Debug, Clone, PartialEq)]
pub struct Cli {
    pub connect: Option<Endpoint>,
    /// A log to replay at start (`--connect tlog:` / `tlogfast:`); excludes `connect`.
    pub replay: Option<(PathBuf, ReplaySpeed)>,
    pub log_dir: PathBuf,
    pub no_log: bool,
    /// Position in the view registry of the tab shown at start (`--view`).
    pub view: Option<usize>,
    /// A DataFlash `.BIN` log to open in the Log tab's review section (`--review`).
    pub review: Option<PathBuf>,
    pub screenshot: Option<PathBuf>,
    pub after_secs: f64,
}

impl Default for Cli {
    fn default() -> Self {
        Cli {
            connect: None,
            replay: None,
            log_dir: PathBuf::from(DEFAULT_LOG_DIR),
            no_log: false,
            view: None,
            review: None,
            screenshot: None,
            after_secs: DEFAULT_AFTER_SECS,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Parsed {
    Run(Cli),
    Help,
}

/// Parse the arguments after the program name. The error text is one line, without the usage.
pub fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Parsed, String> {
    let mut args = args.into_iter();
    let mut cli = Cli::default();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--help" | "-h" => return Ok(Parsed::Help),
            "--connect" => {
                let value = value(&mut args, &arg)?;
                if let Some(replay) = parse_replay(&value)? {
                    cli.replay = Some(replay);
                    cli.connect = None;
                } else {
                    cli.connect = Some(value.parse::<Endpoint>().map_err(|e| e.to_string())?);
                    cli.replay = None;
                }
            }
            "--log-dir" => cli.log_dir = PathBuf::from(value(&mut args, &arg)?),
            "--no-log" => cli.no_log = true,
            "--view" => cli.view = Some(parse_view(&value(&mut args, &arg)?)?),
            "--review" => cli.review = Some(PathBuf::from(value(&mut args, &arg)?)),
            "--screenshot" => cli.screenshot = Some(PathBuf::from(value(&mut args, &arg)?)),
            "--after" => {
                let value = value(&mut args, &arg)?;
                cli.after_secs = match value.parse::<f64>() {
                    Ok(s) if s.is_finite() && s >= 0.0 => s,
                    _ => return Err(format!("invalid --after {value:?}: expected seconds >= 0")),
                };
            }
            _ => return Err(format!("unknown argument {arg:?}")),
        }
    }
    if cli.review.is_some() && cli.view.is_none() {
        cli.view = Some(parse_view("log")?);
    }
    Ok(Parsed::Run(cli))
}

/// The position of the view whose tab title is `name`, in any case.
fn parse_view(name: &str) -> Result<usize, String> {
    let titles: Vec<&'static str> = views::all().iter().map(|view| view.name()).collect();
    titles
        .iter()
        .position(|title| title.eq_ignore_ascii_case(name))
        .ok_or_else(|| {
            let valid: Vec<String> = titles.iter().map(|title| title.to_lowercase()).collect();
            format!(
                "unknown view {name:?}: valid names are {}",
                valid.join(", ")
            )
        })
}

/// `tlog:PATH` and `tlogfast:PATH`; `None` for anything else.
fn parse_replay(value: &str) -> Result<Option<(PathBuf, ReplaySpeed)>, String> {
    let (speed, path) = if let Some(path) = value.strip_prefix("tlog:") {
        (ReplaySpeed::RealTime, path)
    } else if let Some(path) = value.strip_prefix("tlogfast:") {
        (ReplaySpeed::Fast, path)
    } else {
        return Ok(None);
    };
    if path.is_empty() {
        return Err(format!(
            "invalid --connect {value:?}: missing log file path"
        ));
    }
    Ok(Some((PathBuf::from(path), speed)))
}

fn value(args: &mut impl Iterator<Item = String>, option: &str) -> Result<String, String> {
    args.next().ok_or_else(|| format!("{option} needs a value"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_strs(args: &[&str]) -> Result<Parsed, String> {
        parse(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn no_arguments_gives_defaults() {
        assert_eq!(parse_strs(&[]), Ok(Parsed::Run(Cli::default())));
        assert_eq!(Cli::default().after_secs, 3.0);
    }

    #[test]
    fn all_options() {
        let parsed = parse_strs(&[
            "--connect",
            "tcp:127.0.0.1:5760",
            "--screenshot",
            "/tmp/a.png",
            "--after",
            "1.5",
        ]);
        assert_eq!(
            parsed,
            Ok(Parsed::Run(Cli {
                connect: Some(Endpoint::Tcp {
                    addr: "127.0.0.1:5760".into()
                }),
                screenshot: Some(PathBuf::from("/tmp/a.png")),
                after_secs: 1.5,
                ..Cli::default()
            }))
        );
    }

    #[test]
    fn serial_endpoint() {
        let parsed = parse_strs(&["--connect", "serial:/dev/ttyACM0:57600"]);
        let Ok(Parsed::Run(cli)) = parsed else {
            panic!("{parsed:?}")
        };
        assert_eq!(
            cli.connect,
            Some(Endpoint::Serial {
                path: "/dev/ttyACM0".into(),
                baud: 57600
            })
        );
    }

    #[test]
    fn log_options() {
        assert_eq!(Cli::default().log_dir, PathBuf::from("logs"));
        assert!(!Cli::default().no_log);
        let parsed = parse_strs(&["--log-dir", "/tmp/my logs", "--no-log"]);
        assert_eq!(
            parsed,
            Ok(Parsed::Run(Cli {
                log_dir: PathBuf::from("/tmp/my logs"),
                no_log: true,
                ..Cli::default()
            }))
        );
    }

    #[test]
    fn tlog_replays_in_real_time() {
        let parsed = parse_strs(&["--connect", "tlog:/tmp/a b.tlog"]);
        assert_eq!(
            parsed,
            Ok(Parsed::Run(Cli {
                replay: Some((PathBuf::from("/tmp/a b.tlog"), ReplaySpeed::RealTime)),
                ..Cli::default()
            }))
        );
    }

    #[test]
    fn tlogfast_replays_fast() {
        let parsed = parse_strs(&["--connect", "tlogfast:run.tlog"]);
        assert_eq!(
            parsed,
            Ok(Parsed::Run(Cli {
                replay: Some((PathBuf::from("run.tlog"), ReplaySpeed::Fast)),
                ..Cli::default()
            }))
        );
    }

    #[test]
    fn tlog_without_a_path_is_an_error() {
        assert!(parse_strs(&["--connect", "tlog:"]).is_err());
        assert!(parse_strs(&["--connect", "tlogfast:"]).is_err());
    }

    #[test]
    fn the_last_connect_wins() {
        let parsed = parse_strs(&["--connect", "tcp:h:1", "--connect", "tlog:a.tlog"]);
        let Ok(Parsed::Run(cli)) = parsed else {
            panic!("{parsed:?}")
        };
        assert_eq!(cli.connect, None);
        assert!(cli.replay.is_some());
    }

    #[test]
    fn help() {
        assert_eq!(parse_strs(&["--help"]), Ok(Parsed::Help));
        assert_eq!(parse_strs(&["-h"]), Ok(Parsed::Help));
    }

    #[test]
    fn unknown_argument_is_an_error() {
        let e = parse_strs(&["--bogus"]).unwrap_err();
        assert!(e.contains("--bogus"), "{e}");
        assert!(parse_strs(&["positional"]).is_err());
    }

    #[test]
    fn bad_endpoint_is_an_error() {
        assert!(parse_strs(&["--connect", "tcp:"]).is_err());
        assert!(parse_strs(&["--connect", "udp:"]).is_err());
        assert!(parse_strs(&["--connect", "udp:0.0.0.0"]).is_err());
        assert!(parse_strs(&["--connect", "ftp:1.2.3.4:5"]).is_err());
    }

    #[test]
    fn udp_endpoint() {
        let parsed = parse_strs(&["--connect", "udp:0.0.0.0:14550"]);
        let Ok(Parsed::Run(cli)) = parsed else {
            panic!("{parsed:?}")
        };
        assert_eq!(
            cli.connect,
            Some(Endpoint::Udp {
                bind: "0.0.0.0:14550".parse().unwrap()
            })
        );
    }

    #[test]
    fn missing_value_is_an_error() {
        for option in [
            "--connect",
            "--log-dir",
            "--view",
            "--review",
            "--screenshot",
            "--after",
        ] {
            let e = parse_strs(&[option]).unwrap_err();
            assert!(e.contains(option), "{e}");
        }
    }

    #[test]
    fn bad_after_is_an_error() {
        for bad in ["abc", "-1", "NaN", "inf", ""] {
            assert!(parse_strs(&["--after", bad]).is_err(), "{bad:?}");
        }
        assert_eq!(
            parse_strs(&["--after", "0"]),
            Ok(Parsed::Run(Cli {
                after_secs: 0.0,
                ..Cli::default()
            }))
        );
    }

    #[test]
    fn view_names_match_the_tab_titles_in_any_case() {
        let titles: Vec<&str> = views::all().iter().map(|view| view.name()).collect();
        assert_eq!(Cli::default().view, None);
        for (index, title) in titles.iter().enumerate() {
            for spelled in [
                title.to_lowercase(),
                title.to_uppercase(),
                title.to_string(),
            ] {
                assert_eq!(
                    parse_strs(&["--view", &spelled]),
                    Ok(Parsed::Run(Cli {
                        view: Some(index),
                        ..Cli::default()
                    })),
                    "{spelled}"
                );
            }
        }
    }

    #[test]
    fn unknown_view_lists_the_valid_names() {
        for bad in ["nonsense", "", "sensor", "sensors "] {
            let e = parse_strs(&["--view", bad]).unwrap_err();
            assert!(e.contains(&format!("{bad:?}")), "{e}");
            for title in views::all().iter().map(|view| view.name()) {
                assert!(e.contains(&title.to_lowercase()), "{e}");
            }
        }
    }

    #[test]
    fn review_opens_a_file_and_starts_on_the_log_tab() {
        let log = views::all()
            .iter()
            .position(|view| view.name() == "Log")
            .unwrap();
        assert_eq!(Cli::default().review, None);
        assert_eq!(
            parse_strs(&["--review", "/tmp/a b/00000007.BIN"]),
            Ok(Parsed::Run(Cli {
                review: Some(PathBuf::from("/tmp/a b/00000007.BIN")),
                view: Some(log),
                ..Cli::default()
            }))
        );
        for order in [
            ["--view", "sensors", "--review", "x.BIN"],
            ["--review", "x.BIN", "--view", "sensors"],
        ] {
            let Ok(Parsed::Run(cli)) = parse_strs(&order) else {
                panic!("{order:?}")
            };
            assert_eq!(cli.review, Some(PathBuf::from("x.BIN")));
            assert_ne!(cli.view, Some(log), "an explicit --view wins: {order:?}");
            assert!(cli.view.is_some());
        }
        assert!(USAGE.contains("--review <file.BIN>"));
    }

    #[test]
    fn usage_mentions_the_view_option() {
        assert!(USAGE.contains("--view <name>"));
    }
}
