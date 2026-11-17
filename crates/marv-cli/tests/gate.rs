//! The negative control (the design's: "a `udp:` endpoint: configure commands refused, 0 frames"):
//! on an Operate session every command that writes is refused by `Session::send` and a counting
//! writer sees no `PARAM_SET` and no `COMMAND_LONG` other than the session's own capability request;
//! the reads work on the same session.

mod common;

use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use gcs_mavlink::dialect::MavMessage;
use gcs_mavlink::{Access, Body, Deframer, Session, SessionConfig};

use common::{Rig, Shared, TempDir};

/// Passes every write on and keeps a copy of the bytes.
struct Tee {
    inner: Box<dyn Write + Send>,
    seen: Arc<Mutex<Vec<u8>>>,
}

impl Write for Tee {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let n = self.inner.write(buf)?;
        self.seen
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .extend_from_slice(&buf[..n]);
        Ok(n)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

/// A session on the rig's vehicle with the given role, and the bytes it wrote.
fn session(rig: &Rig, access: Access) -> (Session, Arc<Mutex<Vec<u8>>>) {
    let endpoint = format!("tcp:{}", rig.addr).parse().expect("endpoint");
    let link = gcs_transport::open(&endpoint).expect("open");
    let seen = Arc::new(Mutex::new(Vec::new()));
    let writer = Tee {
        inner: link.writer,
        seen: Arc::clone(&seen),
    };
    let session = Session::start(
        link.reader,
        Box::new(writer),
        SessionConfig {
            access,
            ..SessionConfig::default()
        },
        Box::new(|| {}),
    );
    (session, seen)
}

/// The frames written, as names: `COMMAND_LONG(512)` for a command, `PARAM_SET`, ...
fn frames_written(seen: &Mutex<Vec<u8>>) -> Vec<String> {
    let bytes = seen.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let mut frames = Vec::new();
    Deframer::new().push(&bytes, &mut frames);
    frames
        .iter()
        .filter_map(|f| match &f.body {
            Body::Known(MavMessage::COMMAND_LONG(c)) => {
                Some(format!("COMMAND_LONG({})", c.command as u32))
            }
            Body::Known(MavMessage::PARAM_SET(_)) => Some("PARAM_SET".to_string()),
            Body::Known(MavMessage::PARAM_REQUEST_LIST(_)) => {
                Some("PARAM_REQUEST_LIST".to_string())
            }
            Body::Known(MavMessage::HEARTBEAT(_)) => Some("HEARTBEAT".to_string()),
            Body::Known(other) => Some(
                format!("{other:?}")
                    .split('(')
                    .next()
                    .unwrap_or("")
                    .to_string(),
            ),
            _ => None,
        })
        .collect()
}

fn invoke(words: &str, session: &Session, access: Access) -> (i32, String, String) {
    let args: Vec<String> = format!("--connect tcp:127.0.0.1:5760 --timeout 5 {words}")
        .split_whitespace()
        .map(str::to_string)
        .collect();
    let Ok(marv_cli::args::Parsed::Run(invocation)) = marv_cli::args::parse(&args) else {
        panic!("{words}: does not parse");
    };
    let (mut out, mut err) = (Shared::default(), Shared::default());
    let code = marv_cli::commands::execute(&invocation, session, access, &mut out, &mut err);
    (code, out.text(), err.text())
}

#[test]
fn on_an_operate_session_every_write_is_refused_with_zero_frames() {
    let rig = Rig::start();
    let (session, seen) = session(&rig, Access::Operate);
    for words in [
        "param set LOG_RATE_HZ 200",
        "param save",
        "calibrate gyro",
        "device restart BARO",
        "reboot --yes",
    ] {
        let (code, out, err) = invoke(words, &session, Access::Operate);
        assert_eq!(code, marv_cli::exit::REFUSED, "{words}: {out:?} {err:?}");
        assert!(
            err.contains("refused: this link is operate-only"),
            "{words}: {err:?}"
        );
        assert!(!out.contains("accepted"), "{words}: {out:?}");
    }
    assert_eq!(
        session.state().link.refused.as_ref().map(|r| r.count),
        Some(5)
    );

    // The reads work on the same session.
    let (code, out, _) = invoke("devices --for 1.5", &session, Access::Operate);
    assert_eq!((code, out.lines().count()), (0, 8), "{out:?}");
    let (code, out, _) = invoke("status --for 0.2", &session, Access::Operate);
    assert!(out.contains("role\tOperate") && code == 0, "{out:?}");
    let (code, out, _) = invoke("param list", &session, Access::Operate);
    assert_eq!((code, out.lines().count()), (0, 74), "{out:?}");

    std::thread::sleep(Duration::from_millis(100));
    let frames = frames_written(&seen);
    eprintln!("frames written on the operate session: {frames:?}");
    let count = |name: &str| frames.iter().filter(|f| f.as_str() == name).count();
    assert_eq!(count("PARAM_SET"), 0, "{frames:?}");
    let commands: Vec<&String> = frames
        .iter()
        .filter(|f| f.starts_with("COMMAND_LONG"))
        .collect();
    assert!(
        commands.iter().all(|c| c.as_str() == "COMMAND_LONG(512)"),
        "only the session's own capability request may be a COMMAND_LONG: {commands:?}"
    );
    assert!(
        count("PARAM_REQUEST_LIST") >= 1,
        "the read the CLI asked for: {frames:?}"
    );
}

#[test]
fn positive_control_on_a_configure_session_the_same_commands_write() {
    let rig = Rig::start();
    let (session, seen) = session(&rig, Access::Configure);
    let (code, out, err) = invoke("param set LOG_RATE_HZ 200", &session, Access::Configure);
    assert_eq!(code, 0, "{out:?} {err:?}");
    let (code, out, err) = invoke("device restart BARO", &session, Access::Configure);
    assert_eq!(code, 0, "{out:?} {err:?}");
    std::thread::sleep(Duration::from_millis(100));
    let frames = frames_written(&seen);
    assert!(frames.iter().any(|f| f == "PARAM_SET"), "{frames:?}");
    assert!(
        frames.iter().any(|f| f == "COMMAND_LONG(42430)"),
        "{frames:?}"
    );
}

#[test]
fn on_an_operate_session_both_log_commands_are_refused_with_nothing_sent() {
    let rig = Rig::start();
    let dir = TempDir::new("gate-logs");
    let file = dir.file("out.bin");
    let (session, seen) = session(&rig, Access::Operate);
    for words in [
        "logs list".to_string(),
        format!("logs get 2 -o {}", file.display()),
    ] {
        let (code, out, err) = invoke(&words, &session, Access::Operate);
        assert_eq!(code, marv_cli::exit::REFUSED, "{words}: {out:?} {err:?}");
        assert!(
            err.contains("refused: this link is operate-only"),
            "{words}: {err:?}"
        );
        assert_eq!(out, "", "{words}");
    }
    assert!(!file.exists(), "the file is not even created");
    assert_eq!(
        session.state().link.refused.as_ref().map(|r| r.count),
        Some(2)
    );
    // The reads still work on the same session.
    let (code, _, _) = invoke("status --for 0.2", &session, Access::Operate);
    assert_eq!(code, 0);
    std::thread::sleep(Duration::from_millis(100));
    let frames = frames_written(&seen);
    assert!(
        !frames.iter().any(|f| f.starts_with("LOG_")),
        "no log message on the link: {frames:?}"
    );
}

#[test]
fn positive_control_on_a_configure_session_the_log_commands_send() {
    let rig = Rig::start();
    let dir = TempDir::new("gate-logs-configure");
    let file = dir.file("out.bin");
    let (session, seen) = session(&rig, Access::Configure);
    let (code, out, err) = invoke("logs list", &session, Access::Configure);
    assert_eq!(code, 0, "{out:?} {err:?}");
    let (code, out, err) = invoke(
        &format!("logs get 11 -o {}", file.display()),
        &session,
        Access::Configure,
    );
    assert_eq!(code, 0, "{out:?} {err:?}");
    std::thread::sleep(Duration::from_millis(100));
    let frames = frames_written(&seen);
    for name in ["LOG_REQUEST_LIST", "LOG_REQUEST_DATA"] {
        assert!(frames.iter().any(|f| f == name), "{name}: {frames:?}");
    }
}
