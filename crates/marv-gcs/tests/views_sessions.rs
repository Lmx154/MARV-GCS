//! The window's monitoring sessions: the Sensors tab's bar and the Log tab's list, driven through the whole
//! `App` (a simulated vehicle over TCP, a temporary log directory), drawn by egui_kittest.

use std::fs;
use std::io::Write;
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_link::session_folder::{SessionOptions, export_session};
use gcs_mavlink::dialect::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType};
use gcs_mavlink::{Encoder, Message};
use gcs_sim::{SimConfig, SimVehicle, serve_tcp};
use marv_gcs::app::App;
use marv_gcs::cli::{self, Cli, Parsed};
use marv_gcs::screenshot::Outcome;
use marv_gcs::sessions::{self, Exporter};

const WAIT: Duration = Duration::from_secs(15);

struct Sim {
    port: u16,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Sim {
    fn start() -> Sim {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().unwrap().port();
        let vehicle = Arc::new(Mutex::new(SimVehicle::new(SimConfig::default())));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let stop = Arc::clone(&stop);
            thread::spawn(move || serve_tcp(listener, vehicle, stop))
        };
        Sim {
            port,
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for Sim {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "marv-gcs-views-sessions-{name}-{}",
        std::process::id()
    ));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn cli(args: &[&str]) -> Cli {
    match cli::parse(args.iter().map(|a| (*a).to_owned())).expect("arguments parse") {
        Parsed::Run(cli) => cli,
        Parsed::Help => panic!("help"),
    }
}

/// The app is made in the first frame, from the harness's own context (the views reach each other through it).
struct Rig {
    app: Option<App>,
    cli: Option<Cli>,
    exporter: Option<Exporter>,
}

fn harness(cli: Cli, exporter: Option<Exporter>) -> Harness<'static, Rig> {
    let rig = Rig {
        app: None,
        cli: Some(cli),
        exporter,
    };
    Harness::builder()
        .with_size(eframe::egui::vec2(1100.0, 700.0))
        // The connection bar's TCP address field has no accessible name (not this file's to fix).
        .with_accessibility_check(false)
        .build_ui_state(
            |ui, rig: &mut Rig| {
                if rig.app.is_none() {
                    let outcome: Outcome = Arc::new(Mutex::new(None));
                    let mut app = App::with_context(ui.ctx(), rig.cli.take().unwrap(), outcome);
                    if let Some(exporter) = rig.exporter.take() {
                        app.set_exporter(exporter);
                    }
                    rig.app = Some(app);
                }
                rig.app.as_mut().unwrap().frame(ui);
            },
            rig,
        )
}

/// Frames, a little apart, until `done` holds.
fn until(h: &mut Harness<'_, Rig>, what: &str, done: impl Fn(&Harness<'_, Rig>) -> bool) {
    let start = Instant::now();
    loop {
        h.step();
        if done(h) {
            return;
        }
        assert!(start.elapsed() < WAIT, "timed out waiting for: {what}");
        thread::sleep(Duration::from_millis(20));
    }
}

fn frames_for(h: &mut Harness<'_, Rig>, span: Duration) {
    let start = Instant::now();
    while start.elapsed() < span {
        h.step();
        thread::sleep(Duration::from_millis(20));
    }
}

fn has(h: &Harness<'_, Rig>, text: &str) -> bool {
    h.query_all_by_label_contains(text).next().is_some()
}

fn connected_to(sim: &Sim, dir: &Path, extra: &[&str]) -> Harness<'static, Rig> {
    let endpoint = format!("tcp:127.0.0.1:{}", sim.port);
    let dir = dir.display().to_string();
    let mut args = vec![
        "--connect",
        &endpoint,
        "--log-dir",
        &dir,
        "--view",
        "sensors",
    ];
    args.extend_from_slice(extra);
    let mut h = harness(cli(&args), None);
    until(&mut h, "the link to come up", |h| has(h, "Connected"));
    h
}

fn session_folders(log_dir: &Path) -> Vec<PathBuf> {
    let mut folders: Vec<PathBuf> = fs::read_dir(log_dir.join("sessions"))
        .map(|rd| {
            rd.map(|e| e.unwrap().path())
                .filter(|p| p.is_dir())
                .collect()
        })
        .unwrap_or_default();
    folders.sort();
    folders
}

/// The name in `session.json` (the pretty-printed document has one key to a line).
fn session_name(folder: &Path) -> String {
    let text = fs::read_to_string(folder.join("session.json")).unwrap();
    string_after(&text, "\"name\": ").expect("a name")
}

/// The value of the first `"key": "value"` line at or after `from`.
fn string_after(text: &str, key: &str) -> Option<String> {
    let line = text.lines().find(|l| l.trim_start().starts_with(key))?;
    let value = line.trim_start().strip_prefix(key)?.trim_end_matches(',');
    Some(value.trim_matches('"').to_owned())
}

/// The marks of the folder as (kind, text), in the document's order.
fn marks(folder: &Path) -> Vec<(String, String)> {
    let text = fs::read_to_string(folder.join("session.json")).unwrap();
    let marks_at = text.find("\"marks\": [").expect("a marks list");
    let rest = &text[marks_at..];
    let end = rest.find("\n  ]").unwrap_or(rest.len());
    let mut out = Vec::new();
    let mut kind = None;
    for line in rest[..end].lines() {
        let line = line.trim();
        if let Some(k) = line.strip_prefix("\"kind\": ") {
            kind = Some(k.trim_end_matches(',').trim_matches('"').to_owned());
        } else if let Some(t) = line.strip_prefix("\"text\": ") {
            out.push((
                kind.take().unwrap_or_default(),
                t.trim_end_matches(',').trim_matches('"').to_owned(),
            ));
        }
    }
    out
}

fn type_into(h: &mut Harness<'_, Rig>, label: &str, text: &str) {
    h.get_by_label(label).focus();
    h.get_by_label(label).type_text(text);
    h.run();
}

fn leftovers(log_dir: &Path) -> Vec<String> {
    fs::read_dir(log_dir.join("sessions"))
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with('.'))
        .collect()
}

#[test]
fn start_mark_with_a_note_and_stop_write_the_session_folder() {
    let sim = Sim::start();
    let dir = temp_dir("full");
    let mut h = connected_to(&sim, &dir, &[]);
    type_into(&mut h, "Session", "gyro cal");
    h.get_by_label("Start").click();
    until(&mut h, "the session to run", |h| has(h, "No marks yet"));
    frames_for(&mut h, Duration::from_millis(400));
    type_into(&mut h, "Note", "board flipped");
    h.get_by_label("Mark").click();
    until(&mut h, "the mark to show", |h| has(h, "Marks (1)"));
    assert!(has(&h, "board flipped"));
    h.get_by_label("Stop").click();
    until(&mut h, "Saved", |h| has(h, "Saved:"));

    let folders = session_folders(&dir);
    assert_eq!(folders.len(), 1, "{folders:?}");
    assert!(folders[0].join("session.tlog").is_file());
    assert_eq!(session_name(&folders[0]), "gyro cal");
    let user: Vec<_> = marks(&folders[0])
        .into_iter()
        .filter(|(kind, _)| kind == "user")
        .collect();
    assert_eq!(user, [("user".to_owned(), "board flipped".to_owned())]);
    assert_eq!(
        leftovers(&dir),
        Vec::<String>::new(),
        "the recording is deleted"
    );
    assert!(has(&h, &folders[0].display().to_string()));
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn enter_in_the_note_field_marks_and_an_empty_name_is_the_start_time() {
    let sim = Sim::start();
    let dir = temp_dir("enter");
    let mut h = connected_to(&sim, &dir, &[]);
    h.get_by_label("Start").click();
    until(&mut h, "the session to run", |h| has(h, "No marks yet"));
    frames_for(&mut h, Duration::from_millis(300));
    type_into(&mut h, "Note", "one");
    h.key_press(eframe::egui::Key::Enter);
    until(&mut h, "the mark to show", |h| has(h, "Marks (1)"));
    h.get_by_label("Stop").click();
    until(&mut h, "Saved", |h| has(h, "Saved:"));
    let folders = session_folders(&dir);
    assert_eq!(folders.len(), 1);
    let folder = folders[0].file_name().unwrap().to_str().unwrap();
    assert_eq!(
        session_name(&folders[0]),
        folder,
        "the name is the time, as is the folder"
    );
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn a_disconnect_while_running_ends_the_session_and_writes_it() {
    let sim = Sim::start();
    let dir = temp_dir("disconnect");
    let mut h = connected_to(&sim, &dir, &[]);
    type_into(&mut h, "Session", "link drop");
    h.get_by_label("Start").click();
    until(&mut h, "the session to run", |h| has(h, "No marks yet"));
    frames_for(&mut h, Duration::from_millis(400));
    h.get_by_label("Disconnect").click();
    until(&mut h, "Saved", |h| has(h, "Saved:"));
    let folders = session_folders(&dir);
    assert_eq!(folders.len(), 1);
    assert_eq!(session_name(&folders[0]), "link drop");
    assert!(!has(&h, "No marks yet"), "the bar is idle again");
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn closing_the_window_writes_a_running_session() {
    let sim = Sim::start();
    let dir = temp_dir("close");
    let mut h = connected_to(&sim, &dir, &[]);
    type_into(&mut h, "Session", "on exit");
    h.get_by_label("Start").click();
    until(&mut h, "the session to run", |h| has(h, "No marks yet"));
    frames_for(&mut h, Duration::from_millis(400));
    eframe::App::on_exit(h.state_mut().app.as_mut().unwrap(), None);
    let folders = session_folders(&dir);
    assert_eq!(folders.len(), 1, "written before on_exit returns");
    assert_eq!(session_name(&folders[0]), "on exit");
    assert_eq!(leftovers(&dir), Vec::<String>::new());
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn the_window_does_not_wait_for_the_export() {
    let sim = Sim::start();
    let dir = temp_dir("slow");
    let (release, gate) = mpsc::channel::<()>();
    let gate = Mutex::new(gate);
    let exporter: Exporter = Arc::new(move |tlog, out, options| {
        let _ = gate.lock().unwrap().recv();
        export_session(tlog, out, options)
    });
    let endpoint = format!("tcp:127.0.0.1:{}", sim.port);
    let log_dir = dir.display().to_string();
    let mut h = harness(
        cli(&[
            "--connect",
            &endpoint,
            "--log-dir",
            &log_dir,
            "--view",
            "sensors",
        ]),
        Some(exporter),
    );
    until(&mut h, "the link to come up", |h| has(h, "Connected"));
    type_into(&mut h, "Session", "slow");
    h.get_by_label("Start").click();
    until(&mut h, "the session to run", |h| has(h, "No marks yet"));
    frames_for(&mut h, Duration::from_millis(400));
    h.get_by_label("Stop").click();
    until(&mut h, "the export to start", |h| has(h, "Saving"));
    for _ in 0..40 {
        let before = Instant::now();
        h.step();
        assert!(
            before.elapsed() < Duration::from_millis(250),
            "a frame waited for the export"
        );
        thread::sleep(Duration::from_millis(10));
    }
    assert!(has(&h, "Saving"));
    assert!(session_folders(&dir).is_empty(), "still held up");
    release.send(()).unwrap();
    until(&mut h, "Saved", |h| has(h, "Saved:"));
    assert_eq!(session_folders(&dir).len(), 1);
    let _ = fs::remove_dir_all(&dir);
}

/// A recording of a few heartbeats stamped now.
fn write_tlog(path: &Path) {
    let mut encoder = Encoder::new(1, 1);
    let heartbeat = Message::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_STANDBY,
        mavlink_version: 3,
    });
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64;
    let mut file = fs::File::create(path).unwrap();
    for i in 0..5u64 {
        file.write_all(&(now + i * 100_000).to_be_bytes()).unwrap();
        file.write_all(&encoder.encode(&heartbeat)).unwrap();
    }
}

fn start_is_disabled_because(h: &Harness<'_, Rig>, reason: &str) {
    assert!(h.get_by_label("Start").accesskit_node().is_disabled());
    assert!(has(h, reason), "the reason {reason:?} is not shown");
}

#[test]
fn start_is_disabled_in_a_replay_with_the_reason() {
    let dir = temp_dir("replay");
    let tlog = dir.join("a.tlog");
    write_tlog(&tlog);
    let endpoint = format!("tlog:{}", tlog.display());
    let mut h = harness(
        cli(&[
            "--connect",
            &endpoint,
            "--log-dir",
            &dir.display().to_string(),
            "--view",
            "sensors",
        ]),
        None,
    );
    h.run();
    start_is_disabled_because(&h, "replaying a recording");
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn start_is_disabled_without_recording_and_without_a_connection_with_the_reason() {
    let sim = Sim::start();
    let dir = temp_dir("nolog");
    let h = connected_to(&sim, &dir, &["--no-log"]);
    start_is_disabled_because(&h, "recording is off (--no-log)");
    drop(h);
    let mut h = harness(
        cli(&["--log-dir", &dir.display().to_string(), "--view", "sensors"]),
        None,
    );
    h.run();
    start_is_disabled_because(&h, "not connected");
    assert!(session_folders(&dir).is_empty());
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn the_log_tab_lists_the_sessions_newest_first_and_replay_opens_the_tlog() {
    let dir = temp_dir("list");
    let source = dir.join("a.tlog");
    write_tlog(&source);
    let sessions_dir = dir.join("sessions");
    for name in ["alpha", "bravo"] {
        let options = SessionOptions {
            name: name.to_owned(),
            ..SessionOptions::default()
        };
        export_session(&source, &sessions_dir, &options).unwrap();
    }
    let mut h = harness(
        cli(&["--log-dir", &dir.display().to_string(), "--view", "log"]),
        None,
    );
    h.run();
    h.get_by_label("Sessions");
    let alpha = h.get_by_label("alpha").rect();
    let bravo = h.get_by_label("bravo").rect();
    assert!(bravo.min.y < alpha.min.y, "the newest is first");
    let replays = h.get_all_by_label("Replay").count();
    assert_eq!(replays, 2);
    h.get_all_by_label("Replay").next().unwrap().click();
    h.run();
    h.get_by_label("Sensors").click();
    h.run();
    assert!(
        has(&h, sessions::REPLAYING),
        "the session's tlog is replayed"
    );
    let _ = fs::remove_dir_all(&dir);
}
