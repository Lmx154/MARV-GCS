//! Headless tests of the Log view: states built by hand, drawn by egui_kittest, no window.

use std::time::Duration;

use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use gcs_mavlink::services::texts::{Severity, TextEntry};
use gcs_mavlink::tlog::LogState;
use gcs_mavlink::{Command, VehicleState};
use marv_gcs::logfile;
use marv_gcs::views::{self, View, ViewCtx};

fn log_view() -> Box<dyn View> {
    views::all()
        .into_iter()
        .find(|v| v.name() == "Log")
        .expect("Log view is registered")
}

fn entry(secs: f64, severity: Severity, text: &str) -> TextEntry {
    TextEntry {
        time: Duration::from_secs_f64(secs),
        severity,
        text: text.to_string(),
        system_id: 1,
        component_id: 1,
    }
}

fn state_with_texts() -> VehicleState {
    let mut state = VehicleState::default();
    state.texts.entries.extend([
        entry(1.0, Severity::Info, "info text one"),
        entry(2.5, Severity::Warning, "warning text two"),
        entry(3.0, Severity::Error, "error text three"),
        entry(4.0, Severity::Emergency, "emergency text four"),
        entry(5.0, Severity::Debug, "debug text five"),
    ]);
    state
}

fn recording_state() -> VehicleState {
    VehicleState {
        log: LogState {
            recording: Some("logs/a.tlog".into()),
            frames: 120,
            bytes: 45_600,
            error: None,
        },
        ..VehicleState::default()
    }
}

fn harness<'a>(
    state: &'a VehicleState,
    commands: &'a mut Vec<Command>,
    now: Option<Duration>,
) -> Harness<'a> {
    let mut view = log_view();
    Harness::new_ui(move |ui| {
        let mut cx = ViewCtx {
            state,
            commands: &mut *commands,
            now,
            access: gcs_mavlink::Access::Configure,
        };
        view.ui(ui, &mut cx);
    })
}

fn draws(state: &VehicleState, now: Option<Duration>) {
    let mut commands = Vec::new();
    let mut h = harness(state, &mut commands, now);
    h.run();
}

#[test]
fn draws_an_empty_state() {
    draws(&VehicleState::default(), None);
    draws(&VehicleState::default(), Some(Duration::from_secs(1)));
}

#[test]
fn draws_entries_of_several_severities() {
    draws(&state_with_texts(), Some(Duration::from_secs(10)));
}

#[test]
fn draws_without_a_session() {
    draws(&state_with_texts(), None);
}

#[test]
fn empty_state_says_so() {
    let state = VehicleState::default();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, None);
    h.run();
    h.get_by_label("No messages from the vehicle yet");
}

#[test]
fn entry_is_shown_with_time_and_severity() {
    let state = state_with_texts();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(10)));
    h.run();
    h.get_by_label_contains("warning text two");
    h.get_by_label_contains("2.5");
    h.get_by_label_contains("WARNING");
    assert!(
        h.query_by_label("No messages from the vehicle yet")
            .is_none()
    );
}

#[test]
fn minimum_severity_hides_the_lower_ones() {
    let state = state_with_texts();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(10)));
    h.run();
    h.get_by_label_contains("info text one");
    h.get_by_label("Minimum severity").click();
    h.run();
    h.get_by_label("WARNING and worse").click();
    h.run();
    assert!(h.query_by_label_contains("info text one").is_none());
    assert!(h.query_by_label_contains("debug text five").is_none());
    h.get_by_label_contains("warning text two");
    h.get_by_label_contains("error text three");
    h.get_by_label_contains("emergency text four");
}

#[test]
fn clear_hides_what_was_received_but_not_the_state() {
    let state = state_with_texts();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(10)));
    h.run();
    h.get_by_label("Clear").click();
    h.run();
    assert!(h.query_by_label_contains("info text one").is_none());
    assert!(h.query_by_label_contains("debug text five").is_none());
    drop(h);
    assert_eq!(state.texts.entries.len(), 5);
}

#[test]
fn recording_text_while_recording() {
    let state = recording_state();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    h.run();
    h.get_by_label("Recording to a.tlog: 120 frames, 45.6 kB");
    h.get_by_label("Stop");
    assert!(h.query_by_label("Not recording").is_none());
}

#[test]
fn recording_text_when_stopped() {
    let state = VehicleState::default();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    h.run();
    h.get_by_label("Not recording");
    h.get_by_label("Start");
}

#[test]
fn recording_error_is_shown() {
    let state = VehicleState {
        log: LogState {
            error: Some("disk full".into()),
            ..LogState::default()
        },
        ..VehicleState::default()
    };
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    h.run();
    h.get_by_label("Not recording");
    h.get_by_label("disk full");
}

#[test]
fn stop_sends_stop_log() {
    let state = recording_state();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    h.run();
    h.get_by_label("Stop").click();
    h.run();
    drop(h);
    assert!(
        matches!(commands.as_slice(), [Command::StopLog]),
        "{commands:?}"
    );
}

#[test]
fn buttons_are_disabled_without_a_session() {
    for state in [recording_state(), VehicleState::default()] {
        let mut commands = Vec::new();
        let mut h = harness(&state, &mut commands, None);
        h.run();
        let name = if state.log.recording.is_some() {
            "Stop"
        } else {
            "Start"
        };
        h.get_by_label(name).click();
        h.run();
        let requested = logfile::take_request(&h.ctx);
        drop(h);
        assert!(commands.is_empty(), "{commands:?}");
        assert!(!requested);
    }
}

#[test]
fn start_asks_the_app_for_a_new_log() {
    let state = VehicleState::default();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    h.run();
    assert!(!logfile::take_request(&h.ctx));
    h.get_by_label("Start").click();
    h.run();
    assert!(logfile::take_request(&h.ctx));
    assert!(!logfile::take_request(&h.ctx));
    drop(h);
    assert!(commands.is_empty());
}

#[test]
fn app_side_failure_is_shown_in_red_text() {
    let state = VehicleState::default();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    logfile::set_error(&h.ctx, Some("cannot create log directory x".into()));
    h.run();
    h.get_by_label("cannot create log directory x");
}

#[test]
fn bar_offers_a_log_file_choice_with_path_and_speed() {
    use marv_gcs::connection::{Action, Bar};
    use marv_gcs::status;

    let mut bar = Bar::default();
    bar.set_replay(
        std::path::Path::new("run.tlog"),
        gcs_mavlink::tlog::ReplaySpeed::Fast,
    );
    let mut action = None;
    let mut h = Harness::new_ui(|ui| {
        if let Some(a) = bar.ui(ui, false, status::not_connected()) {
            action = Some(a);
        }
    });
    h.run();
    h.get_by_label("Log file");
    h.get_by_label("real time");
    h.get_by_label("fast");
    h.get_by_label("Path");
    h.get_by_label("Connect").click();
    h.run();
    drop(h);
    assert_eq!(
        action,
        Some(Action::Replay(
            "run.tlog".into(),
            gcs_mavlink::tlog::ReplaySpeed::Fast
        ))
    );
}

#[test]
fn a_new_session_drops_the_cleared_marker() {
    use std::cell::RefCell;
    use std::rc::Rc;

    let state = state_with_texts();
    let shared = Rc::new(RefCell::new(log_view()));
    let inner = shared.clone();
    let mut h = Harness::new_ui(move |ui| {
        let mut commands = Vec::new();
        let mut cx = ViewCtx {
            state: &state,
            commands: &mut commands,
            now: Some(Duration::from_secs(10)),
            access: gcs_mavlink::Access::Configure,
        };
        inner.borrow_mut().ui(ui, &mut cx);
    });
    h.run();
    h.get_by_label("Clear").click();
    h.run();
    assert!(h.query_by_label_contains("info text one").is_none());

    shared.borrow_mut().session_changed();
    h.run();
    h.get_by_label_contains("info text one");
}

#[test]
fn a_long_path_shows_the_file_name_and_the_figures_in_a_narrow_window() {
    let long = "/home/someone/projects/flights/2026/november/session-with-a-long-name/logs/2026-11-12_01-23-45Z.tlog";
    let mut state = recording_state();
    state.log.recording = Some(long.into());
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    h.run();
    let line = h.get_by_label("Recording to 2026-11-12_01-23-45Z.tlog: 120 frames, 45.6 kB");
    assert!(line.rect().max.x <= 800.0, "{:?}", line.rect());
    assert!(h.query_by_label_contains("/home/someone").is_none());
    h.get_by_label("Stop");
}

#[test]
fn start_is_disabled_with_a_reason_while_a_log_is_replayed() {
    let state = VehicleState::default();
    let mut commands = Vec::new();
    let mut h = harness(&state, &mut commands, Some(Duration::from_secs(1)));
    logfile::set_replaying(&h.ctx, true);
    h.run();
    h.get_by_label("Start").click();
    h.run();
    assert!(!logfile::take_request(&h.ctx));
    h.get_by_label("Start").hover();
    h.run();
    h.get_by_label_contains("Nothing is recorded while a log file is replayed");
}

// ---- the on-board logs (SD card) section ----

use gcs_mavlink::Access;
use gcs_mavlink::services::logs::{DownloadState, DownloadStatus, ListStatus, LogInfo, LogList};
use gcs_mavlink::services::messages::{Field, FieldValue, MessageEntry, MessageKey};

fn sd_row(state: &str) -> (MessageKey, MessageEntry) {
    let text = |n: &str, v: &str| Field {
        name: n.into(),
        value: FieldValue::Text(v.into()),
    };
    (
        MessageKey {
            system_id: 1,
            component_id: 1,
            message_id: 42_421,
            instance: 9,
        },
        MessageEntry {
            name: "MARV_DEVICE_STATUS".into(),
            label: "MARV_DEVICE_STATUS[9]".into(),
            count: 3,
            rate_hz: 1.0,
            last_seen: Duration::from_secs(9),
            fields: vec![
                text("device_id", "MARV_DEVICE_SD"),
                text("state", &format!("MARV_DEVICE_STATE_{state}")),
                text("last_error", "MARV_DEVICE_ERROR_NONE"),
                Field {
                    name: "samples".into(),
                    value: FieldValue::Number(4321.0),
                },
            ],
        },
    )
}

fn card_state() -> VehicleState {
    let mut state = VehicleState::default();
    let (key, entry) = sd_row("OK");
    state.messages.entries.insert(key, entry);
    state.logs.list = ListStatus::Done(LogList {
        logs: vec![
            LogInfo {
                number: 1,
                size: 1_048_576,
            },
            LogInfo {
                number: 4,
                size: 2_097_152,
            },
        ],
        last_log_num: 4,
    });
    state
}

fn run_with(
    state: &VehicleState,
    access: Access,
    now: Option<Duration>,
    folder: &str,
    f: impl FnOnce(&mut Harness<'_>),
) -> Vec<Command> {
    let mut commands = Vec::new();
    {
        let mut view = log_view();
        let mut h = Harness::new_ui(|ui| {
            logfile::set_log_dir(ui.ctx(), std::path::Path::new(folder));
            let mut cx = ViewCtx {
                state,
                commands: &mut commands,
                now,
                access,
            };
            view.ui(ui, &mut cx);
        });
        h.run();
        f(&mut h);
        h.run();
    }
    commands
}

const NOW: Option<Duration> = Some(Duration::from_secs(10));

#[test]
fn the_two_kinds_of_log_have_their_own_sections() {
    run_with(
        &VehicleState::default(),
        Access::Configure,
        NOW,
        "logs",
        |h| {
            h.get_by_label("Station recording (.tlog)");
            h.get_by_label("On-board logs (SD card)");
            h.get_by_label("no SD row yet");
        },
    );
}

#[test]
fn the_sd_row_is_one_line() {
    run_with(&card_state(), Access::Configure, NOW, "logs", |h| {
        h.get_by_label("SD: OK, last error NONE, 4321 blocks written, 1.0 s");
    });
}

#[test]
fn restart_sd_sends_the_device_restart_for_id_9() {
    let commands = run_with(&card_state(), Access::Configure, NOW, "logs", |h| {
        h.get_by_label("Restart SD").click();
    });
    assert_eq!(commands.len(), 1, "{commands:?}");
    let Command::SendCommand { command, params } = &commands[0] else {
        panic!("{:?}", commands[0])
    };
    assert_eq!(*command as u32, 42_430);
    assert_eq!(*params, [9.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
}

#[test]
fn on_an_operate_link_the_logs_buttons_are_disabled_and_send_nothing() {
    let commands = run_with(&card_state(), Access::Operate, NOW, "logs", |h| {
        for label in ["Restart SD", "Refresh list"] {
            assert!(
                h.get_by_label(label).accesskit_node().is_disabled(),
                "{label}"
            );
            h.get_by_label(label).click();
        }
        for b in h.get_all_by_label("Download") {
            assert!(b.accesskit_node().is_disabled());
        }
    });
    assert!(commands.is_empty(), "{commands:?}");
}

#[test]
fn refresh_list_sends_list_logs() {
    let commands = run_with(
        &VehicleState::default(),
        Access::Configure,
        NOW,
        "logs",
        |h| {
            h.get_by_label("Refresh list").click();
        },
    );
    assert!(matches!(commands[..], [Command::ListLogs]), "{commands:?}");
}

#[test]
fn the_table_shows_number_bytes_mib_and_marks_the_log_being_written() {
    run_with(&card_state(), Access::Configure, NOW, "logs", |h| {
        h.get_by_label("1048576");
        h.get_by_label("2097152");
        h.get_by_label("1.00");
        h.get_by_label("2.00");
        assert_eq!(h.get_all_by_label("being written").count(), 1);
        assert_eq!(h.get_all_by_label("Download").count(), 2);
    });
}

fn temp_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("marv-views-log-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

#[test]
fn download_sends_the_command_and_creates_the_file_in_the_folder() {
    let dir = temp_dir("dl");
    let commands = run_with(
        &card_state(),
        Access::Configure,
        NOW,
        dir.to_str().unwrap(),
        |h| {
            h.get_all_by_label("Download").nth(1).unwrap().click();
        },
    );
    assert_eq!(commands.len(), 1, "{commands:?}");
    let Command::DownloadLog { number, size, .. } = &commands[0] else {
        panic!("{:?}", commands[0])
    };
    assert_eq!((*number, *size), (4, 2_097_152));
    assert!(dir.join("00000004.BIN").is_file());
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn download_refuses_to_overwrite_and_shows_why() {
    let dir = temp_dir("exists");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("00000001.BIN"), b"mine").unwrap();
    let commands = run_with(
        &card_state(),
        Access::Configure,
        NOW,
        dir.to_str().unwrap(),
        |h| {
            h.get_all_by_label("Download").next().unwrap().click();
            h.run();
            h.get_by_label_contains("00000001.BIN exists and is not overwritten");
        },
    );
    assert!(commands.is_empty(), "{commands:?}");
    assert_eq!(std::fs::read(dir.join("00000001.BIN")).unwrap(), b"mine");
    let _ = std::fs::remove_dir_all(&dir);
}

fn downloading(status: DownloadStatus, ended: Option<u64>) -> VehicleState {
    let mut state = card_state();
    state.logs.download = Some(DownloadState {
        number: 4,
        size: 2_000,
        done: 500,
        status,
        requests: 1,
        started: Duration::from_secs(8),
        ended: ended.map(Duration::from_secs),
    });
    state
}

#[test]
fn a_running_download_shows_progress_cancel_and_blocks_other_downloads() {
    let state = downloading(DownloadStatus::Running, None);
    let commands = run_with(&state, Access::Configure, NOW, "logs", |h| {
        assert!(
            h.get_all_by_label_contains("500 of 2000 bytes, 0.2 kB/s")
                .count()
                >= 1
        );
        for b in h.get_all_by_label("Download") {
            assert!(b.accesskit_node().is_disabled());
        }
        assert!(
            h.get_by_label("Refresh list")
                .accesskit_node()
                .is_disabled()
        );
        h.get_by_label("Cancel").click();
    });
    assert!(
        matches!(commands[..], [Command::CancelLogDownload]),
        "{commands:?}"
    );
}

#[test]
fn the_end_of_a_download_is_said_in_words_and_cancel_goes_away() {
    let state = downloading(DownloadStatus::Cancelled, Some(9));
    run_with(&state, Access::Configure, NOW, "logs", |h| {
        h.get_by_label_contains("Log 4 cancelled at byte 500 of 2000");
        assert!(h.query_by_label("Cancel").is_none());
    });
}
