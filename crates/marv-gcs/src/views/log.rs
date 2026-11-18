//! Log view: the vehicle's text messages and the state of the recording.

use std::path::{Path, PathBuf};
use std::time::Duration;

use eframe::egui::{self, Color32};
use egui_extras::{Column, TableBuilder};
use gcs_mavlink::services::logs::{
    DownloadError, DownloadState, DownloadStatus, ListError, ListStatus, LogList,
};
use gcs_mavlink::services::texts::Severity;
use gcs_mavlink::tlog::LogState;
use gcs_mavlink::{Access, Command, VehicleState};

use super::devices::{self, DeviceRow};
use super::log_review::Review;
use super::{View, ViewCtx, actions};
use crate::{logfile, sessions, status};

/// What the user chose in this view.
struct Choices {
    /// Show this severity and the worse ones; `Debug` shows everything.
    min_severity: Severity,
    /// Entries up to this session time were hidden with Clear.
    cleared_until: Option<Duration>,
}

impl Default for Choices {
    fn default() -> Self {
        Choices {
            min_severity: Severity::Debug,
            cleared_until: None,
        }
    }
}

/// What the user typed and what the view remembers of the on-board logs.
#[derive(Default)]
struct OnBoard {
    /// The folder a download goes to; `None` until the first frame fills in the window's log directory.
    folder: Option<String>,
    /// Why the last Download did not start (an existing file, a folder that cannot be made).
    start_error: Option<String>,
    /// The file the last download writes to, with the log's number.
    target: Option<(u16, String)>,
    /// The file a finished download asked to have reviewed (its Review button).
    review: Option<String>,
}

#[derive(Default)]
pub struct Log {
    choices: Choices,
    on_board: OnBoard,
    review: Review,
    sessions: sessions::SessionList,
}

impl View for Log {
    fn name(&self) -> &'static str {
        "Log"
    }

    fn update(&mut self, _state: &VehicleState, _now: Option<Duration>) {
        self.sessions.update();
    }

    fn session_changed(&mut self) {
        self.choices.cleared_until = None;
        self.on_board.start_error = None;
        self.on_board.target = None;
    }

    fn ui(&mut self, ui: &mut egui::Ui, cx: &mut ViewCtx) {
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.heading("Station recording (.tlog)");
                recording_part(ui, cx);
                ui.add_space(4.0);
                self.sessions.ui(ui);
                ui.separator();
                ui.heading("On-board logs (SD card)");
                on_board_part(ui, cx, &mut self.on_board);
                if let Some(path) = self.on_board.review.take() {
                    self.review.open(ui.ctx(), PathBuf::from(path));
                }
                ui.separator();
                ui.heading("Review a log file");
                self.review.ui(ui);
                ui.separator();
                ui.heading("Vehicle messages");
                texts_part(ui, cx, &mut self.choices);
            });
    }
}

fn recording_part(ui: &mut egui::Ui, cx: &mut ViewCtx) {
    let connected = cx.now.is_some();
    let replaying = logfile::replaying(ui.ctx());
    let log: &LogState = &cx.state.log;
    ui.horizontal_wrapped(|ui| match &log.recording {
        Some(label) => {
            ui.label(recording_text(label, log)).on_hover_text(label);
            if ui
                .add_enabled(connected, egui::Button::new("Stop"))
                .clicked()
            {
                cx.commands.push(Command::StopLog);
            }
        }
        None => {
            ui.label("Not recording");
            let start = ui.add_enabled(connected && !replaying, egui::Button::new("Start"));
            if replaying {
                start.on_disabled_hover_text(REPLAY_NOTE);
            } else if start.clicked() {
                logfile::request_new_log(ui.ctx());
            }
        }
    });
    if let Some(error) = &log.error {
        ui.colored_label(status::RED, error);
    }
    if let Some(error) = logfile::error(ui.ctx()) {
        ui.colored_label(status::RED, error);
    }
}

const MIB: f64 = 1_048_576.0;
const LIST_HEIGHT: f32 = 170.0;
/// The vehicle messages scroll in a box of this height: the whole tab scrolls, and a list without a limit would
/// take all of it.
const TEXTS_HEIGHT: f32 = 260.0;
const LIST_BUSY: &str = "the list is being read";
const DOWNLOAD_BUSY: &str = "a download is running";
const MARKER_HELP: &str = "The vehicle's last log number: the log being written, or the highest present when none is open. Its size is as of the last flush.";

/// Why a logs button is disabled, or `None`: configure-only, a session, and nothing else running.
fn logs_gate(access: Access, state: &VehicleState, connected: bool) -> Option<&'static str> {
    if let Some(reason) = status::configure_blocked(access) {
        return Some(reason);
    }
    if !connected {
        return Some(status::NOT_CONNECTED);
    }
    if matches!(state.logs.list, ListStatus::Listing { .. }) {
        return Some(LIST_BUSY);
    }
    if state
        .logs
        .download
        .as_ref()
        .is_some_and(|d| d.status == DownloadStatus::Running)
    {
        return Some(DOWNLOAD_BUSY);
    }
    None
}

fn sd_row(state: &VehicleState, now: Option<Duration>) -> Option<DeviceRow> {
    devices::rows(&state.messages, now)
        .into_iter()
        .find(|row| row.device == devices::SD)
}

/// The SD row of `MARV_DEVICE_STATUS` in one line and its colour.
fn sd_line(row: Option<&DeviceRow>) -> (String, Color32) {
    let Some(row) = row else {
        return ("no SD row yet".to_string(), status::GREY);
    };
    let colour = if row.stale {
        status::GREY
    } else {
        devices::state_colour(&row.state)
    };
    (
        format!(
            "SD: {}, last error {}, {} blocks written, {}",
            row.state, row.last_error, row.samples, row.received
        ),
        colour,
    )
}

fn sd_ok(row: Option<&DeviceRow>) -> bool {
    row.is_some_and(|r| r.state == "OK")
}

/// Bytes and MiB of one log, as two cells.
fn size_cells(size: u32) -> (String, String) {
    (size.to_string(), format!("{:.2}", f64::from(size) / MIB))
}

/// What the listing says, in words and colour. `sd_ok`: the SD row is OK (the vehicle answers nothing otherwise).
fn list_line(list: &ListStatus, sd_ok: bool) -> (String, Color32) {
    match list {
        ListStatus::Idle => ("No list yet: press Refresh list".to_string(), status::GREY),
        ListStatus::Listing {
            received,
            expected: Some(expected),
        } => (
            format!("Listing: {received} of {expected} entries"),
            status::YELLOW,
        ),
        ListStatus::Listing { .. } => (
            "Listing: waiting for the vehicle".to_string(),
            status::YELLOW,
        ),
        ListStatus::Done(LogList { logs, .. }) if logs.is_empty() => {
            ("No logs on the card".to_string(), status::GREEN)
        }
        ListStatus::Done(LogList { logs, .. }) => {
            let bytes: u64 = logs.iter().map(|l| u64::from(l.size)).sum();
            (
                format!(
                    "{} logs on the card, {:.2} MiB in all",
                    logs.len(),
                    bytes as f64 / MIB
                ),
                status::GREEN,
            )
        }
        ListStatus::Failed(error) => {
            let why = match error {
                ListError::NoAnswer if !sd_ok => {
                    "no answer from the vehicle (its SD row is not OK)".to_string()
                }
                ListError::NoAnswer => "no answer from the vehicle".to_string(),
                ListError::Incomplete { received, expected } => {
                    format!("incomplete: {received} of {expected} entries arrived")
                }
                ListError::NoVehicle => "the vehicle is not connected".to_string(),
            };
            (format!("Listing failed: {why}"), status::RED)
        }
    }
}

/// kB/s (1000 bytes) of a download between its start and `until`; `None` before time has passed.
fn rate_kb_s(done: u32, started: Duration, until: Duration) -> Option<f64> {
    let seconds = until.saturating_sub(started).as_secs_f64();
    (seconds > 0.0).then(|| f64::from(done) / 1000.0 / seconds)
}

fn rate_text(rate: Option<f64>) -> String {
    rate.map_or_else(String::new, |r| format!(", {r:.1} kB/s"))
}

/// The progress bar of a download: the fraction and its text.
fn progress(d: &DownloadState, now: Option<Duration>) -> (f32, String) {
    let fraction = if d.size == 0 {
        1.0
    } else {
        (f64::from(d.done) / f64::from(d.size)) as f32
    };
    let until = d.ended.or(now).unwrap_or(d.started);
    (
        fraction.clamp(0.0, 1.0),
        format!(
            "{} of {} bytes{}",
            d.done,
            d.size,
            rate_text(rate_kb_s(d.done, d.started, until))
        ),
    )
}

/// The result of the current or last download in words and colour.
fn download_line(
    d: &DownloadState,
    target: Option<&str>,
    now: Option<Duration>,
) -> (String, Color32) {
    let until = d.ended.or(now).unwrap_or(d.started);
    let rate = rate_text(rate_kb_s(d.done, d.started, until));
    let file = target.map_or_else(String::new, |t| format!(" to {t}"));
    match &d.status {
        DownloadStatus::Running => (
            format!(
                "Downloading log {}{file}: {} of {} bytes{rate}",
                d.number, d.done, d.size
            ),
            status::YELLOW,
        ),
        DownloadStatus::Done => (
            format!("Log {} downloaded{file}: {} bytes{rate}", d.number, d.done),
            status::GREEN,
        ),
        DownloadStatus::Cancelled => (
            format!(
                "Log {} cancelled at byte {} of {}; the partial file stays",
                d.number, d.done, d.size
            ),
            status::YELLOW,
        ),
        DownloadStatus::Failed(DownloadError::Stalled { offset }) => (
            format!(
                "Log {} stalled at byte {offset} of {}: no data from the vehicle; {} bytes are in the file",
                d.number, d.size, d.done
            ),
            status::RED,
        ),
        DownloadStatus::Failed(DownloadError::Sink(why)) => (
            format!("Log {}: cannot write the file: {why}", d.number),
            status::RED,
        ),
        DownloadStatus::Failed(DownloadError::NoVehicle) => (
            format!("Log {}: the vehicle is not connected", d.number),
            status::RED,
        ),
    }
}

fn on_board_part(ui: &mut egui::Ui, cx: &mut ViewCtx, on_board: &mut OnBoard) {
    let connected = cx.now.is_some();
    let row = sd_row(cx.state, cx.now);
    let (text, colour) = sd_line(row.as_ref());
    ui.colored_label(colour, text);

    let gate = status::command_gate(cx.access, cx.state, connected);
    let logs_gate = logs_gate(cx.access, cx.state, connected);
    ui.horizontal(|ui| {
        if actions::button(ui, "Restart SD", gate) {
            cx.commands.push(devices::device_command(devices::SD, 1.0));
        }
        if actions::button(ui, "Refresh list", logs_gate) {
            cx.commands.push(Command::ListLogs);
        }
        actions::status_line(ui, cx.state);
    });
    if let Some(reason) = logs_gate.or(gate)
        && cx.access == Access::Configure
        && reason != status::NOT_CONNECTED
    {
        ui.colored_label(status::YELLOW, reason);
    }

    let folder = on_board
        .folder
        .get_or_insert_with(|| logfile::log_dir(ui.ctx()).unwrap_or_else(|| "logs".to_string()));
    ui.horizontal(|ui| {
        ui.label("Download folder");
        ui.add(egui::TextEdit::singleline(folder).desired_width(300.0))
            .accessible_name("download folder");
    });

    let (text, colour) = list_line(&cx.state.logs.list, sd_ok(row.as_ref()));
    ui.colored_label(colour, text);
    if let ListStatus::Done(list) = &cx.state.logs.list
        && !list.logs.is_empty()
    {
        let folder = folder.clone();
        list_table(ui, cx, on_board, list, &folder, logs_gate);
    }
    download_part(ui, cx, on_board);
}

fn list_table(
    ui: &mut egui::Ui,
    cx: &mut ViewCtx,
    on_board: &mut OnBoard,
    list: &LogList,
    folder: &str,
    gate: Option<&'static str>,
) {
    TableBuilder::new(ui)
        .id_salt("onboard_logs_table")
        .striped(true)
        .max_scroll_height(LIST_HEIGHT)
        .column(Column::auto().at_least(60.0))
        .column(Column::auto().at_least(90.0))
        .column(Column::auto().at_least(70.0))
        .column(Column::auto().at_least(110.0))
        .column(Column::remainder().at_least(80.0))
        .header(20.0, |mut header| {
            for title in ["Log", "Bytes", "MiB", "", ""] {
                header.col(|ui| {
                    ui.strong(title);
                });
            }
        })
        .body(|body| {
            body.rows(22.0, list.logs.len(), |mut row| {
                let info = list.logs[row.index()];
                let (bytes, mib) = size_cells(info.size);
                row.col(|ui| {
                    ui.label(info.number.to_string());
                });
                row.col(|ui| {
                    ui.label(bytes);
                });
                row.col(|ui| {
                    ui.label(mib);
                });
                row.col(|ui| {
                    if info.number == list.last_log_num {
                        ui.label("being written").on_hover_text(MARKER_HELP);
                    }
                });
                row.col(|ui| {
                    if actions::button(ui, "Download", gate) {
                        match logfile::download_command(Path::new(folder), info.number, info.size) {
                            Ok((command, path)) => {
                                on_board.start_error = None;
                                on_board.target = Some((info.number, path.display().to_string()));
                                cx.commands.push(command);
                            }
                            Err(e) => on_board.start_error = Some(e),
                        }
                    }
                });
            });
        });
}

fn download_part(ui: &mut egui::Ui, cx: &mut ViewCtx, on_board: &mut OnBoard) {
    if let Some(error) = &on_board.start_error {
        ui.colored_label(status::RED, error);
    }
    let Some(download) = &cx.state.logs.download else {
        return;
    };
    let target = on_board
        .target
        .as_ref()
        .filter(|(number, _)| *number == download.number)
        .map(|(_, path)| path.as_str());
    let running = download.status == DownloadStatus::Running;
    if running {
        let (fraction, text) = progress(download, cx.now);
        ui.horizontal(|ui| {
            ui.add(
                egui::ProgressBar::new(fraction)
                    .text(text)
                    .desired_width(360.0),
            );
            if ui.button("Cancel").clicked() {
                cx.commands.push(Command::CancelLogDownload);
            }
        });
    }
    let (text, colour) = download_line(download, target, cx.now);
    let review = (download.status == DownloadStatus::Done)
        .then(|| target.map(str::to_string))
        .flatten();
    ui.horizontal(|ui| {
        ui.colored_label(colour, text);
        if let Some(path) = review
            && ui.button("Review").clicked()
        {
            on_board.review = Some(path);
        }
    });
}

const REPLAY_NOTE: &str = "Nothing is recorded while a log file is replayed";

/// The log's file name, not its whole path, so the figures stay in sight.
fn recording_text(label: &str, log: &LogState) -> String {
    let name = Path::new(label)
        .file_name()
        .map_or(label, |n| n.to_str().unwrap_or(label));
    format!(
        "Recording to {name}: {} frames, {:.1} kB",
        log.frames,
        log.bytes as f64 / 1000.0
    )
}

fn severity_choice(severity: Severity) -> String {
    match severity {
        Severity::Debug => "Everything".to_string(),
        other => format!("{} and worse", other.name()),
    }
}

fn texts_part(ui: &mut egui::Ui, cx: &ViewCtx, choices: &mut Choices) {
    let entries = &cx.state.texts.entries;
    // A new session starts its clock again: an old marker would hide its first messages.
    match (choices.cleared_until, entries.back()) {
        (Some(_), None) => choices.cleared_until = None,
        (Some(cleared), Some(last)) if last.time < cleared => choices.cleared_until = None,
        _ => {}
    }

    ui.horizontal(|ui| {
        let label = ui.label("Minimum severity");
        egui::ComboBox::from_id_salt("log_min_severity")
            .selected_text(severity_choice(choices.min_severity))
            .show_ui(ui, |ui| {
                for severity in [
                    Severity::Debug,
                    Severity::Info,
                    Severity::Notice,
                    Severity::Warning,
                    Severity::Error,
                    Severity::Critical,
                    Severity::Alert,
                    Severity::Emergency,
                ] {
                    ui.selectable_value(
                        &mut choices.min_severity,
                        severity,
                        severity_choice(severity),
                    );
                }
            })
            .response
            .labelled_by(label.id);
        if ui.button("Clear").clicked() {
            choices.cleared_until = entries.back().map(|e| e.time);
        }
    });

    let shown: Vec<_> = entries
        .iter()
        .filter(|e| e.severity <= choices.min_severity)
        .filter(|e| choices.cleared_until.is_none_or(|c| e.time > c))
        .collect();
    if shown.is_empty() {
        let text = if entries.is_empty() {
            "No messages from the vehicle yet"
        } else {
            "No messages to show"
        };
        ui.label(text);
        return;
    }
    egui::ScrollArea::vertical()
        .id_salt("vehicle_messages")
        .auto_shrink([false, false])
        .max_height(TEXTS_HEIGHT)
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for entry in shown {
                let line = format!(
                    "{:>8.1}  {:<9}  {}",
                    entry.time.as_secs_f64(),
                    entry.severity.name(),
                    entry.text
                );
                let mut text = egui::RichText::new(line).monospace();
                if let Some(colour) = colour(entry.severity) {
                    text = text.color(colour);
                }
                ui.label(text);
            }
        });
}

fn colour(severity: Severity) -> Option<Color32> {
    if severity <= Severity::Error {
        Some(status::RED)
    } else if severity == Severity::Warning {
        Some(status::YELLOW)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn log(frames: u64, bytes: u64) -> LogState {
        LogState {
            frames,
            bytes,
            ..LogState::default()
        }
    }

    #[test]
    fn the_recording_line_names_the_file_not_the_path() {
        let long = "/home/someone/a/very/deep/directory/of/flight/logs/2026-11-12_01-23-45Z.tlog";
        assert_eq!(
            recording_text(long, &log(120, 45_600)),
            "Recording to 2026-11-12_01-23-45Z.tlog: 120 frames, 45.6 kB"
        );
        assert_eq!(
            recording_text("a.tlog", &log(1, 0)),
            "Recording to a.tlog: 1 frames, 0.0 kB"
        );
        assert_eq!(
            recording_text("logs/", &log(1, 0)),
            "Recording to logs: 1 frames, 0.0 kB"
        );
    }

    use gcs_mavlink::services::logs::LogInfo;

    fn download(done: u32, status: DownloadStatus, ended: Option<u64>) -> DownloadState {
        DownloadState {
            number: 7,
            size: 2_000_000,
            done,
            status,
            requests: 1,
            started: Duration::from_secs(10),
            ended: ended.map(Duration::from_secs),
        }
    }

    #[test]
    fn sizes_are_in_bytes_and_mib() {
        assert_eq!(
            size_cells(1_048_576),
            ("1048576".to_string(), "1.00".to_string())
        );
        assert_eq!(size_cells(0).1, "0.00");
        assert_eq!(size_cells(1_320_000).1, "1.26");
    }

    #[test]
    fn the_list_has_a_text_for_each_state() {
        let line = |s: &ListStatus, ok| list_line(s, ok);
        assert!(line(&ListStatus::Idle, true).0.contains("Refresh list"));
        assert_eq!(
            line(
                &ListStatus::Listing {
                    received: 2,
                    expected: Some(5)
                },
                true
            )
            .0,
            "Listing: 2 of 5 entries"
        );
        assert!(
            line(
                &ListStatus::Listing {
                    received: 0,
                    expected: None
                },
                true
            )
            .0
            .contains("waiting")
        );
        assert_eq!(
            line(&ListStatus::Done(LogList::default()), true).0,
            "No logs on the card"
        );
        let done = ListStatus::Done(LogList {
            logs: vec![
                LogInfo {
                    number: 1,
                    size: 1_048_576,
                },
                LogInfo {
                    number: 3,
                    size: 1_048_576,
                },
            ],
            last_log_num: 3,
        });
        assert_eq!(line(&done, true).0, "2 logs on the card, 2.00 MiB in all");
        let none = line(&ListStatus::Failed(ListError::NoAnswer), true);
        assert_eq!(
            (none.0.as_str(), none.1),
            ("Listing failed: no answer from the vehicle", status::RED)
        );
        assert!(
            line(&ListStatus::Failed(ListError::NoAnswer), false)
                .0
                .contains("SD row is not OK")
        );
        assert_eq!(
            line(
                &ListStatus::Failed(ListError::Incomplete {
                    received: 3,
                    expected: 5
                }),
                true
            )
            .0,
            "Listing failed: incomplete: 3 of 5 entries arrived"
        );
    }

    #[test]
    fn a_download_has_progress_a_rate_and_a_text_for_each_end() {
        let running = download(500_000, DownloadStatus::Running, None);
        let (fraction, text) = progress(&running, Some(Duration::from_secs(15)));
        assert_eq!(fraction, 0.25);
        assert_eq!(text, "500000 of 2000000 bytes, 100.0 kB/s");
        assert_eq!(progress(&running, None).1, "500000 of 2000000 bytes");
        let done = download(2_000_000, DownloadStatus::Done, Some(30));
        assert_eq!(progress(&done, Some(Duration::from_secs(99))).0, 1.0);
        let (text, colour) = download_line(&done, Some("logs/00000007.BIN"), None);
        assert_eq!(
            text,
            "Log 7 downloaded to logs/00000007.BIN: 2000000 bytes, 100.0 kB/s"
        );
        assert_eq!(colour, status::GREEN);
        let stalled = download(
            900,
            DownloadStatus::Failed(DownloadError::Stalled { offset: 900 }),
            Some(20),
        );
        let (text, colour) = download_line(&stalled, None, None);
        assert!(
            text.contains("stalled at byte 900 of 2000000")
                && text.contains("900 bytes are in the file"),
            "{text}"
        );
        assert_eq!(colour, status::RED);
        let writer = download(
            0,
            DownloadStatus::Failed(DownloadError::Sink("disk full".into())),
            Some(11),
        );
        assert!(
            download_line(&writer, None, None)
                .0
                .contains("cannot write the file: disk full")
        );
        let cancelled = download(100, DownloadStatus::Cancelled, Some(12));
        assert!(
            download_line(&cancelled, None, None)
                .0
                .contains("cancelled at byte 100 of 2000000")
        );
        assert_eq!(download(0, DownloadStatus::Done, Some(10)).size, 2_000_000);
        assert_eq!(
            rate_kb_s(10, Duration::from_secs(1), Duration::from_secs(1)),
            None
        );
    }

    #[test]
    fn an_empty_download_is_complete() {
        let mut d = download(0, DownloadStatus::Done, Some(10));
        d.size = 0;
        assert_eq!(progress(&d, None).0, 1.0);
    }

    #[test]
    fn the_logs_buttons_have_their_reasons() {
        let mut state = VehicleState::default();
        assert_eq!(
            logs_gate(Access::Operate, &state, true),
            status::configure_blocked(Access::Operate)
        );
        assert_eq!(
            logs_gate(Access::Configure, &state, false),
            Some(status::NOT_CONNECTED)
        );
        assert_eq!(logs_gate(Access::Configure, &state, true), None);
        state.logs.list = ListStatus::Listing {
            received: 0,
            expected: None,
        };
        assert_eq!(logs_gate(Access::Configure, &state, true), Some(LIST_BUSY));
        state.logs.list = ListStatus::Idle;
        state.logs.download = Some(download(1, DownloadStatus::Running, None));
        assert_eq!(
            logs_gate(Access::Configure, &state, true),
            Some(DOWNLOAD_BUSY)
        );
        state.logs.download = Some(download(1, DownloadStatus::Done, Some(11)));
        assert_eq!(logs_gate(Access::Configure, &state, true), None);
    }

    #[test]
    fn the_sd_line_is_the_row_or_says_there_is_none() {
        assert_eq!(sd_line(None).0, "no SD row yet");
        let row = DeviceRow {
            device: 9,
            name: "SD".into(),
            state: "OK".into(),
            last_error: "NONE".into(),
            samples: "1234".into(),
            samples_per_second: String::new(),
            last_sample: String::new(),
            errors: String::new(),
            timeouts: String::new(),
            recoveries: String::new(),
            attempts: String::new(),
            overflows: String::new(),
            config_restored: String::new(),
            flags: String::new(),
            received: "0.4 s".into(),
            sender: "1/1".into(),
            stale: false,
        };
        assert_eq!(
            sd_line(Some(&row)),
            (
                "SD: OK, last error NONE, 1234 blocks written, 0.4 s".to_string(),
                status::GREEN
            )
        );
        assert!(sd_ok(Some(&row)));
        let stale = DeviceRow { stale: true, ..row };
        assert_eq!(sd_line(Some(&stale)).1, status::GREY);
        assert!(!sd_ok(None));
    }
}
