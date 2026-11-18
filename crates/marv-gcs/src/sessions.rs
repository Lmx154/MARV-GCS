//! Monitoring sessions: the owner names a stretch of monitoring on the Sensors tab, drops marks,
//! and stops; the window then writes a session folder (`gcs_link::session_folder`) into
//! `<log dir>/sessions/`.
//!
//! While a session runs, its frames go to a recording of their own, `<log dir>/sessions/.recording-<UTC time>.tlog`,
//! beside the automatic recording (which is not touched). The export runs on a thread of its own; the recording
//! is deleted when the folder is written and kept (its path is in the message) when the export fails.
//!
//! The views and the app talk through the egui context, as in [`crate::logfile`]: the app publishes a
//! [`BarStatus`] every frame, the views leave [`Request`]s.

use std::fs::{self, File, OpenOptions};
use std::io::{self, BufReader, BufWriter, ErrorKind};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use eframe::egui;
use gcs_link::session_folder::{
    Exported, SessionInfo, SessionOptions, UserMark, export_session, list_sessions,
};
use gcs_mavlink::services::params::{FetchStatus, ParamTable, ParamValue};
use gcs_mavlink::tlog::{LogSink, TlogReader};
use gcs_mavlink::{Command, Session};

use crate::logfile;

/// The folder of the session folders, inside the log directory.
pub const SUBDIR: &str = "sessions";

/// How long a stop waits for the session thread to close the recording before it exports anyway.
const CLOSE_WAIT: Duration = Duration::from_secs(3);

pub const REPLAYING: &str = "replaying a recording";
pub const RECORDING_OFF: &str = "recording is off (--no-log)";
const CLOSING: &str = "closing the previous session";

/// Writes a session folder from a recording; the window uses [`export_session`], a test may hold it up.
pub type Exporter =
    Arc<dyn Fn(&Path, &Path, &SessionOptions) -> io::Result<Exported> + Send + Sync>;

pub fn default_exporter() -> Exporter {
    Arc::new(export_session)
}

// ---- what the views see and ask ----

/// What the Sensors tab's bar shows; published by the app every frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct BarStatus {
    pub running: Option<RunningStatus>,
    /// Why Start is disabled, or `None`.
    pub blocked: Option<String>,
    /// An export is under way.
    pub saving: bool,
    /// The outcome of the last stop: the folder written, or the error in words.
    pub last: Option<Result<String, String>>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct RunningStatus {
    pub name: String,
    pub elapsed: Duration,
    /// Time since the start and the note, in the order made.
    pub marks: Vec<(Duration, String)>,
}

/// What a view asks of the app.
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    /// An empty name means the start time.
    Start(String),
    Mark(String),
    Stop,
    /// Replay this `.tlog` (the Log tab's session list).
    Replay(PathBuf),
}

fn status_id() -> egui::Id {
    egui::Id::unique("marv_gcs_session_bar_status")
}

fn requests_id() -> egui::Id {
    egui::Id::unique("marv_gcs_session_requests")
}

fn generation_id() -> egui::Id {
    egui::Id::unique("marv_gcs_session_generation")
}

pub fn publish(ctx: &egui::Context, status: BarStatus) {
    ctx.data_mut(|d| d.insert_temp(status_id(), status));
}

pub fn status(ctx: &egui::Context) -> BarStatus {
    ctx.data(|d| d.get_temp::<BarStatus>(status_id()))
        .unwrap_or_default()
}

pub fn request(ctx: &egui::Context, request: Request) {
    ctx.data_mut(|d| {
        d.get_temp_mut_or_default::<Vec<Request>>(requests_id())
            .push(request)
    });
}

pub fn take_requests(ctx: &egui::Context) -> Vec<Request> {
    ctx.data_mut(|d| d.remove_temp::<Vec<Request>>(requests_id()))
        .unwrap_or_default()
}

/// A session folder was written: the Log tab's list is out of date.
pub fn bump_generation(ctx: &egui::Context) {
    ctx.data_mut(|d| *d.get_temp_mut_or_default::<u64>(generation_id()) += 1);
}

pub fn generation(ctx: &egui::Context) -> u64 {
    ctx.data(|d| d.get_temp::<u64>(generation_id()))
        .unwrap_or(0)
}

/// `m:ss`, or `h:mm:ss` from an hour on.
pub fn clock(elapsed: Duration) -> String {
    let secs = elapsed.as_secs();
    if secs >= 3600 {
        format!("{}:{:02}:{:02}", secs / 3600, secs / 60 % 60, secs % 60)
    } else {
        format!("{}:{:02}", secs / 60, secs % 60)
    }
}

// ---- the session ----

struct Run {
    name: String,
    started: Instant,
    tlog: PathBuf,
    label: String,
    parameters: Option<Vec<(String, f64)>>,
    marks: Vec<UserMark>,
    shown_marks: Vec<(Duration, String)>,
}

enum Active {
    Running(Run),
    /// Stopped: waiting until the session thread has closed the recording.
    Closing {
        run: Run,
        since: Instant,
        complete: bool,
    },
}

struct Export {
    done: Receiver<Result<String, String>>,
    thread: JoinHandle<()>,
}

pub struct Manager {
    dir: PathBuf,
    ctx: egui::Context,
    exporter: Exporter,
    active: Option<Active>,
    exports: Vec<Export>,
    last: Option<Result<String, String>>,
}

impl Manager {
    /// `log_dir` is the window's log directory; the session folders go to its `sessions` folder.
    pub fn new(ctx: &egui::Context, log_dir: &Path) -> Self {
        Manager {
            dir: log_dir.join(SUBDIR),
            ctx: ctx.clone(),
            exporter: default_exporter(),
            active: None,
            exports: Vec::new(),
            last: None,
        }
    }

    pub fn set_exporter(&mut self, exporter: Exporter) {
        self.exporter = exporter;
    }

    pub fn sessions_dir(&self) -> &Path {
        &self.dir
    }

    /// A session is running or closing: no other can start.
    pub fn busy(&self) -> bool {
        self.active.is_some()
    }

    pub fn running(&self) -> bool {
        matches!(self.active, Some(Active::Running(_)))
    }

    /// What the bar shows. `blocked` is the reason from the app (connection, replay, `--no-log`).
    pub fn bar_status(&self, blocked: Option<String>) -> BarStatus {
        let blocked = blocked.or_else(|| {
            matches!(self.active, Some(Active::Closing { .. })).then(|| CLOSING.to_owned())
        });
        BarStatus {
            running: match &self.active {
                Some(Active::Running(run)) => Some(RunningStatus {
                    name: run.name.clone(),
                    elapsed: run.started.elapsed(),
                    marks: run.shown_marks.clone(),
                }),
                _ => None,
            },
            blocked,
            saving: !self.exports.is_empty() || matches!(self.active, Some(Active::Closing { .. })),
            last: self.last.clone(),
        }
    }

    /// Open the recording and tell the session to write to it.
    pub fn start(&mut self, session: &Session, name: &str) -> Result<(), String> {
        if self.active.is_some() {
            return Err("a session is already running".into());
        }
        let (command, tlog, label) = recording(&self.dir, unix_secs())?;
        let parameters = parameter_snapshot(&session.state().params);
        session.send(command);
        self.last = None;
        self.active = Some(Active::Running(Run {
            name: name.trim().to_owned(),
            started: Instant::now(),
            tlog,
            label,
            parameters,
            marks: Vec::new(),
            shown_marks: Vec::new(),
        }));
        Ok(())
    }

    /// Show an error in the bar (a session that could not start).
    pub fn report(&mut self, error: String) {
        self.last = Some(Err(error));
    }

    pub fn mark(&mut self, note: &str) {
        if let Some(Active::Running(run)) = &mut self.active {
            run.marks.push(UserMark {
                time_us: unix_micros(),
                text: note.trim().to_owned(),
            });
            run.shown_marks
                .push((run.started.elapsed(), note.trim().to_owned()));
        }
    }

    /// End a running session: the recording is closed, then the export starts (see [`Manager::poll`]).
    /// With a `session` the close is asked of it; without, the recording is already complete.
    pub fn stop(&mut self, session: Option<&Session>) {
        if !self.running() {
            return;
        }
        let Some(Active::Running(run)) = self.active.take() else {
            return;
        };
        if let Some(session) = session {
            session.send(Command::StopSessionLog);
        }
        self.active = Some(Active::Closing {
            run,
            since: Instant::now(),
            complete: session.is_none(),
        });
    }

    /// The session thread has ended (its handle was dropped and joined): a recording being closed is complete.
    pub fn session_gone(&mut self) {
        if let Some(Active::Closing { complete, .. }) = &mut self.active {
            *complete = true;
        }
    }

    /// Once a frame. `closed` is the label of the recording the session closed last. Starts the export of a
    /// closed recording and picks up the result of the finished exports.
    pub fn poll(&mut self, closed: Option<&str>) {
        if let Some(Active::Closing {
            run,
            since,
            complete,
        }) = &self.active
            && (*complete || closed == Some(run.label.as_str()) || since.elapsed() >= CLOSE_WAIT)
            && let Some(Active::Closing { run, .. }) = self.active.take()
        {
            self.spawn_export(run);
        }
        let mut index = 0;
        while index < self.exports.len() {
            match self.exports[index].done.try_recv() {
                Ok(result) => {
                    let export = self.exports.remove(index);
                    let _ = export.thread.join();
                    if result.is_ok() {
                        bump_generation(&self.ctx);
                    }
                    self.last = Some(result);
                }
                Err(TryRecvError::Empty) => index += 1,
                Err(TryRecvError::Disconnected) => {
                    let export = self.exports.remove(index);
                    let _ = export.thread.join();
                    self.last = Some(Err("the export stopped without a result".into()));
                }
            }
        }
    }

    fn spawn_export(&mut self, run: Run) {
        let (tx, done) = mpsc::channel();
        let exporter = Arc::clone(&self.exporter);
        let dir = self.dir.clone();
        let ctx = self.ctx.clone();
        let thread = thread::Builder::new()
            .name("gcs-session-export".into())
            .spawn(move || {
                let _ = tx.send(export_run(&exporter, &dir, run));
                ctx.request_repaint();
            })
            .expect("spawn gcs-session-export thread");
        self.exports.push(Export { done, thread });
    }

    /// The window is closing and its session is gone: end what runs, write it, and wait for the exports. Nothing is shown any more,
    /// so a failure is reported on stderr. Blocks for as long as the exports take.
    pub fn finish(&mut self) {
        self.stop(None);
        self.session_gone();
        self.poll(None);
        for export in self.exports.drain(..) {
            if let Ok(Err(e)) = export.done.recv() {
                eprintln!("marv-gcs: {e}");
            }
            let _ = export.thread.join();
        }
    }
}

fn unix_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

fn unix_micros() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| u64::try_from(d.as_micros()).unwrap_or(u64::MAX))
}

/// Create `<dir>/.recording-<UTC time>.tlog` (a second one in the same second gets `-2`, `-3`, ...); the command that
/// makes the session record to it, the path and the label the session will report as closed.
fn recording(dir: &Path, unix_secs: u64) -> Result<(Command, PathBuf, String), String> {
    fs::create_dir_all(dir)
        .map_err(|e| format!("cannot create session folder {}: {e}", dir.display()))?;
    let stamp = logfile::file_name(unix_secs);
    let stem = stamp.strip_suffix(".tlog").unwrap_or(&stamp);
    let mut attempt = 1u32;
    loop {
        let path = if attempt == 1 {
            dir.join(format!(".recording-{stem}.tlog"))
        } else {
            dir.join(format!(".recording-{stem}-{attempt}.tlog"))
        };
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(file) => {
                let label = path.display().to_string();
                let command = Command::StartSessionLog {
                    sink: LogSink::new(BufWriter::new(file)),
                    label: label.clone(),
                };
                return Ok((command, path, label));
            }
            Err(e) if e.kind() == ErrorKind::AlreadyExists => attempt += 1,
            Err(e) => return Err(format!("cannot create {}: {e}", path.display())),
        }
    }
}

/// The parameters as the vehicle sent them, when the whole list is here; `None` otherwise (the converter then
/// takes the first `PARAM_VALUE` of each name).
pub fn parameter_snapshot(table: &ParamTable) -> Option<Vec<(String, f64)>> {
    if table.fetch != FetchStatus::Complete || table.params.is_empty() {
        return None;
    }
    Some(
        table
            .params
            .keys()
            .filter_map(|name| {
                let value = match table.value(name)? {
                    // The same shortest decimal form the converter gives the values it decodes.
                    ParamValue::F32(v) => v.to_string().parse().unwrap_or_else(|_| f64::from(v)),
                    other => other.as_f64(),
                };
                Some((name.clone(), value))
            })
            .collect(),
    )
}

/// The name of a session started without one: the time of its first frame, like the recordings' names.
fn time_name(tlog: &Path) -> String {
    File::open(tlog)
        .ok()
        .and_then(|file| TlogReader::new(BufReader::new(file)).next())
        .and_then(Result::ok)
        .map_or_else(
            || "session".to_owned(),
            |first| {
                let name = logfile::file_name(first.time_us / 1_000_000);
                name.strip_suffix(".tlog").unwrap_or(&name).to_owned()
            },
        )
}

/// Write the folder; delete the recording on success. The text is what the bar shows.
fn export_run(exporter: &Exporter, dir: &Path, run: Run) -> Result<String, String> {
    if fs::metadata(&run.tlog).is_ok_and(|m| m.len() == 0) {
        let _ = fs::remove_file(&run.tlog);
        return Err("session not saved: no frames were recorded".into());
    }
    let name = if run.name.is_empty() {
        time_name(&run.tlog)
    } else {
        run.name.clone()
    };
    let options = SessionOptions {
        name,
        parameters: run.parameters,
        marks: run.marks,
    };
    match exporter(&run.tlog, dir, &options) {
        Ok(exported) => {
            let _ = fs::remove_file(&run.tlog);
            Ok(exported.folder.display().to_string())
        }
        Err(e) => Err(format!(
            "session not saved: {e} (the recording is kept at {})",
            run.tlog.display()
        )),
    }
}

// ---- the bar ----

/// What the user typed in the bar.
#[derive(Default)]
pub struct BarInput {
    name: String,
    note: String,
}

/// The session bar: compact, at the top of the Sensors tab.
pub fn bar_ui(ui: &mut egui::Ui, input: &mut BarInput) {
    let ctx = ui.ctx().clone();
    let status = status(&ctx);
    match &status.running {
        None => {
            ui.horizontal(|ui| {
                let label = ui.label("Session");
                let field = ui.add(
                    egui::TextEdit::singleline(&mut input.name)
                        .hint_text("name (empty: the start time)")
                        .desired_width(220.0),
                );
                field.labelled_by(label.id);
                let start = ui.add_enabled(status.blocked.is_none(), egui::Button::new("Start"));
                match &status.blocked {
                    Some(reason) => {
                        start.on_disabled_hover_text(reason);
                        ui.colored_label(crate::status::GREY, reason);
                    }
                    None if start.clicked() => {
                        request(&ctx, Request::Start(std::mem::take(&mut input.name)));
                    }
                    None => {}
                }
                outcome(ui, &status);
            });
        }
        Some(running) => {
            ui.horizontal(|ui| {
                ui.colored_label(crate::status::RED, "REC");
                let name = if running.name.is_empty() {
                    "session"
                } else {
                    &running.name
                };
                ui.strong(format!("{name}  {}", clock(running.elapsed)));
                let label = ui.label("Note");
                let field = ui.add(
                    egui::TextEdit::singleline(&mut input.note)
                        .hint_text("optional")
                        .desired_width(240.0),
                );
                field.clone().labelled_by(label.id);
                let entered = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                if ui.button("Mark").clicked() || entered {
                    request(&ctx, Request::Mark(std::mem::take(&mut input.note)));
                    if entered {
                        field.request_focus();
                    }
                }
                if ui.button("Stop").clicked() {
                    request(&ctx, Request::Stop);
                }
            });
            ui.add(egui::Label::new(marks_line(&running.marks)).truncate());
        }
    }
}

fn outcome(ui: &mut egui::Ui, status: &BarStatus) {
    if status.saving {
        ui.colored_label(crate::status::GREY, "Saving…");
    } else {
        match &status.last {
            Some(Ok(folder)) => {
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(format!("Saved: {folder}")).color(crate::status::GREEN),
                    )
                    .truncate(),
                )
                .on_hover_text(folder);
            }
            Some(Err(e)) => {
                ui.add(
                    egui::Label::new(egui::RichText::new(e).color(crate::status::RED)).truncate(),
                )
                .on_hover_text(e);
            }
            None => {}
        }
    }
}

/// `No marks yet`, or `Marks (2): 0:05 door open; 0:31 gain up`.
pub fn marks_line(marks: &[(Duration, String)]) -> String {
    if marks.is_empty() {
        return "No marks yet".into();
    }
    let list: Vec<String> = marks
        .iter()
        .map(|(at, note)| {
            if note.is_empty() {
                clock(*at)
            } else {
                format!("{} {note}", clock(*at))
            }
        })
        .collect();
    format!("Marks ({}): {}", marks.len(), list.join("; "))
}

// ---- the list in the Log tab ----

/// The session folders under `<log dir>/sessions`, newest first, read when the tab opens and after a session
/// was written.
#[derive(Default)]
pub struct SessionList {
    rows: Vec<SessionInfo>,
    error: Option<String>,
    dir: Option<PathBuf>,
    /// The generation the rows were read at; `None` before the first read.
    read_at: Option<u64>,
    /// The tab was drawn in the previous frame (set by `update`, which runs for every view every frame).
    was_drawn: bool,
    drawn: bool,
}

impl SessionList {
    /// Called every frame, shown or not.
    pub fn update(&mut self) {
        self.was_drawn = self.drawn;
        self.drawn = false;
    }

    fn refresh(&mut self, dir: Option<PathBuf>, generation: u64) {
        self.dir = dir;
        self.read_at = Some(generation);
        self.error = None;
        self.rows.clear();
        let Some(dir) = &self.dir else {
            return;
        };
        match list_sessions(&dir.join(SUBDIR)) {
            Ok(mut rows) => {
                rows.reverse();
                self.rows = rows;
            }
            Err(e) if e.kind() == ErrorKind::NotFound => {}
            Err(e) => self.error = Some(format!("cannot read {}: {e}", dir.join(SUBDIR).display())),
        }
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let dir = logfile::log_dir(&ctx).map(PathBuf::from);
        let generation = generation(&ctx);
        if !self.was_drawn || self.read_at != Some(generation) || self.dir != dir {
            self.refresh(dir, generation);
        }
        self.drawn = true;
        ui.strong("Sessions");
        if let Some(error) = &self.error {
            ui.colored_label(crate::status::RED, error);
        }
        let Some(dir) = &self.dir else {
            ui.label("no log directory");
            return;
        };
        if self.rows.is_empty() {
            ui.label("No sessions yet: start one on the Sensors tab.");
            return;
        }
        let sessions = dir.join(SUBDIR);
        egui::ScrollArea::vertical()
            .id_salt("session_list")
            .max_height(150.0)
            .show(ui, |ui| {
                egui::Grid::new("session_grid")
                    .striped(true)
                    .num_columns(6)
                    .show(ui, |ui| {
                        for head in ["Folder", "Name", "Start", "Duration", "Marks", ""] {
                            ui.strong(head);
                        }
                        ui.end_row();
                        for row in &self.rows {
                            ui.label(&row.folder);
                            ui.label(&row.name);
                            ui.label(&row.start_utc);
                            ui.label(format!("{:.1} s", row.duration_s));
                            ui.label(row.marks.to_string());
                            if ui
                                .button("Replay")
                                .on_hover_text("Open session.tlog as a replay")
                                .clicked()
                            {
                                let tlog = sessions.join(&row.folder).join("session.tlog");
                                request(&ctx, Request::Replay(tlog));
                            }
                            ui.end_row();
                        }
                    });
            });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_clock_has_minutes_and_from_an_hour_hours() {
        assert_eq!(clock(Duration::from_secs(0)), "0:00");
        assert_eq!(clock(Duration::from_secs(75)), "1:15");
        assert_eq!(clock(Duration::from_secs(3600 + 62)), "1:01:02");
    }

    #[test]
    fn the_marks_line_counts_and_lists() {
        assert_eq!(marks_line(&[]), "No marks yet");
        let marks = [
            (Duration::from_secs(5), "door open".to_owned()),
            (Duration::from_secs(61), String::new()),
        ];
        assert_eq!(marks_line(&marks), "Marks (2): 0:05 door open; 1:01");
    }

    #[test]
    fn a_second_recording_in_the_same_second_gets_its_own_file() {
        let dir =
            std::env::temp_dir().join(format!("marv-gcs-sessions-rec-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        let (_, first, _) = recording(&dir, 1_794_446_625).expect("created");
        let (_, second, _) = recording(&dir, 1_794_446_625).expect("created");
        assert_eq!(
            first.file_name().unwrap().to_str().unwrap(),
            ".recording-2026-11-12_01-23-45Z.tlog"
        );
        assert_eq!(
            second.file_name().unwrap().to_str().unwrap(),
            ".recording-2026-11-12_01-23-45Z-2.tlog"
        );
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn parameters_are_a_snapshot_only_when_the_list_is_complete() {
        let mut table = ParamTable::default();
        assert_eq!(parameter_snapshot(&table), None);
        table.fetch = FetchStatus::Complete;
        assert_eq!(parameter_snapshot(&table), None, "complete but empty");
    }
}
