//! The Log tab's section "Review a log file": a DataFlash `.BIN` log read with `gcs-dflog`.
//!
//! Opening a file runs two passes in sequence on one std thread (the summary, then the events); Find and
//! Export are one pass each on a thread of their own. A pass reports its progress through atomics, stops when
//! its cancel flag is set, and hands its result back through a channel the UI polls once per frame; the UI
//! thread never reads the file. The section works on a local file only: it sends nothing to the vehicle.

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use eframe::egui;
use egui_extras::{Column, TableBuilder};
use gcs_dflog::{
    BoundCondition, Condition, Control, Event, EventSource, Export, ExportError, ExportRequest,
    Found, Progress, Reader, Summary, events_with, export_with, find_with, summary_with,
};

use crate::{logfile, status};

const MIB: f64 = 1_048_576.0;
const TABLE_HEIGHT: f32 = 240.0;
const ROW_HEIGHT: f32 = 20.0;
const REPAINT_WHILE_RUNNING: Duration = Duration::from_millis(100);
/// The listing of the log folder is read again when it is older than this and the drop-down is open.
const LISTING_AGE: Duration = Duration::from_secs(1);

/// What a worker reports: its result, `None` when it was cancelled, or why it failed.
type Outcome<T> = Result<Option<T>, String>;

#[derive(Default)]
struct Shared {
    bytes: AtomicU64,
    total: AtomicU64,
    stage: AtomicU64,
    cancel: AtomicBool,
}

/// What a worker sees of its job: where to report progress and whether to stop.
struct Pass {
    shared: Arc<Shared>,
}

impl Pass {
    fn stage(&self, stage: u64) {
        self.shared.stage.store(stage, Ordering::Relaxed);
        self.shared.bytes.store(0, Ordering::Relaxed);
    }

    fn progress(&self, progress: Progress) -> Control {
        self.shared.bytes.store(progress.bytes, Ordering::Relaxed);
        self.shared
            .total
            .store(progress.total.unwrap_or(0), Ordering::Relaxed);
        if self.shared.cancel.load(Ordering::Relaxed) {
            Control::Stop
        } else {
            Control::Continue
        }
    }
}

/// A pass running on its own thread. Dropping the job cancels it and forgets its result.
struct Job<T> {
    shared: Arc<Shared>,
    rx: Receiver<Outcome<T>>,
    stages: u64,
}

impl<T: Send + 'static> Job<T> {
    fn spawn(
        ctx: &egui::Context,
        stages: u64,
        work: impl FnOnce(&Pass) -> Outcome<T> + Send + 'static,
    ) -> Job<T> {
        let shared = Arc::new(Shared::default());
        let pass = Pass {
            shared: Arc::clone(&shared),
        };
        let (tx, rx) = mpsc::channel();
        let ctx = ctx.clone();
        thread::spawn(move || {
            let outcome = work(&pass);
            let _ = tx.send(outcome);
            ctx.request_repaint();
        });
        Job { shared, rx, stages }
    }
}

impl<T> Job<T> {
    fn cancel(&self) {
        self.shared.cancel.store(true, Ordering::Relaxed);
    }

    fn cancelling(&self) -> bool {
        self.shared.cancel.load(Ordering::Relaxed)
    }

    /// The result once there is one.
    fn poll(&self) -> Option<Outcome<T>> {
        match self.rx.try_recv() {
            Ok(outcome) => Some(outcome),
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => Some(Err("the pass ended unexpectedly".to_string())),
        }
    }

    fn fraction(&self) -> f32 {
        let total = self.shared.total.load(Ordering::Relaxed);
        let done = if total == 0 {
            0.0
        } else {
            self.shared.bytes.load(Ordering::Relaxed) as f64 / total as f64
        };
        let stage = self.shared.stage.load(Ordering::Relaxed) as f64;
        (((stage + done.min(1.0)) / self.stages as f64) as f32).clamp(0.0, 1.0)
    }

    fn text(&self, names: &[&str]) -> String {
        let stage = self.shared.stage.load(Ordering::Relaxed);
        let bytes = self.shared.bytes.load(Ordering::Relaxed);
        let total = self.shared.total.load(Ordering::Relaxed);
        let name = names.get(stage as usize).copied().unwrap_or("reading");
        let of = if self.stages > 1 {
            format!(" (pass {} of {})", stage + 1, self.stages)
        } else {
            String::new()
        };
        let cancelling = if self.cancelling() {
            ", cancelling"
        } else {
            ""
        };
        format!("{name}{of}: {bytes} of {total} bytes{cancelling}")
    }
}

impl<T> Drop for Job<T> {
    fn drop(&mut self) {
        self.cancel();
    }
}

/// A progress bar and Cancel; true when Cancel was clicked.
fn progress_row<T>(ui: &mut egui::Ui, job: &Job<T>, names: &[&str]) -> bool {
    ui.horizontal(|ui| {
        ui.add(
            egui::ProgressBar::new(job.fraction())
                .text(job.text(names))
                .desired_width(420.0),
        );
        ui.add_enabled(!job.cancelling(), egui::Button::new("Cancel"))
            .clicked()
    })
    .inner
}

/// What opening a file found.
struct Opened {
    path: PathBuf,
    name: String,
    summary: Summary,
    events: Vec<Event>,
}

type OpenResult = (Summary, Vec<Event>);

fn read_error(path: &Path, e: &std::io::Error) -> String {
    format!("cannot read {}: {e}", path.display())
}

/// The two passes of opening: the summary, then the events.
fn open_pass(path: &Path, pass: &Pass) -> Outcome<OpenResult> {
    pass.stage(0);
    let mut reader = Reader::open(path).map_err(|e| read_error(path, &e))?;
    let Some(summary) =
        summary_with(&mut reader, |p| pass.progress(p)).map_err(|e| read_error(path, &e))?
    else {
        return Ok(None);
    };
    pass.stage(1);
    let mut reader = Reader::open(path).map_err(|e| read_error(path, &e))?;
    let Some(events) =
        events_with(&mut reader, |p| pass.progress(p)).map_err(|e| read_error(path, &e))?
    else {
        return Ok(None);
    };
    Ok(Some((summary, events)))
}

const OPEN_STAGES: [&str; 2] = ["summary", "events"];

/// The `.BIN` files of a folder, newest first.
fn bin_files(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut files: Vec<(SystemTime, PathBuf)> = entries
        .flatten()
        .filter_map(|entry| {
            let path = entry.path();
            let is_bin = path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("bin"));
            let meta = entry.metadata().ok()?;
            (is_bin && meta.is_file())
                .then(|| (meta.modified().unwrap_or(SystemTime::UNIX_EPOCH), path))
        })
        .collect();
    files.sort_by(|a, b| b.cmp(a));
    files.into_iter().map(|(_, path)| path).collect()
}

#[derive(Default)]
struct Listing {
    dir: String,
    files: Vec<PathBuf>,
    read_at: Option<Instant>,
}

impl Listing {
    fn refresh(&mut self, dir: &str, force: bool) {
        let stale = self.read_at.is_none_or(|t| t.elapsed() > LISTING_AGE);
        if force || stale || self.dir != dir {
            self.dir = dir.to_string();
            self.files = bin_files(Path::new(dir));
            self.read_at = Some(Instant::now());
        }
    }
}

/// Which sources of the events table are hidden, and the rows that remain.
#[derive(Default)]
struct EventsView {
    hidden: [bool; EventSource::ALL.len()],
    shown: Option<([bool; EventSource::ALL.len()], Vec<u32>)>,
}

#[derive(Default)]
struct FindPart {
    text: String,
    /// Why the condition was not run: its position and reason.
    error: Option<String>,
    job: Option<Job<Found>>,
    fields: Vec<String>,
    result: Option<(Vec<String>, Found)>,
    note: Option<(String, egui::Color32)>,
}

enum ExportResult {
    Done(Export),
    Cancelled(Export),
    Failed { text: String, files: Vec<PathBuf> },
}

#[derive(Default)]
struct ExportPart {
    /// One flag per entry of the summary's `counts`.
    chosen: Vec<bool>,
    rate_text: String,
    cond_text: String,
    /// Why the export was not started.
    error: Option<String>,
    folder: String,
    job: Option<Job<Result<Export, ExportError>>>,
    result: Option<ExportResult>,
}

/// The section's state: the opened file, the passes in flight and what the user typed.
#[derive(Default)]
pub struct Review {
    path_text: String,
    listing: Listing,
    /// The file being opened and its passes.
    opening: Option<(PathBuf, Job<OpenResult>)>,
    open_error: Option<String>,
    opened: Option<Opened>,
    events: EventsView,
    find: FindPart,
    export: ExportPart,
    /// The folder of the drop-down was listed once.
    started: bool,
}

impl Review {
    /// Opens `path`: a pass reads it on a thread. What was shown of the previous file is dropped and its
    /// passes are cancelled.
    pub fn open(&mut self, ctx: &egui::Context, path: PathBuf) {
        self.path_text = path.display().to_string();
        self.open_error = None;
        self.opened = None;
        self.events = EventsView::default();
        self.find.job = None;
        self.find.result = None;
        self.find.note = None;
        self.find.error = None;
        self.export.job = None;
        self.export.result = None;
        self.export.error = None;
        self.export.chosen.clear();
        self.export.folder = folder_of(&path);
        self.listing.read_at = None;
        let file = path.clone();
        self.opening = Some((path, Job::spawn(ctx, 2, move |pass| open_pass(&file, pass))));
    }

    fn busy(&self) -> bool {
        self.opening.is_some() || self.find.job.is_some() || self.export.job.is_some()
    }

    fn poll(&mut self) {
        if let Some((path, job)) = &self.opening
            && let Some(outcome) = job.poll()
        {
            let path = path.clone();
            self.opening = None;
            match outcome {
                Ok(Some((summary, events))) => {
                    self.export.chosen = vec![false; summary.counts.len()];
                    let name = path
                        .file_name()
                        .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
                    self.opened = Some(Opened {
                        path,
                        name,
                        summary,
                        events,
                    });
                }
                Ok(None) => self.open_error = Some("Open cancelled".to_string()),
                Err(e) => self.open_error = Some(e),
            }
        }
        self.find.poll();
        self.export.poll();
    }

    pub fn ui(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        if let Some(path) = logfile::take_review(&ctx) {
            self.open(&ctx, path);
        }
        self.poll();
        self.open_part(ui, &ctx);
        if let Some(opened) = &self.opened {
            summary_part(ui, opened);
            events_part(ui, opened, &mut self.events);
            find_part(ui, &ctx, opened, &mut self.find);
            export_part(ui, &ctx, opened, &mut self.export);
        }
        if self.busy() {
            ctx.request_repaint_after(REPAINT_WHILE_RUNNING);
        }
    }

    fn open_part(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let dir = logfile::log_dir(ctx).unwrap_or_else(|| "logs".to_string());
        if !self.started {
            self.started = true;
            self.listing.refresh(&dir, true);
        }
        let mut to_open: Option<PathBuf> = None;
        ui.horizontal(|ui| {
            ui.label("Log file");
            let field = ui
                .add(
                    egui::TextEdit::singleline(&mut self.path_text)
                        .desired_width(420.0)
                        .hint_text("path of a .BIN file"),
                )
                .accessible_name("review file");
            let enter = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if ui.button("Open").clicked() || enter {
                let text = self.path_text.trim();
                if text.is_empty() {
                    self.open_error = Some("Type the path of a .BIN file".to_string());
                } else {
                    to_open = Some(PathBuf::from(text));
                }
            }
        });
        ui.horizontal(|ui| {
            let label = ui.label("or choose one");
            egui::ComboBox::from_id_salt("review_files")
                .selected_text(format!(".BIN files in {dir}, newest first"))
                .show_ui(ui, |ui| {
                    self.listing.refresh(&dir, false);
                    if self.listing.files.is_empty() {
                        ui.label("no .BIN files in this folder");
                    }
                    for file in &self.listing.files {
                        let name = file
                            .file_name()
                            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
                        if ui
                            .selectable_label(false, name)
                            .on_hover_text(file.display().to_string())
                            .clicked()
                        {
                            to_open = Some(file.clone());
                        }
                    }
                })
                .response
                .labelled_by(label.id);
        });
        if let Some(path) = to_open {
            self.open(ctx, path);
        }
        let cancel = match &self.opening {
            Some((path, job)) => {
                ui.label(format!("Reading {}", path.display()));
                progress_row(ui, job, &OPEN_STAGES)
            }
            None => false,
        };
        if cancel {
            self.opening = None;
            self.open_error = Some("Open cancelled".to_string());
        }
        if let Some(error) = &self.open_error {
            ui.colored_label(status::RED, error);
        }
    }
}

fn folder_of(path: &Path) -> String {
    match path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p.display().to_string(),
        _ => ".".to_string(),
    }
}

impl FindPart {
    fn poll(&mut self) {
        let Some(job) = &self.job else {
            return;
        };
        let Some(outcome) = job.poll() else {
            return;
        };
        self.job = None;
        let fields = std::mem::take(&mut self.fields);
        match outcome {
            Ok(Some(found)) => self.result = Some((fields, found)),
            Ok(None) => self.note = Some(("Find cancelled".to_string(), status::YELLOW)),
            Err(e) => self.note = Some((e, status::RED)),
        }
    }

    fn start(&mut self, ctx: &egui::Context, opened: &Opened) {
        self.note = None;
        self.error = None;
        let bound = match bind(&self.text, &opened.summary) {
            Ok(bound) => bound,
            Err(e) => {
                self.error = Some(e);
                return;
            }
        };
        self.result = None;
        self.fields = bound.fields().map(str::to_string).collect();
        let path = opened.path.clone();
        self.job = Some(Job::spawn(ctx, 1, move |pass| {
            let mut reader = Reader::open(&path).map_err(|e| read_error(&path, &e))?;
            find_with(&mut reader, &bound, |p| pass.progress(p)).map_err(|e| read_error(&path, &e))
        }));
    }
}

/// The condition in `text` tied to the file's record types, or the error with its position and reason.
fn bind(text: &str, summary: &Summary) -> Result<BoundCondition, String> {
    Condition::parse(text)
        .and_then(|c| c.bind(&summary.formats))
        .map_err(|e| format!("Condition error {e}"))
}

impl ExportPart {
    fn poll(&mut self) {
        let Some(job) = &self.job else {
            return;
        };
        let Some(outcome) = job.poll() else {
            return;
        };
        self.job = None;
        self.result = match outcome {
            Ok(Some(Ok(export))) if export.cancelled => Some(ExportResult::Cancelled(export)),
            Ok(Some(Ok(export))) => Some(ExportResult::Done(export)),
            Ok(Some(Err(e))) => Some(ExportResult::Failed {
                text: e.to_string(),
                files: e.files,
            }),
            Ok(None) => None,
            Err(text) => Some(ExportResult::Failed {
                text,
                files: Vec::new(),
            }),
        };
    }

    fn start(&mut self, ctx: &egui::Context, opened: &Opened) {
        self.error = None;
        self.result = None;
        let types: Vec<String> = opened
            .summary
            .counts
            .iter()
            .zip(&self.chosen)
            .filter(|(_, on)| **on)
            .map(|(count, _)| count.name.clone())
            .collect();
        if types.is_empty() {
            self.error = Some("Choose at least one record type to export".to_string());
            return;
        }
        let rate_hz = match self.rate_text.trim() {
            "" => None,
            text => match text.parse::<f64>() {
                Ok(rate) if rate.is_finite() && rate > 0.0 => Some(rate),
                _ => {
                    self.error = Some(
                        "The rate must be a number of Hz above 0, or empty for every record"
                            .to_string(),
                    );
                    return;
                }
            },
        };
        let condition = if self.cond_text.trim().is_empty() {
            None
        } else {
            match bind(&self.cond_text, &opened.summary) {
                Ok(bound) => Some(bound),
                Err(e) => {
                    self.error = Some(e);
                    return;
                }
            }
        };
        let folder = self.folder.trim();
        if folder.is_empty() {
            self.error = Some("Choose a folder".to_string());
            return;
        }
        let folder = PathBuf::from(folder);
        let path = opened.path.clone();
        let base = path
            .file_stem()
            .map_or_else(String::new, |s| s.to_string_lossy().into_owned());
        self.job = Some(Job::spawn(ctx, 1, move |pass| {
            let mut reader = Reader::open(&path).map_err(|e| read_error(&path, &e))?;
            let request = ExportRequest {
                types: &types,
                rate_hz,
                condition: condition.as_ref(),
                folder: &folder,
                base: &base,
            };
            Ok(Some(export_with(&mut reader, &request, |p| {
                pass.progress(p)
            })))
        }));
    }
}

/// Seconds since boot with three decimals.
fn seconds(time_us: u64) -> String {
    format!("{:.3}", time_us as f64 / 1e6)
}

/// A value of a field for the eye: whole numbers without decimals, others to four places.
fn number(v: f64) -> String {
    if v.is_nan() {
        return "-".to_string();
    }
    if v == v.trunc() && v.abs() < 1e12 {
        return format!("{v:.0}");
    }
    let text = format!("{v:.4}");
    match text.contains('.') {
        true => text.trim_end_matches('0').trim_end_matches('.').to_string(),
        false => text,
    }
}

/// `2026-11-12 01:23:45.678 UTC` of a Unix time in milliseconds.
fn wall_clock(unix_ms: i64) -> String {
    let ms = unix_ms.max(0) as u64;
    let secs = ms / 1000;
    let (year, month, day) = logfile::civil_from_days(secs / 86_400);
    let rest = secs % 86_400;
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02}:{:02}.{:03} UTC",
        rest / 3600,
        rest / 60 % 60,
        rest % 60,
        ms % 1000
    )
}

fn summary_part(ui: &mut egui::Ui, o: &Opened) {
    egui::CollapsingHeader::new("Summary")
        .default_open(true)
        .show(ui, |ui| {
            let s = &o.summary;
            ui.label(format!("File: {}", o.name))
                .on_hover_text(o.path.display().to_string());
            ui.label(format!(
                "Size: {} bytes ({:.2} MiB)",
                s.bytes,
                s.bytes as f64 / MIB
            ));
            match s.duration_us() {
                Some(us) => ui.label(format!("Duration: {} s", seconds(us))),
                None => ui.label("Duration: unknown (no record has a TimeUS)"),
            };
            match s.start_unix_ms() {
                Some(ms) => ui.label(format!("Wall-clock start: {}", wall_clock(ms))),
                None => ui.label("Wall-clock start: not known (no GPS time in the log)"),
            };
            ui.label(format!("Records: {}", s.records));
            ui.label(format!("Bad bytes: {}", s.bad_bytes));
            ui.label(format!("Tail bytes: {}", s.tail_bytes));
            if s.records == 0 {
                ui.colored_label(
                    status::YELLOW,
                    "No record found: this does not look like a DataFlash log",
                );
            }
            ui.label("Records per type");
            ui.horizontal_wrapped(|ui| {
                for count in &s.counts {
                    ui.label(format!("{}: {}", count.name, count.count));
                }
            });
        });
}

fn source_index(source: EventSource) -> usize {
    EventSource::ALL
        .iter()
        .position(|s| *s == source)
        .unwrap_or(0)
}

fn events_part(ui: &mut egui::Ui, o: &Opened, view: &mut EventsView) {
    egui::CollapsingHeader::new("Events")
        .default_open(true)
        .show(ui, |ui| {
            if o.events.is_empty() {
                ui.label("No events: the log has none of the records the timeline reads");
                return;
            }
            ui.horizontal_wrapped(|ui| {
                ui.label("Show");
                for (i, source) in EventSource::ALL.iter().enumerate() {
                    let mut on = !view.hidden[i];
                    if ui.checkbox(&mut on, source.name()).changed() {
                        view.hidden[i] = !on;
                    }
                }
            });
            if view.shown.as_ref().is_none_or(|(h, _)| *h != view.hidden) {
                let rows = (0..o.events.len() as u32)
                    .filter(|&i| !view.hidden[source_index(o.events[i as usize].source)])
                    .collect();
                view.shown = Some((view.hidden, rows));
            }
            let Some((_, rows)) = &view.shown else {
                return;
            };
            ui.label(format!("{} of {} events", rows.len(), o.events.len()));
            let wall = o.summary.time_base_unix_us.is_some();
            let mut table = TableBuilder::new(ui)
                .id_salt("review_events")
                .striped(true)
                .max_scroll_height(TABLE_HEIGHT)
                .column(Column::auto().at_least(80.0));
            if wall {
                table = table.column(Column::auto().at_least(190.0));
            }
            table
                .column(Column::auto().at_least(60.0))
                .column(Column::auto().at_least(50.0))
                .column(Column::remainder().at_least(200.0))
                .header(ROW_HEIGHT, |mut header| {
                    let titles = if wall {
                        vec!["Time (s)", "Wall clock (UTC)", "Source", "", "Event"]
                    } else {
                        vec!["Time (s)", "Source", "", "Event"]
                    };
                    for title in titles {
                        header.col(|ui| {
                            ui.strong(title);
                        });
                    }
                })
                .body(|body| {
                    body.rows(ROW_HEIGHT, rows.len(), |mut row| {
                        let event = &o.events[rows[row.index()] as usize];
                        row.col(|ui| {
                            ui.label(egui::RichText::new(seconds(event.time_us)).monospace());
                        });
                        if wall {
                            row.col(|ui| {
                                // "UTC" is in the column title, so a cell fits on one line.
                                let text = o
                                    .summary
                                    .unix_ms_at(event.time_us)
                                    .map_or_else(String::new, wall_clock);
                                let text = text.trim_end_matches(" UTC");
                                ui.add(
                                    egui::Label::new(egui::RichText::new(text).monospace())
                                        .extend(),
                                );
                            });
                        }
                        row.col(|ui| {
                            ui.label(event.source.name());
                        });
                        row.col(|ui| {
                            if event.initial {
                                ui.weak("initial");
                            }
                        });
                        row.col(|ui| {
                            ui.add(egui::Label::new(&event.text).truncate());
                        });
                    });
                });
        });
}

fn find_part(ui: &mut egui::Ui, ctx: &egui::Context, o: &Opened, f: &mut FindPart) {
    egui::CollapsingHeader::new("Find where a condition holds")
        .default_open(true)
        .show(ui, |ui| {
            let mut go = false;
            ui.horizontal(|ui| {
                ui.label("Condition");
                let field = ui
                    .add(
                        egui::TextEdit::singleline(&mut f.text)
                            .desired_width(420.0)
                            .hint_text("BAT.Volt < 14.8"),
                    )
                    .accessible_name("find condition");
                let enter = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                let idle = f.job.is_none();
                go = (ui.add_enabled(idle, egui::Button::new("Find")).clicked() || enter) && idle;
            });
            if go {
                f.start(ctx, o);
            }
            if let Some(error) = &f.error {
                ui.colored_label(status::RED, error);
            }
            if let Some(job) = &f.job
                && progress_row(ui, job, &["searching"])
            {
                f.job = None;
                f.fields.clear();
                f.note = Some(("Find cancelled".to_string(), status::YELLOW));
            }
            if let Some((text, colour)) = &f.note {
                ui.colored_label(*colour, text);
            }
            if let Some((fields, found)) = &f.result {
                intervals(ui, fields, found);
            }
        });
}

fn intervals(ui: &mut egui::Ui, fields: &[String], found: &Found) {
    if found.intervals.is_empty() {
        ui.label("The condition never held");
        return;
    }
    ui.label(format!("{} interval(s)", found.total()));
    if found.not_kept > 0 {
        ui.colored_label(
            status::YELLOW,
            format!(
                "{} more not kept (the first {} are listed)",
                found.not_kept,
                found.intervals.len()
            ),
        );
    }
    let mut table = TableBuilder::new(ui)
        .id_salt("review_intervals")
        .striped(true)
        .max_scroll_height(TABLE_HEIGHT)
        .column(Column::auto().at_least(80.0))
        .column(Column::auto().at_least(80.0))
        .column(Column::auto().at_least(90.0));
    for _ in fields {
        table = table
            .column(Column::auto().at_least(70.0))
            .column(Column::auto().at_least(70.0));
    }
    table
        .header(ROW_HEIGHT, |mut header| {
            for title in ["Start (s)", "End (s)", "Duration (s)"] {
                header.col(|ui| {
                    ui.strong(title);
                });
            }
            for field in fields {
                for edge in ["min", "max"] {
                    header.col(|ui| {
                        ui.strong(format!("{field} {edge}"));
                    });
                }
            }
        })
        .body(|body| {
            body.rows(ROW_HEIGHT, found.intervals.len(), |mut row| {
                let interval = &found.intervals[row.index()];
                row.col(|ui| {
                    ui.label(seconds(interval.start_us));
                });
                row.col(|ui| {
                    ui.label(interval.end_us.map_or_else(|| "open".to_string(), seconds));
                });
                row.col(|ui| {
                    ui.label(seconds(interval.duration_us));
                });
                for i in 0..fields.len() {
                    let range = interval.ranges.get(i);
                    row.col(|ui| {
                        ui.label(range.map_or_else(String::new, |r| number(r.min)));
                    });
                    row.col(|ui| {
                        ui.label(range.map_or_else(String::new, |r| number(r.max)));
                    });
                }
            });
        });
}

fn export_part(ui: &mut egui::Ui, ctx: &egui::Context, o: &Opened, e: &mut ExportPart) {
    egui::CollapsingHeader::new("Export to CSV")
        .default_open(true)
        .show(ui, |ui| {
            e.chosen.resize(o.summary.counts.len(), false);
            ui.horizontal(|ui| {
                ui.label("Record types");
                if ui.button("Select all").clicked() {
                    e.chosen.fill(true);
                }
                if ui.button("Select none").clicked() {
                    e.chosen.fill(false);
                }
            });
            ui.horizontal_wrapped(|ui| {
                for (count, on) in o.summary.counts.iter().zip(e.chosen.iter_mut()) {
                    ui.checkbox(on, format!("{} ({} records)", count.name, count.count));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Rate limit (Hz, per type and instance; empty keeps every record)");
                ui.add(egui::TextEdit::singleline(&mut e.rate_text).desired_width(70.0))
                    .accessible_name("export rate");
            });
            ui.horizontal(|ui| {
                ui.label("Only where (optional condition)");
                ui.add(
                    egui::TextEdit::singleline(&mut e.cond_text)
                        .desired_width(420.0)
                        .hint_text("BAT.Volt < 14.8"),
                )
                .accessible_name("export condition");
            });
            let mut go = false;
            ui.horizontal(|ui| {
                ui.label("Folder");
                ui.add(egui::TextEdit::singleline(&mut e.folder).desired_width(420.0))
                    .accessible_name("export folder");
                go = ui
                    .add_enabled(e.job.is_none(), egui::Button::new("Export CSV"))
                    .clicked();
            });
            if go {
                e.start(ctx, o);
            }
            if let Some(error) = &e.error {
                ui.colored_label(status::RED, error);
            }
            if let Some(job) = &e.job
                && progress_row(ui, job, &["exporting"])
            {
                job.cancel();
            }
            match &e.result {
                Some(ExportResult::Done(export)) => export_lines(ui, "Export finished", export),
                Some(ExportResult::Cancelled(export)) => export_lines(
                    ui,
                    "Export cancelled: the files hold the rows written until then",
                    export,
                ),
                Some(ExportResult::Failed { text, files }) => {
                    ui.colored_label(status::RED, format!("Export failed: {text}"));
                    for file in files {
                        ui.label(format!("left: {}", file.display()));
                    }
                }
                None => {}
            }
        });
}

fn export_lines(ui: &mut egui::Ui, title: &str, export: &Export) {
    ui.colored_label(status::GREEN, title);
    for t in &export.types {
        let mut text = format!("{}: {} rows written", t.type_name, t.written);
        if t.skipped_by_rate > 0 {
            text.push_str(&format!(", {} skipped by the rate", t.skipped_by_rate));
        }
        if t.skipped_by_condition > 0 {
            text.push_str(&format!(
                ", {} skipped by the condition",
                t.skipped_by_condition
            ));
        }
        match &t.path {
            Some(path) => text.push_str(&format!(" to {}", path.display())),
            None => text.push_str(", no file"),
        }
        ui.label(text);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbers_for_the_eye() {
        assert_eq!(number(14.0), "14");
        assert_eq!(number(-3.0), "-3");
        assert_eq!(number(14.25), "14.25");
        assert_eq!(number(0.1 + 0.2), "0.3");
        assert_eq!(number(f64::NAN), "-");
        assert_eq!(number(1.0 / 3.0), "0.3333");
        assert_eq!(seconds(1_234_567), "1.235");
        assert_eq!(seconds(0), "0.000");
    }

    #[test]
    fn a_unix_time_in_words() {
        assert_eq!(wall_clock(0), "1970-01-01 00:00:00.000 UTC");
        // 2024-02-29 12:34:56.789 UTC
        assert_eq!(wall_clock(1_709_210_096_789), "2024-02-29 12:34:56.789 UTC");
    }

    #[test]
    fn the_folder_of_a_path() {
        assert_eq!(folder_of(Path::new("/a/b/c.BIN")), "/a/b");
        assert_eq!(folder_of(Path::new("c.BIN")), ".");
    }

    #[test]
    fn bin_files_are_listed_newest_first() {
        let dir = std::env::temp_dir().join(format!("marv_review_list_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        let base = SystemTime::UNIX_EPOCH + Duration::from_secs(1_000_000);
        for (name, age) in [("a.BIN", 30), ("c.bin", 10), ("b.BIN", 20), ("x.txt", 5)] {
            let file = fs::File::create(dir.join(name)).unwrap();
            file.set_modified(base + Duration::from_secs(age)).unwrap();
        }
        fs::create_dir(dir.join("dir.BIN")).unwrap();
        let names: Vec<String> = bin_files(&dir)
            .iter()
            .map(|p| p.file_name().unwrap().to_string_lossy().into_owned())
            .collect();
        assert_eq!(names, ["a.BIN", "b.BIN", "c.bin"]);
        assert!(bin_files(&dir.join("missing")).is_empty());
        let _ = fs::remove_dir_all(&dir);
    }
}
