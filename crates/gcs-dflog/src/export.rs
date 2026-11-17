//! The CSV export: see [`export_with`].

use std::cell::Cell;
use std::collections::HashMap;
use std::fmt::{self, Write as _};
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use crate::condition::{BoundCondition, Latest, instance_column};
use crate::format::{Format, Value};
use crate::reader::{Control, Progress, Reader, Record, scan};

const WRITE_BUFFER: usize = 256 * 1024;

/// What to export: see [`export_with`].
#[derive(Clone, Copy)]
pub struct ExportRequest<'a> {
    /// The record types to export, by the name in the log's FMT. A name the log does not have gives no rows
    /// and no file; a name given twice counts once.
    pub types: &'a [String],
    /// Keep at most this many records per second of each type and instance; `None` keeps every record.
    pub rate_hz: Option<f64>,
    /// Keep only the records at which the condition holds; `None` keeps every record.
    pub condition: Option<&'a BoundCondition>,
    /// The folder of the files; it is created if it does not exist.
    pub folder: &'a Path,
    /// The log's file name without its extension: the files are `<base>_<TYPE>.csv`.
    pub base: &'a str,
}

/// What became of one chosen type.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TypeExport {
    pub type_name: String,
    /// Rows written (the header is not counted).
    pub written: u64,
    /// Records left out because less than `1 / rate` had passed since the last kept one of that instance.
    pub skipped_by_rate: u64,
    /// Records left out because the condition did not hold at them.
    pub skipped_by_condition: u64,
    /// The file, if one was written (that is, if `written` is not 0).
    pub path: Option<PathBuf>,
}

/// What a pass wrote.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Export {
    /// One entry per chosen type, in the order chosen.
    pub types: Vec<TypeExport>,
    /// The pass was stopped by `on_progress`: the files hold the rows up to the point where it stopped.
    pub cancelled: bool,
}

/// An export that did not complete because of an error. The files written until then are left as they
/// are (flushed as far as possible) and named in `files`.
#[derive(Debug)]
pub struct ExportError {
    pub error: io::Error,
    pub files: Vec<PathBuf>,
}

impl fmt::Display for ExportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.error)?;
        if !self.files.is_empty() {
            write!(f, " ({} file(s) written so far are left)", self.files.len())?;
        }
        Ok(())
    }
}

impl std::error::Error for ExportError {}

fn invalid(text: String) -> ExportError {
    ExportError {
        error: io::Error::new(io::ErrorKind::InvalidInput, text),
        files: Vec::new(),
    }
}

/// `text` as a CSV field (RFC 4180): quoted, with quotes doubled, when it holds a comma, a quote or a line break.
fn push_csv_text(line: &mut String, text: &str) {
    if text.contains([',', '"', '\n', '\r']) {
        line.push('"');
        for c in text.chars() {
            if c == '"' {
                line.push('"');
            }
            line.push(c);
        }
        line.push('"');
    } else {
        line.push_str(text);
    }
}

fn push_number(line: &mut String, x: f64) {
    let _ = write!(line, "{x}");
}

fn push_value(line: &mut String, value: Option<Value<'_>>) {
    match value {
        Some(Value::Number(x)) => push_number(line, x),
        Some(Value::Text(t)) => push_csv_text(line, &t),
        Some(Value::Array(a)) => {
            line.push('"');
            for (i, x) in a.iter().enumerate() {
                if i > 0 {
                    line.push(';');
                }
                let _ = write!(line, "{x}");
            }
            line.push('"');
        }
        None => {}
    }
}

struct Target {
    name: String,
    path: PathBuf,
    writer: Option<BufWriter<File>>,
    written: u64,
    by_rate: u64,
    by_condition: u64,
    /// `TimeUS` of the last kept record, per instance (the bits of the instance field's value).
    last_kept: HashMap<u64, u64>,
}

/// How a record type is treated; found once per type number (and again if its FMT is redefined).
struct Resolved {
    name: String,
    target: Option<usize>,
    instance: Option<usize>,
}

struct Exporter<'a> {
    request: &'a ExportRequest<'a>,
    min_gap_us: Option<f64>,
    targets: Vec<Target>,
    resolved: Vec<Option<Resolved>>,
    latest: Option<Latest>,
    holds: bool,
    line: String,
    error: Option<io::Error>,
}

impl Exporter<'_> {
    fn resolve(&mut self, r: &Record<'_>) -> Option<usize> {
        let slot = usize::from(r.type_id());
        if self.resolved[slot]
            .as_ref()
            .is_none_or(|x| x.name != r.type_name())
        {
            self.resolved[slot] = Some(Resolved {
                name: r.type_name().to_owned(),
                target: self.targets.iter().position(|t| t.name == r.type_name()),
                instance: instance_column(r.format()),
            });
        }
        self.resolved[slot].as_ref().and_then(|x| x.target)
    }

    fn open(&mut self, target: usize, format: &Format) -> io::Result<()> {
        fs::create_dir_all(self.request.folder)?;
        let path = &self.targets[target].path;
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| {
                if e.kind() == io::ErrorKind::AlreadyExists {
                    already_exists(path)
                } else {
                    io::Error::new(e.kind(), format!("cannot create {}: {e}", path.display()))
                }
            })?;
        let mut writer = BufWriter::with_capacity(WRITE_BUFFER, file);
        let mut head = String::new();
        for (i, c) in format.columns.iter().enumerate() {
            if i > 0 {
                head.push(',');
            }
            push_csv_text(&mut head, c);
        }
        head.push('\n');
        writer.write_all(head.as_bytes())?;
        self.targets[target].writer = Some(writer);
        Ok(())
    }

    fn feed(&mut self, r: &Record<'_>) {
        if self.error.is_some() {
            return;
        }
        if let (Some(cond), Some(latest)) = (self.request.condition, self.latest.as_mut())
            && r.time_us().is_some()
            && let Some(holds) = latest.update(cond, r)
        {
            self.holds = holds;
        }
        let Some(index) = self.resolve(r) else { return };
        if self.request.condition.is_some() && !self.holds {
            self.targets[index].by_condition += 1;
            return;
        }
        let time = r.time_us();
        let mut key = 0;
        if let (Some(gap), Some(t)) = (self.min_gap_us, time) {
            let instance = self.resolved[usize::from(r.type_id())]
                .as_ref()
                .and_then(|x| x.instance);
            key = instance
                .and_then(|i| r.number(i))
                .map_or(0, |n| (n + 0.0).to_bits());
            let target = &mut self.targets[index];
            if let Some(&last) = target.last_kept.get(&key)
                && !(t >= last && (t - last) as f64 >= gap)
            {
                target.by_rate += 1;
                return;
            }
        }
        if let Err(e) = self.write_row(index, key, time, r) {
            self.error = Some(e);
        }
    }

    fn write_row(
        &mut self,
        index: usize,
        key: u64,
        time: Option<u64>,
        r: &Record<'_>,
    ) -> io::Result<()> {
        if self.targets[index].writer.is_none() {
            self.open(index, r.format())?;
        }
        self.line.clear();
        for i in 0..r.len() {
            if i > 0 {
                self.line.push(',');
            }
            push_value(&mut self.line, r.value(i));
        }
        self.line.push('\n');
        let target = &mut self.targets[index];
        if let Some(writer) = target.writer.as_mut() {
            writer.write_all(self.line.as_bytes())?;
        }
        target.written += 1;
        if self.min_gap_us.is_some()
            && let Some(t) = time
        {
            target.last_kept.insert(key, t);
        }
        Ok(())
    }
}

fn already_exists(path: &Path) -> io::Error {
    io::Error::new(
        io::ErrorKind::AlreadyExists,
        format!(
            "{} exists and is not overwritten: choose another folder or move the file",
            path.display()
        ),
    )
}

fn check_name(what: &str, name: &str) -> Result<(), ExportError> {
    if name.is_empty() || name.contains(['/', '\\', '\0']) {
        return Err(invalid(format!(
            "{what} '{name}' cannot be part of a file name"
        )));
    }
    Ok(())
}

/// Writes the chosen record types of a log as CSV files in one pass over `reader`, with progress and
/// cancellation (`on_progress` answering `Stop` ends the pass; the result then has `cancelled` set).
///
/// # Files
///
/// One file per chosen type that has at least one row written: `<folder>/<base>_<TYPE>.csv`. A file is
/// created new (`create_new`) and never overwritten: if any target file of the chosen types already exists
/// the export is refused before anything is written, with an error of kind `AlreadyExists` that names the
/// file. The folder is created when the first file is. A type without a written row (not in the log, or
/// everything left out by the rate or the condition) creates no file. Writes are buffered.
///
/// If the pass is cancelled, or fails (an I/O error on a file or on the log), the files written so far are
/// left as they are, flushed as far as possible, each holding its header and the rows written until then;
/// the result of a cancel lists them in [`TypeExport::path`], an error names them in [`ExportError::files`].
///
/// # Format
///
/// The first line is the type's FMT column names, comma separated; then one line per row, with every field
/// as decoded (after the scaling of its format character), `,` between fields and `\n` ends the lines:
///
/// - numbers are printed with Rust's shortest text that reads back to the same `f64`: integers without a
///   decimal point (`5`), others as `0.12`, `-1.5e-7` as `-0.00000015`; no exponent, `NaN` and `inf` as such;
/// - text (`n`, `N`, `Z`) is quoted with `"` when it holds a comma, a quote or a line break, and its quotes are
///   doubled (RFC 4180); other text is written as it is;
/// - an `a` array is one quoted field of its 32 values joined with `;` (`"1;-2;3;...;0"`).
///
/// # Rate limit
///
/// With `rate_hz`, per type AND instance (the instance field is the first column after `TimeUS` named `I`,
/// `Inst`, `Instance` or `Id`, as in the conditions; a type without one has a single instance) a record is
/// kept when none of that type and instance has been kept yet, or when its `TimeUS` is at least
/// `1e6 / rate_hz` microseconds after the `TimeUS` of the last kept one (a `TimeUS` smaller than the last
/// kept one's is not). A type without `TimeUS` is not rate limited.
///
/// # Condition
///
/// With `condition`, a record is a candidate only if the condition holds at it, with the semantics of
/// [`find_with`](crate::find_with): the condition is evaluated over the latest record of each type it names,
/// after each record of a type it names, and is false until all of them have appeared. A record that makes it
/// true is written, a record that makes it false is not; a record of a type the condition does not name is
/// written if the condition held after the record before it. The condition is applied first, then the rate
/// limit (which only sees the records the condition let through).
///
/// A `rate_hz` that is not a positive finite number, or a type name or `base` that cannot be part of a file
/// name (empty, or with `/`, `\` or NUL), is an error of kind `InvalidInput`.
pub fn export_with<R: Read>(
    reader: &mut Reader<R>,
    request: &ExportRequest<'_>,
    on_progress: impl FnMut(Progress) -> Control,
) -> Result<Export, ExportError> {
    if let Some(rate) = request.rate_hz
        && !(rate.is_finite() && rate > 0.0)
    {
        return Err(invalid(format!(
            "the rate must be a positive number of Hz, not {rate}"
        )));
    }
    check_name("the base name", request.base)?;
    let mut targets: Vec<Target> = Vec::new();
    for name in request.types {
        if targets.iter().any(|t| &t.name == name) {
            continue;
        }
        check_name("the record type", name)?;
        targets.push(Target {
            name: name.clone(),
            path: request
                .folder
                .join(format!("{}_{}.csv", request.base, name)),
            writer: None,
            written: 0,
            by_rate: 0,
            by_condition: 0,
            last_kept: HashMap::new(),
        });
    }
    if let Some(t) = targets.iter().find(|t| t.path.symlink_metadata().is_ok()) {
        return Err(ExportError {
            error: already_exists(&t.path),
            files: Vec::new(),
        });
    }
    let mut exporter = Exporter {
        request,
        min_gap_us: request.rate_hz.map(|rate| 1e6 / rate),
        targets,
        resolved: (0..256).map(|_| None).collect(),
        latest: request.condition.map(Latest::new),
        holds: false,
        line: String::new(),
        error: None,
    };
    let mut on_progress = on_progress;
    let failed = Cell::new(false);
    let scanned = scan(
        reader,
        |r| {
            exporter.feed(r);
            if exporter.error.is_some() {
                failed.set(true);
            }
        },
        |p| {
            if failed.get() {
                Control::Stop
            } else {
                on_progress(p)
            }
        },
    );
    let mut error = exporter.error.take();
    for t in &mut exporter.targets {
        if let Some(w) = t.writer.as_mut()
            && let Err(e) = w.flush()
            && error.is_none()
        {
            error = Some(e);
        }
    }
    let files = |targets: &[Target]| -> Vec<PathBuf> {
        targets
            .iter()
            .filter(|t| t.writer.is_some())
            .map(|t| t.path.clone())
            .collect()
    };
    let cancelled = match scanned {
        Ok(done) => !done,
        Err(e) => {
            error.get_or_insert(e);
            true
        }
    };
    if let Some(error) = error {
        return Err(ExportError {
            error,
            files: files(&exporter.targets),
        });
    }
    Ok(Export {
        types: exporter
            .targets
            .into_iter()
            .map(|t| TypeExport {
                type_name: t.name,
                written: t.written,
                skipped_by_rate: t.by_rate,
                skipped_by_condition: t.by_condition,
                path: t.writer.is_some().then_some(t.path),
            })
            .collect(),
        cancelled,
    })
}
