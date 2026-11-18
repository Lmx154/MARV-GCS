//! `marv-cli session export` and `session list`: files only, no link and no vehicle.

use std::io::Write;
use std::path::{Path, PathBuf};

use gcs_link::session_folder::{SessionOptions, export_session, list_sessions};

use crate::args::{DEFAULT_SESSIONS, SessionCommand};
use crate::exit;

pub fn run(command: &SessionCommand, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let result = match command {
        SessionCommand::Export {
            file,
            name,
            out: to,
        } => export(file, name.as_deref(), to.as_deref(), out),
        SessionCommand::List { dir } => list(dir.as_deref(), out, err),
    };
    match result {
        Ok(()) => exit::OK,
        Err(message) => {
            let _ = writeln!(err, "marv-cli: {message}");
            exit::FAILED
        }
    }
}

fn export(
    file: &Path,
    name: Option<&str>,
    to: Option<&Path>,
    out: &mut dyn Write,
) -> Result<(), String> {
    let name = name
        .map(str::to_owned)
        .or_else(|| file.file_stem().map(|s| s.to_string_lossy().into_owned()))
        .unwrap_or_else(|| "session".to_owned());
    let dir: PathBuf = match to {
        Some(dir) => dir.to_path_buf(),
        None => file
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."))
            .join("sessions"),
    };
    let options = SessionOptions {
        name,
        ..SessionOptions::default()
    };
    let exported = export_session(file, &dir, &options)
        .map_err(|e| format!("session export of {} failed: {e}", file.display()))?;
    let write = |result: std::io::Result<()>| result.map_err(|e| e.to_string());
    write(writeln!(out, "session\t{}", exported.folder.display()))?;
    for (name, rows) in &exported.files {
        write(writeln!(out, "{name}\t{rows} rows"))?;
    }
    for (name, why) in &exported.skipped {
        write(writeln!(out, "skipped\t{name}\t{why}"))?;
    }
    write(writeln!(out, "marks\t{}", exported.marks))
}

fn list(dir: Option<&Path>, out: &mut dyn Write, err: &mut dyn Write) -> Result<(), String> {
    let dir = dir.unwrap_or(Path::new(DEFAULT_SESSIONS));
    let sessions = match list_sessions(dir) {
        Ok(sessions) => sessions,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
        Err(e) => return Err(format!("cannot read {}: {e}", dir.display())),
    };
    if sessions.is_empty() {
        let _ = writeln!(err, "marv-cli: no sessions in {}", dir.display());
    }
    for s in sessions {
        writeln!(
            out,
            "{}\t{}\t{}\t{:.1} s\t{} marks",
            s.folder, s.name, s.start_utc, s.duration_s, s.marks
        )
        .map_err(|e| e.to_string())?;
    }
    Ok(())
}
