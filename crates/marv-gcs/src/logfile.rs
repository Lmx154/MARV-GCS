//! Log files: the name of a new `.tlog`, creating it, and the small notes the Log view and the
//! app pass to each other through the egui context (a request for a new log, the last failure).

use std::fs::{self, File, OpenOptions};
use std::io::{BufWriter, ErrorKind};
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use eframe::egui;
use gcs_mavlink::Command;
use gcs_mavlink::tlog::LogSink;

/// File name for a log started `unix_secs` seconds after the Unix epoch, in UTC:
/// `2026-11-12_01-23-45Z.tlog`.
pub fn file_name(unix_secs: u64) -> String {
    let days = unix_secs / 86_400;
    let rest = unix_secs % 86_400;
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02}_{:02}-{:02}-{:02}Z.tlog",
        rest / 3600,
        rest / 60 % 60,
        rest % 60
    )
}

/// Year, month and day of a day count since 1970-01-01 (proleptic Gregorian calendar).
fn civil_from_days(days: u64) -> (u64, u64, u64) {
    let z = days + 719_468;
    let era = z / 146_097;
    let day_of_era = z % 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let shifted_month = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * shifted_month + 2) / 5 + 1;
    let month = if shifted_month < 10 {
        shifted_month + 3
    } else {
        shifted_month - 9
    };
    let year = era * 400 + year_of_era + u64::from(month <= 2);
    (year, month, day)
}

/// Create the directory if needed and a new log file in it named after the current UTC time;
/// the command that makes the session record to it.
pub fn start_command(dir: &Path) -> Result<Command, String> {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    create_log(dir, secs)
}

/// As [`start_command`] for a log started at `unix_secs`. When that second's name is taken the
/// next free one, with `-2`, `-3`, ... before the extension, is used.
fn create_log(dir: &Path, unix_secs: u64) -> Result<Command, String> {
    fs::create_dir_all(dir)
        .map_err(|e| format!("cannot create log directory {}: {e}", dir.display()))?;
    let name = file_name(unix_secs);
    let stem = name.strip_suffix(".tlog").unwrap_or(&name);
    let mut attempt = 1u32;
    loop {
        let path: PathBuf = if attempt == 1 {
            dir.join(&name)
        } else {
            dir.join(format!("{stem}-{attempt}.tlog"))
        };
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(file) => return Ok(start_command_for(file, &path)),
            Err(e) if e.kind() == ErrorKind::AlreadyExists => attempt += 1,
            Err(e) => return Err(format!("cannot create log file {}: {e}", path.display())),
        }
    }
}

fn start_command_for(file: File, path: &Path) -> Command {
    Command::StartLog {
        sink: LogSink::new(BufWriter::new(file)),
        label: path.display().to_string(),
    }
}

fn request_id() -> egui::Id {
    egui::Id::unique("marv_gcs_log_request")
}

fn replaying_id() -> egui::Id {
    egui::Id::unique("marv_gcs_log_replaying")
}

fn error_id() -> egui::Id {
    egui::Id::unique("marv_gcs_log_error")
}

/// The Log view asks the app for a new log file.
pub fn request_new_log(ctx: &egui::Context) {
    ctx.data_mut(|d| d.insert_temp(request_id(), true));
}

/// True once per request.
pub fn take_request(ctx: &egui::Context) -> bool {
    ctx.data_mut(|d| d.remove_temp::<bool>(request_id())) == Some(true)
}

/// Why the app could not start recording; shown in the Log view. `None` clears it.
pub fn set_error(ctx: &egui::Context, error: Option<String>) {
    ctx.data_mut(|d| match error {
        Some(e) => {
            d.insert_temp(error_id(), e);
        }
        None => {
            d.remove_temp::<String>(error_id());
        }
    });
}

/// The app tells the Log view that the session replays a log file, so nothing can be recorded.
pub fn set_replaying(ctx: &egui::Context, replaying: bool) {
    ctx.data_mut(|d| d.insert_temp(replaying_id(), replaying));
}

pub fn replaying(ctx: &egui::Context) -> bool {
    ctx.data(|d| d.get_temp::<bool>(replaying_id()))
        .unwrap_or(false)
}

pub fn error(ctx: &egui::Context) -> Option<String> {
    ctx.data(|d| d.get_temp::<String>(error_id()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch() {
        assert_eq!(file_name(0), "1970-01-01_00-00-00Z.tlog");
    }

    #[test]
    fn leap_day() {
        // 2024-02-29 12:34:56 UTC
        assert_eq!(file_name(1_709_210_096), "2024-02-29_12-34-56Z.tlog");
        // 2000-02-29 00:00:00 UTC, a leap day in a century year
        assert_eq!(file_name(951_782_400), "2000-02-29_00-00-00Z.tlog");
    }

    #[test]
    fn known_date() {
        assert_eq!(file_name(1_794_446_625), "2026-11-12_01-23-45Z.tlog");
    }

    #[test]
    fn year_end_and_start() {
        assert_eq!(file_name(86_399), "1970-01-01_23-59-59Z.tlog");
        assert_eq!(file_name(1_767_225_599), "2025-12-31_23-59-59Z.tlog");
        assert_eq!(file_name(1_767_225_600), "2026-01-01_00-00-00Z.tlog");
    }

    #[test]
    fn creates_directory_and_file() {
        let dir = std::env::temp_dir().join(format!("marv-gcs-logfile-{}", std::process::id()));
        let sub = dir.join("a").join("b");
        let command = start_command(&sub).expect("log file is created");
        let Command::StartLog { label, .. } = command else {
            panic!("not StartLog")
        };
        assert!(label.starts_with(&sub.display().to_string()), "{label}");
        assert!(label.ends_with("Z.tlog"), "{label}");
        assert!(Path::new(&label).is_file());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn two_logs_in_the_same_second_get_different_files() {
        let dir =
            std::env::temp_dir().join(format!("marv-gcs-logfile-same-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        let mut labels = Vec::new();
        for _ in 0..3 {
            let Command::StartLog { label, .. } = create_log(&dir, 1_794_446_625).expect("created")
            else {
                panic!("not StartLog")
            };
            labels.push(label);
        }
        let names: Vec<String> = labels
            .iter()
            .map(|l| {
                Path::new(l)
                    .file_name()
                    .unwrap()
                    .to_string_lossy()
                    .into_owned()
            })
            .collect();
        assert_eq!(
            names,
            [
                "2026-11-12_01-23-45Z.tlog",
                "2026-11-12_01-23-45Z-2.tlog",
                "2026-11-12_01-23-45Z-3.tlog"
            ]
        );
        assert_eq!(fs::read_dir(&dir).unwrap().count(), 3);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn unusable_directory_is_an_error() {
        let dir = std::env::temp_dir().join(format!("marv-gcs-logfile-bad-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let blocker = dir.join("file");
        fs::write(&blocker, b"x").unwrap();
        let e = start_command(&blocker.join("sub")).expect_err("fails");
        assert!(e.contains("cannot create log directory"), "{e}");
        let _ = fs::remove_dir_all(&dir);
    }
}
