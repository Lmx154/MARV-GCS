//! `marv-cli log survey`: the survey presses of a DataFlash log of the MARV firmware as a saved waypoint list.
//!
//! A press is an RC channel (default 9) going from low to high in `RCIN` and staying high for at least
//! `MIN_HIGH_SAMPLES` samples. Its point is the median latitude and longitude of the receiver's own fixes (`GPS`
//! records with a 3D fix or better) within the press, widened by `WIDEN_US` each side. The list goes into the
//! window's store (`$XDG_DATA_HOME/marv-gcs/missions`, else `$HOME/.local/share/marv-gcs/missions`) as
//! `<name>.waypoints`, in the format `crates/marv-gcs/src/missions.rs` writes (`waypoints::to_wpl`).
//! Nothing printed contains a coordinate; the coordinates go only into the file.

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

use gcs_dflog::Reader;
use gcs_mavlink::services::missions::{MissionPoint, degrees_to_e7};

use crate::args::SurveyCommand;
use crate::exit;
use crate::waypoints;

/// A press needs the channel high for this many consecutive RCIN samples (a one-sample blip is not a press).
pub const MIN_HIGH_SAMPLES: usize = 2;
/// The channel's own range over the log must be at least this many raw units, else it never moved.
pub const MIN_RANGE: f64 = 400.0;
/// The fixes of a press are those within this long before the first and after the last high sample.
pub const WIDEN_US: u64 = 300_000;
/// A mission holds this many waypoints.
pub const MAX_POINTS: usize = 16;
/// `GPS.Status` of a 3D fix; higher is better.
pub const FIX_3D: f64 = 3.0;
/// The longest list name, in characters (the window's `NAME_MAX`).
const NAME_MAX: usize = 64;

struct Fix {
    t_us: u64,
    lat: f64,
    lon: f64,
    sats: f64,
    hdop: f64,
}

struct Press {
    first_us: u64,
    last_us: u64,
}

struct Point {
    lat: f64,
    lon: f64,
}

/// What the log gave: the channel's samples, the fixes with a 3D fix or better, and a GPS date.
struct Data {
    rcin: Vec<(u64, f64)>,
    fixes: Vec<Fix>,
    /// Unix seconds of the first 3D fix that has a GPS week.
    gps_unix_s: Option<i64>,
}

fn read(path: &Path, channel: u8) -> Result<Data, String> {
    let mut reader = Reader::open(path).map_err(|e| e.to_string())?;
    let column = format!("C{channel}");
    let mut data = Data {
        rcin: Vec::new(),
        fixes: Vec::new(),
        gps_unix_s: None,
    };
    let mut rcin_seen = false;
    loop {
        let rec = match reader.next_record() {
            Ok(Some(rec)) => rec,
            Ok(None) => break,
            Err(e) => return Err(e.to_string()),
        };
        let Some(t) = rec.time_us() else { continue };
        match rec.type_name() {
            "RCIN" => {
                rcin_seen = true;
                let Some(v) = rec.number_of(&column) else {
                    return Err(format!("the RCIN record has no column {column}"));
                };
                data.rcin.push((t, v));
            }
            "GPS" if rec.number_of("I").is_none_or(|i| i == 0.0) => {
                let num = |name: &str| rec.number_of(name).unwrap_or(f64::NAN);
                if num("Status") < FIX_3D {
                    continue;
                }
                let (lat, lon) = (num("Lat"), num("Lng"));
                if !(lat.is_finite() && lon.is_finite()) {
                    continue;
                }
                if data.gps_unix_s.is_none() {
                    let (week, ms) = (num("GWk"), num("GMS"));
                    if week.is_finite() && week > 0.0 && ms.is_finite() {
                        data.gps_unix_s =
                            Some(315_964_800 + week as i64 * 604_800 + (ms / 1000.0) as i64);
                    }
                }
                data.fixes.push(Fix {
                    t_us: t,
                    lat,
                    lon,
                    sats: num("NSats"),
                    hdop: num("HDop"),
                });
            }
            _ => {}
        }
    }
    if !rcin_seen {
        return Err("the log has no RCIN records".to_string());
    }
    Ok(data)
}

/// The presses of the samples, or `None` when the channel's range is under `MIN_RANGE`.
fn presses(samples: &[(u64, f64)]) -> Option<Vec<Press>> {
    let min = samples.iter().map(|s| s.1).fold(f64::INFINITY, f64::min);
    let max = samples
        .iter()
        .map(|s| s.1)
        .fold(f64::NEG_INFINITY, f64::max);
    if samples.is_empty() || max - min < MIN_RANGE {
        return None;
    }
    let mid = (min + max) / 2.0;
    let mut found = Vec::new();
    let mut run: Vec<u64> = Vec::new();
    let mut close = |run: &mut Vec<u64>| {
        if run.len() >= MIN_HIGH_SAMPLES {
            found.push(Press {
                first_us: run[0],
                last_us: run[run.len() - 1],
            });
        }
        run.clear();
    };
    for &(t, v) in samples {
        if v > mid {
            run.push(t);
        } else {
            close(&mut run);
        }
    }
    close(&mut run);
    Some(found)
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    let n = values.len();
    if n % 2 == 1 {
        values[n / 2]
    } else {
        (values[n / 2 - 1] + values[n / 2]) / 2.0
    }
}

/// Great-circle distance in metres (haversine, mean Earth radius), as the window's `distance_m`.
fn distance_m(a: &Point, b: &Point) -> f64 {
    const EARTH_RADIUS_M: f64 = 6_371_008.8;
    let (lat1, lat2) = (a.lat.to_radians(), b.lat.to_radians());
    let dlat = lat2 - lat1;
    let dlon = (b.lon - a.lon).to_radians();
    let h = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().min(1.0).asin()
}

/// The window's rules for a list name (`missions::name_problem`).
fn name_problem(name: &str) -> Option<&'static str> {
    if name.trim().is_empty() {
        Some("the name is empty")
    } else if name != name.trim() {
        Some("the name starts or ends with a space")
    } else if name.chars().count() > NAME_MAX {
        Some("the name is longer than 64 characters")
    } else if name.starts_with('.') {
        Some("the name cannot start with a dot")
    } else if name
        .chars()
        .any(|c| c == '/' || c == '\\' || c.is_control())
    {
        Some("the name cannot contain / or \\ or control characters")
    } else {
        None
    }
}

/// The window's folder of lists (`missions::data_dir` of the environment).
fn store_dir() -> Option<PathBuf> {
    let absolute = |name: &str| {
        std::env::var(name)
            .ok()
            .map(PathBuf::from)
            .filter(|path| path.is_absolute())
    };
    let base = absolute("XDG_DATA_HOME")
        .or_else(|| absolute("HOME").map(|home| home.join(".local").join("share")))?;
    Some(base.join("marv-gcs").join("missions"))
}

/// YYYYMMDD of a Unix time (civil date from days, proleptic Gregorian).
fn yyyymmdd(unix_s: i64) -> String {
    let z = unix_s.div_euclid(86_400) + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}{month:02}{day:02}")
}

pub fn run(command: &SurveyCommand, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let fail = |err: &mut dyn Write, message: String| {
        let _ = writeln!(err, "marv-cli: log survey: {message}");
        exit::FAILED
    };
    let data = match read(&command.file, command.channel) {
        Ok(d) => d,
        Err(e) => {
            return fail(err, format!("cannot read {}: {e}", command.file.display()));
        }
    };
    let channel = command.channel;
    let Some(found) = presses(&data.rcin) else {
        let _ = writeln!(out, "channel {channel} never moved: no survey points");
        return exit::OK;
    };
    if found.is_empty() {
        let _ = writeln!(out, "channel {channel} was never pressed: no survey points");
        return exit::OK;
    }
    let mut points: Vec<(MissionPoint, Point)> = Vec::new();
    let mut left_out = 0usize;
    for (index, press) in found.iter().enumerate() {
        let number = index + 1;
        let from = press.first_us.saturating_sub(WIDEN_US);
        let to = press.last_us + WIDEN_US;
        let used: Vec<&Fix> = data
            .fixes
            .iter()
            .filter(|f| f.t_us >= from && f.t_us <= to)
            .collect();
        if used.is_empty() {
            let _ = writeln!(out, "press {number}: no 3D fix");
            continue;
        }
        let point = Point {
            lat: median(used.iter().map(|f| f.lat).collect()),
            lon: median(used.iter().map(|f| f.lon).collect()),
        };
        let (Some(lat_e7), Some(lon_e7)) = (degrees_to_e7(point.lat), degrees_to_e7(point.lon))
        else {
            let _ = writeln!(out, "press {number}: no 3D fix");
            continue;
        };
        let mut line = format!(
            "press {number}: t {:.1} s, held {:.1} s, {} fixes",
            press.first_us as f64 / 1e6,
            (press.last_us - press.first_us) as f64 / 1e6,
            used.len()
        );
        let sats = used.iter().map(|f| f.sats).fold(f64::INFINITY, f64::min);
        if sats.is_finite() {
            line.push_str(&format!(", {sats:.0} satellites at least"));
        }
        let hdop = used
            .iter()
            .map(|f| f.hdop)
            .fold(f64::NEG_INFINITY, f64::max);
        if hdop.is_finite() {
            line.push_str(&format!(", hdop {hdop:.2} at most"));
        }
        match points.last() {
            Some((_, previous)) => {
                line.push_str(&format!(
                    ", {:.1} m from the previous point",
                    distance_m(previous, &point)
                ));
            }
            None => line.push_str(", first point"),
        }
        if points.len() >= MAX_POINTS {
            left_out += 1;
            let _ = writeln!(out, "{line} (left out: the list is full)");
            continue;
        }
        let _ = writeln!(out, "{line}");
        points.push((
            MissionPoint {
                lat_e7,
                lon_e7,
                z: command.alt_m as f32,
            },
            point,
        ));
    }
    if points.is_empty() {
        let _ = writeln!(out, "no press has a 3D fix: no survey points");
        return exit::OK;
    }
    let name = match &command.name {
        Some(name) => name.clone(),
        None => {
            let unix_s = data.gps_unix_s.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs() as i64)
            });
            let stem = command
                .file
                .file_stem()
                .map_or_else(|| "log".to_string(), |s| s.to_string_lossy().into_owned());
            format!("survey-{}-{stem}", yyyymmdd(unix_s))
        }
    };
    if let Some(problem) = name_problem(&name) {
        return fail(err, format!("the list name is not valid: {problem}"));
    }
    let Some(dir) = store_dir() else {
        return fail(
            err,
            "no folder for the lists: neither XDG_DATA_HOME nor HOME is an absolute path"
                .to_string(),
        );
    };
    let path = dir.join(format!("{name}.waypoints"));
    if let Err(e) = fs::create_dir_all(&dir) {
        return fail(err, format!("cannot create {}: {e}", dir.display()));
    }
    let mut options = OpenOptions::new();
    options.write(true);
    if command.force {
        options.create(true).truncate(true);
    } else {
        options.create_new(true);
    }
    let text = waypoints::to_wpl(&points.iter().map(|p| p.0).collect::<Vec<_>>());
    let written = options
        .open(&path)
        .map_err(|e| {
            if e.kind() == std::io::ErrorKind::AlreadyExists {
                format!("the list \"{name}\" already exists (--force replaces it)")
            } else {
                format!("cannot create {}: {e}", path.display())
            }
        })
        .and_then(|mut f| {
            f.write_all(text.as_bytes())
                .map_err(|e| format!("cannot write {}: {e}", path.display()))
        });
    if let Err(e) = written {
        return fail(err, e);
    }
    if left_out > 0 {
        let _ = writeln!(
            out,
            "{left_out} more presses left out: a mission holds {MAX_POINTS} waypoints"
        );
    }
    let _ = writeln!(
        out,
        "list \"{name}\": {} waypoints at {} m above take-off, saved in {}",
        points.len(),
        command.alt_m,
        dir.display()
    );
    exit::OK
}
