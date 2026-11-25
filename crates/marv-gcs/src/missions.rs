//! Named waypoint lists kept on disk, one file per list, in the QGC WPL 110 text format (the one
//! QGroundControl and Mission Planner read and write). Nothing here talks to the vehicle.
//!
//! Folder: `$XDG_DATA_HOME/marv-gcs/missions`, else `$HOME/.local/share/marv-gcs/missions` (`data_dir`).
//! File: `<name>.waypoints`. Row 0 is a home placeholder; the waypoints are rows 1 to n, frame 3
//! (`MAV_FRAME_GLOBAL_RELATIVE_ALT`), command 16 (`MAV_CMD_NAV_WAYPOINT`). The last row, when there is one, is the
//! return item: command 20 (`MAV_CMD_NAV_RETURN_TO_LAUNCH`), its altitude the return height. A file written before
//! the return item existed has no such row: its return height is `None`.

use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

/// First line of every file.
pub const HEADER: &str = "QGC WPL 110";
/// File name extension of a list.
pub const EXTENSION: &str = "waypoints";
/// Longest list name, in characters.
pub const NAME_MAX: usize = 64;

const FRAME_GLOBAL_RELATIVE_ALT: u32 = 3;
const CMD_NAV_WAYPOINT: u32 = 16;
const CMD_NAV_RETURN: u32 = 20;

/// A point to fly to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    /// Degrees, -90 to 90.
    pub lat: f64,
    /// Degrees, -180 to 180.
    pub lon: f64,
    /// Metres above the home position.
    pub alt_m: f32,
}

impl Waypoint {
    /// True when the coordinates are finite and on the globe.
    pub fn is_valid(&self) -> bool {
        self.lat.is_finite()
            && self.lon.is_finite()
            && self.alt_m.is_finite()
            && self.lat.abs() <= 90.0
            && self.lon.abs() <= 180.0
    }
}

/// Where the lists are kept: `$XDG_DATA_HOME/marv-gcs/missions`, else `$HOME/.local/share/marv-gcs/missions`;
/// `None` when neither is set to an absolute path.
pub fn data_dir(xdg_data_home: Option<&str>, home: Option<&str>) -> Option<PathBuf> {
    let absolute = |value: Option<&str>| value.map(PathBuf::from).filter(|path| path.is_absolute());
    let base = absolute(xdg_data_home)
        .or_else(|| absolute(home).map(|home| home.join(".local").join("share")))?;
    Some(base.join("marv-gcs").join("missions"))
}

/// `data_dir` of the process environment.
pub fn environment_dir() -> Option<PathBuf> {
    data_dir(
        std::env::var("XDG_DATA_HOME").ok().as_deref(),
        std::env::var("HOME").ok().as_deref(),
    )
}

/// The text of a list in QGC WPL 110: the waypoints, then the return item when there is a return height.
pub fn to_wpl(waypoints: &[Waypoint], return_height: Option<f32>) -> String {
    let mut text = format!("{HEADER}\n");
    let (home_lat, home_lon) = waypoints.first().map_or((0.0, 0.0), |w| (w.lat, w.lon));
    let _ = writeln!(
        text,
        "0\t1\t0\t{CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{home_lat:.8}\t{home_lon:.8}\t0\t1"
    );
    for (i, w) in waypoints.iter().enumerate() {
        let _ = writeln!(
            text,
            "{}\t0\t{FRAME_GLOBAL_RELATIVE_ALT}\t{CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{:.8}\t{:.8}\t{}\t1",
            i + 1,
            w.lat,
            w.lon,
            w.alt_m
        );
    }
    if let Some(height) = return_height {
        let _ = writeln!(
            text,
            "{}\t0\t{FRAME_GLOBAL_RELATIVE_ALT}\t{CMD_NAV_RETURN}\t0\t0\t0\t0\t0\t0\t{height}\t1",
            waypoints.len() + 1
        );
    }
    text
}

/// The waypoints and the return height of a QGC WPL 110 text: row 0 (home) is skipped; every other row must be a
/// `MAV_CMD_NAV_WAYPOINT` in `MAV_FRAME_GLOBAL_RELATIVE_ALT` on the globe, except a last row of command 20, whose
/// altitude is the return height (its latitude and longitude are not read). `None`: the text has no such row.
pub fn parse_wpl(text: &str) -> Result<(Vec<Waypoint>, Option<f32>), String> {
    let mut lines = text.lines().enumerate();
    match lines.next() {
        Some((_, first)) if first.trim() == HEADER => {}
        _ => {
            return Err(format!(
                "not a waypoint file: the first line must be \"{HEADER}\""
            ));
        }
    }
    let mut waypoints = Vec::new();
    let mut return_height: Option<(usize, f32)> = None;
    for (index, line) in lines {
        if line.trim().is_empty() {
            continue;
        }
        let line_no = index + 1;
        let fields: Vec<&str> = line.split_whitespace().collect();
        if fields.len() != 12 {
            return Err(format!(
                "line {line_no}: expected 12 fields, found {}",
                fields.len()
            ));
        }
        let int = |i: usize, what: &str| {
            fields[i]
                .parse::<u32>()
                .map_err(|_| format!("line {line_no}: {what} \"{}\" is not a number", fields[i]))
        };
        let seq = int(0, "sequence")?;
        if seq == 0 {
            continue;
        }
        let frame = int(2, "frame")?;
        let command = int(3, "command")?;
        if let Some((at, _)) = return_height {
            return Err(format!(
                "line {line_no}: the return row (command {CMD_NAV_RETURN}, line {at}) must be the last row"
            ));
        }
        if command != CMD_NAV_WAYPOINT && command != CMD_NAV_RETURN {
            return Err(format!(
                "line {line_no}: command {command} is not supported (only {CMD_NAV_WAYPOINT}, NAV_WAYPOINT, and {CMD_NAV_RETURN}, the return height, last)"
            ));
        }
        if frame != FRAME_GLOBAL_RELATIVE_ALT {
            return Err(format!(
                "line {line_no}: frame {frame} is not supported (only {FRAME_GLOBAL_RELATIVE_ALT}, altitude above home)"
            ));
        }
        let float = |i: usize, what: &str| {
            fields[i]
                .parse::<f64>()
                .map_err(|_| format!("line {line_no}: {what} \"{}\" is not a number", fields[i]))
        };
        if command == CMD_NAV_RETURN {
            let height = float(10, "altitude")? as f32;
            if !height.is_finite() {
                return Err(format!(
                    "line {line_no}: the return height \"{}\" is not finite",
                    fields[10]
                ));
            }
            return_height = Some((line_no, height));
            continue;
        }
        let waypoint = Waypoint {
            lat: float(8, "latitude")?,
            lon: float(9, "longitude")?,
            alt_m: float(10, "altitude")? as f32,
        };
        if !waypoint.is_valid() {
            return Err(format!(
                "line {line_no}: {}, {} is not on the globe",
                fields[8], fields[9]
            ));
        }
        waypoints.push(waypoint);
    }
    Ok((waypoints, return_height.map(|(_, height)| height)))
}

/// Why a name cannot be a list's name, or `None` when it can.
pub fn name_problem(name: &str) -> Option<&'static str> {
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

/// A folder of lists.
#[derive(Debug, Clone)]
pub struct Store {
    dir: PathBuf,
}

impl Store {
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self { dir: dir.into() }
    }

    /// The store in the environment's data folder, if there is one.
    pub fn from_environment() -> Option<Self> {
        environment_dir().map(Self::new)
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    fn path(&self, name: &str) -> Result<PathBuf, String> {
        match name_problem(name) {
            Some(problem) => Err(problem.to_string()),
            None => Ok(self.dir.join(format!("{name}.{EXTENSION}"))),
        }
    }

    fn exists(&self, name: &str) -> bool {
        self.path(name).is_ok_and(|path| path.exists())
    }

    /// The names of the lists, sorted. An absent folder has none.
    pub fn names(&self) -> Result<Vec<String>, String> {
        let entries = match fs::read_dir(&self.dir) {
            Ok(entries) => entries,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => return Err(format!("cannot read {}: {e}", self.dir.display())),
        };
        let mut names: Vec<String> = entries
            .filter_map(Result::ok)
            .filter(|entry| entry.path().is_file())
            .filter_map(|entry| {
                let path = entry.path();
                if path.extension()? != EXTENSION {
                    return None;
                }
                let name = path.file_stem()?.to_str()?.to_string();
                name_problem(&name).is_none().then_some(name)
            })
            .collect();
        names.sort();
        Ok(names)
    }

    /// The waypoints of a list and its return height (`None` for a list saved without one).
    pub fn load(&self, name: &str) -> Result<(Vec<Waypoint>, Option<f32>), String> {
        let path = self.path(name)?;
        let text = fs::read_to_string(&path)
            .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        parse_wpl(&text).map_err(|e| format!("{name}: {e}"))
    }

    /// Writes a temporary file beside the list, then renames it over the list.
    pub fn save(
        &self,
        name: &str,
        waypoints: &[Waypoint],
        return_height: Option<f32>,
    ) -> Result<(), String> {
        let path = self.path(name)?;
        fs::create_dir_all(&self.dir)
            .map_err(|e| format!("cannot create {}: {e}", self.dir.display()))?;
        let temporary = self.dir.join(format!(".{name}.{EXTENSION}.tmp"));
        fs::write(&temporary, to_wpl(waypoints, return_height))
            .map_err(|e| format!("cannot write {}: {e}", temporary.display()))?;
        fs::rename(&temporary, &path).map_err(|e| {
            let _ = fs::remove_file(&temporary);
            format!("cannot write {}: {e}", path.display())
        })
    }

    /// A new empty list; an error when the name is not valid or is taken.
    pub fn create(&self, name: &str) -> Result<(), String> {
        self.path(name)?;
        if self.exists(name) {
            return Err(format!("a list named \"{name}\" already exists"));
        }
        self.save(name, &[], None)
    }

    pub fn rename(&self, from: &str, to: &str) -> Result<(), String> {
        let source = self.path(from)?;
        let target = self.path(to)?;
        if from == to {
            return Ok(());
        }
        if target.exists() {
            return Err(format!("a list named \"{to}\" already exists"));
        }
        fs::rename(&source, &target).map_err(|e| format!("cannot rename \"{from}\": {e}"))
    }

    /// Copies `from` under `to`; an error when `to` is taken.
    pub fn duplicate(&self, from: &str, to: &str) -> Result<(), String> {
        let (waypoints, return_height) = self.load(from)?;
        if self.exists(to) {
            return Err(format!("a list named \"{to}\" already exists"));
        }
        self.path(to)?;
        self.save(to, &waypoints, return_height)
    }

    pub fn delete(&self, name: &str) -> Result<(), String> {
        let path = self.path(name)?;
        fs::remove_file(&path).map_err(|e| format!("cannot delete \"{name}\": {e}"))
    }

    /// `base`, or `base 2`, `base 3`, ... : the first that is not taken.
    pub fn unique_name(&self, base: &str) -> String {
        if !self.exists(base) {
            return base.to_string();
        }
        (2..)
            .map(|n| format!("{base} {n}"))
            .find(|candidate| !self.exists(candidate))
            .unwrap_or_else(|| base.to_string())
    }
}

/// Great-circle distance in metres between two points (haversine, mean Earth radius).
pub fn distance_m(a: &Waypoint, b: &Waypoint) -> f64 {
    const EARTH_RADIUS_M: f64 = 6_371_008.8;
    let (lat1, lat2) = (a.lat.to_radians(), b.lat.to_radians());
    let dlat = lat2 - lat1;
    let dlon = (b.lon - a.lon).to_radians();
    let h = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().min(1.0).asin()
}

/// Length in metres of the path through the waypoints in order (horizontal only).
pub fn path_length_m(waypoints: &[Waypoint]) -> f64 {
    waypoints
        .windows(2)
        .fold(0.0, |sum, p| sum + distance_m(&p[0], &p[1]))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wp(lat: f64, lon: f64, alt_m: f32) -> Waypoint {
        Waypoint { lat, lon, alt_m }
    }

    fn temp_store(tag: &str) -> Store {
        let dir = std::env::temp_dir().join(format!(
            "marv-gcs-missions-{tag}-{}-{:?}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        Store::new(dir)
    }

    #[test]
    fn the_folder_is_xdg_data_home_or_home_local_share() {
        let expected = |base: &str| PathBuf::from(base).join("marv-gcs").join("missions");
        assert_eq!(
            data_dir(Some("/x/data"), Some("/home/u")),
            Some(expected("/x/data"))
        );
        assert_eq!(
            data_dir(None, Some("/home/u")),
            Some(expected("/home/u/.local/share"))
        );
        assert_eq!(
            data_dir(Some(""), Some("/home/u")),
            Some(expected("/home/u/.local/share"))
        );
        assert_eq!(
            data_dir(Some("rel"), Some("/home/u")),
            Some(expected("/home/u/.local/share"))
        );
        assert_eq!(data_dir(None, None), None);
    }

    #[test]
    fn a_list_round_trips_through_wpl() {
        let list = vec![
            wp(45.123_456_78, 9.000_000_01, 30.0),
            wp(-33.5, -70.25, 12.5),
        ];
        let text = to_wpl(&list, None);
        assert!(text.starts_with("QGC WPL 110\n"));
        let rows: Vec<&str> = text.lines().collect();
        assert_eq!(rows.len(), 4);
        assert_eq!(
            rows[1],
            "0\t1\t0\t16\t0\t0\t0\t0\t45.12345678\t9.00000001\t0\t1"
        );
        assert_eq!(
            rows[2],
            "1\t0\t3\t16\t0\t0\t0\t0\t45.12345678\t9.00000001\t30\t1"
        );
        assert_eq!(parse_wpl(&text).unwrap(), (list, None));
        assert_eq!(parse_wpl(&to_wpl(&[], None)).unwrap(), (vec![], None));
        assert_eq!(
            to_wpl(&[], None).lines().nth(1).unwrap(),
            "0\t1\t0\t16\t0\t0\t0\t0\t0.00000000\t0.00000000\t0\t1"
        );
    }

    #[test]
    fn the_return_height_is_the_last_row_with_command_20_and_round_trips() {
        let list = vec![wp(45.0, 9.0, 30.0), wp(-33.5, -70.25, 12.5)];
        let text = to_wpl(&list, Some(25.5));
        let rows: Vec<&str> = text.lines().collect();
        assert_eq!(rows.len(), 5);
        assert_eq!(rows[4], "3\t0\t3\t20\t0\t0\t0\t0\t0\t0\t25.5\t1");
        assert_eq!(parse_wpl(&text).unwrap(), (list, Some(25.5)));
        // A return height with no waypoint at all.
        let text = to_wpl(&[], Some(7.0));
        assert_eq!(
            text.lines().nth(2),
            Some("1\t0\t3\t20\t0\t0\t0\t0\t0\t0\t7\t1")
        );
        assert_eq!(parse_wpl(&text).unwrap(), (vec![], Some(7.0)));
    }

    #[test]
    fn a_return_row_that_is_not_last_or_not_finite_is_an_error() {
        let head = "QGC WPL 110\n";
        let ret = "1\t0\t3\t20\t0\t0\t0\t0\t0\t0\t20\t1\n";
        let wp = "2\t0\t3\t16\t0\t0\t0\t0\t1\t2\t10\t1\n";
        assert!(
            parse_wpl(&format!("{head}{ret}{wp}"))
                .unwrap_err()
                .contains("must be the last row")
        );
        assert!(
            parse_wpl(&format!("{head}1\t0\t3\t20\t0\t0\t0\t0\t0\t0\tNaN\t1\n"))
                .unwrap_err()
                .contains("not finite")
        );
    }

    #[test]
    fn the_home_row_is_skipped_whatever_it_holds() {
        let text = "QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t10\t20\t500\t1\n1\t0\t3\t16\t0\t0\t0\t0\t1\t2\t3\t1\n";
        assert_eq!(parse_wpl(text).unwrap(), (vec![wp(1.0, 2.0, 3.0)], None));
    }

    #[test]
    fn a_bad_header_or_row_is_a_clear_error() {
        assert!(parse_wpl("").unwrap_err().contains("QGC WPL 110"));
        assert!(
            parse_wpl("QGC WPL 120\n")
                .unwrap_err()
                .contains("QGC WPL 110")
        );
        let head = "QGC WPL 110\n";
        let row = |frame: u32, cmd: u32, lat: &str, lon: &str| {
            format!("{head}1\t0\t{frame}\t{cmd}\t0\t0\t0\t0\t{lat}\t{lon}\t10\t1\n")
        };
        assert!(parse_wpl(&row(3, 16, "1", "2")).is_ok());
        assert!(
            parse_wpl(&row(3, 22, "1", "2"))
                .unwrap_err()
                .contains("command 22")
        );
        assert!(
            parse_wpl(&row(0, 16, "1", "2"))
                .unwrap_err()
                .contains("frame 0")
        );
        assert!(
            parse_wpl(&row(3, 16, "91", "2"))
                .unwrap_err()
                .contains("globe")
        );
        assert!(
            parse_wpl(&row(3, 16, "1", "-181"))
                .unwrap_err()
                .contains("globe")
        );
        assert!(
            parse_wpl(&row(3, 16, "x", "2"))
                .unwrap_err()
                .contains("latitude")
        );
        assert!(
            parse_wpl(&row(3, 16, "NaN", "2"))
                .unwrap_err()
                .contains("globe")
        );
        let short = format!("{head}1\t0\t3\t16\n");
        assert!(
            parse_wpl(&short)
                .unwrap_err()
                .contains("line 2: expected 12 fields")
        );
    }

    #[test]
    fn names_are_validated() {
        for ok in ["mission", "Field 2", "a.b", "é", &"x".repeat(NAME_MAX)] {
            assert_eq!(name_problem(ok), None, "{ok}");
        }
        for bad in [
            "",
            "  ",
            ".hidden",
            "a/b",
            "a\\b",
            " pad",
            "pad ",
            "a\nb",
            &"x".repeat(NAME_MAX + 1),
        ] {
            assert!(name_problem(bad).is_some(), "{bad:?}");
        }
        let store = temp_store("names");
        assert!(store.create("../x").is_err());
        assert!(store.save("a/b", &[], None).is_err());
        assert!(store.load(".x").is_err());
    }

    #[test]
    fn the_store_creates_lists_renames_duplicates_and_deletes() {
        let store = temp_store("crud");
        assert_eq!(store.names().unwrap(), Vec::<String>::new());
        store.create("beta").unwrap();
        store.create("alpha").unwrap();
        assert!(
            store
                .create("alpha")
                .unwrap_err()
                .contains("already exists")
        );
        assert_eq!(store.names().unwrap(), ["alpha", "beta"]);
        assert_eq!(store.load("alpha").unwrap(), (vec![], None));

        let list = vec![wp(1.0, 2.0, 3.0), wp(4.0, 5.0, 6.0)];
        store.save("alpha", &list, Some(22.0)).unwrap();
        assert_eq!(store.load("alpha").unwrap(), (list.clone(), Some(22.0)));
        assert!(
            !store.dir().join(".alpha.waypoints.tmp").exists(),
            "no temporary file is left"
        );

        store.duplicate("alpha", "gamma").unwrap();
        assert_eq!(store.load("gamma").unwrap(), (list.clone(), Some(22.0)));
        assert!(store.duplicate("alpha", "beta").is_err());
        assert!(store.duplicate("missing", "delta").is_err());

        store.rename("gamma", "omega").unwrap();
        assert_eq!(store.names().unwrap(), ["alpha", "beta", "omega"]);
        assert!(
            store
                .rename("omega", "alpha")
                .unwrap_err()
                .contains("already exists")
        );
        assert!(store.rename("nope", "x").is_err());
        assert_eq!(store.load("omega").unwrap(), (list, Some(22.0)));

        store.delete("omega").unwrap();
        assert!(store.delete("omega").is_err());
        assert_eq!(store.names().unwrap(), ["alpha", "beta"]);
        assert_eq!(store.unique_name("alpha"), "alpha 2");
        assert_eq!(store.unique_name("new"), "new");
        std::fs::write(store.dir().join("notes.txt"), "x").unwrap();
        assert_eq!(
            store.names().unwrap(),
            ["alpha", "beta"],
            "other files are not lists"
        );
        let _ = std::fs::remove_dir_all(store.dir());
    }

    #[test]
    fn a_corrupt_file_is_an_error_naming_the_list() {
        let store = temp_store("corrupt");
        std::fs::create_dir_all(store.dir()).unwrap();
        std::fs::write(store.dir().join("bad.waypoints"), "hello\n").unwrap();
        assert!(store.load("bad").unwrap_err().starts_with("bad: "));
        let _ = std::fs::remove_dir_all(store.dir());
    }

    #[test]
    fn distance_and_path_length_are_great_circle_metres() {
        // One degree of latitude is about 111.2 km.
        let d = distance_m(&wp(0.0, 0.0, 0.0), &wp(1.0, 0.0, 0.0));
        assert!((d - 111_195.0).abs() < 100.0, "{d}");
        let d = distance_m(&wp(0.0, 0.0, 0.0), &wp(0.0, 1.0, 0.0));
        assert!((d - 111_195.0).abs() < 100.0, "{d}");
        assert_eq!(distance_m(&wp(10.0, 10.0, 0.0), &wp(10.0, 10.0, 99.0)), 0.0);
        let path = [wp(0.0, 0.0, 0.0), wp(1.0, 0.0, 0.0), wp(1.0, 1.0, 0.0)];
        let expected = distance_m(&path[0], &path[1]) + distance_m(&path[1], &path[2]);
        assert!((path_length_m(&path) - expected).abs() < 1e-9);
        assert_eq!(path_length_m(&path[..1]), 0.0);
        assert_eq!(path_length_m(&[]), 0.0);
    }
}
