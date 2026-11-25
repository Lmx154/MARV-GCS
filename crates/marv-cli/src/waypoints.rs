//! The waypoint file the window's map tab keeps: QGC WPL 110 text (`crates/marv-gcs/src/missions.rs`, which this
//! reads and writes the same way: that crate is the window's and depends on a GUI, so the format is repeated here).
//! Row 0 is a home placeholder and is never sent; the waypoints are rows 1 to n, frame 3, command 16.

use std::fmt::Write as _;

use gcs_mavlink::services::missions::{MissionPoint, degrees_to_e7};

const HEADER: &str = "QGC WPL 110";
const FRAME_GLOBAL_RELATIVE_ALT: u32 = 3;
const CMD_NAV_WAYPOINT: u32 = 16;

const CMD_NAV_RETURN: u32 = 20;

/// The waypoints and the return height of a QGC WPL 110 text, ready to send: row 0 is skipped; every other row
/// must be a `MAV_CMD_NAV_WAYPOINT` in `MAV_FRAME_GLOBAL_RELATIVE_ALT` on the globe, except a last row with command
/// 20 (the return item), whose altitude is the return height (`None` when the file has no such row).
pub fn parse(text: &str) -> Result<(Vec<MissionPoint>, Option<f32>), String> {
    let mut lines = text.lines().enumerate();
    match lines.next() {
        Some((_, first)) if first.trim() == HEADER => {}
        _ => {
            return Err(format!(
                "not a waypoint file: the first line must be \"{HEADER}\""
            ));
        }
    }
    let mut points = Vec::new();
    let mut return_height = None;
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
        if int(0, "sequence")? == 0 {
            continue;
        }
        let frame = int(2, "frame")?;
        let command = int(3, "command")?;
        if return_height.is_some() {
            return Err(format!(
                "line {line_no}: the return row (command {CMD_NAV_RETURN}) must be the last row"
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
            return_height = Some(height);
            continue;
        }
        let (lat, lon) = (float(8, "latitude")?, float(9, "longitude")?);
        let alt = float(10, "altitude")? as f32;
        let on_globe = lat.is_finite()
            && lon.is_finite()
            && alt.is_finite()
            && lat.abs() <= 90.0
            && lon.abs() <= 180.0;
        let (Some(lat_e7), Some(lon_e7)) = (degrees_to_e7(lat), degrees_to_e7(lon)) else {
            return Err(format!(
                "line {line_no}: {}, {} is not on the globe",
                fields[8], fields[9]
            ));
        };
        if !on_globe {
            return Err(format!(
                "line {line_no}: {}, {} is not on the globe",
                fields[8], fields[9]
            ));
        }
        points.push(MissionPoint {
            lat_e7,
            lon_e7,
            z: alt,
        });
    }
    Ok((points, return_height))
}

/// Degrees of a wire value of degrees x 10^7, to 7 decimals.
pub fn degrees(e7: i32) -> String {
    format!("{:.7}", f64::from(e7) / 1e7)
}

/// The text of a list in QGC WPL 110, as the map tab writes it: row 0 is a home placeholder at the first
/// waypoint's position (0, 0 for an empty list), row i the waypoint `seq` i - 1.
pub fn to_wpl(points: &[MissionPoint], return_height: Option<f32>) -> String {
    let mut text = format!("{HEADER}\n");
    let (home_lat, home_lon) = points.first().map_or((0.0, 0.0), |p| {
        (f64::from(p.lat_e7) / 1e7, f64::from(p.lon_e7) / 1e7)
    });
    let _ = writeln!(
        text,
        "0\t1\t0\t{CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{home_lat:.8}\t{home_lon:.8}\t0\t1"
    );
    for (i, p) in points.iter().enumerate() {
        let _ = writeln!(
            text,
            "{}\t0\t{FRAME_GLOBAL_RELATIVE_ALT}\t{CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{:.8}\t{:.8}\t{}\t1",
            i + 1,
            f64::from(p.lat_e7) / 1e7,
            f64::from(p.lon_e7) / 1e7,
            p.z
        );
    }
    if let Some(height) = return_height {
        let _ = writeln!(
            text,
            "{}\t0\t{FRAME_GLOBAL_RELATIVE_ALT}\t{CMD_NAV_RETURN}\t0\t0\t0\t0\t0\t0\t{height}\t1",
            points.len() + 1
        );
    }
    text
}
