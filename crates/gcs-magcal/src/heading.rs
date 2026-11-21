//! The heading check of the decision's section 5: after a calibration the user walks or flies a straight line and
//! the station compares the tilt-compensated compass heading plus the model's declination with the GPS course. It
//! informs and never rejects. A function over recorded data, so it runs on a log as well.

use crate::Vec3;

/// Smallest ground speed of a GPS record that is used, m/s. Proposal shown to the owner 2026-11-20 (decision
/// section 5 and "Owner's answers").
pub const MIN_SPEED_MPS: f64 = 1.0;
/// Fewest usable GPS records. Proposal shown to the owner 2026-11-20.
pub const MIN_RECORDS: usize = 10;
/// The median difference that reads "within", degrees. Proposal shown to the owner 2026-11-20 (the navigation
/// requirement, decision section 5).
pub const TOLERANCE_DEG: f64 = 10.0;

/// One HIGHRES_IMU sample as recorded: the field is the published one (the calibration applied).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// Vehicle time, the same base as [`GpsRecord::time_usec`] (both messages carry the vehicle's `time_usec`).
    pub time_usec: u64,
    /// Accelerometer, m/s^2, body frame FRD (reads about (0, 0, -g) level at rest).
    pub accel: [f32; 3],
    /// Magnetometer, calibrated, body frame; any unit (only its direction is used).
    pub mag: [f32; 3],
}

/// One GPS_RAW_INT record.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsRecord {
    pub time_usec: u64,
    /// Course over ground, degrees clockwise from true north.
    pub course_deg: f64,
    pub speed_mps: f64,
}

/// The result of [`check`].
#[derive(Clone, Debug, PartialEq)]
pub enum HeadingCheck {
    /// Fewer than [`MIN_RECORDS`] usable records.
    NotEnough { records: usize },
    Measured {
        records: usize,
        /// Median of (compass heading + declination - GPS course), each wrapped to +-180 deg.
        median_deg: f64,
        /// Median absolute deviation about the median, degrees.
        spread_deg: f64,
        /// |median| <= [`TOLERANCE_DEG`].
        within: bool,
    },
}

impl HeadingCheck {
    /// The text of the decision's section 5.
    pub fn message(&self) -> String {
        match self {
            HeadingCheck::NotEnough { records } => {
                format!("not enough straight-line movement yet ({records} of {MIN_RECORDS})")
            }
            HeadingCheck::Measured {
                median_deg, within, ..
            } => {
                if *within {
                    format!("within {TOLERANCE_DEG:.0} deg")
                } else {
                    format!(
                        "off by {:.0} deg: calibrate again or check what is mounted near the compass",
                        median_deg.abs()
                    )
                }
            }
        }
    }
}

/// Wraps an angle to [-180, 180) degrees.
fn wrap180(deg: f64) -> f64 {
    (deg + 180.0).rem_euclid(360.0) - 180.0
}

/// Median of a non-empty slice (sorted in place); the mean of the two middle values for an even count.
fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

/// The tilt-compensated magnetic heading, degrees clockwise from magnetic north in [0, 360): the angle of the
/// body's forward axis, projected on the horizontal plane (perpendicular to the accelerometer's gravity direction),
/// from the horizontal part of the field. `None` when the accelerometer or the field has no usable direction, or
/// the forward axis is vertical.
pub fn compass_heading_deg(accel: [f32; 3], mag: [f32; 3]) -> Option<f64> {
    let a = accel.map(f64::from);
    let m = mag.map(f64::from);
    let an = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    // The accelerometer reads specific force, so down is minus its direction.
    let d = [-a[0] / an, -a[1] / an, -a[2] / an];
    // Forward axis (1, 0, 0) with its vertical part removed, and right = down x forward.
    let f = [1.0 - d[0] * d[0], -d[0] * d[1], -d[0] * d[2]];
    let fnorm = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
    let f: Vec3 = [f[0] / fnorm, f[1] / fnorm, f[2] / fnorm];
    let r = [
        d[1] * f[2] - d[2] * f[1],
        d[2] * f[0] - d[0] * f[2],
        d[0] * f[1] - d[1] * f[0],
    ];
    let m_f = m[0] * f[0] + m[1] * f[1] + m[2] * f[2];
    let m_r = m[0] * r[0] + m[1] * r[1] + m[2] * r[2];
    // The horizontal field points to magnetic north: north is at angle (-m_r, m_f) from forward, so the forward
    // axis is at atan2(-m_r, m_f) from north, clockwise.
    let heading = (-m_r).atan2(m_f).to_degrees();
    heading.is_finite().then(|| heading.rem_euclid(360.0))
}

/// The heading check. `imu` must be sorted by time; each GPS record with a finite course and a ground speed of
/// at least [`MIN_SPEED_MPS`] is paired with the IMU sample nearest in time (the nearest one, with no cutoff: the
/// IMU is at 25 Hz against the GPS' 1 Hz), and gives (compass heading + `declination_deg` - course) wrapped to
/// +-180. A record whose IMU sample has no usable heading is not counted. With fewer than [`MIN_RECORDS`] the
/// result is [`HeadingCheck::NotEnough`]; otherwise the median of the differences and the spread, the median
/// absolute deviation about that median (robust: one wild GPS course does not move it).
pub fn check(imu: &[ImuSample], gps: &[GpsRecord], declination_deg: f64) -> HeadingCheck {
    let mut diffs = Vec::new();
    for g in gps {
        if !(g.course_deg.is_finite() && g.speed_mps.is_finite() && g.speed_mps >= MIN_SPEED_MPS) {
            continue;
        }
        let i = imu.partition_point(|s| s.time_usec < g.time_usec);
        let nearest = [i.checked_sub(1), (i < imu.len()).then_some(i)]
            .into_iter()
            .flatten()
            .min_by_key(|&k| imu[k].time_usec.abs_diff(g.time_usec));
        let Some(s) = nearest.map(|k| &imu[k]) else {
            continue;
        };
        if let Some(h) = compass_heading_deg(s.accel, s.mag) {
            diffs.push(wrap180(h + declination_deg - g.course_deg));
        }
    }
    if diffs.len() < MIN_RECORDS {
        return HeadingCheck::NotEnough {
            records: diffs.len(),
        };
    }
    let records = diffs.len();
    let med = median(&mut diffs);
    let mut dev: Vec<f64> = diffs.iter().map(|d| wrap180(d - med).abs()).collect();
    let spread = median(&mut dev);
    HeadingCheck::Measured {
        records,
        median_deg: med,
        spread_deg: spread,
        within: med.abs() <= TOLERANCE_DEG,
    }
}
