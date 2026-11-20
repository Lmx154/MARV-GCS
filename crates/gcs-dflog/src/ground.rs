//! The ground check of the tilt estimate: with the motors running on the ground the vehicle does not rotate,
//! so the estimated tilt must not move. Figures only, no verdict. See [`crate::flight_report_with`].
//!
//! Streaming: a ground run holds the sums of its reference interval and one small entry per half-second bin.

use std::collections::BTreeMap;

use crate::flight_report::BIN_US;
use crate::reader::Record;
use crate::tracking::{TrackSpec, euler_deg, wrap_deg};

/// The reference interval, seconds: it ends at the start of the ground run.
pub const GROUND_REF_S: f64 = 1.0;
/// The reference interval needs at least this many `NAV` records and this many `IMU` (instance 0) records,
/// else the ground run has no figures.
pub const GROUND_REF_MIN_RECORDS: u64 = 20;

/// A figure of one half-second bin: degrees, signed, with the start of the bin.
#[derive(Clone, Debug, PartialEq)]
pub struct BinFigure {
    pub value: f64,
    pub at_us: u64,
}

/// The figures of one ground run, degrees; roll and pitch are the first and second of each pair.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundFigures {
    /// Mean estimated roll and pitch over the reference interval.
    pub reference: [f64; 2],
    /// Mean gyro X and Y over the reference interval, deg/s.
    pub gyro_bias_dps: [f64; 2],
    /// Bins with an estimate (`NAV`) and gyro (`IMU`) record.
    pub bins: u32,
    /// The largest shift of the bin's mean estimate from the reference.
    pub max_shift: [Option<BinFigure>; 2],
    /// The gyro-integrated rotation about body x and y at the last sample of the run.
    pub gyro_end: [f64; 2],
    /// The largest of (estimate shift) - (gyro-integrated rotation at the end of the bin).
    pub max_unexplained: [Option<BinFigure>; 2],
    /// The largest shift of the accelerometer-implied tilt of a bin from that of the reference interval.
    pub max_accel_shift: [Option<BinFigure>; 2],
}

/// One ground run.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundRunCheck {
    pub window: usize,
    pub number: usize,
    pub start_us: u64,
    pub end_us: u64,
    pub ref_nav_records: u64,
    pub ref_imu_records: u64,
    /// `None`: the reference interval has too few records.
    pub figures: Option<GroundFigures>,
}

/// Why the ground check has, or has not, figures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroundState {
    Done,
    NeedsPoles,
    NotInLog,
}

/// The ground-check section; no ground run: `runs` is empty.
#[derive(Clone, Debug, PartialEq)]
pub struct GroundCheck {
    pub state: GroundState,
    pub runs: Vec<GroundRunCheck>,
}

#[derive(Default, Clone)]
struct BinAcc {
    nav_n: u64,
    roll: f64,
    pitch: f64,
    imu_n: u64,
    acc: [f64; 3],
    /// The gyro integral after the last sample of the bin.
    gyro: Option<[f64; 2]>,
}

#[derive(Default)]
struct RefAcc {
    nav_n: u64,
    roll: f64,
    pitch: f64,
    imu_n: u64,
    gyro: [f64; 2],
    acc: [f64; 3],
}

struct State {
    ref_acc: RefAcc,
    bias: Option<[f64; 2]>,
    prev_t: Option<u64>,
    integral: [f64; 2],
    bins: BTreeMap<u64, BinAcc>,
}

/// Measures the ground runs while the `NAV` and `IMU` records go by.
pub(crate) struct Runner {
    specs: Vec<TrackSpec>,
    states: Vec<State>,
    first: usize,
}

/// Roll and pitch implied by an acceleration (the vehicle at rest), degrees.
fn accel_tilt(a: [f64; 3]) -> [f64; 2] {
    [
        (-a[1]).atan2(-a[2]).to_degrees(),
        a[0].atan2(a[1].hypot(a[2])).to_degrees(),
    ]
}

impl Runner {
    /// `specs` are sorted by start.
    pub fn new(specs: Vec<TrackSpec>) -> Runner {
        let states = specs
            .iter()
            .map(|_| State {
                ref_acc: RefAcc::default(),
                bias: None,
                prev_t: None,
                integral: [0.0; 2],
                bins: BTreeMap::new(),
            })
            .collect();
        Runner {
            specs,
            states,
            first: 0,
        }
    }

    fn ref_us() -> u64 {
        (GROUND_REF_S * 1e6).round() as u64
    }

    /// One `NAV` or `IMU` record.
    pub fn feed(&mut self, r: &Record<'_>) {
        let nav = r.type_name() == "NAV";
        if !nav && r.number_of("I").unwrap_or(0.0) != 0.0 {
            return;
        }
        let Some(t) = r.time_us() else { return };
        while self.first < self.specs.len() && self.specs[self.first].end_us <= t {
            self.first += 1;
        }
        let mut sample = [0.0; 6];
        let mut tilt = [0.0; 2];
        if nav {
            let mut q = [0.0; 4];
            for (v, n) in q.iter_mut().zip(["Q1", "Q2", "Q3", "Q4"]) {
                let Some(x) = r.number_of(n).filter(|x| x.is_finite()) else {
                    return;
                };
                *v = x;
            }
            let e = euler_deg(q);
            tilt = [e[0], e[1]];
        } else {
            for (v, n) in sample
                .iter_mut()
                .zip(["GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"])
            {
                let Some(x) = r.number_of(n).filter(|x| x.is_finite()) else {
                    return;
                };
                *v = x;
            }
        }
        let reference = Self::ref_us();
        for i in self.first..self.specs.len() {
            let sp = &self.specs[i];
            if sp.start_us.saturating_sub(reference) > t {
                break;
            }
            let st = &mut self.states[i];
            if t < sp.start_us {
                let ra = &mut st.ref_acc;
                if nav {
                    ra.nav_n += 1;
                    ra.roll += tilt[0];
                    ra.pitch += tilt[1];
                } else {
                    ra.imu_n += 1;
                    ra.gyro[0] += sample[0];
                    ra.gyro[1] += sample[1];
                    for k in 0..3 {
                        ra.acc[k] += sample[3 + k];
                    }
                }
                continue;
            }
            let idx = (t - sp.start_us) / BIN_US;
            if st.bias.is_none() {
                let ra = &st.ref_acc;
                st.bias = Some(if ra.imu_n > 0 {
                    [ra.gyro[0] / ra.imu_n as f64, ra.gyro[1] / ra.imu_n as f64]
                } else {
                    [0.0; 2]
                });
            }
            let bin = st.bins.entry(idx).or_default();
            if nav {
                bin.nav_n += 1;
                bin.roll += tilt[0];
                bin.pitch += tilt[1];
            } else {
                let bias = st.bias.unwrap_or([0.0; 2]);
                let from = st.prev_t.unwrap_or(sp.start_us).max(sp.start_us);
                let dt = t.saturating_sub(from) as f64 / 1e6;
                for k in 0..2 {
                    st.integral[k] += (sample[k] - bias[k]).to_degrees() * dt;
                }
                st.prev_t = Some(t);
                bin.imu_n += 1;
                for k in 0..3 {
                    bin.acc[k] += sample[3 + k];
                }
                bin.gyro = Some(st.integral);
            }
        }
    }

    pub fn finish(self) -> Vec<GroundRunCheck> {
        self.specs
            .iter()
            .zip(self.states)
            .map(|(sp, st)| {
                let ra = &st.ref_acc;
                let enough =
                    ra.nav_n >= GROUND_REF_MIN_RECORDS && ra.imu_n >= GROUND_REF_MIN_RECORDS;
                let figures = enough.then(|| figures_of(sp, &st));
                GroundRunCheck {
                    window: sp.window,
                    number: sp.number,
                    start_us: sp.start_us,
                    end_us: sp.end_us,
                    ref_nav_records: ra.nav_n,
                    ref_imu_records: ra.imu_n,
                    figures,
                }
            })
            .collect()
    }
}

fn largest(best: &mut Option<BinFigure>, value: f64, at_us: u64) {
    if best.as_ref().is_none_or(|b| value.abs() > b.value.abs()) {
        *best = Some(BinFigure { value, at_us });
    }
}

fn figures_of(sp: &TrackSpec, st: &State) -> GroundFigures {
    let ra = &st.ref_acc;
    let n = ra.nav_n as f64;
    let m = ra.imu_n as f64;
    let reference = [ra.roll / n, ra.pitch / n];
    let bias = st.bias.unwrap_or([ra.gyro[0] / m, ra.gyro[1] / m]);
    let ref_tilt = accel_tilt(ra.acc.map(|a| a / m));
    let mut f = GroundFigures {
        reference,
        gyro_bias_dps: bias.map(f64::to_degrees),
        bins: 0,
        max_shift: [None, None],
        gyro_end: st.integral,
        max_unexplained: [None, None],
        max_accel_shift: [None, None],
    };
    for (idx, b) in &st.bins {
        let at = sp.start_us + idx * BIN_US;
        if b.nav_n > 0 && b.imu_n > 0 {
            f.bins += 1;
        }
        let shift = (b.nav_n > 0).then(|| {
            let k = b.nav_n as f64;
            [
                wrap_deg(b.roll / k - reference[0]),
                wrap_deg(b.pitch / k - reference[1]),
            ]
        });
        if let Some(s) = shift {
            for (best, v) in f.max_shift.iter_mut().zip(s) {
                largest(best, v, at);
            }
            if let Some(g) = b.gyro {
                for a in 0..2 {
                    largest(&mut f.max_unexplained[a], s[a] - g[a], at);
                }
            }
        }
        if b.imu_n > 0 {
            let k = b.imu_n as f64;
            let t = accel_tilt(b.acc.map(|a| a / k));
            for a in 0..2 {
                largest(&mut f.max_accel_shift[a], wrap_deg(t[a] - ref_tilt[a]), at);
            }
        }
    }
    f
}
