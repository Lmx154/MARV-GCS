//! The flight-log report: see [`flight_report_with`].

use std::collections::BTreeMap;
use std::io::{self, Read};
use std::path::Path;

use crate::format::Value;
use crate::reader::{Control, Progress, Reader, Record, scan};

/// Standard gravity, m/s^2.
pub const GRAVITY: f64 = 9.80665;
/// The length of one bin, microseconds (half a second).
pub const BIN_US: u64 = 500_000;
/// A bin is airborne when `kf` x the sum of the squared rotor speeds reaches this share of `mass` x [`GRAVITY`].
pub const AIRBORNE_FRACTION: f64 = 0.95;
/// A run of airborne bins shorter than this is not a flight segment.
pub const MIN_SEGMENT_BINS: usize = 6;
/// A motor bin is steady when the standard deviation of its level is below this (DShot levels).
pub const STEADY_LEVEL_STD: f64 = 12.0;
/// The idle plateau of a motor is its lowest level seen while armed, plus this margin (levels).
pub const IDLE_MARGIN: f64 = 4.0;
/// The pole count used when neither `--poles` nor the log's `MOT_POLES` gives one (the owner's rule for this tool).
pub const DEFAULT_POLES: f64 = 14.0;
/// The parameter that gives the pole count.
pub const POLES_PARAMETER: &str = "MOT_POLES";
/// The motors of the vehicle, in the order of `MOT.A0..A3` and `ESC.E0..E3`.
pub const MOTORS: usize = 4;

const MOT_LEVELS: [&str; MOTORS] = ["A0", "A1", "A2", "A3"];
const ESC_ERPM: [&str; MOTORS] = ["E0", "E1", "E2", "E3"];
const ESC_TEMP: [&str; MOTORS] = ["T0", "T1", "T2", "T3"];
/// Record types that hold a position: never read, not even for their time.
const POSITION_TYPES: [&str; 5] = ["GPS", "GPS2", "GPA", "POS", "ORGN"];

/// What the report needs from the caller: the airframe's numbers are never assumed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReportInputs {
    /// Vehicle mass, kg (above 0).
    pub mass_kg: f64,
    /// Thrust constant, N/(rad/s)^2, per motor (above 0).
    pub kf: f64,
    /// Motor pole count; `None`: the log's `MOT_POLES`, else [`DEFAULT_POLES`].
    pub poles: Option<f64>,
}

/// Where the pole count of the report came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PolesSource {
    Argument,
    Parameter,
    Default,
}

/// Which of the record types the report reads the log defines (a `FMT` for it).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TypesPresent {
    pub nav: bool,
    pub setp: bool,
    pub dmd: bool,
    pub mot: bool,
    pub esc: bool,
    pub rcs: bool,
    pub bat: bool,
    pub fds: bool,
    pub dsf: bool,
}

/// Counters and extremes of the whole log.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Health {
    /// Last `FDS.Dp`, dropped records (cumulative).
    pub dropped_fds: Option<u64>,
    /// Last `DSF.Dp`.
    pub dropped_dsf: Option<u64>,
    /// Least `FDS.FMn`, bytes.
    pub least_free_fds: Option<u64>,
    /// Least `DSF.FMn`, bytes.
    pub least_free_dsf: Option<u64>,
    /// Largest `TimeUS` spacing of consecutive `NAV` records inside an armed window.
    pub max_nav_gap_us: Option<u64>,
    /// `DMD` records with `Sat` != 0 inside the airborne segments.
    pub saturated_airborne: u64,
    /// Highest `ESC` temperature per motor, degC.
    pub esc_temp_max: [Option<f64>; MOTORS],
    /// Lowest `BAT.Volt` of the first battery instance seen.
    pub battery_min_v: Option<f64>,
}

/// One flight segment: airborne bins without the first and last of their run.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub start_us: u64,
    pub end_us: u64,
    pub bins: u32,
    /// Mean throttle stick `RCS.T`, 0..1.
    pub stick_mean: Option<f64>,
    /// Mean `SETP.Thr`, N.
    pub collective_n: Option<f64>,
    /// Mean `SETP.Thr` over mass x g.
    pub collective_fraction: Option<f64>,
    /// Mean mechanical rpm per motor.
    pub rpm: [f64; MOTORS],
    /// Mean of kf x sum of speed^2, over mass x g.
    pub thrust_ratio: f64,
    /// Time-mean of the sum of speed^2 over the motors, (rad/s)^2.
    pub speed_sq_sum: f64,
    /// The thrust constant of this segment alone: mass x g / `speed_sq_sum`.
    pub k: f64,
    pub volts_min: Option<f64>,
    pub amps_mean: Option<f64>,
    pub amps_max: Option<f64>,
    /// Mean `DMD.TX`, `TY`, `TZ`.
    pub torque: [Option<f64>; 3],
}

/// One stretch of `MOT.Arm` = 1.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmedWindow {
    pub start_us: u64,
    /// The first `MOT` record with `Arm` = 0 after it, or the last `MOT` record when the log ends armed.
    pub end_us: u64,
    pub open_at_end: bool,
    /// Empty: "armed, not airborne".
    pub segments: Vec<Segment>,
}

/// The line `speed = a + b x level` of one motor.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorFit {
    pub idle_level: f64,
    /// rad/s.
    pub a: f64,
    /// rad/s per level.
    pub b: f64,
    /// Root mean square residual, rad/s.
    pub rms_residual: f64,
    pub bins: u32,
    pub level_min: f64,
    pub level_max: f64,
    pub mean_volts: Option<f64>,
}

/// Why a motor has no fit, or its fit.
#[derive(Clone, Debug, PartialEq)]
pub enum MotorFitResult {
    Fit(MotorFit),
    /// Fewer than three usable bins, or all at one level; the count of usable bins.
    NotEnough(u32),
}

/// The thrust constant from the airborne segments.
#[derive(Clone, Debug, PartialEq)]
pub struct ThrustConstant {
    /// mass x g over the time-mean of the sum of speed^2 over all airborne bins, N/(rad/s)^2.
    pub k: f64,
    /// Per segment, in order.
    pub per_segment: Vec<f64>,
    pub segments_min: f64,
    pub segments_max: f64,
}

/// The motor identification over all armed time.
#[derive(Clone, Debug, PartialEq)]
pub struct Identification {
    /// `None` when `MOT` or `ESC` is not in the log.
    pub motors: Option<[MotorFitResult; MOTORS]>,
    /// `None` when there is no airborne segment (or no `ESC`).
    pub thrust: Option<ThrustConstant>,
}

/// What one pass over a flight log found.
#[derive(Clone, Debug, PartialEq)]
pub struct FlightReport {
    /// The file's name when the report was made from a path.
    pub file_name: Option<String>,
    pub bytes: u64,
    pub bad_bytes: u64,
    pub tail_bytes: u64,
    /// `TimeUS` of the first and last record that has one, position records excluded.
    pub first_time_us: Option<u64>,
    pub last_time_us: Option<u64>,
    pub mass_kg: f64,
    pub kf: f64,
    pub poles: f64,
    pub poles_source: PolesSource,
    pub present: TypesPresent,
    pub health: Health,
    pub windows: Vec<ArmedWindow>,
    pub identification: Identification,
}

impl FlightReport {
    pub fn duration_us(&self) -> Option<u64> {
        Some(self.last_time_us?.saturating_sub(self.first_time_us?))
    }
}

/// Everything of one half-second bin of an armed window.
#[derive(Clone, Default)]
struct Bin {
    mot_n: u32,
    level_sum: [f64; MOTORS],
    level_sq: [f64; MOTORS],
    esc_n: u32,
    erpm_sum: [f64; MOTORS],
    /// Sum over the records of the sum over the motors of eRPM^2.
    erpm_sq: f64,
    rcs_n: u32,
    rcs_sum: f64,
    setp_n: u32,
    setp_sum: f64,
    dmd_n: u32,
    torque_sum: [f64; 3],
    sat: u64,
    bat_n: u32,
    volt_sum: f64,
    volt_min: f64,
    amp_sum: f64,
    amp_max: f64,
}

struct Window {
    start: u64,
    end: u64,
    open: bool,
    bins: BTreeMap<u64, Bin>,
}

#[derive(Default)]
struct Pass {
    first: Option<u64>,
    last: Option<u64>,
    windows: Vec<Window>,
    last_mot: u64,
    idle: [f64; MOTORS],
    idle_seen: bool,
    poles_param: Option<f64>,
    nav_prev: Option<u64>,
    nav_gap: Option<u64>,
    fds_dp: Option<u64>,
    dsf_dp: Option<u64>,
    fds_min: Option<u64>,
    dsf_min: Option<u64>,
    esc_temp: [Option<f64>; MOTORS],
    bat_inst: Option<u64>,
    bat_min: Option<f64>,
}

fn num(r: &Record<'_>, name: &str) -> Option<f64> {
    r.number_of(name)
}

fn nums<const N: usize>(r: &Record<'_>, names: &[&str; N]) -> Option<[f64; N]> {
    let mut out = [0.0; N];
    for (o, n) in out.iter_mut().zip(names) {
        *o = r.number_of(n)?;
    }
    Some(out)
}

impl Pass {
    /// The bin of the open window that `t` falls in.
    fn bin(&mut self, t: u64) -> Option<&mut Bin> {
        let w = self.windows.last_mut().filter(|w| w.open)?;
        let idx = t.checked_sub(w.start)? / BIN_US;
        Some(w.bins.entry(idx).or_insert_with(|| Bin {
            volt_min: f64::INFINITY,
            amp_max: f64::NEG_INFINITY,
            ..Bin::default()
        }))
    }

    fn feed(&mut self, r: &Record<'_>) {
        let name = r.type_name();
        // A position record is never read.
        if POSITION_TYPES.contains(&name) {
            return;
        }
        let Some(t) = r.time_us() else { return };
        self.first.get_or_insert(t);
        self.last = Some(t);
        match name {
            "PARM" => {
                if let (Some(Value::Text(n)), Some(v)) = (r.get("Name"), num(r, "Value"))
                    && n == POLES_PARAMETER
                {
                    self.poles_param = Some(v);
                }
            }
            "FDS" => {
                if let Some(dp) = num(r, "Dp") {
                    self.fds_dp = Some(dp as u64);
                }
                if let Some(m) = num(r, "FMn") {
                    self.fds_min = Some(self.fds_min.map_or(m as u64, |x| x.min(m as u64)));
                }
            }
            "DSF" => {
                if let Some(dp) = num(r, "Dp") {
                    self.dsf_dp = Some(dp as u64);
                }
                if let Some(m) = num(r, "FMn") {
                    self.dsf_min = Some(self.dsf_min.map_or(m as u64, |x| x.min(m as u64)));
                }
            }
            "MOT" => self.mot(r, t),
            "ESC" => {
                if let Some(temps) = nums(r, &ESC_TEMP) {
                    for (m, v) in self.esc_temp.iter_mut().zip(temps) {
                        *m = Some(m.map_or(v, |x| x.max(v)));
                    }
                }
                if let Some(e) = nums(r, &ESC_ERPM)
                    && let Some(b) = self.bin(t)
                {
                    b.esc_n += 1;
                    for (s, v) in b.erpm_sum.iter_mut().zip(e) {
                        *s += v;
                    }
                    b.erpm_sq += e.iter().map(|v| v * v).sum::<f64>();
                }
            }
            "RCS" => {
                if let Some(v) = num(r, "T")
                    && let Some(b) = self.bin(t)
                {
                    b.rcs_n += 1;
                    b.rcs_sum += v;
                }
            }
            "SETP" => {
                if let Some(v) = num(r, "Thr")
                    && let Some(b) = self.bin(t)
                {
                    b.setp_n += 1;
                    b.setp_sum += v;
                }
            }
            "DMD" => {
                if let (Some(tx), Some(ty), Some(tz), Some(sat)) =
                    (num(r, "TX"), num(r, "TY"), num(r, "TZ"), num(r, "Sat"))
                    && let Some(b) = self.bin(t)
                {
                    b.dmd_n += 1;
                    for (s, v) in b.torque_sum.iter_mut().zip([tx, ty, tz]) {
                        *s += v;
                    }
                    b.sat += u64::from(sat != 0.0);
                }
            }
            "BAT" => self.bat(r, t),
            "NAV" => {
                let armed = self.windows.last().is_some_and(|w| w.open && t >= w.start);
                if armed {
                    let start = self.windows.last().map_or(0, |w| w.start);
                    if let Some(prev) = self.nav_prev.filter(|p| *p >= start) {
                        let gap = t.saturating_sub(prev);
                        self.nav_gap = Some(self.nav_gap.map_or(gap, |g| g.max(gap)));
                    }
                }
                self.nav_prev = Some(t);
            }
            _ => {}
        }
    }

    fn bat(&mut self, r: &Record<'_>, t: u64) {
        let (Some(volt), Some(curr), Some(inst)) = (num(r, "Volt"), num(r, "Curr"), num(r, "Inst"))
        else {
            return;
        };
        // One instance: the first seen.
        if *self.bat_inst.get_or_insert(inst as u64) != inst as u64 {
            return;
        }
        self.bat_min = Some(self.bat_min.map_or(volt, |v| v.min(volt)));
        if let Some(b) = self.bin(t) {
            b.bat_n += 1;
            b.volt_sum += volt;
            b.volt_min = b.volt_min.min(volt);
            b.amp_sum += curr;
            b.amp_max = b.amp_max.max(curr);
        }
    }

    fn mot(&mut self, r: &Record<'_>, t: u64) {
        let (Some(arm), Some(levels)) = (num(r, "Arm"), nums(r, &MOT_LEVELS)) else {
            return;
        };
        self.last_mot = t;
        let open = self.windows.last().is_some_and(|w| w.open);
        if arm != 1.0 {
            if let Some(w) = self.windows.last_mut().filter(|w| w.open) {
                w.open = false;
                w.end = t;
            }
            return;
        }
        if !open {
            self.windows.push(Window {
                start: t,
                end: t,
                open: true,
                bins: BTreeMap::new(),
            });
        }
        if self.idle_seen {
            for (i, l) in self.idle.iter_mut().zip(levels) {
                *i = i.min(l);
            }
        } else {
            self.idle = levels;
            self.idle_seen = true;
        }
        if let Some(b) = self.bin(t) {
            b.mot_n += 1;
            for ((s, q), l) in b.level_sum.iter_mut().zip(&mut b.level_sq).zip(levels) {
                *s += l;
                *q += l * l;
            }
        }
    }
}

/// Mean of the per-bin means of `f` over the bins that have one.
fn bin_mean(bins: &[&Bin], f: impl Fn(&Bin) -> Option<f64>) -> Option<f64> {
    let (mut sum, mut n) = (0.0, 0u32);
    for v in bins.iter().filter_map(|b| f(b)) {
        sum += v;
        n += 1;
    }
    (n > 0).then(|| sum / f64::from(n))
}

struct Fitter {
    points: Vec<(f64, f64)>,
    volts: Vec<f64>,
}

impl Fitter {
    fn finish(&self, idle_level: f64) -> MotorFitResult {
        let n = self.points.len();
        let nf = n as f64;
        let mx = self.points.iter().map(|p| p.0).sum::<f64>() / nf;
        let my = self.points.iter().map(|p| p.1).sum::<f64>() / nf;
        let sxx: f64 = self.points.iter().map(|p| (p.0 - mx).powi(2)).sum();
        if n < 3 || sxx <= 0.0 {
            return MotorFitResult::NotEnough(n as u32);
        }
        let sxy: f64 = self.points.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
        let b = sxy / sxx;
        let a = my - b * mx;
        let ss: f64 = self
            .points
            .iter()
            .map(|p| (p.1 - (a + b * p.0)).powi(2))
            .sum();
        MotorFitResult::Fit(MotorFit {
            idle_level,
            a,
            b,
            rms_residual: (ss / nf).sqrt(),
            bins: n as u32,
            level_min: self
                .points
                .iter()
                .map(|p| p.0)
                .fold(f64::INFINITY, f64::min),
            level_max: self
                .points
                .iter()
                .map(|p| p.0)
                .fold(f64::NEG_INFINITY, f64::max),
            mean_volts: (!self.volts.is_empty())
                .then(|| self.volts.iter().sum::<f64>() / self.volts.len() as f64),
        })
    }
}

/// One pass over `reader` with progress and cancellation; `None` if `on_progress` answered `Stop`.
///
/// What it reads, by the log's own `FMT` columns: `MOT` (`Arm`, `A0..A3`), `ESC` (`E0..E3`, `T0..T3`), `NAV`,
/// `SETP` (`Thr`), `DMD` (`TX`, `TY`, `TZ`, `Sat`), `RCS` (`T`), `BAT` (`Inst`, `Volt`, `Curr`; the first
/// instance seen), `FDS` and `DSF` (`Dp`, `FMn`) and `PARM` (`MOT_POLES`). It never reads a position record
/// (`GPS`, `GPS2`, `GPA`, `POS`, `ORGN`): the report holds no position.
///
/// - An armed window runs from the first `MOT` record with `Arm` = 1 to the first one with `Arm` != 1 after it
///   (a log that ends armed ends the window at its last `MOT` record). Its time is cut in bins of
///   [`BIN_US`] from its start; a partial last bin is left out. Each bin keeps sums only, so memory grows
///   with the armed time in bins, not with the records.
/// - Rotor speed is `eRPM / (poles / 2) x 2 pi / 60` rad/s. A bin is airborne when the mean over its `ESC`
///   records of `kf x sum over the motors of speed^2` is at least [`AIRBORNE_FRACTION`] x `mass` x [`GRAVITY`].
///   Runs of at least [`MIN_SEGMENT_BINS`] airborne bins are segments, each without its first and last bin.
///   Values per segment are means of the per-bin means.
/// - The motor identification takes, per motor, the bins with at least two `MOT` and one `ESC` record whose
///   level standard deviation (population) is below [`STEADY_LEVEL_STD`] and whose mean level is above that
///   motor's lowest level seen while armed plus [`IDLE_MARGIN`], and fits speed = a + b x level to the bin
///   means by least squares.
/// - The thrust constant is `mass x g` over the mean of the per-bin means of the sum of speed^2 over every
///   airborne bin, and per segment the same over its bins.
pub fn flight_report_with<R: Read>(
    reader: &mut Reader<R>,
    inputs: &ReportInputs,
    on_progress: impl FnMut(Progress) -> Control,
) -> io::Result<Option<FlightReport>> {
    let valid = |x: f64| x.is_finite() && x > 0.0;
    if !valid(inputs.mass_kg) || !valid(inputs.kf) || !inputs.poles.is_none_or(valid) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "mass, kf and poles must be numbers above 0",
        ));
    }
    let mut pass = Pass::default();
    if !scan(reader, |r| pass.feed(r), on_progress)? {
        return Ok(None);
    }
    let has = |n: &str| reader.formats().any(|f| f.name == n);
    let present = TypesPresent {
        nav: has("NAV"),
        setp: has("SETP"),
        dmd: has("DMD"),
        mot: has("MOT"),
        esc: has("ESC"),
        rcs: has("RCS"),
        bat: has("BAT"),
        fds: has("FDS"),
        dsf: has("DSF"),
    };
    let (poles, poles_source) = match (inputs.poles, pass.poles_param.filter(|p| valid(*p))) {
        (Some(p), _) => (p, PolesSource::Argument),
        (None, Some(p)) => (p, PolesSource::Parameter),
        (None, None) => (DEFAULT_POLES, PolesSource::Default),
    };
    let ended_armed = pass.windows.last().is_some_and(|w| w.open);
    for w in &mut pass.windows {
        if w.open {
            w.open = false;
            w.end = pass.last_mot.max(w.start);
        }
    }

    let mg = inputs.mass_kg * GRAVITY;
    // rad/s per eRPM.
    let c = 2.0 / poles * std::f64::consts::TAU / 60.0;
    let speed_sq = |b: &Bin| (b.esc_n > 0).then(|| c * c * b.erpm_sq / f64::from(b.esc_n));

    let mut windows = Vec::new();
    let mut airborne_bins: Vec<&Bin> = Vec::new();
    let mut segment_ks = Vec::new();
    let mut saturated = 0u64;
    for w in &pass.windows {
        let nbins = (w.end - w.start) / BIN_US;
        let mut segments = Vec::new();
        let mut run: Vec<u64> = Vec::new();
        for idx in 0..=nbins {
            let airborne = idx < nbins
                && w.bins
                    .get(&idx)
                    .and_then(speed_sq)
                    .is_some_and(|s| inputs.kf * s >= AIRBORNE_FRACTION * mg);
            if airborne {
                run.push(idx);
                continue;
            }
            if run.len() >= MIN_SEGMENT_BINS {
                let inner = &run[1..run.len() - 1];
                let bins: Vec<&Bin> = inner.iter().filter_map(|i| w.bins.get(i)).collect();
                let ssq = bin_mean(&bins, speed_sq).unwrap_or(f64::NAN);
                let collective = bin_mean(&bins, |b| {
                    (b.setp_n > 0).then(|| b.setp_sum / f64::from(b.setp_n))
                });
                let mean_motor = |m: usize| {
                    bin_mean(&bins, |b| {
                        (b.esc_n > 0).then(|| b.erpm_sum[m] / f64::from(b.esc_n))
                    })
                    .map_or(f64::NAN, |e| e / (poles / 2.0))
                };
                let torque = |i: usize| {
                    bin_mean(&bins, |b| {
                        (b.dmd_n > 0).then(|| b.torque_sum[i] / f64::from(b.dmd_n))
                    })
                };
                saturated += bins.iter().map(|b| b.sat).sum::<u64>();
                segment_ks.push(mg / ssq);
                segments.push(Segment {
                    start_us: w.start + inner[0] * BIN_US,
                    end_us: w.start + (inner[inner.len() - 1] + 1) * BIN_US,
                    bins: inner.len() as u32,
                    stick_mean: bin_mean(&bins, |b| {
                        (b.rcs_n > 0).then(|| b.rcs_sum / f64::from(b.rcs_n))
                    }),
                    collective_n: collective,
                    collective_fraction: collective.map(|x| x / mg),
                    rpm: [mean_motor(0), mean_motor(1), mean_motor(2), mean_motor(3)],
                    thrust_ratio: inputs.kf * ssq / mg,
                    speed_sq_sum: ssq,
                    k: mg / ssq,
                    volts_min: bins
                        .iter()
                        .filter(|b| b.bat_n > 0)
                        .map(|b| b.volt_min)
                        .reduce(f64::min),
                    amps_mean: bin_mean(&bins, |b| {
                        (b.bat_n > 0).then(|| b.amp_sum / f64::from(b.bat_n))
                    }),
                    amps_max: bins
                        .iter()
                        .filter(|b| b.bat_n > 0)
                        .map(|b| b.amp_max)
                        .reduce(f64::max),
                    torque: [torque(0), torque(1), torque(2)],
                });
                airborne_bins.extend(bins);
            }
            run.clear();
        }
        windows.push(ArmedWindow {
            start_us: w.start,
            end_us: w.end,
            open_at_end: false,
            segments,
        });
    }
    if let Some(last) = windows.last_mut()
        && ended_armed
    {
        last.open_at_end = true;
    }

    // Identification over all armed time.
    let motors = (present.mot && present.esc).then(|| {
        std::array::from_fn(|m| {
            let mut fit = Fitter {
                points: Vec::new(),
                volts: Vec::new(),
            };
            let idle = if pass.idle_seen { pass.idle[m] } else { 0.0 };
            for w in &pass.windows {
                for idx in 0..(w.end - w.start) / BIN_US {
                    let Some(b) = w.bins.get(&idx) else { continue };
                    if b.mot_n < 2 || b.esc_n < 1 {
                        continue;
                    }
                    let n = f64::from(b.mot_n);
                    let mean = b.level_sum[m] / n;
                    let std = (b.level_sq[m] / n - mean * mean).max(0.0).sqrt();
                    if std < STEADY_LEVEL_STD && mean > idle + IDLE_MARGIN {
                        fit.points
                            .push((mean, c * b.erpm_sum[m] / f64::from(b.esc_n)));
                        if b.bat_n > 0 {
                            fit.volts.push(b.volt_sum / f64::from(b.bat_n));
                        }
                    }
                }
            }
            fit.finish(idle)
        })
    });
    let thrust = bin_mean(&airborne_bins, speed_sq).map(|ssq| ThrustConstant {
        k: mg / ssq,
        segments_min: segment_ks.iter().copied().fold(f64::INFINITY, f64::min),
        segments_max: segment_ks.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        per_segment: segment_ks,
    });

    Ok(Some(FlightReport {
        file_name: None,
        bytes: reader.bytes_consumed(),
        bad_bytes: reader.bad_bytes(),
        tail_bytes: reader.tail_bytes(),
        first_time_us: pass.first,
        last_time_us: pass.last,
        mass_kg: inputs.mass_kg,
        kf: inputs.kf,
        poles,
        poles_source,
        present,
        health: Health {
            dropped_fds: pass.fds_dp,
            dropped_dsf: pass.dsf_dp,
            least_free_fds: pass.fds_min,
            least_free_dsf: pass.dsf_min,
            max_nav_gap_us: pass.nav_gap,
            saturated_airborne: saturated,
            esc_temp_max: pass.esc_temp,
            battery_min_v: pass.bat_min,
        },
        windows,
        identification: Identification { motors, thrust },
    }))
}

/// The report of any stream; `total` is its size when known.
pub fn flight_report_of<R: Read>(
    source: R,
    total: Option<u64>,
    inputs: &ReportInputs,
) -> io::Result<FlightReport> {
    let mut reader = Reader::new(source, total);
    Ok(
        flight_report_with(&mut reader, inputs, |_| Control::Continue)?
            .expect("a pass that is never stopped completes"),
    )
}

/// The report of a `.BIN` file, with its file name.
pub fn flight_report(path: impl AsRef<Path>, inputs: &ReportInputs) -> io::Result<FlightReport> {
    let path = path.as_ref();
    let mut reader = Reader::open(path)?;
    let mut report = flight_report_with(&mut reader, inputs, |_| Control::Continue)?
        .expect("a pass that is never stopped completes");
    report.file_name = path.file_name().map(|n| n.to_string_lossy().into_owned());
    Ok(report)
}
