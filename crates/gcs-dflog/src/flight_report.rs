//! The flight-log report: see [`flight_report_with`].

use std::collections::BTreeMap;
use std::io::{self, Read, Seek};
use std::path::Path;

use crate::format::Value;
use crate::reader::{Control, Progress, Reader, Record, scan};
use crate::vibration::{IntervalKind, Runner, Spec, VibrationReport, VibrationState};

/// Standard gravity, m/s^2.
pub const GRAVITY: f64 = 9.80665;
/// The length of one bin, microseconds (half a second).
pub const BIN_US: u64 = 500_000;
/// A flight starts at the first bin whose r (`kf` x the sum of the squared rotor speeds over `mass` x
/// [`GRAVITY`]) reaches this.
pub const FLIGHT_START_R: f64 = 1.0;
/// A flight ends before the first later bin whose r is below this.
pub const FLIGHT_END_R: f64 = 0.5;
/// A flight, without its dropped last bin, has at least this many bins.
pub const MIN_FLIGHT_BINS: usize = 6;
/// A motor fit whose level range is under this share of its mean level is ill-conditioned.
pub const NARROW_LEVEL_RANGE: f64 = 0.25;
/// A ground run: the mean level of the four motors is at least this many levels above the idle plateau.
pub const GROUND_RUN_MARGIN: f64 = 20.0;
/// A ground run has at least this many bins.
pub const MIN_GROUND_RUN_BINS: usize = 4;
/// A motor bin is steady when the standard deviation of its level is below this (DShot levels).
pub const STEADY_LEVEL_STD: f64 = 12.0;
/// The idle plateau of a motor is its lowest level seen while armed, plus this margin (levels).
pub const IDLE_MARGIN: f64 = 4.0;
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
#[derive(Clone, Debug, PartialEq)]
pub struct ReportInputs {
    /// Vehicle mass, kg (above 0).
    pub mass_kg: f64,
    /// Thrust constant, N/(rad/s)^2, per motor (above 0).
    pub kf: f64,
    /// Motor pole count; `None`: the log's `MOT_POLES`, else the sections that need rotor speed say so.
    pub poles: Option<f64>,
    /// Cut-offs (Hz, each above 0) of the first-order low-passes of the 1 g window of the vibration section.
    pub accel_lpf_hz: Vec<f64>,
}

/// Where the pole count of the report came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PolesSource {
    Argument,
    Parameter,
    /// Neither: nothing that needs the rotor speed is computed.
    Missing,
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
    pub imu: bool,
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
    /// Largest `TimeUS` spacing of consecutive `NAV` records inside one armed window (the armed state of a
    /// `NAV` is that of the `MOT` record with the same `TimeUS`).
    pub max_nav_gap_us: Option<u64>,
    /// `DMD` records with `Sat` != 0 inside the flights (0 without a pole count).
    pub saturated_airborne: u64,
    /// Highest `ESC` temperature per motor, degC.
    pub esc_temp_max: [Option<f64>; MOTORS],
    /// Lowest `BAT.Volt` of the first battery instance seen.
    pub battery_min_v: Option<f64>,
}

/// One flight: from the first bin with r >= [`FLIGHT_START_R`] to before the first later bin with r <
/// [`FLIGHT_END_R`] (or the end of the armed window), without its last bin.
#[derive(Clone, Debug, PartialEq)]
pub struct Flight {
    /// The armed window, from 0.
    pub window: usize,
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
    /// The thrust constant of this flight alone: mass x g / `speed_sq_sum`.
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
    pub flights: Vec<Flight>,
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
    /// The level range (max - min of the bin means) is under [`NARROW_LEVEL_RANGE`] of the mean level.
    pub ill_conditioned: bool,
    pub mean_volts: Option<f64>,
}

/// Why a motor has no fit, or its fit.
#[derive(Clone, Debug, PartialEq)]
pub enum MotorFitResult {
    Fit(MotorFit),
    /// Fewer than three usable bins, or all at one level; the count of usable bins.
    NotEnough(u32),
}

/// The thrust constant from the flights, in two rounds: the flight bounds depend on the given kf.
#[derive(Clone, Debug, PartialEq)]
pub struct ThrustConstant {
    /// Round 1: the bounds from the given kf, N/(rad/s)^2.
    pub k1: f64,
    /// The flights of round 1.
    pub flights_round1: u32,
    /// Round 2: the bounds from `k1` (the flights reported); mass x g over the time-mean of the sum of speed^2
    /// over all their bins. `None` when round 2 finds no flight.
    pub k2: Option<f64>,
    /// k of each flight of round 2, in order.
    pub per_flight: Vec<f64>,
    pub flights_min: f64,
    pub flights_max: f64,
    /// The number of flights changed, or a bound moved by more than one bin, between the rounds.
    pub bounds_moved: bool,
}

/// The motor identification over all armed time.
#[derive(Clone, Debug, PartialEq)]
pub struct Identification {
    /// `None` when `MOT` or `ESC` is not in the log.
    pub motors: Option<[MotorFitResult; MOTORS]>,
    /// The fit over the bins of all four motors; `None` when `motors` is.
    pub pooled: Option<MotorFitResult>,
    /// `None` when there is no flight (or no `ESC`).
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
    /// `None`: no `--poles` and no `MOT_POLES`.
    pub poles: Option<f64>,
    pub poles_source: PolesSource,
    pub present: TypesPresent,
    pub health: Health,
    pub windows: Vec<ArmedWindow>,
    pub identification: Identification,
    pub vibration: VibrationReport,
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
    /// A `NAV` whose `MOT` has not come yet.
    nav_pending: Option<u64>,
    /// The last `MOT`: its time, whether it is armed, and the armed window (counted from 1).
    mot_state: Option<(u64, bool, usize)>,
    /// The previous `NAV` with a known armed window: its time and that window.
    nav_prev: Option<(u64, usize)>,
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
                // The firmware writes the `MOT` of a step after its `NAV`, with the same `TimeUS`.
                if let Some((_, armed, id)) = self.mot_state.filter(|m| m.0 == t) {
                    self.nav_resolved(t, armed.then_some(id));
                } else {
                    if self.nav_pending.is_some() {
                        self.nav_prev = None;
                    }
                    self.nav_pending = Some(t);
                }
            }
            _ => {}
        }
    }

    /// A `NAV` at `t` whose armed window (`None`: disarmed) is known.
    fn nav_resolved(&mut self, t: u64, window: Option<usize>) {
        match window {
            Some(id) => {
                if let Some((prev, prev_id)) = self.nav_prev
                    && prev_id == id
                {
                    let gap = t.saturating_sub(prev);
                    self.nav_gap = Some(self.nav_gap.map_or(gap, |g| g.max(gap)));
                }
                self.nav_prev = Some((t, id));
            }
            None => self.nav_prev = None,
        }
    }

    /// The `MOT` at `t` says whether the `NAV` of the same step was armed.
    fn mot_for_nav(&mut self, t: u64, armed: bool) {
        let id = self.windows.len();
        self.mot_state = Some((t, armed, id));
        match self.nav_pending {
            Some(p) if p == t => {
                self.nav_pending = None;
                self.nav_resolved(t, armed.then_some(id));
            }
            Some(p) if p < t => {
                self.nav_pending = None;
                self.nav_prev = None;
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
            self.mot_for_nav(t, false);
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
        self.mot_for_nav(t, true);
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
        let level_min = self
            .points
            .iter()
            .map(|p| p.0)
            .fold(f64::INFINITY, f64::min);
        let level_max = self
            .points
            .iter()
            .map(|p| p.0)
            .fold(f64::NEG_INFINITY, f64::max);
        MotorFitResult::Fit(MotorFit {
            idle_level,
            a,
            b,
            rms_residual: (ss / nf).sqrt(),
            bins: n as u32,
            level_min,
            level_max,
            ill_conditioned: level_max - level_min < NARROW_LEVEL_RANGE * mx,
            mean_volts: (!self.volts.is_empty())
                .then(|| self.volts.iter().sum::<f64>() / self.volts.len() as f64),
        })
    }
}

/// A flight: bins `start..end` of armed window `window` (the dropped last bin is not in it).
#[derive(Clone, Copy, Debug)]
struct Span {
    window: usize,
    start: u64,
    end: u64,
}

fn speed_sq(c: f64, b: &Bin) -> Option<f64> {
    (b.esc_n > 0).then(|| c * c * b.erpm_sq / f64::from(b.esc_n))
}

/// The flights of every window for the thrust constant `k`.
fn flight_bounds(windows: &[Window], c: f64, k: f64, mg: f64) -> Vec<Span> {
    let mut spans = Vec::new();
    for (window, w) in windows.iter().enumerate() {
        let nbins = (w.end - w.start) / BIN_US;
        let r = |idx: u64| {
            w.bins
                .get(&idx)
                .and_then(|b| speed_sq(c, b))
                .map_or(0.0, |s| k * s / mg)
        };
        let mut idx = 0;
        while idx < nbins {
            if r(idx) < FLIGHT_START_R {
                idx += 1;
                continue;
            }
            let start = idx;
            let mut end = idx + 1;
            while end < nbins && r(end) >= FLIGHT_END_R {
                end += 1;
            }
            // The last bin of the run is dropped.
            if end - 1 - start >= MIN_FLIGHT_BINS as u64 {
                spans.push(Span {
                    window,
                    start,
                    end: end - 1,
                });
            }
            idx = end;
        }
    }
    spans
}

fn span_bins<'a>(windows: &'a [Window], s: &Span) -> Vec<&'a Bin> {
    (s.start..s.end)
        .filter_map(|i| windows[s.window].bins.get(&i))
        .collect()
}

/// The time-mean of the sum of speed^2 over all the bins of all the flights.
fn pooled_speed_sq(windows: &[Window], c: f64, spans: &[Span]) -> Option<f64> {
    let bins: Vec<&Bin> = spans.iter().flat_map(|s| span_bins(windows, s)).collect();
    bin_mean(&bins, |b| speed_sq(c, b))
}

/// Whether a bound moved by more than one bin between the two rounds (or the flights are not the same).
fn bounds_moved(a: &[Span], b: &[Span]) -> bool {
    a.len() != b.len()
        || a.iter().zip(b).any(|(x, y)| {
            x.window != y.window || x.start.abs_diff(y.start) > 1 || x.end.abs_diff(y.end) > 1
        })
}

/// The lowest and highest of the motors' mean rotor frequency (Hz) over bins `start..end` of a window.
fn rotor_range(w: &Window, start: u64, end: u64, poles: f64) -> Option<(f64, f64)> {
    let mut per_motor = [(0.0, 0u32); MOTORS];
    for b in (start..end)
        .filter_map(|i| w.bins.get(&i))
        .filter(|b| b.esc_n > 0)
    {
        for (m, acc) in per_motor.iter_mut().enumerate() {
            acc.0 += b.erpm_sum[m] / f64::from(b.esc_n);
            acc.1 += 1;
        }
    }
    let hz = per_motor
        .iter()
        .filter(|(_, n)| *n > 0)
        .map(|(s, n)| s / f64::from(*n) / (poles / 2.0) / 60.0);
    let lo = hz.clone().reduce(f64::min)?;
    let hi = hz.reduce(f64::max)?;
    Some((lo, hi))
}

/// One pass over `reader` with progress and cancellation; `None` if `on_progress` answered `Stop`.
///
/// What it reads, by the log's own `FMT` columns: `MOT` (`Arm`, `A0..A3`), `ESC` (`E0..E3`, `T0..T3`), `NAV`,
/// `SETP` (`Thr`), `DMD` (`TX`, `TY`, `TZ`, `Sat`), `RCS` (`T`), `BAT` (`Inst`, `Volt`, `Curr`; the first
/// instance seen), `FDS` and `DSF` (`Dp`, `FMn`), `PARM` (`MOT_POLES`) and `IMU` (`I`, `GyrX..GyrZ`,
/// `AccX..AccZ`). It never reads a position record (`GPS`, `GPS2`, `GPA`, `POS`, `ORGN`): the report holds
/// no position.
///
/// - An armed window runs from the first `MOT` record with `Arm` = 1 to the first one with `Arm` != 1 after it
///   (a log that ends armed ends the window at its last `MOT` record). Its time is cut in bins of
///   [`BIN_US`] from its start; a partial last bin is left out. Each bin keeps sums only, so memory grows
///   with the armed time in bins, not with the records.
/// - The armed state of a `NAV` record is that of the `MOT` record with the same `TimeUS`; the largest `NAV`
///   spacing counts only two `NAV` of the same armed window.
/// - Rotor speed is `eRPM / (poles / 2) x 2 pi / 60` rad/s. Without a pole count (`--poles`, else the log's
///   `MOT_POLES`) nothing that needs it is computed.
/// - r of a bin is the mean over its `ESC` records of `kf x sum over the motors of speed^2`, over `mass` x
///   [`GRAVITY`]. A flight starts at the first bin of an armed window with r >= [`FLIGHT_START_R`] and ends
///   before the first later bin with r < [`FLIGHT_END_R`] (a bin without `ESC` records has r = 0) or at the
///   end of the window; its last bin is dropped; it needs at least [`MIN_FLIGHT_BINS`] bins. Values per
///   flight are means of the per-bin means.
/// - The thrust constant of a flight is `mass x g` over the time-mean of the sum of speed^2 over its bins
///   (from lift-off to touchdown the mean thrust equals the weight); pooled, the time-mean is taken over all
///   flights. The bounds depend on kf, so: bounds with the given kf, k1 from them; bounds with k1, k2 from
///   them. The flights reported are those of the second round.
/// - The motor identification takes, per motor, the bins with at least two `MOT` and one `ESC` record whose
///   level standard deviation (population) is below [`STEADY_LEVEL_STD`] and whose mean level is above that
///   motor's lowest level seen while armed plus [`IDLE_MARGIN`], and fits speed = a + b x level to the bin
///   means by least squares; the pooled fit takes the points of the four motors.
/// - The vibration section reads the `IMU` records with `I` = 0 inside each flight and each ground run, in a
///   second pass over the stream (hence `Seek`): the intervals are only known once the first pass is done.
///   See [`crate::IntervalVibration`].
pub fn flight_report_with<R: Read + Seek>(
    reader: &mut Reader<R>,
    inputs: &ReportInputs,
    mut on_progress: impl FnMut(Progress) -> Control,
) -> io::Result<Option<FlightReport>> {
    let valid = |x: f64| x.is_finite() && x > 0.0;
    if !valid(inputs.mass_kg)
        || !valid(inputs.kf)
        || !inputs.poles.is_none_or(valid)
        || !inputs.accel_lpf_hz.iter().all(|f| valid(*f))
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "mass, kf, poles and the cut-offs must be numbers above 0",
        ));
    }
    let mut pass = Pass::default();
    if !scan(reader, |r| pass.feed(r), &mut on_progress)? {
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
        imu: has("IMU"),
    };
    let (poles, poles_source) = match (inputs.poles, pass.poles_param.filter(|p| valid(*p))) {
        (Some(p), _) => (Some(p), PolesSource::Argument),
        (None, Some(p)) => (Some(p), PolesSource::Parameter),
        (None, None) => (None, PolesSource::Missing),
    };
    let (bytes, bad_bytes, tail_bytes) = (
        reader.bytes_consumed(),
        reader.bad_bytes(),
        reader.tail_bytes(),
    );
    let ended_armed = pass.windows.last().is_some_and(|w| w.open);
    for w in &mut pass.windows {
        if w.open {
            w.open = false;
            w.end = pass.last_mot.max(w.start);
        }
    }

    let mg = inputs.mass_kg * GRAVITY;
    let mut windows: Vec<ArmedWindow> = pass
        .windows
        .iter()
        .map(|w| ArmedWindow {
            start_us: w.start,
            end_us: w.end,
            open_at_end: false,
            flights: Vec::new(),
        })
        .collect();
    if let Some(last) = windows.last_mut()
        && ended_armed
    {
        last.open_at_end = true;
    }

    let mut saturated = 0u64;
    let mut thrust = None;
    let mut motors = None;
    let mut pooled = None;
    let mut specs: Vec<Spec> = Vec::new();
    if let Some(poles) = poles {
        // rad/s per eRPM.
        let c = 2.0 / poles * std::f64::consts::TAU / 60.0;

        // Flights, in two rounds: the bounds depend on the constant.
        let round1 = flight_bounds(&pass.windows, c, inputs.kf, mg);
        let k1 = pooled_speed_sq(&pass.windows, c, &round1).map(|s| mg / s);
        let round2 = k1.map_or_else(Vec::new, |k| flight_bounds(&pass.windows, c, k, mg));
        let k2 = pooled_speed_sq(&pass.windows, c, &round2).map(|s| mg / s);
        let mut per_flight = Vec::new();
        for span in &round2 {
            let w = &pass.windows[span.window];
            let bins = span_bins(&pass.windows, span);
            let ssq = bin_mean(&bins, |b| speed_sq(c, b)).unwrap_or(f64::NAN);
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
            per_flight.push(mg / ssq);
            windows[span.window].flights.push(Flight {
                window: span.window,
                start_us: w.start + span.start * BIN_US,
                end_us: w.start + span.end * BIN_US,
                bins: (span.end - span.start) as u32,
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
            specs.push(Spec {
                kind: IntervalKind::Flight,
                window: span.window,
                number: windows[span.window].flights.len(),
                start_us: w.start + span.start * BIN_US,
                end_us: w.start + span.end * BIN_US,
                rotor_hz: rotor_range(w, span.start, span.end, poles),
            });
        }
        thrust = k1.map(|k1| ThrustConstant {
            k1,
            flights_round1: round1.len() as u32,
            k2,
            flights_min: per_flight.iter().copied().fold(f64::INFINITY, f64::min),
            flights_max: per_flight.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            per_flight,
            bounds_moved: bounds_moved(&round1, &round2),
        });

        // Ground runs: armed, not in a flight, the motors well above idle.
        if pass.idle_seen {
            let plateau = pass.idle.iter().sum::<f64>() / MOTORS as f64;
            for (wi, w) in pass.windows.iter().enumerate() {
                let nbins = (w.end - w.start) / BIN_US;
                // The dropped last bin of a flight is still in the flight.
                let in_flight = |i: u64| {
                    round2
                        .iter()
                        .any(|s| s.window == wi && (s.start..=s.end).contains(&i))
                };
                let hot = |i: u64| {
                    !in_flight(i)
                        && w.bins.get(&i).is_some_and(|b| {
                            b.mot_n > 0
                                && b.level_sum.iter().sum::<f64>()
                                    / (MOTORS as f64 * f64::from(b.mot_n))
                                    >= plateau + GROUND_RUN_MARGIN
                        })
                };
                let mut number = 0;
                let mut i = 0;
                while i < nbins {
                    if !hot(i) {
                        i += 1;
                        continue;
                    }
                    let start = i;
                    while i < nbins && hot(i) {
                        i += 1;
                    }
                    if i - start >= MIN_GROUND_RUN_BINS as u64 {
                        number += 1;
                        specs.push(Spec {
                            kind: IntervalKind::GroundRun,
                            window: wi,
                            number,
                            start_us: w.start + start * BIN_US,
                            end_us: w.start + i * BIN_US,
                            rotor_hz: rotor_range(w, start, i, poles),
                        });
                    }
                }
            }
        }

        // Identification over all armed time.
        if present.mot && present.esc {
            let mut all = Fitter {
                points: Vec::new(),
                volts: Vec::new(),
            };
            let fits: [MotorFitResult; MOTORS] = std::array::from_fn(|m| {
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
                all.points.extend(&fit.points);
                all.volts.extend(&fit.volts);
                fit.finish(idle)
            });
            let idle = if pass.idle_seen {
                pass.idle.iter().copied().fold(f64::INFINITY, f64::min)
            } else {
                0.0
            };
            pooled = Some(all.finish(idle));
            motors = Some(fits);
        }
    }
    specs.sort_by_key(|s| s.start_us);

    let state = if poles.is_none() {
        VibrationState::NeedsPoles
    } else if !present.imu {
        VibrationState::NoImu
    } else {
        VibrationState::Done
    };
    let mut intervals = Vec::new();
    if state == VibrationState::Done && !specs.is_empty() {
        let mut runner = Runner::new(specs, inputs.accel_lpf_hz.clone());
        reader.rewind()?;
        let finished = scan(
            reader,
            |r| {
                if r.type_name() == "IMU" {
                    runner.feed(r);
                }
            },
            &mut on_progress,
        )?;
        if !finished {
            return Ok(None);
        }
        intervals = runner.finish();
    }

    Ok(Some(FlightReport {
        file_name: None,
        bytes,
        bad_bytes,
        tail_bytes,
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
        identification: Identification {
            motors,
            pooled,
            thrust,
        },
        vibration: VibrationReport { state, intervals },
    }))
}

/// The report of any stream; `total` is its size when known.
pub fn flight_report_of<R: Read + Seek>(
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

#[cfg(test)]
mod tests {
    use super::*;

    fn span(window: usize, start: u64, end: u64) -> Span {
        Span { window, start, end }
    }

    #[test]
    fn a_bound_that_moves_by_more_than_one_bin_is_noticed() {
        let a = [span(0, 4, 14), span(1, 2, 12)];
        assert!(!bounds_moved(&a, &a));
        assert!(!bounds_moved(&a, &[span(0, 5, 13), span(1, 2, 12)]));
        assert!(bounds_moved(&a, &[span(0, 6, 14), span(1, 2, 12)]));
        assert!(bounds_moved(&a, &[span(0, 4, 11), span(1, 2, 12)]));
        assert!(bounds_moved(&a, &a[..1]));
        assert!(bounds_moved(&a, &[span(0, 4, 14), span(2, 2, 12)]));
    }
}
