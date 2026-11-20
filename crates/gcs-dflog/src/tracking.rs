//! The tracking figures of the flight report: how well the estimated attitude followed the attitude setpoint
//! (`SETP` with `K` = 1) inside each flight. See [`crate::flight_report_with`].
//!
//! Streaming: a flight holds the setpoint window of [`EXCURSION_WINDOW_S`] (a ring buffer) and a few
//! accumulators; across the log only the holds found and one entry per local maximum of the excursion are kept,
//! never the records.

use std::collections::VecDeque;

use crate::reader::Record;

/// The look-back, seconds, over which the range of the commanded angle is taken for the excursion (the current
/// record is included).
pub const EXCURSION_WINDOW_S: f64 = 0.5;
/// Two excursion events of the whole log are at least this many seconds apart.
pub const EXCURSION_SEPARATION_S: f64 = 1.0;
/// A hold: the setpoint of the axis stays within this many degrees of its value at the first record of the run.
pub const HOLD_TOL_DEG: f64 = 1.0;
/// A hold lasts at least this many seconds.
pub const HOLD_MIN_S: f64 = 0.5;
/// A hold's setpoint at its first record is at least this many degrees away from 0 (either way).
pub const HOLD_MIN_DEG: f64 = 10.0;
/// The settled figures of a hold are means over the part from this many seconds after its start to its end.
pub const HOLD_SETTLE_S: f64 = 0.3;

/// Events listed per axis for the whole log.
pub const EVENTS: usize = 3;

fn us(s: f64) -> u64 {
    (s * 1e6).round() as u64
}

/// The axes of the error figures.
pub const AXES: [&str; 3] = ["roll", "pitch", "yaw"];

/// Error figures of one axis in one flight, degrees; the error is setpoint minus estimate.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisError {
    pub setpoint_min: f64,
    pub setpoint_max: f64,
    pub mean_error: f64,
    pub rms_error: f64,
    /// The error (signed) with the largest magnitude, and its time.
    pub max_abs_error: f64,
    pub max_abs_error_at_us: u64,
}

/// How far the estimate lay outside the range of the setpoint over the preceding [`EXCURSION_WINDOW_S`].
#[derive(Clone, Debug, PartialEq)]
pub struct Excursion {
    pub at_us: u64,
    /// Degrees, above 0.
    pub value: f64,
    pub estimate: f64,
    /// The end of the range the estimate passed.
    pub bound: f64,
    /// The body rate of the axis (`GX` for roll, `GY` for pitch) at the record, deg/s.
    pub rate_dps: f64,
}

/// An excursion event of the whole log, with its flight.
#[derive(Clone, Debug, PartialEq)]
pub struct ExcursionEvent {
    pub window: usize,
    pub number: usize,
    pub excursion: Excursion,
}

/// A stretch of constant setpoint on roll or pitch.
#[derive(Clone, Debug, PartialEq)]
pub struct Hold {
    pub window: usize,
    pub number: usize,
    pub start_us: u64,
    pub duration_us: u64,
    /// Means over the settled part, degrees.
    pub mean_setpoint: f64,
    pub mean_estimate: f64,
    /// Setpoint minus estimate.
    pub settled_error: f64,
}

/// The holds of one axis.
#[derive(Clone, Debug, PartialEq)]
pub struct HoldStats {
    pub count: usize,
    pub median_abs_error: Option<f64>,
    pub max_abs_error: Option<f64>,
}

/// The tracking figures of one flight.
#[derive(Clone, Debug, PartialEq)]
pub struct FlightTracking {
    pub window: usize,
    pub number: usize,
    pub start_us: u64,
    pub end_us: u64,
    /// Records used (a `SETP` with `K` = 1 and the `NAV` of the same `TimeUS`).
    pub records: u64,
    /// Roll, pitch, yaw; `None` without records.
    pub axes: Option<[AxisError; 3]>,
    /// Roll and pitch; `None` when the estimate never left the range.
    pub excursions: [Option<Excursion>; 2],
}

/// Why the tracking section has, or has not, figures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingState {
    Done,
    NeedsPoles,
    NotInLog,
}

/// The tracking section.
#[derive(Clone, Debug, PartialEq)]
pub struct TrackingReport {
    pub state: TrackingState,
    /// `SETP` records inside a flight with a `K` other than 1.
    pub other_kind_records: u64,
    pub flights: Vec<FlightTracking>,
    /// Roll, pitch: the largest excursion events, largest first.
    pub events: [Vec<ExcursionEvent>; 2],
    /// Roll, pitch: all holds, in time order.
    pub holds: [Vec<Hold>; 2],
    pub hold_stats: [HoldStats; 2],
}

impl TrackingReport {
    pub(crate) fn without_figures(state: TrackingState) -> TrackingReport {
        TrackingReport {
            state,
            other_kind_records: 0,
            flights: Vec::new(),
            events: [Vec::new(), Vec::new()],
            holds: [Vec::new(), Vec::new()],
            hold_stats: [HoldStats::none(), HoldStats::none()],
        }
    }
}

impl HoldStats {
    fn none() -> HoldStats {
        HoldStats {
            count: 0,
            median_abs_error: None,
            max_abs_error: None,
        }
    }
}

/// A flight to measure.
#[derive(Clone, Debug)]
pub(crate) struct TrackSpec {
    pub window: usize,
    pub number: usize,
    pub start_us: u64,
    pub end_us: u64,
}

/// Roll, pitch, yaw in degrees of a quaternion [w, x, y, z] (body to NED, ZYX).
fn euler_deg(q: [f64; 4]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw].map(f64::to_degrees)
}

/// `d` wrapped to [-180, 180].
fn wrap_deg(d: f64) -> f64 {
    d - 360.0 * (d / 360.0).round()
}

struct Acc {
    min: f64,
    max: f64,
    sum: f64,
    sum_sq: f64,
    big: f64,
    big_at: u64,
}

impl Acc {
    fn new() -> Acc {
        Acc {
            min: f64::INFINITY,
            max: f64::NEG_INFINITY,
            sum: 0.0,
            sum_sq: 0.0,
            big: 0.0,
            big_at: 0,
        }
    }
}

struct Run {
    start: u64,
    first: f64,
    last: u64,
    settled: u64,
    sum_sp: f64,
    sum_est: f64,
}

#[derive(Default)]
struct Peak {
    best: Option<Excursion>,
    prev: Option<Excursion>,
    rising: bool,
}

struct Cur {
    index: usize,
    n: u64,
    acc: [Acc; 3],
    ring: VecDeque<(u64, [f64; 2])>,
    peaks: [Peak; 2],
    runs: [Option<Run>; 2],
}

/// The `NAV` of a step.
struct Nav {
    t: u64,
    q: [f64; 4],
    rate_dps: [f64; 2],
}

/// The `SETP` (kind 1) of a step.
struct Setp {
    t: u64,
    q: [f64; 4],
}

/// Measures the flights while the `NAV` and `SETP` records go by.
pub(crate) struct Tracker {
    specs: Vec<TrackSpec>,
    other: u64,
    nav: Option<Nav>,
    setp: Option<Setp>,
    cur: Option<Cur>,
    done: Vec<Option<FlightTracking>>,
    holds: [Vec<Hold>; 2],
    candidates: [Vec<ExcursionEvent>; 2],
}

fn quaternion(r: &Record<'_>, names: [&str; 4]) -> Option<[f64; 4]> {
    let mut q = [0.0; 4];
    for (v, n) in q.iter_mut().zip(names) {
        *v = r.number_of(n).filter(|x| x.is_finite())?;
    }
    Some(q)
}

impl Tracker {
    pub fn new(specs: Vec<TrackSpec>) -> Tracker {
        let done = specs.iter().map(|_| None).collect();
        Tracker {
            specs,
            other: 0,
            nav: None,
            setp: None,
            cur: None,
            done,
            holds: [Vec::new(), Vec::new()],
            candidates: [Vec::new(), Vec::new()],
        }
    }

    fn flight_at(&self, t: u64) -> Option<usize> {
        let at = self.specs.partition_point(|sp| sp.end_us <= t);
        (at < self.specs.len() && self.specs[at].start_us <= t).then_some(at)
    }

    /// One `NAV` or `SETP` record.
    pub fn feed(&mut self, r: &Record<'_>) {
        let Some(t) = r.time_us() else { return };
        let Some(at) = self.flight_at(t) else { return };
        if r.type_name() == "NAV" {
            let Some(q) = quaternion(r, ["Q1", "Q2", "Q3", "Q4"]) else {
                return;
            };
            let (Some(gx), Some(gy)) = (r.number_of("GX"), r.number_of("GY")) else {
                return;
            };
            let nav = Nav {
                t,
                q,
                rate_dps: [gx.to_degrees(), gy.to_degrees()],
            };
            match self.setp.take_if(|s| s.t == t) {
                Some(setp) => self.pair(at, &setp, &nav),
                None => self.nav = Some(nav),
            }
        } else {
            let Some(kind) = r.number_of("K") else { return };
            if kind != 1.0 {
                self.other += 1;
                return;
            }
            let Some(q) = quaternion(r, ["Q1", "Q2", "Q3", "Q4"]) else {
                return;
            };
            let setp = Setp { t, q };
            match self.nav.take_if(|n| n.t == t) {
                Some(nav) => self.pair(at, &setp, &nav),
                None => self.setp = Some(setp),
            }
        }
    }

    fn pair(&mut self, at: usize, setp: &Setp, nav: &Nav) {
        if self.cur.as_ref().is_none_or(|c| c.index != at) {
            self.finish_cur();
            self.cur = Some(Cur {
                index: at,
                n: 0,
                acc: std::array::from_fn(|_| Acc::new()),
                ring: VecDeque::new(),
                peaks: Default::default(),
                runs: [None, None],
            });
        }
        let t = setp.t;
        let (sp, est) = (euler_deg(setp.q), euler_deg(nav.q));
        let window = us(EXCURSION_WINDOW_S);
        let spec = self.specs[at].clone();
        let Some(c) = self.cur.as_mut() else { return };
        c.n += 1;
        for a in 0..3 {
            let mut e = sp[a] - est[a];
            if a == 2 {
                e = wrap_deg(e);
            }
            let acc = &mut c.acc[a];
            acc.min = acc.min.min(sp[a]);
            acc.max = acc.max.max(sp[a]);
            acc.sum += e;
            acc.sum_sq += e * e;
            if c.n == 1 || e.abs() > acc.big.abs() {
                acc.big = e;
                acc.big_at = t;
            }
        }
        c.ring.push_back((t, [sp[0], sp[1]]));
        while c.ring.front().is_some_and(|f| f.0 + window < t) {
            c.ring.pop_front();
        }
        for a in 0..2 {
            let lo = c.ring.iter().map(|s| s.1[a]).fold(f64::INFINITY, f64::min);
            let hi = c
                .ring
                .iter()
                .map(|s| s.1[a])
                .fold(f64::NEG_INFINITY, f64::max);
            let (value, bound) = if est[a] > hi {
                (est[a] - hi, hi)
            } else if est[a] < lo {
                (lo - est[a], lo)
            } else {
                (0.0, est[a])
            };
            let now = Excursion {
                at_us: t,
                value,
                estimate: est[a],
                bound,
                rate_dps: nav.rate_dps[a],
            };
            let p = &mut c.peaks[a];
            let prev_value = p.prev.as_ref().map(|prev| prev.value);
            if let Some(prev) = p
                .prev
                .take_if(|prev| p.rising && prev.value > 0.0 && prev.value >= value)
            {
                self.candidates[a].push(ExcursionEvent {
                    window: spec.window,
                    number: spec.number,
                    excursion: prev,
                });
            }
            p.rising = prev_value.is_none_or(|v| value > v);
            if value > 0.0 && p.best.as_ref().is_none_or(|b| value > b.value) {
                p.best = Some(now.clone());
            }
            p.prev = Some(now);
            Self::hold(&mut c.runs[a], &mut self.holds[a], &spec, t, sp[a], est[a]);
        }
    }

    fn hold(
        run: &mut Option<Run>,
        out: &mut Vec<Hold>,
        spec: &TrackSpec,
        t: u64,
        sp: f64,
        est: f64,
    ) {
        if let Some(r) = run
            && (sp - r.first).abs() <= HOLD_TOL_DEG
        {
            r.last = t;
            if t - r.start >= us(HOLD_SETTLE_S) {
                r.settled += 1;
                r.sum_sp += sp;
                r.sum_est += est;
            }
            return;
        }
        Self::end_run(run.take(), out, spec);
        let mut r = Run {
            start: t,
            first: sp,
            last: t,
            settled: 0,
            sum_sp: 0.0,
            sum_est: 0.0,
        };
        if us(HOLD_SETTLE_S) == 0 {
            r.settled = 1;
            r.sum_sp = sp;
            r.sum_est = est;
        }
        *run = Some(r);
    }

    fn end_run(run: Option<Run>, out: &mut Vec<Hold>, spec: &TrackSpec) {
        let Some(r) = run else { return };
        if r.first.abs() >= HOLD_MIN_DEG && r.last - r.start >= us(HOLD_MIN_S) && r.settled > 0 {
            let n = r.settled as f64;
            let (sp, est) = (r.sum_sp / n, r.sum_est / n);
            out.push(Hold {
                window: spec.window,
                number: spec.number,
                start_us: r.start,
                duration_us: r.last - r.start,
                mean_setpoint: sp,
                mean_estimate: est,
                settled_error: sp - est,
            });
        }
    }

    fn finish_cur(&mut self) {
        let Some(c) = self.cur.take() else { return };
        let spec = self.specs[c.index].clone();
        let n = c.n as f64;
        let mut excursions = [None, None];
        for (a, mut p) in c.peaks.into_iter().enumerate() {
            if let Some(prev) = p.prev.take_if(|prev| p.rising && prev.value > 0.0) {
                self.candidates[a].push(ExcursionEvent {
                    window: spec.window,
                    number: spec.number,
                    excursion: prev,
                });
            }
            excursions[a] = p.best;
        }
        for (a, run) in c.runs.into_iter().enumerate() {
            Self::end_run(run, &mut self.holds[a], &spec);
        }
        self.done[c.index] = Some(FlightTracking {
            window: spec.window,
            number: spec.number,
            start_us: spec.start_us,
            end_us: spec.end_us,
            records: c.n,
            axes: Some(c.acc.map(|a| AxisError {
                setpoint_min: a.min,
                setpoint_max: a.max,
                mean_error: a.sum / n,
                rms_error: (a.sum_sq / n).sqrt(),
                max_abs_error: a.big,
                max_abs_error_at_us: a.big_at,
            })),
            excursions,
        });
    }

    pub fn finish(mut self) -> TrackingReport {
        self.finish_cur();
        let flights = self
            .specs
            .iter()
            .zip(self.done)
            .map(|(spec, d)| {
                d.unwrap_or(FlightTracking {
                    window: spec.window,
                    number: spec.number,
                    start_us: spec.start_us,
                    end_us: spec.end_us,
                    records: 0,
                    axes: None,
                    excursions: [None, None],
                })
            })
            .collect();
        let separation = us(EXCURSION_SEPARATION_S);
        let events = self.candidates.map(|mut c| {
            c.sort_by(|a, b| {
                b.excursion
                    .value
                    .total_cmp(&a.excursion.value)
                    .then(a.excursion.at_us.cmp(&b.excursion.at_us))
            });
            let mut picked: Vec<ExcursionEvent> = Vec::new();
            for e in c {
                if picked.len() < EVENTS
                    && picked
                        .iter()
                        .all(|p| p.excursion.at_us.abs_diff(e.excursion.at_us) >= separation)
                {
                    picked.push(e);
                }
            }
            picked
        });
        let hold_stats = self.holds.each_ref().map(|h| {
            let mut errs: Vec<f64> = h.iter().map(|x| x.settled_error.abs()).collect();
            errs.sort_by(f64::total_cmp);
            let n = errs.len();
            HoldStats {
                count: n,
                median_abs_error: (n > 0).then(|| (errs[(n - 1) / 2] + errs[n / 2]) / 2.0),
                max_abs_error: errs.last().copied(),
            }
        });
        TrackingReport {
            state: TrackingState::Done,
            other_kind_records: self.other,
            flights,
            events,
            holds: self.holds,
            hold_stats,
        }
    }
}
