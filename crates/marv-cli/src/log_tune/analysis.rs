//! The analysis of one log: armed spans, what was airborne, the events and the measures of sections 1 to 6.
//! Everything here is a number with its count; the words are in `render.rs`.

use super::data::{Armed, GAIN_NAMES, LogData, NOTCH_PARAM, Steps};
use super::dsp::{self, Detrend, Welch, median, percentile};
use super::events::{AxisChunk, Chunk, Event, Excitation, Fit, Stat, find_events};
use super::settings::{FRF_FREQS_HZ, Id, Settings};

pub const AXES: [&str; 3] = ["roll", "pitch", "yaw"];
/// DMD saturation bits (firmware step.rs): roll, pitch, yaw, thrust low, thrust high.
pub const SAT_BITS: [&str; 5] = ["roll", "pitch", "yaw", "thrust_low", "thrust_high"];

pub struct ModeRun {
    pub mode: u8,
    pub start_s: f64,
    pub end_s: f64,
}

pub struct Gains {
    pub at_s: f64,
    /// Each of the 14 gains: the newest PARM value at or before the arming step.
    pub values: Vec<(&'static str, Option<f64>)>,
    /// A gain PARM written during the span (in force from the next arming): name, time, value.
    pub changed_during: Vec<(String, f64, f64)>,
}

pub struct Drops {
    /// Records dropped since the log opened, at the span's end.
    pub total: u64,
    /// Of those, dropped inside the span.
    pub within: u64,
}

pub struct Gaps {
    pub steps: usize,
    pub median_us: f64,
    pub max_us: u64,
    pub gaps: usize,
}

pub struct Saturation {
    pub records: usize,
    pub any: f64,
    pub bits: [f64; 5],
}

pub struct ResponseStats {
    pub n: usize,
    pub lag_s: Option<Stat>,
    pub dead_s: Option<Stat>,
    pub tau_s: Option<Stat>,
    pub gain: Option<Stat>,
    pub fit_r2: Option<Stat>,
    pub fit_used: usize,
    pub fit_at_limit: usize,
    pub rise_s: Option<Stat>,
    pub peak_ratio: Option<Stat>,
    pub held: usize,
    pub overshoot_pct: Option<Stat>,
    pub settle_s: Option<Stat>,
    pub not_settled: usize,
    pub steady_abs: Option<Stat>,
    pub steady_pct: Option<Stat>,
}

pub struct FrfRow {
    pub f_hz: f64,
    pub coherence: f64,
    pub gain_db: f64,
    pub phase_deg: f64,
    pub coherent: bool,
}

pub struct F3 {
    pub hz: f64,
    pub phase_lag_deg: f64,
    pub gain_err: f64,
}

pub struct FrfResult {
    pub segments: usize,
    pub seconds: f64,
    pub df: f64,
    pub rows: Vec<FrfRow>,
    pub coherent_bins: usize,
    pub ref_gain_db: Option<f64>,
    pub f3: Result<F3, String>,
}

pub enum Frf {
    Insufficient(String),
    Done(FrfResult),
}

pub struct TermStats {
    pub steps: usize,
    pub seconds: f64,
    pub rms_p: f64,
    pub rms_i: f64,
    pub rms_d: f64,
    pub mean_i: f64,
}

pub struct HfStats {
    pub seconds: f64,
    pub segments: usize,
    pub rms_d: f64,
    pub rms_p: f64,
}

pub struct Terms {
    pub hover: Option<TermStats>,
    pub events: Option<TermStats>,
    pub hf: Result<HfStats, String>,
    /// Steps and largest |P + I + D - demanded torque| where DMD was accepted and not saturated.
    pub sum_check: Option<(usize, f64)>,
    /// Least-squares slope of P on (target - achieved) over the airborne steps, and the steps used.
    pub implied_kp: Option<(f64, usize)>,
}

pub struct RotorMatch {
    pub line: usize,
    pub harmonic: usize,
    pub lo_hz: f64,
    pub hi_hz: f64,
    pub folded: bool,
}

pub struct PeakInfo {
    pub hz: f64,
    pub rms: f64,
    pub matches: Vec<RotorMatch>,
}

pub struct RotorLine {
    pub line: usize,
    pub valid_frac: f64,
    pub lo_hz: Option<f64>,
    pub hi_hz: Option<f64>,
}

pub struct NoiseAxis {
    pub peaks: Vec<PeakInfo>,
    pub rms_above: f64,
}

pub struct Noise {
    pub seconds: f64,
    pub segments: usize,
    pub df: f64,
    pub axes: [NoiseAxis; 3],
    pub rotors: Vec<RotorLine>,
}

pub struct AxisReport {
    pub excitation: Option<Excitation>,
    /// Sizes of the usable events: (start s, peak rad/s signed, excursion s, lobes).
    pub sizes: Vec<(f64, f64, f64, usize)>,
    pub response: Option<ResponseStats>,
    /// Why the response measures were not made, when they were not.
    pub response_skipped: Option<String>,
    pub frf: Option<Frf>,
    pub terms: Option<Terms>,
    pub sat_axis_frac: Option<f64>,
}

pub struct SpanReport {
    pub index: usize,
    pub start_s: f64,
    pub end_s: f64,
    pub open_at_end: bool,
    /// The part of the span inside --from/--to.
    pub window_s: (f64, f64),
    pub airborne_s: f64,
    pub airborne_known: bool,
    pub thrust_ref_n: Option<f64>,
    pub analysed: bool,
    pub not_analysed: Option<String>,
    pub modes: Vec<ModeRun>,
    pub gains: Option<Gains>,
    pub notch_param: Option<f64>,
    pub notch_on_frac: Option<f64>,
    pub fds: Option<Drops>,
    pub dsf: Option<Drops>,
    pub gaps: Option<Gaps>,
    pub fs_hz: Option<f64>,
    pub chunks: usize,
    pub saturation: Option<Saturation>,
    pub axes: [AxisReport; 3],
    pub noise: Result<Noise, String>,
    pub observations: Vec<String>,
}

pub struct TuneReport {
    pub file_name: String,
    pub bytes: u64,
    pub bad_bytes: u64,
    pub tail_bytes: u64,
    pub time_reversals: u64,
    pub counts: Vec<(String, u64)>,
    pub has_pidt: bool,
    pub has_ntch: bool,
    pub has_dmd: bool,
    pub from_s: Option<f64>,
    pub to_s: Option<f64>,
    pub spans_in_log: usize,
    pub spans: Vec<SpanReport>,
}

fn s(us: u64) -> f64 {
    us as f64 * 1e-6
}

pub fn analyze(
    data: &LogData,
    st: &Settings,
    from_s: Option<f64>,
    to_s: Option<f64>,
) -> TuneReport {
    let from_us = from_s.map_or(0, |v| (v * 1e6).round() as u64);
    let to_us = to_s.map_or(u64::MAX, |v| (v * 1e6).round() as u64);
    let mut spans = Vec::new();
    for (index, armed) in data.armed.iter().enumerate() {
        if armed.end_us <= from_us || armed.start_us >= to_us {
            continue;
        }
        spans.push(analyze_span(data, st, index, armed, from_us, to_us));
    }
    TuneReport {
        file_name: data.file_name.clone(),
        bytes: data.bytes,
        bad_bytes: data.bad_bytes,
        tail_bytes: data.tail_bytes,
        time_reversals: data.time_reversals,
        counts: data.counts.iter().map(|(k, v)| (k.clone(), *v)).collect(),
        has_pidt: data.has_pidt,
        has_ntch: data.has_ntch,
        has_dmd: !data.dmd.is_empty(),
        from_s,
        to_s,
        spans_in_log: data.armed.len(),
        spans,
    }
}

/// Airborne intervals (microseconds, end exclusive) of a span from its DMD thrust, and the reference thrust.
fn airborne(data: &LogData, armed: &Armed, st: &Settings) -> Option<(Vec<(u64, u64)>, f64)> {
    let lo = data.dmd.partition_point(|d| d.t_us < armed.start_us);
    let hi = data.dmd.partition_point(|d| d.t_us < armed.end_us);
    let dmd = &data.dmd[lo..hi];
    if dmd.len() < 2 {
        return None;
    }
    let thr: Vec<f64> = dmd.iter().map(|d| f64::from(d.thrust_n)).collect();
    let reference = percentile(&thr, 90.0)?;
    let level = st.get(Id::AirborneFrac) * reference;
    let dt_us = median(
        &dmd.windows(2)
            .map(|w| (w[1].t_us - w[0].t_us) as f64)
            .collect::<Vec<_>>(),
    )?;
    let merge_us = (st.get(Id::AirborneMerge) * 1e6) as u64;
    let mut runs: Vec<(u64, u64)> = Vec::new();
    for d in dmd {
        if f64::from(d.thrust_n) >= level && level > 0.0 {
            match runs.last_mut() {
                Some(r) if d.t_us <= r.1 + merge_us + dt_us as u64 => r.1 = d.t_us + dt_us as u64,
                _ => runs.push((d.t_us, d.t_us + dt_us as u64)),
            }
        }
    }
    Some((runs, reference))
}

fn analyze_span(
    data: &LogData,
    st: &Settings,
    index: usize,
    armed: &Armed,
    from_us: u64,
    to_us: u64,
) -> SpanReport {
    let (a, b) = (armed.start_us.max(from_us), armed.end_us.min(to_us));
    let (intervals, thrust_ref, airborne_known) = match airborne(data, armed, st) {
        Some((runs, r)) => (runs, Some(r), true),
        None => (vec![(armed.start_us, armed.end_us)], None, false),
    };
    // Clip to the window.
    let intervals: Vec<(u64, u64)> = intervals
        .into_iter()
        .map(|(x, y)| (x.max(a), y.min(b)))
        .filter(|(x, y)| y > x)
        .collect();
    let airborne_s: f64 = intervals.iter().fold(0.0, |acc, (x, y)| acc + s(y - x));
    let mut rep = SpanReport {
        index: index + 1,
        start_s: s(armed.start_us),
        end_s: s(armed.end_us),
        open_at_end: armed.open_at_end,
        window_s: (s(a), s(b)),
        airborne_s,
        airborne_known,
        thrust_ref_n: thrust_ref,
        analysed: false,
        not_analysed: None,
        modes: modes_of(armed, a, b),
        gains: gains_of(data, armed),
        notch_param: parm_at(data, NOTCH_PARAM, armed.start_us),
        notch_on_frac: None,
        fds: drops(&data.fds_drops, armed),
        dsf: drops(&data.dsf_drops, armed),
        gaps: None,
        fs_hz: None,
        chunks: 0,
        saturation: None,
        axes: [empty_axis(), empty_axis(), empty_axis()],
        noise: Err("no airborne step records with a usable spacing".to_string()),
        observations: Vec::new(),
    };
    // Steps of the span inside the window.
    let steps = &data.steps;
    let (lo, hi) = (
        steps.t_us.partition_point(|t| *t < a),
        steps.t_us.partition_point(|t| *t < b),
    );
    rep.gaps = gaps_of(data, lo, hi, a, b, st);
    rep.saturation = saturation_of(data, &intervals);
    if airborne_s < st.get(Id::MinAirborne) {
        rep.not_analysed = Some(if airborne_s == 0.0 {
            "armed, not airborne (no thrust demand at or above the airborne level in the window)"
                .to_string()
        } else {
            format!("airborne {airborne_s:.2} s, below MIN_AIRBORNE_S")
        });
        return rep;
    }
    rep.analysed = true;
    'steps: {
        if hi - lo < 2 || !(data.has_pidt || data.has_ntch) {
            break 'steps;
        }
        let chunks = chunks_of(steps, lo, hi, &intervals, st);
        rep.chunks = chunks.len();
        let n_samples: usize = chunks.iter().map(|c| c.hi - c.lo - 1).sum();
        let seconds: f64 = chunks
            .iter()
            .map(|c| s(steps.t_us[c.hi - 1] - steps.t_us[c.lo]))
            .sum();
        if n_samples == 0 || seconds <= 0.0 {
            break 'steps;
        }
        let fs = n_samples as f64 / seconds;
        rep.fs_hz = Some(fs);
        let on: Vec<f64> = chunks
            .iter()
            .flat_map(|c| steps.notch_on[c.lo..c.hi].iter())
            .filter(|v| **v != 0xFF)
            .map(|v| f64::from(*v))
            .collect();
        if !on.is_empty() {
            rep.notch_on_frac = Some(on.iter().sum::<f64>() / on.len() as f64);
        }

        let both = data.has_pidt && data.has_ntch;
        let band = st.get(Id::QuietBand);
        let hover = if data.has_pidt {
            hover_runs(steps, &chunks, fs, st, band)
        } else {
            Vec::new()
        };
        let mut event_steps: [Vec<(usize, usize)>; 3] = Default::default();
        for (axis, ar) in rep.axes.iter_mut().enumerate() {
            if both {
                let ac: Vec<AxisChunk> =
                    chunks.iter().map(|c| axis_chunk(steps, c, axis)).collect();
                let (ex, events) = find_events(&ac, fs, st);
                ar.sizes = events
                    .iter()
                    .map(|e| (e.start_s, e.peak_rads, e.end_s - e.start_s, e.lobes))
                    .collect();
                event_steps[axis] = events.iter().map(|e| e.steps).collect();
                if events.len() >= st.count(Id::MinEvents) {
                    ar.response = Some(response_stats(&events, st));
                    ar.frf = Some(frf_of(steps, &chunks, axis, fs, st));
                } else {
                    ar.response_skipped = Some(format!(
                        "{} usable event{}, fewer than MIN_EVENTS = {}",
                        events.len(),
                        if events.len() == 1 { "" } else { "s" },
                        st.count(Id::MinEvents)
                    ));
                    ar.frf = Some(Frf::Insufficient(format!(
                        "insufficient excitation ({} usable event{} on this axis, MIN_EVENTS = {})",
                        events.len(),
                        if events.len() == 1 { "" } else { "s" },
                        st.count(Id::MinEvents)
                    )));
                }
                ar.excitation = Some(ex);
            }
            if data.has_pidt {
                ar.terms = Some(terms_of(
                    steps,
                    &chunks,
                    &hover,
                    &event_steps[axis],
                    axis,
                    fs,
                    st,
                ));
            }
            if let Some(sat) = &rep.saturation
                && axis < 3
            {
                ar.sat_axis_frac = Some(sat.bits[axis]);
            }
        }
        rep.noise = if data.has_ntch && data.has_pidt {
            noise_of(steps, &hover, fs, st)
        } else if data.has_ntch {
            Err("needs PIDT: the hover-like spans are found from the rate target".to_string())
        } else {
            Err("needs NTCH".to_string())
        };
    }
    rep.observations = super::observe::observations(&rep, st);
    rep
}

fn empty_axis() -> AxisReport {
    AxisReport {
        excitation: None,
        sizes: Vec::new(),
        response: None,
        response_skipped: None,
        frf: None,
        terms: None,
        sat_axis_frac: None,
    }
}

fn modes_of(armed: &Armed, a: u64, b: u64) -> Vec<ModeRun> {
    let mut runs = Vec::new();
    for (i, (t, mode)) in armed.modes.iter().enumerate() {
        let end = armed.modes.get(i + 1).map_or(armed.end_us, |m| m.0);
        let (x, y) = ((*t).max(a), end.min(b));
        if y > x {
            runs.push(ModeRun {
                mode: *mode,
                start_s: s(x),
                end_s: s(y),
            });
        }
    }
    runs
}

fn parm_at(data: &LogData, name: &str, t_us: u64) -> Option<f64> {
    data.parms
        .iter()
        .rfind(|p| p.name == name && p.t_us <= t_us)
        .map(|p| p.value)
}

fn gains_of(data: &LogData, armed: &Armed) -> Option<Gains> {
    let values: Vec<(&'static str, Option<f64>)> = GAIN_NAMES
        .iter()
        .map(|n| (*n, parm_at(data, n, armed.start_us)))
        .collect();
    if values.iter().all(|v| v.1.is_none()) {
        return None;
    }
    let changed_during = data
        .parms
        .iter()
        .filter(|p| {
            GAIN_NAMES.contains(&p.name.as_str())
                && p.t_us > armed.start_us
                && p.t_us < armed.end_us
        })
        .map(|p| (p.name.clone(), s(p.t_us), p.value))
        .collect();
    Some(Gains {
        at_s: s(armed.start_us),
        values,
        changed_during,
    })
}

fn drops(series: &[(u64, u64)], armed: &Armed) -> Option<Drops> {
    if series.is_empty() {
        return None;
    }
    let at = |t: u64| series.iter().rfind(|(x, _)| *x <= t).map(|v| v.1);
    let end = at(armed.end_us)?;
    let begin = at(armed.start_us).unwrap_or(series[0].1);
    Some(Drops {
        total: end,
        within: end.saturating_sub(begin),
    })
}

fn gaps_of(data: &LogData, lo: usize, hi: usize, a: u64, b: u64, st: &Settings) -> Option<Gaps> {
    let times: Vec<u64> = if hi > lo {
        data.steps.t_us[lo..hi].to_vec()
    } else {
        let l = data.dmd.partition_point(|d| d.t_us < a);
        let h = data.dmd.partition_point(|d| d.t_us < b);
        data.dmd[l..h].iter().map(|d| d.t_us).collect()
    };
    if times.len() < 2 {
        return None;
    }
    let d: Vec<f64> = times.windows(2).map(|w| (w[1] - w[0]) as f64).collect();
    let med = median(&d)?;
    Some(Gaps {
        steps: times.len(),
        median_us: med,
        max_us: d.iter().fold(0.0f64, |m, v| m.max(*v)) as u64,
        gaps: d
            .iter()
            .filter(|v| **v > st.get(Id::GapFactor) * med)
            .count(),
    })
}

fn saturation_of(data: &LogData, intervals: &[(u64, u64)]) -> Option<Saturation> {
    let mut n = 0usize;
    let (mut any, mut bits) = (0usize, [0usize; 5]);
    for d in &data.dmd {
        if intervals.iter().any(|(x, y)| d.t_us >= *x && d.t_us < *y) {
            n += 1;
            any += usize::from(d.sat != 0);
            for (k, c) in bits.iter_mut().enumerate() {
                *c += usize::from(d.sat >> k & 1 != 0);
            }
        }
    }
    (n > 0).then(|| Saturation {
        records: n,
        any: any as f64 / n as f64,
        bits: bits.map(|c| c as f64 / n as f64),
    })
}

/// The runs of consecutive airborne steps without a gap, inside `lo..hi`.
fn chunks_of(
    steps: &Steps,
    lo: usize,
    hi: usize,
    intervals: &[(u64, u64)],
    st: &Settings,
) -> Vec<Chunk> {
    let t = &steps.t_us;
    let d: Vec<f64> = t[lo..hi].windows(2).map(|w| (w[1] - w[0]) as f64).collect();
    let max_dt = median(&d).unwrap_or(0.0) * st.get(Id::GapFactor);
    let in_air = |tt: u64| intervals.iter().any(|(x, y)| tt >= *x && tt < *y);
    let mut out = Vec::new();
    let mut start: Option<usize> = None;
    for k in lo..hi {
        let air = in_air(t[k]);
        let gap = k > lo && (t[k] - t[k - 1]) as f64 > max_dt;
        if start.is_some() && (!air || gap) {
            out.push(Chunk {
                lo: start.take().unwrap_or(k),
                hi: k,
            });
        }
        if air && start.is_none() {
            start = Some(k);
        }
    }
    if let Some(l) = start {
        out.push(Chunk { lo: l, hi });
    }
    out.into_iter().filter(|c| c.hi - c.lo >= 2).collect()
}

fn col(v: &[f32]) -> Vec<f64> {
    v.iter().map(|x| f64::from(*x)).collect()
}

fn axis_chunk(steps: &Steps, c: &Chunk, axis: usize) -> AxisChunk {
    AxisChunk {
        t0: s(steps.t_us[c.lo]),
        lo: c.lo,
        r: col(&steps.tar[axis][c.lo..c.hi]),
        y: col(&steps.g[axis][c.lo..c.hi]),
        sat: steps.sat[c.lo..c.hi].to_vec(),
    }
}

/// Runs of hover-like steps (all three targets in the quiet band for HOVER_SETTLE_S), as step index ranges.
fn hover_runs(
    steps: &Steps,
    chunks: &[Chunk],
    fs: f64,
    st: &Settings,
    band: f64,
) -> Vec<(usize, usize)> {
    let settle = (st.get(Id::HoverSettle) * fs).ceil() as usize;
    let mut runs = Vec::new();
    for c in chunks {
        let mut quiet_run = 0usize;
        let mut start: Option<usize> = None;
        for k in c.lo..c.hi {
            let quiet = (0..3).all(|a| f64::from(steps.tar[a][k]).abs() <= band);
            quiet_run = if quiet { quiet_run + 1 } else { 0 };
            if quiet_run >= settle && quiet {
                start.get_or_insert(k);
            } else if let Some(x) = start.take() {
                runs.push((x, k));
            }
        }
        if let Some(x) = start {
            runs.push((x, c.hi));
        }
    }
    runs
}

fn stat_of(events: &[Event], f: impl Fn(&Event) -> Option<f64>) -> Option<Stat> {
    Stat::of(&events.iter().filter_map(f).collect::<Vec<_>>())
}

fn response_stats(events: &[Event], st: &Settings) -> ResponseStats {
    let min_r2 = st.get(Id::FitMinR2);
    let good = |e: &Event| e.fit.filter(|f| f.r2 >= min_r2 && !f.at_limit);
    let fit_of = |f: fn(&Fit) -> f64| stat_of(events, |e| good(e).map(|x| f(&x)));
    ResponseStats {
        n: events.len(),
        lag_s: stat_of(events, |e| e.lag_s),
        dead_s: fit_of(|f| f.dead_s),
        tau_s: fit_of(|f| f.tau_s),
        gain: fit_of(|f| f.gain),
        fit_r2: stat_of(events, |e| e.fit.map(|f| f.r2)),
        fit_used: events.iter().filter(|e| good(e).is_some()).count(),
        fit_at_limit: events
            .iter()
            .filter(|e| e.fit.is_some_and(|f| f.at_limit))
            .count(),
        rise_s: stat_of(events, |e| e.rise_s),
        peak_ratio: stat_of(events, |e| e.peak_ratio),
        held: events.iter().filter(|e| e.held_s.is_some()).count(),
        overshoot_pct: stat_of(events, |e| e.overshoot_pct),
        settle_s: stat_of(events, |e| e.settle_s.flatten()),
        not_settled: events
            .iter()
            .filter(|e| matches!(e.settle_s, Some(None)))
            .count(),
        steady_abs: stat_of(events, |e| e.steady_err.map(|v| v.0)),
        steady_pct: stat_of(events, |e| e.steady_err.map(|v| v.1)),
    }
}

/// Section 4: H1 = Sxy / Sxx of the achieved rate over the rate target, from every airborne chunk of the span.
fn frf_of(steps: &Steps, chunks: &[Chunk], axis: usize, fs: f64, st: &Settings) -> Frf {
    let nperseg = st.count(Id::FrfSeg);
    let mut w = Welch::new(nperseg, fs, Detrend::Linear);
    for c in chunks {
        let (r, y) = (
            col(&steps.tar[axis][c.lo..c.hi]),
            col(&steps.g[axis][c.lo..c.hi]),
        );
        w.add(&r, Some(&y));
    }
    let need = st.count(Id::FrfMinSegments);
    if w.segments() < need {
        return Frf::Insufficient(format!(
            "insufficient data: {} Welch segment{} of {nperseg} samples ({:.1} s) in the longest gap-free airborne runs, FRF_MIN_SEGMENTS = {need}",
            w.segments(),
            if w.segments() == 1 { "" } else { "s" },
            nperseg as f64 / fs
        ));
    }
    let Some(mut sp) = w.finish(fs) else {
        return Frf::Insufficient("no segment".to_string());
    };
    let smooth = st.count(Id::FrfSmooth);
    smooth_spectra(&mut sp, smooth / 2);
    let thr = st.get(Id::Coherence);
    let nb = sp.sxx.len();
    let h = |k: usize| -> (f64, f64) {
        let (re, im) = sp.sxy[k];
        (re / sp.sxx[k], im / sp.sxx[k])
    };
    let coh = |k: usize| (sp.sxy[k].0.powi(2) + sp.sxy[k].1.powi(2)) / (sp.sxx[k] * sp.syy[k]);
    let k_min = (st.get(Id::FrfMinHz) / sp.df).ceil() as usize;
    let coherent: Vec<bool> = (0..nb)
        .map(|k| k >= k_min.max(1) && sp.sxx[k] > 0.0 && coh(k) >= thr)
        .collect();
    let coherent_bins = coherent.iter().filter(|c| **c).count();
    let seconds = sp.nseg as f64 * (nperseg as f64 / 2.0) / fs + nperseg as f64 / 2.0 / fs;
    let mut seen_bins: Vec<usize> = Vec::new();
    let rows: Vec<FrfRow> = FRF_FREQS_HZ
        .iter()
        .filter_map(|f| {
            let k = (*f / sp.df).round() as usize;
            if seen_bins.contains(&k) {
                return None;
            }
            seen_bins.push(k);
            (k < nb && k >= 1).then(|| {
                let (re, im) = h(k);
                FrfRow {
                    f_hz: k as f64 * sp.df,
                    coherence: coh(k),
                    gain_db: 20.0 * re.hypot(im).log10(),
                    phase_deg: im.atan2(re).to_degrees(),
                    coherent: coherent[k],
                }
            })
        })
        .collect();
    if coherent_bins == 0 {
        return Frf::Insufficient(format!(
            "insufficient excitation: coherence below COHERENCE_MIN = {thr} at every bin ({} segments)",
            sp.nseg
        ));
    }
    let gain = |k: usize| h(k).0.hypot(h(k).1);
    let first = (k_min.max(1)..nb).find(|k| coherent[*k]).unwrap_or(nb - 1);
    let ref_hi = ((st.get(Id::FrfRefMaxHz) / sp.df).floor() as usize).max(first + 1);
    let ref_bins: Vec<f64> = (k_min.max(1)..=ref_hi.min(nb - 1))
        .filter(|k| coherent[*k])
        .map(gain)
        .collect();
    let ref_gain = (ref_bins.len() >= 2).then(|| median(&ref_bins)).flatten();
    let f3 = match ref_gain {
        None => Err(format!(
            "no reference gain: fewer than 2 coherent bins between {} and {} Hz",
            st.get(Id::FrfMinHz),
            st.get(Id::FrfRefMaxHz)
        )),
        Some(rg) => {
            let level = rg / std::f64::consts::SQRT_2;
            let mut found = Err(String::new());
            let mut k = ref_hi.max(k_min.max(1)) + 1;
            let mut last_coherent = k - 1;
            while k < nb {
                if !coherent[k] {
                    break;
                }
                last_coherent = k;
                if gain(k) < level && (k + 1 >= nb || !coherent[k + 1] || gain(k + 1) < level) {
                    let (g0, g1) = (gain(k - 1), gain(k));
                    let frac = if (g0 - g1).abs() > 0.0 {
                        (g0 - level) / (g0 - g1)
                    } else {
                        0.0
                    };
                    let f = (k as f64 - 1.0 + frac.clamp(0.0, 1.0)) * sp.df;
                    let (p0, p1) = (h(k - 1).1.atan2(h(k - 1).0), h(k).1.atan2(h(k).0));
                    let mut dp = p1 - p0;
                    dp -= 2.0 * std::f64::consts::PI * (dp / (2.0 * std::f64::consts::PI)).round();
                    let phase = p0 + frac.clamp(0.0, 1.0) * dp;
                    let g = coh(k).clamp(1e-6, 1.0 - 1e-9);
                    found = Ok(F3 {
                        hz: f,
                        phase_lag_deg: -phase.to_degrees(),
                        gain_err: (1.0 - g).sqrt()
                            / (g.sqrt() * (2.0 * (sp.nseg * smooth) as f64).sqrt()),
                    });
                    break;
                }
                k += 1;
            }
            match found {
                Ok(f) => Ok(f),
                Err(_) => Err(format!(
                    "the gain did not fall 3 dB below its reference inside the coherent band (coherent up to {:.2} Hz)",
                    last_coherent as f64 * sp.df
                )),
            }
        }
    };
    Frf::Done(FrfResult {
        segments: sp.nseg,
        seconds,
        df: sp.df,
        rows,
        coherent_bins,
        ref_gain_db: ref_gain.map(|g| 20.0 * g.log10()),
        f3,
    })
}

/// Moving average of the spectra over `radius` bins each side (the window shrinks at the ends).
fn smooth_spectra(sp: &mut dsp::Spectra, radius: usize) {
    if radius == 0 {
        return;
    }
    let n = sp.sxx.len();
    let avg = |v: &[f64]| -> Vec<f64> {
        (0..n)
            .map(|k| {
                let (lo, hi) = (k.saturating_sub(radius), (k + radius + 1).min(n));
                v[lo..hi].iter().sum::<f64>() / (hi - lo) as f64
            })
            .collect()
    };
    sp.sxx = avg(&sp.sxx);
    sp.syy = avg(&sp.syy);
    let (re, im): (Vec<f64>, Vec<f64>) = sp.sxy.iter().copied().unzip();
    let (re, im) = (avg(&re), avg(&im));
    sp.sxy = re.into_iter().zip(im).collect();
}

fn rms(v: impl Iterator<Item = f32>) -> (usize, f64) {
    let (mut n, mut acc) = (0usize, 0.0);
    for x in v {
        n += 1;
        acc += f64::from(x) * f64::from(x);
    }
    (
        n,
        if n > 0 {
            (acc / n as f64).sqrt()
        } else {
            f64::NAN
        },
    )
}

fn term_stats(steps: &Steps, ranges: &[(usize, usize)], axis: usize, fs: f64) -> Option<TermStats> {
    let idx = || ranges.iter().flat_map(|(a, b)| *a..*b);
    let n = idx().count();
    if n == 0 {
        return None;
    }
    let mean_i = idx().map(|k| f64::from(steps.i[axis][k])).sum::<f64>() / n as f64;
    Some(TermStats {
        steps: n,
        seconds: n as f64 / fs,
        rms_p: rms(idx().map(|k| steps.p[axis][k])).1,
        rms_i: rms(idx().map(|k| steps.i[axis][k])).1,
        rms_d: rms(idx().map(|k| steps.d[axis][k])).1,
        mean_i,
    })
}

fn terms_of(
    steps: &Steps,
    chunks: &[Chunk],
    hover: &[(usize, usize)],
    events: &[(usize, usize)],
    axis: usize,
    fs: f64,
    st: &Settings,
) -> Terms {
    let ev: Vec<(usize, usize)> = events.iter().map(|(a, b)| (*a, *b + 1)).collect();
    // The D and P torque above the split, over the hover-like runs long enough for a segment.
    let nperseg = st.count(Id::NoiseSeg);
    let (mut wd, mut wp) = (
        Welch::new(nperseg, fs, Detrend::Constant),
        Welch::new(nperseg, fs, Detrend::Constant),
    );
    let mut longest = 0usize;
    for (a, b) in hover {
        longest = longest.max(b - a);
        wd.add(&col(&steps.d[axis][*a..*b]), None);
        wp.add(&col(&steps.p[axis][*a..*b]), None);
    }
    let min_seg = st.count(Id::NoiseMinSegments);
    let hf = match (wd.finish(fs), wp.finish(fs)) {
        (Some(d), Some(p)) if d.nseg >= min_seg => Ok(HfStats {
            seconds: d.nseg as f64 * (nperseg as f64 / 2.0) / fs + nperseg as f64 / 2.0 / fs,
            segments: d.nseg,
            rms_d: dsp::rms_above(&d.sxx, d.df, st.get(Id::HfSplit)),
            rms_p: dsp::rms_above(&p.sxx, p.df, st.get(Id::HfSplit)),
        }),
        _ => Err(format!(
            "insufficient hover-like data: {} segment{} of NOISE_SEG = {nperseg} samples ({:.2} s) in the hover-like runs (longest {:.2} s), NOISE_MIN_SEGMENTS = {min_seg}",
            wd.segments(),
            if wd.segments() == 1 { "" } else { "s" },
            nperseg as f64 / fs,
            longest as f64 / fs
        )),
    };
    // P + I + D against the demanded torque, and the gain P implies.
    let (mut sum_n, mut sum_max) = (0usize, 0.0f64);
    let (mut sxe, mut see, mut kn) = (0.0, 0.0, 0usize);
    for c in chunks {
        for k in c.lo..c.hi {
            if steps.dmd_ok[k] == 1 && steps.sat[k] == 0 {
                let total = f64::from(steps.p[axis][k])
                    + f64::from(steps.i[axis][k])
                    + f64::from(steps.d[axis][k]);
                let d = (total - f64::from(steps.dmd_torque[axis][k])).abs();
                if d.is_finite() {
                    sum_n += 1;
                    sum_max = sum_max.max(d);
                }
            }
            let e = f64::from(steps.tar[axis][k]) - f64::from(steps.g[axis][k]);
            let p = f64::from(steps.p[axis][k]);
            if e.is_finite() && p.is_finite() {
                sxe += p * e;
                see += e * e;
                kn += 1;
            }
        }
    }
    Terms {
        hover: term_stats(steps, hover, axis, fs),
        events: term_stats(steps, &ev, axis, fs),
        hf,
        sum_check: (sum_n > 0).then_some((sum_n, sum_max)),
        implied_kp: (kn > 0 && see > 0.0).then(|| (sxe / see, kn)),
    }
}

fn fold(f: f64, fs: f64) -> f64 {
    (f - fs * (f / fs).round()).abs()
}

/// Section 6: the dominant peaks of the achieved rate over the hover-like runs, against the rotor fundamentals.
fn noise_of(
    steps: &Steps,
    hover: &[(usize, usize)],
    fs: f64,
    st: &Settings,
) -> Result<Noise, String> {
    let nperseg = st.count(Id::NoiseSeg);
    let mut w: Vec<Welch> = (0..3)
        .map(|_| Welch::new(nperseg, fs, Detrend::Constant))
        .collect();
    let mut longest = 0usize;
    for (a, b) in hover {
        longest = longest.max(b - a);
        for (axis, wa) in w.iter_mut().enumerate() {
            wa.add(&col(&steps.g[axis][*a..*b]), None);
        }
    }
    let min_seg = st.count(Id::NoiseMinSegments);
    let spectra: Option<Vec<_>> = w.iter().map(|x| x.finish(fs)).collect();
    let Some(spectra) = spectra.filter(|v| v[0].nseg >= min_seg) else {
        return Err(format!(
            "insufficient hover-like data: {} segment{} of NOISE_SEG = {nperseg} samples ({:.2} s) in the hover-like runs (longest {:.2} s), NOISE_MIN_SEGMENTS = {min_seg}",
            w[0].segments(),
            if w[0].segments() == 1 { "" } else { "s" },
            nperseg as f64 / fs,
            longest as f64 / fs
        ));
    };
    let pct = st.get(Id::RotorPct);
    let rotors: Vec<RotorLine> = (0..4)
        .map(|line| {
            let all: Vec<f64> = hover
                .iter()
                .flat_map(|(a, b)| steps.f[line][*a..*b].iter())
                .map(|v| f64::from(*v))
                .collect();
            let valid: Vec<f64> = all
                .iter()
                .copied()
                .filter(|v| *v > 0.0 && v.is_finite())
                .collect();
            RotorLine {
                line,
                valid_frac: if all.is_empty() {
                    0.0
                } else {
                    valid.len() as f64 / all.len() as f64
                },
                lo_hz: percentile(&valid, pct),
                hi_hz: percentile(&valid, 100.0 - pct),
            }
        })
        .collect();
    let df = spectra[0].df;
    let axes: Vec<NoiseAxis> = spectra
        .iter()
        .map(|sp| NoiseAxis {
            peaks: dsp::peaks(
                &sp.sxx,
                sp.df,
                st.get(Id::PeakMinHz),
                st.count(Id::Peaks),
                st.get(Id::PeakSepHz),
                st.count(Id::PeakHalfBins),
            )
            .into_iter()
            .map(|p| PeakInfo {
                hz: p.hz,
                rms: p.rms,
                matches: rotor_matches(p.hz, &rotors, df, fs),
            })
            .collect(),
            rms_above: dsp::rms_above(&sp.sxx, sp.df, st.get(Id::HfSplit)),
        })
        .collect();
    let [x, y, z]: [NoiseAxis; 3] = axes.try_into().map_err(|_| "axes".to_string())?;
    Ok(Noise {
        seconds: spectra[0].nseg as f64 * (nperseg as f64 / 2.0) / fs + nperseg as f64 / 2.0 / fs,
        segments: spectra[0].nseg,
        df,
        axes: [x, y, z],
        rotors,
    })
}

/// The rotor harmonics (1 to 3) whose range, widened by one bin, contains the peak.
fn rotor_matches(hz: f64, rotors: &[RotorLine], df: f64, fs: f64) -> Vec<RotorMatch> {
    let mut out = Vec::new();
    for r in rotors {
        let (Some(lo), Some(hi)) = (r.lo_hz, r.hi_hz) else {
            continue;
        };
        for h in 1..=3usize {
            let (a, b) = (h as f64 * lo, h as f64 * hi);
            let folded = b > fs / 2.0;
            let (a, b) = if folded {
                (fold(a, fs).min(fold(b, fs)), fold(a, fs).max(fold(b, fs)))
            } else {
                (a, b)
            };
            if hz >= a - df && hz <= b + df {
                out.push(RotorMatch {
                    line: r.line,
                    harmonic: h,
                    lo_hz: a,
                    hi_hz: b,
                    folded,
                });
            }
        }
    }
    out
}
