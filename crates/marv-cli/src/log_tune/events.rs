//! Section 2 and 3: the stick-driven events found in the rate target, and what the achieved rate did in each.
//!
//! An event is an excursion of one axis's rate target (PIDT Tar) out of the quiet band and back, found from the
//! data alone. The achieved rate is NTCH G, the rate the loop used.

use super::dsp::median;
use super::settings::{Id, Settings};

/// A run of consecutive steps with no gap, all airborne: indices `lo..hi` into the step arrays.
#[derive(Clone, Copy, Debug)]
pub struct Chunk {
    pub lo: usize,
    pub hi: usize,
}

/// Median and range of a measure over the events that have it.
#[derive(Clone, Copy, Debug)]
pub struct Stat {
    pub n: usize,
    pub median: f64,
    pub min: f64,
    pub max: f64,
}

impl Stat {
    pub fn of(values: &[f64]) -> Option<Stat> {
        let v: Vec<f64> = values.iter().copied().filter(|x| x.is_finite()).collect();
        Some(Stat {
            n: v.len(),
            median: median(&v)?,
            min: v.iter().copied().fold(f64::INFINITY, f64::min),
            max: v.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        })
    }
}

/// First-order-plus-dead-time fit of one event: `y(t) = K x lag(tau)(r(t - dead))`.
#[derive(Clone, Copy, Debug)]
pub struct Fit {
    pub dead_s: f64,
    pub tau_s: f64,
    pub gain: f64,
    /// 1 - SSE / sum of the achieved rate's squares about its baseline.
    pub r2: f64,
    /// The search ended on a limit of the dead time or the time constant.
    pub at_limit: bool,
}

/// One usable event and its measures.
#[derive(Clone, Debug)]
pub struct Event {
    pub start_s: f64,
    pub end_s: f64,
    /// Signed largest target of the excursion, rad/s.
    pub peak_rads: f64,
    /// First and last step index of the excursion in the step arrays.
    pub steps: (usize, usize),
    pub lobes: usize,
    /// Cross-correlation lag of achieved against target over the event window, s.
    pub lag_s: Option<f64>,
    pub fit: Option<Fit>,
    /// First lobe: achieved peak over target peak.
    pub peak_ratio: Option<f64>,
    pub rise_s: Option<f64>,
    /// The first lobe's hold (target within the plateau tolerance of its peak), s; `None`: not held.
    pub held_s: Option<f64>,
    pub overshoot_pct: Option<f64>,
    /// `Some(None)`: held but not settled by the end of the hold.
    pub settle_s: Option<Option<f64>>,
    /// Mean of target minus achieved over the last part of the hold: (rad/s, % of the target).
    pub steady_err: Option<(f64, f64)>,
}

/// What was found on one axis, usable or not.
#[derive(Clone, Debug, Default)]
pub struct Excitation {
    /// Every excursion of the target out of the quiet band.
    pub excursions: usize,
    pub too_small: usize,
    pub no_pre_quiet: usize,
    pub short_tail: usize,
    pub saturated: usize,
    pub usable: usize,
}

/// One chunk of one axis: the target and the achieved rate, and the DMD saturation bits (0xFF unknown).
pub struct AxisChunk {
    /// Time of the chunk's first step, s.
    pub t0: f64,
    /// Index of the chunk's first step in the step arrays.
    pub lo: usize,
    pub r: Vec<f64>,
    pub y: Vec<f64>,
    pub sat: Vec<u8>,
}

fn smooth5(x: &[f64]) -> Vec<f64> {
    (0..x.len())
        .map(|i| {
            let (lo, hi) = (i.saturating_sub(2), (i + 3).min(x.len()));
            x[lo..hi].iter().sum::<f64>() / (hi - lo) as f64
        })
        .collect()
}

/// Cross-correlation lag of `y` behind `r`, s, with parabolic interpolation of the peak.
pub fn xcorr_lag(r: &[f64], y: &[f64], max_lag: usize, dt: f64) -> Option<f64> {
    let n = r.len();
    let (er, ey) = (
        r.iter().map(|v| v * v).sum::<f64>(),
        y.iter().map(|v| v * v).sum::<f64>(),
    );
    if n < 8 || er <= 0.0 || ey <= 0.0 {
        return None;
    }
    let norm = (er * ey).sqrt();
    let corr = |lag: i64| -> f64 {
        let mut s = 0.0;
        for k in 0..n as i64 {
            let j = k + lag;
            if j >= 0 && (j as usize) < n {
                s += r[k as usize] * y[j as usize];
            }
        }
        s / norm
    };
    let max_lag = max_lag.min(n / 2) as i64;
    let lags: Vec<(i64, f64)> = (-3..=max_lag + 1).map(|l| (l, corr(l))).collect();
    let (best, _) = lags
        .iter()
        .enumerate()
        .filter(|(_, (l, _))| *l <= max_lag)
        .max_by(|a, b| a.1.1.total_cmp(&b.1.1))
        .map(|(i, v)| (i, v.1))?;
    if best == 0 || best + 1 >= lags.len() {
        return Some(lags[best].0 as f64 * dt);
    }
    let (a, b, c) = (lags[best - 1].1, lags[best].1, lags[best + 1].1);
    let den = a - 2.0 * b + c;
    let delta = if den < 0.0 {
        (0.5 * (a - c) / den).clamp(-0.5, 0.5)
    } else {
        0.0
    };
    Some((lags[best].0 as f64 + delta) * dt)
}

/// The lag of `r` delayed by `dead` s (a staircase held between samples) through `1/(tau s + 1)`, sampled at the
/// step times, into `out`. Exact for a staircase input.
fn lag_response(r: &[f64], dt: f64, dead: f64, tau: f64, out: &mut [f64]) {
    let m = (dead / dt).floor() as usize;
    let eps = dead / dt - m as f64;
    let e1 = (-eps * dt / tau).exp();
    let e2 = (-(1.0 - eps) * dt / tau).exp();
    let mut y = 0.0;
    for k in 0..r.len() {
        out[k] = y;
        let a = if k > m { r[k - m - 1] } else { 0.0 };
        let b = if k >= m { r[k - m] } else { 0.0 };
        y = y * e1 + a * (1.0 - e1);
        y = y * e2 + b * (1.0 - e2);
    }
}

/// Least-squares fit of dead time, time constant and gain.
pub fn fit_fopdt(r: &[f64], y: &[f64], dt: f64, max_delay: f64) -> Option<Fit> {
    let syy: f64 = y.iter().map(|v| v * v).sum();
    if r.len() < 8 || syy <= 0.0 {
        return None;
    }
    let mut buf = vec![0.0; r.len()];
    let sse = |d: f64, tau: f64, buf: &mut [f64]| -> (f64, f64) {
        lag_response(r, dt, d, tau, buf);
        let (sxy, sxx) = (
            y.iter().zip(buf.iter()).map(|(a, b)| a * b).sum::<f64>(),
            buf.iter().map(|b| b * b).sum::<f64>(),
        );
        if sxx <= 0.0 {
            (syy, 0.0)
        } else {
            (syy - sxy * sxy / sxx, sxy / sxx)
        }
    };
    let tau_lo = (3.0 * dt).max(0.002);
    let tau_hi = 1.5;
    let mut best = (f64::INFINITY, 0.0, 0.0);
    let d_step = 0.010;
    let mut d = 0.0;
    while d <= max_delay + 1e-9 {
        let mut tau = tau_lo;
        while tau <= tau_hi {
            let (e, _) = sse(d, tau, &mut buf);
            if e < best.0 {
                best = (e, d, tau);
            }
            tau *= 1.5;
        }
        d += d_step;
    }
    let (mut d_step, mut ratio) = (d_step, 1.5f64);
    let (mut bd, mut bt) = (best.1, best.2);
    let mut be = best.0;
    for _ in 0..6 {
        for i in -3i32..=3 {
            for j in -3i32..=3 {
                let d = (bd + f64::from(i) * d_step / 2.0).clamp(0.0, max_delay);
                let tau = (bt * ratio.powf(f64::from(j) / 2.0)).clamp(tau_lo / 2.0, tau_hi * 2.0);
                let (e, _) = sse(d, tau, &mut buf);
                if e < be {
                    be = e;
                    (bd, bt) = (d, tau);
                }
            }
        }
        d_step /= 2.0;
        ratio = ratio.sqrt();
    }
    let (e, k) = sse(bd, bt, &mut buf);
    Some(Fit {
        dead_s: bd,
        tau_s: bt,
        gain: k,
        r2: 1.0 - e / syy,
        at_limit: bd <= 1e-9
            || bd >= max_delay - 1e-9
            || bt <= tau_lo / 2.0 + 1e-9
            || bt >= tau_hi * 2.0 - 1e-9,
    })
}

/// Interpolated time (in samples) at which `s` first reaches `level` at or after `from`.
fn first_crossing(s: &[f64], from: usize, level: f64) -> Option<f64> {
    (from.max(1)..s.len())
        .find(|&k| s[k] >= level && s[k - 1] < level)
        .map(|k| {
            let (a, b) = (s[k - 1], s[k]);
            (k - 1) as f64 + (level - a) / (b - a)
        })
}

/// Finds the excursions of one axis in the chunks and measures the usable ones.
///
/// `fs` is the step rate.
pub fn find_events(chunks: &[AxisChunk], fs: f64, st: &Settings) -> (Excitation, Vec<Event>) {
    let band = st.get(Id::QuietBand);
    let hold_n = (st.get(Id::QuietHold) * fs).ceil() as usize;
    let pre_n = (st.get(Id::PreQuiet) * fs).ceil() as usize;
    let tail_n = (st.get(Id::EventTail) * fs).ceil() as usize;
    let min_tail_n = (st.get(Id::MinTail) * fs).ceil() as usize;
    let dt = 1.0 / fs;
    let mut ex = Excitation::default();
    let mut events = Vec::new();
    for AxisChunk { t0, lo, r, y, sat } in chunks {
        let n = r.len();
        let mut spans: Vec<(usize, usize)> = Vec::new();
        let mut i = 0;
        while i < n {
            if r[i].abs() > band {
                let mut last = i;
                let mut j = i + 1;
                while j < n && j - last <= hold_n {
                    if r[j].abs() > band {
                        last = j;
                    }
                    j += 1;
                }
                spans.push((i, last));
                i = last + 1;
            } else {
                i += 1;
            }
        }
        for (e, &(start, end)) in spans.iter().enumerate() {
            ex.excursions += 1;
            let peak = r[start..=end]
                .iter()
                .fold(0.0f64, |m, v| if v.abs() > m.abs() { *v } else { m });
            let wend = (end + tail_n).min(n - 1).min(
                spans
                    .get(e + 1)
                    .map_or(usize::MAX, |s| s.0.saturating_sub(1)),
            );
            if peak.abs() < st.get(Id::MinEventPeak) {
                ex.too_small += 1;
                continue;
            }
            if start < pre_n || r[start - pre_n..start].iter().any(|v| v.abs() > band) {
                ex.no_pre_quiet += 1;
                continue;
            }
            if wend < end + min_tail_n {
                ex.short_tail += 1;
                continue;
            }
            if sat[start - pre_n..=wend]
                .iter()
                .any(|&b| b != 0 && b != 0xFF)
            {
                ex.saturated += 1;
                continue;
            }
            ex.usable += 1;
            let w0 = start - pre_n;
            let (r0, y0) = (
                median(&r[w0..start]).unwrap_or(0.0),
                median(&y[w0..start]).unwrap_or(0.0),
            );
            let rw: Vec<f64> = r[w0..=wend].iter().map(|v| v - r0).collect();
            let yw: Vec<f64> = y[w0..=wend].iter().map(|v| v - y0).collect();
            let max_lag = (st.get(Id::MaxDelay) * fs).round() as usize;
            let mut ev = Event {
                start_s: t0 + start as f64 * dt,
                end_s: t0 + end as f64 * dt,
                peak_rads: peak,
                steps: (lo + start, lo + end),
                lobes: 1,
                lag_s: xcorr_lag(&rw, &yw, max_lag, dt),
                fit: fit_fopdt(&rw, &yw, dt, st.get(Id::MaxDelay)),
                peak_ratio: None,
                rise_s: None,
                held_s: None,
                overshoot_pct: None,
                settle_s: None,
                steady_err: None,
            };
            // Lobes: sign changes of the target beyond the band.
            let mut sign = 0.0f64;
            for v in &r[start..=end] {
                if v.abs() > band && v.signum() != sign {
                    if sign != 0.0 {
                        ev.lobes += 1;
                    }
                    sign = v.signum();
                }
            }
            lobe_measures(&mut ev, &r[start..=end], &yw[pre_n..], &rw[pre_n..], fs, st);
            events.push(ev);
        }
    }
    (ex, events)
}

/// Rise, peak ratio, overshoot, settling and steady error of the first lobe. `r` and `yb`/`rb` start at the excursion's
/// first sample (`yb`, `rb`: baseline removed, the window to its end).
fn lobe_measures(ev: &mut Event, r: &[f64], yb: &[f64], rb: &[f64], fs: f64, st: &Settings) {
    let band = st.get(Id::QuietBand);
    let s = r[0].signum();
    let lobe_end = r.iter().position(|v| s * v < band).unwrap_or(r.len());
    if lobe_end == 0 {
        return;
    }
    let rp = rb[..lobe_end].iter().fold(0.0f64, |m, v| m.max(s * v));
    if rp < st.get(Id::MinEventPeak) {
        return;
    }
    // The achieved peak: up to the longest delay after the lobe's end, ending early when the response, having
    // reached a quarter of the target peak, has fallen to half its running maximum (the release, or the next lobe).
    let ys = smooth5(yb);
    let limit = (lobe_end + (st.get(Id::MaxDelay) * fs) as usize).min(ys.len());
    let (mut run_max, mut stop) = (0.0f64, limit);
    for (k, v) in ys[..limit].iter().enumerate() {
        let sv = s * v;
        if sv > run_max {
            run_max = sv;
        } else if run_max > 0.25 * rp && sv < 0.5 * run_max {
            stop = k;
            break;
        }
    }
    let ysd: Vec<f64> = ys.iter().map(|v| s * v).collect();
    let yp = ysd[..stop]
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    if yp <= 0.0 {
        return;
    }
    ev.peak_ratio = Some(yp / rp);
    let (lo, hi) = (st.get(Id::RiseLo) * yp, st.get(Id::RiseHi) * yp);
    if let Some(k_hi) = first_crossing(&ysd[..stop], 0, hi) {
        let kh = k_hi.floor() as usize;
        let k_lo_idx = (0..=kh).rev().find(|&k| ysd[k] < lo);
        let t_lo = match k_lo_idx {
            Some(k) => first_crossing(&ysd[..stop], k, lo).filter(|t| *t <= k_hi),
            None => None,
        };
        if let Some(t_lo) = t_lo {
            ev.rise_s = Some((k_hi - t_lo) / fs);
        }
    }
    // The hold: the longest run of the lobe with the target within the tolerance of its peak.
    let tol = st.get(Id::PlateauTol);
    let (mut best, mut cur_start) = ((0usize, 0usize), None);
    for k in 0..=lobe_end {
        let inside = rb
            .get(k)
            .is_some_and(|v| k < lobe_end && s * v >= (1.0 - tol) * rp);
        match (inside, cur_start) {
            (true, None) => cur_start = Some(k),
            (false, Some(a)) => {
                if k - a > best.1 - best.0 {
                    best = (a, k);
                }
                cur_start = None;
            }
            _ => {}
        }
    }
    let (a, b) = best;
    if (b - a) as f64 / fs < st.get(Id::PlateauMin) {
        return;
    }
    ev.held_s = Some((b - a) as f64 / fs);
    let hold_level = median(&rb[a..b].iter().map(|v| s * v).collect::<Vec<_>>()).unwrap_or(rp);
    ev.overshoot_pct = Some((100.0 * (yp - hold_level) / hold_level).max(0.0));
    let band_abs = st.get(Id::SettleTol) * hold_level;
    let last_out = (0..b)
        .rev()
        .find(|&k| (ysd[k] - hold_level).abs() > band_abs);
    ev.settle_s = Some(match last_out {
        None => Some(0.0),
        Some(k) if k + 1 >= b => None,
        Some(k) => Some((k + 1) as f64 / fs),
    });
    let len = ((b - a) as f64 * st.get(Id::SteadyFrac)).round().max(1.0) as usize;
    let tail = (b - len)..b;
    let n = tail.len() as f64;
    let (tm, ym) = (
        tail.clone().map(|k| s * rb[k]).sum::<f64>() / n,
        tail.map(|k| s * yb[k]).sum::<f64>() / n,
    );
    ev.steady_err = Some((tm - ym, 100.0 * (tm - ym) / tm));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A step of 1 at sample 40, through a dead time of 33.04 samples and a lag of 48 samples, noiseless.
    fn lagged_step(n: usize, dt: f64, dead: f64, tau: f64) -> (Vec<f64>, Vec<f64>) {
        let r: Vec<f64> = (0..n).map(|k| if k >= 40 { 1.0 } else { 0.0 }).collect();
        let y = (0..n)
            .map(|k| {
                let x = k as f64 * dt - 40.0 * dt - dead;
                if x > 0.0 { 1.0 - (-x / tau).exp() } else { 0.0 }
            })
            .collect();
        (r, y)
    }

    #[test]
    fn the_fit_recovers_a_noiseless_dead_time_and_time_constant() {
        let dt = 1.0 / 800.0;
        let (r, y) = lagged_step(400, dt, 0.0413, 0.06);
        let fit = fit_fopdt(&r, &y, dt, 0.3).expect("a fit");
        assert!((fit.dead_s - 0.0413).abs() < 2e-4, "{fit:?}");
        assert!((fit.tau_s - 0.06).abs() < 6e-4, "{fit:?}");
        assert!(
            (fit.gain - 1.0).abs() < 1e-3 && fit.r2 > 0.9999 && !fit.at_limit,
            "{fit:?}"
        );
    }

    #[test]
    fn the_correlation_lag_lies_between_the_dead_time_and_the_dead_time_plus_the_lag() {
        let dt = 1.0 / 800.0;
        let (r, y) = lagged_step(400, dt, 0.0413, 0.06);
        let lag = xcorr_lag(&r, &y, 240, dt).expect("a lag");
        assert!(lag > 0.0413 && lag < 0.0413 + 0.06, "{lag}");
        // A pure delay of 12 samples is found to a fraction of a sample.
        let y12: Vec<f64> = (0..400).map(|k| if k >= 52 { 1.0 } else { 0.0 }).collect();
        let lag = xcorr_lag(&r, &y12, 240, dt).expect("a lag");
        assert!((lag - 12.0 * dt).abs() < 0.5 * dt, "{lag}");
    }
}
