//! Section 7: the measured facts that bear on tuning, restated in plain words. Each line is a fact with its count;
//! none recommends a value or a direction (the proposal is a separate, human-approved step).

use super::analysis::{AXES, Frf, SAT_BITS, SpanReport};
use super::events::Stat;
use super::settings::{Id, Settings};

fn ms(s: f64) -> String {
    format!("{:.0} ms", s * 1e3)
}

/// "31-44" in the unit the caller names after it.
fn range(st: &Stat, scale: f64, dec: usize) -> String {
    format!("{:.dec$}-{:.dec$}", st.min * scale, st.max * scale)
}

fn z(v: f64) -> f64 {
    if v.abs() < 5e-5 { 0.0 } else { v }
}

fn rate_gain_name(axis: usize) -> &'static str {
    ["ATC_RAT_RLL_P", "ATC_RAT_PIT_P", "ATC_RAT_YAW_P"][axis]
}

pub fn observations(rep: &SpanReport, st: &Settings) -> Vec<String> {
    let mut o = Vec::new();
    for (a, name) in AXES.iter().enumerate() {
        let ax = &rep.axes[a];
        if let Some(r) = &ax.response {
            let lag = r.lag_s.map(|l| {
                format!(
                    "cross-correlation lag {} (n = {}, {} ms)",
                    ms(l.median),
                    l.n,
                    range(&l, 1e3, 0)
                )
            });
            match (&r.dead_s, &r.tau_s) {
                (Some(d), Some(t)) => o.push(format!(
                    "{name}: achieved rate follows the target with {} dead time ({} ms) and a {} first-order time constant ({} ms) (first-order-plus-dead-time fit, n = {} of {} events, R2 median {}); {}",
                    ms(d.median),
                    range(d, 1e3, 0),
                    ms(t.median),
                    range(t, 1e3, 0),
                    r.fit_used,
                    r.n,
                    r.fit_r2.map_or("n/a".to_string(), |v| format!("{:.2}", v.median)),
                    lag.unwrap_or_else(|| "no cross-correlation lag".to_string())
                )),
                _ => o.push(format!(
                    "{name}: {}; the first-order fit passed the quality gate (R2 >= FIT_MIN_R2, not on a search limit) on 0 of {} events",
                    lag.unwrap_or_else(|| "no cross-correlation lag".to_string()),
                    r.n
                )),
            }
            if let Some(p) = &r.peak_ratio {
                o.push(format!(
                    "{name}: achieved rate peaks at {:.0} % of the target peak in the first lobe (median, n = {}, {} %)",
                    100.0 * p.median,
                    p.n,
                    range(p, 100.0, 0)
                ));
            }
            if let Some(v) = &r.overshoot_pct {
                o.push(format!(
                    "{name}: overshoot of a held target {:.1} % (median, n = {} held events, {} %)",
                    v.median,
                    v.n,
                    range(v, 1.0, 1)
                ));
            }
        } else if let Some(why) = &ax.response_skipped {
            o.push(format!("{name}: response measures not made ({why})"));
        }
        match &ax.frf {
            Some(Frf::Done(f)) => match &f.f3 {
                Ok(f3) => o.push(format!(
                    "{name}: the rate response (achieved over target) falls 3 dB at {:.2} Hz with {:.0} deg phase lag there (coherence >= {}; {} segments; gain error about {:.0} % 1-sigma)",
                    f3.hz,
                    f3.phase_lag_deg,
                    st.get(Id::Coherence),
                    f.segments,
                    100.0 * f3.gain_err
                )),
                Err(why) => o.push(format!("{name}: frequency response measured, -3 dB point not found: {why}")),
            },
            Some(Frf::Insufficient(why)) => o.push(format!("{name}: frequency response: {why}")),
            None => {}
        }
        if let Some(t) = &ax.terms {
            if let Ok(hf) = &t.hf {
                let ratio = if hf.rms_p > 0.0 && hf.rms_d > 0.0 {
                    format!(
                        ", {:.1}x the P term ({:.4} N.m)",
                        hf.rms_d / hf.rms_p,
                        hf.rms_p
                    )
                } else {
                    format!(" (P term {:.4} N.m)", hf.rms_p)
                };
                o.push(format!(
                    "{name}: D torque above {} Hz is {:.4} N.m rms{ratio} (hover-like, {} segments of {} samples)",
                    st.get(Id::HfSplit),
                    hf.rms_d,
                    hf.segments,
                    st.count(Id::NoiseSeg)
                ));
            }
            if let Some(h) = &t.hover {
                o.push(format!(
                    "{name}: in hover-like flight ({:.1} s) the mean I term (trim) is {:.4} N.m; rms P {:.4}, I {:.4}, D {:.4} N.m",
                    h.seconds, z(h.mean_i), h.rms_p, h.rms_i, h.rms_d
                ));
            }
            if let (Some((kp, n)), Some(g)) = (t.implied_kp, &rep.gains)
                && let Some((_, Some(v))) = g.values.iter().find(|(k, _)| *k == rate_gain_name(a))
            {
                o.push(format!(
                    "{name}: the P term implies a gain of {kp:.4} N.m/(rad/s) (n = {n} steps) against {} = {v:.4} in force",
                    rate_gain_name(a)
                ));
            }
        }
    }
    if let Some(sat) = &rep.saturation {
        let per: Vec<String> = SAT_BITS
            .iter()
            .zip(sat.bits)
            .map(|(n, f)| format!("{n} {:.2} %", 100.0 * f))
            .collect();
        o.push(format!(
            "demand saturated (any DMD.Sat bit) in {:.2} % of the {} airborne DMD records ({})",
            100.0 * sat.any,
            sat.records,
            per.join(", ")
        ));
    }
    if let Ok(n) = &rep.noise {
        for (a, name) in AXES.iter().enumerate() {
            if let Some(p) = n.axes[a].peaks.first() {
                let near = p
                    .matches
                    .iter()
                    .map(|m| {
                        format!(
                            "h{} of line {} ({:.0}-{:.0} Hz)",
                            m.harmonic, m.line, m.lo_hz, m.hi_hz
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(", ");
                o.push(format!(
                    "gyro {name}: dominant peak {:.1} Hz ({:.4} rad/s rms in the band), {}",
                    p.hz,
                    p.rms,
                    if near.is_empty() {
                        "inside no rotor harmonic range".to_string()
                    } else {
                        format!("inside {near}")
                    }
                ));
            }
        }
    }
    if let Some(f) = rep.notch_on_frac {
        o.push(format!(
            "RPM notch: NTCH.On is set in {:.0} % of the airborne steps{}",
            100.0 * f,
            rep.notch_param.map_or(String::new(), |v| format!(
                "; RPM_NOTCH_EN = {v} in force at arming"
            ))
        ));
    }
    o
}
