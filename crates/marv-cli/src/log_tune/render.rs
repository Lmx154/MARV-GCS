//! The text and the JSON document of a tuning analysis. Both are written from the same `TuneReport`.

use std::fmt::Write as _;

use serde_json::{Value, json};

use super::analysis::{
    AXES, AxisReport, F3, Frf, FrfResult, HfStats, Noise, SAT_BITS, SpanReport, TermStats, Terms,
    TuneReport,
};
use super::events::Stat;
use super::settings::{Id, Settings};

/// Bump when a key of the JSON document is removed or changes meaning.
pub const SCHEMA: u32 = 1;

pub const NEEDS_FIRMWARE: &str = "needs the PIDT and NTCH records, written by the firmware that carries the tuning log records \
     (decision-rpm-notch-20261123, 1B)";

pub fn mode_name(m: u8) -> String {
    match m {
        0 => "Acro".to_string(),
        1 => "Angle".to_string(),
        2 => "Altitude".to_string(),
        3 => "Position".to_string(),
        n => format!("mode {n}"),
    }
}

pub fn line_name(line: usize) -> String {
    match line {
        0 => "line 0 (M4)".to_string(),
        3 => "line 3 (M1)".to_string(),
        n => format!("line {n}"),
    }
}

fn which_missing(r: &TuneReport) -> &'static str {
    match (r.has_pidt, r.has_ntch) {
        (false, false) => "this log has neither PIDT nor NTCH records",
        (false, true) => "this log has no PIDT records",
        (true, false) => "this log has no NTCH records",
        (true, true) => "",
    }
}

fn stat(st: &Option<Stat>, scale: f64, dec: usize, unit: &str) -> String {
    match st {
        None => "not available".to_string(),
        Some(s) => format!(
            "median {:.dec$}{unit} (range {:.dec$} to {:.dec$}{unit}, n = {})",
            s.median * scale,
            s.min * scale,
            s.max * scale,
            s.n
        ),
    }
}

pub fn to_text(r: &TuneReport, st: &Settings) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "Tuning analysis: {}", r.file_name);
    let _ = writeln!(
        s,
        "  size {} bytes, bad bytes {}, tail bytes {}{}",
        r.bytes,
        r.bad_bytes,
        r.tail_bytes,
        if r.time_reversals > 0 {
            format!(
                ", {} PIDT/NTCH records dropped for a time that did not increase",
                r.time_reversals
            )
        } else {
            String::new()
        }
    );
    let count = |n: &str| r.counts.iter().find(|(k, _)| k == n).map_or(0, |(_, v)| *v);
    let _ = writeln!(
        s,
        "  records: PIDT {}, NTCH {}, DMD {}, MOT {}, PARM {}",
        count("PIDT"),
        count("NTCH"),
        count("DMD"),
        count("MOT"),
        count("PARM")
    );
    let _ = match (r.from_s, r.to_s) {
        (None, None) => writeln!(s, "  window: the whole log"),
        (f, t) => writeln!(
            s,
            "  window: from {} to {} (log time, seconds)",
            f.map_or("the start".to_string(), |v| format!("{v} s")),
            t.map_or("the end".to_string(), |v| format!("{v} s"))
        ),
    };
    let _ = writeln!(
        s,
        "  Constants (each is a named constant, overridable by the flag shown; sources: the earlier scripts of \
         flights/20261122-tuning-flight where they define a value):"
    );
    s.push_str(&st.header());
    let _ = writeln!(
        s,
        "  An event is an excursion of one axis's rate target (PIDT Tar) out of the quiet band and back, found from the \
         data alone; the sticks (RCS) are not consulted, so a position-hold correction counts too. The achieved rate is \
         NTCH G (the rate the loop used, after the notch). Times are the log's TimeUS in seconds. No position is read."
    );
    if r.spans.is_empty() {
        let _ = writeln!(
            s,
            "\nNo armed span{} (MOT.Arm; {} in the log).",
            if r.from_s.is_some() || r.to_s.is_some() {
                " in the window"
            } else {
                ""
            },
            r.spans_in_log
        );
    }
    for sp in &r.spans {
        span_text(&mut s, sp, r, st);
    }
    s
}

fn span_text(s: &mut String, sp: &SpanReport, r: &TuneReport, st: &Settings) {
    let _ = writeln!(
        s,
        "\n=== Armed span {}: {:.3} s to {:.3} s, {:.1} s{} ===",
        sp.index,
        sp.start_s,
        sp.end_s,
        sp.end_s - sp.start_s,
        if sp.open_at_end {
            " (the log ends armed)"
        } else {
            ""
        }
    );
    let _ = writeln!(s, "1. Context");
    if sp.window_s.0 != sp.start_s || sp.window_s.1 != sp.end_s {
        let _ = writeln!(
            s,
            "  inside the window: {:.3} s to {:.3} s",
            sp.window_s.0, sp.window_s.1
        );
    }
    let _ = writeln!(
        s,
        "  airborne: {:.1} s{}",
        sp.airborne_s,
        match (sp.airborne_known, sp.thrust_ref_n) {
            (true, Some(t)) => format!(
                " (DMD.Thr at or above {} of the span's 90th percentile, {t:.2} N; the log holds no mass, so a ground-only span cannot be told apart: check this thrust)",
                st.get(Id::AirborneFrac)
            ),
            _ =>
                " (no DMD records: the armed span is used whole, airborne not decided)".to_string(),
        }
    );
    if let Some(why) = &sp.not_analysed {
        let _ = writeln!(s, "  not analysed: {why}");
        return;
    }
    let mut totals: Vec<(u8, f64)> = Vec::new();
    for m in &sp.modes {
        match totals.iter_mut().find(|t| t.0 == m.mode) {
            Some(t) => t.1 += m.end_s - m.start_s,
            None => totals.push((m.mode, m.end_s - m.start_s)),
        }
    }
    let shown: Vec<String> = sp
        .modes
        .iter()
        .take(12)
        .map(|m| format!("{} {:.1}-{:.1} s", mode_name(m.mode), m.start_s, m.end_s))
        .collect();
    let _ = writeln!(
        s,
        "  modes flown (MOT.Mode): {}{}; totals {}",
        shown.join(", "),
        if sp.modes.len() > 12 {
            format!(", and {} more runs", sp.modes.len() - 12)
        } else {
            String::new()
        },
        totals
            .iter()
            .map(|(m, t)| format!("{} {t:.1} s", mode_name(*m)))
            .collect::<Vec<_>>()
            .join(", ")
    );
    match &sp.gains {
        None => {
            let _ = writeln!(s, "  gains in force: not logged: compiled defaults assumed");
        }
        Some(g) => {
            let _ = writeln!(
                s,
                "  gains in force (newest PARM at or before the arming step, {:.3} s):",
                g.at_s
            );
            let line = |names: &[&str]| {
                names
                    .iter()
                    .map(|n| {
                        let v = g.values.iter().find(|(k, _)| k == n).and_then(|x| x.1);
                        format!(
                            "{n} = {}",
                            v.map_or("not logged".to_string(), |v| format!("{v}"))
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            let _ = writeln!(
                s,
                "    attitude: {}",
                line(&["ATC_ANG_RP_P", "ATC_ANG_YAW_P"])
            );
            let _ = writeln!(
                s,
                "    roll rate: {}",
                line(&[
                    "ATC_RAT_RLL_P",
                    "ATC_RAT_RLL_I",
                    "ATC_RAT_RLL_D",
                    "ATC_RAT_RLL_IMAX"
                ])
            );
            let _ = writeln!(
                s,
                "    pitch rate: {}",
                line(&[
                    "ATC_RAT_PIT_P",
                    "ATC_RAT_PIT_I",
                    "ATC_RAT_PIT_D",
                    "ATC_RAT_PIT_IMAX"
                ])
            );
            let _ = writeln!(
                s,
                "    yaw rate: {}",
                line(&["ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_IMAX"])
            );
            let _ = writeln!(s, "    D filter: {}", line(&["ATC_RAT_FLTD"]));
            for (n, t, v) in &g.changed_during {
                let _ = writeln!(
                    s,
                    "    note: {n} was written as {v} at {t:.3} s, inside the span (it takes effect at the next arming)"
                );
            }
        }
    }
    let _ = writeln!(
        s,
        "  RPM notch: {}; RPM_NOTCH_EN {}",
        sp.notch_on_frac
            .map_or("NTCH.On not in this log".to_string(), |f| format!(
                "NTCH.On set in {:.1} % of the airborne steps",
                100.0 * f
            )),
        sp.notch_param.map_or("not logged".to_string(), |v| format!(
            "= {v} (PARM at or before the arming step)"
        ))
    );
    let drops = |d: &Option<super::analysis::Drops>, n: &str| match d {
        Some(d) => format!(
            "{n} {} since the log opened ({} inside the span)",
            d.total, d.within
        ),
        None => format!("{n} not in this log"),
    };
    let _ = writeln!(
        s,
        "  log health: dropped records: {}, {}",
        drops(&sp.fds, "FDS"),
        drops(&sp.dsf, "DSF")
    );
    match &sp.gaps {
        Some(g) => {
            let _ = writeln!(
                s,
                "    step spacing: median {:.0} us, largest {} us over {} steps; {} spacings above GAP_FACTOR x median",
                g.median_us, g.max_us, g.steps, g.gaps
            );
        }
        None => {
            let _ = writeln!(s, "    step spacing: fewer than 2 steps");
        }
    }
    if let Some(fs) = sp.fs_hz {
        let _ = writeln!(
            s,
            "    step rate used: {fs:.2} Hz, {} gap-free airborne run(s)",
            sp.chunks
        );
    }
    match &sp.saturation {
        Some(sat) => {
            let _ = writeln!(
                s,
                "  demand saturation (DMD.Sat over {} airborne records): any bit {:.2} %; {}",
                sat.records,
                100.0 * sat.any,
                SAT_BITS
                    .iter()
                    .zip(sat.bits)
                    .map(|(n, f)| format!("{n} {:.2} %", 100.0 * f))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        None => {
            let _ = writeln!(
                s,
                "  demand saturation: no DMD records in the airborne time"
            );
        }
    }
    // 2 and 3
    let _ = writeln!(
        s,
        "\n2. Excitation found (events: |target| leaves the quiet band and returns)"
    );
    if !(r.has_pidt && r.has_ntch) {
        let _ = writeln!(s, "  not made: {NEEDS_FIRMWARE}; {}.", which_missing(r));
    }
    for (a, name) in AXES.iter().enumerate() {
        let ax = &sp.axes[a];
        if let Some(ex) = &ax.excitation {
            let _ = writeln!(
                s,
                "  {name}: {} excursions of the target; {} usable (rejected: {} peak below MIN_EVENT_PEAK_RADS, {} without PRE_QUIET_S of quiet before, {} with under MIN_TAIL_S after, {} with a saturated demand)",
                ex.excursions,
                ex.usable,
                ex.too_small,
                ex.no_pre_quiet,
                ex.short_tail,
                ex.saturated
            );
            for (t, peak, dur, lobes) in &ax.sizes {
                let _ = writeln!(
                    s,
                    "      at {t:.2} s: peak {peak:+.3} rad/s ({:+.1} deg/s), excursion {dur:.2} s, {lobes} lobe(s)",
                    peak.to_degrees()
                );
            }
        }
    }
    let _ = writeln!(
        s,
        "\n3. Response of the achieved rate to the rate target (per axis, over its usable events; median and range)"
    );
    if !(r.has_pidt && r.has_ntch) {
        let _ = writeln!(s, "  not made: {NEEDS_FIRMWARE}; {}.", which_missing(r));
    }
    for (a, name) in AXES.iter().enumerate() {
        let ax = &sp.axes[a];
        if let Some(rs) = &ax.response {
            let _ = writeln!(s, "  {name} (n = {} events):", rs.n);
            let _ = writeln!(
                s,
                "    cross-correlation lag (apparent delay; includes the lag's own contribution): {}",
                stat(&rs.lag_s, 1e3, 1, " ms")
            );
            let _ = writeln!(
                s,
                "    first-order-plus-dead-time fit, used on {} of {} events (R2 >= FIT_MIN_R2 and no search limit hit; {} hit a limit):",
                rs.fit_used, rs.n, rs.fit_at_limit
            );
            let _ = writeln!(s, "      dead time {}", stat(&rs.dead_s, 1e3, 1, " ms"));
            let _ = writeln!(s, "      time constant {}", stat(&rs.tau_s, 1e3, 1, " ms"));
            let _ = writeln!(s, "      gain {}", stat(&rs.gain, 1.0, 3, ""));
            let _ = writeln!(s, "      R2 of all fits {}", stat(&rs.fit_r2, 1.0, 3, ""));
            let _ = writeln!(
                s,
                "    rise time ({}-{} % of the achieved peak, first lobe): {}",
                100.0 * st.get(Id::RiseLo),
                100.0 * st.get(Id::RiseHi),
                stat(&rs.rise_s, 1e3, 1, " ms")
            );
            let _ = writeln!(
                s,
                "    peak ratio (achieved peak over target peak, first lobe): {}",
                stat(&rs.peak_ratio, 100.0, 0, " %")
            );
            let _ = writeln!(
                s,
                "    held targets (within PLATEAU_TOL of the peak for PLATEAU_MIN_S): {} of {} events; overshoot, settling and steady error use only these",
                rs.held, rs.n
            );
            let _ = writeln!(
                s,
                "    overshoot (peak over the held target level): {}",
                stat(&rs.overshoot_pct, 1.0, 1, " %")
            );
            let _ = writeln!(
                s,
                "    settling to +-SETTLE_TOL of the held level: {}; not settled by the end of the hold in {}",
                stat(&rs.settle_s, 1e3, 0, " ms"),
                rs.not_settled
            );
            let _ = writeln!(
                s,
                "    steady tracking error (target minus achieved, end of the hold): {} ; {}",
                stat(&rs.steady_abs, 1.0, 3, " rad/s"),
                stat(&rs.steady_pct, 1.0, 1, " %")
            );
        } else if let Some(why) = &ax.response_skipped {
            let _ = writeln!(s, "  {name}: not made: {why}");
        }
    }
    // 4
    let _ = writeln!(
        s,
        "\n4. Frequency response of the achieved rate over the rate target (H1 = Sxy / Sxx, Welch {} samples, Hann, 50 % overlap, linear detrend; shown only where coherence >= {})",
        st.count(Id::FrfSeg),
        st.get(Id::Coherence)
    );
    if !(r.has_pidt && r.has_ntch) {
        let _ = writeln!(s, "  not made: {NEEDS_FIRMWARE}; {}.", which_missing(r));
    }
    for (a, name) in AXES.iter().enumerate() {
        match &sp.axes[a].frf {
            Some(Frf::Insufficient(why)) => {
                let _ = writeln!(s, "  {name}: {why}");
            }
            Some(Frf::Done(f)) => frf_text(s, name, f),
            None => {}
        }
    }
    // 5
    let _ = writeln!(s, "\n5. Controller terms (torques, N.m)");
    if !r.has_pidt {
        let _ = writeln!(
            s,
            "  P, I, D, the D torque above the split and the checks: not made: need the PIDT record (the tuning log records, decision-rpm-notch-20261123, 1B); this log has none."
        );
    }
    for (a, name) in AXES.iter().enumerate() {
        let ax = &sp.axes[a];
        if let Some(t) = &ax.terms {
            terms_text(s, name, t, ax, st);
        }
    }
    match &sp.saturation {
        Some(sat) => {
            let _ = writeln!(
                s,
                "  fraction of airborne time with the demand saturated (DMD.Sat, from the DMD records): any bit {:.2} %; per bit {}",
                100.0 * sat.any,
                SAT_BITS
                    .iter()
                    .zip(sat.bits)
                    .map(|(n, f)| format!("{n} {:.2} %", 100.0 * f))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        None => {
            let _ = writeln!(s, "  saturation fraction: no DMD records");
        }
    }
    // 6
    let _ = writeln!(
        s,
        "\n6. Gyro noise (NTCH G over the whole airborne time; the hover-like runs too when long enough) against the rotor fundamentals (NTCH F0..F3)"
    );
    match &sp.noise {
        Err(why) => {
            let _ = writeln!(
                s,
                "  not made: {}",
                if !r.has_ntch {
                    "needs the NTCH record (the tuning log records, decision-rpm-notch-20261123, 1B); this log has none".to_string()
                } else {
                    why.clone()
                }
            );
        }
        Ok(n) => {
            noise_text(s, n, st, "whole airborne time");
            if let Some(h) = &sp.noise_hover {
                let _ = writeln!(s, "  the same over the hover-like runs only:");
                noise_text(s, h, st, "hover-like");
            }
        }
    }
    let _ = writeln!(
        s,
        "\n7. Observations (measured facts only; no gain value or direction is proposed here)"
    );
    if sp.observations.is_empty() {
        let _ = writeln!(
            s,
            "  none: the log does not support a measurement for this span."
        );
    }
    for o in &sp.observations {
        let _ = writeln!(s, "  - {o}");
    }
}

fn frf_text(s: &mut String, name: &str, f: &FrfResult) {
    let _ = writeln!(
        s,
        "  {name}: {} segments (about {:.0} s), bin {:.3} Hz, {} coherent bins; reference gain {}",
        f.segments,
        f.seconds,
        f.df,
        f.coherent_bins,
        f.ref_gain_db
            .map_or("not found".to_string(), |g| format!("{g:.1} dB"))
    );
    for row in &f.rows {
        if row.coherent {
            let _ = writeln!(
                s,
                "      {:6.2} Hz: gain {:+6.1} dB, phase {:+7.1} deg, coherence {:.2}",
                row.f_hz, row.gain_db, row.phase_deg, row.coherence
            );
        } else {
            let _ = writeln!(
                s,
                "      {:6.2} Hz: insufficient excitation (coherence {:.2})",
                row.f_hz, row.coherence
            );
        }
    }
    match &f.f3 {
        Ok(F3 {
            hz,
            phase_lag_deg,
            gain_err,
        }) => {
            let _ = writeln!(
                s,
                "      -3 dB bandwidth {hz:.2} Hz; phase lag there {phase_lag_deg:.1} deg ({:.0} ms as a pure delay); gain error about {:.0} % (1-sigma, from the segment count and the coherence there)",
                phase_lag_deg / 360.0 / hz * 1e3,
                100.0 * gain_err
            );
        }
        Err(why) => {
            let _ = writeln!(s, "      -3 dB bandwidth: not found: {why}");
        }
    }
}

/// A mean that rounds to zero prints as 0, not -0.
fn z(v: f64) -> f64 {
    if v.abs() < 5e-5 { 0.0 } else { v }
}

fn term_line(label: &str, t: &Option<TermStats>) -> String {
    match t {
        None => format!("{label}: no steps"),
        Some(t) => format!(
            "{label} ({} steps, {:.1} s): rms P {:.4}, I {:.4}, D {:.4}; mean I (trim) {:.4}",
            t.steps,
            t.seconds,
            t.rms_p,
            t.rms_i,
            t.rms_d,
            z(t.mean_i)
        ),
    }
}

fn terms_text(s: &mut String, name: &str, t: &Terms, ax: &AxisReport, st: &Settings) {
    let _ = writeln!(s, "  {name}:");
    let _ = writeln!(s, "    {}", term_line("hover-like", &t.hover));
    let _ = writeln!(
        s,
        "    {}",
        term_line("during events (the excursions)", &t.events)
    );
    let hf_line = |h: &HfStats, what: &str| {
        format!(
            "D torque above {} Hz: {:.4} N.m rms; P term above {} Hz: {:.4} N.m rms; D over P {} ({what}, {} segments of {} samples, about {:.0} s)",
            st.get(Id::HfSplit),
            h.rms_d,
            st.get(Id::HfSplit),
            h.rms_p,
            if h.rms_p > 0.0 {
                format!("{:.1}", h.rms_d / h.rms_p)
            } else {
                "undefined (P is 0)".to_string()
            },
            h.segments,
            st.count(Id::NoiseSeg),
            h.seconds
        )
    };
    match &t.hf {
        Ok(h) => {
            let _ = writeln!(s, "    {}", hf_line(h, "whole airborne time"));
        }
        Err(why) => {
            let _ = writeln!(
                s,
                "    D torque above {} Hz: not made: {why}",
                st.get(Id::HfSplit)
            );
        }
    }
    if let Some(h) = &t.hf_hover {
        let _ = writeln!(s, "    {}", hf_line(h, "hover-like"));
    }
    if let Some(f) = ax.sat_axis_frac {
        let _ = writeln!(
            s,
            "    demand saturated on this axis: {:.2} % of the airborne DMD records",
            100.0 * f
        );
    }
    if let Some((n, max)) = t.sum_check {
        let _ = writeln!(
            s,
            "    check: |P + I + D - DMD torque| is at most {max:.2e} N.m over {n} accepted, unsaturated steps"
        );
    }
    if let Some((kp, n)) = t.implied_kp {
        let _ = writeln!(
            s,
            "    check: least-squares P over (target - achieved) = {kp:.4} N.m/(rad/s) over {n} airborne steps"
        );
    }
}

fn noise_text(s: &mut String, n: &Noise, st: &Settings, what: &str) {
    let _ = writeln!(
        s,
        "  {what} data used: {} segments of {} samples (about {:.0} s), bin {:.3} Hz; peaks: the {} highest local maxima from {} Hz, at least {} Hz apart; rms = the band of +-{} bins",
        n.segments,
        st.count(Id::NoiseSeg),
        n.seconds,
        n.df,
        st.count(Id::Peaks),
        st.get(Id::PeakMinHz),
        st.get(Id::PeakSepHz),
        st.count(Id::PeakHalfBins)
    );
    for r in &n.rotors {
        let range = match (r.lo_hz, r.hi_hz) {
            (Some(lo), Some(hi)) => format!(
                "f1 {lo:.1} to {hi:.1} Hz (percentiles {} and {}), 2nd {:.0}-{:.0}, 3rd {:.0}-{:.0} Hz",
                st.get(Id::RotorPct),
                100.0 - st.get(Id::RotorPct),
                2.0 * lo,
                2.0 * hi,
                3.0 * lo,
                3.0 * hi
            ),
            _ => "no valid eRPM (F = 0)".to_string(),
        };
        let _ = writeln!(
            s,
            "  {}: valid in {:.1} % of the {what} steps; {range}",
            super::render::line_name(r.line),
            100.0 * r.valid_frac
        );
    }
    for (a, name) in AXES.iter().enumerate() {
        let ax = &n.axes[a];
        let _ = writeln!(
            s,
            "  gyro {name}: rms above {} Hz {:.4} rad/s; peaks:",
            st.get(Id::HfSplit),
            ax.rms_above
        );
        for p in &ax.peaks {
            let m: Vec<String> = p
                .matches
                .iter()
                .map(|m| {
                    format!(
                        "{} h{}{} {:.0}-{:.0} Hz",
                        line_name(m.line),
                        m.harmonic,
                        if m.folded { " folded" } else { "" },
                        m.lo_hz,
                        m.hi_hz
                    )
                })
                .collect();
            let _ = writeln!(
                s,
                "      {:7.2} Hz  {:.4} rad/s rms  {}",
                p.hz,
                p.rms,
                if m.is_empty() {
                    "inside no rotor harmonic range".to_string()
                } else {
                    format!("inside the range of {}", m.join("; "))
                }
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------

fn jn(v: Option<f64>) -> Value {
    v.filter(|x| x.is_finite())
        .map_or(Value::Null, |x| json!(x))
}

fn jstat(s: &Option<Stat>) -> Value {
    match s {
        None => Value::Null,
        Some(s) => json!({"n": s.n, "median": s.median, "min": s.min, "max": s.max}),
    }
}

pub fn to_json(r: &TuneReport, st: &Settings) -> Value {
    json!({
        "tool": "marv-cli log tune",
        "schema": SCHEMA,
        "file": r.file_name,
        "bytes": r.bytes,
        "bad_bytes": r.bad_bytes,
        "tail_bytes": r.tail_bytes,
        "time_reversals": r.time_reversals,
        "records": r.counts.iter().map(|(k, v)| json!({"type": k, "count": v})).collect::<Vec<_>>(),
        "has_pidt": r.has_pidt,
        "has_ntch": r.has_ntch,
        "has_dmd": r.has_dmd,
        "window": {"from_s": r.from_s, "to_s": r.to_s},
        "constants": st.to_json(),
        "armed_spans_in_log": r.spans_in_log,
        "spans": r.spans.iter().map(|s| span_json(s, r)).collect::<Vec<_>>(),
    })
}

fn span_json(sp: &SpanReport, r: &TuneReport) -> Value {
    let needs: Vec<&str> = {
        let mut n = Vec::new();
        if !(r.has_pidt && r.has_ntch) {
            n.push(
                "sections 2, 3, 4 (excitation, response, frequency response) need PIDT and NTCH",
            );
        }
        if !r.has_pidt {
            n.push("section 5 (P, I, D terms, D above the split) needs PIDT");
        }
        if !r.has_ntch {
            n.push("section 6 (gyro noise against the rotors) needs NTCH");
        }
        n
    };
    json!({
        "index": sp.index,
        "context": {
            "start_s": sp.start_s,
            "end_s": sp.end_s,
            "duration_s": sp.end_s - sp.start_s,
            "open_at_end": sp.open_at_end,
            "window_s": [sp.window_s.0, sp.window_s.1],
            "airborne_s": sp.airborne_s,
            "airborne_decided": sp.airborne_known,
            "airborne_reference_thrust_n": sp.thrust_ref_n,
            "analysed": sp.analysed,
            "not_analysed": sp.not_analysed,
            "modes": sp.modes.iter().map(|m| json!({"mode": mode_name(m.mode), "code": m.mode, "start_s": m.start_s, "end_s": m.end_s})).collect::<Vec<_>>(),
            "gains": match &sp.gains {
                None => json!({"logged": false, "note": "not logged: compiled defaults assumed"}),
                Some(g) => json!({
                    "logged": true,
                    "at_s": g.at_s,
                    "values": g.values.iter().map(|(k, v)| json!({"name": k, "value": jn(*v)})).collect::<Vec<_>>(),
                    "changed_during_span": g.changed_during.iter().map(|(n, t, v)| json!({"name": n, "at_s": t, "value": v})).collect::<Vec<_>>(),
                }),
            },
            "notch": {"ntch_on_fraction": jn(sp.notch_on_frac), "rpm_notch_en": jn(sp.notch_param)},
            "health": {
                "fds_dropped": sp.fds.as_ref().map(|d| json!({"total": d.total, "within_span": d.within})),
                "dsf_dropped": sp.dsf.as_ref().map(|d| json!({"total": d.total, "within_span": d.within})),
                "steps": sp.gaps.as_ref().map(|g| json!({"count": g.steps, "median_spacing_us": g.median_us, "max_spacing_us": g.max_us, "gaps": g.gaps})),
                "step_rate_hz": jn(sp.fs_hz),
                "airborne_runs": sp.chunks,
            },
            "saturation": sp.saturation.as_ref().map(|s| json!({
                "records": s.records,
                "any": s.any,
                "bits": SAT_BITS.iter().zip(s.bits).map(|(n, f)| json!({"bit": n, "fraction": f})).collect::<Vec<_>>(),
            })),
        },
        "needs_newer_firmware": needs,
        "axes": AXES.iter().enumerate().map(|(a, n)| axis_json(n, &sp.axes[a])).collect::<Vec<_>>(),
        "noise": match &sp.noise {
            Ok(n) => {
                let mut v = noise_json(n);
                v["basis"] = json!("whole airborne time");
                v["hover_like"] = sp.noise_hover.as_ref().map_or(Value::Null, noise_json);
                v
            }
            Err(why) => json!({"made": false, "reason": why}),
        },
        "observations": sp.observations,
    })
}

fn axis_json(name: &str, ax: &AxisReport) -> Value {
    json!({
        "axis": name,
        "excitation": ax.excitation.as_ref().map(|e| json!({
            "excursions": e.excursions,
            "usable": e.usable,
            "rejected": {"too_small": e.too_small, "no_pre_quiet": e.no_pre_quiet, "short_tail": e.short_tail, "saturated": e.saturated},
            "events": ax.sizes.iter().map(|(t, p, d, l)| json!({"start_s": t, "peak_rads": p, "excursion_s": d, "lobes": l})).collect::<Vec<_>>(),
        })),
        "response": match (&ax.response, &ax.response_skipped) {
            (Some(r), _) => json!({
                "made": true,
                "events": r.n,
                "cross_correlation_lag_s": jstat(&r.lag_s),
                "fit": {
                    "events_used": r.fit_used,
                    "events_at_search_limit": r.fit_at_limit,
                    "dead_time_s": jstat(&r.dead_s),
                    "time_constant_s": jstat(&r.tau_s),
                    "gain": jstat(&r.gain),
                    "r2_all_events": jstat(&r.fit_r2),
                },
                "rise_time_s": jstat(&r.rise_s),
                "peak_ratio": jstat(&r.peak_ratio),
                "held_events": r.held,
                "overshoot_pct": jstat(&r.overshoot_pct),
                "settling_s": jstat(&r.settle_s),
                "not_settled_events": r.not_settled,
                "steady_error_rads": jstat(&r.steady_abs),
                "steady_error_pct": jstat(&r.steady_pct),
            }),
            (None, Some(why)) => json!({"made": false, "reason": why}),
            (None, None) => json!({"made": false, "reason": "needs PIDT and NTCH"}),
        },
        "frequency_response": match &ax.frf {
            None => json!({"made": false, "reason": "needs PIDT and NTCH"}),
            Some(Frf::Insufficient(why)) => json!({"made": false, "reason": why}),
            Some(Frf::Done(f)) => json!({
                "made": true,
                "segments": f.segments,
                "bin_hz": f.df,
                "coherent_bins": f.coherent_bins,
                "reference_gain_db": jn(f.ref_gain_db),
                "rows": f.rows.iter().map(|w| if w.coherent {
                    json!({"hz": w.f_hz, "coherence": w.coherence, "gain_db": w.gain_db, "phase_deg": w.phase_deg})
                } else {
                    json!({"hz": w.f_hz, "coherence": w.coherence, "insufficient_excitation": true})
                }).collect::<Vec<_>>(),
                "bandwidth_3db": match &f.f3 {
                    Ok(b) => json!({"found": true, "hz": b.hz, "phase_lag_deg": b.phase_lag_deg, "gain_error_1sigma": b.gain_err}),
                    Err(why) => json!({"found": false, "reason": why}),
                },
            }),
        },
        "terms": ax.terms.as_ref().map(|t| json!({
            "hover_like": t.hover.as_ref().map(terms_json),
            "events": t.events.as_ref().map(terms_json),
            "above_split": match &t.hf {
                Ok(h) => json!({"made": true, "rms_d_nm": h.rms_d, "rms_p_nm": h.rms_p, "segments": h.segments}),
                Err(why) => json!({"made": false, "reason": why}),
            },
            "above_split_hover_like": t.hf_hover.as_ref().map(|h| json!({"rms_d_nm": h.rms_d, "rms_p_nm": h.rms_p, "segments": h.segments})),
            "sum_check": t.sum_check.map(|(n, m)| json!({"steps": n, "max_abs_nm": m})),
            "implied_kp": t.implied_kp.map(|(k, n)| json!({"value": k, "steps": n})),
            "demand_saturated_fraction": jn(ax.sat_axis_frac),
        })),
    })
}

fn terms_json(t: &TermStats) -> Value {
    json!({"steps": t.steps, "seconds": t.seconds, "rms_p_nm": t.rms_p, "rms_i_nm": t.rms_i, "rms_d_nm": t.rms_d, "mean_i_nm": t.mean_i})
}

fn noise_json(n: &Noise) -> Value {
    json!({
        "made": true,
        "seconds": n.seconds,
        "segments": n.segments,
        "bin_hz": n.df,
        "rotors": n.rotors.iter().map(|r| json!({
            "line": r.line, "name": line_name(r.line), "valid_fraction": r.valid_frac, "f1_lo_hz": jn(r.lo_hz), "f1_hi_hz": jn(r.hi_hz),
        })).collect::<Vec<_>>(),
        "axes": AXES.iter().enumerate().map(|(a, name)| json!({
            "axis": name,
            "rms_above_split": n.axes[a].rms_above,
            "peaks": n.axes[a].peaks.iter().map(|p| json!({
                "hz": p.hz, "rms": p.rms,
                "rotor_matches": p.matches.iter().map(|m| json!({"line": m.line, "harmonic": m.harmonic, "lo_hz": m.lo_hz, "hi_hz": m.hi_hz, "folded": m.folded})).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
    })
}
