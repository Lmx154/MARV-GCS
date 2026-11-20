//! `marv-cli log report`: the flight report of a DataFlash `.BIN` log, as text and optionally as JSON.
//! The analysis is `gcs_dflog::flight_report`; this file only prints.

use std::fmt::Write as _;
use std::io::Write;

use gcs_dflog::{
    FLIGHT_END_R, FLIGHT_START_R, Flight, FlightReport, GROUND_RUN_MARGIN, IntervalKind,
    IntervalVibration, MIN_FLIGHT_BINS, MIN_GROUND_RUN_BINS, MIN_SPECTRUM_RATE_HZ, MotorFit,
    MotorFitResult, NARROW_LEVEL_RANGE, ONE_G_WINDOW, PolesSource, ReportInputs, SEGMENT,
    VibrationState, flight_report,
};
use serde_json::{Value, json};

use crate::args::LogCommand;
use crate::exit;

/// Said next to the identification: the line and the constant hold only where they were measured.
pub const VALIDITY: &str =
    "valid only for the level range and the battery voltage seen in these bins";

/// Said wherever the rotor speed is needed and the pole count is not known.
pub const NEEDS_POLES: &str = "give --poles: the motor's magnet count";

pub fn run(command: &LogCommand, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let LogCommand::Report {
        file,
        mass_kg,
        kf,
        poles,
        accel_lpf_hz,
        json,
    } = command;
    let inputs = ReportInputs {
        mass_kg: *mass_kg,
        kf: *kf,
        poles: *poles,
        accel_lpf_hz: accel_lpf_hz.clone(),
    };
    let report = match flight_report(file, &inputs) {
        Ok(report) => report,
        Err(e) => {
            let _ = writeln!(
                err,
                "marv-cli: log report of {} failed: {e}",
                file.display()
            );
            return exit::FAILED;
        }
    };
    if let Some(path) = json {
        let text = serde_json::to_string_pretty(&to_json(&report)).expect("a JSON value");
        let written = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .and_then(|mut f| writeln!(f, "{text}"));
        if let Err(e) = written {
            let _ = writeln!(err, "marv-cli: cannot write {}: {e}", path.display());
            return exit::FAILED;
        }
    }
    if out.write_all(to_text(&report).as_bytes()).is_err() {
        return exit::FAILED;
    }
    exit::OK
}

fn secs(us: u64) -> String {
    format!("{:.3} s", us as f64 / 1e6)
}

fn opt(v: Option<f64>, decimals: usize, unit: &str, missing: &str) -> String {
    match v {
        Some(x) => format!("{x:.decimals$}{unit}"),
        None => missing.to_string(),
    }
}

fn gap(present: bool, name: &str) -> String {
    match (present, name.is_empty()) {
        (true, _) => "no records".to_string(),
        (false, true) => "not in this log".to_string(),
        (false, false) => format!("not in this log ({name})"),
    }
}

pub fn to_text(r: &FlightReport) -> String {
    let mut s = String::new();
    let p = &r.present;
    let _ = writeln!(
        s,
        "Flight-log report: {}",
        r.file_name.as_deref().unwrap_or("(stream)")
    );
    let _ = writeln!(
        s,
        "  size {} bytes, bad bytes {}, tail bytes {}, duration {}",
        r.bytes,
        r.bad_bytes,
        r.tail_bytes,
        r.duration_us().map_or("unknown".to_string(), secs)
    );
    let poles = match (r.poles, r.poles_source) {
        (Some(p), PolesSource::Parameter) => format!("poles {p} (from the log's MOT_POLES)"),
        (Some(p), _) => format!("poles {p} (from --poles)"),
        (None, _) => format!("poles unknown ({NEEDS_POLES}; no MOT_POLES in the log)"),
    };
    let _ = writeln!(
        s,
        "  inputs: mass {} kg, kf {:e} N/(rad/s)^2, {poles}",
        r.mass_kg, r.kf
    );

    let h = &r.health;
    let _ = writeln!(s, "\nHealth");
    let drops = [(h.dropped_fds, "FDS", p.fds), (h.dropped_dsf, "DSF", p.dsf)];
    let _ = writeln!(
        s,
        "  dropped records: {}; total {}",
        drops
            .iter()
            .map(|(v, n, present)| format!(
                "{n} {}",
                v.map_or_else(|| gap(*present, ""), |x| x.to_string())
            ))
            .collect::<Vec<_>>()
            .join(", "),
        h.dropped_fds.unwrap_or(0) + h.dropped_dsf.unwrap_or(0)
    );
    let _ = writeln!(
        s,
        "  least free bytes of the ring: FDS {}, DSF {}",
        h.least_free_fds
            .map_or_else(|| gap(p.fds, ""), |x| x.to_string()),
        h.least_free_dsf
            .map_or_else(|| gap(p.dsf, ""), |x| x.to_string())
    );
    let _ = writeln!(
        s,
        "  largest NAV spacing while armed: {}",
        h.max_nav_gap_us
            .map_or_else(|| gap(p.nav, "NAV"), |x| format!("{x} us"))
    );
    let _ = writeln!(
        s,
        "  DMD records with Sat != 0 inside the flights: {}",
        if r.poles.is_none() {
            NEEDS_POLES.to_string()
        } else if p.dmd && p.esc {
            h.saturated_airborne.to_string()
        } else {
            "not in this log (DMD and ESC are needed)".to_string()
        }
    );
    let temps: Vec<String> = h
        .esc_temp_max
        .iter()
        .enumerate()
        .map(|(m, t)| format!("M{m} {}", opt(*t, 0, " degC", &gap(p.esc, "ESC"))))
        .collect();
    let _ = writeln!(s, "  highest ESC temperature: {}", temps.join(", "));
    let _ = writeln!(
        s,
        "  lowest battery voltage: {}",
        opt(h.battery_min_v, 2, " V", &gap(p.bat, "BAT"))
    );

    let _ = writeln!(s, "\nArmed windows (from MOT.Arm)");
    if !p.mot {
        let _ = writeln!(s, "  not in this log (MOT)");
    } else if r.windows.is_empty() {
        let _ = writeln!(s, "  never armed");
    }
    for (i, w) in r.windows.iter().enumerate() {
        let _ = writeln!(
            s,
            "  {}: {} to {}, {}{}",
            i + 1,
            secs(w.start_us),
            secs(w.end_us),
            secs(w.end_us - w.start_us),
            if w.open_at_end {
                " (the log ends armed)"
            } else {
                ""
            }
        );
    }

    let _ = writeln!(
        s,
        "\nFlights (bins of 0.5 s; r = kf x sum of speed^2 / (mass x g); a flight starts at the first bin with \
         r >= {FLIGHT_START_R} and ends before the first later bin with r < {FLIGHT_END_R} or at the end of the \
         armed window; its last bin is dropped; at least {MIN_FLIGHT_BINS} bins)"
    );
    if r.poles.is_none() {
        let _ = writeln!(s, "  {NEEDS_POLES}");
    } else if !(p.mot && p.esc) {
        let _ = writeln!(
            s,
            "  not in this log ({})",
            if p.mot { "ESC" } else { "MOT" }
        );
    }
    for (i, w) in r.windows.iter().enumerate() {
        if !p.esc || r.poles.is_none() {
            break;
        }
        if w.flights.is_empty() {
            let _ = writeln!(s, "  window {}: armed, not airborne", i + 1);
        }
        for (j, f) in w.flights.iter().enumerate() {
            flight_text(&mut s, i + 1, j + 1, f, r);
        }
    }

    let _ = writeln!(s, "\nMotor identification (all armed time)");
    match (&r.identification.motors, r.poles) {
        (_, None) => {
            let _ = writeln!(s, "  {NEEDS_POLES}");
        }
        (None, _) => {
            let _ = writeln!(
                s,
                "  not in this log ({})",
                if p.mot { "ESC" } else { "MOT" }
            );
        }
        (Some(motors), _) => {
            let volts = |f: &MotorFit| opt(f.mean_volts, 2, " V", &gap(p.bat, "BAT"));
            for (m, fit) in motors.iter().enumerate() {
                match fit {
                    MotorFitResult::Fit(f) => {
                        let _ = writeln!(
                            s,
                            "  M{m}: speed = {:.3} + {:.5} x level rad/s; rms residual {:.3} rad/s; {} bins; \
                             level {:.0} to {:.0} (idle {:.0}); mean battery {}{}",
                            f.a,
                            f.b,
                            f.rms_residual,
                            f.bins,
                            f.level_min,
                            f.level_max,
                            f.idle_level,
                            volts(f),
                            ill(f)
                        );
                    }
                    MotorFitResult::NotEnough(n) => {
                        let _ = writeln!(
                            s,
                            "  M{m}: not enough steady bins above idle ({n}; at least three, at two or more levels, are needed)"
                        );
                    }
                }
            }
            match &r.identification.pooled {
                Some(MotorFitResult::Fit(f)) => {
                    let _ = writeln!(
                        s,
                        "  all four motors pooled: speed = {:.3} + {:.5} x level rad/s; rms residual {:.3} rad/s; \
                         {} points; level {:.0} to {:.0}; mean battery {}{}",
                        f.a,
                        f.b,
                        f.rms_residual,
                        f.bins,
                        f.level_min,
                        f.level_max,
                        volts(f),
                        ill(f)
                    );
                }
                Some(MotorFitResult::NotEnough(n)) => {
                    let _ = writeln!(s, "  all four motors pooled: not enough steady bins ({n})");
                }
                None => {}
            }
        }
    }
    match (&r.identification.thrust, r.poles) {
        (_, None) => {
            let _ = writeln!(s, "  thrust constant k: {NEEDS_POLES}");
        }
        (Some(k), _) => {
            let _ = writeln!(
                s,
                "  thrust constant (mass x g over the time-mean of the sum of speed^2 over a whole flight; \
                 pooled over the flights, weighted by duration):"
            );
            let _ = writeln!(
                s,
                "    round 1, flights bounded with the given kf {:e}: flights {}, k1 = {:.4e} N/(rad/s)^2",
                r.kf, k.flights_round1, k.k1
            );
            match k.k2 {
                Some(k2) => {
                    let _ = writeln!(
                        s,
                        "    round 2, flights bounded with k1 (the flights above): k2 = {k2:.4e} N/(rad/s)^2 \
                         (given kf {:e}, ratio {:.3}); per flight {}; spread {:.2} %",
                        r.kf,
                        k2 / r.kf,
                        k.per_flight
                            .iter()
                            .map(|x| format!("{x:.4e}"))
                            .collect::<Vec<_>>()
                            .join(", "),
                        100.0 * (k.flights_max - k.flights_min) / k2
                    );
                }
                None => {
                    let _ = writeln!(s, "    round 2, flights bounded with k1: no flight");
                }
            }
            if k.bounds_moved {
                let _ = writeln!(
                    s,
                    "    a flight bound moved by more than one bin between the rounds (or the number of \
                     flights changed): the constant depends on the given kf here"
                );
            }
        }
        (None, _) => {
            let _ = writeln!(
                s,
                "  thrust constant k: {}",
                if p.esc {
                    "no flight"
                } else {
                    "not in this log (ESC)"
                }
            );
        }
    }
    let _ = writeln!(s, "  These are {VALIDITY}.");

    let _ = writeln!(
        s,
        "\nVibration (IMU instance 0; each flight, and each ground run: armed, not in a flight, the mean level \
         of the motors at least {GROUND_RUN_MARGIN} levels above the idle plateau for at least {MIN_GROUND_RUN_BINS} bins)"
    );
    match r.vibration.state {
        VibrationState::NeedsPoles => {
            let _ = writeln!(s, "  {NEEDS_POLES}");
        }
        VibrationState::NoImu => {
            let _ = writeln!(s, "  not in this log (IMU)");
        }
        VibrationState::Done => {
            if r.vibration.intervals.is_empty() {
                let _ = writeln!(s, "  no flight and no ground run");
            }
            for v in &r.vibration.intervals {
                vibration_text(&mut s, v);
            }
        }
    }
    s
}

fn ill(f: &MotorFit) -> String {
    if f.ill_conditioned {
        format!(
            "; ill-conditioned: narrow level range (under {:.0} % of the mean level)",
            100.0 * NARROW_LEVEL_RANGE
        )
    } else {
        String::new()
    }
}

fn vibration_text(s: &mut String, v: &IntervalVibration) {
    let what = match v.kind {
        IntervalKind::Flight => "flight",
        IntervalKind::GroundRun => "ground run",
    };
    let _ = writeln!(
        s,
        "  window {} {what} {}: {} to {}, {} IMU samples{}",
        v.window + 1,
        v.number,
        secs(v.start_us),
        secs(v.end_us),
        v.samples,
        v.rate_hz
            .map_or(String::new(), |f| format!(" at {f:.1} Hz (median spacing)"))
    );
    if v.samples == 0 {
        let _ = writeln!(s, "    no IMU samples");
        return;
    }
    let _ = writeln!(
        s,
        "    std: gyro (rad/s) X {:.4} Y {:.4} Z {:.4}; accelerometer (m/s^2) X {:.4} Y {:.4} Z {:.4}; \
         accelerometer magnitude {:.4}",
        v.gyro_std[0],
        v.gyro_std[1],
        v.gyro_std[2],
        v.accel_std[0],
        v.accel_std[1],
        v.accel_std[2],
        v.accel_mag_std
    );
    match (&v.spectra, v.too_slow) {
        (_, true) => {
            let _ = writeln!(
                s,
                "    sample rate {:.1} Hz: too slow for spectra (below {MIN_SPECTRUM_RATE_HZ} Hz)",
                v.rate_hz.unwrap_or(0.0)
            );
        }
        (None, false) => {
            let _ = writeln!(
                s,
                "    fewer than {SEGMENT} samples (or no rate): no spectra"
            );
        }
        (Some(sp), false) => {
            let _ = writeln!(
                s,
                "    Welch spectra: {} segments of {SEGMENT} samples, Hann, 50 % overlap",
                sp.segments
            );
            match (v.rotor_hz, &sp.bands) {
                (Some((lo, hi)), Some(b)) => {
                    let _ = writeln!(
                        s,
                        "    rotor frequency of the four motors (mean): {lo:.1} to {hi:.1} Hz; band (10 % either \
                         side) {:.1} to {:.1} Hz{}; twice the band {:.1} to {:.1} Hz{}",
                        b[0].lo_hz,
                        b[0].hi_hz,
                        folded_text(&b[0]),
                        b[1].lo_hz,
                        b[1].hi_hz,
                        folded_text(&b[1])
                    );
                }
                _ => {
                    let _ = writeln!(s, "    rotor band: no ESC records in this interval");
                }
            }
            for (name, (axis, unit)) in [
                "gyro X", "gyro Y", "gyro Z", "accel X", "accel Y", "accel Z",
            ]
            .iter()
            .zip(
                sp.axes
                    .iter()
                    .zip(["rad/s"; 3].into_iter().chain(["m/s^2"; 3])),
            ) {
                let peaks: Vec<String> = axis
                    .peaks
                    .iter()
                    .map(|p| format!("{:.1} Hz (rms {:.4} {unit})", p.hz, p.rms))
                    .collect();
                let share = axis.band_share.map_or(String::new(), |b| {
                    format!(
                        "; variance in the band {:.3}, in twice the band {:.3}",
                        b[0], b[1]
                    )
                });
                let _ = writeln!(s, "    {name}: peaks {}{share}", peaks.join(", "));
            }
        }
    }
    if let Some(g) = &v.one_g {
        let mut line = format!(
            "    accelerometer magnitude within {} g to {} g: raw {:.3}",
            ONE_G_WINDOW.0, ONE_G_WINDOW.1, g.raw
        );
        for (fc, share) in &g.filtered {
            let _ = write!(line, "; low-passed at {fc} Hz {share:.3}");
        }
        let _ = writeln!(s, "{line}");
    }
}

fn folded_text(b: &gcs_dflog::RotorBand) -> String {
    if b.folded {
        format!(
            " (folded: seen at {:.1} to {:.1} Hz)",
            b.shown_lo_hz, b.shown_hi_hz
        )
    } else {
        String::new()
    }
}

fn flight_text(s: &mut String, window: usize, number: usize, seg: &Flight, r: &FlightReport) {
    let p = &r.present;
    let _ = writeln!(
        s,
        "  window {window} flight {number}: {} to {}, duration {}",
        secs(seg.start_us),
        secs(seg.end_us),
        secs(seg.end_us - seg.start_us)
    );
    let _ = writeln!(
        s,
        "    throttle stick (RCS.T) mean {}",
        opt(seg.stick_mean, 3, "", &gap(p.rcs, "RCS"))
    );
    match (seg.collective_n, seg.collective_fraction) {
        (Some(n), Some(f)) => {
            let _ = writeln!(
                s,
                "    commanded collective (SETP.Thr) mean {n:.3} N, {f:.3} of mass x g"
            );
        }
        _ => {
            let _ = writeln!(
                s,
                "    commanded collective (SETP.Thr): {}",
                gap(p.setp, "SETP")
            );
        }
    }
    let _ = writeln!(
        s,
        "    mean rpm per motor: {}",
        seg.rpm
            .iter()
            .map(|x| format!("{x:.0}"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let _ = writeln!(
        s,
        "    kf x sum of speed^2 over mass x g: {:.3}",
        seg.thrust_ratio
    );
    let bat = gap(p.bat, "BAT");
    let _ = writeln!(
        s,
        "    battery: min {}, mean current {}, max current {}",
        opt(seg.volts_min, 2, " V", &bat),
        opt(seg.amps_mean, 2, " A", &bat),
        opt(seg.amps_max, 2, " A", &bat)
    );
    let dmd = gap(p.dmd, "DMD");
    let _ = writeln!(
        s,
        "    mean DMD torque: TX {}, TY {}, TZ {}",
        opt(seg.torque[0], 4, "", &dmd),
        opt(seg.torque[1], 4, "", &dmd),
        opt(seg.torque[2], 4, "", &dmd)
    );
}

fn jn(v: Option<f64>) -> Value {
    v.map_or(Value::Null, |x| json!(x))
}

pub fn to_json(r: &FlightReport) -> Value {
    let p = &r.present;
    let h = &r.health;
    let missing: Vec<&str> = [
        (p.nav, "NAV"),
        (p.setp, "SETP"),
        (p.dmd, "DMD"),
        (p.mot, "MOT"),
        (p.esc, "ESC"),
        (p.rcs, "RCS"),
        (p.bat, "BAT"),
        (p.fds, "FDS"),
        (p.dsf, "DSF"),
        (p.imu, "IMU"),
    ]
    .into_iter()
    .filter(|(present, _)| !present)
    .map(|(_, n)| n)
    .collect();
    let windows: Vec<Value> = r
        .windows
        .iter()
        .map(|w| {
            let flights: Vec<Value> = w
                .flights
                .iter()
                .map(|g| {
                    json!({
                        "start_us": g.start_us,
                        "end_us": g.end_us,
                        "duration_s": (g.end_us - g.start_us) as f64 / 1e6,
                        "bins": g.bins,
                        "stick_mean": jn(g.stick_mean),
                        "collective_n": jn(g.collective_n),
                        "collective_fraction_of_weight": jn(g.collective_fraction),
                        "rpm": g.rpm,
                        "thrust_ratio": g.thrust_ratio,
                        "speed_sq_sum": g.speed_sq_sum,
                        "k": g.k,
                        "volts_min": jn(g.volts_min),
                        "amps_mean": jn(g.amps_mean),
                        "amps_max": jn(g.amps_max),
                        "torque_mean": {"tx": jn(g.torque[0]), "ty": jn(g.torque[1]), "tz": jn(g.torque[2])},
                    })
                })
                .collect();
            json!({
                "start_us": w.start_us,
                "end_us": w.end_us,
                "duration_s": (w.end_us - w.start_us) as f64 / 1e6,
                "ends_armed": w.open_at_end,
                "airborne": !w.flights.is_empty(),
                "flights": flights,
            })
        })
        .collect();
    let fit_json = |m: Option<usize>, f: &MotorFit| {
        json!({
            "motor": m,
            "a_rad_s": f.a,
            "b_rad_s_per_level": f.b,
            "rms_residual_rad_s": f.rms_residual,
            "bins": f.bins,
            "level_min": f.level_min,
            "level_max": f.level_max,
            "idle_level": f.idle_level,
            "ill_conditioned": f.ill_conditioned,
            "ill_conditioned_note": f.ill_conditioned.then_some("ill-conditioned: narrow level range"),
            "mean_volts": jn(f.mean_volts),
        })
    };
    let needs_poles = r.poles.is_none().then_some(NEEDS_POLES);
    let motors = r.identification.motors.as_ref().map(|motors| {
        motors
            .iter()
            .enumerate()
            .map(|(m, fit)| match fit {
                MotorFitResult::Fit(f) => fit_json(Some(m), f),
                MotorFitResult::NotEnough(n) => json!({"motor": m, "fit": null, "usable_bins": n}),
            })
            .collect::<Vec<_>>()
    });
    let pooled = r.identification.pooled.as_ref().map(|fit| match fit {
        MotorFitResult::Fit(f) => fit_json(None, f),
        MotorFitResult::NotEnough(n) => json!({"fit": null, "usable_bins": n}),
    });
    let thrust = r.identification.thrust.as_ref().map(|k| {
        json!({
            "k1": k.k1,
            "flights_round1": k.flights_round1,
            "k2": k.k2,
            "per_flight": k.per_flight,
            "flights_min": k.flights_min,
            "flights_max": k.flights_max,
            "spread_fraction": k.k2.map(|k2| (k.flights_max - k.flights_min) / k2),
            "bounds_moved_more_than_one_bin": k.bounds_moved,
        })
    });
    let band = |b: &gcs_dflog::RotorBand| {
        json!({
            "lo_hz": b.lo_hz,
            "hi_hz": b.hi_hz,
            "folded": b.folded,
            "shown_lo_hz": b.shown_lo_hz,
            "shown_hi_hz": b.shown_hi_hz,
        })
    };
    let intervals: Vec<Value> = r
        .vibration
        .intervals
        .iter()
        .map(|v| {
            let spectra = v.spectra.as_ref().map(|sp| {
                let axes: Vec<Value> = ["gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"]
                    .iter()
                    .zip(&sp.axes)
                    .map(|(name, a)| {
                        json!({
                            "axis": name,
                            "peaks": a.peaks.iter().map(|p| json!({"hz": p.hz, "rms": p.rms})).collect::<Vec<_>>(),
                            "variance_share_in_band": a.band_share.map(|b| b[0]),
                            "variance_share_in_twice_band": a.band_share.map(|b| b[1]),
                        })
                    })
                    .collect();
                json!({
                    "segments": sp.segments,
                    "segment_samples": SEGMENT,
                    "rotor_band": sp.bands.as_ref().map(|b| band(&b[0])),
                    "rotor_band_twice": sp.bands.as_ref().map(|b| band(&b[1])),
                    "axes": axes,
                })
            });
            json!({
                "kind": match v.kind {
                    IntervalKind::Flight => "flight",
                    IntervalKind::GroundRun => "ground_run",
                },
                "window": v.window + 1,
                "number": v.number,
                "start_us": v.start_us,
                "end_us": v.end_us,
                "samples": v.samples,
                "rate_hz": v.rate_hz,
                "too_slow_for_spectra": v.too_slow,
                "gyro_std_rad_s": v.gyro_std,
                "accel_std_m_s2": v.accel_std,
                "accel_magnitude_std_m_s2": v.accel_mag_std,
                "rotor_hz_min_max": v.rotor_hz.map(|(lo, hi)| [lo, hi]),
                "spectra": spectra,
                "one_g_window": v.one_g.as_ref().map(|g| json!({
                    "low_g": ONE_G_WINDOW.0,
                    "high_g": ONE_G_WINDOW.1,
                    "raw_share": g.raw,
                    "filtered": g.filtered.iter().map(|(fc, sh)| json!({"cutoff_hz": fc, "share": sh})).collect::<Vec<_>>(),
                })),
            })
        })
        .collect();
    json!({
        "file": r.file_name,
        "bytes": r.bytes,
        "bad_bytes": r.bad_bytes,
        "tail_bytes": r.tail_bytes,
        "first_time_us": r.first_time_us,
        "last_time_us": r.last_time_us,
        "duration_s": r.duration_us().map(|d| d as f64 / 1e6),
        "inputs": {
            "mass_kg": r.mass_kg,
            "kf": r.kf,
            "poles": r.poles,
            "poles_source": match r.poles_source {
                PolesSource::Argument => "argument",
                PolesSource::Parameter => "parameter MOT_POLES",
                PolesSource::Missing => "missing",
            },
            "needs_poles": needs_poles,
        },
        "not_in_this_log": missing,
        "health": {
            "dropped_fds": h.dropped_fds,
            "dropped_dsf": h.dropped_dsf,
            "dropped_total": h.dropped_fds.unwrap_or(0) + h.dropped_dsf.unwrap_or(0),
            "least_free_bytes_fds": h.least_free_fds,
            "least_free_bytes_dsf": h.least_free_dsf,
            "max_nav_gap_us_armed": h.max_nav_gap_us,
            "dmd_saturated_in_flights": r.poles.map(|_| h.saturated_airborne),
            "esc_temp_max_degc": h.esc_temp_max,
            "battery_min_v": jn(h.battery_min_v),
        },
        "armed_windows": windows,
        "motor_identification": {
            "needs_poles": needs_poles,
            "motors": motors,
            "pooled": pooled,
            "thrust_constant": thrust,
            "validity": VALIDITY,
        },
        "vibration": {
            "state": match r.vibration.state {
                VibrationState::Done => "done",
                VibrationState::NeedsPoles => "needs_poles",
                VibrationState::NoImu => "no_imu",
            },
            "needs_poles": needs_poles,
            "intervals": intervals,
        },
    })
}
