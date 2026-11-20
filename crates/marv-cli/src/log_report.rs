//! `marv-cli log report`: the flight report of a DataFlash `.BIN` log, as text and optionally as JSON.
//! The analysis is `gcs_dflog::flight_report`; this file only prints.

use std::fmt::Write as _;
use std::io::Write;

use gcs_dflog::{
    AIRBORNE_FRACTION, FlightReport, MIN_SEGMENT_BINS, MotorFitResult, PolesSource, ReportInputs,
    Segment, flight_report,
};
use serde_json::{Value, json};

use crate::args::LogCommand;
use crate::exit;

/// Said next to the identification: the line and the constant hold only where they were measured.
pub const VALIDITY: &str =
    "valid only for the level range and the battery voltage seen in these bins";

pub fn run(command: &LogCommand, out: &mut dyn Write, err: &mut dyn Write) -> i32 {
    let LogCommand::Report {
        file,
        mass_kg,
        kf,
        poles,
        json,
    } = command;
    let inputs = ReportInputs {
        mass_kg: *mass_kg,
        kf: *kf,
        poles: *poles,
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
    let source = match r.poles_source {
        PolesSource::Argument => "from --poles".to_string(),
        PolesSource::Parameter => "from the log's MOT_POLES".to_string(),
        PolesSource::Default => {
            "DEFAULT: no --poles and no MOT_POLES in the log; check it".to_string()
        }
    };
    let _ = writeln!(
        s,
        "  inputs: mass {} kg, kf {:e} N/(rad/s)^2, poles {} ({source})",
        r.mass_kg, r.kf, r.poles
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
        "  DMD records with Sat != 0 while airborne: {}",
        if p.dmd && p.esc {
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
        "\nAirborne segments (bins of 0.5 s with kf x sum of speed^2 >= {AIRBORNE_FRACTION} x mass x g, \
         runs of at least {MIN_SEGMENT_BINS} bins, first and last bin dropped)"
    );
    if !(p.mot && p.esc) {
        let _ = writeln!(
            s,
            "  not in this log ({})",
            if p.mot { "ESC" } else { "MOT" }
        );
    }
    for (i, w) in r.windows.iter().enumerate() {
        if !p.esc {
            break;
        }
        if w.segments.is_empty() {
            let _ = writeln!(s, "  window {}: armed, not airborne", i + 1);
        }
        for (j, seg) in w.segments.iter().enumerate() {
            segment_text(&mut s, i + 1, j + 1, seg, r);
        }
    }

    let _ = writeln!(s, "\nMotor identification (all armed time)");
    match &r.identification.motors {
        None => {
            let _ = writeln!(
                s,
                "  not in this log ({})",
                if p.mot { "ESC" } else { "MOT" }
            );
        }
        Some(motors) => {
            for (m, fit) in motors.iter().enumerate() {
                match fit {
                    MotorFitResult::Fit(f) => {
                        let _ = writeln!(
                            s,
                            "  M{m}: speed = {:.3} + {:.5} x level rad/s; rms residual {:.3} rad/s; {} bins; \
                             level {:.0} to {:.0} (idle {:.0}); mean battery {}",
                            f.a,
                            f.b,
                            f.rms_residual,
                            f.bins,
                            f.level_min,
                            f.level_max,
                            f.idle_level,
                            opt(f.mean_volts, 2, " V", &gap(p.bat, "BAT"))
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
        }
    }
    match &r.identification.thrust {
        Some(k) => {
            let _ = writeln!(
                s,
                "  thrust constant k = {:.4e} N/(rad/s)^2 (given kf {:e}, ratio {:.3}); per segment {}; \
                 spread {:.2} %",
                k.k,
                r.kf,
                k.k / r.kf,
                k.per_segment
                    .iter()
                    .map(|x| format!("{x:.4e}"))
                    .collect::<Vec<_>>()
                    .join(", "),
                100.0 * (k.segments_max - k.segments_min) / k.k
            );
        }
        None => {
            let _ = writeln!(
                s,
                "  thrust constant k: {}",
                if p.esc {
                    "no airborne segment"
                } else {
                    "not in this log (ESC)"
                }
            );
        }
    }
    let _ = writeln!(s, "  These are {VALIDITY}.");
    s
}

fn segment_text(s: &mut String, window: usize, number: usize, seg: &Segment, r: &FlightReport) {
    let p = &r.present;
    let _ = writeln!(
        s,
        "  window {window} segment {number}: {} to {}, duration {}",
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
    ]
    .into_iter()
    .filter(|(present, _)| !present)
    .map(|(_, n)| n)
    .collect();
    let windows: Vec<Value> = r
        .windows
        .iter()
        .map(|w| {
            let segments: Vec<Value> = w
                .segments
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
                "airborne": !w.segments.is_empty(),
                "segments": segments,
            })
        })
        .collect();
    let motors = r.identification.motors.as_ref().map(|motors| {
        motors
            .iter()
            .enumerate()
            .map(|(m, fit)| match fit {
                MotorFitResult::Fit(f) => json!({
                    "motor": m,
                    "a_rad_s": f.a,
                    "b_rad_s_per_level": f.b,
                    "rms_residual_rad_s": f.rms_residual,
                    "bins": f.bins,
                    "level_min": f.level_min,
                    "level_max": f.level_max,
                    "idle_level": f.idle_level,
                    "mean_volts": jn(f.mean_volts),
                }),
                MotorFitResult::NotEnough(n) => json!({"motor": m, "fit": null, "usable_bins": n}),
            })
            .collect::<Vec<_>>()
    });
    let thrust = r.identification.thrust.as_ref().map(|k| {
        json!({
            "k": k.k,
            "per_segment": k.per_segment,
            "segments_min": k.segments_min,
            "segments_max": k.segments_max,
            "spread_fraction": (k.segments_max - k.segments_min) / k.k,
        })
    });
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
                PolesSource::Default => "default",
            },
        },
        "not_in_this_log": missing,
        "health": {
            "dropped_fds": h.dropped_fds,
            "dropped_dsf": h.dropped_dsf,
            "dropped_total": h.dropped_fds.unwrap_or(0) + h.dropped_dsf.unwrap_or(0),
            "least_free_bytes_fds": h.least_free_fds,
            "least_free_bytes_dsf": h.least_free_dsf,
            "max_nav_gap_us_armed": h.max_nav_gap_us,
            "dmd_saturated_airborne": h.saturated_airborne,
            "esc_temp_max_degc": h.esc_temp_max,
            "battery_min_v": jn(h.battery_min_v),
        },
        "armed_windows": windows,
        "motor_identification": {
            "motors": motors,
            "thrust_constant": thrust,
            "validity": VALIDITY,
        },
    })
}
