//! `marv-cli log tune`: known-answer tests on synthetic logs written with the firmware's record layouts, read back
//! through the crate's own reader, plus the arguments. Files only: no link, no vehicle.

mod common;

use std::path::PathBuf;
use std::sync::OnceLock;

use common::{TempDir, run};
use marv_cli::args::{Parsed, parse};
use serde_json::Value;

// ----------------------------------------------------------------------------------------------- the log writer

fn padded(text: &str, size: usize) -> Vec<u8> {
    let mut v = text.as_bytes().to_vec();
    v.resize(size, 0);
    v
}

fn fmt(type_id: u8, name: &str, format: &str, columns: &str, length: usize) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, 128, type_id, length as u8];
    r.extend(padded(name, 4));
    r.extend(padded(format, 16));
    r.extend(padded(columns, 64));
    r
}

const PIDT: u8 = 154;
const NTCH: u8 = 155;
const DMD: u8 = 141;
const MOT: u8 = 142;
const PARM: u8 = 129;

/// The firmware's definitions (fw-core/src/dflog.rs), with its lengths.
fn header(with_tuning: bool) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt(PARM, "PARM", "QNff", "TimeUS,Name,Value,Default", 35));
    f.extend(fmt(DMD, "DMD", "QBffffB", "TimeUS,Ok,Thr,TX,TY,TZ,Sat", 29));
    f.extend(fmt(
        MOT,
        "MOT",
        "QBBHHHHHHHH",
        "TimeUS,Arm,Mode,A0,A1,A2,A3,C0,C1,C2,C3",
        29,
    ));
    if with_tuning {
        f.extend(fmt(
            PIDT,
            "PIDT",
            "Qffffffffffff",
            "TimeUS,TarX,PX,IX,DX,TarY,PY,IY,DY,TarZ,PZ,IZ,DZ",
            59,
        ));
        f.extend(fmt(
            NTCH,
            "NTCH",
            "QfffffffB",
            "TimeUS,GX,GY,GZ,F0,F1,F2,F3,On",
            40,
        ));
    }
    f
}

fn rec(type_id: u8, t_us: u64, body: impl FnOnce(&mut Vec<u8>)) -> Vec<u8> {
    let mut r = vec![0xA3, 0x95, type_id];
    r.extend(t_us.to_le_bytes());
    body(&mut r);
    r
}

fn parm(t_us: u64, name: &str, value: f32) -> Vec<u8> {
    rec(PARM, t_us, |r| {
        r.extend(padded(name, 16));
        r.extend(value.to_le_bytes());
        r.extend(value.to_le_bytes());
    })
}

fn f32s(r: &mut Vec<u8>, v: &[f32]) {
    for x in v {
        r.extend(x.to_le_bytes());
    }
}

struct Rng(u64);
impl Rng {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (u, v) = (self.uniform().max(1e-300), self.uniform());
        (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
    }
}

// ------------------------------------------------------------------------------------------ the synthetic flight

const FS: f64 = 800.0;
const DT_US: u64 = 1250;
const SECONDS: f64 = 110.0;
/// The rotor tone, Hz (not on a bin of the 2048-sample Welch: 800 / 2048 = 0.39 Hz).
const TONE_HZ: f64 = 123.4;
const ROLL_DELAY_S: f64 = 0.0413;
const ROLL_TAU_S: f64 = 0.060;
const PITCH_DELAY_S: f64 = 0.020;
const PITCH_ZETA: f64 = 0.4;
const PITCH_WN: f64 = 2.0 * 2.0 * std::f64::consts::PI;
const NOISE: f64 = 0.01;
/// DMD records with the roll saturation bit set, in the quiet middle of the flight.
const SAT_RECORDS: usize = 40;

/// Steps and doublets (a step up then a step down, back to back): (start s, amplitude, [(duration s, sign)]).
fn schedule(first: f64, n: usize) -> Vec<(f64, f64)> {
    // (time of an edge in s, level after the edge)
    let mut edges = Vec::new();
    for i in 0..n {
        let t = first + 4.6 * i as f64;
        let a = 0.8 * if (i / 2) % 2 == 0 { 1.0 } else { -1.0 };
        if i % 2 == 0 {
            edges.extend([(t, a), (t + 0.8, 0.0)]);
        } else {
            edges.extend([(t, a), (t + 0.5, -a), (t + 1.0, 0.0)]);
        }
    }
    edges
}

/// The target as a staircase (a sample holds its value) and the response of `step` (the unit step response of the
/// plant, delayed) summed over the edges: exact for a staircase input and a delay that is not a whole sample.
fn drive(
    edges: &[(f64, f64)],
    n: usize,
    delay: f64,
    step: impl Fn(f64) -> f64,
) -> (Vec<f64>, Vec<f64>) {
    let mut r = vec![0.0; n];
    let mut level = 0.0;
    let mut e = 0;
    let mut jumps = Vec::new();
    for (k, v) in r.iter_mut().enumerate() {
        let t = k as f64 / FS;
        while e < edges.len() && edges[e].0 <= t + 1e-9 {
            jumps.push((k as f64 / FS, edges[e].1 - level));
            level = edges[e].1;
            e += 1;
        }
        *v = level;
    }
    let y = (0..n)
        .map(|k| {
            let t = k as f64 / FS;
            jumps
                .iter()
                .map(|(te, d)| d * step(t - te - delay))
                .sum::<f64>()
        })
        .collect();
    (r, y)
}

struct Options {
    tuning: bool,
    tone: bool,
    /// Roll and pitch targets a square wave of 2 s period from 2 s on (never hover-like), and the gyro only the
    /// tone: no noise, no plant response.
    manoeuvre: bool,
}

fn synthetic(o: &Options) -> Vec<u8> {
    let n = (SECONDS * FS) as usize;
    let mut f = header(o.tuning);
    // Gains at the start, one rewritten inside the span.
    for (name, v) in [
        ("ATC_ANG_RP_P", 4.5),
        ("ATC_ANG_YAW_P", 1.35),
        ("ATC_RAT_RLL_P", 0.166),
        ("ATC_RAT_RLL_I", 0.166),
        ("ATC_RAT_RLL_D", 0.00746),
        ("ATC_RAT_RLL_IMAX", 0.273),
        ("ATC_RAT_PIT_P", 0.159),
        ("ATC_RAT_PIT_I", 0.159),
        ("ATC_RAT_PIT_D", 0.00716),
        ("ATC_RAT_PIT_IMAX", 0.249),
        ("ATC_RAT_YAW_P", 0.0886),
        ("ATC_RAT_YAW_I", 0.00886),
        ("ATC_RAT_YAW_IMAX", 0.0355),
        ("ATC_RAT_FLTD", 20.0),
        ("RPM_NOTCH_EN", 1.0),
    ] {
        f.extend(parm(0, name, v));
    }
    let tau = ROLL_TAU_S;
    let (roll_r, roll_y) = drive(&schedule(6.0, 8), n, ROLL_DELAY_S, |x| {
        if x > 0.0 { 1.0 - (-x / tau).exp() } else { 0.0 }
    });
    let (wn, z) = (PITCH_WN, PITCH_ZETA);
    let wd = wn * (1.0 - z * z).sqrt();
    let (pitch_r, pitch_y) = drive(&schedule(70.0, 8), n, PITCH_DELAY_S, |x| {
        if x > 0.0 {
            1.0 - (-z * wn * x).exp() * ((wd * x).cos() + z * wn / wd * (wd * x).sin())
        } else {
            0.0
        }
    });
    let mut rng = Rng(0x5eed);
    let two_pi = 2.0 * std::f64::consts::PI;
    let kp = [0.166, 0.159, 0.0886];
    let kd = [0.00746, 0.00716, 0.0];
    let trim = [0.012, -0.02, 0.001];
    let mut g_prev = [0.0f64; 3];
    let sat_from = (50.0 * FS) as usize;
    for k in 0..n {
        let t_us = k as u64 * DT_US;
        let t = k as f64 / FS;
        let tone = if o.tone {
            0.02 * (two_pi * TONE_HZ * t).sin() + 0.01 * (two_pi * 2.0 * TONE_HZ * t + 0.7).sin()
        } else {
            0.0
        };
        let square = if t >= 2.0 && (t as usize).is_multiple_of(2) {
            0.8
        } else if t >= 2.0 {
            -0.8
        } else {
            0.0
        };
        let tar = if o.manoeuvre {
            [square, -square, 0.0]
        } else {
            [roll_r[k], pitch_r[k], 0.0]
        };
        let g = if o.manoeuvre {
            [tone; 3]
        } else {
            [
                roll_y[k] + NOISE * rng.normal() + tone,
                pitch_y[k] + NOISE * rng.normal() + tone,
                NOISE * rng.normal() + tone,
            ]
        };
        let airborne = t >= 2.0;
        let mode = if t < 70.0 { 1 } else { 3 };
        let mut p = [0f32; 3];
        let mut i = [0f32; 3];
        let mut d = [0f32; 3];
        let mut torque = [0f32; 3];
        for a in 0..3 {
            p[a] = (kp[a] * (tar[a] - g[a])) as f32;
            i[a] = (trim[a] + 0.001 * (a as f64)) as f32;
            d[a] = (-kd[a] * (g[a] - g_prev[a]) * FS) as f32;
            torque[a] = p[a] + i[a] + d[a];
        }
        g_prev = g;
        // A gain rewritten inside the span: in force from the next arming.
        if k == (30.0 * FS) as usize {
            f.extend(parm(t_us, "ATC_RAT_RLL_P", 0.2));
        }
        f.extend(rec(MOT, t_us, |r| {
            r.extend([1, mode]);
            r.extend([0u8; 16]);
        }));
        let sat = u8::from((sat_from..sat_from + SAT_RECORDS).contains(&k));
        f.extend(rec(DMD, t_us, |r| {
            r.push(1);
            f32s(r, &[if airborne { 9.0 } else { 0.5 }]);
            f32s(r, &torque);
            r.push(sat);
        }));
        if o.tuning {
            f.extend(rec(PIDT, t_us, |r| {
                for a in 0..3 {
                    f32s(r, &[tar[a] as f32, p[a], i[a], d[a]]);
                }
            }));
            f.extend(rec(NTCH, t_us, |r| {
                f32s(r, &[g[0] as f32, g[1] as f32, g[2] as f32]);
                f32s(r, &[123.2, 123.4, 123.5, 123.6]);
                r.push(1);
            }));
        }
    }
    f
}

fn write(dir: &TempDir, name: &str, bytes: &[u8]) -> PathBuf {
    let path = dir.file(name);
    std::fs::write(&path, bytes).expect("write the log");
    path
}

fn tune_json(path: &std::path::Path, extra: &str) -> Value {
    let out = run(&format!("log tune {} --json {extra}", path.display()));
    assert_eq!(out.code, 0, "stderr: {}", out.err);
    serde_json::from_str(&out.out).expect("JSON on stdout")
}

/// The analysis of the standard synthetic flight, run once for all the tests.
fn flight() -> &'static Value {
    static DOC: OnceLock<Value> = OnceLock::new();
    DOC.get_or_init(|| {
        let dir = TempDir::new("tune-flight");
        let path = write(
            &dir,
            "flight.BIN",
            &synthetic(&Options {
                tuning: true,
                tone: true,
                manoeuvre: false,
            }),
        );
        tune_json(&path, "")
    })
}

fn num(v: &Value) -> f64 {
    v.as_f64().unwrap_or_else(|| panic!("not a number: {v}"))
}

// ------------------------------------------------------------------------------------------------- known answers

#[test]
fn the_delay_and_time_constant_of_a_known_lag_are_recovered() {
    let doc = flight();
    let span = &doc["spans"][0];
    assert_eq!(span["context"]["analysed"], true);
    let roll = &span["axes"][0]["response"];
    assert_eq!(roll["made"], true, "{roll}");
    let n = roll["events"].as_u64().unwrap();
    assert_eq!(n, 8, "usable roll events");
    let dead = num(&roll["fit"]["dead_time_s"]["median"]);
    let tau = num(&roll["fit"]["time_constant_s"]["median"]);
    let tol = 1.0 / FS + 0.05 * ROLL_DELAY_S;
    println!(
        "roll: dead time {dead:.5} s (true {ROLL_DELAY_S}), tolerance {tol:.5}; time constant {tau:.5} s (true {ROLL_TAU_S})"
    );
    assert!(
        (dead - ROLL_DELAY_S).abs() <= tol,
        "dead time {dead} against {ROLL_DELAY_S}"
    );
    assert!(
        (tau / ROLL_TAU_S - 1.0).abs() <= 0.10,
        "time constant {tau} against {ROLL_TAU_S}"
    );
    // The cross-correlation lag is the apparent delay: it lies between the dead time and dead time + tau.
    let lag = num(&roll["cross_correlation_lag_s"]["median"]);
    assert!(
        lag > ROLL_DELAY_S && lag < ROLL_DELAY_S + 1.5 * ROLL_TAU_S,
        "lag {lag}"
    );
    assert_eq!(roll["fit"]["events_used"].as_u64().unwrap(), n);
    // Negative control: a first-order lag does not overshoot.
    let overshoot = num(&roll["overshoot_pct"]["median"]);
    assert!(
        overshoot < 3.0,
        "overshoot of a first-order lag {overshoot} %"
    );
}

#[test]
fn the_minus_3_db_frequency_is_recovered() {
    let roll = &flight()["spans"][0]["axes"][0]["frequency_response"];
    assert_eq!(roll["made"], true, "{roll}");
    let f3 = num(&roll["bandwidth_3db"]["hz"]);
    let truth = 1.0 / (2.0 * std::f64::consts::PI * ROLL_TAU_S);
    println!("roll: -3 dB at {f3:.3} Hz (true {truth:.3})");
    assert!((f3 / truth - 1.0).abs() <= 0.10, "{f3} against {truth}");
    // Its phase lag: delay plus first-order lag at that frequency.
    let want = (360.0 * f3 * ROLL_DELAY_S)
        + (2.0 * std::f64::consts::PI * f3 * ROLL_TAU_S)
            .atan()
            .to_degrees();
    let got = num(&roll["bandwidth_3db"]["phase_lag_deg"]);
    assert!((got - want).abs() < 6.0, "phase lag {got} against {want}");
    // Where the coherence is high the table has numbers, and every row carries its coherence.
    assert!(
        roll["rows"]
            .as_array()
            .unwrap()
            .iter()
            .any(|r| r.get("gain_db").is_some())
    );
}

#[test]
fn the_overshoot_of_an_underdamped_axis_is_recovered() {
    let pitch = &flight()["spans"][0]["axes"][1]["response"];
    assert_eq!(pitch["made"], true, "{pitch}");
    let truth =
        100.0 * (-std::f64::consts::PI * PITCH_ZETA / (1.0 - PITCH_ZETA * PITCH_ZETA).sqrt()).exp();
    let got = num(&pitch["overshoot_pct"]["median"]);
    println!(
        "pitch: overshoot {got:.2} % (true {truth:.2} %), held events {}",
        pitch["held_events"]
    );
    assert!((got - truth).abs() <= 5.0, "{got} against {truth}");
    assert!(pitch["held_events"].as_u64().unwrap() >= 3);
    let dead = num(&pitch["fit"]["dead_time_s"]["median"]);
    println!(
        "pitch: first-order fit dead time {dead:.4} s (true delay {PITCH_DELAY_S}; a second-order plant is not first order)"
    );
}

#[test]
fn an_axis_with_a_zero_target_says_insufficient_excitation() {
    let yaw = &flight()["spans"][0]["axes"][2];
    assert_eq!(yaw["excitation"]["usable"], 0);
    assert_eq!(yaw["response"]["made"], false);
    assert_eq!(yaw["frequency_response"]["made"], false);
    assert!(
        yaw["frequency_response"]["reason"]
            .as_str()
            .unwrap()
            .contains("insufficient excitation")
    );
    // The same in the text.
    let dir = TempDir::new("tune-text");
    let path = write(
        &dir,
        "t.BIN",
        &synthetic(&Options {
            tuning: true,
            tone: false,
            manoeuvre: false,
        }),
    );
    let out = run(&format!("log tune {}", path.display()));
    assert!(
        out.out.contains("yaw: insufficient excitation"),
        "{}",
        out.out
    );
}

#[test]
fn the_rotor_tone_is_reported_within_one_bin_and_related_to_the_rotors() {
    let noise = &flight()["spans"][0]["noise"];
    assert_eq!(noise["made"], true, "{noise}");
    let bin = num(&noise["bin_hz"]);
    for axis in 0..3 {
        let peaks = noise["axes"][axis]["peaks"].as_array().unwrap();
        let top = &peaks[0];
        let hz = num(&top["hz"]);
        println!("axis {axis}: strongest peak {hz:.2} Hz (tone {TONE_HZ}), bin {bin:.3} Hz");
        assert!((hz - TONE_HZ).abs() <= bin, "{hz}");
        assert!(
            top["rotor_matches"]
                .as_array()
                .unwrap()
                .iter()
                .any(|m| m["harmonic"] == 1),
            "{top}"
        );
        let second = num(&peaks[1]["hz"]);
        assert!((second - 2.0 * TONE_HZ).abs() <= bin, "{second}");
        assert!(
            peaks[1]["rotor_matches"]
                .as_array()
                .unwrap()
                .iter()
                .any(|m| m["harmonic"] == 2),
            "{}",
            peaks[1]
        );
    }
    let lines = noise["rotors"].as_array().unwrap();
    assert_eq!(lines.len(), 4);
    assert!((num(&lines[0]["f1_lo_hz"]) - 123.2).abs() < 0.01);
    assert!((num(&lines[3]["f1_hi_hz"]) - 123.6).abs() < 0.01);
    assert_eq!(lines[0]["name"], "line 0 (M4)");
    assert_eq!(lines[3]["name"], "line 3 (M1)");
}

#[test]
fn without_the_tone_no_peak_is_related_to_a_rotor() {
    let dir = TempDir::new("tune-notone");
    let path = write(
        &dir,
        "notone.BIN",
        &synthetic(&Options {
            tuning: true,
            tone: false,
            manoeuvre: false,
        }),
    );
    let doc = tune_json(&path, "");
    let peaks = doc["spans"][0]["noise"]["axes"][0]["peaks"]
        .as_array()
        .unwrap();
    assert!(
        peaks.iter().all(|p| num(&p["rms"]) < 0.002),
        "white noise only: no peak should stand out: {peaks:?}"
    );
}

#[test]
fn context_terms_and_saturation_are_traceable() {
    let span = &flight()["spans"][0];
    let ctx = &span["context"];
    // Gains: the PARM at the start, not the one rewritten inside the span.
    assert_eq!(ctx["gains"]["logged"], true);
    let gain = |n: &str| {
        ctx["gains"]["values"]
            .as_array()
            .unwrap()
            .iter()
            .find(|g| g["name"] == n)
            .map(|g| num(&g["value"]))
            .unwrap()
    };
    assert!((gain("ATC_RAT_RLL_P") - 0.166).abs() < 1e-6);
    let changed = ctx["gains"]["changed_during_span"].as_array().unwrap();
    assert_eq!(changed.len(), 1);
    assert_eq!(changed[0]["name"], "ATC_RAT_RLL_P");
    assert!((num(&ctx["notch"]["ntch_on_fraction"]) - 1.0).abs() < 1e-12);
    assert_eq!(num(&ctx["notch"]["rpm_notch_en"]), 1.0);
    // Modes: Angle then Position at 70 s.
    let modes = ctx["modes"].as_array().unwrap();
    assert_eq!(modes[0]["mode"], "Angle");
    assert_eq!(modes[1]["mode"], "Position");
    assert!((num(&modes[1]["start_s"]) - 70.0).abs() < 0.01);
    // Airborne from 2 s: the ground second is not in it.
    assert!(
        (num(&ctx["airborne_s"]) - (SECONDS - 2.0)).abs() < 0.05,
        "{}",
        ctx["airborne_s"]
    );
    // Saturation: exactly SAT_RECORDS of the airborne DMD records, on the roll bit only.
    let sat = &ctx["saturation"];
    let records = num(&sat["records"]);
    assert!((num(&sat["any"]) - SAT_RECORDS as f64 / records).abs() < 1e-12);
    assert_eq!(num(&sat["bits"][0]["fraction"]), num(&sat["any"]));
    assert_eq!(num(&sat["bits"][1]["fraction"]), 0.0);
    // The controller terms: the trim is the mean I, and the P term implies the gain it was made with.
    let roll = &span["axes"][0]["terms"];
    assert!((num(&roll["hover_like"]["mean_i_nm"]) - 0.012).abs() < 1e-6);
    assert!(
        (num(&roll["implied_kp"]["value"]) - 0.166).abs() < 1e-4,
        "{}",
        roll["implied_kp"]
    );
    assert!(num(&roll["sum_check"]["max_abs_nm"]) < 1e-6);
    let hf = &roll["above_split"];
    assert_eq!(hf["made"], true, "{hf}");
    assert!(
        num(&hf["rms_d_nm"]) > num(&hf["rms_p_nm"]),
        "the D term carries the white gyro noise: {hf}"
    );
    // No observation recommends a value or a direction.
    for o in span["observations"].as_array().unwrap() {
        let o = o.as_str().unwrap().to_lowercase();
        for word in [
            "should",
            "increase",
            "decrease",
            "raise",
            "lower",
            "reduce",
            "recommend",
            "try ",
        ] {
            assert!(!o.contains(word), "{o}");
        }
    }
}

#[test]
fn from_and_to_limit_the_analysis() {
    let dir = TempDir::new("tune-window");
    let path = write(
        &dir,
        "w.BIN",
        &synthetic(&Options {
            tuning: true,
            tone: true,
            manoeuvre: false,
        }),
    );
    // Only the roll events: pitch has none inside 0..60 s.
    let doc = tune_json(&path, "--to 60");
    let span = &doc["spans"][0];
    assert_eq!(span["axes"][0]["excitation"]["usable"], 8);
    assert_eq!(span["axes"][1]["excitation"]["usable"], 0);
    assert!(num(&span["context"]["window_s"][1]) <= 60.0 + 1e-6);
    // A window with no armed time.
    let doc = tune_json(&path, "--from 200");
    assert_eq!(doc["spans"].as_array().unwrap().len(), 0);
}

#[test]
fn a_log_without_the_new_records_gets_its_context_and_says_what_needs_newer_firmware() {
    let dir = TempDir::new("tune-old");
    let path = write(
        &dir,
        "old.BIN",
        &synthetic(&Options {
            tuning: false,
            tone: true,
            manoeuvre: false,
        }),
    );
    let out = run(&format!("log tune {}", path.display()));
    assert_eq!(out.code, 0, "{}", out.err);
    for needle in [
        "=== Armed span 1",
        "modes flown",
        "needs the PIDT and NTCH records",
        "this log has neither PIDT nor NTCH records",
        "need the PIDT record",
        "needs the NTCH record",
    ] {
        assert!(
            out.out.contains(needle),
            "missing {needle:?} in\n{}",
            out.out
        );
    }
    // The gains were logged here; an old log has none of the ATC_ names: use a log with no PARM at all.
    let doc = tune_json(&path, "");
    assert_eq!(doc["has_pidt"], false);
    assert_eq!(doc["spans"][0]["axes"][0]["response"]["made"], false);
    assert!(
        doc["spans"][0]["needs_newer_firmware"]
            .as_array()
            .unwrap()
            .len()
            >= 3
    );
    // No numbers of sections 2 to 6 are made up.
    assert!(!out.out.contains("(apparent delay"));
}

#[test]
fn no_gains_in_the_log_says_compiled_defaults_assumed() {
    let dir = TempDir::new("tune-nogains");
    let mut f = header(false);
    for k in 0..4000u64 {
        let t = k * DT_US;
        f.extend(rec(MOT, t, |r| {
            r.extend([1, 1]);
            r.extend([0u8; 16]);
        }));
        f.extend(rec(DMD, t, |r| {
            r.push(1);
            f32s(r, &[9.0, 0.0, 0.0, 0.0]);
            r.push(0);
        }));
    }
    let path = write(&dir, "nogains.BIN", &f);
    let out = run(&format!("log tune {}", path.display()));
    assert_eq!(out.code, 0, "{}", out.err);
    assert!(
        out.out
            .contains("gains in force: not logged: compiled defaults assumed"),
        "{}",
        out.out
    );
}

#[test]
fn the_header_prints_every_constant_and_a_flag_changes_it() {
    let dir = TempDir::new("tune-header");
    let path = write(
        &dir,
        "h.BIN",
        &synthetic(&Options {
            tuning: false,
            tone: false,
            manoeuvre: false,
        }),
    );
    let out = run(&format!(
        "log tune {} --quiet-band 0.2 --frf-seg 1024",
        path.display()
    ));
    assert_eq!(out.code, 0);
    for def in marv_cli::log_tune::DEFS {
        assert!(
            out.out.contains(def.name),
            "{} missing from the header",
            def.name
        );
        assert!(
            out.out.contains(&format!("--{}", def.flag)),
            "{} flag missing",
            def.flag
        );
    }
    assert!(
        out.out
            .contains("QUIET_BAND_RADS = 0.2 rad/s [CHANGED by flag; default 0.1]")
    );
    assert!(out.out.contains("FRF_SEG = 1024 samples [CHANGED"));
    let doc = tune_json(&path, "--quiet-band 0.2");
    let c = doc["constants"]
        .as_array()
        .unwrap()
        .iter()
        .find(|c| c["name"] == "QUIET_BAND_RADS")
        .unwrap();
    assert_eq!(num(&c["value"]), 0.2);
    assert_eq!(num(&c["default"]), 0.1);
}

// ------------------------------------------------------------------------------------------------- the arguments

fn args(line: &str) -> Vec<String> {
    line.split_whitespace().map(str::to_string).collect()
}

#[test]
fn the_arguments_parse_and_fail_cleanly() {
    let Ok(Parsed::LogTune(c)) = parse(&args(
        "log tune x.BIN --json --from 1.5 --to 9 --coherence 0.7 --noise-seg 1024",
    )) else {
        panic!("log tune did not parse");
    };
    assert_eq!(c.file, PathBuf::from("x.BIN"));
    assert!(c.json);
    assert_eq!((c.from_s, c.to_s), (Some(1.5), Some(9.0)));
    assert_eq!(
        c.settings.get(marv_cli::log_tune::settings::Id::Coherence),
        0.7
    );
    assert_eq!(
        c.settings.count(marv_cli::log_tune::settings::Id::NoiseSeg),
        1024
    );
    let e = |l: &str| parse(&args(l)).expect_err(l);
    assert!(e("log tune").contains("needs a FILE.BIN"));
    assert!(e("log tune a.BIN b.BIN").contains("unexpected argument"));
    assert!(e("log tune a.BIN --from 5 --to 2").contains("--from needs a time before --to"));
    assert!(e("log tune a.BIN --from -1").contains("seconds from 0"));
    assert!(e("log tune a.BIN --nonsense 1").contains("unknown option --nonsense"));
    assert!(e("log tune a.BIN --frf-seg 1000").contains("power of two"));
    assert!(e("log tune a.BIN --coherence 2").contains("from 0 to 1"));
    assert!(e("log tune a.BIN --min-events 2.5").contains("whole number"));
    assert!(e("log tune a.BIN --coherence").contains("needs a value"));
    assert_eq!(parse(&args("log tune --help")), Ok(Parsed::Help));
    assert!(marv_cli::args::USAGE.contains("log tune"));
    // `log report` is unchanged.
    assert!(matches!(
        parse(&args("log report r.BIN --mass-kg 1 --kf 1e-5")),
        Ok(Parsed::Log(_))
    ));
}

#[test]
fn a_missing_file_fails_with_a_message() {
    let out = run("log tune /nonexistent/none.BIN");
    assert_eq!(out.code, 1);
    assert!(
        out.err.contains("log tune of /nonexistent/none.BIN failed"),
        "{}",
        out.err
    );
}

#[test]
fn continuous_manoeuvring_still_gives_the_noise_figures_over_the_whole_airborne_time() {
    let dir = TempDir::new("tune-manoeuvre");
    let path = write(
        &dir,
        "manoeuvre.BIN",
        &synthetic(&Options {
            tuning: true,
            tone: true,
            manoeuvre: true,
        }),
    );
    let doc = tune_json(&path, "");
    let span = &doc["spans"][0];
    let noise = &span["noise"];
    assert_eq!(noise["made"], true, "{noise}");
    assert_eq!(noise["basis"], "whole airborne time");
    assert!(noise["hover_like"].is_null(), "no hover-like run: {noise}");
    let bin = num(&noise["bin_hz"]);
    // The gyro: 0.02 at TONE_HZ and 0.01 at twice it, so rms sqrt(0.02^2/2 + 0.01^2/2) above 50 Hz.
    let want_g = (0.02f64.powi(2) / 2.0 + 0.01f64.powi(2) / 2.0).sqrt();
    for axis in 0..3 {
        let a = &noise["axes"][axis];
        let top = &a["peaks"][0];
        assert!((num(&top["hz"]) - TONE_HZ).abs() <= bin, "{top}");
        assert!(
            (num(&top["rms"]) - 0.02 / 2f64.sqrt()).abs() < 0.0015,
            "{top}"
        );
        assert!(
            (num(&a["rms_above_split"]) - want_g).abs() < 0.03 * want_g,
            "{a}"
        );
        assert!(
            top["rotor_matches"]
                .as_array()
                .unwrap()
                .iter()
                .any(|m| m["harmonic"] == 1),
            "{top}"
        );
    }
    // The D torque of roll: the sampled derivative of the tone, amplitude kd * a * 2 sin(pi f / fs) * fs.
    let amp = |kd: f64, a: f64, f: f64| kd * a * 2.0 * (std::f64::consts::PI * f / FS).sin() * FS;
    let want_d =
        ((amp(0.00746, 0.02, TONE_HZ).powi(2) + amp(0.00746, 0.01, 2.0 * TONE_HZ).powi(2)) / 2.0)
            .sqrt();
    let hf = &span["axes"][0]["terms"]["above_split"];
    assert_eq!(hf["made"], true, "{hf}");
    assert!(
        (num(&hf["rms_d_nm"]) - want_d).abs() < 0.03 * want_d,
        "{hf} want {want_d}"
    );
    assert!(span["axes"][0]["terms"]["above_split_hover_like"].is_null());
}

#[test]
fn a_long_hover_gives_the_whole_airborne_and_the_hover_like_lines() {
    let flight = &flight()["spans"][0];
    let noise = &flight["noise"];
    assert_eq!(noise["basis"], "whole airborne time");
    assert!(noise["hover_like"]["made"] == true, "{noise}");
    let t = &flight["axes"][0]["terms"];
    assert_eq!(t["above_split"]["made"], true);
    assert!(num(&t["above_split_hover_like"]["rms_d_nm"]) > 0.0);
    assert!(
        num(&t["above_split"]["segments"]) > num(&t["above_split_hover_like"]["segments"]),
        "{t}"
    );
    let dir = TempDir::new("tune-hover-text");
    let path = write(
        &dir,
        "flight.BIN",
        &synthetic(&Options {
            tuning: true,
            tone: true,
            manoeuvre: false,
        }),
    );
    let out = run(&format!("log tune {}", path.display()));
    assert_eq!(out.code, 0, "{}", out.err);
    assert!(out.out.contains("whole airborne time"), "{}", out.out);
    assert!(out.out.contains("hover-like, "), "{}", out.out);
}
