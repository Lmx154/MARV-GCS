//! The flight report, on synthetic logs built here: no real log, no real position anywhere.

use std::io::Cursor;

use crate::flight_report::{GRAVITY, flight_report_of};
use crate::tests::{V, fmt_record, int, record};
use crate::{
    Control, FlightReport, IntervalKind, MotorFitResult, PolesSource, Reader, ReportInputs,
    VibrationState, flight_report_with,
};

const MOT: &str = "QBBHHHHHHHH";
const ESC: &str = "QIIIIccccBBBBB";
const DMD: &str = "QBffffB";
const SETP: &str = "QBffffffff";
const RCS: &str = "QffffB";
const BAT: &str = "QBff";
const NAV: &str = "QfffffffB";
const FDS: &str = "QII";
const DSF: &str = "QIHIIII";
const PARM: &str = "QNff";
const GPS: &str = "QBBIHBcLLeff";
const IMU: &str = "QBffffff";

const STEP: i64 = 10_000;
const BIN: i64 = 500_000;
const T0: i64 = 2_000_000;
const END1: i64 = T0 + 16 * BIN + 250_000;
/// The first window's bins by target r (kf x sum of speed^2 over mass x g): the ground, a run-up to r = 0.8,
/// the flight (bins 4..=14, with bins just under 1.0 inside it), the ground.
const R_PROFILE: [(usize, f64); 11] = [
    (4, 1.6),
    (5, 1.5),
    (6, 1.8),
    (7, 1.4),
    (8, 1.1),
    (9, 0.95),
    (10, 0.92),
    (11, 1.2),
    (12, 1.5),
    (13, 1.0),
    (14, 0.97),
];
const FLIGHT_FIRST_BIN: usize = 4;
const FLIGHT_LAST_BIN: usize = 14;
const T2: i64 = 11_250_000;
const END2: i64 = 13_250_000;
const LOG_END: i64 = 14_000_000;

const MASS: f64 = 1.2;
/// The thrust constant the report is told; the log's own is different (see `K_TRUE`).
const KF: f64 = 2.8e-5;
const POLES: f64 = 12.0;
const GPS_LAT: i64 = 123_456_789;
const GPS_LNG: i64 = -987_654_321;

/// The motor's line speed = a + b x level, rad/s.
fn line(m: usize) -> (f64, f64) {
    (-40.0 + 5.0 * m as f64, 0.7 + 0.02 * m as f64)
}

/// The level of every motor that gives `r` with the given kf.
fn level_for_r(r: f64) -> i64 {
    let (mut lo, mut hi) = (100.0, 3000.0);
    for _ in 0..60 {
        let mid = (lo + hi) / 2.0;
        if KF * sum_sq(mid) / (MASS * GRAVITY) < r {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo.round() as i64
}

/// The level of bin `idx` of the first window: 0 and 1 ground, 2 a run-up to r = 0.4, 3 to r = 0.8, then the
/// flight, then the ground.
fn bin_level(idx: usize) -> i64 {
    match idx {
        2 => level_for_r(0.4),
        3 => level_for_r(0.8),
        FLIGHT_FIRST_BIN..=FLIGHT_LAST_BIN => level_for_r(R_PROFILE[idx - FLIGHT_FIRST_BIN].1),
        _ => 100,
    }
}

/// The level of every motor at `t` (armed).
fn level_at(t: i64) -> i64 {
    if !(T0..END1).contains(&t) {
        return 100;
    }
    bin_level(((t - T0) / BIN) as usize)
}

fn armed(t: i64) -> bool {
    (T0..END1).contains(&t) || (T2..END2).contains(&t)
}

fn erpm(speed: f64) -> i64 {
    (speed * 60.0 / std::f64::consts::TAU * POLES / 2.0).round() as i64
}

struct Opts {
    esc: bool,
    gps: bool,
    parm_poles: bool,
}

fn header(o: &Opts) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(fmt_record(
        130,
        "MOT",
        MOT,
        "TimeUS,Arm,Mode,A0,A1,A2,A3,C0,C1,C2,C3",
    ));
    if o.esc {
        f.extend(fmt_record(
            131,
            "ESC",
            ESC,
            "TimeUS,E0,E1,E2,E3,V0,V1,V2,V3,T0,T1,T2,T3,Fl",
        ));
    }
    f.extend(fmt_record(132, "DMD", DMD, "TimeUS,Ok,Thr,TX,TY,TZ,Sat"));
    f.extend(fmt_record(
        133,
        "SETP",
        SETP,
        "TimeUS,K,Q1,Q2,Q3,Q4,RX,RY,RZ,Thr",
    ));
    f.extend(fmt_record(134, "RCS", RCS, "TimeUS,R,P,T,Y,Arm"));
    f.extend(fmt_record(135, "BAT", BAT, "TimeUS,Inst,Volt,Curr"));
    f.extend(fmt_record(
        136,
        "NAV",
        NAV,
        "TimeUS,Q1,Q2,Q3,Q4,GX,GY,GZ,Vld",
    ));
    f.extend(fmt_record(137, "FDS", FDS, "TimeUS,Dp,FMn"));
    f.extend(fmt_record(
        138,
        "DSF",
        DSF,
        "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv",
    ));
    f.extend(fmt_record(139, "PARM", PARM, "TimeUS,Name,Value,Default"));
    f.extend(fmt_record(
        142,
        "IMU",
        IMU,
        "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
    ));
    f.extend(fmt_record(
        140,
        "GPS",
        GPS,
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
    ));
    f.extend(fmt_record(
        141,
        "GPS2",
        GPS,
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
    ));
    f
}

fn gps(type_id: u8, t: i64) -> Vec<u8> {
    record(
        type_id,
        GPS,
        &[
            int(t),
            int(0),
            int(3),
            int(0),
            int(0),
            int(9),
            int(120),
            int(GPS_LAT),
            int(GPS_LNG),
            int(5000),
            V::Float(1.0),
            V::Float(2.0),
        ],
    )
}

fn synth(o: &Opts) -> Vec<u8> {
    let mut f = header(o);
    if o.parm_poles {
        f.extend(record(
            139,
            PARM,
            &[int(0), V::Text("MOT_POLES"), V::Float(POLES), V::Float(0.0)],
        ));
    }
    if o.gps {
        // Far later than everything else: a reader that touched it would move the end of the log.
        f.extend(gps(140, 900_000_000));
        f.extend(gps(141, 901_000_000));
    }
    let mut t = 0;
    while t < LOG_END {
        let on = armed(t);
        let level = if on { level_at(t) } else { 0 };
        let l = level as u16 as i64;
        // 100 Hz, one IMU that counts and a second that must be ignored.
        for (inst, az) in [(0, GRAVITY), (1, 50.0)] {
            f.extend(record(
                142,
                IMU,
                &[
                    int(t),
                    int(inst),
                    V::Float(0.0),
                    V::Float(0.0),
                    V::Float(0.0),
                    V::Float(0.0),
                    V::Float(0.0),
                    V::Float(az),
                ],
            ));
        }
        f.extend(record(
            130,
            MOT,
            &[
                int(t),
                int(i64::from(on)),
                int(0),
                int(l),
                int(l),
                int(l),
                int(l),
                int(0),
                int(0),
                int(0),
                int(0),
            ],
        ));
        if (t / STEP) % 2 == 0 {
            if o.esc {
                let e: Vec<V> = (0..4)
                    .map(|m| {
                        let (a, b) = line(m);
                        int(if on { erpm(a + b * level as f64) } else { 0 })
                    })
                    .collect();
                let sec = (t - T0) as f64 / 1e6;
                let mut v = vec![int(t)];
                v.extend(e);
                v.extend((0..4).map(|_| int(0)));
                v.extend((0..4).map(|m| int(50 + m - ((sec - 5.0).abs() * 2.0).round() as i64)));
                v.push(int(0));
                f.extend(record(131, ESC, &v));
            }
            let hover = on && (T0 + 4 * BIN..T0 + 15 * BIN).contains(&t);
            // Saturation: 3 records in the first flight bin, 5 in the fifth, 4 in the last (dropped from
            // the flight), 2 on the ground.
            let rel = t - T0;
            let sat = (hover && (0..60_000).contains(&(rel - 4 * BIN)))
                || (hover && (0..100_000).contains(&(rel - 8 * BIN)))
                || (hover && (0..80_000).contains(&(rel - 14 * BIN)))
                || (on && (0..40_000).contains(&(rel - BIN)));
            f.extend(record(
                132,
                DMD,
                &[
                    int(t),
                    int(1),
                    V::Float(0.0),
                    V::Float(0.1),
                    V::Float(-0.2),
                    V::Float(0.05),
                    int(i64::from(sat)),
                ],
            ));
            let mut setp = vec![int(t), int(0)];
            setp.extend((0..7).map(|_| V::Float(0.0)));
            setp.push(V::Float(if hover { 11.0 } else { 0.0 }));
            f.extend(record(133, SETP, &setp));
            f.extend(record(
                134,
                RCS,
                &[
                    int(t),
                    V::Float(0.0),
                    V::Float(0.0),
                    V::Float(if hover { 0.55 } else { 0.1 }),
                    V::Float(0.0),
                    int(i64::from(on)),
                ],
            ));
            let one_low = t == T0 + 9 * BIN;
            let volt = if one_low {
                14.5
            } else if hover {
                15.0
            } else {
                16.0
            };
            let curr = if one_low {
                30.0
            } else if hover {
                25.0
            } else {
                2.0
            };
            f.extend(record(
                135,
                BAT,
                &[int(t), int(0), V::Float(volt), V::Float(curr)],
            ));
            // A second battery instance that must be ignored.
            f.extend(record(
                135,
                BAT,
                &[int(t), int(1), V::Float(3.0), V::Float(99.0)],
            ));
            let nav_here = !(((6_020_000..6_060_000).contains(&t))
                || (10_400_000..11_000_000).contains(&t)
                || (100_000..1_900_000).contains(&t));
            if nav_here {
                let mut nav = vec![int(t)];
                nav.extend((0..7).map(|_| V::Float(0.0)));
                nav.push(int(1));
                f.extend(record(136, NAV, &nav));
            }
        }
        if t % 1_000_000 == 0 {
            let n = t / 1_000_000;
            f.extend(record(
                137,
                FDS,
                &[
                    int(t),
                    int([0, 0, 3, 3, 7, 7, 7][(n as usize).min(6)]),
                    int([5000, 4000, 4500][(n % 3) as usize]),
                ],
            ));
            f.extend(record(
                138,
                DSF,
                &[
                    int(t),
                    int(n.min(2)),
                    int(1),
                    int(2),
                    int(100 + n),
                    int(9),
                    int(9),
                ],
            ));
        }
        t += STEP;
    }
    f
}

fn inputs() -> ReportInputs {
    ReportInputs {
        mass_kg: MASS,
        kf: KF,
        poles: Some(POLES),
        accel_lpf_hz: Vec::new(),
    }
}

fn report(o: &Opts, i: &ReportInputs) -> FlightReport {
    flight_report_of(Cursor::new(synth(o)), None, i).unwrap()
}

fn full() -> Opts {
    Opts {
        esc: true,
        gps: false,
        parm_poles: false,
    }
}

fn within(got: f64, want: f64, fraction: f64) {
    assert!(
        (got - want).abs() <= fraction * want.abs(),
        "{got} is not within {fraction} of {want}"
    );
}

/// Sum over the motors of the squared speed at `level`.
fn sum_sq(level: f64) -> f64 {
    (0..4)
        .map(|m| {
            let (a, b) = line(m);
            (a + b * level).powi(2)
        })
        .sum()
}

/// The levels of the flight's bins that count (the last of the run is dropped).
fn flight_levels() -> Vec<f64> {
    (FLIGHT_FIRST_BIN..FLIGHT_LAST_BIN)
        .map(|i| bin_level(i) as f64)
        .collect()
}

#[test]
fn windows_flight_fit_and_constant() {
    let r = report(&full(), &inputs());
    assert_eq!(r.bad_bytes, 0);
    assert_eq!(r.first_time_us, Some(0));
    assert_eq!(r.windows.len(), 2);
    let w = &r.windows[0];
    assert_eq!(
        (w.start_us, w.end_us, w.open_at_end),
        (T0 as u64, END1 as u64, false)
    );
    // One flight: the bins just under r = 1 do not split it, the run-up to r = 0.8 is not in it.
    assert_eq!(w.flights.len(), 1);
    let s = &w.flights[0];
    assert_eq!(s.start_us, (T0 + 4 * BIN) as u64);
    assert_eq!(s.end_us, (T0 + 14 * BIN) as u64);
    assert_eq!(s.bins, 10);
    // The second window is armed and never airborne.
    let w2 = &r.windows[1];
    assert_eq!((w2.start_us, w2.end_us), (T2 as u64, END2 as u64));
    assert!(w2.flights.is_empty());

    let mg = MASS * GRAVITY;
    let levels = flight_levels();
    let mean_ssq = levels.iter().map(|l| sum_sq(*l)).sum::<f64>() / 10.0;
    within(s.speed_sq_sum, mean_ssq, 0.01);
    within(s.thrust_ratio, KF * mean_ssq / mg, 0.01);
    within(s.stick_mean.unwrap(), 0.55, 1e-6);
    within(s.collective_n.unwrap(), 11.0, 1e-6);
    within(s.collective_fraction.unwrap(), 11.0 / mg, 1e-6);
    for m in 0..4 {
        let (a, b) = line(m);
        let rpm =
            levels.iter().map(|l| a + b * l).sum::<f64>() / 10.0 * 60.0 / std::f64::consts::TAU;
        within(s.rpm[m], rpm, 0.001);
    }
    assert_eq!(s.volts_min, Some(14.5));
    assert_eq!(s.amps_max, Some(30.0));
    let amps = s.amps_mean.unwrap();
    assert!(amps > 24.9 && amps < 26.0, "{amps}");
    within(s.torque[0].unwrap(), 0.1, 1e-5);
    within(s.torque[1].unwrap(), -0.2, 1e-5);
    within(s.torque[2].unwrap(), 0.05, 1e-5);

    // The motor identification: the run-up, the flight and its last bin, 13 bins; the idle out.
    let all_levels: Vec<f64> = (2..=FLIGHT_LAST_BIN).map(|i| bin_level(i) as f64).collect();
    let (lo, hi) = (
        all_levels.iter().copied().fold(f64::INFINITY, f64::min),
        all_levels.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    );
    let fits = r.identification.motors.as_ref().unwrap();
    for (m, fit) in fits.iter().enumerate() {
        let MotorFitResult::Fit(f) = fit else {
            panic!("motor {m}: {fit:?}")
        };
        let (a, b) = line(m);
        within(f.a, a, 0.01);
        within(f.b, b, 0.01);
        assert_eq!(f.bins, 13);
        assert_eq!((f.level_min, f.level_max), (lo, hi));
        assert_eq!(f.idle_level, 100.0);
        assert!(!f.ill_conditioned);
        assert!(f.rms_residual < 0.05, "{}", f.rms_residual);
        let v = f.mean_volts.unwrap();
        assert!(v > 14.9 && v <= 16.0, "{v}");
    }
    // The pooled fit has the points of the four motors; their lines differ, so it fits worse.
    let MotorFitResult::Fit(p) = r.identification.pooled.as_ref().unwrap() else {
        panic!("no pooled fit")
    };
    assert_eq!(p.bins, 52);
    assert!(!p.ill_conditioned && p.rms_residual > 1.0, "{p:?}");

    let k = r.identification.thrust.as_ref().unwrap();
    within(k.k1, mg / mean_ssq, 0.01);
    within(k.k2.unwrap(), mg / mean_ssq, 0.01);
    assert_eq!(k.k2, Some(k.k1));
    assert_eq!((k.flights_round1, k.bounds_moved), (1, false));
    assert_eq!(k.per_flight.len(), 1);
    within(k.per_flight[0], k.k2.unwrap(), 1e-9);
    assert_eq!(k.flights_min, k.flights_max);
    // The constant of the log is not the one the report was told.
    assert!((k.k1 - KF).abs() / KF > 0.001);
    assert_eq!(r.vibration.state, VibrationState::Done);
}

#[test]
fn health_counts() {
    let h = report(&full(), &inputs()).health;
    assert_eq!(h.dropped_fds, Some(7));
    assert_eq!(h.dropped_dsf, Some(2));
    assert_eq!(h.least_free_fds, Some(4000));
    assert_eq!(h.least_free_dsf, Some(100));
    // Spacing of NAV inside an armed window only: the long gaps are disarmed.
    assert_eq!(h.max_nav_gap_us, Some(60_000));
    // 3 in the first flight bin and 5 in the fifth; 4 in the dropped last bin and 2 on the ground do not count.
    assert_eq!(h.saturated_airborne, 8);
    for m in 0..4 {
        assert_eq!(h.esc_temp_max[m], Some(50.0 + m as f64));
    }
    // The second battery instance (3.0 V) is not the one reported.
    assert_eq!(h.battery_min_v, Some(14.5));
}

#[test]
fn poles_from_argument_parameter_or_nothing() {
    let mut i = inputs();
    let r = report(&full(), &i);
    assert_eq!(
        (r.poles, r.poles_source),
        (Some(POLES), PolesSource::Argument)
    );
    i.poles = None;
    // No pole count anywhere: no default; what needs the rotor speed is not computed, the rest is.
    let r = report(&full(), &i);
    assert_eq!((r.poles, r.poles_source), (None, PolesSource::Missing));
    assert_eq!(r.windows.len(), 2);
    assert!(r.windows.iter().all(|w| w.flights.is_empty()));
    assert!(r.identification.motors.is_none() && r.identification.pooled.is_none());
    assert!(r.identification.thrust.is_none());
    assert_eq!(r.vibration.state, VibrationState::NeedsPoles);
    assert!(r.vibration.intervals.is_empty());
    assert_eq!(r.health.max_nav_gap_us, Some(60_000));
    assert_eq!(r.health.battery_min_v, Some(14.5));
    let o = Opts {
        parm_poles: true,
        ..full()
    };
    let r = report(&o, &i);
    assert_eq!(
        (r.poles, r.poles_source),
        (Some(POLES), PolesSource::Parameter)
    );
    assert_eq!(r.windows[0].flights.len(), 1);
    // An argument wins over the parameter.
    i.poles = Some(POLES + 2.0);
    assert_eq!(report(&o, &i).poles_source, PolesSource::Argument);
}

/// A NAV and then the MOT of the same step, as the firmware writes them (the other records of a step are
/// left out).
fn step(f: &mut Vec<u8>, t: i64, armed: bool) {
    let mut nav = vec![int(t)];
    nav.extend((0..7).map(|_| V::Float(0.0)));
    nav.push(int(1));
    f.extend(record(136, NAV, &nav));
    let mut mot = vec![int(t), int(i64::from(armed)), int(0)];
    mot.extend((0..4).map(|_| int(100)));
    mot.extend((0..4).map(|_| int(0)));
    f.extend(record(130, MOT, &mot));
}

#[test]
fn the_nav_spacing_across_an_arm_or_disarm_edge_is_not_counted() {
    let mut f = header(&full());
    // Disarmed: every 8th step, 10 ms; armed: every step, 1.25 ms; two windows.
    let mut t = 0;
    while t < 90_000 {
        step(&mut f, t, false);
        t += 10_000;
    }
    for window in [100_000, 400_000] {
        let mut t = window;
        while t <= window + 20_000 {
            step(&mut f, t, true);
            t += 1_250;
        }
        for k in 1..=5 {
            step(&mut f, window + 20_000 + 10_000 * k, false);
        }
    }
    let r = flight_report_of(Cursor::new(f), None, &inputs()).unwrap();
    assert_eq!(r.windows.len(), 2);
    assert_eq!(r.health.max_nav_gap_us, Some(1_250));
}

#[test]
fn a_position_record_is_never_read() {
    let with = report(
        &Opts {
            gps: true,
            ..full()
        },
        &inputs(),
    );
    let without = report(&full(), &inputs());
    // Not even its time: the log's end is the same.
    assert_eq!(with.last_time_us, without.last_time_us);
    let mut with_file = with.clone();
    with_file.bytes = without.bytes;
    assert_eq!(with_file, without);
    let text = format!("{with:?}");
    for secret in [
        GPS_LAT.to_string(),
        GPS_LNG.abs().to_string(),
        "900000000".to_string(),
    ] {
        assert!(!text.contains(&secret), "{secret} in {text}");
    }
}

#[test]
fn a_log_without_esc_reports_what_it_can() {
    let r = report(
        &Opts {
            esc: false,
            ..full()
        },
        &inputs(),
    );
    assert!(!r.present.esc && r.present.mot && r.present.bat);
    assert_eq!(r.windows.len(), 2);
    assert!(r.windows.iter().all(|w| w.flights.is_empty()));
    assert!(r.identification.motors.is_none() && r.identification.thrust.is_none());
    assert_eq!(r.health.esc_temp_max, [None; 4]);
    assert_eq!(r.health.battery_min_v, Some(14.5));
    assert_eq!(r.health.dropped_fds, Some(7));
}

#[test]
fn a_log_that_ends_armed_ends_at_its_last_mot_record() {
    let mut f = header(&full());
    for t in [0, 10_000, 20_000] {
        f.extend(record(
            130,
            MOT,
            &[
                int(t),
                int(1),
                int(0),
                int(5),
                int(5),
                int(5),
                int(5),
                int(0),
                int(0),
                int(0),
                int(0),
            ],
        ));
    }
    let r = flight_report_of(Cursor::new(f), None, &inputs()).unwrap();
    assert_eq!(r.windows.len(), 1);
    assert_eq!((r.windows[0].start_us, r.windows[0].end_us), (0, 20_000));
    assert!(r.windows[0].open_at_end);
    assert!(r.windows[0].flights.is_empty());
}

#[test]
fn an_empty_log_and_bad_inputs() {
    let r = flight_report_of(Cursor::new(Vec::new()), None, &inputs()).unwrap();
    assert!(r.windows.is_empty() && r.duration_us().is_none());
    assert_eq!(r.present, Default::default());
    for bad in [
        ReportInputs {
            mass_kg: 0.0,
            ..inputs()
        },
        ReportInputs {
            kf: f64::NAN,
            ..inputs()
        },
        ReportInputs {
            poles: Some(-1.0),
            ..inputs()
        },
    ] {
        let e = flight_report_of(Cursor::new(Vec::new()), None, &bad).unwrap_err();
        assert_eq!(e.kind(), std::io::ErrorKind::InvalidInput);
    }
}

#[test]
fn a_pass_can_be_cancelled() {
    // Progress is reported every 256 KiB: pad the log past one report.
    let mut file = synth(&full());
    let filler = record(
        130,
        MOT,
        &[
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
            int(0),
        ],
    );
    while file.len() < 600_000 {
        file.extend(&filler);
    }
    let mut reader = Reader::new(Cursor::new(file), None);
    let r = flight_report_with(&mut reader, &inputs(), |_| Control::Stop).unwrap();
    assert!(r.is_none());
}

/// A log armed from its first record whose bin `i` holds every motor at `levels[i]`, with the IMU every
/// `imu_us`: `gyro_x` and `acc_z` (functions of the time in s) plus a little noise on every axis.
fn plain_log(
    levels: &[i64],
    imu_us: i64,
    gyro_x: impl Fn(f64) -> f64,
    acc_z: impl Fn(f64) -> f64,
) -> Vec<u8> {
    let mut f = header(&full());
    let mut seed = 12345u64;
    let mut noise = || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.04
    };
    let end = levels.len() as i64 * BIN;
    for t in (0..=end).step_by(500) {
        if t % imu_us == 0 {
            let s = t as f64 / 1e6;
            f.extend(record(
                142,
                IMU,
                &[
                    int(t),
                    int(0),
                    V::Float(gyro_x(s) + noise()),
                    V::Float(noise()),
                    V::Float(noise()),
                    V::Float(noise()),
                    V::Float(noise()),
                    V::Float(acc_z(s) + noise()),
                ],
            ));
        }
        if t % 10_000 == 0 {
            let level = levels[((t / BIN) as usize).min(levels.len() - 1)];
            let mut mot = vec![int(t), int(1), int(0)];
            mot.extend((0..4).map(|_| int(level)));
            mot.extend((0..4).map(|_| int(0)));
            f.extend(record(130, MOT, &mot));
            let mut esc = vec![int(t)];
            esc.extend((0..4).map(|m| {
                let (a, b) = line(m);
                int(erpm(a + b * level as f64))
            }));
            esc.extend((0..4).map(|_| int(0)));
            esc.extend((0..4).map(|_| int(40)));
            esc.push(int(0));
            f.extend(record(131, ESC, &esc));
        }
    }
    f
}

/// A tiny kf: no bin is ever a flight, so every interval is a ground run.
fn ground_inputs(lpf: &[f64]) -> ReportInputs {
    ReportInputs {
        kf: 1e-9,
        accel_lpf_hz: lpf.to_vec(),
        ..inputs()
    }
}

/// The lowest and highest rotor frequency (Hz) of the four motors at `level`.
fn rotor_hz(level: f64) -> (f64, f64) {
    let hz: Vec<f64> = (0..4)
        .map(|m| {
            let (a, b) = line(m);
            (a + b * level) / std::f64::consts::TAU
        })
        .collect();
    (
        hz.iter().copied().fold(f64::INFINITY, f64::min),
        hz.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    )
}

/// Two idle bins, then 22 bins at `level`.
fn run_levels(level: i64) -> Vec<i64> {
    let mut v = vec![100, 100];
    v.extend(std::iter::repeat_n(level, 22));
    v
}

fn ground_run(f: Vec<u8>, i: &ReportInputs) -> crate::IntervalVibration {
    let r = flight_report_of(Cursor::new(f), None, i).unwrap();
    assert_eq!(r.vibration.state, VibrationState::Done);
    assert_eq!(r.vibration.intervals.len(), 1, "{:?}", r.vibration);
    let v = r.vibration.intervals[0].clone();
    assert_eq!(v.kind, IntervalKind::GroundRun);
    assert_eq!((v.start_us, v.end_us), (1_000_000, 12_000_000));
    v
}

#[test]
fn a_sine_on_gyro_x_is_the_top_peak_with_its_frequency_and_amplitude() {
    let (f0, amp) = (87.3, 2.0);
    let log = plain_log(
        &run_levels(300),
        1000,
        |t| amp * (std::f64::consts::TAU * f0 * t).sin(),
        |_| GRAVITY,
    );
    let v = ground_run(log, &ground_inputs(&[]));
    let rate = v.rate_hz.unwrap();
    within(rate, 1000.0, 1e-9);
    assert!(!v.too_slow);
    within(v.gyro_std[0], amp / 2f64.sqrt(), 0.02);
    assert!(v.gyro_std[1] < 0.05 && v.accel_std[2] < 0.05 && v.accel_mag_std < 0.05);
    let sp = v.spectra.as_ref().expect("spectra");
    assert!(sp.segments > 15);
    let peak = &sp.axes[0].peaks[0];
    assert!((peak.hz - f0).abs() <= rate / 1024.0, "{}", peak.hz);
    within(peak.rms, amp / 2f64.sqrt(), 0.05);
    assert_eq!(sp.axes[0].peaks.len(), 3);
    // The rotor band is far from the sine: little of the variance of gyro X is in it.
    let (lo, hi) = v.rotor_hz.unwrap();
    let (elo, ehi) = rotor_hz(300.0);
    within(lo, elo, 1e-3);
    within(hi, ehi, 1e-3);
    let band = sp.bands.as_ref().unwrap();
    assert!(!band[0].folded && !band[1].folded);
    assert!(sp.axes[0].band_share.unwrap()[0] < 0.1);
}

#[test]
fn a_sine_at_the_rotor_frequency_is_in_the_rotor_band() {
    let (elo, ehi) = rotor_hz(300.0);
    let f0 = (elo + ehi) / 2.0;
    let log = plain_log(
        &run_levels(300),
        1000,
        |t| 1.5 * (std::f64::consts::TAU * f0 * t).sin(),
        |_| GRAVITY,
    );
    let v = ground_run(log, &ground_inputs(&[]));
    let sp = v.spectra.unwrap();
    let share = sp.axes[0].band_share.unwrap();
    assert!(share[0] > 0.9, "{share:?}");
    assert!(share[1] < 0.1, "{share:?}");
    // Gyro Y is noise: not concentrated in the band.
    assert!(sp.axes[1].band_share.unwrap()[0] < 0.5);
}

#[test]
fn a_doubled_band_above_half_the_rate_is_folded() {
    // Rotors near 130 Hz sampled at 400 Hz: twice the band is above 200 Hz and is seen folded.
    let level = 1180.0;
    let (elo, ehi) = rotor_hz(level);
    let f_true = elo + ehi; // twice the middle of the band
    let log = plain_log(
        &run_levels(level as i64),
        2500,
        |t| 1.5 * (std::f64::consts::TAU * f_true * t).sin(),
        |_| GRAVITY,
    );
    let v = ground_run(log, &ground_inputs(&[]));
    within(v.rate_hz.unwrap(), 400.0, 1e-9);
    assert!(!v.too_slow);
    let sp = v.spectra.unwrap();
    let bands = sp.bands.as_ref().unwrap();
    assert!(bands[1].folded && bands[1].hi_hz > 200.0);
    // |f - fs x round(f / fs)| of the ends of the doubled band.
    let fold = |f: f64| (f - 400.0 * (f / 400.0).round()).abs();
    within(bands[1].shown_lo_hz, fold(bands[1].lo_hz), 1e-9);
    within(bands[1].shown_hi_hz, fold(bands[1].hi_hz), 1e-9);
    assert!(sp.axes[0].band_share.unwrap()[1] > 0.9);
    // The sine shows at its alias.
    let alias = fold(f_true);
    assert!((sp.axes[0].peaks[0].hz - alias).abs() <= 400.0 / 1024.0);
}

#[test]
fn the_one_g_window_before_and_after_a_low_pass() {
    let tau = std::f64::consts::TAU;
    // 1 g plus a sine of half a g at 30 Hz: the magnitude leaves 0.9..1.1 g most of the time.
    let log = plain_log(
        &run_levels(300),
        1000,
        |_| 0.0,
        |t| GRAVITY * (1.0 + 0.5 * (tau * 30.0 * t).sin()),
    );
    let v = ground_run(log.clone(), &ground_inputs(&[1.0, 200.0]));
    let g = v.one_g.unwrap();
    assert!(g.raw > 0.05 && g.raw < 0.3, "{}", g.raw);
    assert_eq!(g.filtered.len(), 2);
    assert_eq!(g.filtered[0], (1.0, 1.0));
    // A cut-off far above the sine lets it through: nearly the raw share.
    assert!((g.filtered[1].1 - g.raw).abs() < 0.1, "{g:?}");
    // No cut-off given: only the raw share.
    let none = ground_run(log, &ground_inputs(&[])).one_g.unwrap();
    assert!(none.filtered.is_empty());
    assert_eq!(none.raw, g.raw);
}

#[test]
fn an_imu_at_100_hz_is_too_slow_for_spectra_and_the_rest_is_reported() {
    let r = report(&full(), &inputs());
    let v = &r.vibration.intervals;
    assert_eq!(v.len(), 1);
    assert_eq!(v[0].kind, IntervalKind::Flight);
    assert_eq!((v[0].window, v[0].number), (0, 1));
    within(v[0].rate_hz.unwrap(), 100.0, 1e-9);
    assert!(v[0].too_slow && v[0].spectra.is_none());
    // The second IMU (50 m/s^2) is ignored; the first is a constant 1 g.
    assert_eq!(v[0].one_g.as_ref().unwrap().raw, 1.0);
    assert!(v[0].accel_mag_std < 1e-6);
    assert_eq!(v[0].samples, 500);
}

#[test]
fn a_narrow_level_range_is_marked_ill_conditioned() {
    let levels = [100, 100, 500, 502, 504, 506, 508, 510, 512, 514];
    let r = flight_report_of(
        Cursor::new(plain_log(&levels, 10_000, |_| 0.0, |_| GRAVITY)),
        None,
        &ground_inputs(&[]),
    )
    .unwrap();
    for fit in r
        .identification
        .motors
        .as_ref()
        .unwrap()
        .iter()
        .chain(r.identification.pooled.as_ref())
    {
        let MotorFitResult::Fit(f) = fit else {
            panic!("{fit:?}")
        };
        assert!(f.ill_conditioned, "{f:?}");
    }
}

#[test]
fn a_hover_a_few_per_cent_under_the_given_kf_is_one_flight_from_its_first_bin() {
    // Every hover bin lies between 0.90 and 0.99 by the given kf (which is about 6 % high).
    let rs = [
        0.92, 0.90, 0.95, 0.99, 0.93, 0.96, 0.91, 0.97, 0.94, 0.98, 0.95,
    ];
    let mut levels = vec![100, 100, level_for_r(0.4)];
    levels.extend(rs.iter().map(|r| level_for_r(*r)));
    levels.extend([100, 100]);
    let r = flight_report_of(
        Cursor::new(plain_log(&levels, 10_000, |_| 0.0, |_| GRAVITY)),
        None,
        &inputs(),
    )
    .unwrap();
    assert_eq!(r.windows[0].flights.len(), 1, "{:?}", r.windows[0].flights);
    let f = &r.windows[0].flights[0];
    // From the first hover bin (bin 3); the last bin is dropped.
    assert_eq!(f.start_us, (3 * BIN) as u64);
    assert_eq!(f.end_us, (3 * BIN + 10 * BIN) as u64);
    let mg = MASS * GRAVITY;
    let truth = mg / (levels[3..13].iter().map(|l| sum_sq(*l as f64)).sum::<f64>() / 10.0);
    let t = r.identification.thrust.as_ref().unwrap();
    within(t.k2.unwrap(), truth, 0.01);
    assert!(r.largest_r.unwrap() > 0.98);
}

#[test]
fn a_run_up_that_peaks_at_80_per_cent_is_not_a_flight() {
    let up = level_for_r(0.8);
    let levels = [
        100,
        100,
        level_for_r(0.4),
        up,
        up,
        up,
        up,
        up,
        up,
        up,
        level_for_r(0.4),
        100,
        100,
    ];
    let r = flight_report_of(
        Cursor::new(plain_log(&levels, 10_000, |_| 0.0, |_| GRAVITY)),
        None,
        &inputs(),
    )
    .unwrap();
    assert!(r.windows[0].flights.is_empty());
    assert!(r.identification.thrust.is_none());
    let largest = r.largest_r.unwrap();
    assert!((largest - 0.8).abs() < 0.01, "{largest}");
}

// ---- tracking ----

/// The attitude quaternion [w, x, y, z] of roll, pitch, yaw in degrees (ZYX, body to NED).
fn quat(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let [r, p, y] = [roll, pitch, yaw].map(|d| d.to_radians() / 2.0);
    [
        r.cos() * p.cos() * y.cos() + r.sin() * p.sin() * y.sin(),
        r.sin() * p.cos() * y.cos() - r.cos() * p.sin() * y.sin(),
        r.cos() * p.sin() * y.cos() + r.sin() * p.cos() * y.sin(),
        r.cos() * p.cos() * y.sin() - r.sin() * p.sin() * y.cos(),
    ]
}

/// What the log holds at one 10 ms step: the setpoint kind, setpoint and estimate (roll, pitch, yaw in degrees)
/// and the body rates GX, GY (rad/s).
#[derive(Clone, Copy)]
struct Track {
    kind: i64,
    sp: [f64; 3],
    est: [f64; 3],
    rate: [f64; 2],
}

fn on_track(sp: [f64; 3], est: [f64; 3]) -> Track {
    Track {
        kind: 1,
        sp,
        est,
        rate: [0.0; 2],
    }
}

/// The flight of `track_levels`: from 1.5 s to 6.5 s.
const FL_START: i64 = 3 * BIN;
const FL_END: i64 = 13 * BIN;

fn track_levels() -> Vec<i64> {
    let mut levels = vec![100, 100, level_for_r(0.4)];
    levels.extend(std::iter::repeat_n(level_for_r(0.95), 11));
    levels.extend([100, 100]);
    levels
}

/// The steps (every 10 ms) of a log with one flight, and what it holds at each, `None`: no NAV and SETP.
fn track_steps(f: impl Fn(i64) -> Option<Track>) -> Vec<(i64, Track)> {
    let end = track_levels().len() as i64 * BIN;
    (0..=end)
        .step_by(STEP as usize)
        .filter_map(|t| f(t).map(|x| (t, x)))
        .collect()
}

/// A log armed from its first record with the flight of `track_levels`; `NAV` and `SETP` at the given steps
/// (every 10 ms), written in the firmware's order, `NAV`, `SETP`, `MOT`; without `SETP` when `setp` is false.
fn track_log(steps: &[(i64, Track)], setp: bool) -> Vec<u8> {
    let levels = track_levels();
    let mut f = Vec::new();
    f.extend(fmt_record(
        130,
        "MOT",
        MOT,
        "TimeUS,Arm,Mode,A0,A1,A2,A3,C0,C1,C2,C3",
    ));
    f.extend(fmt_record(
        131,
        "ESC",
        ESC,
        "TimeUS,E0,E1,E2,E3,V0,V1,V2,V3,T0,T1,T2,T3,Fl",
    ));
    if setp {
        f.extend(fmt_record(
            133,
            "SETP",
            SETP,
            "TimeUS,K,Q1,Q2,Q3,Q4,RX,RY,RZ,Thr",
        ));
    }
    f.extend(fmt_record(
        136,
        "NAV",
        NAV,
        "TimeUS,Q1,Q2,Q3,Q4,GX,GY,GZ,Vld",
    ));
    let end = levels.len() as i64 * BIN;
    for t in (0..=end).step_by(STEP as usize) {
        if let Some((_, s)) = steps.iter().find(|(st, _)| *st == t) {
            let q = quat(s.est[0], s.est[1], s.est[2]);
            let mut nav = vec![int(t)];
            nav.extend(q.map(V::Float));
            nav.extend([V::Float(s.rate[0]), V::Float(s.rate[1]), V::Float(0.0)]);
            nav.push(int(1));
            f.extend(record(136, NAV, &nav));
            if setp {
                let q = quat(s.sp[0], s.sp[1], s.sp[2]);
                let mut sp = vec![int(t), int(s.kind)];
                sp.extend(q.map(V::Float));
                sp.extend([V::Float(0.0), V::Float(0.0), V::Float(0.0), V::Float(0.0)]);
                f.extend(record(133, SETP, &sp));
            }
        }
        let level = levels[((t / BIN) as usize).min(levels.len() - 1)];
        let mut mot = vec![int(t), int(1), int(0)];
        mot.extend((0..4).map(|_| int(level)));
        mot.extend((0..4).map(|_| int(0)));
        f.extend(record(130, MOT, &mot));
        let mut esc = vec![int(t)];
        esc.extend((0..4).map(|m| {
            let (a, b) = line(m);
            int(erpm(a + b * level as f64))
        }));
        esc.extend((0..4).map(|_| int(0)));
        esc.extend((0..4).map(|_| int(40)));
        esc.push(int(0));
        f.extend(record(131, ESC, &esc));
    }
    f
}

fn tracked(steps: &[(i64, Track)]) -> crate::TrackingReport {
    let r = flight_report_of(Cursor::new(track_log(steps, true)), None, &inputs()).unwrap();
    let flights = &r.windows[0].flights;
    assert_eq!(flights.len(), 1, "{flights:?}");
    assert_eq!(
        (flights[0].start_us, flights[0].end_us),
        (FL_START as u64, FL_END as u64)
    );
    assert_eq!(r.tracking.state, crate::TrackingState::Done);
    assert_eq!(r.tracking.flights.len(), 1);
    r.tracking
}

fn inside(t: i64) -> bool {
    (FL_START..FL_END).contains(&t)
}

fn near_deg(got: f64, want: f64) {
    assert!((got - want).abs() < 1e-3, "{got} != {want}");
}

#[test]
fn a_constant_offset_gives_that_mean_rms_and_largest_error_on_every_axis() {
    let offset = [3.0, -2.0, 1.5];
    let sp = |t: i64| {
        let s = t as f64 / 1e6;
        [
            15.0 * (s * 1.3).sin(),
            -8.0 * (s * 0.7).cos(),
            30.0 + 5.0 * s,
        ]
    };
    let steps = track_steps(|t| {
        let sp = sp(t);
        Some(on_track(
            sp,
            [sp[0] - offset[0], sp[1] - offset[1], sp[2] - offset[2]],
        ))
    });
    let tr = tracked(&steps);
    let f = &tr.flights[0];
    let n = steps.iter().filter(|(t, _)| inside(*t)).count();
    assert_eq!(f.records, n as u64);
    let axes = f.axes.as_ref().unwrap();
    for a in 0..3 {
        let vals: Vec<f64> = steps
            .iter()
            .filter(|(t, _)| inside(*t))
            .map(|(_, s)| s.sp[a])
            .collect();
        near_deg(
            axes[a].setpoint_min,
            vals.iter().copied().fold(f64::INFINITY, f64::min),
        );
        near_deg(
            axes[a].setpoint_max,
            vals.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        );
        near_deg(axes[a].mean_error, offset[a]);
        near_deg(axes[a].rms_error, offset[a].abs());
        near_deg(axes[a].max_abs_error, offset[a]);
        assert!(inside(axes[a].max_abs_error_at_us as i64));
    }
}

#[test]
fn the_largest_error_is_found_at_its_time_and_only_inside_the_flight() {
    let spike = 4_000_000;
    let steps = track_steps(|t| {
        let e = if t == spike {
            -7.0
        } else if t == FL_START - STEP {
            50.0
        } else {
            1.0
        };
        Some(on_track([10.0, 0.0, 0.0], [10.0 - e, 0.0, 0.0]))
    });
    let f = &tracked(&steps).flights[0];
    let a = &f.axes.as_ref().unwrap()[0];
    near_deg(a.max_abs_error, -7.0);
    assert_eq!(a.max_abs_error_at_us, spike as u64);
    let n = f.records as f64;
    near_deg(a.mean_error, (n - 1.0 + -7.0) / n);
}

#[test]
fn an_estimate_past_the_recent_setpoint_range_is_an_excursion_and_a_lag_inside_it_is_not() {
    let (over, small, near_small, rate) = (4.0, 2.0, 3.0, 0.5);
    let (lo, hi) = (9.0, 11.0);
    let t_over = 3_500_000;
    let t_small = 5_500_000;
    let t_near = t_small + 400_000;
    let steps = track_steps(|t| {
        // Roll: a setpoint alternating between lo and hi, the estimate between them, with three
        // single-record overshoots.
        let wob = if (t / STEP) % 2 == 0 { -1.0 } else { 1.0 };
        let mut est = [(lo + hi) / 2.0, 0.0, 0.0];
        // The first records of the flight are exactly on target: the range of the window is then one record.
        let mut sp = if t < FL_START + 100_000 {
            est
        } else {
            [if wob < 0.0 { lo } else { hi }, wob, 0.0]
        };
        let mut rate_x = 0.0;
        if t == t_over {
            est[0] = hi + over;
            rate_x = rate;
        }
        if t == t_small {
            est[0] = lo - small;
        }
        if t == t_near {
            est[0] = hi + near_small;
        }
        // Pitch: a pulse of the setpoint from 4.5 s for 0.4 s that the estimate follows with a lag of 0.3 s,
        // up and down, always inside the range of the last 0.5 s.
        let s = t - 4_500_000;
        if (0..700_000).contains(&s) {
            sp[1] = if s < 400_000 { 20.0 } else { wob };
            est[1] = 19.0
                * if s < 300_000 {
                    s as f64 / 300_000.0
                } else if s < 400_000 {
                    1.0
                } else {
                    1.0 - (s - 400_000) as f64 / 300_000.0
                };
        }
        Some(Track {
            kind: 1,
            sp,
            est,
            rate: [rate_x, 0.0],
        })
    });
    let tr = tracked(&steps);
    let f = &tr.flights[0];
    let roll = f.excursions[0].as_ref().expect("a roll excursion");
    near_deg(roll.value, over);
    assert_eq!(roll.at_us, t_over as u64);
    near_deg(roll.estimate, hi + over);
    near_deg(roll.bound, hi);
    near_deg(roll.rate_dps, rate.to_degrees());
    assert!(f.excursions[1].is_none(), "{:?}", f.excursions[1]);
    // Events: 4 at 3.5 s, then 3 at 5.9 s; the 2 at 5.5 s lies within the separation of the 3.
    let ev = &tr.events[0];
    assert_eq!(ev.len(), 2, "{ev:?}");
    assert_eq!(ev[0].excursion.at_us, t_over as u64);
    near_deg(ev[0].excursion.value, over);
    assert_eq!(ev[1].excursion.at_us, t_near as u64);
    near_deg(ev[1].excursion.value, near_small);
    assert!(
        (t_near - t_small) < (crate::EXCURSION_SEPARATION_S * 1e6) as i64,
        "the test needs the two close"
    );
    assert!(tr.events[1].is_empty());
}

#[test]
fn an_overshoot_of_a_setpoint_that_moved_is_measured_against_the_last_half_second_only() {
    // The setpoint steps from 0 to 20 at 4 s; the estimate overshoots to 25 right after the step (5 over
    // the new top) and, later, when the window holds only 20, stays 1 over it.
    let step_at = 4_000_000;
    let steps = track_steps(|t| {
        let s = t - step_at;
        let (sp, est) = if s < 0 {
            (0.0, 0.0)
        } else if s < 100_000 {
            (20.0, 25.0)
        } else if s < 1_000_000 {
            (20.0, 19.0)
        } else {
            (20.0, 21.0)
        };
        Some(on_track([sp, 0.0, 0.0], [est, 0.0, 0.0]))
    });
    let tr = tracked(&steps);
    let roll = tr.flights[0].excursions[0].as_ref().unwrap();
    near_deg(roll.value, 5.0);
    near_deg(roll.bound, 20.0);
    assert_eq!(tr.events[0].len(), 2, "{:?}", tr.events[0]);
    near_deg(tr.events[0][1].excursion.value, 1.0);
}

#[test]
fn a_hold_is_found_with_its_start_duration_and_settled_error_and_short_or_small_runs_are_not() {
    let settle = (crate::HOLD_SETTLE_S * 1e6) as i64;
    let min_hold = (crate::HOLD_MIN_S * 1e6) as i64;
    let (roll_sp, roll_err) = (20.0, 2.0);
    let (roll_start, roll_len) = (2_000_000, 1_500_000);
    // Pitch: a hold at a negative angle (error -1), a run just under the minimum length, a run just under the
    // minimum angle.
    let (pitch_sp, pitch_err) = (-(crate::HOLD_MIN_DEG + 10.0), -1.0);
    let (pitch_start, pitch_len) = (5_000_000, 1_000_000);
    let (short_start, short_len) = (3_700_000, min_hold - 100_000);
    let (low_start, low_len) = (4_000_000, 900_000);
    let low_sp = crate::HOLD_MIN_DEG - 2.0;
    let steps = track_steps(|t| {
        // A drift of the setpoint within the tolerance, so that the mean is not the first value.
        let wob = if (t / STEP) % 2 == 0 {
            0.4 * crate::HOLD_TOL_DEG
        } else {
            0.0
        };
        let mut sp = [0.0; 3];
        let mut est = [0.0; 3];
        if (roll_start..roll_start + roll_len).contains(&t) {
            sp[0] = roll_sp + wob;
            est[0] = if t < roll_start + settle {
                roll_sp - 15.0
            } else {
                sp[0] - roll_err
            };
        }
        if (short_start..short_start + short_len).contains(&t) {
            sp[1] = 15.0 + wob;
            est[1] = sp[1];
        }
        if (low_start..low_start + low_len).contains(&t) {
            sp[1] = low_sp + wob;
            est[1] = sp[1];
        }
        if (pitch_start..pitch_start + pitch_len).contains(&t) {
            sp[1] = pitch_sp - wob;
            est[1] = if t < pitch_start + settle {
                0.0
            } else {
                sp[1] - pitch_err
            };
        }
        Some(on_track(sp, est))
    });
    let tr = tracked(&steps);
    assert_eq!(tr.holds[0].len(), 1, "{:?}", tr.holds[0]);
    assert_eq!(tr.holds[1].len(), 1, "{:?}", tr.holds[1]);
    let h = &tr.holds[0][0];
    assert_eq!(h.start_us, roll_start as u64);
    // The last record of the run is the one before the setpoint leaves.
    assert_eq!(h.duration_us, (roll_len - STEP) as u64);
    near_deg(h.settled_error, roll_err);
    // The mean setpoint of the settled part, from the data.
    let settled: Vec<f64> = steps
        .iter()
        .filter(|(t, _)| (roll_start + settle..roll_start + roll_len).contains(t))
        .map(|(_, s)| s.sp[0])
        .collect();
    near_deg(
        h.mean_setpoint,
        settled.iter().sum::<f64>() / settled.len() as f64,
    );
    near_deg(h.mean_estimate, h.mean_setpoint - roll_err);
    let p = &tr.holds[1][0];
    assert_eq!(p.start_us, pitch_start as u64);
    assert_eq!(p.duration_us, (pitch_len - STEP) as u64);
    near_deg(p.settled_error, pitch_err);
    assert_eq!(tr.hold_stats[0].count, 1);
    near_deg(tr.hold_stats[0].max_abs_error.unwrap(), roll_err);
    near_deg(tr.hold_stats[0].median_abs_error.unwrap(), roll_err);
    near_deg(tr.hold_stats[1].max_abs_error.unwrap(), pitch_err.abs());
}

#[test]
fn the_median_of_the_holds_is_taken_over_the_magnitudes() {
    let errs = [1.0, -3.0, 2.0, -6.0];
    let steps = track_steps(|t| {
        let mut sp = [0.0; 3];
        let mut est = [0.0; 3];
        for (i, e) in errs.iter().enumerate() {
            let start = FL_START + 200_000 + i as i64 * 1_000_000;
            if (start..start + 800_000).contains(&t) {
                sp[0] = 30.0;
                est[0] = 30.0 - e;
            }
        }
        Some(on_track(sp, est))
    });
    let tr = tracked(&steps);
    assert_eq!(tr.hold_stats[0].count, errs.len());
    near_deg(tr.hold_stats[0].median_abs_error.unwrap(), 2.5);
    near_deg(tr.hold_stats[0].max_abs_error.unwrap(), 6.0);
}

#[test]
fn records_of_another_setpoint_kind_are_left_out_and_counted() {
    let (t0, t1) = (3_000_000, 3_400_000);
    // Before the flight too: not counted.
    let early = 100_000..300_000;
    let steps = track_steps(|t| {
        let other = (t0..t1).contains(&t) || early.contains(&t);
        Some(Track {
            kind: if other { 2 } else { 1 },
            // Garbage in the other kind: it would show if it were used.
            sp: if other { [80.0, 80.0, 80.0] } else { [5.0; 3] },
            est: [4.0; 3],
            rate: [0.0; 2],
        })
    });
    let tr = tracked(&steps);
    let others = steps.iter().filter(|(t, _)| (t0..t1).contains(t)).count() as u64;
    assert_eq!(tr.other_kind_records, others);
    let in_flight = steps.iter().filter(|(t, _)| inside(*t)).count() as u64;
    assert_eq!(tr.flights[0].records, in_flight - others);
    let a = &tr.flights[0].axes.as_ref().unwrap()[0];
    near_deg(a.setpoint_max, 5.0);
    near_deg(a.mean_error, 1.0);
}

#[test]
fn a_log_without_setp_says_not_in_this_log() {
    let steps = track_steps(|_| Some(on_track([1.0; 3], [0.0; 3])));
    let r = flight_report_of(Cursor::new(track_log(&steps, false)), None, &inputs()).unwrap();
    assert!(!r.present.setp && r.present.nav);
    assert_eq!(r.tracking.state, crate::TrackingState::NotInLog);
    assert!(r.tracking.flights.is_empty());
}

#[test]
fn the_yaw_error_wraps_across_180_degrees() {
    for (sp, est, want) in [
        (175.0, -175.0, -10.0),
        (-175.0, 175.0, 10.0),
        (10.0, 20.0, -10.0),
    ] {
        let steps = track_steps(|_| Some(on_track([0.0, 0.0, sp], [0.0, 0.0, est])));
        let a = &tracked(&steps).flights[0].axes.as_ref().unwrap()[2].clone();
        near_deg(a.mean_error, want);
        near_deg(a.rms_error, 10.0);
        near_deg(a.max_abs_error, want);
    }
}

#[test]
fn a_flight_without_setpoint_records_has_no_figures() {
    let steps = track_steps(|t| (!inside(t)).then(|| on_track([1.0; 3], [0.0; 3])));
    let tr = tracked(&steps);
    assert_eq!(tr.flights[0].records, 0);
    assert!(tr.flights[0].axes.is_none());
}
