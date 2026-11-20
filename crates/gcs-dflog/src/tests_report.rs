//! The flight report, on synthetic logs built here: no real log, no real position anywhere.

use std::io::Cursor;

use crate::flight_report::{GRAVITY, flight_report_of};
use crate::tests::{V, fmt_record, int, record};
use crate::{
    Control, FlightReport, MotorFitResult, PolesSource, Reader, ReportInputs, flight_report_with,
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

const STEP: i64 = 10_000;
const BIN: i64 = 500_000;
const T0: i64 = 2_000_000;
const END1: i64 = T0 + 16 * BIN + 250_000;
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

/// The level of every motor at `t` (armed); the bins of the first window:
/// 0..2 ground, 3 half ground half hover, 4..13 hover (520, 540, ... 700), 14..16 ground.
fn level_at(t: i64) -> i64 {
    if !(T0..END1).contains(&t) {
        return 100;
    }
    let idx = (t - T0) / BIN;
    match idx {
        3 => {
            if (t - T0) % BIN < BIN / 2 {
                100
            } else {
                520
            }
        }
        4..=13 => 520 + 20 * (idx - 4),
        _ => 100,
    }
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
            let hover = on && (T0 + 4 * BIN..T0 + 14 * BIN).contains(&t);
            // Saturation: 3 records in the first hover bin (dropped from the segment), 5 in the
            // eighth (kept), 2 on the ground.
            let rel = t - T0;
            let sat = (hover && (0..60_000).contains(&(rel - 4 * BIN)))
                || (hover && (0..100_000).contains(&(rel - 8 * BIN)))
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

#[test]
fn windows_segment_fit_and_constant() {
    let r = report(&full(), &inputs());
    assert_eq!(r.bad_bytes, 0);
    assert_eq!(r.first_time_us, Some(0));
    assert_eq!(r.windows.len(), 2);
    let w = &r.windows[0];
    assert_eq!(
        (w.start_us, w.end_us, w.open_at_end),
        (T0 as u64, END1 as u64, false)
    );
    assert_eq!(w.segments.len(), 1);
    // Hover bins 4..=13 are airborne; the first and last of the run go: 5..=12.
    let s = &w.segments[0];
    assert_eq!(s.start_us, (T0 + 5 * BIN) as u64);
    assert_eq!(s.end_us, (T0 + 13 * BIN) as u64);
    assert_eq!(s.bins, 8);
    // The second window is armed and never airborne.
    let w2 = &r.windows[1];
    assert_eq!((w2.start_us, w2.end_us), (T2 as u64, END2 as u64));
    assert!(w2.segments.is_empty());

    let mg = MASS * GRAVITY;
    let levels: Vec<f64> = (5..=12).map(|i| (520 + 20 * (i - 4)) as f64).collect();
    let mean_ssq = levels.iter().map(|l| sum_sq(*l)).sum::<f64>() / 8.0;
    within(s.speed_sq_sum, mean_ssq, 0.01);
    within(s.thrust_ratio, KF * mean_ssq / mg, 0.01);
    within(s.stick_mean.unwrap(), 0.55, 1e-6);
    within(s.collective_n.unwrap(), 11.0, 1e-6);
    within(s.collective_fraction.unwrap(), 11.0 / mg, 1e-6);
    for m in 0..4 {
        let (a, b) = line(m);
        let rpm =
            levels.iter().map(|l| a + b * l).sum::<f64>() / 8.0 * 60.0 / std::f64::consts::TAU;
        within(s.rpm[m], rpm, 0.001);
    }
    assert_eq!(s.volts_min, Some(14.5));
    assert_eq!(s.amps_max, Some(30.0));
    let amps = s.amps_mean.unwrap();
    assert!(amps > 24.9 && amps < 26.0, "{amps}");
    within(s.torque[0].unwrap(), 0.1, 1e-5);
    within(s.torque[1].unwrap(), -0.2, 1e-5);
    within(s.torque[2].unwrap(), 0.05, 1e-5);

    // The motor identification: 10 hover bins, the idle and the mixed bin out.
    let fits = r.identification.motors.as_ref().unwrap();
    for (m, fit) in fits.iter().enumerate() {
        let MotorFitResult::Fit(f) = fit else {
            panic!("motor {m}: {fit:?}")
        };
        let (a, b) = line(m);
        within(f.a, a, 0.01);
        within(f.b, b, 0.01);
        assert_eq!(f.bins, 10);
        assert_eq!((f.level_min, f.level_max), (520.0, 700.0));
        assert_eq!(f.idle_level, 100.0);
        assert!(f.rms_residual < 0.05, "{}", f.rms_residual);
        let v = f.mean_volts.unwrap();
        assert!(v > 14.9 && v <= 15.0, "{v}");
    }
    let k = r.identification.thrust.as_ref().unwrap();
    within(k.k, mg / mean_ssq, 0.01);
    assert_eq!(k.per_segment.len(), 1);
    within(k.per_segment[0], k.k, 1e-9);
    assert_eq!(k.segments_min, k.segments_max);
    // The constant of the log is not the one the report was told.
    assert!((k.k - KF).abs() / KF > 0.001);
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
    // 5 in a kept bin; 3 in the dropped first bin and 2 on the ground do not count.
    assert_eq!(h.saturated_airborne, 5);
    for m in 0..4 {
        assert_eq!(h.esc_temp_max[m], Some(50.0 + m as f64));
    }
    // The second battery instance (3.0 V) is not the one reported.
    assert_eq!(h.battery_min_v, Some(14.5));
}

#[test]
fn poles_from_argument_parameter_or_default() {
    let mut i = inputs();
    let r = report(&full(), &i);
    assert_eq!((r.poles, r.poles_source), (POLES, PolesSource::Argument));
    i.poles = None;
    let r = report(&full(), &i);
    assert_eq!(
        (r.poles, r.poles_source),
        (crate::DEFAULT_POLES, PolesSource::Default)
    );
    let o = Opts {
        parm_poles: true,
        ..full()
    };
    let r = report(&o, &i);
    assert_eq!((r.poles, r.poles_source), (POLES, PolesSource::Parameter));
    assert_eq!(r.windows[0].segments.len(), 1);
    // An argument wins over the parameter.
    i.poles = Some(POLES + 2.0);
    assert_eq!(report(&o, &i).poles_source, PolesSource::Argument);
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
    assert!(r.windows.iter().all(|w| w.segments.is_empty()));
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
    assert!(r.windows[0].segments.is_empty());
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
