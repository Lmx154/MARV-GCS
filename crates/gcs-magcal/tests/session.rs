//! The session state machine (decision sections 2-4, 9 invariants I5 and I8): a scripted user performing six poses
//! with one turn each on synthetic data, the endings, the refusals to start and the write plan.

use gcs_magcal::Vec3;
use gcs_magcal::accept::{self, Refusal};
use gcs_magcal::pose::{ONE_G, Pose};
use gcs_magcal::session::{
    CAL_TYPE_GUIDED, Ending, Gps, Outcome, POSE_TIMEOUT_S, ParamValue, Session, StartRefusal,
    Status, WATCHED_PARAMS, verify_read_back,
};
use gcs_magcal::undo::Calibration;
use gcs_magcal::wmm;

mod common;
use common::*;

const S_TRUE: M3 = [[1.06, 0.04, -0.05], [0.04, 0.95, 0.06], [-0.05, 0.06, 0.78]];
const O_TRUE: Vec3 = [32.0, -18.5, 47.0];
const NOISE_UT: f64 = 0.3;
/// One of NOAA's published WMM2025 test locations (the 100-row table: 2026.5, 12 km, 33 N, 145 W).
const SITE: (f64, f64, f64, f64) = (33.0, -145.0, 12_000.0, 2026.5);
const HZ: f64 = 25.0;
const DT: f64 = 1.0 / HZ;

/// The contract's test vector as the calibration in force: a matrix far from identity.
fn in_force() -> Calibration {
    Calibration {
        offsets: [-14.2, -4.2, 1.5],
        diag: [1.0625, 0.9375, 0.78125],
        offdiag: [0.03125, -0.046875, 0.0625],
    }
}

fn gps() -> Gps {
    Gps {
        fix_type: 3,
        age_s: 0.4,
        lat_deg: SITE.0,
        lon_deg: SITE.1,
        height_m: SITE.2,
    }
}

fn start_at(now: f64) -> Session {
    Session::start(in_force(), Some(gps()), SITE.3, now).expect("starts")
}

/// One synthetic HIGHRES_IMU sample.
#[derive(Clone, Copy)]
struct Imu {
    t_usec: u64,
    accel: [f32; 3],
    gyro: [f32; 3],
    mag_gauss: [f32; 3],
}

struct Scene {
    ned: Vec3,
    rng: Rng,
    noise_ut: f64,
}

impl Scene {
    fn new(noise_ut: f64) -> Self {
        let f = wmm::field(SITE.0, SITE.1, SITE.2, SITE.3).unwrap();
        Scene {
            ned: f.ned_ut,
            rng: Rng(0x00C0_FFEE),
            noise_ut,
        }
    }

    /// The sample of a vehicle with attitude `att` (body-from-NED) turning at body rate `gyro`: the field the
    /// vehicle publishes (the calibration in force applied to the true raw field), in gauss.
    fn sample(&mut self, t: f64, att: &M3, gyro: Vec3) -> Imu {
        let b = mv(att, self.ned);
        let r = mv(&inv3(&S_TRUE), b);
        let r = [0, 1, 2].map(|i| r[i] + O_TRUE[i] + self.noise_ut * self.rng.gauss());
        let m = in_force().apply(r);
        let down = mv(att, [0.0, 0.0, 1.0]);
        Imu {
            t_usec: (t * 1e6).round() as u64,
            accel: to_f32(down.map(|d| -ONE_G * d)),
            gyro: to_f32(gyro),
            mag_gauss: to_f32(m.map(|v| v / 100.0)),
        }
    }

    /// Attitudes at 25 Hz of a user who holds `pose` for `hold` s, turns once round at `deg_s`, then holds again.
    fn pose_turn(&self, pose: usize, yaw0: f64, deg_s: f64, hold: f64) -> Vec<M3> {
        let mut v = Vec::new();
        let n_hold = (hold * HZ) as usize;
        let n_turn = (360.0 / deg_s * HZ).round() as usize;
        for _ in 0..n_hold {
            v.push(pose_attitude(pose, yaw0));
        }
        for k in 1..=n_turn {
            v.push(pose_attitude(pose, yaw0 + 360.0 * k as f64 / n_turn as f64));
        }
        for _ in 0..n_hold {
            v.push(pose_attitude(pose, yaw0));
        }
        v
    }
}

/// The attitudes of a vehicle carried from `a` to `b` in `seconds`: a smooth rotation about one axis.
fn transition(a: &M3, b: &M3, seconds: f64) -> Vec<M3> {
    let rel = mm(b, &transpose(a));
    let (axis, angle) = axis_angle(&rel);
    let n = (seconds * HZ) as usize;
    (1..=n)
        .map(|k| {
            let s = k as f64 / n as f64;
            let s = s * s * (3.0 - 2.0 * s);
            mm(&rotation(axis, angle * s), a)
        })
        .collect()
}

/// The six poses in `order`, one turn each, with 1.5 s between poses.
fn script(scene: &Scene, order: &[usize], deg_s: f64) -> Vec<M3> {
    let mut all: Vec<M3> = Vec::new();
    for (k, &p) in order.iter().enumerate() {
        let leg = scene.pose_turn(p, 20.0 * k as f64, deg_s, 0.5);
        if let Some(last) = all.last() {
            let t = transition(last, &leg[0], 1.5);
            all.extend(t);
        }
        all.extend(leg);
    }
    all
}

/// Feeds `atts[range]` through the session at 25 Hz from time `t0`, `tick` at each sample's time; the gyro is the
/// body rate between consecutive attitudes. Stops at the first sample after which the session is no longer
/// running. Returns the time of the next sample.
fn run_range(
    session: &mut Session,
    scene: &mut Scene,
    atts: &[M3],
    range: std::ops::Range<usize>,
    t0: f64,
) -> f64 {
    let mut t = t0;
    for k in range.start..range.end.min(atts.len()) {
        let next = atts.get(k + 1).unwrap_or(&atts[k]);
        let imu = scene.sample(t, &atts[k], body_rate(&atts[k], next, DT));
        session.tick(t);
        session.sample(imu.t_usec, imu.accel, imu.gyro, imu.mag_gauss);
        if !session.is_running() {
            break;
        }
        t += DT;
    }
    t
}

fn run(session: &mut Session, scene: &mut Scene, atts: &[M3], t0: f64) -> f64 {
    run_range(session, scene, atts, 0..atts.len(), t0)
}

const ORDER: [usize; 6] = [0, 2, 1, 4, 3, 5];

#[test]
fn a_scripted_user_with_six_turns_is_accepted_and_the_plan_is_in_contract_order() {
    let mut scene = Scene::new(NOISE_UT);
    let atts = script(&scene, &ORDER, 30.0);
    let mut session = start_at(0.0);
    // The first 30 s, then a look at the status, then the rest.
    let split = (30.0 * HZ) as usize;
    let t_mid = run_range(&mut session, &mut scene, &atts, 0..split, 0.0);
    let mid: Status = session.status(t_mid);
    let t = run_range(&mut session, &mut scene, &atts, split..atts.len(), t_mid);
    assert!(mid.poses.iter().filter(|p| p.complete).count() >= 1);
    assert!(mid.samples_kept > 20);
    assert!(mid.seconds_left > 0.0 && mid.seconds_left <= POSE_TIMEOUT_S);

    let Outcome::Accepted(done) = session.outcome().clone() else {
        panic!(
            "not accepted: {:?} after {t:.1} s",
            session.outcome().message()
        );
    };
    let a = &done.accepted;
    // Recovery as in the existing I8 test (tests/fit.rs, six_rings_recover_s_and_o).
    let s = a.calibration.matrix();
    let mut worst_o = 0.0f64;
    let (mut worst_diag_rel, mut worst_off) = (0.0f64, 0.0f64);
    for i in 0..3 {
        worst_o = worst_o.max((a.calibration.offsets[i] - O_TRUE[i]).abs());
        for j in 0..3 {
            let e = s[i][j] - S_TRUE[i][j];
            if i == j {
                worst_diag_rel = worst_diag_rel.max(e.abs() / S_TRUE[i][i]);
            } else {
                worst_off = worst_off.max(e.abs());
            }
        }
    }
    println!(
        "scripted user: {} samples kept, {:.1} s, fitness {:.3} uT, correction {:.4}, worst offset error {worst_o:.3} uT, \
         worst diagonal error {:.2} %, worst off-diagonal error {worst_off:.4}, dip {:.2} deg (model {:.2})",
        done.samples,
        t,
        a.fitness_ut,
        a.correction,
        worst_diag_rel * 100.0,
        a.measured_inclination_deg,
        a.inclination_deg
    );
    assert!(worst_o < 0.5);
    assert!(worst_diag_rel < 0.02);
    assert!(worst_off < 0.02);
    assert!(a.fitness_ut < 0.6);
    assert!((a.measured_inclination_deg - a.inclination_deg).abs() < 1.0);
    assert_eq!(a.field_ut, session.model().intensity_ut);
    assert_eq!(done.samples, session.status(t).samples_kept);
    assert_eq!(
        session.outcome().message().unwrap(),
        accept::ACCEPTED_MESSAGE
    );
    assert!(session.status(t).poses.iter().all(|p| p.complete));

    // The plan: the contract's order, 13 writes.
    let names: Vec<&str> = done.plan.iter().map(|w| w.0).collect();
    assert_eq!(
        names,
        [
            "MAG_CAL_TYPE",
            "MAG_OFFX",
            "MAG_OFFY",
            "MAG_OFFZ",
            "MAG_DIAGX",
            "MAG_DIAGY",
            "MAG_DIAGZ",
            "MAG_ODIAGXY",
            "MAG_ODIAGXZ",
            "MAG_ODIAGYZ",
            "MAG_FLD_UT",
            "MAG_FLD_INC",
            "MAG_CAL_TYPE"
        ]
    );
    assert_eq!(done.plan.len(), 13);
    assert_eq!(done.plan[0].1, ParamValue::I32(0));
    assert_eq!(done.plan[12].1, ParamValue::I32(CAL_TYPE_GUIDED));
    assert_eq!(
        done.plan[1].1,
        ParamValue::F32(a.calibration.offsets[0] as f32)
    );
    assert_eq!(
        done.plan[6].1,
        ParamValue::F32(a.calibration.diag[2] as f32)
    );
    assert_eq!(
        done.plan[9].1,
        ParamValue::F32(a.calibration.offdiag[2] as f32)
    );
    assert_eq!(done.plan[10].1, ParamValue::F32(a.field_ut as f32));
    assert_eq!(done.plan[11].1, ParamValue::F32(a.inclination_deg as f32));

    // The read-back check: what a vehicle that stored the plan returns passes; any difference is named.
    // (MAG_CAL_TYPE is written twice; the vehicle holds the last value, so it is read back once.)
    let mut back: Vec<(&str, f32)> = done.plan[1..]
        .iter()
        .map(|&(n, v)| (n, v.as_f32()))
        .collect();
    assert_eq!(back.len(), 12);
    assert_eq!(verify_read_back(&done.plan, &back), Ok(()));
    back[3].1 += 1e-4;
    assert_eq!(
        verify_read_back(&done.plan, &back)
            .unwrap_err()
            .split(' ')
            .next(),
        Some("MAG_DIAGX")
    );
    back[3].1 = done.plan[4].1.as_f32();
    back[11].1 = 1.0; // MAG_CAL_TYPE left at 1: the last write is what counts
    assert!(verify_read_back(&done.plan, &back).is_err());
    back.pop();
    back.pop();
    assert!(
        verify_read_back(&done.plan, &back)
            .unwrap_err()
            .contains("MAG_FLD_INC")
    );
    // A later sample or event changes nothing.
    session.cancel();
    assert!(matches!(session.outcome(), Outcome::Accepted(_)));
}

#[test]
fn one_pose_skipped_times_out_naming_it() {
    let mut scene = Scene::new(NOISE_UT);
    // Five poses turned; right side down never taken. The user then stands holding the last pose (already
    // complete) until the 90 s run out.
    let mut atts = script(&scene, &ORDER[..5], 30.0);
    let last = *atts.last().unwrap();
    atts.extend(std::iter::repeat_n(last, (100.0 * HZ) as usize));
    let mut session = start_at(0.0);
    let t = run(&mut session, &mut scene, &atts, 0.0);
    assert_eq!(
        session.outcome().message().unwrap(),
        "Timed out waiting for right side down: start again"
    );
    assert_eq!(
        session.outcome(),
        &Outcome::Ended(Ending::TimedOut(Pose::RightSideDown))
    );
    let st = session.status(t);
    assert_eq!(st.poses.iter().filter(|p| p.complete).count(), 5);
    assert_eq!(
        st.instruction,
        "Timed out waiting for right side down: start again"
    );
    // Nothing is accepted later.
    assert!(!matches!(session.outcome(), Outcome::Accepted(_)));
}

#[test]
fn the_pose_being_held_is_named_when_it_is_incomplete_else_the_first_incomplete() {
    let mut scene = Scene::new(NOISE_UT);
    // Level turned; then nose down held still, never turned.
    let mut atts = script(&scene, &ORDER[..1], 30.0);
    let to = transition(atts.last().unwrap(), &pose_attitude(2, 0.0), 1.5);
    atts.extend(to);
    atts.extend(std::iter::repeat_n(
        pose_attitude(2, 0.0),
        (100.0 * HZ) as usize,
    ));
    let mut session = start_at(0.0);
    run(&mut session, &mut scene, &atts, 0.0);
    assert_eq!(
        session.outcome(),
        &Outcome::Ended(Ending::TimedOut(Pose::NoseDown))
    );
    // Nothing held at all: the first incomplete pose in the order.
    let mut s = start_at(10.0);
    assert!(s.tick(99.9).message().is_none());
    s.tick(100.0);
    assert_eq!(s.outcome(), &Outcome::Ended(Ending::TimedOut(Pose::Level)));
}

#[test]
fn ninety_seconds_restart_when_a_pose_completes() {
    let mut scene = Scene::new(NOISE_UT);
    // 15 s for level, then a still pose until 15 + 89 s: not yet expired; 15 + 91 s: expired.
    let mut atts = script(&scene, &ORDER[..1], 30.0);
    let t_level = atts.len() as f64 * DT;
    atts.extend(std::iter::repeat_n(
        *atts.last().unwrap(),
        (95.0 * HZ) as usize,
    ));
    let mut session = start_at(0.0);
    let mut t = 0.0;
    let mut completed_at = None;
    for k in 0..atts.len() {
        let next = atts.get(k + 1).unwrap_or(&atts[k]);
        let imu = scene.sample(t, &atts[k], body_rate(&atts[k], next, DT));
        session.tick(t);
        session.sample(imu.t_usec, imu.accel, imu.gyro, imu.mag_gauss);
        if completed_at.is_none() && session.status(t).poses[0].complete {
            completed_at = Some(t);
        }
        if !session.is_running() {
            break;
        }
        t += DT;
    }
    let c = completed_at.expect("level completed");
    assert!(c < t_level);
    assert!(
        t - c >= POSE_TIMEOUT_S - 1.0 && t - c <= POSE_TIMEOUT_S + 1.0,
        "expired {:.1} s after it",
        t - c
    );
}

#[test]
fn a_parameter_change_armed_and_cancel_end_the_run_without_a_plan() {
    for name in WATCHED_PARAMS {
        let mut s = start_at(0.0);
        s.param_changed(name);
        assert_eq!(
            s.outcome(),
            &Outcome::Ended(Ending::ParamChanged(name.to_string()))
        );
        let m = s.outcome().message().unwrap();
        assert!(m.contains(name) && m.contains("start again"), "{m}");
        // A sample afterwards is ignored.
        s.sample(1_000_000, [0.0, 0.0, -9.8], [0.0; 3], [0.2, 0.0, 0.4]);
        assert_eq!(s.status(1.0).samples_kept, 0);
    }
    assert_eq!(WATCHED_PARAMS.len(), 9);
    for name in [
        "MAG_CAL_TYPE",
        "MAG_FLD_UT",
        "MAG_FLD_INC",
        "MAG_HLT_STR",
        "GPS_RATE_MS",
    ] {
        let mut s = start_at(0.0);
        assert!(s.param_changed(name).message().is_none(), "{name}");
    }
    let mut s = start_at(0.0);
    s.armed();
    assert_eq!(s.outcome(), &Outcome::Ended(Ending::Armed));
    assert_eq!(
        s.outcome().message().unwrap(),
        "The vehicle was armed: calibration stopped"
    );
    let mut s = start_at(0.0);
    s.cancel();
    assert_eq!(s.outcome(), &Outcome::Ended(Ending::Cancelled));
    assert_eq!(s.outcome().message().unwrap(), "Calibration cancelled");
    // The first ending stands.
    s.armed();
    s.param_changed("MAG_OFFX");
    assert_eq!(s.outcome(), &Outcome::Ended(Ending::Cancelled));
    assert_eq!(s.status(5.0).seconds_left, 0.0);
}

#[test]
fn start_refuses_without_a_usable_position_or_a_valid_matrix() {
    const TEXT: &str =
        "Needs a GPS position to know the Earth's field here; take the vehicle outside";
    let start = |g: Option<Gps>| Session::start(in_force(), g, SITE.3, 0.0).map(|_| ());
    assert_eq!(start(None).unwrap_err().message(), TEXT);
    assert_eq!(
        start(Some(Gps {
            fix_type: 1,
            ..gps()
        }))
        .unwrap_err(),
        StartRefusal::NoPosition
    );
    assert_eq!(
        start(Some(Gps {
            fix_type: 0,
            ..gps()
        }))
        .unwrap_err()
        .message(),
        TEXT
    );
    assert_eq!(
        start(Some(Gps {
            age_s: 100.0,
            ..gps()
        }))
        .unwrap_err()
        .message(),
        TEXT
    );
    assert_eq!(
        start(Some(Gps {
            age_s: 250.0,
            ..gps()
        }))
        .unwrap_err()
        .message(),
        TEXT
    );
    assert_eq!(
        start(Some(Gps {
            age_s: f64::NAN,
            ..gps()
        }))
        .unwrap_err()
        .message(),
        TEXT
    );
    assert!(
        start(Some(Gps {
            age_s: 99.9,
            ..gps()
        }))
        .is_ok()
    );
    assert!(
        start(Some(Gps {
            fix_type: 2,
            ..gps()
        }))
        .is_ok()
    );
    assert!(
        start(Some(Gps {
            fix_type: 6,
            ..gps()
        }))
        .is_ok()
    );

    for bad in [
        Calibration {
            diag: [1.0, -1.0, 1.0],
            ..Calibration::default()
        },
        Calibration {
            offdiag: [2.0, 0.0, 0.0],
            ..Calibration::default()
        },
    ] {
        let e = Session::start(bad, Some(gps()), SITE.3, 0.0)
            .map(|_| ())
            .unwrap_err();
        assert_eq!(e, StartRefusal::InvalidMatrix);
        assert_eq!(
            e.message(),
            "The stored compass matrix is not valid. Reset the compass calibration first."
        );
        // The matrix is judged before the position.
        assert_eq!(
            Session::start(bad, None, SITE.3, 0.0)
                .map(|_| ())
                .unwrap_err(),
            StartRefusal::InvalidMatrix
        );
    }
    // A date the model does not cover.
    let e = Session::start(in_force(), Some(gps()), 2031.0, 0.0)
        .map(|_| ())
        .unwrap_err();
    assert!(matches!(e, StartRefusal::Model(_)));
    assert!(
        e.message()
            .starts_with("The Earth's field model cannot be used")
    );
}

#[test]
fn the_status_before_any_sample_says_hold_it_as_shown() {
    let s = start_at(5.0);
    let st = s.status(5.0);
    assert_eq!(st.recognised, None);
    assert_eq!(st.instruction, "hold it as shown");
    assert_eq!(st.target, Some(Pose::Level));
    assert_eq!(st.samples_kept, 0);
    assert_eq!(st.seconds_left, POSE_TIMEOUT_S);
    assert!(st.poses.iter().all(|p| !p.complete && p.visited_count == 0));
    assert_eq!(s.status(35.0).seconds_left, 60.0);
    assert_eq!(st.poses.map(|p| p.pose), Pose::ALL);
}

#[test]
fn a_scattered_run_is_refused_with_the_acceptance_text_and_no_plan() {
    let mut scene = Scene::new(8.0);
    let atts = script(&scene, &ORDER, 30.0);
    let mut session = start_at(0.0);
    run(&mut session, &mut scene, &atts, 0.0);
    match session.outcome() {
        Outcome::Ended(Ending::Refused(Refusal::TooScattered { fitness_ut })) => {
            assert!(*fitness_ut > accept::FITNESS_LIMIT_UT);
            assert!(
                session
                    .outcome()
                    .message()
                    .unwrap()
                    .starts_with("Readings too scattered")
            );
        }
        other => panic!("{other:?}"),
    }
}
