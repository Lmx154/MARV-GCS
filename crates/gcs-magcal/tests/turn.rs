//! Turn progress (decision section 3): the ring of a pose, forwards, backwards, with a reversal, the clearing rule
//! and the gap safeguard.

use gcs_magcal::pose::{ONE_G, Pose};
use gcs_magcal::turn::{MAX_STEP_S, SECTORS, TurnTracker};

/// A user holding a pose and turning, one sample at a time at 25 Hz.
struct Hold {
    tracker: TurnTracker,
    t_usec: u64,
}

const DT_USEC: u64 = 40_000;

impl Hold {
    fn new() -> Self {
        Hold {
            tracker: TurnTracker::new(),
            t_usec: 1_000_000,
        }
    }

    /// One sample in `pose` turning at `deg_s` about the vertical (the gyro is rate times the unit down vector).
    fn step(&mut self, pose: Pose, deg_s: f64) -> gcs_magcal::turn::Update {
        let a = pose.expected_accel();
        let down = [-a[0] / ONE_G, -a[1] / ONE_G, -a[2] / ONE_G];
        let w = deg_s.to_radians();
        self.t_usec += DT_USEC;
        self.tracker
            .update(self.t_usec, a, [w * down[0], w * down[1], w * down[2]])
    }

    /// Turns `deg` degrees at `deg_s` (sign of `deg` gives the direction).
    fn turn(&mut self, pose: Pose, deg: f64, deg_s: f64) {
        let steps = (deg.abs() / (deg_s * 0.04)).round() as usize;
        for _ in 0..steps {
            self.step(pose, deg_s * deg.signum());
        }
    }

    fn count(&self, pose: Pose) -> usize {
        self.tracker.ring(pose).visited_count()
    }
}

#[test]
fn a_full_turn_forwards_completes_and_reports_it_once() {
    let mut h = Hold::new();
    let mut completions = 0;
    h.step(Pose::Level, 0.0);
    assert_eq!(h.count(Pose::Level), 1);
    for _ in 0..(12.5 / 0.04) as usize {
        let u = h.step(Pose::Level, 30.0);
        if u.completed == Some(Pose::Level) {
            completions += 1;
        }
    }
    assert!(h.tracker.is_complete(Pose::Level));
    assert_eq!(completions, 1);
    assert!(!h.tracker.all_complete());
    assert_eq!(h.tracker.incomplete().len(), 5);
    assert_eq!(h.tracker.incomplete()[0], Pose::UpsideDown);
}

#[test]
fn a_turn_of_300_degrees_is_not_enough_and_backwards_works() {
    for dir in [1.0, -1.0] {
        let mut h = Hold::new();
        h.step(Pose::NoseUp, 0.0);
        h.turn(Pose::NoseUp, 300.0 * dir, 30.0);
        assert!(!h.tracker.is_complete(Pose::NoseUp), "dir {dir}");
        assert!(h.count(Pose::NoseUp) >= 10 && h.count(Pose::NoseUp) < SECTORS);
        h.turn(Pose::NoseUp, 70.0 * dir, 30.0);
        assert!(h.tracker.is_complete(Pose::NoseUp), "dir {dir}");
    }
}

#[test]
fn a_reversal_keeps_the_visited_sectors() {
    let mut h = Hold::new();
    h.step(Pose::Level, 0.0);
    h.turn(Pose::Level, 100.0, 30.0); // sectors 0..3
    h.turn(Pose::Level, -200.0, 30.0); // back through 0 to -100: sectors -4..3
    h.turn(Pose::Level, 300.0, 30.0); // forward to +200
    assert!(!h.tracker.is_complete(Pose::Level));
    assert_eq!(h.count(Pose::Level), 11);
    h.turn(Pose::Level, 30.0, 30.0); // reaches sector 7 (210-240 deg)
    assert!(h.tracker.is_complete(Pose::Level));
}

#[test]
fn a_fast_turn_marks_every_sector_it_passes() {
    let mut h = Hold::new();
    h.step(Pose::Level, 0.0);
    // At 25 Hz and 1000 deg/s one step is 40 deg, more than a sector: the sectors in between are still marked.
    h.turn(Pose::Level, 360.0, 1000.0);
    assert!(h.tracker.is_complete(Pose::Level));
}

#[test]
fn another_pose_clears_an_incomplete_ring_but_none_does_not() {
    let mut h = Hold::new();
    h.step(Pose::Level, 0.0);
    h.turn(Pose::Level, 100.0, 30.0);
    let before = h.count(Pose::Level);
    assert!(before > 1);
    // The reading falls to "none" (in the air): nothing is cleared, and no turn is counted meanwhile.
    for _ in 0..50 {
        h.t_usec += DT_USEC;
        let u = h
            .tracker
            .update(h.t_usec, [3.0, 3.0, -3.0], [0.0, 0.0, 2.0]);
        assert_eq!(u.pose, None);
    }
    assert_eq!(h.count(Pose::Level), before);
    // Back in the same pose: the ring continues.
    h.step(Pose::Level, 30.0);
    h.turn(Pose::Level, 60.0, 30.0);
    assert!(h.count(Pose::Level) > before);
    let continued = h.count(Pose::Level);
    // Another pose is recognised: the incomplete ring of level is cleared; the new one starts.
    h.step(Pose::NoseDown, 0.0);
    assert_eq!(h.count(Pose::Level), 0);
    assert_eq!(h.count(Pose::NoseDown), 1);
    assert!(continued > 1);
}

#[test]
fn a_complete_pose_stays_complete_and_other_incomplete_rings_are_cleared_on_return() {
    let mut h = Hold::new();
    h.step(Pose::Level, 0.0);
    h.turn(Pose::Level, 370.0, 30.0);
    assert!(h.tracker.is_complete(Pose::Level));
    h.step(Pose::NoseDown, 0.0);
    h.turn(Pose::NoseDown, 90.0, 30.0);
    assert!(h.count(Pose::NoseDown) > 1);
    assert!(
        h.tracker.is_complete(Pose::Level),
        "a complete pose stays complete"
    );
    // Back to level (complete): the incomplete nose-down ring is cleared, level stays complete.
    h.step(Pose::Level, 30.0);
    assert!(h.tracker.is_complete(Pose::Level));
    assert_eq!(h.count(Pose::NoseDown), 0);
}

#[test]
fn a_gap_in_the_samples_does_not_advance_the_ring() {
    let mut h = Hold::new();
    h.step(Pose::Level, 0.0);
    h.turn(Pose::Level, 60.0, 30.0);
    let mask = h.tracker.ring(Pose::Level).visited_mask();
    // A 3 s hole, then a sample with a fast gyro: the 3 s are not integrated (MAX_STEP_S is 0.5 s).
    assert_eq!(MAX_STEP_S, 0.5);
    h.t_usec += 3_000_000 - DT_USEC;
    h.step(Pose::Level, 120.0);
    assert_eq!(h.tracker.ring(Pose::Level).visited_mask(), mask);
    // The next step integrates again.
    h.turn(Pose::Level, 90.0, 30.0);
    assert!(h.tracker.ring(Pose::Level).visited_count() > mask.count_ones() as usize);
    // A time that does not move forward is not integrated either.
    let mask = h.tracker.ring(Pose::Level).visited_mask();
    h.t_usec -= 2 * DT_USEC;
    h.step(Pose::Level, 300.0);
    assert_eq!(h.tracker.ring(Pose::Level).visited_mask(), mask);
}
