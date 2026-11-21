//! The six poses of the guided calibration and their recognition from one accelerometer sample (decision
//! section 3, "Recognising a pose").
//!
//! The body frame is FRD (x forward, y right, z down) and the accelerometer reads specific force: the reaction to
//! gravity, which points up. So when the vehicle rests with body-frame unit down vector d, it reads -g d:
//!
//! | pose | down in the body | accelerometer reads |
//! |---|---|---|
//! | level | +z | (0, 0, -g) |
//! | upside down | -z | (0, 0, +g) |
//! | nose down | +x (nose to the ground) | (-g, 0, 0) |
//! | nose up | -x | (+g, 0, 0) |
//! | left side down | -y (the left side is -y) | (0, +g, 0) |
//! | right side down | +y | (0, -g, 0) |
//!
//! These are the readings PX4's `detect_orientation` tests (calibration_routines.cpp:171-190: tail down [g, 0, 0]
//! = nose up, nose down [-g, 0, 0], left [0, g, 0], right [0, -g, 0], upside down [0, 0, g], right side up
//! [0, 0, -g] = level).
//!
//! The pose-recognition rule and its constant are PX4's (BSD-3-Clause, Copyright (c) PX4 Development Team):
//! `fabsf(accel[i] - expected[i]) < accel_err_thr` on all three axes (calibration_routines.cpp:74, :171-190).
//! PX4 returns the first pose in its own order whose test passes and averages the accelerometer over time first;
//! the station recognises from one sample (the view needs it at once).

use crate::Vec3;

/// Standard gravity, m/s^2: the value of PX4's `CONSTANTS_ONE_G` (its definition is not under `ref/`).
pub const ONE_G: f64 = 9.80665;
/// Per-axis tolerance of the pose test, m/s^2: `accel_err_thr = 5.0f`, calibration_routines.cpp:74. Along one
/// axis it allows asin(5 / 9.80665) = 30.6 deg of tilt.
pub const POSE_TOLERANCE_MPS2: f64 = 5.0;

/// A pose, in the order the decision lists them (and the order a timeout or the coverage row names the first
/// incomplete one).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Pose {
    Level,
    UpsideDown,
    NoseDown,
    NoseUp,
    LeftSideDown,
    RightSideDown,
}

impl Pose {
    /// The six poses in the decision's order (section 3).
    pub const ALL: [Pose; 6] = [
        Pose::Level,
        Pose::UpsideDown,
        Pose::NoseDown,
        Pose::NoseUp,
        Pose::LeftSideDown,
        Pose::RightSideDown,
    ];

    /// Position in [`Pose::ALL`].
    pub fn index(self) -> usize {
        match self {
            Pose::Level => 0,
            Pose::UpsideDown => 1,
            Pose::NoseDown => 2,
            Pose::NoseUp => 3,
            Pose::LeftSideDown => 4,
            Pose::RightSideDown => 5,
        }
    }

    /// The name shown to the user and used in the messages.
    pub fn name(self) -> &'static str {
        match self {
            Pose::Level => "level",
            Pose::UpsideDown => "upside down",
            Pose::NoseDown => "nose down",
            Pose::NoseUp => "nose up",
            Pose::LeftSideDown => "left side down",
            Pose::RightSideDown => "right side down",
        }
    }

    /// What the accelerometer reads at rest in this pose, m/s^2 (the table in the module text).
    pub fn expected_accel(self) -> Vec3 {
        match self {
            Pose::Level => [0.0, 0.0, -ONE_G],
            Pose::UpsideDown => [0.0, 0.0, ONE_G],
            Pose::NoseDown => [-ONE_G, 0.0, 0.0],
            Pose::NoseUp => [ONE_G, 0.0, 0.0],
            Pose::LeftSideDown => [0.0, ONE_G, 0.0],
            Pose::RightSideDown => [0.0, -ONE_G, 0.0],
        }
    }

    /// PX4's test: every axis strictly within [`POSE_TOLERANCE_MPS2`] of the pose's expected reading. A reading
    /// with a NaN never matches.
    pub fn matches(self, accel: Vec3) -> bool {
        let e = self.expected_accel();
        (0..3).all(|i| (accel[i] - e[i]).abs() < POSE_TOLERANCE_MPS2)
    }
}

impl std::fmt::Display for Pose {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// The pose a single accelerometer sample (m/s^2, body frame) is in, or `None`.
///
/// PX4's per-axis boxes overlap in a thin wedge on a 45 deg tilt between two poses, e.g. (4.9, 0, -4.9) passes the
/// test of both level and nose up, at 0.7 g, far from any held pose. PX4 takes the first pose in its list; here
/// the pose whose expected reading is nearest wins (ties: the earlier in [`Pose::ALL`]), so one sample is one
/// pose. For a reading within 10 % of 1 g the boxes never overlap (tests/pose.rs).
pub fn recognise(accel: Vec3) -> Option<Pose> {
    let mut best: Option<(Pose, f64)> = None;
    for pose in Pose::ALL {
        if !pose.matches(accel) {
            continue;
        }
        let e = pose.expected_accel();
        let d2: f64 = (0..3).map(|i| (accel[i] - e[i]).powi(2)).sum();
        if best.is_none_or(|(_, b)| d2 < b) {
            best = Some((pose, d2));
        }
    }
    best.map(|(p, _)| p)
}
