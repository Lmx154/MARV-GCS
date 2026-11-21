//! A scripted user who performs the guided compass calibration (decision section 3): it holds each of the six
//! poses of `gcs_magcal::pose::Pose` in order and, in each, makes one full turn about the vertical, then
//! stays where it ended. The attitude is a pure function of the time; the accelerometer (specific force) and
//! the gyro (body rate) of HIGHRES_IMU follow from it.
//!
//! Per pose: a transition of [`TRANSITION_S`] from the previous pose (none before the first), a hold of
//! [`HOLD_S`], one turn of [`CalibrationUser::turn_s`] at a constant rate, a hold of [`HOLD_S`]. The numbers
//! are the scenario's, not the vehicle's: the decision asks for about 12 s per turn and short transitions.
//! Variants: a user who skips a pose ([`CalibrationUser::skipping`]); the magnet and the anomaly site are
//! properties of the place ([`crate::mag::MagEnvironment`]); link loss is [`SimVehicle::set_silent`] and
//! arming is [`SimVehicle::set_armed`] at the chosen time.
//!
//! [`SimVehicle::set_silent`]: crate::SimVehicle::set_silent
//! [`SimVehicle::set_armed`]: crate::SimVehicle::set_armed

use gcs_magcal::pose::{ONE_G, Pose};

use crate::mag::{IDENTITY, M3, mul, mul_vec, transpose};
use crate::signals::GRAVITY_MS2;

/// Seconds to carry the vehicle from one pose to the next (a scenario choice).
pub const TRANSITION_S: f64 = 1.5;
/// Seconds held still before and after each turn (a scenario choice).
pub const HOLD_S: f64 = 0.5;
/// Seconds of one turn: the decision's "about 12 s".
pub const TURN_S: f64 = 12.0;

/// A scripted user. The script starts at `start_ms` of the simulator's time; before it the vehicle lies in the
/// first pose, after it in the last.
#[derive(Debug, Clone, PartialEq)]
pub struct CalibrationUser {
    pub start_ms: u64,
    /// Seconds of one turn.
    pub turn_s: f64,
    /// The index (in [`Pose::ALL`]) of a pose the user never visits.
    pub skip_pose: Option<usize>,
}

/// What the sensors of the vehicle feel at one time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Motion {
    /// `R`: from NED to the body frame (FRD).
    pub body_from_ned: M3,
    /// Specific force, m/s^2, body frame (level and still: (0, 0, -g)).
    pub accel: [f64; 3],
    /// Body rate, rad/s.
    pub gyro: [f64; 3],
}

fn rodrigues(axis: [f64; 3], angle: f64) -> M3 {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let t = 1.0 - c;
    [
        [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
        [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
        [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
    ]
}

/// Axis and angle (0 to pi) of a rotation matrix.
fn axis_angle(r: &M3) -> ([f64; 3], f64) {
    let angle = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0)
        .clamp(-1.0, 1.0)
        .acos();
    if angle < 1e-9 {
        return ([0.0, 0.0, 1.0], 0.0);
    }
    if angle.sin() > 1e-6 {
        let k = 2.0 * angle.sin();
        return (
            [
                (r[2][1] - r[1][2]) / k,
                (r[0][2] - r[2][0]) / k,
                (r[1][0] - r[0][1]) / k,
            ],
            angle,
        );
    }
    // Half a turn: the axis is the largest column of (R + I) / 2.
    let i = (0..3)
        .max_by(|&a, &b| r[a][a].total_cmp(&r[b][b]))
        .unwrap_or(0);
    let ki = ((r[i][i] + 1.0) / 2.0).sqrt();
    let mut k = [0.0; 3];
    for (j, kj) in k.iter_mut().enumerate() {
        *kj = if j == i {
            ki
        } else {
            (r[i][j] + r[j][i]) / (4.0 * ki)
        };
    }
    (k, angle)
}

/// `R` of the vehicle level with heading `psi` (rad): `Rz(psi)^T`.
fn level(psi: f64) -> M3 {
    transpose(&rodrigues([0.0, 0.0, 1.0], psi))
}

/// `R` of pose `index` of [`Pose::ALL`] at heading `psi`: the pose's body-frame down vector from its
/// accelerometer reading, then a turn of `psi` about the vertical.
fn pose_attitude(index: usize, psi: f64) -> M3 {
    let a = Pose::ALL[index].expected_accel();
    let down = [-a[0] / ONE_G, -a[1] / ONE_G, -a[2] / ONE_G];
    let q = if down[2] < -0.999_999 {
        rodrigues([1.0, 0.0, 0.0], std::f64::consts::PI)
    } else {
        // z x down, and the angle between z and down.
        let axis = [-down[1], down[0], 0.0];
        let n = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if n < 1e-12 {
            IDENTITY
        } else {
            rodrigues([axis[0] / n, axis[1] / n, 0.0], n.atan2(down[2]))
        }
    };
    mul(&q, &level(psi))
}

impl CalibrationUser {
    /// A user who visits all six poses, starting at `start_ms`, with turns of [`TURN_S`].
    pub fn new(start_ms: u64) -> Self {
        Self {
            start_ms,
            turn_s: TURN_S,
            skip_pose: None,
        }
    }

    /// The same user, never visiting the pose `index` of [`Pose::ALL`].
    #[must_use]
    pub fn skipping(mut self, index: usize) -> Self {
        self.skip_pose = Some(index);
        self
    }

    /// Seconds from the start to the end of the last turn's closing hold.
    pub fn duration_s(&self) -> f64 {
        let visited = Pose::ALL.len() - usize::from(self.skip_pose.is_some());
        visited as f64 * self.leg_s() - TRANSITION_S
    }

    fn leg_s(&self) -> f64 {
        TRANSITION_S + HOLD_S + self.turn_s + HOLD_S
    }

    fn attitude(&self, t_s: f64) -> M3 {
        let poses: Vec<usize> = (0..Pose::ALL.len())
            .filter(|p| Some(*p) != self.skip_pose)
            .collect();
        // The first pose has a transition too, from itself: a hold.
        let t = t_s.max(0.0) + TRANSITION_S;
        let leg = (t / self.leg_s()).floor() as usize;
        let Some(&pose) = poses.get(leg) else {
            return pose_attitude(*poses.last().unwrap_or(&0), 0.0);
        };
        let u = t - leg as f64 * self.leg_s();
        if u < TRANSITION_S {
            let previous = if leg == 0 { pose } else { poses[leg - 1] };
            let (a, b) = (pose_attitude(previous, 0.0), pose_attitude(pose, 0.0));
            let (axis, angle) = axis_angle(&mul(&b, &transpose(&a)));
            let s = u / TRANSITION_S;
            let s = s * s * (3.0 - 2.0 * s);
            return mul(&rodrigues(axis, angle * s), &a);
        }
        let turned = u - TRANSITION_S - HOLD_S;
        if (0.0..self.turn_s).contains(&turned) {
            return pose_attitude(pose, std::f64::consts::TAU * turned / self.turn_s);
        }
        pose_attitude(pose, 0.0)
    }

    /// The attitude, specific force and body rate at the simulator's time `now_ms`.
    pub fn motion(&self, now_ms: u64) -> Motion {
        let t = (now_ms as f64 - self.start_ms as f64) / 1000.0;
        let r = self.attitude(t);
        let h = 0.005;
        let (p, m) = (self.attitude(t + h), self.attitude(t - h));
        // For a fixed NED vector v, d(R v)/dt = -w x (R v): [w]x = -Rdot R^T.
        let rdot: M3 =
            std::array::from_fn(|i| std::array::from_fn(|j| (p[i][j] - m[i][j]) / (2.0 * h)));
        let w = mul(&rdot, &transpose(&r));
        let gyro = [
            (w[1][2] - w[2][1]) / 2.0,
            (w[2][0] - w[0][2]) / 2.0,
            (w[0][1] - w[1][0]) / 2.0,
        ];
        let down = mul_vec(&r, [0.0, 0.0, 1.0]);
        Motion {
            body_from_ned: r,
            accel: down.map(|d| -GRAVITY_MS2 * d),
            gyro,
        }
    }
}
