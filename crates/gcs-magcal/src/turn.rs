//! Turn progress: while a pose is held, the gyro is integrated about the gravity axis and the visited 30-degree
//! sectors of the pose's ring are recorded (decision section 3, "Turn progress and coverage").
//!
//! The ring is for display and for the coverage row only; the fit uses the magnetometer alone. The gyro in
//! HIGHRES_IMU at 25 Hz is a sampled value, not an integral, so the angle is approximate (decision section 13).

use crate::Vec3;
use crate::pose::{Pose, recognise};

/// Sectors per ring. Decision section 3: 30-deg sectors, a PROPOSAL, for display only.
pub const SECTORS: usize = 12;
/// Size of one sector, degrees (360 / [`SECTORS`]).
pub const SECTOR_DEG: f64 = 360.0 / SECTORS as f64;
/// Longest time step that is integrated, seconds. Packet B3b ruling; a step that is not positive or longer than
/// this is a gap (link stall, clock restart) and is not integrated.
pub const MAX_STEP_S: f64 = 0.5;

/// One pose's ring: the sectors visited so far.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ring {
    visited: u16,
    /// Turn angle accumulated since the ring was started, radians, either sign. Only its sector matters.
    angle_rad: f64,
}

impl Ring {
    /// Bit i is set when sector i has been visited.
    pub fn visited_mask(&self) -> u16 {
        self.visited
    }

    pub fn is_visited(&self, sector: usize) -> bool {
        sector < SECTORS && self.visited & (1 << sector) != 0
    }

    pub fn visited_count(&self) -> usize {
        self.visited.count_ones() as usize
    }

    /// Every sector visited.
    pub fn is_complete(&self) -> bool {
        self.visited_count() == SECTORS
    }

    fn sector_of(angle_rad: f64) -> i64 {
        (angle_rad.to_degrees() / SECTOR_DEG).floor() as i64
    }

    /// Marks the sector of the current angle.
    fn mark_here(&mut self) {
        let s = Self::sector_of(self.angle_rad).rem_euclid(SECTORS as i64);
        self.visited |= 1 << s;
    }

    /// Turns by `delta_rad` and marks every sector the turn passed through, in either direction.
    fn advance(&mut self, delta_rad: f64) {
        let from = Self::sector_of(self.angle_rad);
        self.angle_rad += delta_rad;
        let to = Self::sector_of(self.angle_rad);
        for s in from.min(to)..=from.max(to) {
            self.visited |= 1 << s.rem_euclid(SECTORS as i64);
        }
    }
}

/// What one sample did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Update {
    /// The pose this sample is in, if any.
    pub pose: Option<Pose>,
    /// The pose whose ring this sample completed, if any.
    pub completed: Option<Pose>,
}

/// The six rings and the state between samples.
#[derive(Clone, Debug, Default)]
pub struct TurnTracker {
    rings: [Ring; 6],
    /// The pose of the previous sample and its time, when it had one.
    previous: Option<(u64, Pose)>,
    /// The last pose recognised, kept while readings fall to "none".
    last_pose: Option<Pose>,
}

impl TurnTracker {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ring(&self, pose: Pose) -> &Ring {
        &self.rings[pose.index()]
    }

    pub fn is_complete(&self, pose: Pose) -> bool {
        self.ring(pose).is_complete()
    }

    pub fn all_complete(&self) -> bool {
        self.rings.iter().all(Ring::is_complete)
    }

    /// The poses that still need a full turn, in the decision's order.
    pub fn incomplete(&self) -> Vec<Pose> {
        Pose::ALL
            .into_iter()
            .filter(|&p| !self.is_complete(p))
            .collect()
    }

    /// Takes one sample: accelerometer m/s^2 and gyro rad/s in the body frame, `time_usec` from HIGHRES_IMU.
    ///
    /// - A pose is recognised from the accelerometer ([`recognise`]).
    /// - When a pose other than the last recognised one is recognised, the ring of the last one is cleared unless
    ///   it is complete. A reading that falls to "none" clears nothing; a complete ring stays complete.
    /// - The gyro rate about the unit down vector (minus the accelerometer's direction) times the time step since
    ///   the previous sample turns the ring, only when the previous sample was in the same pose. A step that is
    ///   not positive or longer than [`MAX_STEP_S`] is not integrated: no turn is counted across a gap. This is a
    ///   display-only safeguard, since the fit does not use the gyro.
    pub fn update(&mut self, time_usec: u64, accel: Vec3, gyro: Vec3) -> Update {
        let pose = recognise(accel);
        let mut completed = None;
        if let Some(p) = pose {
            if let Some(last) = self.last_pose
                && last != p
                && !self.rings[last.index()].is_complete()
            {
                self.rings[last.index()] = Ring::default();
            }
            self.last_pose = Some(p);
            let was_complete = self.rings[p.index()].is_complete();
            if !was_complete {
                let norm = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
                if let Some((t_prev, p_prev)) = self.previous
                    && p_prev == p
                    && norm > 0.0
                    && time_usec > t_prev
                {
                    let dt = (time_usec - t_prev) as f64 * 1e-6;
                    if dt <= MAX_STEP_S {
                        let rate =
                            -(gyro[0] * accel[0] + gyro[1] * accel[1] + gyro[2] * accel[2]) / norm;
                        if rate.is_finite() {
                            self.rings[p.index()].advance(rate * dt);
                        }
                    }
                }
                self.rings[p.index()].mark_here();
                if self.rings[p.index()].is_complete() {
                    completed = Some(p);
                }
            }
        }
        self.previous = pose.map(|p| (time_usec, p));
        Update { pose, completed }
    }
}
