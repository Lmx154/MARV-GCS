//! Sample collection: the raw field of each accepted sample, thinned by PX4's spacing rule (decision section 3,
//! "Turn progress and coverage"; section 2, "How the station gets raw samples").
//!
//! The rule is PX4's `reject_sample` (BSD-3-Clause, Copyright (c) PX4 Development Team,
//! src/modules/commander/mag_calibration.cpp:144-166): a sample is rejected when it is closer than
//! `fabsf(5.4f * mag_sphere_radius / sqrtf(max_count)) / 3.0f` to ANY sample already accepted, in the same units as
//! the samples and over all poses together (PX4's arrays hold every side). The radius it uses is
//! `mag_sphere_radius`, the Earth's field strength from the world magnetic model at the GPS position when there is a
//! fix (`get_sphere_radius`, :181-190; 0.4 gauss otherwise, which the station never uses because it needs a fix).
//! `max_count` is `calibration_sides * calibration_points_perside` = 6 * (240 / 6) = 240 (:75, :315, :444).
//! PX4 also stops a side after `calibration_points_perside` = 40 points (:274); the station has no such cap: the
//! spacing alone bounds the number of samples (the decision expects about 30 per ring).

use crate::Vec3;
use crate::undo::{Undo, gauss_to_ut};

/// The 5.4 of `reject_sample`, mag_calibration.cpp:147.
pub const SPACING_FACTOR: f64 = 5.4;
/// `max_count`: 240 points (`calibration_total_points`, mag_calibration.cpp:75; the product at :315).
pub const SPACING_POINTS: f64 = 240.0;
/// The divisor of `reject_sample`, mag_calibration.cpp:147.
pub const SPACING_DIVISOR: f64 = 3.0;

/// The smallest distance between two kept samples, uT, for the model's field strength `radius_ut`.
pub fn min_spacing_ut(radius_ut: f64) -> f64 {
    (SPACING_FACTOR * radius_ut / SPACING_POINTS.sqrt()).abs() / SPACING_DIVISOR
}

/// A kept sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// The raw field in the body frame, uT (the calibration in force undone).
    pub raw: Vec3,
    /// The unit down vector in the body frame (from the accelerometer).
    pub down: Vec3,
}

/// The kept samples of one run.
#[derive(Clone, Debug)]
pub struct Collector {
    undo: Undo,
    min_distance_ut: f64,
    samples: Vec<Sample>,
}

impl Collector {
    /// `undo` of the calibration in force read at the start; `radius_ut` the model's field strength there.
    pub fn new(undo: Undo, radius_ut: f64) -> Self {
        Collector {
            undo,
            min_distance_ut: min_spacing_ut(radius_ut),
            samples: Vec::new(),
        }
    }

    /// Offers one published magnetometer reading (gauss, HIGHRES_IMU) and the unit down vector; true when it is
    /// kept. A reading that is not finite is rejected.
    pub fn offer(&mut self, mag_gauss: [f32; 3], down: Vec3) -> bool {
        let raw = self.undo.raw(gauss_to_ut(mag_gauss));
        if !raw.iter().chain(&down).all(|v| v.is_finite()) {
            return false;
        }
        let too_close = self.samples.iter().any(|s| {
            let d = [raw[0] - s.raw[0], raw[1] - s.raw[1], raw[2] - s.raw[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() < self.min_distance_ut
        });
        if too_close {
            return false;
        }
        self.samples.push(Sample { raw, down });
        true
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }

    /// The raw fields and the down vectors, as `fit::fit` and `accept::Inputs` take them.
    pub fn raw_and_down(&self) -> (Vec<Vec3>, Vec<Vec3>) {
        (
            self.samples.iter().map(|s| s.raw).collect(),
            self.samples.iter().map(|s| s.down).collect(),
        )
    }
}
