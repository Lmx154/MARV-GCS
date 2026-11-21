//! The numerical core of the guided compass calibration (`_plan/decision-compass-cal-20261120.md`, link
//! contract section 16). Pure: no I/O, no clock, no dependency beyond `std`; f64 throughout.
//!
//! - [`wmm`]: the Earth's field from the World Magnetic Model 2025 (NOAA's coefficient file, embedded).
//! - [`undo`]: the calibration in force, m = S (r - o), applied and undone (raw r from the published m).
//! - [`fit`]: PX4's Levenberg-Marquardt sphere fit, then ellipsoid fit, on raw samples in microtesla.
//! - [`accept`]: the acceptance checks of the decision's section 4, in its order and with its messages, and
//!   the scaling of the accepted matrix to the model's field strength.
//!
//! Units: microtesla (uT) for every field, degrees for every angle. HIGHRES_IMU carries gauss: convert with
//! [`undo::gauss_to_ut`] first.

pub mod accept;
pub mod fit;
mod linalg;
pub mod undo;
pub mod wmm;

/// A 3-vector: a field in uT (body frame FRD, or NED for the model) or a direction.
pub type Vec3 = [f64; 3];
