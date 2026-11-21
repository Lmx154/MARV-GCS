//! The calibration in force and its undo (link contract section 16; decision section 2, "How the station gets raw
//! samples").
//!
//! The vehicle publishes m = S (r - o): r the body-frame raw field, o the offsets `MAG_OFFX/Y/Z`, S the symmetric
//! matrix of `MAG_DIAGX/Y/Z` and `MAG_ODIAGXY/XZ/YZ`. The station gets raw samples back with r = S^-1 m + o,
//! with the S and o it read at the start of the run.

use crate::Vec3;
use crate::linalg::{self, Mat3};

/// 1 gauss = 100 uT. HIGHRES_IMU carries the field in gauss (decision Amendment A5; contract section 16).
pub const UT_PER_GAUSS: f64 = 100.0;

/// The refusal when the stored matrix cannot be undone (decision section 2, verbatim).
pub const INVALID_MATRIX_MESSAGE: &str =
    "The stored compass matrix is not valid. Reset the compass calibration first.";

/// HIGHRES_IMU `xmag`, `ymag`, `zmag` (gauss) to microtesla.
pub fn gauss_to_ut(gauss: [f32; 3]) -> Vec3 {
    [
        f64::from(gauss[0]) * UT_PER_GAUSS,
        f64::from(gauss[1]) * UT_PER_GAUSS,
        f64::from(gauss[2]) * UT_PER_GAUSS,
    ]
}

/// A compass calibration: the eleven calibration parameters that shape the published field.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// `MAG_OFFX/Y/Z`, uT.
    pub offsets: Vec3,
    /// `MAG_DIAGX/Y/Z`.
    pub diag: Vec3,
    /// `MAG_ODIAGXY`, `MAG_ODIAGXZ`, `MAG_ODIAGYZ`.
    pub offdiag: Vec3,
}

impl Default for Calibration {
    /// The parameters' defaults: offsets 0, S = identity.
    fn default() -> Self {
        Calibration {
            offsets: [0.0; 3],
            diag: [1.0; 3],
            offdiag: [0.0; 3],
        }
    }
}

impl Calibration {
    /// S in the contract's layout: [[DIAGX, ODIAGXY, ODIAGXZ], [ODIAGXY, DIAGY, ODIAGYZ], [ODIAGXZ, ODIAGYZ, DIAGZ]].
    pub fn matrix(&self) -> [[f64; 3]; 3] {
        let (d, o) = (self.diag, self.offdiag);
        [[d[0], o[0], o[1]], [o[0], d[1], o[2]], [o[1], o[2], d[2]]]
    }

    /// m = S (r - o) in f64, in the contract's order (d = r - o; m_i = (S_i0 d_0 + S_i1 d_1) + S_i2 d_2).
    pub fn apply(&self, raw_ut: Vec3) -> Vec3 {
        linalg::mul_vec(&self.matrix(), linalg::sub(raw_ut, self.offsets))
    }

    /// Prepares the undo; `Err` when S is not positive definite (the station then refuses to start).
    pub fn undo(&self) -> Result<Undo, InvalidMatrix> {
        let s = self.matrix();
        if !linalg::is_positive_definite(&s) {
            return Err(InvalidMatrix);
        }
        let s_inv = linalg::invert(&s).ok_or(InvalidMatrix)?;
        Ok(Undo {
            s_inv,
            offsets: self.offsets,
        })
    }
}

/// The stored matrix is not positive definite; [`InvalidMatrix::message`] is the decision's text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidMatrix;

impl InvalidMatrix {
    pub fn message(&self) -> &'static str {
        INVALID_MATRIX_MESSAGE
    }
}

impl std::fmt::Display for InvalidMatrix {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(INVALID_MATRIX_MESSAGE)
    }
}

impl std::error::Error for InvalidMatrix {}

/// The undo of one calibration, S^-1 computed once.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Undo {
    s_inv: Mat3,
    offsets: Vec3,
}

impl Undo {
    /// r = S^-1 m + o, uT in and out.
    pub fn raw(&self, published_ut: Vec3) -> Vec3 {
        linalg::add(linalg::mul_vec(&self.s_inv, published_ut), self.offsets)
    }
}
