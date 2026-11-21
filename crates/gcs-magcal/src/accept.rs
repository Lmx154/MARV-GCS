//! The acceptance checks of the guided calibration (decision section 4 with Amendment A1), in the table's order,
//! each with the table's message; and, on acceptance, the scaling of S to the model's field strength (decision
//! section 1, "Expected field": ArduPilot's `fix_radius` idea).

use crate::Vec3;
use crate::fit::Fit;
use crate::linalg;
use crate::undo::Calibration;
use crate::wmm::Field;

/// Fitness limit, uT RMS. Decision section 4 (row "fitness") and the owner's answer 3; ArduPilot
/// `AP_COMPASS_CALIBRATION_FITNESS_DEFAULT 16.0f` mGauss (ref/ardupilot-compass-20261120/AP_Compass.h:35), checked
/// as `_fitness > sq(_tolerance)` on the mean squared residual (CompassCalibrator.cpp:541).
pub const FITNESS_LIMIT_UT: f64 = 1.6;
/// Fitted radius bounds, uT, exclusive. Decision section 4 (row "radius"); ArduPilot `FIELD_RADIUS_MIN 150`,
/// `FIELD_RADIUS_MAX 950` mGauss (CompassCalibrator.cpp:73-74, checked with `<=`/`>=` at :525).
pub const RADIUS_MIN_UT: f64 = 15.0;
pub const RADIUS_MAX_UT: f64 = 95.0;
/// Diagonal bounds, exclusive, and the off-diagonal magnitude bound, exclusive. Decision section 4 (row
/// "diagonal / off-diagonal"); ArduPilot CompassCalibrator.cpp:533-538.
pub const DIAG_MIN: f64 = 0.2;
pub const DIAG_MAX: f64 = 5.0;
pub const OFFDIAG_MAX: f64 = 1.0;
/// Offset bound, uT, inclusive: the `MAG_OFF*` range. Decision section 4 (row "offsets"); link contract section 16.
pub const OFFSET_LIMIT_UT: f64 = 200.0;
/// Default inclination tolerance, degrees, inclusive. Decision section 4 (row "inclination", `MAG_HLT_INC`),
/// approved by the owner's answer 2 ("Owner's answers": inclination acceptance 10 deg).
pub const INCLINATION_TOLERANCE_DEG: f64 = 10.0;
/// Bounds of the correction model strength / fitted radius, inclusive. Decision Amendment A1; ArduPilot
/// `COMPASS_MAX_SCALE_FACTOR 1.5`, `COMPASS_MIN_SCALE_FACTOR (1.0/COMPASS_MAX_SCALE_FACTOR)`
/// (AP_Compass_config.h:41-42, checked at CompassCalibrator.cpp:1149).
pub const SCALE_MAX: f64 = 1.5;
pub const SCALE_MIN: f64 = 1.0 / SCALE_MAX;

/// Shown on acceptance (decision section 4).
pub const ACCEPTED_MESSAGE: &str = "Compass calibrated (guided): in use now, not saved to flash";

/// What the checks look at.
#[derive(Clone, Copy, Debug)]
pub struct Inputs<'a> {
    /// The poses that still need a full turn, in the order to name them; empty when coverage is complete.
    /// Supplied by the pose and turn detection (next packet); the first one is named in the refusal.
    pub incomplete_poses: &'a [&'a str],
    /// The fit of `samples`.
    pub fit: &'a Fit,
    /// The raw samples that were fitted, body frame, uT.
    pub samples: &'a [Vec3],
    /// For each sample, the down direction in the body frame (any length; normalised here), e.g. from the
    /// accelerometer. Must have the same length as `samples`.
    pub down: &'a [Vec3],
    /// The model's field at the calibration site.
    pub model: &'a Field,
    /// The inclination tolerance, degrees (normally [`INCLINATION_TOLERANCE_DEG`]).
    pub inclination_tolerance_deg: f64,
}

/// An accepted calibration: what is written (link contract section 16, write order step 2) and shown.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accepted {
    /// `MAG_OFF*`, `MAG_DIAG*`, `MAG_ODIAG*`: the fit with S multiplied by `correction`.
    pub calibration: Calibration,
    /// `MAG_FLD_UT`: the model's field strength, uT.
    pub field_ut: f64,
    /// `MAG_FLD_INC`: the model's inclination, degrees.
    pub inclination_deg: f64,
    /// The fit's fitness, uT (before scaling).
    pub fitness_ut: f64,
    /// The measured inclination (median over the samples), degrees.
    pub measured_inclination_deg: f64,
    /// model strength / fitted radius, the factor applied to S.
    pub correction: f64,
}

/// Why a calibration was refused; [`Refusal::message`] is the decision's text with the numbers filled in.
#[derive(Clone, Debug, PartialEq)]
pub enum Refusal {
    Coverage { pose: String },
    FitFailed,
    TooScattered { fitness_ut: f64 },
    ImplausibleStrength { radius_ut: f64 },
    Distortion,
    NotPositiveDefinite,
    OffsetOutOfRange { offsets: Vec3 },
    Disturbed { dip_deg: f64, expected_deg: f64 },
    Scale { radius_ut: f64, expected_ut: f64 },
}

impl Refusal {
    pub fn message(&self) -> String {
        match self {
            Refusal::Coverage { pose } => format!("Not finished: {pose} still needs a full turn"),
            Refusal::FitFailed => "The fit failed: try again away from metal".to_string(),
            Refusal::TooScattered { fitness_ut } => format!(
                "Readings too scattered (fit {fitness_ut:.2} uT, limit {FITNESS_LIMIT_UT}): keep away from metal, turn more slowly, try again"
            ),
            Refusal::ImplausibleStrength { radius_ut } => {
                format!(
                    "Field strength {radius_ut:.1} uT is not plausible: compass faulty or a magnet nearby"
                )
            }
            Refusal::Distortion | Refusal::NotPositiveDefinite => {
                "Compass distortion too large: is something magnetic mounted near it?".to_string()
            }
            Refusal::OffsetOutOfRange { offsets: o } => {
                format!("Offset {:.1} {:.1} {:.1} uT out of range", o[0], o[1], o[2])
            }
            Refusal::Disturbed {
                dip_deg,
                expected_deg,
            } => format!(
                "The field here is disturbed (dip {dip_deg:.1} deg, expected {expected_deg:.1}): move away from metal or the ground spot and try again"
            ),
            Refusal::Scale {
                radius_ut,
                expected_ut,
            } => format!(
                "Field strength {radius_ut:.1} uT against {expected_ut:.1} expected here: compass faulty or a magnet nearby"
            ),
        }
    }
}

impl std::fmt::Display for Refusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message())
    }
}

/// The median over the samples of asin(down_unit . m_unit), m = S (r - o) with the fitted S and o, in degrees;
/// NaN when no sample gives a finite value.
pub fn measured_inclination_deg(fit: &Fit, samples: &[Vec3], down: &[Vec3]) -> f64 {
    assert_eq!(samples.len(), down.len(), "one down vector per sample");
    let mut dips: Vec<f64> = samples
        .iter()
        .zip(down)
        .map(|(&r, &d)| {
            let m = fit.correct(r);
            (linalg::dot(d, m) / (linalg::norm(d) * linalg::norm(m)))
                .clamp(-1.0, 1.0)
                .asin()
                .to_degrees()
        })
        .filter(|v| v.is_finite())
        .collect();
    if dips.is_empty() {
        return f64::NAN;
    }
    dips.sort_by(f64::total_cmp);
    let n = dips.len();
    if n % 2 == 1 {
        dips[n / 2]
    } else {
        0.5 * (dips[n / 2 - 1] + dips[n / 2])
    }
}

/// Runs the checks in the decision's order; the first that fails is returned.
pub fn accept(inputs: &Inputs) -> Result<Accepted, Refusal> {
    let fit = inputs.fit;
    if let Some(pose) = inputs.incomplete_poses.first() {
        return Err(Refusal::Coverage {
            pose: pose.to_string(),
        });
    }
    let finite = fit
        .offsets
        .iter()
        .chain(&fit.diag)
        .chain(&fit.offdiag)
        .all(|v| v.is_finite())
        && fit.radius_ut.is_finite()
        && fit.fitness_ut.is_finite();
    if !fit.converged() || !finite {
        return Err(Refusal::FitFailed);
    }
    if fit.fitness_ut > FITNESS_LIMIT_UT {
        return Err(Refusal::TooScattered {
            fitness_ut: fit.fitness_ut,
        });
    }
    if !(fit.radius_ut > RADIUS_MIN_UT && fit.radius_ut < RADIUS_MAX_UT) {
        return Err(Refusal::ImplausibleStrength {
            radius_ut: fit.radius_ut,
        });
    }
    if !fit.diag.iter().all(|&d| d > DIAG_MIN && d < DIAG_MAX)
        || !fit.offdiag.iter().all(|&o| o.abs() < OFFDIAG_MAX)
    {
        return Err(Refusal::Distortion);
    }
    if !linalg::is_positive_definite(&fit.matrix()) {
        return Err(Refusal::NotPositiveDefinite);
    }
    if !fit.offsets.iter().all(|&o| o.abs() <= OFFSET_LIMIT_UT) {
        return Err(Refusal::OffsetOutOfRange {
            offsets: fit.offsets,
        });
    }
    let dip = measured_inclination_deg(fit, inputs.samples, inputs.down);
    let expected = inputs.model.inclination_deg;
    if !dip.is_finite() || (dip - expected).abs() > inputs.inclination_tolerance_deg {
        return Err(Refusal::Disturbed {
            dip_deg: dip,
            expected_deg: expected,
        });
    }
    let correction = inputs.model.intensity_ut / fit.radius_ut;
    if !(SCALE_MIN..=SCALE_MAX).contains(&correction) {
        return Err(Refusal::Scale {
            radius_ut: fit.radius_ut,
            expected_ut: inputs.model.intensity_ut,
        });
    }
    let scale = |v: Vec3| [v[0] * correction, v[1] * correction, v[2] * correction];
    Ok(Accepted {
        calibration: Calibration {
            offsets: fit.offsets,
            diag: scale(fit.diag),
            offdiag: scale(fit.offdiag),
        },
        field_ut: inputs.model.intensity_ut,
        inclination_deg: expected,
        fitness_ut: fit.fitness_ut,
        measured_inclination_deg: dip,
        correction,
    })
}
