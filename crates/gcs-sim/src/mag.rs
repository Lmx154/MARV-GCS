//! The simulated magnetometer (`docs/link-contract.md` sections 2 and 16; decision
//! `_plan/decision-compass-cal-20261120.md`, sections 1 and 8).
//!
//! # Model
//!
//! With `R` the rotation from the NED frame to the body frame (FRD), `B` the field of the place in NED
//! (uT) and [`MagEnvironment`] holding `S_true`, `o_true`:
//!
//! ```text
//! r = S_true^-1 (R B) + o_true + noise + magnet          raw field of the sensor, body frame, uT
//! m = S (r - o)                                           the calibration in force (MAG_DIAG*, MAG_ODIAG*, MAG_OFF*)
//! xmag, ymag, zmag = m / 100                              gauss, HIGHRES_IMU
//! ```
//!
//! so a vehicle whose parameters are `S = S_true` and `o = o_true` publishes `R B`, the Earth's field in
//! the body frame, plus the noise. `B` is the World Magnetic Model at the simulator's position and
//! [`DECIMAL_YEAR`] (`gcs_magcal::wmm::field`), with the inclination shifted by
//! [`MagEnvironment::inclination_shift_deg`] at an anomaly site (horizontal direction and total strength
//! kept). `S` and `o` are applied in single precision in the contract's order, d = r - o,
//! m_i = (S_i0 d_0 + S_i1 d_1) + S_i2 d_2, without fused multiply-add ([`published_gauss`]).
//! Defaults: `S_true` = identity, `o_true` = 0, no noise, no magnet, no anomaly.

use crate::signals::{ALTITUDE_MM, LATITUDE_E7, LONGITUDE_E7};

/// A 3x3 matrix, row major.
pub type M3 = [[f64; 3]; 3];

/// The simulator's date for the field model (a decimal year, inside the model's 2025.0 to 2030.0). The
/// simulator has no calendar: this is the date of its design.
pub const DECIMAL_YEAR: f64 = 2026.9;

/// What the sensor is in: the vehicle's soft and hard iron and what disturbs the field.
#[derive(Debug, Clone, PartialEq)]
pub struct MagEnvironment {
    /// The sensor's true soft-iron matrix: the calibration that recovers the Earth's field is S = `s_true`.
    pub s_true: M3,
    /// The sensor's true offset, uT (the hard iron): the calibration that recovers it is o = `o_true`.
    pub o_true: [f64; 3],
    /// Standard deviation of the white Gaussian noise added to the raw field, uT, per axis.
    pub noise_ut: f64,
    /// Local inclination minus the model's, degrees (an anomaly site); strength and horizontal direction kept.
    pub inclination_shift_deg: f64,
    /// Amplitude of a magnet moved by hand near the sensor, uT: a body-frame disturbance of three sines of
    /// about 1 Hz that the fit cannot explain as hard or soft iron. 0 is no magnet.
    pub magnet_ut: f64,
}

impl Default for MagEnvironment {
    fn default() -> Self {
        Self {
            s_true: IDENTITY,
            o_true: [0.0; 3],
            noise_ut: 0.0,
            inclination_shift_deg: 0.0,
            magnet_ut: 0.0,
        }
    }
}

pub const IDENTITY: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

pub fn mul_vec(m: &M3, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}

pub fn mul(a: &M3, b: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

pub fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// The inverse of a 3x3 matrix; `None` when it is singular.
pub fn invert(m: &M3) -> Option<M3> {
    let c = |i: usize, j: usize| {
        let (i1, i2, j1, j2) = ((i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3);
        m[i1][j1] * m[i2][j2] - m[i1][j2] * m[i2][j1]
    };
    let det = m[0][0] * c(0, 0) + m[0][1] * c(0, 1) + m[0][2] * c(0, 2);
    if det.abs() < 1e-12 {
        return None;
    }
    Some(std::array::from_fn(|i| {
        std::array::from_fn(|j| c(j, i) / det)
    }))
}

/// The field of the place in NED, uT: the model at the simulator's position, with the anomaly's inclination.
pub fn earth_ned_ut(env: &MagEnvironment) -> [f64; 3] {
    let f = gcs_magcal::wmm::field(
        f64::from(LATITUDE_E7) / 1e7,
        f64::from(LONGITUDE_E7) / 1e7,
        f64::from(ALTITUDE_MM) / 1e3,
        DECIMAL_YEAR,
    )
    .expect("the simulator's position and date are inside the model");
    if env.inclination_shift_deg == 0.0 {
        return f.ned_ut;
    }
    let inc = (f.inclination_deg + env.inclination_shift_deg).to_radians();
    let (h, d) = (f.intensity_ut * inc.cos(), f.intensity_ut * inc.sin());
    let k = h / f.horizontal_ut;
    [f.ned_ut[0] * k, f.ned_ut[1] * k, d]
}

/// The generator of the loss gates (xorshift64), seeded by `seed` and the time `stamp_ms`: the noise of a
/// sample is a pure function of the time, as every sensor value is.
struct Noise(u64);

impl Noise {
    fn new(seed: u64, stamp_ms: u64) -> Self {
        let mut n = Self(seed.wrapping_add(stamp_ms.wrapping_mul(0x9E37_79B9_7F4A_7C15)) | 1);
        for _ in 0..4 {
            n.next();
        }
        n
    }

    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform in (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal (Box-Muller).
    fn gauss(&mut self) -> f64 {
        let (a, b) = (self.uniform(), self.uniform());
        (-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
    }
}

/// The raw field of the sensor, body frame, uT: `S_true^-1 (R B) + o_true + noise + magnet` (module text).
/// `body_from_ned` is `R`. `stamp_ms` is the time (the noise and the magnet are functions of it).
pub fn raw_field_ut(
    env: &MagEnvironment,
    body_from_ned: &M3,
    stamp_ms: u64,
    seed: u64,
) -> [f64; 3] {
    let earth_body = mul_vec(body_from_ned, earth_ned_ut(env));
    let s_inv = invert(&env.s_true).expect("S_true is invertible");
    let mut r = mul_vec(&s_inv, earth_body);
    let mut noise = Noise::new(seed, stamp_ms);
    let t = stamp_ms as f64 / 1000.0;
    for (i, v) in r.iter_mut().enumerate() {
        let magnet = env.magnet_ut
            * (std::f64::consts::TAU * (0.9 + 0.4 * i as f64) * t + 2.1 * i as f64).sin();
        *v += env.o_true[i] + magnet + env.noise_ut * noise.gauss();
    }
    r
}

/// `m = S (r - o)` in single precision in the contract's order (d = r - o; m_i = (S_i0 d_0 + S_i1 d_1) + S_i2
/// d_2), converted to gauss. `offsets` is `MAG_OFFX/Y/Z`, `diag` `MAG_DIAGX/Y/Z`, `offdiag`
/// `MAG_ODIAGXY/XZ/YZ`.
pub fn published_gauss(
    raw_ut: [f64; 3],
    offsets: [f32; 3],
    diag: [f32; 3],
    offdiag: [f32; 3],
) -> [f32; 3] {
    let r = raw_ut.map(|v| v as f32);
    let d = [r[0] - offsets[0], r[1] - offsets[1], r[2] - offsets[2]];
    let s = [
        [diag[0], offdiag[0], offdiag[1]],
        [offdiag[0], diag[1], offdiag[2]],
        [offdiag[1], offdiag[2], diag[2]],
    ];
    [0, 1, 2].map(|i| ((s[i][0] * d[0] + s[i][1] * d[1]) + s[i][2] * d[2]) / 100.0)
}
