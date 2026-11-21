//! The World Magnetic Model 2025 (NOAA NCEI, public domain), degree and order 12.
//!
//! The coefficients are NOAA's file `WMM.COF` (epoch 2025.0, released 2024-11-13), embedded unchanged from
//! `data/WMM.COF`; where it was fetched from is in `_plan/ref/wmm2025/SOURCE.txt`. The method is the one of the
//! WMM technical report: geodetic to geocentric spherical coordinates on WGS-84, Schmidt semi-normalised
//! associated Legendre functions, the field as the gradient of the potential, then rotated back to the geodetic
//! north-east-down frame. Guarded by NOAA's test values (`tests/wmm.rs`).

use std::sync::OnceLock;

use crate::Vec3;

const COF: &str = include_str!("../data/WMM.COF");

/// Degree and order of WMM2025.
pub const DEGREE: usize = 12;
/// The model's epoch and the end of its validity (WMM2025 is valid 2025.0 to 2030.0, NOAA).
pub const EPOCH: f64 = 2025.0;
pub const VALID_UNTIL: f64 = 2030.0;

/// WGS-84 semi-major axis in km and flattening; the geomagnetic reference radius in km (WMM technical report).
const WGS84_A_KM: f64 = 6378.137;
const WGS84_F: f64 = 1.0 / 298.257223563;
const REFERENCE_RADIUS_KM: f64 = 6371.2;

/// The model's field at one place and time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Field {
    /// Total intensity F, uT.
    pub intensity_ut: f64,
    /// Inclination (dip), degrees, positive down.
    pub inclination_deg: f64,
    /// Declination, degrees, positive east of true north.
    pub declination_deg: f64,
    /// Horizontal intensity H, uT.
    pub horizontal_ut: f64,
    /// The field vector in the geodetic north-east-down frame (X, Y, Z), uT.
    pub ned_ut: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WmmError {
    /// The date is outside 2025.0 ..= 2030.0, where WMM2025 is defined.
    OutsideValidity,
    /// Latitude outside -90 ..= 90 or a value that is not finite.
    InvalidPosition,
}

impl std::fmt::Display for WmmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WmmError::OutsideValidity => write!(
                f,
                "the date is outside WMM2025's validity ({EPOCH} to {VALID_UNTIL})"
            ),
            WmmError::InvalidPosition => write!(f, "the position is not valid"),
        }
    }
}

impl std::error::Error for WmmError {}

/// Main-field (nT) and secular-variation (nT/year) coefficients, indexed by [n][m].
struct Coefficients {
    g: [[f64; DEGREE + 1]; DEGREE + 1],
    h: [[f64; DEGREE + 1]; DEGREE + 1],
    gdot: [[f64; DEGREE + 1]; DEGREE + 1],
    hdot: [[f64; DEGREE + 1]; DEGREE + 1],
}

fn coefficients() -> &'static Coefficients {
    static PARSED: OnceLock<Coefficients> = OnceLock::new();
    PARSED.get_or_init(|| {
        let z = [[0.0; DEGREE + 1]; DEGREE + 1];
        let mut c = Coefficients {
            g: z,
            h: z,
            gdot: z,
            hdot: z,
        };
        let mut lines = COF.lines();
        let header = lines.next().expect("WMM.COF has a header");
        let epoch: f64 = header
            .split_whitespace()
            .next()
            .and_then(|t| t.parse().ok())
            .expect("WMM.COF epoch");
        assert_eq!(epoch, EPOCH, "WMM.COF epoch");
        let mut count = 0;
        for line in lines {
            if line.starts_with("9999") {
                break;
            }
            let t: Vec<&str> = line.split_whitespace().collect();
            assert_eq!(t.len(), 6, "WMM.COF line: {line}");
            let n: usize = t[0].parse().expect("n");
            let m: usize = t[1].parse().expect("m");
            let v: Vec<f64> = t[2..]
                .iter()
                .map(|s| s.parse().expect("coefficient"))
                .collect();
            c.g[n][m] = v[0];
            c.h[n][m] = v[1];
            c.gdot[n][m] = v[2];
            c.hdot[n][m] = v[3];
            count += 1;
        }
        assert_eq!(
            count,
            DEGREE * (DEGREE + 3) / 2,
            "WMM.COF coefficient count"
        );
        c
    })
}

/// The field at geodetic latitude and longitude (degrees), height above the WGS-84 ellipsoid (metres) and date
/// (decimal year, e.g. 2026.5).
pub fn field(
    lat_deg: f64,
    lon_deg: f64,
    height_m: f64,
    decimal_year: f64,
) -> Result<Field, WmmError> {
    if !(lat_deg.is_finite()
        && lon_deg.is_finite()
        && height_m.is_finite()
        && decimal_year.is_finite())
        || lat_deg.abs() > 90.0
    {
        return Err(WmmError::InvalidPosition);
    }
    if !(EPOCH..=VALID_UNTIL).contains(&decimal_year) {
        return Err(WmmError::OutsideValidity);
    }
    let c = coefficients();
    let dt = decimal_year - EPOCH;

    // Geodetic to geocentric spherical (WGS-84).
    let phi = lat_deg.to_radians();
    let lambda = lon_deg.to_radians();
    let h_km = height_m / 1000.0;
    let e2 = WGS84_F * (2.0 - WGS84_F);
    let (sin_phi, cos_phi) = phi.sin_cos();
    let rc = WGS84_A_KM / (1.0 - e2 * sin_phi * sin_phi).sqrt();
    let p = (rc + h_km) * cos_phi;
    let z = (rc * (1.0 - e2) + h_km) * sin_phi;
    let r = (p * p + z * z).sqrt();
    let phi_c = (z / r).asin();
    // Colatitude theta: cos(theta) = sin(phi_c), sin(theta) = cos(phi_c) >= 0.
    let (ct, st) = (phi_c.sin(), phi_c.cos());

    // Schmidt semi-normalised P_n^m(cos theta), dP/dtheta, and Q = P / sin(theta) for m >= 1 (finite at the
    // poles: Q obeys the same recursion in n as P, seeded with the sectoral term divided by sin(theta)).
    let mut pnm = [[0.0f64; DEGREE + 1]; DEGREE + 1];
    let mut dp = [[0.0f64; DEGREE + 1]; DEGREE + 1];
    let mut q = [[0.0f64; DEGREE + 1]; DEGREE + 1];
    pnm[0][0] = 1.0;
    for m in 0..=DEGREE {
        if m >= 1 {
            // Sectoral: P_m^m = k_m sin(theta) P_{m-1}^{m-1}, k_1 = 1, k_m = sqrt((2m-1)/(2m)).
            let k = if m == 1 {
                1.0
            } else {
                ((2 * m - 1) as f64 / (2 * m) as f64).sqrt()
            };
            pnm[m][m] = k * st * pnm[m - 1][m - 1];
            dp[m][m] = k * (ct * pnm[m - 1][m - 1] + st * dp[m - 1][m - 1]);
            q[m][m] = if m == 1 {
                1.0
            } else {
                k * st * q[m - 1][m - 1]
            };
        }
        for n in m + 1..=DEGREE {
            let a = (2 * n - 1) as f64;
            let b = (((n - 1) * (n - 1) - m * m) as f64).sqrt();
            let d = ((n * n - m * m) as f64).sqrt();
            let (p1, dp1, q1) = (pnm[n - 1][m], dp[n - 1][m], q[n - 1][m]);
            let (p2, dp2, q2) = if n >= m + 2 {
                (pnm[n - 2][m], dp[n - 2][m], q[n - 2][m])
            } else {
                (0.0, 0.0, 0.0)
            };
            pnm[n][m] = (a * ct * p1 - b * p2) / d;
            dp[n][m] = (a * (ct * dp1 - st * p1) - b * dp2) / d;
            q[n][m] = (a * ct * q1 - b * q2) / d;
        }
    }

    // The field in geocentric spherical components: B_r (up), B_theta (south), B_lambda (east).
    let (mut br, mut bt, mut bl) = (0.0f64, 0.0f64, 0.0f64);
    let ratio = REFERENCE_RADIUS_KM / r;
    let mut rn = ratio * ratio; // (a/r)^(n+2), starting at n = 0
    for n in 1..=DEGREE {
        rn *= ratio;
        let (mut sr, mut st_sum, mut sl) = (0.0f64, 0.0f64, 0.0f64);
        for m in 0..=n {
            let g = c.g[n][m] + dt * c.gdot[n][m];
            let h = c.h[n][m] + dt * c.hdot[n][m];
            let (sm, cm) = (m as f64 * lambda).sin_cos();
            let gc = g * cm + h * sm;
            sr += gc * pnm[n][m];
            st_sum += gc * dp[n][m];
            if m >= 1 {
                sl += m as f64 * (g * sm - h * cm) * q[n][m];
            }
        }
        br += (n + 1) as f64 * rn * sr;
        bt -= rn * st_sum;
        bl += rn * sl;
    }
    // Geocentric north, east, down, then rotated by psi = phi_c - phi into the geodetic frame.
    let (xc, yc, zc) = (-bt, bl, -br);
    let psi = phi_c - phi;
    let (sp, cp) = psi.sin_cos();
    let x = xc * cp - zc * sp;
    let zg = xc * sp + zc * cp;
    let y = yc;

    let hz = (x * x + y * y).sqrt();
    let f = (hz * hz + zg * zg).sqrt();
    Ok(Field {
        intensity_ut: f / 1000.0,
        inclination_deg: zg.atan2(hz).to_degrees(),
        declination_deg: y.atan2(x).to_degrees(),
        horizontal_ut: hz / 1000.0,
        ned_ut: [x / 1000.0, y / 1000.0, zg / 1000.0],
    })
}
