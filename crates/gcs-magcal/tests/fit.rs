//! I8: the fit recovers a known S and o from six synthetic rings and bad coverage does not give an accepted wild
//! result (decision section 9); and every acceptance row fires with its message (decision section 4).

use gcs_magcal::Vec3;
use gcs_magcal::accept::{self, Inputs, Refusal};
use gcs_magcal::fit::{Fit, fit};
use gcs_magcal::wmm::Field;

type M3 = [[f64; 3]; 3];

const FIELD_UT: f64 = 43.7;
const INCLINATION_DEG: f64 = 54.6;
const NOISE_UT: f64 = 0.3;
const S_TRUE: M3 = [[1.06, 0.04, -0.05], [0.04, 0.95, 0.06], [-0.05, 0.06, 0.78]];
const O_TRUE: Vec3 = [32.0, -18.5, 47.0];

fn mv(m: &M3, v: Vec3) -> Vec3 {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}
fn mtv(m: &M3, v: Vec3) -> Vec3 {
    [0, 1, 2].map(|i| m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2])
}
fn mm(a: &M3, b: &M3) -> M3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}
fn norm(v: Vec3) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
fn inv3(m: &M3) -> M3 {
    let c = |r: usize, s: usize| {
        let (r1, r2, s1, s2) = ((r + 1) % 3, (r + 2) % 3, (s + 1) % 3, (s + 2) % 3);
        m[r1][s1] * m[r2][s2] - m[r1][s2] * m[r2][s1]
    };
    let det = m[0][0] * c(0, 0) + m[0][1] * c(0, 1) + m[0][2] * c(0, 2);
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| c(j, i) / det))
}

/// SplitMix64 with Box-Muller: the fixed-seed noise generator.
struct Rng(u64);
impl Rng {
    fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

fn field_ned() -> Vec3 {
    field_ned_of(FIELD_UT)
}

fn field_ned_of(field_ut: f64) -> Vec3 {
    let i = INCLINATION_DEG.to_radians();
    [field_ut * i.cos(), 0.0, field_ut * i.sin()]
}

fn model() -> Field {
    let n = field_ned();
    Field {
        intensity_ut: FIELD_UT,
        inclination_deg: INCLINATION_DEG,
        declination_deg: 0.0,
        horizontal_ut: n[0],
        ned_ut: n,
    }
}

/// The six poses as body-from-level rotations: level, upside down, nose down, nose up, left side down, right side down.
const POSES: [(&str, M3); 6] = [
    ("level", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
    (
        "upside down",
        [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
    ),
    (
        "nose down",
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
    ),
    (
        "nose up",
        [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    ),
    (
        "left side down",
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
    ),
    (
        "right side down",
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
    ),
];

struct Data {
    raw: Vec<Vec3>,
    /// The same raw samples without noise.
    clean: Vec<Vec3>,
    down: Vec<Vec3>,
    /// Body-from-NED attitude of each sample.
    attitude: Vec<M3>,
}

/// One full turn about the vertical in each listed pose, 30 points per ring, noise on the raw field.
fn rings(poses: &[usize], seed: u64) -> Data {
    rings_of(poses, seed, FIELD_UT)
}

/// As [`rings`] with another true field strength.
fn rings_of(poses: &[usize], seed: u64, field_ut: f64) -> Data {
    let mut rng = Rng(seed);
    let s_inv = inv3(&S_TRUE);
    let mut d = Data {
        raw: vec![],
        clean: vec![],
        down: vec![],
        attitude: vec![],
    };
    for &p in poses {
        for k in 0..30 {
            let psi = (k as f64 * 12.0 + 3.0 * p as f64).to_radians();
            let (s, c) = psi.sin_cos();
            let yaw: M3 = [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]];
            let att = mm(&POSES[p].1, &yaw);
            let b = mv(&att, field_ned_of(field_ut));
            let r = mv(&s_inv, b);
            d.raw
                .push([0, 1, 2].map(|i| r[i] + O_TRUE[i] + NOISE_UT * rng.gauss()));
            d.clean.push([0, 1, 2].map(|i| r[i] + O_TRUE[i]));
            d.down.push(mv(&att, [0.0, 0.0, 1.0]));
            d.attitude.push(att);
        }
    }
    d
}

fn inputs<'a>(f: &'a Fit, d: &'a Data, m: &'a Field) -> Inputs<'a> {
    Inputs {
        incomplete_poses: &[],
        fit: f,
        samples: &d.raw,
        down: &d.down,
        model: m,
        inclination_tolerance_deg: accept::INCLINATION_TOLERANCE_DEG,
    }
}

fn six() -> (Data, Fit) {
    let d = rings(&[0, 1, 2, 3, 4, 5], 0x00C0_FFEE);
    let f = fit(&d.raw, FIELD_UT);
    (d, f)
}

#[test]
fn six_rings_recover_s_and_o() {
    let (d, f) = six();
    let m = model();
    let a = accept::accept(&inputs(&f, &d, &m)).expect("six rings are accepted");
    let s = a.calibration.matrix();
    let mut worst_o = 0.0f64;
    let (mut worst_diag_rel, mut worst_off) = (0.0f64, 0.0f64);
    for i in 0..3 {
        worst_o = worst_o.max((a.calibration.offsets[i] - O_TRUE[i]).abs());
        for j in 0..3 {
            let e = s[i][j] - S_TRUE[i][j];
            if i == j {
                worst_diag_rel = worst_diag_rel.max(e.abs() / S_TRUE[i][i]);
            } else {
                worst_off = worst_off.max(e.abs());
            }
        }
    }
    let mut mean_strength = 0.0;
    for r in &d.raw {
        let o = a.calibration.offsets;
        mean_strength += norm(mv(&s, [r[0] - o[0], r[1] - o[1], r[2] - o[2]])) / d.raw.len() as f64;
    }
    let (worst_heading, rms_noisy) =
        heading_errors(&a.calibration.matrix(), a.calibration.offsets, &d);
    println!(
        "six rings: {} samples, fitness {:.3} uT, radius {:.3} uT, correction {:.4}; worst offset error {worst_o:.3} uT, \
         worst diagonal error {:.2} %, worst off-diagonal error {worst_off:.4}; worst heading error {worst_heading:.3} deg (noise-free samples; \
         RMS {rms_noisy:.3} deg on the noisy ones); \
         mean strength {mean_strength:.3} uT; dip {:.2} deg",
        d.raw.len(),
        f.fitness_ut,
        f.radius_ut,
        a.correction,
        worst_diag_rel * 100.0,
        a.measured_inclination_deg
    );
    assert!(worst_o < 0.5);
    assert!(worst_diag_rel < 0.02);
    assert!(
        worst_off < 0.02,
        "off-diagonal error against 2 % of the unit diagonal"
    );
    assert!(worst_heading < 2.0);
    assert!((mean_strength - FIELD_UT).abs() < 0.05 * FIELD_UT);
    assert_eq!(a.field_ut, FIELD_UT);
    assert_eq!(a.inclination_deg, INCLINATION_DEG);
    assert!((a.measured_inclination_deg - INCLINATION_DEG).abs() < 1.0);
    assert!(f.fitness_ut < 0.6);
}

#[test]
fn five_rings() {
    let m = model();
    for (skip, (skipped, _)) in POSES.iter().enumerate() {
        let poses: Vec<usize> = (0..6).filter(|&p| p != skip).collect();
        let d = rings(&poses, 0x00C0_FFEE);
        let f = fit(&d.raw, FIELD_UT);
        let result = accept::accept(&inputs(&f, &d, &m));
        let worst_o = (0..3)
            .map(|i| (f.offsets[i] - O_TRUE[i]).abs())
            .fold(0.0, f64::max);
        println!(
            "five rings without {}: converged {}, fitness {:.3}, worst offset error {worst_o:.3} uT, {}",
            skipped,
            f.converged(),
            f.fitness_ut,
            match &result {
                Ok(a) => format!("accepted (diag {:.4?})", a.calibration.diag),
                Err(e) => format!("refused: {e}"),
            }
        );
        // The numerics alone accept five rings (each ring still spans the sphere well); the refusal is the coverage
        // row, fed by the pose detection.
        assert!(result.is_ok());
        assert!(worst_o < 0.5);
        let incomplete = [*skipped];
        let refused = accept::accept(&Inputs {
            incomplete_poses: &incomplete,
            ..inputs(&f, &d, &m)
        });
        assert_eq!(
            refused.unwrap_err().message(),
            format!("Not finished: {skipped} still needs a full turn")
        );
    }
}

/// Heading error of a calibration over the attitudes of `d` (the true field has declination 0): the worst over
/// the noise-free samples (the calibration's own error) and the RMS over the noisy ones.
fn heading_errors(s: &M3, o: Vec3, d: &Data) -> (f64, f64) {
    let heading = |r: &Vec3, att: &M3| {
        let ned = mtv(att, mv(s, [r[0] - o[0], r[1] - o[1], r[2] - o[2]]));
        ned[1].atan2(ned[0]).to_degrees()
    };
    let worst = d
        .clean
        .iter()
        .zip(&d.attitude)
        .map(|(r, a)| heading(r, a).abs())
        .fold(0.0, f64::max);
    let ms = d
        .raw
        .iter()
        .zip(&d.attitude)
        .map(|(r, a)| heading(r, a).powi(2))
        .sum::<f64>()
        / d.raw.len() as f64;
    (worst, ms.sqrt())
}

/// Smallest eigenvalue of a symmetric 3x3 matrix (trigonometric method).
fn smallest_eigenvalue(a: &M3) -> f64 {
    let p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    let q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
    let p2 = (a[0][0] - q).powi(2) + (a[1][1] - q).powi(2) + (a[2][2] - q).powi(2) + 2.0 * p1;
    let p = (p2 / 6.0).sqrt();
    let b: M3 = [0, 1, 2].map(|i| [0, 1, 2].map(|j| (a[i][j] - if i == j { q } else { 0.0 }) / p));
    let det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1])
        - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0])
        + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
    let phi = (det / 2.0).clamp(-1.0, 1.0).acos() / 3.0;
    q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos()
}

#[test]
/// Window-B-like data (decision section 1: one direction missing, smallest direction-spread eigenvalue 0.009).
/// What the numerics do with an exact ellipsoid plus 0.3 uT noise: both fits converge and every numeric row passes,
/// with a result close to the truth (not wild); only the coverage row, fed by the pose detection, refuses it.
fn thin_band_numerics_give_a_close_result_and_coverage_refuses_it() {
    let mut rng = Rng(0xBAD_BA4D);
    let s_inv = inv3(&S_TRUE);
    let sin_i = INCLINATION_DEG.to_radians().sin();
    let cos_i = INCLINATION_DEG.to_radians().cos();
    let (mut raw, mut down, mut dirs) = (vec![], vec![], vec![]);
    let half_width = 0.164; // uniform in +-a: variance a^2/3 = 0.009
    for k in 0..180 {
        let z = half_width * (2.0 * rng.uniform() - 1.0);
        let az = (k as f64 * 2.0).to_radians();
        let u = [
            (1.0 - z * z).sqrt() * az.cos(),
            (1.0 - z * z).sqrt() * az.sin(),
            z,
        ];
        let r = mv(&s_inv, u.map(|c| c * FIELD_UT));
        raw.push([0, 1, 2].map(|i| r[i] + O_TRUE[i] + NOISE_UT * rng.gauss()));
        // A down direction at the model's dip from the field direction.
        let w = [-u[2] * u[0], -u[2] * u[1], 1.0 - u[2] * u[2]];
        let wn = norm(w);
        down.push([0, 1, 2].map(|i| sin_i * u[i] + cos_i * w[i] / wn));
        dirs.push(u);
    }
    let n = dirs.len() as f64;
    let mean = [0, 1, 2].map(|i| dirs.iter().map(|u| u[i]).sum::<f64>() / n);
    let cov: M3 = [0, 1, 2].map(|i| {
        [0, 1, 2].map(|j| {
            dirs.iter()
                .map(|u| (u[i] - mean[i]) * (u[j] - mean[j]))
                .sum::<f64>()
                / n
        })
    });
    let spread = smallest_eigenvalue(&cov);
    let f = fit(&raw, FIELD_UT);
    let m = model();
    let d = Data {
        raw,
        clean: vec![],
        down,
        attitude: vec![],
    };
    let result = accept::accept(&inputs(&f, &d, &m));
    println!(
        "thin band: smallest spread eigenvalue {spread:.4}; sphere {} ellipsoid {}, offsets {:.2?}, diag {:.3?}, offdiag {:.3?}, \
         radius {:.2}, fitness {:.3}; {}",
        f.sphere_converged,
        f.ellipsoid_converged,
        f.offsets,
        f.diag,
        f.offdiag,
        f.radius_ut,
        f.fitness_ut,
        match &result {
            Ok(a) => format!(
                "ACCEPTED, offsets {:.2?} diag {:.3?}",
                a.calibration.offsets, a.calibration.diag
            ),
            Err(e) => format!("refused: {e}"),
        }
    );
    assert!((spread - 0.009).abs() < 0.003);
    let a = result.expect("the numeric rows pass");
    let worst_o = (0..3)
        .map(|i| (a.calibration.offsets[i] - O_TRUE[i]).abs())
        .fold(0.0, f64::max);
    let worst_diag = (0..3)
        .map(|i| (a.calibration.diag[i] / S_TRUE[i][i] - 1.0).abs())
        .fold(0.0, f64::max);
    let six = rings(&[0, 1, 2, 3, 4, 5], 0x00C0_FFEE);
    let (heading, _) = heading_errors(&a.calibration.matrix(), a.calibration.offsets, &six);
    println!(
        "thin band accepted by the numeric rows: worst offset error {worst_o:.3} uT, worst diagonal error {:.2} %, \
         worst heading error over the six-ring attitudes {heading:.2} deg",
        worst_diag * 100.0
    );
    assert!(worst_o < 0.5 && worst_diag < 0.05 && heading < 5.0);
    let incomplete = ["upside down", "nose down"];
    let refused = accept::accept(&Inputs {
        incomplete_poses: &incomplete,
        ..inputs(&f, &d, &m)
    });
    assert_eq!(
        refused.unwrap_err().message(),
        "Not finished: upside down still needs a full turn"
    );
}

/// The six-ring fit with one field changed, for the rows that look only at the fit.
fn refusal(change: impl Fn(&mut Fit), model_change: impl Fn(&mut Field)) -> Refusal {
    let (d, mut f) = six();
    change(&mut f);
    let mut m = model();
    model_change(&mut m);
    accept::accept(&inputs(&f, &d, &m)).unwrap_err()
}

#[test]
fn row_coverage() {
    let (d, f) = six();
    let m = model();
    let poses = ["nose up", "level"];
    let e = accept::accept(&Inputs {
        incomplete_poses: &poses,
        ..inputs(&f, &d, &m)
    })
    .unwrap_err();
    assert_eq!(e.message(), "Not finished: nose up still needs a full turn");
}

#[test]
fn row_fit_failed() {
    for change in [
        (|f: &mut Fit| f.ellipsoid_converged = false) as fn(&mut Fit),
        |f| f.sphere_converged = false,
        |f| f.offsets[1] = f64::NAN,
        |f| f.diag[2] = f64::INFINITY,
    ] {
        let e = refusal(change, |_| {});
        assert_eq!(e, Refusal::FitFailed);
        assert_eq!(e.message(), "The fit failed: try again away from metal");
    }
    let empty = fit(&[], FIELD_UT);
    assert!(!empty.converged());
}

#[test]
fn row_fitness() {
    let e = refusal(|f| f.fitness_ut = 1.75, |_| {});
    assert_eq!(
        e.message(),
        "Readings too scattered (fit 1.75 uT, limit 1.6): keep away from metal, turn more slowly, try again"
    );
    // The limit itself passes.
    let (d, mut f) = six();
    f.fitness_ut = accept::FITNESS_LIMIT_UT;
    assert!(accept::accept(&inputs(&f, &d, &model())).is_ok());
}

#[test]
fn row_radius() {
    for radius in [15.0, 95.0, 120.0, 9.5] {
        let e = refusal(|f| f.radius_ut = radius, |_| {});
        assert_eq!(
            e.message(),
            format!(
                "Field strength {radius:.1} uT is not plausible: compass faulty or a magnet nearby"
            )
        );
    }
}

#[test]
fn row_diagonal_and_off_diagonal() {
    let text = "Compass distortion too large: is something magnetic mounted near it?";
    for change in [
        (|f: &mut Fit| f.diag[0] = 5.0) as fn(&mut Fit),
        |f| f.diag[1] = 0.2,
        |f| f.offdiag[2] = -1.0,
        |f| f.offdiag[0] = 1.2,
    ] {
        let e = refusal(change, |_| {});
        assert_eq!(e, Refusal::Distortion);
        assert_eq!(e.message(), text);
    }
}

#[test]
fn row_positive_definite() {
    let e = refusal(
        |f| {
            f.diag = [1.0, 1.0, 1.0];
            f.offdiag = [0.9, 0.9, -0.9];
        },
        |_| {},
    );
    assert_eq!(e, Refusal::NotPositiveDefinite);
    assert_eq!(
        e.message(),
        "Compass distortion too large: is something magnetic mounted near it?"
    );
}

#[test]
fn row_offsets() {
    let e = refusal(|f| f.offsets = [250.0, 0.0, -3.04], |_| {});
    assert_eq!(e.message(), "Offset 250.0 0.0 -3.0 uT out of range");
    let e = refusal(|f| f.offsets[2] = -200.5, |_| {});
    assert!(matches!(e, Refusal::OffsetOutOfRange { .. }));
}

#[test]
fn row_inclination() {
    let e = refusal(|_| {}, |m| m.inclination_deg = INCLINATION_DEG + 10.5);
    let Refusal::Disturbed {
        dip_deg,
        expected_deg,
    } = e
    else {
        panic!("{e:?}")
    };
    assert!((dip_deg - INCLINATION_DEG).abs() < 1.0);
    assert_eq!(expected_deg, 65.1);
    assert_eq!(
        e.message(),
        format!(
            "The field here is disturbed (dip {dip_deg:.1} deg, expected 65.1): move away from metal or the ground spot and try again"
        )
    );
}

#[test]
fn row_scale_to_the_model() {
    let (_, f) = six();
    let expected = f.radius_ut * 1.55;
    let e = refusal(|_| {}, |m| m.intensity_ut = expected);
    assert_eq!(
        e.message(),
        format!(
            "Field strength {:.1} uT against {expected:.1} expected here: compass faulty or a magnet nearby",
            f.radius_ut
        )
    );
    let e = refusal(|_| {}, |m| m.intensity_ut = f.radius_ut / 1.55);
    assert!(matches!(e, Refusal::Scale { .. }));
}

/// The radius row governs the field strength (not PX4's 20..70 uT convergence window): a true field of 12 or 100 uT
/// converges and is refused by the radius row; 80 uT passes the radius row and is refused by the scale row against
/// the 43.7 uT model.
#[test]
fn implausible_and_off_model_field_strengths() {
    let m = model();
    for field in [12.0, 100.0, 80.0] {
        let d = rings_of(&[0, 1, 2, 3, 4, 5], 0x00C0_FFEE, field);
        let f = fit(&d.raw, FIELD_UT);
        assert!(f.converged(), "{field} uT: {f:?}");
        let e = accept::accept(&inputs(&f, &d, &m)).unwrap_err();
        println!("true field {field} uT: radius {:.2} uT, {e}", f.radius_ut);
        if field == 80.0 {
            assert!(f.radius_ut > accept::RADIUS_MIN_UT && f.radius_ut < accept::RADIUS_MAX_UT);
            assert_eq!(
                e.message(),
                format!(
                    "Field strength {:.1} uT against {FIELD_UT:.1} expected here: compass faulty or a magnet nearby",
                    f.radius_ut
                )
            );
        } else {
            assert_eq!(
                e.message(),
                format!(
                    "Field strength {:.1} uT is not plausible: compass faulty or a magnet nearby",
                    f.radius_ut
                )
            );
        }
    }
}

/// A diagonal that passes before the scaling to the model and not after it is refused with the distortion text.
#[test]
fn scaled_terms_are_checked_again() {
    let (d, mut f) = six();
    f.diag[0] = 4.0;
    let mut m = model();
    m.intensity_ut = f.radius_ut * 1.4;
    let wide = Inputs {
        inclination_tolerance_deg: 90.0,
        ..inputs(&f, &d, &m)
    };
    let e = accept::accept(&wide).unwrap_err();
    assert_eq!(e, Refusal::Distortion);
    assert_eq!(
        e.message(),
        "Compass distortion too large: is something magnetic mounted near it?"
    );
    // Control: the same fit at correction 1.2 (scaled diagonal 4.8) is accepted.
    m.intensity_ut = f.radius_ut * 1.2;
    let ok = Inputs {
        inclination_tolerance_deg: 90.0,
        ..inputs(&f, &d, &m)
    };
    assert!(accept::accept(&ok).is_ok());
}
