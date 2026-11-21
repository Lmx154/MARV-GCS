//! I3: the station's undo agrees with the firmware's apply (link contract section 16, test vector) and undoes any
//! positive-definite calibration (round-trip property).

use gcs_magcal::undo::{Calibration, INVALID_MATRIX_MESSAGE, InvalidMatrix, gauss_to_ut};

fn f(bits: u32) -> f64 {
    f64::from(f32::from_bits(bits))
}

/// The contract's worked vector: S, o, r and the published m (f32 bit patterns).
fn contract() -> (Calibration, [f64; 3], [f64; 3]) {
    let cal = Calibration {
        offsets: [f(0xC163_3333), f(0xC086_6666), f(0x3FC0_0000)],
        diag: [1.0625, 0.9375, 0.78125],
        offdiag: [0.03125, -0.046875, 0.0625],
    };
    let r = [f(0x41A4_0000), f(0xC050_0000), f(0x4218_0000)];
    let m = [f(0x420C_C000), f(0x4088_3333), f(0x41D7_9666)];
    (cal, r, m)
}

#[test]
fn contract_vector_undo_gives_r() {
    let (cal, r, m) = contract();
    assert_eq!(r, [20.5, -3.25, 38.0]);
    let back = cal.undo().unwrap().raw(m);
    for i in 0..3 {
        assert!(
            (back[i] - r[i]).abs() < 1e-4,
            "axis {i}: {} against {}",
            back[i],
            r[i]
        );
    }
}

#[test]
fn contract_vector_apply_in_f64_matches_the_f32_result() {
    let (cal, r, m) = contract();
    let got = cal.apply(r);
    for i in 0..3 {
        assert!(
            (got[i] - m[i]).abs() < 1e-4,
            "axis {i}: {} against {}",
            got[i],
            m[i]
        );
    }
}

#[test]
fn defaults_undo_to_m_plus_o() {
    let cal = Calibration {
        offsets: [3.0, -4.0, 5.0],
        ..Calibration::default()
    };
    assert_eq!(cal.undo().unwrap().raw([1.0, 2.0, 3.0]), [4.0, -2.0, 8.0]);
}

/// SplitMix64: a fixed-seed generator for the property test.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }
}

#[test]
fn round_trip_over_random_positive_definite_calibrations() {
    let mut rng = Rng(0x5EED_0003);
    let mut tested = 0;
    let mut worst = 0.0f64;
    while tested < 10_000 {
        let cal = Calibration {
            offsets: [
                rng.range(-200.0, 200.0),
                rng.range(-200.0, 200.0),
                rng.range(-200.0, 200.0),
            ],
            diag: [
                rng.range(0.2, 5.0),
                rng.range(0.2, 5.0),
                rng.range(0.2, 5.0),
            ],
            offdiag: [
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
                rng.range(-1.0, 1.0),
            ],
        };
        let Ok(undo) = cal.undo() else { continue };
        let r = [
            rng.range(-150.0, 150.0),
            rng.range(-150.0, 150.0),
            rng.range(-150.0, 150.0),
        ];
        let back = undo.raw(cal.apply(r));
        for i in 0..3 {
            worst = worst.max((back[i] - r[i]).abs());
        }
        tested += 1;
    }
    assert!(worst < 1e-6, "worst round-trip error {worst} uT");
}

#[test]
fn a_matrix_that_is_not_positive_definite_is_refused_with_the_decision_text() {
    let indefinite = Calibration {
        offdiag: [0.9, 0.0, 0.0],
        diag: [0.5, 0.5, 1.0],
        ..Calibration::default()
    };
    assert_eq!(indefinite.undo(), Err(InvalidMatrix));
    let negative = Calibration {
        diag: [1.0, -1.0, 1.0],
        ..Calibration::default()
    };
    let err = negative.undo().unwrap_err();
    assert_eq!(
        err.message(),
        "The stored compass matrix is not valid. Reset the compass calibration first."
    );
    assert_eq!(err.to_string(), INVALID_MATRIX_MESSAGE);
}

#[test]
fn highres_imu_gauss_to_microtesla() {
    let ut = gauss_to_ut([0.5, -0.25, 0.437]);
    assert_eq!(ut[0], 50.0);
    assert_eq!(ut[1], -25.0);
    assert!((ut[2] - 43.7).abs() < 1e-5);
}
