//! Thinning: PX4's spacing rule (mag_calibration.cpp:147-149), in uT on the raw field.

use gcs_magcal::collect::{Collector, min_spacing_ut};
use gcs_magcal::undo::Calibration;

#[test]
fn the_spacing_is_px4s_rule() {
    // 5.4 * R / sqrt(240) / 3 = 0.11619 R.
    let r = 43.7;
    assert!((min_spacing_ut(r) - 5.4 * r / 240f64.sqrt() / 3.0).abs() < 1e-12);
    assert!((min_spacing_ut(r) - 5.0777).abs() < 1e-3);
    assert_eq!(min_spacing_ut(-r), min_spacing_ut(r), "fabsf");
}

#[test]
fn a_sample_closer_than_the_spacing_to_any_kept_one_is_rejected() {
    let r = 40.0;
    let d = min_spacing_ut(r);
    let mut c = Collector::new(Calibration::default().undo().unwrap(), r);
    let down = [0.0, 0.0, 1.0];
    let gauss = |x_ut: f64| [(x_ut / 100.0) as f32, 0.0, 0.0];
    assert!(c.offer(gauss(0.0), down));
    assert!(!c.offer(gauss(0.0), down));
    assert!(!c.offer(gauss(0.9 * d), down));
    assert!(c.offer(gauss(1.1 * d), down));
    // 0.9 d from the second but 2 d from the first: still rejected (any kept sample counts).
    assert!(!c.offer(gauss(1.1 * d + 0.9 * d), down));
    assert!(!c.offer([f32::NAN, 0.0, 0.0], down));
    assert_eq!(c.len(), 2);
}

#[test]
fn the_raw_field_is_the_published_one_with_the_calibration_undone() {
    let cal = Calibration {
        offsets: [10.0, -5.0, 2.0],
        diag: [1.2, 0.9, 1.1],
        offdiag: [0.05, 0.0, -0.02],
    };
    let raw = [30.0, -12.0, 41.0];
    let m = cal.apply(raw);
    let mut c = Collector::new(cal.undo().unwrap(), 40.0);
    assert!(c.offer(
        [
            (m[0] / 100.0) as f32,
            (m[1] / 100.0) as f32,
            (m[2] / 100.0) as f32
        ],
        [0.0, 0.0, 1.0]
    ));
    let got = c.samples()[0].raw;
    for i in 0..3 {
        assert!((got[i] - raw[i]).abs() < 1e-3, "{got:?}");
    }
}
