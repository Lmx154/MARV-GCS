//! The heading check (decision section 5): straight legs with a known compass error, and too few records.

use gcs_magcal::heading::{
    self, GpsRecord, HeadingCheck, ImuSample, MIN_RECORDS, MIN_SPEED_MPS, TOLERANCE_DEG,
};
use gcs_magcal::pose::ONE_G;
use gcs_magcal::wmm;

mod common;
use common::*;

const SITE: (f64, f64, f64, f64) = (33.0, -145.0, 12_000.0, 2026.5);

/// A walk of `n_fast` GPS records at 1 Hz (course around the compass, speed 1.4 m/s) and `n_slow` records at 0.5 m/s
/// with a nonsense course, and the IMU at 25 Hz. The compass reads `error_deg` more than the truth: the vehicle's
/// attitude is generated with heading course + error. `course_noise_deg` is the GPS course noise.
fn walk(
    error_deg: f64,
    n_fast: usize,
    n_slow: usize,
    course_noise_deg: f64,
) -> (Vec<ImuSample>, Vec<GpsRecord>, f64) {
    let f = wmm::field(SITE.0, SITE.1, SITE.2, SITE.3).unwrap();
    let mut rng = Rng(0xFEED);
    let (mut imu, mut gps) = (Vec::new(), Vec::new());
    let total = n_fast + n_slow;
    for sec in 0..total {
        let slow = sec >= n_fast;
        let course = if slow {
            123.0
        } else {
            (sec as f64 * 53.0 + 355.0) % 360.0
        };
        let heading = course + error_deg;
        for k in 0..25 {
            // A little wobble in pitch and roll, as carried by hand.
            let att = attitude(
                heading,
                20.0 + 5.0 * (k as f64 / 4.0).sin(),
                -15.0 + 4.0 * (k as f64 / 3.0).cos(),
            );
            let b = mv(&att, f.ned_ut);
            let a = mv(&att, [0.0, 0.0, 1.0]).map(|d| -ONE_G * d);
            imu.push(ImuSample {
                time_usec: sec as u64 * 1_000_000 + k as u64 * 40_000,
                accel: to_f32(a),
                mag: to_f32(b.map(|v| v / 100.0)),
            });
        }
        gps.push(GpsRecord {
            // GPS records come a few tens of ms away from an IMU sample.
            time_usec: sec as u64 * 1_000_000 + 13_000,
            course_deg: (course + course_noise_deg * rng.gauss()).rem_euclid(360.0),
            speed_mps: if slow { 0.5 } else { 1.4 },
        });
    }
    (imu, gps, f.declination_deg)
}

#[test]
fn the_constants_are_the_proposals() {
    assert_eq!(MIN_SPEED_MPS, 1.0);
    assert_eq!(MIN_RECORDS, 10);
    assert_eq!(TOLERANCE_DEG, 10.0);
}

#[test]
fn a_7_degree_error_is_within_10() {
    let (imu, gps, decl) = walk(7.0, 15, 4, 0.0);
    match heading::check(&imu, &gps, decl) {
        HeadingCheck::Measured {
            records,
            median_deg,
            spread_deg,
            within,
        } => {
            assert_eq!(records, 15, "the slow records are not used");
            assert!((median_deg - 7.0).abs() < 0.05, "{median_deg}");
            assert!(spread_deg < 0.05, "{spread_deg}");
            assert!(within);
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(heading::check(&imu, &gps, decl).message(), "within 10 deg");
}

#[test]
fn a_25_degree_error_is_off_by_25() {
    let (imu, gps, decl) = walk(25.0, 15, 4, 0.0);
    let r = heading::check(&imu, &gps, decl);
    assert_eq!(
        r.message(),
        "off by 25 deg: calibrate again or check what is mounted near the compass"
    );
    let (imu, gps, decl) = walk(-25.0, 15, 0, 0.0);
    assert_eq!(
        heading::check(&imu, &gps, decl).message(),
        "off by 25 deg: calibrate again or check what is mounted near the compass"
    );
}

#[test]
fn with_gps_noise_the_median_is_close_and_the_spread_is_the_mad() {
    let sigma = 4.0;
    let (imu, gps, decl) = walk(25.0, 60, 0, sigma);
    let HeadingCheck::Measured {
        median_deg,
        spread_deg,
        records,
        ..
    } = heading::check(&imu, &gps, decl)
    else {
        panic!()
    };
    assert_eq!(records, 60);
    assert!((median_deg - 25.0).abs() < 2.0, "{median_deg}");
    // The MAD of a Gaussian is 0.6745 sigma.
    assert!((spread_deg - 0.6745 * sigma).abs() < 1.2, "{spread_deg}");
}

#[test]
fn nine_records_are_not_enough() {
    let (imu, gps, decl) = walk(7.0, 9, 6, 0.0);
    let r = heading::check(&imu, &gps, decl);
    assert_eq!(r, HeadingCheck::NotEnough { records: 9 });
    assert_eq!(
        r.message(),
        "not enough straight-line movement yet (9 of 10)"
    );
    let (imu, gps, decl) = walk(7.0, 10, 0, 0.0);
    assert!(matches!(
        heading::check(&imu, &gps, decl),
        HeadingCheck::Measured { records: 10, .. }
    ));
    assert_eq!(
        heading::check(&[], &[], 0.0).message(),
        "not enough straight-line movement yet (0 of 10)"
    );
}

#[test]
fn heading_of_a_tilted_compass_is_tilt_compensated() {
    // Level, nose to the field's north: heading 0. Nose to the east: 90.
    let f = wmm::field(SITE.0, SITE.1, SITE.2, SITE.3).unwrap();
    for (yaw, pitch, roll) in [
        (0.0, 0.0, 0.0),
        (90.0, 25.0, -10.0),
        (200.0, -30.0, 35.0),
        (-45.0, 0.0, 60.0),
    ] {
        let att = attitude(yaw, pitch, roll);
        let b = mv(&att, f.ned_ut);
        let a = mv(&att, [0.0, 0.0, 1.0]).map(|d| -ONE_G * d);
        let h = heading::compass_heading_deg(to_f32(a), to_f32(b)).unwrap();
        // The horizontal field is `declination` east of true north, so the compass reads yaw - declination.
        let want = (yaw - f.declination_deg).rem_euclid(360.0);
        let d = (h - want + 180.0).rem_euclid(360.0) - 180.0;
        assert!(d.abs() < 0.01, "yaw {yaw}: {h} against {want}");
    }
    assert_eq!(
        heading::compass_heading_deg([0.0; 3], [1.0, 0.0, 0.0]),
        None
    );
}
