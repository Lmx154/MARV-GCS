//! Pose recognition from one accelerometer sample (decision section 3): the six poses at their exact readings,
//! 25 degrees of tilt recognised, 40 degrees not, and one reading is never two poses.

use gcs_magcal::Vec3;
use gcs_magcal::pose::{ONE_G, POSE_TOLERANCE_MPS2, Pose, recognise};

mod common;
use common::{mv, rotation};

#[test]
fn exact_poses_are_recognised_and_the_table_is_the_frd_convention() {
    // Derived from the convention: FRD, the accelerometer reads specific force = -g * (unit down in the body).
    // Level: down is +z. Upside down: -z. Nose down: the nose (+x) points down, so down is +x. Nose up: -x.
    // Left side down: the left side is -y, so down is -y. Right side down: +y.
    let down: [(Pose, Vec3); 6] = [
        (Pose::Level, [0.0, 0.0, 1.0]),
        (Pose::UpsideDown, [0.0, 0.0, -1.0]),
        (Pose::NoseDown, [1.0, 0.0, 0.0]),
        (Pose::NoseUp, [-1.0, 0.0, 0.0]),
        (Pose::LeftSideDown, [0.0, -1.0, 0.0]),
        (Pose::RightSideDown, [0.0, 1.0, 0.0]),
    ];
    for (pose, d) in down {
        let reading = [-ONE_G * d[0], -ONE_G * d[1], -ONE_G * d[2]];
        assert_eq!(pose.expected_accel(), reading, "{pose}");
        assert_eq!(recognise(reading), Some(pose), "{pose}");
    }
    assert_eq!(Pose::Level.expected_accel(), [0.0, 0.0, -ONE_G]);
    assert_eq!(Pose::UpsideDown.expected_accel(), [0.0, 0.0, ONE_G]);
    assert_eq!(Pose::NoseDown.expected_accel(), [-ONE_G, 0.0, 0.0]);
    assert_eq!(Pose::NoseUp.expected_accel(), [ONE_G, 0.0, 0.0]);
    assert_eq!(Pose::LeftSideDown.expected_accel(), [0.0, ONE_G, 0.0]);
    assert_eq!(Pose::RightSideDown.expected_accel(), [0.0, -ONE_G, 0.0]);
    for (i, p) in Pose::ALL.into_iter().enumerate() {
        assert_eq!(p.index(), i);
    }
}

/// The expected reading of `pose` tilted by `deg` about the body axis `axis`.
fn tilted(pose: Pose, axis: usize, deg: f64) -> Vec3 {
    let mut a = [0.0; 3];
    a[axis] = 1.0;
    mv(&rotation(a, deg.to_radians()), pose.expected_accel())
}

#[test]
fn tilt_of_25_degrees_is_recognised_and_40_degrees_is_none() {
    for pose in Pose::ALL {
        let e = pose.expected_accel();
        let main = (0..3).find(|&i| e[i] != 0.0).unwrap();
        for axis in (0..3).filter(|&i| i != main) {
            for sign in [-1.0, 1.0] {
                assert_eq!(
                    recognise(tilted(pose, axis, 25.0 * sign)),
                    Some(pose),
                    "{pose} 25 deg about {axis}"
                );
                assert_eq!(
                    recognise(tilted(pose, axis, 40.0 * sign)),
                    None,
                    "{pose} 40 deg about {axis}"
                );
            }
        }
    }
    // PX4's limit along one axis: asin(5 / g) = 30.6 deg.
    let limit = (POSE_TOLERANCE_MPS2 / ONE_G).asin().to_degrees();
    assert!((limit - 30.65).abs() < 0.1, "{limit}");
    assert_eq!(recognise(tilted(Pose::Level, 0, 30.0)), Some(Pose::Level));
    assert_eq!(recognise(tilted(Pose::Level, 0, 31.5)), None);
}

#[test]
fn garbage_is_none() {
    assert_eq!(recognise([0.0, 0.0, 0.0]), None);
    assert_eq!(recognise([f64::NAN, 0.0, -ONE_G]), None);
    assert_eq!(recognise([0.0, 0.0, -2.0 * ONE_G]), None);
    assert_eq!(recognise([6.0, 6.0, -6.0]), None);
}

#[test]
fn no_reading_matches_two_poses_at_rest_magnitudes() {
    // Over a grid of readings of 0.9 to 1.1 g (any resting vehicle), at most one pose's test passes.
    let mut checked = 0;
    let n = 120;
    for i in 0..=n {
        for j in 0..=n {
            for k in 0..=n {
                let a = [
                    -12.0 + 24.0 * i as f64 / n as f64,
                    -12.0 + 24.0 * j as f64 / n as f64,
                    -12.0 + 24.0 * k as f64 / n as f64,
                ];
                let mag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                if !(0.9 * ONE_G..=1.1 * ONE_G).contains(&mag) {
                    continue;
                }
                checked += 1;
                let matching = Pose::ALL.iter().filter(|p| p.matches(a)).count();
                assert!(matching <= 1, "{a:?} matches {matching} poses");
                assert_eq!(recognise(a).is_some(), matching == 1, "{a:?}");
            }
        }
    }
    assert!(checked > 100_000);
}

#[test]
fn px4_boxes_overlap_on_a_45_degree_tilt_and_the_nearest_pose_wins() {
    // (4.9, 0, -4.9) is 0.7 g: both PX4 tests pass. One pose is returned: the nearer; ties go to the earlier.
    let a = [4.9, 0.0, -4.9];
    assert!(Pose::Level.matches(a) && Pose::NoseUp.matches(a));
    assert_eq!(recognise(a), Some(Pose::Level)); // distance to (0,0,-g) is 4.91*sqrt2 either way: tie, earlier wins
    assert_eq!(recognise([4.95, 0.0, -4.85]), Some(Pose::NoseUp));
}
