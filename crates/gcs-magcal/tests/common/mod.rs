//! Shared synthetic-data helpers for the procedure tests (not a test file itself).
#![allow(dead_code)]

use gcs_magcal::Vec3;

pub type M3 = [[f64; 3]; 3];

pub fn mv(m: &M3, v: Vec3) -> Vec3 {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}
pub fn mm(a: &M3, b: &M3) -> M3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}
pub fn transpose(m: &M3) -> M3 {
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| m[j][i]))
}
pub fn inv3(m: &M3) -> M3 {
    let c = |r: usize, s: usize| {
        let (r1, r2, s1, s2) = ((r + 1) % 3, (r + 2) % 3, (s + 1) % 3, (s + 2) % 3);
        m[r1][s1] * m[r2][s2] - m[r1][s2] * m[r2][s1]
    };
    let det = m[0][0] * c(0, 0) + m[0][1] * c(0, 1) + m[0][2] * c(0, 2);
    [0, 1, 2].map(|i| [0, 1, 2].map(|j| c(j, i) / det))
}
pub fn to_f32(v: Vec3) -> [f32; 3] {
    v.map(|x| x as f32)
}

/// SplitMix64 with Box-Muller: the fixed-seed noise generator (as in `tests/fit.rs`).
pub struct Rng(pub u64);
impl Rng {
    pub fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    pub fn gauss(&mut self) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// Body-from-NED attitude of a vehicle with heading `yaw`, pitch and roll (degrees, aerospace 3-2-1).
pub fn attitude(yaw_deg: f64, pitch_deg: f64, roll_deg: f64) -> M3 {
    let (sy, cy) = yaw_deg.to_radians().sin_cos();
    let (sp, cp) = pitch_deg.to_radians().sin_cos();
    let (sr, cr) = roll_deg.to_radians().sin_cos();
    let rz: M3 = [[cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0]];
    let ry: M3 = [[cp, 0.0, -sp], [0.0, 1.0, 0.0], [sp, 0.0, cp]];
    let rx: M3 = [[1.0, 0.0, 0.0], [0.0, cr, sr], [0.0, -sr, cr]];
    mm(&rx, &mm(&ry, &rz))
}

/// The six poses as body-from-level rotations, in `Pose::ALL` order (as in `tests/fit.rs`).
pub const POSES: [M3; 6] = [
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
    [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
    [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
];

/// Body-from-NED attitude in pose `pose` (an index into `POSES`) at heading `yaw_deg`.
pub fn pose_attitude(pose: usize, yaw_deg: f64) -> M3 {
    mm(&POSES[pose], &attitude(yaw_deg, 0.0, 0.0))
}

/// Rotation by `angle` about the unit `axis` (Rodrigues).
pub fn rotation(axis: Vec3, angle: f64) -> M3 {
    let (s, c) = angle.sin_cos();
    let (x, y, z) = (axis[0], axis[1], axis[2]);
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

/// Axis and angle of a rotation matrix (angle in [0, pi)); an arbitrary axis for the identity.
pub fn axis_angle(r: &M3) -> (Vec3, f64) {
    let angle = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0)
        .clamp(-1.0, 1.0)
        .acos();
    let v = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n < 1e-12 {
        ([0.0, 0.0, 1.0], 0.0)
    } else {
        ([v[0] / n, v[1] / n, v[2] / n], angle)
    }
}

/// Body angular rate, rad/s, from two attitudes `dt` seconds apart: R' = -[w]x R, so [w]x = -(R2 R1^T - I) / dt.
pub fn body_rate(r1: &M3, r2: &M3, dt: f64) -> Vec3 {
    let d = mm(r2, &transpose(r1));
    [
        -(d[2][1] - d[1][2]) / (2.0 * dt),
        -(d[0][2] - d[2][0]) / (2.0 * dt),
        -(d[1][0] - d[0][1]) / (2.0 * dt),
    ]
}
