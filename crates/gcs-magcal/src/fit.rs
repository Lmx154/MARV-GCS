// A port to Rust of PX4's magnetometer fit, src/modules/commander/lm_fit.cpp and lm_fit.hpp, and of the order
// in which src/modules/commander/mag_calibration.cpp calls it (sphere, then ellipsoid). The original notice:
//
// ***************************************************************************
//
//   Copyright (c) 2021 PX4 Development Team. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name PX4 nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ***************************************************************************

//! PX4's Levenberg-Marquardt magnetometer fit: a sphere fit (offset and radius), then an ellipsoid fit (offset,
//! three diagonal and three off-diagonal terms at the sphere's radius). Model: |S (r - o)| = radius.
//!
//! What differs from the C++: f64 instead of f32. PX4 works in gauss and its thresholds (`cost_threshold`,
//! `step_threshold`, the radius window, the initial damping) are in gauss units, so the samples are converted to
//! gauss inside and the results back to uT; every iteration count, damping factor and acceptance rule is PX4's.
//! In PX4's sphere iteration the trial residual for `fit1` reads `z[k] + fit1_params[3]` in two terms
//! (lm_fit.cpp:124, :127); those terms are multiplied by the off-diagonal, which is zero during the sphere fit,
//! so writing `z - offset` here changes nothing. Two other PX4 behaviours are kept as they are because they
//! decide the iteration path: the ellipsoid iteration's `fit1` also sums the residuals at the current point
//! (lm_fit.cpp:202, not reset before :267), and when both trials are worse the parameters still take the
//! `fit1` trial while the recorded cost stays (lm_fit.cpp:149-170, :283-308).

use crate::Vec3;
use crate::linalg;
use crate::undo::UT_PER_GAUSS;

/// LM damping factor, lm_fit.cpp:48 and :180.
const LMA_DAMPING: f64 = 10.0;
/// lm_fit.cpp:320-323.
const MAX_ITERATIONS: usize = 100;
const MIN_ITERATIONS: usize = 10;
const COST_THRESHOLD: f64 = 0.01;
const STEP_THRESHOLD: f64 = 0.001;
/// The radius window of a converged fit, gauss (lm_fit.cpp:325-326; 20 to 70 uT).
const MIN_RADIUS_GAUSS: f64 = 0.2;
const MAX_RADIUS_GAUSS: f64 = 0.7;
/// Initial cost and damping, lm_fit.cpp:329-330.
const INITIAL_COST: f64 = 1e30;
const INITIAL_DAMPING: f64 = 1.0;

/// The result of [`fit`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fit {
    /// o, uT.
    pub offsets: Vec3,
    /// Diagonal of S.
    pub diag: Vec3,
    /// Off-diagonal of S: xy, xz, yz.
    pub offdiag: Vec3,
    /// The sphere's radius (the ellipsoid fit keeps it), uT.
    pub radius_ut: f64,
    /// RMS over the samples of (radius - |S (r - o)|), uT, before any scaling to the model.
    pub fitness_ut: f64,
    /// The sphere fit met PX4's convergence rule.
    pub sphere_converged: bool,
    /// The ellipsoid fit met PX4's convergence rule (it runs only after a converged sphere fit).
    pub ellipsoid_converged: bool,
}

impl Fit {
    /// Both stages converged.
    pub fn converged(&self) -> bool {
        self.sphere_converged && self.ellipsoid_converged
    }

    /// S in the contract's layout.
    pub fn matrix(&self) -> [[f64; 3]; 3] {
        let (d, o) = (self.diag, self.offdiag);
        [[d[0], o[0], o[1]], [o[0], d[1], o[2]], [o[1], o[2], d[2]]]
    }

    /// S (r - o) for one raw sample, uT (not scaled to the model).
    pub fn correct(&self, raw_ut: Vec3) -> Vec3 {
        linalg::mul_vec(&self.matrix(), linalg::sub(raw_ut, self.offsets))
    }
}

/// `sphere_params` (lm_fit.hpp:41-44), in gauss.
#[derive(Clone, Copy)]
struct Params {
    offset: Vec3,
    diag: Vec3,
    offdiag: Vec3,
    radius: f64,
}

/// `iteration_result` (lm_fit.cpp:36-42).
struct Iteration {
    gradient_damping: f64,
    cost: f64,
    success: bool,
}

/// (A, B, C) = S (s - o) for the given terms.
fn abc(diag: Vec3, offdiag: Vec3, offset: Vec3, s: Vec3) -> (f64, f64, f64) {
    let (dx, dy, dz) = (s[0] - offset[0], s[1] - offset[1], s[2] - offset[2]);
    (
        diag[0] * dx + offdiag[0] * dy + offdiag[1] * dz,
        offdiag[0] * dx + diag[1] * dy + offdiag[2] * dz,
        offdiag[1] * dx + offdiag[2] * dy + diag[2] * dz,
    )
}

/// The common LM decision (lm_fit.cpp:149-171 and :283-310). `fit1`/`fit2` are the trial costs.
fn decide<const N: usize>(
    result: &mut Iteration,
    fit1: f64,
    fit2: f64,
    mut fit1_params: [f64; N],
    fit2_params: [f64; N],
) -> Option<[f64; N]> {
    let mut fitness = result.cost;
    if fit1 > result.cost && fit2 > result.cost {
        result.gradient_damping *= LMA_DAMPING;
    } else if fit2 < result.cost && fit2 < fit1 {
        result.gradient_damping /= LMA_DAMPING;
        fit1_params = fit2_params;
        fitness = fit2;
    } else if fit1 < result.cost {
        fitness = fit1;
    }
    if fitness.is_finite() && fitness <= result.cost {
        result.cost = fitness;
        result.success = true;
        Some(fit1_params)
    } else {
        result.success = false;
        None
    }
}

/// The two damped steps: params - (JTJ + damping I)^-1 JTFI and the same with damping / LMA_DAMPING.
fn steps<const N: usize>(
    jtj: &[[f64; N]; N],
    jtfi: &[f64; N],
    params: [f64; N],
    damping: f64,
) -> Option<([f64; N], [f64; N])> {
    let mut a = *jtj;
    let mut b = *jtj;
    for i in 0..N {
        a[i][i] += damping;
        b[i][i] += damping / LMA_DAMPING;
    }
    let a = linalg::invert(&a)?;
    let b = linalg::invert(&b)?;
    let (mut p1, mut p2) = (params, params);
    for row in 0..N {
        for col in 0..N {
            p1[row] -= jtfi[col] * a[row][col];
            p2[row] -= jtfi[col] * b[row][col];
        }
    }
    Some((p1, p2))
}

/// `lm_sphere_fit_iteration`, lm_fit.cpp:44-174.
#[allow(clippy::needless_range_loop)] // index loops as in the C++
fn sphere_iteration(samples: &[Vec3], params: &mut Params, result: &mut Iteration) {
    let n = samples.len() as f64;
    let mut jtj = [[0.0f64; 4]; 4];
    let mut jtfi = [0.0f64; 4];
    for &s in samples {
        let (a, b, c) = abc(params.diag, params.offdiag, params.offset, s);
        let length = (a * a + b * b + c * c).sqrt();
        let (d, o) = (params.diag, params.offdiag);
        let jac = [
            1.0,
            (d[0] * a + o[0] * b + o[1] * c) / length,
            (o[0] * a + d[1] * b + o[2] * c) / length,
            (o[1] * a + o[2] * b + d[2] * c) / length,
        ];
        let residual = params.radius - length;
        for i in 0..4 {
            for j in 0..4 {
                jtj[i][j] += jac[i] * jac[j];
            }
            jtfi[i] += jac[i] * residual;
        }
    }
    // lm_fit.cpp:104, "fix numerical issues".
    for i in 0..4 {
        for j in 0..i {
            let m = (jtj[i][j] + jtj[j][i]) * 0.5;
            jtj[i][j] = m;
            jtj[j][i] = m;
        }
    }
    let current = [
        params.radius,
        params.offset[0],
        params.offset[1],
        params.offset[2],
    ];
    let Some((p1, p2)) = steps(&jtj, &jtfi, current, result.gradient_damping) else {
        result.success = false;
        return;
    };
    let (mut fit1, mut fit2) = (0.0f64, 0.0f64);
    for &s in samples {
        let (a, b, c) = abc(params.diag, params.offdiag, [p1[1], p1[2], p1[3]], s);
        let r = p1[0] - (a * a + b * b + c * c).sqrt();
        fit1 += r * r;
        let (a, b, c) = abc(params.diag, params.offdiag, [p2[1], p2[2], p2[3]], s);
        let r = p2[0] - (a * a + b * b + c * c).sqrt();
        fit2 += r * r;
    }
    let fit1 = fit1.sqrt() / n;
    let fit2 = fit2.sqrt() / n;
    if let Some(p) = decide(result, fit1, fit2, p1, p2) {
        params.radius = p[0];
        params.offset = [p[1], p[2], p[3]];
    }
}

/// `lm_ellipsoid_fit_iteration`, lm_fit.cpp:176-312.
#[allow(clippy::needless_range_loop)] // index loops as in the C++
fn ellipsoid_iteration(samples: &[Vec3], params: &mut Params, result: &mut Iteration) {
    let n = samples.len() as f64;
    let mut jtj = [[0.0f64; 9]; 9];
    let mut jtfi = [0.0f64; 9];
    let (mut fit1, mut fit2) = (0.0f64, 0.0f64);
    for &s in samples {
        let (a, b, c) = abc(params.diag, params.offdiag, params.offset, s);
        let length = (a * a + b * b + c * c).sqrt();
        let residual = params.radius - length;
        fit1 += residual * residual;
        let (d, o, off) = (params.diag, params.offdiag, params.offset);
        let (dx, dy, dz) = (s[0] - off[0], s[1] - off[1], s[2] - off[2]);
        let jac = [
            (d[0] * a + o[0] * b + o[1] * c) / length,
            (o[0] * a + d[1] * b + o[2] * c) / length,
            (o[1] * a + o[2] * b + d[2] * c) / length,
            -(dx * a) / length,
            -(dy * b) / length,
            -(dz * c) / length,
            -(dy * a + dx * b) / length,
            -(dz * a + dx * c) / length,
            -(dz * b + dy * c) / length,
        ];
        for i in 0..9 {
            for j in 0..9 {
                jtj[i][j] += jac[i] * jac[j];
            }
            jtfi[i] += jac[i] * residual;
        }
    }
    let (d, o, off) = (params.diag, params.offdiag, params.offset);
    let current = [off[0], off[1], off[2], d[0], d[1], d[2], o[0], o[1], o[2]];
    let Some((p1, p2)) = steps(&jtj, &jtfi, current, result.gradient_damping) else {
        result.success = false;
        return;
    };
    for &s in samples {
        let (a, b, c) = abc(
            [p1[3], p1[4], p1[5]],
            [p1[6], p1[7], p1[8]],
            [p1[0], p1[1], p1[2]],
            s,
        );
        let r = params.radius - (a * a + b * b + c * c).sqrt();
        fit1 += r * r;
        let (a, b, c) = abc(
            [p2[3], p2[4], p2[5]],
            [p2[6], p2[7], p2[8]],
            [p2[0], p2[1], p2[2]],
            s,
        );
        let r = params.radius - (a * a + b * b + c * c).sqrt();
        fit2 += r * r;
    }
    let fit1 = fit1.sqrt() / n;
    let fit2 = fit2.sqrt() / n;
    if let Some(p) = decide(result, fit1, fit2, p1, p2) {
        params.offset = [p[0], p[1], p[2]];
        params.diag = [p[3], p[4], p[5]];
        params.offdiag = [p[6], p[7], p[8]];
    }
}

/// `lm_mag_fit`, lm_fit.cpp:316-355: true when the fit converged.
fn lm_mag_fit(samples: &[Vec3], params: &mut Params, full_ellipsoid: bool) -> bool {
    let mut iter = Iteration {
        gradient_damping: INITIAL_DAMPING,
        cost: INITIAL_COST,
        success: true,
    };
    for i in 0..MAX_ITERATIONS {
        if full_ellipsoid {
            ellipsoid_iteration(samples, params, &mut iter);
        } else {
            sphere_iteration(samples, params, &mut iter);
        }
        if iter.success
            && MIN_RADIUS_GAUSS < params.radius
            && params.radius < MAX_RADIUS_GAUSS
            && i > MIN_ITERATIONS
            && (iter.cost < COST_THRESHOLD || iter.gradient_damping < STEP_THRESHOLD)
        {
            return true;
        }
    }
    false
}

/// Fits raw samples (body frame, uT). `initial_radius_ut` is the sphere fit's starting radius: PX4 starts at the
/// model's field strength when it has a position (mag_calibration.cpp:173-190, :565-568), as the station does.
/// The offsets start at zero and S at identity (mag_calibration.cpp:566-572). The ellipsoid fit runs only when
/// the sphere fit converged (mag_calibration.cpp:593-604).
pub fn fit(samples_ut: &[Vec3], initial_radius_ut: f64) -> Fit {
    let samples: Vec<Vec3> = samples_ut
        .iter()
        .map(|s| {
            [
                s[0] / UT_PER_GAUSS,
                s[1] / UT_PER_GAUSS,
                s[2] / UT_PER_GAUSS,
            ]
        })
        .collect();
    let mut p = Params {
        offset: [0.0; 3],
        diag: [1.0; 3],
        offdiag: [0.0; 3],
        radius: initial_radius_ut / UT_PER_GAUSS,
    };
    let (mut sphere_converged, mut ellipsoid_converged) = (false, false);
    if !samples.is_empty() {
        sphere_converged = lm_mag_fit(&samples, &mut p, false);
        if sphere_converged {
            ellipsoid_converged = lm_mag_fit(&samples, &mut p, true);
        }
    }
    let mut result = Fit {
        offsets: [
            p.offset[0] * UT_PER_GAUSS,
            p.offset[1] * UT_PER_GAUSS,
            p.offset[2] * UT_PER_GAUSS,
        ],
        diag: p.diag,
        offdiag: p.offdiag,
        radius_ut: p.radius * UT_PER_GAUSS,
        fitness_ut: f64::NAN,
        sphere_converged,
        ellipsoid_converged,
    };
    if !samples_ut.is_empty() {
        let sum: f64 = samples_ut
            .iter()
            .map(|&s| {
                let e = result.radius_ut - linalg::norm(result.correct(s));
                e * e
            })
            .sum();
        result.fitness_ut = (sum / samples_ut.len() as f64).sqrt();
    }
    result
}
