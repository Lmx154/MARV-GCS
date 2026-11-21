//! The little linear algebra the crate needs: 3x3 products and inverse, a positive-definiteness test, and the
//! inverse of a small square matrix (the 4x4 and 9x9 normal equations of the fit).

use crate::Vec3;

pub type Mat3 = [[f64; 3]; 3];

pub fn mul_vec(m: &Mat3, v: Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

/// True when the symmetric matrix `m` is positive definite (a Cholesky factorisation succeeds).
#[allow(clippy::needless_range_loop)] // index loops are the plain form of the factorisation
pub fn is_positive_definite(m: &Mat3) -> bool {
    let mut l = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..=i {
            let mut s = m[i][j];
            for k in 0..j {
                s -= l[i][k] * l[j][k];
            }
            if i == j {
                if !s.is_finite() || s <= 0.0 {
                    return false;
                }
                l[i][i] = s.sqrt();
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    true
}

/// The inverse of an N x N matrix by Gauss-Jordan elimination with partial pivoting; `None` when it is singular
/// (a zero or non-finite pivot) or the result is not finite.
#[allow(clippy::needless_range_loop)] // index loops are the plain form of the elimination
pub fn invert<const N: usize>(m: &[[f64; N]; N]) -> Option<[[f64; N]; N]> {
    let mut a = *m;
    let mut inv = [[0.0f64; N]; N];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..N {
        let mut pivot = col;
        for row in col + 1..N {
            if a[row][col].abs() > a[pivot][col].abs() {
                pivot = row;
            }
        }
        let p = a[pivot][col];
        if p == 0.0 || !p.is_finite() {
            return None;
        }
        a.swap(col, pivot);
        inv.swap(col, pivot);
        for k in 0..N {
            a[col][k] /= p;
            inv[col][k] /= p;
        }
        for row in 0..N {
            if row != col {
                let f = a[row][col];
                if f != 0.0 {
                    for k in 0..N {
                        a[row][k] -= f * a[col][k];
                        inv[row][k] -= f * inv[col][k];
                    }
                }
            }
        }
    }
    if inv.iter().flatten().all(|v| v.is_finite()) {
        Some(inv)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn inverse_times_matrix_is_identity() {
        let m = [
            [4.0, 1.0, 0.5, 0.0],
            [1.0, 3.0, 0.2, 0.1],
            [0.5, 0.2, 2.0, 0.3],
            [0.0, 0.1, 0.3, 1.0],
        ];
        let inv = invert(&m).unwrap();
        for i in 0..4 {
            for j in 0..4 {
                let s: f64 = (0..4).map(|k| m[i][k] * inv[k][j]).sum();
                assert!((s - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12);
            }
        }
        assert!(invert(&[[1.0, 2.0], [2.0, 4.0]]).is_none());
    }

    #[test]
    fn positive_definite_test() {
        assert!(is_positive_definite(&[
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]));
        assert!(!is_positive_definite(&[
            [1.0, 2.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]));
        assert!(!is_positive_definite(&[
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]));
    }
}
