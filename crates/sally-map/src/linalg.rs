//! The map solve's small linear algebra (driver_map_solve.c's own: row-major 3x3 rotations, rotation vectors, a
//! pivoting Gauss solve and a 6x6 Gauss-Jordan inverse) and linmath's quatinnerproduct, which sally-math does not have,
//! operation for operation. (Its PoseSlerp is sally-math's pose_slerp.)

use sally_math::{c_acos, c_cos, c_sin, Quat};

/// q2R: a quaternion (w x y z, normalized here) to a row-major rotation matrix
pub fn q2r(q: &Quat) -> [f64; 9] {
	let (mut w, mut x, mut y, mut z) = (q[0], q[1], q[2], q[3]);
	let mut n = w * w + x * x + y * y + z * z;
	if n > 0.0 {
		n = 1. / n.sqrt();
		w *= n;
		x *= n;
		y *= n;
		z *= n;
	}
	[
		1.0 - 2.0 * (y * y + z * z),
		2.0 * (x * y - z * w),
		2.0 * (x * z + y * w),
		2.0 * (x * y + z * w),
		1.0 - 2.0 * (x * x + z * z),
		2.0 * (y * z - x * w),
		2.0 * (x * z - y * w),
		2.0 * (y * z + x * w),
		1.0 - 2.0 * (x * x + y * y),
	]
}

/// R2q: a rotation matrix to its quaternion, w >= 0
pub fn r2q(r: &[f64; 9]) -> Quat {
	let tr = r[0] + r[4] + r[8];
	let mut q = if tr > 0.0 {
		let s = (tr + 1.).sqrt() * 2.0;
		[0.25 * s, (r[7] - r[5]) / s, (r[2] - r[6]) / s, (r[3] - r[1]) / s]
	} else if r[0] > r[4] && r[0] > r[8] {
		let s = (1. + r[0] - r[4] - r[8]).sqrt() * 2.0;
		[(r[7] - r[5]) / s, 0.25 * s, (r[1] + r[3]) / s, (r[2] + r[6]) / s]
	} else if r[4] > r[8] {
		let s = (1. + r[4] - r[0] - r[8]).sqrt() * 2.0;
		[(r[2] - r[6]) / s, (r[1] + r[3]) / s, 0.25 * s, (r[5] + r[7]) / s]
	} else {
		let s = (1. + r[8] - r[0] - r[4]).sqrt() * 2.0;
		[(r[3] - r[1]) / s, (r[2] + r[6]) / s, (r[5] + r[7]) / s, 0.25 * s]
	};
	if q[0] < 0.0 {
		for v in q.iter_mut() {
			*v = -*v;
		}
	}
	q
}

/// rv2R: a rotation vector to its rotation matrix (Rodrigues); below 1e-15 rad the first-order matrix
pub fn rv2r(r: &[f64; 3]) -> [f64; 9] {
	let a = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
	if a < 1e-15 {
		return [1.0, -r[2], r[1], r[2], 1.0, -r[0], -r[1], r[0], 1.0];
	}
	let k = [r[0] / a, r[1] / a, r[2] / a];
	let s = c_sin(a);
	let c = 1.0 - c_cos(a);
	let kx = [0.0, -k[2], k[1], k[2], 0.0, -k[0], -k[1], k[0], 0.0];
	let mut out = [0.0; 9];
	for i in 0..3 {
		for j in 0..3 {
			let mut kk = 0.0;
			for m in 0..3 {
				kk += kx[i * 3 + m] * kx[m * 3 + j];
			}
			out[i * 3 + j] = (i == j) as i32 as f64 + s * kx[i * 3 + j] + c * kk;
		}
	}
	out
}

/// R2rv: a rotation matrix to its rotation vector (the angle from the trace, clamped to [-1, 1])
pub fn r2rv(r: &[f64; 9]) -> [f64; 3] {
	let mut c = (r[0] + r[4] + r[8] - 1.0) / 2.0;
	if c > 1.0 {
		c = 1.0;
	}
	if c < -1.0 {
		c = -1.0;
	}
	let a = c_acos(c);
	let s = c_sin(a);
	let f = if a < 1e-9 { 0.5 } else { a / (2.0 * s) };
	[(r[7] - r[5]) * f, (r[2] - r[6]) * f, (r[3] - r[1]) * f]
}

/// mm3: A B
pub fn mm3(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
	let mut t = [0.0; 9];
	for i in 0..3 {
		for j in 0..3 {
			t[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
		}
	}
	t
}

/// mv3: A x
pub fn mv3(a: &[f64; 9], x: &[f64; 3]) -> [f64; 3] {
	let mut t = [0.0; 3];
	for i in 0..3 {
		t[i] = a[i * 3] * x[0] + a[i * 3 + 1] * x[1] + a[i * 3 + 2] * x[2];
	}
	t
}

/// mtv3: A^T x
pub fn mtv3(a: &[f64; 9], x: &[f64; 3]) -> [f64; 3] {
	let mut t = [0.0; 3];
	for i in 0..3 {
		t[i] = a[i] * x[0] + a[3 + i] * x[1] + a[6 + i] * x[2];
	}
	t
}

/// The transpose of a row-major 3x3 (the C's `Rt[i * 3 + j] = R[j * 3 + i]` loops)
pub fn transpose3(r: &[f64; 9]) -> [f64; 9] {
	let mut t = [0.0; 9];
	for i in 0..3 {
		for j in 0..3 {
			t[i * 3 + j] = r[j * 3 + i];
		}
	}
	t
}

/// gsolve: solve A x = b (n x n, row-major) by elimination with partial pivoting, A and b overwritten (b becomes x);
/// false if a pivot is below 1e-300
pub fn gsolve(a: &mut [f64], b: &mut [f64], n: usize) -> bool {
	assert!(a.len() >= n * n && b.len() >= n);
	for c in 0..n {
		let mut p = c;
		for r in c + 1..n {
			if a[r * n + c].abs() > a[p * n + c].abs() {
				p = r;
			}
		}
		if a[p * n + c].abs() < 1e-300 {
			return false;
		}
		if p != c {
			for k in 0..n {
				a.swap(c * n + k, p * n + k);
			}
			b.swap(c, p);
		}
		for r in c + 1..n {
			let f = a[r * n + c] / a[c * n + c];
			if f == 0.0 {
				continue;
			}
			for k in c..n {
				a[r * n + k] -= f * a[c * n + k];
			}
			b[r] -= f * b[c];
		}
	}
	for r in (0..n).rev() {
		let mut s = b[r];
		for k in r + 1..n {
			s -= a[r * n + k] * b[k];
		}
		b[r] = s / a[r * n + r];
	}
	true
}

/// inv6: a 6x6 inverse by Gauss-Jordan with partial pivoting; None if a pivot is below 1e-300
pub fn inv6(a: &[f64]) -> Option<[f64; 36]> {
	let mut m = [[0.0f64; 12]; 6];
	for i in 0..6 {
		for j in 0..12 {
			m[i][j] = if j < 6 { a[i * 6 + j] } else { (j - 6 == i) as i32 as f64 };
		}
	}
	for c in 0..6 {
		let mut p = c;
		for r in c + 1..6 {
			if m[r][c].abs() > m[p][c].abs() {
				p = r;
			}
		}
		if m[p][c].abs() < 1e-300 {
			return None;
		}
		if p != c {
			m.swap(c, p);
		}
		let d = m[c][c];
		for k in 0..12 {
			m[c][k] /= d;
		}
		for r in 0..6 {
			if r != c && m[r][c] != 0.0 {
				let f = m[r][c];
				for k in 0..12 {
					m[r][k] -= f * m[c][k];
				}
			}
		}
	}
	let mut ai = [0.0; 36];
	for i in 0..6 {
		for j in 0..6 {
			ai[i * 6 + j] = m[i][j + 6];
		}
	}
	Some(ai)
}

/// linmath's quatinnerproduct
#[inline]
pub fn quatinnerproduct(qa: &Quat, qb: &Quat) -> f64 {
	(qa[0] * qb[0]) + (qa[1] * qb[1]) + (qa[2] * qb[2]) + (qa[3] * qb[3])
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn rotations_round_trip() {
		let q = [0.9, 0.1, -0.3, 0.2];
		let r = q2r(&q);
		let q2 = r2q(&r);
		let n = quatinnerproduct(&q, &q).sqrt();
		for k in 0..4 {
			assert!((q2[k] - q[k] / n).abs() < 1e-12);
		}
		let rv = [0.1, -0.2, 0.05];
		let back = r2rv(&rv2r(&rv));
		for k in 0..3 {
			assert!((back[k] - rv[k]).abs() < 1e-12);
		}
		let rt = mm3(&r, &transpose3(&r));
		for i in 0..3 {
			for j in 0..3 {
				assert!((rt[i * 3 + j] - (i == j) as i32 as f64).abs() < 1e-12);
			}
		}
	}

	#[test]
	fn solves_and_inverts() {
		let a0 = [4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0];
		let (mut a, mut b) = (a0, [1.0, 2.0, 3.0]);
		assert!(gsolve(&mut a, &mut b, 3));
		for i in 0..3 {
			let s = a0[i * 3] * b[0] + a0[i * 3 + 1] * b[1] + a0[i * 3 + 2] * b[2];
			assert!((s - [1.0, 2.0, 3.0][i]).abs() < 1e-12);
		}
		let mut m = [0.0; 36];
		for i in 0..6 {
			for j in 0..6 {
				m[i * 6 + j] = if i == j { 3.0 + i as f64 } else { 1.0 / (1.0 + (i + j) as f64) };
			}
		}
		let mi = inv6(&m).unwrap();
		for i in 0..6 {
			for j in 0..6 {
				let s: f64 = (0..6).map(|k| m[i * 6 + k] * mi[k * 6 + j]).sum();
				assert!((s - (i == j) as i32 as f64).abs() < 1e-12);
			}
		}
		assert!(inv6(&[0.0; 36]).is_none());
	}
}
