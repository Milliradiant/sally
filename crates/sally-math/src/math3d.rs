//! The vectors, quaternions and rigid fit of the SteamVR driver and the panel: libsurvive's frame (metres, Z up),
//! quaternions w x y z. `Pose` here is a V3 and a Q4, not linmath's.
//!
//! As everywhere in this crate, every expression keeps its written order (`a.x * b.x + a.y * b.y + a.z * b.z` is `((a.x
//! * b.x) + (a.y * b.y)) + (a.z * b.z)`), and acos is the C library's (`c_acos`). The robust fit's selections are
//! `nth_element`.

// loops over indices, for the fixed order of operations
#![allow(clippy::needless_range_loop)]

use crate::libc::c_acos;
use crate::nth_element::nth_element;

/// A vector (math3d.h's V3)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
	pub x: f64,
	pub y: f64,
	pub z: f64,
}

impl V3 {
	pub const fn new(x: f64, y: f64, z: f64) -> V3 {
		V3 { x, y, z }
	}

	/// The components as (&a.x)[i] reads them
	pub fn get(&self, i: usize) -> f64 {
		[self.x, self.y, self.z][i]
	}
}

impl std::ops::Add for V3 {
	type Output = V3;
	fn add(self, o: V3) -> V3 {
		V3::new(self.x + o.x, self.y + o.y, self.z + o.z)
	}
}

impl std::ops::Sub for V3 {
	type Output = V3;
	fn sub(self, o: V3) -> V3 {
		V3::new(self.x - o.x, self.y - o.y, self.z - o.z)
	}
}

impl std::ops::Mul<f64> for V3 {
	type Output = V3;
	fn mul(self, s: f64) -> V3 {
		V3::new(self.x * s, self.y * s, self.z * s)
	}
}

impl std::ops::Neg for V3 {
	type Output = V3;
	fn neg(self) -> V3 {
		V3::new(-self.x, -self.y, -self.z)
	}
}

impl std::ops::AddAssign for V3 {
	fn add_assign(&mut self, o: V3) {
		self.x += o.x;
		self.y += o.y;
		self.z += o.z;
	}
}

pub fn dot(a: V3, b: V3) -> f64 {
	a.x * b.x + a.y * b.y + a.z * b.z
}

pub fn cross(a: V3, b: V3) -> V3 {
	V3::new(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}

pub fn len(a: V3) -> f64 {
	dot(a, a).sqrt()
}

/// The unit vector, or 0 for one shorter than 1e-12
pub fn norm(a: V3) -> V3 {
	let l = len(a);
	if l > 1e-12 {
		a * (1.0 / l)
	} else {
		V3::new(0.0, 0.0, 0.0)
	}
}

/// A quaternion w x y z (math3d.h's Q4; the identity by default)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Q4 {
	pub w: f64,
	pub x: f64,
	pub y: f64,
	pub z: f64,
}

impl Q4 {
	pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Q4 {
		Q4 { w, x, y, z }
	}
}

impl Default for Q4 {
	fn default() -> Q4 {
		Q4::new(1.0, 0.0, 0.0, 0.0)
	}
}

pub fn qmul(a: Q4, b: Q4) -> Q4 {
	Q4::new(
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
	)
}

pub fn qconj(q: Q4) -> Q4 {
	Q4::new(q.w, -q.x, -q.y, -q.z)
}

/// The unit quaternion, or the identity for one shorter than 1e-12 (or NaN)
pub fn qnorm(q: Q4) -> Q4 {
	let l = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
	if l > 1e-12 {
		Q4::new(q.w / l, q.x / l, q.y / l, q.z / l)
	} else {
		Q4::default()
	}
}

/// v' = v + 2w (u x v) + 2 u x (u x v)
pub fn qrot(q: Q4, v: V3) -> V3 {
	let u = V3::new(q.x, q.y, q.z);
	let t = cross(u, v) * 2.0;
	v + t * q.w + cross(u, t)
}

/// The angle of a rotation in radians (0..pi)
pub fn qangle(q: Q4) -> f64 {
	let mut w = q.w.abs();
	if w > 1.0 {
		w = 1.0;
	}
	2.0 * c_acos(w)
}

/// A rotation matrix (row major) to a quaternion
pub fn qfrom_mat(m: &[[f64; 3]; 3]) -> Q4 {
	let tr = m[0][0] + m[1][1] + m[2][2];
	let q = if tr > 0.0 {
		let s = (tr + 1.0).sqrt() * 2.0;
		Q4::new(0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
	} else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
		let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
		Q4::new((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
	} else if m[1][1] > m[2][2] {
		let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
		Q4::new((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
	} else {
		let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
		Q4::new((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
	};
	qnorm(q)
}

/// A rigid transform (math3d.h's Pose: the position and the rotation; the identity by default)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose {
	pub p: V3,
	pub q: Q4,
}

pub fn xform(t: &Pose, v: V3) -> V3 {
	qrot(t.q, v) + t.p
}

pub fn compose(a: &Pose, b: &Pose) -> Pose {
	Pose { p: xform(a, b.p), q: qnorm(qmul(a.q, b.q)) }
}

pub fn inverse(a: &Pose) -> Pose {
	let qi = qconj(a.q);
	Pose { p: qrot(qi, -a.p), q: qi }
}

/// Jacobi eigen decomposition of a symmetric 4x4 matrix (in place): the eigenvector of the largest eigenvalue
pub fn max_eigvec4(a: &mut [[f64; 4]; 4]) -> [f64; 4] {
	let mut v = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
	for _sweep in 0..60 {
		let mut off = 0.0;
		for p in 0..4 {
			for q in p + 1..4 {
				off += a[p][q] * a[p][q];
			}
		}
		if off < 1e-22 {
			break;
		}
		for p in 0..4 {
			for q in p + 1..4 {
				if a[p][q].abs() < 1e-30 {
					continue;
				}
				let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
				let t = (if theta >= 0.0 { 1.0 } else { -1.0 }) / (theta.abs() + (theta * theta + 1.0).sqrt());
				let c = 1.0 / (t * t + 1.0).sqrt();
				let s = t * c;
				for k in 0..4 {
					let (akp, akq) = (a[k][p], a[k][q]);
					a[k][p] = c * akp - s * akq;
					a[k][q] = s * akp + c * akq;
				}
				for k in 0..4 {
					let (apk, aqk) = (a[p][k], a[q][k]);
					a[p][k] = c * apk - s * aqk;
					a[q][k] = s * apk + c * aqk;
				}
				for k in 0..4 {
					let (vkp, vkq) = (v[k][p], v[k][q]);
					v[k][p] = c * vkp - s * vkq;
					v[k][q] = s * vkp + c * vkq;
				}
			}
		}
	}
	let mut best = 0;
	for i in 1..4 {
		if a[i][i] > a[best][best] {
			best = i;
		}
	}
	[v[0][best], v[1][best], v[2][best], v[3][best]]
}

/// Horn's closed-form absolute orientation: the rigid transform T with T(src_i) ~ dst_i (least squares), and the rms
/// of its residuals. None when there are too few points (under 4) or they are degenerate (all within about 1 cm)
pub fn fit_rigid(src: &[V3], dst: &[V3]) -> Option<(Pose, f64)> {
	let n = src.len() as i32;
	if n < 4 {
		return None;
	}
	let mut cs = V3::default();
	let mut cd = V3::default();
	for i in 0..n as usize {
		cs += src[i];
		cd += dst[i];
	}
	cs = cs * (1.0 / n as f64);
	cd = cd * (1.0 / n as f64);
	let mut s = [[0.0f64; 3]; 3];
	let mut spread = 0.0;
	for i in 0..n as usize {
		let a = src[i] - cs;
		let b = dst[i] - cd;
		spread += dot(a, a);
		let av = [a.x, a.y, a.z];
		let bv = [b.x, b.y, b.z];
		for r in 0..3 {
			for c in 0..3 {
				s[r][c] += av[r] * bv[c];
			}
		}
	}
	if spread / (n as f64) < 1e-4 {
		// all points within ~1 cm: no orientation information
		return None;
	}
	let (sxx, sxy, sxz, syx, syy, syz, szx, szy, szz) =
		(s[0][0], s[0][1], s[0][2], s[1][0], s[1][1], s[1][2], s[2][0], s[2][1], s[2][2]);
	let mut nm = [
		[sxx + syy + szz, syz - szy, szx - sxz, sxy - syx],
		[syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
		[szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
		[sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
	];
	let v = max_eigvec4(&mut nm);
	let q = qnorm(Q4::new(v[0], v[1], v[2], v[3]));
	let out = Pose { p: cd - qrot(q, cs), q };
	let mut e = 0.0;
	for i in 0..n as usize {
		let d = xform(&out, src[i]) - dst[i];
		e += dot(d, d);
	}
	Some((out, (e / n as f64).sqrt()))
}

/// fit_rigid with the outliers trimmed: refits on the pairs whose residual is under max(floor_m, the 70th
/// percentile), six rounds at most. The fit, the median residual over all pairs (outliers included) and the pairs in
/// the final fit; None when the first fit fails
pub fn fit_rigid_robust(src: &[V3], dst: &[V3], floor_m: f64) -> Option<(Pose, f64, i32)> {
	let n = src.len() as i32;
	let mut a: Vec<V3> = src.to_vec();
	let mut b: Vec<V3> = dst.to_vec();
	let mut res = vec![0.0f64; n.max(0) as usize];
	let (mut f, _) = fit_rigid(&a, &b)?;
	let mut kept = n;
	for _round in 0..6 {
		for i in 0..n as usize {
			res[i] = len(xform(&f, src[i]) - dst[i]);
		}
		let mut tmp = res.clone();
		let k = (n as f64 * 0.7) as usize;
		nth_element(&mut tmp, k);
		// std::max(floor_m, tmp[k]): (floor_m < tmp[k]) ? tmp[k] : floor_m
		let cut = if floor_m < tmp[k] { tmp[k] } else { floor_m };
		a.clear();
		b.clear();
		for i in 0..n as usize {
			if res[i] < cut {
				a.push(src[i]);
				b.push(dst[i]);
			}
		}
		kept = a.len() as i32;
		if kept < 4 {
			break;
		}
		match fit_rigid(&a, &b) {
			Some((g, _)) => f = g,
			None => break,
		}
	}
	for i in 0..n as usize {
		res[i] = len(xform(&f, src[i]) - dst[i]);
	}
	nth_element(&mut res, (n / 2) as usize);
	Some((f, res[(n / 2) as usize], kept))
}
