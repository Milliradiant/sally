//! The linear algebra sally computes with, ported from linmath.c operation for operation (docs/checking.md: the same
//! floating-point operations in the same order), the C library's fmax / fmin / pow / erfcf and strtod / strtof / atoi
//! where their exact behaviour shows in a result (`libc`), printf (`printf`) and sscanf with fgets (`scanf`); and the
//! vectors, quaternions and rigid fit of the driver and the panel (`math3d`), with the selection they pick a percentile
//! with (`nth_element`).
//!
//! Every expression keeps the C's evaluation order: `a * b * c` is `(a * b) * c`, C's integer literals are the same
//! doubles here (`2 * x` is `2.0 * x`), and nothing is simplified, even where the algebra allows it (no fused
//! multiply-add, no reassociation: Rust does neither on its own, and the C build has no FMA on its baseline x86-64).

pub mod libc;
pub mod math3d;
pub mod nth_element;
pub mod printf;
pub mod scanf;

pub use libc::{c_acos, c_atof, c_atoi, c_cos, c_erfcf, c_exp, c_fmax, c_fmin, c_pow, c_sin, c_strtod, c_strtof};

pub type Quat = [f64; 4];

/// A pose (LinmathPose, SurvivePose in the C): Pos[3], Rot[4] (w x y z)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose {
	pub pos: [f64; 3],
	pub rot: [f64; 4],
}

/// LINMATHPI and LINMATHPI_2 (linmath.h): `(FLT)3.14159265358979323846` and `(FLT)1.57079632679489661923`,
/// the C's literals as written (the same doubles as f64::consts::PI and FRAC_PI_2)
#[allow(clippy::approx_constant, clippy::excessive_precision)]
pub const LINMATHPI: f64 = 3.14159265358979323846;
#[allow(clippy::approx_constant, clippy::excessive_precision)]
pub const LINMATHPI_2: f64 = 1.57079632679489661923;

#[inline]
pub fn cross3d(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
	[a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

#[inline]
pub fn sub3d(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
	[a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub fn add3d(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
	[a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
pub fn scale3d(a: &[f64; 3], scalar: f64) -> [f64; 3] {
	[a[0] * scalar, a[1] * scalar, a[2] * scalar]
}

#[inline]
pub fn norm3d(a: &[f64; 3]) -> f64 {
	(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

#[inline]
pub fn normalize3d(a: &[f64; 3]) -> [f64; 3] {
	let r = 1.0 / norm3d(a);
	[a[0] * r, a[1] * r, a[2] * r]
}

#[inline]
pub fn dot3d(a: &[f64; 3], b: &[f64; 3]) -> f64 {
	a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// dist3d: magnitude3d of the difference
#[inline]
pub fn dist3d(a: &[f64; 3], b: &[f64; 3]) -> f64 {
	norm3d(&sub3d(a, b))
}

/// linmath.h's linmath_max: `x > y ? x : y` (not fmax: a NaN x gives y, a NaN y gives y)
#[inline]
pub fn linmath_max(x: f64, y: f64) -> f64 {
	if x > y {
		x
	} else {
		y
	}
}

/// linmath.h's linmath_min: `x < y ? x : y`
#[inline]
pub fn linmath_min(x: f64, y: f64) -> f64 {
	if x < y {
		x
	} else {
		y
	}
}

/// linmath_chauvenet_criterion: `erfcf((fabs(v - mu) / sigma) * M_SQRT1_2) * n`, with the C's types: the argument goes
/// to float for erfcf, and the product with the int n is a float product
#[inline]
pub fn linmath_chauvenet_criterion(v: f64, mu: f64, sigma: f64, n: i32) -> f64 {
	(c_erfcf((((v - mu).abs() / sigma) * std::f64::consts::FRAC_1_SQRT_2) as f32) * n as f32) as f64
}

#[inline]
pub fn quatiszero(q: &Quat) -> bool {
	q[0] == 0.0 && q[1] == 0.0 && q[2] == 0.0 && q[3] == 0.0
}

/// quatinvsqmagnitude, quatgetconjugate, quatscale
#[inline]
pub fn quatgetreciprocal(qin: &Quat) -> Quat {
	let m = 1.0 / ((qin[0] * qin[0]) + (qin[1] * qin[1]) + (qin[2] * qin[2]) + (qin[3] * qin[3])).sqrt();
	let qout = [qin[0], -qin[1], -qin[2], -qin[3]];
	[qout[0] * m, qout[1] * m, qout[2] * m, qout[3] * m]
}

#[inline]
pub fn quatmagnitude(q: &Quat) -> f64 {
	((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3])).sqrt()
}

/// quatnormalize: quatdivs by quatmagnitude
#[inline]
pub fn quatnormalize(q: &Quat) -> Quat {
	let norm = quatmagnitude(q);
	[q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
}

#[inline]
pub fn quatscale(q: &Quat, s: f64) -> Quat {
	[q[0] * s, q[1] * s, q[2] * s, q[3] * s]
}

#[inline]
pub fn quatadd(a: &Quat, b: &Quat) -> Quat {
	[a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]]
}

#[inline]
pub fn quatsub(a: &Quat, b: &Quat) -> Quat {
	[a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]]
}

/// quatslerp as linmath has it: the cosine between the two taken over their first three components only (dot3d on
/// the quaternions), a linear blend renormalized above 0.9995, the sines' blend below. (The C also normalizes copies
/// of both inputs, which it never reads.)
pub fn quatslerp(qa: &Quat, qb: &Quat, t: f64) -> Quat {
	let mut dot = dot3d(&[qa[0], qa[1], qa[2]], &[qb[0], qb[1], qb[2]]);
	let nqb = if dot < 0.0 {
		dot = -dot;
		quatscale(qb, -1.0)
	} else {
		quatscale(qb, 1.0)
	};
	const DOT_THRESHOLD: f64 = 0.9995;
	if dot > DOT_THRESHOLD {
		let tmp = quatscale(&quatsub(&nqb, qa), t);
		return quatnormalize(&quatadd(qa, &tmp));
	}
	let theta_0 = dot.acos();
	let theta = theta_0 * t;
	let sin_theta = theta.sin();
	let sin_theta_0 = theta_0.sin();
	let s0 = theta.cos() - dot * sin_theta / sin_theta_0;
	let s1 = sin_theta / sin_theta_0;
	let bside = quatscale(&nqb, s1);
	let aside = quatscale(qa, s0);
	quatnormalize(&quatadd(&aside, &bside))
}

/// PoseSlerp: the position blended linearly, the rotation by quatslerp
pub fn pose_slerp(start: &Pose, end: &Pose, t: f64) -> Pose {
	let delta = scale3d(&sub3d(&end.pos, &start.pos), t);
	Pose { pos: add3d(&delta, &start.pos), rot: quatslerp(&start.rot, &end.rot, t) }
}

/// q1 * q2 (does not normalize)
#[inline]
pub fn quatrotateabout(q1: &Quat, q2: &Quat) -> Quat {
	[
		(q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3]),
		(q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2]),
		(q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1]),
		(q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0]),
	]
}

/// rotate a vector by a quaternion (http://www.geeks3d.com/20141201/how-to-rotate-a-vertex-by-a-quaternion-in-glsl/)
#[inline]
pub fn quatrotatevector(quat: &Quat, vec3in: &[f64; 3]) -> [f64; 3] {
	let qv = [quat[1], quat[2], quat[3]];
	let mut tmp = cross3d(&qv, vec3in);
	tmp[0] += vec3in[0] * quat[0];
	tmp[1] += vec3in[1] * quat[0];
	tmp[2] += vec3in[2] * quat[0];
	let tmp2 = cross3d(&qv, &tmp);
	[vec3in[0] + 2.0 * tmp2[0], vec3in[1] + 2.0 * tmp2[1], vec3in[2] + 2.0 * tmp2[2]]
}

/// row-major 3x3 to quaternion (the C's `(m00 > m11) & (m00 > m22)` is a bitwise and of two comparisons)
#[inline]
pub fn quatfrommatrix33(m: &[f64; 9]) -> Quat {
	let (m00, m01, m02, m10, m11, m12, m20, m21, m22) = (m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	let tr = m00 + m11 + m22;
	if tr > 0.0 {
		let s = (tr + 1.0).sqrt() * 2.0; // S=4*qw
		[0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
	} else if (m00 > m11) & (m00 > m22) {
		let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0; // S=4*q[1]
		[(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
	} else if m11 > m22 {
		let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0; // S=4*q[2]
		[(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
	} else {
		let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0; // S=4*q[3]
		[(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
	}
}

/// quatrotatevector then add3d(pout, tmp, pose->Pos)
#[inline]
pub fn apply_pose_to_point(pose: &Pose, pin: &[f64; 3]) -> [f64; 3] {
	let tmp = quatrotatevector(&pose.rot, pin);
	add3d(&tmp, &pose.pos)
}

#[inline]
pub fn apply_pose_to_pose(lhs: &Pose, rhs: &Pose) -> Pose {
	Pose { pos: apply_pose_to_point(lhs, &rhs.pos), rot: quatrotateabout(&lhs.rot, &rhs.rot) }
}

/// InvertPose (InvertPoseRtn): the reciprocal rotation, the position rotated by it and scaled by -1
#[inline]
pub fn invert_pose(pose: &Pose) -> Pose {
	let rot = quatgetreciprocal(&pose.rot);
	let pos = quatrotatevector(&rot, &pose.pos);
	Pose { pos: scale3d(&pos, -1.0), rot }
}

/// linmath.h's linmath_enforce_range (a NaN stays NaN)
#[inline]
pub fn linmath_enforce_range(v: f64, mn: f64, mx: f64) -> f64 {
	if v < mn {
		return mn;
	}
	if v > mx {
		return mx;
	}
	v
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn fmax_fmin_ties_and_nans() {
		#[cfg(all(target_arch = "x86_64", target_os = "linux", target_env = "gnu"))]
		{
			let (p, n) = (0.0f64, -0.0f64);
			// glibc 2.39 measured: fmax(+0,-0) = -0, fmax(-0,+0) = +0, fmin likewise the second
			assert!(c_fmax(p, n).is_sign_negative() && c_fmax(n, p).is_sign_positive());
			assert!(c_fmin(p, n).is_sign_negative() && c_fmin(n, p).is_sign_positive());
		}
		assert_eq!(c_fmax(f64::NAN, 1.0), 1.0);
		assert_eq!(c_fmax(1.0, f64::NAN), 1.0);
		assert_eq!(c_fmin(f64::NAN, 1.0), 1.0);
		assert_eq!(c_fmin(1.0, f64::NAN), 1.0);
		assert_eq!(c_fmax(2.0, 1.0), 2.0);
		assert_eq!(c_fmin(2.0, 1.0), 1.0);
		assert!(c_fmax(f64::NAN, f64::NAN).is_nan());
	}

	#[test]
	fn quaternions() {
		let q: Quat = [0.9, 0.1, -0.3, 0.2];
		let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
		let q = [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
		let v = [0.3, -1.2, 2.5];
		let r = quatrotatevector(&q, &v);
		let back = quatrotatevector(&quatgetreciprocal(&q), &r);
		for i in 0..3 {
			assert!((back[i] - v[i]).abs() < 1e-12);
		}
		let pose = Pose { pos: [1.0, 2.0, 3.0], rot: q };
		let id = apply_pose_to_pose(&pose, &invert_pose(&pose));
		for i in 0..3 {
			assert!(id.pos[i].abs() < 1e-12);
		}
		assert!((id.rot[0] - 1.0).abs() < 1e-12);
		// the rotation matrix of q back to q
		let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
		let m = [
			1.0 - 2.0 * (y * y + z * z),
			2.0 * (x * y - w * z),
			2.0 * (x * z + w * y),
			2.0 * (x * y + w * z),
			1.0 - 2.0 * (x * x + z * z),
			2.0 * (y * z - w * x),
			2.0 * (x * z - w * y),
			2.0 * (y * z + w * x),
			1.0 - 2.0 * (x * x + y * y),
		];
		let qm = quatfrommatrix33(&m);
		for i in 0..4 {
			assert!((qm[i] - q[i]).abs() < 1e-12);
		}
	}
}
