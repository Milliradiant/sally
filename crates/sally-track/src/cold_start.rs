//! The tracker's cold start: a device's pose from the hits of one station alone, with no prior, as the first guess of
//! the seed solve. libsurvive's barycentric SVD (EPnP adapted to the lighthouse's planes): every sensor is written as
//! weights of four control points, every hit gives one linear equation in the control points' 12 coordinates in the
//! station's frame, the null space of those equations (the four smallest singular vectors of M^T M) is combined with
//! the weights (betas) that keep the control points' mutual distances, and the pose that carries the sensors onto the
//! result is the answer. Three ways of finding the betas are tried; the one that fits the hits best wins.
//!
//! Ported from libsurvive's barycentric_svd.c with the linear algebra it takes from Eigen written here (Jacobi
//! rotations, least squares through the SVD): the same algorithm and the same pose to rounding, not the same bits
//! (Eigen's sums depend on the memory alignment of the data they run over).

use sally_lighthouse::bc_fill_m;

/// one hit: the sensor, the plane (0 or 1), the angle with the station's phase applied
#[derive(Clone, Copy, Debug)]
struct Meas {
	obj: usize,
	axis: i32,
	angle: f64,
}

/// The solver for one device: its sensor layout, written as weights of the control points once, and the hits of the
/// current attempt
pub struct BarycentricSvd {
	obj_pts: Vec<[f64; 3]>,
	alphas: Vec<[f64; 4]>,
	control_points: [[f64; 3]; 4],
	meas: Vec<Meas>,
	object_pts_in_camera: Vec<[f64; 3]>,
	control_points_in_camera: [[f64; 3]; 4],
}

fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
	a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn dist2(a: &[f64; 3], b: &[f64; 3]) -> f64 {
	(a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2])
}

impl BarycentricSvd {
	/// bc_svd_bc_svd: the control points (the sensors' centroid and three points around it) and every sensor's
	/// weights of them
	pub fn new(obj_pts: Vec<[f64; 3]>) -> Self {
		let n = obj_pts.len();
		let mut s = BarycentricSvd {
			obj_pts,
			alphas: vec![[0.0; 4]; n],
			control_points: [[0.0; 3]; 4],
			meas: Vec::new(),
			object_pts_in_camera: vec![[0.0; 3]; n],
			control_points_in_camera: [[0.0; 3]; 4],
		};
		s.choose_control_points();
		s.compute_barycentric_coordinates();
		s
	}

	/// C0 is the sensors' centroid; C1..C3 are C0 plus the rows of a fixed rotation, scaled by 1/sqrt(n). The original
	/// EPnP takes the principal axes of the points, which fails on a nearly planar layout (the headset) with spotty
	/// data; a fixed rotation around the centroid is robust (libsurvive's choice)
	fn choose_control_points(&mut self) {
		let n = self.obj_pts.len();
		let mut c0 = [0.0; 3];
		for p in &self.obj_pts {
			for j in 0..3 {
				c0[j] += p[j];
			}
		}
		for v in c0.iter_mut() {
			*v /= n as f64;
		}
		self.control_points[0] = c0;
		// quattomatrix33 of the normalised (1, 1, 1, 1): a third of a turn about (1, 1, 1)
		let r = quattomatrix33(&quatnormalize(&[1.0, 1.0, 1.0, 1.0]));
		let k = (1.0 / n as f64).sqrt();
		for i in 1..4 {
			for j in 0..3 {
				self.control_points[i][j] = c0[j] + k * r[3 * (i - 1) + j];
			}
		}
	}

	/// every sensor as the weights of the four control points that give it (they sum to 1)
	fn compute_barycentric_coordinates(&mut self) {
		let mut cc = [0.0; 9];
		for i in 0..3 {
			for j in 1..4 {
				cc[i * 3 + j - 1] = self.control_points[j][i] - self.control_points[0][i];
			}
		}
		let ci = pseudo_inverse(&cc, 3, 3);
		let c0 = self.control_points[0];
		for (p, a) in self.obj_pts.iter().zip(self.alphas.iter_mut()) {
			for j in 0..3 {
				a[1 + j] = ci[j * 3] * (p[0] - c0[0]) + ci[j * 3 + 1] * (p[1] - c0[1]) + ci[j * 3 + 2] * (p[2] - c0[2]);
			}
			a[0] = 1.0 - a[1] - a[2] - a[3];
		}
	}

	/// a new attempt: no hits
	pub fn reset(&mut self) {
		self.meas.clear();
	}

	/// One hit: sensor `obj`, plane `axis`, `angle` with the station's phase applied. A NaN angle is not taken, and
	/// neither is a sensor the layout does not have
	pub fn add(&mut self, obj: usize, axis: i32, angle: f64) {
		if angle.is_nan() || obj >= self.obj_pts.len() || !(axis == 0 || axis == 1) {
			return;
		}
		self.meas.push(Meas { obj, axis, angle });
	}

	pub fn meas_cnt(&self) -> usize {
		self.meas.len()
	}

	/// bc_svd_compute_pose: the pose (Xc = R pw + t, the station's frame from the device's) and its reprojection
	/// error, or -1 when the hits cannot give one (one plane only, or a control point coordinate no hit touches)
	pub fn compute_pose(&mut self, lh_version: i32, r_out: &mut [[f64; 3]; 3], t_out: &mut [f64; 3]) -> f64 {
		let n = self.meas.len();
		let mut m = vec![0.0; n * 12];
		let mut col_covered = [false; 12];
		let mut has_axis = [false; 2];
		for (i, me) in self.meas.iter().enumerate() {
			let eq = bc_fill_m(lh_version, me.axis, me.angle);
			let a = &self.alphas[me.obj];
			for c in 0..4 {
				for j in 0..3 {
					let v = eq[j] * a[c];
					m[i * 12 + c * 3 + j] = v;
					if v != 0.0 {
						col_covered[c * 3 + j] = true;
					}
				}
			}
			has_axis[me.axis as usize] = true;
		}
		// gen2 can technically solve with one plane only, but very noisily
		if !has_axis[0] || !has_axis[1] || col_covered.iter().any(|&c| !c) {
			return -1.0;
		}

		// M^T M and its singular vectors: the null space of the hit equations is spanned by the last four
		let mut mtm = vec![0.0; 144];
		for a in 0..12 {
			for b in 0..12 {
				let mut s = 0.0;
				for i in 0..n {
					s += m[i * 12 + a] * m[i * 12 + b];
				}
				mtm[a * 12 + b] = s;
			}
		}
		let (_, u) = symmetric_eigen(&mut mtm, 12);
		// ut(i, k): the i-th singular vector's k-th coordinate (U transposed, singular values decreasing)
		let ut = |i: usize, k: usize| u[k * 12 + i];

		let l = compute_l_6x10(&ut);
		let rho = self.compute_rho();

		let mut betas = [[0.0; 4]; 4];
		let mut rs = [[[0.0; 3]; 3]; 4];
		let mut ts = [[0.0; 3]; 4];
		let mut rep_errors = [0.0; 3];

		find_betas_approx_1(&l, &rho, &mut betas[1]);
		gauss_newton(&l, &rho, &mut betas[1]);
		rep_errors[0] = self.compute_r_and_t(lh_version, &ut, &betas[1], &mut rs[1], &mut ts[1]);

		find_betas_approx_2(&l, &rho, &mut betas[2]);
		gauss_newton(&l, &rho, &mut betas[2]);
		rep_errors[1] = self.compute_r_and_t(lh_version, &ut, &betas[2], &mut rs[2], &mut ts[2]);

		find_betas_approx_3(&l, &rho, &mut betas[3]);
		gauss_newton(&l, &rho, &mut betas[3]);
		rep_errors[2] = self.compute_r_and_t(lh_version, &ut, &betas[3], &mut rs[3], &mut ts[3]);

		let mut best = 0;
		if rep_errors[1] < rep_errors[0] {
			best = 1;
		}
		if rep_errors[2] < rep_errors[best] {
			best = 2;
		}
		*r_out = rs[best + 1];
		*t_out = ts[best + 1];
		rep_errors[best]
	}

	/// the squared distances between the control points (the six pairs in order 01 02 03 12 13 23)
	fn compute_rho(&self) -> [f64; 6] {
		let c = &self.control_points;
		[
			dist2(&c[0], &c[1]),
			dist2(&c[0], &c[2]),
			dist2(&c[0], &c[3]),
			dist2(&c[1], &c[2]),
			dist2(&c[1], &c[3]),
			dist2(&c[2], &c[3]),
		]
	}

	fn compute_r_and_t(
		&mut self, lh_version: i32, ut: &impl Fn(usize, usize) -> f64, betas: &[f64; 4], r: &mut [[f64; 3]; 3],
		t: &mut [f64; 3],
	) -> f64 {
		self.compute_ccs(betas, ut);
		self.compute_pcs();
		self.solve_for_sign();
		self.estimate_r_and_t(r, t);
		self.reprojection_error(lh_version, r, t)
	}

	/// the control points in the station's frame: the null space vectors weighted by the betas
	fn compute_ccs(&mut self, betas: &[f64; 4], ut: &impl Fn(usize, usize) -> f64) {
		self.control_points_in_camera = [[0.0; 3]; 4];
		for i in 0..4 {
			for j in 0..4 {
				for k in 0..3 {
					self.control_points_in_camera[j][k] += betas[i] * ut(11 - i, 3 * j + k);
				}
			}
		}
	}

	/// the sensors in the station's frame, from their weights
	fn compute_pcs(&mut self) {
		let cc = &self.control_points_in_camera;
		for (a, pc) in self.alphas.iter().zip(self.object_pts_in_camera.iter_mut()) {
			for j in 0..3 {
				pc[j] = a[0] * cc[0][j] + a[1] * cc[1][j] + a[2] * cc[2][j] + a[3] * cc[3][j];
			}
		}
	}

	/// the null space has no sign: the device is in front of the station (z > 0 in the solver's frame)
	fn solve_for_sign(&mut self) {
		if self.object_pts_in_camera.first().is_some_and(|p| p[2] < 0.0) {
			for c in self.control_points_in_camera.iter_mut() {
				for v in c.iter_mut() {
					*v = -*v;
				}
			}
			for p in self.object_pts_in_camera.iter_mut() {
				for v in p.iter_mut() {
					*v = -*v;
				}
			}
		}
	}

	/// the rigid motion that carries the layout onto its points in the station's frame (Procrustes through the SVD of
	/// the centred cross-covariance, the reflection taken out)
	fn estimate_r_and_t(&self, r: &mut [[f64; 3]; 3], t: &mut [f64; 3]) {
		let n = self.obj_pts.len() as f64;
		let mut pc0 = [0.0; 3];
		let mut pw0 = [0.0; 3];
		for (pc, pw) in self.object_pts_in_camera.iter().zip(&self.obj_pts) {
			for j in 0..3 {
				pc0[j] += pc[j];
				pw0[j] += pw[j];
			}
		}
		for j in 0..3 {
			pc0[j] /= n;
			pw0[j] /= n;
		}
		let mut abt = [0.0; 9];
		for (pc, pw) in self.object_pts_in_camera.iter().zip(&self.obj_pts) {
			for j in 0..3 {
				abt[3 * j] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
				abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
				abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
			}
		}
		let (u, _, v) = svd(&abt, 3, 3);
		for i in 0..3 {
			for j in 0..3 {
				// R = U V^T
				r[i][j] = u[i * 3] * v[j * 3] + u[i * 3 + 1] * v[j * 3 + 1] + u[i * 3 + 2] * v[j * 3 + 2];
			}
		}
		let det = r[0][0] * r[1][1] * r[2][2] + r[0][1] * r[1][2] * r[2][0] + r[0][2] * r[1][0] * r[2][1]
			- r[0][2] * r[1][1] * r[2][0]
			- r[0][1] * r[1][0] * r[2][2]
			- r[0][0] * r[1][2] * r[2][1];
		if det < 0.0 {
			for v in r[2].iter_mut() {
				*v = -*v;
			}
		}
		for i in 0..3 {
			t[i] = pc0[i] - dot(&r[i], &pw0);
		}
	}

	/// the hits' plane equations evaluated at the pose: sqrt(sum of squares) / number of hits
	fn reprojection_error(&self, lh_version: i32, r: &[[f64; 3]; 3], t: &[f64; 3]) -> f64 {
		let mut sum2 = 0.0;
		for me in &self.meas {
			let pw = &self.obj_pts[me.obj];
			let xc = [dot(&r[0], pw) + t[0], dot(&r[1], pw) + t[1], dot(&r[2], pw) + t[2]];
			let eq = bc_fill_m(lh_version, me.axis, me.angle);
			let rerr = eq[0] * xc[0] + eq[1] * xc[1] + eq[2] * xc[2];
			sum2 += rerr * rerr;
		}
		sum2.sqrt() / self.meas.len() as f64
	}
}

/// the 6 x 10 matrix whose product with the betas' products (B11 B12 B22 B13 B23 B33 B14 B24 B34 B44) gives the
/// control points' squared distances, from the null space vectors
fn compute_l_6x10(ut: &impl Fn(usize, usize) -> f64) -> [[f64; 10]; 6] {
	let mut dv = [[[0.0; 3]; 6]; 4];
	for (i, dvi) in dv.iter_mut().enumerate() {
		let (mut a, mut b) = (0, 1);
		for d in dvi.iter_mut() {
			for h in 0..3 {
				d[h] = ut(11 - i, 3 * a + h) - ut(11 - i, 3 * b + h);
			}
			b += 1;
			if b > 3 {
				a += 1;
				b = a + 1;
			}
		}
	}
	let mut l = [[0.0; 10]; 6];
	for (i, row) in l.iter_mut().enumerate() {
		*row = [
			dot(&dv[0][i], &dv[0][i]),
			2.0 * dot(&dv[0][i], &dv[1][i]),
			dot(&dv[1][i], &dv[1][i]),
			2.0 * dot(&dv[0][i], &dv[2][i]),
			2.0 * dot(&dv[1][i], &dv[2][i]),
			dot(&dv[2][i], &dv[2][i]),
			2.0 * dot(&dv[0][i], &dv[3][i]),
			2.0 * dot(&dv[1][i], &dv[3][i]),
			2.0 * dot(&dv[2][i], &dv[3][i]),
			dot(&dv[3][i], &dv[3][i]),
		];
	}
	l
}

/// the 6 x k matrix of the given columns of L
fn columns(l: &[[f64; 10]; 6], cols: &[usize]) -> Vec<f64> {
	let mut out = Vec::with_capacity(6 * cols.len());
	for row in l {
		for &c in cols {
			out.push(row[c]);
		}
	}
	out
}

/// betas_approx_1 = [B11 B12 B13 B14]
fn find_betas_approx_1(l: &[[f64; 10]; 6], rho: &[f64; 6], betas: &mut [f64; 4]) {
	let b4 = least_squares(&columns(l, &[0, 1, 3, 6]), 6, 4, rho);
	if b4[0] < 0.0 {
		betas[0] = (-b4[0]).sqrt();
		betas[1] = -b4[1] / betas[0];
		betas[2] = -b4[2] / betas[0];
		betas[3] = -b4[3] / betas[0];
	} else {
		betas[0] = b4[0].sqrt();
		betas[1] = b4[1] / betas[0];
		betas[2] = b4[2] / betas[0];
		betas[3] = b4[3] / betas[0];
	}
}

/// betas_approx_2 = [B11 B12 B22]
fn find_betas_approx_2(l: &[[f64; 10]; 6], rho: &[f64; 6], betas: &mut [f64; 4]) {
	let b3 = least_squares(&columns(l, &[0, 1, 2]), 6, 3, rho);
	if b3[0] < 0.0 {
		betas[0] = (-b3[0]).sqrt();
		betas[1] = if b3[2] < 0.0 { (-b3[2]).sqrt() } else { 0.0 };
	} else {
		betas[0] = b3[0].sqrt();
		betas[1] = if b3[2] > 0.0 { b3[2].sqrt() } else { 0.0 };
	}
	if b3[1] < 0.0 {
		betas[0] = -betas[0];
	}
	betas[2] = 0.0;
	betas[3] = 0.0;
}

/// betas_approx_3 = [B11 B12 B22 B13 B23]
fn find_betas_approx_3(l: &[[f64; 10]; 6], rho: &[f64; 6], betas: &mut [f64; 4]) {
	let b5 = least_squares(&columns(l, &[0, 1, 2, 3, 4]), 6, 5, rho);
	if b5[0] < 0.0 {
		betas[0] = (-b5[0]).sqrt();
		betas[1] = if b5[2] < 0.0 { (-b5[2]).sqrt() } else { 0.0 };
	} else {
		betas[0] = b5[0].sqrt();
		betas[1] = if b5[2] > 0.0 { b5[2].sqrt() } else { 0.0 };
	}
	if b5[1] < 0.0 {
		betas[0] = -betas[0];
	}
	betas[2] = b5[3] / betas[0];
	betas[3] = 0.0;
}

/// five Gauss-Newton steps on the betas against the squared distances
fn gauss_newton(l: &[[f64; 10]; 6], rho: &[f64; 6], betas: &mut [f64; 4]) {
	for _ in 0..5 {
		let mut a = vec![0.0; 24];
		let mut b = [0.0; 6];
		for i in 0..6 {
			let li = &l[i];
			a[i * 4] = 2.0 * li[0] * betas[0] + li[1] * betas[1] + li[3] * betas[2] + li[6] * betas[3];
			a[i * 4 + 1] = li[1] * betas[0] + 2.0 * li[2] * betas[1] + li[4] * betas[2] + li[7] * betas[3];
			a[i * 4 + 2] = li[3] * betas[0] + li[4] * betas[1] + 2.0 * li[5] * betas[2] + li[8] * betas[3];
			a[i * 4 + 3] = li[6] * betas[0] + li[7] * betas[1] + li[8] * betas[2] + 2.0 * li[9] * betas[3];
			b[i] = rho[i]
				- (li[0] * betas[0] * betas[0]
					+ li[1] * betas[0] * betas[1]
					+ li[2] * betas[1] * betas[1]
					+ li[3] * betas[0] * betas[2]
					+ li[4] * betas[1] * betas[2]
					+ li[5] * betas[2] * betas[2]
					+ li[6] * betas[0] * betas[3]
					+ li[7] * betas[1] * betas[3]
					+ li[8] * betas[2] * betas[3]
					+ li[9] * betas[3] * betas[3]);
		}
		let x = least_squares(&a, 6, 4, &b);
		for i in 0..4 {
			betas[i] += x[i];
		}
	}
}

// ---- the linear algebra (Eigen's, in libsurvive)

/// linmath's quatnormalize
fn quatnormalize(q: &[f64; 4]) -> [f64; 4] {
	let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
	[q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// linmath's quattomatrix33 (row-major; it normalises its argument again)
fn quattomatrix33(qin: &[f64; 4]) -> [f64; 9] {
	let q = quatnormalize(qin);
	let xx = 2.0 * q[1] * q[1];
	let xy = 2.0 * q[1] * q[2];
	let xz = 2.0 * q[1] * q[3];
	let xw = 2.0 * q[1] * q[0];
	let yy = 2.0 * q[2] * q[2];
	let yz = 2.0 * q[2] * q[3];
	let yw = 2.0 * q[2] * q[0];
	let zz = 2.0 * q[3] * q[3];
	let zw = 2.0 * q[3] * q[0];
	[1.0 - yy - zz, xy - zw, xz + yw, xy + zw, 1.0 - xx - zz, yz - xw, xz - yw, yz + xw, 1.0 - xx - yy]
}

/// a rotation (c, s) that zeroes the off-diagonal of the 2 x 2 symmetric [[app, apq], [apq, aqq]]: t = tan(angle) is
/// the smaller root of t^2 + 2 theta t - 1 = 0, theta = (aqq - app) / (2 apq)
fn jacobi_rotation(app: f64, aqq: f64, apq: f64) -> (f64, f64) {
	let theta = (aqq - app) / (2.0 * apq);
	let t = if theta.is_infinite() { 0.0 } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
	let c = 1.0 / (t * t + 1.0).sqrt();
	(c, t * c)
}

const JACOBI_SWEEPS: usize = 64;

/// The eigen-decomposition of a symmetric n x n matrix (row-major, overwritten) by cyclic Jacobi rotations: the
/// eigenvalues ordered by decreasing magnitude, and the eigenvectors as the columns of the returned n x n matrix, in
/// the same order. For M^T M (positive semi-definite) these are its singular values and vectors (libsurvive uses
/// Eigen's JacobiSVD here); the vectors of the smallest values are accurate even where the values are all but zero.
fn symmetric_eigen(a: &mut [f64], n: usize) -> (Vec<f64>, Vec<f64>) {
	let mut v = vec![0.0; n * n];
	for i in 0..n {
		v[i * n + i] = 1.0;
	}
	let scale = a.iter().fold(0.0f64, |m, x| m.max(x.abs()));
	if scale > 0.0 {
		for _ in 0..JACOBI_SWEEPS {
			let mut rotated = false;
			for p in 0..n {
				for q in p + 1..n {
					let apq = a[p * n + q];
					// below this the element is rounding noise of the matrix's size: rotating it away changes nothing
					if apq.abs() <= f64::EPSILON * f64::EPSILON * scale {
						continue;
					}
					let (c, s) = jacobi_rotation(a[p * n + p], a[q * n + q], apq);
					if s == 0.0 {
						continue;
					}
					rotated = true;
					for k in 0..n {
						let (akp, akq) = (a[k * n + p], a[k * n + q]);
						a[k * n + p] = c * akp - s * akq;
						a[k * n + q] = s * akp + c * akq;
					}
					for k in 0..n {
						let (apk, aqk) = (a[p * n + k], a[q * n + k]);
						a[p * n + k] = c * apk - s * aqk;
						a[q * n + k] = s * apk + c * aqk;
					}
					a[p * n + q] = 0.0;
					a[q * n + p] = 0.0;
					for k in 0..n {
						let (vkp, vkq) = (v[k * n + p], v[k * n + q]);
						v[k * n + p] = c * vkp - s * vkq;
						v[k * n + q] = s * vkp + c * vkq;
					}
				}
			}
			if !rotated {
				break;
			}
		}
	}
	let mut order: Vec<usize> = (0..n).collect();
	order.sort_by(|&i, &j| a[j * n + j].abs().total_cmp(&a[i * n + i].abs()));
	let values = order.iter().map(|&i| a[i * n + i]).collect();
	let mut vectors = vec![0.0; n * n];
	for (col, &i) in order.iter().enumerate() {
		for k in 0..n {
			vectors[k * n + col] = v[k * n + i];
		}
	}
	(values, vectors)
}

/// The SVD of an m x n matrix (row-major, m >= n) by one-sided Jacobi rotations: a = u diag(s) v^T, u m x n with
/// orthonormal columns (a column of a zero singular value completed to an orthonormal set), s decreasing, v n x n
/// orthogonal. Returns (u, s, v).
fn svd(a: &[f64], m: usize, n: usize) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
	assert!(m >= n && a.len() == m * n);
	let mut u = a.to_vec();
	let mut v = vec![0.0; n * n];
	for i in 0..n {
		v[i * n + i] = 1.0;
	}
	for _ in 0..JACOBI_SWEEPS {
		let mut rotated = false;
		for p in 0..n {
			for q in p + 1..n {
				let (mut alpha, mut beta, mut gamma) = (0.0, 0.0, 0.0);
				for i in 0..m {
					alpha += u[i * n + p] * u[i * n + p];
					beta += u[i * n + q] * u[i * n + q];
					gamma += u[i * n + p] * u[i * n + q];
				}
				// the columns are orthogonal to working precision: nothing to rotate
				if gamma == 0.0 || gamma.abs() <= f64::EPSILON * (alpha * beta).sqrt() {
					continue;
				}
				let (c, s) = jacobi_rotation(alpha, beta, gamma);
				if s == 0.0 {
					continue;
				}
				rotated = true;
				for i in 0..m {
					let (up, uq) = (u[i * n + p], u[i * n + q]);
					u[i * n + p] = c * up - s * uq;
					u[i * n + q] = s * up + c * uq;
				}
				for k in 0..n {
					let (vp, vq) = (v[k * n + p], v[k * n + q]);
					v[k * n + p] = c * vp - s * vq;
					v[k * n + q] = s * vp + c * vq;
				}
			}
		}
		if !rotated {
			break;
		}
	}
	let mut sv: Vec<f64> = (0..n).map(|j| (0..m).map(|i| u[i * n + j] * u[i * n + j]).sum::<f64>().sqrt()).collect();
	let mut order: Vec<usize> = (0..n).collect();
	order.sort_by(|&i, &j| sv[j].total_cmp(&sv[i]));
	let mut us = vec![0.0; m * n];
	let mut vs = vec![0.0; n * n];
	for (col, &j) in order.iter().enumerate() {
		for i in 0..m {
			us[i * n + col] = if sv[j] > 0.0 { u[i * n + j] / sv[j] } else { 0.0 };
		}
		for k in 0..n {
			vs[k * n + col] = v[k * n + j];
		}
	}
	sv = order.iter().map(|&j| sv[j]).collect();
	// complete u where a singular value is zero (Eigen's full U is orthonormal): the first unit vector with something
	// left after the other columns are taken out
	for col in 0..n {
		if sv[col] > 0.0 {
			continue;
		}
		for e in 0..m {
			let mut w = vec![0.0; m];
			w[e] = 1.0;
			for other in 0..n {
				if other == col || (sv[other] == 0.0 && other > col) {
					continue;
				}
				let d: f64 = (0..m).map(|i| w[i] * us[i * n + other]).sum();
				for i in 0..m {
					w[i] -= d * us[i * n + other];
				}
			}
			let norm = w.iter().map(|x| x * x).sum::<f64>().sqrt();
			if norm > 0.5 {
				for i in 0..m {
					us[i * n + col] = w[i] / norm;
				}
				break;
			}
		}
	}
	(us, sv, vs)
}

/// The least squares (minimum norm) solution of a x = b through the SVD, singular values below n * epsilon times the
/// largest taken as zero (what Eigen's JacobiSVD solve() gives, and its QR solve for a full-rank system)
fn least_squares(a: &[f64], m: usize, n: usize, b: &[f64]) -> Vec<f64> {
	let (u, s, v) = svd(a, m, n);
	let threshold = (s[0] * n as f64 * f64::EPSILON).max(f64::MIN_POSITIVE);
	let mut x = vec![0.0; n];
	for j in 0..n {
		if !(s[j] >= threshold) {
			break;
		}
		let ub: f64 = (0..m).map(|i| u[i * n + j] * b[i]).sum();
		let w = ub / s[j];
		for k in 0..n {
			x[k] += v[k * n + j] * w;
		}
	}
	x
}

/// the pseudo-inverse of an m x n matrix (m >= n) through the SVD, the same threshold as least_squares
fn pseudo_inverse(a: &[f64], m: usize, n: usize) -> Vec<f64> {
	let (u, s, v) = svd(a, m, n);
	let threshold = (s[0] * n as f64 * f64::EPSILON).max(f64::MIN_POSITIVE);
	let mut p = vec![0.0; n * m];
	for j in 0..n {
		if !(s[j] >= threshold) {
			break;
		}
		for r in 0..n {
			for c in 0..m {
				p[r * m + c] += v[r * n + j] * u[c * n + j] / s[j];
			}
		}
	}
	p
}

#[cfg(test)]
mod tests {
	use super::*;

	fn mul(a: &[f64], b: &[f64], n: usize, k: usize, m: usize) -> Vec<f64> {
		let mut c = vec![0.0; n * m];
		for i in 0..n {
			for j in 0..m {
				c[i * m + j] = (0..k).map(|l| a[i * k + l] * b[l * m + j]).sum();
			}
		}
		c
	}

	fn close(a: &[f64], b: &[f64], tol: f64) -> bool {
		a.iter().zip(b).all(|(x, y)| (x - y).abs() <= tol)
	}

	/// a fixed, scrambled matrix: no randomness in the tests
	fn sample(m: usize, n: usize, seed: u64) -> Vec<f64> {
		let mut x = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		(0..m * n)
			.map(|_| {
				x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
				((x >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
			})
			.collect()
	}

	#[test]
	fn svd_rebuilds_the_matrix() {
		for (m, n, seed) in [(3, 3, 1), (6, 4, 2), (6, 5, 3), (6, 3, 4), (12, 12, 5)] {
			let a = sample(m, n, seed);
			let (u, s, v) = svd(&a, m, n);
			assert!(s.windows(2).all(|w| w[0] >= w[1]), "decreasing: {s:?}");
			let mut us = u.clone();
			for i in 0..m {
				for j in 0..n {
					us[i * n + j] *= s[j];
				}
			}
			let vt: Vec<f64> = (0..n * n).map(|k| v[(k % n) * n + k / n]).collect();
			assert!(close(&mul(&us, &vt, m, n, n), &a, 1e-12), "{m}x{n}");
			let utu = mul(&(0..n * m).map(|k| u[(k % m) * n + k / m]).collect::<Vec<_>>(), &u, n, m, n);
			let id: Vec<f64> = (0..n * n).map(|k| if k % (n + 1) == 0 { 1.0 } else { 0.0 }).collect();
			assert!(close(&utu, &id, 1e-12), "u orthonormal {m}x{n}");
		}
	}

	#[test]
	fn svd_completes_u_for_a_singular_matrix() {
		// rank 2: the third column is the sum of the first two
		let a = [1.0, 2.0, 3.0, 4.0, 5.0, 9.0, 7.0, 8.0, 15.0];
		let (u, s, _) = svd(&a, 3, 3);
		assert!(s[2] < 1e-12);
		let utu = mul(&(0..9).map(|k| u[(k % 3) * 3 + k / 3]).collect::<Vec<_>>(), &u, 3, 3, 3);
		assert!(close(&utu, &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], 1e-9), "{utu:?}");
	}

	#[test]
	fn symmetric_eigen_of_a_gram_matrix() {
		// M^T M of a 20 x 12 M of rank 8: four eigenvalues all but zero, and their vectors span M's null space
		let mut m = sample(20, 12, 7);
		for i in 0..20 {
			for k in 8..12 {
				m[i * 12 + k] = m[i * 12 + k - 8] + 0.5 * m[i * 12 + k - 4];
			}
		}
		let mut mtm = mul(&(0..240).map(|k| m[(k % 20) * 12 + k / 20]).collect::<Vec<_>>(), &m, 12, 20, 12);
		let a0 = mtm.clone();
		let (vals, vecs) = symmetric_eigen(&mut mtm, 12);
		assert!(vals.windows(2).all(|w| w[0].abs() >= w[1].abs()));
		assert!(vals[8..].iter().all(|v| v.abs() < 1e-10), "{vals:?}");
		for col in 8..12 {
			let x: Vec<f64> = (0..12).map(|k| vecs[k * 12 + col]).collect();
			let mx = mul(&m, &x, 20, 12, 1);
			assert!(mx.iter().all(|v| v.abs() < 1e-10), "null space vector {col}");
		}
		for col in 0..12 {
			let x: Vec<f64> = (0..12).map(|k| vecs[k * 12 + col]).collect();
			let ax = mul(&a0, &x, 12, 12, 1);
			assert!(close(&ax, &x.iter().map(|v| v * vals[col]).collect::<Vec<_>>(), 1e-9), "eigenpair {col}");
		}
	}

	#[test]
	fn least_squares_of_an_overdetermined_system() {
		let a = sample(6, 4, 9);
		let x0 = [0.3, -1.2, 2.0, 0.7];
		let b = mul(&a, &x0, 6, 4, 1);
		assert!(close(&least_squares(&a, 6, 4, &b), &x0, 1e-12));
	}

	/// the angle at which the sensor at xc (the station's frame, the solver's convention) is crossed by plane `axis`:
	/// the root of bc_fill_m(1, axis, a) . xc = 0 nearest atan2(x, z)
	fn lh2_angle(xc: &[f64; 3], axis: i32) -> f64 {
		let mut a = xc[0].atan2(xc[2]);
		for _ in 0..50 {
			let eq = bc_fill_m(1, axis, a);
			let f = eq[0] * xc[0] + eq[1] * xc[1] + eq[2] * xc[2];
			let df = -a.sin() * xc[0] - a.cos() * xc[2];
			a -= f / df;
		}
		a
	}

	#[test]
	fn recovers_a_pose_from_exact_hits() {
		// a tracker-like layout (16 sensors on a 5 cm cap), 2 m in front of the station, turned
		let mut pts = Vec::new();
		for i in 0..16 {
			let th = i as f64 * 2.399963;
			let z = 1.0 - (i as f64 + 0.5) / 12.0;
			let r = (1.0 - z * z).max(0.0).sqrt();
			pts.push([r * th.cos() * 0.05, r * th.sin() * 0.05, z * 0.05]);
		}
		let r_true = quattomatrix33(&[0.9, 0.2, -0.3, 0.25]);
		let r_true =
			[[r_true[0], r_true[1], r_true[2]], [r_true[3], r_true[4], r_true[5]], [r_true[6], r_true[7], r_true[8]]];
		let t_true = [0.3, -0.2, 2.0];
		let mut bc = BarycentricSvd::new(pts.clone());
		for (i, p) in pts.iter().enumerate() {
			let xc = [dot(&r_true[0], p) + t_true[0], dot(&r_true[1], p) + t_true[1], dot(&r_true[2], p) + t_true[2]];
			for axis in 0..2 {
				bc.add(i, axis, lh2_angle(&xc, axis));
			}
		}
		assert_eq!(bc.meas_cnt(), 32);
		let (mut r, mut t) = ([[0.0; 3]; 3], [0.0; 3]);
		let err = bc.compute_pose(1, &mut r, &mut t);
		assert!((0.0..1e-9).contains(&err), "error {err}");
		assert!(close(&t, &t_true, 1e-6), "{t:?}");
		for i in 0..3 {
			assert!(close(&r[i], &r_true[i], 1e-6), "{r:?}");
		}
		// one plane only: no pose
		bc.reset();
		for (i, p) in pts.iter().enumerate() {
			let xc = [dot(&r_true[0], p) + t_true[0], dot(&r_true[1], p) + t_true[1], dot(&r_true[2], p) + t_true[2]];
			bc.add(i, 0, lh2_angle(&xc, 0));
		}
		assert_eq!(bc.compute_pose(1, &mut r, &mut t), -1.0);
	}
}
