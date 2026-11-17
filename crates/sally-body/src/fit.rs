//! The joints' centres found from the motion itself (docs/mocap_plan.md section 2). Three rigid bodies in a chain, each
//! with a tracker on it (the waist, a knee, a foot; or the chest, an elbow, a hand): the joint between the first two is
//! a point fixed in both trackers' frames, so its distance from the next joint stays the same however the limb moves.
//! Fitted by least squares: the hip centre in the waist tracker's frame, the knee centre in the knee tracker's, the
//! ankle in the foot's, and the two segments' lengths, so that every sampled frame has |hip - knee| = thigh and |knee -
//! ankle| = shin.
//!
//! Over every session at once: the lengths are the same bones in all of them, the points are each session's own (a
//! tracker can be strapped on again in between). So a session that keeps a leg straight (standing, twisting), which
//! shows only the leg's length and not where along it the knee is, takes the split from the sessions that bend it.
//! Weak priors on the points and the lengths hold what no session shows (where along a hinge's axis its centre is).
//! Robust: a frame whose segment is far off its length (a controller hanging from its strap, a tracker slipping) is
//! weighed down, so it does not drag the fit.

use sally_math::math3d::{dot, len, qconj, qrot, xform, Pose, V3};

/// A chain fitted for one session: `a` in the first tracker's frame, `b` in the second's, `c` in the third's; the
/// segments' lengths (m, shared by the sessions fitted together), this session's residuals' rms of each (m, the
/// weighed-down frames left out) and its frames, and how many were weighed down to under half
#[derive(Clone, Copy, Debug, Default)]
pub struct Chain {
	pub a: V3,
	pub b: V3,
	pub c: V3,
	pub l1: f64,
	pub l2: f64,
	pub rms1: f64,
	pub rms2: f64,
	pub n: usize,
	pub outliers: usize,
}

/// How much a session's frames weigh against the priors: as this many independent frames of this sd (m), however many
/// it has. The frames are not independent (a tracker's error is the same for a while, skin and strap move with the
/// motion), so all of them as such would outweigh any prior and let a joint run off along a direction the motion
/// hardly shows; this way a prior holds there and the motion still moves a joint where it shows it
const N_EFF: f64 = 100.0;
const SD_FRAME: f64 = 0.005;
/// The robust weight's scale: a residual of this (m) weighs half
const ROBUST: f64 = 0.015;

/// One session's frames of a chain's three trackers (the same instants in each) and its prior on the three points
pub struct Set<'a> {
	pub a: &'a [Pose],
	pub b: &'a [Pose],
	pub c: &'a [Pose],
	pub points: [V3; 3],
}

/// The priors: the points' sd, the lengths and their sd (m)
#[derive(Clone, Copy, Debug)]
pub struct Prior {
	pub points_sd: f64,
	pub lengths: [f64; 2],
	pub lengths_sd: f64,
}

fn v3(x: &[f64]) -> V3 {
	V3::new(x[0], x[1], x[2])
}

/// Solves A x = b for a small dense A (Gaussian elimination, partial pivoting); None when singular
pub fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
	let n = b.len();
	for col in 0..n {
		let piv = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
		if a[piv][col].abs() < 1e-300 {
			return None;
		}
		a.swap(col, piv);
		b.swap(col, piv);
		for row in col + 1..n {
			let f = a[row][col] / a[col][col];
			for k in col..n {
				a[row][k] -= f * a[col][k];
			}
			b[row] -= f * b[col];
		}
	}
	let mut x = vec![0.0; n];
	for row in (0..n).rev() {
		let s: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
		x[row] = (b[row] - s) / a[row][row];
	}
	Some(x)
}

/// The two residuals of frame `i` of set `s` (m) and their gradients: (r, d/dl, d/d(first point), d/d(second point))
fn frame(set: &Set, i: usize, x: &[f64], base: usize) -> [(f64, V3, V3); 2] {
	let (a, b, c) = (v3(&x[base..]), v3(&x[base + 3..]), v3(&x[base + 6..]));
	let (pa, pb, pc) = (xform(&set.a[i], a), xform(&set.b[i], b), xform(&set.c[i], c));
	let (d1, d2) = (pa - pb, pb - pc);
	let (n1, n2) = (len(d1).max(1e-9), len(d2).max(1e-9));
	let (u1, u2) = (d1 * (1.0 / n1), d2 * (1.0 / n2));
	// d|R a + p - (S b + q)|/da = u^T R = (R^T u)^T
	[
		(n1 - x[0], qrot(qconj(set.a[i].q), u1), -qrot(qconj(set.b[i].q), u1)),
		(n2 - x[1], qrot(qconj(set.b[i].q), u2), -qrot(qconj(set.c[i].q), u2)),
	]
}

/// The cost at x with the frames' weights w
fn cost(sets: &[Set], prior: &Prior, w: &[Vec<[f64; 2]>], x: &[f64]) -> f64 {
	let mut c = 0.0;
	for (s, set) in sets.iter().enumerate() {
		let k2 = N_EFF / set.a.len().max(1) as f64 / (SD_FRAME * SD_FRAME);
		for i in 0..set.a.len() {
			let f = frame(set, i, x, 2 + 9 * s);
			c += k2 * (w[s][i][0] * f[0].0 * f[0].0 + w[s][i][1] * f[1].0 * f[1].0);
		}
		for (j, p) in set.points.iter().enumerate() {
			let q = v3(&x[2 + 9 * s + 3 * j..]);
			let d = q - *p;
			c += dot(d, d) / (prior.points_sd * prior.points_sd);
		}
	}
	for j in 0..2 {
		c += (x[j] - prior.lengths[j]).powi(2) / (prior.lengths_sd * prior.lengths_sd);
	}
	c
}

/// Fits a chain over sessions: x = [l1, l2, then a, b, c per set]; Levenberg-Marquardt on the normal equations with the
/// gradients worked out, three rounds of robust weights
pub fn fit_chains(sets: &[Set], prior: &Prior) -> Vec<Chain> {
	let np = 2 + 9 * sets.len();
	let mut x = vec![0.0; np];
	x[..2].copy_from_slice(&prior.lengths);
	for (s, set) in sets.iter().enumerate() {
		for (j, p) in set.points.iter().enumerate() {
			x[2 + 9 * s + 3 * j..2 + 9 * s + 3 * j + 3].copy_from_slice(&[p.x, p.y, p.z]);
		}
	}
	let mut w: Vec<Vec<[f64; 2]>> = sets.iter().map(|s| vec![[1.0, 1.0]; s.a.len()]).collect();
	for round in 0..3 {
		let mut lambda = 1e-3;
		let mut c = cost(sets, prior, &w, &x);
		for _ in 0..60 {
			let mut ata = vec![vec![0.0; np]; np];
			let mut atr = vec![0.0; np];
			for (s, set) in sets.iter().enumerate() {
				let base = 2 + 9 * s;
				let k2 = N_EFF / set.a.len().max(1) as f64 / (SD_FRAME * SD_FRAME);
				for i in 0..set.a.len() {
					let f = frame(set, i, &x, base);
					for (seg, (r, g1, g2)) in f.iter().enumerate() {
						let wt = k2 * w[s][i][seg];
						// the row's nonzero entries: its length, then its two points
						let off = base + 3 * seg;
						let idx = [seg, off, off + 1, off + 2, off + 3, off + 4, off + 5];
						let jr = [-1.0, g1.x, g1.y, g1.z, g2.x, g2.y, g2.z];
						for p in 0..7 {
							atr[idx[p]] -= wt * jr[p] * r;
							for q in 0..7 {
								ata[idx[p]][idx[q]] += wt * jr[p] * jr[q];
							}
						}
					}
				}
				for (j, p) in set.points.iter().enumerate() {
					let q = v3(&x[base + 3 * j..]);
					let d = q - *p;
					let wp = 1.0 / (prior.points_sd * prior.points_sd);
					for (k, dk) in [d.x, d.y, d.z].iter().enumerate() {
						ata[base + 3 * j + k][base + 3 * j + k] += wp;
						atr[base + 3 * j + k] -= wp * dk;
					}
				}
			}
			for j in 0..2 {
				let wl = 1.0 / (prior.lengths_sd * prior.lengths_sd);
				ata[j][j] += wl;
				atr[j] -= wl * (x[j] - prior.lengths[j]);
			}
			let mut stepped = false;
			while lambda < 1e10 {
				let mut a = ata.clone();
				for k in 0..np {
					a[k][k] += lambda * ata[k][k].max(1e-12);
				}
				let Some(dx) = solve(a, atr.clone()) else {
					lambda *= 10.0;
					continue;
				};
				let xn: Vec<f64> = x.iter().zip(&dx).map(|(a, d)| a + d).collect();
				let cn = cost(sets, prior, &w, &xn);
				if cn < c {
					let small = dx.iter().map(|d| d.abs()).fold(0.0, f64::max) < 1e-8;
					(x, c) = (xn, cn);
					lambda = (lambda * 0.3).max(1e-9);
					stepped = !small;
					break;
				}
				lambda *= 10.0;
			}
			if !stepped {
				break;
			}
		}
		if round < 2 {
			for (s, set) in sets.iter().enumerate() {
				for i in 0..set.a.len() {
					let f = frame(set, i, &x, 2 + 9 * s);
					for seg in 0..2 {
						w[s][i][seg] = 1.0 / (1.0 + (f[seg].0 / ROBUST).powi(2));
					}
				}
			}
		}
	}
	sets.iter()
		.enumerate()
		.map(|(s, set)| {
			let base = 2 + 9 * s;
			let (mut s1, mut s2, mut n, mut out) = (0.0, 0.0, 0usize, 0usize);
			for i in 0..set.a.len() {
				let f = frame(set, i, &x, base);
				if w[s][i][0] < 0.5 || w[s][i][1] < 0.5 {
					out += 1;
					continue;
				}
				s1 += f[0].0 * f[0].0;
				s2 += f[1].0 * f[1].0;
				n += 1;
			}
			let rms = |v: f64| if n > 0 { (v / n as f64).sqrt() } else { f64::NAN };
			Chain {
				a: v3(&x[base..]),
				b: v3(&x[base + 3..]),
				c: v3(&x[base + 6..]),
				l1: x[0],
				l2: x[1],
				rms1: rms(s1),
				rms2: rms(s2),
				n: set.a.len(),
				outliers: out,
			}
		})
		.collect()
}

#[cfg(test)]
mod tests {
	use super::*;
	use sally_math::math3d::{qmul, qnorm, qrot, Q4};

	fn axis_angle(ax: V3, ang: f64) -> Q4 {
		let s = (ang / 2.0).sin();
		qnorm(Q4::new((ang / 2.0).cos(), ax.x * s, ax.y * s, ax.z * s))
	}

	#[test]
	fn solve_small_system() {
		let x = solve(vec![vec![2.0, 1.0], vec![1.0, 3.0]], vec![3.0, 5.0]).unwrap();
		assert!((x[0] - 0.8).abs() < 1e-12 && (x[1] - 1.4).abs() < 1e-12, "{x:?}");
	}

	/// A leg swinging and bending: the pelvis turning about, the thigh about the hip, the shin about the knee (a hinge
	/// about the thigh's x), the foot about the ankle (two axes); each tracker strapped off its joint. From a start 8 cm
	/// off the fit finds the hip and the ankle back, and the knee up to where along the hinge's axis (which no motion of
	/// a hinge shows: the prior's x stays)
	#[test]
	fn a_moving_leg_gives_its_joints_back() {
		let (thigh, shin) = (0.42, 0.40);
		let hip_in_waist = V3::new(0.09, -0.10, 0.12);
		let knee_in_kt = V3::new(0.0, 0.03, 0.06);
		let ankle_in_ft = V3::new(0.0, -0.05, 0.07);
		let (mut sa, mut sb, mut sc) = (Vec::new(), Vec::new(), Vec::new());
		for i in 0..400 {
			let t = i as f64 * 0.05;
			let pelvis = Pose {
				p: V3::new(0.3 * (0.2 * t).sin(), 1.0, 0.2 * t),
				q: axis_angle(V3::new(0.0, 1.0, 0.0), 0.8 * (0.3 * t).sin()),
			};
			let hip = xform(&pelvis, hip_in_waist);
			// the thigh: flexion and abduction
			let tq = qmul(
				pelvis.q,
				qmul(
					axis_angle(V3::new(1.0, 0.0, 0.0), 0.9 * (0.7 * t).sin()),
					axis_angle(V3::new(0.0, 0.0, 1.0), 0.3 * (1.1 * t).cos()),
				),
			);
			let knee = hip + qrot(tq, V3::new(0.0, -thigh, 0.0));
			let sq = qmul(tq, axis_angle(V3::new(1.0, 0.0, 0.0), -1.2 * (0.5 * (0.9 * t).sin() + 0.5)));
			let ankle = knee + qrot(sq, V3::new(0.0, -shin, 0.0));
			// the knee's tracker on the thigh, the foot's below the ankle, turned with the shin and a bit more
			let kt = Pose { p: knee - qrot(tq, knee_in_kt), q: tq };
			let ankle_turn = qmul(
				axis_angle(V3::new(1.0, 0.0, 0.0), 0.3 * (1.3 * t).sin()),
				axis_angle(V3::new(0.0, 0.0, 1.0), 0.25 * (0.8 * t).cos()),
			);
			let fq = qmul(sq, ankle_turn);
			let ft = Pose { p: ankle - qrot(fq, ankle_in_ft), q: fq };
			sa.push(pelvis);
			sb.push(kt);
			sc.push(ft);
		}
		let off = V3::new(0.05, -0.05, 0.04);
		let prior = Prior { points_sd: 0.15, lengths: [thigh + 0.01, shin - 0.01], lengths_sd: 0.04 };
		let set = Set { a: &sa, b: &sb, c: &sc, points: [hip_in_waist + off, knee_in_kt - off, ankle_in_ft + off] };
		let ch = fit_chains(&[set], &prior)[0];
		// the ankle's two small ranges show its centre less sharply: a few mm of the lengths' prior gets in
		assert!(len(ch.a - hip_in_waist) < 3e-3 && len(ch.c - ankle_in_ft) < 6e-3, "{ch:?}");
		let kb = ch.b - knee_in_kt;
		assert!(kb.y.abs() < 3e-3 && kb.z.abs() < 3e-3 && (kb.x + off.x).abs() < 0.01, "{ch:?}");
		assert!(ch.rms1 < 1e-3 && ch.rms2 < 1e-3, "{ch:?}");
	}
}
