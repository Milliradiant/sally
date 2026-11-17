//! The stations' corrections, solved with the stations (`sally calibrate`, docs/sally_design.md 4.2): each station's
//! factory calibration (OOTX fcal: phase, tilt, curve, gibbous phase and magnitude per sweep plane) is not quite right,
//! and no station pose can make up for it. Over the calibration's snapshots, one robust Levenberg-Marquardt problem:
//! every snapshot pose (6 each, eliminated by the Schur complement), every station pose but the held one (6 each), and
//! each station's ten correction terms with a prior each (FCAL_SD: over 10 sessions left out one at a time, the
//! corrections halved the station-set hop, 1.31 -> 0.60 mm, findings.md section 2).

use sally_lighthouse::BaseStationCal;

use crate::linalg::{gsolve, inv6};
use crate::model::{apply_delta, huber_rho, huber_w, model_angle, stepped, Snapshot, Station, MS_MAXH, MS_MAXLH};

/// The correction terms' prior sd, in the file's order (phase, tilt, curve, gibbous phase, gibbous magnitude; each axis
/// 0 then axis 1): rad, the gibbous phase's an angle of its own. Wider ones give the same terms, more terms overfit
/// (findings.md section 2)
pub const FCAL_SD: [f64; 10] = [2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 2e-3, 0.5, 0.5, 2e-3, 2e-3];

/// The ten terms of a station's calibration, in the file's order (corrections.rs's fcal_get)
pub fn fcal_terms(f: &[BaseStationCal; 2]) -> [f64; 10] {
	let mut v = [0.0; 10];
	for a in 0..2 {
		v[a] = f[a].phase;
		v[2 + a] = f[a].tilt;
		v[4 + a] = f[a].curve;
		v[6 + a] = f[a].gibpha;
		v[8 + a] = f[a].gibmag;
	}
	v
}

/// A calibration with the ten terms moved by `d` (the ogee terms as they are)
pub fn fcal_moved(f: &[BaseStationCal; 2], d: &[f64; 10]) -> [BaseStationCal; 2] {
	let mut o = *f;
	for a in 0..2 {
		o[a].phase += d[a];
		o[a].tilt += d[2 + a];
		o[a].curve += d[4 + a];
		o[a].gibpha += d[6 + a];
		o[a].gibmag += d[8 + a];
	}
	o
}

/// What the solve found
#[derive(Clone, Debug)]
pub struct FcalFit {
	/// per station (bsd index) the correction of its ten terms from the calibration it started from (0 for a station
	/// not in the solve)
	pub delta: [[f64; 10]; MS_MAXLH],
	/// the median |residual| of the hits taken (rad), before and after
	pub med_before: f64,
	pub med_after: f64,
	/// per station the median |residual| of its hits after (rad; NaN for none)
	pub med_station: [f64; MS_MAXLH],
	pub iterations: usize,
}

const EPS: f64 = 1e-6;

/// A snapshot's pose (r0, p0)
type SnapPose = ([f64; 9], [f64; 3]);

/// The robust cost of the hits taken (Huber, scaled by c^2 as the joint solve's), the snapshots at `poses`, and the
/// priors on the corrections
fn cost(
	snaps: &[Snapshot], poses: &[SnapPose], st: &[Station; MS_MAXLH], active: &[u8], c: f64,
	delta: &[[f64; 10]; MS_MAXLH],
) -> f64 {
	let mut sum = 0.0;
	for (i, s) in snaps.iter().enumerate() {
		let (r0, p0) = &poses[i];
		for h in 0..s.n {
			if active[i * MS_MAXH + h] == 0 {
				continue;
			}
			let l = s.lh[h] as usize;
			let r = s.angle[h] - model_angle(s, h, r0, p0, &st[l]);
			sum += huber_rho(r, c) / (c * c);
		}
	}
	for l in 0..MS_MAXLH {
		if st[l].used {
			for k in 0..10 {
				sum += (delta[l][k] / FCAL_SD[k]) * (delta[l][k] / FCAL_SD[k]);
			}
		}
	}
	sum
}

fn median_abs(snaps: &[Snapshot], st: &[Station; MS_MAXLH], active: &[u8], only: Option<usize>) -> f64 {
	let mut v = Vec::new();
	for (i, s) in snaps.iter().enumerate() {
		for h in 0..s.n {
			let l = s.lh[h] as usize;
			if active[i * MS_MAXH + h] == 0 || only.is_some_and(|o| o != l) {
				continue;
			}
			v.push((s.angle[h] - model_angle(s, h, &s.r0, &s.p0, &st[l])).abs());
		}
	}
	if v.is_empty() {
		return f64::NAN;
	}
	v.sort_by(|a, b| a.total_cmp(b));
	v[v.len() / 2]
}

/// Solve the stations' poses (but `hold`'s) and corrections with the snapshot poses, from where they are, over the
/// hits `active` marks (per snapshot MS_MAXH, the joint solve's last round's). The snapshots' seeds (r0, p0) and the
/// stations (poses, calibration) are moved to the solution; `c` is the Huber knee (rad)
pub fn solve_fcal(
	snaps: &mut [Snapshot], st: &mut [Station; MS_MAXLH], active: &[u8], hold: usize, c: f64, iters: usize,
) -> FcalFit {
	// the global parameters: 6 per station but the held one, then 10 per station
	let mut pose_g = [usize::MAX; MS_MAXLH];
	let mut fcal_g = [usize::MAX; MS_MAXLH];
	let mut g = 0;
	for l in 0..MS_MAXLH {
		if st[l].used && l != hold {
			pose_g[l] = g;
			g += 6;
		}
	}
	for l in 0..MS_MAXLH {
		if st[l].used {
			fcal_g[l] = g;
			g += 10;
		}
	}
	let n = snaps.len();
	let mut delta = [[0.0; 10]; MS_MAXLH];
	let base: Vec<[BaseStationCal; 2]> = st.iter().map(|s| s.fcal).collect();
	let med_before = median_abs(snaps, st, active, None);
	let poses_of = |snaps: &[Snapshot]| -> Vec<SnapPose> { snaps.iter().map(|s| (s.r0, s.p0)).collect() };
	let mut cur = cost(snaps, &poses_of(snaps), st, active, c, &delta);
	let mut lam = 1e-3;
	let mut it = 0;
	while it < iters {
		it += 1;
		// each station stepped in each of its parameters, once for every hit on it
		let mut st_pose: Vec<[Station; 6]> = vec![[Station::default(); 6]; MS_MAXLH];
		let mut st_fcal: Vec<[Station; 10]> = vec![[Station::default(); 10]; MS_MAXLH];
		for l in 0..MS_MAXLH {
			if !st[l].used {
				continue;
			}
			if pose_g[l] != usize::MAX {
				let poses = stepped(&st[l].r, &st[l].p, &[0.0; 6]);
				for k in 0..6 {
					st_pose[l][k] = st[l];
					(st_pose[l][k].r, st_pose[l][k].p) = poses[k];
				}
			}
			for k in 0..10 {
				let mut d = [0.0; 10];
				d[k] = EPS;
				st_fcal[l][k] = st[l];
				st_fcal[l][k].set_fcal(fcal_moved(&st[l].fcal, &d));
			}
		}
		// the normal equations: per snapshot A (6x6), b (6), B (6 x g); the global block H (g x g), gv (g)
		let mut aa = vec![0.0; n * 36];
		let mut bb = vec![0.0; n * 6];
		let mut cb = vec![0.0; n * 6 * g];
		let mut hh = vec![0.0; g * g];
		let mut gv = vec![0.0; g];
		for i in 0..n {
			let s = &snaps[i];
			let sp6 = stepped(&s.r0, &s.p0, &[0.0; 6]);
			for h in 0..s.n {
				if active[i * MS_MAXH + h] == 0 {
					continue;
				}
				let l = s.lh[h] as usize;
				let r = s.angle[h] - model_angle(s, h, &s.r0, &s.p0, &st[l]);
				let w = huber_w(r, c) / (c * c);
				let mut js = [0.0; 6];
				for k in 0..6 {
					js[k] = (s.angle[h] - model_angle(s, h, &sp6[k].0, &sp6[k].1, &st[l]) - r) / EPS;
				}
				// the global columns this hit touches and their derivatives
				let mut cols = [0usize; 16];
				let mut jg = [0.0; 16];
				let mut m = 0;
				if pose_g[l] != usize::MAX {
					for k in 0..6 {
						cols[m] = pose_g[l] + k;
						jg[m] = (s.angle[h] - model_angle(s, h, &s.r0, &s.p0, &st_pose[l][k]) - r) / EPS;
						m += 1;
					}
				}
				for k in 0..10 {
					let d = (s.angle[h] - model_angle(s, h, &s.r0, &s.p0, &st_fcal[l][k]) - r) / EPS;
					if d != 0.0 {
						cols[m] = fcal_g[l] + k;
						jg[m] = d;
						m += 1;
					}
				}
				for a in 0..6 {
					for b in 0..6 {
						aa[i * 36 + a * 6 + b] += w * js[a] * js[b];
					}
					bb[i * 6 + a] += w * js[a] * r;
					for q in 0..m {
						cb[(i * 6 + a) * g + cols[q]] += w * js[a] * jg[q];
					}
				}
				for p in 0..m {
					gv[cols[p]] += w * jg[p] * r;
					for q in 0..m {
						hh[cols[p] * g + cols[q]] += w * jg[p] * jg[q];
					}
				}
			}
		}
		// the corrections' priors
		for l in 0..MS_MAXLH {
			if fcal_g[l] == usize::MAX {
				continue;
			}
			for k in 0..10 {
				let j = fcal_g[l] + k;
				let iw = 1.0 / (FCAL_SD[k] * FCAL_SD[k]);
				hh[j * g + j] += iw;
				gv[j] += iw * delta[l][k];
			}
		}
		// damped steps until one lowers the cost
		let mut accepted = false;
		while lam <= 1e4 {
			let mut s = hh.clone();
			let mut rhs = gv.clone();
			for j in 0..g {
				s[j * g + j] += lam * (hh[j * g + j] + 1e-12);
			}
			let mut ainv = vec![0.0; n * 36];
			let mut ok = true;
			for i in 0..n {
				let mut a = [0.0; 36];
				a.copy_from_slice(&aa[i * 36..i * 36 + 36]);
				for k in 0..6 {
					a[k * 6 + k] += lam * (a[k * 6 + k] + 1e-12);
				}
				let Some(inv) = inv6(&a) else {
					ok = false;
					break;
				};
				ainv[i * 36..i * 36 + 36].copy_from_slice(&inv);
				// S -= B' A^-1 B, rhs -= B' A^-1 b
				let bi = &cb[i * 6 * g..(i + 1) * 6 * g];
				let mut aib = vec![0.0; 6 * g]; // A^-1 B
				for a in 0..6 {
					for k in 0..6 {
						let v = inv[a * 6 + k];
						if v == 0.0 {
							continue;
						}
						for j in 0..g {
							aib[a * g + j] += v * bi[k * g + j];
						}
					}
				}
				let mut aibb = [0.0; 6];
				for a in 0..6 {
					for k in 0..6 {
						aibb[a] += inv[a * 6 + k] * bb[i * 6 + k];
					}
				}
				for p in 0..g {
					let mut any = false;
					for a in 0..6 {
						if bi[a * g + p] != 0.0 {
							any = true;
						}
					}
					if !any {
						continue;
					}
					for q in 0..g {
						let mut v = 0.0;
						for a in 0..6 {
							v += bi[a * g + p] * aib[a * g + q];
						}
						s[p * g + q] -= v;
					}
					let mut v = 0.0;
					for a in 0..6 {
						v += bi[a * g + p] * aibb[a];
					}
					rhs[p] -= v;
				}
			}
			if !ok || !gsolve(&mut s, &mut rhs, g) {
				lam *= 5.0;
				continue;
			}
			// dg = -S^-1 rhs (r = measured - model, J = dr/dx: the step is -(J'WJ)^-1 J'W r)
			let dg: Vec<f64> = rhs.iter().map(|v| -v).collect();
			let mut trial: Vec<SnapPose> = poses_of(snaps);
			for i in 0..n {
				let bi = &cb[i * 6 * g..(i + 1) * 6 * g];
				let mut t = [0.0; 6];
				for a in 0..6 {
					let mut v = bb[i * 6 + a];
					for j in 0..g {
						v += bi[a * g + j] * dg[j];
					}
					t[a] = v;
				}
				let mut dx = [0.0; 6];
				for a in 0..6 {
					for k in 0..6 {
						dx[a] -= ainv[i * 36 + a * 6 + k] * t[k];
					}
				}
				trial[i] = apply_delta(&snaps[i].r0, &snaps[i].p0, &dx);
			}
			let mut tst = *st;
			let mut tdelta = delta;
			for l in 0..MS_MAXLH {
				if !st[l].used {
					continue;
				}
				if pose_g[l] != usize::MAX {
					let (r, p) = apply_delta(&st[l].r, &st[l].p, &dg[pose_g[l]..pose_g[l] + 6]);
					tst[l].r = r;
					tst[l].p = p;
				}
				for k in 0..10 {
					tdelta[l][k] += dg[fcal_g[l] + k];
				}
				tst[l].set_fcal(fcal_moved(&base[l], &tdelta[l]));
			}
			let new = cost(snaps, &trial, &tst, active, c, &tdelta);
			if new < cur {
				for (s, (r0, p0)) in snaps.iter_mut().zip(&trial) {
					s.r0 = *r0;
					s.p0 = *p0;
				}
				*st = tst;
				delta = tdelta;
				cur = new;
				lam = (lam / 3.0).max(1e-6);
				accepted = true;
				break;
			}
			lam *= 5.0;
		}
		if !accepted {
			break;
		}
	}
	let mut med_station = [f64::NAN; MS_MAXLH];
	for l in 0..MS_MAXLH {
		if st[l].used {
			med_station[l] = median_abs(snaps, st, active, Some(l));
		}
	}
	FcalFit { delta, med_before, med_after: median_abs(snaps, st, active, None), med_station, iterations: it }
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::linalg::rv2r;
	use crate::model::Station;

	/// A station's factory calibration with typical terms (a real station's: phase, tilt, curve, gibbous)
	fn factory() -> [BaseStationCal; 2] {
		let mut f = [BaseStationCal::default(); 2];
		f[0] = BaseStationCal { phase: 0.0, tilt: -0.0483, curve: 0.0731, gibpha: 3.06, gibmag: -0.0056, ..f[0] };
		f[1] = BaseStationCal { phase: -0.0067, tilt: 0.0410, curve: 0.0211, gibpha: 1.03, gibmag: 0.0039, ..f[1] };
		f
	}

	/// Three stations around a room whose calibrations are off by 1-4 mrad in tilt and curve (as real ones are), the
	/// hits of a 20-sensor device at many places made with the true ones. The solve, started from the factory values
	/// with the stations a little off, fits them; and the stations and corrections it finds predict the hits of new
	/// places (between and beyond the ones it saw) far better than the factory values with the true poses do. (The
	/// terms themselves are not all identifiable from a room's angles: some combinations trade with the poses, and the
	/// priors pick among them; what matters is that the light is predicted.)
	#[test]
	fn the_corrections_predict_new_places() {
		let truth_d: [[f64; 10]; 3] = [
			[0.0, 0.0, 3e-3, 4e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
			[0.0, 0.0, 0.0, 0.0, -3e-3, 2e-3, 0.0, 0.0, 0.0, 0.0],
			[0.0, 0.0, 2e-3, -1e-3, 1e-3, 0.0, 0.0, 0.0, 0.0, 0.0],
		];
		let mut truth = [Station::default(); MS_MAXLH];
		let places = [
			([0.0, 0.0, 2.2], [0.0, 0.0, 0.0]),
			([3.5, 0.2, 2.1], [0.0, 1.2, 0.0]),
			([1.5, 3.8, 2.3], [0.0, -1.5, 0.3]),
		];
		for (l, (p, rv)) in places.iter().enumerate() {
			// looking down at the room's middle, turned about the vertical
			let r = crate::linalg::mm3(&rv2r(rv), &rv2r(&[0.6, 0.0, 0.0]));
			truth[l].r = r;
			truth[l].p = *p;
			truth[l].used = true;
			truth[l].set_fcal(fcal_moved(&factory(), &truth_d[l]));
		}
		// a device: 20 sensors on a 6 cm cap
		let sensors: Vec<[f64; 3]> = (0..20)
			.map(|i| {
				let a = i as f64 * 2.39996;
				[0.03 * a.cos(), 0.03 * a.sin(), 0.02 * ((i % 4) as f64)]
			})
			.collect();
		let snaps_at = |places: &[[f64; 3]], st: &[Station; MS_MAXLH]| -> Vec<Snapshot> {
			let mut out = Vec::new();
			for (k, p0) in places.iter().enumerate() {
				let r0 = rv2r(&[0.3 * (k % 5) as f64, 0.7 * (k % 3) as f64, 0.9 * k as f64]);
				let mut s = Snapshot { r0, p0: *p0, ..Snapshot::default() };
				for l in 0..3 {
					for (si, pt) in sensors.iter().enumerate() {
						for plane in 0..2i8 {
							if s.n >= MS_MAXH || (si + k + l) % 3 != 0 {
								continue;
							}
							s.lh[s.n] = l as u8;
							s.plane[s.n] = plane;
							s.pt[s.n] = *pt;
							s.n += 1;
							let h = s.n - 1;
							s.angle[h] = model_angle(&s, h, &r0, p0, &st[l]);
						}
					}
				}
				out.push(s);
			}
			out
		};
		let grid = |n: usize, off: f64| -> Vec<[f64; 3]> {
			let mut v = Vec::new();
			for x in 0..n {
				for y in 0..n {
					for t in 0..3 {
						let f = 2.5 / (n - 1) as f64;
						v.push([0.4 + off + f * x as f64, 0.4 + off + f * 1.1 * y as f64, 0.3 + off + 0.5 * t as f64]);
					}
				}
			}
			v
		};
		let mut snaps = snaps_at(&grid(6, 0.0), &truth);
		// start: the factory values, stations 1 and 2 off by mm and mrad, the snapshots a little off
		let mut st = truth;
		for l in 0..3 {
			st[l].set_fcal(factory());
		}
		for l in 1..3 {
			let (r, p) = apply_delta(&st[l].r, &st[l].p, &[0.004, -0.003, 0.002, 0.002, -0.001, 0.0015]);
			st[l].r = r;
			st[l].p = p;
		}
		for (i, s) in snaps.iter_mut().enumerate() {
			let d = [0.001 * ((i % 3) as f64 - 1.0), 0.0005, -0.001, 0.0, 0.001, -0.0005];
			let (r0, p0) = apply_delta(&s.r0, &s.p0, &d);
			s.r0 = r0;
			s.p0 = p0;
		}
		let active = vec![1u8; snaps.len() * MS_MAXH];
		let fit = solve_fcal(&mut snaps, &mut st, &active, 0, 1e-3, 30);
		assert!(fit.med_before > 5e-4 && fit.med_after < 2e-5, "{} -> {}", fit.med_before, fit.med_after);
		// new places, each fitted (its pose only) to the solved stations, and to the true poses with the factory values
		let held = snaps_at(&grid(5, 0.17), &truth);
		let mut factory_st = truth;
		for l in 0..3 {
			factory_st[l].set_fcal(factory());
		}
		let fit_poses = |st: &[Station; MS_MAXLH]| -> f64 {
			let mut ss = held.clone();
			let mut s2 = *st;
			let act = vec![1u8; ss.len() * MS_MAXH];
			// the stations held: only the snapshot poses move (no station free, the corrections as they are)
			for l in 0..MS_MAXLH {
				s2[l].used = false;
			}
			let mut v = Vec::new();
			for (i, s) in ss.iter_mut().enumerate() {
				let one = std::slice::from_mut(s);
				let mut st1 = *st;
				for l in 0..MS_MAXLH {
					st1[l].used = st[l].used;
				}
				pose_only(one, &st1, &act[i * MS_MAXH..(i + 1) * MS_MAXH]);
				for h in 0..one[0].n {
					let l = one[0].lh[h] as usize;
					v.push((one[0].angle[h] - model_angle(&one[0], h, &one[0].r0, &one[0].p0, &st[l])).abs());
				}
			}
			v.sort_by(|a, b| a.total_cmp(b));
			v[v.len() / 2]
		};
		let solved = fit_poses(&st);
		let factory_only = fit_poses(&factory_st);
		assert!(solved < factory_only / 10.0, "held out: solved {solved} factory {factory_only}");
	}

	/// The snapshot's pose fitted to the stations as they are (Gauss-Newton, a few steps)
	fn pose_only(s: &mut [Snapshot], st: &[Station; MS_MAXLH], active: &[u8]) {
		for _ in 0..8 {
			let sp6 = stepped(&s[0].r0, &s[0].p0, &[0.0; 6]);
			let mut a = [0.0; 36];
			let mut b = [0.0; 6];
			for h in 0..s[0].n {
				if active[h] == 0 {
					continue;
				}
				let l = s[0].lh[h] as usize;
				let r = s[0].angle[h] - model_angle(&s[0], h, &s[0].r0, &s[0].p0, &st[l]);
				let mut j = [0.0; 6];
				for k in 0..6 {
					j[k] = (s[0].angle[h] - model_angle(&s[0], h, &sp6[k].0, &sp6[k].1, &st[l]) - r) / EPS;
				}
				for p in 0..6 {
					for q in 0..6 {
						a[p * 6 + q] += j[p] * j[q];
					}
					b[p] += j[p] * r;
				}
			}
			let inv = inv6(&a).expect("posed");
			let mut dx = [0.0; 6];
			for p in 0..6 {
				for q in 0..6 {
					dx[p] -= inv[p * 6 + q] * b[q];
				}
			}
			let (r0, p0) = apply_delta(&s[0].r0, &s[0].p0, &dx);
			s[0].r0 = r0;
			s[0].p0 = p0;
		}
	}
}
