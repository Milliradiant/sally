//! Moved stations and the frame (driver_map_solve.c's station_check, relocate_station, pose_gap, rigid_fit): each
//! station's light checked against poses fitted to the others only, a moved station solved on its own from those poses
//! (resection), and the rigid motion of the station set relative to the calibrated map (Horn).

use sally_math::{c_acos, dist3d, norm3d, Pose};

use crate::linalg::{gsolve, mv3, q2r, quatinnerproduct};
use crate::model::{
	apply_delta, huber_rho, huber_w, model_angle, stepped_station, Snapshot, Station, MS_MAXH, MS_MAXLH,
};
use crate::settings::Settings;
use crate::solve::{median_of, warm_start};

/// station_check's results per station
pub struct StationCheck {
	/// the median |residual| of the station's own hits from poses fitted to the other stations (NaN: not checked)
	pub med: [f64; MS_MAXLH],
	/// its hits and the snapshots they came from
	pub nh: [i32; MS_MAXLH],
	pub ns: [i32; MS_MAXLH],
	/// the median residual of the OTHER stations' hits in that same fit
	pub rest: [f64; MS_MAXLH],
}

/// station_check: per station, leave one out: every snapshot that two other stations light well is fitted to the
/// other stations only, and med is the median |angle residual| of the station's own hits from those poses; nh its hit
/// count, ns the number of such snapshots; rest is the median residual of the OTHER stations' hits in that same fit
/// (how well the rest agrees once the station is gone: when one station moved, removing it is the only removal that
/// leaves the rest consistent, while every other station's own residual is inflated too, since its poses still lean
/// on the moved one). Fitting the poses with the station itself in (robust or not) lets a moved station pull them
/// toward itself and hides most of the disagreement: on a 30 mm / 26 mrad bump the in-fit median was 3.6 mrad, this
/// 8.7.
pub fn station_check(ms: &Settings, p: &[Snapshot], st: &[Station; MS_MAXLH]) -> StationCheck {
	let n_snaps = p.len();
	let mut xs = vec![0.0; n_snaps * 6];
	let mut active = vec![0u8; n_snaps * MS_MAXH];
	let mut res = vec![0.0; n_snaps * MS_MAXH];
	let mut ores = vec![0.0; n_snaps * MS_MAXH];
	let mut cnt = [0i32; MS_MAXLH];
	for i in 0..n_snaps {
		for h in 0..p[i].n {
			cnt[p[i].lh[h] as usize] += 1;
		}
	}
	let mut out = StationCheck { med: [0.0; MS_MAXLH], nh: [0; MS_MAXLH], ns: [0; MS_MAXLH], rest: [0.0; MS_MAXLH] };
	for l in 0..MS_MAXLH {
		out.med[l] = f64::NAN;
		out.rest[l] = f64::NAN;
		out.nh[l] = 0;
		out.ns[l] = 0;
		if !st[l].used || cnt[l] < ms.min_station_hits {
			continue;
		}
		let mut so = *st;
		so[l].used = false;
		xs.iter_mut().for_each(|v| *v = 0.0);
		warm_start(ms, p, &so, &mut xs, &mut active);
		let (mut n, mut no) = (0, 0);
		for i in 0..n_snaps {
			let s = &p[i];
			let mut per = [0i32; MS_MAXLH];
			let (mut own, mut others) = (0, 0);
			for h in 0..s.n {
				if s.lh[h] as usize == l {
					own += 1;
				} else if active[i * MS_MAXH + h] != 0 {
					per[s.lh[h] as usize] += 1;
				}
			}
			for k in 0..MS_MAXLH {
				others += (per[k] >= ms.min_hits) as i32;
			}
			if others < 2 || own < ms.min_hits {
				continue;
			}
			out.ns[l] += 1;
			let (rs, ps) = apply_delta(&s.r0, &s.p0, &xs[i * 6..i * 6 + 6]);
			for h in 0..s.n {
				let lh = s.lh[h] as usize;
				if lh == l {
					res[n] = (s.angle[h] - model_angle(s, h, &rs, &ps, &st[l])).abs();
					n += 1;
				} else if so[lh].used {
					ores[no] = (s.angle[h] - model_angle(s, h, &rs, &ps, &st[lh])).abs();
					no += 1;
				}
			}
		}
		out.nh[l] = n as i32;
		if n != 0 {
			out.med[l] = median_of(&mut res[..n]);
		}
		if no != 0 {
			out.rest[l] = median_of(&mut ores[..no]);
		}
	}
	out
}

/// relocate_station's results
pub struct Relocation {
	/// false if too few snapshots qualify (the rest is then as far as it got)
	pub ok: bool,
	/// the solved station (R, p)
	pub out: Station,
	/// the median residual of its hits at the solved pose
	pub med_after: f64,
	pub nsnap: i32,
	pub nhit: i32,
	/// the median residual of the same hits at the map's pose of the station, and at alt when one was given
	pub med_cur: f64,
	pub med_alt: f64,
}

/// relocate_station: station l solved alone: the snapshots are fitted to the other stations only (st with l left
/// out), and l's pose is the one that best explains its own hits from those poses (resection; Levenberg, robust with a
/// scale that follows the residuals down, since the start can be centimeters off). Only snapshots with two other
/// stations give a pose. med_cur, med_alt, med_after keep the values they come in with where the C leaves its outputs
/// unwritten.
#[allow(clippy::too_many_arguments)] // the C's in and out parameters
pub fn relocate_station(
	ms: &Settings, p: &[Snapshot], st: &[Station; MS_MAXLH], l: usize, out: Station, med_after: f64, med_cur: f64,
	alt: Option<&Station>, med_alt: f64,
) -> Relocation {
	let n_snaps = p.len();
	let mut so = *st;
	so[l].used = false;
	let mut xs = vec![0.0; n_snaps * 6];
	let mut active = vec![0u8; n_snaps * MS_MAXH];
	warm_start(ms, p, &so, &mut xs, &mut active);
	// the hits of l, with the pose of their snapshot
	let mut hi: Vec<usize> = Vec::new();
	let mut hh: Vec<usize> = Vec::new();
	let mut rp: Vec<([f64; 9], [f64; 3])> = vec![([0.0; 9], [0.0; 3]); n_snaps];
	let mut ns = 0;
	for i in 0..n_snaps {
		let s = &p[i];
		let mut per = [0i32; MS_MAXLH];
		let mut own = 0;
		for h in 0..s.n {
			if s.lh[h] as usize == l {
				own += 1;
			} else if active[i * MS_MAXH + h] != 0 {
				per[s.lh[h] as usize] += 1;
			}
		}
		let mut others = 0;
		for k in 0..MS_MAXLH {
			others += (per[k] >= ms.min_hits) as i32;
		}
		if others < 2 || own < ms.min_hits {
			continue;
		}
		rp[i] = apply_delta(&s.r0, &s.p0, &xs[i * 6..i * 6 + 6]);
		ns += 1;
		for h in 0..s.n {
			if s.lh[h] as usize == l {
				hi.push(i);
				hh.push(h);
			}
		}
	}
	let n = hi.len();
	let mut res =
		Relocation { ok: ns >= ms.move_min_snaps, out, med_after, nsnap: ns, nhit: n as i32, med_cur, med_alt };
	let ok = res.ok;
	let mut r = vec![0.0; n + 1];
	let mut ra = vec![0.0; n + 1];
	let angle_at = |k: usize, at: &Station| -> f64 {
		let (rs, ps) = &rp[hi[k]];
		p[hi[k]].angle[hh[k]] - model_angle(&p[hi[k]], hh[k], rs, ps, at)
	};
	let passes = if alt.is_some() { 2 } else { 1 };
	let mut pass = 0;
	while ok && pass < passes {
		let at = if pass != 0 { alt.unwrap_or(&st[l]) } else { &st[l] };
		for k in 0..n {
			ra[k] = angle_at(k, at).abs();
		}
		let m = median_of(&mut ra[..n]);
		if pass != 0 {
			res.med_alt = m;
		} else {
			res.med_cur = m;
		}
		pass += 1;
	}
	let mut cur = st[l];
	let mut lambda = 1e-3;
	let mut it = 0;
	while ok && it < 40 {
		for k in 0..n {
			r[k] = angle_at(k, &cur);
			ra[k] = r[k].abs();
		}
		let mut c = 1.4826 * median_of(&mut ra[..n]);
		if c < ms.huber {
			c = ms.huber;
		}
		let cut = 5.0 * c;
		let mut cost = 0.0;
		let mut jtj = [0.0; 36];
		let mut jtr = [0.0; 6];
		// sally: the station's six stepped poses (d[a] = 1e-6 from zero), once for all its hits (stepped_station)
		let curp = stepped_station(&cur, &cur.r, &cur.p, &[0.0; 6]);
		for k in 0..n {
			if r[k].abs() > cut {
				continue;
			}
			cost += huber_rho(r[k], c);
			let mut j = [0.0; 6];
			for a in 0..6 {
				j[a] = (angle_at(k, &curp[a]) - r[k]) / 1e-6;
			}
			let w = huber_w(r[k], c);
			for a in 0..6 {
				jtr[a] -= w * j[a] * r[k];
				for b in 0..6 {
					jtj[a * 6 + b] += w * j[a] * j[b];
				}
			}
		}
		let mut accepted = false;
		let mut tries = 0;
		while tries < 8 && !accepted {
			let mut a = jtj;
			let mut b = jtr;
			for ai in 0..6 {
				a[ai * 7] += 1e-12 + lambda * a[ai * 7];
			}
			if !gsolve(&mut a, &mut b, 6) {
				break;
			}
			let mut tr = cur;
			let (r2, p2) = apply_delta(&cur.r, &cur.p, &b);
			tr.r = r2;
			tr.p = p2;
			let mut nc = 0.0;
			for k in 0..n {
				if r[k].abs() > cut {
					continue;
				}
				nc += huber_rho(angle_at(k, &tr), c);
			}
			if nc < cost {
				cur = tr;
				lambda = if lambda / 3.0 > 1e-7 { lambda / 3.0 } else { 1e-7 };
				accepted = true;
				if norm3d(&[b[0], b[1], b[2]]) < 1e-7 && norm3d(&[b[3], b[4], b[5]]) < 1e-8 && it > 4 {
					it = 40;
				}
			} else {
				lambda *= 10.0;
			}
			tries += 1;
		}
		if !accepted && it > 4 {
			break;
		}
		it += 1;
	}
	if ok {
		for k in 0..n {
			ra[k] = angle_at(k, &cur).abs();
		}
		res.med_after = median_of(&mut ra[..n]);
		res.out = cur;
	}
	res
}

/// pose_gap: the distance between two station poses, position (m) and rotation (rad)
pub fn pose_gap(a: &Pose, b: &Pose) -> (f64, f64) {
	let dp = dist3d(&a.pos, &b.pos);
	let na = (quatinnerproduct(&a.rot, &a.rot) * quatinnerproduct(&b.rot, &b.rot)).sqrt(); // config poses need not be unit
	let qa = if na > 0.0 { quatinnerproduct(&a.rot, &b.rot).abs() / na } else { 1.0 };
	let dr = 2.0 * c_acos(if qa > 1.0 { 1.0 } else { qa });
	(dp, dr)
}

/// rigid_fit: the rotation and translation taking the points `from` onto `to` best (Horn's quaternion method, its
/// largest eigenvector by a shifted power iteration); None for fewer than three points, a point that is not finite
/// (sally: the C's would hold the frame with NaN, which then moved every station there) or a degenerate iteration
pub fn rigid_fit(from: &[[f64; 3]], to: &[[f64; 3]]) -> Option<([f64; 9], [f64; 3])> {
	let n = from.len();
	if n < 3 || from.iter().chain(to).flatten().any(|v| !v.is_finite()) {
		return None;
	}
	let nf = n as i32 as f64;
	let mut cf = [0.0; 3];
	let mut ct = [0.0; 3];
	for i in 0..n {
		for k in 0..3 {
			cf[k] += from[i][k] / nf;
			ct[k] += to[i][k] / nf;
		}
	}
	let mut sx = [0.0; 9];
	for i in 0..n {
		for a in 0..3 {
			for b in 0..3 {
				sx[a * 3 + b] += (from[i][a] - cf[a]) * (to[i][b] - ct[b]);
			}
		}
	}
	// Horn's 4x4 matrix; largest eigenvector by power iteration (shifted)
	let (sxx, sxy, sxz, syx, syy, syz, szx, szy, szz) = (sx[0], sx[1], sx[2], sx[3], sx[4], sx[5], sx[6], sx[7], sx[8]);
	#[rustfmt::skip]
	let mut n4 = [
		sxx + syy + szz, syz - szy, szx - sxz, sxy - syx,
		syz - szy, sxx - syy - szz, sxy + syx, szx + sxz,
		szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy,
		sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz,
	];
	let mut shift = 0.0;
	for v in n4.iter() {
		shift += v.abs();
	}
	for i in 0..4 {
		n4[i * 5] += shift;
	}
	let mut q = [1.0, 0.0, 0.0, 0.0];
	for _ in 0..200 {
		let mut nq = [0.0; 4];
		for a in 0..4 {
			for b in 0..4 {
				nq[a] += n4[a * 4 + b] * q[b];
			}
		}
		let nn = (nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3]).sqrt();
		if nn == 0.0 {
			return None;
		}
		for a in 0..4 {
			q[a] = nq[a] / nn;
		}
	}
	let r = q2r(&q);
	let rc = mv3(&r, &cf);
	let mut t = [0.0; 3];
	for k in 0..3 {
		t[k] = ct[k] - rc[k];
	}
	Some((r, t))
}
