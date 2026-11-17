//! The joint solve (driver_map_solve.c's ms_sys, total_cost, assemble, schur, median_of, warm_start, joint_solve): one
//! robust Levenberg-Marquardt problem over every snapshot pose (6 each) and every station pose (6 each), the snapshot
//! blocks eliminated by the Schur complement on the station block, with what earlier solves learned (an information
//! matrix) and a weak prior toward the calibrated map.

use crate::linalg::{gsolve, inv6, mm3, r2rv, transpose3};
use crate::model::{
	apply_delta, huber_rho, huber_w, model_angle, stepped, stepped_station, Snapshot, Station, MS_MAXG, MS_MAXH,
	MS_MAXLH,
};
use crate::settings::Settings;

/// ms_sys: the normal equations of a batch
pub struct System {
	pub n: usize,
	/// G: six per station taking part
	pub g: usize,
	/// station (bsd index) of each 6-block
	pub gl: [usize; MS_MAXG / 6],
	/// N x 6 snapshot deltas
	pub xs: Vec<f64>,
	/// per snapshot: 6x6, 6, 6xG
	pub a: Vec<f64>,
	pub b: Vec<f64>,
	pub c: Vec<f64>,
	/// N x MS_MAXH: the hits that take part
	pub active: Vec<u8>,
	/// G x G (stride G) and G: the station block and its gradient
	pub h: Vec<f64>,
	pub gv: [f64; MS_MAXG],
}

/// stations_at: the stations moved by the station deltas xg (each station's block at its g; fixed stations as they are)
pub fn stations_at(st: &[Station; MS_MAXLH], xg: &[f64]) -> [Station; MS_MAXLH] {
	let mut out = *st;
	for l in 0..MS_MAXLH {
		if st[l].g >= 0 {
			let g = st[l].g as usize;
			let (r, p) = apply_delta(&st[l].r, &st[l].p, &xg[g..g + 6]);
			out[l].r = r;
			out[l].p = p;
		}
	}
	out
}

/// start_dev: the deviation of a station (in delta space) from the calibrated map: the position difference and the
/// world-left rotation vector
pub fn start_dev(s: &Station, start: &Station) -> [f64; 6] {
	let mut dev = [0.0; 6];
	for k in 0..3 {
		dev[k] = s.p[k] - start.p[k];
	}
	let rt = transpose3(&start.r);
	let dr = mm3(&s.r, &rt);
	let rv = r2rv(&dr);
	dev[3..6].copy_from_slice(&rv);
	dev
}

/// total_cost: robust data + Xg' Lam Xg + the weak prior to the calibrated map
#[allow(clippy::too_many_arguments)] // the C's signature
pub fn total_cost(
	ms: &Settings, p: &[Snapshot], s: &System, xs: &[f64], st: &[Station; MS_MAXLH], xg: &[f64],
	start: &[Station; MS_MAXLH], lam: &[f64],
) -> f64 {
	let sc = stations_at(st, xg);
	let mut cost = 0.0;
	for i in 0..s.n {
		let sn = &p[i];
		let (rs, ps) = apply_delta(&sn.r0, &sn.p0, &xs[i * 6..i * 6 + 6]);
		let mut c = 0.0;
		for h in 0..sn.n {
			if s.active[i * MS_MAXH + h] != 0 {
				c += huber_rho(sn.angle[h] - model_angle(sn, h, &rs, &ps, &sc[sn.lh[h] as usize]), ms.huber);
			}
		}
		cost += c / (ms.huber * ms.huber);
	}
	let g = s.g;
	for a in 0..g {
		for c in 0..g {
			cost += xg[a] * lam[a * g + c] * xg[c];
		}
	}
	for l in 0..MS_MAXLH {
		if st[l].g < 0 {
			continue;
		}
		let dev = start_dev(&sc[l], &start[l]);
		for k in 0..3 {
			cost += st[l].prior_w
				* (dev[k] * dev[k] / (ms.prior_pos * ms.prior_pos)
					+ dev[3 + k] * dev[3 + k] / (ms.prior_rot * ms.prior_rot));
		}
	}
	cost
}

/// assemble: the normal-equation pieces at (Xs, Xg), data only
pub fn assemble(ms: &Settings, p: &[Snapshot], s: &mut System, st: &[Station; MS_MAXLH], xg: &[f64]) {
	let (n, g) = (s.n, s.g);
	let stc = stations_at(st, xg);
	// sally: each station's six stepped poses, which the C works out again for every hit (stepped_station)
	let mut stp = [[Station::default(); 6]; MS_MAXLH];
	for l in 0..MS_MAXLH {
		if st[l].g >= 0 {
			let g0 = st[l].g as usize;
			stp[l] = stepped_station(&stc[l], &st[l].r, &st[l].p, &xg[g0..g0 + 6]);
		}
	}
	s.h.iter_mut().for_each(|v| *v = 0.0);
	s.gv = [0.0; MS_MAXG];
	s.c[..n * 6 * g].iter_mut().for_each(|v| *v = 0.0);
	for i in 0..n {
		let sn = &p[i];
		let (rs, ps) = apply_delta(&sn.r0, &sn.p0, &s.xs[i * 6..i * 6 + 6]);
		// sally: the snapshot's six stepped poses, once for all its hits (stepped)
		let sp6 = stepped(&sn.r0, &sn.p0, &s.xs[i * 6..i * 6 + 6]);
		let mut a = [0.0; 36];
		let mut b = [0.0; 6];
		for h in 0..sn.n {
			if s.active[i * MS_MAXH + h] == 0 {
				continue;
			}
			let l = sn.lh[h] as usize;
			let g0 = st[l].g;
			let r = sn.angle[h] - model_angle(sn, h, &rs, &ps, &stc[l]);
			let w = huber_w(r, ms.huber) / (ms.huber * ms.huber);
			let mut js = [0.0; 6];
			let mut jg = [0.0; 6];
			for k in 0..6 {
				let (r2, p2) = &sp6[k];
				js[k] = (sn.angle[h] - model_angle(sn, h, r2, p2, &stc[l]) - r) / 1e-6;
			}
			if g0 >= 0 {
				for k in 0..6 {
					jg[k] = (sn.angle[h] - model_angle(sn, h, &rs, &ps, &stp[l][k]) - r) / 1e-6;
				}
			}
			let cb = &mut s.c[i * 6 * g..(i + 1) * 6 * g];
			for ai in 0..6 {
				b[ai] += w * js[ai] * r;
				for c in 0..6 {
					a[ai * 6 + c] += w * js[ai] * js[c];
				}
				if g0 >= 0 {
					for c in 0..6 {
						cb[ai * g + g0 as usize + c] += w * js[ai] * jg[c];
					}
				}
			}
			if g0 >= 0 {
				let g0 = g0 as usize;
				for ai in 0..6 {
					s.gv[g0 + ai] += w * jg[ai] * r;
					for c in 0..6 {
						s.h[(g0 + ai) * g + g0 + c] += w * jg[ai] * jg[c];
					}
				}
			}
		}
		s.a[i * 36..i * 36 + 36].copy_from_slice(&a);
		s.b[i * 6..i * 6 + 6].copy_from_slice(&b);
	}
}

/// schur: the Schur complement of the station block, Sm = H + extra - sum C' (A + lambda diag A)^-1 C, and the right
/// hand side likewise. ai receives the damped block inverses. False if a block is singular (sm, rhs and ai are then
/// as far as the blocks before it took them).
pub fn schur(s: &System, lambda: f64, ai: &mut [f64], sm: &mut [f64], mut rhs: Option<&mut [f64]>) -> bool {
	let g = s.g;
	let mut t = vec![0.0; 6 * MS_MAXG];
	for i in 0..s.n {
		let mut ad = [0.0; 36];
		ad.copy_from_slice(&s.a[i * 36..i * 36 + 36]);
		for a in 0..6 {
			ad[a * 7] += lambda * ad[a * 7] + 1e-12;
		}
		let Some(inv) = inv6(&ad) else { return false };
		ai[i * 36..i * 36 + 36].copy_from_slice(&inv);
		let c = &s.c[i * 6 * g..(i + 1) * 6 * g];
		let ainv = &inv;
		let b = &s.b[i * 6..i * 6 + 6];
		let mut u = [0.0; 6];
		for a in 0..6 {
			u[a] = 0.0;
			for cc in 0..6 {
				u[a] += ainv[a * 6 + cc] * b[cc];
			}
			for k in 0..g {
				let mut v = 0.0;
				for cc in 0..6 {
					v += ainv[a * 6 + cc] * c[cc * g + k];
				}
				t[a * g + k] = v;
			}
		}
		for a in 0..g {
			let mut v = 0.0;
			for cc in 0..6 {
				v += c[cc * g + a] * u[cc];
			}
			if let Some(rhs) = rhs.as_deref_mut() {
				rhs[a] -= v;
			}
			for k in 0..g {
				let mut m = 0.0;
				for cc in 0..6 {
					m += c[cc * g + a] * t[cc * g + k];
				}
				sm[a * g + k] -= m;
			}
		}
	}
	true
}

/// median_of: the median of v (quickselect: v is reordered), NaN for none
pub fn median_of(v: &mut [f64]) -> f64 {
	let n = v.len();
	if n == 0 {
		return f64::NAN;
	}
	let k = (n / 2) as isize;
	let (mut lo, mut hi) = (0isize, n as isize - 1);
	while lo < hi {
		let piv = v[((lo + hi) / 2) as usize];
		let (mut i, mut j) = (lo, hi);
		while i <= j {
			while v[i as usize] < piv {
				i += 1;
			}
			while v[j as usize] > piv {
				j -= 1;
			}
			if i <= j {
				v.swap(i as usize, j as usize);
				i += 1;
				j -= 1;
			}
		}
		if k <= j {
			hi = j;
		} else if k >= i {
			lo = i;
		} else {
			break;
		}
	}
	v[k as usize]
}

/// warm_start: a pose-only fit of every snapshot with the stations st fixed (only stations with .used take part): the
/// seeds (filter poses) must not bias the first joint step; this also decides which hits take part (active, N x
/// MS_MAXH). xs (N x 6, zero on entry) receives the snapshot deltas.
pub fn warm_start(ms: &Settings, p: &[Snapshot], st: &[Station; MS_MAXLH], xs: &mut [f64], active: &mut [u8]) {
	let n = p.len();
	for i in 0..n {
		for h in 0..p[i].n {
			active[i * MS_MAXH + h] = st[p[i].lh[h] as usize].used as u8;
		}
	}
	for it in 0..4 {
		for i in 0..n {
			let s = &p[i];
			let mut jtj = [0.0; 36];
			let mut jtr = [0.0; 6];
			let (rs, ps) = apply_delta(&s.r0, &s.p0, &xs[i * 6..i * 6 + 6]);
			// sally: the six stepped poses, once for all the hits (xs moves only after them)
			let sp6 = stepped(&s.r0, &s.p0, &xs[i * 6..i * 6 + 6]);
			for h in 0..s.n {
				let l = s.lh[h] as usize;
				if !st[l].used {
					continue;
				}
				let r = s.angle[h] - model_angle(s, h, &rs, &ps, &st[l]);
				let act = &mut active[i * MS_MAXH + h];
				if it == 0 {
					*act = (r.abs() < 4.0 * ms.reject) as u8; // seeds can be a few mrad off
				}
				if it == 3 {
					*act = (r.abs() < ms.reject) as u8;
				}
				if *act == 0 {
					continue;
				}
				let mut j = [0.0; 6];
				for k in 0..6 {
					let (r2, p2) = &sp6[k];
					j[k] = (s.angle[h] - model_angle(s, h, r2, p2, &st[l]) - r) / 1e-6;
				}
				let w = huber_w(r, ms.huber);
				for a in 0..6 {
					jtr[a] -= w * j[a] * r;
					for c in 0..6 {
						jtj[a * 6 + c] += w * j[a] * j[c];
					}
				}
			}
			for a in 0..6 {
				jtj[a * 7] += 1e-9 + 1e-3 * jtj[a * 7];
			}
			if gsolve(&mut jtj, &mut jtr, 6) {
				for a in 0..6 {
					xs[i * 6 + a] += jtr[a];
				}
			}
		}
	}
}

/// What joint_solve returns besides the solved map
pub struct JointSolve {
	/// false if fewer than two stations have enough light in the batch
	pub ok: bool,
	/// the median |residual| of the hits taking part before (after the warm start) and after the solve
	pub med_before: f64,
	pub med_after: f64,
	/// the stations with enough light
	pub nst: i32,
}

/// joint_solve: one joint solve over the batch p, linearized at st (the solved map so far), with the station
/// information lam (MS_MAXG x MS_MAXG, index 6*bsd+k, tangent space at st) as prior and a weak prior toward start.
/// Writes the solved map into out and the information this batch adds into lam_add (same layout). st's .g are set
/// (-1 for a station not taking part), whatever the result.
#[allow(clippy::too_many_arguments)] // the C's signature
pub fn joint_solve(
	ms: &Settings, p: &[Snapshot], st: &mut [Station; MS_MAXLH], start: &[Station; MS_MAXLH], lam: &[f64],
	out: &mut [Station; MS_MAXLH], lam_add: &mut [f64],
) -> JointSolve {
	let n = p.len();
	let mut cnt = [0i32; MS_MAXLH];
	for i in 0..n {
		for h in 0..p[i].n {
			cnt[p[i].lh[h] as usize] += 1;
		}
	}
	let mut s = System {
		n,
		g: 0,
		gl: [0; MS_MAXG / 6],
		xs: Vec::new(),
		a: Vec::new(),
		b: Vec::new(),
		c: Vec::new(),
		active: Vec::new(),
		h: Vec::new(),
		gv: [0.0; MS_MAXG],
	};
	// every known station is an unknown of the solve: one without light in this batch is still held by what earlier
	// batches learned (Lam, which ties it to the others) and the weak prior, so a batch seen by two stations cannot move
	// that pair as a block relative to the rest
	let mut lit = 0;
	for l in 0..MS_MAXLH {
		st[l].g = if st[l].used { s.g as i32 } else { -1 };
		if st[l].g >= 0 {
			s.gl[s.g / 6] = l;
			s.g += 6;
		}
		lit += (st[l].used && cnt[l] >= ms.min_station_hits) as i32;
	}
	let g = s.g;
	let mut res = JointSolve { ok: false, med_before: f64::NAN, med_after: f64::NAN, nst: lit };
	if lit < 2 {
		return res;
	}

	s.xs = vec![0.0; n * 6];
	s.a = vec![0.0; n * 36];
	s.b = vec![0.0; n * 6];
	s.c = vec![0.0; n * 6 * g];
	s.active = vec![0u8; n * MS_MAXH];
	s.h = vec![0.0; g * g];
	let mut ai = vec![0.0; n * 36];
	let mut xs_try = vec![0.0; n * 6];
	let mut resv = vec![0.0; n * MS_MAXH];
	let mut xg = [0.0; MS_MAXG];
	let mut lam_s = vec![0.0; g * g];
	for a in 0..g {
		for c in 0..g {
			lam_s[a * g + c] = lam[(6 * s.gl[a / 6] + a % 6) * MS_MAXG + 6 * s.gl[c / 6] + c % 6];
		}
	}

	// pose-only warm start with the stations fixed: the seeds (filter poses) must not bias the first joint step;
	// this also decides which hits take part
	warm_start(ms, p, st, &mut s.xs, &mut s.active);
	let mut nres = 0;
	for i in 0..n {
		let sn = &p[i];
		let (rs, ps) = apply_delta(&sn.r0, &sn.p0, &s.xs[i * 6..i * 6 + 6]);
		for h in 0..sn.n {
			if s.active[i * MS_MAXH + h] != 0 {
				resv[nres] = (sn.angle[h] - model_angle(sn, h, &rs, &ps, &st[sn.lh[h] as usize])).abs();
				nres += 1;
			}
		}
	}
	res.med_before = median_of(&mut resv[..nres]);

	// joint Levenberg-Marquardt. residual r = meas - model, J = dr/dx: (J'WJ + priors) dx = -(J'Wr + prior gradient)
	let mut lambda = 1e-3;
	let mut cur = total_cost(ms, p, &s, &s.xs, st, &xg, start, &lam_s);
	let mut pd = [0.0; MS_MAXG];
	for a in 0..g {
		pd[a] = st[s.gl[a / 6]].prior_w
			* (if a % 6 < 3 { 1.0 / (ms.prior_pos * ms.prior_pos) } else { 1.0 / (ms.prior_rot * ms.prior_rot) });
	}
	let mut sm = vec![0.0; g * g];
	for _ in 0..ms.iters {
		assemble(ms, p, &mut s, st, &xg);
		let stc = stations_at(st, &xg);
		let mut gp = [0.0; MS_MAXG];
		for bl in 0..g / 6 {
			let dev = start_dev(&stc[s.gl[bl]], &start[s.gl[bl]]);
			for k in 0..6 {
				gp[6 * bl + k] = pd[6 * bl + k] * dev[k];
			}
		}
		for a in 0..g {
			for c in 0..g {
				gp[a] += lam_s[a * g + c] * xg[c];
			}
		}
		let mut accepted = false;
		let mut tries = 0;
		while tries < 8 && !accepted {
			let mut rhs = [0.0; MS_MAXG];
			for a in 0..g {
				rhs[a] = s.gv[a] + gp[a];
				for c in 0..g {
					sm[a * g + c] = s.h[a * g + c] + lam_s[a * g + c];
				}
				sm[a * g + a] += pd[a];
				sm[a * g + a] *= 1.0 + lambda;
			}
			if !schur(&s, lambda, &mut ai, &mut sm, Some(&mut rhs[..g])) {
				break;
			}
			for a in 0..g {
				rhs[a] = -rhs[a];
			}
			if !gsolve(&mut sm, &mut rhs[..g], g) {
				break;
			}
			let mut xg_try = [0.0; MS_MAXG];
			for a in 0..g {
				xg_try[a] = xg[a] + rhs[a];
			}
			for i in 0..n {
				let c = &s.c[i * 6 * g..(i + 1) * 6 * g];
				let ainv = &ai[i * 36..i * 36 + 36];
				let b = &s.b[i * 6..i * 6 + 6];
				let mut t = [0.0; 6];
				for cc in 0..6 {
					t[cc] = b[cc];
					for k in 0..g {
						t[cc] += c[cc * g + k] * rhs[k];
					}
				}
				for a in 0..6 {
					let mut v = 0.0;
					for cc in 0..6 {
						v += ainv[a * 6 + cc] * t[cc];
					}
					xs_try[i * 6 + a] = s.xs[i * 6 + a] - v;
				}
			}
			let nc = total_cost(ms, p, &s, &xs_try, st, &xg_try, start, &lam_s);
			if nc < cur {
				cur = nc;
				s.xs.copy_from_slice(&xs_try);
				xg[..g].copy_from_slice(&xg_try[..g]);
				lambda = if lambda / 3.0 > 1e-7 { lambda / 3.0 } else { 1e-7 };
				accepted = true;
			} else {
				lambda *= 5.0;
			}
			tries += 1;
		}
		if !accepted {
			break;
		}
	}
	*out = stations_at(st, &xg);
	nres = 0;
	for i in 0..n {
		let sn = &p[i];
		let (rs, ps) = apply_delta(&sn.r0, &sn.p0, &s.xs[i * 6..i * 6 + 6]);
		for h in 0..sn.n {
			if s.active[i * MS_MAXH + h] != 0 {
				resv[nres] = (sn.angle[h] - model_angle(sn, h, &rs, &ps, &out[sn.lh[h] as usize])).abs();
				nres += 1;
			}
		}
	}
	res.med_after = median_of(&mut resv[..nres]);

	// what this batch says about the stations: data-only Schur complement at the solution
	assemble(ms, p, &mut s, st, &xg);
	{
		sm.copy_from_slice(&s.h[..g * g]);
		lam_add.iter_mut().for_each(|v| *v = 0.0);
		if schur(&s, 0.0, &mut ai, &mut sm, None) {
			for a in 0..g {
				for c in 0..g {
					lam_add[(6 * s.gl[a / 6] + a % 6) * MS_MAXG + 6 * s.gl[c / 6] + c % 6] =
						0.5 * (sm[a * g + c] + sm[c * g + a]);
				}
			}
		}
	}
	res.ok = true;
	res
}
