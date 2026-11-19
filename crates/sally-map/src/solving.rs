//! The solves (driver_map_solve.c's run_solve and solve_thread): one solve of the batch, and where it runs. Inline, in
//! the imupose call that asks for it, as the C solves with map-solve-threaded 0 or at playback factor 0; or on the
//! C's solve thread, which that call only wakes.
//!
//! A solve's state is split as the threads need it. What the hooks keep (the poses in use and the solved map, the
//! calibrated anchors, what earlier solves learned, the counts) is the `MapSolve`, which a threaded solve reaches only
//! with the context lock, through `Locked::locked`; with the runtime it reads the stations and devices from and
//! writes the poses in use and the recording to. The snapshots since the last solve are the batch, behind batch_lock,
//! which the hooks add to and the solve takes. The rest is the solve's own (`Solver`: the C's fields only run_solve
//! touches, and its static Lam), locked by whoever solves.
//!
//! run_solve takes the context lock where the C's does when it solves on its thread: to read the stations in use and
//! the map as it starts; around each of a moved station's decisions, the MAPSTN and MAPMOVE lines and the log lines
//! (MS_LOCKED); for the stations' moved flags; and to apply a solve, or record a skipped one. Inline the lock is the
//! hook's already, and each of those is a call at once: the same reads and writes in the same order as before. Where
//! the C reads without the lock something only the lock keeps, the Rust takes it: the stations' channels (dump_debug's,
//! and the summary of the applied solve's, built under the lock that applies it) and the anchors (read as the solve
//! starts: only the solve changes them). The bound options are read once, as a solve starts, where the C's reads each
//! field as it uses it: a live set during a solve on the thread reaches the next one.

use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::JoinHandle;

use sally_lighthouse::BaseStationCal;
use sally_math::printf::{sprintf, Arg};
use sally_math::{c_exp, norm3d, Pose};

use crate::host::Host;
use crate::linalg::{mm3, mv3, q2r, r2q, r2rv, transpose3};
use crate::map_solve::MapSolve;
use crate::model::{Snapshot, Station, MS_MAXG, MS_MAXLH};
use crate::relocate::{pose_gap, relocate_station, rigid_fit, station_check};
use crate::settings::Settings;
use crate::solve::joint_solve;
use crate::text::{cut, info};

/// MS_POOL_MAX: most snapshots a pending station's pool keeps
const MS_POOL_MAX: usize = 240;

/// What a solve reaches with the context lock, and what it reads without
pub trait Locked {
	/// The bound options as their variables hold them now, taken as a solve starts
	fn settings(&mut self) -> Settings;
	/// OGRelativeTime: the wall clock, for the time a solve took (in its log line)
	fn relative_time(&mut self) -> f64;
	/// `f` with the context locked: the map solve's hooks' side and the runtime. None when the map solve is gone, which
	/// it never is while a solve runs (survive_close joins the thread before it closes the map solve)
	fn locked<R>(&mut self, f: impl FnOnce(&mut MapSolve, &mut dyn Host) -> R) -> Option<R>;
}

/// A solve in the imupose call that asks for it: the context lock is the hook's already
struct Inline<'a, H: Host> {
	ms: &'a mut MapSolve,
	host: &'a mut H,
}

impl<H: Host> Locked for Inline<'_, H> {
	fn settings(&mut self) -> Settings {
		self.ms.s.clone()
	}
	fn relative_time(&mut self) -> f64 {
		self.host.relative_time()
	}
	fn locked<R>(&mut self, f: impl FnOnce(&mut MapSolve, &mut dyn Host) -> R) -> Option<R> {
		Some(f(&mut *self.ms, &mut *self.host))
	}
}

/// A lock that a panic would have left poisoned is taken all the same (a panic aborts sally anyway)
pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
	m.lock().unwrap_or_else(|e| e.into_inner())
}

/// run_solve's own state: the fields of the C's map_solve only the solve reads and writes, and its static Lam
pub(crate) struct Solver {
	/// the batch the hooks fill (MapSolve::batch: the same, behind batch_lock)
	batch: Arc<Mutex<Vec<Snapshot>>>,
	/// the solver's copy of the batch
	work: Vec<Snapshot>,
	/// moved stations (relocate): flagged and waiting for a confirmation
	moved_pending: [bool; MS_MAXLH],
	/// where the map had a pending station before its provisional pose was put in use
	pend_old: [Pose; MS_MAXLH],
	/// a pending station's snapshots from the batches since it was flagged
	pool: Vec<Vec<Snapshot>>,
	pending_pose: [Pose; MS_MAXLH],
	pending_time: [f64; MS_MAXLH],
	pub(crate) solves: i32,
	/// moved-station flags
	pub(crate) suspects: i32,
}

impl Solver {
	pub(crate) fn new(batch: Arc<Mutex<Vec<Snapshot>>>, batch_max: usize) -> Solver {
		Solver {
			batch,
			work: Vec::with_capacity(batch_max),
			moved_pending: [false; MS_MAXLH],
			pend_old: [Pose::default(); MS_MAXLH],
			pool: (0..MS_MAXLH).map(|_| Vec::new()).collect(),
			pending_pose: [Pose::default(); MS_MAXLH],
			pending_time: [0.0; MS_MAXLH],
			solves: 0,
			suspects: 0,
		}
	}

	/// pool_add: keep the snapshots of the batch that station l lit (for a pending station's pooled solve); reset first
	/// if asked
	fn pool_add(&mut self, l: usize, reset: bool, s: &Settings) {
		if reset {
			self.pool[l].clear();
		}
		for i in 0..self.work.len() {
			let mut own = 0;
			for h in 0..self.work[i].n {
				own += (self.work[i].lh[h] as usize == l) as i32;
			}
			if own < s.min_hits {
				continue;
			}
			if self.pool[l].len() >= MS_POOL_MAX {
				// keep the newest: drop the oldest half
				self.pool[l].drain(..MS_POOL_MAX / 2);
			}
			let snap = self.work[i].clone();
			self.pool[l].push(snap);
		}
	}

	/// apply_relocation (with the context locked): station l put where a confirmed relocation found it: at once (the
	/// old place is wrong, slewing would keep the tracker on it for many seconds), with what earlier solves learned about
	/// it forgotten (it was about the old place). It stops anchoring the frame: its calibrated position says nothing any
	/// more, and its relocation is only as good as one or two batches (a bump that is partly a pivot leaves millimeters
	/// along the pivot), so the frame hold would turn the whole map by a few mrad toward it. The frame is then held by
	/// the other stations' calibrated positions alone, which also takes back whatever the moved station bent before it
	/// was caught (re-expressing the remaining anchors so the frame stays exactly where it was kept that bend: 3.4 mrad
	/// on a bump present from the start).
	fn apply_relocation(&mut self, ms: &mut MapSolve, host: &mut (impl Host + ?Sized), l: usize, np: &Pose) {
		let mut n = 0;
		let mut k = 0;
		while k < MS_MAXLH && (k as i32) < host.active_lighthouses() {
			n += (k != l && ms.have_start[k] && !ms.unanchored[k]) as i32;
			k += 1;
		}
		if n >= 3 {
			ms.unanchored[l] = true;
		} else {
			let mode = host.bsd_mode(l);
			info(
				host,
				"map-solve: LH ch %d relocated, but fewer than three other stations anchor the frame: it keeps anchoring it",
				&[Arg::I(mode as i64)],
			);
		}
		ms.start[l] = *np;
		ms.target[l] = *np;
		ms.have_target[l] = true;
		for k in 0..6 {
			for a in 0..MS_MAXG {
				ms.lam[a * MS_MAXG + 6 * l + k] = 0.0;
				ms.lam[(6 * l + k) * MS_MAXG + a] = 0.0;
			}
		}
		self.moved_pending[l] = false;
		ms.relocations += 1;
		host.set_bsd_sally_moved(l, 2);
		let t = host.run_time();
		host.set_bsd_sally_moved_time(l, t);
		host.raw_lighthouse_pose(l, np);
	}

	/// dump_debug: SALLY_MAPSOLVE_DUMP=<file> (the environment, read at every solve as the C reads it): the batch and
	/// the map of every solve (the last one stays), for the offline checks
	fn dump_debug(&self, ctx: &mut impl Locked, st: &[Station; MS_MAXLH], start: &[Station; MS_MAXLH]) {
		let Some(path) = std::env::var_os("SALLY_MAPSOLVE_DUMP") else { return };
		let Ok(mut f) = std::fs::File::create(path) else { return };
		// the stations' channels (which the C reads without the lock)
		let Some(modes) = ctx.locked(|_, host| {
			let mut modes = [0u8; MS_MAXLH];
			for l in 0..MS_MAXLH {
				if st[l].used {
					modes[l] = host.bsd_mode(l);
				}
			}
			modes
		}) else {
			return;
		};
		let mut t = String::new();
		let g = |v: f64| sprintf(" %.17g", &[Arg::F(v)]);
		for l in 0..MS_MAXLH {
			if st[l].used {
				t.push_str(&sprintf("ST %d %d", &[Arg::I(l as i64), Arg::I(modes[l] as i64)]));
				st[l].r.iter().chain(&st[l].p).chain(&start[l].r).chain(&start[l].p).for_each(|&v| t.push_str(&g(v)));
				for a in 0..2 {
					let c: &BaseStationCal = &st[l].fcal[a];
					for v in [c.phase, c.tilt, c.curve, c.gibpha, c.gibmag, c.ogeephase, c.ogeemag] {
						t.push_str(&g(v));
					}
				}
				t.push('\n');
			}
		}
		for sp in &self.work {
			t.push_str(&sprintf("SN %d %.9f", &[Arg::I(sp.dev as i64), Arg::F(sp.time)]));
			sp.r0.iter().chain(&sp.p0).chain(&sp.v).chain(&sp.w).for_each(|&v| t.push_str(&g(v)));
			t.push_str(&sprintf(" %d\n", &[Arg::I(sp.n as i64)]));
			for h in 0..sp.n {
				t.push_str(&sprintf(
					"H %d %d %.17g %.17g %.17g %.17g %.17g\n",
					&[
						Arg::I(sp.lh[h] as i64),
						Arg::I(sp.plane[h] as i64),
						Arg::F(sp.angle[h]),
						Arg::F(sp.dt[h]),
						Arg::F(sp.pt[h][0]),
						Arg::F(sp.pt[h][1]),
						Arg::F(sp.pt[h][2]),
					],
				));
			}
		}
		use std::io::Write;
		let _ = f.write_all(&sally_math::textmode::file(t.as_bytes()));
	}

	/// run_solve: one solve of the batch, if it has map-solve-min-snaps snapshots: the moved-station check and the
	/// relocations, the joint solve, the frame hold, and the result applied (or not: it moved a station too far), with
	/// the context lock taken where the C's takes it (see the module). None when the map solve went away mid-solve
	fn run(&mut self, ctx: &mut impl Locked) -> Option<()> {
		let s = ctx.settings();
		// batch_lock: the batch, consumed (what it says about the map is kept in the information matrix)
		let n_snaps = {
			let mut batch = lock(&self.batch);
			let n = batch.len();
			if n as i32 >= s.min_snaps {
				std::mem::swap(&mut self.work, &mut *batch);
				batch.clear();
			}
			n
		};
		if (n_snaps as i32) < s.min_snaps {
			return Some(());
		}

		let mut st = [Station::default(); MS_MAXLH];
		let mut start = [Station::default(); MS_MAXLH];
		let mut out = [Station::default(); MS_MAXLH];
		// the stations in use and the calibrated map, what earlier solves learned (faded), the stations that no longer
		// anchor the frame
		let (now, lam, unanchored) = ctx.locked(|ms, host| {
			let now = host.run_time();
			let mut l = 0;
			while l < MS_MAXLH && (l as i32) < host.active_lighthouses() {
				if !host.bsd_position_set(l) || !host.bsd_ootx_set(l) || !ms.have_start[l] {
					l += 1;
					continue;
				}
				let cur = if ms.have_target[l] { ms.target[l] } else { host.bsd_pose(l) };
				st[l].r = q2r(&cur.rot);
				st[l].p = cur.pos;
				st[l].set_fcal(host.bsd_fcal(l));
				st[l].used = true;
				st[l].prior_w = if ms.unanchored[l] { 1e-6 } else { 1.0 }; // keeps an unlit relocated station regular, nothing more
				start[l].r = q2r(&ms.start[l].rot);
				start[l].p = ms.start[l].pos;
				l += 1;
			}
			// fade what earlier solves learned
			let fade = if s.tau > 0.0 && now > ms.lam_time { c_exp(-(now - ms.lam_time) / s.tau) } else { 1.0 };
			let lam: Vec<f64> = ms.lam.iter().map(|&v| v * fade).collect();
			(now, lam, ms.unanchored)
		})?;
		let mut lam_add = vec![0.0; MS_MAXG * MS_MAXG];

		self.dump_debug(ctx, &st, &start);
		let t0 = ctx.relative_time();

		// moved stations: one whose light disagrees with the map while the others agree is left out of this solve
		// (its hits would only be rejected, or drag the others) and relocated on its own below
		let mut suspect = [false; MS_MAXLH];
		let mut excl = [false; MS_MAXLH];
		let mut reloc = [false; MS_MAXLH];
		let mut reloc_pose = [Pose::default(); MS_MAXLH];
		let mut som = [f64::NAN; MS_MAXLH];
		if s.relocate != 0 {
			let chk = station_check(&s, &self.work, &st);
			let (smed, snh, sns, srest) = (chk.med, chk.nh, chk.ns, chk.rest);
			for l in 0..MS_MAXLH {
				som[l] = f64::NAN;
				if self.moved_pending[l] && now - self.pending_time[l] > s.move_pending_max {
					self.moved_pending[l] = false;
					let map_place = self.pend_old[l];
					// unconfirmed: back to the map's place
					ctx.locked(|ms, host| ms.set_in_use(host, l, &map_place))?;
					st[l].r = q2r(&map_place.rot);
					st[l].p = map_place.pos;
					self.pool[l].clear();
					ctx.locked(|_, host| {
						let mode = host.bsd_mode(l);
						info(
							host,
							"map-solve: LH ch %d: relocation not confirmed within %.0f s, dropped",
							&[Arg::I(mode as i64), Arg::F(s.move_pending_max)],
						);
					})?;
				}
				if smed[l].is_nan() || srest[l].is_nan() || sns[l] < s.move_min_snaps {
					continue;
				}
				// som: how well the rest agrees when another station is left out instead (the best such)
				let mut no = 0;
				for k in 0..MS_MAXLH {
					if k != l && !srest[k].is_nan() && sns[k] >= s.move_min_snaps {
						if no == 0 || srest[k] < som[l] {
							som[l] = srest[k];
						}
						no += 1;
					}
				}
				if no < 2 {
					continue;
				}
				suspect[l] = smed[l] > s.move_resid
					&& smed[l] > s.move_ratio * srest[l]
					&& som[l] > s.move_rest_ratio * srest[l];
			}
			for l in 0..MS_MAXLH {
				if st[l].used && (suspect[l] || self.moved_pending[l]) {
					excl[l] = true;
					st[l].used = false; // relocate_station still reads its pose
				}
			}
			// per station: channel, leave-one-out median residual (mrad), hits, snapshots, the rest's residual (mrad)
			let solves = self.solves;
			ctx.locked(|_, host| {
				let mut sl = String::new();
				let mut so = 0usize;
				let mut l = 0;
				while l < MS_MAXLH && so < 400 - 40 {
					if snh[l] > 0 {
						let mode = host.bsd_mode(l);
						let e = sprintf(
							" %d %.3f %d %d %.3f",
							&[
								Arg::I(mode as i64),
								Arg::F(smed[l] * 1e3),
								Arg::I(snh[l] as i64),
								Arg::I(sns[l] as i64),
								Arg::F(srest[l] * 1e3),
							],
						);
						so += e.len();
						sl.push_str(&e);
					}
					l += 1;
				}
				let sl = cut(sl, 400);
				host.record(&sprintf("MAPSTN %d%s\n", &[Arg::I(solves as i64 + 1), Arg::S(&sl)]));
			})?;
			// each left-out station: relocate it from the others' snapshot poses, and decide (applied after this
			// solve) (the diagnosis shows a left-out station as disagreeing with the map until it is relocated or
			// agrees again)
			for l in 0..MS_MAXLH {
				if !excl[l] {
					continue;
				}
				let pend = self.moved_pending[l];
				if suspect[l] {
					self.suspects += 1;
				}
				// a pending station's pose in use (st[l]) is already its provisional new place; pst is where the map
				// had it
				let mut pst = st[l];
				if pend {
					pst.r = q2r(&self.pend_old[l].rot);
					pst.p = self.pend_old[l].pos;
				}
				let (mut dp, mut dr, mut medp) = (f64::NAN, f64::NAN, f64::NAN);
				// this batch alone: mcur at the pose in use, malt at the map's old place (pending only)
				let rel = relocate_station(
					&s,
					&self.work,
					&st,
					l,
					Station::default(),
					f64::NAN,
					f64::NAN,
					if pend { Some(&pst) } else { None },
					f64::NAN,
				);
				let (rok, nl, med2, nsn, nhh, mcur, malt) =
					(rel.ok, rel.out, rel.med_after, rel.nsnap, rel.nhit, rel.med_cur, rel.med_alt);
				// the new place has to explain its light as well as the others explain theirs
				let fits = rok && med2 < s.move_fit;
				let mut np = Pose::default();
				let mut pp = Pose::default();
				let at = if pend { &pst } else { &st[l] };
				let cp = Pose { pos: at.p, rot: r2q(&at.r) };
				if fits {
					np = Pose { pos: nl.p, rot: r2q(&nl.r) };
					(dp, dr) = pose_gap(&np, &cp);
				}
				// pending: one place solved from every batch since the flag; if it explains them all, that is the move
				let mut pok = false;
				if pend && rok {
					let mut u: Vec<Snapshot> = self.pool[l].clone();
					u.extend(self.work.iter().cloned());
					let pr = relocate_station(&s, &u, &st, l, Station::default(), medp, f64::NAN, None, f64::NAN);
					medp = pr.med_after;
					pok = pr.ok && medp < s.move_fit;
					if pok {
						pp = Pose { pos: pr.out.p, rot: r2q(&pr.out.r) };
					}
				}
				let solves = self.solves;
				let state = ctx.locked(|ms, host| {
					let mut state = "unsolved";
					if pend && rok {
						// a station already in use at its provisional new place: this batch decides. Back to the map
						// when the map explains its light again; confirmed when one place explains every batch since
						// the flag (a move that is partly a pivot is pinned down by light from more than one part of
						// the room); otherwise a better guess from this batch replaces the provisional one
						if malt < 2.0 * s.move_fit && malt < 0.8 * mcur {
							self.moved_pending[l] = false;
							let map_place = self.pend_old[l];
							ms.set_in_use(host, l, &map_place);
							// the joint solve below takes it back in: from the map's place, not the provisional one
							// st[l] was loaded from (its information would hold it there and the solve would put it
							// back)
							st[l].r = q2r(&map_place.rot);
							st[l].p = map_place.pos;
							self.pool[l].clear();
							state = "dropped";
						} else if pok {
							(dp, dr) = pose_gap(&pp, &cp);
							reloc[l] = true;
							reloc_pose[l] = pp;
							self.pool[l].clear();
							state = "applied";
						} else if fits && med2 < 0.5 * mcur {
							self.pending_pose[l] = np;
							self.pending_time[l] = now;
							ms.set_in_use(host, l, &np);
							self.pool_add(l, true, &s);
							state = "replaced";
						} else {
							self.pool_add(l, false, &s);
							state = "undecided";
						}
					} else if !pend && fits && dp < s.move_min_pos && dr < s.move_min_rot {
						state = "small";
					} else if !pend && fits {
						// flagged: the tracker gets the new place now (a 5 mrad station error costs centimetres while
						// this waits for confirmation); rolled back if the next batches say the map was right
						self.moved_pending[l] = true;
						self.pending_pose[l] = np;
						self.pending_time[l] = now;
						self.pend_old[l] = cp;
						ms.set_in_use(host, l, &np);
						self.pool_add(l, true, &s);
						state = "pending";
					} else if pend {
						state = "unseen";
					}
					let mode = host.bsd_mode(l);
					host.record(&sprintf(
						"MAPMOVE %d %d %s %.6f %.6f %.6f %.6f %.6f %.6f %d %d %.2f %.3f\n",
						&[
							Arg::I(solves as i64 + 1),
							Arg::I(mode as i64),
							Arg::S(state),
							Arg::F(smed[l]),
							Arg::F(srest[l]),
							Arg::F(som[l]),
							Arg::F(mcur),
							Arg::F(malt),
							Arg::F(med2),
							Arg::I(nsn as i64),
							Arg::I(nhh as i64),
							Arg::F(dp * 1e3),
							Arg::F(dr * 1e3),
						],
					));
					state
				})?;
				if state == "small" || state == "dropped" {
					excl[l] = false; // the map is right about it, or close enough for the normal solve
					st[l].used = true;
				}
				ctx.locked(|_, host| {
					let mode = host.bsd_mode(l);
					info(
						host,
						"map-solve %d: LH ch %d %s (its light %.2f mrad off, the others agreeing to %.3f, %.3f without \
						 another station; in use %.2f, the map's old place %.2f, solved here %.2f mrad (from all batches \
						 since the flag %.2f), %.1f mm / %.2f mrad from the map's old place; %d snapshots, %d hits)",
						&[
							Arg::I(solves as i64 + 1),
							Arg::I(mode as i64),
							Arg::S(state),
							Arg::F(smed[l] * 1e3),
							Arg::F(srest[l] * 1e3),
							Arg::F(som[l] * 1e3),
							Arg::F(mcur * 1e3),
							Arg::F(malt * 1e3),
							Arg::F(med2 * 1e3),
							Arg::F(medp * 1e3),
							Arg::F(dp * 1e3),
							Arg::F(dr * 1e3),
							Arg::I(nsn as i64),
							Arg::I(nhh as i64),
						],
					);
				})?;
			}
		}
		if s.relocate != 0 {
			ctx.locked(|_, host| {
				let mut l = 0;
				while l < MS_MAXLH && (l as i32) < host.active_lighthouses() {
					if excl[l] && !reloc[l] {
						host.set_bsd_sally_moved(l, 1);
					} else if host.bsd_sally_moved(l) == 1 && !reloc[l] {
						host.set_bsd_sally_moved(l, 0);
					}
					l += 1;
				}
			})?;
		}
		let js = joint_solve(&s, &self.work, &mut st, &start, &lam, &mut out, &mut lam_add);
		let (ok, mb, ma, nst) = (js.ok, js.med_before, js.med_after, js.nst);
		let took = ctx.relative_time() - t0;
		self.solves += 1;
		if !ok {
			return ctx.locked(|ms, host| {
				ms.verbose(
					host,
					10,
					"map-solve: solve %d skipped (%d snapshots, %d stations with enough light)",
					&[Arg::I(self.solves as i64), Arg::I(n_snaps as i64), Arg::I(nst as i64)],
				);
				for l in 0..MS_MAXLH {
					if reloc[l] {
						let p = reloc_pose[l];
						self.apply_relocation(ms, host, l, &p);
					}
				}
			});
		}
		// hold the frame: remove the rigid motion of the solved stations relative to the calibrated map
		let mut from: Vec<[f64; 3]> = Vec::with_capacity(MS_MAXLH);
		let mut to: Vec<[f64; 3]> = Vec::with_capacity(MS_MAXLH);
		let mut idx: Vec<usize> = Vec::with_capacity(MS_MAXLH);
		// the frame is held by the stations that still anchor it (not relocated), three at least; a station left out
		// of this solve (moved) still takes part as it is, so leaving it out does not move the frame
		let mut nanch = 0;
		for l in 0..MS_MAXLH {
			nanch += ((st[l].g >= 0 || excl[l]) && !unanchored[l]) as i32;
		}
		let anch_only = nanch >= 3;
		for l in 0..MS_MAXLH {
			if st[l].g >= 0 {
				idx.push(l);
				if anch_only && unanchored[l] {
					continue;
				}
				from.push(out[l].p);
				to.push(start[l].p);
			}
		}
		for l in 0..MS_MAXLH {
			if excl[l] && !(anch_only && unanchored[l]) {
				// a pending station is in use at its provisional place; for the frame it still sits where the map had
				// it
				from.push(if self.moved_pending[l] { self.pend_old[l].pos } else { st[l].p });
				to.push(start[l].p);
			}
		}
		if let Some((rg, tg)) = rigid_fit(&from, &to) {
			for &l in &idx {
				let p = mv3(&rg, &out[l].p);
				for k in 0..3 {
					out[l].p[k] = p[k] + tg[k];
				}
				out[l].r = mm3(&rg, &out[l].r);
			}
		}
		// the result, with the lock that applies it (or not: it moved a station too far)
		ctx.locked(|ms, host| {
			let mut apply = true;
			let mut msg = String::new();
			for &l in &idx {
				let (mut dp, mut sp) = (0.0, 0.0);
				for k in 0..3 {
					dp += (out[l].p[k] - st[l].p[k]) * (out[l].p[k] - st[l].p[k]);
					sp += (out[l].p[k] - start[l].p[k]) * (out[l].p[k] - start[l].p[k]);
				}
				let dp = dp.sqrt();
				let rt = transpose3(&st[l].r);
				let dr = norm3d(&r2rv(&mm3(&out[l].r, &rt)));
				// sally: a move that is not a number is not applied either (the C's dp > max || dr > max let a NaN map
				// through, to the tracker and the map file)
				if !(dp <= s.max_move && dr <= s.max_rot) {
					apply = false;
				}
				let mode = host.bsd_mode(l);
				msg.push_str(&sprintf(
					" LH%d %.1fmm/%.2fmrad (calib %.1fmm, info %.0f)",
					&[
						Arg::I(mode as i64),
						Arg::F(dp * 1e3),
						Arg::F(dr * 1e3),
						Arg::F(sp.sqrt() * 1e3),
						Arg::F(lam[(6 * l) * MS_MAXG + 6 * l] + lam_add[(6 * l) * MS_MAXG + 6 * l]),
					],
				));
				msg = cut(msg, 600);
			}
			if apply {
				for &l in &idx {
					ms.target[l] = Pose { pos: out[l].p, rot: r2q(&out[l].r) };
					ms.have_target[l] = true;
				}
				for i in 0..MS_MAXG * MS_MAXG {
					ms.lam[i] = lam[i] + lam_add[i];
				}
				ms.lam_time = now;
				ms.applied += 1;
				if s.save_period > 0.0 && now - ms.last_save >= s.save_period {
					ms.last_save = now;
					ms.save_map_file(host, true);
				}
			} else {
				ms.rejected_moves += 1;
			}
			for l in 0..MS_MAXLH {
				if reloc[l] {
					let p = reloc_pose[l];
					self.apply_relocation(ms, host, l, &p); // after the information update, which would undo its reset
				}
			}
			host.record(&sprintf(
				"MAPSOLVE %d %d %d %.6f %.6f %d%s\n",
				&[
					Arg::I(self.solves as i64),
					Arg::I(n_snaps as i64),
					Arg::I(idx.len() as i64),
					Arg::F(mb),
					Arg::F(ma),
					Arg::I(apply as i64),
					Arg::S(&msg),
				],
			));
			ms.verbose(
				host,
				if apply { 10 } else { 0 },
				"map-solve %d: %d snapshots, %d stations, residual %.3f -> %.3f mrad, %.2f s,%s%s",
				&[
					Arg::I(self.solves as i64),
					Arg::I(n_snaps as i64),
					Arg::I(idx.len() as i64),
					Arg::F(mb * 1e3),
					Arg::F(ma * 1e3),
					Arg::F(took),
					Arg::S(&msg),
					Arg::S(if apply { "" } else { "  NOT APPLIED (moves too far)" }),
				],
			);
		})
	}
}

impl MapSolve {
	/// run_solve inline, in the imupose call that asks for it
	pub(crate) fn solve_inline(&mut self, host: &mut impl Host) {
		let solver = self.solver.clone();
		let mut solver = lock(&solver);
		let _ = solver.run(&mut Inline { ms: self, host });
	}
}

/// cv_lock's state: a solve asked for (pending), the thread wanted (active)
struct WakeState {
	active: bool,
	pending: bool,
}

/// cv_lock and cv
struct Wake {
	state: Mutex<WakeState>,
	cv: Condvar,
}

/// The solve thread (ms->threaded), from the hooks' side
pub(crate) struct SolveThread {
	wake: Arc<Wake>,
	thread: Option<JoinHandle<()>>,
}

impl SolveThread {
	/// OGCreateThread(solve_thread, "map-solve", ms), active: it waits for its first solve
	pub(crate) fn start(solver: Arc<Mutex<Solver>>, locked: impl Locked + Send + 'static) -> SolveThread {
		let wake = Arc::new(Wake { state: Mutex::new(WakeState { active: true, pending: false }), cv: Condvar::new() });
		let thread = {
			let wake = wake.clone();
			sally_os::create_thread("map-solve", move || solve_thread(&wake, &solver, locked))
		};
		SolveThread { wake, thread }
	}

	/// imupose_fn's request: pending set and the cv signalled, under cv_lock
	pub(crate) fn wake(&self) {
		let mut w = lock(&self.wake.state);
		w.pending = true;
		self.wake.cv.notify_one();
	}

	/// For the close: active cleared and the cv signalled under cv_lock, then the thread handed back to be joined
	/// without the context lock
	pub(crate) fn stop(&mut self) -> Option<JoinHandle<()>> {
		{
			let mut w = lock(&self.wake.state);
			w.active = false;
			self.wake.cv.notify_one();
		}
		self.thread.take()
	}
}

/// solve_thread: while active, wait for a solve to be asked for, and run it (cv_lock let go while it runs)
fn solve_thread(wake: &Wake, solver: &Mutex<Solver>, mut locked: impl Locked) {
	let mut w = lock(&wake.state);
	while w.active {
		while w.active && !w.pending {
			w = wake.cv.wait(w).unwrap_or_else(|e| e.into_inner());
		}
		if !w.active {
			break;
		}
		w.pending = false;
		drop(w);
		let _ = lock(solver).run(&mut locked);
		w = lock(&wake.state);
	}
}
