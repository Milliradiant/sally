//! The map file (driver_map_solve.c's load_map_file and save_map_file): the solved map and its information, written at
//! exit (and every map-solve-save-period s after an applied solve) and read at the next start, by station id.
//!
//! ```text
//! LH <id> x y z qw qx qy qz            solved pose (lh2world, config frame)
//! LAM <id_a> <id_b> 36 values         information block between two stations (tangent space at the solved poses)
//! ANCHOR <id> <cal x y z qw qx qy qz> <anchor x y z qw qx qy qz> <0|1>
//!                                     only once a station was relocated: the frame anchor of every station, the
//!                                     calibration it was derived from, and whether the station still anchors the
//!                                     frame (0 = relocated). Used only while the calibration is still the same.
//! ```

use sally_math::printf::{sprintf, Arg};
use sally_math::{c_strtod, Pose};

use crate::corrections::read_text_file;
use crate::host::Host;
use crate::map_solve::MapSolve;
use crate::model::{MS_MAXG, MS_MAXLH};
use crate::relocate::pose_gap;
use crate::text::{cut, info, warn};
use sally_math::scanf::{c_str, fgets_lines, sscanf, Value};

/// The doubles of a sscanf result from `from` on
fn doubles<const N: usize>(v: &[Value], from: usize) -> [f64; N] {
	let mut out = [0.0; N];
	for k in 0..N {
		let Value::Lf(x) = v[from + k] else { unreachable!() };
		out[k] = x;
	}
	out
}

/// A pose from 7 doubles: x y z qw qx qy qz
fn pose_of(d: &[f64]) -> Pose {
	Pose { pos: [d[0], d[1], d[2]], rot: [d[3], d[4], d[5], d[6]] }
}

/// sally: every value finite. A line with a nan or an inf (a damaged file: strtod reads both, and 1e400 as inf) is
/// left out with a warning; the C took it, and its NaN went to the tracker, into every later solve and back into the
/// file
fn finite(d: &[f64]) -> bool {
	d.iter().all(|v| v.is_finite())
}

/// sally: SV_WARN for a saved line station l would have taken, left out (not finite)
fn left_out(host: &mut (impl Host + ?Sized), what: &str, l: usize) {
	let mode = host.bsd_mode(l);
	warn(
		host,
		"map-solve: the saved map's %s line of LH ch %d holds a value that is not finite: left out",
		&[Arg::S(what), Arg::I(mode as i64)],
	);
}

impl MapSolve {
	/// load_map_file: the saved map, for the stations ready for the first time (a station ready later must not have
	/// the file replace what the live solves have learned about the others since, and a pair's information is loaded
	/// once, when its second station is ready)
	pub(crate) fn load_map_file(&mut self, host: &mut (impl Host + ?Sized)) {
		let mut fname = host.configs("map-solve-test-load", "");
		if fname.is_empty() {
			fname = host.configs("map-solve-file", "");
		}
		if fname.is_empty() {
			return;
		}
		let Some(bytes) = read_text_file(&fname) else {
			info(host, "map-solve: no saved map at %s yet; starting from the calibrated map", &[Arg::S(&fname)]);
			return;
		};
		let (mut n, mut na, mut nastale) = (0, 0, 0);
		let mut fresh = [false; MS_MAXLH];
		let mut any_before = false;
		for l in 0..MS_MAXLH {
			any_before |= self.file_loaded[l];
			fresh[l] = self.have_start[l] && !self.file_loaded[l];
		}
		let mut anc = [Pose::default(); MS_MAXLH];
		let mut anc_un = [0i32; MS_MAXLH];
		let mut have_anc = [false; MS_MAXLH];
		for line in fgets_lines(&bytes, 4096) {
			let (r, v) = sscanf(&line, "ANCHOR %u %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d");
			if r == 16 {
				let Value::U(aid) = v[0] else { unreachable!() };
				let d: [f64; 14] = doubles(&v, 1);
				let (c, a) = (pose_of(&d[0..7]), pose_of(&d[7..14]));
				let Value::D(un) = v[15] else { unreachable!() };
				let mut l = 0;
				while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
					if host.bsd_base_station_id(l) == aid && fresh[l] {
						if !finite(&d) {
							left_out(host, "ANCHOR", l);
							l += 1;
							continue;
						}
						let (dp, dr) = pose_gap(&c, &self.cal[l]);
						if dp > 1e-5 || dr > 1e-5 {
							// recalibrated since
							nastale += 1;
							if host.log_level() >= 10 {
								let mode = host.bsd_mode(l);
								info(
									host,
									"map-solve: LH ch %d calibration differs from the saved anchors' by %.2f mm / %.3f mrad",
									&[Arg::I(mode as i64), Arg::F(dp * 1e3), Arg::F(dr * 1e3)],
								);
							}
						}
						anc[l] = a;
						anc_un[l] = un;
						have_anc[l] = true;
					}
					l += 1;
				}
				continue;
			}
			let (r, v) = sscanf(&line, "LH %u %lf %lf %lf %lf %lf %lf %lf");
			if r == 8 {
				let Value::U(id) = v[0] else { unreachable!() };
				let d: [f64; 7] = doubles(&v, 1);
				let p = pose_of(&d);
				let mut l = 0;
				while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
					if host.bsd_base_station_id(l) == id && host.bsd_position_set(l) && fresh[l] {
						if finite(&d) {
							self.target[l] = p;
							self.have_target[l] = true;
							host.raw_lighthouse_pose(l, &p);
							n += 1;
						} else {
							left_out(host, "LH", l);
						}
					}
					l += 1;
				}
				continue;
			}
			let (r, v) = sscanf(&line, "LAM %u %u%n");
			if r == 2 {
				let (Value::U(id), Value::U(id2), Value::N(off)) = (&v[0], &v[1], &v[2]) else { unreachable!() };
				let (mut la, mut lb): (i32, i32) = (-1, -1);
				let mut l = 0;
				while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
					let bid = host.bsd_base_station_id(l);
					if bid == *id {
						la = l as i32;
					}
					if bid == *id2 {
						lb = l as i32;
					}
					l += 1;
				}
				if la < 0 || lb < 0 {
					continue;
				}
				let (la, lb) = (la as usize, lb as usize);
				if !self.have_start[la] || !self.have_start[lb] || !(fresh[la] || fresh[lb]) {
					continue;
				}
				// the values up to the first that is not a number, as the C takes them, each into its place: only once
				// all of them are seen finite
				let s = c_str(&line);
				let mut c = *off as usize;
				let mut vals = [0.0; 36];
				let mut nv = 0;
				while nv < 36 {
					let (v, len) = c_strtod(&s[c..]);
					if len == 0 {
						break;
					}
					c += len;
					vals[nv] = v;
					nv += 1;
				}
				if !finite(&vals[..nv]) {
					let (ma, mb) = (host.bsd_mode(la), host.bsd_mode(lb));
					warn(
						host,
						"map-solve: the saved map's LAM line of LH ch %d and ch %d holds a value that is not finite: left \
						 out",
						&[Arg::I(ma as i64), Arg::I(mb as i64)],
					);
					continue;
				}
				for k in 0..nv {
					self.lam[(6 * la + k / 6) * MS_MAXG + 6 * lb + k % 6] = self.s.load_weight * vals[k];
				}
			}
		}
		if nastale != 0 {
			info(host, "map-solve: the saved frame anchors are from another calibration; using the calibration's", &[]);
		} else {
			for l in 0..MS_MAXLH {
				if have_anc[l] {
					self.start[l] = anc[l];
					self.unanchored[l] = anc_un[l] == 0;
					self.anchors_saved = true;
					na += 1;
					if anc_un[l] == 0 {
						let mode = host.bsd_mode(l);
						info(
							host,
							"map-solve: LH ch %d was relocated in an earlier session: it does not anchor the frame",
							&[Arg::I(mode as i64)],
						);
					}
				}
			}
		}
		for l in 0..MS_MAXLH {
			if fresh[l] {
				self.file_loaded[l] = true;
			}
		}
		if !any_before {
			self.lam_time = host.run_time();
		}
		info(
			host,
			"map-solve: %d station(s) taken from the saved map %s%s",
			&[Arg::I(n), Arg::S(&fname), Arg::S(if na != 0 { " (with its frame anchors)" } else { "" })],
		);
	}

	/// save_map_file: the solved map written to a temporary file and moved over the old one, so a runtime killed
	/// mid-write leaves the last good map (nothing before the first applied solve). sally: and so does a write that
	/// fails (a full disk: the C moved what it got written over the map) or a machine that goes down after the move (the
	/// file is on the disk before it)
	pub(crate) fn save_map_file(&mut self, host: &mut (impl Host + ?Sized), quiet: bool) {
		let fname = host.configs("map-solve-file", "");
		if fname.is_empty() || self.applied == 0 {
			return;
		}
		let tmp = cut(format!("{fname}.tmp"), 1100);
		let Ok(mut f) = std::fs::File::create(&tmp) else {
			warn(host, "map-solve: cannot write %s", &[Arg::S(&tmp)]);
			return;
		};
		let mut t = sprintf(
			"# sally map-solve: live-solved station poses (lh2world, config frame) and their information; %d solves applied\n",
			&[Arg::I(self.applied as i64)],
		);
		let p9 = |p: &Pose| -> Vec<Arg<'static>> { p.pos.iter().chain(&p.rot).map(|&v| Arg::F(v)).collect() };
		let mut l = 0;
		while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
			if self.have_target[l] {
				let mut args = vec![Arg::U(host.bsd_base_station_id(l) as u64)];
				args.extend(p9(&self.target[l]));
				t.push_str(&sprintf("LH %u %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", &args));
			}
			l += 1;
		}
		let mut a = 0;
		while (a as i32) < host.active_lighthouses() && a < MS_MAXLH {
			let mut b = 0;
			while (b as i32) < host.active_lighthouses() && b < MS_MAXLH {
				if self.have_target[a] && self.have_target[b] {
					let (ida, idb) = (host.bsd_base_station_id(a), host.bsd_base_station_id(b));
					t.push_str(&sprintf("LAM %u %u", &[Arg::U(ida as u64), Arg::U(idb as u64)]));
					for k in 0..36 {
						t.push_str(&sprintf(" %.9g", &[Arg::F(self.lam[(6 * a + k / 6) * MS_MAXG + 6 * b + k % 6])]));
					}
					t.push('\n');
				}
				b += 1;
			}
			a += 1;
		}
		if self.relocations > 0 || self.anchors_saved {
			let mut l = 0;
			while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
				if self.have_start[l] {
					let mut args = vec![Arg::U(host.bsd_base_station_id(l) as u64)];
					args.extend(p9(&self.cal[l]));
					args.extend(p9(&self.start[l]));
					args.push(Arg::I(if self.unanchored[l] { 0 } else { 1 }));
					t.push_str(&sprintf(
						"ANCHOR %u %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %d\n",
						&args,
					));
				}
				l += 1;
			}
		}
		let written = {
			use std::io::Write;
			f.write_all(&sally_math::textmode::file(t.as_bytes())).and_then(|()| f.sync_all())
		};
		drop(f);
		if written.is_err() {
			warn(host, "map-solve: cannot write %s; the saved map stays as it was", &[Arg::S(&tmp)]);
			return;
		}
		// one atomic step (MoveFileExA with MOVEFILE_REPLACE_EXISTING on Windows): a kill between a remove and a
		// rename lost the map
		if std::fs::rename(&tmp, &fname).is_err() {
			warn(host, "map-solve: cannot move %s to %s", &[Arg::S(&tmp), Arg::S(&fname)]);
			return;
		}
		if quiet {
			self.verbose(host, 10, "map-solve: solved map written to %s", &[Arg::S(&fname)]);
		} else {
			info(host, "map-solve: solved map written to %s", &[Arg::S(&fname)]);
		}
	}
}
