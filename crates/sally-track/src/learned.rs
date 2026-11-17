//! What the tracker learns of a device, carried from one session to the next (docs/sally_design.md 3): its gyro bias,
//! accelerometer bias and sensor-layout scale. They are the device's, nearly the same every session (a tracker's gyro
//! bias holds within +-0.002 rad/s and its accelerometer's within +-0.1 m/s^2 over weeks, a layout scale within +-0.15
//! % of its own), while the factory prior a session would otherwise start them from is wide (0.01 rad/s, 0.1 m/s^2, 1
//! %): each start would spend seconds to minutes learning them again, and a device lit by one station would never learn
//! its layout scale at all.
//!
//! The runtime keeps a file (`device-learned-file`) with each device's sessions, by serial, and the tracker of a device
//! starts from what they say:
//!
//! - **A session's entry is its own measurement.** The tracker reports where each state started (mean, variance) and
//!   where it is now; the start's prior is taken out (the posterior's information less the prior's), so a session that
//!   measured nothing adds nothing, and the history never feeds on itself. A state the session measured less well than
//!   the factory prior does is left out of its entry.
//! - **The prior is the sessions' random-effects mean.** Each state on its own: the sessions' measurements and the spread
//!   between sessions beyond their own variances (DerSimonian-Laird). With fewer than four sessions the factory prior
//!   counts as one more session's worth of spread, so one session moves the start but not its uncertainty; from four on
//!   the spread is the sessions' own, widened for its few degrees of freedom. The variance is never above the factory
//!   prior's. (The factory prior as a session's worth of spread at any count kept a layout scale that varies 0.1-0.15 %
//!   from session to session at 0.32-0.45 %: too loose to hold it at a start.)
//! - **A device whose config changed starts over.** The entries are for the device as its config describes it (the
//!   sensor layout, the IMU's factory calibration): another config is another device to the states.
//!
//! The file, plain text, one block per device:
//!
//! ```text
//! DEV <serial> <fingerprint, 16 hex digits>
//! S <session: unix s> <tracker within the session> <s tracked> <mean variance> x 7    (- - for a state not measured)
//! ```
//!
//! The states in their order: gyro bias x y z (rad/s), accelerometer bias x y z (m/s^2), layout scale (a fraction).

/// The calibration states carried: gyro bias (3), accelerometer bias (3), layout scale
pub const NQ: usize = 7;
/// Entries kept per device (the newest)
pub const KEEP: usize = 10;
/// Sessions from which their own spread sets the prior's (below: the factory prior counts as one of them)
pub const MIN_SPREAD: usize = 4;

/// A state's mean and variance
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Est {
	pub mean: f64,
	pub var: f64,
}

/// What a tracker learned while it ran: each state where it started (at the first seed, or where a reset put it) and
/// where it is now, and how long the tracker held the device (s of the device's clock since the first seed)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Learned {
	pub start: [Est; NQ],
	pub now: [Est; NQ],
	pub tracked_s: f64,
}

/// What a tracker starts from: each state's prior, None where there is none (the factory prior then), and how many
/// sessions it comes from
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Prior {
	pub est: [Option<Est>; NQ],
	pub sessions: [usize; NQ],
}

impl Prior {
	/// No state has a prior
	pub fn is_empty(&self) -> bool {
		self.est.iter().all(Option::is_none)
	}
}

/// One session's measurement of each state (one tracker's: a device that came and went in a session has an entry for
/// each time)
#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
	pub session: i64,
	pub seq: u32,
	pub tracked_s: f64,
	pub x: [Option<Est>; NQ],
}

/// The factory priors (the tracker's starting variances, track-p0-*) and the bounds beyond which a state is not a
/// device's (the tracker's: BG_MAX, BA_MAX, twice track-layout-max), per state
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
	pub factory: [f64; NQ],
	pub bound: [f64; NQ],
}

impl Entry {
	/// The session's own measurement from what its tracker learned: per state, the posterior with the start's prior
	/// taken out (information I1 - I0, mean (m1 I1 - m0 I0) / (I1 - I0)), kept where it is at least as good as the
	/// factory prior and within the bounds
	pub fn from_learned(l: &Learned, lim: &Limits, session: i64, seq: u32) -> Entry {
		let mut x = [None; NQ];
		for k in 0..NQ {
			let (s, n) = (l.start[k], l.now[k]);
			let ok = |e: Est| e.mean.is_finite() && e.var.is_finite() && e.var > 0.0;
			if !ok(s) || !ok(n) || !(lim.factory[k] > 0.0) {
				continue;
			}
			let (i0, i1) = (1.0 / s.var, 1.0 / n.var);
			let gain = i1 - i0;
			if !(gain > 1.0 / lim.factory[k]) {
				continue;
			}
			let mean = (n.mean * i1 - s.mean * i0) / gain;
			if mean.is_finite() && mean.abs() <= lim.bound[k] {
				x[k] = Some(Est { mean, var: 1.0 / gain });
			}
		}
		Entry { session, seq, tracked_s: l.tracked_s, x }
	}

	/// Whether the entry measured any state
	pub fn measured(&self) -> bool {
		self.x.iter().any(Option::is_some)
	}
}

/// The prior a tracker starts from, from the sessions' entries (any order): per state, the random-effects mean of the
/// sessions that measured it, with the factory prior as one session's worth of between-session spread
pub fn prior<'a>(entries: impl IntoIterator<Item = &'a Entry>, lim: &Limits) -> Prior {
	let entries: Vec<&Entry> = entries.into_iter().collect();
	let mut p = Prior { est: [None; NQ], sessions: [0; NQ] };
	for k in 0..NQ {
		let xs: Vec<Est> = entries.iter().filter_map(|e| e.x[k]).collect();
		let n = xs.len();
		let f = lim.factory[k];
		if n == 0 || !(f > 0.0) {
			continue;
		}
		// the fixed-effect mean and Cochran's Q, then the spread between sessions (DerSimonian-Laird)
		let w: Vec<f64> = xs.iter().map(|e| 1.0 / e.var).collect();
		let sw: f64 = w.iter().sum();
		let sw2: f64 = w.iter().map(|w| w * w).sum();
		let m_fe = xs.iter().zip(&w).map(|(e, w)| w * e.mean).sum::<f64>() / sw;
		let q: f64 = xs.iter().zip(&w).map(|(e, w)| w * (e.mean - m_fe) * (e.mean - m_fe)).sum();
		let c = sw - sw2 / sw;
		let tau2_dl = if n > 1 && c > 0.0 { ((q - (n - 1) as f64) / c).max(0.0) } else { 0.0 };
		// with few sessions their spread says little: below MIN_SPREAD the factory prior counts as one session's worth of
		// it; from there on the sessions' own, widened for the few degrees of freedom it has (the mean of its posterior
		// with no prior on the scale of it: (n - 1) / (n - 3))
		let tau2 = if n >= MIN_SPREAD {
			tau2_dl * (n - 1) as f64 / (n - 3) as f64
		} else {
			(f + (n - 1) as f64 * tau2_dl) / n as f64
		};
		let ws: Vec<f64> = xs.iter().map(|e| 1.0 / (e.var + tau2)).collect();
		let sws: f64 = ws.iter().sum();
		let mean = xs.iter().zip(&ws).map(|(e, w)| w * e.mean).sum::<f64>() / sws;
		let var = (tau2 + 1.0 / sws).min(f);
		if mean.is_finite() && var.is_finite() && var > 0.0 && mean.abs() <= lim.bound[k] {
			p.est[k] = Some(Est { mean, var });
			p.sessions[k] = n;
		}
	}
	p
}

/// A device's sessions in the file
#[derive(Clone, Debug, PartialEq)]
pub struct History {
	pub serial: String,
	pub fingerprint: u64,
	pub entries: Vec<Entry>,
}

impl History {
	/// The entries sorted by session and tracker, the newest KEEP kept
	pub fn trim(&mut self) {
		self.entries.sort_by_key(|e| (e.session, e.seq));
		let n = self.entries.len();
		if n > KEEP {
			self.entries.drain(..n - KEEP);
		}
	}
}

/// The file's text read: every device's history, and the number of lines that did not parse (skipped)
pub fn parse(text: &str) -> (Vec<History>, usize) {
	let mut out: Vec<History> = Vec::new();
	let mut bad = 0;
	for line in text.lines() {
		let line = line.trim();
		if line.is_empty() || line.starts_with('#') {
			continue;
		}
		let f: Vec<&str> = line.split_whitespace().collect();
		match f[0] {
			"DEV" if f.len() == 3 => match u64::from_str_radix(f[2], 16) {
				Ok(fp) => out.push(History { serial: f[1].to_string(), fingerprint: fp, entries: Vec::new() }),
				Err(_) => bad += 1,
			},
			"S" if f.len() == 4 + 2 * NQ && !out.is_empty() => match parse_entry(&f) {
				Some(e) => out.last_mut().unwrap().entries.push(e),
				None => bad += 1,
			},
			_ => bad += 1,
		}
	}
	(out, bad)
}

fn parse_entry(f: &[&str]) -> Option<Entry> {
	let session = f[1].parse().ok()?;
	let seq = f[2].parse().ok()?;
	let tracked_s: f64 = f[3].parse().ok()?;
	let mut x = [None; NQ];
	for k in 0..NQ {
		let (m, v) = (f[4 + 2 * k], f[5 + 2 * k]);
		if m == "-" && v == "-" {
			continue;
		}
		let (mean, var): (f64, f64) = (m.parse().ok()?, v.parse().ok()?);
		// a value that is not finite or a variance that is not positive (a damaged file) is not a measurement
		if mean.is_finite() && var.is_finite() && var > 0.0 {
			x[k] = Some(Est { mean, var });
		}
	}
	Some(Entry { session, seq, tracked_s, x })
}

/// The file's text for the histories (in their order), with its header
pub fn format(histories: &[History]) -> String {
	let mut t = String::from(concat!(
		"# sally device-learned-file: what the tracker learned of each device, one S line per session (the newest last),\n",
		"# each state the session's own measurement (mean and variance, its start's prior taken out; - - where it measured\n",
		"# nothing): gyro bias x y z (rad/s), accelerometer bias x y z (m/s^2), sensor-layout scale. Written by the runtime\n",
		"# at exit and every minute; a device's tracker starts from its sessions (crates/sally-track/src/learned.rs)\n",
	));
	for h in histories {
		t.push_str(&format!("DEV {} {:016x}\n", h.serial, h.fingerprint));
		for e in &h.entries {
			t.push_str(&format!("S {} {} {:.1}", e.session, e.seq, e.tracked_s));
			for x in &e.x {
				match x {
					// the shortest text that reads back to the same value
					Some(x) => t.push_str(&format!(" {:e} {:e}", x.mean, x.var)),
					None => t.push_str(" - -"),
				}
			}
			t.push('\n');
		}
	}
	t
}

/// A device config's fingerprint: FNV-1a over the bits of the values the states are relative to (the sensor layout and
/// normals, the IMU's factory calibration), in the order given
pub fn fingerprint(values: impl IntoIterator<Item = f64>) -> u64 {
	let mut h: u64 = 0xcbf2_9ce4_8422_2325;
	for v in values {
		for b in v.to_bits().to_le_bytes() {
			h ^= b as u64;
			h = h.wrapping_mul(0x0000_0100_0000_01b3);
		}
	}
	h
}

#[cfg(test)]
mod tests {
	use super::*;

	const LIM: Limits =
		Limits { factory: [1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-4], bound: [0.2, 0.2, 0.2, 2.0, 2.0, 2.0, 0.03] };

	fn est(mean: f64, var: f64) -> Est {
		Est { mean, var }
	}

	fn learned(start: Est, now: Est) -> Learned {
		Learned { start: [start; NQ], now: [now; NQ], tracked_s: 600.0 }
	}

	/// A session that started from the factory prior keeps its posterior, the shrinkage toward zero taken out
	#[test]
	fn a_session_from_the_factory_prior_is_its_posterior_unshrunk() {
		let e = Entry::from_learned(&learned(est(0.0, 1e-4), est(0.0099, 1e-6)), &LIM, 1, 0);
		let x = e.x[0].unwrap();
		assert!((x.mean - 0.01).abs() < 1e-12, "{x:?}");
		assert!((x.var - 1.0 / (1e6 - 1e4)).abs() < 1e-15);
	}

	/// A session that measured nothing (the posterior is the prior it started from) adds nothing, however tight that
	/// prior was: the history does not feed on itself
	#[test]
	fn a_session_that_measured_nothing_adds_nothing() {
		let e = Entry::from_learned(&learned(est(0.03, 1e-5), est(0.03, 1e-5)), &LIM, 1, 0);
		assert!(!e.measured());
		// ... and one that learned less than the factory prior gives is left out too
		let e = Entry::from_learned(&learned(est(0.03, 1e-5), est(0.031, 1.0 / (1e5 + 5e3))), &LIM, 1, 0);
		assert!(e.x[0].is_none());
	}

	/// A session from a tight prior: the measurement taken out is the one that, combined with the prior, gives the
	/// posterior
	#[test]
	fn a_session_from_a_prior_gives_back_its_own_measurement() {
		let (m0, v0) = (0.03, 1e-5);
		let (mz, vz) = (0.034, 2e-6); // what the session measured on its own
		let v1 = 1.0 / (1.0 / v0 + 1.0 / vz);
		let m1 = v1 * (m0 / v0 + mz / vz);
		let x = Entry::from_learned(&learned(est(m0, v0), est(m1, v1)), &LIM, 1, 0).x[0].unwrap();
		assert!((x.mean - mz).abs() < 1e-12 && (x.var - vz).abs() < 1e-15, "{x:?}");
	}

	/// One session moves the start to its value but keeps the factory uncertainty; many that agree tighten it, never
	/// below the spread they show
	#[test]
	fn the_prior_moves_with_one_session_and_tightens_with_many() {
		let one = |m: f64| Entry { session: 1, seq: 0, tracked_s: 600.0, x: [Some(est(m, 1e-8)); NQ] };
		let p = prior(&[one(0.033)], &LIM);
		let g = p.est[0].unwrap();
		assert!((g.mean - 0.033).abs() < 1e-12 && g.var == 1e-4 && p.sessions[0] == 1);
		// three: the factory prior still one session's worth of the spread
		let es: Vec<Entry> = (0..3).map(|i| one(0.033 + 0.0015 * (i as f64 - 1.0))).collect();
		let g = prior(&es, &LIM).est[0].unwrap();
		let tau2 = (1e-4 + 2.0 * (0.0015f64.powi(2) - 1e-8)) / 3.0;
		assert!((g.var - (tau2 + (1e-8 + tau2) / 3.0)).abs() < 1e-12, "{g:?}");
		// ten sessions spread +-0.0015 about 0.033: their own spread, (0.0015^2 * 10 / 9) less their own variance (1e-8),
		// widened by 9 / 7
		let es: Vec<Entry> = (0..10).map(|i| one(0.033 + 0.0015 * if i % 2 == 0 { 1.0 } else { -1.0 })).collect();
		let p = prior(&es, &LIM);
		let g = p.est[0].unwrap();
		assert!((g.mean - 0.033).abs() < 1e-9);
		let tau2 = (0.0015f64.powi(2) * 10.0 / 9.0 - 1e-8) * 9.0 / 7.0;
		assert!((g.var - (tau2 + (1e-8 + tau2) / 10.0)).abs() < 1e-12, "{g:?}");
		assert!(g.var < 1e-4 / 20.0 && g.var > 0.0015f64.powi(2));
	}

	/// The variance a prior gives is never above the factory's, and a state no session measured has none
	#[test]
	fn no_prior_is_wider_than_the_factory_and_unmeasured_states_have_none() {
		let mut x = [None; NQ];
		x[0] = Some(est(0.05, 5e-5));
		x[1] = Some(est(-0.05, 5e-5));
		let a = Entry { session: 1, seq: 0, tracked_s: 60.0, x };
		let mut x2 = [None; NQ];
		x2[0] = Some(est(-0.05, 5e-5));
		let b = Entry { session: 2, seq: 0, tracked_s: 60.0, x: x2 };
		let p = prior(&[a, b], &LIM);
		assert!(p.est[0].unwrap().var <= 1e-4);
		assert_eq!(p.sessions[1], 1);
		assert!(p.est[2].is_none() && p.sessions[2] == 0);
	}

	/// The file reads back what was written, value for value, and a damaged line is skipped
	#[test]
	fn the_file_reads_back_what_was_written() {
		let mut x = [None; NQ];
		x[0] = Some(est(0.1 + 0.2, 1.0 / 3.0 * 1e-7));
		x[6] = Some(est(-0.00471, 9.5e-7));
		let h = vec![
			History {
				serial: "LHR-2D779713".into(),
				fingerprint: 0x0123_4567_89ab_cdef,
				entries: vec![Entry { session: 1794174498, seq: 1, tracked_s: 612.3, x }],
			},
			History { serial: "LHR-1B80952C".into(), fingerprint: 7, entries: vec![] },
		];
		let t = format(&h);
		assert_eq!(parse(&t), (h.clone(), 0));
		let damaged = t.replace("DEV LHR-1B80952C", "DEV LHR-1B80952C x y")
			+ "S 1 2 3\nS 1 0 1.0 nan 1 - - - - - - - - - - - -\n";
		let (back, bad) = parse(&damaged);
		assert_eq!(bad, 2);
		// the last S line had no DEV of its own left: it went to the first device, its NaN state not taken
		assert_eq!(back.len(), 1);
		assert_eq!(back[0].entries.len(), 2);
		assert!(!back[0].entries[1].measured());
	}

	/// A history keeps its newest KEEP entries, in order
	#[test]
	fn a_history_keeps_its_newest_entries() {
		let e = |s: i64, q: u32| Entry { session: s, seq: q, tracked_s: 1.0, x: [None; NQ] };
		let mut h = History { serial: "a".into(), fingerprint: 0, entries: (0..14).rev().map(|s| e(s, 0)).collect() };
		h.entries.push(e(13, 1));
		h.trim();
		assert_eq!(h.entries.len(), KEEP);
		assert_eq!((h.entries[0].session, h.entries.last().unwrap().seq), (5, 1));
	}
}
