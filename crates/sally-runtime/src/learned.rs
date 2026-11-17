//! What the tracker learned of each device, carried from one session to the next (`sally_track::learned` has the model
//! and the file's format): the runtime's `device-learned-file`, read when the first device's config arrives, written
//! every minute and at exit. A device's tracker starts from its sessions when its config arrives (before its first
//! seed); every tracker of the session (one per device, and another when a device leaves its dongle and comes back on
//! one) is one entry. Empty option: nothing is read or written (replays and the bench unless they ask).

use sally_track::learned::{self, Entry, History, Prior};
use sally_track::tracker::learned_limits;

use crate::config::SC_GET;
use crate::object::TrackedObject;
use crate::runtime::Runtime;
use crate::ObjectId;

/// The file is written this often (s of run time) besides at exit, so a runtime that is killed keeps most of its
/// session
const SAVE_PERIOD: f64 = 60.0;

/// One tracker of this session: the device it tracks, its entry as last taken, and its object while it lives
#[derive(Debug)]
struct Slot {
	serial: String,
	fingerprint: u64,
	seq: u32,
	entry: Option<Entry>,
	obj: Option<ObjectId>,
}

/// The file and this session's trackers
#[derive(Debug)]
pub struct Store {
	file: String,
	/// this session: unix s at the store's start
	session: i64,
	/// the devices' histories as read
	histories: Vec<History>,
	slots: Vec<Slot>,
	last_save: f64,
	write_warned: bool,
}

impl Store {
	/// The entries the device's tracker starts from: its history in the file, if its config is the one the history is
	/// for, and this session's finished trackers of it. Also whether the file has a history for another config of it
	fn entries(&self, serial: &str, fingerprint: u64) -> (Vec<&Entry>, bool) {
		let mut out = Vec::new();
		let mut other = false;
		if let Some(h) = self.histories.iter().find(|h| h.serial == serial) {
			if h.fingerprint == fingerprint {
				out.extend(h.entries.iter());
			} else {
				other = true;
			}
		}
		for s in &self.slots {
			if s.serial == serial && s.fingerprint == fingerprint && s.obj.is_none() {
				out.extend(s.entry.iter());
			}
		}
		(out, other)
	}

	/// The histories to write: the file's, each device's trackers of this session that measured something added (a
	/// device whose config changed starts a new history with them), the newest KEEP kept
	fn merged(&self) -> Vec<History> {
		let mut out = self.histories.clone();
		for s in &self.slots {
			let Some(e) = s.entry.as_ref().filter(|e| e.measured()) else { continue };
			let i = match out.iter().position(|h| h.serial == s.serial) {
				Some(i) => i,
				None => {
					out.push(History { serial: s.serial.clone(), fingerprint: s.fingerprint, entries: Vec::new() });
					out.len() - 1
				}
			};
			let h = &mut out[i];
			if h.fingerprint != s.fingerprint {
				*h = History { serial: s.serial.clone(), fingerprint: s.fingerprint, entries: Vec::new() };
			}
			h.entries.retain(|x| (x.session, x.seq) != (e.session, e.seq));
			h.entries.push(e.clone());
			h.trim();
		}
		out
	}
}

/// A device's config, as far as the learned states are relative to it: the sensor layout and normals, the IMU's
/// factory calibration
pub(crate) fn fingerprint(so: &TrackedObject) -> u64 {
	let n = 3 * so.sensor_ct.max(0) as usize;
	let take = |v: &Option<Vec<f64>>| v.as_ref().map(|v| v[..n.min(v.len())].to_vec()).unwrap_or_default();
	let mut values = vec![so.sensor_ct as f64];
	values.extend(take(&so.sensor_locations));
	values.extend(take(&so.sensor_normals));
	for a in [so.acc_bias, so.acc_scale, so.gyro_bias, so.gyro_scale] {
		values.extend(a);
	}
	learned::fingerprint(values)
}

/// The prior's line: each state's mean, the largest sd of its axes, and the sessions it comes from
fn prior_text(p: &Prior) -> String {
	let part = |name: &str, ks: std::ops::Range<usize>, scale: f64, prec: usize, unit: &str| -> String {
		if ks.clone().all(|k| p.est[k].is_none()) {
			return format!("{name} from the factory prior");
		}
		let mut t = name.to_string();
		let mut sd: f64 = 0.0;
		for k in ks.clone() {
			match p.est[k] {
				Some(e) => {
					t.push_str(&format!(" {:+.*}", prec, e.mean * scale));
					sd = sd.max(e.var.sqrt() * scale);
				}
				None => t.push_str(" -"),
			}
		}
		let n = ks.map(|k| p.sessions[k]).max().unwrap_or(0);
		format!("{t}{unit} (sd {sd:.prec$}, {n} session{})", if n == 1 { "" } else { "s" })
	};
	format!(
		"{}, {}, {}",
		part("gyro bias", 0..3, 1.0, 4, " rad/s"),
		part("accelerometer bias", 3..6, 1.0, 3, " m/s^2"),
		part("layout scale", 6..7, 100.0, 3, " %")
	)
}

impl Runtime {
	/// The store, read at the first call when device-learned-file names one (None without)
	fn learned_open(&mut self) -> Option<Store> {
		if let Some(s) = self.learned.take() {
			return Some(s);
		}
		if self.learned_tried {
			return None;
		}
		self.learned_tried = true;
		let file = self.config.configs("device-learned-file", SC_GET, Some(b"")).unwrap_or_default();
		let file = String::from_utf8_lossy(&file).into_owned();
		if file.is_empty() {
			return None;
		}
		let histories = match std::fs::read(&file) {
			Ok(bytes) => {
				let (h, bad) = learned::parse(&String::from_utf8_lossy(&bytes));
				if bad > 0 {
					self.warn(&format!("device-learned: {bad} line(s) of {file} not understood: skipped"));
				}
				let most = h.iter().map(|h| h.entries.len()).max().unwrap_or(0);
				self.info(&format!(
					"device-learned: what earlier sessions learned of {} devices (up to {most} sessions each), from {file}",
					h.len()
				));
				h
			}
			Err(_) => {
				self.info(&format!("device-learned: no {file} yet: every device starts from the factory prior"));
				Vec::new()
			}
		};
		let session =
			std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0);
		let last_save = self.peek_run_time();
		Some(Store { file, session, histories, slots: Vec::new(), last_save, write_warned: false })
	}

	/// The device's config arrived: its tracker starts from what earlier sessions learned of it (before its first seed),
	/// and is this session's next entry for it
	pub(crate) fn learned_device(&mut self, obj: ObjectId) {
		let Some(mut store) = self.learned_open() else { return };
		let so = &self.objects[obj];
		let (serial, codename) = (so.serial_number.clone(), so.codename.clone());
		let fp = fingerprint(so);
		let live = store.slots.iter().position(|s| s.obj == Some(obj));
		let start = match live {
			// the same device's config again: the tracker goes on as it was
			Some(i) if store.slots[i].serial == serial && store.slots[i].fingerprint == fp => false,
			Some(i) => {
				store.slots[i].entry = self.learned_entry(obj, &store, i);
				store.slots[i].obj = None;
				true
			}
			None => true,
		};
		if start && !serial.is_empty() && so.tracker.is_some() {
			let lim = learned_limits(&self.settings.tracker);
			let (entries, other) = store.entries(&serial, fp);
			let prior = learned::prior(entries, &lim);
			let seq = store.slots.iter().filter(|s| s.serial == serial).count() as u32;
			store.slots.push(Slot { serial: serial.clone(), fingerprint: fp, seq, entry: None, obj: Some(obj) });
			if other {
				self.info(&format!(
					"{codename} ({serial}): its config is not the one its learned calibration is for: the tracker \
					 starts from the factory prior, and its history starts over"
				));
			}
			if !prior.is_empty() {
				let taken = self.objects[obj].tracker.as_mut().is_some_and(|t| t.set_prior(prior));
				if taken {
					self.info(&format!("{codename} ({serial}): the tracker starts from {}", prior_text(&prior)));
				}
			}
		}
		self.learned = Some(store);
	}

	/// What the slot's tracker (the object's) learned so far, as its entry (the slot's last one when the tracker has
	/// nothing yet)
	fn learned_entry(&self, obj: ObjectId, store: &Store, i: usize) -> Option<Entry> {
		let s = &store.slots[i];
		let l = self.objects[obj].tracker.as_ref().and_then(|t| t.learned());
		match l {
			Some(l) => Some(Entry::from_learned(&l, &learned_limits(&self.settings.tracker), store.session, s.seq)),
			None => s.entry.clone(),
		}
	}

	/// The object's tracker is about to go (its object starts over): its entry taken
	pub(crate) fn learned_detach(&mut self, obj: ObjectId) {
		let Some(mut store) = self.learned.take() else { return };
		if let Some(i) = store.slots.iter().position(|s| s.obj == Some(obj)) {
			store.slots[i].entry = self.learned_entry(obj, &store, i);
			store.slots[i].obj = None;
		}
		self.learned = Some(store);
	}

	/// The file written when it is due (every SAVE_PERIOD s of run time)
	pub(crate) fn learned_tick(&mut self) {
		let due = match &self.learned {
			Some(s) => self.peek_run_time() - s.last_save >= SAVE_PERIOD,
			None => false,
		};
		if due {
			self.learned_save(true);
		}
	}

	/// Every live tracker's entry taken and the file written (a temporary file moved over it, so a runtime killed
	/// mid-write leaves the last one); at exit (not quiet) with a line
	pub(crate) fn learned_save(&mut self, quiet: bool) {
		let Some(mut store) = self.learned.take() else { return };
		for i in 0..store.slots.len() {
			if let Some(obj) = store.slots[i].obj {
				store.slots[i].entry = self.learned_entry(obj, &store, i);
			}
		}
		store.last_save = self.peek_run_time();
		let merged = store.merged();
		let text = learned::format(&merged);
		let tmp = format!("{}.tmp", store.file);
		let written = std::fs::write(&tmp, text.as_bytes()).and_then(|()| std::fs::rename(&tmp, &store.file));
		let measured = store.slots.iter().filter(|s| s.entry.as_ref().is_some_and(Entry::measured)).count();
		match written {
			Ok(()) if !quiet => self.info(&format!(
				"device-learned: {} devices' calibration written to {} ({measured} tracker(s) of this session measured some)",
				merged.len(),
				store.file
			)),
			Ok(()) => {}
			Err(e) => {
				if !store.write_warned || !quiet {
					self.warn(&format!("device-learned: cannot write {}: {e}", store.file));
				}
				store.write_warned = true;
			}
		}
		self.learned = Some(store);
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use sally_track::learned::{Est, NQ};

	fn entry(session: i64, m: f64) -> Entry {
		Entry { session, seq: 0, tracked_s: 60.0, x: [Some(Est { mean: m, var: 1e-8 }); NQ] }
	}

	fn store(histories: Vec<History>) -> Store {
		Store { file: String::new(), session: 9, histories, slots: Vec::new(), last_save: 0.0, write_warned: false }
	}

	/// A device's tracker starts from its history only if its config is the one the history is for, and this
	/// session's finished trackers of it count too
	#[test]
	fn a_device_starts_from_its_own_configs_history() {
		let mut s = store(vec![History { serial: "A".into(), fingerprint: 1, entries: vec![entry(1, 0.01)] }]);
		s.slots.push(Slot { serial: "A".into(), fingerprint: 1, seq: 0, entry: Some(entry(9, 0.02)), obj: None });
		s.slots.push(Slot { serial: "A".into(), fingerprint: 1, seq: 1, entry: Some(entry(9, 0.03)), obj: Some(0) });
		let (e, other) = s.entries("A", 1);
		assert_eq!((e.len(), other), (2, false));
		let (e, other) = s.entries("A", 2);
		assert_eq!((e.len(), other), (0, true));
		assert_eq!(s.entries("B", 1).0.len(), 0);
	}

	/// What is written: the file's histories with this session's measured entries added (once each, however often
	/// written), a changed config starting its device's history over, a tracker that measured nothing adding nothing
	#[test]
	fn the_session_is_added_once_and_a_changed_config_starts_over() {
		let mut s = store(vec![
			History { serial: "A".into(), fingerprint: 1, entries: vec![entry(1, 0.01)] },
			History { serial: "B".into(), fingerprint: 5, entries: vec![entry(1, 0.01), entry(2, 0.01)] },
		]);
		s.slots.push(Slot { serial: "A".into(), fingerprint: 1, seq: 0, entry: Some(entry(9, 0.02)), obj: None });
		s.slots.push(Slot { serial: "B".into(), fingerprint: 6, seq: 0, entry: Some(entry(9, 0.02)), obj: None });
		let mut quiet = entry(9, 0.0);
		quiet.x = [None; NQ];
		s.slots.push(Slot { serial: "C".into(), fingerprint: 1, seq: 0, entry: Some(quiet), obj: None });
		let m = s.merged();
		assert_eq!(m.len(), 2);
		assert_eq!(m[0].entries.len(), 2);
		assert_eq!((m[1].fingerprint, m[1].entries.len()), (6, 1));
		// written again with the same entries: nothing doubles
		s.histories = m.clone();
		assert_eq!(s.merged(), m);
	}
}
