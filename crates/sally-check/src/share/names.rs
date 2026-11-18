//! The stand-ins: what each serial of this PC's devices is called in anything shared.
//!
//! A stand-in has its serial's own shape, so everything that reads a recording, a config or a room's files reads a
//! shared one unchanged: `LHR-00000001` for a tracked device, `LHB-00000001` for a base station (its id 1),
//! `0000000001-7YX` for a dongle (its ten digits; the three characters after them say which kind of dongle it is,
//! not which one, and stay), `hmd-1` or `controller-1` for a device of another tracking system. They are numbered in
//! the order they are first wanted and say nothing else.
//!
//! The table is a file in sally's folder (`kind`, the real serial, the stand-in, tab-separated), so a device keeps
//! its stand-in from one share to the next. It holds the real serials: it never goes into a share, and a share is
//! searched for every serial in it before it is handed over (`verify`).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// What a serial is the serial of
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Kind {
	/// a tracked lighthouse device: the eight hex digits after `LHR-`
	Device,
	/// a base station: its id's eight hex digits (after `LHB-`, or the id as a number)
	Station,
	/// a dongle: its ten hex digits
	Dongle,
	/// a device of another tracking system: its serial as it is
	Other,
}

impl Kind {
	fn word(self) -> &'static str {
		match self {
			Kind::Device => "device",
			Kind::Station => "station",
			Kind::Dongle => "dongle",
			Kind::Other => "other",
		}
	}

	fn of(word: &str) -> Option<Kind> {
		[Kind::Device, Kind::Station, Kind::Dongle, Kind::Other].into_iter().find(|k| k.word() == word)
	}
}

/// The table: every real serial with its stand-in
#[derive(Clone, Debug, Default)]
pub struct Names {
	path: Option<PathBuf>,
	/// (kind, the real serial's digits in upper case, or an other device's serial as it is) to the stand-in's
	pairs: BTreeMap<(Kind, String), String>,
	changed: bool,
}

fn hex(s: &str, digits: usize) -> bool {
	s.len() == digits && s.bytes().all(|c| c.is_ascii_hexdigit())
}

/// A lighthouse device's digits from its serial (`LHR-` and eight hex digits)
pub fn device_digits(serial: &str) -> Option<String> {
	let d = serial.get(4..)?;
	(serial.get(..4)?.eq_ignore_ascii_case("LHR-") && hex(d, 8)).then(|| d.to_ascii_uppercase())
}

/// A base station's id from its serial (`LHB-` and eight hex digits)
pub fn station_id(serial: &str) -> Option<u32> {
	let d = serial.get(4..)?;
	(serial.get(..4)?.eq_ignore_ascii_case("LHB-") && hex(d, 8)).then(|| u32::from_str_radix(d, 16).ok())?
}

/// A dongle's ten digits and the three characters after them, from its serial (`0A1B2C3D4E-6YX`)
pub fn dongle_parts(serial: &str) -> Option<(String, &str)> {
	let (d, rest) = (serial.get(..10)?, serial.get(10..)?);
	let tail = rest.strip_prefix('-')?;
	(hex(d, 10) && tail.len() == 3 && tail.bytes().all(|c| c.is_ascii_alphanumeric()))
		.then(|| (d.to_ascii_uppercase(), tail))
}

impl Names {
	/// The table in `path` (none there yet: an empty one, written there by `save`)
	pub fn load(path: &Path) -> Names {
		let mut n = Names { path: Some(path.to_path_buf()), ..Names::default() };
		for line in std::fs::read_to_string(path).unwrap_or_default().lines() {
			let f: Vec<&str> = line.split('\t').collect();
			if let (3, Some(kind)) = (f.len(), f.first().and_then(|k| Kind::of(k))) {
				n.pairs.insert((kind, f[1].to_string()), f[2].to_string());
			}
		}
		n
	}

	/// The table written back, when it has grown
	pub fn save(&mut self) -> std::io::Result<()> {
		let Some(path) = self.path.as_ref().filter(|_| self.changed) else { return Ok(()) };
		let mut text = String::from(
			"# sally: the stand-ins for this PC's devices in what it shares (kind, serial, stand-in). Kept so that a\n\
			 # device has the same stand-in every time. It holds the real serials: it is never shared.\n",
		);
		for ((kind, real), stand_in) in &self.pairs {
			text.push_str(&format!("{}\t{real}\t{stand_in}\n", kind.word()));
		}
		if let Some(dir) = path.parent() {
			std::fs::create_dir_all(dir)?;
		}
		std::fs::write(path, text)?;
		self.changed = false;
		Ok(())
	}

	/// The stand-in's digits for a real serial's (its text for an other device, where `class` names the first one:
	/// `hmd`, `controller`), made on its first asking
	pub fn digits(&mut self, kind: Kind, real: &str, class: &str) -> String {
		let key = (kind, if kind == Kind::Other { real.to_string() } else { real.to_ascii_uppercase() });
		if let Some(s) = self.pairs.get(&key) {
			return s.clone();
		}
		let taken = |s: &str| self.pairs.iter().any(|((k, _), v)| *k == kind && v == s);
		let mut n = 1u64;
		let stand_in = loop {
			let s = match kind {
				Kind::Device | Kind::Station => format!("{n:08X}"),
				Kind::Dongle => format!("{n:010X}"),
				Kind::Other => format!("{}-{n}", if class.is_empty() { "device" } else { class }),
			};
			if !taken(&s) {
				break s;
			}
			n += 1;
		};
		self.pairs.insert(key, stand_in.clone());
		self.changed = true;
		stand_in
	}

	/// A base station's stand-in id
	pub fn station(&mut self, id: u32) -> u32 {
		u32::from_str_radix(&self.digits(Kind::Station, &format!("{id:08X}"), ""), 16).unwrap_or(0)
	}

	/// The stand-in for a whole serial as SteamVR or a recording gives it: a lighthouse device's, a base station's,
	/// a dongle's by their shapes, anything else as an other device of `class`
	pub fn serial(&mut self, serial: &str, class: &str) -> String {
		if let Some(d) = device_digits(serial) {
			format!("LHR-{}", self.digits(Kind::Device, &d, ""))
		} else if let Some(id) = station_id(serial) {
			format!("LHB-{:08X}", self.station(id))
		} else if let Some((d, tail)) = dongle_parts(serial) {
			format!("{}-{tail}", self.digits(Kind::Dongle, &d, ""))
		} else {
			self.digits(Kind::Other, serial, class)
		}
	}

	/// How many serials the table holds
	pub fn len(&self) -> usize {
		self.pairs.len()
	}

	pub fn is_empty(&self) -> bool {
		self.pairs.is_empty()
	}

	/// The stand-in's digits for a real serial's, if the table has it
	pub fn known(&self, kind: Kind, real: &str) -> Option<&str> {
		let key = (kind, if kind == Kind::Other { real.to_string() } else { real.to_ascii_uppercase() });
		self.pairs.get(&key).map(String::as_str)
	}

	/// Whether `digits` are a stand-in's of that kind
	pub fn is_stand_in(&self, kind: Kind, digits: &str) -> bool {
		self.pairs.iter().any(|((k, _), v)| *k == kind && v.eq_ignore_ascii_case(digits))
	}

	/// Every real serial of a kind (its digits, or an other device's serial) that is not also a stand-in's
	pub fn real(&self, kind: Kind) -> Vec<&str> {
		self.pairs.keys().filter(|(k, r)| *k == kind && !self.is_stand_in(kind, r)).map(|(_, r)| r.as_str()).collect()
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_serial_gets_a_stand_in_of_its_own_shape_and_keeps_it() {
		let mut n = Names::default();
		assert_eq!(n.serial("LHR-1A2B3C4D", "tracker"), "LHR-00000001");
		assert_eq!(n.serial("lhr-1a2b3c4d", "tracker"), "LHR-00000001");
		assert_eq!(n.serial("LHR-99999999", "tracker"), "LHR-00000002");
		assert_eq!(n.serial("LHB-B1C2D3E4", "station"), "LHB-00000001");
		assert_eq!(n.station(0xb1c2_d3e4), 1);
		assert_eq!(n.station(7), 2);
		assert_eq!(n.serial("0A1B2C3D4E-6YX", ""), "0000000001-6YX");
		assert_eq!(n.serial("1122334455-RYB", ""), "0000000002-RYB");
		assert_eq!(n.serial("1PASH0000000", "hmd"), "hmd-1");
		assert_eq!(n.serial("1PASH0000000_Controller_Left", "controller"), "controller-1");
		assert_eq!(n.serial("1PASH0000000_Controller_Right", "controller"), "controller-2");
		assert_eq!(n.serial("1PASH0000000", "other"), "hmd-1");
		assert!(n.is_stand_in(Kind::Device, "00000002") && !n.is_stand_in(Kind::Device, "1A2B3C4D"));
		assert_eq!(n.real(Kind::Device), ["1A2B3C4D", "99999999"]);
	}

	#[test]
	fn the_table_is_read_back_as_it_was_written() {
		let dir = std::env::temp_dir().join(format!("sally-names-{}", std::process::id()));
		let path = dir.join("share_names.txt");
		let mut n = Names::load(&path);
		let a = n.serial("LHR-1A2B3C4D", "");
		let b = n.serial("some serial with spaces", "hmd");
		n.save().unwrap();
		let mut again = Names::load(&path);
		assert_eq!(again.serial("LHR-1A2B3C4D", ""), a);
		assert_eq!(again.serial("some serial with spaces", "other"), b);
		assert_eq!(again.serial("LHR-00000009", ""), "LHR-00000002");
		let _ = std::fs::remove_dir_all(dir);
	}

	#[test]
	fn a_serial_that_is_a_stand_in_already_is_not_a_real_one() {
		// a recording that was shared before, shared again: its serials are stand-ins and stay what they are only if
		// the table gives them themselves, which a real serial of the same digits would not
		let mut n = Names::default();
		assert_eq!(n.serial("LHR-00000001", ""), "LHR-00000001");
		assert!(n.real(Kind::Device).is_empty());
	}
}
