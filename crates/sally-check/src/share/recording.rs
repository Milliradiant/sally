//! A recording made fit to share: cut to a span, every serial a stand-in, the base stations' ids in the light
//! rewritten (`light`), and what nothing reads of a report set to zeros.
//!
//! `cut` writes a span of a session's recording as a recording of its own: the lines that say which devices there
//! are and which device each dongle carries, as they stood at the span's start, then the span's reports.
//!
//! `scrub` then reads that through the parser as the runtime reads it (`Analyzer::watch`) and writes it again:
//! - a device line, an assignment and a cache comment with the stand-ins for their serials;
//! - an input report with its dongle's stand-in, each sync's data bit as `light` gives it, and zeros in the bytes the
//!   parser passes over (`Host::light_unread`: a packet's light while the light mode is not known yet or after a
//!   packet's error, a report that is not one sally reads). A report that reaches no device is left out;
//! - nothing else: other comments, feature reports (a device's config is one, serial and all) and reports written
//!   to a device are not lines a live recording has, and a replay needs none of them.
//!
//! What comes out is read once more the same way, and has to say the same: the parser hands on what it handed on of
//! the original, sync for sync and hit for hit (the digest), every sync's data bit is the one it was given, every
//! byte passed over is zero, and the only station ids in it are stand-ins. A recording that fails any of it is not
//! written.

use std::io::{BufRead, BufReader, Write as _};
use std::path::Path;

use sally_hid::line::{self, Line};
use sally_hid::pack::{self, PackWriter};
use sally_hid::replay::Replay;
use sally_hid::tap::Options;

use super::light::{Stations, DATA_BIT_BYTE, DATA_BIT_MASK};
use super::names::{Kind, Names};
use crate::analysis::{Analyzer, ConfigSource};

/// How long after a span's end reports of it may still come (the tap's readers write in turns), in microseconds
const LATE_US: u64 = 2_000_000;

/// What a cut holds
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Cut {
	/// the reports in it, and the first's and the last's time
	pub reports: u64,
	pub from_us: u64,
	pub to_us: u64,
	/// the dongles it names (a wired device's own serial among them), and each line that says which device a dongle
	/// carries: (the dongle, the device), in the recording's order
	pub dongles: Vec<String>,
	pub pairs: Vec<(String, String)>,
}

fn words(line: &[u8]) -> Vec<&str> {
	std::str::from_utf8(line).unwrap_or("").split_ascii_whitespace().collect()
}

fn lines(path: &Path) -> std::io::Result<BufReader<Box<dyn std::io::Read + Send>>> {
	Ok(BufReader::with_capacity(1 << 20, pack::open(path)?))
}

/// The span `from_us..=to_us` of the recording `source` (None: all of it) written to `out`, packed: the devices and
/// the assignments as they stand at the span's start (each at the span's first report's time: a replay's clock
/// starts at the first line), then the span's reports
pub fn cut(source: &Path, span: Option<(u64, u64)>, out: &Path) -> std::io::Result<Cut> {
	let (from, to) = span.unwrap_or((0, u64::MAX));
	let mut input = lines(source)?;
	let mut w = PackWriter::new(std::io::BufWriter::new(std::fs::File::create(out)?))?;
	// what stands before the span: (the line's letter or "#", the dongle) with the line's words
	let mut before: Vec<((String, String), Vec<String>)> = Vec::new();
	let mut c = Cut::default();
	let mut buf = Vec::new();
	loop {
		buf.clear();
		if input.read_until(b'\n', &mut buf)? == 0 {
			break;
		}
		let f = words(&buf);
		let time = f.get(1).and_then(|t| t.parse::<u64>().ok());
		// who is in the recording: every device line, and every assignment up to the span's end
		match f.as_slice() {
			["D", _, dongle, _] if !c.dongles.iter().any(|d| d == dongle) => c.dongles.push(dongle.to_string()),
			["A", _, dongle, device] | ["#", "cache", dongle, device] if *device != "-" => {
				c.pairs.push((dongle.to_string(), device.to_string()))
			}
			_ => {}
		}
		match (f.first().copied(), time) {
			(Some("R"), Some(t)) => {
				if t < from {
					continue;
				}
				if t > to {
					if t > to.saturating_add(LATE_US) {
						break;
					}
					continue;
				}
				if c.reports == 0 {
					for (_, f) in before.drain(..) {
						let at = if f[0] == "#" { f.join(" ") } else { format!("{} {t} {}", f[0], f[2..].join(" ")) };
						w.write(format!("{at}\n").as_bytes())?;
					}
					c.from_us = t;
				}
				c.reports += 1;
				c.to_us = c.to_us.max(t);
				// a recording names a dongle by its reports alone, unless it was made from a capture
				if let Some(dongle) = f.get(2).filter(|d| !c.dongles.iter().any(|x| x == *d)) {
					c.dongles.push(dongle.to_string());
				}
				w.write(&buf)?;
				if !buf.ends_with(b"\n") {
					w.write(b"\n")?;
				}
			}
			(Some("D" | "A"), Some(_)) if f.len() >= 4 => {
				if c.reports > 0 {
					w.write(&buf)?;
				} else {
					let key = (f[0].to_string(), f[2].to_string());
					let line: Vec<String> = f.iter().map(|s| s.to_string()).collect();
					// a device is announced once, where it was first seen; a dongle's assignment is its latest
					match before.iter_mut().find(|(k, _)| *k == key) {
						Some(_) if f[0] == "D" => {}
						Some(entry) => entry.1 = line,
						None => before.push((key, line)),
					}
				}
			}
			(Some("#"), _) if f.len() == 4 && f[1] == "cache" => {
				if c.reports > 0 {
					w.write(&buf)?;
				} else {
					let key = ("#".to_string(), f[2].to_string());
					let line: Vec<String> = f.iter().map(|s| s.to_string()).collect();
					match before.iter_mut().find(|(k, _)| *k == key) {
						Some(entry) => entry.1 = line,
						None => before.push((key, line)),
					}
				}
			}
			_ => {}
		}
	}
	w.finish()?.flush()?;
	Ok(c)
}

/// What a report gets changed: its line's number, each sync (where its entry stands, its number among the syncs),
/// and the bytes passed over
struct Edit {
	line: u64,
	syncs: Vec<(u16, u32)>,
	unread: Vec<(u16, u16)>,
}

/// What `scrub` did
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Scrubbed {
	/// the reports written, and the ones left out because they reach no device or are not input reports
	pub reports: u64,
	pub left_out: u64,
	/// the syncs, and the station blocks found whole in the original and in what was written
	pub syncs: u64,
	pub blocks: u64,
	pub blocks_written: u64,
	/// the stations seen, by their stand-in ids
	pub stations: Vec<u32>,
}

fn options() -> Options {
	Options {
		playback_factor: 0.0,
		allow_fs_read: true,
		config_fetch: false,
		lightcap_mode: -2,
		past_end: 2,
		..Options::default()
	}
}

/// What `read` gives for each line: its number, its text, whether a report of it reached a device, and what the
/// parser said of that report
type Each<'a> = &'a mut dyn FnMut(u64, &[u8], bool, &crate::analysis::Seen) -> Result<(), String>;

/// One read of a recording as the runtime reads it, line by line. Returns the digest
fn read(path: &Path, configs: ConfigSource, each: Each) -> Result<u64, String> {
	let mut a = Analyzer::watch(configs);
	let mut input = lines(path).map_err(|e| e.to_string())?;
	let mut replay = Replay::new(&mut a, options(), &path.to_string_lossy(), None);
	let mut buf = Vec::new();
	let mut n = 0u64;
	loop {
		buf.clear();
		if input.read_until(b'\n', &mut buf).map_err(|e| e.to_string())? == 0 {
			break;
		}
		let before = a.reports_read();
		replay.line(&mut a, &buf);
		let read = a.reports_read() != before;
		let seen = a.seen().ok_or("no watch")?;
		if !read {
			seen.syncs.clear();
			seen.unread.clear();
		}
		each(n, &buf, read, seen)?;
		seen.syncs.clear();
		seen.unread.clear();
		n += 1;
	}
	replay.close(&mut a);
	Ok(a.seen().map_or(0, |s| s.digest))
}

/// The recording `cut` (one `cut` wrote, or a whole one) written to `out`, packed, fit to share (module doc).
/// `configs`: a device's config by its dongle's serial or its own, the real ones; `shared_configs`: the same for
/// what is written, by the stand-ins (the configs that go with it, as `config::scrub` leaves them)
pub fn scrub(
	cut: &Path, configs: ConfigSource, shared_configs: ConfigSource, names: &mut Names, out: &Path,
) -> Result<Scrubbed, String> {
	// the first read: what each report gets changed
	let mut stations = Stations::new();
	let mut edits: Vec<Edit> = Vec::new();
	let mut unread_lines = std::collections::BTreeSet::new();
	let digest = read(cut, configs, &mut |n, text, read, seen| {
		if text.first() != Some(&b'R') {
			return Ok(());
		}
		if !read {
			unread_lines.insert(n);
			return Ok(());
		}
		if seen.syncs.is_empty() && seen.unread.is_empty() {
			return Ok(());
		}
		let syncs = seen
			.syncs
			.iter()
			.map(|s| {
				let mark = stations.sync(s.obj, s.channel, s.timecode, s.data, &mut |id| names.station(id));
				(s.at as u16, mark as u32)
			})
			.collect();
		let unread = seen.unread.iter().map(|&(at, len)| (at as u16, len.min(u16::MAX as usize) as u16)).collect();
		edits.push(Edit { line: n, syncs, unread });
		Ok(())
	})?;

	stations.finish();

	// the second: the lines written
	let mut s = Scrubbed { syncs: stations.bits.len() as u64, blocks: stations.blocks, ..Scrubbed::default() };
	s.stations = stations.ids.iter().map(|&id| names.station(id)).collect();
	let file = std::fs::File::create(out).map_err(|e| format!("cannot write {}: {e}", out.display()))?;
	let mut w = PackWriter::new(std::io::BufWriter::new(file)).map_err(|e| e.to_string())?;
	let mut input = lines(cut).map_err(|e| e.to_string())?;
	let mut next = edits.iter().peekable();
	let mut buf = Vec::new();
	let mut n = 0u64;
	let io = |e: std::io::Error| e.to_string();
	loop {
		buf.clear();
		if input.read_until(b'\n', &mut buf).map_err(io)? == 0 {
			break;
		}
		let number = n;
		n += 1;
		match line::parse(&buf) {
			Line::Device { time_us, serial, pid } => {
				w.write(format!("D {time_us} {} {pid:04x}\n", names.serial(&serial, "device")).as_bytes())
					.map_err(io)?;
			}
			Line::Assign { time_us, serial, device } => {
				let to = if device == "-" { device } else { names.serial(&device, "device") };
				w.write(&line::assign(time_us, &names.serial(&serial, "device"), &to)).map_err(io)?;
			}
			Line::Report { time_us, serial, pid, mi, col, kind: b'I', mut data } => {
				if unread_lines.contains(&number) {
					s.left_out += 1;
					continue;
				}
				if let Some(e) = next.next_if(|e| e.line == number) {
					for &(at, len) in &e.unread {
						let (from, to) = (at as usize + 1, (at as usize + len as usize).min(data.len()));
						if from < to {
							data[from..to].fill(0);
						}
					}
					for &(at, mark) in &e.syncs {
						if let Some(b) = data.get_mut(at as usize + DATA_BIT_BYTE) {
							*b = if stations.bits[mark as usize] { *b | DATA_BIT_MASK } else { *b & !DATA_BIT_MASK };
						}
					}
				}
				s.reports += 1;
				let serial = names.serial(&serial, "device");
				w.write(&line::input_report(time_us, &serial, pid, mi, col, &data)).map_err(io)?;
			}
			Line::Report { .. } => s.left_out += 1,
			Line::Ignored => {
				let f = words(&buf);
				if f.len() == 4 && f[0] == "#" && f[1] == "cache" {
					let (dongle, device) = (names.serial(f[2], "device"), names.serial(f[3], "device"));
					w.write(&line::cache_comment(&dongle, &device)).map_err(io)?;
				}
			}
			Line::BadLine | Line::BadHex => s.left_out += 1,
		}
	}
	w.finish().map_err(io)?.flush().map_err(io)?;

	// the third: what was written, read as the first read the original
	let mut after = Stations::new();
	let mut sync_no = 0usize;
	let mut wrong = None;
	let written = read(out, shared_configs, &mut |n, text, read, seen| {
		if text.first() != Some(&b'R') {
			return Ok(());
		}
		if !read && wrong.is_none() {
			wrong = Some(format!("line {n} reaches no device"));
		}
		let data = match line::parse(text) {
			Line::Report { data, .. } => data,
			_ => Vec::new(),
		};
		for sy in &seen.syncs {
			if stations.bits.get(sync_no) != Some(&sy.data) && wrong.is_none() {
				wrong = Some(format!("line {n}: a sync's data bit is not the one it was given"));
			}
			after.sync(sy.obj, sy.channel, sy.timecode, sy.data, &mut |id| id);
			sync_no += 1;
		}
		for &(at, len) in &seen.unread {
			let (from, to) = (at + 1, (at + len).min(data.len()));
			if from < to && data[from..to].iter().any(|&b| b != 0) && wrong.is_none() {
				wrong = Some(format!("line {n}: bytes passed over are not zero"));
			}
		}
		Ok(())
	})?;
	if let Some(w) = wrong {
		return Err(format!("the recording was not made fit to share ({w})"));
	}
	if written != digest || sync_no != stations.bits.len() {
		return Err("the recording made to share does not read as the one recorded".into());
	}
	if let Some(id) = after.ids.iter().find(|id| !names.is_stand_in(Kind::Station, &format!("{id:08X}"))) {
		return Err(format!("the recording made to share still names a base station ({id:08x})"));
	}
	s.blocks_written = after.blocks;
	Ok(s)
}

/// The dongles' and devices' serials of a recording fit to share: every one of them a stand-in. Err names the
/// first line that has another
pub fn only_stand_ins(path: &Path, names: &Names) -> Result<(), String> {
	let mut input = lines(path).map_err(|e| e.to_string())?;
	let mut buf = Vec::new();
	let mut n = 0u64;
	let known = |serial: &str| -> bool {
		if let Some(d) = super::names::device_digits(serial) {
			names.is_stand_in(Kind::Device, &d)
		} else if let Some((d, _)) = super::names::dongle_parts(serial) {
			names.is_stand_in(Kind::Dongle, &d)
		} else {
			names.is_stand_in(Kind::Other, serial)
		}
	};
	loop {
		buf.clear();
		if input.read_until(b'\n', &mut buf).map_err(|e| e.to_string())? == 0 {
			return Ok(());
		}
		n += 1;
		let ok = match line::parse(&buf) {
			Line::Device { serial, .. } => known(&serial),
			Line::Assign { serial, device, .. } => known(&serial) && (device == "-" || known(&device)),
			Line::Report { serial, kind, .. } => kind == b'I' && known(&serial),
			Line::Ignored => {
				let f = words(&buf);
				f.len() == 4 && f[0] == "#" && f[1] == "cache" && known(f[2]) && known(f[3])
			}
			Line::BadLine | Line::BadHex => false,
		};
		if !ok {
			return Err(format!("line {n} of the recording made to share is not one a share has"));
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn text(path: &Path) -> String {
		let mut s = String::new();
		std::io::Read::read_to_string(&mut pack::open(path).unwrap(), &mut s).unwrap();
		s
	}

	#[test]
	fn a_cut_is_the_span_with_what_stood_before_it() {
		let dir = std::env::temp_dir().join(format!("sally-cut-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let (source, out) = (dir.join("session.hidtap"), dir.join("cut.hidtapz"));
		let recording = "# a comment\n\
			# cache AAAA000001-1YX LHR-00000001\n\
			A 90 AAAA000002-2YX LHR-00000002\n\
			R 100 AAAA000001-1YX 2102 -1 -1 I 2301\n\
			R 150 AAAA000002-2YX 2102 -1 -1 I 2302\n\
			A 180 AAAA000002-2YX LHR-00000003\n\
			R 200 AAAA000001-1YX 2102 -1 -1 I 2303\n\
			R 250 AAAA000002-2YX 2102 -1 -1 I 2304\n\
			A 260 AAAA000001-1YX -\n\
			R 300 AAAA000001-1YX 2102 -1 -1 I 2305\n\
			R 5000000 AAAA000001-1YX 2102 -1 -1 I 2306\n\
			R 299 AAAA000002-2YX 2102 -1 -1 I 2307\n";
		std::fs::write(&source, recording).unwrap();

		let c = cut(&source, Some((200, 300)), &out).unwrap();
		assert_eq!((c.reports, c.from_us, c.to_us), (3, 200, 300));
		assert_eq!(c.dongles, ["AAAA000001-1YX", "AAAA000002-2YX"]);
		// the dongles' devices as they stood at the span's start, at its first report's time; then the span, and
		// no report from after it (one later than the readers' turns allow ends the reading)
		assert_eq!(
			text(&out),
			"# cache AAAA000001-1YX LHR-00000001\n\
			 A 200 AAAA000002-2YX LHR-00000003\n\
			 R 200 AAAA000001-1YX 2102 -1 -1 I 2303\n\
			 R 250 AAAA000002-2YX 2102 -1 -1 I 2304\n\
			 A 260 AAAA000001-1YX -\n\
			 R 300 AAAA000001-1YX 2102 -1 -1 I 2305\n"
		);

		// all of it: every line a share's recording can have
		let c = cut(&source, None, &out).unwrap();
		assert_eq!(c.reports, 7);
		assert!(
			text(&out).starts_with("# cache AAAA000001-1YX LHR-00000001\nA 100 AAAA000002-2YX LHR-00000002\nR 100 ")
		);
		let _ = std::fs::remove_dir_all(dir);
	}

	#[test]
	fn a_recording_with_another_serial_is_not_a_shares() {
		let dir = std::env::temp_dir().join(format!("sally-standins-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let path = dir.join("tap.hidtap");
		let mut names = Names::default();
		let (dongle, device) = (names.serial("0A1B2C3D4E-6YX", ""), names.serial("LHR-1A2B3C4D", ""));
		let good = format!(
			"# cache {dongle} {device}\nA 5 {dongle} {device}\nR 10 {dongle} 2102 -1 -1 I 2301\nA 20 {dongle} -\n"
		);
		std::fs::write(&path, &good).unwrap();
		assert_eq!(only_stand_ins(&path, &names), Ok(()));
		for bad in [
			"R 10 0A1B2C3D4E-6YX 2102 -1 -1 I 2301\n".to_string(),
			format!("A 5 {dongle} LHR-1A2B3C4D\n"),
			format!("R 10 {dongle} 2102 -1 -1 F 10aa\n"),
			"# a comment\n".to_string(),
			format!("D 5 {dongle} 2102\nR 10 LHR-0BADC0DE 2000 0 -1 I 2301\n"),
		] {
			std::fs::write(&path, format!("{good}{bad}")).unwrap();
			assert!(only_stand_ins(&path, &names).is_err(), "{bad}");
		}
		let _ = std::fs::remove_dir_all(dir);
	}
}
