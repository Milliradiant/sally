//! `sally pack` and `sally unpack`: a tap recording from its text to its packed form (`sally_hid::pack`) and back,
//! and a pose log (`sally_pack::table`).
//!
//! ```text
//! sally pack [--strip] [--to <folder>] [--replace] [--force] <file>...     x.hidtap -> x.hidtapz, x.csv -> x.csvz
//! sally unpack [--to <folder>] [--replace] [--force] <file>...             x.hidtapz -> x.hidtap, x.csvz -> x.csv
//! ```
//!
//! Which of the two a file is: a packed one by its first bytes, a text one a pose log where its name ends in .csv
//! and a recording otherwise.
//!
//! The new file goes beside the one it is made from, or into `--to`'s folder. It is written, then read back and held
//! against the one it was made from, byte for byte; only then does it count as done, and only then is the one it was
//! made from removed, if `--replace` asks for that. A file that exists is not written over without `--force`. The new
//! file keeps the old one's time, as the sessions are told apart by it.
//!
//! `--strip` leaves out of each report what nothing reads (`sally_hid::strip`), as the runtime does when it records:
//! the packed recording then holds the text with zeros in those places, about a third smaller. A recording that is
//! packed already is done so in its place (`--replace`), or into another folder. A pose log has nothing of the kind
//! to leave out.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use sally_hid::pack;
use sally_hid::strip;
use sally_pack::table;

#[derive(Clone, Copy, PartialEq)]
enum To {
	Packed,
	Text,
}

/// What a file is: a tap recording or a pose log
#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
	Recording,
	PoseLog,
}

impl Kind {
	/// What the file at `path` is, and whether it is packed
	fn of(path: &Path) -> std::io::Result<(Kind, bool)> {
		let mut input = File::open(path)?;
		if pack::packed(&mut input)? {
			return Ok((Kind::Recording, true));
		}
		if table::packed(&mut input)? {
			return Ok((Kind::PoseLog, true));
		}
		let is_table = path.file_name().is_some_and(|n| n.to_string_lossy().ends_with(table::TEXT_END));
		Ok((if is_table { Kind::PoseLog } else { Kind::Recording }, false))
	}

	/// The file at `path`, packed or text, as its text
	fn open(self, path: &Path) -> std::io::Result<Box<dyn Read>> {
		Ok(match self {
			Kind::Recording => Box::new(pack::open(path)?),
			Kind::PoseLog => Box::new(table::open(path)?),
		})
	}
}

/// What is asked of each recording
#[derive(Clone, Default)]
struct Ask {
	strip: bool,
	replace: bool,
	force: bool,
	folder: Option<PathBuf>,
}

/// The name a recording gets in the other form: its end changed, or the other end added to a name without it
fn other_name(path: &Path, to: To, kind: Kind) -> PathBuf {
	let name = path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
	let (to_end, stem) = match (kind, to) {
		(Kind::Recording, To::Packed) => (pack::PACKED_END, pack::stem(&name)),
		(Kind::Recording, To::Text) => (pack::TEXT_END, pack::stem(&name)),
		(Kind::PoseLog, To::Packed) => (table::PACKED_END, table::stem(&name)),
		(Kind::PoseLog, To::Text) => (table::TEXT_END, table::stem(&name)),
	};
	path.with_file_name(format!("{}{to_end}", stem.unwrap_or(&name)))
}

/// A recording's text, with what `strip` leaves out gone when that is asked: the lines as they are otherwise
struct Lines<R> {
	input: R,
	strip: bool,
	line: Vec<u8>,
	given: usize,
}

impl<R: BufRead> Lines<R> {
	fn new(input: R, strip: bool) -> Lines<R> {
		Lines { input, strip, line: Vec::new(), given: 0 }
	}
}

impl<R: BufRead> Read for Lines<R> {
	fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
		if !self.strip {
			return self.input.read(buf);
		}
		if self.given == self.line.len() {
			self.line.clear();
			self.given = 0;
			if self.input.read_until(b'\n', &mut self.line)? == 0 {
				return Ok(0);
			}
			strip::line(&mut self.line);
		}
		let n = buf.len().min(self.line.len() - self.given);
		buf[..n].copy_from_slice(&self.line[self.given..self.given + n]);
		self.given += n;
		Ok(n)
	}
}

/// The file at `path` as its text, as `ask` wants it
fn text_of(path: &Path, kind: Kind, ask: &Ask) -> std::io::Result<impl Read> {
	Ok(Lines::new(BufReader::with_capacity(1 << 20, kind.open(path)?), ask.strip && kind == Kind::Recording))
}

/// Text into a packed file of `kind`: the file back when all of it is in
fn pack_into<W: Write>(kind: Kind, text: &mut impl Read, out: W) -> std::io::Result<W> {
	let mut piece = vec![0u8; 1 << 16];
	match kind {
		Kind::Recording => {
			let mut packed = pack::PackWriter::new(out)?;
			loop {
				let n = text.read(&mut piece)?;
				if n == 0 {
					return packed.finish();
				}
				packed.write(&piece[..n])?;
			}
		}
		Kind::PoseLog => {
			let mut packed = table::PackWriter::new(out)?;
			loop {
				let n = text.read(&mut piece)?;
				if n == 0 {
					return packed.finish();
				}
				packed.write(&piece[..n])?;
			}
		}
	}
}

/// Whether two readers give the same bytes to their ends
fn same(a: &mut dyn Read, b: &mut dyn Read) -> std::io::Result<bool> {
	let (mut a, mut b) = (BufReader::with_capacity(1 << 20, a), BufReader::with_capacity(1 << 20, b));
	let (mut x, mut y) = (vec![0u8; 1 << 16], vec![0u8; 1 << 16]);
	loop {
		let n = a.read(&mut x)?;
		if n == 0 {
			return Ok(b.read(&mut y)? == 0);
		}
		if b.read_exact(&mut y[..n]).is_err() || x[..n] != y[..n] {
			return Ok(false);
		}
	}
}

/// One recording into its other form: what was done, or why not
fn one(path: &Path, to: To, ask: &Ask) -> Result<String, String> {
	let name = path.display();
	let (kind, is_packed) = Kind::of(path).map_err(|e| format!("{name}: {e}"))?;
	let strip = ask.strip && kind == Kind::Recording;
	let mut out_path = other_name(path, to, kind);
	if let (Some(folder), Some(file)) = (&ask.folder, out_path.file_name()) {
		out_path = folder.join(file);
	}
	// a recording made anew where it is: written beside itself first
	let in_place = out_path == path;
	match (to, is_packed) {
		(To::Text, false) if in_place => return Ok(format!("{name}: text already")),
		(To::Packed, true) if in_place && !strip => return Ok(format!("{name}: packed already")),
		(To::Packed, true) if in_place && !ask.replace => {
			return Err(format!(
				"{name}: packed already (--strip with --replace does it in its place, or --to <folder>)"
			))
		}
		_ => {}
	}
	let written = match in_place {
		true => path.with_file_name(format!("{}.part", out_path.file_name().unwrap_or_default().to_string_lossy())),
		false => out_path.clone(),
	};
	if written.exists() && !ask.force {
		return Err(format!("{name}: {} exists (--force writes over it)", written.display()));
	}
	let write = || -> std::io::Result<()> {
		let mut text = text_of(path, kind, ask)?;
		let made = File::create(&written)
			.map_err(|e| std::io::Error::new(e.kind(), format!("cannot write {}: {e}", written.display())))?;
		let out = BufWriter::with_capacity(1 << 20, made);
		let out = match to {
			To::Packed => pack_into(kind, &mut text, out)?,
			To::Text => {
				let mut out = out;
				std::io::copy(&mut text, &mut out)?;
				out
			}
		};
		let out = out.into_inner().map_err(|e| e.into_error())?;
		if let Ok(time) = std::fs::metadata(path).and_then(|m| m.modified()) {
			let _ = out.set_modified(time);
		}
		out.sync_all()
	};
	// the new file read back as text against the text it was made from
	let check = || -> std::io::Result<bool> { same(&mut text_of(path, kind, ask)?, &mut kind.open(&written)?) };
	let done = write().and_then(|()| check());
	match done {
		Ok(true) => {}
		Ok(false) => {
			let _ = std::fs::remove_file(&written);
			return Err(format!("{name}: its other form did not read back the same; nothing kept of it"));
		}
		Err(e) => {
			let _ = std::fs::remove_file(&written);
			return Err(format!("{name}: {e}"));
		}
	}
	let size = |p: &Path| std::fs::metadata(p).map(|m| m.len()).unwrap_or(0) as f64;
	let (before, after) = (size(path), size(&written));
	let mut said = format!(
		"{name}: {:.1} MB -> {:.1} MB as {}, read back the same",
		before / 1e6,
		after / 1e6,
		out_path.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
	);
	if to == To::Packed && after > 0.0 {
		said += &format!(" ({:.1} times smaller)", before / after);
	}
	if in_place {
		// (the checks above let this happen only with --replace)
		if let Err(e) = std::fs::rename(&written, path) {
			let _ = std::fs::remove_file(&written);
			return Err(format!("{name}: its new form could not take its place: {e}"));
		}
		said += "; in the old one's place";
	} else if ask.replace {
		match std::fs::remove_file(path) {
			Ok(()) => said += "; the old one removed",
			Err(e) => said += &format!("; the old one could not be removed: {e}"),
		}
	}
	Ok(said)
}

fn run(args: &[String], to: To) -> i32 {
	let (mut ask, mut paths) = (Ask::default(), Vec::new());
	let usage = match to {
		To::Packed => {
			"usage: sally pack [--strip] [--to <folder>] [--replace] [--force] <tap recording or pose log>..."
		}
		To::Text => "usage: sally unpack [--to <folder>] [--replace] [--force] <tap recording or pose log>...",
	};
	let mut args = args.iter();
	while let Some(a) = args.next() {
		match a.as_str() {
			"--replace" => ask.replace = true,
			"--force" => ask.force = true,
			"--strip" if to == To::Packed => ask.strip = true,
			"--to" => match args.next() {
				Some(folder) => ask.folder = Some(PathBuf::from(folder)),
				None => paths.clear(),
			},
			_ => paths.push(PathBuf::from(a)),
		}
	}
	if paths.is_empty() {
		eprintln!("{usage}");
		return 2;
	}
	let mut code = 0;
	for p in &paths {
		match one(p, to, &ask) {
			Ok(said) => {
				println!("{said}");
				let _ = std::io::stdout().flush();
			}
			Err(why) => {
				eprintln!("{why}");
				code = 1;
			}
		}
	}
	code
}

pub fn pack(args: &[String]) -> i32 {
	run(args, To::Packed)
}

pub fn unpack(args: &[String]) -> i32 {
	run(args, To::Text)
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A dongle's reports: a packet of light (a channel and two entries), the dongle's bytes and leftovers after it
	fn text(lines: usize) -> Vec<u8> {
		let mut t = b"# cache D LHR-1\r\n".to_vec();
		for n in 0..lines {
			let report: String = (0..59)
				.map(|i| {
					let byte = match i {
						0 => 0x23,
						2 => 11,
						4 => 0x10,
						5 => 0x31,
						_ => ((n * 7 + i) % 251) & 0xfc | 2,
					};
					format!("{byte:02x}")
				})
				.collect();
			t.extend(format!("R {} D 2102 -1 -1 I {report}\r\n", 1000 + n * 2000).bytes());
		}
		t
	}

	fn ask(strip: bool, replace: bool, force: bool) -> Ask {
		Ask { strip, replace, force, folder: None }
	}

	#[test]
	fn a_recording_goes_there_and_back() {
		let dir = std::env::temp_dir().join(format!("sally-packing-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let plain = dir.join("sallytap_20261101_100000.hidtap");
		let packed = dir.join("sallytap_20261101_100000.hidtapz");
		let original = text(400);
		std::fs::write(&plain, &original).unwrap();
		let time = std::time::SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(1_790_000_000);
		File::options().write(true).open(&plain).unwrap().set_modified(time).unwrap();

		// packed beside it, the text kept, the time kept
		let said = one(&plain, To::Packed, &ask(false, false, false)).unwrap();
		assert!(said.contains("read back the same") && said.contains("times smaller"), "{said}");
		assert!(plain.exists() && pack::is_packed(&std::fs::read(&packed).unwrap()));
		assert_eq!(std::fs::metadata(&packed).unwrap().modified().unwrap(), time);
		assert!(std::fs::metadata(&packed).unwrap().len() < original.len() as u64 / 2);

		// not written over, not packed twice
		assert!(one(&plain, To::Packed, &ask(false, false, false)).unwrap_err().contains("exists"));
		assert!(one(&packed, To::Packed, &ask(false, false, false)).unwrap().contains("packed already"));
		assert!(one(&plain, To::Text, &ask(false, false, false)).unwrap().contains("text already"));

		// replaced: the text goes once the packed one has read back the same
		let said = one(&plain, To::Packed, &ask(false, true, true)).unwrap();
		assert!(said.contains("the old one removed"), "{said}");
		assert!(!plain.exists());

		// and back
		let said = one(&packed, To::Text, &ask(false, true, false)).unwrap();
		assert!(said.contains("read back the same"), "{said}");
		assert!(!packed.exists());
		assert_eq!(std::fs::read(&plain).unwrap(), original);
		assert_eq!(std::fs::metadata(&plain).unwrap().modified().unwrap(), time);

		// a name without the usual end gets the other end added
		assert_eq!(other_name(Path::new("a/tap.rec"), To::Packed, Kind::Recording), Path::new("a/tap.rec.hidtapz"));
		assert_eq!(other_name(Path::new("a/tap.hidtapz"), To::Text, Kind::Recording), Path::new("a/tap.hidtap"));
		assert!(one(&dir.join("none.hidtap"), To::Packed, &ask(false, false, false)).is_err());
		std::fs::remove_dir_all(&dir).unwrap();
	}

	/// A pose log is told from a recording by its name as text and by its first bytes packed, and goes there and back
	/// the same way; `--strip` is not for it
	#[test]
	fn a_pose_log_goes_there_and_back() {
		let dir = std::env::temp_dir().join(format!("sally-packing-poses-{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(&dir).unwrap();
		let plain = dir.join("sally_poses_20261101_100000.csv");
		let packed = dir.join("sally_poses_20261101_100000.csvz");
		let mut original = b"t,id,serial,sx,sqw\n".to_vec();
		for n in 0..3000 {
			let row = format!(
				"{:.6},{},LHR-0000000{},{:.5},{:.6}\n",
				100.0 + n as f64 * 0.011,
				n % 3,
				n % 3,
				n as f64 * 0.002,
				0.25
			);
			original.extend(row.bytes());
		}
		std::fs::write(&plain, &original).unwrap();
		assert_eq!(Kind::of(&plain).unwrap(), (Kind::PoseLog, false));

		let said = one(&plain, To::Packed, &ask(true, false, false)).unwrap();
		assert!(said.contains("read back the same") && said.contains("times smaller"), "{said}");
		assert_eq!(Kind::of(&packed).unwrap(), (Kind::PoseLog, true));
		assert!(std::fs::metadata(&packed).unwrap().len() < original.len() as u64 / 8);
		assert!(one(&packed, To::Packed, &ask(true, true, false)).unwrap().contains("packed already"));

		// packed, it is a pose log whatever it is called
		let renamed = dir.join("poses.hidtapz");
		std::fs::rename(&packed, &renamed).unwrap();
		assert_eq!(Kind::of(&renamed).unwrap(), (Kind::PoseLog, true));
		std::fs::remove_file(&plain).unwrap();
		let said = one(&renamed, To::Text, &ask(false, true, false)).unwrap();
		assert!(said.contains("as poses.hidtapz.csv") && said.contains("the old one removed"), "{said}");
		assert_eq!(std::fs::read(dir.join("poses.hidtapz.csv")).unwrap(), original);
		std::fs::remove_dir_all(&dir).unwrap();
	}

	/// `--strip`: the packed recording holds the text without what nothing reads, from a text recording or a packed
	/// one, beside it, in another folder or in its place
	#[test]
	fn a_recording_is_packed_without_what_nothing_reads() {
		let dir = std::env::temp_dir().join(format!("sally-packing-strip-{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(dir.join("lean")).unwrap();
		let plain = dir.join("sallytap_20261101_100000.hidtap");
		let packed = dir.join("sallytap_20261101_100000.hidtapz");
		let original = text(400);
		std::fs::write(&plain, &original).unwrap();
		let read = |p: &Path| {
			let mut t = Vec::new();
			pack::open(p).unwrap().read_to_end(&mut t).unwrap();
			t
		};
		// what the lines are without those bytes: the same length, the report's start as it was, zeros after
		let mut lean = Vec::new();
		text_of(&plain, Kind::Recording, &ask(true, false, false)).unwrap().read_to_end(&mut lean).unwrap();
		assert_eq!(lean.len(), original.len());
		let (first, was) =
			(lean.split(|&b| b == b'\n').nth(1).unwrap(), original.split(|&b| b == b'\n').nth(1).unwrap());
		assert_eq!(first[..22 + 28], was[..22 + 28]);
		assert_eq!(first[22 + 28..], format!("{}\r", "00".repeat(45)).into_bytes());

		// whole first, to compare the sizes
		one(&plain, To::Packed, &ask(false, false, false)).unwrap();
		let whole_size = std::fs::metadata(&packed).unwrap().len();
		assert_eq!(read(&packed), original);

		// a packed one is not done in its place without --replace; into another folder it is, the first left alone
		assert!(one(&packed, To::Packed, &ask(true, false, false)).unwrap_err().contains("--replace"));
		let elsewhere = Ask { strip: true, folder: Some(dir.join("lean")), ..Ask::default() };
		let said = one(&packed, To::Packed, &elsewhere).unwrap();
		assert!(said.contains("read back the same"), "{said}");
		let other = dir.join("lean").join("sallytap_20261101_100000.hidtapz");
		assert_eq!(read(&other), lean);
		assert_eq!(read(&packed), original);
		assert!(std::fs::metadata(&other).unwrap().len() * 3 < whole_size * 2);
		// from the text into that folder: the same file
		let from_packed = std::fs::read(&other).unwrap();
		assert!(one(&plain, To::Packed, &elsewhere).unwrap_err().contains("exists"));
		one(&plain, To::Packed, &Ask { force: true, ..elsewhere.clone() }).unwrap();
		assert_eq!(std::fs::read(&other).unwrap(), from_packed);

		// in its place
		let said = one(&packed, To::Packed, &ask(true, true, false)).unwrap();
		assert!(said.contains("in the old one's place"), "{said}");
		assert_eq!(std::fs::read(&packed).unwrap(), from_packed);
		assert!(plain.exists() && !dir.join("sallytap_20261101_100000.hidtapz.part").exists());
		// done again it is the same, and unpacked it is the lean text
		one(&packed, To::Packed, &ask(true, true, false)).unwrap();
		assert_eq!(std::fs::read(&packed).unwrap(), from_packed);
		std::fs::remove_file(&plain).unwrap();
		one(&packed, To::Text, &ask(false, false, false)).unwrap();
		assert_eq!(std::fs::read(&plain).unwrap(), lean);
		std::fs::remove_dir_all(&dir).unwrap();
	}
}
