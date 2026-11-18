//! Text made fit to share: a log, a settings file, the room's files. Every serial in it becomes its stand-in
//! (`names`), a path loses the folders that name the PC's user, and the user's and the PC's names go where they
//! stand as words.
//!
//! A serial is found by its shape wherever it stands as a word of its own: `LHR-` or `LHB-` and eight hex digits; a
//! dongle's ten hex digits, with or without the three characters after them; and the bare digits of a serial the
//! table knows (a base station's id is written without `LHB-` by Valve's driver and by sally's log, and as a number
//! in the room's files). In the lines of Valve's driver any eight hex digits with a letter and a digit among them
//! are taken for a base station, known or not: that is how it names them. Digits after a decimal point are part of a
//! number and never a serial.
//!
//! `clean` is the check after: text is clean when scrubbing it again changes nothing and finds no new serial.

use super::names::{Kind, Names};

/// What must not be in a share besides the serials: the Windows user's name and folder, the PC's name, and sally's
/// own folder (which is under the user's)
#[derive(Clone, Debug, Default)]
pub struct Private {
	pub user: String,
	pub pc: String,
	pub home: String,
	pub sally: String,
}

impl Private {
	/// This PC's, from the environment, with sally's folder
	pub fn here(sally: &std::path::Path) -> Private {
		let var = |k: &str| std::env::var(k).unwrap_or_default();
		let home = if var("USERPROFILE").is_empty() { var("HOME") } else { var("USERPROFILE") };
		let user = if var("USERNAME").is_empty() { var("USER") } else { var("USERNAME") };
		let pc = if var("COMPUTERNAME").is_empty() { var("HOSTNAME") } else { var("COMPUTERNAME") };
		Private { user, pc, home, sally: sally.to_string_lossy().into_owned() }
	}
}

fn sep(c: u8) -> bool {
	c == b'\\' || c == b'/'
}

fn word(c: u8) -> bool {
	c.is_ascii_alphanumeric()
}

/// Where the path `pat` stands in `text` from `at` on, whatever its case and its separators (`\`, `/`, or either
/// doubled, as a JSON string writes one): the match's end. A match ends where a folder's name does
fn path_at(text: &[u8], at: usize, pat: &[u8]) -> Option<usize> {
	let (mut i, mut j) = (at, 0);
	while j < pat.len() {
		if sep(pat[j]) {
			if i >= text.len() || !sep(text[i]) {
				return None;
			}
			while i < text.len() && sep(text[i]) {
				i += 1;
			}
			while j < pat.len() && sep(pat[j]) {
				j += 1;
			}
		} else {
			if i >= text.len() || !text[i].eq_ignore_ascii_case(&pat[j]) {
				return None;
			}
			i += 1;
			j += 1;
		}
	}
	(i >= text.len() || !word(text[i])).then_some(i)
}

/// The paths: sally's folder as `<sally>`, the user's as `<home>`, and the name after any `Users` folder as `user`
fn paths(text: &[u8], private: &Private) -> Vec<u8> {
	let named: Vec<(&[u8], &[u8])> =
		[(private.sally.as_bytes(), &b"<sally>"[..]), (private.home.as_bytes(), b"<home>")]
			.into_iter()
			.filter(|(p, _)| p.len() > 3)
			.collect();
	let mut out = Vec::with_capacity(text.len());
	let mut i = 0;
	'text: while i < text.len() {
		let start = i == 0 || !word(text[i - 1]);
		if start {
			for (pat, with) in &named {
				let pat = pat.strip_suffix(b"\\").or(pat.strip_suffix(b"/")).unwrap_or(pat);
				if let Some(end) = path_at(text, i, pat) {
					out.extend_from_slice(with);
					i = end;
					continue 'text;
				}
			}
			// `Users\<name>`: the name up to the next separator, or to where a path ends
			if i > 0 && sep(text[i - 1]) && text.len() >= i + 6 && text[i..i + 5].eq_ignore_ascii_case(b"users") {
				let mut j = i + 5;
				if sep(text[j]) {
					while j < text.len() && sep(text[j]) {
						j += 1;
					}
					let from = j;
					while j < text.len() && !sep(text[j]) && !b"\"'<>|\r\n\t".contains(&text[j]) {
						j += 1;
					}
					if j > from {
						out.extend_from_slice(&text[i..from]);
						out.extend_from_slice(b"user");
						i = j;
						continue 'text;
					}
				}
			}
		}
		out.push(text[i]);
		i += 1;
	}
	out
}

/// The user's and the PC's names where they stand as words (a name under three characters is left: it is found in
/// paths alone)
fn words(text: &[u8], private: &Private) -> Vec<u8> {
	let named: Vec<(&[u8], &[u8])> = [(private.user.as_bytes(), &b"<user>"[..]), (private.pc.as_bytes(), b"<pc>")]
		.into_iter()
		.filter(|(n, _)| n.len() >= 3)
		.collect();
	let mut out = Vec::with_capacity(text.len());
	let mut i = 0;
	'text: while i < text.len() {
		if i == 0 || !word(text[i - 1]) {
			for (name, with) in &named {
				let end = i + name.len();
				if end <= text.len()
					&& text[i..end].eq_ignore_ascii_case(name)
					&& (end == text.len() || !word(text[end]))
				{
					out.extend_from_slice(with);
					i = end;
					continue 'text;
				}
			}
		}
		out.push(text[i]);
		i += 1;
	}
	out
}

/// The serials of devices of other tracking systems the table knows, where they stand as words: they have no shape
/// to be found by (the longest first: a headset's serial starts its controllers')
fn others(text: &[u8], names: &Names) -> Vec<u8> {
	let mut known: Vec<(&str, &str)> =
		names.real(Kind::Other).into_iter().filter_map(|r| Some((r, names.known(Kind::Other, r)?))).collect();
	known.sort_by_key(|(r, _)| std::cmp::Reverse(r.len()));
	let mut out = Vec::with_capacity(text.len());
	let mut i = 0;
	'text: while i < text.len() {
		if i == 0 || !word(text[i - 1]) {
			for (real, stand_in) in &known {
				let end = i + real.len();
				if end <= text.len() && &text[i..end] == real.as_bytes() && (end == text.len() || !word(text[end])) {
					out.extend_from_slice(stand_in.as_bytes());
					i = end;
					continue 'text;
				}
			}
		}
		out.push(text[i]);
		i += 1;
	}
	out
}

fn is_hex(s: &[u8]) -> bool {
	s.iter().all(u8::is_ascii_hexdigit)
}

/// `digits` in the case `like` is written in (lower when it has a lower-case letter)
fn cased(digits: &str, like: &[u8]) -> String {
	if like.iter().any(u8::is_ascii_lowercase) {
		digits.to_ascii_lowercase()
	} else {
		digits.to_string()
	}
}

/// The serials: each word of a serial's shape as its stand-in. `valve`: the text is Valve's driver's (module doc)
fn serials(text: &[u8], names: &mut Names, valve: bool) -> Vec<u8> {
	let mut out = Vec::with_capacity(text.len());
	let mut i = 0;
	while i < text.len() {
		if !word(text[i]) || (i > 0 && word(text[i - 1])) {
			out.push(text[i]);
			i += 1;
			continue;
		}
		let mut end = i;
		while end < text.len() && word(text[end]) {
			end += 1;
		}
		let run = &text[i..end];
		// the word after a `-`, when the run is followed by one
		let after = |from: usize| {
			let mut e = from;
			while e < text.len() && word(text[e]) {
				e += 1;
			}
			&text[from..e]
		};
		let dash = end < text.len() && text[end] == b'-';
		let str_of = |b: &[u8]| String::from_utf8_lossy(b).into_owned();
		let fraction = i > 0 && text[i - 1] == b'.' && run.iter().all(u8::is_ascii_digit);

		// LHR-xxxxxxxx, LHB-xxxxxxxx
		if run.len() == 3 && (run.eq_ignore_ascii_case(b"LHR") || run.eq_ignore_ascii_case(b"LHB")) && dash {
			let d = after(end + 1);
			if d.len() == 8 && is_hex(d) {
				let kind = if run.eq_ignore_ascii_case(b"LHR") { Kind::Device } else { Kind::Station };
				let real = str_of(d);
				let stand_in =
					if names.is_stand_in(kind, &real) { real.clone() } else { names.digits(kind, &real, "") };
				out.extend_from_slice(run);
				out.push(b'-');
				out.extend_from_slice(cased(&stand_in, d).as_bytes());
				i = end + 1 + d.len();
				continue;
			}
		}
		// a dongle: ten hex digits, and `-` with three characters
		if run.len() == 10 && is_hex(run) && !fraction {
			let real = str_of(run);
			let tail = if dash { after(end + 1) } else { &[] };
			let whole = tail.len() == 3;
			let stand_in = if names.is_stand_in(Kind::Dongle, &real) {
				Some(real.clone())
			} else if whole {
				Some(names.digits(Kind::Dongle, &real, ""))
			} else {
				names.known(Kind::Dongle, &real).map(str::to_string)
			};
			if let Some(s) = stand_in {
				out.extend_from_slice(cased(&s, run).as_bytes());
				i = end;
				continue;
			}
		}
		// eight hex digits: a device's or a station's, bare or after `0x`
		let (prefix, digits) = match run {
			[b'0', b'x' | b'X', rest @ ..] if rest.len() == 8 => (&run[..2], rest),
			_ => (&run[..0], run),
		};
		if digits.len() == 8 && is_hex(digits) && !fraction {
			let real = str_of(digits);
			let mixed = digits.iter().any(u8::is_ascii_digit) && digits.iter().any(u8::is_ascii_alphabetic);
			let stand_in = if names.is_stand_in(Kind::Device, &real) || names.is_stand_in(Kind::Station, &real) {
				None
			} else if let Some(s) = names.known(Kind::Device, &real).or(names.known(Kind::Station, &real)) {
				Some(s.to_string())
			} else if valve && mixed {
				Some(names.digits(Kind::Station, &real, ""))
			} else {
				None
			};
			if let Some(s) = stand_in {
				out.extend_from_slice(prefix);
				out.extend_from_slice(cased(&s, digits).as_bytes());
				i = end;
				continue;
			}
		}
		// a base station's id as a number
		if run.len() <= 10 && run.iter().all(u8::is_ascii_digit) && !fraction {
			if let Ok(id) = str_of(run).parse::<u32>() {
				let real = format!("{id:08X}");
				if !names.is_stand_in(Kind::Station, &real) {
					if let Some(s) = names.known(Kind::Station, &real) {
						let n = u32::from_str_radix(s, 16).unwrap_or(0);
						out.extend_from_slice(n.to_string().as_bytes());
						i = end;
						continue;
					}
				}
			}
		}
		out.extend_from_slice(run);
		i = end;
	}
	out
}

/// `text` fit to share
pub fn scrub(text: &[u8], names: &mut Names, private: &Private, valve: bool) -> Vec<u8> {
	let text = others(&words(&paths(text, private), private), names);
	serials(&text, names, valve)
}

/// Whether `text` is fit to share: scrubbed again it is the same, and no serial in it is new to the table. `free`:
/// the user's and the PC's names are looked for as words too (a log; not a file of numbers and device names)
pub fn clean(text: &[u8], names: &Names, private: &Private, free: bool) -> bool {
	let mut again = names.clone();
	let before = again.len();
	let pathless = paths(text, private);
	let worded = if free { words(&pathless, private) } else { pathless };
	let out = serials(&others(&worded, names), &mut again, false);
	out == text && again.len() == before
}

#[cfg(test)]
mod tests {
	use super::*;

	fn private() -> Private {
		Private {
			user: "Robin".into(),
			pc: "DESKTOP-4KQ2".into(),
			home: "C:\\Users\\Robin".into(),
			sally: "C:\\Users\\Robin\\AppData\\Local\\sally".into(),
		}
	}

	fn s(text: &str, names: &mut Names, valve: bool) -> String {
		String::from_utf8(scrub(text.as_bytes(), names, &private(), valve)).unwrap()
	}

	#[test]
	fn serials_become_their_stand_ins_wherever_they_stand_as_words() {
		let mut n = Names::default();
		assert_eq!(
			s("KN0 (LHR-20304050): device 0A1B2C3D4E-6YX is KN0, lhr-20304050_config.json", &mut n, false),
			"KN0 (LHR-00000001): device 0000000001-6YX is KN0, lhr-00000001_config.json"
		);
		// a station by its id: after LHB-, bare in either case, and as a number
		n.station(0xb1c2_d3e4);
		assert_eq!(
			s(
				"LHB-B1C2D3E4 Got OOTX packet 9 b1c2d3e4 (ch 9, B1C2D3E4) \"id\":\"2982335460\" 0xb1c2d3e4",
				&mut n,
				false
			),
			"LHB-00000001 Got OOTX packet 9 00000001 (ch 9, 00000001) \"id\":\"1\" 0x00000001"
		);
		// a known dongle without its tail; an unknown ten digits stay
		assert_eq!(s("s/n 0a1b2c3d4e and 0123456789", &mut n, false), "s/n 0000000001 and 0123456789");
		// a longer word is not a serial, nor are a number's digits; a known serial's digits are, wherever they stand
		assert_eq!(
			s("xLHR-20304050 LHR-203040501 0.20304050 -1.2982335460 20304050", &mut n, false),
			"xLHR-00000001 LHR-203040501 0.20304050 -1.2982335460 00000001"
		);
	}

	#[test]
	fn valves_lines_name_stations_by_their_bare_ids() {
		let mut n = Names::default();
		assert_eq!(
			s(
				"LHR-20304050 C: ----- CALIBRATED base 7C8D9E0F at pitch 1 deg ----- e1f2a3b4 12345678 deadbeef",
				&mut n,
				true
			),
			"LHR-00000001 C: ----- CALIBRATED base 00000001 at pitch 1 deg ----- 00000002 12345678 deadbeef"
		);
		// the same words in a text that is not Valve's stay, but for the ones known by now
		assert_eq!(s("7C8D9E0F 0AB12CD3", &mut n, false), "00000001 0AB12CD3");
	}

	#[test]
	fn paths_and_names_lose_the_user() {
		let mut n = Names::default();
		assert_eq!(
			s("from C:\\Users\\Robin\\AppData\\Local\\sally\\data\\map_solve.txt", &mut n, false),
			"from <sally>\\data\\map_solve.txt"
		);
		assert_eq!(s("c:/users/robin/Downloads/x.zip", &mut n, false), "<home>/Downloads/x.zip");
		assert_eq!(s("\"C:\\\\Users\\\\Robin\\\\x\"", &mut n, false), "\"<home>\\\\x\"");
		assert_eq!(s("D:\\Users\\Someone Else\\Steam\\config", &mut n, false), "D:\\Users\\user\\Steam\\config");
		assert_eq!(s("C:\\Users\\Robinson\\x", &mut n, false), "C:\\Users\\user\\x");
		assert_eq!(s("host desktop-4kq2, user Robin; Robins", &mut n, false), "host <pc>, user <user>; Robins");
	}

	#[test]
	fn clean_text_is_text_that_scrubbing_leaves_alone() {
		let mut n = Names::default();
		let p = private();
		let dirty = "KN0 (LHR-20304050) on 0A1B2C3D4E-6YX at C:\\Users\\Robin\\x, LHB-B1C2D3E4";
		assert!(!clean(dirty.as_bytes(), &n, &p, true));
		let once = scrub(dirty.as_bytes(), &mut n, &p, false);
		assert!(clean(&once, &n, &p, true), "{}", String::from_utf8_lossy(&once));
		// each thing alone is found
		for bad in [
			"LHR-20304050",
			"20304050",
			"lhb-b1c2d3e4",
			"b1c2d3e4",
			"2982335460",
			"0A1B2C3D4E-6YX",
			"0a1b2c3d4e",
			"C:\\Users\\Robin",
			"c:/users/robin/AppData/Local/sally",
			"D:\\Users\\Other\\x",
			"LHR-0BADC0DE",
			"1111111111-1YX",
		] {
			assert!(!clean(bad.as_bytes(), &n, &p, true), "{bad}");
		}
		assert!(!clean(b"user Robin", &n, &p, true));
		assert!(clean(b"user Robin", &n, &p, false));
		assert!(clean(b"LHR-00000001 0000000001-6YX LHB-00000001 1 00000001 <home> <sally> <user>", &n, &p, true));
		// a device of another tracking system, by its whole serial
		let quest = n.serial("1PASH0000000", "hmd");
		n.serial("1PASH0000000_Controller_Left", "controller");
		assert_eq!(
			s("1PASH0000000 1PASH0000000_Controller_Left x1PASH0000000", &mut n, false),
			"hmd-1 controller-1 x1PASH0000000"
		);
		assert!(!clean(b"serial 1PASH0000000,", &n, &p, false));
		assert!(clean(format!("serial {quest},").as_bytes(), &n, &p, false));
	}
}
