//! What the installer carries: sally's files, packed after the program itself.
//!
//! The installer is one file: the program, then the payload, then a footer. The payload is the files one after
//! another: the path (a u16 length, then UTF-8 with '/' between folders), the file's length and its packed length
//! (u32 each), and its bytes deflated. The footer is the payload's length and its check (u64 each) and a mark. The
//! mark is looked for from the file's end backwards and believed only where the length and the check hold, so what a
//! code signature adds after the footer does not hide it, and the mark's own bytes inside the program are not taken
//! for one. `sally-release` packs (docs/release.md), the installer unpacks.
//!
//! `folder` says what in sally's folder is sally's: the installer puts and removes nothing else there.

pub mod folder {
	/// The first parts of the paths the installer puts in sally's folder: the programs (as Windows names them, or
	/// another system), OpenVR's library, the driver's folder, the icon, the licenses, and the installer's own copy
	/// there. `sally-release` packs nothing else
	const PUT: [&str; 9] = [
		"sally",
		"sally-panel",
		"uninstall",
		"openvr_api.dll",
		"libopenvr_api.so",
		"driver",
		"sally.ico",
		"LICENSE.txt",
		"THIRD-PARTY-NOTICES.txt",
	];
	/// What sally makes there: what it learned and what was set (`data`), what sessions leave (`logs`), the panel's
	/// replays, its checks and its settings
	const MADE: [&str; 5] = ["data", "logs", "replays", "sally-check recordings", "sally-panel.cfg"];

	/// `name` without what an install adds to a file it sets aside (`.old-<n>`) or has not finished writing (`.new`)
	pub fn settled(name: &str) -> &str {
		if let Some(whole) = name.strip_suffix(".new") {
			return whole;
		}
		match name.rsplit_once(".old-") {
			Some((whole, n)) if !n.is_empty() && n.bytes().all(|c| c.is_ascii_digit()) => whole,
			_ => name,
		}
	}

	fn among(names: &[&str], name: &str) -> bool {
		names.iter().any(|n| n.eq_ignore_ascii_case(name))
	}

	/// Whether a name in sally's folder is one the installer puts there (or what an install set aside of one)
	pub fn puts(name: &str) -> bool {
		let name = settled(name);
		among(&PUT, name) || name.strip_suffix(".exe").is_some_and(|program| among(&PUT[..3], program))
	}

	/// Whether a name in sally's folder is sally's: one the installer puts there, or one sally makes
	pub fn sallys(name: &str) -> bool {
		puts(name) || among(&MADE, settled(name))
	}

	#[cfg(test)]
	mod tests {
		use super::*;

		#[test]
		fn sallys_own_names_in_its_folder() {
			for name in
				["sally.exe", "Sally-Panel.exe", "sally", "uninstall.exe", "driver", "openvr_api.dll", "LICENSE.txt"]
			{
				assert!(puts(name) && sallys(name), "{name}");
			}
			// what an install set aside or left half written of one
			for name in ["sally.exe.old-0", "openvr_api.dll.old-12", "sally-panel.exe.new", "sally.ico.new"] {
				assert!(puts(name) && settled(name) != name, "{name}");
			}
			for name in ["data", "logs", "replays", "sally-check recordings", "sally-panel.cfg"] {
				assert!(sallys(name) && !puts(name), "{name}");
			}
			for name in [
				"notes.txt",
				"sally.exe.bak",
				"driver.zip",
				"data2",
				"photos",
				"x.new",
				"x.old-1",
				"sally.exe.old-",
				"sally.exe.old-1a",
			] {
				assert!(!sallys(name), "{name}");
			}
			assert_eq!(
				(settled("a.old-3"), settled("a.new"), settled("a.old-x"), settled("a.old-")),
				("a", "a", "a.old-x", "a.old-")
			);
		}
	}
}

pub mod payload {
	/// The footer's last 16 bytes
	pub const MARK: &[u8; 16] = b"sally-payload-1\0";
	/// The footer: the payload's length, its check, the mark
	const FOOTER: usize = 8 + 8 + MARK.len();

	/// One file of the payload: its path under sally's folder ('/' between folders) and its bytes
	#[derive(Clone, Debug, PartialEq)]
	pub struct File {
		pub path: String,
		pub bytes: Vec<u8>,
	}

	/// FNV-1a over the payload: a wrong footer is told from a right one, nothing more
	fn check(bytes: &[u8]) -> u64 {
		bytes.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, &b| (h ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3))
	}

	/// A path the installer writes: under sally's folder and nowhere else
	fn sound(path: &str) -> bool {
		!path.is_empty()
			&& !path.starts_with('/')
			&& !path.contains(['\\', ':'])
			&& path.split('/').all(|part| !part.is_empty() && part != "." && part != "..")
	}

	/// The files packed, with the footer: what goes after the installer program's own bytes
	pub fn pack(files: &[File]) -> Result<Vec<u8>, String> {
		let mut out = Vec::new();
		for f in files {
			if !sound(&f.path) || f.path.len() > u16::MAX as usize {
				return Err(format!("not a path for the installer: {:?}", f.path));
			}
			let raw = u32::try_from(f.bytes.len()).map_err(|_| format!("{} is too large", f.path))?;
			let packed = miniz_oxide::deflate::compress_to_vec(&f.bytes, 9);
			let packed_len = u32::try_from(packed.len()).map_err(|_| format!("{} is too large", f.path))?;
			out.extend((f.path.len() as u16).to_le_bytes());
			out.extend(f.path.as_bytes());
			out.extend(raw.to_le_bytes());
			out.extend(packed_len.to_le_bytes());
			out.extend(packed);
		}
		let (len, sum) = (out.len() as u64, check(&out));
		out.extend(len.to_le_bytes());
		out.extend(sum.to_le_bytes());
		out.extend(MARK);
		Ok(out)
	}

	/// Where an installer's payload is: the program's own length, and the payload's bytes. None for a program with
	/// nothing after it (the uninstaller the installer leaves in sally's folder)
	pub fn find(installer: &[u8]) -> Option<(usize, &[u8])> {
		let mut end = installer.len();
		while let Some(at) = installer[..end].windows(MARK.len()).rposition(|w| w == MARK) {
			let word = |from: usize| -> Option<u64> {
				Some(u64::from_le_bytes(installer.get(from..from + 8)?.try_into().ok()?))
			};
			if at >= FOOTER - MARK.len() {
				let foot = at - (FOOTER - MARK.len());
				if let (Some(len), Some(sum)) = (word(foot), word(foot + 8)) {
					if let Some(start) = usize::try_from(len).ok().and_then(|len| foot.checked_sub(len)) {
						if check(&installer[start..foot]) == sum {
							return Some((start, &installer[start..foot]));
						}
					}
				}
			}
			// the mark's bytes somewhere else (the program holds them too): on to the one before
			end = at + MARK.len() - 1;
		}
		None
	}

	/// The payload's files
	pub fn unpack(payload: &[u8]) -> Result<Vec<File>, String> {
		let broken = || "the installer's files are damaged: download it again".to_string();
		let mut files = Vec::new();
		let mut i = 0;
		let take = |i: &mut usize, n: usize| -> Option<&[u8]> {
			let s = payload.get(*i..i.checked_add(n)?)?;
			*i += n;
			Some(s)
		};
		while i < payload.len() {
			let path_len = u16::from_le_bytes(take(&mut i, 2).ok_or_else(broken)?.try_into().unwrap()) as usize;
			let path = std::str::from_utf8(take(&mut i, path_len).ok_or_else(broken)?).map_err(|_| broken())?;
			let raw = u32::from_le_bytes(take(&mut i, 4).ok_or_else(broken)?.try_into().unwrap()) as usize;
			let packed = u32::from_le_bytes(take(&mut i, 4).ok_or_else(broken)?.try_into().unwrap()) as usize;
			let bytes =
				miniz_oxide::inflate::decompress_to_vec_with_limit(take(&mut i, packed).ok_or_else(broken)?, raw)
					.map_err(|_| broken())?;
			if bytes.len() != raw || !sound(path) {
				return Err(broken());
			}
			files.push(File { path: path.to_string(), bytes });
		}
		Ok(files)
	}

	#[cfg(test)]
	mod tests {
		use super::*;

		fn files() -> Vec<File> {
			vec![
				File {
					path: "sally.exe".into(),
					bytes: (0..50_000u32).flat_map(|n| (n % 251).to_le_bytes()).collect(),
				},
				File { path: "driver/bin/win64/driver_sally.dll".into(), bytes: vec![7; 3000] },
				File { path: "LICENSE.txt".into(), bytes: Vec::new() },
			]
		}

		/// The installer is the program with the payload after it: found again, with the program's length, whatever
		/// follows the footer (a code signature), and although the program itself holds the mark's bytes
		#[test]
		fn the_payload_is_found_after_the_program_and_unpacks_to_the_files() {
			let mut program = vec![0x4d, 0x5a];
			program.extend(MARK);
			program.extend([9u8; 100]);
			assert_eq!(find(&program), None);
			let packed = pack(&files()).unwrap();
			assert!(packed.len() < 60_000, "{}", packed.len());
			for signature in [0usize, 9000] {
				let mut installer = program.clone();
				installer.extend(&packed);
				installer.extend(vec![0xa5u8; signature]);
				let (len, payload) = find(&installer).unwrap();
				assert_eq!(len, program.len());
				assert_eq!(unpack(payload).unwrap(), files());
			}
			// cut short, or changed: not a payload
			let mut installer = program.clone();
			installer.extend(&packed);
			let n = installer.len();
			assert_eq!(find(&installer[..n - 1]), None);
			installer[program.len() + 40] ^= 1;
			assert_eq!(find(&installer), None);
		}

		#[test]
		fn only_paths_under_sallys_folder_are_packed_and_unpacked() {
			for bad in ["", "/x", "..", "a/../b", "a//b", "a\\b", "C:/x", "a/./b", "a/"] {
				assert!(pack(&[File { path: bad.into(), bytes: vec![1] }]).is_err(), "{bad:?}");
			}
			// a payload naming one (not one pack makes) does not unpack
			let mut p = Vec::new();
			p.extend(4u16.to_le_bytes());
			p.extend(b"../x");
			let packed = miniz_oxide::deflate::compress_to_vec(b"hi", 9);
			p.extend(2u32.to_le_bytes());
			p.extend((packed.len() as u32).to_le_bytes());
			p.extend(packed);
			assert!(unpack(&p).is_err());
			assert!(unpack(&[1, 2, 3]).is_err());
			assert_eq!(unpack(&[]).unwrap(), Vec::new());
		}
	}
}
