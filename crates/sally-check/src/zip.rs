//! A folder of files as one zip (deflated, no zip64: every file under 4 GB), and a zip of ours read back into a
//! folder, so a check can be sent as one file and opened again by dropping it on sally-check.

// zipping is the recorder's, which is Windows only
#![cfg_attr(not(windows), allow(dead_code))]

use std::fs;
use std::io::{self, Write};
use std::path::Path;

/// CRC-32 (IEEE), as zip has it
fn crc32(data: &[u8]) -> u32 {
	static TABLE: std::sync::OnceLock<[u32; 256]> = std::sync::OnceLock::new();
	let t = TABLE.get_or_init(|| {
		let mut t = [0u32; 256];
		for (i, e) in t.iter_mut().enumerate() {
			let mut c = i as u32;
			for _ in 0..8 {
				c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
			}
			*e = c;
		}
		t
	});
	!data.iter().fold(!0u32, |c, &b| t[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8))
}

/// The DOS date and time of a zip entry, from the system clock (UTC: zip has no time zone)
fn dos_time() -> (u16, u16) {
	let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
	let days = (secs / 86_400) as i64;
	let rem = secs % 86_400;
	// civil from days (Howard Hinnant's algorithm)
	let z = days + 719_468;
	let era = z.div_euclid(146_097);
	let doe = z - era * 146_097;
	let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
	let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	let mp = (5 * doy + 2) / 153;
	let d = doy - (153 * mp + 2) / 5 + 1;
	let m = if mp < 10 { mp + 3 } else { mp - 9 };
	let y = yoe + era * 400 + if m <= 2 { 1 } else { 0 };
	let date = (((y - 1980).max(0) as u16) << 9) | ((m as u16) << 5) | d as u16;
	let time = (((rem / 3600) as u16) << 11) | ((((rem % 3600) / 60) as u16) << 5) | ((rem % 60) / 2) as u16;
	(time, date)
}

/// Zips every file directly in `dir` into `out`, each under its own name
pub fn zip_folder(dir: &Path, out: &Path) -> io::Result<()> {
	let mut names: Vec<String> = fs::read_dir(dir)?
		.filter_map(|e| e.ok())
		.filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
		.map(|e| e.file_name().to_string_lossy().into_owned())
		.collect();
	names.sort();
	let (time, date) = dos_time();
	let mut f = io::BufWriter::new(fs::File::create(out)?);
	let mut central = Vec::new();
	let mut offset: u64 = 0;
	for name in &names {
		let data = fs::read(dir.join(name))?;
		let packed = miniz_oxide::deflate::compress_to_vec(&data, 6);
		let crc = crc32(&data);
		if data.len() as u64 > u32::MAX as u64 || offset > u32::MAX as u64 {
			return Err(io::Error::other("a file over 4 GB"));
		}
		let mut local = Vec::with_capacity(30 + name.len());
		local.extend_from_slice(&0x0403_4b50u32.to_le_bytes());
		local.extend_from_slice(&20u16.to_le_bytes()); // version needed
		local.extend_from_slice(&0x0800u16.to_le_bytes()); // UTF-8 names
		local.extend_from_slice(&8u16.to_le_bytes()); // deflate
		local.extend_from_slice(&time.to_le_bytes());
		local.extend_from_slice(&date.to_le_bytes());
		local.extend_from_slice(&crc.to_le_bytes());
		local.extend_from_slice(&(packed.len() as u32).to_le_bytes());
		local.extend_from_slice(&(data.len() as u32).to_le_bytes());
		local.extend_from_slice(&(name.len() as u16).to_le_bytes());
		local.extend_from_slice(&0u16.to_le_bytes());
		local.extend_from_slice(name.as_bytes());
		f.write_all(&local)?;
		f.write_all(&packed)?;

		central.extend_from_slice(&0x0201_4b50u32.to_le_bytes());
		central.extend_from_slice(&20u16.to_le_bytes()); // made by
		central.extend_from_slice(&20u16.to_le_bytes());
		central.extend_from_slice(&0x0800u16.to_le_bytes());
		central.extend_from_slice(&8u16.to_le_bytes());
		central.extend_from_slice(&time.to_le_bytes());
		central.extend_from_slice(&date.to_le_bytes());
		central.extend_from_slice(&crc.to_le_bytes());
		central.extend_from_slice(&(packed.len() as u32).to_le_bytes());
		central.extend_from_slice(&(data.len() as u32).to_le_bytes());
		central.extend_from_slice(&(name.len() as u16).to_le_bytes());
		central.extend_from_slice(&[0; 12]); // extra, comment, disk, internal attributes, external attributes
		central.extend_from_slice(&(offset as u32).to_le_bytes());
		central.extend_from_slice(name.as_bytes());
		offset += local.len() as u64 + packed.len() as u64;
	}
	if offset > u32::MAX as u64 {
		return Err(io::Error::other("zip over 4 GB"));
	}
	f.write_all(&central)?;
	f.write_all(&0x0605_4b50u32.to_le_bytes())?;
	f.write_all(&[0; 4])?; // disk numbers
	f.write_all(&(names.len() as u16).to_le_bytes())?;
	f.write_all(&(names.len() as u16).to_le_bytes())?;
	f.write_all(&(central.len() as u32).to_le_bytes())?;
	f.write_all(&(offset as u32).to_le_bytes())?;
	f.write_all(&0u16.to_le_bytes())?;
	f.flush()
}

/// The most a file of a zip unpacks to, and all of them together (bytes): the largest file of a session is its tap
/// recording, some 1.7 GB for the longest one (20 minutes) of 16 devices, and the rest of the folder is less than that
/// again. A zip from someone else says what its files unpack to and may not mean it (deflate packs a GB of zeros into
/// a MB): unpacked whole and unbounded, it took the memory the zip asked for
const MAX_FILE: usize = 2 << 30;
const MAX_TOTAL: usize = 4 << 30;

/// Unpacks a zip (stored or deflated entries, flat) into `dir`; names with a path in them are refused, and so is a file
/// that does not unpack to the size the zip gives it, or to more than MAX_FILE (MAX_TOTAL all of them)
pub fn unzip(zip: &Path, dir: &Path) -> io::Result<()> {
	unzip_within(zip, dir, MAX_FILE, MAX_TOTAL)
}

/// unzip, with the limits given
fn unzip_within(zip: &Path, dir: &Path, max_file: usize, max_total: usize) -> io::Result<()> {
	let z = fs::read(zip)?;
	let bad = || io::Error::new(io::ErrorKind::InvalidData, "not a zip sally-check can read");
	let too_big = || io::Error::new(io::ErrorKind::InvalidData, "a file in the zip is larger than a recording's");
	let eocd =
		(0..z.len().saturating_sub(21)).rev().find(|&i| z[i..i + 4] == 0x0605_4b50u32.to_le_bytes()).ok_or_else(bad)?;
	let u16_at = |i: usize| u16::from_le_bytes([z[i], z[i + 1]]) as usize;
	let u32_at = |i: usize| u32::from_le_bytes([z[i], z[i + 1], z[i + 2], z[i + 3]]) as usize;
	let count = u16_at(eocd + 10);
	let mut p = u32_at(eocd + 16);
	fs::create_dir_all(dir)?;
	let mut total = 0usize;
	for _ in 0..count {
		if p + 46 > z.len() || u32_at(p) != 0x0201_4b50 {
			return Err(bad());
		}
		let method = u16_at(p + 10);
		let packed_len = u32_at(p + 20);
		// what it unpacks to, as the central directory says
		let size = u32_at(p + 24);
		let (name_len, extra_len, comment_len) = (u16_at(p + 28), u16_at(p + 30), u16_at(p + 32));
		let local = u32_at(p + 42);
		let name = String::from_utf8_lossy(z.get(p + 46..p + 46 + name_len).ok_or_else(bad)?).into_owned();
		p += 46 + name_len + extra_len + comment_len;
		if name.ends_with('/') {
			continue;
		}
		if name.contains(['/', '\\']) || name.contains("..") || name.contains(':') {
			return Err(bad());
		}
		if local + 30 > z.len() {
			return Err(bad());
		}
		let start = local + 30 + u16_at(local + 26) + u16_at(local + 28);
		let packed = z.get(start..start + packed_len).ok_or_else(bad)?;
		if size > max_file || total + size > max_total {
			return Err(too_big());
		}
		let data = match method {
			0 => packed.to_vec(),
			// no further than the size it says (the end of the data needs no room past it)
			8 => miniz_oxide::inflate::decompress_to_vec_with_limit(packed, size).map_err(|_| bad())?,
			_ => return Err(bad()),
		};
		if data.len() != size {
			return Err(bad());
		}
		total += size;
		fs::write(dir.join(&name), data)?;
	}
	Ok(())
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn crc_is_zips() {
		assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
	}

	#[test]
	fn a_folder_zips_and_unzips() {
		let base = std::env::temp_dir().join(format!("sally-check-zip-{}", std::process::id()));
		let (a, b) = (base.join("a"), base.join("b"));
		fs::create_dir_all(&a).unwrap();
		fs::write(a.join("one.txt"), "hello hello hello hello").unwrap();
		fs::write(a.join("two.bin"), (0..5000u32).map(|i| (i * 7) as u8).collect::<Vec<_>>()).unwrap();
		zip_folder(&a, &base.join("x.zip")).unwrap();
		unzip(&base.join("x.zip"), &b).unwrap();
		assert_eq!(fs::read(a.join("one.txt")).unwrap(), fs::read(b.join("one.txt")).unwrap());
		assert_eq!(fs::read(a.join("two.bin")).unwrap(), fs::read(b.join("two.bin")).unwrap());
		let _ = fs::remove_dir_all(&base);
	}

	#[test]
	fn a_file_unpacks_to_the_size_it_says_and_within_the_limits() {
		let base = std::env::temp_dir().join(format!("sally-check-zip-limits-{}", std::process::id()));
		let a = base.join("a");
		fs::create_dir_all(&a).unwrap();
		// 4 MB of zeros (a few KB deflated), and a file of nothing
		fs::write(a.join("zeros.bin"), vec![0u8; 4 << 20]).unwrap();
		fs::write(a.join("empty.txt"), "").unwrap();
		let zip = base.join("x.zip");
		zip_folder(&a, &zip).unwrap();
		let within = |dir: &str, file: usize, total: usize| unzip_within(&zip, &base.join(dir), file, total);
		within("b", 8 << 20, 8 << 20).unwrap();
		assert_eq!(fs::read(base.join("b").join("zeros.bin")).unwrap().len(), 4 << 20);
		assert!(fs::read(base.join("b").join("empty.txt")).unwrap().is_empty());
		// a file past the limit, or the files together: refused
		assert_eq!(within("c", 1 << 20, 8 << 20).unwrap_err().kind(), io::ErrorKind::InvalidData);
		assert_eq!(within("d", 8 << 20, 3 << 20).unwrap_err().kind(), io::ErrorKind::InvalidData);
		// a zip that says its file is smaller than what its data unpacks to (a bomb's way: says 4 KB, unpacks to
		// anything), or larger: refused, and nothing past what it said was made
		for said in [4096u32, (4 << 20) + 1] {
			let mut z = fs::read(&zip).unwrap();
			// the central directory from the end record (22 bytes, no comment); zeros.bin is the second of the names,
			// sorted: its entry follows empty.txt's
			let eocd = z.len() - 22;
			let cd = u32::from_le_bytes(z[eocd + 16..eocd + 20].try_into().unwrap()) as usize;
			let second = cd + 46 + "empty.txt".len();
			assert_eq!(&z[second..second + 4], 0x0201_4b50u32.to_le_bytes());
			z[second + 24..second + 28].copy_from_slice(&said.to_le_bytes());
			let bomb = base.join("bomb.zip");
			fs::write(&bomb, &z).unwrap();
			let out = base.join("e");
			assert!(unzip_within(&bomb, &out, 8 << 20, 8 << 20).is_err(), "said {said}");
			assert!(!out.join("zeros.bin").exists());
		}
		let _ = fs::remove_dir_all(&base);
	}
}
