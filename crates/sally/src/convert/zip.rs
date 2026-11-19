//! The files of a zip, as Python's zipfile lists and reads them for the converter: the central directory's entries
//! in its order (zip64 too, and a zip with something before it), each read whole from its local header, stored or
//! deflated, its CRC checked. Not read: encrypted entries, and the other compressions zipfile knows (bzip2, lzma).

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};

use sally_math::crc::crc32;

/// One file of the zip, as its central directory gives it
#[derive(Clone, Debug)]
pub struct Entry {
	pub name: String,
	flags: u16,
	method: u16,
	crc: u32,
	packed: u64,
	size: u64,
	/// its local header
	offset: u64,
}

fn u16_at(b: &[u8], i: usize) -> u16 {
	u16::from_le_bytes([b[i], b[i + 1]])
}

fn u32_at(b: &[u8], i: usize) -> u32 {
	u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]])
}

fn u64_at(b: &[u8], i: usize) -> u64 {
	u64::from_le_bytes(b[i..i + 8].try_into().unwrap_or_default())
}

const NOT_A_ZIP: &str = "File is not a zip file";

fn read_at(f: &mut File, at: u64, n: usize) -> Result<Vec<u8>, String> {
	let mut b = vec![0; n];
	f.seek(SeekFrom::Start(at)).map_err(|e| e.to_string())?;
	f.read_exact(&mut b).map_err(|_| NOT_A_ZIP.to_string())?;
	Ok(b)
}

/// ZipFile(path).infolist(): the end record found in the last 64 KB (a comment's room), the zip64 end record where its
/// locator is there, then the central directory
pub fn entries(f: &mut File) -> Result<Vec<Entry>, String> {
	let len = f.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
	let tail_len = len.min(22 + 65535) as usize;
	let tail = read_at(f, len - tail_len as u64, tail_len)?;
	let sig = 0x0605_4b50u32.to_le_bytes();
	let at = (0..tail.len().saturating_sub(21)).rev().find(|&i| tail[i..i + 4] == sig).ok_or(NOT_A_ZIP)?;
	let eocd_pos = len - tail_len as u64 + at as u64;
	let e = &tail[at..];
	let (mut count, mut cd_size, mut cd_offset) = (u16_at(e, 10) as u64, u32_at(e, 12) as u64, u32_at(e, 16) as u64);
	// concat: what comes before the zip (a self-extractor's stub), from where the end record is
	let mut concat = eocd_pos as i128 - cd_size as i128 - cd_offset as i128;
	// zip64: its locator just before the end record
	if eocd_pos >= 20 {
		let loc = read_at(f, eocd_pos - 20, 20)?;
		if u32_at(&loc, 0) == 0x0706_4b50 {
			let rec = read_at(f, u64_at(&loc, 8), 56).map_err(|_| "Corrupt zip64 end of central directory locator")?;
			if u32_at(&rec, 0) != 0x0606_4b50 {
				return Err("Bad magic number for zip64 end of central directory".into());
			}
			count = u64_at(&rec, 32);
			cd_size = u64_at(&rec, 40);
			cd_offset = u64_at(&rec, 48);
			concat = eocd_pos as i128 - 76 - cd_size as i128 - cd_offset as i128;
		}
	}
	if concat < 0 || cd_size > len {
		return Err(NOT_A_ZIP.into());
	}
	let cd = read_at(f, (cd_offset as i128 + concat) as u64, cd_size as usize)?;
	let mut out = Vec::new();
	let mut p = 0;
	while p + 46 <= cd.len() {
		if u32_at(&cd, p) != 0x0201_4b50 {
			return Err("Bad magic number for central directory".into());
		}
		let (name_len, extra_len, comment_len) =
			(u16_at(&cd, p + 28) as usize, u16_at(&cd, p + 30) as usize, u16_at(&cd, p + 32) as usize);
		let name = cd.get(p + 46..p + 46 + name_len).ok_or(NOT_A_ZIP)?;
		let extra = cd.get(p + 46 + name_len..p + 46 + name_len + extra_len).ok_or(NOT_A_ZIP)?;
		let mut ent = Entry {
			// UTF-8 where the entry says so (zipfile decodes the others as cp437; lossy UTF-8 here)
			name: String::from_utf8_lossy(name).into_owned(),
			flags: u16_at(&cd, p + 8),
			method: u16_at(&cd, p + 10),
			crc: u32_at(&cd, p + 16),
			packed: u32_at(&cd, p + 20) as u64,
			size: u32_at(&cd, p + 24) as u64,
			offset: u32_at(&cd, p + 42) as u64,
		};
		// zip64's extra field: the values that did not fit, in this order
		let mut x = 0;
		while x + 4 <= extra.len() {
			let (id, n) = (u16_at(extra, x), u16_at(extra, x + 2) as usize);
			if id == 1 {
				let mut v =
					extra[(x + 4).min(extra.len())..(x + 4 + n).min(extra.len())].chunks_exact(8).map(|c| u64_at(c, 0));
				for field in [&mut ent.size, &mut ent.packed, &mut ent.offset] {
					if *field == 0xffff_ffff {
						*field = v.next().ok_or("Corrupt extra field 0001 (zip64)")?;
					}
				}
			}
			x += 4 + n;
		}
		ent.offset = (ent.offset as i128 + concat) as u64;
		out.push(ent);
		p += 46 + name_len + extra_len + comment_len;
		if out.len() as u64 >= count && count != 0 {
			break;
		}
	}
	Ok(out)
}

/// ZipFile.read(name): the entry's data from its local header, inflated where deflated, checked against its CRC
pub fn read(f: &mut File, e: &Entry) -> Result<Vec<u8>, String> {
	if e.flags & 1 != 0 {
		return Err(format!("File {} is encrypted, password required for extraction", py_repr(&e.name)));
	}
	let local = read_at(f, e.offset, 30)?;
	if u32_at(&local, 0) != 0x0403_4b50 {
		return Err("Bad magic number for file header".into());
	}
	let start = e.offset + 30 + u16_at(&local, 26) as u64 + u16_at(&local, 28) as u64;
	let packed = read_at(f, start, e.packed as usize).map_err(|_| "Truncated file header".to_string())?;
	let data = match e.method {
		0 => packed,
		// no further than the size the entry gives (zipfile reads that much and checks the CRC of it)
		8 => match miniz_oxide::inflate::decompress_to_vec_with_limit(&packed, e.size as usize) {
			Ok(d) => d,
			Err(err) if err.status == miniz_oxide::inflate::TINFLStatus::HasMoreOutput => err.output,
			Err(_) => return Err("Error -3 while decompressing data: invalid deflate stream".into()),
		},
		_ => return Err("That compression method is not supported".into()),
	};
	if data.len() as u64 != e.size || crc32(&data) != e.crc {
		return Err(format!("Bad CRC-32 for file {}", py_repr(&e.name)));
	}
	Ok(data)
}

/// repr(str) for a plain name: in single quotes
fn py_repr(s: &str) -> String {
	if s.contains('\'') && !s.contains('"') {
		format!("\"{s}\"")
	} else {
		format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
	}
}

#[cfg(test)]
pub(crate) mod tests {
	use super::*;

	/// A zip of these files, each stored or deflated (`b"..."` names and data), with `prefix` before it and the
	/// comment after it
	pub fn zip_of(prefix: &[u8], files: &[(&str, &[u8], bool)], comment: &[u8]) -> Vec<u8> {
		let mut z = prefix.to_vec();
		let mut cd = Vec::new();
		for (name, data, deflate) in files {
			let packed = if *deflate { miniz_oxide::deflate::compress_to_vec(data, 6) } else { data.to_vec() };
			let offset = (z.len() - prefix.len()) as u32;
			let head = |sig: u32, central: bool| {
				let mut h = sig.to_le_bytes().to_vec();
				if central {
					h.extend_from_slice(&20u16.to_le_bytes());
				}
				h.extend_from_slice(&20u16.to_le_bytes());
				h.extend_from_slice(&0u16.to_le_bytes());
				h.extend_from_slice(&(if *deflate { 8u16 } else { 0 }).to_le_bytes());
				h.extend_from_slice(&[0; 4]);
				h.extend_from_slice(&crc32(data).to_le_bytes());
				h.extend_from_slice(&(packed.len() as u32).to_le_bytes());
				h.extend_from_slice(&(data.len() as u32).to_le_bytes());
				h.extend_from_slice(&(name.len() as u16).to_le_bytes());
				h.extend_from_slice(&0u16.to_le_bytes());
				if central {
					// comment length, disk, internal and external attributes
					h.extend_from_slice(&[0; 10]);
					h.extend_from_slice(&offset.to_le_bytes());
				}
				h.extend_from_slice(name.as_bytes());
				h
			};
			z.extend(head(0x0403_4b50, false));
			z.extend(&packed);
			cd.extend(head(0x0201_4b50, true));
		}
		let cd_offset = (z.len() - prefix.len()) as u32;
		z.extend(&cd);
		z.extend(0x0605_4b50u32.to_le_bytes());
		z.extend([0; 4]);
		z.extend((files.len() as u16).to_le_bytes());
		z.extend((files.len() as u16).to_le_bytes());
		z.extend((cd.len() as u32).to_le_bytes());
		z.extend(cd_offset.to_le_bytes());
		z.extend((comment.len() as u16).to_le_bytes());
		z.extend(comment);
		z
	}

	#[test]
	fn a_zips_files_read_back() {
		let dir = std::env::temp_dir().join(format!("sally-convert-zip-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let big: Vec<u8> = (0..20000u32).map(|i| (i * 7 % 251) as u8).collect();
		for (prefix, comment) in [(&b""[..], &b""[..]), (b"MZ stub", b"a comment")] {
			let path = dir.join("x.zip");
			std::fs::write(&path, zip_of(prefix, &[("a.txt", b"hello", false), ("d/b.lhraw", &big, true)], comment))
				.unwrap();
			let mut f = File::open(&path).unwrap();
			let es = entries(&mut f).unwrap();
			assert_eq!(es.iter().map(|e| e.name.as_str()).collect::<Vec<_>>(), ["a.txt", "d/b.lhraw"]);
			assert_eq!(read(&mut f, &es[0]).unwrap(), b"hello");
			assert_eq!(read(&mut f, &es[1]).unwrap(), big);
			// a bad CRC
			let mut bad = es[1].clone();
			bad.crc ^= 1;
			assert_eq!(read(&mut f, &bad).unwrap_err(), "Bad CRC-32 for file 'd/b.lhraw'");
		}
		std::fs::write(dir.join("y.zip"), b"not a zip at all, not at all").unwrap();
		assert_eq!(entries(&mut File::open(dir.join("y.zip")).unwrap()).unwrap_err(), NOT_A_ZIP);
		let _ = std::fs::remove_dir_all(&dir);
	}
}
