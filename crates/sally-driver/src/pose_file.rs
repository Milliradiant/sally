//! The pose log's file: its rows as text, or packed (`sally_pack::table`: the same text in about an eighteenth of the
//! bytes, and read back to it) where the file's name ends in `.csvz`, which is the name `sally setup` gives a
//! session's. Either way it is the C library's stream underneath (`cstdio`), written out at most once a second.

use std::io;

use sally_pack::table::{PackWriter, PACKED_END};

use crate::cstdio::CFile;

/// The stream as what a packed table is written to
pub struct Stream(CFile);

impl io::Write for Stream {
	fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
		self.0.write(bytes);
		Ok(bytes.len())
	}

	fn flush(&mut self) -> io::Result<()> {
		self.0.flush();
		Ok(())
	}
}

pub enum PoseFile {
	Text(CFile),
	Packed(Box<PackWriter<Stream>>),
}

impl PoseFile {
	/// The file at `path` opened for writing from its start; None where it cannot be
	pub fn open(path: &[u8]) -> Option<PoseFile> {
		let file = CFile::open(path, c"wb")?;
		if path.ends_with(PACKED_END.as_bytes()) {
			PackWriter::new(Stream(file)).ok().map(|packed| PoseFile::Packed(Box::new(packed)))
		} else {
			Some(PoseFile::Text(file))
		}
	}

	/// More of the log's text: a row is handed over in pieces, its newline last
	pub fn write(&mut self, text: &[u8]) {
		match self {
			PoseFile::Text(file) => file.write(text),
			PoseFile::Packed(packed) => {
				let _ = packed.write(text);
			}
		}
	}

	/// What has been handed over is in the file: the stream's buffer written out, and of a packed log the rows since
	/// the last time as a block of their own (a block is whole or not there, so a log reads up to here whatever
	/// becomes of its writer)
	pub fn flush(&mut self) {
		match self {
			PoseFile::Text(file) => file.flush(),
			PoseFile::Packed(packed) => {
				let _ = packed.end();
			}
		}
	}
}

#[cfg(test)]
mod tests {
	use std::io::Read;

	use super::*;

	/// Rows written in pieces and flushed now and then are in the file as their text, and packed as that text
	#[test]
	fn a_log_is_its_text_in_either_form() {
		let dir = std::env::temp_dir().join(format!("sally_pose_file_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(&dir).unwrap();
		let mut text = b"t,id,serial,x,qw\n".to_vec();
		for i in 0..3000 {
			text.extend_from_slice(
				format!(
					"{:.6},{},LHR-0000000{},{:.5},{:.6}\n",
					100.0 + i as f64 * 0.011,
					i % 3,
					i % 3,
					i as f64 * 0.001,
					0.5
				)
				.as_bytes(),
			);
		}
		for name in ["poses_20261118_154450.csv", "poses_20261118_154450.csvz"] {
			let path = dir.join(name);
			let mut file = PoseFile::open(path.to_str().unwrap().as_bytes()).unwrap();
			assert_eq!(matches!(file, PoseFile::Packed(_)), name.ends_with('z'));
			for (i, line) in text.split_inclusive(|&c| c == b'\n').enumerate() {
				let (a, b) = line.split_at(line.len() - 1);
				file.write(a);
				file.write(b);
				if i % 640 == 639 {
					file.flush();
				}
			}
			file.flush();
			let mut got = Vec::new();
			sally_pack::table::open(&path).unwrap().read_to_end(&mut got).unwrap();
			assert!(got == text, "{name}");
			let size = std::fs::metadata(&path).unwrap().len() as usize;
			assert!(if name.ends_with('z') { size * 8 < text.len() } else { size == text.len() }, "{name}: {size}");
		}
		std::fs::remove_dir_all(&dir).unwrap();
	}
}
