//! A device's config as it streams out of it: feature report 0x10 starts it, each 0x11 carries a chunk of the
//! zlib-compressed JSON (a count byte and the bytes), and a chunk of no bytes ends it (survive_vive_report_config_start
//! and _chunk of survive_vive_report.c; the inflate is survive.c's survive_simple_inflate).

/// survive_simple_inflate's output limit: the 65536-byte buffer less one (the JSON's terminator)
pub const MAX_CONFIG_LEN: usize = 65536 - 1;

/// The compressed bytes so far (vive_device_inst_t's compressed_data and its uint16_t index)
#[derive(Clone, Debug)]
pub struct ConfigStream {
	data: Box<[u8; 65536]>,
	idx: u16,
}

/// What a chunk did
#[derive(Debug, PartialEq)]
pub enum Chunk {
	/// bytes taken (or the device's leading non-zlib chunk skipped, or a chunk that would not fit dropped)
	Taken,
	/// the end of the stream: the inflated config, or why it did not inflate
	End(Result<Vec<u8>, InflateError>),
}

/// Why the stream did not inflate, as zlib would say it (survive_simple_inflate's warning prints both)
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InflateError {
	/// zlib's z_stream.msg (None where zlib leaves it null, which glibc's printf shows as "(null)")
	pub msg: Option<&'static str>,
	/// zlib's return code
	pub code: i32,
}

impl Default for ConfigStream {
	fn default() -> Self {
		ConfigStream { data: Box::new([0; 65536]), idx: 0 }
	}
}

impl ConfigStream {
	/// survive_vive_report_config_start: a new config begins
	pub fn start(&mut self) {
		self.idx = 0;
	}

	/// survive_vive_report_config_chunk: a 0x11 report (the whole report, its id first; at least two bytes)
	pub fn chunk(&mut self, data: &[u8]) -> Chunk {
		let len = data.len();
		debug_assert!(len >= 2);
		let mut cnt = data[1] as u16;
		// the report's own length field must not point past the captured bytes (malformed capture)
		if cnt as usize > len - 2 {
			cnt = (len - 2) as u16;
		}
		if cnt != 0 {
			// Some (Tracker at least?) devices send a uint64_t before data; not sure what it means but skip it for now.
			if self.idx == 0 && cnt >= 2 && data[2] != 0x78 {
			} else if self.idx as usize + cnt as usize <= self.data.len() {
				let at = self.idx as usize;
				self.data[at..at + cnt as usize].copy_from_slice(&data[2..2 + cnt as usize]);
				// the C's index is a uint16_t: filling the buffer exactly wraps it to 0
				self.idx = self.idx.wrapping_add(cnt);
			}
			Chunk::Taken
		} else {
			Chunk::End(simple_inflate(&self.data[..self.idx as usize], MAX_CONFIG_LEN))
		}
	}

	/// The compressed bytes taken so far (what survive_simple_inflate writes to libz_error.stream when it fails)
	pub fn compressed(&self) -> &[u8] {
		&self.data[..self.idx as usize]
	}
}

/// survive_simple_inflate: a zlib stream inflated whole into at most `limit` bytes. miniz_oxide inflates exactly what
/// zlib does; where it fails, zlib's code is given for what failed, and zlib's message where only one fits (a
/// malformed deflate block has a dozen messages in zlib that miniz_oxide does not tell apart: those read "invalid
/// stream", which zlib never says).
pub fn simple_inflate(input: &[u8], limit: usize) -> Result<Vec<u8>, InflateError> {
	use miniz_oxide::inflate::TINFLStatus;
	match miniz_oxide::inflate::decompress_to_vec_zlib_with_limit(input, limit) {
		Ok(v) => Ok(v),
		Err(e) => Err(match e.status {
			// out of input, or out of room: Z_BUF_ERROR, with no message
			TINFLStatus::FailedCannotMakeProgress | TINFLStatus::NeedsMoreInput | TINFLStatus::HasMoreOutput => {
				InflateError { msg: None, code: -5 }
			}
			TINFLStatus::Adler32Mismatch => InflateError { msg: Some("incorrect data check"), code: -3 },
			_ => InflateError { msg: Some("invalid stream"), code: -3 },
		}),
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_streamed_config_inflates() {
		let json = b"{ \"device_class\": \"controller\" }\r\n".repeat(40);
		let z = miniz_oxide::deflate::compress_to_vec_zlib(&json, 6);
		let mut s = ConfigStream::default();
		s.start();
		// a tracker's leading chunk that is not zlib is skipped
		assert_eq!(s.chunk(&[0x11, 8, 1, 2, 3, 4, 5, 6, 7, 8]), Chunk::Taken);
		for part in z.chunks(62) {
			let mut r = vec![0x11, part.len() as u8];
			r.extend_from_slice(part);
			assert_eq!(s.chunk(&r), Chunk::Taken);
		}
		assert_eq!(s.chunk(&[0x11, 0]), Chunk::End(Ok(json.clone())));
		// truncated: zlib's buffer error
		s.start();
		let mut r = vec![0x11, 40];
		r.extend_from_slice(&z[..40]);
		s.chunk(&r);
		assert_eq!(s.chunk(&[0x11, 0]), Chunk::End(Err(InflateError { msg: None, code: -5 })));
	}
}
