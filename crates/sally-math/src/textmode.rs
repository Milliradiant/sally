//! The C library's text mode on Windows (the ucrt's), which the C's files and streams went through and sally's still
//! do: a file opened with "r" or "w", stdout and stderr.

/// A text-mode read: CR LF becomes LF, and a Ctrl-Z (0x1A) ends the text
pub fn read(bytes: &[u8]) -> Vec<u8> {
	let mut out = Vec::with_capacity(bytes.len());
	let mut i = 0;
	while i < bytes.len() {
		match bytes[i] {
			0x1A => break,
			b'\r' if bytes.get(i + 1) == Some(&b'\n') => {}
			c => out.push(c),
		}
		i += 1;
	}
	out
}

/// A text-mode write: each LF goes out as CR LF (a CR already before it stays)
pub fn write(bytes: &[u8]) -> Vec<u8> {
	let mut out = Vec::with_capacity(bytes.len() + bytes.len() / 16);
	for &c in bytes {
		if c == b'\n' {
			out.push(b'\r');
		}
		out.push(c);
	}
	out
}

/// What fwrite puts in a file the C opened with "w": the bytes as they are, but on Windows through the text mode
pub fn file(bytes: &[u8]) -> Vec<u8> {
	if cfg!(windows) {
		write(bytes)
	} else {
		bytes.to_vec()
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A write puts a CR before each LF and keeps one already there; a read takes a CR LF back to LF and stops at a
	/// Ctrl-Z. So a config read from a cache file (CR LF in, LF kept) and written again keeps the file's CR LF, and
	/// an LF-only one gets them, as the C's KN<n>_config.json and LHR-<serial>_config.json have
	#[test]
	fn a_write_adds_a_cr_and_a_read_takes_it_back() {
		assert_eq!(write(b"{\n\t\"a\": 1\r\n}\n"), b"{\r\n\t\"a\": 1\r\r\n}\r\n".to_vec());
		assert_eq!(write(b"a\nb\r\n\nc"), b"a\r\nb\r\r\n\r\nc".to_vec());
		let crlf = b"{\r\n\t\"a\": 1\r\n}\r\n";
		assert_eq!(write(&read(crlf)), crlf.to_vec());
		let lf = b"{\n\t\"a\": 1\n}\n";
		assert_eq!(write(lf), crlf.to_vec());
		assert_eq!(read(&write(lf)), lf.to_vec());
		assert_eq!(read(b"a\rb\r\nc\x1ad\n"), b"a\rb\nc".to_vec());
		assert_eq!(file(lf), if cfg!(windows) { crlf.to_vec() } else { lf.to_vec() });
	}
}
