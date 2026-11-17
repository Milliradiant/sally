//! The HID tap's recordings, line by line: one text line per report.
//!
//! ```text
//! R <host_time_us> <serial> <pid_hex> <mi> <col> <kind> <hexbytes>
//! D <host_time_us> <serial> <pid_hex>
//! A <host_time_us> <serial> <device serial>
//! ```
//! `kind` is `I` (an input report), `F` (a feature report read: 0x10 / 0x11 are the config stream), `W` (a feature
//! report set) or `O` (an output report written); `hexbytes` is the whole report with its id byte. A `D` line announces
//! a device (so devices are numbered in the order they were seen); `#` lines and anything else are ignored. The live
//! tap writes `R ... I` lines, and an `A` line where SteamVR told it which device a dongle carries
//! (`tap::Tap::assign`): from there on the dongle `serial` is the device `device serial` (`-`: none), its config the
//! device's own. A replay takes the line where it stands, as the live run did; older readers skip it. And a `# cache
//! <serial> <device serial>` comment where a dongle opened with its cache, the config of that device: what a replay's
//! folder needs as `<serial>_config.json` to start as the live run did (the bench's tools read it).

/// One line of a tap recording, parsed as driver_hidtap.c's `hidtap_line` does (its sscanf formats)
#[derive(Clone, Debug, PartialEq)]
pub enum Line {
	/// `R`: a report
	Report { time_us: u64, serial: String, pid: u16, mi: i32, col: i32, kind: u8, data: Vec<u8> },
	/// `D`: a device seen
	Device { time_us: u64, serial: String, pid: u16 },
	/// `A`: the device a dongle carries from here on (`device` "-": none)
	Assign { time_us: u64, serial: String, device: String },
	/// an `R` line its parse rejected (the C warns "hidtap: bad line" for the first five)
	BadLine,
	/// an `R` line whose bytes are not hex (the C warns "hidtap: bad hex")
	BadHex,
	/// anything else (comments, blank lines)
	Ignored,
}

/// The longest report a line may carry (driver_hidtap.c's `uint8_t data[1024]`)
pub const MAX_REPORT: usize = 1024;

/// Parses one line (with or without its newline)
pub fn parse(line: &[u8]) -> Line {
	match line.first() {
		Some(b'D') => parse_device(line),
		Some(b'R') => parse_report(line),
		Some(b'A') => parse_assign(line),
		_ => Line::Ignored,
	}
}

// "A %llu %63s %63s": all three or the line is ignored
fn parse_assign(line: &[u8]) -> Line {
	let mut s = Scanner::new(&line[1..]);
	let (Some(time_us), Some(serial), Some(device)) = (s.ull(), s.word(63), s.word(63)) else { return Line::Ignored };
	Line::Assign { time_us, serial, device }
}

/// An `A` line: from `time_us` on, the dongle `serial` carries the device `device` ("-" for none)
pub fn assign(time_us: u64, serial: &str, device: &str) -> Vec<u8> {
	format!("A {time_us} {serial} {device}\n").into_bytes()
}

/// The comment a live recording gets where a dongle opened with its cache: `device`'s config
pub fn cache_comment(serial: &str, device: &str) -> Vec<u8> {
	format!("# cache {serial} {device}\n").into_bytes()
}

// "D %llu %63s %x": all three or the line is ignored
fn parse_device(line: &[u8]) -> Line {
	let mut s = Scanner::new(&line[1..]);
	let (Some(time_us), Some(serial), Some(pid)) = (s.ull(), s.word(63), s.hex()) else { return Line::Ignored };
	Line::Device { time_us, serial, pid: pid as u16 }
}

// "R %llu %63s %x %d %d %c %n": six conversions and %n reached, then the hex from there
fn parse_report(line: &[u8]) -> Line {
	let mut s = Scanner::new(&line[1..]);
	let (Some(time_us), Some(serial), Some(pid), Some(mi), Some(col), Some(kind)) =
		(s.ull(), s.word(63), s.hex(), s.int(), s.int(), s.char())
	else {
		return Line::BadLine;
	};
	// the format's trailing space skips white space before %n; %n then records where the hex starts
	s.skip_ws();
	let Some(data) = decode_hex(s.rest()) else { return Line::BadHex };
	Line::Report { time_us, serial, pid: pid as u16, mi, col, kind, data }
}

/// driver_hidtap.c's decode_hex: pairs of hex digits up to a newline, CR or space; an odd digit, a non-hex character or
/// more than MAX_REPORT bytes is an error
pub fn decode_hex(hex: &[u8]) -> Option<Vec<u8>> {
	let mut out = Vec::new();
	let mut i = 0;
	while i < hex.len() && hex[i] != b'\n' && hex[i] != b'\r' && hex[i] != b' ' {
		let hi = nibble(hex[i]);
		let lo = if i + 1 < hex.len() { nibble(hex[i + 1]) } else { None };
		let (Some(hi), Some(lo)) = (hi, lo) else { return None };
		if out.len() >= MAX_REPORT {
			return None;
		}
		out.push((hi << 4) | lo);
		i += 2;
	}
	Some(out)
}

fn nibble(c: u8) -> Option<u8> {
	match c {
		b'0'..=b'9' => Some(c - b'0'),
		b'a'..=b'f' => Some(c - b'a' + 10),
		b'A'..=b'F' => Some(c - b'A' + 10),
		_ => None,
	}
}

/// The line the live tap writes for an input report (hidtap_live_report): "R %llu %s %04x %d %d I ", the report in lower
/// case hex, a newline. The C formats it in a 4400-byte buffer and stops the hex where three more bytes would not fit
/// (no report the tap reads is that long: its reads are 2048 bytes at most)
pub fn input_report(time_us: u64, serial: &str, pid: u16, mi: i32, col: i32, data: &[u8]) -> Vec<u8> {
	const LINE_LEN: usize = 4400;
	const HEX: &[u8; 16] = b"0123456789abcdef";
	let mut line = Vec::with_capacity(64 + 2 * data.len());
	line.extend_from_slice(format!("R {time_us} {serial} {pid:04x} {mi} {col} I ").as_bytes());
	for &d in data {
		if line.len() + 3 >= LINE_LEN {
			break;
		}
		line.push(HEX[(d >> 4) as usize]);
		line.push(HEX[(d & 15) as usize]);
	}
	line.push(b'\n');
	line
}

/// The subset of sscanf the two formats use, over bytes: every conversion but %c skips leading white space
struct Scanner<'a> {
	b: &'a [u8],
	i: usize,
}

impl<'a> Scanner<'a> {
	fn new(b: &'a [u8]) -> Self {
		Scanner { b, i: 0 }
	}
	fn skip_ws(&mut self) {
		while self.i < self.b.len() && is_space(self.b[self.i]) {
			self.i += 1;
		}
	}
	fn rest(&self) -> &'a [u8] {
		&self.b[self.i..]
	}
	// %llu: optional sign, decimal digits (a '-' negates modulo 2^64, as strtoull does)
	fn ull(&mut self) -> Option<u64> {
		self.skip_ws();
		let neg = self.sign();
		let start = self.i;
		let mut v: u64 = 0;
		while self.i < self.b.len() && self.b[self.i].is_ascii_digit() {
			v = v.wrapping_mul(10).wrapping_add((self.b[self.i] - b'0') as u64);
			self.i += 1;
		}
		if self.i == start {
			return None;
		}
		Some(if neg { v.wrapping_neg() } else { v })
	}
	// %d: optional sign, decimal digits, into an int (values out of range are not in tap files)
	fn int(&mut self) -> Option<i32> {
		self.skip_ws();
		let neg = self.sign();
		let start = self.i;
		let mut v: i64 = 0;
		while self.i < self.b.len() && self.b[self.i].is_ascii_digit() {
			v = v.wrapping_mul(10).wrapping_add((self.b[self.i] - b'0') as i64);
			self.i += 1;
		}
		if self.i == start {
			return None;
		}
		let v = if neg { v.wrapping_neg() } else { v };
		Some(v as i32)
	}
	// %x: optional sign, optional 0x, hex digits, into an unsigned int
	fn hex(&mut self) -> Option<u32> {
		self.skip_ws();
		let neg = self.sign();
		if self.i + 1 < self.b.len()
			&& self.b[self.i] == b'0'
			&& (self.b[self.i + 1] == b'x' || self.b[self.i + 1] == b'X')
			&& self.i + 2 < self.b.len()
			&& nibble(self.b[self.i + 2]).is_some()
		{
			self.i += 2;
		}
		let start = self.i;
		let mut v: u32 = 0;
		while self.i < self.b.len() {
			let Some(d) = nibble(self.b[self.i]) else { break };
			v = v.wrapping_mul(16).wrapping_add(d as u32);
			self.i += 1;
		}
		if self.i == start {
			return None;
		}
		Some(if neg { v.wrapping_neg() } else { v })
	}
	// %Ns: up to N non-space bytes
	fn word(&mut self, max: usize) -> Option<String> {
		self.skip_ws();
		let start = self.i;
		while self.i < self.b.len() && self.i - start < max && !is_space(self.b[self.i]) {
			self.i += 1;
		}
		if self.i == start {
			return None;
		}
		Some(String::from_utf8_lossy(&self.b[start..self.i]).into_owned())
	}
	// " %c": the format's space skips white space, then one byte
	fn char(&mut self) -> Option<u8> {
		self.skip_ws();
		let c = *self.b.get(self.i)?;
		self.i += 1;
		Some(c)
	}
	fn sign(&mut self) -> bool {
		match self.b.get(self.i) {
			Some(b'-') => {
				self.i += 1;
				true
			}
			Some(b'+') => {
				self.i += 1;
				false
			}
			_ => false,
		}
	}
}

fn is_space(c: u8) -> bool {
	c == b' ' || (b'\t'..=b'\r').contains(&c)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn report_and_device_lines() {
		let l = parse(b"R 1234 LHR-8C52CF4E 2102 -1 -1 I 2301ff\n");
		assert_eq!(
			l,
			Line::Report {
				time_us: 1234,
				serial: "LHR-8C52CF4E".into(),
				pid: 0x2102,
				mi: -1,
				col: -1,
				kind: b'I',
				data: vec![0x23, 0x01, 0xff]
			}
		);
		assert_eq!(
			parse(b"D 5 EBF4140A47-4YX 2102"),
			Line::Device { time_us: 5, serial: "EBF4140A47-4YX".into(), pid: 0x2102 }
		);
		assert_eq!(parse(b"R 5 x 2102 0 0 I 2"), Line::BadHex);
		assert_eq!(parse(b"R 5 x 2102 0"), Line::BadLine);
		assert_eq!(parse(b"# comment"), Line::Ignored);
		assert_eq!(
			parse(b"A 77 1B7A62914E-3YX LHR-1B80952C\r\n"),
			Line::Assign { time_us: 77, serial: "1B7A62914E-3YX".into(), device: "LHR-1B80952C".into() }
		);
		assert_eq!(parse(b"A 77 1B7A62914E-3YX"), Line::Ignored);
		assert_eq!(
			parse(&assign(9, "A9DDD3B47A-LYM", "-")),
			Line::Assign { time_us: 9, serial: "A9DDD3B47A-LYM".into(), device: "-".into() }
		);
		assert_eq!(decode_hex(b"0AfF\r\n"), Some(vec![0x0a, 0xff]));
	}

	/// The first line of the live recording sallytap_20261101_182807 (the C's, on Windows, its CR LF the text mode's)
	/// formatted again from its fields
	#[test]
	fn input_reports_are_written_as_the_c_writes_them() {
		let c = "R 24869598568 EBF4140A47-4YX 2102 -1 -1 I 23430800f7000000000000b7a1ec47c46d43e1e6031e87e2514692595186a20fc\
		         880f3d407250d940709001d00d8ff0b37ce5cb29c260e7ef68ffb\n";
		let Line::Report { time_us, serial, pid, mi, col, kind: b'I', data } = parse(c.as_bytes()) else { panic!() };
		assert_eq!(data.len(), 59);
		assert_eq!(input_report(time_us, &serial, pid, mi, col, &data), c.as_bytes());
		assert_eq!(input_report(7, "nosn-3", 0x2300, 0, 1, &[0x0a, 0xff]), b"R 7 nosn-3 2300 0 1 I 0aff\n");
		assert_eq!(input_report(7, "x", 0x28, -1, -1, &[]), b"R 7 x 0028 -1 -1 I \n");
		// the C's buffer: a byte's hex is added while the line and three more bytes fit in 4400 (a 19-byte start here,
		// so 2189 bytes of hex)
		let long = input_report(1, "s", 0x2102, -1, -1, &[0xab; 3000]);
		assert_eq!(long.len(), 19 + 2 * 2189 + 1);
		assert!(long.ends_with(b"ab\n"));
	}
}
