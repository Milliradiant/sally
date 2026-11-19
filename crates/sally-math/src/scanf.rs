//! The C library's text input as sally's files and lines go through it: fopen(.., "r") and fgets into a fixed buffer
//! (a long line comes in pieces), and sscanf for the directives its formats use (literal text, white space, %u %d %lu
//! %lf %Ns %n, each with `*` to read without assigning). sscanf follows glibc's (stdio-common/vfscanf-internal.c,
//! 2.39) in the "C" locale, its numbers read by the same strtod / strtol rules; the ucrt's differs only on malformed
//! input, which the files and lines sally writes never have.

use crate::libc::c_strtod;

/// The lines of a text file as fgets(line, size, f) gives them: each up to and with its '\n', a longer one in pieces
/// of size - 1 bytes. A text-mode read on Windows: CR LF becomes LF and a Ctrl-Z ends the text
pub fn fgets_lines(bytes: &[u8], size: usize) -> Vec<Vec<u8>> {
	assert!(size >= 2);
	let text: Vec<u8> = if cfg!(windows) { crate::textmode::read(bytes) } else { bytes.to_vec() };
	let mut out = Vec::new();
	let mut i = 0;
	while i < text.len() {
		let mut end = i;
		while end < text.len() && end - i < size - 1 {
			end += 1;
			if text[end - 1] == b'\n' {
				break;
			}
		}
		out.push(text[i..end].to_vec());
		i = end;
	}
	out
}

/// A C string in a buffer: the bytes up to the first NUL
pub fn c_str(b: &[u8]) -> &[u8] {
	match b.iter().position(|&c| c == 0) {
		Some(n) => &b[..n],
		None => b,
	}
}

/// One conversion's value, in the order of the format's directives
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
	/// %u (unsigned int)
	U(u32),
	/// %d (int)
	D(i32),
	/// %lu (unsigned long, 64 bits as on Linux)
	Lu(u64),
	/// %lf (double)
	Lf(f64),
	/// %Ns
	S(Vec<u8>),
	/// %n: the bytes read so far (not counted in the result)
	N(i32),
}

/// isspace in the "C" locale
fn is_space(c: u8) -> bool {
	matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// The input as sscanf reads it: a string up to its NUL, the end being EOF
struct Input<'a> {
	s: &'a [u8],
	i: usize,
}

impl Input<'_> {
	/// inchar(): the next byte, None at the end
	fn inchar(&mut self) -> Option<u8> {
		let c = self.s.get(self.i).copied();
		if c.is_some() {
			self.i += 1;
		}
		c
	}
	/// ungetc(c): the last byte read back (nothing for EOF)
	fn ungetc(&mut self, c: Option<u8>) {
		if c.is_some() {
			self.i -= 1;
		}
	}
}

/// How a conversion ended
enum Fail {
	/// input_error(): the input ended before the conversion (sscanf returns EOF when nothing was converted)
	Input,
	/// conv_error(): the input did not match
	Conv,
}

/// sscanf(input, fmt, ...): glibc's result (the conversions assigned, or -1 for EOF when the input ended before the
/// first) and the values assigned before it stopped (a `%*` conversion reads its field, and stops the scan where it
/// does not match, but assigns nothing and is not counted). The format may use literal bytes, white space, %u %d %lu
/// %lf %<width>s and %n, `*` after the % on any but %n; anything else in it is a caller's bug and stops.
pub fn sscanf(input: &[u8], fmt: &str) -> (i32, Vec<Value>) {
	let mut inp = Input { s: c_str(input), i: 0 };
	let f = fmt.as_bytes();
	let mut values = Vec::new();
	let mut done = 0i32;
	let mut skip_space = false;
	let mut fi = 0;
	let fail = |e: Fail, done: i32| -> i32 {
		match e {
			Fail::Input if done == 0 => -1,
			_ => done,
		}
	};
	while fi < f.len() {
		let fc = f[fi];
		fi += 1;
		if fc != b'%' {
			if is_space(fc) {
				skip_space = true;
				continue;
			}
			let mut c = inp.inchar();
			if c.is_none() {
				return (fail(Fail::Input, done), values);
			}
			if skip_space {
				while c.is_some_and(is_space) {
					c = inp.inchar();
					if c.is_none() {
						return (fail(Fail::Input, done), values);
					}
				}
				skip_space = false;
			}
			if c != Some(fc) {
				inp.ungetc(c);
				return (fail(Fail::Conv, done), values);
			}
			continue;
		}
		// the assignment suppressed, the width and the length modifier
		let suppress = fi < f.len() && f[fi] == b'*';
		if suppress {
			fi += 1;
		}
		let mut width: i32 = -1;
		if fi < f.len() && f[fi].is_ascii_digit() {
			width = 0;
			while fi < f.len() && f[fi].is_ascii_digit() {
				width = width * 10 + (f[fi] - b'0') as i32;
				fi += 1;
			}
		}
		let mut long = false;
		if fi < f.len() && f[fi] == b'l' {
			long = true;
			fi += 1;
		}
		let conv = f[fi];
		fi += 1;
		if skip_space || conv != b'n' {
			// white space is eaten before every conversion but %n (and %c, %[)
			loop {
				let c = inp.inchar();
				if !c.is_some_and(is_space) {
					inp.ungetc(c);
					break;
				}
			}
			skip_space = false;
		}
		// the value and whether it counts in the result (%n does not)
		let r = match conv {
			b'n' => Ok((Value::N(inp.i as i32), false)),
			b'u' | b'd' => integer(&mut inp, conv == b'd', long).map(|v| (v, true)),
			b'f' if long => double(&mut inp).map(|v| (Value::Lf(v), true)),
			b's' => string(&mut inp, width).map(|v| (Value::S(v), true)),
			_ => panic!("sscanf: the directive %{} is not ported", conv as char),
		};
		match r {
			Ok((v, counted)) => {
				if !suppress {
					values.push(v);
					done += counted as i32;
				}
			}
			Err(e) => return (fail(e, done), values),
		}
	}
	(done, values)
}

/// %d %u %lu: an optional sign and decimal digits, converted by strtol / strtoul (saturating), then to the type's width
fn integer(inp: &mut Input, signed: bool, long: bool) -> Result<Value, Fail> {
	let mut c = inp.inchar();
	if c.is_none() {
		return Err(Fail::Input);
	}
	let mut buf = Vec::new();
	if c == Some(b'-') || c == Some(b'+') {
		buf.push(c.unwrap_or(0));
		c = inp.inchar();
	}
	while let Some(d) = c {
		if !d.is_ascii_digit() {
			break;
		}
		buf.push(d);
		c = inp.inchar();
	}
	inp.ungetc(c);
	if buf.is_empty() || (buf.len() == 1 && (buf[0] == b'+' || buf[0] == b'-')) {
		return Err(Fail::Conv);
	}
	let negative = buf[0] == b'-';
	let digits = if buf[0] == b'-' || buf[0] == b'+' { &buf[1..] } else { &buf[..] };
	// the magnitude, saturated past u64
	let mut mag: Option<u64> = Some(0);
	for &d in digits {
		mag = mag.and_then(|m| m.checked_mul(10)).and_then(|m| m.checked_add((d - b'0') as u64));
	}
	Ok(if signed {
		// strtol: LONG_MIN..LONG_MAX, saturated; then (int)
		let l: i64 = match mag {
			Some(m) if !negative && m <= i64::MAX as u64 => m as i64,
			Some(m) if negative && m <= i64::MAX as u64 + 1 => (m as i64).wrapping_neg(),
			_ if negative => i64::MIN,
			_ => i64::MAX,
		};
		Value::D(l as i32)
	} else {
		// strtoul: ULONG_MAX past the range, a negative value negated in unsigned arithmetic
		let ul: u64 = match mag {
			Some(m) if negative => m.wrapping_neg(),
			Some(m) => m,
			None => u64::MAX,
		};
		if long {
			Value::Lu(ul)
		} else {
			Value::U(ul as u32)
		}
	})
}

/// %lf: the bytes that look like a number collected as glibc collects them, then strtod on them
fn double(inp: &mut Input) -> Result<f64, Fail> {
	let mut c = inp.inchar();
	if c.is_none() {
		return Err(Fail::Input);
	}
	let mut buf: Vec<u8> = Vec::new();
	let mut got_sign = false;
	if c == Some(b'-') || c == Some(b'+') {
		got_sign = true;
		buf.push(c.unwrap_or(0));
		c = inp.inchar();
		if c.is_none() {
			return Err(Fail::Conv);
		}
	}
	let lower = |c: Option<u8>| c.map(|c| c.to_ascii_lowercase());
	// "nan", "inf", "infinity"
	let expect = |inp: &mut Input, buf: &mut Vec<u8>, want: u8| -> Result<(), Fail> {
		let c = inp.inchar();
		if lower(c) != Some(want) {
			return Err(Fail::Conv);
		}
		buf.push(c.unwrap_or(0));
		Ok(())
	};
	if lower(c) == Some(b'n') {
		buf.push(c.unwrap_or(0));
		expect(inp, &mut buf, b'a')?;
		expect(inp, &mut buf, b'n')?;
		return convert(&buf);
	}
	if lower(c) == Some(b'i') {
		buf.push(c.unwrap_or(0));
		expect(inp, &mut buf, b'n')?;
		expect(inp, &mut buf, b'f')?;
		let c2 = inp.inchar();
		if c2.is_some() {
			if lower(c2) == Some(b'i') {
				buf.push(c2.unwrap_or(0));
				for want in *b"nity" {
					expect(inp, &mut buf, want)?;
				}
			} else {
				inp.ungetc(c2);
			}
		}
		return convert(&buf);
	}
	let mut exp_char = b'e';
	let mut hexa = false;
	let (mut got_digit, mut got_dot, mut got_e) = (false, false, false);
	if c == Some(b'0') {
		buf.push(b'0');
		c = inp.inchar();
		if lower(c) == Some(b'x') {
			buf.push(c.unwrap_or(0));
			hexa = true;
			exp_char = b'p';
			c = inp.inchar();
		} else {
			got_digit = true;
		}
	}
	while let Some(ch) = c {
		if ch.is_ascii_digit() || (!got_e && hexa && ch.is_ascii_hexdigit()) {
			buf.push(ch);
			got_digit = true;
		} else if got_e && buf.last() == Some(&exp_char) && (ch == b'-' || ch == b'+') {
			buf.push(ch);
		} else if got_digit && !got_e && ch.to_ascii_lowercase() == exp_char {
			buf.push(exp_char);
			got_e = true;
			got_dot = true;
		} else if !got_dot && ch == b'.' {
			buf.push(ch);
			got_dot = true;
		} else {
			// the last byte read is not part of the number any more
			inp.ungetc(c);
			break;
		}
		c = inp.inchar();
	}
	if buf.len() == got_sign as usize || (hexa && buf.len() == 2 + got_sign as usize) {
		return Err(Fail::Conv);
	}
	convert(&buf)
}

/// scan_float's conversion: strtod on the collected bytes; nothing read is a conversion error
fn convert(buf: &[u8]) -> Result<f64, Fail> {
	let (v, n) = c_strtod(buf);
	if n == 0 {
		return Err(Fail::Conv);
	}
	Ok(v)
}

/// %Ns: up to N bytes that are not white space (any number without a width)
fn string(inp: &mut Input, width: i32) -> Result<Vec<u8>, Fail> {
	let mut c = inp.inchar();
	if c.is_none() {
		return Err(Fail::Input);
	}
	let mut out = Vec::new();
	let mut width = width;
	while let Some(ch) = c {
		if is_space(ch) {
			inp.ungetc(c);
			break;
		}
		out.push(ch);
		let more = if width <= 0 {
			true
		} else {
			width -= 1;
			width > 0
		};
		if !more {
			break;
		}
		c = inp.inchar();
	}
	Ok(out)
}

#[cfg(test)]
mod tests {
	use super::*;
	use std::ffi::{c_char, c_int, c_uint, c_ulong, CString};

	extern "C" {
		#[link_name = "sscanf"]
		fn c_sscanf(s: *const c_char, fmt: *const c_char, ...) -> c_int;
	}

	/// The C library's sscanf on the map solve's formats, its values as `Value`s
	fn c_scan(input: &[u8], fmt: &str) -> (i32, Vec<Value>) {
		let s = CString::new(c_str(input)).unwrap();
		let f = CString::new(fmt).unwrap();
		let mut u = [0 as c_uint; 2];
		let mut d = [0 as c_int; 2];
		let mut lu: c_ulong = 0;
		let mut lf = [0f64; 20];
		let mut st = [0u8; 40];
		let mut n: c_int = -7;
		let (pu, pd, pf) = (u.as_mut_ptr(), d.as_mut_ptr(), lf.as_mut_ptr());
		// SAFETY: each call passes exactly the pointers its format converts into, to buffers large enough (%31s: 32)
		let r = unsafe {
			match fmt {
				"LAM %u %u%n" => c_sscanf(s.as_ptr(), f.as_ptr(), pu, pu.add(1), &mut n as *mut c_int),
				"LH %u %lf %lf %lf %lf %lf %lf %lf" => c_sscanf(
					s.as_ptr(),
					f.as_ptr(),
					pu,
					pf,
					pf.add(1),
					pf.add(2),
					pf.add(3),
					pf.add(4),
					pf.add(5),
					pf.add(6),
				),
				"SENSOR %31s %d %lf %lf %lf" => {
					c_sscanf(s.as_ptr(), f.as_ptr(), st.as_mut_ptr(), pd, pf, pf.add(1), pf.add(2))
				}
				"FCAL %lu %lf %lf" => c_sscanf(s.as_ptr(), f.as_ptr(), &mut lu as *mut c_ulong, pf, pf.add(1)),
				"%lf %d %lf %lf" => c_sscanf(s.as_ptr(), f.as_ptr(), pf, pd, pf.add(1), pf.add(2)),
				_ => panic!("{fmt}"),
			}
		};
		let strlen = st.iter().position(|&c| c == 0).unwrap_or(st.len());
		let all: Vec<Value> = match fmt {
			"LAM %u %u%n" => vec![Value::U(u[0]), Value::U(u[1]), Value::N(n)],
			"LH %u %lf %lf %lf %lf %lf %lf %lf" => {
				let mut v = vec![Value::U(u[0])];
				v.extend(lf[..7].iter().map(|&x| Value::Lf(x)));
				v
			}
			"SENSOR %31s %d %lf %lf %lf" => vec![
				Value::S(st[..strlen].to_vec()),
				Value::D(d[0]),
				Value::Lf(lf[0]),
				Value::Lf(lf[1]),
				Value::Lf(lf[2]),
			],
			"FCAL %lu %lf %lf" => vec![Value::Lu(lu as u64), Value::Lf(lf[0]), Value::Lf(lf[1])],
			"%lf %d %lf %lf" => vec![Value::Lf(lf[0]), Value::D(d[0]), Value::Lf(lf[1]), Value::Lf(lf[2])],
			_ => unreachable!(),
		};
		// the values assigned: the first r conversions, and %n (last in its format) when the scan got to it
		let mut assigned: Vec<Value> =
			all.iter().filter(|v| !matches!(v, Value::N(_))).take(r.max(0) as usize).cloned().collect();
		if n != -7 {
			assigned.push(Value::N(n));
		}
		(r, assigned)
	}

	fn same(a: &(i32, Vec<Value>), b: &(i32, Vec<Value>)) -> bool {
		a.0 == b.0
			&& a.1.len() == b.1.len()
			&& a.1.iter().zip(&b.1).all(|(x, y)| match (x, y) {
				(Value::Lf(p), Value::Lf(q)) => p.to_bits() == q.to_bits(),
				_ => x == y,
			})
	}

	#[test]
	fn as_glibc_reads_the_map_files() {
		let fmts = [
			"LAM %u %u%n",
			"LH %u %lf %lf %lf %lf %lf %lf %lf",
			"SENSOR %31s %d %lf %lf %lf",
			"FCAL %lu %lf %lf",
			"%lf %d %lf %lf",
		];
		let mut inputs: Vec<Vec<u8>> = [
			"LAM 287454020 2578103244 -78447255.7 -4877596.85",
			"LAM 1 2",
			"LAM 1",
			"LAM",
			"",
			"LAM  4294967296 -1x",
			"LAM 99999999999999999999 +7",
			"LAM -0 -",
			"LH 287454020 -0.012624243 2.343556150 1.562553479 0.062028179 0.071935990 -0.489113241 -0.867032846\n",
			"LH 1 1e 2E+ 3e-2 .5 5. -.e1 0x1p3",
			"LH 1 nan -nan inf -infinity INFINITE 1 2",
			"LH 1 0x 2 3 4 5 6 7",
			"LH 1 0x1.8p1 1e400 -1e-400 4 5 6 7",
			"LH 1 1..5 2 3 4 5 6 7",
			"LH\t2\t1\t2\t3\t4\t5\t6\t7",
			"LH 3 + 2 3 4 5 6 7",
			"LH 3 na 2 3 4 5 6 7",
			"LH 3 infin 2 3 4 5 6 7",
			"SENSOR LHR-12345678 7 0.001 -0.002 0.0005",
			"SENSOR 0123456789012345678901234567890123456789 7 1 2 3",
			"SENSOR  ab   -12  1  2  3",
			"SENSOR ab 2147483648 1 2 3",
			"SENSOR ab -9223372036854775809 1 2 3",
			"SENSOR",
			"SENSOR ",
			"FCAL 439041101 0 -0.006664276123",
			"FCAL 18446744073709551616 1 2",
			"FCAL -5 1 2",
			"1.5,3,0.01,0.02",
			"1.5 3 0.01 0.02",
			"  12.5   -1   0   0  ",
		]
		.iter()
		.map(|s| s.as_bytes().to_vec())
		.collect();
		// random bytes over the characters a number is made of
		let mut x: u64 = 0x9e37_79b9_7f4a_7c15;
		let alphabet = b"0123456789+-.eExXpPnNaAiIfFtTyY \t\n,";
		for _ in 0..40000 {
			let mut s = b"LH ".to_vec();
			x ^= x << 13;
			x ^= x >> 7;
			x ^= x << 17;
			let len = (x % 40) as usize;
			for _ in 0..len {
				x ^= x << 13;
				x ^= x >> 7;
				x ^= x << 17;
				s.push(alphabet[(x % alphabet.len() as u64) as usize]);
			}
			inputs.push(s.clone());
			let mut t = b"LAM ".to_vec();
			t.extend_from_slice(&s[3..]);
			inputs.push(t);
			let mut t = b"SENSOR ".to_vec();
			t.extend_from_slice(&s[3..]);
			inputs.push(t);
			inputs.push(s[3..].to_vec());
		}
		for input in &inputs {
			for fmt in fmts {
				let c = c_scan(input, fmt);
				let r = sscanf(input, fmt);
				assert!(same(&c, &r), "{fmt:?} on {:?}: C {:?}, ours {:?}", String::from_utf8_lossy(input), c, r);
			}
		}
	}

	#[test]
	fn lines_as_fgets_gives_them() {
		let l = fgets_lines(b"ab\ncdefg\n\nh", 4);
		assert_eq!(l, vec![b"ab\n".to_vec(), b"cde".to_vec(), b"fg\n".to_vec(), b"\n".to_vec(), b"h".to_vec()]);
		assert!(fgets_lines(b"", 10).is_empty());
	}

	/// The SteamVR driver's Q and P lines, %*s among them
	const Q_FMT: &str = "Q %31s %63s %*s %*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \
	                     %lf %lf %lf %lf %lf %lf %lf %lf";
	const P_FMT: &str = "P %31s %63s %*s %*s %*s %lf %lf %lf %lf %lf %lf %lf";

	/// The C library's sscanf on the driver's formats, its values as `Value`s
	fn c_scan_driver(input: &[u8], fmt: &str) -> (i32, Vec<Value>) {
		let s = CString::new(c_str(input)).unwrap();
		let f = CString::new(fmt).unwrap();
		let mut code = [0u8; 32];
		let mut serial = [0u8; 64];
		let mut v = [0f64; 26];
		let (pc, ps, p) = (code.as_mut_ptr(), serial.as_mut_ptr(), v.as_mut_ptr());
		// SAFETY: each call passes exactly the pointers its format converts into (%31s: 32 bytes, %63s: 64, 26 and 7
		// doubles), to buffers that live through the call
		let r = unsafe {
			if fmt == P_FMT {
				c_sscanf(s.as_ptr(), f.as_ptr(), pc, ps, p, p.add(1), p.add(2), p.add(3), p.add(4), p.add(5), p.add(6))
			} else {
				#[rustfmt::skip]
				let r = c_sscanf(s.as_ptr(), f.as_ptr(), pc, ps, p, p.add(1), p.add(2), p.add(3), p.add(4), p.add(5),
					p.add(6), p.add(7), p.add(8), p.add(9), p.add(10), p.add(11), p.add(12), p.add(13), p.add(14),
					p.add(15), p.add(16), p.add(17), p.add(18), p.add(19), p.add(20), p.add(21), p.add(22), p.add(23),
					p.add(24), p.add(25));
				r
			}
		};
		let mut all = vec![Value::S(c_str(&code).to_vec()), Value::S(c_str(&serial).to_vec())];
		all.extend(v.iter().map(|&x| Value::Lf(x)));
		(r, all.into_iter().take(r.max(0) as usize).collect())
	}

	#[test]
	fn as_glibc_reads_the_driver_lines() {
		let mut inputs: Vec<Vec<u8>> = [
			"Q KN4 LHR-PARSE 123456 12.5 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10 0 0 0 0.0025 0 0 0 0 0 0 0 0 -500",
			"Q KN4 LHR-PARSE 123457 12.6 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10 0 0 0 0.0025 0 0 0 0 0 0",
			"Q KN4 LHR-PARSE 123458 12.7 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10",
			"Q KN4 LHR-PARSE 123459 12.8 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10 0 0 0 0.0025 0 0 0 0 0 0 nan 0 0",
			"Q KN4 - 1 2 -nan inf -infinity 1e 2E+ .5 5. 0x1p3 0x 7 8 9 10 11",
			"Q 0123456789012345678901234567890123456789 LHR-1 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15",
			"Q a b",
			"Q a b c",
			"Q a b c d",
			"Q a b c d 1",
			"Q",
			"Q ",
			"P KN0 LHR-ABCDEF01 123 4.5 1730000000.123 0.1 0.2 0.3 1 0 0 0",
			"P KN0 LHR-ABCDEF01 123 4.5 1730000000.123 0.1 0.2 0.3 1 0 0",
			"P KN0 - 123 4.5 1730000000.123 nan(1) 0.2 0.3 1 0 0 0",
			"P\tKN0\tLHR-X\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10",
		]
		.iter()
		.map(|s| s.as_bytes().to_vec())
		.collect();
		// random lines over the characters numbers and names are made of
		let mut x: u64 = 0x2545_f491_4f6c_dd1d;
		let mut next = || {
			x ^= x << 13;
			x ^= x >> 7;
			x ^= x << 17;
			x
		};
		let alphabet = b"0123456789+-.eExXpPnNaAiIfFtTyY()_LHR \t";
		for _ in 0..20000 {
			let q = next() % 2 == 0;
			let mut s = if q { b"Q ".to_vec() } else { b"P ".to_vec() };
			let len = (next() % 200) as usize;
			for _ in 0..len {
				let c = alphabet[(next() % alphabet.len() as u64) as usize];
				s.push(c);
			}
			inputs.push(s);
		}
		for input in &inputs {
			let fmt = if input[0] == b'Q' { Q_FMT } else { P_FMT };
			let c = c_scan_driver(input, fmt);
			let r = sscanf(input, fmt);
			assert!(same(&c, &r), "{fmt:?} on {:?}: C {:?}, ours {:?}", String::from_utf8_lossy(input), c, r);
		}
	}
}
