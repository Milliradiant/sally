//! printf, in Rust, for the text sally writes as the C wrote it: the conversions d i u x X o c s f F e E g G and %%,
//! with the flags - + space 0 #, a width and a precision (numbers or `*`), and the length modifiers hh h l ll z j t L
//! (accepted; the argument's own type decides). The numbers come out as glibc's printf writes them: Rust rounds a
//! double's exact binary value to the precision asked, ties to even, as glibc does (checked against glibc on millions
//! of values in the tests); this module only spells the result the C way (exponents of two digits or more, NaN as
//! "nan" or "-nan", %g's choice of style and its trailing zeros). Not followed: glibc's %#g where rounding carries into
//! a new digit (999999.5 with "%#g" is "1.e+06" there, the zeros lost); nothing in sally prints with "%#g".

use std::fmt::Write;

/// One printf argument, as the C's variadic call would have passed it
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Arg<'a> {
	/// a signed integer (char, short, int, long, long long, after the C's promotions)
	I(i64),
	/// an unsigned integer
	U(u64),
	/// a double (a float is promoted to one)
	F(f64),
	/// a C string
	S(&'a str),
}

impl From<i32> for Arg<'_> {
	fn from(v: i32) -> Self {
		Arg::I(v as i64)
	}
}
impl From<i64> for Arg<'_> {
	fn from(v: i64) -> Self {
		Arg::I(v)
	}
}
impl From<u32> for Arg<'_> {
	fn from(v: u32) -> Self {
		Arg::U(v as u64)
	}
}
impl From<u64> for Arg<'_> {
	fn from(v: u64) -> Self {
		Arg::U(v)
	}
}
impl From<usize> for Arg<'_> {
	fn from(v: usize) -> Self {
		Arg::U(v as u64)
	}
}
impl From<f64> for Arg<'_> {
	fn from(v: f64) -> Self {
		Arg::F(v)
	}
}
impl<'a> From<&'a str> for Arg<'a> {
	fn from(v: &'a str) -> Self {
		Arg::S(v)
	}
}

/// A conversion specification's flags, width and precision
#[derive(Clone, Copy, Default)]
struct Spec {
	left: bool,
	plus: bool,
	space: bool,
	zero: bool,
	alt: bool,
	width: usize,
	precision: Option<usize>,
}

/// One printf argument whose string is bytes, as a C string is: a name or a path that came off the wire or out of a
/// file need not be UTF-8
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BArg<'a> {
	I(i64),
	U(u64),
	F(f64),
	S(&'a [u8]),
}

/// sprintf with byte strings, as the C's printf takes them: each byte goes through as one char (U+0000 to U+00FF), so
/// a width or a precision counts bytes, and comes back as that byte. The format is ASCII, as the C's literals are
pub fn bsprintf(fmt: &str, args: &[BArg]) -> Vec<u8> {
	debug_assert!(fmt.is_ascii());
	let texts: Vec<String> = args
		.iter()
		.map(|a| match a {
			BArg::S(b) => b.iter().map(|&c| c as char).collect(),
			_ => String::new(),
		})
		.collect();
	let args: Vec<Arg> = args
		.iter()
		.zip(&texts)
		.map(|(a, t)| match *a {
			BArg::S(_) => Arg::S(t),
			BArg::I(v) => Arg::I(v),
			BArg::U(v) => Arg::U(v),
			BArg::F(v) => Arg::F(v),
		})
		.collect();
	sprintf(fmt, &args).chars().map(|c| c as u32 as u8).collect()
}

/// sprintf: `fmt` with `args`. A conversion without its argument, or with one of another kind, writes nothing for it
/// (the C would read whatever the stack held: a caller's bug either way)
pub fn sprintf(fmt: &str, args: &[Arg]) -> String {
	let mut out = String::with_capacity(fmt.len() + 16 * args.len());
	sprintf_into(&mut out, fmt, args);
	out
}

/// sprintf's text appended to `out`, for a writer that keeps one buffer from line to line: each conversion is written
/// straight into it (an integer's digits through a buffer on the stack, a width's padding put in after), so a line
/// costs no allocation once the buffer has grown to it. The same text as `sprintf`, byte for byte
pub fn sprintf_into(out: &mut String, fmt: &str, args: &[Arg]) {
	let b = fmt.as_bytes();
	let mut args = args.iter().copied();
	let mut i = 0;
	while i < b.len() {
		if b[i] != b'%' {
			let start = i;
			while i < b.len() && b[i] != b'%' {
				i += 1;
			}
			out.push_str(&fmt[start..i]);
			continue;
		}
		i += 1;
		let mut s = Spec::default();
		while i < b.len() {
			match b[i] {
				b'-' => s.left = true,
				b'+' => s.plus = true,
				b' ' => s.space = true,
				b'0' => s.zero = true,
				b'#' => s.alt = true,
				_ => break,
			}
			i += 1;
		}
		if i < b.len() && b[i] == b'*' {
			i += 1;
			if let Some(Arg::I(w)) = args.next() {
				if w < 0 {
					s.left = true;
				}
				s.width = w.unsigned_abs() as usize;
			}
		} else {
			while i < b.len() && b[i].is_ascii_digit() {
				s.width = s.width * 10 + (b[i] - b'0') as usize;
				i += 1;
			}
		}
		if i < b.len() && b[i] == b'.' {
			i += 1;
			if i < b.len() && b[i] == b'*' {
				i += 1;
				// a negative precision is taken as if none were given
				s.precision = match args.next() {
					Some(Arg::I(p)) if p >= 0 => Some(p as usize),
					_ => None,
				};
			} else {
				let mut p = 0;
				while i < b.len() && b[i].is_ascii_digit() {
					p = p * 10 + (b[i] - b'0') as usize;
					i += 1;
				}
				s.precision = Some(p);
			}
		}
		while i < b.len() && matches!(b[i], b'h' | b'l' | b'z' | b'j' | b't' | b'L' | b'q') {
			i += 1;
		}
		let Some(&conv) = b.get(i) else { break };
		i += 1;
		if conv == b'%' {
			out.push('%');
			continue;
		}
		let arg = args.next();
		convert(out, conv, s, arg);
	}
}

/// The most digits an integer takes: u64::MAX in octal has 22, and %#o's leading 0 is one more
const DIGITS: usize = 24;

/// An integer's digits in base 8, 10 or 16, written into the end of `buf` (with a 0 in front when `zero_first` and they
/// do not start with one: %#o)
fn digits(buf: &mut [u8; DIGITS], mut v: u64, base: u64, upper: bool, zero_first: bool) -> &str {
	let set: &[u8; 16] = if upper { b"0123456789ABCDEF" } else { b"0123456789abcdef" };
	let mut i = DIGITS;
	loop {
		i -= 1;
		buf[i] = set[(v % base) as usize];
		v /= base;
		if v == 0 {
			break;
		}
	}
	if zero_first && buf[i] != b'0' {
		i -= 1;
		buf[i] = b'0';
	}
	std::str::from_utf8(&buf[i..]).unwrap_or_default()
}

/// One conversion, appended
fn convert(out: &mut String, conv: u8, s: Spec, arg: Option<Arg>) {
	let mut buf = [0u8; DIGITS];
	match (conv, arg) {
		(b'd' | b'i', Some(Arg::I(v))) => {
			integer(out, s, v < 0, digits(&mut buf, v.unsigned_abs(), 10, false, false), "")
		}
		(b'd' | b'i', Some(Arg::U(v))) => integer(out, s, false, digits(&mut buf, v, 10, false, false), ""),
		(b'u', Some(Arg::U(v))) => {
			integer(out, Spec { plus: false, space: false, ..s }, false, digits(&mut buf, v, 10, false, false), "")
		}
		(b'u', Some(Arg::I(v))) => {
			let d = digits(&mut buf, v as u64, 10, false, false);
			integer(out, Spec { plus: false, space: false, ..s }, false, d, "")
		}
		(b'x' | b'X' | b'o', Some(a @ (Arg::I(_) | Arg::U(_)))) => {
			let v = match a {
				Arg::I(v) => v as u64,
				Arg::U(v) => v,
				_ => 0,
			};
			let d = match conv {
				b'x' => digits(&mut buf, v, 16, false, false),
				b'X' => digits(&mut buf, v, 16, true, false),
				_ => digits(&mut buf, v, 8, false, s.alt),
			};
			let prefix = match conv {
				_ if !s.alt || v == 0 => "",
				b'x' => "0x",
				b'X' => "0X",
				_ => "",
			};
			integer(out, Spec { plus: false, space: false, ..s }, false, d, prefix);
		}
		(b'c', Some(Arg::I(v))) => {
			let start = out.len();
			out.push((v as u8) as char);
			pad(out, s, start, 0, false);
		}
		(b's', Some(Arg::S(v))) => {
			let v = match s.precision {
				Some(p) => {
					let mut end = p.min(v.len());
					while !v.is_char_boundary(end) {
						end -= 1;
					}
					&v[..end]
				}
				None => v,
			};
			let start = out.len();
			out.push_str(v);
			pad(out, Spec { zero: false, ..s }, start, 0, false);
		}
		(b'f' | b'F' | b'e' | b'E' | b'g' | b'G', Some(Arg::F(v))) => float(out, conv, s, v),
		_ => {}
	}
}

/// An integer: the sign and prefix, the digits with the precision's leading zeros, then the width
fn integer(out: &mut String, s: Spec, negative: bool, digits: &str, prefix: &str) {
	let digits = if s.precision == Some(0) && digits == "0" { "" } else { digits };
	let start = out.len();
	out.push_str(if negative {
		"-"
	} else if s.plus {
		"+"
	} else if s.space {
		" "
	} else {
		""
	});
	out.push_str(prefix);
	let lead = out.len() - start;
	if let Some(p) = s.precision {
		for _ in digits.len()..p {
			out.push('0');
		}
	}
	out.push_str(digits);
	// the 0 flag is ignored when a precision is given
	pad(out, Spec { zero: s.zero && s.precision.is_none(), ..s }, start, lead, true);
}

/// What was written from `start` on (its first `lead` bytes the sign or prefix, the body after them) padded to the
/// width, which counts characters: on the left with spaces, on the right with spaces (the - flag), or between the sign
/// and the body with zeros (the 0 flag, for numbers)
fn pad(out: &mut String, s: Spec, start: usize, lead: usize, numeric: bool) {
	if s.width == 0 {
		return;
	}
	let fill = s.width.saturating_sub(out[start..].chars().count());
	if s.left {
		out.extend(std::iter::repeat_n(' ', fill));
	} else if s.zero && numeric {
		insert_fill(out, start + lead, b'0', fill);
	} else {
		insert_fill(out, start, b' ', fill);
	}
}

/// `n` spaces or zeros put in at byte `at`, in front of what was written after it
fn insert_fill(out: &mut String, at: usize, c: u8, mut n: usize) {
	const SPACES: &str = "                                ";
	const ZEROS: &str = "00000000000000000000000000000000";
	let run = if c == b'0' { ZEROS } else { SPACES };
	out.reserve(n);
	while n > 0 {
		let k = n.min(run.len());
		out.insert_str(at, &run[..k]);
		n -= k;
	}
}

/// %f %F %e %E %g %G
fn float(out: &mut String, conv: u8, s: Spec, v: f64) {
	// a NaN with its sign bit set is "-nan" too
	let sign = if v.is_sign_negative() {
		"-"
	} else if s.plus {
		"+"
	} else if s.space {
		" "
	} else {
		""
	};
	let upper = conv.is_ascii_uppercase();
	let start = out.len();
	out.push_str(sign);
	if !v.is_finite() {
		out.push_str(match (v.is_nan(), upper) {
			(true, false) => "nan",
			(true, true) => "NAN",
			(false, false) => "inf",
			(false, true) => "INF",
		});
		// NaN and infinity take no zero padding
		pad(out, Spec { zero: false, ..s }, start, sign.len(), true);
		return;
	}
	let a = v.abs();
	let p = s.precision.unwrap_or(6);
	let body = out.len();
	match conv {
		// (%F's body is digits and a point: its upper case is itself)
		b'f' | b'F' => fixed(out, a, p, s.alt),
		b'e' | b'E' => exponential(out, a, p, s.alt, upper),
		_ => {
			// %g: the precision P (0 is 1); the style e would have exponent X after rounding to P digits: f with
			// precision P - 1 - X when P > X >= -4, else e with P - 1; then the trailing zeros go (unless #)
			let p = if p == 0 { 1 } else { p };
			let x = exponent_after_rounding(out, a, p - 1);
			if (x as i64) < p as i64 && x >= -4 {
				fixed(out, a, (p as i64 - 1 - x as i64) as usize, s.alt);
			} else {
				exponential(out, a, p - 1, s.alt, upper);
			}
			if !s.alt {
				strip_zeros(out, body);
			}
		}
	}
	pad(out, s, start, sign.len(), true);
}

/// "%.{p}f" of a non-negative finite value, appended (with # a point even at precision 0). Writing into a String does
/// not fail
fn fixed(out: &mut String, a: f64, p: usize, alt: bool) {
	let _ = write!(out, "{a:.p$}");
	if alt && p == 0 {
		out.push('.');
	}
}

/// "%.{p}e" of a non-negative finite value, appended: the exponent Rust writes ("e-5", "e12") written again signed, two
/// digits at least
fn exponential(out: &mut String, a: f64, p: usize, alt: bool, upper: bool) {
	let at = out.len();
	let _ = write!(out, "{a:.p$e}");
	let e_at = out[at..].find('e').map_or(out.len(), |i| at + i);
	let e: i32 = out[e_at..].get(1..).and_then(|e| e.parse().ok()).unwrap_or(0);
	out.truncate(e_at);
	if alt && p == 0 {
		out.push('.');
	}
	out.push(if upper { 'E' } else { 'e' });
	out.push(if e < 0 { '-' } else { '+' });
	let _ = write!(out, "{:02}", e.unsigned_abs());
}

/// The decimal exponent of a non-negative finite value written with `p` digits after the point in style e (written at
/// the end of `out` to be read, then taken off again)
fn exponent_after_rounding(out: &mut String, a: f64, p: usize) -> i32 {
	let at = out.len();
	let _ = write!(out, "{a:.p$e}");
	let x = out[at..].split_once('e').and_then(|(_, e)| e.parse().ok()).unwrap_or(0);
	out.truncate(at);
	x
}

/// %g's trailing zeros of the fraction removed from the body written from `body` on, and the point if nothing follows
/// it
fn strip_zeros(out: &mut String, body: usize) {
	let (mantissa_end, exp) = match out[body..].find(['e', 'E']) {
		Some(i) => (body + i, true),
		None => (out.len(), false),
	};
	if !out[body..mantissa_end].contains('.') {
		return;
	}
	let kept = body + out[body..mantissa_end].trim_end_matches('0').trim_end_matches('.').len();
	if exp {
		out.replace_range(kept..mantissa_end, "");
	} else {
		out.truncate(kept);
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use std::ffi::{c_char, c_int, CString};

	extern "C" {
		fn snprintf(buf: *mut c_char, n: usize, fmt: *const c_char, ...) -> c_int;
	}

	fn c_f(fmt: &str, v: f64) -> String {
		let f = CString::new(fmt).unwrap();
		let mut b = [0u8; 600];
		// SAFETY: a bounded buffer, a format with one double conversion and one double
		let n = unsafe { snprintf(b.as_mut_ptr() as *mut c_char, b.len(), f.as_ptr(), v) };
		String::from_utf8(b[..n as usize].to_vec()).unwrap()
	}

	fn c_i(fmt: &str, v: i64) -> String {
		let f = CString::new(fmt).unwrap();
		let mut b = [0u8; 600];
		// SAFETY: a bounded buffer, a format with one long long conversion and one long long
		let n = unsafe { snprintf(b.as_mut_ptr() as *mut c_char, b.len(), f.as_ptr(), v) };
		String::from_utf8(b[..n as usize].to_vec()).unwrap()
	}

	/// The C library's printf and this agree on the conversions sally's text uses, flags, widths and precisions
	#[test]
	fn as_the_c_library_prints() {
		let mut x: u64 = 0x2545_f491_4f6c_dd1d;
		let mut next = move || {
			x ^= x << 13;
			x ^= x >> 7;
			x ^= x << 17;
			x
		};
		let mut values = vec![
			0.0,
			-0.0,
			1.0,
			-1.0,
			0.5,
			1.5,
			2.5,
			9.5,
			0.05,
			0.0001,
			0.00001,
			123456.0,
			1234567.0,
			1e-300,
			5e-324,
			1e300,
			99999.95,
			999999.5,
			0.000099995,
			f64::INFINITY,
			-f64::INFINITY,
			f64::NAN,
			-f64::NAN,
		];
		for _ in 0..30000 {
			let r = next();
			values.push(f64::from_bits(r));
			values.push((r % 2_000_000_000) as f64 / 1e6 - 1000.0);
			values.push(((r % 1_000_000) as f64) / 64.0);
			values.push((r % 100000) as f64 * 10f64.powi((r % 13) as i32 - 9));
		}
		let fmts = [
			"%f", "%.0f", "%.1f", "%.3f", "%+.3f", "%.6f", "%7.3f", "%-9.2f|", "%08.3f", "% .2f", "%#.0f", "%e", "%+e",
			"%.3e", "%E", "%12.4e", "%g", "%.3g", "%.10g", "%G", "%+g", "%10g", "%-10g|", "%.0g", "%.17g", "%F",
		];
		for &v in &values {
			for f in fmts {
				assert_eq!(sprintf(f, &[Arg::F(v)]), c_f(f, v), "{f} of {v:e} ({:#x})", v.to_bits());
			}
		}
		let ints = [0i64, 1, -1, 7, -42, 255, 65535, 1 << 31, -(1 << 31), i64::MAX, i64::MIN + 1, 123456789];
		let ifmts = [
			"%lld", "%5lld", "%-5lld|", "%05lld", "%+lld", "% lld", "%.3lld", "%.0lld", "%llx", "%#llx", "%llX",
			"%llo", "%#llo", "%llu",
		];
		for &v in &ints {
			for f in ifmts {
				let arg = if f.ends_with('u') { Arg::U(v as u64) } else { Arg::I(v) };
				assert_eq!(sprintf(f, &[arg]), c_i(f, v), "{f} of {v}");
			}
		}
		assert_eq!(sprintf("%s=%d%%", &[Arg::S("KN0"), Arg::I(3)]), "KN0=3%");
		assert_eq!(sprintf("[%5s|%-5s|%.2s]", &[Arg::S("ab"), Arg::S("ab"), Arg::S("abc")]), "[   ab|ab   |ab]");
		assert_eq!(
			sprintf("%*d|%-*d|%.*f", &[Arg::I(4), Arg::I(7), Arg::I(4), Arg::I(7), Arg::I(2), Arg::F(1.005)]),
			"   7|7   |1.00"
		);
		// into a buffer that holds text already: after it
		let mut buf = String::from("\u{e9}|");
		sprintf_into(&mut buf, "%5.2f", &[Arg::F(1.005)]);
		assert_eq!(buf, "\u{e9}| 1.00");
	}

	#[test]
	fn byte_strings_go_through_as_bytes() {
		let name = b"LHR-\xff\x80\xc3(";
		assert_eq!(bsprintf("[%s] %u %.1f", &[BArg::S(name), BArg::U(7), BArg::F(2.25)]), b"[LHR-\xff\x80\xc3(] 7 2.2");
		// a width counts bytes, as the C's
		assert_eq!(bsprintf("%6s|%-4s|", &[BArg::S(b"\xe9a"), BArg::S(b"\xe9")]), b"    \xe9a|\xe9   |");
	}
}
