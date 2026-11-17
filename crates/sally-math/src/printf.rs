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
	}

	/// A xorshift of the tests' own: the same cases on every run
	struct Rng(u64);

	impl Rng {
		fn next(&mut self) -> u64 {
			self.0 ^= self.0 << 13;
			self.0 ^= self.0 >> 7;
			self.0 ^= self.0 << 17;
			self.0
		}

		fn below(&mut self, n: u64) -> u64 {
			self.next() % n
		}

		/// A double of every kind printf meets
		fn float(&mut self) -> f64 {
			const TIES: [f64; 10] = [0.5, 1.5, 2.5, 9.5, 0.05, 99999.95, 999999.5, 0.000099995, 9.9999995, 1.005];
			let r = self.next();
			match self.below(20) {
				0 => f64::NAN,
				1 => -f64::NAN,
				// a NaN with a payload, of either sign
				2 => f64::from_bits(0x7ff0_0000_0000_0001 | (r & 0x800f_ffff_ffff_ffff)),
				3 => f64::INFINITY,
				4 => f64::NEG_INFINITY,
				5 => 0.0,
				6 => -0.0,
				// subnormal
				7 => f64::from_bits(r & 0x800f_ffff_ffff_ffff),
				8 | 9 => f64::from_bits(r),
				10 => f64::MAX,
				11 => -1e300,
				12 => TIES[(r % 10) as usize],
				_ => (r as i64 as f64) * 10f64.powi((r % 41) as i32 - 30),
			}
		}

		fn int(&mut self) -> i64 {
			match self.below(8) {
				0 => i64::MIN,
				1 => i64::MAX,
				2 => 0,
				3 => -1,
				_ => (self.next() as i64) >> self.below(64),
			}
		}

		fn uint(&mut self) -> u64 {
			match self.below(6) {
				0 => u64::MAX,
				1 => 0,
				_ => self.next() >> self.below(64),
			}
		}
	}

	/// sprintf_into, and sprintf through it, against the implementation they replaced (`frozen`): formats of one to
	/// four conversions with random flags (repeated too), widths and precisions (written, or `*` from the arguments:
	/// negative, or not an int), length modifiers, every conversion (an unknown one too, and a format ending inside a
	/// conversion), arguments of the right kind, of the wrong kind or none, and every kind of value (NaN of both signs
	/// and with payloads, the infinities, -0, subnormals, the largest double, rounding ties, characters of more than one
	/// byte); appended to a buffer that holds text already. Then sally's own line formats with random values
	#[test]
	fn sprintf_into_writes_what_sprintf_wrote() {
		const CONVERSIONS: &[u8] = b"diuxXocsfFeEgG%k";
		const TEXTS: [&str; 8] = ["", "KN0", "LHR-9365E531", "a b", "\u{e9}", "\u{ff}x\u{80}", "\u{65e5}\u{672c}", "-"];
		const LENGTHS: [&str; 10] = ["", "", "", "h", "hh", "l", "ll", "z", "j", "L"];
		const BEFORE: &str = "\u{e9}|";
		let mut r = Rng(0x9e37_79b9_7f4a_7c15);
		let mut buf = String::new();
		let mut check = |fmt: &str, args: &[Arg]| {
			let want = frozen::sprintf(fmt, args);
			assert_eq!(sprintf(fmt, args), want, "{fmt:?} {args:?}");
			buf.clear();
			buf.push_str(BEFORE);
			sprintf_into(&mut buf, fmt, args);
			assert_eq!(&buf[BEFORE.len()..], want, "{fmt:?} {args:?} after text");
		};
		for _ in 0..200_000 {
			let mut fmt = String::new();
			let mut args = Vec::new();
			let n = 1 + r.below(4);
			for k in 0..n {
				if r.below(2) == 0 {
					fmt.push_str(["<", " ", "ab", "%%", "\u{e9}|"][r.below(5) as usize]);
				}
				fmt.push('%');
				for _ in 0..r.below(4) {
					fmt.push(['-', '+', ' ', '0', '#'][r.below(5) as usize]);
				}
				match r.below(5) {
					0 | 1 => {}
					2 => fmt.push_str(&r.below(40).to_string()),
					3 => {
						fmt.push('*');
						args.push(Arg::I(r.below(81) as i64 - 40));
					}
					_ => {
						fmt.push('*');
						args.push(Arg::U(r.below(40)));
					}
				}
				match r.below(6) {
					0 | 1 => {}
					2 => fmt.push('.'),
					3 => fmt.push_str(&format!(".{}", r.below(25))),
					4 => {
						fmt.push_str(".*");
						args.push(Arg::I(r.below(31) as i64 - 5));
					}
					_ => {
						fmt.push_str(".*");
						args.push(Arg::F(2.0));
					}
				}
				fmt.push_str(LENGTHS[r.below(LENGTHS.len() as u64) as usize]);
				if k + 1 == n && r.below(40) == 0 {
					break;
				}
				let conv = CONVERSIONS[r.below(CONVERSIONS.len() as u64) as usize];
				fmt.push(conv as char);
				if conv == b'%' {
					// %% takes no argument
					continue;
				}
				let text = TEXTS[r.below(TEXTS.len() as u64) as usize];
				match r.below(10) {
					// none (for the last conversion: one before would hand its argument to a later `*`): a conversion
					// past the arguments writes nothing
					0 if k + 1 == n => {}
					1 => args.push(match r.below(4) {
						0 => Arg::I(r.int()),
						1 => Arg::U(r.uint()),
						2 => Arg::F(r.float()),
						_ => Arg::S(text),
					}),
					_ => args.push(match conv {
						b'f' | b'F' | b'e' | b'E' | b'g' | b'G' => Arg::F(r.float()),
						b's' => Arg::S(text),
						b'c' => Arg::I(r.below(256) as i64),
						_ if r.below(2) == 0 => Arg::I(r.int()),
						_ => Arg::U(r.uint()),
					}),
				}
			}
			check(&fmt, &args);
		}
		let lines = [
			"%0.6f ",
			"%s B %u %u %u %d %+le\n",
			"%s POSE %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f  %llu\r\n",
			"Q %s %s %llu %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f %.5f %.5f %.5f %.5f %.5f %.5f %.4f %.4f %.4f %.5f ",
			"S %s %s %llu %.6f %s %.4f %.6f %.6f %x %.3f\n",
			"%s DIAG %.3f %s %d %d %.1f %.0f %d %d %d %.1f %d %d %s %.2f %d %.2f",
			"LH_UP %d %+le   %+le   %+le\n",
			" %u %u %u %u %u %u %u %.6g %u %u %u %u",
			"%.9g|%.17g|%15f|%15e|%-12s|",
		];
		for fmt in lines {
			for _ in 0..20_000 {
				let mut args = Vec::new();
				for c in fmt.bytes().filter(|c| b"sdiuxfeg".contains(c)) {
					args.push(match c {
						b's' => Arg::S(TEXTS[r.below(TEXTS.len() as u64) as usize]),
						b'd' | b'i' => Arg::I(r.int()),
						b'u' | b'x' => Arg::U(r.uint()),
						_ => Arg::F(r.float()),
					});
				}
				check(fmt, &args);
			}
		}
	}

	#[test]
	fn byte_strings_go_through_as_bytes() {
		let name = b"LHR-\xff\x80\xc3(";
		assert_eq!(bsprintf("[%s] %u %.1f", &[BArg::S(name), BArg::U(7), BArg::F(2.25)]), b"[LHR-\xff\x80\xc3(] 7 2.2");
		// a width counts bytes, as the C's
		assert_eq!(bsprintf("%6s|%-4s|", &[BArg::S(b"\xe9a"), BArg::S(b"\xe9")]), b"    \xe9a|\xe9   |");
	}
}

/// The implementation sprintf_into replaced (a String for each conversion and for the padding's parts), kept as it was
/// so that the tests hold the new one to it byte for byte (`sprintf_into_writes_what_sprintf_wrote`)
#[cfg(test)]
mod frozen {
	use super::{Arg, Spec};

	/// sprintf: `fmt` with `args`. A conversion without its argument, or with one of another kind, writes nothing for it
	/// (the C would read whatever the stack held: a caller's bug either way)
	pub fn sprintf(fmt: &str, args: &[Arg]) -> String {
		let mut out = String::with_capacity(fmt.len() + 16 * args.len());
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
			convert(&mut out, conv, s, arg);
		}
		out
	}

	/// One conversion
	fn convert(out: &mut String, conv: u8, s: Spec, arg: Option<Arg>) {
		match (conv, arg) {
			(b'd' | b'i', Some(Arg::I(v))) => integer(out, s, v < 0, &v.unsigned_abs().to_string(), ""),
			(b'd' | b'i', Some(Arg::U(v))) => integer(out, s, false, &v.to_string(), ""),
			(b'u', Some(Arg::U(v))) => integer(out, Spec { plus: false, space: false, ..s }, false, &v.to_string(), ""),
			(b'u', Some(Arg::I(v))) => {
				integer(out, Spec { plus: false, space: false, ..s }, false, &(v as u64).to_string(), "")
			}
			(b'x' | b'X' | b'o', Some(a @ (Arg::I(_) | Arg::U(_)))) => {
				let v = match a {
					Arg::I(v) => v as u64,
					Arg::U(v) => v,
					_ => 0,
				};
				let digits = match conv {
					b'x' => format!("{v:x}"),
					b'X' => format!("{v:X}"),
					_ => format!("{v:o}"),
				};
				let prefix = match conv {
					_ if !s.alt || v == 0 => "",
					b'x' => "0x",
					b'X' => "0X",
					_ => "",
				};
				let mut digits = digits;
				if conv == b'o' && s.alt && !digits.starts_with('0') {
					digits.insert(0, '0');
				}
				integer(out, Spec { plus: false, space: false, ..s }, false, &digits, prefix);
			}
			(b'c', Some(Arg::I(v))) => pad(out, s, "", &((v as u8) as char).to_string(), false),
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
				pad(out, Spec { zero: false, ..s }, "", v, false);
			}
			(b'f' | b'F' | b'e' | b'E' | b'g' | b'G', Some(Arg::F(v))) => float(out, conv, s, v),
			_ => {}
		}
	}

	/// An integer's digits with the precision's leading zeros, the sign and prefix, and the width
	fn integer(out: &mut String, s: Spec, negative: bool, digits: &str, prefix: &str) {
		let mut body = String::new();
		let digits = if s.precision == Some(0) && digits == "0" { "" } else { digits };
		if let Some(p) = s.precision {
			for _ in digits.len()..p {
				body.push('0');
			}
		}
		body.push_str(digits);
		let sign = if negative {
			"-"
		} else if s.plus {
			"+"
		} else if s.space {
			" "
		} else {
			""
		};
		let lead = format!("{sign}{prefix}");
		// the 0 flag is ignored when a precision is given
		pad(out, Spec { zero: s.zero && s.precision.is_none(), ..s }, &lead, &body, true);
	}

	/// The sign or prefix, the body, padded to the width: on the left with spaces, on the right with spaces (the - flag),
	/// or between the sign and the body with zeros (the 0 flag, for numbers)
	fn pad(out: &mut String, s: Spec, lead: &str, body: &str, numeric: bool) {
		let len = lead.chars().count() + body.chars().count();
		let fill = s.width.saturating_sub(len);
		if s.left {
			out.push_str(lead);
			out.push_str(body);
			out.extend(std::iter::repeat_n(' ', fill));
		} else if s.zero && numeric {
			out.push_str(lead);
			out.extend(std::iter::repeat_n('0', fill));
			out.push_str(body);
		} else {
			out.extend(std::iter::repeat_n(' ', fill));
			out.push_str(lead);
			out.push_str(body);
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
		if !v.is_finite() {
			let t = if v.is_nan() { "nan" } else { "inf" };
			let t = if upper { t.to_ascii_uppercase() } else { t.to_string() };
			// NaN and infinity take no zero padding
			pad(out, Spec { zero: false, ..s }, sign, &t, true);
			return;
		}
		let a = v.abs();
		let p = s.precision.unwrap_or(6);
		let mut body = match conv {
			b'f' | b'F' => fixed_body(a, p, s.alt),
			b'e' | b'E' => exp_body(a, p, s.alt, upper),
			_ => {
				// %g: the precision P (0 is 1); the style e would have exponent X after rounding to P digits: f with
				// precision P - 1 - X when P > X >= -4, else e with P - 1; then the trailing zeros go (unless #)
				let p = if p == 0 { 1 } else { p };
				let x = exponent_after_rounding(a, p - 1);
				let t = if (x as i64) < p as i64 && x >= -4 {
					fixed_body(a, (p as i64 - 1 - x as i64) as usize, s.alt)
				} else {
					exp_body(a, p - 1, s.alt, upper)
				};
				if s.alt {
					t
				} else {
					strip_zeros(&t)
				}
			}
		};
		if upper && matches!(conv, b'F') {
			body = body.to_ascii_uppercase();
		}
		pad(out, s, sign, &body, true);
	}

	/// "%.{p}f" of a non-negative finite value (with # a point even at precision 0)
	fn fixed_body(a: f64, p: usize, alt: bool) -> String {
		let mut t = format!("{a:.p$}");
		if alt && p == 0 {
			t.push('.');
		}
		t
	}

	/// "%.{p}e" of a non-negative finite value: the exponent signed, two digits at least
	fn exp_body(a: f64, p: usize, alt: bool, upper: bool) -> String {
		let t = format!("{a:.p$e}");
		let (m, e) = t.split_once('e').unwrap_or((&t, "0"));
		let e: i32 = e.parse().unwrap_or(0);
		let mut m = m.to_string();
		if alt && p == 0 {
			m.push('.');
		}
		format!("{}{}{}{:02}", m, if upper { 'E' } else { 'e' }, if e < 0 { '-' } else { '+' }, e.unsigned_abs())
	}

	/// The decimal exponent of a non-negative finite value written with `p` digits after the point in style e
	fn exponent_after_rounding(a: f64, p: usize) -> i32 {
		let t = format!("{a:.p$e}");
		t.split_once('e').and_then(|(_, e)| e.parse().ok()).unwrap_or(0)
	}

	/// %g's trailing zeros of the fraction removed, and the point if nothing follows it
	fn strip_zeros(t: &str) -> String {
		let (mantissa, exp) = match t.find(['e', 'E']) {
			Some(i) => (&t[..i], &t[i..]),
			None => (t, ""),
		};
		let m = if mantissa.contains('.') { mantissa.trim_end_matches('0').trim_end_matches('.') } else { mantissa };
		format!("{m}{exp}")
	}
}
