//! The C library's fmax, fmin and pow as the C sally ported from gets them: on a tie of +0 and -0 the platforms' fmax and
//! fmin differ, and pow, erfcf, sin, cos, acos and exp are the C library's own. And its number conversions where their
//! exact value shows: strtod, strtof and atoi as glibc reads a number from a string (the device configs). printf is
//! `printf`.

/// C's `fmax` as the C core gets it. On Linux x86_64 that is glibc's assembly fmax (sysdeps/x86_64/fpu/s_fmax.S,
/// checked on glibc 2.39): `maxsd`, which on a tie (+0 against -0 included) returns the SECOND argument; a quiet NaN
/// argument gives the other one, two NaNs the first. Elsewhere (llvm-mingw's clang lowers fmax to llvm.maxnum, and
/// mingw-w64's own fmax is `isgreaterequal(x, y) || isnan(y) ? x : y`) a tie returns the FIRST argument. Only the sign
/// of a zero can tell the two apart, never a finite value; f64::max leaves exactly that unspecified, so it is not used.
#[cfg(all(target_arch = "x86_64", target_os = "linux", target_env = "gnu"))]
#[inline]
pub fn c_fmax(x: f64, y: f64) -> f64 {
	if y.is_nan() {
		x
	} else if x.is_nan() {
		y
	} else if x > y {
		x
	} else {
		y
	}
}
#[cfg(not(all(target_arch = "x86_64", target_os = "linux", target_env = "gnu")))]
#[inline]
pub fn c_fmax(x: f64, y: f64) -> f64 {
	if y.is_nan() {
		x
	} else if x.is_nan() {
		y
	} else if x >= y {
		x
	} else {
		y
	}
}

/// C's `fmin`, as c_fmax: glibc x86_64 `minsd` (a tie returns the second argument), elsewhere the first
#[cfg(all(target_arch = "x86_64", target_os = "linux", target_env = "gnu"))]
#[inline]
pub fn c_fmin(x: f64, y: f64) -> f64 {
	if y.is_nan() {
		x
	} else if x.is_nan() {
		y
	} else if x < y {
		x
	} else {
		y
	}
}
#[cfg(not(all(target_arch = "x86_64", target_os = "linux", target_env = "gnu")))]
#[inline]
pub fn c_fmin(x: f64, y: f64) -> f64 {
	if y.is_nan() {
		x
	} else if x.is_nan() {
		y
	} else if x <= y {
		x
	} else {
		y
	}
}

/// C's `pow`: the C library's own, called as the C core calls it (gcc emits `call pow` for its one use, a variable
/// exponent; LLVM's pow simplifications do not apply to it either, but this leaves nothing to chance)
#[inline]
pub fn c_pow(x: f64, y: f64) -> f64 {
	// SAFETY: a pure C library function of two doubles
	unsafe { pow(x, y) }
}

/// C's `erfcf`: the C library's single-precision complementary error function (the light's outlier test is in floats)
#[inline]
pub fn c_erfcf(x: f32) -> f32 {
	// SAFETY: a pure C library function of one float
	unsafe { erfcf(x) }
}

/// C's `sin`: the C library's own (the map solve's rotation vectors and slerp)
#[inline]
pub fn c_sin(x: f64) -> f64 {
	// SAFETY: a pure C library function of one double
	unsafe { sin(x) }
}

/// C's `cos`: the C library's own
#[inline]
pub fn c_cos(x: f64) -> f64 {
	// SAFETY: a pure C library function of one double
	unsafe { cos(x) }
}

/// C's `acos`: the C library's own
#[inline]
pub fn c_acos(x: f64) -> f64 {
	// SAFETY: a pure C library function of one double
	unsafe { acos(x) }
}

/// C's `exp`: the C library's own (the map solve's information fade)
#[inline]
pub fn c_exp(x: f64) -> f64 {
	// SAFETY: a pure C library function of one double
	unsafe { exp(x) }
}

/// C's `sincos` (glibc's): both at once, as a compiler calls it for the sin and the cos of one argument (the driver's
/// turns); glibc's sincos is its own code path, so its results are not assumed to be sin's and cos's. The ucrt has
/// none: llvm-mingw's clang calls sin and cos there
#[cfg(all(target_os = "linux", target_env = "gnu"))]
#[inline]
pub fn c_sincos(x: f64) -> (f64, f64) {
	let (mut s, mut c) = (0.0, 0.0);
	// SAFETY: sincos writes one double through each pointer it is given
	unsafe { sincos(x, &mut s, &mut c) };
	(s, c)
}

extern "C" {
	fn pow(x: f64, y: f64) -> f64;
	fn erfcf(x: f32) -> f32;
	fn sin(x: f64) -> f64;
	fn cos(x: f64) -> f64;
	fn acos(x: f64) -> f64;
	fn exp(x: f64) -> f64;
	#[cfg(all(target_os = "linux", target_env = "gnu"))]
	fn sincos(x: f64, s: *mut f64, c: *mut f64);
}

// ---- strtod, strtof, atof, atoi: glibc's (stdlib/strtod_l.c, strtod_nan_main.c, strtol_l.c; glibc 2.39) in the "C"
// locale. A string is the bytes up to the slice's end or its first NUL, whichever comes first, as the C reads a string
// up to its terminating NUL.

/// isspace in the "C" locale
#[inline]
fn c_isspace(c: u8) -> bool {
	matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// The byte at `i`, NUL past the end
#[inline]
fn at(s: &[u8], i: usize) -> u8 {
	s.get(i).copied().unwrap_or(0)
}

/// STRNCASECMP(s + i, lit, lit.len()) == 0 for a lower-case ASCII literal
fn starts_with_ci(s: &[u8], i: usize, lit: &[u8]) -> bool {
	lit.iter().enumerate().all(|(k, &l)| at(s, i + k).to_ascii_lowercase() == l)
}

/// A number as strtod reads it from the start of a string, before it is rounded to a type; `len` is how many bytes it
/// takes (where the C sets endptr)
enum Scanned<'a> {
	/// No number: the C returns +0 (even after a '-') and endptr is the string itself
	NoNumber,
	/// The digits (the integer part without its leading zeros, the fraction) and the exponent (base 10, or base 2 for
	/// a hexadecimal number), saturated far beyond any value a double can hold
	Finite {
		negative: bool,
		hex: bool,
		int: &'a [u8],
		frac: &'a [u8],
		exp: i64,
		len: usize,
	},
	Infinity {
		negative: bool,
		len: usize,
	},
	/// NAN or NAN(n-char-sequence); `payload` is what strtoull made of the sequence when it read all of it
	Nan {
		negative: bool,
		payload: Option<u64>,
		len: usize,
	},
}

/// An exponent beyond which every value is 0 or infinite whatever its digits (a string has fewer than 2^40 of them)
const EXP_SATURATED: i64 = 1 << 50;

/// ____STRTOF_INTERNAL's reading of the string, in its order: white space, a sign, INF / INFINITY / NAN / NAN(...) case
/// insensitively, or a decimal or "0x" hexadecimal number (a point with a digit on at least one side, an exponent `e`
/// or, for hex, `p` that counts only with a digit after its optional sign)
fn scan(s: &[u8]) -> Scanned<'_> {
	let mut i = 0;
	while c_isspace(at(s, i)) {
		i += 1;
	}
	let mut negative = false;
	match at(s, i) {
		b'-' => {
			negative = true;
			i += 1;
		}
		b'+' => i += 1,
		_ => {}
	}
	let c = at(s, i);
	if !(c.is_ascii_digit() || (c == b'.' && at(s, i + 1).is_ascii_digit())) {
		if starts_with_ci(s, i, b"inf") {
			let len = if starts_with_ci(s, i + 3, b"inity") { i + 8 } else { i + 3 };
			return Scanned::Infinity { negative, len };
		}
		if starts_with_ci(s, i, b"nan") {
			let mut len = i + 3;
			let mut payload = None;
			if at(s, len) == b'(' {
				// STRTOD_NAN: the n-char-sequence is [0-9A-Za-z_]*; without its ')' only "NAN" is read
				let start = len + 1;
				let mut k = start;
				while at(s, k).is_ascii_alphanumeric() || at(s, k) == b'_' {
					k += 1;
				}
				if at(s, k) == b')' {
					payload = nan_payload(&s[start..k]);
					len = k + 1;
				}
			}
			return Scanned::Nan { negative, payload, len };
		}
		return Scanned::NoNumber;
	}

	let hex = c == b'0' && at(s, i + 1).eq_ignore_ascii_case(&b'x');
	if hex {
		i += 2;
	}
	let is_digit = |c: u8| c.is_ascii_digit() || (hex && matches!(c.to_ascii_lowercase(), b'a'..=b'f'));
	let start_of_digits = i;
	while at(s, i) == b'0' {
		i += 1;
	}
	let c = at(s, i);
	let lc = c.to_ascii_lowercase();
	let more = is_digit(c)
		|| (c == b'.' && (!hex || i != start_of_digits || is_digit(at(s, i + 1))))
		|| (hex && i != start_of_digits && lc == b'p')
		|| (!hex && lc == b'e');
	if !more {
		// zeros and nothing after them that continues a number: +-0, read up to the last zero (a bare "0x" up to its 0)
		let len = if i != start_of_digits {
			i
		} else if hex {
			i - 1
		} else {
			0
		};
		return Scanned::Finite { negative, hex, int: &[], frac: &[], exp: 0, len };
	}

	let int_start = i;
	while is_digit(at(s, i)) {
		i += 1;
	}
	let int = &s[int_start..i];
	let mut frac: &[u8] = &[];
	if at(s, i) == b'.' {
		i += 1;
		let frac_start = i;
		while is_digit(at(s, i)) {
			i += 1;
		}
		frac = &s[frac_start..i];
	}
	let mut exp = 0i64;
	if at(s, i).to_ascii_lowercase() == if hex { b'p' } else { b'e' } {
		let mut k = i + 1;
		let exp_negative = at(s, k) == b'-';
		if exp_negative || at(s, k) == b'+' {
			k += 1;
		}
		if at(s, k).is_ascii_digit() {
			while at(s, k).is_ascii_digit() {
				exp = (exp * 10 + (at(s, k) - b'0') as i64).min(EXP_SATURATED);
				k += 1;
			}
			if exp_negative {
				exp = -exp;
			}
			i = k;
		}
	}
	Scanned::Finite { negative, hex, int, frac, exp, len: i }
}

/// STRTOD_NAN's payload: strtoull(sequence, &endp, 0) when endp is the sequence's end (so for an empty one too, as 0);
/// glibc passes no binary-constant flag there, so "0b" is not a prefix. Past ULLONG_MAX strtoull returns ULLONG_MAX
fn nan_payload(seq: &[u8]) -> Option<u64> {
	let (digits, radix) = match seq {
		[b'0', x, rest @ ..] if x.eq_ignore_ascii_case(&b'x') => (rest, 16),
		[b'0', ..] => (seq, 8),
		_ => (seq, 10),
	};
	if radix == 16 && digits.is_empty() {
		return None; // "0x" alone: strtoull reads the 0 and stops at the x
	}
	let mut mant = 0u64;
	for &c in digits {
		let d = (c as char).to_digit(radix)? as u64;
		mant = mant.checked_mul(radix as u64).and_then(|m| m.checked_add(d)).unwrap_or(u64::MAX);
	}
	Some(mant)
}

/// A decimal as Rust's parser gets it (Rust's float parsing rounds correctly, as glibc's does): the significant digits
/// as d.ddd and the exponent of the first. Rust saturates a long exponent before it subtracts the digits after the
/// point, so the exponent it gets is kept small: past +-400 every value is 0 or infinite in either type
enum Decimal {
	Zero,
	Huge,
	Tiny,
	Text(String),
}

fn decimal_text(int: &[u8], frac: &[u8], exp: i64) -> Decimal {
	let mut digits = int.iter().chain(frac).copied().skip_while(|&d| d == b'0');
	let Some(first) = digits.next() else { return Decimal::Zero };
	let mut text = String::with_capacity(int.len() + frac.len() + 8);
	text.push(first as char);
	text.push('.');
	let mut n = 1i64;
	for d in digits {
		text.push(d as char);
		n += 1;
	}
	// the value is int.frac * 10^exp: its first significant digit stands for 10^(exp - frac.len() + n - 1)
	let e = exp - frac.len() as i64 + n - 1;
	if e > 400 {
		return Decimal::Huge;
	}
	if e < -400 {
		return Decimal::Tiny;
	}
	text.push('e');
	text.push_str(&e.to_string());
	Decimal::Text(text)
}

/// A hexadecimal significand times 2^exp, rounded to nearest (ties to even) into a binary float with `p` significand
/// bits and exponents emin..=emax, subnormals included: the bits of the magnitude (ROUND_AND_RETURN)
fn hex_bits(int: &[u8], frac: &[u8], exp: i64, p: u32, emin: i64, emax: i64) -> u64 {
	// the first 60 or more bits of the digits, then whether any bit after them is set
	let mut m = 0u64;
	let mut sticky = false;
	let mut dropped = 0i64;
	for &c in int.iter().chain(frac).skip_while(|&&d| d == b'0') {
		let d = (c as char).to_digit(16).unwrap_or(0) as u64;
		if m >> 60 == 0 {
			m = (m << 4) | d;
		} else {
			sticky |= d != 0;
			dropped += 1;
		}
	}
	if m == 0 {
		return 0;
	}
	let e2 = exp + 4 * dropped - 4 * frac.len() as i64; // value = m * 2^e2
	let mut x = 63 - m.leading_zeros() as i64 + e2; // value in [2^x, 2^(x+1))
	let lsb = x.max(emin) - (p as i64 - 1); // the exponent of the result's last significand bit
	let shift = lsb - e2;
	let mut q = if shift <= 0 {
		(m as u128) << -shift
	} else if shift >= 128 {
		0
	} else {
		let mm = m as u128;
		let q = mm >> shift;
		let rem = mm & ((1u128 << shift) - 1);
		let half = 1u128 << (shift - 1);
		if rem > half || (rem == half && (sticky || q & 1 == 1)) {
			q + 1
		} else {
			q
		}
	};
	if x < emin {
		// subnormal: the bits are the significand (rounding up to 2^(p-1) gives the smallest normal)
		return q as u64;
	}
	if q >> p != 0 {
		q >>= 1;
		x += 1;
	}
	if x > emax {
		return ((2 * emax + 1) as u64) << (p - 1); // infinity: the exponent all ones, the significand zero
	}
	// the exponent biased by emax, the significand without its leading bit
	(((x + emax) as u64) << (p - 1)) | (q as u64 & ((1u64 << (p - 1)) - 1))
}

/// C's `strtod(s, &end)` as glibc computes it: the double and how many bytes it read (0: no number, end == s)
pub fn c_strtod(s: &[u8]) -> (f64, usize) {
	match scan(s) {
		Scanned::NoNumber => (0.0, 0),
		Scanned::Infinity { negative, len } => (if negative { f64::NEG_INFINITY } else { f64::INFINITY }, len),
		Scanned::Nan { negative, payload, len } => {
			let bits = 0x7ff8_0000_0000_0000 | (payload.unwrap_or(0) & 0x0007_ffff_ffff_ffff);
			(f64::from_bits(bits | ((negative as u64) << 63)), len)
		}
		Scanned::Finite { negative, hex, int, frac, exp, len } => {
			let v = if hex {
				f64::from_bits(hex_bits(int, frac, exp, 53, -1022, 1023))
			} else {
				match decimal_text(int, frac, exp) {
					Decimal::Zero | Decimal::Tiny => 0.0,
					Decimal::Huge => f64::INFINITY,
					Decimal::Text(t) => t.parse().unwrap_or(f64::NAN),
				}
			};
			(if negative { -v } else { v }, len)
		}
	}
}

/// C's `strtof(s, &end)` as glibc computes it (rounded once, straight to a float)
pub fn c_strtof(s: &[u8]) -> (f32, usize) {
	match scan(s) {
		Scanned::NoNumber => (0.0, 0),
		Scanned::Infinity { negative, len } => (if negative { f32::NEG_INFINITY } else { f32::INFINITY }, len),
		Scanned::Nan { negative, payload, len } => {
			let bits = 0x7fc0_0000 | (payload.unwrap_or(0) & 0x003f_ffff) as u32;
			(f32::from_bits(bits | ((negative as u32) << 31)), len)
		}
		Scanned::Finite { negative, hex, int, frac, exp, len } => {
			let v = if hex {
				f32::from_bits(hex_bits(int, frac, exp, 24, -126, 127) as u32)
			} else {
				match decimal_text(int, frac, exp) {
					Decimal::Zero | Decimal::Tiny => 0.0,
					Decimal::Huge => f32::INFINITY,
					Decimal::Text(t) => t.parse().unwrap_or(f32::NAN),
				}
			};
			(if negative { -v } else { v }, len)
		}
	}
}

/// C's `atof(s)`: glibc's is strtod(s, NULL)
#[inline]
pub fn c_atof(s: &[u8]) -> f64 {
	c_strtod(s).0
}

/// C's `atoi(s)`: `(int) strtol(s, NULL, 10)`. White space, a sign, decimal digits; past the range of a long, strtol
/// returns LONG_MAX or LONG_MIN. A long is 64 bits on Linux and the conversion to int keeps the low 32 bits; on Windows
/// a long is 32 bits and the ucrt's atoi saturates at INT_MAX / INT_MIN
pub fn c_atoi(s: &[u8]) -> i32 {
	let mut i = 0;
	while c_isspace(at(s, i)) {
		i += 1;
	}
	let negative = at(s, i) == b'-';
	if negative || at(s, i) == b'+' {
		i += 1;
	}
	let mut v = 0u64;
	let mut overflow = false;
	while at(s, i).is_ascii_digit() {
		if !overflow {
			match v.checked_mul(10).and_then(|v| v.checked_add((at(s, i) - b'0') as u64)) {
				Some(n) => v = n,
				None => overflow = true,
			}
		}
		i += 1;
	}
	#[cfg(not(windows))]
	const LONG_MAX: i64 = i64::MAX;
	#[cfg(windows)]
	const LONG_MAX: i64 = i32::MAX as i64;
	let long = if overflow || v > LONG_MAX as u64 + negative as u64 {
		if negative {
			-LONG_MAX - 1
		} else {
			LONG_MAX
		}
	} else if negative {
		(v as i64).wrapping_neg()
	} else {
		v as i64
	};
	long as i32
}

#[cfg(test)]
mod tests {
	use super::*;

	// glibc 2.39's results on the same strings and doubles

	#[test]
	fn strtod_reads_as_glibc_does() {
		let d = |s: &str| {
			let (v, n) = c_strtod(s.as_bytes());
			(v.to_bits(), n)
		};
		assert_eq!(d("1e"), (0x3ff0000000000000, 1));
		assert_eq!(d(" +.5e1x"), (0x4014000000000000, 6));
		assert_eq!(d("-"), (0, 0)); // no number: +0
		assert_eq!(d("-0x"), (0x8000000000000000, 2));
		assert_eq!(d("0x"), (0, 1));
		assert_eq!(d("0x1p-1075"), (0, 9)); // half the smallest subnormal: ties to even
		assert_eq!(d("0x3p-1076"), (1, 9));
		assert_eq!(d("0x1.fffffffffffff8p1023"), (0x7ff0000000000000, 23));
		assert_eq!(d("infinite"), (0x7ff0000000000000, 3));
		assert_eq!(d("INFINITY"), (0x7ff0000000000000, 8));
		assert_eq!(d("nan(0x1f)"), (0x7ff800000000001f, 9));
		assert_eq!(d("nan(0b1)"), (0x7ff8000000000000, 8));
		assert_eq!(d("-nan(5)"), (0xfff8000000000005, 7));
		assert_eq!(d("nan(1"), (0x7ff8000000000000, 3));
		assert_eq!(d("2.4703282292062328e-324"), (1, 23));
		assert_eq!(d("1e-99999999999999999999"), (0, 23));
		let compensated = format!("0.{}1e1001", "0".repeat(1000));
		assert_eq!(d(&compensated), (0x3ff0000000000000, 1008));
		assert_eq!(d("0.1"), (0x3fb999999999999a, 3));
		assert_eq!(c_atof(b"1.5abc"), 1.5);
		assert_eq!(d("  -1.5e3x"), ((-1500.0f64).to_bits(), 8));
		assert_eq!(d("0x1p3"), (8.0f64.to_bits(), 5));
		assert_eq!(d("0x1p"), (1.0f64.to_bits(), 3));
		assert_eq!(d("0x.8"), (0.5f64.to_bits(), 4));
		assert_eq!(d("0x.p1"), (0, 1));
		assert_eq!(d("1e+"), (1.0f64.to_bits(), 1));
		assert_eq!(d(".5"), (0.5f64.to_bits(), 2));
		assert_eq!(d("."), (0, 0));
		assert_eq!(d("+inf"), (f64::INFINITY.to_bits(), 4));
		assert_eq!(d("-Infinity!"), (f64::NEG_INFINITY.to_bits(), 9));
		assert_eq!(d("infinit"), (f64::INFINITY.to_bits(), 3));
		assert_eq!(d("nan"), (0x7ff8_0000_0000_0000, 3));
		assert_eq!(d("-nan"), (0xfff8_0000_0000_0000, 4));
		assert_eq!(d("nan(0x123)"), (0x7ff8_0000_0000_0123, 10));
		assert_eq!(d("nan(abc)"), (0x7ff8_0000_0000_0000, 8));
		assert_eq!(d("1e99999"), (f64::INFINITY.to_bits(), 7));
		assert_eq!(d("4.9e-324"), (1, 8));
		assert_eq!(d("2.4703282292062327e-324"), (0, 23));
		assert_eq!(d("0x1.00000000000008p0"), (1.0f64.to_bits(), 20));
		assert_eq!(d("0x1.000000000000080001p0"), (0x3ff0_0000_0000_0001, 24));
	}

	#[test]
	fn strtof_rounds_once_to_a_float() {
		let f = |s: &str| {
			let (v, n) = c_strtof(s.as_bytes());
			(v.to_bits(), n)
		};
		assert_eq!(f("16777217"), (0x4b800000, 8));
		assert_eq!(f("3.4028235e38"), (0x7f7fffff, 12));
		assert_eq!(f("1e39"), (0x7f800000, 4));
		assert_eq!(f("-1e-46"), (0x80000000, 6));
		assert_eq!(f("0.1"), (0x3dcccccd, 3));
		assert_eq!(f("nan(0x1f)"), (0x7fc0001f, 9));
		assert_eq!(f("-nan(5)"), (0xffc00005, 7));
		// 1 + 2^-24 + 2^-60: halfway between two floats only after rounding to a double first
		assert_eq!(f("1.000000059604644861"), (0x3f80_0001, 20));
		assert_eq!(f("2.341761589050"), (0x4015_df6c, 14));
		assert_eq!(f("-nan(0x7fffff)"), (0xffff_ffff, 14));
		assert_eq!(f("3.4028236e38"), (0x7f80_0000, 12));
	}

	#[test]
	fn atoi_as_strtol() {
		assert_eq!(c_atoi(b"  -42x"), -42);
		assert_eq!(c_atoi(b"+7"), 7);
		assert_eq!(c_atoi(b"x"), 0);
		#[cfg(not(windows))]
		{
			// a 64-bit long, its low 32 bits kept
			assert_eq!(c_atoi(b"4294967296"), 0);
			assert_eq!(c_atoi(b"-4294967295"), 1);
			assert_eq!(c_atoi(b"2147483648"), i32::MIN);
			assert_eq!(c_atoi(b"9223372036854775808"), -1);
			assert_eq!(c_atoi(b"-9223372036854775809"), 0);
		}
		#[cfg(windows)]
		{
			assert_eq!(c_atoi(b"4294967296"), i32::MAX);
			assert_eq!(c_atoi(b"-4294967295"), i32::MIN);
		}
	}
}
