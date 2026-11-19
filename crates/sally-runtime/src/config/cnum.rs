//! The C library's number parsing, printing and casts as the config store uses them: glibc's, in the "C" locale (sally
//! never calls setlocale), over C strings (a byte slice read up to its first NUL, or to its end). Each function stands
//! for one C function or cast and says which; the floats are read and printed by sally-math's (its libc.rs and
//! printf), the integers and the casts are here. The store calls nothing else for numbers.
//!
//! The integer widths follow the platform's C `long` (64 bits on Linux, 32 on Windows), as both glibc's and the
//! ucrt's strtol / strtoul do. Everything else is glibc's behaviour. Two known ucrt differences are not reproduced: it
//! prints NaN as "nan", "-nan(ind)" and the like where glibc prints "nan" / "-nan", and its strtod gives "nan(...)"
//! payloads its own meaning.

use std::ffi::{c_int, c_long, c_ulong};

use sally_math::printf::{sprintf, Arg};

/// The bytes of a C string: everything before the first NUL
pub(crate) fn c_str(s: &[u8]) -> &[u8] {
	match s.iter().position(|&b| b == 0) {
		Some(n) => &s[..n],
		None => s,
	}
}

/// isspace in the "C" locale
fn is_space(c: u8) -> bool {
	matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

/// The byte at `i`, or the NUL that ends a C string past its end
fn at(s: &[u8], i: usize) -> u8 {
	s.get(i).copied().unwrap_or(0)
}

/// strtod: the value and how many bytes it read (0 when nothing was converted)
pub(crate) fn strtod(s: &[u8]) -> (f64, usize) {
	sally_math::c_strtod(s)
}

/// strtof: the value and how many bytes it read. The float is rounded once, from the text (not through a double)
pub(crate) fn strtof(s: &[u8]) -> (f32, usize) {
	sally_math::c_strtof(s)
}

/// atof: strtod(s, NULL)
pub(crate) fn atof(s: &[u8]) -> f64 {
	sally_math::c_atof(s)
}

/// What glibc's ____strtoul_l_internal reads: (negative, magnitude, overflowed past `max`, bytes read). Nothing read is
/// (0, 0) except after a "0x" prefix with no digit, which reads as the "0"
fn scan_integer(s: &[u8], base: u32, max: u64) -> (bool, u64, bool, usize) {
	let s = c_str(s);
	let mut i = 0;
	while is_space(at(s, i)) {
		i += 1;
	}
	let negative = at(s, i) == b'-';
	if negative || at(s, i) == b'+' {
		i += 1;
	}
	let mut base = base;
	if at(s, i) == b'0' {
		if (base == 0 || base == 16) && at(s, i + 1).eq_ignore_ascii_case(&b'x') {
			i += 2;
			base = 16;
		} else if base == 0 {
			base = 8;
		}
	} else if base == 0 {
		base = 10;
	}
	let save = i;
	let mut value: u64 = 0;
	let mut overflow = false;
	while let Some(d) = (at(s, i) as char).to_digit(36).filter(|&d| d < base) {
		match value.checked_mul(base as u64).and_then(|v| v.checked_add(d as u64)) {
			Some(v) if v <= max => value = v,
			_ => overflow = true,
		}
		i += 1;
	}
	if i == save {
		let end = if save >= 2 && at(s, save - 1).eq_ignore_ascii_case(&b'x') && at(s, save - 2) == b'0' {
			save - 1
		} else {
			0
		};
		return (false, 0, false, end);
	}
	(negative, value, overflow, i)
}

/// ULONG_MAX, as the u64 scan_integer counts in
#[allow(clippy::unnecessary_cast)] // a no-op where the C long is 64 bits, not on Windows
const ULONG_MAX: u64 = c_ulong::MAX as u64;

/// -LONG_MIN, the largest magnitude strtol takes after a minus sign
#[allow(clippy::unnecessary_cast)] // as ULONG_MAX
const LONG_MIN_MAGNITUDE: u64 = c_long::MIN.unsigned_abs() as u64;

/// strtol: the value and how many bytes it read. Out of range saturates at LONG_MIN / LONG_MAX
pub(crate) fn strtol(s: &[u8], base: u32) -> (c_long, usize) {
	let (negative, value, overflow, end) = scan_integer(s, base, ULONG_MAX);
	let v = if negative {
		if overflow || value > LONG_MIN_MAGNITUDE {
			c_long::MIN
		} else {
			(0 as c_long).wrapping_sub(value as c_long)
		}
	} else if overflow || value > c_long::MAX as u64 {
		c_long::MAX
	} else {
		value as c_long
	};
	(v, end)
}

/// strtoul: the value and how many bytes it read. Out of range saturates at ULONG_MAX; a minus sign negates in the
/// unsigned type ("-1" is ULONG_MAX)
pub(crate) fn strtoul(s: &[u8], base: u32) -> (c_ulong, usize) {
	let (negative, value, overflow, end) = scan_integer(s, base, ULONG_MAX);
	let v = if overflow {
		c_ulong::MAX
	} else if negative {
		(value as c_ulong).wrapping_neg()
	} else {
		value as c_ulong
	};
	(v, end)
}

/// (uint32_t) of an unsigned long: its low 32 bits
#[allow(clippy::unnecessary_cast)] // a no-op where the C long is 32 bits (Windows)
pub(crate) fn uint32_of_ulong(v: c_ulong) -> u32 {
	v as u32
}

/// atoi: glibc's (int)strtol(s, NULL, 10), the long truncated to an int (the ucrt's long is already an int)
pub(crate) fn atoi(s: &[u8]) -> c_int {
	sally_math::c_atoi(s)
}

/// sscanf's "%d" on what follows the literal text of the format: white space, then a sign and at least one decimal
/// digit, read as strtol(s, 10) and stored into an int (glibc's vfscanf truncates the long). None when no digit
pub(crate) fn scan_int(s: &[u8]) -> Option<c_int> {
	let s = c_str(s);
	let mut i = 0;
	while is_space(at(s, i)) {
		i += 1;
	}
	let mut j = i;
	if at(s, j) == b'-' || at(s, j) == b'+' {
		j += 1;
	}
	if !at(s, j).is_ascii_digit() {
		return None;
	}
	Some(strtol(&s[i..], 10).0 as c_int)
}

/// (float)x, as the x86-64 C compilers convert (cvtsd2ss): rounded to nearest, ties to even; a NaN keeps its sign and
/// the top of its payload, quieted (spelled out: Rust leaves a NaN's sign and payload through `as` unspecified)
pub(crate) fn to_float(x: f64) -> f32 {
	if x.is_nan() {
		let b = x.to_bits();
		f32::from_bits((((b >> 63) as u32) << 31) | 0x7fc0_0000 | ((b >> 29) as u32 & 0x003f_ffff))
	} else {
		x as f32
	}
}

/// (double)x for a float (cvtss2sd): exact; a NaN keeps its sign and payload, quieted
pub(crate) fn to_double(x: f32) -> f64 {
	if x.is_nan() {
		let b = x.to_bits() as u64;
		f64::from_bits(((b >> 31) << 63) | 0x7ff8_0000_0000_0000 | ((b & 0x003f_ffff) << 29))
	} else {
		x as f64
	}
}

/// (uint32_t)roundf((float)x), as gcc and clang compile config_entry_as_uint32_t: roundf (half away from zero), then
/// cvttss2si into a 64-bit register and its low 32 bits. A NaN or a value outside the 64-bit range gives
/// 0x8000000000000000 there, so 0; a negative value wraps (-1 is 4294967295)
pub(crate) fn uint32_from_roundf(x: f64) -> u32 {
	let r = to_float(x).round();
	let wide =
		if (-9_223_372_036_854_775_808.0..9_223_372_036_854_775_808.0).contains(&r) { r as i64 } else { i64::MIN };
	wide as u32
}

/// (int32_t)x, as gcc and clang compile config_set_float's write to an attached int: cvttsd2si into a 32-bit register,
/// truncating; a NaN or a value outside the int range gives INT_MIN
pub(crate) fn int32_from_double(x: f64) -> i32 {
	if x > -2_147_483_649.0 && x < 2_147_483_648.0 {
		x as i32
	} else {
		i32::MIN
	}
}

/// printf's "%.<prec>f" (sally_math's printf: glibc's text, its spelling of a NaN or an infinity included)
pub(crate) fn fixed(out: &mut Vec<u8>, v: f64, prec: usize) {
	out.extend_from_slice(sprintf(&format!("%.{prec}f"), &[Arg::F(v)]).as_bytes());
}

/// printf's "%.<prec>e"
pub(crate) fn exp(out: &mut Vec<u8>, v: f64, prec: usize) {
	out.extend_from_slice(sprintf(&format!("%.{prec}e"), &[Arg::F(v)]).as_bytes());
}

/// A printf field width: `text` right-justified in `width` columns (never cut)
pub(crate) fn pad(out: &mut Vec<u8>, text: &[u8], width: usize) {
	out.resize(out.len() + width.saturating_sub(text.len()), b' ');
	out.extend_from_slice(text);
}

#[cfg(test)]
mod tests {
	use super::*;

	// Every expected value below is glibc 2.39's, printed by a C program calling these functions on these strings
	#[test]
	fn integers_read_what_glibc_reads() {
		assert_eq!(strtoul(b" +12", 10), (12, 4));
		assert_eq!(strtoul(b"", 10), (0, 0));
		assert_eq!(strtol(b"0x1f", 0), (31, 4));
		assert_eq!(strtol(b"0x", 0), (0, 1));
		assert_eq!(strtol(b"010", 0), (8, 3));
		assert_eq!(strtol(b"0b101", 0), (0, 1));
		assert_eq!(atoi(b"-0.5"), 0);
		assert_eq!(scan_int(b" 3x"), Some(3));
		assert_eq!(scan_int(b"-"), None);
	}

	#[cfg(all(target_os = "linux", target_pointer_width = "64"))]
	#[test]
	fn integers_wrap_and_saturate_as_on_linux() {
		assert_eq!(strtoul(b"-1", 10), (u64::MAX, 2));
		assert_eq!(strtoul(b"99999999999999999999", 10), (u64::MAX, 20));
		assert_eq!(strtol(b"-99999999999999999999", 10), (i64::MIN, 21));
		assert_eq!(atoi(b"4294967297"), 1);
		assert_eq!(atoi(b"-99999999999999999999"), 0);
		assert_eq!(scan_int(b"4294967296"), Some(0));
	}

	#[test]
	fn casts_follow_the_x86_instructions() {
		assert_eq!(to_double(f32::from_bits(0xffff_ffff)).to_bits(), 0xffff_ffff_e000_0000);
		assert_eq!(uint32_from_roundf(-1.0), 0xffff_ffff);
		assert_eq!(uint32_from_roundf(2.5), 3);
		assert_eq!(uint32_from_roundf(4294967296.0), 0);
		assert_eq!(uint32_from_roundf(1e30), 0);
		assert_eq!(uint32_from_roundf(f64::NAN), 0);
		assert_eq!(int32_from_double(-2.9), -2);
		assert_eq!(int32_from_double(3e9), i32::MIN);
		assert_eq!(int32_from_double(f64::NAN), i32::MIN);
	}

	#[test]
	fn printing_matches_glibc() {
		let p = |v: f64, e: bool| {
			let mut out = Vec::new();
			if e {
				exp(&mut out, v, 6)
			} else {
				fixed(&mut out, v, 12)
			}
			String::from_utf8(out).unwrap()
		};
		assert_eq!(p(1e-5, true), "1.000000e-05");
		assert_eq!(p(-0.0, true), "-0.000000e+00");
		assert_eq!(p(1e300, true), "1.000000e+300");
		assert_eq!(p(-1e-20, false), "-0.000000000000");
		assert_eq!(p(f64::from_bits(0xfff8_0000_0000_0000), false), "-nan");
		assert_eq!(p(f64::NEG_INFINITY, true), "-inf");
		let mut out = Vec::new();
		pad(&mut out, b"nan", 15);
		assert_eq!(out, b"            nan");
	}
}
