//! driver_sally.cpp's small helpers: the log, the clocks, the quaternion helpers of the pose math, the string helpers
//! of the settings and the protocol, the driver's folder and the time stamp. Strings are bytes, as the C++'s
//! std::strings are: what comes off the wire or out of a file need not be UTF-8.

use sally_math::printf::{bsprintf, BArg};

/// Log(fmt, ...): vsnprintf into char buf[1400] (a longer line is cut to its first 1399 bytes), then vrserver's log
pub fn log(fmt: &str, args: &[BArg]) {
	let mut buf = bsprintf(fmt, args);
	buf.truncate(1399);
	crate::openvr::driver_log(&buf);
}

/// NowSeconds(): duration<double>(steady_clock::now().time_since_epoch()).count(), monotonic
pub fn now_seconds() -> f64 {
	sally_os::chrono::steady_clock_seconds()
}

/// WallSeconds(): duration<double>(system_clock::now().time_since_epoch()).count()
pub fn wall_seconds() -> f64 {
	sally_os::chrono::system_clock_seconds()
}

/// QuatRotate(q, v): q = w x y z
pub fn quat_rotate(q: &[f64; 4], v: &[f64; 3]) -> [f64; 3] {
	let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
	let tx = 2.0 * (y * v[2] - z * v[1]);
	let ty = 2.0 * (z * v[0] - x * v[2]);
	let tz = 2.0 * (x * v[1] - y * v[0]);
	[v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)]
}

/// QuatMul(a, b): a b (w x y z)
pub fn quat_mul(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
	[
		a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
		a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
		a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
		a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
	]
}

pub fn quat_conj(q: &[f64; 4]) -> [f64; 4] {
	[q[0], -q[1], -q[2], -q[3]]
}

pub fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
	[a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Norm(a): of its first three values
pub fn norm(a: &[f64]) -> f64 {
	(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// sin and cos of one argument as the C++ gets them: gcc (the Linux build) merges the two calls into glibc's sincos,
/// llvm-mingw's clang (the Windows build) calls the ucrt's sin and cos
pub fn sin_cos(x: f64) -> (f64, f64) {
	#[cfg(all(target_os = "linux", target_env = "gnu"))]
	return sally_math::libc::c_sincos(x);
	#[cfg(not(all(target_os = "linux", target_env = "gnu")))]
	return (sally_math::c_sin(x), sally_math::c_cos(x));
}

/// std::max(a, b): `(a < b) ? b : a` (a NaN b gives a)
pub fn std_max(a: f64, b: f64) -> f64 {
	if a < b {
		b
	} else {
		a
	}
}

/// std::min(a, b): `(b < a) ? b : a` (a NaN b gives a)
pub fn std_min(a: f64, b: f64) -> f64 {
	if b < a {
		b
	} else {
		a
	}
}

/// A C string in a buffer: the bytes up to the first NUL (std::string(buf))
pub fn c_str(b: &[u8]) -> &[u8] {
	&b[..b.iter().position(|&c| c == 0).unwrap_or(b.len())]
}

/// Trim(s): without the " \t\r\n" at either end
pub fn trim(s: &[u8]) -> &[u8] {
	let blank = |c: &u8| matches!(c, b' ' | b'\t' | b'\r' | b'\n');
	match (s.iter().position(|c| !blank(c)), s.iter().rposition(|c| !blank(c))) {
		(Some(a), Some(b)) => &s[a..=b],
		_ => &[],
	}
}

pub fn parse_bool(v: &[u8]) -> bool {
	v == b"1" || v == b"true" || v == b"on" || v == b"yes"
}

/// Split(s): the words between spaces, CRs and LFs (a tab is part of a word)
pub fn split(s: &[u8]) -> Vec<&[u8]> {
	let sep = |c: u8| c == b' ' || c == b'\r' || c == b'\n';
	let mut out = Vec::new();
	let mut i = 0;
	while i < s.len() {
		while i < s.len() && sep(s[i]) {
			i += 1;
		}
		let mut j = i;
		while j < s.len() && !sep(s[j]) {
			j += 1;
		}
		if j > i {
			out.push(&s[i..j]);
		}
		i = j;
	}
	out
}

/// strtok_r(buf, "\n", &save) over a datagram: its lines between LFs (none empty), the buffer read as a C string up
/// to its first NUL
pub fn lines(buf: &[u8]) -> impl Iterator<Item = &[u8]> {
	c_str(buf).split(|&c| c == b'\n').filter(|l| !l.is_empty())
}

pub const K_SEP: u8 = if cfg!(windows) { b'\\' } else { b'/' };

pub fn is_absolute(p: &[u8]) -> bool {
	!p.is_empty() && (p[0] == b'/' || p[0] == b'\\' || (p.len() > 1 && p[1] == b':'))
}

/// `s.find("{time}")` replaced until there is none, each by `stamp()` (the C++'s loops: the pose log's calls
/// TimeStamp() for each, the runtime's start one stamp for all)
pub fn expand_time(mut s: Vec<u8>, mut stamp: impl FnMut() -> Vec<u8>) -> Vec<u8> {
	while let Some(k) = s.windows(6).position(|w| w == b"{time}") {
		s.splice(k..k + 6, stamp());
	}
	s
}

/// TimeStamp(): the local time as strftime's "%Y%m%d_%H%M%S"
pub fn time_stamp() -> Vec<u8> {
	sys::time_stamp()
}

/// DllDir(): the folder of the driver's own library, "." without one
pub fn dll_dir() -> Vec<u8> {
	let p = sys::module_path();
	match p.iter().rposition(|&c| c == b'\\' || c == b'/') {
		Some(cut) => p[..cut].to_vec(),
		None => b".".to_vec(),
	}
}

#[cfg(windows)]
pub use sys::{narrow, wide_c_str, widen};

#[cfg(unix)]
mod sys {
	use std::ffi::{c_char, c_int, c_void, CStr};

	#[repr(C)]
	struct DlInfo {
		dli_fname: *const c_char,
		dli_fbase: *mut c_void,
		dli_sname: *const c_char,
		dli_saddr: *mut c_void,
	}

	/// glibc's struct tm
	#[repr(C)]
	struct Tm {
		tm_sec: c_int,
		tm_min: c_int,
		tm_hour: c_int,
		tm_mday: c_int,
		tm_mon: c_int,
		tm_year: c_int,
		tm_wday: c_int,
		tm_yday: c_int,
		tm_isdst: c_int,
		tm_gmtoff: i64,
		tm_zone: *const c_char,
	}

	extern "C" {
		fn dladdr(addr: *const c_void, info: *mut DlInfo) -> c_int;
		fn time(t: *mut i64) -> i64;
		fn localtime_r(t: *const i64, tm: *mut Tm) -> *mut Tm;
		fn strftime(s: *mut c_char, max: usize, format: *const c_char, tm: *const Tm) -> usize;
	}

	/// dladdr((void *)&DllDir, &info): the library's path as it was loaded, "." when there is none
	pub fn module_path() -> Vec<u8> {
		let mut info = DlInfo {
			dli_fname: std::ptr::null(),
			dli_fbase: std::ptr::null_mut(),
			dli_sname: std::ptr::null(),
			dli_saddr: std::ptr::null_mut(),
		};
		let me: fn() -> Vec<u8> = super::dll_dir;
		// SAFETY: dladdr only looks up the address (a function of this library) and fills the Dl_info it is given
		unsafe { dladdr(me as *const c_void, &mut info) };
		if info.dli_fname.is_null() {
			return b".".to_vec();
		}
		// SAFETY: dli_fname is the loader's NUL-terminated name of the library, which stays loaded while it runs
		unsafe { CStr::from_ptr(info.dli_fname) }.to_bytes().to_vec()
	}

	/// time, localtime_r, strftime(buf, 32, "%Y%m%d_%H%M%S")
	pub fn time_stamp() -> Vec<u8> {
		// SAFETY: plain C library calls: time with no result pointer, localtime_r and strftime on locals of the
		// right types, strftime writing at most the 32 bytes it is given
		unsafe {
			let t = time(std::ptr::null_mut());
			let mut lt: Tm = std::mem::zeroed();
			localtime_r(&t, &mut lt);
			let mut buf = [0u8; 32];
			let n = strftime(buf.as_mut_ptr().cast(), buf.len(), c"%Y%m%d_%H%M%S".as_ptr(), &lt);
			buf[..n].to_vec()
		}
	}
}

#[cfg(windows)]
mod sys {
	#![allow(non_snake_case)]

	use std::ffi::{c_char, c_int, c_void};

	const CP_UTF8: u32 = 65001;
	const GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS: u32 = 0x4;
	const GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT: u32 = 0x2;
	const MAX_PATH: usize = 260;

	/// The ucrt's struct tm
	#[repr(C)]
	struct Tm {
		tm_sec: c_int,
		tm_min: c_int,
		tm_hour: c_int,
		tm_mday: c_int,
		tm_mon: c_int,
		tm_year: c_int,
		tm_wday: c_int,
		tm_yday: c_int,
		tm_isdst: c_int,
	}

	#[link(name = "kernel32")]
	extern "system" {
		fn WideCharToMultiByte(
			cp: u32, flags: u32, w: *const u16, wn: c_int, s: *mut u8, sn: c_int, d: *const u8, used: *mut i32,
		) -> c_int;
		fn MultiByteToWideChar(cp: u32, flags: u32, s: *const u8, sn: c_int, w: *mut u16, wn: c_int) -> c_int;
		fn GetModuleHandleExW(flags: u32, name: *const u16, module: *mut *mut c_void) -> i32;
		fn GetModuleFileNameW(module: *mut c_void, name: *mut u16, size: u32) -> u32;
	}

	extern "C" {
		fn _time64(t: *mut i64) -> i64;
		fn _localtime64_s(tm: *mut Tm, t: *const i64) -> c_int;
		fn strftime(s: *mut c_char, max: usize, format: *const c_char, tm: *const Tm) -> usize;
	}

	/// Narrow(w): UTF-16 to UTF-8 by WideCharToMultiByte(CP_UTF8, 0), the string up to its first NUL
	pub fn narrow(w: &[u16]) -> Vec<u8> {
		let mut wz: Vec<u16> = w.iter().copied().take_while(|&c| c != 0).collect();
		wz.push(0);
		// SAFETY: wz is NUL-terminated (-1 asks for that), and the second call writes at most n bytes into s
		unsafe {
			let n = WideCharToMultiByte(
				CP_UTF8,
				0,
				wz.as_ptr(),
				-1,
				std::ptr::null_mut(),
				0,
				std::ptr::null(),
				std::ptr::null_mut(),
			);
			let mut s = vec![0u8; if n > 0 { n as usize } else { 1 }];
			if n > 1 {
				WideCharToMultiByte(
					CP_UTF8,
					0,
					wz.as_ptr(),
					-1,
					s.as_mut_ptr(),
					n,
					std::ptr::null(),
					std::ptr::null_mut(),
				);
			}
			s.truncate(if n > 0 { n as usize - 1 } else { 0 });
			s
		}
	}

	/// Widen(s): UTF-8 to UTF-16 by MultiByteToWideChar(CP_UTF8, 0) (what is not UTF-8 becomes U+FFFD), the string up
	/// to its first NUL (a std::wstring: no terminator; `wide_c_str` adds one)
	pub fn widen(s: &[u8]) -> Vec<u16> {
		let mut sz: Vec<u8> = super::c_str(s).to_vec();
		sz.push(0);
		// SAFETY: sz is NUL-terminated (-1 asks for that), and the second call writes at most n chars into w
		unsafe {
			let n = MultiByteToWideChar(CP_UTF8, 0, sz.as_ptr(), -1, std::ptr::null_mut(), 0);
			let mut w = vec![0u16; if n > 0 { n as usize } else { 1 }];
			if n > 1 {
				MultiByteToWideChar(CP_UTF8, 0, sz.as_ptr(), -1, w.as_mut_ptr(), n);
			}
			w.truncate(if n > 0 { n as usize - 1 } else { 0 });
			w
		}
	}

	/// A wide string's c_str(): its chars and a NUL
	pub fn wide_c_str(w: &[u16]) -> Vec<u16> {
		let mut z = w.to_vec();
		z.push(0);
		z
	}

	/// GetModuleHandleExW(FROM_ADDRESS | UNCHANGED_REFCOUNT, &DllDir) and GetModuleFileNameW(MAX_PATH * 2), narrowed
	pub fn module_path() -> Vec<u8> {
		let mut me: *mut c_void = std::ptr::null_mut();
		let f: fn() -> Vec<u8> = super::dll_dir;
		let mut path = [0u16; MAX_PATH * 2];
		// SAFETY: the address is a function of this library (the flags take it for the module's address, and keep its
		// count); GetModuleFileNameW writes at most the buffer's length of chars
		unsafe {
			GetModuleHandleExW(
				GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
				f as *const u16,
				&mut me,
			);
			GetModuleFileNameW(me, path.as_mut_ptr(), path.len() as u32);
		}
		narrow(&path)
	}

	/// _time64, _localtime64_s (localtime_s), strftime(buf, 32, "%Y%m%d_%H%M%S")
	pub fn time_stamp() -> Vec<u8> {
		// SAFETY: plain C library calls on locals of the right types, strftime writing at most the 32 bytes it is
		// given
		unsafe {
			let t = _time64(std::ptr::null_mut());
			let mut lt: Tm = std::mem::zeroed();
			_localtime64_s(&mut lt, &t);
			let mut buf = [0u8; 32];
			let n = strftime(buf.as_mut_ptr().cast(), buf.len(), c"%Y%m%d_%H%M%S".as_ptr(), &lt);
			buf[..n].to_vec()
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn strings_as_the_cpp_cuts_them() {
		assert_eq!(trim(b" \t a b \r\n"), b"a b");
		assert_eq!(trim(b" \t\r\n"), b"");
		assert_eq!(split(b"  #d DEV\tx  y\r\n"), vec![&b"#d"[..], b"DEV\tx", b"y"]);
		assert!(split(b"\r\n ").is_empty());
		assert_eq!(lines(b"\n\nQ a\nP b\n\0Q c\n").collect::<Vec<_>>(), vec![&b"Q a"[..], b"P b"]);
		assert!(parse_bool(b"on") && !parse_bool(b"On") && !parse_bool(b"0"));
		assert!(is_absolute(b"/a") && is_absolute(b"\\a") && is_absolute(b"C:x") && !is_absolute(b"a/b"));
		let mut n = 0;
		let s = expand_time(b"a{time}b{time}".to_vec(), || {
			n += 1;
			format!("<{n}>").into_bytes()
		});
		assert_eq!(s, b"a<1>b<2>");
		assert_eq!(time_stamp().len(), 15);
	}

	#[test]
	fn std_min_max_on_nan_and_zeros() {
		assert_eq!(std_max(-0.05, std_min(0.05, f64::NAN)), 0.05);
		assert_eq!(std_min(0.2, f64::NAN), 0.2);
		assert!(std_max(f64::NAN, 1.0).is_nan());
		assert!(std_max(0.0, -0.0).is_sign_positive() && std_max(-0.0, 0.0).is_sign_negative());
	}
}
