//! The C library's stdio for the driver's two files, opened as the C++ opens them: sally_settings.txt read with fgets
//! into a 4096-byte buffer (a longer line comes in pieces, a NUL ends what a line says), and the pose log written into
//! the stream's buffer, flushed by fflush at most once a second, and never closed (the C library writes out what is
//! left when the process exits, as for the C++'s). Both are binary ("rb", "wb"): no CR LF in either direction on
//! Windows. The same FILE functions the C++ calls (glibc's, the ucrt's), so a file opens, fails and fills as its does.

use std::ffi::{c_char, c_int, c_void, CStr};

/// A FILE *
pub struct CFile(*mut c_void);

// SAFETY: a FILE is the C library's, locked by it on every call, and this handle is used by one thread at a time
unsafe impl Send for CFile {}

#[cfg(unix)]
extern "C" {
	fn fopen(path: *const c_char, mode: *const c_char) -> *mut c_void;
}
#[cfg(windows)]
extern "C" {
	fn _wfopen(path: *const u16, mode: *const u16) -> *mut c_void;
}
extern "C" {
	fn fgets(s: *mut c_char, n: c_int, f: *mut c_void) -> *mut c_char;
	fn fwrite(p: *const c_void, size: usize, n: usize, f: *mut c_void) -> usize;
	fn fputc(c: c_int, f: *mut c_void) -> c_int;
	fn fflush(f: *mut c_void) -> c_int;
	fn fclose(f: *mut c_void) -> c_int;
}

impl CFile {
	/// fopen(path.c_str(), mode) on Linux, _wfopen(Widen(path).c_str(), L"mode") on Windows; None where it fails
	pub fn open(path: &[u8], mode: &CStr) -> Option<CFile> {
		#[cfg(unix)]
		let f = {
			let mut p = crate::helpers::c_str(path).to_vec();
			p.push(0);
			// SAFETY: both are NUL-terminated strings
			unsafe { fopen(p.as_ptr().cast(), mode.as_ptr()) }
		};
		#[cfg(windows)]
		let f = {
			let p = crate::helpers::wide_c_str(&crate::helpers::widen(path));
			let m: Vec<u16> = mode.to_bytes().iter().map(|&c| c as u16).chain([0]).collect();
			// SAFETY: both are NUL-terminated wide strings
			unsafe { _wfopen(p.as_ptr(), m.as_ptr()) }
		};
		if f.is_null() {
			None
		} else {
			Some(CFile(f))
		}
	}

	/// fgets(buf, sizeof buf, f): the C string it read (up to its first NUL), None at the end
	pub fn gets<'a>(&mut self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
		// SAFETY: fgets writes at most buf.len() bytes, the last a NUL, into buf
		let r = unsafe { fgets(buf.as_mut_ptr().cast(), buf.len() as c_int, self.0) };
		if r.is_null() {
			None
		} else {
			Some(crate::helpers::c_str(buf))
		}
	}

	/// fprintf(f, "%s", text): the text into the stream
	pub fn write(&mut self, text: &[u8]) {
		// SAFETY: fwrite reads text.len() bytes of text
		unsafe { fwrite(text.as_ptr().cast(), 1, text.len(), self.0) };
	}

	/// fputc(c, f)
	pub fn putc(&mut self, c: u8) {
		// SAFETY: a byte into the stream
		unsafe { fputc(c as c_int, self.0) };
	}

	/// fflush(f)
	pub fn flush(&mut self) {
		// SAFETY: the stream is open
		unsafe { fflush(self.0) };
	}

	/// fclose(f)
	pub fn close(self) {
		// SAFETY: the stream is open, and this consumes the only handle to it
		unsafe { fclose(self.0) };
	}
}
