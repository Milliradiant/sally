//! The tracker's text: each line is a printf format string and its arguments in the C types the format reads them as,
//! formatted by sally_math's printf (glibc's text to the byte, which the recordings and logs are compared by). The
//! types are kept so a line cannot be given an argument its format would read as another type.

use std::ffi::CStr;

use sally_math::printf::{sprintf, Arg};

mod sealed {
	pub trait Sealed {}
}

/// A value a format takes as it is: double, int, unsigned, unsigned long long, a C string, a device's name
pub trait CArg: Copy + sealed::Sealed {
	fn arg(&self) -> Arg<'_>;
}
impl sealed::Sealed for f64 {}
impl CArg for f64 {
	fn arg(&self) -> Arg<'_> {
		Arg::F(*self)
	}
}
impl sealed::Sealed for i32 {}
impl CArg for i32 {
	fn arg(&self) -> Arg<'_> {
		Arg::I(*self as i64)
	}
}
impl sealed::Sealed for u32 {}
impl CArg for u32 {
	fn arg(&self) -> Arg<'_> {
		Arg::U(*self as u64)
	}
}
/// A u64, printed with %llu
impl sealed::Sealed for u64 {}
impl CArg for u64 {
	fn arg(&self) -> Arg<'_> {
		Arg::U(*self)
	}
}
impl sealed::Sealed for &'static CStr {}
impl CArg for &'static CStr {
	fn arg(&self) -> Arg<'_> {
		Arg::S(self.to_str().unwrap_or(""))
	}
}

/// A device's codename, for a %s: held by value, so a line can name the device while the host it came from is borrowed
/// for the line
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Name {
	bytes: [u8; 16],
	len: u8,
}

impl Name {
	/// The name, cut at 16 bytes (a codename has 3)
	pub fn new(s: &str) -> Name {
		let mut bytes = [0u8; 16];
		let mut len = s.len().min(16);
		while !s.is_char_boundary(len) {
			len -= 1;
		}
		bytes[..len].copy_from_slice(&s.as_bytes()[..len]);
		Name { bytes, len: len as u8 }
	}

	pub fn as_str(&self) -> &str {
		std::str::from_utf8(&self.bytes[..self.len as usize]).unwrap_or("")
	}
}

impl sealed::Sealed for Name {}
impl CArg for Name {
	fn arg(&self) -> Arg<'_> {
		Arg::S(self.as_str())
	}
}

/// A line's arguments: a tuple of CArgs
pub trait CArgs: Copy {
	/// The text of `fmt` with these arguments
	fn format(&self, fmt: &CStr) -> String;
}

macro_rules! cargs_tuple {
	($($t:ident $v:ident),+) => {
		impl<$($t: CArg),+> CArgs for ($($t,)+) {
			fn format(&self, fmt: &CStr) -> String {
				let ($($v,)+) = self;
				sprintf(fmt.to_str().unwrap_or(""), &[$($v.arg()),+])
			}
		}
	};
}
cargs_tuple!(A a);
cargs_tuple!(A a, B b);
cargs_tuple!(A a, B b, C c);
cargs_tuple!(A a, B b, C c, D d);
cargs_tuple!(A a, B b, C c, D d, E e);
cargs_tuple!(A a, B b, C c, D d, E e, F f);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t, U u);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t, U u, V v);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t, U u, V v, W w);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t, U u, V v, W w, X x);
cargs_tuple!(A a, B b, C c, D d, E e, F f, G g, H h, I i, J j, K k, L l, M m, N n, O o, P p, Q q, R r, S s, T t, U u, V v, W w, X x, Y y);

/// SV_INFO's buffer: snprintf(stbuff, 1024, ...) keeps 1023 bytes
pub const SV_LOG_MAX: usize = 1023;

/// A log line as SV_INFO leaves it (cut at SV_LOG_MAX bytes)
pub fn log_text(mut text: String) -> String {
	if text.len() > SV_LOG_MAX {
		let mut end = SV_LOG_MAX;
		while !text.is_char_boundary(end) {
			end -= 1;
		}
		text.truncate(end);
	}
	text
}
