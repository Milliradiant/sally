//! The C's text into fixed buffers: snprintf cuts a line to its buffer, SV_INFO to 1023 bytes, and SV_WARN's sprintf
//! would write past its 1024 bytes (the port stops there instead), each formatted by sally_math's printf.

use sally_math::printf::{sprintf, Arg};

use crate::host::Host;

/// snprintf into a buffer of `size` bytes: the text cut to size - 1 bytes (at a character boundary: the C would cut a
/// multi-byte character, which only a file name could hold)
pub fn cut(mut s: String, size: usize) -> String {
	if s.len() > size - 1 {
		let mut end = size - 1;
		while !s.is_char_boundary(end) {
			end -= 1;
		}
		s.truncate(end);
	}
	s
}

/// SV_INFO(fmt, ...): snprintf into the log's 1024 bytes, then the log hook
pub fn info(host: &mut (impl Host + ?Sized), fmt: &str, args: &[Arg]) {
	host.info(&cut(sprintf(fmt, args), 1024));
}

/// SV_WARN(fmt, ...): sprintf into the log's 1024 bytes, then the log hook. A longer text would overrun the C's
/// buffer: the port stops
pub fn warn(host: &mut (impl Host + ?Sized), fmt: &str, args: &[Arg]) {
	let msg = sprintf(fmt, args);
	assert!(msg.len() < 1024, "SV_WARN: {} bytes overrun the C's 1024-byte buffer", msg.len());
	host.warn(&msg);
}
