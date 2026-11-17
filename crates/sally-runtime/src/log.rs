//! The log as the C writes it (survive.c's default info, warn and error, and the printf hook): "Info: " and
//! "Warning: " lines to stdout or the --log file, a warning in red where the C colours it (everywhere but Windows, in
//! the file too), and names coloured by survive_colorize. The C writes through a text-mode stream, so on Windows each
//! newline goes out as CR LF; so does this.
//!
//! The log also keeps survive.c's start (OGStartTimeS), the zero of the run clock's wall time (timestamp_in_s): the
//! first of survive.c's timed calls takes it, which is the first line its default info, warn or error writes (each
//! times its call of the printf hook) unless the run clock is read before.

use std::io::Write;

use sally_os::StartTime;

/// SurviveLogLevel: the level a log line is handed to the log hook with (SV_WARN's warning, SV_INFO's and
/// SV_ERROR's info; nothing in the C logs at the error level)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
	Error = 0,
	Warning = 1,
	Info = 2,
}

/// A log function in the default's place (survive_install_log_fn, survive_init_with_logger's log_func): every line
/// SV_INFO, SV_WARN and SV_ERROR log goes to it with its level, and the default's work (the log's text, the
/// recording's INFO LOG line) is not done
pub type LogFn = Box<dyn FnMut(LogLevel, &str) + Send>;

/// ctx->log_target and the colouring, and survive.c's start
pub struct Log {
	target: Option<Box<dyn Write + Send>>,
	/// survive_colorize's: off when logging to a file (--log) or recording to stdout, and always on Windows
	colorize: bool,
	/// survive.c's OGStartTimeS
	start: StartTime,
}

impl Log {
	/// The log to `target` (None: nowhere); `colorize` as survive_init decides it
	pub fn new(target: Option<Box<dyn Write + Send>>, colorize: bool) -> Self {
		Log { target, colorize: colorize && !cfg!(windows), start: StartTime::new() }
	}

	/// survive.c's OGStartTimeS: the wall clock as survive.c first timed a call
	pub fn start_time(&self) -> f64 {
		self.start.get()
	}

	/// SURVIVE_INVOKE_HOOK(printf, ...) in survive_default_info, _warn and _error, after their check of the log target:
	/// the hook's call is timed (OGRelativeTime), which takes survive.c's start if nothing has yet
	fn time_printf_hook(&self) {
		if self.target.is_some() {
			self.start.get();
		}
	}

	/// The text as a text-mode stream writes it
	fn put(&mut self, text: &str) {
		let Some(t) = self.target.as_mut() else { return };
		if cfg!(windows) {
			let _ = t.write_all(text.replace('\n', "\r\n").as_bytes());
		} else {
			let _ = t.write_all(text.as_bytes());
		}
	}

	fn flush(&mut self) {
		if let Some(t) = self.target.as_mut() {
			let _ = t.flush();
		}
	}

	/// survive_default_info's text: "Info: %s\n", flushed
	pub fn info(&mut self, msg: &str) {
		self.time_printf_hook();
		self.put(&format!("Info: {msg}\n"));
		self.flush();
	}

	/// survive_default_warn's text: "Warning: %s\n", in red where the C writes the colour codes, flushed
	pub fn warn(&mut self, msg: &str) {
		self.time_printf_hook();
		if !cfg!(windows) {
			self.put("\x1b[0;31m");
		}
		self.put(&format!("Warning: {msg}\n"));
		if !cfg!(windows) {
			self.put("\x1b[0m");
		}
		self.flush();
	}

	/// survive_default_error's text: "Error %d: %s" in red, then a newline (SV_ERROR itself logs at the info level:
	/// this is for an error the error hook reports)
	pub fn error(&mut self, code: i32, msg: &str) {
		self.time_printf_hook();
		if !cfg!(windows) {
			self.put("\x1b[0;31m");
		}
		self.put(&format!("Error {code}: {msg}"));
		if !cfg!(windows) {
			self.put("\x1b[0m");
		}
		self.put("\n");
		self.flush();
	}

	/// survive_default_printf_process: raw text (not flushed: the next info or warning flushes it)
	pub fn print(&mut self, text: &str) {
		self.put(text);
	}

	/// At shutdown
	pub fn close(&mut self) {
		self.flush();
	}

	/// Whether survive_colorize colours names
	pub fn colorizes(&self) -> bool {
		self.colorize
	}

	/// survive_colorize: a name in one of eight colours by its hash ("\033[0;%dm%s\033[0m"), or plain where the C
	/// leaves it plain (and for a name too long for the C's 128-byte buffer)
	pub fn colorize(&self, s: &str) -> String {
		if !self.colorize || s.len() > 128 - 18 - 1 {
			return s.to_string();
		}
		format!("\x1b[0;{}m{}\x1b[0m", survive_hash(s.as_bytes()) % 8 + 30, s)
	}
}

/// survive_hash: djb2 (hash * 33 + c), in 32 bits
pub fn survive_hash(data: &[u8]) -> u32 {
	let mut hash: u32 = 5381;
	for &c in data {
		hash = (hash << 5).wrapping_add(hash).wrapping_add(c as u32);
	}
	hash
}

/// SV_INFO formats into a 1024-byte buffer (snprintf: the text is cut at 1023 bytes); SV_WARN and SV_ERROR use sprintf
/// into the same size, which a longer text would overrun (none is that long)
pub fn info_text(msg: &str) -> &str {
	if msg.len() <= 1023 {
		return msg;
	}
	let mut end = 1023;
	while !msg.is_char_boundary(end) {
		end -= 1;
	}
	&msg[..end]
}

#[cfg(test)]
mod tests {
	use super::*;

	/// The colours a C replay's log shows for these names
	#[test]
	fn names_get_the_c_colours() {
		if cfg!(windows) {
			return;
		}
		let log = Log::new(None, true);
		for (name, colour) in
			[("KN0", 36), ("KN1", 37), ("KN2", 30), ("KN3", 31), ("KN4", 32), ("KN5", 33), ("HID", 32)]
		{
			assert_eq!(log.colorize(name), format!("\x1b[0;{colour}m{name}\x1b[0m"));
		}
		assert_eq!(Log::new(None, false).colorize("KN0"), "KN0");
	}
}
