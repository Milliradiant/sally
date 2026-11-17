//! survive-cli.c's stops, on `run`'s KEEP_RUNNING. Windows: its console handler (Ctrl+C, Ctrl+Break, the console
//! closing) clears keepRunning, then sleeps 3 s to give the main loop time to close and flush the recording, and
//! returns TRUE (handled: the process goes on, or, for the console closing, ends once the handler returns). Linux:
//! SIGINT and SIGTERM clear keepRunning, and a second one exits at once with -1. The C asks for SIGKILL too, which
//! cannot be caught (signal() refuses it); that is not asked here. Declared by hand, the workspace taking no crate for
//! them.

use std::sync::atomic::Ordering;

use super::KEEP_RUNNING;

/// The console handler (Windows) or the signal handlers (Linux); elsewhere none, as in the C
pub fn catch_stops() {
	imp::catch_stops();
}

/// keepRunning cleared; whether it was set
fn stop() -> bool {
	KEEP_RUNNING.swap(false, Ordering::SeqCst)
}

#[cfg(windows)]
mod imp {
	#[link(name = "kernel32")]
	extern "system" {
		// as `sally ctl` declares it (ctl.rs)
		fn SetConsoleCtrlHandler(handler: extern "system" fn(u32) -> i32, add: i32) -> i32;
		fn Sleep(milliseconds: u32);
	}

	/// consoleHandler: keepRunning = false; Sleep(3000); return TRUE
	extern "system" fn console_handler(_ctrl_type: u32) -> i32 {
		super::stop();
		// SAFETY: Sleep takes a number of milliseconds and nothing else
		unsafe { Sleep(3000) };
		1
	}

	pub fn catch_stops() {
		// SAFETY: the handler is a function of the signature the call asks for, and lives as long as the program
		unsafe { SetConsoleCtrlHandler(console_handler, 1) };
	}
}

#[cfg(target_os = "linux")]
mod imp {
	use std::ffi::c_int;

	const SIGINT: c_int = 2;
	const SIGTERM: c_int = 15;

	extern "C" {
		/// glibc's signal(): the handler stays installed, with SA_RESTART (BSD semantics); it returns the old handler, a
		/// pointer (sighandler_t)
		fn signal(signum: c_int, handler: extern "C" fn(c_int)) -> usize;
		fn _exit(status: c_int) -> !;
	}

	/// intHandler: exit(-1) if keepRunning is already 0, else keepRunning = 0. The flag is swapped, not read and then
	/// written, so a SIGTERM landing inside a SIGINT's handler counts as the second. The exit is _exit, which a signal
	/// handler may call: the C's exit() also writes out the C's stdio buffers (the --hidtap-record file's last lines),
	/// which no file of the Rust's goes through
	extern "C" fn int_handler(_signal: c_int) {
		if !super::stop() {
			// SAFETY: _exit ends the process at once and may be called from a signal handler
			unsafe { _exit(-1) };
		}
	}

	pub fn catch_stops() {
		// SAFETY: the handler is an extern "C" fn(int), which signal() installs for each of the two signals
		unsafe {
			signal(SIGINT, int_handler);
			signal(SIGTERM, int_handler);
		}
	}
}

#[cfg(not(any(windows, target_os = "linux")))]
mod imp {
	pub fn catch_stops() {}
}
