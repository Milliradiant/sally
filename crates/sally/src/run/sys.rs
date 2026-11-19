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

/// sally: the process a run was started for (--stop-with: the SteamVR server, whose driver started it), watched so
/// that the run ends with it
pub struct Owner(imp::Owner);

impl Owner {
	/// The process with this id, watched (0: none is). One that is named but cannot be watched is not, and the text
	/// says why: the run then goes on as one started by hand does
	pub fn watch(pid: u32) -> (Owner, Option<String>) {
		let (owner, why) = imp::Owner::watch(pid);
		(Owner(owner), why)
	}

	/// Whether the process has ended (never, with none watched)
	pub fn gone(&self) -> bool {
		self.0.gone()
	}
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

	type Handle = *mut std::ffi::c_void;
	const SYNCHRONIZE: u32 = 0x0010_0000;
	const WAIT_OBJECT_0: u32 = 0;
	const ERROR_INVALID_PARAMETER: u32 = 87;

	#[link(name = "kernel32")]
	extern "system" {
		fn OpenProcess(access: u32, inherit: i32, pid: u32) -> Handle;
		fn WaitForSingleObject(handle: Handle, milliseconds: u32) -> u32;
		fn CloseHandle(handle: Handle) -> i32;
		fn GetLastError() -> u32;
	}

	/// The process's handle, held from the start: the id of a process that has ended can be given to another, a
	/// handle stays that process's. Null: none is watched, or the process had ended before it could be opened
	pub struct Owner {
		handle: Handle,
		gone: bool,
	}

	impl Owner {
		pub fn watch(pid: u32) -> (Owner, Option<String>) {
			let none = Owner { handle: std::ptr::null_mut(), gone: false };
			if pid == 0 {
				return (none, None);
			}
			// SAFETY: OpenProcess takes an access mask, a flag and a process id; GetLastError reads this thread's
			// last error
			let (handle, error) = unsafe { (OpenProcess(SYNCHRONIZE, 0, pid), GetLastError()) };
			if !handle.is_null() {
				(Owner { handle, gone: false }, None)
			} else if error == ERROR_INVALID_PARAMETER {
				// no process has that id: it ended before this one got to look
				(Owner { handle, gone: true }, None)
			} else {
				(none, Some(format!("process {pid} cannot be opened (error {error})")))
			}
		}

		pub fn gone(&self) -> bool {
			// SAFETY: the handle is this process's own, open until the drop; a wait of no time only asks
			self.gone || (!self.handle.is_null() && unsafe { WaitForSingleObject(self.handle, 0) } == WAIT_OBJECT_0)
		}
	}

	impl Drop for Owner {
		fn drop(&mut self) {
			if !self.handle.is_null() {
				// SAFETY: the handle was opened by watch and is closed once, here
				unsafe { CloseHandle(self.handle) };
			}
		}
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

	const ESRCH: i32 = 3;

	extern "C" {
		/// kill(pid, 0) sends nothing and says whether the process is there to be sent to
		fn kill(pid: i32, signal: c_int) -> c_int;
	}

	/// The process's id (0: none is watched)
	pub struct Owner(i32);

	impl Owner {
		pub fn watch(pid: u32) -> (Owner, Option<String>) {
			match i32::try_from(pid) {
				Ok(pid) => (Owner(pid), None),
				Err(_) => (Owner(0), Some(format!("{pid} is not a process id"))),
			}
		}

		pub fn gone(&self) -> bool {
			// SAFETY: signal 0 is sent to no one: the call only looks the process up
			self.0 > 0
				&& unsafe { kill(self.0, 0) } != 0
				&& std::io::Error::last_os_error().raw_os_error() == Some(ESRCH)
		}
	}
}

#[cfg(not(any(windows, target_os = "linux")))]
mod imp {
	pub fn catch_stops() {}

	/// No process is watched here
	pub struct Owner;

	impl Owner {
		pub fn watch(_pid: u32) -> (Owner, Option<String>) {
			(Owner, None)
		}

		pub fn gone(&self) -> bool {
			false
		}
	}
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
	use super::*;

	/// A run started for a process ends with it: the owner is gone once that process has ended, and never where
	/// none was named
	#[test]
	fn the_owner_is_gone_once_its_process_has_ended() {
		let mut child = std::process::Command::new("sleep").arg("60").spawn().unwrap();
		let (owner, why) = Owner::watch(child.id());
		assert!(why.is_none() && !owner.gone());
		child.kill().unwrap();
		child.wait().unwrap();
		assert!(owner.gone());
		assert!(!Owner::watch(0).0.gone());
	}
}
