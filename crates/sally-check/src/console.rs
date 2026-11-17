//! The console sally-check runs in: on Windows its VT mode (for the live view that redraws in place), UTF-8, the title,
//! Ctrl+C as "stop" rather than "kill", whether it was opened just for us (double-clicked: then it waits before it
//! closes), the local time, and opening files the way Explorer does.

// recording is Windows only: elsewhere only the check of a recording runs, without the stop flag or the time stamp
#![cfg_attr(not(windows), allow(dead_code))]

use std::sync::atomic::{AtomicBool, Ordering};

/// Set by Ctrl+C, Enter, or the window closing
pub static STOP: AtomicBool = AtomicBool::new(false);

pub fn stop_requested() -> bool {
	STOP.load(Ordering::SeqCst)
}

/// The local date and time as YYYYMMDD_HHMMSS, and as people read it
pub fn local_stamp() -> (String, String) {
	let t = local_time();
	(
		format!("{:04}{:02}{:02}_{:02}{:02}{:02}", t[0], t[1], t[2], t[3], t[4], t[5]),
		format!("{:04}-{:02}-{:02} {:02}:{:02}", t[0], t[1], t[2], t[3], t[4]),
	)
}

#[cfg(windows)]
mod sys {
	use std::ffi::c_void;
	use std::sync::atomic::Ordering;

	#[repr(C)]
	#[derive(Default)]
	struct SystemTime {
		year: u16,
		month: u16,
		day_of_week: u16,
		day: u16,
		hour: u16,
		minute: u16,
		second: u16,
		ms: u16,
	}

	#[link(name = "kernel32")]
	extern "system" {
		fn GetStdHandle(n: u32) -> *mut c_void;
		fn GetConsoleMode(h: *mut c_void, mode: *mut u32) -> i32;
		fn SetConsoleMode(h: *mut c_void, mode: u32) -> i32;
		fn SetConsoleOutputCP(cp: u32) -> i32;
		fn SetConsoleTitleW(title: *const u16) -> i32;
		fn SetConsoleCtrlHandler(handler: Option<unsafe extern "system" fn(u32) -> i32>, add: i32) -> i32;
		fn GetConsoleProcessList(list: *mut u32, count: u32) -> u32;
		fn GetLocalTime(t: *mut SystemTime);
	}

	const STD_OUTPUT_HANDLE: u32 = -11i32 as u32;
	const ENABLE_VIRTUAL_TERMINAL_PROCESSING: u32 = 0x4;

	unsafe extern "system" fn on_ctrl(_kind: u32) -> i32 {
		super::STOP.store(true, Ordering::SeqCst);
		// handled: Ctrl+C stops the recording instead of ending the program (closing the window still ends it, a few
		// seconds later)
		1
	}

	/// VT mode, UTF-8, the title and the Ctrl+C handler; true when the console can redraw in place
	pub fn setup(title: &str) -> bool {
		let w: Vec<u16> = title.encode_utf16().chain(Some(0)).collect();
		unsafe {
			SetConsoleOutputCP(65001);
			SetConsoleTitleW(w.as_ptr());
			SetConsoleCtrlHandler(Some(on_ctrl), 1);
			let h = GetStdHandle(STD_OUTPUT_HANDLE);
			let mut mode = 0;
			GetConsoleMode(h, &mut mode) != 0 && SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != 0
		}
	}

	/// Whether this console is ours alone (sally-check was double-clicked, not run from a prompt)
	pub fn own_window() -> bool {
		let mut list = [0u32; 4];
		unsafe { GetConsoleProcessList(list.as_mut_ptr(), 4) <= 1 }
	}

	pub fn local_time() -> [u16; 6] {
		let mut t = SystemTime::default();
		unsafe { GetLocalTime(&mut t) };
		[t.year, t.month, t.day, t.hour, t.minute, t.second]
	}

	/// A file opened with its usual program, or a folder's file shown selected in Explorer
	pub fn open(path: &std::path::Path, select: bool) {
		let mut c = std::process::Command::new("explorer.exe");
		if select {
			c.arg(format!("/select,{}", path.display()));
		} else {
			c.arg(path);
		}
		let _ = c.spawn();
	}
}

#[cfg(not(windows))]
mod sys {
	pub fn setup(_title: &str) -> bool {
		false
	}
	pub fn own_window() -> bool {
		false
	}
	pub fn local_time() -> [u16; 6] {
		let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
		let days = (secs / 86_400) as i64;
		let rem = secs % 86_400;
		let z = days + 719_468;
		let era = z.div_euclid(146_097);
		let doe = z - era * 146_097;
		let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
		let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
		let mp = (5 * doy + 2) / 153;
		let d = doy - (153 * mp + 2) / 5 + 1;
		let m = if mp < 10 { mp + 3 } else { mp - 9 };
		let y = yoe + era * 400 + if m <= 2 { 1 } else { 0 };
		[y as u16, m as u16, d as u16, (rem / 3600) as u16, ((rem % 3600) / 60) as u16, (rem % 60) as u16]
	}
	pub fn open(_path: &std::path::Path, _select: bool) {}
}

use sys::local_time;
pub use sys::{open, own_window, setup};

/// Waits for Enter (or Ctrl+C)
pub fn wait_enter() {
	let mut s = String::new();
	let _ = std::io::stdin().read_line(&mut s);
}

/// Enter sets STOP, on a thread of its own
pub fn stop_on_enter() {
	let _ = std::thread::Builder::new().name("enter".into()).spawn(|| {
		let mut s = String::new();
		let _ = std::io::stdin().read_line(&mut s);
		STOP.store(true, Ordering::SeqCst);
	});
}
