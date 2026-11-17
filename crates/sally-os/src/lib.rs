//! The operating system as os_generic.h gave it to the C runtime: its clock (OGGetAbsoluteTime,
//! OGGetAbsoluteTimeUS, OGGetAbsoluteTimeMS, and OGStartTimeS / OGRelativeTime over it) and its sleep (OGUSleep), each
//! as the platform's half of the header has it (os_generic.unix.h, os_generic.windows.h), its integer arithmetic
//! included. The two halves differ: the clock is gettimeofday's wall clock on Linux and the performance counter on
//! Windows, and the sleep is one nanosleep, to the microsecond, on Linux and Sleep, to the millisecond and the
//! system's timer tick, on Windows (Rust's own sleep waits on a high-resolution timer there, so the runtime's cadence
//! would not be the C's).
//!
//! A thread the C makes with OGCreateThread gets the header's stack: `create_thread`.
//!
//! The SteamVR driver and the panel, C++ in origin, keep the C++ standard library's clocks and sleep (std::chrono's
//! steady_clock and system_clock, this_thread::sleep_for): `chrono`. The panel's clock times are the C library's local
//! time: `localtime`.
//!
//! Not here: the header's mutexes, semaphores and condition variables (the ports use std's), and OGSleep and
//! OGGetFileTime (nothing ported calls them).

use std::sync::OnceLock;
use std::thread::JoinHandle;

pub mod chrono;
pub mod localtime;
mod sys;

/// OGCreateThread's stack: 2000 * 4096 bytes on unix (glibc's default, as its comment says), 8 MB on Windows (sally's:
/// the solvers overflow the 1 MB Windows default)
pub const THREAD_STACK: usize = if cfg!(windows) { 8 * 1024 * 1024 } else { 2000 * 4096 };

/// OGCreateThread(routine, name, parameter): a thread of that name with the header's stack; None when it cannot be
/// made (the C's handle is then 0, and joining it does nothing)
pub fn create_thread<T: Send + 'static>(name: &str, f: impl FnOnce() -> T + Send + 'static) -> Option<JoinHandle<T>> {
	std::thread::Builder::new().name(name.into()).stack_size(THREAD_STACK).spawn(f).ok()
}

/// OGGetAbsoluteTime: the clock in seconds. Linux: gettimeofday, tv_usec / 1e6 + tv_sec (the epoch's seconds); Windows:
/// QueryPerformanceCounter / QueryPerformanceFrequency (seconds since the machine started)
pub fn absolute_time() -> f64 {
	sys::absolute_time()
}

/// OGGetAbsoluteTimeUS: the clock in whole microseconds. Linux: tv_sec * 1000000 + tv_usec; Windows: counter * 1000 *
/// 1000 / frequency, the product in 128 bits (sally: the C's signed 64-bit product wraps once the machine has been up
/// for 2^63 / 10^6 counts, 10.7 days at the usual 10 MHz; the same value until then)
pub fn absolute_time_us() -> u64 {
	sys::absolute_time_us()
}

/// OGGetAbsoluteTimeMS: the clock in milliseconds, rounded up: (OGGetAbsoluteTimeUS() + 999) / 1000
pub fn absolute_time_ms() -> u64 {
	absolute_time_us().wrapping_add(999) / 1000
}

/// OGUSleep(ius): Linux: one nanosleep of ius microseconds (tv_sec = ius / 1000000, tv_nsec = (ius % 1000000) * 1000;
/// a signal ends it early and a negative time fails at once, as in the C), its result; Windows: Sleep(ius / 1000), so
/// whole milliseconds on the system's timer tick (a negative time is a DWORD of 49 days), and 0
pub fn usleep(ius: i32) -> i32 {
	sys::usleep(ius)
}

/// OGStartTimeS and OGRelativeTime of one C translation unit: the header defines them static inline, so each C file
/// that uses them has a start of its own, the clock's reading at the file's first timed call (a log line it writes
/// through SV_INFO times its hook call, and so does every hook it invokes)
pub struct StartTime(OnceLock<f64>);

impl StartTime {
	/// The start, not taken yet (static double start_time_s = 0)
	pub const fn new() -> StartTime {
		StartTime(OnceLock::new())
	}

	/// OGStartTimeS: the clock as this was first called
	pub fn get(&self) -> f64 {
		*self.0.get_or_init(absolute_time)
	}

	/// OGRelativeTime: OGGetAbsoluteTime() - OGStartTimeS()
	pub fn relative_time(&self) -> f64 {
		absolute_time() - self.get()
	}
}

impl Default for StartTime {
	fn default() -> StartTime {
		StartTime::new()
	}
}

impl std::fmt::Debug for StartTime {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "StartTime({:?})", self.0.get())
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_clocks_agree() {
		let us = absolute_time_us();
		let s = absolute_time();
		let ms = absolute_time_ms();
		// the same clock in three units, read one after the other
		assert!((s - us as f64 * 1e-6).abs() < 0.5, "{s} {us}");
		assert!(ms >= us / 1000 && ms - us / 1000 < 1000, "{ms} {us}");
	}

	#[test]
	fn a_start_is_taken_once() {
		let t = StartTime::new();
		let first = t.get();
		assert!(t.relative_time() >= 0.0);
		assert_eq!(t.get(), first);
	}

	#[test]
	fn usleep_sleeps() {
		let t0 = absolute_time();
		assert_eq!(usleep(20_000), 0);
		let slept = absolute_time() - t0;
		assert!(slept >= 0.019, "{slept}");
		// a negative time: Linux's nanosleep refuses it at once (EINVAL)
		#[cfg(unix)]
		assert_eq!(usleep(-1000), -1);
	}
}
