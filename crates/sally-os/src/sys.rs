//! The two halves of os_generic.h's clock and sleep, each with the system calls its C makes (declared by hand: the
//! workspace takes no crate for them).

#[cfg(unix)]
pub use unix::*;
#[cfg(windows)]
pub use windows::*;

/// Windows' OGGetAbsoluteTimeUS from the counter and its frequency: counter * 1000 * 1000 / frequency. sally: the
/// product in 128 bits. The C's LONGLONG wraps once the counter passes 2^63 / 10^6 (10.7 days of uptime at the usual
/// 10 MHz), and from there its clock jumped by about 2^64 microseconds, forward and back by turns, every 10.7 days (a
/// jump back during a long poll could make survive_poll's wait one of 49 days). The same value until then
#[cfg_attr(not(windows), allow(dead_code))]
fn us_of(counter: i64, frequency: i64) -> u64 {
	(counter as i128 * 1_000_000 / frequency as i128) as u64
}

/// os_generic.unix.h
#[cfg(unix)]
mod unix {
	use std::ffi::{c_int, c_long};
	use std::time::{SystemTime, UNIX_EPOCH};

	/// struct timespec on the 64-bit targets sally builds for (time_t and long both 64 bits)
	#[repr(C)]
	struct Timespec {
		tv_sec: c_long,
		tv_nsec: c_long,
	}
	const _: () = assert!(std::mem::size_of::<Timespec>() == 16);

	extern "C" {
		fn nanosleep(req: *const Timespec, rem: *mut Timespec) -> c_int;
	}

	/// gettimeofday: CLOCK_REALTIME (SystemTime's clock_gettime) cut to microseconds, as glibc's gettimeofday cuts it
	fn timeofday() -> (u64, u64) {
		let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
		(d.as_secs(), d.subsec_micros() as u64)
	}

	/// ((double)tv.tv_usec) / 1000000. + (tv.tv_sec)
	pub fn absolute_time() -> f64 {
		let (sec, usec) = timeofday();
		usec as f64 / 1000000.0 + sec as f64
	}

	/// ((uint64_t)tv.tv_sec * 1000000) + tv.tv_usec
	pub fn absolute_time_us() -> u64 {
		let (sec, usec) = timeofday();
		sec.wrapping_mul(1000000).wrapping_add(usec)
	}

	pub fn usleep(ius: i32) -> i32 {
		let ts = Timespec { tv_sec: (ius / 1000000) as c_long, tv_nsec: ((ius % 1000000) * 1000) as c_long };
		// SAFETY: nanosleep reads the timespec it is given and writes no remainder (a null pointer asks for none)
		unsafe { nanosleep(&ts, std::ptr::null_mut()) }
	}
}

/// os_generic.windows.h
#[cfg(windows)]
mod windows {
	use std::sync::OnceLock;

	#[link(name = "kernel32")]
	extern "system" {
		fn QueryPerformanceCounter(count: *mut i64) -> i32;
		fn QueryPerformanceFrequency(frequency: *mut i64) -> i32;
		fn Sleep(milliseconds: u32);
	}

	/// The counter's frequency, read once (the C's static LARGE_INTEGER lpf: it is fixed while the machine runs)
	fn frequency() -> i64 {
		static FREQUENCY: OnceLock<i64> = OnceLock::new();
		*FREQUENCY.get_or_init(|| {
			let mut f = 0;
			// SAFETY: the call writes the LARGE_INTEGER it is given
			unsafe { QueryPerformanceFrequency(&mut f) };
			f
		})
	}

	fn counter() -> i64 {
		let mut c = 0;
		// SAFETY: the call writes the LARGE_INTEGER it is given
		unsafe { QueryPerformanceCounter(&mut c) };
		c
	}

	/// (double)li.QuadPart / (double)lpf.QuadPart
	pub fn absolute_time() -> f64 {
		let f = frequency();
		counter() as f64 / f as f64
	}

	/// li.QuadPart * 1000 * 1000 / lpf.QuadPart, returned as a uint64_t, without the C's overflow (us_of)
	pub fn absolute_time_us() -> u64 {
		super::us_of(counter(), frequency())
	}

	pub fn usleep(ius: i32) -> i32 {
		// SAFETY: Sleep takes a number of milliseconds and nothing else (the int's division, then the DWORD it becomes)
		unsafe { Sleep((ius / 1000) as u32) };
		0
	}
}

#[cfg(test)]
mod tests {
	use super::us_of;

	#[test]
	fn the_windows_microseconds_do_not_wrap() {
		let f = 10_000_000;
		// before the C's product wraps, the C's value
		let c = i64::MAX / 1_000_000 - 1;
		assert_eq!(us_of(c, f), (c * 1000 * 1000 / f) as u64);
		assert_eq!(us_of(123_456_789, f), 12_345_678);
		// after it (10.7, 21.3 and 100 days at 10 MHz), the counter's microseconds, still rising
		let mut last = us_of(c, f);
		for days in [10.7, 10.8, 21.3, 21.4, 100.0] {
			let c = (days * 86400.0 * f as f64) as i64;
			let us = us_of(c, f);
			assert_eq!(us, (c / 10) as u64);
			assert!(us > last);
			last = us;
		}
	}
}
