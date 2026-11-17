//! The clocks and sleep of the SteamVR driver and the panel: the C++ standard library's, as libstdc++ has them on Linux
//! and libc++ (llvm-mingw, LLVM 21) on Windows, each with that library's own arithmetic, kept from the C++ the two were
//! ported from so that their times are the same doubles:
//!
//! - `steady_clock`: Linux: clock_gettime(CLOCK_MONOTONIC), tv_sec * 10^9 + tv_nsec nanoseconds; Windows: the
//!   performance counter, which libc++ turns into nanoseconds as whole seconds and the fraction apart (counter /
//!   frequency * 10^9 + counter % frequency * 10^9 / frequency), the frequency read once
//! - `system_clock`: Linux: clock_gettime(CLOCK_REALTIME) in nanoseconds; Windows: in microseconds since 1970, from
//!   GetSystemTimePreciseAsFileTime, which libc++ (built for Windows 7) looks up in kernel32 at start and replaces by
//!   GetSystemTimeAsFileTime where it is missing: the FILETIME's 100 ns ticks less 1601-1970's, divided by 10
//! - `duration<double>(clock::now().time_since_epoch()).count()`: the count as a double over the period's denominator
//!   (count / 1e9, or count / 1e6 for libc++'s system_clock)
//! - `this_thread::sleep_for`: Linux: one nanosleep, again with what is left whenever a signal cuts it short; Windows:
//!   Sleep of the milliseconds, rounded up (libc++'s __libcpp_thread_sleep_for), so on the system's timer tick

#[cfg(unix)]
pub use unix::*;
#[cfg(windows)]
pub use windows::*;

#[cfg(unix)]
mod unix {
	use std::ffi::c_int;

	/// struct timespec on the 64-bit targets sally builds for (time_t and long both 64 bits)
	#[repr(C)]
	struct Timespec {
		tv_sec: i64,
		tv_nsec: i64,
	}
	const _: () = assert!(std::mem::size_of::<Timespec>() == 16);

	const CLOCK_REALTIME: c_int = 0;
	const CLOCK_MONOTONIC: c_int = 1;
	const EINTR: i32 = 4;

	extern "C" {
		fn clock_gettime(clock: c_int, tp: *mut Timespec) -> c_int;
		fn nanosleep(req: *const Timespec, rem: *mut Timespec) -> c_int;
	}

	/// chrono::seconds(tp.tv_sec) + chrono::nanoseconds(tp.tv_nsec): the clock's count of nanoseconds
	fn nanoseconds(clock: c_int) -> i64 {
		let mut tp = Timespec { tv_sec: 0, tv_nsec: 0 };
		// SAFETY: clock_gettime writes the timespec it is given
		unsafe { clock_gettime(clock, &mut tp) };
		tp.tv_sec.wrapping_mul(1_000_000_000).wrapping_add(tp.tv_nsec)
	}

	/// steady_clock::now().time_since_epoch().count(): nanoseconds of CLOCK_MONOTONIC
	pub fn steady_clock_now() -> i64 {
		nanoseconds(CLOCK_MONOTONIC)
	}

	/// duration<double>(steady_clock::now().time_since_epoch()).count()
	pub fn steady_clock_seconds() -> f64 {
		steady_clock_now() as f64 / 1e9
	}

	/// system_clock::now().time_since_epoch().count(): nanoseconds of CLOCK_REALTIME
	pub fn system_clock_now() -> i64 {
		nanoseconds(CLOCK_REALTIME)
	}

	/// duration<double>(system_clock::now().time_since_epoch()).count()
	pub fn system_clock_seconds() -> f64 {
		system_clock_now() as f64 / 1e9
	}

	/// this_thread::sleep_for(chrono::milliseconds(ms)): nothing for 0 or less; otherwise the seconds and the
	/// nanoseconds left over, and `while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}`
	pub fn sleep_for_ms(ms: i64) {
		if ms <= 0 {
			return;
		}
		let mut ts = Timespec { tv_sec: ms / 1000, tv_nsec: (ms % 1000) * 1_000_000 };
		loop {
			let p: *mut Timespec = &mut ts;
			// SAFETY: nanosleep reads the request and writes what is left into the same timespec, which lives here
			if unsafe { nanosleep(p, p) } != -1 || std::io::Error::last_os_error().raw_os_error() != Some(EINTR) {
				break;
			}
		}
	}
}

#[cfg(windows)]
mod windows {
	#![allow(non_snake_case, clippy::upper_case_acronyms)]

	use std::ffi::c_void;
	use std::sync::OnceLock;

	#[repr(C)]
	struct FILETIME {
		dwLowDateTime: u32,
		dwHighDateTime: u32,
	}

	type GetSystemTimeAsFileTimePtr = unsafe extern "system" fn(*mut FILETIME);

	#[link(name = "kernel32")]
	extern "system" {
		fn QueryPerformanceCounter(count: *mut i64) -> i32;
		fn QueryPerformanceFrequency(frequency: *mut i64) -> i32;
		fn GetSystemTimeAsFileTime(ft: *mut FILETIME);
		fn GetModuleHandleW(name: *const u16) -> *mut c_void;
		fn GetProcAddress(module: *mut c_void, name: *const u8) -> *mut c_void;
		fn Sleep(milliseconds: u32);
	}

	/// static const LARGE_INTEGER freq = __QueryPerformanceFrequency(): read once
	fn frequency() -> i64 {
		static FREQUENCY: OnceLock<i64> = OnceLock::new();
		*FREQUENCY.get_or_init(|| {
			let mut f = 0;
			// SAFETY: the call writes the LARGE_INTEGER it is given
			unsafe { QueryPerformanceFrequency(&mut f) };
			f
		})
	}

	/// __libcpp_steady_clock_now: seconds * nano::den + fractions * nano::den / freq, in __int64 arithmetic
	pub fn steady_clock_now() -> i64 {
		let freq = frequency();
		let mut counter = 0i64;
		// SAFETY: the call writes the LARGE_INTEGER it is given
		unsafe { QueryPerformanceCounter(&mut counter) };
		let seconds = counter / freq;
		let fractions = counter % freq;
		seconds.wrapping_mul(1_000_000_000).wrapping_add(fractions.wrapping_mul(1_000_000_000) / freq)
	}

	/// duration<double>(steady_clock::now().time_since_epoch()).count()
	pub fn steady_clock_seconds() -> f64 {
		steady_clock_now() as f64 / 1e9
	}

	/// GetSystemTimeInit: GetSystemTimePreciseAsFileTime from kernel32.dll, or GetSystemTimeAsFileTime without it
	fn get_system_time_as_file_time() -> GetSystemTimeAsFileTimePtr {
		static FP: OnceLock<usize> = OnceLock::new();
		let fp = *FP.get_or_init(|| {
			let kernel32: Vec<u16> = "kernel32.dll\0".encode_utf16().collect();
			// SAFETY: both calls take NUL-terminated names and only look the module and the function up
			let f = unsafe {
				GetProcAddress(GetModuleHandleW(kernel32.as_ptr()), c"GetSystemTimePreciseAsFileTime".as_ptr().cast())
			};
			if f.is_null() {
				GetSystemTimeAsFileTime as GetSystemTimeAsFileTimePtr as usize
			} else {
				f as usize
			}
		});
		// SAFETY: fp is the address of a kernel32 function of the type `void (LPFILETIME)`, found above
		unsafe { std::mem::transmute::<usize, GetSystemTimeAsFileTimePtr>(fp) }
	}

	/// __libcpp_system_clock_now: duration_cast<microseconds>(the FILETIME's 100 ns ticks - nt_to_unix_epoch), the
	/// count of microseconds since 1970 (the division by 10 cuts toward zero)
	pub fn system_clock_now() -> i64 {
		let mut ft = FILETIME { dwLowDateTime: 0, dwHighDateTime: 0 };
		// SAFETY: the function writes the FILETIME it is given
		unsafe { get_system_time_as_file_time()(&mut ft) };
		let ticks = ((ft.dwHighDateTime as i64) << 32) | ft.dwLowDateTime as i64;
		ticks.wrapping_sub(11_644_473_600 * 10_000_000) / 10
	}

	/// duration<double>(system_clock::now().time_since_epoch()).count(): libc++'s system_clock counts microseconds
	pub fn system_clock_seconds() -> f64 {
		system_clock_now() as f64 / 1e6
	}

	/// this_thread::sleep_for(chrono::milliseconds(ms)): nothing for 0 or less, otherwise Sleep(ms) (the nanoseconds'
	/// ceiling in milliseconds is ms itself)
	pub fn sleep_for_ms(ms: i64) {
		if ms <= 0 {
			return;
		}
		// SAFETY: Sleep takes a number of milliseconds and nothing else
		unsafe { Sleep(ms as u32) };
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_clocks_run() {
		let s0 = steady_clock_seconds();
		let w0 = system_clock_seconds();
		sleep_for_ms(20);
		let s1 = steady_clock_seconds();
		let w1 = system_clock_seconds();
		assert!(s1 - s0 >= 0.019, "{s0} {s1}");
		assert!((w1 - w0) > 0.0 && (w1 - w0) < 5.0, "{w0} {w1}");
		// the wall clock is the epoch's seconds, as std's
		let std_now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64();
		assert!((std_now - w1).abs() < 5.0, "{std_now} {w1}");
		sleep_for_ms(0);
		sleep_for_ms(-5);
	}
}
