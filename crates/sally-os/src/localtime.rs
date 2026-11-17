//! The C library's local time, for clock times (the events' "%H:%M:%S" and the recordings' "sally_%Y%m%d_%H%M%S"):
//! glibc's localtime_r on Linux, the ucrt's _localtime64_s on Windows, each with the system's time zone and its
//! daylight saving.

use std::ffi::c_int;

/// struct tm: the nine ints both libraries start it with, and glibc's two fields after them (tm_gmtoff, tm_zone),
/// which the ucrt's has not
#[repr(C)]
struct Tm {
	tm_sec: c_int,
	tm_min: c_int,
	tm_hour: c_int,
	tm_mday: c_int,
	tm_mon: c_int,
	tm_year: c_int,
	tm_wday: c_int,
	tm_yday: c_int,
	tm_isdst: c_int,
	#[cfg(unix)]
	tm_gmtoff: i64,
	#[cfg(unix)]
	tm_zone: *const u8,
}

#[cfg(unix)]
extern "C" {
	fn localtime_r(t: *const i64, tm: *mut Tm) -> *mut Tm;
}

#[cfg(windows)]
extern "C" {
	fn _localtime64_s(tm: *mut Tm, t: *const i64) -> c_int;
}

/// A local date and time of day, to the second
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LocalTime {
	pub year: i32,
	/// 1 to 12
	pub month: i32,
	/// 1 to 31
	pub day: i32,
	pub hour: i32,
	pub minute: i32,
	pub second: i32,
}

impl LocalTime {
	/// strftime's "%H:%M:%S"
	pub fn hms(&self) -> String {
		format!("{:02}:{:02}:{:02}", self.hour, self.minute, self.second)
	}

	/// strftime's "%Y%m%d_%H%M%S"
	pub fn stamp(&self) -> String {
		format!("{:04}{:02}{:02}_{:02}{:02}{:02}", self.year, self.month, self.day, self.hour, self.minute, self.second)
	}
}

/// localtime(&t) for a Unix time t (seconds): the local date and time, None where the library has none for t
pub fn local_time(t: i64) -> Option<LocalTime> {
	let mut tm = Tm {
		tm_sec: 0,
		tm_min: 0,
		tm_hour: 0,
		tm_mday: 0,
		tm_mon: 0,
		tm_year: 0,
		tm_wday: 0,
		tm_yday: 0,
		tm_isdst: 0,
		#[cfg(unix)]
		tm_gmtoff: 0,
		#[cfg(unix)]
		tm_zone: std::ptr::null(),
	};
	// SAFETY: both functions read the time_t they are given and write the struct tm they are given, nothing else
	#[cfg(unix)]
	let ok = unsafe { !localtime_r(&t, &mut tm).is_null() };
	// SAFETY: as above
	#[cfg(windows)]
	let ok = unsafe { _localtime64_s(&mut tm, &t) == 0 };
	ok.then_some(LocalTime {
		year: tm.tm_year + 1900,
		month: tm.tm_mon + 1,
		day: tm.tm_mday,
		hour: tm.tm_hour,
		minute: tm.tm_min,
		second: tm.tm_sec,
	})
}

/// The local time now
pub fn local_now() -> Option<LocalTime> {
	let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0);
	local_time(t)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_day_has_its_fields_in_range() {
		// 2026-11-07 12:00:00 UTC: whatever the zone, the same date within a day and the fields in their ranges
		let lt = local_time(1_794_052_800).expect("localtime");
		assert_eq!((lt.year, lt.month), (2026, 11));
		assert!((6..=8).contains(&lt.day), "{lt:?}");
		assert!((0..24).contains(&lt.hour) && (0..60).contains(&lt.minute) && lt.second == 0, "{lt:?}");
		assert_eq!(lt.hms().len(), 8);
		assert_eq!(lt.stamp().len(), 15);
	}
}
