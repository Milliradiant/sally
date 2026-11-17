//! The tap on the dongles (the live half of the C runtime's driver_hidtap.c, `--hidtap 1`): the lighthouse HID devices
//! opened shared and read only, next to Valve's driver, their input reports handed to the tap (`tap::Tap`) and copied
//! to the `--hidtap-record` file. Windows only, as the C is: elsewhere the tap does not start.
//!
//! The threads are the C's. The scan thread lists the HID interfaces every 3 s (SetupDi) and hands over the paths that
//! carry the tap's vendor id. The reader thread opens the new ones (each device created in the tap as it is opened, its
//! first read started at once), waits on the overlapped reads 100 ms at a time, splits each read into reports on the
//! device's input report length, and for each report writes its line to the record file and hands it to the tap under
//! the context lock. A device that has no config gets a config fetch thread of its own once it has streamed for a
//! while, which reads the config over feature reports and takes the lock for each chunk.
//!
//! The context lock is a `Mutex` around whatever holds the tap's state and the runtime it feeds (a `Context`), as the
//! C's context lock covers both: `Live::start` starts the reader on it (with the lock held, as survive_startup holds
//! it), `Live::stop` stops it (without).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;

use crate::tap::{Record, Runtime, Tap};

#[cfg(windows)]
pub mod sys;

/// HIDTAP_LIVE_MAX: the most devices open at once
pub const MAX_OPEN: usize = 32;

/// HIDTAP_SCAN_MAX: the most paths one scan hands over
pub const SCAN_MAX: usize = 64;

/// hidtap_live_dev_t's buffer for a device path (char path[512]): a longer path is not opened
pub const PATH_LEN: usize = 512;

/// hidtap_live_dev_t's buffer for a serial (char serial[64])
pub const SERIAL_LEN: usize = 64;

/// hidtap_live_dev_t's buffer for one read (uint8_t buf[2048])
pub const READ_LEN: usize = 2048;

/// sally: the failed reads in a row that close a device (the next scan opens it again if it is still there)
pub const READ_ERRORS_MAX: i32 = 3;

/// sally: how often a device's config is read from the tap again once it has one (the tap gives a device up when
/// SteamVR puts it on another dongle, and the dongle is fetched again)
pub const CONFIGURED_RECHECK_S: f64 = 5.0;

/// What the live tap's threads lock, as the C's context lock: the tap's state and the runtime it feeds
pub trait Context: Send + 'static {
	fn tap(&mut self) -> (&mut Tap, &mut dyn Runtime);
}

/// The live tap running (the C's threaded driver "hidtap_live"): its reader thread and the flag that keeps it going
pub struct Live<C: Context> {
	ctx: Arc<Mutex<C>>,
	keep_running: Arc<AtomicBool>,
	/// the reader thread, which gives the record file back when it ends
	thread: Option<JoinHandle<Option<Record>>>,
}

impl<C: Context> Live<C> {
	/// DriverRegHIDTap in live mode, on a tap made with `Mode::Live`: on Windows "hidtap: live, reading vid ...", the
	/// record file opened when `record` names one, and the reader thread started. Elsewhere the C's error ("hidtap:
	/// live mode is Windows only; replay a recording instead"), and None (SURVIVE_DRIVER_ERROR). Called with the
	/// context locked, as survive_startup calls it (`locked` is the locked context): the reader waits for the lock at
	/// its first device
	#[cfg(not(windows))]
	pub fn start(ctx: &Arc<Mutex<C>>, locked: &mut C, record: Option<&str>) -> Option<Live<C>> {
		let _ = (ctx, record);
		let (_, rt) = locked.tap();
		rt.error(
			crate::tap::SURVIVE_ERROR_INVALID_CONFIG,
			"hidtap: live mode is Windows only; replay a recording instead",
		);
		None
	}

	/// DriverRegHIDTap in live mode, on a tap made with `Mode::Live`: on Windows "hidtap: live, reading vid ...", the
	/// record file opened when `record` names one, and the reader thread started. Elsewhere the C's error ("hidtap:
	/// live mode is Windows only; replay a recording instead"), and None (SURVIVE_DRIVER_ERROR). Called with the
	/// context locked, as survive_startup calls it (`locked` is the locked context): the reader waits for the lock at
	/// its first device
	#[cfg(windows)]
	pub fn start(ctx: &Arc<Mutex<C>>, locked: &mut C, record: Option<&str>) -> Option<Live<C>> {
		let (tap, rt) = locked.tap();
		let options = tap.options().clone();
		rt.info(&format!(
			"hidtap: live, reading vid {:04x} HID devices next to Valve's driver (lightcap mode {}, config fetch {})",
			options.vid,
			options.lightcap_mode,
			if options.config_fetch { "on" } else { "off" }
		));
		let record = record.filter(|r| !r.is_empty()).and_then(|r| Record::open(rt, r));
		let keep_running = Arc::new(AtomicBool::new(true));
		// survive_add_threaded_driver: OGCreateThread's thread (a thread that cannot be made leaves the driver loaded
		// with nothing reading, as the C's handle 0 does)
		let thread = {
			let (ctx, keep_running) = (ctx.clone(), keep_running.clone());
			sally_os::create_thread("hidtap_live", move || reader::run(ctx, &keep_running, record, &options))
		};
		Some(Live { ctx: ctx.clone(), keep_running, thread })
	}

	/// The stop: the keep-running flag cleared and the reader joined (it waits up to 5 s each for the scan thread and
	/// the config fetches, and closes every device), then with the context locked the summary logged and the record
	/// file closed. Called without the context locked: the reader and the fetches take the lock until they end
	pub fn stop(mut self) {
		self.keep_running.store(false, Ordering::SeqCst);
		let record = self.thread.take().and_then(|t| t.join().ok()).flatten();
		let mut g = lock(&self.ctx);
		let (tap, rt) = g.tap();
		tap.close(rt, record);
	}
}

/// The context lock (survive_get_ctx_lock)
fn lock<C>(ctx: &Mutex<C>) -> MutexGuard<'_, C> {
	ctx.lock().unwrap_or_else(|e| e.into_inner())
}

/// The number after `key` in a device path ("&mi_00" is interface 0, "&col02" collection 2), read as the ucrt's
/// strtol(.., 16) reads it; -1 when the path does not have `key`
pub fn path_field(path: &[u8], key: &[u8]) -> i32 {
	// strstr
	let found = if key.is_empty() { Some(0) } else { path.windows(key.len()).position(|w| w == key) };
	match found {
		Some(at) => strtol_hex(&path[at + key.len()..]),
		None => -1,
	}
}

/// strtol(s, NULL, 16) as the ucrt has it, a long being 32 bits on Windows: white space, a sign, an optional 0x, hex
/// digits; 0 without any, LONG_MAX or LONG_MIN past the range
pub fn strtol_hex(s: &[u8]) -> i32 {
	let at = |i: usize| s.get(i).copied().unwrap_or(0);
	let mut i = 0;
	while matches!(at(i), b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r') {
		i += 1;
	}
	let negative = at(i) == b'-';
	if negative || at(i) == b'+' {
		i += 1;
	}
	if at(i) == b'0' && (at(i + 1) == b'x' || at(i + 1) == b'X') {
		i += 2;
	}
	// past 2^32 the value only needs to stay past the range
	let mut v: u64 = 0;
	while let Some(d) = (at(i) as char).to_digit(16) {
		v = (v * 16 + d as u64).min(1 << 33);
		i += 1;
	}
	match (negative, v) {
		(false, v) if v > i32::MAX as u64 => i32::MAX,
		(true, v) if v > 1 << 31 => i32::MIN,
		(true, v) => (v as i64).wrapping_neg() as i32,
		(false, v) => v as i32,
	}
}

/// A wide string up to its 0 as ASCII, '?' for anything past 127, in a buffer of `n` (so at most n - 1 characters)
pub fn wide_to_ascii(w: &[u16], n: usize) -> String {
	w.iter()
		.take_while(|&&c| c != 0)
		.take(n.saturating_sub(1))
		.map(|&c| if c < 128 { c as u8 as char } else { '?' })
		.collect()
}

/// hidtap_live_got's split: one ReadFile can return several queued reports back to back, so a read of `got` bytes is
/// cut at the device's input report length (the whole read when the length is not known); the last piece may be short
pub fn reports_in(got: usize, input_len: usize) -> impl Iterator<Item = std::ops::Range<usize>> {
	let rlen = if input_len != 0 { input_len } else { got };
	let mut off = 0;
	std::iter::from_fn(move || {
		if off >= got {
			return None;
		}
		let n = if got - off < rlen { got - off } else { rlen };
		let r = off..off + n;
		off += rlen;
		Some(r)
	})
}

/// The reader, the scan and the config fetches (Windows)
#[cfg(windows)]
mod reader {
	use std::ffi::CString;
	use std::os::windows::io::AsRawHandle;
	use std::ptr::{null, null_mut};
	use std::sync::atomic::{AtomicBool, Ordering};
	use std::sync::{Arc, Mutex};
	use std::thread::JoinHandle;

	use sally_os::absolute_time;

	use super::sys::*;
	use super::{
		lock, path_field, reports_in, wide_to_ascii, Context, CONFIGURED_RECHECK_S, MAX_OPEN, PATH_LEN,
		READ_ERRORS_MAX, READ_LEN, SCAN_MAX, SERIAL_LEN,
	};
	use crate::line;
	use crate::tap::{DeviceId, Options, Record};

	fn info<C: Context>(ctx: &Mutex<C>, text: &str) {
		lock(ctx).tap().1.info(text);
	}

	fn warn<C: Context>(ctx: &Mutex<C>, text: &str) {
		lock(ctx).tap().1.warn(text);
	}

	/// WaitForSingleObject on a thread: whether it ended within `ms`
	fn ended<T>(t: &JoinHandle<T>, ms: DWORD) -> bool {
		unsafe { WaitForSingleObject(t.as_raw_handle() as HANDLE, ms) == WAIT_OBJECT_0 }
	}

	/// A thread waited for up to `ms`, then its handle closed: joined if it ended, left to run on if not
	fn wait_and_close<T>(t: JoinHandle<T>, ms: DWORD) {
		if ended(&t, ms) {
			let _ = t.join();
		}
	}

	/// The scan's hand-over (scan_lock, scan_paths, scan_ready, scan_stop): the paths of the last scan until the reader
	/// takes them
	#[derive(Default)]
	struct Scan {
		paths: Mutex<Option<Vec<Vec<u8>>>>,
		stop: AtomicBool,
	}

	/// One read's OVERLAPPED and buffer, on the heap so they stay where the system writes them while a read is pending
	struct Io {
		ov: OVERLAPPED,
		buf: [u8; READ_LEN],
	}

	impl Io {
		fn new() -> Box<Io> {
			Box::new(Io {
				ov: OVERLAPPED { Internal: 0, InternalHigh: 0, Offset: 0, OffsetHigh: 0, hEvent: null_mut() },
				buf: [0; READ_LEN],
			})
		}
	}

	/// An open device (or a slot free for the next)
	struct LiveDev {
		path: CString,
		serial: String,
		pid: u16,
		mi: i32,
		col: i32,
		input_len: u16,
		feature_len: u16,
		h: HANDLE,
		io: Box<Io>,
		/// the tap's device (hidtap_device when it was opened); None for one of an unknown type
		dev: Option<DeviceId>,
		/// dev->so != 0 (set when the device is created) and dev->hasConfiged as last read from the tap (sally: at each
		/// pass while it has none, and every CONFIGURED_RECHECK_S once it has one, at `configured_at`): what the config
		/// fetch asks of the tap
		has_object: bool,
		configured: bool,
		configured_at: f64,
		first_seen: f64,
		next_fetch: f64,
		fetch_thread: Option<JoinHandle<()>>,
		gone: bool,
		reports: u64,
		/// the read errors since the device was opened (the first three are logged), and those since its last good read
		errors: i32,
		errors_in_a_row: i32,
	}

	impl LiveDev {
		/// memset(ld, 0), ld->h = INVALID_HANDLE_VALUE, the path copied
		fn new(path: &[u8]) -> LiveDev {
			LiveDev {
				path: CString::new(&path[..path.len().min(PATH_LEN - 1)]).unwrap_or_default(),
				serial: String::new(),
				pid: 0,
				mi: 0,
				col: 0,
				input_len: 0,
				feature_len: 0,
				h: INVALID_HANDLE_VALUE,
				io: Io::new(),
				dev: None,
				has_object: false,
				configured: false,
				configured_at: 0.0,
				first_seen: 0.0,
				next_fetch: 0.0,
				fetch_thread: None,
				gone: false,
				reports: 0,
				errors: 0,
				errors_in_a_row: 0,
			}
		}
	}

	/// The slot a new device goes in: a dropped one, or the next one not used yet
	#[derive(Clone, Copy)]
	enum Slot {
		Reuse(usize),
		New,
	}

	/// The reader thread's state
	struct Reader<C: Context> {
		ctx: Arc<Mutex<C>>,
		/// d->record, written from this thread
		record: Option<Record>,
		vid: u16,
		config_fetch: bool,
		config_delay: f64,
		scan: Arc<Scan>,
		/// devs[0..ndevs]
		devs: Vec<LiveDev>,
	}

	/// The scan thread started; then until keepRunning is cleared, what the scan found opened, the config fetches
	/// looked after, and the reads waited on (100 ms at a time) and each one's reports handed on before the next read
	/// is started. At the end the scan thread is stopped and waited for (5 s), every device closed, and every config
	/// fetch waited for (5 s). The record file is given back
	pub(super) fn run<C: Context>(
		ctx: Arc<Mutex<C>>, keep_running: &AtomicBool, record: Option<Record>, options: &Options,
	) -> Option<Record> {
		let scan = Arc::new(Scan::default());
		let scan_thread = {
			let (scan, vid) = (scan.clone(), options.vid);
			std::thread::Builder::new().name("hidtap_scan".into()).spawn(move || scan_devices(&scan, vid)).ok()
		};
		let mut r = Reader {
			ctx,
			record,
			vid: options.vid,
			config_fetch: options.config_fetch,
			config_delay: options.config_delay,
			scan,
			devs: Vec::with_capacity(MAX_OPEN),
		};

		while keep_running.load(Ordering::SeqCst) {
			let now = absolute_time();
			// opens what the scan thread found (nothing to do between scans)
			r.enumerate();
			let mut events: Vec<HANDLE> = Vec::with_capacity(MAX_OPEN);
			let mut by_event: Vec<usize> = Vec::with_capacity(MAX_OPEN);
			for i in 0..r.devs.len() {
				if r.devs[i].h == INVALID_HANDLE_VALUE || r.devs[i].gone {
					continue;
				}
				r.maybe_fetch(i, now);
				events.push(r.devs[i].io.ov.hEvent);
				by_event.push(i);
			}
			if events.is_empty() {
				sally_os::usleep(50000);
				continue;
			}
			let w = unsafe { WaitForMultipleObjects(events.len() as DWORD, events.as_ptr(), FALSE, 100) };
			if w == WAIT_TIMEOUT {
				continue;
			}
			let idx = w.wrapping_sub(WAIT_OBJECT_0) as i32;
			if idx < 0 || idx as usize >= events.len() {
				continue;
			}
			let i = by_event[idx as usize];
			let mut got: DWORD = 0;
			let ld = &mut r.devs[i];
			if unsafe { GetOverlappedResult(ld.h, &mut ld.io.ov, &mut got, FALSE) } != 0 {
				r.got(i, got);
			} else {
				let e = unsafe { GetLastError() };
				if e == ERROR_IO_INCOMPLETE {
					unsafe { ResetEvent(ld.io.ov.hEvent) };
					continue;
				}
				if ld.errors < 3 {
					warn(&r.ctx, &format!("hidtap: {}: read error {}", ld.serial, e));
				}
				ld.errors = ld.errors.saturating_add(1);
				if e == ERROR_DEVICE_NOT_CONNECTED || e == ERROR_OPERATION_ABORTED {
					info(&r.ctx, &format!("hidtap: {} unplugged", ld.serial));
					r.drop_dev(i);
					continue;
				}
				// sally: any other error is taken as passing and the read started again, but a few in a row close the
				// device (a read that fails at once never lets the reader wait, and the wait gives the first signaled
				// device, so the devices after it would get no reads); the next scan opens it again if it is still there
				ld.errors_in_a_row = ld.errors_in_a_row.saturating_add(1);
				if ld.errors_in_a_row >= READ_ERRORS_MAX {
					info(
						&r.ctx,
						&format!("hidtap: {} closed after {} read errors in a row", ld.serial, ld.errors_in_a_row),
					);
					r.drop_dev(i);
					continue;
				}
			}
			r.start_read(i);
		}

		r.scan.stop.store(true, Ordering::SeqCst);
		if let Some(t) = scan_thread {
			wait_and_close(t, 5000);
		}
		for ld in &mut r.devs {
			let ev = ld.io.ov.hEvent;
			if ld.h != INVALID_HANDLE_VALUE {
				unsafe { CancelIo(ld.h) };
				// the system writes a cancelled read's OVERLAPPED when it finishes: its memory is freed only after
				// that, and never if it has not finished in a second
				if unsafe { WaitForSingleObject(ev, 1000) } != WAIT_OBJECT_0 {
					std::mem::forget(std::mem::replace(&mut ld.io, Io::new()));
				}
				unsafe { CloseHandle(ld.h) };
			}
			if !ev.is_null() {
				unsafe { CloseHandle(ev) };
			}
			if let Some(t) = ld.fetch_thread.take() {
				wait_and_close(t, 5000);
			}
		}
		r.record
	}

	impl<C: Context> Reader<C> {
		/// Whether a device of this path is open
		fn find(&self, path: &[u8]) -> bool {
			self.devs.iter().any(|ld| !ld.gone && ld.path.as_bytes() == path)
		}

		/// A dropped slot whose config fetch has finished, else a new one while there is room
		fn slot(&mut self) -> Option<Slot> {
			for i in 0..self.devs.len() {
				let ld = &mut self.devs[i];
				if !ld.gone {
					continue;
				}
				if let Some(t) = &ld.fetch_thread {
					if !ended(t, 0) {
						continue;
					}
					if let Some(t) = ld.fetch_thread.take() {
						let _ = t.join();
					}
				}
				return Some(Slot::Reuse(i));
			}
			if self.devs.len() < MAX_OPEN {
				Some(Slot::New)
			} else {
				None
			}
		}

		/// The devices the scan found that are not open yet, opened shared and read only (with the scan's paths taken,
		/// when it has handed some over)
		fn enumerate(&mut self) {
			let Some(paths) = lock(&self.scan.paths).take() else { return };
			for path in &paths {
				let slot = if self.find(path) { None } else { self.slot() };
				let Some(slot) = slot else { continue };
				let Ok(cpath) = CString::new(path.as_slice()) else { continue };
				let h = unsafe {
					CreateFileA(
						cpath.as_ptr().cast(),
						0,
						FILE_SHARE_READ | FILE_SHARE_WRITE,
						null_mut(),
						OPEN_EXISTING,
						0,
						null_mut(),
					)
				};
				if h == INVALID_HANDLE_VALUE {
					continue;
				}
				let mut a =
					HIDD_ATTRIBUTES { Size: std::mem::size_of::<HIDD_ATTRIBUTES>() as u32, ..Default::default() };
				if unsafe { HidD_GetAttributes(h, &mut a) } != 0 && a.VendorID == self.vid {
					self.open(slot, path, h, a.ProductID);
				}
				unsafe { CloseHandle(h) };
			}
		}

		/// A device of the tap's vendor id opened in `slot`: its interface and collection from its path, its serial
		/// (or "nosn-<slot>"), its report lengths, then the handle the reads go through (overlapped), 512 input
		/// buffers, "hidtap: opened ...", the device created in the tap, and the first read started
		fn open(&mut self, slot: Slot, path: &[u8], h: HANDLE, pid: u16) {
			let index = match slot {
				Slot::Reuse(i) => i,
				Slot::New => self.devs.len(),
			};
			let mut ld = LiveDev::new(path);
			ld.pid = pid;
			ld.mi = path_field(ld.path.as_bytes(), b"&mi_");
			ld.col = path_field(ld.path.as_bytes(), b"&col");
			let mut w = [0u16; 128];
			if unsafe { HidD_GetSerialNumberString(h, w.as_mut_ptr().cast(), std::mem::size_of_val(&w) as u32) } != 0 {
				ld.serial = wide_to_ascii(&w, SERIAL_LEN);
			} else {
				ld.serial = format!("nosn-{index}");
			}
			let mut pp: PHIDP_PREPARSED_DATA = null_mut();
			if unsafe { HidD_GetPreparsedData(h, &mut pp) } != 0 {
				let mut caps = HIDP_CAPS::default();
				if unsafe { HidP_GetCaps(pp, &mut caps) } == HIDP_STATUS_SUCCESS {
					ld.input_len = caps.InputReportByteLength;
					ld.feature_len = caps.FeatureReportByteLength;
				}
				unsafe { HidD_FreePreparsedData(pp) };
			}
			ld.h = unsafe {
				CreateFileA(
					ld.path.as_ptr().cast(),
					GENERIC_READ,
					FILE_SHARE_READ | FILE_SHARE_WRITE,
					null_mut(),
					OPEN_EXISTING,
					FILE_FLAG_OVERLAPPED,
					null_mut(),
				)
			};
			let failed = if ld.h == INVALID_HANDLE_VALUE {
				Some(("", unsafe { GetLastError() }))
			} else {
				unsafe { HidD_SetNumInputBuffers(ld.h, 512) };
				ld.io.ov.hEvent = unsafe { CreateEventA(null_mut(), TRUE, FALSE, null()) };
				// sally: without its event the open fails too (a null handle fails every wait on the devices, so none
				// would be read). No read was started on the handle
				if ld.io.ov.hEvent.is_null() {
					let e = unsafe { GetLastError() };
					unsafe { CloseHandle(ld.h) };
					ld.h = INVALID_HANDLE_VALUE;
					Some((" (no event)", e))
				} else {
					None
				}
			};
			if let Some((what, e)) = failed {
				warn(
					&self.ctx,
					&format!("hidtap: cannot open {} ({:04x}) for reading{what}: error {}", ld.serial, ld.pid, e),
				);
				// the slot stays free (a reused one must not look open)
				ld.gone = true;
				if let Slot::Reuse(i) = slot {
					self.devs[i] = ld;
				}
				return;
			}
			ld.first_seen = absolute_time();
			info(
				&self.ctx,
				&format!("hidtap: opened {} ({:04x}, mi {}, {}-byte reports)", ld.serial, ld.pid, ld.mi, ld.input_len),
			);
			match slot {
				Slot::Reuse(i) => self.devs[i] = ld,
				Slot::New => self.devs.push(ld),
			}
			{
				let mut g = lock(&self.ctx);
				let (tap, rt) = g.tap();
				let ld = &mut self.devs[index];
				ld.dev = tap.device(rt, &ld.serial, ld.pid);
				ld.has_object = ld.dev.is_some_and(|d| tap.has_object(d));
			}
			self.start_read(index);
		}

		/// A device that went away (unplugged, or a read error): its handle closed, its slot free for the next scan,
		/// which opens it again when it is back. No read of it is pending: it failed or ended
		fn drop_dev(&mut self, i: usize) {
			let ld = &mut self.devs[i];
			ld.gone = true;
			if ld.h != INVALID_HANDLE_VALUE && !ld.h.is_null() {
				unsafe {
					CancelIo(ld.h);
					CloseHandle(ld.h);
				}
			}
			ld.h = INVALID_HANDLE_VALUE;
			if !ld.io.ov.hEvent.is_null() {
				unsafe { CloseHandle(ld.io.ov.hEvent) };
			}
			ld.io.ov.hEvent = null_mut();
		}

		/// The next read started; one that completes at once is handed on and another started; one that fails drops the
		/// device
		fn start_read(&mut self, i: usize) {
			loop {
				let ld = &mut self.devs[i];
				let ev = ld.io.ov.hEvent;
				ld.io.ov = OVERLAPPED { Internal: 0, InternalHigh: 0, Offset: 0, OffsetHigh: 0, hEvent: ev };
				unsafe { ResetEvent(ev) };
				let mut got: DWORD = 0;
				let io = &mut *ld.io;
				if unsafe { ReadFile(ld.h, io.buf.as_mut_ptr().cast(), READ_LEN as DWORD, &mut got, &mut io.ov) } != 0 {
					self.got(i, got);
					continue;
				}
				let e = unsafe { GetLastError() };
				if e == ERROR_IO_PENDING {
					return;
				}
				if ld.errors < 3 {
					warn(&self.ctx, &format!("hidtap: {}: read failed: error {}", ld.serial, e));
				}
				ld.errors = ld.errors.saturating_add(1);
				self.drop_dev(i);
				return;
			}
		}

		/// A read of `got` bytes, its time taken once, then each report in it (hidtap_live_report): counted, its line
		/// written to the record file, and handed to the tap under the context lock (sally: with the `A` lines the tap
		/// is owing the record file, which follow the report's line: a replay applies an identity one report later than
		/// the live run did)
		fn got(&mut self, i: usize, got: DWORD) {
			let t = (absolute_time() * 1e6) as u64;
			let ld = &mut self.devs[i];
			// sally: a good read
			ld.errors_in_a_row = 0;
			for r in reports_in(got as usize, ld.input_len as usize) {
				ld.reports += 1;
				let data = &ld.io.buf[r];
				if let Some(rec) = self.record.as_mut() {
					rec.put(&line::input_report(t, &ld.serial, ld.pid, ld.mi, ld.col, data));
				}
				let mut g = lock(&self.ctx);
				let (tap, rt) = g.tap();
				// sally: identities given since the last report (Tap::assign), written where the reader is
				if let Some(rec) = self.record.as_mut() {
					for l in tap.take_record_lines() {
						rec.put(&l);
					}
				}
				tap.live_report(rt, ld.dev, ld.pid, ld.mi, ld.col, t, data);
			}
		}

		/// A config fetch started for a device that has streamed for config-delay seconds and has no config (with
		/// config fetch on, and a tracked object), one at a time and a minute apart; a fetch that ended with the device
		/// still unconfigured failed. Sally: a device the tap has given up (`Tap::assign`) has no config again, seen
		/// within CONFIGURED_RECHECK_S, and is fetched as one that never had one
		fn maybe_fetch(&mut self, i: usize, now: f64) {
			let ld = &mut self.devs[i];
			let Some(dev) = ld.dev else { return };
			if !self.config_fetch || !ld.has_object || ld.reports == 0 {
				return;
			}
			if !ld.configured || now - ld.configured_at >= CONFIGURED_RECHECK_S {
				ld.configured = lock(&self.ctx).tap().0.is_configured(dev);
				ld.configured_at = now;
			}
			if ld.configured {
				return;
			}
			if let Some(t) = &ld.fetch_thread {
				if !ended(t, 0) {
					// still running
					return;
				}
				if let Some(t) = ld.fetch_thread.take() {
					let _ = t.join();
				}
				// failed: try again in a minute
				ld.next_fetch = now + 60.0;
				return;
			}
			if now - ld.first_seen < self.config_delay || now < ld.next_fetch {
				return;
			}
			let (ctx, path, serial, feature_len) =
				(self.ctx.clone(), ld.path.clone(), ld.serial.clone(), ld.feature_len);
			ld.fetch_thread = std::thread::Builder::new()
				.name("hidtap_fetch".into())
				.spawn(move || fetch_config(&ctx, &path, &serial, feature_len, dev))
				.ok();
			ld.next_fetch = now + 60.0;
		}
	}

	/// Every 3 s, the HID interfaces listed (SetupDi) and the paths with the vendor id in them ("vid_28de" in the
	/// lower-cased path: no device is opened here) handed over, at most SCAN_MAX
	fn scan_devices(scan: &Scan, vid: u16) {
		let want = format!("vid_{vid:04x}").into_bytes();
		let mut guid = GUID::default();
		unsafe { HidD_GetHidGuid(&mut guid) };
		while !scan.stop.load(Ordering::SeqCst) {
			let mut found: Vec<Vec<u8>> = Vec::new();
			let set = unsafe { SetupDiGetClassDevsA(&guid, null(), null_mut(), DIGCF_PRESENT | DIGCF_DEVICEINTERFACE) };
			if set != INVALID_HANDLE_VALUE {
				let mut ifd = SP_DEVICE_INTERFACE_DATA {
					cbSize: std::mem::size_of::<SP_DEVICE_INTERFACE_DATA>() as DWORD,
					InterfaceClassGuid: GUID::default(),
					Flags: 0,
					Reserved: 0,
				};
				let mut i: DWORD = 0;
				while !scan.stop.load(Ordering::SeqCst)
					&& found.len() < SCAN_MAX
					&& unsafe { SetupDiEnumDeviceInterfaces(set, null_mut(), &guid, i, &mut ifd) } != 0
				{
					let mut need: DWORD = 0;
					unsafe { SetupDiGetDeviceInterfaceDetailA(set, &mut ifd, null_mut(), 0, &mut need, null_mut()) };
					// calloc(1, need + 8), aligned for the structure
					let mut det = vec![0u64; (need as usize + 8).div_ceil(8)];
					let p = det.as_mut_ptr() as *mut SP_DEVICE_INTERFACE_DETAIL_DATA_A;
					unsafe { (*p).cbSize = std::mem::size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_A>() as DWORD };
					if unsafe { SetupDiGetDeviceInterfaceDetailA(set, &mut ifd, p, need, null_mut(), null_mut()) } != 0
					{
						let at = std::mem::offset_of!(SP_DEVICE_INTERFACE_DETAIL_DATA_A, DevicePath);
						let bytes = unsafe {
							std::slice::from_raw_parts((det.as_ptr() as *const u8).add(at), det.len() * 8 - at)
						};
						let path = &bytes[..bytes.iter().position(|&c| c == 0).unwrap_or(bytes.len())];
						if path.len() < PATH_LEN {
							let low: Vec<u8> = path.iter().map(|c| c.to_ascii_lowercase()).collect();
							if low.windows(want.len()).any(|w| w == want.as_slice()) {
								found.push(path.to_vec());
							}
						}
					}
					i += 1;
				}
				unsafe { SetupDiDestroyDeviceInfoList(set) };
			}
			*lock(&scan.paths) = Some(found);
			for _ in 0..30 {
				if scan.stop.load(Ordering::SeqCst) {
					break;
				}
				unsafe { Sleep(100) };
			}
		}
	}

	/// A feature report read, tried up to `tries` times 20 ms apart
	fn get_feature_retry(h: HANDLE, buf: &mut [u8], id: u8, tries: i32) -> bool {
		for _ in 0..tries {
			buf.fill(0);
			buf[0] = id;
			if unsafe { HidD_GetFeature(h, buf.as_mut_ptr().cast(), buf.len() as u32) } != 0 {
				return true;
			}
			unsafe { Sleep(20) };
		}
		false
	}

	/// The protocol driver_vive has: feature report 0x10 starts the read, repeated 0x11 reads carry chunks, a chunk of
	/// no bytes ends it (or the device is configured); each through the tap's config state machine under the context
	/// lock, which inflates, parses and (live) saves the config
	fn fetch_config<C: Context>(ctx: &Mutex<C>, path: &CString, serial: &str, feature_len: u16, dev: DeviceId) {
		let h = unsafe {
			CreateFileA(
				path.as_ptr().cast(),
				GENERIC_READ | GENERIC_WRITE,
				FILE_SHARE_READ | FILE_SHARE_WRITE,
				null_mut(),
				OPEN_EXISTING,
				0,
				null_mut(),
			)
		};
		if h == INVALID_HANDLE_VALUE {
			let e = unsafe { GetLastError() };
			warn(ctx, &format!("hidtap: {serial}: cannot open for config fetch: error {e}"));
			return;
		}
		let mut buf = [0u8; 256];
		let flen = if feature_len != 0 { feature_len as usize } else { 64 }.min(buf.len());
		info(ctx, &format!("hidtap: fetching config for {serial}"));
		let mut ok = false;
		if get_feature_retry(h, &mut buf[..flen], 0x10, 50) {
			{
				let mut g = lock(ctx);
				let (tap, rt) = g.tap();
				tap.config_report(rt, dev, &buf[..flen]);
			}
			for _chunk in 0..4096 {
				if !get_feature_retry(h, &mut buf[..flen], 0x11, 50) {
					break;
				}
				{
					let mut g = lock(ctx);
					let (tap, rt) = g.tap();
					tap.config_report(rt, dev, &buf[..flen]);
					ok = tap.is_configured(dev);
				}
				if buf[1] == 0 || ok {
					break;
				}
			}
		}
		unsafe { CloseHandle(h) };
		if !ok {
			warn(
				ctx,
				&format!(
					"hidtap: config fetch for {serial} failed (tracker off, nothing paired, or Valve mid-fetch); retrying \
					 later"
				),
			);
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn path_fields_read_as_strtol_does() {
		let dongle = b"\\\\?\\hid#vid_28de&pid_2102#8&2cea1e2e&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}";
		assert_eq!(path_field(dongle, b"&mi_"), -1);
		assert_eq!(path_field(dongle, b"&col"), -1);
		let tracker =
			b"\\\\?\\hid#vid_28de&pid_2300&mi_02&col01#9&1c4b05&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}";
		assert_eq!(path_field(tracker, b"&mi_"), 2);
		assert_eq!(path_field(tracker, b"&col"), 1);
		// the key as it is written (a path in capitals has none), then strtol's hex: a 0x, a sign, its limits
		assert_eq!(path_field(b"x&MI_01", b"&mi_"), -1);
		assert_eq!(path_field(b"&mi_ff#", b"&mi_"), 255);
		assert_eq!(path_field(b"&mi_0x1A", b"&mi_"), 26);
		assert_eq!(path_field(b"&mi_#", b"&mi_"), 0);
		assert_eq!(path_field(b"&mi_ -1", b"&mi_"), -1);
		assert_eq!(path_field(b"&mi_7fffffff", b"&mi_"), i32::MAX);
		assert_eq!(path_field(b"&mi_80000000", b"&mi_"), i32::MAX);
		assert_eq!(path_field(b"&mi_-80000000", b"&mi_"), i32::MIN);
		assert_eq!(path_field(b"&mi_123456789abcdef0123", b"&mi_"), i32::MAX);
		// the first one
		assert_eq!(path_field(b"&mi_01&mi_02", b"&mi_"), 1);
	}

	#[test]
	fn serials_are_ascii() {
		let w: Vec<u16> = "EBF4140A47-4YX".encode_utf16().chain([0, 65, 66]).collect();
		assert_eq!(wide_to_ascii(&w, SERIAL_LEN), "EBF4140A47-4YX");
		assert_eq!(wide_to_ascii(&[0x41, 0xe9, 0x263a, 0x42, 0], SERIAL_LEN), "A??B");
		let long = vec![0x31u16; 128];
		assert_eq!(wide_to_ascii(&long, SERIAL_LEN).len(), SERIAL_LEN - 1);
		assert_eq!(wide_to_ascii(&[0], SERIAL_LEN), "");
	}

	#[test]
	fn reads_split_on_the_report_length() {
		let split = |got, len| reports_in(got, len).map(|r| (r.start, r.end)).collect::<Vec<_>>();
		assert_eq!(split(59, 59), [(0, 59)]);
		assert_eq!(split(118, 59), [(0, 59), (59, 118)]);
		assert_eq!(split(100, 59), [(0, 59), (59, 100)]);
		assert_eq!(split(30, 59), [(0, 30)]);
		assert_eq!(split(64, 0), [(0, 64)]);
		assert!(split(0, 59).is_empty());
		assert!(split(0, 0).is_empty());
	}
}
