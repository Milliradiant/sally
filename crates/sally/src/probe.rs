//! `sally probe`: the dongle check and the config cache seeder (docs/sally_design.md 4.1), the port of hidprobe.c, its
//! options and its output. The runtime reads the dongles itself (the live tap, `--hidtap 1`); this is for the two
//! things around it:
//! - checking that the dongles deliver input reports to a second reader, with or without SteamVR running (when the tap
//!   shows nothing, this says whether the problem is the dongle or the runtime);
//! - seeding the per-dongle config cache the tap loads (`--config`).
//!
//! ```text
//! sally probe               the HID interfaces of vendor 28de listed, opened shared and read for 5 s, their reports
//!                           counted
//! sally probe -t 20         read for 20 s
//! sally probe --config      each paired tracker's config (its sensor layout) read over its dongle and saved as
//!                           <dongle serial>_config.json in the current folder. With SteamVR CLOSED and the trackers
//!                           on: the read goes over the radio to the tracker and must not overlap Valve's own. Needed
//!                           once per dongle (again after re-pairing a tracker to another dongle)
//! sally probe --vid 28de    another vendor id (hex)
//! ```
//!
//! It reads only: input reports, and the config as feature reports (HidD_GetFeature, the protocol the live tap's
//! config fetch has); nothing is written to a device. The Windows API is sally-hid's (`live::sys`), the config
//! inflated as the live tap inflates it (`config_stream::simple_inflate`, zlib's checks and error codes where hidprobe
//! skipped the zlib header and inflated with puff). Windows only, as hidprobe was.

// what reads the dongles is Windows only
#![cfg_attr(not(windows), allow(dead_code))]

use sally_math::printf::{sprintf, Arg};

use crate::text;

/// The command line
#[derive(Debug, PartialEq)]
pub struct Options {
	pub vid: u16,
	pub seconds: i32,
	pub config: bool,
}

/// hidprobe's main's arguments: `-t <s>` (atoi), `--vid <hex>` (strtol, base 16), `--config`; anything else (or an
/// option without its value) is the argument it does not know
pub fn parse(args: &[String]) -> Result<Options, String> {
	let mut o = Options { vid: 0x28de, seconds: 5, config: false };
	let mut i = 0;
	while i < args.len() {
		match args[i].as_str() {
			"-t" if i + 1 < args.len() => {
				i += 1;
				o.seconds = sally_math::c_atoi(args[i].as_bytes());
			}
			"--vid" if i + 1 < args.len() => {
				i += 1;
				o.vid = sally_hid::live::strtol_hex(args[i].as_bytes()) as u16;
			}
			"--config" => o.config = true,
			other => return Err(other.to_string()),
		}
		i += 1;
	}
	Ok(o)
}

/// parse_path_field: the number (strtol, base 16) after the first `key` in a device path that has two characters
/// after it ("&mi_01" is interface 1, "&col02" collection 2); -1 where there is none
pub fn path_field(path: &[u8], key: &[u8]) -> i32 {
	let mut from = 0;
	while let Some(at) = path[from..].windows(key.len().max(1)).position(|w| w == key).map(|p| from + p) {
		let p = at + key.len();
		if p + 1 < path.len() {
			return sally_hid::live::strtol_hex(&path[p..]);
		}
		from = p.max(at + 1);
		if from >= path.len() {
			break;
		}
	}
	-1
}

/// How many first reads of a device are shown (MAX_PENDING_SHOW)
const SHOW_READS: i32 = 3;

/// A device's reads counted (got_report): reads, bytes, the reports in them; its first reads shown and its first
/// three reads that are not whole reports
#[derive(Debug, Default, PartialEq)]
pub struct Counts {
	pub reports: u64,
	pub reads: u64,
	pub bytes: u64,
	shown: i32,
	errors: i32,
}

impl Counts {
	/// A read of `got` bytes (`buf`, its first bytes at least) on a device with reports of `input_len` bytes (0: not
	/// known, the read is one): counted, and what hidprobe printed for it
	pub fn got(&mut self, serial: &str, mi: i32, input_len: u16, buf: &[u8], got: u32) -> String {
		self.reads += 1;
		self.bytes += got as u64;
		let rlen = if input_len != 0 { input_len as u32 } else { got };
		let mut out = String::new();
		// a read of nothing on an interface without its lengths: nothing to split
		if rlen == 0 {
			return out;
		}
		if self.shown < SHOW_READS {
			self.shown += 1;
			out += &sprintf(
				"  %s mi%d: read %lu bytes = %lu reports:",
				&[Arg::S(serial), Arg::I(mi as i64), Arg::U(got as u64), Arg::U((got / rlen) as u64)],
			);
			for b in buf.iter().take((got as usize).min(24)) {
				out += &format!(" {b:02x}");
			}
			out += if got > 24 { " ...\n" } else { "\n" };
		}
		if !got.is_multiple_of(rlen) {
			let shown = self.errors < 3;
			self.errors += 1;
			if shown {
				out += &sprintf(
					"  %s: read of %lu bytes is not a multiple of the %lu-byte report\n",
					&[Arg::S(serial), Arg::U(got as u64), Arg::U(rlen as u64)],
				);
			}
		}
		self.reports += sally_hid::live::reports_in(got as usize, rlen as usize).count() as u64;
		out
	}
}

/// The file a dongle's config is saved in, in the current folder (snprintf into 128 bytes)
pub fn config_file_name(serial: &str) -> String {
	let mut f = format!("{serial}_config.json");
	while f.len() > 127 {
		f.pop();
	}
	f
}

/// The tracker's serial in a config: after the first "device_serial_number" key, the next quoted text (63 characters at
/// most), "?" without one
pub fn tracker_serial(json: &[u8]) -> String {
	// the JSON as a C string, up to its first 0
	let json = &json[..json.iter().position(|&c| c == 0).unwrap_or(json.len())];
	let key = b"\"device_serial_number\"";
	let Some(at) = json.windows(key.len()).position(|w| w == key) else { return "?".into() };
	let rest = &json[at + key.len()..];
	let Some(q) = rest.iter().position(|&c| c == b'"') else { return "?".into() };
	let value: Vec<u8> = rest[q + 1..].iter().copied().take_while(|&c| c != b'"').take(63).collect();
	if value.is_empty() {
		"?".into()
	} else {
		String::from_utf8_lossy(&value).into_owned()
	}
}

/// vrserver_running's answer, as hidprobe printed it
fn vr_state(vr: i32) -> &'static str {
	match vr {
		1 => "RUNNING",
		0 => "NOT running",
		_ => "state unknown",
	}
}

pub fn main(args: &[String]) -> i32 {
	let options = match parse(args) {
		Ok(o) => o,
		Err(arg) => {
			text::out(&format!("unknown arg {arg}\n"));
			return 1;
		}
	};
	run(&options)
}

#[cfg(not(windows))]
fn run(_: &Options) -> i32 {
	text::err("sally probe: it reads the dongles through Windows' HID, so it needs Windows\n");
	1
}

#[cfg(windows)]
fn run(options: &Options) -> i32 {
	win::run(options)
}

/// The dongles, through Windows' HID (hidprobe.c's Windows half)
#[cfg(windows)]
mod win {
	use std::ffi::CString;
	use std::ptr::{null, null_mut};

	use sally_hid::config_stream::{simple_inflate, MAX_CONFIG_LEN};
	use sally_hid::live::sys::*;
	use sally_hid::live::wide_to_ascii;
	use sally_math::printf::{sprintf, Arg};

	use super::{config_file_name, path_field, tracker_serial, vr_state, Counts, Options};
	use crate::text::out;

	const MAX_DEVS: usize = 64;
	/// dev_t's buffers: the path, the serial and the product (char[512], char[64], char[64])
	const PATH_LEN: usize = 512;
	const NAME_LEN: usize = 64;
	/// One read's buffer (unsigned char buf[1024])
	const READ_LEN: usize = 1024;

	/// A read's OVERLAPPED and buffer, on the heap so they stay where the system writes them while the read is pending
	struct Io {
		ov: OVERLAPPED,
		buf: [u8; READ_LEN],
	}

	/// dev_t: an interface found, and its reading
	struct Dev {
		path: CString,
		pid: u16,
		mi: i32,
		col: i32,
		serial: String,
		product: String,
		usage_page: u16,
		usage: u16,
		input_len: u16,
		output_len: u16,
		feature_len: u16,
		h: HANDLE,
		/// how the handle was got ("FAILED" when it was not); None before an open is tried
		open_mode: Option<&'static str>,
		open_error: DWORD,
		io: Box<Io>,
		counts: Counts,
	}

	/// vrserver_running: 1 when vrserver.exe is in the process list, 0 when not, -1 when the list cannot be had
	fn vrserver_running() -> i32 {
		// SAFETY: a snapshot of the processes, closed below
		let snap = unsafe { CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0) };
		if snap == INVALID_HANDLE_VALUE {
			return -1;
		}
		// SAFETY: PROCESSENTRY32 is plain data; zero is a valid value of it
		let mut pe: PROCESSENTRY32 = unsafe { std::mem::zeroed() };
		pe.dwSize = std::mem::size_of::<PROCESSENTRY32>() as DWORD;
		let mut found = false;
		// SAFETY: the calls fill in the entry they are given, of the size it says
		if unsafe { Process32First(snap, &mut pe) } != 0 {
			loop {
				let name = &pe.szExeFile[..pe.szExeFile.iter().position(|&c| c == 0).unwrap_or(MAX_PATH)];
				// _stricmp
				found = name.eq_ignore_ascii_case(b"vrserver.exe");
				// SAFETY: as above
				if found || unsafe { Process32Next(snap, &mut pe) } == 0 {
					break;
				}
			}
		}
		// SAFETY: the snapshot's handle
		unsafe { CloseHandle(snap) };
		found as i32
	}

	/// werr: a system error's text, on one line (empty where Windows has none)
	fn werr(e: DWORD) -> String {
		let mut msg = [0u8; 256];
		// SAFETY: the call writes at most the buffer's size into it
		let n = unsafe {
			FormatMessageA(
				FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				null(),
				e,
				0,
				msg.as_mut_ptr(),
				msg.len() as DWORD,
				null_mut(),
			)
		};
		let text = &msg[..(n as usize).min(msg.len())];
		let end = text.iter().position(|&c| c == 0 || c == b'\r' || c == b'\n').unwrap_or(text.len());
		String::from_utf8_lossy(&text[..end]).into_owned()
	}

	fn last_error() -> DWORD {
		// SAFETY: no arguments
		unsafe { GetLastError() }
	}

	/// enumerate: the HID interfaces of the vendor id (64 at most), each opened for its metadata only (no read or write
	/// access, shared with everyone): its ids, interface and collection, serial ("-" without one), product, usage and
	/// report lengths
	fn enumerate(want_vid: u16) -> Vec<Dev> {
		let mut devs = Vec::new();
		let mut guid = GUID::default();
		// SAFETY: the call writes the GUID it is given
		unsafe { HidD_GetHidGuid(&mut guid) };
		// SAFETY: a device information set, destroyed below
		let set = unsafe { SetupDiGetClassDevsA(&guid, null(), null_mut(), DIGCF_PRESENT | DIGCF_DEVICEINTERFACE) };
		if set == INVALID_HANDLE_VALUE {
			out(&format!("SetupDiGetClassDevs failed: {}\n", werr(last_error())));
			return devs;
		}
		let mut ifd = SP_DEVICE_INTERFACE_DATA {
			cbSize: std::mem::size_of::<SP_DEVICE_INTERFACE_DATA>() as DWORD,
			InterfaceClassGuid: GUID::default(),
			Flags: 0,
			Reserved: 0,
		};
		let mut i: DWORD = 0;
		// SAFETY: the set's interfaces in turn, into the structure given
		while unsafe { SetupDiEnumDeviceInterfaces(set, null_mut(), &guid, i, &mut ifd) } != 0 {
			i += 1;
			let mut need: DWORD = 0;
			// SAFETY: asks only for the size the detail needs
			unsafe { SetupDiGetDeviceInterfaceDetailA(set, &mut ifd, null_mut(), 0, &mut need, null_mut()) };
			// calloc(1, need + 8), aligned for the structure
			let mut det = vec![0u64; (need as usize + 8).div_ceil(8)];
			let p = det.as_mut_ptr() as *mut SP_DEVICE_INTERFACE_DETAIL_DATA_A;
			// SAFETY: the buffer holds the structure (8 bytes at least)
			unsafe { (*p).cbSize = std::mem::size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_A>() as DWORD };
			// SAFETY: the buffer is `need` bytes and more
			if unsafe { SetupDiGetDeviceInterfaceDetailA(set, &mut ifd, p, need, null_mut(), null_mut()) } == 0 {
				continue;
			}
			let at = std::mem::offset_of!(SP_DEVICE_INTERFACE_DETAIL_DATA_A, DevicePath);
			// SAFETY: the buffer's bytes from the path on
			let bytes = unsafe { std::slice::from_raw_parts((det.as_ptr() as *const u8).add(at), det.len() * 8 - at) };
			// strncpy into char[512]: 511 bytes at most
			let path = &bytes[..bytes.iter().position(|&c| c == 0).unwrap_or(bytes.len()).min(PATH_LEN - 1)];
			let Ok(cpath) = CString::new(path) else { continue };
			// SAFETY: a metadata handle on a NUL-terminated path, closed below
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
			let mut a = HIDD_ATTRIBUTES { Size: std::mem::size_of::<HIDD_ATTRIBUTES>() as u32, ..Default::default() };
			// SAFETY: the call writes the attributes it is given
			if unsafe { HidD_GetAttributes(h, &mut a) } != 0 && a.VendorID == want_vid && devs.len() < MAX_DEVS {
				let mut d = Dev {
					mi: path_field(cpath.as_bytes(), b"&mi_"),
					col: path_field(cpath.as_bytes(), b"&col"),
					path: cpath,
					pid: a.ProductID,
					serial: "-".into(),
					product: String::new(),
					usage_page: 0,
					usage: 0,
					input_len: 0,
					output_len: 0,
					feature_len: 0,
					h: INVALID_HANDLE_VALUE,
					open_mode: None,
					open_error: 0,
					io: Box::new(Io {
						ov: OVERLAPPED { Internal: 0, InternalHigh: 0, Offset: 0, OffsetHigh: 0, hEvent: null_mut() },
						buf: [0; READ_LEN],
					}),
					counts: Counts::default(),
				};
				let mut w = [0u16; 128];
				// SAFETY: the calls write at most the buffer's size in bytes
				if unsafe { HidD_GetSerialNumberString(h, w.as_mut_ptr().cast(), std::mem::size_of_val(&w) as u32) }
					!= 0
				{
					d.serial = wide_to_ascii(&w, NAME_LEN);
				}
				w[0] = 0;
				// SAFETY: as above
				if unsafe { HidD_GetProductString(h, w.as_mut_ptr().cast(), std::mem::size_of_val(&w) as u32) } != 0 {
					d.product = wide_to_ascii(&w, NAME_LEN);
				}
				let mut pp: PHIDP_PREPARSED_DATA = null_mut();
				// SAFETY: the preparsed data, freed below
				if unsafe { HidD_GetPreparsedData(h, &mut pp) } != 0 {
					let mut caps = HIDP_CAPS::default();
					// SAFETY: the call writes the caps it is given
					if unsafe { HidP_GetCaps(pp, &mut caps) } == HIDP_STATUS_SUCCESS {
						d.usage_page = caps.UsagePage;
						d.usage = caps.Usage;
						d.input_len = caps.InputReportByteLength;
						d.output_len = caps.OutputReportByteLength;
						d.feature_len = caps.FeatureReportByteLength;
					}
					// SAFETY: the preparsed data got above
					unsafe { HidD_FreePreparsedData(pp) };
				}
				devs.push(d);
			}
			// SAFETY: the metadata handle
			unsafe { CloseHandle(h) };
		}
		// SAFETY: the set made above
		unsafe { SetupDiDestroyDeviceInfoList(set) };
		devs
	}

	/// open_shared: a handle for overlapped reads, shared, read and write access asked first, read only then
	fn open_shared(d: &mut Dev) {
		for (name, access) in [("read+write shared", GENERIC_READ | GENERIC_WRITE), ("read-only shared", GENERIC_READ)]
		{
			// SAFETY: a NUL-terminated path; the handle is closed at the end of the reading
			d.h = unsafe {
				CreateFileA(
					d.path.as_ptr().cast(),
					access,
					FILE_SHARE_READ | FILE_SHARE_WRITE,
					null_mut(),
					OPEN_EXISTING,
					FILE_FLAG_OVERLAPPED,
					null_mut(),
				)
			};
			if d.h != INVALID_HANDLE_VALUE {
				d.open_mode = Some(name);
				return;
			}
			d.open_error = last_error();
		}
		d.open_mode = Some("FAILED");
	}

	/// got_report: a read counted, what it prints printed
	fn got_report(d: &mut Dev, got: DWORD) {
		let shown = d.counts.got(&d.serial, d.mi, d.input_len, &d.io.buf, got);
		out(&shown);
	}

	/// start_read: reads issued while they complete at once (reports already queued), one left pending; one that fails
	/// closes the device
	fn start_read(d: &mut Dev) {
		loop {
			let ev = if d.io.ov.hEvent.is_null() {
				// SAFETY: a manual-reset event, not signaled
				unsafe { CreateEventA(null_mut(), TRUE, FALSE, null()) }
			} else {
				d.io.ov.hEvent
			};
			d.io.ov = OVERLAPPED { Internal: 0, InternalHigh: 0, Offset: 0, OffsetHigh: 0, hEvent: ev };
			// SAFETY: the event made above
			unsafe { ResetEvent(ev) };
			let mut got: DWORD = 0;
			let io = &mut *d.io;
			// SAFETY: the buffer and the OVERLAPPED live on the heap until the read is done or cancelled
			if unsafe { ReadFile(d.h, io.buf.as_mut_ptr().cast(), READ_LEN as DWORD, &mut got, &mut io.ov) } != 0 {
				got_report(d, got);
				continue;
			}
			let e = last_error();
			if e == ERROR_IO_PENDING {
				return;
			}
			out(&format!("  {}: ReadFile failed: {}\n", d.serial, werr(e)));
			// SAFETY: the device's handle (no read of it is pending)
			unsafe { CloseHandle(d.h) };
			d.h = INVALID_HANDLE_VALUE;
			return;
		}
	}

	/// read_loop: every opened device read for `seconds`, the reads waited on 200 ms at a time; then the counts per
	/// device, with whether SteamVR runs
	fn read_loop(devs: &mut [Dev], seconds: i32) {
		let t0 = sally_os::absolute_time_us();
		let mut events: Vec<HANDLE> = Vec::new();
		let mut by_event: Vec<usize> = Vec::new();
		for (i, d) in devs.iter_mut().enumerate() {
			if d.h == INVALID_HANDLE_VALUE {
				continue;
			}
			start_read(d);
			if d.h == INVALID_HANDLE_VALUE {
				continue;
			}
			events.push(d.io.ov.hEvent);
			by_event.push(i);
		}
		if events.is_empty() {
			out("nothing opened, nothing to read\n");
			return;
		}
		out(&format!("reading {} handles for {seconds} s...\n", events.len()));
		// (unsigned long long)seconds * 1000000, as the C wraps it
		let end = t0.wrapping_add((seconds as i64 as u64).wrapping_mul(1_000_000));
		while sally_os::absolute_time_us() < end {
			// SAFETY: the devices' events, which stay open until the end
			let r = unsafe { WaitForMultipleObjects(events.len() as DWORD, events.as_ptr(), FALSE, 200) };
			if r == WAIT_TIMEOUT {
				continue;
			}
			let idx = r.wrapping_sub(WAIT_OBJECT_0) as i32;
			if idx < 0 || idx as usize >= events.len() {
				break;
			}
			let d = &mut devs[by_event[idx as usize]];
			let mut got: DWORD = 0;
			// SAFETY: the read's own OVERLAPPED
			if unsafe { GetOverlappedResult(d.h, &mut d.io.ov, &mut got, FALSE) } != 0 {
				got_report(d, got);
			} else {
				let e = last_error();
				if e == ERROR_IO_INCOMPLETE {
					// a spurious wake, the read still pending: keep waiting on it
					// SAFETY: the read's event
					unsafe { ResetEvent(d.io.ov.hEvent) };
					continue;
				}
				let shown = d.counts.read_error();
				if shown {
					out(&format!("  {} mi{}: read error: {}\n", d.serial, d.mi, werr(e)));
				}
				if e == ERROR_DEVICE_NOT_CONNECTED || e == ERROR_OPERATION_ABORTED {
					continue;
				}
			}
			start_read(d);
		}
		for d in devs.iter_mut() {
			if d.h != INVALID_HANDLE_VALUE {
				// SAFETY: the device's handle; its cancelled read's memory is kept until the system is done with it
				unsafe {
					CancelIo(d.h);
					if !d.io.ov.hEvent.is_null() && WaitForSingleObject(d.io.ov.hEvent, 1000) != WAIT_OBJECT_0 {
						std::mem::forget(std::mem::replace(
							&mut d.io,
							Box::new(Io {
								ov: OVERLAPPED {
									Internal: 0,
									InternalHigh: 0,
									Offset: 0,
									OffsetHigh: 0,
									hEvent: null_mut(),
								},
								buf: [0; READ_LEN],
							}),
						));
					}
					CloseHandle(d.h);
				}
			}
		}
		let secs = sally_os::absolute_time_us().wrapping_sub(t0) as f64 / 1e6;
		let vr = vrserver_running();
		out(&sprintf(
			"\nreports received in %.1f s (SteamVR vrserver.exe %s):\n",
			&[Arg::F(secs), Arg::S(vr_state(vr))],
		));
		for d in devs.iter() {
			if let Some(mode) = d.open_mode.filter(|m| *m != "FAILED") {
				out(&sprintf(
					"  %-16s pid %04x mi %2d col %2d  %8llu reports  %7.1f /s  (%llu reads)  %s\n",
					&[
						Arg::S(&d.serial),
						Arg::U(d.pid as u64),
						Arg::I(d.mi as i64),
						Arg::I(d.col as i64),
						Arg::U(d.counts.reports),
						Arg::F(d.counts.reports as f64 / secs),
						Arg::U(d.counts.reads),
						Arg::S(mode),
					],
				));
			}
		}
	}

	/// REPORT_CONFIG_READMODE and REPORT_CONFIG_READ: the feature reports that start a config read and carry its chunks
	const REPORT_CONFIG_READMODE: u8 = 0x10;
	const REPORT_CONFIG_READ: u8 = 0x11;

	/// get_feature_retry: a feature report read, tried up to `tries` times 20 ms apart
	fn get_feature_retry(h: HANDLE, buf: &mut [u8], id: u8, tries: i32) -> bool {
		for _ in 0..tries {
			buf.fill(0);
			buf[0] = id;
			// SAFETY: the call writes at most the buffer's length
			if unsafe { HidD_GetFeature(h, buf.as_mut_ptr().cast(), buf.len() as u32) } != 0 {
				return true;
			}
			// SAFETY: a number of milliseconds
			unsafe { Sleep(20) };
		}
		false
	}

	/// fetch_config: libsurvive's driver_vive protocol, reads only: feature 0x10 answers with the compressed length,
	/// repeated 0x11 reads return chunks (a count byte, the bytes), a chunk of none ends it. The zlib stream inflated,
	/// saved as <serial>_config.json in the current folder. Whether it was saved
	fn fetch_config(d: &Dev) -> bool {
		// the report's bytes, and two past the 256 a count byte can point at (zeros: never read into)
		let mut buf = [0u8; 258];
		let flen = if d.feature_len != 0 { d.feature_len as usize } else { 64 }.min(256);
		if !get_feature_retry(d.h, &mut buf[..flen], REPORT_CONFIG_READMODE, 50) {
			out(&format!("  {}: readmode request failed: {}\n", d.serial, werr(last_error())));
			return false;
		}
		let expected = ((buf[1] as u32) << 8) | buf[2] as u32;
		let mut blob = vec![0u8; 65536];
		let mut got = 0usize;
		for _chunk in 0..4096 {
			if !get_feature_retry(d.h, &mut buf[..flen], REPORT_CONFIG_READ, 50) {
				out(&format!("  {}: chunk read failed after {got} bytes: {}\n", d.serial, werr(last_error())));
				return false;
			}
			let size = buf[1] as usize;
			if size == 0 || got + size > 65536 {
				break;
			}
			blob[got..got + size].copy_from_slice(&buf[2..2 + size]);
			got += size;
		}
		if got == 0 || (got as u32) + 2 < expected {
			out(&format!(
				"  {}: got {got} bytes, expected {expected} (tracker off, not paired, or Valve mid-fetch?)\n",
				d.serial
			));
			return false;
		}
		let json = match simple_inflate(&blob[..got], MAX_CONFIG_LEN) {
			Ok(j) => j,
			Err(e) => {
				out(&format!("  {}: inflate failed ({}) on {got} bytes\n", d.serial, e.code));
				return false;
			}
		};
		let fname = config_file_name(&d.serial);
		let Ok(mut f) = std::fs::File::create(&fname) else {
			out(&format!("  {}: cannot write {fname}\n", d.serial));
			return false;
		};
		// (the write's own result is not checked, as in the C)
		let _ = std::io::Write::write_all(&mut f, &json);
		drop(f);
		out(&sprintf(
			"  %-16s -> %s  (%lu bytes of config, tracker %s)\n",
			&[Arg::S(&d.serial), Arg::S(&fname), Arg::U(json.len() as u64), Arg::S(&tracker_serial(&json))],
		));
		true
	}

	/// fetch_configs: refused while SteamVR runs; each interface opened (read and write access, which reading feature
	/// reports needs) and its config fetched
	fn fetch_configs(devs: &mut [Dev]) {
		if vrserver_running() == 1 {
			out("SteamVR is running. Close it first: the config fetch goes over the radio to the trackers and must not\n\
			     run at the same time as Valve's own fetch.\n");
			return;
		}
		let mut ok = 0;
		for d in devs.iter_mut() {
			// SAFETY: a NUL-terminated path; the handle is closed below
			d.h = unsafe {
				CreateFileA(
					d.path.as_ptr().cast(),
					GENERIC_READ | GENERIC_WRITE,
					FILE_SHARE_READ | FILE_SHARE_WRITE,
					null_mut(),
					OPEN_EXISTING,
					0,
					null_mut(),
				)
			};
			if d.h == INVALID_HANDLE_VALUE {
				out(&format!("  {}: open failed: {}\n", d.serial, werr(last_error())));
				continue;
			}
			ok += fetch_config(d) as i32;
			// SAFETY: the handle opened above
			unsafe { CloseHandle(d.h) };
			d.h = INVALID_HANDLE_VALUE;
		}
		out(&format!(
			"{ok} of {} configs saved. A dongle with nothing paired or its tracker off fails, that is expected.\n",
			devs.len()
		));
	}

	pub fn run(o: &Options) -> i32 {
		out(&format!("SteamVR vrserver.exe: {}\n", vr_state(vrserver_running())));
		let mut devs = enumerate(o.vid);
		out(&sprintf("%d HID interfaces with vid %04x:\n", &[Arg::I(devs.len() as i64), Arg::U(o.vid as u64)]));
		for d in devs.iter_mut() {
			if !o.config {
				open_shared(d);
			}
			out(&sprintf(
				"  %-16s pid %04x mi %2d col %2d  usage %04x:%04x  in/out/feat %3u/%3u/%3u  %-20s  %s\n",
				&[
					Arg::S(&d.serial),
					Arg::U(d.pid as u64),
					Arg::I(d.mi as i64),
					Arg::I(d.col as i64),
					Arg::U(d.usage_page as u64),
					Arg::U(d.usage as u64),
					Arg::U(d.input_len as u64),
					Arg::U(d.output_len as u64),
					Arg::U(d.feature_len as u64),
					Arg::S(&d.product),
					Arg::S(if o.config { "" } else { d.open_mode.unwrap_or("") }),
				],
			));
			if !o.config && d.h == INVALID_HANDLE_VALUE {
				out(&format!("      open failed: {}\n", werr(d.open_error)));
			}
		}
		if devs.is_empty() {
			out("no devices; are the dongles plugged in?\n");
			return 1;
		}
		if o.config {
			fetch_configs(&mut devs);
			return 0;
		}
		read_loop(&mut devs, o.seconds);
		out("\nA dongle with a tracker that is on and paired delivers ~500 reports/s, with or without SteamVR running.\n\
		     Zero on every dongle: trackers off or not paired. Zero only while SteamVR runs: Windows stopped sharing\n\
		     the reports with a second reader, and the tap cannot work that way.\n");
		0
	}
}

impl Counts {
	/// A read that failed: whether hidprobe printed it (its first three errors)
	pub fn read_error(&mut self) -> bool {
		let shown = self.errors < 3;
		self.errors += 1;
		shown
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn args(a: &[&str]) -> Vec<String> {
		a.iter().map(|s| s.to_string()).collect()
	}

	#[test]
	fn its_options_are_hidprobes() {
		assert_eq!(parse(&args(&[])), Ok(Options { vid: 0x28de, seconds: 5, config: false }));
		assert_eq!(parse(&args(&["-t", "20", "--config"])), Ok(Options { vid: 0x28de, seconds: 20, config: true }));
		assert_eq!(parse(&args(&["--vid", "0x0BB4"])).unwrap().vid, 0x0bb4);
		assert_eq!(parse(&args(&["--vid", "28de"])).unwrap().vid, 0x28de);
		// strtol's limit, cut to 16 bits; atoi's "12x"
		assert_eq!(parse(&args(&["--vid", "123456789"])).unwrap().vid, 0xffff);
		assert_eq!(parse(&args(&["-t", "12x"])).unwrap().seconds, 12);
		assert_eq!(parse(&args(&["-t"])), Err("-t".into()));
		assert_eq!(parse(&args(&["--vid"])), Err("--vid".into()));
		assert_eq!(parse(&args(&["-t", "3", "x"])), Err("x".into()));
	}

	#[test]
	fn path_fields_as_hidprobe_reads_them() {
		let tracker =
			b"\\\\?\\hid#vid_28de&pid_2300&mi_02&col01#9&1c4b05&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}";
		assert_eq!(path_field(tracker, b"&mi_"), 2);
		assert_eq!(path_field(tracker, b"&col"), 1);
		assert_eq!(path_field(b"\\\\?\\hid#vid_28de&pid_2102#8&2cea1e2e&0&0000", b"&mi_"), -1);
		// a key with fewer than two characters after it is passed over, for a later one
		assert_eq!(path_field(b"x&mi_0", b"&mi_"), -1);
		assert_eq!(path_field(b"x&mi_", b"&mi_"), -1);
		assert_eq!(path_field(b"&mi_1&mi_02", b"&mi_"), 1);
		assert_eq!(path_field(b"&mi_ff#", b"&mi_"), 255);
	}

	#[test]
	fn reads_are_counted_and_shown_as_hidprobe_did() {
		let mut c = Counts::default();
		let buf: Vec<u8> = (0..118u8).collect();
		assert_eq!(
			c.got("EBF4140A47-4YX", -1, 59, &buf, 118),
			"  EBF4140A47-4YX mi-1: read 118 bytes = 2 reports: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 \
			 12 13 14 15 16 17 ...\n"
		);
		assert_eq!((c.reports, c.reads, c.bytes), (2, 1, 118));
		// not whole reports: a third report counted for the rest, the first three said
		assert_eq!(
			c.got("x", 0, 59, &buf, 100),
			"  x mi0: read 100 bytes = 1 reports: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 \
			 17 ...\n  x: read of 100 bytes is not a multiple of the 59-byte report\n"
		);
		assert_eq!(c.reports, 4);
		assert_eq!(c.got("x", 0, 59, &buf, 20), "  x mi0: read 20 bytes = 0 reports: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13\n  x: read of 20 bytes is not a multiple of the 59-byte report\n");
		// shown three times only; a read error counts with the bad reads (the third of them still said)
		assert_eq!(c.got("x", 0, 59, &buf, 59), "");
		assert!(c.read_error());
		assert_eq!(c.got("x", 0, 59, &buf, 10), "");
		assert_eq!((c.reports, c.reads, c.bytes), (7, 5, 307));
		// a read of nothing without report lengths: counted as a read, nothing else
		let mut c = Counts::default();
		assert_eq!(c.got("x", 0, 0, &buf, 0), "");
		assert_eq!((c.reports, c.reads), (0, 1));
		// without report lengths a read is one report
		assert_eq!(
			c.got("x", 0, 0, &buf, 30),
			"  x mi0: read 30 bytes = 1 reports: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 \
			 ...\n"
		);
		assert_eq!(c.reports, 1);
	}

	#[test]
	fn the_config_file_and_its_tracker() {
		assert_eq!(config_file_name("EBF4140A47-4YX"), "EBF4140A47-4YX_config.json");
		assert_eq!(config_file_name(&"a".repeat(200)).len(), 127);
		let cfg =
			b"{\r\n\t\"device_class\": \"generic_tracker\",\r\n\t\"device_serial_number\": \"LHR-1B80952C\",\r\n}";
		assert_eq!(tracker_serial(cfg), "LHR-1B80952C");
		assert_eq!(tracker_serial(b"{\"device_serial_number\": \"\"}"), "?");
		assert_eq!(tracker_serial(b"{\"x\": 1}"), "?");
		assert_eq!(tracker_serial(b"{\"x\": 1}\0\"device_serial_number\": \"LHR-1\""), "?");
		// a value that is not text: the next quoted thing, as hidprobe's strchr found it
		assert_eq!(tracker_serial(b"{\"device_serial_number\": 5, \"next\": 1}"), "next");
		assert_eq!(
			tracker_serial(format!("{{\"device_serial_number\": \"{}\"}}", "9".repeat(80)).as_bytes()).len(),
			63
		);
	}
}
