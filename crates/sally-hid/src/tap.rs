//! The HID tap's driver, shared by its two inputs, the replay of a recording (`replay`) and the live reads on the
//! dongles (`live`): a device is created as it is first seen (numbered by type in that order) and configured from its
//! cached `<serial>_config.json` or the config stream it sends, its reports go to its parser, and its light packets are
//! watched for the lightcap mode Valve's driver has put it in. Live, a config that streams in is saved as
//! `<serial>_config.json` for the next start.
//!
//! A dongle's cached config is only a guess at what it carries: devices get paired to other dongles. Which device is on
//! which dongle is SteamVR's to say (the driver passes it on with the protocol's `pair`, a recording has it as `A`
//! lines): `Tap::assign` gives the dongle's object that device's own config, and takes it from any other dongle whose
//! object had it.
//!
//! The runtime the tap feeds is a `Runtime`: it owns the tracked objects and runs each device's parser with its own
//! `report::Host`. `Record` is the `--hidtap-record` file, written by either input.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::Arc;

use crate::config_stream::{Chunk, ConfigStream};
use crate::devices::{self, usb_if, DEVICES};
use crate::line;
use crate::pack::{self, PackWriter};
use crate::replay::PlaybackTime;
use crate::report::{self, Globals, Parser, BUFFER_LEN};

/// VIVE_DEVICE_INST_MAX: the most devices a tap can have
pub const MAX_DEVICES: usize = 32;

/// sally: the dongles SteamVR's word is kept for (`assign`: the latest, so that a client naming made-up ones fills
/// nothing)
pub const MAX_PAIRS: usize = 64;

/// HIDTAP_MODE_VOTES: light packets in a row that must look like one mode before it is assumed
const MODE_VOTES: i8 = 6;

/// SURVIVE_ERROR_INVALID_CONFIG, the error the tap raises when it cannot start
pub const SURVIVE_ERROR_INVALID_CONFIG: i32 = -4;

/// A tracked object of the runtime's
pub type ObjectId = usize;

/// One of the tap's devices (a vive_device_inst_t): its index, in the order they were seen
pub type DeviceId = usize;

/// The tap's settings, from the options of the same names
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
	/// hidtap-allow-fs-config: try `<serial>_config.json` when no config stream is seen
	pub allow_fs_read: bool,
	/// hidtap-vid: the vendor id the devices are looked up (and, live, found) with
	pub vid: u16,
	/// hidtap-lightcap-mode: -2 detect it from the light packets, 0/1/2 fixed, -1 leave it
	pub lightcap_mode: i32,
	/// hidtap-config-fetch: live, fetch a device's config over feature reports when none is cached
	pub config_fetch: bool,
	/// hidtap-config-delay: live, the seconds a device must have been open before its config is fetched
	pub config_delay: f64,
	/// playback-factor: the replay's speed against real time (0 = as fast as it goes)
	pub playback_factor: f64,
	/// run-time: stop after this much recording time (s); 0 or less = the whole recording
	pub run_time: f64,
	/// vive-test-drop: TEST ONLY, "T0,T1,CODENAME,FRACTION" drops that share of a device's reports between run times
	/// T0 and T1, as a bad radio link would. Empty = off
	pub test_drop: String,
	/// hidtap-record-strip: live, the recording is written without the bytes of a dongle's report that nothing reads
	/// (`crate::strip`)
	pub record_strip: bool,
	/// hidtap-past-end: what the parser does with a packet it would read past the end of
	/// (`report::Globals::past_end`; the runtime sets 2, the reading stopped there)
	pub past_end: u8,
}

impl Default for Options {
	/// The options' defaults
	fn default() -> Self {
		Options {
			allow_fs_read: false,
			vid: 0x28de,
			lightcap_mode: -2,
			config_fetch: true,
			config_delay: 5.0,
			playback_factor: 1.0,
			run_time: -1.0,
			test_drop: String::new(),
			record_strip: false,
			past_end: 0,
		}
	}
}

/// enum hidtap_mode: where the reports come from
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
	/// --hidtap 1: the dongles themselves (Windows)
	Live,
	/// --hidtap-playback: a recording
	Playback,
}

/// What the tap needs from the runtime it feeds
pub trait Runtime {
	/// survive_create_device and survive_add_object: a tracked object for the device `name` ("KN0") on the dongle
	/// `serial`; None when the name is blacklisted
	fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId>;
	/// survive_load_htc_config_format_from_file: the object's config from a file; false when the file does not open
	fn load_config_file(&mut self, obj: ObjectId, filename: &str) -> bool;
	/// ctx->configproc: a config streamed from the device (None when it did not inflate: the C passes a length of -1);
	/// 0 when the object took it
	fn config(&mut self, obj: ObjectId, json: Option<Vec<u8>>) -> i32;
	/// survive_data_cb_locked: one report through the object's parser, with the runtime's report::Host for it
	#[allow(clippy::too_many_arguments)]
	fn report(
		&mut self, obj: ObjectId, parser: &mut Parser, globals: &mut Globals, time_us: u64, iface: i32,
		buffer: &[u8; BUFFER_LEN], len: usize,
	);
	/// survive_data_on_setup_write: a write to the device, through the parser's bookkeeping
	fn setup_write(
		&mut self, obj: ObjectId, parser: &mut Parser, bm_request_type: u8, b_request: u8, w_value: u16, data: &[u8],
	);
	/// survive_usb_feature_read: a feature report read from the device
	fn feature_read(&mut self, obj: ObjectId, parser: &Parser, data: &[u8]);
	/// survive_install_run_time_fn: the run clock from now on the playback's time (hidtap_run_time), or with None
	/// frozen at its last value if a clock was installed (static_time), as hidtap_close leaves it
	fn install_run_time_fn(&mut self, time: Option<Arc<PlaybackTime>>);
	fn run_time(&self) -> f64;
	/// SV_INFO, SV_WARN
	fn info(&mut self, text: &str);
	fn warn(&mut self, text: &str);
	/// SV_ERROR: the error raised, and its text logged
	fn error(&mut self, code: i32, text: &str);
	/// sally: the device `device` (its serial) is the one on the object's dongle: its own config into the object, the
	/// object started over first if it had another device's (`assign`). A runtime that cannot find configs by device
	/// keeps the default and fails
	fn identify(&mut self, _obj: ObjectId, _device: &str) -> Identified {
		Identified::Failed
	}
	/// sally: the object's device is not on its dongle any more: the object starts over without a config
	fn release(&mut self, _obj: ObjectId) {}
	/// sally: the serial of the device the object's config is for, None before it has one
	fn object_serial(&self, _obj: ObjectId) -> Option<String> {
		None
	}
}

/// What `Runtime::identify` came to
#[derive(Clone, Debug, PartialEq)]
pub enum Identified {
	/// the object had that device's config already
	Same,
	/// the device's config is in the object: its text (for the dongle's cache file)
	Configured(Vec<u8>),
	/// no config for that device, or the object did not take it
	Failed,
}

/// What `Tap::assign` did
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Assigned {
	/// the dongle is not open (yet): it gets the device when it opens
	Pending,
	/// the dongle's kind has no tracked object
	NotTracked,
	/// the dongle had that device already
	Same,
	/// the dongle's object has the device's config now
	Configured,
	/// the dongle's object gave up its device ("-")
	Released,
	/// no config for the device: the dongle's object has none either (not another device's)
	Failed,
}

/// vive_device_inst_t, with the parser's state (its SurviveUSBInfo) and the mode votes
#[derive(Debug)]
struct Device {
	/// as the C keeps it: 31 bytes at most
	serial: String,
	name: String,
	obj: Option<ObjectId>,
	parser: Option<Parser>,
	has_configed: bool,
	config: ConfigStream,
	packets_without_config: usize,
	tried_config_file: bool,
	write_config_cache: bool,
	config_by_serial_only: bool,
	/// consecutive light packets that look raw1 (+) or not (-), and the mode assumed (-1 = none yet)
	mode_votes: i8,
	mode_now: i8,
}

/// vive-test-drop's state (the C's test_drop static)
#[derive(Clone, Debug, Default)]
struct TestDrop {
	on: bool,
	t0: f64,
	t1: f64,
	frac: f64,
	acc: f64,
	codename: String,
}

/// The tap's driver state (SurviveDriverHIDTap)
#[derive(Debug)]
pub struct Tap {
	options: Options,
	live: bool,
	devices: Vec<Device>,
	cnt_per_type: [i32; 16],
	globals: Globals,
	test_drop: TestDrop,
	/// lines read, reports handled, devices of unknown type seen, malformed lines
	pub lines: u64,
	pub reports: u64,
	pub unknown_pid: u64,
	pub bad_lines: u64,
	/// sally: devices assigned to dongles not open yet (dongle, device), given when they open
	pending: Vec<(String, String)>,
	/// sally: SteamVR's latest word on each dongle it has named (dongle, device), open or not
	assigned: Vec<(String, String)>,
	/// sally: the `A` lines the live recording is owed, which its reader writes (`take_record_lines`)
	record_lines: Vec<Vec<u8>>,
	/// sally: the newest report's time (an `A` line written when a dongle opens takes it)
	last_time_us: u64,
}

impl Tap {
	/// DriverRegHIDTap_'s state, before the input starts (it logs nothing)
	pub fn new(options: Options, mode: Mode) -> Self {
		let test_drop = parse_test_drop(&options.test_drop);
		let globals = Globals { past_end: options.past_end, ..Globals::default() };
		Tap {
			options,
			live: mode == Mode::Live,
			devices: Vec::new(),
			cnt_per_type: [0; 16],
			globals,
			test_drop,
			lines: 0,
			reports: 0,
			unknown_pid: 0,
			bad_lines: 0,
			pending: Vec::new(),
			assigned: Vec::new(),
			record_lines: Vec::new(),
			last_time_us: 0,
		}
	}

	pub fn options(&self) -> &Options {
		&self.options
	}

	/// hidtap_close, with the context locked: the summary, the stream's recording closed, and the run clock frozen at
	/// its last value if the playback had installed its own (survive_install_run_time_fn(ctx, 0, 0))
	pub fn close(&self, rt: &mut dyn Runtime, record: Option<Record>) {
		rt.info(&format!("hidtap: {} lines, {} reports, {} bad lines", self.lines, self.reports, self.bad_lines));
		if self.globals.past_end > 0 {
			// (by `strip::Past`)
			let p = self.globals.past;
			rt.info(&format!(
				"hidtap: packets read past their end{}: {} (no payload {}, size {}, IMU {}, status {}, input {}, light {}, light entry {})",
				if self.globals.past_end > 1 { ", each reading stopped there" } else { "" },
				p.iter().sum::<u32>(),
				p[0],
				p[1],
				p[3],
				p[4],
				p[5],
				p[6],
				p[7]
			));
		}
		if let Some(r) = record {
			r.close();
		}
		rt.install_run_time_fn(None);
	}

	/// Whether the device has a tracked object (dev->so != 0)
	pub fn has_object(&self, dev: DeviceId) -> bool {
		self.devices[dev].obj.is_some()
	}

	/// Whether the device has its config (dev->hasConfiged)
	pub fn is_configured(&self, dev: DeviceId) -> bool {
		self.devices[dev].has_configed
	}

	/// The device with this serial, created when first seen (None for an unknown type or past the limit)
	pub fn device(&mut self, rt: &mut dyn Runtime, serial: &str, pid: u16) -> Option<DeviceId> {
		// sally: looked for as it is kept (a longer serial is the same device, not a new one at every line)
		let key = dongle_key(serial);
		if let Some(i) = self.devices.iter().position(|d| d.serial == key) {
			return Some(i);
		}
		let Some(type_idx) = DEVICES.iter().position(|t| t.vid == self.options.vid && t.pid == pid) else {
			if self.unknown_pid == 0 {
				rt.warn(&format!(
					"hidtap: unknown device {:04x}:{:04x} ({}); ignoring it and any others",
					self.options.vid, pid, serial
				));
			}
			self.unknown_pid += 1;
			return None;
		};
		if self.devices.len() >= MAX_DEVICES {
			rt.warn(&format!("hidtap: too many devices, ignoring {serial}"));
			return None;
		}
		let device = &DEVICES[type_idx];
		let name = if type_idx == 0 {
			"HMD".to_string()
		} else {
			// char name[16]
			let mut n = format!("{}{}", device.codename, self.cnt_per_type[type_idx]);
			n.truncate(15);
			n
		};
		self.cnt_per_type[type_idx] += 1;
		let idx = self.devices.len();
		self.devices.push(Device {
			serial: key.to_string(),
			name: name.clone(),
			obj: None,
			parser: None,
			has_configed: false,
			config: ConfigStream::default(),
			packets_without_config: 0,
			tried_config_file: false,
			write_config_cache: false,
			config_by_serial_only: false,
			mode_votes: 0,
			mode_now: 0,
		});
		if device.def_config.is_some() {
			if let Some(obj) = rt.create_object(&name, serial) {
				let d = &mut self.devices[idx];
				d.obj = Some(obj);
				d.parser = Some(Parser::new(&name));
				d.mode_now = -1;
				if self.options.lightcap_mode >= 0 {
					self.assume_lightcap_mode(rt, idx, self.options.lightcap_mode);
				}
				let d = &mut self.devices[idx];
				// live, a config that streams in is saved as <serial>_config.json (a replay never writes it)
				d.write_config_cache = self.live;
				// <dongle serial>_config.json only, never KN<n>_config.json
				d.config_by_serial_only = true;
				if self.load_config_file(rt, idx) {
					rt.info(&format!("hidtap: {name} configured from cache"));
					// sally: which device the cache was, for the recording (a replay's own folder may have other caches)
					if self.live {
						if let Some(device) = rt.object_serial(obj) {
							self.record_lines.push(line::cache_comment(serial, &device));
						}
					}
				} else if self.live {
					rt.info(&format!(
						"hidtap: {name}: no {serial}_config.json; {}",
						if self.options.config_fetch {
							"fetching it from the device once it streams"
						} else {
							"not tracked until SteamVR names the device on it (its config is then SteamVR's own copy)"
						}
					));
				}
			}
		}
		rt.info(&format!("hidtap: device {} is {} ({:04x}:{:04x})", serial, name, device.vid, device.pid));
		// sally: a device SteamVR named for this dongle before it opened
		let t = self.last_time_us;
		if let Some(k) = self.pending.iter().position(|(s, _)| s == key) {
			let (_, dev) = self.pending.remove(k);
			self.assign(rt, key, &dev, t);
		}
		// sally: a cache that is the config of a device SteamVR has put on another dongle is not this one's device
		let cached = match self.devices[idx].obj {
			Some(obj) if self.devices[idx].has_configed => rt.object_serial(obj),
			_ => None,
		};
		if let Some(cached) = cached {
			if self.assigned.iter().any(|(d, s)| d != key && s.eq_ignore_ascii_case(&cached)) {
				rt.info(&format!("hidtap: {name}'s cache is {cached}'s config, which SteamVR has on another dongle"));
				self.unconfigure(rt, idx, t);
			}
		}
		Some(idx)
	}

	/// sally: the device `device` (a serial; "-" for none) is the one on the dongle `serial`, as SteamVR says (the
	/// runtime protocol's `pair`, from the driver) or a recording's `A` line said: the dongle's object takes that
	/// device's own config (`Runtime::identify`) and any other dongle whose object had the device gives it up (a device
	/// talks through one dongle). A device whose config cannot be found leaves its dongle with none rather than another
	/// device's. Live, the dongle's cache file gets the config (the next start begins with it) and the recording an `A`
	/// line (`take_record_lines`); `time_us` is the time it is written with. A dongle not open yet gets the device when
	/// it opens
	pub fn assign(&mut self, rt: &mut dyn Runtime, serial: &str, device: &str, time_us: u64) -> Assigned {
		let serial = dongle_key(serial);
		// a device is on one dongle: the latest word for it stands
		self.assigned.retain(|(d, s)| d != serial && !(serial_is_key(device) && s.eq_ignore_ascii_case(device)));
		self.assigned.push((serial.to_string(), device.to_string()));
		if self.assigned.len() > MAX_PAIRS {
			self.assigned.remove(0);
		}
		let Some(idx) = self.devices.iter().position(|d| d.serial == serial) else {
			self.pending.retain(|(s, _)| s != serial);
			self.pending.push((serial.to_string(), device.to_string()));
			if self.pending.len() > MAX_PAIRS {
				self.pending.remove(0);
			}
			return Assigned::Pending;
		};
		let Some(obj) = self.devices[idx].obj else { return Assigned::NotTracked };
		let has = |rt: &dyn Runtime, d: &Device| {
			let serial = if d.has_configed { d.obj.and_then(|o| rt.object_serial(o)) } else { None };
			serial.is_some_and(|s| s.eq_ignore_ascii_case(device))
		};
		if has(rt, &self.devices[idx]) {
			return Assigned::Same;
		}
		if serial_is_key(device) {
			for j in 0..self.devices.len() {
				if j != idx && has(rt, &self.devices[j]) {
					self.unconfigure(rt, j, time_us);
				}
			}
		} else {
			if self.devices[idx].has_configed {
				self.unconfigure(rt, idx, time_us);
				return Assigned::Released;
			}
			return Assigned::Same;
		}
		match rt.identify(obj, device) {
			Identified::Same => {
				// the object had it already: the tap goes by it
				let d = &mut self.devices[idx];
				d.has_configed = true;
				d.tried_config_file = true;
				if let Some(p) = d.parser.as_mut() {
					p.configured = true;
				}
				Assigned::Same
			}
			Identified::Configured(json) => {
				let live = self.live;
				let d = &mut self.devices[idx];
				d.has_configed = true;
				// its cache, or a config streaming in, is not wanted any more
				d.tried_config_file = true;
				if let Some(p) = d.parser.as_mut() {
					p.configured = true;
					p.buttonmask = 0;
					p.touchmask = 0;
					p.axis = [0.0; 16];
				}
				if d.write_config_cache && serial_is_key(&d.serial) {
					let filename = format!("{}_config.json", d.serial);
					match write_cache(&filename, &json) {
						Some(Ok(())) => {
							rt.info(&format!("hidtap: {}'s cache {} is {}'s config now", d.name, filename, device))
						}
						_ => rt.warn(&format!("Could not write config cache {filename}")),
					}
				}
				if live {
					self.record_lines.push(line::assign(time_us, serial, device));
				}
				Assigned::Configured
			}
			Identified::Failed => {
				if self.devices[idx].has_configed {
					self.unconfigure(rt, idx, time_us);
				}
				Assigned::Failed
			}
		}
	}

	/// sally: the dongle's object gives up its device (`Runtime::release`): the dongle's reports stop at the tap until a
	/// device is assigned to it, and its cache is not read again (it is another device's)
	fn unconfigure(&mut self, rt: &mut dyn Runtime, idx: usize, time_us: u64) {
		let live = self.live;
		let d = &mut self.devices[idx];
		if let Some(obj) = d.obj {
			rt.release(obj);
		}
		d.has_configed = false;
		d.tried_config_file = true;
		if let Some(p) = d.parser.as_mut() {
			p.configured = false;
		}
		if live {
			let l = line::assign(time_us, &d.serial, "-");
			self.record_lines.push(l);
		}
	}

	/// sally: the `A` lines the live recording is owed since the last call (the live reader writes them)
	pub fn take_record_lines(&mut self) -> Vec<Vec<u8>> {
		std::mem::take(&mut self.record_lines)
	}

	/// An input report of the device's, as both inputs hand it on: its interface from the HID path's mi / col, its light
	/// packets sniffed for the mode, and the report to its parser at its true length (hidtap_line's 'I' case,
	/// hidtap_live_report)
	#[allow(clippy::too_many_arguments)]
	pub(crate) fn input_report(
		&mut self, rt: &mut dyn Runtime, dev: DeviceId, pid: u16, mi: i32, col: i32, time_us: u64, data: &[u8],
	) {
		let iface = devices::interface_from_hid(pid, mi, col);
		self.last_time_us = time_us;
		self.sniff_mode(rt, dev, data);
		let len = true_len(pid, data);
		self.input(rt, dev, iface, time_us, &data[..len]);
	}

	/// hidtap_live_report under the lock: the report counted, and handed on when its device is known (`dev`, the
	/// hidtap_device the reader found when it opened the device)
	#[allow(clippy::too_many_arguments)]
	pub fn live_report(
		&mut self, rt: &mut dyn Runtime, dev: Option<DeviceId>, pid: u16, mi: i32, col: i32, time_us: u64, data: &[u8],
	) {
		self.reports += 1;
		if let Some(dev) = dev {
			self.input_report(rt, dev, pid, mi, col, time_us, data);
		}
	}

	/// In detect mode, the pure light packets (flags 0x10) vote on the mode by their first byte, which in raw1 is a
	/// channel byte (bit 0 set, bits 0x0A clear); six in a row flip the mode assumed
	fn sniff_mode(&mut self, rt: &mut dyn Runtime, idx: usize, data: &[u8]) {
		let d = &mut self.devices[idx];
		if self.options.lightcap_mode != -2
			|| d.obj.is_none()
			|| data.len() < 6
			|| (data[0] != 0x23 && data[0] != 0x24)
			|| data[4] != 0x10
		{
			return;
		}
		let b = data[5];
		let raw1like = (b & 1) != 0 && (b & 0x0a) == 0;
		let v = &mut d.mode_votes;
		if raw1like {
			*v = if *v < 0 {
				1
			} else if *v < 100 {
				*v + 1
			} else {
				*v
			};
		} else {
			*v = if *v > 0 {
				-1
			} else if *v > -100 {
				*v - 1
			} else {
				*v
			};
		}
		let want: i8 = if *v >= MODE_VOTES {
			1
		} else if *v <= -MODE_VOTES {
			0
		} else {
			-1
		};
		if want >= 0 && want != d.mode_now {
			rt.info(&format!("hidtap: {} light packets look like raw{}, assuming that mode", d.name, want));
			d.mode_now = want;
			self.assume_lightcap_mode(rt, idx, want as i32);
		}
	}

	/// survive_vive_report_assume_lightcap_mode: what Valve's driver sends an RF dongle to switch the mode, given to
	/// the parser's bookkeeping only
	fn assume_lightcap_mode(&mut self, rt: &mut dyn Runtime, idx: usize, mode: i32) {
		if self.devices[idx].obj.is_none() || mode < 0 {
			return;
		}
		let pkt = [0xff, 0x87, 6, 1, if mode == 0 { 0 } else { 1 }, 0, 2, if mode == 2 { 1 } else { 0 }, 0];
		self.setup_write(rt, idx, 0x21, 0x09, 0x3ff, &pkt);
	}

	/// survive_vive_report_setup_write
	pub(crate) fn setup_write(
		&mut self, rt: &mut dyn Runtime, idx: usize, bm_request_type: u8, b_request: u8, w_value: u16, data: &[u8],
	) {
		let d = &mut self.devices[idx];
		if let (Some(obj), Some(parser)) = (d.obj, d.parser.as_mut()) {
			rt.setup_write(obj, parser, bm_request_type, b_request, w_value, data);
		}
	}

	/// survive_vive_report_feature_read
	pub(crate) fn feature_read(&mut self, rt: &mut dyn Runtime, idx: usize, data: &[u8]) {
		let d = &self.devices[idx];
		if let (Some(obj), Some(parser), false) = (d.obj, d.parser.as_ref(), data.is_empty()) {
			rt.feature_read(obj, parser, data);
		}
	}

	/// survive_vive_report_config: a config feature report (0x10 starts one, 0x11 is a chunk while unconfigured);
	/// false when the report is not one
	pub fn config_report(&mut self, rt: &mut dyn Runtime, idx: DeviceId, data: &[u8]) -> bool {
		if data.is_empty() {
			return false;
		}
		let d = &mut self.devices[idx];
		if data[0] == 0x10 {
			d.config.start();
			return true;
		}
		if data[0] == 0x11 && !d.has_configed {
			self.config_chunk(rt, idx, data);
			self.devices[idx].packets_without_config = 0;
			return true;
		}
		false
	}

	/// survive_vive_report_config_chunk: a chunk, and at the end the inflate and the config process; a config the
	/// object took is saved as `<serial>_config.json` where the device writes the cache (live)
	fn config_chunk(&mut self, rt: &mut dyn Runtime, idx: usize, data: &[u8]) {
		let d = &mut self.devices[idx];
		let Some(obj) = d.obj else { return };
		if data.len() < 2 {
			return;
		}
		let Chunk::End(inflated) = d.config.chunk(data) else { return };
		let json = match inflated {
			Ok(json) if !json.is_empty() => {
				rt.info(&format!("usbmon loaded {} total bytes of config data for {}", json.len(), d.name));
				Some(json)
			}
			Ok(_) => {
				rt.warn("Error: data for config descriptor");
				Some(Vec::new())
			}
			Err(e) => {
				rt.warn(&format!(
					"survive_simple_inflate could not inflate: {} {} (stream written to 'libz_error.stream')",
					e.msg.unwrap_or("(null)"),
					e.code
				));
				let _ = std::fs::write("libz_error.stream", d.config.compressed());
				rt.warn("Error: data for config descriptor");
				None
			}
		};
		if !d.has_configed {
			// the config process keeps the bytes; the cache gets the same
			let cache = if d.write_config_cache { json.clone() } else { None };
			if rt.config(obj, json) == 0 {
				let d = &mut self.devices[idx];
				d.has_configed = true;
				if let Some(p) = d.parser.as_mut() {
					p.configured = true;
				}
				if d.write_config_cache && serial_is_key(&d.serial) {
					// char filename[128], fopen(filename, "wb")
					let mut filename = format!("{}_config.json", d.serial);
					filename.truncate(127);
					// (the write's own result is not checked, as in the C)
					match write_cache(&filename, cache.as_deref().unwrap_or_default()) {
						Some(_) => rt.info(&format!("Cached config for {} in {}", d.name, filename)),
						None => rt.warn(&format!("Could not write config cache {filename}")),
					}
				}
			} else {
				rt.warn("Could not load from config");
			}
		}
	}

	/// survive_vive_report_input: a report to the object's parser once the device is configured (the info interface
	/// before that too); otherwise it counts towards the config file fallback
	fn input(&mut self, rt: &mut dyn Runtime, idx: usize, iface: i32, time_us: u64, data: &[u8]) {
		let d = &self.devices[idx];
		let forward =
			iface != 0 && (d.has_configed || iface == usb_if::TRACKER_INFO) && d.obj.is_some() && !data.is_empty();
		if forward {
			let (buffer, len) = report::report_buffer(data);
			// survive_data_cb: vive-test-drop's judgement, then the parser
			if self.test_drop_report(rt, idx) {
				return;
			}
			let d = &mut self.devices[idx];
			if let (Some(obj), Some(parser)) = (d.obj, d.parser.as_mut()) {
				rt.report(obj, parser, &mut self.globals, time_us, iface, &buffer, len);
			}
			return;
		}
		if !d.has_configed && self.options.allow_fs_read {
			self.try_config_file(rt, idx);
		}
	}

	/// test_drop_report: whether vive-test-drop drops this report
	fn test_drop_report(&mut self, rt: &mut dyn Runtime, idx: usize) -> bool {
		let td = &mut self.test_drop;
		if !td.on || td.codename != self.devices[idx].name {
			return false;
		}
		let t = rt.run_time();
		if t < td.t0 || t > td.t1 {
			return false;
		}
		td.acc += td.frac;
		if td.acc < 1.0 {
			return false;
		}
		td.acc -= 1.0;
		true
	}

	/// survive_vive_report_try_config_file: after 1000 reports without a config stream, the config file, once
	fn try_config_file(&mut self, rt: &mut dyn Runtime, idx: usize) {
		let d = &mut self.devices[idx];
		if d.obj.is_none() || d.has_configed || d.tried_config_file {
			return;
		}
		let n = d.packets_without_config;
		d.packets_without_config += 1;
		if n <= 1000 {
			return;
		}
		self.load_config_file(rt, idx);
	}

	/// survive_vive_report_load_config_file: `<serial>_config.json` (then `<name>_config.json`, unless by serial only,
	/// and nothing for a serial that cannot key a file); true when the device is configured after
	fn load_config_file(&mut self, rt: &mut dyn Runtime, idx: usize) -> bool {
		let d = &mut self.devices[idx];
		let Some(obj) = d.obj else { return d.has_configed };
		if d.has_configed {
			return true;
		}
		let mut tries = 2;
		if d.config_by_serial_only {
			tries = if serial_is_key(&d.serial) { 1 } else { 0 };
		}
		let mut i = 0;
		while i < tries && !self.devices[idx].has_configed {
			let d = &mut self.devices[idx];
			// char filename[128]
			let mut filename = format!("{}_config.json", if i == 0 { &d.serial } else { &d.name });
			filename.truncate(127);
			if rt.load_config_file(obj, &filename) {
				let d = &mut self.devices[idx];
				d.has_configed = true;
				if let Some(p) = d.parser.as_mut() {
					p.configured = true;
				}
			}
			self.devices[idx].tried_config_file = true;
			i += 1;
		}
		self.devices[idx].has_configed
	}
}

/// The `--hidtap-record` file (d->record): the tap's stream as lines, written by the input that reads it. The C writes
/// it through a text-mode FILE, so on Windows each newline goes out as CR LF (the live recordings' CR LF); so does this.
/// sally: a file named `.hidtapz` is written packed (`pack`): the same lines, to the byte, in a fraction of the space
pub struct Record {
	out: Out,
	/// the text mode's CR LF (Windows)
	crlf: bool,
}

enum Out {
	Text(BufWriter<File>),
	Packed(Box<PackWriter<File>>),
}

impl Record {
	/// DriverRegHIDTap_'s fopen(record, "w"): "hidtap: recording stream to %s", or "hidtap: cannot open %s for
	/// recording" and no recording
	pub fn open(rt: &mut dyn Runtime, path: &str) -> Option<Record> {
		let made = File::create(path).and_then(|f| match path.ends_with(pack::PACKED_END) {
			true => Record::packed(f, cfg!(windows)),
			false => Ok(Record::new(f, cfg!(windows))),
		});
		match made {
			Ok(r) => {
				rt.info(&format!("hidtap: recording stream to {path}"));
				Some(r)
			}
			Err(_) => {
				rt.warn(&format!("hidtap: cannot open {path} for recording"));
				None
			}
		}
	}

	fn new(f: File, crlf: bool) -> Record {
		Record { out: Out::Text(BufWriter::new(f)), crlf }
	}

	/// sally: the record as a packed recording, its header written
	fn packed(f: File, crlf: bool) -> std::io::Result<Record> {
		Ok(Record { out: Out::Packed(Box::new(PackWriter::new(f)?)), crlf })
	}

	// a write that fails is not reported, as the C's fputs is not checked
	fn write(&mut self, text: &[u8]) {
		let _ = match &mut self.out {
			Out::Text(out) => out.write_all(text),
			Out::Packed(out) => out.write(text),
		};
	}

	/// fputs: the text up to its first NUL, through the text mode
	pub fn put(&mut self, text: &[u8]) {
		let text = &text[..text.iter().position(|&c| c == 0).unwrap_or(text.len())];
		if !self.crlf {
			return self.write(text);
		}
		for (i, part) in text.split(|&c| c == b'\n').enumerate() {
			if i > 0 {
				self.write(b"\r\n");
			}
			self.write(part);
		}
	}

	/// hidtap_line's copy of a line as fgets read it, and a newline after a piece that has none (a line cut at the
	/// fgets limit, the last line of a file without one). The C reads in text mode, so on Windows a CR LF was an LF
	/// when it was read, and the text mode writes it as CR LF again
	pub fn line_as_read(&mut self, piece: &[u8]) {
		let line = &piece[..piece.iter().position(|&c| c == 0).unwrap_or(piece.len())];
		if self.crlf && line.ends_with(b"\r\n") {
			self.put(&line[..line.len() - 2]);
			self.put(b"\n");
			return;
		}
		self.put(line);
		// line[strlen(line) - 1] != '\n' (for an empty string the C reads the byte before its buffer: taken as not one)
		if line.last() != Some(&b'\n') {
			self.put(b"\n");
		}
	}

	/// fclose: what is buffered written out
	pub fn close(mut self) {
		self.end();
	}

	fn end(&mut self) {
		let _ = match &mut self.out {
			Out::Text(out) => out.flush(),
			Out::Packed(out) => out.end(),
		};
	}
}

/// sally: a record let go of without `close` still has every line it was given (the text's buffer writes itself
/// out as it goes; a packed one's last lines would stay behind)
impl Drop for Record {
	fn drop(&mut self) {
		self.end();
	}
}

/// A dongle's serial as the tap keeps it: its first 31 characters (char serial[32])
fn dongle_key(serial: &str) -> &str {
	serial.char_indices().nth(31).map_or(serial, |(i, _)| &serial[..i])
}

/// sally: a dongle's cache file written whole or not at all: into `<name>.tmp`, then renamed over the old one, so that
/// a run killed while it writes leaves the old cache as it was (the C's fopen empties it first). Where that cannot be
/// done it is written in place as the C writes it: a name that is not a plain file, a file that cannot be opened for
/// writing, a temporary that cannot be made or written, or a rename that fails (on Windows, while another process has
/// the file open; the temporary is removed). None when it cannot be opened for writing in place either (the C's fopen
/// failing), else the write's result
fn write_cache(filename: &str, bytes: &[u8]) -> Option<std::io::Result<()>> {
	let path = std::path::Path::new(filename);
	let replace = match std::fs::symlink_metadata(path) {
		Ok(m) => m.file_type().is_file() && std::fs::OpenOptions::new().write(true).open(path).is_ok(),
		Err(_) => true,
	};
	if replace {
		let tmp = format!("{filename}.tmp");
		if let Ok(mut f) = File::create(&tmp) {
			let written = f.write_all(bytes).is_ok();
			drop(f);
			if let Ok(m) = std::fs::metadata(path) {
				// the file's permissions go with it
				let _ = std::fs::set_permissions(&tmp, m.permissions());
			}
			if written && std::fs::rename(&tmp, path).is_ok() {
				return Some(Ok(()));
			}
			let _ = std::fs::remove_file(&tmp);
		}
	}
	let mut f = File::create(path).ok()?;
	Some(f.write_all(bytes))
}

/// survive_vive_report_serial_is_key: a serial that can key a config file (not empty, "-" or a "nosn-" placeholder)
pub fn serial_is_key(s: &str) -> bool {
	!s.is_empty() && s != "-" && !s.starts_with("nosn-")
}

/// Windows' ReadFile gives an RF dongle's whole 59-byte report, padded with stale bytes; report 0x23 is one watchman
/// packet (30 bytes)
pub fn true_len(pid: u16, data: &[u8]) -> usize {
	if (pid == 0x2101 || pid == 0x2102) && data.len() > 30 && data[0] == 0x23 {
		return 30;
	}
	data.len()
}

/// vive-test-drop's value: "T0,T1,CODENAME,FRACTION" (sscanf "%lf %lf %7s %lf" once the commas are spaces)
fn parse_test_drop(v: &str) -> TestDrop {
	let mut td = TestDrop::default();
	// char buf[128]
	let buf: String = v.chars().take(127).map(|c| if c == ',' { ' ' } else { c }).collect();
	let mut it = buf.split_whitespace();
	let (Some(t0), Some(t1), Some(name), Some(frac)) = (it.next(), it.next(), it.next(), it.next()) else { return td };
	let (Ok(t0), Ok(t1), Ok(frac)) = (t0.parse(), t1.parse(), frac.parse()) else { return td };
	td.on = true;
	td.t0 = t0;
	td.t1 = t1;
	td.frac = frac;
	td.codename = name.chars().take(7).collect();
	td
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn test_drop_takes_its_four_fields() {
		let td = parse_test_drop("10,20,KN1,0.25");
		assert!(td.on && td.t0 == 10.0 && td.t1 == 20.0 && td.frac == 0.25 && td.codename == "KN1");
		assert!(!parse_test_drop("").on);
		assert!(!parse_test_drop("10,20,KN1").on);
	}

	/// A runtime that notes what the tap asks of it; configs are taken when they inflated to something, files open
	/// when they exist
	#[derive(Default)]
	struct Notes {
		said: Vec<String>,
		objects: usize,
	}

	impl Runtime for Notes {
		fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId> {
			self.said.push(format!("object {name} {serial}"));
			self.objects += 1;
			Some(self.objects - 1)
		}
		fn load_config_file(&mut self, _obj: ObjectId, filename: &str) -> bool {
			self.said.push(format!("load {filename}"));
			std::path::Path::new(filename).is_file()
		}
		fn config(&mut self, _obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
			let json = json.unwrap_or_default();
			self.said.push(format!("config {}", String::from_utf8_lossy(&json)));
			if json.is_empty() {
				-1
			} else {
				0
			}
		}
		fn report(
			&mut self, obj: ObjectId, _parser: &mut Parser, _globals: &mut Globals, time_us: u64, iface: i32,
			buffer: &[u8; BUFFER_LEN], len: usize,
		) {
			self.said.push(format!("report {obj} {time_us} {iface} {len} {:02x}", buffer[len]));
		}
		fn setup_write(
			&mut self, obj: ObjectId, _parser: &mut Parser, _bm_request_type: u8, _b_request: u8, w_value: u16,
			data: &[u8],
		) {
			self.said.push(format!("setup {obj} {w_value:x} {data:?}"));
		}
		fn feature_read(&mut self, _obj: ObjectId, _parser: &Parser, _data: &[u8]) {}
		fn install_run_time_fn(&mut self, _time: Option<Arc<PlaybackTime>>) {}
		fn run_time(&self) -> f64 {
			0.0
		}
		fn info(&mut self, text: &str) {
			self.said.push(format!("Info: {text}"));
		}
		fn warn(&mut self, text: &str) {
			self.said.push(format!("Warning: {text}"));
		}
		fn error(&mut self, code: i32, text: &str) {
			self.said.push(format!("Error {code}: {text}"));
		}
	}

	/// A device's lines as it is first seen: live, a missing cache is said, in either of its two texts
	#[test]
	fn a_live_device_says_it_has_no_cached_config() {
		for (mode, fetch, line) in [
			(Mode::Live, true, Some("fetching it from the device once it streams")),
			(
				Mode::Live,
				false,
				Some("not tracked until SteamVR names the device on it (its config is then SteamVR's own copy)"),
			),
			(Mode::Playback, true, None),
		] {
			let mut rt = Notes::default();
			let mut tap = Tap::new(Options { config_fetch: fetch, ..Options::default() }, mode);
			assert_eq!(tap.device(&mut rt, "0000000000-9ZZ", 0x2102), Some(0));
			assert_eq!(tap.device(&mut rt, "0000000000-9ZZ", 0x2102), Some(0));
			let mut want = vec!["object KN0 0000000000-9ZZ".to_string(), "load 0000000000-9ZZ_config.json".into()];
			if let Some(l) = line {
				want.push(format!("Info: hidtap: KN0: no 0000000000-9ZZ_config.json; {l}"));
			}
			want.push("Info: hidtap: device 0000000000-9ZZ is KN0 (28de:2102)".into());
			assert_eq!(rt.said, want);
			assert!(tap.has_object(0) && !tap.is_configured(0));
			// a device without a usable serial looks for no file
			let mut rt = Notes::default();
			assert_eq!(tap.device(&mut rt, "nosn-3", 0x2300), Some(1));
			assert_eq!(rt.said[0], "object T20 nosn-3");
			assert!(!rt.said.iter().any(|l| l.starts_with("load")));
		}
	}

	/// A config fetched over feature reports (0x10, then 0x11 chunks, a chunk of no bytes last): live it is cached
	/// as <serial>_config.json, from a replay it is not
	#[test]
	fn a_live_config_stream_is_cached() {
		let json = b"{ \"device_class\": \"generic_tracker\" }";
		let z = miniz_oxide::deflate::compress_to_vec_zlib(json, 6);
		let serial = format!("TEST{}-0YX", std::process::id());
		let cache = format!("{serial}_config.json");
		for mode in [Mode::Playback, Mode::Live] {
			let _ = std::fs::remove_file(&cache);
			let mut rt = Notes::default();
			let mut tap = Tap::new(Options::default(), mode);
			let dev = tap.device(&mut rt, &serial, 0x2102).unwrap();
			rt.said.clear();
			let mut report = vec![0u8; 64];
			report[0] = 0x10;
			assert!(tap.config_report(&mut rt, dev, &report));
			for part in z.chunks(62) {
				report.fill(0);
				report[0] = 0x11;
				report[1] = part.len() as u8;
				report[2..2 + part.len()].copy_from_slice(part);
				assert!(tap.config_report(&mut rt, dev, &report));
			}
			report.fill(0);
			report[0] = 0x11;
			assert!(tap.config_report(&mut rt, dev, &report));
			assert!(tap.is_configured(dev));
			// configured: a chunk is not one any more
			assert!(!tap.config_report(&mut rt, dev, &report));
			let mut want = vec![
				format!("Info: usbmon loaded {} total bytes of config data for KN0", json.len()),
				format!("config {}", String::from_utf8_lossy(json)),
			];
			if mode == Mode::Live {
				want.push(format!("Info: Cached config for KN0 in {cache}"));
				assert_eq!(std::fs::read(&cache).unwrap(), json);
			} else {
				assert!(!std::path::Path::new(&cache).exists());
			}
			assert_eq!(rt.said, want);
		}
		let _ = std::fs::remove_file(&cache);
	}

	/// Live reports: each counted, a known device's light packets sniffed for the mode, and its reports to the parser
	/// at their true length once it is configured
	#[test]
	fn live_reports_go_to_the_parser() {
		let mut rt = Notes::default();
		let mut tap = Tap::new(Options::default(), Mode::Live);
		let dev = tap.device(&mut rt, "nosn-0", 0x2102).unwrap();
		// a device without a usable serial has no cache file: it is configured by a stream of one chunk
		let json = b"{}";
		let z = miniz_oxide::deflate::compress_to_vec_zlib(json, 6);
		let mut chunk = vec![0x11, z.len() as u8];
		chunk.extend_from_slice(&z);
		tap.config_report(&mut rt, dev, &[0x10]);
		tap.config_report(&mut rt, dev, &chunk);
		tap.config_report(&mut rt, dev, &[0x11, 0]);
		assert!(tap.is_configured(dev));
		rt.said.clear();
		// a 59-byte 0x23 report (one watchman packet and stale bytes) of pure light whose first byte looks raw1
		let mut light = [0x77u8; 59];
		light[0] = 0x23;
		light[4] = 0x10;
		light[5] = 0x01;
		for i in 0..6 {
			tap.live_report(&mut rt, Some(dev), 0x2102, -1, -1, 1000 + i, &light);
		}
		tap.live_report(&mut rt, None, 0x2613, -1, -1, 2000, &light);
		assert_eq!(tap.reports, 7);
		let mut want: Vec<String> = (0..5).map(|i| format!("report 0 {} 3 30 ca", 1000 + i)).collect();
		want.push("Info: hidtap: KN0 light packets look like raw1, assuming that mode".into());
		want.push("setup 0 3ff [255, 135, 6, 1, 1, 0, 2, 0, 0]".into());
		want.push("report 0 1005 3 30 ca".into());
		assert_eq!(rt.said, want);
	}

	/// A runtime that finds configs by device serial (`Runtime::identify`): the devices' configs it has, and each
	/// object's device
	#[derive(Default)]
	struct Devices {
		notes: Notes,
		configs: Vec<(String, String)>,
		objects: Vec<Option<String>>,
		/// cache files and the device each is the config of
		cache_is: Vec<(String, String)>,
	}

	impl Runtime for Devices {
		fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId> {
			self.objects.push(None);
			self.notes.create_object(name, serial)
		}
		fn load_config_file(&mut self, obj: ObjectId, filename: &str) -> bool {
			let found = self.notes.load_config_file(obj, filename);
			if found {
				self.objects[obj] = self.cache_is.iter().find(|(f, _)| f == filename).map(|(_, d)| d.clone());
			}
			found
		}
		fn config(&mut self, obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
			self.notes.config(obj, json)
		}
		fn report(
			&mut self, obj: ObjectId, parser: &mut Parser, globals: &mut Globals, time_us: u64, iface: i32,
			buffer: &[u8; BUFFER_LEN], len: usize,
		) {
			self.notes.report(obj, parser, globals, time_us, iface, buffer, len);
		}
		fn setup_write(&mut self, obj: ObjectId, parser: &mut Parser, t: u8, r: u8, w_value: u16, data: &[u8]) {
			self.notes.setup_write(obj, parser, t, r, w_value, data);
		}
		fn feature_read(&mut self, _obj: ObjectId, _parser: &Parser, _data: &[u8]) {}
		fn install_run_time_fn(&mut self, _time: Option<Arc<PlaybackTime>>) {}
		fn run_time(&self) -> f64 {
			0.0
		}
		fn info(&mut self, text: &str) {
			self.notes.info(text);
		}
		fn warn(&mut self, text: &str) {
			self.notes.warn(text);
		}
		fn error(&mut self, code: i32, text: &str) {
			self.notes.error(code, text);
		}
		fn identify(&mut self, obj: ObjectId, device: &str) -> Identified {
			self.notes.said.push(format!("identify {obj} {device}"));
			if self.objects[obj].as_deref() == Some(device) {
				return Identified::Same;
			}
			let Some((_, json)) = self.configs.iter().find(|(d, _)| d == device) else { return Identified::Failed };
			self.objects[obj] = Some(device.to_string());
			Identified::Configured(json.clone().into_bytes())
		}
		fn release(&mut self, obj: ObjectId) {
			self.notes.said.push(format!("release {obj}"));
			self.objects[obj] = None;
		}
		fn object_serial(&self, obj: ObjectId) -> Option<String> {
			self.objects[obj].clone()
		}
	}

	/// sally: a serial longer than the 31 characters kept is the same device at every line, not a new one each time; and
	/// SteamVR's word is kept for MAX_PAIRS dongles, the latest
	#[test]
	fn long_serials_and_many_pairs() {
		let mut rt = Notes::default();
		let mut tap = Tap::new(Options { config_fetch: false, ..Options::default() }, Mode::Playback);
		let long = "0123456789012345678901234567890123456789";
		assert_eq!(tap.device(&mut rt, long, 0x2102), Some(0));
		assert_eq!(tap.device(&mut rt, long, 0x2102), Some(0));
		assert_eq!((tap.devices.len(), tap.devices[0].serial.as_str()), (1, &long[..31]));
		for i in 0..100 {
			assert_eq!(tap.assign(&mut rt, &format!("NOTOPEN{i}"), &format!("LHR-{i:08X}"), i), Assigned::Pending);
		}
		assert_eq!((tap.pending.len(), tap.assigned.len()), (MAX_PAIRS, MAX_PAIRS));
		assert_eq!((tap.pending[0].0.as_str(), tap.assigned[0].0.as_str()), ("NOTOPEN36", "NOTOPEN36"));
		assert_eq!(tap.assigned.last().unwrap().0, "NOTOPEN99");
	}

	/// SteamVR's word on which device is on which dongle: the dongle's object takes the device's own config (and its
	/// cache and the recording follow, live), the device leaves the dongle that had it, a device with no config leaves
	/// its dongle with none, a dongle not open yet gets its device when it opens, "-" takes the device away
	#[test]
	fn a_dongle_carries_the_device_steamvr_says() {
		let id = std::process::id();
		let (d1, d2, d3, d4) =
			(format!("TEST{id}-1YX"), format!("TEST{id}-2YX"), format!("TEST{id}-3YX"), format!("TEST{id}-4YX"));
		let caches = [&d1, &d2, &d3].map(|d| format!("{d}_config.json"));
		for c in &caches {
			let _ = std::fs::remove_file(c);
		}
		let mut rt = Devices {
			configs: vec![("LHR-0000000A".into(), "{\"a\": 1}".into()), ("LHR-0000000B".into(), "{\"b\": 2}".into())],
			..Devices::default()
		};
		let mut tap = Tap::new(Options { config_fetch: false, ..Options::default() }, Mode::Live);
		let (k1, k2) = (tap.device(&mut rt, &d1, 0x2102).unwrap(), tap.device(&mut rt, &d2, 0x2102).unwrap());
		rt.notes.said.clear();
		assert_eq!(tap.assign(&mut rt, &d1, "LHR-0000000A", 10), Assigned::Configured);
		assert!(tap.is_configured(k1));
		assert_eq!(std::fs::read_to_string(&caches[0]).unwrap(), "{\"a\": 1}");
		// told again: nothing to do
		assert_eq!(tap.assign(&mut rt, &d1, "LHR-0000000A", 11), Assigned::Same);
		// A paired to the other dongle: the first gives it up
		assert_eq!(tap.assign(&mut rt, &d2, "LHR-0000000A", 12), Assigned::Configured);
		assert!(!tap.is_configured(k1) && tap.is_configured(k2));
		assert_eq!(rt.objects, [None, Some("LHR-0000000A".to_string())]);
		// a device with no config anywhere: its dongle is left with none, not another device's
		assert_eq!(tap.assign(&mut rt, &d2, "LHR-0000000C", 13), Assigned::Failed);
		assert!(!tap.is_configured(k2));
		// a dongle not open yet gets its device when it opens
		assert_eq!(tap.assign(&mut rt, &d3, "LHR-0000000B", 14), Assigned::Pending);
		let k3 = tap.device(&mut rt, &d3, 0x2102).unwrap();
		assert!(tap.is_configured(k3));
		assert_eq!(rt.objects[k3].as_deref(), Some("LHR-0000000B"));
		// "-": the dongle's device taken away
		assert_eq!(tap.assign(&mut rt, &d3, "-", 15), Assigned::Released);
		assert!(!tap.is_configured(k3));
		assert_eq!(tap.assign(&mut rt, "nosuchdongle", "-", 16), Assigned::Pending);
		// a dongle opening with a cache that is a device SteamVR has on another dongle: not taken
		std::fs::write(format!("{d4}_config.json"), "{\"b\": 2}").unwrap();
		rt.cache_is.push((format!("{d4}_config.json"), "LHR-0000000B".into()));
		assert_eq!(tap.assign(&mut rt, &d1, "LHR-0000000B", 17), Assigned::Configured);
		let k4 = tap.device(&mut rt, &d4, 0x2102).unwrap();
		assert!(!tap.is_configured(k4) && tap.is_configured(k1));
		let _ = std::fs::remove_file(format!("{d4}_config.json"));
		let said: Vec<&str> = rt
			.notes
			.said
			.iter()
			.map(String::as_str)
			.filter(|l| l.starts_with("identify") || l.starts_with("release"))
			.collect();
		assert_eq!(
			said,
			[
				"identify 0 LHR-0000000A",
				"release 0",
				"identify 1 LHR-0000000A",
				"identify 1 LHR-0000000C",
				"release 1",
				"identify 2 LHR-0000000B",
				"release 2",
				"identify 0 LHR-0000000B",
				"release 3"
			]
		);
		// the recording's A lines, each where the live run took it (the one written as the dongle opened takes the
		// newest report's time: none yet)
		let lines: Vec<String> = tap.take_record_lines().into_iter().map(|l| String::from_utf8(l).unwrap()).collect();
		assert_eq!(
			lines,
			[
				format!("A 10 {d1} LHR-0000000A\n"),
				format!("A 12 {d1} -\n"),
				format!("A 12 {d2} LHR-0000000A\n"),
				format!("A 13 {d2} -\n"),
				format!("A 0 {d3} LHR-0000000B\n"),
				format!("A 15 {d3} -\n"),
				format!("A 17 {d1} LHR-0000000B\n"),
				format!("# cache {d4} LHR-0000000B\n"),
				format!("A 0 {d4} -\n"),
			]
		);
		assert!(tap.take_record_lines().is_empty());
		// a replay applies its A lines and writes no cache and no lines
		for c in &caches {
			let _ = std::fs::remove_file(c);
		}
		let mut rt = Devices { configs: vec![("LHR-0000000A".into(), "{}".into())], ..Devices::default() };
		let mut tap = Tap::new(Options::default(), Mode::Playback);
		let k = tap.device(&mut rt, &d1, 0x2102).unwrap();
		assert_eq!(tap.assign(&mut rt, &d1, "LHR-0000000A", 10), Assigned::Configured);
		assert!(
			tap.is_configured(k) && tap.take_record_lines().is_empty() && !std::path::Path::new(&caches[0]).exists()
		);
		// a headset on its cable is a device under its own serial, and SteamVR's word names it on that: its own config,
		// kept under its serial for the next start, and the recording told
		let hmd = format!("LHR-{:08X}", id);
		let cache = format!("{hmd}_config.json");
		let _ = std::fs::remove_file(&cache);
		let mut rt = Devices { configs: vec![(hmd.clone(), "{\"h\": 3}".into())], ..Devices::default() };
		let mut tap = Tap::new(Options { config_fetch: false, ..Options::default() }, Mode::Live);
		let k = tap.device(&mut rt, &hmd, 0x2300).unwrap();
		assert!(!tap.is_configured(k));
		assert_eq!(tap.assign(&mut rt, &hmd, &hmd, 20), Assigned::Configured);
		assert!(tap.is_configured(k));
		assert_eq!(std::fs::read_to_string(&cache).unwrap(), "{\"h\": 3}");
		assert_eq!(tap.assign(&mut rt, &hmd, &hmd, 21), Assigned::Same);
		let lines: Vec<String> = tap.take_record_lines().into_iter().map(|l| String::from_utf8(l).unwrap()).collect();
		assert_eq!(lines, [format!("A 20 {hmd} {hmd}\n")]);
		let _ = std::fs::remove_file(&cache);
	}

	#[test]
	fn records_take_each_line_as_the_c_writes_it() {
		let path = std::env::temp_dir().join(format!("sally-hid-record-{}.hidtap", std::process::id()));
		for crlf in [false, true] {
			let mut r = Record::new(File::create(&path).unwrap(), crlf);
			r.line_as_read(b"R 1 a 2102 -1 -1 I 23\r\n");
			r.line_as_read(b"R 2 a 2102 -1 -1 I 24\n");
			r.line_as_read(b"# a piece without its newline");
			r.line_as_read(b"x\0y\n");
			r.put(&crate::line::input_report(3, "b", 0x2102, -1, -1, &[0x23]));
			r.close();
			let want: &[u8] = if crlf {
				b"R 1 a 2102 -1 -1 I 23\r\nR 2 a 2102 -1 -1 I 24\r\n# a piece without its newline\r\nx\r\nR 3 b 2102 -1 -1 I 23\r\n"
			} else {
				b"R 1 a 2102 -1 -1 I 23\r\nR 2 a 2102 -1 -1 I 24\n# a piece without its newline\nx\nR 3 b 2102 -1 -1 I 23\n"
			};
			assert_eq!(std::fs::read(&path).unwrap(), want);

			// sally: packed, the same lines to the byte
			let packed = path.with_extension("hidtapz");
			let mut r = Record::packed(File::create(&packed).unwrap(), crlf).unwrap();
			r.line_as_read(b"R 1 a 2102 -1 -1 I 23\r\n");
			r.line_as_read(b"R 2 a 2102 -1 -1 I 24\n");
			r.line_as_read(b"# a piece without its newline");
			r.line_as_read(b"x\0y\n");
			r.put(&crate::line::input_report(3, "b", 0x2102, -1, -1, &[0x23]));
			r.close();
			let mut text = Vec::new();
			std::io::Read::read_to_end(&mut pack::open(&packed).unwrap(), &mut text).unwrap();
			assert_eq!(text, want);
			assert!(pack::is_packed(&std::fs::read(&packed).unwrap()));
			let _ = std::fs::remove_file(&packed);
		}
		let _ = std::fs::remove_file(&path);
	}

	#[test]
	fn a_record_is_packed_by_its_name() {
		let mut rt = Notes::default();
		let dir = std::env::temp_dir();
		for (name, packed) in [("sally-hid-named.hidtap", false), ("sally-hid-named.hidtapz", true)] {
			let path = dir.join(format!("{}-{name}", std::process::id()));
			let mut r = Record::open(&mut rt, path.to_str().unwrap()).unwrap();
			r.put(b"R 1 a 2102 -1 -1 I 23\n");
			r.close();
			assert_eq!(pack::is_packed(&std::fs::read(&path).unwrap()), packed, "{name}");
			// and one let go of without close holds what it was given, a line begun included
			let mut r = Record::open(&mut rt, path.to_str().unwrap()).unwrap();
			r.put(b"R 1 a 2102 -1 -1 I 23\nR 2 a 2102");
			drop(r);
			let mut text = Vec::new();
			std::io::Read::read_to_end(&mut pack::open(&path).unwrap(), &mut text).unwrap();
			let end = if cfg!(windows) { "\r\n" } else { "\n" };
			assert_eq!(String::from_utf8(text).unwrap(), format!("R 1 a 2102 -1 -1 I 23{end}R 2 a 2102"), "{name}");
			let _ = std::fs::remove_file(&path);
		}
	}
}
