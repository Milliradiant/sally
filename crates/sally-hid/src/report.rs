//! The Vive-family report parser: from a report's bytes to IMU samples, light pulses, buttons and the device's mode
//! bookkeeping, as driver_vive.c's parser does it (survive_data_cb_locked and everything under it; the
//! test-drop of survive_data_cb is the runtime's). One `Parser` per device holds what the C keeps in the device's
//! SurviveUSBInfo and the input state of its SurviveObject; what the parser finds goes to a `Host` at once, in the C's
//! order, because the host's answers (the IMU's reference time above all) shape the rest of the same report.
//!
//! The parser reads the report as the C does, from a 128-byte buffer filled with 0xCA past the report
//! (survive_vive_report_input's copy), so a read past a short report sees what the C saw. A read the C would make
//! outside that buffer stops the parse instead (in the C it reads whatever lies on the stack).
//!
//! Not ported: the SV_VERBOSE lines (the runtime runs at verbosity 0); libsurvive's own device writes (the parser
//! never sends anything to a device).

use crate::devices::usb_if;

/// survive_vive_report_input's buffer: two INTBUFFSIZE halves
pub const BUFFER_LEN: usize = 128;

// enum vive_report_ids
const VIVE_REPORT_IMU_SCALES: u8 = 1;
const VIVE_REPORT_CHANGE_MODE: u8 = 0x04;
const VIVE_REPORT_VERSION: u8 = 0x05;
const VIVE_REPORT_HMD_SET_READOFFSET_USERDATA: u8 = 0x08;
const VIVE_REPORT_CONFIG_READMODE: u8 = 0x10;
const VIVE_REPORT_CONFIG_READ: u8 = 0x11;
const VIVE_REPORT_VERSION_ALT: u8 = 0x13;
const VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1: u8 = 0x21;
const VIVE_REPORT_RF_WATCHMAN: u8 = 0x23;
const VIVE_REPORT_RF_WATCHMANX2: u8 = 0x24;
const VIVE_REPORT_USB_LIGHTCAP_REPORT_V1: u8 = 0x25;
const VIVE_REPORT_RF_TURN_OFF: u8 = 0x26;
const VIVE_REPORT_USB_LIGHTCAP_REPORT_V2: u8 = 0x27;
const VIVE_REPORT_USB_LIGHTCAP_REPORT_RAW_MODE_1: u8 = 0x28;
const VIVE_REPORT_COMMAND: u8 = 0xff;
// enum vive_commands
const VIVE_COMMAND_CHANGE_PROTOCOL: u8 = 0x87;

/// SurviveObjectSubtype
pub mod subtype {
	pub const GENERIC: i32 = 0;
	pub const VIVE_HMD: i32 = 1;
	pub const INDEX_HMD: i32 = 2;
	pub const WAND: i32 = 3;
	pub const KNUCKLES_R: i32 = 4;
	pub const KNUCKLES_L: i32 = 5;
	pub const TRACKER: i32 = 6;
	pub const TRACKER_GEN2: i32 = 7;
	pub const COUNT: i32 = 8;
}

pub mod input_event {
	pub const NONE: i32 = 0;
	pub const BUTTON_DOWN: i32 = 0x3;
	pub const BUTTON_UP: i32 = 0x2;
	pub const TOUCH_DOWN: i32 = 0x5;
	pub const TOUCH_UP: i32 = 0x4;
	pub const AXIS_CHANGED: i32 = 0x8;
}

/// SURVIVE_BUTTON_UNKNOWN / SURVIVE_AXIS_UNKNOWN
pub const UNKNOWN: i32 = 255;

// SurviveButton / SurviveAxis values the mappings use
const BUTTON_TRIGGER: i32 = 0;
const BUTTON_TRACKPAD: i32 = 1;
const BUTTON_THUMBSTICK: i32 = 2;
const BUTTON_SYSTEM: i32 = 3;
const BUTTON_A: i32 = 4;
const BUTTON_B: i32 = 5;
const BUTTON_MENU: i32 = 6;
const BUTTON_GRIP: i32 = 7;
const AXIS_TRIGGER: i32 = 1;
const AXIS_TRACKPAD_X: i32 = 2;
const AXIS_TRACKPAD_Y: i32 = 3;
/// the first of the six proximity / force axes (middle, ring, pinky, trigger finger, grip force, trackpad force)
const AXIS_MIDDLE_FINGER_PROXIMITY: i32 = 4;
const AXIS_JOYSTICK_X: i32 = 10;
const AXIS_JOYSTICK_Y: i32 = 11;

/// enum LightcapMode, numbered as the C numbers it (its log lines print the numbers)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LightcapMode {
	Unknown = 0,
	/// the starting mode after an 0x40 0 0 0 0. Used for LH1
	Raw0 = 1,
	/// the mode used for LH2
	Raw1 = 2,
	Raw2 = 3,
}

/// LightcapElement: an LH1-style light pulse (raw0 mode)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LightcapElement {
	pub sensor_id: u8,
	pub length: u16,
	pub timestamp: u32,
}

/// A ButtonQueueEntry as it is posted (survive.h): the event, the button, and up to 16 axis changes (ids 255 = none)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ButtonEntry {
	pub event_type: i32,
	pub button_id: i32,
	pub ids: [i32; 16],
	pub values: [f64; 16],
}

impl ButtonEntry {
	/// prepareNextButtonEvent: a fresh entry
	fn new() -> Self {
		ButtonEntry { event_type: input_event::NONE, button_id: UNKNOWN, ids: [UNKNOWN; 16], values: [0.0; 16] }
	}
}

/// What the parser needs from the runtime, called in the C's order while a report is parsed
pub trait Host {
	/// ctx->lh_version: -1 unknown, 1 LH2 (the runtime's; sally tracks with LH2 alone). 0 has a host take a device's
	/// pulses as pulses (sally-check, for a device still in raw0 mode), as the C's LH1 value did
	fn lh_version(&self) -> i32;
	/// the device's activations.last_imu (the 64-bit IMU timecode the light's 24-bit times are placed against)
	fn last_imu(&self) -> u64;
	/// the device's object_subtype (from its config)
	fn object_subtype(&self) -> i32;
	/// whether the device has its config (obj->conf != 0)
	fn has_config(&self) -> bool;
	/// survive_map_sensor_id: the reported sensor through the config's channel map (255 = not a sensor)
	fn map_sensor_id(&self, reported: u8) -> u8;
	/// the raw_imu hook: raw accelerometer / gyro counts (agm[0..6]) at a 32-bit device timecode
	fn raw_imu(&mut self, mask: i32, agm: &[f64; 9], timecode: u32, id: i32);
	/// SurviveSensorActivations_register_runtime(&activations, activations.last_imu, time_received_us)
	fn register_runtime(&mut self, time_received_us: u64);
	/// the sync hook (gen2): a station's sync pulse
	fn sync(&mut self, channel: u8, timecode: u32, ootx: bool, gen: bool);
	/// sally: where in the report the sync that `sync` is about to be given stands (its entry's first byte, counted
	/// from the report's id). For a host that changes a recording's syncs where they stand; the others pass it over
	fn sync_at(&mut self, _at: usize) {}
	/// sally: the bytes [at, at + len) of the report are not read as a gen2 station's light: a packet (from the byte
	/// after its flags) or the rest of one passed over, or light read as the pulses of a 1.0 station. The byte at
	/// `at` is the last that the parser, or the tap's look at the light mode, goes by. For a host that clears what
	/// sally does not read; the others pass it over
	fn light_unread(&mut self, _at: usize, _len: usize) {}
	/// the sweep hook (gen2): a sensor hit by a station's sweep (sensor_id already mapped)
	fn sweep(&mut self, channel: u8, sensor_id: u8, timecode: u32, half_clock: bool);
	/// handle_lightcap: an LH1-style pulse (raw0); false when the disambiguator rejects it
	fn lightcap(&mut self, le: &LightcapElement) -> bool;
	/// a posted button event (incrementAndPostButtonQueue)
	fn button(&mut self, entry: &ButtonEntry);
	/// survive_notify_gen2 (the wired devices' gen2 reports)
	fn notify_gen2(&mut self, reason: &str);
	/// SV_WARN / SV_INFO text, as the C formats it
	fn warn(&mut self, text: &str);
	fn info(&mut self, text: &str);
	/// the printf hook's raw text (survive_dump_buffer and the binary dumps), and the run time it starts with
	fn print(&mut self, text: &str);
	fn run_time(&self) -> f64;
	/// survive_colorize: a name as the log shows it
	fn colorize(&self, s: &str) -> String;
	/// survive_default_set_imu_scale_modes: the IMU scale modes a feature read reports
	fn set_imu_scale_modes(&mut self, gyro_scale_mode: i32, acc_scale_mode: i32);
	/// handle_battery's so->charge and so->charging: the battery a status byte reports, its charge (%) and whether it
	/// is charging (the runtime's object keeps them, read by the diagnosis)
	fn battery(&mut self, charge: i8, charging: bool);
}

/// The C's process-wide statics the parser shares between devices: each warning is printed once (or ten times) per
/// run, not per device
#[derive(Debug, Default)]
pub struct Globals {
	/// vive_switch_mode's transfer_null_warning
	pub mode_switch_warned: bool,
	/// the 0x27 report's force_gen_warning
	pub force_gen_warned: bool,
	/// parse_and_process_raw1_lightcap's unknown_count
	pub raw1_unknown_count: i32,
	/// sally (hidtap-past-end, 2 in the runtime): 1 counts the radio packets the newer layout's reading goes past the
	/// end of (`strip::past_end`), 2 also stops every such reading at the packet's end. 0, as the C does: neither
	pub past_end: u8,
	/// those packets, by `strip::Past`
	pub past: [u32; 9],
}

/// One device's parser state: SurviveUSBInfo and the input state of the SurviveObject the parser compares against
/// (the battery it only reports: `Host::battery`)
#[derive(Clone, Debug)]
pub struct Parser {
	/// reports from the radio are parsed only once the config is in (survive_vive_mark_configured)
	pub configured: bool,
	pub lightcap_mode: LightcapMode,
	pub time_without_flag: usize,
	pub packets_seen_waiting_for_v2: usize,
	pub ignore_cnt: usize,
	pub buttonmask: u32,
	pub touchmask: u32,
	pub axis: [f64; 16],
	pub ison: bool,
	pub oldcode: i8,
	codename: String,
}

/// buttonEvent: what one input report says, before it becomes queue entries
#[derive(Clone, Copy, Debug, Default)]
struct ButtonEvent {
	pressed_buttons_valid: bool,
	touched_buttons_valid: bool,
	touchpad_horizontal_valid: bool,
	touchpad_vertical_valid: bool,
	trigger_high_res_valid: bool,
	proximity_valid: bool,
	raw_axis_cnt: u8,
	raw_axis: [f64; 16],
	proximity: [f64; 6],
	pressed_buttons: u32,
	touched_buttons: u32,
	touchpad_horizontal: f64,
	touchpad_vertical: f64,
	trigger_high_res: f64,
}

// enum ButtonEventSource
const BUTTON_EVENT_SOURCE_DEFAULT: u32 = 0;
const BUTTON_EVENT_SOURCE_RF: u32 = 1;
const BUTTON_TOUCH_EVENT_SOURCE: u32 = 0x80;

/// A read outside the C's 128-byte buffer: the parse stops
struct OutOfBuffer;

/// The report's bytes as the C's buffer holds them
struct Buf<'a> {
	b: &'a [u8; BUFFER_LEN],
	/// where reading ends: the buffer's length, or a packet's own end (sally's, `Globals::past_end`)
	end: isize,
}

impl Buf<'_> {
	fn at(&self, i: isize) -> Result<u8, OutOfBuffer> {
		if i < 0 || i >= self.end {
			return Err(OutOfBuffer);
		}
		Ok(self.b[i as usize])
	}
	fn u16(&self, i: isize) -> Result<u16, OutOfBuffer> {
		Ok(u16::from_le_bytes([self.at(i)?, self.at(i + 1)?]))
	}
	fn u32(&self, i: isize) -> Result<u32, OutOfBuffer> {
		Ok(u32::from_le_bytes([self.at(i)?, self.at(i + 1)?, self.at(i + 2)?, self.at(i + 3)?]))
	}
	/// packetToHex over [from, to): "%02x " per byte
	fn hex(&self, from: isize, to: isize) -> String {
		let mut s = String::new();
		let mut i = from;
		while i < to {
			match self.at(i) {
				Ok(v) => s.push_str(&format!("{v:02x} ")),
				Err(_) => break,
			}
			i += 1;
		}
		s
	}
}

/// survive_vive_report_input's copy: the report into a 0xCA-filled 128-byte buffer, its length capped at 128
pub fn report_buffer(data: &[u8]) -> ([u8; BUFFER_LEN], usize) {
	let mut b = [0xCAu8; BUFFER_LEN];
	let n = data.len().min(BUFFER_LEN);
	b[..n].copy_from_slice(&data[..n]);
	(b, n)
}

/// sally: `Host::light_unread` for the bytes from `from` (the last one read) up to `to`
fn unread(host: &mut dyn Host, from: isize, to: isize) {
	if from >= 0 && to > from {
		host.light_unread(from as usize, (to - from) as usize);
	}
}

/// sally: `Host::light_unread` for a radio packet passed over whole: from the byte after its flags (the one the tap's
/// look at the mode may read) to its end
fn unread_after_flags(host: &mut dyn Host, b: &Buf, readdata: isize) {
	if let Ok(size) = b.at(readdata + 1) {
		let payload = readdata + 3;
		unread(host, payload + 1, payload + size.wrapping_sub(1) as isize);
	}
}

/// fix_time24: a 24-bit time placed in the 32-bit range nearest the reference
pub fn fix_time24(time24: u32, ref_time: u32) -> u32 {
	let mut upper_ref = ref_time & 0xFF000000;
	let lower_ref = ref_time & 0x00FFFFFF;
	if lower_ref > time24 && lower_ref - time24 > (1 << 23) {
		upper_ref = upper_ref.wrapping_add(0x01000000);
	} else if lower_ref < time24 && time24 - lower_ref > (1 << 23) {
		// sally: below the reference's first 2^24 ticks too, into the period before the 32-bit wrap (the C's
		// `&& upper_ref > 0` put light from just before a device's clock wrapped 2^24 ticks, 0.35 s, ahead of it, where
		// its sweeps were dropped and its syncs were bad ones; the chain after takes the wrap: make_long_timecode)
		upper_ref = upper_ref.wrapping_sub(0x01000000);
	}
	upper_ref | time24
}

/// survive_dump_buffer's text: the run time, then 32 bytes a row in hex and as characters
pub fn dump_buffer(run_time: f64, data: &[u8]) -> String {
	let mut s = format_run_time(run_time);
	let bytes_per_row = 32;
	let mut i = 0;
	while i < data.len() {
		for j in 0..bytes_per_row {
			if j > 0 && j % 4 == 0 {
				s.push_str("  ");
			}
			if i + j < data.len() {
				s.push_str(&format!("{:02x} ", data[i + j]));
			} else {
				s.push_str("   ");
			}
		}
		s.push_str("    |    ");
		for j in 0..bytes_per_row {
			if j > 0 && j % 4 == 0 {
				s.push_str("  ");
			}
			if i + j < data.len() {
				let d = data[i + j];
				s.push(if d >= 32 && d < 127 { d as char } else { '.' });
			} else {
				s.push_str("   ");
			}
		}
		s.push('\n');
		i += bytes_per_row;
	}
	s
}

/// "%.7f " of a run time (a value C's printf and Rust's format agree on: both round the exact binary value)
fn format_run_time(t: f64) -> String {
	format!("{t:.7} ")
}

fn byte_to_bin(b: u8) -> String {
	(0..8).map(|i| if (b >> (7 - i)) & 1 != 0 { '1' } else { '0' }).collect()
}

impl Parser {
	/// survive_vive_register_driver: a device's parser before anything arrived (raw0, not configured)
	pub fn new(codename: &str) -> Self {
		Parser {
			configured: false,
			lightcap_mode: LightcapMode::Raw0,
			time_without_flag: 0,
			packets_seen_waiting_for_v2: 0,
			ignore_cnt: 0,
			buttonmask: 0,
			touchmask: 0,
			axis: [0.0; 16],
			ison: false,
			oldcode: 0,
			codename: codename.to_string(),
		}
	}

	/// survive_data_on_setup_write: the host wrote to the device; a lightcap mode switch is noted (and opens the
	/// discard window), nothing else matters
	pub fn on_setup_write(
		&mut self, host: &mut dyn Host, bm_request_type: u8, b_request: u8, w_value: u16, data: &[u8],
	) {
		let length = data.len();
		let is_usb = bm_request_type == 0x21 && b_request == 0x09 && w_value == 0x304 && length >= 8;
		if is_usb {
			let m = if data[1] == 0 {
				LightcapMode::Raw0
			} else if data[1] == 1 {
				LightcapMode::Raw1
			} else {
				LightcapMode::Raw2
			};
			host.info(&format!("LightcapMode usb {} {} -> {}", self.codename, self.lightcap_mode as i32, m as i32));
			self.lightcap_mode = m;
			self.ignore_cnt = 10;
			self.time_without_flag = 1;
		}
		let is_rf = bm_request_type == 0x21
			&& b_request == 0x09
			&& w_value == 0x3ff
			&& length >= 8
			&& data[1] == VIVE_COMMAND_CHANGE_PROTOCOL
			&& data[2] >= 6;
		if is_rf {
			let m = if data[4] == 0 {
				LightcapMode::Raw0
			} else if data[7] == 1 {
				LightcapMode::Raw2
			} else {
				LightcapMode::Raw1
			};
			host.info(&format!("LightcapMode rf {} {} -> {}", self.codename, self.lightcap_mode as i32, m as i32));
			self.lightcap_mode = m;
			self.ignore_cnt = 10;
			self.time_without_flag = 1;
		}
	}

	/// survive_usb_feature_read: a feature report read from the device (at least one byte). The C reads past a short
	/// report into its line buffer; here that reads zeros.
	pub fn feature_read(&self, host: &mut dyn Host, data: &[u8]) {
		let at = |i: usize| data.get(i).copied().unwrap_or(0);
		match at(0) {
			VIVE_REPORT_IMU_SCALES => host.set_imu_scale_modes(at(1) as i32, at(2) as i32),
			VIVE_REPORT_VERSION => {
				let rest = data.get(1..).unwrap_or(&[]);
				parse_tracker_version_info(host, &self.codename, rest, rest.len() as isize);
			}
			// the mode report and anything else: SV_VERBOSE only
			_ => {}
		}
	}

	/// vive_switch_mode: the parser cannot send a mode switch, so it only opens the discard window and warns once
	fn switch_mode(&mut self, host: &mut dyn Host, g: &mut Globals) {
		if self.time_without_flag == 0 {
			self.time_without_flag = 1;
			if !g.mode_switch_warned {
				host.warn(&format!("Can't update the usb device {} out of raw 0 mode; dumping data", self.codename));
				g.mode_switch_warned = true;
			}
		}
	}

	/// survive_data_cb_locked: one report (the C's buffer, its length, and the interface it arrived on)
	pub fn report(
		&mut self, host: &mut dyn Host, g: &mut Globals, time_received_us: u64, iface: i32, buffer: &[u8; BUFFER_LEN],
		actual_len: usize,
	) {
		let _ = self.report_inner(host, g, time_received_us, iface, buffer, actual_len);
	}

	fn report_inner(
		&mut self, host: &mut dyn Host, g: &mut Globals, time_received_us: u64, iface: i32, buffer: &[u8; BUFFER_LEN],
		actual_len: usize,
	) -> Result<(), OutOfBuffer> {
		let b = Buf { b: buffer, end: BUFFER_LEN as isize };
		let mut size = actual_len as isize;
		let enddata = size;
		let mut rd: isize = 0;
		let id = b.at(rd)?;
		rd += 1;
		size -= 1;

		if iface == usb_if::TRACKER_INFO {
			return self.parse_tracker_info(host, &b, id, rd, size);
		}
		if !host.has_config() {
			return Ok(());
		}

		match iface {
			usb_if::HMD_HEADSET_INFO => {
				// (the touched buttons are read but not valid: the C never posts them)
				rd += 2;
				let mut event = ButtonEvent { raw_axis_cnt: 3, ..Default::default() };
				event.touched_buttons = b.at(rd)? as u32; // Lens
				rd += 1;
				event.raw_axis[1] = b.u16(rd)? as f64; // Lens Separation
				rd += 2;
				rd += 2;
				event.touched_buttons |= b.at(rd)? as u32; // Button
				rd += 1;
				rd += 3;
				rd += 1; // Proxchange
				rd += 1;
				event.raw_axis[2] = b.u16(rd)? as f64; // Proximity
				rd += 2;
				event.raw_axis[0] = b.u16(rd)? as f64; // IPD
				self.ison = true;
				self.register_button_event(host, &event, BUTTON_EVENT_SOURCE_DEFAULT);
			}
			usb_if::HMD_IMU | usb_if::W_WATCHMAN1_IMU | usb_if::TRACKER0_IMU | usb_if::TRACKER1_IMU => {
				let mut i = 0;
				while i < 3 && rd < enddata {
					let accel = rd;
					rd += 12;
					let timecode = b.u32(rd)?;
					rd += 4;
					let code = b.at(rd)?;
					rd += 1;
					let cd = (code as i8).wrapping_sub(self.oldcode);
					if cd > 0 {
						self.oldcode = code as i8;
						let mut agm = [0.0f64; 9];
						for k in 0..6 {
							agm[k] = b.u16(accel + 2 * k as isize)? as i16 as f64;
						}
						host.raw_imu(3, &agm, timecode, code as i32);
						host.register_runtime(time_received_us);
					}
					i += 1;
				}
			}
			usb_if::WATCHMAN1 | usb_if::WATCHMAN2 => {
				if id == VIVE_REPORT_RF_WATCHMAN {
					self.handle_watchman(host, g, &b, time_received_us, rd)?;
				} else if id == VIVE_REPORT_RF_WATCHMANX2 {
					self.handle_watchman(host, g, &b, time_received_us, rd)?;
					self.handle_watchman(host, g, &b, time_received_us, rd + 29)?;
				} else if id == VIVE_REPORT_RF_TURN_OFF {
					self.ison = false;
				}
				// any other id: SV_VERBOSE only (report 1 arrives constantly from the 7-in-1 receiver)
			}
			usb_if::HMD_LIGHTCAP | usb_if::TRACKER1_LIGHTCAP => {
				// (the C's dump_binary here only ever feeds an SV_VERBOSE line)
				if id == VIVE_REPORT_USB_LIGHTCAP_REPORT_V1 {
					let mut i = 0;
					while i < 9 && rd < enddata {
						let le =
							LightcapElement { sensor_id: b.at(rd)?, length: b.u16(rd + 1)?, timestamp: b.u32(rd + 3)? };
						rd += 7;
						i += 1;
						if le.sensor_id > 0xfd {
							continue;
						}
						if host.lh_version() != 1 {
							host.lightcap(&le);
						}
					}
				} else if id == VIVE_REPORT_USB_LIGHTCAP_REPORT_V2 {
					if host.lh_version() == 0 {
						return Ok(());
					}
					host.notify_gen2("Report id 39");
					if host.lh_version() != 1 {
						if !g.force_gen_warned {
							host.warn(&format!("LH Gen is {}, dumping data", host.lh_version()));
							g.force_gen_warned = true;
						}
					} else {
						self.switch_mode(host, g);
					}
				} else if id == VIVE_REPORT_USB_LIGHTCAP_REPORT_RAW_MODE_1 {
					host.notify_gen2("Report ID 40");
					let length = b.at(rd)?;
					self.parse_raw1_lightcap(host, g, &b, 0, rd + 1, length)?;
				} else if id == VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1 {
					host.info(&format!(
						"USB lightcap report is of an unexpected type for {}: {} (0x{:02x})",
						self.codename, id, id
					));
				} else {
					host.warn(&format!(
						"USB lightcap report is of an unknown type for {}: {} (0x{:02x}); ignoring",
						self.codename, id, id
					));
				}
			}
			usb_if::W_WATCHMAN1_LIGHTCAP | usb_if::TRACKER0_LIGHTCAP => {
				if id == VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1 {
					for _ in 0..7 {
						let le = LightcapElement {
							sensor_id: b.u16(rd)? as u8,
							length: b.u16(rd + 2)?,
							timestamp: b.u32(rd + 4)?,
						};
						rd += 8;
						if le.sensor_id > 0xfd {
							continue;
						}
						host.lightcap(&le);
					}
				} else {
					host.warn(&format!("Unrecognized report id: 0x{:02x} for {}", id, self.codename));
				}
			}
			usb_if::TRACKER0_BUTTONS | usb_if::TRACKER1_BUTTONS | usb_if::W_WATCHMAN1_BUTTONS => {
				if id == VIVE_REPORT_IMU_SCALES {
					let report_type = b.u16(rd)?;
					match report_type {
						0x0400 => {
							let battery_info = b.at(rd + 13)?;
							self.handle_battery(host, battery_info);
						}
						0x0700 => {}
						0x0800 => {
							let ipd = b.at(rd + 11)? as u16 | ((b.at(rd + 12)? as u16) << 8);
							let proximity = (b.at(rd + 13)? as u16 | ((b.at(rd + 14)? as u16) << 8)) as i16;
							let on_face = b.at(rd + 19)?;
							let mut evt = ButtonEvent {
								raw_axis_cnt: 2,
								pressed_buttons_valid: true,
								pressed_buttons: on_face as u32,
								..Default::default()
							};
							evt.raw_axis[0] = ipd as f64 / 65536.0;
							evt.raw_axis[1] = proximity as f64 / 32768.0;
							self.register_button_event(host, &evt, BUTTON_EVENT_SOURCE_DEFAULT);
						}
						0x100 => {
							let ev = ButtonEvent {
								pressed_buttons_valid: true,
								pressed_buttons: b.u32(rd + 0x7)?,
								trigger_high_res_valid: true,
								trigger_high_res: b.u16(rd + 0x19)? as f64 / 32768.0,
								touchpad_horizontal_valid: true,
								touchpad_vertical_valid: true,
								touchpad_horizontal: b.u16(rd + 0x13)? as i16 as f64 / 32768.0,
								touchpad_vertical: b.u16(rd + 0x15)? as i16 as f64 / 32768.0,
								..Default::default()
							};
							self.register_button_event(host, &ev, BUTTON_EVENT_SOURCE_DEFAULT);
							// the C falls through into its default: the dump
							host.print(&dump_buffer(host.run_time(), &collect(&b, rd, size)));
						}
						_ => {
							host.print(&dump_buffer(host.run_time(), &collect(&b, rd, size)));
						}
					}
				} else {
					host.warn(&format!("Unknown report id 0x{:02x} for {}", id, self.codename));
				}
			}
			_ => {
				host.warn(&format!("Unknown interface {} for {}", iface, self.codename));
			}
		}
		Ok(())
	}

	/// parse_tracker_info: the tracker's info interface (version reports), handled before any config
	fn parse_tracker_info(
		&mut self, host: &mut dyn Host, b: &Buf, id: u8, rd: isize, size: isize,
	) -> Result<(), OutOfBuffer> {
		let dump = |host: &mut dyn Host| host.print(&dump_buffer(host.run_time(), &collect(b, rd, size)));
		match id {
			VIVE_REPORT_IMU_SCALES => {
				host.info(&format!("IMU Scales 1: 0x{:x} 0x{:x}", b.at(rd)?, b.at(rd + 1)?));
				dump(host);
			}
			VIVE_REPORT_CHANGE_MODE => {
				host.info("Info 4: ");
				dump(host);
			}
			VIVE_REPORT_VERSION_ALT | VIVE_REPORT_VERSION => {
				// the C copies the whole version struct whatever the report's length: past it, the buffer's fill
				let data = collect(b, rd, VERSION_INFO_LEN as isize);
				parse_tracker_version_info(host, &self.codename, &data, size);
			}
			VIVE_REPORT_HMD_SET_READOFFSET_USERDATA => {
				host.info(&format!("Info 8: 0x{:x}", b.at(rd)?));
			}
			VIVE_REPORT_CONFIG_READMODE | VIVE_REPORT_CONFIG_READ => {}
			VIVE_REPORT_COMMAND => {
				let n = b.at(rd + 1)?;
				host.info(&format!("Received cmd 0x{:02x} with {} bytes of data", b.at(rd)?, n));
				host.print(&dump_buffer(host.run_time(), &collect(b, rd + 2, n as isize)));
			}
			_ => {
				host.info(&format!("Unknown field 0x{:02x} for {}", id, self.codename));
				dump(host);
			}
		}
		Ok(())
	}

	/// survive_handle_watchman: one radio packet (a report holds one or two)
	fn handle_watchman(
		&mut self, host: &mut dyn Host, g: &mut Globals, b: &Buf, time_in_us: u64, readdata: isize,
	) -> Result<(), OutOfBuffer> {
		if self.ignore_cnt > 0 {
			self.ignore_cnt -= 1;
			unread_after_flags(host, b, readdata);
			return Ok(());
		}
		// AS_SHORT(readdata[0], readdata[2]): the time's two bytes around the size byte
		let time = ((b.at(readdata)? as u16) << 8) | b.at(readdata + 2)? as u16;
		let payload_size = b.at(readdata + 1)?.wrapping_sub(1);
		let payload = readdata + 3;
		let payload_end = payload + payload_size as isize;

		if self.use_watchman_v2(host) {
			if g.past_end > 1 {
				// sally: the reading stops at the packet's own end, with what it had handed on by then. A packet not
				// laid out as its flags say would be read on into the dongle's bytes after it, and those handed on
				let inside = Buf { b: b.b, end: payload_end.min(b.end) };
				let _ = self.handle_watchman_v2(host, g, &inside, time_in_us, time, payload, payload_end);
				return Ok(());
			}
			return self.handle_watchman_v2(host, g, b, time_in_us, time, payload, payload_end);
		}

		if self.lightcap_mode != LightcapMode::Raw0 {
			unread(host, payload + 1, payload_end);
			return Ok(());
		}
		let mut p = payload;
		if !self.read_event(host, b, time_in_us, time, &mut p, payload_end)? {
			host.warn(&format!("Read event failed; full payload: {}", b.hex(readdata + 3, payload_end)));
			return Ok(());
		}
		if p < payload_end {
			unread(host, p, payload_end);
			let mut les = [LightcapElement::default(); 10];
			let cnt = self.read_light_data(host, b, time, &mut p, payload_end, &mut les)?;
			if cnt < 0 {
				host.warn(&format!(
					"Read light data error {}   [Time:{:04X}] [Payload: {}]",
					cnt,
					time,
					b.hex(p, payload_end)
				));
			} else {
				for i in (0..cnt as usize).rev() {
					host.lightcap(&les[i]);
				}
			}
		}
		Ok(())
	}

	fn use_watchman_v2(&self, host: &dyn Host) -> bool {
		if host.lh_version() == 1 {
			return true;
		}
		!matches!(host.object_subtype(), subtype::TRACKER | subtype::WAND)
	}

	#[allow(clippy::too_many_arguments)]
	fn handle_watchman_v2(
		&mut self, host: &mut dyn Host, g: &mut Globals, b: &Buf, time_in_us: u64, time: u16, payload: isize,
		payload_end: isize,
	) -> Result<(), OutOfBuffer> {
		let origin = payload;
		let mut p = payload;
		if !self.configured {
			unread(host, payload + 1, payload_end);
			return Ok(());
		}
		if host.lh_version() == 1 && self.lightcap_mode != LightcapMode::Raw1 {
			self.switch_mode(host, g);
			return Ok(());
		}

		if g.past_end > 0 {
			if let Some(why) = crate::strip::past_end(&collect(b, origin - 3, 29)) {
				g.past[why as usize] += 1;
			}
		}

		let flags = b.at(p)?;
		p += 1;
		let mut has_errors = false;

		// Some kind of startup heartbeat?
		if flags == 0xe2 {
			return Ok(());
		}

		let mut flag_lightcap = flags & 0x10 == 0x10;
		let flag_input = flags & 0x20 == 0x20;
		let flag_meta_data = flags & 0x40 == 0x40;
		let flag_imu = flags & 0x80 == 0x80;

		if flag_imu {
			self.read_imu_data(host, b, time_in_us, time, &mut p, payload_end)?;
		}

		// These things happen every 10 seconds
		if flag_meta_data {
			let marker_byte = b.at(p)?;
			p += 1;
			match marker_byte {
				0x80 => {
					let battery_byte = b.at(p)?;
					p += 1;
					self.handle_battery(host, battery_byte);
				}
				0x20 => {
					p += 1; // mode?
				}
				0x30 => {
					p += 2; // only zeros seen here
				}
				_ => {
					// SV_VERBOSE only; the rest of the packet is dropped
					unread(host, p - 1, payload_end);
					return Ok(());
				}
			}
		}

		if flag_input {
			if payload_end - p < 1 {
				host.warn(&format!(
					"{} handle_input needed {} bytes but had {}",
					self.codename,
					1,
					(payload_end - p) as u32
				));
				// VERIFY_LENGTH_OR_FAIL jumps to the dump
				host.print(&dump_buffer(host.run_time(), &collect(b, origin, payload_end - origin)));
				return Ok(());
			}
			let input_flags = b.at(p)?;
			p += 1;
			has_errors = !self.handle_input(host, b, input_flags, &mut p, payload_end)?;
		}

		if self.packets_seen_waiting_for_v2 > 200 {
			self.packets_seen_waiting_for_v2 = 0;
			self.time_without_flag = 0;
		}

		if self.time_without_flag > 0 && self.time_without_flag < 20 {
			self.packets_seen_waiting_for_v2 += 1;
			if flag_lightcap {
				self.time_without_flag = 1;
				flag_lightcap = false;
			} else {
				self.time_without_flag += 1;
			}
		}

		// sally: light that is passed over, or read as the pulses of a 1.0 station
		if flags & 0x10 == 0x10 && !(flag_lightcap && !has_errors && self.lightcap_mode != LightcapMode::Raw0) {
			unread(host, p, payload_end);
		}
		if flag_lightcap && !has_errors {
			if self.lightcap_mode == LightcapMode::Raw0 {
				self.parse_and_process_lightcap(host, b, time, p, payload_end)?;
			} else {
				let len = (payload_end - p) as u8;
				let read = self.parse_raw1_lightcap(host, g, b, time, p, len)?;
				if read == -1 {
					has_errors = true;
				} else {
					p += read as isize;
				}
			}
		}

		if self.time_without_flag == 20 && p != payload_end {
			has_errors = true;
			host.warn(&format!("Did not read full input packet; {} bytes remain", payload_end - p));
		}

		if !has_errors {
			return Ok(());
		}
		host.print(&dump_buffer(host.run_time(), &collect(b, origin, payload_end - origin)));
		Ok(())
	}

	/// read_imu_data: a time byte and six shorts
	fn read_imu_data(
		&mut self, host: &mut dyn Host, b: &Buf, time_in_us: u64, time: u16, p: &mut isize, payload_end: isize,
	) -> Result<bool, OutOfBuffer> {
		if payload_end - *p < 7 {
			return Ok(false);
		}
		let mut q = *p;
		let time_lsb = b.at(q)?;
		q += 1;
		let mut agm = [0.0f64; 9];
		for k in 0..6 {
			agm[k] = b.u16(q)? as i16 as f64;
			q += 2;
		}
		host.raw_imu(3, &agm, ((time as u32) << 16) | ((time_lsb as u32) << 8), 0);
		host.register_runtime(time_in_us);
		*p = q;
		Ok(true)
	}

	/// handle_battery: a status byte's charge (its low 7 bits, %) and charging bit, given to the device at once when
	/// either is set
	fn handle_battery(&self, host: &mut dyn Host, bat_status: u8) {
		let percent = (bat_status & 0x7f) as i8;
		let charging = (bat_status & 0x80) == 0x80;
		if percent != 0 || charging {
			host.battery(percent, charging);
		}
	}

	/// read_event: the v1 packets' input and status events
	fn read_event(
		&mut self, host: &mut dyn Host, b: &Buf, time_in_us: u64, time: u16, read_ptr: &mut isize, payload_end: isize,
	) -> Result<bool, OutOfBuffer> {
		let mut p = *read_ptr;
		// If we're looking at light data, return
		if b.at(p)? & 0xE0 != 0xE0 {
			return Ok(true);
		}
		// This is some kind of heartbeat
		if b.at(p)? == 0xE2 {
			*read_ptr = payload_end;
			return Ok(true);
		}
		let flags = b.at(p)?;
		p += 1;
		let flag_input = flags & 0x10 == 0x10;
		let flag_imu = flags & 0x08 == 0x08;
		if flag_input {
			let first_gen = (flags & 0x7) != 0;
			let mut ev = ButtonEvent::default();
			if first_gen {
				let flag_trigger = flags & 0x4 == 0x4;
				let flag_motion = flags & 0x2 == 0x2;
				let flag_button = flags & 0x1 == 0x1;
				if flag_button {
					ev.pressed_buttons_valid = true;
					ev.pressed_buttons = b.at(p)? as u32;
					p += 1;
				}
				if flag_trigger {
					ev.trigger_high_res_valid = true;
					ev.trigger_high_res = b.at(p)? as f64 / 255.0;
					p += 1;
				}
				if flag_motion {
					ev.touchpad_horizontal_valid = true;
					ev.touchpad_vertical_valid = true;
					ev.touchpad_horizontal = b.u16(p)? as i16 as f64 / 32768.0;
					p += 2;
					ev.touchpad_vertical = b.u16(p)? as i16 as f64 / 32768.0;
					p += 2;
				}
			} else {
				let gen_two_type = b.at(p)?;
				p += 1;
				if gen_two_type == 0xA1 {
					let touch_flags = b.at(p)? as u32;
					p += 1;
					ev.touched_buttons_valid = true;
					ev.touched_buttons = touch_flags & !0x40u32;
					ev.touched_buttons |= (touch_flags & 0x40) >> 4;
					ev.proximity_valid = true;
					for k in 0..6 {
						ev.proximity[k] = b.at(p)? as f64 / 255.0;
						p += 1;
					}
				} else {
					let prev = b.at(p - 1)?;
					host.warn(&format!(
						"Unknown gen two event {} 0x{:02X} 0b{} [Time:{:04X}] [Payload: {}] <<ABORT FURTHER READ>>",
						self.codename,
						prev,
						byte_to_bin(prev),
						time,
						b.hex(p, payload_end)
					));
					return Ok(false);
				}
			}
			self.register_button_event(host, &ev, BUTTON_EVENT_SOURCE_RF);
		} else {
			let flag_battery_status = flags & 0x1 == 0x1;
			let flag_unknown = (flags & 0x6) != 0;
			if flag_unknown {
				host.warn(&format!(
					"Unknown status event 0x{:02X} [Time:{:04X}] [Payload: {}] <<ABORT FURTHER READ>>",
					b.at(p - 1)?,
					time,
					b.hex(p, payload_end)
				));
				*read_ptr = p;
				return Ok(false);
			}
			if flag_battery_status {
				let bat_status = b.at(p)?;
				p += 1;
				self.handle_battery(host, bat_status);
				// Maybe read another event, IMU data or Light Data
				let rtn = self.read_event(host, b, time_in_us, time, &mut p, payload_end)?;
				*read_ptr = p;
				return Ok(rtn);
			}
		}
		// Read off any IMU data present
		if flag_imu {
			self.read_imu_data(host, b, time_in_us, time, &mut p, payload_end)?;
		}
		*read_ptr = p;
		Ok(true)
	}

	/// handle_input: the v2 packets' input event
	fn handle_input(
		&mut self, host: &mut dyn Host, b: &Buf, flags: u8, p: &mut isize, payload_end: isize,
	) -> Result<bool, OutOfBuffer> {
		let first_gen = (flags & 0x7) != 0;
		let mut ev = ButtonEvent::default();
		let need = |host: &mut dyn Host, p: isize, len: isize| -> bool {
			if payload_end - p < len {
				host.warn(&format!(
					"{} handle_input needed {} bytes but had {}",
					self.codename,
					len,
					(payload_end - p) as u32
				));
				return false;
			}
			true
		};
		if first_gen {
			let flag_trigger = flags & 0x4 == 0x4;
			let flag_motion = flags & 0x2 == 0x2;
			let flag_button = flags & 0x1 == 0x1;
			if flag_button {
				ev.pressed_buttons_valid = true;
				if !need(host, *p, 1) {
					return Ok(false);
				}
				ev.pressed_buttons = b.at(*p)? as u32;
				*p += 1;
			}
			if flag_trigger {
				ev.trigger_high_res_valid = true;
				if !need(host, *p, 1) {
					return Ok(false);
				}
				ev.trigger_high_res = b.at(*p)? as f64 / 255.0;
				*p += 1;
			}
			if flag_motion {
				ev.touchpad_horizontal_valid = true;
				ev.touchpad_vertical_valid = true;
				if !need(host, *p, 4) {
					return Ok(false);
				}
				ev.touchpad_horizontal = b.u16(*p)? as i16 as f64 / 32768.0;
				*p += 2;
				ev.touchpad_vertical = b.u16(*p)? as i16 as f64 / 32768.0;
				*p += 2;
			}
		} else {
			let gen_two_type = b.at(*p)?;
			*p += 1;
			if gen_two_type == 0xA1 {
				if !need(host, *p, 7) {
					return Ok(false);
				}
				let touch_flags = b.at(*p)? as u32;
				*p += 1;
				ev.touched_buttons_valid = true;
				ev.touched_buttons = touch_flags & !0x40u32;
				ev.touched_buttons |= (touch_flags & 0x40) >> 4;
				ev.proximity_valid = true;
				for k in 0..6 {
					ev.proximity[k] = b.at(*p)? as f64 / 255.0;
					*p += 1;
				}
			} else {
				let prev = b.at(*p - 1)?;
				host.warn(&format!(
					"Unknown gen two event 0x{:02X} 0b{} [Payload: {}] <<ABORT FURTHER READ>>",
					prev,
					byte_to_bin(prev),
					b.hex(*p, payload_end)
				));
				return Ok(false);
			}
		}
		self.register_button_event(host, &ev, BUTTON_EVENT_SOURCE_RF);
		Ok(true)
	}

	/// parse_and_process_lightcap: raw0 light (LH1-style pulses), each to the disambiguator
	fn parse_and_process_lightcap(
		&mut self, host: &mut dyn Host, b: &Buf, time: u16, payload: isize, payload_end: isize,
	) -> Result<(), OutOfBuffer> {
		let mut les = [LightcapElement::default(); 10];
		let start = payload;
		let mut p = payload;
		let cnt = self.read_light_data(host, b, time, &mut p, payload_end, &mut les)?;
		// (the C's sensor-id check here is compiled only without NDEBUG)
		if cnt < 0 {
			host.warn(&format!(
				"Read light data error {}   [Time:{:04X}] [Payload: {}]",
				cnt,
				time,
				b.hex(p, payload_end)
			));
			host.warn(&format!("Full payload: {}", b.hex(start, payload_end)));
		} else {
			for i in (0..cnt as usize).rev() {
				host.lightcap(&les[i]);
			}
		}
		Ok(())
	}

	/// read_light_data: raw0 light, sensors at the front, variable-length time deltas from the back (see the C for the
	/// format). Returns the number of pulses, or the C's negative error codes.
	fn read_light_data(
		&mut self, host: &mut dyn Host, b: &Buf, time: u16, read_ptr: &mut isize, payload_end: isize,
		output: &mut [LightcapElement; 10],
	) -> Result<i32, OutOfBuffer> {
		let payload = *read_ptr;
		let reference_time = host.last_imu() as u32;
		if payload_end - payload <= 3 {
			return Ok(0);
		}
		if (b.at(payload)? & 0xE0) == 0xE0 {
			host.warn(&format!(
				"Light contains probable non-light data : 0x{:02X} [Time:{:04X}] [Payload: {}]",
				b.at(payload)?,
				time,
				b.hex(payload, payload_end)
			));
		}

		// Step 1 - Extract deltas between events the corresponding sensors
		let mut time_index: usize = 0;
		// times[16] in the C, and one more slot: its sixteenth delta is stored one past the array (the check before the
		// store lets index 16 through). The slot takes that stray store and is never read, as the C's neighbour on the
		// stack is assumed to be.
		let mut times = [0u32; 17];
		let max_time_index = 16;
		let max_events = max_time_index >> 1;
		let mut report_order = [0u8; 16];
		let mut sensors = [(0u8, 0u8); 8]; // (sensorId, edgeCount)

		let mut ids = payload;
		let mut event_ptr = payload_end;

		// Last three bytes of light data are the LSB of the time of the last event
		event_ptr -= 4;
		let mut last_event_time: u32 = (((time >> 8) as u32) << 24)
			| ((b.at(event_ptr + 3)? as u32) << 16)
			| ((b.at(event_ptr + 2)? as u32) << 8)
			| (b.at(event_ptr + 1)? as u32);
		if last_event_time > reference_time && last_event_time - reference_time > (1 << 23) {
			last_event_time = last_event_time.wrapping_sub(1 << 24);
		} else if reference_time > last_event_time && reference_time - last_event_time > (1 << 23) {
			last_event_time = last_event_time.wrapping_add(1 << 24);
		}
		times[0] = last_event_time;

		'outer: while ids + ((time_index >> 1) as isize) < event_ptr {
			let mut time_delta: u32 = 0;
			let event_ptr_start = event_ptr;
			loop {
				time_delta <<= 7;
				let v = b.at(event_ptr)?;
				time_delta |= (v & 0x7F) as u32;
				event_ptr -= 1;
				if v & 0x80 == 0x80 {
					break;
				}
				if ids + ((time_index >> 1) as isize) > event_ptr {
					event_ptr = event_ptr_start;
					break 'outer;
				}
			}
			last_event_time = last_event_time.wrapping_sub(time_delta);
			if time_index >= 16 {
				return Ok(-8);
			}
			time_index += 1;
			times[time_index] = last_event_time;
		}

		if time_index.is_multiple_of(2) && time_index > 0 {
			time_index -= 1;
			loop {
				event_ptr += 1;
				if b.at(event_ptr)? & 0x80 != 0 {
					break;
				}
			}
		}

		let sensor_byte_cnt = event_ptr - ids + 1;
		if sensor_byte_cnt > ((time_index >> 1) + 1) as isize {
			ids = event_ptr - (time_index >> 1) as isize;
		}

		// There are two time deltas per 'event' (at most 15 times here, so at most max_events sensors)
		let n_sensors = (time_index >> 1) + 1;
		debug_assert!(n_sensors <= max_events);
		for i in 0..n_sensors {
			let v = b.at(ids)?;
			sensors[i] = ((v >> 3) & 0x1F, v & 0x7);
			ids += 1;
		}

		// Step 2 - Convert events to pulses
		let mut les = [LightcapElement::default(); 8];
		let event_count = (time_index + 1) >> 1;
		let mut ti: usize = usize::MAX; // timeIndex = -1
		for i in 0..event_count {
			// (ti stays below 16: an end time at 15 leaves no room for its start, -4)
			loop {
				ti = ti.wrapping_add(1);
				if times[ti] != 0 {
					break;
				}
				if ti + 1 >= max_time_index {
					return Ok(-2);
				}
			}
			if ti >= max_time_index {
				return Ok(-3);
			}
			let start_time_index = ti + (sensors[i].1 as usize + 1);
			if start_time_index >= max_time_index {
				return Ok(-4);
			}
			if report_order[start_time_index] != 0 {
				return Ok(-5);
			}
			report_order[start_time_index] = (i + 1) as u8;
			let le = &mut les[i];
			le.sensor_id = sensors[i].0;
			le.timestamp = times[start_time_index];
			le.length = times[ti].wrapping_sub(times[start_time_index]) as u16;
			times[start_time_index] = 0;
		}

		// Output the events in ascending time order
		let mut out = 0;
		let mut output_cnt = output.len();
		for i in 0..max_time_index {
			if output_cnt == 0 {
				break;
			}
			let ordered = report_order[i];
			if ordered != 0 {
				let ol = les[ordered as usize - 1];
				if !(ol.length != 0 || ol.timestamp != 0) {
					return Ok(-6);
				}
				output[out] = ol;
				out += 1;
				output_cnt -= 1;
			}
		}
		Ok(event_count as i32)
	}

	/// parse_and_process_raw1_lightcap: gen2 light, a channel byte then 4-byte syncs and sweeps; returns the bytes read
	/// or -1
	fn parse_raw1_lightcap(
		&mut self, host: &mut dyn Host, g: &mut Globals, b: &Buf, time: u16, packet: isize, length: u8,
	) -> Result<i32, OutOfBuffer> {
		let _ = time;
		let mut has_errors = false;
		let mut idx: u8 = 0;
		let mut channel: u8 = 255;
		let mut dump_binary = false;

		while idx < length {
			let data = b.at(packet + idx as isize)?;
			if data & 0x1 != 0 {
				if (data & 0x0A) != 0 {
					let n = g.raw1_unknown_count;
					// unknown_count++: past INT_MAX it wraps, as the C's int does
					g.raw1_unknown_count = n.wrapping_add(1);
					if n < 10 {
						host.warn(&format!(
							"Not entirely sure what this data is; errors may occur ({}, 0x{:02x})\n",
							idx, data
						));
						dump_binary = true;
					}
					unread(host, packet + idx as isize, packet + length as isize);
					break;
				}
				// encodes like so: 0bcccc ?F?C
				let has_conflict = data & 0x04 != 0;
				if has_conflict {
					host.warn(&format!(
						"Two or more lighthouses are on channel {}; tracking is most likely going to fail.",
						data >> 4
					));
				}
				channel = data >> 4;
				idx = idx.wrapping_add(1);
			} else {
				let mut timecode = b.u32(packet + idx as isize)?;
				let reference_time = host.last_imu() as u32;
				let sync = timecode & 0x2 != 0;
				if !sync {
					let ootx = (timecode >> 26) & 1 != 0;
					let gbit = (timecode >> 27) & 1 != 0;
					let time24 = (timecode >> 2) & 0xFFFFFF;
					timecode = fix_time24(time24, reference_time);
					let unused = timecode >> 28;
					if unused != 0 && dump_binary {
						host.warn(&format!("Not sure what this is: {:x}", unused));
					}
					if channel == 255 {
						host.warn("No channel specified for sync");
						dump_binary = true;
					} else {
						host.sync_at((packet + idx as isize) as usize);
						host.sync(channel, timecode, ootx, gbit);
					}
				} else {
					let half_clock_flag = timecode & 0x4 != 0;
					let sensor = (timecode >> 27) as u8;
					timecode = fix_time24((timecode >> 3) & 0xFFFFFF, reference_time);
					if channel == 255 {
						host.warn("No channel specified for sweep");
						dump_binary = true;
					} else {
						let mapped = host.map_sensor_id(sensor);
						host.sweep(channel, mapped, timecode, half_clock_flag);
					}
				}
				if channel == 255 {
					dump_binary = true;
					has_errors = true;
					unread(host, packet + idx as isize, packet + length as isize);
					break;
				}
				idx = idx.wrapping_add(4);
			}
		}

		if dump_binary {
			let mut s = String::new();
			for i in 0..length as usize {
				if (i + 2) % 4 == 0 {
					s.push_str("  ");
				}
				s.push_str(&format!("{:02x} ", b.at(packet + i as isize).unwrap_or(0)));
			}
			s.push('\n');
			host.print(&s);
		}
		Ok(if has_errors { -1 } else { idx as i32 })
	}

	/// registerButtonEvent: one input report as queue entries (presses, touches, axes)
	fn register_button_event(&mut self, host: &mut dyn Host, event: &ButtonEvent, source: u32) {
		let mut entry = ButtonEntry::new();
		let subtype = host.object_subtype();

		if event.pressed_buttons_valid {
			entry = self.register_button_on_off(host, entry, subtype, event.pressed_buttons, source);
		}
		if event.touched_buttons_valid {
			entry = self.register_button_on_off(
				host,
				entry,
				subtype,
				event.touched_buttons,
				source | BUTTON_TOUCH_EVENT_SOURCE,
			);
		}
		if event.trigger_high_res_valid && self.axis[AXIS_TRIGGER as usize] != event.trigger_high_res {
			entry.event_type = input_event::AXIS_CHANGED;
			entry.ids[0] = AXIS_TRIGGER;
			entry.values[0] = event.trigger_high_res;
			entry = self.post(host, &entry);
		}
		if event.touchpad_horizontal_valid && event.touchpad_vertical_valid {
			let mut ax = AXIS_TRACKPAD_X;
			let mut ay = AXIS_TRACKPAD_Y;
			if (subtype == subtype::KNUCKLES_R || subtype == subtype::KNUCKLES_L)
				&& (self.touchmask & (1u32 << BUTTON_TRACKPAD)) == 0
			{
				if self.axis[ax as usize] != 0.0 || self.axis[ay as usize] != 0.0 {
					entry.event_type = input_event::AXIS_CHANGED;
					entry.ids[0] = ax;
					entry.ids[1] = ay;
					entry.values[0] = 0.0;
					entry.values[1] = 0.0;
					entry = self.post(host, &entry);
				}
				ax = AXIS_JOYSTICK_X;
				ay = AXIS_JOYSTICK_Y;
			}
			if self.axis[ax as usize] != event.touchpad_horizontal || self.axis[ay as usize] != event.touchpad_vertical
			{
				entry.event_type = input_event::AXIS_CHANGED;
				entry.ids[0] = ax;
				entry.ids[1] = ay;
				entry.values[0] = event.touchpad_horizontal;
				entry.values[1] = event.touchpad_vertical;
				entry = self.post(host, &entry);
			}
		}
		if event.proximity_valid {
			let mut axis_cnt = 0;
			for i in 0..6 {
				if event.proximity[i] != self.axis[AXIS_MIDDLE_FINGER_PROXIMITY as usize + i] {
					entry.event_type = input_event::AXIS_CHANGED;
					entry.ids[axis_cnt] = AXIS_MIDDLE_FINGER_PROXIMITY + i as i32;
					entry.values[axis_cnt] = event.proximity[i];
					axis_cnt += 1;
				}
			}
			if axis_cnt > 0 {
				entry = self.post(host, &entry);
			}
		}
		for i in 0..event.raw_axis_cnt as usize {
			if event.raw_axis[i] != self.axis[i] {
				entry.event_type = input_event::AXIS_CHANGED;
				entry.ids[0] = i as i32;
				entry.values[0] = event.raw_axis[i];
				entry = self.post(host, &entry);
			}
		}
		// (the C's batteryChargeValid is never set by any report: the charge comes from handle_battery)
	}

	/// registerButtonOnOff: an entry per button whose state the mask changes
	fn register_button_on_off(
		&mut self, host: &mut dyn Host, mut entry: ButtonEntry, subtype: i32, incoming_mask: u32, source: u32,
	) -> ButtonEntry {
		for a in 0u8..32 {
			let mut is_touch_event = (source & BUTTON_TOUCH_EVENT_SOURCE) != 0;
			let id = button_id_for_idx(subtype, a, &mut is_touch_event, source);
			let event_type_down = if is_touch_event { input_event::TOUCH_DOWN } else { input_event::BUTTON_DOWN };
			let event_type_up = if is_touch_event { input_event::TOUCH_UP } else { input_event::BUTTON_UP };
			// the C keeps the current mask in a uint8_t: bits 8 and up read as clear
			let current_mask: u8 =
				if event_type_down == input_event::BUTTON_DOWN { self.buttonmask as u8 } else { self.touchmask as u8 };
			let incoming = (incoming_mask & (1u32 << a)) != 0;
			if id == UNKNOWN {
				if incoming {
					host.warn(&format!("{} has unknown button input {} {}", self.codename, a, source));
				}
				continue;
			}
			let current = (current_mask as u32 & (1u32.wrapping_shl(id as u32))) != 0;
			if incoming != current {
				entry.event_type = if incoming { event_type_down } else { event_type_up };
				entry.button_id = id;
				entry = self.post(host, &entry);
			}
		}
		entry
	}

	/// incrementAndPostButtonQueue: the entry's axes and button into the device state, the entry to the host, and a
	/// fresh entry back (the C's queue never fills here: the host takes each entry at once)
	fn post(&mut self, host: &mut dyn Host, entry: &ButtonEntry) -> ButtonEntry {
		for i in 0..16 {
			if entry.ids[i] == 255 {
				break;
			}
			let id = entry.ids[i];
			if (0..16).contains(&id) {
				self.axis[id as usize] = entry.values[i];
			}
		}
		if entry.button_id != 255 {
			let is_touch = entry.event_type == input_event::TOUCH_UP || entry.event_type == input_event::TOUCH_DOWN;
			let is_clear = entry.event_type == input_event::TOUCH_UP || entry.event_type == input_event::BUTTON_UP;
			let mask = 1u32.wrapping_shl(entry.button_id as u32);
			let maskp = if is_touch { &mut self.touchmask } else { &mut self.buttonmask };
			if is_clear {
				*maskp &= !mask;
			} else {
				*maskp |= mask;
			}
		}
		host.button(entry);
		ButtonEntry::new()
	}
}

/// The bytes [from, from + n) of the buffer (as far as it reaches)
fn collect(b: &Buf, from: isize, n: isize) -> Vec<u8> {
	let mut v = Vec::new();
	let mut i = 0;
	while i < n {
		match b.at(from + i) {
			Ok(x) => v.push(x),
			Err(_) => break,
		}
		i += 1;
	}
	v
}

/// The version report's packed struct: revision, board_model, fw_name[32], hardware_id, a, fpga major (u16), minor,
/// patch, h, k (u64)
const VERSION_INFO_LEN: usize = 4 + 4 + 32 + 4 + 4 + 2 + 1 + 1 + 1 + 8;

/// parse_tracker_version_info: the version report's fields, logged. `data` is what the C's memcpy of the struct reads
/// (zeros past it when there is less); `len` the report length it prints.
fn parse_tracker_version_info(host: &mut dyn Host, codename: &str, data: &[u8], len: isize) {
	let mut v = [0u8; VERSION_INFO_LEN];
	let n = data.len().min(v.len());
	v[..n].copy_from_slice(&data[..n]);
	let u32_at = |i: usize| u32::from_le_bytes([v[i], v[i + 1], v[i + 2], v[i + 3]]);
	let revision = u32_at(0);
	let board_model = u32_at(4);
	// fw_name[33]: the 32 bytes, each to 7 bits, and a terminator ('%31s' pads to 31 but prints all 32)
	let mut fw_name = [0u8; 32];
	for i in 0..32 {
		fw_name[i] = 0x7f & v[8 + i];
	}
	let hardware_id = u32_at(40);
	let fpga_major = u16::from_le_bytes([v[48], v[49]]);
	let fpga_minor = v[50];
	let fpga_patch = v[51];
	let name_len = fw_name.iter().position(|&c| c == 0).unwrap_or(32);
	let name: String = fw_name[..name_len].iter().map(|&c| c as char).collect();
	host.info(&format!(
		"Device {} has watchman FW version {} and FPGA version {}/{}/{}; named '{:>31}'. Hardware id 0x{:08x} Board rev: {} (len {})",
		host.colorize(codename),
		revision,
		fpga_major,
		fpga_minor,
		fpga_patch,
		name,
		hardware_id,
		board_model as i32,
		len as i32
	));
	let earliest_version: u32 = 1462663157;
	let latest_version: u32 = 1632556731;
	if earliest_version > revision {
		host.warn(&format!(
			"The detected version for device {} is {}; the earliest that is verified to work is {}. You may want to update its firmware.",
			codename, revision as i32, earliest_version as i32
		));
	} else if latest_version < revision {
		host.warn(&format!(
			"The detected version for device {} is {}; the latest that was verified to work (in libsurvive) is {}.",
			codename, revision as i32, latest_version as i32
		));
	}
}

/// get_button_id_for_idx: the device's mapping of report bit `idx` to a button (and whether it is a touch)
fn button_id_for_idx(subtype: i32, idx: u8, is_touch_event: &mut bool, source: u32) -> i32 {
	let rf = (source & BUTTON_EVENT_SOURCE_RF) != 0;
	let Some(mapping) = device_mapping(subtype, rf) else { return idx as i32 };
	let button_idx = idx as usize + if (source & BUTTON_TOUCH_EVENT_SOURCE) != 0 { 32 } else { 0 };
	*is_touch_event = mapping.is_touch[button_idx];
	mapping.button_map[button_idx]
}

/// struct DeviceMapping: a device's report bits to buttons
struct DeviceMapping {
	is_touch: [bool; 64],
	button_map: [i32; 64],
}

fn base_mapping() -> DeviceMapping {
	let mut m = DeviceMapping { is_touch: [false; 64], button_map: [UNKNOWN; 64] };
	for i in 32..64 {
		m.is_touch[i] = true;
	}
	m
}

/// getDeviceMapping: by subtype, wired or RF
fn device_mapping(subtype: i32, rf: bool) -> Option<DeviceMapping> {
	use subtype::*;
	match (rf, subtype) {
		(false, WAND) => {
			let mut m = base_mapping();
			m.button_map[0] = BUTTON_TRIGGER;
			m.button_map[2] = BUTTON_GRIP;
			m.button_map[12] = BUTTON_MENU;
			m.button_map[13] = BUTTON_SYSTEM;
			m.button_map[18] = BUTTON_TRACKPAD;
			m.button_map[20] = BUTTON_TRACKPAD;
			m.is_touch[20] = true;
			Some(m)
		}
		(false, TRACKER) | (false, TRACKER_GEN2) => {
			let mut m = base_mapping();
			m.button_map[13] = BUTTON_SYSTEM;
			Some(m)
		}
		(true, WAND) => {
			let mut m = base_mapping();
			m.button_map[0] = BUTTON_TRIGGER;
			m.button_map[1] = BUTTON_TRACKPAD;
			m.is_touch[1] = true;
			m.button_map[2] = BUTTON_TRACKPAD;
			m.button_map[3] = BUTTON_SYSTEM;
			m.button_map[4] = BUTTON_GRIP;
			m.button_map[5] = BUTTON_MENU;
			Some(m)
		}
		(true, KNUCKLES_R) | (true, KNUCKLES_L) => {
			let mut m = base_mapping();
			m.button_map[0] = BUTTON_TRIGGER;
			m.button_map[1] = BUTTON_TRACKPAD;
			m.is_touch[1] = true;
			m.button_map[2] = BUTTON_THUMBSTICK;
			m.button_map[3] = BUTTON_SYSTEM;
			m.button_map[4] = BUTTON_A;
			m.button_map[5] = BUTTON_B;
			m.button_map[6] = BUTTON_MENU;
			for i in 0..32 {
				m.button_map[i + 32] = m.button_map[i];
			}
			m.button_map[1 + 32] = UNKNOWN;
			Some(m)
		}
		_ => None,
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[derive(Default)]
	struct Recorder {
		last_imu: u64,
		lightcaps: Vec<LightcapElement>,
		log: Vec<String>,
	}

	impl Host for Recorder {
		fn lh_version(&self) -> i32 {
			-1
		}
		fn last_imu(&self) -> u64 {
			self.last_imu
		}
		fn object_subtype(&self) -> i32 {
			subtype::KNUCKLES_R
		}
		fn has_config(&self) -> bool {
			true
		}
		fn map_sensor_id(&self, reported: u8) -> u8 {
			reported
		}
		fn raw_imu(&mut self, _mask: i32, _agm: &[f64; 9], _timecode: u32, _id: i32) {}
		fn register_runtime(&mut self, _time_received_us: u64) {}
		fn sync(&mut self, _channel: u8, _timecode: u32, _ootx: bool, _gen: bool) {}
		fn sweep(&mut self, _channel: u8, _sensor_id: u8, _timecode: u32, _half_clock: bool) {}
		fn lightcap(&mut self, le: &LightcapElement) -> bool {
			self.lightcaps.push(*le);
			true
		}
		fn button(&mut self, _entry: &ButtonEntry) {}
		fn notify_gen2(&mut self, _reason: &str) {}
		fn warn(&mut self, text: &str) {
			self.log.push(format!("W {text}"));
		}
		fn info(&mut self, text: &str) {
			self.log.push(format!("I {text}"));
		}
		fn print(&mut self, text: &str) {
			self.log.push(text.to_string());
		}
		fn run_time(&self) -> f64 {
			0.0
		}
		fn colorize(&self, s: &str) -> String {
			s.to_string()
		}
		fn set_imu_scale_modes(&mut self, _gyro_scale_mode: i32, _acc_scale_mode: i32) {}
		fn battery(&mut self, _charge: i8, _charging: bool) {}
	}

	#[test]
	fn time24_is_placed_near_the_reference() {
		assert_eq!(fix_time24(0x000010, 0x05FFFFF0), 0x06000010);
		assert_eq!(fix_time24(0xFFFFF0, 0x06000010), 0x05FFFFF0);
		assert_eq!(fix_time24(0x123456, 0x00123000), 0x00123456);
		// across the device clock's 32-bit wrap, both ways: light from just before it, with the reference just after
		assert_eq!(fix_time24(0xFFFFF0, 0x00000010), 0xFFFFFFF0);
		assert_eq!(fix_time24(0x000010, 0xFFFFFFF0), 0x00000010);
	}

	#[test]
	fn dump_matches_the_c_layout() {
		// a line of a replay's log (a device's first dump)
		let data = [
			0x30, 0x98, 0x58, 0x10, 0xdb, 0x02, 0xef, 0x05, 0xd6, 0x02, 0xc6, 0x08, 0xee, 0x02, 0xad, 0x65, 0x04, 0xc0,
			0x02, 0x73, 0x29, 0xc5,
		];
		let c = "0.0399950 30 98 58 10   db 02 ef 05   d6 02 c6 08   ee 02 ad 65   04 c0 02 73   29 c5                     \
		         \x20                 |    0.X.  ....  ....  ...e  ...s  ).                                  \n";
		assert_eq!(dump_buffer(0.039995, &data), c);
	}

	/// The worked example in the C's comment on read_light_data: five pulses, given back latest start first
	#[test]
	fn light_data_decodes_the_documented_packet() {
		let packet = [
			0x2b, 0x51, 0x4a, 0x28, 0x60, // sensors
			0xc7, 0x2e, 0xe6, 0x66, 0x84, 0x1f, 0xfb, 0x31, 0x0b, 0xd9, 0x01, 0xda, 0xca, 0xdb, 0x02, 0xe4,
			0x02, // deltas
			0xf6, 0xb4, 0x5b, // the last event's time
		];
		let (buf, _) = report_buffer(&packet);
		let b = Buf { b: &buf, end: BUFFER_LEN as isize };
		let mut host = Recorder { last_imu: 6010000, ..Default::default() };
		let mut parser = Parser::new("KN0");
		let mut out = [LightcapElement::default(); 10];
		let mut p = 0;
		let n = parser.read_light_data(&mut host, &b, 0x00f6, &mut p, packet.len() as isize, &mut out).ok().unwrap();
		assert_eq!(n, 5);
		let got: Vec<(u8, u32, u32)> =
			out[..5].iter().map(|le| (le.sensor_id, le.timestamp, le.timestamp + le.length as u32)).collect();
		assert_eq!(
			got,
			[
				(10, 6009325, 6009746),
				(5, 6009235, 6010102),
				(9, 6009018, 6009399),
				(5, 5818427, 5822399),
				(12, 5799310, 5805269)
			]
		);
		assert!(host.log.is_empty());
	}

	/// The count of the raw1 data the parser does not know wraps past INT_MAX as the C's int does, and its warnings
	/// start again (a build with overflow checks stopped there)
	#[test]
	fn the_unknown_raw1_count_wraps() {
		let mut host = Recorder::default();
		let mut parser = Parser::new("KN0");
		let mut g = Globals { raw1_unknown_count: i32::MAX, ..Globals::default() };
		// report 0x28 on the HMD's lightcap interface: one byte of light, a channel byte with bits 0x0A set
		let (buf, n) = report_buffer(&[0x28, 1, 0x0b]);
		parser.report(&mut host, &mut g, 0, usb_if::HMD_LIGHTCAP, &buf, n);
		assert_eq!(g.raw1_unknown_count, i32::MIN);
		assert!(host.log.is_empty());
		parser.report(&mut host, &mut g, 0, usb_if::HMD_LIGHTCAP, &buf, n);
		assert_eq!(g.raw1_unknown_count, i32::MIN + 1);
		assert_eq!(host.log, ["W Not entirely sure what this data is; errors may occur (0, 0x0b)\n", "0b \n"]);
	}
}
