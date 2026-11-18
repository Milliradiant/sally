//! The numbers: every report of a session through its device's parser (sally-hid's, the runtime's own), counted per
//! quarter second. Nothing here tracks: what a device's light and radio did is read straight from what it sent, so the
//! numbers hold whatever the base station map or the tracker would have made of them.
//!
//! Per device and quarter second:
//! - the radio: input reports, the radio packets in them (a report carries one or two), their bytes and the light's
//!   share of them, and the IMU samples against the IMU clock's own count (a sample that never arrived leaves a gap in
//!   the device's timecodes, so the loss is measured, not guessed from a nominal rate)
//! - the light: hits per base station (gen2: by channel; gen1 light is counted but not split by station) and which
//!   sensors they were on, and how old the light is when it arrives against the IMU sample before it (the radio
//!   holding light back)
//!
//! And per device over the whole session, for the sensors: each sweep across the device (one station's plane passing
//! over it, under half a millisecond) is a set of lit sensors. A sensor whose closest neighbours (normals within 30
//! degrees) are lit in a sweep should mostly be lit too, when the light comes from in front of it; one that almost
//! never is, while they are, is dead, dirty or covered. Where the light comes from is read off the sweep itself: the
//! lit sensors face the station and the dark ones face away from it, so the mean normal of the lit ones less that of
//! the dark ones points at it (the lit ones' alone leans toward the middle of the device, which has sensors on one
//! side only). A sensor that faces away from that is at the edge of what the station lights and stays dark while
//! neighbours a little nearer the station are lit: such a sweep says nothing about it.
//!
//! `Analyzer` is the tap's `Runtime`: the live tap and a replay feed it the same way.

use std::sync::Arc;
use std::time::Instant;

use sally_hid::devices::usb_if;
use sally_hid::replay::PlaybackTime;
use sally_hid::report::{ButtonEntry, Globals, Host, LightcapElement, LightcapMode, Parser, BUFFER_LEN};
use sally_hid::tap::{Identified, ObjectId, Runtime};
use sally_lighthouse::device::{ConfigOptions, DeviceConfig};

/// The length of a bin
pub const BIN_US: u64 = 250_000;
/// Bins per second
pub const BINS_PER_S: f64 = 1e6 / BIN_US as f64;
/// The most bins a session keeps (an hour)
const MAX_BINS: usize = 3600 * 4;
/// gen2 channels (the raw channel numbers 0-15; SteamVR shows them one higher)
pub const CHANNELS: usize = 16;
/// The sensors a device has room for
pub const SENSORS: usize = 32;
/// The device clock: 48 MHz
pub const TICKS_PER_MS: f64 = 48_000.0;
/// Hits closer together than this on one station are one sweep across the device
const SWEEP_TICKS: u32 = 24_000;
/// A gen1 pulse longer than this is a sync flash, not a sweep (ticks)
const GEN1_SYNC_LEN: u16 = 2_000;
/// Neighbours: sensors whose normals are within this angle of each other
const NEIGHBOUR_COS: f64 = 0.866;
/// A sweep counts for a sensor when the light comes from within this angle of its normal (45 degrees), the light's
/// direction taken from the other sensors: the mean normal of those the sweep lit less that of those it left dark
const FRONT_COS: f64 = 0.707;
/// A station sees a device in a bin with at least this many hits
pub const SEEN_HITS: u16 = 8;
/// The IMU steps kept for the device's nominal IMU interval
const STEP_SAMPLES: usize = 4_000;
/// Report ids of the radio's packets (watchman): one packet, two packets
const RF_WATCHMAN: u8 = 0x23;
const RF_WATCHMAN_X2: u8 = 0x24;

/// A vector at unit length (zero stays zero)
fn unit(v: [f64; 3]) -> [f64; 3] {
	let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
	if len > 0.0 {
		[v[0] / len, v[1] / len, v[2] / len]
	} else {
		v
	}
}

/// A quarter second of a device
#[derive(Clone, Debug, Default)]
pub struct Bin {
	/// input reports from the dongle
	pub reports: u16,
	/// radio packets in them, and the reports that carried two
	pub packets: u16,
	pub doubled: u16,
	/// the packets' payload bytes, and roughly how many of them were light
	pub bytes: u32,
	pub light_bytes: u32,
	/// IMU samples, and how far the IMU clock moved across them (device ticks)
	pub imu: u16,
	pub imu_span: u32,
	/// gen2 hits per channel, and which sensors they were on
	pub hits: [u16; CHANNELS],
	pub sensors: [u32; CHANNELS],
	/// gen1 sweep hits (not split by station) and their sensors
	pub gen1_hits: u16,
	pub gen1_sensors: u32,
	/// the light's age when it arrived against the latest IMU sample (ms): sum, count, largest
	pub delay_sum: f32,
	pub delay_n: u16,
	pub delay_max: f32,
}

impl Bin {
	/// The stations that saw the device in this bin
	pub fn stations(&self) -> impl Iterator<Item = usize> + '_ {
		(0..CHANNELS).filter(|&c| self.hits[c] >= SEEN_HITS)
	}
	pub fn station_count(&self) -> usize {
		self.stations().count()
	}
	/// Hits of all stations (and gen1)
	pub fn all_hits(&self) -> u32 {
		self.hits.iter().map(|&h| h as u32).sum::<u32>() + self.gen1_hits as u32
	}
	/// Sensors lit by any station
	pub fn sensors_lit(&self) -> u32 {
		(self.sensors.iter().fold(self.gen1_sensors, |m, &s| m | s)).count_ones()
	}
	pub fn delay_mean(&self) -> Option<f32> {
		(self.delay_n > 0).then(|| self.delay_sum / self.delay_n as f32)
	}
}

/// What a device's config says, as far as the check needs it
#[derive(Clone, Debug)]
pub struct Config {
	/// the device's own serial (LHR-...)
	pub serial: String,
	pub subtype: i32,
	pub sensor_ct: usize,
	channel_map: Option<[i32; 32]>,
	pub normals: Option<[[f64; 3]; 32]>,
}

impl Config {
	fn from(c: &DeviceConfig) -> Config {
		let end = c.serial_number.iter().position(|&b| b == 0).unwrap_or(c.serial_number.len());
		Config {
			serial: String::from_utf8_lossy(&c.serial_number[..end]).into_owned(),
			subtype: c.object_subtype as i32,
			sensor_ct: c.sensor_ct.min(SENSORS),
			channel_map: c.channel_map,
			normals: c.sensor_normals,
		}
	}

	/// survive_map_sensor_id, as the runtime has it
	fn map_sensor_id(&self, reported: u8) -> u8 {
		if let Some(map) = &self.channel_map {
			let mapped = map.get(reported as usize).copied().unwrap_or(-1) as u8;
			if mapped as usize >= self.sensor_ct {
				return 255;
			}
			return mapped;
		}
		reported
	}

	/// What kind of device the config is for
	pub fn kind(&self) -> &'static str {
		match self.subtype {
			1 | 2 => "headset",
			3 => "controller",
			4 | 5 => "controller",
			6 | 7 => "tracker",
			_ => "device",
		}
	}
}

/// One station's sweep across the device, as it builds up
#[derive(Clone, Copy, Debug, Default)]
struct Sweep {
	active: bool,
	start: u32,
	mask: u32,
}

/// One device (one dongle's stream)
#[derive(Debug)]
pub struct Device {
	/// the dongle's serial, as its HID attributes give it
	pub dongle: String,
	/// the tap's name for it (KN0, ...)
	pub name: String,
	pub config: Option<Config>,
	pub bins: Vec<Bin>,
	/// hits per sensor over the session
	pub sensor_hits: [u64; SENSORS],
	/// sweeps in which a sensor's neighbours were lit, and those in which it was lit too
	pub eligible: [u32; SENSORS],
	pub eligible_lit: [u32; SENSORS],
	/// each sensor's neighbours (by normal), once the config is in
	pub neighbours: Vec<Vec<usize>>,
	/// each sensor's normal, unit length (zero where the config has none)
	normals: Vec<[f64; 3]>,
	/// the charge it reported (%), with the bin it came in (a device's first values are placeholders: see
	/// `battery_min`)
	pub battery: Vec<(usize, i8)>,
	/// "two or more stations on one channel" warnings from the parser
	pub channel_conflicts: u32,
	/// the light modes its reports were parsed in (raw0 = gen1 / before SteamVR switched it, raw1 = gen2)
	pub raw0_reports: u64,
	pub raw1_reports: u64,
	imu_last: Option<u32>,
	imu_steps: Vec<u32>,
	sweeps: [Sweep; CHANNELS + 1],
}

impl Device {
	pub(crate) fn new(name: &str, dongle: &str) -> Device {
		Device {
			dongle: dongle.to_string(),
			name: name.to_string(),
			config: None,
			bins: Vec::new(),
			sensor_hits: [0; SENSORS],
			eligible: [0; SENSORS],
			eligible_lit: [0; SENSORS],
			neighbours: Vec::new(),
			normals: Vec::new(),
			battery: Vec::new(),
			channel_conflicts: 0,
			raw0_reports: 0,
			raw1_reports: 0,
			imu_last: None,
			imu_steps: Vec::new(),
			sweeps: [Sweep::default(); CHANNELS + 1],
		}
	}

	fn set_config(&mut self, c: Config) {
		self.neighbours = (0..c.sensor_ct)
			.map(|s| match &c.normals {
				Some(n) => (0..c.sensor_ct)
					.filter(|&t| t != s && n[s][0] * n[t][0] + n[s][1] * n[t][1] + n[s][2] * n[t][2] > NEIGHBOUR_COS)
					.collect(),
				None => Vec::new(),
			})
			.collect();
		self.normals = (0..c.sensor_ct).map(|s| c.normals.map_or([0.0; 3], |n| unit(n[s]))).collect();
		self.config = Some(c);
	}

	/// The serial people know the device by: its own, or its dongle's while there is no config
	pub fn serial(&self) -> &str {
		self.config.as_ref().map_or(&self.dongle, |c| &c.serial)
	}

	/// The device's usual IMU interval in ticks (the median step)
	pub fn imu_step(&self) -> Option<f64> {
		if self.imu_steps.len() < 50 {
			return None;
		}
		let mut s = self.imu_steps.clone();
		s.sort_unstable();
		Some(s[s.len() / 2] as f64)
	}

	/// The bins from two seconds after the device's first light to its last: before it, SteamVR may still be setting
	/// the device up (switching its light mode, its protocol: its IMU runs at half rate for most of a second then), and
	/// after it, it is being switched off
	pub fn active(&self) -> std::ops::Range<usize> {
		let lit = |b: &Bin| b.all_hits() >= SEEN_HITS as u32;
		match (self.bins.iter().position(lit), self.bins.iter().rposition(lit)) {
			(Some(a), Some(b)) if b > a + 16 => a + 8..b + 1,
			(Some(a), Some(b)) => a..b + 1,
			_ => 0..self.bins.len(),
		}
	}

	/// The lowest charge it reported, leaving out its first 10 s of reports (placeholders like 1 % come then)
	pub fn battery_min(&self) -> Option<i8> {
		let first = self.bins.iter().position(|b| b.reports > 0)?;
		self.battery.iter().filter(|&&(bin, _)| bin >= first + 40).map(|&(_, c)| c).min()
	}

	/// IMU samples lost in a bin (estimated from how far the IMU clock moved)
	pub fn imu_lost(&self, b: &Bin, step: f64) -> f64 {
		if b.imu_span == 0 {
			return 0.0;
		}
		(b.imu_span as f64 / step - b.imu as f64).max(0.0)
	}

	fn bin(&mut self, i: usize) -> &mut Bin {
		if self.bins.len() <= i {
			self.bins.resize(i + 1, Bin::default());
		}
		&mut self.bins[i]
	}

	/// A hit on `sensor` by the station on `slot` (a gen2 channel, or CHANNELS for gen1) at device time `t`
	fn sweep_hit(&mut self, slot: usize, sensor: u8, t: u32) {
		if sensor as usize >= SENSORS {
			return;
		}
		let sw = self.sweeps[slot];
		if sw.active && t.wrapping_sub(sw.start) > SWEEP_TICKS {
			self.close_sweep(slot);
		}
		let sw = &mut self.sweeps[slot];
		if !sw.active {
			*sw = Sweep { active: true, start: t, mask: 0 };
		}
		sw.mask |= 1 << sensor;
	}

	fn close_sweep(&mut self, slot: usize) {
		let mask = self.sweeps[slot].mask;
		self.sweeps[slot].active = false;
		for s in 0..self.neighbours.len() {
			let nb = &self.neighbours[s];
			if nb.is_empty() {
				continue;
			}
			let lit = nb.iter().filter(|&&t| mask & (1 << t) != 0).count();
			// most of them, all of them when there are two or fewer
			let need = if nb.len() <= 2 { nb.len() } else { (nb.len() * 6).div_ceil(10) };
			if lit >= need && self.lit_from_the_front(s, mask) {
				self.eligible[s] += 1;
				if mask & (1 << s) != 0 {
					self.eligible_lit[s] += 1;
				}
			}
		}
	}

	/// Whether a sweep's light came from in front of sensor `s`, within 45 degrees of its normal: the light's direction
	/// from the other sensors, the mean normal of the lit ones less that of the dark ones (whether `s` itself was lit
	/// must not decide whether the sweep counts for it). Without normals (no config yet) every sweep counts, as its
	/// neighbours alone decide
	fn lit_from_the_front(&self, s: usize, mask: u32) -> bool {
		let Some(n) = self.normals.get(s) else {
			return true;
		};
		let (mut lit, mut dark, mut lit_ct, mut dark_ct) = ([0.0; 3], [0.0; 3], 0.0, 0.0);
		for (t, m) in self.normals.iter().enumerate() {
			if t == s {
				continue;
			}
			let (sum, ct) = if mask & (1 << t) != 0 { (&mut lit, &mut lit_ct) } else { (&mut dark, &mut dark_ct) };
			*sum = [sum[0] + m[0], sum[1] + m[1], sum[2] + m[2]];
			*ct += 1.0;
		}
		let (l, d) = (1.0 / f64::max(lit_ct, 1.0), 1.0 / f64::max(dark_ct, 1.0));
		let t = [lit[0] * l - dark[0] * d, lit[1] * l - dark[1] * d, lit[2] * l - dark[2] * d];
		let len = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
		len > 0.0 && (n[0] * t[0] + n[1] * t[1] + n[2] * t[2]) / len >= FRONT_COS
	}

	/// The sweeps still open, closed (the end of the session)
	pub fn finish(&mut self) {
		for slot in 0..=CHANNELS {
			if self.sweeps[slot].active {
				self.close_sweep(slot);
			}
		}
	}
}

/// Where the device configs come from: a dongle's serial to the config JSON of the device paired to it, and a
/// device's own serial to its config (for a recording's line that says which device a dongle carries)
pub type ConfigSource = Box<dyn FnMut(&str) -> Option<Vec<u8>> + Send>;

/// A station's sync as the parser handed it on, and where its entry stands in the report
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SyncSeen {
	pub obj: ObjectId,
	pub channel: u8,
	pub timecode: u32,
	/// the station's data bit (its OOTX bit)
	pub data: bool,
	/// the entry's first byte, counted from the report's id
	pub at: usize,
}

/// What `share` needs of each report to change a recording where it stands (`Analyzer::watch`): the syncs of the
/// report last read, the bytes of it that are not read as gen2 light, and a digest of everything the parser has
/// handed on so far but the syncs' data bits and the pulses a 1.0 station's light would be: two recordings with the
/// same digest are the same to sally
#[derive(Debug, Default)]
pub struct Seen {
	pub syncs: Vec<SyncSeen>,
	/// (the last byte read, the bytes from it to the end of what is passed over)
	pub unread: Vec<(usize, usize)>,
	pub digest: u64,
	/// the reports that reached a device
	pub reports: u64,
	at: usize,
}

impl Seen {
	fn add(&mut self, words: &[u64]) {
		for w in words {
			for b in w.to_le_bytes() {
				self.digest = (self.digest ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3);
			}
		}
	}
}

/// The radio's report ids, the wired light's gen2 one, and the report's slot for a second packet
const USB_LIGHTCAP_RAW1: u8 = 0x28;
const SECOND_PACKET: usize = 30;

/// The tap's runtime for the check
pub struct Analyzer {
	pub devices: Vec<Device>,
	/// the first report's time and the last's (the recording's clock, microseconds)
	pub start_us: Option<u64>,
	pub last_us: u64,
	/// the tap's and the parser's messages (the first few hundred)
	pub log: Vec<String>,
	configs: ConfigSource,
	time: Option<Arc<PlaybackTime>>,
	started: Instant,
	/// `watch`: the reports read as the runtime reads them, and what `share` needs of each
	seen: Option<Seen>,
}

impl Analyzer {
	pub fn new(configs: ConfigSource) -> Analyzer {
		Analyzer {
			devices: Vec::new(),
			start_us: None,
			last_us: 0,
			log: Vec::new(),
			configs,
			time: None,
			started: Instant::now(),
			seen: None,
		}
	}

	/// An analyzer that reads each report as the runtime does (a device's light as pulses until its mode is known,
	/// as gen2 light from then on), and keeps what `share` needs of each (`seen`)
	pub fn watch(configs: ConfigSource) -> Analyzer {
		Analyzer { seen: Some(Seen { digest: 0xcbf2_9ce4_8422_2325, ..Seen::default() }), ..Analyzer::new(configs) }
	}

	/// What the report last read gave `share`, and the digest so far (None unless made by `watch`)
	pub fn seen(&mut self) -> Option<&mut Seen> {
		self.seen.as_mut()
	}

	/// How many reports have reached a device (made by `watch`; else 0)
	pub fn reports_read(&self) -> u64 {
		self.seen.as_ref().map_or(0, |s| s.reports)
	}

	/// The session's length in bins
	pub fn bins(&self) -> usize {
		self.devices.iter().map(|d| d.bins.len()).max().unwrap_or(0)
	}

	/// The open sweeps closed
	pub fn finish(&mut self) {
		for d in &mut self.devices {
			d.finish();
		}
	}

	fn note(&mut self, text: &str) {
		if self.log.len() < 400 {
			self.log.push(text.trim_end().to_string());
		}
	}

	fn configure(&mut self, obj: ObjectId, json: &[u8]) -> bool {
		let name = self.devices[obj].name.clone();
		match DeviceConfig::parse(json, &name, &ConfigOptions::default()) {
			Ok(c) => {
				let c = Config::from(&c);
				self.note(&format!(
					"{} ({}): config of {} ({}, {} sensors)",
					name,
					self.devices[obj].dongle,
					c.serial,
					c.kind(),
					c.sensor_ct
				));
				self.devices[obj].set_config(c);
				true
			}
			Err(e) => {
				self.note(&format!("{name}: config not usable: {e}"));
				false
			}
		}
	}
}

/// The radio packets in a watchman report: (payload bytes, light bytes) for each
fn radio_packets(buffer: &[u8], len: usize) -> ([(u32, u32); 2], usize) {
	let mut out = [(0, 0); 2];
	let offsets: &[usize] = match buffer.first() {
		Some(&RF_WATCHMAN) => &[1],
		Some(&RF_WATCHMAN_X2) => &[1, 30],
		_ => return (out, 0),
	};
	let mut n = 0;
	for &o in offsets {
		if o + 3 >= len {
			break;
		}
		let size = buffer[o + 1];
		if size < 2 {
			continue;
		}
		let payload = (size - 1) as u32;
		let flags = buffer[o + 3];
		// flags, then the IMU sample (7 bytes) when there is one; light makes up most of what remains
		let light =
			if flags & 0x10 != 0 { payload.saturating_sub(1 + if flags & 0x80 != 0 { 7 } else { 0 }) } else { 0 };
		out[n] = (payload, light);
		n += 1;
	}
	(out, n)
}

/// The bytes of a report the parser does not get to at all, as `Host::light_unread` gives the ones it stops at: a
/// radio report that is not one or two packets, a wired light report that is not gen2 light, and every such report
/// of a device without its config. Each from the last byte something may read (the report's id; in a packet the
/// byte after its flags, which the tap's look at the light mode reads)
fn passed_over(iface: i32, report: &[u8], configured: bool) -> Vec<(usize, usize)> {
	let Some(&id) = report.first() else { return Vec::new() };
	let whole = vec![(0, report.len())];
	match iface {
		usb_if::WATCHMAN1 | usb_if::WATCHMAN2 => {
			let slots: &[usize] = match id {
				RF_WATCHMAN => &[1],
				RF_WATCHMAN_X2 => &[1, SECOND_PACKET],
				_ => return whole,
			};
			if configured {
				return Vec::new();
			}
			slots
				.iter()
				.filter(|&&o| o + 4 < report.len())
				.map(|&o| (o + 4, (report[o + 1] as usize).saturating_sub(2).min(report.len() - o - 4)))
				.collect()
		}
		usb_if::HMD_LIGHTCAP | usb_if::TRACKER1_LIGHTCAP if configured && id == USB_LIGHTCAP_RAW1 => Vec::new(),
		usb_if::HMD_LIGHTCAP | usb_if::TRACKER0_LIGHTCAP | usb_if::TRACKER1_LIGHTCAP | usb_if::W_WATCHMAN1_LIGHTCAP => {
			whole
		}
		_ => Vec::new(),
	}
}

/// The parser's host for one report of one device
struct HostFor<'a> {
	dev: &'a mut Device,
	obj: ObjectId,
	bin: usize,
	lh_version: i32,
	run_time: f64,
	seen: Option<&'a mut Seen>,
}

impl Host for HostFor<'_> {
	fn lh_version(&self) -> i32 {
		self.lh_version
	}
	fn last_imu(&self) -> u64 {
		self.dev.imu_last.unwrap_or(0) as u64
	}
	fn object_subtype(&self) -> i32 {
		self.dev.config.as_ref().map_or(0, |c| c.subtype)
	}
	fn has_config(&self) -> bool {
		self.dev.config.is_some()
	}
	fn map_sensor_id(&self, reported: u8) -> u8 {
		self.dev.config.as_ref().map_or(reported, |c| c.map_sensor_id(reported))
	}
	fn raw_imu(&mut self, mask: i32, agm: &[f64; 9], timecode: u32, id: i32) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[1, self.obj as u64, mask as u64, timecode as u64, id as u64]);
			s.add(&agm.map(f64::to_bits));
		}
		if id != 0 {
			return;
		}
		let step = self.dev.imu_last.map(|l| timecode.wrapping_sub(l));
		self.dev.imu_last = Some(timecode);
		let bin = self.bin;
		let b = self.dev.bin(bin);
		b.imu = b.imu.saturating_add(1);
		// a step backwards or over a second is a clock jump, not a gap
		if let Some(step) = step.filter(|&s| s > 0 && (s as f64) < 1000.0 * TICKS_PER_MS) {
			b.imu_span = b.imu_span.saturating_add(step);
			if self.dev.imu_steps.len() < STEP_SAMPLES {
				self.dev.imu_steps.push(step);
			}
		}
	}
	fn register_runtime(&mut self, _time_received_us: u64) {}
	fn sync(&mut self, channel: u8, timecode: u32, ootx: bool, gen: bool) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[2, self.obj as u64, channel as u64, timecode as u64, gen as u64]);
			let at = s.at;
			s.syncs.push(SyncSeen { obj: self.obj, channel, timecode, data: ootx, at });
		}
	}
	fn sync_at(&mut self, at: usize) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.at = at;
		}
	}
	fn light_unread(&mut self, at: usize, len: usize) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.unread.push((at, len));
		}
	}
	fn sweep(&mut self, channel: u8, sensor_id: u8, timecode: u32, half_clock: bool) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[3, self.obj as u64, channel as u64, sensor_id as u64, timecode as u64, half_clock as u64]);
		}
		let c = channel as usize;
		if c >= CHANNELS || sensor_id as usize >= SENSORS {
			return;
		}
		let delay = self.dev.imu_last.map(|l| l.wrapping_sub(timecode) as i32 as f64 / TICKS_PER_MS);
		let bin = self.bin;
		let b = self.dev.bin(bin);
		b.hits[c] = b.hits[c].saturating_add(1);
		b.sensors[c] |= 1 << sensor_id;
		if let Some(d) = delay.filter(|d| d.abs() < 1000.0) {
			let d = d.max(0.0) as f32;
			b.delay_sum += d;
			b.delay_n = b.delay_n.saturating_add(1);
			b.delay_max = b.delay_max.max(d);
		}
		self.dev.sensor_hits[sensor_id as usize] += 1;
		self.dev.sweep_hit(c, sensor_id, timecode);
	}
	fn lightcap(&mut self, le: &LightcapElement) -> bool {
		if le.length >= GEN1_SYNC_LEN || le.sensor_id as usize >= SENSORS {
			return true;
		}
		let bin = self.bin;
		let b = self.dev.bin(bin);
		b.gen1_hits = b.gen1_hits.saturating_add(1);
		b.gen1_sensors |= 1 << le.sensor_id;
		self.dev.sensor_hits[le.sensor_id as usize] += 1;
		self.dev.sweep_hit(CHANNELS, le.sensor_id, le.timestamp);
		true
	}
	fn button(&mut self, entry: &ButtonEntry) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[7, self.obj as u64]);
			for b in format!("{entry:?}").bytes() {
				s.add(&[b as u64]);
			}
		}
	}
	fn notify_gen2(&mut self, _reason: &str) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[8, self.obj as u64]);
		}
	}
	fn warn(&mut self, text: &str) {
		if text.contains("Two or more lighthouses are on channel") {
			self.dev.channel_conflicts += 1;
		}
	}
	fn info(&mut self, _text: &str) {}
	fn print(&mut self, _text: &str) {}
	fn run_time(&self) -> f64 {
		self.run_time
	}
	fn colorize(&self, s: &str) -> String {
		s.to_string()
	}
	fn set_imu_scale_modes(&mut self, gyro_scale_mode: i32, acc_scale_mode: i32) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[5, self.obj as u64, gyro_scale_mode as u64, acc_scale_mode as u64]);
		}
	}
	fn battery(&mut self, charge: i8, charging: bool) {
		if let Some(s) = self.seen.as_deref_mut() {
			s.add(&[6, self.obj as u64, charge as u64, charging as u64]);
		}
		if self.dev.battery.len() < 10_000 {
			self.dev.battery.push((self.bin, charge));
		}
	}
}

impl Runtime for Analyzer {
	fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId> {
		self.devices.push(Device::new(name, serial));
		Some(self.devices.len() - 1)
	}

	fn load_config_file(&mut self, obj: ObjectId, filename: &str) -> bool {
		let dongle = filename.strip_suffix("_config.json").unwrap_or(filename).to_string();
		match (self.configs)(&dongle) {
			Some(json) => self.configure(obj, &json),
			None => false,
		}
	}

	fn config(&mut self, obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
		match json {
			Some(j) if self.configure(obj, &j) => 0,
			_ => -1,
		}
	}

	fn report(
		&mut self, obj: ObjectId, parser: &mut Parser, globals: &mut Globals, time_us: u64, iface: i32,
		buffer: &[u8; BUFFER_LEN], len: usize,
	) {
		let start = *self.start_us.get_or_insert(time_us);
		self.last_us = self.last_us.max(time_us);
		let mut bin = (time_us.saturating_sub(start) / BIN_US) as usize;
		if let Some(s) = &mut self.seen {
			// every report is read, however long the recording: the counts past the last bin go into it
			bin = bin.min(MAX_BINS - 1);
			s.reports += 1;
			s.syncs.clear();
			s.unread = passed_over(iface, &buffer[..len.min(BUFFER_LEN)], self.devices[obj].config.is_some());
		}
		if bin >= MAX_BINS {
			return;
		}
		let run_time = self.run_time();
		let dev = &mut self.devices[obj];
		match parser.lightcap_mode {
			LightcapMode::Raw0 => dev.raw0_reports += 1,
			LightcapMode::Raw1 => dev.raw1_reports += 1,
			_ => {}
		}
		let (packets, n) = radio_packets(&buffer[..], len);
		let b = dev.bin(bin);
		b.reports = b.reports.saturating_add(1);
		b.packets = b.packets.saturating_add(n as u16);
		if n == 2 {
			b.doubled = b.doubled.saturating_add(1);
		}
		for &(bytes, light) in &packets[..n] {
			b.bytes += bytes;
			b.light_bytes += light;
		}
		let seen = self.seen.as_mut();
		// the check takes a device's light as pulses until its mode is known (0); read as the runtime reads it, the
		// generation is not known until then (-1), and the pulses read meanwhile are not sally's to use
		let lh_version = match (parser.lightcap_mode == LightcapMode::Raw1, seen.is_some()) {
			(true, _) => 1,
			(false, true) => -1,
			(false, false) => 0,
		};
		let mut host = HostFor { dev, obj, bin, lh_version, run_time, seen };
		parser.report(&mut host, globals, time_us, iface, buffer, len);
	}

	/// The tap's own mode switch (it follows Valve's): the parser's bookkeeping only, nothing is sent
	fn setup_write(
		&mut self, obj: ObjectId, parser: &mut Parser, bm_request_type: u8, b_request: u8, w_value: u16, data: &[u8],
	) {
		let run_time = self.run_time();
		let mut host = HostFor { dev: &mut self.devices[obj], obj, bin: 0, lh_version: 1, run_time, seen: None };
		parser.on_setup_write(&mut host, bm_request_type, b_request, w_value, data);
	}

	fn feature_read(&mut self, obj: ObjectId, parser: &Parser, data: &[u8]) {
		let run_time = self.run_time();
		let mut host = HostFor { dev: &mut self.devices[obj], obj, bin: 0, lh_version: 1, run_time, seen: None };
		parser.feature_read(&mut host, data);
	}

	fn install_run_time_fn(&mut self, time: Option<Arc<PlaybackTime>>) {
		self.time = time;
	}

	fn run_time(&self) -> f64 {
		match &self.time {
			Some(t) => t.get(),
			None => self.started.elapsed().as_secs_f64(),
		}
	}

	fn info(&mut self, text: &str) {
		self.note(text);
	}

	fn warn(&mut self, text: &str) {
		self.note(text);
	}

	fn error(&mut self, _code: i32, text: &str) {
		self.note(text);
	}

	/// The device a recording's line names for the object's dongle: its config, from where the dongles' are (the
	/// source is asked by the device's own serial)
	fn identify(&mut self, obj: ObjectId, device: &str) -> Identified {
		if self.devices[obj].config.as_ref().is_some_and(|c| c.serial.eq_ignore_ascii_case(device)) {
			return Identified::Same;
		}
		match (self.configs)(device) {
			Some(json) if self.configure(obj, &json) => Identified::Configured(json),
			_ => Identified::Failed,
		}
	}

	fn release(&mut self, obj: ObjectId) {
		let d = &mut self.devices[obj];
		d.config = None;
		d.neighbours.clear();
	}

	fn object_serial(&self, obj: ObjectId) -> Option<String> {
		self.devices[obj].config.as_ref().map(|c| c.serial.clone())
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn radio_packets_split_as_the_parser_does() {
		let mut r = [0u8; 59];
		r[0] = RF_WATCHMAN_X2;
		// first packet: 20 payload bytes with IMU and light; second: 8 bytes, IMU only
		r[2] = 21;
		r[4] = 0x90;
		r[31] = 9;
		r[33] = 0x80;
		let (p, n) = radio_packets(&r, 59);
		assert_eq!(n, 2);
		assert_eq!(p[0], (20, 12));
		assert_eq!(p[1], (8, 0));
		r[0] = RF_WATCHMAN;
		assert_eq!(radio_packets(&r, 59).1, 1);
		r[0] = 0x26;
		assert_eq!(radio_packets(&r, 59).1, 0);
	}

	#[test]
	fn a_sensor_dark_while_its_neighbours_are_lit_is_counted() {
		let mut d = Device::new("KN0", "X");
		d.neighbours = vec![vec![1, 2], vec![0, 2], vec![0, 1]];
		// sensors 1 and 2 lit together ten times, sensor 0 never
		for k in 0..10u32 {
			let t = k * 1_000_000;
			d.sweep_hit(0, 1, t);
			d.sweep_hit(0, 2, t + 10);
		}
		d.finish();
		assert_eq!(d.eligible[0], 10);
		assert_eq!(d.eligible_lit[0], 0);
		// 1 and 2 each need both neighbours, and 0 is never lit
		assert_eq!(d.eligible[1], 0);
	}

	/// A dome of 19 sensors: one on the axis (z), six 40 degrees off it and twelve 80 degrees off it
	fn dome() -> Device {
		let at = |off: f64, around: f64| {
			let (o, a) = (off.to_radians(), around.to_radians());
			[o.sin() * a.cos(), o.sin() * a.sin(), o.cos()]
		};
		let mut d = Device::new("KN0", "X");
		d.normals = vec![at(0.0, 0.0)];
		d.normals.extend((0..6).map(|k| at(40.0, 60.0 * k as f64)));
		d.normals.extend((0..12).map(|k| at(80.0, 30.0 * k as f64)));
		let n = d.normals.clone();
		let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		d.neighbours =
			(0..n.len()).map(|s| (0..n.len()).filter(|&t| t != s && dot(n[s], n[t]) > 0.7).collect()).collect();
		d
	}

	/// Ten sweeps of a station in direction `from`: every sensor within 65 degrees of it lit, but `dead`
	fn sweeps(d: &mut Device, from: [f64; 3], dead: Option<usize>) {
		let n = d.normals.clone();
		for k in 0..10u32 {
			for (s, m) in n.iter().enumerate() {
				if m[0] * from[0] + m[1] * from[1] + m[2] * from[2] > 65f64.to_radians().cos() && Some(s) != dead {
					d.sweep_hit(0, s as u8, k * 1_000_000 + s as u32);
				}
			}
		}
		d.finish();
	}

	#[test]
	fn a_sweep_from_the_side_says_nothing_about_a_sensor_at_its_edge() {
		// a station level with the dome's rim, in the direction of rim sensor 7: the rim sensors 9 and 17, 60 degrees
		// round from 7, are in its light, and 10 and 16, 90 degrees round, are just out of it, with their neighbours
		// 9 and 17 lit. The light is 89 degrees off their normals: those sweeps do not count for them
		let mut d = dome();
		sweeps(&mut d, [1.0, 0.0, 0.0], None);
		assert_eq!((d.eligible[10], d.eligible[16]), (0, 0));
		// and a sensor in the middle of that light is held to it
		assert_eq!((d.eligible[7], d.eligible_lit[7]), (10, 10));
	}

	#[test]
	fn a_sensor_dark_in_the_middle_of_the_light_is_counted_against() {
		// the same station with sensor 7, the one that faces it, dead: every sweep counts for it, none lit
		let mut d = dome();
		sweeps(&mut d, [1.0, 0.0, 0.0], Some(7));
		assert_eq!((d.eligible[7], d.eligible_lit[7]), (10, 0));
		// from above, the sensor on the axis dead
		let mut d = dome();
		sweeps(&mut d, [0.0, 0.0, 1.0], Some(0));
		assert_eq!((d.eligible[0], d.eligible_lit[0]), (10, 0));
	}
}
