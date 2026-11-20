//! The runtime's client (docs/sally_runtime_protocol.md): subscribes to the driver, state and diag streams, keeps the
//! latest picture of the devices, the stations, the coverage under them and an event log, and sends commands. R and T
//! lines are not read (the runtime sends no R lines, and T's live fields are S's).
//!
//! The panel's clock (`now`, seconds) is the host's, handed to `poll`; events are stamped with the wall clock as well.

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};

use sally_math::math3d::{len, qconj, qnorm, qrot, Pose, Q4, V3};
use sally_math::{c_atof, c_atoi};

use crate::history::{History, Light, Sample};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvLevel {
	Info,
	Good,
	Warn,
	Bad,
}

#[derive(Clone, Debug)]
pub struct Event {
	/// the wall clock when it happened (Unix seconds)
	pub clock: i64,
	/// the runtime's clock when known, else -1
	pub run_time: f64,
	pub level: EvLevel,
	/// a device's serial, a station, "runtime", "panel", "map", "SteamVR"
	pub who: String,
	pub text: String,
	/// shown in the important view (state changes, stations, diagnosis, the runtime itself)
	pub matters: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct TrailPoint {
	pub p: V3,
	/// the panel's clock
	pub t: f64,
	/// 0 tracking, 1 dark, 2 coasting
	pub kind: u8,
}

/// A device's light from one station, in the last H period
#[derive(Clone, Copy, Debug, Default)]
pub struct StationLight {
	/// hits summed over the sensors
	pub hits: i32,
	/// sensors with any hit
	pub sensors_lit: i32,
}

/// One H period of a device: each station's hits on each of its sensors, and where the device was when the period's
/// lines came (the period's end: the runtime sends them as it closes the period)
#[derive(Clone, Debug, Default)]
pub struct LightPeriod {
	/// the panel's clock when its first line came
	pub wall: f64,
	/// the period's length (s), as the H lines give it
	pub period: f64,
	/// the device's pose then (its head, in the runtime's world), and whether it was tracking with a fresh pose
	pub pose: Pose,
	pub tracking: bool,
	/// by station index: hits per sensor
	pub hits: BTreeMap<i32, Vec<i32>>,
}

impl LightPeriod {
	/// A station's hits on a sensor, 0 where it sent none
	pub fn hits_of(&self, lh: i32, sensor: usize) -> i32 {
		self.hits.get(&lh).and_then(|v| v.get(sensor)).copied().unwrap_or(0)
	}
}

/// The runtime's own verdict on a device (protocol 4 D and C lines, crates/sally-runtime/src/diagnosis.rs)
#[derive(Clone, Debug)]
pub struct Diagnosis {
	/// the panel's clock at the last D line
	pub wall: f64,
	/// fine, radio, station, device, light, reflect
	pub headline: String,
	/// 0 fine, 1 hurting a little, 2 hurting
	pub severity: i32,
	/// radio signal 0-4, -1 unknown
	pub bars: i32,
	pub loss_pct: f64,
	pub gap_ms: f64,
	pub stations: i32,
	pub sensors: i32,
	pub back_pct: f64,
	pub gated: i32,
	pub discards: i32,
	pub charge: i32,
	/// (cause, text), worst first as the runtime sends them
	pub causes: Vec<(String, String)>,
	/// being collected until the next D line
	causes_next: Vec<(String, String)>,
	/// the headline's text last logged, to log changes only
	last_text: String,
	pub last_change: f64,
	// a verdict is logged once it has held for 2 s (fine: 5 s), and the mild ones never are
	cand_head: String,
	logged_head: String,
	cand_sev: i32,
	logged_sev: i32,
	cand_since: f64,
	last_log: f64,
	log_pending: bool,
}

impl Default for Diagnosis {
	fn default() -> Diagnosis {
		Diagnosis {
			wall: -1e9,
			headline: "fine".into(),
			severity: 0,
			bars: -1,
			loss_pct: 0.0,
			gap_ms: 0.0,
			stations: 0,
			sensors: 0,
			back_pct: 0.0,
			gated: 0,
			discards: 0,
			charge: -1,
			causes: Vec::new(),
			causes_next: Vec::new(),
			last_text: String::new(),
			last_change: -1e9,
			cand_head: "fine".into(),
			logged_head: "fine".into(),
			cand_sev: 0,
			logged_sev: 0,
			cand_since: -1e9,
			last_log: -1e9,
			log_pending: false,
		}
	}
}

#[derive(Clone, Debug)]
pub struct Device {
	pub codename: String,
	/// "-" until the runtime knows it
	pub serial: String,
	/// generic, vive_hmd, index_hmd, wand, knuckles_r, knuckles_l, tracker, tracker_gen2
	pub subtype: String,
	/// the USB serial of the dongle it talks through, "" when unknown
	pub dongle: String,
	pub sensor_ct: i32,
	pub have_pose: bool,
	pub pos: V3,
	pub vel: V3,
	pub angvel: V3,
	pub rot: Q4,
	/// the runtime's clock at the last pose
	pub run_time: f64,
	pub last_pose_wall: f64,
	/// P lines are ignored while Q lines arrive (the driver stream sends P only for a device without Q for 0.5 s)
	pub last_q_wall: f64,
	pub state: String,
	pub light_age: f64,
	pub pos_sd: f64,
	pub rot_sd: f64,
	pub lh_mask: u32,
	/// the light's residual (mrad, S lines' eleventh field), -1 when unknown
	pub res_mrad: f64,
	/// the panel's clock at the last state change
	pub state_since: f64,
	pub head2imu: Pose,
	pub charge: i32,
	pub charging: bool,

	pub sensor_pos: Vec<V3>,
	pub sensor_nrm: Vec<V3>,
	sensors_asked: f64,
	/// hits per sensor, summed over the stations, in the last H period (sized to the sensors)
	pub sensor_hits: Vec<i32>,
	/// the panel's clock at the last H line
	pub hits_wall: f64,
	/// by station index
	pub light: BTreeMap<i32, StationLight>,
	/// the last H periods, oldest first (at most LIGHT_PERIODS): each station's hits per sensor, for the device page's
	/// sensors
	pub periods: VecDeque<LightPeriod>,

	updates: u64,
	/// poses a second
	pub rate_hz: f64,
	rate_mark_updates: u64,
	rate_mark_wall: f64,

	pub trail: VecDeque<TrailPoint>,
	pub history: History,
	/// in the runtime's last device list
	pub present: bool,
	/// listed and posing (a pose in the last ACTIVE_S): only active devices are shown. A paired device that is
	/// switched off is listed by its dongle but never poses.
	pub active: bool,
	/// has been active this session
	pub was_active: bool,
	quiet_logged: bool,

	pub diag: Diagnosis,
	/// the panel's clock at every light loss (tracking to dark or coasting), the last 10 minutes
	pub drops: VecDeque<f64>,
}

impl Device {
	/// Seconds without a pose after which a device is no longer active
	pub const ACTIVE_S: f64 = 3.0;
	/// H periods kept (the sensors' verdicts take three: a sensor dark in the last two, and the poses at both ends)
	pub const LIGHT_PERIODS: usize = 4;

	/// A sensor in the device's head frame (the frame of its pose): the runtime gives them in its IMU's (the config's
	/// modelPoints moved into it), and its pose is the head's, the IMU's pose times head2imu. Where it is and which way
	/// it faces
	pub fn sensor_in_head(&self, i: usize) -> Option<(V3, V3)> {
		let s = *self.sensor_pos.get(i)?;
		let n = self.sensor_nrm.get(i).copied().unwrap_or_default();
		let iq = qconj(self.head2imu.q);
		Some((qrot(iq, s - self.head2imu.p), qrot(iq, n)))
	}

	pub fn new(codename: &str, now: f64) -> Device {
		Device {
			codename: codename.into(),
			serial: "-".into(),
			subtype: "generic".into(),
			dongle: String::new(),
			sensor_ct: 0,
			have_pose: false,
			pos: V3::default(),
			vel: V3::default(),
			angvel: V3::default(),
			rot: Q4::default(),
			run_time: 0.0,
			last_pose_wall: -1e9,
			last_q_wall: -1e9,
			state: "none".into(),
			light_age: -1.0,
			pos_sd: -1.0,
			rot_sd: -1.0,
			lh_mask: 0,
			res_mrad: -1.0,
			state_since: now,
			head2imu: Pose::default(),
			charge: -1,
			charging: false,
			sensor_pos: Vec::new(),
			sensor_nrm: Vec::new(),
			sensors_asked: -1e9,
			sensor_hits: Vec::new(),
			hits_wall: -1e9,
			light: BTreeMap::new(),
			periods: VecDeque::new(),
			updates: 0,
			rate_hz: 0.0,
			rate_mark_updates: 0,
			rate_mark_wall: now,
			trail: VecDeque::new(),
			history: History::default(),
			present: true,
			active: false,
			was_active: false,
			quiet_logged: false,
			diag: Diagnosis::default(),
			drops: VecDeque::new(),
		}
	}

	/// Light losses in the last `window` seconds
	pub fn drops_last(&self, window: f64, now: f64) -> usize {
		self.drops.iter().filter(|&&t| now - t <= window).count()
	}

	/// Sensors that got light in the last H period (none once it is 2.5 s old)
	pub fn lit_sensors(&self, now: f64) -> i32 {
		if now - self.hits_wall >= 2.5 {
			return 0;
		}
		self.sensor_hits.iter().filter(|&&h| h > 0).count() as i32
	}

	/// The battery, from DEV lines, else from the diagnosis; -1 when unknown
	pub fn charge(&self) -> i32 {
		if self.charge >= 0 {
			self.charge
		} else if self.diag.charge > 0 {
			self.diag.charge
		} else {
			-1
		}
	}

	/// A copy of everything but the history and the trail, which are most of a device's memory (up to 120 s of
	/// samples, 1.5 MB) and which the boxes that draw from a copy do not read: a frame's copy, for a box that changes
	/// the panel's state while it draws the device (a whole one a frame, for every view, was a megabyte copied each)
	pub fn copy_light(&mut self) -> Device {
		let history = std::mem::take(&mut self.history);
		let trail = std::mem::take(&mut self.trail);
		let copy = self.clone();
		self.history = history;
		self.trail = trail;
		copy
	}

	/// A sample for the history at the line's run time
	fn sample(&mut self) {
		let s = Sample {
			t: self.run_time,
			pos: self.pos,
			rot: self.rot,
			speed: len(self.vel),
			sd_mm: if self.pos_sd >= 0.0 { self.pos_sd * 1000.0 } else { -1.0 },
			lh_mask: self.lh_mask,
			res_mrad: self.res_mrad,
			light: Light::of(&self.state),
			rate: self.rate_hz,
		};
		self.history.push(s);
	}
}

#[derive(Clone, Debug, Default)]
pub struct Station {
	pub idx: i32,
	pub id: u32,
	/// the channel less one (LH2), -1 unknown
	pub mode: i32,
	pub position_set: bool,
	pub locked: bool,
	pub have_pose: bool,
	/// station to world
	pub pose: Pose,
	/// any device got its light in the last period
	pub seen: bool,
	seen_logged: bool,
	pub devices_lit: i32,
	/// hits from it in the last period, all devices
	pub hits: i32,
}

impl Station {
	fn new(idx: i32) -> Station {
		Station { idx, mode: -1, ..Default::default() }
	}

	/// "LHB-D0875DAE"
	pub fn name(&self) -> String {
		station_name(self.id)
	}

	/// "ch 8", or the index when the channel is not known
	pub fn channel_label(&self) -> String {
		if self.mode >= 0 {
			format!("ch {}", self.mode + 1)
		} else {
			format!("bsd {}", self.idx)
		}
	}
}

pub fn station_name(id: u32) -> String {
	format!("LHB-{id:08X}")
}

/// A floor square's coverage: how often a device was over it, with how many stations, or dark
#[derive(Clone, Copy, Debug, Default)]
pub struct Voxel {
	pub n: u32,
	/// stations tracking it (at most 3) summed over the samples
	pub stations_sum: u32,
	pub dark: u32,
}

#[derive(Clone, Debug, Default)]
pub struct CmdReply {
	pub id: String,
	/// without the id, the final OK or ERR line last
	pub lines: Vec<String>,
}

/// What a command's reply is for
#[derive(Clone, Debug, PartialEq)]
pub enum Want {
	/// nobody waits: SENS lines are taken in, an ERR is logged
	Nothing,
	Hello,
	Devices,
	/// `calibrate status`, `calibrate`, `calibrate apply`
	CalibrateState,
	Calibrate,
	CalibrateApply,
}

/// The room's calibration as `calibrate status` says it stands
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Calib {
	/// the runtime keeps the calibration up (a live one): None until it has answered, Some(false) when it cannot (a
	/// replay, an older runtime)
	pub available: Option<bool>,
	pub running: Option<CalibRun>,
	/// the room is not calibrated yet and its first calibration has not started: the seconds until it does
	pub new_room: Option<f64>,
	/// LH1 base stations were seen (sally tracks with LH2 ones only)
	pub lh1: bool,
	/// one ready for the next start: its recording and why
	pub ready: Option<(String, String)>,
	/// one waiting for the next session's word: its recording
	pub pending: Option<String>,
	/// log.txt's last line: time, what, recording, text
	pub last: Option<String>,
	/// this session's own calibration failed: why, as it said (a runtime before 0.31.0 does not say)
	pub failed: Option<String>,
}

/// A calibration running: what of, its recording, where it is (replaying, solving), how far, the still moments so far
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CalibRun {
	pub kind: String,
	pub recording: String,
	pub state: String,
	pub progress: f64,
	pub moments: u64,
}

impl Calib {
	/// `OK calibrate idle`, or its parts joined by " | ": `running <kind> <recording> <state> <progress> <moments>`,
	/// `newroom <seconds>`, `lh1`, `ready <recording> <why>`, `pending <recording>`, `failed <why>`,
	/// `last <time> <what> <recording> <text>`
	pub fn parse(reply: &str) -> Calib {
		let Some(body) = reply.strip_prefix("OK calibrate ") else {
			return Calib { available: Some(false), ..Calib::default() };
		};
		let mut c = Calib { available: Some(true), ..Calib::default() };
		for part in body.split(" | ") {
			let (head, tail) = part.split_once(' ').unwrap_or((part, ""));
			match head {
				"running" => {
					let f: Vec<&str> = tail.split(' ').collect();
					c.running = Some(CalibRun {
						kind: f.first().unwrap_or(&"").to_string(),
						recording: f.get(1).unwrap_or(&"").to_string(),
						state: f.get(2).unwrap_or(&"").to_string(),
						progress: f.get(3).and_then(|v| v.parse().ok()).unwrap_or(0.0),
						moments: f.get(4).and_then(|v| v.parse().ok()).unwrap_or(0),
					});
				}
				"newroom" => c.new_room = tail.trim().parse().ok(),
				"lh1" => c.lh1 = true,
				"ready" => {
					let (rec, why) = tail.split_once(' ').unwrap_or((tail, ""));
					c.ready = Some((rec.to_string(), why.to_string()));
				}
				"pending" => c.pending = Some(tail.to_string()),
				"last" => c.last = Some(tail.to_string()),
				"failed" => c.failed = Some(tail.trim().to_string()),
				_ => {}
			}
		}
		c
	}
}

struct Pending {
	want: Want,
	reply: CmdReply,
	sent: f64,
}

pub struct Runtime {
	pub host: String,
	pub port: u16,
	pub protocol: i32,
	pub run_time: f64,
	last_rx_wall: f64,
	pub map_locked: bool,
	/// the room's calibration as the runtime keeps it up (`calibrate status`)
	pub calib: Calib,
	pub datagrams: u64,
	/// datagrams a second
	pub rx_rate: f64,

	/// by codename: every device the runtime mentioned (`active` says which to show)
	pub devices: BTreeMap<String, Device>,
	/// by station index
	pub stations: BTreeMap<i32, Station>,

	pub events: VecDeque<Event>,

	/// the coverage heat map: floor squares (x, y) of voxel_size, the floor grid's squares
	pub voxel_size: f64,
	pub voxels: HashMap<(i32, i32), Voxel>,

	pub trail_seconds: f64,
	/// the panel's clock, set by poll
	pub now: f64,

	sock: Option<UdpSocket>,
	last_sub: f64,
	last_devices: f64,
	last_hello: f64,
	last_rate: f64,
	rate_mark: u64,
	next_id: u64,
	pending: BTreeMap<String, Pending>,
	was_connected: bool,
	/// the receive buffer (a datagram's most), kept from poll to poll
	rx: Vec<u8>,
}

/// The most devices (codenames) the panel keeps, and floor squares in the heat map: a line can name any codename, and
/// a position run away puts its device on a square of its own with every S line (the runtime has 32 devices at most,
/// a room a few hundred squares)
const MAX_DEVICES: usize = 64;
const MAX_VOXELS: usize = 1 << 16;
/// The causes a diagnosis window collects (its C lines, until the next D line; the runtime has five kinds of cause)
const MAX_CAUSES: usize = 16;

/// strtoul(s, nullptr, base) on a token: leading digits of that base (an optional 0x for 16), 0 for none
fn strtoul(s: &str, base: u32) -> u64 {
	let mut t = s.trim_start();
	let neg = t.starts_with('-');
	t = t.strip_prefix(['+', '-']).unwrap_or(t);
	if base == 16 {
		t = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")).unwrap_or(t);
	}
	let mut v: u64 = 0;
	for c in t.chars() {
		match c.to_digit(base) {
			Some(d) => v = v.wrapping_mul(base as u64).wrapping_add(d as u64),
			None => break,
		}
	}
	if neg {
		v.wrapping_neg()
	} else {
		v
	}
}

fn atof(s: &str) -> f64 {
	c_atof(s.as_bytes())
}

fn atoi(s: &str) -> i32 {
	c_atoi(s.as_bytes())
}

/// The wall clock (Unix seconds)
pub fn wall_clock() -> i64 {
	std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

impl Default for Runtime {
	fn default() -> Runtime {
		Runtime::new()
	}
}

impl Runtime {
	pub fn new() -> Runtime {
		Runtime {
			host: "127.0.0.1".into(),
			port: 9100,
			protocol: 0,
			run_time: 0.0,
			last_rx_wall: -1e9,
			map_locked: false,
			calib: Calib::default(),
			datagrams: 0,
			rx_rate: 0.0,
			devices: BTreeMap::new(),
			stations: BTreeMap::new(),
			events: VecDeque::new(),
			voxel_size: 0.25,
			voxels: HashMap::new(),
			trail_seconds: 6.0,
			now: 0.0,
			sock: None,
			last_sub: -1e9,
			last_devices: -1e9,
			last_hello: -1e9,
			last_rate: 0.0,
			rate_mark: 0,
			next_id: 1,
			pending: BTreeMap::new(),
			was_connected: false,
			rx: Vec::new(),
		}
	}

	pub fn set_target(&mut self, host: &str, port: u16) {
		self.host = host.into();
		self.port = port;
		self.last_sub = -1e9;
	}

	/// The runtime answered in the last 2 s
	pub fn connected(&self, now: f64) -> bool {
		now - self.last_rx_wall < 2.0
	}

	/// The active devices (listed and posing)
	pub fn active(&self) -> impl Iterator<Item = &Device> {
		self.devices.values().filter(|d| d.active)
	}

	pub fn clear_heat(&mut self) {
		self.voxels.clear();
	}

	pub fn clear_trails(&mut self) {
		for d in self.devices.values_mut() {
			d.trail.clear();
		}
	}

	pub fn log(&mut self, level: EvLevel, who: &str, text: &str, run_time: f64, matters: bool) {
		self.events.push_back(Event {
			clock: wall_clock(),
			run_time,
			level,
			who: who.into(),
			text: text.into(),
			matters,
		});
		while self.events.len() > 3000 {
			self.events.pop_front();
		}
	}

	fn open_socket(&mut self) -> bool {
		if self.sock.is_some() {
			return true;
		}
		use socket2::{Domain, Socket, Type};
		let Ok(s) = Socket::new(Domain::IPV4, Type::DGRAM, None) else { return false };
		let _ = s.set_nonblocking(true);
		let _ = s.set_recv_buffer_size(4 << 20);
		let loopback = self.host == "127.0.0.1" || self.host == "localhost";
		let ip = if loopback { Ipv4Addr::LOCALHOST } else { Ipv4Addr::UNSPECIFIED };
		if s.bind(&SocketAddr::V4(SocketAddrV4::new(ip, 0)).into()).is_err() {
			return false;
		}
		self.sock = Some(s.into());
		true
	}

	fn send_raw(&mut self, line: &str) {
		if !self.open_socket() {
			return;
		}
		let host = if self.host == "localhost" { "127.0.0.1" } else { self.host.as_str() };
		let Ok(ip) = host.parse::<Ipv4Addr>() else { return };
		if let Some(s) = &self.sock {
			let _ = s.send_to(format!("{line}\n").as_bytes(), SocketAddrV4::new(ip, self.port));
		}
	}

	/// Sends a command; its reply goes where `want` says
	pub fn command(&mut self, cmd: &str, want: Want) {
		let id = format!("c{}", self.next_id);
		self.next_id += 1;
		self.pending.insert(
			id.clone(),
			Pending { want, reply: CmdReply { id: id.clone(), lines: Vec::new() }, sent: self.now },
		);
		self.send_raw(&format!("#{id} {cmd}"));
	}

	/// Reads everything waiting on the socket and keeps the subscription alive: once a frame
	pub fn poll(&mut self, now: f64) {
		self.now = now;
		if !self.open_socket() {
			return;
		}
		if now - self.last_sub > 1.0 {
			// sub again refreshes the subscription; a restarted runtime takes it as a new subscriber (the map resent).
			// Protocol 3 has the lean set (Q, L, S and the diagnosis); an older runtime rejects "state" and gets all.
			let sub = if self.protocol >= 3 { "#sub sub driver state diag" } else { "#sub sub all" };
			self.send_raw(sub);
			self.last_sub = now;
		}
		if now - self.last_hello > 2.0 {
			self.command("hello", Want::Hello);
			self.command("calibrate status", Want::CalibrateState);
			self.last_hello = now;
		}
		if now - self.last_devices > 2.0 && self.connected(now) {
			self.command("devices", Want::Devices);
			self.last_devices = now;
		}
		if self.protocol >= 3 && self.connected(now) {
			let ask: Vec<String> = self
				.devices
				.values_mut()
				.filter(|d| d.sensor_ct > 0 && d.sensor_pos.is_empty() && now - d.sensors_asked > 3.0)
				.map(|d| {
					d.sensors_asked = now;
					d.codename.clone()
				})
				.collect();
			for c in ask {
				self.command(&format!("sensors {c}"), Want::Nothing);
			}
		}

		// the buffer is the struct's, out of it while lines are handled (a 64 KB one made for every poll)
		let mut buf = std::mem::take(&mut self.rx);
		buf.resize(65536, 0);
		for _ in 0..50000 {
			let Some(sock) = &self.sock else { break };
			match sock.recv_from(&mut buf) {
				Ok((n, _)) => {
					self.datagrams += 1;
					self.last_rx_wall = now;
					let text = String::from_utf8_lossy(&buf[..n]);
					// one line a datagram, but several are taken too
					for line in text.split('\n') {
						if !line.is_empty() {
							self.handle_line(line);
						}
					}
				}
				Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
				// a stray ICMP error (Windows reports a closed port on the next receive): keep reading
				Err(_) => continue,
			}
		}
		self.rx = buf;

		let c = self.connected(now);
		if c != self.was_connected {
			if c {
				let t = format!("connected to {}:{}", self.host, self.port);
				self.log(EvLevel::Good, "runtime", &t, -1.0, true);
			} else if self.last_rx_wall > 0.0 {
				self.log(EvLevel::Bad, "runtime", "runtime stopped answering", -1.0, true);
			}
			self.was_connected = c;
			if !c {
				self.protocol = 0;
			}
		}

		// active devices: listed and posing. Going quiet after having posed is an event (switched off, flat, out of
		// range), unless the whole runtime went quiet (a restart is one event, not one a device)
		let mut evs = Vec::new();
		for d in self.devices.values_mut() {
			let act = d.present && d.have_pose && now - d.last_pose_wall < Device::ACTIVE_S;
			if act != d.active {
				if !act && c {
					evs.push((EvLevel::Warn, d.serial.clone(), "no poses for 3 s"));
					d.quiet_logged = true;
				} else if act && d.quiet_logged {
					evs.push((EvLevel::Good, d.serial.clone(), "poses again"));
					d.quiet_logged = false;
				}
				d.active = act;
				if act {
					d.was_active = true;
				}
			}
		}
		for (l, who, t) in evs {
			self.log(l, &who, t, -1.0, true);
		}

		// unanswered commands go after 5 s
		self.pending.retain(|_, p| now - p.sent <= 5.0);

		if now - self.last_rate >= 1.0 {
			let dt = now - self.last_rate;
			if self.last_rate > 0.0 {
				self.rx_rate = (self.datagrams - self.rate_mark) as f64 / dt;
			}
			self.rate_mark = self.datagrams;
			self.last_rate = now;
			for d in self.devices.values_mut() {
				let ddt = now - d.rate_mark_wall;
				if ddt > 0.5 {
					d.rate_hz = (d.updates - d.rate_mark_updates) as f64 / ddt;
					d.rate_mark_updates = d.updates;
					d.rate_mark_wall = now;
				}
			}
		}

		// the stations' totals from every device's latest H period
		for s in self.stations.values_mut() {
			s.hits = 0;
			s.devices_lit = 0;
		}
		for d in self.devices.values() {
			if now - d.hits_wall > 2.5 {
				continue;
			}
			for (lh, l) in &d.light {
				if l.hits <= 0 {
					continue;
				}
				if let Some(s) = self.stations.get_mut(lh) {
					s.hits = s.hits.saturating_add(l.hits);
					s.devices_lit += 1;
				}
			}
		}
		for s in self.stations.values_mut() {
			s.seen = s.devices_lit > 0;
		}
		let keep = self.trail_seconds;
		for d in self.devices.values_mut() {
			while d.trail.front().is_some_and(|p| now - p.t > keep) {
				d.trail.pop_front();
			}
		}
	}

	fn handle_line(&mut self, line: &str) {
		if let Some(rest) = line.strip_prefix('#') {
			if let Some((id, rest)) = rest.split_once(' ') {
				self.handle_reply(id, rest);
			}
			return;
		}
		let f: Vec<&str> = line.split(' ').filter(|t| !t.is_empty()).collect();
		if f.is_empty() || f[0].len() != 1 {
			return;
		}
		match f[0] {
			"Q" => self.on_q(&f),
			"P" => self.on_p(&f),
			"S" => self.on_s(&f),
			"H" => self.on_h(&f),
			"M" => self.on_m(&f),
			"L" => self.on_l(&f),
			"F" => self.on_f(&f),
			"D" => self.on_d(&f),
			"C" => self.on_c(&f),
			_ => {}
		}
	}

	fn handle_reply(&mut self, id: &str, rest: &str) {
		if id == "sub" {
			return;
		}
		let Some(p) = self.pending.get_mut(id) else { return };
		p.reply.lines.push(rest.into());
		if !(rest.starts_with("OK ") || rest == "OK" || rest.starts_with("ERR ")) {
			return;
		}
		let Some(p) = self.pending.remove(id) else { return };
		let r = p.reply;
		let last = r.lines.last().cloned().unwrap_or_default();
		match p.want {
			Want::Hello => self.on_hello(&last),
			Want::CalibrateState => self.on_calibrate_state(&last),
			Want::Calibrate | Want::CalibrateApply => {
				let (level, text) = match last.strip_prefix("OK calibrate ") {
					Some(t) => (EvLevel::Good, t.to_string()),
					None => (EvLevel::Bad, last.strip_prefix("ERR calibrate ").unwrap_or(&last).to_string()),
				};
				self.log(level, "calibration", &text, -1.0, true);
				self.last_hello = 0.0;
			}
			Want::Devices => self.on_devices(&r),
			Want::Nothing => {
				for ln in &r.lines {
					if ln.starts_with("SENS ") {
						let f: Vec<&str> = ln.split(' ').filter(|t| !t.is_empty()).collect();
						self.on_sens(&f);
					} else if let Some(e) = ln.strip_prefix("ERR ") {
						self.log(EvLevel::Bad, "runtime", e, -1.0, false);
					}
				}
			}
		}
	}

	/// OK hello sally-runtime <protocol> <run_time> ...
	/// `calibrate status`'s reply: the state kept, and an event when a calibration starts, finishes or is ready
	fn on_calibrate_state(&mut self, last: &str) {
		let new = Calib::parse(last);
		let old = std::mem::replace(&mut self.calib, new.clone());
		if new.available != Some(true) {
			return;
		}
		if let (Some(r), None) = (&new.running, &old.running) {
			let t = format!("calibrating {} ({})", r.recording, r.kind);
			self.log(EvLevel::Info, "calibration", &t, -1.0, true);
		}
		if let Some((rec, why)) = new.ready.as_ref().filter(|_| old.ready.is_none() && old.available.is_some()) {
			let t = format!("a new calibration is ready (of {rec}: {why}): it is used from the next start");
			self.log(EvLevel::Good, "calibration", &t, -1.0, true);
		}
		if new.last != old.last && old.available.is_some() {
			if let Some(l) = &new.last {
				self.log(EvLevel::Info, "calibration", l, -1.0, false);
			}
		}
	}

	fn on_hello(&mut self, last: &str) {
		let f: Vec<&str> = last.split(' ').filter(|t| !t.is_empty()).collect();
		if f.len() < 5 || f[0] != "OK" || f[1] != "hello" || f[2] != "sally-runtime" {
			return;
		}
		let (Ok(proto), Ok(rt)) = (f[3].parse::<i32>(), f[4].parse::<f64>()) else { return };
		if proto != self.protocol {
			let (level, text) = if proto >= 3 {
				(EvLevel::Info, format!("runtime protocol {proto}"))
			} else {
				(
					EvLevel::Warn,
					format!("runtime speaks protocol {proto}: update it for sensors, battery and the state stream"),
				)
			};
			self.log(level, "runtime", &text, rt, false);
			// subscribe again with the streams this protocol has
			self.last_sub = -1e9;
		}
		self.protocol = proto;
	}

	fn on_devices(&mut self, r: &CmdReply) {
		let mut seen = Vec::new();
		for ln in &r.lines {
			if !ln.starts_with("DEV ") {
				continue;
			}
			let f: Vec<&str> = ln.split(' ').filter(|t| !t.is_empty()).collect();
			self.on_dev(&f);
			if f.len() > 1 {
				seen.push(f[1].to_string());
			}
		}
		let mut left = Vec::new();
		for (code, d) in self.devices.iter_mut() {
			let p = seen.contains(code);
			if d.present && !p {
				left.push(d.serial.clone());
			}
			d.present = p;
		}
		for s in left {
			self.log(EvLevel::Warn, &s, "left the runtime's device list", -1.0, true);
		}
	}

	/// The device of a codename, made when it is first named: None for a new one once there are MAX_DEVICES
	fn dev(&mut self, codename: &str) -> Option<&mut Device> {
		if !self.devices.contains_key(codename) {
			if self.devices.len() >= MAX_DEVICES {
				return None;
			}
			self.devices.insert(codename.to_string(), Device::new(codename, self.now));
		}
		self.devices.get_mut(codename)
	}

	/// A station by index; the index comes from the network and is a bit index (1 << idx) and a key: kept to 0..31
	fn sta(&mut self, idx: i32) -> &mut Station {
		let idx = idx.clamp(0, 31);
		self.stations.entry(idx).or_insert_with(|| Station::new(idx))
	}

	fn add_trail(d: &mut Device, now: f64) {
		if d.trail.back().is_some_and(|p| now - p.t < 1.0 / 90.0) {
			return;
		}
		let kind = match d.state.as_str() {
			"tracking" => 0,
			"coasting" => 2,
			_ => 1,
		};
		d.trail.push_back(TrailPoint { p: d.pos, t: now, kind });
	}

	fn on_q(&mut self, f: &[&str]) {
		if f.len() < 18 {
			return;
		}
		let now = self.now;
		let Some(d) = self.dev(f[1]) else { return };
		if (!f[2].starts_with('-') || d.serial == "-") && d.serial != f[2] {
			d.serial = f[2].into();
		}
		d.run_time = atof(f[4]);
		d.pos = V3::new(atof(f[5]), atof(f[6]), atof(f[7]));
		d.rot = qnorm(Q4::new(atof(f[8]), atof(f[9]), atof(f[10]), atof(f[11])));
		d.vel = V3::new(atof(f[12]), atof(f[13]), atof(f[14]));
		d.angvel = V3::new(atof(f[15]), atof(f[16]), atof(f[17]));
		d.have_pose = true;
		d.last_pose_wall = now;
		d.last_q_wall = now;
		d.updates += 1;
		Runtime::add_trail(d, now);
		d.sample();
		let rt = d.run_time;
		self.run_time = rt;
	}

	fn on_p(&mut self, f: &[&str]) {
		if f.len() < 13 {
			return;
		}
		let now = self.now;
		let Some(d) = self.dev(f[1]) else { return };
		if (!f[2].starts_with('-') || d.serial == "-") && d.serial != f[2] {
			d.serial = f[2].into();
		}
		if now - d.last_q_wall < 0.5 {
			return;
		}
		d.run_time = atof(f[4]);
		d.pos = V3::new(atof(f[6]), atof(f[7]), atof(f[8]));
		d.rot = qnorm(Q4::new(atof(f[9]), atof(f[10]), atof(f[11]), atof(f[12])));
		d.have_pose = true;
		d.last_pose_wall = now;
		d.updates += 1;
		Runtime::add_trail(d, now);
		d.sample();
		let rt = d.run_time;
		self.run_time = rt;
	}

	fn on_s(&mut self, f: &[&str]) {
		if f.len() < 10 {
			return;
		}
		let now = self.now;
		let vs = self.voxel_size;
		let Some(d) = self.dev(f[1]) else { return };
		if !f[2].starts_with('-') && d.serial != f[2] {
			d.serial = f[2].into();
		}
		let rt = atof(f[4]);
		let st = f[5];
		d.light_age = atof(f[6]);
		d.pos_sd = atof(f[7]);
		d.rot_sd = atof(f[8]);
		d.lh_mask = strtoul(f[9], 16) as u32;
		d.res_mrad = f.get(10).map(|v| atof(v)).unwrap_or(-1.0);
		let mut ev = None;
		if st != d.state {
			let was = now - d.state_since;
			if d.state == "tracking" && (st == "dark" || st == "coasting") {
				d.drops.push_back(now);
				while d.drops.front().is_some_and(|&t| now - t > 600.0) {
					d.drops.pop_front();
				}
			}
			if st == "coasting" {
				ev = Some((EvLevel::Bad, format!("coasting on the IMU (no light for {:.2} s)", d.light_age)));
			} else if st == "tracking" && (d.state == "coasting" || (d.state == "dark" && was > 0.25)) {
				let how = if d.state == "coasting" { "coasting" } else { "without light" };
				ev = Some((EvLevel::Good, format!("tracking again after {was:.2} s {how}")));
			} else if st == "tracking" && d.state == "none" {
				ev = Some((EvLevel::Good, "tracking".into()));
			}
			d.state = st.into();
			d.state_since = now;
		}
		// (a position that is not a number is on no square: as an integer it was the one at the origin)
		let voxel = if d.have_pose && now - d.last_pose_wall < 0.5 && d.pos.x.is_finite() && d.pos.y.is_finite() {
			let key = ((d.pos.x / vs).floor() as i32, (d.pos.y / vs).floor() as i32);
			Some((key, st == "tracking", d.lh_mask.count_ones().min(3)))
		} else {
			None
		};
		let serial = d.serial.clone();
		if let Some((key, tracking, n)) = voxel {
			// every height falls into the floor square under it; a new square while there is room
			if self.voxels.len() < MAX_VOXELS || self.voxels.contains_key(&key) {
				let v = self.voxels.entry(key).or_default();
				v.n += 1;
				if tracking {
					v.stations_sum += n;
				} else {
					v.dark += 1;
				}
			}
		}
		if let Some((l, t)) = ev {
			self.log(l, &serial, &t, rt, true);
		}
	}

	/// H codename lh period_s n_sensors count_0 ... count_{n-1}
	fn on_h(&mut self, f: &[&str]) {
		if f.len() < 5 {
			return;
		}
		let now = self.now;
		let Some(d) = self.dev(f[1]) else { return };
		// a station index, a key: kept to 0..31 as `sta` keeps it
		let lh = atoi(f[2]).clamp(0, 31);
		// network text sizes an allocation: kept to 64 (devices have at most 32 sensors)
		let ns = atoi(f[4]).clamp(0, 64) as usize;
		if now - d.hits_wall > 0.3 {
			// a new period: the last one's counts go
			for l in d.light.values_mut() {
				*l = StationLight::default();
			}
			d.sensor_hits = vec![0; ns.max(d.sensor_ct.max(0) as usize)];
			// kept per station a few periods, with where the device was as they came
			let tracking = d.have_pose && now - d.last_pose_wall < 0.5 && d.state == "tracking";
			d.periods.push_back(LightPeriod {
				wall: now,
				period: atof(f[3]),
				pose: Pose { p: d.pos, q: d.rot },
				tracking,
				hits: BTreeMap::new(),
			});
			while d.periods.len() > Device::LIGHT_PERIODS {
				d.periods.pop_front();
			}
		}
		d.hits_wall = now;
		if d.sensor_hits.len() < ns {
			d.sensor_hits.resize(ns, 0);
		}
		let l = d.light.entry(lh).or_default();
		*l = StationLight::default();
		let mut counts = vec![0; ns];
		for (i, c_out) in counts.iter_mut().enumerate() {
			let Some(t) = f.get(5 + i) else { break };
			// (counts from the network: an absurd one would overflow the sums; a negative one is none)
			let c = atoi(t).max(0);
			*c_out = c;
			l.hits = l.hits.saturating_add(c);
			if c > 0 {
				l.sensors_lit += 1;
			}
			d.sensor_hits[i] = d.sensor_hits[i].saturating_add(c);
		}
		if let Some(p) = d.periods.back_mut() {
			p.hits.insert(lh, counts);
		}
	}

	/// M bsd_idx station_id mode position_set locked x y z qw qx qy qz
	fn on_m(&mut self, f: &[&str]) {
		if f.len() < 13 {
			return;
		}
		let locked = atoi(f[5]) != 0;
		if locked != self.map_locked {
			let (l, t) = if locked {
				(EvLevel::Info, "map locked")
			} else {
				(EvLevel::Warn, "map unlocked (stations may be re-solved)")
			};
			self.log(l, "map", t, -1.0, false);
			self.map_locked = locked;
		}
		let s = self.sta(atoi(f[1]));
		s.id = strtoul(f[2], 10) as u32;
		// the runtime's is a byte; the channel shown is one more, which i32::MAX would overflow
		s.mode = atoi(f[3]).clamp(-1, 255);
		s.position_set = atoi(f[4]) != 0;
		s.locked = locked;
		if s.position_set && !s.have_pose {
			s.pose.p = V3::new(atof(f[6]), atof(f[7]), atof(f[8]));
			s.pose.q = qnorm(Q4::new(atof(f[9]), atof(f[10]), atof(f[11]), atof(f[12])));
			s.have_pose = true;
		}
	}

	/// L bsd_idx station_id wall_epoch x y z qw qx qy qz
	fn on_l(&mut self, f: &[&str]) {
		if f.len() < 11 {
			return;
		}
		let s = self.sta(atoi(f[1]));
		s.id = strtoul(f[2], 10) as u32;
		let np = Pose {
			p: V3::new(atof(f[4]), atof(f[5]), atof(f[6])),
			q: qnorm(Q4::new(atof(f[7]), atof(f[8]), atof(f[9]), atof(f[10]))),
		};
		let ev = if s.have_pose {
			let moved = len(np.p - s.pose.p);
			(moved > 0.02).then(|| (EvLevel::Warn, format!("moved {:.0} mm in the map", moved * 1000.0)))
		} else {
			Some((EvLevel::Info, "placed in the map".to_string()))
		};
		s.pose = np;
		s.have_pose = true;
		s.position_set = true;
		let name = s.name();
		if let Some((l, t)) = ev {
			self.log(l, &name, &t, -1.0, false);
		}
	}

	/// F bsd_idx seen|lost run_time
	fn on_f(&mut self, f: &[&str]) {
		if f.len() < 4 {
			return;
		}
		let s = self.sta(atoi(f[1]));
		let seen = f[2] == "seen";
		let was = s.seen;
		s.seen = seen;
		if seen == was && s.seen_logged {
			return;
		}
		s.seen_logged = true;
		let who = if s.id != 0 { s.name() } else { format!("station {}", s.idx) };
		let (l, t) =
			if seen { (EvLevel::Good, "light seen") } else { (EvLevel::Bad, "light lost (no device sees it)") };
		self.log(l, &who, t, atof(f[3]), true);
	}

	/// D codename run_time headline severity bars loss_pct gap_ms stations sensors back_pct gated discards state
	/// light_age charge n_causes
	fn on_d(&mut self, f: &[&str]) {
		if f.len() < 17 {
			return;
		}
		let now = self.now;
		let Some(d) = self.dev(f[1]) else { return };
		let active = d.active;
		let serial = d.serial.clone();
		let g = &mut d.diag;
		let rt = atof(f[2]);
		let head = f[3];
		let sev = atoi(f[4]);
		// the C lines of the window before are complete now
		g.causes = std::mem::take(&mut g.causes_next);
		if atoi(f[16]) == 0 {
			g.causes.clear();
		}
		g.wall = now;
		g.bars = atoi(f[5]);
		g.loss_pct = atof(f[6]);
		g.gap_ms = atof(f[7]);
		g.stations = atoi(f[8]);
		g.sensors = atoi(f[9]);
		g.back_pct = atof(f[10]);
		g.gated = atoi(f[11]);
		g.discards = atoi(f[12]);
		g.charge = atoi(f[15]);
		if head != g.headline || sev != g.severity {
			g.last_change = now;
		}
		g.headline = head.into();
		g.severity = sev;
		// events: a verdict is logged once it has held 2 s, only the ones worth a line (hurting, or radio, station or
		// device at any severity); "fine again" after 5 s fine, and only if something was logged for it
		if head != g.cand_head || sev != g.cand_sev {
			g.cand_head = head.into();
			g.cand_sev = sev;
			g.cand_since = now;
		}
		let held = now - g.cand_since;
		let mut fine_again = false;
		if head != g.logged_head || sev != g.logged_sev {
			if head == "fine" {
				if held >= 5.0 {
					fine_again = !g.last_text.is_empty() && active;
					g.last_text.clear();
					g.logged_head = head.into();
					g.logged_sev = 0;
				}
			} else if held >= 2.0 {
				g.log_pending = sev >= 2 || head == "radio" || head == "station" || head == "device";
				g.logged_head = head.into();
				g.logged_sev = sev;
			}
		}
		if fine_again {
			self.log(EvLevel::Good, &serial, "fine again", rt, true);
		}
	}

	/// C codename run_time cause severity text...
	fn on_c(&mut self, f: &[&str]) {
		if f.len() < 5 {
			return;
		}
		let now = self.now;
		let Some(d) = self.dev(f[1]) else { return };
		let text = f[5..].join(" ");
		// (C lines without their D line would collect without end)
		if d.diag.causes_next.len() < MAX_CAUSES {
			d.diag.causes_next.push((f[3].into(), text.clone()));
		}
		// the worst cause's text is the headline's: logged once the verdict has held (on_d sets log_pending); a device
		// that never posed (paired but switched off) is not shown, so its verdicts are not events either
		let g = &mut d.diag;
		if g.log_pending && f[3] == g.headline && d.was_active {
			g.log_pending = false;
			if text != g.last_text || now - g.last_log > 120.0 {
				g.last_text = text.clone();
				g.last_log = now;
				let level = if g.severity >= 2 { EvLevel::Bad } else { EvLevel::Warn };
				let serial = d.serial.clone();
				self.log(level, &serial, &text, atof(f[2]), true);
			}
		}
	}

	/// DEV codename serial sensors state light_age subtype timebase h2i(x y z qw qx qy qz) [charge charging] [dongle]
	fn on_dev(&mut self, f: &[&str]) {
		if f.len() < 15 {
			return;
		}
		let fresh = !self.devices.contains_key(f[1]);
		let Some(d) = self.dev(f[1]) else { return };
		if !f[2].starts_with('-') {
			d.serial = f[2].into();
		}
		// it sizes every H period's sensor_hits (on_h): kept to 64 as an H line's own count is
		d.sensor_ct = atoi(f[3]).clamp(0, 64);
		d.subtype = f[6].into();
		d.head2imu.p = V3::new(atof(f[8]), atof(f[9]), atof(f[10]));
		d.head2imu.q = qnorm(Q4::new(atof(f[11]), atof(f[12]), atof(f[13]), atof(f[14])));
		if f.len() >= 18 && !f[17].starts_with('-') {
			d.dongle = f[17].into();
		}
		if f.len() >= 17 {
			d.charge = atoi(f[15]);
			d.charging = atoi(f[16]) != 0;
		}
		let new = fresh || !d.present;
		d.present = true;
		if new {
			let (serial, t) = (d.serial.clone(), format!("device {} ({})", d.codename, d.subtype));
			self.log(EvLevel::Info, &serial, &t, -1.0, false);
		}
	}

	/// SENS codename index x y z nx ny nz
	fn on_sens(&mut self, f: &[&str]) {
		if f.len() < 9 {
			return;
		}
		let Some(d) = self.devices.get_mut(f[1]) else { return };
		let i = atoi(f[2]);
		if !(0..=64).contains(&i) {
			return;
		}
		let i = i as usize;
		if d.sensor_pos.len() <= i {
			d.sensor_pos.resize(i + 1, V3::default());
			d.sensor_nrm.resize(i + 1, V3::default());
		}
		d.sensor_pos[i] = V3::new(atof(f[3]), atof(f[4]), atof(f[5]));
		d.sensor_nrm[i] = V3::new(atof(f[6]), atof(f[7]), atof(f[8]));
	}
}

#[cfg(test)]
mod tests;
