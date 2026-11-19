//! The runtime (SurviveContext and the default processes of survive_process.c, survive_process_gen2.c,
//! survive_disambiguator.c and the parts of survive.c they use): the tracked objects and the stations, the run
//! clock, the log and the recording, and the path each measurement takes from a device's parser to the recording: raw
//! IMU to calibrated IMU and the device clock; a station's sync to its rotor interval and OOTX bits; a sweep hit to a
//! rotor angle, a plane and an angle on it; LH1 pulses only to tell the system's generation.

use std::sync::Arc;

use sally_hid::replay::PlaybackTime;
use sally_hid::report::{ButtonEntry, LightcapElement};
use sally_hid::tap::Identified;
use sally_lighthouse::activations::{Gen2Light, ImuSample};
use sally_lighthouse::device::{ConfigOptions, DeviceConfig, Log as DeviceLog, SURVIVE_ERROR_GENERAL};
use sally_lighthouse::gen2::{self, angle_for_axis, plane_from_angle};
use sally_lighthouse::ootx::{LighthouseInfoV15, OotxDecoder, OotxEvent, OotxPacket};
use sally_lighthouse::steamvr;
use sally_lighthouse::NUM_GEN2_LIGHTHOUSES;
use sally_math::{
	apply_pose_to_point, dist3d, dot3d, norm3d, normalize3d, pose_slerp, quatiszero, quatrotatevector, sub3d, textmode,
	Pose, LINMATHPI,
};

use crate::config::{
	write_whole, Binding, Config, DetachError, SaveOutcome, NUM_GEN2_LIGHTHOUSES as CONFIG_LIGHTHOUSES, SC_GET,
	SC_OVERRIDE, SC_SETCONFIG,
};
use crate::log::{info_text, Log, LogFn, LogLevel};
use crate::object::{GenDetect, TrackedObject};
use crate::recording::Recording;
use crate::settings::{activation_params, in_list, Settings, StationMoves, ACTIVATION_OPTIONS};
use crate::station::{AxisAnglePose, OotxSlot, Station};
use crate::ObjectId;

/// SurviveError values the runtime raises (survive_types.h)
pub mod error {
	pub const OK: i32 = 0;
	pub const NO_TRACKABLE_OBJECTS: i32 = -2;
	pub const INVALID_CONFIG: i32 = -4;
}

/// sally: the devices `identify_object` remembers it found no config for (the latest ones)
const MAX_MISSES: usize = 64;

/// survive_run_time's clock: the wall clock until a playback installs its own (survive_install_run_time_fn), the
/// playback's after, and frozen at the last value once the playback is closed (static_time)
#[derive(Clone, Debug)]
enum Clock {
	/// timestamp_in_s: OGGetAbsoluteTime since survive.c's start (OGStartTimeS, which the log keeps), + 1 ms
	Wall,
	/// The time the playback has reached, which its own thread sets
	Installed(Arc<PlaybackTime>),
	/// static_time: the last run time read
	Frozen,
}

/// sally: the room's calibration kept up while the runtime runs live (`sally run`'s, docs/sally_design.md 4.2), as the
/// runtime protocol's `calibrate` reaches it. Without one (`Runtime::calibrator` None: a replay, the bench) the command
/// says so
pub trait Calibrator: Send {
	/// `calibrate [start | status | apply]`: the reply's text, and whether the runtime is to stop (`apply`: a restarted
	/// runtime takes the new calibration up as it starts)
	fn command(&mut self, rt: &mut Runtime, args: &[&[u8]]) -> (Vec<u8>, bool);
}

/// sally: what `sally calibrate` takes from a replay: every light hit the tracker would get (past the reflection
/// gate) and every calibrated IMU sample, as they come. The runtime calls it beside the tracker and changes nothing
/// else; without one (`Runtime::observer` None, as in `sally run` and `sally replay`) nothing is called
pub trait Observer: Send {
	/// a sweep hit: the device, the station (bsd index), the plane, the sensor, the device's long clock, the angle
	fn hit(&mut self, obj: ObjectId, lh: usize, plane: i8, sensor: i32, timecode: u64, angle: f64);
	/// a calibrated IMU sample: the device, its long clock, the accelerometer (g) and the gyro (rad/s), in the
	/// device's IMU frame
	fn imu(&mut self, obj: ObjectId, timecode: u64, accel: &[f64; 3], gyro: &[f64; 3]);
}

/// SurviveContext
pub struct Runtime {
	/// the config store: the options, the config file's groups, the stations' records
	pub config: Config,
	/// the options as the processes read them (the C mostly reads each once, into a static)
	pub settings: Settings,
	/// ctx->settings: the context's bound options (how a station's new pose is taken)
	pub station_moves: StationMoves,
	pub log: Log,
	/// the log hook installed in the default's place (ctx->logproc: None is survive_default_log_process)
	log_fn: Option<LogFn>,
	pub recording: Option<Recording>,
	clock: Clock,
	/// pctx->lastRunTime: the last run time read (what the frozen clock gives)
	last_run_time: f64,
	/// ctx->objs, in their order (creation, and survive_destroy_device's swap at shutdown)
	pub objects: Vec<TrackedObject>,
	/// the objects survive_add_object listed (one whose name was taken is used but never listed)
	listed: Vec<bool>,
	/// ctx->bsd: the stations by index, and the index each channel has (-1 none)
	pub bsd: Vec<Station>,
	pub bsd_map: [i8; NUM_GEN2_LIGHTHOUSES],
	pub active_lighthouses: i32,
	/// -1 unknown, 1 LH2 (sally tracks with LH2 alone; the C's 0 was LH1, 3 both)
	pub lh_version: i32,
	pub floor_offset: f64,
	pub current_error: i32,
	/// LH1 stations were seen (and said so)
	lh1_warned: bool,
	/// ctx->drivers: the drivers loaded, in load order (their hooks run in it)
	pub drivers: Vec<crate::driver::Loaded>,
	/// sally: the dongles' devices the runtime protocol's `pair` was told (dongle, device), for the main loop to give the
	/// live tap (`Context::apply_pairs`)
	pub pending_pairs: Vec<(String, String)>,
	/// sally: the devices no config was found for (`identify_object`), and when it was last looked for
	identify_misses: Vec<(String, f64)>,
	/// sally: `sally calibrate`'s collector (None otherwise)
	pub observer: Option<Box<dyn Observer>>,
	/// sally: the live runtime's calibration (`sally run`'s; None otherwise)
	pub calibrator: Option<Box<dyn Calibrator>>,
	/// sally: what the tracker learned of each device in earlier sessions and this one (device-learned-file; read at
	/// the first device's config: learned_tried)
	pub(crate) learned: Option<crate::learned::Store>,
	pub(crate) learned_tried: bool,
}

// The runtime is Send: threads take it in turn, under the context lock
const _: fn() = || {
	fn send<T: Send>() {}
	send::<Runtime>();
};

impl Runtime {
	/// survive_init's state, before the recording and the drivers: the context's own options bound
	/// (SurviveContext_attach_config) and the settings read from `config` (the command line, the environment and the
	/// config file in it), the stations not yet (load_lighthouses)
	pub fn new(mut config: Config, log: Log) -> Self {
		let station_moves = StationMoves::attach(&mut config);
		let settings = Settings::from_config(&mut config);
		let floor_offset = config.configf("floor-offset", SC_GET, 0.0);
		Runtime {
			config,
			settings,
			station_moves,
			log,
			log_fn: None,
			recording: None,
			clock: Clock::Wall,
			last_run_time: 0.0,
			objects: Vec::new(),
			listed: Vec::new(),
			bsd: (0..NUM_GEN2_LIGHTHOUSES).map(|_| Station::unused()).collect(),
			bsd_map: [-1; NUM_GEN2_LIGHTHOUSES],
			active_lighthouses: 0,
			lh_version: -1,
			floor_offset,
			current_error: error::OK,
			lh1_warned: false,
			drivers: Vec::new(),
			pending_pairs: Vec::new(),
			identify_misses: Vec::new(),
			observer: None,
			calibrator: None,
			learned: None,
			learned_tried: false,
		}
	}

	// ---------------------------------------------------------------- the clock, the log, the recording

	pub fn run_time(&mut self) -> f64 {
		self.last_run_time = self.peek_run_time();
		self.last_run_time
	}

	/// survive_run_time without noting the value (for a caller that only reads: the next call notes it)
	pub fn peek_run_time(&self) -> f64 {
		match &self.clock {
			// timestamp_in_s: OGGetAbsoluteTime() - OGStartTimeS() + 1e-3 (the C leaves the order of the two calls open,
			// which shows only in the first reading: here the start, then the clock)
			Clock::Wall => {
				let start = self.log.start_time();
				sally_os::absolute_time() - start + 1e-3
			}
			Clock::Installed(t) => t.get(),
			Clock::Frozen => self.last_run_time,
		}
	}

	/// survive_install_run_time_fn: the playback's time as the run clock (hidtap_run_time), or with None a clock that
	/// was installed frozen at the last run time read (static_time; the wall clock stays as it is)
	pub fn install_run_time_fn(&mut self, time: Option<Arc<PlaybackTime>>) {
		match time {
			Some(t) => self.clock = Clock::Installed(t),
			None if !matches!(self.clock, Clock::Wall) => self.clock = Clock::Frozen,
			None => {}
		}
	}

	/// A run clock of its own at `t` from now on (the tests' and the tools' replays, which set the time themselves)
	pub fn set_replay_time(&mut self, t: f64) {
		self.install_run_time_fn(Some(Arc::new(PlaybackTime::new(t))));
	}

	/// survive_install_log_fn: the log hook every SV_INFO, SV_WARN and SV_ERROR line goes to (None: the default's,
	/// survive_default_log_process)
	pub fn install_log_fn(&mut self, log_fn: Option<LogFn>) {
		self.log_fn = log_fn;
	}

	/// SV_INFO: the line (cut where SV_INFO's buffer cuts it) to the log hook: the one installed, or the default's
	/// (survive_default_info: the recording's INFO LOG line, then the log's)
	pub fn info(&mut self, msg: &str) {
		let msg = info_text(msg);
		if let Some(f) = self.log_fn.as_mut() {
			f(LogLevel::Info, msg);
			return;
		}
		if self.recording.is_some() {
			let t = self.run_time();
			if let Some(r) = self.recording.as_mut() {
				r.info(t, msg);
			}
		}
		self.log.info(msg);
	}

	/// SV_WARN: the line to the log hook: the one installed, or the default's (survive_default_warn)
	pub fn warn(&mut self, msg: &str) {
		if let Some(f) = self.log_fn.as_mut() {
			f(LogLevel::Warning, msg);
			return;
		}
		if self.recording.is_some() {
			let t = self.run_time();
			if let Some(r) = self.recording.as_mut() {
				r.info(t, msg);
			}
		}
		self.log.warn(msg);
	}

	/// SV_ERROR: the error raised (the report_error hook), and the text logged at the info level
	pub fn error(&mut self, code: i32, msg: &str) {
		self.current_error = code;
		self.info(msg);
	}

	/// The recording's line at the current run time
	pub(crate) fn record(&mut self, f: impl FnOnce(&mut Recording, f64)) {
		if self.recording.is_some() {
			let t = self.run_time();
			if let Some(r) = self.recording.as_mut() {
				f(r, t);
			}
		}
	}

	/// The recording's line about a device at the current run time, with the device's codename (`record`, without a
	/// copy of the name for each measurement when nothing is recorded)
	pub(crate) fn record_object(&mut self, obj: ObjectId, f: impl FnOnce(&mut Recording, f64, &str)) {
		if self.recording.is_some() {
			let t = self.run_time();
			if let Some(r) = self.recording.as_mut() {
				f(r, t, &self.objects[obj].codename);
			}
		}
	}

	/// survive_recording_write_to_output(ctx->recptr, "%s", line): a line already formatted, at the run time
	pub fn record_line(&mut self, line: &str) {
		self.record(|r, t| r.line(t, line));
	}

	// ---------------------------------------------------------------- objects and stations

	/// survive_create_device and survive_add_object: a device named `codename` from the driver `drivername`, with the
	/// serials its driver gives it between the two (the device's own and its dongle's); None when the name is
	/// blacklisted
	pub fn create_object(&mut self, drivername: &str, codename: &str, serial: &str, dongle: &str) -> Option<ObjectId> {
		if in_list(&self.settings.blacklist_devs, codename) {
			return None;
		}
		let mut so = TrackedObject::new(drivername, codename, self.settings.activations);
		// The C compares 8 bytes from the codename (char[4]), which runs into the start of sally_dongle: a new object's
		// is empty, a tap object's holds its dongle's serial, so only objects without a dongle can collide here
		let key = |name: &str, dongle: &str| -> [u8; 8] {
			let mut k = [0u8; 8];
			for (i, b) in name.bytes().take(4).enumerate() {
				k[i] = b;
			}
			for (i, b) in dongle.bytes().take(4).enumerate() {
				k[4 + i] = b;
			}
			k
		};
		let mut i = 0;
		while i < self.objects.len() {
			if key(&so.codename, &so.sally_dongle) == key(&self.objects[i].codename, &self.objects[i].sally_dongle) {
				// codename[2]++, and the loop goes on from 1 (i = 0, then i++)
				let mut b = so.codename.clone().into_bytes();
				if b.len() > 2 {
					b[2] = b[2].wrapping_add(1);
				}
				so.codename = String::from_utf8_lossy(&b).into_owned();
				i = 1;
				continue;
			}
			i += 1;
		}
		// survive_c2_create: the tracker, last in survive_create_device
		so.tracker = Some(self.new_tracker());
		// char serial_number[16], sally_dongle[32]
		so.serial_number = serial.chars().take(15).collect();
		so.sally_dongle = dongle.chars().take(31).collect();
		let id = self.objects.len();
		let name_taken = self.objects.iter().zip(&self.listed).any(|(o, &l)| l && o.codename == so.codename);
		self.objects.push(so);
		self.listed.push(false);
		if name_taken {
			let msg = format!("Object named {} already exists", self.objects[id].codename);
			self.error(error::INVALID_CONFIG, &msg);
		} else {
			let msg = format!(
				"Adding tracked object {} from {}",
				self.log.colorize(&self.objects[id].codename),
				self.log.colorize(&self.objects[id].drivername)
			);
			self.info(&msg);
			self.listed[id] = true;
		}
		Some(id)
	}

	/// Whether the device is one of ctx->objs (survive_add_object listed it)
	pub(crate) fn is_listed(&self, obj: ObjectId) -> bool {
		self.listed[obj]
	}

	/// survive_get_bsd_idx: the station index of a channel, a free index given it when it is new (-1 for no channel
	/// or no free index)
	pub fn get_bsd_idx(&mut self, channel: u8) -> i8 {
		if channel >= 16 {
			return -1;
		}
		let i = self.bsd_map[channel as usize];
		if i != -1 {
			return i;
		}
		for i in 0..NUM_GEN2_LIGHTHOUSES {
			if self.bsd[i].mode == 0xFF {
				self.bsd[i] = Station { mode: channel, ..Station::default() };
				if self.active_lighthouses < i as i32 + 1 {
					self.active_lighthouses = i as i32 + 1;
				}
				let msg = format!("Adding lighthouse ch {} (idx: {}, cnt: {})", channel, i, self.active_lighthouses);
				self.info(&msg);
				self.bsd_map[channel as usize] = i as i8;
				return i as i8;
			}
		}
		-1
	}

	pub fn reset_lighthouse_position(&mut self, bsd_idx: usize) {
		self.bsd[bsd_idx].position_set = false;
		self.bsd[bsd_idx].variance = AxisAnglePose::default();
	}

	// ---------------------------------------------------------------- configs

	/// survive_load_htc_config_format_from_file: the object's config from a file; false when it does not open. An
	/// empty file opens and gives nothing (the device then counts as configured without a config, as in the C). The C
	/// reads in text mode: on Windows a CR LF is read as LF, and a Ctrl-Z ends the text
	pub fn load_config_file(&mut self, obj: ObjectId, filename: &str) -> bool {
		let Ok(bytes) = std::fs::read(filename) else { return false };
		let bytes = if cfg!(windows) { textmode::read(&bytes) } else { bytes };
		if !bytes.is_empty() {
			self.config(obj, Some(bytes));
		}
		true
	}

	/// survive_default_config_process: a config recorded (CONFIG), kept (so->conf), and read into the object; 0 when
	/// the object took it. None is a stream that did not inflate (the C's length of -1: not recorded, and not readable)
	pub fn config(&mut self, obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
		if let Some(j) = json.as_ref() {
			self.record_object(obj, |r, t, codename| r.config(t, codename, j));
		}
		let bytes = json.unwrap_or_default();
		let rtn = self.load_device_config(obj, &bytes);
		self.objects[obj].conf = Some(bytes);
		if rtn == 0 {
			// sally: the tracker starts from what earlier sessions learned of the device
			self.learned_device(obj);
		}
		if self.settings.serialize_device_config {
			let so = &self.objects[obj];
			let conf = so.conf.clone().unwrap_or_default();
			for name in [so.serial_number.clone(), so.codename.clone()] {
				let raw_fname = format!("{name}_config.json");
				let _ = write_whole(std::path::Path::new(&raw_fname), &textmode::file(&conf));
				self.info(&format!("Wrote out {} bytes to {}", conf.len(), raw_fname));
			}
		}
		rtn
	}

	/// survive_load_htc_config_format: the config's sensors, IMU calibration and kind into the object, with the
	/// lines the C logs while it reads them; 0 when taken. A config on which the C's behaviour is undefined (it would
	/// read or write out of bounds: see sally_lighthouse::device) is refused, with a warning saying why
	fn load_device_config(&mut self, obj: ObjectId, json: &[u8]) -> i32 {
		let options = ConfigOptions {
			imu_acc_bias_units: self.settings.imu_acc_bias_units,
			ignore_config_imu_bias: self.settings.ignore_config_imu_bias,
			colorize: self.log.colorizes(),
		};
		let codename = self.objects[obj].codename.clone();
		match DeviceConfig::parse(json, &codename, &options) {
			Ok(cfg) => {
				for line in &cfg.log {
					self.device_config_log(line);
				}
				let so = &mut self.objects[obj];
				so.object_type = cfg.object_type as i32;
				so.object_subtype = cfg.object_subtype as i32;
				let n = cfg.serial_number.iter().position(|&c| c == 0).unwrap_or(cfg.serial_number.len());
				so.serial_number = String::from_utf8_lossy(&cfg.serial_number[..n]).into_owned();
				so.sensor_ct = cfg.sensor_ct as i8;
				so.sensor_locations = cfg.sensor_locations.map(|l| l.iter().flatten().copied().collect());
				so.sensor_normals = cfg.sensor_normals.map(|l| l.iter().flatten().copied().collect());
				so.channel_map = cfg.channel_map.map(|m| m.to_vec());
				so.has_sensor_locations = cfg.has_sensor_locations;
				so.imu_freq = cfg.imu_freq;
				so.head2trackref = cfg.head2trackref;
				so.imu2trackref = cfg.imu2trackref;
				so.head2imu = cfg.head2imu;
				so.raw_acc_scale = cfg.raw_acc_scale;
				so.raw_gyro_scale = cfg.raw_gyro_scale;
				so.acc_bias = cfg.acc_bias;
				so.acc_scale = cfg.acc_scale;
				so.gyro_bias = cfg.gyro_bias;
				so.gyro_scale = cfg.gyro_scale;
				0
			}
			Err(e) => {
				if let Some(line) = e.log() {
					self.device_config_log(&line);
				}
				match e.code() {
					Some(code) => code,
					None => {
						self.warn(&format!("{codename}: {e}"));
						-1
					}
				}
			}
		}
	}

	/// sally: the device `device` is the one on the object's dongle (the tap's assign, from SteamVR's pairing): its own
	/// config into the object, from `<device>_config.json` in the working directory (the runtime writes one for every
	/// device it configures) or else SteamVR's copy (Steam\config\lighthouse\<serial>\config.json, which Valve's driver
	/// writes for every device it connects: a device never seen by sally has one). An object that had another device's
	/// config starts over first (`restart_object`): nothing of the other device carries over. A device with no config
	/// anywhere is looked for again at most every 5 s, and logged once
	pub fn identify_object(&mut self, obj: ObjectId, device: &str) -> Identified {
		const RETRY_S: f64 = 5.0;
		let so = &self.objects[obj];
		if so.conf.is_some() && so.serial_number.eq_ignore_ascii_case(device) {
			return Identified::Same;
		}
		let now = self.peek_run_time();
		let miss = self.identify_misses.iter().position(|(d, _)| d == device);
		if let Some(m) = miss {
			if now - self.identify_misses[m].1 < RETRY_S {
				return Identified::Failed;
			}
		}
		let steamvr = steamvr::Paths::find();
		let options = ConfigOptions {
			imu_acc_bias_units: self.settings.imu_acc_bias_units,
			ignore_config_imu_bias: self.settings.ignore_config_imu_bias,
			colorize: false,
		};
		let Some((json, from)) = device_config(device, &steamvr, &options) else {
			let (name, dongle) = (self.objects[obj].codename.clone(), self.objects[obj].sally_dongle.clone());
			match miss {
				Some(m) => self.identify_misses[m].1 = now,
				None => {
					self.identify_misses.push((device.to_string(), now));
					// the latest MAX_MISSES devices (a client naming made-up ones fills nothing)
					if self.identify_misses.len() > MAX_MISSES {
						self.identify_misses.remove(0);
					}
					let where_steamvr = steamvr
						.device_config_path(device)
						.map_or_else(|| "SteamVR's folders not found".to_string(), |p| p.display().to_string());
					self.warn(&format!(
						"hidtap: {name} (dongle {dongle}) is {device}, but there is no config of its own for it \
						 ({device}_config.json here, {where_steamvr}): not tracked"
					));
				}
			}
			return Identified::Failed;
		};
		if let Some(m) = miss {
			self.identify_misses.remove(m);
		}
		let so = &self.objects[obj];
		let (name, dongle) = (so.codename.clone(), so.sally_dongle.clone());
		let before = so.conf.is_some().then(|| so.serial_number.clone());
		if before.is_some() {
			self.restart_object(obj);
		}
		if self.config(obj, Some(json.clone())) != 0 {
			self.warn(&format!("hidtap: {name} (dongle {dongle}): {device}'s config from {from} was not taken"));
			self.restart_object(obj);
			return Identified::Failed;
		}
		let was = match before {
			Some(b) => format!(", was {b}: started over"),
			None => String::new(),
		};
		self.info(&format!("hidtap: {name} (dongle {dongle}) is {device}, its config from {from}{was}"));
		Identified::Configured(json)
	}

	/// sally: the object's device is on another dongle now (or SteamVR knows of none on its dongle): the object starts
	/// over without a config, so nothing tracks it as that device
	pub fn release_object(&mut self, obj: ObjectId) {
		let so = &self.objects[obj];
		let (name, dongle, serial) = (so.codename.clone(), so.sally_dongle.clone(), so.serial_number.clone());
		self.restart_object(obj);
		self.info(&format!("hidtap: {name} (dongle {dongle}) does not carry {serial} any more"));
	}

	/// sally: the object as survive_create_device makes it, its names and its dongle kept (the tap's object for a dongle
	/// that carries another device now): no config, a new tracker, no light, no pose; the drivers forget it
	/// (device_reset)
	pub fn restart_object(&mut self, obj: ObjectId) {
		self.learned_detach(obj);
		let old = &self.objects[obj];
		let mut so = TrackedObject::new(&old.drivername, &old.codename, self.settings.activations);
		so.sally_dongle = old.sally_dongle.clone();
		so.serial_number = old.sally_dongle.chars().take(15).collect();
		so.tracker = Some(self.new_tracker());
		self.objects[obj] = so;
		self.device_reset_hook(obj);
	}

	/// A line the device config's load logs, as the C's log hook takes it
	fn device_config_log(&mut self, line: &DeviceLog) {
		match line {
			DeviceLog::Warn(t) => self.warn(&String::from_utf8_lossy(t)),
			DeviceLog::Info(t) => self.info(&String::from_utf8_lossy(t)),
			DeviceLog::Verbose(level, t) => {
				if self.settings.log_level >= *level {
					self.info(&String::from_utf8_lossy(t));
				}
			}
			DeviceLog::GeneralError(t) => {
				self.current_error = SURVIVE_ERROR_GENERAL;
				self.info(&String::from_utf8_lossy(t));
			}
		}
	}

	// ---------------------------------------------------------------- the generation

	pub fn notify_gen2(&mut self, obj: ObjectId, _reason: &str) {
		if self.settings.lh_version_forced != -1 && self.settings.lh_version_forced != 1 {
			return;
		}
		if self.lh_version != 1 {
			self.gen_detected(obj, 1);
		}
	}

	/// Whether LH1 base stations have been seen this run
	pub fn saw_lh1(&self) -> bool {
		self.lh1_warned
	}

	/// LH1 base stations seen. sally tracks with LH2 alone: said once, and the system's generation stays unknown, so
	/// nothing is tracked from their light
	fn lh1_seen(&mut self) {
		if !self.lh1_warned {
			self.warn("LH1 base stations: sally tracks with LH2 base stations only");
			self.lh1_warned = true;
		}
	}

	/// survive_default_gen_detected_process: the system is LH2 (the one generation sally tracks); each device's
	/// measurements start over (its clock extensions kept)
	fn gen_detected(&mut self, _obj: ObjectId, lh_version: i32) {
		self.info(&format!("Detected LH gen {} system.", lh_version + 1));
		let configed = self.settings.lh_version_configed;
		if lh_version != configed && configed != -1 {
			self.warn(&format!("Configuration was valid for gen {}; resetting BSD positions and OOTX", configed + 1));
			for b in self.bsd.iter_mut() {
				b.position_set = false;
				b.ootx_set = false;
				b.mode = 0xFF;
			}
		}
		for (o, &listed) in self.objects.iter_mut().zip(&self.listed) {
			if !listed {
				continue;
			}
			let act = &mut o.activations;
			let (last_imu, last_light, runtime_offset) = (act.last_imu, act.last_light, act.runtime_offset);
			act.reset();
			act.last_imu = last_imu;
			act.last_light = last_light;
			act.runtime_offset = runtime_offset;
		}
		self.lh_version = lh_version;
		self.config.configi("configed-lighthouse-gen", SC_OVERRIDE | SC_SETCONFIG, (lh_version + 1) as u32);
		self.config_save();
	}

	/// A pulse (a device in raw0 mode). While the generation is unknown, the pulses tell it (OOTX pulse lengths at
	/// LH1's 60 or 120 Hz are LH1 stations, which sally does not track; 500 pulses without say LH2); after that a pulse
	/// has no use (sally has no LH1 decoder)
	pub fn lightcap(&mut self, obj: ObjectId, le: &LightcapElement) -> bool {
		if le.length == 0 {
			self.warn(&format!(
				"LightcapElement has zero length [SensorId:{}] [Timestamp:{:04X}]",
				le.sensor_id, le.timestamp as u16
			));
			return false;
		}
		if self.lh_version == -1 {
			if self.lh1_warned {
				return true;
			}
			let dv = self.objects[obj].gen_detect.get_or_insert_with(GenDetect::default);
			dv.total_count += 1;
			let is_ootx_pulse_length = le.length >= 3000 && le.length < 6500;
			// (a sensor id past the array, which only a wired tracker's report can carry, is not timed)
			if is_ootx_pulse_length && (le.sensor_id as usize) < dv.last_pulse_times.len() {
				let s = le.sensor_id as usize;
				let first_pulse = dv.last_pulse_times[s] == 0;
				let pulse_dist = le.timestamp.wrapping_sub(dv.last_pulse_times[s]);
				dv.last_pulse_times[s] = le.timestamp;
				if !first_pulse {
					let dist_at_120hz: u32 = 400000;
					let dist_at_60hz = 2 * dist_at_120hz;
					let d = pulse_dist as f64;
					let matches_60hz_lh1 = d > dist_at_60hz as f64 * 0.95 && d < dist_at_60hz as f64 * 1.05;
					let matches_120hz_lh1 = d > dist_at_120hz as f64 * 0.95 && d < dist_at_120hz as f64 * 1.05;
					if matches_120hz_lh1 || matches_60hz_lh1 {
						let n = dv.pulse_count;
						dv.pulse_count += 1;
						if n > 10 {
							self.remove_all_gen_detect();
							self.lh1_seen();
							return true;
						}
					}
				}
			}
			let total = self.objects[obj].gen_detect.as_ref().map(|d| d.total_count).unwrap_or(0);
			if total > 500 {
				self.remove_all_gen_detect();
				self.notify_gen2(obj, "no OOTX pulses detected");
			}
			return true;
		}
		self.record_object(obj, |r, t, codename| r.lightcap(t, codename, le.sensor_id, le.timestamp, le.length));
		if self.objects[obj].map_sensor_id(le.sensor_id) == 255 {
			return false;
		}
		true
	}

	/// remove_all_disambiguate_version
	fn remove_all_gen_detect(&mut self) {
		for (o, &listed) in self.objects.iter_mut().zip(&self.listed) {
			if listed {
				o.gen_detect = None;
			}
		}
	}

	// ---------------------------------------------------------------- the IMU

	/// survive_default_raw_imu_process: the raw counts scaled and calibrated, the raw sample recorded (i), and the
	/// calibrated one to the IMU process
	pub fn raw_imu(&mut self, obj: ObjectId, mask: i32, accelgyromag: &[f64; 9], timecode: u32, id: i32) {
		let so = &self.objects[obj];
		let mut agm = *accelgyromag;
		for i in 0..3 {
			agm[i] *= so.raw_acc_scale;
		}
		for i in 3..6 {
			agm[i] *= so.raw_gyro_scale;
		}
		// calibrate_acc, calibrate_gyro
		for i in 0..3 {
			agm[i] -= so.acc_bias[i];
		}
		for i in 0..3 {
			agm[i] *= so.acc_scale[i];
		}
		for i in 0..3 {
			agm[3 + i] -= so.gyro_bias[i];
		}
		for i in 0..3 {
			agm[3 + i] *= so.gyro_scale[i];
		}
		self.record_object(obj, |r, t, codename| r.raw_imu(t, codename, mask, accelgyromag, timecode, id));
		self.imu_hook(obj, 3, &agm, timecode, id);
	}

	/// survive_default_imu_process: the sample on the device's long clock, into its activations and its tracker (whose
	/// report goes out through the imupose and velocity hooks), then recorded calibrated (I) when asked
	pub fn imu(&mut self, obj: ObjectId, mask: i32, accelgyromag: &[f64; 9], timecode: u32, id: i32) {
		let so = &mut self.objects[obj];
		let long_timecode = so.activations.long_timecode_imu(timecode);
		let imu = ImuSample {
			timecode: long_timecode,
			accel: [accelgyromag[0], accelgyromag[1], accelgyromag[2]],
			gyro: [accelgyromag[3], accelgyromag[4], accelgyromag[5]],
			mag: [accelgyromag[6], accelgyromag[7], accelgyromag[8]],
		};
		so.activations.add_imu(&imu);
		if let Some(o) = self.observer.as_mut() {
			o.imu(obj, long_timecode, &imu.accel, &imu.gyro);
		}
		self.tracker_imu(obj, long_timecode, &imu.accel, &imu.gyro);
		self.record_object(obj, |r, t, codename| r.imu(t, codename, mask, accelgyromag, timecode, id));
	}

	/// survive_default_set_imu_scale_modes: recorded (IMU_SCALES), then the scales
	pub fn set_imu_scale_modes(&mut self, obj: ObjectId, gyro_scale_mode: i32, acc_scale_mode: i32) {
		self.record_object(obj, |r, t, codename| r.imu_scales(t, codename, gyro_scale_mode, acc_scale_mode));
		self.objects[obj].set_imu_scale_modes(gyro_scale_mode, acc_scale_mode);
	}

	// ---------------------------------------------------------------- the gen2 light

	/// survive_default_sync_process: a station's sync. Recorded (Y); its interval from the last one checked against
	/// the channel's rotor (an interval more than 1 Hz off is dropped, and after more than 10 rotations the rotor is
	/// timed afresh); the OOTX bit (and a -1 for each sync skipped) to the station's decoder; the sync to the activations
	pub fn sync(&mut self, obj: ObjectId, channel: u8, timecode: u32, ootx: bool, gen: bool) {
		let bsd_idx = self.get_bsd_idx(channel);
		if bsd_idx == -1 {
			let msg = format!("Invalid channel requested({}) for {}", channel, self.objects[obj].codename);
			self.warn(&msg);
			return;
		}
		let b = bsd_idx as usize;
		if self.bsd[b].disable {
			return;
		}
		self.record_object(obj, |r, t, codename| r.sync(t, codename, channel, timecode, ootx, gen));

		let is_initing = self.objects[obj].last_sync_time[b] == 0;
		if !is_initing {
			let so = &mut self.objects[obj];
			let time_delta = gen2::timecode_difference(timecode, so.last_sync_time[b]);
			let interval = gen2::sync_interval(time_delta, channel as usize);
			let skipped_syncs = interval.rotations_since - 1;
			if interval.err > 1.0 {
				if skipped_syncs > 10 {
					so.last_sync_time[b] = 0;
					so.stats.sync_resets[b] = so.stats.sync_resets[b].wrapping_add(1);
				}
				so.stats.bad_syncs[b] = so.stats.bad_syncs[b].wrapping_add(skipped_syncs as u32);
				so.sync_count[b] = so.sync_count[b].wrapping_add(skipped_syncs as u32);
				return;
			}
			so.stats.skipped_syncs[b] = so.stats.skipped_syncs[b].wrapping_add(skipped_syncs as u32);
			// Every skipped sync halves our ootx success rate; but if we don't send anything in it will be wrong 100% of
			// the time
			for _ in 0..skipped_syncs {
				self.ootx_behavior(obj, b, self.lh_version, -1);
			}
			self.objects[obj].last_time_between_sync[b] = interval.time_delta;
		}

		let so = &mut self.objects[obj];
		so.stats.syncs[b] = so.stats.syncs[b].wrapping_add(1);
		so.sync_count[b] = so.sync_count[b].wrapping_add(1);
		so.last_sync_time[b] = timecode;

		self.ootx_behavior(obj, b, self.lh_version, ootx as i8);

		let so = &mut self.objects[obj];
		let long_timecode = so.activations.long_timecode_light(timecode);
		if (b as i32) < self.active_lighthouses {
			if so.activations.add_gen2(&Gen2Light::Sync { timecode: long_timecode, lh: b }) {
				so.stats.accepted_data[b] = so.stats.accepted_data[b].wrapping_add(1);
			} else {
				so.stats.rejected_data[b] = so.stats.rejected_data[b].wrapping_add(1);
			}
		}
		so.stats.hit_from_lhs[b] = so.stats.hit_from_lhs[b].wrapping_add(1);
	}

	/// survive_default_sweep_process: a sensor hit by a station's sweep. Recorded (W); timed from the station's last
	/// sync into a rotor angle; given a plane; on to the angle process with its angle on that plane
	pub fn sweep(&mut self, obj: ObjectId, channel: u8, sensor_id: i32, timecode: u32, half_clock_flag: bool) {
		let bsd_idx = self.get_bsd_idx(channel);
		if bsd_idx == -1 {
			let msg = format!("Invalid channel requested({}) for {}", channel, self.objects[obj].codename);
			self.warn(&msg);
			return;
		}
		let b = bsd_idx as usize;
		if self.bsd[b].disable {
			return;
		}
		self.notify_gen2(obj, "sweep called");
		self.record_object(obj, |r, t, codename| r.sweep(t, codename, channel, sensor_id, timecode, half_clock_flag));

		let so = &mut self.objects[obj];
		let last_sweep = so.last_sync_time[b];
		if last_sweep == 0 {
			return;
		}
		let time_delta = gen2::timecode_difference(timecode, last_sweep);
		let timing = gen2::sweep_timing(time_delta, half_clock_flag, so.last_time_between_sync[b], channel as usize);
		if timing.rotations_since > 10 {
			so.stats.dropped_light[b] = so.stats.dropped_light[b].wrapping_add(1);
			return;
		}
		let angle = timing.angle;
		let plane = self.determine_plane(obj, b, sensor_id, angle);
		let so = &mut self.objects[obj];
		so.stats.hit_from_lhs[b] = so.stats.hit_from_lhs[b].wrapping_add(1);
		if plane >= 0 {
			self.sweep_angle_hook(obj, channel, sensor_id, timecode, plane, angle_for_axis(angle)[plane as usize]);
		}
	}

	/// determine_plane: the plain rule (with the means near its split), or, near where the plain rule fails, the
	/// plane whose predicted pulse is nearer (gen2-plane-predict: from the device's pose while the tracker holds one);
	/// -1 drops the hit
	fn determine_plane(&mut self, obj: ObjectId, bsd_idx: usize, sensor_id: i32, angle: f64) -> i8 {
		let naive_only = self.settings.naive_plane_only;
		if !naive_only {
			if let Some(plane) = self.predicted_plane(obj, bsd_idx, sensor_id, angle) {
				return plane;
			}
		}
		let so = &mut self.objects[obj];
		let center = so.activations.angles_center_x[bsd_idx];
		plane_from_angle(angle, &center, naive_only, &mut so.stats.plane)
	}

	/// determine_plane's prediction: the plane (or -1) when the predicted pulses decide it, None for the plain rule
	fn predicted_plane(&mut self, obj: ObjectId, bsd_idx: usize, sensor_id: i32, angle: f64) -> Option<i8> {
		let predict_margin = self.settings.gen2_plane_predict;
		let predict_tol = self.settings.gen2_plane_predict_tol;
		let naive_plane = (angle > LINMATHPI) as i8;
		let so = &self.objects[obj];
		if !(predict_margin > 0.0
			&& self.bsd[bsd_idx].position_set
			&& so.sensor_locations.is_some()
			&& sensor_id >= 0
			&& sensor_id < so.sensor_ct as i32
			&& !quatiszero(&so.out_pose_imu.rot)
			&& self.plane_pose_held(obj))
		{
			return None;
		}
		let so = &self.objects[obj];
		let pw = apply_pose_to_point(&so.out_pose_imu, &sensor_location(so, sensor_id));
		let pred = self.reproject_gen2(bsd_idx, &pw);
		if !(pred[0].is_finite() && pred[1].is_finite()) {
			return None;
		}
		let r0 = pred[0] + 2.0 / 3.0 * LINMATHPI;
		let r1 = pred[1] + 4.0 / 3.0 * LINMATHPI;
		if !(r0 > LINMATHPI - predict_margin || r1 < LINMATHPI + predict_margin) {
			return None;
		}
		let st = &mut self.objects[obj].stats;
		st.plane_predict_checks = st.plane_predict_checks.wrapping_add(1);
		let (d0, d1) = (rotor_dist(angle, r0), rotor_dist(angle, r1));
		let plane = (d1 < d0) as i8;
		if (if plane != 0 { d1 } else { d0 }) > predict_tol {
			st.plane_predict_drops = st.plane_predict_drops.wrapping_add(1);
			return Some(-1);
		}
		if plane != naive_plane {
			st.plane_predict_fixes = st.plane_predict_fixes.wrapping_add(1);
		}
		Some(plane)
	}

	/// A hit's angle on its plane. Past the bench's simulated occlusions and the reflection gate (light-facing-deg: a
	/// sensor facing away from the station, by the device's pose), to the tracker and the activations, and recorded (B)
	pub fn sweep_angle(&mut self, obj: ObjectId, channel: u8, sensor_id: i32, timecode: u32, plane: i8, angle: f64) {
		let bsd_idx = self.get_bsd_idx(channel);
		if bsd_idx == -1 {
			let msg = format!("Invalid channel requested({}) for {}", channel, self.objects[obj].codename);
			self.warn(&msg);
			return;
		}
		let b = bsd_idx as usize;
		if self.settings.sim_gap_period > 0.0 {
			let t = self.run_time();
			// the object's index in ctx->objs (0 if not listed)
			let idx = if self.listed[obj] { obj } else { 0 };
			let p = self.settings.sim_gap_period;
			let ph = (t + idx as f64 * p / 6.37) % p;
			if t > self.settings.sim_gap_start && ph < self.settings.sim_gap_len {
				return;
			}
		}
		let long_timecode = self.objects[obj].activations.long_timecode_light(timecode);
		if self.settings.record_facing && self.recording.is_some() && self.facing_known(obj, b, sensor_id) {
			let (pw, nw, dir) = self.facing(obj, b, sensor_id);
			let pred = self.reproject_gen2(b, &pw);
			let incidence = dot3d(&nw, &dir);
			let residual = angle - pred[plane as usize];
			self.record_object(obj, |r, t, codename| {
				r.facing(t, codename, b as i8, sensor_id, plane, angle, residual, incidence)
			});
		}
		// light-facing-deg (its cosine; -2 off), with the pose fresh within light-facing-max-age
		let deg = self.settings.light_facing_deg;
		let rf_cos = if deg > 0.0 && deg < 180.0 { (deg * LINMATHPI / 180.0).cos() } else { -2.0 };
		if rf_cos > -1.0
			&& self.facing_known(obj, b, sensor_id)
			&& self.rf_pose_fresh(obj, self.settings.light_facing_max_age)
		{
			let (_, nw, dir) = self.facing(obj, b, sensor_id);
			let st = &mut self.objects[obj].stats;
			st.facing_checks = st.facing_checks.wrapping_add(1);
			if dot3d(&nw, &dir) < rf_cos {
				st.facing_drops = st.facing_drops.wrapping_add(1);
				return;
			}
		}
		// the tracker takes every hit that passed the reflection gate, before the activations filter
		if let Some(o) = self.observer.as_mut() {
			o.hit(obj, b, plane, sensor_id, long_timecode, angle);
		}
		self.tracker_hit(obj, long_timecode, b as i32, plane as i32, sensor_id, angle);
		let so = &mut self.objects[obj];
		if (b as i32) < self.active_lighthouses {
			let light = Gen2Light::Hit {
				timecode: long_timecode,
				lh: b,
				sensor_id: sensor_id as usize,
				plane: plane as usize,
				angle,
			};
			if sensor_id >= 0 && so.activations.add_gen2(&light) {
				so.stats.accepted_data[b] = so.stats.accepted_data[b].wrapping_add(1);
			} else {
				so.stats.rejected_data[b] = so.stats.rejected_data[b].wrapping_add(1);
			}
		}
		self.record_object(obj, |r, t, codename| {
			r.sweep_angle(t, codename, channel, sensor_id, timecode, plane, angle)
		});
	}

	// ---------------------------------------------------------------- OOTX

	/// survive_ootx_behavior: a station's OOTX bit (-1 for a sync not seen) to its decoder, which the first device to
	/// sync it is given and feeds alone, until a packet checks the station
	fn ootx_behavior(&mut self, obj: ObjectId, bsd_idx: usize, lh_version: i32, ootx: i8) {
		if self.bsd[bsd_idx].ootx_checked {
			return;
		}
		if self.bsd[bsd_idx].ootx.is_none() {
			let codename = self.objects[obj].codename.clone();
			let msg = if lh_version == 1 {
				format!(
					"OOTX not set for LH in channel {}; attaching ootx decoder using device {}",
					self.bsd[bsd_idx].mode, codename
				)
			} else {
				format!("OOTX not set for LH {bsd_idx}; attaching ootx decoder using device {codename}")
			};
			self.info(&msg);
			let mut decoder = OotxDecoder::new(self.run_time() as f32);
			decoder.ignore_sync_bit_error = self.config.configi("ootx-ignore-sync-error", SC_SETCONFIG, 0) as i32;
			self.bsd[bsd_idx].ootx = Some(OotxSlot { decoder, device: obj });
		}
		let Some(slot) = self.bsd[bsd_idx].ootx.as_mut() else { return };
		if slot.device != obj {
			return;
		}
		let event = slot.decoder.pump_bit(ootx);
		match event {
			Some(OotxEvent::Error(msg)) => self.ootx_error(bsd_idx, msg),
			Some(OotxEvent::BadCrc { .. }) | None => {}
			Some(OotxEvent::Packet(packet)) => self.ootx_packet_gen2(bsd_idx, &packet),
		}
		if self.bsd[bsd_idx].ootx_checked {
			self.bsd[bsd_idx].ootx_checked = false;
		}
	}

	/// ootx_error_clbk_d: the decoder's messages, for a station without OOTX yet
	fn ootx_error(&mut self, id: usize, msg: &str) {
		if !self.bsd[id].ootx_set {
			let mode = self.bsd[id].mode;
			let shown = if mode != 255 { mode as usize } else { id };
			self.info(&format!("({shown}) {msg}"));
		}
	}

	/// ootx_packet_clbk_d_gen2: a gen2 station's info block: its calibration, up direction and channel taken when the
	/// station or its up direction is new to the runtime (then its position is forgotten, unless lock-lighthouses
	/// keeps a known one), and the OOTX received process
	fn ootx_packet_gen2(&mut self, id: usize, packet: &OotxPacket) {
		// sally: a gen2 info block is 43 bytes; anything shorter, or a station id of 0, is not a real packet
		let Some(v15) = LighthouseInfoV15::new(&packet.data).filter(|_| packet.length >= 43) else {
			let msg = format!(
				"Ignoring a {}-byte OOTX packet on LH {} (channel {}); a gen2 station sends 43 bytes",
				packet.length, id, self.bsd[id].mode
			);
			self.warn(&msg);
			return;
		};
		if v15.id == 0 {
			let msg = format!("Ignoring an OOTX packet with station id 0 on LH {} (channel {})", id, self.bsd[id].mode);
			self.warn(&msg);
			return;
		}
		if self.settings.serialize_ootx {
			// opened with "w" in the C, so on Windows its 0x0A bytes go out as CR LF too
			let filename = format!("LH{:02}_{:08x}.ootx", v15.mode_current & 0x7F, v15.id);
			let n = (packet.length as usize).min(packet.data.len());
			let _ = std::fs::write(filename, textmode::file(&packet.data[..n]));
		}
		let b = &mut self.bsd[id];
		b.ootx_checked = true;
		let accel = [v15.accel_dir[0] as f64, v15.accel_dir[1] as f64, v15.accel_dir[2] as f64];
		let up_changed = norm3d(&b.accel) != 0.0 && dist3d(&b.accel, &accel) > 1e-3;
		let same_station = b.base_station_id == v15.id;
		let do_save = b.base_station_id != v15.id || !b.ootx_set || up_changed;
		b.ootx_set = true;
		if do_save {
			// sally: a station is named as SteamVR names it, by its serial (a check's share finds a serial by that shape)
			let msg = format!("Got OOTX packet {} LHB-{:08X}", self.bsd[id].mode, v15.id);
			self.info(&msg);
			let b = &mut self.bsd[id];
			b.base_station_id = v15.id;
			for i in 0..2 {
				b.fcal[i].phase = v15.fcal_phase[i] as f64;
				b.fcal[i].tilt = v15.fcal_tilt[i] as f64;
				b.fcal[i].curve = v15.fcal_curve[i] as f64;
				b.fcal[i].gibpha = v15.fcal_gibphase[i] as f64;
				b.fcal[i].gibmag = v15.fcal_gibmag[i] as f64;
				b.fcal[i].ogeephase = v15.fcal_ogeephase[i] as f64;
				b.fcal[i].ogeemag = v15.fcal_ogeemag[i] as f64;
			}
			b.accel = accel;
			b.sys_unlock_count = v15.sys_unlock_count;
			// Although we know this already....
			b.mode = v15.mode_current & 0x7F;
			if same_station && b.position_set && self.config.configi("lock-lighthouses", SC_GET, 0) != 0 {
				self.info(&format!("Keeping known position for LH {} (LHB-{:08X})", id, v15.id));
			} else {
				self.reset_lighthouse_position(id);
			}
			self.ootx_received(id);
		}
	}

	/// survive_default_ootx_received_process: the station to the config (config_set_lighthouse), its up direction
	/// recorded (LH_UP), the config saved
	pub fn ootx_received(&mut self, bsd_idx: usize) {
		self.config_set_lighthouse(bsd_idx);
		let (mode, accel) = (self.bsd[bsd_idx].mode, self.bsd[bsd_idx].accel);
		self.record(|r, t| r.lh_up(t, mode, &accel));
		self.config_save();
	}

	/// survive_default_raw_lighthouse_pose_process: a station's pose (None: unknown). A known station's move below
	/// lighthouse-max-update or above lighthouse-max-nudge-distance is taken at once, one in between at
	/// lighthouse-update-velocity (survive_get_lighthouse_position slerps it); the station to the config, saved; then
	/// the lighthouse pose hook
	pub fn raw_lighthouse_pose(&mut self, idx: usize, pose: Option<&Pose>) {
		match pose {
			Some(p) if self.bsd[idx].position_set => {
				let d = dist3d(&p.pos, &self.bsd[idx].pose.pos);
				let m = &self.station_moves;
				if d < m.max_update.float() || d > m.max_nudge_distance.float() {
					self.bsd[idx].pose = *p;
					self.bsd[idx].true_pos = *p;
				} else {
					let velocity = m.update_velocity.float();
					let t = self.run_time();
					let b = &mut self.bsd[idx];
					b.old_pos = b.pose;
					b.true_pos = *p;
					b.old_pos_time = t;
					b.true_pos_time = b.old_pos_time + d / velocity;
				}
			}
			Some(p) => {
				self.bsd[idx].pose = *p;
				self.bsd[idx].true_pos = *p;
				self.bsd[idx].position_set = true;
			}
			None => self.bsd[idx].position_set = false,
		}
		self.config_set_lighthouse(idx);
		self.config_save();
		let mut external = self.bsd[idx].pose;
		external.pos[2] -= self.floor_offset;
		self.lighthouse_pose_hook(idx, &external);
	}

	/// survive_default_lighthouse_pose_process: recorded (LH_POSE)
	pub fn lighthouse_pose(&mut self, idx: usize, pose: &Pose) {
		self.record_lighthouse_pose(idx, pose);
	}

	/// survive_recording_lighthouse_process
	pub(crate) fn record_lighthouse_pose(&mut self, idx: usize, pose: &Pose) {
		let (mode, id) = (self.bsd[idx].mode as i8, self.bsd[idx].base_station_id);
		self.record(|r, t| r.lighthouse_pose(t, mode, pose, id));
	}

	/// survive_get_lighthouse_position: a station's pose. One being moved slowly (raw_lighthouse_pose's nudge) moves on
	/// to where it is at this run time, the step written into its pose and recorded (LH_POSE, the floor offset off)
	pub fn get_lighthouse_position(&mut self, idx: usize) -> Pose {
		if self.bsd[idx].true_pos_time != 0.0 {
			let t_diff = self.bsd[idx].true_pos_time - self.bsd[idx].old_pos_time;
			let mut t = (self.run_time() - self.bsd[idx].old_pos_time) / t_diff;
			if t > 1.0 {
				t = 1.0;
			}
			let b = &mut self.bsd[idx];
			if t >= 0.0 {
				b.pose = pose_slerp(&b.old_pos, &b.true_pos, t);
			}
			if t == 1.0 {
				b.true_pos_time = 0.0;
				b.old_pos_time = f64::NAN;
			}
			let mut p = b.pose;
			p.pos[2] -= self.floor_offset;
			self.record_lighthouse_pose(idx, &p);
		}
		self.bsd[idx].pose
	}

	/// The facing checks' condition on the device and the station: the station placed, the device's sensors known, the
	/// sensor one of them, and a pose (the light-facing gate checks it is fresh too)
	fn facing_known(&self, obj: ObjectId, b: usize, sensor_id: i32) -> bool {
		let so = &self.objects[obj];
		(b as i32) < self.active_lighthouses
			&& self.bsd[b].position_set
			&& so.sensor_locations.is_some()
			&& so.sensor_normals.is_some()
			&& sensor_id >= 0
			&& sensor_id < so.sensor_ct as i32
			&& !quatiszero(&so.out_pose_imu.rot)
	}

	/// A sensor in the world by the device's pose (so->OutPoseIMU): its position, its normal, and the direction from it
	/// to the station (survive_get_lighthouse_position's, which moves a slowly moving station on)
	fn facing(&mut self, obj: ObjectId, b: usize, sensor_id: i32) -> ([f64; 3], [f64; 3], [f64; 3]) {
		let so = &self.objects[obj];
		let pw = apply_pose_to_point(&so.out_pose_imu, &sensor_location(so, sensor_id));
		let nw = quatrotatevector(&so.out_pose_imu.rot, &sensor_normal(so, sensor_id));
		let lh2world = self.get_lighthouse_position(b);
		let dir = normalize3d(&sub3d(&lh2world.pos, &pw));
		(pw, nw, dir)
	}

	// ---------------------------------------------------------------- the config file

	/// The stations the config file holds (survive_init: config_read_lighthouse for each index, each found one given
	/// its channel's place and counted), then each lighthouse-N-disable
	pub fn load_lighthouses(&mut self) {
		self.active_lighthouses = 0;
		for i in 0..NUM_GEN2_LIGHTHOUSES {
			let mut record = self.bsd[i].record();
			if self.config.read_lighthouse(i as u8, &mut record) {
				self.bsd[i].set_record(&record);
				let mode = self.bsd[i].mode;
				if mode < 16 {
					self.bsd_map[mode as usize] = i as i8;
				}
				self.active_lighthouses += 1;
			}
			let disable = self.config.configi(format!("lighthouse-{i}-disable"), SC_GET, 0) != 0;
			self.bsd[i].disable = disable;
			if disable {
				self.warn(&format!("Disabling LH {i}"));
			}
		}
	}

	/// config_set_lighthouse: the station into its group of the config (which normalizes the station's rotation)
	pub fn config_set_lighthouse(&mut self, idx: usize) {
		let mut record = self.bsd[idx].record();
		self.config.set_lighthouse(idx as u8, &mut record);
		self.bsd[idx].pose.rot = record.pose.rot;
	}

	/// config_save: the config file written with the stations that have OOTX (unless config-readonly or
	/// lock-lighthouses), a warning the first time it cannot be
	pub fn config_save(&mut self) {
		let mut ootx_set = [false; CONFIG_LIGHTHOUSES];
		for (o, b) in ootx_set.iter_mut().zip(&self.bsd) {
			*o = b.ootx_set;
		}
		let outcome = self.config.save(&ootx_set);
		// SV_VERBOSE(10, ..) for a read-only config, SV_WARN for one that cannot be written
		if let (SaveOutcome::OpenFailed { .. }, Some(msg)) = (&outcome, outcome.message()) {
			self.warn(&String::from_utf8_lossy(&msg));
		}
	}

	/// survive_config_set_live (sally): a value set while running, from text, into the store (Config::set_live: into
	/// the group that holds the tag, else the temporary one; never the file), and given at once to the variables the C
	/// binds to the tag. Bound in the store, and so written by Config::set_live: the context's three
	/// (lighthouse-max-update and its kin), the recording's record-* while one is open, the diagnosis's and the map
	/// solve's. Bound per device, each device's copy given the store's values again: its tracker's (track-*) and its
	/// light filter's (move-threshold-*, filter-*), which the C binds as the device is made. Not bound here: the LH1
	/// disambiguator's light-min-length and disambiguator-verbosity (the C binds them when an LH1 pulse first reaches
	/// it, which on a gen2 system none does) and the drivers that are not ported (the .rec playback's playback-*, the
	/// simulator's). Returns -1 for an unknown tag, -2 for a bad value (nothing is set), else the number of variables
	/// bound to the tag (`bound_count`'s, the C's)
	pub fn set_live(&mut self, tag: impl AsRef<[u8]>, value: impl AsRef<[u8]>) -> i32 {
		let tag = tag.as_ref();
		let bound = self.config.set_live(tag, value);
		if bound < 0 {
			return bound;
		}
		let tag = option_tag(tag);
		bound + self.tracker_option_set(tag) + self.activations_option_set(tag)
	}

	/// survive_config_bound_count (sally): the number of variables bound to the tag, which a set reaches at once: the
	/// store's, then one per device for a tracker or a light filter option
	pub fn bound_count(&self, tag: impl AsRef<[u8]>) -> i32 {
		let tag = tag.as_ref();
		let per_device = self.tracker_bound(option_tag(tag)) + self.activations_bound(option_tag(tag));
		self.config.bound_count(tag) + per_device
	}

	/// A light filter option set live: each device's activations given the options as the store has them now, and
	/// the runtime's copy, which a device made later starts with (as tracker_option_set does for the trackers, with
	/// its caveat). The number of devices bound to the tag: every device made has its activations
	fn activations_option_set(&mut self, tag: &str) -> i32 {
		if !ACTIVATION_OPTIONS.contains(&tag) {
			return 0;
		}
		self.settings.activations = activation_params(&mut self.config);
		for so in self.objects.iter_mut() {
			so.activations.params = self.settings.activations;
		}
		self.objects.len() as i32
	}

	/// The devices whose activations are bound to the tag (all of them, for a light filter option)
	fn activations_bound(&self, tag: &str) -> i32 {
		if ACTIVATION_OPTIONS.contains(&tag) {
			self.objects.len() as i32
		} else {
			0
		}
	}

	/// survive_detach_config, with what the C logs when it cannot
	pub(crate) fn detach_config(&mut self, tag: &str, var: &Binding) {
		match self.config.detach(tag, var) {
			Ok(()) => {}
			Err(e @ DetachError::NotInitialized(_)) => self.error(SURVIVE_ERROR_GENERAL, &e.to_string()),
			Err(e @ DetachError::NotAttached(_)) => self.warn(&e.to_string()),
		}
	}

	/// The listed devices (ctx->objs), in their order
	pub fn listed_objects(&self) -> impl Iterator<Item = ObjectId> + '_ {
		(0..self.objects.len()).filter(|&i| self.listed[i])
	}

	// ---------------------------------------------------------------- input

	/// The button servicer's work for one entry, done as the entry is posted (the C's servicer thread does it a moment
	/// later): recorded (BUTTON), then the button process: survive-cli's, where the menu button forgets the stations'
	/// positions unless the map is locked
	pub fn button(&mut self, obj: ObjectId, entry: &ButtonEntry) {
		self.record_object(obj, |r, t, codename| r.button(t, codename, entry.event_type, entry.button_id));
		// SURVIVE_BUTTON_MENU
		if entry.button_id == 6 && self.config.configi("lock-lighthouses", SC_GET, 0) == 0 {
			self.reset_lighthouse_positions();
		}
	}

	/// survive_reset_lighthouse_positions: every active station's position forgotten, and the floor offset
	pub fn reset_lighthouse_positions(&mut self) {
		self.floor_offset = 0.0;
		for i in 0..self.active_lighthouses.max(0) as usize {
			self.reset_lighthouse_position(i);
		}
	}

	// ---------------------------------------------------------------- shutdown

	/// survive_close's end, after the drivers: the config saved, each device destroyed (the first, then the last moved
	/// into its place: survive_destroy_device's swap) with its disconnect recorded, then the recording closed and the
	/// context's options unbound
	pub fn close(&mut self) {
		self.config_save();
		// sally: what the trackers learned of the devices, before they go
		self.learned_save(false);
		let mut order: Vec<ObjectId> = (0..self.objects.len()).filter(|&i| self.listed[i]).collect();
		while !order.is_empty() {
			let obj = order[0];
			self.record_object(obj, |r, t, codename| r.disconnect(t, codename));
			self.device_summary(obj);
			self.free_tracker(obj);
			let last = order.len() - 1;
			order[0] = order[last];
			order.pop();
		}
		self.destroy_recording();
		// SurviveContext_detach_config
		let moves = self.station_moves.clone();
		for (tag, var) in moves.items() {
			self.detach_config(tag, var);
		}
		self.log.close();
	}

	/// survive_destroy_device's lines that show at the default level
	fn device_summary(&mut self, obj: ObjectId) {
		let so = &self.objects[obj];
		let (c, st) = (so.codename.clone(), so.stats.clone());
		if st.facing_checks != 0 {
			self.info(&format!(
				"{} light facing check: {} hits checked, {} dropped as facing away (light-facing-deg)",
				c, st.facing_checks, st.facing_drops
			));
		}
		if st.plane_predict_checks != 0 {
			self.info(&format!(
				"{} gen2 plane from the predicted pose: {} pulses checked, {} relabeled, {} dropped",
				c, st.plane_predict_checks, st.plane_predict_fixes, st.plane_predict_drops
			));
		}
	}
}

/// A tag as the store takes it (a C string: up to its first NUL), as text to compare with the options' names (one
/// that is not UTF-8 names none of them)
fn option_tag(tag: &[u8]) -> &str {
	let end = tag.iter().position(|&c| c == 0).unwrap_or(tag.len());
	std::str::from_utf8(&tag[..end]).unwrap_or("")
}

/// rotor_dist: the circular distance between two rotor angles (rad)
fn rotor_dist(a: f64, b: f64) -> f64 {
	let d = (a - b).abs() % (2.0 * LINMATHPI);
	if d > LINMATHPI {
		2.0 * LINMATHPI - d
	} else {
		d
	}
}

/// &so->sensor_locations[sensor_id * 3], for a sensor the caller checked (with the array)
fn sensor_location(so: &TrackedObject, sensor_id: i32) -> [f64; 3] {
	let l = so.sensor_locations.as_deref().unwrap_or_default();
	let j = 3 * sensor_id as usize;
	[l[j], l[j + 1], l[j + 2]]
}

/// &so->sensor_normals[sensor_id * 3]
fn sensor_normal(so: &TrackedObject, sensor_id: i32) -> [f64; 3] {
	let n = so.sensor_normals.as_deref().unwrap_or_default();
	let j = 3 * sensor_id as usize;
	[n[j], n[j + 1], n[j + 2]]
}

/// sally: a device's config by its serial, and where it came from: `<serial>_config.json` in the working directory,
/// else SteamVR's own copy, each read as the C reads a config file (text mode); one that does not parse, or is another
/// device's (its device_serial_number), is passed over
fn device_config(serial: &str, steamvr: &steamvr::Paths, options: &ConfigOptions) -> Option<(Vec<u8>, String)> {
	let own = std::path::PathBuf::from(format!("{serial}_config.json"));
	for path in std::iter::once(own).chain(steamvr.device_config_path(serial)) {
		let Ok(bytes) = std::fs::read(&path) else { continue };
		let bytes = if cfg!(windows) { textmode::read(&bytes) } else { bytes };
		let Ok(cfg) = DeviceConfig::parse(&bytes, "", options) else { continue };
		// one whose sensors do not read (SV_GENERAL_ERROR: the run would end at the next poll, and, written into the
		// dongle's cache, every run after it at its first) is no device's config either
		if cfg.log.iter().any(|l| matches!(l, DeviceLog::GeneralError(_))) {
			continue;
		}
		let n = cfg.serial_number.iter().position(|&c| c == 0).unwrap_or(cfg.serial_number.len());
		if String::from_utf8_lossy(&cfg.serial_number[..n]).eq_ignore_ascii_case(serial) {
			return Some((bytes, path.display().to_string()));
		}
	}
	None
}

#[cfg(test)]
mod tests {
	use std::io::Write;
	use std::sync::{Arc, Mutex};

	use super::*;

	/// A log target the test can read back
	#[derive(Clone, Default)]
	struct Sink(Arc<Mutex<Vec<u8>>>);

	impl Write for Sink {
		fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
			self.0.lock().unwrap().extend_from_slice(b);
			Ok(b.len())
		}
		fn flush(&mut self) -> std::io::Result<()> {
			Ok(())
		}
	}

	fn runtime() -> Runtime {
		let mut config = Config::default();
		crate::options::register(&mut config);
		Runtime::new(config, Log::new(Some(Box::new(Sink::default())), false))
	}

	/// timestamp_in_s: the wall clock counts from survive.c's first timed call, here the first line the log writes,
	/// plus 1 ms
	#[test]
	fn the_wall_clock_starts_at_the_first_log_line() {
		let mut rt = runtime();
		std::thread::sleep(std::time::Duration::from_millis(30));
		rt.info("the first line");
		let t = rt.run_time();
		assert!((1e-3..0.02).contains(&t), "{t}");
	}

	/// survive_install_run_time_fn: a playback's time is the run clock while it is installed, and frozen at the last
	/// run time read once it is not; the wall clock is left alone
	#[test]
	fn an_installed_clock_is_frozen_when_it_goes() {
		let mut rt = runtime();
		rt.install_run_time_fn(None);
		assert!(matches!(rt.clock, Clock::Wall));
		let time = Arc::new(PlaybackTime::new(2.5));
		rt.install_run_time_fn(Some(time.clone()));
		assert_eq!(rt.run_time(), 2.5);
		time.set(3.75);
		assert_eq!(rt.peek_run_time(), 3.75);
		assert_eq!(rt.run_time(), 3.75);
		rt.install_run_time_fn(None);
		time.set(9.0);
		assert_eq!(rt.run_time(), 3.75);
	}

	/// sally: a config whose sensors do not read (SV_GENERAL_ERROR, which ends the C's run) is no device's: passed over,
	/// and the run goes on; the devices with no config remembered are the latest MAX_MISSES
	#[test]
	fn a_config_that_would_end_the_run_is_passed_over() {
		let d = format!("LHR-DD{:04X}01", std::process::id() & 0xffff);
		let file = format!("{d}_config.json");
		let json = format!(
			"{{\"device_serial_number\":\"{d}\",\"modelPoints\":[[\"0.1\",\"0.2\",\"0.3\"]],\"modelNormals\":[[0,0,1]]}}"
		);
		std::fs::write(&file, &json).unwrap();
		let options = ConfigOptions { imu_acc_bias_units: 0, ignore_config_imu_bias: false, colorize: false };
		let cfg = DeviceConfig::parse(json.as_bytes(), "", &options).unwrap();
		assert!(cfg.log.iter().any(|l| matches!(l, DeviceLog::GeneralError(_))));
		let mut rt = runtime();
		rt.set_replay_time(1.0);
		let obj = rt.create_object("HID", "KN0", "00A4AE2B50-RYB", "00A4AE2B50-RYB").unwrap();
		assert_eq!(rt.identify_object(obj, &d), Identified::Failed);
		assert_eq!(rt.current_error, error::OK);
		assert!(rt.objects[obj].conf.is_none());
		let _ = std::fs::remove_file(&file);
		for i in 0..100 {
			assert_eq!(rt.identify_object(obj, &format!("LHR-NO{i:06}")), Identified::Failed);
		}
		assert_eq!(rt.identify_misses.len(), MAX_MISSES);
		assert_eq!(rt.identify_misses[0].0, "LHR-NO000036");
	}

	/// The device SteamVR says a dongle carries: its own config (`<serial>_config.json` here), the object started over
	/// first when it had another device's (names and dongle kept), a file that is another device's passed over, a
	/// device with no config looked for again only after 5 s, the release
	#[test]
	fn a_dongles_object_takes_the_device_steamvr_names() {
		let tracker = include_str!("../../sally-lighthouse/src/testdata/vive_tracker_3_config.json");
		let knuckles = include_str!("../../sally-lighthouse/src/testdata/knuckles_left_config.json");
		let id = std::process::id() & 0xffff;
		let (a, b, c) = (format!("LHR-AA{id:04X}01"), format!("LHR-BB{id:04X}02"), format!("LHR-CC{id:04X}03"));
		let files = [&a, &b, &c].map(|s| format!("{s}_config.json"));
		std::fs::write(&files[0], tracker.replace("LHR-B0D67561", &a)).unwrap();
		std::fs::write(&files[1], knuckles.replace("LHR-9365E531", &b)).unwrap();
		// named for c, but another device's config
		std::fs::write(&files[2], tracker.replace("LHR-B0D67561", &a)).unwrap();
		let mut rt = runtime();
		rt.set_replay_time(1.0);
		let obj = rt.create_object("HID", "KN0", "00A4AE2B50-RYB", "00A4AE2B50-RYB").unwrap();
		let r = rt.identify_object(obj, &a);
		assert!(matches!(r, Identified::Configured(ref j) if j == tracker.replace("LHR-B0D67561", &a).as_bytes()));
		assert_eq!(rt.objects[obj].serial_number, a);
		assert_eq!(rt.identify_object(obj, &a.to_ascii_lowercase()), Identified::Same);
		// another device on that dongle: started over (nothing of the other device kept), then configured as it
		rt.objects[obj].charge = 42;
		assert!(matches!(rt.identify_object(obj, &b), Identified::Configured(_)));
		let so = &rt.objects[obj];
		assert_eq!(
			(so.serial_number.as_str(), so.codename.as_str(), so.sally_dongle.as_str()),
			(b.as_str(), "KN0", "00A4AE2B50-RYB")
		);
		assert!(so.charge != 42 && so.tracker.is_some());
		// c's file is a's config: not c's
		assert_eq!(rt.identify_object(obj, &c), Identified::Failed);
		// and not looked for again within 5 s (the warning once)
		std::fs::write(&files[2], tracker.replace("LHR-B0D67561", &c)).unwrap();
		rt.set_replay_time(3.0);
		assert_eq!(rt.identify_object(obj, &c), Identified::Failed);
		rt.set_replay_time(6.5);
		assert!(matches!(rt.identify_object(obj, &c), Identified::Configured(_)));
		assert_eq!(rt.objects[obj].serial_number, c);
		// released: no config, the dongle's serial
		rt.release_object(obj);
		let so = &rt.objects[obj];
		assert!(so.conf.is_none() && so.serial_number == "00A4AE2B50-RYB" && so.codename == "KN0");
		for f in &files {
			let _ = std::fs::remove_file(f);
		}
	}
}
