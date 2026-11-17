//! The recording (survive_recording.c): one text line per event, each started with the run time ("%0.6f "), in the
//! C's formats to the byte (each line below is written with the C's own format string). The C writes it through
//! zlib's gzFile (a binary stream: its "\r\n" and "\n" endings go out as they are), or also to stdout with
//! --record-stdout.

use std::fs::File;
use std::io::{BufWriter, Write};

use sally_math::printf::{sprintf, sprintf_into, Arg};
use sally_math::Pose;
use sally_track::host::SurviveVelocity;

use crate::config::{Binding, Config, VarType};
use crate::runtime::Runtime;

/// SurviveRecordingData's bound options (its STRUCT_CONFIG_SECTION): which kinds of line the recording writes. Bound
/// while the recording is open (SurviveRecordingData_attach_config, survive_destroy_recording's detach), so a live set
/// reaches the recording being written
#[derive(Clone, Debug)]
pub struct RecordingOptions {
	/// record-rawlight: LH1 light pulses (C lines)
	pub raw_light: Binding,
	/// record-imu: raw IMU samples (i lines)
	pub imu: Binding,
	/// record-cal-imu: calibrated IMU samples (I lines)
	pub cal_imu: Binding,
	/// record-angle: syncs, sweeps and angles (Y, W, B lines)
	pub angle: Binding,
}

impl Default for RecordingOptions {
	/// The four variables, not bound yet
	fn default() -> Self {
		RecordingOptions {
			raw_light: Binding::of(VarType::Bool),
			imu: Binding::of(VarType::Bool),
			cal_imu: Binding::of(VarType::Bool),
			angle: Binding::of(VarType::Bool),
		}
	}
}

impl RecordingOptions {
	/// The section's items, in the C's order
	pub fn items(&self) -> [(&'static str, &Binding); 4] {
		[
			("record-rawlight", &self.raw_light),
			("record-imu", &self.imu),
			("record-cal-imu", &self.cal_imu),
			("record-angle", &self.angle),
		]
	}

	/// SurviveRecordingData_attach_config: each variable bound, taking its option's value
	pub fn attach(&self, config: &mut Config) {
		for (tag, var) in self.items() {
			config.attach(tag, var);
		}
	}
}

/// SurviveRecordingData: where the lines go and which kinds are written
pub struct Recording {
	file: Option<BufWriter<File>>,
	/// record-stdout: every line to stdout too
	stdout: bool,
	pub options: RecordingOptions,
	/// the line being written: one buffer, kept from line to line
	line: String,
}

/// FLT_PRINTF: "%0.6f " (the hex float form, SURVIVE_HEX_FLOATS, is never built), for the formats put together from it
macro_rules! flt {
	() => {
		"%0.6f "
	};
}
const FLT_PRINTF: &str = flt!();
/// The i and I lines: the head, FLT_PRINTF six times, a space, three more, "%d\r\n"
const RAW_IMU_FMT: &str =
	concat!("%s i %d %u ", flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), " ", flt!(), flt!(), flt!(), "%d\r\n");
const IMU_FMT: &str =
	concat!("%s I %d %u ", flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), " ", flt!(), flt!(), flt!(), "%d\r\n");
/// LH_POSE: "%d LH_POSE ", FLT_PRINTF seven times, " %u\r\n"
const LH_POSE_FMT: &str = concat!("%d LH_POSE ", flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), " %u\r\n");
/// POSE: FLT_PRINTF seven times, then " %" PRIu64
const POSE_FMT: &str = concat!("%s POSE ", flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), " %llu\r\n");
/// VELOCITY: FLT_PRINTF six times, then " %" PRIu64
const VELOCITY_FMT: &str = concat!("%s VELOCITY ", flt!(), flt!(), flt!(), flt!(), flt!(), flt!(), " %llu\r\n");

impl Recording {
	/// survive_install_recording's gzopen of the file ("w6F" for a .gz name, else "wT", uncompressed; a .gz name is
	/// written uncompressed here: no recording sally makes is compressed), with the stdout copy, writing the kinds of
	/// line `options` say
	pub fn open(path: Option<&str>, stdout: bool, options: RecordingOptions) -> std::io::Result<Recording> {
		let file = match path {
			Some(p) if !p.is_empty() => Some(BufWriter::with_capacity(1 << 16, File::create(p)?)),
			_ => None,
		};
		Ok(Recording { file, stdout, options, line: String::new() })
	}

	fn raw(&mut self, bytes: &[u8]) {
		if let Some(f) = self.file.as_mut() {
			let _ = f.write_all(bytes);
		}
		if self.stdout {
			let _ = std::io::stdout().write_all(bytes);
		}
	}

	/// survive_recording_write_to_output: the run time, then the line formatted (both into the recording's line buffer)
	pub fn write(&mut self, run_time: f64, fmt: &str, args: &[Arg]) {
		let mut line = std::mem::take(&mut self.line);
		line.clear();
		sprintf_into(&mut line, FLT_PRINTF, &[Arg::F(run_time)]);
		sprintf_into(&mut line, fmt, args);
		self.raw(line.as_bytes());
		self.line = line;
	}

	/// The run time, then a line already formatted ("%s")
	pub fn line(&mut self, run_time: f64, body: &str) {
		self.write(run_time, "%s", &[Arg::S(body)]);
	}

	/// OPTION: a registered option and its value as text, as the recording starts (survive_record_config)
	pub fn option(&mut self, run_time: f64, name: &[u8], type_char: u8, value: &[u8]) {
		let mut s = sprintf(FLT_PRINTF, &[Arg::F(run_time)]).into_bytes();
		s.extend_from_slice(b"OPTION ");
		s.extend_from_slice(name);
		s.push(b' ');
		s.push(type_char);
		s.push(b' ');
		s.extend_from_slice(value);
		s.push(b'\n');
		self.raw(&s);
	}

	/// INFO LOG: every info and warning (survive_recording_info_process)
	pub fn info(&mut self, run_time: f64, msg: &str) {
		self.write(run_time, "INFO LOG %s\r\n", &[Arg::S(msg)]);
	}

	/// i: a raw IMU sample (survive_recording_raw_imu_process)
	pub fn raw_imu(&mut self, run_time: f64, codename: &str, mask: i32, agm: &[f64; 9], timecode: u32, id: i32) {
		if self.options.imu.bool() {
			self.imu_line(run_time, RAW_IMU_FMT, codename, mask, agm, timecode, id);
		}
	}

	/// I: a calibrated IMU sample (survive_recording_imu_process)
	pub fn imu(&mut self, run_time: f64, codename: &str, mask: i32, agm: &[f64; 9], timecode: u32, id: i32) {
		if self.options.cal_imu.bool() {
			self.imu_line(run_time, IMU_FMT, codename, mask, agm, timecode, id);
		}
	}

	/// The i and I lines (RAW_IMU_FMT, IMU_FMT)
	#[allow(clippy::too_many_arguments)]
	fn imu_line(
		&mut self, run_time: f64, fmt: &str, codename: &str, mask: i32, agm: &[f64; 9], timecode: u32, id: i32,
	) {
		let mut args = [Arg::F(0.0); 13];
		args[..3].copy_from_slice(&[Arg::S(codename), Arg::I(mask as i64), Arg::U(timecode as u64)]);
		for (a, &v) in args[3..12].iter_mut().zip(agm) {
			*a = Arg::F(v);
		}
		args[12] = Arg::I(id as i64);
		self.write(run_time, fmt, &args);
	}

	/// Y: a station's sync (SYNC_PRINTF)
	pub fn sync(&mut self, run_time: f64, codename: &str, channel: u8, timecode: u32, ootx: bool, gen: bool) {
		if self.options.angle.bool() {
			let args = [
				Arg::S(codename),
				Arg::U(channel as u64),
				Arg::U(timecode as u64),
				Arg::U(ootx as u64),
				Arg::U(gen as u64),
			];
			self.write(run_time, "%s Y %u %u %u %u\n", &args);
		}
	}

	/// W: a sensor's sweep hit (SWEEP_PRINTF)
	pub fn sweep(&mut self, run_time: f64, codename: &str, channel: u8, sensor_id: i32, timecode: u32, flag: bool) {
		if self.options.angle.bool() {
			let args = [
				Arg::S(codename),
				Arg::U(channel as u64),
				Arg::I(sensor_id as i64),
				Arg::U(timecode as u64),
				Arg::U(flag as u64),
			];
			self.write(run_time, "%s W %u %d %u %u\n", &args);
		}
	}

	/// B: a hit's angle on its plane (SWEEP_ANGLE_PRINTF: the sensor, an int, printed with %u)
	#[allow(clippy::too_many_arguments)]
	pub fn sweep_angle(
		&mut self, run_time: f64, codename: &str, channel: u8, sensor_id: i32, timecode: u32, plane: i8, angle: f64,
	) {
		if self.options.angle.bool() {
			let args = [
				Arg::S(codename),
				Arg::U(channel as u64),
				Arg::U(sensor_id as u32 as u64),
				Arg::U(timecode as u64),
				Arg::I(plane as i64),
				Arg::F(angle),
			];
			self.write(run_time, "%s B %u %u %u %d %+le\n", &args);
		}
	}

	/// CONFIG: a device's config, its line breaks made spaces (survive_recording_config_process)
	pub fn config(&mut self, run_time: f64, codename: &str, conf: &[u8]) {
		let body: Vec<u8> = conf.iter().map(|&c| if c == b'\n' || c == b'\r' { b' ' } else { c }).collect();
		self.write(run_time, "%s CONFIG ", &[Arg::S(codename)]);
		self.raw(&body);
		self.raw(b"\r\n");
	}

	/// LH_UP: a station's up direction, at its OOTX (survive_default_ootx_received_process: "LH_UP %d " Point3_format,
	/// the channel a uint8_t)
	pub fn lh_up(&mut self, run_time: f64, mode: u8, accel: &[f64; 3]) {
		let args = [Arg::I(mode as i64), Arg::F(accel[0]), Arg::F(accel[1]), Arg::F(accel[2])];
		self.write(run_time, "LH_UP %d %+le   %+le   %+le\n", &args);
	}

	/// LH_POSE: a station's pose (survive_recording_lighthouse_process: the channel through an int8_t, so a free slot's
	/// 0xFF is -1)
	pub fn lighthouse_pose(&mut self, run_time: f64, mode: i8, pose: &Pose, base_station_id: u32) {
		let mut args = [Arg::F(0.0); 9];
		args[0] = Arg::I(mode as i64);
		for (a, &v) in args[1..8].iter_mut().zip(pose.pos.iter().chain(&pose.rot)) {
			*a = Arg::F(v);
		}
		args[8] = Arg::U(base_station_id as u64);
		self.write(run_time, LH_POSE_FMT, &args);
	}

	/// POSE: a device's pose at its tracked point, and the device time of the state it came from
	/// (survive_recording_raw_pose_process: FLT_PRINTF seven times, then " %" PRIu64)
	pub fn pose(&mut self, run_time: f64, codename: &str, pose: &Pose, timecode: u64) {
		let mut args = [Arg::F(0.0); 9];
		args[0] = Arg::S(codename);
		for (a, &v) in args[1..8].iter_mut().zip(pose.pos.iter().chain(&pose.rot)) {
			*a = Arg::F(v);
		}
		args[8] = Arg::U(timecode);
		self.write(run_time, POSE_FMT, &args);
	}

	/// VELOCITY: a device's velocity and angular velocity (survive_recording_velocity_process)
	pub fn velocity(&mut self, run_time: f64, codename: &str, vel: &SurviveVelocity, timecode: u64) {
		let mut args = [Arg::F(0.0); 8];
		args[0] = Arg::S(codename);
		for (a, &v) in args[1..7].iter_mut().zip(vel.pos.iter().chain(&vel.axis_angle_rot)) {
			*a = Arg::F(v);
		}
		args[7] = Arg::U(timecode);
		self.write(run_time, VELOCITY_FMT, &args);
	}

	/// FAC: a gen2 hit with its residual against the device's pose and its sensor's incidence cosine (record-facing)
	#[allow(clippy::too_many_arguments)]
	pub fn facing(
		&mut self, run_time: f64, codename: &str, bsd_idx: i8, sensor_id: i32, plane: i8, angle: f64, residual: f64,
		incidence: f64,
	) {
		let args = [
			Arg::S(codename),
			Arg::I(bsd_idx as i64),
			Arg::I(sensor_id as i64),
			Arg::I(plane as i64),
			Arg::F(angle),
			Arg::F(residual),
			Arg::F(incidence),
		];
		self.write(run_time, "%s FAC %d %d %d %.6f %.6f %.4f\r\n", &args);
	}

	/// BUTTON: an input event (survive_recording_button_process)
	pub fn button(&mut self, run_time: f64, codename: &str, event_type: i32, button_id: i32) {
		let args = [Arg::S(codename), Arg::U(event_type as u32 as u64), Arg::U(button_id as u32 as u64)];
		self.write(run_time, "%s BUTTON %u %u\r\n", &args);
	}

	/// IMU_SCALES: the scale modes a feature read reported (survive_recording_imu_scales)
	pub fn imu_scales(&mut self, run_time: f64, codename: &str, gyro_scale_mode: i32, acc_scale_mode: i32) {
		let args = [Arg::S(codename), Arg::I(gyro_scale_mode as i64), Arg::I(acc_scale_mode as i64)];
		self.write(run_time, "%s IMU_SCALES %d %d\r\n", &args);
	}

	/// C: an LH1 light pulse (survive_recording_lightcap)
	pub fn lightcap(&mut self, run_time: f64, codename: &str, sensor_id: u8, timestamp: u32, length: u16) {
		if self.options.raw_light.bool() {
			let args = [Arg::S(codename), Arg::I(sensor_id as i64), Arg::U(timestamp as u64), Arg::U(length as u64)];
			self.write(run_time, "%s C %d %u %u\r\n", &args);
		}
	}

	/// DISCONNECT: a device gone, at shutdown (survive_recording_disconnect_process)
	pub fn disconnect(&mut self, run_time: f64, codename: &str) {
		self.write(run_time, "%s DISCONNECT\r\n", &[Arg::S(codename)]);
	}

	/// survive_destroy_recording: the file written out and closed
	pub fn close(&mut self) {
		if let Some(f) = self.file.as_mut() {
			let _ = f.flush();
		}
		self.file = None;
		if self.stdout {
			let _ = std::io::stdout().flush();
		}
	}
}

impl Runtime {
	/// survive_recording_start (sally): a text recording started while running (the runtime protocol's, and the
	/// tracker's harness). The header a recording started with --record has by then is written first: the start's
	/// line (with the default log hook, an INFO LOG line in the new recording), every OPTION, each listed device's
	/// CONFIG and each placed station's LH_POSE, so that the file replays like one started at startup. 0 when started;
	/// -1 when a recording is being written already, -2 for no path or a USB capture (.pcap: driver_usbmon makes those
	/// at startup only), -3 when the file does not open. Never on stdout (record-stdout is startup's)
	pub fn recording_start(&mut self, path: &str) -> i32 {
		if self.recording.is_some() {
			return -1;
		}
		if path.is_empty() || path.contains(".pcap") {
			return -2;
		}
		let use_compression = path.len() > 3 && path.ends_with(".gz");
		let options = RecordingOptions::default();
		let Ok(r) = Recording::open(Some(path), false, options.clone()) else {
			return -3;
		};
		options.attach(&mut self.config);
		self.recording = Some(r);
		self.info(&format!("Recording to '{path}' Compression: {} (started live)", use_compression as i32));

		self.record_options();
		// each device of ctx->objs that has its config (so->conf, so->conf_cnt > 0)
		for obj in 0..self.objects.len() {
			let so = &self.objects[obj];
			if !self.is_listed(obj) || so.conf.as_ref().is_none_or(|c| c.is_empty()) {
				continue;
			}
			let (codename, conf) = (so.codename.clone(), so.conf.clone().unwrap_or_default());
			self.record(|r, t| r.config(t, &codename, &conf));
		}
		for i in 0..self.active_lighthouses.max(0) as usize {
			if self.bsd[i].position_set {
				let pose = self.bsd[i].pose;
				self.record_lighthouse_pose(i, &pose);
			}
		}
		0
	}

	/// survive_recording_stop (sally): the recording written out and closed (survive_destroy_recording), then "Recording
	/// stopped" logged, which is not in it. 0 when stopped, -1 when nothing was being recorded
	pub fn recording_stop(&mut self) -> i32 {
		if self.recording.is_none() {
			return -1;
		}
		self.destroy_recording();
		self.info("Recording stopped");
		0
	}

	/// survive_destroy_recording: the recording's options unbound, then the file written out and closed
	pub fn destroy_recording(&mut self) {
		let Some(options) = self.recording.as_ref().map(|r| r.options.clone()) else { return };
		for (tag, var) in options.items() {
			self.detach_config(tag, var);
		}
		if let Some(mut r) = self.recording.take() {
			r.close();
		}
	}

	/// survive_config_iterate(ctx, survive_record_config, ..): an OPTION line for each registered option, in their
	/// order, with its value as survive_config_as_str gives it ("OPTION %s %c %s\n": the name, the type char, the text
	/// cut to 127 bytes)
	pub fn record_options(&mut self) {
		let options: Vec<(Vec<u8>, u8)> =
			self.config.options().iter().map(|o| (o.name().to_vec(), o.type_char())).collect();
		for (name, type_char) in options {
			let value = self.config.as_str(&name, Some(b""), 128);
			self.record(|r, t| r.option(t, &name, type_char, &value));
		}
	}
}
