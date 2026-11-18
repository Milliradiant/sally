//! The HID tap feeding the runtime: the tap's `Runtime` (objects created and configured, reports through their
//! parsers, the run clock), each device's parser `Host` (what a report's IMU, light and input become), and the options
//! the tap starts with.

use std::sync::Arc;

use sally_hid::replay::PlaybackTime;
use sally_hid::report::{self, ButtonEntry, Globals, LightcapElement, Parser, BUFFER_LEN};
use sally_hid::tap::{self, Identified, Options};

use crate::config::SC_GET;
use crate::runtime::Runtime;
use crate::ObjectId;

/// DriverRegHIDTap_'s reads of its options as it starts, live or replaying (and vive-test-drop, which the C's parser
/// reads at the first report)
pub fn options(rt: &mut Runtime) -> Options {
	let c = &mut rt.config;
	Options {
		allow_fs_read: c.configi("hidtap-allow-fs-config", SC_GET, 0) != 0,
		vid: c.configi("hidtap-vid", SC_GET, 0x28de) as u16,
		lightcap_mode: c.configi("hidtap-lightcap-mode", SC_GET, -2i32 as u32) as i32,
		config_fetch: c.configi("hidtap-config-fetch", SC_GET, 1) != 0,
		config_delay: c.configf("hidtap-config-delay", SC_GET, 5.0),
		playback_factor: c.configf("playback-factor", SC_GET, 1.0),
		run_time: c.configf("run-time", SC_GET, -1.0),
		test_drop: String::from_utf8_lossy(&c.configs("vive-test-drop", SC_GET, Some(b"")).unwrap_or_default())
			.into_owned(),
		record_strip: c.configi("hidtap-record-strip", SC_GET, 1) != 0,
		past_end: c.configi("hidtap-past-end", SC_GET, 2).clamp(0, 2) as u8,
	}
}

/// A device's parser host: the runtime, seen from one of its objects
struct ObjectHost<'a> {
	rt: &'a mut Runtime,
	obj: ObjectId,
}

impl report::Host for ObjectHost<'_> {
	fn lh_version(&self) -> i32 {
		self.rt.lh_version
	}
	fn last_imu(&self) -> u64 {
		self.rt.objects[self.obj].activations.last_imu
	}
	fn object_subtype(&self) -> i32 {
		self.rt.objects[self.obj].object_subtype
	}
	fn has_config(&self) -> bool {
		self.rt.objects[self.obj].conf.is_some()
	}
	fn map_sensor_id(&self, reported: u8) -> u8 {
		self.rt.objects[self.obj].map_sensor_id(reported)
	}
	fn raw_imu(&mut self, mask: i32, agm: &[f64; 9], timecode: u32, id: i32) {
		self.rt.raw_imu(self.obj, mask, agm, timecode, id);
	}
	fn register_runtime(&mut self, time_received_us: u64) {
		let act = &mut self.rt.objects[self.obj].activations;
		let last_imu = act.last_imu;
		act.register_runtime(last_imu, time_received_us);
	}
	fn sync(&mut self, channel: u8, timecode: u32, ootx: bool, gen: bool) {
		self.rt.sync(self.obj, channel, timecode, ootx, gen);
	}
	fn sweep(&mut self, channel: u8, sensor_id: u8, timecode: u32, half_clock: bool) {
		self.rt.sweep(self.obj, channel, sensor_id as i32, timecode, half_clock);
	}
	fn lightcap(&mut self, le: &LightcapElement) -> bool {
		self.rt.lightcap(self.obj, le)
	}
	fn button(&mut self, entry: &ButtonEntry) {
		self.rt.button(self.obj, entry);
	}
	fn notify_gen2(&mut self, reason: &str) {
		self.rt.notify_gen2(self.obj, reason);
	}
	fn warn(&mut self, text: &str) {
		self.rt.warn(text);
	}
	fn info(&mut self, text: &str) {
		self.rt.info(text);
	}
	fn print(&mut self, text: &str) {
		self.rt.log.print(text);
	}
	fn run_time(&self) -> f64 {
		self.rt.peek_run_time()
	}
	fn colorize(&self, s: &str) -> String {
		self.rt.log.colorize(s)
	}
	fn set_imu_scale_modes(&mut self, gyro_scale_mode: i32, acc_scale_mode: i32) {
		self.rt.set_imu_scale_modes(self.obj, gyro_scale_mode, acc_scale_mode);
	}
	fn battery(&mut self, charge: i8, charging: bool) {
		let so = &mut self.rt.objects[self.obj];
		so.charge = charge;
		so.charging = charging;
	}
}

impl tap::Runtime for Runtime {
	fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId> {
		// the tap's objects come from the driver "HID"; a device's serial is its dongle's
		Runtime::create_object(self, "HID", name, serial, serial)
	}

	fn load_config_file(&mut self, obj: ObjectId, filename: &str) -> bool {
		self.load_config_file(obj, filename)
	}

	fn config(&mut self, obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
		self.config(obj, json)
	}

	fn report(
		&mut self, obj: ObjectId, parser: &mut Parser, globals: &mut Globals, time_us: u64, iface: i32,
		buffer: &[u8; BUFFER_LEN], len: usize,
	) {
		parser.report(&mut ObjectHost { rt: self, obj }, globals, time_us, iface, buffer, len);
	}

	fn setup_write(
		&mut self, obj: ObjectId, parser: &mut Parser, bm_request_type: u8, b_request: u8, w_value: u16, data: &[u8],
	) {
		parser.on_setup_write(&mut ObjectHost { rt: self, obj }, bm_request_type, b_request, w_value, data);
	}

	fn feature_read(&mut self, obj: ObjectId, parser: &Parser, data: &[u8]) {
		parser.feature_read(&mut ObjectHost { rt: self, obj }, data);
	}

	fn install_run_time_fn(&mut self, time: Option<Arc<PlaybackTime>>) {
		Runtime::install_run_time_fn(self, time);
	}

	fn run_time(&self) -> f64 {
		self.peek_run_time()
	}

	fn info(&mut self, text: &str) {
		Runtime::info(self, text);
	}

	fn warn(&mut self, text: &str) {
		Runtime::warn(self, text);
	}

	fn error(&mut self, code: i32, text: &str) {
		Runtime::error(self, code, text);
	}

	fn identify(&mut self, obj: ObjectId, device: &str) -> Identified {
		self.identify_object(obj, device)
	}

	fn release(&mut self, obj: ObjectId) {
		self.release_object(obj);
	}

	fn object_serial(&self, obj: ObjectId) -> Option<String> {
		let so = &self.objects[obj];
		so.conf.as_ref().map(|_| so.serial_number.clone())
	}
}
