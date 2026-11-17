//! The runtime's drivers: libsurvive's plugins as survive_startup loads them (its driver loop) and the hooks they put
//! in the processes' way. In the C each plugin installs its own function as a hook and calls the one it replaced first,
//! so a measurement goes through the default process, then through each driver's hook in the order they were loaded;
//! here the runtime calls them in that order. The order is the C's plugin load order, which is not the same on Linux and
//! on Windows (it decides the order of their lines in the recording, so each platform keeps its own). survive_poll
//! polls them in the same order, and survive_close closes them in it, before the devices go: one at a time, each
//! unloaded as it closes, as the C's hooks only pass a call on once their driver's close has begun. A driver with a
//! thread of its own (the map solve's) has it stopped and joined first, without the context lock that thread may be
//! waiting for, while the drivers after it still take the calls of the threads that go on.

use std::any::Any;
use std::thread::JoinHandle;

use sally_math::Pose;
use sally_track::host::SurviveVelocity;

use crate::diagnosis::DeviceDiagnosis;
use crate::runtime::{error, Runtime};
use crate::ObjectId;

/// A driver's hooks, each called after the default process and the drivers loaded before it, its poll and its close.
/// A hook may call back into the runtime (a nested call does not reach the driver whose hook is running). A driver
/// goes where the runtime goes: to whichever thread holds the context lock
pub trait Driver: Any + Send {
	/// the imupose hook: a device's pose in IMU space, as the tracker reported it
	fn imupose(&mut self, _rt: &mut Runtime, _obj: ObjectId, _timecode: u64, _pose: &Pose) {}
	/// the pose hook: the device's pose at its tracked point (the floor offset off), which the imupose process makes
	fn pose(&mut self, _rt: &mut Runtime, _obj: ObjectId, _timecode: u64, _pose: &Pose) {}
	/// the velocity hook: the velocity of the tracker's report (the IMU point's, world frame), after its pose
	fn velocity(&mut self, _rt: &mut Runtime, _obj: ObjectId, _timecode: u64, _velocity: &SurviveVelocity) {}
	/// the lighthouse pose hook: a station's pose set or moved (the floor offset off)
	fn lighthouse_pose(&mut self, _rt: &mut Runtime, _bsd_idx: usize, _pose: &Pose) {}
	/// the sweep_angle hook: a gen2 hit's angle on its plane
	#[allow(clippy::too_many_arguments)]
	fn sweep_angle(
		&mut self, _rt: &mut Runtime, _obj: ObjectId, _channel: u8, _sensor_id: i32, _timecode: u32, _plane: i8,
		_angle: f64,
	) {
	}
	/// the imu hook: a calibrated IMU sample
	fn imu(&mut self, _rt: &mut Runtime, _obj: ObjectId, _mask: i32, _agm: &[f64; 9], _timecode: u32, _id: i32) {}
	/// the diagnosis hook: a device's verdict on the window the diagnosis just closed
	fn diagnosis(&mut self, _rt: &mut Runtime, _obj: ObjectId, _diagnosis: &DeviceDiagnosis) {}
	/// sally: the object is another device from here on (its dongle carries another one, or none: the tap's assign),
	/// started over: what the driver keeps of it was measured on the other device
	fn device_reset(&mut self, _rt: &mut Runtime, _obj: ObjectId) {}
	/// Whether the driver has a poll: the C's other drivers' polls return 0 and do nothing, so only a driver that says
	/// so is polled
	fn polls(&self) -> bool {
		false
	}
	/// survive_poll's call: nonzero ends the session (survive_poll returns it, and the main loop ends)
	fn poll(&mut self, _rt: &mut Runtime) -> i32 {
		0
	}
	/// survive_close's call, its first half: the driver's hooks pass every call on from here, and a thread of its own
	/// is told to stop and handed back, for survive_close to join without the context lock (the thread may need it to
	/// finish). None for a driver without one
	fn stop(&mut self) -> Option<JoinHandle<()>> {
		None
	}
	/// survive_close's call (after `stop`, and the join): the driver's summary, and the driver gone
	fn close(&mut self, _rt: &mut Runtime) {}
}

/// The drivers sally knows, in the C's plugin load order on this platform: the name the "Loaded drivers" line gives and
/// the option that enables it (survive_startup's driver loop: the name lowercased, '_' made '-'). Pose_Udp and
/// Runtime_Port are the one runtime protocol (driver_pose_udp.c registers it under both names). The tap's two, live and
/// replaying, are the last on both, in their file's order. Runtime_Driver_File is that protocol once more, started by
/// its file's option alone (a bench replay has no port to open)
#[cfg(not(windows))]
pub const ORDER: [(&str, &str); 8] = [
	("Diagnosis", "diagnosis"),
	("Map_Solve", "map-solve"),
	("DiagRec", "diagrec"),
	("Pose_Udp", "pose-udp"),
	("Runtime_Port", "runtime-port"),
	("Runtime_Driver_File", "runtime-driver-file"),
	("HIDTap", "hidtap"),
	("HIDTap_Playback", "hidtap-playback"),
];
#[cfg(windows)]
pub const ORDER: [(&str, &str); 8] = [
	("Diagnosis", "diagnosis"),
	("DiagRec", "diagrec"),
	("Map_Solve", "map-solve"),
	("Pose_Udp", "pose-udp"),
	("Runtime_Port", "runtime-port"),
	("Runtime_Driver_File", "runtime-driver-file"),
	("HIDTap", "hidtap"),
	("HIDTap_Playback", "hidtap-playback"),
];

/// A loaded driver (ctx->drivers): its name, and the driver, out of its slot while one of its hooks runs
pub struct Loaded {
	pub name: &'static str,
	driver: Option<Box<dyn Driver>>,
}

impl Runtime {
	/// survive_add_driver: the driver loaded, after those loaded before it
	pub fn add_driver(&mut self, name: &'static str, driver: Box<dyn Driver>) {
		self.drivers.push(Loaded { name, driver: Some(driver) });
	}

	/// Each loaded driver in load order, given the runtime
	fn each_driver(&mut self, mut f: impl FnMut(&mut dyn Driver, &mut Runtime)) {
		for i in 0..self.drivers.len() {
			if let Some(mut d) = self.drivers[i].driver.take() {
				f(d.as_mut(), self);
				self.drivers[i].driver = Some(d);
			}
		}
	}

	/// The imupose hook: the default process (which raises the pose hook), then the drivers'
	pub fn imupose_hook(&mut self, obj: ObjectId, timecode: u64, pose: &Pose) {
		self.imupose(obj, timecode, pose);
		self.each_driver(|d, rt| d.imupose(rt, obj, timecode, pose));
	}

	/// The pose hook: the default process, then the drivers'
	pub fn pose_hook(&mut self, obj: ObjectId, timecode: u64, pose: &Pose) {
		self.pose(obj, timecode, pose);
		self.each_driver(|d, rt| d.pose(rt, obj, timecode, pose));
	}

	/// The velocity hook: the default process, then the drivers'
	pub fn velocity_hook(&mut self, obj: ObjectId, timecode: u64, velocity: &SurviveVelocity) {
		self.velocity(obj, timecode, velocity);
		self.each_driver(|d, rt| d.velocity(rt, obj, timecode, velocity));
	}

	/// The lighthouse pose hook: the default process, then the drivers'
	pub fn lighthouse_pose_hook(&mut self, bsd_idx: usize, pose: &Pose) {
		self.lighthouse_pose(bsd_idx, pose);
		self.each_driver(|d, rt| d.lighthouse_pose(rt, bsd_idx, pose));
	}

	/// The sweep_angle hook: the default process, then the drivers'
	pub fn sweep_angle_hook(
		&mut self, obj: ObjectId, channel: u8, sensor_id: i32, timecode: u32, plane: i8, angle: f64,
	) {
		self.sweep_angle(obj, channel, sensor_id, timecode, plane, angle);
		self.each_driver(|d, rt| d.sweep_angle(rt, obj, channel, sensor_id, timecode, plane, angle));
	}

	/// The imu hook: the default process, then the drivers'
	pub fn imu_hook(&mut self, obj: ObjectId, mask: i32, agm: &[f64; 9], timecode: u32, id: i32) {
		self.imu(obj, mask, agm, timecode, id);
		self.each_driver(|d, rt| d.imu(rt, obj, mask, agm, timecode, id));
	}

	/// The diagnosis hook (SURVIVE_INVOKE_HOOK_SO(diagnosis, so, &g)): the drivers' (the default process is none)
	pub fn diagnosis_hook(&mut self, obj: ObjectId, diagnosis: &DeviceDiagnosis) {
		self.each_driver(|d, rt| d.diagnosis(rt, obj, diagnosis));
	}

	/// sally: the device reset hook, the drivers' (`Runtime::restart_object`)
	pub fn device_reset_hook(&mut self, obj: ObjectId) {
		self.each_driver(|d, rt| d.device_reset(rt, obj));
	}

	/// survive_poll: the error raised, if one was, ends the session; else the drivers that poll are polled in load
	/// order, and the first that returns nonzero ends it ("Driver reported %d"). 0 while the session goes on. The main
	/// loop calls it and ends on nonzero: survive-cli's every 10 ms of wall time beside the device threads, sally
	/// replay's every 10 ms of the playback's pace on the recording's clock
	pub fn poll(&mut self) -> i32 {
		if self.current_error != error::OK {
			return self.current_error;
		}
		for i in 0..self.drivers.len() {
			let Some(mut d) = self.drivers[i].driver.take() else { continue };
			let r = if d.polls() { d.poll(self) } else { 0 };
			self.drivers[i].driver = Some(d);
			if r != 0 {
				self.warn(&format!("Driver reported {r}"));
				return r;
			}
		}
		0
	}

	/// `f` given the loaded driver of that name as the type it is, and the runtime: what a driver's own thread reaches
	/// with the context lock. None when no such driver is loaded, or while it is out of its slot (one of its hooks is
	/// running, or it is closing)
	pub fn with_driver<D: Driver, R>(&mut self, name: &str, f: impl FnOnce(&mut D, &mut Runtime) -> R) -> Option<R> {
		let i = self.drivers.iter().position(|l| l.name == name)?;
		let mut d = self.drivers[i].driver.take()?;
		let r = (d.as_mut() as &mut dyn Any).downcast_mut::<D>().map(|d| f(d, self));
		self.drivers[i].driver = Some(d);
		r
	}

	/// survive_close's driver loop, the first half of the next driver's close (`Driver::stop`): its thread, if it has
	/// one, to be joined without the context lock. The driver stays loaded, so that thread still reaches it
	pub fn stop_first_driver(&mut self) -> Option<JoinHandle<()>> {
		self.drivers.first_mut()?.driver.as_mut()?.stop()
	}

	/// survive_close's driver loop, the rest of the next driver's close: unloaded, then closed (`Driver::close`), the
	/// drivers after it still loaded. False when none is left
	pub fn close_first_driver(&mut self) -> bool {
		if self.drivers.is_empty() {
			return false;
		}
		let mut l = self.drivers.remove(0);
		if let Some(d) = l.driver.as_mut() {
			d.close(self);
		}
		true
	}

	/// survive_close's driver loop on one thread: each driver stopped (its thread joined; on one thread there is no
	/// context lock to let go of) and closed, in load order
	pub fn close_drivers(&mut self) {
		loop {
			if let Some(thread) = self.stop_first_driver() {
				let _ = thread.join();
			}
			if !self.close_first_driver() {
				break;
			}
		}
	}
}
