//! The tracker on this runtime: each device's `sally_track::Tracker`, made with the device and freed with it, the host
//! it reads the device and the stations through, its options (a live set reaches each tracker's copy), and its report's
//! way out: the imupose and velocity hooks, and the default processes behind them (so->OutPoseIMU, then the pose at the
//! tracked point: POSE; the velocity: VELOCITY).

use sally_lighthouse::reproject::REPROJECT_AXIS_FN_GEN2;
use sally_lighthouse::{BaseStationCal, NUM_GEN2_LIGHTHOUSES};
use sally_math::{apply_pose_to_point, apply_pose_to_pose, invert_pose, Pose};
use sally_track::host::{Host, SurviveVelocity, SURVIVE_DIAG_STATE_NONE, SURVIVE_DIAG_STATE_REACQUIRING};
use sally_track::options::OPTIONS as TRACKER_OPTIONS;
use sally_track::text::Name;
use sally_track::tracker::{Diag, Handover, Tracker};

use crate::object::TrackedObject;
use crate::runtime::Runtime;
use crate::settings::tracker_settings;
use crate::ObjectId;

/// The runtime as one device's tracker sees it while it runs: the device, the context, the text
pub struct TrackerHost<'a> {
	rt: &'a mut Runtime,
	obj: ObjectId,
}

impl TrackerHost<'_> {
	fn so(&self) -> &TrackedObject {
		&self.rt.objects[self.obj]
	}
}

/// ctx->bsd has NUM_GEN2_LIGHTHOUSES entries: the tracker reads them only for stations it checked (the port stops
/// where the C would have read out of bounds)
fn lh_index(lh: i32) -> usize {
	assert!(lh >= 0 && (lh as usize) < NUM_GEN2_LIGHTHOUSES, "track: ctx->bsd[{lh}] read out of bounds");
	lh as usize
}

/// &array[3 * i], for a sensor of the device (an index that is not a sensor, or no array, stops)
fn sensor_vec(v: Option<&Vec<f64>>, sensor_ct: i8, i: i32, what: &str) -> [f64; 3] {
	let ok = i >= 0 && i < sensor_ct as i32;
	match v {
		Some(v) if ok && v.len() >= 3 * (i as usize + 1) => {
			let j = 3 * i as usize;
			[v[j], v[j + 1], v[j + 2]]
		}
		_ => panic!("track: so->{what}[{i}] read out of bounds"),
	}
}

impl Host for TrackerHost<'_> {
	fn sensor_ct(&self) -> i32 {
		self.so().sensor_ct as i32
	}
	fn sensor_locations_set(&self) -> bool {
		self.so().sensor_locations.is_some()
	}
	fn sensor_normals_set(&self) -> bool {
		self.so().sensor_normals.is_some()
	}
	fn sensor_location(&self, i: i32) -> [f64; 3] {
		let so = self.so();
		sensor_vec(so.sensor_locations.as_ref(), so.sensor_ct, i, "sensor_locations")
	}
	fn sensor_normal(&self, i: i32) -> [f64; 3] {
		let so = self.so();
		sensor_vec(so.sensor_normals.as_ref(), so.sensor_ct, i, "sensor_normals")
	}
	fn object_type(&self) -> i32 {
		self.so().object_type
	}
	fn timebase_hz(&self) -> i32 {
		self.so().timebase_hz
	}
	fn raw_acc_scale(&self) -> f64 {
		self.so().raw_acc_scale
	}
	fn raw_gyro_scale(&self) -> f64 {
		self.so().raw_gyro_scale
	}
	fn acc_scale(&self, i: usize) -> f64 {
		assert!(i < 3, "track: so->acc_scale[{i}] read out of bounds");
		self.so().acc_scale[i]
	}
	fn acc_bias(&self, i: usize) -> f64 {
		assert!(i < 3, "track: so->acc_bias[{i}] read out of bounds");
		self.so().acc_bias[i]
	}
	fn codename(&self) -> Name {
		Name::new(&self.so().codename)
	}
	fn active_lighthouses(&self) -> i32 {
		self.rt.active_lighthouses
	}
	fn lh_version(&self) -> i32 {
		self.rt.lh_version
	}
	fn bsd_mode(&self, lh: i32) -> i32 {
		self.rt.bsd[lh_index(lh)].mode as i32
	}
	fn bsd_position_set(&self, lh: i32) -> bool {
		self.rt.bsd[lh_index(lh)].position_set
	}
	fn bsd_fcal(&self, lh: i32) -> [BaseStationCal; 2] {
		self.rt.bsd[lh_index(lh)].fcal
	}
	fn lighthouse_position(&mut self, lh: i32) -> Pose {
		self.rt.get_lighthouse_position(lh_index(lh))
	}
	fn recording(&self) -> bool {
		self.rt.recording.is_some()
	}
	fn log(&mut self, warn: bool, text: &str) {
		if warn {
			self.rt.warn(text);
		} else {
			self.rt.info(text);
		}
	}
	fn record_line(&mut self, line: &str) {
		self.rt.record_line(line);
	}
}

impl Runtime {
	/// survive_c2_create: a device's tracker, with the tracker's options as they are now
	pub(crate) fn new_tracker(&self) -> Box<Tracker> {
		Tracker::create(self.settings.tracker)
	}

	/// survive_c2_create(so): the device given a new tracker (survive_create_device makes one for every device; this is
	/// for a device made otherwise, as the tracker's harness makes its own). A tracker it had is dropped without its
	/// summary, as the C's would be overwritten
	pub fn create_tracker(&mut self, obj: ObjectId) {
		self.objects[obj].tracker = Some(self.new_tracker());
	}

	/// A tracker option set live (set_live, after the store took the value): each device's tracker given the options
	/// as the store has them now, as the C's bound variables are written by the set; the runtime's copy too, which a
	/// tracker made later starts with. Returns the number of trackers bound to the tag (one per device that has a
	/// tracker; 0 for a tag that is not a tracker option, where nothing changes).
	///
	/// Where this is not the C's binding: the C writes the variables bound to the entry the set wrote, each tracker's
	/// bound by survive_c2_create's attach to the entry its lookup found then (a temporary one made for an unset tag);
	/// here all of a tracker's options are read again with that lookup. The two give the same values whenever the
	/// entry a lookup finds now is the one the variables were bound to, which holds as long as nothing but a live set
	/// writes a tracker option after the trackers are made (nothing in this runtime does). One that did, for a tag
	/// bound to the file's entry, would write a temporary entry that shadows it: this re-read would take that value,
	/// the C's variables would keep the file's
	pub(crate) fn tracker_option_set(&mut self, tag: &str) -> i32 {
		if !TRACKER_OPTIONS.iter().any(|o| o.tag == tag) {
			return 0;
		}
		self.settings.tracker = tracker_settings(&mut self.config);
		let mut bound = 0;
		for so in self.objects.iter_mut() {
			if let Some(t) = so.tracker.as_mut() {
				t.set_settings(self.settings.tracker);
				bound += 1;
			}
		}
		bound
	}

	/// The trackers bound to the tag: one per device that has one, for a tracker option
	pub(crate) fn tracker_bound(&self, tag: &str) -> i32 {
		if !TRACKER_OPTIONS.iter().any(|o| o.tag == tag) {
			return 0;
		}
		self.objects.iter().filter(|so| so.tracker.is_some()).count() as i32
	}

	/// A call into a device's tracker, with its host. The tracker is out of the device for the call (its host reads
	/// the device, the stations and the text, never a tracker); None when the device has none
	pub(crate) fn with_tracker<R>(
		&mut self, obj: ObjectId, f: impl FnOnce(&mut Tracker, &mut TrackerHost<'_>) -> R,
	) -> Option<R> {
		let mut tracker = self.objects[obj].tracker.take()?;
		let r = f(&mut tracker, &mut TrackerHost { rt: self, obj });
		self.objects[obj].tracker = Some(tracker);
		Some(r)
	}

	/// survive_c2_diag_state: the device's tracking state (NONE, with -1 for the age and the sd, when it has no
	/// tracker)
	pub fn tracker_diag(&mut self, obj: ObjectId) -> Diag {
		self.with_tracker(obj, |t, h| t.diag_state(h)).unwrap_or(Diag {
			state: SURVIVE_DIAG_STATE_NONE,
			light_age: -1.0,
			pos_sd: -1.0,
			lh_mask: 0,
			res_mrad: -1.0,
		})
	}

	/// plane_pose_held: the tracker holds a pose for the device (it is not re-acquiring it)
	pub(crate) fn plane_pose_held(&mut self, obj: ObjectId) -> bool {
		let st = self.tracker_diag(obj).state;
		st != SURVIVE_DIAG_STATE_NONE && st != SURVIVE_DIAG_STATE_REACQUIRING
	}

	/// rf_pose_fresh: the device's pose is fresh enough for the reflection gate (the tracker's light age under
	/// `max_age`)
	pub(crate) fn rf_pose_fresh(&mut self, obj: ObjectId, max_age: f64) -> bool {
		let d = self.tracker_diag(obj);
		d.state != SURVIVE_DIAG_STATE_NONE && d.light_age >= 0.0 && d.light_age < max_age
	}

	/// survive_c2_integrate_imu: the sample to the tracker, and its report, when there is one, through the imupose
	/// and the velocity hooks, after the tracker has returned
	pub fn tracker_imu(&mut self, obj: ObjectId, timecode: u64, accel: &[f64; 3], gyro: &[f64; 3]) {
		let report = self.with_tracker(obj, |t, h| t.integrate_imu(h, timecode, accel, gyro)).flatten();
		if let Some((pose, vel)) = report {
			// the imupose hook sets so->OutPoseIMU: the device's pose for the reflection gate and the map solve
			self.imupose_hook(obj, timecode, &pose);
			// the velocity hook carries the IMU point's velocity: the state's
			self.velocity_hook(obj, timecode, &vel);
		}
		// sally: what the trackers learned, written every minute
		self.learned_tick();
	}

	/// survive_c2_handover: the tracker's hand-over look-ahead (None without a seeded tracker)
	pub fn tracker_handover(&self, obj: ObjectId) -> Option<Handover> {
		self.objects[obj].tracker.as_ref().and_then(|t| t.handover())
	}

	/// survive_c2_handover_alpha: the angular acceleration the look-ahead carries (None without a seeded tracker)
	pub fn tracker_handover_alpha(&self, obj: ObjectId) -> Option<[f64; 3]> {
		self.objects[obj].tracker.as_ref().and_then(|t| t.handover_alpha())
	}

	/// survive_c2_integrate_hit
	pub fn tracker_hit(&mut self, obj: ObjectId, timecode: u64, lh: i32, plane: i32, sensor_id: i32, angle: f64) {
		self.with_tracker(obj, |t, h| t.integrate_hit(h, timecode, lh, plane, sensor_id, angle));
	}

	/// survive_c2_free: the tracker's summary, and the tracker gone
	pub fn free_tracker(&mut self, obj: ObjectId) {
		self.with_tracker(obj, |t, h| t.print_stats(h));
		self.objects[obj].tracker = None;
	}

	/// survive_default_imupose_process: the pose in IMU space kept (so->OutPoseIMU), and the pose at the tracked point
	/// (report-in-imu: the IMU's), moved down by the floor offset, to the pose hook
	pub fn imupose(&mut self, obj: ObjectId, timecode: u64, imu2world: &Pose) {
		let so = &mut self.objects[obj];
		so.out_pose_imu = *imu2world;
		let mut head2world =
			if !self.settings.report_in_imu { apply_pose_to_pose(imu2world, &so.head2imu) } else { *imu2world };
		head2world.pos[2] -= self.floor_offset;
		self.pose_hook(obj, timecode, &head2world);
	}

	/// survive_default_pose_process: kept (so->OutPose) and recorded (POSE)
	pub fn pose(&mut self, obj: ObjectId, timecode: u64, pose: &Pose) {
		let so = &mut self.objects[obj];
		so.out_pose = *pose;
		so.out_pose_timecode = timecode;
		self.record_object(obj, |r, t, codename| r.pose(t, codename, pose, timecode));
	}

	/// survive_default_velocity_process: recorded (VELOCITY), then kept (so->velocity)
	pub fn velocity(&mut self, obj: ObjectId, timecode: u64, velocity: &SurviveVelocity) {
		self.record_object(obj, |r, t, codename| r.velocity(t, codename, velocity, timecode));
		let so = &mut self.objects[obj];
		so.velocity = *velocity;
		so.velocity_timecode = timecode;
	}

	/// survive_reproject_gen2: where a point in the world appears to a station (both planes), from the station's pose
	/// as survive_get_lighthouse_position gives it (a station being moved slowly moves on, with its LH_POSE line)
	pub(crate) fn reproject_gen2(&mut self, lh: usize, pt_in_world: &[f64; 3]) -> [f64; 2] {
		let world2lh = invert_pose(&self.get_lighthouse_position(lh));
		let pt_in_lh = apply_pose_to_point(&world2lh, pt_in_world);
		let fcal = &self.bsd[lh].fcal;
		[REPROJECT_AXIS_FN_GEN2[0](fcal, &pt_in_lh), REPROJECT_AXIS_FN_GEN2[1](fcal, &pt_in_lh)]
	}
}
