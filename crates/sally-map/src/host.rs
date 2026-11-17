//! What the map solve reads from the runtime and does in it (the plugin's use of SurviveContext and SurviveObject in
//! driver_map_solve.c), one method per thing the C reads or does, called at the moments the C does it.
//!
//! The runtime that owns a `MapSolve` implements this and passes itself to each of its methods. A runtime that keeps
//! the `MapSolve` in one of its own fields takes it out for the call (`if let Some(mut ms) = self.map_solve.take() {
//! ms.imupose(self, ..); self.map_solve = Some(ms) }`): the map solve never calls back into itself through the host.

use sally_lighthouse::BaseStationCal;
use sally_math::Pose;

/// A tracked device, by the id the host gives it (the C keys its devices by their SurviveObject)
pub type DeviceId = usize;

/// SurviveVelocity (LinmathAxisAnglePose): Pos[3], AxisAngleRot[3]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Velocity {
	pub pos: [f64; 3],
	pub axis_angle_rot: [f64; 3],
}

/// The runtime as the map solve sees it. Stations are by index (`lh` < NUM_GEN2_LIGHTHOUSES, checked by the caller),
/// devices by the id the host gave. The reads take `&self` and change nothing the map solve can see; the methods that
/// take `&mut self` are the C's calls into the runtime and its writes to it.
pub trait Host {
	// ---- the clock

	fn run_time(&mut self) -> f64;
	/// OGRelativeTime(): the wall clock, only for the time a solve took (in its log line)
	fn relative_time(&mut self) -> f64;

	// ---- the stations (ctx->bsd)

	fn active_lighthouses(&self) -> i32;
	/// survive_get_bsd_idx(ctx, channel): the station index of a channel (-1 none), the station made if it is new
	fn get_bsd_idx(&mut self, channel: u8) -> i8;
	/// ctx->bsd[lh].Pose
	fn bsd_pose(&self, lh: usize) -> Pose;
	/// ctx->bsd[lh].PositionSet
	fn bsd_position_set(&self, lh: usize) -> bool;
	/// ctx->bsd[lh].OOTXSet
	fn bsd_ootx_set(&self, lh: usize) -> bool;
	/// ctx->bsd[lh].BaseStationID
	fn bsd_base_station_id(&self, lh: usize) -> u32;
	/// ctx->bsd[lh].mode: the station's channel
	fn bsd_mode(&self, lh: usize) -> u8;
	/// ctx->bsd[lh].fcal
	fn bsd_fcal(&self, lh: usize) -> [BaseStationCal; 2];
	/// ctx->bsd[lh].fcal = *fcal (station-fcal-file's corrections)
	fn set_bsd_fcal(&mut self, lh: usize, fcal: &[BaseStationCal; 2]);
	/// ctx->bsd[lh].sally_moved: 0 fine, 1 disagrees with the map, 2 relocated
	fn bsd_sally_moved(&self, lh: usize) -> i8;
	/// ctx->bsd[lh].sally_moved = moved
	fn set_bsd_sally_moved(&mut self, lh: usize, moved: i8);
	/// ctx->bsd[lh].sally_moved_time = t
	fn set_bsd_sally_moved_time(&mut self, lh: usize, t: f64);
	/// SURVIVE_INVOKE_HOOK(raw_lighthouse_pose, ctx, lh, pose): the pose the tracker is to use for station lh (the
	/// runtime's raw lighthouse pose process sets ctx->bsd[lh].Pose from it, at once or nudged, and records LH_POSE)
	fn raw_lighthouse_pose(&mut self, lh: usize, pose: &Pose);

	// ---- the devices (SurviveObject)

	fn sensor_ct(&self, dev: DeviceId) -> i32;
	/// so->sensor_locations != 0
	fn sensor_locations_set(&self, dev: DeviceId) -> bool;
	/// so->sensor_normals != 0
	fn sensor_normals_set(&self, dev: DeviceId) -> bool;
	/// so->sensor_locations[3 * sensor ..][..3] (sensor < so->sensor_ct, checked by the caller; a host whose array is
	/// shorter stops: the C would read past it)
	fn sensor_location(&self, dev: DeviceId, sensor: usize) -> [f64; 3];
	/// so->sensor_locations[3 * sensor ..][..3] = *loc (device-calibration-file's corrections)
	fn set_sensor_location(&mut self, dev: DeviceId, sensor: usize, loc: &[f64; 3]);
	/// so->sensor_normals[3 * sensor ..][..3]
	fn sensor_normal(&self, dev: DeviceId, sensor: usize) -> [f64; 3];
	fn timebase_hz(&self, dev: DeviceId) -> i32;
	/// so->velocity as it stands when the imupose hook runs (the tracker's velocity hook comes after its imupose
	/// hook, so this is the velocity reported with the previous pose)
	fn velocity(&self, dev: DeviceId) -> Velocity;
	fn serial_number(&self, dev: DeviceId) -> String;
	fn codename(&self, dev: DeviceId) -> String;
	/// SurviveSensorActivations_long_timecode_light(&so->activations, timecode): a hit's 32-bit timecode extended
	/// with the device's clock
	fn long_timecode_light(&self, dev: DeviceId, timecode: u32) -> u64;

	// ---- the config store (the map solve's options are registered there, with their defaults)

	/// survive_configf(ctx, tag, SC_GET, def): an option's value as a double (the registered default when it is not
	/// set, def for a tag not registered). The C's survive_attach_configf for the bound options reads this, after it has
	/// set an absent entry to the default in the temporary group (SC_SET) and bound the variable
	fn configf(&mut self, tag: &str, def: f64) -> f64;
	/// survive_configi(ctx, tag, SC_GET, def): as an integer
	fn configi(&mut self, tag: &str, def: u32) -> u32;
	/// survive_configs(ctx, tag, SC_GET, def): as text (a file name)
	fn configs(&mut self, tag: &str, def: &str) -> String;
	/// ctx->log_level (the option v): SV_VERBOSE(level, ...) writes when it is at least the line's level
	fn log_level(&self) -> i32;

	// ---- text

	/// SV_INFO's text, as the log hook gets it (SURVIVE_LOG_LEVEL_INFO)
	fn info(&mut self, msg: &str);
	/// SV_WARN's text (SURVIVE_LOG_LEVEL_WARNING)
	fn warn(&mut self, msg: &str);
	/// survive_recording_write_to_output(ctx->recptr, "%s", line): a recording line after the run time (nothing when
	/// there is no recording)
	fn record(&mut self, line: &str);
}
