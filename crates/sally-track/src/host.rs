//! What the tracker reads and does outside its own state: the `Host` trait (the device, the stations, the text), its
//! settings, and the plain data it shares with its host. sally-runtime implements the trait over its own objects and
//! stations.

use std::ffi::{c_int, CStr};

use crate::text::{log_text, CArgs, Name};

pub use sally_lighthouse::{BaseStationCal, NUM_GEN2_LIGHTHOUSES};

/// SurvivePose (LinmathPose): Pos[3], Rot[4] (w x y z)
pub type SurvivePose = sally_math::Pose;
/// SurviveObjectType: SURVIVE_OBJECT_TYPE_CONTROLLER
pub const SURVIVE_OBJECT_TYPE_CONTROLLER: c_int = 2;
/// A device's tracking state as the recordings number it (libsurvive's SURVIVE_DIAG_STATE_*)
pub const SURVIVE_DIAG_STATE_NONE: c_int = 0;
pub const SURVIVE_DIAG_STATE_TRACKING: c_int = 1;
pub const SURVIVE_DIAG_STATE_DARK: c_int = 2;
pub const SURVIVE_DIAG_STATE_COASTING: c_int = 3;
pub const SURVIVE_DIAG_STATE_REACQUIRING: c_int = 4;

/// SurviveVelocity (LinmathAxisAnglePose): Pos[3], AxisAngleRot[3]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SurviveVelocity {
	pub pos: [f64; 3],
	pub axis_angle_rot: [f64; 3],
}

/// The tracker's 44 options (options.rs has their tags, descriptions and defaults), in their order, with their C types.
/// Each tracker has its own copy and reads a field at every use; the runtime gives it a new copy when an option is set
/// live (`Tracker::set_settings`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
	pub sig_hit: f64,
	pub sig_common: f64,
	pub sig_join: f64,
	pub hit_knee: f64,
	pub gate: f64,
	pub gate_floor: f64,
	pub zupt_sd: f64,
	pub zupt_gyro: f64,
	pub zupt_vib: f64,
	pub zupt_time: f64,
	pub hit_shift: f64,
	pub sig_acc: f64,
	pub acc_knoise: f64,
	pub sig_gyr: f64,
	pub sig_ba: f64,
	pub sig_bg: f64,
	pub sig_ls: f64,
	pub ls_max: f64,
	pub adapt: f64,
	pub acc_tnoise: f64,
	pub adapt_trackers: c_int,
	pub vib0: f64,
	pub facing_deg: f64,
	pub blink: f64,
	pub probe_ch: c_int,
	pub only_mask: c_int,
	pub reseed_frac: f64,
	pub gap_coast: f64,
	pub gap_tau: f64,
	pub gap_reseed: f64,
	pub seed_fit: f64,
	pub seed_window: f64,
	pub clip_sd: f64,
	pub reseed_min_hits: c_int,
	pub p0_pos: f64,
	pub p0_vel: f64,
	pub p0_rot: f64,
	pub p0_bg: f64,
	pub p0_ba: f64,
	pub p0_ls: f64,
	pub gt_mag: f64,
	pub gt_az: f64,
	pub gt_sd: f64,
	pub verbose: c_int,
}

/// What the tracker reads and does outside its own state. A method that takes a sensor or a station index asserts it is
/// one
pub trait Host {
	// the device (SurviveObject)
	/// so->sensor_ct
	fn sensor_ct(&self) -> i32;
	/// so->sensor_locations != 0, so->sensor_normals != 0
	fn sensor_locations_set(&self) -> bool;
	fn sensor_normals_set(&self) -> bool;
	/// &so->sensor_locations[3 * i] (and the normals)
	fn sensor_location(&self, i: i32) -> [f64; 3];
	fn sensor_normal(&self, i: i32) -> [f64; 3];
	fn object_type(&self) -> i32;
	fn timebase_hz(&self) -> i32;
	/// so->raw_acc_scale, so->raw_gyro_scale, so->acc_scale[i], so->acc_bias[i]
	fn raw_acc_scale(&self) -> f64;
	fn raw_gyro_scale(&self) -> f64;
	fn acc_scale(&self, i: usize) -> f64;
	fn acc_bias(&self, i: usize) -> f64;
	fn codename(&self) -> Name;

	// the context (SurviveContext)
	/// ctx->activeLighthouses
	fn active_lighthouses(&self) -> i32;
	fn lh_version(&self) -> i32;
	/// ctx->bsd[lh].mode (a uint8_t promoted to int), ctx->bsd[lh].PositionSet, ctx->bsd[lh].fcal
	fn bsd_mode(&self, lh: i32) -> i32;
	fn bsd_position_set(&self, lh: i32) -> bool;
	fn bsd_fcal(&self, lh: i32) -> [BaseStationCal; 2];
	/// *survive_get_lighthouse_position(ctx, lh): a station being moved slowly is moved on (and its LH_POSE recorded)
	fn lighthouse_position(&mut self, lh: i32) -> SurvivePose;

	// text
	/// ctx->recptr != 0: a recording is being written
	fn recording(&self) -> bool;
	/// SV_INFO("%s", text) / SV_WARN("%s", text)
	fn log(&mut self, warn: bool, text: &str);
	/// A line for the recording, as it is
	fn record_line(&mut self, line: &str);

	/// An info line for the log (libsurvive's SV_INFO)
	fn sv_info<A: CArgs>(&mut self, fmt: &CStr, args: A) {
		self.log(false, &log_text(args.format(fmt)));
	}
	/// A warning line for the log (libsurvive's SV_WARN), cut at 1023 characters as an info line is (the tracker's
	/// warnings are a third of that at most)
	fn sv_warn<A: CArgs>(&mut self, fmt: &CStr, args: A) {
		self.log(true, &log_text(args.format(fmt)));
	}
	/// A formatted line for the recording: the whole line, however long (the caller checks recording() first)
	fn record<A: CArgs>(&mut self, fmt: &CStr, args: A) {
		self.record_line(&args.format(fmt));
	}
}
