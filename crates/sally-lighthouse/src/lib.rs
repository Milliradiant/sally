//! The lighthouse model: a base station's factory calibration, as its OOTX data brings it, and the geometry of its
//! gen2 sweeps (where a point appears on each sweep plane, the reprojection of survive_reproject_gen2.c), the
//! timing that turns a sweep hit into a rotor angle and a plane, and a device's recent measurements; ported operation
//! for operation (docs/checking.md).
//!
//! - `ootx`: the calibration bit stream's decoder and the gen2 info block it carries
//! - `device`: a device's configuration (its sensors, IMU calibration and poses), from the JSON it carries
//! - `reproject`: where a point appears on a station's two sweep planes
//! - `gen2`: syncs and sweep hits to rotor angles and planes
//! - `activations`: a device's recent light and IMU, with the light's outlier filter
//! - `steamvr`: SteamVR's folders, and its own copy of each device's config

// The port keeps the C's loops over indices and its order of operations, so the style lints that would rewrite them are
// off here; anything that would change a result is not.
#![allow(clippy::needless_range_loop)]

pub mod activations;
pub mod device;
pub mod gen2;
pub mod ootx;
pub mod reproject;
pub mod steamvr;

pub use reproject::{
	bc_fill_m, gen2_axis_consts, survive_reproject_axis_gen2_with, survive_reproject_axis_x_gen2,
	survive_reproject_axis_y_gen2, Gen2AxisConsts, ReprojectAxisFn, REPROJECT_AXIS_FN_GEN2,
};

/// survive_types.h: the most stations a gen2 system has (one per channel)
pub const NUM_GEN2_LIGHTHOUSES: usize = 16;

/// BaseStationCal (survive.h), field for field: one sweep plane's factory calibration (from the station's OOTX
/// data)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BaseStationCal {
	pub phase: f64,
	pub tilt: f64,
	pub curve: f64,
	pub gibpha: f64,
	pub gibmag: f64,
	// Gen 2 specific cal params
	pub ogeephase: f64,
	pub ogeemag: f64,
}
