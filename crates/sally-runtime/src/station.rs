//! A base station as the runtime knows it (BaseStationData, survive.h): its channel, the calibration and up direction
//! its OOTX data gave, its pose in the world, and the decoder its OOTX bits go to.

use sally_lighthouse::ootx::OotxDecoder;
use sally_lighthouse::BaseStationCal;
use sally_math::Pose;

use crate::config::LighthouseRecord;
use crate::ObjectId;

/// SurviveAxisAnglePose: a pose's variance, position and axis-angle
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AxisAnglePose {
	pub pos: [f64; 3],
	pub axis_angle_rot: [f64; 3],
}

/// The OOTX decoder a station's bits go to (ctx->bsd[i].ootx_data), fed by one device only (its `user`)
#[derive(Clone, Debug)]
pub struct OotxSlot {
	pub decoder: OotxDecoder,
	/// the device whose syncs feed it
	pub device: ObjectId,
}

/// BaseStationData
#[derive(Clone, Debug)]
pub struct Station {
	pub position_set: bool,
	/// the station's pose: lighthouse to world
	pub pose: Pose,
	pub ootx_set: bool,
	pub base_station_id: u32,
	pub fcal: [BaseStationCal; 2],
	pub sys_unlock_count: u8,
	/// the "up" vector of its accelerometer
	pub accel: [f64; 3],
	/// the channel (0xFF: the slot is free)
	pub mode: u8,
	pub confidence: f64,
	pub ootx: Option<OotxSlot>,
	pub disable: bool,
	pub ootx_checked: bool,
	pub variance: AxisAnglePose,
	/// a move in progress (survive_default_raw_lighthouse_pose_process's slow nudge): from old_pos at old_pos_time to
	/// true_pos at true_pos_time (0: none)
	pub old_pos_time: f64,
	pub old_pos: Pose,
	pub true_pos_time: f64,
	pub true_pos: Pose,
	/// sally: map-solve's moved-station verdict: 0 fine, 1 disagrees with the map, 2 relocated at sally_moved_time
	pub sally_moved: i8,
	pub sally_moved_time: f64,
}

impl Default for Station {
	/// (BaseStationData){0}: all zero, the channel 0 (survive_get_bsd_idx sets it right after)
	fn default() -> Self {
		Station {
			position_set: false,
			pose: Pose::default(),
			ootx_set: false,
			base_station_id: 0,
			fcal: [BaseStationCal::default(); 2],
			sys_unlock_count: 0,
			accel: [0.0; 3],
			mode: 0,
			confidence: 0.0,
			ootx: None,
			disable: false,
			ootx_checked: false,
			variance: AxisAnglePose::default(),
			old_pos_time: 0.0,
			old_pos: Pose::default(),
			true_pos_time: 0.0,
			true_pos: Pose::default(),
			sally_moved: 0,
			sally_moved_time: 0.0,
		}
	}
}

impl Station {
	/// A free slot, as survive_init leaves each (mode = -1)
	pub fn unused() -> Self {
		Station { mode: 0xFF, ..Station::default() }
	}

	/// The fields of the station the config file keeps (config_read_lighthouse and config_set_lighthouse work on the
	/// BaseStationData itself)
	pub fn record(&self) -> LighthouseRecord {
		let v = &self.variance;
		LighthouseRecord {
			position_set: self.position_set,
			pose: self.pose,
			ootx_set: self.ootx_set,
			base_station_id: self.base_station_id,
			fcal: self.fcal,
			sys_unlock_count: self.sys_unlock_count,
			accel: self.accel,
			mode: self.mode,
			variance: [v.pos[0], v.pos[1], v.pos[2], v.axis_angle_rot[0], v.axis_angle_rot[1], v.axis_angle_rot[2]],
		}
	}

	/// The fields of `record` back into the station
	pub fn set_record(&mut self, r: &LighthouseRecord) {
		self.position_set = r.position_set;
		self.pose = r.pose;
		self.ootx_set = r.ootx_set;
		self.base_station_id = r.base_station_id;
		self.fcal = r.fcal;
		self.sys_unlock_count = r.sys_unlock_count;
		self.accel = r.accel;
		self.mode = r.mode;
		let v = &r.variance;
		self.variance = AxisAnglePose { pos: [v[0], v[1], v[2]], axis_angle_rot: [v[3], v[4], v[5]] };
	}
}
