//! A tracked object (SurviveObject, survive.h): one device as the runtime keeps it: its names, its config and what the
//! config says about it (the sensors, the IMU's calibration), its recent measurements, and its view of each station's
//! rotor (the last sync, the interval, the count).

use sally_lighthouse::activations::{Activations, Params, SENSORS_PER_OBJECT};
use sally_lighthouse::gen2::PlaneStats;
use sally_lighthouse::NUM_GEN2_LIGHTHOUSES;
use sally_math::{Pose, LINMATHPI};
use sally_track::host::SurviveVelocity;
use sally_track::tracker::Tracker;

/// SurviveObjectType (survive_types.h)
pub mod object_type {
	pub const UNKNOWN: i32 = 0;
	pub const HMD: i32 = 1;
	pub const CONTROLLER: i32 = 2;
	pub const OTHER: i32 = 3;
}

/// VIVE_DEFAULT_IMU_HZ (a float in the C)
pub const VIVE_DEFAULT_IMU_HZ: f64 = 250.0f32 as f64;

/// SurviveObject's stats, the counters its processes keep
#[derive(Clone, Debug, Default)]
pub struct Stats {
	pub syncs: [u32; NUM_GEN2_LIGHTHOUSES],
	pub skipped_syncs: [u32; NUM_GEN2_LIGHTHOUSES],
	pub bad_syncs: [u32; NUM_GEN2_LIGHTHOUSES],
	pub hit_from_lhs: [u32; NUM_GEN2_LIGHTHOUSES],
	pub accepted_data: [u32; NUM_GEN2_LIGHTHOUSES],
	pub rejected_data: [u32; NUM_GEN2_LIGHTHOUSES],
	pub dropped_light: [u32; NUM_GEN2_LIGHTHOUSES],
	pub sync_resets: [u32; NUM_GEN2_LIGHTHOUSES],
	/// determine_plane's: extent hits and misses, naive hits, the extent's range
	pub plane: PlaneStats,
	/// sally: gen2-plane-predict's checks, relabels and drops; light-facing-deg's checks and drops
	pub plane_predict_checks: u32,
	pub plane_predict_fixes: u32,
	pub plane_predict_drops: u32,
	pub facing_checks: u32,
	pub facing_drops: u32,
}

/// handle_lightcap's gen detection state per device (disambiguate_version): pulses counted while the system's
/// generation is unknown
#[derive(Clone, Debug)]
pub struct GenDetect {
	pub pulse_count: usize,
	pub total_count: usize,
	pub last_pulse_times: [u32; SENSORS_PER_OBJECT],
}

impl Default for GenDetect {
	fn default() -> Self {
		GenDetect { pulse_count: 0, total_count: 0, last_pulse_times: [0; SENSORS_PER_OBJECT] }
	}
}

/// SurviveObject
pub struct TrackedObject {
	/// "KN0": the C keeps it in char[4]
	pub codename: String,
	pub drivername: String,
	/// the device's serial (char[16]: 15 at most) and the dongle it talks through (char[32])
	pub serial_number: String,
	pub sally_dongle: String,
	pub object_type: i32,
	pub object_subtype: i32,
	/// the config's text, once it came (so->conf)
	pub conf: Option<Vec<u8>>,
	/// the battery as the device last reported it (so->charge, %, and so->charging: handle_battery's, from the parser
	/// through the tap's host; 0 and false until a report says)
	pub charge: i8,
	pub charging: bool,

	/// the device's tracker (so->tracker): made with the device, freed with it
	pub tracker: Option<Box<Tracker>>,
	/// the last pose in IMU space (so->OutPoseIMU: the tracker's, for the plane prediction and the facing gate), the
	/// last pose at the tracked point (OutPose) and velocity, with the device times they came at
	pub out_pose_imu: Pose,
	pub out_pose: Pose,
	pub out_pose_timecode: u64,
	pub velocity: SurviveVelocity,
	pub velocity_timecode: u64,

	pub sensor_ct: i8,
	pub channel_map: Option<Vec<i32>>,
	pub has_sensor_locations: bool,
	pub sensor_locations: Option<Vec<f64>>,
	pub sensor_normals: Option<Vec<f64>>,

	pub timebase_hz: i32,
	/// per station: the interval between its last two syncs, its last sync, and the syncs counted (with the skipped)
	pub last_time_between_sync: [u32; NUM_GEN2_LIGHTHOUSES],
	pub last_sync_time: [u32; NUM_GEN2_LIGHTHOUSES],
	pub sync_count: [u32; NUM_GEN2_LIGHTHOUSES],
	pub imu_freq: f64,

	pub head2trackref: Pose,
	pub imu2trackref: Pose,
	pub head2imu: Pose,

	/// the IMU's raw scales (counts to m/s^2 and rad/s) and its calibration
	pub raw_acc_scale: f64,
	pub raw_gyro_scale: f64,
	pub acc_bias: [f64; 3],
	pub acc_scale: [f64; 3],
	pub gyro_bias: [f64; 3],
	pub gyro_scale: [f64; 3],

	pub activations: Box<Activations>,
	pub stats: Stats,
	pub sensor_scale: f64,
	pub sensor_scale_var: f64,

	/// handle_lightcap's state while the generation is unknown (so->disambiguator_data)
	pub gen_detect: Option<GenDetect>,
}

impl TrackedObject {
	/// survive_create_device: a device before its config, named `codename` (by the driver `drivername`)
	pub fn new(drivername: &str, codename: &str, activation_params: Params) -> Self {
		TrackedObject {
			codename: codename.to_string(),
			drivername: drivername.chars().take(8).collect(),
			serial_number: String::new(),
			sally_dongle: String::new(),
			object_type: object_type::OTHER,
			object_subtype: 0,
			conf: None,
			charge: 0,
			charging: false,
			tracker: None,
			out_pose_imu: Pose::default(),
			out_pose: Pose::default(),
			out_pose_timecode: 0,
			velocity: SurviveVelocity::default(),
			velocity_timecode: 0,
			sensor_ct: 0,
			channel_map: None,
			has_sensor_locations: false,
			sensor_locations: None,
			sensor_normals: None,
			timebase_hz: 48000000,
			last_time_between_sync: [0; NUM_GEN2_LIGHTHOUSES],
			last_sync_time: [0; NUM_GEN2_LIGHTHOUSES],
			sync_count: [0; NUM_GEN2_LIGHTHOUSES],
			imu_freq: VIVE_DEFAULT_IMU_HZ,
			head2trackref: Pose { pos: [0.0; 3], rot: [1.0, 0.0, 0.0, 0.0] },
			imu2trackref: Pose { pos: [0.0; 3], rot: [1.0, 0.0, 0.0, 0.0] },
			head2imu: Pose::default(),
			raw_acc_scale: 0.0,
			raw_gyro_scale: 0.0,
			acc_bias: [0.0; 3],
			acc_scale: [1.0; 3],
			gyro_bias: [0.0; 3],
			gyro_scale: [1.0; 3],
			activations: Box::new(Activations::new(activation_params)),
			stats: Stats::default(),
			sensor_scale: 1.0,
			sensor_scale_var: 0.1,
			gen_detect: None,
		}
	}

	/// survive_map_sensor_id: a reported sensor through the config's channel map (255: not one of this device's)
	pub fn map_sensor_id(&self, reported_id: u8) -> u8 {
		if let Some(map) = &self.channel_map {
			let mapped_id = map.get(reported_id as usize).copied().unwrap_or(-1) as u8;
			if mapped_id as i32 >= self.sensor_ct as i32 {
				return 255;
			}
			return mapped_id;
		}
		reported_id
	}

	/// survive_default_set_imu_scale_modes' scales (the recording's IMU_SCALES line is the runtime's): the full ranges
	/// of the accelerometer (2, 4, 8, 16 g) and the gyro (250, 500, 1000, 2000 deg/s) over 2^15
	pub fn set_imu_scale_modes(&mut self, gyro_scale_mode: i32, acc_scale_mode: i32) {
		const GYRO_SCALES: [f64; 4] = [250.0, 500.0, 1000.0, 2000.0];
		const ACC_SCALES: [f64; 4] = [2.0, 4.0, 8.0, 16.0];
		// the C's indices are ints, compared with < 4 only (a negative one would read before the arrays; the parser
		// gives bytes, never negative)
		if (0..4).contains(&acc_scale_mode) {
			self.raw_acc_scale = ACC_SCALES[acc_scale_mode as usize] / (1 << 15) as f64;
		}
		if (0..4).contains(&gyro_scale_mode) {
			self.raw_gyro_scale = GYRO_SCALES[gyro_scale_mode as usize] / (1 << 15) as f64 * LINMATHPI / 180.0;
		}
	}
}
