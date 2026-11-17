//! A device's recent measurements: the last angle each sensor saw from each station (with the outlier filter that
//! decides whether a new one is believed), the mean angle per station and sweep plane at each sync, the IMU's running
//! averages, and the device clock as the light and the IMU last gave it (survive_sensor_activations.c,
//! SurviveSensorActivations). The gen2 half only: the gen1 `SurviveSensorActivations_add` went with the LH1 decoder.

use sally_math::{dist3d, linmath_chauvenet_criterion, linmath_max, norm3d};

use crate::NUM_GEN2_LIGHTHOUSES;

/// survive_types.h: the most sensors a device has
pub const SENSORS_PER_OBJECT: usize = 32;

/// The filter's settings (struct SurviveSensorActivations_params), from the options of the same names
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
	/// move-threshold-gyro: gyro norms above this count as moving
	pub move_threshold_gyro: f64,
	/// move-threshold-acc: accelerometer differences (to the running average) above this count as moving
	pub move_threshold_acc: f64,
	/// move-threshold-ang: light angle changes above this count as moving
	pub move_threshold_ang: f64,
	/// filter-threshold-ang-per-sec: an angle changing faster than this (rad/s) is rejected
	pub filter_light_change: f64,
	/// filter-light-outlier-criteria: Chauvenet's criterion below this is an outlier
	pub filter_outlier_criteria: f64,
	/// filter-variance-minimum: the least deviation the outlier test assumes
	pub filter_variance_min: f64,
	/// filter-outlier-minimum-count: the least population the outlier test assumes
	pub filter_outlier_min_count: i32,
}

impl Default for Params {
	/// The options' defaults
	fn default() -> Self {
		Params {
			move_threshold_gyro: 0.075,
			move_threshold_acc: 0.03,
			move_threshold_ang: 0.02,
			filter_light_change: 50.0,
			filter_outlier_criteria: 0.5,
			filter_variance_min: 0.05,
			filter_outlier_min_count: 11,
		}
	}
}

/// One IMU sample as the activations take it (PoserDataIMU: the long timecode and the calibrated values)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
	pub timecode: u64,
	pub accel: [f64; 3],
	pub gyro: [f64; 3],
	pub mag: [f64; 3],
}

/// What a gen2 light event is (PoserDataLightGen2): a station's sync, or a sensor's hit on one of its planes
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gen2Light {
	/// POSERDATA_SYNC_GEN2
	Sync { timecode: u64, lh: usize },
	/// POSERDATA_LIGHT_GEN2: the angle on the plane (0 or 1)
	Hit { timecode: u64, lh: usize, sensor_id: usize, plane: usize, angle: f64 },
}

/// SurviveSensorActivations (gen2)
#[derive(Clone, Debug)]
pub struct Activations {
	/// -1 until light arrives, 1 once gen2 light did (0 was gen1)
	pub lh_gen: i32,
	/// the rotor angle of the last believed hit per sensor, station and plane
	pub angles: [[[f64; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
	/// per station and plane, at each sync: the mean of the recent raw angles, their deviation, and how many there were
	pub angles_center_x: [[f64; 2]; NUM_GEN2_LIGHTHOUSES],
	pub angles_center_dev: [[f64; 2]; NUM_GEN2_LIGHTHOUSES],
	pub angles_center_cnt: [[i32; 2]; NUM_GEN2_LIGHTHOUSES],
	/// the last angle per sensor, station and plane whether believed or not, and its timecode
	pub raw_angles: [[[f64; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
	pub raw_timecode: [[[u64; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
	/// the timecode of each believed angle
	pub timecode: [[[u64; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
	/// IMU samples still skipped at the start
	pub imu_init_cnt: usize,
	pub last_imu: u64,
	pub last_light: u64,
	pub last_light_change: u64,
	/// the timecode of the last sample (IMU or light) that saw movement
	pub last_movement: u64,
	/// the host clock (us) less the device clock (us), smoothed
	pub runtime_offset: f64,
	/// the IMU's running averages (NaN before the first sample), and the last accelerometer sample
	pub accel: [f64; 3],
	pub last_accel: [f64; 3],
	pub gyro: [f64; 3],
	pub mag: [f64; 3],
	pub params: Params,
}

impl Activations {
	/// SurviveSensorActivations_ctor: a device's activations before anything arrived
	pub fn new(params: Params) -> Self {
		let mut a = Activations {
			lh_gen: 0,
			angles: [[[0.0; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			angles_center_x: [[0.0; 2]; NUM_GEN2_LIGHTHOUSES],
			angles_center_dev: [[0.0; 2]; NUM_GEN2_LIGHTHOUSES],
			angles_center_cnt: [[0; 2]; NUM_GEN2_LIGHTHOUSES],
			raw_angles: [[[0.0; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			raw_timecode: [[[0; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			timecode: [[[0; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			imu_init_cnt: 0,
			last_imu: 0,
			last_light: 0,
			last_light_change: 0,
			last_movement: 0,
			runtime_offset: 0.0,
			accel: [0.0; 3],
			last_accel: [0.0; 3],
			gyro: [0.0; 3],
			mag: [0.0; 3],
			params,
		};
		a.reset();
		a.lh_gen = -1;
		a
	}

	/// SurviveSensorActivations_reset: everything back to the start but the settings (lh_gen to 0, as the C's memset
	/// leaves it)
	pub fn reset(&mut self) {
		let params = self.params;
		*self = Activations {
			lh_gen: 0,
			angles: [[[f64::NAN; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			angles_center_x: [[f64::NAN; 2]; NUM_GEN2_LIGHTHOUSES],
			angles_center_dev: [[0.0; 2]; NUM_GEN2_LIGHTHOUSES],
			angles_center_cnt: [[0; 2]; NUM_GEN2_LIGHTHOUSES],
			raw_angles: [[[f64::NAN; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			raw_timecode: [[[0; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			timecode: [[[0; 2]; NUM_GEN2_LIGHTHOUSES]; SENSORS_PER_OBJECT],
			imu_init_cnt: 30,
			last_imu: 0,
			last_light: 0,
			last_light_change: 0,
			last_movement: 0,
			runtime_offset: 0.0,
			accel: [f64::NAN; 3],
			last_accel: [0.0; 3],
			gyro: [0.0; 3],
			mag: [0.0; 3],
			params,
		};
	}

	/// SurviveSensorActivations_last_time: the later of the last light and the last IMU timecode
	pub fn last_time(&self) -> u64 {
		if self.last_imu > self.last_light {
			self.last_imu
		} else {
			self.last_light
		}
	}

	/// SurviveSensorActivations_stationary_time: how long since the last movement (0 if none seen)
	pub fn stationary_time(&self) -> u64 {
		let last_time = self.last_time();
		let last_move = self.last_movement;
		if last_move == 0 {
			return 0;
		}
		last_time.wrapping_sub(last_move)
	}

	/// SurviveSensorActivations_register_runtime: the host clock at a sample of device timecode `tc`
	pub fn register_runtime(&mut self, tc: u64, runtime_clock_us: u64) {
		let runtime_offset = runtime_clock_us.wrapping_sub((tc as f64 * 0.0208333333) as u64) as f64;
		if self.runtime_offset == 0.0 {
			self.runtime_offset = runtime_offset;
		} else {
			self.runtime_offset = self.runtime_offset * 0.90 + 0.1 * runtime_offset;
		}
	}

	/// SurviveSensorActivations_add_imu: an IMU sample (the first 30 only move the clock)
	pub fn add_imu(&mut self, imu: &ImuSample) {
		self.last_imu = imu.timecode;
		if self.imu_init_cnt > 0 {
			self.imu_init_cnt -= 1;
			return;
		}
		self.last_accel = imu.accel;
		if self.accel[0].is_nan() {
			for i in 0..3 {
				self.accel[i] = imu.accel[i];
				self.gyro[i] = imu.gyro[i];
				self.mag[i] = imu.mag[i];
			}
			self.last_movement = imu.timecode;
		} else {
			for i in 0..3 {
				self.accel[i] = 0.98 * self.accel[i] + 0.02 * imu.accel[i];
				self.gyro[i] = 0.98 * self.gyro[i] + 0.02 * imu.gyro[i];
				self.mag[i] = 0.98 * self.mag[i] + 0.02 * imu.mag[i];
			}
		}
		if norm3d(&imu.gyro) > self.params.move_threshold_gyro
			|| dist3d(&self.accel, &imu.accel) > self.params.move_threshold_acc
		{
			self.last_movement = imu.timecode;
		}
	}

	/// SurviveSensorActivations_check_outlier: whether a hit is to be rejected (too fast a change from the sensor's
	/// last believed angle, or too far from the plane's mean at the last sync by Chauvenet's criterion)
	fn check_outlier(&self, sensor_id: usize, lh: usize, axis: usize, timecode: u64, angle: f64) -> bool {
		if self.angles_center_dev[lh][axis] == 0.0 {
			return false;
		}
		let oldangle = self.angles[sensor_id][lh][axis];
		let data_timecode = self.timecode[sensor_id][lh][axis];
		let change_rate = (oldangle - angle).abs() / timecode.wrapping_sub(data_timecode) as f64 * 48000000.0;
		if data_timecode != 0 && change_rate > self.params.filter_light_change && self.params.filter_light_change > -1.0
		{
			return true;
		}
		let measured_dev = self.angles_center_dev[lh][axis];
		let dev = linmath_max(self.params.filter_variance_min, measured_dev);
		let mut cnt = self.angles_center_cnt[lh][axis];
		if cnt < self.params.filter_outlier_min_count {
			cnt = self.params.filter_outlier_min_count;
		}
		let chauvenet_criterion = linmath_chauvenet_criterion(angle, self.angles_center_x[lh][axis], dev, cnt);
		measured_dev > 0.0
			&& self.params.filter_outlier_criteria > 0.0
			&& chauvenet_criterion < self.params.filter_outlier_criteria
	}

	/// SurviveSensorActivations_add_gen2: a sync or a hit; false when the hit is rejected (or its sensor out of range)
	pub fn add_gen2(&mut self, light: &Gen2Light) -> bool {
		self.lh_gen = 1;
		let timecode = match *light {
			Gen2Light::Sync { timecode, lh } => {
				self.add_sync(lh, timecode);
				timecode
			}
			Gen2Light::Hit { timecode, lh, sensor_id, plane: axis, angle } => {
				if sensor_id >= SENSORS_PER_OBJECT {
					return false;
				}
				self.raw_angles[sensor_id][lh][axis] = angle;
				self.raw_timecode[sensor_id][lh][axis] = timecode;
				if self.check_outlier(sensor_id, lh, axis, timecode, angle) {
					return false;
				}
				let old = self.angles[sensor_id][lh][axis];
				if !old.is_nan() && (old - angle).abs() > self.params.move_threshold_ang {
					self.last_light_change = timecode;
					self.last_movement = timecode;
				}
				if old.is_nan() {
					self.last_light_change = timecode;
				}
				self.timecode[sensor_id][lh][axis] = timecode;
				self.angles[sensor_id][lh][axis] = angle;
				timecode
			}
		};
		if timecode > self.last_light {
			self.last_light = timecode;
		}
		true
	}

	/// SurviveSensorActivations_add_sync: each plane's mean and deviation over the sensors' raw angles of the last
	/// half second, in up to two passes (the second leaves out what the first's criterion calls outliers)
	fn add_sync(&mut self, lh: usize, timecode: u64) {
		for axis in 0..2 {
			let mut changes = true;
			let mut mean = 0.0f64;
			let mut deviation = 0.0f64;
			let mut cnt: i32 = 0;
			let mut passes = 0;
			while passes < 2 && changes {
				changes = passes == 0;
				let mut variance = Variance::default();
				for i in 0..SENSORS_PER_OBJECT {
					let sensor_timecode = self.raw_timecode[i][lh][axis];
					let angle = self.raw_angles[i][lh][axis];
					let is_recent = timecode.wrapping_sub(sensor_timecode) < 48000000 / 2;
					if is_recent && angle.is_finite() {
						let unbias_deviation = deviation - (mean - angle).abs() / cnt as f64;
						let chauvenet_criterion = linmath_chauvenet_criterion(angle, mean, unbias_deviation, cnt);
						let is_outlier = self.params.filter_outlier_criteria > 0.0
							&& chauvenet_criterion < self.params.filter_outlier_criteria
							&& deviation != 0.0;
						if !is_outlier {
							variance.add(angle);
						} else {
							changes = true;
						}
					}
				}
				if variance.n != 0 {
					deviation = variance.calc().sqrt();
					mean = variance.sum / variance.n as f64;
					cnt = variance.n as i32;
				}
				passes += 1;
			}
			self.angles_center_x[lh][axis] = mean;
			self.angles_center_dev[lh][axis] = deviation;
			self.angles_center_cnt[lh][axis] = cnt;
		}
	}

	/// SurviveSensorActivations_long_timecode_imu: an IMU's 32-bit timecode extended past the wraps
	pub fn long_timecode_imu(&self, timecode: u32) -> u64 {
		make_long_timecode(self.last_imu, timecode)
	}

	/// SurviveSensorActivations_long_timecode_light: a light timecode extended past the wraps, and moved by whole 2^28
	/// ticks when it lies more than a second from the IMU's clock (the firmware's lightcap / IMU clock slip)
	pub fn long_timecode_light(&self, timecode: u32) -> u64 {
		let mut initial_time = make_long_timecode(self.last_light, timecode);
		let time_sync_error = initial_time.wrapping_sub(self.last_imu) as i64;
		if self.last_imu != 0 && time_sync_error.wrapping_abs() > 48000000 {
			let offset: i64 = 0x10000000;
			// DIV_ROUND_CLOSEST, into an int
			let scale = if (time_sync_error < 0) ^ (offset < 0) {
				(time_sync_error - offset / 2) / offset
			} else {
				(time_sync_error + offset / 2) / offset
			} as i32;
			initial_time = initial_time.wrapping_sub(offset.wrapping_mul(scale as i64) as u64);
		}
		initial_time
	}
}

/// make_long_timecode: a 32-bit timecode in the 2^32 period that puts it nearest the previous long timecode
fn make_long_timecode(prev: u64, current: u32) -> u64 {
	let mut rtn = current as u64 | (prev & 0xFFFFFFFF00000000);
	if rtn < prev && rtn.wrapping_add(0x80000000) < prev {
		rtn = rtn.wrapping_add(0x100000000);
	}
	if rtn > prev && prev.wrapping_add(0x80000000) < rtn && rtn > 0x100000000 {
		rtn = rtn.wrapping_sub(0x100000000);
	}
	rtn
}

/// struct variance_measure (variance.h) of one value: the count, the sum and the sum of squares
#[derive(Default)]
struct Variance {
	n: usize,
	sum: f64,
	sum_sq: f64,
}

impl Variance {
	/// variance_measure_add (its guard against non-finite values never fires here: add_sync passes finite ones only)
	fn add(&mut self, d: f64) {
		self.n += 1;
		self.sum += d;
		self.sum_sq += d * d;
	}

	/// variance_measure_calc
	fn calc(&self) -> f64 {
		(self.sum_sq - (self.sum * self.sum) / self.n as f64) / self.n as f64
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn long_timecodes_follow_the_wraps() {
		assert_eq!(make_long_timecode(0, 5), 5);
		assert_eq!(make_long_timecode(0xFFFF_FFF0, 0x10), 0x1_0000_0010);
		assert_eq!(make_long_timecode(0x1_0000_0010, 0xFFFF_FFF0), 0xFFFF_FFF0);
		let mut a = Activations::new(Params::default());
		a.last_imu = 0x1_0000_0000 + 48000000 * 3;
		a.last_light = a.last_imu;
		// a light time 2^28 ticks (5.6 s) off the IMU's is moved onto it
		let t = (a.last_imu + 0x10000000) as u32;
		assert_eq!(a.long_timecode_light(t), a.last_imu);
	}

	#[test]
	fn a_sync_takes_the_mean_of_recent_angles() {
		let mut a = Activations::new(Params::default());
		for (i, angle) in [0.1, 0.2, 0.3].into_iter().enumerate() {
			assert!(a.add_gen2(&Gen2Light::Hit { timecode: 1000 + i as u64, lh: 2, sensor_id: i, plane: 1, angle }));
		}
		assert!(a.add_gen2(&Gen2Light::Sync { timecode: 2000, lh: 2 }));
		// the second pass's criterion (with the first pass's deviation less each angle's share) leaves out the outer two
		assert!((a.angles_center_x[2][1] - 0.2).abs() < 1e-15);
		assert_eq!(a.angles_center_cnt[2][1], 1);
		assert_eq!(a.angles_center_dev[2][1], 0.0);
		// no recent angles on plane 0: the mean is 0, not NaN, as in the C
		assert_eq!(a.angles_center_x[2][0], 0.0);
		assert_eq!(a.last_light, 2000);
	}
}
