//! Analysis diagnostics into the recording (--diagrec 1, off by default, never used live): a DST line per reported
//! pose, the device's tracking state with it. It writes no RPT lines: nothing reports solves.

use sally_math::printf::Arg;
use sally_math::Pose;

use crate::driver::Driver;
use crate::runtime::Runtime;
use crate::ObjectId;

/// The diagrec driver
pub struct DiagRec;

impl Driver for DiagRec {
	/// "%s DST run_t dev_t still_s pos_sd lhmask last_obs_age x y z vx vy vz ax ay az": the pose in IMU space, the
	/// velocity the device had before this report (its velocity hook runs after this one), and three zeros where an
	/// acceleration state once was (kept so the fields stay where they are)
	fn imupose(&mut self, rt: &mut Runtime, obj: ObjectId, timecode: u64, pose: &Pose) {
		let so = &rt.objects[obj];
		if so.timebase_hz == 0 {
			return; // a device with no clock yet: nothing to time the line with
		}
		let hz = so.timebase_hz as f64;
		let still = so.activations.stationary_time() as f64 / hz;
		let tt = timecode as f64 / hz;
		let (codename, vel) = (so.codename.clone(), so.velocity.pos);
		let d = rt.tracker_diag(obj);
		let t = rt.run_time();
		let args = [
			Arg::S(&codename),
			Arg::F(t),
			Arg::F(tt),
			Arg::F(still),
			Arg::F(d.pos_sd),
			Arg::U(d.lh_mask as u64),
			Arg::F(d.light_age),
			Arg::F(pose.pos[0]),
			Arg::F(pose.pos[1]),
			Arg::F(pose.pos[2]),
			Arg::F(vel[0]),
			Arg::F(vel[1]),
			Arg::F(vel[2]),
			Arg::F(0.0),
			Arg::F(0.0),
			Arg::F(0.0),
		];
		rt.record(|r, now| {
			r.write(now, "%s DST %.6f %.6f %.4f %.6f %x %.4f %.6f %.6f %.6f %.5f %.5f %.5f %.3f %.3f %.3f\n", &args)
		});
	}
}
