//! A base station's part of the config file: the fields of BaseStationData kept per station, read from and written to
//! its "lighthouseN" group (config_read_lighthouse, config_set_lighthouse in survive_config.c).

use sally_lighthouse::BaseStationCal;
use sally_math::{quatiszero, Pose, Quat};

use super::store::{Config, NUM_GEN2_LIGHTHOUSES};

/// The fields of BaseStationData (survive.h) that config_read_lighthouse and config_set_lighthouse touch, with their C
/// types. The runtime copies between this and its own station state
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LighthouseRecord {
	/// PositionSet, a 1-bit field: a value read from the file keeps only its lowest bit
	pub position_set: bool,
	/// Pose: "a transformation from lh space to world space"
	pub pose: Pose,
	/// OOTXSet, a 1-bit field like PositionSet
	pub ootx_set: bool,
	pub base_station_id: u32,
	/// fcal: the factory calibration of the two sweep planes
	pub fcal: [BaseStationCal; 2],
	/// sys_unlock_count: written as "unlock_count", never read back
	pub sys_unlock_count: u8,
	/// accel: the "Up" vector
	pub accel: [f64; 3],
	/// mode, a uint8_t: a value read from the file keeps its low byte
	pub mode: u8,
	/// variance, a SurviveAxisAnglePose: `Pos[3]`, then `AxisAngleRot[3]`
	pub variance: [f64; 6],
}

/// quatnormalize (linmath.c): quatmagnitude, then quatdivs
fn quatnormalize(q: &Quat) -> Quat {
	let norm = ((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3])).sqrt();
	[q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
}

/// The file's names for the calibration fields, in the order of BaseStationCal and of the C's cal[] array (cal[2k + i]
/// is field k of fcal[i])
const FCAL_TAGS: [&str; 7] =
	["fcalphase", "fcaltilt", "fcalcurve", "fcalgibpha", "fcalgibmag", "fcalogeephase", "fcalogeemag"];

fn cal_fields(c: &mut BaseStationCal) -> [&mut f64; 7] {
	[&mut c.phase, &mut c.tilt, &mut c.curve, &mut c.gibpha, &mut c.gibmag, &mut c.ogeephase, &mut c.ogeemag]
}

impl Config {
	/// config_read_lighthouse: reads lighthouse `idx`'s group into `bsd` if its "index" is `idx` ("assumes that
	/// lighthouses are stored in the config in order"), else returns false and leaves `bsd` alone. Reading sets what is
	/// missing to its default: "index" to 0xffffffff in any group, and when found "id", "mode", the float arrays
	/// (zeros) and the two flags (0). A float array shorter than the field fills only its first values: the rest of the
	/// pose and the variance keep what `bsd` held, the rest of accel and of each calibration field becomes 0. Numbers
	/// are read as config_read_uint32 reads them (the low 32 bits of the entry's union, whatever its type)
	pub fn read_lighthouse(&mut self, idx: u8, bsd: &mut LighthouseRecord) -> bool {
		let cg = &mut self.lighthouses[idx as usize];
		let mut found = false;
		for i in 0..NUM_GEN2_LIGHTHOUSES {
			let index = cg.read_uint32("index", 0xffff_ffff);
			if index == idx as u32 && i == idx as usize {
				found = true;
				break;
			}
		}
		// "assert(found); // throw an assertion if we didn't find it...  Is this good?  not necessarily?"
		if !found {
			return false;
		}
		let defaults = [0.0; 7];
		bsd.base_station_id = cg.read_uint32("id", 0);
		bsd.mode = cg.read_uint32("mode", 0) as u8;

		let p = &mut bsd.pose;
		let mut pose = [p.pos[0], p.pos[1], p.pos[2], p.rot[0], p.rot[1], p.rot[2], p.rot[3]];
		cg.read_float_array("pose", &mut pose, Some(&defaults));
		p.pos.copy_from_slice(&pose[..3]);
		p.rot.copy_from_slice(&pose[3..]);
		cg.read_float_array("variance", &mut bsd.variance, Some(&defaults));

		let mut accel = [0.0; 3];
		cg.read_float_array("accel", &mut accel, Some(&defaults));
		bsd.accel = accel;

		let mut cal = [0.0; 14];
		for (k, tag) in FCAL_TAGS.iter().enumerate() {
			cg.read_float_array(tag, &mut cal[2 * k..2 * k + 2], Some(&defaults));
		}
		for (i, c) in bsd.fcal.iter_mut().enumerate() {
			for (k, field) in cal_fields(c).into_iter().enumerate() {
				*field = cal[2 * k + i];
			}
		}

		bsd.ootx_set = cg.read_uint32("OOTXSet", 0) & 1 != 0;
		bsd.position_set = cg.read_uint32("PositionSet", 0) & 1 != 0;
		true
	}

	/// config_set_lighthouse: writes `bsd` into lighthouse `idx`'s group (the entries it lacks appended in this order:
	/// index, id, mode, pose, variance, unlock_count, accel, the seven fcal fields, OOTXSet, PositionSet). Then, as the
	/// C does after writing the pose, normalizes `bsd`'s rotation unless it is all zero: the file keeps the rotation as
	/// it was given
	pub fn set_lighthouse(&mut self, idx: u8, bsd: &mut LighthouseRecord) {
		let cg = &mut self.lighthouses[idx as usize];
		cg.set_uint32("index", idx as u32);

		cg.set_uint32("id", bsd.base_station_id);
		cg.set_uint32("mode", bsd.mode as u32);
		let p = &bsd.pose;
		cg.set_float_array("pose", &[p.pos[0], p.pos[1], p.pos[2], p.rot[0], p.rot[1], p.rot[2], p.rot[3]]);
		cg.set_float_array("variance", &bsd.variance);
		cg.set_uint32("unlock_count", bsd.sys_unlock_count as u32);
		cg.set_float_array("accel", &bsd.accel);
		if !quatiszero(&bsd.pose.rot) {
			bsd.pose.rot = quatnormalize(&bsd.pose.rot);
		}

		let mut cal = [0.0; 14];
		for (i, c) in bsd.fcal.iter_mut().enumerate() {
			for (k, field) in cal_fields(c).into_iter().enumerate() {
				cal[2 * k + i] = *field;
			}
		}
		for (k, tag) in FCAL_TAGS.iter().enumerate() {
			cg.set_float_array(tag, &cal[2 * k..2 * k + 2]);
		}

		cg.set_uint32("OOTXSet", bsd.ootx_set as u32);
		cg.set_uint32("PositionSet", bsd.position_set as u32);
	}
}
