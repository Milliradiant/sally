//! Where a point in a station's frame appears on each of an LH2 station's two sweep planes, with the station's
//! calibration (libsurvive's survive_reproject_gen2.c); and the plane a sweep angle stands for

use crate::BaseStationCal;
use sally_math::{linmath_enforce_range, LINMATHPI, LINMATHPI_2};

// a_i = a_(i-1) * s + m[i-1]; a[0] = 0
// m_i = m_(i-1) * s + f[i]; m[0] = f[0]
fn calc_cal_series(s: f64) -> (f64, f64) {
	// four initialisers in the C, the last two elements zero; the C's literals as written
	#[allow(clippy::excessive_precision)]
	const F: [f64; 6] = [-8.0108022e-06, 0.0028679863, 5.3685255000000001e-06, 0.0076069798000000001, 0.0, 0.0];
	let mut m = F[0];
	let mut a = 0.0;
	for i in 1..6 {
		a = a * s + m;
		m = m * s + F[i];
	}
	(m, a)
}

/// The terms of a sweep plane's reprojection that are the station's alone (the plane's tilt, ydeg: its tan, sin and
/// cos), worked out once for the many points reprojected on one plane: `survive_reproject_axis_gen2_with` on them
/// gives the plain call's bits (the same operations on the same values; only when they are done moves)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Gen2AxisConsts {
	pub tan_a: f64,
	pub sin_ydeg: f64,
	pub cos_ydeg: f64,
}

fn axis_consts(bcal: &BaseStationCal, axis: bool) -> Gen2AxisConsts {
	// (axis ? -1 : 1) * LINMATHPI / 6.: the int converted, then the product, then the quotient
	let ydeg = bcal.tilt + (if axis { -1.0 } else { 1.0 }) * LINMATHPI / 6.0;
	Gen2AxisConsts { tan_a: ydeg.tan(), sin_ydeg: ydeg.sin(), cos_ydeg: ydeg.cos() }
}

/// The consts of sweep plane `plane` (0: x, 1: y) of a station's calibration
pub fn gen2_axis_consts(bcal: &[BaseStationCal; 2], plane: usize) -> Gen2AxisConsts {
	axis_consts(&bcal[plane], plane == 1)
}

/// `REPROJECT_AXIS_FN_GEN2[plane](bcal, pt_in_lh)` with the plane's consts (`gen2_axis_consts(bcal, plane)`) given:
/// the same bits
pub fn survive_reproject_axis_gen2_with(
	bcal: &[BaseStationCal; 2], k: &Gen2AxisConsts, plane: usize, pt_in_lh: &[f64; 3],
) -> f64 {
	axis_gen2(&bcal[plane], k, pt_in_lh[0], pt_in_lh[1], -pt_in_lh[2])
}

fn survive_reproject_axis_gen2(bcal: &BaseStationCal, x: f64, y: f64, z: f64, axis: bool) -> f64 {
	axis_gen2(bcal, &axis_consts(bcal, axis), x, y, z)
}

fn axis_gen2(bcal: &BaseStationCal, k: &Gen2AxisConsts, x: f64, y: f64, z: f64) -> f64 {
	let phase = bcal.phase;
	let curve = bcal.curve;
	let gib_phase = bcal.gibpha;
	let gib_mag = bcal.gibmag;
	let ogee_phase = bcal.ogeephase;
	let ogee_mag = bcal.ogeemag;

	let b = z.atan2(x);

	let tan_a = k.tan_a;
	let norm_xz = (x * x + z * z).sqrt();

	let asin_arg = tan_a * y / norm_xz;
	let asin_arg_sanitized = linmath_enforce_range(asin_arg, -1.0, 1.0);

	let sin_ydeg = k.sin_ydeg;
	let cos_ydeg = k.cos_ydeg;

	let sin_part = (b - asin_arg_sanitized.asin() + ogee_phase).sin() * ogee_mag;

	let norm_xyz = (x * x + y * y + z * z).sqrt();

	let mod_asin_arg = linmath_enforce_range(y / norm_xyz / cos_ydeg, -1.0, 1.0);

	let asin_out = mod_asin_arg.asin();

	let (mod_, acc) = calc_cal_series(asin_out);

	let bcal_curved = sin_part + curve;
	let asin_arg2 =
		linmath_enforce_range(asin_arg + mod_ * bcal_curved / (cos_ydeg - acc * bcal_curved * sin_ydeg), -1.0, 1.0);

	let asin_out2 = asin_arg2.asin();
	let sin_out2 = (b - asin_out2 + gib_phase).sin();

	b - asin_out2 + sin_out2 * gib_mag - phase - LINMATHPI_2
}

pub fn survive_reproject_axis_x_gen2(bcal: &[BaseStationCal; 2], pt_in_lh: &[f64; 3]) -> f64 {
	survive_reproject_axis_gen2(&bcal[0], pt_in_lh[0], pt_in_lh[1], -pt_in_lh[2], false)
}

pub fn survive_reproject_axis_y_gen2(bcal: &[BaseStationCal; 2], pt_in_lh: &[f64; 3]) -> f64 {
	survive_reproject_axis_gen2(&bcal[1], pt_in_lh[0], pt_in_lh[1], -pt_in_lh[2], true)
}

/// survive_reproject_gen2_model.reprojectAxisFn, indexed by the sweep plane
pub type ReprojectAxisFn = fn(&[BaseStationCal; 2], &[f64; 3]) -> f64;
pub const REPROJECT_AXIS_FN_GEN2: [ReprojectAxisFn; 2] = [survive_reproject_axis_x_gen2, survive_reproject_axis_y_gen2];

/// The normal of the sweep plane a station's angle stands for: the plane equation of the cold start and the seed solve
/// (libsurvive's survive_fill_m). lh_version: 1 once the system is known to be LH2 (before that, no plane: zeros)
#[inline]
pub fn bc_fill_m(lh_version: i32, axis: i32, angle: f64) -> [f64; 3] {
	let sv = angle.sin();
	let cv = angle.cos();
	if lh_version == 1 {
		let tan30 = 0.57735026919;
		[cv, if axis == 0 { -tan30 } else { tan30 }, -sv]
	} else {
		[0.0, 0.0, 0.0]
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn reprojection_ideal_station() {
		// an ideal station (no calibration): a point straight ahead (-z in the station's frame) is at angle 0 on both
		// planes
		let bcal = [BaseStationCal::default(); 2];
		let ax = survive_reproject_axis_x_gen2(&bcal, &[0.0, 0.0, -2.0]);
		let ay = survive_reproject_axis_y_gen2(&bcal, &[0.0, 0.0, -2.0]);
		assert!(ax.abs() < 1e-12, "{ax}");
		assert!((ay - ax).abs() < 1e-15);
		// above the axis the two planes part, symmetrically
		let ax = survive_reproject_axis_x_gen2(&bcal, &[0.0, 0.5, -2.0]);
		let ay = survive_reproject_axis_y_gen2(&bcal, &[0.0, 0.5, -2.0]);
		assert!((ax + ay).abs() < 1e-12 && ax.abs() > 0.1);
	}

	/// survive_reproject_axis_gen2 as it was before its station terms were split out (the C's order, all in one)
	fn as_it_was(bcal: &BaseStationCal, x: f64, y: f64, z: f64, axis: bool) -> f64 {
		let b = z.atan2(x);
		let ydeg = bcal.tilt + (if axis { -1.0 } else { 1.0 }) * LINMATHPI / 6.0;
		let tan_a = ydeg.tan();
		let norm_xz = (x * x + z * z).sqrt();
		let asin_arg = tan_a * y / norm_xz;
		let asin_arg_sanitized = linmath_enforce_range(asin_arg, -1.0, 1.0);
		let sin_ydeg = ydeg.sin();
		let cos_ydeg = ydeg.cos();
		let sin_part = (b - asin_arg_sanitized.asin() + bcal.ogeephase).sin() * bcal.ogeemag;
		let norm_xyz = (x * x + y * y + z * z).sqrt();
		let mod_asin_arg = linmath_enforce_range(y / norm_xyz / cos_ydeg, -1.0, 1.0);
		let asin_out = mod_asin_arg.asin();
		let (mod_, acc) = calc_cal_series(asin_out);
		let bcal_curved = sin_part + bcal.curve;
		let asin_arg2 =
			linmath_enforce_range(asin_arg + mod_ * bcal_curved / (cos_ydeg - acc * bcal_curved * sin_ydeg), -1.0, 1.0);
		let asin_out2 = asin_arg2.asin();
		let sin_out2 = (b - asin_out2 + bcal.gibpha).sin();
		b - asin_out2 + sin_out2 * bcal.gibmag - bcal.phase - LINMATHPI_2
	}

	#[test]
	fn reprojection_with_the_station_terms_split_out_is_bit_for_bit_the_same() {
		// calibrations and points from a fixed xorshift: in range, and any bit pattern (NaN, inf, huge)
		let mut r = 0x9e37_79b9_7f4a_7c15u64;
		let mut next = move || {
			r ^= r << 13;
			r ^= r >> 7;
			r ^= r << 17;
			r
		};
		let mut unit = move || (next() >> 11) as f64 / (1u64 << 53) as f64;
		let mut cases = 0;
		for i in 0..20000 {
			let mut v = [0.0f64; 14];
			for x in v.iter_mut() {
				*x = if i % 10 == 9 { f64::from_bits(next_bits(&mut unit)) } else { (unit() - 0.5) * 0.2 };
			}
			let c = |o: usize| BaseStationCal {
				phase: v[o],
				tilt: v[o + 1],
				curve: v[o + 2],
				gibpha: v[o + 3] * 30.0,
				gibmag: v[o + 4],
				ogeephase: v[o + 5] * 30.0,
				ogeemag: v[o + 6],
			};
			let bcal = [c(0), c(7)];
			let k = [gen2_axis_consts(&bcal, 0), gen2_axis_consts(&bcal, 1)];
			for _ in 0..8 {
				let pt = [(unit() - 0.5) * 8.0, (unit() - 0.5) * 6.0, -unit() * 8.0];
				for plane in 0..2 {
					let want = as_it_was(&bcal[plane], pt[0], pt[1], -pt[2], plane == 1);
					let plain = REPROJECT_AXIS_FN_GEN2[plane](&bcal, &pt);
					let with = survive_reproject_axis_gen2_with(&bcal, &k[plane], plane, &pt);
					assert_eq!(plain.to_bits(), want.to_bits(), "{bcal:?} {pt:?} {plane}");
					assert_eq!(with.to_bits(), want.to_bits(), "{bcal:?} {pt:?} {plane}");
					cases += 1;
				}
			}
		}
		assert_eq!(cases, 320000);
	}

	/// Any 64 bits, from two draws
	fn next_bits(unit: &mut impl FnMut() -> f64) -> u64 {
		((unit() * 4294967296.0) as u64) << 32 | (unit() * 4294967296.0) as u64
	}
}
