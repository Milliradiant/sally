//! The solve's model (driver_map_solve.c's "model" section): a snapshot of one device's sweep hits (ms_snap), a station
//! as the solve sees it (ms_station), the angle a hit is predicted at, and the robust (Huber) weight and cost.

use sally_lighthouse::{
	gen2_axis_consts, survive_reproject_axis_gen2_with, BaseStationCal, Gen2AxisConsts, NUM_GEN2_LIGHTHOUSES,
};

use crate::linalg::{mm3, mtv3, mv3, rv2r};

/// MS_MAXH: most hits one snapshot keeps
pub const MS_MAXH: usize = 128;
/// MS_MAXLH: one station per gen2 channel
pub const MS_MAXLH: usize = NUM_GEN2_LIGHTHOUSES;
/// MS_MAXG: six parameters per station
pub const MS_MAXG: usize = 6 * MS_MAXLH;

/// ms_snap: a slow device's raw sweep angles with the filter's pose as a seed only
#[derive(Clone)]
pub struct Snapshot {
	/// the device's index in the map solve's list of devices
	pub dev: i32,
	/// run time when taken
	pub time: f64,
	/// seed: imu2world (row-major R)
	pub r0: [f64; 9],
	pub p0: [f64; 3],
	/// filter velocity (world)
	pub v: [f64; 3],
	pub w: [f64; 3],
	/// the hits: station index, plane, angle, time from the pose (s), the sensor's place on the device
	pub n: usize,
	pub lh: [u8; MS_MAXH],
	pub plane: [i8; MS_MAXH],
	pub angle: [f64; MS_MAXH],
	pub dt: [f64; MS_MAXH],
	pub pt: [[f64; 3]; MS_MAXH],
}

impl Default for Snapshot {
	fn default() -> Self {
		Snapshot {
			dev: 0,
			time: 0.0,
			r0: [0.0; 9],
			p0: [0.0; 3],
			v: [0.0; 3],
			w: [0.0; 3],
			n: 0,
			lh: [0; MS_MAXH],
			plane: [0; MS_MAXH],
			angle: [0.0; MS_MAXH],
			dt: [0.0; MS_MAXH],
			pt: [[0.0; 3]; MS_MAXH],
		}
	}
}

/// ms_station: a station in the solve
#[derive(Clone, Copy, Debug)]
pub struct Station {
	/// lh2world
	pub r: [f64; 9],
	pub p: [f64; 3],
	/// set with `set_fcal`, which keeps `k` with it
	pub fcal: [BaseStationCal; 2],
	/// sally: the reprojection's terms of each sweep plane that are fcal's alone (gen2_axis_consts), worked out once
	/// for the station rather than in every model_angle on it
	pub k: [Gen2AxisConsts; 2],
	pub used: bool,
	/// weight of the weak prior to the calibrated map (1; tiny for a relocated station, which no longer anchors)
	pub prior_w: f64,
	/// first parameter index, -1 = fixed
	pub g: i32,
}

impl Station {
	/// The station's calibration, and its planes' terms with it
	pub fn set_fcal(&mut self, fcal: [BaseStationCal; 2]) {
		self.fcal = fcal;
		self.k = [gen2_axis_consts(&fcal, 0), gen2_axis_consts(&fcal, 1)];
	}
}

impl Default for Station {
	/// All zero, as the C's memset station: fcal too, and so k of the zero calibration
	fn default() -> Station {
		let mut st = Station {
			r: [0.0; 9],
			p: [0.0; 3],
			fcal: [BaseStationCal::default(); 2],
			k: [Gen2AxisConsts::default(); 2],
			used: false,
			prior_w: 0.0,
			g: 0,
		};
		st.set_fcal(st.fcal);
		st
	}
}

/// model_angle: the angle predicted for hit h of snapshot s with snapshot pose (rs, ps) and station st
pub fn model_angle(s: &Snapshot, h: usize, rs: &[f64; 9], ps: &[f64; 3], st: &Station) -> f64 {
	let mut x = mv3(rs, &s.pt[h]);
	let dt = s.dt[h];
	let rw = [s.w[0] * dt, s.w[1] * dt, s.w[2] * dt];
	if rw[0] != 0.0 || rw[1] != 0.0 || rw[2] != 0.0 {
		let th2 = rw[0] * rw[0] + rw[1] * rw[1] + rw[2] * rw[2];
		if th2 < 1e-4 {
			// Rodrigues on the vector itself with the series of sin(t)/t and (1 - cos t)/t^2: exact to rounding for
			// t < 0.01 rad (a snapshot turns at most wmax over a window, ~3 mrad), and no sin/cos per hit per
			// iteration (rv2R here was ~4 % of the runtime's CPU)
			let sa = 1.0 - th2 / 6.0 * (1.0 - th2 / 20.0);
			let cb = 0.5 - th2 / 24.0 * (1.0 - th2 / 30.0);
			let c = [rw[1] * x[2] - rw[2] * x[1], rw[2] * x[0] - rw[0] * x[2], rw[0] * x[1] - rw[1] * x[0]];
			let cc = [rw[1] * c[2] - rw[2] * c[1], rw[2] * c[0] - rw[0] * c[2], rw[0] * c[1] - rw[1] * c[0]];
			for k in 0..3 {
				x[k] += sa * c[k] + cb * cc[k];
			}
		} else {
			let rm = rv2r(&rw);
			x = mv3(&rm, &x);
		}
	}
	let mut w = [0.0; 3];
	for k in 0..3 {
		w[k] = x[k] + ps[k] + s.v[k] * dt;
	}
	let d = [w[0] - st.p[0], w[1] - st.p[1], w[2] - st.p[2]];
	let loc = mtv3(&st.r, &d);
	// survive_reproject_axis_x_gen2 for plane 0, the y variant otherwise, each on the station's fcal ARRAY (the y
	// variant reads bcal[1] itself), with the plane's terms the station keeps (k): the same bits
	let plane = (s.plane[h] != 0) as usize;
	survive_reproject_axis_gen2_with(&st.fcal, &st.k[plane], plane, &loc)
}

/// apply_delta: a pose moved by d (the position by d[0..3], the rotation by the world-left rotation vector d[3..6])
pub fn apply_delta(r0: &[f64; 9], p0: &[f64; 3], d: &[f64]) -> ([f64; 9], [f64; 3]) {
	let dr = rv2r(&[d[3], d[4], d[5]]);
	let r = mm3(&dr, r0);
	let mut p = [0.0; 3];
	for k in 0..3 {
		p[k] = p0[k] + d[k];
	}
	(r, p)
}

/// sally: the six poses a forward-difference Jacobian takes at d: apply_delta(r0, p0, d with 1e-6 added to d[k]) for
/// k = 0..6, the C's loop body, worked out once for all the hits evaluated at them (the C works them out again for
/// every hit: the same values, so the same bits)
pub fn stepped(r0: &[f64; 9], p0: &[f64; 3], d: &[f64]) -> [([f64; 9], [f64; 3]); 6] {
	std::array::from_fn(|k| {
		let mut dk = [0.0; 6];
		dk.copy_from_slice(d);
		dk[k] += 1e-6;
		apply_delta(r0, p0, &dk)
	})
}

/// `stepped` for a station: st with each of the six poses (the rest of it as it is)
pub fn stepped_station(st: &Station, r0: &[f64; 9], p0: &[f64; 3], d: &[f64]) -> [Station; 6] {
	let poses = stepped(r0, p0, d);
	std::array::from_fn(|k| {
		let mut sp = *st;
		(sp.r, sp.p) = poses[k];
		sp
	})
}

/// huber_w: the robust weight of a residual
#[inline]
pub fn huber_w(r: f64, c: f64) -> f64 {
	let a = r.abs();
	if a <= c {
		1.
	} else {
		c / a
	}
}

/// huber_rho: the robust cost of a residual
#[inline]
pub fn huber_rho(r: f64, c: f64) -> f64 {
	let a = r.abs();
	if a <= c {
		r * r
	} else {
		2.0 * c * a - c * c
	}
}
