//! `sally calibrate`'s solve: a room's base stations placed from nothing but a recording, every device solving them
//! together, and the map levelled with the devices' accelerometers (docs/sally_design.md 4.2, "Calibration").
//!
//! 1. **Still moments.** Per device, every `step` s, a window of `window` s in which its IMU says it holds still (the
//!    gyro under `still_gyro`, each accelerometer axis steady to `still_acc_sd`): its hits then come from one pose.
//! 2. **One station's view.** In each still window, each station that gives the device enough hits in both planes
//!    places the device in that station's frame alone: the cold start (barycentric SVD) as a first guess, then a
//!    robust fit of the hits with the full gen2 model, the factory calibration included. Hits on sensors facing away
//!    from the station, or far off the fit, are left out (reflections).
//! 3. **Station pairs.** A window two stations both place gives one estimate of where one station sits from the
//!    other. Each pair's estimates are combined robustly (medians), and the stations are joined from the best-linked
//!    one along the pairs with the most windows (a maximum spanning tree).
//! 4. **The joint solve.** Every window seen by two stations becomes a snapshot, and the map solve's own joint
//!    Levenberg-Marquardt (`solve::joint_solve`: every snapshot pose and every station pose at once, robust) refines
//!    the placement in three rounds, each rejecting hits more tightly. The first station is held (the solve's frame);
//!    the scale is the devices' sensor layouts'.
//! 5. **Gravity.** At rest a device's accelerometer reads up, turned into its own frame, plus its own bias: f = R'
//!    (g up) + b. Over the still windows of many headings the world-fixed up and each device's body-fixed bias come
//!    apart (a device that only turns about the vertical still separates them: the tilt turns with it, the bias does
//!    not). Solved jointly, linearly, over all devices.
//! 6. **The frame.** Turned so up is +z (the smallest turn, so the first station's heading stays), the play area's
//!    middle at x = y = 0 and the floor at z = 0 (the lowest device 3 cm above it, as the panel's Set floor).
//!
//! Every station's pose comes out in that frame, with what the solve saw (snapshots, links between stations, the
//! light's residual per station, the tilt each device alone would give) as report lines.

use sally_lighthouse::{gen2_axis_consts, survive_reproject_axis_gen2_with, BaseStationCal, Gen2AxisConsts};
use sally_math::Pose;
use sally_track::cold_start::BarycentricSvd;

use crate::fcal_solve::solve_fcal;
use crate::linalg::{gsolve, mm3, mtv3, mv3, r2q, r2rv, rv2r, transpose3};
use crate::model::{apply_delta, stepped, Snapshot, Station, MS_MAXG, MS_MAXH, MS_MAXLH};
use crate::settings::{Settings, Source};
use crate::solve::{joint_solve, median_of, warm_start};

/// Standard gravity (m/s^2), as the tracker takes it
pub const GRAVITY: f64 = 9.80665;

/// One sweep hit: the device's long clock, the station (bsd index), the sensor, the plane, the angle
#[derive(Clone, Copy, Debug, Default)]
pub struct Hit {
	pub tc: u64,
	pub lh: u8,
	pub sensor: u8,
	pub plane: i8,
	pub angle: f64,
}

/// One calibrated IMU sample: the device's long clock, the accelerometer (g), the gyro (rad/s), in its IMU frame
#[derive(Clone, Copy, Debug, Default)]
pub struct Imu {
	pub tc: u64,
	pub acc: [f64; 3],
	pub gyro: [f64; 3],
}

/// A device as the recording had it: its name, its clock and its sensor layout (IMU frame)
#[derive(Clone, Debug, Default)]
pub struct Device {
	pub name: String,
	/// its clock's ticks per second
	pub hz: f64,
	pub sensors: Vec<[f64; 3]>,
	pub normals: Vec<[f64; 3]>,
}

/// A still moment of a device (step 1): when it ends on the device's clock, its mean accelerometer (g, its frame),
/// and the newest hit per station, sensor and plane in it
#[derive(Clone, Debug, Default)]
pub struct Moment {
	pub dev: usize,
	pub tc: u64,
	pub acc: [f64; 3],
	pub hits: Vec<Hit>,
}

/// A station as the recording had it: its bsd index, channel, id and factory calibration (OOTX, with any corrections)
#[derive(Clone, Copy, Debug)]
pub struct StationIn {
	pub lh: usize,
	pub channel: u8,
	pub id: u32,
	pub fcal: [BaseStationCal; 2],
}

/// The calibration's settings
#[derive(Clone, Debug)]
pub struct Options {
	/// a still window's length (s): about two LH2 rotor turns
	pub window: f64,
	/// at most one still window per device every this many s
	pub step: f64,
	/// still: the gyro under this (rad/s) over the window and 50 ms either side
	pub still_gyro: f64,
	/// ... and each accelerometer axis within this sd (g)
	pub still_acc_sd: f64,
	/// hits one station must give a window (after the reflection checks) to place the device
	pub min_view_hits: usize,
	/// a station's view of a window is kept when the median of its fit's residuals is under this (rad)
	pub max_view_res: f64,
	/// the joint solve takes at most this many snapshots (spread evenly over each device's windows)
	pub max_snaps: usize,
	/// a hit counts when its sensor faces the station to within this angle (deg): the runtime's light-facing-deg
	pub facing_deg: f64,
	/// no calibration when up is known less well than this (rad, one sd): a frame tilted off gravity misleads the
	/// tracker
	pub max_up_sd: f64,
	/// the stations' corrections solved with them (fcal_solve); off: the calibrations given are kept as they are
	pub corrections: bool,
	/// the room's calibration so far, per station id its lh2world: when two or more of the placed stations are in it,
	/// the new frame is tied to it (levelled to gravity, then turned about the vertical and moved to fit them), so a
	/// new calibration leaves the room where the old one had it
	pub anchors: Vec<(u32, Pose)>,
}

impl Default for Options {
	fn default() -> Options {
		Options {
			window: 0.045,
			step: 0.25,
			still_gyro: 0.08,
			still_acc_sd: 0.006,
			min_view_hits: 8,
			max_view_res: 0.002,
			max_snaps: 2500,
			facing_deg: 115.0,
			max_up_sd: 0.008,
			corrections: true,
			anchors: Vec::new(),
		}
	}
}

/// A placed station
#[derive(Clone, Copy, Debug)]
pub struct Placed {
	pub lh: usize,
	pub channel: u8,
	pub id: u32,
	/// lh2world, in the calibration's frame
	pub pose: Pose,
	/// the hits of the joint solve on it, and the median of their residuals (rad)
	pub hits: usize,
	pub med_res: f64,
	/// the correction of its factory calibration's ten terms (phase, tilt, curve, gibbous phase and magnitude, each
	/// axis 0 then 1; fcal_solve), from the calibration it was given; zero when the corrections were not solved
	pub correction: [f64; 10],
}

/// What a device's accelerometer said about up, on its own and in the joint solve
#[derive(Clone, Debug)]
pub struct DeviceGravity {
	pub name: String,
	pub windows: usize,
	/// the headings it was still at cover all but this largest gap (deg)
	pub largest_gap_deg: f64,
	/// up as this device alone puts it, in the solve's frame (before levelling): tilt (rad) and azimuth (deg from +x
	/// toward +y), None when its headings do not separate tilt and bias
	pub alone: Option<(f64, f64)>,
	/// its accelerometer bias (m/s^2, its frame)
	pub bias: [f64; 3],
}

/// The calibration
#[derive(Clone, Debug)]
pub struct Calibration {
	pub stations: Vec<Placed>,
	/// up in the solve's frame (the first station's, before levelling) as tilt (rad) and azimuth (deg)
	pub up_tilt: (f64, f64),
	/// how well up is known (rad, one sd)
	pub up_sd: f64,
	/// the levelling and the move to the room's middle and floor: final = frame * solve's frame
	pub frame: Pose,
	pub devices: Vec<DeviceGravity>,
	pub snapshots: usize,
	/// the median |residual| of the joint solve's hits (rad)
	pub med_res: f64,
	/// the report, line by line
	pub lines: Vec<String>,
}

/// One station's view of a still window: the device in the station's frame (dev2lh) and the hits that fit it
#[derive(Clone, Debug)]
struct View {
	lh: usize,
	r: [f64; 9],
	p: [f64; 3],
	/// indices into the window's hits
	hits: Vec<usize>,
}

/// A still moment and the stations' views of it
#[derive(Clone, Debug)]
struct Window {
	m: Moment,
	views: Vec<View>,
}

/// The gen2 sweep planes' constant terms of each station
fn station_k(fcal: &[BaseStationCal; 2]) -> [Gen2AxisConsts; 2] {
	[gen2_axis_consts(fcal, 0), gen2_axis_consts(fcal, 1)]
}

/// The angle a sensor at pt (device frame) is seen at by a station, the device at (r, p) in the station's frame
fn predict(st: &StationIn, k: &[Gen2AxisConsts; 2], plane: i8, r: &[f64; 9], p: &[f64; 3], pt: &[f64; 3]) -> f64 {
	let x = mv3(r, pt);
	let loc = [x[0] + p[0], x[1] + p[1], x[2] + p[2]];
	let pl = (plane != 0) as usize;
	survive_reproject_axis_gen2_with(&st.fcal, &k[pl], pl, &loc)
}

/// Whether a sensor faces the station (at the origin of the frame the device is posed in) within cos_max
fn facing(dev: &Device, sensor: usize, r: &[f64; 9], p: &[f64; 3], cos_max: f64) -> bool {
	let x = mv3(r, &dev.sensors[sensor]);
	let n = mv3(r, &dev.normals[sensor]);
	let to = [-(x[0] + p[0]), -(x[1] + p[1]), -(x[2] + p[2])];
	let d = (to[0] * to[0] + to[1] * to[1] + to[2] * to[2]).sqrt();
	d > 0.0 && (n[0] * to[0] + n[1] * to[1] + n[2] * to[2]) / d > cos_max
}

/// A device seen by one station: what fitting its view needs
struct Sight<'a> {
	dev: &'a Device,
	st: &'a StationIn,
	k: &'a [Gen2AxisConsts; 2],
}

impl Sight<'_> {
	/// A hit's angle less the angle predicted with the device at (r, p) in the station's frame
	fn residual(&self, h: &Hit, r: &[f64; 9], p: &[f64; 3]) -> f64 {
		h.angle - predict(self.st, self.k, h.plane, r, p, &self.dev.sensors[h.sensor as usize])
	}
}

/// Gauss-Newton on one station's hits (Huber at 1 mrad): the device's pose in that station's frame
fn fit_view(s: &Sight, hits: &[Hit], use_: &[usize], r: &mut [f64; 9], p: &mut [f64; 3], iters: usize) {
	const HUBER: f64 = 0.001;
	for _ in 0..iters {
		let mut jtj = [0.0; 36];
		let mut jtr = [0.0; 6];
		let sp6 = stepped(r, p, &[0.0; 6]);
		for &i in use_ {
			let h = &hits[i];
			let res = s.residual(h, r, p);
			let mut j = [0.0; 6];
			for (c, (r2, p2)) in sp6.iter().enumerate() {
				j[c] = (s.residual(h, r2, p2) - res) / 1e-6;
			}
			let w = if res.abs() <= HUBER { 1.0 } else { HUBER / res.abs() };
			for a in 0..6 {
				jtr[a] -= w * j[a] * res;
				for c in 0..6 {
					jtj[a * 6 + c] += w * j[a] * j[c];
				}
			}
		}
		for a in 0..6 {
			jtj[a * 7] += 1e-9 + 1e-4 * jtj[a * 7];
		}
		if !gsolve(&mut jtj, &mut jtr, 6) {
			return;
		}
		let (r2, p2) = apply_delta(r, p, &jtr);
		*r = r2;
		*p = p2;
		if jtr.iter().map(|v| v.abs()).fold(0.0, f64::max) < 1e-7 {
			return;
		}
	}
}

/// The device placed by one station from the window's hits on it (`idx`), or None
fn view(bc: &mut BarycentricSvd, s: &Sight, hits: &[Hit], idx: &[usize], opt: &Options) -> Option<View> {
	let (dev, st) = (s.dev, s.st);
	bc.reset();
	for &i in idx {
		let h = &hits[i];
		bc.add(h.sensor as usize, h.plane as i32, h.angle + st.fcal[(h.plane != 0) as usize].phase);
	}
	if bc.meas_cnt() < opt.min_view_hits {
		return None;
	}
	let mut rb = [[0.0; 3]; 3];
	let mut t = [0.0; 3];
	let err = bc.compute_pose(1, &mut rb, &mut t);
	if !(0.0..=1.0).contains(&err) {
		return None;
	}
	// the solver's frame looks along +z, the station's along -z: a half turn about y (x and z flipped)
	let mut r = [-rb[0][0], -rb[0][1], -rb[0][2], rb[1][0], rb[1][1], rb[1][2], -rb[2][0], -rb[2][1], -rb[2][2]];
	let mut p = [-t[0], t[1], -t[2]];
	if !r.iter().chain(p.iter()).all(|v| v.is_finite()) {
		return None;
	}
	let cos_max = (opt.facing_deg.to_radians()).cos();
	let mut use_: Vec<usize> = idx.to_vec();
	fit_view(s, hits, &use_, &mut r, &mut p, 10);
	// reflections and stray hits out: facing away, or far off the fit
	use_.retain(|&i| {
		let h = &hits[i];
		facing(dev, h.sensor as usize, &r, &p, cos_max) && s.residual(h, &r, &p).abs() < 0.004
	});
	if use_.len() < opt.min_view_hits {
		return None;
	}
	fit_view(s, hits, &use_, &mut r, &mut p, 10);
	let mut res: Vec<f64> = use_.iter().map(|&i| s.residual(&hits[i], &r, &p).abs()).collect();
	use_ = use_.into_iter().zip(res.iter()).filter(|(_, &e)| e < 0.003).map(|(i, _)| i).collect();
	let planes = [0i8, 1].map(|pl| use_.iter().any(|&i| hits[i].plane == pl));
	let dist = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
	if use_.len() < opt.min_view_hits
		|| !planes[0]
		|| !planes[1]
		|| median_of(&mut res) > opt.max_view_res
		|| p[2] > -0.1
		|| !(0.3..=15.0).contains(&dist)
	{
		return None;
	}
	Some(View { lh: st.lh, r, p, hits: use_ })
}

/// Step 1, as the samples come: a device's still moments, from its IMU samples and hits fed in the order they arrive
/// (light can come up to `margin` late). Every `step` s a window of `window` s is looked at once the IMU has passed
/// its end by `margin`: still when the gyro stays under `still_gyro` and each accelerometer axis within
/// `still_acc_sd` over the window and `margin` either side; a moving device is looked at again 20 ms later
pub struct StillFinder {
	dev: usize,
	hz: f64,
	opt: Options,
	imu: std::collections::VecDeque<Imu>,
	hits: std::collections::VecDeque<Hit>,
	/// the end of the next window to look at (device clock), 0 until the first sample
	next: u64,
	/// moments found, and windows looked at
	pub found: usize,
	pub looked: usize,
}

/// The samples kept either side of a window (s)
const MARGIN: f64 = 0.05;

impl StillFinder {
	pub fn new(dev: usize, d: &Device, opt: &Options) -> StillFinder {
		StillFinder {
			dev,
			hz: d.hz,
			opt: opt.clone(),
			imu: Default::default(),
			hits: Default::default(),
			next: 0,
			found: 0,
			looked: 0,
		}
	}

	fn ticks(&self, s: f64) -> u64 {
		(s * self.hz) as u64
	}

	/// A hit (its timecode on the device's long clock; its sensor is checked against the layout later)
	pub fn hit(&mut self, h: Hit) {
		self.hits.push_back(h);
	}

	/// An IMU sample: the moment it completes, if one
	pub fn imu(&mut self, x: Imu) -> Option<Moment> {
		if self.hz <= 0.0 {
			return None;
		}
		let (w, margin) = (self.ticks(self.opt.window), self.ticks(MARGIN));
		if self.next == 0 {
			self.next = x.tc + w + margin;
		}
		self.imu.push_back(x);
		let mut out = None;
		if x.tc >= self.next + margin {
			out = self.look(self.next);
			self.next += if out.is_some() { self.ticks(self.opt.step) } else { self.ticks(0.02).max(1) };
			if self.next + margin < x.tc {
				// a gap in the samples: on from here
				self.next = x.tc;
			}
		}
		// what no later window needs
		let keep = self.next.saturating_sub(w + margin);
		while self.imu.front().is_some_and(|s| s.tc < keep) {
			self.imu.pop_front();
		}
		let keep = self.next.saturating_sub(w).min(x.tc.saturating_sub(self.ticks(0.5)));
		while self.hits.front().is_some_and(|h| h.tc < keep) {
			self.hits.pop_front();
		}
		out
	}

	fn look(&mut self, t: u64) -> Option<Moment> {
		self.looked += 1;
		let (w, margin) = (self.ticks(self.opt.window), self.ticks(MARGIN));
		let lo = t.saturating_sub(w + margin);
		let s: Vec<&Imu> = self.imu.iter().filter(|s| s.tc >= lo && s.tc <= t + margin).collect();
		if s.len() < 10 {
			return None;
		}
		let n = s.len() as f64;
		if !s.iter().all(|x| x.gyro.iter().map(|v| v * v).sum::<f64>().sqrt() < self.opt.still_gyro) {
			return None;
		}
		let mut mean = [0.0; 3];
		for x in &s {
			for k in 0..3 {
				mean[k] += x.acc[k] / n;
			}
		}
		let mut var = [0.0; 3];
		for x in &s {
			for k in 0..3 {
				var[k] += (x.acc[k] - mean[k]) * (x.acc[k] - mean[k]) / n;
			}
		}
		if var.iter().any(|v| v.sqrt() > self.opt.still_acc_sd) {
			return None;
		}
		// the newest hit per (station, sensor, plane) in the window
		let mut seen = std::collections::HashSet::new();
		let mut hits: Vec<Hit> = Vec::new();
		for h in self.hits.iter().rev() {
			if h.tc <= t && h.tc >= t.saturating_sub(w) && seen.insert((h.lh, h.sensor, h.plane)) {
				hits.push(*h);
			}
		}
		if hits.len() < self.opt.min_view_hits {
			return None;
		}
		self.found += 1;
		Some(Moment { dev: self.dev, tc: t, acc: mean, hits })
	}
}

/// The median of each component
fn median3(v: &[[f64; 3]]) -> [f64; 3] {
	let mut out = [0.0; 3];
	for k in 0..3 {
		let mut c: Vec<f64> = v.iter().map(|x| x[k]).collect();
		out[k] = median_of(&mut c);
	}
	out
}

/// A robust mean of rotations: the median rotation vector about the current mean, three times
fn rotation_median(rs: &[[f64; 9]]) -> [f64; 9] {
	let mut m = rs[0];
	for _ in 0..3 {
		let mt = transpose3(&m);
		let d: Vec<[f64; 3]> = rs.iter().map(|r| r2rv(&mm3(r, &mt))).collect();
		m = mm3(&rv2r(&median3(&d)), &m);
	}
	m
}

/// The joint solve's settings: what joint_solve reads, the rest 0
fn solve_settings(reject: f64, iters: i32) -> Settings {
	struct S(f64, i32);
	impl Source for S {
		fn int(&mut self, tag: &'static str) -> i32 {
			match tag {
				"map-solve-iters" => self.1,
				"map-solve-min-station-hits" => 60,
				_ => 0,
			}
		}
		fn float(&mut self, tag: &'static str) -> f64 {
			match tag {
				"map-solve-huber" => 0.001,
				"map-solve-reject" => self.0,
				// the first station's hold (its prior_w is 1, the others' 0)
				"map-solve-prior-pos" => 1e-4,
				"map-solve-prior-rot" => 1e-4,
				_ => 0.0,
			}
		}
	}
	Settings::read_from(&mut S(reject, iters))
}

/// Tilt (rad) and azimuth (deg from +x toward +y) of a direction from +z
pub fn tilt_az(u: &[f64; 3]) -> (f64, f64) {
	let n = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
	let tilt = (u[2] / n).clamp(-1.0, 1.0).acos();
	let mut az = u[1].atan2(u[0]).to_degrees();
	if az < 0.0 {
		az += 360.0;
	}
	(tilt, az)
}

/// The largest gap (deg) between the headings of a set of directions (radians), 360 for one or none
fn largest_gap(mut hd: Vec<f64>) -> f64 {
	if hd.len() < 2 {
		return 360.0;
	}
	hd.sort_by(|a, b| a.total_cmp(b));
	let mut gap = hd[0] + std::f64::consts::TAU - hd[hd.len() - 1];
	for i in 1..hd.len() {
		gap = gap.max(hd[i] - hd[i - 1]);
	}
	gap.to_degrees()
}

/// A row of the gravity solve: the device, its orientation (imu2world, row-major) and its accelerometer (m/s^2)
type Row = (usize, [f64; 9], [f64; 3]);

/// What the gravity solve gives: up, every device's bias, and the variances of up's x and y per unit row variance
type UpFit = ([f64; 3], Vec<[f64; 3]>, [f64; 2]);

/// A snapshot's pose is well enough fixed for gravity with this many hits in the joint solve, fitting this well (rad)
const GRAVITY_MIN_HITS: usize = 12;
const GRAVITY_MAX_RES: f64 = 0.0003;
/// A gravity row is out when its accelerometer is this far (m/s^2) from up and its bias (or 3x the median, if more),
/// and a device is out when its rows' median is
const GRAVITY_ROW_LIMIT: f64 = 0.15;
const GRAVITY_DEVICE_LIMIT: f64 = 0.25;

/// The accelerometer's noise at rest (m/s^2) and the sd of a device's bias as its factory calibration leaves it (most
/// devices 0.08-0.18 m/s^2, docs/sally_panel.md): the bias prior's weight is their ratio squared
const ACC_SD: f64 = 0.05;
const BIAS_SD: f64 = 0.15;

/// Up from the accelerometers (step 5): rows (device, imu2world, accelerometer in m/s^2). Unknowns: up's x and y
/// (z = 1) and every device's bias, which has a weak prior at 0 (so a device that never turned still says something
/// about up, as much as its factory calibration is worth); f - R' (0, 0, g) = R' (g ux, g uy, 0) + b. Returns up
/// (normalized), the biases and the variances of ux and uy per unit row variance, None when singular
fn solve_up(rows: &[Row], ndev: usize) -> Option<UpFit> {
	let n = 2 + 3 * ndev;
	let mut a = vec![0.0; n * n];
	let mut b = vec![0.0; n];
	for (d, r, f) in rows {
		// (R' x)_i = R[0][i] x0 + R[1][i] x1 + R[2][i] x2
		for i in 0..3 {
			let j = [(0usize, GRAVITY * r[i]), (1, GRAVITY * r[3 + i]), (2 + 3 * d + i, 1.0)];
			let y = f[i] - GRAVITY * r[6 + i];
			for &(p, vp) in &j {
				b[p] += vp * y;
				for &(q, vq) in &j {
					a[p * n + q] += vp * vq;
				}
			}
		}
	}
	let prior = (ACC_SD / BIAS_SD) * (ACC_SD / BIAS_SD);
	for p in 2..n {
		a[p * n + p] += prior;
	}
	// the variances of ux and uy: the first two diagonal entries of the inverse
	let mut var = [0.0; 2];
	for (k, v) in var.iter_mut().enumerate() {
		let mut ak = a.clone();
		let mut e = vec![0.0; n];
		e[k] = 1.0;
		if !gsolve(&mut ak, &mut e, n) {
			return None;
		}
		*v = e[k];
	}
	if !gsolve(&mut a, &mut b, n) {
		return None;
	}
	let u = [b[0], b[1], 1.0];
	let un = (u[0] * u[0] + u[1] * u[1] + 1.0).sqrt();
	let biases = (0..ndev).map(|d| [b[2 + 3 * d], b[3 + 3 * d], b[4 + 3 * d]]).collect();
	Some(([u[0] / un, u[1] / un, 1.0 / un], biases, var))
}

/// The rows' accelerometer residuals (m/s^2) against up and the biases
fn up_residuals(rows: &[Row], up: &[f64; 3], b: &[[f64; 3]]) -> Vec<f64> {
	rows.iter()
		.map(|(d, r, f)| {
			let m = mtv3(r, &[GRAVITY * up[0], GRAVITY * up[1], GRAVITY * up[2]]);
			let e = [f[0] - m[0] - b[*d][0], f[1] - m[1] - b[*d][1], f[2] - m[2] - b[*d][2]];
			(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt()
		})
		.collect()
}

/// One row per orientation a device held: a still moment within 5 degrees of one already taken says nothing new, and
/// counting it again would make the gravity solve sure of more than it saw
fn distinct_rows(rows: &[Row]) -> Vec<Row> {
	let cos5 = 5f64.to_radians().cos();
	let mut out: Vec<Row> = Vec::new();
	for r in rows {
		let near = out.iter().any(|k| {
			k.0 == r.0 && {
				let t = mm3(&k.1, &transpose3(&r.1));
				(t[0] + t[4] + t[8] - 1.0) / 2.0 > cos5
			}
		});
		if !near {
			out.push(*r);
		}
	}
	out
}

/// The smallest rotation that turns the direction u onto +z
fn turn_to_z(u: &[f64; 3]) -> [f64; 9] {
	let n = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
	let ax = [u[1] / n, -u[0] / n, 0.0]; // u x z
	let s = (ax[0] * ax[0] + ax[1] * ax[1]).sqrt();
	if s < 1e-15 {
		return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
	}
	let ang = s.atan2(u[2] / n);
	rv2r(&[ax[0] / s * ang, ax[1] / s * ang, 0.0])
}

/// Up and the biases for any tilt: the linear solve is exact only near +z, so it is made in a frame turned to bring
/// the last estimate onto +z, four times, starting from the mean of the turned accelerometers. Also the sd of up's
/// direction (rad), from the solve's covariance and the rows' own scatter
fn solve_up_any(rows: &[Row], ndev: usize) -> Option<([f64; 3], Vec<[f64; 3]>, f64)> {
	let mut m = [0.0; 3];
	for (_, r, f) in rows {
		let w = mv3(r, f);
		for k in 0..3 {
			m[k] += w[k];
		}
	}
	let mut q = turn_to_z(&m);
	let mut biases = Vec::new();
	let mut var = [0.0; 2];
	for _ in 0..4 {
		let turned: Vec<Row> = rows.iter().map(|(d, r, f)| (*d, mm3(&q, r), *f)).collect();
		let (u, b, v) = solve_up(&turned, ndev)?;
		q = mm3(&turn_to_z(&u), &q);
		biases = b;
		var = v;
	}
	let up = mtv3(&q, &[0.0, 0.0, 1.0]);
	let e = up_residuals(rows, &up, &biases);
	let dof = (3 * rows.len()).saturating_sub(2 + 3 * ndev).max(1) as f64;
	let s2 = (e.iter().map(|v| v * v).sum::<f64>() / dof).max(ACC_SD * ACC_SD);
	Some((up, biases, (s2 * (var[0] + var[1])).sqrt()))
}

/// The calibration from the recording's devices, their still moments (`StillFinder`) and the stations
pub fn calibrate(
	devs: &[Device], moments: Vec<Moment>, stations: &[StationIn], opt: &Options,
) -> Result<Calibration, String> {
	let mut lines = Vec::new();
	let ks: Vec<[Gen2AxisConsts; 2]> = stations.iter().map(|s| station_k(&s.fcal)).collect();
	let by_lh = |lh: usize| stations.iter().position(|s| s.lh == lh);

	// 2: each station's view of the still moments
	let mut wins: Vec<Window> = Vec::new();
	let mut counts = vec![[0usize; 3]; devs.len()];
	let mut bcs: Vec<Option<BarycentricSvd>> = devs
		.iter()
		.map(|d| {
			(d.sensors.len() >= 4 && d.normals.len() == d.sensors.len()).then(|| BarycentricSvd::new(d.sensors.clone()))
		})
		.collect();
	for m in moments {
		let Some(dev) = devs.get(m.dev) else { continue };
		let Some(bc) = bcs[m.dev].as_mut() else { continue };
		let mut win = Window { m, views: Vec::new() };
		for (si, st) in stations.iter().enumerate() {
			let hits = &win.m.hits;
			let idx: Vec<usize> = (0..hits.len())
				.filter(|&i| hits[i].lh as usize == st.lh && (hits[i].sensor as usize) < dev.sensors.len())
				.collect();
			if idx.len() < opt.min_view_hits {
				continue;
			}
			if let Some(v) = view(bc, &Sight { dev, st, k: &ks[si] }, hits, &idx, opt) {
				win.views.push(v);
			}
		}
		let c = &mut counts[win.m.dev];
		c[0] += 1;
		c[1] += !win.views.is_empty() as usize;
		c[2] += (win.views.len() >= 2) as usize;
		if !win.views.is_empty() {
			wins.push(win);
		}
	}
	for (di, (d, c)) in devs.iter().zip(&counts).enumerate() {
		if bcs[di].is_none() {
			lines.push(format!("{}: no sensor layout, left out", d.name));
		} else if c[0] == 0 {
			lines.push(format!("{}: no still moments", d.name));
		} else {
			lines.push(format!(
				"{}: {} still moments, {} placed by a station, {} by two or more",
				d.name, c[0], c[1], c[2]
			));
		}
	}
	bcs.clear();

	// 3: station pairs: lhB2lhA = dev2lhA * inv(dev2lhB)
	let ns = stations.len();
	let mut pair_r: Vec<Vec<[f64; 9]>> = vec![Vec::new(); ns * ns];
	let mut pair_p: Vec<Vec<[f64; 3]>> = vec![Vec::new(); ns * ns];
	for win in &wins {
		for va in &win.views {
			for vb in &win.views {
				let (Some(a), Some(b)) = (by_lh(va.lh), by_lh(vb.lh)) else { continue };
				if a == b {
					continue;
				}
				let r = mm3(&va.r, &transpose3(&vb.r));
				let rp = mv3(&r, &vb.p);
				pair_r[a * ns + b].push(r);
				pair_p[a * ns + b].push([va.p[0] - rp[0], va.p[1] - rp[1], va.p[2] - rp[2]]);
			}
		}
	}
	let count = |a: usize, b: usize| pair_r[a * ns + b].len();
	// the root: the most links
	let root = (0..ns).max_by_key(|&a| (0..ns).map(|b| count(a, b)).sum::<usize>()).ok_or("no stations")?;
	if (0..ns).map(|b| count(root, b)).sum::<usize>() == 0 {
		return Err("no still moment was seen by two stations: the stations cannot be placed".into());
	}
	// lh2world (the root's frame) of each placed station, along the maximum spanning tree
	let mut placed: Vec<Option<([f64; 9], [f64; 3])>> = vec![None; ns];
	placed[root] = Some(([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0; 3]));
	loop {
		let mut best: Option<(usize, usize, usize)> = None;
		for a in 0..ns {
			if placed[a].is_none() {
				continue;
			}
			for b in 0..ns {
				if placed[b].is_some() || count(a, b) < 5 {
					continue;
				}
				if best.is_none_or(|(_, _, c)| count(a, b) > c) {
					best = Some((a, b, count(a, b)));
				}
			}
		}
		let Some((a, b, c)) = best else { break };
		// lhB2lhA, robustly
		let r_ab = rotation_median(&pair_r[a * ns + b]);
		let p_ab = median3(&pair_p[a * ns + b]);
		let (ra, pa) = placed[a].expect("placed");
		let r = mm3(&ra, &r_ab);
		let rp = mv3(&ra, &p_ab);
		placed[b] = Some((r, [rp[0] + pa[0], rp[1] + pa[1], rp[2] + pa[2]]));
		lines.push(format!(
			"start: ch {} placed from ch {} ({} windows, {:.0} mm apart)",
			stations[b].channel,
			stations[a].channel,
			c,
			(p_ab[0] * p_ab[0] + p_ab[1] * p_ab[1] + p_ab[2] * p_ab[2]).sqrt() * 1e3
		));
	}
	for (s, p) in stations.iter().zip(&placed) {
		if p.is_none() {
			lines.push(format!("ch {}: never seen in a still moment with a placed station, left out", s.channel));
		}
	}

	// 4: snapshots (the views' hits, which passed the reflection checks), and the joint solve
	let mut snaps: Vec<Snapshot> = Vec::new();
	let mut snap_win: Vec<usize> = Vec::new();
	let mut per_dev: Vec<Vec<usize>> = vec![Vec::new(); devs.len()];
	for (wi, win) in wins.iter().enumerate() {
		let views: Vec<&View> = win.views.iter().filter(|v| by_lh(v.lh).is_some_and(|s| placed[s].is_some())).collect();
		if views.len() >= 2 {
			per_dev[win.m.dev].push(wi);
		}
	}
	let total: usize = per_dev.iter().map(|v| v.len()).sum();
	let keep = if total > opt.max_snaps { opt.max_snaps as f64 / total as f64 } else { 1.0 };
	for list in &per_dev {
		let mut acc = 0.0;
		for &wi in list {
			acc += keep;
			if acc < 1.0 {
				continue;
			}
			acc -= 1.0;
			let win = &wins[wi];
			let dev = &devs[win.m.dev];
			let views: Vec<&View> =
				win.views.iter().filter(|v| by_lh(v.lh).is_some_and(|s| placed[s].is_some())).collect();
			// the seed from the view with the most hits
			let best = views.iter().max_by_key(|v| v.hits.len()).expect("two views");
			let (rl, pl) = placed[by_lh(best.lh).expect("placed")].expect("placed");
			let r0 = mm3(&rl, &best.r);
			let rp = mv3(&rl, &best.p);
			let p0 = [rp[0] + pl[0], rp[1] + pl[1], rp[2] + pl[2]];
			let mut s =
				Snapshot { dev: win.m.dev as i32, time: win.m.tc as f64 / dev.hz, r0, p0, ..Snapshot::default() };
			for v in &views {
				for &i in &v.hits {
					if s.n >= MS_MAXH {
						break;
					}
					let h = &win.m.hits[i];
					s.lh[s.n] = h.lh;
					s.plane[s.n] = h.plane;
					s.angle[s.n] = h.angle;
					s.dt[s.n] = (h.tc as f64 - win.m.tc as f64) / dev.hz;
					s.pt[s.n] = dev.sensors[h.sensor as usize];
					s.n += 1;
				}
			}
			snaps.push(s);
			snap_win.push(wi);
		}
	}
	if snaps.len() < 20 {
		return Err(format!("only {} still moments were seen by two placed stations: too few", snaps.len()));
	}
	let mut st = [Station::default(); MS_MAXLH];
	for (si, s) in stations.iter().enumerate() {
		if let Some((r, p)) = placed[si] {
			st[s.lh].r = r;
			st[s.lh].p = p;
			st[s.lh].set_fcal(s.fcal);
			st[s.lh].used = true;
			st[s.lh].prior_w = if si == root { 1.0 } else { 0.0 };
		}
	}
	let lam = vec![0.0; MS_MAXG * MS_MAXG];
	let mut lam_add = vec![0.0; MS_MAXG * MS_MAXG];
	let n = snaps.len();
	let mut active = vec![0u8; n * MS_MAXH];
	let mut med_res = f64::NAN;
	for (round, (reject, iters)) in [(0.03, 15), (0.012, 15), (0.005, 25)].into_iter().enumerate() {
		let ms = solve_settings(reject, iters);
		let start = st;
		let mut out = st;
		let r = joint_solve(&ms, &snaps, &mut st, &start, &lam, &mut out, &mut lam_add);
		if !r.ok {
			return Err(format!("the joint solve failed in round {} ({} stations lit)", round + 1, r.nst));
		}
		// the snapshots re-seeded at their fit to the solved stations (which also picks the hits)
		let mut xs = vec![0.0; n * 6];
		warm_start(&ms, &snaps, &out, &mut xs, &mut active);
		for (i, s) in snaps.iter_mut().enumerate() {
			let (r0, p0) = apply_delta(&s.r0, &s.p0, &xs[i * 6..i * 6 + 6]);
			s.r0 = r0;
			s.p0 = p0;
		}
		let moved: f64 = (0..MS_MAXLH)
			.filter(|&l| st[l].used)
			.map(|l| {
				let d = [out[l].p[0] - st[l].p[0], out[l].p[1] - st[l].p[1], out[l].p[2] - st[l].p[2]];
				(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
			})
			.fold(0.0, f64::max);
		lines.push(format!(
			"joint solve {}: {} snapshots, residual median {:.3} -> {:.3} mrad (reject {:.0} mrad), stations moved up to {:.1} mm",
			round + 1,
			n,
			r.med_before * 1e3,
			r.med_after * 1e3,
			reject * 1e3,
			moved * 1e3
		));
		med_res = r.med_after;
		st = out;
	}
	// 4b: the stations' corrections, solved with the stations and the snapshots (the first station held), over the
	// last round's hits
	let mut correction = [[0.0; 10]; MS_MAXLH];
	if opt.corrections {
		let fit = solve_fcal(&mut snaps, &mut st, &active, stations[root].lh, 0.001, 30);
		correction = fit.delta;
		lines.push(format!(
			"the stations' corrections: residual median {:.3} -> {:.3} mrad in {} steps",
			fit.med_before * 1e3,
			fit.med_after * 1e3,
			fit.iterations
		));
		for s in stations {
			if !st[s.lh].used {
				continue;
			}
			let d = &fit.delta[s.lh];
			lines.push(format!(
				"  ch {}: phase {:+.2}/{:+.2} tilt {:+.2}/{:+.2} curve {:+.2}/{:+.2} gibbous {:+.2}/{:+.2} rad x {:+.2}/{:+.2} mrad, residual median {:.3} mrad",
				s.channel,
				d[0] * 1e3,
				d[1] * 1e3,
				d[2] * 1e3,
				d[3] * 1e3,
				d[4] * 1e3,
				d[5] * 1e3,
				d[6],
				d[7],
				d[8] * 1e3,
				d[9] * 1e3,
				fit.med_station[s.lh] * 1e3
			));
		}
		med_res = fit.med_after;
	}

	// per station: its hits and their residuals; per pair: the snapshots linking them; per snapshot: whether its pose
	// is well fixed (enough hits, a tight fit), which its orientation has to be for gravity
	let mut res_l: Vec<Vec<f64>> = vec![Vec::new(); MS_MAXLH];
	let mut links = vec![0usize; MS_MAXLH * MS_MAXLH];
	let mut firm = vec![false; n];
	for (i, s) in snaps.iter().enumerate() {
		let mut lit = [false; MS_MAXLH];
		let mut res = Vec::new();
		for h in 0..s.n {
			if active[i * MS_MAXH + h] == 0 {
				continue;
			}
			let l = s.lh[h] as usize;
			lit[l] = true;
			let m = crate::model::model_angle(s, h, &s.r0, &s.p0, &st[l]);
			res_l[l].push((s.angle[h] - m).abs());
			res.push((s.angle[h] - m).abs());
		}
		for a in 0..MS_MAXLH {
			for b in 0..MS_MAXLH {
				if a != b && lit[a] && lit[b] {
					links[a * MS_MAXLH + b] += 1;
				}
			}
		}
		firm[i] = res.len() >= GRAVITY_MIN_HITS && median_of(&mut res) <= GRAVITY_MAX_RES;
	}

	// 5: up from the accelerometers, one row per distinct orientation of each device, from the snapshots whose pose is
	// well fixed
	let mut all: Vec<Row> = Vec::new();
	for (i, s) in snaps.iter().enumerate() {
		if !firm[i] {
			continue;
		}
		let win = &wins[snap_win[i]];
		let f = [win.m.acc[0] * GRAVITY, win.m.acc[1] * GRAVITY, win.m.acc[2] * GRAVITY];
		all.push((s.dev as usize, s.r0, f));
	}
	let mut kept = distinct_rows(&all);
	if kept.len() < 3 {
		return Err("too few still moments with a well-fixed pose to find up".into());
	}
	let (mut up, mut biases, mut up_sd) = solve_up_any(&kept, devs.len()).ok_or("the accelerometers do not give up")?;
	let mut dropped = Vec::new();
	// what up and a bias cannot explain out, twice: single rows far off (a device that moved after all), then a device
	// whose rows as a whole are (its orientation or its accelerometer is not to be trusted)
	for _ in 0..2 {
		let e = up_residuals(&kept, &up, &biases);
		let med_e = median_of(&mut e.clone());
		let limit = (3.0 * med_e).max(GRAVITY_ROW_LIMIT);
		let mut rows: Vec<Row> = kept.iter().zip(&e).filter(|(_, e)| **e < limit).map(|(r, _)| *r).collect();
		let e = up_residuals(&rows, &up, &biases);
		for d in 0..devs.len() {
			let mut mine: Vec<f64> = rows.iter().zip(&e).filter(|(r, _)| r.0 == d).map(|(_, e)| *e).collect();
			if !mine.is_empty() && median_of(&mut mine) > GRAVITY_DEVICE_LIMIT {
				rows.retain(|r| r.0 != d);
				dropped.push(devs[d].name.clone());
			}
		}
		if rows.len() < 3 {
			return Err("no device's accelerometer agrees with the light's orientation: up is not found".into());
		}
		kept = rows;
		(up, biases, up_sd) = solve_up_any(&kept, devs.len()).ok_or("the accelerometers do not give up")?;
	}
	let mut e = up_residuals(&kept, &up, &biases);
	let med_e = median_of(&mut e);
	let up_tilt = tilt_az(&up);
	lines.push(format!(
		"gravity: {} distinct still orientations of {} devices (accelerometer residual median {:.3} m/s^2): up is {:.2} mrad off the solve's z toward {:.1} deg, +-{:.2} mrad",
		kept.len(),
		devs.iter().enumerate().filter(|(d, _)| kept.iter().any(|r| r.0 == *d)).count(),
		med_e,
		up_tilt.0 * 1e3,
		up_tilt.1,
		up_sd * 1e3
	));
	if !dropped.is_empty() {
		lines.push(format!(
			"gravity: left out, their accelerometers not agreeing with their orientation: {}",
			dropped.join(", ")
		));
	}
	if up_sd > opt.max_up_sd {
		return Err(format!(
			"up is known to +-{:.1} mrad only (at most {:.1} wanted): the devices were still at too few headings to tell \
			 a tilt from their accelerometers' own offsets. Record again turning slowly on the spot, pausing now and then",
			up_sd * 1e3,
			opt.max_up_sd * 1e3
		));
	}
	if up_sd > 0.002 {
		lines.push(format!(
			"gravity: up is known to +-{:.1} mrad only: the devices were still at too few headings to tell a tilt from \
			 their accelerometers' own offsets (turn slowly on the spot and pause now and then)",
			up_sd * 1e3
		));
	}
	let mut devices = Vec::new();
	for (di, dev) in devs.iter().enumerate() {
		let mine: Vec<Row> = kept.iter().filter(|r| r.0 == di).map(|r| (0, r.1, r.2)).collect();
		if mine.is_empty() {
			continue;
		}
		// the headings: the device's x axis (or y, whichever lies flatter) projected on the floor
		let hd: Vec<f64> = mine
			.iter()
			.map(|(_, r, _)| {
				let (x, y) = if r[6].abs() < r[7].abs() { (r[0], r[3]) } else { (r[1], r[4]) };
				y.atan2(x)
			})
			.collect();
		let gap = largest_gap(hd);
		let alone =
			if gap < 240.0 && mine.len() >= 8 { solve_up_any(&mine, 1).map(|(u, _, _)| tilt_az(&u)) } else { None };
		let b = biases[di];
		lines.push(format!(
			"  {}: {} moments, headings all but a {:.0} deg gap, bias {:+.3} {:+.3} {:+.3} m/s^2{}",
			dev.name,
			mine.len(),
			gap,
			b[0],
			b[1],
			b[2],
			match alone {
				Some((t, a)) => format!(", alone: {:.2} mrad toward {:.1} deg", t * 1e3, a),
				None => String::new(),
			}
		));
		devices.push(DeviceGravity {
			name: dev.name.clone(),
			windows: mine.len(),
			largest_gap_deg: gap,
			alone,
			bias: b,
		});
	}

	// 6: the frame: up to +z by the smallest turn, the devices' middle at x = y = 0, the floor at 0; or, with the room's
	// calibration so far, turned about the vertical and moved onto it
	let mut rf = turn_to_z(&up);
	let pos: Vec<[f64; 3]> = snaps.iter().map(|s| mv3(&rf, &s.p0)).collect();
	let mut xs: Vec<f64> = pos.iter().map(|p| p[0]).collect();
	let mut ys: Vec<f64> = pos.iter().map(|p| p[1]).collect();
	let mut zs: Vec<f64> = pos.iter().map(|p| p[2]).collect();
	zs.sort_by(|a, b| a.total_cmp(b));
	let floor = zs[zs.len() / 100] - 0.03;
	let mut o = [median_of(&mut xs), median_of(&mut ys), floor];
	let tied: Vec<(usize, [f64; 3], [f64; 3])> = stations
		.iter()
		.enumerate()
		.filter(|(si, _)| placed[*si].is_some())
		.filter_map(|(_, s)| {
			let a = opt.anchors.iter().find(|(id, _)| *id == s.id)?;
			Some((s.lh, mv3(&rf, &st[s.lh].p), a.1.pos))
		})
		.collect();
	if tied.len() >= 2 {
		// the turn about z and the move that fit the levelled stations onto their old places (Procrustes in the
		// plane, the height by the mean)
		let n = tied.len() as f64;
		let mut cn = [0.0; 3];
		let mut co = [0.0; 3];
		for (_, pn, po) in &tied {
			for k in 0..3 {
				cn[k] += pn[k] / n;
				co[k] += po[k] / n;
			}
		}
		let (mut sc, mut ss) = (0.0, 0.0);
		for (_, pn, po) in &tied {
			let (xn, yn) = (pn[0] - cn[0], pn[1] - cn[1]);
			let (xo, yo) = (po[0] - co[0], po[1] - co[1]);
			sc += xn * xo + yn * yo;
			ss += xn * yo - yn * xo;
		}
		let yaw = ss.atan2(sc);
		let rz = [yaw.cos(), -yaw.sin(), 0.0, yaw.sin(), yaw.cos(), 0.0, 0.0, 0.0, 1.0];
		let rc = mv3(&rz, &cn);
		rf = mm3(&rz, &rf);
		// final = rf p - o: o puts the turned centroid on the old one
		o = [rc[0] - co[0], rc[1] - co[1], rc[2] - co[2]];
		let mut far = Vec::new();
		for (l, pn, po) in &tied {
			let q = mv3(&rz, pn);
			let d = [q[0] - o[0] - po[0], q[1] - o[1] - po[1], q[2] - o[2] - po[2]];
			let ch = stations.iter().find(|s| s.lh == *l).map(|s| s.channel).unwrap_or(0);
			far.push(format!("ch {ch} {:.1} mm", (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() * 1e3));
		}
		lines.push(format!(
			"the frame tied to the room's calibration so far ({} stations): turned {:+.2} mrad about the vertical; each station from its old place: {}",
			tied.len(),
			yaw * 1e3,
			far.join(", ")
		));
	}
	let frame = Pose { pos: [-o[0], -o[1], -o[2]], rot: r2q(&rf) };
	let mut out = Vec::new();
	for (si, s) in stations.iter().enumerate() {
		if placed[si].is_none() {
			continue;
		}
		let l = s.lh;
		let r = mm3(&rf, &st[l].r);
		let p = mv3(&rf, &st[l].p);
		let mut res = res_l[l].clone();
		let med = median_of(&mut res);
		out.push(Placed {
			lh: l,
			channel: s.channel,
			id: s.id,
			pose: Pose { pos: [p[0] - o[0], p[1] - o[1], p[2] - o[2]], rot: r2q(&r) },
			hits: res_l[l].len(),
			med_res: med,
			correction: correction[l],
		});
	}
	for a in &out {
		let others: Vec<String> = out
			.iter()
			.filter(|b| b.lh != a.lh)
			.map(|b| format!("ch {} {}", b.channel, links[a.lh * MS_MAXLH + b.lh]))
			.collect();
		lines.push(format!(
			"ch {} ({:08x}): at {:+.3} {:+.3} {:+.3} m, {} hits, residual median {:.3} mrad; snapshots shared with {}",
			a.channel,
			a.id,
			a.pose.pos[0],
			a.pose.pos[1],
			a.pose.pos[2],
			a.hits,
			a.med_res * 1e3,
			others.join(", ")
		));
	}
	Ok(Calibration { stations: out, up_tilt, up_sd, frame, devices, snapshots: n, med_res, lines })
}

/// A station as `score` takes it: its id, lh2world, and the calibration its light is predicted with
#[derive(Clone, Copy, Debug)]
pub struct Scored {
	pub id: u32,
	pub pose: Pose,
	pub fcal: [BaseStationCal; 2],
}

/// How well a set of stations predicts the light of still moments it was not solved from: each moment placed by the
/// station that sees it best (its view), then its pose fitted to every station's hits (robust, the stations held), and
/// the median |residual| of the hits within 10 mrad of it (reflections out) over all the moments. Each device's sensor
/// layout is scaled as fits it best over its moments, as the tracker learns it (its layout scale): the map's scale is
/// the devices' as one session had them (findings.md), and a set solved with other devices would otherwise be scored
/// on that. Stations by id (`moments`' hits by index into `stations_in`, as `calibrate` takes them). None when no
/// moment is placed
pub fn score(
	devs: &[Device], moments: &[Moment], stations_in: &[StationIn], set: &[Scored], opt: &Options,
) -> Option<f64> {
	let mut res: Vec<f64> =
		score_moments(devs, moments, stations_in, set, opt).into_iter().flatten().flatten().collect();
	if res.is_empty() {
		return None;
	}
	Some(median_of(&mut res))
}

/// The gate: hits beyond it are reflections, left out of a moment's fit and its residuals
const SCORE_GATE: f64 = 0.01;

/// Fewer moments than this, and a device's layout scale is not fitted (it stays 1)
const SCORE_SCALE_MOMENTS: usize = 10;

/// `score`'s residuals moment by moment: for each of `moments` the |residuals| of its hits within 10 mrad of its
/// fitted pose (None when no station of the set places it)
pub fn score_moments(
	devs: &[Device], moments: &[Moment], stations_in: &[StationIn], set: &[Scored], opt: &Options,
) -> Vec<Option<Vec<f64>>> {
	let mut st = [Station::default(); MS_MAXLH];
	let mut sin: Vec<Option<StationIn>> = vec![None; MS_MAXLH];
	for s in stations_in {
		if let Some(x) = set.iter().find(|x| x.id == s.id) {
			let l = s.lh;
			st[l].r = crate::linalg::q2r(&x.pose.rot);
			st[l].p = x.pose.pos;
			st[l].set_fcal(x.fcal);
			st[l].used = true;
			sin[l] = Some(StationIn { fcal: x.fcal, ..*s });
		}
	}
	let mut bcs: Vec<Option<BarycentricSvd>> = devs
		.iter()
		.map(|d| {
			(d.sensors.len() >= 4 && d.normals.len() == d.sensors.len()).then(|| BarycentricSvd::new(d.sensors.clone()))
		})
		.collect();
	// each moment seeded once (its best station's view), its pose then fitted at each scale tried, from where the last
	// fit left it
	let mut fits: Vec<Option<Snapshot>> =
		moments.iter().map(|m| score_seed(devs, &mut bcs, m, &st, &sin, opt)).collect();
	// each device's layout scale: the light's robust cost over its moments at three scales, the parabola's least,
	// twice, the second time over half the step
	let mut scale = vec![1.0; devs.len()];
	for (d, sc) in scale.iter_mut().enumerate() {
		let idx: Vec<usize> = (0..moments.len()).filter(|&i| moments[i].dev == d && fits[i].is_some()).collect();
		if idx.len() < SCORE_SCALE_MOMENTS {
			continue;
		}
		let sensors = &devs[d].sensors;
		let mut cost = |s: f64| -> f64 {
			let mut c = 0.0;
			for &i in &idx {
				let Some(f) = fits[i].as_mut() else { continue };
				for r in score_fit(f, &moments[i], sensors, &st, s) {
					c += crate::model::huber_rho(r.min(SCORE_GATE), 0.001);
				}
			}
			c
		};
		let mut c = 1.0;
		let mut h = 0.005;
		let mut fc = cost(c);
		for _ in 0..2 {
			let (fa, fb) = (cost(c - h), cost(c + h));
			let curv = fa + fb - 2.0 * fc;
			let step = if curv > 0.0 { (0.5 * h * (fa - fb) / curv).clamp(-2.0 * h, 2.0 * h) } else { 0.0 };
			let t = c + step;
			let ft = cost(t);
			if ft < fc {
				c = t;
				fc = ft;
			}
			h *= 0.5;
		}
		*sc = c;
	}
	(0..moments.len())
		.map(|i| {
			let f = fits[i].as_mut()?;
			let m = &moments[i];
			let r = score_fit(f, m, &devs[m.dev].sensors, &st, scale[m.dev]);
			Some(r.into_iter().filter(|&r| r <= SCORE_GATE).collect())
		})
		.collect()
}

/// A moment's snapshot for `score_moments`: its hits on the set's stations and its pose seeded from the best
/// station's own view (None when no station of the set places it)
fn score_seed(
	devs: &[Device], bcs: &mut [Option<BarycentricSvd>], m: &Moment, st: &[Station; MS_MAXLH],
	sin: &[Option<StationIn>], opt: &Options,
) -> Option<Snapshot> {
	let dev = devs.get(m.dev)?;
	let bc = bcs.get_mut(m.dev)?.as_mut()?;
	let mut best: Option<(usize, View)> = None;
	for l in 0..MS_MAXLH {
		let Some(si) = sin[l].as_ref() else { continue };
		let idx: Vec<usize> = (0..m.hits.len())
			.filter(|&i| m.hits[i].lh as usize == l && (m.hits[i].sensor as usize) < dev.sensors.len())
			.collect();
		if idx.len() < opt.min_view_hits {
			continue;
		}
		let k = station_k(&si.fcal);
		if let Some(v) = view(bc, &Sight { dev, st: si, k: &k }, &m.hits, &idx, opt) {
			if best.as_ref().is_none_or(|(_, b)| v.hits.len() > b.hits.len()) {
				best = Some((l, v));
			}
		}
	}
	let (l, v) = best?;
	let mut s = Snapshot {
		dev: m.dev as i32,
		r0: mm3(&st[l].r, &v.r),
		p0: {
			let rp = mv3(&st[l].r, &v.p);
			[rp[0] + st[l].p[0], rp[1] + st[l].p[1], rp[2] + st[l].p[2]]
		},
		..Snapshot::default()
	};
	for h in &m.hits {
		if s.n >= MS_MAXH || !st[h.lh as usize].used || (h.sensor as usize) >= dev.sensors.len() {
			continue;
		}
		s.lh[s.n] = h.lh;
		s.plane[s.n] = h.plane;
		s.angle[s.n] = h.angle;
		s.dt[s.n] = (h.tc as f64 - m.tc as f64) / dev.hz;
		s.pt[s.n] = dev.sensors[h.sensor as usize];
		s.n += 1;
	}
	Some(s)
}

/// A seeded moment's pose fitted to every station with its device's sensors at `scale` (Gauss-Newton, Huber at 1
/// mrad, hits beyond the gate left out), from its pose so far; all its hits' |residuals|
fn score_fit(s: &mut Snapshot, m: &Moment, sensors: &[[f64; 3]], st: &[Station; MS_MAXLH], scale: f64) -> Vec<f64> {
	let mut k = 0;
	for h in &m.hits {
		if k >= s.n {
			break;
		}
		if !st[h.lh as usize].used || (h.sensor as usize) >= sensors.len() {
			continue;
		}
		let p = sensors[h.sensor as usize];
		s.pt[k] = [p[0] * scale, p[1] * scale, p[2] * scale];
		k += 1;
	}
	let gate = SCORE_GATE;
	for _ in 0..6 {
		let sp6 = stepped(&s.r0, &s.p0, &[0.0; 6]);
		let mut a = [0.0; 36];
		let mut b = [0.0; 6];
		for h in 0..s.n {
			let lh = s.lh[h] as usize;
			let r = s.angle[h] - crate::model::model_angle(s, h, &s.r0, &s.p0, &st[lh]);
			if r.abs() > gate {
				continue;
			}
			let w = crate::model::huber_w(r, 0.001);
			let mut j = [0.0; 6];
			for k in 0..6 {
				j[k] = (s.angle[h] - crate::model::model_angle(s, h, &sp6[k].0, &sp6[k].1, &st[lh]) - r) / 1e-6;
			}
			for p in 0..6 {
				for q in 0..6 {
					a[p * 6 + q] += w * j[p] * j[q];
				}
				b[p] += w * j[p] * r;
			}
		}
		let Some(inv) = crate::linalg::inv6(&a) else { break };
		let mut dx = [0.0; 6];
		for p in 0..6 {
			for q in 0..6 {
				dx[p] -= inv[p * 6 + q] * b[q];
			}
		}
		let (r0, p0) = apply_delta(&s.r0, &s.p0, &dx);
		s.r0 = r0;
		s.p0 = p0;
		// converged: the step under a micron and a microradian
		if dx.iter().all(|v| v.abs() < 1e-6) {
			break;
		}
	}
	(0..s.n)
		.map(|h| (s.angle[h] - crate::model::model_angle(s, h, &s.r0, &s.p0, &st[s.lh[h] as usize])).abs())
		.collect()
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::linalg::q2r;

	/// A small deterministic generator: uniform in [0, 1) and normal
	struct Rng(u64);
	impl Rng {
		fn u(&mut self) -> f64 {
			self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
			(self.0 >> 11) as f64 / (1u64 << 53) as f64
		}
		fn n(&mut self) -> f64 {
			let (a, b) = (self.u().max(1e-300), self.u());
			(-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
		}
	}

	/// A station at pos looking at the room's middle (its -z toward (0, 0, 1)), its y up
	fn station_r(pos: [f64; 3]) -> [f64; 9] {
		let d = [pos[0], pos[1], pos[2] - 1.0];
		let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
		let z = [d[0] / n, d[1] / n, d[2] / n];
		let mut x = [-z[1], z[0], 0.0];
		let xn = (x[0] * x[0] + x[1] * x[1]).sqrt();
		x = [x[0] / xn, x[1] / xn, 0.0];
		let y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
		[x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]
	}

	fn fcal(seed: f64) -> [BaseStationCal; 2] {
		let one = |s: f64| BaseStationCal {
			phase: 0.001 * s,
			tilt: -0.047 + 0.002 * s,
			curve: 0.2 * s,
			gibpha: 1.0 + s,
			gibmag: 0.005 * s,
			ogeephase: 0.7 + s,
			ogeemag: -0.3 * s,
		};
		[one(seed), one(-seed * 0.7)]
	}

	/// 24 sensors on a 4 cm sphere, facing out
	fn layout() -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
		let (mut loc, mut nrm) = (Vec::new(), Vec::new());
		for i in 0..24 {
			let a = i as f64 * 2.399963;
			let z = 0.95 - 1.9 * i as f64 / 23.0;
			let r = (1.0 - z * z).sqrt();
			let n = [r * a.cos(), r * a.sin(), z];
			nrm.push(n);
			loc.push([0.04 * n[0], 0.04 * n[1], 0.04 * n[2]]);
		}
		(loc, nrm)
	}

	/// The room: four stations in the corners at 2.2 m, the world tilted by `tilt` toward `az` against the stations
	/// as the calibration first sees them, three devices still at many places and headings
	/// The test room: its devices, what each measured (hits, IMU), its stations and where they truly are
	struct Room {
		devs: Vec<Device>,
		data: Vec<(Vec<Hit>, Vec<Imu>)>,
		stations: Vec<StationIn>,
		truth: Vec<([f64; 9], [f64; 3])>,
	}

	fn room(tilt: f64, az: f64) -> Room {
		let mut rng = Rng(7);
		let corners = [[2.4, 2.1, 2.2], [-2.3, 2.2, 2.25], [-2.2, -2.4, 2.2], [2.5, -2.2, 2.3]];
		let truth: Vec<([f64; 9], [f64; 3])> = corners.iter().map(|&p| (station_r(p), p)).collect();
		let stations: Vec<StationIn> = (0..4)
			.map(|l| StationIn {
				lh: l,
				channel: [3u8, 7, 9, 13][l],
				id: 1000 + l as u32,
				fcal: fcal(l as f64 * 0.3 + 0.1),
			})
			.collect();
		let ks: Vec<[Gen2AxisConsts; 2]> = stations.iter().map(|s| station_k(&s.fcal)).collect();
		let up = [tilt.sin() * az.to_radians().cos(), tilt.sin() * az.to_radians().sin(), tilt.cos()];
		let (sensors, normals) = layout();
		let hz = 48e6;
		let mut devs = Vec::new();
		let mut data = Vec::new();
		for d in 0..3 {
			let bias = [0.15 * (d as f64 - 1.0), 0.08, -0.1 + 0.12 * d as f64];
			let dev = Device { name: format!("D{d}"), hz, sensors: sensors.clone(), normals: normals.clone() };
			let (mut hits, mut imu): (Vec<Hit>, Vec<Imu>) = (Vec::new(), Vec::new());
			let mut t = 1.0;
			for _ in 0..90 {
				// a still pose: anywhere in the room, any heading, a little tilted
				let p = [rng.u() * 3.0 - 1.5, rng.u() * 3.0 - 1.5, 0.1 + rng.u() * 1.6];
				let r = mm3(
					&rv2r(&[0.0, 0.0, rng.u() * std::f64::consts::TAU]),
					&rv2r(&[0.3 * rng.n(), 0.3 * rng.n(), 0.0]),
				);
				let (t0, t1) = (t, t + 0.4);
				let mut s = t0;
				while s < t1 {
					let acc = mtv3(&r, &up);
					imu.push(Imu {
						tc: (s * hz) as u64,
						acc: [0, 1, 2].map(|k| acc[k] + bias[k] / GRAVITY + 0.001 * rng.n()),
						gyro: [0, 1, 2].map(|_| 0.004 * rng.n()),
					});
					s += 0.001;
				}
				// a turn of each station every 20 ms, its two planes 5 ms apart
				let mut s = t0 + 0.003;
				while s < t1 {
					for (l, (rl, pl)) in truth.iter().enumerate() {
						for plane in 0..2i8 {
							let tc = ((s + 0.004 * l as f64 + 0.005 * plane as f64) * hz) as u64;
							for (i, (pt, n)) in sensors.iter().zip(&normals).enumerate() {
								let w = mv3(&r, pt);
								let x = [w[0] + p[0], w[1] + p[1], w[2] + p[2]];
								let nw = mv3(&r, n);
								let to = [pl[0] - x[0], pl[1] - x[1], pl[2] - x[2]];
								let dn = (to[0] * to[0] + to[1] * to[1] + to[2] * to[2]).sqrt();
								if (nw[0] * to[0] + nw[1] * to[1] + nw[2] * to[2]) / dn < 0.3 {
									continue;
								}
								let loc = mtv3(rl, &[x[0] - pl[0], x[1] - pl[1], x[2] - pl[2]]);
								let pi = plane as usize;
								let angle = survive_reproject_axis_gen2_with(&stations[l].fcal, &ks[l][pi], pi, &loc)
									+ 0.0001 * rng.n();
								hits.push(Hit { tc, lh: l as u8, sensor: i as u8, plane, angle });
							}
						}
					}
					s += 0.02;
				}
				// then it moves on: turning, no light kept
				let mut s = t1;
				while s < t1 + 0.3 {
					imu.push(Imu { tc: (s * hz) as u64, acc: [0.0, 0.0, 1.0], gyro: [1.0, 0.5, 0.0] });
					s += 0.001;
				}
				t = t1 + 0.3;
			}
			hits.sort_by_key(|h| h.tc);
			devs.push(dev);
			data.push((hits, imu));
		}
		Room { devs, data, stations, truth }
	}

	/// Each device's samples through its StillFinder in the order of their clock, the light 20 ms late
	fn moments(devs: &[Device], data: &[(Vec<Hit>, Vec<Imu>)], opt: &Options) -> Vec<Moment> {
		let mut out = Vec::new();
		for (d, (dev, (hits, imu))) in devs.iter().zip(data).enumerate() {
			let mut f = StillFinder::new(d, dev, opt);
			let late = (0.02 * dev.hz) as u64;
			let mut h = 0;
			for x in imu {
				while h < hits.len() && hits[h].tc + late <= x.tc {
					f.hit(hits[h]);
					h += 1;
				}
				out.extend(f.imu(*x));
			}
		}
		out
	}

	#[test]
	fn a_room_is_placed_and_levelled_from_still_moments() {
		let (tilt, az) = (0.011, 300.0);
		let Room { devs, data, stations, truth } = room(tilt, az);
		// the calibration sees the room as the stations do, turned by an arbitrary rotation: what it can recover is
		// the stations' layout and where up is
		let opt = Options::default();
		let c = calibrate(&devs, moments(&devs, &data, &opt), &stations, &opt).expect("calibrates");
		for l in &c.lines {
			eprintln!("{l}");
		}
		assert_eq!(c.stations.len(), 4);
		// the layout: the calibrated stations against the truth, after the best rigid fit
		let src: Vec<sally_math::math3d::V3> = c
			.stations
			.iter()
			.map(|s| sally_math::math3d::V3::new(s.pose.pos[0], s.pose.pos[1], s.pose.pos[2]))
			.collect();
		let dst: Vec<sally_math::math3d::V3> = c
			.stations
			.iter()
			.map(|s| {
				let p = truth[s.lh].1;
				sally_math::math3d::V3::new(p[0], p[1], p[2])
			})
			.collect();
		let (fit, rms) = sally_math::math3d::fit_rigid(&src, &dst).expect("fits");
		assert!(rms < 0.001, "station layout rms {rms} m");
		// up: the calibration's z is the truth's up
		let up_true = [tilt.sin() * az.to_radians().cos(), tilt.sin() * az.to_radians().sin(), tilt.cos()];
		let z_in_truth = sally_math::math3d::qrot(fit.q, sally_math::math3d::V3::new(0.0, 0.0, 1.0));
		let cosang = z_in_truth.x * up_true[0] + z_in_truth.y * up_true[1] + z_in_truth.z * up_true[2];
		let err = cosang.clamp(-1.0, 1.0).acos();
		assert!(err < 0.0005, "up off by {:.3} mrad", err * 1e3);
		// each station's orientation too
		for s in &c.stations {
			let rc = q2r(&s.pose.rot);
			let fr = q2r(&[fit.q.w, fit.q.x, fit.q.y, fit.q.z]);
			let d = r2rv(&mm3(&mm3(&fr, &rc), &transpose3(&truth[s.lh].0)));
			let a = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
			assert!(a < 0.001, "ch {} orientation off by {:.3} mrad", s.channel, a * 1e3);
		}
		// the biases
		for (d, g) in c.devices.iter().enumerate() {
			let b = [0.15 * (d as f64 - 1.0), 0.08, -0.1 + 0.12 * d as f64];
			for k in 0..3 {
				assert!((g.bias[k] - b[k]).abs() < 0.01, "{} bias {:?}", g.name, g.bias);
			}
		}
	}
}
