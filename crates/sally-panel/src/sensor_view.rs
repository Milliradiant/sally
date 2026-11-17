//! The device page's sensors: which of a device's light sensors each station lit in the runtime's last periods (its H
//! lines), which it should have lit, and which got light that cannot have come straight from it (docs/sally_panel.md,
//! "Sensors").
//!
//! A sensor should be lit by a station that lights the device (LIGHTS_MIN of its sensors from the front, in the
//! period) when it faces the station within EXPECT_DEG, lies inside the station's field of view and within its reach,
//! and the device's own body is not in the way: a ray from the sensor toward the station meets none of the device's
//! render model's triangles (a `Bvh` over them). It is **missing** when it got no light at all in the last two periods
//! while one station should have lit it all along (at both ends of both periods, each a pose of the device as it was
//! tracking): covered (a hand, a strap, a sticker) or dead. It is lit **from behind** when every station whose light it
//! got lies further round than BACK_DEG from its normal, the runtime's own test for a reflection (diagnosis-back-deg).
//!
//! Everything is in the device's head frame (the frame of its pose), the sensors moved into it from the IMU's
//! (`Device::sensor_in_head`), the model turned into it by the offset the room view uses.

use std::collections::BTreeMap;

use sally_math::math3d::{cross, dot, len, qconj, qrot, Pose, Q4, V3};

use crate::runtime::Device;

/// A sensor is expected to be lit by a station it faces within this of its normal (deg). Measured on a replay's light
/// (docs/sally_panel.md, "Sensors"): below it a sensor in a station's view, not hidden by its device, got that
/// station's light in nearly every second the station lit its device
pub const EXPECT_DEG: f64 = 70.0;
/// Light from further round than this cannot have come straight from the station (deg): the runtime's
/// diagnosis-back-deg and the live light-facing-deg (real hits reach 114 deg on two Index controller sensors)
pub const BACK_DEG: f64 = 115.0;
/// A station lights the device in a period when it lit this many of its sensors from the front: one or two can be
/// stray light
pub const LIGHTS_MIN: usize = 3;
/// How far a station's light reaches (m)
pub const REACH_M: f64 = 7.0;
/// The ray from a sensor toward a station starts this far out of the model's shell over it (m): a sensor sits up to
/// 3.5 mm under an Index controller's shell and 7 mm under a Vive tracker 3.0's (its config's place on SteamVR's
/// model), so the ray first goes out along its normal through the shell (`Bvh::out_of`), out to SHELL_M
pub const RAY_START_M: f64 = 0.001;
/// How far out along its normal a sensor's shell can be (m)
pub const SHELL_M: f64 = 0.012;

/// A station's field of view, half its width and half its height (deg): gen2 150 by 110, gen1 120 by 120 (the room's
/// cones)
pub fn fov_half_deg(gen2: bool) -> (f64, f64) {
	if gen2 {
		(75.0, 55.0)
	} else {
		(60.0, 60.0)
	}
}

/// A station as the sensors are judged against it
#[derive(Clone, Copy, Debug)]
pub struct StationAt {
	pub idx: i32,
	/// station to world
	pub pose: Pose,
	pub gen2: bool,
}

/// How a sensor at `p` facing `n` (both in the room) sees a station: the angle off its normal (deg), whether it lies
/// in the station's field of view (looking down the station's -Z) and within its reach, and the unit direction to it
pub fn look(p: V3, n: V3, st: &StationAt) -> (f64, bool, V3) {
	let to = st.pose.p - p;
	let d = len(to);
	if d < 1e-9 {
		return (f64::NAN, false, V3::default());
	}
	let u = to * (1.0 / d);
	let nl = len(n);
	let angle = if nl > 1e-9 { (dot(n, u) / nl).clamp(-1.0, 1.0).acos().to_degrees() } else { f64::NAN };
	// the sensor seen from the station, in its frame
	let v = qrot(qconj(st.pose.q), u * -1.0);
	let (h, w) = fov_half_deg(st.gen2);
	let in_view = d <= REACH_M
		&& v.z < 0.0
		&& v.x.abs() <= h.to_radians().tan() * -v.z
		&& v.y.abs() <= w.to_radians().tan() * -v.z;
	(angle, in_view, u)
}

// ---------------------------------------------------------------- the device's own body

/// A bounding volume tree over a model's triangles, for rays from its sensors: a node's box holds its triangles; a
/// leaf has at most LEAF of them
pub struct Bvh {
	nodes: Vec<Node>,
	tris: Vec<[V3; 3]>,
}

#[derive(Clone, Copy, Debug)]
struct Node {
	lo: V3,
	hi: V3,
	/// a leaf: its triangles from `a`, `n` of them; else its children at `a` and `a + 1`
	a: u32,
	n: u32,
}

const LEAF: usize = 4;

fn vmin(a: V3, b: V3) -> V3 {
	V3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z))
}

fn vmax(a: V3, b: V3) -> V3 {
	V3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))
}

fn axis(v: V3, k: usize) -> f64 {
	match k {
		0 => v.x,
		1 => v.y,
		_ => v.z,
	}
}

impl Bvh {
	pub fn new(tris: &[[V3; 3]]) -> Bvh {
		let mut tris: Vec<[V3; 3]> = tris
			.iter()
			.filter(|t| t.iter().all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()))
			.copied()
			.collect();
		let mut nodes = Vec::with_capacity(2 * tris.len() / LEAF + 1);
		nodes.push(Node { lo: V3::default(), hi: V3::default(), a: 0, n: 0 });
		if !tris.is_empty() {
			let n = tris.len();
			Bvh::build(&mut nodes, &mut tris, 0, 0, n);
		}
		Bvh { nodes, tris }
	}

	/// Node `at` over tris[from..to]: its box, then split on the box's longest side at the middle triangle
	fn build(nodes: &mut Vec<Node>, tris: &mut [[V3; 3]], at: usize, from: usize, to: usize) {
		let (mut lo, mut hi) = (V3::new(f64::MAX, f64::MAX, f64::MAX), V3::new(f64::MIN, f64::MIN, f64::MIN));
		for t in &tris[from..to] {
			for v in t {
				lo = vmin(lo, *v);
				hi = vmax(hi, *v);
			}
		}
		if to - from <= LEAF {
			nodes[at] = Node { lo, hi, a: from as u32, n: (to - from) as u32 };
			return;
		}
		let ext = hi - lo;
		let k = if ext.x >= ext.y && ext.x >= ext.z {
			0
		} else if ext.y >= ext.z {
			1
		} else {
			2
		};
		let mid = (from + to) / 2;
		let centre = |t: &[V3; 3]| axis(t[0], k) + axis(t[1], k) + axis(t[2], k);
		tris[from..to].select_nth_unstable_by(mid - from, |a, b| centre(a).total_cmp(&centre(b)));
		let left = nodes.len();
		nodes.push(Node { lo, hi, a: 0, n: 0 });
		nodes.push(Node { lo, hi, a: 0, n: 0 });
		nodes[at] = Node { lo, hi, a: left as u32, n: 0 };
		Bvh::build(nodes, tris, left, from, mid);
		Bvh::build(nodes, tris, left + 1, mid, to);
	}

	pub fn triangles(&self) -> usize {
		self.tris.len()
	}

	/// Where the ray from `o` along unit `d` first meets a triangle, within `t_max` (m)
	pub fn first(&self, o: V3, d: V3, t_max: f64) -> Option<f64> {
		if self.tris.is_empty() {
			return None;
		}
		let inv = V3::new(1.0 / d.x, 1.0 / d.y, 1.0 / d.z);
		let mut best = t_max;
		let mut found = false;
		let mut stack = [0u32; 64];
		let mut sp = 1;
		while sp > 0 {
			sp -= 1;
			let nd = self.nodes[stack[sp] as usize];
			if !slab(o, inv, nd.lo, nd.hi, best) {
				continue;
			}
			if nd.n > 0 {
				let (a, n) = (nd.a as usize, nd.n as usize);
				for t in &self.tris[a..a + n] {
					if let Some(at) = ray_tri(o, d, t).filter(|&at| at < best) {
						best = at;
						found = true;
					}
				}
			} else if sp + 2 <= stack.len() {
				stack[sp] = nd.a;
				stack[sp + 1] = nd.a + 1;
				sp += 2;
			}
		}
		found.then_some(best)
	}

	/// Where a sensor at `p` facing unit `n` comes out of the model: out along its normal past every shell within
	/// SHELL_M, then RAY_START_M more
	pub fn out_of(&self, p: V3, n: V3) -> V3 {
		let mut o = p;
		let mut gone = 0.0;
		for _ in 0..8 {
			match self.first(o, n, SHELL_M - gone) {
				Some(t) => {
					o += n * (t + 1e-4);
					gone += t + 1e-4;
				}
				None => break,
			}
		}
		o + n * RAY_START_M
	}

	/// Whether the ray from `o` along unit `d` meets a triangle within `t_max` (m)
	pub fn hits(&self, o: V3, d: V3, t_max: f64) -> bool {
		if self.tris.is_empty() {
			return false;
		}
		let inv = V3::new(1.0 / d.x, 1.0 / d.y, 1.0 / d.z);
		let mut stack = [0u32; 64];
		let mut sp = 1;
		while sp > 0 {
			sp -= 1;
			let nd = self.nodes[stack[sp] as usize];
			if !slab(o, inv, nd.lo, nd.hi, t_max) {
				continue;
			}
			if nd.n > 0 {
				let (a, n) = (nd.a as usize, nd.n as usize);
				if self.tris[a..a + n].iter().any(|t| ray_tri(o, d, t).is_some_and(|t| t < t_max)) {
					return true;
				}
			} else if sp + 2 <= stack.len() {
				stack[sp] = nd.a;
				stack[sp + 1] = nd.a + 1;
				sp += 2;
			}
		}
		false
	}
}

/// The ray against a box (its slabs), between 0 and t_max
fn slab(o: V3, inv: V3, lo: V3, hi: V3, t_max: f64) -> bool {
	let (mut t0, mut t1) = (0.0f64, t_max);
	for k in 0..3 {
		let (oi, ii, l, h) = (axis(o, k), axis(inv, k), axis(lo, k), axis(hi, k));
		let (mut a, mut b) = ((l - oi) * ii, (h - oi) * ii);
		if a > b {
			std::mem::swap(&mut a, &mut b);
		}
		// (NaN from 0 times infinity, a ray along the box's face: kept in)
		if a.is_finite() || a.is_infinite() {
			t0 = t0.max(a);
		}
		if b.is_finite() || b.is_infinite() {
			t1 = t1.min(b);
		}
		if t0 > t1 {
			return false;
		}
	}
	true
}

/// Möller-Trumbore: where along the ray (o, d) it meets the triangle, beyond a hundredth of a millimetre
pub fn ray_tri(o: V3, d: V3, t: &[V3; 3]) -> Option<f64> {
	let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
	let p = cross(d, e2);
	let det = dot(e1, p);
	if det.abs() < 1e-15 {
		return None;
	}
	let inv = 1.0 / det;
	let s = o - t[0];
	let u = dot(s, p) * inv;
	if !(0.0..=1.0).contains(&u) {
		return None;
	}
	let q = cross(s, e1);
	let v = dot(d, q) * inv;
	if v < 0.0 || u + v > 1.0 {
		return None;
	}
	let at = dot(e2, q) * inv;
	(at > 1e-5).then_some(at)
}

// ---------------------------------------------------------------- the verdicts

/// A sensor's state over the last periods
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorState {
	/// got light from a station it faces
	Lit,
	/// got light only from stations behind it (BACK_DEG): reflected
	Behind,
	/// dark in the last two periods while a station that lit the device should have lit it: covered or dead
	Missing,
	Dark,
}

impl SensorState {
	pub fn label(self) -> &'static str {
		match self {
			SensorState::Lit => "lit",
			SensorState::Behind => "lit from behind",
			SensorState::Missing => "dark, should be lit",
			SensorState::Dark => "dark",
		}
	}
}

/// A sensor and a station in the last period
#[derive(Clone, Copy, Debug)]
pub struct Cell {
	/// the station's hits on it in the last period
	pub hits: i32,
	/// the angle off its normal to the station (deg), NaN where the pose is not known
	pub angle: f64,
	/// in the station's field of view and reach
	pub in_view: bool,
	/// facing it and in its view, but the device's own body is in the way
	pub hidden: bool,
	/// the station lights the device and should light this sensor
	pub expected: bool,
	/// the station is behind it (its light on it was reflected)
	pub behind: bool,
}

impl Default for Cell {
	fn default() -> Cell {
		Cell { hits: 0, angle: f64::NAN, in_view: false, hidden: false, expected: false, behind: false }
	}
}

/// A device's sensors against the stations
#[derive(Clone, Debug, Default)]
pub struct Verdict {
	/// the stations (columns), and whether each lit the device in the last period
	pub stations: Vec<i32>,
	pub lights: Vec<bool>,
	/// per sensor: per station
	pub cells: Vec<Vec<Cell>>,
	pub states: Vec<SensorState>,
	/// the device was tracking at the last period's end, so where its sensors faced is known
	pub judged: bool,
	/// the period's length (s)
	pub period: f64,
}

impl Verdict {
	pub fn count(&self, s: SensorState) -> usize {
		self.states.iter().filter(|&&x| x == s).count()
	}

	/// Sensors some station that lights the device should light now
	pub fn expected(&self) -> usize {
		self.cells.iter().filter(|r| r.iter().any(|c| c.expected)).count()
	}
}

/// A sensor against a station at one pose of the device
#[derive(Clone, Copy, Debug)]
struct Seen {
	angle: f64,
	in_view: bool,
	hidden: bool,
}

impl Seen {
	fn expected(&self) -> bool {
		self.angle < EXPECT_DEG && self.in_view && !self.hidden
	}

	fn behind(&self) -> bool {
		self.angle > BACK_DEG
	}
}

/// How each sensor (in the head frame) sees each station at a pose of the device; the body tested only where it
/// matters (facing the station and in its view). `body`: the device's model and the head frame into the model's
fn seen_at(sensors: &[(V3, V3)], stations: &[StationAt], pose: &Pose, body: Option<(&Bvh, Q4)>) -> Vec<Vec<Seen>> {
	let into_model = body.map(|(_, off)| qconj(off));
	sensors
		.iter()
		.map(|&(p, n)| {
			let (pw, nw) = (pose.p + qrot(pose.q, p), qrot(pose.q, n));
			stations
				.iter()
				.map(|st| {
					let (angle, in_view, u) = look(pw, nw, st);
					let mut hidden = false;
					if let (Some((bvh, _)), Some(im)) = (body, into_model) {
						if angle < EXPECT_DEG && in_view {
							// in the model's frame: from where the sensor comes out of the shell, toward the station
							let uh = qrot(qconj(pose.q), u);
							let nm = qrot(im, n * (1.0 / len(n).max(1e-9)));
							hidden = bvh.hits(bvh.out_of(qrot(im, p), nm), qrot(im, uh), 1.0);
						}
					}
					Seen { angle, in_view, hidden }
				})
				.collect()
		})
		.collect()
}

/// The verdict on a device's sensors from its last H periods (`Device::periods`) against the stations, with its
/// render model for its own body where there is one
pub fn judge(d: &Device, stations: &[StationAt], body: Option<(&Bvh, Q4)>) -> Verdict {
	let sensors: Vec<(V3, V3)> = (0..d.sensor_pos.len()).filter_map(|i| d.sensor_in_head(i)).collect();
	let n = sensors.len().max(d.periods.iter().flat_map(|p| p.hits.values()).map(|v| v.len()).max().unwrap_or(0));
	let mut v = Verdict {
		stations: stations.iter().map(|s| s.idx).collect(),
		lights: vec![false; stations.len()],
		cells: vec![vec![Cell::default(); stations.len()]; n],
		states: vec![SensorState::Dark; n],
		judged: false,
		period: 0.0,
	};
	let Some(last) = d.periods.back() else { return v };
	v.period = last.period;
	let k = d.periods.len() - 1;
	// the periods back from the last: whether each is the one just before the next (no gap of a missed period)
	let joined = |i: usize| i >= 1 && d.periods[i].wall - d.periods[i - 1].wall < 2.5 * d.periods[i].period.max(0.5);
	let lit_now = |i: usize| last.hits.values().any(|h| h.get(i).is_some_and(|&c| c > 0));
	// where the sensors faced at each period's end, the ones the device was tracking at
	let have_geom = !sensors.is_empty();
	let at: Vec<Option<Vec<Vec<Seen>>>> = d
		.periods
		.iter()
		.map(|p| (have_geom && p.tracking).then(|| seen_at(&sensors, stations, &p.pose, body)))
		.collect();
	// a station lights the device in a period: LIGHTS_MIN of its sensors from the front (all of them where the pose
	// is not known)
	let lights = |pi: usize, j: usize| -> bool {
		let p = &d.periods[pi];
		let Some(h) = p.hits.get(&stations[j].idx) else { return false };
		let front = h
			.iter()
			.enumerate()
			.filter(|&(i, &c)| c > 0 && at[pi].as_ref().is_none_or(|g| g.get(i).is_none_or(|r| !r[j].behind())))
			.count();
		front >= LIGHTS_MIN
	};
	for j in 0..stations.len() {
		v.lights[j] = lights(k, j);
	}
	v.judged = at[k].is_some();
	for i in 0..n {
		for (j, st) in stations.iter().enumerate() {
			let c = &mut v.cells[i][j];
			c.hits = last.hits_of(st.idx, i);
			if let Some(g) = at[k].as_ref().and_then(|g| g.get(i)) {
				let s = g[j];
				c.angle = s.angle;
				c.in_view = s.in_view;
				c.hidden = s.hidden;
				c.behind = s.behind();
				c.expected = v.lights[j] && s.expected();
			}
		}
		// lit: by a station it faces; from behind: only by stations behind it (at both ends of the period, where the
		// one before is known)
		if lit_now(i) {
			let behind_all = v.judged
				&& v.cells[i].iter().enumerate().filter(|(_, c)| c.hits > 0).all(|(j, c)| {
					c.behind
						&& (!joined(k) || at[k - 1].as_ref().is_none_or(|g| g.get(i).is_none_or(|r| r[j].behind())))
				});
			v.states[i] = if behind_all { SensorState::Behind } else { SensorState::Lit };
			continue;
		}
		// missing: dark in the last two periods (no station's light at all) while a station that lit the device in both
		// should have lit it at both ends of both: three poses of the device, all tracking
		if k >= 2 && joined(k) && joined(k - 1) {
			let prev = &d.periods[k - 1];
			let dark_before = prev.hits.values().all(|h| h.get(i).is_none_or(|&c| c == 0));
			let poses = [&at[k], &at[k - 1], &at[k - 2]];
			if dark_before && poses.iter().all(|g| g.is_some()) {
				let missing = (0..stations.len()).any(|j| {
					lights(k, j)
						&& lights(k - 1, j)
						&& poses.iter().all(|g| g.as_ref().and_then(|g| g.get(i)).is_some_and(|r| r[j].expected()))
				});
				if missing {
					v.states[i] = SensorState::Missing;
				}
			}
		}
	}
	v
}

/// The stations as the runtime has them, for `judge`: the placed ones
pub fn stations_of(rt: &crate::runtime::Runtime) -> Vec<StationAt> {
	rt.stations
		.values()
		.filter(|s| s.have_pose)
		.map(|s| StationAt { idx: s.idx, pose: s.pose, gen2: s.mode >= 0 })
		.collect()
}

/// The models' trees by name, made once each
#[derive(Default)]
pub struct Bodies {
	trees: BTreeMap<String, std::sync::Arc<Bvh>>,
}

impl Bodies {
	pub fn get(&mut self, name: &str, m: &crate::models::Model) -> std::sync::Arc<Bvh> {
		self.trees.entry(name.to_string()).or_insert_with(|| std::sync::Arc::new(Bvh::new(&m.tris))).clone()
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::runtime::LightPeriod;
	use sally_math::math3d::{norm, qnorm};

	/// A box of half side `h` about the origin, its 12 triangles
	fn cube(h: f64) -> Vec<[V3; 3]> {
		let c = |i: usize| {
			V3::new(if i & 1 != 0 { h } else { -h }, if i & 2 != 0 { h } else { -h }, if i & 4 != 0 { h } else { -h })
		};
		let faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]];
		faces.iter().flat_map(|f| [[c(f[0]), c(f[1]), c(f[2])], [c(f[0]), c(f[2]), c(f[3])]]).collect()
	}

	#[test]
	fn a_ray_meets_a_triangle_in_front_only() {
		let t = [V3::new(-1.0, -1.0, 1.0), V3::new(1.0, -1.0, 1.0), V3::new(0.0, 1.0, 1.0)];
		let at = ray_tri(V3::default(), V3::new(0.0, 0.0, 1.0), &t).unwrap();
		assert!((at - 1.0).abs() < 1e-12);
		assert!(ray_tri(V3::default(), V3::new(0.0, 0.0, -1.0), &t).is_none());
		assert!(ray_tri(V3::new(5.0, 0.0, 0.0), V3::new(0.0, 0.0, 1.0), &t).is_none());
	}

	#[test]
	fn the_tree_finds_what_every_triangle_would() {
		// many small boxes in a row: the tree's answer is the brute force's, ray after ray
		let mut tris = Vec::new();
		for k in 0..40 {
			for t in cube(0.01) {
				let o = V3::new(k as f64 * 0.05, (k % 3) as f64 * 0.02, 0.0);
				tris.push([t[0] + o, t[1] + o, t[2] + o]);
			}
		}
		let bvh = Bvh::new(&tris);
		assert_eq!(bvh.triangles(), tris.len());
		let mut seed = 7u64;
		let mut rnd = || {
			seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
			((seed >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0
		};
		for _ in 0..500 {
			let o = V3::new(rnd() * 1.2 + 1.0, rnd() * 0.2, rnd() * 0.2);
			let d = norm(V3::new(rnd(), rnd(), rnd()));
			let brute = tris.iter().any(|t| ray_tri(o, d, t).is_some_and(|x| x < 0.5));
			assert_eq!(bvh.hits(o, d, 0.5), brute);
		}
	}

	fn station(idx: i32, p: V3) -> StationAt {
		// looking at the origin: its -Z toward it
		let f = norm(V3::default() - p);
		let z = f * -1.0;
		let x = norm(cross(V3::new(0.0, 0.0, 1.0), z));
		let y = cross(z, x);
		// the rotation whose columns are x, y, z
		let m = [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]];
		let w = (1.0 + m[0][0] + m[1][1] + m[2][2]).max(1e-12).sqrt() / 2.0;
		let q = qnorm(Q4::new(
			w,
			(m[2][1] - m[1][2]) / (4.0 * w),
			(m[0][2] - m[2][0]) / (4.0 * w),
			(m[1][0] - m[0][1]) / (4.0 * w),
		));
		StationAt { idx, pose: Pose { p, q }, gen2: true }
	}

	#[test]
	fn a_sensor_sees_a_station_it_faces_inside_its_view() {
		let st = station(0, V3::new(3.0, 0.0, 0.0));
		let (a, in_view, _) = look(V3::default(), V3::new(1.0, 0.0, 0.0), &st);
		assert!(a.abs() < 1e-6 && in_view);
		let (a, _, _) = look(V3::default(), V3::new(-1.0, 0.0, 0.0), &st);
		assert!((a - 180.0).abs() < 1e-6);
		// off to the side of the station's view: 80 deg round from its axis is past its 75
		let side = V3::new(3.0, 0.0, 0.0) + qrot(st.pose.q, norm(V3::new(80f64.to_radians().tan(), 0.0, -1.0)) * 2.0);
		let (_, in_view, _) = look(side, V3::new(1.0, 0.0, 0.0), &st);
		assert!(!in_view);
	}

	/// A device of five sensors on a box's faces (+x, -x, +y, +z, -y), no head offset; its periods (tracking, each
	/// station's hits), all at the origin unturned
	fn device(periods: &[(bool, [&[i32]; 2])]) -> Device {
		let mut d = Device::new("KN0", 0.0);
		let h = 0.02;
		let faces = [
			V3::new(1.0, 0.0, 0.0),
			V3::new(-1.0, 0.0, 0.0),
			V3::new(0.0, 1.0, 0.0),
			V3::new(0.0, 0.0, 1.0),
			V3::new(0.0, -1.0, 0.0),
		];
		d.sensor_pos = faces.iter().map(|f| *f * h).collect();
		d.sensor_nrm = faces.to_vec();
		for (k, (tracking, hits)) in periods.iter().enumerate() {
			let mut p = LightPeriod { wall: k as f64, period: 1.0, tracking: *tracking, ..LightPeriod::default() };
			for (lh, h) in hits.iter().enumerate() {
				if !h.is_empty() {
					p.hits.insert(lh as i32, h.to_vec());
				}
			}
			d.periods.push_back(p);
		}
		d
	}

	#[test]
	fn a_dark_sensor_facing_a_station_that_lights_the_device_is_missing() {
		// station 0 on the +x side (sensor 0 faces it at 37 deg, sensor 3 at 53, sensors 2 and 4 at 90, sensor 1 is
		// behind), station 1 on the -x side
		let st = [station(0, V3::new(2.0, 0.0, 1.5)), station(1, V3::new(-2.0, 0.0, 1.5))];
		let lit1: &[i32] = &[0, 9, 9, 9, 9];
		// station 0 lit only two sensors: it does not light the device, so nothing is missing for it
		let two: &[i32] = &[0, 0, 0, 9, 9];
		let d = device(&[(true, [two, lit1]), (true, [two, lit1]), (true, [two, lit1])]);
		let v = judge(&d, &st, None);
		assert!(v.judged && v.lights == vec![false, true], "{:?}", v.lights);
		assert_eq!(v.states[0], SensorState::Dark);
		// three: sensor 0, dark in both periods while station 0 should have lit it all along, is missing
		let three: &[i32] = &[0, 0, 9, 9, 9];
		let d = device(&[(true, [three, lit1]), (true, [three, lit1]), (true, [three, lit1])]);
		let v = judge(&d, &st, None);
		assert!(v.lights[0]);
		assert_eq!(v.states[0], SensorState::Missing);
		assert!(v.cells[0][0].expected && !v.cells[0][1].expected && v.cells[0][1].behind);
		assert!(!v.cells[2][0].expected, "90 deg round is past EXPECT_DEG");
		assert_eq!(v.states[2], SensorState::Lit);
		assert_eq!((v.count(SensorState::Missing), v.expected()), (1, 3));
		// lit once in the period before: not missing yet
		let once: &[i32] = &[3, 0, 9, 9, 9];
		let d = device(&[(true, [three, lit1]), (true, [once, lit1]), (true, [three, lit1])]);
		assert_eq!(judge(&d, &st, None).states[0], SensorState::Dark);
		// not tracking at one of the ends: not judged missing
		let d = device(&[(false, [three, lit1]), (true, [three, lit1]), (true, [three, lit1])]);
		assert_eq!(judge(&d, &st, None).states[0], SensorState::Dark);
		// two periods only: not yet either
		let d = device(&[(true, [three, lit1]), (true, [three, lit1])]);
		assert_eq!(judge(&d, &st, None).states[0], SensorState::Dark);
	}

	#[test]
	fn light_only_from_a_station_behind_a_sensor_is_reflected() {
		// station 0 on +x: sensor 1 (facing -x) lit by it is lit from behind; sensor 0 lit by it is lit
		let st = [station(0, V3::new(2.0, 0.0, 0.2))];
		let hits: &[i32] = &[5, 2, 5, 5, 5];
		let d = device(&[(true, [hits, &[]]), (true, [hits, &[]])]);
		let v = judge(&d, &st, None);
		assert_eq!(v.states[0], SensorState::Lit);
		assert_eq!(v.states[1], SensorState::Behind);
		assert!(v.cells[1][0].behind);
	}

	#[test]
	fn a_ray_starts_where_the_sensor_comes_out_of_its_shell() {
		// a sensor 5 mm inside a box of half side 1 cm, facing +z: it comes out at the box's top
		let bvh = Bvh::new(&cube(0.01));
		let o = bvh.out_of(V3::new(0.0, 0.0, 0.005), V3::new(0.0, 0.0, 1.0));
		assert!((o.z - (0.01 + 1e-4 + RAY_START_M)).abs() < 1e-9, "{o:?}");
		assert!(!bvh.hits(o, norm(V3::new(1.0, 0.0, 1.0)), 1.0));
		// one outside it stays where it is, RAY_START_M out
		let o = bvh.out_of(V3::new(0.0, 0.0, 0.02), V3::new(0.0, 0.0, 1.0));
		assert!((o.z - (0.02 + RAY_START_M)).abs() < 1e-9);
		assert_eq!(
			bvh.first(V3::new(0.0, 0.0, -0.05), V3::new(0.0, 0.0, 1.0), 1.0).map(|t| (t * 1e6).round()),
			Some(40000.0)
		);
	}

	#[test]
	fn the_device_s_own_body_hides_a_sensor_behind_it() {
		// a sensor on top facing up and outward a little, a wall of the body between it and the station
		let st = [station(0, V3::new(2.0, 0.0, 0.3))];
		let mut d = device(&[(true, [&[9, 0, 9, 9, 9], &[]])]);
		d.sensor_pos[3] = V3::new(-0.03, 0.0, 0.0);
		d.sensor_nrm[3] = norm(V3::new(1.0, 0.0, 0.3));
		// the box of half side 1 cm at the origin stands between it and +x
		let bvh = Bvh::new(&cube(0.01));
		let v = judge(&d, &st, Some((&bvh, Q4::default())));
		assert!(v.cells[3][0].hidden && !v.cells[3][0].expected);
		assert!(!v.cells[0][0].hidden);
		let v = judge(&d, &st, None);
		assert!(!v.cells[3][0].hidden);
	}
}
