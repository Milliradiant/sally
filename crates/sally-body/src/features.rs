//! What the virtual trackers' model sees and says (docs/mocap_plan.md section 1): the same for a truth recording, a
//! synthetic session and, later, live.
//!
//! Calibrated at the T-pose: each device's turn taken out (its orientation from then on as the change since the
//! T-pose, put on the body's T-pose frame), so a tracker worn turned or on the back reads as one worn straight. Then
//! per frame, in a frame of the body's own: at the waist tracker, turned with the pelvis's heading (the waist
//! tracker's forward as in the T-pose, on the floor), lengths as shares of the stature:
//!
//! - in: the input devices (the headset, both controllers, the waist and both feet; with a real chest tracker, that
//!   too), now and 5 ... 30 frames back (0.5 s at 60 Hz), each as its position and its orientation's first two columns
//!   (6D); where every device sat in the T-pose (from the waist, in the body's frame), the targets too (where the
//!   virtual trackers are worn); the stature;
//! - out: the targets (the chest, knees and elbows; the knees and elbows where the chest is an input), each as
//!   position and 6D orientation; the position as the change from where the target sat in the T-pose (from the waist,
//!   in the body's frame), so a tracker worn higher or wider than any in training is carried over as worn instead of
//!   pulled toward the training's average place.
//!
//! Which devices are in and out is a model's `Layout` (its model.txt says it); `Layout::first` is the six in and five
//! out. Decoded back to each target tracker's own pose with its T-pose place and turn put back.

use sally_math::math3d::{cross, dot, len, norm, qconj, qfrom_mat, qmul, qrot, Pose, Q4, V3};

use crate::poses::Session;
use crate::roles::{Role, Roles};

/// The first models' inputs and targets, in this order (the targets' T-pose places are inputs too)
pub const INPUTS: [Role; 6] = [Role::Head, Role::LHand, Role::RHand, Role::Waist, Role::LFoot, Role::RFoot];
pub const TARGETS: [Role; 5] = [Role::Chest, Role::LKnee, Role::RKnee, Role::LElbow, Role::RElbow];
/// How far back each input is seen, frames at 60 Hz
pub const LAGS: [usize; 7] = [0, 5, 10, 15, 20, 25, 30];
pub const FPS: f64 = 60.0;
/// per device and lag: position 3, 6D 6
pub const PER: usize = 9;
/// What the targets are, written with a model so it is never decoded as another's
pub const TARGETS_ARE: &str = "position from the T-pose place, 6D orientation";

/// The short names model.txt, places.txt and the driver's T lines use
pub fn short_name(r: Role) -> &'static str {
	match r {
		Role::Head => "head",
		Role::LHand => "lhand",
		Role::RHand => "rhand",
		Role::Waist => "waist",
		Role::LFoot => "lfoot",
		Role::RFoot => "rfoot",
		Role::Chest => "chest",
		Role::LKnee => "lknee",
		Role::RKnee => "rknee",
		Role::LElbow => "lelbow",
		Role::RElbow => "relbow",
		Role::HeadTracker => "headtracker",
	}
}

/// A role by its short name
pub fn by_short_name(n: &str) -> Option<Role> {
	[
		Role::Head,
		Role::LHand,
		Role::RHand,
		Role::Waist,
		Role::LFoot,
		Role::RFoot,
		Role::Chest,
		Role::LKnee,
		Role::RKnee,
		Role::LElbow,
		Role::RElbow,
	]
	.into_iter()
	.find(|&r| short_name(r) == n)
}

/// What a model sees and says: its input devices and its targets, in order. The headset is the first input and the
/// waist an input (the body's frame rides it)
#[derive(Clone, Debug, PartialEq)]
pub struct Layout {
	pub inputs: Vec<Role>,
	pub targets: Vec<Role>,
}

impl Layout {
	/// The six of ordinary full-body tracking in, the chest, knees and elbows out
	pub fn first() -> Layout {
		Layout { inputs: INPUTS.to_vec(), targets: TARGETS.to_vec() }
	}

	/// A real chest tracker in too, the knees and elbows out
	pub fn with_chest() -> Layout {
		let mut inputs = INPUTS.to_vec();
		inputs.push(Role::Chest);
		Layout { inputs, targets: vec![Role::LKnee, Role::RKnee, Role::LElbow, Role::RElbow] }
	}

	pub fn n_in(&self) -> usize {
		self.inputs.len() * LAGS.len() * PER + (self.inputs.len() + self.targets.len()) * 3 + 1
	}

	pub fn n_out(&self) -> usize {
		self.targets.len() * PER
	}

	/// Where a role is among the inputs
	pub fn input(&self, r: Role) -> Option<usize> {
		self.inputs.iter().position(|&x| x == r)
	}

	/// Where a role is among the targets
	pub fn target(&self, r: Role) -> Option<usize> {
		self.targets.iter().position(|&x| x == r)
	}

	fn waist(&self) -> usize {
		self.input(Role::Waist).expect("a layout has the waist in")
	}
}

/// A session's calibration
#[derive(Clone, Debug)]
pub struct Calib {
	pub layout: Layout,
	/// the devices of the inputs then the targets, by index into the session's
	pub dev: Vec<usize>,
	/// each device's turn taken out: q(t) * c is its orientation as the body's in the T-pose
	pub c: Vec<Q4>,
	/// the waist tracker's forward in its own frame
	pub fwd_waist: V3,
	/// where each device sat in the T-pose, from the waist in the body's frame, in statures
	pub tpose: Vec<V3>,
	/// m
	pub stature: f64,
}

impl Calib {
	/// The waist tracker's device
	pub fn waist(&self) -> usize {
		self.dev[self.layout.waist()]
	}

	/// The device of a target's truth tracker (a live calibration's: none worn, the waist's)
	pub fn target_dev(&self, k: usize) -> usize {
		self.dev[self.layout.inputs.len() + k]
	}
}

/// A rotation from its three axes (columns)
fn axes(x: V3, y: V3, z: V3) -> Q4 {
	qfrom_mat(&[[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]])
}

fn pose_at(s: &Session, k: usize, t: f64) -> Option<Pose> {
	let d = &s.devices[k];
	if d.t.is_empty() || t < d.t[0] - 1e-6 || t > d.t[d.t.len() - 1] + 1e-6 {
		return None;
	}
	Some(d.pose[d.at(t + 1e-6)])
}

pub fn calibrate(s: &Session, r: &Roles, layout: &Layout) -> Result<Calib, String> {
	let roles: Vec<Role> = layout.inputs.iter().chain(layout.targets.iter()).copied().collect();
	let dev: Vec<usize> = roles
		.iter()
		.map(|role| r.device(*role).ok_or_else(|| format!("no {} found in the T-pose", role.name())))
		.collect::<Result<_, _>>()?;
	let up = V3::new(0.0, 1.0, 0.0);
	let body = axes(r.right, up, -r.forward);
	let t = r.tpose_t;
	let at = |k: usize| pose_at(s, k, t).ok_or("a device without a pose in the T-pose");
	let waist = at(dev[layout.waist()])?;
	let stature = (r.eye + 0.06) / 0.936;
	let n = dev.len();
	let mut c = Calib {
		layout: layout.clone(),
		dev,
		c: vec![Q4::default(); n],
		fwd_waist: V3::default(),
		tpose: vec![V3::default(); n],
		stature,
	};
	for k in 0..n {
		let p = at(c.dev[k])?;
		c.c[k] = qmul(qconj(p.q), body);
		c.tpose[k] = qrot(qconj(body), p.p - waist.p) * (1.0 / stature);
	}
	c.fwd_waist = qrot(qconj(waist.q), r.forward);
	Ok(c)
}

/// A live session's calibration, where the targets are the virtual trackers (none is worn): the inputs as `calibrate`
/// takes them, each target where `places` puts it in the T-pose (from the headset in the body's frame, in statures:
/// where the real one was worn in the truth recordings; from the headset, so wherever the waist tracker is worn that
/// day) and turned as the body was then (its turn the identity). A target's `dev` is the waist's, never read
pub fn calibrate_live(s: &Session, r: &Roles, layout: &Layout, places: &[V3]) -> Result<Calib, String> {
	if places.len() != layout.targets.len() {
		return Err(format!("{} places for {} virtual trackers", places.len(), layout.targets.len()));
	}
	let mut dev: Vec<usize> = layout
		.inputs
		.iter()
		.map(|role| r.device(*role).ok_or_else(|| format!("no {} found in the T-pose", role.name())))
		.collect::<Result<_, _>>()?;
	let ni = dev.len();
	let w = dev[layout.waist()];
	dev.extend(std::iter::repeat_n(w, layout.targets.len()));
	let up = V3::new(0.0, 1.0, 0.0);
	let body = axes(r.right, up, -r.forward);
	let t = r.tpose_t;
	let at = |k: usize| pose_at(s, k, t).ok_or("a device without a pose in the T-pose");
	let waist = at(w)?;
	let stature = (r.eye + 0.06) / 0.936;
	let n = dev.len();
	let mut c = Calib {
		layout: layout.clone(),
		dev,
		c: vec![Q4::default(); n],
		fwd_waist: V3::default(),
		tpose: vec![V3::default(); n],
		stature,
	};
	for k in 0..ni {
		let p = at(c.dev[k])?;
		c.c[k] = qmul(qconj(p.q), body);
		c.tpose[k] = qrot(qconj(body), p.p - waist.p) * (1.0 / stature);
	}
	for (k, p) in places.iter().enumerate() {
		c.tpose[ni + k] = c.tpose[0] + *p;
	}
	c.fwd_waist = qrot(qconj(waist.q), r.forward);
	Ok(c)
}

/// The body's frame at a frame: the waist tracker's place, the pelvis's heading
pub fn reference(c: &Calib, waist: &Pose) -> Pose {
	let up = V3::new(0.0, 1.0, 0.0);
	let f = qrot(waist.q, c.fwd_waist);
	let mut h = norm(V3::new(f.x, 0.0, f.z));
	if len(h) < 0.5 {
		h = V3::new(0.0, 0.0, -1.0);
	}
	Pose { p: waist.p, q: axes(cross(h, up), up, -h) }
}

fn six_d(q: Q4) -> [f64; 6] {
	let x = qrot(q, V3::new(1.0, 0.0, 0.0));
	let y = qrot(q, V3::new(0.0, 1.0, 0.0));
	[x.x, x.y, x.z, y.x, y.y, y.z]
}

/// A rotation from its 6D (Gram-Schmidt on the two columns)
pub fn from_six_d(v: &[f32]) -> Q4 {
	let a = V3::new(v[0] as f64, v[1] as f64, v[2] as f64);
	let b = V3::new(v[3] as f64, v[4] as f64, v[5] as f64);
	let x = norm(a);
	let y = norm(b - x * dot(x, b));
	axes(x, y, cross(x, y))
}

/// One device's pose in the body's frame: position in statures, the calibrated orientation's 6D
fn encode(c: &Calib, frame: &Pose, k: usize, p: &Pose, out: &mut Vec<f32>) {
	let rp = qrot(qconj(frame.q), p.p - frame.p) * (1.0 / c.stature);
	let rq = qmul(qconj(frame.q), qmul(p.q, c.c[k]));
	out.extend([rp.x as f32, rp.y as f32, rp.z as f32]);
	out.extend(six_d(rq).map(|v| v as f32));
}

/// The inputs at time `t` (None where a device has no pose that far back)
pub fn inputs(s: &Session, c: &Calib, t: f64) -> Option<(Vec<f32>, Pose)> {
	let frame = reference(c, &pose_at(s, c.waist(), t)?);
	let mut out = Vec::with_capacity(c.layout.n_in());
	for k in 0..c.layout.inputs.len() {
		for &lag in &LAGS {
			let p = pose_at(s, c.dev[k], t - lag as f64 / FPS)?;
			encode(c, &frame, k, &p, &mut out);
		}
	}
	for v in &c.tpose {
		out.extend([v.x as f32, v.y as f32, v.z as f32]);
	}
	out.push(c.stature as f32);
	Some((out, frame))
}

/// The targets at time `t`, in the frame `inputs` gave
pub fn targets(s: &Session, c: &Calib, frame: &Pose, t: f64) -> Option<Vec<f32>> {
	let mut out = Vec::with_capacity(c.layout.n_out());
	for k in 0..c.layout.targets.len() {
		let j = c.layout.inputs.len() + k;
		let p = pose_at(s, c.dev[j], t)?;
		encode(c, frame, j, &p, &mut out);
		let at = out.len() - PER;
		for (a, v) in [c.tpose[j].x, c.tpose[j].y, c.tpose[j].z].iter().enumerate() {
			out[at + a] -= *v as f32;
		}
	}
	Some(out)
}

/// The model's output back to each target tracker's pose
pub fn decode(c: &Calib, frame: &Pose, out: &[f32]) -> Vec<Pose> {
	let ni = c.layout.inputs.len();
	(0..c.layout.targets.len())
		.map(|k| {
			let o = &out[k * PER..(k + 1) * PER];
			let rp = (V3::new(o[0] as f64, o[1] as f64, o[2] as f64) + c.tpose[ni + k]) * c.stature;
			let rq = from_six_d(&o[3..9]);
			let q = qmul(qmul(frame.q, rq), qconj(c.c[ni + k]));
			Pose { p: frame.p + qrot(frame.q, rp), q }
		})
		.collect()
}

#[cfg(test)]
mod tests {
	use super::*;
	use sally_math::math3d::qangle;

	#[test]
	fn six_d_round_trips() {
		let q = sally_math::math3d::qnorm(Q4::new(0.3, -0.5, 0.7, 0.2));
		let v: Vec<f32> = six_d(q).iter().map(|x| *x as f32).collect();
		let back = from_six_d(&v);
		assert!(qangle(qmul(qconj(q), back)) < 1e-6);
	}

	#[test]
	fn targets_decode_back() {
		use crate::poses::Device;
		for layout in [Layout::first(), Layout::with_chest()] {
			let n = layout.inputs.len() + layout.targets.len();
			let mut s = Session { start_epoch: 0.0, devices: Vec::new() };
			for k in 0..n {
				let f = k as f64;
				let q = sally_math::math3d::qnorm(Q4::new(1.0, 0.1 * f, -0.05 * f, 0.02 * f));
				let p = Pose { p: V3::new(0.1 * f, 1.0 - 0.05 * f, -0.02 * f), q };
				// a second back too, for the inputs' history
				s.devices.push(Device {
					serial: format!("D{k}"),
					class: "tracker".into(),
					t: vec![-1.0, 0.0],
					pose: vec![p, p],
					ok: vec![true, true],
				});
			}
			let mut c = Calib {
				layout: layout.clone(),
				dev: (0..n).collect(),
				c: vec![Q4::default(); n],
				fwd_waist: V3::new(0.0, 0.0, -1.0),
				tpose: vec![V3::default(); n],
				stature: 1.7,
			};
			for k in 0..n {
				c.c[k] = sally_math::math3d::qnorm(Q4::new(0.9, 0.0, 0.3, -0.1 * k as f64));
				c.tpose[k] = V3::new(0.01 * k as f64, -0.2, 0.03);
			}
			let frame = Pose { p: V3::new(0.2, 0.9, 0.1), q: sally_math::math3d::qnorm(Q4::new(0.8, 0.0, 0.6, 0.0)) };
			let out = targets(&s, &c, &frame, 0.0).unwrap();
			assert_eq!(out.len(), layout.n_out());
			let back = decode(&c, &frame, &out);
			for (k, b) in back.iter().enumerate() {
				let want = s.devices[layout.inputs.len() + k].pose[1];
				assert!(len(b.p - want.p) < 1e-5, "{k}: {:?} {:?}", b.p, want.p);
				assert!(qangle(qmul(qconj(want.q), b.q)) < 1e-5, "{k}");
			}
			let (x, _) = inputs(&s, &c, 0.0).unwrap();
			assert_eq!(x.len(), layout.n_in());
		}
	}

	#[test]
	fn the_sizes_add_up() {
		assert_eq!(Layout::first().n_in(), 6 * 7 * 9 + 11 * 3 + 1);
		assert_eq!(Layout::first().n_out(), 45);
		let l = Layout::with_chest();
		assert_eq!((l.n_in(), l.n_out()), (7 * 7 * 9 + 11 * 3 + 1, 36));
		assert_eq!((l.input(Role::Waist), l.input(Role::Chest), l.target(Role::LElbow)), (Some(3), Some(6), Some(2)));
		for r in l.inputs.iter().chain(&l.targets) {
			assert_eq!(by_short_name(short_name(*r)), Some(*r));
		}
	}
}
