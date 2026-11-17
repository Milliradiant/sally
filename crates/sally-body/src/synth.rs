//! The synthetic rig (docs/mocap_plan.md section 2, "making training data from mocap"): a headset, two controllers, a
//! waist and two foot trackers, and the five truth trackers (chest, knees, elbows) put on a motion-captured body where
//! they are worn, and moved with it. What comes out is a session like a truth recording, a T-pose first, so the same
//! code finds its body parts, calibrates and scores it.
//!
//! Each device rides one bone: the headset the head, a controller its hand, the waist tracker the pelvis, a foot
//! tracker its foot, the chest tracker the chest, a knee's tracker the thigh, an elbow's the upper arm. In the T-pose
//! it sits off a joint by a few centimetres (in front of the pelvis, above the knee, behind the elbow ...) and is
//! turned as a real one was in a truth recording's T-pose (a template, picked per session: the waist and chest trackers
//! on the back as in the crawling session too). Then a jitter: the body's size, each mount's place and turn. And what
//! the mocap lacks: a strap on skin and muscle moves under its tracker (a slow wander about its mount, most on a thigh
//! or an upper arm, least on the headset), and the tracking itself is noisy.

use std::path::Path;

use sally_math::math3d::{compose, cross, inverse, len, norm, qfrom_mat, qmul, Pose, Q4, V3};

use crate::bvh::Bvh;
use crate::poses::{Device, Session};

/// The devices, in the order of the mount templates: head, hands, waist, feet, chest, knees, elbows
pub const DEVICES: [&str; 11] = [
	"head",
	"left_hand",
	"right_hand",
	"waist",
	"left_foot",
	"right_foot",
	"chest",
	"left_knee",
	"right_knee",
	"left_elbow",
	"right_elbow",
];
const CLASS: [&str; 11] = [
	"hmd",
	"controller",
	"controller",
	"tracker",
	"tracker",
	"tracker",
	"tracker",
	"tracker",
	"tracker",
	"tracker",
	"tracker",
];

/// The joints a rig needs, by index into the BVH's
#[derive(Clone, Copy, Debug)]
pub struct Skeleton {
	pub pelvis: usize,
	pub hip: [usize; 2],
	pub knee: [usize; 2],
	pub ankle: [usize; 2],
	pub chest: usize,
	pub head: usize,
	pub shoulder: [usize; 2],
	pub elbow: [usize; 2],
	pub wrist: [usize; 2],
	/// the T-pose is the rest pose (Xsens), else the first frame (CMU's conversion puts one there)
	pub tpose_rest: bool,
}

/// The two datasets' joint names: SparsePoser's Xsens and CMU's 2010 MotionBuilder-friendly conversion
pub fn skeleton(b: &Bvh) -> Result<Skeleton, String> {
	let i = |n: &str| b.index(n).ok_or_else(|| format!("no joint {n}"));
	if b.index("LeftWrist").is_some() {
		Ok(Skeleton {
			pelvis: i("Hips")?,
			hip: [i("LeftHip")?, i("RightHip")?],
			knee: [i("LeftKnee")?, i("RightKnee")?],
			ankle: [i("LeftAnkle")?, i("RightAnkle")?],
			chest: i("Chest4")?,
			head: i("Head")?,
			shoulder: [i("LeftShoulder")?, i("RightShoulder")?],
			elbow: [i("LeftElbow")?, i("RightElbow")?],
			wrist: [i("LeftWrist")?, i("RightWrist")?],
			tpose_rest: true,
		})
	} else {
		Ok(Skeleton {
			pelvis: i("Hips")?,
			hip: [i("LeftUpLeg")?, i("RightUpLeg")?],
			knee: [i("LeftLeg")?, i("RightLeg")?],
			ankle: [i("LeftFoot")?, i("RightFoot")?],
			chest: i("Spine1")?,
			head: i("Head")?,
			shoulder: [i("LeftArm")?, i("RightArm")?],
			elbow: [i("LeftForeArm")?, i("RightForeArm")?],
			wrist: [i("LeftHand")?, i("RightHand")?],
			tpose_rest: false,
		})
	}
}

/// One recording's mounts: each device's turn in the body's T-pose frame (x right, y up, z back), and whether the
/// waist and chest trackers were on the back
#[derive(Clone, Debug)]
pub struct Mount {
	pub name: String,
	pub q: [Q4; 11],
	pub back: bool,
}

/// Reads the templates: `session device qw qx qy qz x y z` lines, the position from the headset's in the body's frame
pub fn load_mounts(path: &Path) -> Result<Vec<Mount>, String> {
	let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
	let mut out: Vec<Mount> = Vec::new();
	for l in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
		let w: Vec<&str> = l.split_whitespace().collect();
		let bad = || format!("{}: not `session device qw qx qy qz x y z`: {l}", path.display());
		if w.len() < 9 {
			return Err(bad());
		}
		let v: Vec<f64> = w[2..9].iter().map(|x| x.parse::<f64>()).collect::<Result<_, _>>().map_err(|_| bad())?;
		let k = DEVICES.iter().position(|d| *d == w[1]).ok_or_else(bad)?;
		if out.last().is_none_or(|m| m.name != w[0]) {
			out.push(Mount { name: w[0].to_string(), q: [Q4::default(); 11], back: false });
		}
		let m = out.last_mut().expect("pushed");
		m.q[k] = Q4::new(v[0], v[1], v[2], v[3]);
		// behind the headset by over 10 cm: on the back
		if (k == 3 || k == 6) && v[6] > 0.1 {
			m.back = true;
		}
	}
	Ok(out)
}

/// The jitter: the size as a share, a mount's place (m, each axis) and turn (rad); the wander of each mount (a share
/// of WANDER's) and the tracking's noise (m, rad)
#[derive(Clone, Copy, Debug)]
pub struct Jitter {
	pub size: f64,
	pub pos: f64,
	pub rot: f64,
	pub wander: f64,
	pub noise_pos: f64,
	pub noise_rot: f64,
}

/// How far a mount wanders about its place (sd, m and deg), per device in DEVICES' order: the headset strapped to the
/// skull, a controller in a hand's grip, the waist on the belly, a foot on the instep, the chest on the sternum, a knee
/// and an elbow on muscle
const WANDER: [(f64, f64); 11] = [
	(0.002, 1.0),
	(0.006, 4.0),
	(0.006, 4.0),
	(0.008, 4.0),
	(0.005, 3.0),
	(0.005, 3.0),
	(0.008, 4.0),
	(0.010, 6.0),
	(0.010, 6.0),
	(0.010, 6.0),
	(0.010, 6.0),
];
/// The wander's time constant, s
const WANDER_TAU: f64 = 0.5;

/// The jitter the training data is made with: sizes +-6 %, each mount +-4 cm and +-10 deg off its place (a tracker may
/// be worn anywhere on its body part; at +-2 cm real places sit outside what the model has seen), the wander at 0.3 of
/// WANDER's and 1.5 mm / 0.3 deg of tracking noise. The wander's share is what makes the IK baseline do as well on this
/// data as on the truth recordings (without it the synthetic knees are 10 mm off against the real 23-25; at 0.3, 23 mm;
/// at 0.5 and 1, 30 and 46 mm)
pub const JITTER: Jitter = Jitter {
	size: 0.06,
	pos: 0.04,
	rot: 10.0 * std::f64::consts::PI / 180.0,
	wander: 0.3,
	noise_pos: 0.0015,
	noise_rot: 0.3 * std::f64::consts::PI / 180.0,
};

/// A small seeded generator (xorshift64*), so a session is the same every time it is made
pub struct Rng(u64);

impl Rng {
	pub fn new(seed: u64) -> Rng {
		Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
	}
	/// uniform in [-1, 1)
	pub fn sym(&mut self) -> f64 {
		self.0 ^= self.0 >> 12;
		self.0 ^= self.0 << 25;
		self.0 ^= self.0 >> 27;
		let x = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11;
		x as f64 / (1u64 << 52) as f64 - 1.0
	}
	/// normal, sd 1 (Box-Muller)
	pub fn normal(&mut self) -> f64 {
		let u = (self.sym() + 1.0) / 2.0;
		let v = (self.sym() + 1.0) / 2.0;
		(-2.0 * (1.0 - u).ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
	}
	fn normal3(&mut self) -> V3 {
		V3::new(self.normal(), self.normal(), self.normal())
	}
}

fn horiz(v: V3) -> V3 {
	norm(V3::new(v.x, 0.0, v.z))
}

/// A rotation from its three axes (columns)
fn axes(x: V3, y: V3, z: V3) -> Q4 {
	qfrom_mat(&[[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]])
}

fn axis_angle(ax: V3, ang: f64) -> Q4 {
	let a = norm(ax);
	let (s, c) = ((ang / 2.0).sin(), (ang / 2.0).cos());
	Q4::new(c, a.x * s, a.y * s, a.z * s)
}

/// Where each device sits off its joint in the T-pose, in the body's frame (x right, y up, z back), m: the headset
/// in front of the head joint at the eyes, a controller just past the wrist, the waist tracker on the belly (or the
/// back), a foot's on the instep, the chest's on the sternum (or the back), a knee's on the thigh above the knee, an
/// elbow's on the back of the upper arm above the elbow
fn offset(k: usize, back: bool) -> V3 {
	match k {
		0 => V3::new(0.0, 0.08, -0.09),
		1 => V3::new(-0.03, -0.01, 0.0),
		2 => V3::new(0.03, -0.01, 0.0),
		3 => V3::new(0.0, 0.03, if back { 0.12 } else { -0.13 }),
		4 | 5 => V3::new(0.0, 0.02, -0.08),
		6 => V3::new(0.0, 0.0, if back { 0.11 } else { -0.11 }),
		7 | 8 => V3::new(0.0, 0.08, -0.06),
		9 => V3::new(0.07, 0.02, 0.04),
		_ => V3::new(-0.07, 0.02, 0.04),
	}
}

/// Where the limbs' joints are at one of a session's samples, in the room (left, right)
#[derive(Clone, Copy, Debug, Default)]
pub struct Joints {
	pub knee: [V3; 2],
	pub ankle: [V3; 2],
	pub shoulder: [V3; 2],
	pub elbow: [V3; 2],
	pub wrist: [V3; 2],
}

/// A session of the rig on the BVH's motion at 60 Hz, 2 s of T-pose first: its headset `eye` m above its lowest
/// tracker in the T-pose (before the size's jitter)
pub fn session(b: &Bvh, sk: &Skeleton, mount: &Mount, eye: f64, j: Jitter, seed: u64) -> Session {
	session_joints(b, sk, mount, eye, j, seed).0
}

/// `session`, and the limbs' joints at each of its samples (for the probes: a body part moved about its joint)
pub fn session_joints(b: &Bvh, sk: &Skeleton, mount: &Mount, eye: f64, j: Jitter, seed: u64) -> (Session, Vec<Joints>) {
	let mut rng = Rng::new(seed);
	let up = V3::new(0.0, 1.0, 0.0);
	let t0 = b.pose(0, 1.0, sk.tpose_rest);
	// the scale: the headset ~8 cm above the head joint, a foot tracker ~2 cm above the ankle
	let ankle = t0[sk.ankle[0]].p.y.min(t0[sk.ankle[1]].p.y);
	let target = eye * (1.0 + j.size * rng.sym());
	let scale = (target - 0.06) / (t0[sk.head].p.y - ankle);
	let tp = b.pose(0, scale, sk.tpose_rest);
	let right = horiz(tp[sk.wrist[1]].p - tp[sk.wrist[0]].p);
	let fwd = cross(up, right);
	let body = axes(right, up, -fwd);
	let rot_body = |v: V3| sally_math::math3d::qrot(body, v);
	// each device's bone, and the joint it sits off
	let bone = [
		sk.head,
		sk.wrist[0],
		sk.wrist[1],
		sk.pelvis,
		sk.ankle[0],
		sk.ankle[1],
		sk.chest,
		sk.hip[0],
		sk.hip[1],
		sk.shoulder[0],
		sk.shoulder[1],
	];
	let near = [
		sk.head,
		sk.wrist[0],
		sk.wrist[1],
		sk.pelvis,
		sk.ankle[0],
		sk.ankle[1],
		sk.chest,
		sk.knee[0],
		sk.knee[1],
		sk.elbow[0],
		sk.elbow[1],
	];
	let mut rel = [Pose::default(); 11];
	let mut tdev = [Pose::default(); 11];
	for k in 0..11 {
		let off = offset(k, mount.back) + V3::new(rng.sym(), rng.sym(), rng.sym()) * j.pos;
		let turn = axis_angle(V3::new(rng.sym(), rng.sym(), rng.sym()), j.rot * rng.sym());
		let p = tp[near[k]].p + rot_body(off);
		let q = qmul(qmul(body, mount.q[k]), turn);
		tdev[k] = Pose { p, q };
		rel[k] = compose(&inverse(&tp[bone[k]]), &tdev[k]);
	}
	// a turn about up and a place in the room, so nothing is learned from where the room's origin is
	let yaw = axis_angle(up, std::f64::consts::PI * rng.sym());
	let at = V3::new(rng.sym(), 0.0, rng.sym());
	let place = |p: &Pose| compose(&Pose { p: at, q: yaw }, p);
	let joints_of = |g: &[Pose]| {
		let at_ = |k: usize| place(&g[k]).p;
		Joints {
			knee: sk.knee.map(at_),
			ankle: sk.ankle.map(at_),
			shoulder: sk.shoulder.map(at_),
			elbow: sk.elbow.map(at_),
			wrist: sk.wrist.map(at_),
		}
	};
	let mut joints = Vec::new();
	let step = ((1.0 / 60.0) / b.frame_time.max(1e-6)).round().max(1.0) as usize;
	let n_t = 120;
	let frames: Vec<usize> = (0..b.frames()).step_by(step).collect();
	let n = n_t + frames.len();
	let mut s = Session { start_epoch: 0.0, devices: Vec::new() };
	for k in 0..11 {
		let mut d = Device {
			serial: format!("SYN-{}", DEVICES[k]),
			class: CLASS[k].to_string(),
			t: (0..n).map(|i| i as f64 / 60.0).collect(),
			pose: Vec::with_capacity(n),
			ok: vec![true; n],
		};
		for _ in 0..n_t {
			d.pose.push(place(&tdev[k]));
		}
		s.devices.push(d);
	}
	joints.resize(n_t, joints_of(&tp));
	// each mount's wander: a slow random walk pulled back to its place (in the device's frame)
	let a = (-1.0 / 60.0 / WANDER_TAU).exp();
	let b_ = (1.0 - a * a).sqrt();
	let mut wp = [V3::default(); 11];
	let mut wr = [V3::default(); 11];
	for &f in &frames {
		let g = b.pose(f, scale, false);
		joints.push(joints_of(&g));
		for k in 0..11 {
			let (sp, sr) = (WANDER[k].0 * j.wander, WANDER[k].1.to_radians() * j.wander);
			wp[k] = wp[k] * a + rng.normal3() * (b_ * sp);
			wr[k] = wr[k] * a + rng.normal3() * (b_ * sr);
			let dp = wp[k] + rng.normal3() * j.noise_pos;
			let dr = wr[k] + rng.normal3() * j.noise_rot;
			let wobble = Pose { p: dp, q: axis_angle(dr, len(dr)) };
			let p = compose(&compose(&g[bone[k]], &rel[k]), &wobble);
			s.devices[k].pose.push(place(&p));
		}
	}
	// a frame where the motion jumps (a broken file) leaves its devices not tracking
	for d in s.devices.iter_mut() {
		for i in 1..n {
			if len(d.pose[i].p - d.pose[i - 1].p) > 0.5 {
				d.ok[i] = false;
			}
		}
	}
	(s, joints)
}

/// The joints' own motions, which the mocap has little of (where an IK gives itself away: a foot turned with the knee
/// still, a hand twisted with the elbow still, an elbow swung with the hand in place): each a slow random turn (sd,
/// rad; time constant, s) about its joint, put on top of the mocap's own
#[derive(Clone, Copy, Debug)]
pub struct Free {
	/// the foot tracker about the ankle: up and down (the axis square to the shin and the foot), and rolled (about the
	/// line from the ankle to the tracker)
	pub foot: f64,
	pub foot_roll: f64,
	/// the controller about the forearm's axis through the wrist
	pub twist: f64,
	/// the upper arm's tracker and the controller about the line from the shoulder to the wrist, through the shoulder
	pub swing: f64,
	pub tau: f64,
}

/// What `train --free` puts on: the foot 15 deg up and down and 8 rolled, the forearm 30 deg, the swing 12 deg (sd),
/// changing over about 0.4 s
pub const FREE: Free = Free {
	foot: 15.0 * std::f64::consts::PI / 180.0,
	foot_roll: 8.0 * std::f64::consts::PI / 180.0,
	twist: 30.0 * std::f64::consts::PI / 180.0,
	swing: 12.0 * std::f64::consts::PI / 180.0,
	tau: 0.4,
};

fn rot_about(p: &Pose, o: V3, q: Q4) -> Pose {
	Pose { p: o + sally_math::math3d::qrot(q, p.p - o), q: qmul(q, p.q) }
}

/// The joints' free motions put on a session `session_joints` made (its devices in DEVICES' order), from the end of
/// its T-pose on, each starting from none; the truth trackers moved as the body would move them (the swing carries the
/// upper arm's), the rest as they were. Its own generator, so the session is otherwise the same
pub fn free_motions(s: &mut Session, joints: &[Joints], f: Free, seed: u64) {
	let mut rng = Rng::new(seed ^ 0x5EED_F00D);
	let a = (-1.0 / 60.0 / f.tau).exp();
	let b = (1.0 - a * a).sqrt();
	// per side: foot up-down, foot roll, twist, swing
	let mut ang = [[0.0f64; 4]; 2];
	let sd = [f.foot, f.foot_roll, f.twist, f.swing];
	for (i, j) in joints.iter().enumerate().skip(120) {
		for side in 0..2 {
			for (x, sd) in ang[side].iter_mut().zip(sd) {
				*x = *x * a + rng.normal() * b * sd;
			}
			let [up, roll, twist, swing] = ang[side];
			let (hand, foot, elbow) = (1 + side, 4 + side, 9 + side);
			let (ankle, knee) = (j.ankle[side], j.knee[side]);
			let fp = s.devices[foot].pose[i];
			let ax = cross(ankle - knee, fp.p - ankle);
			if len(ax) > 1e-6 && len(fp.p - ankle) > 1e-6 {
				let q = qmul(axis_angle(ax, up), axis_angle(fp.p - ankle, roll));
				s.devices[foot].pose[i] = rot_about(&fp, ankle, q);
			}
			let (sh, wr) = (j.shoulder[side], j.wrist[side]);
			if len(wr - sh) > 1e-6 {
				let qs = axis_angle(wr - sh, swing);
				s.devices[elbow].pose[i] = rot_about(&s.devices[elbow].pose[i], sh, qs);
				s.devices[hand].pose[i] = rot_about(&s.devices[hand].pose[i], sh, qs);
				let el = sh + sally_math::math3d::qrot(qs, j.elbow[side] - sh);
				if len(wr - el) > 1e-6 {
					s.devices[hand].pose[i] = rot_about(&s.devices[hand].pose[i], wr, axis_angle(wr - el, twist));
				}
			}
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_free_motions_leave_the_truth_as_the_body_would() {
		// a straight standing body: hip over knee over ankle, shoulder to elbow to wrist along x
		let n = 400;
		let pose = |p: V3| Pose { p, q: Q4::default() };
		let mut s = Session { start_epoch: 0.0, devices: Vec::new() };
		let at = [
			V3::new(0.0, 1.6, 0.0),
			V3::new(-0.6, 1.4, 0.0),
			V3::new(0.6, 1.4, 0.0),
			V3::new(0.0, 1.0, -0.1),
			V3::new(-0.1, 0.1, -0.08),
			V3::new(0.1, 0.1, -0.08),
			V3::new(0.0, 1.3, -0.1),
			V3::new(-0.1, 0.55, -0.06),
			V3::new(0.1, 0.55, -0.06),
			V3::new(-0.3, 1.42, 0.04),
			V3::new(0.3, 1.42, 0.04),
		];
		for p in at {
			s.devices.push(Device {
				serial: String::new(),
				class: String::new(),
				t: (0..n).map(|i| i as f64 / 60.0).collect(),
				pose: vec![pose(p); n],
				ok: vec![true; n],
			});
		}
		let j = Joints {
			knee: [V3::new(-0.1, 0.5, 0.0), V3::new(0.1, 0.5, 0.0)],
			ankle: [V3::new(-0.1, 0.08, 0.0), V3::new(0.1, 0.08, 0.0)],
			shoulder: [V3::new(-0.18, 1.4, 0.0), V3::new(0.18, 1.4, 0.0)],
			elbow: [V3::new(-0.35, 1.38, 0.0), V3::new(0.35, 1.38, 0.0)],
			wrist: [V3::new(-0.55, 1.4, 0.0), V3::new(0.55, 1.4, 0.0)],
		};
		let before = s.clone();
		free_motions(&mut s, &vec![j; n], FREE, 3);
		for k in [0, 3, 6, 7, 8] {
			assert!(
				s.devices[k].pose.iter().zip(&before.devices[k].pose).all(|(a, b)| a.p == b.p && a.q == b.q),
				"{k}"
			);
		}
		// the T-pose untouched; then the feet turn about their ankles (their distance to it kept), the hands keep
		// theirs to the wrist, the upper arms' trackers theirs to the shoulder
		for k in [1, 2, 4, 5, 9, 10] {
			assert!((0..120).all(|i| s.devices[k].pose[i].p == before.devices[k].pose[i].p), "{k}");
			assert!((120..n).any(|i| len(s.devices[k].pose[i].p - before.devices[k].pose[i].p) > 1e-3), "{k}");
		}
		for i in 0..n {
			for side in 0..2 {
				let d =
					|k: usize, o: V3| (len(s.devices[k].pose[i].p - o) - len(before.devices[k].pose[i].p - o)).abs();
				assert!(d(4 + side, j.ankle[side]) < 1e-9 && d(1 + side, j.wrist[side]) < 1e-9);
				assert!(d(9 + side, j.shoulder[side]) < 1e-9);
			}
		}
	}

	#[test]
	fn the_generator_is_seeded_and_symmetric() {
		let mut a = Rng::new(7);
		let mut b = Rng::new(7);
		let v: Vec<f64> = (0..1000).map(|_| a.sym()).collect();
		assert!(v.iter().zip((0..1000).map(|_| b.sym())).all(|(x, y)| *x == y));
		assert!(v.iter().all(|x| (-1.0..1.0).contains(x)));
		let m = v.iter().sum::<f64>() / v.len() as f64;
		assert!(m.abs() < 0.08, "{m}");
	}
}
