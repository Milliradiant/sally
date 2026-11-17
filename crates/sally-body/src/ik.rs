//! The yardstick (docs/mocap_plan.md section 6, step 2): an IK of VRChat's kind run on a truth recording and scored
//! against the truth trackers, per body part. What a virtual tracker has to beat.
//!
//! Scored where VRChat takes it: each truth tracker's own pose (chest, knees, elbows), against where the IK's bones
//! carry it. Each truth tracker is fixed to its bone as it sat in the T-pose (a knee's to the thigh, an elbow's to the
//! upper arm, the chest's to the spine), so a virtual tracker is scored the same way whatever it does inside.
//!
//! The body is calibrated generously, as an avatar that fits the body perfectly would be: the legs' hip and ankle
//! centres and the thigh and shin fitted from the motion of every session together, the truth trackers on (`fit`); the
//! arms scaled to the T-pose's reach. From then on the IK sees only what VRChat sees, the headset, the hands, the waist
//! and the feet:
//!
//! - legs: the hip centre rides the waist tracker, the ankle the foot tracker; the knee by two-bone IK, bent toward
//!   where the foot and the pelvis point (as VRChat bends a knee without a knee tracker);
//! - spine: from the pelvis (between the two hip centres) to a neck point fixed in the headset's frame; the chest's
//!   twist halfway between the pelvis's and the head's (VRIK's spine), its place along the spine and its own
//!   orientation in that frame taken from the T-pose;
//! - arms: the shoulders ride that chest, the wrist is the controller; the elbow by two-bone IK, bent back and down
//!   as the hand and the chest have it.
//!
//! The arms are not fitted from the motion: the shoulder girdle moves under the chest tracker (arms overhead, a
//! shrug), and a controller held loosely or hanging from its strap is not the hand, so the elbow chain broke the fit
//! (half the frames thrown out, a 0.6 m forearm).

use sally_math::math3d::{
	compose, cross, dot, inverse, len, norm, qangle, qconj, qfrom_mat, qmul, qrot, xform, Pose, Q4, V3,
};

use crate::fit::{fit_chains, Chain, Prior, Set};
use crate::poses::{Dark, Session};
use crate::roles::{Role, Roles};

pub const SIDES: [&str; 2] = ["left", "right"];

/// What the IK sees in a frame
#[derive(Clone, Copy, Debug, Default)]
pub struct Inputs {
	pub head: Pose,
	pub waist: Pose,
	pub hand: [Pose; 2],
	pub foot: [Pose; 2],
}

/// The truth trackers in a frame (or where the IK puts them)
#[derive(Clone, Copy, Debug, Default)]
pub struct Trackers {
	pub chest: Pose,
	pub knee: [Pose; 2],
	pub elbow: [Pose; 2],
}

/// The IK's bones: the chest (the chest tracker's pose), each thigh and upper arm (origin at the hip or shoulder, y
/// along the bone, z toward its bend)
#[derive(Clone, Copy, Debug, Default)]
pub struct Bones {
	pub chest: Pose,
	pub thigh: [Pose; 2],
	pub upper_arm: [Pose; 2],
}

/// A session's body
#[derive(Clone, Copy, Debug, Default)]
pub struct Calib {
	/// hip in the waist tracker's frame, knee in the knee tracker's (the fit's, not the IK's), ankle in the foot's
	pub leg: [Chain; 2],
	/// the shoulders in the chest's frame, the wrists in the controllers', the upper arm and forearm lengths
	pub shoulder: [V3; 2],
	pub wrist: [V3; 2],
	pub arm: [[f64; 2]; 2],
	/// the neck point in the headset's frame
	pub neck: V3,
	/// the body's forward in the waist tracker's, the headset's and each foot tracker's frame (from the T-pose)
	pub fwd_waist: V3,
	pub fwd_head: V3,
	pub fwd_foot: [V3; 2],
	/// the chest tracker in the spine's frame at the T-pose, and the spine's length then
	pub chest_rel: V3,
	pub chest_q: Q4,
	pub spine_len: f64,
	/// where an elbow bends to, in the controller's frame and in the chest's
	pub bend_hand: [V3; 2],
	pub bend_chest: [V3; 2],
	/// each knee's and elbow's truth tracker in its bone's frame, as in the T-pose
	pub knee_rel: [Pose; 2],
	pub elbow_rel: [Pose; 2],
	/// the stature the starting points were scaled by (m)
	pub stature: f64,
}

/// The priors' sd: on the joints' starting points, on the segments' lengths (m). Loose ones (0.15 / 0.04) let real
/// sessions run off (a 1.3 m shin: a joint far away along a direction the tracker hardly turned about fits the noise
/// as well); a joint's centre is within a few cm of where a T-pose of that stature puts it
const POINTS_SD: f64 = 0.05;
const LENGTHS_SD: f64 = 0.03;
/// The segments as shares of the stature (Winter's table, as avatars are scaled): thigh, shin; upper arm, forearm
const LEG: [f64; 2] = [0.245, 0.246];
const ARM: [f64; 2] = [0.186, 0.146];
/// The fit's frames: one every this many seconds, at most this many
const FIT_STEP: f64 = 0.1;
const FIT_MAX: usize = 6000;

/// A device's pose at `t` if it tracked there with light
fn pose_at(s: &Session, dark: &Dark, k: usize, t: f64) -> Option<Pose> {
	let d = &s.devices[k];
	if d.t.is_empty() || t < d.t[0] || t > d.t[d.t.len() - 1] {
		return None;
	}
	let i = d.at(t);
	(d.ok[i] && (d.t[i] - t).abs() < 0.01 && dark.lit(&d.serial, t)).then_some(d.pose[i])
}

fn local(p: &Pose, world: V3) -> V3 {
	qrot(qconj(p.q), world - p.p)
}

/// A rotation from its three axes (columns)
fn axes(x: V3, y: V3, z: V3) -> Q4 {
	qfrom_mat(&[[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]])
}

/// Any unit vector square to `d`
fn square_to(d: V3) -> V3 {
	norm(cross(d, if d.y.abs() < 0.9 { V3::new(0.0, 1.0, 0.0) } else { V3::new(1.0, 0.0, 0.0) }))
}

/// A bone's frame: at `from`, y toward `to`, z toward `bend` (square to y)
fn bone(from: V3, to: V3, bend: V3) -> Pose {
	let y = norm(to - from);
	let mut z = bend - y * dot(bend, y);
	z = if len(z) < 1e-6 { square_to(y) } else { norm(z) };
	Pose { p: from, q: axes(cross(y, z), y, z) }
}

/// The spine's frame: its origin (the pelvis), rotation (x right, y up the spine, z back) and length
fn spine(c: &Calib, waist: &Pose, head: &Pose) -> (V3, Q4, f64) {
	let pc = (xform(waist, c.leg[0].a) + xform(waist, c.leg[1].a)) * 0.5;
	let n = xform(head, c.neck);
	let u = norm(n - pc);
	let f0 = qrot(waist.q, c.fwd_waist) + qrot(head.q, c.fwd_head);
	let mut f = norm(f0 - u * dot(f0, u));
	if len(f) < 0.5 {
		let fw = qrot(waist.q, c.fwd_waist);
		f = norm(fw - u * dot(fw, u));
	}
	(pc, axes(cross(f, u), u, -f), len(n - pc))
}

/// The middle joint of a two-bone limb from `a` to `c` with segments `l1`, `l2`, out toward `bend`
pub fn two_bone(a: V3, c: V3, l1: f64, l2: f64, bend: V3) -> V3 {
	let d = c - a;
	let dn = norm(d);
	let dist = len(d).clamp((l1 - l2).abs() + 1e-6, l1 + l2 - 1e-6);
	let x = (l1 * l1 - l2 * l2 + dist * dist) / (2.0 * dist);
	let h = (l1 * l1 - x * x).max(0.0).sqrt();
	let mut perp = bend - dn * dot(bend, dn);
	perp = if len(perp) < 1e-6 { square_to(dn) } else { norm(perp) };
	a + dn * x + perp * h
}

/// The IK's bones for a frame
pub fn bones(c: &Calib, i: &Inputs) -> Bones {
	let (pc, qs, l) = spine(c, &i.waist, &i.head);
	let scale = if c.spine_len > 0.0 { l / c.spine_len } else { 1.0 };
	let chest = Pose { p: pc + qrot(qs, c.chest_rel * scale), q: qmul(qs, c.chest_q) };
	let mut b = Bones { chest, ..Default::default() };
	for s in 0..2 {
		let hip = xform(&i.waist, c.leg[s].a);
		let ankle = xform(&i.foot[s], c.leg[s].c);
		let bend = qrot(i.foot[s].q, c.fwd_foot[s]) + qrot(i.waist.q, c.fwd_waist);
		let knee = two_bone(hip, ankle, c.leg[s].l1, c.leg[s].l2, bend);
		b.thigh[s] = bone(hip, knee, bend);
		let shoulder = xform(&chest, c.shoulder[s]);
		let wrist = xform(&i.hand[s], c.wrist[s]);
		let bend = qrot(i.hand[s].q, c.bend_hand[s]) + qrot(chest.q, c.bend_chest[s]);
		let elbow = two_bone(shoulder, wrist, c.arm[s][0], c.arm[s][1], bend);
		b.upper_arm[s] = bone(shoulder, elbow, bend);
	}
	b
}

/// Where the IK puts the truth trackers
pub fn trackers(c: &Calib, b: &Bones) -> Trackers {
	let mut t = Trackers { chest: b.chest, ..Default::default() };
	for s in 0..2 {
		t.knee[s] = compose(&b.thigh[s], &c.knee_rel[s]);
		t.elbow[s] = compose(&b.upper_arm[s], &c.elbow_rel[s]);
	}
	t
}

/// The devices of the roles a calibration and a score need
struct Devices {
	head: usize,
	waist: usize,
	chest: usize,
	hand: [usize; 2],
	foot: [usize; 2],
	knee: [usize; 2],
	elbow: [usize; 2],
}

fn devices(r: &Roles) -> Result<Devices, String> {
	let need = |x: Role| r.device(x).ok_or_else(|| format!("no {} found in the T-pose", x.name()));
	Ok(Devices {
		head: need(Role::Head)?,
		waist: need(Role::Waist)?,
		chest: need(Role::Chest)?,
		hand: [need(Role::LHand)?, need(Role::RHand)?],
		foot: [need(Role::LFoot)?, need(Role::RFoot)?],
		knee: [need(Role::LKnee)?, need(Role::RKnee)?],
		elbow: [need(Role::LElbow)?, need(Role::RElbow)?],
	})
}

/// Frames of three devices together, one every FIT_STEP s from the T-pose on, where all three had light
fn chain_frames(s: &Session, dark: &Dark, t0: f64, k: [usize; 3]) -> [Vec<Pose>; 3] {
	let end = s.devices[k[0]].t.last().copied().unwrap_or(t0);
	let mut out = [Vec::new(), Vec::new(), Vec::new()];
	let mut t = t0;
	while t < end && out[0].len() < FIT_MAX {
		if let (Some(a), Some(b), Some(c)) =
			(pose_at(s, dark, k[0], t), pose_at(s, dark, k[1], t), pose_at(s, dark, k[2], t))
		{
			out[0].push(a);
			out[1].push(b);
			out[2].push(c);
		}
		t += FIT_STEP;
	}
	out
}

/// What one session gives the calibration: its devices' poses in the T-pose, the legs' starting points (in their
/// trackers' frames at the T-pose) and frames, where the shoulders are then, its stature
struct Prep {
	d: Devices,
	pose: Vec<Pose>,
	stature: f64,
	forward: V3,
	leg_init: [[V3; 3]; 2],
	leg_frames: [[Vec<Pose>; 3]; 2],
	shoulder: [V3; 2],
}

fn prep(s: &Session, r: &Roles, dark: &Dark) -> Result<Prep, String> {
	let d = devices(r)?;
	let t = r.tpose_t;
	let pose: Vec<Pose> =
		s.devices.iter().map(|dv| if dv.t.is_empty() { Pose::default() } else { dv.pose[dv.at(t)] }).collect();
	let (rt, up) = (r.right, V3::new(0.0, 1.0, 0.0));
	// the stature from the headset's height above the feet's trackers (on the feet, ~6 cm up), eyes at 93.6 %
	let stature = (r.eye + 0.06) / 0.936;
	let head = pose[d.head];
	let centre = V3::new(head.p.x, 0.0, head.p.z);
	let height = |share: f64| head.p.y - (0.936 - share) * stature;
	let mut p = Prep {
		pose: pose.clone(),
		stature,
		forward: r.forward,
		leg_init: [[V3::default(); 3]; 2],
		leg_frames: Default::default(),
		shoulder: [V3::default(); 2],
		d: devices(r)?,
	};
	for (side, sgn) in [(0, -1.0), (1, 1.0)] {
		let lat = |share: f64| rt * (sgn * share * stature);
		let hip = centre + lat(0.052) + up * height(0.53);
		let knee = centre + lat(0.052) + up * height(0.285);
		let ankle = centre + lat(0.052) + up * height(0.045);
		p.leg_init[side] =
			[local(&pose[d.waist], hip), local(&pose[d.knee[side]], knee), local(&pose[d.foot[side]], ankle)];
		p.leg_frames[side] = chain_frames(s, dark, t, [d.waist, d.knee[side], d.foot[side]]);
		p.shoulder[side] = centre + lat(0.129) + up * height(0.818);
	}
	Ok(p)
}

/// Calibrates the sessions' bodies together: the same person, so the same bones (docs in `fit`)
pub fn calibrate(list: &[(&Session, &Roles, &Dark)]) -> Result<Vec<Calib>, String> {
	let preps: Vec<Prep> = list.iter().map(|(s, r, d)| prep(s, r, d)).collect::<Result<_, _>>()?;
	if preps.is_empty() {
		return Ok(Vec::new());
	}
	let mut st: Vec<f64> = preps.iter().map(|p| p.stature).collect();
	st.sort_by(f64::total_cmp);
	let stature = st[st.len() / 2];
	let mut out: Vec<Calib> = preps.iter().map(|p| Calib { stature: p.stature, ..Default::default() }).collect();
	for side in 0..2 {
		let leg = Prior { points_sd: POINTS_SD, lengths: LEG.map(|x| x * stature), lengths_sd: LENGTHS_SD };
		let sets: Vec<Set> = preps
			.iter()
			.map(|p| Set {
				a: &p.leg_frames[side][0],
				b: &p.leg_frames[side][1],
				c: &p.leg_frames[side][2],
				points: p.leg_init[side],
			})
			.collect();
		for (c, ch) in out.iter_mut().zip(fit_chains(&sets, &leg)) {
			c.leg[side] = ch;
		}
	}
	for (c, p) in out.iter_mut().zip(&preps) {
		let (d, pose, fw, up) = (&p.d, &p.pose, p.forward, V3::new(0.0, 1.0, 0.0));
		c.neck = V3::new(0.0, -0.07, 0.09) * (p.stature / 1.66);
		c.fwd_waist = qrot(qconj(pose[d.waist].q), fw);
		c.fwd_head = qrot(qconj(pose[d.head].q), fw);
		let (pc, qs, l) = spine(c, &pose[d.waist], &pose[d.head]);
		let chest = pose[d.chest];
		c.chest_rel = qrot(qconj(qs), chest.p - pc);
		c.chest_q = qmul(qconj(qs), chest.q);
		c.spine_len = l;
		let bend = norm(fw * -0.7 + up * -0.7);
		for side in 0..2 {
			c.fwd_foot[side] = qrot(qconj(pose[d.foot[side]].q), fw);
			c.bend_hand[side] = qrot(qconj(pose[d.hand[side]].q), bend);
			c.bend_chest[side] = qrot(qconj(chest.q), bend);
			// the arm: from the shoulder to the controller as in the T-pose, split as an avatar's is
			c.shoulder[side] = local(&chest, p.shoulder[side]);
			c.wrist[side] = V3::default();
			let reach = len(pose[d.hand[side]].p - p.shoulder[side]);
			c.arm[side] = [reach * ARM[0] / (ARM[0] + ARM[1]), reach * ARM[1] / (ARM[0] + ARM[1])];
		}
		// the truth trackers on the bones as in the T-pose
		let inputs = Inputs {
			head: pose[d.head],
			waist: pose[d.waist],
			hand: d.hand.map(|k| pose[k]),
			foot: d.foot.map(|k| pose[k]),
		};
		let b = bones(c, &inputs);
		for side in 0..2 {
			c.knee_rel[side] = compose(&inverse(&b.thigh[side]), &pose[d.knee[side]]);
			c.elbow_rel[side] = compose(&inverse(&b.upper_arm[side]), &pose[d.elbow[side]]);
		}
	}
	Ok(out)
}

/// One body part's errors, sorted: position (m), orientation (rad)
#[derive(Clone, Debug, Default)]
pub struct Errors {
	pub pos: Vec<f64>,
	pub ang: Vec<f64>,
}

impl Errors {
	fn q(v: &[f64], f: f64) -> f64 {
		if v.is_empty() {
			return f64::NAN;
		}
		v[((v.len() - 1) as f64 * f).round() as usize]
	}
	pub fn median(&self) -> f64 {
		Self::q(&self.pos, 0.5)
	}
	pub fn p90(&self) -> f64 {
		Self::q(&self.pos, 0.9)
	}
	pub fn mean(&self) -> f64 {
		self.pos.iter().sum::<f64>() / self.pos.len().max(1) as f64
	}
	pub fn ang_median(&self) -> f64 {
		Self::q(&self.ang, 0.5)
	}
	pub fn ang_p90(&self) -> f64 {
		Self::q(&self.ang, 0.9)
	}
	fn sort(&mut self) {
		self.pos.sort_by(f64::total_cmp);
		self.ang.sort_by(f64::total_cmp);
	}
	pub fn extend(&mut self, o: &Errors) {
		self.pos.extend_from_slice(&o.pos);
		self.ang.extend_from_slice(&o.ang);
		self.sort();
	}
	fn push(&mut self, ik: &Pose, truth: &Pose) {
		self.pos.push(len(ik.p - truth.p));
		self.ang.push(qangle(qmul(qconj(ik.q), truth.q)));
	}
}

/// The parts scored, in order: chest, left knee, right knee, left elbow, right elbow
pub const PARTS: [&str; 5] = ["chest", "left knee", "right knee", "left elbow", "right elbow"];

/// A session scored
#[derive(Clone, Debug, Default)]
pub struct Score {
	pub parts: [Errors; 5],
	pub frames: usize,
}

/// Scores the IK on every frame at 50 Hz from 1 s after the T-pose, each part where its devices had light
pub fn score(s: &Session, r: &Roles, dark: &Dark, c: &Calib) -> Result<Score, String> {
	let d = devices(r)?;
	let mut sc = Score::default();
	let end = s.devices[d.head].t.last().copied().unwrap_or(0.0);
	let mut t = r.tpose_t + 1.0;
	while t < end {
		let p = |k: usize| pose_at(s, dark, k, t);
		sc.frames += 1;
		if let (Some(head), Some(waist)) = (p(d.head), p(d.waist)) {
			let (hand, foot, knee, elbow) = (d.hand.map(p), d.foot.map(p), d.knee.map(p), d.elbow.map(p));
			let inputs = Inputs {
				head,
				waist,
				hand: [hand[0].unwrap_or_default(), hand[1].unwrap_or_default()],
				foot: [foot[0].unwrap_or_default(), foot[1].unwrap_or_default()],
			};
			let ik = trackers(c, &bones(c, &inputs));
			if let Some(ch) = p(d.chest) {
				sc.parts[0].push(&ik.chest, &ch);
			}
			for side in 0..2 {
				if let (Some(_), Some(k)) = (foot[side], knee[side]) {
					sc.parts[1 + side].push(&ik.knee[side], &k);
				}
				if let (Some(_), Some(e)) = (hand[side], elbow[side]) {
					sc.parts[3 + side].push(&ik.elbow[side], &e);
				}
			}
		}
		t += 0.02;
	}
	for e in sc.parts.iter_mut() {
		e.sort();
	}
	Ok(sc)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn two_bone_bends_toward_the_goal() {
		let a = V3::new(0.0, 1.0, 0.0);
		let c = V3::new(0.0, 0.2, 0.0);
		let k = two_bone(a, c, 0.45, 0.45, V3::new(0.0, 0.0, -1.0));
		assert!((len(k - a) - 0.45).abs() < 1e-9 && (len(k - c) - 0.45).abs() < 1e-9);
		assert!(k.z < -0.2 && k.x.abs() < 1e-12, "{k:?}");
		// out of reach: straight, at the segments' share
		let k = two_bone(a, V3::new(0.0, -1.0, 0.0), 0.45, 0.45, V3::new(0.0, 0.0, -1.0));
		assert!((k.y - 0.55).abs() < 1e-4, "{k:?}");
	}

	#[test]
	fn a_bone_frame_is_a_rotation_along_the_bone() {
		let b = bone(V3::new(0.0, 1.0, 0.0), V3::new(0.0, 0.5, 0.1), V3::new(0.0, 0.0, -1.0));
		let y = qrot(b.q, V3::new(0.0, 1.0, 0.0));
		let z = qrot(b.q, V3::new(0.0, 0.0, 1.0));
		assert!(len(y - norm(V3::new(0.0, -0.5, 0.1))) < 1e-9, "{y:?}");
		assert!(z.z < 0.0 && dot(y, z).abs() < 1e-9, "{z:?}");
	}
}
