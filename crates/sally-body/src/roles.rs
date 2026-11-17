//! Which device is on which body part, from the T-pose at a session's start (docs/mocap_plan.md section 1): the way
//! VRChat assigns trackers, by where they are, never by serial or by kind (any tracker may be worn in any position).
//!
//! The T-pose is the first second in which everything is still, the hands are level, at about shoulder height and
//! spread wide. Its frame: up is +Y, the right is along the line from the left hand to the right (which hand is which
//! from where the headset looks), heights are measured from the lowest worn tracker and scaled by the headset's height
//! above it. A tracker over NOT_NEAR from the headset sideways in the T-pose is not worn (one left on a table) and gets
//! no role: a tracker that merely stays put cannot tell, feet planted through a seated session stay put too. The
//! headset's tracker is the one that sits still in the headset's frame all session; feet are at the bottom, knees below
//! the hips' height, the waist and the chest on the midline above (the lower one the waist), the elbows out to the
//! sides. Where a side has more than one candidate, the one nearest where that body part is in a T-pose is taken.

use sally_math::math3d::{cross, dot, len, norm, qconj, qrot, V3};

use crate::poses::Session;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
	Head,
	LHand,
	RHand,
	Waist,
	LFoot,
	RFoot,
	Chest,
	LKnee,
	RKnee,
	LElbow,
	RElbow,
	/// the tracker on the headset for Space Calibrator: not a body part
	HeadTracker,
}

impl Role {
	pub fn name(self) -> &'static str {
		match self {
			Role::Head => "head",
			Role::LHand => "left hand",
			Role::RHand => "right hand",
			Role::Waist => "waist",
			Role::LFoot => "left foot",
			Role::RFoot => "right foot",
			Role::Chest => "chest",
			Role::LKnee => "left knee",
			Role::RKnee => "right knee",
			Role::LElbow => "left elbow",
			Role::RElbow => "right elbow",
			Role::HeadTracker => "headset tracker",
		}
	}
}

/// The T-pose found and what it gave
#[derive(Debug, Default)]
pub struct Roles {
	/// the T-pose's middle, in the log's time
	pub tpose_t: f64,
	/// the headset's height above the lowest tracker, m
	pub eye: f64,
	/// the hands' distance, m
	pub span: f64,
	/// each device's role, by index into the session's devices
	pub role: Vec<Option<Role>>,
	/// the trackers away from the body in the T-pose (not worn), by index
	pub not_worn: Vec<usize>,
	/// what did not fit
	pub notes: Vec<String>,
	/// a T-pose was found that can be gone by: it held, the hands' line is across where the headset looks, and
	/// every tracker's part could be told. The notes it may still have say which trackers got no role (not worn,
	/// left out, not tracking then) or how the midline's were taken: none of them makes the T-pose a poor one
	pub tpose_ok: bool,
	/// the T-pose's frame: the headset's position, the body's right and forward (horizontal, unit), the floor's height
	/// (the lowest worn tracker)
	pub head_at: V3,
	pub right: V3,
	pub forward: V3,
	pub floor: f64,
}

impl Roles {
	pub fn device(&self, r: Role) -> Option<usize> {
		self.role.iter().position(|&x| x == Some(r))
	}
}

/// Grid step of the T-pose search, s
const STEP: f64 = 0.05;
/// How far into the session the T-pose is looked for, s
const SEARCH: f64 = 120.0;
/// How long it has to hold, s
const HOLD: f64 = 1.0;
/// Still: each device within this of where it was 0.25 s before and after, m
const STILL: f64 = 0.04;
/// Midline: within this of the headset sideways, m
const MIDLINE: f64 = 0.2;
/// Not worn: a tracker further than this from the headset sideways in the T-pose (the hands are about 0.8 m out), m
const NOT_NEAR: f64 = 0.9;
/// The headset's tracker: within this of the headset in the T-pose (m) and this still in the headset's frame (sd, m).
/// One on a Quest sits 10-15 cm from the headset's point; a chest tracker high on the sternum sits about 30 cm away,
/// and over a short or still stretch it moves little in the headset's frame
const HEAD_NEAR: f64 = 0.2;
const HEAD_RIGID: f64 = 0.02;
/// Where a body part sits sideways in a T-pose, as a share of the headset's height: feet and knees, elbows
const SIDE_LEG: f64 = 0.08;
const SIDE_ELBOW: f64 = 0.25;

fn horiz(v: V3) -> V3 {
	V3::new(v.x, 0.0, v.z)
}

/// The position at `t` if the device tracks there
fn pos_at(s: &Session, k: usize, t: f64) -> Option<V3> {
	let d = &s.devices[k];
	if d.t.is_empty() || t < d.t[0] || t > d.t[d.t.len() - 1] {
		return None;
	}
	let i = d.at(t);
	d.ok[i].then_some(d.pose[i].p)
}

fn still(s: &Session, k: usize, t: f64) -> bool {
	match (pos_at(s, k, t - 0.25), pos_at(s, k, t), pos_at(s, k, t + 0.25)) {
		(Some(a), Some(b), Some(c)) => len(a - b) < STILL && len(c - b) < STILL,
		_ => false,
	}
}

/// The device of `class` with the most tracking rows, and the others of that class by the same count
fn ranked(s: &Session, class: &str) -> Vec<usize> {
	let mut v: Vec<(usize, usize)> = s.by_class(class).map(|(k, d)| (k, d.ok.iter().filter(|&&o| o).count())).collect();
	v.sort_by_key(|x| std::cmp::Reverse(x.1));
	v.into_iter().map(|(k, _)| k).collect()
}

/// Whether the T-pose holds at `t`: everything that tracks is still, the hands level, at shoulder height and wide
fn tpose_at(s: &Session, head: usize, hands: [usize; 2], trackers: &[usize], t: f64) -> Option<(f64, f64)> {
	let h = pos_at(s, head, t)?;
	let a = pos_at(s, hands[0], t)?;
	let b = pos_at(s, hands[1], t)?;
	let floor = trackers.iter().filter_map(|&k| pos_at(s, k, t)).map(|p| p.y).fold(f64::INFINITY, f64::min);
	let floor = if floor.is_finite() { floor } else { 0.0 };
	let eye = h.y - floor;
	if eye < 0.8 {
		return None;
	}
	let span = len(a - b);
	let (ha, hb) = ((a.y - floor) / eye, (b.y - floor) / eye);
	let level = (a.y - b.y).abs() < 0.15;
	let shoulder = (0.65..1.05).contains(&ha) && (0.65..1.05).contains(&hb);
	if span < 0.75 * eye || !level || !shoulder {
		return None;
	}
	let all_still =
		[head, hands[0], hands[1]].iter().chain(trackers).all(|&k| pos_at(s, k, t).is_none() || still(s, k, t));
	all_still.then_some((eye, span))
}

/// How still device `a` sits in the headset's frame over the session: the largest of the three axes' sd of its
/// position there, on every 25th row both track (m), and how many rows
fn head_frame_sd(s: &Session, a: usize, head: usize) -> (f64, usize) {
	let (da, dh) = (&s.devices[a], &s.devices[head]);
	let mut v: Vec<V3> = Vec::new();
	for i in (0..da.t.len()).step_by(25) {
		if !da.ok[i] {
			continue;
		}
		let j = dh.at(da.t[i]);
		if dh.ok[j] && (dh.t[j] - da.t[i]).abs() < 0.01 {
			v.push(qrot(qconj(dh.pose[j].q), da.pose[i].p - dh.pose[j].p));
		}
	}
	if v.len() < 2 {
		return (f64::INFINITY, v.len());
	}
	let n = v.len() as f64;
	let sd = (0..3)
		.map(|ax| {
			let m = v.iter().map(|p| p.get(ax)).sum::<f64>() / n;
			(v.iter().map(|p| (p.get(ax) - m).powi(2)).sum::<f64>() / (n - 1.0)).sqrt()
		})
		.fold(0.0, f64::max);
	(sd, v.len())
}

/// One body part on each side from the candidates (sideways offset, device): per side the one nearest `expect`
/// sideways; one side's two split by order when the other side has none. The ones left out, by index
fn pick_pair(v: &mut [(f64, usize)], expect: f64) -> (Option<usize>, Option<usize>, Vec<usize>) {
	v.sort_by(|a, b| a.0.total_cmp(&b.0));
	let nearest = |side: &[(f64, usize)]| {
		side.iter().min_by(|a, b| (a.0.abs() - expect).abs().total_cmp(&(b.0.abs() - expect).abs())).map(|x| x.1)
	};
	let split = v.partition_point(|x| x.0 < 0.0);
	let (left, right) = v.split_at(split);
	let (mut l, mut r) = (nearest(left), nearest(right));
	if v.len() >= 2 && (l.is_none() || r.is_none()) {
		(l, r) = (Some(v[0].1), Some(v[v.len() - 1].1));
	}
	let out = v.iter().map(|x| x.1).filter(|&k| Some(k) != l && Some(k) != r).collect();
	(l, r, out)
}

pub fn assign(s: &Session) -> Roles {
	let mut r = Roles { role: vec![None; s.devices.len()], ..Default::default() };
	let hmds = ranked(s, "hmd");
	let controllers = ranked(s, "controller");
	let trackers = ranked(s, "tracker");
	let Some(&head) = hmds.first() else {
		r.notes.push("no headset in the log".into());
		return r;
	};
	if controllers.len() < 2 {
		r.notes.push(format!("{} controllers in the log, the T-pose needs two", controllers.len()));
		return r;
	}
	let hands = [controllers[0], controllers[1]];
	r.role[head] = Some(Role::Head);

	// the first stretch of HOLD seconds in T-pose, else the longest of at least 0.3 s
	let t0 = s.devices[head].t.first().copied().unwrap_or(0.0);
	let end = s.devices[head].t.last().copied().unwrap_or(0.0).min(t0 + SEARCH);
	let mut run: Vec<(f64, f64, f64)> = Vec::new();
	let mut best: Vec<(f64, f64, f64)> = Vec::new();
	let mut t = t0 + 0.25;
	while t < end - 0.25 {
		match tpose_at(s, head, hands, &trackers, t) {
			Some((eye, span)) => {
				run.push((t, eye, span));
				if run.len() as f64 * STEP >= HOLD {
					break;
				}
			}
			None => {
				if run.len() > best.len() {
					best = std::mem::take(&mut run);
				}
				run.clear();
			}
		}
		t += STEP;
	}
	if run.len() > best.len() {
		best = run;
	}
	if (best.len() as f64) * STEP < 0.3 {
		r.notes.push(format!("no T-pose in the first {SEARCH:.0} s (still, hands level at shoulder height and wide)"));
		return r;
	}
	r.tpose_ok = true;
	if (best.len() as f64) * STEP < HOLD {
		r.notes.push(format!("the T-pose held only {:.2} s", best.len() as f64 * STEP));
		r.tpose_ok = false;
	}
	let (tt, eye, span) = best[best.len() / 2];
	(r.tpose_t, r.eye, r.span) = (tt, eye, span);

	// the T-pose's frame
	let h = pos_at(s, head, tt).unwrap_or_default();
	let look = {
		let d = &s.devices[head];
		norm(horiz(qrot(d.pose[d.at(tt)].q, V3::new(0.0, 0.0, -1.0))))
	};
	let up = V3::new(0.0, 1.0, 0.0);
	let (a, b) = (pos_at(s, hands[0], tt).unwrap_or_default(), pos_at(s, hands[1], tt).unwrap_or_default());
	let mut right = norm(horiz(b - a));
	// the right is where the headset's right is (cross(look, up))
	if dot(right, cross(look, up)) < 0.0 {
		right = -right;
	}
	let side = |p: V3| dot(p - h, right);
	(r.head_at, r.right, r.forward) = (h, right, cross(up, right));
	let (l, rr) = if side(a) < side(b) { (hands[0], hands[1]) } else { (hands[1], hands[0]) };
	r.role[l] = Some(Role::LHand);
	r.role[rr] = Some(Role::RHand);
	if dot(norm(horiz(b - a)), look).abs() > 0.5 {
		r.notes.push("the hands' line is not across where the headset looks".into());
		r.tpose_ok = false;
	}

	// trackers away from the body (left on a table), then the floor from the worn ones
	let mut trackers = trackers;
	for &k in &trackers {
		if pos_at(s, k, tt).is_some_and(|p| len(horiz(p - h)) > NOT_NEAR) {
			r.not_worn.push(k);
			r.notes.push(format!("{} away from the body in the T-pose: not worn", s.devices[k].serial));
		}
	}
	trackers.retain(|k| !r.not_worn.contains(k));
	let floor = trackers.iter().filter_map(|&k| pos_at(s, k, tt)).map(|p| p.y).fold(f64::INFINITY, f64::min);
	r.floor = floor;
	let rel = |p: V3| (p.y - floor) / eye;
	let mut feet = Vec::new();
	let mut knees = Vec::new();
	let mut mid = Vec::new();
	let mut elbows = Vec::new();
	for &k in &trackers {
		let Some(p) = pos_at(s, k, tt) else {
			r.notes.push(format!("{} not tracking at the T-pose: no role", s.devices[k].serial));
			continue;
		};
		if len(p - h) < HEAD_NEAR {
			let (sd, n) = head_frame_sd(s, k, head);
			if n > 10 && sd < HEAD_RIGID {
				r.role[k] = Some(Role::HeadTracker);
				continue;
			}
			if n <= 10 {
				// too little of the session yet to tell whether it sits still in the headset's frame (live, just after
				// the T-pose was asked for again): no role, and no T-pose to go by until there is. Taken by its place
				// it is on the midline above everything, and was the chest
				r.notes.push(format!(
					"{} by the headset in the T-pose: too little of the session yet to tell whether it is the headset's tracker",
					s.devices[k].serial
				));
				r.tpose_ok = false;
				continue;
			}
		}
		let (y, x) = (rel(p), side(p));
		if y < 0.15 {
			feet.push((x, k));
		} else if y < 0.45 {
			knees.push((x, k));
		} else if x.abs() < MIDLINE {
			mid.push((y, k));
		} else {
			elbows.push((x, k));
		}
	}
	for (v, left, right, expect, what) in [
		(&mut feet, Role::LFoot, Role::RFoot, SIDE_LEG, "feet"),
		(&mut knees, Role::LKnee, Role::RKnee, SIDE_LEG, "knees"),
		(&mut elbows, Role::LElbow, Role::RElbow, SIDE_ELBOW, "elbows"),
	] {
		let n = v.len();
		let (l, rr, out) = pick_pair(v, expect * eye);
		if n == 1 {
			// one alone: its side says which
			let (x, k) = v[0];
			r.role[k] = Some(if x < 0.0 { left } else { right });
			continue;
		}
		if let Some(k) = l {
			r.role[k] = Some(left);
		}
		if let Some(k) = rr {
			r.role[k] = Some(right);
		}
		if !out.is_empty() {
			let names: Vec<&str> = out.iter().map(|&k| s.devices[k].serial.as_str()).collect();
			r.notes.push(format!("{n} trackers where the {what} are: {} left out", names.join(", ")));
		}
	}
	mid.sort_by(|a, b| a.0.total_cmp(&b.0));
	match mid.len() {
		0 => {}
		1 => r.role[mid[0].1] = Some(if mid[0].0 > 0.72 { Role::Chest } else { Role::Waist }),
		n => {
			if n > 2 {
				r.notes.push(format!("{n} trackers on the midline: the lowest is the waist, the highest the chest"));
			}
			r.role[mid[0].1] = Some(Role::Waist);
			r.role[mid[n - 1].1] = Some(Role::Chest);
		}
	}
	r
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::poses::Device;
	use sally_math::math3d::{qmul, Pose, Q4};

	const N: usize = 3000;

	fn times() -> Vec<f64> {
		(0..N).map(|i| i as f64 * 0.004).collect()
	}

	/// After the T-pose (6 s at 250 Hz) the body walks 3 m along X
	fn walk(t: f64) -> V3 {
		V3::new(if t > 6.0 { (t - 6.0) * 0.5 } else { 0.0 }, 0.0, 0.0)
	}

	/// The head's yaw then: it looks about while walking
	fn yaw(t: f64) -> Q4 {
		let a = if t > 6.0 { 0.6 * (2.0 * (t - 6.0)).sin() } else { 0.0 };
		Q4::new((a / 2.0).cos(), 0.0, (a / 2.0).sin(), 0.0)
	}

	/// A device on the body: still at `p` in the T-pose, then walking with it
	fn device(serial: &str, class: &str, p: V3, q: Q4) -> Device {
		let t = times();
		let pose = t.iter().map(|&tt| Pose { p: p + walk(tt), q }).collect();
		Device { serial: serial.into(), class: class.into(), t, pose, ok: vec![true; N] }
	}

	/// A user in a T-pose looking along -Z (right is +X), the headset 1.50 m up, every tracker the plan's truth rig
	/// has and the headset's tracker
	fn rig(look_turned: bool) -> Session {
		let q = if look_turned { Q4::new(0.0, 0.0, 1.0, 0.0) } else { Q4::default() };
		let sx = if look_turned { -1.0 } else { 1.0 };
		let at = |x: f64, y: f64, z: f64| V3::new(sx * x, y, sx * z);
		let mut head = device("H", "hmd", at(0.0, 1.50, 0.0), q);
		for (i, p) in head.pose.iter_mut().enumerate() {
			p.q = qmul(q, yaw(head.t[i]));
		}
		// the headset's tracker sits on the headset: still in its frame
		let local = qrot(qconj(q), at(0.0, 0.12, 0.05));
		let mut ht = device("HT", "tracker", V3::default(), q);
		for (i, p) in ht.pose.iter_mut().enumerate() {
			*p = Pose { p: head.pose[i].p + qrot(head.pose[i].q, local), q: head.pose[i].q };
		}
		let mut d = vec![
			head,
			device("C1", "controller", at(0.72, 1.33, 0.02), q),
			device("C2", "controller", at(-0.72, 1.33, 0.02), q),
			device("FL", "tracker", at(-0.10, 0.07, -0.08), q),
			device("FR", "tracker", at(0.10, 0.07, -0.08), q),
			device("W", "tracker", at(0.0, 0.95, -0.12), q),
			device("CH", "tracker", at(0.0, 1.22, -0.12), q),
			device("KL", "tracker", at(-0.10, 0.48, -0.08), q),
			device("KR", "tracker", at(0.10, 0.48, -0.08), q),
			device("EL", "tracker", at(-0.42, 1.30, 0.03), q),
			device("ER", "tracker", at(0.42, 1.30, 0.03), q),
		];
		d.push(ht);
		Session { start_epoch: 0.0, devices: d }
	}

	/// A tracker left on a table at `p`
	fn lying(serial: &str, p: V3) -> Device {
		let t = times();
		Device {
			serial: serial.into(),
			class: "tracker".into(),
			pose: vec![Pose { p, q: Q4::default() }; N],
			t,
			ok: vec![true; N],
		}
	}

	fn roles_by_serial(s: &Session, r: &Roles) -> Vec<(String, Option<Role>)> {
		s.devices.iter().zip(&r.role).map(|(d, &x)| (d.serial.clone(), x)).collect()
	}

	/// A session as the live path has it a moment after its history was cleared: every third row (83 a second, a
	/// headset's frames), the first `rows` of them
	fn live_start(mut s: Session, rows: usize) -> Session {
		fn thin<T: Clone>(v: &[T], rows: usize) -> Vec<T> {
			v.iter().step_by(3).take(rows).cloned().collect()
		}
		for d in &mut s.devices {
			(d.t, d.pose, d.ok) = (thin(&d.t, rows), thin(&d.pose, rows), thin(&d.ok, rows));
		}
		s
	}

	#[test]
	fn the_headsets_tracker_is_not_the_chest_while_the_session_is_too_short_to_tell() {
		// 2 s: a T-pose is found, but the tracker by the headset cannot be told yet
		let mut s = live_start(rig(false), 170);
		s.devices.retain(|d| d.serial != "CH");
		let r = assign(&s);
		let ht = s.devices.iter().position(|d| d.serial == "HT").unwrap();
		assert_eq!(r.role[ht], None, "{:?}", r.notes);
		assert_eq!(r.device(Role::Chest), None);
		assert!(!r.tpose_ok && r.notes.iter().any(|n| n.contains("HT by the headset")), "{:?}", r.notes);
		// 4 s: it is the headset's
		let mut s = live_start(rig(false), 340);
		s.devices.retain(|d| d.serial != "CH");
		let r = assign(&s);
		assert_eq!(r.role[ht], Some(Role::HeadTracker));
		assert!(r.tpose_ok && r.notes.is_empty(), "{:?}", r.notes);
	}

	#[test]
	fn a_tracker_without_a_role_leaves_the_t_pose_good() {
		// one lying on a table, and one that is on but not tracking
		let mut s = rig(false);
		s.devices.push(lying("TABLE", V3::new(2.0, 0.8, 1.0)));
		let mut dark = lying("DARK", V3::new(0.0, 1.0, 0.0));
		dark.ok = vec![false; N];
		s.devices.push(dark);
		let r = assign(&s);
		assert!(r.tpose_ok, "{:?}", r.notes);
		assert!(r.notes.iter().any(|n| n.contains("TABLE") && n.contains("not worn")), "{:?}", r.notes);
		assert!(r.notes.iter().any(|n| n.contains("DARK not tracking")), "{:?}", r.notes);
		assert_eq!(r.device(Role::Waist).map(|k| s.devices[k].serial.as_str()), Some("W"));
	}

	#[test]
	fn the_truth_rig_is_assigned() {
		for turned in [false, true] {
			let s = rig(turned);
			let r = assign(&s);
			assert!(r.notes.is_empty(), "{:?}", r.notes);
			let want = [
				("H", Role::Head),
				("C1", Role::RHand),
				("C2", Role::LHand),
				("FL", Role::LFoot),
				("FR", Role::RFoot),
				("W", Role::Waist),
				("CH", Role::Chest),
				("KL", Role::LKnee),
				("KR", Role::RKnee),
				("EL", Role::LElbow),
				("ER", Role::RElbow),
				("HT", Role::HeadTracker),
			];
			let got = roles_by_serial(&s, &r);
			for (serial, role) in want {
				assert!(got.contains(&(serial.to_string(), Some(role))), "{serial} turned={turned}: {got:?}");
			}
			assert!((r.eye - 1.43).abs() < 1e-9, "{}", r.eye);
			assert!(r.tpose_t < 1.5, "{}", r.tpose_t);
		}
	}

	#[test]
	fn play_rig_waist_only() {
		let mut s = rig(false);
		s.devices.retain(|d| ["H", "C1", "C2", "FL", "FR", "W"].contains(&d.serial.as_str()));
		let r = assign(&s);
		assert!(r.notes.is_empty(), "{:?}", r.notes);
		assert_eq!(r.role[s.devices.iter().position(|d| d.serial == "W").unwrap()], Some(Role::Waist));
	}

	#[test]
	fn no_tpose_is_said() {
		let mut s = rig(false);
		// the hands down at the sides
		for d in s.devices.iter_mut().filter(|d| d.class == "controller") {
			for p in d.pose.iter_mut() {
				p.p.y = 0.8;
			}
		}
		let r = assign(&s);
		assert!(r.notes.iter().any(|n| n.contains("no T-pose")), "{:?}", r.notes);
	}

	#[test]
	fn a_tracker_left_on_a_table_is_not_worn() {
		// to the right at elbow height
		let mut s = rig(false);
		s.devices.push(lying("T", V3::new(1.4, 0.84, -0.9)));
		let r = assign(&s);
		let k = s.devices.iter().position(|d| d.serial == "T").unwrap();
		assert_eq!(r.role[k], None);
		assert_eq!(r.not_worn, vec![k]);
		let er = s.devices.iter().position(|d| d.serial == "ER").unwrap();
		assert_eq!(r.role[er], Some(Role::RElbow));
	}

	#[test]
	fn a_third_tracker_at_an_elbow_is_left_out() {
		let mut s = rig(false);
		s.devices.push(device("E3", "tracker", V3::new(0.68, 1.31, 0.02), Q4::default()));
		let r = assign(&s);
		let er = s.devices.iter().position(|d| d.serial == "ER").unwrap();
		assert_eq!(r.role[er], Some(Role::RElbow));
		assert!(r.notes.iter().any(|n| n.contains("E3 left out")), "{:?}", r.notes);
	}

	#[test]
	fn planted_feet_through_a_seated_session_are_worn() {
		// the feet and knees stay where they were in the T-pose while the rest moves (a seated session)
		let mut s = rig(false);
		for d in s.devices.iter_mut().filter(|d| ["FL", "FR", "KL", "KR"].contains(&d.serial.as_str())) {
			let p0 = d.pose[0];
			d.pose.iter_mut().for_each(|p| *p = p0);
		}
		let r = assign(&s);
		assert!(r.not_worn.is_empty() && r.notes.is_empty(), "{:?}", r.notes);
		let got = roles_by_serial(&s, &r);
		for (serial, role) in [("FL", Role::LFoot), ("FR", Role::RFoot), ("KL", Role::LKnee), ("KR", Role::RKnee)] {
			assert!(got.contains(&(serial.to_string(), Some(role))), "{serial}: {got:?}");
		}
	}
}
