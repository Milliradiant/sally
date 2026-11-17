//! The probes (docs/mocap_plan.md step 6): where an IK of VRChat's kind gives itself away, and whether a model does the
//! same.
//!
//! - **The foot turned about the ankle, the knee still**: an IK bends the knee toward where the foot points, so the
//!   knee follows the foot up and down.
//! - **The hand twisted about the forearm, the elbow still**: an IK bends the elbow as the hand is turned.
//! - **The elbow swung about the line from the shoulder to the wrist, the hand in place**: an IK does not follow the
//!   real elbow. With the hand turning with the forearm, and with the hand's turn kept.
//!
//! On mocap (a BVH, the rig put on it as in training, `--mounts`), each is done to the rig's own joints: the foot
//! tracker turned about the ankle joint (about the axis square to the shin and the foot), the controller about the
//! forearm's axis through the wrist, the upper arm's tracker (and the controller, the first way) about the
//! shoulder-wrist line through the shoulder, by a 1 Hz swing (25, 45 and 30 deg), left and right at once; the truth
//! moves only in the swing. Each virtual tracker and the IK's are compared, frame by frame, with themselves on the same
//! frames unmoved; counted where the swing is past 70 % of its reach. On truth recordings (`sally-poses` CSVs), the
//! same three looked for in the recorded motion: half-second stretches where the foot turned over 20 deg while the real
//! knee tracker moved under 2 cm and 5 deg and the waist under 3 cm; where a controller turned over 30 deg and moved
//! under 5 cm while the real elbow tracker moved under 2 cm and 5 deg; where the real elbow tracker moved over 5 cm
//! while the controller moved under 3 cm. Each file is put through the model named before it (the folds of a tuning, as
//! `eval`). The IK is the yardstick's (`sally_body::ik`, calibrated as `body score` does: the truth recordings
//! together, a mocap session on its own).

use std::path::{Path, PathBuf};

use burn::prelude::*;
use sally_body::features::{self, Calib};
use sally_body::poses::{self, Dark, Session};
use sally_body::roles::{Role, Roles};
use sally_body::{bvh, ik, roles, synth};
use sally_math::math3d::{cross, dot, len, norm, qangle, qconj, qmul, qrot, Pose, Q4, V3};

use crate::{load_net, normalize, read_norm, tensor, tracking, Net, Norm, EYE};

/// The swings: how far (rad) and how fast (Hz); counted past this share of the reach
const FOOT_DEG: f64 = 25.0;
const TWIST_DEG: f64 = 45.0;
const SWING_DEG: f64 = 30.0;
const HZ: f64 = 1.0;
const PAST: f64 = 0.7;
/// The foot in the air: the ankle this far above where it was in the T-pose, m
const AIR: f64 = 0.10;
/// The truth recordings' stretches, s
const WINDOW: f64 = 0.5;

#[derive(Default)]
struct Acc(Vec<f64>);

impl Acc {
	fn push(&mut self, v: f64) {
		if v.is_finite() {
			self.0.push(v);
		}
	}
	fn q(&mut self, f: f64) -> f64 {
		self.0.sort_by(f64::total_cmp);
		if self.0.is_empty() {
			f64::NAN
		} else {
			self.0[((self.0.len() - 1) as f64 * f).round() as usize]
		}
	}
	/// median / p90, scaled
	fn mp(&mut self, scale: f64) -> String {
		format!("{:.0} / {:.0}", self.q(0.5) * scale, self.q(0.9) * scale)
	}
	fn n(&self) -> usize {
		self.0.len()
	}
}

/// What one probe found for the model and the IK: how far the virtual tracker moved (m) and turned (rad) against
/// itself unmoved; for the swings also the truth's move, the share of it followed, the error before and after
#[derive(Default)]
struct Found {
	moved: [Acc; 2],
	turned: [Acc; 2],
	truth: Acc,
	follows: [Acc; 2],
	err0: [Acc; 2],
	err1: [Acc; 2],
}

const WHO: [&str; 2] = ["model", "IK"];

/// One frame of a probe: the model's (who 0) or the IK's (1) virtual tracker unmoved (`a`) and moved (`z`), the truth
/// unmoved and moved
fn record(fo: &mut Found, who: usize, a: &Pose, z: &Pose, t0: &Pose, t1: &Pose) {
	let dt = t1.p - t0.p;
	if who == 0 {
		fo.truth.push(len(dt));
	}
	fo.moved[who].push(len(z.p - a.p));
	fo.turned[who].push(qangle(qmul(qconj(a.q), z.q)));
	if len(dt) > 0.02 {
		fo.follows[who].push(dot(z.p - a.p, dt) / dot(dt, dt));
	}
	fo.err0[who].push(len(a.p - t0.p));
	fo.err1[who].push(len(z.p - t1.p));
}

fn rot_about(p: &Pose, o: V3, q: Q4) -> Pose {
	Pose { p: o + qrot(q, p.p - o), q: qmul(q, p.q) }
}

fn axis_angle(ax: V3, ang: f64) -> Q4 {
	let (s, c) = ((ang / 2.0).sin(), (ang / 2.0).cos());
	Q4::new(c, ax.x * s, ax.y * s, ax.z * s)
}

/// The model's virtual trackers at each time, in its targets' order (None where it has no inputs)
fn predict(model: &Net, norm: &Norm, s: &Session, c: &Calib, times: &[f64]) -> Result<Vec<Option<Vec<Pose>>>, String> {
	let device = Device::default();
	let (n_in, n_out) = (c.layout.n_in(), c.layout.n_out());
	let (mut xs, mut frames, mut idx) = (Vec::new(), Vec::new(), Vec::new());
	for (i, &t) in times.iter().enumerate() {
		if let Some((mut x, frame)) = features::inputs(s, c, t) {
			normalize(&mut x, n_in, &norm.xm, &norm.xs);
			xs.extend(x);
			frames.push(frame);
			idx.push(i);
		}
	}
	let mut out = vec![None; times.len()];
	let n = frames.len();
	for start in (0..n).step_by(4096) {
		let k = (n - start).min(4096);
		let y = model.forward(tensor(&xs[start * n_in..(start + k) * n_in], k, n_in, &device));
		let v: Vec<f32> = y.into_data().try_to_vec::<f32>().map_err(|e| format!("{e:?}"))?;
		for r in 0..k {
			let o: Vec<f32> =
				v[r * n_out..(r + 1) * n_out].iter().enumerate().map(|(j, x)| x * norm.ys[j] + norm.ym[j]).collect();
			out[idx[start + r]] = Some(features::decode(c, &frames[start + r], &o));
		}
	}
	Ok(out)
}

fn pose(s: &Session, k: usize, t: f64) -> Pose {
	let d = &s.devices[k];
	d.pose[d.at(t + 1e-6)]
}

fn lit(s: &Session, k: usize, t: f64) -> bool {
	let d = &s.devices[k];
	!d.t.is_empty() && t >= d.t[0] - 1e-6 && t <= d.t[d.t.len() - 1] + 1e-6 && d.ok[d.at(t + 1e-6)]
}

/// The devices a probe needs, by role (the session's T-pose found them all)
struct Devs {
	head: usize,
	waist: usize,
	hand: [usize; 2],
	foot: [usize; 2],
	knee: [usize; 2],
	elbow: [usize; 2],
}

fn devs(r: &Roles) -> Result<Devs, String> {
	let d = |x: Role| r.device(x).ok_or_else(|| format!("no {} found in the T-pose", x.name()));
	Ok(Devs {
		head: d(Role::Head)?,
		waist: d(Role::Waist)?,
		hand: [d(Role::LHand)?, d(Role::RHand)?],
		foot: [d(Role::LFoot)?, d(Role::RFoot)?],
		knee: [d(Role::LKnee)?, d(Role::RKnee)?],
		elbow: [d(Role::LElbow)?, d(Role::RElbow)?],
	})
}

/// The IK's trackers at `t`
fn ik_at(k: &ik::Calib, s: &Session, d: &Devs, t: f64) -> ik::Trackers {
	let p = |j: usize| pose(s, j, t);
	let i = ik::Inputs { head: p(d.head), waist: p(d.waist), hand: d.hand.map(p), foot: d.foot.map(p) };
	ik::trackers(k, &ik::bones(k, &i))
}

/// A knee's (`elbow` false) or an elbow's virtual tracker from the IK's
fn ik_side(t: &ik::Trackers, elbow: bool, side: usize) -> Pose {
	if elbow {
		t.elbow[side]
	} else {
		t.knee[side]
	}
}

/// Where a knee's or an elbow's virtual tracker is among the model's targets
fn target(l: &features::Layout, elbow: bool, side: usize) -> Option<usize> {
	l.target(match (elbow, side) {
		(false, 0) => Role::LKnee,
		(false, _) => Role::RKnee,
		(true, 0) => Role::LElbow,
		(true, _) => Role::RElbow,
	})
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
	Foot,
	Twist,
	SwingWithHand,
	SwingHandStill,
}

const KINDS: [Kind; 4] = [Kind::Foot, Kind::Twist, Kind::SwingWithHand, Kind::SwingHandStill];

fn kind_name(k: Kind) -> String {
	match k {
		Kind::Foot => format!("foot turned {FOOT_DEG:.0} deg about the ankle, the knee still: the knee moves"),
		Kind::Twist => format!("hand twisted {TWIST_DEG:.0} deg about the forearm, the elbow still: the elbow moves"),
		Kind::SwingWithHand => {
			format!("elbow swung {SWING_DEG:.0} deg about shoulder-wrist, the hand turning with the forearm")
		}
		Kind::SwingHandStill => format!("elbow swung {SWING_DEG:.0} deg about shoulder-wrist, the hand's turn kept"),
	}
}

/// The session with one probe's swing on both sides from `t0` on
fn perturb(s: &Session, joints: &[synth::Joints], d: &Devs, kind: Kind, t0: f64) -> Session {
	let mut out = s.clone();
	for side in 0..2 {
		let (foot, hand, elbow) = (d.foot[side], d.hand[side], d.elbow[side]);
		for (i, j) in joints.iter().enumerate() {
			let t = s.devices[foot].t[i];
			if t < t0 {
				continue;
			}
			let w = (2.0 * std::f64::consts::PI * HZ * (t - t0)).sin();
			match kind {
				Kind::Foot => {
					let f = s.devices[foot].pose[i];
					let ax = cross(j.ankle[side] - j.knee[side], f.p - j.ankle[side]);
					if len(ax) > 1e-6 {
						let q = axis_angle(norm(ax), FOOT_DEG.to_radians() * w);
						out.devices[foot].pose[i] = rot_about(&f, j.ankle[side], q);
					}
				}
				Kind::Twist => {
					let q = axis_angle(norm(j.wrist[side] - j.elbow[side]), TWIST_DEG.to_radians() * w);
					out.devices[hand].pose[i] = rot_about(&s.devices[hand].pose[i], j.wrist[side], q);
				}
				Kind::SwingWithHand | Kind::SwingHandStill => {
					let q = axis_angle(norm(j.wrist[side] - j.shoulder[side]), SWING_DEG.to_radians() * w);
					out.devices[elbow].pose[i] = rot_about(&s.devices[elbow].pose[i], j.shoulder[side], q);
					if kind == Kind::SwingWithHand {
						out.devices[hand].pose[i] = rot_about(&s.devices[hand].pose[i], j.shoulder[side], q);
					}
				}
			}
		}
	}
	out
}

/// The probes on one mocap file
#[allow(clippy::too_many_arguments)]
fn synthetic(
	model: &Net, norm: &Norm, f: &str, mount: &synth::Mount, seed: u64, found: &mut [Found; 4], air: &mut Found,
) -> Result<(), String> {
	let b = bvh::load(Path::new(f))?;
	let sk = synth::skeleton(&b)?;
	let (s, joints) = synth::session_joints(&b, &sk, mount, EYE, synth::JITTER, seed);
	let r = roles::assign(&s);
	if !r.notes.is_empty() {
		return Err(format!("left out: {}", r.notes.join("; ")));
	}
	let c = features::calibrate(&s, &r, &norm.layout)?;
	let d = devs(&r)?;
	let k = ik::calibrate(&[(&s, &r, &Dark::default())])?.remove(0);
	// the swing from the end of the rig's T-pose; scored every other frame from half a second on
	let t0 = 2.0f64.max(r.tpose_t + 1.0);
	let end = s.devices[c.dev[0]].t.last().copied().unwrap_or(0.0);
	let mut times = Vec::new();
	let mut t = t0 + 0.5;
	while t < end {
		if tracking(&s, &c, t) {
			times.push(t);
		}
		t += 2.0 / features::FPS;
	}
	let base = predict(model, norm, &s, &c, &times)?;
	let ik_base: Vec<ik::Trackers> = times.iter().map(|&t| ik_at(&k, &s, &d, t)).collect();
	for (q, &kind) in KINDS.iter().enumerate() {
		let sp = perturb(&s, &joints, &d, kind, t0);
		let pert = predict(model, norm, &sp, &c, &times)?;
		let elbow = kind != Kind::Foot;
		for (i, &t) in times.iter().enumerate() {
			let w = (2.0 * std::f64::consts::PI * HZ * (t - t0)).sin();
			if w.abs() < PAST || !tracking(&sp, &c, t) {
				continue;
			}
			let (Some(b0), Some(b1)) = (&base[i], &pert[i]) else { continue };
			let ik1 = ik_at(&k, &sp, &d, t);
			let ji = s.devices[d.head].at(t + 1e-6);
			for side in 0..2 {
				let truth = if elbow { d.elbow[side] } else { d.knee[side] };
				let (truth0, truth1) = (pose(&s, truth, t), pose(&sp, truth, t));
				let in_air = !elbow && joints[ji].ankle[side].y - joints[0].ankle[side].y > AIR;
				let mut ours = vec![(1, ik_side(&ik_base[i], elbow, side), ik_side(&ik1, elbow, side))];
				if let Some(p) = target(&norm.layout, elbow, side) {
					ours.push((0, b0[p], b1[p]));
				}
				for (who, a, z) in &ours {
					record(&mut found[q], *who, a, z, &truth0, &truth1);
					if in_air {
						record(air, *who, a, z, &truth0, &truth1);
					}
				}
			}
		}
	}
	Ok(())
}

/// The three looked for in one truth recording
struct Natural {
	foot: Found,
	twist: Found,
	chicken: Found,
}

fn natural(model: &Net, norm: &Norm, s: &Session, r: &Roles, k: &ik::Calib, nat: &mut Natural) -> Result<(), String> {
	let c = features::calibrate(s, r, &norm.layout)?;
	let d = devs(r)?;
	let end = s.devices[d.head].t.last().copied().unwrap_or(0.0);
	let mut times = Vec::new();
	let mut t = r.tpose_t + 1.0;
	while t < end {
		times.push(t);
		t += 1.0 / features::FPS;
	}
	let pred = predict(model, norm, s, &c, &times)?;
	let lag = (WINDOW * features::FPS).round() as usize;
	let moved = |k_: usize, a: f64, b: f64| len(pose(s, k_, b).p - pose(s, k_, a).p);
	let turned = |k_: usize, a: f64, b: f64| qangle(qmul(qconj(pose(s, k_, a).q), pose(s, k_, b).q));
	for i in lag..times.len() {
		let (a, b) = (times[i - lag], times[i]);
		let (Some(pa), Some(pb)) = (&pred[i - lag], &pred[i]) else { continue };
		let all_lit = |ks: &[usize]| ks.iter().all(|&j| lit(s, j, a) && lit(s, j, b));
		let (ika, ikb) = (ik_at(k, s, &d, a), ik_at(k, s, &d, b));
		for side in 0..2 {
			let (hand, foot, knee, elbow) = (d.hand[side], d.foot[side], d.knee[side], d.elbow[side]);
			let add = |fo: &mut Found, is_elbow: bool, truth: usize| {
				let (t0, t1) = (pose(s, truth, a), pose(s, truth, b));
				record(fo, 1, &ik_side(&ika, is_elbow, side), &ik_side(&ikb, is_elbow, side), &t0, &t1);
				if let Some(p) = target(&norm.layout, is_elbow, side) {
					record(fo, 0, &pa[p], &pb[p], &t0, &t1);
				}
			};
			if all_lit(&[d.head, d.waist, foot, knee])
				&& turned(foot, a, b) > 20f64.to_radians()
				&& moved(knee, a, b) < 0.02
				&& turned(knee, a, b) < 5f64.to_radians()
				&& moved(d.waist, a, b) < 0.03
			{
				add(&mut nat.foot, false, knee);
			}
			if all_lit(&[d.head, d.waist, hand, elbow]) {
				if turned(hand, a, b) > 30f64.to_radians()
					&& moved(hand, a, b) < 0.05
					&& moved(elbow, a, b) < 0.02
					&& turned(elbow, a, b) < 5f64.to_radians()
				{
					add(&mut nat.twist, true, elbow);
				}
				if moved(elbow, a, b) > 0.05 && moved(hand, a, b) < 0.03 {
					add(&mut nat.chicken, true, elbow);
				}
			}
		}
	}
	Ok(())
}

fn report(name: &str, f: &mut Found, swing: bool) {
	let frames = f.moved[0].n().max(f.moved[1].n());
	if frames == 0 {
		println!("  {name}: none");
		return;
	}
	println!("  {name} ({frames} frames)");
	if swing {
		println!("    the truth moves {} mm (median / p90)", f.truth.mp(1e3));
	}
	for (who, name) in WHO.iter().enumerate() {
		if f.moved[who].n() == 0 {
			continue;
		}
		let follows =
			if swing { format!(", follows {:.0} % of it", f.follows[who].q(0.5) * 1e2) } else { String::new() };
		println!(
			"    {:<5} moves {} mm, turns {} deg{follows}; its error {} mm before, {} mm after",
			name,
			f.moved[who].mp(1e3),
			f.turned[who].mp(1f64.to_degrees()),
			f.err0[who].mp(1e3),
			f.err1[who].mp(1e3)
		);
	}
}

pub fn probe(args: &[String]) -> Result<(), String> {
	let (mut dir, mut mounts) = (None, None);
	let mut files: Vec<(String, String)> = Vec::new();
	let mut i = 0;
	while i < args.len() {
		match args[i].as_str() {
			"--model" => dir = args.get(i + 1).cloned(),
			"--mounts" => mounts = args.get(i + 1).cloned(),
			_ => {
				files.push((dir.clone().ok_or("probe needs --model DIR before its files")?, args[i].clone()));
				i += 1;
				continue;
			}
		}
		i += 2;
	}
	let mounts = match mounts {
		Some(m) => synth::load_mounts(Path::new(&m))?,
		None => Vec::new(),
	};
	let mut loaded: Option<(String, Norm, Net)> = None;
	fn load(loaded: &mut Option<(String, Norm, Net)>, dir: &String) -> Result<(), String> {
		if loaded.as_ref().map(|l| &l.0) != Some(dir) {
			let d = PathBuf::from(dir);
			let norm = read_norm(&d.join("model.txt"))?;
			let model = load_net(&d, &norm, &Device::default())?;
			*loaded = Some((dir.clone(), norm, model));
		}
		Ok(())
	}
	// the truth recordings: the IK calibrated on them together, as the yardstick is
	let csvs: Vec<&(String, String)> = files.iter().filter(|f| !f.1.ends_with(".bvh")).collect();
	let mut sessions = Vec::new();
	for (_, f) in &csvs {
		let s = poses::load(Path::new(f))?;
		let r = roles::assign(&s);
		sessions.push((s, r));
	}
	let dark = Dark::default();
	let list: Vec<(&Session, &Roles, &Dark)> = sessions.iter().map(|(s, r)| (s, r, &dark)).collect();
	let iks = ik::calibrate(&list)?;
	let mut nat = Natural { foot: Found::default(), twist: Found::default(), chicken: Found::default() };
	for (j, (dir, f)) in csvs.iter().enumerate() {
		load(&mut loaded, dir)?;
		let (_, norm, model) = loaded.as_ref().expect("loaded");
		natural(model, norm, &sessions[j].0, &sessions[j].1, &iks[j], &mut nat).map_err(|e| format!("{f}: {e}"))?;
	}
	let mut found: [Found; 4] = Default::default();
	let mut air = Found::default();
	let mut used = 0;
	for (n, (dir, f)) in files.iter().enumerate().filter(|(_, f)| f.1.ends_with(".bvh")) {
		if mounts.is_empty() {
			return Err("a BVH needs --mounts FILE".into());
		}
		load(&mut loaded, dir)?;
		let (_, norm, model) = loaded.as_ref().expect("loaded");
		match synthetic(model, norm, f, &mounts[n % mounts.len()], 2_000_000 + n as u64, &mut found, &mut air) {
			Ok(()) => used += 1,
			Err(e) => eprintln!("{f}: {e}"),
		}
	}
	if used > 0 {
		println!(
			"the probes on mocap ({used} files, both sides; each virtual tracker against itself on the same frames \
			 unmoved, median / p90):"
		);
		for (q, &kind) in KINDS.iter().enumerate() {
			report(&kind_name(kind), &mut found[q], kind != Kind::Foot && kind != Kind::Twist);
			if kind == Kind::Foot {
				report(&format!("  the same with the foot over {:.0} cm up", AIR * 1e2), &mut air, false);
			}
		}
	}
	if !csvs.is_empty() {
		println!(
			"the same in the truth recordings ({} files, half-second stretches, both sides; median / p90):",
			csvs.len()
		);
		report("the foot turned over 20 deg, the real knee still", &mut nat.foot, false);
		report("the hand turned over 30 deg in place, the real elbow still", &mut nat.twist, false);
		report("the real elbow moved over 5 cm, the hand in place", &mut nat.chicken, true);
	}
	Ok(())
}
