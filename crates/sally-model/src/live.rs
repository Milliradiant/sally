//! The virtual trackers live (docs/mocap_plan.md section 5): SteamVR's poses as they come, the T-pose looked for in
//! them (`roles::assign` on the last SEARCH_KEEP s, every SEARCH_EVERY s), the session calibrated on it with the
//! virtual trackers' places (`features::calibrate_live`), then every frame the model's poses (its targets: the chest,
//! knees and elbows, or the knees and elbows where the chest is an input) with their velocities. `run` is the process
//! the sally driver talks to over loopback UDP; `replay` puts a recording through the same path and scores it against
//! the truth trackers in it.
//!
//! The driver's datagrams (one per SteamVR frame): `F <t>` then `R <t> <class> <serial> <valid> x y z qw qx qy qz` for
//! each device SteamVR has (its raw universe: the world the virtual trackers are handed back in), `C calibrate` to look
//! for the T-pose again. The answers: `T <name> <valid> x y z qw qx qy qz vx vy vz wx wy wz` for each target (one
//! datagram; the angular velocity in the tracker's own frame, as the driver hands SteamVR its devices' rates), and
//! every second `S <state> <seconds in it> <what>`.

use std::collections::{BTreeMap, VecDeque};
use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::time::Duration;

use burn::prelude::*;
use sally_body::features::{self, Calib, Layout};
use sally_body::poses::{self, Device as LogDevice, Session};
use sally_body::roles::{self, Role, Roles};
use sally_math::math3d::{len, norm, qangle, qconj, qmul, qrot, Pose, Q4, V3};

use crate::{load_net, normalize, read_norm, tensor, Net, Norm};

/// How far back the T-pose is looked for while searching, s
const SEARCH_KEEP: f64 = 30.0;
/// How much history is kept once tracking (the model sees 0.5 s), s
const TRACK_KEEP: f64 = 1.0;
/// How often the T-pose is looked for, s
const SEARCH_EVERY: f64 = 0.5;
/// The velocity's baseline: the output this long before, s
const VEL_DT: f64 = 1.0 / 30.0;
/// An input device unheard or not tracking for this long: the virtual trackers out of range; unheard this long: the
/// T-pose looked for again (another device may be on that body part now), s
const STALE: f64 = 0.25;
const GONE: f64 = 5.0;

/// Where the model's virtual trackers sit in the T-pose (DIR\\places.txt, written by `tune`: the median over the
/// recordings it tuned on of where the real ones sat, from the headset in the body's frame, in statures), in its
/// targets' order
pub fn read_places(path: &Path, layout: &Layout) -> Result<Vec<V3>, String> {
	let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
	let mut out = vec![None; layout.targets.len()];
	for l in text.lines().filter(|l| !l.starts_with('#')) {
		let w: Vec<&str> = l.split_whitespace().collect();
		if let ["place", name, x, y, z] = w.as_slice() {
			if let (Some(k), Ok(x), Ok(y), Ok(z)) =
				(features::by_short_name(name).and_then(|r| layout.target(r)), x.parse(), y.parse(), z.parse())
			{
				out[k] = Some(V3::new(x, y, z));
			}
		}
	}
	out.iter()
		.zip(&layout.targets)
		.map(|(p, &r)| p.ok_or_else(|| format!("{}: no place for the {}", path.display(), features::short_name(r))))
		.collect()
}

pub fn write_places(path: &Path, layout: &Layout, places: &[V3], from: usize) -> std::io::Result<()> {
	let mut s = format!(
		"# sally-model: where the virtual trackers sit in the T-pose, from the headset in the body's frame (x right, y up, \
		 z back), in statures: the median over {from} truth recordings of where the real ones sat\n"
	);
	for (&r, p) in layout.targets.iter().zip(places) {
		s += &format!("place {} {:.6} {:.6} {:.6}\n", features::short_name(r), p.x, p.y, p.z);
	}
	std::fs::write(path, s)
}

/// The model, loaded: its weights, normalisation and the virtual trackers' places
pub struct Model {
	norm: Norm,
	mlp: Net,
	pub places: Vec<V3>,
	device: Device,
}

impl Model {
	pub fn load(dir: &Path) -> Result<Model, String> {
		let norm = read_norm(&dir.join("model.txt"))?;
		let places = read_places(&dir.join("places.txt"), &norm.layout)?;
		let device = Device::default();
		let mlp = load_net(dir, &norm, &device)?;
		Ok(Model { norm, mlp, places, device })
	}

	pub fn layout(&self) -> &Layout {
		&self.norm.layout
	}

	/// The targets' poses from one frame's inputs
	fn predict(&self, c: &Calib, x: Vec<f32>, frame: &Pose) -> Result<Vec<Pose>, String> {
		let (n_in, n_out) = (self.layout().n_in(), self.layout().n_out());
		let mut x = x;
		normalize(&mut x, n_in, &self.norm.xm, &self.norm.xs);
		let y = self.mlp.forward(tensor(&x, 1, n_in, &self.device));
		let mut o: Vec<f32> = y.into_data().try_to_vec::<f32>().map_err(|e| format!("{e:?}"))?;
		for (k, v) in o.iter_mut().enumerate().take(n_out) {
			*v = *v * self.norm.ys[k] + self.norm.ym[k];
		}
		Ok(features::decode(c, frame, &o))
	}
}

/// One virtual tracker's pose for SteamVR
#[derive(Clone, Copy, Debug, Default)]
pub struct Out {
	pub valid: bool,
	pub pose: Pose,
	/// m/s, in the world
	pub vel: V3,
	/// rad/s, in the tracker's own frame
	pub rate: V3,
}

enum State {
	Searching { since: f64, last_try: f64 },
	Tracking { calib: Box<Calib>, since: f64 },
}

pub struct Live {
	model: Model,
	session: Session,
	by_serial: BTreeMap<String, usize>,
	/// per device: when it was last heard, and last heard tracking
	heard: Vec<(f64, f64)>,
	state: State,
	outs: VecDeque<(f64, Vec<Pose>)>,
	/// the last frame's time, and the last trim's
	now: f64,
	trimmed: f64,
	/// what to say (the log), taken by the caller
	pub said: Vec<String>,
	/// why the last T-pose found was not taken (a device the model needs missing), said once until it changes
	why_not: String,
}

impl Live {
	pub fn new(model: Model) -> Live {
		Live {
			model,
			session: Session { start_epoch: 0.0, devices: Vec::new() },
			by_serial: BTreeMap::new(),
			heard: Vec::new(),
			state: State::Searching { since: f64::NAN, last_try: f64::NEG_INFINITY },
			outs: VecDeque::new(),
			now: f64::NEG_INFINITY,
			trimmed: f64::NEG_INFINITY,
			said: Vec::new(),
			why_not: String::new(),
		}
	}

	/// The virtual trackers' names, in the order `step` gives them
	pub fn names(&self) -> Vec<&'static str> {
		self.model.layout().targets.iter().map(|&r| features::short_name(r)).collect()
	}

	/// One device's pose at `t` (`class` as sally-poses names them: hmd, controller, tracker)
	pub fn feed(&mut self, t: f64, class: &str, serial: &str, valid: bool, pose: Pose) {
		if class != "hmd" && class != "controller" && class != "tracker" {
			return;
		}
		let k = match self.by_serial.get(serial) {
			Some(&k) => k,
			None => {
				let k = self.session.devices.len();
				self.session.devices.push(LogDevice {
					serial: serial.to_string(),
					class: class.to_string(),
					t: Vec::new(),
					pose: Vec::new(),
					ok: Vec::new(),
				});
				self.heard.push((f64::NEG_INFINITY, f64::NEG_INFINITY));
				self.by_serial.insert(serial.to_string(), k);
				k
			}
		};
		let d = &mut self.session.devices[k];
		if d.t.last().is_some_and(|&last| t <= last) {
			return;
		}
		d.t.push(t);
		d.pose.push(pose);
		d.ok.push(valid);
		self.heard[k].0 = t;
		if valid {
			self.heard[k].1 = t;
		}
	}

	/// Look for the T-pose again (the virtual trackers out of range until it is found)
	pub fn calibrate(&mut self) {
		self.say("asked to look for the T-pose again".into());
		self.search_again();
	}

	fn search_again(&mut self) {
		for d in &mut self.session.devices {
			d.t.clear();
			d.pose.clear();
			d.ok.clear();
		}
		self.outs.clear();
		self.state = State::Searching { since: f64::NAN, last_try: f64::NEG_INFINITY };
	}

	fn say(&mut self, s: String) {
		self.said.push(s);
	}

	/// The state as the S line has it: its name, the seconds in it, what it is waiting on
	pub fn state(&self) -> (&'static str, f64, String) {
		match &self.state {
			State::Searching { since, .. } => (
				"searching",
				if since.is_nan() { 0.0 } else { self.now - since },
				"stand in a T-pose (still, hands level and wide) for a second".into(),
			),
			State::Tracking { since, .. } => ("tracking", self.now - since, String::new()),
		}
	}

	/// Samples before `from` dropped, one kept before it (the pose there), every TRACK_KEEP s at most
	fn trim(&mut self, from: f64) {
		if self.now - self.trimmed < 1.0 {
			return;
		}
		self.trimmed = self.now;
		for d in &mut self.session.devices {
			let cut = d.t.partition_point(|&x| x < from).saturating_sub(1);
			if cut > 0 {
				d.t.drain(..cut);
				d.pose.drain(..cut);
				d.ok.drain(..cut);
			}
		}
	}

	/// The T-pose's roles, if a T-pose that holds and gives every input is in what is kept; a T-pose that holds but
	/// lacks an input the model needs (a chest tracker not worn, for a model that sees one) is said why, once
	fn try_tpose(&mut self) -> Option<(Roles, Calib)> {
		let r = roles::assign(&self.session);
		// a tracker without a role (left lying, a spare, not tracking then) is fine: the inputs the model needs are
		// checked below
		if !r.tpose_ok {
			return None;
		}
		match features::calibrate_live(&self.session, &r, self.model.layout(), &self.model.places) {
			Ok(c) => Some((r, c)),
			Err(e) => {
				if e != self.why_not {
					self.say(format!("a T-pose, but not one to track from: {e} (this model's inputs: {})", {
						let n: Vec<&str> = self.model.layout().inputs.iter().map(|&x| x.name()).collect();
						n.join(", ")
					}));
					self.why_not = e;
				}
				None
			}
		}
	}

	/// The frame at `t`: the targets' poses once tracking (None while searching)
	pub fn step(&mut self, t: f64) -> Result<Option<Vec<Out>>, String> {
		self.now = t;
		if let State::Searching { since, last_try } = &mut self.state {
			if since.is_nan() {
				*since = t;
			}
			let due = t - *last_try >= SEARCH_EVERY;
			if due {
				*last_try = t;
			}
			self.trim(t - SEARCH_KEEP);
			if !due {
				return Ok(None);
			}
			if let Some((r, c)) = self.try_tpose() {
				let name =
					|role: Role| r.device(role).map(|k| self.session.devices[k].serial.clone()).unwrap_or_default();
				let parts: Vec<String> =
					c.layout.inputs.iter().map(|&role| format!("{} {}", role.name(), name(role))).collect();
				let said = format!(
					"T-pose found {:.1} s ago: headset {:.2} m up, hands {:.2} m apart, stature taken as {:.2} m; {}",
					t - r.tpose_t,
					r.eye,
					r.span,
					c.stature,
					parts.join(", ")
				);
				self.say(said);
				self.state = State::Tracking { calib: Box::new(c), since: t };
			}
			return Ok(None);
		}
		self.trim(t - TRACK_KEEP);
		let State::Tracking { calib, .. } = &self.state else { return Ok(None) };
		let calib = (**calib).clone();
		let inputs = &calib.dev[..calib.layout.inputs.len()];
		let nt = calib.layout.targets.len();
		// an input gone for good: look for the T-pose again
		if let Some(&k) = inputs.iter().find(|&&k| t - self.heard[k].0 > GONE) {
			let serial = self.session.devices[k].serial.clone();
			self.say(format!("{serial} unheard for {GONE:.0} s: looking for the T-pose again"));
			self.search_again();
			return Ok(None);
		}
		let stale = inputs.iter().any(|&k| t - self.heard[k].1 > STALE);
		let Some((x, frame)) = (!stale).then(|| features::inputs(&self.session, &calib, t)).flatten() else {
			return Ok(Some(vec![Out::default(); nt]));
		};
		let poses = self.model.predict(&calib, x, &frame)?;
		let back = self.outs.iter().rev().find(|(u, _)| t - u >= VEL_DT - 1e-6).cloned();
		let mut out = vec![Out::default(); nt];
		for (k, o) in out.iter_mut().enumerate() {
			let p = poses[k];
			let (vel, rate) = match back {
				Some((u, ref old)) if t - u < 4.0 * VEL_DT => {
					let dt = t - u;
					let mut dq = qmul(p.q, qconj(old[k].q));
					if dq.w < 0.0 {
						dq = Q4::new(-dq.w, -dq.x, -dq.y, -dq.z);
					}
					let axis = V3::new(dq.x, dq.y, dq.z);
					let w_world = if len(axis) > 1e-12 { norm(axis) * (qangle(dq) / dt) } else { V3::default() };
					((p.p - old[k].p) * (1.0 / dt), qrot(qconj(p.q), w_world))
				}
				_ => (V3::default(), V3::default()),
			};
			*o = Out { valid: true, pose: p, vel, rate };
		}
		self.outs.push_back((t, poses));
		while self.outs.front().is_some_and(|(u, _)| t - u > 4.0 * VEL_DT) {
			self.outs.pop_front();
		}
		Ok(Some(out))
	}
}

/// The T lines for the virtual trackers, by their names
pub fn t_lines(out: &[Out], names: &[&str]) -> String {
	let mut s = String::new();
	for (k, o) in out.iter().enumerate() {
		let (p, q, v, w) = (o.pose.p, o.pose.q, o.vel, o.rate);
		s += &format!(
			"T {} {} {:.5} {:.5} {:.5} {:.6} {:.6} {:.6} {:.6} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}\n",
			names[k], o.valid as i32, p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z
		);
	}
	s
}

/// An R line's device: (t, class, serial, valid, pose)
fn parse_r(l: &str) -> Option<(f64, String, String, bool, Pose)> {
	let w: Vec<&str> = l.split_whitespace().collect();
	if w.len() != 12 || w[0] != "R" {
		return None;
	}
	let n: Vec<f64> = w[5..12].iter().filter_map(|x| x.parse().ok()).collect();
	if n.len() != 7 || !n.iter().all(|x| x.is_finite()) {
		return None;
	}
	let t: f64 = w[1].parse().ok()?;
	let pose = Pose { p: V3::new(n[0], n[1], n[2]), q: sally_math::math3d::qnorm(Q4::new(n[3], n[4], n[5], n[6])) };
	Some((t, w[2].to_string(), w[3].to_string(), w[4] == "1", pose))
}

/// sally-model run --model DIR [--port N]: the process the driver talks to
pub fn run(args: &[String]) -> Result<(), String> {
	let (mut dir, mut port) = (None, 9106u16);
	let mut i = 0;
	while i < args.len() {
		let v = args.get(i + 1).cloned();
		match args[i].as_str() {
			"--model" => dir = v,
			"--port" => port = v.and_then(|x| x.parse().ok()).unwrap_or(port),
			other => return Err(format!("run: {other}?")),
		}
		i += 2;
	}
	let dir = PathBuf::from(dir.ok_or("run needs --model DIR")?);
	let model = Model::load(&dir)?;
	let sock = UdpSocket::bind(("127.0.0.1", port)).map_err(|e| format!("port {port}: {e}"))?;
	sock.set_read_timeout(Some(Duration::from_millis(100))).map_err(|e| e.to_string())?;
	println!("sally-model run: {} on 127.0.0.1:{port}, waiting for the sally driver's poses", dir.display());
	let mut live = Live::new(model);
	let mut buf = vec![0u8; 65536];
	let mut last_status = f64::NEG_INFINITY;
	let mut errors = 0u64;
	loop {
		let (n, from) = match sock.recv_from(&mut buf) {
			Ok(x) => x,
			Err(_) => continue,
		};
		let text = String::from_utf8_lossy(&buf[..n]).into_owned();
		let mut frame_t = None;
		for l in text.lines() {
			if let Some(t) = l.strip_prefix("F ") {
				frame_t = t.trim().parse::<f64>().ok();
			} else if l == "C calibrate" {
				live.calibrate();
			} else if let Some((t, class, serial, valid, pose)) = parse_r(l) {
				live.feed(t, &class, &serial, valid, pose);
			}
		}
		let Some(t) = frame_t else { continue };
		match live.step(t) {
			Ok(Some(out)) => {
				let _ = sock.send_to(t_lines(&out, &live.names()).as_bytes(), from);
			}
			Ok(None) => {}
			Err(e) => {
				errors += 1;
				if errors < 10 {
					println!("the model: {e}");
				}
			}
		}
		for s in live.said.drain(..) {
			println!("{s}");
		}
		if t - last_status >= 1.0 {
			last_status = t;
			let (name, secs, what) = live.state();
			let _ = sock.send_to(format!("S {name} {secs:.1} {what}\n").as_bytes(), from);
		}
	}
}

/// sally-model run --model DIR --replay [--own-places] POSES_CSV...: each recording through the live path frame by
/// frame (its T-pose found as live, the virtual trackers' places the model's), scored against its own truth trackers as
/// eval scores. Where a real tracker sat otherwise than the model's place, that offset is in the score (the knees of 10
/// Nov's recordings 3-6 cm apart from one to the next); `--own-places` gives the model each recording's own places, as
/// eval does, which holds the live path to eval's numbers
pub fn replay(dir: &Path, args: &[String]) -> Result<(), String> {
	let mut pooled: Vec<(Role, Vec<f64>)> = Vec::new();
	let own = args.iter().any(|a| a == "--own-places");
	for f in args.iter().filter(|a| *a != "--own-places") {
		let s = poses::load(Path::new(f))?;
		let truth = roles::assign(&s);
		let mut model = Model::load(dir)?;
		let layout = model.layout().clone();
		if own {
			// the recording's own places, as eval has them: the live path held to eval's numbers
			let c = features::calibrate(&s, &truth, &layout)?;
			for k in 0..layout.targets.len() {
				model.places[k] = c.tpose[layout.inputs.len() + k] - c.tpose[0];
			}
		}
		let mut live = Live::new(model);
		// every device's rows in time order, a frame at each of the headset's
		let head = truth.device(Role::Head).ok_or("no headset")?;
		let mut next = vec![0usize; s.devices.len()];
		let mut found = None;
		let mut errs: Vec<Vec<f64>> = vec![Vec::new(); layout.targets.len()];
		for &t in &s.devices[head].t {
			for (k, d) in s.devices.iter().enumerate() {
				while next[k] < d.t.len() && d.t[next[k]] <= t {
					let i = next[k];
					live.feed(d.t[i], &d.class, &d.serial, d.ok[i], d.pose[i]);
					next[k] += 1;
				}
			}
			let out = live.step(t)?;
			for line in live.said.drain(..) {
				if found.is_none() {
					found = Some(t);
				}
				println!("  {line}");
			}
			let Some(out) = out else { continue };
			for (p, o) in out.iter().enumerate() {
				let Some(k) = truth.device(layout.targets[p]) else { continue };
				let d = &s.devices[k];
				let i = d.at(t + 1e-6);
				if o.valid && d.ok[i] && (d.t[i] - t).abs() < 0.02 {
					errs[p].push(len(o.pose.p - d.pose[i].p));
				}
			}
		}
		let mut line = Vec::new();
		for (p, e) in errs.iter_mut().enumerate() {
			e.sort_by(f64::total_cmp);
			let m = if e.is_empty() { f64::NAN } else { e[e.len() / 2] };
			let r = layout.targets[p];
			line.push(format!("{} {:.0} mm", r.name(), m * 1e3));
			match pooled.iter_mut().find(|x| x.0 == r) {
				Some(x) => x.1.extend(e.iter()),
				None => pooled.push((r, e.clone())),
			}
		}
		println!("{f}: tracking from {:.1} s: {}", found.unwrap_or(f64::NAN), line.join("  "));
	}
	let mut line = Vec::new();
	for (r, e) in pooled.iter_mut() {
		e.sort_by(f64::total_cmp);
		let m = if e.is_empty() { f64::NAN } else { e[e.len() / 2] };
		line.push(format!("{} {:.0} mm", r.name(), m * 1e3));
	}
	println!("all pooled: {}", line.join("  "));
	Ok(())
}
