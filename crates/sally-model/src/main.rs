//! sally-model: the virtual trackers' model (docs/mocap_plan.md section 3, "first of ours"): small MLPs over the input
//! devices' last 0.5 s that say where the chest, knees and elbows are (`sally_body::features`; with a real chest
//! tracker as an input, the knees and elbows), trained on the tracker rig put on motion capture (`sally_body::synth`),
//! scored on the truth recordings as the IK baseline is.
//!
//!   sally-model train --mounts FILE --out DIR [--epochs N] [--arch limbs|joint] [--hidden N] [--depth N]
//!                     [--with-chest] [--free] BVH...
//!   sally-model tune --model DIR --out DIR [--epochs N] [--rate R] [--mounts FILE] POSES_CSV... [BVH...]
//!   sally-model eval [--mounts FILE] [--split] [--crosstalk] --model DIR POSES_CSV_OR_BVH... [--model DIR FILE...]...
//!   sally-model run --model DIR [--port N]
//!   sally-model run --model DIR --replay [--own-places] POSES_CSV...
//!   sally-model probe [--mounts FILE] --model DIR POSES_CSV_OR_BVH... [--model DIR FILE...]...
//!
//! `train` writes DIR\model.bpk (the weights, burnpack) and DIR\model.txt (what it sees and says, the sizes, the
//! inputs' and targets' means and sd); `--with-chest` makes a real chest tracker an input and the chest no target
//! (`Layout::with_chest`), `--free` puts the joints' free motions on the mocap (`synth::free_motions`; `tune` keeps
//! them for the mocap it mixes in). A BVH whose path has `/eval/` in it, and every 20th of the others, is held out to
//! watch the loss on. `tune` carries a trained model on over truth recordings (the wearer's own body, rig and T-pose),
//! its rate from R down to a hundredth of it, with mocap mixed in if BVH files are given; a recording tuned on is no
//! longer one to score it on. `eval` takes truth recordings as `sally-poses` writes them (a device marked not valid is
//! one without light), or BVH files with the rig put on them as in training (`--mounts`), to tell the model's own error
//! from the gap to the real; `--split` adds each part's error split into what stays the same through a file and the
//! rest (`split_line`). Each file is scored with the model named before it, so the folds of a tuning are pooled in one
//! table. `--crosstalk` adds how far each target moves when one input device's motion is swapped for another moment's
//! (`crosstalk_of`). `run` is the virtual trackers live, the process the sally driver feeds SteamVR's poses to
//! (`live`); with `--replay` it puts recordings through the same path. `tune` writes DIR\\places.txt too, where the
//! virtual trackers sit in the T-pose. `probe` looks at where an IK gives itself away (`probe`).

mod live;
mod probe;

use std::path::{Path, PathBuf};

use burn::nn::{Gelu, Linear, LinearConfig};
use burn::optim::{AdamConfig, GradientsParams};
use burn::prelude::*;
use burn::store::{BurnpackStore, ModuleSnapshot};
use burn::tensor::TensorData;
use sally_body::features::{self, Layout};
use sally_body::poses::{self, Session};
use sally_body::roles::Role;
use sally_body::{bvh, roles, synth};
use sally_math::math3d::{len, norm, qangle, qconj, qmul, qnorm, qrot, Q4, V3};

#[derive(Module, Debug)]
struct Mlp {
	layers: Vec<Linear>,
	act: Gelu,
}

impl Mlp {
	fn new(device: &Device, n_in: usize, hidden: usize, depth: usize, n_out: usize) -> Mlp {
		let mut layers = Vec::new();
		let mut n = n_in;
		for _ in 0..depth {
			layers.push(LinearConfig::new(n, hidden).init(device));
			n = hidden;
		}
		layers.push(LinearConfig::new(n, n_out).init(device));
		Mlp { layers, act: Gelu::new() }
	}

	fn forward(&self, x: Tensor<2>) -> Tensor<2> {
		let mut x = x;
		let last = self.layers.len() - 1;
		for (i, l) in self.layers.iter().enumerate() {
			x = l.forward(x);
			if i < last {
				x = self.act.forward(x);
			}
		}
		x
	}
}

/// Each virtual tracker's own network's devices (by index into the layout's inputs, in their order): what moves that
/// body part. A knee the waist and its own foot, an elbow the headset, the waist and its own hand (and a real chest
/// tracker where one is an input: the shoulder rides the thorax); the chest the upper body, the headset, the waist and
/// both hands (a thorax turns with the arms when it twists, reaches or walks: with the headset and the waist alone its
/// error was 27 mm / 7.0 deg against 21 / 6.1 with every input, 14 deg against 6 standing and twisting). One network
/// over every input (the first runs, `joint`) has a hand's motion move the knees and one hand's the other elbow (`eval
/// --crosstalk`: 8-14 mm at the median, 27-54 at p90); one per tracker over its own (`limbs`) cannot. A model keeps its
/// own in model.txt (`sees` lines); these are what `train` gives a new one
fn default_sees(layout: &Layout, target: Role) -> Vec<usize> {
	let roles: &[Role] = match target {
		Role::Chest => &[Role::Head, Role::LHand, Role::RHand, Role::Waist],
		Role::LKnee => &[Role::Waist, Role::LFoot],
		Role::RKnee => &[Role::Waist, Role::RFoot],
		Role::LElbow => &[Role::Head, Role::LHand, Role::Waist, Role::Chest],
		Role::RElbow => &[Role::Head, Role::RHand, Role::Waist, Role::Chest],
		_ => &[],
	};
	let mut v: Vec<usize> = roles.iter().filter_map(|&r| layout.input(r)).collect();
	v.sort();
	v
}

/// The input columns a tracker's network sees from its devices: their blocks (every lag), their T-pose places and its
/// own, the stature
fn limb_columns(layout: &Layout, k: usize, devices: &[usize]) -> Vec<usize> {
	let block = features::LAGS.len() * features::PER;
	let places = layout.inputs.len() * block;
	let mut cols = Vec::new();
	for &d in devices {
		cols.extend(d * block..(d + 1) * block);
	}
	for &d in devices {
		cols.extend(places + 3 * d..places + 3 * d + 3);
	}
	let t = layout.inputs.len() + k;
	cols.extend(places + 3 * t..places + 3 * t + 3);
	cols.push(layout.n_in() - 1);
	cols
}

/// The model: one MLP over every input (`joint`), or one per virtual tracker over the devices it sees (`limbs`), their
/// outputs side by side in features' order
#[derive(Module, Debug)]
struct Net {
	nets: Vec<Mlp>,
	/// each per-tracker network's input columns (none for a joint one)
	#[module(skip)]
	cols: Vec<Vec<i64>>,
}

impl Net {
	fn new(device: &Device, norm: &Norm) -> Net {
		let l = &norm.layout;
		let per = features::PER;
		if norm.arch != "limbs" {
			return Net {
				nets: vec![Mlp::new(device, l.n_in(), norm.hidden, norm.depth, l.n_out())],
				cols: Vec::new(),
			};
		}
		let cols: Vec<Vec<usize>> = (0..l.targets.len()).map(|k| limb_columns(l, k, &norm.sees[k])).collect();
		let nets = cols.iter().map(|c| Mlp::new(device, c.len(), norm.hidden, norm.depth, per)).collect();
		Net { nets, cols: cols.into_iter().map(|c| c.into_iter().map(|x| x as i64).collect()).collect() }
	}

	fn forward(&self, x: Tensor<2>) -> Tensor<2> {
		if self.nets.len() == 1 {
			return self.nets[0].forward(x);
		}
		let device = x.device();
		let outs: Vec<Tensor<2>> = self
			.nets
			.iter()
			.zip(self.cols.iter())
			.map(|(net, cols)| {
				let idx = Tensor::<1, Int>::from_data(TensorData::new(cols.clone(), [cols.len()]), &device);
				net.forward(x.clone().select(1, idx))
			})
			.collect();
		Tensor::cat(outs, 1)
	}
}

/// A model's network from DIR\\model.bpk, as its model.txt has it (a joint one from before the limbs, saved as an
/// MLP of its own, read into the one network it is)
fn load_net(dir: &Path, norm: &Norm, device: &Device) -> Result<Net, String> {
	let mut net = Net::new(device, norm);
	let mut store = BurnpackStore::from_file(dir.join("model.bpk")).with_remap_pattern("^layers", "nets.0.layers");
	net.load_from(&mut store).map_err(|e| format!("loading {}: {e:?}", dir.display()))?;
	Ok(net)
}

/// The headset's height over the feet the rig is made for, m (each body varied about it by the jitter's size)
const EYE: f64 = 1.56;

/// Samples: inputs and targets, row by row, `n_in` and `n_out` wide
struct Data {
	x: Vec<f32>,
	y: Vec<f32>,
	n_in: usize,
	n_out: usize,
}

impl Data {
	fn new(l: &Layout) -> Data {
		Data { x: Vec::new(), y: Vec::new(), n_in: l.n_in(), n_out: l.n_out() }
	}
	fn rows(&self) -> usize {
		self.y.len() / self.n_out
	}
}

/// Whether every device tracks over the window the inputs see at `t` (a synthetic session marks a broken file's jumps
/// as not tracking)
fn tracking(s: &Session, c: &features::Calib, t: f64) -> bool {
	c.dev.iter().all(|&k| {
		let d = &s.devices[k];
		features::LAGS.iter().all(|&lag| {
			let u = t - lag as f64 / features::FPS;
			u >= d.t[0] - 1e-6 && d.ok[d.at(u + 1e-6)]
		})
	})
}

/// A session's samples: one every `step` frames from 0.6 s after the T-pose on, where every device tracks
fn add_session(d: &mut Data, s: &Session, step: usize, layout: &Layout) -> Result<usize, String> {
	let r = roles::assign(s);
	if !r.notes.is_empty() {
		return Err(r.notes.join("; "));
	}
	let c = features::calibrate(s, &r, layout)?;
	let end = s.devices[c.dev[0]].t.last().copied().unwrap_or(0.0);
	let mut t = r.tpose_t + 0.6;
	let mut n = 0;
	while t < end {
		if !tracking(s, &c, t) {
			t += step as f64 / features::FPS;
			continue;
		}
		if let Some((x, frame)) = features::inputs(s, &c, t) {
			if let Some(y) = features::targets(s, &c, &frame, t) {
				d.x.extend(x);
				d.y.extend(y);
				n += 1;
			}
		}
		t += step as f64 / features::FPS;
	}
	Ok(n)
}

/// Per column mean and sd (sd at least 1e-3)
fn stats(v: &[f32], width: usize) -> (Vec<f32>, Vec<f32>) {
	let rows = v.len() / width;
	let mut m = vec![0f64; width];
	let mut q = vec![0f64; width];
	for r in 0..rows {
		for c in 0..width {
			let x = v[r * width + c] as f64;
			m[c] += x;
			q[c] += x * x;
		}
	}
	let n = rows.max(1) as f64;
	let mean: Vec<f32> = m.iter().map(|s| (s / n) as f32).collect();
	let sd: Vec<f32> =
		m.iter().zip(&q).map(|(s, qq)| (qq / n - (s / n) * (s / n)).max(1e-6).sqrt().max(1e-3) as f32).collect();
	(mean, sd)
}

fn normalize(v: &mut [f32], width: usize, mean: &[f32], sd: &[f32]) {
	for (i, x) in v.iter_mut().enumerate() {
		let c = i % width;
		*x = (*x - mean[c]) / sd[c];
	}
}

struct Norm {
	/// what it sees and says
	layout: Layout,
	/// `joint` or `limbs` (Net)
	arch: String,
	/// per target, the input devices its network sees (`limbs`)
	sees: Vec<Vec<usize>>,
	/// trained with the joints' free motions (synth::FREE) on the mocap
	free: bool,
	hidden: usize,
	depth: usize,
	xm: Vec<f32>,
	xs: Vec<f32>,
	ym: Vec<f32>,
	ys: Vec<f32>,
}

/// Roles by their short names, a space between
fn names(r: &[Role]) -> String {
	r.iter().map(|&x| features::short_name(x)).collect::<Vec<_>>().join(" ")
}

fn write_norm(path: &Path, n: &Norm) -> std::io::Result<()> {
	let line = |v: &[f32]| v.iter().map(|x| format!("{x:e}")).collect::<Vec<_>>().join(" ");
	let text = format!(
		"# sally-model: what it sees and says, the network (one over every input, or one per virtual tracker over the devices it sees) and its MLPs' sizes, then the inputs' mean and sd, the targets' mean and sd (sally_body::features)\ninputs {}\noutputs {}\n{}arch {}\n{}hidden {}\ndepth {}\nin {}\nout {}\ntargets {}\nxm {}\nxs {}\nym {}\nys {}\n",
		names(&n.layout.inputs),
		names(&n.layout.targets),
		if n.free { "free foot 15 8 twist 30 swing 12 deg tau 0.4 s\n" } else { "" },
		n.arch,
		if n.arch == "limbs" {
			n.layout
				.targets
				.iter()
				.enumerate()
				.map(|(k, &t)| {
					let d: Vec<&str> = n.sees[k].iter().map(|&i| features::short_name(n.layout.inputs[i])).collect();
					format!("sees {} {}\n", features::short_name(t), d.join(" "))
				})
				.collect::<String>()
		} else {
			String::new()
		},
		n.hidden,
		n.depth,
		n.layout.n_in(),
		n.layout.n_out(),
		features::TARGETS_ARE,
		line(&n.xm),
		line(&n.xs),
		line(&n.ym),
		line(&n.ys)
	);
	std::fs::write(path, text)
}

fn read_norm(path: &Path) -> Result<Norm, String> {
	let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
	// a model from before the inputs and outputs lines: the first layout
	let roles_of = |l: &str| -> Result<Vec<Role>, String> {
		l.split_whitespace()
			.skip(1)
			.map(|x| features::by_short_name(x).ok_or_else(|| format!("{}: no such device {x}", path.display())))
			.collect()
	};
	let mut layout = Layout::first();
	for l in text.lines().filter(|l| !l.starts_with('#')) {
		match l.split_whitespace().next() {
			Some("inputs") => layout.inputs = roles_of(l)?,
			Some("outputs") => layout.targets = roles_of(l)?,
			_ => {}
		}
	}
	if layout.input(Role::Waist).is_none() || layout.inputs.first() != Some(&Role::Head) {
		return Err(format!("{}: its inputs need the headset first and the waist", path.display()));
	}
	let mut n = Norm {
		sees: layout.targets.iter().map(|&t| default_sees(&layout, t)).collect(),
		layout,
		arch: "joint".into(),
		free: false,
		hidden: 0,
		depth: 0,
		xm: vec![],
		xs: vec![],
		ym: vec![],
		ys: vec![],
	};
	// a limbs model from before the sees lines: the chest saw the headset and the waist
	let mut said_sees = false;
	let mut targets = "";
	for l in text.lines().filter(|l| !l.starts_with('#')) {
		let mut w = l.split_whitespace();
		let key = w.next().unwrap_or("");
		if key == "inputs" || key == "outputs" {
			continue;
		}
		if key == "free" {
			n.free = true;
			continue;
		}
		if key == "targets" {
			targets = l["targets".len()..].trim();
			continue;
		}
		if key == "arch" {
			n.arch = w.next().unwrap_or("joint").to_string();
			continue;
		}
		if key == "sees" {
			let rest: Vec<&str> = w.collect();
			let k = rest.first().and_then(|t| features::by_short_name(t)).and_then(|r| n.layout.target(r));
			let devs: Option<Vec<usize>> =
				rest.iter().skip(1).map(|d| features::by_short_name(d).and_then(|r| n.layout.input(r))).collect();
			match (k, devs) {
				(Some(k), Some(d)) if !d.is_empty() => n.sees[k] = d,
				_ => return Err(format!("{}: a sees line it cannot read: {l}", path.display())),
			}
			said_sees = true;
			continue;
		}
		let vals: Vec<f32> = w.filter_map(|x| x.parse().ok()).collect();
		match key {
			"hidden" => n.hidden = vals.first().copied().unwrap_or(0.0) as usize,
			"depth" => n.depth = vals.first().copied().unwrap_or(0.0) as usize,
			"in" if vals.first().copied() != Some(n.layout.n_in() as f32) => return Err("made for other inputs".into()),
			"xm" => n.xm = vals,
			"xs" => n.xs = vals,
			"ym" => n.ym = vals,
			"ys" => n.ys = vals,
			_ => {}
		}
	}
	if n.arch == "limbs" && !said_sees && n.layout == Layout::first() {
		n.sees[0] = vec![0, 3];
	}
	if targets != features::TARGETS_ARE {
		return Err(format!(
			"{}: made for targets as \"{targets}\", these are \"{}\"",
			path.display(),
			features::TARGETS_ARE
		));
	}
	if n.xm.len() != n.layout.n_in() || n.ym.len() != n.layout.n_out() {
		return Err(format!("{}: sizes do not match", path.display()));
	}
	Ok(n)
}

fn tensor(v: &[f32], rows: usize, width: usize, device: &Device) -> Tensor<2> {
	Tensor::from_data(TensorData::new(v.to_vec(), [rows, width]), device)
}

/// A small shuffle (xorshift), seeded
fn shuffle(v: &mut [usize], seed: u64) {
	let mut s = seed | 1;
	for i in (1..v.len()).rev() {
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		v.swap(i, (s % (i as u64 + 1)) as usize);
	}
}

fn train(args: &[String]) -> Result<(), String> {
	let (mut mounts, mut out, mut epochs, mut hidden, mut depth) = (None, None, 20usize, 128usize, 3usize);
	let mut arch = "limbs".to_string();
	let (mut layout, mut free) = (Layout::first(), false);
	let mut files = Vec::new();
	let mut i = 0;
	while i < args.len() {
		let v = args.get(i + 1).cloned();
		match args[i].as_str() {
			"--with-chest" | "--free" => {
				if args[i] == "--free" {
					free = true;
				} else {
					layout = Layout::with_chest();
				}
				i += 1;
				continue;
			}
			"--mounts" => mounts = v,
			"--out" => out = v,
			"--epochs" => epochs = v.and_then(|x| x.parse().ok()).unwrap_or(epochs),
			"--hidden" => hidden = v.and_then(|x| x.parse().ok()).unwrap_or(hidden),
			"--depth" => depth = v.and_then(|x| x.parse().ok()).unwrap_or(depth),
			"--arch" => arch = v.unwrap_or(arch),
			_ => {
				files.push(args[i].clone());
				i += 1;
				continue;
			}
		}
		i += 2;
	}
	let mounts = synth::load_mounts(Path::new(&mounts.ok_or("train needs --mounts FILE")?))?;
	let out = PathBuf::from(out.ok_or("train needs --out DIR")?);
	std::fs::create_dir_all(&out).map_err(|e| format!("{}: {e}", out.display()))?;
	let (mut tr, mut va) = (Data::new(&layout), Data::new(&layout));
	let (mut ok, mut bad) = (0, 0);
	for (n, f) in files.iter().enumerate() {
		let held = f.contains("/eval/") || f.contains("\\eval\\") || n % 20 == 19;
		let res = (|| -> Result<usize, String> {
			let s = mocap_session(f, &mounts[n % mounts.len()], n as u64 + 1, free)?;
			add_session(if held { &mut va } else { &mut tr }, &s, 3, &layout)
		})();
		match res {
			Ok(_) => ok += 1,
			Err(e) => {
				eprintln!("{f}: left out: {e}");
				bad += 1;
			}
		}
	}
	println!("{ok} files ({bad} left out): {} training samples, {} held out", tr.rows(), va.rows());
	let (xm, xs) = stats(&tr.x, tr.n_in);
	let (ym, ys) = stats(&tr.y, tr.n_out);
	for d in [&mut tr, &mut va] {
		normalize(&mut d.x, d.n_in, &xm, &xs);
		normalize(&mut d.y, d.n_out, &ym, &ys);
	}
	if arch != "limbs" && arch != "joint" {
		return Err(format!("--arch {arch}: limbs or joint"));
	}
	let sees = layout.targets.iter().map(|&t| default_sees(&layout, t)).collect();
	let norm = Norm { layout, arch: arch.clone(), sees, free, hidden, depth, xm, xs, ym, ys };
	write_norm(&out.join("model.txt"), &norm).map_err(|e| e.to_string())?;
	let device = Device::default().autodiff();
	fit(Net::new(&device, &norm), &tr, &va, epochs, (1e-3, 1e-5), &out)
}

/// A mocap file with the rig put on it (`synth::session`), and the joints' free motions if `free`
fn mocap_session(f: &str, mount: &synth::Mount, seed: u64, free: bool) -> Result<Session, String> {
	let b = bvh::load(Path::new(f))?;
	let sk = synth::skeleton(&b)?;
	let (mut s, joints) = synth::session_joints(&b, &sk, mount, EYE, synth::JITTER, seed);
	if free {
		synth::free_motions(&mut s, &joints, synth::FREE, seed);
	}
	Ok(s)
}

/// Fits the model to the samples for `epochs` over all of `tr`, the rate from `rate.0` down to `rate.1` (cosine), Adam
/// on the mean squared error in batches of 512; the held-out loss after each epoch, the weights saved to DIR\model.bpk
/// after each
fn fit(mut model: Net, tr: &Data, va: &Data, epochs: usize, rate: (f64, f64), out: &Path) -> Result<(), String> {
	let device = Device::default().autodiff();
	let mut optim = AdamConfig::new().init();
	let batch = 512;
	let rows = tr.rows();
	let mut order: Vec<usize> = (0..rows).collect();
	let steps_total = (epochs * rows.div_ceil(batch)) as f64;
	let mut step = 0f64;
	let (n_in, n_out) = (tr.n_in, tr.n_out);
	let (mut xb, mut yb) = (Vec::with_capacity(batch * n_in), Vec::with_capacity(batch * n_out));
	for epoch in 0..epochs {
		let t0 = std::time::Instant::now();
		shuffle(&mut order, epoch as u64 + 7);
		let mut sum = 0.0;
		let mut nb = 0;
		for chunk in order.chunks(batch) {
			xb.clear();
			yb.clear();
			for &r in chunk {
				xb.extend_from_slice(&tr.x[r * n_in..(r + 1) * n_in]);
				yb.extend_from_slice(&tr.y[r * n_out..(r + 1) * n_out]);
			}
			let x = tensor(&xb, chunk.len(), n_in, &device);
			let y = tensor(&yb, chunk.len(), n_out, &device);
			let loss = (model.forward(x) - y).powi_scalar(2).mean();
			sum += loss.clone().into_scalar::<f32>() as f64;
			nb += 1;
			let grads = GradientsParams::from_grads(loss.backward(), &model);
			let lr = rate.1 + 0.5 * (rate.0 - rate.1) * (1.0 + (std::f64::consts::PI * step / steps_total).cos());
			model = optim.step(lr, model, grads);
			step += 1.0;
		}
		let held =
			if va.rows() > 0 { format!(", held out {:.4}", val_loss(&model.valid(), va)) } else { String::new() };
		println!(
			"epoch {epoch}: training loss {:.4}{held} ({:.0} s)",
			sum / nb.max(1) as f64,
			t0.elapsed().as_secs_f64()
		);
		model
			.valid()
			.save_into(&mut BurnpackStore::from_file(out.join("model.bpk")).overwrite(true))
			.map_err(|e| format!("saving: {e:?}"))?;
	}
	Ok(())
}

/// Fine-tunes a model on truth recordings (every 60 Hz frame where every device has light over the window), from its
/// weights and with its normalisation, into another folder
fn tune(args: &[String]) -> Result<(), String> {
	let (mut dir, mut out, mut epochs, mut rate, mut mounts) = (None, None, 20usize, 1e-4f64, None);
	let mut files = Vec::new();
	let mut i = 0;
	while i < args.len() {
		let v = args.get(i + 1).cloned();
		match args[i].as_str() {
			"--model" => dir = v,
			"--out" => out = v,
			"--mounts" => mounts = v,
			"--epochs" => epochs = v.and_then(|x| x.parse().ok()).unwrap_or(epochs),
			"--rate" => rate = v.and_then(|x| x.parse().ok()).unwrap_or(rate),
			_ => {
				files.push(args[i].clone());
				i += 1;
				continue;
			}
		}
		i += 2;
	}
	let dir = PathBuf::from(dir.ok_or("tune needs --model DIR")?);
	let out = PathBuf::from(out.ok_or("tune needs --out DIR")?);
	std::fs::create_dir_all(&out).map_err(|e| format!("{}: {e}", out.display()))?;
	let norm = read_norm(&dir.join("model.txt"))?;
	let mounts = match mounts {
		Some(m) => synth::load_mounts(Path::new(&m))?,
		None => Vec::new(),
	};
	let layout = norm.layout.clone();
	let mut tr = Data::new(&layout);
	let (mut real, mut synthetic) = (0, 0);
	// where the real targets sat in the T-pose from the headset, per recording, for the virtual ones
	let mut places: Vec<Vec<V3>> = Vec::new();
	for (n, f) in files.iter().enumerate() {
		if f.ends_with(".bvh") {
			// mocap kept in the mix, as in training, so what the recordings lack is not forgotten
			if mounts.is_empty() {
				return Err("a BVH needs --mounts FILE".into());
			}
			let res = (|| -> Result<usize, String> {
				let s = mocap_session(f, &mounts[n % mounts.len()], 2_000_000 + n as u64, norm.free)?;
				add_session(&mut tr, &s, 3, &layout)
			})();
			match res {
				Ok(k) => synthetic += k,
				Err(e) => eprintln!("{f}: left out: {e}"),
			}
			continue;
		}
		let s = poses::load(Path::new(f))?;
		let k = add_session(&mut tr, &s, 1, &layout).map_err(|e| format!("{f}: {e}"))?;
		let c = features::calibrate(&s, &roles::assign(&s), &layout)?;
		places.push((0..layout.targets.len()).map(|k| c.tpose[layout.inputs.len() + k] - c.tpose[0]).collect());
		println!("{f}: {k} samples");
		real += k;
	}
	if !places.is_empty() {
		// the median, axis by axis: one recording with a tracker worn otherwise (the crawling session's chest on the
		// back) does not move it
		let median = |k: usize, ax: usize| {
			let mut v: Vec<f64> = places.iter().map(|p| p[k].get(ax)).collect();
			v.sort_by(f64::total_cmp);
			let n = v.len();
			if n % 2 == 1 {
				v[n / 2]
			} else {
				(v[n / 2 - 1] + v[n / 2]) / 2.0
			}
		};
		let mid: Vec<V3> =
			(0..layout.targets.len()).map(|k| V3::new(median(k, 0), median(k, 1), median(k, 2))).collect();
		live::write_places(&out.join("places.txt"), &layout, &mid, places.len()).map_err(|e| e.to_string())?;
	}
	println!("{real} samples from recordings, {synthetic} from mocap");
	normalize(&mut tr.x, tr.n_in, &norm.xm, &norm.xs);
	normalize(&mut tr.y, tr.n_out, &norm.ym, &norm.ys);
	write_norm(&out.join("model.txt"), &norm).map_err(|e| e.to_string())?;
	let device = Device::default().autodiff();
	let model = load_net(&dir, &norm, &device)?;
	fit(model, &tr, &Data::new(&layout), epochs, (rate, rate / 100.0), &out)
}

fn val_loss(model: &Net, d: &Data) -> f64 {
	let device = Device::default();
	let rows = d.rows();
	let (mut sum, mut n) = (0.0, 0usize);
	for start in (0..rows).step_by(4096) {
		let k = (rows - start).min(4096);
		let x = tensor(&d.x[start * d.n_in..(start + k) * d.n_in], k, d.n_in, &device);
		let y = tensor(&d.y[start * d.n_out..(start + k) * d.n_out], k, d.n_out, &device);
		let l = (model.forward(x) - y).powi_scalar(2).mean().into_scalar::<f32>() as f64;
		sum += l * k as f64;
		n += k;
	}
	sum / n.max(1) as f64
}

/// The parts any model may score (features::TARGETS) and the input devices any may see (the first inputs and a real
/// chest), for the pooled tables
const ALL_INPUTS: [Role; 7] =
	[Role::Head, Role::LHand, Role::RHand, Role::Waist, Role::LFoot, Role::RFoot, Role::Chest];

/// Which devices (by index into the calibration's: the inputs, then the truth trackers) need light for target `k`'s
/// frame to count, as the yardstick has it: the headset, the waist, the limb's own input (a knee's foot, an elbow's
/// hand) and its truth tracker
fn needs(l: &Layout, k: usize) -> Vec<usize> {
	let own = match l.targets[k] {
		Role::LKnee => Some(Role::LFoot),
		Role::RKnee => Some(Role::RFoot),
		Role::LElbow => Some(Role::LHand),
		Role::RElbow => Some(Role::RHand),
		_ => None,
	};
	let mut v = vec![0, l.input(Role::Waist).expect("the waist is in")];
	v.extend(own.and_then(|r| l.input(r)));
	v.push(l.inputs.len() + k);
	v
}

/// A part's error over a file split into what stays the same through it and the rest: the mean position error in the
/// body's frame (x right, y up, z back), the median of what is left; the mean turn and its axis in the body's frame as
/// it stood in the T-pose (x: an arm's own axis), the median left. A constant part that dominates is the rig's or the
/// T-pose's difference from the training data, not the motion's
fn split_line(name: &str, e: &[(V3, Q4)]) -> String {
	if e.is_empty() {
		return format!("{name} -");
	}
	let n = e.len() as f64;
	let m = e.iter().fold(V3::default(), |a, (v, _)| a + *v) * (1.0 / n);
	let mut q = Q4::new(0.0, 0.0, 0.0, 0.0);
	for (_, r) in e {
		let k = if r.w < 0.0 { -1.0 } else { 1.0 };
		q = Q4::new(q.w + k * r.w, q.x + k * r.x, q.y + k * r.y, q.z + k * r.z);
	}
	let q = qnorm(q);
	let ax = norm(V3::new(q.x, q.y, q.z)) * if q.w < 0.0 { -1.0 } else { 1.0 };
	let mut rest: Vec<f64> = e.iter().map(|(v, _)| len(*v - m)).collect();
	let mut turn: Vec<f64> = e.iter().map(|(_, r)| qangle(qmul(qconj(q), *r))).collect();
	rest.sort_by(f64::total_cmp);
	turn.sort_by(f64::total_cmp);
	format!(
		"{name} [{:.0} {:.0} {:.0}] mm + {:.0}, {:.0} deg about [{:.2} {:.2} {:.2}] + {:.0}",
		m.x * 1e3,
		m.y * 1e3,
		m.z * 1e3,
		rest[rest.len() / 2] * 1e3,
		qangle(q).to_degrees(),
		ax.x,
		ax.y,
		ax.z,
		turn[turn.len() / 2].to_degrees()
	)
}

/// Crosstalk: on every 10th frame with a frame 2 s on in the same stretch, each input device's last 0.5 s (its block of
/// the inputs) swapped for its own 2 s later, the rest as it was; how far each target's place moved, into `cross`
/// (per device of ALL_INPUTS, per target of features::TARGETS). A hand's motion moves its own elbow; whatever it moves
/// the knees by is crosstalk
#[allow(clippy::too_many_arguments)]
fn crosstalk_of(
	model: &Net, norm: &Norm, c: &features::Calib, frames: &[sally_math::math3d::Pose], times: &[f64], xs: &[f32],
	outs: &[f32], cross: &mut [Vec<Vec<f64>>],
) -> Result<(), String> {
	let device = Device::default();
	let l = &c.layout;
	let (n_in, n_out) = (l.n_in(), l.n_out());
	let lag = 2 * features::FPS as usize;
	let block = features::LAGS.len() * features::PER;
	let n = frames.len();
	let (mut batch, mut which) = (Vec::new(), Vec::new());
	for j in (0..n.saturating_sub(lag)).step_by(10) {
		if (times[j + lag] - times[j] - 2.0).abs() > 0.05 {
			continue;
		}
		for d in 0..l.inputs.len() {
			let mut x = xs[j * n_in..(j + 1) * n_in].to_vec();
			let (b, e) = (d * block, (d + 1) * block);
			x[b..e].copy_from_slice(&xs[(j + lag) * n_in + b..(j + lag) * n_in + e]);
			batch.extend(x);
			which.push((j, d));
		}
	}
	let decode = |o: &[f32], j: usize| {
		let o: Vec<f32> = o.iter().enumerate().map(|(k, v)| v * norm.ys[k] + norm.ym[k]).collect();
		features::decode(c, &frames[j], &o)
	};
	for start in (0..which.len()).step_by(4096) {
		let k = (which.len() - start).min(4096);
		let y = model.forward(tensor(&batch[start * n_in..(start + k) * n_in], k, n_in, &device));
		let v: Vec<f32> = y.into_data().try_to_vec::<f32>().map_err(|e| format!("{e:?}"))?;
		for r in 0..k {
			let (j, d) = which[start + r];
			let was = decode(&outs[j * n_out..(j + 1) * n_out], j);
			let now = decode(&v[r * n_out..(r + 1) * n_out], j);
			let row = ALL_INPUTS.iter().position(|&x| x == l.inputs[d]).expect("an input of ALL_INPUTS");
			for (p, &t) in l.targets.iter().enumerate() {
				let col = features::TARGETS.iter().position(|&x| x == t).expect("a target of TARGETS");
				cross[row][col].push(len(now[p].p - was[p].p));
			}
		}
	}
	Ok(())
}

fn eval(args: &[String]) -> Result<(), String> {
	let (mut dir, mut mounts, mut split, mut crosstalk) = (None, None, false, false);
	// each file with the model named before it
	let mut files: Vec<(String, String)> = Vec::new();
	let mut i = 0;
	while i < args.len() {
		match args[i].as_str() {
			"--model" => dir = args.get(i + 1).cloned(),
			"--mounts" => mounts = args.get(i + 1).cloned(),
			"--split" => {
				split = true;
				i += 1;
				continue;
			}
			"--crosstalk" => {
				crosstalk = true;
				i += 1;
				continue;
			}
			_ => {
				files.push((dir.clone().ok_or("eval needs --model DIR before its files")?, args[i].clone()));
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
	let device = Device::default();
	let mut loaded: Option<(String, Norm, Net)> = None;
	// per part of features::TARGETS
	let mut pooled: Vec<(Vec<f64>, Vec<f64>)> = vec![(vec![], vec![]); 5];
	let mut jitter: Vec<(Vec<f64>, Vec<f64>)> = vec![(vec![], vec![]); 5];
	// per input device swapped (ALL_INPUTS), per target: how far it moved (m)
	let mut cross: Vec<Vec<Vec<f64>>> = vec![vec![Vec::new(); 5]; ALL_INPUTS.len()];
	for (n, (dir, f)) in files.iter().enumerate() {
		if loaded.as_ref().map(|l| &l.0) != Some(dir) {
			let d = PathBuf::from(dir);
			let norm = read_norm(&d.join("model.txt"))?;
			let model = load_net(&d, &norm, &device)?;
			loaded = Some((dir.clone(), norm, model));
		}
		let (_, norm, model) = loaded.as_ref().expect("loaded");
		let layout = &norm.layout;
		let (n_in, n_out) = (layout.n_in(), layout.n_out());
		// a BVH: the rig put on it as in training (without the free motions), seeded apart from training's
		let s = if f.ends_with(".bvh") {
			if mounts.is_empty() {
				return Err("a BVH needs --mounts FILE".into());
			}
			mocap_session(f, &mounts[n % mounts.len()], 1_000_000 + n as u64, false)?
		} else {
			poses::load(Path::new(f))?
		};
		let r = roles::assign(&s);
		if !r.notes.is_empty() && f.ends_with(".bvh") {
			eprintln!("{f}: left out: {}", r.notes.join("; "));
			continue;
		}
		let c = features::calibrate(&s, &r, layout)?;
		let end = s.devices[c.dev[0]].t.last().copied().unwrap_or(0.0);
		// every frame from 1 s after the T-pose
		let (mut xs, mut frames, mut times) = (Vec::new(), Vec::new(), Vec::new());
		let mut t = r.tpose_t + 1.0;
		let synthetic = f.ends_with(".bvh");
		while t < end {
			if synthetic && !tracking(&s, &c, t) {
				t += 1.0 / features::FPS;
				continue;
			}
			if let Some((mut x, frame)) = features::inputs(&s, &c, t) {
				normalize(&mut x, n_in, &norm.xm, &norm.xs);
				xs.extend(x);
				frames.push(frame);
				times.push(t);
			}
			t += 1.0 / features::FPS;
		}
		let n = frames.len();
		let mut outs = Vec::with_capacity(n * n_out);
		for start in (0..n).step_by(4096) {
			let k = (n - start).min(4096);
			let y = model.forward(tensor(&xs[start * n_in..(start + k) * n_in], k, n_in, &device));
			let v: Vec<f32> = y.into_data().try_to_vec::<f32>().map_err(|e| format!("{e:?}"))?;
			outs.extend(v);
		}
		if crosstalk {
			crosstalk_of(model, norm, &c, &frames, &times, &xs, &outs, &mut cross)?;
		}
		let nt = layout.targets.len();
		let mut part: Vec<(Vec<f64>, Vec<f64>)> = vec![(vec![], vec![]); nt];
		// each frame's error in the body's frame and in the tracker's, for --split
		let mut errs: Vec<Vec<(V3, Q4)>> = vec![vec![]; nt];
		// each frame's predicted and true place where the part counts, for the jitter
		let mut places: Vec<Vec<Option<(V3, V3)>>> = vec![vec![None; n]; nt];
		for (j, (frame, &t)) in frames.iter().zip(&times).enumerate() {
			let mut o: Vec<f32> = outs[j * n_out..(j + 1) * n_out].to_vec();
			for (c_, v) in o.iter_mut().enumerate() {
				*v = *v * norm.ys[c_] + norm.ym[c_];
			}
			let pred = features::decode(&c, frame, &o);
			let lit = |k: usize| {
				let d = &s.devices[c.dev[k]];
				let i = d.at(t + 1e-6);
				d.ok[i]
			};
			for p in 0..nt {
				if needs(layout, p).iter().all(|&k| lit(k)) {
					let d = &s.devices[c.target_dev(p)];
					let truth = d.pose[d.at(t + 1e-6)];
					part[p].0.push(len(pred[p].p - truth.p));
					part[p].1.push(qangle(qmul(qconj(pred[p].q), truth.q)));
					let cc = c.c[layout.inputs.len() + p];
					errs[p].push((
						qrot(qconj(frame.q), pred[p].p - truth.p),
						qmul(qmul(qconj(cc), qmul(qconj(truth.q), pred[p].q)), cc),
					));
					places[p][j] = Some((pred[p].p, truth.p));
				}
			}
		}
		let col = |p: usize| features::TARGETS.iter().position(|&x| x == layout.targets[p]).expect("a target");
		// the jitter: each place's second difference over three frames in a row (1/60 s apart), predicted and true
		for (p, pl) in places.iter().enumerate() {
			for j in 1..n.saturating_sub(1) {
				if (times[j + 1] - times[j - 1] - 2.0 / features::FPS).abs() > 1e-6 {
					continue;
				}
				if let (Some(a), Some(b), Some(c_)) = (pl[j - 1], pl[j], pl[j + 1]) {
					jitter[col(p)].0.push(len(a.0 + c_.0 - b.0 * 2.0));
					jitter[col(p)].1.push(len(a.1 + c_.1 - b.1 * 2.0));
				}
			}
		}
		let med = |v: &mut Vec<f64>| {
			v.sort_by(f64::total_cmp);
			if v.is_empty() {
				f64::NAN
			} else {
				v[v.len() / 2]
			}
		};
		let mut line = Vec::new();
		for (p, (a, b)) in part.iter_mut().enumerate() {
			line.push(format!("{} {:.0} mm {:.1} deg", layout.targets[p].name(), med(a) * 1e3, med(b).to_degrees()));
			pooled[col(p)].0.extend(a.iter());
			pooled[col(p)].1.extend(b.iter());
		}
		println!("{f}: {}", line.join("  "));
		if split {
			let parts: Vec<String> =
				errs.iter().enumerate().map(|(p, e)| split_line(layout.targets[p].name(), e)).collect();
			println!("  split: {}", parts.join("; "));
		}
	}
	println!("all pooled: part, frames, position median p90 mean (mm), orientation median p90 (deg)");
	for (p, (a, b)) in pooled.iter_mut().enumerate() {
		if a.is_empty() {
			continue;
		}
		a.sort_by(f64::total_cmp);
		b.sort_by(f64::total_cmp);
		let q = |v: &Vec<f64>, f: f64| if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * f) as usize] };
		let mean = a.iter().sum::<f64>() / a.len().max(1) as f64;
		println!(
			"  {:<12} {:>7} {:>6.0} {:>6.0} {:>6.0}   {:>6.1} {:>6.1}",
			features::TARGETS[p].name(),
			a.len(),
			q(a, 0.5) * 1e3,
			q(a, 0.9) * 1e3,
			mean * 1e3,
			q(b, 0.5).to_degrees(),
			q(b, 0.9).to_degrees()
		);
	}
	if crosstalk {
		println!(
			"crosstalk: how far each virtual tracker moves (median / p90, mm) when one device's last 0.5 s is swapped \
			 for its own motion 2 s later, the rest as it was"
		);
		println!(
			"  {:<12} {}",
			"swapped",
			features::TARGETS.iter().map(|p| format!("{:>13}", p.name())).collect::<Vec<_>>().join("")
		);
		for (d, row) in cross.iter_mut().enumerate() {
			if row.iter().all(|v| v.is_empty()) {
				continue;
			}
			let cells: Vec<String> = row
				.iter_mut()
				.map(|v| {
					v.sort_by(f64::total_cmp);
					let q = |f: f64| if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * f) as usize] * 1e3 };
					format!("{:>6.0} / {:<5.0}", q(0.5), q(0.9))
				})
				.collect();
			println!("  {:<12} {}", ALL_INPUTS[d].name(), cells.join(""));
		}
	}
	println!("the jitter: each place's change of velocity from one 60 Hz frame to the next, median and p90, predicted / true (mm)");
	for (p, (a, b)) in jitter.iter_mut().enumerate() {
		if a.is_empty() {
			continue;
		}
		a.sort_by(f64::total_cmp);
		b.sort_by(f64::total_cmp);
		let q =
			|v: &Vec<f64>, f: f64| if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * f) as usize] * 1e3 };
		println!(
			"  {:<12} {:>5.1} / {:>4.1}   {:>5.1} / {:>4.1}",
			features::TARGETS[p].name(),
			q(a, 0.5),
			q(b, 0.5),
			q(a, 0.9),
			q(b, 0.9)
		);
	}
	Ok(())
}

fn main() {
	let args: Vec<String> = std::env::args().collect();
	let rest = args.get(2..).unwrap_or(&[]);
	let r = match args.get(1).map(String::as_str) {
		Some("train") => train(rest),
		Some("tune") => tune(rest),
		Some("run") => match rest.iter().position(|a| a == "--replay") {
			// --replay FILE...: recordings through the live path
			Some(i) => {
				let dir = rest.iter().position(|a| a == "--model").and_then(|k| rest.get(k + 1));
				match dir {
					Some(d) => live::replay(Path::new(d), &rest[i + 1..]),
					None => Err("run --replay needs --model DIR first".into()),
				}
			}
			None => live::run(rest),
		},
		Some("eval") => eval(rest),
		Some("probe") => probe::probe(rest),
		_ => Err("usage: sally-model train --mounts FILE --out DIR [--epochs N] [--arch limbs|joint] [--hidden N] [--depth N] [--with-chest] [--free] BVH...\n       sally-model tune --model DIR --out DIR [--epochs N] [--rate R] [--mounts FILE] POSES_CSV... [BVH...]\n       sally-model run --model DIR [--port N]  |  run --model DIR --replay [--own-places] POSES_CSV...\n       sally-model eval [--mounts FILE] [--split] [--crosstalk] --model DIR POSES_CSV_OR_BVH... [--model DIR FILE...]...\n       sally-model probe [--mounts FILE] --model DIR POSES_CSV_OR_BVH... [--model DIR FILE...]...".into()),
	};
	if let Err(e) = r {
		eprintln!("{e}");
		std::process::exit(1);
	}
}
