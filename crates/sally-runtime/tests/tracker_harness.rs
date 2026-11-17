//! The tracker's harness on the Rust runtime (the C tracker harness's, core2_harness.c, which left with the C
//! runtime): the tracker on a synthetic scenario built to reach its rare paths (the cold start through the real
//! barycentric SVD, the accelerometer saturated, pinned at its range's end and not, and the gyro at full scale, a stuck
//! gyro axis, a blackout and the light's return, a wrong station with its coherent rejections, ties and disagreements,
//! NaN and out-of-range hits, a repeated IMU sample, the clock going back, a NaN sample that makes the state run away,
//! live setting changes), driven as the C harness drove it through the C runtime, with
//! its outputs held to the ones its header recorded: the C core's, which the Rust tracker matched in full before the C
//! core left (docs/checking.md). For a tracker and for a controller, the md5 of the recording without its first column
//! (the run time, the wall clock's) and its OPTION and LOG lines, and of the log without its "Recording to" line,
//! computed as the header said:
//!
//! awk '$2 != "OPTION" && $3 != "LOG" { $1 = ""; print }' rec | md5sum, and grep -v 'Recording to' log | md5sum
//!
//! The scenario is the C harness's to the bit: the same floating-point operations in the same order, through the same
//! libm (sin and cos are the C library's, called as the C calls them; where the C computes both of one argument gcc
//! calls glibc's sincos, which gives its sin and cos to the bit: checked on 20 million arguments over the scenario's
//! ranges), so the events are the C's. The tracker's outputs move only with a deliberate change to it; then these are
//! its new outputs. The md5s are the Linux build's (glibc's libm), so the test runs there only. When one differs, the
//! test leaves its log and recording in Cargo's temporary directory for the target and names them, to diff against the
//! same test's at a commit where it passes.
#![cfg(all(target_os = "linux", target_env = "gnu", target_arch = "x86_64"))]
// the port keeps the C's loops over indices (as the crate does)
#![allow(clippy::needless_range_loop)]

use std::f64::consts::PI;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use sally_lighthouse::reproject::{survive_reproject_axis_x_gen2, survive_reproject_axis_y_gen2};
use sally_lighthouse::BaseStationCal;
use sally_math::{
	apply_pose_to_point, c_cos, c_fmax, c_sin, cross3d, dot3d, invert_pose, normalize3d, quatfrommatrix33,
	quatrotatevector, sub3d, Pose,
};
use sally_runtime::config::{Config, SC_SETCONFIG};
use sally_runtime::log::{Log, LogLevel};
use sally_runtime::object::object_type;
use sally_runtime::{options, ObjectId, Runtime};

/// The md5s (their first 16 hex digits) of a tracker's (0) and a controller's (1) recording and log on the scenario. A
/// change that means to move them puts the new ones here and says why in its commit (docs/checking.md)
const WANT: [(&str, &str); 2] = [("d1474abe2a9eab91", "6c4d94af512d1a8a"), ("cea384c9523d3e4f", "a5d25c3eb3c56e0e")];

const NSENS: usize = 20;
const TB: f64 = 48000000.0;

/// ev_t: one event of the scenario, fed in order
#[derive(Clone, Copy, Debug)]
enum Ev {
	/// kind 0: an IMU sample (g, rad/s) at its device time
	Imu { tc: u64, accel: [f64; 3], gyro: [f64; 3] },
	/// kind 1: a sweep hit
	Hit { tc: u64, lh: i32, plane: i32, sensor: i32, angle: f64 },
	/// kind 2: an option set live (survive_config_set_live)
	Set { tag: &'static str, val: &'static str },
}

/// (uint64_t)x as the C build converts it: gcc on x86-64 takes cvttsd2si below 2^63, so a negative time wraps (the
/// first sweeps' hits, at -4 ms, are 2^64 - 192000 ticks), where Rust's `as u64` would give 0. Every time here is
/// below 2^63 ticks (and none is NaN)
fn c_u64(x: f64) -> u64 {
	assert!(x < 9223372036854775808.0, "{x}: not below 2^63");
	x as i64 as u64
}

fn qmul(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
	[
		a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
		a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
		a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
		a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
	]
}

fn qexpv(v: &[f64; 3]) -> [f64; 4] {
	let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
	if n < 1e-15 {
		return [1.0, 0.0, 0.0, 0.0];
	}
	let mut q = [c_cos(n / 2.0), 0.0, 0.0, 0.0];
	for i in 0..3 {
		q[1 + i] = v[i] * c_sin(n / 2.0) / n;
	}
	q
}

/// the true motion: a body rate w_b (piecewise constant, integrated) and a smooth position
fn wb_of(_t: f64) -> f64 {
	0.0
}

/// truth(t, p, q, acc_w): the position, the orientation and the world acceleration at t
fn truth(t: f64) -> ([f64; 3], [f64; 4], [f64; 3]) {
	let a = [0.3, 0.2, 0.1];
	let w = [1.3, 0.9, 1.7];
	let mut p = [0.0; 3];
	let mut acc_w = [0.0; 3];
	for i in 0..3 {
		p[i] = (if i == 2 { 1.0 } else { 0.2 * i as f64 }) + a[i] * c_sin(w[i] * t);
		acc_w[i] = -a[i] * w[i] * w[i] * c_sin(w[i] * t);
	}
	let rv = [0.4 * c_sin(0.7 * t), 0.3 * c_sin(0.5 * t + 1.0), 0.8 * t + wb_of(t)];
	(p, qexpv(&rv), acc_w)
}

/// The device's sensors and the two stations (main's setup), what gen reads
struct Scenario {
	sens: [[f64; 3]; NSENS],
	norms: [[f64; 3]; NSENS],
	lh2world: [Pose; 2],
	fcal: [[BaseStationCal; 2]; 2],
}

impl Scenario {
	fn new() -> Scenario {
		let mut sens = [[0.0; 3]; NSENS];
		let mut norms = [[0.0; 3]; NSENS];
		// a cap of sensors, 6 cm, normals outward, more on the +z side
		for i in 0..NSENS {
			let th = i as f64 * 2.399963;
			let z = 1.0 - (i as f64 + 0.5) / (NSENS as f64 * 0.75);
			let r = c_fmax(0.0, 1.0 - z * z).sqrt();
			norms[i] = [r * c_cos(th), r * c_sin(th), z];
			for k in 0..3 {
				sens[i][k] = norms[i][k] * 0.06 + if k == 0 { 0.01 } else { 0.0 };
			}
		}
		let lpos = [[2.1, 1.8, 2.5], [-1.9, -1.6, 2.4]];
		let mut lh2world = [Pose::default(); 2];
		let mut fcal = [[BaseStationCal::default(); 2]; 2];
		for l in 0..2 {
			let up = [0.0, 0.0, 1.0];
			let z = normalize3d(&lpos[l]);
			let x = normalize3d(&cross3d(&up, &z));
			let y = cross3d(&z, &x);
			let m = [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
			lh2world[l] = Pose { pos: lpos[l], rot: quatfrommatrix33(&m) };
			for a in 0..2 {
				fcal[l][a] = BaseStationCal {
					phase: 0.01 * (a + 1) as f64 * if l != 0 { -1.0 } else { 1.0 },
					tilt: 0.005 * if a != 0 { -1.0 } else { 1.0 },
					curve: 0.002 * (l + 1) as f64,
					gibpha: 1.1 + a as f64,
					gibmag: 0.003,
					ogeephase: 0.7 + l as f64,
					ogeemag: 0.001 * (a + 1) as f64,
				};
			}
		}
		Scenario { sens, norms, lh2world, fcal }
	}

	/// gen: every event, a step every 4 ms for 16 s
	fn gen(&self) -> Vec<Ev> {
		let mut evs = Vec::new();
		let hit_shift = 0.0045;
		let mut step = 0;
		let mut ti = 0.004;
		while ti < 16.0 {
			self.step(&mut evs, ti, step, hit_shift);
			ti += 0.004;
			step += 1;
		}
		evs
	}

	/// One step of gen's loop (its `continue` is this function's return)
	fn step(&self, evs: &mut Vec<Ev>, ti: f64, step: i32, hit_shift: f64) {
		// the hits of the last step (they arrive before this IMU sample), each station one plane, alternating
		let blackout = ti > 6.0 && ti < 8.2;
		let wrong = ti > 9.0 && ti < 10.0;
		for lh in 0..2 {
			if blackout {
				break;
			}
			let plane = (step + lh as i32) % 2;
			let mut l2w = self.lh2world[lh];
			if wrong && lh == 1 {
				l2w.pos[0] += 0.3;
			}
			let w2l = invert_pose(&l2w);
			for s in 0..NSENS {
				let th = ti - 0.0035 + s as f64 * 2e-5;
				let (p, q, _) = truth(th);
				let r_s = quatrotatevector(&q, &self.sens[s]);
				let nw = quatrotatevector(&q, &self.norms[s]);
				let mut pw = [0.0; 3];
				for i in 0..3 {
					pw[i] = p[i] + r_s[i];
				}
				let dir = normalize3d(&sub3d(&self.lh2world[lh].pos, &pw));
				if dot3d(&dir, &nw) < 0.05 {
					continue;
				}
				let pl = apply_pose_to_point(&w2l, &pw);
				let ang = if plane == 0 {
					survive_reproject_axis_x_gen2(&self.fcal[lh], &pl)
				} else {
					survive_reproject_axis_y_gen2(&self.fcal[lh], &pl)
				};
				let t = th - hit_shift;
				let (mut e_lh, mut e_plane, mut sensor, mut angle) = (lh as i32, plane, s as i32, ang);
				if ti > 10.5 && ti < 10.6 && s == 3 {
					angle = f64::NAN;
				}
				if ti > 10.6 && ti < 10.7 && s == 4 {
					sensor = 40;
				}
				if ti > 10.7 && ti < 10.8 && s == 5 {
					e_lh = 7;
				}
				if ti > 10.8 && ti < 10.9 && s == 6 {
					e_plane = 3;
				}
				if ti > 10.9 && ti < 11.0 && s == 7 {
					angle += 0.02;
				}
				evs.push(Ev::Hit { tc: c_u64(t * TB), lh: e_lh, plane: e_plane, sensor, angle });
			}
		}
		// the IMU sample (the C calls truth(ti) again after truth(ti + 1e-5), for the same p, q and aw)
		let (_, q, aw) = truth(ti);
		let (_, qn, _) = truth(ti + 1e-5);
		// body rate from the orientation's derivative: w_b = 2 vec(q^-1 dq/dt)
		let qc = [q[0], -q[1], -q[2], -q[3]];
		let mut dq = [0.0; 4];
		for i in 0..4 {
			dq[i] = (qn[i] - q[i]) / 1e-5;
		}
		let wq = qmul(&qc, &dq);
		let f_w = [aw[0], aw[1], aw[2] + 9.80665];
		let f_b = quatrotatevector(&qc, &f_w);
		let mut accel = [0.0; 3];
		let mut gyro = [0.0; 3];
		for i in 0..3 {
			accel[i] = f_b[i] / 9.80665;
			gyro[i] = 2.0 * wq[1 + i];
		}
		if ti > 4.0 && ti < 4.03 {
			accel[0] = 32767.0 / 4096.0; // pinned at the end of the range
		}
		if ti > 4.1 && ti < 4.12 {
			accel[1] = -7.9; // beyond CLIP_G, not pinned
		}
		if ti > 4.3 && ti < 4.33 {
			gyro[2] = 36.0; // gyro at full scale
		}
		if ti > 5.0 && ti < 5.5 {
			// a stuck axis while the others swing (PI is M_PI)
			gyro[2] = 0.032;
			gyro[0] = 8.0 * c_sin(2.0 * PI * 5.0 * ti);
			gyro[1] = 8.0 * c_cos(2.0 * PI * 5.0 * ti);
		}
		let e = evs.len();
		evs.push(Ev::Imu { tc: c_u64(ti * TB), accel, gyro });
		// an IMU gap of 0.2 s: the skip comes after the sample is pushed, so the samples are fed, and only what follows
		// is skipped (none of which falls in the window)
		if ti > 11.5 && ti < 11.7 {
			return;
		}
		if ti > 12.0 && ti < 12.02 {
			// a repeated sample
			evs.push(evs[e]);
		}
		if let Ev::Imu { tc, accel, .. } = &mut evs[e] {
			if ti > 12.5 && ti < 12.504 {
				accel[2] = f64::NAN; // a NaN sample: the state runs away
			}
			if ti > 14.0 {
				*tc = c_u64((ti - 2.0) * TB); // the clock goes back 2 s
			}
		}
		let set = |tag, val| Ev::Set { tag, val };
		match step {
			400 => evs.push(set("track-gate", "6")),
			125 => evs.push(set("track-blink", "0.5")),
			900 => evs.push(set("track-blink", "-0.3")),
			1450 => evs.push(set("track-blink", "0")),
			2100 => evs.push(set("track-probe-ch", "9")),
			2200 => evs.push(set("track-probe-ch", "-1")),
			2300 => evs.push(set("track-only", "128")),
			2450 => evs.push(set("track-only", "0")),
			2600 => evs.push(set("track-zupt-sd", "0")),
			_ => {}
		}
	}
}

/// main's command line
const ARGV: [&str; 11] = [
	"harness",
	"--htcvive",
	"0",
	"--track-verbose",
	"3",
	"--track-gravity-tilt",
	"0.0108",
	"--track-gravity-tilt-az",
	"303.7",
	"--track-gravity-tilt-sd",
	"0.003",
];

/// core2_harness <controller> <log> <rec>: the tracker live on the scenario, its log lines and recording to the files
fn harness(controller: bool, log_path: &Path, rec_path: &Path) {
	// survive_init_with_logger(11, av, 0, logfn): the options registered (survive-cli's, which has one the harness's
	// library lacks, stop-after: one more OPTION line), the command line, the log level. The C also reads the
	// SURVIVE_* environment and ./config.json, which the harness's runs have neither of: the test reads neither
	let mut config = Config::default();
	options::register(&mut config);
	config.apply_args(&ARGV);
	config.configi("v", SC_SETCONFIG, 0);
	let mut rt = Runtime::new(config, Log::new(None, false));
	rt.load_lighthouses();
	// logfn: "%d %s\n", the level and the line, into the log file once it is open (nothing before)
	let logf: Arc<Mutex<Option<BufWriter<File>>>> = Arc::new(Mutex::new(None));
	let sink = Arc::clone(&logf);
	rt.install_log_fn(Some(Box::new(move |ll: LogLevel, msg: &str| {
		if let Some(f) = sink.lock().unwrap().as_mut() {
			writeln!(f, "{} {}", ll as i32, msg).unwrap();
		}
	})));

	let sc = Scenario::new();
	rt.lh_version = 1;
	rt.active_lighthouses = 2;
	for l in 0..2 {
		let b = &mut rt.bsd[l];
		b.position_set = true;
		b.pose = sc.lh2world[l];
		b.fcal = sc.fcal[l];
		b.mode = if l != 0 { 9 } else { 7 };
	}
	let evs = sc.gen();
	let so = device(&mut rt, &sc, controller);

	*logf.lock().unwrap() = Some(BufWriter::new(File::create(log_path).unwrap()));
	assert_eq!(rt.recording_start(rec_path.to_str().unwrap()), 0);
	rt.create_tracker(so);
	for e in &evs {
		match *e {
			// the C ignores what the set returns: the variables bound to the tag, here the tracker's one
			Ev::Set { tag, val } => assert_eq!(rt.set_live(tag, val), 1),
			Ev::Imu { tc, accel, gyro } => rt.tracker_imu(so, tc, &accel, &gyro),
			Ev::Hit { tc, lh, plane, sensor, angle } => rt.tracker_hit(so, tc, lh, plane, sensor, angle),
		}
	}
	rt.free_tracker(so);
	assert_eq!(rt.recording_stop(), 0);
	logf.lock().unwrap().take().unwrap().flush().unwrap();
}

/// device(): the harness's SurviveObject, calloc'd and never added to the context. Made here as survive_create_device
/// and survive_add_object make a device, then given the harness's fields, and no tracker until survive_c2_create.
/// What that does beyond the C's shows nowhere: its "Adding tracked object" line goes to no file (the log file is not
/// open yet, as the C's is not), its being listed would show only in the recording's CONFIG lines (it has no config),
/// and its defaults differ from calloc's zeros only in what nothing on this path reads (the IMU rate, the gyro's
/// calibration, the reference poses, the sensor scale, the light filter); head2imu, which the imupose process applies,
/// is zero in both
fn device(rt: &mut Runtime, sc: &Scenario, controller: bool) -> ObjectId {
	let obj = rt.create_object("", if controller { "WM0" } else { "TR0" }, "", "").unwrap();
	let so = &mut rt.objects[obj];
	so.tracker = None;
	so.object_type = if controller { object_type::CONTROLLER } else { object_type::OTHER };
	so.timebase_hz = 48000000;
	so.sensor_ct = NSENS as i8;
	so.sensor_locations = Some(sc.sens.iter().flatten().copied().collect());
	so.sensor_normals = Some(sc.norms.iter().flatten().copied().collect());
	so.raw_acc_scale = 1.0 / 4096.0;
	so.raw_gyro_scale = 0.00106526;
	so.acc_scale = [1.0; 3];
	so.acc_bias = [0.0; 3];
	assert_eq!(so.head2imu, Pose { pos: [0.0; 3], rot: [0.0; 4] });
	obj
}

/// The recording as the header digests it: awk '$2 != "OPTION" && $3 != "LOG" { $1 = ""; print }'. awk's records are
/// the lines, its fields split at runs of spaces and tabs (a "\r" stays in its field); emptying the first rebuilds
/// the line from the fields, joined by single spaces
fn digest_recording(rec: &[u8]) -> Vec<u8> {
	let mut out = Vec::with_capacity(rec.len());
	for line in lines(rec) {
		let fields: Vec<&[u8]> = line.split(|&c| c == b' ' || c == b'\t').filter(|f| !f.is_empty()).collect();
		let field = |i: usize| fields.get(i).copied().unwrap_or(b"");
		if field(1) == b"OPTION" || field(2) == b"LOG" {
			continue;
		}
		for f in fields.iter().skip(1) {
			out.push(b' ');
			out.extend_from_slice(f);
		}
		out.push(b'\n');
	}
	out
}

/// The log as the header digests it: grep -v 'Recording to'
fn digest_log(log: &[u8]) -> Vec<u8> {
	const DROPPED: &[u8] = b"Recording to";
	let mut out = Vec::with_capacity(log.len());
	for line in lines(log) {
		if !line.windows(DROPPED.len()).any(|w| w == DROPPED) {
			out.extend_from_slice(line);
			out.push(b'\n');
		}
	}
	out
}

/// The lines of a text as awk and grep read them, without their "\n" (a last line without one is a line too)
fn lines(text: &[u8]) -> Vec<&[u8]> {
	let mut lines: Vec<&[u8]> = text.split(|&c| c == b'\n').collect();
	// what follows a last "\n" is no line (nor is an empty text)
	if lines.last().is_some_and(|l| l.is_empty()) {
		lines.pop();
	}
	lines
}

/// md5sum | cut -c1-16
fn md5_16(data: &[u8]) -> String {
	md5(data)[..8].iter().map(|b| format!("{b:02x}")).collect()
}

/// MD5 (RFC 1321)
fn md5(data: &[u8]) -> [u8; 16] {
	const S: [u32; 64] = [
		7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14,
		20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6,
		10, 15, 21,
	];
	const K: [u32; 64] = [
		0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501, 0x698098d8,
		0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340,
		0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87,
		0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
		0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039,
		0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92,
		0xffeff47d, 0x85845dd1, 0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
		0xeb86d391,
	];
	let mut msg = data.to_vec();
	msg.push(0x80);
	while msg.len() % 64 != 56 {
		msg.push(0);
	}
	msg.extend_from_slice(&((data.len() as u64).wrapping_mul(8)).to_le_bytes());
	let mut h: [u32; 4] = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476];
	for chunk in msg.chunks_exact(64) {
		let m: Vec<u32> = chunk.chunks_exact(4).map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]])).collect();
		let [mut a, mut b, mut c, mut d] = h;
		for i in 0..64 {
			let (f, g) = match i / 16 {
				0 => ((b & c) | (!b & d), i),
				1 => ((d & b) | (!d & c), (5 * i + 1) % 16),
				2 => (b ^ c ^ d, (3 * i + 5) % 16),
				_ => (c ^ (b | !d), (7 * i) % 16),
			};
			let f = f.wrapping_add(a).wrapping_add(K[i]).wrapping_add(m[g]);
			a = d;
			d = c;
			c = b;
			b = b.wrapping_add(f.rotate_left(S[i]));
		}
		h = [h[0].wrapping_add(a), h[1].wrapping_add(b), h[2].wrapping_add(c), h[3].wrapping_add(d)];
	}
	let mut out = [0u8; 16];
	for (i, v) in h.iter().enumerate() {
		out[4 * i..4 * i + 4].copy_from_slice(&v.to_le_bytes());
	}
	out
}

/// The harness for one device, its two md5s against the C's; the outputs are kept when they differ
fn check(controller: bool) {
	let c = controller as usize;
	let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
	let (log_path, rec_path) =
		(dir.join(format!("tracker_harness_log{c}.txt")), dir.join(format!("tracker_harness_rec{c}.txt")));
	harness(controller, &log_path, &rec_path);
	let rec = md5_16(&digest_recording(&std::fs::read(&rec_path).unwrap()));
	let log = md5_16(&digest_log(&std::fs::read(&log_path).unwrap()));
	let (want_rec, want_log) = WANT[c];
	assert!(
		rec == want_rec && log == want_log,
		"harness {c}: recording {rec} log {log}, the C's {want_rec} {want_log}; the outputs are {} and {}",
		rec_path.display(),
		log_path.display()
	);
	std::fs::remove_file(&rec_path).unwrap();
	std::fs::remove_file(&log_path).unwrap();
}

#[test]
fn a_tracker_gives_the_c_cores_outputs() {
	check(false);
}

#[test]
fn a_controller_gives_the_c_cores_outputs() {
	check(true);
}

#[test]
fn md5_gives_the_known_digests() {
	assert_eq!(md5_16(b""), "d41d8cd98f00b204");
	assert_eq!(md5_16(b"The quick brown fox jumps over the lazy dog"), "9e107d9d372bb682");
	// a message over one block, with its length's padding in a block of its own
	assert_eq!(md5_16(&[b'a'; 64]), "014842d480b57149");
}
