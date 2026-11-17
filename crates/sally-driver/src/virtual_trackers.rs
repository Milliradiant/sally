//! The virtual trackers (docs/sally_driver.md "The virtual trackers", docs/mocap_plan.md section 5): with
//! `driver.virtual`, five devices of the driver's own, SALLY-CHEST, -LKNEE, -RKNEE, -LELBOW and -RELBOW, whose poses
//! the model process (`sally-model run`) works out from the six devices of ordinary full-body tracking (headset, hands,
//! waist, feet). Each is a tracker in the headset's tracking system (Space Calibrator moves only the devices of its
//! target system, so it leaves them where they are), with Valve's tracker render model, input profile and icons, handed
//! to SteamVR in its raw universe with the identity world-from-driver, out of range while it has no fresh pose.
//!
//! Every frame (RunFrame) the driver sends the process what SteamVR has for each device (GetRawTrackedDevicePoses, the
//! raw universe the virtual trackers are handed back in): `F <t>`, then `R <t> <class> <serial> <valid> x y z qw qx qy
//! qz` per connected headset, controller and tracker, ours left out. The process answers each frame with `T` lines (the
//! five poses), and every second with an `S` line (what it is doing: looking for the T-pose, tracking); the receive
//! thread hands each T line's pose to SteamVR as it comes. `virtual on` / `off` / `toggle` / `calibrate` on cmd_port
//! (the panel's switch): off reports the five disconnected, `calibrate` has the process look for the T-pose again.
//! `virtual` alone answers with the X line (`virtual_line`), as every one of them does before the status line.
//!
//! `driver.virtual_trackers` names the ones SteamVR is given (all five when it is empty): the others are never added,
//! and their T lines are dropped (the chest left to a real tracker, say).
//!
//! The switch is kept: `virtual on` / `off` write sally_virtual.txt beside the driver (`on=1` / `on=0`), read when
//! SteamVR starts the driver; without the file they are off. Off at the start: none of them is added and the model
//! process is not started, so SteamVR has no virtual tracker at all. Switched on: the process started and the ones
//! shown added at the next frame. Switched off once they are added: reported disconnected and the process stopped
//! (vrserver has no way to take a device away again), gone at SteamVR's next start.
//!
//! The process is started when they are on and `driver.virtual_exe` is set (`run --model <virtual_model> --port
//! <virtual_port>` in its own folder), started again when it exits (at most every 5 s), and killed when SteamVR exits
//! or they are switched off: it keeps nothing.

use std::collections::BTreeMap;
use std::ffi::{c_char, c_void, CStr};
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, Ordering::SeqCst};
use std::sync::{Mutex, MutexGuard};
use std::thread::JoinHandle;

use sally_math::math3d::{qfrom_mat, qnorm, Q4};
use sally_math::printf::{bsprintf, BArg};

use crate::helpers::{expand_time, is_absolute, lines, log, now_seconds, split, time_stamp, K_SEP};
use crate::net::{self, SockAddrIn, SOCKADDR_IN_LEN, SOCK_BAD};
use crate::openvr::*;
use crate::runtime_process::RuntimeProcess;
use crate::settings::g_opt;

/// The five, in sally-model's order (sally_body::features::TARGETS): its name for each, the serial, the model number
/// and the controller type (where SteamVR keeps a tracker's role; VRChat assigns trackers by where they are)
pub const TRACKERS: [(&str, &CStr, &str, &str); 5] = [
	("chest", c"SALLY-CHEST", "sally virtual chest", "vive_tracker_chest"),
	("lknee", c"SALLY-LKNEE", "sally virtual left knee", "vive_tracker_left_knee"),
	("rknee", c"SALLY-RKNEE", "sally virtual right knee", "vive_tracker_right_knee"),
	("lelbow", c"SALLY-LELBOW", "sally virtual left elbow", "vive_tracker_left_elbow"),
	("relbow", c"SALLY-RELBOW", "sally virtual right elbow", "vive_tracker_right_elbow"),
];

/// Which of the five `driver.virtual_trackers` names (empty: all five); the names it has that are none of them
pub fn shown(list: &[u8]) -> ([bool; 5], Vec<Vec<u8>>) {
	if list.iter().all(|c| c.is_ascii_whitespace()) {
		return ([true; 5], Vec::new());
	}
	let mut on = [false; 5];
	let mut unknown = Vec::new();
	for name in list.split(|&c| c == b',' || c.is_ascii_whitespace()).filter(|w| !w.is_empty()) {
		match TRACKERS.iter().position(|t| t.0.as_bytes() == name) {
			Some(k) => on[k] = true,
			None => unknown.push(name.to_vec()),
		}
	}
	(on, unknown)
}

/// The file beside the driver that keeps the switch
pub const SWITCH_FILE: &[u8] = b"sally_virtual.txt";

/// The switch as kept: on only for an `on=1` line (no file, or anything else: off)
pub fn parse_switch(text: &[u8]) -> bool {
	let mut on = false;
	for l in lines(text) {
		let l: Vec<u8> = l.iter().copied().filter(|c| !c.is_ascii_whitespace()).collect();
		if let Some(v) = l.strip_prefix(b"on=") {
			on = v == b"1";
		}
	}
	on
}

fn read_switch(path: &[u8]) -> bool {
	let mut text = Vec::new();
	if let Some(mut f) = crate::cstdio::CFile::open(path, c"rb") {
		let mut line = [0u8; 256];
		while let Some(l) = f.gets(&mut line) {
			text.extend_from_slice(l);
		}
		f.close();
	}
	parse_switch(&text)
}

fn write_switch(path: &[u8], on: bool) {
	if path.is_empty() {
		return;
	}
	match crate::cstdio::CFile::open(path, c"wb") {
		Some(mut f) => {
			f.write(if on {
				b"# sally's virtual trackers, kept by the panel's switch (off without this file)\non=1\n"
			} else {
				b"# sally's virtual trackers, kept by the panel's switch (off without this file)\non=0\n"
			});
			f.close();
		}
		None => log("sally: virtual trackers: cannot write %s\n", &[BArg::S(path)]),
	}
}

/// A pose older than this is none: the tracker goes out of range, s
const STALE_S: f64 = 0.25;
/// No S line for this long: the process is not running, s
const QUIET_S: f64 = 3.0;
/// A device whose class or serial did not read is read again after this, s
const KNOWN_RETRY_S: f64 = 1.0;
/// The pose's age as declared to SteamVR is at most this, s
const MAX_AGE_S: f64 = 0.05;

/// One of the five as vrserver holds it: an ITrackedDeviceServerDriver (its vtable first) and which one it is
#[repr(C)]
pub struct VirtualDevice {
	vtbl: *const ITrackedDeviceServerDriverVtbl,
	k: usize,
}
// SAFETY: a static vtable pointer and an index, never written
unsafe impl Sync for VirtualDevice {}

static DEVICES: [VirtualDevice; 5] = [
	VirtualDevice { vtbl: &DEVICE_VTBL, k: 0 },
	VirtualDevice { vtbl: &DEVICE_VTBL, k: 1 },
	VirtualDevice { vtbl: &DEVICE_VTBL, k: 2 },
	VirtualDevice { vtbl: &DEVICE_VTBL, k: 3 },
	VirtualDevice { vtbl: &DEVICE_VTBL, k: 4 },
];

/// What each of the five has (under one lock)
#[derive(Default)]
struct State {
	/// the last pose SteamVR was handed, and when a fresh one last came
	pose: [DriverPose_t; 5],
	fresh: [f64; 5],
	/// when SteamVR was last handed a pose not tracking (out of range, or disconnected), so it is said again every
	/// second, not every frame
	idle_sent: [f64; 5],
	/// the process's last S line (its words after "S") and when it came
	status: Vec<u8>,
	status_t: f64,
	/// each SteamVR device's class and serial, by id (read once); for one whose read failed, when it is read again
	known: BTreeMap<u32, Result<(i32, Vec<u8>), f64>>,
	/// the process the driver started, and when
	proc: RuntimeProcess,
	last_start: f64,
	/// sally_virtual.txt, where the switch is kept
	switch_path: Vec<u8>,
}

pub struct VirtualTrackers {
	/// driver.virtual: the five can be had (their pose the process's)
	enabled: AtomicBool,
	/// the switch (kept in sally_virtual.txt, off without it): on adds them and runs the process, off reports the ones
	/// added disconnected and stops the process
	on: AtomicBool,
	/// driver.virtual_trackers: the ones SteamVR is given
	shown: [AtomicBool; 5],
	added: AtomicBool,
	/// vrserver's ids for the five (k_unTrackedDeviceIndexInvalid until Activate)
	ids: [AtomicU32; 5],
	sock: AtomicI64,
	running: AtomicBool,
	thread: Mutex<Option<JoinHandle<()>>>,
	/// the driver's folder
	dir: Mutex<Vec<u8>>,
	/// the frames sent, and the T datagrams taken (each a double's bits for when)
	frames: AtomicU64,
	answers: AtomicU64,
	state: Mutex<State>,
}

pub static G_VIRTUAL: VirtualTrackers = VirtualTrackers::new();

fn locked<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
	m.lock().unwrap_or_else(|e| e.into_inner())
}

/// The identity world-from-driver and driver-from-head, out of range, connected
fn idle_pose() -> DriverPose_t {
	let id = HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
	DriverPose_t {
		qWorldFromDriverRotation: id,
		qDriverFromHeadRotation: id,
		qRotation: id,
		result: TrackingResult_Running_OutOfRange,
		deviceIsConnected: 1,
		..Default::default()
	}
}

/// A T line's pose: `T <name> <valid> x y z qw qx qy qz vx vy vz wx wy wz`; the index of the tracker it is for
pub fn parse_t(line: &[u8], age: f64) -> Option<(usize, DriverPose_t)> {
	let f = split(line);
	if f.len() != 16 || f[0] != b"T" {
		return None;
	}
	let k = TRACKERS.iter().position(|t| t.0.as_bytes() == f[1])?;
	let mut v = [0.0f64; 13];
	for (i, x) in v.iter_mut().enumerate() {
		*x = std::str::from_utf8(f[3 + i]).ok()?.parse().ok()?;
	}
	if !v.iter().all(|x| x.is_finite()) {
		return None;
	}
	let mut p = idle_pose();
	if f[2] != b"1" {
		return Some((k, p));
	}
	let q = qnorm(Q4::new(v[3], v[4], v[5], v[6]));
	p.vecPosition = [v[0], v[1], v[2]];
	p.qRotation = HmdQuaternion_t { w: q.w, x: q.x, y: q.y, z: q.z };
	p.vecVelocity = [v[7], v[8], v[9]];
	p.vecAngularVelocity = [v[10], v[11], v[12]];
	p.poseTimeOffset = -age.clamp(0.0, MAX_AGE_S);
	p.result = TrackingResult_Running_OK;
	p.poseIsValid = 1;
	Some((k, p))
}

/// An R line for a device SteamVR has: its class by name, serial, whether it tracks, its pose (position, rotation)
pub fn r_line(t: f64, class: i32, serial: &[u8], p: &TrackedDevicePose_t) -> Option<Vec<u8>> {
	let name: &[u8] = match class {
		c if c == TrackedDeviceClass_HMD => b"hmd",
		c if c == TrackedDeviceClass_Controller => b"controller",
		c if c == TrackedDeviceClass_GenericTracker => b"tracker",
		_ => return None,
	};
	let m = &p.mDeviceToAbsoluteTracking.m;
	let r = [
		[m[0][0] as f64, m[0][1] as f64, m[0][2] as f64],
		[m[1][0] as f64, m[1][1] as f64, m[1][2] as f64],
		[m[2][0] as f64, m[2][1] as f64, m[2][2] as f64],
	];
	let q = qfrom_mat(&r);
	let valid = p.bPoseIsValid != 0 && p.eTrackingResult == TrackingResult_Running_OK;
	Some(bsprintf(
		"R %.4f %s %s %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f\n",
		&[
			BArg::F(t),
			BArg::S(name),
			BArg::S(serial),
			BArg::I(valid as i64),
			BArg::F(m[0][3] as f64),
			BArg::F(m[1][3] as f64),
			BArg::F(m[2][3] as f64),
			BArg::F(q.w),
			BArg::F(q.x),
			BArg::F(q.y),
			BArg::F(q.z),
		],
	))
}

impl VirtualTrackers {
	const fn new() -> VirtualTrackers {
		VirtualTrackers {
			enabled: AtomicBool::new(false),
			on: AtomicBool::new(false),
			shown: [
				AtomicBool::new(true),
				AtomicBool::new(true),
				AtomicBool::new(true),
				AtomicBool::new(true),
				AtomicBool::new(true),
			],
			added: AtomicBool::new(false),
			ids: [
				AtomicU32::new(k_unTrackedDeviceIndexInvalid),
				AtomicU32::new(k_unTrackedDeviceIndexInvalid),
				AtomicU32::new(k_unTrackedDeviceIndexInvalid),
				AtomicU32::new(k_unTrackedDeviceIndexInvalid),
				AtomicU32::new(k_unTrackedDeviceIndexInvalid),
			],
			sock: AtomicI64::new(SOCK_BAD),
			running: AtomicBool::new(false),
			thread: Mutex::new(None),
			dir: Mutex::new(Vec::new()),
			frames: AtomicU64::new(0),
			answers: AtomicU64::new(0),
			state: Mutex::new(State {
				pose: [DriverPose_t {
					poseTimeOffset: 0.0,
					qWorldFromDriverRotation: HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
					vecWorldFromDriverTranslation: [0.0; 3],
					qDriverFromHeadRotation: HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
					vecDriverFromHeadTranslation: [0.0; 3],
					vecPosition: [0.0; 3],
					vecVelocity: [0.0; 3],
					vecAcceleration: [0.0; 3],
					qRotation: HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
					vecAngularVelocity: [0.0; 3],
					vecAngularAcceleration: [0.0; 3],
					result: TrackingResult_Running_OutOfRange,
					poseIsValid: 0,
					willDriftInYaw: 0,
					shouldApplyHeadModel: 0,
					deviceIsConnected: 1,
				}; 5],
				fresh: [f64::NEG_INFINITY; 5],
				idle_sent: [f64::NEG_INFINITY; 5],
				status: Vec::new(),
				status_t: f64::NEG_INFINITY,
				known: BTreeMap::new(),
				proc: RuntimeProcess::NONE,
				last_start: f64::NEG_INFINITY,
				switch_path: Vec::new(),
			}),
		}
	}

	pub fn enabled(&self) -> bool {
		self.enabled.load(SeqCst)
	}

	/// vrserver's id for one of the five (k_unTrackedDeviceIndexInvalid before it is activated)
	pub fn id(&self, k: usize) -> u32 {
		self.ids[k].load(SeqCst)
	}

	/// Whether SteamVR is given this one (driver.virtual_trackers)
	pub fn is_shown(&self, k: usize) -> bool {
		self.shown[k].load(SeqCst)
	}

	/// The names of the ones SteamVR is given, commas between
	fn shown_names(&self) -> Vec<u8> {
		let names: Vec<&[u8]> = (0..5).filter(|&k| self.is_shown(k)).map(|k| TRACKERS[k].0.as_bytes()).collect();
		names.join(&b","[..])
	}

	fn is_ours(&self, id: u32) -> bool {
		self.ids.iter().any(|x| x.load(SeqCst) == id)
	}

	/// Init: with driver.virtual, the switch read (sally_virtual.txt), the socket to the model process, its receive
	/// thread, and the process started when they are on
	pub fn init(&'static self, dir: &[u8]) {
		let o = g_opt();
		self.enabled.store(o.virtual_on, SeqCst);
		if !o.virtual_on {
			return;
		}
		*locked(&self.dir) = dir.to_vec();
		let (shown, unknown) = shown(&o.virtual_trackers);
		for (k, x) in shown.iter().enumerate() {
			self.shown[k].store(*x, SeqCst);
		}
		for name in &unknown {
			log(
				"sally: virtual trackers: driver.virtual_trackers has %s: not chest, lknee, rknee, lelbow or relbow\n",
				&[BArg::S(name)],
			);
		}
		let sock = net::socket_udp();
		if sock == SOCK_BAD {
			log("sally: virtual trackers: cannot create a UDP socket\n", &[]);
			self.enabled.store(false, SeqCst);
			return;
		}
		net::bind(sock, &SockAddrIn::loopback(0));
		net::set_rcvbuf(sock, 1 << 20);
		net::set_recv_timeout_50ms(sock);
		self.sock.store(sock, SeqCst);
		let mut path = dir.to_vec();
		path.push(K_SEP);
		path.extend_from_slice(SWITCH_FILE);
		let on = read_switch(&path);
		self.on.store(on, SeqCst);
		let serials: Vec<&[u8]> = (0..5).filter(|&k| self.is_shown(k)).map(|k| TRACKERS[k].1.to_bytes()).collect();
		let serials = if serials.is_empty() { b"none".to_vec() } else { serials.join(&b", "[..]) };
		if on {
			log(
				"sally: virtual trackers on: %s from the model process on port %d\n",
				&[BArg::S(&serials), BArg::I(o.virtual_port as i64)],
			);
		} else {
			log(
				"sally: virtual trackers off (%s): none added, no model process; the panel's switch turns them on\n",
				&[BArg::S(&path)],
			);
		}
		{
			let mut st = locked(&self.state);
			st.switch_path = path;
			if on {
				self.start_process_if_set(&mut st);
			}
		}
		self.running.store(true, SeqCst);
		*locked(&self.thread) = Some(std::thread::spawn(|| G_VIRTUAL.receive_thread()));
	}

	/// The model process started, when driver.virtual_exe names it (else it is said once that it is started
	/// separately)
	fn start_process_if_set(&self, st: &mut State) {
		if g_opt().virtual_exe.is_empty() {
			log("sally: virtual trackers: driver.virtual_exe not set: the model process is started separately\n", &[]);
		} else {
			self.start_process(st);
		}
	}

	/// The model process: `virtual_exe run --model <virtual_model> --port <virtual_port>` in the program's folder
	fn start_process(&self, st: &mut State) {
		let o = g_opt();
		st.last_start = now_seconds();
		let dir = locked(&self.dir).clone();
		let joined = |rel: &[u8]| -> Vec<u8> {
			let mut p = dir.clone();
			p.push(K_SEP);
			p.extend_from_slice(rel);
			p
		};
		let exe = if is_absolute(&o.virtual_exe) { o.virtual_exe.clone() } else { joined(&o.virtual_exe) };
		let cwd = match exe.iter().rposition(|&c| c == b'\\' || c == b'/') {
			Some(i) => exe[..i].to_vec(),
			None => dir.clone(),
		};
		let stamp = time_stamp();
		let vlog = expand_time(o.virtual_log.clone(), || stamp.clone());
		let log_path = if vlog.is_empty() || is_absolute(&vlog) { vlog } else { joined(&vlog) };
		let argv = vec![
			b"run".to_vec(),
			b"--model".to_vec(),
			o.virtual_model.clone(),
			b"--port".to_vec(),
			o.virtual_port.to_string().into_bytes(),
		];
		log(
			"sally: virtual trackers: starting the model process %s (model %s, console output %s)\n",
			&[
				BArg::S(&exe),
				BArg::S(&o.virtual_model),
				BArg::S(if log_path.is_empty() { b"discarded" } else { &log_path }),
			],
		);
		if !st.proc.start_program(&exe, &cwd, &log_path, &argv) {
			log("sally: virtual trackers: the model process did not start\n", &[]);
		}
	}

	/// Cleanup: the thread stopped, the process killed, the socket closed
	pub fn cleanup(&self) {
		if !self.enabled() {
			return;
		}
		self.running.store(false, SeqCst);
		if let Some(t) = locked(&self.thread).take() {
			let _ = t.join();
		}
		locked(&self.state).proc.kill();
		let sock = self.sock.swap(SOCK_BAD, SeqCst);
		if sock != SOCK_BAD {
			net::close(sock);
		}
	}

	fn send(&self, data: &[u8]) {
		let sock = self.sock.load(SeqCst);
		if sock == SOCK_BAD {
			return;
		}
		let to = SockAddrIn::loopback(g_opt().virtual_port as u16);
		net::sendto(sock, data, &to, SOCKADDR_IN_LEN);
	}

	/// RunFrame: while on, the ones shown added (the first frame they are on), SteamVR's poses sent to the process and
	/// the process kept running; stale ones put out of range, and switched-off ones said disconnected
	pub fn run_frame(&self, now: f64) {
		if !self.enabled() {
			return;
		}
		let on = self.on.load(SeqCst);
		if on && !self.added.swap(true, SeqCst) {
			for (k, d) in DEVICES.iter().enumerate().filter(|(k, _)| self.is_shown(*k)) {
				// SAFETY: the device is a static ITrackedDeviceServerDriver, live as long as the driver
				let took = unsafe {
					tracked_device_added(TRACKERS[k].1, TrackedDeviceClass_GenericTracker, d as *const _ as *mut c_void)
				};
				if !took {
					log("sally: virtual trackers: SteamVR did not take %s\n", &[BArg::S(TRACKERS[k].1.to_bytes())]);
				}
			}
		}
		if on {
			self.send_poses(now);
		}
		// stale or switched off: said to SteamVR once, and again every second
		let mut pushes = Vec::new();
		{
			let mut st = locked(&self.state);
			for k in 0..5 {
				let id = self.id(k);
				if id == k_unTrackedDeviceIndexInvalid {
					continue;
				}
				let stale = now - st.fresh[k] > STALE_S;
				if (stale || !on) && now - st.idle_sent[k] >= 1.0 {
					let mut p = idle_pose();
					p.deviceIsConnected = on as u8;
					st.pose[k] = p;
					st.idle_sent[k] = now;
					pushes.push((id, p));
				}
			}
			// the process: started again when it exits, at most every 5 s (while on)
			let o = g_opt();
			let mut code = 0;
			if on
				&& !o.virtual_exe.is_empty()
				&& now - st.last_start >= 5.0
				&& st.proc.started()
				&& !st.proc.alive(&mut code)
			{
				log(
					"sally: virtual trackers: the model process exited (code %d), starting it again\n",
					&[BArg::I(code as i64)],
				);
				self.start_process(&mut st);
			}
		}
		for (id, p) in pushes {
			tracked_device_pose_updated(id, &p);
		}
	}

	/// What SteamVR has, to the process: an F line, then an R line per headset, controller and tracker
	fn send_poses(&self, now: f64) {
		let mut raw = [TrackedDevicePose_t::default(); k_unMaxTrackedDeviceCount as usize];
		get_raw_tracked_device_poses(0.0, &mut raw);
		let mut out = bsprintf("F %.4f\n", &[BArg::F(now)]);
		for (id, p) in raw.iter().enumerate() {
			let id = id as u32;
			if p.bDeviceIsConnected == 0 || self.is_ours(id) {
				continue;
			}
			let info = locked(&self.state).known.get(&id).cloned();
			let info = match info {
				Some(Ok(i)) => Some(i),
				Some(Err(retry)) if now < retry => None,
				_ => {
					// (the reads are calls into vrserver, outside the lock)
					let c = tracked_device_to_property_container(id);
					let class = get_int32_property(c, Prop_DeviceClass_Int32);
					let (serial, err) = get_string_property(c, Prop_SerialNumber_String);
					let i = class.filter(|_| err == TrackedProp_Success && !serial.is_empty()).map(|cl| (cl, serial));
					// a read that fails (a device just connected, its properties not written yet) is tried again
					// after KNOWN_RETRY_S, not kept for the session: the model would never see the device
					locked(&self.state).known.insert(id, i.clone().ok_or(now + KNOWN_RETRY_S));
					i
				}
			};
			if let Some((class, serial)) = info {
				if let Some(l) = r_line(now, class, &serial, p) {
					out.extend_from_slice(&l);
				}
			}
		}
		self.send(&out);
		self.frames.fetch_add(1, SeqCst);
	}

	/// The receive thread: T lines handed to SteamVR as they come, S lines kept for the X line
	fn receive_thread(&self) {
		let mut buf = vec![0u8; 65536];
		while self.running.load(SeqCst) {
			let n = net::recv(self.sock.load(SeqCst), &mut buf[..65535]);
			if n <= 0 {
				continue;
			}
			let now = now_seconds();
			let data = buf[..n as usize].to_vec();
			let mut frame_t = None;
			let mut pushes = Vec::new();
			for line in lines(&data) {
				if let Some(rest) = line.strip_prefix(b"F ") {
					frame_t = std::str::from_utf8(rest).ok().and_then(|x| x.trim().parse::<f64>().ok());
				} else if let Some(rest) = line.strip_prefix(b"S ") {
					let mut st = locked(&self.state);
					st.status = rest.to_vec();
					st.status_t = now;
				} else if line.first() == Some(&b'T') {
					let age = frame_t.map(|t| now - t).unwrap_or(0.0);
					if let Some((k, p)) = parse_t(line, age) {
						pushes.push((k, p));
					}
				}
			}
			if pushes.is_empty() {
				continue;
			}
			self.answers.fetch_add(1, SeqCst);
			let on = self.on.load(SeqCst);
			let mut out = Vec::new();
			{
				let mut st = locked(&self.state);
				for (k, p) in pushes {
					let id = self.id(k);
					if !on || id == k_unTrackedDeviceIndexInvalid {
						continue;
					}
					if p.poseIsValid != 0 {
						st.fresh[k] = now;
						st.idle_sent[k] = f64::NEG_INFINITY;
					} else if now - st.idle_sent[k] < 1.0 {
						continue;
					} else {
						st.idle_sent[k] = now;
					}
					st.pose[k] = p;
					out.push((id, p));
				}
			}
			for (id, p) in out {
				tracked_device_pose_updated(id, &p);
			}
		}
	}

	/// The X line: whether they are on, how many vrserver has and how many are tracking, which ones they are
	/// (driver.virtual_trackers), and the process's word (or "none" when it is quiet), last: `X <on|off> devices N
	/// tracking N trackers <names> model <its word>`
	pub fn virtual_line(&self, now: f64) -> Vec<u8> {
		if !self.enabled() {
			return b"X none\n".to_vec();
		}
		let st = locked(&self.state);
		let active = (0..5).filter(|&k| self.id(k) != k_unTrackedDeviceIndexInvalid).count();
		let on = self.on.load(SeqCst);
		let tracking = if on { (0..5).filter(|&k| now - st.fresh[k] <= STALE_S).count() } else { 0 };
		let model: &[u8] = if now - st.status_t <= QUIET_S { &st.status } else { b"none" };
		let names = self.shown_names();
		let mut l = bsprintf(
			"X %s devices %d tracking %d trackers %s model %s\n",
			&[
				BArg::S(if on { b"on" } else { b"off" }),
				BArg::I(active as i64),
				BArg::I(tracking as i64),
				BArg::S(if names.is_empty() { b"none" } else { &names }),
				BArg::S(model),
			],
		);
		l.truncate(255);
		if l.last() != Some(&b'\n') {
			l.push(b'\n');
		}
		l
	}

	/// `virtual [on | off | toggle | calibrate]` from cmd_port: the X line to answer with. A switch is kept in
	/// sally_virtual.txt; on starts the process (the ones shown are added at the next frame), off stops it
	pub fn command(&self, args: &[&[u8]]) -> Vec<Vec<u8>> {
		if self.enabled() {
			let before = self.on.load(SeqCst);
			match args.first().copied() {
				Some(b"on") => self.on.store(true, SeqCst),
				Some(b"off") => self.on.store(false, SeqCst),
				Some(b"toggle") => self.on.store(!before, SeqCst),
				Some(b"calibrate") => {
					log("sally: virtual trackers: the T-pose asked for again\n", &[]);
					self.send(b"C calibrate\n");
				}
				_ => {}
			}
			let after = self.on.load(SeqCst);
			if after != before {
				let added = self.added.load(SeqCst);
				log(
					"sally: virtual trackers switched %s%s\n",
					&[
						BArg::S(if after { b"on" } else { b"off" }),
						BArg::S(if after || !added { b"" } else { b" (disconnected until SteamVR starts again)" }),
					],
				);
				// said to SteamVR at the next frame (disconnected) or with the next pose (connected)
				let mut st = locked(&self.state);
				st.idle_sent = [f64::NEG_INFINITY; 5];
				write_switch(&st.switch_path, after);
				if after {
					if !st.proc.started() || !st.proc.alive(&mut 0) {
						self.start_process_if_set(&mut st);
					}
				} else if st.proc.started() {
					log("sally: virtual trackers: stopping the model process\n", &[]);
					st.proc.kill();
					st.status_t = f64::NEG_INFINITY;
				}
			}
		}
		vec![self.virtual_line(now_seconds())]
	}

	/// Activate(id): the properties written, the id kept
	fn activate(&self, k: usize, id: u32) -> EVRInitError {
		self.ids[k].store(id, SeqCst);
		let c = tracked_device_to_property_container(id);
		// the headset's tracking system: its poses are in the headset's space already, and Space Calibrator moves the
		// devices of its target system (the lighthouse ones) onto the headset's, so it must leave these alone
		let (mut sys, err) = get_string_property(
			tracked_device_to_property_container(k_unTrackedDeviceIndex_Hmd),
			Prop_TrackingSystemName_String,
		);
		if err != TrackedProp_Success || sys.is_empty() {
			sys = b"sally".to_vec();
		}
		let (_, serial, model, ctype) = TRACKERS[k];
		set_string_property(c, Prop_TrackingSystemName_String, &sys);
		set_string_property(c, Prop_SerialNumber_String, serial.to_bytes());
		set_string_property(c, Prop_ModelNumber_String, model.as_bytes());
		set_string_property(c, Prop_ManufacturerName_String, b"sally");
		// Valve's tracker's render model, input profile and icons (without the icons SteamVR shows its invalid-status
		// one), the role in the controller type
		set_string_property(c, Prop_RenderModelName_String, b"{htc}vr_tracker_vive_1_0");
		set_string_property(c, Prop_InputProfilePath_String, b"{htc}/input/vive_tracker_profile.json");
		set_string_property(c, Prop_ControllerType_String, ctype.as_bytes());
		for (prop, icon) in [
			(Prop_NamedIconPathDeviceOff_String, &b"{htc}/icons/tracker_status_off.png"[..]),
			(Prop_NamedIconPathDeviceSearching_String, b"{htc}/icons/tracker_status_searching.gif"),
			(Prop_NamedIconPathDeviceSearchingAlert_String, b"{htc}/icons/tracker_status_searching_alert.gif"),
			(Prop_NamedIconPathDeviceReady_String, b"{htc}/icons/tracker_status_ready.png"),
			(Prop_NamedIconPathDeviceReadyAlert_String, b"{htc}/icons/tracker_status_ready_alert.png"),
			(Prop_NamedIconPathDeviceNotReady_String, b"{htc}/icons/tracker_status_error.png"),
			(Prop_NamedIconPathDeviceStandby_String, b"{htc}/icons/tracker_status_standby.png"),
			(Prop_NamedIconPathDeviceAlertLow_String, b"{htc}/icons/tracker_status_ready_low.png"),
		] {
			set_string_property(c, prop, icon);
		}
		set_int32_property(c, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
		set_bool_property(c, Prop_DeviceProvidesBatteryStatus_Bool, false);
		set_bool_property(c, Prop_WillDriftInYaw_Bool, false);
		set_bool_property(c, Prop_Identifiable_Bool, false);
		log(
			"sally: virtual tracker %s is device %u (tracking system %s, the headset's)\n",
			&[BArg::S(serial.to_bytes()), BArg::U(id as u64), BArg::S(&sys)],
		);
		VRInitError_None
	}

	/// The pose SteamVR was last handed for one of the five (GetPose)
	fn pose(&self, k: usize) -> DriverPose_t {
		locked(&self.state).pose[k]
	}
}

// ---------------------------------------------------------------- the devices' vtable, as vrserver calls it

/// Which of the five `this` is
///
/// # Safety
/// `this` is one of DEVICES, as vrserver hands back the device it was given
unsafe fn which(this: *mut ITrackedDeviceServerDriver) -> usize {
	// SAFETY: one of the static devices, whose layout starts with the vtable and then its index
	unsafe { (*(this as *const VirtualDevice)).k.min(4) }
}

unsafe extern "C" fn device_activate(this: *mut ITrackedDeviceServerDriver, id: u32) -> EVRInitError {
	// SAFETY: vrserver activates a device it was given
	G_VIRTUAL.activate(unsafe { which(this) }, id)
}

unsafe extern "C" fn device_deactivate(this: *mut ITrackedDeviceServerDriver) {
	// SAFETY: as above
	let k = unsafe { which(this) };
	G_VIRTUAL.ids[k].store(k_unTrackedDeviceIndexInvalid, SeqCst);
}

unsafe extern "C" fn device_enter_standby(_this: *mut ITrackedDeviceServerDriver) {}

unsafe extern "C" fn device_get_component(_this: *mut ITrackedDeviceServerDriver, _name: *const c_char) -> *mut c_void {
	std::ptr::null_mut()
}

unsafe extern "C" fn device_debug_request(
	_this: *mut ITrackedDeviceServerDriver, _req: *const c_char, buf: *mut c_char, n: u32,
) {
	if n > 0 && !buf.is_null() {
		// SAFETY: vrserver's buffer of n bytes
		unsafe { *buf = 0 };
	}
}

#[cfg(windows)]
unsafe extern "C" fn device_get_pose(
	this: *mut ITrackedDeviceServerDriver, ret: *mut DriverPose_t,
) -> *mut DriverPose_t {
	// SAFETY: vrserver's place for the pose, and a device it was given
	unsafe {
		*ret = G_VIRTUAL.pose(which(this));
	}
	ret
}

#[cfg(not(windows))]
unsafe extern "C" fn device_get_pose(this: *mut ITrackedDeviceServerDriver) -> DriverPose_t {
	// SAFETY: a device vrserver was given
	G_VIRTUAL.pose(unsafe { which(this) })
}

static DEVICE_VTBL: ITrackedDeviceServerDriverVtbl = ITrackedDeviceServerDriverVtbl {
	Activate: device_activate,
	Deactivate: device_deactivate,
	EnterStandby: device_enter_standby,
	GetComponent: device_get_component,
	DebugRequest: device_debug_request,
	GetPose: device_get_pose,
};

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_t_line_is_a_pose() {
		let (k, p) = parse_t(b"T lknee 1 0.1 0.5 -0.2 1 0 0 0 0.5 0 0 0 0.1 0", 0.004).unwrap();
		assert_eq!(k, 1);
		assert_eq!(p.vecPosition, [0.1, 0.5, -0.2]);
		assert_eq!(p.qRotation.w, 1.0);
		assert_eq!(p.vecVelocity[0], 0.5);
		assert_eq!(p.vecAngularVelocity[1], 0.1);
		assert_eq!(p.poseIsValid, 1);
		assert_eq!(p.result, TrackingResult_Running_OK);
		assert!((p.poseTimeOffset + 0.004).abs() < 1e-12);
		assert_eq!(p.qWorldFromDriverRotation.w, 1.0);
		let (_, idle) = parse_t(b"T chest 0 0 0 0 1 0 0 0 0 0 0 0 0 0", 0.0).unwrap();
		assert_eq!((idle.poseIsValid, idle.result, idle.deviceIsConnected), (0, TrackingResult_Running_OutOfRange, 1));
		assert!(parse_t(b"T knee 1 0 0 0 1 0 0 0 0 0 0 0 0 0", 0.0).is_none());
		assert!(parse_t(b"T chest 1 nan 0 0 1 0 0 0 0 0 0 0 0 0", 0.0).is_none());
	}

	#[test]
	fn the_switch_is_off_unless_kept_on() {
		assert!(!parse_switch(b""));
		assert!(!parse_switch(b"# nothing\n"));
		assert!(parse_switch(b"# kept\non=1\n"));
		assert!(parse_switch(b"on = 1\r\n"));
		assert!(!parse_switch(b"on=0\n"));
		assert!(!parse_switch(b"on=yes\n"));
		assert!(!parse_switch(b"on=1\non=0\n"));
	}

	#[test]
	fn the_trackers_shown() {
		assert_eq!(shown(b""), ([true; 5], vec![]));
		assert_eq!(shown(b"lknee,rknee,lelbow,relbow"), ([false, true, true, true, true], vec![]));
		assert_eq!(shown(b" chest , relbow"), ([true, false, false, false, true], vec![]));
		assert_eq!(shown(b"lknee,knee"), ([false, true, false, false, false], vec![b"knee".to_vec()]));
	}

	#[test]
	fn an_r_line_has_the_pose() {
		let mut p = TrackedDevicePose_t::default();
		p.mDeviceToAbsoluteTracking.m = [[1.0, 0.0, 0.0, 0.25], [0.0, 1.0, 0.0, 1.5], [0.0, 0.0, 1.0, -0.125]];
		p.bPoseIsValid = 1;
		p.eTrackingResult = TrackingResult_Running_OK;
		let l = r_line(12.5, TrackedDeviceClass_GenericTracker, b"LHR-1", &p).unwrap();
		assert_eq!(
			l,
			b"R 12.5000 tracker LHR-1 1 0.25000 1.50000 -0.12500 1.000000 0.000000 0.000000 0.000000\n".to_vec()
		);
		assert!(r_line(0.0, 4, b"LHB-1", &p).is_none());
	}
}
