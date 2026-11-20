//! The pose override (PoseOverride, docs/sally_driver.md).
//!
//! Valve's lighthouse driver runs the receivers as usual and the sally runtime reads the same reports read-only (HID
//! tap), so nothing ever talks to the receivers twice. This driver adds no devices. It replaces
//! IVRServerDriverHost::TrackedDevicePoseUpdated in the server's interface vtables, so every pose Valve's driver reports
//! passes through here first:
//!  * for Vive trackers and Index controllers (tracking system "lighthouse", serial LHR-...) Valve's pose (head pose in
//!    Valve's driver space) goes to the panels as a "V" line, and the pose SteamVR gets is the runtime's pose of the
//!    same serial, mapped into Valve's driver space by a rigid fit of the last fit_window_s of pairs (robust, the
//!    panel's fit). Valve's world-from-driver transform is kept (Space Calibrator's offset rides on it), the
//!    driver-from-head offset becomes identity (the runtime's pose already is the head pose).
//!  * everything else (headset, base stations, other drivers) passes through untouched.
//!
//! Apps, Space Calibrator and the headset view therefore see sally's tracking on Valve's own device entries (same ids,
//! roles, buttons, battery). "seen valve" on cmd_port switches SteamVR back to Valve's poses live, "seen sally" again.
//!
//! Which device is on which dongle is Valve's driver's to say (Prop_ConnectedWirelessDongle_String): the runtime's
//! devices are only as right as the configs it has for its dongles, and a device paired to another dongle leaves the
//! old one's config with another device (re-paired trackers then fly about on each other's poses). The driver tells the
//! runtime which device each dongle carries (`pairs`, the protocol's `pair`), and a device's runtime pose stands in for
//! Valve's only while the runtime has it on the dongle Valve's driver does (`same_device`). The headset, on a cable
//! with no dongle, is named to the runtime on its own serial, and keeps Valve's pose unless driver.hmd is set. Until
//! the frame fit is ready, or when sally has no fresh pose for a device, Valve's
//! pose is shown (blended, see on_pose). If Valve's driver stops reporting a device the runtime still tracks, the
//! runtime's pose is pushed by this driver.
//!
//! The state, by who reaches it: what is shared is `State`, behind `m`; what only RunFrame's thread touches (the
//! frame's timers, the pose log) is `Frame`; what is read without the lock (the switches, the hooks' originals, the
//! sockets) are atomics, and the panel ports have a lock of their own.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::ffi::{c_void, CStr};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicI64, AtomicUsize, Ordering::SeqCst};
use std::sync::{Mutex, MutexGuard, RwLock};

use sally_math::math3d::{
	compose, cross, inverse, len, norm, off_one_line, qangle, qconj, qmul, qnorm, qrot, xform, Pose, Q4, V3,
};
use sally_math::printf::{bsprintf, BArg};

use crate::body::{Body, Dash};
use crate::helpers::{expand_time, log, now_seconds, sin_cos, std_max, std_min, time_stamp, wall_seconds};
use crate::net::{self, SockAddrIn, SOCKADDR_IN_LEN, SOCK_BAD};
use crate::openvr::*;
use crate::output::Shown;
use crate::pose_file::PoseFile;
use crate::settings::{g_opt, Options};
use crate::toys::Kin;

/// The interface versions hooked, newest first
pub const K_VERSIONS: usize = 3;
pub const K_NAMES: [&CStr; K_VERSIONS] =
	[c"IVRServerDriverHost_006", c"IVRServerDriverHost_005", c"IVRServerDriverHost_004"];

/// The size TrackedDevicePoseUpdated is called with for a DriverPose_t
const POSE_SIZE: u32 = std::mem::size_of::<DriverPose_t>() as u32;

/// The runtime's latest pose of a device
#[derive(Clone, Copy, Debug)]
pub struct SallyLatest {
	pub have: bool,
	pub lost: bool,
	/// monotonic
	pub t_recv: f64,
	/// Y-up sally world, head frame, poseTimeOffset = -age
	pub pose: DriverPose_t,
	/// the output layer's part of the hand-over (output.rs): what the toys show in the pose's place, where the
	/// device is shown, the velocity's scale
	pub shown: Shown,
}

impl Default for SallyLatest {
	fn default() -> SallyLatest {
		SallyLatest { have: false, lost: true, t_recv: 0.0, pose: DriverPose_t::default(), shown: Shown::default() }
	}
}

/// One of vrserver's devices as Valve's driver reports it (std::map's operator[] value-initialises one: zeros, and
/// the member defaults)
#[derive(Clone, Debug)]
pub struct ValveDev {
	pub checked: bool,
	pub target: bool,
	pub next_check: f64,
	pub serial: Vec<u8>,
	/// vrserver's device 0: SteamVR's headset
	pub hmd: bool,
	/// the dongle Valve's driver says the device talks through (empty: it names none, or the read failed)
	pub dongle: Vec<u8>,
	/// whether the runtime's pose of the serial was the device's at the last pose (`same_device`; logged on change)
	pub ours: bool,
	/// the IVRServerDriverHost the pose came through (`self`), as an address
	pub this: usize,
	pub ver: i32,
	pub last_call: f64,
	pub last_export: f64,
	pub last_pair: f64,
	pub last_push: f64,
	pub last_exported_valid: i32,
	/// Valve's last pose as it arrived (its world-from-driver is reused for pushes)
	pub last: DriverPose_t,
	/// Valve's head pose in its driver space (the device's own: see `head_f`)
	pub head: Pose,
	pub valid: bool,
	/// monotonic time since which Valve has reported this device valid without a break
	pub valid_since: f64,
	pub last_sample: V3,
	pub last_sample_t: f64,
	pub sampled: bool,
	pub calls: u64,
	pub shown_sally: u64,
	pub shown_valve: u64,
	pub pushed: u64,
	/// Valve's world-from-driver for the device at its last pose, the head's position in Valve's driver space then,
	/// when, and whether that pose was valid (`follow_frame`)
	pub wfd: Pose,
	pub have_wfd: bool,
	pub wfd_head: V3,
	pub wfd_t: f64,
	pub wfd_valid: bool,
	/// the device's world-from-driver changed while its pose in Valve's driver space went on as it was (Valve's world
	/// moved, if the frame's other devices say so too): from, to, when
	pub world_move: Option<(Pose, Pose, f64)>,
	/// from the device's driver space (its own world-from-driver's) to the frame's: the identity while it is on the
	/// frame's
	pub to_frame: Pose,
	/// Valve's head pose read into the frame's driver space (`head` through `to_frame`)
	pub head_f: Pose,
	pub wfd_log_t: f64,
	/// poses on a world-from-driver other than the frame's, since the last stats
	pub wfd_other: u64,
	/// what we last handed SteamVR for this device (world = world-from-driver applied), and in Valve's driver space
	pub sent_w: V3,
	pub sent_head: V3,
	pub sent_t: f64,
	/// the whole pose we last handed SteamVR (for the pose log)
	pub sent: DriverPose_t,
	pub prev_shown: V3,
	pub prev_sent: V3,
	pub have_prev: bool,
	pub shown_log_t: f64,
	pub shown_jumps: u64,
	pub shown_invalid: u64,
	/// source blend: 1 = sally's pose, 0 = Valve's; slides toward the wanted source at 1/blend_ms per ms
	pub blend: f64,
	pub blend_t: f64,
	/// -1 unknown, 0 valve, 1 sally (what we are heading for; logged on change)
	pub source: i32,
	/// monotonic time since which Valve's pose has been continuously plausible
	pub valve_ok_since: f64,
	/// Valve's head pose with its world-from-driver applied (logged)
	pub valve_world: V3,
	pub prev_head: V3,
	pub have_prev_head: bool,
	/// last distance between Valve's head pose and ours, in the frame's driver space, and the angle between them (the
	/// head frames: HeadFix)
	pub valve_disagree: f64,
	pub valve_disagree_deg: f64,
	pub fell_back: u64,
}

impl Default for ValveDev {
	fn default() -> ValveDev {
		ValveDev {
			checked: false,
			target: false,
			next_check: 0.0,
			serial: Vec::new(),
			hmd: false,
			dongle: Vec::new(),
			ours: true,
			this: 0,
			ver: -1,
			last_call: 0.0,
			last_export: 0.0,
			last_pair: 0.0,
			last_push: 0.0,
			last_exported_valid: -1,
			last: DriverPose_t::default(),
			head: Pose::default(),
			valid: false,
			valid_since: -1.0,
			last_sample: V3::default(),
			last_sample_t: 0.0,
			sampled: false,
			calls: 0,
			shown_sally: 0,
			shown_valve: 0,
			pushed: 0,
			wfd: Pose::default(),
			have_wfd: false,
			wfd_head: V3::default(),
			wfd_t: 0.0,
			wfd_valid: false,
			world_move: None,
			to_frame: Pose::default(),
			head_f: Pose::default(),
			wfd_log_t: 0.0,
			wfd_other: 0,
			sent_w: V3::default(),
			sent_head: V3::default(),
			sent_t: 0.0,
			sent: DriverPose_t::default(),
			prev_shown: V3::default(),
			prev_sent: V3::default(),
			have_prev: false,
			shown_log_t: 0.0,
			shown_jumps: 0,
			shown_invalid: 0,
			blend: 0.0,
			blend_t: 0.0,
			source: -1,
			valve_ok_since: -1.0,
			valve_world: V3::default(),
			prev_head: V3::default(),
			have_prev_head: false,
			valve_disagree: 0.0,
			valve_disagree_deg: 0.0,
			fell_back: 0,
		}
	}
}

/// A fit pair: runtime (Y-up sally world) -> Valve driver space
#[derive(Clone, Copy, Debug)]
pub struct Sample {
	pub t: f64,
	pub src: V3,
	pub dst: V3,
	pub id: u32,
}

/// What the C++ keeps under m_
#[derive(Debug)]
pub struct State {
	pub devs: BTreeMap<u32, ValveDev>,
	pub sally: BTreeMap<Vec<u8>, SallyLatest>,
	pub first_pose_logged: BTreeSet<Vec<u8>>,
	pub samples: VecDeque<Sample>,
	/// T_: the runtime's frame in Valve's driver space
	pub t: Pose,
	pub fit_med: f64,
	pub fit_n: i32,
	pub fit_kept: i32,
	pub last_stats_t: f64,
	/// the dongle each of the runtime's devices is on, by serial (its device list's DEV lines)
	pub runtime_dongles: BTreeMap<Vec<u8>, Vec<u8>>,
	/// every dongle the runtime has, a device on it known or not
	pub known_dongles: BTreeSet<Vec<u8>>,
	/// Valve's driver and the runtime name dongles alike (a dongle of Valve's devices is one of the runtime's): until
	/// they do, `same_device` lets every pose through, as before there was a check
	pub dongles_agree: bool,
	/// when the dongles were first compared and did not agree (0: not yet, or they agree); logged once after 30 s
	pub disagree_since: f64,
	pub disagree_logged: bool,
	/// The frame: the world-from-driver of Valve's that sally's poses go out on, and the driver space the frame fit maps
	/// into (`follow_frame`); None until one of Valve's devices is valid
	pub frame_wfd: Option<Pose>,
	/// counts the frame's re-bases (the fit and its pairs moved with it): a fit worked out from pairs taken before one
	/// is dropped
	pub frame_gen: u64,
	/// a re-align asked for ("refit"): the pairs taken anew, and the first fit from them taken as it is (not eased in)
	pub refit_jump: bool,
	/// The runtime's head-to-IMU of each device (its DEV lines), by serial
	pub runtime_h2i: BTreeMap<Vec<u8>, Pose>,
	/// Valve's head frame against the runtime's, by serial (`HeadFix`)
	pub head_fix: BTreeMap<Vec<u8>, HeadFix>,
	/// `level`: the free fits' up in driver space averaged over LEVEL_TAU (a lighthouse headset's room setup), the
	/// frame it was taken in, when it was last updated; the correction last logged (rad) and when; a refusal logged
	pub level_up: Option<V3>,
	pub level_gen: u64,
	pub level_t: f64,
	pub level_logged: f64,
	pub level_log_t: f64,
	pub level_refused: bool,
	/// The toys that move the whole body (body.rs): their state between poses
	pub body: Body,
}

/// `fit`: how far off their own main line a fit's pairs have to lie (m, root mean square). A body's trackers standing
/// still are 0.1 to 0.3 m off theirs; at 0.1 m, 150 pairs that agree to 5 mm pin the turn about that line to 4 mrad
const FIT_OFF_LINE_M: f64 = 0.1;

/// `level`: how long the free fits' up is averaged over with a lighthouse headset (s)
const LEVEL_TAU: f64 = 600.0;
/// ... a free fit whose up is this far from the average (rad) starts it over (Valve's frame moved, as follow_frame's
/// jump)
const LEVEL_RESTART: f64 = 0.035;
/// ... and the runtime's up this far from the apps' (rad) is not levelled: the runtime's world is not level (a map
/// from before sally calibrate) or the apps' up is not where gravity is
const LEVEL_MAX: f64 = 0.1;

/// The smallest rotation taking the direction a onto the direction b (both unit)
fn rotation_between(a: V3, b: V3) -> Q4 {
	let c = cross(a, b);
	let s = len(c);
	let d = a.x * b.x + a.y * b.y + a.z * b.z;
	let ang = s.atan2(d);
	if s < 1e-12 {
		return Q4::new(1.0, 0.0, 0.0, 0.0);
	}
	let (sh, ch) = ((ang / 2.0).sin(), (ang / 2.0).cos());
	qnorm(Q4::new(ch, c.x / s * sh, c.y / s * sh, c.z / s * sh))
}

/// Level: the frame fit's tilt from gravity rather than from the pairs. The runtime's world is level (sally calibrate
/// levels its map: z, here +Y, is up), so its up belongs on the apps' up, which the fit only finds from where Valve's
/// devices are: over 60 s of a body turning in place a few metres of pairs pin a tilt poorly, and the frame wanders 1-7
/// mrad from minute to minute, 1.6-11 mm at the feet against the headset. And where Valve's devices sit in the apps'
/// world is itself only as level as Space Calibrator's alignment (5 to 40 mrad off). Both make a body's trackers lean
/// against the headset by a fixed direction of the room: turning around turns that lean against the body.
///
/// With a headset tracked in its own space (a Quest over Virtual Desktop or ALVR), the apps' world is level by that
/// headset's own IMU, so the fit's up goes on the +Y of the frame's world-from-driver seen from driver space: the
/// apps' +Y as far as nothing tilts the devices after this driver (Space Calibrator's alignment is put on after it,
/// and what that tilts is not seen here). With a lighthouse headset the apps' world is Valve's room setup, tilted
/// against gravity as it may be, and the headset sits in it: the fit keeps the room setup's tilt, the free fits' up
/// averaged over LEVEL_TAU instead of the last 60 s's. Yaw and position stay the fit's: the free fit is turned about the
/// pairs' centroid by the smallest rotation that takes its up onto the target
fn level(st: &mut State, tf: Pose, centroid: V3, now: f64) -> Pose {
	let up = V3::new(0.0, 1.0, 0.0);
	let free_up = qrot(tf.q, up);
	let lighthouse_hmd = st.devs.values().any(|d| d.target && d.hmd);
	let target = if lighthouse_hmd {
		let avg = match st.level_up {
			Some(u) if st.level_gen == st.frame_gen && qangle(rotation_between(u, free_up)) < LEVEL_RESTART => {
				let k = ((now - st.level_t) / LEVEL_TAU).clamp(0.0, 1.0);
				norm(u * (1.0 - k) + free_up * k)
			}
			_ => free_up,
		};
		st.level_up = Some(avg);
		st.level_gen = st.frame_gen;
		st.level_t = now;
		avg
	} else {
		let Some(wfd) = st.frame_wfd else { return tf };
		qrot(qconj(wfd.q), up)
	};
	let swing = rotation_between(free_up, target);
	let off = qangle(swing);
	if off > LEVEL_MAX {
		if !st.level_refused {
			st.level_refused = true;
			log(
				"sally: frame not levelled: the runtime's up is %.1f deg off the apps' (is its map from sally calibrate?)\n",
				&[BArg::F(off * 57.2958)],
			);
		}
		return tf;
	}
	if (off - st.level_logged).abs() > 0.003 && now - st.level_log_t > 30.0 {
		st.level_logged = off;
		st.level_log_t = now;
		let what: &[u8] = if lighthouse_hmd { b"the room setup's long-run up" } else { b"the apps' up" };
		log("sally: frame levelled: its fit's up %.1f mrad off %s\n", &[BArg::F(off * 1e3), BArg::S(what)]);
	}
	let q = qnorm(qmul(swing, tf.q));
	Pose { p: xform(&tf, centroid) - qrot(q, centroid), q }
}

/// Valve's head frame for a device against the runtime's. Valve's driver pose is the IMU's, in the frame of the
/// config's "imu", and its driver-from-head says where Valve puts the head point and axes; the runtime's head-to-IMU
/// (from the config's "head") says where the runtime does. On Index controllers and Vive trackers 3.0 the two are the
/// same (to 0.1 mm / 0.6 deg); on Tundra trackers Valve's head is turned 180 deg about its Y axis and 1.5 mm off, so a
/// Tundra on the runtime's head frame shows upside down. SteamVR, its render models and apps go by Valve's, so the
/// runtime's head pose goes out turned into it: runtime head pose * `c` = Valve's head pose, `c` = head-to-IMU^-1 *
/// driver-from-head. Only where the two put the head point within K_HEAD_FIX_M of each other: further apart, Valve's
/// driver pose is not at the IMU the runtime has (another driver, another convention) and the two say nothing about
/// each other's head (`apply` false: the runtime's head frame as it is)
#[derive(Clone, Copy, Debug)]
pub struct HeadFix {
	pub h2i: Pose,
	pub dfh: Pose,
	pub c: Pose,
	pub apply: bool,
}

/// How far apart Valve's and the runtime's head points may be for HeadFix (a Tundra's are 1.5 mm, an Index controller's
/// 0.1)
const K_HEAD_FIX_M: f64 = 0.005;

/// What only RunFrame's thread touches
struct Frame {
	last_fit: f64,
	last_log: f64,
	plog: Option<PoseFile>,
	plog_failed: bool,
	plog_flush: f64,
}

/// The hooks' slots (void **) and detours, as addresses
struct Hooks {
	slot: [usize; K_VERSIONS],
	detour: [usize; K_VERSIONS],
}

pub struct PoseOverride {
	active: AtomicBool,
	passthrough: AtomicBool,
	/// pose hook calls in progress: Stop waits for them before it closes the sockets
	inflight: AtomicI32,
	seen: AtomicI32,
	/// a lighthouse headset gets sally's pose (the panel's Headset tracking; with driver.hmd)
	headset: AtomicBool,
	/// the pose log is written (the panel's Pose log, over driver.pose_log_on; with a file named, driver.pose_log)
	pose_log_on: AtomicBool,
	hooked: AtomicI32,
	fit_ok: AtomicBool,
	orig: [AtomicUsize; K_VERSIONS],
	hooks: Mutex<Hooks>,
	out: AtomicI64,
	cmd: AtomicI64,
	outs: RwLock<Vec<SockAddrIn>>,
	m: Mutex<State>,
	frame: Mutex<Frame>,
	/// a device came (back): its dongle is read at the next frame rather than at the next second
	dongle_check: AtomicBool,
	/// a device's dongle changed: the receive thread sends the pairs now rather than at the next second
	pairs_changed: AtomicBool,
	/// "refit" came on the command port: the next fit starts over (`fit`)
	refit_req: AtomicBool,
	/// A toy that moves the whole body is on, or something of one is still shown (body.rs): every pose that goes to
	/// SteamVR is moved. While it is not, the hook does nothing of it: a pose goes out exactly as before there were any
	body_on: AtomicBool,
}

/// The override (g_override)
pub static G_OVERRIDE: PoseOverride = PoseOverride::new();

/// A pose hook call in progress, counted while it lives
struct Inflight<'a>(&'a AtomicI32);

impl<'a> Inflight<'a> {
	fn new(n: &'a AtomicI32) -> Inflight<'a> {
		n.fetch_add(1, SeqCst);
		Inflight(n)
	}
}

impl Drop for Inflight<'_> {
	fn drop(&mut self) {
		self.0.fetch_sub(1, SeqCst);
	}
}

/// A Pose from a DriverPose_t's quaternion and translation
fn pose_of(q: &HmdQuaternion_t, p: &[f64; 3]) -> Pose {
	Pose { p: V3::new(p[0], p[1], p[2]), q: Q4::new(q.w, q.x, q.y, q.z) }
}

fn hmd_quat(q: Q4) -> HmdQuaternion_t {
	HmdQuaternion_t { w: q.w, x: q.x, y: q.y, z: q.z }
}

/// the period of the pose log's look ahead (kPoseLogAhead)
const K_POSE_LOG_AHEAD: f64 = 0.04;

impl PoseOverride {
	const fn new() -> PoseOverride {
		PoseOverride {
			active: AtomicBool::new(false),
			passthrough: AtomicBool::new(false),
			inflight: AtomicI32::new(0),
			seen: AtomicI32::new(0),
			headset: AtomicBool::new(true),
			pose_log_on: AtomicBool::new(true),
			hooked: AtomicI32::new(0),
			fit_ok: AtomicBool::new(false),
			orig: [AtomicUsize::new(0), AtomicUsize::new(0), AtomicUsize::new(0)],
			hooks: Mutex::new(Hooks { slot: [0; K_VERSIONS], detour: [0; K_VERSIONS] }),
			out: AtomicI64::new(SOCK_BAD),
			cmd: AtomicI64::new(SOCK_BAD),
			outs: RwLock::new(Vec::new()),
			m: Mutex::new(State {
				devs: BTreeMap::new(),
				sally: BTreeMap::new(),
				first_pose_logged: BTreeSet::new(),
				samples: VecDeque::new(),
				t: Pose { p: V3::new(0.0, 0.0, 0.0), q: Q4::new(1.0, 0.0, 0.0, 0.0) },
				fit_med: 0.0,
				fit_n: 0,
				fit_kept: 0,
				last_stats_t: 0.0,
				runtime_dongles: BTreeMap::new(),
				known_dongles: BTreeSet::new(),
				dongles_agree: false,
				disagree_since: 0.0,
				disagree_logged: false,
				frame_wfd: None,
				frame_gen: 0,
				refit_jump: false,
				runtime_h2i: BTreeMap::new(),
				head_fix: BTreeMap::new(),
				level_up: None,
				level_gen: 0,
				level_t: 0.0,
				level_logged: 0.0,
				level_log_t: -1e9,
				level_refused: false,
				body: Body::new(),
			}),
			frame: Mutex::new(Frame { last_fit: 0.0, last_log: 0.0, plog: None, plog_failed: false, plog_flush: 0.0 }),
			dongle_check: AtomicBool::new(false),
			pairs_changed: AtomicBool::new(false),
			refit_req: AtomicBool::new(false),
			body_on: AtomicBool::new(false),
		}
	}

	/// m_ locked (a panic while it was held aborts the process, so a poisoned lock never reaches here)
	pub fn lock(&self) -> MutexGuard<'_, State> {
		self.m.lock().unwrap_or_else(|e| e.into_inner())
	}

	fn frame(&self) -> MutexGuard<'_, Frame> {
		self.frame.lock().unwrap_or_else(|e| e.into_inner())
	}

	/// orig_[ver], the TrackedDevicePoseUpdated the hook replaced
	fn orig(&self, ver: usize) -> Option<PoseUpdatedFn> {
		let f = self.orig[ver].load(SeqCst);
		if f == 0 {
			return None;
		}
		// SAFETY: a non-zero orig_ is the slot's function as Install found it, of the slot's type
		Some(unsafe { std::mem::transmute::<usize, PoseUpdatedFn>(f) })
	}

	pub fn active(&self) -> bool {
		self.active.load(SeqCst)
	}

	/// seen_: 0 = SteamVR shows sally's poses, 1 = Valve's
	pub fn seen(&self) -> i32 {
		self.seen.load(SeqCst)
	}

	pub fn fit_ok(&self) -> bool {
		self.fit_ok.load(SeqCst)
	}

	/// Start(ctx): the panels' sockets, the command socket, the hooks
	pub fn start(&self, ctx: *mut IVRDriverContext) {
		let o = g_opt();
		self.active.store(true, SeqCst);
		self.seen.store(if o.seen == b"valve" { 1 } else { 0 }, SeqCst);
		let user = crate::user::get();
		self.headset.store(user.headset_sally, SeqCst);
		self.pose_log_on.store(user.pose_log.unwrap_or(o.pose_log_on), SeqCst);
		// outputs: Valve's poses + status to the panels
		self.out.store(net::socket_udp(), SeqCst);
		let ports: Vec<u8> = o.valve_udp.iter().map(|&c| if c == b',' || c == b';' { b' ' } else { c }).collect();
		{
			let mut outs = self.outs.write().unwrap_or_else(|e| e.into_inner());
			for tok in crate::helpers::split(&ports) {
				let port = sally_math::c_atoi(tok);
				if port <= 0 {
					continue;
				}
				outs.push(SockAddrIn::loopback(port as u16));
			}
		}
		// commands
		let cmd = net::socket_udp();
		self.cmd.store(cmd, SeqCst);
		if cmd != SOCK_BAD {
			let a = SockAddrIn::loopback(o.cmd_port as u16);
			if net::bind(cmd, &a) != 0 {
				log("sally: cannot listen for commands on port %d\n", &[BArg::I(o.cmd_port as i64)]);
				net::close(cmd);
				self.cmd.store(SOCK_BAD, SeqCst);
			} else {
				net::set_nonblocking(cmd);
			}
		}
		self.install(ctx);
		log(
			"sally: pose override: SteamVR shows %s's poses on Valve's devices (Valve's poses go to 127.0.0.1 ports %s, \
			 commands on %d)\n",
			&[
				BArg::S(if self.seen() != 0 { b"Valve" } else { b"sally" }),
				BArg::S(&o.valve_udp),
				BArg::I(o.cmd_port as i64),
			],
		);
	}

	/// Stop(): the hooks restored (where they are still ours) and the sockets closed, once the hook calls in progress
	/// are done
	pub fn stop(&self) {
		if !self.active() {
			return;
		}
		self.passthrough.store(true, SeqCst);
		// a Valve thread may be inside the hook (or sending its V line after it): let it finish before the restore
		// and the socket close
		let mut i = 0;
		while i < 100 && self.inflight.load(SeqCst) > 0 {
			sally_os::chrono::sleep_for_ms(1);
			i += 1;
		}
		let hooks = self.hooks.lock().unwrap_or_else(|e| e.into_inner());
		for i in 0..K_VERSIONS {
			if hooks.slot[i] == 0 {
				continue;
			}
			let slot = hooks.slot[i] as *mut *mut c_void;
			// SAFETY: slot is the vtable slot Install hooked, in vrserver's live interface
			let current = unsafe { slot.read_volatile() } as usize;
			if current == hooks.detour[i] && write_slot(slot, self.orig[i].load(SeqCst) as *mut c_void) {
				log("sally: restored %s::TrackedDevicePoseUpdated\n", &[BArg::S(K_NAMES[i].to_bytes())]);
			} else {
				log(
					"sally: %s::TrackedDevicePoseUpdated was hooked again after us; ours stays as a pass-through\n",
					&[BArg::S(K_NAMES[i].to_bytes())],
				);
			}
		}
		let out = self.out.load(SeqCst);
		if out != SOCK_BAD {
			net::close(out);
		}
		let cmd = self.cmd.load(SeqCst);
		if cmd != SOCK_BAD {
			net::close(cmd);
		}
		self.out.store(SOCK_BAD, SeqCst);
		self.cmd.store(SOCK_BAD, SeqCst);
		// the pose log's last rows: a packed one holds them until its next block
		if let Some(plog) = self.frame().plog.as_mut() {
			plog.flush();
		}
	}

	/// OnPose: the hook, on whichever thread the reporting driver uses
	///
	/// # Safety
	/// `p` points at a pose of `size` bytes, live through the call, and `this` is the interface it came through, as
	/// vrserver's caller passes them
	pub unsafe fn on_pose(&self, ver: usize, this: *mut c_void, id: u32, p: *const DriverPose_t, size: u32) {
		let Some(orig) = self.orig(ver) else { return };
		let _guard = Inflight::new(&self.inflight);
		if self.passthrough.load(SeqCst) || size != POSE_SIZE || id >= k_unMaxTrackedDeviceCount {
			// SAFETY: the caller's arguments, handed on unchanged to the function the hook replaced
			unsafe { orig(this, id, p, size) };
			return;
		}
		// SAFETY: a pose of `size` bytes, which is a DriverPose_t's (checked above), live through the call
		let pr: &DriverPose_t = unsafe { &*p };
		let now = now_seconds();
		let need_check = {
			let mut st = self.lock();
			let d = st.devs.entry(id).or_default();
			// a failed property read is retried every 0.25 s, not on every pose (each read is a call into vrserver)
			(!d.checked || !d.target) && now >= d.next_check
		};
		if need_check {
			self.identify(id, now);
		}
		// not ours: handed on as it came
		let (mut out, mut replace, exp, record) =
			self.on_pose_locked(ver, this, id, pr, now).unwrap_or((*pr, false, None, None));
		// the toys that move the whole body (body.rs): every device's pose, ours or not, goes out moved as one
		if self.body_on.load(SeqCst) {
			if let Some(moved) = self.body_move(id, if replace { &out } else { pr }, now) {
				(out, replace) = (moved, true);
			}
		}
		// a pose of sally's goes out on Valve's own record (valve_record), last: everything up to here goes by the head
		// pose
		if let Some(dfh) = record.filter(|_| replace) {
			valve_record(&mut out, &dfh);
		}
		// SAFETY: the caller's interface and id, with its pose or ours (which lives through the call), handed to the
		// function the hook replaced
		unsafe { orig(this, id, if replace { &out } else { p }, size) };
		if let Some(line) = exp.and_then(|v| v.text()) {
			let sock = self.out.load(SeqCst);
			for a in self.outs.read().unwrap_or_else(|e| e.into_inner()).iter() {
				net::sendto(sock, &line, a, SOCKADDR_IN_LEN);
			}
		}
	}

	/// The body's move (body.rs: dash punch's slide) on a pose that is about to go to SteamVR; None where there is no
	/// move
	pub fn body_move(&self, id: u32, src: &DriverPose_t, now: f64) -> Option<DriverPose_t> {
		let mut st = self.lock();
		let m = st.body.at(now);
		if !st.body.busy() {
			self.body_on.store(false, SeqCst);
		}
		let mut o = *src;
		m?.apply(&mut o);
		if let Some(d) = st.devs.get_mut(&id).filter(|d| d.target && d.sent_t == now) {
			// what check_shown compares SteamVR's pose with
			let wu = pose_of(&o.qWorldFromDriverRotation, &o.vecWorldFromDriverTranslation);
			d.sent_w = xform(&wu, V3::new(o.vecPosition[0], o.vecPosition[1], o.vecPosition[2]));
		}
		Some(o)
	}

	/// on_pose under m_: None for a device that is not ours (handed on as it is), else the pose SteamVR gets (ours
	/// where `replace`), the V line to send once it has it, and for a pose of sally's the driver-from-head it goes out
	/// on (`valve_record`; None for Valve's own pose, and with driver.valve_record off)
	#[allow(clippy::type_complexity)]
	fn on_pose_locked(
		&self, ver: usize, this: *mut c_void, id: u32, p: &DriverPose_t, now: f64,
	) -> Option<(DriverPose_t, bool, Option<VLine>, Option<Pose>)> {
		let o = g_opt();
		let mut out = DriverPose_t::default();
		let mut replace = false;
		let mut record: Option<Pose> = None;
		let mut st = self.lock();
		let fit_ok = self.fit_ok();
		if !st.devs.entry(id).or_default().target {
			return None;
		}
		let w = pose_of(&p.qWorldFromDriverRotation, &p.vecWorldFromDriverTranslation);
		let dp = pose_of(&p.qRotation, &p.vecPosition);
		let h = pose_of(&p.qDriverFromHeadRotation, &p.vecDriverFromHeadTranslation);
		let head = compose(&dp, &h);
		let valid = p.poseIsValid != 0 && p.result == TrackingResult_Running_OK;
		let vel = V3::new(p.vecVelocity[0], p.vecVelocity[1], p.vecVelocity[2]);
		follow_frame(&mut st, id, &w, head.p, vel, valid, now);
		update_head_fix(&mut st, id, &h);
		let State { devs, sally, samples, t, runtime_dongles, dongles_agree, frame_wfd, .. } = &mut *st;
		let frame: Option<Pose> = *frame_wfd;
		let d = devs.entry(id).or_default();
		if p.deviceIsConnected != 0 && (d.last.deviceIsConnected == 0 || now - d.last_call > 1.0) {
			// connected (again): it may have come back on another dongle
			self.dongle_check.store(true, SeqCst);
		}
		d.calls += 1;
		d.this = this as usize;
		d.ver = ver as i32;
		d.last_call = now;
		d.last = *p;
		d.head = head;
		d.valid = valid;
		// Valve's head pose in the frame's driver space: through its own world-from-driver into the world, and out of it
		// through the frame's (follow_frame). A device Valve tracks in a driver space of its own (one it boots or
		// re-boots) then reads where it is, not metres off
		let on_frame = frame.is_none_or(|f| near(&f, &w, 1e-6, 1e-7));
		d.to_frame = match frame {
			Some(f) if !on_frame => compose(&inverse(&f), &w),
			_ => Pose::default(),
		};
		d.head_f = if on_frame { d.head } else { compose(&d.to_frame, &d.head) };
		if let Some(f) = frame {
			let (dpos, da) = (len(w.p - f.p), qangle(qmul(w.q, qconj(f.q))));
			if dpos > 0.01 || da > 0.0087 {
				d.wfd_other += 1;
				if now - d.wfd_log_t > 1.0 {
					d.wfd_log_t = now;
					log(
						"sally: %s Valve's world-from-driver is %.0f mm / %.1f deg from the frame's (Valve %s): its pose \
						 read through it, sally's on the frame's\n",
						&[
							BArg::S(&d.serial),
							BArg::F(dpos * 1e3),
							BArg::F(da * 57.2958),
							BArg::S(if d.valid { b"valid" } else { b"not tracking" }),
						],
					);
				}
			}
		}
		// sent after SteamVR has its pose, outside the lock
		let exp = self.export(d, now, fit_ok);

		// the runtime's pose of the serial, while it is this device's (the headset's only with driver.hmd)
		let ours = (!d.hmd || (o.hmd && self.headset.load(SeqCst))) && same_device(d, runtime_dongles, *dongles_agree);
		if ours != d.ours {
			d.ours = ours;
			log_identity(d, runtime_dongles);
		}
		let s: Option<&SallyLatest> = sally.get(&d.serial).filter(|s| s.have && ours);
		// fit pairs: the runtime's pose carried to the time of Valve's pose. Valve's first seconds on a device it had
		// lost do not say where its frame is: its driver re-boots the device and puts it back where it should be over
		// 2-12 s, 1.5-7 m off meanwhile while calling it valid. A device Valve tracks moves the body around in the
		// window, so those pairs would pull the whole frame (the shown body turned 10 deg / moved 20 cm for 2 s). Pairs
		// are taken only once Valve has tracked the device for K_PAIR_SETTLE_S without a break; a move of Valve's whole
		// frame keeps its devices valid, so it is followed.
		const K_PAIR_SETTLE_S: f64 = 20.0;
		if !d.valid {
			d.valid_since = -1.0;
		} else if d.valid_since < 0.0 {
			d.valid_since = now;
		}
		if let Some(s) = s {
			if d.valid
				&& now - d.valid_since >= K_PAIR_SETTLE_S
				&& !s.lost && now - s.t_recv < 0.03
				&& now - d.last_pair >= 0.1
			{
				let t_sally = s.t_recv + s.pose.poseTimeOffset;
				let t_valve = now + p.poseTimeOffset;
				let dt = std_max(-0.05, std_min(0.05, t_valve - t_sally));
				let src = V3::new(
					s.pose.vecPosition[0] + s.pose.vecVelocity[0] * dt,
					s.pose.vecPosition[1] + s.pose.vecVelocity[1] * dt,
					s.pose.vecPosition[2] + s.pose.vecVelocity[2] * dt,
				);
				// a pair when the device moved 2 cm, and every 2 s while it is still: a still body is evidence of the
				// frame too, and without it whatever moves (above: a device Valve is still putting back) fills the
				// window (this compared with last_pair, which is refreshed every 0.1 s, so still devices gave no pairs
				// at all)
				if !d.sampled || len(d.head_f.p - d.last_sample) > 0.02 || now - d.last_sample_t > 2.0 {
					samples.push_back(Sample { t: now, src, dst: d.head_f.p, id });
					d.last_sample = d.head_f.p;
					d.last_sample_t = now;
					d.sampled = true;
				}
				d.last_pair = now;
			}
		}
		// Is Valve's pose a usable fallback right now? The driver pose is in Valve's driver space; the room-setup
		// (standing universe) transform is applied by SteamVR afterwards and affects our pose and Valve's alike, so
		// absolute height says nothing here (first live session: Valve "0.5-2.3 m below the floor" all day, fine in
		// VR). Plausible = valid, no frame-to-frame jump, and while we have a fresh pose, within valve_agree_m of it
		// (in the frame's driver space: where SteamVR would show it, its own world-from-driver applied)
		let mut valve_plausible = d.valid;
		{
			d.valve_world = xform(&w, d.head.p);
			if d.have_prev_head && len(d.head_f.p - d.prev_head) > 0.5 {
				valve_plausible = false; // a jump: Valve re-acquiring or resetting
			}
			d.prev_head = d.head_f.p;
			d.have_prev_head = true;
			if let Some(s) = s {
				if valve_plausible && !s.lost && fit_ok && now - s.t_recv < 0.05 {
					let sp = &s.pose;
					let ours = xform(t, V3::new(sp.vecPosition[0], sp.vecPosition[1], sp.vecPosition[2]));
					d.valve_disagree = len(ours - d.head_f.p);
					let ours_q = qmul(t.q, Q4::new(sp.qRotation.w, sp.qRotation.x, sp.qRotation.y, sp.qRotation.z));
					d.valve_disagree_deg = qangle(qmul(qconj(ours_q), d.head_f.q)) * 57.2958;
					if d.valve_disagree > o.valve_agree_m {
						valve_plausible = false;
					}
				}
			}
		}
		if valve_plausible {
			if d.valve_ok_since < 0.0 {
				d.valve_ok_since = now;
			}
		} else {
			d.valve_ok_since = -1.0;
		}
		let valve_usable = valve_plausible && now - d.valve_ok_since >= o.valve_settle_s;

		let sally_fresh = s.is_some_and(|s| !s.lost && now - s.t_recv < o.fallback_after_ms * 1e-3);
		let sally_long_stale = match s {
			None => true,
			Some(s) => s.lost || now - s.t_recv > o.fallback_force_ms * 1e-3,
		};
		// fresh sally > settled Valve > (sally stale for a while) any valid Valve > stale sally
		let valve_wins = o.fallback && (valve_usable || (sally_long_stale && d.valid));
		let want_sally = self.seen() == 0 && fit_ok && s.is_some() && (sally_fresh || !valve_wins);
		let want: i32 = if want_sally { 1 } else { 0 };
		if want != d.source {
			if d.source >= 0 {
				log(
					"sally: %s: SteamVR gets %s's pose (sally %s, Valve %s: valid %d, %.0f mm from ours, plausible for \
					 %.1f s)\n",
					&[
						BArg::S(&d.serial),
						BArg::S(if want != 0 { b"sally" } else { b"Valve" }),
						BArg::S(if sally_fresh {
							b"fresh"
						} else if s.is_some() {
							b"stale"
						} else {
							b"none"
						}),
						BArg::S(if valve_usable { b"usable" } else { b"not usable" }),
						BArg::I(d.valid as i64),
						BArg::F(d.valve_disagree * 1e3),
						BArg::F(if d.valve_ok_since < 0.0 { 0.0 } else { now - d.valve_ok_since }),
					],
				);
			}
			if want == 0 && d.source == 1 && self.seen() == 0 {
				d.fell_back += 1;
			}
			d.source = want;
			if d.blend_t == 0.0 {
				d.blend = want as f64; // first decision: no blend, nothing shown yet
			}
		}
		// slide the blend toward the wanted source
		if d.blend_t > 0.0 && o.blend_ms > 0.0 {
			// per call, so a pause in Valve's updates cannot turn the slide into a jump (at least 5 frames)
			let step = std_min(0.2, (now - d.blend_t) * 1e3 / o.blend_ms);
			d.blend = if want != 0 { std_min(1.0, d.blend + step) } else { std_max(0.0, d.blend - step) };
		} else {
			d.blend = want as f64;
		}
		d.blend_t = now;

		// where the device is shown against where it is (the output layer's offset) holds for Valve's pose too while
		// sally is what SteamVR is to show: a fallback to it does not move the device by the offset
		let offset = s.filter(|_| self.seen() == 0).and_then(|s| s.shown.offset);
		match s {
			Some(s) if fit_ok && d.blend > 0.0 => {
				substitute(&o, t, s, p, frame.as_ref(), now, &mut out, d);
				if d.blend < 1.0 {
					// mid-blend: mix sally's and Valve's head poses in the frame's driver space (`out` carries the
					// frame's world-from-driver), Valve's velocity scaled in so SteamVR's prediction stays continuous
					let a = d.blend;
					let vh = match &offset {
						Some(off) => compose(&d.head_f, off),
						None => d.head_f,
					};
					// Valve's velocity is its driver pose's origin's (the IMU's), sally's the shown point's: Valve's is
					// carried to that point by the turn of the arm between the two (v + w x r, its body rate in the driver
					// pose's own axes), as `valve_record` carries sally's the other way. A pose of Valve's without a
					// velocity keeps none
					let mut vv = qrot(d.to_frame.q, vel);
					let origin = compose(&d.to_frame, &dp);
					let rate = V3::new(p.vecAngularVelocity[0], p.vecAngularVelocity[1], p.vecAngularVelocity[2]);
					let turn = cross(qrot(origin.q, rate), vh.p - origin.p);
					if p.vecVelocity != [0.0; 3] && turn.x.is_finite() && turn.y.is_finite() && turn.z.is_finite() {
						vv += turn;
					}
					for i in 0..3 {
						out.vecPosition[i] = a * out.vecPosition[i] + (1.0 - a) * vh.p.get(i);
						out.vecVelocity[i] = a * out.vecVelocity[i] + (1.0 - a) * vv.get(i);
					}
					let qs = Q4::new(out.qRotation.w, out.qRotation.x, out.qRotation.y, out.qRotation.z);
					let mut qv = vh.q;
					if qs.w * qv.w + qs.x * qv.x + qs.y * qv.y + qs.z * qv.z < 0.0 {
						qv = Q4::new(-qv.w, -qv.x, -qv.y, -qv.z);
					}
					let q = Q4::new(
						a * qs.w + (1.0 - a) * qv.w,
						a * qs.x + (1.0 - a) * qv.x,
						a * qs.y + (1.0 - a) * qv.y,
						a * qs.z + (1.0 - a) * qv.z,
					);
					let n = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
					out.qRotation = HmdQuaternion_t { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
					// while blending toward Valve because sally is gone, sally's pose may be stale: report it as valid
					// anyway (it is within blend_ms of a good one) rather than flashing out-of-range
					out.poseIsValid = 1;
					out.result = TrackingResult_Running_OK;
				}
				replace = true;
				if o.valve_record {
					// on Valve's driver-from-head, with the device's offset as a fallback to Valve's pose has it: the
					// driver pose is then the IMU's, as Valve's own is
					record = Some(match &offset {
						Some(off) => compose(&h, off),
						None => h,
					});
				}
				if want != 0 {
					d.shown_sally += 1;
				} else {
					d.shown_valve += 1;
				}
			}
			_ => {
				if let Some(off) = offset.filter(|_| fit_ok) {
					// Valve's pose as the fallback, with the device's offset: on its driver-from-head, which is where
					// Valve's driver says the device is against its pose
					let dfh = compose(&h, &off);
					out = *p;
					out.qDriverFromHeadRotation = hmd_quat(dfh.q);
					out.vecDriverFromHeadTranslation = [dfh.p.x, dfh.p.y, dfh.p.z];
					replace = true;
				}
				d.shown_valve += 1;
			}
		}
		Some((out, replace, exp, record))
	}

	/// OnSallyPose: a runtime pose for serial (receive thread); pushed to SteamVR when Valve's driver has gone quiet
	/// on that device
	pub fn on_sally_pose(&self, serial: &[u8], pose: &DriverPose_t, shown: Shown, now: f64) {
		let mut st = self.lock();
		// in Valve's head frame (HeadFix); what the toys show in the pose's place goes into it as the pose does
		let mut shown = shown;
		let pose = match st.head_fix.get(serial) {
			Some(h) if h.apply => {
				shown.reshaped = shown.reshaped.map(|k| {
					let mut o = *pose;
					k.put(&mut o);
					Kin::of(&head_fixed(&o, &h.c))
				});
				head_fixed(pose, &h.c)
			}
			_ => *pose,
		};
		if !st.sally.contains_key(serial) {
			st.sally.insert(serial.to_vec(), SallyLatest::default());
		}
		let Some(s) = st.sally.get_mut(serial) else { return };
		let was_lost = s.lost;
		s.have = true;
		s.lost = false;
		s.t_recv = now;
		s.pose = pose;
		s.shown = shown;
		if was_lost && st.first_pose_logged.insert(serial.to_vec()) {
			log("sally: runtime pose for %s\n", &[BArg::S(serial)]);
		}
		// dash punch (body.rs): what the device's rail gives the whole body to slide by, in SteamVR's world (through
		// the fit and the frame's world-from-driver), while SteamVR is shown sally's poses
		let dash = shown.dash.filter(|_| self.seen() == 0 && self.fit_ok());
		if dash.is_some() || !st.body.slides.is_empty() {
			let q = match st.frame_wfd {
				Some(f) => qmul(f.q, st.t.q),
				None => st.t.q,
			};
			let world = |v: [f64; 3]| {
				let r = qrot(q, V3::new(v[0], v[1], v[2]));
				[r.x, r.y, r.z]
			};
			let dash = dash.map(|d| Dash { shift: world(d.shift), rate: world(d.rate), back_s: d.back_s });
			st.body.dash(now, serial, dash);
			if st.body.busy() {
				self.body_on.store(true, SeqCst);
			}
		}
		drop(self.push_if_quiet(serial, now, st));
	}

	/// The runtime's head-to-IMU of a device (its DEV line's: translation, then the rotation w x y z), for HeadFix
	pub fn runtime_head(&self, serial: &[u8], h2i: &[f64; 7]) {
		let p = Pose { p: V3::new(h2i[0], h2i[1], h2i[2]), q: Q4::new(h2i[3], h2i[4], h2i[5], h2i[6]) };
		if near(&p, &Pose::default(), 1e-9, 1e-9) {
			return; // the runtime has not read the device's config
		}
		self.lock().runtime_h2i.insert(serial.to_vec(), p);
	}

	/// OnSallyLost
	/// (`since`: the time of the last runtime pose the loss was decided on; one received after it keeps the device)
	pub fn on_sally_lost(&self, serial: &[u8], now: f64, since: f64) {
		let mut st = self.lock();
		let Some(s) = st.sally.get_mut(serial) else { return };
		if s.t_recv > since {
			return;
		}
		s.lost = true;
		drop(self.push_if_quiet(serial, now, st));
	}

	/// Tick: RunFrame's commands, the fit, status
	pub fn tick(&self, now: f64) {
		if !self.active() {
			return;
		}
		self.commands();
		if self.fit_ok() && self.seen() == 0 {
			self.check_shown(now);
		}
		let fit_due = {
			let mut f = self.frame();
			let due = now - f.last_fit >= 1.0;
			if due {
				f.last_fit = now;
			}
			due
		};
		if self.dongle_check.swap(false, SeqCst) || fit_due {
			self.dongles(now);
		}
		if fit_due {
			self.fit(now);
			self.status(now);
		}
		let log_due = {
			let mut f = self.frame();
			let due = now - f.last_log >= 10.0;
			if due {
				f.last_log = now;
			}
			due
		};
		if log_due {
			self.log_stats(now);
		}
	}

	/// Install(ctx): TrackedDevicePoseUpdated replaced in each IVRServerDriverHost version vrserver offers, once per
	/// vtable
	fn install(&self, ctx: *mut IVRDriverContext) {
		let dets: [PoseUpdatedFn; K_VERSIONS] = [detour::<0>, detour::<1>, detour::<2>];
		let mut hooks = self.hooks.lock().unwrap_or_else(|e| e.into_inner());
		for i in 0..K_VERSIONS {
			// SAFETY: ctx is the context vrserver handed to Init, which calls this
			let (iface, err) = unsafe { get_generic_interface(ctx, K_NAMES[i], VRInitError_None) };
			if iface.is_null() || err != VRInitError_None {
				log("sally: %s not offered by this SteamVR\n", &[BArg::S(K_NAMES[i].to_bytes())]);
				continue;
			}
			// SAFETY: an interface object starts with its vtable pointer; slot 1 is TrackedDevicePoseUpdated (the second
			// virtual of IVRServerDriverHost 004..006)
			let slot = unsafe { (*(iface as *mut *mut *mut c_void)).add(1) };
			let shared = (0..i).any(|j| hooks.slot[j] == slot as usize);
			if shared {
				log(
					"sally: %s shares its vtable with a newer version, hooked once\n",
					&[BArg::S(K_NAMES[i].to_bytes())],
				);
				continue;
			}
			// SAFETY: the slot is readable, a function of the PoseUpdatedFn type
			self.orig[i].store(unsafe { slot.read_volatile() } as usize, SeqCst);
			hooks.detour[i] = dets[i] as usize;
			if !write_slot(slot, dets[i] as *mut c_void) {
				log("sally: could not hook %s (memory protection)\n", &[BArg::S(K_NAMES[i].to_bytes())]);
				self.orig[i].store(0, SeqCst);
				hooks.detour[i] = 0;
				continue;
			}
			hooks.slot[i] = slot as usize;
			self.hooked.fetch_add(1, SeqCst);
			log("sally: hooked %s::TrackedDevicePoseUpdated\n", &[BArg::S(K_NAMES[i].to_bytes())]);
		}
		if self.hooked.load(SeqCst) == 0 {
			log("sally: NO pose hook installed; SteamVR shows Valve's tracking only\n", &[]);
		}
	}

	/// Identify: the device's tracking system and serial (property reads outside our lock: they call into vrserver)
	fn identify(&self, id: u32, now: f64) {
		let c = tracked_device_to_property_container(id);
		let (sys, e1) = get_string_property(c, Prop_TrackingSystemName_String);
		let (serial, e2) = get_string_property(c, Prop_SerialNumber_String);
		let ok = e1 == TrackedProp_Success && e2 == TrackedProp_Success;
		let target = ok && sys == b"lighthouse" && serial.starts_with(b"LHR-");
		let mut st = self.lock();
		let d = st.devs.entry(id).or_default();
		d.checked = ok;
		d.next_check = now + if ok { 2.0 } else { 0.25 };
		if target && !d.target {
			d.target = true;
			d.serial = serial.clone();
			d.hmd = id == k_unTrackedDeviceIndex_Hmd;
			log("sally: Valve device %u = %s (%s)\n", &[BArg::U(id as u64), BArg::S(&serial), BArg::S(&sys)]);
			// its dongle at the next frame
			self.dongle_check.store(true, SeqCst);
		}
	}

	/// Dongles: each of Valve's devices' dongle (Prop_ConnectedWirelessDongle_String) read again, as a device can be
	/// paired to another one while SteamVR runs; a change is logged and sent to the runtime at once (`pairs`). The
	/// reads are outside the lock (they call into vrserver)
	fn dongles(&self, now: f64) {
		let ids: Vec<u32> = self.lock().devs.iter().filter(|(_, d)| d.target).map(|(&id, _)| id).collect();
		let read: Vec<(u32, Option<Vec<u8>>)> =
			ids.into_iter().map(|id| (id, read_dongle(tracked_device_to_property_container(id)))).collect();
		let mut st = self.lock();
		for (id, dongle) in read {
			// a read that fails keeps what the last one said
			let (Some(d), Some(dongle)) = (st.devs.get_mut(&id), dongle) else { continue };
			if dongle == d.dongle {
				continue;
			}
			if dongle.is_empty() {
				log(
					"sally: %s: Valve's driver names no dongle for it now (was %s)\n",
					&[BArg::S(&d.serial), BArg::S(&d.dongle)],
				);
			} else {
				log("sally: %s is on dongle %s\n", &[BArg::S(&d.serial), BArg::S(&dongle)]);
			}
			d.dongle = dongle;
			self.pairs_changed.store(true, SeqCst);
		}
		compare_dongles(&mut st, now);
	}

	/// RuntimeDongle: the runtime has the dongle (its device list), and `serial`'s config on it (None: no device known
	/// on it yet)
	pub fn runtime_dongle(&self, dongle: &[u8], serial: Option<&[u8]>) {
		if dongle.is_empty() || dongle == b"-" {
			return;
		}
		let mut st = self.lock();
		if !st.known_dongles.contains(dongle) {
			st.known_dongles.insert(dongle.to_vec());
		}
		if let Some(serial) = serial {
			if st.runtime_dongles.get(serial).map(Vec::as_slice) != Some(dongle) {
				st.runtime_dongles.insert(serial.to_vec(), dongle.to_vec());
			}
		}
	}

	/// Pairs: the runtime protocol's `pair <dongle> <serial>` for each dongle one of Valve's devices is on, as Valve's
	/// driver says (the devices it has reported connected within the last 1.5 s; the headset only with driver.hmd). A
	/// dongle two of them name is left out until one goes (Valve's driver is moving a device over).
	///
	/// The headset is on a cable and Valve's driver names no dongle for it: it is named on its own serial, which is what
	/// the runtime has a device on a cable under, so that the runtime takes its config as it takes any device's (it has
	/// none of its own for a headset it has not tracked before, and nothing else tells it whose to take). Another
	/// device with no dongle (a tracker or a controller on a cable) is not named: SteamVR keeps Valve's pose for it
	pub fn pairs(&self, now: f64) -> Vec<Vec<u8>> {
		let o = g_opt();
		let st = self.lock();
		let mut by: BTreeMap<&[u8], Option<&[u8]>> = BTreeMap::new();
		for d in st.devs.values() {
			let connected = d.last.deviceIsConnected != 0 && now - d.last_call < 1.5;
			let dongle = if d.hmd && d.dongle.is_empty() { &d.serial } else { &d.dongle };
			if !d.target
				|| (d.hmd && !(o.hmd && self.headset.load(SeqCst)))
				|| !connected
				|| !word(dongle)
				|| !word(&d.serial)
			{
				continue;
			}
			by.entry(dongle).and_modify(|e| *e = None).or_insert(Some(&d.serial));
		}
		by.into_iter()
			.filter_map(|(dongle, serial)| {
				serial.map(|serial| bsprintf("#p pair %s %s", &[BArg::S(dongle), BArg::S(serial)]))
			})
			.collect()
	}

	/// Whether a dongle changed since the last call (the receive thread sends the pairs then)
	pub fn take_pairs_changed(&self) -> bool {
		self.pairs_changed.swap(false, SeqCst)
	}

	/// Export: Valve's head pose in the frame's driver space (`ValveDev::head_f`: its own driver space while the device is
	/// on the frame's world-from-driver) -> panels (at most every 4 ms per device, and on every validity change): what
	/// the V line says, taken under m_ and formatted once SteamVR has its pose and m_ is let go (`VLine::text`)
	pub fn export(&self, d: &mut ValveDev, now: f64, fit_ok: bool) -> Option<VLine> {
		if self.out.load(SeqCst) == SOCK_BAD || self.outs.read().unwrap_or_else(|e| e.into_inner()).is_empty() {
			return None;
		}
		let v = d.valid as i32;
		if now - d.last_export < 0.004 && v == d.last_exported_valid {
			return None;
		}
		d.last_export = now;
		d.last_exported_valid = v;
		Some(VLine {
			serial: d.serial.clone(),
			wall: wall_seconds(),
			valid: v,
			head: d.head_f,
			shown: self.seen() == 0 && fit_ok,
		})
	}

	/// CheckShown: what SteamVR itself has for the devices we replace (GetRawTrackedDevicePoses): a jump away from what
	/// we sent, or a device it shows as not tracking although we sent it tracking, means something other than our
	/// pose reaches it
	fn check_shown(&self, now: f64) {
		let o = g_opt();
		let mut poses = [TrackedDevicePose_t::default(); k_unMaxTrackedDeviceCount as usize];
		let mut ahead = [TrackedDevicePose_t::default(); k_unMaxTrackedDeviceCount as usize];
		get_raw_tracked_device_poses(0.0, &mut poses);
		let plog = !o.pose_log.is_empty() && self.pose_log_on.load(SeqCst);
		if plog {
			get_raw_tracked_device_poses(K_POSE_LOG_AHEAD as f32, &mut ahead);
		}
		let mut rows = Vec::new();
		{
			let mut st = self.lock();
			if plog {
				pose_log_collect(&st, now, &mut rows);
			}
			for (&id, d) in st.devs.iter_mut() {
				if !d.target || id >= k_unMaxTrackedDeviceCount || now - d.sent_t > 0.1 {
					d.have_prev = false;
					continue;
				}
				let r = &poses[id as usize];
				if r.bDeviceIsConnected == 0 {
					d.have_prev = false;
					continue;
				}
				if r.bPoseIsValid == 0 || r.eTrackingResult != TrackingResult_Running_OK {
					d.shown_invalid += 1;
					d.have_prev = false;
					if now - d.shown_log_t > 1.0 {
						d.shown_log_t = now;
						log(
							"sally: SteamVR shows %s as not tracking (result %d) although we sent it tracking (Valve \
							 %s)\n",
							&[
								BArg::S(&d.serial),
								BArg::I(r.eTrackingResult as i64),
								BArg::S(if d.valid { b"valid" } else { b"not tracking" }),
							],
						);
					}
					continue;
				}
				let m = &r.mDeviceToAbsoluteTracking.m;
				let shown = V3::new(m[0][3] as f64, m[1][3] as f64, m[2][3] as f64);
				// SteamVR's pose is ours with Space Calibrator's offset on top, a rotation as well (lighthouse vs Quest
				// space), so the two are compared by how far each moved since the last frame, which no fixed rotation
				// changes
				if d.have_prev {
					let jump = (len(shown - d.prev_shown) - len(d.sent_w - d.prev_sent)).abs();
					if jump > 0.2 {
						d.shown_jumps += 1;
						if now - d.shown_log_t > 1.0 {
							d.shown_log_t = now;
							log(
								"sally: SteamVR moved %s %.0f mm more than the pose we sent did (Valve %s, %.0f mm from \
								 ours in its driver space)\n",
								&[
									BArg::S(&d.serial),
									BArg::F(jump * 1e3),
									BArg::S(if d.valid { b"valid" } else { b"not tracking" }),
									BArg::F(len(d.head.p - d.sent_head) * 1e3),
								],
							);
						}
					}
				}
				d.prev_shown = shown;
				d.prev_sent = d.sent_w;
				d.have_prev = true;
			}
		}
		// formatted and written outside the lock: Valve's pose threads wait for it (the flush is file I/O)
		if plog {
			self.pose_log(&o, now, &rows, &poses, &ahead);
		} else if !o.pose_log.is_empty() {
			self.pose_log_rest();
		}
	}

	/// The pose log switched off while a session's file is open: what it holds goes out, so the file ends on a whole
	/// row; switched on again in the session, the rows go on in the same file
	fn pose_log_rest(&self) {
		let mut f = self.frame();
		if f.plog_flush != 0.0 {
			f.plog_flush = 0.0;
			if let Some(plog) = f.plog.as_mut() {
				plog.flush();
			}
		}
	}

	/// PoseLog (driver.pose_log): per frame and replaced device what we last sent (Valve's driver space), what SteamVR
	/// has for it now and K_POSE_LOG_AHEAD s ahead (its world, Space Calibrator included), and Valve's own last pose.
	/// Answers whether SteamVR's prediction uses vecAcceleration and how Valve's velocity / acceleration / age compare.
	fn pose_log(
		&self, o: &Options, now: f64, rows: &[PlogRow], p0: &[TrackedDevicePose_t], p1: &[TrackedDevicePose_t],
	) {
		let mut f = self.frame();
		if f.plog.is_none() && !f.plog_failed {
			let path = expand_time(o.pose_log.clone(), time_stamp);
			f.plog = PoseFile::open(&path);
			let Some(plog) = f.plog.as_mut() else {
				f.plog_failed = true;
				log("sally: pose log: cannot write %s\n", &[BArg::S(&path)]);
				return;
			};
			log("sally: pose log: %s\n", &[BArg::S(&path)]);
			plog.write(
				b"t,id,serial,sent_age,sx,sy,sz,svx,svy,svz,sax,say,saz,spto,\
				  w0x,w0y,w0z,w1x,w1y,w1z,v0x,v0y,v0z,ok,\
				  valve_age,vx,vy,vz,vvx,vvy,vvz,vax,vay,vaz,vpto,vvalid,\
				  sqw,sqx,sqy,sqz,swx,swy,swz,vqw,vqx,vqy,vqz,vwx,vwy,vwz,vhqw,vhqx,vhqy,vhqz,vhx,vhy,vhz,\
				  m00,m01,m02,m10,m11,m12,m20,m21,m22,a00,a01,a02,a10,a11,a12,a20,a21,a22,\
				  vfqw,vfqx,vfqy,vfqz,vfx,vfy,vfz,sfqw,sfqx,sfqy,sfqz,sfx,sfy,sfz\n",
			);
		}
		let Some(plog) = f.plog.as_mut() else { return };
		for d in rows {
			let (a, b) = (&p0[d.id as usize], &p1[d.id as usize]);
			let (s, v) = (&d.s, &d.v);
			let am = &a.mDeviceToAbsoluteTracking.m;
			let bm = &b.mDeviceToAbsoluteTracking.m;
			plog.write(&bsprintf(
				"%.6f,%u,%s,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,\
				 %.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%d,\
				 %.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%d,",
				&[
					BArg::F(now),
					BArg::U(d.id as u64),
					BArg::S(&d.serial),
					BArg::F(d.sent_age),
					BArg::F(s.vecPosition[0]),
					BArg::F(s.vecPosition[1]),
					BArg::F(s.vecPosition[2]),
					BArg::F(s.vecVelocity[0]),
					BArg::F(s.vecVelocity[1]),
					BArg::F(s.vecVelocity[2]),
					BArg::F(s.vecAcceleration[0]),
					BArg::F(s.vecAcceleration[1]),
					BArg::F(s.vecAcceleration[2]),
					BArg::F(s.poseTimeOffset),
					BArg::F(am[0][3] as f64),
					BArg::F(am[1][3] as f64),
					BArg::F(am[2][3] as f64),
					BArg::F(bm[0][3] as f64),
					BArg::F(bm[1][3] as f64),
					BArg::F(bm[2][3] as f64),
					BArg::F(a.vVelocity.v[0] as f64),
					BArg::F(a.vVelocity.v[1] as f64),
					BArg::F(a.vVelocity.v[2] as f64),
					BArg::I((a.bPoseIsValid != 0 && b.bPoseIsValid != 0) as i64),
					BArg::F(d.valve_age),
					BArg::F(v.vecPosition[0]),
					BArg::F(v.vecPosition[1]),
					BArg::F(v.vecPosition[2]),
					BArg::F(v.vecVelocity[0]),
					BArg::F(v.vecVelocity[1]),
					BArg::F(v.vecVelocity[2]),
					BArg::F(v.vecAcceleration[0]),
					BArg::F(v.vecAcceleration[1]),
					BArg::F(v.vecAcceleration[2]),
					BArg::F(v.poseTimeOffset),
					BArg::I(d.valid as i64),
				],
			));
			// rotations: ours as sent, Valve's driver pose and its driver-from-head, SteamVR's now and K_POSE_LOG_AHEAD
			// ahead
			plog.write(&bsprintf(
				"%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f",
				&[
					BArg::F(s.qRotation.w),
					BArg::F(s.qRotation.x),
					BArg::F(s.qRotation.y),
					BArg::F(s.qRotation.z),
					BArg::F(s.vecAngularVelocity[0]),
					BArg::F(s.vecAngularVelocity[1]),
					BArg::F(s.vecAngularVelocity[2]),
					BArg::F(v.qRotation.w),
					BArg::F(v.qRotation.x),
					BArg::F(v.qRotation.y),
					BArg::F(v.qRotation.z),
					BArg::F(v.vecAngularVelocity[0]),
					BArg::F(v.vecAngularVelocity[1]),
					BArg::F(v.vecAngularVelocity[2]),
					BArg::F(v.qDriverFromHeadRotation.w),
					BArg::F(v.qDriverFromHeadRotation.x),
					BArg::F(v.qDriverFromHeadRotation.y),
					BArg::F(v.qDriverFromHeadRotation.z),
					BArg::F(v.vecDriverFromHeadTranslation[0]),
					BArg::F(v.vecDriverFromHeadTranslation[1]),
					BArg::F(v.vecDriverFromHeadTranslation[2]),
				],
			));
			for x in [am, bm] {
				for row in x.iter().take(3) {
					for &e in row.iter().take(3) {
						plog.write(&bsprintf(",%.6f", &[BArg::F(e as f64)]));
					}
				}
			}
			// the world-from-driver transforms: Valve's for the device (its own driver space's), and the one we sent
			// (the frame's, follow_frame)
			for x in [v, s] {
				let (q, t) = (&x.qWorldFromDriverRotation, &x.vecWorldFromDriverTranslation);
				plog.write(&bsprintf(
					",%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f",
					&[
						BArg::F(q.w),
						BArg::F(q.x),
						BArg::F(q.y),
						BArg::F(q.z),
						BArg::F(t[0]),
						BArg::F(t[1]),
						BArg::F(t[2]),
					],
				));
			}
			plog.write(b"\n");
		}
		if now - f.plog_flush > 1.0 {
			f.plog_flush = now;
			if let Some(plog) = f.plog.as_mut() {
				plog.flush();
			}
		}
	}

	/// PushIfQuiet: the runtime's pose of serial handed to SteamVR by us when Valve's driver has gone quiet on the
	/// device. m_ is let go for the call into vrserver and taken again, as the C++'s unique_lock is
	fn push_if_quiet<'a>(&'a self, serial: &[u8], now: f64, mut st: MutexGuard<'a, State>) -> MutexGuard<'a, State> {
		if self.passthrough.load(SeqCst) || self.seen() != 0 || !self.fit_ok() {
			return st;
		}
		let o = g_opt();
		let mut push: Option<(usize, u32, i32, DriverPose_t, Pose)> = None;
		{
			let State { devs, sally, t, runtime_dongles, dongles_agree, frame_wfd, .. } = &mut *st;
			let Some(si) = sally.get(serial) else { return st };
			for (&id, d) in devs.iter_mut() {
				if !d.target || d.serial != serial || d.this == 0 || d.ver < 0 || now - d.last_push < 0.004 {
					continue;
				}
				// not the device's pose (on_pose_locked's check); or a device Valve's driver last said is disconnected:
				// what the runtime still gets on its dongle is another device's (paired to it in the meantime)
				if (d.hmd && !(o.hmd && self.headset.load(SeqCst)))
					|| !same_device(d, runtime_dongles, *dongles_agree)
					|| d.last.deviceIsConnected == 0
				{
					continue;
				}
				// quiet for 30 ms: Valve has stopped reporting the device, sally's pose (or her loss) is pushed. Quiet
				// for push_quiet_ms while SteamVR shows sally steadily (no slide in progress) and she has the device:
				// Valve has paused (its accelerometer at full scale), her pose goes on being handed over
				let quiet = now - d.last_call;
				let short_quiet = o.push_quiet_ms > 0.0
					&& quiet >= o.push_quiet_ms * 1e-3
					&& d.source == 1
					&& d.blend >= 1.0
					&& !si.lost;
				if quiet < 0.03 && !short_quiet {
					continue;
				}
				let mut out = DriverPose_t::default();
				let last = d.last;
				substitute(&o, t, si, &last, frame_wfd.as_ref(), now, &mut out, d);
				if si.lost {
					// Valve is quiet, so there is nothing to fall back to: a device sally has lost shows as lost (it
					// showed sally's last pose as tracking, frozen, until the runtime found it again), and one Valve
					// last reported disconnected stays disconnected
					out.poseIsValid = 0;
					out.result = TrackingResult_Running_OutOfRange;
					out.deviceIsConnected = d.last.deviceIsConnected;
					for i in 0..3 {
						out.vecVelocity[i] = 0.0;
						out.vecAngularVelocity[i] = 0.0;
						out.vecAcceleration[i] = 0.0;
					}
					d.sent = out;
				}
				d.last_push = now;
				d.pushed += 1;
				// the driver-from-head of Valve's last pose of the device, with the device's offset (on_pose_locked's)
				let h = pose_of(&last.qDriverFromHeadRotation, &last.vecDriverFromHeadTranslation);
				let dfh = match &si.shown.offset {
					Some(off) => compose(&h, off),
					None => h,
				};
				push = Some((d.this, id, d.ver, out, dfh));
				break;
			}
		}
		let Some((this, id, ver, mut out, dfh)) = push else { return st };
		if self.body_on.load(SeqCst) {
			// with the body's move (body.rs), as every pose that goes to SteamVR
			if let Some(m) = st.body.at(now) {
				m.apply(&mut out);
				if let Some(d) = st.devs.get_mut(&id) {
					let wu = pose_of(&out.qWorldFromDriverRotation, &out.vecWorldFromDriverTranslation);
					d.sent_w = xform(&wu, V3::new(out.vecPosition[0], out.vecPosition[1], out.vecPosition[2]));
				}
			}
		}
		if o.valve_record {
			valve_record(&mut out, &dfh);
		}
		let orig = self.orig(ver as usize);
		drop(st);
		{
			let _guard = Inflight::new(&self.inflight);
			if !self.passthrough.load(SeqCst) {
				if let Some(orig) = orig {
					// SAFETY: the interface and the slot's function Valve's driver last called through, with a pose of
					// ours that lives through the call
					unsafe { orig(this as *mut c_void, id, &out, POSE_SIZE) };
				}
			}
		}
		self.lock()
	}

	/// Fit: the frame fit over the last fit_window_s of pairs, eased in. After a "refit" (the panel's "Re-align to
	/// Valve") the pairs start over, SteamVR keeps the fit it has until the new pairs make one (150 that pin it, as
	/// the first), and that one is taken as it is.
	///
	/// A fit is made of pairs that pin it: 150 of them (~15 s of play: a first fit from a few pairs can be centimetres
	/// off) whose points lie FIT_OFF_LINE_M off their own main line, however many devices they are of. Two devices
	/// carried about the room give that, and so does one; devices standing in a row do not, as many as they are (any
	/// turn about the row fits them). Without it the fit stays as it is, and before the first one SteamVR keeps
	/// Valve's tracking
	fn fit(&self, now: f64) {
		let o = g_opt();
		let mut a: Vec<V3> = Vec::new();
		let mut b: Vec<V3> = Vec::new();
		let devices;
		let gen;
		{
			let mut st = self.lock();
			if self.refit_req.swap(false, SeqCst) {
				st.samples.clear();
				st.refit_jump = true;
				log(
					"sally: re-align asked: the frame fit starts over from new pairs (SteamVR keeps the present one until \
					 they make one)\n",
					&[],
				);
			}
			gen = st.frame_gen;
			while st.samples.front().is_some_and(|x| x.t < now - o.fit_window_s) {
				st.samples.pop_front();
			}
			let mut per: BTreeMap<u32, i32> = BTreeMap::new();
			for x in &st.samples {
				a.push(x.src);
				b.push(x.dst);
				*per.entry(x.id).or_default() += 1;
			}
			devices = per.len() as i32;
		}
		if (a.len() as i32) < 150 || !off_one_line(&a, FIT_OFF_LINE_M) {
			return;
		}
		let Some((tf, med, kept)) = sally_math::math3d::fit_rigid_robust(&a, &b, 0.03) else { return };
		let mut st = self.lock();
		if st.frame_gen != gen {
			return; // the frame was re-based meanwhile: the pairs it came from are elsewhere now
		}
		let tf = if o.level {
			let mut c = V3::new(0.0, 0.0, 0.0);
			for x in &a {
				c += *x;
			}
			level(&mut st, tf, c * (1.0 / a.len() as f64), now)
		} else {
			tf
		};
		st.fit_n = a.len() as i32;
		st.fit_kept = kept;
		st.fit_med = med;
		if med < 0.05 {
			if !self.fit_ok() {
				log(
					"sally: frame fit ready (%d pairs from %d devices, median %.1f mm): SteamVR shows %s\n",
					&[
						BArg::I(st.fit_n as i64),
						BArg::I(devices as i64),
						BArg::F(med * 1e3),
						BArg::S(if self.seen() == 0 { b"sally" } else { b"Valve" }),
					],
				);
				st.t = tf;
				st.refit_jump = false;
			} else if st.refit_jump {
				log(
					"sally: re-aligned (%d pairs from %d devices, median %.1f mm): the frame moved %.0f mm / %.1f deg\n",
					&[
						BArg::I(st.fit_n as i64),
						BArg::I(devices as i64),
						BArg::F(med * 1e3),
						BArg::F(len(tf.p - st.t.p) * 1e3),
						BArg::F(qangle(qmul(tf.q, qconj(st.t.q))) * 57.2958),
					],
				);
				st.t = tf;
				st.refit_jump = false;
			} else {
				// refits move the whole body; ease small changes in (a fifth per second), jump on big ones (Valve's
				// own frame moved: its driver re-solved the base stations)
				let dp = len(tf.p - st.t.p);
				let da = qangle(qmul(tf.q, qconj(st.t.q)));
				if dp > 0.03 || da > 0.035 {
					log(
						"sally: frame moved %.0f mm / %.1f deg, following it\n",
						&[BArg::F(dp * 1e3), BArg::F(da * 57.2958)],
					);
					st.t = tf;
				} else {
					let mut dq = qmul(tf.q, qconj(st.t.q));
					if dq.w < 0.0 {
						dq = Q4::new(-dq.w, -dq.x, -dq.y, -dq.z);
					}
					let k = 0.2;
					let step = qnorm(Q4::new(1.0 + k * (dq.w - 1.0), k * dq.x, k * dq.y, k * dq.z));
					st.t.q = qnorm(qmul(step, st.t.q));
					st.t.p = st.t.p + (tf.p - st.t.p) * k;
				}
			}
			self.fit_ok.store(true, SeqCst);
		}
	}

	/// Commands: "seen sally" / "seen valve" / "seen toggle", "refit", "output [<scope> <name> <value>]" (the output
	/// layer's settings, answered with its O lines first; output.rs) (anything else: just the status), each answered
	/// with the status line, at most 16 a frame
	fn commands(&self) {
		let cmd = self.cmd.load(SeqCst);
		if cmd == SOCK_BAD {
			return;
		}
		let mut buf = [0u8; 512];
		for _k in 0..16 {
			let (n, from, fl) = net::recvfrom(cmd, &mut buf[..511]);
			if n <= 0 {
				return;
			}
			buf[n as usize] = 0;
			let c = crate::helpers::trim(crate::helpers::c_str(&buf));
			let before = self.seen();
			let f = crate::helpers::split(c);
			if f.first() == Some(&&b"output"[..]) {
				// the output layer's settings (output.rs): its O lines, then the status as for every command
				for l in crate::output::command(&f[1..]) {
					net::sendto(cmd, &l, &from, fl);
				}
			} else if c == b"seen sally" {
				self.seen.store(0, SeqCst);
			} else if c == b"seen valve" {
				self.seen.store(1, SeqCst);
			} else if c == b"seen toggle" {
				self.seen.store(1 - self.seen(), SeqCst);
			} else if c == b"refit" {
				self.refit_req.store(true, SeqCst);
			} else if c == b"headset sally" || c == b"headset valve" {
				// whose pose a lighthouse headset gets (the panel's Headset tracking), kept for the sessions to come
				let sally = c == b"headset sally";
				if self.headset.swap(sally, SeqCst) != sally {
					crate::user::set(|u| u.headset_sally = sally);
					log("sally: the headset now gets %s's pose\n", &[BArg::S(if sally { b"sally" } else { b"Valve" })]);
				}
			} else if c == b"pose_log on" || c == b"pose_log off" {
				// whether the pose log is written (the panel's Pose log): from this frame on, and kept for the sessions
				// to come
				let on = c == b"pose_log on";
				if self.pose_log_on.swap(on, SeqCst) != on {
					crate::user::set(|u| u.pose_log = Some(on));
					log("sally: the pose log is %s\n", &[BArg::S(if on { b"on" } else { b"off" })]);
				}
			} else if f.len() == 2 && f[0] == b"keep" {
				// how many sessions' recordings and logs stay (the panel's Recordings), from SteamVR's next start
				let n = sally_math::c_atoi(f[1]).max(0);
				crate::user::set(|u| u.keep_sessions = Some(n));
				log("sally: the newest %d sessions' files stay from the next start (0: all)\n", &[BArg::I(n as i64)]);
			}
			if self.seen() != before {
				log(
					"sally: SteamVR now shows %s's poses\n",
					&[BArg::S(if self.seen() == 0 { b"sally" } else { b"Valve" })],
				);
			}
			let st = self.status_line(now_seconds());
			net::sendto(cmd, &st, &from, fl);
		}
	}

	/// StatusLine: "C ..." (in char line[256]: cut to 255 bytes)
	pub fn status_line(&self, now: f64) -> Vec<u8> {
		let st = self.lock();
		let valve = st.devs.values().filter(|d| d.target && now - d.last_call < 1.0).count() as i32;
		let ours = st.sally.values().filter(|s| s.have && !s.lost).count() as i32;
		// the panel's choices as they stand: whose pose a lighthouse headset gets (and whether there is one), how many
		// sessions' files stay, whether the pose log is written (none: the settings name no file for it)
		let o = g_opt();
		let headset = o.hmd && self.headset.load(SeqCst);
		let lighthouse_headset = st.devs.values().any(|d| d.target && d.hmd);
		let keep = crate::user::get().keep_sessions.unwrap_or(o.keep_sessions);
		let pose_log: &[u8] = match (o.pose_log.is_empty(), self.pose_log_on.load(SeqCst)) {
			(true, _) => b"none",
			(false, true) => b"on",
			(false, false) => b"off",
		};
		let mut line = bsprintf(
			"C seen %s fit %d median_mm %.1f pairs %d hooks %d valve_devices %d sally_devices %d headset %s \
			 lighthouse_headset %d keep_sessions %d pose_log %s\n",
			&[
				BArg::S(if self.seen() == 0 { b"sally" } else { b"valve" }),
				BArg::I(self.fit_ok() as i64),
				BArg::F(st.fit_med * 1e3),
				BArg::I(st.fit_n as i64),
				BArg::I(self.hooked.load(SeqCst) as i64),
				BArg::I(valve as i64),
				BArg::I(ours as i64),
				BArg::S(if headset { b"sally" } else { b"valve" }),
				BArg::I(lighthouse_headset as i64),
				BArg::I(keep as i64),
				BArg::S(pose_log),
			],
		);
		line.truncate(255);
		line
	}

	/// FrameLine: "F ..." for the panels, after each fit: whether the fit is ready, the pairs it has now, whether a
	/// re-align is under way, the fit (T_: the runtime's world, OpenVR's axes, to the frame's driver space) and the
	/// frame's world-from-driver (0 and the identity without one), positions in metres, rotations w x y z. The panel
	/// shows Valve's devices through the fit SteamVR shows sally's through. The world-from-driver is Valve's own: what
	/// a driver hooked in after this one adds to it (Space Calibrator) is not in it, so it does not say where
	/// OpenVR's raw universe is, and the panel measures that itself
	pub fn frame_line(&self) -> Vec<u8> {
		let st = self.lock();
		let (t, f) = (st.t, st.frame_wfd);
		let w = f.unwrap_or_default();
		bsprintf(
			"F %d %d %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f\n",
			&[
				BArg::I(self.fit_ok() as i64),
				BArg::I(st.samples.len() as i64),
				BArg::I((st.refit_jump || self.refit_req.load(SeqCst)) as i64),
				BArg::F(t.p.x),
				BArg::F(t.p.y),
				BArg::F(t.p.z),
				BArg::F(t.q.w),
				BArg::F(t.q.x),
				BArg::F(t.q.y),
				BArg::F(t.q.z),
				BArg::I(f.is_some() as i64),
				BArg::F(w.p.x),
				BArg::F(w.p.y),
				BArg::F(w.p.z),
				BArg::F(w.q.w),
				BArg::F(w.q.x),
				BArg::F(w.q.y),
				BArg::F(w.q.z),
			],
		)
	}

	/// Status: the status line and the frame line to the panels
	fn status(&self, now: f64) {
		let out = self.out.load(SeqCst);
		if out == SOCK_BAD {
			return;
		}
		let st = self.status_line(now);
		let fl = self.frame_line();
		for a in self.outs.read().unwrap_or_else(|e| e.into_inner()).iter() {
			net::sendto(out, &st, a, SOCKADDR_IN_LEN);
			net::sendto(out, &fl, a, SOCKADDR_IN_LEN);
		}
	}

	/// LogStats: per device what Valve sent and what SteamVR was shown, per second since the last stats
	/// (the lines made under m_ and logged once it is let go: vrserver's log is not waited for by the pose hooks)
	fn log_stats(&self, now: f64) {
		let mut st = self.lock();
		let mut lines: Vec<Vec<u8>> = Vec::new();
		let dt = if st.last_stats_t > 0.0 { now - st.last_stats_t } else { 0.0 };
		st.last_stats_t = now;
		for d in st.devs.values_mut() {
			if !d.target {
				continue;
			}
			if dt > 0.0 {
				lines.push(bsprintf(
					"sally: %s Valve %.0f poses/s, shown sally %.0f/s, Valve %.0f/s, pushed %.0f/s, fell back %llu; SteamVR \
					 off our pose %llu, not tracking %llu; on another world-from-driver %llu; Valve %.0f mm / %.1f deg from ours, \
					 plausible for %.1f s\n",
					&[
						BArg::S(&d.serial),
						BArg::F(d.calls as f64 / dt),
						BArg::F(d.shown_sally as f64 / dt),
						BArg::F(d.shown_valve as f64 / dt),
						BArg::F(d.pushed as f64 / dt),
						BArg::U(d.fell_back),
						BArg::U(d.shown_jumps),
						BArg::U(d.shown_invalid),
						BArg::U(d.wfd_other),
						BArg::F(d.valve_disagree * 1e3),
						BArg::F(d.valve_disagree_deg),
						BArg::F(if d.valve_ok_since < 0.0 { 0.0 } else { now - d.valve_ok_since }),
					],
				));
			}
			d.calls = 0;
			d.shown_sally = 0;
			d.shown_valve = 0;
			d.pushed = 0;
			d.shown_jumps = 0;
			d.shown_invalid = 0;
			d.wfd_other = 0;
			d.fell_back = 0;
		}
		lines.push(bsprintf(
			"sally: seen %s, fit %s (%d pairs, %d kept, median %.1f mm)\n",
			&[
				BArg::S(if self.seen() == 0 { b"sally" } else { b"Valve" }),
				BArg::S(if self.fit_ok() { b"ready" } else { b"not ready" }),
				BArg::I(st.fit_n as i64),
				BArg::I(st.fit_kept as i64),
				BArg::F(st.fit_med * 1e3),
			],
		));
		drop(st);
		for mut line in lines {
			line.truncate(1399);
			crate::openvr::driver_log(&line);
		}
	}
}

/// A V line's values (Export)
pub struct VLine {
	pub serial: Vec<u8>,
	pub wall: f64,
	pub valid: i32,
	pub head: Pose,
	pub shown: bool,
}

impl VLine {
	/// The line; None where it would not fit the C++'s char exp[256] (not sent)
	pub fn text(&self) -> Option<Vec<u8>> {
		let h = &self.head;
		let line = bsprintf(
			"V %s %.6f %d %.5f %.5f %.5f %.6f %.6f %.6f %.6f %d\n",
			&[
				BArg::S(&self.serial),
				BArg::F(self.wall),
				BArg::I(self.valid as i64),
				BArg::F(h.p.x),
				BArg::F(h.p.y),
				BArg::F(h.p.z),
				BArg::F(h.q.w),
				BArg::F(h.q.x),
				BArg::F(h.q.y),
				BArg::F(h.q.z),
				BArg::I(self.shown as i64),
			],
		);
		(!line.is_empty() && line.len() < 256).then_some(line)
	}
}

/// What the pose log needs from each replaced device, copied under the lock
pub struct PlogRow {
	pub id: u32,
	pub serial: Vec<u8>,
	pub sent_age: f64,
	pub valve_age: f64,
	pub s: DriverPose_t,
	pub v: DriverPose_t,
	pub valid: bool,
}

fn pose_log_collect(st: &State, now: f64, rows: &mut Vec<PlogRow>) {
	for (&id, d) in &st.devs {
		if !d.target || id >= k_unMaxTrackedDeviceCount || now - d.sent_t > 0.1 {
			continue;
		}
		rows.push(PlogRow {
			id,
			serial: d.serial.clone(),
			sent_age: now - d.sent_t,
			valve_age: now - d.last_call,
			s: d.sent,
			v: d.last,
			valid: d.valid,
		});
	}
}

/// CompareDongles: whether Valve's driver and the runtime name dongles alike, from what each has said so far (logged
/// when they come to; if they never do, once after 30 s)
fn compare_dongles(st: &mut State, now: f64) {
	let valve: Vec<&Vec<u8>> =
		st.devs.values().filter(|d| d.target && !d.dongle.is_empty()).map(|d| &d.dongle).collect();
	let agree = valve.iter().any(|v| st.known_dongles.contains(*v));
	if agree && !st.dongles_agree {
		log(
			"sally: Valve's driver and the runtime name the dongles alike: a device has sally's pose only while both put \
			 it on the same dongle\n",
			&[],
		);
	}
	let (first_valve, first_runtime) = (valve.first().map(|v| v.to_vec()), st.known_dongles.iter().next().cloned());
	if let (false, Some(v), Some(r)) = (agree, first_valve, first_runtime) {
		if st.disagree_since == 0.0 {
			st.disagree_since = now;
		}
		if now - st.disagree_since > 30.0 && !st.disagree_logged {
			log(
				"sally: none of Valve's dongles (%s, ...) is one of the runtime's (%s, ...): devices are matched by serial \
				 alone\n",
				&[BArg::S(&v), BArg::S(&r)],
			);
			st.disagree_logged = true;
		}
	} else {
		st.disagree_since = 0.0;
	}
	st.dongles_agree = agree;
}

/// Whether two transforms are within `dp` metres and `da` radians of each other (quaternions as they come, which need
/// not be unit to the last bits: a transform is near itself whatever its quaternion's length)
fn near(a: &Pose, b: &Pose, dp: f64, da: f64) -> bool {
	if a == b {
		return true;
	}
	let n = |q: &Q4| (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
	let dot = (a.q.w * b.q.w + a.q.x * b.q.x + a.q.y * b.q.y + a.q.z * b.q.z).abs();
	len(a.p - b.p) <= dp && dot >= (0.5 * da).cos() * n(&a.q) * n(&b.q)
}

/// FollowFrame: which of Valve's world-from-driver transforms sally's poses go out on (`State::frame_wfd`, the
/// frame's), after a pose of Valve's for device `id`: its world-from-driver `w`, its head in its driver space at `head`
/// moving at `vel`.
///
/// Valve's driver has more than one. A device it boots or re-boots it tracks in a driver space of its own for tens of
/// seconds, with a world-from-driver of its own that puts it where it is (findings.md, "Keep Valve's last good
/// world-from-driver": metres and over 100 deg from the others', for a minute or more). Its pose in that driver space
/// is metres from the others', and sally's pose, mapped into the others' by the frame fit and sent out on that device's
/// own world-from-driver, would show it metres from where it is. So every device's pose goes out on the frame's
/// world-from-driver, and each of Valve's poses is read into the frame's driver space through its own
/// (`ValveDev::head_f`): its pairs, its V lines, the fallback's checks. What SteamVR shows of a device is then the
/// frame's world-from-driver on the fit of the pairs, which is Valve's world, whatever driver space Valve has it in.
///
/// The frame's moves with Valve's world. When Valve moves its world (Space Calibrator, a new room setup) every device's
/// world-from-driver changes in the same update and its pose in the driver space stays; SteamVR's view moves, the
/// headset's with it, and sally's poses move along: a device's change is such a move when its pose in Valve's driver
/// space went on as it was across it (no step beyond its own motion), and the frame follows once most of its devices
/// have moved alike; small changes (under SAME_M / SAME_RAD) as they come. Where most devices are on another
/// world-from-driver without having moved so (Valve's driver space put elsewhere with the world kept, devices booted
/// elsewhere before the frame fit), the frame is re-based onto it: the fit and its pairs go with it, and nothing
/// SteamVR shows moves. The frame's is the first valid pose's to begin with.
#[allow(clippy::too_many_arguments)]
fn follow_frame(st: &mut State, id: u32, w: &Pose, head: V3, vel: V3, valid: bool, now: f64) {
	const SAME_M: f64 = 0.002;
	const SAME_RAD: f64 = 0.002;
	let Some(d) = st.devs.get_mut(&id) else { return };
	if d.have_wfd && !near(&d.wfd, w, 0.0005, 0.0005) {
		let dt = now - d.wfd_t;
		let steady = d.wfd_valid && valid && dt < 0.05 && len(head - d.wfd_head) < 0.005 + 2.0 * len(vel) * dt;
		d.world_move = if steady { Some((d.wfd, *w, now)) } else { None };
	}
	(d.wfd, d.have_wfd, d.wfd_head, d.wfd_t, d.wfd_valid) = (*w, true, head, now, valid);
	if !valid {
		return;
	}
	let Some(f) = st.frame_wfd else {
		st.frame_wfd = Some(*w);
		log("sally: the frame: Valve's world-from-driver of %s\n", &[BArg::S(&st.devs[&id].serial)]);
		return;
	};
	if near(&f, w, SAME_M, SAME_RAD) {
		if f != *w {
			st.frame_wfd = Some(*w);
		}
		return;
	}
	let recent = |d: &ValveDev| d.target && d.have_wfd && d.wfd_valid && now - d.wfd_t < 0.5;
	let n_recent = st.devs.values().filter(|d| recent(d)).count();
	// Valve's world moved
	if let Some((from, to, _)) =
		st.devs[&id].world_move.filter(|(a, b, _)| near(a, &f, SAME_M, SAME_RAD) && near(b, w, SAME_M, SAME_RAD))
	{
		let moved = |d: &ValveDev| {
			d.world_move.is_some_and(|(a, b, t)| {
				now - t < 0.5 && near(&a, &from, SAME_M, SAME_RAD) && near(&b, &to, SAME_M, SAME_RAD)
			})
		};
		let n_move = st.devs.values().filter(|d| recent(d) && moved(d)).count();
		let n_frame =
			st.devs.values().filter(|d| recent(d) && (moved(d) || near(&d.wfd, &f, SAME_M, SAME_RAD))).count();
		if n_move >= n_frame.min(2) && 2 * n_move > n_frame {
			st.frame_wfd = Some(to);
			for d in st.devs.values_mut() {
				d.world_move = None;
			}
			log(
				"sally: Valve's world moved %.0f mm / %.1f deg (%d of the frame's %d devices, their poses in its driver \
				 space as they were): following it\n",
				&[
					BArg::F(len(to.p - f.p) * 1e3),
					BArg::F(qangle(qmul(to.q, qconj(f.q))) * 57.2958),
					BArg::I(n_move as i64),
					BArg::I(n_frame as i64),
				],
			);
			return;
		}
	}
	// most devices on another world-from-driver: re-based onto it (SteamVR's view the same: frame' * t' = frame * t)
	let on = |x: &Pose| st.devs.values().filter(|d| recent(d) && near(&d.wfd, x, SAME_M, SAME_RAD)).count();
	let (on_w, on_f) = (on(w), on(&f));
	if on_w >= n_recent.min(2)
		&& on_w > on_f
		&& st.devs.values().all(|d| d.world_move.is_none_or(|(_, _, t)| now - t >= 0.5))
	{
		let m = compose(&inverse(w), &f);
		st.t = compose(&m, &st.t);
		for x in st.samples.iter_mut() {
			x.dst = xform(&m, x.dst);
		}
		// and what each device keeps in the frame's driver space: its last head (the fallback's jump test) and its last
		// pair's place
		for d in st.devs.values_mut() {
			d.prev_head = xform(&m, d.prev_head);
			d.last_sample = xform(&m, d.last_sample);
		}
		st.frame_gen += 1;
		st.frame_wfd = Some(*w);
		log(
			"sally: the frame: most of Valve's devices are on another world-from-driver (%d against %d, %.0f mm / %.1f \
			 deg from it): re-based onto it, nothing shown moves\n",
			&[
				BArg::I(on_w as i64),
				BArg::I(on_f as i64),
				BArg::F(len(w.p - f.p) * 1e3),
				BArg::F(qangle(qmul(w.q, qconj(f.q))) * 57.2958),
			],
		);
	}
}

/// UpdateHeadFix: a device's HeadFix from Valve's driver-from-head `dfh` (at each of its poses; worked out again only
/// when it or the runtime's head-to-IMU changed), logged when it turns or moves the head
fn update_head_fix(st: &mut State, id: u32, dfh: &Pose) {
	let State { devs, runtime_h2i, head_fix, .. } = st;
	let Some(serial) = devs.get(&id).map(|d| d.serial.as_slice()) else { return };
	let Some(&h2i) = runtime_h2i.get(serial) else { return };
	if head_fix.get(serial).is_some_and(|h| h.h2i == h2i && h.dfh == *dfh) {
		return;
	}
	if near(dfh, &Pose::default(), 1e-9, 1e-9) {
		// no head offset from Valve's driver (not from its config): nothing to go by
		head_fix.remove(serial);
		return;
	}
	let mut c = compose(&inverse(&h2i), dfh);
	if c.q.w < 0.0 {
		c.q = Q4::new(-c.q.w, -c.q.x, -c.q.y, -c.q.z);
	}
	let deg = qangle(c.q) * 57.2958;
	let apply = len(c.p) <= K_HEAD_FIX_M;
	if !apply {
		log(
			"sally: %s head frame: Valve's driver-from-head puts the head %.1f mm from the runtime's head point: not the \
			 runtime's IMU, the runtime's head frame kept\n",
			&[BArg::S(serial), BArg::F(len(c.p) * 1e3)],
		);
	} else if deg > 1.0 || len(c.p) > 0.001 {
		let s = (1.0 - c.q.w * c.q.w).max(0.0).sqrt();
		let ax = if s > 1e-9 { V3::new(c.q.x / s, c.q.y / s, c.q.z / s) } else { V3::default() };
		log(
			"sally: %s head frame: Valve's is turned %.1f deg about %.2f %.2f %.2f and %.1f mm off (%.1f %.1f %.1f mm, \
			 the runtime's head frame) from the runtime's: SteamVR gets the runtime's pose in Valve's\n",
			&[
				BArg::S(serial),
				BArg::F(deg),
				BArg::F(ax.x),
				BArg::F(ax.y),
				BArg::F(ax.z),
				BArg::F(len(c.p) * 1e3),
				BArg::F(c.p.x * 1e3),
				BArg::F(c.p.y * 1e3),
				BArg::F(c.p.z * 1e3),
			],
		);
	}
	head_fix.insert(serial.to_vec(), HeadFix { h2i, dfh: *dfh, c, apply });
}

/// HeadFixed: the runtime's pose of a device (its head pose, Y-up sally world) in Valve's head frame: head * c, the
/// head point's velocity and acceleration carried to c's origin (+ w x r, + alpha x r + w x (w x r): r is 1.5 mm on a
/// Tundra), the body rate and its change in c's axes. A pose without a velocity (driver.velocity 0) keeps none
fn head_fixed(pose: &DriverPose_t, c: &Pose) -> DriverPose_t {
	let mut o = *pose;
	let q = Q4::new(pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z);
	let r = qrot(q, c.p);
	let w_body = V3::new(pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]);
	let al_body =
		V3::new(pose.vecAngularAcceleration[0], pose.vecAngularAcceleration[1], pose.vecAngularAcceleration[2]);
	let (w, al) = (qrot(q, w_body), qrot(q, al_body));
	let dv = cross(w, r);
	let da = cross(al, r) + cross(w, dv);
	let qi = qconj(c.q);
	let (wb, alb) = (qrot(qi, w_body), qrot(qi, al_body));
	let moving = pose.vecVelocity != [0.0; 3];
	o.qRotation = hmd_quat(qnorm(qmul(q, c.q)));
	for i in 0..3 {
		o.vecPosition[i] += r.get(i);
		if moving {
			o.vecVelocity[i] += dv.get(i);
			o.vecAcceleration[i] += da.get(i);
		}
		o.vecAngularVelocity[i] = wb.get(i);
		o.vecAngularAcceleration[i] = alb.get(i);
	}
	o
}

/// ValveRecord: a pose sally built (the head pose, no driver-from-head) on Valve's own record: the driver pose at the
/// origin `dfh` puts the head from (Valve's IMU, in the IMU's axes), `dfh` as its driver-from-head. Driver pose *
/// driver-from-head is the head pose as it was, so SteamVR and apps are shown the same pose; a reader of the driver
/// pose alone (Space Calibrator skips the driver-from-head) sees the device where Valve's own pose has it, and no
/// turn when SteamVR goes from one's pose to the other's. The origin's velocity is the head point's carried to it
/// (v + w x r), the body rate is in the IMU's axes. The accelerations stay as they are: every pose of sally's has
/// none (`substitute`), as Valve's own, and SteamVR's prediction reads none. SteamVR carries the driver pose by its
/// velocity and turns it by its rate, the head on top: the head then goes on the arc the turn gives it about the
/// origin where it went straight, the same to first order
pub fn valve_record(o: &mut DriverPose_t, dfh: &Pose) {
	let qh = Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z);
	let qi = qnorm(qmul(qh, qconj(dfh.q)));
	// from the head point to the origin, driver space
	let r = qrot(qi, dfh.p) * -1.0;
	let w_head = V3::new(o.vecAngularVelocity[0], o.vecAngularVelocity[1], o.vecAngularVelocity[2]);
	let dv = cross(qrot(qh, w_head), r);
	let w_imu = qrot(dfh.q, w_head);
	let al_imu =
		qrot(dfh.q, V3::new(o.vecAngularAcceleration[0], o.vecAngularAcceleration[1], o.vecAngularAcceleration[2]));
	let moving = o.vecVelocity != [0.0; 3];
	o.qRotation = hmd_quat(qi);
	for i in 0..3 {
		o.vecPosition[i] += r.get(i);
		if moving {
			o.vecVelocity[i] += dv.get(i);
		}
		o.vecAngularVelocity[i] = w_imu.get(i);
		o.vecAngularAcceleration[i] = al_imu.get(i);
	}
	o.qDriverFromHeadRotation = hmd_quat(dfh.q);
	o.vecDriverFromHeadTranslation = [dfh.p.x, dfh.p.y, dfh.p.z];
}

/// SameDevice: whether the runtime's pose of a device's serial is the device's: unless Valve's driver and the runtime
/// both say which dongle it is on and differ (the runtime has the serial's config on a dongle that carries another
/// device). Where either says nothing, or the two do not name dongles alike, it is taken on its serial as before
pub fn same_device(d: &ValveDev, runtime_dongles: &BTreeMap<Vec<u8>, Vec<u8>>, agree: bool) -> bool {
	if !agree || d.dongle.is_empty() {
		return true;
	}
	match runtime_dongles.get(&d.serial) {
		Some(r) if !r.is_empty() && r.as_slice() != b"-" => *r == d.dongle,
		_ => true,
	}
}

/// What changed in `same_device`'s answer for a device, logged
fn log_identity(d: &ValveDev, runtime_dongles: &BTreeMap<Vec<u8>, Vec<u8>>) {
	let rt: &[u8] = runtime_dongles.get(&d.serial).map_or(b"-", Vec::as_slice);
	if d.ours {
		log("sally: %s: the runtime has it on dongle %s too\n", &[BArg::S(&d.serial), BArg::S(rt)]);
	} else if d.hmd {
		log(
			"sally: %s is the headset: Valve's pose (the panel's Headset tracking, or driver.hmd=0)\n",
			&[BArg::S(&d.serial)],
		);
	} else {
		log(
			"sally: %s is on dongle %s, the runtime has it on %s: Valve's pose until they agree\n",
			&[BArg::S(&d.serial), BArg::S(&d.dongle), BArg::S(rt)],
		);
	}
}

/// A device's dongle as Valve's driver names it (empty: none), None when the property does not read
fn read_dongle(c: PropertyContainerHandle_t) -> Option<Vec<u8>> {
	let (v, e) = get_string_property(c, Prop_ConnectedWirelessDongle_String);
	(e == TrackedProp_Success).then_some(v)
}

/// A word the runtime protocol takes as one argument
fn word(b: &[u8]) -> bool {
	!b.is_empty() && b.len() < 64 && b.iter().all(|c| c.is_ascii_graphic())
}

/// Substitute: the runtime's pose in the frame's driver space, on Valve's pose record (its flags kept), with the
/// frame's world-from-driver (`frame`: follow_frame's; Valve's own without one); `t` is T_, the fit
#[allow(clippy::too_many_arguments)]
pub fn substitute(
	opt: &Options, t: &Pose, s: &SallyLatest, valve: &DriverPose_t, frame: Option<&Pose>, now: f64,
	o: &mut DriverPose_t, d: &mut ValveDev,
) {
	*o = *valve;
	if let Some(f) = frame {
		o.qWorldFromDriverRotation = hmd_quat(f.q);
		for i in 0..3 {
			o.vecWorldFromDriverTranslation[i] = f.p.get(i);
		}
	}
	// what the toys show of the device (toys.rs: the report's kinematics reshaped) goes out in its own pose's place;
	// the fit's pairs and the fallback's checks are taken from its own
	let reshaped = s.shown.reshaped.map(|k| {
		let mut o = s.pose;
		k.put(&mut o);
		o
	});
	let sp = reshaped.as_ref().unwrap_or(&s.pose);
	let sv = pose_of(&sp.qRotation, &sp.vecPosition);
	let mut pp = compose(t, &sv);
	o.vecPosition = [pp.p.x, pp.p.y, pp.p.z];
	o.qRotation = hmd_quat(pp.q);
	o.qDriverFromHeadRotation = HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
	o.vecDriverFromHeadTranslation = [0.0; 3];
	let v = qrot(t.q, V3::new(sp.vecVelocity[0], sp.vecVelocity[1], sp.vecVelocity[2]));
	o.vecVelocity = [v.x, v.y, v.z];
	for i in 0..3 {
		o.vecAngularVelocity[i] = sp.vecAngularVelocity[i]; // device frame: unchanged by the frame fit
		o.vecAcceleration[i] = 0.0;
		o.vecAngularAcceleration[i] = 0.0;
	}
	o.poseTimeOffset = std_max(-0.25, sp.poseTimeOffset - (now - s.t_recv));
	if opt.carry && !s.lost {
		// the hand-over carry: the pose is `age` old; carry it to now as a rigid body with the head point's
		// acceleration (the IMU's, plus the turn of the lever arm, from the runtime) and the body rate, and declare it
		// current. SteamVR then extrapolates in a straight line only over its own prediction, not over our pipeline age
		// too (which shows as the overshoot of a shaken controller). Without an acceleration this is SteamVR's own
		// straight line. Constant acceleration holds for a few ms only: a pose no new report followed (the radio
		// dropped some, the runtime is gone) is carried less, not at all after 16 ms without a report (they come every
		// 4 ms), and is then handed over exactly as without the carry (carried over a growing age, the slide onto
		// Valve's pose after the runtime stops steps by centimetres). What is not carried is declared as the pose's
		// age, so SteamVR's straight line takes it on
		let fresh = std_min(1.0, std_max(0.0, 1.0 - ((now - s.t_recv) - 0.006) / 0.010));
		let age_all = std_max(0.0, -o.poseTimeOffset);
		let age = std_min(0.05, age_all) * fresh;
		let a = qrot(t.q, V3::new(sp.vecAcceleration[0], sp.vecAcceleration[1], sp.vecAcceleration[2]));
		for i in 0..3 {
			o.vecPosition[i] += o.vecVelocity[i] * age + 0.5 * a.get(i) * age * age;
			o.vecVelocity[i] += a.get(i) * age;
		}
		// the orientation: the body rate over the age and its change by the angular acceleration (device frame, on the
		// right; the acceleration is 0 on trackers and from a runtime that sends none), and the rate at the end of it
		// goes to SteamVR, whose straight line then starts from the turn's own rate
		let w = V3::new(sp.vecAngularVelocity[0], sp.vecAngularVelocity[1], sp.vecAngularVelocity[2]);
		let mut al = V3::new(sp.vecAngularAcceleration[0], sp.vecAngularAcceleration[1], sp.vecAngularAcceleration[2]);
		let change = len(al) * age;
		let rate = len(w);
		if change > rate {
			// as in update: the change over the carry no larger than the rate it starts from
			al = al * (if change > 0.0 { rate / change } else { 0.0 });
		}
		let rv = w * age + al * (0.5 * age * age);
		let th = len(rv);
		if th > 1e-9 {
			let ax = rv * (1.0 / th);
			let (sh, ch) = sin_cos(0.5 * th);
			let dq = Q4::new(ch, ax.x * sh, ax.y * sh, ax.z * sh);
			let q = qnorm(qmul(Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z), dq));
			o.qRotation = hmd_quat(q);
		}
		for i in 0..3 {
			o.vecAngularVelocity[i] = w.get(i) + al.get(i) * age;
		}
		o.poseTimeOffset = -(age_all - age);
		pp.p = V3::new(o.vecPosition[0], o.vecPosition[1], o.vecPosition[2]);
	}
	// the output layer (output.rs), after the carry: where the device is shown against where it is, in its own axes
	// (the pose moved and turned as a rigid body: the offset point's velocity, the body rate in the turned axes), the
	// velocity SteamVR is given scaled, and SteamVR's own prediction cut
	if let Some(off) = &s.shown.offset {
		let q = Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z);
		let w_body = V3::new(o.vecAngularVelocity[0], o.vecAngularVelocity[1], o.vecAngularVelocity[2]);
		let r = qrot(q, off.p);
		let dv = cross(qrot(q, w_body), r);
		let wb = qrot(qconj(off.q), w_body);
		o.qRotation = hmd_quat(qnorm(qmul(q, off.q)));
		for i in 0..3 {
			o.vecPosition[i] += r.get(i);
			o.vecVelocity[i] += dv.get(i);
			o.vecAngularVelocity[i] = wb.get(i);
		}
		pp.p = V3::new(o.vecPosition[0], o.vecPosition[1], o.vecPosition[2]);
	}
	if s.shown.velocity_scale != 1.0 {
		for v in o.vecVelocity.iter_mut() {
			*v *= s.shown.velocity_scale;
		}
	}
	if s.shown.prediction != 1.0 {
		// the movement smoothing: SteamVR draws its straight line ahead along the velocity and the body rate (no
		// acceleration is handed on), so this much of each is this much of its prediction, the shake in them with it
		for i in 0..3 {
			o.vecVelocity[i] *= s.shown.prediction;
			o.vecAngularVelocity[i] *= s.shown.prediction;
		}
	}
	if s.lost && !opt.fallback {
		o.poseIsValid = 0;
		o.result = TrackingResult_Running_OutOfRange;
		for i in 0..3 {
			o.vecVelocity[i] = 0.0;
			o.vecAngularVelocity[i] = 0.0;
			o.vecAcceleration[i] = 0.0;
		}
	} else {
		o.poseIsValid = 1;
		o.result = TrackingResult_Running_OK;
	}
	o.deviceIsConnected = 1;
	let wu = pose_of(&o.qWorldFromDriverRotation, &o.vecWorldFromDriverTranslation);
	d.sent_w = xform(&wu, pp.p);
	d.sent_head = pp.p;
	d.sent_t = now;
	d.sent = *o;
}

/// Detour<N>: what vrserver's TrackedDevicePoseUpdated slot calls once hooked
///
/// # Safety
/// Called by vrserver as TrackedDevicePoseUpdated: `this` its interface, `p` a pose of `size` bytes
unsafe extern "C" fn detour<const N: usize>(this: *mut c_void, id: u32, p: *const DriverPose_t, size: u32) {
	// SAFETY: vrserver's arguments, as on_pose requires them
	unsafe { G_OVERRIDE.on_pose(N, this, id, p, size) };
}

/// WriteSlot: one vtable slot written through the page's protection (the page made writable, then back)
pub fn write_slot(slot: *mut *mut c_void, value: *mut c_void) -> bool {
	sys::write_slot(slot, value)
}

#[cfg(unix)]
mod sys {
	use std::ffi::{c_int, c_void};

	const PROT_READ: c_int = 1;
	const PROT_WRITE: c_int = 2;
	const SC_PAGESIZE: c_int = 30;

	extern "C" {
		fn mprotect(addr: *mut c_void, len: usize, prot: c_int) -> c_int;
		fn sysconf(name: c_int) -> i64;
	}

	/// mprotect the slot's pages read-write, write, then read-only (vrserver's vtables are read-only data)
	pub fn write_slot(slot: *mut *mut c_void, value: *mut c_void) -> bool {
		// SAFETY: the pages are the slot's own, which the process maps; the slot is written only once they are
		// writable, and the value is a function of the slot's type
		unsafe {
			let page = sysconf(SC_PAGESIZE) as usize;
			let a = slot as usize & !(page - 1);
			let n = (slot as usize + std::mem::size_of::<*mut c_void>() - a).div_ceil(page) * page;
			if mprotect(a as *mut c_void, n, PROT_READ | PROT_WRITE) != 0 {
				return false;
			}
			slot.write_volatile(value);
			mprotect(a as *mut c_void, n, PROT_READ);
		}
		true
	}
}

#[cfg(windows)]
mod sys {
	use std::ffi::c_void;

	const PAGE_EXECUTE_READWRITE: u32 = 0x40;

	#[link(name = "kernel32")]
	extern "system" {
		fn VirtualProtect(addr: *mut c_void, size: usize, prot: u32, old: *mut u32) -> i32;
	}

	/// VirtualProtect the slot PAGE_EXECUTE_READWRITE, write, then its old protection
	pub fn write_slot(slot: *mut *mut c_void, value: *mut c_void) -> bool {
		let mut old: u32 = 0;
		// SAFETY: the slot is in vrserver's mapped vtable; it is written only once writable, and the value is a
		// function of the slot's type
		unsafe {
			if VirtualProtect(slot.cast(), std::mem::size_of::<*mut c_void>(), PAGE_EXECUTE_READWRITE, &mut old) == 0 {
				return false;
			}
			slot.write_volatile(value);
			let mut dummy: u32 = 0;
			VirtualProtect(slot.cast(), std::mem::size_of::<*mut c_void>(), old, &mut dummy);
		}
		true
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	const DONGLE: &[u8] = b"00A4AE2B50-RYB";
	const OTHER: &[u8] = b"A9DDD3B47A-LYM";
	const SERIAL: &[u8] = b"LHR-C57C5A45";

	fn quat1() -> HmdQuaternion_t {
		HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
	}

	/// A pose of Valve's: tracking, connected, at the origin
	fn valve_pose() -> DriverPose_t {
		DriverPose_t {
			qWorldFromDriverRotation: quat1(),
			qDriverFromHeadRotation: quat1(),
			qRotation: quat1(),
			result: TrackingResult_Running_OK,
			poseIsValid: 1,
			deviceIsConnected: 1,
			..DriverPose_t::default()
		}
	}

	/// A rotation of `ang` rad about the unit axis (x, y, z)
	fn rot(x: f64, y: f64, z: f64, ang: f64) -> Q4 {
		let (s, c) = ((ang / 2.0).sin(), (ang / 2.0).cos());
		Q4::new(c, x * s, y * s, z * s)
	}

	#[test]
	fn levelling_puts_the_runtimes_up_on_the_apps_up_with_a_headset_in_its_own_space() {
		let po = PoseOverride::new();
		let mut st = po.lock();
		// Space Calibrator's alignment, tilted 20 mrad, turned 1 rad, moved
		let wfd = Pose { p: V3::new(0.3, 0.1, -0.2), q: qmul(rot(1.0, 0.0, 0.0, 0.02), rot(0.0, 1.0, 0.0, 1.0)) };
		st.frame_wfd = Some(wfd);
		// a free fit 5 mrad off level in the apps' world
		let tf = Pose {
			p: V3::new(1.0, 0.5, 2.0),
			q: qmul(qconj(wfd.q), qmul(rot(0.0, 0.0, 1.0, 0.005), rot(0.0, 1.0, 0.0, 0.4))),
		};
		let c = V3::new(0.2, 1.0, -0.4);
		let l = level(&mut st, tf, c, 100.0);
		// the runtime's up shows as the apps' +Y
		let apps_up = qrot(wfd.q, qrot(l.q, V3::new(0.0, 1.0, 0.0)));
		assert!(len(apps_up - V3::new(0.0, 1.0, 0.0)) < 1e-9, "{apps_up:?}");
		// the pairs' centroid stays where the free fit put it, the heading too
		assert!(len(xform(&l, c) - xform(&tf, c)) < 1e-9);
		let fwd = |q: Q4| qrot(wfd.q, qrot(q, V3::new(0.0, 0.0, 1.0)));
		let (a, b) = (fwd(tf.q), fwd(l.q));
		assert!((a.z.atan2(a.x) - b.z.atan2(b.x)).abs() < 1e-4);
		// a runtime that is not level (40 deg) is left alone
		let bad = Pose { p: tf.p, q: qmul(rot(1.0, 0.0, 0.0, 0.7), tf.q) };
		let l2 = level(&mut st, bad, c, 101.0);
		assert!(qangle(qmul(l2.q, qconj(bad.q))) < 1e-12 && st.level_refused);
	}

	#[test]
	fn levelling_holds_a_lighthouse_headsets_room_setup_at_its_long_run_tilt() {
		let po = PoseOverride::new();
		let mut st = po.lock();
		let mut h = dev(SERIAL, DONGLE);
		h.hmd = true;
		st.devs.insert(k_unTrackedDeviceIndex_Hmd, h);
		st.frame_wfd = Some(Pose { p: V3::new(0.0, 0.0, 0.0), q: Q4::new(1.0, 0.0, 0.0, 0.0) });
		let c = V3::new(0.0, 1.0, 0.0);
		// free fits whose tilt wanders +-6 mrad about 10 mrad, one a second for 20 minutes
		let mut last = Pose::default();
		for i in 0..1200 {
			let wobble = 0.006 * (i as f64 * 0.05).sin();
			let tf = Pose {
				p: V3::new(0.0, 0.0, 0.0),
				q: qmul(rot(1.0, 0.0, 0.0, 0.010 + wobble), rot(0.0, 1.0, 0.0, 0.3)),
			};
			last = level(&mut st, tf, c, i as f64);
		}
		// the frame's up holds the room setup's 10 mrad, not the last fit's
		let u = qrot(last.q, V3::new(0.0, 1.0, 0.0));
		let tilt = u.z.atan2(u.y);
		assert!((tilt - 0.010).abs() < 0.0015, "tilt {tilt}");
	}

	fn dev(serial: &[u8], dongle: &[u8]) -> ValveDev {
		ValveDev {
			checked: true,
			target: true,
			serial: serial.to_vec(),
			dongle: dongle.to_vec(),
			..ValveDev::default()
		}
	}

	/// An override with the frame fit ready, Valve's device `id` (serial SERIAL on dongle DONGLE) and a fresh runtime
	/// pose of SERIAL 1 m up, the runtime having SERIAL on `runtime` (the two naming dongles alike)
	fn bench(id: u32, runtime: &[u8], now: f64) -> PoseOverride {
		let po = PoseOverride::new();
		po.fit_ok.store(true, SeqCst);
		{
			let mut st = po.lock();
			let mut d = dev(SERIAL, DONGLE);
			d.hmd = id == k_unTrackedDeviceIndex_Hmd;
			st.devs.insert(id, d);
			let pose = DriverPose_t { qRotation: quat1(), vecPosition: [0.0, 1.0, 0.0], ..DriverPose_t::default() };
			let latest = SallyLatest { have: true, lost: false, t_recv: now, pose, shown: Shown::default() };
			st.sally.insert(SERIAL.to_vec(), latest);
			st.known_dongles.extend([DONGLE.to_vec(), OTHER.to_vec()]);
			st.runtime_dongles.insert(SERIAL.to_vec(), runtime.to_vec());
			compare_dongles(&mut st, now);
			assert!(st.dongles_agree);
		}
		po
	}

	#[test]
	fn a_runtime_pose_is_the_devices_only_on_valves_dongle() {
		let mut rd = BTreeMap::new();
		let d = dev(SERIAL, DONGLE);
		// nothing to compare: taken on its serial
		assert!(same_device(&d, &rd, true));
		assert!(same_device(&dev(SERIAL, b""), &rd, true));
		rd.insert(SERIAL.to_vec(), b"-".to_vec());
		assert!(same_device(&d, &rd, true));
		rd.insert(SERIAL.to_vec(), DONGLE.to_vec());
		assert!(same_device(&d, &rd, true));
		rd.insert(SERIAL.to_vec(), OTHER.to_vec());
		assert!(!same_device(&d, &rd, true));
		// Valve's driver and the runtime not naming dongles alike: no check
		assert!(same_device(&d, &rd, false));
	}

	#[test]
	fn the_dongles_agree_once_one_of_valves_is_the_runtimes() {
		let mut st = PoseOverride::new().m.into_inner().unwrap();
		st.devs.insert(1, dev(SERIAL, b"00A4AE2B50"));
		st.known_dongles.insert(DONGLE.to_vec());
		compare_dongles(&mut st, 10.0);
		assert!(!st.dongles_agree);
		assert_eq!(st.disagree_since, 10.0);
		compare_dongles(&mut st, 41.0);
		assert!(st.disagree_logged);
		st.devs.insert(2, dev(b"LHR-9365E531", DONGLE));
		compare_dongles(&mut st, 42.0);
		assert!(st.dongles_agree);
		assert_eq!(st.disagree_since, 0.0);
	}

	#[test]
	fn steamvr_gets_valves_pose_while_the_runtime_has_the_device_on_another_dongle() {
		let now = 100.0;
		let p = valve_pose();
		// on Valve's dongle: the runtime's pose, 1 m up
		let po = bench(3, DONGLE, now);
		let (out, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		assert!(replace);
		assert_eq!(out.vecPosition, [0.0, 1.0, 0.0]);
		assert_eq!(po.lock().devs[&3].shown_sally, 1);
		// on another: Valve's, as it came
		let po = bench(3, OTHER, now);
		let (_, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		assert!(!replace);
		let st = po.lock();
		assert!(!st.devs[&3].ours);
		assert_eq!((st.devs[&3].shown_sally, st.devs[&3].shown_valve), (0, 1));
		drop(st);
		// the runtime has it on Valve's dongle again: the runtime's pose (blended in from Valve's)
		po.runtime_dongle(DONGLE, Some(SERIAL));
		let (out, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now + 0.01).unwrap();
		assert!(replace && po.lock().devs[&3].ours);
		assert!(out.vecPosition[1] > 0.0 && out.vecPosition[1] < 1.0);
	}

	/// The output layer's part of the hand-over: the device shown where its offset puts it, as a rigid body (the offset
	/// point's velocity, the body rate in the turned axes), the velocity SteamVR gets scaled, and with the movement
	/// smoothing the velocity and the body rate both cut to the share of its prediction SteamVR is left
	#[test]
	fn the_offset_moves_the_device_as_a_rigid_body_and_the_boost_scales_its_velocity() {
		let h = 0.5f64.sqrt();
		// the device turned 90 deg about y (its x along world -z, its z along world x), turning at 2 rad/s about its z
		let pose = DriverPose_t {
			qRotation: HmdQuaternion_t { w: h, x: 0.0, y: h, z: 0.0 },
			vecPosition: [1.0, 2.0, 3.0],
			vecVelocity: [0.1, 0.0, 0.0],
			vecAngularVelocity: [0.0, 0.0, 2.0],
			..DriverPose_t::default()
		};
		// shown 10 cm along its x, turned 90 deg about its z; the velocity half again
		let offset = Pose { p: V3::new(0.1, 0.0, 0.0), q: Q4::new(h, 0.0, 0.0, h) };
		let shown = Shown { velocity_scale: 1.5, offset: Some(offset), ..Shown::default() };
		let s = SallyLatest { have: true, lost: false, t_recv: 100.0, pose, shown };
		let (mut o, mut d) = (DriverPose_t::default(), ValveDev::default());
		substitute(&Options::default(), &Pose::default(), &s, &valve_pose(), None, 100.0, &mut o, &mut d);
		let near3 = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12);
		assert!(near3(o.vecPosition, [1.0, 2.0, 2.9]), "{:?}", o.vecPosition);
		// w (world) = 2 about x, r = -0.1 z: w x r = 0.2 y; with the device's own 0.1 x, times 1.5
		assert!(near3(o.vecVelocity, [0.15, 0.3, 0.0]), "{:?}", o.vecVelocity);
		assert!(near3(o.vecAngularVelocity, [0.0, 0.0, 2.0]), "{:?}", o.vecAngularVelocity);
		let want = qmul(Q4::new(h, 0.0, h, 0.0), offset.q);
		let q = o.qRotation;
		assert!((q.w - want.w).abs() < 1e-12 && (q.x - want.x).abs() < 1e-12 && (q.y - want.y).abs() < 1e-12);
		assert_eq!(d.sent_head, V3::new(o.vecPosition[0], o.vecPosition[1], o.vecPosition[2]));
		// SteamVR left half its prediction: the pose where it was, half the velocity and half the body rate to go on
		let s = SallyLatest { shown: Shown { prediction: 0.5, ..shown }, ..s };
		substitute(&Options::default(), &Pose::default(), &s, &valve_pose(), None, 100.0, &mut o, &mut d);
		assert!(near3(o.vecPosition, [1.0, 2.0, 2.9]), "{:?}", o.vecPosition);
		assert!(near3(o.vecVelocity, [0.075, 0.15, 0.0]), "{:?}", o.vecVelocity);
		assert!(near3(o.vecAngularVelocity, [0.0, 0.0, 1.0]), "{:?}", o.vecAngularVelocity);
		// nothing set: the pose as the runtime has it
		let s = SallyLatest { shown: Shown::default(), ..s };
		substitute(&Options::default(), &Pose::default(), &s, &valve_pose(), None, 100.0, &mut o, &mut d);
		assert_eq!((o.vecPosition, o.vecVelocity), ([1.0, 2.0, 3.0], [0.1, 0.0, 0.0]));
	}

	/// What the toys show of a device goes out in its own pose's place: through the fit to Valve's frame, carried by
	/// its own velocity (a held pose stays where it is)
	#[test]
	fn what_the_toys_show_goes_out_in_the_poses_place() {
		// the runtime's world a quarter turn about y from Valve's driver space (its x is the driver's -z), 1 m up
		let h = 0.5f64.sqrt();
		let fit = Pose { p: V3::new(0.0, 1.0, 0.0), q: Q4::new(h, 0.0, h, 0.0) };
		let pose = DriverPose_t {
			qRotation: quat1(),
			vecPosition: [0.3, 0.1, 0.0],
			vecVelocity: [4.0, 1.0, 0.0],
			poseTimeOffset: -0.01,
			..DriverPose_t::default()
		};
		// shown 0.6 m along the runtime's x going 8 m/s along it, a quarter turn about its z
		let kin = Kin {
			p: [0.6, 0.0, 0.0],
			q: [h, 0.0, 0.0, h],
			v: [8.0, 0.0, 0.0],
			a: [0.0; 3],
			w: [0.0; 3],
			al: [0.0; 3],
			age: 0.01,
		};
		let s = SallyLatest {
			have: true,
			lost: false,
			t_recv: 100.0,
			pose,
			shown: Shown { reshaped: Some(kin), ..Shown::default() },
		};
		let (mut o, mut d) = (DriverPose_t::default(), ValveDev::default());
		let near3 = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12);
		substitute(&Options::default(), &fit, &s, &valve_pose(), None, 100.0, &mut o, &mut d);
		assert!(near3(o.vecPosition, [0.0, 1.0, -0.6]), "{:?}", o.vecPosition);
		assert!(near3(o.vecVelocity, [0.0, 0.0, -8.0]), "{:?}", o.vecVelocity);
		let q = qmul(fit.q, Q4::new(h, 0.0, 0.0, h));
		let got = Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z);
		assert!(qangle(qmul(qconj(q), got)) < 1e-12);
		assert_eq!(d.sent_head, V3::new(o.vecPosition[0], o.vecPosition[1], o.vecPosition[2]));
		// the carry: the pose is 10 ms old and goes on by its own velocity, 8 cm
		let carry = Options { carry: true, ..Options::default() };
		substitute(&carry, &fit, &s, &valve_pose(), None, 100.0, &mut o, &mut d);
		assert!(near3(o.vecPosition, [0.0, 1.0, -0.68]), "{:?}", o.vecPosition);
		// a held pose (stop motion: no velocity) is where it is held, however old
		let held = Kin { v: [0.0; 3], ..kin };
		let s = SallyLatest { shown: Shown { reshaped: Some(held), ..Shown::default() }, ..s };
		substitute(&carry, &fit, &s, &valve_pose(), None, 100.004, &mut o, &mut d);
		assert!(near3(o.vecPosition, [0.0, 1.0, -0.6]), "{:?}", o.vecPosition);
		assert_eq!(o.vecVelocity, [0.0; 3]);
	}

	/// Dash punch's slide (body.rs): without one no pose is touched; with one, every device's pose goes out moved as
	/// one, on its world-from-driver, the pose in its driver space as it was; a hand's dash is taken into SteamVR's
	/// world
	#[test]
	fn the_dash_moves_every_pose_as_one() {
		let po = bench(3, DONGLE, 100.0);
		let h = 0.5f64.sqrt();
		let head = DriverPose_t { vecPosition: [0.0, 1.7, 0.0], ..valve_pose() };
		// a base station 1 m in front of the head (the world's -z), in the world's own axes
		let station = DriverPose_t { vecPosition: [1.0, 1.7, -1.0], ..valve_pose() };
		assert!(po.body_move(0, &head, 100.0).is_none() && po.body_move(7, &station, 100.0).is_none());
		assert!(!po.body_on.load(SeqCst));

		let t = 100.0;

		// dash punch: the hand's rail gives 0.4 m along the runtime's x; the fit turns that into the driver's -z and
		// the frame's world-from-driver (a half turn about y) into the world's +z
		po.fit_ok.store(true, SeqCst);
		{
			let mut st = po.lock();
			st.t = Pose { p: V3::new(0.0, 1.0, 0.0), q: Q4::new(h, 0.0, h, 0.0) };
			st.frame_wfd = Some(Pose { p: V3::new(0.0, 0.0, 0.0), q: Q4::new(0.0, 0.0, 1.0, 0.0) });
		}
		let dash = Dash { shift: [0.4, 0.0, 0.0], rate: [0.0; 3], back_s: 0.3 };
		for i in 0..100 {
			let shown = Shown { dash: Some(dash), ..Shown::default() };
			po.on_sally_pose(SERIAL, &valve_pose(), shown, t + 1.0 + i as f64 * 0.004);
		}
		assert!(po.body_on.load(SeqCst));
		let m = po.body_move(7, &station, t + 1.4).expect("slid");
		assert!((m.vecWorldFromDriverTranslation[2] - 0.4).abs() < 1e-3, "{:?}", m.vecWorldFromDriverTranslation);
		assert!(m.vecWorldFromDriverTranslation[0].abs() < 1e-9 && m.qWorldFromDriverRotation.w == 1.0);
		// the rail gives nothing any more: back, and then no move
		for i in 0..500 {
			po.on_sally_pose(SERIAL, &valve_pose(), Shown::default(), t + 1.4 + i as f64 * 0.004);
		}
		assert!(po.body_move(7, &station, t + 3.4).is_none() && !po.body_on.load(SeqCst));
	}

	/// A device with an offset falls back to Valve's pose with it (on Valve's driver-from-head), so the fallback does
	/// not move it by the offset; with Valve shown for everything (the panel's switch) Valve's pose is as it came
	#[test]
	fn the_fallback_keeps_a_devices_offset() {
		let now = 100.0;
		let p = valve_pose();
		let offset = Pose { p: V3::new(0.0, 0.05, 0.0), q: Q4::new(1.0, 0.0, 0.0, 0.0) };
		let po = bench(3, DONGLE, now);
		{
			let mut st = po.lock();
			let s = st.sally.get_mut(SERIAL).unwrap();
			s.shown.offset = Some(offset);
			s.lost = true; // sally has lost it: Valve's pose is the one to show
		}
		let (out, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		assert!(replace);
		assert_eq!((out.vecPosition, out.qRotation.w), (p.vecPosition, p.qRotation.w));
		let dfh = out.vecDriverFromHeadTranslation;
		assert!((dfh[1] - p.vecDriverFromHeadTranslation[1] - 0.05).abs() < 1e-12, "{dfh:?}");
		// Valve for everything: as it came
		po.seen.store(1, SeqCst);
		let (_, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now + 0.004).unwrap();
		assert!(!replace);
	}

	/// The head pose a record gives: the driver pose with its driver-from-head on top
	fn head_of(o: &DriverPose_t) -> Pose {
		compose(
			&pose_of(&o.qRotation, &o.vecPosition),
			&pose_of(&o.qDriverFromHeadRotation, &o.vecDriverFromHeadTranslation),
		)
	}

	/// Where SteamVR shows a record's head `dt` on: the driver pose carried by its velocity and turned by its body
	/// rate, the driver-from-head on top
	fn shown_ahead(o: &DriverPose_t, dt: f64) -> Pose {
		let w = V3::new(o.vecAngularVelocity[0], o.vecAngularVelocity[1], o.vecAngularVelocity[2]);
		let th = len(w) * dt;
		let ax = if th > 0.0 { w * (1.0 / len(w)) } else { V3::default() };
		let (sh, ch) = ((th / 2.0).sin(), (th / 2.0).cos());
		let mut c = *o;
		c.qRotation = hmd_quat(qmul(
			Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z),
			Q4::new(ch, ax.x * sh, ax.y * sh, ax.z * sh),
		));
		for i in 0..3 {
			c.vecPosition[i] += o.vecVelocity[i] * dt;
		}
		head_of(&c)
	}

	/// A pose of sally's: the head pose, no driver-from-head, moving and turning
	fn head_record() -> DriverPose_t {
		let q = qnorm(Q4::new(0.8, 0.1, -0.5, 0.3));
		DriverPose_t {
			qRotation: hmd_quat(q),
			vecPosition: [1.0, 2.0, 3.0],
			vecVelocity: [0.4, -0.2, 0.7],
			vecAngularVelocity: [1.5, -2.0, 0.5],
			..valve_pose()
		}
	}

	/// An Index controller's driver-from-head, about: 159 mm off and turned
	fn controller_dfh() -> Pose {
		Pose { p: V3::new(0.006, -0.015, 0.158), q: qnorm(Q4::new(0.2, 0.9, -0.1, 0.35)) }
	}

	/// On Valve's record the head is where it was, turned as it was, and moves as it did: the driver pose * the
	/// driver-from-head is the head pose, the origin's velocity carried back to the head point (v + w x r) is the head
	/// point's, and the body rate is the same turn in the origin's axes
	#[test]
	fn valves_record_shows_the_same_head() {
		let head = head_record();
		let dfh = controller_dfh();
		let mut o = head;
		valve_record(&mut o, &dfh);
		assert!(near(&head_of(&o), &head_of(&head), 1e-12, 1e-12));
		assert!(near(&pose_of(&o.qDriverFromHeadRotation, &o.vecDriverFromHeadTranslation), &dfh, 1e-15, 1e-15));
		let qd = Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z);
		let w = V3::new(o.vecAngularVelocity[0], o.vecAngularVelocity[1], o.vecAngularVelocity[2]);
		let head_v =
			V3::new(o.vecVelocity[0], o.vecVelocity[1], o.vecVelocity[2]) + cross(qrot(qd, w), qrot(qd, dfh.p));
		assert!(len(head_v - V3::new(0.4, -0.2, 0.7)) < 1e-12, "{head_v:?}");
		assert!(len(qrot(qconj(dfh.q), w) - V3::new(1.5, -2.0, 0.5)) < 1e-12);
		// the origin's velocity is what the origin does: where it is a moment on, the head carried as a rigid body
		let dt = 1e-6;
		let later = compose(&shown_ahead(&head, dt), &inverse(&dfh));
		let moved = (later.p - V3::new(o.vecPosition[0], o.vecPosition[1], o.vecPosition[2])) * (1.0 / dt);
		assert!(len(moved - V3::new(o.vecVelocity[0], o.vecVelocity[1], o.vecVelocity[2])) < 1e-5, "{moved:?}");
		// what SteamVR shows 40 ms on: the head on the arc its turn gives it about the origin where it went straight,
		// apart by 1/2 w^2 r t^2 at most (2.6 rad/s, 159 mm: under 1 mm), and turned the same
		let (a, b) = (shown_ahead(&head, 0.04), shown_ahead(&o, 0.04));
		let apart = len(a.p - b.p);
		assert!(apart > 1e-5 && apart < 0.5 * 6.5 * 0.16 * 0.04 * 0.04, "{apart}");
		assert!(qangle(qmul(a.q, qconj(b.q))) < 1e-12);
		// a pose without a velocity keeps none, and no driver-from-head leaves the pose as it is
		let mut still = DriverPose_t { vecVelocity: [0.0; 3], ..head };
		valve_record(&mut still, &dfh);
		assert_eq!(still.vecVelocity, [0.0; 3]);
		let mut same = head;
		valve_record(&mut same, &Pose::default());
		assert!(near(
			&pose_of(&same.qRotation, &same.vecPosition),
			&pose_of(&head.qRotation, &head.vecPosition),
			1e-15,
			1e-15
		));
		assert_eq!((same.vecVelocity, same.vecAngularVelocity), (head.vecVelocity, head.vecAngularVelocity));
	}

	/// sally's pose of a device goes out as Valve's own of it does: the driver pose Valve's driver gives for that head
	/// (head * driver-from-head^-1), so what reads the driver pose alone sees the same device under either
	#[test]
	fn valves_record_is_the_driver_pose_valves_driver_gives() {
		let dfh = controller_dfh();
		let valve_driver = Pose { p: V3::new(-0.3, 1.1, 0.4), q: qnorm(Q4::new(0.3, -0.6, 0.2, 0.7)) };
		let head = compose(&valve_driver, &dfh);
		let mut o =
			DriverPose_t { qRotation: hmd_quat(head.q), vecPosition: [head.p.x, head.p.y, head.p.z], ..valve_pose() };
		valve_record(&mut o, &dfh);
		assert!(near(&pose_of(&o.qRotation, &o.vecPosition), &valve_driver, 1e-12, 1e-12));
	}

	/// on_pose: a pose of sally's comes with the driver-from-head it goes out on, Valve's of that call with the device's
	/// offset as the fallback has it; Valve's own pose with none
	#[test]
	fn a_pose_of_sallys_goes_out_on_valves_driver_from_head() {
		let now = 100.0;
		let dfh = controller_dfh();
		let p = DriverPose_t {
			qDriverFromHeadRotation: hmd_quat(dfh.q),
			vecDriverFromHeadTranslation: [dfh.p.x, dfh.p.y, dfh.p.z],
			..valve_pose()
		};
		let offset = Pose { p: V3::new(0.0, 0.05, 0.0), q: Q4::new(1.0, 0.0, 0.0, 0.0) };
		let po = bench(3, DONGLE, now);
		po.fit_ok.store(true, SeqCst);
		let (out, replace, _, record) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		assert!(replace && po.lock().devs[&3].source == 1);
		assert!(near(&record.unwrap(), &dfh, 1e-15, 1e-15));
		// the pose itself is still the head pose here: it is put on the record as it leaves (on_pose)
		assert_eq!(out.vecDriverFromHeadTranslation, [0.0; 3]);
		po.lock().sally.get_mut(SERIAL).unwrap().shown.offset = Some(offset);
		let (_, _, _, record) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now + 0.004).unwrap();
		assert!(near(&record.unwrap(), &compose(&dfh, &offset), 1e-15, 1e-15));
		// Valve for everything: its own pose, as it came
		po.seen.store(1, SeqCst);
		po.lock().devs.get_mut(&3).unwrap().blend = 0.0;
		let (_, replace, _, record) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now + 0.008).unwrap();
		assert!(!replace && record.is_none());
	}

	/// A switch between the two is shown as the device moves: half way through the blend, with sally's pose and Valve's
	/// of the same turning device, SteamVR gets that head, turning and moving as both say. Valve's velocity is its
	/// IMU's, 159 mm from a controller's head: mixed in as the head's, the head was shown up to 0.2 m/s off its own
	#[test]
	fn mid_blend_the_head_moves_as_both_poses_say() {
		let now = 100.0;
		let dfh = controller_dfh();
		let head = head_record();
		// Valve's pose of it: the same head on Valve's record
		let mut p = head;
		valve_record(&mut p, &dfh);
		let po = bench(3, DONGLE, now);
		po.lock().sally.get_mut(SERIAL).unwrap().pose = head;
		po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		{
			let mut st = po.lock();
			st.sally.get_mut(SERIAL).unwrap().t_recv = now + 0.004;
			let d = st.devs.get_mut(&3).unwrap();
			(d.blend, d.blend_t) = (0.5, now + 0.004);
		}
		let (mut out, replace, _, record) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now + 0.004).unwrap();
		assert!(replace && po.lock().devs[&3].blend == 0.5);
		valve_record(&mut out, &record.unwrap());
		assert!(near(&head_of(&out), &head_of(&head), 1e-12, 1e-12));
		let qd = Q4::new(out.qRotation.w, out.qRotation.x, out.qRotation.y, out.qRotation.z);
		let w = V3::new(out.vecAngularVelocity[0], out.vecAngularVelocity[1], out.vecAngularVelocity[2]);
		let head_v =
			V3::new(out.vecVelocity[0], out.vecVelocity[1], out.vecVelocity[2]) + cross(qrot(qd, w), qrot(qd, dfh.p));
		assert!(len(head_v - V3::new(0.4, -0.2, 0.7)) < 1e-12, "{head_v:?}");
	}

	#[test]
	fn a_device_valve_says_is_disconnected_gets_no_pushes() {
		let now = 100.0;
		for connected in [1, 0] {
			let po = bench(3, DONGLE, now);
			{
				let mut st = po.lock();
				let d = st.devs.get_mut(&3).unwrap();
				// Valve's driver quiet on it for a second, its last pose connected or not
				(d.this, d.ver, d.last_call) = (1, 0, now - 1.0);
				d.last = valve_pose();
				d.last.deviceIsConnected = connected;
			}
			let pose = DriverPose_t { qRotation: quat1(), ..DriverPose_t::default() };
			po.on_sally_pose(SERIAL, &pose, Shown::default(), now);
			assert_eq!(po.lock().devs[&3].pushed, connected as u64);
		}
	}

	/// A lighthouse headset gets sally's pose like any device, and Valve's once the panel's Headset tracking says so:
	/// then the runtime is not asked for it either
	#[test]
	fn the_headset_keeps_valves_pose_when_that_is_chosen() {
		let now = 100.0;
		let po = bench(k_unTrackedDeviceIndex_Hmd, DONGLE, now);
		let shown = |po: &PoseOverride| {
			po.on_pose_locked(0, std::ptr::null_mut(), k_unTrackedDeviceIndex_Hmd, &valve_pose(), now).unwrap().1
		};
		let tracker = bench(3, DONGLE, now);
		let tracker_shown = tracker.on_pose_locked(0, std::ptr::null_mut(), 3, &valve_pose(), now).unwrap().1;
		assert_eq!(shown(&po), tracker_shown);
		assert_eq!(po.pairs(now), tracker.pairs(now));
		po.headset.store(false, SeqCst);
		assert!(!shown(&po));
		assert!(po.pairs(now).is_empty());
	}

	#[test]
	fn pairs_name_each_dongle_one_connected_device_is_on() {
		let now = 50.0;
		let po = PoseOverride::new();
		let connected = |serial: &[u8], dongle: &[u8], last_call: f64| {
			let mut d = dev(serial, dongle);
			d.last_call = last_call;
			d.last.deviceIsConnected = 1;
			d
		};
		{
			let mut st = po.lock();
			st.devs.insert(1, connected(SERIAL, DONGLE, now - 0.01));
			// gone quiet, disconnected, no dongle, a dongle no protocol word: not named
			st.devs.insert(2, connected(b"LHR-00000002", b"1B7A62914E-3YX", now - 2.0));
			let mut off = connected(b"LHR-00000003", b"2829C4CAEA-5YX", now);
			off.last.deviceIsConnected = 0;
			st.devs.insert(3, off);
			st.devs.insert(4, connected(b"LHR-00000004", b"", now));
			st.devs.insert(5, connected(b"LHR-00000005", b"0A0E4D6852 2YX", now));
			// two on one dongle (Valve's driver moving a device over): neither
			st.devs.insert(6, connected(b"LHR-00000006", b"4BCAD73D0E-6YX", now));
			st.devs.insert(7, connected(b"LHR-00000007", b"4BCAD73D0E-6YX", now));
			// not one of ours
			let mut other = connected(b"LHB-00000008", b"EBF4140A47-4YX", now);
			other.target = false;
			st.devs.insert(8, other);
		}
		assert_eq!(po.pairs(now), [b"#p pair 00A4AE2B50-RYB LHR-C57C5A45".to_vec()]);
		// the headset, on its cable with no dongle: named on its own serial (the runtime has it under that, and gets
		// its config by it), and not at all while the panel's Headset tracking says Valve's
		{
			let mut st = po.lock();
			let mut hmd = connected(b"LHR-00000009", b"", now);
			hmd.hmd = true;
			st.devs.insert(k_unTrackedDeviceIndex_Hmd, hmd);
		}
		assert_eq!(
			po.pairs(now),
			[b"#p pair 00A4AE2B50-RYB LHR-C57C5A45".to_vec(), b"#p pair LHR-00000009 LHR-00000009".to_vec()]
		);
		po.headset.store(false, SeqCst);
		assert_eq!(po.pairs(now), [b"#p pair 00A4AE2B50-RYB LHR-C57C5A45".to_vec()]);
	}

	fn hq(q: Q4) -> HmdQuaternion_t {
		HmdQuaternion_t { w: q.w, x: q.x, y: q.y, z: q.z }
	}

	fn about(ax: V3, deg: f64) -> Q4 {
		let (s, c) = ((0.5 * deg.to_radians()).sin(), (0.5 * deg.to_radians()).cos());
		Q4::new(c, ax.x * s, ax.y * s, ax.z * s)
	}

	/// A pose of Valve's for a head pose in the world `x`, on world-from-driver `w` (its driver pose w^-1 * x; no
	/// driver-from-head)
	fn valve_at(x: &Pose, w: &Pose) -> DriverPose_t {
		let d = compose(&inverse(w), x);
		DriverPose_t {
			qWorldFromDriverRotation: hq(w.q),
			vecWorldFromDriverTranslation: [w.p.x, w.p.y, w.p.z],
			qRotation: hq(d.q),
			vecPosition: [d.p.x, d.p.y, d.p.z],
			..valve_pose()
		}
	}

	/// An override with Valve's devices 1..=n (serials LHR-0000000k), none of them the runtime's
	fn devices(n: u32, fit_ok: bool) -> PoseOverride {
		let po = PoseOverride::new();
		po.fit_ok.store(fit_ok, SeqCst);
		let mut st = po.lock();
		for k in 1..=n {
			st.devs.insert(k, dev(format!("LHR-0000000{k}").as_bytes(), b""));
		}
		drop(st);
		po
	}

	fn at(k: u32) -> Pose {
		Pose { p: V3::new(0.3 * k as f64, 1.0, -0.2 * k as f64), q: about(V3::new(0.0, 1.0, 0.0), 20.0 * k as f64) }
	}

	fn moved() -> Pose {
		Pose { p: V3::new(0.06, -0.01, 0.05), q: about(V3::new(0.0, 1.0, 0.0), 1.0) }
	}

	#[test]
	fn a_device_valve_tracks_in_a_driver_space_of_its_own_goes_out_on_the_frames() {
		// a device booted on a world-from-driver 5.3 m / 151 deg from the others', its pose in the world where it is
		let now = 100.0;
		let po = bench(3, DONGLE, now);
		po.lock().devs.insert(4, dev(b"LHR-00000004", b""));
		let a = Pose { p: V3::new(2.8, -0.04, -3.15), q: about(V3::new(0.0, 1.0, 0.0), 112.7) };
		let b = Pose { p: V3::new(-2.34, 0.06, -2.08), q: about(V3::new(0.0, 1.0, 0.0), -100.9) };
		po.on_pose_locked(0, std::ptr::null_mut(), 4, &valve_at(&at(4), &a), now).unwrap();
		assert_eq!(po.lock().frame_wfd, Some(a));
		let x = compose(&a, &Pose { p: V3::new(0.1, 1.2, 0.3), q: Q4::default() });
		let (out, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &valve_at(&x, &b), now).unwrap();
		assert!(replace);
		// sally's pose (the fit maps it into the frame's driver space) on the frame's world-from-driver, not its own
		assert_eq!(pose_of(&out.qWorldFromDriverRotation, &out.vecWorldFromDriverTranslation), a);
		assert_eq!(out.vecPosition, [0.0, 1.0, 0.0]);
		// Valve's pose read into the frame's driver space: where it is, for the pairs and the V line
		let st = po.lock();
		let d = &st.devs[&3];
		assert!(len(d.head_f.p - V3::new(0.1, 1.2, 0.3)) < 1e-9, "{:?}", d.head_f);
		assert!(len(d.head.p - d.head_f.p) > 3.0);
		assert_eq!(d.wfd_other, 1);
	}

	#[test]
	fn the_frame_follows_valves_world_when_it_moves_for_all_its_devices() {
		let po = devices(3, true);
		let a = Pose::default();
		let a2 = compose(&moved(), &a);
		let mut now = 10.0;
		for k in 1..=3 {
			po.on_pose_locked(0, std::ptr::null_mut(), k, &valve_at(&at(k), &a), now).unwrap();
		}
		assert_eq!(po.lock().frame_wfd, Some(a));
		// the world-from-driver moves for every device, their poses in Valve's driver space as they were
		now += 0.004;
		let still = |k: u32| valve_at(&compose(&a2, &compose(&inverse(&a), &at(k))), &a2);
		po.on_pose_locked(0, std::ptr::null_mut(), 1, &still(1), now).unwrap();
		assert_eq!(po.lock().frame_wfd, Some(a), "one device is not Valve's world");
		po.on_pose_locked(0, std::ptr::null_mut(), 2, &still(2), now).unwrap();
		let st = po.lock();
		let f = st.frame_wfd.unwrap();
		assert!(near(&f, &a2, 1e-9, 1e-9), "{f:?}");
		assert!(st.devs.values().all(|d| d.world_move.is_none()));
		drop(st);
		// the third, still on the old one until its pose comes: read through it; then on the frame's again
		po.on_pose_locked(0, std::ptr::null_mut(), 3, &still(3), now + 0.001).unwrap();
		let st = po.lock();
		assert!(near(&st.frame_wfd.unwrap(), &a2, 1e-9, 1e-9));
		assert_eq!(st.devs[&3].to_frame, Pose::default());
	}

	#[test]
	fn a_driver_space_put_elsewhere_with_the_world_kept_moves_nothing_shown() {
		// Valve's driver space moved 1 m / 25 deg for every device, each world-from-driver moved with it: the poses
		// jump in the driver space and stay in the world
		let po = devices(3, true);
		let a = Pose { p: V3::new(0.1, 0.2, 0.3), q: about(V3::new(0.0, 1.0, 0.0), 10.0) };
		let l = Pose { p: V3::new(-0.66, 0.34, 0.57), q: about(V3::new(0.0, 1.0, 0.0), 25.0) };
		let t = Pose { p: V3::new(0.4, -0.1, 0.2), q: about(V3::new(0.0, 1.0, 0.0), 30.0) };
		po.lock().t = t;
		let mut now = 10.0;
		for k in 1..=3 {
			po.on_pose_locked(0, std::ptr::null_mut(), k, &valve_at(&at(k), &a), now).unwrap();
			po.lock().samples.push_back(Sample { t: now, src: V3::default(), dst: V3::new(0.1, 0.0, 0.0), id: k });
			let mut st = po.lock();
			let d = st.devs.get_mut(&k).unwrap();
			d.last_sample = d.head_f.p;
		}
		now += 0.004;
		let al = compose(&a, &l);
		po.on_pose_locked(0, std::ptr::null_mut(), 1, &valve_at(&at(1), &al), now).unwrap();
		assert_eq!(po.lock().frame_wfd, Some(a), "one device moved over");
		for k in 2..=3 {
			po.on_pose_locked(0, std::ptr::null_mut(), k, &valve_at(&at(k), &al), now).unwrap();
		}
		let st = po.lock();
		// the frame on the new one, the fit and the pairs re-based with it: what SteamVR shows is where it was
		let f = st.frame_wfd.unwrap();
		assert!(near(&f, &al, 1e-12, 1e-9));
		assert!(near(&compose(&f, &st.t), &compose(&a, &t), 1e-9, 1e-9));
		assert_eq!(st.frame_gen, 1);
		for x in &st.samples {
			assert!(len(xform(&f, x.dst) - xform(&a, V3::new(0.1, 0.0, 0.0))) < 1e-9);
		}
		// a device's pose in the frame's driver space: where it is in the world
		assert!(len(xform(&f, st.devs[&3].head_f.p) - at(3).p) < 1e-9);
		assert!(st.devs.values().all(|d| d.world_move.is_none()));
		// what the devices keep in the frame's driver space re-based too: their last pair's place, and their last head,
		// so the fallback saw no jump (Valve's pose usable all along)
		for (k, d) in &st.devs {
			assert!(len(xform(&f, d.last_sample) - at(*k).p) < 1e-9, "{k}");
			assert_eq!(d.valve_ok_since, 10.0, "{k}");
		}
	}

	#[test]
	fn a_pose_is_near_itself_with_a_quaternion_a_little_off_unit() {
		// Valve's quaternions come through floats: |q| 4e-7 off 1 is (1 - 8e-7) against itself, past cos(1 mrad / 2)
		let s = 1.0 - 4e-7;
		let a = Pose { p: V3::new(0.1, 1.2, -0.3), q: Q4::new(0.6 * s, 0.0, 0.8 * s, 0.0) };
		let b = Pose { p: a.p + V3::new(1e-4, 0.0, 0.0), q: a.q };
		assert!(near(&a, &a, 0.0, 0.0));
		assert!(near(&a, &b, 0.002, 0.002));
		let c = Pose { p: a.p, q: qnorm(qmul(a.q, about(V3::new(1.0, 0.0, 0.0), 0.2))) };
		assert!(!near(&a, &c, 0.002, 0.002), "0.2 deg is not 0.11");
	}

	#[test]
	fn a_loss_decided_before_a_newer_runtime_pose_is_not_taken() {
		let now = 100.0;
		let po = bench(3, DONGLE, now);
		// the receive thread's check saw the pose before `now`; the one at `now` came in between
		po.on_sally_lost(SERIAL, now + 0.5, now - 0.1);
		assert!(!po.lock().sally[SERIAL].lost);
		po.on_sally_lost(SERIAL, now + 0.5, now);
		assert!(po.lock().sally[SERIAL].lost);
	}

	#[test]
	fn before_the_fit_the_frame_is_the_one_most_devices_are_on() {
		let po = devices(3, false);
		let a = Pose::default();
		let b = Pose { p: V3::new(-2.34, 0.06, -2.08), q: about(V3::new(0.0, 1.0, 0.0), -100.9) };
		let now = 10.0;
		po.on_pose_locked(0, std::ptr::null_mut(), 1, &valve_at(&at(1), &b), now).unwrap();
		assert_eq!(po.lock().frame_wfd, Some(b));
		po.lock().samples.push_back(Sample { t: now, src: V3::default(), dst: V3::default(), id: 1 });
		po.on_pose_locked(0, std::ptr::null_mut(), 2, &valve_at(&at(2), &a), now).unwrap();
		assert_eq!(po.lock().frame_wfd, Some(b), "one against one");
		po.on_pose_locked(0, std::ptr::null_mut(), 3, &valve_at(&at(3), &a), now).unwrap();
		let st = po.lock();
		assert_eq!(st.frame_wfd, Some(a));
		// the pair taken on the other one, re-based: the same place in the world
		assert!(len(xform(&a, st.samples[0].dst) - xform(&b, V3::default())) < 1e-9);
	}

	/// A Tundra's: Valve's head 180 deg about the runtime's head Y and 1.5 mm off it
	fn tundra() -> (Pose, Pose) {
		let h2i = Pose { p: V3::new(0.0140, 0.0097, -0.0053), q: qnorm(Q4::new(0.2272, -0.6845, -0.2391, 0.6502)) };
		let c = Pose { p: V3::new(0.0, 0.0, 0.0015), q: about(V3::new(0.0, 1.0, 0.0), 180.0) };
		(h2i, compose(&h2i, &c))
	}

	#[test]
	fn the_head_frame_is_valves() {
		let now = 100.0;
		let po = bench(3, DONGLE, now);
		let (h2i, dfh) = tundra();
		po.runtime_head(SERIAL, &[h2i.p.x, h2i.p.y, h2i.p.z, h2i.q.w, h2i.q.x, h2i.q.y, h2i.q.z]);
		let mut p = valve_pose();
		p.qDriverFromHeadRotation = hq(dfh.q);
		p.vecDriverFromHeadTranslation = [dfh.p.x, dfh.p.y, dfh.p.z];
		po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		let c = po.lock().head_fix[SERIAL].c;
		assert!(near(&c, &compose(&inverse(&h2i), &dfh), 1e-12, 1e-9));
		assert!((qangle(c.q).to_degrees() - 180.0).abs() < 1e-6 && (len(c.p) - 0.0015).abs() < 1e-9);
		// a runtime pose: turning about its own x at 2 rad/s
		let head = Pose { p: V3::new(0.2, 1.1, -0.4), q: about(norm_v(V3::new(0.3, 0.8, -0.2)), 70.0) };
		let pose = DriverPose_t {
			qRotation: hq(head.q),
			vecPosition: [head.p.x, head.p.y, head.p.z],
			vecVelocity: [0.5, 0.0, -0.1],
			vecAngularVelocity: [2.0, 0.0, 0.0],
			..DriverPose_t::default()
		};
		po.on_sally_pose(SERIAL, &pose, Shown::default(), now);
		let got = po.lock().sally[SERIAL].pose;
		let want = compose(&head, &c);
		let g = pose_of(&got.qRotation, &got.vecPosition);
		assert!(near(&g, &want, 1e-12, 1e-9), "{g:?} {want:?}");
		// the body rate in Valve's head axes: the same turn in the world
		let w_world = qrot(head.q, V3::new(2.0, 0.0, 0.0));
		let w_got = qrot(g.q, V3::new(got.vecAngularVelocity[0], got.vecAngularVelocity[1], got.vecAngularVelocity[2]));
		assert!(len(w_got - w_world) < 1e-12);
		// the head point moved 1.5 mm: its velocity has w x r on top
		let r = qrot(head.q, c.p);
		let dv = V3::new(got.vecVelocity[0] - 0.5, got.vecVelocity[1], got.vecVelocity[2] + 0.1);
		assert!(len(dv - cross(w_world, r)) < 1e-12);
		// and SteamVR gets it so
		let (out, replace, _, _) = po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		assert!(replace && near(&pose_of(&out.qRotation, &out.vecPosition), &want, 1e-12, 1e-9));
	}

	#[test]
	fn a_head_frame_valve_shares_is_left_as_it_is() {
		let now = 100.0;
		let po = bench(3, DONGLE, now);
		// an Index controller's: Valve's driver-from-head is the runtime's head-to-IMU
		let h2i = Pose { p: V3::new(-0.149, 0.0546, -0.0087), q: qnorm(Q4::new(0.1136, 0.6767, -0.1975, 0.7001)) };
		po.runtime_head(SERIAL, &[h2i.p.x, h2i.p.y, h2i.p.z, h2i.q.w, h2i.q.x, h2i.q.y, h2i.q.z]);
		let mut p = valve_pose();
		p.qDriverFromHeadRotation = hq(h2i.q);
		p.vecDriverFromHeadTranslation = [h2i.p.x, h2i.p.y, h2i.p.z];
		po.on_pose_locked(0, std::ptr::null_mut(), 3, &p, now).unwrap();
		let pose = DriverPose_t {
			qRotation: hq(about(V3::new(0.0, 0.0, 1.0), 30.0)),
			vecPosition: [0.2, 1.1, -0.4],
			vecVelocity: [0.5, 0.0, -0.1],
			vecAngularVelocity: [2.0, 1.0, 0.0],
			..DriverPose_t::default()
		};
		po.on_sally_pose(SERIAL, &pose, Shown::default(), now);
		let got = po.lock().sally[SERIAL].pose;
		for i in 0..3 {
			assert!((got.vecPosition[i] - pose.vecPosition[i]).abs() < 1e-12);
			assert!((got.vecVelocity[i] - pose.vecVelocity[i]).abs() < 1e-12);
			assert!((got.vecAngularVelocity[i] - pose.vecAngularVelocity[i]).abs() < 1e-12);
		}
		assert!(near(
			&pose_of(&got.qRotation, &got.vecPosition),
			&pose_of(&pose.qRotation, &pose.vecPosition),
			1e-12,
			1e-9
		));
	}

	fn norm_v(v: V3) -> V3 {
		v * (1.0 / len(v))
	}

	#[test]
	fn the_first_fit_is_of_pairs_that_pin_it_however_many_devices_they_are_of() {
		let truth = Pose { p: V3::new(0.11, 0.0, 0.0), q: Q4::default() };
		let now = 50.0;
		let pairs = |po: &PoseOverride, of: u32, at: &dyn Fn(u32) -> V3| {
			let mut st = po.lock();
			for i in 0..200 {
				let src = at(i);
				st.samples.push_back(Sample { t: now, src, dst: xform(&truth, src), id: 1 + i % of });
			}
		};
		// four devices standing in a row, a few millimetres off it: any turn about the row fits them, and SteamVR
		// keeps Valve's tracking
		let po = devices(4, false);
		pairs(&po, 4, &|i| V3::new(0.3 * (i % 4) as f64, 1.0 + 0.002 * (i % 3) as f64, 0.1 * (i % 4) as f64));
		po.fit(now);
		assert!(!po.fit_ok());
		// two devices carried about the room: their pairs make the fit
		let po = devices(2, false);
		pairs(&po, 2, &|i| V3::new((i % 7) as f64 * 0.1, (i % 5) as f64 * 0.2, (i % 3) as f64 * 0.3));
		po.fit(now);
		assert!(po.fit_ok() && len(po.lock().t.p - truth.p) < 1e-9);
	}

	#[test]
	fn a_refit_takes_new_pairs_and_their_fit_as_it_is() {
		let po = devices(3, true);
		let old = Pose { p: V3::new(0.1, 0.0, 0.0), q: Q4::default() };
		// 1 cm off the old one: a refit without the request would ease a fifth of the way
		let new = Pose { p: V3::new(0.11, 0.0, 0.0), q: Q4::default() };
		po.lock().t = old;
		let now = 50.0;
		let pairs = |po: &PoseOverride| {
			let mut st = po.lock();
			for i in 0..200 {
				let src = V3::new((i % 7) as f64 * 0.1, (i % 5) as f64 * 0.2, (i % 3) as f64 * 0.3);
				st.samples.push_back(Sample { t: now, src, dst: xform(&new, src), id: 1 + i % 3 });
			}
		};
		pairs(&po);
		po.fit(now);
		let eased = po.lock().t;
		assert!((eased.p.x - 0.102).abs() < 1e-6, "{eased:?}");
		// asked: the pairs it had are dropped, SteamVR keeps the fit it has, then the new pairs' fit is taken whole
		po.lock().t = old;
		po.refit_req.store(true, SeqCst);
		po.fit(now);
		assert!(po.lock().samples.is_empty() && po.lock().refit_jump && po.lock().t == old);
		assert!(po.frame_line().starts_with(b"F 1 0 1 0.10000 "));
		pairs(&po);
		po.fit(now);
		let st = po.lock();
		assert!(len(st.t.p - new.p) < 1e-9 && !st.refit_jump, "{:?}", st.t);
	}
}
