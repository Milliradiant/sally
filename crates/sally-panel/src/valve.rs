//! The comparison against Valve's lighthouse driver through OpenVR, and the OpenVR side of the SteamVR dashboard
//! overlay. OpenVR is reached through `Vr` (crate::openvr's library, or `fake`'s stand-in), and only ever attached to:
//! the panel never starts SteamVR.
//!
//! - The session: a Background app first, which only attaches to a SteamVR that runs; then, when the overlay is
//!   wanted, shut down and started again as an Overlay app, the only type that may own one (as the first it would
//!   have started SteamVR). Tried every 3 s while there is no SteamVR, 15 s after one without IVRSystem, and 10 s
//!   after SteamVR's Quit, which is acknowledged as exiting.
//! - The devices: every 2 s each index's class, tracking system, serial and driver (ResourceRoot and
//!   RegisteredDeviceType: the sally driver can call its devices "lighthouse", for Space Calibrator, so the name does
//!   not make a device Valve's, and sally's own are never compared), and a tracker's role (steamvr.vrsettings,
//!   section "trackers"); every poll the poses in the raw universe, the headset's with them.
//! - The fit: Valve's trackers and controllers paired with the runtime's devices by serial; a pair of positions every
//!   3 cm of motion while both track, the last minute's (3000 at most); the robust Horn fit (outliers trimmed) every
//!   2 s from 60 pairs. Then per device the distance (its median and p90 over 10 s), the rotation once the constant
//!   offset between the two device frames is learnt, and a trail.
//! - The sally driver's compare mode: SteamVR shows sally's poses on Valve's device entries, so OpenVR no longer has
//!   Valve's; the driver sends them as UDP "V" lines to 127.0.0.1:9103 instead, with "C" lines of its state, and takes
//!   `driver_cmd`'s on 9104.
//! - The overlay: the dashboard overlay "sally" and its button, whose textures the host draws and hands over
//!   (`overlay_submit`, `overlay_submit_thumb`), the laser pointer's events queued in `ov_events` in the order SteamVR
//!   sent them, and SteamVR's keyboard for naming a device from the headset (`overlay_keyboard`).

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};

use sally_math::math3d::{
	compose, fit_rigid_robust, inverse, len, qangle, qconj, qfrom_mat, qmul, qnorm, xform, Pose, Q4, V3,
};
use sally_math::nth_element::nth_element;
use sally_math::scanf::{c_str, sscanf, Value};

use crate::openvr::{self, *};
use crate::runtime::{EvLevel, Runtime, TrailPoint};

pub use crate::openvr::Vr;

pub mod fake;
pub mod fake_driver;

/// A device of Valve's driver as OpenVR shows it, and how it compares with ours
#[derive(Clone, Debug)]
pub struct ValveDevice {
	pub index: i32,
	/// ETrackedDeviceClass
	pub cls: ETrackedDeviceClass,
	/// the tracking system's name, "lighthouse" for Valve's driver
	pub system: String,
	pub serial: String,
	/// published by the sally driver (never compared against ourselves)
	pub ours_driver: bool,
	/// SteamVR's tracker role ("TrackerRole_Waist"), empty when it has none
	pub role: String,
	/// its pose valid and running OK this frame
	pub valid: bool,
	/// in OpenVR's raw universe (Y up)
	pub raw: Pose,
	/// in sally's frame through the fit
	pub ours: Pose,
	pub have_ours: bool,
	/// the distance from our pose of the same serial (mm); err_med, err_p90: its median and 90th percentile over the
	/// last 10 s; err_deg_ema: the rotation left after the learnt offset, smoothed (degrees)
	pub err_mm: f64,
	pub err_med: f64,
	pub err_deg_ema: f64,
	/// (time, mm)
	pub err_window: VecDeque<(f64, f64)>,
	pub err_p90: f64,
	/// the constant rotation between Valve's device frame and ours, learnt over the first 300 frames
	pub rot_offset: Q4,
	pub rot_offset_n: i32,
	/// where and when the last fit pair was taken (raw)
	pub last_sample: V3,
	pub last_sample_t: f64,
	pub sampled: bool,
	pub trail: VecDeque<TrailPoint>,
}

impl Default for ValveDevice {
	fn default() -> ValveDevice {
		ValveDevice {
			index: -1,
			cls: TrackedDeviceClass_Invalid,
			system: String::new(),
			serial: String::new(),
			ours_driver: false,
			role: String::new(),
			valid: false,
			raw: Pose::default(),
			ours: Pose::default(),
			have_ours: false,
			err_mm: -1.0,
			err_med: -1.0,
			err_deg_ema: -1.0,
			err_window: VecDeque::new(),
			err_p90: -1.0,
			rot_offset: Q4::default(),
			rot_offset_n: 0,
			last_sample: V3::default(),
			last_sample_t: 0.0,
			sampled: false,
			trail: VecDeque::new(),
		}
	}
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OverlayEventKind {
	#[default]
	Move,
	Button,
	Wheel,
	Leave,
}

/// One of the laser pointer's events on the overlay
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OverlayEvent {
	pub kind: OverlayEventKind,
	/// texture pixels, y down (a button's are the move's before it)
	pub x: f32,
	pub y: f32,
	/// 0 left, 1 right, 2 middle
	pub button: i32,
	pub down: bool,
	pub wheel: f32,
}

/// The comparison and the overlay's OpenVR side (valve.h's ValveCompare)
pub struct ValveCompare {
	pub enabled: bool,
	/// connected to SteamVR
	pub running: bool,
	pub status: String,
	/// by serial (Valve's lighthouse devices only)
	pub devs: BTreeMap<String, ValveDevice>,
	pub have_hmd: bool,
	pub hmd_raw: Pose,
	pub hmd_ours: Pose,
	pub fit_ok: bool,
	/// OpenVR's raw universe to sally's world
	pub fit: Pose,
	/// the median distance between Valve's (aligned) and ours over the fit's pairs (m)
	pub fit_median: f64,
	pub fit_n: i32,
	pub fit_kept: i32,
	/// only the last minute's pairs: Valve's frame settles after SteamVR starts, and can move later
	pub fit_window_s: f64,
	pub compared: i32,
	pub trail_seconds: f64,
	/// SteamVR's tracker roles, by serial
	pub roles_by_serial: BTreeMap<String, String>,

	/// the sally driver's compare mode: SteamVR shows sally's poses on Valve's device entries, so OpenVR no longer has
	/// Valve's; the driver sends them here instead (UDP "V" lines, its state in "C" lines), and takes `driver_cmd` on
	/// the other port
	pub driver_port: u16,
	pub driver_cmd_port: u16,
	/// V lines came in the last 2 s
	pub hook_mode: bool,
	/// "sally" or "valve": what SteamVR shows now
	pub hook_seen: String,
	/// the driver's own frame fit is ready (it shows Valve's poses until it is)
	pub hook_fit: bool,
	pub hook_fit_mm: f64,
	/// the rest of the driver's status (its C line), as it last said: how many of SteamVR's pose interfaces it could
	/// take (0: SteamVR is shown Valve's tracking whatever is chosen), the devices sally has a pose for, whose pose a
	/// lighthouse headset gets and whether there is one, how many sessions' files stay
	pub drv_hooks: Option<i32>,
	pub drv_sally_devices: i32,
	pub drv_headset_sally: Option<bool>,
	pub drv_lighthouse_headset: bool,
	pub drv_keep_sessions: Option<i32>,
	/// the driver's last F line: its fit and frame
	pub drv_fit: Option<DriverFit>,
	/// the comparison goes through the driver's fit (`fit` is the driver's, turned into the panel's terms): in the
	/// driver's compare mode, once it sends its fit. "Re-align to Valve" then asks the driver to re-align
	pub fit_from_driver: bool,
	driver_refitting: bool,
	/// the driver's output layer (docs/sally_driver.md, "The output layer"): (scope, setting) -> value, from its O
	/// lines, which `output` asks for and every set answers with; the clock of the last complete list (O end), and
	/// the driver's word on the last set it refused
	pub output: BTreeMap<(String, String), f64>,
	pub output_t: f64,
	pub output_error: String,
	output_asked: f64,
	/// the list as it comes in, until its O end
	output_next: BTreeMap<(String, String), f64>,
	/// the clock of the driver's last status line (C): every command is answered with one, the output command by a
	/// driver without an output layer too (with no O lines)
	drv_c_t: f64,

	/// the SteamVR dashboard overlay ("sally" tab): the host renders the panel itself into a GL texture and hands it
	/// to overlay_submit while overlay_visible. Made once SteamVR is seen (it takes an Overlay session)
	pub overlay_wanted: bool,
	pub overlay_ok: bool,
	pub overlay_visible: bool,
	pub thumb_visible: bool,
	/// counts the overlay's creations (a new dashboard button needs its picture)
	pub overlay_gen: i32,
	/// the texture's size, which is the mouse scale
	pub overlay_w: i32,
	pub overlay_h: i32,
	pub overlay_width_m: f32,
	pub overlay_status: String,
	/// the last texture hand-over failure, shown instead of the visibility until one works
	pub overlay_error: String,
	/// the laser pointer's events in the order SteamVR sent them (a click's down and up often land between two
	/// frames); the host takes them every overlay frame
	pub ov_events: Vec<OverlayEvent>,
	/// the pointer's y: SteamVR on Windows reports it from the top of the overlay as shown (OpenVR Advanced Settings
	/// uses it as is there and flips it only on Linux), whatever openvr.h's "GL space" comment says; the cfg can flip
	/// it if it ever is
	pub overlay_flip_pointer: bool,
	/// SteamVR's keyboard over the overlay (naming a device from the headset): `overlay_keyboard` shows it with a
	/// text to edit, and the text comes back with the keyboard's Done: kb_done is set and kb_text holds it, for kb_tag
	/// (whatever the caller passed). Closed without Done is cancelled
	pub kb_open: bool,
	pub kb_done: bool,
	pub kb_tag: String,
	pub kb_text: String,
	/// why the keyboard could not be shown
	pub kb_error: String,
	/// SteamVR's chaperone: its walls (each one's four corners) in OpenVR's raw universe, read every WALLS_S, and in
	/// sally's world through the fit (empty without one)
	pub walls_raw: Vec<[V3; 4]>,
	pub walls: Vec<[V3; 4]>,

	vr: Box<dyn Vr>,
	/// 127.0.0.1:driver_port, bound by the first poll that can
	drv_sock: Option<UdpSocket>,
	last_v: f64,
	/// serial -> (valid, the head's pose in Valve's driver space), from the V lines
	driver_valve: BTreeMap<String, (bool, Pose)>,
	driver_valve_t: BTreeMap<String, f64>,
	ov_main: VROverlayHandle_t,
	ov_thumb: VROverlayHandle_t,
	/// initialised as an Overlay app (needed for IVROverlay)
	overlay_session: bool,
	/// keyboard requests made and answered (the request's number rides on its events)
	kb_seq: u64,
	kb_seq_done: u64,
	next_try: f64,
	next_props: f64,
	last_fit: f64,
	next_create: f64,
	samples_src: Vec<V3>,
	samples_dst: Vec<V3>,
	samples_t: Vec<f64>,
	/// the serial each pair is of
	samples_dev: Vec<String>,
	by_index: BTreeMap<i32, ValveDevice>,
	/// while a check records: what SteamVR shows, written down as it is read (check.rs)
	pub check_log: Option<crate::check::PoseLog>,
	/// the last poll's clock (`refit` is asked between polls)
	last_poll: f64,
	/// when the driver was asked to re-align, until an F line says it is at it: F lines sent before it had the
	/// command still say it is not, and one of them had the panel log a re-align that had not happened
	refit_asked: Option<f64>,
	/// the per-device percentiles' buffer, kept from frame to frame
	scratch: Vec<f64>,
	next_walls: f64,
	/// Locate's buzz on a controller: its device index and when it began
	buzz: Option<(TrackedDeviceIndex_t, f64)>,
}

/// How often the chaperone's walls are read again (s)
const WALLS_S: f64 = 5.0;

/// How often the driver's output settings are asked for while they are shown (s)
const OUTPUT_ASK_S: f64 = 2.0;

/// Locate's buzz: BUZZES of BUZZ_S each, BUZZ_GAP_S apart, a pulse of PULSE_US every frame meanwhile (the legacy
/// pulse is at most 4 ms, one a controller every 5 ms)
const BUZZES: f64 = 3.0;
const BUZZ_S: f64 = 0.15;
const BUZZ_GAP_S: f64 = 0.15;
const PULSE_US: u16 = 3999;

/// The pairs the fit needs (the first fit and every refit), from at least K_FIT_DEVICES devices where there are as many
/// to compare. 60, taken only on moves, are a fit to whatever moves most: with one tracker giving 80 % of them, a
/// re-align moved the frame 0.3-0.8 m six times in 40 s (with still devices' pairs and 100 of them, once by 66 mm)
const K_FIT_PAIRS: usize = 100;
const K_FIT_DEVICES: usize = 2;
/// The serials the V lines may name at once, and how long one is kept unheard (s): anything on the machine can send to
/// the driver's port, and a serial a line was kept for good
const K_DRIVER_SERIALS: usize = k_unMaxTrackedDeviceCount as usize;
const K_DRIVER_FORGET_S: f64 = 10.0;
/// How long an F line saying "not re-aligning" is taken for one sent before the driver had the refit (s): its F lines
/// come every second
const K_REFIT_ACK_S: f64 = 3.0;

impl Default for ValveCompare {
	fn default() -> ValveCompare {
		ValveCompare::new()
	}
}

impl Drop for ValveCompare {
	fn drop(&mut self) {
		self.shutdown();
	}
}

/// HmdMatrix34_t to a pose: the rotation's rows, the position in the last column
pub(crate) fn from_mat34(m: &HmdMatrix34_t) -> Pose {
	let mut r = [[0.0f64; 3]; 3];
	for (i, row) in r.iter_mut().enumerate() {
		for (j, x) in row.iter_mut().enumerate() {
			*x = m.m[i][j] as f64;
		}
	}
	Pose { p: V3::new(m.m[0][3] as f64, m.m[1][3] as f64, m.m[2][3] as f64), q: qfrom_mat(&r) }
}

/// snprintf(buf, size, "%s", s.c_str()): the string up to its first NUL, cut to size - 1 bytes
fn c_buffer(s: &str, size: usize) -> Vec<u8> {
	let b = c_str(s.as_bytes());
	b[..b.len().min(size - 1)].to_vec()
}

const V_LINE: &str = "V %63s %lf %d %lf %lf %lf %lf %lf %lf %lf %d";
const C_LINE: &str = "C seen %15s fit %d median_mm %lf";
const F_LINE: &str = "F %d %d %d %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf";

/// The sally driver's frame line ("F", sent after each of its fits): the fit SteamVR shows sally's poses through (the
/// runtime's world in OpenVR's axes, Y up, to the frame's driver space, which the V lines are in) and the frame's
/// world-from-driver (SteamVR's raw universe from that driver space)
#[derive(Clone, Copy, Debug, Default)]
pub struct DriverFit {
	pub t_recv: f64,
	pub ok: bool,
	/// the pairs it has now (a re-align waits for 150 from 3 devices)
	pub samples: i32,
	pub refitting: bool,
	pub t: Pose,
	pub wfd: Option<Pose>,
}

/// The runtime's world (Z up) from OpenVR's axes (Y up): (x, y, z) -> (x, -z, y), the driver's turn undone
fn zup_from_yup() -> Pose {
	let s = 0.5f64.sqrt();
	Pose { p: V3::default(), q: Q4::new(s, s, 0.0, 0.0) }
}

impl ValveCompare {
	/// Through openvr_api, which the first poll looks for
	pub fn new() -> ValveCompare {
		ValveCompare::with_vr(Box::new(OpenVr::new()))
	}

	/// Through any `Vr` (a test's fake)
	pub fn with_vr(vr: Box<dyn Vr>) -> ValveCompare {
		ValveCompare {
			enabled: true,
			running: false,
			status: "SteamVR not checked yet".into(),
			devs: BTreeMap::new(),
			have_hmd: false,
			hmd_raw: Pose::default(),
			hmd_ours: Pose::default(),
			fit_ok: false,
			fit: Pose::default(),
			fit_median: 0.0,
			fit_n: 0,
			fit_kept: 0,
			fit_window_s: 60.0,
			compared: 0,
			trail_seconds: 6.0,
			roles_by_serial: BTreeMap::new(),
			driver_port: 9103,
			driver_cmd_port: 9104,
			hook_mode: false,
			hook_seen: String::new(),
			hook_fit: false,
			hook_fit_mm: 0.0,
			drv_hooks: None,
			drv_sally_devices: 0,
			drv_headset_sally: None,
			drv_lighthouse_headset: false,
			drv_keep_sessions: None,
			drv_fit: None,
			fit_from_driver: false,
			driver_refitting: false,
			output: BTreeMap::new(),
			output_t: f64::NEG_INFINITY,
			output_error: String::new(),
			output_asked: f64::NEG_INFINITY,
			output_next: BTreeMap::new(),
			drv_c_t: f64::NEG_INFINITY,
			overlay_wanted: true,
			overlay_ok: false,
			overlay_visible: false,
			thumb_visible: false,
			overlay_gen: 0,
			overlay_w: 2000,
			overlay_h: 1175,
			overlay_width_m: 2.5,
			overlay_status: "off".into(),
			overlay_error: String::new(),
			ov_events: Vec::new(),
			overlay_flip_pointer: false,
			kb_open: false,
			kb_done: false,
			kb_tag: String::new(),
			kb_text: String::new(),
			kb_error: String::new(),
			walls_raw: Vec::new(),
			walls: Vec::new(),
			vr,
			drv_sock: None,
			last_v: -1e9,
			driver_valve: BTreeMap::new(),
			driver_valve_t: BTreeMap::new(),
			ov_main: 0,
			ov_thumb: 0,
			overlay_session: false,
			kb_seq: 0,
			kb_seq_done: 0,
			next_try: 0.0,
			next_props: 0.0,
			last_fit: 0.0,
			next_create: 0.0,
			samples_src: Vec::new(),
			samples_dst: Vec::new(),
			samples_t: Vec::new(),
			samples_dev: Vec::new(),
			by_index: BTreeMap::new(),
			check_log: None,
			last_poll: 0.0,
			refit_asked: None,
			scratch: Vec::new(),
			next_walls: 0.0,
			buzz: None,
		}
	}

	/// SteamVR's devices as it lists them now, for a check's session (check.rs): each with its class, role, dongle,
	/// model and tracking system, a base station with its mode's label (its channel). Empty without SteamVR
	pub fn steam_devices(&mut self) -> Vec<sally_check::session::SteamDevice> {
		if !self.running {
			return Vec::new();
		}
		let vr = &mut self.vr;
		self.by_index
			.values()
			.filter(|d| d.cls != TrackedDeviceClass_Invalid && !d.serial.is_empty())
			.map(|d| {
				let i = d.index as u32;
				let class = crate::check::class_word(d.cls);
				sally_check::session::SteamDevice {
					serial: d.serial.clone(),
					class: class.to_string(),
					role: if class == "hmd" { "Head".to_string() } else { d.role.clone() },
					dongle: vr.string_tracked_device_property(i, Prop_ConnectedWirelessDongle_String),
					model: vr.string_tracked_device_property(i, Prop_ModelNumber_String),
					system: d.system.clone(),
					mode: if class == "station" {
						vr.string_tracked_device_property(i, Prop_ModeLabel_String)
					} else {
						String::new()
					},
				}
			})
			.collect()
	}

	/// "Re-align to Valve": through the driver's fit, the driver re-aligns (`refit`: new pairs, SteamVR keeps its
	/// alignment until they make one); else the panel forgets its fit and its pairs and aligns again
	pub fn refit(&mut self) {
		if self.fit_from_driver {
			self.driver_cmd("refit");
			self.refit_asked = Some(self.last_poll);
			if let Some(f) = self.drv_fit.as_mut() {
				f.refitting = true; // until the driver's next F line says how far it is
			}
			return;
		}
		self.forget_fit();
	}

	/// The fit and its pairs forgotten: the next K_FIT_PAIRS align again
	fn forget_fit(&mut self) {
		self.samples_src.clear();
		self.samples_dst.clear();
		self.samples_t.clear();
		self.samples_dev.clear();
		self.fit_ok = false;
		self.forget_measures();
	}

	/// What each device measured through the alignment, which a new one makes old: its pairs' places, the rotation
	/// offset learnt, the last 10 s of distances
	fn forget_measures(&mut self) {
		for d in self.devs.values_mut() {
			d.sampled = false;
			d.rot_offset_n = 0;
			d.err_med = -1.0;
			d.err_p90 = -1.0;
			d.err_window.clear();
		}
	}

	/// Before the host deletes its GL context: OpenVR holds the overlay's GL textures
	pub fn close_vr(&mut self) {
		self.shutdown();
	}

	fn shutdown(&mut self) {
		if self.vr.has(Interface::Overlay) && self.ov_main != 0 {
			self.vr.destroy_overlay(self.ov_main);
			self.ov_main = 0;
			self.ov_thumb = 0;
		}
		// a session runs only with the library loaded (the C++'s `lib && running`); Vr::shutdown forgets the tables
		// with it, where the C++ nulled its pointers (none are held without a session)
		if self.running {
			self.vr.shutdown();
		}
		self.running = false;
		self.overlay_ok = false;
		self.overlay_visible = false;
		self.thumb_visible = false;
		self.overlay_session = false;
		self.kb_open = false;
		self.ov_events.clear();
		self.overlay_status = if self.overlay_wanted { "SteamVR not running" } else { "off" }.into();
		self.by_index.clear();
		self.have_hmd = false;
		for d in self.devs.values_mut() {
			d.valid = false;
		}
		self.walls_raw.clear();
		self.walls.clear();
		self.next_walls = 0.0;
		self.buzz = None;
	}

	/// SteamVR's chaperone's walls (its live collision bounds, in its standing universe) in OpenVR's raw universe:
	/// none without IVRChaperoneSetup or when it has none
	fn read_walls(&mut self) -> Vec<[V3; 4]> {
		if !self.vr.has(Interface::ChaperoneSetup) {
			return Vec::new();
		}
		let (Some(quads), Some(m)) = (self.vr.live_collision_bounds(), self.vr.raw_zero_to_standing()) else {
			return Vec::new();
		};
		// the raw universe's zero in the standing one: a standing point into the raw universe is its inverse
		let raw_from_standing = inverse(&from_mat34(&m));
		quads
			.iter()
			.map(|q| {
				q.vCorners.map(|c| xform(&raw_from_standing, V3::new(c.v[0] as f64, c.v[1] as f64, c.v[2] as f64)))
			})
			.filter(|q| q.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()))
			.collect()
	}

	/// Locate on a device: a controller of Valve's (by serial) buzzes three times, through SteamVR's legacy haptic
	/// pulse. Whether it went out: a controller SteamVR lists, with SteamVR running
	pub fn buzz(&mut self, serial: &str, now: f64) -> bool {
		let Some(d) = self.devs.get(serial).filter(|d| d.cls == TrackedDeviceClass_Controller && d.index >= 0) else {
			return false;
		};
		if !self.running || !self.vr.has(Interface::System) {
			return false;
		}
		self.buzz = Some((d.index as TrackedDeviceIndex_t, now));
		true
	}

	/// A pulse a frame while a buzz is on
	fn buzz_poll(&mut self, now: f64) {
		let Some((index, t0)) = self.buzz else { return };
		let e = now - t0;
		let period = BUZZ_S + BUZZ_GAP_S;
		if e >= BUZZES * period - BUZZ_GAP_S || e < 0.0 {
			self.buzz = None;
			return;
		}
		if e % period < BUZZ_S {
			self.vr.trigger_haptic_pulse(index, 0, PULSE_US);
		}
	}

	fn try_init(&mut self, now: f64) -> bool {
		if now < self.next_try {
			return false;
		}
		self.next_try = now + 3.0;
		match self.vr.load() {
			Ok(()) => {}
			Err(LoadError::Missing) => {
				self.status = "openvr_api library not found next to the panel".into();
				self.overlay_status = if self.overlay_wanted { "no openvr_api library" } else { "off" }.into();
				return false;
			}
			Err(LoadError::Unusable) => {
				self.status = "openvr_api library is not usable".into();
				return false;
			}
		}
		// Background: attach to a running SteamVR, never start one
		if let Err(err) = self.vr.init(VRApplication_Background) {
			self.status = if err == VRInitError_Init_NoServerForBackgroundApp {
				"SteamVR not running".into()
			} else {
				format!("SteamVR init error {err}")
			};
			self.overlay_status = if self.overlay_wanted { self.status.clone() } else { "off".into() };
			return false;
		}
		self.running = true;
		// SteamVR is running: come back as an Overlay app (the only type that may own an overlay; it would have
		// launched SteamVR had we started with it). A library without VR_ShutdownInternal stays a Background app
		if self.overlay_wanted && self.vr.shutdown() {
			if let Err(err) = self.vr.init(VRApplication_Overlay) {
				self.running = false;
				self.status = format!("SteamVR overlay init error {err}");
				self.overlay_status = format!("init error {err}");
				return false;
			}
			self.overlay_session = true;
		}
		let system = self.vr.interface(Interface::System);
		if self.overlay_session {
			if let Err(err) = self.vr.interface(Interface::Overlay) {
				self.overlay_status = format!("SteamVR has no {IVROverlay_Version} (error {err})");
			}
		}
		// without IVRSettings there are no tracker roles, nothing more; without IVRChaperoneSetup no walls
		let _ = self.vr.interface(Interface::Settings);
		let _ = self.vr.interface(Interface::ChaperoneSetup);
		if system.is_err() {
			self.status = format!("SteamVR has no {IVRSystem_Version}");
			self.shutdown();
			self.next_try = now + 15.0;
			return false;
		}
		self.status = "SteamVR connected".into();
		self.next_props = 0.0;
		if self.vr.has(Interface::Overlay) {
			self.overlay_create();
		}
		true
	}

	fn overlay_create(&mut self) {
		if !self.vr.has(Interface::Overlay) || self.ov_main != 0 {
			return;
		}
		match self.vr.create_dashboard_overlay("sally.panel", "sally") {
			Ok((main, thumb)) => {
				self.ov_main = main;
				self.ov_thumb = thumb;
			}
			Err(e) => {
				self.ov_main = 0;
				self.ov_thumb = 0;
				let why = if e == VROverlayError_KeyInUse { " (key in use: another panel running?)" } else { "" };
				self.overlay_status = format!("create error {e}{why}");
				return;
			}
		}
		self.vr.set_overlay_width_in_meters(self.ov_main, self.overlay_width_m);
		self.vr.set_overlay_input_method(self.ov_main, VROverlayInputMethod_Mouse);
		let scale = HmdVector2_t { v: [self.overlay_w as f32, self.overlay_h as f32] };
		self.vr.set_overlay_mouse_scale(self.ov_main, &scale);
		self.vr.set_overlay_flag(self.ov_main, VROverlayFlags_SendVRSmoothScrollEvents, true);
		// the button's picture: drawn mirrored by the panel's thumbnail, as the dashboard shows it (texture bounds
		// don't reach it)
		if self.ov_thumb != 0 {
			self.vr.set_overlay_flag(self.ov_thumb, VROverlayFlags_IsPremultiplied, true);
		}
		self.overlay_ok = true;
		self.overlay_gen += 1;
		self.overlay_status = "in the dashboard".into();
	}

	pub fn overlay_set_width(&mut self, metres: f32) {
		self.overlay_width_m = metres;
		if self.overlay_ok {
			self.vr.set_overlay_width_in_meters(self.ov_main, self.overlay_width_m);
		}
	}

	/// The host drains the queue every overlay frame; a hidden overlay never lets it pile up
	fn push_ov(&mut self, e: OverlayEvent) {
		if self.ov_events.len() < 512 {
			self.ov_events.push(e);
		}
	}

	fn overlay_poll(&mut self) {
		if !self.overlay_ok {
			return;
		}
		self.overlay_visible = self.vr.is_overlay_visible(self.ov_main);
		self.thumb_visible = self.ov_thumb != 0 && self.vr.is_overlay_visible(self.ov_thumb);
		self.overlay_status = if !self.overlay_error.is_empty() {
			self.overlay_error.clone()
		} else if self.overlay_visible {
			"visible".into()
		} else {
			"in the dashboard".into()
		};
		while let Some(ev) = self.vr.poll_next_overlay_event(self.ov_main) {
			let mut e = OverlayEvent::default();
			let y = if self.overlay_flip_pointer { self.overlay_h as f32 - ev.mouse.y } else { ev.mouse.y };
			// (the constants by their module in the patterns: a misspelt one cannot become a binding that takes all)
			match ev.event_type {
				openvr::VREvent_MouseMove => {
					e.kind = OverlayEventKind::Move;
					e.x = ev.mouse.x;
					e.y = y;
					self.push_ov(e);
				}
				openvr::VREvent_MouseButtonDown | openvr::VREvent_MouseButtonUp => {
					e.kind = OverlayEventKind::Move;
					e.x = ev.mouse.x;
					e.y = y;
					self.push_ov(e);
					e.kind = OverlayEventKind::Button;
					e.down = ev.event_type == VREvent_MouseButtonDown;
					e.button = if ev.mouse.button & VRMouseButton_Right != 0 {
						1
					} else if ev.mouse.button & VRMouseButton_Middle != 0 {
						2
					} else {
						0
					};
					self.push_ov(e);
				}
				openvr::VREvent_ScrollSmooth | openvr::VREvent_ScrollDiscrete => {
					// a thumb resting on the stick sends these too
					if ev.scroll.ydelta != 0.0 {
						e.kind = OverlayEventKind::Wheel;
						e.wheel = ev.scroll.ydelta;
						self.push_ov(e);
					}
				}
				openvr::VREvent_FocusLeave => {
					e.kind = OverlayEventKind::Leave;
					self.push_ov(e);
				}
				_ => self.kb_event(ev.event_type, ev.keyboard.uUserValue),
			}
		}
		if self.ov_thumb != 0 {
			while let Some(ev) = self.vr.poll_next_overlay_event(self.ov_thumb) {
				self.kb_event(ev.event_type, ev.keyboard.uUserValue);
			}
		}
		if !self.overlay_visible {
			// hidden: nothing queued matters any more, but the panel must not keep a pointer (or a held trigger) on
			// reopening
			self.ov_events.clear();
			self.ov_events.push(OverlayEvent { kind: OverlayEventKind::Leave, ..OverlayEvent::default() });
		}
	}

	/// An OpenGL texture's name as SteamVR takes it, in gamma colour space
	fn gl_texture(gl_texture: u32) -> Texture_t {
		Texture_t {
			handle: std::ptr::without_provenance_mut(gl_texture as usize),
			eType: TextureType_OpenGL,
			eColorSpace: ColorSpace_Gamma,
		}
	}

	/// The panel's picture for the overlay (an OpenGL texture of overlay_w x overlay_h)
	pub fn overlay_submit(&mut self, gl_texture: u32) {
		if !self.overlay_ok {
			return;
		}
		let e = self.vr.set_overlay_texture(self.ov_main, &ValveCompare::gl_texture(gl_texture));
		self.overlay_error = if e == VROverlayError_None { String::new() } else { format!("texture error {e}") };
	}

	/// The dashboard button's picture (an OpenGL texture)
	pub fn overlay_submit_thumb(&mut self, gl_texture: u32) {
		if !self.overlay_ok || self.ov_thumb == 0 {
			return;
		}
		let e = self.vr.set_overlay_texture(self.ov_thumb, &ValveCompare::gl_texture(gl_texture));
		if e != VROverlayError_None {
			self.overlay_error = format!("button texture error {e}");
		}
	}

	/// Shows SteamVR's keyboard with `text` to edit (both cut to 127 bytes, as the C++'s buffers cut them), its answer
	/// for `tag`; false, and kb_error says why, when it could not be shown
	pub fn overlay_keyboard(&mut self, tag: &str, description: &str, text: &str) -> bool {
		self.kb_error.clear();
		if !self.overlay_ok {
			self.kb_error = "no SteamVR overlay".into();
			return false;
		}
		let request = KeyboardRequest {
			input_mode: k_EGamepadTextInputModeNormal,
			line_mode: k_EGamepadTextInputLineModeSingleLine,
			flags: KeyboardFlag_Modal,
			description: c_buffer(description, 128),
			char_max: 32,
			existing_text: c_buffer(text, 128),
			user_value: self.kb_seq + 1,
		};
		let mut e = self.vr.show_keyboard_for_overlay(self.ov_main, &request);
		if e == VROverlayError_KeyboardAlreadyInUse {
			// still up from an earlier request (its Close never came): take it over
			self.vr.hide_keyboard();
			e = self.vr.show_keyboard_for_overlay(self.ov_main, &request);
		}
		if e != VROverlayError_None {
			self.kb_error = if e == VROverlayError_KeyboardAlreadyInUse {
				"keyboard in use".into()
			} else {
				format!("keyboard error {e}")
			};
			return false;
		}
		self.kb_seq = request.user_value;
		self.kb_tag = tag.into();
		self.kb_open = true;
		true
	}

	/// The keyboard's events, from whichever queue they come in (the overlay's own; the system queue too, to be
	/// safe): the text is taken once per request, at its Done
	fn kb_event(&mut self, event_type: EVREventType, user_value: u64) {
		if event_type == VREvent_KeyboardDone {
			// nothing asked, answered already (the same Done from the other queue), or an older request's
			if !self.vr.has(Interface::Overlay)
				|| self.kb_seq_done == self.kb_seq
				|| (user_value != self.kb_seq && user_value != 0)
			{
				return;
			}
			self.kb_seq_done = self.kb_seq;
			self.kb_text = self.vr.keyboard_text();
			self.kb_done = true;
			self.kb_open = false;
		} else if event_type == VREvent_KeyboardClosed || event_type == VREvent_KeyboardClosed_Global {
			self.kb_open = false;
		}
	}

	/// The sally driver's compare mode: Valve's poses and the driver's state over UDP
	fn poll_driver(&mut self, now: f64) {
		if self.drv_sock.is_none() {
			// a port another panel holds is tried again every poll
			let Ok(s) = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, self.driver_port)) else { return };
			let _ = s.set_nonblocking(true);
			self.drv_sock = Some(s);
		}
		let mut buf = [0u8; 1023];
		for _ in 0..4000 {
			let Some(sock) = &self.drv_sock else { break };
			let n = match sock.recv(&mut buf) {
				Ok(n) => n,
				Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
				// a stray ICMP error (Windows reports a closed port, driver_cmd's to a driver not listening, on the
				// next receive): keep reading, where the C++ stopped for the frame
				Err(_) => continue,
			};
			let line = &buf[..n];
			if line.first() == Some(&b'V') {
				// the serial, the driver's wall clock, valid, the position, the rotation (the shown flag after them
				// may be missing)
				let (got, v) = sscanf(line, V_LINE);
				if got < 10 {
					continue;
				}
				let (Some(Value::S(ser)), Some(Value::D(valid))) = (v.first(), v.get(2)) else { continue };
				let lf = |i: usize| if let Some(Value::Lf(x)) = v.get(i) { *x } else { 0.0 };
				let pose = Pose { p: V3::new(lf(3), lf(4), lf(5)), q: qnorm(Q4::new(lf(6), lf(7), lf(8), lf(9))) };
				let ser = String::from_utf8_lossy(ser).into_owned();
				// a serial not heard yet while there is room for it (the port takes lines from anything)
				if self.driver_valve.len() >= K_DRIVER_SERIALS && !self.driver_valve.contains_key(&ser) {
					continue;
				}
				self.driver_valve.insert(ser.clone(), (*valid != 0, pose));
				self.driver_valve_t.insert(ser, now);
				self.last_v = now;
			} else if line.first() == Some(&b'F') {
				let (got, v) = sscanf(line, F_LINE);
				if got < 18 {
					continue;
				}
				let d = |i: usize| if let Some(Value::D(x)) = v.get(i) { *x } else { 0 };
				let lf = |i: usize| if let Some(Value::Lf(x)) = v.get(i) { *x } else { 0.0 };
				let pose = |i: usize| Pose {
					p: V3::new(lf(i), lf(i + 1), lf(i + 2)),
					q: qnorm(Q4::new(lf(i + 3), lf(i + 4), lf(i + 5), lf(i + 6))),
				};
				let mut refitting = d(2) != 0;
				if let Some(at) = self.refit_asked {
					if refitting {
						// the driver has it
						self.refit_asked = None;
					} else if now - at < K_REFIT_ACK_S {
						// a line sent before it had it
						refitting = true;
					} else {
						// never taken up (the command lost): no re-align to say is done
						self.refit_asked = None;
						self.driver_refitting = false;
					}
				}
				self.drv_fit = Some(DriverFit {
					t_recv: now,
					ok: d(0) != 0,
					samples: d(1),
					refitting,
					t: pose(3),
					wfd: (d(10) != 0).then(|| pose(11)),
				});
			} else if line.first() == Some(&b'O') && line.get(1) == Some(&b' ') {
				// the output layer's settings: "O <scope> <name> <value>" each, "O end" after the list, "O error <why>"
				let text = String::from_utf8_lossy(line);
				let f: Vec<&str> = text.split_whitespace().collect();
				match f.as_slice() {
					["O", "end"] => {
						self.output = std::mem::take(&mut self.output_next);
						self.output_t = now;
					}
					["O", "error", why @ ..] => self.output_error = why.join(" "),
					["O", scope, name, value] => {
						if let Ok(v) = value.parse::<f64>() {
							self.output_next.insert((scope.to_string(), name.to_string()), v);
						}
					}
					_ => {}
				}
			} else if line.first() == Some(&b'C') {
				let (got, v) = sscanf(line, C_LINE);
				self.drv_c_t = now;
				if let (3, [Value::S(seen), Value::D(fit), Value::Lf(mm)]) = (got, v.as_slice()) {
					self.hook_seen = String::from_utf8_lossy(seen).into_owned();
					self.hook_fit = *fit != 0;
					self.hook_fit_mm = *mm;
				}
				// the words after those, each a name and its value (a driver from before one does not say it)
				let text = String::from_utf8_lossy(line);
				let words: Vec<&str> = text.split_whitespace().collect();
				let word = |name: &str| words.iter().position(|w| *w == name).and_then(|i| words.get(i + 1)).copied();
				let number = |name: &str| word(name).and_then(|v| v.parse::<i32>().ok());
				self.drv_hooks = number("hooks");
				self.drv_sally_devices = number("sally_devices").unwrap_or(0);
				self.drv_headset_sally = word("headset").map(|v| v == "sally");
				self.drv_lighthouse_headset = number("lighthouse_headset") == Some(1);
				self.drv_keep_sessions = number("keep_sessions");
			}
		}
		// a serial unheard for K_DRIVER_FORGET_S goes (its pose is stale after 0.1 s already)
		self.driver_valve_t.retain(|_, t| now - *t < K_DRIVER_FORGET_S);
		let heard = &self.driver_valve_t;
		self.driver_valve.retain(|s, _| heard.contains_key(s));
		self.hook_mode = now - self.last_v < 2.0;
	}

	/// The driver's output settings asked for again (every OUTPUT_ASK_S while someone looks at them: the Settings tab)
	pub fn output_ask(&mut self, now: f64) {
		if now - self.output_asked >= OUTPUT_ASK_S {
			self.output_asked = now;
			self.output_next.clear();
			self.driver_cmd("output");
		}
	}

	/// One of the driver's output settings set (`scope` a class or a serial); the driver answers with the list
	pub fn output_set(&mut self, scope: &str, name: &str, value: f64) {
		self.output_error.clear();
		self.output_next.clear();
		self.output.insert((scope.to_string(), name.to_string()), value);
		self.driver_cmd(&format!("output {scope} {name} {value}"));
	}

	/// A device's own value of an output setting taken away (it follows its class again); the driver answers with
	/// the list
	pub fn output_clear(&mut self, serial: &str, name: &str) {
		self.output_error.clear();
		self.output_next.clear();
		self.output.remove(&(serial.to_string(), name.to_string()));
		self.driver_cmd(&format!("output {serial} {name} -"));
	}

	/// The driver has answered an output list lately (it runs, it is a driver with the output layer)
	pub fn output_known(&self, now: f64) -> bool {
		now - self.output_t < 3.0 * OUTPUT_ASK_S
	}

	/// The driver answers commands but not with output lists: an old driver, without the output layer
	pub fn output_old_driver(&self, now: f64) -> bool {
		!self.output_known(now) && now - self.drv_c_t < 3.0 * OUTPUT_ASK_S
	}

	/// Whether the sally driver is heard: its status comes every second while SteamVR runs with it
	pub fn driver_heard(&self, now: f64) -> bool {
		now - self.drv_c_t < 3.0
	}

	/// A command to the sally driver ("seen sally", "seen valve"), once the driver's socket is open
	pub fn driver_cmd(&self, cmd: &str) {
		let Some(s) = &self.drv_sock else { return };
		let _ = s.send_to(c_str(cmd.as_bytes()), SocketAddrV4::new(Ipv4Addr::LOCALHOST, self.driver_cmd_port));
	}

	/// Once a frame: the session, the overlay's events, Valve's devices and poses, the pairs and the fit, the
	/// comparison per device, the status
	pub fn poll(&mut self, now: f64, rt: &mut Runtime) {
		self.last_poll = now;
		self.poll_driver(now);
		if !self.enabled && !self.overlay_wanted {
			if self.running {
				self.shutdown();
			}
			self.status = "comparison off".into();
			return;
		}
		if self.running && self.overlay_wanted && !self.overlay_session {
			// the overlay was turned on after a background session started: come back as an overlay app
			self.shutdown();
			self.next_try = 0.0;
		}
		if self.running && !self.overlay_wanted && self.overlay_ok {
			self.vr.destroy_overlay(self.ov_main);
			self.ov_main = 0;
			self.ov_thumb = 0;
			self.overlay_ok = false;
			self.overlay_visible = false;
			self.thumb_visible = false;
			self.ov_events.clear();
		}
		if !self.overlay_wanted {
			self.overlay_status = "off".into();
		}
		if !self.running && !self.try_init(now) {
			return;
		}
		if self.overlay_wanted
			&& self.overlay_session
			&& self.vr.has(Interface::Overlay)
			&& self.ov_main == 0
			&& now >= self.next_create
		{
			self.next_create = now + 3.0;
			self.overlay_create();
		}
		self.overlay_poll();

		// SteamVR asks background apps to let go when it exits
		while let Some(ev) = self.vr.poll_next_event() {
			self.kb_event(ev.event_type, ev.keyboard.uUserValue);
			if ev.event_type == VREvent_Quit {
				self.vr.acknowledge_quit_exiting();
				self.shutdown();
				self.status = "SteamVR exited".into();
				self.next_try = now + 10.0;
				return;
			}
		}

		if now >= self.next_props {
			self.next_props = now + 2.0;
			self.roles_by_serial.clear();
			let settings = self.vr.has(Interface::Settings);
			let vr = &mut self.vr;
			for i in 0..k_unMaxTrackedDeviceCount {
				let d = self.by_index.entry(i as i32).or_default();
				d.index = i as i32;
				d.cls = vr.tracked_device_class(i);
				if d.cls == TrackedDeviceClass_Invalid {
					d.serial.clear();
					continue;
				}
				d.system = vr.string_tracked_device_property(i, Prop_TrackingSystemName_String);
				d.serial = vr.string_tracked_device_property(i, Prop_SerialNumber_String);
				// which driver owns it: the sally driver can call its devices "lighthouse" (driver.tracking_system,
				// for Space Calibrator), so the tracking system name alone does not say they are Valve's
				let root = vr.string_tracked_device_property(i, Prop_ResourceRoot_String);
				let reg = vr.string_tracked_device_property(i, Prop_RegisteredDeviceType_String);
				d.ours_driver = root == "sally" || reg.starts_with("sally/");
				d.role.clear();
				if settings && d.cls == TrackedDeviceClass_GenericTracker && !d.serial.is_empty() {
					// SteamVR keeps tracker roles in steamvr.vrsettings, section "trackers"
					let key = format!("/devices/htc/vive_tracker{}", d.serial);
					if let Ok(role) = vr.settings_string("trackers", &key) {
						d.role = role;
					}
				}
			}
			for d in self.by_index.values() {
				if !d.role.is_empty() && d.role != "TrackerRole_None" {
					self.roles_by_serial.insert(d.serial.clone(), d.role.clone());
				}
			}
			// a device neither SteamVR nor the driver lists now, with nothing of it left to show (no trail, nothing in
			// the last 10 s), is let go: the V lines can name any serial, and each one named was kept for good
			let (listed, heard) = (&self.by_index, &self.driver_valve);
			self.devs.retain(|s, d| {
				d.valid
					|| !d.trail.is_empty()
					|| !d.err_window.is_empty()
					|| heard.contains_key(s)
					|| listed.values().any(|b| b.serial == *s)
			});
		}

		let mut poses = [TrackedDevicePose_t::default(); k_unMaxTrackedDeviceCount as usize];
		self.vr.device_to_absolute_tracking_pose(TrackingUniverseRawAndUncalibrated, 0.0, &mut poses);
		if let Some(log) = &mut self.check_log {
			log.rows(&self.by_index, &poses);
		}
		self.have_hmd = false;
		for d in self.devs.values_mut() {
			d.valid = false;
		}
		let mut lighthouse_devices = 0;
		for bi in self.by_index.values() {
			let Some(p) = poses.get(bi.index as usize) else { continue };
			if bi.cls == TrackedDeviceClass_Invalid || p.bDeviceIsConnected == 0 {
				continue;
			}
			let ok = p.bPoseIsValid != 0 && p.eTrackingResult == TrackingResult_Running_OK;
			if bi.cls == TrackedDeviceClass_HMD && p.bPoseIsValid != 0 {
				self.have_hmd = true;
				self.hmd_raw = from_mat34(&p.mDeviceToAbsoluteTracking);
			}
			if bi.system != "lighthouse" || bi.serial.is_empty() || bi.ours_driver {
				continue;
			}
			if bi.cls != TrackedDeviceClass_GenericTracker && bi.cls != TrackedDeviceClass_Controller {
				continue;
			}
			lighthouse_devices += 1;
			let d = self.devs.entry(bi.serial.clone()).or_default();
			d.index = bi.index;
			d.cls = bi.cls;
			d.system = bi.system.clone();
			d.serial = bi.serial.clone();
			d.role = bi.role.clone();
			if self.hook_mode {
				// OpenVR shows the sally driver's substitute here; Valve's own pose comes from the driver below
				continue;
			}
			d.valid = ok;
			if ok {
				d.raw = from_mat34(&p.mDeviceToAbsoluteTracking);
			}
		}
		if self.hook_mode {
			for (ser, &(valid, pose)) in &self.driver_valve {
				let t = self.driver_valve_t.get(ser);
				let d = self.devs.entry(ser.clone()).or_default();
				d.serial = ser.clone();
				if d.system.is_empty() {
					d.system = "lighthouse".into();
				}
				d.valid = valid && t.is_some_and(|&t| now - t < 0.1);
				if d.valid {
					d.raw = pose;
				}
			}
		}

		// the driver's fit, where it sends one (its compare mode): Valve's devices are compared through the alignment
		// SteamVR shows sally's through. Leaving it (no V lines for 2 s: OpenVR's raw universe again) the panel aligns
		// anew on its own
		let driver_fit = self.drv_fit.filter(|f| self.hook_mode && f.ok && now - f.t_recv < 3.0);
		match driver_fit {
			Some(f) => {
				if !self.fit_from_driver {
					self.forget_fit();
					self.fit_from_driver = true;
					rt.log(
						EvLevel::Good,
						"SteamVR",
						"comparing through the driver's alignment (the one SteamVR shows sally's poses through)",
						-1.0,
						false,
					);
				}
				if self.driver_refitting && !f.refitting {
					let t = format!("the driver re-aligned (fit median {:.0} mm)", self.hook_fit_mm);
					rt.log(EvLevel::Good, "SteamVR", &t, -1.0, false);
					// what was measured through the old alignment goes with it, as with the panel's own re-align
					self.forget_measures();
				}
				self.driver_refitting = f.refitting;
				self.fit = compose(&zup_from_yup(), &inverse(&f.t));
				self.fit_ok = true;
				self.fit_median = self.hook_fit_mm * 1e-3;
				self.fit_n = f.samples;
			}
			None if self.fit_from_driver => {
				self.fit_from_driver = false;
				self.driver_refitting = false;
				self.forget_fit();
			}
			None => {}
		}

		// pair with our devices by serial, collect fit samples (the panel's own fit)
		self.compared = 0;
		for v in self.devs.values_mut() {
			v.have_ours = false;
			if !v.valid {
				continue;
			}
			// the last of ours with the serial, as the C++'s loop kept it
			let Some(o) = rt.devices.values().rfind(|od| od.serial == v.serial) else { continue };
			if !o.have_pose || now - o.last_pose_wall > 0.1 || o.state != "tracking" {
				continue;
			}
			self.compared += 1;
			// a pair when the device moved 3 cm, and every 2 s while it is still (the driver's rule): a still device is
			// evidence of the frame too, and without its pairs whatever moves most fills the window
			if !self.fit_from_driver
				&& (!v.sampled || len(v.raw.p - v.last_sample) > 0.03 || now - v.last_sample_t > 2.0)
			{
				self.samples_src.push(v.raw.p);
				self.samples_dst.push(o.pos);
				self.samples_t.push(now);
				self.samples_dev.push(v.serial.clone());
				v.last_sample = v.raw.p;
				v.last_sample_t = now;
				v.sampled = true;
			}
			if self.fit_ok {
				v.ours = compose(&self.fit, &v.raw);
				v.have_ours = true;
				v.err_mm = len(v.ours.p - o.pos) * 1000.0;
				v.err_window.push_back((now, v.err_mm));
				// rotation: learn the constant frame offset between the two device frames, report the rest
				let mut off = qmul(qconj(o.rot), v.ours.q);
				let r = v.rot_offset;
				if v.rot_offset_n > 0 && (off.w * r.w + off.x * r.x + off.y * r.y + off.z * r.z) < 0.0 {
					off = Q4::new(-off.w, -off.x, -off.y, -off.z);
				}
				if v.rot_offset_n < 300 {
					let a = 1.0 / (v.rot_offset_n + 1) as f64;
					v.rot_offset = qnorm(Q4::new(
						r.w * (1.0 - a) + off.w * a,
						r.x * (1.0 - a) + off.x * a,
						r.y * (1.0 - a) + off.y * a,
						r.z * (1.0 - a) + off.z * a,
					));
					if v.rot_offset_n == 0 {
						v.rot_offset = off;
					}
					v.rot_offset_n += 1;
				}
				let deg = qangle(qmul(qconj(qmul(o.rot, v.rot_offset)), v.ours.q)) * 57.29578;
				if v.rot_offset_n >= 60 {
					v.err_deg_ema = if v.err_deg_ema < 0.0 { deg } else { v.err_deg_ema * 0.97 + deg * 0.03 };
				}
				if v.trail.back().is_none_or(|b| now - b.t > 1.0 / 90.0) {
					v.trail.push_back(TrailPoint { p: v.ours.p, t: now, kind: 0 });
				}
			}
		}
		let keep = self.trail_seconds;
		let e = &mut self.scratch;
		for d in self.devs.values_mut() {
			while d.trail.front().is_some_and(|p| now - p.t > keep) {
				d.trail.pop_front();
			}
			// the last 10 s, compared this frame or not: one no longer compared kept its last 10 s, and showed their
			// median as now's, for good
			while d.err_window.front().is_some_and(|&(t, _)| now - t > 10.0) {
				d.err_window.pop_front();
			}
			let w = &d.err_window;
			if w.len() >= 10 {
				e.clear();
				e.extend(w.iter().map(|&(_, mm)| mm));
				let k = (e.len() as f64 * 0.9) as usize;
				nth_element(e, k);
				d.err_p90 = e[k];
				let m = e.len() / 2;
				nth_element(e, m);
				d.err_med = e[m];
			} else {
				d.err_med = -1.0;
				d.err_p90 = -1.0;
			}
		}

		// forget pairs older than the fit window (and keep at most 3000)
		let mut stale = 0;
		while stale < self.samples_t.len()
			&& (now - self.samples_t[stale] > self.fit_window_s || self.samples_t.len() - stale > 3000)
		{
			stale += 1;
		}
		if stale > 0 {
			self.samples_src.drain(..stale);
			self.samples_dst.drain(..stale);
			self.samples_t.drain(..stale);
			self.samples_dev.drain(..stale);
		}
		let n = self.samples_src.len() as i32;
		let fit_due = !self.fit_from_driver && n as usize >= K_FIT_PAIRS && now - self.last_fit > 2.0;
		// the devices the pairs are of, for a fit that is due and for the status while there is none (a set of up to
		// 3000 serials every frame otherwise)
		let devices = if fit_due || !self.fit_ok { self.samples_dev.iter().collect::<BTreeSet<_>>().len() } else { 0 };
		let need_devices = K_FIT_DEVICES.min((self.compared as usize).max(1));
		if fit_due && devices >= need_devices {
			if let Some((f, med, kept)) = fit_rigid_robust(&self.samples_src, &self.samples_dst, 0.03) {
				if self.fit_ok {
					// how far the new alignment moves a point in the middle of the play area
					let mut c = V3::default();
					for s in &self.samples_src {
						c += *s;
					}
					c = c * (1.0 / n as f64);
					let moved = len(xform(&f, c) - xform(&self.fit, c));
					if moved > 0.02 {
						let mm = (moved * 1000.0 + 0.5) as i32;
						let t = format!("Valve's frame moved {mm} mm against ours (re-aligned)");
						rt.log(EvLevel::Warn, "SteamVR", &t, -1.0, false);
					}
				} else {
					let mm = (med * 1000.0 + 0.5) as i32;
					let t = format!("aligned Valve's frame to ours ({n} samples, median {mm} mm)");
					rt.log(EvLevel::Good, "SteamVR", &t, -1.0, false);
				}
				self.fit = f;
				self.fit_median = med;
				self.fit_n = n;
				self.fit_kept = kept;
				self.fit_ok = true;
			}
			self.last_fit = now;
		}
		// OpenVR's raw universe into sally's world; through the driver's fit, out of it into the frame's driver space
		// first
		let wfd = driver_fit.and_then(|f| f.wfd).filter(|_| self.fit_from_driver).unwrap_or_default();
		let ours_from_raw = compose(&self.fit, &inverse(&wfd));
		if self.have_hmd && self.fit_ok {
			self.hmd_ours = compose(&ours_from_raw, &self.hmd_raw);
		}
		// the chaperone's walls, as the headset
		if now >= self.next_walls {
			self.next_walls = now + WALLS_S;
			self.walls_raw = self.read_walls();
		}
		self.walls.clear();
		if self.fit_ok {
			self.walls.extend(self.walls_raw.iter().map(|q| q.map(|p| xform(&ours_from_raw, p))));
		}
		self.buzz_poll(now);

		self.status = if lighthouse_devices == 0 {
			"SteamVR connected, Valve's lighthouse driver has no devices".into()
		} else if !self.fit_ok {
			format!("aligning to Valve's frame ({n}/{K_FIT_PAIRS} samples from {devices} devices, move around)")
		} else if self.fit_from_driver {
			format!("comparing {} devices with Valve's, through the driver's alignment", self.compared)
		} else {
			format!("comparing {} devices with Valve's", self.compared)
		};
	}
}

#[cfg(test)]
mod tests;
