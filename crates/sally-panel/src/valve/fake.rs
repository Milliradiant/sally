//! A stand-in for openvr_api and SteamVR, for the tests and the headless renderer: the C++ panel's
//! test/fake_openvr.cpp (as test/test_panel.py drives it) in Rust, behind `Vr`. It plays Valve's driver: the devices
//! it is given, their poses in sally's frame (fed by the caller: `SteamVrState::set_pose`, or `follow` for a
//! runtime's), reported in OpenVR's raw universe, a rotated, shifted Y-up frame, with 2 mm of noise on each axis;
//! tracker roles in its settings; a dashboard overlay that is visible when told so and takes textures; SteamVR's
//! keyboard, which answers every request at once with the text it is given ("Bench tracker"); a chaperone's walls in
//! its standing universe, and the haptic pulses it was asked for.
//!
//! `FakeVr::new` gives the `Vr` for a ValveCompare and a `SteamVr`, the caller's handle to the same state: what SteamVR
//! has (devices, settings, events to deliver, failures to report) and what the panel did to it (`calls`, the textures,
//! the keyboard's requests). The state must not be held locked while the panel is called (the fake locks it for each
//! call).

use std::collections::{BTreeMap, VecDeque};
use std::f64::consts::PI;
use std::sync::{Arc, Mutex, MutexGuard};

use sally_math::math3d::{compose, qmul, qnorm, Pose, Q4, V3};

// the constants by their module in patterns: a misspelt one cannot become a binding that takes all
use crate::openvr::{self, *};
use crate::runtime::Runtime;

/// The handles CreateDashboardOverlay gives: the main overlay and its thumbnail (the C++ fake's)
pub const MAIN: VROverlayHandle_t = 1;
pub const THUMB: VROverlayHandle_t = 2;

/// openvr_api beside the panel
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Library {
	#[default]
	Usable,
	/// not there
	Missing,
	/// without its entry points
	Unusable,
}

/// A device of the fake SteamVR
#[derive(Clone, Debug, PartialEq)]
pub struct FakeDevice {
	pub serial: String,
	pub class: ETrackedDeviceClass,
	/// Prop_TrackingSystemName_String: "lighthouse", Valve's driver
	pub system: String,
	/// Prop_ResourceRoot_String and Prop_RegisteredDeviceType_String: empty for Valve's devices ("sally" and
	/// "sally/<serial>" would be the sally driver's)
	pub resource_root: String,
	pub registered_type: String,
	/// where it is, in sally's frame; None while it has no pose (reported as not connected)
	pub pose: Option<Pose>,
	/// Valve's frame of the device against sally's: its raw rotation is frame.q * pose.q * rot_offset
	pub rot_offset: Q4,
	pub tracking_result: ETrackingResult,
}

impl FakeDevice {
	/// One of Valve's lighthouse devices, without a pose yet
	pub fn valve(serial: &str, class: ETrackedDeviceClass) -> FakeDevice {
		FakeDevice {
			serial: serial.into(),
			class,
			system: "lighthouse".into(),
			resource_root: String::new(),
			registered_type: String::new(),
			pose: None,
			rot_offset: Q4::default(),
			tracking_result: TrackingResult_Running_OK,
		}
	}
}

/// The C++ fake's classes by the runtime's subtype
pub fn class_of(subtype: &str) -> ETrackedDeviceClass {
	match subtype {
		"vive_hmd" | "index_hmd" => TrackedDeviceClass_HMD,
		"knuckles_l" | "knuckles_r" | "wand" => TrackedDeviceClass_Controller,
		_ => TrackedDeviceClass_GenericTracker,
	}
}

/// The C++ fake's frame: libsurvive's Z up turned to Y up ((x, y, z) to (x, z, -y)), then 30 degrees of yaw about the
/// new Y, shifted by (0.5, 1.0, -0.3): raw = frame * ours
pub fn cpp_fake_frame() -> Pose {
	let s = 0.5f64.sqrt();
	let zup2yup = Q4::new(s, -s, 0.0, 0.0);
	let yaw = Q4::new((PI / 12.0).cos(), 0.0, (PI / 12.0).sin(), 0.0);
	Pose { p: V3::new(0.5, 1.0, -0.3), q: qmul(yaw, zup2yup) }
}

/// Standard normal numbers, the same sequence for the same seed: splitmix64's bits in Box and Muller's pairs
#[derive(Clone, Debug)]
pub struct Gaussian {
	state: u64,
	spare: Option<f64>,
}

impl Gaussian {
	pub fn new(seed: u64) -> Gaussian {
		Gaussian { state: seed, spare: None }
	}

	fn bits(&mut self) -> u64 {
		self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
		let mut z = self.state;
		z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
		z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
		z ^ (z >> 31)
	}

	/// Uniform in (0, 1)
	fn uniform(&mut self) -> f64 {
		((self.bits() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
	}

	pub fn sample(&mut self) -> f64 {
		if let Some(s) = self.spare.take() {
			return s;
		}
		let r = (-2.0 * self.uniform().ln()).sqrt();
		let a = 2.0 * PI * self.uniform();
		self.spare = Some(r * a.sin());
		r * a.cos()
	}
}

/// A texture the panel handed over
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HandedTexture {
	pub overlay: VROverlayHandle_t,
	/// the OpenGL texture's name
	pub gl_texture: u32,
	pub texture_type: ETextureType,
	pub color_space: EColorSpace,
}

/// What the fake SteamVR has and what the panel did to it
pub struct SteamVrState {
	pub library: Library,
	/// SteamVR runs: init attaches to it (and fails with NoServerForBackgroundApp when it does not)
	pub running: bool,
	/// the EVRInitError every init gets instead, when set
	pub init_error: Option<EVRInitError>,
	/// the EVRInitError an Overlay app's init gets instead, when set
	pub overlay_init_error: Option<EVRInitError>,
	/// the interfaces SteamVR does not have (VR_GetGenericInterface fails with InterfaceNotFound)
	pub missing: Vec<Interface>,
	/// the session's application type, None when there is none
	pub session: Option<EVRApplicationType>,
	/// the interfaces the session fetched
	pub tables: Vec<Interface>,

	/// Valve's devices, by index
	pub devices: Vec<FakeDevice>,
	/// sally's frame to OpenVR's raw universe
	pub frame: Pose,
	/// the position noise added to every pose, on each axis (m)
	pub noise_m: f64,
	pub noise: Gaussian,
	/// steamvr.vrsettings: (section, key) -> value
	pub settings: BTreeMap<(String, String), String>,
	/// for PollNextEvent, and for PollNextOverlayEvent by overlay
	pub system_events: VecDeque<VrEvent>,
	pub overlay_events: VecDeque<(VROverlayHandle_t, VrEvent)>,

	/// the overlays that exist
	pub overlays: Vec<VROverlayHandle_t>,
	/// CreateDashboardOverlay's error, when set
	pub create_error: EVROverlayError,
	pub overlay_visible: bool,
	pub thumb_visible: bool,
	/// SetOverlayTexture's error, when set
	pub texture_error: EVROverlayError,
	pub textures: Vec<HandedTexture>,

	/// what the keyboard answers every request with (the C++ fake's FAKE_OPENVR_KEYBOARD)
	pub keyboard_text: String,
	/// the keyboard answers each request at once: KeyboardDone, then KeyboardClosed, on the overlay's queue
	pub keyboard_answers: bool,
	/// and on the system queue as well
	pub keyboard_on_system_queue: bool,
	/// ShowKeyboardForOverlay fails with KeyboardAlreadyInUse until HideKeyboard
	pub keyboard_in_use: bool,
	/// ShowKeyboardForOverlay's error for every request, when set
	pub keyboard_error: EVROverlayError,
	pub keyboard_requests: Vec<(VROverlayHandle_t, KeyboardRequest)>,
	/// GetKeyboardText calls
	pub keyboard_reads: u32,
	pub quits_acknowledged: u32,
	/// the chaperone's walls (its standing universe), and where OpenVR's raw universe sits in that one
	pub walls: Vec<HmdQuad_t>,
	pub standing_from_raw: Pose,
	/// TriggerHapticPulse's calls: device index, axis, microseconds
	pub pulses: Vec<(u32, u32, u16)>,
	/// the calls that change something, in order: "init background", "interface IVRSystem_026", "create sally.panel
	/// sally", "width 1 2.5", "flag 1 131072 true", "texture 1 7", "keyboard 1 #1", "hide keyboard", "acknowledge
	/// quit", "destroy 1", "shutdown", ...
	pub calls: Vec<String>,
}

impl Default for SteamVrState {
	fn default() -> SteamVrState {
		SteamVrState {
			library: Library::Usable,
			running: true,
			init_error: None,
			overlay_init_error: None,
			missing: Vec::new(),
			session: None,
			tables: Vec::new(),
			devices: Vec::new(),
			frame: cpp_fake_frame(),
			noise_m: 0.002,
			noise: Gaussian::new(1),
			settings: BTreeMap::new(),
			system_events: VecDeque::new(),
			overlay_events: VecDeque::new(),
			overlays: Vec::new(),
			create_error: VROverlayError_None,
			overlay_visible: false,
			thumb_visible: false,
			texture_error: VROverlayError_None,
			textures: Vec::new(),
			keyboard_text: "Bench tracker".into(),
			keyboard_answers: true,
			keyboard_on_system_queue: false,
			keyboard_in_use: false,
			keyboard_error: VROverlayError_None,
			keyboard_requests: Vec::new(),
			keyboard_reads: 0,
			quits_acknowledged: 0,
			walls: room_walls(4.0, 3.0, 2.4),
			standing_from_raw: Pose { p: V3::new(0.3, 0.0, -0.2), q: Q4::new(0.9659258, 0.0, 0.258819, 0.0) },
			pulses: Vec::new(),
			calls: Vec::new(),
		}
	}
}

/// A room's four walls in a standing universe (Y up, the floor at 0), `w` by `d` metres about its middle, `h` high:
/// each wall as SteamVR's chaperone has one, from a floor corner up, along the top and down to the other floor corner
pub fn room_walls(w: f32, d: f32, h: f32) -> Vec<HmdQuad_t> {
	let (x, z) = (w / 2.0, d / 2.0);
	let floor = [[-x, -z], [x, -z], [x, z], [-x, z]];
	(0..4)
		.map(|i| {
			let (a, b) = (floor[i], floor[(i + 1) % 4]);
			let c = |p: [f32; 2], y: f32| HmdVector3_t { v: [p[0], y, p[1]] };
			HmdQuad_t { vCorners: [c(a, 0.0), c(a, h), c(b, h), c(b, 0.0)] }
		})
		.collect()
}

impl SteamVrState {
	/// The device of that serial: one of Valve's trackers at the next index when it is new
	pub fn device(&mut self, serial: &str) -> &mut FakeDevice {
		let i = match self.devices.iter().position(|d| d.serial == serial) {
			Some(i) => i,
			None => {
				self.devices.push(FakeDevice::valve(serial, TrackedDeviceClass_GenericTracker));
				self.devices.len() - 1
			}
		};
		&mut self.devices[i]
	}

	/// Valve's driver sees the device at `pose` (sally's frame; None: no pose)
	pub fn set_pose(&mut self, serial: &str, class: ETrackedDeviceClass, pose: Option<Pose>) {
		let d = self.device(serial);
		d.class = class;
		d.pose = pose;
	}

	/// Valve's driver sees what the runtime sees: every runtime device with a serial and a pose, by serial, its class
	/// by the runtime's subtype (the C++ fake took them from the runtime's Q and DEV lines)
	pub fn follow(&mut self, rt: &Runtime) {
		for d in rt.devices.values() {
			if d.serial.starts_with('-') || !d.have_pose {
				continue;
			}
			self.set_pose(&d.serial, class_of(&d.subtype), Some(Pose { p: d.pos, q: d.rot }));
		}
	}

	/// The tracker's role in steamvr.vrsettings
	pub fn set_tracker_role(&mut self, serial: &str, role: &str) {
		self.settings.insert(("trackers".into(), format!("/devices/htc/vive_tracker{serial}")), role.into());
	}

	pub fn post_system_event(&mut self, ev: VrEvent) {
		self.system_events.push_back(ev);
	}

	pub fn post_overlay_event(&mut self, overlay: VROverlayHandle_t, ev: VrEvent) {
		self.overlay_events.push_back((overlay, ev));
	}

	/// The panel calls into a table only once it has it (the C++ would call through a null pointer)
	fn table(&self, which: Interface) {
		assert!(self.tables.contains(&which), "a call into {} without its table", which.version());
	}
}

/// An event of a type, its data all zeros
pub fn event(event_type: EVREventType) -> VrEvent {
	VrEvent { event_type, ..VrEvent::default() }
}

/// A mouse event (MouseMove, MouseButtonDown, MouseButtonUp) at x, y in the overlay's mouse scale, with its buttons
pub fn mouse_event(event_type: EVREventType, x: f32, y: f32, button: EVRMouseButton) -> VrEvent {
	VrEvent { event_type, mouse: VREvent_Mouse_t { x, y, button, cursorIndex: 0 }, ..VrEvent::default() }
}

/// A scroll event (ScrollSmooth, ScrollDiscrete)
pub fn scroll_event(event_type: EVREventType, xdelta: f32, ydelta: f32) -> VrEvent {
	let scroll = VREvent_Scroll_t { xdelta, ydelta, unused: 0, viewportscale: 0.0, cursorIndex: 0 };
	VrEvent { event_type, scroll, ..VrEvent::default() }
}

/// A keyboard event (KeyboardDone, KeyboardClosed, KeyboardClosed_Global) of a request
pub fn keyboard_event(event_type: EVREventType, user_value: u64, overlay: VROverlayHandle_t) -> VrEvent {
	let keyboard = VREvent_Keyboard_t { cNewInput: [0; 8], uUserValue: user_value, overlayHandle: overlay };
	VrEvent { event_type, keyboard, ..VrEvent::default() }
}

/// The rotation's matrix (rows) with the position in the last column, in floats (the C++ fake's)
pub fn mat34(q: Q4, p: V3) -> HmdMatrix34_t {
	let (w, x, y, z) = (q.w, q.x, q.y, q.z);
	let r = [
		[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
		[2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
		[2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
	];
	let t = [p.x, p.y, p.z];
	let mut m = HmdMatrix34_t::default();
	for (row, (rr, tt)) in m.m.iter_mut().zip(r.iter().zip(t)) {
		for (c, v) in rr.iter().enumerate() {
			row[c] = *v as f32;
		}
		row[3] = tt as f32;
	}
	m
}

/// The application type's name in `calls`
fn app_name(app: EVRApplicationType) -> String {
	match app {
		openvr::VRApplication_Background => "background".into(),
		openvr::VRApplication_Overlay => "overlay".into(),
		_ => app.to_string(),
	}
}

/// The caller's handle to the fake SteamVR's state
#[derive(Clone, Default)]
pub struct SteamVr(Arc<Mutex<SteamVrState>>);

impl SteamVr {
	/// The state, to look at or to change (a test that failed while holding it does not stop the next look)
	pub fn lock(&self) -> MutexGuard<'_, SteamVrState> {
		self.0.lock().unwrap_or_else(|e| e.into_inner())
	}
}

/// The fake openvr_api, for `ValveCompare::with_vr`
pub struct FakeVr {
	steamvr: SteamVr,
}

impl FakeVr {
	/// A SteamVR that runs, with Valve's lighthouse driver and no devices yet, and the handle to it
	pub fn new() -> (FakeVr, SteamVr) {
		let steamvr = SteamVr::default();
		(FakeVr { steamvr: steamvr.clone() }, steamvr)
	}
}

impl Vr for FakeVr {
	fn load(&mut self) -> Result<(), LoadError> {
		match self.steamvr.lock().library {
			Library::Usable => Ok(()),
			Library::Missing => Err(LoadError::Missing),
			Library::Unusable => Err(LoadError::Unusable),
		}
	}

	fn init(&mut self, app: EVRApplicationType) -> Result<(), EVRInitError> {
		let mut s = self.steamvr.lock();
		s.calls.push(format!("init {}", app_name(app)));
		if !s.running {
			return Err(VRInitError_Init_NoServerForBackgroundApp);
		}
		if let Some(e) = s.init_error {
			return Err(e);
		}
		if let (openvr::VRApplication_Overlay, Some(e)) = (app, s.overlay_init_error) {
			return Err(e);
		}
		s.session = Some(app);
		s.tables.clear();
		Ok(())
	}

	fn shutdown(&mut self) -> bool {
		let mut s = self.steamvr.lock();
		s.calls.push("shutdown".into());
		s.session = None;
		s.tables.clear();
		// an application's overlays go with its connection
		s.overlays.clear();
		s.overlay_events.clear();
		true
	}

	fn interface(&mut self, which: Interface) -> Result<(), EVRInitError> {
		let mut s = self.steamvr.lock();
		s.calls.push(format!("interface {}", which.version()));
		if s.session.is_none() {
			return Err(VRInitError_Init_NotInitialized);
		}
		if s.missing.contains(&which) {
			return Err(VRInitError_Init_InterfaceNotFound);
		}
		if !s.tables.contains(&which) {
			s.tables.push(which);
		}
		Ok(())
	}

	fn has(&self, which: Interface) -> bool {
		self.steamvr.lock().tables.contains(&which)
	}

	fn tracked_device_class(&mut self, index: TrackedDeviceIndex_t) -> ETrackedDeviceClass {
		let s = self.steamvr.lock();
		s.table(Interface::System);
		s.devices.get(index as usize).map_or(TrackedDeviceClass_Invalid, |d| d.class)
	}

	fn string_tracked_device_property(&mut self, index: TrackedDeviceIndex_t, prop: ETrackedDeviceProperty) -> String {
		let s = self.steamvr.lock();
		s.table(Interface::System);
		let Some(d) = s.devices.get(index as usize) else { return String::new() };
		match prop {
			openvr::Prop_TrackingSystemName_String => d.system.clone(),
			openvr::Prop_SerialNumber_String => d.serial.clone(),
			openvr::Prop_ResourceRoot_String => d.resource_root.clone(),
			openvr::Prop_RegisteredDeviceType_String => d.registered_type.clone(),
			_ => String::new(),
		}
	}

	fn device_to_absolute_tracking_pose(
		&mut self, _origin: ETrackingUniverseOrigin, _predicted_seconds: f32, poses: &mut [TrackedDevicePose_t],
	) {
		let mut guard = self.steamvr.lock();
		let s = &mut *guard;
		s.table(Interface::System);
		poses.fill(TrackedDevicePose_t::default());
		for (p, d) in poses.iter_mut().zip(&s.devices) {
			let Some(pose) = d.pose else { continue };
			let raw = compose(&s.frame, &pose);
			let mut at = raw.p;
			at.x += s.noise_m * s.noise.sample();
			at.y += s.noise_m * s.noise.sample();
			at.z += s.noise_m * s.noise.sample();
			p.mDeviceToAbsoluteTracking = mat34(qnorm(qmul(raw.q, d.rot_offset)), at);
			p.bPoseIsValid = 1;
			p.bDeviceIsConnected = 1;
			p.eTrackingResult = d.tracking_result;
		}
	}

	fn poll_next_event(&mut self) -> Option<VrEvent> {
		let mut s = self.steamvr.lock();
		s.table(Interface::System);
		s.system_events.pop_front()
	}

	fn acknowledge_quit_exiting(&mut self) {
		let mut s = self.steamvr.lock();
		s.table(Interface::System);
		s.calls.push("acknowledge quit".into());
		s.quits_acknowledged += 1;
	}

	fn raw_zero_to_standing(&mut self) -> Option<HmdMatrix34_t> {
		let s = self.steamvr.lock();
		s.table(Interface::System);
		Some(mat34(s.standing_from_raw.q, s.standing_from_raw.p))
	}

	fn trigger_haptic_pulse(&mut self, index: TrackedDeviceIndex_t, axis: u32, micros: u16) {
		let mut s = self.steamvr.lock();
		s.table(Interface::System);
		s.pulses.push((index, axis, micros));
	}

	fn live_collision_bounds(&mut self) -> Option<Vec<HmdQuad_t>> {
		let s = self.steamvr.lock();
		s.table(Interface::ChaperoneSetup);
		Some(s.walls.clone())
	}

	fn settings_string(&mut self, section: &str, key: &str) -> Result<String, EVRSettingsError> {
		let s = self.steamvr.lock();
		s.table(Interface::Settings);
		s.settings.get(&(section.into(), key.into())).cloned().ok_or(VRSettingsError_UnsetSettingHasNoDefault)
	}

	fn create_dashboard_overlay(
		&mut self, key: &str, name: &str,
	) -> Result<(VROverlayHandle_t, VROverlayHandle_t), EVROverlayError> {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("create {key} {name}"));
		if s.create_error != VROverlayError_None {
			return Err(s.create_error);
		}
		if s.overlays.contains(&MAIN) {
			return Err(VROverlayError_KeyInUse);
		}
		s.overlays = vec![MAIN, THUMB];
		Ok((MAIN, THUMB))
	}

	fn destroy_overlay(&mut self, handle: VROverlayHandle_t) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("destroy {handle}"));
		// a dashboard overlay's thumbnail goes with it
		let gone = if handle == MAIN { vec![MAIN, THUMB] } else { vec![handle] };
		s.overlays.retain(|h| !gone.contains(h));
		s.overlay_events.retain(|(h, _)| !gone.contains(h));
		VROverlayError_None
	}

	fn set_overlay_width_in_meters(&mut self, handle: VROverlayHandle_t, metres: f32) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("width {handle} {metres}"));
		VROverlayError_None
	}

	fn set_overlay_input_method(&mut self, handle: VROverlayHandle_t, method: VROverlayInputMethod) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("input method {handle} {method}"));
		VROverlayError_None
	}

	fn set_overlay_mouse_scale(&mut self, handle: VROverlayHandle_t, scale: &HmdVector2_t) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("mouse scale {handle} {} {}", scale.v[0], scale.v[1]));
		VROverlayError_None
	}

	fn set_overlay_flag(&mut self, handle: VROverlayHandle_t, flag: VROverlayFlags, enabled: bool) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("flag {handle} {flag} {enabled}"));
		VROverlayError_None
	}

	fn is_overlay_visible(&mut self, handle: VROverlayHandle_t) -> bool {
		let s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.overlays.contains(&handle)
			&& match handle {
				MAIN => s.overlay_visible,
				THUMB => s.thumb_visible,
				_ => false,
			}
	}

	fn poll_next_overlay_event(&mut self, handle: VROverlayHandle_t) -> Option<VrEvent> {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		let i = s.overlay_events.iter().position(|(h, _)| *h == handle)?;
		s.overlay_events.remove(i).map(|(_, ev)| ev)
	}

	fn set_overlay_texture(&mut self, handle: VROverlayHandle_t, texture: &Texture_t) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		let gl_texture = texture.handle.addr() as u32;
		s.calls.push(format!("texture {handle} {gl_texture}"));
		s.textures.push(HandedTexture {
			overlay: handle,
			gl_texture,
			texture_type: texture.eType,
			color_space: texture.eColorSpace,
		});
		s.texture_error
	}

	fn show_keyboard_for_overlay(&mut self, handle: VROverlayHandle_t, request: &KeyboardRequest) -> EVROverlayError {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push(format!("keyboard {handle} #{}", request.user_value));
		s.keyboard_requests.push((handle, request.clone()));
		if s.keyboard_in_use {
			return VROverlayError_KeyboardAlreadyInUse;
		}
		if s.keyboard_error != VROverlayError_None {
			return s.keyboard_error;
		}
		if s.keyboard_answers {
			for t in [VREvent_KeyboardDone, VREvent_KeyboardClosed] {
				let ev = keyboard_event(t, request.user_value, handle);
				s.post_overlay_event(handle, ev);
				if s.keyboard_on_system_queue {
					s.post_system_event(ev);
				}
			}
		}
		VROverlayError_None
	}

	fn hide_keyboard(&mut self) {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.calls.push("hide keyboard".into());
		s.keyboard_in_use = false;
	}

	fn keyboard_text(&mut self) -> String {
		let mut s = self.steamvr.lock();
		s.table(Interface::Overlay);
		s.keyboard_reads += 1;
		s.keyboard_text.clone()
	}
}
