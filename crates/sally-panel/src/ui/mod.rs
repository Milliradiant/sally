//! The panel's UI, independent of what it draws into: the desktop window and the SteamVR overlay both call
//! `Panel::frame`, each with its own egui context, sharing every bit of state (tab, selection, camera, the device
//! page). The tabs (docs/sally_panel.md): Tracking (and from it the device page), Replay, Output and
//! Settings, under a bar that holds what is wanted from anywhere: what SteamVR is shown, the runtime and the
//! alignment.
//!
//! Layout is in points: the body text is 15 points, and a point is a pixel at the window's scale 1 (the overlay draws
//! the 1600 x 940 point layout at 1.25 pixels a point). The look is `theme`'s, the controls `kit`'s.

mod check;
pub mod icons;
mod kit;
mod output;
mod page;
mod replay;
mod room;
mod sensors;
mod settings_tab;
pub mod state;
pub mod theme;
mod topbar;
mod tracking;

use egui::{Color32, Id, Pos2, Rect, Vec2};

use crate::runtime::{Device, EvLevel, Runtime, Want};
use crate::settings::Settings;
use crate::valve::{ValveCompare, ValveDevice};
use theme::*;

/// Which of the panel's views a frame is for
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum View {
	Window,
	Overlay,
}

impl View {
	fn index(self) -> usize {
		match self {
			View::Window => 0,
			View::Overlay => 1,
		}
	}
}

/// What the Selected box shows
#[derive(Clone, Debug, PartialEq)]
pub enum Sel {
	/// the first active device
	None,
	/// a device by codename
	Device(String),
	/// a station by index
	Station(i32),
}

/// The key a base station's symbol is kept under among the devices' (no serial is this)
const STATION: &str = "\u{1}station";

/// A device's state as the panel shows it: the runtime's own state and diagnosis, without interpretation
#[derive(Clone, Debug)]
pub struct Health {
	/// 0 fine, 1 note (one station, back-facing hits), 2 attention, 3 problem, 4 not active
	pub level: i32,
	pub color: Color32,
	/// "tracking", "dark 0.4 s", "coasting 1.2 s", "reacquiring"
	pub state: String,
	/// the runtime's (cause, text), worst first
	pub causes: Vec<(String, String)>,
}

impl Health {
	/// hurting (severity 2) is a problem; a mild light or reflection verdict is a note (one station, back-facing
	/// hits); anything else mild needs attention
	pub fn cause_level(cause: &str, severity: i32) -> i32 {
		if severity <= 0 {
			0
		} else if severity >= 2 {
			3
		} else if cause == "light" || cause == "reflect" {
			1
		} else {
			2
		}
	}
}

pub fn level_color(level: i32) -> Color32 {
	tk().level(level)
}

/// The roles a device can be given: key, label, short label
pub const ROLES: &[(&str, &str, &str)] = &[
	("auto", "Auto", ""),
	("head", "Head", "head"),
	("chest", "Chest", "chest"),
	("waist", "Waist", "waist"),
	("left_hand", "Left hand", "L hand"),
	("right_hand", "Right hand", "R hand"),
	("left_elbow", "Left elbow", "L elbow"),
	("right_elbow", "Right elbow", "R elbow"),
	("left_shoulder", "Left shoulder", "L shoulder"),
	("right_shoulder", "Right shoulder", "R shoulder"),
	("left_knee", "Left knee", "L knee"),
	("right_knee", "Right knee", "R knee"),
	("left_foot", "Left foot", "L foot"),
	("right_foot", "Right foot", "R foot"),
	("held", "Held", "held"),
	("none", "No role", ""),
];

fn role_def(key: &str) -> (&'static str, &'static str, &'static str) {
	*ROLES.iter().find(|r| r.0 == key).unwrap_or(&ROLES[ROLES.len() - 1])
}

fn role_order(key: &str) -> usize {
	ROLES.iter().position(|r| r.0 == key).unwrap_or(ROLES.len())
}

/// A SteamVR tracker role as the panel's role
fn role_from_steamvr(r: &str) -> &'static str {
	match r {
		"TrackerRole_LeftFoot" | "TrackerRole_LeftAnkle" => "left_foot",
		"TrackerRole_RightFoot" | "TrackerRole_RightAnkle" => "right_foot",
		"TrackerRole_LeftShoulder" => "left_shoulder",
		"TrackerRole_RightShoulder" => "right_shoulder",
		"TrackerRole_LeftElbow" => "left_elbow",
		"TrackerRole_RightElbow" => "right_elbow",
		"TrackerRole_LeftKnee" => "left_knee",
		"TrackerRole_RightKnee" => "right_knee",
		"TrackerRole_Waist" => "waist",
		"TrackerRole_Chest" => "chest",
		"TrackerRole_Handed" => "held",
		"TrackerRole_LeftWrist" => "left_hand",
		"TrackerRole_RightWrist" => "right_hand",
		_ => "",
	}
}

/// A name as typed: no control characters, no spaces around it, at most 32 bytes (cut between characters)
fn clean_name(s: &str) -> String {
	let mut o: String = s.chars().filter(|&c| c >= ' ' && c != '\u{7f}').collect();
	if o.len() > 32 {
		let mut n = 32;
		while !o.is_char_boundary(n) {
			n -= 1;
		}
		o.truncate(n);
	}
	o.trim_matches(' ').to_string()
}

/// A config's model number without its build's letters ("VIVE Tracker 3.0 MV": mass production)
fn model_words(model: &str) -> &str {
	let m = model.trim();
	[" MV", " MP", " PVT", " DVT", " EVT"].iter().find_map(|x| m.strip_suffix(x)).unwrap_or(m).trim_end()
}

/// A serial's last four characters (all of it when shorter), cut between characters: a serial is the runtime's text,
/// and one cut at a byte inside a character stopped the panel
fn serial_tail(s: &str) -> &str {
	s.char_indices().nth_back(3).map_or(s, |(i, _)| &s[i..])
}

/// A device's name being typed (the Selected box, the Settings tab's list): in the view that started it, until Enter,
/// Escape or a click away
struct Rename {
	serial: String,
	view: View,
	buf: String,
	frames: u32,
	last_frame: u64,
}

pub struct Panel {
	pub rt: Runtime,
	pub vc: ValveCompare,
	pub st: Settings,
	pub version: String,
	/// set by the UI for the host: the overlay was switched on or off
	pub toggle_overlay: bool,
	/// host overrides (the headless runs): a device (codename or serial) or a station (name or index) to select
	pub select_device: String,
	pub select_station: String,
	/// host override: open the device page of this device (codename or serial; "KN1:sensors" its Sensors tab)
	pub open_page: String,
	/// the render models (SteamVR's), for the room and the replay
	pub models: crate::models::Models,

	now: f64,
	view: View,
	/// each view's frames
	frame_no: [u64; 2],
	sel: Sel,
	cam: room::Camera,
	auto_floor: f64,
	/// since when SteamVR has been running (state.rs)
	steamvr_since: Option<f64>,
	/// the sessions' files in the logs folder: when they were last added up, and their bytes (Settings)
	logs_size: Option<(f64, u64)>,
	/// the dashboard button's picture as it was last drawn, and the dot it was drawn with
	button_picture: Option<(crate::icon::Dot, egui::TextureHandle)>,
	/// the device page's device (codename), while it is open
	page: Option<String>,
	page_state: page::PageState,

	confirm_unlock: bool,
	confirm_restart: bool,
	/// Use new calibration's question is up
	confirm_apply: bool,
	/// an output slider being dragged (its scope and setting): its value, sent to the driver when the drag ends
	output_drag: Option<(String, &'static str, f64)>,
	/// the device (serial) whose own output values each class's box shows (headset, controllers, trackers); None:
	/// the class's
	output_scope: [Option<String>; 3],
	/// what the Output tab's foot line says of the setting under the pointer, in each view
	output_hint: [&'static str; 2],
	/// the pop-up that is open, and in which view; the frame it was opened in
	pop: Option<(kit::Pop, View)>,
	pop_frame: u64,
	/// the device (serial) whose place on the body is being picked in the Settings tab's list
	role_pick: Option<String>,

	rename: Option<Rename>,
	kb_error_until: f64,

	/// SteamVR's own icons of the devices (symbols.rs), and this frame's by serial in each view (STATION for a base
	/// station's)
	symbols: crate::symbols::Symbols,
	sym: [std::collections::HashMap<String, crate::symbols::Symbol>; 2],

	/// the tokens each view's context was styled with, to style it again only when they change
	style_set: [Option<theme::Tk>; 2],
	/// the Replay tab's
	rp: replay::ReplayState,
	/// the Check tab's: the check that is recording, being made, or done
	ck: crate::check::Check,
}

impl Panel {
	pub fn new(st: Settings, vc: ValveCompare) -> Panel {
		let mut rt = Runtime::new();
		rt.set_target(&st.host, st.port);
		rt.trail_seconds = st.trail_seconds;
		let mut vc = vc;
		vc.enabled = st.compare;
		vc.trail_seconds = st.trail_seconds;
		vc.overlay_wanted = st.overlay;
		vc.overlay_width_m = st.overlay_width as f32;
		vc.overlay_flip_pointer = st.overlay_flip_pointer;
		Panel {
			rt,
			vc,
			st,
			version: env!("CARGO_PKG_VERSION").into(),
			toggle_overlay: false,
			select_device: String::new(),
			select_station: String::new(),
			open_page: String::new(),
			models: crate::models::Models::default(),
			now: 0.0,
			view: View::Window,
			frame_no: [0, 0],
			sel: Sel::None,
			cam: room::Camera::default(),
			auto_floor: f64::NAN,
			steamvr_since: None,
			logs_size: None,
			button_picture: None,
			page: None,
			page_state: page::PageState::default(),
			confirm_unlock: false,
			confirm_restart: false,
			confirm_apply: false,
			output_drag: None,
			output_scope: [None, None, None],
			output_hint: ["", ""],
			pop: None,
			pop_frame: 0,
			role_pick: None,
			rename: None,
			kb_error_until: 0.0,
			symbols: crate::symbols::Symbols::default(),
			sym: [std::collections::HashMap::new(), std::collections::HashMap::new()],
			style_set: [None, None],
			rp: replay::ReplayState::default(),
			ck: crate::check::Check::default(),
		}
	}

	/// The fonts and the style for a context the panel draws into
	pub fn setup(ctx: &egui::Context) {
		theme::setup(ctx);
	}

	/// The runtime's socket and Valve's side, once a loop: before the frames
	pub fn poll(&mut self, now: f64) {
		self.now = now;
		self.rt.poll(now);
		if self.vc.overlay_width_m != self.st.overlay_width as f32 {
			self.vc.overlay_set_width(self.st.overlay_width as f32);
		}
		self.vc.poll(now, &mut self.rt);
		self.check_poll();
	}

	/// The device whose page is open (its codename)
	pub fn page(&self) -> Option<&str> {
		self.page.as_deref()
	}

	/// The device page shows its Sensors tab (else the curves)
	pub fn page_sensors_tab(&self) -> bool {
		self.page_state.sensors_tab
	}

	/// What the Selected box shows
	pub fn selection(&self) -> &Sel {
		&self.sel
	}

	/// The name the panel calls a device by (given, else by its role or type)
	pub fn device_name(&self, codename: &str) -> Option<String> {
		self.rt.devices.get(codename).map(|d| self.short_name(d))
	}

	/// The settings file, when something changed (the host calls it every few seconds and at the end)
	pub fn save_settings(&mut self) {
		if self.st.dirty {
			self.st.save();
		}
	}

	fn floor_z(&self) -> f64 {
		self.floor().0
	}

	/// The room's floor and where it is from
	fn floor(&self) -> (f64, FloorFrom) {
		floor_of(self.st.floor_z, self.vc.floor, self.auto_floor)
	}

	fn role_of(&self, d: &Device) -> &'static str {
		if let Some(r) = self.st.roles.get(&d.serial).filter(|r| *r != "auto") {
			return role_def(r).0;
		}
		self.auto_role(d)
	}

	/// The role a device has without one given in the panel: SteamVR's for it, else its type's
	fn auto_role(&self, d: &Device) -> &'static str {
		if let Some(r) = self.vc.roles_by_serial.get(&d.serial) {
			let r = role_from_steamvr(r);
			if !r.is_empty() {
				return r;
			}
		}
		match d.subtype.as_str() {
			"knuckles_l" => "left_hand",
			"knuckles_r" => "right_hand",
			"vive_hmd" | "index_hmd" => "head",
			"wand" => "held",
			_ => "none",
		}
	}

	/// The device's type, as its config says
	fn type_name(d: &Device) -> String {
		match d.subtype.as_str() {
			"tracker" | "tracker_gen2" => "VIVE Tracker".into(),
			"knuckles_l" => "Index controller L".into(),
			"knuckles_r" => "Index controller R".into(),
			"wand" => "Vive wand".into(),
			"index_hmd" => "Index headset".into(),
			"vive_hmd" => "Vive headset".into(),
			_ => format!("Device {}", d.codename),
		}
	}

	/// What a device is: a tracker by its config's model where that was read ("Tundra Tracker", "VIVE Tracker 3.0"),
	/// anything else by its type
	fn type_of(&self, d: &Device) -> String {
		if matches!(d.subtype.as_str(), "tracker" | "tracker_gen2" | "generic") {
			if let Some(m) = self.symbols.model_of(&d.serial) {
				return model_words(m).to_string();
			}
		}
		Self::type_name(d)
	}

	fn given_name(&self, d: &Device) -> Option<&str> {
		self.st.names.get(&d.serial).map(String::as_str)
	}

	/// The name without a given one: the role ("Left foot"), else the type ("Right controller", "Tracker 000A")
	fn default_name(&self, d: &Device) -> String {
		let r = self.role_of(d);
		if r != "none" && r != "held" {
			return role_def(r).1.into();
		}
		match d.subtype.as_str() {
			"knuckles_l" => "Left controller".into(),
			"knuckles_r" => "Right controller".into(),
			"wand" => "Wand".into(),
			"index_hmd" | "vive_hmd" => "Headset".into(),
			"tracker" | "tracker_gen2" => format!("Tracker {}", serial_tail(&d.serial)),
			_ => format!("Device {}", d.codename),
		}
	}

	/// The name given in the panel, else the default name
	fn short_name(&self, d: &Device) -> String {
		self.given_name(d).map(str::to_string).unwrap_or_else(|| self.default_name(d))
	}

	/// Names are kept by serial; "" (or the default name) is the default name again
	fn set_name(&mut self, serial: &str, name: &str) {
		if serial.is_empty() || serial == "-" {
			return;
		}
		let mut n = clean_name(name);
		let def = self.rt.devices.values().find(|d| d.serial == serial).map(|d| self.default_name(d));
		if Some(&n) == def.as_ref() {
			n.clear();
		}
		if self.st.names.get(serial).map(String::as_str).unwrap_or("") == n {
			return;
		}
		if n.is_empty() {
			self.st.names.remove(serial);
		} else {
			self.st.names.insert(serial.into(), n);
		}
		self.st.dirty = true;
	}

	fn health_of(&self, d: &Device) -> Health {
		let now = self.now;
		if !d.active {
			return Health { level: 4, color: tk().faint, state: "not active".into(), causes: Vec::new() };
		}
		let level = self.level_of(d);
		let state = if d.state == "dark" || d.state == "coasting" {
			format!("{} {:.1} s", d.state, if d.light_age >= 0.0 { d.light_age } else { now - d.state_since })
		} else if d.state == "none" {
			"starting".into()
		} else {
			d.state.clone()
		};
		let causes = if now - d.diag.wall < 2.0 { d.diag.causes.clone() } else { Vec::new() };
		Health { level, color: level_color(level), state, causes }
	}

	/// The level of health_of alone, without the texts it makes (the counts of flagged devices take it every frame)
	fn level_of(&self, d: &Device) -> i32 {
		if !d.active {
			return 4;
		}
		let mut level = match d.state.as_str() {
			"coasting" => 3,
			"dark" => 2,
			_ => 0,
		};
		if self.now - d.diag.wall < 2.0 {
			for (c, _) in &d.diag.causes {
				let sev = if *c == d.diag.headline { d.diag.severity } else { 1 };
				level = level.max(Health::cause_level(c, sev));
			}
		}
		level
	}

	/// The active devices, by role, then codename
	fn active_devices(&self) -> Vec<&Device> {
		let mut list: Vec<&Device> = self.rt.devices.values().filter(|d| d.active).collect();
		list.sort_by(|a, b| {
			role_order(self.role_of(a)).cmp(&role_order(self.role_of(b))).then_with(|| a.codename.cmp(&b.codename))
		});
		list
	}

	/// The selected device, else the first active one
	fn shown_device(&self) -> Option<&Device> {
		if let Sel::Device(c) = &self.sel {
			if let Some(d) = self.rt.devices.get(c).filter(|d| d.active) {
				return Some(d);
			}
		}
		self.active_devices().first().copied()
	}

	/// SteamVR's symbols of the active devices and of a base station for this frame's view, where its folder has
	/// them: looked up by each device's config, read once, a texture each
	fn refresh_symbols(&mut self, ctx: &egui::Context) {
		let view = self.view.index();
		self.symbols.set_steamvr(&self.st.steamvr);
		let dirs = self.config_dirs();
		let mut list: Vec<(String, String)> =
			self.rt.devices.values().filter(|d| d.active).map(|d| (d.serial.clone(), d.subtype.clone())).collect();
		list.extend(self.replay_serials());
		let mut map = std::collections::HashMap::new();
		for (serial, subtype) in list {
			if map.contains_key(&serial) {
				continue;
			}
			let kind = self.symbols.kind_of(&serial, &subtype, &dirs);
			if let Some(s) = self.symbols.get(ctx, view, &kind) {
				map.insert(serial, s);
			}
		}
		if let Some(s) = self.symbols.get(ctx, view, &crate::symbols::Kind::station()) {
			map.insert(STATION.to_string(), s);
		}
		self.sym[view] = map;
	}

	/// SteamVR's symbol of a device by its serial (STATION: of a base station), in this frame's view
	fn symbol_of(&self, serial: &str) -> Option<crate::symbols::Symbol> {
		self.sym[self.view.index()].get(serial).copied()
	}

	/// A device's symbol in `r`: SteamVR's where it has one, else the panel's icon for its type
	fn draw_symbol(&self, p: &egui::Painter, r: Rect, d: &Device, col: Color32) {
		kit::symbol(p, r, self.symbol_of(&d.serial), kit::glyph_of(&d.subtype), col);
	}

	/// A base station's symbol in `r`
	fn draw_station(&self, p: &egui::Painter, r: Rect, col: Color32) {
		kit::symbol(p, r, self.symbol_of(STATION), icons::LIGHTHOUSE, col);
	}

	fn flagged_count(&self) -> usize {
		self.rt.devices.values().filter(|d| (2..=3).contains(&self.level_of(d))).count()
	}

	fn valve_of(&self, d: &Device) -> Option<&ValveDevice> {
		self.vc.devs.get(&d.serial)
	}

	/// The dot's colour for a device: green with two stations or more, blue with one, amber dark, red coasting
	fn state_color(&self, d: &Device) -> Color32 {
		let t = tk();
		if !d.present || !d.have_pose || self.now - d.last_pose_wall > 1.0 {
			return t.faint;
		}
		match d.state.as_str() {
			"tracking" => {
				if d.lh_mask.count_ones() >= 2 {
					t.good
				} else {
					t.note
				}
			}
			"dark" | "reacquiring" => t.warn,
			"coasting" => t.bad,
			_ => t.faint,
		}
	}

	fn set_floor(&mut self) {
		let now = self.now;
		let lo = self
			.rt
			.devices
			.values()
			.filter(|d| d.active && d.state == "tracking" && now - d.last_pose_wall < 0.5)
			.map(|d| d.pos.z)
			.fold(f64::NAN, f64::min);
		if !lo.is_nan() {
			self.st.floor_z = lo - 0.03;
			self.st.dirty = true;
			let t = format!("floor set at {:.3} m (lowest device - 3 cm)", self.st.floor_z);
			self.rt.log(EvLevel::Info, "panel", &t, -1.0, true);
		}
	}

	/// One frame of the panel into `ui` (the root Ui of the view's context), at the host's clock `now`
	pub fn frame(&mut self, ui: &mut egui::Ui, now: f64, view: View) {
		self.now = now;
		self.view = view;
		self.frame_no[view.index()] += 1;
		let ctx = ui.ctx().clone();
		let screen = ui.max_rect();

		// what was typed on SteamVR's keyboard: a device's name
		if self.vc.kb_done {
			self.vc.kb_done = false;
			let (tag, text) = (self.vc.kb_tag.clone(), self.vc.kb_text.clone());
			self.set_name(&tag, &text);
		}
		self.host_overrides();

		// the automatic floor: the lowest point a tracking device has reached, less a tracker's height over it
		for d in self.rt.devices.values() {
			if d.have_pose && d.state == "tracking" && now - d.last_pose_wall <= 0.5 {
				let z = d.pos.z - 0.03;
				if self.auto_floor.is_nan() || z < self.auto_floor {
					self.auto_floor = z;
				}
			}
		}

		// the look: the tokens for this frame
		let t = theme::Tk::default();
		set_tk(t);
		if self.style_set[view.index()] != Some(t) {
			theme::apply(&ctx, &t);
			self.style_set[view.index()] = Some(t);
		}
		self.refresh_symbols(&ctx);
		let painter = ui.painter().clone();
		painter.rect_filled(screen, 0.0, t.bg);

		// the selection follows the active devices: one that stopped posing is neither shown nor selected
		match &self.sel {
			Sel::Device(c) if !self.rt.devices.get(c).is_some_and(|d| d.active) => self.sel = Sel::None,
			Sel::Station(i) if !self.rt.stations.contains_key(i) => self.sel = Sel::None,
			_ => {}
		}
		if self.page.as_ref().is_some_and(|c| !self.rt.devices.get(c).is_some_and(|d| d.active)) {
			self.page = None;
		}
		if view == View::Window && self.page.is_some() && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
			self.page = None;
		}

		let m = kit::GAP;
		let mut top_h = topbar::BAR_H;
		self.top_bar(ui, Rect::from_min_size(screen.min, Vec2::new(screen.width(), top_h)));
		// what sally is doing, while its tracking is not what SteamVR is shown
		let strip = self.state_strip(ui, screen.min + Vec2::new(m, top_h + m), screen.width() - 2.0 * m);
		if strip > 0.0 {
			top_h += m + strip;
		}
		let body = Rect::from_min_max(screen.min + Vec2::new(m, top_h + m), screen.max - Vec2::splat(m));
		match self.st.tab {
			1 => self.replay_tab(ui, body),
			2 => self.output_tab(ui, body),
			3 => self.settings_tab(ui, body),
			4 => self.check_tab(ui, body),
			_ => {
				if self.page.is_some() {
					self.device_page(ui, body);
				} else {
					self.tracking_tab(ui, body);
				}
			}
		}
		self.confirmations(ui);
	}

	fn host_overrides(&mut self) {
		if !self.select_device.is_empty() {
			let want = std::mem::take(&mut self.select_device);
			if let Some(d) = self.rt.devices.values().find(|d| d.codename == want || d.serial == want) {
				self.sel = Sel::Device(d.codename.clone());
			} else {
				self.select_device = want;
			}
		}
		if !self.select_station.is_empty() {
			let want = std::mem::take(&mut self.select_station);
			if let Some(s) = self.rt.stations.values().find(|s| s.name() == want || s.idx.to_string() == want) {
				self.sel = Sel::Station(s.idx);
			} else {
				self.select_station = want;
			}
		}
		if !self.open_page.is_empty() {
			// "KN1", or "KN1:sensors" for its Sensors tab
			let want = std::mem::take(&mut self.open_page);
			let (dev, tab) = want.split_once(':').unwrap_or((want.as_str(), ""));
			if let Some(d) = self.rt.devices.values().find(|d| d.active && (d.codename == dev || d.serial == dev)) {
				self.page = Some(d.codename.clone());
				self.sel = Sel::Device(d.codename.clone());
				self.page_state.sensors_tab = tab == "sensors";
				self.st.tab = 0;
			} else {
				self.open_page = want;
			}
		}
	}

	/// Opens a device's page
	fn open_device_page(&mut self, codename: &str) {
		self.page = Some(codename.into());
		self.sel = Sel::Device(codename.into());
		self.page_state.hover = [None, None];
	}

	/// The dashboard button's picture: sally's icon, its dot saying what sally is doing as on the taskbar (icon.rs)
	pub fn thumbnail(&mut self, ui: &mut egui::Ui, now: f64) {
		self.now = now;
		let dot = self.dot(now);
		if self.button_picture.as_ref().is_none_or(|(drawn, _)| *drawn != dot) {
			let side = ui.max_rect().width().round() as usize;
			let Some(p) = crate::icon::picture(side, dot) else { return };
			let image = egui::ColorImage::from_rgba_unmultiplied([p.side, p.side], &p.rgba);
			let texture = ui.ctx().load_texture("sally-icon", image, egui::TextureOptions::LINEAR);
			self.button_picture = Some((dot, texture));
		}
		if let Some((_, texture)) = &self.button_picture {
			let whole = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
			ui.painter().image(texture.id(), ui.max_rect(), whole, Color32::WHITE);
		}
	}

	/// The questions asked before something that stops the tracking for a moment
	fn confirmations(&mut self, ui: &mut egui::Ui) {
		let ask = |ui: &mut egui::Ui, lines: &[String], yes: &str| -> Option<bool> {
			let mut answer = None;
			ui.set_max_width(520.0);
			for l in lines.iter().filter(|l| !l.is_empty()) {
				ui.label(egui::RichText::new(l).font(regular(15.0)).color(tk().soft));
			}
			ui.add_space(10.0);
			ui.horizontal(|ui| {
				if ui.add_sized([150.0, 38.0], egui::Button::new(yes)).clicked() {
					answer = Some(true);
				}
				if ui.add_sized([150.0, 38.0], egui::Button::new("Cancel")).clicked() {
					answer = Some(false);
				}
			});
			answer
		};
		if self.confirm_apply {
			let why = self.rt.calib.ready.as_ref().map(|(rec, why)| format!("Of {rec}: {why}.")).unwrap_or_default();
			let lines = [
				"The runtime stops, and the sally driver starts it again on the new calibration. Valve's tracking \
				 covers the gap, and sally lines up with it again as before."
					.to_string(),
				why,
			];
			let mut answer = None;
			let modal = egui::Modal::new(Id::unique("apply-modal")).show(ui.ctx(), |ui| {
				answer = ask(ui, &lines, "Use it");
			});
			if answer == Some(true) {
				self.rt.command("calibrate apply", Want::CalibrateApply);
			}
			if answer.is_some() || modal.should_close() {
				self.confirm_apply = false;
			}
		}
		if self.confirm_restart {
			let lines = ["The runtime quits and closes its recording. Under SteamVR the sally driver starts it again \
			              and Valve's tracking covers the gap; started by hand, it stays stopped."
				.to_string()];
			let mut answer = None;
			let modal = egui::Modal::new(Id::unique("restart-modal")).show(ui.ctx(), |ui| {
				answer = ask(ui, &lines, "Restart");
			});
			if answer == Some(true) {
				self.rt.command("quit", Want::Nothing);
				self.rt.log(EvLevel::Warn, "panel", "asked the runtime to quit", -1.0, true);
			}
			if answer.is_some() || modal.should_close() {
				self.confirm_restart = false;
			}
		}
		if self.confirm_unlock {
			let lines =
				["Unlocked, the map is re-solved during the session and written to the config file.".to_string()];
			let mut answer = None;
			let modal = egui::Modal::new(Id::unique("unlock-modal")).show(ui.ctx(), |ui| {
				answer = ask(ui, &lines, "Unlock");
			});
			if answer == Some(true) {
				self.rt.command("map unlock", Want::Nothing);
			}
			if answer.is_some() || modal.should_close() {
				self.confirm_unlock = false;
			}
		}
	}
}

/// Where the room's floor is from
#[derive(Clone, Copy, Debug, PartialEq)]
enum FloorFrom {
	/// set by hand (Set it here)
	Set,
	/// SteamVR's own: where its walls stand
	SteamVr,
	/// the lowest a tracking device has been
	Devices,
}

/// The room's floor (m, in sally's world) and where it is from: the one set by hand (`set`, NaN for none); else
/// SteamVR's own, where the panel has its walls (`steamvr`); else 3 cm under the lowest a tracking device has been
/// since the panel started (`lowest`, NaN before one has: 0 then). SteamVR's comes before the devices' because one
/// pose of one device under the floor, at any moment, takes theirs down for good
fn floor_of(set: f64, steamvr: Option<f64>, lowest: f64) -> (f64, FloorFrom) {
	if !set.is_nan() {
		(set, FloorFrom::Set)
	} else if let Some(z) = steamvr {
		(z, FloorFrom::SteamVr)
	} else if lowest.is_nan() {
		(0.0, FloorFrom::Devices)
	} else {
		(lowest, FloorFrom::Devices)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_rooms_floor_is_the_set_one_then_steamvrs_then_the_lowest_a_device_went() {
		let nan = f64::NAN;
		assert_eq!(floor_of(0.4, Some(1.3), -0.02), (0.4, FloorFrom::Set));
		// SteamVR's stands whatever a device's pose once said
		assert_eq!(floor_of(nan, Some(1.3), -0.02), (1.3, FloorFrom::SteamVr));
		assert_eq!(floor_of(nan, None, -0.02), (-0.02, FloorFrom::Devices));
		assert_eq!(floor_of(nan, None, nan), (0.0, FloorFrom::Devices));
	}

	#[test]
	fn names_are_cleaned_and_cut_between_characters() {
		assert_eq!(clean_name("  Left\tfoot  "), "Leftfoot");
		let long = "é".repeat(20); // 40 bytes
		let c = clean_name(&long);
		assert_eq!(c.len(), 32);
		assert!(c.chars().all(|ch| ch == 'é'));
	}

	#[test]
	fn a_models_name_loses_its_builds_letters() {
		assert_eq!(model_words("VIVE Tracker 3.0 MV"), "VIVE Tracker 3.0");
		assert_eq!(model_words("Tundra Tracker"), "Tundra Tracker");
		assert_eq!(model_words(" Utah MP "), "Utah");
	}

	#[test]
	fn a_serials_tail_is_cut_between_characters() {
		assert_eq!(serial_tail("LHR-0A1B2C3D"), "2C3D");
		assert_eq!(serial_tail("2C3D"), "2C3D");
		assert_eq!(serial_tail("-"), "-");
		// a character of two bytes, and the one a byte that is not UTF-8 becomes (three)
		assert_eq!(serial_tail("a\u{e9}123"), "\u{e9}123");
		assert_eq!(serial_tail("\u{fffd}AB"), "\u{fffd}AB");
		assert_eq!(serial_tail("xx\u{fffd}ABC"), "\u{fffd}ABC");
	}
}
