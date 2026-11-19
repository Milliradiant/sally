//! The panel against the fake runtime, without a display: egui contexts run frames (nothing is drawn), the panel's
//! state checked between them: the tabs, the device page and its Sensors view, the bar (what SteamVR is shown),
//! the Output tab against a stand-in for the sally driver, the theme and the floor's pop-up. The clicks are where a
//! 1600 x 940 window has the controls.

use std::time::{Duration, Instant};

use egui::{Event, Key, Modifiers, PointerButton, Pos2, Rect, Vec2};
use sally_panel::fake_runtime::{FakeRuntime, COVERED};
use sally_panel::runtime::EvLevel;
use sally_panel::sensor_view::{self, SensorState};
use sally_panel::settings::Settings;
use sally_panel::ui::{Panel, Sel, View};
use sally_panel::valve::fake_driver::FakeDriver;
use sally_panel::valve::ValveCompare;

/// A panel on a fake runtime, and a window's egui context at 1600 x 940 points
struct Rig {
	_fake: FakeRuntime,
	/// a stand-in for the sally driver, where the test has one
	driver: Option<FakeDriver>,
	panel: Panel,
	ctx: egui::Context,
	t0: Instant,
}

impl Rig {
	fn new(drop: Vec<(String, f64)>, st: Settings) -> Rig {
		Rig::build(drop, st, false)
	}

	/// With the sally driver's side answering (SteamVR shown sally's tracking, the output layer's settings)
	fn with_driver() -> Rig {
		Rig::build(Vec::new(), Settings::default(), true)
	}

	fn build(drop: Vec<(String, f64)>, st: Settings, driver: bool) -> Rig {
		let fake = FakeRuntime::start(0, drop).expect("fake runtime");
		let mut st = st;
		st.host = "127.0.0.1".into();
		st.port = fake.port;
		// a runtime of the panel's own, as --host and --port give it one: no strip about this PC's SteamVR over the
		// layout the clicks are placed in
		st.file_target = Some((st.host.clone(), st.port));
		let mut vc = ValveCompare::new();
		let driver = driver.then(|| FakeDriver::new(&mut vc).expect("fake driver"));
		let panel = Panel::new(st, vc);
		let ctx = egui::Context::default();
		Panel::setup(&ctx);
		Rig { _fake: fake, driver, panel, ctx, t0: Instant::now() }
	}

	fn now(&self) -> f64 {
		self.t0.elapsed().as_secs_f64()
	}

	/// One loop: the socket, then a frame with these events
	fn frame(&mut self, events: Vec<Event>) {
		let now = self.now();
		if let Some(d) = &mut self.driver {
			d.poll(now, &self.panel.rt);
		}
		self.panel.poll(now);
		let raw = egui::RawInput {
			screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 940.0))),
			time: Some(now),
			events,
			focused: true,
			..Default::default()
		};
		let panel = &mut self.panel;
		// nothing is drawn: the textures' changes go nowhere
		self.ctx.run_ui(raw, |ui| panel.frame(ui, now, View::Window)).drop_without_applying_deltas();
	}

	/// Frames at about 60 a second until `seconds` into the run
	fn run_until(&mut self, seconds: f64) {
		while self.now() < seconds {
			self.frame(Vec::new());
			std::thread::sleep(Duration::from_millis(16));
		}
	}

	/// A click: the press, and the release a frame later
	fn click(&mut self, x: f32, y: f32) {
		let p = Pos2::new(x, y);
		self.frame(vec![Event::PointerMoved(p)]);
		self.frame(vec![Event::PointerButton {
			pos: p,
			button: PointerButton::Primary,
			pressed: true,
			modifiers: Modifiers::NONE,
		}]);
		self.frame(vec![Event::PointerButton {
			pos: p,
			button: PointerButton::Primary,
			pressed: false,
			modifiers: Modifiers::NONE,
		}]);
		self.frame(Vec::new());
	}

	fn key(&mut self, key: Key) {
		let ev = |pressed| Event::Key { key, physical_key: None, pressed, repeat: false, modifiers: Modifiers::NONE };
		self.frame(vec![ev(true), ev(false)]);
		self.frame(Vec::new());
	}

	fn events_about(&self, serial: &str) -> Vec<String> {
		self.panel.rt.events.iter().filter(|e| e.who == serial).map(|e| e.text.clone()).collect()
	}
}

#[test]
fn the_panel_shows_the_active_devices_and_not_the_switched_off_one() {
	let mut r = Rig::new(Vec::new(), Settings::default());
	r.run_until(4.0);
	let rt = &r.panel.rt;
	assert_eq!(rt.protocol, 4);
	assert_eq!(rt.stations.len(), 4);
	assert_eq!(rt.devices.len(), 7);
	assert_eq!(rt.active().count(), 6);
	assert!(rt.devices["KN6"].present && !rt.devices["KN6"].active);
	// the paired controller that is switched off makes no events, its diagnosis included
	assert!(
		r.events_about("LHR-7E87BF59").iter().all(|t| t.starts_with("device ")),
		"{:?}",
		r.events_about("LHR-7E87BF59")
	);
	assert!(rt.events.iter().any(|e| e.text.starts_with("connected to 127.0.0.1:")));
	// the fake's sensor layouts came in, and the H lines' light
	assert_eq!(rt.devices["KN0"].sensor_pos.len(), 22);
	assert!(rt.devices["KN0"].lit_sensors(r.now()) > 0);
	// the waist's weak radio is an event once it has held 2 s
	assert!(r.events_about("LHR-2D779713").iter().any(|t| t == "6% of reports lost"));
	// the first device until one is picked
	assert_eq!(r.panel.selection(), &Sel::None);
}

#[test]
fn a_device_that_goes_quiet_is_an_event_and_leaves_the_list() {
	let mut r = Rig::new(vec![("KN2".into(), 2.0)], Settings::default());
	r.run_until(6.5);
	let d = &r.panel.rt.devices["KN2"];
	assert!(!d.active);
	assert!(r.events_about("LHR-9365E531").iter().any(|t| t == "no poses for 3 s"));
	assert_eq!(r.panel.rt.active().count(), 5);
}

#[test]
fn the_device_page_opens_and_escape_closes_it() {
	let mut r = Rig::new(Vec::new(), Settings::default());
	r.run_until(2.0);
	r.panel.open_page = "LHR-0334A73B".into();
	r.frame(Vec::new());
	assert_eq!(r.panel.page(), Some("KN0"));
	r.run_until(4.0);
	// two seconds of history at up to 120 samples a second (the fake sends 125)
	let n = r.panel.rt.devices["KN0"].history.samples.len();
	assert!((150..=500).contains(&n), "{n}");
	r.key(Key::Escape);
	assert_eq!(r.panel.page(), None);
}

#[test]
fn the_sensors_tab_shows_a_covered_sensor_and_light_from_behind() {
	let mut r = Rig::new(Vec::new(), Settings::default());
	r.run_until(2.0);
	r.panel.open_page = "KN0:sensors".into();
	r.frame(Vec::new());
	assert_eq!(r.panel.page(), Some("KN0"));
	// the sensor layout, and three of the fake's H periods (one a second) with the poses at their ends
	r.run_until(5.5);
	let rt = &r.panel.rt;
	let stations = sensor_view::stations_of(rt);
	assert_eq!(stations.len(), 4);
	let v = sensor_view::judge(&rt.devices["KN0"], &stations, None);
	assert!(v.judged && v.lights.iter().all(|&l| l), "{:?}", v.lights);
	assert_eq!(v.states[COVERED], SensorState::Missing, "{:?}", v.states);
	assert_eq!(v.count(SensorState::Missing), 1);
	assert_eq!(v.count(SensorState::Behind), 0);
	assert!(v.cells[COVERED][0].expected && v.cells[COVERED][0].angle < 30.0);
	// the tracker whose light comes from behind
	let v = sensor_view::judge(&rt.devices["KN4"], &stations, None);
	assert!(v.count(SensorState::Behind) > 0, "{:?}", v.states);
	assert_eq!(v.count(SensorState::Missing), 0);
	// the page's view back to the curves: a click on "Curves", left of "Sensors" at the right end of the page's head
	assert!(r.panel.page_sensors_tab());
	r.click(1467.0, 94.0);
	assert!(!r.panel.page_sensors_tab());
	// and the list beside the page picks its device: the second row is the right hand's
	r.click(200.0, 198.0);
	assert_eq!(r.panel.page(), Some("KN1"));
	assert_eq!(r.panel.selection(), &Sel::Device("KN1".into()));
}

/// Where the Selected box puts the device's name in a 1600 x 940 window: the right column (414 points wide) from
/// x = 1176, the name 58 points in and 12 down
const NAME: (f32, f32) = (1250.0, 88.0);

#[test]
fn renaming_in_the_window() {
	let dir = std::env::temp_dir().join(format!("sally-panel-rename-{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let st = Settings::load_dir(&dir);
	let mut r = Rig::new(Vec::new(), st);
	r.run_until(2.0);
	r.panel.select_device = "KN4".into();
	r.frame(Vec::new());
	assert_eq!(r.panel.device_name("KN4"), Some(tracker_name(&r, "KN4")));
	r.click(NAME.0, NAME.1);
	r.frame(vec![Event::Text("Left foot spare".into())]);
	r.key(Key::Enter);
	assert_eq!(r.panel.device_name("KN4").as_deref(), Some("Left foot spare"));
	assert_eq!(r.panel.st.names["LHR-B0D67561"], "Left foot spare");
	// Escape keeps the name it had
	r.click(NAME.0, NAME.1);
	r.frame(vec![Event::Text("something else".into())]);
	r.key(Key::Escape);
	assert_eq!(r.panel.device_name("KN4").as_deref(), Some("Left foot spare"));
	// an empty name is the default name again
	r.click(NAME.0, NAME.1);
	r.key(Key::Backspace);
	r.key(Key::Enter);
	assert_eq!(r.panel.device_name("KN4"), Some(tracker_name(&r, "KN4")));
	assert!(!r.panel.st.names.contains_key("LHR-B0D67561"));
	// and the names are written to the file
	r.panel.st.names.insert("LHR-0334A73B".into(), "Head".into());
	r.panel.st.dirty = true;
	r.panel.save_settings();
	let back = Settings::load_dir(&dir);
	assert_eq!(back.names["LHR-0334A73B"], "Head");
	std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn an_old_files_tab_opens_where_its_things_are_now() {
	// there is no Simple tab any more, and Advanced is in Settings: a sally-panel.cfg from before with either opens on
	// Settings, and is written with the tab's name
	let dir = std::env::temp_dir().join(format!("sally_panel_simple_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let path = dir.join("sally-panel.cfg");
	std::fs::write(&path, "tab 3\nsimple 1\n").unwrap();
	let st = Settings::load(&path);
	assert!(!st.text().contains("simple"));
	assert!(st.text().contains("tab settings"));
	let mut r = Rig::new(Vec::new(), st);
	r.frame(Vec::new());
	assert_eq!(r.panel.st.tab, 3);
	std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn the_floor_is_set_from_the_lowest_device() {
	let mut r = Rig::new(Vec::new(), Settings::default());
	r.run_until(2.0);
	assert!(r.panel.st.floor_z.is_nan());
	// the room's Floor chip (its top left), then Set it here in what opens under it: the feet are at 0.06 to 0.10 m,
	// the floor 3 cm under the lowest
	let before = r.panel.rt.events.len();
	r.click(470.0, 125.0);
	r.click(504.0, 253.0);
	let set = r.panel.rt.events.iter().skip(before).find(|e| e.text.starts_with("floor set at"));
	assert!(
		set.is_some_and(|e| e.level == EvLevel::Info),
		"{:?}",
		r.panel.rt.events.iter().skip(before).collect::<Vec<_>>()
	);
	assert!((0.02..0.08).contains(&r.panel.st.floor_z), "{}", r.panel.st.floor_z);
	// Auto, beside it, finds it by itself again
	r.click(644.0, 253.0);
	assert!(r.panel.st.floor_z.is_nan());
}

#[test]
fn the_bar_says_what_steamvr_is_shown() {
	let mut r = Rig::with_driver();
	r.run_until(2.5);
	// the driver's word is in: SteamVR is shown sally's tracking, and a click on Valve asks it for Valve's
	assert!(r.panel.vc.hook_mode && r.panel.vc.hook_seen == "sally");
	r.click(1553.0, 27.0);
	r.run_until(4.0);
	let d = r.driver.as_ref().unwrap();
	assert!(d.commands.iter().any(|c| c == "seen valve"), "{:?}", d.commands);
	assert_eq!(r.panel.vc.hook_seen, "valve");
}

#[test]
fn the_output_tab_sets_the_drivers_values() {
	let mut r = Rig::with_driver();
	r.panel.st.tab = 2;
	r.run_until(3.0);
	let value =
		|r: &Rig, scope: &str, name: &str| r.panel.vc.output.get(&(scope.to_string(), name.to_string())).copied();
	// the driver's list is in: the controllers' smoothing at its default
	assert_eq!(value(&r, "controllers", "smoothing_ms"), Some(0.0));
	// the stepper at the right end of the controllers' Still smoothing: 5 ms more, sent to the driver
	r.click(1031.0, 192.0);
	r.run_until(3.6);
	let d = r.driver.as_ref().unwrap();
	assert!(d.commands.iter().any(|c| c == "output controllers smoothing_ms 5"), "{:?}", d.commands);
	assert_eq!(d.output[&("controllers".to_string(), "smoothing_ms".to_string())], 5.0);
	assert_eq!(value(&r, "controllers", "smoothing_ms"), Some(5.0));
	// the row under it is the other smoothing, the movement's: its own setting, and the still one's stays
	r.click(1031.0, 250.0);
	r.run_until(4.0);
	assert_eq!(value(&r, "controllers", "movement_smoothing"), Some(5.0));
	assert_eq!(value(&r, "controllers", "smoothing_ms"), Some(5.0));
	// a device's own values: its chip (the left hand's, after All controllers), the same stepper, and the driver has
	// the value under its serial; the class's stays
	r.click(706.0, 123.0);
	r.click(1031.0, 192.0);
	r.run_until(4.8);
	assert_eq!(value(&r, "LHR-9365E531", "smoothing_ms"), Some(10.0));
	assert_eq!(value(&r, "controllers", "smoothing_ms"), Some(5.0));
}

/// The panel's copy of the driver's output settings (the fake driver's list, which the Output tab's ticks and its
/// Defaults button and the punch settings' numbers are held to in their own tests) is the driver's own: every name, default
/// and range. So Defaults in the panel is the driver's default, the tracker's own output
#[test]
fn the_panels_output_settings_are_the_drivers() {
	use driver_sally::output::SETTINGS as DRIVERS;
	use sally_panel::valve::fake_driver::SETTINGS as PANELS;
	assert_eq!(PANELS.len(), DRIVERS.len());
	for (p, d) in PANELS.iter().zip(&DRIVERS) {
		assert_eq!((p.0, p.1, p.2, p.3), (d.name, d.default, d.min, d.max), "{}", d.name);
	}
}

/// The controllers' punch settings, under their five on the Output tab: the dash's switch is the rail's too (on with
/// it, off with it), the rail's off takes the dash with it, and the rail's own switch is the hands' rail alone
#[test]
fn the_output_tab_switches_the_punch_rail_and_the_dash() {
	let mut r = Rig::with_driver();
	r.panel.st.tab = 2;
	r.run_until(3.0);
	let value =
		|r: &Rig, scope: &str, name: &str| r.panel.vc.output.get(&(scope.to_string(), name.to_string())).copied();
	let both = |r: &Rig| (value(r, "controllers", "rail"), value(r, "controllers", "dash"));
	assert_eq!(both(&r), (Some(0.0), Some(0.0)));
	// dash punch's switch: on, and the rail with it; for the hands alone
	r.click(1025.0, 664.0);
	r.run_until(3.6);
	assert_eq!(both(&r), (Some(1.0), Some(1.0)));
	assert_eq!((value(&r, "trackers", "rail"), value(&r, "headset", "dash")), (Some(0.0), Some(0.0)));
	// the stepper at the right end of "Taken from": a tenth of a metre a second more
	r.click(1031.0, 562.0);
	r.run_until(4.2);
	assert_eq!(value(&r, "controllers", "rail_speed"), Some(1.6));
	// dash punch's switch again: off, and the rail with it
	r.click(1025.0, 664.0);
	r.run_until(4.8);
	assert_eq!(both(&r), (Some(0.0), Some(0.0)));
	// the rail's own switch: the rail alone
	r.click(1025.0, 498.0);
	r.run_until(5.4);
	assert_eq!(both(&r), (Some(1.0), Some(0.0)));
	// the dash on it, then the rail off: the dash goes with it
	r.click(1025.0, 664.0);
	r.run_until(6.0);
	assert_eq!(both(&r), (Some(1.0), Some(1.0)));
	r.click(1025.0, 498.0);
	r.run_until(6.6);
	assert_eq!(both(&r), (Some(0.0), Some(0.0)));
}

/// The strip under the bar says what sally is doing while SteamVR is not shown its tracking: here, with no SteamVR,
/// that it is not running. It takes its place over the tab's content, and goes for a panel given its own runtime
#[test]
fn the_state_strip_says_steamvr_is_not_running() {
	let mut r = Rig::new(Vec::new(), Settings::default());
	r.run_until(1.0);
	let not_running = sally_panel::ui::state::state(&sally_panel::ui::state::Facts::default());
	assert!(not_running.is_some_and(|(_, text)| text.starts_with("SteamVR is not running")));
	// each tab drawn with the strip over it (Settings with the driver's choices not yet heard), then without
	let own_runtime = r.panel.st.file_target.take();
	for tab in [0, 3] {
		r.panel.st.tab = tab;
		r.frame(Vec::new());
	}
	r.panel.st.file_target = own_runtime;
	r.frame(Vec::new());
	// the icon's dot: for a panel with a runtime of its own, whether that answers; else this PC's state, here no SteamVR
	let now = r.now();
	assert_eq!(r.panel.dot(now), sally_panel::icon::Dot::Tracking);
	r.panel.st.file_target = None;
	assert_eq!(r.panel.dot(now), sally_panel::icon::Dot::Idle);
}

/// The name a tracker has until it is given one: its kind and its serial's last four
fn tracker_name(r: &Rig, codename: &str) -> String {
	let serial = &r.panel.rt.devices[codename].serial;
	format!("Tracker {}", &serial[serial.len() - 4..])
}
