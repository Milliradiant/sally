//! The output layer (design doc 9.4; docs/sally_driver.md, "The output layer"): how SteamVR is shown sally's tracking,
//! chosen per device class with per-device overrides, from the panel. Preference, not tracking: the runtime's estimate
//! is untouched, and the panel and the recordings show it as it is.
//!
//! The settings are numbers, each per class (`headset`, `controllers`, `trackers`) and per device (by serial, over
//! its class):
//!
//! - `recovery_blend_ms`: when the light comes back after a dark spell, the jump from where the device was carried in
//!   the dark to where the light puts it, shown as a glide that settles in this long (ms) instead of in one frame. 0 is
//!   the jump as it comes. Only the corrections of the light's return are blended ([`Blend::show`]).
//! - `smoothing_ms`: a device that is held still is shown calm ([`Smooth`]): what it moves by while still (a tremble,
//!   a pulse, the sensors' noise: up to a millimetre and 3 mrad of it) and the light's corrections are let in as a
//!   glide that settles in this long (ms), and SteamVR is given no velocity to carry them on with. A device that
//!   moves, fast or slowly, is shown as the tracker has it:
//!   its motion passes whole and at once, and only the light's corrections (a station taking over, a restart) still
//!   glide in, sooner the faster it moves or turns. 0 is the tracker's own pose.
//! - `prediction_ms`: how old the pose is declared when it is handed over (ms), which is how far ahead of the tracker's
//!   state SteamVR is shown the device. Over the radio the measured age is the default (9 ms); the headset's is 0
//!   (measured 4 ms on its cable). More shows the device further ahead, less further behind.
//! - `velocity_boost`: the velocity SteamVR is given, this many percent over the device's own, for games that score
//!   on how fast a hand moves. SteamVR's own prediction reaches that much further too. 0 is the device's own.
//! - `movement_smoothing`: SteamVR's own prediction cut by this many percent, which is what a smoothing add-on does:
//!   SteamVR shows a device where it will be by the time the frame is seen, on a straight line along the velocity
//!   and the body rate it is given, and that line carries whatever shakes in them several times over. The velocity
//!   and the rate are handed over this much smaller, so the device is shown calmer whether it moves or not, and
//!   trails what it does by as much of SteamVR's look-ahead. 0 is the device's own; at 100 SteamVR is shown the pose
//!   as it is now and nothing ahead of it.
//! - `offset_x_mm`, `offset_y_mm`, `offset_z_mm`, `offset_rx_deg`, `offset_ry_deg`, `offset_rz_deg`: where the device
//!   is shown against where it is, in its own axes (the ones SteamVR draws it in): moved, then turned about its x, its
//!   y and its z axis. 0 is where it is.
//! - `rail`, `rail_speed`, `rail_boost`: the punch rail ([`Rail`]), a cheat for games that score a punch by the hand's
//!   speed. With `rail` 1, a hand that gets as fast as `rail_speed` (m/s) is shown on a straight line from there, along
//!   the way it is going (what it moves sideways is left out), and `rail_boost` percent further and faster along it
//!   than it moves; as it slows it is let off the line and eased back to where it is. `rail` 0 is no rail.
//! - `dash`, `dash_percent`, `dash_back_ms` (a hand's, with its rail): dash punch (toys.rs, body.rs), a switch and
//!   its numbers: a punch on its rail takes the whole body with it, the headset too, and the body is eased back.
//!
//! They live in sally_output.txt next to the driver (`<scope>.<name>=<value>` lines, the scope a class or a serial),
//! which the driver reads when SteamVR starts it and writes whenever one changes. The panel sets them live on the
//! command port: `output` lists them as `O <scope> <name> <value>` lines; `output <scope> <name> <value>` sets one
//! (`-` for a device takes its own value away), and the list comes back.
//!
//! A toy is for the session it is switched on in: the toys' switches ([`TOY_SWITCHES`]) are put off as SteamVR
//! closes, and again as it starts in case it did not close in order, so a toy is never on in a session it was not
//! switched on in ([`toys_off`]). Their numbers, and every other setting, are kept as they were set.

use std::collections::BTreeMap;
use std::sync::{LazyLock, Mutex, MutexGuard};

use sally_math::c_atof;
use sally_math::math3d::{dot, len, qmul, Pose, Q4, V3};
use sally_math::printf::{bsprintf, BArg};

use crate::body::Dash;
use crate::cstdio::CFile;
use crate::helpers::{log, now_seconds, quat_conj, quat_mul, sin_cos, trim, write_whole, K_SEP};
use crate::runtime_device::Kind;
use crate::toys::Kin;

/// One setting: its name, its default per class (CLASSES' order) and its range
pub struct Setting {
	pub name: &'static str,
	pub default: [f64; CLASSES.len()],
	pub min: f64,
	pub max: f64,
}

/// The classes, as the settings' scopes name them
pub const CLASSES: [&str; 3] = ["headset", "controllers", "trackers"];

/// The settings, in the order the panel is sent them, their defaults per class (headset, controllers, trackers). The
/// defaults are the tracker's own output, the one sally's tracking is tuned on: no glide and nothing added. The
/// recovery blend, the smoothing, the velocity boost and the offsets are 0 and every toy is off, so a setting changes
/// what SteamVR is shown only once it has been set, from the panel (and is kept in sally_output.txt from then on). The
/// prediction is not something added: it is the pose's own age, 9 ms over the radio (measured; the tracking is tuned
/// and benched at it), 0 on the headset (measured 4 ms on its cable). What the bench found best where a glide is wanted
/// (150 ms, 100 ms for a controller's return) is in sally_driver.md, "The output layer", as values to set, not as
/// defaults. `the_defaults_are_the_trackers_own_output` holds this
pub const SETTINGS: [Setting; 17] = [
	Setting { name: "recovery_blend_ms", default: [0.0; 3], min: 0.0, max: 1000.0 },
	Setting { name: "smoothing_ms", default: [0.0; 3], min: 0.0, max: 1000.0 },
	Setting { name: "prediction_ms", default: [0.0, 9.0, 9.0], min: 0.0, max: 40.0 },
	Setting { name: "velocity_boost", default: [0.0; 3], min: 0.0, max: 200.0 },
	Setting { name: "offset_x_mm", default: [0.0; 3], min: -500.0, max: 500.0 },
	Setting { name: "offset_y_mm", default: [0.0; 3], min: -500.0, max: 500.0 },
	Setting { name: "offset_z_mm", default: [0.0; 3], min: -500.0, max: 500.0 },
	Setting { name: "offset_rx_deg", default: [0.0; 3], min: -180.0, max: 180.0 },
	Setting { name: "offset_ry_deg", default: [0.0; 3], min: -180.0, max: 180.0 },
	Setting { name: "offset_rz_deg", default: [0.0; 3], min: -180.0, max: 180.0 },
	Setting { name: "rail", default: [0.0; 3], min: 0.0, max: 1.0 },
	Setting { name: "rail_speed", default: [1.5; 3], min: 0.3, max: 10.0 },
	Setting { name: "rail_boost", default: [50.0; 3], min: 0.0, max: 500.0 },
	Setting { name: "dash", default: [0.0; 3], min: 0.0, max: 1.0 },
	Setting { name: "dash_percent", default: [100.0; 3], min: 0.0, max: 100.0 },
	Setting { name: "dash_back_ms", default: [400.0; 3], min: 50.0, max: 2000.0 },
	Setting { name: "movement_smoothing", default: [0.0; 3], min: 0.0, max: 100.0 },
];
/// SETTINGS' indices
pub const RECOVERY_BLEND_MS: usize = 0;
pub const SMOOTHING_MS: usize = 1;
pub const PREDICTION_MS: usize = 2;
pub const VELOCITY_BOOST: usize = 3;
pub const OFFSET_MM: usize = 4;
pub const OFFSET_DEG: usize = 7;
pub const RAIL: usize = 10;
pub const RAIL_SPEED: usize = 11;
pub const RAIL_BOOST: usize = 12;
pub const DASH: usize = 13;
pub const DASH_PERCENT: usize = 14;
pub const DASH_BACK_MS: usize = 15;
pub const MOVEMENT_SMOOTHING: usize = 16;
/// The toys' switches: each 0 or 1, off by default, and off again when SteamVR closes (`toys_off`)
pub const TOY_SWITCHES: [usize; 2] = [RAIL, DASH];

/// A device's value of every setting
pub type Values = [f64; SETTINGS.len()];

/// A device's class
pub fn class_of(kind: Kind) -> usize {
	match kind {
		Kind::Headset => 0,
		Kind::LeftController | Kind::RightController => 1,
		Kind::Tracker => 2,
	}
}

/// What the override does to a device's pose as it hands it to SteamVR (pose_override.rs, `substitute`): what the
/// toys show in its place (toys.rs: the report's kinematics reshaped), where it is shown against where it is (its
/// own axes), the velocity's scale, and how much of its own prediction SteamVR is left. The fit to Valve's frame and
/// the fallback's checks go by the pose before it
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shown {
	pub velocity_scale: f64,
	/// the movement smoothing: the share of the velocity and the body rate SteamVR is handed to predict on (1: all)
	pub prediction: f64,
	pub offset: Option<Pose>,
	pub reshaped: Option<Kin>,
	/// dash punch: what the device's punch rail gives the whole body to slide by (body.rs; the runtime's world)
	pub dash: Option<Dash>,
}

impl Default for Shown {
	fn default() -> Shown {
		Shown { velocity_scale: 1.0, prediction: 1.0, offset: None, reshaped: None, dash: None }
	}
}

impl Shown {
	pub fn of(v: &Values) -> Shown {
		let mm = [v[OFFSET_MM], v[OFFSET_MM + 1], v[OFFSET_MM + 2]];
		let deg = [v[OFFSET_DEG], v[OFFSET_DEG + 1], v[OFFSET_DEG + 2]];
		let offset = (mm != [0.0; 3] || deg != [0.0; 3]).then(|| {
			// turned about the device's x, then its y, then its z
			let about = |axis: usize, deg: f64| {
				let (s, c) = sin_cos(0.5 * deg.to_radians());
				let mut q = [c, 0.0, 0.0, 0.0];
				q[1 + axis] = s;
				Q4::new(q[0], q[1], q[2], q[3])
			};
			Pose {
				p: V3::new(mm[0] * 1e-3, mm[1] * 1e-3, mm[2] * 1e-3),
				q: qmul(qmul(about(0, deg[0]), about(1, deg[1])), about(2, deg[2])),
			}
		});
		Shown {
			velocity_scale: 1.0 + v[VELOCITY_BOOST] * 1e-2,
			prediction: 1.0 - v[MOVEMENT_SMOOTHING] * 1e-2,
			offset,
			reshaped: None,
			dash: None,
		}
	}
}

/// The output layer's settings
#[derive(Clone, Debug, PartialEq)]
pub struct Output {
	/// per class (CLASSES), per setting (SETTINGS)
	pub class: [[f64; SETTINGS.len()]; CLASSES.len()],
	/// per device serial, the settings it has its own value for
	pub device: BTreeMap<Vec<u8>, [Option<f64>; SETTINGS.len()]>,
}

impl Default for Output {
	fn default() -> Output {
		let mut class = [[0.0; SETTINGS.len()]; CLASSES.len()];
		for (c, row) in class.iter_mut().enumerate() {
			for (i, s) in SETTINGS.iter().enumerate() {
				row[i] = s.default[c];
			}
		}
		Output { class, device: BTreeMap::new() }
	}
}

impl Output {
	/// A device's value of a setting: its own, else its class's
	pub fn value(&self, serial: &[u8], kind: Kind, i: usize) -> f64 {
		self.device.get(serial).and_then(|d| d[i]).unwrap_or(self.class[class_of(kind)][i])
	}

	/// A device's value of every setting
	pub fn values(&self, serial: &[u8], kind: Kind) -> Values {
		let mut v = self.class[class_of(kind)];
		if let Some(d) = self.device.get(serial) {
			for (v, own) in v.iter_mut().zip(d) {
				if let Some(own) = own {
					*v = *own;
				}
			}
		}
		v
	}

	/// Set one (`scope` a class or a serial, `value` a number in the setting's range, or `-` to take a device's own
	/// value away). Err says why not
	pub fn set(&mut self, scope: &[u8], name: &[u8], value: &[u8]) -> Result<(), &'static str> {
		let Some(i) = SETTINGS.iter().position(|s| s.name.as_bytes() == name) else { return Err("no such setting") };
		let class = CLASSES.iter().position(|c| c.as_bytes() == scope);
		if value == b"-" {
			if class.is_some() {
				return Err("a class always has a value");
			}
			if let Some(d) = self.device.get_mut(scope) {
				d[i] = None;
				if d.iter().all(|v| v.is_none()) {
					self.device.remove(scope);
				}
			}
			return Ok(());
		}
		let v = c_atof(value);
		if !v.is_finite() || v < SETTINGS[i].min || v > SETTINGS[i].max {
			return Err("out of range");
		}
		match class {
			Some(c) => self.class[c][i] = v,
			None => {
				if scope.is_empty() || !scope.iter().all(|&b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_') {
					return Err("not a class or a serial");
				}
				self.device.entry(scope.to_vec()).or_insert([None; SETTINGS.len()])[i] = Some(v);
			}
		}
		Ok(())
	}

	/// Every toy switched off: the classes' switches at 0 and a device's own switch taken away (it follows its class).
	/// The toys' numbers and every other setting stay. The switches that were on, as `<scope>.<name>`; None when no
	/// switch was on and no device had one of its own (nothing changed)
	pub fn toys_off(&mut self) -> Option<Vec<Vec<u8>>> {
		let mut were_on = Vec::new();
		let mut changed = false;
		let named = |scope: &[u8], i: usize| [scope, b".", SETTINGS[i].name.as_bytes()].concat();
		for (c, class) in CLASSES.iter().enumerate() {
			for i in TOY_SWITCHES {
				if self.class[c][i] != 0.0 {
					were_on.push(named(class.as_bytes(), i));
					self.class[c][i] = 0.0;
					changed = true;
				}
			}
		}
		for (serial, d) in self.device.iter_mut() {
			for i in TOY_SWITCHES {
				if let Some(v) = d[i].take() {
					changed = true;
					if v != 0.0 {
						were_on.push(named(serial, i));
					}
				}
			}
		}
		self.device.retain(|_, d| d.iter().any(|v| v.is_some()));
		changed.then_some(were_on)
	}

	/// One `<scope>.<name>=<value>` line (sally_output.txt); anything else is left alone
	pub fn parse_line(&mut self, line: &[u8]) {
		let s = trim(line);
		if s.is_empty() || s[0] == b'#' {
			return;
		}
		// the name's dot is the last one before the '=' (a value has its own: 137.5)
		let Some(eq) = s.iter().position(|&c| c == b'=') else { return };
		let Some(dot) = s[..eq].iter().rposition(|&c| c == b'.') else { return };
		if let Err(why) = self.set(trim(&s[..dot]), trim(&s[dot + 1..eq]), trim(&s[eq + 1..])) {
			log("sally: sally_output.txt: %s (%s)\n", &[BArg::S(s), BArg::S(why.as_bytes())]);
		}
	}

	/// The file's text: the classes, then the devices with values of their own
	pub fn text(&self) -> Vec<u8> {
		let mut t =
			b"# sally's output layer (docs/sally_driver.md): what SteamVR is shown, per class and per device.\n\
		               # The driver writes this file when the panel changes a setting.\n"
				.to_vec();
		for (c, name) in CLASSES.iter().enumerate() {
			for (i, s) in SETTINGS.iter().enumerate() {
				t.extend(bsprintf(
					"%s.%s=%g\n",
					&[BArg::S(name.as_bytes()), BArg::S(s.name.as_bytes()), BArg::F(self.class[c][i])],
				));
			}
		}
		for (serial, d) in &self.device {
			for (i, s) in SETTINGS.iter().enumerate() {
				if let Some(v) = d[i] {
					t.extend(bsprintf("%s.%s=%g\n", &[BArg::S(serial), BArg::S(s.name.as_bytes()), BArg::F(v)]));
				}
			}
		}
		t
	}

	/// The `O` lines for the panel: one per class and setting, one per device's own value, then `O end`
	pub fn lines(&self) -> Vec<Vec<u8>> {
		let mut l = Vec::new();
		for (c, name) in CLASSES.iter().enumerate() {
			for (i, s) in SETTINGS.iter().enumerate() {
				l.push(bsprintf(
					"O %s %s %g\n",
					&[BArg::S(name.as_bytes()), BArg::S(s.name.as_bytes()), BArg::F(self.class[c][i])],
				));
			}
		}
		for (serial, d) in &self.device {
			for (i, s) in SETTINGS.iter().enumerate() {
				if let Some(v) = d[i] {
					l.push(bsprintf("O %s %s %g\n", &[BArg::S(serial), BArg::S(s.name.as_bytes()), BArg::F(v)]));
				}
			}
		}
		l.push(b"O end\n".to_vec());
		l
	}
}

struct Store {
	output: Output,
	/// sally_output.txt, once the driver knows its folder
	path: Option<Vec<u8>>,
	/// when a setting was last set from the panel and the file has not been written since (`save_due`)
	changed: Option<f64>,
}

static G_OUTPUT: LazyLock<Mutex<Store>> =
	LazyLock::new(|| Mutex::new(Store { output: Output::default(), path: None, changed: None }));

/// A setting set from the panel is in the file this long after the last one set (s): a slider being dragged sets
/// its value at every frame of the panel's, and the file is written once, when it comes to rest
const SAVE_AFTER_S: f64 = 0.5;

fn store() -> MutexGuard<'static, Store> {
	G_OUTPUT.lock().unwrap_or_else(|e| e.into_inner())
}

/// The settings as they stand
pub fn g_output() -> Output {
	store().output.clone()
}

/// A device's value of every setting
pub fn values(serial: &[u8], kind: Kind) -> Values {
	store().output.values(serial, kind)
}

/// The settings from now on (a test's)
pub fn set_g_output(o: Output) {
	store().output = o;
}

/// <dir>/sally_output.txt read (its settings over the defaults), and kept as the file the panel's changes go to
pub fn read_output(dir: &[u8]) {
	let mut path = dir.to_vec();
	path.push(K_SEP);
	path.extend_from_slice(b"sally_output.txt");
	let mut o = Output::default();
	if let Some(mut f) = CFile::open(&path, c"rb") {
		let mut line = [0u8; 1024];
		let mut headset_blend = false;
		while let Some(l) = f.gets(&mut line) {
			headset_blend |= trim(l).starts_with(b"headset.recovery_blend_ms");
			o.parse_line(l);
		}
		f.close();
		if !headset_blend {
			// a file from before the headset had a class of its own (0.11): its blend was the trackers'
			o.class[0][RECOVERY_BLEND_MS] = o.class[2][RECOVERY_BLEND_MS];
		}
		log("sally: read %s\n", &[BArg::S(&path)]);
	} else {
		log("sally: no %s, the output layer's defaults\n", &[BArg::S(&path)]);
	}
	let mut s = store();
	s.output = o;
	s.path = Some(path);
	drop(s);
	// a toy left on by a SteamVR that did not close in order (a crash, a kill) is off for this session too
	toys_off("SteamVR starts");
}

/// The settings written to sally_output.txt, once the driver knows its folder
fn write_file(s: &mut Store) {
	s.changed = None;
	if let Some(path) = &s.path {
		if !write_whole(path, &s.output.text()) {
			log("sally: cannot write %s\n", &[BArg::S(path)]);
		}
	}
}

/// What the panel set since the file was last written, written once nothing has been set for SAVE_AFTER_S (the
/// provider's RunFrame), or at once (`now` None: its Cleanup)
pub fn save_due(now: Option<f64>) {
	let mut s = store();
	if s.changed.is_some_and(|at| now.is_none_or(|now| now - at >= SAVE_AFTER_S)) {
		write_file(&mut s);
	}
}

/// Every toy switched off and the file written, where one was on (`Output::toys_off`); `when` for the log: as SteamVR
/// closes (the provider's Cleanup) and as it starts. A toy is on only in the session it was switched on in: one left on
/// would carry into sessions it was not switched on for
pub fn toys_off(when: &str) {
	let mut s = store();
	if let Some(were_on) = s.output.toys_off() {
		let list = if were_on.is_empty() {
			b"none on, a device's own switch taken away".to_vec()
		} else {
			were_on.join(&b' ')
		};
		log("sally: the toys off as %s (%s)\n", &[BArg::S(when.as_bytes()), BArg::S(&list)]);
		write_file(&mut s);
	}
}

/// The `output` command: `output` alone lists the settings, `output <scope> <name> <value>` sets one first (the file
/// is written by `save_due`); the reply's lines either way, after an `O error <why>` line when the set was refused
pub fn command(args: &[&[u8]]) -> Vec<Vec<u8>> {
	let mut s = store();
	let mut reply = Vec::new();
	if args.len() == 3 {
		match s.output.set(args[0], args[1], args[2]) {
			Ok(()) => {
				log("sally: output %s.%s = %s\n", &[BArg::S(args[0]), BArg::S(args[1]), BArg::S(args[2])]);
				s.changed = Some(now_seconds());
			}
			Err(why) => reply.push(bsprintf("O error %s\n", &[BArg::S(why.as_bytes())])),
		}
	} else if !args.is_empty() {
		reply.push(b"O error usage: output [<scope> <name> <value>]\n".to_vec());
	}
	reply.extend(s.output.lines());
	reply
}

// ---------------------------------------------------------------- the recovery blend

/// The light's return, as the runtime reports it (its Q line's last two fields), is blended when the device was dark
/// for at least this long (s) ...
pub const RETURN_DARK_S: f64 = 0.1;
/// ... for the corrections of this long after the return began (s): the restart, the hypothesis's confirmation (0.2
/// s), the other stations' first sweeps
pub const RETURN_WINDOW_S: f64 = 0.3;
/// The blend's settle time is when 95 % of a jump has been shown: a critically damped glide x(t) = (1 + w t) exp(-w t)
/// is at 0.05 when w t = 4.744
pub(crate) const SETTLE_WT: f64 = 4.744;
/// The smoothing lets a correction in sooner on a device that moves: within this much travel (m) ...
pub const GLIDE_TRAVEL_M: f64 = 0.05;
/// ... or this much turning (rad), when that is sooner than its settle time. Both from the bench (sally_driver.md,
/// "The output layer"): shorter (2 cm, 0.1 rad) gives away the glide in a sway, where the waist's snaps are; longer
/// (15 cm, 0.75 rad) costs what SteamVR shows ahead in motion and gains nothing
pub const GLIDE_TURN_RAD: f64 = 0.25;
/// The smoothing takes a device for still by how it has moved lately: its head's velocity and its rate of turning,
/// each averaged over this long (s). A tremble goes back and forth within it and cancels; a move does not ...
pub const STILL_AVERAGE_S: f64 = 0.06;
/// ... and the device is still while those averages are well under these (m/s, rad/s): from them on its motion is
/// shown whole. A body part held still sways and trembles at 0.2 to 2 cm/s and turns at 0.01 to 0.1 rad/s ...
pub const STILL_SPEED: f64 = 0.05;
pub const STILL_TURN: f64 = 0.3;
/// ... and while its velocity and its rate are within these of their averages (m/s, rad/s): a move that begins is
/// away from its average before the average has it (at 2 m/s^2 it is taken for moving in 50 ms), a tremble is not ...
pub const STILL_JOLT: f64 = 0.08;
pub const STILL_JOLT_TURN: f64 = 0.5;
/// ... and once it has moved it stays moving for a while: the measure of its motion (1 at the limits above) falls
/// with this time constant (s), from STILL_CAP at most, so a hand going back and forth is not still at each end,
/// and one that has stopped is shown still again 0.15 to 0.4 s later
pub const STILL_HOLD_S: f64 = 0.1;
pub const STILL_CAP: f64 = 4.0;
/// And still is a matter of how far as well as how fast: a tremble stays within this of where the device rests (m,
/// rad). What a still device moves by is held back only up to here. A device that goes further has moved, however
/// slowly (a weight shifted, a slow turn: under the speeds above for a second or more): it is shown this far behind
/// and no further, and is taken for moving. Without it a slow move was shown late by the settle time and then caught
/// up (0.28: a 10 degree turn in 2 s a tenth of a second late at 300 ms and a quarter to half a second at 1000). From
/// the bench (sally_driver.md, "The output layer"): half of it shows a third more of a still tracker's jitter in
/// orientation, twice of it shows a slow turn twice as far behind and no less jitter ...
pub const STILL_REACH_M: f64 = 0.001;
pub const STILL_REACH_RAD: f64 = 0.003;
/// ... and the device counts for moving over the last part of that reach, from this share of it on
const STILL_REACH_EDGE: f64 = 0.8;

/// What a device's blend carries between reports: the offset between the runtime's pose and the one shown, which the
/// return's corrections add to and which then settles back to nothing. The offset moves as a critically damped spring,
/// so a jump starts and ends without a kick (x the offset, v its rate; position in metres, world, orientation as a
/// world-side rotation vector)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Blend {
	have: bool,
	/// the driver's clock at the last report
	t: f64,
	/// the runtime's summed corrections at the last report (head point, world; orientation, world side)
	corr: [f64; 3],
	corr_q: [f64; 4],
	x: [f64; 3],
	v: [f64; 3],
	xr: [f64; 3],
	vr: [f64; 3],
}

pub(crate) fn qlog(q: &[f64; 4]) -> [f64; 3] {
	let (w, v) = if q[0] < 0.0 { (-q[0], [-q[1], -q[2], -q[3]]) } else { (q[0], [q[1], q[2], q[3]]) };
	let s = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
	if s < 1e-12 {
		return [2.0 * v[0], 2.0 * v[1], 2.0 * v[2]];
	}
	let k = 2.0 * s.atan2(w) / s;
	[v[0] * k, v[1] * k, v[2] * k]
}

pub(crate) fn qexp(r: &[f64; 3]) -> [f64; 4] {
	let th = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
	if th < 1e-12 {
		return [1.0, 0.5 * r[0], 0.5 * r[1], 0.5 * r[2]];
	}
	let (s, c) = sin_cos(0.5 * th);
	[c, r[0] * s / th, r[1] * s / th, r[2] * s / th]
}

/// x, v after dt of x'' = -2 w x' - w^2 x: x(t) = (x0 + b t) exp(-w t), b = v0 + w x0
pub(crate) fn settle(x: &mut [f64; 3], v: &mut [f64; 3], w: f64, dt: f64) {
	let e = (-w * dt).exp();
	for i in 0..3 {
		let b = v[i] + w * x[i];
		let xt = x[i] + b * dt;
		x[i] = xt * e;
		v[i] = (b - w * xt) * e;
	}
}

impl Blend {
	/// The pose to show for one report: `pos` / `rot` the runtime's (head point, Z-up world), `corr` the Q line's last
	/// nine fields (the summed corrections, the light's return), `now` the driver's clock, `settle_ms` the device's
	/// recovery_blend_ms, `gap_s` how long without a report starts the blend over (the driver has shown Valve or a held
	/// pose meanwhile). Returns whether the shown pose differs from the runtime's
	pub fn show(
		&mut self, pos: &mut [f64; 3], rot: &mut [f64; 4], corr: &[f64; 9], now: f64, settle_ms: f64, gap_s: f64,
	) -> bool {
		let rate = (settle_ms > 0.0).then(|| SETTLE_WT / (settle_ms * 1e-3));
		self.advance(corr, now, gap_s, rate, in_return(corr));
		self.apply(pos, rot)
	}

	/// One report: the offset settled over the time since the last one at `rate` (1/s; None: no glide, the offset
	/// dropped), and this report's corrections added to it when they are this glide's (`take`)
	fn advance(&mut self, corr: &[f64; 9], now: f64, gap_s: f64, rate: Option<f64>, take: bool) {
		let cp = [corr[0], corr[1], corr[2]];
		let cq = [corr[3], corr[4], corr[5], corr[6]];
		let fresh = self.have && now >= self.t && now - self.t <= gap_s;
		match rate {
			Some(w) if fresh && corr.iter().all(|x| x.is_finite()) => {
				let dt = now - self.t;
				settle(&mut self.x, &mut self.v, w, dt);
				settle(&mut self.xr, &mut self.vr, w, dt);
				if take {
					// the corrections since the last report: kept out of what is shown, for the spring to let in
					let e = qlog(&quat_mul(&cq, &quat_conj(&self.corr_q)));
					for i in 0..3 {
						self.x[i] += cp[i] - self.corr[i];
						self.xr[i] += e[i];
					}
				}
			}
			_ => *self = Blend::default(),
		}
		if corr.iter().all(|x| x.is_finite()) {
			self.have = true;
			self.t = now;
			self.corr = cp;
			self.corr_q = cq;
		}
	}

	/// The offset taken out of a pose; whether there is one
	fn apply(&self, pos: &mut [f64; 3], rot: &mut [f64; 4]) -> bool {
		unoffset(&self.x, &self.xr, pos, rot)
	}

	/// The offset now shown (m, rad), for the stats
	pub fn offset(&self) -> (f64, f64) {
		let n = |a: &[f64; 3]| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
		(n(&self.x), n(&self.xr))
	}
}

// ---------------------------------------------------------------- the smoothing

/// What the smoothing goes by: the pose and the rates the hand-over makes of a report as it came (runtime_device.rs,
/// `look_ahead`): the head point and the orientation (the runtime's world), the head point's velocity and the rate
/// of turning (world)
#[derive(Clone, Copy, Debug)]
pub struct Own<'a> {
	pub pos: &'a [f64; 3],
	pub rot: &'a [f64; 4],
	pub vel: &'a [f64; 3],
	pub turn: &'a [f64; 3],
}

/// The smoothing of a device between its reports (`smoothing_ms`). A pose changes from one report to the next by the
/// device's own motion (the IMU's) and by the light's corrections; the runtime's sums say which is which. While the
/// device is held still its own motion is a tremble, a pulse and the sensors' noise, and SteamVR shows a pose
/// 40 ms on by its velocity, which makes two to five times as much of them. So what a still device moves by is not
/// shown as it comes: it goes into an offset between the tracker's pose and the one shown, as the corrections do,
/// and the offset settles back to nothing as a critically damped glide (the shown pose is the tracker's through a
/// low-pass of two equal poles at 4.744 / the settle time); and the rates handed on are scaled down with it
/// (`step`'s return), so nothing carries the tremble on. A device that moves is shown as the tracker has it: all
/// of its motion passes, with its rates, and only the corrections go through the glide, which lets them in sooner
/// the faster it moves or turns (GLIDE_*). It moves when it is fast enough (STILL_SPEED and the rest) or has gone
/// further than a tremble does (STILL_REACH_*): what is held back of its own motion never comes to more than that
/// reach, so a move that begins slowly is shown a millimetre and a sixth of a degree behind at most, not late by
/// the settle time. Between still and moving the share that passes follows how much it moves, so nothing steps
/// as a move begins or ends.
/// The corrections of the light's return are not the smoothing's (they are the recovery blend's, or come as they
/// are). A headset is never taken for still: what is seen through it has to turn with the head, so its motion always
/// passes and only its corrections glide
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Smooth {
	have: bool,
	/// the driver's clock at the last report
	t: f64,
	/// the tracker's pose at the last report (`Own`'s) and the runtime's summed corrections
	p: [f64; 3],
	q: [f64; 4],
	corr: [f64; 3],
	corr_q: [f64; 4],
	/// how the device has moved lately: its velocity and its rate averaged over STILL_AVERAGE_S, and the measure of
	/// its motion (1 and over: moving)
	v_avg: [f64; 3],
	w_avg: [f64; 3],
	moving: f64,
	/// the offset between the tracker's pose and the one shown, and its rate (as the blend's)
	x: [f64; 3],
	v: [f64; 3],
	xr: [f64; 3],
	vr: [f64; 3],
	/// what of the device's own motion is held back while it is still (within STILL_REACH_*), and its rate
	h: [f64; 3],
	hv: [f64; 3],
	hr: [f64; 3],
	hrv: [f64; 3],
}

impl Smooth {
	/// One report: `own` the pose and the rates the hand-over makes of it as it came (None for a report without a
	/// velocity), `corr` the Q line's last nine fields, `now` the driver's clock, `settle_ms` the device's
	/// smoothing_ms, `gap_s` how long without a report starts over. `hold` false leaves the device's own motion
	/// alone however still it is, and only the corrections glide: a headset's, where the picture has to turn with
	/// the head. Returns the share of the device's own motion that is shown as it comes, which is what its rates are
	/// handed on scaled by: 1 for a device that moves (and with no smoothing), 0 for one that is still
	pub fn step(
		&mut self, own: Option<&Own>, corr: &[f64; 9], now: f64, settle_ms: f64, gap_s: f64, hold: bool,
	) -> f64 {
		let Some(own) = own.filter(|o| {
			settle_ms > 0.0 && corr.iter().chain(o.pos).chain(o.rot).chain(o.vel).chain(o.turn).all(|x| x.is_finite())
		}) else {
			*self = Smooth::default();
			return 1.0;
		};
		let cp = [corr[0], corr[1], corr[2]];
		let cq = [corr[3], corr[4], corr[5], corr[6]];
		if !(self.have && now >= self.t && now - self.t <= gap_s) {
			// the first report, or the first after a gap: shown as it is, and taken for moving until it has been still
			*self = Smooth {
				have: true,
				t: now,
				p: *own.pos,
				q: *own.rot,
				corr: cp,
				corr_q: cq,
				v_avg: *own.vel,
				w_avg: *own.turn,
				moving: STILL_CAP,
				..Smooth::default()
			};
			return 1.0;
		}
		let dt = now - self.t;
		// how much the device moves: its averages against the limits, and how far it is from them
		let a = 1.0 - (-dt / STILL_AVERAGE_S).exp();
		let sq = |x: &[f64; 3]| x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
		let (mut dv, mut dw) = ([0.0; 3], [0.0; 3]);
		for i in 0..3 {
			self.v_avg[i] += a * (own.vel[i] - self.v_avg[i]);
			self.w_avg[i] += a * (own.turn[i] - self.w_avg[i]);
			dv[i] = own.vel[i] - self.v_avg[i];
			dw[i] = own.turn[i] - self.w_avg[i];
		}
		let moves = sq(&self.v_avg) / (STILL_SPEED * STILL_SPEED)
			+ sq(&dv) / (STILL_JOLT * STILL_JOLT)
			+ sq(&self.w_avg) / (STILL_TURN * STILL_TURN)
			+ sq(&dw) / (STILL_JOLT_TURN * STILL_JOLT_TURN);
		self.moving = (self.moving * (-dt / STILL_HOLD_S).exp()).max(moves).min(STILL_CAP);
		let open = if hold { self.moving.min(1.0) } else { 1.0 };
		// the offset settles: in the settle time while the device is still, sooner the faster it moves or turns
		let rate = SETTLE_WT
			* (1.0 / (settle_ms * 1e-3)
				+ open * (sq(own.vel).sqrt() / GLIDE_TRAVEL_M + sq(own.turn).sqrt() / GLIDE_TURN_RAD));
		settle(&mut self.x, &mut self.v, rate, dt);
		settle(&mut self.xr, &mut self.vr, rate, dt);
		// this report's change of the pose: the corrections' part, kept out of what is shown for the glide to let in
		// (but a return's) ...
		let dc = [cp[0] - self.corr[0], cp[1] - self.corr[1], cp[2] - self.corr[2]];
		let dcr = qlog(&quat_mul(&cq, &quat_conj(&self.corr_q)));
		let dr = qlog(&quat_mul(own.rot, &quat_conj(&self.q)));
		let take = if in_return(corr) { 0.0 } else { 1.0 };
		for i in 0..3 {
			self.x[i] += take * dc[i];
			self.xr[i] += take * dcr[i];
		}
		// ... and the device's own motion, of which the share that is not shown as it comes is held back: let in over
		// the settle time, and never more of it than a tremble's reach. A device that goes further than that has
		// moved, however slowly: it is shown that far behind at most, and is taken for moving from there on
		let still = SETTLE_WT / (settle_ms * 1e-3);
		settle(&mut self.h, &mut self.hv, still, dt);
		settle(&mut self.hr, &mut self.hrv, still, dt);
		for i in 0..3 {
			self.h[i] += (1.0 - open) * (own.pos[i] - self.p[i] - dc[i]);
			self.hr[i] += (1.0 - open) * (dr[i] - dcr[i]);
		}
		let mut gone: f64 = 0.0;
		for (h, hv, reach) in
			[(&mut self.h, &mut self.hv, STILL_REACH_M), (&mut self.hr, &mut self.hrv, STILL_REACH_RAD)]
		{
			let n = sq(h).sqrt();
			if n > reach {
				// at the end of its reach: no further, and no rate that takes it there
				let out = (h[0] * hv[0] + h[1] * hv[1] + h[2] * hv[2]) / (n * n);
				for i in 0..3 {
					if out > 0.0 {
						hv[i] -= out * h[i];
					}
					h[i] *= reach / n;
				}
			}
			gone = gone.max((n / reach).min(1.0));
		}
		// the last part of the reach counts for moving, smoothly: 0 up to STILL_REACH_EDGE of it, 1 at its end
		let u = ((gone - STILL_REACH_EDGE) / (1.0 - STILL_REACH_EDGE)).max(0.0);
		let u = u * u * (3.0 - 2.0 * u);
		self.moving = self.moving.max(u);
		let open = open.max(u);
		(self.t, self.p, self.q, self.corr, self.corr_q) = (now, *own.pos, *own.rot, cp, cq);
		open
	}

	/// The offset taken out of a pose (the runtime's, with the recovery blend's glide if there is one); whether there
	/// is one
	pub fn apply(&self, pos: &mut [f64; 3], rot: &mut [f64; 4]) -> bool {
		let x: [f64; 3] = std::array::from_fn(|i| self.x[i] + self.h[i]);
		let xr: [f64; 3] = std::array::from_fn(|i| self.xr[i] + self.hr[i]);
		unoffset(&x, &xr, pos, rot)
	}
}

/// An offset (the point's, m; the turn's, a rotation vector on the world's side) taken out of a pose; whether there
/// is one
fn unoffset(x: &[f64; 3], xr: &[f64; 3], pos: &mut [f64; 3], rot: &mut [f64; 4]) -> bool {
	let off = x.iter().chain(xr).any(|&x| x != 0.0);
	if off {
		for i in 0..3 {
			pos[i] -= x[i];
		}
		let r = quat_mul(&qexp(&[-xr[0], -xr[1], -xr[2]]), rot);
		let n = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]).sqrt();
		for i in 0..4 {
			rot[i] = r[i] / n;
		}
	}
	off
}

// ---------------------------------------------------------------- the punch rail

/// A punch stays on its rail while the hand's speed along it is at least this share of `rail_speed` ...
const RAIL_RELEASE: f64 = 0.5;
/// ... and this share of its speed (the hand goes more along the line than aside: within 45 degrees of it; a swing
/// that curves is let off as it turns, before the hand is far from its line) ...
const RAIL_TURN: f64 = std::f64::consts::FRAC_1_SQRT_2;
/// ... and no longer than this (s): a punch is over in a third of that
const RAIL_MAX_S: f64 = 0.8;
/// The rail's line is the way the hand goes over this long from where it got fast enough (s): the first reports'
/// velocities summed, so the faster ones say more
const RAIL_AIM_S: f64 = 0.04;
/// The rail takes the hand over in this long (s): the shown hand starts where the hand is, and the rail comes in
/// without a kick at either end (its weight a smooth step)
const RAIL_IN_S: f64 = 0.03;
/// Off the rail, what the shown hand is ahead and aside of the hand by is eased back in this long (s, 95 % there)
const RAIL_BACK_S: f64 = 0.15;
/// After a punch, a hand going the other way within this long (s) is coming back, not punching: no rail for it
const RAIL_RETRACT_S: f64 = 0.5;
/// The next rail waits until the hand has been slower than this share of `rail_speed`
const RAIL_REARM: f64 = 0.8;
/// No report for this long (s) starts the rail over
const RAIL_GAP_S: f64 = 0.1;

/// What the punch rail does to the pose a report hands over (toys.rs, `Toys::step`), in the runtime's world as the
/// driver has it (OpenVR's axes, before the fit to Valve's frame): with `weight` of it, the point is put
/// on the line from `origin` along `dir` at `gain` times how far along it is, and its velocity is its velocity along
/// the line times `gain` (and, while the weight comes in at `weight_rate`, what that moves the point by); then `shift`
/// (and its rate, on the velocity), what is being eased back after a punch. The velocity is the shown point's own,
/// so SteamVR's straight line goes on where the point goes; and so is the acceleration (`map_acc`: the weight's and
/// the shift's second rates with it), for the carry
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RailShown {
	pub origin: V3,
	pub dir: V3,
	pub gain: f64,
	pub weight: f64,
	pub weight_rate: f64,
	pub weight_acc: f64,
	pub shift: V3,
	pub shift_rate: V3,
	pub shift_acc: V3,
}

impl RailShown {
	/// A point and its velocity as the rail shows them
	pub fn map(&self, p: V3, v: V3) -> (V3, V3) {
		let on_p = self.origin + self.dir * (self.gain * dot(p - self.origin, self.dir));
		let on_v = self.dir * (self.gain * dot(v, self.dir));
		(
			p + (on_p - p) * self.weight + self.shift,
			v + (on_v - v) * self.weight + (on_p - p) * self.weight_rate + self.shift_rate,
		)
	}

	/// The shown point's acceleration, where the point is at `p` going `v` with the acceleration `a`
	pub fn map_acc(&self, p: V3, v: V3, a: V3) -> V3 {
		let on_p = self.origin + self.dir * (self.gain * dot(p - self.origin, self.dir));
		let on_v = self.dir * (self.gain * dot(v, self.dir));
		let on_a = self.dir * (self.gain * dot(a, self.dir));
		a + (on_a - a) * self.weight
			+ (on_v - v) * (2.0 * self.weight_rate)
			+ (on_p - p) * self.weight_acc
			+ self.shift_acc
	}
}

/// The punch rail of a device between its reports (`rail_speed`, `rail_boost`): a cheat for games that score a punch
/// by the hand's speed, for a hand that is not accurate when it is only made faster. When the hand gets as fast as
/// `rail_speed` it is latched onto a line from where it is, along the way it goes (settled over the first
/// RAIL_AIM_S); on the line, what it moves sideways is left out and what it moves along it is shown `rail_boost`
/// percent further and faster. When its speed along the line falls under RAIL_RELEASE of `rail_speed` (the punch
/// ends) or it goes more aside than along the line (RAIL_TURN: the swing turns) it is let off, and what the shown
/// hand is off by is eased back (a critically damped glide, as the blends'). A hand coming straight back from a
/// punch gets no rail of its own
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Rail {
	have: bool,
	/// the driver's clock at the last report
	t: f64,
	/// a punch is on its rail: where it began, its line, when, the velocities summed for the line, how far the rail
	/// has come in (0 to 1 over RAIL_IN_S)
	on: bool,
	origin: [f64; 3],
	dir: [f64; 3],
	since: f64,
	aim: [f64; 3],
	ramp: f64,
	/// what the shown hand is still off by after a punch, and its rate
	shift: [f64; 3],
	shift_rate: [f64; 3],
	/// the last punch's line and when it ended
	last_dir: [f64; 3],
	released: f64,
	/// the hand has slowed since the last time it was fast
	armed: bool,
}

pub(crate) fn v3(a: &[f64; 3]) -> V3 {
	V3::new(a[0], a[1], a[2])
}

pub(crate) fn arr(a: V3) -> [f64; 3] {
	[a.x, a.y, a.z]
}

impl Rail {
	/// One report: `p` and `v` the pose's point and its velocity as they are handed over (the runtime's world in
	/// OpenVR's axes), `now` the driver's clock, `rail_speed` (m/s) and `rail_boost` (percent) the device's. What the
	/// override is to do with the pose, None where there is nothing to do
	pub fn step(
		&mut self, now: f64, rail_speed: f64, rail_boost: f64, p: &[f64; 3], v: &[f64; 3],
	) -> Option<RailShown> {
		let finite = p.iter().chain(v).all(|x| x.is_finite());
		if !(rail_speed > 0.0) || !finite {
			*self = Rail::default();
			return None;
		}
		let (p, v) = (v3(p), v3(v));
		let gain = 1.0 + rail_boost.max(0.0) * 1e-2;
		if !(self.have && now >= self.t && now - self.t <= RAIL_GAP_S) {
			*self = Rail { armed: self.armed, released: f64::NEG_INFINITY, ..Rail::default() };
		}
		let dt = if self.have { now - self.t } else { 0.0 };
		self.have = true;
		self.t = now;
		settle(&mut self.shift, &mut self.shift_rate, SETTLE_WT / RAIL_BACK_S, dt);
		let speed = len(v);
		if self.on {
			if now - self.since <= RAIL_AIM_S {
				// the line is still being taken: the way the hand has gone since it got fast
				let aim = v3(&self.aim) + v;
				self.aim = arr(aim);
				if len(aim) > 0.0 {
					self.dir = arr(aim * (1.0 / len(aim)));
				}
			}
			self.ramp = (self.ramp + dt / RAIL_IN_S).min(1.0);
			let along = dot(v, v3(&self.dir));
			if along < RAIL_RELEASE * rail_speed || along < RAIL_TURN * speed || now - self.since > RAIL_MAX_S {
				// off the rail: what the shown hand is off by, and how fast that changes, go into the ease-back, so
				// neither the point nor its velocity steps
				let (sp, sv) = self.shown(gain).map(p, v);
				self.shift = arr(sp - p);
				self.shift_rate = arr(sv - v);
				self.on = false;
				self.ramp = 0.0;
				self.last_dir = self.dir;
				self.released = now;
			}
		} else {
			if speed < RAIL_REARM * rail_speed {
				self.armed = true;
			}
			if self.armed && speed >= rail_speed {
				self.armed = false;
				let d = v * (1.0 / speed);
				// a hand coming back from a punch is not a punch
				let back = now - self.released < RAIL_RETRACT_S && dot(d, v3(&self.last_dir)) < 0.0;
				if !back {
					self.on = true;
					self.origin = arr(p);
					self.dir = arr(d);
					self.aim = arr(v);
					self.since = now;
					self.ramp = 0.0;
				}
			}
		}
		// eased back to within a micrometre: nothing left to show
		let small = |a: &[f64; 3], lim: f64| a.iter().all(|x| x.abs() < lim);
		if !self.on && small(&self.shift, 1e-6) && small(&self.shift_rate, 1e-5) {
			(self.shift, self.shift_rate) = ([0.0; 3], [0.0; 3]);
			return None;
		}
		Some(self.shown(gain))
	}

	fn shown(&self, gain: f64) -> RailShown {
		// the weight a smooth step of the ramp: no rate at either end
		let r = if self.on { self.ramp } else { 0.0 };
		// the ease-back's glide: x'' = -2 w x' - w^2 x
		let wb = SETTLE_WT / RAIL_BACK_S;
		RailShown {
			origin: v3(&self.origin),
			dir: v3(&self.dir),
			gain,
			weight: r * r * (3.0 - 2.0 * r),
			weight_rate: 6.0 * r * (1.0 - r) / RAIL_IN_S,
			weight_acc: if self.on && r < 1.0 { (6.0 - 12.0 * r) / (RAIL_IN_S * RAIL_IN_S) } else { 0.0 },
			shift: v3(&self.shift),
			shift_rate: v3(&self.shift_rate),
			shift_acc: v3(&self.shift_rate) * (-2.0 * wb) + v3(&self.shift) * (-wb * wb),
		}
	}

	/// A punch is on its rail
	pub fn on(&self) -> bool {
		self.on
	}
}

/// Whether a report's corrections are the light's return's (the Q line's last two fields)
fn in_return(corr: &[f64; 9]) -> bool {
	let (ret_age, ret_gap) = (corr[7], corr[8]);
	(0.0..=RETURN_WINDOW_S).contains(&ret_age) && ret_gap >= RETURN_DARK_S
}

#[cfg(test)]
mod tests {
	use super::*;

	/// The tests that read and write the driver's one set of settings and its file take turns
	static STORE_TEST: Mutex<()> = Mutex::new(());

	fn corr(p: [f64; 3], age: f64, gap: f64) -> [f64; 9] {
		[p[0], p[1], p[2], 1.0, 0.0, 0.0, 0.0, age, gap]
	}

	/// The defaults are the tracker's own output: every setting that changes what SteamVR is shown is 0 in every class,
	/// so nothing does until it is set. The two that are not 0 change nothing by being there: the prediction is the
	/// pose's own age (9 ms over the radio, 0 on the headset), and a toy's numbers wait behind its switch. A default
	/// that becomes something else changes what every install shows untouched: that is decided here first
	#[test]
	fn the_defaults_are_the_trackers_own_output() {
		let numbers = ["prediction_ms", "rail_speed", "rail_boost", "dash_percent", "dash_back_ms"];
		for s in &SETTINGS {
			if !numbers.contains(&s.name) {
				assert_eq!(s.default, [0.0; 3], "{}: a default that changes what SteamVR is shown", s.name);
			}
		}
		assert_eq!(SETTINGS[PREDICTION_MS].default, [0.0, 9.0, 9.0]);
		// each of those numbers has its switch, and the switch is off
		for (number, switch) in [(RAIL_SPEED, RAIL), (RAIL_BOOST, RAIL), (DASH_PERCENT, DASH), (DASH_BACK_MS, DASH)] {
			assert_eq!((SETTINGS[switch].default, SETTINGS[switch].max), ([0.0; 3], 1.0), "{}", SETTINGS[number].name);
		}
		// and so a report passes as it came: a correction of the light is shown at once, in a return or out of one, the
		// offset and the velocity are the device's own, and no toy shows another pose in its place
		let o = Output::default();
		for (kind, serial) in [(Kind::Headset, "H"), (Kind::LeftController, "C"), (Kind::Tracker, "T")] {
			let v = o.values(serial.as_bytes(), kind);
			assert_eq!(Shown::of(&v), Shown::default());
			for (age, gap) in [(-1.0, 0.0), (0.1, 0.8)] {
				let (mut blend, mut smooth) = (Blend::default(), Smooth::default());
				for k in 0..100 {
					let c = if k < 50 { 0.0 } else { 0.03 };
					let (mut p, mut r) = ([c, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0]);
					let t = k as f64 * 0.004;
					let a = blend.show(&mut p, &mut r, &corr([c, 0.0, 0.0], age, gap), t, v[RECOVERY_BLEND_MS], 0.1);
					let own = Own { pos: &[c, 0.0, 1.0], rot: &[1.0, 0.0, 0.0, 0.0], vel: &[0.0; 3], turn: &[0.0; 3] };
					let open = smooth.step(Some(&own), &corr([c, 0.0, 0.0], age, gap), t, v[SMOOTHING_MS], 0.1, true);
					let b = smooth.apply(&mut p, &mut r);
					assert!(
						!a && !b && open == 1.0 && p == [c, 0.0, 1.0] && r == [1.0, 0.0, 0.0, 0.0],
						"{kind:?} report {k}"
					);
				}
			}
			let k = Kin {
				p: [0.0; 3],
				q: [1.0, 0.0, 0.0, 0.0],
				v: [3.0, 0.0, 0.0],
				a: [0.0; 3],
				w: [0.0; 3],
				al: [0.0; 3],
				age: 0.0,
			};
			assert_eq!(crate::toys::Toys::default().step(kind, &v, 1.0, &k, &k), None);
		}
	}

	/// a value with a fraction comes back from the file as it was written
	#[test]
	fn a_fractional_value_reads_back() {
		let mut o = Output::default();
		o.set(b"trackers", b"recovery_blend_ms", b"137.5").unwrap();
		o.set(b"LHR-1", b"recovery_blend_ms", b"0.5").unwrap();
		let mut back = Output::default();
		for l in o.text().split(|&c| c == b'\n') {
			back.parse_line(l);
		}
		assert_eq!(back, o);
		// a line without a name's dot is left alone
		let mut d = Output::default();
		d.parse_line(b"recovery_blend_ms=1.5");
		assert_eq!(d, Output::default());
	}

	/// a return's jump is shown as a glide that is 95 % there at the settle time, without a kick at either end;
	/// outside a return the corrections pass as they come
	#[test]
	fn a_return_glides_and_the_rest_passes() {
		let mut b = Blend::default();
		let rot0 = [1.0, 0.0, 0.0, 0.0];
		let mut t = 0.0;
		let shown = |b: &mut Blend, est: [f64; 3], c: [f64; 9], t: f64| {
			let (mut p, mut r) = (est, rot0);
			b.show(&mut p, &mut r, &c, t, 150.0, 0.1);
			p
		};
		// tracking: corrections pass (the estimate is shown)
		for k in 0..50 {
			t = k as f64 * 0.004;
			let p = shown(&mut b, [0.001 * k as f64, 0.0, 1.0], corr([0.001 * k as f64, 0.0, 0.0], -1.0, 0.0), t);
			assert_eq!(p, [0.001 * k as f64, 0.0, 1.0]);
		}
		// the light back after 0.8 s dark: the estimate jumps 0.2 m along x
		let mut xs = Vec::new();
		for k in 0..200 {
			t += 0.004;
			let age = k as f64 * 0.004;
			let p = shown(&mut b, [0.249, 0.0, 1.0], corr([0.249, 0.0, 0.0], age, 0.8), t);
			xs.push(p[0]);
		}
		// the first report still shows where the device was; the jump settles over 150 ms
		assert!((xs[0] - 0.049).abs() < 1e-9, "{}", xs[0]);
		let at = |ms: f64| xs[(ms / 4.0) as usize];
		assert!(at(150.0) > 0.049 + 0.95 * 0.2 - 1e-3 && at(150.0) < 0.249, "{}", at(150.0));
		assert!((at(600.0) - 0.249).abs() < 1e-5);
		// no kick: the first step is a small part of the jump, and the glide never runs past it
		assert!(xs[1] - xs[0] < 0.01 * 0.2, "{}", xs[1] - xs[0]);
		assert!(xs.iter().all(|&x| x <= 0.249 + 1e-9));
		// a gap in the reports starts over: the estimate shown as it is
		let mut c = Blend::default();
		let mut p = [0.0, 0.0, 1.0];
		let mut r = rot0;
		c.show(&mut p, &mut r, &corr([0.0; 3], -1.0, 0.0), 0.0, 150.0, 0.1);
		let mut p = [0.3, 0.0, 1.0];
		c.show(&mut p, &mut r, &corr([0.3, 0.0, 0.0], 0.0, 1.0), 0.5, 150.0, 0.1);
		assert_eq!(p, [0.3, 0.0, 1.0]);
		// 0 ms: the jump as it comes
		let mut d = Blend::default();
		let mut p = [0.0, 0.0, 1.0];
		d.show(&mut p, &mut r, &corr([0.0; 3], -1.0, 0.0), 0.0, 0.0, 0.1);
		let mut p = [0.3, 0.0, 1.0];
		d.show(&mut p, &mut r, &corr([0.3, 0.0, 0.0], 0.0, 1.0), 0.004, 0.0, 0.1);
		assert_eq!(p, [0.3, 0.0, 1.0]);
	}

	/// a return's turn glides too: the orientation shown starts where it was
	#[test]
	fn a_return_turn_glides() {
		let mut b = Blend::default();
		let (mut p, mut r) = ([0.0; 3], [1.0, 0.0, 0.0, 0.0]);
		b.show(&mut p, &mut r, &corr([0.0; 3], -1.0, 0.0), 0.0, 150.0, 0.1);
		// the restart turns the device 0.3 rad about z (world side)
		let e = qexp(&[0.0, 0.0, 0.3]);
		let mut r = e;
		let c = [0.0, 0.0, 0.0, e[0], e[1], e[2], e[3], 0.0, 2.0];
		b.show(&mut p, &mut r, &c, 0.004, 150.0, 0.1);
		assert!((r[0] - 1.0).abs() < 1e-9 && r[3].abs() < 1e-9, "{r:?}");
		let mut last = 0.0;
		for k in 2..150 {
			let mut r = e;
			b.show(&mut p, &mut r, &c, k as f64 * 0.004, 150.0, 0.1);
			let ang = qlog(&r)[2];
			assert!(ang >= last - 1e-12 && ang <= 0.3 + 1e-9);
			last = ang;
		}
		assert!((last - 0.3).abs() < 1e-4, "{last}");
	}

	#[test]
	fn settings_parse_set_and_list() {
		let mut o = Output::default();
		assert_eq!(o.class[2][RECOVERY_BLEND_MS], 0.0);
		o.set(b"trackers", b"recovery_blend_ms", b"150").unwrap();
		o.parse_line(b"controllers.recovery_blend_ms=60");
		o.parse_line(b"LHR-9365E531.recovery_blend_ms = 0");
		o.parse_line(b"# a comment");
		o.parse_line(b"trackers.nope=3");
		assert_eq!(o.value(b"LHR-1", Kind::LeftController, RECOVERY_BLEND_MS), 60.0);
		assert_eq!(o.value(b"LHR-9365E531", Kind::LeftController, RECOVERY_BLEND_MS), 0.0);
		assert_eq!(o.value(b"LHR-1", Kind::Tracker, RECOVERY_BLEND_MS), 150.0);
		assert_eq!(o.set(b"trackers", b"recovery_blend_ms", b"5000"), Err("out of range"));
		assert_eq!(o.set(b"trackers", b"recovery_blend_ms", b"-"), Err("a class always has a value"));
		assert_eq!(o.set(b"LHR-9365E531", b"recovery_blend_ms", b"-"), Ok(()));
		assert!(o.device.is_empty());
		let mut back = Output::default();
		for l in o.text().split(|&c| c == b'\n') {
			back.parse_line(l);
		}
		assert_eq!(back, o);
		let l = o.lines();
		assert_eq!(l[0], b"O headset recovery_blend_ms 0\n");
		assert_eq!(l[SETTINGS.len()], b"O controllers recovery_blend_ms 60\n");
		assert_eq!(l[2 * SETTINGS.len() + PREDICTION_MS], b"O trackers prediction_ms 9\n");
		assert_eq!(l.last().unwrap(), b"O end\n");
	}

	/// every class has every setting, a device its own over its class's, and the headset is a class of its own
	#[test]
	fn a_device_has_its_own_values_over_its_classs() {
		let mut o = Output::default();
		o.set(b"headset", b"prediction_ms", b"4").unwrap();
		o.set(b"controllers", b"velocity_boost", b"30").unwrap();
		o.set(b"trackers", b"smoothing_ms", b"200").unwrap();
		o.set(b"trackers", b"movement_smoothing", b"40").unwrap();
		o.set(b"LHR-FOOT", b"offset_y_mm", b"-25").unwrap();
		o.set(b"LHR-FOOT", b"smoothing_ms", b"0").unwrap();
		let head = o.values(b"LHR-HMD", Kind::Headset);
		assert_eq!((head[PREDICTION_MS], head[SMOOTHING_MS], head[VELOCITY_BOOST]), (4.0, 0.0, 0.0));
		let hand = o.values(b"LHR-HAND", Kind::RightController);
		assert_eq!((hand[PREDICTION_MS], hand[VELOCITY_BOOST]), (9.0, 30.0));
		let foot = o.values(b"LHR-FOOT", Kind::Tracker);
		assert_eq!((foot[SMOOTHING_MS], foot[OFFSET_MM + 1], foot[PREDICTION_MS]), (0.0, -25.0, 9.0));
		assert_eq!(o.values(b"LHR-WAIST", Kind::Tracker)[SMOOTHING_MS], 200.0);
		let bad = [
			("prediction_ms", "41"),
			("velocity_boost", "-1"),
			("offset_rx_deg", "181"),
			("movement_smoothing", "101"),
		];
		for (name, bad) in bad {
			assert_eq!(o.set(b"trackers", name.as_bytes(), bad.as_bytes()), Err("out of range"), "{name}");
		}
		// what the override is handed: nothing to do at the defaults, the boost as a scale, the movement smoothing as
		// the share of its prediction SteamVR is left, the offset as a pose
		assert_eq!(Shown::of(&head), Shown::default());
		assert_eq!((Shown::of(&hand).velocity_scale, Shown::of(&hand).prediction), (1.3, 1.0));
		assert_eq!(Shown::of(&foot).prediction, 0.6);
		let off = Shown::of(&foot).offset.unwrap();
		assert_eq!((off.p.x, off.p.y, off.p.z), (0.0, -0.025, 0.0));
		assert_eq!((off.q.w, off.q.x, off.q.y, off.q.z), (1.0, 0.0, 0.0, 0.0));
		// turned about x, then y: the device's z axis ends up along its old x (90 deg about y after 0 about x)
		o.set(b"LHR-FOOT", b"offset_ry_deg", b"90").unwrap();
		let q = Shown::of(&o.values(b"LHR-FOOT", Kind::Tracker)).offset.unwrap().q;
		assert!((q.w - 0.5f64.sqrt()).abs() < 1e-12 && (q.y - 0.5f64.sqrt()).abs() < 1e-12 && q.x == 0.0 && q.z == 0.0);
	}

	/// A device through the smoothing: `n` reports 4 ms apart, its head at `at(t)` along x (m; its velocity the
	/// path's own), the corrections' sum `c(t)` along x, which is part of where the head is. Per report: where the
	/// tracker has the head, where it is shown, and the share of its motion that passes
	fn smoothed(
		ms: f64, n: usize, ret: (f64, f64), at: impl Fn(f64) -> f64, c: impl Fn(f64) -> f64,
	) -> Vec<(f64, f64, f64)> {
		smoothed_as(true, ms, n, ret, at, c)
	}

	/// ... `hold` false as a headset
	fn smoothed_as(
		hold: bool, ms: f64, n: usize, ret: (f64, f64), at: impl Fn(f64) -> f64, c: impl Fn(f64) -> f64,
	) -> Vec<(f64, f64, f64)> {
		let mut s = Smooth::default();
		(0..n)
			.map(|k| {
				let t = k as f64 * 0.004;
				let x = at(t) + c(t);
				let vel = [(at(t + 0.002) - at(t - 0.002)) / 0.004, 0.0, 0.0];
				let (mut p, mut r) = ([x, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0]);
				let own = Own { pos: &[x, 0.0, 1.0], rot: &[1.0, 0.0, 0.0, 0.0], vel: &vel, turn: &[0.0; 3] };
				let open = s.step(Some(&own), &corr([c(t), 0.0, 0.0], ret.0, ret.1), t, ms, 0.1, hold);
				s.apply(&mut p, &mut r);
				(x, p[0], open)
			})
			.collect()
	}

	/// ... and one that turns about z by `at(t)` (rad; its rate the path's own) where it is: the tracker's angle, the
	/// one shown, the share that passes
	fn turned(ms: f64, n: usize, at: impl Fn(f64) -> f64) -> Vec<(f64, f64, f64)> {
		let mut s = Smooth::default();
		(0..n)
			.map(|k| {
				let t = k as f64 * 0.004;
				let a = at(t);
				let rot = [(a / 2.0).cos(), 0.0, 0.0, (a / 2.0).sin()];
				let turn = [0.0, 0.0, (at(t + 0.002) - at(t - 0.002)) / 0.004];
				let (mut p, mut r) = ([0.0, 0.0, 1.0], rot);
				let own = Own { pos: &[0.0, 0.0, 1.0], rot: &rot, vel: &[0.0; 3], turn: &turn };
				let open = s.step(Some(&own), &corr([0.0; 3], -1.0, 0.0), t, ms, 0.1, true);
				s.apply(&mut p, &mut r);
				(a, 2.0 * r[3].atan2(r[0]), open)
			})
			.collect()
	}

	/// A device held still is shown calm: what it trembles by is not shown, and its rates are not handed on; a
	/// correction comes in as a glide, a return's as it is. With no smoothing all of it passes as it comes
	#[test]
	fn a_still_device_is_shown_calm() {
		let none = |_: f64| 0.0;
		let after = |from: f64, x: &[(f64, f64, f64)]| x[(from / 0.004) as usize..].to_vec();
		// a hand held still trembles 0.3 mm either way at 8 Hz (15 mm/s at its fastest): at 300 ms under a tenth of
		// that is shown, and next to nothing of its velocity is handed on
		let tremble = |t: f64| 0.0003 * (2.0 * std::f64::consts::PI * 8.0 * t).sin();
		let x = after(1.0, &smoothed(300.0, 750, (-1.0, 0.0), tremble, none));
		let swing = |i: usize| x.iter().map(|r| [r.0, r.1, r.2][i]).fold(0.0f64, |a, v| a.max(v.abs()));
		assert!(swing(0) > 0.00029 && swing(1) < 0.1 * swing(0) && swing(2) < 0.15, "{} {}", swing(1), swing(2));
		// off: as it comes, to the bit
		assert!(smoothed(0.0, 750, (-1.0, 0.0), tremble, none).iter().all(|r| r.0 == r.1 && r.2 == 1.0));
		// a station takes over 2 s in, the estimate steps 4 mm: not shown at once, 95 % of it after the 300 ms, all
		// of it in the end, never past it
		let step = |t: f64| if t < 2.0 { 0.0 } else { 0.004 };
		let x = after(2.0, &smoothed(300.0, 1000, (-1.0, 0.0), none, step));
		assert!(x[0].1 < 0.01 * 0.004 && x[75].1 > 0.95 * 0.004 - 1e-5 && x[75].1 < 0.004, "{} {}", x[0].1, x[75].1);
		assert!((x[499].1 - 0.004).abs() < 1e-6 && x.iter().all(|r| r.1 <= 0.004 + 1e-12));
		// inside a return's window the corrections are not the smoothing's
		assert_eq!(after(2.0, &smoothed(300.0, 1000, (0.1, 0.5), none, step))[0].1, 0.004);
		// a headset's own motion is shown whole, the tremble too (the picture turns with the head): only the
		// corrections glide
		assert!(smoothed_as(false, 300.0, 750, (-1.0, 0.0), tremble, none).iter().all(|r| r.0 == r.1 && r.2 == 1.0));
		let x = after(2.0, &smoothed_as(false, 300.0, 1000, (-1.0, 0.0), none, step));
		assert!(x[0].1 < 0.01 * 0.004 && x[75].1 > 0.95 * 0.004 - 1e-5 && x[0].2 == 1.0);
	}

	/// A device that moves is shown where the tracker has it: its motion passes whole, a correction is in within
	/// GLIDE_TRAVEL_M of travel, a move out of stillness leaves nothing behind, and a hand going back and forth is
	/// not held at either end
	#[test]
	fn a_moving_device_is_not_held_back() {
		let none = |_: f64| 0.0;
		// at 1 m/s: every report shown as it is, to the bit
		let x = smoothed(300.0, 500, (-1.0, 0.0), |t| t, none);
		assert!(x.iter().all(|r| r.0 == r.1 && r.2 == 1.0));
		// a 4 mm correction at 1 m/s: in after 5 cm of travel (50 ms), where a still device has shown a fifth of it
		let step = |t: f64| if t < 1.0 { 0.0 } else { 0.004 };
		let at = 250 + (GLIDE_TRAVEL_M / 1.0 / 0.004) as usize;
		let moving = smoothed(300.0, 500, (-1.0, 0.0), |t| t, step);
		let still = smoothed(300.0, 500, (-1.0, 0.0), none, step);
		assert!(moving[at].0 - moving[at].1 < 0.05 * 0.004 && still[at].1 < 0.2 * 0.004, "{}", still[at].1);
		// still for 2 s, then off at 2 m/s^2 up to 1 m/s: the hand is taken for moving 50 ms into the move, is never
		// shown more than 2 mm behind, and half a second in is where the tracker has it
		let go = |t: f64| {
			let m = (t - 2.0).max(0.0);
			if m < 0.5 {
				m * m
			} else {
				0.25 + (m - 0.5)
			}
		};
		let x = smoothed(300.0, 750, (-1.0, 0.0), go, none);
		assert!(x[499].2 < 0.01 && x[513].2 == 1.0, "{} {}", x[499].2, x[513].2);
		let behind = x.iter().map(|r| r.0 - r.1).fold(0.0f64, f64::max);
		assert!(behind > 0.0002 && behind <= STILL_REACH_M + 1e-12 && (x[625].0 - x[625].1).abs() < 1e-5, "{behind}");
		// a move that begins and ends gently, under the speeds that tell a move for most or all of its way (still for
		// 2 s, then 2 cm in a second, 5 cm in two, 20 cm in two; a turn of 5 degrees in a second, 10 and 45 in two):
		// never shown further behind than a tremble's reach, at the longest settle time as at 300 ms, where it would
		// be late by the settle time were its own motion held back for as long as it is slow
		let gently = |d: f64, s: f64| {
			move |t: f64| {
				let u = ((t - 2.0) / s).clamp(0.0, 1.0);
				d * u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
			}
		};
		let far = |x: &[(f64, f64, f64)]| x.iter().map(|r| (r.0 - r.1).abs()).fold(0.0f64, f64::max);
		for ms in [300.0, 1000.0] {
			for (d, s) in [(0.02, 1.0), (0.05, 2.0), (0.2, 2.0)] {
				let x = smoothed(ms, 1250, (-1.0, 0.0), gently(d, s), none);
				assert!(far(&x) <= STILL_REACH_M + 1e-12, "{ms} ms, {d} m in {s} s: {}", far(&x));
			}
			for (d, s) in [(5.0f64, 1.0), (10.0, 2.0), (45.0, 2.0)] {
				let x = turned(ms, 1250, gently(d.to_radians(), s));
				assert!(far(&x) <= STILL_REACH_RAD + 1e-12, "{ms} ms, {d} degrees in {s} s: {}", far(&x));
			}
		}
		// a hand going 5 cm either way at 2 Hz (0.6 m/s at its fastest) passes whole: it is not still at its ends
		let wave = |t: f64| 0.05 * (2.0 * std::f64::consts::PI * 2.0 * (t - 0.0625)).sin();
		let x = smoothed(300.0, 1000, (-1.0, 0.0), wave, none);
		assert!(x[250..].iter().all(|r| r.2 == 1.0 && (r.0 - r.1).abs() < 1e-6), "{:?}", x[300]);
	}

	/// A hand's path for the rail's tests: `n` reports 4 ms apart, the velocity `v(t)` integrated from the origin;
	/// each report's time, point, velocity, what the rail showed of them and whether a punch was on it
	#[allow(clippy::type_complexity)]
	fn hand(
		rail: &mut Rail, speed: f64, boost: f64, n: usize, v: impl Fn(f64) -> [f64; 3],
	) -> Vec<(f64, V3, V3, V3, V3, bool)> {
		let mut p = [0.0f64; 3];
		(0..n)
			.map(|k| {
				let t = k as f64 * 0.004;
				let vel = v(t);
				for i in 0..3 {
					p[i] += vel[i] * 0.004;
				}
				let (rp, rv) = (v3(&p), v3(&vel));
				let (sp, sv) = match rail.step(t, speed, boost, &p, &vel) {
					Some(r) => r.map(rp, rv),
					None => (rp, rv),
				};
				(t, rp, rv, sp, sv, rail.on())
			})
			.collect()
	}

	/// A punch along x that veers aside at its end: up to 5 m/s in 80 ms, held 60 ms (from 0.2 s also along y, up to
	/// 1 m/s in 40 ms), stopped in 40 ms, then pulled back at 3 m/s for 150 ms, then still
	fn punch(t: f64) -> [f64; 3] {
		let along = if t < 0.1 {
			0.0
		} else if t < 0.18 {
			5.0 * (t - 0.1) / 0.08
		} else if t < 0.24 {
			5.0
		} else if t < 0.28 {
			5.0 * (0.28 - t) / 0.04
		} else if t < 0.43 {
			-3.0
		} else {
			0.0
		};
		[along, if (0.2..0.28).contains(&t) { ((t - 0.2) / 0.04).min(1.0) } else { 0.0 }, 0.0]
	}

	/// a punch is latched where the hand gets fast, shown on a straight line and further and faster along it, let
	/// off as it stops and eased back without a step; the hand coming back gets no rail
	#[test]
	fn a_punch_rides_its_rail_and_is_eased_back() {
		let mut rail = Rail::default();
		let path = hand(&mut rail, 1.5, 100.0, 300, punch);
		let first = path.iter().position(|r| r.5).expect("no rail");
		let last = path.iter().rposition(|r| r.5).unwrap();
		// latched as the hand passes 1.5 m/s (0.124 s), off as it stops (under 0.75 m/s along the line: 0.274 s)
		assert!((path[first].0 - 0.124).abs() < 0.005 && (path[last].0 - 0.272).abs() < 0.006, "{first} {last}");
		// until then the hand is shown where it is, and at the latch it still is
		assert!(path[..first].iter().all(|r| r.1 == r.3 && r.2 == r.4));
		assert!(len(path[first].3 - path[first].1) < 1e-9);
		// on the rail: the hand veers 5 cm off its line and the shown hand stays on it; along it the speed is twice
		// the hand's once the rail is in
		let on = &path[first + 10..=last];
		assert!(on.iter().all(|r| r.3.y.abs() < 1e-9 && r.4.y.abs() < 1e-9), "{:?}", on.last());
		assert!(path[last].1.y > 0.045);
		assert!(on.iter().all(|r| (r.4.x / r.2.x - 2.0).abs() < 1e-9), "{:?}", on[0]);
		// further: at its end the shown hand is ahead of the hand by what it went along the rail
		let ahead = path[last].3.x - path[last].1.x;
		let along = path[last].1.x - path[first].1.x;
		assert!((ahead - along).abs() < 0.05 * along && along > 0.4, "{ahead} {along}");
		// never a step: what the shown hand is off the hand by moves by its own rate from report to report, the rail
		// coming in, on it, let off and eased back (to what 4 ms between this hand's reports leave: 1 mm as it stops
		// at 125 m/s^2; a rail that came in without its rate in the velocity was 5 mm off here)
		for w in path.windows(2) {
			let step = len((w[1].3 - w[1].1) - (w[0].3 - w[0].1) - ((w[0].4 - w[0].2) + (w[1].4 - w[1].2)) * 0.002);
			assert!(step < 0.0015, "a step of {step} m at {}", w[1].0);
		}
		// the hand coming back at 3 m/s is no punch; half a second on the shown hand is the hand again
		assert!(path[last + 1..].iter().all(|r| !r.5));
		let end = path.last().unwrap();
		assert!(end.1 == end.3 && end.2 == end.4, "{end:?}");
		// no rail without a speed, and the next punch the same way is taken again
		let mut off = Rail::default();
		assert!(hand(&mut off, 0.0, 100.0, 300, punch).iter().all(|r| r.1 == r.3 && !r.5));
		let again = hand(&mut rail, 1.5, 100.0, 300, punch);
		assert!(again.iter().any(|r| r.5));
	}

	/// a swing that curves (4 m/s round a 0.4 m circle, as a flailing arm's) is on its rail only until it has turned
	/// 45 degrees from it: the hand is let off before it is far from its line, and is not taken again while it is fast
	#[test]
	fn a_swing_that_curves_is_let_off_as_it_turns() {
		let mut rail = Rail::default();
		let swing = |t: f64| {
			if (0.1..0.5).contains(&t) {
				let (s, c) = sin_cos(10.0 * (t - 0.1));
				[4.0 * c, 4.0 * s, 0.0]
			} else {
				[0.0; 3]
			}
		};
		let path = hand(&mut rail, 1.5, 0.0, 200, swing);
		let first = path.iter().position(|r| r.5).expect("no rail");
		let last = path.iter().rposition(|r| r.5).unwrap();
		// taken at 0.1 s; its line the way of its first 40 ms (0.2 rad round), left 45 degrees on: a tenth of a second
		assert!((path[first].0 - 0.1).abs() < 0.005 && (path[last].0 - 0.196).abs() < 0.009, "{}", path[last].0);
		// 11 cm from its line as it is let off, and no further than 15 cm in the ease-back (which starts at the rate
		// the two were parting at)
		let off = path.iter().map(|r| len(r.3 - r.1)).fold(0.0, f64::max);
		assert!(len(path[last].3 - path[last].1) < 0.12 && off > 0.1 && off < 0.16, "{off}");
		assert!(path[last + 1..].iter().all(|r| !r.5));
		let end = path.last().unwrap();
		assert!(end.1 == end.3 && end.2 == end.4);
	}

	/// A toy is on only in the session it was switched on in: as SteamVR closes, and as it starts, every toy's switch
	/// is off again (a class's at 0, a device's own taken away) and the file says so; the toys' numbers and every
	/// other setting are kept as they were set
	#[test]
	fn the_toys_are_off_again_when_steamvr_closes() {
		let mut o = Output::default();
		for (scope, name, value) in [
			("controllers", "rail", "1"),
			("controllers", "rail_speed", "4.8"),
			("controllers", "rail_boost", "300"),
			("controllers", "velocity_boost", "25"),
			("trackers", "dash", "1"),
			("trackers", "smoothing_ms", "150"),
			("LHR-WAIST", "rail", "1"),
			("LHR-WAIST", "offset_y_mm", "-25"),
			("LHR-FOOT", "dash", "0"),
		] {
			o.set(scope.as_bytes(), name.as_bytes(), value.as_bytes()).unwrap();
		}
		let mut want = o.clone();
		let were_on = o.toys_off().expect("toys were on");
		assert_eq!(were_on, [b"controllers.rail".to_vec(), b"trackers.dash".to_vec(), b"LHR-WAIST.rail".to_vec()]);
		// only the switches moved: the classes' off, the devices' own gone (the foot's, which was off, with them)
		want.class[1][RAIL] = 0.0;
		want.class[2][DASH] = 0.0;
		want.device.get_mut(b"LHR-WAIST".as_slice()).unwrap()[RAIL] = None;
		want.device.remove(b"LHR-FOOT".as_slice());
		assert_eq!(o, want);
		let hand = o.values(b"LHR-HAND", Kind::LeftController);
		assert_eq!((hand[RAIL], hand[RAIL_SPEED], hand[RAIL_BOOST], hand[VELOCITY_BOOST]), (0.0, 4.8, 300.0, 25.0));
		let waist = o.values(b"LHR-WAIST", Kind::Tracker);
		assert_eq!((waist[RAIL], waist[DASH], waist[SMOOTHING_MS], waist[OFFSET_MM + 1]), (0.0, 0.0, 150.0, -25.0));
		// nothing on: nothing to do, and so no file written
		assert_eq!(o.toys_off(), None);
		assert_eq!(Output::default().toys_off(), None);
		// every switch is one of them: a setting that is 0 or 1 and nothing else
		for (i, s) in SETTINGS.iter().enumerate() {
			assert_eq!(TOY_SWITCHES.contains(&i), (s.min, s.max) == (0.0, 1.0), "{}", s.name);
		}

		// the file: read at SteamVR's start with a toy on (it did not close in order), it is off and the file says so;
		// switched on in the session and SteamVR closed, off again
		let _turn = STORE_TEST.lock().unwrap_or_else(|e| e.into_inner());
		let dir = std::env::temp_dir().join(format!("sally-toys-off-test-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let file = dir.join("sally_output.txt");
		std::fs::write(
			&file,
			"controllers.rail=1\ncontrollers.rail_speed=4.8\nLHR-WAIST.rail=1\ntrackers.smoothing_ms=150\n\
			 trackers.jelly=1\ntrackers.hip_gain_percent=300\n",
		)
		.unwrap();
		let before = g_output();
		read_output(dir.to_string_lossy().as_bytes());
		let o = g_output();
		assert_eq!((o.class[1][RAIL], o.class[1][RAIL_SPEED], o.class[2][SMOOTHING_MS]), (0.0, 4.8, 150.0));
		assert!(o.device.is_empty());
		let text = std::fs::read_to_string(&file).unwrap();
		assert!(text.contains("controllers.rail=0\n") && text.contains("controllers.rail_speed=4.8\n"), "{text}");
		assert!(text.contains("trackers.smoothing_ms=150\n") && !text.contains("LHR-WAIST"), "{text}");
		// a line of a toy this driver does not have (a file from before 0.15) is passed over, and not written again
		assert!(!text.contains("jelly") && !text.contains("hip_gain"), "{text}");
		// a setting set from the panel is in the file once nothing more has been set for a moment (a slider being
		// dragged sets one at every frame), not at each
		command(&[b"trackers", b"dash", b"1"]);
		save_due(Some(now_seconds()));
		assert!(!std::fs::read_to_string(&file).unwrap().contains("trackers.dash=1\n"));
		save_due(Some(now_seconds() + SAVE_AFTER_S));
		assert!(std::fs::read_to_string(&file).unwrap().contains("trackers.dash=1\n"));
		toys_off("SteamVR closes");
		assert_eq!(g_output().class[2][DASH], 0.0);
		let text = std::fs::read_to_string(&file).unwrap();
		assert!(text.contains("trackers.dash=0\n") && text.contains("trackers.smoothing_ms=150\n"), "{text}");
		set_g_output(before);
		store().path = None;
		let _ = std::fs::remove_dir_all(&dir);
	}

	/// a sally_output.txt from before the headset had a class: its recovery blend was the trackers'
	#[test]
	fn the_headset_takes_the_trackers_blend_from_an_older_file() {
		let _turn = STORE_TEST.lock().unwrap_or_else(|e| e.into_inner());
		let dir = std::env::temp_dir().join(format!("sally-output-test-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let file = dir.join("sally_output.txt");
		std::fs::write(&file, "trackers.recovery_blend_ms=0\ncontrollers.recovery_blend_ms=20\n").unwrap();
		let before = g_output();
		read_output(dir.to_string_lossy().as_bytes());
		let o = g_output();
		assert_eq!(o.class[0][RECOVERY_BLEND_MS], 0.0);
		assert_eq!(o.class[1][RECOVERY_BLEND_MS], 20.0);
		std::fs::write(&file, "trackers.recovery_blend_ms=0\nheadset.recovery_blend_ms=80\n").unwrap();
		read_output(dir.to_string_lossy().as_bytes());
		assert_eq!(g_output().class[0][RECOVERY_BLEND_MS], 80.0);
		set_g_output(before);
		store().path = None;
		let _ = std::fs::remove_dir_all(&dir);
	}
}
