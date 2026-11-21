//! The Output tab: what SteamVR is shown of sally's tracking, the driver's output layer (docs/sally_driver.md, "The
//! output layer"). Preference, not tracking: the room view and the recordings keep the tracker's own.
//!
//! A box for each device class, headset, controllers and trackers, side by side so that a setting reads across them:
//! the two smoothings, recovery blend, prediction and velocity boost, each a slider with a stepper at either end and
//! a tick at its default. The two smoothings are different things, and each says beside its name when it acts and
//! what it costs: the still smoothing only while a device is held still, with no delay; the movement smoothing
//! while it moves too, with a delay. The buttons under a box's title pick whose values it shows: the class's, or one
//! of its devices' own over them, with where that device is shown against where it is (its offset, in its own axes).
//! Under the controllers' five are the punch rail and the dash punch that goes by it, each a tick box with its two
//! numbers. A row is ruled off from the next, and its value is a figure at its right end.
//!
//! The panel asks the driver for the values every 2 s while the tab is open and sends a value when it is let go; the
//! driver keeps them in its sally_output.txt and applies them from its next report.

use egui::{Align2, Color32, Id, Pos2, Rect, Vec2};

use super::kit::{self, Look, Range, Surface, CTL_H, GAP, HEADER_H, PAD};
use super::theme::*;
use super::Panel;
use crate::runtime::Device;

/// The classes, as the driver's scopes name them: the scope, the box's title, its icon
const CLASSES: [(&str, &str); 3] = [("headset", "Headset"), ("controllers", "Controllers"), ("trackers", "Trackers")];

/// One setting: the driver's name for it, the label, a few words that stay beside the label where two settings could
/// be taken for one another (empty: none), what it does (shown while the pointer is on its row), the same two for
/// the headset's box where the setting is another thing there (empty: as for the rest), the slider's range (the
/// driver's own can be wider: a value in its file past it is shown at the rail's end), the step its values are sent
/// in, the unit, and each class's default: the driver's, which are the tracker's own output (no glide, nothing
/// added; the prediction at the pose's own age), so Defaults puts a class back to what the tracking is tuned on and
/// a setting is on only once it has been set
struct Row {
	name: &'static str,
	label: &'static str,
	note: &'static str,
	hint: &'static str,
	headset: (&'static str, &'static str),
	min: f64,
	max: f64,
	step: f64,
	unit: &'static str,
	defaults: [f64; 3],
}

/// The two smoothings come first and together: the still smoothing (the driver's `smoothing_ms`) and the movement
/// smoothing (its `movement_smoothing`, what a smoothing add-on does)
const ROWS: [Row; 5] = [
	Row {
		name: "smoothing_ms",
		label: "Still smoothing",
		note: "only while held still, no delay",
		hint: "Only while a device is held still: its trembling and the light's corrections are let in over this \
		       long, so it rests calm. Once it moves, slowly too, it is shown as it is tracked: no delay. At 0 the \
		       tracker's own pose.",
		headset: (
			"eases the light's corrections in, no delay",
			"On the headset only the light's corrections are let in over this long, sooner the faster it moves: the \
			 head's own motion is never held back, so no delay. At 0 the tracker's own pose.",
		),
		min: 0.0,
		max: 1000.0,
		step: 5.0,
		unit: "ms",
		defaults: [0.0, 0.0, 0.0],
	},
	Row {
		name: "movement_smoothing",
		label: "Movement smoothing",
		note: "while moving too, adds delay",
		hint: "While it moves too, as a smoothing add-on does: SteamVR's look-ahead, which makes a device's shake several \
		       times larger, is cut by this much. The device trails its movement for it (about 40 ms at 100), and \
		       games read its speed lower.",
		headset: (
			"the view trails your head: best at 0",
			"SteamVR's look-ahead cut by this much. On the headset the whole view then trails the head as it turns, \
			 which is felt at once: best left at 0, a shot for a film aside.",
		),
		min: 0.0,
		max: 100.0,
		step: 5.0,
		unit: "%",
		defaults: [0.0, 0.0, 0.0],
	},
	Row {
		name: "recovery_blend_ms",
		label: "Recovery blend",
		note: "",
		hint: "Light back after a dark spell: the jump is shown as a glide that settles in this long. At 0 the jump \
		       comes as it is.",
		headset: ("", ""),
		min: 0.0,
		max: 1000.0,
		step: 5.0,
		unit: "ms",
		defaults: [0.0, 0.0, 0.0],
	},
	Row {
		name: "prediction_ms",
		label: "Prediction",
		note: "",
		hint:
			"The age the pose is declared: how far ahead of the tracker's state SteamVR shows the device. Its default \
		       is the pose's own age, nothing added; more leads, less trails.",
		headset: ("", ""),
		min: 0.0,
		max: 40.0,
		step: 0.5,
		unit: "ms",
		defaults: [0.0, 9.0, 9.0],
	},
	Row {
		name: "velocity_boost",
		label: "Velocity boost",
		note: "",
		hint: "The velocity SteamVR is given, this much over the device's own, for games that score on a hand's \
		       speed; SteamVR's own prediction reaches further with it.",
		headset: ("", ""),
		min: 0.0,
		max: 200.0,
		step: 5.0,
		unit: "%",
		defaults: [0.0, 0.0, 0.0],
	},
];

type Offset = (&'static str, &'static str, [&'static str; 3], f64, &'static str);

/// A device's offset, where it is shown against where it is, in its own axes: the label, what it does, the three
/// settings, how far the slider goes either way (the driver takes 500 mm), the unit
const OFFSETS: [Offset; 2] = [
	(
		"Shown moved",
		"Where the device is shown against where it is: moved along its own x, y and z.",
		["offset_x_mm", "offset_y_mm", "offset_z_mm"],
		200.0,
		"mm",
	),
	(
		"Shown turned",
		"Where the device is shown against where it is: turned about its own x, then y, then z.",
		["offset_rx_deg", "offset_ry_deg", "offset_rz_deg"],
		180.0,
		"°",
	),
];

/// A setting's row: its label's line, the slider under it. Five of them, the Defaults button and the two punch
/// settings with their two each fit the controllers' box at the layout's height
const ROW_H: f32 = 58.0;
/// Where the slider's rail starts under the row's top
const ROW_RAIL: f32 = 26.0;

/// A punch setting of the controllers': its switch (the driver's name for it), its title and icon, what it does, and
/// its numbers
struct Punch {
	key: &'static str,
	title: &'static str,
	says: &'static str,
	rows: [Row; 2],
}

/// The punch rail and the dash punch that goes by it (sally_driver.md "The punch rail", "The body's toys"): off
/// until they are switched on, and off again when SteamVR closes
const PUNCHES: [Punch; 2] = [
	Punch {
		key: "rail",
		title: "Punch rail",
		says: "Punch rail: a hand that gets fast is shown on a straight line from there, further and faster along it.",
		rows: [
			Row {
				name: "rail_speed",
				label: "Taken from",
				note: "",
				hint: "The punch rail: a hand that gets this fast is put on a straight line from where it is, along \
				       the way it is going.",
				headset: ("", ""),
				min: 0.5,
				max: 6.0,
				step: 0.1,
				unit: "m/s",
				defaults: [1.5, 1.5, 1.5],
			},
			Row {
				name: "rail_boost",
				label: "Further and faster",
				note: "",
				hint: "The punch rail: how much further along its line the hand is shown than it went, and that much \
				       faster.",
				headset: ("", ""),
				min: 0.0,
				max: 500.0,
				step: 5.0,
				unit: "%",
				defaults: [50.0, 50.0, 50.0],
			},
		],
	},
	Punch {
		key: "dash",
		title: "Dash punch",
		says: "Dash punch: a punch on its rail takes your whole body with it, so the fist gets there with the arm as \
		       you hold it. Then you are eased back.",
		rows: [
			Row {
				name: "dash_percent",
				label: "Body goes along",
				note: "",
				hint: "Dash punch: how much of what the rail shows the fist ahead by is the whole body's to slide by. \
				       At 100 % the fist is where the rail puts it and the arm is as you hold it.",
				headset: ("", ""),
				min: 0.0,
				max: 100.0,
				step: 5.0,
				unit: "%",
				defaults: [100.0, 100.0, 100.0],
			},
			Row {
				name: "dash_back_ms",
				label: "Back in",
				note: "",
				hint: "Dash punch: how long the body takes to glide back to where you are once the punch is over \
				       (95 % of the way).",
				headset: ("", ""),
				min: 100.0,
				max: 2000.0,
				step: 50.0,
				unit: "ms",
				defaults: [400.0, 400.0, 400.0],
			},
		],
	},
];

/// Whether a setting is one of a device's own on this tab (the five of every class, a device's offsets)
fn of_this_tab(name: &str) -> bool {
	ROWS.iter().any(|row| row.name == name) || OFFSETS.iter().any(|o| o.2.contains(&name))
}

/// The class a device's output settings come from, by its type (an index into CLASSES)
pub(super) fn class_of(subtype: &str) -> usize {
	match subtype {
		"index_hmd" | "vive_hmd" => 0,
		"knuckles_l" | "knuckles_r" | "wand" => 1,
		_ => 2,
	}
}

/// A value as its slider says it
pub(super) fn shown(v: f64, step: f64) -> String {
	if step < 1.0 {
		format!("{v:.1}")
	} else {
		format!("{v:.0}")
	}
}

impl Panel {
	/// The Output tab opened on a device's own values (from its box in the Tracking tab)
	pub(super) fn output_show(&mut self, d: &Device) {
		if d.serial != "-" {
			self.output_scope[class_of(&d.subtype)] = Some(d.serial.clone());
		}
		self.st.tab = 2;
		self.st.dirty = true;
	}

	/// `scope`'s value of a setting (a class's, or a device's own with its class's, `class`, until it has one), and
	/// whether it is the scope's own
	fn out_value(&self, scope: &str, class: Option<&str>, name: &str) -> (Option<f64>, bool) {
		let of = |s: &str| self.vc.output.get(&(s.to_string(), name.to_string())).copied();
		match of(scope) {
			Some(v) => (Some(v), true),
			None => (class.and_then(of), false),
		}
	}

	/// The value a slider shows: the one being dragged, else the driver's
	fn out_live(&self, scope: &str, name: &str, have: Option<f64>) -> f64 {
		match &self.output_drag {
			Some((s, n, v)) if s == scope && *n == name => *v,
			_ => have.unwrap_or(0.0),
		}
	}

	/// What a slider or stepper did to `scope`'s setting: kept while it is dragged, sent when it is let go
	fn out_moved(&mut self, scope: &str, name: &'static str, v: f64, slid: kit::Slid) {
		if slid.changed {
			self.output_drag = Some((scope.to_string(), name, v));
		}
		if slid.released {
			if let Some((s, n, v)) = self.output_drag.take() {
				self.vc.output_set(&s, n, v);
			}
		}
	}

	/// One setting's row in a class's box: its label, the value at the right, the slider under them, a line under it
	#[allow(clippy::too_many_arguments)]
	fn out_row(
		&mut self, ui: &mut egui::Ui, r: Rect, ci: usize, scope: &str, class: Option<&str>, row: &'static Row,
		known: bool,
	) {
		let t = tk();
		let p = ui.painter().clone();
		let (have, own) = self.out_value(scope, class, row.name);
		let on = known && have.is_some();
		let mut v = self.out_live(scope, row.name, have);
		let cy = r.min.y + 13.0;
		let lr = kit::text(
			&p,
			Pos2::new(r.min.x + 2.0, cy),
			Align2::LEFT_CENTER,
			row.label,
			body(),
			if on { t.text } else { t.faint },
		);
		let (note, hint) = match row.headset {
			(note, hint) if ci == 0 && !hint.is_empty() => (note, hint),
			_ => (row.note, row.hint),
		};
		// the value; for a device, whose it is (its own can be taken away again)
		let mut right = r.max.x;
		if let Some(cls) = class {
			// (Unset is there, off, while the value is the class's: the value keeps its place when it becomes its own)
			let bw = kit::button_w(&p, "Unset", false);
			let br = Rect::from_min_size(Pos2::new(right - bw, cy - 12.0), Vec2::new(bw, 24.0));
			let resp =
				kit::button(ui, Id::unique(("out-own", scope, row.name)), br, "Unset", None, Look::Plain, known && own)
					.on_hover_text(format!("Its own value: back to the {cls}'"));
			if resp.clicked() {
				self.vc.output_clear(scope, row.name);
				self.output_drag = None;
			}
			right = br.min.x - 6.0;
		}
		if on {
			let vs = shown(v, row.step);
			let font = figure();
			let uw = kit::text_w(&p, row.unit, medium(12.5)) + 4.0;
			let vw = kit::text_w(&p, &vs, font.clone());
			let col = if class.is_some() && !own { t.muted } else { t.text };
			kit::value_unit(&p, Pos2::new(right - uw - vw, cy + 10.0), &vs, row.unit, font, col);
			right -= uw + vw;
			if let (Some(cls), false) = (class, own) {
				let s = format!("the {cls}'");
				right = kit::text(&p, Pos2::new(right - 10.0, cy + 1.0), Align2::RIGHT_CENTER, &s, caption(), t.faint)
					.min
					.x;
			}
		}
		// the few words that tell this setting from the one it could be taken for, in what the value leaves free
		if !note.is_empty() {
			let nr = Rect::from_min_max(Pos2::new(lr.max.x + 10.0, cy - 9.0), Pos2::new(right - 12.0, cy + 11.0));
			if nr.width() > 40.0 {
				kit::text_in(&p, nr, note, label(), if on { t.soft } else { t.faint });
			}
		}
		let sr =
			Rect::from_min_max(Pos2::new(r.min.x, r.min.y + ROW_RAIL), Pos2::new(r.max.x, r.min.y + ROW_RAIL + CTL_H));
		let inherits = match class {
			Some(cls) if own => self.out_value(cls, None, row.name).0,
			_ => None,
		};
		let range = Range { min: row.min, max: row.max, step: row.step, default: Some(row.defaults[ci]), inherits };
		let slid = kit::slider(ui, Id::unique(("out", scope, row.name)), sr, &mut v, &range, on);
		self.out_moved(scope, row.name, v, slid);
		p.line_segment(
			[Pos2::new(r.min.x, r.max.y + 1.0), Pos2::new(r.max.x, r.max.y + 1.0)],
			egui::Stroke::new(1.0, t.line),
		);
		if ui.rect_contains_pointer(r) {
			self.output_hint[self.view.index()] = hint;
		}
	}

	/// One of a device's offsets on a line: its axis, a slider with its steppers, the value
	#[allow(clippy::too_many_arguments)]
	fn out_offset(
		&mut self, ui: &mut egui::Ui, r: Rect, scope: &str, name: &'static str, axis: (&str, Color32), limit: f64,
		unit: &str, known: bool,
	) {
		let t = tk();
		let p = ui.painter().clone();
		let (have, own) = self.out_value(scope, None, name);
		let mut v = self.out_live(scope, name, have);
		let cy = r.center().y;
		p.text(
			Pos2::new(r.min.x + 7.0, cy),
			Align2::CENTER_CENTER,
			axis.0,
			semibold(14.0),
			if known { axis.1 } else { t.faint },
		);
		let vs = format!("{v:.0}");
		let col = if own && v != 0.0 { t.text } else { t.muted };
		let ur = kit::text(&p, Pos2::new(r.max.x, cy + 1.0), Align2::RIGHT_CENTER, unit, caption(), t.muted);
		kit::text(&p, Pos2::new(ur.min.x - 4.0, cy), Align2::RIGHT_CENTER, &vs, num(), col);
		let sr = Rect::from_min_max(Pos2::new(r.min.x + 20.0, r.min.y), Pos2::new(r.max.x - 62.0, r.max.y));
		let range = Range { min: -limit, max: limit, step: 1.0, default: Some(0.0), inherits: None };
		let slid = kit::slider(ui, Id::unique(("out-off", scope, name)), sr, &mut v, &range, known);
		self.out_moved(scope, name, v, slid);
	}

	pub(super) fn output_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		if self.output_drag.is_none() {
			self.vc.output_ask(self.now);
		}
		let known = self.vc.output_known(self.now);
		self.output_hint[self.view.index()] = "";
		let foot_h = 34.0;
		let cw = (r.width() - 2.0 * GAP) / 3.0;
		for ci in 0..3 {
			let cr = Rect::from_min_size(
				Pos2::new(r.min.x + ci as f32 * (cw + GAP), r.min.y),
				Vec2::new(cw, r.height() - foot_h - GAP),
			);
			self.class_box(ui, cr, ci, known);
		}

		// the foot line: what the row under the pointer does, else what the driver said
		let fr = Rect::from_min_max(Pos2::new(r.min.x, r.max.y - foot_h), r.max);
		let p = ui.painter().clone();
		self.surface(&p, fr, Surface::Card);
		let old = self.vc.output_old_driver(self.now);
		let hint = self.output_hint[self.view.index()];
		// (yellow where the driver has something to say; what a row does is said in ink)
		let (text, col) = if !self.vc.output_error.is_empty() {
			(format!("The driver: {}", self.vc.output_error), t.warn)
		} else if !known && old {
			("The sally driver is one from before 0.8.8: it has no output layer.".to_string(), t.warn)
		} else if !known {
			("The sally driver is not answering: these are set while SteamVR runs.".to_string(), t.warn)
		} else if !hint.is_empty() {
			(hint.to_string(), t.soft)
		} else {
			(
				"What SteamVR is shown, after the tracking: the room and the recordings keep the tracker's own. Nothing \
				 here is on until it is set; the tick under a rail is its default, the tracker's own output."
					.to_string(),
				t.muted,
			)
		};
		let clip = Rect::from_min_max(Pos2::new(fr.min.x + PAD, fr.min.y), Pos2::new(fr.max.x - PAD, fr.max.y));
		kit::text_in(&p, clip, &text, label(), col);
	}

	/// A class's box
	fn class_box(&mut self, ui: &mut egui::Ui, r: Rect, ci: usize, known: bool) {
		let t = tk();
		let (class, title) = CLASSES[ci];
		let p = ui.painter().clone();
		self.card(ui, r, title);
		let x0 = r.min.x + PAD;
		let xr = r.max.x - PAD;
		let hy = r.min.y + HEADER_H * 0.5;
		// whose values: the class's, or one of its devices' own
		let devices: Vec<(String, String, bool)> = self
			.active_devices()
			.iter()
			.filter(|d| d.serial != "-" && class_of(&d.subtype) == ci)
			.map(|d| {
				let own = self.vc.output.keys().any(|(s, name)| *s == d.serial && of_this_tab(name));
				(d.serial.clone(), self.short_name(d), own)
			})
			.collect();
		if self.output_scope[ci].as_ref().is_some_and(|s| !devices.iter().any(|d| d.0 == *s)) {
			self.output_scope[ci] = None;
		}
		let n = devices.len();
		let count = if n == 0 { "none tracking".to_string() } else { format!("{n} tracking") };
		kit::text(&p, Pos2::new(xr, hy), Align2::RIGHT_CENTER, &count, label(), t.muted);
		let mut y = r.min.y + HEADER_H + 10.0;
		if n > 0 {
			let all = format!("All {}", if ci == 0 { "headsets" } else { class });
			let mut chips: Vec<(&str, bool, Option<Color32>)> =
				vec![(all.as_str(), self.output_scope[ci].is_none(), None)];
			for (serial, name, own) in &devices {
				// a mark on a device with values of its own: something set
				chips.push((name.as_str(), self.output_scope[ci].as_deref() == Some(serial), own.then_some(t.accent)));
			}
			let (clicked, end) = kit::chips(ui, Id::unique(("out-scope", ci)), Pos2::new(x0, y), xr, 28.0, &chips);
			match clicked {
				Some(0) => self.output_scope[ci] = None,
				Some(i) => self.output_scope[ci] = Some(devices[i - 1].0.clone()),
				None => {}
			}
			y = end.y + 12.0;
		}
		let device = self.output_scope[ci].clone();
		let (scope, of_class) = match &device {
			Some(serial) => (serial.clone(), Some(class)),
			None => (class.to_string(), None),
		};

		// the five settings
		for row in &ROWS {
			let rr = Rect::from_min_max(Pos2::new(x0, y), Pos2::new(xr, y + ROW_H - 4.0));
			self.out_row(ui, rr, ci, &scope, of_class, row, known);
			y += ROW_H;
		}

		if let Some(serial) = &device {
			// the device's offset, its own alone
			p.line_segment([Pos2::new(x0, y), Pos2::new(xr, y)], egui::Stroke::new(1.0, t.line));
			y += 8.0;
			for (what, says, names, limit, unit) in &OFFSETS {
				let top = y;
				kit::text(&p, Pos2::new(x0 + 2.0, y + 13.0), Align2::LEFT_CENTER, what, body(), t.text);
				y += 28.0;
				for (i, name) in names.iter().enumerate() {
					let rr = Rect::from_min_max(Pos2::new(x0, y), Pos2::new(xr, y + 30.0));
					let axis = (["x", "y", "z"][i], [AXIS_X, AXIS_Y, AXIS_Z][i]);
					self.out_offset(ui, rr, &scope, name, axis, *limit, unit, known);
					y += 32.0;
				}
				if ui.rect_contains_pointer(Rect::from_min_max(Pos2::new(x0, top), Pos2::new(xr, y))) {
					self.output_hint[self.view.index()] = says;
				}
				y += 6.0;
			}
			// its own values taken away again: it follows its class
			let own: Vec<String> = self
				.vc
				.output
				.keys()
				.filter(|(sc, name)| sc == serial && of_this_tab(name))
				.map(|(_, name)| name.clone())
				.collect();
			if known && !own.is_empty() && y + 30.0 <= r.max.y - PAD + 2.0 {
				let says = "Follow the class again";
				let bw = kit::button_w(&p, says, false);
				let br = Rect::from_min_size(Pos2::new(xr - bw, y + 2.0), Vec2::new(bw, 28.0));
				let resp = kit::button(ui, Id::unique(("out-follow", ci)), br, says, None, Look::Plain, true)
					.on_hover_text(format!("Takes this device's own values away: it follows the {class}' again"));
				if resp.clicked() {
					for name in own {
						self.vc.output_clear(serial, &name);
					}
					self.output_drag = None;
				}
			}
			return;
		}

		// reset: the class's five back to their defaults, where one is off its default
		let off: Vec<(&'static str, f64)> = ROWS
			.iter()
			.filter(|row| self.out_value(class, None, row.name).0.is_some_and(|v| (v - row.defaults[ci]).abs() > 1e-9))
			.map(|row| (row.name, row.defaults[ci]))
			.collect();
		if known && !off.is_empty() {
			let bw = kit::button_w(&p, "Defaults", false);
			let br = Rect::from_min_size(Pos2::new(xr - bw, y + 4.0), Vec2::new(bw, 26.0));
			let resp = kit::button(ui, Id::unique(("out-defaults", ci)), br, "Defaults", None, Look::Plain, true)
				.on_hover_text(format!("The {class}' five settings back to the tracker's own output"));
			if resp.clicked() {
				for (name, v) in off {
					self.vc.output_set(class, name, v);
				}
			}
		}
		y += 36.0;
		if ci == 1 {
			self.punch_block(ui, Rect::from_min_max(Pos2::new(x0, y), Pos2::new(xr, r.max.y - PAD)), known);
		}
	}

	/// The controllers' punch settings under their five: each a tick box with its two numbers. The dash goes by the
	/// rail, so its box puts the rail on and off with it, and the rail's off is the dash's too; the rail's own
	/// box is the rail alone. A driver from before one of them has no box for it
	fn punch_block(&mut self, ui: &mut egui::Ui, r: Rect, known: bool) {
		const CLASS: &str = "controllers";
		let t = tk();
		let p = ui.painter().clone();
		let value = |panel: &Panel, name: &str| panel.vc.output.get(&(CLASS.to_string(), name.to_string())).copied();
		let mut y = r.min.y;
		for punch in &PUNCHES {
			if y + 40.0 + 2.0 * ROW_H > r.max.y + 2.0 {
				break;
			}
			p.line_segment([Pos2::new(r.min.x, y), Pos2::new(r.max.x, y)], egui::Stroke::new(1.0, t.card_edge));
			let has = known && value(self, punch.key).is_some();
			let on = has && value(self, punch.key).is_some_and(|v| v > 0.0);
			let head = Rect::from_min_max(Pos2::new(r.min.x, y), Pos2::new(r.max.x, y + 40.0));
			let cy = y + 22.0;
			let col = if has { t.text } else { t.faint };
			kit::text(&p, Pos2::new(r.min.x + 2.0, cy), Align2::LEFT_CENTER, punch.title, name(), col);
			let at = Pos2::new(r.max.x - kit::TICK - 3.0, cy - kit::TICK * 0.5);
			if kit::tick(ui, Id::unique(("punch-switch", punch.key)), at, on, has) {
				self.vc.output_set(CLASS, punch.key, if on { 0.0 } else { 1.0 });
				// the dash goes by the rail: its switch is the rail's too, and the rail's off is the dash's
				let other = |panel: &Panel, name: &str| value(panel, name).is_some_and(|v| v > 0.0);
				match (punch.key, on) {
					("dash", false) if !other(self, "rail") => self.vc.output_set(CLASS, "rail", 1.0),
					("dash", true) if other(self, "rail") => self.vc.output_set(CLASS, "rail", 0.0),
					("rail", true) if other(self, "dash") => self.vc.output_set(CLASS, "dash", 0.0),
					_ => {}
				}
			}
			if ui.rect_contains_pointer(head) {
				self.output_hint[self.view.index()] = punch.says;
			}
			y += 44.0;
			for row in &punch.rows {
				let rr = Rect::from_min_max(Pos2::new(r.min.x, y), Pos2::new(r.max.x, y + ROW_H - 6.0));
				self.out_row(ui, rr, 1, CLASS, None, row, has);
				y += ROW_H;
			}
			y += 6.0;
		}
	}
}

#[cfg(test)]
mod tests {
	/// the ticks under the rails and what Defaults sets are the driver's defaults (the fake driver's list is held to the
	/// driver's own in tests/panel.rs): the tracker's own output, with no glide and nothing added
	/// the punch settings are the driver's: each switch off by default, the numbers' defaults its own for the
	/// controllers and on their sliders, the sliders inside its ranges
	#[test]
	fn the_punch_settings_are_the_drivers() {
		use crate::valve::fake_driver::SETTINGS;
		let known = |name: &str| SETTINGS.iter().find(|s| s.0 == name).unwrap_or_else(|| panic!("no setting {name}"));
		for punch in &super::PUNCHES {
			let s = known(punch.key);
			assert_eq!((s.1, s.2, s.3), ([0.0; 3], 0.0, 1.0), "{}", punch.key);
			for row in &punch.rows {
				let s = known(row.name);
				assert!(s.2 <= row.min && row.max <= s.3, "{}", row.name);
				assert_eq!(row.defaults, s.1, "{}", row.name);
				let steps = (row.defaults[1] - row.min) / row.step;
				assert!((steps - steps.round()).abs() < 1e-9, "{}", row.name);
			}
		}
	}

	#[test]
	fn the_rows_defaults_are_the_drivers() {
		use crate::valve::fake_driver::SETTINGS;
		for row in &super::ROWS {
			let s = SETTINGS.iter().find(|s| s.0 == row.name).unwrap_or_else(|| panic!("no setting {}", row.name));
			assert_eq!(row.defaults, s.1, "{}", row.name);
			assert!(s.2 <= row.min && row.max <= s.3, "{}", row.name);
			if row.name != "prediction_ms" {
				assert_eq!(row.defaults, [0.0; 3], "{}: on by default", row.name);
			}
		}
	}

	#[test]
	fn a_devices_class_by_its_type() {
		assert_eq!(super::class_of("index_hmd"), 0);
		assert_eq!(super::class_of("knuckles_l"), 1);
		assert_eq!(super::class_of("tracker_gen2"), 2);
	}

	#[test]
	fn a_value_is_shown_as_its_step_has_it() {
		assert_eq!(super::shown(9.0, 0.5), "9.0");
		assert_eq!(super::shown(150.0, 5.0), "150");
		assert_eq!(super::shown(4.8, 0.1), "4.8");
	}
}
