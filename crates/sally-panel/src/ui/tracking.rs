//! The Tracking tab: the room, the base stations and the events over one table of the devices.
//!
//! The table is where a device's numbers are, each once: a row a device, a column a reading, the figures set from the
//! right under their headings so that one column reads down the devices. What it is (SteamVR's own symbol of it,
//! symbols.rs), its state as the runtime says it, how many of its sensors each station lights, how sure its position
//! is and how well the light fits it, its radio, its battery, how far it is from Valve's pose of it, its light from
//! behind and its losses, its last minute as a strip, and what the runtime's diagnosis says of it. A row is ink but
//! for what is a status: its state (green tracking; ink through a brief gap in its light, yellow once the gap has
//! held 2 s; never red: a device without light is not broken), its radio's loss by the runtime's own
//! grades, a low battery, and the strip. Its symbol has its kind's hue. The buttons over the table act on the
//! selected row.
//!
//! Nothing moves when something changes: a mark has its room whether it is there or not, so the words after it stay
//! where they were; a figure is set from the right in its column; and the boxes keep their size when a device or a
//! station comes and goes (each table keeps the room of the most rows it has had).

use egui::{Align2, Id, Painter, Pos2, Rect, ScrollArea, Sense, Stroke, UiBuilder, Vec2};

use super::kit::{self, Col, Look, Table, BAND_H, GAP, HEADER_H, PAD, TH_H, TR_H};
use super::theme::*;
use super::{Health, Panel, Rename, Sel};
use crate::history::Light;
use crate::runtime::{Device, EvLevel};

/// How far back a row's strip goes (s)
const STRIP_S: f64 = 60.0;
/// The stations' box: the shown station's place under the rows (three lines)
const PLACE_H: f32 = 3.0 * 24.0 + 6.0;

/// A column of the devices' table
#[derive(Clone, Copy, Debug, PartialEq)]
enum Reading {
	Symbol,
	Name,
	State,
	/// a station (its index): how many of the device's sensors it lights
	Station(i32),
	Lit,
	Sure,
	Residual,
	Loss,
	Gap,
	Battery,
	Valve,
	Valve90,
	Behind,
	Losses,
	Strip,
	Note,
}

impl Reading {
	/// What a column's heading says in full, under the pointer
	fn hint(self) -> &'static str {
		match self {
			Reading::Symbol | Reading::Name => "",
			Reading::State => {
				"The runtime's state of it: tracking, dark (no light, the IMU carrying it), coasting. Yellow from 2 s"
			}
			Reading::Station(_) => "Sensors this station lit in the last period (3 or more place the device from it)",
			Reading::Lit => "Sensors lit by any station, of those it has",
			Reading::Sure => "How sure the position is: the tracker's own standard deviation",
			Reading::Residual => "How well the light fits the pose: the residual of the hits, in milliradians",
			Reading::Loss => "Radio reports lost, of those sent",
			Reading::Gap => "The longest gap between two radio reports",
			Reading::Battery => "Charge",
			Reading::Valve => "Distance from Valve's pose of it: the median of the last seconds",
			Reading::Valve90 => "Distance from Valve's pose of it: 9 in 10 are within this",
			Reading::Behind => "Hits on sensors that face away from the station (reflections), of all hits",
			Reading::Losses => "Times it lost its light in the last 10 minutes",
			Reading::Strip => "Its last minute: green with light, yellow without (brighter while it coasted)",
			Reading::Note => "The runtime's diagnosis, as it words it",
		}
	}
}

use super::kit::{DOCK_DEVICES as DEVICES, DOCK_EVENTS as EVENTS, DOCK_ROOM as ROOM, DOCK_STATIONS as STATIONS};

/// Where the boxes lie over the room
struct Docks {
	room: Rect,
	stations: Rect,
	events: Rect,
	devices: Rect,
	/// what the room is seen through, and what the boxes leave free now
	view: Rect,
	free: Rect,
	/// whether the stations' box has room for a station's place under its rows
	place: bool,
	/// the most the events can be tall (down to the devices), and the range of the devices' height
	events_most: f32,
	devices_range: (f32, f32),
	/// the range of the stations' and the events' column's width
	side_range: (f32, f32),
}

impl Panel {
	/// The room over the whole tab, and the boxes over it at its edges: the room's own (its floor and what it shows)
	/// at the top left, the base stations over the events at the right, the devices' table across the bottom. Each
	/// folds to its title line, and the edges between them are dragged: the column's width, the events' and the
	/// devices' height
	pub(super) fn tracking_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		let d = self.docks(ui.painter(), r);
		// (the room first: the boxes lie over it, and take the pointer where they are)
		self.room_view(ui, r.expand(GAP), d.view, d.free);
		self.room_box(ui, d.room);
		self.stations_card(ui, d.stations, d.place);
		self.events_card(ui, d.events);
		self.devices_card(ui, d.devices);
		self.dock_edges(ui, r, &d);
	}

	/// Where the boxes go in `r`, by what is folded and what was dragged
	fn docks(&mut self, p: &Painter, r: Rect) -> Docks {
		let m = GAP;
		let fold = self.st.fold;
		// the devices' table across the bottom: as tall as its rows up to a little over half the tab (a whole body's
		// twelve devices fit the overlay), or as tall as it was dragged; whole rows, and it scrolls past that. A
		// table keeps the room of the most rows it has had, so a device that goes quiet for a moment does not move
		// every box
		self.rows_most[0] = self.rows_most[0].max(self.active_devices().len().max(1));
		let nst = self.rt.stations.values().filter(|s| s.have_pose || s.seen).count().max(1);
		self.rows_most[1] = self.rows_most[1].max(nst);
		let n = self.rows_most[0] as f32;
		let fit = ((r.height() * 0.52 - HEADER_H - TH_H - 3.0) / TR_H).floor().max(3.0);
		let rows = |n: f32| HEADER_H + TH_H + n * TR_H + 3.0;
		let devices_range = (rows(1.0), (r.height() - 2.0 * (BAND_H + m)).max(rows(1.0)));
		let devices_h = if fold[DEVICES] {
			BAND_H
		} else if self.st.devices_h > 0.0 {
			// (dragged: to whole rows, and never taller than the rows there are)
			let room_for = ((devices_range.1 - HEADER_H - TH_H - 3.0) / TR_H).floor().max(1.0);
			let asked = ((self.st.devices_h as f32 - HEADER_H - TH_H - 3.0) / TR_H).round();
			rows(asked.clamp(1.0, n.min(room_for).max(1.0)))
		} else {
			rows(n.min(fit))
		};
		let devices = Rect::from_min_max(Pos2::new(r.min.x, r.max.y - devices_h), r.max);
		let top = Rect::from_min_max(r.min, Pos2::new(r.max.x, devices.min.y - m));

		// the column at the right: three tenths of the width, or as wide as it was dragged
		let side_range = (300.0, (r.width() - 380.0).max(300.0));
		let side_w = if self.st.side_w > 0.0 {
			(self.st.side_w as f32).clamp(side_range.0, side_range.1)
		} else {
			(r.width() * 0.3).clamp(380.0, 480.0)
		};
		let rx = r.max.x - side_w;
		// the base stations with as much as their rows and a station's place take, then the events (which keep four
		// lines: where the window is too low for both, the stations' box is its rows alone)
		let events_least = if fold[EVENTS] { BAND_H } else { HEADER_H + 4.0 * 24.0 };
		let rows_h = HEADER_H + TH_H + self.rows_most[1] as f32 * TR_H + 2.0;
		let place = !fold[STATIONS] && rows_h + PLACE_H <= top.height() - events_least - m;
		let sta_h = if fold[STATIONS] {
			BAND_H
		} else {
			(rows_h + if place { PLACE_H } else { 0.0 })
				.min(top.height() - events_least - m)
				.max(HEADER_H + TH_H + TR_H)
		};
		let stations = Rect::from_min_size(Pos2::new(rx, top.min.y), Vec2::new(side_w, sta_h));
		// the events under them: down to the devices, or as tall as they were dragged
		let ev_top = stations.max.y + m;
		let events_most = (top.max.y - ev_top).max(BAND_H);
		let ev_h = if fold[EVENTS] {
			BAND_H
		} else if self.st.events_h > 0.0 {
			(self.st.events_h as f32).clamp((HEADER_H + 2.0 + 2.0 * 24.0).min(events_most), events_most)
		} else {
			events_most
		};
		let events = Rect::from_min_size(Pos2::new(rx, ev_top), Vec2::new(side_w, ev_h));

		// the room's own box over its top left: its title line and its tick boxes
		let room_w = rx - m - r.min.x;
		let room_h = if fold[ROOM] { BAND_H } else { self.room_box_height(p, room_w, true) };
		let room = Rect::from_min_size(r.min, Vec2::new(room_w, room_h));
		// The room is seen through what the boxes leave free as the panel lays them out by itself (its own column,
		// the devices' rows, the room's box without what a tick adds to it): whatever is folded or dragged, the
		// room stays where it is and as large as it is, under the boxes
		let own_rx = r.max.x - (r.width() * 0.3).clamp(380.0, 480.0);
		let view_top = r.min.y + self.room_box_height(p, own_rx - m - r.min.x, false) + m;
		let view_bottom = (r.max.y - rows(n.min(fit)) - m).max(view_top + 60.0);
		let view = Rect::from_min_max(Pos2::new(r.min.x, view_top), Pos2::new(own_rx - m, view_bottom));
		// what they leave free now, beside the column: the coverage's key is written in its corner
		let free = Rect::from_min_max(Pos2::new(r.min.x, room.max.y + m), Pos2::new(rx - m, top.max.y));
		Docks { room, stations, events, devices, view, free, place, events_most, devices_range, side_range }
	}

	/// The edges that are dragged: the devices' top, the column's left, the events' bottom. Each has a grip in the
	/// gap beside it; a double click gives the box back the size the panel lays it out with
	fn dock_edges(&mut self, ui: &mut egui::Ui, r: Rect, d: &Docks) {
		let m = GAP;
		let fold = self.st.fold;
		// (the events' own edge where they do not reach down to the devices; where they do, the gap under them is
		// theirs and the rest of it the devices')
		let events_reach = d.events.max.y + m >= d.devices.min.y - 0.5;
		let col_top = d.stations.min.y;
		let col_bottom = d.events.max.y;
		let mut edges: Vec<(usize, Rect, bool)> = Vec::new();
		if !fold[DEVICES] {
			let right = if !fold[EVENTS] && events_reach { d.events.min.x - m } else { d.devices.max.x };
			let strip = Rect::from_min_max(
				Pos2::new(d.devices.min.x, d.devices.min.y - m),
				Pos2::new(right, d.devices.min.y + 2.0),
			);
			edges.push((DEVICES, strip, false));
		}
		if !fold[EVENTS] {
			let strip = Rect::from_min_max(
				Pos2::new(d.events.min.x, d.events.max.y - 2.0),
				Pos2::new(d.events.max.x, d.events.max.y + m),
			);
			edges.push((EVENTS, strip, false));
		}
		let strip =
			Rect::from_min_max(Pos2::new(d.stations.min.x - m, col_top), Pos2::new(d.stations.min.x + 2.0, col_bottom));
		edges.push((STATIONS, strip, true));
		for (k, strip, upright) in edges {
			let resp = ui.interact(strip, Id::unique(("dock-edge", k)), Sense::click_and_drag());
			let active = resp.hovered() || resp.dragged();
			if active {
				ui.ctx().set_cursor_icon(if upright {
					egui::CursorIcon::ResizeHorizontal
				} else {
					egui::CursorIcon::ResizeVertical
				});
			}
			// (the release too: a drag made within one frame is only known by it)
			if resp.dragged() || resp.drag_stopped() {
				if let Some(at) = resp.interact_pointer_pos() {
					match k {
						DEVICES => {
							let h = (r.max.y - at.y - m * 0.5).clamp(d.devices_range.0, d.devices_range.1);
							self.st.devices_h = h as f64;
						}
						EVENTS => {
							let h = at.y - m * 0.5 - d.events.min.y;
							// (dragged down to the devices: theirs again, down to wherever the devices' top is)
							self.st.events_h = if h >= d.events_most - 6.0 { 0.0 } else { h.max(1.0) as f64 };
						}
						_ => {
							let w = (r.max.x - at.x - m * 0.5).clamp(d.side_range.0, d.side_range.1);
							self.st.side_w = w as f64;
						}
					}
					self.st.dirty = true;
				}
			}
			if resp.double_clicked() {
				match k {
					DEVICES => self.st.devices_h = 0.0,
					EVENTS => self.st.events_h = 0.0,
					_ => self.st.side_w = 0.0,
				}
				self.st.dirty = true;
			}
			let says = match k {
				DEVICES => "Drag: how tall the devices' table is. Double click: as tall as its rows",
				EVENTS => "Drag: how tall the events are. Double click: down to the devices",
				_ => "Drag: how wide this column is. Double click: its own width",
			};
			let resp = resp.on_hover_text(says);
			// its grip: a short bar in the gap, lighter under the pointer
			let t = tk();
			let c = strip.center();
			let grip = if upright {
				Rect::from_center_size(Pos2::new(strip.min.x + m * 0.5, c.y), Vec2::new(3.0, 34.0))
			} else if k == DEVICES {
				Rect::from_center_size(Pos2::new(c.x, strip.min.y + m * 0.5), Vec2::new(34.0, 3.0))
			} else {
				Rect::from_center_size(Pos2::new(c.x, strip.max.y - m * 0.5), Vec2::new(34.0, 3.0))
			};
			let col = if resp.dragged() || resp.hovered() { t.text } else { t.muted };
			ui.painter().rect_filled(grip, 1.5, col);
		}
	}

	/// The columns that fit `w`, for these stations: the ones read least go first where the table is narrow
	fn device_columns(w: f32, stations: &[(i32, String)], compare: bool) -> Vec<(Reading, Col<'static>)> {
		// a station's column: wide enough for "ch 14" while they are few
		let sw = if stations.len() <= 6 { 50.0 } else { 34.0 };
		let mut cols: Vec<(Reading, Col<'static>)> = vec![
			(Reading::Symbol, Col::text("", 38.0)),
			(Reading::Name, Col::text("Device", 156.0)),
			(Reading::State, Col::text("State", 112.0).sep()),
		];
		for (k, (idx, _)) in stations.iter().enumerate() {
			let c = Col::num("", sw);
			cols.push((Reading::Station(*idx), if k == 0 { c.sep() } else { c }));
		}
		cols.push((Reading::Lit, Col::num("Lit", 62.0).sep()));
		cols.push((Reading::Sure, Col::num("± mm", 62.0).sep()));
		cols.push((Reading::Residual, Col::num("Resid mrad", 82.0)));
		cols.push((Reading::Loss, Col::num("RF loss %", 78.0).sep()));
		cols.push((Reading::Gap, Col::num("Gap ms", 64.0)));
		cols.push((Reading::Battery, Col::num("Batt %", 62.0).sep()));
		if compare {
			cols.push((Reading::Valve, Col::num("Valve mm", 76.0).sep()));
			cols.push((Reading::Valve90, Col::num("p90", 56.0)));
		}
		cols.push((Reading::Behind, Col::num("Behind %", 76.0).sep()));
		cols.push((Reading::Losses, Col::num("Losses", 62.0)));
		cols.push((Reading::Strip, Col::text("Last 60 s", 132.0).sep()));
		cols.push((Reading::Note, Col::text("Diagnosis", -1.0).sep()));
		// too narrow: the diagnosis keeps 150 points while anything else can go
		let fixed = |cols: &[(Reading, Col)]| cols.iter().map(|(_, c)| c.w.max(0.0)).sum::<f32>();
		for gone in [
			Reading::Gap,
			Reading::Valve90,
			Reading::Residual,
			Reading::Losses,
			Reading::Lit,
			Reading::Behind,
			Reading::Strip,
			Reading::Valve,
		] {
			if fixed(&cols) + 150.0 <= w {
				break;
			}
			cols.retain(|(k, _)| *k != gone);
		}
		cols
	}

	/// The devices, a row each; over them, what is done to the selected one
	pub(super) fn devices_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let list: Vec<String> = self.active_devices().iter().map(|d| d.codename.clone()).collect();
		let shown = self.shown_device().map(|d| d.codename.clone());
		let folded = self.dock(ui, r, "Devices", DEVICES);
		let hr = Self::dock_header(r);
		let p = ui.painter().clone();
		let tw = kit::text_w(&p, "Devices", title());
		let count = format!("{} active", list.len());
		kit::text(&p, Pos2::new(hr.min.x + tw + 10.0, hr.center().y), Align2::LEFT_CENTER, &count, label(), t.muted);

		// the selected device's buttons, its name before them
		let has = shown.is_some();
		let items = [
			("Locate", Look::Plain, has, "The room turns to it (a controller buzzes)"),
			("Details", Look::Plain, has, "Its last seconds as curves, and its sensors"),
			("Output", Look::Plain, has, "What SteamVR is shown of it: smoothing, prediction, offsets"),
			("Rename", Look::Plain, has, "Its name in the panel"),
		];
		let (clicked, left) =
			kit::buttons_right(ui, Id::unique("dev-act"), hr.max.x - 3.0, hr.center().y, 26.0, &items);
		if let Some(d) = shown.as_ref().and_then(|c| self.rt.devices.get(c)) {
			let mut says = self.short_name(d);
			let mut level = 0;
			if self.in_overlay() && self.vc.kb_open && self.vc.kb_tag == d.serial {
				says = "SteamVR's keyboard is open".into();
			} else if self.in_overlay() && self.now < self.kb_error_until {
				says = self.vc.kb_error.clone();
				level = 2;
			}
			let c = if level > 0 { t.status(level) } else { t.soft };
			kit::text(&p, Pos2::new(left - 10.0, hr.center().y), Align2::RIGHT_CENTER, &says, label(), c);
		}
		if let (Some(k), Some(code)) = (clicked, shown.clone()) {
			match k {
				0 => self.locate(&code),
				1 => {
					self.open_device_page(&code);
					return;
				}
				2 => {
					if let Some(d) = self.rt.devices.get_mut(&code).map(Device::copy_light) {
						self.output_show(&d);
					}
					return;
				}
				_ => {
					if let Some(d) = self.rt.devices.get_mut(&code).map(Device::copy_light) {
						self.sel = Sel::Device(code);
						self.begin_rename(&d);
					}
				}
			}
		}
		if folded {
			return;
		}

		let body = Self::under_header(r);
		let stations: Vec<(i32, String)> = self
			.rt
			.stations
			.iter()
			.filter(|(_, s)| s.have_pose || s.seen)
			.map(|(i, s)| (*i, if s.mode >= 0 { (s.mode + 1).to_string() } else { i.to_string() }))
			.collect();
		let layout = Self::device_columns(body.width(), &stations, self.st.compare);
		let mut cols: Vec<Col> = layout.iter().map(|(_, c)| *c).collect();
		// the stations' headings are their channels
		let titles: Vec<String> =
			stations.iter().map(|(_, ch)| if stations.len() <= 6 { format!("ch {ch}") } else { ch.clone() }).collect();
		let mut k = 0;
		for (i, (kind, _)) in layout.iter().enumerate() {
			if matches!(kind, Reading::Station(_)) {
				cols[i].title = &titles[k];
				k += 1;
			}
		}
		let table = Table::new(body.min.x, body.width(), &cols);
		let y0 = table.header(&p, body.min.y, &cols);
		for (i, (kind, _)) in layout.iter().enumerate() {
			let hint = kind.hint();
			if !hint.is_empty() {
				let cell = table.cell(i, body.min.y, TH_H);
				ui.interact(cell, Id::unique(("dev-col", i)), Sense::hover()).on_hover_text(hint);
			}
		}
		let list_r = Rect::from_min_max(Pos2::new(body.min.x, y0), body.max);
		let mut child = ui.new_child(UiBuilder::new().max_rect(list_r).id_salt("devices"));
		let (mut select, mut details) = (None, None);
		// (as low as one row: left to itself a list that scrolls keeps 64 points, and draws its second row under the box)
		ScrollArea::vertical().id_salt("devlist").auto_shrink([false, false]).min_scrolled_height(TR_H).show(
			&mut child,
			|ui| {
				ui.spacing_mut().item_spacing = Vec2::ZERO;
				if list.is_empty() {
					let s = if self.rt.connected(self.now) { "No device is tracking" } else { "No runtime" };
					let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), TR_H), Sense::hover());
					kit::text(
						ui.painter(),
						Pos2::new(rr.min.x + PAD, rr.center().y),
						Align2::LEFT_CENTER,
						s,
						label(),
						t.muted,
					);
				}
				let mut last_y = y0;
				for (k, code) in list.iter().enumerate() {
					let Some(d) = self.rt.devices.get_mut(code).map(Device::copy_light) else { continue };
					let selected = shown.as_deref() == Some(code.as_str());
					let (rr, resp) = ui.allocate_exact_size(Vec2::new(table.width, TR_H), Sense::click());
					let hv = ui.ctx().animate_bool_with_time(Id::unique(("rowh", code)), resp.hovered(), 0.08);
					kit::row_ground(ui.painter(), rr, k % 2 == 1, selected, hv);
					self.device_row(ui, &table, &layout, rr.min.y, &d);
					if resp.double_clicked() {
						details = Some(code.clone());
					} else if resp.clicked() {
						select = Some(code.clone());
					}
					last_y = rr.max.y;
				}
				table.seps(ui.painter(), y0, last_y);
			},
		);
		if let Some(c) = details {
			self.open_device_page(&c);
		} else if let Some(c) = select {
			self.sel = Sel::Device(c);
		}
	}

	/// A device's last `STRIP_S` seconds in `r`: green where it had light, a dim yellow where it had none, yellow
	/// where it coasted, grey where it was not tracking yet, empty where it sent nothing
	fn light_strip(&self, p: &Painter, r: Rect, d: &Device) {
		let t = tk();
		p.rect_filled(r, 0.0, t.ctl);
		let n = ((r.width() / 3.0).floor() as usize).clamp(8, 400);
		let t1 = self.rt.run_time;
		let t0 = t1 - STRIP_S;
		// each stretch's worst: 1 lit, 2 not tracking yet, 3 dark, 4 coasting
		let mut cell = vec![0u8; n];
		for s in d.history.since(t0) {
			let k = ((s.t - t0) / STRIP_S * n as f64) as usize;
			if k < n {
				let v = match s.light {
					Light::Tracking => 1,
					Light::Other => 2,
					Light::Dark => 3,
					Light::Coasting => 4,
				};
				cell[k] = cell[k].max(v);
			}
		}
		let w = r.width() / n as f32;
		let mut i = 0;
		while i < n {
			let v = cell[i];
			let mut j = i + 1;
			while j < n && cell[j] == v {
				j += 1;
			}
			if v != 0 {
				let col = match v {
					1 => t.good,
					2 => t.faint,
					3 => mix(t.ctl, t.warn, 0.55),
					_ => t.warn,
				};
				let rr = Rect::from_min_max(
					Pos2::new(r.min.x + i as f32 * w, r.min.y),
					Pos2::new(r.min.x + j as f32 * w, r.max.y),
				);
				p.rect_filled(rr, 0.0, col);
			}
			i = j;
		}
	}

	/// One device's row at `y`: each column's reading
	fn device_row(&mut self, ui: &mut egui::Ui, table: &Table, layout: &[(Reading, Col)], y: f32, d: &Device) {
		let t = tk();
		let p = ui.painter().clone();
		let now = self.now;
		let h = self.health_of(d);
		let dash = "–";
		let have_diag = now - d.diag.wall < 2.0;
		let fresh = now - d.hits_wall < 2.5;
		let valve = self.valve_of(d).filter(|v| v.err_med >= 0.0).map(|v| (v.err_med, v.err_p90));
		let figure = |i: usize, s: &str, known: bool| {
			table.put(&p, i, y, s, num(), if known { t.text } else { t.faint });
		};
		for (i, (kind, _)) in layout.iter().enumerate() {
			let cell = table.cell(i, y, TR_H);
			if cell.width() < 4.0 {
				continue;
			}
			match *kind {
				Reading::Symbol => {
					let gr =
						Rect::from_center_size(Pos2::new(cell.center().x + 2.0, cell.center().y), Vec2::splat(22.0));
					self.draw_symbol(&p, gr, d, t.kind(&d.subtype));
				}
				Reading::Name => self.name_cell(ui, d, cell, semibold(14.0)),
				Reading::State => {
					table.put(&p, i, y, &h.state, label(), self.state_ink(d));
				}
				Reading::Station(idx) => match d.light.get(&idx).filter(|l| fresh && l.hits > 0) {
					Some(l) => {
						let c = if l.sensors_lit >= 3 { t.text } else { t.muted };
						table.put(&p, i, y, &l.sensors_lit.to_string(), num(), c);
					}
					None => figure(i, "·", false),
				},
				Reading::Lit => {
					if d.sensor_ct > 0 {
						figure(i, &format!("{}/{}", d.lit_sensors(now), d.sensor_ct), true);
					} else {
						figure(i, dash, false);
					}
				}
				Reading::Sure => {
					let mm = d.pos_sd * 1000.0;
					if d.pos_sd < 0.0 {
						figure(i, dash, false);
					} else if mm < 99.95 {
						// (one decimal all the way: a column's figures are compared by their places)
						figure(i, &format!("{mm:.1}"), true);
					} else {
						figure(i, &format!("{mm:.0}"), true);
					}
				}
				Reading::Residual => {
					if d.res_mrad >= 0.0 {
						figure(i, &format!("{:.2}", d.res_mrad), true);
					} else {
						figure(i, dash, false);
					}
				}
				Reading::Loss => {
					if have_diag && d.diag.bars >= 0 {
						// the runtime's own grades of it: under 5 % fine, under 15 % a look, more than that bad
						let c = match d.diag.bars {
							3.. => t.good,
							2 => t.warn,
							_ => t.bad,
						};
						table.put(&p, i, y, &format!("{:.1}", d.diag.loss_pct), num(), c);
					} else {
						figure(i, dash, false);
					}
				}
				Reading::Gap => {
					if have_diag && d.diag.bars >= 0 {
						figure(i, &format!("{:.0}", d.diag.gap_ms), true);
					} else {
						figure(i, dash, false);
					}
				}
				Reading::Battery => {
					let charge = d.charge();
					if charge < 0 {
						figure(i, dash, false);
					} else {
						let low = charge < 15 && !d.charging;
						table.put(&p, i, y, &charge.to_string(), num(), if low { t.warn } else { t.text });
					}
				}
				Reading::Valve => match valve {
					Some((med, _)) => figure(i, &format!("{med:.1}"), true),
					None => figure(i, dash, false),
				},
				Reading::Valve90 => match valve {
					Some((_, p90)) => figure(i, &format!("{p90:.1}"), true),
					None => figure(i, dash, false),
				},
				Reading::Behind => {
					if have_diag {
						figure(i, &format!("{:.1}", d.diag.back_pct), true);
					} else {
						figure(i, dash, false);
					}
				}
				Reading::Losses => figure(i, &d.drops_last(600.0, now).to_string(), true),
				Reading::Strip => {
					if let Some(dv) = self.rt.devices.get(&d.codename) {
						let sr = Rect::from_min_max(
							Pos2::new(cell.min.x, cell.center().y - 4.0),
							Pos2::new(cell.max.x, cell.center().y + 4.0),
						);
						self.light_strip(&p, sr, dv);
					}
				}
				Reading::Note => {
					// the runtime's causes, the worst first; a mark where one needs attention
					let mut level = 0;
					let mut says: Vec<String> = Vec::new();
					for (c, s) in &h.causes {
						// (a gap in the light is graded as the state is: a note until it has held)
						let lv = match c.as_str() {
							"light" if d.state != "tracking" => self.state_level(d).min(2),
							_ => Health::cause_level(if *c == d.diag.headline { d.diag.severity } else { 1 }),
						};
						level = level.max(lv);
						says.push(format!("{c}: {s}"));
					}
					if d.charging {
						says.push("charging".into());
					}
					if says.is_empty() {
						continue;
					}
					let line = says.join("   ");
					// (yellow where a cause hurts the tracking; a note is ink)
					kit::text_in(&p, cell, &line, label(), if level >= 2 { t.warn } else { t.soft });
					if kit::text_cut(&p, cell.width(), &line, label()) {
						ui.interact(cell, Id::unique(("note", &d.codename)), Sense::hover()).on_hover_ui(|ui| {
							ui.set_max_width(420.0);
							for s in &says {
								ui.label(egui::RichText::new(s).font(label()).color(tk().text));
							}
						});
					}
				}
			}
		}
	}

	/// What the stations' title line says of the room's calibration: a text, what it says in full, and whether one
	/// is ready to be used. None where there is nothing happening and nothing to choose: the runtime keeps the
	/// calibration up by itself, and one it is still trying against the coming sessions is its own business (the
	/// Calibrate button's hint says so)
	fn calibration_says(&self) -> Option<(String, String, bool)> {
		let c = &self.rt.calib;
		if !self.rt.connected(self.now) || c.available != Some(true) {
			return None;
		}
		if let Some(run) = &c.running {
			return Some(if run.state == "replaying" {
				(
					format!("calibrating {:.0} %", run.progress * 100.0),
					format!("Calibrating from this session: {} still moments so far", run.moments),
					false,
				)
			} else {
				let state = if run.state.is_empty() { "starting" } else { &run.state };
				(format!("calibrating: {state}"), "Calibrating from this session".to_string(), false)
			});
		}
		c.ready.as_ref().map(|_| {
			(
				"calibration ready".to_string(),
				"A better calibration is ready: used from SteamVR's next start".to_string(),
				true,
			)
		})
	}

	/// What the runtime's diagnosis says about a station (moved, disagrees with the map, hits from behind), and the
	/// fault's level it is said at
	fn station_issue(&self, mode: i32) -> Option<(String, i32)> {
		let want = format!("ch {} ", mode + 1);
		let mut issue = None;
		for dv in self.rt.devices.values() {
			if !dv.active || self.now - dv.diag.wall > 2.0 {
				continue;
			}
			for (cz, s) in &dv.diag.causes {
				if (cz == "station" || cz == "reflect") && s.contains(&want) {
					issue = Some(if cz == "reflect" {
						("hits from behind sensors".to_string(), 0)
					} else {
						(s.clone(), 2)
					});
				}
			}
		}
		issue
	}

	/// The base stations, a row each: channel, serial, state, the devices it lights, its hits a second. Under the
	/// rows, the place in the room of the selected station (the first until one is picked). In the title line the
	/// map's state, or what a calibration is doing, and Calibrate (the session so far; the runtime keeps the room's
	/// calibration up by itself, so this is for a room changed now)
	fn stations_card(&mut self, ui: &mut egui::Ui, r: Rect, place: bool) {
		let t = tk();
		let folded = self.dock(ui, r, "Base stations", STATIONS);
		let conn = self.rt.connected(self.now);
		let hr = Self::dock_header(r);
		{
			let live = conn && self.rt.calib.available == Some(true);
			let hint = if self.rt.calib.pending.is_some() {
				"Calibrates the room from this session so far, in the background. sally keeps the calibration up by \
				 itself: this is for a room changed now (a station moved). Right now it has a calibration from an \
				 earlier session on trial: it takes over by itself if it fits the next two sessions better than the \
				 one in use"
			} else {
				"Calibrates the room from this session so far, in the background. sally keeps the calibration up by \
				 itself: this is for a room changed now (a station moved)"
			};
			let says = self.calibration_says();
			let ready = says.as_ref().is_some_and(|s| s.2);
			let mut items = vec![("Calibrate", Look::Plain, live && self.rt.calib.running.is_none(), hint)];
			if ready {
				items.insert(0, ("Use it now", Look::Primary, true, "Restarts the runtime on the new calibration"));
			}
			let (clicked, left) =
				kit::buttons_right(ui, Id::unique("calibrate"), hr.max.x - 3.0, hr.center().y, 26.0, &items);
			match clicked.map(|i| items[i].0) {
				Some("Calibrate") => self.rt.command("calibrate", crate::runtime::Want::Calibrate),
				Some(_) => self.confirm_apply = true,
				None => {}
			}
			// what a calibration is doing, else the map's state
			let at = Pos2::new(left - 10.0, hr.center().y);
			match says {
				Some((short, full, _)) => {
					let tr = kit::text(ui.painter(), at, Align2::RIGHT_CENTER, &short, label(), t.text);
					ui.interact(tr, Id::unique("calibration-says"), Sense::hover()).on_hover_text(full);
				}
				None if !self.rt.stations.is_empty() => {
					let s = if self.rt.map_locked { "map locked" } else { "map live" };
					kit::text(ui.painter(), at, Align2::RIGHT_CENTER, s, label(), t.muted);
				}
				None => {}
			}
		}
		if folded {
			return;
		}
		let body = Self::under_header(r);
		let p = ui.painter().clone();
		let cols = [
			Col::text("", 34.0),
			Col::text("Ch", 40.0),
			Col::text("Serial", 114.0),
			Col::text("State", -1.0).sep(),
			Col::num("Devices", 64.0).sep(),
			Col::num("Hits/s", 60.0),
		];
		let table = Table::new(body.min.x, body.width(), &cols);
		let y0 = table.header(&p, body.min.y, &cols);
		// the station whose place is under the rows: the selected one, else the first that has one
		let shown = match self.sel {
			Sel::Station(i) if self.rt.stations.get(&i).is_some_and(|s| s.have_pose) => Some(i),
			_ => self.rt.stations.iter().find(|(_, s)| s.have_pose).map(|(i, _)| *i),
		};
		// (without the room for a station's place, the rows alone, none of them picked out)
		let place_h = if place { PLACE_H } else { 0.0 };
		let shown = shown.filter(|i| place || self.sel == Sel::Station(*i));
		let list_r = Rect::from_min_max(Pos2::new(body.min.x, y0), Pos2::new(body.max.x, body.max.y - place_h));
		let mut child = ui.new_child(UiBuilder::new().max_rect(list_r).id_salt("stations"));
		let mut clicked = None;
		ScrollArea::vertical().id_salt("stlist").auto_shrink([false, false]).min_scrolled_height(TR_H).show(
			&mut child,
			|ui| {
				ui.spacing_mut().item_spacing = Vec2::ZERO;
				if self.rt.stations.is_empty() {
					let s = if conn { "No station yet" } else { "No runtime" };
					let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), TR_H), Sense::hover());
					kit::text(
						ui.painter(),
						Pos2::new(rr.min.x + PAD, rr.center().y),
						Align2::LEFT_CENTER,
						s,
						label(),
						t.muted,
					);
				}
				let mut last_y = y0;
				let mut k = 0;
				for (idx, sn) in &self.rt.stations {
					if !sn.have_pose && !sn.seen {
						continue;
					}
					let (rr, resp) = ui.allocate_exact_size(Vec2::new(table.width, TR_H), Sense::click());
					let p = ui.painter();
					let hv = ui.ctx().animate_bool_with_time(Id::unique(("sth", *idx)), resp.hovered(), 0.08);
					kit::row_ground(p, rr, k % 2 == 1, shown == Some(*idx), hv);
					k += 1;
					if resp.clicked() {
						clicked = Some(*idx);
					}
					let y = rr.min.y;
					let c0 = table.cell(0, y, TR_H);
					self.draw_station(
						p,
						Rect::from_center_size(Pos2::new(c0.center().x + 2.0, c0.center().y), Vec2::splat(20.0)),
						if sn.have_pose { t.soft } else { t.faint },
					);
					let ch = if sn.mode >= 0 { (sn.mode + 1).to_string() } else { idx.to_string() };
					table.put(p, 1, y, &ch, semibold(14.0), t.text);
					if sn.id != 0 {
						table.put(p, 2, y, &sn.name(), code(), t.soft);
					}
					// its state: the diagnosis about it where there is one
					let (says, level) = match self.station_issue(sn.mode) {
						Some(issue) => issue,
						None if !sn.have_pose => ("not in the map".to_string(), 0),
						None if !sn.seen => ("dark for every device".to_string(), 3),
						None => ("seen".to_string(), 0),
					};
					let c = match level {
						0 if says == "seen" => t.good,
						0 => t.muted,
						l => t.status(l),
					};
					table.put(p, 3, y, &says, label(), c);
					if sn.have_pose {
						table.put(p, 4, y, &sn.devices_lit.to_string(), num(), t.text);
						table.put(p, 5, y, &sn.hits.to_string(), num(), t.text);
					}
					last_y = rr.max.y;
				}
				table.seps(ui.painter(), y0, last_y);
			},
		);
		if let Some(i) = clicked {
			self.sel = Sel::Station(i);
		}

		// the shown station's place: where it is, how high over the floor, how it is turned
		if !place {
			return;
		}
		let mut y = body.max.y - PLACE_H;
		p.line_segment([Pos2::new(body.min.x, y), Pos2::new(body.max.x, y)], Stroke::new(1.0, t.card_edge));
		y += 3.0;
		let rows: [(&str, String); 3] = match shown.and_then(|i| self.rt.stations.get(&i)) {
			Some(sn) => {
				let q = &sn.pose;
				let (yaw, pitch, roll) = crate::history::yaw_pitch_roll(&q.q);
				[
					("Position", format!("{:+.3}  {:+.3}  {:+.3} m", q.p.x, q.p.y, q.p.z)),
					("Height", format!("{:.2} m over the floor", q.p.z - self.floor_z())),
					("Turned", format!("yaw {yaw:+.1}°  pitch {pitch:+.1}°  roll {roll:+.1}°")),
				]
			}
			None => [("Position", String::new()), ("Height", String::new()), ("Turned", String::new())],
		};
		for (k, v) in rows {
			let rr = Rect::from_min_max(Pos2::new(body.min.x + PAD, y), Pos2::new(body.max.x - PAD, y + 24.0));
			kit::kv_line(&p, rr, 84.0, k, &v, true);
			y += 24.0;
		}
	}

	fn events_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let folded = self.dock(ui, r, "Events", EVENTS);
		// which ones, at the title line's right end
		{
			let hr = Self::dock_header(r);
			let items = ["Important", "All"];
			let w = kit::segmented_w(ui.painter(), 26.0, &items);
			let pos = Pos2::new(hr.max.x - w - 3.0, hr.center().y - 13.0);
			let (_, c) = kit::segmented(
				ui,
				Id::unique("evfilter"),
				pos,
				26.0,
				&items,
				self.st.events_filter.clamp(0, 1) as usize,
			);
			if let Some(i) = c {
				self.st.events_filter = i as i32;
				self.st.dirty = true;
			}
		}
		if folded {
			return;
		}
		let body = Self::under_header(r);
		let all = self.st.events_filter == 1;
		let shown: Vec<usize> = (0..self.rt.events.len()).filter(|&i| all || self.rt.events[i].matters).collect();
		// who: a device's serial becomes its name
		let names: Vec<(String, String)> =
			self.rt.devices.values().map(|d| (d.serial.clone(), self.short_name(d))).collect();
		let font = label();
		let p0 = ui.painter().clone();
		// (a set width, so that a device with a longer name does not move every line's text)
		let who_w = (body.width() * 0.26).clamp(96.0, 150.0);
		let tm_w = kit::text_w(&p0, "00:00:00", num());
		let text_dx = PAD + tm_w + 12.0 + who_w;
		let row_h = 24.0;
		// whole rows only (the newest at the bottom, none cut in half at the top)
		let mut list_r = body;
		let spare = (list_r.height() % row_h).max(0.0);
		list_r.min.y += spare;
		let text_w = (list_r.width() - text_dx - 16.0).max(60.0);
		// an event's text goes on in the line under it where it does not fit; a warning is yellow and what stops
		// sally red, what came right again and what is only said are ink
		let galleys: Vec<std::sync::Arc<egui::Galley>> = shown
			.iter()
			.map(|&i| {
				let e = &self.rt.events[i];
				let ink = match e.level {
					EvLevel::Bad => t.bad,
					EvLevel::Warn => t.warn,
					EvLevel::Good => t.text,
					EvLevel::Info => t.soft,
				};
				p0.layout(e.text.clone(), font.clone(), ink, text_w)
			})
			.collect();
		let heights: Vec<f32> = galleys.iter().map(|g| (g.rows.len().max(1) as f32) * row_h).collect();
		let total: f32 = heights.iter().sum();
		let mut child = ui.new_child(UiBuilder::new().max_rect(list_r).id_salt("events"));
		ScrollArea::vertical()
			.id_salt("evscroll")
			.auto_shrink([false, false])
			.min_scrolled_height(row_h)
			.stick_to_bottom(true)
			.show_viewport(&mut child, |ui, view| {
				ui.set_height(total.max(row_h));
				let top = ui.min_rect().min;
				let right = top.x + list_r.width();
				if shown.is_empty() {
					kit::text(
						ui.painter(),
						top + Vec2::new(PAD, row_h * 0.5),
						Align2::LEFT_CENTER,
						"Nothing yet",
						font.clone(),
						t.muted,
					);
				}
				let mut y = 0.0;
				for (k, &i) in shown.iter().enumerate() {
					let h = heights[k];
					if y + h < view.min.y || y > view.max.y {
						y += h;
						continue;
					}
					let at = top + Vec2::new(0.0, y);
					y += h;
					let e = &self.rt.events[i];
					let p = ui.painter();
					let cy = at.y + row_h * 0.5;
					// (striped by the event's own place in the list, so a row keeps its ground as the list scrolls)
					if k % 2 == 1 {
						p.rect_filled(Rect::from_min_max(at, Pos2::new(right, at.y + h)), 0.0, t.stripe);
					}
					p.line_segment([Pos2::new(at.x, at.y + h), Pos2::new(right, at.y + h)], Stroke::new(1.0, t.line));
					// (the lines between the time, who and what)
					for x in [at.x + PAD + tm_w + 6.0, at.x + text_dx - 4.0] {
						p.line_segment([Pos2::new(x, at.y), Pos2::new(x, at.y + h)], Stroke::new(1.0, t.line));
					}
					let tm = sally_os::localtime::local_time(e.clock).map(|x| x.hms()).unwrap_or_default();
					kit::text(p, Pos2::new(at.x + PAD, cy), Align2::LEFT_CENTER, &tm, num(), t.muted);
					let who = names
						.iter()
						.find(|(s, _)| *s == e.who)
						.map(|(_, n)| n.clone())
						.unwrap_or_else(|| e.who.clone());
					let wr = Rect::from_min_max(
						Pos2::new(at.x + PAD + tm_w + 12.0, at.y),
						Pos2::new(at.x + text_dx - 10.0, at.y + row_h),
					);
					kit::text_in(p, wr, &who, font.clone(), t.soft);
					let tx = at.x + text_dx + 6.0;
					let g = galleys[k].clone();
					let gy = at.y + (row_h - g.rows.first().map_or(row_h, |r| r.height())) * 0.5;
					p.galley(Pos2::new(tx, gy), g, t.text);
				}
			});
	}

	/// A device's name in `r`: as text, or as a field while it is being renamed in this view (Rename, over the
	/// devices' table and in the Settings tab's rows). Enter or a click away keeps the text, Escape puts the old name
	/// back, an empty field is the default name again
	pub(super) fn name_cell(&mut self, ui: &mut egui::Ui, d: &Device, r: Rect, font: egui::FontId) {
		let t = tk();
		let p = ui.painter().clone();
		let frame = self.frame_no[self.view.index()];
		if let Some(rn) = self.rename.as_ref().filter(|rn| rn.view == self.view && rn.serial == d.serial) {
			// the field went away without Enter or Escape (another tab): the text is kept
			if frame > rn.last_frame + 1 && rn.frames > 0 {
				let (s, b) = (rn.serial.clone(), rn.buf.clone());
				self.rename = None;
				self.set_name(&s, &b);
			}
		}
		if self.rename.as_ref().is_some_and(|rn| rn.view == self.view && rn.serial == d.serial) {
			let hint = self.default_name(d);
			let rn = self.rename.as_mut().unwrap();
			rn.last_frame = frame;
			let h = font.size + 8.0;
			let fr = Rect::from_min_size(
				Pos2::new(r.min.x - 6.0, r.center().y - h * 0.5),
				Vec2::new((r.width() + 6.0).clamp(120.0, 360.0), h),
			);
			let edit = egui::TextEdit::singleline(&mut rn.buf)
				.font(font)
				.hint_text(hint)
				.char_limit(32)
				.margin(Vec2::new(6.0, 2.0));
			let resp = ui.put(fr, edit);
			if rn.frames == 0 {
				resp.request_focus();
				// the whole name selected, so typing replaces it
				if let Some(mut state) = egui::TextEdit::load_state(ui.ctx(), resp.id) {
					let n = rn.buf.chars().count();
					state.cursor.set_char_range(Some(egui::text::CCursorRange::two(
						egui::text::CCursor::new(0),
						egui::text::CCursor::new(n),
					)));
					state.store(ui.ctx(), resp.id);
				}
			}
			rn.frames += 1;
			let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
			let escape = ui.input(|i| i.key_pressed(egui::Key::Escape));
			if escape {
				self.rename = None;
			} else if enter || (rn.frames > 2 && !resp.has_focus()) {
				let (s, b) = (rn.serial.clone(), rn.buf.clone());
				self.rename = None;
				self.set_name(&s, &b);
			}
			return;
		}
		if self.in_overlay() && self.vc.kb_open && self.vc.kb_tag == d.serial {
			p.rect_filled(r.expand2(Vec2::new(4.0, -3.0)), t.ctl_radius, t.ctl_hover);
		}
		kit::text_in(&p, r, &self.short_name(d), font, t.text);
	}

	/// Starts renaming a device: in the window its name turns into a field (`name_cell`); in the overlay SteamVR's
	/// keyboard comes up instead, and its text is applied when it reports Done (Panel::frame)
	pub(super) fn begin_rename(&mut self, d: &Device) {
		if d.serial.is_empty() || d.serial == "-" {
			return;
		}
		let name = self.short_name(d);
		if self.in_overlay() {
			let desc = format!("Name for {}", d.codename);
			if !self.vc.overlay_keyboard(&d.serial, &desc, &name) {
				self.kb_error_until = self.now + 4.0;
			}
			return;
		}
		// a name still being typed elsewhere in this view is kept first
		if let Some(rn) = self.rename.take() {
			self.set_name(&rn.serial, &rn.buf);
		}
		let frame = self.frame_no[self.view.index()];
		self.rename =
			Some(Rename { serial: d.serial.clone(), view: self.view, buf: name, frames: 0, last_frame: frame });
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_devices_columns_go_one_by_one_where_the_table_is_narrow() {
		let stations: Vec<(i32, String)> = (0..4).map(|i| (i, (i + 1).to_string())).collect();
		let kinds =
			|w: f32| -> Vec<Reading> { Panel::device_columns(w, &stations, true).iter().map(|(k, _)| *k).collect() };
		// wide: every reading, the diagnosis last
		let all = kinds(1580.0);
		assert_eq!(all.len(), 19);
		assert_eq!(all.last(), Some(&Reading::Note));
		// the smallest window: the gap and the 90 % go first; the name, the state, the stations and the diagnosis stay
		let few = kinds(1080.0);
		assert!(!few.contains(&Reading::Gap) && !few.contains(&Reading::Valve90));
		for k in [Reading::Name, Reading::State, Reading::Station(3), Reading::Sure, Reading::Battery, Reading::Note] {
			assert!(few.contains(&k), "{k:?}");
		}
		// without the comparison there is no Valve to be far from
		assert!(!Panel::device_columns(1580.0, &stations, false).iter().any(|(k, _)| *k == Reading::Valve));
	}
}
