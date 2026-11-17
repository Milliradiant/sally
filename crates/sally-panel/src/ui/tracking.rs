//! The Tracking tab: the devices, the room, the selected device (or station) in numbers and small pictures, the base
//! stations, the events.
//!
//! A device's row says what it is (SteamVR's own symbol of it, symbols.rs), how it is doing (the rail down its left
//! edge, its state in words when it is not tracking), which stations light it (a chip each), its radio and battery,
//! how sure its position is, and its last minute as a strip: green lit by two stations or more, blue by one, amber
//! dark, red coasting.

use egui::{Align2, Color32, Id, Painter, Pos2, Rect, ScrollArea, Sense, Stroke, UiBuilder, Vec2};

use super::icons;
use super::kit::{self, Look, Surface, CTL_H, GAP, HEADER_H, PAD};
use super::theme::*;
use super::{role_def, Health, Panel, Rename, Sel};
use crate::history::Light;
use crate::runtime::{Device, EvLevel};

/// What a device row was asked
enum RowAction {
	Select(String),
	Locate(String),
	Details(String),
}

/// A device row's height (twelve of them fit the overlay's list: a headset, two controllers, nine trackers), and how
/// far back its strip goes (s)
const ROW_H: f32 = 64.0;
const STRIP_S: f64 = 60.0;

/// One of the selected device's numbers
struct Tile<'a> {
	label: &'a str,
	value: String,
	unit: &'a str,
	color: Color32,
	foot: String,
	foot_color: Color32,
}

impl Panel {
	pub(super) fn tracking_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		let m = GAP;
		let left_w = Self::list_width(r.width());
		let right_w = (r.width() * 0.262).clamp(352.0, 414.0);
		let cx = r.min.x + left_w + m;
		let cw = r.width() - left_w - right_w - 2.0 * m;
		let rx = r.max.x - right_w;
		let events_h = (r.height() * 0.21).clamp(128.0, 186.0);

		self.devices_card(ui, Rect::from_min_size(r.min, Vec2::new(left_w, r.height())));
		self.room_card(ui, Rect::from_min_size(Pos2::new(cx, r.min.y), Vec2::new(cw, r.height() - events_h - m)));
		self.events_card(ui, Rect::from_min_size(Pos2::new(cx, r.max.y - events_h), Vec2::new(cw, events_h)));

		// right: the selected device, then the base stations with as much as their rows take (whichever does not fit
		// scrolls)
		let nst = self.rt.stations.values().filter(|s| s.have_pose || s.seen).count().max(1) as f32;
		let foot = if self.calibration_foot().is_some() { 46.0 } else { 0.0 };
		let sta_h = (HEADER_H + nst * 46.0 + foot + PAD - 4.0).min(r.height() * 0.46);
		let sel_h = r.height() - sta_h - m;
		self.selected_card(ui, Rect::from_min_size(Pos2::new(rx, r.min.y), Vec2::new(right_w, sel_h)));
		self.stations_card(ui, Rect::from_min_size(Pos2::new(rx, r.min.y + sel_h + m), Vec2::new(right_w, sta_h)));
	}

	pub(super) fn devices_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let list: Vec<String> = self.active_devices().iter().map(|d| d.codename.clone()).collect();
		let nflag = self.flagged_count();
		let inner = self.card(ui, r, "Devices");
		{
			let p = ui.painter();
			let hr = Self::header(r);
			let mut x = hr.max.x - 6.0;
			if nflag > 0 {
				let s = format!("{nflag} flagged");
				x = kit::text(p, Pos2::new(x, hr.center().y), Align2::RIGHT_CENTER, &s, label(), t.warn).min.x - 10.0;
			}
			let s = format!("{} active", list.len());
			kit::text(p, Pos2::new(x, hr.center().y), Align2::RIGHT_CENTER, &s, label(), t.muted);
		}
		let list_r = Rect::from_min_max(Pos2::new(r.min.x + 6.0, inner.min.y), Pos2::new(r.max.x - 2.0, r.max.y - 6.0));
		let mut child = ui.new_child(UiBuilder::new().max_rect(list_r).id_salt("devices"));
		let mut action = None;
		let shown = self.shown_device().map(|d| d.codename.clone());
		ScrollArea::vertical().id_salt("devlist").auto_shrink([false, false]).show(&mut child, |ui| {
			ui.spacing_mut().item_spacing = Vec2::ZERO;
			if list.is_empty() {
				let s = if self.rt.connected(self.now) { "No device is tracking" } else { "No runtime" };
				let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 40.0), Sense::hover());
				kit::text(ui.painter(), rr.min + Vec2::new(8.0, 10.0), Align2::LEFT_TOP, s, body(), t.muted);
			}
			for code in &list {
				let Some(d) = self.rt.devices.get(code) else { continue };
				let selected = shown.as_deref() == Some(code.as_str());
				if let Some(a) = self.device_row(ui, d, selected) {
					action = Some(a);
				}
			}
		});
		match action {
			Some(RowAction::Select(c)) => self.sel = Sel::Device(c),
			Some(RowAction::Locate(c)) => {
				self.sel = Sel::Device(c.clone());
				self.locate(&c);
			}
			Some(RowAction::Details(c)) => self.open_device_page(&c),
			None => {}
		}
	}

	/// A device's last `STRIP_S` seconds in `r`: each stretch by the light it had
	fn light_strip(&self, p: &Painter, r: Rect, d: &Device) {
		let t = tk();
		p.rect_filled(r, 2.5, t.ctl);
		let n = ((r.width() / 3.0).floor() as usize).clamp(8, 400);
		let t1 = self.rt.run_time;
		let t0 = t1 - STRIP_S;
		// each stretch's worst: 1 two stations or more, 2 one, 3 dark, 4 coasting
		let mut cell = vec![0u8; n];
		for s in d.history.since(t0) {
			let k = ((s.t - t0) / STRIP_S * n as f64) as usize;
			if k < n {
				let v = match s.light {
					Light::Tracking if s.lh_mask.count_ones() >= 2 => 1,
					Light::Tracking => 2,
					Light::Coasting => 4,
					Light::Dark | Light::Other => 3,
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
					1 => alpha(t.good, 150),
					2 => t.note,
					3 => t.warn,
					_ => t.bad,
				};
				let rr = Rect::from_min_max(
					Pos2::new(r.min.x + i as f32 * w, r.min.y),
					Pos2::new(r.min.x + j as f32 * w, r.max.y),
				);
				p.rect_filled(rr, 2.0, col);
			}
			i = j;
		}
	}

	/// One device: the rail and the figure, its name (and its state when it is not tracking), how sure its position
	/// is; the stations lighting it, its radio and battery; its last minute; Locate and Details at the right end
	fn device_row(&self, ui: &mut egui::Ui, d: &Device, selected: bool) -> Option<RowAction> {
		let t = tk();
		let w = ui.available_width();
		let (rr, resp) = ui.allocate_exact_size(Vec2::new(w, ROW_H), Sense::click());
		let p = ui.painter().clone();
		let h = self.health_of(d);
		let body = Rect::from_min_max(rr.min + Vec2::new(0.0, 2.0), Pos2::new(rr.max.x - 4.0, rr.max.y - 2.0));
		let over = resp.contains_pointer();
		let hv = ui.ctx().animate_bool_with_time(Id::unique(("rowh", &d.codename)), over, 0.1);
		if selected {
			p.rect_filled(body, t.ctl_radius, t.ctl);
		} else if hv > 0.0 {
			p.rect_filled(body, t.ctl_radius, fade(t.ctl, 0.6 * hv));
		}
		if (2..=3).contains(&h.level) {
			p.rect_filled(body, t.ctl_radius, alpha(h.color, 18));
		}
		// the rail
		let rail = Rect::from_min_max(
			Pos2::new(body.min.x + 5.0, body.min.y + 8.0),
			Pos2::new(body.min.x + 8.0, body.max.y - 8.0),
		);
		p.rect_filled(rail, 1.5, h.color);
		// what it is: SteamVR's symbol of it
		let gr = Rect::from_min_size(Pos2::new(body.min.x + 16.0, body.min.y + 7.0), Vec2::new(32.0, 32.0));
		self.draw_symbol(&p, gr, d, if h.level <= 1 { t.soft } else { h.color });

		let x0 = body.min.x + 58.0;
		let xr = body.max.x - 40.0;
		// line 1: the name, the state, how sure the position is
		let sdmm = if d.pos_sd >= 0.0 { d.pos_sd * 1000.0 } else { -1.0 };
		let conf = if sdmm < 0.0 {
			String::new()
		} else if sdmm < 9.95 {
			format!("±{sdmm:.1} mm")
		} else {
			format!("±{sdmm:.0} mm")
		};
		let y1 = body.min.y + 15.0;
		let mut right = xr;
		if !conf.is_empty() {
			right = kit::text(&p, Pos2::new(xr, y1), Align2::RIGHT_CENTER, &conf, label(), kit::sd_color(sdmm)).min.x
				- 10.0;
		}
		let drops = d.drops_last(600.0, self.now);
		if drops > 0 {
			let c = if drops >= 10 { t.warn } else { t.muted };
			let tr = kit::text(&p, Pos2::new(right, y1), Align2::RIGHT_CENTER, &drops.to_string(), label(), c);
			kit::icon_at(&p, Pos2::new(tr.min.x - 9.0, y1), icons::LIGHTNING_SLASH, 13.0, c);
			right = tr.min.x - 24.0;
		}
		{
			let pc = p.with_clip_rect(
				Rect::from_min_max(Pos2::new(x0, body.min.y), Pos2::new(right, body.max.y)).intersect(p.clip_rect()),
			);
			let nr = kit::text(&pc, Pos2::new(x0, y1), Align2::LEFT_CENTER, &self.short_name(d), name(), t.text);
			if d.state != "tracking" {
				let c = if h.level >= 3 { t.bad } else { t.warn };
				kit::text(&pc, Pos2::new(nr.max.x + 8.0, y1 + 1.0), Align2::LEFT_CENTER, &h.state, label(), c);
			}
		}
		// line 2: the stations lighting it; from the right the battery, the radio
		let y2 = body.min.y + 36.0;
		self.station_chips(&p, Pos2::new(x0, y2 - 8.5), 17.0, d);
		let mut x = xr;
		let charge = d.charge();
		if charge >= 0 {
			let c = if charge < 15 {
				t.bad
			} else if charge < 30 {
				t.warn
			} else {
				t.muted
			};
			let tr = kit::text(&p, Pos2::new(x, y2), Align2::RIGHT_CENTER, &format!("{charge}"), label(), c);
			kit::battery(&p, Pos2::new(tr.min.x - 27.0, y2 - 5.5), 22.0, 11.0, charge, d.charging);
			x = tr.min.x - 27.0 - 14.0;
		}
		if self.now - d.diag.wall < 2.0 && d.diag.bars >= 0 {
			let c = match d.diag.bars {
				3.. => t.muted,
				2 => t.warn,
				_ => t.bad,
			};
			let loss = if d.diag.loss_pct < 9.95 {
				format!("{:.1}%", d.diag.loss_pct)
			} else {
				format!("{:.0}%", d.diag.loss_pct)
			};
			let tr = kit::text(&p, Pos2::new(x, y2), Align2::RIGHT_CENTER, &loss, label(), c);
			kit::signal(&p, Pos2::new(tr.min.x - kit::signal_w(12.0) - 5.0, y2 - 6.5), 12.0, d.diag.bars);
		}
		// line 3: the last minute
		let sr = Rect::from_min_max(Pos2::new(x0, body.max.y - 10.0), Pos2::new(xr, body.max.y - 6.0));
		self.light_strip(&p, sr, d);

		// Locate over Details, on the row under the pointer and the selected one
		let (mut loc, mut det) = (false, false);
		if over || selected {
			let ax = body.max.x - 33.0;
			let loc_r = Rect::from_min_size(Pos2::new(ax, body.min.y + 2.0), Vec2::splat(28.0));
			let det_r = Rect::from_min_size(Pos2::new(ax, body.max.y - 30.0), Vec2::splat(28.0));
			loc =
				kit::icon_button(ui, Id::unique(("locate", &d.codename)), loc_r, icons::CROSSHAIR_SIMPLE, false, true)
					.on_hover_text("Locate: the room turns to it (a controller buzzes)")
					.clicked();
			det = kit::icon_button(ui, Id::unique(("details", &d.codename)), det_r, icons::CHART_LINE, false, true)
				.on_hover_text("Details: its last seconds as curves, its sensors")
				.clicked();
		}
		let resp = resp.on_hover_ui(|ui| {
			ui.set_max_width(380.0);
			if h.causes.is_empty() {
				ui.label(egui::RichText::new("Nothing to note").font(label()).color(tk().soft));
			}
			for (c, s) in &h.causes {
				ui.label(egui::RichText::new(format!("{c}: {s}")).font(label()).color(tk().text));
			}
			if drops > 0 {
				let s = format!("lost its light {drops} time{} in the last 10 min", if drops == 1 { "" } else { "s" });
				ui.label(egui::RichText::new(s).font(label()).color(tk().muted));
			}
		});
		if loc {
			Some(RowAction::Locate(d.codename.clone()))
		} else if det || resp.double_clicked() {
			Some(RowAction::Details(d.codename.clone()))
		} else if resp.clicked() {
			Some(RowAction::Select(d.codename.clone()))
		} else {
			None
		}
	}

	fn selected_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		if let Sel::Station(i) = self.sel {
			if self.rt.stations.contains_key(&i) {
				self.selected_station(ui, r, i);
				return;
			}
		}
		let Some(code) = self.shown_device().map(|d| d.codename.clone()) else {
			let inner = self.card(ui, r, "Selected");
			let s = if self.rt.connected(self.now) { "No device is tracking" } else { "No runtime" };
			kit::text(ui.painter(), inner.min + Vec2::new(0.0, 4.0), Align2::LEFT_TOP, s, body(), t.muted);
			return;
		};
		// a copy (renaming changes the panel's state while it draws), without the history this box reads apart
		let sd_line: Vec<Option<f32>> = self.sd_history(&code, 30.0, 60);
		let Some(d) = self.rt.devices.get_mut(&code).map(Device::copy_light) else { return };
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Card);
		let h = self.health_of(&d);
		let x0 = r.min.x + PAD;
		let xr = r.max.x - PAD;

		// the head: its symbol, the name (a click renames it), what it is; Locate, Details and Output at the right
		let gr = Rect::from_min_size(Pos2::new(x0, r.min.y + 14.0), Vec2::new(36.0, 38.0));
		self.draw_symbol(&p, gr, &d, if h.level <= 1 { t.soft } else { h.color });
		let bw = 30.0;
		let mut bx = xr - bw;
		let top = r.min.y + 12.0;
		let mut pressed = None;
		for (k, glyph, tip) in [
			(2, icons::SLIDERS_HORIZONTAL, "Output: what SteamVR is shown of it"),
			(1, icons::CHART_LINE, "Details: its last seconds as curves, its sensors"),
			(0, icons::CROSSHAIR_SIMPLE, "Locate: the room turns to it (a controller buzzes)"),
		] {
			let br = Rect::from_min_size(Pos2::new(bx, top), Vec2::splat(bw));
			if kit::icon_button(ui, Id::unique(("sel-act", k)), br, glyph, false, true).on_hover_text(tip).clicked() {
				pressed = Some(k);
			}
			bx -= bw + 2.0;
		}
		match pressed {
			Some(0) => self.locate(&code),
			Some(1) => {
				self.open_device_page(&code);
				return;
			}
			Some(2) => {
				self.output_show(&d);
				return;
			}
			_ => {}
		}
		let nx = x0 + 44.0;
		self.name_field(ui, &d, Pos2::new(nx, r.min.y + 12.0), bx + bw - nx - 8.0, semibold(20.0), true);
		{
			let role = self.role_of(&d);
			let mut sub = self.type_of(&d);
			let mut subc = t.muted;
			if self.given_name(&d).is_some() && role != "none" && role != "held" {
				sub = format!("{}, {sub}", role_def(role).1);
			}
			if self.in_overlay() && self.vc.kb_open && self.vc.kb_tag == d.serial {
				sub = "SteamVR's keyboard is open".into();
			} else if self.in_overlay() && self.now < self.kb_error_until {
				sub = self.vc.kb_error.clone();
				subc = t.warn;
			}
			let sr = kit::text(&p, Pos2::new(nx, r.min.y + 46.0), Align2::LEFT_CENTER, &sub, label(), subc);
			let id = format!("{}  {}", d.codename, d.serial);
			let clip = Rect::from_min_max(Pos2::new(sr.max.x + 10.0, r.min.y + 34.0), Pos2::new(xr, r.min.y + 58.0));
			kit::text_in(&p, clip, &id, super::theme::code(), t.faint);
		}

		// the state, and the runtime's causes as it words them: two lines kept, so nothing below moves
		let mut y = r.min.y + 68.0;
		let sc = if h.level <= 1 { t.good } else { h.color };
		let pr = {
			let s = h.state.clone();
			let tw = kit::text_w(&p, &s, label());
			let pr = Rect::from_min_size(Pos2::new(x0, y), Vec2::new(tw + 30.0, 24.0));
			p.rect_filled(pr, 12.0, alpha(sc, 30));
			kit::dot(
				&p,
				Pos2::new(pr.min.x + 12.0, pr.center().y),
				4.0,
				sc,
				d.state == "dark" || d.state == "coasting",
			);
			p.text(Pos2::new(pr.min.x + 22.0, pr.center().y), Align2::LEFT_CENTER, s, label(), t.text);
			pr
		};
		{
			let cx = pr.max.x + 10.0;
			let w = xr - cx;
			if h.causes.is_empty() {
				kit::text(&p, Pos2::new(cx, pr.center().y), Align2::LEFT_CENTER, "nothing to note", label(), t.faint);
			}
			for (i, (c, s)) in h.causes.iter().take(2).enumerate() {
				let lv = Health::cause_level(c, if *c == d.diag.headline { d.diag.severity } else { 1 });
				let col = if lv == 0 { t.muted } else { t.level(lv) };
				let mut line = format!("{c}: {s}");
				if i == 1 && h.causes.len() > 2 {
					line += &format!("  (+{})", h.causes.len() - 2);
				}
				// one cause is centred on the state's pill, two take a line each around it
				let ly = if h.causes.len() == 1 { pr.center().y } else { pr.center().y - 9.0 + i as f32 * 18.0 };
				let clip = Rect::from_min_max(Pos2::new(cx, ly - 10.0), Pos2::new(cx + w, ly + 10.0));
				kit::text_in(&p, clip, &line, label(), col);
			}
		}
		y += 24.0 + 16.0;

		// the numbers
		let now = self.now;
		let (mut nseen, mut nall) = (0, 0);
		for (idx, s) in &self.rt.stations {
			if s.have_pose || s.seen {
				nall += 1;
			}
			if d.lh_mask & (1u32 << idx) != 0 {
				nseen += 1;
			}
		}
		let sdmm = if d.pos_sd >= 0.0 { d.pos_sd * 1000.0 } else { -1.0 };
		let lit = d.lit_sensors(now);
		let have_diag = now - d.diag.wall < 2.0;
		let bars = if have_diag { d.diag.bars } else { -1 };
		let charge = d.charge();
		let v = self.valve_of(&d).filter(|v| v.err_med >= 0.0).map(|v| (v.err_med, v.err_p90));
		let drops = d.drops_last(600.0, now);
		let dash = || "–".to_string();
		let tiles = [
			Tile {
				label: "Position",
				value: if sdmm < 0.0 { dash() } else { format!("±{sdmm:.1}") },
				unit: "mm",
				color: kit::sd_color(sdmm),
				foot: if d.res_mrad >= 0.0 {
					format!("light fits to {:.2} mrad", d.res_mrad)
				} else if d.rot_sd >= 0.0 {
					format!("turn ±{:.2}°", d.rot_sd * 57.29578)
				} else {
					String::new()
				},
				foot_color: t.muted,
			},
			Tile {
				label: "Light",
				value: format!("{nseen}/{}", nall.max(1)),
				unit: "stations",
				color: match nseen {
					0 => t.bad,
					1 => t.note,
					_ => t.text,
				},
				foot: if d.sensor_ct > 0 { format!("{lit} of {} sensors lit", d.sensor_ct) } else { String::new() },
				foot_color: if d.sensor_ct > 0 && lit < 3 { t.warn } else { t.muted },
			},
			Tile {
				label: "Radio",
				value: if bars < 0 { dash() } else { format!("{:.1}", d.diag.loss_pct) },
				unit: "% lost",
				color: match bars {
					-1 | 3.. => t.text,
					2 => t.warn,
					_ => t.bad,
				},
				foot: if bars < 0 { String::new() } else { format!("longest gap {:.0} ms", d.diag.gap_ms) },
				foot_color: t.muted,
			},
			Tile {
				label: "Battery",
				value: if charge < 0 { dash() } else { charge.to_string() },
				unit: "%",
				color: if (0..15).contains(&charge) {
					t.bad
				} else if (0..30).contains(&charge) {
					t.warn
				} else {
					t.text
				},
				foot: if d.charging { "charging".into() } else { String::new() },
				foot_color: t.note,
			},
			Tile {
				label: "From Valve",
				value: match v {
					Some((med, _)) => format!("{med:.1}"),
					None => dash(),
				},
				unit: "mm",
				color: if v.is_some_and(|v| v.0 >= 15.0) { t.warn } else { t.text },
				foot: match v {
					Some((_, p90)) => format!("9 in 10 within {p90:.1} mm"),
					None if !self.st.compare => "comparison off".into(),
					None => String::new(),
				},
				foot_color: t.muted,
			},
			Tile {
				label: "From behind",
				value: if have_diag { format!("{:.1}", d.diag.back_pct) } else { dash() },
				unit: "% of hits",
				color: if have_diag && d.diag.back_pct > 5.0 { t.note } else { t.text },
				foot: format!("light lost {drops}× in 10 min"),
				foot_color: if drops >= 10 { t.warn } else { t.muted },
			},
		];
		let (tw, th) = ((r.width() - 2.0 * PAD - 8.0) / 2.0, 72.0);
		for (i, tile) in tiles.iter().enumerate() {
			let (col, row) = ((i % 2) as f32, (i / 2) as f32);
			let tr = Rect::from_min_size(Pos2::new(x0 + col * (tw + 8.0), y + row * (th + 8.0)), Vec2::new(tw, th));
			let g = Self::tile(&p, tr, tile);
			match i {
				0 => {
					let hi = sd_line.iter().flatten().fold(2.0f32, |a, b| a.max(*b)) * 1.15;
					kit::sparkline(&p, g, &sd_line, 0.0, hi, if sdmm >= 5.0 { t.warn } else { t.good });
				}
				1 => {
					// the chips from the right, smaller where there are many
					let off = Pos2::new(-4000.0, -4000.0);
					let mut ch = 18.0;
					let mut w = self.station_chips(&p.with_clip_rect(Rect::NOTHING), off, ch, &d);
					if w > g.width() {
						ch = (ch * g.width() / w).max(12.0);
						w = self.station_chips(&p.with_clip_rect(Rect::NOTHING), off, ch, &d);
					}
					self.station_chips(&p, Pos2::new(g.max.x - w, g.min.y + 4.0), ch, &d);
				}
				2 if bars >= 0 => {
					kit::signal(&p, Pos2::new(g.max.x - kit::signal_w(22.0), g.min.y + 2.0), 22.0, bars);
				}
				3 if charge >= 0 => {
					kit::battery(&p, Pos2::new(g.max.x - 38.0, g.min.y + 5.0), 38.0, 18.0, charge, d.charging);
				}
				4 => {
					let line: Vec<Option<f32>> = self
						.valve_of(&d)
						.map(|v| v.err_window.iter().map(|q| Some(q.1 as f32)).collect())
						.unwrap_or_default();
					let hi = line.iter().flatten().fold(5.0f32, |a, b| a.max(*b)) * 1.15;
					kit::sparkline(&p, g, &line, 0.0, hi, t.valve);
				}
				_ => {}
			}
		}
		y += 3.0 * (th + 8.0) + 6.0;

		// the light by station: hits a second, sensors lit
		let pc = p.with_clip_rect(
			Rect::from_min_max(Pos2::new(r.min.x, y - 4.0), r.max - Vec2::new(0.0, 6.0)).intersect(p.clip_rect()),
		);
		kit::text(&pc, Pos2::new(x0, y + 8.0), Align2::LEFT_CENTER, "Light by station", label(), t.muted);
		kit::text(&pc, Pos2::new(xr, y + 8.0), Align2::RIGHT_CENTER, "sensors", caption(), t.faint);
		kit::text(&pc, Pos2::new(xr - 62.0, y + 8.0), Align2::RIGHT_CENTER, "hits/s", caption(), t.faint);
		y += 24.0;
		let fresh = now - d.hits_wall < 2.5;
		let max_hits = d.light.values().map(|l| if fresh { l.hits } else { 0 }).max().unwrap_or(0).max(1);
		for (idx, sn) in &self.rt.stations {
			if !sn.have_pose && !sn.seen {
				continue;
			}
			let l = d.light.get(idx).filter(|_| fresh);
			let (hits, sens) = l.map(|l| (l.hits, l.sensors_lit)).unwrap_or((0, 0));
			let cy = y + 9.0;
			let c = if hits > 0 { t.text } else { t.faint };
			kit::text(&pc, Pos2::new(x0, cy), Align2::LEFT_CENTER, &sn.channel_label(), label(), c);
			let mc = if sens < 3 { t.note } else { t.good };
			let mr = Rect::from_min_max(Pos2::new(x0 + 54.0, cy - 2.5), Pos2::new(xr - 116.0, cy + 2.5));
			kit::meter(&pc, mr, hits as f64 / max_hits as f64, mc);
			kit::text(&pc, Pos2::new(xr - 62.0, cy), Align2::RIGHT_CENTER, &hits.to_string(), label(), c);
			kit::text(&pc, Pos2::new(xr, cy), Align2::RIGHT_CENTER, &sens.to_string(), label(), c);
			y += 21.0;
		}

		// the last minute, where the box has room left: how sure the position was, and the light under it
		let room = r.max.y - PAD - y;
		if room >= 84.0 {
			y += 12.0;
			let line: Vec<Option<f32>> = self.sd_history(&code, STRIP_S, 120);
			let worst = line.iter().flatten().fold(0.0f32, |a, b| a.max(*b));
			kit::text(&p, Pos2::new(x0, y + 8.0), Align2::LEFT_CENTER, "Last minute", label(), t.muted);
			if worst > 0.0 {
				let s = format!("up to ±{worst:.1} mm");
				kit::text(&p, Pos2::new(xr, y + 8.0), Align2::RIGHT_CENTER, &s, caption(), t.faint);
			}
			y += 24.0;
			let plot = Rect::from_min_max(Pos2::new(x0, y), Pos2::new(xr, (r.max.y - PAD - 12.0).min(y + 96.0)));
			p.rect_filled(plot.expand2(Vec2::new(0.0, 3.0)), 4.0, fade(t.well, 0.6));
			let hi = worst.max(2.0) * 1.2;
			// a line at 5 mm, where the figure turns amber
			if hi > 5.0 {
				let ly = plot.max.y - 5.0 / hi * plot.height();
				p.line_segment(
					[Pos2::new(plot.min.x, ly), Pos2::new(plot.max.x, ly)],
					Stroke::new(1.0, alpha(t.warn, 60)),
				);
			}
			kit::sparkline(&p, plot, &line, 0.0, hi, if worst >= 5.0 { t.warn } else { t.good });
			if let Some(dv) = self.rt.devices.get(&code) {
				let sr = Rect::from_min_max(Pos2::new(x0, plot.max.y + 7.0), Pos2::new(xr, plot.max.y + 12.0));
				self.light_strip(&p, sr, dv);
			}
		}
	}

	/// A number's tile: its label, the value and unit, a line under them; the room at its right for a small picture
	fn tile(p: &Painter, r: Rect, tile: &Tile) -> Rect {
		let t = tk();
		p.rect_filled(r, t.ctl_radius, mix(t.card, t.ctl, 0.55));
		let x = r.min.x + 11.0;
		kit::text(p, Pos2::new(x, r.min.y + 15.0), Align2::LEFT_CENTER, tile.label, label(), t.muted);
		kit::value_unit(p, Pos2::new(x, r.min.y + 49.0), &tile.value, tile.unit, figure(), tile.color);
		if !tile.foot.is_empty() {
			let clip = Rect::from_min_max(Pos2::new(x, r.max.y - 21.0), Pos2::new(r.max.x - 8.0, r.max.y - 3.0));
			kit::text_in(p, clip, &tile.foot, caption(), tile.foot_color);
		}
		let lw = kit::text_w(p, tile.label, label());
		Rect::from_min_max(
			Pos2::new((x + lw + 12.0).max(r.min.x + r.width() * 0.42), r.min.y + 8.0),
			Pos2::new(r.max.x - 11.0, r.min.y + 34.0),
		)
	}

	/// A device's position sd (mm) over its last `span` seconds in `n` steps (None where it had none)
	fn sd_history(&self, code: &str, span: f64, n: usize) -> Vec<Option<f32>> {
		let Some(d) = self.rt.devices.get(code) else { return Vec::new() };
		let t0 = self.rt.run_time - span;
		let mut out = vec![None; n];
		for s in d.history.since(t0) {
			let k = ((s.t - t0) / span * n as f64) as usize;
			if k < n && s.sd_mm >= 0.0 {
				out[k] = Some(out[k].map_or(s.sd_mm as f32, |v: f32| v.max(s.sd_mm as f32)));
			}
		}
		out
	}

	fn selected_station(&mut self, ui: &mut egui::Ui, r: Rect, idx: i32) {
		let t = tk();
		let sn = self.rt.stations[&idx].clone();
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Card);
		let x0 = r.min.x + PAD;
		let xr = r.max.x - PAD;
		let (c, state) = if !sn.have_pose {
			(t.faint, "not in the map")
		} else if sn.seen {
			(t.good, "lighting devices")
		} else {
			(t.bad, "dark for every device")
		};
		let sc = if sn.have_pose && sn.seen { t.soft } else { c };
		self.draw_station(&p, Rect::from_min_size(Pos2::new(x0, r.min.y + 16.0), Vec2::new(36.0, 36.0)), sc);
		kit::text(
			&p,
			Pos2::new(x0 + 44.0, r.min.y + 24.0),
			Align2::LEFT_CENTER,
			&sn.channel_label(),
			semibold(20.0),
			t.text,
		);
		kit::text(&p, Pos2::new(x0 + 44.0, r.min.y + 46.0), Align2::LEFT_CENTER, state, label(), c);
		kit::text(&p, Pos2::new(xr, r.min.y + 24.0), Align2::RIGHT_CENTER, &sn.name(), code(), t.faint);
		let mut rows: Vec<(&str, String)> = vec![(
			"Map",
			if sn.have_pose {
				if sn.locked {
					"locked"
				} else {
					"live"
				}
			} else {
				"not placed"
			}
			.into(),
		)];
		if sn.have_pose {
			let q = &sn.pose;
			rows.push(("Position", format!("{:+.3}  {:+.3}  {:+.3} m", q.p.x, q.p.y, q.p.z)));
			rows.push(("Height", format!("{:.2} m over the floor", q.p.z - self.floor_z())));
			let (y, pi, ro) = crate::history::yaw_pitch_roll(&q.q);
			rows.push(("Turned", format!("yaw {y:+.1}°  pitch {pi:+.1}°  roll {ro:+.1}°")));
		}
		rows.push(("Lights", format!("{} devices, {} hits a second", sn.devices_lit, sn.hits)));
		let mut y = r.min.y + 76.0;
		for (k, v) in rows {
			kit::text(&p, Pos2::new(x0, y), Align2::LEFT_CENTER, k, label(), t.muted);
			kit::text(&p, Pos2::new(x0 + 84.0, y), Align2::LEFT_CENTER, &v, label(), t.text);
			y += 23.0;
		}
		// what it lights: each device's hits from it, and how many of its sensors
		y += 10.0;
		let pc = p.with_clip_rect(
			Rect::from_min_max(Pos2::new(r.min.x, y - 6.0), r.max - Vec2::new(0.0, 6.0)).intersect(p.clip_rect()),
		);
		kit::text(&pc, Pos2::new(x0, y), Align2::LEFT_CENTER, "What it lights", label(), t.muted);
		kit::text(&pc, Pos2::new(xr, y), Align2::RIGHT_CENTER, "sensors", caption(), t.faint);
		y += 22.0;
		let devs = self.active_devices();
		let max_hits = devs
			.iter()
			.filter(|d| self.now - d.hits_wall < 2.5)
			.filter_map(|d| d.light.get(&idx).map(|l| l.hits))
			.max()
			.unwrap_or(0)
			.max(1);
		for d in devs {
			let l = d.light.get(&idx).filter(|_| self.now - d.hits_wall < 2.5);
			let (hits, sens) = l.map(|l| (l.hits, l.sensors_lit)).unwrap_or((0, 0));
			let c = if hits > 0 { t.text } else { t.faint };
			let clip = Rect::from_min_max(Pos2::new(x0, y - 10.0), Pos2::new(x0 + 128.0, y + 10.0));
			kit::text_in(&pc, clip, &self.short_name(d), label(), c);
			let mr = Rect::from_min_max(Pos2::new(x0 + 136.0, y - 2.5), Pos2::new(xr - 44.0, y + 2.5));
			kit::meter(&pc, mr, hits as f64 / max_hits as f64, if sens < 3 { t.note } else { t.good });
			kit::text(&pc, Pos2::new(xr, y), Align2::RIGHT_CENTER, &sens.to_string(), label(), c);
			y += 21.0;
		}
	}

	/// What the stations' box says of the room's calibration under its rows: a text, how far a running one has got,
	/// and whether one is ready to be used. None where there is nothing happening and nothing to choose: the runtime
	/// keeps the calibration up by itself, and one it is still trying against the coming sessions is its own business
	/// (the Calibrate button's hint says so)
	fn calibration_foot(&self) -> Option<(String, Option<f64>, bool)> {
		let c = &self.rt.calib;
		if !self.rt.connected(self.now) || c.available != Some(true) {
			return None;
		}
		if let Some(run) = &c.running {
			let s = if run.state == "replaying" {
				format!("Calibrating: {:.0} %, {} still moments", run.progress * 100.0, run.moments)
			} else {
				format!("Calibrating: {}", if run.state.is_empty() { "starting" } else { &run.state })
			};
			return Some((s, Some(run.progress), false));
		}
		c.ready
			.as_ref()
			.map(|_| ("A better calibration of this room is ready: used from SteamVR's next start".into(), None, true))
	}

	fn stations_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let inner = self.card(ui, r, "Base stations");
		let conn = self.rt.connected(self.now);
		// in the header: the map's state, and Calibrate (the session so far; the runtime keeps the room's calibration
		// up by itself, so this is for a room changed now)
		{
			let hr = Self::header(r);
			let live = conn && self.rt.calib.available == Some(true);
			let bw = kit::button_w(ui.painter(), "Calibrate", true);
			let br = Rect::from_min_size(Pos2::new(hr.max.x - bw - 4.0, hr.center().y - 14.0), Vec2::new(bw, 28.0));
			let resp = kit::button(
				ui,
				Id::unique("calibrate"),
				br,
				"Calibrate",
				Some(icons::RULER),
				Look::Ghost,
				live && self.rt.calib.running.is_none(),
			)
			.on_hover_text(if self.rt.calib.pending.is_some() {
				"Calibrates the room from this session so far, in the background. sally keeps the calibration up by \
				 itself: this is for a room changed now (a station moved). Right now it has a calibration from an \
				 earlier session on trial: it takes over by itself if it fits the next two sessions better than the \
				 one in use"
			} else {
				"Calibrates the room from this session so far, in the background. sally keeps the calibration up by \
				 itself: this is for a room changed now (a station moved)"
			});
			if resp.clicked() {
				self.rt.command("calibrate", crate::runtime::Want::Calibrate);
			}
			if !self.rt.stations.is_empty() {
				let s = if self.rt.map_locked { "map locked" } else { "map live" };
				kit::text(
					ui.painter(),
					Pos2::new(br.min.x - 10.0, hr.center().y),
					Align2::RIGHT_CENTER,
					s,
					label(),
					t.muted,
				);
			}
		}
		let foot = self.calibration_foot();
		let list_bottom = if foot.is_some() { inner.max.y - 36.0 } else { r.max.y - 6.0 };
		let list_r = Rect::from_min_max(Pos2::new(r.min.x + 6.0, inner.min.y), Pos2::new(r.max.x - 2.0, list_bottom));
		let mut child = ui.new_child(UiBuilder::new().max_rect(list_r).id_salt("stations"));
		let mut clicked = None;
		let max_hits = self.rt.stations.values().map(|s| s.hits).max().unwrap_or(0).max(1);
		ScrollArea::vertical().id_salt("stlist").auto_shrink([false, false]).show(&mut child, |ui| {
			ui.spacing_mut().item_spacing = Vec2::ZERO;
			if self.rt.stations.is_empty() {
				let s = if conn { "No station yet" } else { "No runtime" };
				let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 36.0), Sense::hover());
				kit::text(ui.painter(), rr.min + Vec2::new(8.0, 8.0), Align2::LEFT_TOP, s, body(), t.muted);
			}
			for (idx, sn) in &self.rt.stations {
				if !sn.have_pose && !sn.seen {
					continue;
				}
				let (rr, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 46.0), Sense::click());
				let p = ui.painter();
				let selected = self.sel == Sel::Station(*idx);
				let body = Rect::from_min_max(rr.min + Vec2::new(0.0, 2.0), Pos2::new(rr.max.x - 4.0, rr.max.y - 2.0));
				let hv = ui.ctx().animate_bool_with_time(Id::unique(("sth", *idx)), resp.hovered(), 0.1);
				if selected {
					p.rect_filled(body, t.ctl_radius, t.ctl);
				} else if hv > 0.0 {
					p.rect_filled(body, t.ctl_radius, fade(t.ctl, 0.6 * hv));
				}
				if resp.clicked() {
					clicked = Some(*idx);
				}
				let c = if !sn.have_pose {
					t.faint
				} else if sn.seen {
					t.soft
				} else {
					t.bad
				};
				self.draw_station(
					p,
					Rect::from_center_size(Pos2::new(body.min.x + 22.0, body.center().y), Vec2::splat(26.0)),
					c,
				);
				let x0 = body.min.x + 44.0;
				let xr = body.max.x - 12.0;
				let nr = kit::text(
					p,
					Pos2::new(x0, body.min.y + 13.0),
					Align2::LEFT_CENTER,
					&sn.channel_label(),
					semibold(15.0),
					t.text,
				);
				if sn.id != 0 {
					kit::text(
						p,
						Pos2::new(nr.max.x + 9.0, body.min.y + 13.5),
						Align2::LEFT_CENTER,
						&sn.name(),
						code(),
						t.faint,
					);
				}
				// the diagnosis about this station (moved, disagrees with the map, dark for everyone, reflections)
				let want = format!("ch {} ", sn.mode + 1);
				let mut issue = None;
				for dv in self.rt.devices.values() {
					if !dv.active || self.now - dv.diag.wall > 2.0 {
						continue;
					}
					for (cz, s) in &dv.diag.causes {
						if (cz == "station" || cz == "reflect") && s.contains(&want) {
							issue = Some(if cz == "reflect" {
								("hits from behind sensors".to_string(), t.note)
							} else {
								(s.clone(), t.warn)
							});
						}
					}
				}
				let (sub, sc) = match issue {
					Some((s, c)) => (s, c),
					None if !sn.have_pose => ("not in the map".to_string(), t.muted),
					None if !sn.seen => ("dark".to_string(), t.bad),
					None => (
						format!("lights {} device{}", sn.devices_lit, if sn.devices_lit == 1 { "" } else { "s" }),
						t.muted,
					),
				};
				let clip = Rect::from_min_max(Pos2::new(x0, body.min.y + 21.0), Pos2::new(xr - 96.0, body.max.y - 1.0));
				kit::text_in(p, clip, &sub, label(), sc);
				if sn.have_pose {
					// its hits a second, and how they stand against the busiest station's
					let hy = body.min.y + 13.5;
					let ur = kit::text(p, Pos2::new(xr, hy), Align2::RIGHT_CENTER, "hits/s", caption(), t.faint);
					kit::text(
						p,
						Pos2::new(ur.min.x - 5.0, hy),
						Align2::RIGHT_CENTER,
						&sn.hits.to_string(),
						label(),
						t.soft,
					);
					let mr =
						Rect::from_min_max(Pos2::new(xr - 84.0, body.max.y - 13.0), Pos2::new(xr, body.max.y - 8.0));
					kit::meter(p, mr, sn.hits as f64 / max_hits as f64, if sn.seen { t.good } else { t.bad });
				}
			}
		});
		if let Some(i) = clicked {
			self.sel = Sel::Station(i);
		}
		if let Some((text, progress, ready)) = foot {
			let p = ui.painter();
			let fr = Rect::from_min_max(
				Pos2::new(inner.min.x, inner.max.y - 30.0),
				Pos2::new(inner.max.x, inner.max.y + 2.0),
			);
			p.line_segment(
				[Pos2::new(r.min.x + 1.0, fr.min.y - 6.0), Pos2::new(r.max.x - 1.0, fr.min.y - 6.0)],
				Stroke::new(1.0, t.line),
			);
			let mut right = fr.max.x;
			if ready {
				let bw = kit::button_w(p, "Use it now", false);
				let br = Rect::from_min_size(Pos2::new(right - bw, fr.center().y - 14.0), Vec2::new(bw, 28.0));
				let resp = kit::button(ui, Id::unique("calibrate-apply"), br, "Use it now", None, Look::Primary, true)
					.on_hover_text("Restarts the runtime on the new calibration");
				if resp.clicked() {
					self.confirm_apply = true;
				}
				right = br.min.x - 10.0;
			}
			if let Some(k) = progress {
				let mr = Rect::from_min_max(
					Pos2::new(right - 90.0, fr.center().y - 2.5),
					Pos2::new(right, fr.center().y + 2.5),
				);
				kit::meter(p, mr, k, t.note);
				right = mr.min.x - 10.0;
			}
			let clip = Rect::from_min_max(fr.min, Pos2::new(right, fr.max.y));
			kit::text_in(p, clip, &text, label(), if ready { t.good } else { t.soft });
		}
	}

	fn events_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let inner = self.card(ui, r, "Events");
		// which ones, at the header's right end
		{
			let hr = Self::header(r);
			let items = ["Important", "All"];
			let w = kit::segmented_w(ui.painter(), 28.0, &items);
			let pos = Pos2::new(hr.max.x - w - 4.0, hr.center().y - 14.0);
			let (_, c) = kit::segmented(
				ui,
				Id::unique("evfilter"),
				pos,
				28.0,
				&items,
				self.st.events_filter.clamp(0, 1) as usize,
			);
			if let Some(i) = c {
				self.st.events_filter = i as i32;
				self.st.dirty = true;
			}
		}
		let all = self.st.events_filter == 1;
		let shown: Vec<usize> = (0..self.rt.events.len()).filter(|&i| all || self.rt.events[i].matters).collect();
		// who: a device's serial becomes its name; wide enough for the longest, within reason
		let names: Vec<(String, String)> =
			self.rt.devices.values().map(|d| (d.serial.clone(), self.short_name(d))).collect();
		let font = label();
		let p0 = ui.painter().clone();
		let who_w = names
			.iter()
			.map(|(_, n)| kit::text_w(&p0, n, font.clone()))
			.fold(96.0f32, f32::max)
			.min(inner.width() * 0.3)
			+ 14.0;
		let tm_w = kit::text_w(&p0, "00:00:00", font.clone());
		let text_dx = tm_w + 26.0 + who_w;
		let row_h = 21.0;
		// whole rows only (the newest at the bottom, none cut in half at the top): what is left of the height is
		// shared above the first and under the last
		let mut list_r = Rect::from_min_max(inner.min, Pos2::new(r.max.x - 4.0, r.max.y - 8.0));
		let spare = (list_r.height() % row_h).max(0.0);
		list_r.min.y += (spare * 0.5).floor();
		list_r.max.y -= spare - (spare * 0.5).floor();
		let text_w = (list_r.width() - text_dx - 16.0).max(60.0);
		// an event's text goes on in the line under it where it does not fit
		let galleys: Vec<std::sync::Arc<egui::Galley>> = shown
			.iter()
			.map(|&i| {
				let e = &self.rt.events[i];
				let tc = if e.level == EvLevel::Info { t.soft } else { t.text };
				p0.layout(e.text.clone(), font.clone(), tc, text_w)
			})
			.collect();
		let heights: Vec<f32> = galleys.iter().map(|g| (g.rows.len().max(1) as f32) * row_h).collect();
		let total: f32 = heights.iter().sum();
		let mut child = ui.new_child(UiBuilder::new().max_rect(list_r).id_salt("events"));
		ScrollArea::vertical().id_salt("evscroll").auto_shrink([false, false]).stick_to_bottom(true).show_viewport(
			&mut child,
			|ui, view| {
				ui.set_height(total.max(row_h));
				let top = ui.min_rect().min;
				if shown.is_empty() {
					kit::text(
						ui.painter(),
						top + Vec2::new(0.0, 2.0),
						Align2::LEFT_TOP,
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
					let c = match e.level {
						EvLevel::Good => t.good,
						EvLevel::Warn => t.warn,
						EvLevel::Bad => t.bad,
						EvLevel::Info => t.faint,
					};
					let cy = at.y + row_h * 0.5;
					let tm = sally_os::localtime::local_time(e.clock).map(|x| x.hms()).unwrap_or_default();
					kit::text(p, Pos2::new(at.x, cy), Align2::LEFT_CENTER, &tm, font.clone(), t.muted);
					p.circle_filled(Pos2::new(at.x + tm_w + 13.0, cy), 3.5, c);
					let who = names
						.iter()
						.find(|(s, _)| *s == e.who)
						.map(|(_, n)| n.clone())
						.unwrap_or_else(|| e.who.clone());
					let wr = Rect::from_min_max(
						Pos2::new(at.x + tm_w + 26.0, at.y),
						Pos2::new(at.x + text_dx - 12.0, at.y + row_h),
					);
					kit::text_in(p, wr, &who, font.clone(), t.soft);
					let g = galleys[k].clone();
					let gy = at.y + (row_h - g.rows.first().map_or(row_h, |r| r.height())) * 0.5;
					let tc = if e.level == EvLevel::Info { t.soft } else { t.text };
					p.galley(Pos2::new(at.x + text_dx, gy), g, tc);
				}
			},
		);
	}

	/// A device's name at `pos` (its top left), in `font`; a click renames it. In the window a field takes the name's
	/// place: Enter or a click away keeps the text, Escape puts the old name back, an empty field is the default name
	/// again. In the overlay SteamVR's keyboard comes up instead, and its text is applied when it reports Done
	/// (Panel::frame). `only`: the one name field of its tab (the Selected box's), which ends the renaming of any other
	/// device; a list's rows each see to their own
	pub(super) fn name_field(
		&mut self, ui: &mut egui::Ui, d: &Device, pos: Pos2, max_w: f32, font: egui::FontId, only: bool,
	) {
		let t = tk();
		let p = ui.painter().clone();
		let h = font.size + 8.0;
		let frame = self.frame_no[self.view.index()];
		if let Some(rn) = self.rename.as_ref().filter(|r| r.view == self.view) {
			// the field went away without Enter or Escape (another device picked, another tab): the text is kept
			let gone = if rn.serial == d.serial { frame > rn.last_frame + 1 } else { only };
			if gone {
				let (s, b) = (rn.serial.clone(), rn.buf.clone());
				self.rename = None;
				self.set_name(&s, &b);
			}
		}
		if self.rename.as_ref().is_some_and(|r| r.view == self.view && r.serial == d.serial) {
			let hint = self.default_name(d);
			let rn = self.rename.as_mut().unwrap();
			rn.last_frame = frame;
			let w = (120.0f32).max((max_w + 6.0).min(360.0));
			let fr = Rect::from_min_size(Pos2::new(pos.x - 6.0, pos.y - 2.0), Vec2::new(w, h));
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
		let name = self.short_name(d);
		let nw = kit::text_w(&p, &name, font.clone()).min(max_w);
		let can = !d.serial.is_empty() && d.serial != "-";
		let kb = self.in_overlay() && self.vc.kb_open && self.vc.kb_tag == d.serial;
		let hr = Rect::from_min_max(Pos2::new(pos.x - 6.0, pos.y - 2.0), Pos2::new(pos.x + nw + 6.0, pos.y + h - 2.0));
		let mut clicked = false;
		let mut hov = false;
		if can {
			let resp = ui.interact(hr, Id::unique(("rename", &d.serial)), Sense::click());
			hov = resp.hovered();
			if hov {
				ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
			}
			clicked = resp.on_hover_text("Rename").clicked();
		}
		if hov || kb {
			p.rect_filled(hr, t.ctl_radius, if kb { t.ctl_hover } else { t.ctl });
		}
		let clip = Rect::from_min_max(Pos2::new(pos.x, pos.y - 2.0), Pos2::new(pos.x + max_w, pos.y + h - 2.0));
		kit::text_in(&p, clip, &name, font, t.text);
		if clicked {
			if self.in_overlay() {
				let desc = format!("Name for {}", d.codename);
				if !self.vc.overlay_keyboard(&d.serial, &desc, &name) {
					self.kb_error_until = self.now + 4.0;
				}
			} else {
				// a name still being typed elsewhere in this view is kept first
				if let Some(rn) = self.rename.take() {
					self.set_name(&rn.serial, &rn.buf);
				}
				self.rename =
					Some(Rename { serial: d.serial.clone(), view: self.view, buf: name, frames: 0, last_frame: frame });
			}
		}
	}
}

/// A row of a pop-up with a switch at its right end: whether it was clicked
pub(super) fn switch_row(ui: &mut egui::Ui, id: Id, text: &str, on: bool) -> bool {
	let t = tk();
	let (rr, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), CTL_H), Sense::click());
	kit::text(ui.painter(), Pos2::new(rr.min.x, rr.center().y), Align2::LEFT_CENTER, text, body(), t.text);
	let c = kit::switch(ui, id, Pos2::new(rr.max.x - 42.0, rr.center().y - 12.0), on, true);
	c || resp.clicked()
}
