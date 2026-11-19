//! The Settings tab: the devices' names and where each is worn, the SteamVR overlay, what the driver does that is the
//! user's to choose (whose tracking a lighthouse headset gets, how many sessions' recordings stay, whether a session
//! keeps a pose log), and the advanced
//! things: the station map's lock and the comparison with Valve's driver.

use egui::{Align2, Id, Pos2, Rect, ScrollArea, Sense, Stroke, UiBuilder, Vec2};

use super::icons;
use super::kit::{self, Look, Pop, Range, CTL_H, GAP, PAD};
use super::theme::*;
use super::{role_def, Panel, ROLES};
use crate::runtime::{Device, Want};

impl Panel {
	pub(super) fn settings_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		// the devices at the left; at the right the overlay's box over the advanced things
		let w = r.width();
		let c2 = (w * 0.42).clamp(400.0, 680.0);
		let c1 = w - c2 - GAP;
		let overlay_h = 150.0;
		let choices_h = 202.0;
		let x2 = r.min.x + c1 + GAP;
		self.worn_card(ui, Rect::from_min_max(r.min, Pos2::new(r.min.x + c1, r.max.y)));
		self.overlay_card(ui, Rect::from_min_max(Pos2::new(x2, r.min.y), Pos2::new(r.max.x, r.min.y + overlay_h)));
		let y = r.min.y + overlay_h + GAP;
		self.choices_card(ui, Rect::from_min_max(Pos2::new(x2, y), Pos2::new(r.max.x, y + choices_h)));
		self.advanced_card(ui, Rect::from_min_max(Pos2::new(x2, y + choices_h + GAP), r.max));
	}

	/// A segmented control that cannot be used yet (no word from the driver): there, and faded
	fn segmented_off(p: &egui::Painter, pos: Pos2, items: &[&str]) {
		let t = tk();
		let w = kit::segmented_w(p, CTL_H, items);
		let r = Rect::from_min_size(pos, Vec2::new(w, CTL_H));
		p.rect_filled(r, t.ctl_radius, fade(t.ctl, 0.6));
		let each = w / items.len() as f32;
		for (i, s) in items.iter().enumerate() {
			let c = Pos2::new(r.min.x + each * (i as f32 + 0.5), r.center().y);
			p.text(c, Align2::CENTER_CENTER, s, label(), t.faint);
		}
	}

	/// The sessions' files in the logs folder, added up again every 20 s while they are asked for
	fn logs_bytes(&mut self) -> u64 {
		let now = self.now;
		if self.logs_size.is_none_or(|(at, _)| now - at > 20.0) {
			let dir = self.replay_paths().logs.clone();
			let bytes = std::fs::read_dir(dir)
				.map(|rd| rd.flatten().filter_map(|e| e.metadata().ok()).filter(|m| m.is_file()).map(|m| m.len()).sum())
				.unwrap_or(0);
			self.logs_size = Some((now, bytes));
		}
		self.logs_size.map_or(0, |(_, b)| b)
	}

	/// What the driver does that is the user's to choose, kept by the driver between sessions: whose tracking a
	/// lighthouse headset gets, how many sessions' recordings and logs stay, whether a session keeps a pose log
	fn choices_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let inner = self.card(ui, r, "Headset and recordings");
		let p = ui.painter().clone();
		let heard = self.vc.driver_heard(self.now);
		let mut y = inner.min.y + 2.0;

		// the headset
		let items = ["sally", "Valve"];
		kit::text(&p, Pos2::new(inner.min.x + 2.0, y + 11.0), Align2::LEFT_CENTER, "Headset tracking", body(), t.text);
		let sp = Pos2::new(inner.max.x - kit::segmented_w(&p, CTL_H, &items), y + 4.0);
		let says = if !heard {
			"Chosen while SteamVR runs with sally"
		} else if self.vc.drv_lighthouse_headset {
			"Whose tracking this headset gets while SteamVR is shown sally's"
		} else {
			"For an Index or Vive headset: this one is not tracked by the base stations"
		};
		let clip = Rect::from_min_max(Pos2::new(inner.min.x + 2.0, y + 20.0), Pos2::new(sp.x - 8.0, y + 40.0));
		kit::text_in(&p, clip, says, label(), t.muted);
		if heard {
			let on = if self.vc.drv_headset_sally.unwrap_or(true) { 0 } else { 1 };
			if let (_, Some(i)) = kit::segmented(ui, Id::unique("headset"), sp, CTL_H, &items, on) {
				self.vc.driver_cmd(if i == 0 { "headset sally" } else { "headset valve" });
			}
		} else {
			Self::segmented_off(&p, sp, &items);
		}
		y += 50.0;

		// the recordings
		const KEPT: [(&str, i32); 5] = [("1", 1), ("2", 2), ("5", 5), ("10", 10), ("All", 0)];
		let items = KEPT.map(|(name, _)| name);
		kit::text(&p, Pos2::new(inner.min.x + 2.0, y + 11.0), Align2::LEFT_CENTER, "Sessions kept", body(), t.text);
		let sp = Pos2::new(inner.max.x - kit::segmented_w(&p, CTL_H, &items), y + 4.0);
		let bytes = self.logs_bytes() as f64;
		let size = if bytes >= 1e9 { format!("{:.1} GB", bytes / 1e9) } else { format!("{:.0} MB", bytes / 1e6) };
		let says = format!("Their recordings and logs: {size} now");
		let clip = Rect::from_min_max(Pos2::new(inner.min.x + 2.0, y + 20.0), Pos2::new(sp.x - 8.0, y + 40.0));
		kit::text_in(&p, clip, &says, label(), t.muted);
		let row = Rect::from_min_max(Pos2::new(inner.min.x, y), Pos2::new(sp.x - 8.0, y + 44.0));
		ui.interact(row, Id::unique("keep-sessions-tip"), Sense::hover()).on_hover_text(
			"Every session is recorded: sally calibrates the room from the recordings, and they are what to send when \
			 something goes wrong. The older ones are deleted when SteamVR starts",
		);
		match self.vc.drv_keep_sessions.filter(|_| heard) {
			Some(keep) => {
				let on = KEPT.iter().position(|(_, n)| *n == keep).unwrap_or(usize::MAX);
				if let (_, Some(i)) = kit::segmented(ui, Id::unique("keep-sessions"), sp, CTL_H, &items, on) {
					self.vc.driver_cmd(&format!("keep {}", KEPT[i].1));
				}
			}
			None => Self::segmented_off(&p, sp, &items),
		}
		y += 50.0;

		// the pose log
		kit::text(&p, Pos2::new(inner.min.x + 2.0, y + 11.0), Align2::LEFT_CENTER, "Pose log", body(), t.text);
		let clip = Rect::from_min_max(Pos2::new(inner.min.x + 2.0, y + 20.0), Pos2::new(inner.max.x - 52.0, y + 40.0));
		kit::text_in(&p, clip, "Valve's tracking kept for Replay: about 100 MB an hour", label(), t.muted);
		let row = Rect::from_min_max(Pos2::new(inner.min.x, y), Pos2::new(inner.max.x - 52.0, y + 44.0));
		ui.interact(row, Id::unique("pose-log-tip"), Sense::hover()).on_hover_text(
			"Each session also keeps what Valve's driver tracked and what SteamVR was given, so Replay can show them \
			 beside sally's. It is kept and deleted with the sessions, and starts the moment this is switched on",
		);
		let on = self.vc.drv_pose_log.filter(|_| heard);
		let at = Pos2::new(inner.max.x - 42.0, y + 8.0);
		if kit::switch(ui, Id::unique("pose-log"), at, on == Some(true), on.is_some()) {
			self.vc.driver_cmd(if on == Some(true) { "pose_log off" } else { "pose_log on" });
		}
	}

	/// A setting as a label, its value at the right, and a slider under them; whether it moved
	#[allow(clippy::too_many_arguments)]
	fn slider_row(
		&mut self, ui: &mut egui::Ui, id: Id, r: Rect, text: &str, value: &str, v: &mut f64, min: f64, max: f64,
		step: f64, default: f64,
	) -> bool {
		let t = tk();
		let p = ui.painter().clone();
		kit::text(&p, Pos2::new(r.min.x, r.min.y + 9.0), Align2::LEFT_CENTER, text, body(), t.text);
		kit::text(&p, Pos2::new(r.max.x, r.min.y + 9.0), Align2::RIGHT_CENTER, value, body(), t.soft);
		let sr = Rect::from_min_max(Pos2::new(r.min.x, r.min.y + 19.0), r.max);
		let range = Range { min, max, step, default: Some(default), inherits: None };
		kit::slider(ui, id, sr, v, &range, true).changed
	}

	/// The overlay: the same panel as a tab of SteamVR's dashboard
	fn overlay_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let inner = self.card(ui, r, "In VR");
		let p = ui.painter().clone();
		let hr = Self::header(r);
		if kit::switch(
			ui,
			Id::unique("overlay-switch"),
			Pos2::new(hr.max.x - 48.0, hr.center().y - 12.0),
			self.st.overlay,
			true,
		) {
			self.st.overlay = !self.st.overlay;
			self.st.dirty = true;
			self.toggle_overlay = true;
		}
		// what the overlay says of itself, after the title
		let tw = kit::text_w(&p, "In VR", title());
		let (col, says) = if !self.st.overlay {
			(t.faint, "off".to_string())
		} else if self.vc.overlay_ok {
			(t.good, self.vc.overlay_status.clone())
		} else {
			(t.muted, self.vc.overlay_status.clone())
		};
		let sx = inner.min.x + tw + 14.0;
		p.circle_filled(Pos2::new(sx + 4.0, hr.center().y + 1.0), 3.5, col);
		let clip = Rect::from_min_max(Pos2::new(sx + 14.0, hr.min.y), Pos2::new(hr.max.x - 58.0, hr.max.y));
		kit::text_in(&p, clip, &says, label(), t.muted);

		let sr = Rect::from_min_size(Pos2::new(inner.min.x, inner.min.y + 8.0), Vec2::new(inner.width(), 50.0));
		let mut v = self.st.overlay_width;
		let says = format!("{v:.2} m");
		if self.slider_row(
			ui,
			Id::unique("overlay-width"),
			sr,
			"Width in the dashboard",
			&says,
			&mut v,
			1.0,
			5.0,
			0.05,
			2.5,
		) {
			self.st.overlay_width = v;
			self.st.dirty = true;
		}
		let cy = sr.max.y + 18.0;
		kit::text(
			&p,
			Pos2::new(inner.min.x, cy),
			Align2::LEFT_CENTER,
			"The tab \"sally\" of SteamVR's dashboard",
			label(),
			t.faint,
		);
	}

	/// The devices: each one's name (a click renames it) and where it is worn
	fn worn_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let inner = self.card(ui, r, "Devices");
		// what each row says of its device, before the rows (they change names and roles)
		let codes: Vec<String> = self.active_devices().iter().map(|d| d.codename.clone()).collect();
		let list: Vec<Device> =
			codes.iter().filter_map(|c| self.rt.devices.get_mut(c).map(Device::copy_light)).collect();
		let hr = Self::header(r);
		let count = match list.len() {
			0 => String::new(),
			n => format!("{n} tracking"),
		};
		kit::text(
			ui.painter(),
			Pos2::new(hr.max.x - 6.0, hr.center().y + 1.0),
			Align2::RIGHT_CENTER,
			&count,
			label(),
			t.muted,
		);
		if list.is_empty() {
			let s = if self.rt.connected(self.now) { "No device is tracking" } else { "No runtime" };
			kit::text(ui.painter(), inner.min + Vec2::new(0.0, 4.0), Align2::LEFT_TOP, s, body(), t.muted);
			return;
		}
		const ROW: f32 = 60.0;
		let lr = Rect::from_min_max(inner.min, Pos2::new(inner.max.x + PAD - 2.0, inner.max.y));
		let mut child = ui.new_child(UiBuilder::new().max_rect(lr).id_salt("devices"));
		child.spacing_mut().item_spacing = Vec2::ZERO;
		ScrollArea::vertical().id_salt("devices-scroll").auto_shrink([false, false]).show(&mut child, |ui| {
			let (area, _) = ui.allocate_exact_size(Vec2::new(inner.width(), ROW * list.len() as f32), Sense::hover());
			let p = ui.painter().clone();
			for (i, d) in list.iter().enumerate() {
				let rr = Rect::from_min_size(
					Pos2::new(area.min.x, area.min.y + i as f32 * ROW),
					Vec2::new(area.width(), ROW),
				);
				if i > 0 {
					p.line_segment([rr.left_top(), rr.right_top()], Stroke::new(1.0, fade(t.line, 0.7)));
				}
				let role = self.role_of(d);

				// where it is worn: the button at the right, in words
				let own = self.st.roles.get(&d.serial).filter(|k| *k != "auto").cloned();
				let says = match &own {
					Some(k) => role_def(k).1.to_string(),
					None => format!("Auto: {}", role_def(role).1),
				};
				let bw = (kit::text_w(&p, &says, body()) + 46.0).clamp(130.0, 200.0).min(rr.width() * 0.5);
				let br =
					Rect::from_min_size(Pos2::new(rr.max.x - bw, rr.center().y - CTL_H * 0.5), Vec2::new(bw, CTL_H));
				let open = self.pop == Some((Pop::Role, self.view)) && self.role_pick.as_deref() == Some(&d.serial);
				let can = d.serial != "-" && !d.serial.is_empty();
				let resp =
					ui.interact(br, Id::unique(("role", &d.serial)), if can { Sense::click() } else { Sense::hover() });
				let fill = if open {
					t.ctl_press
				} else if resp.hovered() {
					t.ctl_hover
				} else {
					t.ctl
				};
				p.rect_filled(br, t.ctl_radius, fill);
				let clip =
					Rect::from_min_max(Pos2::new(br.min.x + 12.0, br.min.y), Pos2::new(br.max.x - 26.0, br.max.y));
				p.with_clip_rect(clip.intersect(p.clip_rect())).text(
					Pos2::new(clip.min.x, br.center().y),
					Align2::LEFT_CENTER,
					&says,
					body(),
					if own.is_some() { t.text } else { t.soft },
				);
				kit::icon_at(&p, Pos2::new(br.max.x - 15.0, br.center().y), icons::CARET_DOWN, 13.0, t.muted);
				if resp.clicked() {
					if open {
						self.pop = None;
					} else {
						self.pop = None;
						self.role_pick = Some(d.serial.clone());
						self.toggle_pop(Pop::Role);
					}
				}
				if self.role_pick.as_deref() == Some(&d.serial) {
					let serial = d.serial.clone();
					let auto = self.auto_role(d);
					self.popup(ui, Pop::Role, br, 248.0, true, |s, ui| s.role_pop(ui, &serial, auto));
				}

				// its name, and under it what it is
				let nx = rr.min.x + 4.0;
				let nw = br.min.x - nx - 12.0;
				self.name_field(ui, d, Pos2::new(nx, rr.min.y + 9.0), nw, semibold(15.0), false);
				let kb = self.in_overlay() && self.vc.kb_open && self.vc.kb_tag == d.serial;
				let sub = if kb { "SteamVR's keyboard is open".to_string() } else { self.type_of(d) };
				let y2 = rr.min.y + 42.0;
				let sr = kit::text(&p, Pos2::new(nx, y2), Align2::LEFT_CENTER, &sub, label(), t.muted);
				let id = format!("{}  {}", d.codename, d.serial);
				let clip =
					Rect::from_min_max(Pos2::new(sr.max.x + 10.0, y2 - 10.0), Pos2::new(br.min.x - 10.0, y2 + 10.0));
				kit::text_in(&p, clip, &id, code(), t.faint);
			}
		});
	}

	/// Where a device is worn, picked from the roles by name: the automatic one first (what the device is taken for
	/// without a choice), then each role. The one in use is ticked; a place another device is worn at has that
	/// device's name beside it
	fn role_pop(&mut self, ui: &mut egui::Ui, serial: &str, auto: &'static str) {
		let t = tk();
		super::topbar::pop_title(ui, "Worn on");
		let cur = self.st.roles.get(serial).filter(|k| *k != "auto").cloned();
		// the places other devices are worn at
		let taken: Vec<(&'static str, String)> = self
			.active_devices()
			.iter()
			.filter(|d| d.serial != serial)
			.map(|d| (self.role_of(d), self.short_name(d)))
			.filter(|(k, _)| !matches!(*k, "auto" | "held" | "none"))
			.collect();
		let auto_says = format!("Auto: {}", role_def(auto).1);
		let mut pick: Option<Option<&'static str>> = None;
		ui.spacing_mut().item_spacing.y = 2.0;
		for &(key, name, _) in ROLES {
			let is_auto = key == "auto";
			let on = if is_auto { cur.is_none() } else { cur.as_deref() == Some(key) };
			let (rr, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 28.0), Sense::click());
			let p = ui.painter().clone();
			if resp.hovered() {
				p.rect_filled(rr, t.ctl_radius, t.ctl_hover);
			} else if on {
				p.rect_filled(rr, t.ctl_radius, t.ctl);
			}
			let says = if is_auto { auto_says.as_str() } else { name };
			let col = if on { t.text } else { t.soft };
			kit::text(&p, Pos2::new(rr.min.x + 10.0, rr.center().y), Align2::LEFT_CENTER, says, body(), col);
			if on {
				kit::icon_at(&p, Pos2::new(rr.max.x - 14.0, rr.center().y), icons::CHECK, 15.0, t.on);
			} else if let Some((_, other)) = taken.iter().find(|(k, _)| *k == key) {
				let at = Pos2::new(rr.max.x - 10.0, rr.center().y);
				kit::text(&p, at, Align2::RIGHT_CENTER, other, caption(), t.faint);
			}
			if resp.clicked() {
				pick = Some(if is_auto { None } else { Some(key) });
			}
		}
		if let Some(choice) = pick {
			match choice {
				Some(key) => {
					self.st.roles.insert(serial.to_string(), key.to_string());
				}
				None => {
					self.st.roles.remove(serial);
				}
			}
			self.st.dirty = true;
			self.pop = None;
		}
	}

	/// The advanced things: the station map's lock, the comparison with Valve's driver
	fn advanced_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let inner = self.card(ui, r, "Advanced");
		let p = ui.painter().clone();
		let conn = self.rt.connected(self.now);
		let mut y = inner.min.y + 2.0;

		// the station map
		let locked = self.rt.map_locked;
		kit::text(&p, Pos2::new(inner.min.x + 2.0, y + 11.0), Align2::LEFT_CENTER, "Station map", body(), t.text);
		let says = if locked {
			"Locked: the stations stay where they are"
		} else {
			"Live: re-solved, and written to the config"
		};
		let bl = if locked { "Unlock" } else { "Lock" };
		let bw = kit::button_w(&p, bl, false).max(86.0);
		let br = Rect::from_min_size(Pos2::new(inner.max.x - bw, y + 4.0), Vec2::new(bw, CTL_H));
		let clip = Rect::from_min_max(Pos2::new(inner.min.x + 2.0, y + 20.0), Pos2::new(br.min.x - 8.0, y + 40.0));
		kit::text_in(&p, clip, says, label(), t.muted);
		if kit::button(ui, Id::unique("map-lock"), br, bl, None, Look::Plain, conn).clicked() {
			if locked {
				self.confirm_unlock = true;
			} else {
				self.rt.command("map lock", Want::Nothing);
			}
		}
		y += 50.0;

		// the comparison
		kit::text(
			&p,
			Pos2::new(inner.min.x + 2.0, y + 11.0),
			Align2::LEFT_CENTER,
			"Compare with Valve's driver",
			body(),
			t.text,
		);
		let clip = Rect::from_min_max(Pos2::new(inner.min.x + 2.0, y + 20.0), Pos2::new(inner.max.x - 52.0, y + 40.0));
		kit::text_in(&p, clip, "Each device against Valve's pose of it", label(), t.muted);
		if kit::switch(ui, Id::unique("compare"), Pos2::new(inner.max.x - 42.0, y + 8.0), self.st.compare, true) {
			self.st.compare = !self.st.compare;
			self.vc.enabled = self.st.compare;
			self.st.dirty = true;
		}
	}
}
