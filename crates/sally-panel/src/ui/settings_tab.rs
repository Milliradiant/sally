//! The Settings tab: the devices' names and where each is worn (every place a button, the one in use lit), the
//! SteamVR overlay, what the driver does that is the user's to choose (whose tracking a lighthouse headset gets, how
//! many sessions' recordings stay, whether a session keeps a pose log), and the advanced things: the station map's
//! lock and the comparison with Valve's driver. A setting is a ruled row: what it is and what it does at the left,
//! its control at the right. Last, the wallpaper: a picture of the user's own behind the panel, which makes glass
//! of the boxes.

use egui::{Align2, Id, Pos2, Rect, ScrollArea, Sense, Stroke, UiBuilder, Vec2};

use super::kit::{self, Col, Look, Range, Table, CTL_H, GAP, HEADER_H, PAD, TR_H};
use super::theme::*;
use super::wallpaper;
use super::{role_def, Panel, ROLES};
use crate::runtime::{Device, Want};

/// A setting's row
const ROW: f32 = 54.0;
/// The wallpaper's box: its row of pictures, and the line under it
const WALL_STRIP: f32 = 76.0;
const WALL_SAYS: f32 = 26.0;

/// A setting's row in `r`: its name, what it does under it, a line under the row. `ctl_w`: the room its control
/// takes at the right
fn setting(p: &egui::Painter, r: Rect, name: &str, says: &str, ctl_w: f32) {
	let t = tk();
	kit::text(p, Pos2::new(r.min.x, r.min.y + 17.0), Align2::LEFT_CENTER, name, body(), t.text);
	let clip = Rect::from_min_max(Pos2::new(r.min.x, r.min.y + 26.0), Pos2::new(r.max.x - ctl_w - 10.0, r.max.y - 6.0));
	kit::text_in(p, clip, says, label(), t.muted);
	p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.line));
}

impl Panel {
	pub(super) fn settings_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		// the devices and where the selected one is worn at the left; at the right the overlay's box over the
		// driver's choices and the advanced things
		let w = r.width();
		let c2 = (w * 0.4).clamp(400.0, 640.0);
		let c1 = w - c2 - GAP;
		let x2 = r.min.x + c1 + GAP;
		let roles_h = HEADER_H + 8.0 + 4.0 * (CTL_H + 6.0) + PAD - 6.0;
		let overlay_h = HEADER_H + 2.0 * ROW + 2.0;
		let choices_h = HEADER_H + 3.0 * ROW + 2.0;
		let advanced_h = HEADER_H + 2.0 * ROW + 2.0;
		// the wallpaper's box: under the right column's boxes where there is room, else over the roles
		let wall_h = HEADER_H + WALL_STRIP + WALL_SAYS + ROW + 2.0;
		let right_used = overlay_h + choices_h + advanced_h + 3.0 * GAP;
		let wall_right = r.height() >= right_used + wall_h;
		let left_foot = roles_h + if wall_right { 0.0 } else { wall_h + GAP };
		let devices = Rect::from_min_max(r.min, Pos2::new(r.min.x + c1, r.max.y - left_foot - GAP));
		self.worn_card(ui, devices);
		if !wall_right {
			let top = devices.max.y + GAP;
			self.wallpaper_card(ui, Rect::from_min_max(Pos2::new(r.min.x, top), Pos2::new(r.min.x + c1, top + wall_h)));
		}
		self.roles_card(
			ui,
			Rect::from_min_max(Pos2::new(r.min.x, r.max.y - roles_h), Pos2::new(r.min.x + c1, r.max.y)),
		);
		let mut y = r.min.y;
		self.overlay_card(ui, Rect::from_min_max(Pos2::new(x2, y), Pos2::new(r.max.x, y + overlay_h)));
		y += overlay_h + GAP;
		self.choices_card(ui, Rect::from_min_max(Pos2::new(x2, y), Pos2::new(r.max.x, y + choices_h)));
		y += choices_h + GAP;
		self.advanced_card(ui, Rect::from_min_max(Pos2::new(x2, y), Pos2::new(r.max.x, (y + advanced_h).min(r.max.y))));
		if wall_right {
			y += advanced_h + GAP;
			self.wallpaper_card(ui, Rect::from_min_max(Pos2::new(x2, y), Pos2::new(r.max.x, y + wall_h)));
		}
	}

	/// The wallpaper: the pictures of the folder as a row of small copies (the first place is none: the panel flat),
	/// the folder itself from the title line, and how the glass over the picture is made: how much the picture is
	/// blurred under a box, and how dark the glass is
	fn wallpaper_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		self.card(ui, r, "Wallpaper");
		let inside = Self::under_header(r);
		let dir = wallpaper::folder(&self.st.path);
		let view = self.view.index();
		let hr = Self::header(r);
		let hint = "The folder the pictures are in: one put there is offered here";
		let items = [("Open folder", Look::Plain, !self.in_overlay(), hint)];
		let (clicked, left) =
			kit::buttons_right(ui, Id::unique("wallpaper-folder"), hr.max.x - 3.0, hr.center().y, 26.0, &items);
		if clicked.is_some() {
			let _ = std::fs::create_dir_all(&dir);
			crate::check::open(&dir, false);
		}
		// the folder's scan is file I/O: not every frame
		if self.now - self.wall_files_t > 2.0 || self.now < self.wall_files_t {
			self.wall_files = wallpaper::files(&dir);
			self.wall_files_t = self.now;
			let files = self.wall_files.clone();
			self.wall.keep_thumbs(&files);
		}
		let files = self.wall_files.clone();
		let p = ui.painter().clone();
		// what is chosen, at the title line's right: its name, or why it is not there
		let (says, col) = match (&self.wall.error, self.st.wallpaper.as_str()) {
			(Some(e), _) => (e.clone(), t.warn),
			(None, "") => (String::new(), t.muted),
			(None, name) if self.wall.busy() && !self.wall.shown() => (format!("reading {name}"), t.muted),
			(None, name) => (name.to_string(), t.muted),
		};
		let tw = kit::text_w(&p, "Wallpaper", title());
		let clip = Rect::from_min_max(Pos2::new(r.min.x + PAD + tw + 16.0, hr.min.y), Pos2::new(left - 10.0, hr.max.y));
		let pc = p.with_clip_rect(clip.intersect(p.clip_rect()));
		kit::text(&pc, Pos2::new(clip.max.x, hr.center().y), Align2::RIGHT_CENTER, &says, label(), col);

		// the pictures: a row that scrolls where there are more than fit
		let (tw, th, gap) = (96.0, 54.0, 8.0);
		let strip = Rect::from_min_max(
			Pos2::new(inside.min.x + PAD, inside.min.y),
			Pos2::new(inside.max.x - PAD, inside.min.y + WALL_STRIP),
		);
		let n = files.len() + 1;
		let mut pick: Option<String> = None;
		let mut child = ui.new_child(UiBuilder::new().max_rect(strip).id_salt("wallpapers"));
		child.spacing_mut().item_spacing = Vec2::ZERO;
		ScrollArea::horizontal().id_salt("wallpapers-scroll").auto_shrink([false, false]).show(&mut child, |ui| {
			let total = n as f32 * (tw + gap) - gap + 4.0;
			let (area, _) =
				ui.allocate_exact_size(Vec2::new(total.max(strip.width()), WALL_STRIP - 12.0), Sense::hover());
			let p = ui.painter().clone();
			for i in 0..n {
				let tr = Rect::from_min_size(
					Pos2::new(area.min.x + 2.0 + i as f32 * (tw + gap), area.min.y + 10.0),
					Vec2::new(tw, th),
				);
				if !ui.is_rect_visible(tr) {
					continue;
				}
				let name = if i == 0 { "" } else { files[i - 1].as_str() };
				let resp = ui.interact(tr, Id::unique(("wallpaper-pick", i)), Sense::click());
				let on = self.st.wallpaper == name;
				let cr = egui::CornerRadius::same(t.ctl_radius as u8);
				if i == 0 {
					p.rect_filled(tr, cr, Tk::default().card);
					p.text(tr.center(), Align2::CENTER_CENTER, "None", label(), Tk::default().soft);
				} else {
					match self.wall.thumb(ui.ctx(), view, &dir, name) {
						Some((tex, size)) => {
							let uv = wallpaper::cover_uv([size[0] as f32, size[1] as f32], tr, tr);
							p.add(egui::epaint::RectShape::filled(tr, cr, egui::Color32::WHITE).with_texture(tex, uv));
						}
						None => {
							p.rect_filled(tr, cr, t.ctl);
							ui.ctx().request_repaint();
						}
					}
				}
				let edge = if on {
					Stroke::new(2.0, t.accent)
				} else if resp.hovered() {
					Stroke::new(1.0, t.soft)
				} else {
					Stroke::new(1.0, t.group)
				};
				p.rect_stroke(tr, cr, edge, egui::StrokeKind::Inside);
				let resp = if i == 0 {
					resp.on_hover_text("No picture: the panel in black and greys")
				} else {
					resp.on_hover_text(name)
				};
				if resp.clicked() && !on {
					pick = Some(name.to_string());
				}
			}
		});
		if let Some(name) = pick {
			self.st.wallpaper = name;
			self.st.dirty = true;
		}
		// how to get one in, under the row
		let says =
			Rect::from_min_max(Pos2::new(strip.min.x, strip.max.y), Pos2::new(strip.max.x, strip.max.y + WALL_SAYS));
		let how = if self.in_overlay() {
			"A picture put in the panel's wallpapers folder is offered here (png, jpg, bmp)"
		} else {
			"A picture dropped on this window, or put in the folder, is offered here (png, jpg, bmp)"
		};
		kit::text_in(&p, says, how, label(), t.muted);
		p.line_segment([says.left_bottom(), says.right_bottom()], Stroke::new(1.0, t.line));

		// the glass: the two things that make it, side by side
		let on = !self.st.wallpaper.is_empty();
		let row = Rect::from_min_max(Pos2::new(says.min.x, says.max.y), Pos2::new(says.max.x, says.max.y + ROW));
		let half = (row.width() - 2.0 * PAD) * 0.5;
		let (lo, hi, def) = wallpaper::BLUR;
		let blur = Range { min: lo, max: hi, step: 1.0, default: Some(def), inherits: None };
		let (lo, hi, def) = wallpaper::DARK;
		let dark = Range { min: lo, max: hi, step: 2.0, default: Some(def), inherits: None };
		let mut values = [self.st.glass_blur, self.st.glass_dark];
		for (i, (name, range, unit)) in [("Blur", blur, ""), ("Darkness", dark, " %")].into_iter().enumerate() {
			let x0 = row.min.x + i as f32 * (half + 2.0 * PAD);
			let cell = Rect::from_min_max(Pos2::new(x0, row.min.y), Pos2::new(x0 + half, row.max.y));
			let ink = if on { t.text } else { t.faint };
			kit::text(&p, Pos2::new(cell.min.x, cell.min.y + 17.0), Align2::LEFT_CENTER, name, body(), ink);
			let shown = format!("{:.0}{unit}", values[i]);
			kit::text(
				&p,
				Pos2::new(cell.min.x, cell.min.y + 37.0),
				Align2::LEFT_CENTER,
				&shown,
				num(),
				if on { t.soft } else { t.faint },
			);
			let sr = Rect::from_min_max(
				Pos2::new(cell.min.x + 84.0, cell.min.y + 10.0),
				Pos2::new(cell.max.x, cell.max.y - 10.0),
			);
			if kit::slider(ui, Id::unique(("glass", i)), sr, &mut values[i], &range, on).changed {
				self.st.dirty = true;
			}
		}
		[self.st.glass_blur, self.st.glass_dark] = values;
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
		self.card(ui, r, "Headset and recordings");
		let body = Self::under_header(r);
		let p = ui.painter().clone();
		let heard = self.vc.driver_heard(self.now);
		let row = |k: usize| {
			Rect::from_min_size(
				Pos2::new(body.min.x + PAD, body.min.y + k as f32 * ROW),
				Vec2::new(body.width() - 2.0 * PAD, ROW),
			)
		};

		// the headset
		let items = ["sally", "Valve"];
		let rr = row(0);
		let cw = kit::segmented_w(&p, CTL_H, &items);
		let says = if !heard {
			"Chosen while SteamVR runs with sally"
		} else if self.vc.drv_lighthouse_headset {
			"Whose tracking this headset gets while SteamVR is shown sally's"
		} else {
			"For an Index or Vive headset: this one is not tracked by the base stations"
		};
		setting(&p, rr, "Headset tracking", says, cw);
		let sp = Pos2::new(rr.max.x - cw, rr.center().y - CTL_H * 0.5);
		if heard {
			let on = if self.vc.drv_headset_sally.unwrap_or(true) { 0 } else { 1 };
			if let (_, Some(i)) = kit::segmented(ui, Id::unique("headset"), sp, CTL_H, &items, on) {
				self.vc.driver_cmd(if i == 0 { "headset sally" } else { "headset valve" });
			}
		} else {
			kit::segmented_off(&p, sp, CTL_H, &items);
		}

		// the recordings
		const KEPT: [(&str, i32); 5] = [("1", 1), ("2", 2), ("5", 5), ("10", 10), ("All", 0)];
		let items = KEPT.map(|(name, _)| name);
		let rr = row(1);
		let cw = kit::segmented_w(&p, CTL_H, &items);
		let bytes = self.logs_bytes() as f64;
		let size = if bytes >= 1e9 { format!("{:.1} GB", bytes / 1e9) } else { format!("{:.0} MB", bytes / 1e6) };
		setting(&p, rr, "Sessions kept", &format!("Their recordings and logs: {size} now"), cw);
		let sp = Pos2::new(rr.max.x - cw, rr.center().y - CTL_H * 0.5);
		let tip = Rect::from_min_max(rr.min, Pos2::new(sp.x - 8.0, rr.max.y));
		ui.interact(tip, Id::unique("keep-sessions-tip"), Sense::hover()).on_hover_text(
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
			None => kit::segmented_off(&p, sp, CTL_H, &items),
		}

		// the pose log
		let rr = row(2);
		setting(&p, rr, "Pose log", "Valve's tracking kept for Replay: about 100 MB an hour", kit::TICK);
		let tip = Rect::from_min_max(rr.min, Pos2::new(rr.max.x - kit::TICK - 10.0, rr.max.y));
		ui.interact(tip, Id::unique("pose-log-tip"), Sense::hover()).on_hover_text(
			"Each session also keeps what Valve's driver tracked and what SteamVR was given, so Replay can show them \
			 beside sally's. It is kept and deleted with the sessions, and starts the moment this is switched on",
		);
		let on = self.vc.drv_pose_log.filter(|_| heard);
		let at = Pos2::new(rr.max.x - kit::TICK, rr.center().y - kit::TICK * 0.5);
		if kit::tick(ui, Id::unique("pose-log"), at, on == Some(true), on.is_some()) {
			self.vc.driver_cmd(if on == Some(true) { "pose_log off" } else { "pose_log on" });
		}
	}

	/// The overlay: the same panel as a tab of SteamVR's dashboard
	fn overlay_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		self.card(ui, r, "In VR");
		let body = Self::under_header(r);
		let p = ui.painter().clone();
		let row = |k: usize| {
			Rect::from_min_size(
				Pos2::new(body.min.x + PAD, body.min.y + k as f32 * ROW),
				Vec2::new(body.width() - 2.0 * PAD, ROW),
			)
		};
		// the switch, and what the overlay says of itself
		let rr = row(0);
		let says = if !self.st.overlay { "off".to_string() } else { self.vc.overlay_status.clone() };
		setting(&p, rr, "The tab \"sally\" of SteamVR's dashboard", &says, kit::TICK);
		let at = Pos2::new(rr.max.x - kit::TICK, rr.center().y - kit::TICK * 0.5);
		if kit::tick(ui, Id::unique("overlay-switch"), at, self.st.overlay, true) {
			self.st.overlay = !self.st.overlay;
			self.st.dirty = true;
			self.toggle_overlay = true;
		}
		// its width
		let rr = row(1);
		let sw = (rr.width() * 0.5).min(300.0);
		let mut v = self.st.overlay_width;
		setting(&p, rr, "Width in the dashboard", "", sw);
		kit::text(&p, Pos2::new(rr.min.x, rr.min.y + 37.0), Align2::LEFT_CENTER, &format!("{v:.2} m"), num(), t.soft);
		let sr = Rect::from_min_max(Pos2::new(rr.max.x - sw, rr.min.y + 10.0), Pos2::new(rr.max.x, rr.max.y - 10.0));
		let range = Range { min: 1.0, max: 5.0, step: 0.05, default: Some(2.5), inherits: None };
		if kit::slider(ui, Id::unique("overlay-width"), sr, &mut v, &range, true).changed {
			self.st.overlay_width = v;
			self.st.dirty = true;
		}
	}

	/// The device the roles' box is for: the one picked in the list, else the first
	fn role_device(&self) -> Option<String> {
		let list = self.active_devices();
		self.role_pick
			.as_ref()
			.filter(|s| list.iter().any(|d| d.serial == **s))
			.cloned()
			.or_else(|| list.first().map(|d| d.serial.clone()))
	}

	/// The devices: each one's name, what it is, and where it is worn. A click picks the one the roles under the
	/// list are for; Rename, over the list, renames it
	fn worn_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		self.card(ui, r, "Devices");
		// what each row says of its device, before the rows (they change names and roles)
		let codes: Vec<String> = self.active_devices().iter().map(|d| d.codename.clone()).collect();
		let list: Vec<Device> =
			codes.iter().filter_map(|c| self.rt.devices.get_mut(c).map(Device::copy_light)).collect();
		let picked = self.role_device();
		let hr = Self::header(r);
		let items = [("Rename", Look::Plain, picked.is_some(), "Its name in the panel")];
		let (clicked, left) =
			kit::buttons_right(ui, Id::unique("worn-rename"), hr.max.x - 3.0, hr.center().y, 26.0, &items);
		if let Some(d) = list.iter().find(|d| Some(&d.serial) == picked.as_ref()) {
			let mut says = self.short_name(d);
			if self.in_overlay() && self.vc.kb_open && self.vc.kb_tag == d.serial {
				says = "SteamVR's keyboard is open".into();
			}
			let at = Pos2::new(left - 10.0, hr.center().y);
			kit::text(ui.painter(), at, Align2::RIGHT_CENTER, &says, label(), t.soft);
			if clicked.is_some() {
				self.begin_rename(d);
			}
		}
		let body = Self::under_header(r);
		let p = ui.painter().clone();
		let cols = [
			Col::text("Device", -3.0),
			Col::text("Type", -3.0),
			Col::text("Codename", 86.0),
			Col::text("Serial", 132.0),
			Col::text("Worn on", -2.0).sep(),
		];
		let table = Table::new(body.min.x, body.width(), &cols);
		let y0 = table.header(&p, body.min.y, &cols);
		let lr = Rect::from_min_max(Pos2::new(body.min.x, y0), body.max);
		let mut child = ui.new_child(UiBuilder::new().max_rect(lr).id_salt("devices"));
		child.spacing_mut().item_spacing = Vec2::ZERO;
		let mut pick = None;
		ScrollArea::vertical().id_salt("devices-scroll").auto_shrink([false, false]).show(&mut child, |ui| {
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
			for (k, d) in list.iter().enumerate() {
				let (rr, resp) = ui.allocate_exact_size(Vec2::new(table.width, TR_H), Sense::click());
				let hv = ui.ctx().animate_bool_with_time(Id::unique(("wornh", &d.serial)), resp.hovered(), 0.08);
				kit::row_ground(ui.painter(), rr, k % 2 == 1, picked.as_deref() == Some(d.serial.as_str()), hv);
				if resp.clicked() {
					pick = Some(d.serial.clone());
				}
				let y = rr.min.y;
				let mut name_r = table.cell(0, y, TR_H);
				name_r.min.x += 4.0;
				self.name_cell(ui, d, name_r, semibold(14.0));
				let p = ui.painter();
				table.put(p, 1, y, &self.type_of(d), label(), t.kind(&d.subtype));
				table.put(p, 2, y, &d.codename, code(), t.soft);
				table.put(p, 3, y, &d.serial, code(), t.soft);
				// where it is worn, in words: its own choice, or what it is taken for without one
				let own = self.st.roles.get(&d.serial).filter(|k| *k != "auto");
				let says = match own {
					Some(k) => role_def(k).1.to_string(),
					None => format!("Auto: {}", role_def(self.role_of(d)).1),
				};
				table.put(p, 4, y, &says, label(), if own.is_some() { t.text } else { t.soft });
				last_y = rr.max.y;
			}
			table.seps(ui.painter(), y0, last_y);
		});
		if let Some(s) = pick {
			self.role_pick = Some(s);
		}
	}

	/// Where the picked device is worn: every place a button (the automatic one first: what the device is taken for
	/// without a choice), the one in use lit. A place another device is worn at has that device's name on it. The
	/// place is for the name and the order
	fn roles_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let Some(serial) = self.role_device() else {
			self.card(ui, r, "Worn on");
			return;
		};
		let found =
			self.active_devices().iter().find(|d| d.serial == serial).map(|d| (self.auto_role(d), self.short_name(d)));
		let Some((auto, name)) = found else { return };
		let inner = self.boxed(ui, r, "Worn on", &name, Some(t.soft));
		let cur = self.st.roles.get(&serial).filter(|k| *k != "auto").cloned();
		// the places other devices are worn at
		let taken: Vec<(&'static str, String)> = self
			.active_devices()
			.iter()
			.filter(|o| o.serial != serial)
			.map(|o| (self.role_of(o), self.short_name(o)))
			.filter(|(k, _)| !matches!(*k, "auto" | "held" | "none"))
			.collect();
		let auto_says = format!("Auto: {}", role_def(auto).1);
		let can = serial != "-" && !serial.is_empty();
		let per_row = 4;
		let gap = 6.0;
		let bw = (inner.width() - gap * (per_row as f32 - 1.0)) / per_row as f32;
		let mut pick: Option<Option<&'static str>> = None;
		for (i, &(key, name, _)) in ROLES.iter().enumerate() {
			let is_auto = key == "auto";
			let on = if is_auto { cur.is_none() } else { cur.as_deref() == Some(key) };
			let (col, row) = ((i % per_row) as f32, (i / per_row) as f32);
			let br = Rect::from_min_size(
				Pos2::new(inner.min.x + col * (bw + gap), inner.min.y + row * (CTL_H + gap)),
				Vec2::new(bw, CTL_H),
			);
			let says = if is_auto { auto_says.as_str() } else { name };
			// (a device named as the place is, as one without a name of its own is, says nothing more)
			let other = taken
				.iter()
				.find(|(k, _)| *k == key)
				.map(|(_, n)| n.as_str())
				.filter(|n| !on && !is_auto && *n != name);
			// the place's name from the left, the device already worn there from the right
			let look = if on { Look::Primary } else { Look::Plain };
			let resp = kit::button(ui, Id::unique(("role", key)), br, "", None, look, can);
			let p = ui.painter();
			let fg = if !can {
				t.faint
			} else if on {
				t.on_ink
			} else {
				t.text
			};
			let mut right = br.max.x - 10.0;
			if let Some(o) = other {
				let max = Rect::from_min_max(Pos2::new(br.center().x, br.min.y), Pos2::new(right, br.max.y));
				right = kit::text_right(p, max, o, caption(), t.muted).min.x - 6.0;
			}
			let clip = Rect::from_min_max(Pos2::new(br.min.x + 10.0, br.min.y), Pos2::new(right, br.max.y));
			kit::text_in(p, clip, says, label(), fg);
			if resp.clicked() && !on {
				pick = Some(if is_auto { None } else { Some(key) });
			}
		}
		if let Some(choice) = pick {
			match choice {
				Some(key) => {
					self.st.roles.insert(serial.clone(), key.to_string());
				}
				None => {
					self.st.roles.remove(&serial);
				}
			}
			self.st.dirty = true;
		}
	}

	/// The advanced things: the station map's lock, the comparison with Valve's driver
	fn advanced_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		self.card(ui, r, "Advanced");
		let body = Self::under_header(r);
		let p = ui.painter().clone();
		let conn = self.rt.connected(self.now);
		let row = |k: usize| {
			Rect::from_min_size(
				Pos2::new(body.min.x + PAD, body.min.y + k as f32 * ROW),
				Vec2::new(body.width() - 2.0 * PAD, ROW),
			)
		};

		// the station map
		let rr = row(0);
		let locked = self.rt.map_locked;
		let says = if locked {
			"Locked: the stations stay where they are"
		} else {
			"Live: re-solved, and written to the config"
		};
		let bl = if locked { "Unlock" } else { "Lock" };
		let bw = kit::button_w(&p, bl, false).max(86.0);
		setting(&p, rr, "Station map", says, bw);
		let br = Rect::from_min_size(Pos2::new(rr.max.x - bw, rr.center().y - CTL_H * 0.5), Vec2::new(bw, CTL_H));
		if kit::button(ui, Id::unique("map-lock"), br, bl, None, Look::Plain, conn).clicked() {
			if locked {
				self.confirm_unlock = true;
			} else {
				self.rt.command("map lock", Want::Nothing);
			}
		}

		// the comparison
		let rr = row(1);
		setting(&p, rr, "Compare with Valve's driver", "Each device against Valve's pose of it", kit::TICK);
		let at = Pos2::new(rr.max.x - kit::TICK, rr.center().y - kit::TICK * 0.5);
		if kit::tick(ui, Id::unique("compare"), at, self.st.compare, true) {
			self.st.compare = !self.st.compare;
			self.vc.enabled = self.st.compare;
			self.st.dirty = true;
		}
	}
}
