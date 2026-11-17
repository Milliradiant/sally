//! The bar across the top: the tabs, and what is wanted from any of them. From the right: what SteamVR is shown
//! (sally's tracking or Valve's: the driver's switch), and two readings that open into what belongs to them: the
//! alignment with Valve (and Re-align), the runtime (and its restart). In a narrow window the readings keep their
//! icons and drop their words.

use egui::{Align2, Color32, Id, Pos2, Rect, Sense, Vec2};

use super::icons;
use super::kit::{self, Look, Pop, Surface};
use super::theme::*;
use super::Panel;

pub const BAR_H: f32 = 54.0;

/// The tabs in the bar's order, each with its number
const TABS: [(&str, i32); 4] = [("Tracking", 0), ("Replay", 1), ("Output", 2), ("Settings", 3)];

/// Seconds as m:ss, or h:mm:ss from an hour
pub fn clock(s: f64) -> String {
	let s = s.max(0.0) as u64;
	if s >= 3600 {
		format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
	} else {
		format!("{}:{:02}", s / 60, s % 60)
	}
}

/// A key and its value on a line of a pop-up (the value on as many lines as it takes)
pub fn kv(ui: &mut egui::Ui, key: &str, value: &str, color: Color32) {
	let t = tk();
	let w = ui.available_width();
	let key_w = 96.0;
	let g = ui.painter().layout(value.to_string(), label(), color, (w - key_w).max(40.0));
	let h = g.size().y.max(18.0) + 4.0;
	let (r, _) = ui.allocate_exact_size(Vec2::new(w, h), Sense::hover());
	let p = ui.painter();
	p.text(Pos2::new(r.min.x, r.min.y + 11.0), Align2::LEFT_CENTER, key, label(), t.muted);
	p.galley(Pos2::new(r.min.x + key_w, r.min.y + 2.0), g, color);
}

/// A pop-up's title
pub fn pop_title(ui: &mut egui::Ui, s: &str) {
	ui.label(egui::RichText::new(s).font(title()).color(tk().text));
	ui.add_space(2.0);
}

impl Panel {
	pub(super) fn top_bar(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Bar);
		let cy = r.center().y;

		let wr =
			kit::text(&p, Pos2::new(r.min.x + 16.0, cy - 0.5), Align2::LEFT_CENTER, "sally", semibold(19.0), t.text);
		let version = self.version.clone();
		kit::text(&p, Pos2::new(wr.max.x + 6.0, wr.max.y - 3.0), Align2::LEFT_BOTTOM, &version, caption(), t.faint);

		// the tabs
		let mut x = wr.max.x + 52.0;
		let flagged = self.flagged_count() > 0;
		for (name, tab) in TABS {
			let tw = kit::text_w(&p, name, body());
			let w = tw + 28.0;
			let tr = Rect::from_min_size(Pos2::new(x, cy - 17.0), Vec2::new(w, 34.0));
			// the whole bar's height is the target
			let hit = Rect::from_min_max(Pos2::new(tr.min.x, r.min.y), Pos2::new(tr.max.x, r.max.y));
			let resp = ui.interact(hit, Id::unique(("tab", tab)), Sense::click());
			let on = self.st.tab == tab;
			if resp.clicked() {
				if on && tab == 0 {
					// Tracking again: back from the device page
					self.page = None;
				}
				self.st.tab = tab;
				self.st.dirty = true;
				self.pop = None;
			}
			let h = ui.ctx().animate_bool_with_time(Id::unique(("tabh", tab)), resp.hovered(), 0.1);
			let cr = t.ctl_radius;
			if on {
				p.rect_filled(tr, cr, t.ctl);
			} else {
				p.rect_filled(tr, cr, fade(t.ctl, h));
			}
			let col = if on { t.text } else { mix(t.muted, t.soft, h) };
			kit::text(&p, tr.center(), Align2::CENTER_CENTER, name, body(), col);
			if tab == 0 && flagged {
				p.circle_filled(Pos2::new(tr.max.x - 8.0, tr.min.y + 8.0), 3.5, t.warn);
			}
			x += w + 4.0;
		}
		let left_end = x + 14.0;

		let conn = self.rt.connected(self.now);
		let bh = 34.0;
		let mut right = r.max.x - 14.0;

		// what SteamVR is shown: sally's tracking or Valve's
		let hook = self.vc.hook_mode && self.st.compare;
		let showing_sally = self.vc.hook_seen == "sally";
		let sw = kit::segmented_w(&p, bh, &["sally", "Valve"]);
		let sp = Pos2::new(right - sw, cy - bh * 0.5);
		if hook {
			let (sr, clicked) =
				kit::segmented(ui, Id::unique("seen"), sp, bh, &["sally", "Valve"], if showing_sally { 0 } else { 1 });
			if let Some(i) = clicked {
				self.vc.driver_cmd(if i == 0 { "seen sally" } else { "seen valve" });
			}
			if showing_sally && !self.vc.hook_fit {
				// sally chosen, Valve still shown until the driver has lined the two up
				p.circle_filled(Pos2::new(sr.min.x + 9.0, sr.min.y + 9.0), 3.5, t.warn);
			}
			let tip = if !showing_sally {
				"SteamVR is shown Valve's tracking for every device"
			} else if self.vc.hook_fit {
				"SteamVR is shown sally's tracking (Valve's where sally has none)"
			} else {
				"sally is lining up with Valve's frame: SteamVR is shown Valve's until it has"
			};
			ui.interact(sr, Id::unique("seen-tip"), Sense::hover()).on_hover_text(tip);
		} else {
			// no word from the driver: the switch is there, and off
			let sr = Rect::from_min_size(sp, Vec2::new(sw, bh));
			p.rect_filled(sr, t.ctl_radius, fade(t.ctl, 0.6));
			let half = sw * 0.5;
			for (i, s) in ["sally", "Valve"].into_iter().enumerate() {
				let c = Pos2::new(sr.min.x + half * (i as f32 + 0.5), cy);
				p.text(c, Align2::CENTER_CENTER, s, label(), t.faint);
			}
			ui.interact(sr, Id::unique("seen-tip"), Sense::hover())
				.on_hover_text("No word from the sally driver: SteamVR is not running, or sally is off in it");
		}
		right = sp.x - 14.0;

		// the readings: the alignment with Valve, the runtime
		let (rt_col, rt_text, rt_icon) = if conn {
			(t.good, "Runtime".to_string(), icons::PLUGS_CONNECTED)
		} else {
			(t.bad, "No runtime".to_string(), icons::PLUGS)
		};
		let align = self.st.compare.then(|| {
			let refitting = self.vc.fit_from_driver && self.vc.drv_fit.is_some_and(|f| f.refitting);
			if refitting {
				let n = self.vc.drv_fit.map_or(0, |f| f.samples);
				(t.warn, format!("Aligning {n}/150"))
			} else if self.vc.fit_ok {
				(t.good, format!("Aligned {:.1} mm", self.vc.fit_median * 1000.0))
			} else if self.vc.running {
				(t.warn, "Aligning".to_string())
			} else {
				(t.faint, "No SteamVR".to_string())
			}
		});
		let chip_w = |s: &str, words: bool| if words { kit::text_w(&p, s, label()) + 44.0 } else { 34.0 };
		let full = chip_w(&rt_text, true) + align.as_ref().map_or(0.0, |a| chip_w(&a.1, true) + 4.0);
		let words = right - full > left_end;
		let chip = |s: &mut Panel, ui: &mut egui::Ui, right: f32, which: Pop, glyph: &str, col: Color32, text: &str| {
			let w = chip_w(text, words);
			let cr = Rect::from_min_size(Pos2::new(right - w, cy - bh * 0.5), Vec2::new(w, bh));
			let open = s.pop == Some((which, s.view));
			let resp = kit::button(ui, Id::unique(("reading", which as u8)), cr, "", None, Look::Ghost, true);
			if open {
				p.rect_filled(cr, t.ctl_radius, t.ctl);
			}
			kit::icon_at(&p, Pos2::new(cr.min.x + 17.0, cy), glyph, 17.0, col);
			if words {
				kit::text(&p, Pos2::new(cr.min.x + 32.0, cy), Align2::LEFT_CENTER, text, label(), t.soft);
			}
			if resp.clicked() {
				s.toggle_pop(which);
			}
			cr
		};
		let rt_r = chip(self, ui, right, Pop::Runtime, rt_icon, rt_col, &rt_text);
		let al_r = align
			.as_ref()
			.map(|(col, text)| chip(self, ui, rt_r.min.x - 4.0, Pop::Align, icons::CROSSHAIR, *col, text));

		self.popup(ui, Pop::Runtime, rt_r, 330.0, true, |s, ui| {
			let t = tk();
			pop_title(ui, "Runtime");
			let conn = s.rt.connected(s.now);
			if conn {
				kv(ui, "Address", &format!("{}:{}", s.rt.host, s.rt.port), t.text);
				kv(ui, "Running", &clock(s.rt.run_time), t.text);
				kv(
					ui,
					"Messages",
					&format!("{:.1}k a second, protocol {}", s.rt.rx_rate / 1000.0, s.rt.protocol),
					t.text,
				);
				let (n, all) = (s.active_devices().len(), s.rt.devices.len());
				kv(ui, "Devices", &format!("{n} active of {all} listed"), t.text);
				kv(ui, "Map", if s.rt.map_locked { "locked" } else { "live" }, t.text);
			} else {
				kv(ui, "Address", &format!("{}:{}", s.rt.host, s.rt.port), t.text);
				kv(ui, "State", "not answering", t.bad);
			}
			ui.add_space(8.0);
			let (br, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), kit::CTL_H), Sense::hover());
			let resp = kit::button(
				ui,
				Id::unique("pop-restart"),
				br,
				"Restart the runtime",
				Some(icons::ARROWS_CLOCKWISE),
				Look::Plain,
				conn,
			);
			if resp.clicked() {
				s.confirm_restart = true;
				s.pop = None;
			}
		});
		if let Some(al_r) = al_r {
			self.popup(ui, Pop::Align, al_r, 360.0, true, |s, ui| {
				let t = tk();
				pop_title(ui, "SteamVR and Valve's tracking");
				let status = s.vc.status.clone();
				kv(ui, "SteamVR", &status, t.text);
				if s.vc.hook_mode {
					let (text, col) = if s.vc.hook_seen == "sally" && s.vc.hook_fit {
						(format!("sally, fit {:.1} mm", s.vc.hook_fit_mm), t.good)
					} else if s.vc.hook_seen == "sally" {
						("Valve, while sally lines up".to_string(), t.warn)
					} else {
						("Valve".to_string(), t.warn)
					};
					kv(ui, "In VR", &text, col);
				}
				let fit = match s.vc.drv_fit {
					Some(f) if s.vc.fit_from_driver && f.refitting => {
						format!("re-aligning, {} of 150 pairs", f.samples)
					}
					_ if s.vc.fit_from_driver => format!("SteamVR's own, median {:.1} mm", s.vc.fit_median * 1000.0),
					_ if s.vc.fit_ok => format!("{} pairs, median {:.1} mm", s.vc.fit_n, s.vc.fit_median * 1000.0),
					_ => "not aligned yet".to_string(),
				};
				kv(ui, "Alignment", &fit, t.text);
				let ov = s.vc.overlay_status.clone();
				kv(ui, "Overlay", &ov, if s.vc.overlay_ok { t.good } else { t.text });
				ui.add_space(8.0);
				let (br, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), kit::CTL_H), Sense::hover());
				let resp = kit::button(
					ui,
					Id::unique("pop-realign"),
					br,
					"Re-align to Valve",
					Some(icons::CROSSHAIR),
					Look::Plain,
					s.st.compare,
				);
				if resp.clicked() {
					s.vc.refit();
				}
			});
		}
	}
}

#[cfg(test)]
mod tests {
	#[test]
	fn a_clock_reads_as_minutes_then_hours() {
		assert_eq!(super::clock(0.4), "0:00");
		assert_eq!(super::clock(75.9), "1:15");
		assert_eq!(super::clock(3725.0), "1:02:05");
		assert_eq!(super::clock(-3.0), "0:00");
	}
}
