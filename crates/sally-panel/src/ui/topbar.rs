//! The bar across the top and the status line across the bottom. The bar: the tabs, what sally is doing while its
//! tracking is not what SteamVR is shown (state.rs's line, on up to two lines between the tabs and the switch), and
//! what SteamVR is shown (sally's tracking or Valve's: the driver's switch). The status line, under every tab: the
//! runtime (its address, how long it has run, its message rate, its devices) and SteamVR's side (its state, what VR
//! shows, the alignment with Valve's frame), each a reading in a cell as wide as its widest value, so that a figure
//! that grows moves nothing; and at its right end the two things to do about them: Re-align and Restart the runtime.
//! In a narrow window the readings that matter least go first.

use egui::{Align2, Color32, Id, Pos2, Rect, Sense, Stroke, Vec2};

use super::kit::{self, Look, Surface};
use super::theme::*;
use super::Panel;

pub const BAR_H: f32 = 44.0;
pub const STATUS_H: f32 = 36.0;

/// The tabs in the bar's order, each with its number
const TABS: [(&str, i32); 5] = [("Tracking", 0), ("Replay", 1), ("Check", 4), ("Output", 2), ("Settings", 3)];

/// Seconds as m:ss, or h:mm:ss from an hour
pub fn clock(s: f64) -> String {
	let s = s.max(0.0) as u64;
	if s >= 3600 {
		format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
	} else {
		format!("{}:{:02}", s / 60, s % 60)
	}
}

/// A reading of the status line: what it is, its value, the widest its value gets (its cell is that wide), its level
/// where it is a status (`marks`: 0 fine, 1 a note, 2 and 3 a fault, written in that colour), whether the value is
/// a figure, and how soon it goes when the line is too narrow (the highest first)
struct Reading {
	key: &'static str,
	value: String,
	widest: &'static str,
	level: i32,
	marks: bool,
	figure: bool,
	rank: u8,
}

impl Reading {
	fn font(&self) -> egui::FontId {
		if self.figure {
			num()
		} else {
			label()
		}
	}

	/// The value's room: its widest, or its own where that is wider still
	fn value_w(&self, p: &egui::Painter) -> f32 {
		kit::text_w(p, &self.value, self.font()).max(kit::text_w(p, self.widest, self.font()))
	}

	fn width(&self, p: &egui::Painter) -> f32 {
		kit::text_w(p, self.key, caption()) + 7.0 + self.value_w(p) + 25.0
	}
}

impl Panel {
	pub(super) fn top_bar(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Bar);
		let cy = r.center().y;

		let wr = kit::text(&p, Pos2::new(r.min.x + 14.0, cy), Align2::LEFT_CENTER, "sally", semibold(17.0), t.text);
		let version = self.version.clone();
		kit::text(&p, Pos2::new(wr.max.x + 7.0, wr.max.y - 3.0), Align2::LEFT_BOTTOM, &version, mono(11.5), t.muted);

		// the tabs: the open one filled as anything that is set
		let mut x = wr.max.x + 68.0;
		for (name, tab) in TABS {
			let w = kit::text_w(&p, name, body()) + 26.0;
			let tr = Rect::from_min_size(Pos2::new(x, cy - 15.0), Vec2::new(w, 30.0));
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
			}
			let h = ui.ctx().animate_bool_with_time(Id::unique(("tabh", tab)), resp.hovered(), 0.08);
			if on {
				p.rect_filled(tr, t.ctl_radius, t.on);
				p.rect_stroke(tr, t.ctl_radius, Stroke::new(1.0, mix(t.on, t.accent, 0.5)), egui::StrokeKind::Inside);
			} else if h > 0.0 {
				p.rect_filled(tr, t.ctl_radius, fade(t.ctl_hover, h));
			}
			let col = if on { t.on_ink } else { mix(t.soft, t.text, h) };
			kit::text(&p, tr.center(), Align2::CENTER_CENTER, name, body(), col);
			x += w + 4.0;
		}
		let tabs_end = x;

		// what SteamVR is shown: sally's tracking or Valve's
		let bh = 30.0;
		let items = ["sally", "Valve"];
		let hook = self.vc.hook_mode && self.st.compare;
		let showing_sally = self.vc.hook_seen == "sally";
		let sw = kit::segmented_w(&p, bh, &items);
		let sp = Pos2::new(r.max.x - 12.0 - sw, cy - bh * 0.5);
		let sr = Rect::from_min_size(sp, Vec2::new(sw, bh));
		let tip = if hook {
			let (_, clicked) =
				kit::segmented(ui, Id::unique("seen"), sp, bh, &items, if showing_sally { 0 } else { 1 });
			if let Some(i) = clicked {
				self.vc.driver_cmd(if i == 0 { "seen sally" } else { "seen valve" });
			}
			if !showing_sally {
				"SteamVR is shown Valve's tracking for every device"
			} else if self.vc.hook_fit {
				"SteamVR is shown sally's tracking (Valve's where sally has none)"
			} else {
				"sally is lining up with Valve's frame: SteamVR is shown Valve's until it has"
			}
		} else {
			// no word from the driver: the choice is there, and off
			kit::segmented_off(&p, sp, bh, &items);
			"No word from the sally driver: SteamVR is not running, or sally is off in it"
		};
		ui.interact(sr, Id::unique("seen-tip"), Sense::hover()).on_hover_text(tip);
		let mut right = sp.x - 12.0;
		// (the switch's label only where the bar has room to spare: the state's line comes first)
		if right - kit::text_w(&p, "SteamVR is shown", label()) > tabs_end + 420.0 {
			let c = if hook { t.muted } else { t.faint };
			right =
				kit::text(&p, Pos2::new(sp.x - 10.0, cy), Align2::RIGHT_CENTER, "SteamVR is shown", label(), c).min.x;
		}

		// what sally is doing, while its tracking is not what SteamVR is shown: between the tabs and the switch, on up
		// to two lines, in its status' colour (yellow on its way, red where it needs the user)
		if let Some((level, text)) = self.state_line() {
			let area = Rect::from_min_max(Pos2::new(tabs_end + 18.0, r.min.y), Pos2::new(right - 18.0, r.max.y));
			if area.width() > 120.0 {
				let tx = area.min.x;
				let ink = if level >= 2 { t.status(level) } else { t.text };
				let format = egui::TextFormat { font_id: label(), color: ink, ..Default::default() };
				let mut job = egui::text::LayoutJob::single_section(text.clone(), format);
				job.wrap = egui::text::TextWrapping {
					max_width: area.max.x - tx,
					max_rows: 2,
					break_anywhere: false,
					overflow_character: Some('…'),
				};
				let galley = p.layout_job(job);
				let at = Pos2::new(tx, cy - galley.size().y * 0.5);
				let said = Rect::from_min_size(at, galley.size());
				p.galley(at, galley, ink);
				// (all of it under the pointer, for where two lines cut it short)
				ui.interact(said, Id::unique("state-line"), Sense::hover()).on_hover_text(text);
			}
		}
	}

	/// The status line: the runtime's readings and SteamVR's, and at the right Re-align and Restart the runtime
	pub(super) fn status_bar(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let p = ui.painter().clone();
		if self.wall.shown() {
			self.wall.glass(&p, r, 0.0, self.view.index(), false);
		} else {
			p.rect_filled(r, 0.0, t.card);
		}
		p.line_segment([r.left_top(), r.right_top()], Stroke::new(1.0, t.card_edge));
		let cy = r.center().y;
		let conn = self.rt.connected(self.now);

		// the two things to do, from the right
		let items = [
			(
				"Re-align",
				Look::Plain,
				self.st.compare && self.vc.running,
				"Lines sally's frame up with Valve's again, from the devices as they move",
			),
			(
				"Restart runtime",
				Look::Plain,
				conn,
				"The runtime quits; under SteamVR the sally driver starts it again, Valve's tracking covers the gap",
			),
		];
		let (clicked, left_of_buttons) = kit::buttons_right(ui, Id::unique("status"), r.max.x - 12.0, cy, 26.0, &items);
		match clicked {
			Some(0) => self.vc.refit(),
			Some(1) => self.confirm_restart = true,
			_ => {}
		}

		let mut list: Vec<Reading> = Vec::new();
		let figure = |key, value: String, widest, rank| Reading {
			key,
			value,
			widest,
			level: 0,
			marks: false,
			figure: true,
			rank,
		};
		let words = |key, value: String, widest, level, marks, rank| Reading {
			key,
			value,
			widest,
			level,
			marks,
			figure: false,
			rank,
		};
		let address = format!("{}:{}", self.rt.host, self.rt.port);
		if conn {
			list.push(figure("Runtime", address, "", 1));
			list.push(figure("Up", clock(self.rt.run_time), "0:00:00", 5));
			list.push(figure("Messages", format!("{:.1}k/s", self.rt.rx_rate / 1000.0), "00.0k/s", 6));
			let (n, all) = (self.active_devices().len(), self.rt.devices.len());
			list.push(figure("Devices", format!("{n} of {all}"), "00 of 00", 7));
			list.push(figure("Protocol", self.rt.protocol.to_string(), "0", 8));
		} else {
			list.push(words("Runtime", "not answering".into(), "", 3, true, 1));
			list.push(figure("Address", address, "", 6));
		}
		if self.st.compare {
			list.push(words("SteamVR", self.vc.status.clone(), "", 0, false, 4));
			if self.vc.hook_mode {
				let (value, level) = if self.vc.hook_seen == "sally" && self.vc.hook_fit {
					(format!("sally, fit {:.1} mm", self.vc.hook_fit_mm), 0)
				} else if self.vc.hook_seen == "sally" {
					("Valve, while sally lines up".to_string(), 2)
				} else {
					("Valve".to_string(), 1)
				};
				list.push(words("In VR", value, "Valve, while sally lines up", level, true, 2));
			}
			let fit = match self.vc.drv_fit {
				Some(f) if self.vc.fit_from_driver && f.refitting => format!("aligning, {} of 150 pairs", f.samples),
				_ if self.vc.fit_from_driver => format!("SteamVR's own, median {:.1} mm", self.vc.fit_median * 1000.0),
				_ if self.vc.fit_ok => format!("{} pairs, median {:.1} mm", self.vc.fit_n, self.vc.fit_median * 1000.0),
				_ if self.vc.running => "not aligned yet".to_string(),
				_ => "no SteamVR".to_string(),
			};
			list.push(words("Alignment", fit, "SteamVR's own, median 00.0 mm", 0, false, 3));
		} else {
			list.push(words("Valve comparison", "off".into(), "", 0, false, 3));
		}

		// the readings that fit, the least needed dropped first
		let room = left_of_buttons - 14.0 - (r.min.x + 12.0);
		while list.len() > 1 && list.iter().map(|c| c.width(&p)).sum::<f32>() > room {
			let (worst, _) = list.iter().enumerate().max_by_key(|(_, c)| c.rank).unwrap();
			list.remove(worst);
		}
		let mut x = r.min.x + 12.0;
		let pc = p.with_clip_rect(Rect::from_min_max(r.min, Pos2::new(left_of_buttons - 8.0, r.max.y)));
		for (i, c) in list.iter().enumerate() {
			if i > 0 {
				pc.line_segment(
					[Pos2::new(x, r.min.y + 9.0), Pos2::new(x, r.max.y - 9.0)],
					Stroke::new(1.0, t.card_edge),
				);
				x += 12.0;
			}
			let kr = kit::text(&pc, Pos2::new(x, cy), Align2::LEFT_CENTER, c.key, caption(), t.muted);
			x = kr.max.x + 7.0;
			// (a reading that is a status is written in its colour: green fine, yellow on its way, red not there)
			let col: Color32 = if c.marks { t.status(c.level) } else { t.text };
			kit::text(&pc, Pos2::new(x, cy), Align2::LEFT_CENTER, &c.value, c.font(), col);
			x += c.value_w(&pc) + 13.0;
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
