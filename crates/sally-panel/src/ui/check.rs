//! The Check tab: a stretch of the session recorded, checked, and made into one file to send (check.rs,
//! `sally_check::share`). At the left what it does and its one button (Start, then Stop); at the right what the check
//! found, in a few lines per device, with the file and the page.

use egui::{Align2, Id, Pos2, Rect, ScrollArea, UiBuilder, Vec2};

use super::kit::{self, Look, CTL_H, GAP};
use super::theme::*;
use super::topbar::clock;
use super::Panel;
use crate::check::{self, Check};

/// `text` wrapped to `width` at `pos`: the height it took
fn wrapped(p: &egui::Painter, pos: Pos2, width: f32, text: &str, font: egui::FontId, color: egui::Color32) -> f32 {
	let galley = p.layout(text.to_string(), font, color, width.max(40.0));
	let h = galley.size().y;
	p.galley(pos, galley, color);
	h
}

impl Panel {
	/// A check that has run its longest stops by itself; one being made is looked at
	pub(super) fn check_poll(&mut self) {
		self.ck.poll();
		if self.ck.recording().is_some_and(|r| self.now - r.since >= check::MAX_S) {
			self.check_stop();
		}
	}

	fn check_start(&mut self) {
		let paths = self.replay_paths().clone();
		let devices = self.vc.steam_devices();
		// the runtime's word that it is recording: a live one with a recording keeps the room's calibration up
		let recording = self.rt.connected(self.now) && self.rt.calib.available == Some(true);
		self.ck = match Check::start(&paths, devices, recording, self.now) {
			Ok((rec, log)) => {
				self.vc.check_log = Some(log);
				Check::Recording(rec)
			}
			Err(e) => Check::Done(Err(e)),
		};
	}

	fn check_stop(&mut self) {
		if let Check::Recording(rec) = std::mem::take(&mut self.ck) {
			let paths = self.replay_paths().clone();
			let log = self.vc.check_log.take();
			let devices = self.vc.steam_devices();
			self.ck = Check::stop(rec, log, &paths, devices, &self.version, self.now);
		}
	}

	pub(super) fn check_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let c1 = (r.width() * 0.4).clamp(360.0, 560.0);
		let left = Rect::from_min_max(r.min, Pos2::new(r.min.x + c1, r.max.y));
		let right = Rect::from_min_max(Pos2::new(left.max.x + GAP, r.min.y), r.max);

		// what it is, and its button
		let inner = self.card(ui, left, "Check");
		let p = ui.painter().clone();
		let mut y = inner.min.y + 4.0;
		for line in [
			"Records what the trackers, controllers and headset send, says for each where its trouble comes from (the \
			 light, the radio or the device), and writes one file to send.",
			"Start, do what goes wrong, Stop.",
			"The file names no one: every serial in it is a stand-in, and it holds neither this PC's name nor the \
			 Windows user's.",
		] {
			y += wrapped(&p, Pos2::new(inner.min.x + 2.0, y), inner.width() - 4.0, line, body(), t.text) + 12.0;
		}
		y += 6.0;
		let br = |w: f32| Rect::from_min_size(Pos2::new(inner.min.x + 2.0, y), Vec2::new(w, CTL_H + 4.0));
		let after = |r: Rect| Pos2::new(r.max.x + 14.0, r.center().y);
		match &self.ck {
			Check::Recording(rec) => {
				let b = br(kit::button_w(&p, "Stop", false).max(96.0));
				let secs = self.now - rec.since;
				let says = format!("Recording {}  (stops by itself at {})", clock(secs), clock(check::MAX_S));
				kit::text(&p, after(b), Align2::LEFT_CENTER, &says, body(), t.text);
				if kit::button(ui, Id::unique("check-stop"), b, "Stop", None, Look::Danger, true).clicked() {
					self.check_stop();
				}
			}
			Check::Making(since, seconds, _) => {
				let b = br(kit::button_w(&p, "Start", false).max(96.0));
				kit::button(ui, Id::unique("check-wait"), b, "Start", None, Look::Plain, false);
				let says = format!("Checking {} of recording… {}", clock(*seconds), clock(self.now - since));
				kit::text(&p, after(b), Align2::LEFT_CENTER, &says, body(), t.muted);
			}
			Check::Idle | Check::Done(_) => {
				let b = br(kit::button_w(&p, "Start", false).max(96.0));
				if kit::button(ui, Id::unique("check-start"), b, "Start", None, Look::Primary, true).clicked() {
					self.check_start();
				}
			}
		}

		// what it found
		let inner = self.card(ui, right, "What it found");
		let p = ui.painter().clone();
		match &self.ck {
			Check::Done(Ok(s)) => {
				let (zip, page) = (s.zip.clone(), s.report.clone());
				let mb = s.bytes as f64 / 1e6;
				let mut x = inner.min.x + 2.0;
				let y = inner.min.y + 4.0;
				for (id, label, look, path, select) in [
					("check-file", "Show the file to send", Look::Primary, &zip, true),
					("check-page", "Open the report", Look::Plain, &page, false),
				] {
					let b = Rect::from_min_size(Pos2::new(x, y), Vec2::new(kit::button_w(&p, label, false), CTL_H));
					if kit::button(ui, Id::unique(id), b, label, None, look, true).clicked() {
						check::open(path, select);
					}
					x = b.max.x + 8.0;
				}
				let name = zip.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
				let says = format!("{name}, {mb:.1} MB");
				let clip = Rect::from_min_max(Pos2::new(x + 6.0, y), Pos2::new(inner.max.x, y + CTL_H));
				kit::text_in(&p, clip, &says, label(), t.muted);

				let mut text = s.text.clone();
				for n in &s.notes {
					text.push_str(&format!("\n{n}"));
				}
				let area = Rect::from_min_max(Pos2::new(inner.min.x, y + CTL_H + 12.0), inner.max);
				let mut child = ui.new_child(UiBuilder::new().max_rect(area).id_salt("check-found"));
				ScrollArea::vertical().id_salt("check-found-scroll").auto_shrink([false, false]).show(
					&mut child,
					|ui| {
						ui.add(egui::Label::new(egui::RichText::new(text).font(body()).color(t.text)).wrap());
					},
				);
			}
			Check::Done(Err(e)) => {
				let at = Pos2::new(inner.min.x + 2.0, inner.min.y + 4.0);
				wrapped(&p, at, inner.width() - 4.0, e, body(), t.bad);
			}
			Check::Recording(_) => {
				let says = "Recording. Stop when what goes wrong has happened.";
				wrapped(
					&p,
					Pos2::new(inner.min.x + 2.0, inner.min.y + 4.0),
					inner.width() - 4.0,
					says,
					body(),
					t.muted,
				);
			}
			Check::Making(..) => {
				let says = "Reading the recording, checking it and making the file. A minute of recording takes a few seconds.";
				wrapped(
					&p,
					Pos2::new(inner.min.x + 2.0, inner.min.y + 4.0),
					inner.width() - 4.0,
					says,
					body(),
					t.muted,
				);
			}
			Check::Idle => {
				let at = Pos2::new(inner.min.x + 2.0, inner.min.y + 4.0);
				kit::text(&p, at, Align2::LEFT_TOP, "Nothing yet", body(), t.muted);
			}
		}
	}
}
