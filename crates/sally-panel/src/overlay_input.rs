//! The laser pointer on the overlay: SteamVR's mouse events, queued by `valve` in the order they came
//! (`ValveCompare::ov_events`), turned into the overlay view's egui events once a frame. The C++ panel's
//! overlay_input.h (feed_overlay_events).
//!
//! SteamVR sends pointer moves and scroll ticks faster than the overlay draws, and interleaved. Fed one by one (ImGui's
//! own trickling did that), about one got through a frame: the panel fell seconds behind a drag on the room view and
//! buttons waited behind it. So a frame takes the moves collapsed to the latest position and the scroll ticks added up,
//! and stops at its first press or release: that one goes in, after the position before it, and the rest waits a frame.
//! A click whose down and up came together lands across two frames, its press seen before its release as a mouse's
//! would be. A leave (the laser off the overlay, or the overlay hidden) is a change too: every button still held is
//! released and the pointer is gone.

use egui::{Event, Modifiers, MouseWheelSource, MouseWheelUnit, PointerButton, Pos2, TouchPhase, Vec2};

use crate::valve::{OverlayEvent, OverlayEventKind};

/// egui's buttons by the queue's numbers (0 left, 1 right, 2 middle)
const BUTTONS: [PointerButton; 3] = [PointerButton::Primary, PointerButton::Secondary, PointerButton::Middle];

/// The overlay's pointer between frames: where it was last put, and the buttons held
#[derive(Clone, Debug, Default)]
pub struct OverlayInput {
	/// the last position given to egui (points); none before the first and after a leave
	pos: Option<Pos2>,
	/// pressed and not released yet, by `BUTTONS`
	held: [bool; 3],
}

impl OverlayInput {
	/// The next frame's share of `queue` as egui events, taken off its front. The queue's positions are texture pixels,
	/// y down; egui's are points, at `ppp` pixels a point.
	pub fn take(&mut self, queue: &mut Vec<OverlayEvent>, ppp: f32) -> Vec<Event> {
		let mut out = Vec::new();
		// the latest move not given yet, the ticks added up, whether a button changed
		let mut moved = None;
		let mut wheel = 0.0;
		let mut changed = false;
		let mut used = 0;
		for e in queue.iter() {
			let at = Pos2::new(e.x / ppp, e.y / ppp);
			match e.kind {
				// ticks are taken past the frame's change too, up to the next move or button
				OverlayEventKind::Wheel => wheel += e.wheel,
				_ if changed => break,
				OverlayEventKind::Move => moved = Some(at),
				OverlayEventKind::Button => {
					// valve.rs puts a move before each button; the button's own position is for a queue without one
					self.put(&mut out, moved.take());
					if let Some(&button) = BUTTONS.get(e.button as usize) {
						let pos = *self.pos.get_or_insert(at);
						out.push(Event::PointerButton { pos, button, pressed: e.down, modifiers: Modifiers::NONE });
						self.held[e.button as usize] = e.down;
						changed = true;
					}
				}
				OverlayEventKind::Leave => {
					// the latest move goes in first: it is where the buttons come up (and how far the pointer went
					// from a press, which egui needs to tell a click from a drag)
					self.put(&mut out, moved.take());
					let pos = self.pos.unwrap_or(at);
					for (button, held) in BUTTONS.iter().zip(&mut self.held) {
						if *held {
							out.push(Event::PointerButton {
								pos,
								button: *button,
								pressed: false,
								modifiers: Modifiers::NONE,
							});
							*held = false;
						}
					}
					// after the releases, each of which would put the pointer back
					out.push(Event::PointerGone);
					self.pos = None;
					changed = true;
				}
			}
			used += 1;
		}
		self.put(&mut out, moved);
		if wheel != 0.0 {
			// lines, as a mouse wheel's notches; SteamVR's positive ydelta is up (away), as a wheel turned away is
			out.push(Event::MouseWheel {
				unit: MouseWheelUnit::Line,
				delta: Vec2::new(0.0, wheel),
				phase: TouchPhase::Move,
				source: MouseWheelSource::Unknown,
				modifiers: Modifiers::NONE,
			});
		}
		queue.drain(..used);
		out
	}

	/// Any button held
	pub fn any_held(&self) -> bool {
		self.held.iter().any(|&h| h)
	}

	/// A move given to egui
	fn put(&mut self, out: &mut Vec<Event>, pos: Option<Pos2>) {
		if let Some(p) = pos {
			out.push(Event::PointerMoved(p));
			self.pos = Some(p);
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// The overlay view's pixels a point
	const PPP: f32 = 1.25;

	fn event(kind: OverlayEventKind, x: f32, y: f32, button: i32, down: bool, wheel: f32) -> OverlayEvent {
		OverlayEvent { kind, x, y, button, down, wheel }
	}
	fn mv(x: f32, y: f32) -> OverlayEvent {
		event(OverlayEventKind::Move, x, y, 0, false, 0.0)
	}
	/// A press or release where valve.rs puts it: at the move before it
	fn button(button: i32, down: bool, x: f32, y: f32) -> OverlayEvent {
		event(OverlayEventKind::Button, x, y, button, down, 0.0)
	}
	fn tick(wheel: f32) -> OverlayEvent {
		event(OverlayEventKind::Wheel, 0.0, 0.0, 0, false, wheel)
	}
	fn leave() -> OverlayEvent {
		event(OverlayEventKind::Leave, 0.0, 0.0, 0, false, 0.0)
	}

	/// Texture pixels as egui's points
	fn at(x: f32, y: f32) -> Pos2 {
		Pos2::new(x / PPP, y / PPP)
	}
	fn moved(x: f32, y: f32) -> Event {
		Event::PointerMoved(at(x, y))
	}
	fn pressed(button: PointerButton, pressed: bool, x: f32, y: f32) -> Event {
		Event::PointerButton { pos: at(x, y), button, pressed, modifiers: Modifiers::NONE }
	}
	fn wheel(lines: f32) -> Event {
		Event::MouseWheel {
			unit: MouseWheelUnit::Line,
			delta: Vec2::new(0.0, lines),
			phase: TouchPhase::Move,
			source: MouseWheelSource::Unknown,
			modifiers: Modifiers::NONE,
		}
	}
	fn button_changes(events: &[Event]) -> usize {
		events.iter().filter(|e| matches!(e, Event::PointerButton { .. })).count()
	}

	#[test]
	fn moves_and_ticks_interleaved_are_one_move_and_one_wheel() {
		let mut input = OverlayInput::default();
		let mut q = vec![mv(100.0, 100.0), tick(1.0), mv(200.0, 100.0), tick(0.5), mv(300.0, 125.0), tick(-0.25)];
		assert_eq!(input.take(&mut q, PPP), vec![moved(300.0, 125.0), wheel(1.25)]);
		assert!(q.is_empty());
		// and the position is in points: 300 x 125 texture pixels at 1.25 pixels a point
		assert_eq!(at(300.0, 125.0), Pos2::new(240.0, 100.0));
		assert!(input.take(&mut q, PPP).is_empty());
	}

	#[test]
	fn a_down_and_up_in_one_queue_land_in_two_frames() {
		let mut input = OverlayInput::default();
		let mut q = vec![mv(500.0, 250.0), button(0, true, 500.0, 250.0), button(0, false, 500.0, 250.0)];
		assert_eq!(
			input.take(&mut q, PPP),
			vec![moved(500.0, 250.0), pressed(PointerButton::Primary, true, 500.0, 250.0)]
		);
		assert_eq!(q.len(), 1);
		assert!(input.any_held());
		assert_eq!(input.take(&mut q, PPP), vec![pressed(PointerButton::Primary, false, 500.0, 250.0)]);
		assert!(q.is_empty() && !input.any_held());
	}

	#[test]
	fn a_button_takes_the_position_before_it() {
		let mut input = OverlayInput::default();
		// a press queued after its frame's change waits, and goes in where the pointer was put, not at its own place
		let mut q = vec![mv(10.0, 10.0), button(1, true, 10.0, 10.0), mv(40.0, 20.0), button(1, false, 99.0, 99.0)];
		assert_eq!(
			input.take(&mut q, PPP),
			vec![moved(10.0, 10.0), pressed(PointerButton::Secondary, true, 10.0, 10.0)]
		);
		assert_eq!(
			input.take(&mut q, PPP),
			vec![moved(40.0, 20.0), pressed(PointerButton::Secondary, false, 40.0, 20.0)]
		);
		// with no position known yet, a button is where it says
		let mut fresh = OverlayInput::default();
		let mut q = vec![button(0, true, 25.0, 50.0)];
		assert_eq!(fresh.take(&mut q, PPP), vec![pressed(PointerButton::Primary, true, 25.0, 50.0)]);
	}

	#[test]
	fn ticks_after_the_change_go_with_it_up_to_the_next_move() {
		let mut input = OverlayInput::default();
		let mut q = vec![
			mv(80.0, 80.0),
			button(0, true, 80.0, 80.0),
			tick(1.0),
			tick(1.0),
			mv(90.0, 80.0),
			tick(0.5),
			button(0, false, 90.0, 80.0),
		];
		assert_eq!(
			input.take(&mut q, PPP),
			vec![moved(80.0, 80.0), pressed(PointerButton::Primary, true, 80.0, 80.0), wheel(2.0)]
		);
		assert_eq!(
			input.take(&mut q, PPP),
			vec![moved(90.0, 80.0), pressed(PointerButton::Primary, false, 90.0, 80.0), wheel(0.5)]
		);
		assert!(q.is_empty());
	}

	#[test]
	fn a_leave_releases_what_is_held() {
		let mut input = OverlayInput::default();
		let mut q = vec![mv(100.0, 100.0), button(0, true, 100.0, 100.0), button(2, true, 100.0, 100.0)];
		input.take(&mut q, PPP);
		input.take(&mut q, PPP);
		assert!(q.is_empty() && input.any_held());
		// the latest move first, the releases there, then the pointer gone; what came after waits
		let mut q = vec![mv(150.0, 100.0), mv(1990.0, 100.0), leave(), mv(300.0, 300.0)];
		assert_eq!(
			input.take(&mut q, PPP),
			vec![
				moved(1990.0, 100.0),
				pressed(PointerButton::Primary, false, 1990.0, 100.0),
				pressed(PointerButton::Middle, false, 1990.0, 100.0),
				Event::PointerGone,
			]
		);
		assert!(!input.any_held());
		assert_eq!(input.take(&mut q, PPP), vec![moved(300.0, 300.0)]);
		// a leave with nothing held is the pointer gone alone, and still the frame's change
		let mut q = vec![leave(), mv(5.0, 5.0)];
		assert_eq!(input.take(&mut q, PPP), vec![Event::PointerGone]);
		assert_eq!(q.len(), 1);
	}

	#[test]
	fn at_most_one_button_change_a_frame() {
		let mut input = OverlayInput::default();
		let mut q = vec![
			mv(10.0, 10.0),
			button(0, true, 10.0, 10.0),
			button(1, true, 10.0, 10.0),
			button(1, false, 10.0, 10.0),
			button(0, false, 10.0, 10.0),
			leave(),
		];
		let mut frames = Vec::new();
		while !q.is_empty() {
			let out = input.take(&mut q, PPP);
			assert!(button_changes(&out) <= 1, "{out:?}");
			frames.push(out);
		}
		assert_eq!(frames.len(), 5);
		assert_eq!(frames[4], vec![Event::PointerGone]);
		// a button the queue has no name for is taken and ignored
		let mut q = vec![button(7, true, 10.0, 10.0), mv(20.0, 20.0)];
		assert_eq!(input.take(&mut q, PPP), vec![moved(20.0, 20.0)]);
	}

	/// main_headless.cpp's --overlay-stress, at the queue: a laser held on the room view and dragged with the thumb on
	/// the stick, moves and scroll ticks interleaved at 300 events a second for 3 s, the overlay drawing at 90 Hz. Fed
	/// one event a frame it backed up by the second; here every frame takes the lot, into an egui context too, whose
	/// pointer is always at the latest move with the button down throughout.
	#[test]
	fn a_held_drag_with_ticks_never_backs_up() {
		let ctx = egui::Context::default();
		crate::ui::Panel::setup(&ctx);
		let run = |time: f64, events: Vec<Event>| {
			let raw = egui::RawInput {
				screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 940.0))),
				time: Some(time),
				events,
				..Default::default()
			};
			let mut seen = (None, false);
			// nothing is drawn: the textures' changes go nowhere
			ctx.run_ui(raw, |ui| seen = ui.input(|i| (i.pointer.latest_pos(), i.pointer.primary_down())))
				.drop_without_applying_deltas();
			seen
		};
		let mut input = OverlayInput::default();
		let (rate, fps, seconds) = (300.0, 90.0, 3.0);
		let mut q = vec![mv(1000.0, 470.0), button(0, true, 1000.0, 470.0)];
		let (mut sent, mut last_x, mut ticks_in, mut ticks_out, mut peak) = (0u32, 1000.0f32, 0.0f32, 0.0f32, 0);
		for frame in 1..=(seconds * fps) as u32 {
			let t = frame as f64 / fps;
			// what arrived since the last frame
			while (sent as f64) < t * rate {
				if sent % 2 == 0 {
					last_x = 1000.0 + 250.0 * (sent as f32 * 0.02).sin();
					q.push(mv(last_x, 470.0));
				} else {
					q.push(tick(0.01));
					ticks_in += 0.01;
				}
				sent += 1;
			}
			peak = peak.max(q.len());
			let out = input.take(&mut q, PPP);
			let count = |f: fn(&Event) -> bool| out.iter().filter(|e| f(e)).count();
			assert!(count(|e| matches!(e, Event::PointerMoved(_))) <= 1);
			assert!(count(|e| matches!(e, Event::MouseWheel { .. })) <= 1);
			assert!(button_changes(&out) <= 1);
			for e in &out {
				if let Event::MouseWheel { delta, .. } = e {
					ticks_out += delta.y;
				}
			}
			// after the first frame (the press's), nothing is left over
			if frame > 1 {
				assert!(q.is_empty(), "frame {frame}: {} left", q.len());
			}
			let (pos, down) = run(t, out);
			if frame > 1 {
				assert_eq!(pos, Some(at(last_x, 470.0)), "frame {frame}");
			}
			assert!(down, "frame {frame}");
		}
		q.push(button(0, false, last_x, 470.0));
		let out = input.take(&mut q, PPP);
		assert_eq!(out, vec![pressed(PointerButton::Primary, false, last_x, 470.0)]);
		assert!(!run(seconds + 1.0 / fps, out).1);
		assert_eq!(sent, (rate * seconds) as u32);
		// a frame's worth and the press frame's leftovers, at most
		assert!(peak <= 8, "the queue reached {peak}");
		assert!((ticks_in - ticks_out).abs() < 1e-3, "{ticks_in} ticks in, {ticks_out} out");
	}

	/// A click from the queue is a click to egui: the press and release of one queue, two frames apart
	#[test]
	fn a_click_from_one_queue_clicks_a_button() {
		let ctx = egui::Context::default();
		crate::ui::Panel::setup(&ctx);
		let rect = egui::Rect::from_min_size(Pos2::new(100.0, 100.0), Vec2::new(120.0, 40.0));
		let mut clicks = 0;
		let mut input = OverlayInput::default();
		let mut q = vec![mv(200.0, 150.0), button(0, true, 200.0, 150.0), button(0, false, 200.0, 150.0)];
		for frame in 0..5 {
			// egui finds what the pointer is on from the frame before: the first frame lays the button out
			let events = if frame == 0 { Vec::new() } else { input.take(&mut q, PPP) };
			let raw = egui::RawInput {
				screen_rect: Some(egui::Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 940.0))),
				time: Some(frame as f64 / 90.0),
				events,
				..Default::default()
			};
			ctx.run_ui(raw, |ui| {
				if ui.put(rect, egui::Button::new("Set floor")).clicked() {
					clicks += 1;
				}
			})
			.drop_without_applying_deltas();
		}
		assert_eq!(clicks, 1);
	}
}
