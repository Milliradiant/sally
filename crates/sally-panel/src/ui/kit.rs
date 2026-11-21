//! The kit: what every tab is drawn with. The boxes with their title line, text, the ruled tables and lists the
//! numbers stand in, and the controls the panel draws itself so that they look and feel the same in the window and
//! under the overlay's laser pointer: buttons, tick boxes, segmented choices, the slider with its steppers. A control
//! always looks like one: a filled, outlined shape with its label on it, there whether or not the pointer is over
//! it. Nothing opens out of another thing: there are no pop-ups and no drop-down lists.

use egui::{Align2, Color32, CornerRadius, FontId, Id, Painter, Pos2, Rect, Response, Sense, Stroke, StrokeKind, Vec2};

use super::icons;
use super::theme::*;
use super::{Panel, View};
use crate::symbols::Symbol;

/// Between boxes, and a box's inner margin
pub const GAP: f32 = 8.0;
pub const PAD: f32 = 12.0;
/// A box's title line (its title, and what sits at its right end)
pub const HEADER_H: f32 = 36.0;
/// A control's height: buttons, tick boxes, segmented choices
pub const CTL_H: f32 = 30.0;
/// The boxes docked over the room (the Tracking tab's), by their place in the settings' `fold`
pub const DOCK_ROOM: usize = 0;
pub const DOCK_STATIONS: usize = 1;
pub const DOCK_EVENTS: usize = 2;
pub const DOCK_DEVICES: usize = 3;
/// The room a docked box's arrow takes before its title
pub const FOLD_W: f32 = 26.0;
/// A docked box folded to its title line
pub const BAND_H: f32 = HEADER_H + 1.0;
/// A table's header row and its rows
pub const TH_H: f32 = 26.0;
pub const TR_H: f32 = 30.0;

/// What a box is: a card, something set into one (the room, a plot), or the bar across the top
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
	Card,
	Well,
	Bar,
}

/// A button's look
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Look {
	/// a control like any other
	Plain,
	/// the one to press: filled as the on state
	Primary,
	/// outlined in red (something that stops or cannot be undone)
	Danger,
}

// ---- text ----

/// Text at `pos`; the rect it took
pub fn text(p: &Painter, pos: Pos2, align: Align2, s: &str, font: FontId, color: Color32) -> Rect {
	p.text(pos, align, s, font, color)
}

pub fn text_w(p: &Painter, s: &str, font: FontId) -> f32 {
	p.layout_no_wrap(s.to_string(), font, Color32::WHITE).size().x
}

/// Text in `r` from its left, centred on its height; one too long for it ends in "…"
pub fn text_in(p: &Painter, r: Rect, s: &str, font: FontId, color: Color32) -> Rect {
	let pc = p.with_clip_rect(r.intersect(p.clip_rect()));
	if !text_cut(p, r.width(), s, font.clone()) {
		return text(&pc, Pos2::new(r.min.x, r.center().y), Align2::LEFT_CENTER, s, font, color);
	}
	let mut job = egui::text::LayoutJob::simple_singleline(s.to_string(), font, color);
	job.wrap = egui::text::TextWrapping {
		max_width: r.width().max(0.0),
		max_rows: 1,
		break_anywhere: true,
		overflow_character: Some('…'),
	};
	let galley = p.layout_job(job);
	let pos = Pos2::new(r.min.x, r.center().y - galley.size().y * 0.5);
	let rect = Rect::from_min_size(pos, galley.size());
	pc.galley(pos, galley, color);
	rect
}

/// Text in `r` from its right, centred on its height (a figure in its column), cut at the rect
pub fn text_right(p: &Painter, r: Rect, s: &str, font: FontId, color: Color32) -> Rect {
	let pc = p.with_clip_rect(r.intersect(p.clip_rect()));
	text(&pc, Pos2::new(r.max.x, r.center().y), Align2::RIGHT_CENTER, s, font, color)
}

/// Whether `s` is too long for `width` (and `text_in` shortens it)
pub fn text_cut(p: &Painter, width: f32, s: &str, font: FontId) -> bool {
	text_w(p, s, font) > width + 0.5
}

/// An icon centred at `c`
pub fn icon_at(p: &Painter, c: Pos2, glyph: &str, size: f32, color: Color32) {
	p.text(c, Align2::CENTER_CENTER, glyph, icon(size), color);
}

/// A value and its unit from `pos` (the value's left, both on one baseline): where they end
pub fn value_unit(p: &Painter, pos: Pos2, value: &str, unit: &str, font: FontId, color: Color32) -> f32 {
	let t = tk();
	let size = font.size;
	let vr = text(p, pos, Align2::LEFT_BOTTOM, value, font, color);
	if unit.is_empty() {
		return vr.max.x;
	}
	let us = (size * 0.7).max(11.5);
	// the unit on the value's baseline: both rects' bottoms are their fonts' descent below it
	let drop = (size - us) * 0.18;
	text(p, Pos2::new(vr.max.x + size * 0.3, pos.y - drop), Align2::LEFT_BOTTOM, unit, medium(us), t.muted).max.x
}

// ---- controls ----

fn hover_t(ui: &egui::Ui, id: Id, on: bool) -> f32 {
	ui.ctx().animate_bool_with_time(id.with("hover"), on, 0.08)
}

fn ctl_radius() -> CornerRadius {
	CornerRadius::same(tk().ctl_radius as u8)
}

/// A control's body in `r`: its fill and the line around it
fn ctl_body(p: &Painter, r: Rect, fill: Color32, edge: Color32) {
	p.rect_filled(r, ctl_radius(), fill);
	p.rect_stroke(r, ctl_radius(), Stroke::new(1.0, edge), StrokeKind::Inside);
}

/// A button: an icon, a label, or both, centred in `r`
pub fn button(ui: &egui::Ui, id: Id, r: Rect, label: &str, glyph: Option<&str>, look: Look, enabled: bool) -> Response {
	let t = tk();
	let resp = ui.interact(r, id, if enabled { Sense::click() } else { Sense::hover() });
	let p = ui.painter();
	let h = hover_t(ui, id, enabled && resp.hovered());
	let down = enabled && resp.is_pointer_button_down_on();
	let (fill, edge, fg) = match look {
		Look::Plain => (if down { t.ctl_press } else { mix(t.ctl, t.ctl_hover, h) }, t.card_edge, t.text),
		Look::Primary => (
			if down { mix(t.on, t.bg, 0.2) } else { mix(t.on, t.accent, h * 0.25) },
			mix(t.on, t.accent, 0.5),
			t.on_ink,
		),
		Look::Danger => (if down { t.ctl_press } else { mix(t.ctl, t.ctl_hover, h) }, t.bad, t.text),
	};
	let k: f32 = if enabled { 1.0 } else { 0.45 };
	ctl_body(p, r, fade(fill, k.max(0.6)), fade(edge, k));
	let fg = fade(fg, if enabled { 1.0 } else { 0.45 });
	let font = self::label();
	let lw = if label.is_empty() { 0.0 } else { text_w(p, label, font.clone()) };
	let (iw, gap) = match glyph {
		Some(_) if label.is_empty() => (15.0, 0.0),
		Some(_) => (15.0, 6.0),
		None => (0.0, 0.0),
	};
	let x0 = r.center().x - (iw + gap + lw) * 0.5;
	let cy = r.center().y;
	if let Some(g) = glyph {
		icon_at(p, Pos2::new(x0 + iw * 0.5, cy), g, 15.0, fg);
	}
	if !label.is_empty() {
		let pc = p.with_clip_rect(r.shrink2(Vec2::new(4.0, 0.0)).intersect(p.clip_rect()));
		pc.text(Pos2::new(x0 + iw + gap, cy), Align2::LEFT_CENTER, label, font, fg);
	}
	resp
}

fn label() -> FontId {
	super::theme::label()
}

/// How wide a button is with this label (and an icon)
pub fn button_w(p: &Painter, label: &str, glyph: bool) -> f32 {
	text_w(p, label, self::label()) + if glyph { 21.0 } else { 0.0 } + 24.0
}

/// A row of buttons from `right` leftward, each as wide as its label, `h` tall, centred on `cy`: the one clicked (its
/// index in `items`: label, look, enabled, hint) and the row's left end
pub fn buttons_right(
	ui: &egui::Ui, id: Id, right: f32, cy: f32, h: f32, items: &[(&str, Look, bool, &str)],
) -> (Option<usize>, f32) {
	let mut x = right;
	let mut clicked = None;
	for (i, (text_, look, enabled, hint)) in items.iter().enumerate().rev() {
		let w = button_w(ui.painter(), text_, false);
		let r = Rect::from_min_size(Pos2::new(x - w, cy - h * 0.5), Vec2::new(w, h));
		let mut resp = button(ui, id.with(i), r, text_, None, *look, *enabled);
		if !hint.is_empty() {
			resp = resp.on_hover_text(*hint);
		}
		if resp.clicked() {
			clicked = Some(i);
		}
		x = r.min.x - 6.0;
	}
	(clicked, x + 6.0)
}

/// A square button of one icon; `on`: filled as the on state
pub fn icon_button(ui: &egui::Ui, id: Id, r: Rect, glyph: &str, on: bool, enabled: bool) -> Response {
	let t = tk();
	let resp = ui.interact(r, id, if enabled { Sense::click() } else { Sense::hover() });
	let p = ui.painter();
	let h = hover_t(ui, id, enabled && resp.hovered());
	let down = enabled && resp.is_pointer_button_down_on();
	let size = (r.height() * 0.5).clamp(13.0, 17.0);
	let k: f32 = if enabled { 1.0 } else { 0.45 };
	if on {
		ctl_body(p, r, mix(t.on, t.accent, h * 0.25), mix(t.on, t.accent, 0.5));
		icon_at(p, r.center(), glyph, size, t.on_ink);
	} else {
		let fill = if down { t.ctl_press } else { mix(t.ctl, t.ctl_hover, h) };
		ctl_body(p, r, fade(fill, k.max(0.6)), fade(t.card_edge, k));
		icon_at(p, r.center(), glyph, size, fade(t.text, k));
	}
	resp
}

/// A tick box's side
pub const TICK: f32 = 20.0;

/// A tick box at `pos` (its top left), `TICK` square; whether it was clicked
pub fn tick(ui: &egui::Ui, id: Id, pos: Pos2, on: bool, enabled: bool) -> bool {
	let r = Rect::from_min_size(pos, Vec2::splat(TICK));
	// the target a little larger than the picture
	let resp = ui.interact(r.expand(5.0), id, if enabled { Sense::click() } else { Sense::hover() });
	tick_box(ui, id, r, on, enabled, resp.hovered());
	enabled && resp.clicked()
}

fn tick_box(ui: &egui::Ui, id: Id, r: Rect, on: bool, enabled: bool, hovered: bool) {
	let t = tk();
	let p = ui.painter();
	let h = hover_t(ui, id, enabled && hovered);
	let k: f32 = if enabled { 1.0 } else { 0.45 };
	if on {
		ctl_body(p, r, fade(mix(t.on, t.accent, h * 0.25), k), fade(mix(t.on, t.accent, 0.5), k));
		icon_at(p, r.center(), icons::CHECK, 14.0, fade(t.on_ink, k));
	} else {
		ctl_body(
			p,
			r,
			fade(mix(t.well, t.ctl_hover, h), k.max(0.6)),
			// (its edge 3 to 1 against a box's ground and near it on a title line: an empty box is its edge)
			fade(mix(t.card_edge, t.muted, 0.6 + 0.3 * h), k),
		);
	}
}

/// How wide a tick box with this label is
pub fn tick_label_w(p: &Painter, text_: &str) -> f32 {
	TICK + 7.0 + text_w(p, text_, label())
}

/// A tick box with its label after it, `h` tall at `pos`: a click on either. Its rect and whether it was clicked
pub fn tick_label(ui: &egui::Ui, id: Id, pos: Pos2, h: f32, text_: &str, on: bool, enabled: bool) -> (Rect, bool) {
	let t = tk();
	let w = tick_label_w(ui.painter(), text_);
	let r = Rect::from_min_size(pos, Vec2::new(w, h));
	let resp = ui.interact(r.expand2(Vec2::new(3.0, 0.0)), id, if enabled { Sense::click() } else { Sense::hover() });
	let b = Rect::from_min_size(Pos2::new(r.min.x, r.center().y - TICK * 0.5), Vec2::splat(TICK));
	tick_box(ui, id, b, on, enabled, resp.hovered());
	let col = if !enabled {
		t.faint
	} else if on {
		t.text
	} else {
		t.soft
	};
	ui.painter().text(Pos2::new(b.max.x + 7.0, r.center().y), Align2::LEFT_CENTER, text_, label(), col);
	(r, enabled && resp.clicked())
}

/// A row of tick boxes with their labels from `pos` to `right`, `h` tall, going on in the next line where they do not
/// fit: each one's label, whether it is on, and a series' colour as a square before its label (what is drawn in that
/// colour). Which one was clicked, and the last one's right end and bottom
pub fn ticks(
	ui: &egui::Ui, id: Id, pos: Pos2, right: f32, h: f32, items: &[(&str, bool, Option<Color32>)],
) -> (Option<usize>, Pos2) {
	const GAP: f32 = 16.0;
	let t = tk();
	let (mut x, mut y) = (pos.x, pos.y);
	let mut clicked = None;
	for (i, (text_, on, mark)) in items.iter().enumerate() {
		let mw = if mark.is_some() { 14.0 } else { 0.0 };
		let w = TICK + 7.0 + mw + text_w(ui.painter(), text_, label());
		if x > pos.x && x + w > right {
			x = pos.x;
			y += h + 2.0;
		}
		let r = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w, h));
		let resp = ui.interact(r.expand2(Vec2::new(3.0, 0.0)), id.with(i), Sense::click());
		let b = Rect::from_min_size(Pos2::new(r.min.x, r.center().y - TICK * 0.5), Vec2::splat(TICK));
		tick_box(ui, id.with(i), b, *on, true, resp.hovered());
		let p = ui.painter();
		let mut tx = b.max.x + 7.0;
		if let Some(c) = mark {
			p.rect_filled(Rect::from_center_size(Pos2::new(tx + 4.0, r.center().y), Vec2::splat(9.0)), 1.0, *c);
			tx += mw;
		}
		p.text(Pos2::new(tx, r.center().y), Align2::LEFT_CENTER, text_, label(), if *on { t.text } else { t.soft });
		if resp.clicked() {
			clicked = Some(i);
		}
		x = r.max.x + GAP;
	}
	(clicked, Pos2::new(x - GAP, y + h))
}

/// A choice of one among a few, in a row from `pos`: its rect and the one clicked
pub fn segmented(ui: &egui::Ui, id: Id, pos: Pos2, h: f32, items: &[&str], selected: usize) -> (Rect, Option<usize>) {
	let t = tk();
	let p = ui.painter();
	let font = label();
	let pad = h * 0.4;
	let widths: Vec<f32> = items.iter().map(|s| text_w(p, s, font.clone()) + pad * 2.0).collect();
	let r = Rect::from_min_size(pos, Vec2::new(widths.iter().sum::<f32>(), h));
	ctl_body(p, r, t.ctl, t.card_edge);
	let mut x = pos.x;
	let mut clicked = None;
	for (i, (s, w)) in items.iter().zip(&widths).enumerate() {
		let cell = Rect::from_min_size(Pos2::new(x, pos.y), Vec2::new(*w, h));
		let resp = ui.interact(cell, id.with(i), Sense::click());
		let hv = hover_t(ui, id.with(i), resp.hovered());
		if i == selected {
			ctl_body(p, cell, t.on, mix(t.on, t.accent, 0.5));
			p.text(cell.center(), Align2::CENTER_CENTER, s, font.clone(), t.on_ink);
		} else {
			if hv > 0.0 {
				p.rect_filled(cell.shrink(1.0), ctl_radius(), fade(t.ctl_hover, hv));
			}
			if i > 0 && i != selected + 1 {
				p.line_segment(
					[Pos2::new(cell.min.x, cell.min.y + 6.0), Pos2::new(cell.min.x, cell.max.y - 6.0)],
					Stroke::new(1.0, t.card_edge),
				);
			}
			p.text(cell.center(), Align2::CENTER_CENTER, s, font.clone(), mix(t.soft, t.text, hv));
		}
		if resp.clicked() && i != selected {
			clicked = Some(i);
		}
		x += w;
	}
	(r, clicked)
}

/// How wide a segmented choice of these is
pub fn segmented_w(p: &Painter, h: f32, items: &[&str]) -> f32 {
	items.iter().map(|s| text_w(p, s, label()) + h * 0.8).sum::<f32>()
}

/// A segmented choice that cannot be used yet: there, and faded
pub fn segmented_off(p: &Painter, pos: Pos2, h: f32, items: &[&str]) {
	let t = tk();
	let w = segmented_w(p, h, items);
	let r = Rect::from_min_size(pos, Vec2::new(w, h));
	ctl_body(p, r, fade(t.ctl, 0.6), fade(t.card_edge, 0.45));
	let mut x = pos.x;
	for s in items {
		let cw = text_w(p, s, label()) + h * 0.8;
		p.text(Pos2::new(x + cw * 0.5, r.center().y), Align2::CENTER_CENTER, s, label(), t.faint);
		x += cw;
	}
}

#[allow(clippy::too_many_arguments)]
/// A button that is on or off, `h` tall at `pos`, with `pad` points around its label in all: its rect, and whether it
/// was clicked. `mark`: a small square in that colour before the label
pub fn chip(
	ui: &egui::Ui, id: Id, pos: Pos2, h: f32, text_: &str, on: bool, mark: Option<Color32>, pad: f32,
) -> (Rect, bool) {
	let t = tk();
	let p = ui.painter();
	let font = label();
	let dw = if mark.is_some() { 13.0 } else { 0.0 };
	let w = text_w(p, text_, font.clone()) + pad + dw;
	let r = Rect::from_min_size(pos, Vec2::new(w, h));
	let resp = ui.interact(r, id, Sense::click());
	let hv = hover_t(ui, id, resp.hovered());
	let (fill, edge, fg) = if on {
		(mix(t.on, t.accent, hv * 0.25), mix(t.on, t.accent, 0.5), t.on_ink)
	} else {
		(mix(t.ctl, t.ctl_hover, hv), t.card_edge, mix(t.soft, t.text, hv))
	};
	ctl_body(p, r, fill, edge);
	let x = r.min.x + pad * 0.5;
	if let Some(c) = mark {
		p.rect_filled(Rect::from_center_size(Pos2::new(x + 3.5, r.center().y), Vec2::splat(7.0)), 1.0, c);
	}
	p.text(Pos2::new(x + dw, r.center().y), Align2::LEFT_CENTER, text_, font, fg);
	(r, resp.clicked())
}

/// A row of on/off buttons from `pos` to `right`: where they do not fit, the room around their labels shrinks, and
/// only then do they wrap. Which one was clicked, and the last one's right end and bottom
pub fn chips(
	ui: &egui::Ui, id: Id, pos: Pos2, right: f32, h: f32, items: &[(&str, bool, Option<Color32>)],
) -> (Option<usize>, Pos2) {
	const GAP: f32 = 6.0;
	let p = ui.painter();
	let widths: Vec<f32> =
		items.iter().map(|(s, _, d)| text_w(p, s, label()) + if d.is_some() { 13.0 } else { 0.0 }).collect();
	let n = items.len().max(1) as f32;
	let room = right - pos.x - GAP * (n - 1.0) - widths.iter().sum::<f32>();
	let pad = (room / n).clamp(h * 0.45, h * 0.8);
	let (mut x, mut y) = (pos.x, pos.y);
	let mut clicked = None;
	for (i, ((s, on, d), w)) in items.iter().zip(&widths).enumerate() {
		if x > pos.x && x + w + pad > right {
			x = pos.x;
			y += h + GAP;
		}
		let (r, c) = chip(ui, id.with(i), Pos2::new(x, y), h, s, *on, *d, pad);
		if c {
			clicked = Some(i);
		}
		x = r.max.x + GAP;
	}
	(clicked, Pos2::new(x - GAP, y + h))
}

/// A slider's range and marks
#[derive(Clone, Copy, Debug)]
pub struct Range {
	pub min: f64,
	pub max: f64,
	/// what a click on a stepper moves it by, and what its values are rounded to
	pub step: f64,
	/// a tick on the rail: the value it has by default
	pub default: Option<f64>,
	/// a ring on the rail: the value it would have without its own (a device's class's)
	pub inherits: Option<f64>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Slid {
	/// the value moved this frame
	pub changed: bool,
	/// and was let go (a drag's end, a click on the rail or a stepper): the moment to send it
	pub released: bool,
}

/// A slider in `r`: a stepper at each end (one step a click, for the laser pointer, which drags roughly) and the rail
/// between them. A value outside the range is shown at the rail's end
pub fn slider(ui: &egui::Ui, id: Id, r: Rect, v: &mut f64, range: &Range, enabled: bool) -> Slid {
	let t = tk();
	let mut out = Slid::default();
	let bw = r.height().min(26.0);
	let snap = |x: f64| (x / range.step).round() * range.step;
	let minus = Rect::from_min_size(Pos2::new(r.min.x, r.center().y - bw * 0.5), Vec2::splat(bw));
	let plus = Rect::from_min_size(Pos2::new(r.max.x - bw, r.center().y - bw * 0.5), Vec2::splat(bw));
	if icon_button(ui, id.with("minus"), minus, icons::MINUS, false, enabled && *v > range.min).clicked() {
		*v = snap((*v - range.step).max(range.min));
		out = Slid { changed: true, released: true };
	}
	if icon_button(ui, id.with("plus"), plus, icons::PLUS, false, enabled && *v < range.max).clicked() {
		*v = snap((*v + range.step).min(range.max));
		out = Slid { changed: true, released: true };
	}
	let knob = 6.0;
	let rail =
		Rect::from_min_max(Pos2::new(minus.max.x + 10.0 + knob, r.min.y), Pos2::new(plus.min.x - 10.0 - knob, r.max.y));
	let hit = rail.expand2(Vec2::new(knob + 4.0, 0.0));
	let resp = ui.interact(hit, id, if enabled { Sense::click_and_drag() } else { Sense::hover() });
	let at = |x: f64| rail.min.x + (((x - range.min) / (range.max - range.min)).clamp(0.0, 1.0) as f32) * rail.width();
	if enabled && (resp.dragged() || resp.clicked() || resp.drag_started()) {
		if let Some(mp) = resp.interact_pointer_pos() {
			let k = ((mp.x - rail.min.x) / rail.width()).clamp(0.0, 1.0) as f64;
			let nv = snap(range.min + k * (range.max - range.min)).clamp(range.min, range.max);
			if nv != *v {
				*v = nv;
				out.changed = true;
			}
		}
	}
	if enabled && (resp.drag_stopped() || resp.clicked()) {
		out.released = true;
	}
	let p = ui.painter();
	let cy = r.center().y;
	let k: f32 = if enabled { 1.0 } else { 0.4 };
	let track = Rect::from_min_max(Pos2::new(rail.min.x - knob, cy - 2.0), Pos2::new(rail.max.x + knob, cy + 2.0));
	// (the rail to its end 3 to 1 against the box: at 0 it is all there is to see of how far the slider goes)
	p.rect_filled(track, 1.0, fade(t.group, k.max(0.7)));
	let x = at(*v);
	// filled from the rail's start, or from 0 where the range goes either side of it
	let filled = if range.min < 0.0 && range.max > 0.0 {
		let zero = at(0.0);
		Rect::from_min_max(Pos2::new(zero.min(x), track.min.y), Pos2::new(zero.max(x), track.max.y))
	} else {
		Rect::from_min_max(track.min, Pos2::new(x, track.max.y))
	};
	p.rect_filled(filled, 1.0, fade(t.accent, k));
	if let Some(d) = range.default {
		let dx = at(d);
		p.line_segment([Pos2::new(dx, cy + 8.0), Pos2::new(dx, cy + 12.0)], Stroke::new(1.5, fade(t.muted, k)));
	}
	if let Some(d) = range.inherits {
		p.circle_stroke(Pos2::new(at(d), cy), 4.5, Stroke::new(1.5, fade(t.muted, k)));
	}
	let h = hover_t(ui, id, enabled && (resp.hovered() || resp.dragged()));
	let grab = Rect::from_center_size(Pos2::new(x, cy), Vec2::new(knob * 2.0, 18.0));
	p.rect_filled(grab, 2.0, fade(mix(t.soft, t.on_ink, h), k));
	out
}

/// A stepper in place of a slider where the room is a toolbar's: `text_` and the value as `says`, with a button
/// either side that moves it a step; whether it moved, and the rect taken (from `pos`, `h` tall)
#[allow(clippy::too_many_arguments)]
pub fn stepper(
	ui: &egui::Ui, id: Id, pos: Pos2, h: f32, text_: &str, says: &str, v: &mut f64, range: &Range,
) -> (Rect, bool) {
	let t = tk();
	let p = ui.painter();
	let snap = |x: f64| (x / range.step).round() * range.step;
	let lw = text_w(p, text_, label());
	// the value's room is that of its widest
	let vw = text_w(p, "00.0 m", num());
	let cy = pos.y + h * 0.5;
	p.text(Pos2::new(pos.x, cy), Align2::LEFT_CENTER, text_, label(), t.soft);
	let bw = h.min(26.0);
	let minus = Rect::from_min_size(Pos2::new(pos.x + lw + 8.0, cy - bw * 0.5), Vec2::splat(bw));
	let vr = Rect::from_min_size(Pos2::new(minus.max.x, pos.y), Vec2::new(vw + 10.0, h));
	let plus = Rect::from_min_size(Pos2::new(vr.max.x, cy - bw * 0.5), Vec2::splat(bw));
	let mut moved = false;
	if icon_button(ui, id.with("minus"), minus, icons::MINUS, false, *v > range.min).clicked() {
		*v = snap((*v - range.step).max(range.min));
		moved = true;
	}
	ui.painter().text(vr.center(), Align2::CENTER_CENTER, says, num(), t.text);
	if icon_button(ui, id.with("plus"), plus, icons::PLUS, false, *v < range.max).clicked() {
		*v = snap((*v + range.step).min(range.max));
		moved = true;
	}
	(Rect::from_min_max(pos, Pos2::new(plus.max.x, pos.y + h)), moved)
}

/// How wide `stepper` is with this label
pub fn stepper_w(p: &Painter, h: f32, text_: &str) -> f32 {
	text_w(p, text_, label()) + 8.0 + 2.0 * h.min(26.0) + text_w(p, "00.0 m", num()) + 10.0
}

// ---- small pictures ----

/// A meter: `frac` of `r` filled
pub fn meter(p: &Painter, r: Rect, frac: f64, col: Color32) {
	let t = tk();
	p.rect_filled(r, 1.0, t.ctl);
	let w = (frac.clamp(0.0, 1.0) as f32) * r.width();
	if w > 0.0 {
		p.rect_filled(Rect::from_min_size(r.min, Vec2::new(w.max(2.0), r.height())), 1.0, col);
	}
}

/// The panel's own icon for a device the runtime calls `subtype` (where SteamVR's symbol of it is not to be had)
pub fn glyph_of(subtype: &str) -> &'static str {
	match subtype {
		"knuckles_l" | "knuckles_r" | "wand" => icons::GAME_CONTROLLER,
		"index_hmd" | "vive_hmd" => icons::VIRTUAL_REALITY,
		_ => icons::HEXAGON,
	}
}

/// A device's symbol in `r`, in `col`: SteamVR's own icon of it (symbols.rs) where there is one, fitted and centred,
/// else the panel's `glyph`
pub fn symbol(p: &Painter, r: Rect, sym: Option<Symbol>, glyph: &str, col: Color32) {
	match sym {
		Some((tex, size)) => {
			let (w, h) = (size[0].max(1) as f32, size[1].max(1) as f32);
			let k = (r.width() / w).min(r.height() / h);
			let (w, h) = ((w * k).round(), (h * k).round());
			let min = Pos2::new((r.center().x - w * 0.5).round(), (r.center().y - h * 0.5).round());
			let uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
			p.image(tex, Rect::from_min_size(min, Vec2::new(w, h)), uv, col);
		}
		None => icon_at(p, r.center(), glyph, r.height().min(r.width()) * 0.84, col),
	}
}

// ---- tables and lists ----

/// A column of a table: its heading, its width (points; 0 or less: a share of what the fixed ones leave, by its size),
/// whether it holds figures (set from the right, as its heading is), and whether a strong line stands before it (the
/// first column of a group)
#[derive(Clone, Copy, Debug)]
pub struct Col<'a> {
	pub title: &'a str,
	pub w: f32,
	pub right: bool,
	pub sep: bool,
}

impl<'a> Col<'a> {
	/// A column of text
	pub fn text(title: &'a str, w: f32) -> Col<'a> {
		Col { title, w, right: false, sep: false }
	}
	/// A column of figures
	pub fn num(title: &'a str, w: f32) -> Col<'a> {
		Col { title, w, right: true, sep: false }
	}
	/// The first column of a group: a strong line before it
	pub fn sep(mut self) -> Col<'a> {
		self.sep = true;
		self
	}
}

/// A table's columns laid out in a rect: where each one is
pub struct Table {
	/// each column's left and right (the cell, before its padding)
	xs: Vec<(f32, f32)>,
	right: Vec<bool>,
	sep: Vec<bool>,
	pub left: f32,
	pub width: f32,
}

/// The padding either side of a cell's content
const CELL_PAD: f32 = 8.0;

impl Table {
	/// The columns from `left`, `width` wide in all
	pub fn new(left: f32, width: f32, cols: &[Col]) -> Table {
		let fixed: f32 = cols.iter().map(|c| c.w.max(0.0)).sum();
		let shares: f32 = cols.iter().map(|c| (-c.w).max(0.0)).sum();
		let spare = (width - fixed).max(0.0);
		let mut x = left;
		let xs = cols
			.iter()
			.map(|c| {
				let w = if c.w > 0.0 {
					c.w
				} else if shares > 0.0 {
					spare * (-c.w) / shares
				} else {
					0.0
				};
				let a = x;
				x += w;
				(a, x)
			})
			.collect();
		Table {
			xs,
			right: cols.iter().map(|c| c.right).collect(),
			sep: cols.iter().map(|c| c.sep).collect(),
			left,
			width,
		}
	}

	/// The header row at `y`: the headings on the header's ground, the columns' lines through it, a line under it.
	/// Where the rows start
	pub fn header(&self, p: &Painter, y: f32, cols: &[Col]) -> f32 {
		let t = tk();
		let r = Rect::from_min_size(Pos2::new(self.left, y), Vec2::new(self.width, TH_H));
		p.rect_filled(r, 0.0, t.head);
		self.seps(p, r.min.y, r.max.y);
		p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.card_edge));
		for (i, c) in cols.iter().enumerate() {
			let cell = self.cell(i, y, TH_H);
			if cell.width() < 4.0 {
				continue;
			}
			if self.right[i] {
				text_right(p, cell, c.title, caption(), t.muted);
			} else {
				text_in(p, cell, c.title, caption(), t.muted);
			}
		}
		y + TH_H
	}

	/// The lines between the columns, from `y0` to `y1`: a light one before every column but the first, a strong one
	/// where a group of columns begins
	pub fn seps(&self, p: &Painter, y0: f32, y1: f32) {
		let t = tk();
		for (i, (a, _)) in self.xs.iter().enumerate().skip(1) {
			let col = if self.sep[i] { t.group } else { t.line };
			p.line_segment([Pos2::new(*a, y0), Pos2::new(*a, y1)], Stroke::new(1.0, col));
		}
	}

	/// Column `i`'s cell in the row at `y`, `h` tall, inside its padding
	pub fn cell(&self, i: usize, y: f32, h: f32) -> Rect {
		let (a, b) = self.xs[i];
		Rect::from_min_max(Pos2::new(a + CELL_PAD, y), Pos2::new((b - CELL_PAD).max(a + CELL_PAD), y + h))
	}

	/// A row's whole rect
	pub fn row(&self, y: f32, h: f32) -> Rect {
		Rect::from_min_size(Pos2::new(self.left, y), Vec2::new(self.width, h))
	}

	/// A figure in column `i` of the row at `y` (from the right), or a text (from the left, cut with "…")
	pub fn put(&self, p: &Painter, i: usize, y: f32, s: &str, font: FontId, color: Color32) {
		let cell = self.cell(i, y, TR_H);
		if cell.width() < 4.0 {
			return;
		}
		if self.right[i] {
			text_right(p, cell, s, font, color);
		} else {
			text_in(p, cell, s, font, color);
		}
	}
}

/// A row's ground: every other row striped (`odd`), the line under it, and its fill when it is the selected one
/// (with the mark at its left end) or under the pointer
pub fn row_ground(p: &Painter, r: Rect, odd: bool, selected: bool, hover: f32) {
	let t = tk();
	if odd {
		p.rect_filled(r, 0.0, t.stripe);
	}
	if selected {
		p.rect_filled(r, 0.0, alpha(t.accent, 42));
		p.rect_filled(Rect::from_min_max(r.min, Pos2::new(r.min.x + 3.0, r.max.y)), 0.0, t.accent);
	} else if hover > 0.0 {
		p.rect_filled(r, 0.0, fade(white(14), hover));
	}
	p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.line));
}

/// A key and its value on a ruled line in `r` (the key from its left, `key_w` wide; the value after it, a figure in
/// the figures' face where `figure`)
pub fn kv_line(p: &Painter, r: Rect, key_w: f32, key: &str, value: &str, figure: bool) {
	let t = tk();
	text_in(p, Rect::from_min_max(r.min, Pos2::new(r.min.x + key_w - 8.0, r.max.y)), key, label(), t.muted);
	let vr = Rect::from_min_max(Pos2::new(r.min.x + key_w, r.min.y), r.max);
	text_in(p, vr, value, if figure { num() } else { label() }, t.text);
	p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.line));
}

/// A reading in a cell of its own: what it is over its figure and unit, a line at the cell's right where `rule`
pub fn reading(p: &Painter, r: Rect, name: &str, value: &str, unit: &str, rule: bool) {
	let t = tk();
	text(p, Pos2::new(r.min.x, r.min.y + 9.0), Align2::LEFT_CENTER, name, caption(), t.muted);
	value_unit(p, Pos2::new(r.min.x, r.max.y - 3.0), value, unit, figure(), t.text);
	if rule {
		p.line_segment([Pos2::new(r.max.x, r.min.y), Pos2::new(r.max.x, r.max.y)], Stroke::new(1.0, t.line));
	}
}

impl Panel {
	/// A box's fill: a card (`Well`: set into one, darker)
	pub(super) fn surface(&mut self, p: &Painter, r: Rect, s: Surface) {
		let t = tk();
		// over a picture a box is a pane of glass, and a bar one without an edge
		if self.wall.shown() && s != Surface::Well {
			let view = self.view.index();
			match s {
				Surface::Card => self.wall.glass(p, r, t.radius, view, true),
				_ => {
					self.wall.glass(p, r, 0.0, view, false);
					p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.card_edge));
				}
			}
			return;
		}
		match s {
			Surface::Card => {
				let cr = CornerRadius::same(t.radius as u8);
				p.rect_filled(r, cr, t.card);
				p.rect_stroke(r, cr, Stroke::new(1.0, t.card_edge), StrokeKind::Inside);
			}
			Surface::Well => {
				p.rect_filled(r, 0.0, t.well);
				p.rect_stroke(r, 0.0, Stroke::new(1.0, t.line), StrokeKind::Inside);
			}
			Surface::Bar => {
				p.rect_filled(r, 0.0, t.card);
				p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.card_edge));
			}
		}
	}

	/// A card with its title on the title line: the rect under the line for what goes in it. What sits at the
	/// line's right end is drawn by the caller, in `header(r)`
	pub(super) fn card(&mut self, ui: &egui::Ui, r: Rect, title: &str) -> Rect {
		let t = tk();
		let p = ui.painter();
		self.surface(p, r, Surface::Card);
		let cr = CornerRadius { nw: t.radius as u8, ne: t.radius as u8, sw: 0, se: 0 };
		let hr = Rect::from_min_max(r.min + Vec2::splat(1.0), Pos2::new(r.max.x - 1.0, r.min.y + HEADER_H));
		p.rect_filled(hr, cr, t.head);
		p.line_segment([hr.left_bottom(), hr.right_bottom()], Stroke::new(1.0, t.card_edge));
		text(
			p,
			Pos2::new(r.min.x + PAD, r.min.y + HEADER_H * 0.5),
			Align2::LEFT_CENTER,
			title,
			super::theme::title(),
			t.text,
		);
		Rect::from_min_max(Pos2::new(r.min.x + PAD, r.min.y + HEADER_H + 8.0), Pos2::new(r.max.x - PAD, r.max.y - PAD))
	}

	/// A docked box: one of the boxes that lie over the room (the Tracking tab's; `k` is its place in the settings'
	/// `fold`). A card whose title line begins with the arrow that folds it to that line alone, and that takes the
	/// pointer from the room under it. Whether it is folded in this frame (`r` is its title line then): a click on
	/// the arrow is for the next. What sits in the title line is the caller's, in `dock_header(r)`
	pub(super) fn dock(&mut self, ui: &egui::Ui, r: Rect, title: &str, k: usize) -> bool {
		let t = tk();
		let folded = self.st.fold[k];
		ui.interact(r, Id::unique(("dock", k)), Sense::click_and_drag());
		self.card(ui, r, "");
		let cy = r.min.y + HEADER_H * 0.5;
		let ar = Rect::from_center_size(Pos2::new(r.min.x + PAD + 7.0, cy), Vec2::splat(24.0));
		let glyph = if folded { icons::CARET_RIGHT } else { icons::CARET_DOWN };
		let hint = if folded { "Opens this box again" } else { "Folds this box to its title line" };
		if icon_button(ui, Id::unique(("fold", k)), ar, glyph, false, true).on_hover_text(hint).clicked() {
			self.st.fold[k] = !folded;
			self.st.dirty = true;
		}
		text(
			ui.painter(),
			Pos2::new(r.min.x + PAD + FOLD_W, cy),
			Align2::LEFT_CENTER,
			title,
			super::theme::title(),
			t.text,
		);
		folded
	}

	/// A docked box's title line after its arrow, inside its margins
	pub(super) fn dock_header(r: Rect) -> Rect {
		let h = Self::header(r);
		Rect::from_min_max(Pos2::new(h.min.x + FOLD_W, h.min.y), h.max)
	}

	/// A card's title line, inside its margins
	pub(super) fn header(r: Rect) -> Rect {
		Rect::from_min_max(Pos2::new(r.min.x + PAD, r.min.y + 3.0), Pos2::new(r.max.x - 5.0, r.min.y + HEADER_H - 2.0))
	}

	/// What is under a card's title line, edge to edge (a table that fills its box)
	pub(super) fn under_header(r: Rect) -> Rect {
		Rect::from_min_max(Pos2::new(r.min.x + 1.0, r.min.y + HEADER_H + 1.0), r.max - Vec2::splat(1.0))
	}

	/// A card with its title and, at the title line's right end, a line of text (cut on its left where it does not
	/// fit): the rect under the line
	pub(super) fn boxed(
		&mut self, ui: &egui::Ui, r: Rect, title: &str, right: &str, right_col: Option<Color32>,
	) -> Rect {
		let inner = self.card(ui, r, title);
		if !right.is_empty() {
			let t = tk();
			let p = ui.painter();
			let tw = text_w(p, title, super::theme::title());
			let clip = Rect::from_min_max(
				Pos2::new(r.min.x + PAD + tw + 16.0, r.min.y),
				Pos2::new(r.max.x, r.min.y + HEADER_H),
			);
			let pc = p.with_clip_rect(clip.intersect(p.clip_rect()));
			let at = Pos2::new(r.max.x - PAD, r.min.y + HEADER_H * 0.5);
			text(&pc, at, Align2::RIGHT_CENTER, right, label(), right_col.unwrap_or(t.muted));
		}
		inner
	}

	/// Text at `pos` (its top left)
	pub(super) fn text(&self, p: &Painter, pos: Pos2, font: FontId, color: Color32, s: &str) -> Rect {
		text(p, pos, Align2::LEFT_TOP, s, font, color)
	}

	pub(super) fn text_at(&self, p: &Painter, pos: Pos2, align: Align2, font: FontId, color: Color32, s: &str) -> Rect {
		text(p, pos, align, s, font, color)
	}

	pub(super) fn text_width(p: &Painter, font: FontId, s: &str) -> f32 {
		text_w(p, s, font)
	}

	/// A line across a box
	pub(super) fn rule(&self, p: &Painter, a: Pos2, b: Pos2) {
		p.line_segment([a, b], Stroke::new(1.0, tk().card_edge));
	}

	/// A fainter line, under a row
	pub(super) fn row_rule(&self, p: &Painter, a: Pos2, b: Pos2) {
		p.line_segment([a, b], Stroke::new(1.0, tk().line));
	}

	/// An on/off button `h` tall at `pos`: its rect and whether it was clicked
	pub(super) fn pill_toggle(&self, ui: &egui::Ui, id: Id, pos: Pos2, h: f32, s: &str, on: bool) -> (Rect, bool) {
		chip(ui, id, pos, h, s, on, None, h * 0.8)
	}

	/// Is this frame for the overlay (no keyboard there: typing goes through SteamVR's)
	pub(super) fn in_overlay(&self) -> bool {
		self.view == View::Overlay
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_tables_columns_fill_its_width_the_shares_by_their_size() {
		let cols = [Col::text("a", 40.0), Col::text("b", -1.0), Col::num("c", 60.0), Col::text("d", -3.0)];
		let t = Table::new(10.0, 300.0, &cols);
		assert_eq!(t.xs[0], (10.0, 50.0));
		// 200 left over: a quarter and three quarters
		assert_eq!(t.xs[1], (50.0, 100.0));
		assert_eq!(t.xs[2], (100.0, 160.0));
		assert_eq!(t.xs[3], (160.0, 310.0));
		// narrower than its fixed columns: the shares are nothing, and their cells empty
		let t = Table::new(0.0, 80.0, &cols);
		assert_eq!(t.xs[1], (40.0, 40.0));
		assert!(t.cell(1, 0.0, TR_H).width() < 1.0);
	}
}
