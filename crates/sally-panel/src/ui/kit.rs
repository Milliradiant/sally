//! The kit: what every tab is drawn with. The boxes (cards), text, and the controls the
//! panel draws itself so that they look and feel the same in the window and under the overlay's laser pointer:
//! buttons, icon buttons, switches, segmented choices, chips, the slider with its steppers. And the small pictures
//! that say a number at a glance: meters, sparklines, signal bars, the battery, a station's chip, the figure that
//! shows where on the body a device is worn.

use egui::{
	Align2, Color32, CornerRadius, FontId, Id, Order, Painter, Pos2, Rect, Response, Sense, Shape, Stroke, StrokeKind,
	Vec2,
};

use super::icons;
use super::theme::*;
use super::{Panel, View};
use crate::runtime::Device;
use crate::symbols::Symbol;

/// Between boxes, and a box's inner margin
pub const GAP: f32 = 10.0;
pub const PAD: f32 = 14.0;
/// A box's header line (its title, and what sits at its right end)
pub const HEADER_H: f32 = 40.0;
/// A control's height: buttons, chips, segmented choices
pub const CTL_H: f32 = 32.0;

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
	/// outlined in red (recording, something that cannot be undone)
	Danger,
	/// no fill until the pointer is on it
	Ghost,
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
	let us = (size * 0.6).max(11.5);
	// the unit on the value's baseline: both rects' bottoms are their fonts' descent below it
	let drop = (size - us) * 0.18;
	text(p, Pos2::new(vr.max.x + size * 0.16, pos.y - drop), Align2::LEFT_BOTTOM, unit, medium(us), t.muted).max.x
}

// ---- controls ----

fn hover_t(ui: &egui::Ui, id: Id, on: bool) -> f32 {
	ui.ctx().animate_bool_with_time(id.with("hover"), on, 0.10)
}

/// A button: an icon, a label, or both, centred in `r`
pub fn button(ui: &egui::Ui, id: Id, r: Rect, label: &str, glyph: Option<&str>, look: Look, enabled: bool) -> Response {
	let t = tk();
	let resp = ui.interact(r, id, if enabled { Sense::click() } else { Sense::hover() });
	let p = ui.painter();
	let h = hover_t(ui, id, enabled && resp.hovered());
	let down = enabled && resp.is_pointer_button_down_on();
	let cr = CornerRadius::same(t.ctl_radius as u8);
	let (fill, fg) = match look {
		Look::Plain => (if down { t.ctl_press } else { mix(t.ctl, t.ctl_hover, h) }, t.text),
		Look::Primary => (mix(t.on, t.text, if down { 0.0 } else { h * 0.35 }), t.on_ink),
		Look::Danger => (if down { alpha(t.bad, 70) } else { mix(alpha(t.bad, 26), alpha(t.bad, 52), h) }, t.text),
		Look::Ghost => (if down { t.ctl_press } else { fade(t.ctl_hover, h) }, mix(t.soft, t.text, h)),
	};
	let k = if enabled { 1.0 } else { 0.42 };
	p.rect_filled(r, cr, fade(fill, k));
	if look == Look::Danger {
		p.rect_stroke(r, cr, Stroke::new(1.0, fade(alpha(t.bad, 170), k)), StrokeKind::Inside);
	}
	let fg = fade(fg, if enabled { 1.0 } else { 0.5 });
	let font = body();
	let lw = if label.is_empty() { 0.0 } else { text_w(p, label, font.clone()) };
	let (iw, gap) = match glyph {
		Some(_) if label.is_empty() => (17.0, 0.0),
		Some(_) => (17.0, 7.0),
		None => (0.0, 0.0),
	};
	let x0 = r.center().x - (iw + gap + lw) * 0.5;
	let cy = r.center().y;
	if let Some(g) = glyph {
		icon_at(p, Pos2::new(x0 + iw * 0.5, cy), g, 17.0, fg);
	}
	if !label.is_empty() {
		let pc = p.with_clip_rect(r.shrink2(Vec2::new(4.0, 0.0)).intersect(p.clip_rect()));
		pc.text(Pos2::new(x0 + iw + gap, cy), Align2::LEFT_CENTER, label, font, fg);
	}
	resp
}

/// How wide a button is with this label (and an icon)
pub fn button_w(p: &Painter, label: &str, glyph: bool) -> f32 {
	text_w(p, label, body()) + if glyph { 24.0 } else { 0.0 } + 26.0
}

/// A square button of one icon; `on`: filled as the on state
pub fn icon_button(ui: &egui::Ui, id: Id, r: Rect, glyph: &str, on: bool, enabled: bool) -> Response {
	let t = tk();
	let resp = ui.interact(r, id, if enabled { Sense::click() } else { Sense::hover() });
	let p = ui.painter();
	let h = hover_t(ui, id, enabled && resp.hovered());
	let down = enabled && resp.is_pointer_button_down_on();
	let cr = CornerRadius::same(t.ctl_radius as u8);
	let size = (r.height() * 0.56).clamp(14.0, 20.0);
	if on {
		p.rect_filled(r, cr, mix(t.on, t.text, h * 0.3));
		p.text(r.center(), Align2::CENTER_CENTER, glyph, icon_fill(size), t.on_ink);
	} else {
		p.rect_filled(r, cr, if down { t.ctl_press } else { fade(t.ctl_hover, h) });
		let c = if enabled { mix(t.muted, t.text, h) } else { t.faint };
		icon_at(p, r.center(), glyph, size, c);
	}
	resp
}

/// A switch, 42 by 24 points at `pos`; whether it was clicked
pub fn switch(ui: &egui::Ui, id: Id, pos: Pos2, on: bool, enabled: bool) -> bool {
	let t = tk();
	let r = Rect::from_min_size(pos, Vec2::new(42.0, 24.0));
	// the target a little larger than the picture
	let resp = ui.interact(r.expand(4.0), id, if enabled { Sense::click() } else { Sense::hover() });
	let p = ui.painter();
	let k = ui.ctx().animate_bool_with_time(id.with("on"), on, 0.12);
	let h = hover_t(ui, id, enabled && resp.hovered());
	let off = mix(t.ctl, t.ctl_hover, 0.5 + 0.5 * h);
	let track = fade(mix(off, t.on, k), if enabled { 1.0 } else { 0.45 });
	p.rect_filled(r, CornerRadius::same(12), track);
	if k < 0.5 {
		p.rect_stroke(r, CornerRadius::same(12), Stroke::new(1.0, t.card_edge), StrokeKind::Inside);
	}
	let cx = r.min.x + 12.0 + k * 18.0;
	let knob = mix(t.soft, t.on_ink, k);
	p.circle_filled(Pos2::new(cx, r.center().y), 8.0 + h * 0.5, fade(knob, if enabled { 1.0 } else { 0.6 }));
	enabled && resp.clicked()
}

/// A choice of one among a few, in a row from `pos`: its rect and the one clicked
pub fn segmented(ui: &egui::Ui, id: Id, pos: Pos2, h: f32, items: &[&str], selected: usize) -> (Rect, Option<usize>) {
	let t = tk();
	let p = ui.painter();
	let font = label();
	let pad = h * 0.42;
	let widths: Vec<f32> = items.iter().map(|s| text_w(p, s, font.clone()) + pad * 2.0).collect();
	let r = Rect::from_min_size(pos, Vec2::new(widths.iter().sum::<f32>() + 4.0, h));
	let cr = CornerRadius::same(t.ctl_radius as u8);
	p.rect_filled(r, cr, t.ctl);
	let mut x = pos.x + 2.0;
	let mut clicked = None;
	for (i, (s, w)) in items.iter().zip(&widths).enumerate() {
		let cell = Rect::from_min_size(Pos2::new(x, pos.y + 2.0), Vec2::new(*w, h - 4.0));
		let resp = ui.interact(cell, id.with(i), Sense::click());
		let hv = hover_t(ui, id.with(i), resp.hovered());
		let icr = CornerRadius::same((t.ctl_radius - 2.0).max(2.0) as u8);
		if i == selected {
			p.rect_filled(cell, icr, t.on);
			p.text(cell.center(), Align2::CENTER_CENTER, s, font.clone(), t.on_ink);
		} else {
			p.rect_filled(cell, icr, fade(t.ctl_hover, hv));
			p.text(cell.center(), Align2::CENTER_CENTER, s, font.clone(), mix(t.muted, t.text, hv));
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
	items.iter().map(|s| text_w(p, s, label()) + h * 0.84).sum::<f32>() + 4.0
}

#[allow(clippy::too_many_arguments)]
/// A chip that is on or off, `h` tall at `pos`, with `pad` points around its label in all: its rect, and whether it
/// was clicked. `dot`: a state's colour before the label
pub fn chip(
	ui: &egui::Ui, id: Id, pos: Pos2, h: f32, text_: &str, on: bool, dot: Option<Color32>, pad: f32,
) -> (Rect, bool) {
	let t = tk();
	let p = ui.painter();
	let font = label();
	let dw = if dot.is_some() { 12.0 } else { 0.0 };
	let w = text_w(p, text_, font.clone()) + pad + dw;
	let r = Rect::from_min_size(pos, Vec2::new(w, h));
	let resp = ui.interact(r, id, Sense::click());
	let hv = hover_t(ui, id, resp.hovered());
	let cr = CornerRadius::same(t.ctl_radius as u8);
	let (fill, fg) = if on {
		(mix(t.on, t.text, hv * 0.3), t.on_ink)
	} else {
		(mix(t.ctl, t.ctl_hover, hv), mix(t.soft, t.text, hv))
	};
	p.rect_filled(r, cr, fill);
	let x = r.min.x + pad * 0.5;
	if let Some(c) = dot {
		p.circle_filled(Pos2::new(x + 3.5, r.center().y), 3.5, c);
	}
	p.text(Pos2::new(x + dw, r.center().y), Align2::LEFT_CENTER, text_, font, fg);
	(r, resp.clicked())
}

/// A row of chips from `pos` to `right`: where they do not fit, the room around their labels shrinks, and only then do
/// they wrap. Which one was clicked, and the last one's right end and bottom
pub fn chips(
	ui: &egui::Ui, id: Id, pos: Pos2, right: f32, h: f32, items: &[(&str, bool, Option<Color32>)],
) -> (Option<usize>, Pos2) {
	const GAP: f32 = 6.0;
	let p = ui.painter();
	let widths: Vec<f32> =
		items.iter().map(|(s, _, d)| text_w(p, s, label()) + if d.is_some() { 12.0 } else { 0.0 }).collect();
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
	let bw = r.height().min(30.0);
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
	let knob = 9.0;
	let rail =
		Rect::from_min_max(Pos2::new(minus.max.x + 8.0 + knob, r.min.y), Pos2::new(plus.min.x - 8.0 - knob, r.max.y));
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
	let track = Rect::from_min_max(Pos2::new(rail.min.x - knob, cy - 3.0), Pos2::new(rail.max.x + knob, cy + 3.0));
	p.rect_filled(track, 3.0, fade(t.ctl, k.max(0.7)));
	let x = at(*v);
	// filled from the rail's start, or from 0 where the range goes either side of it
	let filled = if range.min < 0.0 && range.max > 0.0 {
		let zero = at(0.0);
		Rect::from_min_max(Pos2::new(zero.min(x), track.min.y), Pos2::new(zero.max(x), track.max.y))
	} else {
		Rect::from_min_max(track.min, Pos2::new(x, track.max.y))
	};
	p.rect_filled(filled, 3.0, fade(mix(t.soft, t.on, 0.5), k));
	if let Some(d) = range.default {
		let dx = at(d);
		p.line_segment([Pos2::new(dx, cy + 7.0), Pos2::new(dx, cy + 12.0)], Stroke::new(1.5, fade(t.faint, k)));
	}
	if let Some(d) = range.inherits {
		p.circle_stroke(Pos2::new(at(d), cy), 5.0, Stroke::new(1.5, fade(t.note, k)));
	}
	let h = hover_t(ui, id, enabled && (resp.hovered() || resp.dragged()));
	if enabled {
		p.circle_filled(Pos2::new(x, cy + 1.0), knob + 1.5 + h, Color32::from_black_alpha(60));
	}
	p.circle_filled(Pos2::new(x, cy), knob + h * 1.5, fade(t.on, k));
	p.circle_filled(Pos2::new(x, cy), 3.0, fade(t.on_ink, k * 0.55));
	out
}

// ---- small pictures ----

/// A meter: `frac` of `r` filled
pub fn meter(p: &Painter, r: Rect, frac: f64, col: Color32) {
	let t = tk();
	let rad = r.height() * 0.5;
	p.rect_filled(r, rad, t.ctl);
	let w = (frac.clamp(0.0, 1.0) as f32) * r.width();
	if w > 0.0 {
		p.rect_filled(Rect::from_min_size(r.min, Vec2::new(w.max(rad * 2.0), r.height())), rad, col);
	}
}

/// A sparkline of values (each None a break) between `lo` and `hi` in `r`, with a wash under it
pub fn sparkline(p: &Painter, r: Rect, values: &[Option<f32>], lo: f32, hi: f32, col: Color32) {
	if values.len() < 2 {
		return;
	}
	let span = (hi - lo).max(1e-9);
	let n = values.len() - 1;
	let at = |i: usize, v: f32| {
		Pos2::new(r.min.x + r.width() * i as f32 / n as f32, r.max.y - ((v - lo) / span).clamp(0.0, 1.0) * r.height())
	};
	let mut run: Vec<Pos2> = Vec::new();
	let flush = |run: &mut Vec<Pos2>| {
		if run.len() > 1 {
			let mut mesh = egui::Mesh::default();
			for q in run.iter() {
				mesh.colored_vertex(*q, alpha(col, 46));
				mesh.colored_vertex(Pos2::new(q.x, r.max.y), alpha(col, 0));
			}
			for i in 0..run.len() as u32 - 1 {
				mesh.add_triangle(2 * i, 2 * i + 1, 2 * i + 2);
				mesh.add_triangle(2 * i + 1, 2 * i + 2, 2 * i + 3);
			}
			p.add(Shape::mesh(mesh));
			p.add(Shape::line(std::mem::take(run), Stroke::new(1.5, col)));
		}
		run.clear();
	};
	for (i, v) in values.iter().enumerate() {
		match v {
			Some(v) => run.push(at(i, *v)),
			None => flush(&mut run),
		}
	}
	flush(&mut run);
}

/// Four signal bars `h` tall, `bars` of them lit: fine from three up, amber at two, red under
pub fn signal(p: &Painter, pos: Pos2, h: f32, bars: i32) {
	let t = tk();
	let (bw, gap) = (h * 0.2, h * 0.11);
	let on = match bars {
		3.. => t.soft,
		2 => t.warn,
		_ => t.bad,
	};
	for i in 0..4 {
		let bh = h * (0.34 + 0.22 * i as f32);
		let a = Pos2::new(pos.x + i as f32 * (bw + gap), pos.y + h - bh);
		let off = t.ctl_hover;
		p.rect_filled(Rect::from_min_size(a, Vec2::new(bw, bh)), 1.0, if i < bars { on } else { off });
	}
}

/// How wide `signal` is
pub fn signal_w(h: f32) -> f32 {
	h * (0.2 * 4.0 + 0.11 * 3.0)
}

/// A battery `w` by `h` at `pos`: filled to `pct`, amber under 30, red under 15, blue while charging
pub fn battery(p: &Painter, pos: Pos2, w: f32, h: f32, pct: i32, charging: bool) {
	let t = tk();
	let body = Rect::from_min_size(pos, Vec2::new(w - 2.5, h));
	p.rect_stroke(body, 3.0, Stroke::new(1.3, t.muted), StrokeKind::Inside);
	p.rect_filled(
		Rect::from_min_max(Pos2::new(pos.x + w - 2.0, pos.y + h * 0.3), Pos2::new(pos.x + w, pos.y + h * 0.7)),
		1.0,
		t.muted,
	);
	if pct >= 0 {
		let c = if charging {
			t.note
		} else if pct < 15 {
			t.bad
		} else if pct < 30 {
			t.warn
		} else {
			t.soft
		};
		let fw = (body.width() - 5.0) * pct.min(100) as f32 / 100.0;
		let fr = Rect::from_min_max(pos + Vec2::splat(2.5), Pos2::new(pos.x + 2.5 + fw.max(1.5), pos.y + h - 2.5));
		p.rect_filled(fr, 1.5, c);
	}
}

/// A state's dot; `ring`: ringed, as a device that has lost its light is
pub fn dot(p: &Painter, c: Pos2, r: f32, col: Color32, ring: bool) {
	p.circle_filled(c, r, col);
	if ring {
		p.circle_stroke(c, r + 3.5, Stroke::new(1.3, alpha(col, 150)));
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

impl Panel {
	/// A box's fill: a card (`Well`: set into one, darker)
	pub(super) fn surface(&mut self, p: &Painter, r: Rect, s: Surface) {
		let t = tk();
		let rounding = match s {
			Surface::Card => t.radius,
			Surface::Well => t.ctl_radius,
			Surface::Bar => 0.0,
		};
		let cr = CornerRadius::same(rounding as u8);
		match s {
			Surface::Card => {
				p.rect_filled(r, cr, t.card);
				p.rect_stroke(r, cr, Stroke::new(1.0, t.card_edge), StrokeKind::Inside);
				let y = r.min.y + 1.0;
				p.line_segment(
					[Pos2::new(r.min.x + rounding, y), Pos2::new(r.max.x - rounding, y)],
					Stroke::new(1.0, t.card_hi),
				);
			}
			Surface::Well => {
				p.rect_filled(r, cr, t.well);
				p.rect_stroke(r, cr, Stroke::new(1.0, alpha(t.card_edge, 150)), StrokeKind::Inside);
			}
			Surface::Bar => {
				p.line_segment([r.left_bottom(), r.right_bottom()], Stroke::new(1.0, t.line));
			}
		}
	}

	/// A card with its title in the header line: the rect under the header for what goes in it. What sits at the
	/// header's right end is drawn by the caller, in `header(r)`
	pub(super) fn card(&mut self, ui: &egui::Ui, r: Rect, title: &str) -> Rect {
		let t = tk();
		let p = ui.painter();
		self.surface(p, r, Surface::Card);
		text(
			p,
			Pos2::new(r.min.x + PAD, r.min.y + HEADER_H * 0.5 + 1.0),
			Align2::LEFT_CENTER,
			title,
			super::theme::title(),
			t.soft,
		);
		Rect::from_min_max(Pos2::new(r.min.x + PAD, r.min.y + HEADER_H), Pos2::new(r.max.x - PAD, r.max.y - PAD))
	}

	/// A card's header line, inside its margins
	pub(super) fn header(r: Rect) -> Rect {
		Rect::from_min_max(
			Pos2::new(r.min.x + PAD, r.min.y + 4.0),
			Pos2::new(r.max.x - PAD + 6.0, r.min.y + HEADER_H - 2.0),
		)
	}

	/// A card with its title and, at the header's right end, a line of text (cut on its left where it does not fit):
	/// the rect under the header
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
			let at = Pos2::new(r.max.x - PAD, r.min.y + HEADER_H * 0.5 + 1.0);
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
		p.line_segment([a, b], Stroke::new(1.0, tk().line));
	}

	/// A fainter line, under a row
	pub(super) fn row_rule(&self, p: &Painter, a: Pos2, b: Pos2) {
		p.line_segment([a, b], Stroke::new(1.0, fade(tk().line, 0.7)));
	}

	/// A chip `h` tall at `pos`: its rect and whether it was clicked
	pub(super) fn pill_toggle(&self, ui: &egui::Ui, id: Id, pos: Pos2, h: f32, s: &str, on: bool) -> (Rect, bool) {
		chip(ui, id, pos, h, s, on, None, h * 0.8)
	}

	/// Is this frame for the overlay (no keyboard there: typing goes through SteamVR's)
	pub(super) fn in_overlay(&self) -> bool {
		self.view == View::Overlay
	}

	/// One square per station with its channel in it: filled where it lights the device (3 or more of its sensors;
	/// the note's colour with 1 or 2), an outline where it does not; the width taken
	pub(super) fn station_chips(&self, p: &Painter, pos: Pos2, h: f32, d: &Device) -> f32 {
		let t = tk();
		let fresh = self.now - d.hits_wall < 2.5;
		let font = semibold(h * 0.62);
		let mut x = pos.x;
		for (idx, sn) in &self.rt.stations {
			if !sn.have_pose && !sn.seen {
				continue;
			}
			let l = d.light.get(idx).filter(|l| fresh && l.hits > 0);
			let label = if sn.mode >= 0 { (sn.mode + 1).to_string() } else { idx.to_string() };
			let w = (text_w(p, &label, font.clone()) + h * 0.5).max(h);
			let r = Rect::from_min_size(Pos2::new(x, pos.y), Vec2::new(w, h));
			let cr = CornerRadius::same((h * 0.28) as u8);
			match l {
				Some(l) => {
					p.rect_filled(r, cr, if l.sensors_lit < 3 { t.note } else { t.good });
					p.text(r.center(), Align2::CENTER_CENTER, label, font.clone(), t.on_ink);
				}
				None => {
					p.rect_stroke(r, cr, Stroke::new(1.0, t.faint), StrokeKind::Inside);
					p.text(r.center(), Align2::CENTER_CENTER, label, font.clone(), t.faint);
				}
			}
			x += w + 3.0;
		}
		(x - pos.x - 3.0).max(0.0)
	}

	/// A pop-up under `anchor` (its right end at the anchor's when `right`), `width` wide, drawn over everything; it
	/// goes with a click anywhere else. `add` draws into it
	pub(super) fn popup(
		&mut self, ui: &egui::Ui, which: Pop, anchor: Rect, width: f32, right: bool,
		add: impl FnOnce(&mut Panel, &mut egui::Ui),
	) {
		if self.pop != Some((which, self.view)) {
			return;
		}
		let t = tk();
		let screen = ui.ctx().content_rect();
		let x = if right { anchor.max.x - width } else { anchor.min.x };
		let x = x.clamp(screen.min.x + 8.0, (screen.max.x - width - 8.0).max(screen.min.x + 8.0));
		let pos = Pos2::new(x, anchor.max.y + 8.0);
		let frame = egui::Frame::NONE
			.fill(t.float)
			.stroke(Stroke::new(1.0, t.card_edge))
			.corner_radius(CornerRadius::same(t.radius as u8))
			.inner_margin(egui::Margin::same(14))
			.shadow(ui.visuals().popup_shadow);
		let area = egui::Area::new(Id::unique(("pop", which as u8, self.view.index())))
			.order(Order::Foreground)
			.fixed_pos(pos)
			.constrain(true)
			.show(ui.ctx(), |ui| {
				frame.show(ui, |ui| {
					ui.set_width(width - 28.0);
					add(self, ui);
				});
			});
		// a click anywhere else closes it; not the click that opened it
		if area.response.clicked_elsewhere() && self.pop_frame != self.frame_no[self.view.index()] {
			self.pop = None;
		}
	}

	/// Opens a pop-up, or closes it when it is the one open
	pub(super) fn toggle_pop(&mut self, which: Pop) {
		let here = (which, self.view);
		self.pop = if self.pop == Some(here) { None } else { Some(here) };
		self.pop_frame = self.frame_no[self.view.index()];
	}
}

/// The pop-ups
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pop {
	Runtime,
	Align,
	Layers,
	Floor,
	/// where a device is worn (the Settings tab's list)
	Role,
}

/// The position sd's colour (mm): as any text under 5, amber under 20, red over
pub fn sd_color(mm: f64) -> Color32 {
	let t = tk();
	if mm < 5.0 {
		t.text
	} else if mm < 20.0 {
		t.warn
	} else {
		t.bad
	}
}
