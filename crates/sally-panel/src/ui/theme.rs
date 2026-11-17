//! The panel's look: one set of tokens (`Tk`) every box and control is drawn from, the fonts, and egui's own style set
//! to match (its text fields, pop-ups and scroll bars). Dark neutral greys, the controls in white on dark, and colour
//! kept for what a colour says: a device's state, a station's light, a value out of its range. The tokens are the
//! frame's: `set_tk` at its start, `tk()` wherever something is drawn.
//!
//! Type: IBM Plex Sans (the variable font: one file, the weights and the condensed width cut from it) for everything
//! read, its figures tabular as they come; IBM Plex Mono for what a machine named (serials, parameters); Phosphor's
//! icons. Controls are sized for the overlay's laser pointer as much as for a mouse: 32 points and up.

use std::cell::Cell;

use egui::epaint::text::{FontTweak, VariationCoords};
use egui::epaint::Shadow;
use egui::{Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Stroke, TextStyle, Vec2};

/// The window's clear colour before the first frame (the tokens' background)
pub const BG: Color32 = Color32::from_rgb(14, 15, 18);

/// The look's tokens
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tk {
	/// behind everything
	pub bg: Color32,
	/// a box's fill
	pub card: Color32,
	pub card_edge: Color32,
	/// the light along a box's top edge
	pub card_hi: Color32,
	/// what is set into a box: the room, a plot, a slider's rail
	pub well: Color32,
	/// a pop-up's fill
	pub float: Color32,
	/// a control at rest, under the pointer, held
	pub ctl: Color32,
	pub ctl_hover: Color32,
	pub ctl_press: Color32,
	/// a control that is on, and what is written on it
	pub on: Color32,
	pub on_ink: Color32,
	/// a hairline
	pub line: Color32,
	pub text: Color32,
	pub soft: Color32,
	pub muted: Color32,
	pub faint: Color32,
	/// states: fine, a note (one station, light from behind), attention, a problem
	pub good: Color32,
	pub note: Color32,
	pub warn: Color32,
	pub bad: Color32,
	/// Valve's tracking, wherever it is drawn against sally's
	pub valve: Color32,
	/// light from behind a sensor; the headset SteamVR has
	pub violet: Color32,
	/// the room's floor grid, every metre and between
	pub grid: Color32,
	pub grid_minor: Color32,
	/// a box's corners, and a control's
	pub radius: f32,
	pub ctl_radius: f32,
}

/// the room's axes on a device: x, y, z
pub const AXIS_X: Color32 = Color32::from_rgb(240, 110, 100);
pub const AXIS_Y: Color32 = Color32::from_rgb(110, 205, 140);
pub const AXIS_Z: Color32 = Color32::from_rgb(110, 160, 245);

impl Default for Tk {
	fn default() -> Tk {
		Tk {
			bg: BG,
			card: Color32::from_rgb(22, 23, 27),
			card_edge: Color32::from_rgb(37, 39, 45),
			card_hi: white(10),
			well: Color32::from_rgb(12, 13, 16),
			float: Color32::from_rgb(30, 31, 37),
			ctl: Color32::from_rgb(36, 38, 44),
			ctl_hover: Color32::from_rgb(47, 49, 57),
			ctl_press: Color32::from_rgb(59, 61, 71),
			on: Color32::from_rgb(233, 235, 241),
			on_ink: Color32::from_rgb(18, 19, 24),
			line: Color32::from_rgb(38, 40, 46),
			text: Color32::from_rgb(238, 239, 243),
			soft: Color32::from_rgb(203, 206, 216),
			muted: Color32::from_rgb(148, 152, 164),
			faint: Color32::from_rgb(96, 100, 112),
			good: Color32::from_rgb(124, 211, 164),
			note: Color32::from_rgb(122, 174, 245),
			warn: Color32::from_rgb(240, 181, 99),
			bad: Color32::from_rgb(240, 118, 106),
			valve: Color32::from_rgb(232, 143, 184),
			violet: Color32::from_rgb(176, 155, 240),
			grid: Color32::from_rgb(56, 59, 68),
			grid_minor: Color32::from_rgb(33, 35, 42),
			radius: 10.0,
			ctl_radius: 8.0,
		}
	}
}

impl Tk {
	/// The colour of a level: 0 fine, 1 a note, 2 attention, 3 a problem, else not active
	pub fn level(&self, level: i32) -> Color32 {
		match level {
			0 => self.good,
			1 => self.note,
			2 => self.warn,
			3 => self.bad,
			_ => self.faint,
		}
	}
}

thread_local! {
	static TK: Cell<Tk> = Cell::new(Tk::default());
}

/// The frame's tokens
pub fn tk() -> Tk {
	TK.with(Cell::get)
}

pub fn set_tk(t: Tk) {
	TK.with(|c| c.set(t));
}

/// A colour at another alpha (0-255)
pub fn alpha(c: Color32, a: i32) -> Color32 {
	let [r, g, b, _] = c.to_srgba_unmultiplied();
	Color32::from_rgba_unmultiplied(r, g, b, a.clamp(0, 255) as u8)
}

/// A colour's own alpha times `k`
pub fn fade(c: Color32, k: f32) -> Color32 {
	let [r, g, b, a] = c.to_srgba_unmultiplied();
	Color32::from_rgba_unmultiplied(r, g, b, (a as f32 * k.clamp(0.0, 1.0)) as u8)
}

/// White at an alpha
pub fn white(a: u8) -> Color32 {
	Color32::from_white_alpha(a)
}

/// `a` toward `b` by `t`
pub fn mix(a: Color32, b: Color32, t: f32) -> Color32 {
	let t = t.clamp(0.0, 1.0);
	let (x, y) = (a.to_srgba_unmultiplied(), b.to_srgba_unmultiplied());
	let m = |i: usize| (x[i] as f32 + (y[i] as f32 - x[i] as f32) * t).round() as u8;
	Color32::from_rgba_unmultiplied(m(0), m(1), m(2), m(3))
}

// ---- type ----

const SANS: &str = "sans";
const SANS_TEXT: &str = "sans-text";
const SANS_SEMI: &str = "sans-semi";
const SANS_COND: &str = "sans-cond";
const MONO: &str = "mono";
const ICONS: &str = "icons";
const ICONS_FILL: &str = "icons-fill";

fn named(size: f32, family: &str) -> FontId {
	FontId::new(size, FontFamily::Name(family.into()))
}

/// Plex Sans Medium: labels, controls, values
pub fn medium(size: f32) -> FontId {
	named(size, SANS)
}

/// Plex Sans Text (between Regular and Medium): sentences
pub fn regular(size: f32) -> FontId {
	named(size, SANS_TEXT)
}

/// Plex Sans SemiBold: names, titles, the figures that matter
pub fn semibold(size: f32) -> FontId {
	named(size, SANS_SEMI)
}

/// Plex Sans Medium Condensed: where a table is tight
pub fn condensed(size: f32) -> FontId {
	named(size, SANS_COND)
}

/// Plex Mono: what a machine named (serials, codenames, parameters)
pub fn mono(size: f32) -> FontId {
	named(size, MONO)
}

/// An icon (icons.rs), `size` points square
pub fn icon(size: f32) -> FontId {
	named(size, ICONS)
}

pub fn icon_fill(size: f32) -> FontId {
	named(size, ICONS_FILL)
}

/// The scale: what is read at a glance to what is looked up
pub fn body() -> FontId {
	medium(15.0)
}
pub fn label() -> FontId {
	medium(13.0)
}
pub fn caption() -> FontId {
	medium(12.0)
}
pub fn title() -> FontId {
	semibold(14.0)
}
pub fn name() -> FontId {
	semibold(16.0)
}
pub fn figure() -> FontId {
	semibold(21.0)
}
pub fn code() -> FontId {
	mono(12.5)
}

/// The fonts and the style, once for each egui context the panel draws into
pub fn setup(ctx: &egui::Context) {
	let mut fonts = FontDefinitions::default();
	let plex: &'static [u8] = include_bytes!("../../fonts/IBMPlexSansVar-Roman.ttf");
	let cut = |wght: f32, wdth: f32| {
		FontData::from_static(plex)
			.tweak(FontTweak { coords: VariationCoords::new([(b"wght", wght), (b"wdth", wdth)]), ..Default::default() })
	};
	let own = [
		(SANS, cut(500.0, 100.0)),
		(SANS_TEXT, cut(450.0, 100.0)),
		(SANS_SEMI, cut(600.0, 100.0)),
		(SANS_COND, cut(500.0, 88.0)),
		(MONO, FontData::from_static(include_bytes!("../../fonts/IBMPlexMono-Medium.ttf"))),
		(ICONS, FontData::from_static(include_bytes!("../../fonts/Phosphor-Bold.ttf"))),
		(ICONS_FILL, FontData::from_static(include_bytes!("../../fonts/Phosphor-Fill.ttf"))),
	];
	// egui's own fonts stay behind ours for the characters ours have not
	let prop = fonts.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
	let monof = fonts.families.get(&FontFamily::Monospace).cloned().unwrap_or_default();
	let with = |first: &str, rest: &[String]| -> Vec<String> {
		std::iter::once(first.to_string()).chain(rest.iter().cloned()).collect()
	};
	for (key, data) in own {
		fonts.font_data.insert(key.into(), data.into());
		let rest = if key == MONO { &monof } else { &prop };
		fonts.families.insert(FontFamily::Name(key.into()), with(key, rest));
	}
	fonts.families.insert(FontFamily::Proportional, with(SANS, &prop));
	fonts.families.insert(FontFamily::Monospace, with(MONO, &monof));
	ctx.set_fonts(fonts);
	ctx.options_mut(|o| {
		// Ctrl +/- would zoom the whole panel
		o.zoom_with_keyboard = false;
	});
	apply(ctx, &tk());
}

/// egui's own style from the tokens: what the panel does not draw itself (text fields, pop-ups, scroll bars, tips)
pub fn apply(ctx: &egui::Context, t: &Tk) {
	let t = *t;
	ctx.all_styles_mut(|s| {
		s.text_styles = [
			(TextStyle::Small, caption()),
			(TextStyle::Body, body()),
			(TextStyle::Button, body()),
			(TextStyle::Heading, name()),
			(TextStyle::Monospace, code()),
		]
		.into();
		let sp = &mut s.spacing;
		sp.item_spacing = Vec2::new(8.0, 8.0);
		sp.button_padding = Vec2::new(12.0, 7.0);
		sp.interact_size = Vec2::new(32.0, 32.0);
		sp.icon_width = 20.0;
		sp.icon_width_inner = 11.0;
		sp.icon_spacing = 8.0;
		sp.slider_rail_height = 6.0;
		sp.combo_height = 420.0;
		sp.scroll.floating = false;
		sp.scroll.bar_width = 10.0;
		sp.scroll.bar_inner_margin = 2.0;
		sp.scroll.bar_outer_margin = 2.0;
		sp.scroll.handle_min_length = 24.0;
		sp.scroll.foreground_color = false;
		sp.tooltip_width = 460.0;
		sp.menu_margin = egui::Margin::same(8);
		sp.window_margin = egui::Margin::same(18);
		s.interaction.tooltip_delay = 0.3;
		s.visuals = visuals(&t);
	});
}

fn visuals(t: &Tk) -> egui::Visuals {
	let mut v = egui::Visuals::dark();
	let r = CornerRadius::same(t.ctl_radius as u8);
	let w = &mut v.widgets;
	w.noninteractive.bg_fill = Color32::TRANSPARENT;
	w.noninteractive.weak_bg_fill = Color32::TRANSPARENT;
	w.noninteractive.bg_stroke = Stroke::new(1.0, t.line);
	w.noninteractive.fg_stroke = Stroke::new(1.0, t.text);
	w.noninteractive.corner_radius = r;
	for (wv, bg) in [
		(&mut w.inactive, t.ctl),
		(&mut w.hovered, t.ctl_hover),
		(&mut w.active, t.ctl_press),
		(&mut w.open, t.ctl_press),
	] {
		wv.bg_fill = bg;
		wv.weak_bg_fill = bg;
		wv.bg_stroke = Stroke::NONE;
		wv.fg_stroke = Stroke::new(1.5, t.text);
		wv.corner_radius = r;
		wv.expansion = 0.0;
	}
	w.inactive.fg_stroke = Stroke::new(1.0, t.text);
	v.selection.bg_fill = alpha(t.on, 60);
	v.selection.stroke = Stroke::new(1.5, t.on);
	v.extreme_bg_color = t.ctl;
	v.text_edit_bg_color = Some(t.ctl);
	v.faint_bg_color = white(6);
	v.panel_fill = Color32::TRANSPARENT;
	v.window_fill = t.float;
	v.window_stroke = Stroke::new(1.0, t.card_edge);
	v.window_corner_radius = CornerRadius::same(t.radius as u8);
	v.menu_corner_radius = CornerRadius::same(t.radius as u8);
	v.window_shadow = Shadow { offset: [0, 10], blur: 28, spread: 0, color: Color32::from_black_alpha(110) };
	v.popup_shadow = Shadow { offset: [0, 6], blur: 18, spread: 0, color: Color32::from_black_alpha(90) };
	v.weak_text_color = Some(t.muted);
	v.hyperlink_color = t.on;
	v.slider_trailing_fill = true;
	v.override_text_color = None;
	v
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn colours_mix_and_fade() {
		assert_eq!(mix(Color32::BLACK, Color32::WHITE, 0.0), Color32::BLACK);
		assert_eq!(mix(Color32::BLACK, Color32::WHITE, 1.0), Color32::WHITE);
		assert_eq!(alpha(Color32::from_rgb(10, 20, 30), 128).to_srgba_unmultiplied(), [10, 20, 30, 128]);
		assert_eq!(fade(Color32::from_rgb(10, 20, 30), 0.5).to_srgba_unmultiplied()[3], 127);
	}
}
