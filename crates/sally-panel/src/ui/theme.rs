//! The panel's look: one set of tokens (`Tk`) every box and control is drawn from, the fonts, and egui's own style set
//! to match (its text fields and scroll bars). Flat throughout: no gradient, wash or shadow anywhere, but for the
//! glass a picture behind the panel makes of the boxes (`ui/wallpaper.rs`), which has a second set of the same
//! tokens: what is a grey on the flat panel is light or dark laid over the glass there.
//!
//! How colour works. The panel itself is black and greys: its surfaces, its lines, its controls (one that is on is
//! a lighter grey) and what is read on them. Colour is kept for what it says. Status is green, yellow and red: a
//! state where it is written (yellow: something that wants a look; red: what stops sally), a radio's loss, a strip
//! or a lane of a timeline. There is no mark beside such words: the word itself has the colour, so the colour
//! never stands alone. A device's kind has a hue of its own where the kind is written or drawn (tracker,
//! controller, headset). And the data hues tell series apart in the charts, the room and the replay (the axes,
//! Valve's tracking against sally's).
//!
//! Type: IBM Plex Sans (the variable font: one file, the weights cut from it) for what is read, IBM Plex Mono for
//! every figure and for what a machine named (serials, parameters), so a column of numbers lines up digit under
//! digit; Phosphor for the few marks on buttons. Controls are sized for the overlay's laser pointer as much as for
//! a mouse: 28 points and up.

use std::cell::Cell;

use egui::epaint::text::{FontTweak, VariationCoords};
use egui::epaint::Shadow;
use egui::{Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Stroke, TextStyle, Vec2};

/// The window's clear colour before the first frame (the tokens' background)
pub const BG: Color32 = Color32::from_rgb(10, 10, 11);

/// The look's tokens
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tk {
	/// behind everything
	pub bg: Color32,
	/// a box's fill, and the line around it
	pub card: Color32,
	pub card_edge: Color32,
	/// a table's header row, a box's title line
	pub head: Color32,
	/// every other row of a table
	pub stripe: Color32,
	/// what is set into a box: the room, a plot
	pub well: Color32,
	/// what lies over a plot: the values at its cursor
	pub float: Color32,
	/// a control at rest, under the pointer, held
	pub ctl: Color32,
	pub ctl_hover: Color32,
	pub ctl_press: Color32,
	/// a control that is on (its fill), and what is written on it
	pub on: Color32,
	pub on_ink: Color32,
	/// what is set or picked, as a line or a mark: the selected row, a tick, a slider's filled rail
	pub accent: Color32,
	/// a hairline between rows and between columns, and the stronger line where a group of columns begins
	pub line: Color32,
	pub group: Color32,
	/// The inks, from what is read to what is passed over. Each of the first three is 4.5 to 1 or more against
	/// every ground a text stands on (the header band is the lightest), the measure for text of this size; `faint`
	/// is for what is not to be read (a place without a value, a control that cannot be used) and stays 3 to 1
	/// against a box's ground
	pub text: Color32,
	pub soft: Color32,
	pub muted: Color32,
	pub faint: Color32,
	/// status: fine, no light (the IMU carrying) or something that wants a look, lost
	pub good: Color32,
	pub warn: Color32,
	pub bad: Color32,
	/// a device's kind, where it is written or drawn
	pub tracker: Color32,
	pub controller: Color32,
	pub headset: Color32,
	/// a chart's line where it has one series
	pub series: Color32,
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

/// the room's axes on a device, and a chart's x, y, z
pub const AXIS_X: Color32 = Color32::from_rgb(230, 103, 103);
pub const AXIS_Y: Color32 = Color32::from_rgb(12, 163, 12);
pub const AXIS_Z: Color32 = Color32::from_rgb(57, 135, 229);

impl Default for Tk {
	fn default() -> Tk {
		Tk {
			bg: BG,
			card: Color32::from_rgb(17, 17, 18),
			card_edge: Color32::from_rgb(64, 64, 69),
			head: Color32::from_rgb(47, 47, 52),
			stripe: Color32::from_rgb(34, 34, 37),
			well: Color32::from_rgb(6, 6, 7),
			float: Color32::from_rgb(36, 36, 39),
			ctl: Color32::from_rgb(36, 36, 39),
			ctl_hover: Color32::from_rgb(50, 50, 54),
			ctl_press: Color32::from_rgb(64, 64, 69),
			on: Color32::from_rgb(92, 92, 99),
			on_ink: Color32::from_rgb(250, 250, 250),
			accent: Color32::from_rgb(226, 226, 230),
			line: Color32::from_rgb(44, 44, 48),
			group: Color32::from_rgb(104, 104, 112),
			text: Color32::from_rgb(232, 232, 234),
			soft: Color32::from_rgb(190, 190, 194),
			muted: Color32::from_rgb(154, 154, 160),
			faint: Color32::from_rgb(110, 110, 116),
			good: Color32::from_rgb(76, 194, 98),
			warn: Color32::from_rgb(232, 184, 58),
			bad: Color32::from_rgb(240, 96, 88),
			tracker: Color32::from_rgb(150, 190, 250),
			controller: Color32::from_rgb(236, 126, 178),
			headset: Color32::from_rgb(120, 214, 214),
			series: Color32::from_rgb(76, 148, 240),
			valve: Color32::from_rgb(213, 81, 129),
			violet: Color32::from_rgb(144, 133, 233),
			grid: Color32::from_rgb(50, 50, 54),
			grid_minor: Color32::from_rgb(29, 29, 31),
			radius: 4.0,
			ctl_radius: 3.0,
		}
	}
}

impl Tk {
	/// The tokens over a picture, where every box is a pane of glass: a box's own ground is the glass (drawn by
	/// `Wallpaper::glass`), and what the flat panel sets off from it by a lighter or darker grey is white or black
	/// laid over it here, so the picture goes on under a title line, a stripe, a control. The third ink is lighter
	/// than on the flat panel: the ground under it is the brightest the picture can make it
	pub fn glass() -> Tk {
		let over = |a: u8| Color32::from_white_alpha(a);
		Tk {
			head: over(15),
			stripe: over(11),
			well: Color32::from_black_alpha(92),
			ctl: over(24),
			ctl_hover: over(40),
			ctl_press: over(56),
			on: over(84),
			card_edge: over(44),
			line: over(24),
			group: over(74),
			grid: over(40),
			grid_minor: over(18),
			muted: Color32::from_rgb(178, 178, 184),
			faint: Color32::from_rgb(132, 132, 138),
			..Tk::default()
		}
	}

	/// The colour of a status that is written out: 0 green, 2 yellow, 3 red; a note (1) is ink, and anything else is
	/// not active
	pub fn status(&self, level: i32) -> Color32 {
		match level {
			0 => self.good,
			1 => self.soft,
			2 => self.warn,
			3 => self.bad,
			_ => self.faint,
		}
	}

	/// The hue of a device's kind, by its type as the runtime names it
	pub fn kind(&self, subtype: &str) -> Color32 {
		match subtype {
			"index_hmd" | "vive_hmd" => self.headset,
			"knuckles_l" | "knuckles_r" | "wand" => self.controller,
			_ => self.tracker,
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
	medium(14.0)
}
pub fn label() -> FontId {
	medium(13.0)
}
pub fn caption() -> FontId {
	medium(12.0)
}
pub fn title() -> FontId {
	semibold(13.5)
}
pub fn name() -> FontId {
	semibold(15.0)
}
/// A figure that stands alone (a reading in a cell of its own)
pub fn figure() -> FontId {
	mono(17.0)
}
/// A figure in a table or beside its label
pub fn num() -> FontId {
	mono(13.0)
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

/// egui's own style from the tokens: what the panel does not draw itself (text fields, questions, scroll bars, tips)
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
		sp.button_padding = Vec2::new(12.0, 6.0);
		sp.interact_size = Vec2::new(30.0, 30.0);
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
		sp.window_margin = egui::Margin::same(16);
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
		wv.bg_stroke = Stroke::new(1.0, t.card_edge);
		wv.fg_stroke = Stroke::new(1.5, t.text);
		wv.corner_radius = r;
		wv.expansion = 0.0;
	}
	w.inactive.fg_stroke = Stroke::new(1.0, t.text);
	v.selection.bg_fill = alpha(t.accent, 70);
	v.selection.stroke = Stroke::new(1.0, t.accent);
	v.extreme_bg_color = t.well;
	v.text_edit_bg_color = Some(t.well);
	v.faint_bg_color = white(5);
	v.panel_fill = Color32::TRANSPARENT;
	v.window_fill = t.card;
	v.window_stroke = Stroke::new(1.0, t.card_edge);
	v.window_corner_radius = CornerRadius::same(t.radius as u8);
	v.menu_corner_radius = CornerRadius::same(t.radius as u8);
	v.window_shadow = Shadow::NONE;
	v.popup_shadow = Shadow::NONE;
	v.weak_text_color = Some(t.muted);
	v.hyperlink_color = t.accent;
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
