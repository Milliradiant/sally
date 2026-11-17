//! The device page: one device over its last seconds (10 to 120, 30 by default), beside the devices' list (which picks
//! its device): which stations' light it got and its state, its position, orientation, speed, pose rate and position sd
//! as charts with one time cursor across them, and under each chart its series over the range (now, min, max, mean,
//! peak to peak). Pause holds the range still. Selected is the device now; this is how it got there. The range is on
//! the runtime's clock, as the history is. Its Sensors view is `sensors.rs`'s.

use egui::{Align2, Color32, Id, Pos2, Rect, Sense, Shape, Stroke, Vec2};

use super::icons;
use super::kit::{self, sd_color, Surface, GAP, HEADER_H, PAD};
use super::theme;
use super::theme::*;
use super::{Panel, Sel};
use crate::history::{yaw_pitch_roll, Light, Sample};
use crate::runtime::Device;

#[derive(Default)]
pub struct PageState {
	/// the end of the range Pause holds (the runtime's clock)
	pub paused_at: Option<f64>,
	/// the time under the pointer, on any chart or strip, in each view
	pub hover: [Option<f64>; 2],
	/// the Sensors tab is open (else the curves)
	pub sensors_tab: bool,
	pub sensors: super::sensors::SensorsState,
}

const RANGES: [f64; 4] = [10.0, 30.0, 60.0, 120.0];

/// A sample of the range with its yaw, pitch and roll worked out once: three series take them, for the chart's scale,
/// its line and the table, from every sample every frame (each of those a yaw_pitch_roll of its own was 9 a sample)
#[derive(Clone, Copy)]
struct Point {
	s: Sample,
	ypr: (f64, f64, f64),
}

impl Point {
	fn of(s: &Sample) -> Point {
		Point { s: *s, ypr: yaw_pitch_roll(&s.rot) }
	}
}

/// A series of a chart: its name, colour (of the frame's tokens) and value of a sample
struct Series {
	name: &'static str,
	color: fn(&Tk) -> Color32,
	value: fn(&Point) -> f64,
}

struct Chart {
	title: &'static str,
	unit: &'static str,
	series: Vec<Series>,
	/// decimals of its values at the cursor and in the table
	decimals: usize,
	/// drawn as steps (a value held until the next) instead of lines
	steps: bool,
	/// the axis starts at 0
	from_zero: bool,
	/// angles in -180..180: the line is broken where one wraps round instead of crossing the chart
	wraps: bool,
}

impl Chart {
	/// The value of a sample, None where it is unknown (an sd of -1)
	fn value(&self, se: &Series, p: &Point) -> Option<f64> {
		let v = (se.value)(p);
		(v.is_finite() && !(self.from_zero && v < 0.0)).then_some(v)
	}
}

/// A lane of the light's strip, and a row of a chart's table
const LANE: f32 = 20.0;
const ROW: f32 = 19.0;

fn charts() -> Vec<Chart> {
	// one series: the theme's light ink, so that colour stays what it says elsewhere (a state, an axis)
	let ink: fn(&Tk) -> Color32 = |t| t.soft;
	vec![
		Chart {
			title: "Position",
			unit: "m",
			series: vec![
				Series { name: "x", color: |_| AXIS_X, value: |p| p.s.pos.x },
				Series { name: "y", color: |_| AXIS_Y, value: |p| p.s.pos.y },
				Series { name: "z", color: |_| AXIS_Z, value: |p| p.s.pos.z },
			],
			decimals: 3,
			steps: false,
			from_zero: false,
			wraps: false,
		},
		Chart {
			title: "Orientation",
			unit: "°",
			series: vec![
				Series { name: "yaw", color: |_| AXIS_Z, value: |p| p.ypr.0 },
				Series { name: "pitch", color: |_| AXIS_Y, value: |p| p.ypr.1 },
				Series { name: "roll", color: |_| AXIS_X, value: |p| p.ypr.2 },
			],
			decimals: 1,
			steps: false,
			from_zero: false,
			wraps: true,
		},
		Chart {
			title: "Speed",
			unit: "m/s",
			series: vec![Series { name: "speed", color: ink, value: |p| p.s.speed }],
			decimals: 2,
			steps: false,
			from_zero: true,
			wraps: false,
		},
		Chart {
			title: "Pose rate",
			unit: "a second",
			series: vec![Series { name: "rate", color: ink, value: |p| p.s.rate }],
			decimals: 0,
			steps: true,
			from_zero: true,
			wraps: false,
		},
		Chart {
			title: "Position, how sure",
			unit: "± mm",
			series: vec![Series { name: "± pos", color: |t| t.good, value: |p| p.s.sd_mm }],
			decimals: 1,
			steps: false,
			from_zero: true,
			wraps: false,
		},
		Chart {
			title: "Light's residual",
			unit: "mrad",
			series: vec![Series { name: "residual", color: |t| t.violet, value: |p| p.s.res_mrad }],
			decimals: 2,
			steps: false,
			from_zero: true,
			wraps: false,
		},
	]
}

/// Ticks for a range: a step of 1, 2 or 5 times a power of ten giving 3 to 6 of them
fn ticks(lo: f64, hi: f64) -> Vec<f64> {
	let span = (hi - lo).max(1e-9);
	let p = 10f64.powf((span / 4.0).log10().floor());
	let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * p).find(|s| span / s <= 5.0).unwrap_or(10.0 * p);
	let mut t = (lo / step).ceil() * step;
	let mut out = Vec::new();
	while t <= hi + step * 1e-6 && out.len() < 12 {
		out.push(if t.abs() < step * 1e-6 { 0.0 } else { t });
		t += step;
	}
	out
}

/// A tick's label: as many decimals as the step needs
fn tick_label(v: f64, step: f64) -> String {
	let d = if step >= 1.0 { 0 } else { (-step.log10()).ceil().clamp(0.0, 6.0) as usize };
	format!("{v:.d$}")
}

impl Panel {
	/// How wide the devices' list is beside the room, and beside a device's page
	pub(super) fn list_width(w: f32) -> f32 {
		(w * 0.245).clamp(330.0, 392.0)
	}

	pub(super) fn device_page(&mut self, ui: &mut egui::Ui, r: Rect) {
		// the list stays beside the page where there is room for both, and picks its device
		let with_list = r.width() >= 1300.0;
		if with_list {
			if let Sel::Device(c) = &self.sel {
				if self.page.as_deref() != Some(c) && self.rt.devices.get(c).is_some_and(|d| d.active) {
					self.page = Some(c.clone());
					self.page_state.hover = [None, None];
				}
			}
		}
		let Some(code) = self.page.clone() else { return };
		let r = if with_list {
			let lw = Self::list_width(r.width());
			self.devices_card(ui, Rect::from_min_size(r.min, Vec2::new(lw, r.height())));
			if self.page.is_none() {
				return;
			}
			Rect::from_min_max(Pos2::new(r.min.x + lw + GAP, r.min.y), r.max)
		} else {
			r
		};
		// a copy for the boxes (which change the panel's state as they draw) without the history, whose range is taken
		// from the device itself below: a whole copy a frame was its 120 s of samples
		let Some(d) = self.rt.devices.get_mut(&code).map(Device::copy_light) else {
			self.page = None;
			return;
		};
		let range = if RANGES.contains(&self.st.page_range) { self.st.page_range } else { 30.0 };
		let t1 = self.page_state.paused_at.unwrap_or(self.rt.run_time);
		let span = (t1 - range, t1);

		let head_h = 108.0;
		if !self.page_head(ui, Rect::from_min_size(r.min, Vec2::new(r.width(), head_h)), &d, !with_list) {
			return;
		}
		let mut y = r.min.y + head_h + GAP;
		if self.page_state.sensors_tab {
			self.page_sensors(ui, Rect::from_min_max(Pos2::new(r.min.x, y), r.max), &d);
			ui.ctx().request_repaint();
			return;
		}

		let samples: Vec<Point> = self
			.rt
			.devices
			.get(&code)
			.map_or(Vec::new(), |dev| dev.history.since(span.0).filter(|s| s.t <= t1).map(Point::of).collect());
		let stations: Vec<(i32, String)> = self
			.rt
			.stations
			.iter()
			.filter(|(_, s)| s.have_pose || s.seen)
			.map(|(i, s)| (*i, s.channel_label()))
			.collect();
		let light_h = HEADER_H + LANE * (stations.len() as f32 + 1.0) + 22.0 + PAD - 4.0;
		let light_r = Rect::from_min_size(Pos2::new(r.min.x, y), Vec2::new(r.width(), light_h));
		let mut hover = self.page_light(ui, light_r, &stations, &samples, span);
		y += light_h + GAP;

		// the charts in two rows of three, each with its numbers under it; the residual's chart only where the runtime
		// sends it (S lines' eleventh field), the position sd's chart over its slot too otherwise
		let mut cs = charts();
		let has_res = samples.iter().any(|p| p.s.res_mrad >= 0.0);
		if !has_res {
			cs.retain(|c| c.title != "Light's residual");
		}
		let row_h = ((r.max.y - y - GAP) / 2.0).max(150.0);
		let w3 = (r.width() - 2.0 * GAP) / 3.0;
		for (i, ch) in cs.iter().enumerate() {
			let (col, row) = ((i % 3) as f32, (i / 3) as f32);
			let w = if !has_res && ch.title == "Position, how sure" { 2.0 * w3 + GAP } else { w3 };
			let at = Pos2::new(r.min.x + col * (w3 + GAP), y + row * (row_h + GAP));
			if let Some(t) = self.page_chart(ui, Rect::from_min_size(at, Vec2::new(w, row_h)), ch, &samples, span, i) {
				hover = Some(t);
			}
		}
		self.page_state.hover[self.view.index()] = hover;
		if self.page_state.paused_at.is_none() {
			ui.ctx().request_repaint();
		}
	}

	/// The page's head: back to the room, the device (its symbol, name, what it is), the view (curves or sensors), the
	/// curves' range and pause; under them its numbers now. `switcher`: with arrows to the devices before and after it
	/// (where the list is not beside the page). False when the page was left
	fn page_head(&mut self, ui: &mut egui::Ui, r: Rect, d: &Device, switcher: bool) -> bool {
		let t = theme::tk();
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Card);
		let h = self.health_of(d);
		let x0 = r.min.x + PAD;
		let xr = r.max.x - PAD;
		let cy = r.min.y + 30.0;
		let back = Rect::from_min_size(Pos2::new(x0 - 2.0, cy - 16.0), Vec2::splat(32.0));
		if kit::icon_button(ui, Id::unique("page-back"), back, icons::ARROW_LEFT, false, true)
			.on_hover_text("Back to the room")
			.clicked()
		{
			self.page = None;
			return false;
		}
		let mut x = back.max.x + 8.0;
		if switcher {
			// the devices in the list's order, round
			let codes: Vec<String> = self.active_devices().iter().map(|d| d.codename.clone()).collect();
			let at = codes.iter().position(|c| *c == d.codename).unwrap_or(0);
			for (k, glyph, step) in [(0, icons::CARET_LEFT, codes.len().saturating_sub(1)), (1, icons::CARET_RIGHT, 1)]
			{
				let br = Rect::from_min_size(Pos2::new(x, cy - 16.0), Vec2::splat(32.0));
				if kit::icon_button(ui, Id::unique(("page-step", k)), br, glyph, false, codes.len() > 1).clicked() {
					let next = codes[(at + step) % codes.len().max(1)].clone();
					self.open_device_page(&next);
				}
				x = br.max.x + 2.0;
			}
			x += 6.0;
		}
		self.draw_symbol(
			&p,
			Rect::from_min_size(Pos2::new(x, cy - 17.0), Vec2::new(36.0, 34.0)),
			d,
			if h.level <= 1 { t.soft } else { h.color },
		);
		x += 46.0;

		// from the right: the view, the range, pause
		let bh = 30.0;
		let views = ["Curves", "Sensors"];
		let vw = kit::segmented_w(&p, bh, &views);
		let (_, c) = kit::segmented(
			ui,
			Id::unique("page-view"),
			Pos2::new(xr - vw, cy - bh * 0.5),
			bh,
			&views,
			self.page_state.sensors_tab as usize,
		);
		if let Some(i) = c {
			self.page_state.sensors_tab = i == 1;
		}
		let mut right = xr - vw - 12.0;
		if !self.page_state.sensors_tab {
			let paused = self.page_state.paused_at.is_some();
			let pr = Rect::from_min_size(Pos2::new(right - bh, cy - bh * 0.5), Vec2::splat(bh));
			let glyph = if paused { icons::PLAY } else { icons::PAUSE };
			let tip = if paused { "Goes on" } else { "Holds the range still" };
			if kit::icon_button(ui, Id::unique("page-pause"), pr, glyph, paused, true).on_hover_text(tip).clicked() {
				self.page_state.paused_at = if paused { None } else { Some(self.rt.run_time) };
			}
			right = pr.min.x - 8.0;
			let labels: Vec<String> = RANGES.iter().map(|rg| format!("{rg:.0} s")).collect();
			let items: Vec<&str> = labels.iter().map(String::as_str).collect();
			let rw = kit::segmented_w(&p, bh, &items);
			let range = if RANGES.contains(&self.st.page_range) { self.st.page_range } else { 30.0 };
			let sel = RANGES.iter().position(|rg| *rg == range).unwrap_or(1);
			let (_, c) =
				kit::segmented(ui, Id::unique("page-range"), Pos2::new(right - rw, cy - bh * 0.5), bh, &items, sel);
			if let Some(i) = c {
				self.st.page_range = RANGES[i];
				self.st.dirty = true;
			}
			right -= rw + 12.0;
		}

		// the name, and what it is
		let clip = Rect::from_min_max(Pos2::new(x, r.min.y + 8.0), Pos2::new(right, r.min.y + 32.0));
		kit::text_in(&p, clip, &self.short_name(d), semibold(20.0), t.text);
		let sub = self.type_of(d);
		let sr = kit::text(&p, Pos2::new(x, r.min.y + 43.0), Align2::LEFT_CENTER, &sub, theme::label(), t.muted);
		let mut id = format!("{}  {}", d.codename, d.serial);
		if !d.dongle.is_empty() && d.dongle != "-" {
			id += &format!("  dongle {}", d.dongle);
		}
		let clip = Rect::from_min_max(Pos2::new(sr.max.x + 10.0, r.min.y + 33.0), Pos2::new(right, r.min.y + 53.0));
		kit::text_in(&p, clip, &id, code(), t.faint);

		// its numbers now
		let sdmm = if d.pos_sd >= 0.0 { d.pos_sd * 1000.0 } else { -1.0 };
		let charge = d.charge();
		let dash = || "–".to_string();
		let fields: Vec<(&str, String, &str, Color32)> = vec![
			("State", h.state.clone(), "", if h.level <= 1 { t.good } else { h.color }),
			("Pose rate", format!("{:.0}", d.rate_hz), "/s", t.text),
			("Position", if sdmm < 0.0 { dash() } else { format!("±{sdmm:.1}") }, "mm", sd_color(sdmm)),
			("Light's age", if d.light_age < 0.0 { dash() } else { format!("{:.3}", d.light_age) }, "s", t.text),
			("Speed", format!("{:.2}", sally_math::math3d::len(d.vel)), "m/s", t.text),
			("Height", format!("{:.2}", d.pos.z - self.floor_z()), "m", t.text),
			(
				"Battery",
				if charge < 0 { dash() } else { charge.to_string() },
				if charge < 0 { "" } else { "%" },
				t.text,
			),
		];
		let fy = r.min.y + 64.0;
		p.line_segment([Pos2::new(r.min.x + 1.0, fy), Pos2::new(r.max.x - 1.0, fy)], Stroke::new(1.0, t.line));
		let cw = ((xr - x0) / fields.len() as f32).min(190.0);
		for (i, (k, v, unit, c)) in fields.iter().enumerate() {
			let fx = x0 + i as f32 * cw;
			kit::text(&p, Pos2::new(fx, fy + 13.0), Align2::LEFT_CENTER, k, caption(), t.muted);
			kit::value_unit(&p, Pos2::new(fx, fy + 38.0), v, unit, semibold(16.0), *c);
		}
		true
	}

	/// A strip per station over the range, filled where the tracker got its sweeps (S lines' station mask), and one
	/// for the state (tracking, dark, coasting); the time under the pointer
	fn page_light(
		&mut self, ui: &mut egui::Ui, r: Rect, stations: &[(i32, String)], samples: &[Point], span: (f64, f64),
	) -> Option<f64> {
		let t = theme::tk();
		let inner = self.boxed(ui, r, "Light", "the stations whose sweeps it got, and its state", None);
		let p = ui.painter().clone();
		let (t0, t1) = span;
		let (x0, x1) = (inner.min.x + 58.0, inner.max.x);
		let tx = |t: f64| x0 + ((t - t0) / (t1 - t0)) as f32 * (x1 - x0);
		let rows = stations.len() + 1;
		for i in 0..rows {
			let y = inner.min.y + i as f32 * LANE;
			let name = if i < stations.len() { stations[i].1.as_str() } else { "state" };
			kit::text(
				&p,
				Pos2::new(inner.min.x, y + LANE * 0.5 - 1.0),
				Align2::LEFT_CENTER,
				name,
				theme::label(),
				t.muted,
			);
			let lane = Rect::from_min_max(Pos2::new(x0, y + 3.0), Pos2::new(x1, y + LANE - 5.0));
			p.rect_filled(lane, 3.0, t.ctl);
			// runs of samples with the bit set (or in one state), split where samples are over 0.25 s apart
			let mut run: Option<(f64, f64, Color32)> = None;
			let flush = |run: &mut Option<(f64, f64, Color32)>| {
				if let Some((a, b, c)) = run.take() {
					let rr =
						Rect::from_min_max(Pos2::new(tx(a), lane.min.y), Pos2::new(tx(b).max(tx(a) + 1.0), lane.max.y));
					p.rect_filled(rr, 2.0, c);
				}
			};
			let mut last_t = f64::NEG_INFINITY;
			for s in samples.iter().map(|pt| &pt.s) {
				let c = if i < stations.len() {
					(s.lh_mask & (1u32 << stations[i].0) != 0).then_some(alpha(t.good, 215))
				} else {
					Some(match s.light {
						Light::Tracking if s.lh_mask.count_ones() >= 2 => alpha(t.good, 130),
						Light::Tracking => t.note,
						Light::Dark => t.warn,
						Light::Coasting => t.bad,
						Light::Other => t.faint,
					})
				};
				let gap = s.t - last_t > 0.25;
				last_t = s.t;
				match (&mut run, c) {
					(Some((_, b, rc)), Some(c)) if !gap && *rc == c => *b = s.t,
					(_, Some(c)) => {
						flush(&mut run);
						run = Some((s.t, s.t, c));
					}
					(_, None) => flush(&mut run),
				}
			}
			flush(&mut run);
		}
		let ay = inner.min.y + rows as f32 * LANE + 2.0;
		self.time_axis(&p, x0, x1, ay, span);
		let area = Rect::from_min_max(Pos2::new(x0, inner.min.y), Pos2::new(x1, ay));
		let resp = ui.interact(area, Id::unique("page-light"), Sense::hover());
		let hover = resp.hover_pos().map(|hp| t0 + ((hp.x - x0) / (x1 - x0)) as f64 * (t1 - t0));
		if let Some(t_) = hover.or(self.page_state.hover[self.view.index()]) {
			let x = tx(t_);
			if (x0..=x1).contains(&x) {
				p.line_segment(
					[Pos2::new(x, inner.min.y), Pos2::new(x, ay - 2.0)],
					Stroke::new(1.0, alpha(t.text, 140)),
				);
			}
		}
		hover
	}

	/// The time axis's labels under a range: "-30 s" ... "now"
	fn time_axis(&self, p: &egui::Painter, x0: f32, x1: f32, y: f32, span: (f64, f64)) {
		let t = theme::tk();
		let range = span.1 - span.0;
		// the smallest round step that keeps the labels 64 points apart
		let w = (x1 - x0).max(1.0) as f64;
		let step = [1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0].into_iter().find(|s| s / range * w >= 64.0).unwrap_or(range);
		let mut k: f64 = 0.0;
		while k <= range + 1e-6 {
			let x = x1 - (k / range) as f32 * (x1 - x0);
			let says = match (k == 0.0, self.page_state.paused_at.is_some()) {
				(true, true) => "paused".to_string(),
				(true, false) => "now".to_string(),
				_ => format!("-{k:.0} s"),
			};
			let align = if k == 0.0 {
				Align2::RIGHT_TOP
			} else if (k - range).abs() < 1e-6 {
				Align2::LEFT_TOP
			} else {
				Align2::CENTER_TOP
			};
			kit::text(p, Pos2::new(x, y), align, &says, caption(), t.faint);
			k += step;
		}
	}

	/// A chart, and under it each of its series over the range (now, min, max, mean, peak to peak: the rows are its
	/// legend too); the time under the pointer when it is over it
	fn page_chart(
		&mut self, ui: &mut egui::Ui, r: Rect, ch: &Chart, samples: &[Point], span: (f64, f64), i: usize,
	) -> Option<f64> {
		let t = theme::tk();
		let inner = self.card(ui, r, ch.title);
		let p = ui.painter().clone();
		let tw = kit::text_w(&p, ch.title, title());
		kit::text(
			&p,
			Pos2::new(inner.min.x + tw + 8.0, r.min.y + HEADER_H * 0.5 + 1.0),
			Align2::LEFT_CENTER,
			ch.unit,
			theme::label(),
			t.muted,
		);
		let (t0, t1) = span;
		let table_h = 20.0 + ch.series.len() as f32 * ROW;
		let plot = Rect::from_min_max(
			Pos2::new(inner.min.x + 40.0, inner.min.y + 2.0),
			Pos2::new(inner.max.x - 2.0, inner.max.y - table_h - 24.0),
		);
		// (each sample's values straight in: a list of them a sample was the frame's most allocations)
		let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
		for s in samples {
			for v in ch.series.iter().filter_map(|se| ch.value(se, s)) {
				lo = lo.min(v);
				hi = hi.max(v);
			}
		}
		self.chart_table(&p, Rect::from_min_max(Pos2::new(inner.min.x, inner.max.y - table_h), inner.max), ch, samples);
		if !lo.is_finite() {
			kit::text(&p, plot.center(), Align2::CENTER_CENTER, "no samples", theme::label(), t.muted);
			return None;
		}
		if ch.from_zero {
			lo = lo.min(0.0);
		}
		// room above and below the lines; an axis from 0 starts at its bottom edge (a speed is never below 0)
		let pad = ((hi - lo) * 0.08).max(10f64.powi(-(ch.decimals as i32)));
		let (lo, hi) = (if ch.from_zero { lo } else { lo - pad }, hi + pad);
		let ty = |v: f64| plot.max.y - ((v - lo) / (hi - lo)) as f32 * plot.height();
		let tx = |t: f64| plot.min.x + ((t - t0) / (t1 - t0)) as f32 * plot.width();
		let marks = ticks(lo, hi);
		let tstep = if marks.len() > 1 { marks[1] - marks[0] } else { 1.0 };
		for v in &marks {
			let y = ty(*v);
			let grid = if *v == 0.0 { t.grid } else { fade(t.line, 0.8) };
			p.line_segment([Pos2::new(plot.min.x, y), Pos2::new(plot.max.x, y)], Stroke::new(1.0, grid));
			kit::text(
				&p,
				Pos2::new(plot.min.x - 7.0, y),
				Align2::RIGHT_CENTER,
				&tick_label(*v, tstep),
				caption(),
				t.faint,
			);
		}
		self.time_axis(&p, plot.min.x, plot.max.x, plot.max.y + 5.0, span);
		// the lines, thinned to about two samples a point, broken where samples are over 0.25 s apart; a chart of one
		// series from 0 has a wash under its line
		let step = (samples.len() as f32 / (plot.width() * 2.0)).ceil().max(1.0) as usize;
		let pc = p.with_clip_rect(plot.expand(1.0).intersect(p.clip_rect()));
		let wash = ch.series.len() == 1 && ch.from_zero && !ch.steps;
		for se in &ch.series {
			let col = (se.color)(&t);
			let mut line: Vec<Pos2> = Vec::new();
			let mut last_t = f64::NEG_INFINITY;
			let mut last_v = f64::NAN;
			let draw = |line: &mut Vec<Pos2>| {
				if line.len() > 1 {
					if wash {
						let mut mesh = egui::Mesh::default();
						for q in line.iter() {
							mesh.colored_vertex(*q, alpha(col, 34));
							mesh.colored_vertex(Pos2::new(q.x, plot.max.y), alpha(col, 0));
						}
						for k in 0..line.len() as u32 - 1 {
							mesh.add_triangle(2 * k, 2 * k + 1, 2 * k + 2);
							mesh.add_triangle(2 * k + 1, 2 * k + 2, 2 * k + 3);
						}
						pc.add(Shape::mesh(mesh));
					}
					pc.add(Shape::line(std::mem::take(line), Stroke::new(1.6, col)));
				}
				line.clear();
			};
			for s in samples.iter().step_by(step) {
				let v = ch.value(se, s);
				let wrapped = ch.wraps && v.is_some_and(|v| (v - last_v).abs() > 180.0);
				if v.is_none() || s.s.t - last_t > 0.25 * step as f64 || wrapped {
					draw(&mut line);
				}
				last_t = s.s.t;
				last_v = v.unwrap_or(f64::NAN);
				if let Some(v) = v {
					let q = Pos2::new(tx(s.s.t), ty(v));
					if ch.steps {
						if let Some(prev) = line.last().copied() {
							line.push(Pos2::new(q.x, prev.y));
						}
					}
					line.push(q);
				}
			}
			draw(&mut line);
		}
		// the cursor: the time under the pointer, on every chart at once; the values of this one beside it
		let resp = ui.interact(plot, Id::unique(("page-chart", i)), Sense::hover());
		let hover_t = resp.hover_pos().map(|hp| t0 + ((hp.x - plot.min.x) / plot.width()) as f64 * (t1 - t0));
		let Some(at_t) = hover_t.or(self.page_state.hover[self.view.index()]) else { return hover_t };
		let x = tx(at_t);
		if !(plot.min.x..=plot.max.x).contains(&x) {
			return hover_t;
		}
		p.line_segment([Pos2::new(x, plot.min.y), Pos2::new(x, plot.max.y)], Stroke::new(1.0, alpha(t.text, 140)));
		let at = samples.partition_point(|s| s.s.t < at_t).min(samples.len().saturating_sub(1));
		let Some(s) = samples.get(at) else { return hover_t };
		for se in &ch.series {
			if let Some(v) = ch.value(se, s) {
				let c = Pos2::new(x, ty(v));
				p.circle_filled(c, 4.5, t.card);
				p.circle_stroke(c, 3.5, Stroke::new(1.6, (se.color)(&t)));
			}
		}
		if hover_t.is_some() {
			// the values at the cursor: the time, then each series by its colour's mark
			let d = ch.decimals;
			let lines: Vec<(Option<Color32>, String, String)> =
				std::iter::once((None, "at".to_string(), format!("{:.2} s", s.s.t - t1)))
					.chain(ch.series.iter().map(|se| {
						let v = ch.value(se, s).map(|v| format!("{v:.d$}")).unwrap_or_else(|| "–".into());
						(Some((se.color)(&t)), se.name.to_string(), v)
					}))
					.collect();
			let kw = lines.iter().map(|l| kit::text_w(&p, &l.1, theme::label())).fold(0.0, f32::max);
			let vw = lines.iter().map(|l| kit::text_w(&p, &l.2, theme::label())).fold(0.0, f32::max);
			let size = Vec2::new(14.0 + kw + 16.0 + vw + 20.0, lines.len() as f32 * 18.0 + 12.0);
			let mut bx = Rect::from_min_size(Pos2::new(x + 12.0, plot.min.y + 4.0), size);
			if bx.max.x > plot.max.x {
				bx = bx.translate(Vec2::new(-(bx.width() + 24.0), 0.0));
			}
			p.rect_filled(bx, 6.0, t.float);
			p.rect_stroke(bx, 6.0, Stroke::new(1.0, t.card_edge), egui::StrokeKind::Inside);
			for (k, (c, name, v)) in lines.iter().enumerate() {
				let ly = bx.min.y + 15.0 + k as f32 * 18.0;
				if let Some(c) = c {
					p.line_segment(
						[Pos2::new(bx.min.x + 9.0, ly), Pos2::new(bx.min.x + 19.0, ly)],
						Stroke::new(2.0, *c),
					);
				}
				p.text(Pos2::new(bx.min.x + 24.0, ly), Align2::LEFT_CENTER, name, theme::label(), t.muted);
				p.text(Pos2::new(bx.max.x - 10.0, ly), Align2::RIGHT_CENTER, v, theme::label(), t.text);
			}
		}
		hover_t
	}

	/// A chart's series over the range, a row each: its colour's mark and name, then now, min, max, mean and peak to
	/// peak
	fn chart_table(&self, p: &egui::Painter, r: Rect, ch: &Chart, samples: &[Point]) {
		let t = theme::tk();
		let cols = ["now", "min", "max", "mean", "p-p"];
		let name_w = 62.0;
		// the columns that fit, from the left: the peak to peak, then the mean, go first
		let n = (((r.width() - name_w) / 50.0).floor() as usize).clamp(2, cols.len());
		let shown: Vec<usize> = match n {
			5 => vec![0, 1, 2, 3, 4],
			4 => vec![0, 1, 2, 4],
			3 => vec![0, 1, 2],
			_ => vec![0, 4],
		};
		let cw = (r.width() - name_w) / shown.len() as f32;
		let col_x = |k: usize| r.min.x + name_w + (k as f32 + 1.0) * cw;
		for (k, c) in shown.iter().enumerate() {
			kit::text(p, Pos2::new(col_x(k), r.min.y + 8.0), Align2::RIGHT_CENTER, cols[*c], caption(), t.faint);
		}
		let mut y = r.min.y + 20.0;
		for se in &ch.series {
			// now, min, max and the sum in one pass over the values (a list of them a series was 13 a frame); the
			// sum from -0.0, as Iterator::sum's
			let (mut now, mut mn, mut mx, mut sum, mut n) = (None, f64::INFINITY, f64::NEG_INFINITY, -0.0, 0usize);
			for v in samples.iter().filter_map(|s| ch.value(se, s)) {
				now = Some(v);
				mn = mn.min(v);
				mx = mx.max(v);
				sum += v;
				n += 1;
			}
			let cy = y + ROW * 0.5;
			p.line_segment([Pos2::new(r.min.x, y), Pos2::new(r.max.x, y)], Stroke::new(1.0, fade(t.line, 0.7)));
			p.line_segment([Pos2::new(r.min.x, cy), Pos2::new(r.min.x + 10.0, cy)], Stroke::new(2.0, (se.color)(&t)));
			kit::text(p, Pos2::new(r.min.x + 15.0, cy), Align2::LEFT_CENTER, se.name, theme::label(), t.soft);
			if let Some(now) = now {
				let mean = sum / n as f64;
				let d = ch.decimals;
				let all = [now, mn, mx, mean, mx - mn];
				for (k, c) in shown.iter().enumerate() {
					let s = format!("{:.d$}", all[*c]);
					kit::text(
						p,
						Pos2::new(col_x(k), cy),
						Align2::RIGHT_CENTER,
						&s,
						theme::label(),
						if *c == 0 { t.text } else { t.soft },
					);
				}
			}
			y += ROW;
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn ticks_are_round_and_in_range() {
		assert_eq!(ticks(0.0, 10.0), vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
		let t = ticks(-1.07, -0.54);
		assert!(t.len() >= 3 && t.len() <= 6, "{t:?}");
		assert!(t.iter().all(|v| (-1.07..=-0.54).contains(v)));
		assert_eq!(tick_label(0.25, 0.05), "0.25");
		assert_eq!(tick_label(200.0, 50.0), "200");
	}
}
