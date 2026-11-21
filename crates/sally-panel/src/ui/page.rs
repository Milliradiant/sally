//! The device page: one device over its last seconds (10 to 120, 30 by default), under a row of the devices (a
//! button each, which picks the page's device): which stations' light it got and its state, its position,
//! orientation, speed, pose rate and position sd as charts with one time cursor across them, and under each chart
//! its series over the range (now, min, max, mean, peak to peak). Pause holds the range still. The table on the
//! Tracking tab is the device now; this is how it got there. The range is on the runtime's clock, as the history
//! is. Its Sensors view is `sensors.rs`'s.

use egui::{Align2, Color32, Id, Pos2, Rect, Sense, Shape, Stroke, Vec2};

use super::kit::{self, Look, Surface, CTL_H, GAP, HEADER_H, PAD};
use super::theme;
use super::theme::*;
use super::Panel;
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
const ROW: f32 = 20.0;
/// A chart's table's header row
const TABLE_HEAD: f32 = 22.0;

fn charts() -> Vec<Chart> {
	// one series: the charts' own hue, which is no axis's and no status's
	let ink: fn(&Tk) -> Color32 = |t| t.series;
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
			title: "Position ±",
			unit: "mm",
			series: vec![Series { name: "± pos", color: ink, value: |p| p.s.sd_mm }],
			decimals: 1,
			steps: false,
			from_zero: true,
			wraps: false,
		},
		Chart {
			title: "Residual",
			unit: "mrad",
			series: vec![Series { name: "residual", color: ink, value: |p| p.s.res_mrad }],
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
	/// The devices as a row of buttons over the page, each its symbol and name, the page's own lit; before them the
	/// way back to the room. False when the page was left
	fn page_devices(&mut self, ui: &mut egui::Ui, r: Rect) -> bool {
		let t = theme::tk();
		let p = ui.painter().clone();
		let bw = kit::button_w(&p, "Back to the room", false);
		let back = Rect::from_min_size(r.min, Vec2::new(bw, r.height()));
		if kit::button(ui, Id::unique("page-back"), back, "Back to the room", None, Look::Plain, true).clicked() {
			self.page = None;
			return false;
		}
		let codes: Vec<String> = self.active_devices().iter().map(|d| d.codename.clone()).collect();
		let n = codes.len().max(1) as f32;
		let left = back.max.x + 14.0;
		let each = ((r.max.x - left - 6.0 * (n - 1.0)) / n).clamp(64.0, 190.0);
		let mut x = left;
		let mut open = None;
		for code in &codes {
			let Some(d) = self.rt.devices.get(code) else { continue };
			let br = Rect::from_min_size(Pos2::new(x, r.min.y), Vec2::new(each, r.height()));
			let on = self.page.as_deref() == Some(code.as_str());
			let look = if on { Look::Primary } else { Look::Plain };
			if kit::button(ui, Id::unique(("page-dev", code)), br, "", None, look, true).clicked() && !on {
				open = Some(code.clone());
			}
			let fg = if on { t.on_ink } else { t.text };
			let sr = Rect::from_center_size(Pos2::new(br.min.x + 20.0, br.center().y), Vec2::splat(20.0));
			self.draw_symbol(&p, sr, d, if on { fg } else { t.kind(&d.subtype) });
			let clip = Rect::from_min_max(Pos2::new(br.min.x + 36.0, br.min.y), Pos2::new(br.max.x - 8.0, br.max.y));
			kit::text_in(&p, clip, &self.short_name(d), theme::label(), fg);
			x = br.max.x + 6.0;
		}
		if let Some(c) = open {
			self.open_device_page(&c);
		}
		true
	}

	pub(super) fn device_page(&mut self, ui: &mut egui::Ui, r: Rect) {
		if !self.page_devices(ui, Rect::from_min_size(r.min, Vec2::new(r.width(), CTL_H))) {
			return;
		}
		let Some(code) = self.page.clone() else { return };
		let r = Rect::from_min_max(Pos2::new(r.min.x, r.min.y + CTL_H + GAP), r.max);
		// a copy for the boxes (which change the panel's state as they draw) without the history, whose range is taken
		// from the device itself below: a whole copy a frame was its 120 s of samples
		let Some(d) = self.rt.devices.get_mut(&code).map(Device::copy_light) else {
			self.page = None;
			return;
		};
		let range = if RANGES.contains(&self.st.page_range) { self.st.page_range } else { 30.0 };
		let t1 = self.page_state.paused_at.unwrap_or(self.rt.run_time);
		let span = (t1 - range, t1);

		let head_h = 104.0;
		self.page_head(ui, Rect::from_min_size(r.min, Vec2::new(r.width(), head_h)), &d);
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
		let light_h = HEADER_H + 8.0 + LANE * (stations.len() as f32 + 1.0) + 22.0 + PAD - 4.0;
		let light_r = Rect::from_min_size(Pos2::new(r.min.x, y), Vec2::new(r.width(), light_h));
		let mut hover = self.page_light(ui, light_r, &stations, &samples, span);
		y += light_h + GAP;

		// the charts in two rows of three, each with its numbers under it; the residual's chart only where the runtime
		// sends it (S lines' eleventh field), the position sd's chart over its slot too otherwise
		let mut cs = charts();
		let has_res = samples.iter().any(|p| p.s.res_mrad >= 0.0);
		if !has_res {
			cs.retain(|c| c.title != "Residual");
		}
		let row_h = ((r.max.y - y - GAP) / 2.0).max(150.0);
		let w3 = (r.width() - 2.0 * GAP) / 3.0;
		for (i, ch) in cs.iter().enumerate() {
			let (col, row) = ((i % 3) as f32, (i / 3) as f32);
			let w = if !has_res && ch.title == "Position ±" { 2.0 * w3 + GAP } else { w3 };
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

	/// The page's head: the device (its symbol, name, what it is), the view (curves or sensors), the curves' range and
	/// Pause; under them its numbers now, a cell each
	fn page_head(&mut self, ui: &mut egui::Ui, r: Rect, d: &Device) {
		let t = theme::tk();
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Card);
		let h = self.health_of(d);
		let x0 = r.min.x + PAD;
		let xr = r.max.x - PAD;
		let cy = r.min.y + 24.0;
		self.draw_symbol(
			&p,
			Rect::from_min_size(Pos2::new(x0, cy - 15.0), Vec2::new(30.0, 30.0)),
			d,
			t.kind(&d.subtype),
		);
		let x = x0 + 40.0;

		// from the right: the view, Pause, the range
		let bh = 28.0;
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
			let (says, look, tip) = if paused {
				("Go on", Look::Primary, "The range follows the device again")
			} else {
				("Pause", Look::Plain, "Holds the range still")
			};
			let (clicked, left) =
				kit::buttons_right(ui, Id::unique("page-pause"), right, cy, bh, &[(says, look, true, tip)]);
			if clicked.is_some() {
				self.page_state.paused_at = if paused { None } else { Some(self.rt.run_time) };
			}
			right = left - 8.0;
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
		let nr = kit::text_in(
			&p,
			Rect::from_min_max(Pos2::new(x, cy - 14.0), Pos2::new(right, cy + 14.0)),
			&self.short_name(d),
			semibold(17.0),
			t.text,
		);
		let sub = self.type_of(d);
		let kind = t.kind(&d.subtype);
		let sr = kit::text(&p, Pos2::new(nr.max.x + 14.0, cy + 1.0), Align2::LEFT_CENTER, &sub, theme::label(), kind);
		let mut id = format!("{}  {}", d.codename, d.serial);
		if !d.dongle.is_empty() && d.dongle != "-" {
			id += &format!("  dongle {}", d.dongle);
		}
		let clip = Rect::from_min_max(Pos2::new(sr.max.x + 12.0, cy - 10.0), Pos2::new(right, cy + 12.0));
		kit::text_in(&p, clip, &id, code(), t.muted);

		// its numbers now: a cell each, ruled apart
		let sdmm = if d.pos_sd >= 0.0 { d.pos_sd * 1000.0 } else { -1.0 };
		let charge = d.charge();
		let dash = || "–".to_string();
		let fields: Vec<(&str, String, &str)> = vec![
			("State", h.state.clone(), ""),
			("Pose rate", format!("{:.0}", d.rate_hz), "/s"),
			("Position", if sdmm < 0.0 { dash() } else { format!("±{sdmm:.1}") }, "mm"),
			("Light's age", if d.light_age < 0.0 { dash() } else { format!("{:.3}", d.light_age) }, "s"),
			("Speed", format!("{:.2}", sally_math::math3d::len(d.vel)), "m/s"),
			("Height", format!("{:.2}", d.pos.z - self.floor_z()), "m"),
			("Battery", if charge < 0 { dash() } else { charge.to_string() }, if charge < 0 { "" } else { "%" }),
		];
		let fy = r.min.y + 48.0;
		p.line_segment([Pos2::new(r.min.x + 1.0, fy), Pos2::new(r.max.x - 1.0, fy)], Stroke::new(1.0, t.card_edge));
		let cw = (r.width() - 2.0) / fields.len() as f32;
		for (i, (k, v, unit)) in fields.iter().enumerate() {
			let cell = Rect::from_min_max(
				Pos2::new(r.min.x + 1.0 + i as f32 * cw, fy + 1.0),
				Pos2::new(r.min.x + 1.0 + (i as f32 + 1.0) * cw, r.max.y - 1.0),
			);
			if i > 0 {
				p.line_segment([cell.left_top(), cell.left_bottom()], Stroke::new(1.0, t.line));
			}
			let inner = Rect::from_min_max(cell.min + Vec2::new(PAD, 5.0), cell.max - Vec2::new(8.0, 5.0));
			if i > 0 {
				kit::reading(&p, inner, k, v, unit, false);
				continue;
			}
			// the state: a word, in its status' colour
			kit::text(&p, Pos2::new(inner.min.x, inner.min.y + 9.0), Align2::LEFT_CENTER, k, caption(), t.muted);
			let at = Pos2::new(inner.min.x, inner.max.y - 3.0);
			kit::text(&p, at, Align2::LEFT_BOTTOM, v, name(), self.state_ink(d));
		}
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
			p.rect_filled(lane, 0.0, t.ctl);
			// runs of samples with the bit set (or in one state), split where samples are over 0.25 s apart
			let mut run: Option<(f64, f64, Color32)> = None;
			let flush = |run: &mut Option<(f64, f64, Color32)>| {
				if let Some((a, b, c)) = run.take() {
					let rr =
						Rect::from_min_max(Pos2::new(tx(a), lane.min.y), Pos2::new(tx(b).max(tx(a) + 1.0), lane.max.y));
					p.rect_filled(rr, 0.0, c);
				}
			};
			let mut last_t = f64::NEG_INFINITY;
			for s in samples.iter().map(|pt| &pt.s) {
				let c = if i < stations.len() {
					(s.lh_mask & (1u32 << stations[i].0) != 0).then_some(t.good)
				} else {
					Some(match s.light {
						Light::Tracking => t.good,
						Light::Dark => mix(t.ctl, t.warn, 0.55),
						Light::Coasting => t.warn,
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
			kit::text(p, Pos2::new(x, y), align, &says, mono(11.5), t.muted);
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
		// the table of the series at the box's foot, from edge to edge; the plot over it
		let table_h = TABLE_HEAD + ch.series.len() as f32 * ROW;
		let foot = r.max.y - 1.0;
		let plot = Rect::from_min_max(
			Pos2::new(inner.min.x + 40.0, inner.min.y + 2.0),
			Pos2::new(inner.max.x - 2.0, foot - table_h - 26.0),
		);
		// (each sample's values straight in: a list of them a sample was the frame's most allocations)
		let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
		for s in samples {
			for v in ch.series.iter().filter_map(|se| ch.value(se, s)) {
				lo = lo.min(v);
				hi = hi.max(v);
			}
		}
		let table = Rect::from_min_max(Pos2::new(r.min.x + 1.0, foot - table_h), Pos2::new(r.max.x - 1.0, foot));
		self.chart_table(&p, table, ch, samples);
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
				mono(11.5),
				t.muted,
			);
		}
		self.time_axis(&p, plot.min.x, plot.max.x, plot.max.y + 5.0, span);
		// the lines, thinned to about two samples a point, broken where samples are over 0.25 s apart
		let step = (samples.len() as f32 / (plot.width() * 2.0)).ceil().max(1.0) as usize;
		let pc = p.with_clip_rect(plot.expand(1.0).intersect(p.clip_rect()));
		for se in &ch.series {
			let col = (se.color)(&t);
			let mut line: Vec<Pos2> = Vec::new();
			let mut last_t = f64::NEG_INFINITY;
			let mut last_v = f64::NAN;
			let draw = |line: &mut Vec<Pos2>| {
				if line.len() > 1 {
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
			let vw = lines.iter().map(|l| kit::text_w(&p, &l.2, num())).fold(0.0, f32::max);
			let size = Vec2::new(14.0 + kw + 16.0 + vw + 20.0, lines.len() as f32 * 18.0 + 12.0);
			let mut bx = Rect::from_min_size(Pos2::new(x + 12.0, plot.min.y + 4.0), size);
			if bx.max.x > plot.max.x {
				bx = bx.translate(Vec2::new(-(bx.width() + 24.0), 0.0));
			}
			p.rect_filled(bx, t.ctl_radius, t.float);
			p.rect_stroke(bx, t.ctl_radius, Stroke::new(1.0, t.card_edge), egui::StrokeKind::Inside);
			for (k, (c, name, v)) in lines.iter().enumerate() {
				let ly = bx.min.y + 15.0 + k as f32 * 18.0;
				if let Some(c) = c {
					p.line_segment(
						[Pos2::new(bx.min.x + 9.0, ly), Pos2::new(bx.min.x + 19.0, ly)],
						Stroke::new(2.0, *c),
					);
				}
				p.text(Pos2::new(bx.min.x + 24.0, ly), Align2::LEFT_CENTER, name, theme::label(), t.muted);
				p.text(Pos2::new(bx.max.x - 10.0, ly), Align2::RIGHT_CENTER, v, num(), t.text);
			}
		}
		hover_t
	}

	/// A chart's series over the range, a table as the panel's others (a header row, ruled and striped rows, a line
	/// between the columns), a row each: its colour's mark and name, then now, min, max, mean and peak to peak
	fn chart_table(&self, p: &egui::Painter, r: Rect, ch: &Chart, samples: &[Point]) {
		let t = theme::tk();
		let cols = ["now", "min", "max", "mean", "p-p"];
		let name_w = 76.0 + PAD;
		// the columns that fit, from the left: the peak to peak, then the mean, go first
		let n = (((r.width() - name_w - PAD) / 50.0).floor() as usize).clamp(2, cols.len());
		let shown: Vec<usize> = match n {
			5 => vec![0, 1, 2, 3, 4],
			4 => vec![0, 1, 2, 4],
			3 => vec![0, 1, 2],
			_ => vec![0, 4],
		};
		let cw = (r.width() - name_w) / shown.len() as f32;
		// a column's right edge, and where its figures end
		let edge = |k: usize| r.min.x + name_w + (k as f32 + 1.0) * cw;
		let col_x = |k: usize| edge(k) - 10.0;
		let head = Rect::from_min_max(r.min, Pos2::new(r.max.x, r.min.y + TABLE_HEAD));
		p.rect_filled(head, 0.0, t.head);
		p.line_segment([head.left_top(), head.right_top()], Stroke::new(1.0, t.card_edge));
		p.line_segment([head.left_bottom(), head.right_bottom()], Stroke::new(1.0, t.card_edge));
		for (k, c) in shown.iter().enumerate() {
			kit::text(p, Pos2::new(col_x(k), head.center().y), Align2::RIGHT_CENTER, cols[*c], caption(), t.muted);
		}
		let mut y = head.max.y;
		for (row, se) in ch.series.iter().enumerate() {
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
			let ground = Rect::from_min_max(Pos2::new(r.min.x, y), Pos2::new(r.max.x, (y + ROW).min(r.max.y)));
			if row % 2 == 1 {
				p.rect_filled(ground, 0.0, t.stripe);
			}
			if row > 0 {
				p.line_segment([ground.left_top(), ground.right_top()], Stroke::new(1.0, t.line));
			}
			let x = r.min.x + PAD;
			p.line_segment([Pos2::new(x, cy), Pos2::new(x + 10.0, cy)], Stroke::new(2.0, (se.color)(&t)));
			kit::text(p, Pos2::new(x + 15.0, cy), Align2::LEFT_CENTER, se.name, theme::label(), t.soft);
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
						num(),
						if *c == 0 { t.text } else { t.soft },
					);
				}
			}
			y += ROW;
		}
		// the lines between the columns, through the header and the rows: a strong one after the names, a light one
		// between the figures
		for k in 0..shown.len() {
			let x = edge(k) - cw;
			let col = if k == 0 { t.group } else { t.line };
			p.line_segment([Pos2::new(x, head.min.y), Pos2::new(x, y.min(r.max.y))], Stroke::new(1.0, col));
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
