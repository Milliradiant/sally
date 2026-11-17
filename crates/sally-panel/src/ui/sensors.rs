//! The device page's Sensors tab: the device alone and large, its sensors numbered with their normals, coloured by what
//! the runtime's last periods say of them (`crate::sensor_view`: lit, lit from behind, dark where a station that lights
//! the device should light it, dark), and a table of each sensor against each station: its hits in the last period and
//! the angle off its normal to the station. Drag turns it, the wheel zooms, a double click starts over; "turned as in
//! the room" shows it as it is turned now (the stations' directions with it), else in its own frame.

use std::sync::Arc;

use egui::{Align2, Color32, Id, PointerButton, Pos2, Rect, Sense, Stroke, Vec2};
use sally_math::math3d::{dot, len, norm, qconj, qmul, qrot, Pose, Q4, V3};

use super::room::{model_name_for, Cam3, Camera};
use super::theme;
use super::theme::*;
use super::Panel;
use crate::models::Model;
use crate::runtime::Device;
use crate::sensor_view::{judge, stations_of, Bvh, SensorState, Verdict};

/// The tab's own state: its camera, whether the device is shown turned as in the room, the sensor under the pointer
pub struct SensorsState {
	pub yaw: f64,
	pub pitch: f64,
	/// the camera's distance in the device's size (the farthest sensor from their middle): the wheel's
	pub zoom: f64,
	pub as_in_room: bool,
	/// the sensor under the pointer (in the view or the table), in each view
	pub hover: [Option<usize>; 2],
	pub bodies: crate::sensor_view::Bodies,
}

impl Default for SensorsState {
	fn default() -> SensorsState {
		SensorsState {
			yaw: -2.2,
			pitch: 0.45,
			zoom: 1.0,
			as_in_room: true,
			hover: [None, None],
			bodies: Default::default(),
		}
	}
}

/// A sensor's colour by its state
pub fn state_color(s: SensorState) -> Color32 {
	match s {
		SensorState::Lit => theme::tk().good,
		SensorState::Behind => theme::tk().violet,
		SensorState::Missing => theme::tk().bad,
		SensorState::Dark => theme::tk().faint,
	}
}

/// How long a sensor's normal is drawn (m)
const NORMAL_M: f64 = 0.012;

impl Panel {
	/// The device's render model, its tree for rays, and its head frame into the model's (as the room view turns it)
	fn device_body(&mut self, d: &Device) -> Option<(Arc<Model>, Arc<Bvh>, Q4)> {
		let dirs = self.config_dirs();
		self.models.set_steamvr(&self.st.steamvr);
		let mut name = self.models.name_by_serial(&d.serial, &dirs);
		if name.is_empty() {
			name = model_name_for(&d.subtype).to_string();
		}
		let m = self.models.get(&name)?;
		let off = self
			.vc
			.devs
			.get(&d.serial)
			.filter(|v| v.rot_offset_n >= 60)
			.map(|v| v.rot_offset)
			.unwrap_or_else(|| crate::models::head_turn(&name));
		let bvh = self.page_state.sensors.bodies.get(&name, &m);
		Some((m, bvh, off))
	}

	/// The Sensors tab under the device box: the view on the left, the table on the right
	pub(super) fn page_sensors(&mut self, ui: &mut egui::Ui, r: Rect, d: &Device) {
		let model = self.device_body(d);
		let stations = stations_of(&self.rt);
		let v = judge(d, &stations, model.as_ref().map(|(_, b, off)| (b.as_ref(), *off)));
		let m = 10.0;
		let n = v.states.len();
		let table_w = (210.0 + 118.0 * v.stations.len() as f32).clamp(440.0, r.width() * 0.6);
		let view_r = Rect::from_min_max(r.min, Pos2::new(r.max.x - table_w - m, r.max.y));
		let table_r = Rect::from_min_max(Pos2::new(r.max.x - table_w, r.min.y), r.max);

		let missing = v.count(SensorState::Missing);
		let (summary, col) = if n == 0 {
			("No sensor layout yet".to_string(), None)
		} else if missing > 0 {
			(format!("{missing} of {n} dark that should be lit"), Some(theme::tk().bad))
		} else {
			(format!("{} of {n} lit", v.count(SensorState::Lit) + v.count(SensorState::Behind)), None)
		};
		let inner = self.boxed(ui, view_r, "Sensors", &summary, col);
		self.sensors_view(ui, inner, d, &v, model.as_ref());
		let right = if v.period > 0.0 { format!("the last {:.1} s", v.period) } else { String::new() };
		let tinner = self.boxed(ui, table_r, "By station", &right, None);
		self.sensors_table(ui, tinner, &v);
	}

	/// The device alone: its model, its sensors and their normals, the stations' directions
	fn sensors_view(
		&mut self, ui: &mut egui::Ui, r: Rect, d: &Device, v: &Verdict, model: Option<&(Arc<Model>, Arc<Bvh>, Q4)>,
	) {
		let vi = self.view.index();
		// the toggle over the view's top left, the legend along its bottom
		let (tr, flip) = super::kit::chip(
			ui,
			Id::unique("sensors-as-room"),
			r.min,
			28.0,
			"Turned as in the room",
			self.page_state.sensors.as_in_room,
			None,
			22.0,
		);
		if flip {
			self.page_state.sensors.as_in_room = !self.page_state.sensors.as_in_room;
		}
		let canvas = Rect::from_min_max(Pos2::new(r.min.x, tr.max.y + 6.0), Pos2::new(r.max.x, r.max.y - 22.0));
		let resp = ui.interact(canvas, Id::unique("sensors-view"), Sense::click_and_drag());
		{
			let sc = &mut self.page_state.sensors;
			if resp.dragged_by(PointerButton::Primary) {
				let dl = resp.drag_delta();
				sc.yaw -= dl.x as f64 * 0.008;
				sc.pitch = (sc.pitch + dl.y as f64 * 0.008).clamp(-1.5, 1.5);
			}
			if resp.hovered() {
				let wheel = ui.input(|i| i.smooth_scroll_delta.y) as f64 / 40.0;
				if wheel != 0.0 {
					sc.zoom = (sc.zoom * 0.88f64.powf(wheel)).clamp(0.3, 6.0);
				}
			}
			if resp.double_clicked() {
				let keep = sc.as_in_room;
				*sc = SensorsState {
					as_in_room: keep,
					bodies: std::mem::take(&mut sc.bodies),
					..SensorsState::default()
				};
			}
		}
		let p = ui.painter_at(canvas);
		self.surface(&p, canvas, super::kit::Surface::Well);

		// the sensors in the head frame, and how they are shown: turned as the device is now, or in its own frame
		let sensors: Vec<(V3, V3)> = (0..d.sensor_pos.len()).filter_map(|i| d.sensor_in_head(i)).collect();
		if sensors.is_empty() {
			self.text_at(&p, canvas.center(), Align2::CENTER_CENTER, body(), theme::tk().muted, "No sensor layout yet");
			return;
		}
		let turn = if self.page_state.sensors.as_in_room && d.have_pose { d.rot } else { Q4::default() };
		let shown = |h: V3| qrot(turn, h);
		let centre = sensors.iter().fold(V3::default(), |a, s| a + s.0) * (1.0 / sensors.len() as f64);
		let size = sensors.iter().map(|s| len(s.0 - centre)).fold(0.03, f64::max);
		let sc = &self.page_state.sensors;
		let mut cam = Camera::default();
		cam.target = shown(centre);
		cam.yaw = sc.yaw;
		cam.pitch = sc.pitch;
		// the device about two thirds of the view's height
		cam.dist = size * 3.4 * sc.zoom;
		let c3 = Cam3::new(&cam, canvas);
		let eye = c3.eye();

		// the model, under everything else
		if let Some((m, _, off)) = model {
			let tint = Color32::from_rgb(150, 153, 162);
			c3.model(&p, m, &Pose { p: V3::default(), q: qmul(turn, *off) }, tint, 255);
		}

		// the stations' directions from the device, out to a little past it: brighter for the ones lighting it
		let reach = size * 1.9;
		for (j, idx) in v.stations.iter().enumerate() {
			let Some(st) = self.rt.stations.get(idx) else { continue };
			if !d.have_pose {
				break;
			}
			let dir_h = qrot(qconj(d.rot), norm(st.pose.p - d.pos));
			let a = shown(centre);
			let b = shown(centre + dir_h * reach);
			let c = if v.lights[j] { alpha(theme::tk().text, 170) } else { alpha(theme::tk().faint, 200) };
			c3.line(&p, a, b, c, 1.2);
			if let Some(q) = c3.pt(b) {
				self.text(&p, q + Vec2::new(4.0, -8.0), theme::label(), c, &st.channel_label());
			}
		}

		// the sensors: the ones the model hides from the eye drawn small and faint, then the others over them
		let hover_now = resp.hover_pos();
		let mut drawn: Vec<(usize, Pos2, bool)> = Vec::new();
		let into_model = model.map(|(_, _, off)| qconj(qmul(turn, *off)));
		for (i, &(hp, hn)) in sensors.iter().enumerate() {
			let (ps, ns) = (shown(hp), shown(hn));
			let Some((q, _)) = c3.proj(ps) else { continue };
			let to_eye = eye - ps;
			let mut seen = dot(ns, to_eye) > 0.0;
			if let (Some((_, bvh, _)), Some(im)) = (model, into_model) {
				let o = bvh.out_of(qrot(im, ps), qrot(im, norm(ns)));
				seen = !bvh.hits(o, qrot(im, norm(to_eye)), len(to_eye));
			}
			drawn.push((i, q, seen));
		}
		drawn.sort_by_key(|x| x.2);
		let hovered = self.page_state.sensors.hover[vi];
		let mut near: Option<(usize, f32)> = None;
		for &(i, q, seen) in &drawn {
			let st = v.states.get(i).copied().unwrap_or(SensorState::Dark);
			let c = state_color(st);
			let (hp, hn) = sensors[i];
			let tip = c3.pt(shown(hp + hn * NORMAL_M));
			let a = if seen { 255 } else { 90 };
			if let Some(t) = tip {
				p.line_segment([q, t], Stroke::new(if seen { 1.6 } else { 1.0 }, alpha(c, a)));
			}
			let rad = if seen { 5.5 } else { 3.5 };
			p.circle_filled(q, rad, alpha(if st == SensorState::Dark { theme::tk().faint } else { c }, a));
			if st == SensorState::Missing || st == SensorState::Behind {
				p.circle_stroke(q, rad + 2.5, Stroke::new(1.5, alpha(c, a)));
			}
			if hovered == Some(i) {
				p.circle_stroke(q, rad + 5.0, Stroke::new(1.8, theme::tk().text));
			}
			// its number, away from the normal's tip
			let away = tip.map(|t| (q - t).normalized()).filter(|x| x.is_finite()).unwrap_or(Vec2::new(1.0, -1.0));
			let lp = q + away * 13.0;
			self.text_at(
				&p,
				lp,
				Align2::CENTER_CENTER,
				theme::label(),
				if seen { theme::tk().text } else { alpha(theme::tk().muted, 150) },
				&i.to_string(),
			);
			if let Some(hp) = hover_now {
				let dd = hp.distance(q);
				if dd < 14.0 && near.is_none_or(|n| dd < n.1 || (seen && dd < n.1 + 4.0)) {
					near = Some((i, dd));
				}
			}
		}
		if resp.hovered() {
			self.page_state.sensors.hover[vi] = near.map(|n| n.0);
		}

		// the legend, and why nothing is judged
		let mut x = r.min.x;
		let y = r.max.y - 16.0;
		for (st, label) in [
			(SensorState::Lit, "lit"),
			(SensorState::Behind, "lit from behind"),
			(SensorState::Missing, "dark, should be lit"),
			(SensorState::Dark, "dark"),
		] {
			let c = if st == SensorState::Dark { theme::tk().faint } else { state_color(st) };
			ui.painter().circle_filled(Pos2::new(x + 5.0, y + 7.0), 4.5, c);
			let nr = self.text(
				ui.painter(),
				Pos2::new(x + 14.0, y),
				theme::label(),
				theme::tk().text,
				&v.count(st).to_string(),
			);
			let tr = self.text(ui.painter(), Pos2::new(nr.max.x + 5.0, y), theme::label(), theme::tk().muted, label);
			x = tr.max.x + 18.0;
		}
		let note = if !v.judged {
			"not tracking: which sensors should be lit is not known"
		} else if model.is_none() {
			"no model: the device's own body is not taken into account"
		} else {
			""
		};
		if !note.is_empty() {
			self.text_at(
				ui.painter(),
				Pos2::new(r.max.x, y),
				Align2::RIGHT_TOP,
				theme::label(),
				theme::tk().warn,
				note,
			);
		}
	}

	/// Each sensor against each station: its state, then per station its hits in the last period and the angle off
	/// its normal to it (a ring where the station should light it; "body" where the device is in the way, "out"
	/// outside the station's view)
	fn sensors_table(&mut self, ui: &mut egui::Ui, r: Rect, v: &Verdict) {
		let vi = self.view.index();
		let p = ui.painter().clone();
		let n = v.states.len();
		if n == 0 {
			self.text_at(&p, r.center(), Align2::CENTER_CENTER, body(), theme::tk().muted, "No sensor layout yet");
			return;
		}
		let x_state = r.min.x + 30.0;
		let x0 = r.min.x + 196.0;
		let cw = ((r.max.x - x0) / v.stations.len().max(1) as f32).min(124.0);
		let head_h = 20.0;
		let legend_h = 40.0;
		let row = ((r.height() - head_h - legend_h) / n as f32).clamp(13.0, 22.0);
		self.text(&p, Pos2::new(r.min.x, r.min.y), theme::label(), theme::tk().faint, "#");
		self.text(&p, Pos2::new(x_state, r.min.y), theme::label(), theme::tk().faint, "state");
		for (j, idx) in v.stations.iter().enumerate() {
			let label = self.rt.stations.get(idx).map(|s| s.channel_label()).unwrap_or_else(|| idx.to_string());
			let c = if v.lights[j] { theme::tk().soft } else { theme::tk().faint };
			self.text(&p, Pos2::new(x0 + j as f32 * cw, r.min.y), theme::label(), c, &label);
		}
		let mut y = r.min.y + head_h;
		let hovered = self.page_state.sensors.hover[vi];
		let mut row_hover = None;
		let pointer = ui.input(|i| i.pointer.hover_pos());
		for i in 0..n {
			let rr = Rect::from_min_max(Pos2::new(r.min.x - 4.0, y - 2.0), Pos2::new(r.max.x + 4.0, y + row - 2.0));
			if pointer.is_some_and(|q| rr.contains(q)) {
				row_hover = Some(i);
			}
			if hovered == Some(i) {
				p.rect_filled(rr, 3.0, white(16));
			}
			self.row_rule(&p, Pos2::new(r.min.x, y - 2.0), Pos2::new(r.max.x, y - 2.0));
			let st = v.states[i];
			let sc = if st == SensorState::Dark { theme::tk().muted } else { state_color(st) };
			self.text(&p, Pos2::new(r.min.x, y), theme::label(), theme::tk().soft, &i.to_string());
			self.text(&p, Pos2::new(x_state, y), theme::label(), sc, st.label());
			for (j, c) in v.cells[i].iter().enumerate() {
				let x = x0 + j as f32 * cw;
				let (txt, col) = if c.hits > 0 {
					(c.hits.to_string(), if c.behind { theme::tk().violet } else { theme::tk().good })
				} else {
					("-".to_string(), if c.expected { theme::tk().bad } else { theme::tk().faint })
				};
				if c.expected {
					p.circle_stroke(Pos2::new(x + 4.0, y + 7.0), 4.0, Stroke::new(1.3, col));
				}
				self.text(&p, Pos2::new(x + 12.0, y), theme::label(), col, &txt);
				let ang = if c.angle.is_nan() {
					String::new()
				} else if c.hidden {
					format!("{:.0}° body", c.angle)
				} else if !c.in_view {
					format!("{:.0}° out", c.angle)
				} else {
					format!("{:.0}°", c.angle)
				};
				self.text(&p, Pos2::new(x + 40.0, y), theme::label(), theme::tk().faint, &ang);
			}
			y += row;
		}
		if row_hover.is_some() || pointer.is_some_and(|q| r.contains(q)) {
			self.page_state.sensors.hover[vi] = row_hover;
		}
		// what the columns say
		let ly = r.max.y - legend_h + 8.0;
		p.circle_stroke(Pos2::new(r.min.x + 4.0, ly + 7.0), 4.0, Stroke::new(1.3, theme::tk().soft));
		self.text(
			&p,
			Pos2::new(r.min.x + 12.0, ly),
			theme::label(),
			theme::tk().muted,
			"it should light the sensor   hits in the last period, angle off the sensor's normal",
		);
		self.text(
			&p,
			Pos2::new(r.min.x + 12.0, ly + 17.0),
			theme::label(),
			theme::tk().muted,
			"body: the device itself in the way   out: outside the station's view",
		);
	}
}
