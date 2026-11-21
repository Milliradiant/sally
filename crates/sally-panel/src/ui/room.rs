//! The room: the 3D view of the stations, their coverage, the devices (their shape, sensors and axes), their trails,
//! Valve's devices aligned into sally's frame, and the coverage heat map on the floor. Drawn with egui's painter after
//! projecting on the CPU, as the C++ panel did; orbit with the left button, pan with the right or middle one, zoom with
//! the wheel, click to select, double-click to follow. Locate turns the view to a device.

use egui::{Align2, Color32, Id, PointerButton, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use sally_math::math3d::{cross, dot, len, norm, qconj, qmul, qrot, xform, Pose, Q4, V3};

use crate::models::{shade_by, Model};

use super::kit::{self, Look, HEADER_H, PAD};
use super::theme::*;
use super::{FloorFrom, Panel, Sel};

/// The orbit camera (around `target`, Z up)
pub struct Camera {
	pub target: V3,
	pub yaw: f64,
	pub pitch: f64,
	pub dist: f64,
	pub framed: bool,
	/// Locate's turn: from where to where, since when
	anim: Option<(V3, f64, V3, f64, f64)>,
}

impl Default for Camera {
	fn default() -> Camera {
		Camera { target: V3::default(), yaw: -2.2, pitch: 0.55, dist: 6.5, framed: false, anim: None }
	}
}

/// A projection: the eye, its right, up and forward axes, the focal length (points) and the centre
pub(super) struct Cam3 {
	eye: V3,
	pub(super) r: V3,
	pub(super) u: V3,
	f: V3,
	pub(super) focal: f64,
	pub(super) c: Pos2,
}

const NEAR: f64 = 0.05;

impl Cam3 {
	pub(super) fn new(cam: &Camera, canvas: Rect) -> Cam3 {
		let dir = V3::new(cam.pitch.cos() * cam.yaw.cos(), cam.pitch.cos() * cam.yaw.sin(), cam.pitch.sin());
		let eye = cam.target + dir * cam.dist;
		let f = norm(cam.target - eye);
		let r = norm(cross(f, V3::new(0.0, 0.0, 1.0)));
		let u = cross(r, f);
		let focal = canvas.height() as f64 * 0.5 / (55.0f64 * std::f64::consts::PI / 360.0).tan();
		Cam3 { eye, r, u, f, focal, c: canvas.center() }
	}

	/// A point on the screen and its depth, None behind the near plane
	pub(super) fn proj(&self, p: V3) -> Option<(Pos2, f64)> {
		let d = p - self.eye;
		let z = dot(d, self.f);
		if z < NEAR {
			return None;
		}
		let x = self.c.x as f64 + dot(d, self.r) / z * self.focal;
		let y = self.c.y as f64 - dot(d, self.u) / z * self.focal;
		Some((Pos2::new(x as f32, y as f32), z))
	}

	pub(super) fn eye(&self) -> V3 {
		self.eye
	}

	pub(super) fn forward(&self) -> V3 {
		self.f
	}

	pub(super) fn pt(&self, p: V3) -> Option<Pos2> {
		self.proj(p).map(|(q, _)| q)
	}

	/// A segment, cut at the near plane
	pub(super) fn line(&self, painter: &egui::Painter, mut a: V3, mut b: V3, col: Color32, w: f32) {
		let (za, zb) = (dot(a - self.eye, self.f), dot(b - self.eye, self.f));
		if za < NEAR && zb < NEAR {
			return;
		}
		if za < NEAR {
			a = a + (b - a) * ((NEAR - za) / (zb - za));
		} else if zb < NEAR {
			b = b + (a - b) * ((NEAR - zb) / (za - zb));
		}
		if let (Some(pa), Some(pb)) = (self.pt(a), self.pt(b)) {
			painter.line_segment([pa, pb], Stroke::new(w, col));
		}
	}

	/// A filled polygon, when every corner is in front
	pub(super) fn poly(&self, painter: &egui::Painter, pts: &[V3], fill: Color32) {
		let q: Option<Vec<Pos2>> = pts.iter().map(|p| self.pt(*p)).collect();
		if let Some(q) = q {
			painter.add(Shape::convex_polygon(q, fill, Stroke::NONE));
		}
	}

	/// A model at `pose` (its head frame in the room), shaded in `color`: its triangles far ones first, one mesh. A
	/// triangle with a corner behind the near plane is left out
	pub(super) fn model(&self, painter: &egui::Painter, m: &Model, pose: &Pose, color: Color32, a: u8) {
		self.model_faces(painter, m, pose, color, a, false);
	}

	/// A model see-through: only the triangles facing the camera, so that its back does not pile up behind its front
	pub(super) fn model_ghost(&self, painter: &egui::Painter, m: &Model, pose: &Pose, color: Color32, a: u8) {
		self.model_faces(painter, m, pose, color, a, true);
	}

	fn model_faces(&self, painter: &egui::Painter, m: &Model, pose: &Pose, color: Color32, a: u8, front: bool) {
		// back faces left out where the normals say which way is out (the file's), or for a see-through one
		let cull = front || m.outward;
		// in the model's frame: the eye (for the back faces) and the light (for the shading), so that nothing is turned
		// a triangle at a time
		let inv = qconj(pose.q);
		let eye = qrot(inv, self.eye - pose.p);
		let lm = qrot(inv, crate::models::light());
		let at: Vec<Option<(Pos2, f64)>> = m.verts.iter().map(|v| self.proj(xform(pose, *v))).collect();
		let col: Vec<Color32> = m
			.norms
			.iter()
			.map(|n| {
				let k = shade_by(*n, lm);
				Color32::from_rgba_unmultiplied(
					(color.r() as f32 * k) as u8,
					(color.g() as f32 * k) as u8,
					(color.b() as f32 * k) as u8,
					a,
				)
			})
			.collect();
		let mut tris: Vec<(f64, [Pos2; 3], [Color32; 3])> = Vec::with_capacity(m.tri_v.len());
		for ((tv, tn), f) in m.tri_v.iter().zip(&m.tri_n).zip(&m.nrm) {
			if cull && dot(*f, eye - m.verts[tv[0] as usize]) <= 0.0 {
				continue;
			}
			let (Some(p0), Some(p1), Some(p2)) = (at[tv[0] as usize], at[tv[1] as usize], at[tv[2] as usize]) else {
				continue;
			};
			tris.push((p0.1 + p1.1 + p2.1, [p0.0, p1.0, p2.0], tn.map(|k| col[k as usize])));
		}
		tris.sort_unstable_by(|x, y| y.0.total_cmp(&x.0));
		let mut mesh = egui::Mesh::default();
		mesh.reserve_triangles(tris.len());
		mesh.reserve_vertices(tris.len() * 3);
		for (_, q, c) in tris {
			let i = mesh.vertices.len() as u32;
			for (v, col) in q.into_iter().zip(c) {
				mesh.colored_vertex(v, col);
			}
			mesh.add_triangle(i, i + 1, i + 2);
		}
		painter.add(Shape::mesh(mesh));
	}

	/// How a point's place on the screen moves as `p` moves along `dir` (points a metre), None behind the near plane
	pub(super) fn proj_d(&self, p: V3, dir: V3) -> Option<Vec2> {
		let d = p - self.eye;
		let z = dot(d, self.f);
		if z < NEAR {
			return None;
		}
		let zd = dot(dir, self.f);
		let x = (dot(dir, self.r) * z - dot(d, self.r) * zd) / (z * z) * self.focal;
		let y = -(dot(dir, self.u) * z - dot(d, self.u) * zd) / (z * z) * self.focal;
		Some(Vec2::new(x as f32, y as f32))
	}

	/// The floor grid between (x0..x1, y0..y1) squares of `step` at height `z`, a brighter line every metre: one mesh,
	/// the squares' lines under the metres'. A line is drawn a pixel and a half wide, flat for its middle half pixel
	/// (GRID_CORE, GRID_EDGE), so that it looks as bright wherever it falls between the pixels: one a pixel wide is a
	/// whole pixel on a pixel's middle and two halves between two, which looks dimmer, so a line came and went as the
	/// view moved and along a slanting line. Where the lines crowd together on the screen (far off, or seen from low
	/// down, as when zoomed in) they fade out rather than shimmer (grid_fade)
	pub(super) fn grid(
		&self, p: &egui::Painter, sq: (i64, i64, i64, i64), step: f64, z: f64, major: Color32, minor: Color32,
	) {
		let (x0, x1, y0, y1) = sq;
		let per_m = ((1.0 / step).round() as i64).max(1);
		let ppp = p.pixels_per_point();
		let mut mesh = egui::Mesh::default();
		for metre in [false, true] {
			let (col, gap) = if metre { (major, step * per_m as f64) } else { (minor, step) };
			for i in (x0..=x1).filter(|i| (i % per_m == 0) == metre) {
				let x = i as f64 * step;
				let (a, b) = (V3::new(x, y0 as f64 * step, z), V3::new(x, y1 as f64 * step, z));
				self.grid_line(&mut mesh, a, b, V3::new(1.0, 0.0, 0.0), gap, col, ppp);
			}
			for j in (y0..=y1).filter(|j| (j % per_m == 0) == metre) {
				let y = j as f64 * step;
				let (a, b) = (V3::new(x0 as f64 * step, y, z), V3::new(x1 as f64 * step, y, z));
				self.grid_line(&mut mesh, a, b, V3::new(0.0, 1.0, 0.0), gap, col, ppp);
			}
		}
		p.add(Shape::mesh(mesh));
	}

	/// One of the grid's lines, from `a` to `b` (the part in front of the near plane), its neighbours `gap` from it
	/// along `across`: a strip of four vertices at every GRID_PIECE, faded by how far from its neighbours it is there
	#[allow(clippy::too_many_arguments)]
	fn grid_line(&self, mesh: &mut egui::Mesh, a: V3, b: V3, across: V3, gap: f64, col: Color32, ppp: f32) {
		let (za, zb) = (dot(a - self.eye, self.f), dot(b - self.eye, self.f));
		// a hair inside the near plane, so that its ends project
		let near = NEAR * 1.001;
		if za < near && zb < near {
			return;
		}
		// (one end behind it and one in front, so za and zb differ)
		let t0 = if za < near { (near - za) / (zb - za) } else { 0.0 };
		let t1 = if zb < near { (near - za) / (zb - za) } else { 1.0 };
		let n = ((len(b - a) * (t1 - t0) / GRID_PIECE).ceil() as usize).max(1);
		let at: Vec<(Pos2, Vec2)> = (0..=n)
			.filter_map(|k| {
				let pt = a + (b - a) * (t0 + (t1 - t0) * k as f64 / n as f64);
				Some((self.pt(pt)?, self.proj_d(pt, across)?))
			})
			.collect();
		let (Some(first), Some(last)) = (at.first(), at.last()) else {
			return;
		};
		let dir = last.0 - first.0;
		if at.len() < 2 || dir.length() < 0.5 {
			return;
		}
		let side = dir.normalized().rot90();
		let (core, edge) = (side * (GRID_CORE / ppp), side * (GRID_EDGE / ppp));
		let base = mesh.vertices.len() as u32;
		for (k, &(q, d)) in at.iter().enumerate() {
			// how far the neighbours are across this line on the screen here, in pixels
			let apart = d.dot(side).abs() * gap as f32 * ppp;
			let c = col.gamma_multiply(GRID_INK * grid_fade(apart));
			mesh.colored_vertex(q + edge, Color32::TRANSPARENT);
			mesh.colored_vertex(q + core, c);
			mesh.colored_vertex(q - core, c);
			mesh.colored_vertex(q - edge, Color32::TRANSPARENT);
			if k > 0 {
				let (i0, i1) = (base + 4 * (k as u32 - 1), base + 4 * k as u32);
				for e in 0..3 {
					mesh.add_triangle(i0 + e, i0 + e + 1, i1 + e);
					mesh.add_triangle(i0 + e + 1, i1 + e, i1 + e + 1);
				}
			}
		}
	}

	/// A base station in `col`: SteamVR's model of one where there is one to draw, else a box (its corners from its
	/// pose). The model goes on the station's pose as it is: a station's frame is the model's, its top along Y and
	/// its front along -Z
	pub(super) fn station(&self, p: &egui::Painter, model: Option<&Model>, pose: &Pose, col: Color32) {
		if let Some(m) = model {
			let [r, g, b, a] = col.to_srgba_unmultiplied();
			self.model(p, m, pose, Color32::from_rgb(r, g, b), a);
			return;
		}
		let h = 0.06;
		let c: Vec<V3> = (0..8)
			.map(|i| {
				xform(
					pose,
					V3::new(
						if i & 1 != 0 { h } else { -h },
						if i & 2 != 0 { h } else { -h },
						if i & 4 != 0 { h * 0.7 } else { -h * 0.7 },
					),
				)
			})
			.collect();
		for (a, b) in EDGES {
			self.line(p, c[a], c[b], col, 1.6);
		}
	}
}

/// The grid's lines: flat out to GRID_CORE pixels from the middle, gone at GRID_EDGE (a pixel and a half wide in all).
/// Sampled at the pixels' middles that is a whole pixel and two quarters on a pixel's middle, two three-quarters
/// between two, alike to a few per cent in light
const GRID_CORE: f32 = 0.25;
const GRID_EDGE: f32 = 1.25;

/// The grid's colours at this much of their strength: so a line a pixel and a half wide is about as bright as the
/// pixel-wide one was
const GRID_INK: f32 = 0.62;

/// How far along a grid line apart its points are (m): where it fades, it fades smoothly
const GRID_PIECE: f64 = 0.5;

/// A grid line's strength by how far its neighbours are from it on the screen (pixels): gone closer than 2.5, whole
/// from 7 (the squares' lines fade out first, then, much farther off, the metres')
fn grid_fade(apart: f32) -> f32 {
	let t = ((apart - 2.5) / (7.0 - 2.5)).clamp(0.0, 1.0);
	t * t * (3.0 - 2.0 * t)
}

/// The orbit camera's input on a canvas: orbit with the left button, pan with the right or middle one, zoom with the
/// wheel (points, so the overlay orbits as the window does); whether it was panned
pub(super) fn camera_input(cam: &mut Camera, ui: &egui::Ui, resp: &egui::Response, canvas: Rect) -> bool {
	let mut panned = false;
	if resp.dragged_by(PointerButton::Primary) {
		let dl = resp.drag_delta();
		cam.yaw -= dl.x as f64 * 0.006;
		cam.pitch = (cam.pitch + dl.y as f64 * 0.006).clamp(-1.45, 1.5);
		cam.anim = None;
	}
	if resp.dragged_by(PointerButton::Secondary) || resp.dragged_by(PointerButton::Middle) {
		let c3 = Cam3::new(cam, canvas);
		let dl = resp.drag_delta();
		let k = cam.dist / c3.focal;
		cam.target = cam.target - c3.r * (dl.x as f64 * k) + c3.u * (dl.y as f64 * k);
		cam.anim = None;
		panned = true;
	}
	if resp.hovered() {
		let wheel = ui.input(|i| i.smooth_scroll_delta.y) as f64 / 40.0;
		if wheel != 0.0 {
			cam.dist = (cam.dist * 0.88f64.powf(wheel)).clamp(0.3, 60.0);
		}
	}
	panned
}

/// A base station's model in the room, SteamVR's own of a base station 2.0 (the only ones sally tracks from): in
/// SteamVR's resources, under no driver
pub(super) const STATION_MODEL: &str = "lh_basestation_valve_gen2";

/// A device's model in the room: its render model by the config of its serial, else by its type
pub(super) fn model_name_for(subtype: &str) -> &'static str {
	match subtype {
		"knuckles_l" => "{indexcontroller}valve_controller_knu_1_0_left",
		"knuckles_r" => "{indexcontroller}valve_controller_knu_1_0_right",
		"tracker" | "tracker_gen2" => "{htc}vr_tracker_vive_3_0",
		_ => "",
	}
}

/// Where a model sits on a device whose pose is the runtime's head: turned by `off` (sally's head frame into
/// Valve's, which the model is in)
pub(super) fn model_pose(p: V3, q: Q4, off: Q4) -> Pose {
	Pose { p, q: qmul(q, off) }
}

/// The box's 12 edges, between the corner indices (bit 0 x, bit 1 y, bit 2 z)
const EDGES: [(usize, usize); 12] =
	[(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)];

/// What a click can pick
enum Pick {
	Device(String),
	Station(i32),
}

/// The convex hull of points on the screen (Andrew's monotone chain)
fn hull(mut pts: Vec<Pos2>) -> Vec<Pos2> {
	pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
	pts.dedup();
	if pts.len() < 3 {
		return pts;
	}
	let crossz = |o: Pos2, a: Pos2, b: Pos2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
	let mut h: Vec<Pos2> = Vec::with_capacity(pts.len() * 2);
	for pass in 0..2 {
		let start = h.len();
		let it: Box<dyn Iterator<Item = &Pos2>> =
			if pass == 0 { Box::new(pts.iter()) } else { Box::new(pts.iter().rev()) };
		for &p in it {
			while h.len() >= start + 2 && crossz(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
				h.pop();
			}
			h.push(p);
		}
		h.pop();
	}
	h
}

/// How far from the origin a station may be and still widen the floor grid (m): the light reaches some 7 m
const GRID_REACH: f64 = 50.0;

/// The floor grid's squares of `step` (x from, x to, y from, y to): the play area's 6 by 6 m, and a metre around every
/// station placed within GRID_REACH. A station's place is the runtime's text: an absurd one (1e300, inf) made a range
/// of lines without end, each a shape, and the panel ran out of memory drawing them
pub(super) fn grid_squares(stations: impl Iterator<Item = V3>, step: f64) -> (i64, i64, i64, i64) {
	let (mut lo, mut hi) = (V3::new(-3.0, -3.0, 0.0), V3::new(3.0, 3.0, 0.0));
	// (false for a NaN too)
	for p in stations.filter(|p| p.x.abs() <= GRID_REACH && p.y.abs() <= GRID_REACH) {
		lo = V3::new(lo.x.min(p.x - 1.0), lo.y.min(p.y - 1.0), 0.0);
		hi = V3::new(hi.x.max(p.x + 1.0), hi.y.max(p.y + 1.0), 0.0);
	}
	(
		(lo.x / step).floor() as i64,
		(hi.x / step).ceil() as i64,
		(lo.y / step).floor() as i64,
		(hi.y / step).ceil() as i64,
	)
}

/// A label in a room (the live one's and the Replay tab's): the mark it is for, how far from the mark it starts, and
/// what it says. `depth`: a device's, how far it is from the camera (0 for the selected one); None for what is not
/// a device's (a station's channel), which is drawn first and as it is
pub(super) struct Tag {
	pub at: Pos2,
	pub gap: f32,
	pub text: String,
	pub font: egui::FontId,
	pub color: Color32,
	pub depth: Option<f64>,
}

/// How strong a device's label is under a nearer one's
const TAG_UNDER: f32 = 0.72;

/// Draws a room's labels, each at its mark's right whatever else is there: a label never moves to make room, so one
/// may lie on another. The devices' are drawn from the farthest to the nearest, the nearest on top and at full
/// strength, the others a little fainter under it (`tag_order`)
pub(super) fn draw_tags(p: &egui::Painter, tags: Vec<Tag>) {
	let depths: Vec<Option<f64>> = tags.iter().map(|g| g.depth).collect();
	for (i, strength) in tag_order(&depths) {
		let g = &tags[i];
		let at = Pos2::new(g.at.x + g.gap, g.at.y);
		kit::text(p, at, Align2::LEFT_CENTER, &g.text, g.font.clone(), fade(g.color, strength));
	}
}

/// The order labels are drawn in, by their depths, and each one's strength: those without a depth first, as they
/// came and at full strength; then the devices' from the farthest to the nearest, the nearest alone at full strength
fn tag_order(depths: &[Option<f64>]) -> Vec<(usize, f32)> {
	let mut out: Vec<(usize, f32)> =
		depths.iter().enumerate().filter(|(_, d)| d.is_none()).map(|(i, _)| (i, 1.0)).collect();
	let mut devices: Vec<(usize, f64)> = depths.iter().enumerate().filter_map(|(i, d)| d.map(|d| (i, d))).collect();
	devices.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| b.0.cmp(&a.0)));
	let nearest = devices.len().saturating_sub(1);
	out.extend(devices.iter().enumerate().map(|(k, (i, _))| (*i, if k == nearest { 1.0 } else { TAG_UNDER })));
	out
}

/// A line of the room's box's tick boxes
const ROOM_BAR_H: f32 = 28.0;

/// What the room can show, each a tick box over it: its setting's key, its label, and what it is in full
const LAYERS: [(&str, &str, &str); 9] = [
	(
		"models",
		"Models",
		"The devices and the base stations as SteamVR's models (else a device's outline, a station's box)",
	),
	("sensors", "Sensors", "Each device's sensors, the lit ones in its kind's colour"),
	("rays", "Rays", "A line from a station to each device it lights"),
	("cones", "Cones", "Each station's field of view, out to the cones' reach"),
	("trails", "Trails", "Where each device has been, in its status' colour: green with light, yellow without"),
	("labels", "Labels", "The devices' names and the stations' channels"),
	("valve", "Valve", "Valve's pose of each device, lined up with sally's frame"),
	("walls", "Walls", "SteamVR's chaperone, where SteamVR has given one"),
	("heat", "Coverage", "The floor squares the devices have been over, by the light they had there"),
];

/// Where a row of things `widths` wide goes between `left` and `right`, `gap` apart: each one's left end and line, a
/// thing that does not fit going on in the next line; and the number of lines
fn flow(widths: &[f32], left: f32, right: f32, gap: f32) -> (Vec<(f32, usize)>, usize) {
	let (mut x, mut line) = (left, 0);
	let mut out = Vec::with_capacity(widths.len());
	for w in widths {
		if x > left && x + w > right {
			x = left;
			line += 1;
		}
		out.push((x, line));
		x += w + gap;
	}
	(out, line + 1)
}

impl Panel {
	/// Frames the stations and the devices
	fn frame_camera(&mut self) {
		let (mut lo, mut hi, mut n) = (V3::new(1e9, 1e9, 1e9), V3::new(-1e9, -1e9, -1e9), 0);
		let mut add = |p: V3| {
			// a place that is not a number frames nothing (an infinite one put the camera nowhere)
			if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
				return;
			}
			lo = V3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
			hi = V3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
			n += 1;
		};
		for s in self.rt.stations.values().filter(|s| s.have_pose) {
			add(s.pose.p);
		}
		for d in self.rt.devices.values().filter(|d| d.have_pose) {
			add(d.pos);
		}
		if n == 0 {
			return;
		}
		let mut t = (lo + hi) * 0.5;
		t.z = (t.z + self.floor_z()) * 0.5;
		self.cam.target = t;
		self.cam.dist = (len(hi - lo) * 1.25).max(3.0);
		self.cam.framed = true;
		self.cam.anim = None;
	}

	/// Turns the room view to a device: its position the centre, 1.6 m away, in 0.45 s; a controller buzzes three
	/// times as well (where SteamVR runs and lists it)
	pub(super) fn locate(&mut self, codename: &str) {
		let Some(d) = self.rt.devices.get(codename).filter(|d| d.have_pose) else { return };
		self.sel = Sel::Device(codename.into());
		let to_dist = self.cam.dist.min(1.6);
		self.cam.anim = Some((self.cam.target, self.cam.dist, d.pos, to_dist, self.now));
		self.cam.framed = true;
		let serial = d.serial.clone();
		self.vc.buzz(&serial, self.now);
	}

	/// A layer's switch in the settings, by its key
	fn layer(&mut self, key: &str) -> &mut bool {
		let st = &mut self.st;
		match key {
			"models" => &mut st.show_models,
			"sensors" => &mut st.show_sensors,
			"rays" => &mut st.show_rays,
			"cones" => &mut st.show_frustums,
			"trails" => &mut st.show_trails,
			"labels" => &mut st.show_labels,
			"valve" => &mut st.show_valve,
			"walls" => &mut st.show_walls,
			_ => &mut st.show_heat,
		}
	}

	/// What the room's box shows of the layers, each one's width in its row: the tick boxes, the two lengths, and
	/// last the coverage's own where it is on. Nothing in the row moves for what comes and goes: the walls' tick box
	/// is there before SteamVR has given any (and cannot be ticked then), and the coverage's own is at the end
	fn room_bar(&self, p: &egui::Painter) -> (Vec<(&'static str, &'static str, &'static str)>, Vec<f32>) {
		let shown: Vec<(&str, &str, &str)> =
			LAYERS.iter().copied().filter(|(key, ..)| *key != "valve" || self.st.compare).collect();
		let mut widths: Vec<f32> = shown.iter().map(|(_, text, _)| kit::tick_label_w(p, text) + 4.0).collect();
		for text in ["Trails", "Cones"] {
			widths.push(kit::stepper_w(p, ROOM_BAR_H, text) + 8.0);
		}
		if self.st.show_heat {
			let squares = format!("{} squares", self.rt.voxels.len());
			widths.push(kit::text_w(p, &squares, label()) + 8.0 + kit::button_w(p, "Clear", false) + 6.0);
		}
		(shown, widths)
	}

	/// How tall the room's box is at this width: its title line and the lines its tick boxes take, with the
	/// coverage's own where it is on (`all`) or without it
	pub(super) fn room_box_height(&self, p: &egui::Painter, width: f32, all: bool) -> f32 {
		let (shown, widths) = self.room_bar(p);
		let n = if all { widths.len() } else { shown.len() + 2 };
		let (_, lines) = flow(&widths[..n], PAD, width - PAD, 10.0);
		HEADER_H + 5.0 + lines as f32 * (ROOM_BAR_H + 2.0) + 5.0
	}

	/// How the room's view is turned (yaw and pitch, rad): what a drag in the room changes
	pub fn room_turn(&self) -> (f64, f64) {
		(self.cam.yaw, self.cam.pitch)
	}

	/// Where on the tab the room was last seen through: its middle is the view's, its height the view's scale
	pub fn room_place(&self) -> Rect {
		self.room_place
	}

	/// The room's own box, over the room's top left: in its title line the floor, Follow and Whole room; under it
	/// what the room shows, a tick box each, and the two lengths
	pub(super) fn room_box(&mut self, ui: &mut egui::Ui, r: Rect) {
		let t = tk();
		let folded = self.dock(ui, r, "Room", kit::DOCK_ROOM);
		let hr = Self::dock_header(r);
		let cy = hr.center().y;
		let bh = 26.0;

		// from the right: the whole room, Follow
		let items = [("Whole room", Look::Plain, true, "The stations and the devices all in view")];
		let (clicked, left) = kit::buttons_right(ui, Id::unique("room-reset"), hr.max.x - 3.0, cy, bh, &items);
		if clicked.is_some() {
			self.frame_camera();
		}
		let fw = kit::tick_label_w(ui.painter(), "Follow");
		let (fr, c) = kit::tick_label(
			ui,
			Id::unique("room-follow"),
			Pos2::new(left - 14.0 - fw, cy - bh * 0.5),
			bh,
			"Follow",
			self.st.follow,
			true,
		);
		ui.interact(fr, Id::unique("room-follow-tip"), Sense::hover())
			.on_hover_text("The view goes with the selected device (a double click on one in the room does the same)");
		if c {
			self.st.follow = !self.st.follow;
			self.st.dirty = true;
		}

		// the floor: the two buttons that set it, then where it is and where that is from (the text that changes last,
		// so that it moves nothing)
		let conn = self.rt.connected(self.now);
		let (fz, from) = self.floor();
		let p = ui.painter().clone();
		let tw = kit::text_w(&p, "Room", title());
		let mut x = hr.min.x + tw + 22.0;
		x = kit::text(&p, Pos2::new(x, cy), Align2::LEFT_CENTER, "Floor", label(), t.muted).max.x + 10.0;
		let floor_items = [
			("Set here", conn, "3 cm under the lowest device that is tracking now: put a tracker on the floor"),
			(
				"Auto",
				from != FloorFrom::SteamVr,
				"SteamVR's floor where its walls are known, else 3 cm under the lowest device from now on",
			),
		];
		for (k, (text, enabled, hint)) in floor_items.iter().enumerate() {
			let w = kit::button_w(&p, text, false);
			let br = Rect::from_min_size(Pos2::new(x, cy - bh * 0.5), Vec2::new(w, bh));
			if br.max.x > fr.min.x - 12.0 {
				break;
			}
			let resp = kit::button(ui, Id::unique(("room-floor", k)), br, text, None, Look::Plain, *enabled);
			if resp.on_hover_text(*hint).clicked() {
				if k == 0 {
					self.set_floor();
				} else {
					self.st.floor_z = f64::NAN;
					self.auto_floor = f64::NAN;
					self.st.dirty = true;
				}
			}
			x = br.max.x + 6.0;
		}
		let says = match from {
			FloorFrom::Set => "set by hand",
			FloorFrom::SteamVr => "SteamVR's",
			FloorFrom::Devices => "lowest device",
		};
		let clip = p.with_clip_rect(Rect::from_min_max(Pos2::new(x, hr.min.y), Pos2::new(fr.min.x - 10.0, hr.max.y)));
		x = kit::text(&clip, Pos2::new(x + 6.0, cy), Align2::LEFT_CENTER, &format!("{fz:+.2} m"), num(), t.text).max.x;
		kit::text(&clip, Pos2::new(x + 8.0, cy), Align2::LEFT_CENTER, says, label(), t.muted);

		if folded {
			return;
		}

		// the layers: a tick box each, then the two lengths and the coverage's own; on as many lines as the box's
		// width takes
		let (shown, widths) = self.room_bar(&p);
		// (the row is laid out for what was on when the frame began: a tick on Coverage in this frame shows the
		// coverage's own from the next, where the row has a place for it)
		let heat = self.st.show_heat;
		let walls = !self.vc.walls.is_empty();
		let th = ROOM_BAR_H;
		let squares = format!("{} squares", self.rt.voxels.len());
		let steppers = [("Trails", 1.0, 30.0, 1.0, 6.0), ("Cones", 1.0, 6.0, 0.5, 3.0)];
		let left = r.min.x + PAD;
		let (places, _) = flow(&widths, left, r.max.x - PAD, 10.0);
		let top = r.min.y + HEADER_H + 5.0;
		let at = |k: usize| Pos2::new(places[k].0, top + places[k].1 as f32 * (th + 2.0));
		for (k, (key, text, hint)) in shown.iter().enumerate() {
			let on = *self.layer(key);
			let enabled = *key != "walls" || walls;
			let (tr, c) = kit::tick_label(ui, Id::unique(("layer", *key)), at(k), th, text, on, enabled);
			ui.interact(tr, Id::unique(("layer-tip", *key)), Sense::hover()).on_hover_text(*hint);
			if c {
				*self.layer(key) = !on;
				self.st.dirty = true;
			}
		}
		for (i, (text, min, max, step, default)) in steppers.into_iter().enumerate() {
			let range = kit::Range { min, max, step, default: Some(default), inherits: None };
			let mut v = if i == 0 { self.st.trail_seconds } else { self.st.frustum_len };
			let says = if i == 0 { format!("{v:.0} s") } else { format!("{v:.1} m") };
			let (sr, moved) =
				kit::stepper(ui, Id::unique(("room-len", i)), at(shown.len() + i), th, text, &says, &mut v, &range);
			let hint = if i == 0 { "How long the trails last" } else { "How far the stations' cones reach" };
			ui.interact(
				Rect::from_min_size(sr.min, Vec2::new(kit::text_w(&p, text, label()), th)),
				Id::unique(("room-len-tip", i)),
				Sense::hover(),
			)
			.on_hover_text(hint);
			if moved {
				if i == 0 {
					self.st.trail_seconds = v;
					self.rt.trail_seconds = v;
					self.vc.trail_seconds = v;
				} else {
					self.st.frustum_len = v;
				}
				self.st.dirty = true;
			}
		}
		if heat {
			// the coverage's own: how much there is of it, and Clear
			let a = at(shown.len() + 2);
			let tr = kit::text(&p, Pos2::new(a.x, a.y + th * 0.5), Align2::LEFT_CENTER, &squares, label(), t.muted);
			let bw = kit::button_w(&p, "Clear", false);
			let br = Rect::from_min_size(Pos2::new(tr.max.x + 8.0, a.y + 1.0), Vec2::new(bw, bh));
			if kit::button(ui, Id::unique("heat-clear"), br, "Clear", None, Look::Plain, true)
				.on_hover_text("Forgets the floor squares covered this session")
				.clicked()
			{
				self.rt.clear_heat();
			}
		}
	}

	/// The room, drawn over all of `canvas` (the tab: the boxes lie over it) and seen through `view`: the camera looks
	/// at its middle, and its height is the view's scale. `free` is what the boxes leave free now: the coverage's
	/// key is written in its corner
	pub(super) fn room_view(&mut self, ui: &mut egui::Ui, canvas: Rect, view: Rect, free: Rect) {
		let t = tk();
		self.room_place = view;
		let resp = ui.interact(canvas, Id::unique("room"), Sense::click_and_drag());
		let now = self.now;
		if !self.cam.framed {
			self.frame_camera();
		}
		if let Some((t0, d0, t1, d1, since)) = self.cam.anim {
			let k = ((now - since) / 0.45).clamp(0.0, 1.0);
			let e = 1.0 - (1.0 - k).powi(3);
			// the device moves meanwhile: aim at where it is now
			let t1 = match &self.sel {
				Sel::Device(c) => self.rt.devices.get(c).map(|d| d.pos).unwrap_or(t1),
				_ => t1,
			};
			self.cam.target = t0 + (t1 - t0) * e;
			self.cam.dist = d0 + (d1 - d0) * e;
			if k >= 1.0 {
				self.cam.anim = None;
			}
			ui.ctx().request_repaint();
		}
		if self.st.follow {
			if let Sel::Device(c) = &self.sel {
				if let Some(d) = self.rt.devices.get(c).filter(|d| d.have_pose) {
					self.cam.target = self.cam.target * 0.85 + d.pos * 0.15;
				}
			}
		}
		// the camera's input: points, so the overlay (1.25 pixels a point) orbits as the window does
		if camera_input(&mut self.cam, ui, &resp, view) && self.st.follow {
			self.st.follow = false;
			self.st.dirty = true;
		}
		let cam = Cam3::new(&self.cam, view);
		let p = ui.painter_at(canvas);
		// (over a picture the room is what is seen through the panes: the picture as it is, darkened)
		if self.wall.shown() {
			self.wall.window(&p, canvas);
		} else {
			p.rect_filled(canvas, 0.0, t.well);
		}
		let fz = self.floor_z();
		let mut picks: Vec<(Pos2, Pick)> = Vec::new();
		let mut tags: Vec<Tag> = Vec::new();

		// the coverage on the floor: a tile per floor square a device has been over this session (all heights)
		if self.st.show_heat {
			let vs = self.rt.voxel_size;
			let g = vs * 0.04;
			for (&(ix, iy), v) in &self.rt.voxels {
				if v.n < 3 {
					continue;
				}
				let (avg, dark) = (v.stations_sum as f64 / v.n as f64, v.dark as f64 / v.n as f64);
				// the light a square had, as a status: red none, yellow one station, green two or more
				let c = if dark > 0.5 || avg < 0.8 {
					alpha(t.bad, 120)
				} else if avg >= 1.6 {
					alpha(t.good, 110)
				} else {
					alpha(t.warn, 90)
				};
				let (x, y) = (ix as f64 * vs, iy as f64 * vs);
				let q = [
					V3::new(x + g, y + g, fz),
					V3::new(x + vs - g, y + g, fz),
					V3::new(x + vs - g, y + vs - g, fz),
					V3::new(x + g, y + vs - g, fz),
				];
				cam.poly(&p, &q, c);
			}
		}

		// the floor grid: squares the size of a coverage cell, a brighter line every metre
		{
			let step = self.rt.voxel_size;
			let stations = self.rt.stations.values().filter(|s| s.have_pose).map(|s| s.pose.p);
			cam.grid(&p, grid_squares(stations, step), step, fz, t.grid, t.grid_minor);
		}

		// SteamVR's chaperone: its walls as their edges, the floor edges (between a wall's two lowest corners) brighter
		if self.st.show_walls {
			for q in &self.vc.walls {
				let on_floor = crate::valve::floor_corners(q);
				for i in 0..4 {
					let floor = on_floor.contains(&i) && on_floor.contains(&((i + 1) % 4));
					cam.line(
						&p,
						q[i],
						q[(i + 1) % 4],
						if floor { t.muted } else { alpha(t.muted, 90) },
						if floor { 1.6 } else { 1.0 },
					);
				}
			}
		}

		// the stations: SteamVR's model of one each (grey: a station has no kind's hue), fainter while none of the
		// devices has its light, or a box in lines; and their coverage out to the cones' length
		let station_model = if self.st.show_models { self.models.get(STATION_MODEL) } else { None };
		for (idx, sn) in &self.rt.stations {
			if !sn.have_pose {
				continue;
			}
			let selected = self.sel == Sel::Station(*idx);
			let pose = &sn.pose;
			let bc = if selected {
				t.accent
			} else if sn.seen {
				t.soft
			} else {
				t.faint
			};
			let body = match &station_model {
				Some(_) if selected => t.on_ink,
				Some(_) => alpha(Color32::from_rgb(208, 211, 220), if sn.seen { 255 } else { 110 }),
				None => bc,
			};
			cam.station(&p, station_model.as_deref(), pose, body);
			if self.st.show_frustums {
				let gen2 = sn.mode >= 0;
				let (h, v) = crate::sensor_view::fov_half_deg(gen2);
				let (th, tv) = (h.to_radians().tan(), v.to_radians().tan());
				let far: Vec<V3> = (0..4)
					.map(|i| {
						let d =
							norm(V3::new(if i == 0 || i == 3 { -th } else { th }, if i < 2 { -tv } else { tv }, -1.0));
						xform(pose, d * self.st.frustum_len)
					})
					.collect();
				// its field of view as its edges alone
				let fc = if selected { alpha(t.accent, 200) } else { alpha(t.soft, if sn.seen { 56 } else { 28 }) };
				for i in 0..4 {
					cam.line(&p, pose.p, far[i], fc, 1.0);
					cam.line(&p, far[i], far[(i + 1) % 4], fc, 1.0);
				}
			}
			if let Some(sp) = cam.pt(pose.p) {
				picks.push((sp, Pick::Station(*idx)));
				if selected {
					p.circle_stroke(sp, 14.0, Stroke::new(1.6, t.accent));
				}
				if self.st.show_labels {
					let c = if sn.seen { t.soft } else { t.faint };
					tags.push(Tag {
						at: sp,
						gap: 12.0,
						text: sn.channel_label(),
						font: label(),
						color: c,
						depth: None,
					});
				}
			}
		}

		// the light: station to device for every pair with light in the last period
		if self.st.show_rays {
			for d in self.rt.devices.values() {
				if !d.active || now - d.last_pose_wall > 1.0 || now - d.hits_wall > 2.5 {
					continue;
				}
				for (lh, l) in &d.light {
					if l.hits <= 0 {
						continue;
					}
					if let Some(s) = self.rt.stations.get(lh).filter(|s| s.have_pose) {
						cam.line(&p, s.pose.p, d.pos, alpha(t.soft, if l.sensors_lit < 3 { 26 } else { 56 }), 1.0);
					}
				}
			}
		}

		// the trails: Valve's under the devices' own, each of which is in its status' colour then (green with light,
		// yellow without, brighter while it coasted)
		let ts = self.st.trail_seconds;
		if self.st.show_trails {
			if self.st.compare && self.st.show_valve {
				for v in self.vc.devs.values() {
					let tr = &v.trail;
					for i in 1..tr.len() {
						let a = (170.0 * (1.0 - (now - tr[i].t) / ts)) as i32;
						if len(tr[i].p - tr[i - 1].p) < 0.5 {
							cam.line(&p, tr[i - 1].p, tr[i].p, alpha(t.valve, a), 1.3);
						}
					}
				}
			}
			for d in self.rt.devices.values().filter(|d| d.active) {
				let tr = &d.trail;
				for i in 1..tr.len() {
					let a = (235.0 * (1.0 - (now - tr[i].t) / ts)) as i32;
					let c = match tr[i].kind {
						0 => t.good,
						2 => t.warn,
						_ => mix(t.good, t.warn, 0.6),
					};
					if len(tr[i].p - tr[i - 1].p) < 0.5 {
						cam.line(&p, tr[i - 1].p, tr[i].p, alpha(c, a), 1.8);
					}
				}
			}
		}

		// Valve's devices (aligned into sally's frame) and the headset
		if self.st.compare && self.st.show_valve && self.vc.fit_ok {
			for v in self.vc.devs.values().filter(|v| v.have_ours) {
				if let Some(q) = cam.pt(v.ours.p) {
					p.circle_stroke(q, 5.0, Stroke::new(1.5, alpha(t.valve, 220)));
				}
			}
		}
		if self.vc.have_hmd && self.vc.from_raw.is_some() {
			let hp = &self.vc.hmd_ours;
			let c: Vec<V3> = (0..8)
				.map(|i| {
					xform(
						hp,
						V3::new(
							if i & 1 != 0 { 0.09 } else { -0.09 },
							if i & 2 != 0 { 0.05 } else { -0.05 },
							if i & 4 != 0 { 0.05 } else { -0.05 },
						),
					)
				})
				.collect();
			// (in a headset's hue, as every device is in its kind's)
			for (a, b) in EDGES {
				cam.line(&p, c[a], c[b], alpha(t.headset, 200), 1.4);
			}
			if self.st.show_labels {
				if let Some(q) = cam.pt(hp.p) {
					let c = alpha(t.headset, 230);
					tags.push(Tag {
						at: q,
						gap: 12.0,
						text: "Headset (SteamVR's)".into(),
						font: label(),
						color: c,
						depth: None,
					});
				}
			}
		}

		// the devices: their model (SteamVR's, else the outline of their sensors) in their kind's hue, their
		// sensors, axes and label; the models far ones first. The one a click in the room picked has its label
		// alone, until a click on nothing
		let dirs = self.config_dirs();
		self.models.set_steamvr(&self.st.steamvr);
		let mut order: Vec<&crate::runtime::Device> = self.rt.devices.values().filter(|d| d.active).collect();
		order.sort_by(|a, b| {
			let da = dot(a.pos - cam.eye, cam.f);
			let db = dot(b.pos - cam.eye, cam.f);
			db.total_cmp(&da)
		});
		let only = match (&self.sel, &self.room_only) {
			(Sel::Device(c), Some(o)) if c == o => Some(o.clone()),
			_ => None,
		};
		for d in order {
			let stale = now - d.last_pose_wall > 1.0;
			let pose = Pose { p: d.pos, q: d.rot };
			let a = if stale { 80 } else { 255 };
			let kind = t.kind(&d.subtype);
			// the sensors in the head frame (the runtime gives them in the IMU's), each in its place: index i is sensor i
			let sensors: Vec<Option<(Pos2, f64)>> = (0..d.sensor_pos.len())
				.map(|i| d.sensor_in_head(i).and_then(|(s, _)| cam.proj(xform(&pose, s))))
				.collect();
			let mut name = String::new();
			let model = if self.st.show_models {
				name = self.models.name_by_serial(&d.serial, &dirs);
				if name.is_empty() {
					name = model_name_for(&d.subtype).to_string();
				}
				self.models.get(&name)
			} else {
				None
			};
			if let Some(m) = model {
				// the runtime's head frame into Valve's (the model's), as the comparison has learnt it, else as the
				// model's kind has it
				let off = self
					.vc
					.devs
					.get(&d.serial)
					.filter(|v| v.rot_offset_n >= 60)
					.map(|v| v.rot_offset)
					.unwrap_or_else(|| crate::models::head_turn(&name));
				let tint = mix(Color32::from_rgb(208, 211, 220), kind, 0.45);
				cam.model(&p, &m, &model_pose(d.pos, d.rot, off), tint, if stale { 90 } else { 255 });
			} else if self.st.show_models && sensors.iter().flatten().count() >= 3 {
				let outline = hull(sensors.iter().flatten().map(|s| s.0).collect());
				if outline.len() >= 3 {
					p.add(Shape::closed_line(outline, Stroke::new(1.2, alpha(kind, if stale { 60 } else { 170 }))));
				}
			}
			// its axes in ink, x the strongest and z the faintest: red and green are a status' here
			let al = 0.07;
			for (axis, strength) in
				[(V3::new(al, 0.0, 0.0), 1.0), (V3::new(0.0, al, 0.0), 0.62), (V3::new(0.0, 0.0, al), 0.36)]
			{
				cam.line(&p, d.pos, xform(&pose, axis), alpha(fade(t.text, strength), a), 2.0);
			}
			if self.st.show_sensors {
				let fresh = now - d.hits_wall < 2.5;
				for (i, (q, depth)) in sensors.iter().enumerate().filter_map(|(i, s)| s.map(|s| (i, s))) {
					let lit = fresh && d.sensor_hits.get(i).is_some_and(|&h| h > 0);
					let rad = (0.006 * cam.focal / depth).clamp(1.2, 4.0) as f32;
					p.circle_filled(q, rad, if lit { alpha(kind, a) } else { alpha(t.faint, a) });
				}
			}
			if let Some(q) = cam.pt(d.pos) {
				picks.push((q, Pick::Device(d.codename.clone())));
				let selected = self.sel == Sel::Device(d.codename.clone());
				if selected {
					p.circle_stroke(q, 15.0, Stroke::new(1.8, t.accent));
				}
				if self.st.show_labels && only.as_ref().is_none_or(|o| *o == d.codename) {
					// (its name in ink, in yellow once it has been without light for the hold)
					let c = if self.state_level(d) == 2 { t.warn } else { t.text };
					tags.push(Tag {
						at: q,
						gap: if selected { 21.0 } else { 13.0 },
						text: self.short_name(d),
						font: if selected { semibold(13.0) } else { label() },
						color: alpha(c, a),
						depth: Some(if selected { 0.0 } else { dot(d.pos - cam.eye, cam.f) }),
					});
				}
			}
		}

		// the labels, each at its mark's right: the nearest device's on top and strongest
		draw_tags(&p, tags);

		// over it: the coverage's key
		if self.st.show_heat {
			let mut x = free.max.x - 12.0;
			let y = free.max.y - 16.0;
			for (c, s) in [
				(alpha(t.bad, 120), "no light"),
				(alpha(t.warn, 90), "one station"),
				(alpha(t.good, 110), "two or more"),
			] {
				let tr = kit::text(&p, Pos2::new(x, y), Align2::RIGHT_CENTER, s, label(), t.muted);
				x = tr.min.x - 14.0;
				p.rect_filled(Rect::from_center_size(Pos2::new(x + 4.0, y), Vec2::splat(9.0)), 0.0, c);
				x -= 14.0;
			}
		}
		if self.active_devices().is_empty() && self.rt.stations.is_empty() {
			let s = if self.rt.connected(now) { "No poses yet" } else { "No runtime" };
			kit::text(&p, cam.c, Align2::CENTER_CENTER, s, body(), t.muted);
		}

		// a click (a release without a drag) selects; a double click follows
		if resp.clicked() {
			if let Some(mp) = resp.interact_pointer_pos() {
				let best = picks
					.iter()
					.map(|(q, k)| (q.distance(mp), k))
					.filter(|(dd, _)| *dd < 22.0)
					.min_by(|a, b| a.0.total_cmp(&b.0));
				self.sel = match best {
					Some((_, Pick::Device(c))) => Sel::Device(c.clone()),
					Some((_, Pick::Station(i))) => Sel::Station(*i),
					None => Sel::None,
				};
				// (a device picked here has its label alone)
				self.room_only = match &self.sel {
					Sel::Device(c) => Some(c.clone()),
					_ => None,
				};
			}
		}
		if resp.double_clicked() && matches!(self.sel, Sel::Device(_)) {
			self.st.follow = true;
			self.st.dirty = true;
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn labels_are_drawn_from_the_farthest_and_the_nearest_is_the_strongest() {
		// a station's label, then devices at 3, 1 and 2 m, and the selected one (0)
		let order = tag_order(&[Some(3.0), None, Some(1.0), Some(2.0), Some(0.0)]);
		assert_eq!(order.iter().map(|(i, _)| *i).collect::<Vec<_>>(), [1, 0, 3, 2, 4]);
		assert_eq!(order.iter().map(|(_, s)| *s).collect::<Vec<_>>(), [1.0, TAG_UNDER, TAG_UNDER, TAG_UNDER, 1.0]);
		// one device alone is at full strength, and none is no trouble
		assert_eq!(tag_order(&[Some(2.0)]), [(0, 1.0)]);
		assert!(tag_order(&[]).is_empty());
	}

	#[test]
	fn the_hull_of_a_square_and_its_middle() {
		let pts = vec![
			Pos2::new(0.0, 0.0),
			Pos2::new(1.0, 0.0),
			Pos2::new(1.0, 1.0),
			Pos2::new(0.0, 1.0),
			Pos2::new(0.5, 0.5),
		];
		let h = hull(pts);
		assert_eq!(h.len(), 4);
		assert!(!h.contains(&Pos2::new(0.5, 0.5)));
	}

	#[test]
	fn a_grid_line_is_a_pixel_and_a_half_and_fades_where_the_lines_crowd() {
		// eye 1.6 m over the floor, looking along +x a little down: the line along x runs off to the horizon
		let cam = Camera {
			target: V3::new(2.0, 0.0, 0.0),
			yaw: std::f64::consts::PI,
			pitch: 0.6,
			dist: 2.0,
			..Default::default()
		};
		let c3 = Cam3::new(&cam, Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0)));
		let mut mesh = egui::Mesh::default();
		let (a, b) = (V3::new(-1.0, 0.0, 0.0), V3::new(200.0, 0.0, 0.0));
		c3.grid_line(&mut mesh, a, b, V3::new(0.0, 1.0, 0.0), 0.25, Color32::WHITE, 1.0);
		// all of it in front of the near plane: four vertices at each piece's end, the outer two clear
		assert_eq!(mesh.vertices.len() % 4, 0);
		assert!(mesh.vertices.len() / 4 > 40);
		let quad = |k: usize| &mesh.vertices[4 * k..4 * k + 4];
		let first = quad(0);
		assert_eq!(first[0].color, Color32::TRANSPARENT);
		assert_eq!(first[3].color, Color32::TRANSPARENT);
		assert_eq!(first[1].pos.distance(first[2].pos), 2.0 * GRID_CORE);
		assert!((first[0].pos.distance(first[3].pos) - 2.0 * GRID_EDGE).abs() < 1e-4);
		// near the eye the neighbours (0.25 m off) are far apart on the screen: whole; 200 m off they are not: gone
		assert_eq!(first[1].color, Color32::WHITE.gamma_multiply(GRID_INK));
		let last = quad(mesh.vertices.len() / 4 - 1);
		assert_eq!(last[1].color, Color32::TRANSPARENT);
		assert_eq!(mesh.indices.len(), (mesh.vertices.len() / 4 - 1) * 18);
		assert_eq!((grid_fade(2.0), grid_fade(7.0), grid_fade(20.0)), (0.0, 1.0, 1.0));
	}

	#[test]
	fn a_grid_line_through_the_eye_is_cut_at_the_near_plane() {
		let cam = Camera { target: V3::new(0.0, 0.0, 0.0), yaw: 0.0, pitch: 0.3, dist: 2.0, ..Default::default() };
		let c3 = Cam3::new(&cam, Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0)));
		let eye = c3.eye();
		let mut mesh = egui::Mesh::default();
		// a line under the eye, from behind it to in front of it: drawn, and every vertex a finite place
		let (a, b) = (V3::new(eye.x + 5.0, eye.y, 0.0), V3::new(eye.x - 5.0, eye.y, 0.0));
		c3.grid_line(&mut mesh, a, b, V3::new(0.0, 1.0, 0.0), 0.25, Color32::WHITE, 1.0);
		assert!(!mesh.vertices.is_empty());
		assert!(mesh.vertices.iter().all(|v| v.pos.x.is_finite() && v.pos.y.is_finite()));
		// one wholly behind it: nothing
		let mut none = egui::Mesh::default();
		let (a, b) = (V3::new(eye.x + 5.0, eye.y - 1.0, 0.0), V3::new(eye.x + 5.0, eye.y + 1.0, 0.0));
		c3.grid_line(&mut none, a, b, V3::new(1.0, 0.0, 0.0), 0.25, Color32::WHITE, 1.0);
		assert!(none.vertices.is_empty());
	}

	#[test]
	fn the_floor_grid_takes_stations_within_reach_only() {
		// the play area alone: 6 m of 0.25 m squares each way
		assert_eq!(grid_squares(std::iter::empty(), 0.25), (-12, 12, -12, 12));
		// a station 4.5 m out widens it to a metre past it
		let near = V3::new(4.5, -2.0, 2.4);
		assert_eq!(grid_squares([near].into_iter(), 0.25), (-12, 22, -12, 12));
		// places that are not numbers, or absurd, change nothing (they made lines without end)
		let absurd = [V3::new(f64::INFINITY, 0.0, 0.0), V3::new(f64::NAN, f64::NAN, 2.0), V3::new(1e300, -1e300, 0.0)];
		assert_eq!(grid_squares(absurd.into_iter().chain([near]), 0.25), (-12, 22, -12, 12));
	}

	#[test]
	fn a_point_ahead_projects_to_the_centre() {
		let cam = Camera { target: V3::new(1.0, 2.0, 0.5), ..Default::default() };
		let c3 = Cam3::new(&cam, Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0)));
		let (q, z) = c3.proj(cam.target).unwrap();
		assert!((q.x - 200.0).abs() < 1e-3 && (q.y - 150.0).abs() < 1e-3);
		assert!((z - cam.dist).abs() < 1e-9);
		assert!(c3.proj(c3.eye - c3.f).is_none());
	}
}
