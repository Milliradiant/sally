//! The room: the 3D view of the stations, their coverage, the devices (their shape, sensors and axes), their trails,
//! Valve's devices aligned into sally's frame, and the coverage heat map on the floor. Drawn with egui's painter after
//! projecting on the CPU, as the C++ panel did; orbit with the left button, pan with the right or middle one, zoom with
//! the wheel, click to select, double-click to follow. Locate turns the view to a device.

use egui::{Align2, Color32, Id, PointerButton, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use sally_math::math3d::{cross, dot, len, norm, qconj, qmul, qrot, xform, Pose, Q4, V3};

use crate::models::{shade_by, Model};

use super::icons;
use super::kit::{self, Pop, Surface, CTL_H, HEADER_H};
use super::theme::*;
use super::{Panel, Sel};

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

	/// A station's box (its corners from its pose)
	pub(super) fn station(&self, p: &egui::Painter, pose: &Pose, col: Color32) {
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
/// what it says. `rank`: who places first (0 the selected device, 1 a station, 2 a device, 3 the rest)
pub(super) struct Tag {
	pub at: Pos2,
	pub gap: f32,
	pub text: String,
	pub font: egui::FontId,
	pub color: Color32,
	pub rank: u8,
}

/// Draws a room's labels within `bounds`, each beside its mark where there is room (the selected device's first),
/// else near it with a thin line to it (`place_tags`)
pub(super) fn draw_tags(p: &egui::Painter, mut tags: Vec<Tag>, bounds: Rect) {
	tags.sort_by(|a, b| a.rank.cmp(&b.rank).then_with(|| a.text.cmp(&b.text)));
	let sizes: Vec<(Pos2, f32, Vec2)> =
		tags.iter().map(|g| (g.at, g.gap, p.layout_no_wrap(g.text.clone(), g.font.clone(), g.color).size())).collect();
	for (g, (lr, away)) in tags.iter().zip(place_tags(&sizes, bounds)) {
		if away {
			let to = Pos2::new(g.at.x.clamp(lr.min.x, lr.max.x), g.at.y.clamp(lr.min.y, lr.max.y));
			let from = g.at + (to - g.at).normalized() * (g.gap * 0.55);
			p.line_segment([from, to], Stroke::new(1.0, alpha(g.color, 90)));
		}
		kit::text(p, lr.left_center(), Align2::LEFT_CENTER, &g.text, g.font.clone(), g.color);
	}
}

/// Where each label goes, in the order given (the first has the first pick), from its mark, the gap to it and its
/// size: at the mark's right where that is free, else at its left, else the nearest free place above or below those.
/// Free: inside `bounds`, on no label placed before it, and on no other label's mark. The label's rect, and whether it
/// is away from its mark (a line joins them then). One with no free place goes at its mark's right all the same
fn place_tags(tags: &[(Pos2, f32, Vec2)], bounds: Rect) -> Vec<(Rect, bool)> {
	let mut placed: Vec<Rect> = Vec::new();
	let mut out = Vec::new();
	for (i, (at, gap, size)) in tags.iter().enumerate() {
		let right = Rect::from_min_size(Pos2::new(at.x + gap, at.y - size.y * 0.5), *size);
		let left = Rect::from_min_size(Pos2::new(at.x - gap - size.x, at.y - size.y * 0.5), *size);
		let mut cands = vec![(right, false), (left, false)];
		for ring in [1.0f32, 2.0, 3.0, 4.0] {
			let dy = ring * (size.y + 2.0);
			for (r, dx) in [(right, 2.0), (left, -2.0)] {
				cands.push((r.translate(Vec2::new(dx, -dy)), true));
				cands.push((r.translate(Vec2::new(dx, dy)), true));
			}
		}
		let free = |r: &Rect| {
			bounds.contains_rect(*r)
				&& !placed.iter().any(|q| q.expand(2.0).intersects(*r))
				&& !tags.iter().enumerate().any(|(j, (o, g, _))| j != i && r.expand(g * 0.5).contains(*o))
		};
		let pick = cands.iter().find(|(r, _)| free(r)).copied().unwrap_or(cands[0]);
		placed.push(pick.0);
		out.push(pick);
	}
	out
}

/// SteamVR's chaperone's walls in the room
const WALL: Color32 = Color32::from_rgb(124, 196, 206);

/// A row of a pop-up: a label, its value, and a slider under them; whether the value moved
fn pop_slider(ui: &mut egui::Ui, id: Id, text: &str, value: &str, v: &mut f64, range: &kit::Range) -> bool {
	let t = tk();
	let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 52.0), Sense::hover());
	let p = ui.painter();
	kit::text(p, Pos2::new(rr.min.x, rr.min.y + 11.0), Align2::LEFT_CENTER, text, body(), t.text);
	kit::text(p, Pos2::new(rr.max.x, rr.min.y + 11.0), Align2::RIGHT_CENTER, value, body(), t.soft);
	let sr = Rect::from_min_max(Pos2::new(rr.min.x, rr.min.y + 22.0), rr.max);
	kit::slider(ui, id, sr, v, range, true).changed
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

	/// The room in its box. In the header: Follow (the view goes with the selected device), the whole room in view
	/// again, and what the room shows (the layers, the trails' and the cones' lengths)
	pub(super) fn room_card(&mut self, ui: &mut egui::Ui, r: Rect) {
		self.card(ui, r, "Room");
		let hr = Self::header(r);
		let bh = 28.0;
		let y = hr.center().y - bh * 0.5;
		let layers_r = Rect::from_min_size(Pos2::new(hr.max.x - 4.0 - bh, y), Vec2::splat(bh));
		let open = self.pop == Some((Pop::Layers, self.view));
		if kit::icon_button(ui, Id::unique("room-layers"), layers_r, icons::STACK, open, true)
			.on_hover_text("What the room shows")
			.clicked()
		{
			self.toggle_pop(Pop::Layers);
		}
		let reset_r = layers_r.translate(Vec2::new(-(bh + 4.0), 0.0));
		if kit::icon_button(ui, Id::unique("room-reset"), reset_r, icons::CORNERS_OUT, false, true)
			.on_hover_text("The whole room in view")
			.clicked()
		{
			self.frame_camera();
		}
		let pad = 22.0;
		let fw = kit::text_w(ui.painter(), "Follow", label()) + pad;
		let (_, c) = kit::chip(
			ui,
			Id::unique("room-follow"),
			Pos2::new(reset_r.min.x - 6.0 - fw, y),
			bh,
			"Follow",
			self.st.follow,
			None,
			pad,
		);
		if c {
			self.st.follow = !self.st.follow;
			self.st.dirty = true;
		}
		let canvas = Rect::from_min_max(
			Pos2::new(r.min.x + 8.0, r.min.y + HEADER_H),
			Pos2::new(r.max.x - 8.0, (r.max.y - 8.0).max(r.min.y + HEADER_H + 20.0)),
		);
		self.room_contents(ui, canvas);
		self.popup(ui, Pop::Layers, layers_r, 300.0, true, |s, ui| s.layers_pop(ui));
	}

	/// What the room shows
	fn layers_pop(&mut self, ui: &mut egui::Ui) {
		ui.spacing_mut().item_spacing.y = 2.0;
		let mut rows: Vec<(&str, &str)> = vec![
			("models", "Models"),
			("sensors", "Sensors"),
			("rays", "Light rays"),
			("cones", "Station cones"),
			("trails", "Trails"),
			("labels", "Labels"),
		];
		if self.st.compare {
			rows.push(("valve", "Valve's poses"));
		}
		if !self.vc.walls.is_empty() {
			rows.push(("walls", "SteamVR's walls"));
		}
		rows.push(("heat", "Coverage on the floor"));
		for (key, text) in rows {
			let st = &mut self.st;
			let v = match key {
				"models" => &mut st.show_models,
				"sensors" => &mut st.show_sensors,
				"rays" => &mut st.show_rays,
				"cones" => &mut st.show_frustums,
				"trails" => &mut st.show_trails,
				"labels" => &mut st.show_labels,
				"valve" => &mut st.show_valve,
				"walls" => &mut st.show_walls,
				_ => &mut st.show_heat,
			};
			if super::tracking::switch_row(ui, Id::unique(("layer", key)), text, *v) {
				*v = !*v;
				st.dirty = true;
			}
		}
		if self.st.show_heat {
			let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), CTL_H), Sense::hover());
			let s = format!("{} floor squares this session", self.rt.voxels.len());
			kit::text(ui.painter(), Pos2::new(rr.min.x, rr.center().y), Align2::LEFT_CENTER, &s, label(), tk().muted);
			let bw = kit::button_w(ui.painter(), "Clear", false);
			let br = Rect::from_min_size(Pos2::new(rr.max.x - bw, rr.min.y + 2.0), Vec2::new(bw, 28.0));
			if kit::button(ui, Id::unique("heat-clear"), br, "Clear", None, kit::Look::Plain, true).clicked() {
				self.rt.clear_heat();
			}
		}
		ui.add_space(6.0);
		let mut v = self.st.trail_seconds;
		let range = kit::Range { min: 1.0, max: 30.0, step: 1.0, default: Some(6.0), inherits: None };
		if pop_slider(ui, Id::unique("trail-len"), "Trails last", &format!("{v:.0} s"), &mut v, &range) {
			self.st.trail_seconds = v;
			self.rt.trail_seconds = v;
			self.vc.trail_seconds = v;
			self.st.dirty = true;
		}
		let mut v = self.st.frustum_len;
		let range = kit::Range { min: 1.0, max: 6.0, step: 0.5, default: Some(3.0), inherits: None };
		if pop_slider(ui, Id::unique("cone-len"), "Cones reach", &format!("{v:.1} m"), &mut v, &range) {
			self.st.frustum_len = v;
			self.st.dirty = true;
		}
	}

	/// The floor's pop-up: set it from the lowest device now, or let the panel find it again
	fn floor_pop(&mut self, ui: &mut egui::Ui) {
		let t = tk();
		super::topbar::pop_title(ui, "Floor");
		let s = if self.st.floor_z.is_nan() {
			"Found by itself: 3 cm under the lowest a tracking device has been."
		} else {
			"Set by hand. Auto finds it again from the devices."
		};
		ui.label(egui::RichText::new(s).font(label()).color(t.muted));
		ui.add_space(6.0);
		let (rr, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), CTL_H), Sense::hover());
		let w = (rr.width() - 8.0) / 2.0;
		let a = Rect::from_min_size(rr.min, Vec2::new(w, CTL_H));
		let b = Rect::from_min_size(Pos2::new(rr.min.x + w + 8.0, rr.min.y), Vec2::new(w, CTL_H));
		let conn = self.rt.connected(self.now);
		if kit::button(ui, Id::unique("floor-set"), a, "Set it here", None, kit::Look::Plain, conn)
			.on_hover_text("3 cm under the lowest device that is tracking now: put a tracker on the floor")
			.clicked()
		{
			self.set_floor();
		}
		if kit::button(ui, Id::unique("floor-auto"), b, "Auto", None, kit::Look::Plain, !self.st.floor_z.is_nan())
			.clicked()
		{
			self.st.floor_z = f64::NAN;
			self.auto_floor = f64::NAN;
			self.st.dirty = true;
		}
	}

	fn room_contents(&mut self, ui: &mut egui::Ui, canvas: Rect) {
		let t = tk();
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
		if camera_input(&mut self.cam, ui, &resp, canvas) && self.st.follow {
			self.st.follow = false;
			self.st.dirty = true;
		}
		let cam = Cam3::new(&self.cam, canvas);
		let p = ui.painter_at(canvas);
		self.surface(&p, canvas, Surface::Well);
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
				let c = if dark > 0.5 {
					t.bad
				} else if avg >= 1.6 {
					t.good
				} else if avg >= 0.8 {
					t.warn
				} else {
					t.bad
				};
				let (x, y) = (ix as f64 * vs, iy as f64 * vs);
				let q = [
					V3::new(x + g, y + g, fz),
					V3::new(x + vs - g, y + g, fz),
					V3::new(x + vs - g, y + vs - g, fz),
					V3::new(x + g, y + vs - g, fz),
				];
				cam.poly(&p, &q, alpha(c, 110));
			}
		}

		// the floor grid: squares the size of a coverage cell, a brighter line every metre
		{
			let step = self.rt.voxel_size;
			let stations = self.rt.stations.values().filter(|s| s.have_pose).map(|s| s.pose.p);
			cam.grid(&p, grid_squares(stations, step), step, fz, t.grid, t.grid_minor);
		}

		// SteamVR's chaperone: its walls, faint, their floor edges (a wall's first two corners) brighter
		if self.st.show_walls {
			for q in &self.vc.walls {
				cam.poly(&p, q, alpha(WALL, 10));
				for i in 0..4 {
					let floor = i == 0;
					cam.line(
						&p,
						q[i],
						q[(i + 1) % 4],
						alpha(WALL, if floor { 150 } else { 60 }),
						if floor { 1.6 } else { 1.0 },
					);
				}
			}
		}

		// the stations: a box each, and their coverage out to the cones' length
		for (idx, sn) in &self.rt.stations {
			if !sn.have_pose {
				continue;
			}
			let selected = self.sel == Sel::Station(*idx);
			let pose = &sn.pose;
			let bc = if selected {
				t.on
			} else if sn.seen {
				t.soft
			} else {
				t.faint
			};
			cam.station(&p, pose, bc);
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
				let face = alpha(
					t.soft,
					if selected {
						18
					} else if sn.seen {
						7
					} else {
						3
					},
				);
				for i in 0..4 {
					cam.poly(&p, &[pose.p, far[i], far[(i + 1) % 4]], face);
				}
				let fc = alpha(
					t.soft,
					if selected {
						190
					} else if sn.seen {
						64
					} else {
						32
					},
				);
				for i in 0..4 {
					cam.line(&p, pose.p, far[i], fc, 1.0);
					cam.line(&p, far[i], far[(i + 1) % 4], fc, 1.0);
				}
			}
			if let Some(sp) = cam.pt(pose.p) {
				picks.push((sp, Pick::Station(*idx)));
				if selected {
					p.circle_stroke(sp, 14.0, Stroke::new(1.6, t.on));
				}
				if self.st.show_labels {
					let c = if sn.seen { t.soft } else { t.faint };
					tags.push(Tag { at: sp, gap: 12.0, text: sn.channel_label(), font: label(), color: c, rank: 1 });
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
						cam.line(&p, s.pose.p, d.pos, alpha(if l.sensors_lit < 3 { t.note } else { t.good }, 38), 1.0);
					}
				}
			}
		}

		// the trails
		let ts = self.st.trail_seconds;
		if self.st.show_trails {
			for d in self.rt.devices.values().filter(|d| d.active) {
				let tr = &d.trail;
				for i in 1..tr.len() {
					let a = (220.0 * (1.0 - (now - tr[i].t) / ts)) as i32;
					let c = match tr[i].kind {
						0 => t.on,
						2 => t.bad,
						_ => t.warn,
					};
					if len(tr[i].p - tr[i - 1].p) < 0.5 {
						cam.line(&p, tr[i - 1].p, tr[i].p, alpha(c, a), 1.6);
					}
				}
			}
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
		}

		// Valve's devices (aligned into sally's frame) and the headset
		if self.st.compare && self.st.show_valve && self.vc.fit_ok {
			for v in self.vc.devs.values().filter(|v| v.have_ours) {
				if let Some(q) = cam.pt(v.ours.p) {
					p.circle_stroke(q, 5.0, Stroke::new(1.5, alpha(t.valve, 220)));
				}
			}
		}
		if self.vc.have_hmd && self.vc.fit_ok {
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
			for (a, b) in EDGES {
				cam.line(&p, c[a], c[b], alpha(t.violet, 200), 1.4);
			}
			if self.st.show_labels {
				if let Some(q) = cam.pt(hp.p) {
					let c = alpha(t.violet, 230);
					tags.push(Tag {
						at: q,
						gap: 12.0,
						text: "Headset (SteamVR's)".into(),
						font: label(),
						color: c,
						rank: 3,
					});
				}
			}
		}

		// the devices: their model (SteamVR's, else the outline of their sensors), sensors, axes and label; the
		// models far ones first
		let dirs = self.config_dirs();
		self.models.set_steamvr(&self.st.steamvr);
		let mut order: Vec<&crate::runtime::Device> = self.rt.devices.values().filter(|d| d.active).collect();
		order.sort_by(|a, b| {
			let da = dot(a.pos - cam.eye, cam.f);
			let db = dot(b.pos - cam.eye, cam.f);
			db.total_cmp(&da)
		});
		for d in order {
			let stale = now - d.last_pose_wall > 1.0;
			let pose = Pose { p: d.pos, q: d.rot };
			let a = if stale { 80 } else { 255 };
			let sc = self.state_color(d);
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
				let tint = mix(Color32::from_rgb(208, 211, 220), sc, 0.25);
				cam.model(&p, &m, &model_pose(d.pos, d.rot, off), tint, if stale { 90 } else { 255 });
			} else if self.st.show_models && sensors.iter().flatten().count() >= 3 {
				let outline = hull(sensors.iter().flatten().map(|s| s.0).collect());
				if outline.len() >= 3 {
					p.add(Shape::convex_polygon(outline.clone(), alpha(sc, if stale { 20 } else { 46 }), Stroke::NONE));
					p.add(Shape::closed_line(outline, Stroke::new(1.2, alpha(sc, if stale { 60 } else { 150 }))));
				}
			}
			let al = 0.07;
			cam.line(&p, d.pos, xform(&pose, V3::new(al, 0.0, 0.0)), alpha(AXIS_X, a), 2.0);
			cam.line(&p, d.pos, xform(&pose, V3::new(0.0, al, 0.0)), alpha(AXIS_Y, a), 2.0);
			cam.line(&p, d.pos, xform(&pose, V3::new(0.0, 0.0, al)), alpha(AXIS_Z, a), 2.0);
			if self.st.show_sensors {
				let fresh = now - d.hits_wall < 2.5;
				for (i, (q, depth)) in sensors.iter().enumerate().filter_map(|(i, s)| s.map(|s| (i, s))) {
					let lit = fresh && d.sensor_hits.get(i).is_some_and(|&h| h > 0);
					let rad = (0.006 * cam.focal / depth).clamp(1.2, 4.0) as f32;
					p.circle_filled(q, rad, if lit { alpha(t.good, a) } else { alpha(t.faint, a) });
				}
			}
			if let Some(q) = cam.pt(d.pos) {
				picks.push((q, Pick::Device(d.codename.clone())));
				let selected = self.sel == Sel::Device(d.codename.clone());
				if selected {
					p.circle_stroke(q, 15.0, Stroke::new(1.8, t.on));
				}
				if self.st.show_labels {
					let c = if selected { t.text } else { mix(t.soft, sc, 0.35) };
					tags.push(Tag {
						at: q,
						gap: if selected { 21.0 } else { 13.0 },
						text: self.short_name(d),
						font: if selected { semibold(13.0) } else { label() },
						color: alpha(c, a),
						rank: if selected { 0 } else { 2 },
					});
				}
			}
		}

		// the labels, each beside its mark where there is room (the selected device's first), else near it with a line
		// to it
		draw_tags(&p, tags, canvas.shrink(4.0));

		// over it: the floor (a click opens what sets it), the coverage's key
		let ft = if self.st.floor_z.is_nan() { format!("Floor {fz:.2} m") } else { format!("Floor {fz:.2} m, set") };
		let fw = kit::text_w(&p, &ft, label()) + 20.0;
		let floor_r = Rect::from_min_size(canvas.min + Vec2::new(8.0, 8.0), Vec2::new(fw, 26.0));
		let fresp = kit::button(ui, Id::unique("room-floor"), floor_r, "", None, kit::Look::Ghost, true);
		kit::text(&p, floor_r.center(), Align2::CENTER_CENTER, &ft, label(), t.muted);
		if fresp.clicked() {
			self.toggle_pop(Pop::Floor);
		}
		if self.st.show_heat {
			let mut x = canvas.max.x - 12.0;
			let y = canvas.max.y - 16.0;
			for (c, s) in [(t.bad, "no light"), (t.warn, "one station"), (t.good, "two or more")] {
				let tr = kit::text(&p, Pos2::new(x, y), Align2::RIGHT_CENTER, s, label(), t.muted);
				x = tr.min.x - 14.0;
				p.rect_filled(Rect::from_center_size(Pos2::new(x + 4.0, y), Vec2::splat(9.0)), 2.0, alpha(c, 200));
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
			}
		}
		if resp.double_clicked() && matches!(self.sel, Sel::Device(_)) {
			self.st.follow = true;
			self.st.dirty = true;
		}
		self.popup(ui, Pop::Floor, floor_r, 300.0, false, |s, ui| s.floor_pop(ui));
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn labels_keep_off_each_other_and_off_the_marks() {
		let bounds = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 300.0));
		let size = Vec2::new(80.0, 16.0);
		// three marks in a huddle, a fourth alone, a fifth against the right edge
		let tags = [
			(Pos2::new(200.0, 150.0), 13.0, size),
			(Pos2::new(206.0, 154.0), 13.0, size),
			(Pos2::new(196.0, 158.0), 13.0, size),
			(Pos2::new(60.0, 40.0), 13.0, size),
			(Pos2::new(380.0, 100.0), 13.0, size),
		];
		let got = place_tags(&tags, bounds);
		assert_eq!(got.len(), 5);
		for (i, (a, _)) in got.iter().enumerate() {
			assert!(bounds.contains_rect(*a), "{i}: {a:?}");
			for (j, (b, _)) in got.iter().enumerate().skip(i + 1) {
				assert!(!a.intersects(*b), "{i} on {j}: {a:?} {b:?}");
			}
			for (j, (o, _, _)) in tags.iter().enumerate() {
				assert!(i == j || !a.contains(*o), "{i} on the mark of {j}");
			}
		}
		// the first is at its mark's right, the one alone too; the one at the edge at its left
		assert_eq!(got[0], (Rect::from_min_size(Pos2::new(213.0, 142.0), size), false));
		assert!(!got[3].1 && got[3].0.min.x > 60.0);
		assert!(!got[4].1 && got[4].0.max.x < 380.0);
		// the huddle's others are away from their marks
		assert!(got[1].1 || got[2].1);
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
