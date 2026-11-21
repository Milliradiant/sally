//! A picture of the user's own behind the panel, and the glass it makes of the boxes. The picture is one of the
//! `wallpapers` folder beside the panel's settings (none ship with the program); with one chosen every box is a
//! pane of glass over it.
//!
//! The glass is made to be read through both ways. The picture under a pane is blurred only a little, so that what
//! is in it can still be made out, and what is written on the pane stays clear for three things done to that copy:
//! the little blur takes away the fine grain a text would get lost in, what is bright in it comes down (a text is
//! light: a cloud or a lamp behind it would take it away), and it is drawn darkened, by multiplying, which keeps
//! its colours. A pane's own look is its edge: a rim of light, brighter along the top, a dark line at its foot, a
//! shadow under it, and the picture inside drawn a little larger than around it, as thick glass shows it.
//!
//! Pictures are read off the frame's thread (a large one takes a moment): the one shown stays until the next is
//! ready. The Settings tab's row of small copies asks for them the same way.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver};
use std::sync::Arc;

use egui::epaint::{Mesh, RectShape, Shadow, Vertex};
use egui::{
	Color32, ColorImage, CornerRadius, Painter, Pos2, Rect, Stroke, StrokeKind, TextureHandle, TextureOptions, Vec2,
};

use super::theme::white;

/// How much of the picture's light is left around the boxes
const AROUND: f32 = 0.62;
/// The picture under a pane is drawn this much larger than around it
const LENS: f32 = 1.018;
/// How wide a pane's cut edge is (points)
const BEVEL: f32 = 5.0;
/// Under a pane the picture's brightness (its strongest channel, 0 to 1) above the knee rises this much slower:
/// white comes down to 0.64 of white
const KNEE: f32 = 0.46;
const SLOPE: f32 = 0.34;
/// and its colours are this much stronger, since they are about to be darkened
const SATURATION: f32 = 1.15;

/// The blur's settings (0: none; each step about a point on a window 1600 wide) and the glass' darkness (the part
/// of the picture's light a pane takes away, %)
pub const BLUR: (f64, f64, f64) = (0.0, 10.0, 3.0);
pub const DARK: (f64, f64, f64) = (50.0, 90.0, 70.0);

/// A picture and its copy for under the panes
type Pair = (ColorImage, ColorImage);
/// What a copy is of: a file's name ("" for none) and the blur it was made with
type Key = (String, u8);

pub struct Wallpaper {
	/// what the images below are of
	loaded: Key,
	/// what is being made, and where it arrives
	loading: Option<(Key, Receiver<Result<Pair, String>>)>,
	image: Option<Arc<ColorImage>>,
	frosted: Option<Arc<ColorImage>>,
	/// each view's textures of them (an egui context has its own)
	textures: [Option<(TextureHandle, TextureHandle)>; 2],
	/// why the last one did not load
	pub error: Option<String>,
	/// what the panel covers in this frame, and how much of the picture's light a pane leaves
	screen: Rect,
	light: f32,
	/// the small copies by file name (None: asked for, not there yet), each view's texture of them
	thumbs: BTreeMap<String, Option<Arc<ColorImage>>>,
	thumb_rx: Option<Receiver<(String, Option<ColorImage>)>>,
	thumb_tex: [BTreeMap<String, TextureHandle>; 2],
}

impl Default for Wallpaper {
	fn default() -> Wallpaper {
		Wallpaper {
			loaded: (String::new(), 0),
			loading: None,
			image: None,
			frosted: None,
			textures: [None, None],
			error: None,
			screen: Rect::NOTHING,
			light: 0.3,
			thumbs: BTreeMap::new(),
			thumb_rx: None,
			thumb_tex: [BTreeMap::new(), BTreeMap::new()],
		}
	}
}

/// The pictures' folder: beside the panel's settings
pub fn folder(cfg_path: &Path) -> PathBuf {
	cfg_path.parent().map(|p| p.join("wallpapers")).unwrap_or_else(|| PathBuf::from("wallpapers"))
}

/// Whether a file is a picture the panel reads, by its name
pub fn is_picture(name: &str) -> bool {
	let l = name.to_lowercase();
	[".png", ".jpg", ".jpeg", ".bmp"].iter().any(|x| l.ends_with(x))
}

/// The pictures in the folder, sorted by name
pub fn files(dir: &Path) -> Vec<String> {
	let mut out: Vec<String> = std::fs::read_dir(dir)
		.into_iter()
		.flatten()
		.flatten()
		.filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
		.filter_map(|e| e.file_name().into_string().ok())
		.filter(|n| is_picture(n))
		.collect();
	out.sort();
	out
}

/// A picture from elsewhere copied into the folder (a file dropped on the window): the name it has there. One of
/// that name already there is written over: it is the same picture again, or the newer one of it
pub fn bring(dir: &Path, from: &Path) -> Result<String, String> {
	let name = from.file_name().and_then(|n| n.to_str()).ok_or("a file without a name")?.to_string();
	if !is_picture(&name) {
		return Err(format!("{name} is not a picture the panel reads (png, jpg, bmp)"));
	}
	std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
	let to = dir.join(&name);
	if from != to {
		std::fs::copy(from, &to).map_err(|e| e.to_string())?;
	}
	Ok(name)
}

impl Wallpaper {
	/// A picture is behind the boxes
	pub fn shown(&self) -> bool {
		self.image.is_some()
	}

	/// A picture is being read
	pub fn busy(&self) -> bool {
		self.loading.is_some()
	}

	/// Sees to it that what is behind the boxes is `name` from `dir` ("" for none), its copy for under the panes
	/// blurred by `blur`. A picture is read on a thread of its own and takes the place of what is shown when it is
	/// ready (a change of the blur alone makes the copy again, from the picture in hand); the error of one that did
	/// not load is returned once
	pub fn update(&mut self, dir: &Path, name: &str, blur: f64) -> Option<String> {
		let blur = blur.clamp(BLUR.0, BLUR.1).round() as u8;
		let mut said = None;
		if let Some((key, rx)) = &self.loading {
			if let Ok(result) = rx.try_recv() {
				let key = key.clone();
				self.loading = None;
				match result {
					Ok(pair) => self.set(key, Some(pair)),
					Err(e) => {
						let e = format!("wallpaper {}: {e}", key.0);
						self.error = Some(e.clone());
						said = Some(e);
						self.set(key, None);
					}
				}
			}
		}
		let want = (name.to_string(), blur);
		if name.is_empty() {
			if self.shown() || self.loading.is_some() || !self.loaded.0.is_empty() {
				self.loading = None;
				self.error = None;
				self.set((String::new(), 0), None);
			}
			return said;
		}
		if want == self.loaded || self.loading.as_ref().is_some_and(|l| l.0 == want) {
			return said;
		}
		// (one at a time: a blur dragged through its values is made for the value it stops at)
		if self.loading.is_some() {
			return said;
		}
		let (tx, rx) = mpsc::channel();
		match self.image.clone().filter(|_| self.loaded.0 == name) {
			Some(img) => {
				std::thread::spawn(move || {
					let _ = tx.send(Ok((ColorImage::clone(&img), frost(img.as_raw(), img.size[0], img.size[1], blur))));
				});
			}
			None => {
				self.error = None;
				let path = dir.join(name);
				std::thread::spawn(move || {
					let _ = tx.send(load(&path, blur));
				});
			}
		}
		self.loading = Some((want, rx));
		said
	}

	fn set(&mut self, key: Key, pair: Option<Pair>) {
		self.loaded = key;
		self.textures = [None, None];
		match pair {
			Some((img, frosted)) => {
				self.image = Some(Arc::new(img));
				self.frosted = Some(Arc::new(frosted));
			}
			None => {
				self.image = None;
				self.frosted = None;
			}
		}
	}

	fn textures(&mut self, ctx: &egui::Context, view: usize) -> Option<&(TextureHandle, TextureHandle)> {
		let (img, frosted) = (self.image.as_ref()?, self.frosted.as_ref()?);
		if self.textures[view].is_none() {
			let t = ctx.load_texture("wallpaper", img.clone(), TextureOptions::LINEAR);
			let f = ctx.load_texture("wallpaper-glass", frosted.clone(), TextureOptions::LINEAR);
			self.textures[view] = Some((t, f));
		}
		self.textures[view].as_ref()
	}

	/// The part of the picture under `r` when it covers the panel (cut to fill it, centred), as texture coordinates
	fn uv(&self, r: Rect) -> Rect {
		let Some(img) = &self.image else { return Rect::ZERO };
		cover_uv([img.size[0] as f32, img.size[1] as f32], self.screen, r)
	}

	/// The picture across `screen`, as much of its light as is left around the boxes. `dark` is the glass' darkness
	/// for this frame's panes
	pub fn draw(&mut self, painter: &Painter, screen: Rect, view: usize, dark: f64) {
		self.screen = screen;
		self.light = 1.0 - (dark.clamp(DARK.0, DARK.1) as f32) / 100.0;
		let uv = self.uv(screen);
		let Some((tex, _)) = self.textures(painter.ctx(), view) else { return };
		painter.image(tex.id(), screen, uv, Color32::from_gray((255.0 * AROUND) as u8));
	}

	/// What is seen through rather than read on (the room): the picture as it is, sharp, darkened as far as a pane
	/// darkens its own
	pub fn window(&self, painter: &Painter, r: Rect) {
		let a = 1.0 - (self.light * 0.9 / AROUND).min(1.0);
		painter.rect_filled(r, 0.0, Color32::from_black_alpha((255.0 * a) as u8));
	}

	/// A pane of glass over the picture: the copy for under the panes, darkened, and with `edge` its rim, its light
	/// and its shadow (a bar across the panel has none: `rounding` 0 and a line where it ends is the caller's)
	pub fn glass(&mut self, painter: &Painter, r: Rect, rounding: f32, view: usize, edge: bool) {
		let cr = CornerRadius::same(rounding.round() as u8);
		if edge {
			// the shadow it throws on what is behind it
			let shadow = Shadow { offset: [0, 4], blur: 16, spread: 0, color: Color32::from_black_alpha(120) };
			painter.add(shadow.as_shape(r, cr));
		}
		// the picture inside a pane a little larger than around it
		let c = r.center();
		let lens = if edge { LENS } else { 1.0 };
		let inner = Rect::from_center_size(c, r.size() / lens);
		let uv = self.uv(inner);
		let light = Color32::from_gray((255.0 * self.light) as u8);
		if let Some((_, frosted)) = self.textures(painter.ctx(), view) {
			painter.add(RectShape::filled(r, cr, light).with_texture(frosted.id(), uv));
		}
		if edge && r.width() > 6.0 * BEVEL && r.height() > 6.0 * BEVEL {
			self.bevel(painter, r, rounding, view);
		}
		// the glass' own body: a pane over a part of the picture that is black is still a pane
		painter.rect_filled(r, cr, white(7));
		if !edge {
			return;
		}
		// light falling on it from above: a sheen that is gone a little way down
		let h = 46.0_f32.min(r.height() * 0.5);
		let sheen = Rect::from_min_max(
			Pos2::new(r.min.x + rounding, r.min.y + 1.0),
			Pos2::new(r.max.x - rounding, r.min.y + h),
		);
		let mut mesh = Mesh::default();
		for (at, col) in [
			(sheen.left_top(), white(13)),
			(sheen.right_top(), white(13)),
			(sheen.left_bottom(), white(0)),
			(sheen.right_bottom(), white(0)),
		] {
			mesh.vertices.push(Vertex { pos: at, uv: egui::epaint::WHITE_UV, color: col });
		}
		mesh.indices.extend([0, 1, 2, 2, 1, 3]);
		painter.add(mesh);
		// its rim, the light along its top edge and the dark at its foot
		painter.rect_stroke(r, cr, Stroke::new(1.0, white(40)), StrokeKind::Inside);
		painter.line_segment(
			[Pos2::new(r.min.x + rounding, r.min.y + 0.5), Pos2::new(r.max.x - rounding, r.min.y + 0.5)],
			Stroke::new(1.0, white(92)),
		);
		painter.line_segment(
			[Pos2::new(r.min.x + rounding, r.max.y - 0.5), Pos2::new(r.max.x - rounding, r.max.y - 0.5)],
			Stroke::new(1.0, Color32::from_black_alpha(110)),
		);
	}

	/// A pane's cut edge: along each side a narrow band in which what is a little further in is seen again, turned
	/// over and brighter, as the slanted edge of thick glass shows it
	fn bevel(&mut self, painter: &Painter, r: Rect, rounding: f32, view: usize) {
		let Some(img) = &self.image else { return };
		let size = [img.size[0] as f32, img.size[1] as f32];
		let screen = self.screen;
		let at = |p: Pos2| cover_uv(size, screen, Rect::from_min_max(p, p)).min;
		let light = Color32::from_gray((255.0 * (self.light * 1.55).min(1.0)) as u8);
		let Some((_, frosted)) = self.textures(painter.ctx(), view) else { return };
		let (b, far) = (BEVEL, BEVEL * 3.5);
		// each band: its outer edge's two ends, and the way in
		let bands = [
			(Pos2::new(r.min.x + rounding, r.min.y), Pos2::new(r.max.x - rounding, r.min.y), Vec2::new(0.0, 1.0)),
			(Pos2::new(r.min.x + rounding, r.max.y), Pos2::new(r.max.x - rounding, r.max.y), Vec2::new(0.0, -1.0)),
			(Pos2::new(r.min.x, r.min.y + rounding), Pos2::new(r.min.x, r.max.y - rounding), Vec2::new(1.0, 0.0)),
			(Pos2::new(r.max.x, r.min.y + rounding), Pos2::new(r.max.x, r.max.y - rounding), Vec2::new(-1.0, 0.0)),
		];
		let mut mesh = Mesh::with_texture(frosted.id());
		for (p0, p1, inward) in bands {
			let i = mesh.vertices.len() as u32;
			for (pos, from) in [
				(p0, p0 + inward * far),
				(p1, p1 + inward * far),
				(p0 + inward * b, p0 + inward * b),
				(p1 + inward * b, p1 + inward * b),
			] {
				mesh.vertices.push(Vertex { pos, uv: at(from), color: light });
			}
			mesh.indices.extend([i, i + 1, i + 2, i + 2, i + 1, i + 3]);
		}
		painter.add(mesh);
	}

	/// The small copy of a picture in `dir` and its size, once it is read (asked for on the first call)
	pub fn thumb(
		&mut self, ctx: &egui::Context, view: usize, dir: &Path, name: &str,
	) -> Option<(egui::TextureId, [usize; 2])> {
		if let Some(rx) = &self.thumb_rx {
			while let Ok((n, img)) = rx.try_recv() {
				self.thumbs.insert(n, img.map(Arc::new));
			}
		}
		if !self.thumbs.contains_key(name) {
			// one at a time: the next is asked for when this one is in
			if self.thumbs.values().any(Option::is_none) && self.thumb_rx.is_some() {
				return None;
			}
			self.thumbs.insert(name.to_string(), None);
			let (tx, rx) = mpsc::channel();
			let (path, n) = (dir.join(name), name.to_string());
			std::thread::spawn(move || {
				let _ = tx.send((n, thumbnail(&path)));
			});
			self.thumb_rx = Some(rx);
			return None;
		}
		let img = self.thumbs.get(name)?.as_ref()?;
		let size = img.size;
		let tex = self.thumb_tex[view].entry(name.to_string()).or_insert_with(|| {
			ctx.load_texture(format!("wallpaper-small-{name}"), img.clone(), TextureOptions::LINEAR)
		});
		Some((tex.id(), size))
	}

	/// The folder's files changed: small copies of files that are gone are let go, and one written over is read
	/// again
	pub fn keep_thumbs(&mut self, files: &[String]) {
		self.thumbs.retain(|k, _| files.contains(k));
		for t in &mut self.thumb_tex {
			t.retain(|k, _| files.contains(k));
		}
	}

	/// A picture's small copy is made again (the file was written over)
	pub fn forget(&mut self, name: &str) {
		self.thumbs.remove(name);
		for t in &mut self.thumb_tex {
			t.remove(name);
		}
		if self.loaded.0 == name {
			self.loaded = (String::new(), 0);
		}
	}
}

/// The part of a picture of `size` under `r` when the picture covers `screen` (cut to fill it, centred), as texture
/// coordinates
pub fn cover_uv(size: [f32; 2], screen: Rect, r: Rect) -> Rect {
	let sc = (screen.width() / size[0]).max(screen.height() / size[1]);
	let (dw, dh) = (size[0] * sc, size[1] * sc);
	let o = Vec2::new((dw - screen.width()) * 0.5, (dh - screen.height()) * 0.5);
	let at = |p: Pos2| Pos2::new((p.x - screen.min.x + o.x) / dw, (p.y - screen.min.y + o.y) / dh);
	Rect::from_min_max(at(r.min), at(r.max))
}

/// A picture halved while over `most` pixels a side
fn halved(mut img: image::RgbaImage, most: u32) -> image::RgbaImage {
	while img.width() > most || img.height() > most {
		let (w2, h2) = (img.width() / 2, img.height() / 2);
		let mut q = image::RgbaImage::new(w2, h2);
		for y in 0..h2 {
			for x in 0..w2 {
				let mut s = [0u32; 4];
				for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
					let p = img.get_pixel(2 * x + dx, 2 * y + dy).0;
					for c in 0..4 {
						s[c] += p[c] as u32;
					}
				}
				q.put_pixel(x, y, image::Rgba(s.map(|v| (v / 4) as u8)));
			}
		}
		img = q;
	}
	img
}

/// A picture's small copy (at most 320 pixels a side)
fn thumbnail(path: &Path) -> Option<ColorImage> {
	let img = image::open(path).ok()?.thumbnail(320, 320).to_rgba8();
	Some(ColorImage::from_rgba_unmultiplied([img.width() as usize, img.height() as usize], img.as_raw()))
}

/// The picture (halved while over 3000 pixels: the GPU's and the overlay's sake; what of it was see-through is
/// black) and its copy for under the panes
fn load(path: &Path, blur: u8) -> Result<Pair, String> {
	let img = halved(image::open(path).map_err(|e| e.to_string())?.to_rgba8(), 3000);
	let (w, h) = (img.width() as usize, img.height() as usize);
	let mut px = img.into_raw();
	for q in px.chunks_exact_mut(4) {
		let a = q[3] as u32;
		if a < 255 {
			for c in &mut q[..3] {
				*c = (*c as u32 * a / 255) as u8;
			}
			q[3] = 255;
		}
	}
	let frosted = frost(&px, w, h, blur);
	Ok((ColorImage::from_rgba_unmultiplied([w, h], &px), frosted))
}

/// One pass of a box blur along a line of `n` pixels `stride` apart, each the mean of the `2 r + 1` around it (the
/// line's ends repeated past them): a running sum, so a wide blur costs what a narrow one does
fn box_line(src: &[u8], dst: &mut [u8], start: usize, stride: usize, n: usize, r: usize) {
	let at = |i: isize| start + (i.clamp(0, n as isize - 1) as usize) * stride;
	let width = (2 * r + 1) as u32;
	for c in 0..3 {
		let mut sum: u32 = (-(r as isize)..=r as isize).map(|i| src[at(i) + c] as u32).sum();
		for i in 0..n as isize {
			dst[at(i) + c] = ((sum + width / 2) / width) as u8;
			sum += src[at(i + r as isize + 1) + c] as u32;
			sum -= src[at(i - r as isize) + c] as u32;
		}
	}
}

/// The box blur's radius, three passes of which come to a Gaussian of `sigma`
fn box_radius(sigma: f32) -> usize {
	(((4.0 * sigma * sigma + 1.0).sqrt() - 1.0) * 0.5).round().max(0.0) as usize
}

/// The copy of an RGBA picture for under the panes: at half its size where it is large, blurred by `blur` (about
/// that many points across a window 1600 wide; three passes of a box blur across and down), its colours a little
/// stronger and what is bright in it brought down. Drawn back up with linear filtering
fn frost(px: &[u8], w: usize, h: usize, blur: u8) -> ColorImage {
	let f = if w.max(h) > 1400 { 2usize } else { 1 };
	let (bw, bh) = ((w / f).max(1), (h / f).max(1));
	let mut a = vec![255u8; bw * bh * 4];
	for y in 0..bh {
		for x in 0..bw {
			for c in 0..3 {
				let (mut sum, mut n) = (0u32, 0u32);
				for dy in 0..f {
					for dx in 0..f {
						let (sx, sy) = (x * f + dx, y * f + dy);
						if sx < w && sy < h {
							sum += px[(sy * w + sx) * 4 + c] as u32;
							n += 1;
						}
					}
				}
				a[(y * bw + x) * 4 + c] = (sum / n.max(1)) as u8;
			}
		}
	}
	// the blur in this copy's pixels: a picture covers a window about 1600 points wide
	let r = box_radius(blur as f32 * (w as f32 / 1600.0).max(0.5) / f as f32);
	if blur > 0 {
		let r = r.max(1);
		let mut b = a.clone();
		for _ in 0..3 {
			for y in 0..bh {
				box_line(&a, &mut b, y * bw * 4, 4, bw, r);
			}
			for x in 0..bw {
				box_line(&b, &mut a, x * 4, bw * 4, bh, r);
			}
		}
	}
	for q in a.chunks_exact_mut(4) {
		tone(q);
	}
	ColorImage::from_rgba_unmultiplied([bw, bh], &a)
}

/// A pixel of the copy for under the panes: its colour stronger, and what is bright in it brought down, the colour
/// kept
fn tone(q: &mut [u8]) {
	let mut v = [q[0] as f32 / 255.0, q[1] as f32 / 255.0, q[2] as f32 / 255.0];
	let grey = 0.2126 * v[0] + 0.7152 * v[1] + 0.0722 * v[2];
	for c in &mut v {
		*c = (grey + (*c - grey) * SATURATION).clamp(0.0, 1.0);
	}
	let hi = v[0].max(v[1]).max(v[2]);
	let k = if hi > KNEE { (KNEE + (hi - KNEE) * SLOPE) / hi } else { 1.0 };
	for (c, out) in v.iter().zip(q.iter_mut()) {
		*out = (c * k * 255.0).round() as u8;
	}
}

/// The brightest a pane's ground can be (0 to 255, as drawn) at a darkness: what a text on it has to stand against
/// over the brightest picture
pub fn brightest_ground(dark: f64) -> f32 {
	let light = 1.0 - (dark.clamp(DARK.0, DARK.1) as f32) / 100.0;
	let under = (KNEE + (1.0 - KNEE) * SLOPE) * light * 255.0;
	// the glass' own body over it
	under + (255.0 - under) * 7.0 / 255.0
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_copy_under_the_panes_is_blurred_a_little_and_brought_down() {
		// a white line on black, 400 wide: with no blur it stays a line, with the blur it spreads, a little
		let (w, h) = (400usize, 40usize);
		let mut px = vec![0u8; w * h * 4];
		for y in 0..h {
			for c in 0..4 {
				px[(y * w + 200) * 4 + c] = 255;
			}
		}
		let value = |img: &ColorImage, x: usize| img.pixels[20 * w + x].r() as f32;
		let sharp = frost(&px, w, h, 0);
		assert_eq!(sharp.size, [w, h]);
		assert_eq!(value(&sharp, 190), 0.0);
		// white is brought down: 0.64 of it
		assert!((value(&sharp, 200) - 0.64 * 255.0).abs() < 3.0, "{}", value(&sharp, 200));
		let soft = frost(&px, w, h, 3);
		assert!(value(&soft, 200) < value(&sharp, 200) && value(&soft, 200) > 0.0);
		assert!(value(&soft, 198) > 0.0, "the line spreads");
		assert_eq!(value(&soft, 185), 0.0, "and not far: what is in the picture can still be made out");
		// a colour stays that colour, stronger
		let mut q = [200u8, 100, 100, 255];
		tone(&mut q);
		assert!(q[0] > q[1] && q[1] == q[2] && q[0] as f32 <= 0.64 * 255.0 + 1.0, "{q:?}");
	}

	#[test]
	fn a_text_stands_against_the_brightest_picture_at_the_darkness_the_panel_starts_with() {
		// the lightest ground a text stands on is the header band over a pane; WCAG's contrast of the inks on it
		let lum = |v: f32| {
			let v = v / 255.0;
			if v <= 0.04045 {
				v / 12.92
			} else {
				((v + 0.055) / 1.055).powf(2.4)
			}
		};
		let t = crate::ui::theme::Tk::glass();
		let pane = brightest_ground(DARK.2);
		let band = pane + (255.0 - pane) * t.head.a() as f32 / 255.0;
		let ratio = |ink: Color32, ground: f32| (lum(ink.r() as f32) + 0.05) / (lum(ground) + 0.05);
		assert!(ratio(t.text, band) >= 7.0, "{}", ratio(t.text, band));
		assert!(ratio(t.soft, band) >= 4.5, "{}", ratio(t.soft, band));
		assert!(ratio(t.muted, band) >= 4.5, "{}", ratio(t.muted, band));
		assert!(ratio(t.muted, pane) >= 4.5);
	}

	#[test]
	fn a_picture_covers_the_panel_and_a_box_shows_its_own_part() {
		let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(1600.0, 900.0));
		// a picture wider than the panel: its sides are cut, its height is all there
		let uv = cover_uv([3200.0, 900.0], screen, screen);
		assert!((uv.min.x - 0.25).abs() < 1e-6 && (uv.max.x - 0.75).abs() < 1e-6);
		assert!((uv.min.y, uv.max.y) == (0.0, 1.0));
		// and a box in the panel's middle is over the picture's middle
		let r = Rect::from_center_size(screen.center(), Vec2::new(160.0, 90.0));
		let uv = cover_uv([3200.0, 900.0], screen, r);
		assert!((uv.center().x - 0.5).abs() < 1e-6 && (uv.center().y - 0.5).abs() < 1e-6);
	}

	#[test]
	fn a_dropped_picture_goes_into_the_folder_and_nothing_else_does() {
		let dir = std::env::temp_dir().join(format!("sally-wallpaper-test-{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(&dir).unwrap();
		let from = dir.join("Dusk.PNG");
		std::fs::write(&from, b"not really a picture").unwrap();
		let folder = dir.join("wallpapers");
		assert_eq!(bring(&folder, &from).as_deref(), Ok("Dusk.PNG"));
		assert_eq!(files(&folder), ["Dusk.PNG"]);
		let other = dir.join("notes.txt");
		std::fs::write(&other, b"x").unwrap();
		assert!(bring(&folder, &other).is_err());
		assert_eq!(files(&folder), ["Dusk.PNG"]);
		std::fs::remove_dir_all(&dir).unwrap();
	}
}
