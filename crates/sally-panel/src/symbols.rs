//! The devices' symbols as SteamVR shows them: the status icons in the drivers' resources (drivers\<driver>\resources\
//! icons), which each driver's driver.vrresources names per device class and model. Read from the SteamVR install on
//! this PC, as the render models are (models.rs): nothing of them ships with the panel, and where there is no SteamVR
//! (or no icon for a device) the panel draws an icon of its own.
//!
//! A device's set is named by its config: `resource_root` is the driver whose resources it uses ("knuckles" and
//! "utah" are the Index controllers' and headset's, in drivers\indexcontroller and drivers\indexhmd), and in that
//! driver's `statusicons` it is the entry of its `model_number`, else of its class ("HMD", "LeftController",
//! "RightController", "Controller", "Tracker", "TrackingReference"), an "Alias" followed. What is not found there is
//! looked for in the lighthouse driver's, as SteamVR's own default.
//!
//! The symbol is the ready icon's shape as a mask (its alpha, times how bright each pixel is between the icon's
//! darkest ink and its brightest, so a drawing's lines are cut out of its shape): the panel colours it itself. The
//! larger `_2x` file is taken where there is one. Where only the off icon has one, that, if it is the ready icon's
//! shape: one flat shape (a Vive tracker's, grey), or a drawing with the ready icon's outline (a Tundra tracker's
//! ready icon is 32 pixels and nothing larger; its off icon is the same drawing at 64, dark with a light edge).

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;

use egui::{ColorImage, TextureHandle, TextureId, TextureOptions};

use crate::models::json_string;

/// A device's class, as driver.vrresources keys its icon sets
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Class {
	Hmd,
	LeftController,
	RightController,
	Controller,
	Tracker,
	Station,
}

impl Class {
	/// The statusicons keys that hold this class's set, the first that is there
	fn keys(self) -> &'static [&'static str] {
		match self {
			Class::Hmd => &["HMD"],
			Class::LeftController => &["LeftController", "Controller"],
			Class::RightController => &["RightController", "Controller"],
			Class::Controller => &["Controller", "LeftController"],
			Class::Tracker => &["Tracker", "GenericTracker"],
			Class::Station => &["TrackingReference"],
		}
	}
}

/// What picks a device's symbol
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Kind {
	/// the config's resource_root
	pub root: String,
	/// the config's model_number
	pub model: String,
	pub class: Class,
}

impl Kind {
	/// From a device's config (the JSON SteamVR keeps of it), with the runtime's word for what it is where the config
	/// does not say which hand
	pub fn of_config(text: &str, subtype: &str) -> Kind {
		let model = json_string(text, "model_number");
		let class = match json_string(text, "device_class").as_str() {
			"hmd" => Class::Hmd,
			"generic_tracker" => Class::Tracker,
			"controller" => {
				let m = model.to_ascii_lowercase();
				if subtype == "knuckles_l" || m.contains("left") {
					Class::LeftController
				} else if subtype == "knuckles_r" || m.contains("right") {
					Class::RightController
				} else {
					Class::Controller
				}
			}
			_ => Kind::of_subtype(subtype).class,
		};
		Kind { root: json_string(text, "resource_root"), model, class }
	}

	/// Without a config: by the runtime's word for what it is
	pub fn of_subtype(subtype: &str) -> Kind {
		let (root, class) = match subtype {
			"knuckles_l" => ("knuckles", Class::LeftController),
			"knuckles_r" => ("knuckles", Class::RightController),
			"wand" => ("htc", Class::Controller),
			"index_hmd" => ("utah", Class::Hmd),
			"vive_hmd" => ("htc", Class::Hmd),
			_ => ("htc", Class::Tracker),
		};
		Kind { root: root.into(), model: String::new(), class }
	}

	/// A base station (the 2.0 ones: sally's)
	pub fn station() -> Kind {
		Kind { root: "lighthouse".into(), model: "Valve SR".into(), class: Class::Station }
	}
}

/// The driver folder a resource root's files are in, where it is not the root's own name
fn driver_of(root: &str) -> &str {
	match root {
		"knuckles" => "indexcontroller",
		"utah" => "indexhmd",
		r => r,
	}
}

// ---- driver.vrresources ----

/// As much of JSON as a driver.vrresources is made of: objects and strings (anything else is skipped)
#[derive(Clone, Debug, PartialEq)]
enum Json {
	Obj(Vec<(String, Json)>),
	Str(String),
	Other,
}

impl Json {
	fn get(&self, key: &str) -> Option<&Json> {
		match self {
			Json::Obj(v) => v.iter().find(|(k, _)| k == key).map(|(_, j)| j),
			_ => None,
		}
	}

	fn str(&self, key: &str) -> Option<&str> {
		match self.get(key) {
			Some(Json::Str(s)) => Some(s),
			_ => None,
		}
	}
}

struct Reader<'a> {
	s: &'a [u8],
	i: usize,
	depth: u32,
}

impl Reader<'_> {
	fn ws(&mut self) {
		while self.i < self.s.len() && self.s[self.i].is_ascii_whitespace() {
			self.i += 1;
		}
	}

	fn string(&mut self) -> Option<String> {
		// at the opening quote
		self.i += 1;
		let mut out: Vec<u8> = Vec::new();
		while self.i < self.s.len() {
			let c = self.s[self.i];
			self.i += 1;
			match c {
				b'"' => return Some(String::from_utf8_lossy(&out).into_owned()),
				b'\\' => {
					let e = *self.s.get(self.i)?;
					self.i += 1;
					match e {
						b'n' => out.push(b'\n'),
						b't' => out.push(b'\t'),
						b'u' => {
							let hex = std::str::from_utf8(self.s.get(self.i..self.i + 4)?).ok()?;
							let ch = char::from_u32(u32::from_str_radix(hex, 16).ok()?).unwrap_or('?');
							out.extend(ch.to_string().as_bytes());
							self.i += 4;
						}
						other => out.push(other),
					}
				}
				other => out.push(other),
			}
		}
		None
	}

	fn value(&mut self) -> Option<Json> {
		self.ws();
		match *self.s.get(self.i)? {
			b'{' => {
				self.depth += 1;
				if self.depth > 32 {
					return None;
				}
				self.i += 1;
				let mut members = Vec::new();
				loop {
					self.ws();
					match *self.s.get(self.i)? {
						b'}' => {
							self.i += 1;
							break;
						}
						b',' => self.i += 1,
						b'"' => {
							let key = self.string()?;
							self.ws();
							if *self.s.get(self.i)? != b':' {
								return None;
							}
							self.i += 1;
							members.push((key, self.value()?));
						}
						_ => return None,
					}
				}
				self.depth -= 1;
				Some(Json::Obj(members))
			}
			b'[' => {
				self.depth += 1;
				if self.depth > 32 {
					return None;
				}
				self.i += 1;
				loop {
					self.ws();
					match *self.s.get(self.i)? {
						b']' => {
							self.i += 1;
							break;
						}
						b',' => self.i += 1,
						_ => {
							self.value()?;
						}
					}
				}
				self.depth -= 1;
				Some(Json::Other)
			}
			b'"' => self.string().map(Json::Str),
			_ => {
				// a number, true, false, null: to the next delimiter
				let from = self.i;
				while self.i < self.s.len()
					&& !matches!(self.s[self.i], b',' | b'}' | b']')
					&& !self.s[self.i].is_ascii_whitespace()
				{
					self.i += 1;
				}
				(self.i > from).then_some(Json::Other)
			}
		}
	}
}

fn parse_json(text: &str) -> Option<Json> {
	Reader { s: text.as_bytes(), i: 0, depth: 0 }.value()
}

/// The ready and the off icon a driver.vrresources names for a kind ("{htc}/icons/tracker_status_ready.png"): its
/// model's entry, else its class's, an "Alias" followed
pub fn icons_named(vrresources: &str, kind: &Kind) -> Option<(String, String)> {
	let json = parse_json(vrresources)?;
	let sets = json.get("statusicons")?;
	let mut keys: Vec<&str> = Vec::new();
	if !kind.model.is_empty() {
		keys.push(&kind.model);
	}
	keys.extend(kind.class.keys());
	for key in keys {
		let mut set = sets.get(key);
		// an alias names another set of the same file
		for _ in 0..4 {
			match set.and_then(|s| s.str("Alias")) {
				Some(a) => set = sets.get(a),
				None => break,
			}
		}
		let Some(set) = set else { continue };
		let ready = set.str("Prop_NamedIconPathDeviceReady_String").unwrap_or("");
		let off = set.str("Prop_NamedIconPathDeviceOff_String").unwrap_or("");
		if !ready.is_empty() || !off.is_empty() {
			return Some((ready.to_string(), off.to_string()));
		}
	}
	None
}

/// The driver's folder of that name among `drivers` (each a driver's own folder, its last name the driver's)
fn driver_dir<'a>(drivers: &'a [PathBuf], name: &str) -> Option<&'a PathBuf> {
	drivers.iter().find(|d| d.file_name().and_then(|f| f.to_str()).is_some_and(|f| f.eq_ignore_ascii_case(name)))
}

/// An icon's name ("{htc}/icons/tracker_status_ready.png") as a file under the drivers' folders, and its `_2x` file's
/// path (there or not)
fn icon_file(drivers: &[PathBuf], named: &str) -> Option<(PathBuf, PathBuf)> {
	let (driver, rel) = named.strip_prefix('{')?.split_once('}')?;
	let rel = rel.trim_start_matches(['/', '\\']);
	if rel.is_empty() || rel.contains("..") || rel.contains(':') {
		return None;
	}
	let dir = driver_dir(drivers, driver)?.join("resources");
	let file = rel.split(['/', '\\']).fold(dir, |p, part| p.join(part));
	let stem = file.file_stem()?.to_str()?;
	let ext = file.extension()?.to_str()?;
	let big = file.with_file_name(format!("{stem}_2x.{ext}"));
	Some((file, big))
}

/// The files a kind's symbol can be read from, the best first, each with whether it is the off icon's (taken only
/// where it is the ready icon's shape: `load_symbol`)
pub fn symbol_files(drivers: &[PathBuf], kind: &Kind) -> Vec<(PathBuf, bool)> {
	let named = [driver_of(&kind.root), "lighthouse"].iter().find_map(|name| {
		let text = std::fs::read(driver_dir(drivers, name)?.join("resources").join("driver.vrresources")).ok()?;
		icons_named(&String::from_utf8_lossy(&text), kind)
	});
	let Some((ready, off)) = named else { return Vec::new() };
	let (ready, off) = (icon_file(drivers, &ready), icon_file(drivers, &off));
	let mut out = Vec::new();
	if let Some((_, big)) = &ready {
		out.push((big.clone(), false));
	}
	if let Some((_, big)) = &off {
		out.push((big.clone(), true));
	}
	if let Some((file, _)) = ready {
		out.push((file, false));
	}
	if let Some((file, _)) = off {
		out.push((file, true));
	}
	out.retain(|(p, _)| {
		p.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("png")) && p.is_file()
	});
	out
}

/// How bright a pixel is: its strongest channel
fn value(p: &image::Rgba<u8>) -> u8 {
	p.0[0].max(p.0[1]).max(p.0[2])
}

/// A pixel that is there altogether (what the icon's own brightness is read from), and one that is there at all
const SOLID: u8 = 200;
const DRAWN: u8 = 8;

/// The icon is one flat shape: its solid pixels all about as bright as its brightest
pub fn is_flat(img: &image::RgbaImage) -> bool {
	let solid = || img.pixels().filter(|p| p.0[3] > SOLID).map(value);
	let (hi, lo) = (solid().max().unwrap_or(0), solid().min().unwrap_or(0));
	hi > 0 && lo as f32 >= 0.8 * hi as f32
}

/// An icon's pixels with the light edge around a dark icon taken out (their alpha 0). An off icon has one, to be seen
/// on a dark window: its soft pixels (drawn, not solid) are then half again as bright as any solid one, half of
/// them or more (a Tundra tracker's: 84 of 88; a few such pixels among an icon's own soft edge are specks, left as
/// they are). It is not the device's shape, and what it touches it has lightened: those pixels go with it, the
/// ones beside its faintest pixels too. A picture with hardly a solid pixel has no such edge
fn without_light_edge(img: &image::RgbaImage) -> image::RgbaImage {
	let mut out = img.clone();
	let drawn = img.pixels().filter(|p| p.0[3] > DRAWN).count();
	let solid = || img.pixels().filter(|p| p.0[3] > SOLID).map(value);
	let Some(hi) = solid().max().filter(|_| solid().count() * 4 >= drawn) else { return out };
	let soft = |p: &image::Rgba<u8>| p.0[3] > DRAWN && p.0[3] <= SOLID;
	let light = |p: &image::Rgba<u8>| p.0[3] > 0 && p.0[3] <= SOLID && value(p) as f32 > 1.5 * hi as f32;
	let lights = img.pixels().filter(|p| soft(p) && light(p)).count();
	if lights < 16 || lights * 2 < img.pixels().filter(|p| soft(p)).count() {
		return out;
	}
	let (w, h) = (img.width() as i64, img.height() as i64);
	for (x, y, p) in img.enumerate_pixels() {
		if !light(p) {
			continue;
		}
		for (dx, dy) in (-1..=1).flat_map(|dy| (-1..=1).map(move |dx| (dx, dy))) {
			let (nx, ny) = (x as i64 + dx, y as i64 + dy);
			if nx >= 0 && ny >= 0 && nx < w && ny < h {
				out.get_pixel_mut(nx as u32, ny as u32).0[3] = 0;
			}
		}
	}
	out
}

/// How an icon's pixels are read into a mask
#[derive(Clone, Copy, Debug, PartialEq)]
struct Levels {
	/// what is white in the mask: the brightest of the icon's solid pixels (a dark icon, an off one, is its shape as
	/// much as a bright one); of all its pixels where hardly any is solid
	hi: f32,
	/// what is left out of it: the darkest of the solid pixels where that is a second ink (under half of `hi`: the
	/// lines of a drawing), 0 where it is a shade of the one ink, which stays as it is
	lo: f32,
}

impl Levels {
	fn of(img: &image::RgbaImage) -> Option<Levels> {
		let drawn = img.pixels().filter(|p| p.0[3] > DRAWN).count();
		let solid = || img.pixels().filter(|p| p.0[3] > SOLID).map(value);
		// a picture of thin or see-through strokes has hardly a solid pixel to go by: its brightest pixel, as it is
		if solid().count() * 4 < drawn.max(1) {
			let hi = img.pixels().filter(|p| p.0[3] > 0).map(value).max().filter(|v| *v > 0)? as f32;
			return Some(Levels { hi, lo: 0.0 });
		}
		let hi = solid().max().filter(|v| *v > 0)? as f32;
		let lo = solid().min().map_or(0.0, |v| v as f32);
		Some(Levels { hi, lo: if lo < 0.5 * hi { lo } else { 0.0 } })
	}

	/// How much of the mask a pixel is, 0 to 255
	fn share(&self, p: &image::Rgba<u8>) -> u8 {
		let ink = ((value(p) as f32 - self.lo) / (self.hi - self.lo)).clamp(0.0, 1.0);
		(p.0[3] as f32 * ink).round() as u8
	}
}

/// The box of a picture's pixels (row by row, `w` wide) over DRAWN, and those pixels cut to it; None for none
fn cut(w: usize, px: &[u8]) -> Option<(usize, usize, Vec<u8>)> {
	let (mut x0, mut y0, mut x1, mut y1) = (usize::MAX, usize::MAX, 0usize, 0usize);
	for (i, a) in px.iter().enumerate() {
		if *a > DRAWN {
			let (x, y) = (i % w, i / w);
			(x0, y0, x1, y1) = (x0.min(x), y0.min(y), x1.max(x), y1.max(y));
		}
	}
	if x0 > x1 || y0 > y1 {
		return None;
	}
	let (cw, ch) = (x1 - x0 + 1, y1 - y0 + 1);
	let mut out = Vec::with_capacity(cw * ch);
	for y in y0..=y1 {
		out.extend_from_slice(&px[y * w + x0..=y * w + x1]);
	}
	Some((cw, ch, out))
}

/// An icon as a mask, cut to what it leaves of it: white, each pixel's alpha the icon's own times how bright it is
/// between the icon's darkest ink and its brightest (`Levels`): a flat icon is its shape, a drawing its shape with
/// its lines cut out; a light edge around a dark icon is not in it. None for an empty picture
pub fn mask(img: &image::RgbaImage) -> Option<ColorImage> {
	let img = without_light_edge(img);
	let lv = Levels::of(&img)?;
	let shares: Vec<u8> = img.pixels().map(|p| lv.share(p)).collect();
	let (w, h, shares) = cut(img.width() as usize, &shares)?;
	let px: Vec<u8> = shares.iter().flat_map(|a| [255, 255, 255, *a]).collect();
	Some(ColorImage::from_rgba_unmultiplied([w, h], &px))
}

/// What an icon covers, whatever its colours: its alpha (without a light edge around it), cut to its box
fn cover(img: &image::RgbaImage) -> Option<(usize, usize, Vec<u8>)> {
	let alpha: Vec<u8> = without_light_edge(img).pixels().map(|p| p.0[3]).collect();
	cut(img.width() as usize, &alpha)
}

/// Whether two icons are one shape, at whichever sizes: what each covers, in its own box and the larger brought to
/// the smaller's size, is the same but for a twentieth, the boxes as wide for their height within a tenth. So an
/// icon's off picture is told from another drawing under the same name (its outline alone, one with a badge on it)
pub fn same_outline(a: &image::RgbaImage, b: &image::RgbaImage) -> bool {
	let (Some(a), Some(b)) = (cover(a), cover(b)) else { return false };
	let ((w, h, small), (bw, bh, big)) = if a.0 * a.1 <= b.0 * b.1 { (a, b) } else { (b, a) };
	let wide = |w: usize, h: usize| w as f32 / h as f32;
	if (wide(w, h) / wide(bw, bh) - 1.0).abs() > 0.1 {
		return false;
	}
	let (sx, sy) = (bw as f32 / w as f32, bh as f32 / h as f32);
	let (mut both, mut either) = (0u32, 0u32);
	for y in 0..h {
		let (y0, y1) = (y as f32 * sy, (y + 1) as f32 * sy);
		for x in 0..w {
			let (x0, x1) = (x as f32 * sx, (x + 1) as f32 * sx);
			// the larger one's pixels under this one, each by how much of it lies there
			let (mut sum, mut area) = (0.0f32, 0.0f32);
			for by in y0 as usize..(y1.ceil() as usize).min(bh) {
				let part_y = (by as f32 + 1.0).min(y1) - (by as f32).max(y0);
				for bx in x0 as usize..(x1.ceil() as usize).min(bw) {
					let part = part_y * ((bx as f32 + 1.0).min(x1) - (bx as f32).max(x0));
					sum += big[by * bw + bx] as f32 * part;
					area += part;
				}
			}
			let (p, q) = (small[y * w + x] > 127, area > 0.0 && sum / area > 127.0);
			both += (p && q) as u32;
			either += (p || q) as u32;
		}
	}
	either > 0 && both as f32 >= 0.95 * either as f32
}

/// A kind's symbol read from the drivers' folders: the first of its files that is a picture of it
pub fn load_symbol(drivers: &[PathBuf], kind: &Kind) -> Option<ColorImage> {
	let files = symbol_files(drivers, kind);
	let open = |path: &PathBuf| image::open(path).ok().map(|i| i.to_rgba8());
	// the ready icon, whichever size there is of it: what the off icon is held against
	let ready = files.iter().filter(|(_, off)| !off).find_map(|(path, _)| open(path));
	files.iter().find_map(|(path, off)| {
		let img = open(path)?;
		// the off icon's file where it is the ready icon's shape: with its outline; one flat shape where there is no
		// ready icon to hold it against
		let is_it = || match &ready {
			Some(r) => same_outline(&img, r),
			None => is_flat(&img),
		};
		if *off && !is_it() {
			return None;
		}
		mask(&img)
	})
}

/// A symbol to draw: its texture in the view asked for, and its size in pixels
pub type Symbol = (TextureId, [usize; 2]);

/// The symbols read so far
#[derive(Default)]
pub struct Symbols {
	/// sally-panel.cfg's `steamvr` ("" for the one openvrpaths.vrpath names), and its drivers' folders once looked for
	steamvr: String,
	drivers: Option<Vec<PathBuf>>,
	/// by kind: its mask, None where SteamVR has none for it
	cache: HashMap<Kind, Option<Arc<ColorImage>>>,
	/// a device's kind by its serial, and where it is from: its config (None), else guessed from what the runtime
	/// called it then (a device is listed before the runtime says what it is)
	by_serial: HashMap<String, (Kind, Option<String>)>,
	/// each view's textures (an egui context has its own)
	tex: [HashMap<Kind, TextureHandle>; 2],
}

impl Symbols {
	/// Where SteamVR is (sally-panel.cfg's `steamvr`); a change reads everything again
	pub fn set_steamvr(&mut self, dir: &str) {
		if self.drivers.is_none() || self.steamvr != dir {
			self.steamvr = dir.into();
			self.drivers = None;
			self.cache.clear();
			for t in &mut self.tex {
				t.clear();
			}
		}
	}

	/// The devices whose config was not found are looked up again (the panel does this now and then: a device's
	/// config appears when SteamVR first sees it)
	pub fn retry(&mut self) {
		self.by_serial.retain(|_, (_, guessed)| guessed.is_none());
	}

	/// A device's kind: by its config (`<serial>_config.json` or `<serial>\config.json` in one of `dirs`), else by the
	/// runtime's word for what it is
	pub fn kind_of(&mut self, serial: &str, subtype: &str, dirs: &[PathBuf]) -> Kind {
		if let Some((k, guessed)) = self.by_serial.get(serial) {
			if guessed.as_deref().is_none_or(|g| g == subtype) {
				return k.clone();
			}
		}
		let config = dirs.iter().find_map(|d| {
			[d.join(format!("{serial}_config.json")), d.join(serial.to_lowercase()).join("config.json")]
				.iter()
				.find_map(|p| std::fs::read(p).ok())
		});
		let (kind, guessed) = match config {
			Some(b) => (Kind::of_config(&String::from_utf8_lossy(&b), subtype), None),
			None => (Kind::of_subtype(subtype), Some(subtype.to_string())),
		};
		let kind = if kind.root.is_empty() { Kind { root: Kind::of_subtype(subtype).root, ..kind } } else { kind };
		self.by_serial.insert(serial.to_string(), (kind.clone(), guessed));
		kind
	}

	/// A device's model as its config names it ("Tundra Tracker", "VIVE Tracker 3.0 MV"), where its config was read
	pub fn model_of(&self, serial: &str) -> Option<&str> {
		self.by_serial
			.get(serial)
			.filter(|(k, guessed)| guessed.is_none() && !k.model.is_empty())
			.map(|(k, _)| k.model.as_str())
	}

	/// A kind's symbol for the view (0 the window, 1 the overlay), None where SteamVR has none
	pub fn get(&mut self, ctx: &egui::Context, view: usize, kind: &Kind) -> Option<Symbol> {
		if !self.cache.contains_key(kind) {
			let drivers = self.drivers.get_or_insert_with(|| crate::models::driver_dirs(&self.steamvr));
			let img = load_symbol(drivers, kind).map(Arc::new);
			self.cache.insert(kind.clone(), img);
		}
		let img = self.cache.get(kind)?.as_ref()?;
		let size = img.size;
		let tex = self.tex[view].entry(kind.clone()).or_insert_with(|| {
			ctx.load_texture(
				format!("symbol-{}-{}-{:?}", kind.root, kind.model, kind.class),
				img.clone(),
				TextureOptions::LINEAR,
			)
		});
		Some((tex.id(), size))
	}
}

/// The drivers' folders under a SteamVR folder made for a test: `<root>\drivers\<name>`
#[cfg(test)]
fn test_drivers(root: &std::path::Path, names: &[&str]) -> Vec<PathBuf> {
	names.iter().map(|n| root.join("drivers").join(n)).collect()
}

#[cfg(test)]
mod tests {
	use super::*;

	const HTC: &str = r#"{
	"jsonid" : "vrresources",
	"statusicons" : {
		"HMD" : {
			"Prop_NamedIconPathDeviceOff_String" : "{htc}/icons/headset_status_off.png",
			"Prop_NamedIconPathDeviceReady_String" : "{htc}/icons/headset_status_ready.png",
			"Prop_NamedIconPathDeviceAlertLow_String" : ""
		},
		"Controller" : {
			"Prop_NamedIconPathDeviceOff_String" : "{htc}/icons/controller_status_off.png",
			"Prop_NamedIconPathDeviceReady_String" : "{htc}/icons/controller_status_ready.png"
		},
		"LeftController" : { "Alias" : "Controller" },
		"Tracker" : {
			"Prop_NamedIconPathDeviceOff_String" : "{htc}/icons/tracker_status_off.png",
			"Prop_NamedIconPathDeviceReady_String" : "{htc}/icons/tracker_status_ready.png"
		},
		"HTC Vive Pro" : {
			"Prop_NamedIconPathDeviceReady_String" : "{htc}/icons/vivepro_status_ready.png"
		},
		"Vive_Pro" : { "Alias" : "HTC Vive Pro" }
    }
}"#;

	fn kind(root: &str, model: &str, class: Class) -> Kind {
		Kind { root: root.into(), model: model.into(), class }
	}

	#[test]
	fn a_drivers_resources_name_the_icons_by_model_then_class() {
		let named = |k: &Kind| icons_named(HTC, k).map(|(r, _)| r);
		assert_eq!(
			named(&kind("htc", "VIVE Tracker 3.0 MV", Class::Tracker)).as_deref(),
			Some("{htc}/icons/tracker_status_ready.png")
		);
		// a model's own set before its class's, through an alias
		assert_eq!(
			named(&kind("htc", "Vive_Pro", Class::Hmd)).as_deref(),
			Some("{htc}/icons/vivepro_status_ready.png")
		);
		assert_eq!(named(&kind("htc", "", Class::Hmd)).as_deref(), Some("{htc}/icons/headset_status_ready.png"));
		// a left controller through its alias, a right one (not named here) as a controller
		assert_eq!(
			named(&kind("htc", "", Class::LeftController)).as_deref(),
			Some("{htc}/icons/controller_status_ready.png")
		);
		assert_eq!(
			named(&kind("htc", "", Class::RightController)).as_deref(),
			Some("{htc}/icons/controller_status_ready.png")
		);
		assert_eq!(named(&kind("htc", "", Class::Station)), None);
		assert_eq!(icons_named("{ \"statusicons\" : 3 }", &kind("htc", "", Class::Hmd)), None);
		assert_eq!(icons_named("not json", &kind("htc", "", Class::Hmd)), None);
	}

	#[test]
	fn json_is_read_as_far_as_objects_and_strings() {
		let j = parse_json(r#"{ "a" : { "b" : "c\\d", "n" : [1, {"x": true}, "s"], "z" : null, }, "e" : -1.5e3 }"#)
			.unwrap();
		assert_eq!(j.get("a").and_then(|a| a.str("b")), Some("c\\d"));
		assert_eq!(j.get("a").and_then(|a| a.get("n")), Some(&Json::Other));
		assert_eq!(j.get("e"), Some(&Json::Other));
		assert!(parse_json("{ \"a\" : ").is_none());
		assert!(parse_json(&"{\"a\":".repeat(100)).is_none());
	}

	#[test]
	fn a_devices_kind_is_its_configs() {
		let tundra = r#"{ "device_class" : "generic_tracker", "model_number" : "Tundra Tracker",
			"render_model" : "{tundra_labs}tundra_tracker", "resource_root" : "tundra_labs" }"#;
		assert_eq!(Kind::of_config(tundra, "tracker_gen2"), kind("tundra_labs", "Tundra Tracker", Class::Tracker));
		let knuckles =
			r#"{ "device_class" : "controller", "model_number" : "Knuckles Right", "resource_root" : "knuckles" }"#;
		assert_eq!(Kind::of_config(knuckles, "generic"), kind("knuckles", "Knuckles Right", Class::RightController));
		assert_eq!(Kind::of_config(knuckles, "knuckles_l").class, Class::LeftController);
		assert_eq!(Kind::of_subtype("index_hmd"), kind("utah", "", Class::Hmd));
		assert_eq!(driver_of("knuckles"), "indexcontroller");
		assert_eq!(driver_of("utah"), "indexhmd");
		assert_eq!(driver_of("tundra_labs"), "tundra_labs");
	}

	#[test]
	fn a_device_guessed_before_the_runtime_said_what_it_is_is_looked_up_again() {
		let mut s = Symbols::default();
		assert_eq!(s.kind_of("LHR-1", "generic", &[]).class, Class::Tracker);
		assert_eq!(s.kind_of("LHR-1", "knuckles_r", &[]), kind("knuckles", "", Class::RightController));
		assert_eq!(s.kind_of("LHR-1", "knuckles_r", &[]).class, Class::RightController);
		// a config, once found, stays (whatever the runtime calls the device)
		let dir = std::env::temp_dir().join(format!("sally_symbols_cfg_{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		std::fs::write(
			dir.join("LHR-2_config.json"),
			r#"{ "device_class" : "generic_tracker", "model_number" : "Tundra Tracker", "resource_root" : "tundra_labs" }"#,
		)
		.unwrap();
		assert_eq!(
			s.kind_of("LHR-2", "tracker_gen2", std::slice::from_ref(&dir)),
			kind("tundra_labs", "Tundra Tracker", Class::Tracker)
		);
		std::fs::remove_dir_all(&dir).ok();
		assert_eq!(s.kind_of("LHR-2", "generic", &[]).root, "tundra_labs");
		s.retry();
		assert_eq!(s.kind_of("LHR-2", "generic", &[]).root, "tundra_labs");
		assert_eq!(s.kind_of("LHR-1", "wand", &[]).class, Class::Controller);
	}

	/// A picture of `w` by `h` with a block of `rgba` from (2, 1) to its far corner less one
	fn picture(w: u32, h: u32, rgba: [u8; 4]) -> image::RgbaImage {
		let mut img = image::RgbaImage::new(w, h);
		for y in 1..h - 1 {
			for x in 2..w - 1 {
				img.put_pixel(x, y, image::Rgba(rgba));
			}
		}
		img
	}

	#[test]
	fn an_icon_becomes_a_mask_cut_to_its_shape() {
		let mut img = picture(32, 32, [133, 183, 23, 255]);
		// a shade of its green in it, and a soft edge
		img.put_pixel(10, 10, image::Rgba([93, 128, 16, 255]));
		img.put_pixel(2, 1, image::Rgba([133, 183, 23, 128]));
		let m = mask(&img).unwrap();
		assert_eq!(m.size, [29, 30]);
		let at = |x: usize, y: usize| m.pixels[y * 29 + x];
		assert_eq!(at(5, 5).a(), 255);
		// the shade as dark as it is (0.7 of the green), the edge half there
		assert!((at(8, 9).a() as i32 - 178).abs() <= 2, "{}", at(8, 9).a());
		assert_eq!(at(0, 0).a(), 128);
		assert!(is_flat(&picture(8, 8, [70, 70, 75, 255])));
		assert!(!is_flat(&img));
		assert!(mask(&image::RgbaImage::new(8, 8)).is_none());
		// a brighter pixel that is hardly there does not dim the rest (a Tundra tracker's ready icon has one)
		img.put_pixel(1, 1, image::Rgba([255, 255, 255, 40]));
		let m = mask(&img).unwrap();
		assert_eq!(m.size, [30, 30]);
		assert_eq!((m.pixels[0].a(), m.pixels[5 * 30 + 5].a()), (40, 255));
		// a picture of see-through strokes has no solid pixel to go by: its brightest is white
		let thin = picture(16, 16, [90, 90, 90, 150]);
		assert_eq!(mask(&thin).unwrap().pixels[0].a(), 150);
	}

	#[test]
	fn a_dark_drawing_becomes_its_shape_with_its_lines_cut_out() {
		// an off icon as a Tundra tracker's: dark grey with darker lines, and all around it a light edge that is half
		// there
		let mut img = picture(64, 64, [20, 20, 21, 255]);
		for y in 1..63 {
			img.put_pixel(30, y, image::Rgba([4, 4, 4, 255]));
			img.put_pixel(31, y, image::Rgba([12, 12, 12, 255]));
		}
		let edge = image::Rgba([124, 124, 130, 100]);
		for i in 1..64 {
			for (x, y) in [(i, 0), (i, 63), (1, i - 1), (63, i - 1)] {
				img.put_pixel(x, y, edge);
			}
		}
		assert!(!is_flat(&img));
		assert_eq!(Levels::of(&without_light_edge(&img)), Some(Levels { hi: 21.0, lo: 4.0 }));
		let m = mask(&img).unwrap();
		// the light edge is not of it, nor the pixels it touched: the box is the grey's, less one all around
		assert_eq!(m.size, [59, 60]);
		let at = |x: usize, y: usize| m.pixels[y * 59 + x].a();
		// the grey white, the line gone, the pixel between them half
		assert_eq!((at(5, 5), at(27, 5)), (255, 0));
		assert!((at(28, 5) as i32 - 120).abs() <= 2, "{}", at(28, 5));
		// a light speck is not an edge: left as it is
		let mut speck = picture(64, 64, [20, 20, 21, 255]);
		speck.put_pixel(1, 30, edge);
		assert_eq!(mask(&speck).unwrap().size, [62, 62]);
		// its outline is the ready icon's, at half the size too: the same block, bright and flat
		let ready = picture(32, 32, [133, 183, 23, 255]);
		assert!(same_outline(&img, &ready) && same_outline(&ready, &img));
		assert!(same_outline(&ready, &ready));
		// not the same: the block's outline alone, the block with a badge on its corner, a block twice as wide
		let mut outline = image::RgbaImage::new(64, 64);
		let mut badge = picture(64, 64, [133, 183, 23, 255]);
		for i in 2..63 {
			for (x, y) in [(i, 1), (i, 62), (2, i.min(62)), (62, i.min(62))] {
				outline.put_pixel(x, y, image::Rgba([133, 183, 23, 255]));
			}
		}
		for y in 40..64 {
			for x in 40..64 {
				badge.put_pixel(x, y, image::Rgba([0, 0, 0, 0]));
			}
		}
		assert!(!same_outline(&outline, &ready));
		assert!(!same_outline(&badge, &ready));
		assert!(!same_outline(&picture(64, 32, [133, 183, 23, 255]), &ready));
		assert!(!same_outline(&image::RgbaImage::new(8, 8), &ready));
	}

	#[test]
	fn a_symbol_is_found_under_its_driver_the_larger_file_first() {
		let root = std::env::temp_dir().join(format!("sally_symbols_{}", std::process::id()));
		let icons = |d: &str| root.join("drivers").join(d).join("resources").join("icons");
		for d in ["htc", "indexcontroller", "lighthouse"] {
			std::fs::create_dir_all(icons(d)).unwrap();
		}
		std::fs::write(root.join("drivers").join("htc").join("resources").join("driver.vrresources"), HTC).unwrap();
		let index = r#"{ "statusicons" : { "LeftController" : {
			"Prop_NamedIconPathDeviceOff_String" : "{indexcontroller}/icons/left_off.png",
			"Prop_NamedIconPathDeviceReady_String" : "{indexcontroller}/icons/left_ready.png" } } }"#;
		std::fs::write(
			root.join("drivers").join("indexcontroller").join("resources").join("driver.vrresources"),
			index,
		)
		.unwrap();
		let lighthouse = r#"{ "statusicons" : {
			"TrackingReference" : { "Prop_NamedIconPathDeviceReady_String" : "{lighthouse}/icons/base_ready.png" },
			"Valve SR" : { "Prop_NamedIconPathDeviceReady_String" : "{lighthouse}/icons/base2_ready.png" },
			"Tracker" : { "Prop_NamedIconPathDeviceReady_String" : "{lighthouse}/icons/../../x.png" } } }"#;
		std::fs::write(
			root.join("drivers").join("lighthouse").join("resources").join("driver.vrresources"),
			lighthouse,
		)
		.unwrap();
		let green = [133, 183, 23, 255];
		// the tracker: a ready icon of 32, an off one of 64 that is flat
		picture(32, 32, green).save(icons("htc").join("tracker_status_ready.png")).unwrap();
		picture(64, 64, [68, 69, 74, 255]).save(icons("htc").join("tracker_status_off_2x.png")).unwrap();
		// the controller: both sizes of the ready one
		picture(32, 32, green).save(icons("indexcontroller").join("left_ready.png")).unwrap();
		picture(64, 64, green).save(icons("indexcontroller").join("left_ready_2x.png")).unwrap();
		picture(48, 48, green).save(icons("lighthouse").join("base2_ready.png")).unwrap();
		let drivers = test_drivers(&root, &["htc", "indexcontroller", "lighthouse"]);

		let tracker = kind("htc", "VIVE Tracker 3.0 MV", Class::Tracker);
		let files = symbol_files(&drivers, &tracker);
		assert_eq!(files.len(), 2);
		assert!(files[0].0.ends_with("tracker_status_off_2x.png") && files[0].1);
		assert_eq!(load_symbol(&drivers, &tracker).map(|m| m.size), Some([61, 62]));
		// "knuckles" is the Index controllers' folder
		let left = kind("knuckles", "Knuckles Left", Class::LeftController);
		assert_eq!(load_symbol(&drivers, &left).map(|m| m.size), Some([61, 62]));
		// a station by its model
		assert_eq!(load_symbol(&drivers, &Kind::station()).map(|m| m.size), Some([45, 46]));
		// a driver that is not there: the lighthouse driver's set for the class, whose file here leaves its folder
		assert!(load_symbol(&drivers, &kind("tundra_labs", "Tundra Tracker", Class::Tracker)).is_none());
		assert!(load_symbol(&drivers, &kind("htc", "", Class::Station)).is_none());
		// an off icon that is a drawing with the ready icon's outline is taken as well, its line cut out of it
		let mut drawn = picture(64, 64, [18, 18, 20, 255]);
		drawn.put_pixel(20, 20, image::Rgba([4, 4, 4, 255]));
		drawn.save(icons("htc").join("tracker_status_off_2x.png")).unwrap();
		let m = load_symbol(&drivers, &tracker).unwrap();
		assert_eq!(m.size, [61, 62]);
		assert_eq!((m.pixels[0].a(), m.pixels[19 * 61 + 18].a()), (255, 0));
		// one that is another picture is not, flat or not: the ready icon's own file then, small as it is
		picture(64, 40, [68, 69, 74, 255]).save(icons("htc").join("tracker_status_off_2x.png")).unwrap();
		assert_eq!(load_symbol(&drivers, &tracker).map(|m| m.size), Some([29, 30]));
		std::fs::remove_dir_all(&root).ok();
	}
}
