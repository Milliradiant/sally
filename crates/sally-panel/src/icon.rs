//! The panel's icon on the taskbar and in the title bar: sally's icon (crates/sally-setup/sally.ico, the file the
//! installer hands Windows too) with its dot painted again in the colour of what sally is doing: tracking, waiting,
//! stopped by something, or idle. The dot's place and size are the drawing's (sally.svg, and sally-small.svg for 16
//! and 24 pixels).

/// The icon file, as the installer ships it
const ICO: &[u8] = include_bytes!("../../sally-setup/sally.ico");

/// The drawing's tile, behind the leaf and the dot
const TILE: [u8; 3] = [0x15, 0x17, 0x1b];
/// The dot's centre in the drawing's 256 units (its (182, 198) through its scale of 0.86 about the middle)
const DOT_AT: (f64, f64) = (174.44, 188.2);
/// The dot's radius, and that of the bite it sits in, in sally.svg (22 and 35 through the scale)
const LARGE: (f64, f64) = (18.92, 30.1);
/// ... and in sally-small.svg (27 and 40)
const SMALL: (f64, f64) = (23.22, 34.4);
/// The largest picture drawn from sally-small.svg
const SMALL_UP_TO: usize = 24;

/// What the dot says
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dot {
	/// SteamVR is shown sally's tracking
	Tracking,
	/// sally is on its way there: starting, learning the room, lining up
	Waiting,
	/// something only the user can change is in the way
	Stopped,
	/// nothing to track with: SteamVR is not running, or is shown Valve's tracking by choice
	Idle,
}

impl Dot {
	pub fn rgb(self) -> [u8; 3] {
		match self {
			Dot::Tracking => [0x8f, 0xf0, 0xb4],
			Dot::Waiting => [0xf0, 0xb5, 0x63],
			Dot::Stopped => [0xf0, 0x76, 0x6a],
			Dot::Idle => [0x7b, 0x80, 0x90],
		}
	}
}

/// One of the icon file's pictures: its side in pixels, and its pixels as RGBA from the top row down
#[derive(Clone, Debug, PartialEq)]
pub struct Picture {
	pub side: usize,
	pub rgba: Vec<u8>,
}

/// The icon file's bitmaps (its 256-pixel picture is a PNG, and left out): each a 40-byte header, then 32-bit BGRA
/// rows from the bottom up
fn pictures(ico: &[u8]) -> Vec<Picture> {
	let word = |i: usize| ico.get(i..i + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize);
	let count = ico.get(4..6).map_or(0, |b| u16::from_le_bytes([b[0], b[1]]) as usize);
	let mut out = Vec::new();
	for n in 0..count {
		let entry = 6 + 16 * n;
		let (Some(&width), Some(size), Some(at)) = (ico.get(entry), word(entry + 8), word(entry + 12)) else { break };
		let side = if width == 0 { 256 } else { width as usize };
		let Some(data) = at.checked_add(size).and_then(|end| ico.get(at..end)) else { continue };
		if data.len() < 40 + side * side * 4 || data[..4] != [40, 0, 0, 0] || data[14..16] != [32, 0] {
			continue;
		}
		let mut rgba = Vec::with_capacity(side * side * 4);
		for y in (0..side).rev() {
			for bgra in data[40 + y * side * 4..][..side * 4].chunks_exact(4) {
				rgba.extend([bgra[2], bgra[1], bgra[0], bgra[3]]);
			}
		}
		out.push(Picture { side, rgba });
	}
	out
}

/// How much of a pixel a disc covers, 0 to 1: 4 x 4 points of the pixel
fn cover(px: usize, py: usize, centre: (f64, f64), radius: f64) -> f64 {
	let mut inside = 0;
	for sy in 0..4 {
		for sx in 0..4 {
			let (x, y) = (px as f64 + (sx as f64 + 0.5) / 4.0, py as f64 + (sy as f64 + 0.5) / 4.0);
			if (x - centre.0).powi(2) + (y - centre.1).powi(2) <= radius * radius {
				inside += 1;
			}
		}
	}
	inside as f64 / 16.0
}

/// The picture's dot in another colour. The dot as it is drawn goes first, its soft edge with it: the tile's colour
/// over a disc a little larger, kept inside the bite; then the dot again
fn paint(p: &mut Picture, colour: [u8; 3]) {
	let k = p.side as f64 / 256.0;
	let (dot, bite) = if p.side <= SMALL_UP_TO { SMALL } else { LARGE };
	let (dot, bite) = (dot * k, bite * k);
	let centre = (DOT_AT.0 * k, DOT_AT.1 * k);
	let wipe = (dot + 1.0).min((dot + bite) / 2.0 + 0.25);
	for (radius, colour) in [(wipe, TILE), (dot, colour)] {
		let from = |c: f64| (c - radius).floor().max(0.0) as usize;
		let to = |c: f64| ((c + radius).ceil().max(0.0) as usize).min(p.side);
		for y in from(centre.1)..to(centre.1) {
			for x in from(centre.0)..to(centre.0) {
				let c = cover(x, y, centre, radius);
				let px = &mut p.rgba[(y * p.side + x) * 4..][..3];
				for (v, new) in px.iter_mut().zip(colour) {
					*v = (*v as f64 * (1.0 - c) + new as f64 * c).round() as u8;
				}
			}
		}
	}
}

/// The icon for a place `want` pixels wide, its dot saying `dot`: the smallest picture at least that large (the
/// largest where none is). None where the icon file holds no bitmap, which it always does
pub fn picture(want: usize, dot: Dot) -> Option<Picture> {
	let mut all = pictures(ICO);
	all.sort_by_key(|p| p.side);
	let at = all.iter().position(|p| p.side >= want).or(all.len().checked_sub(1))?;
	let mut p = all.swap_remove(at);
	paint(&mut p, dot.rgb());
	Some(p)
}

#[cfg(test)]
mod tests {
	use super::*;

	fn pixel(p: &Picture, x: usize, y: usize) -> [u8; 4] {
		p.rgba[(y * p.side + x) * 4..][..4].try_into().unwrap()
	}

	fn near(a: [u8; 4], rgb: [u8; 3]) -> bool {
		a[3] == 255 && a[..3].iter().zip(rgb).all(|(v, w)| v.abs_diff(w) <= 3)
	}

	/// The icon file's pictures are read, each at its size, and the one asked for is the smallest that is large enough
	#[test]
	fn the_icon_files_pictures_by_size() {
		let sides: Vec<usize> = pictures(ICO).iter().map(|p| p.side).collect();
		assert_eq!(sides, [16, 24, 32, 48, 64]);
		for (want, side) in [(1, 16), (16, 16), (17, 24), (32, 32), (40, 48), (64, 64), (200, 64)] {
			let p = picture(want, Dot::Tracking).unwrap();
			assert_eq!((p.side, p.rgba.len()), (side, side * side * 4), "{want}");
		}
		// a corner is outside the tile's round corner, the tile's middle edge is the tile
		let p = picture(64, Dot::Idle).unwrap();
		assert_eq!(pixel(&p, 0, 0)[3], 0);
		assert!(near(pixel(&p, 32, 2), TILE));
		assert!(pictures(&ICO[..40]).is_empty() && pictures(&[]).is_empty());
	}

	/// The dot takes each colour at every size, nothing of the drawn one's colour is left around it, and the rest of
	/// the picture is as the file has it
	#[test]
	fn the_dot_is_painted_in_the_colour_of_what_sally_is_doing() {
		for original in pictures(ICO) {
			let side = original.side;
			let k = side as f64 / 256.0;
			let (cx, cy) = ((DOT_AT.0 * k) as usize, (DOT_AT.1 * k) as usize);
			// as drawn, the dot is the tracking one's colour
			assert!(near(pixel(&original, cx, cy), Dot::Tracking.rgb()), "{side}");
			for dot in [Dot::Waiting, Dot::Stopped, Dot::Idle, Dot::Tracking] {
				let p = picture(side, dot).unwrap();
				assert!(near(pixel(&p, cx, cy), dot.rgb()), "{side} {dot:?}");
				let bite = if side <= SMALL_UP_TO { SMALL.1 } else { LARGE.1 } * k;
				for y in 0..side {
					for x in 0..side {
						let d =
							((x as f64 + 0.5 - DOT_AT.0 * k).powi(2) + (y as f64 + 0.5 - DOT_AT.1 * k).powi(2)).sqrt();
						let (was, is) = (pixel(&original, x, y), pixel(&p, x, y));
						if d > bite + 1.0 {
							assert_eq!(was, is, "{side} {dot:?} at {x} {y}");
						} else if dot == Dot::Stopped && d < bite - 1.0 {
							// red has little green: what is left in the bite is the tile or the dot, no mint between
							assert!(is[1] <= Dot::Stopped.rgb()[1].max(TILE[1]) + 3, "{side} at {x} {y}: {is:?}");
						}
					}
				}
			}
		}
	}
}
