//! Motion capture as BVH (docs/mocap_plan.md section 2: SparsePoser's Xsens recordings, CMU as Bruce Hahne's 2010
//! conversion): the hierarchy (each joint's parent, offset and channels), the frames, and each joint's pose in the
//! world by forward kinematics. A joint's rotation channels compose in the order the file lists them (`Zrotation
//! Yrotation Xrotation` is Rz Ry Rx), in degrees; its translation is its offset, plus its position channels (the
//! root's). Right-handed, Y up, in the file's own units (metres for Xsens, CMU's ~5.6 cm).

use std::path::Path;

use sally_math::math3d::{compose, qmul, Pose, Q4, V3};

/// What a channel moves
#[derive(Clone, Copy, Debug, PartialEq)]
enum Channel {
	Pos(usize),
	Rot(usize),
}

#[derive(Debug, Default)]
pub struct Bvh {
	pub names: Vec<String>,
	/// each joint's parent; the root's is None (joints come after their parents)
	pub parents: Vec<Option<usize>>,
	pub offsets: Vec<V3>,
	chans: Vec<Vec<Channel>>,
	/// where each joint's channels start in a frame's values
	first: Vec<usize>,
	/// values per frame
	width: usize,
	pub frame_time: f64,
	values: Vec<f64>,
}

impl Bvh {
	pub fn frames(&self) -> usize {
		self.values.len().checked_div(self.width).unwrap_or(0)
	}

	pub fn index(&self, name: &str) -> Option<usize> {
		self.names.iter().position(|n| n == name)
	}

	/// Every joint's world pose in frame `f` (the root's translation included), its units times `scale`; `rest`
	/// leaves every channel at zero (the skeleton's rest pose at the origin)
	pub fn pose(&self, f: usize, scale: f64, rest: bool) -> Vec<Pose> {
		let mut out: Vec<Pose> = Vec::with_capacity(self.names.len());
		let row = if rest { &[][..] } else { &self.values[f * self.width..(f + 1) * self.width] };
		for j in 0..self.names.len() {
			let mut t = self.offsets[j];
			let mut q = Q4::default();
			for (k, c) in self.chans[j].iter().enumerate() {
				let v = if rest { 0.0 } else { row[self.first[j] + k] };
				match *c {
					Channel::Pos(a) => match a {
						0 => t.x += v,
						1 => t.y += v,
						_ => t.z += v,
					},
					Channel::Rot(a) => {
						let h = v.to_radians() / 2.0;
						let (s, w) = (h.sin(), h.cos());
						let r = match a {
							0 => Q4::new(w, s, 0.0, 0.0),
							1 => Q4::new(w, 0.0, s, 0.0),
							_ => Q4::new(w, 0.0, 0.0, s),
						};
						q = qmul(q, r);
					}
				}
			}
			let local = Pose { p: t * scale, q };
			out.push(match self.parents[j] {
				Some(p) => compose(&out[p], &local),
				None => local,
			});
		}
		out
	}
}

/// Reads a BVH file
pub fn load(path: &Path) -> Result<Bvh, String> {
	let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
	parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

pub fn parse(text: &str) -> Result<Bvh, String> {
	let mut b = Bvh::default();
	let mut tok = text.split_whitespace().peekable();
	let mut stack: Vec<Option<usize>> = Vec::new();
	let mut current: Option<usize> = None;
	let mut end_site = false;
	let num = |t: Option<&str>| -> Result<f64, String> {
		t.ok_or("the file ends early")?.parse::<f64>().map_err(|e| format!("a number: {e}"))
	};
	while let Some(t) = tok.next() {
		match t {
			"HIERARCHY" => {}
			"ROOT" | "JOINT" => {
				let name = tok.next().ok_or("a joint without a name")?;
				b.names.push(name.to_string());
				b.parents.push(current);
				b.offsets.push(V3::default());
				b.chans.push(Vec::new());
				b.first.push(0);
				current = Some(b.names.len() - 1);
				end_site = false;
			}
			"End" => {
				tok.next(); // Site
				end_site = true;
			}
			"{" => stack.push(if end_site { None } else { current }),
			"}" => {
				let was = stack.pop().ok_or("an unmatched }")?;
				if was.is_none() {
					end_site = false;
				} else if let Some(j) = was {
					current = b.parents[j];
				}
			}
			"OFFSET" => {
				let v = V3::new(num(tok.next())?, num(tok.next())?, num(tok.next())?);
				if !end_site {
					let j = current.ok_or("an OFFSET outside a joint")?;
					b.offsets[j] = v;
				}
			}
			"CHANNELS" => {
				let j = current.ok_or("CHANNELS outside a joint")?;
				let n = num(tok.next())? as usize;
				b.first[j] = b.width;
				for _ in 0..n {
					let c = match tok.next().ok_or("the file ends early")? {
						"Xposition" => Channel::Pos(0),
						"Yposition" => Channel::Pos(1),
						"Zposition" => Channel::Pos(2),
						"Xrotation" => Channel::Rot(0),
						"Yrotation" => Channel::Rot(1),
						"Zrotation" => Channel::Rot(2),
						other => return Err(format!("a channel {other}")),
					};
					b.chans[j].push(c);
				}
				b.width += n;
			}
			"MOTION" => {
				// Frames: N  Frame Time: dt
				while let Some(&t) = tok.peek() {
					match t {
						"Frames:" => {
							tok.next();
							tok.next();
						}
						"Frame" => {
							tok.next();
							tok.next(); // Time:
							b.frame_time = num(tok.next())?;
							break;
						}
						_ => {
							tok.next();
						}
					}
				}
				for t in tok.by_ref() {
					b.values.push(t.parse::<f64>().map_err(|e| format!("a frame's value: {e}"))?);
				}
				let w = b.width.max(1);
				b.values.truncate(b.values.len() / w * w);
				break;
			}
			other => return Err(format!("unexpected {other}")),
		}
	}
	if b.names.is_empty() {
		return Err("no joints".into());
	}
	Ok(b)
}

#[cfg(test)]
mod tests {
	use super::*;
	use sally_math::math3d::len;

	const TWO: &str = "HIERARCHY
ROOT Hips
{
	OFFSET 0 0 0
	CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation
	JOINT Knee
	{
		OFFSET 0 -0.5 0
		CHANNELS 3 Zrotation Yrotation Xrotation
		End Site
		{
			OFFSET 0 -0.4 0
		}
	}
	JOINT Spine
	{
		OFFSET 0 0.3 0
		CHANNELS 3 Zrotation Xrotation Yrotation
	}
}
MOTION
Frames: 2
Frame Time: 0.016667
1 1 0  0 0 0  0 0 0  0 0 0
0 1 0  0 90 0  90 0 0  0 0 0
";

	#[test]
	fn parses_and_poses() {
		let b = parse(TWO).unwrap();
		assert_eq!(b.names, vec!["Hips", "Knee", "Spine"]);
		assert_eq!(b.parents, vec![None, Some(0), Some(0)]);
		assert_eq!((b.frames(), b.width), (2, 12));
		assert!((b.frame_time - 0.016667).abs() < 1e-9);
		let p = b.pose(0, 1.0, false);
		assert!(len(p[1].p - V3::new(1.0, 0.5, 0.0)) < 1e-12, "{:?}", p[1].p);
		// the root turned 90 deg about Y: the knee stays under it; then a turn of 90 deg about the knee's Z
		let p = b.pose(1, 2.0, false);
		assert!(len(p[1].p - V3::new(0.0, 1.0, 0.0)) < 1e-9, "{:?}", p[1].p);
		let foot = sally_math::math3d::xform(&p[1], V3::new(0.0, -0.4, 0.0) * 2.0);
		// the knee's local Z turned by the root's Y turn is the world's X: the shin swings toward -Z... sideways
		assert!((foot.y - 1.0).abs() < 1e-9, "{foot:?}");
		let rest = b.pose(0, 1.0, true);
		assert!(len(rest[2].p - V3::new(0.0, 0.3, 0.0)) < 1e-12);
	}
}
