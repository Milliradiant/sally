//! The devices' 3D models: SteamVR's own render models (the OBJ files in the drivers' resources/rendermodels), read
//! from the SteamVR install on this PC, which %LOCALAPPDATA%\openvr\openvrpaths.vrpath names (or sally-panel.cfg's
//! `steamvr`). Nothing of them ships with the panel. A device config's "render_model" is "{driver}name": the folder
//! `name` under any driver's resources/rendermodels (the Index controllers' "{knuckles}" lives in drivers\
//! indexcontroller): its `name`.obj (8926 triangles for an Index controller, 1883 for a Vive tracker 3.0, 700 for a
//! Tundra), its `name`_low_res.obj only where there is no other or the full one is over MAX_TRIS. A model is in the
//! device's head frame (the frame of SteamVR's pose of it), in metres. It is drawn smooth: each corner shaded by its
//! normal (the file's `vn`, else the faces' around it within CREASE_DEG of each other).

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use sally_math::math3d::{cross, dot, len, norm, Q4, V3};

/// A model's triangles, their normals (unit, from the winding: turned to agree with the corners' where the file has
/// them) and their corners' normals (for smooth shading)
pub struct Model {
	pub tris: Vec<[V3; 3]>,
	pub nrm: Vec<V3>,
	pub corner: Vec<[V3; 3]>,
	/// the same indexed, as it is drawn (each vertex moved and each normal shaded once a frame, not once a corner):
	/// the vertices, each triangle's three, the corners' normals, each triangle's three
	pub verts: Vec<V3>,
	pub tri_v: Vec<[u32; 3]>,
	pub norms: Vec<V3>,
	pub tri_n: Vec<[u32; 3]>,
	/// whether the normals face out, as the file's `vn` say (its back faces can be left out); else drawn two-sided
	pub outward: bool,
	/// the farthest vertex from the origin (m)
	pub radius: f64,
}

/// The most triangles a model is drawn with: past it, the low-res one where there is one
pub const MAX_TRIS: usize = 30_000;
/// Faces meeting at a sharper angle keep their edge when the corners' normals are made from the faces'
const CREASE_DEG: f64 = 40.0;

/// An OBJ file's faces as triangles (a polygon as a fan); vertex indices from 1, negative from the end. None
/// without a triangle
pub fn parse_obj(text: &str) -> Option<Model> {
	let mut v: Vec<V3> = Vec::new();
	let mut vn: Vec<V3> = Vec::new();
	// each triangle's corners: the vertex's index, and its normal's where the face gives one
	let mut faces: Vec<[(usize, Option<usize>); 3]> = Vec::new();
	let mut idx: Vec<(usize, Option<usize>)> = Vec::new();
	let at = |w: Option<&str>, n: usize| -> Option<usize> {
		let i = w?.parse::<i64>().ok()?;
		let k = if i < 0 { n as i64 + i } else { i - 1 };
		(0..n as i64).contains(&k).then_some(k as usize)
	};
	for line in text.lines() {
		let mut it = line.split_ascii_whitespace();
		match it.next() {
			Some(kind @ ("v" | "vn")) => {
				let c: Vec<f64> = it.take(3).filter_map(|w| w.parse().ok()).collect();
				// kept when it is no number, so the indices after it still count right
				let p = if c.len() == 3 && c.iter().all(|x| x.is_finite()) {
					V3::new(c[0], c[1], c[2])
				} else {
					V3::default()
				};
				if kind == "v" {
					v.push(p);
				} else {
					vn.push(if len(p) > 0.0 { norm(p) } else { p });
				}
			}
			Some("f") => {
				idx.clear();
				for w in it {
					let mut parts = w.split('/');
					let Some(k) = at(parts.next(), v.len()) else { continue };
					let n = at(parts.nth(1), vn.len());
					idx.push((k, n));
				}
				for j in 1..idx.len().saturating_sub(1) {
					faces.push([idx[0], idx[j], idx[j + 1]]);
				}
			}
			_ => {}
		}
	}
	if faces.is_empty() {
		return None;
	}
	let tris: Vec<[V3; 3]> = faces.iter().map(|f| [v[f[0].0], v[f[1].0], v[f[2].0]]).collect();
	let mut nrm: Vec<V3> = tris
		.iter()
		.map(|t| {
			let n = cross(t[1] - t[0], t[2] - t[0]);
			if len(n) > 0.0 {
				norm(n)
			} else {
				V3::new(0.0, 0.0, 1.0)
			}
		})
		.collect();
	let outward = faces.iter().all(|f| f.iter().all(|c| c.1.is_some_and(|n| len(vn[n]) > 0.0)));
	let corner: Vec<[V3; 3]> = if outward {
		// the file's: each face turned to agree with its corners
		for (n, f) in nrm.iter_mut().zip(&faces) {
			let s = vn[f[0].1.unwrap_or(0)] + vn[f[1].1.unwrap_or(0)] + vn[f[2].1.unwrap_or(0)];
			if dot(*n, s) < 0.0 {
				*n = *n * -1.0;
			}
		}
		faces.iter().map(|f| [vn[f[0].1.unwrap_or(0)], vn[f[1].1.unwrap_or(0)], vn[f[2].1.unwrap_or(0)]]).collect()
	} else {
		// made: at each corner the faces around its vertex that are within CREASE_DEG of this one, by their area
		let mut around: Vec<Vec<usize>> = vec![Vec::new(); v.len()];
		for (i, f) in faces.iter().enumerate() {
			for c in f {
				around[c.0].push(i);
			}
		}
		let cos = CREASE_DEG.to_radians().cos();
		faces
			.iter()
			.enumerate()
			.map(|(i, f)| {
				let mut out = [nrm[i]; 3];
				for (k, c) in f.iter().enumerate() {
					let mut s = V3::default();
					for &j in &around[c.0] {
						// two-sided: a neighbour wound the other way counts turned
						let d = dot(nrm[i], nrm[j]);
						if d.abs() >= cos {
							let t = &tris[j];
							let area = len(cross(t[1] - t[0], t[2] - t[0]));
							s += nrm[j] * (area * d.signum());
						}
					}
					if len(s) > 0.0 {
						out[k] = norm(s);
					}
				}
				out
			})
			.collect()
	};
	let radius = tris.iter().flat_map(|t| t.iter()).map(|p| len(*p)).fold(0.0, f64::max);
	let tri_v: Vec<[u32; 3]> = faces.iter().map(|f| [f[0].0 as u32, f[1].0 as u32, f[2].0 as u32]).collect();
	let (norms, tri_n): (Vec<V3>, Vec<[u32; 3]>) = if outward {
		(vn, faces.iter().map(|f| [0, 1, 2].map(|k| f[k].1.unwrap_or(0) as u32)).collect())
	} else {
		let n = corner.len() as u32;
		(corner.iter().flatten().copied().collect(), (0..n).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect())
	};
	Some(Model { tris, nrm, corner, verts: v, tri_v, norms, tri_n, outward, radius })
}

/// The strings of a JSON array under `key` (openvrpaths.vrpath's "runtime" and "external_drivers"), with \\ and \"
/// undone: enough for that file, not JSON at large
pub fn json_strings(text: &str, key: &str) -> Vec<String> {
	let mut out = Vec::new();
	let Some(k) = text.find(&format!("\"{key}\"")) else { return out };
	let rest = &text[k + key.len() + 2..];
	let Some(open) = rest.find('[') else { return out };
	let Some(close) = rest[open..].find(']') else { return out };
	let body = &rest[open + 1..open + close];
	let mut cur: Option<String> = None;
	let mut esc = false;
	for ch in body.chars() {
		match (&mut cur, ch) {
			(None, '"') => cur = Some(String::new()),
			(None, _) => {}
			(Some(s), c) if esc => {
				s.push(c);
				esc = false;
			}
			(Some(_), '\\') => esc = true,
			(Some(_), '"') => out.push(cur.take().unwrap_or_default()),
			(Some(s), c) => s.push(c),
		}
	}
	out
}

/// A JSON string value under `key`, the first one (a device config's "render_model"); "" when there is none
pub fn json_string(text: &str, key: &str) -> String {
	let Some(k) = text.find(&format!("\"{key}\"")) else { return String::new() };
	let rest = text[k + key.len() + 2..].trim_start();
	let Some(rest) = rest.strip_prefix(':') else { return String::new() };
	let Some(rest) = rest.trim_start().strip_prefix('"') else { return String::new() };
	rest.split('"').next().unwrap_or("").replace("\\\\", "\\")
}

/// Where SteamVR is: sally-panel.cfg's `steamvr`, else the runtimes openvrpaths.vrpath names; and the external
/// drivers it lists
fn steamvr_dirs(steamvr_override: &str) -> (Vec<PathBuf>, Vec<PathBuf>) {
	let mut runtimes: Vec<PathBuf> = Vec::new();
	let mut external: Vec<PathBuf> = Vec::new();
	if !steamvr_override.is_empty() {
		runtimes.push(PathBuf::from(steamvr_override));
	} else if let Some(text) = openvrpaths() {
		runtimes.extend(json_strings(&text, "runtime").into_iter().map(PathBuf::from));
		external.extend(json_strings(&text, "external_drivers").into_iter().map(PathBuf::from));
	}
	(runtimes, external)
}

/// Every driver's own folder (SteamVR's `drivers\<name>`, then the external drivers'), each named as its driver
pub fn driver_dirs(steamvr_override: &str) -> Vec<PathBuf> {
	let (runtimes, external) = steamvr_dirs(steamvr_override);
	let mut dirs = Vec::new();
	for r in &runtimes {
		if let Ok(rd) = std::fs::read_dir(r.join("drivers")) {
			let mut ds: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
			ds.sort();
			dirs.extend(ds);
		}
	}
	dirs.extend(external);
	dirs.retain(|d| d.is_dir());
	dirs
}

/// The folders that may hold render models: every driver's resources/rendermodels, the external drivers' too, and
/// SteamVR's own
pub fn model_roots(steamvr_override: &str) -> Vec<PathBuf> {
	let (runtimes, external) = steamvr_dirs(steamvr_override);
	let mut roots = Vec::new();
	for r in &runtimes {
		if let Ok(rd) = std::fs::read_dir(r.join("drivers")) {
			let mut ds: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
			ds.sort();
			roots.extend(ds.into_iter().map(|d| d.join("resources").join("rendermodels")));
		}
		roots.push(r.join("resources").join("rendermodels"));
	}
	roots.extend(external.into_iter().map(|d| d.join("resources").join("rendermodels")));
	roots.retain(|r| r.is_dir());
	roots
}

pub fn openvrpaths() -> Option<String> {
	let dir = if cfg!(windows) {
		PathBuf::from(std::env::var_os("LOCALAPPDATA")?).join("openvr")
	} else {
		PathBuf::from(std::env::var_os("HOME")?).join(".config").join("openvr")
	};
	std::fs::read(dir.join("openvrpaths.vrpath")).ok().map(|b| String::from_utf8_lossy(&b).into_owned())
}

/// The OBJ files of a render model name ("{driver}name" or "name") under the roots, the driver's own folder first:
/// the full one, then the low-res one
pub fn find_models(roots: &[PathBuf], name: &str) -> Vec<PathBuf> {
	let (driver, bare) = match name.strip_prefix('{').and_then(|r| r.split_once('}')) {
		Some((d, n)) => (d, n),
		None => ("", name),
	};
	if bare.is_empty() || bare.contains(['/', '\\']) || bare.contains("..") {
		return Vec::new();
	}
	let mut order: Vec<&PathBuf> = roots.iter().collect();
	// the driver named first ("{htc}" is drivers\htc), the rest after it
	order.sort_by_key(|r| {
		let d = r.parent().and_then(Path::parent).and_then(Path::file_name).and_then(|f| f.to_str()).unwrap_or("");
		!(d.eq_ignore_ascii_case(driver))
	});
	for r in order {
		let dir = r.join(bare);
		let found: Vec<PathBuf> = [format!("{bare}.obj"), format!("{bare}_low_res.obj")]
			.iter()
			.map(|f| dir.join(f))
			.filter(|p| p.is_file())
			.collect();
		if !found.is_empty() {
			return found;
		}
	}
	Vec::new()
}

/// The models read so far, by render model name (None: looked for and not found, or not read)
#[derive(Default)]
pub struct Models {
	roots: Option<Vec<PathBuf>>,
	steamvr: String,
	cache: HashMap<String, Option<Arc<Model>>>,
	/// device configs' render model names by serial ("" where none was found)
	by_serial: HashMap<String, String>,
}

impl Models {
	/// The SteamVR folder to read from ("" = openvrpaths.vrpath's): the models are looked for again
	pub fn set_steamvr(&mut self, dir: &str) {
		if self.roots.is_none() || self.steamvr != dir {
			self.steamvr = dir.into();
			self.roots = None;
			self.cache.clear();
		}
	}

	/// A render model by name, read the first time it is asked for
	pub fn get(&mut self, name: &str) -> Option<Arc<Model>> {
		if name.is_empty() {
			return None;
		}
		if let Some(m) = self.cache.get(name) {
			return m.clone();
		}
		let roots = self.roots.get_or_insert_with(|| model_roots(&self.steamvr));
		let files = find_models(roots, name);
		let n = files.len();
		// the full one, unless it is over MAX_TRIS and there is a low-res one
		let m = files
			.into_iter()
			.enumerate()
			.filter_map(|(i, p)| {
				let m = parse_obj(&String::from_utf8_lossy(&std::fs::read(p).ok()?))?;
				(m.tris.len() <= MAX_TRIS || i + 1 == n).then_some(m)
			})
			.next()
			.map(Arc::new);
		self.cache.insert(name.into(), m.clone());
		m
	}

	/// A device's render model name by its serial, from its config in `dirs` (LHR-<serial>_config.json, or
	/// <dir>\<serial lowercase>\config.json as SteamVR keeps them), read once
	pub fn name_by_serial(&mut self, serial: &str, dirs: &[PathBuf]) -> String {
		if let Some(n) = self.by_serial.get(serial) {
			return n.clone();
		}
		let mut name = String::new();
		for d in dirs {
			for p in [d.join(format!("{serial}_config.json")), d.join(serial.to_lowercase()).join("config.json")] {
				if let Ok(b) = std::fs::read(&p) {
					name = json_string(&String::from_utf8_lossy(&b), "render_model");
					if !name.is_empty() {
						break;
					}
				}
			}
			if !name.is_empty() {
				break;
			}
		}
		self.by_serial.insert(serial.into(), name.clone());
		name
	}
}

/// A model's head frame against the runtime's where nothing measured it (no SteamVR to compare with, no pose log): a
/// Tundra's is turned 180 deg about its Y axis (sally-driver's HeadFix), the others' the same
pub fn head_turn(model: &str) -> Q4 {
	if model.to_ascii_lowercase().contains("tundra") {
		Q4::new(0.0, 0.0, 1.0, 0.0)
	} else {
		Q4::new(1.0, 0.0, 0.0, 0.0)
	}
}

/// The light the models are shaded by: from above, a little from the side
pub fn light() -> V3 {
	norm(V3::new(0.35, -0.45, 0.82))
}

/// How bright a face whose normal is `n` is under the light `l` (both in one frame): lit from either side
pub fn shade_by(n: V3, l: V3) -> f32 {
	(0.38 + 0.62 * dot(n, l).abs()) as f32
}

/// How bright a face is whose normal in the room is `n`
pub fn shade(n: V3) -> f32 {
	shade_by(n, light())
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_tundra_s_model_is_turned_about_its_y() {
		let q = head_turn("{tundra_labs}tundra_tracker");
		let v = sally_math::math3d::qrot(q, V3::new(0.0, 0.0, 1.0));
		assert!((v.z + 1.0).abs() < 1e-12 && v.x.abs() < 1e-12);
		assert_eq!(head_turn("{htc}vr_tracker_vive_3_0").w, 1.0);
	}

	#[test]
	fn an_obj_s_faces_become_triangles() {
		let text = "# a square and a triangle\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\n\
		            f 1/1/1 2/1/1 3/1/1 4/1/1\nf -4 -3 -1\nf 1 2 9\n";
		let m = parse_obj(text).expect("a model");
		// the quad as two, the triangle by negative indices, the face with an index past the end cut to two corners
		assert_eq!(m.tris.len(), 3);
		assert_eq!(m.tris[2], [V3::new(0.0, 0.0, 0.0), V3::new(1.0, 0.0, 0.0), V3::new(0.0, 1.0, 0.0)]);
		assert!((m.nrm[0].z - 1.0).abs() < 1e-12);
		assert!((m.radius - 2f64.sqrt()).abs() < 1e-12);
		// the quad's corners have the file's normal, the others none: the model's normals are made, not the file's
		assert!(!m.outward);
		assert!(parse_obj("v 0 0 0\n").is_none());
	}

	#[test]
	fn the_corners_take_the_file_s_normals_else_the_faces_around_them() {
		// a roof: two faces at 90 deg (over CREASE_DEG: each keeps its own), no normals in the file
		let roof = "v -1 0 0\nv 0 1 0\nv 0 1 1\nv -1 0 1\nv 1 0 0\nv 1 0 1\n";
		let m = parse_obj(&format!("{roof}f 1 2 3\nf 1 3 4\nf 2 5 6\nf 2 6 3\n")).unwrap();
		assert!(!m.outward);
		assert!(dot(m.corner[0][1], m.nrm[0]) > 0.999, "a crease keeps its edge");
		// a gentle fold (20 deg): one normal at the shared corners, between the two faces
		let fold = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0.36\n";
		let m = parse_obj(&format!("{fold}f 1 2 3\nf 2 4 3\n")).unwrap();
		let shared = m.corner[0][1];
		assert!(dot(shared, m.nrm[0]) < 0.999 && dot(shared, m.nrm[1]) < 0.999 && dot(shared, m.nrm[0]) > 0.9);
		// the file's normals, a face wound the other way turned to agree with them
		let m = parse_obj("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 -1\nf 1//1 2//1 3//1\n").unwrap();
		assert!(m.outward && m.nrm[0].z < -0.999 && m.corner[0][2].z < -0.999);
		// the indexed form is the same triangles and corners
		for m in [m, parse_obj(&format!("{fold}f 1 2 3\nf 2 4 3\n")).unwrap()] {
			for i in 0..m.tris.len() {
				assert_eq!(m.tri_v[i].map(|k| m.verts[k as usize]), m.tris[i]);
				assert_eq!(m.tri_n[i].map(|k| m.norms[k as usize]), m.corner[i]);
			}
		}
	}

	#[test]
	fn openvrpaths_runtime_and_drivers() {
		let text = r#"{ "config" : [ "C:\\Program Files (x86)\\Steam\\config" ], "external_drivers" : [ "D:\\drv\\a", "e" ],
			"jsonid" : "vrpathreg", "runtime" : [ "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR" ] }"#;
		assert_eq!(json_strings(text, "runtime"), vec![r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR"]);
		assert_eq!(json_strings(text, "external_drivers"), vec![r"D:\drv\a", "e"]);
		assert!(json_strings(text, "nothing").is_empty());
		let cfg = r#"{ "device_class": "generic_tracker", "render_model" : "{htc}vr_tracker_vive_3_0", "x": 1 }"#;
		assert_eq!(json_string(cfg, "render_model"), "{htc}vr_tracker_vive_3_0");
		assert_eq!(json_string(cfg, "model"), "");
	}

	#[test]
	fn a_model_is_found_under_any_driver_the_full_one_first() {
		let root = std::env::temp_dir().join(format!("sally_models_{}", std::process::id()));
		let rm = |d: &str| root.join("drivers").join(d).join("resources").join("rendermodels");
		let k = rm("indexcontroller").join("valve_controller_knu_1_0_left");
		let t = rm("htc").join("vr_tracker_vive_3_0");
		std::fs::create_dir_all(&k).unwrap();
		std::fs::create_dir_all(&t).unwrap();
		std::fs::write(
			k.join("valve_controller_knu_1_0_left.obj"),
			"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\n",
		)
		.unwrap();
		std::fs::write(k.join("valve_controller_knu_1_0_left_low_res.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
			.unwrap();
		std::fs::write(t.join("vr_tracker_vive_3_0.obj"), "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\n")
			.unwrap();
		let roots = model_roots(root.to_str().unwrap());
		assert_eq!(roots.len(), 2);
		// "{knuckles}" names no folder: the model is in indexcontroller's
		let p = find_models(&roots, "{knuckles}valve_controller_knu_1_0_left");
		assert!(
			p[0].ends_with("valve_controller_knu_1_0_left.obj")
				&& p[1].ends_with("valve_controller_knu_1_0_left_low_res.obj")
		);
		assert!(find_models(&roots, "{htc}../htc").is_empty());
		let mut ms = Models::default();
		ms.set_steamvr(root.to_str().unwrap());
		assert_eq!(ms.get("{htc}vr_tracker_vive_3_0").map(|m| m.tris.len()), Some(2));
		assert_eq!(ms.get("{knuckles}valve_controller_knu_1_0_left").map(|m| m.tris.len()), Some(2));
		assert!(ms.get("{htc}nothing").is_none());
		std::fs::remove_dir_all(&root).ok();
	}
}
