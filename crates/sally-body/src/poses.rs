//! A session's poses as `sally-poses` writes them (docs/sally_panel.md): `logs\poses_<time>_<label>.csv`, a
//! `# start_epoch,<s>` line, a header, then one row per device and frame (250 Hz): t, idx, class, serial, valid,
//! result, the 3x4 matrix row by row (the position in its last column), the velocity and the angular velocity. Each
//! device's rows are kept in order with their pose and whether SteamVR called it tracking (valid and
//! TrackingResult_Running_OK).

use std::io::{BufRead, BufReader};
use std::path::Path;

use sally_math::math3d::{qfrom_mat, Pose, V3};

/// TrackingResult_Running_OK
const RUNNING_OK: i64 = 200;

/// One device's rows
#[derive(Clone, Debug, Default)]
pub struct Device {
	pub serial: String,
	/// SteamVR's device class as the log names it: hmd, controller, tracker, basestation
	pub class: String,
	pub t: Vec<f64>,
	pub pose: Vec<Pose>,
	/// valid and running OK
	pub ok: Vec<bool>,
}

impl Device {
	/// The last row at or before `t` (the first row before the first)
	pub fn at(&self, t: f64) -> usize {
		self.t.partition_point(|&x| x <= t).saturating_sub(1)
	}

	pub fn duration(&self) -> f64 {
		match (self.t.first(), self.t.last()) {
			(Some(a), Some(b)) => b - a,
			_ => 0.0,
		}
	}
}

/// A whole log
#[derive(Clone, Debug, Default)]
pub struct Session {
	pub start_epoch: f64,
	/// in the order the devices first appear
	pub devices: Vec<Device>,
}

impl Session {
	pub fn by_class<'a>(&'a self, class: &'a str) -> impl Iterator<Item = (usize, &'a Device)> + 'a {
		self.devices.iter().enumerate().filter(move |(_, d)| d.class == class)
	}
}

fn col(header: &[&str], name: &str) -> Result<usize, String> {
	header.iter().position(|h| *h == name).ok_or_else(|| format!("no column {name}"))
}

/// Reads a log; base stations are left out
pub fn load(path: &Path) -> Result<Session, String> {
	let f = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
	let mut lines = BufReader::with_capacity(1 << 20, f).lines();
	let mut s = Session::default();
	let mut header: Option<Vec<String>> = None;
	for l in lines.by_ref() {
		let l = l.map_err(|e| e.to_string())?;
		let l = l.trim_end();
		if let Some(rest) = l.strip_prefix("# start_epoch,") {
			s.start_epoch = rest.trim().parse().unwrap_or(0.0);
		} else if !l.is_empty() && !l.starts_with('#') {
			header = Some(l.split(',').map(str::to_owned).collect());
			break;
		}
	}
	let header = header.ok_or("no header")?;
	let h: Vec<&str> = header.iter().map(String::as_str).collect();
	let (ct, cclass, cser, cvalid, cres) =
		(col(&h, "t")?, col(&h, "class")?, col(&h, "serial")?, col(&h, "valid")?, col(&h, "result")?);
	let names = ["m00", "m01", "m02", "x", "m10", "m11", "m12", "y", "m20", "m21", "m22", "z"];
	let mut cm = [0usize; 12];
	for (i, n) in names.iter().enumerate() {
		cm[i] = col(&h, n)?;
	}
	let mut index: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
	for l in lines {
		let l = l.map_err(|e| e.to_string())?;
		let w: Vec<&str> = l.trim_end().split(',').collect();
		if w.len() < h.len() {
			continue;
		}
		if w[cclass] == "basestation" {
			continue;
		}
		let num = |i: usize| w[i].parse::<f64>().unwrap_or(f64::NAN);
		let t = num(ct);
		if !t.is_finite() {
			continue;
		}
		let mut m = [0.0f64; 12];
		for i in 0..12 {
			m[i] = num(cm[i]);
		}
		let rot = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]];
		let pose = Pose { p: V3::new(m[3], m[7], m[11]), q: qfrom_mat(&rot) };
		let finite = m.iter().all(|x| x.is_finite());
		let ok = finite && w[cvalid] == "1" && w[cres].parse::<i64>().ok() == Some(RUNNING_OK);
		let k = match index.get(w[cser]) {
			Some(&k) => k,
			None => {
				s.devices.push(Device {
					serial: w[cser].to_owned(),
					class: w[cclass].to_owned(),
					..Default::default()
				});
				index.insert(w[cser].to_owned(), s.devices.len() - 1);
				s.devices.len() - 1
			}
		};
		let d = &mut s.devices[k];
		d.t.push(t);
		d.pose.push(pose);
		d.ok.push(ok);
	}
	Ok(s)
}

/// How a device's tracking held: rows, the share tracking, the dark stretches (not tracking, or no rows) over `min_gap`
/// seconds and the longest
#[derive(Debug, Default, Clone, Copy)]
pub struct Coverage {
	pub rows: usize,
	pub ok_share: f64,
	pub gaps: usize,
	pub longest_gap: f64,
	pub dark: f64,
}

pub fn coverage(d: &Device, min_gap: f64) -> Coverage {
	let mut c = Coverage { rows: d.t.len(), ..Default::default() };
	if d.t.is_empty() {
		return c;
	}
	c.ok_share = d.ok.iter().filter(|&&o| o).count() as f64 / d.t.len() as f64;
	// a dark stretch runs from the last tracking row before it (or the log's first row) to the first after it
	let start = d.t[0];
	let mut last_ok: Option<f64> = None;
	for i in 0..d.t.len() {
		if d.ok[i] {
			let gap = d.t[i] - last_ok.unwrap_or(start);
			if gap > min_gap {
				c.gaps += 1;
				c.dark += gap;
				c.longest_gap = c.longest_gap.max(gap);
			}
			last_ok = Some(d.t[i]);
		}
	}
	let tail = d.t[d.t.len() - 1] - last_ok.unwrap_or(start);
	if tail > min_gap {
		c.gaps += 1;
		c.dark += tail;
		c.longest_gap = c.longest_gap.max(tail);
	}
	c
}

/// Where a device had no light: its gaps over 0.1 s without a sweep the tracker used, per serial, in the log's time.
/// SteamVR's flag cannot say it (sally keeps a device valid while it coasts on its IMU); a replay of the session's tap
/// recording can (its TRACK_SWEEP lines), and a file of `serial start end` lines carries it here ('#' lines skipped)
#[derive(Debug, Default)]
pub struct Dark {
	pub gaps: std::collections::HashMap<String, Vec<(f64, f64)>>,
}

impl Dark {
	pub fn load(path: &Path) -> Result<Dark, String> {
		let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
		let mut d = Dark::default();
		for l in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
			let w: Vec<&str> = l.split_whitespace().collect();
			let (Some(s), Some(a), Some(b)) =
				(w.first(), w.get(1).and_then(|x| x.parse().ok()), w.get(2).and_then(|x| x.parse().ok()))
			else {
				return Err(format!("{}: not `serial start end`: {l}", path.display()));
			};
			d.gaps.entry(s.to_string()).or_default().push((a, b));
		}
		for v in d.gaps.values_mut() {
			v.sort_by(|x, y| x.0.total_cmp(&y.0));
		}
		Ok(d)
	}

	/// Whether `serial` had light at `t` (true where nothing is known)
	pub fn lit(&self, serial: &str, t: f64) -> bool {
		let Some(v) = self.gaps.get(serial) else { return true };
		let i = v.partition_point(|g| g.0 <= t);
		i == 0 || t > v[i - 1].1
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn dev(ok: &[bool], dt: f64) -> Device {
		let n = ok.len();
		Device {
			serial: "x".into(),
			class: "tracker".into(),
			t: (0..n).map(|i| i as f64 * dt).collect(),
			pose: vec![Pose::default(); n],
			ok: ok.to_vec(),
		}
	}

	#[test]
	fn coverage_counts_dark_stretches() {
		// 0.004 s rows: 50 rows tracking, 100 dark (0.4 s), 50 tracking
		let mut ok = vec![true; 50];
		ok.extend(vec![false; 100]);
		ok.extend(vec![true; 50]);
		let c = coverage(&dev(&ok, 0.004), 0.1);
		assert_eq!(c.gaps, 1);
		assert!((c.longest_gap - 0.404).abs() < 1e-9, "{}", c.longest_gap);
		assert!((c.ok_share - 0.5).abs() < 1e-12);
	}

	#[test]
	fn coverage_counts_a_dark_start_and_end() {
		let mut ok = vec![false; 60];
		ok.extend(vec![true; 10]);
		ok.extend(vec![false; 60]);
		let c = coverage(&dev(&ok, 0.004), 0.1);
		assert_eq!(c.gaps, 2);
	}

	#[test]
	fn dark_says_where_the_light_was_gone() {
		let mut d = Dark::default();
		d.gaps.insert("A".into(), vec![(1.0, 2.0), (5.0, 5.5)]);
		assert!(d.lit("A", 0.5) && !d.lit("A", 1.5) && d.lit("A", 3.0) && !d.lit("A", 5.2) && d.lit("A", 6.0));
		assert!(d.lit("B", 1.5));
	}

	#[test]
	fn load_reads_rows() {
		let dir = std::env::temp_dir().join(format!("sally-body-test-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let p = dir.join("poses.csv");
		let text = "# start_epoch,1793576427.881054\r\n\
			t,idx,class,serial,valid,result,m00,m01,m02,x,m10,m11,m12,y,m20,m21,m22,z,vx,vy,vz,wx,wy,wz\r\n\
			0.0008,0,hmd,H1,1,200,1,0,0,0.1,0,1,0,1.6,0,0,1,0.2,0,0,0,0,0,0\r\n\
			0.0008,5,basestation,B1,1,200,1,0,0,0,0,1,0,2,0,0,1,0,0,0,0,0,0,0\r\n\
			0.0048,0,hmd,H1,1,201,1,0,0,0.1,0,1,0,1.6,0,0,1,0.2,0,0,0,0,0,0\r\n";
		std::fs::write(&p, text).unwrap();
		let s = load(&p).unwrap();
		std::fs::remove_dir_all(&dir).ok();
		assert_eq!(s.devices.len(), 1);
		let d = &s.devices[0];
		assert_eq!((d.class.as_str(), d.t.len()), ("hmd", 2));
		assert_eq!(d.ok, vec![true, false]);
		assert!((d.pose[0].p.y - 1.6).abs() < 1e-12);
		assert!((s.start_epoch - 1793576427.881054).abs() < 1e-6);
	}
}
