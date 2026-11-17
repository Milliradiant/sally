//! The corrections the map solve puts on the stations and the devices (driver_map_solve.c's station-fcal-file and
//! device-calibration-file sections): each station's factory calibration (OOTX fcal: phase, tilt, curve, gibbous phase
//! and magnitude per sweep plane) corrected by values solved offline, applied only while the factory values are the
//! ones they were solved against, and again whenever an OOTX packet rewrites them; and a device's sensor layout
//! scaled and single sensors moved.

use sally_lighthouse::BaseStationCal;
use sally_math::norm3d;
use sally_math::printf::Arg;

use crate::host::{DeviceId, Host};
use crate::model::MS_MAXLH;
use crate::text::{info, warn};
use sally_math::scanf::{fgets_lines, sscanf, Value};

/// MS_MAXDEV: the devices the map solve keeps (and the layout scales device-calibration-file gives)
pub const MS_MAXDEV: usize = 32;

/// The bytes of a text file as fopen(path, "r") and fgets read it: None when it cannot be opened, what was read
/// before an error otherwise (a directory opens on Linux and reads as nothing, as with fgets)
pub fn read_text_file(path: &str) -> Option<Vec<u8>> {
	use std::io::Read;
	let mut f = std::fs::File::open(path).ok()?;
	let mut bytes = Vec::new();
	let mut chunk = [0u8; 65536];
	loop {
		match f.read(&mut chunk) {
			Ok(0) | Err(_) => break,
			Ok(n) => bytes.extend_from_slice(&chunk[..n]),
		}
	}
	Some(bytes)
}

/// station-fcal-file: per station id the factory values a correction was solved against, and the correction
pub struct StationCorrections {
	/// fc_n, fc_id, fc_ref, fc_d
	fc: Vec<(u32, [f64; 10], [f64; 10])>,
	/// per bsd index: 0 not seen yet, 1 applied, 2 factory values differ (left alone)
	fc_state: [i32; MS_MAXLH],
}

/// The 10 values of a station's fcal in the file's order: phase, tilt, curve, gibpha, gibmag, each axis 0 then axis 1
/// (fcal_get)
fn fcal_get(f: &[BaseStationCal; 2]) -> [f64; 10] {
	let mut v = [0.0; 10];
	for a in 0..2 {
		v[a] = f[a].phase;
		v[2 + a] = f[a].tilt;
		v[4 + a] = f[a].curve;
		v[6 + a] = f[a].gibpha;
		v[8 + a] = f[a].gibmag;
	}
	v
}

/// fcal_set: the 10 values put back (the ogee terms stay)
fn fcal_set(f: &mut [BaseStationCal; 2], v: &[f64; 10]) {
	for a in 0..2 {
		f[a].phase = v[a];
		f[a].tilt = v[2 + a];
		f[a].curve = v[4 + a];
		f[a].gibpha = v[6 + a];
		f[a].gibmag = v[8 + a];
	}
}

/// fcal_same: every value within 1e-6 (a NaN is not)
fn fcal_same(a: &[f64; 10], b: &[f64; 10]) -> bool {
	for i in 0..10 {
		if !((a[i] - b[i]).abs() < 1e-6) {
			return false;
		}
	}
	true
}

impl StationCorrections {
	/// No corrections (station-fcal-file empty)
	pub fn empty() -> StationCorrections {
		StationCorrections { fc: Vec::new(), fc_state: [0; MS_MAXLH] }
	}

	/// fcal_load: the corrections from station-fcal-file (lines 'FCAL <station id> <the 10 factory values> <the 10
	/// deltas>'; at most one per station slot)
	pub fn load(host: &mut (impl Host + ?Sized)) -> StationCorrections {
		let mut sc = StationCorrections::empty();
		let file = host.configs("station-fcal-file", "");
		let file = file.as_str();
		if file.is_empty() {
			return sc;
		}
		let Some(bytes) = read_text_file(file) else {
			warn(host, "station-fcal: cannot read %s; stations keep their factory calibration", &[Arg::S(file)]);
			return sc;
		};
		let fmt = "FCAL %lu %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf";
		for line in fgets_lines(&bytes, 2048) {
			if sc.fc.len() >= MS_MAXLH {
				break;
			}
			let (n, vals) = sscanf(&line, fmt);
			if n != 21 {
				continue;
			}
			let Value::Lu(id) = vals[0] else { unreachable!() };
			let mut v = [0.0; 20];
			for i in 0..20 {
				let Value::Lf(x) = vals[1 + i] else { unreachable!() };
				v[i] = x;
			}
			let mut fref = [0.0; 10];
			let mut fd = [0.0; 10];
			fref.copy_from_slice(&v[..10]);
			fd.copy_from_slice(&v[10..]);
			sc.fc.push((id as u32, fref, fd));
		}
		info(host, "station-fcal: %d station corrections from %s", &[Arg::I(sc.fc.len() as i64), Arg::S(file)]);
		sc
	}

	/// fcal_apply: called with every pose: puts the correction on a station once its factory values are known, and
	/// again after an OOTX packet has rewritten them (the packet also writes the config, so it never stores the
	/// corrected values from that path; the config may still get them from a later save, which the check below
	/// recognizes)
	pub fn apply(&mut self, host: &mut (impl Host + ?Sized), l: usize) {
		if self.fc.is_empty() || !host.bsd_ootx_set(l) || self.fc_state[l] == 2 {
			return;
		}
		let mut k: Option<usize> = None;
		let id = host.bsd_base_station_id(l);
		for i in 0..self.fc.len() {
			if self.fc[i].0 == id {
				k = Some(i);
			}
		}
		let Some(k) = k else { return };
		let mut fcal = host.bsd_fcal(l);
		let cur = fcal_get(&fcal);
		let (_, fref, fd) = self.fc[k];
		let mut want = [0.0; 10];
		for i in 0..10 {
			want[i] = fref[i] + fd[i];
		}
		if fcal_same(&cur, &want) {
			self.fc_state[l] = 1;
			return;
		}
		if !fcal_same(&cur, &fref) {
			warn(
				host,
				"station-fcal: LH %d (%08x): its factory calibration is not the one the correction was solved against; \
				 left uncorrected",
				&[Arg::I(l as i64), Arg::U(id as u64)],
			);
			self.fc_state[l] = 2;
			return;
		}
		fcal_set(&mut fcal, &want);
		host.set_bsd_fcal(l, &fcal);
		let mode = host.bsd_mode(l);
		let state = if self.fc_state[l] != 0 { "re-applied after an OOTX packet" } else { "corrected" };
		info(
			host,
			"station-fcal: LH %d (ch %d, %08x) %s: tilt %+.2f/%+.2f curve %+.2f/%+.2f gibmag %+.2f/%+.2f phase %+.2f/%+.2f mrad",
			&[
				Arg::I(l as i64),
				Arg::I(mode as i64),
				Arg::U(id as u64),
				Arg::S(state),
				Arg::F(fd[2] * 1e3),
				Arg::F(fd[3] * 1e3),
				Arg::F(fd[4] * 1e3),
				Arg::F(fd[5] * 1e3),
				Arg::F(fd[8] * 1e3),
				Arg::F(fd[9] * 1e3),
				Arg::F(fd[0] * 1e3),
				Arg::F(fd[1] * 1e3),
			],
		);
		self.fc_state[l] = 1;
	}
}

/// device-calibration-file: layout scales and sensor moves per device serial, and the devices already corrected
pub struct DeviceCorrections {
	/// ds_serial, ds_scale (at most MS_MAXDEV)
	scales: Vec<(Vec<u8>, f64)>,
	/// dsn_serial, dsn_idx, dsn_d (at most 1024)
	sensors: Vec<(Vec<u8>, i32, [f64; 3])>,
	/// ds_done
	done: Vec<DeviceId>,
}

impl DeviceCorrections {
	/// No corrections (device-calibration-file empty)
	pub fn empty() -> DeviceCorrections {
		DeviceCorrections { scales: Vec::new(), sensors: Vec::new(), done: Vec::new() }
	}

	/// dscale_load: 'SENSOR <serial> <sensor index> <dx> <dy> <dz>' moves one sensor (m, config frame), 'SCALE
	/// <serial> <scale>' scales the whole layout
	pub fn load(host: &mut (impl Host + ?Sized)) -> DeviceCorrections {
		let mut dc = DeviceCorrections::empty();
		let file = host.configs("device-calibration-file", "");
		let file = file.as_str();
		if file.is_empty() {
			return dc;
		}
		let Some(bytes) = read_text_file(file) else {
			warn(host, "device-scale: cannot read %s", &[Arg::S(file)]);
			return dc;
		};
		for line in fgets_lines(&bytes, 256) {
			if dc.scales.len() >= MS_MAXDEV {
				break;
			}
			let (n, v) = sscanf(&line, "SCALE %31s %lf");
			if n == 2 {
				let (Value::S(ser), Value::Lf(sc)) = (&v[0], &v[1]) else { unreachable!() };
				if *sc > 0.95 && *sc < 1.05 {
					dc.scales.push((ser.clone(), *sc));
					continue;
				}
			}
			let (n, v) = sscanf(&line, "SENSOR %31s %d %lf %lf %lf");
			if n == 5 {
				let (Value::S(ser), Value::D(idx), Value::Lf(dx), Value::Lf(dy), Value::Lf(dz)) =
					(&v[0], &v[1], &v[2], &v[3], &v[4])
				else {
					unreachable!()
				};
				if dc.sensors.len() < 1024 && dx.abs() < 0.01 && dy.abs() < 0.01 && dz.abs() < 0.01 {
					dc.sensors.push((ser.clone(), *idx, [*dx, *dy, *dz]));
				}
			}
		}
		info(
			host,
			"device-calibration: %d layout scales and %d sensor corrections from %s",
			&[Arg::I(dc.scales.len() as i64), Arg::I(dc.sensors.len() as i64), Arg::S(file)],
		);
		dc
	}

	/// sally: a device whose layout is another device's from here on (its dongle carries another one): corrected again
	/// once it has its new layout
	pub fn forget(&mut self, dev: DeviceId) {
		self.done.retain(|&d| d != dev);
	}

	/// dscale_apply: a device's corrections, once, as soon as it has a serial and a sensor layout
	pub fn apply(&mut self, host: &mut (impl Host + ?Sized), dev: DeviceId) {
		if self.scales.is_empty() && self.sensors.is_empty() {
			return;
		}
		let serial = host.serial_number(dev);
		if serial.is_empty() || !host.sensor_locations_set(dev) {
			return;
		}
		let sct = host.sensor_ct(dev);
		if sct <= 0 {
			return;
		}
		if self.done.contains(&dev) || self.done.len() >= MS_MAXDEV {
			return;
		}
		self.done.push(dev);
		for i in 0..self.scales.len() {
			if self.scales[i].0 == serial.as_bytes() {
				// the sensor positions themselves (libsurvive's sensor_scale is used by the filter but not by the
				// solver unless the solver estimates it): every user of the layout sees the same corrected size
				let scale = self.scales[i].1;
				for si in 0..sct as usize {
					let mut v = host.sensor_location(dev, si);
					for x in v.iter_mut() {
						*x *= scale;
					}
					host.set_sensor_location(dev, si, &v);
				}
				let code = host.codename(dev);
				info(
					host,
					"device-calibration: %s (%s) sensor layout scaled by %.5f",
					&[Arg::S(&code), Arg::S(&serial), Arg::F(scale)],
				);
			}
		}
		let mut moved = 0;
		let mut mx = 0.0;
		for i in 0..self.sensors.len() {
			let (ref ser, idx, d) = self.sensors[i];
			if *ser == serial.as_bytes() && idx >= 0 && idx < sct {
				let mut v = host.sensor_location(dev, idx as usize);
				for k in 0..3 {
					v[k] += d[k];
				}
				host.set_sensor_location(dev, idx as usize, &v);
				let n = norm3d(&d);
				if n > mx {
					mx = n;
				}
				moved += 1;
			}
		}
		if moved != 0 {
			let code = host.codename(dev);
			info(
				host,
				"device-calibration: %s (%s) %d sensors moved (largest %.2f mm)",
				&[Arg::S(&code), Arg::S(&serial), Arg::I(moved), Arg::F(mx * 1e3)],
			);
		}
	}
}
