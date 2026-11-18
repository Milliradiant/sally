//! A check's session: one folder holding what was recorded, named so that sally replays it as it is (`sally replay
//! --hidtap-playback tap.hidtapz --hidtap-allow-fs-config 1`, run in the folder), and the same folder zipped to send.
//!
//! - `tap.hidtapz`: every report the dongles sent, in the tap's line format, packed (`tap.hidtap`, the text, in a
//!   session an older sally-check recorded)
//! - `<dongle serial>_config.json`: the config of the device paired to each dongle (SteamVR's own copy)
//! - `steamvr_poses.csv`: what SteamVR showed for every device on the tap's clock
//! - `devices.txt`: SteamVR's devices: serial, class, role, dongle, model, tracking system, mode label
//! - `steamvr_log.txt`: SteamVR's server log (vrserver.txt) at the end of the session
//! - `report.html`: what the check found
//! - `about.txt`: what the folder is
//!
//! A session the panel shares (`share`) has stand-ins for every serial in all of these, the room's files and sally's
//! own log beside them, and of SteamVR's log only the lines of Valve's lighthouse driver and of sally's.

use std::collections::HashMap;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::Path;

use sally_hid::replay::Replay;
use sally_hid::tap::Options;

use crate::analysis::Analyzer;

pub const TAP: &str = "tap.hidtapz";
/// The recording as an older sally-check wrote it
pub const TAP_TEXT: &str = "tap.hidtap";
pub const POSES: &str = "steamvr_poses.csv";
pub const DEVICES: &str = "devices.txt";
pub const STEAMVR_LOG: &str = "steamvr_log.txt";
pub const REPORT: &str = "report.html";
pub const ABOUT: &str = "about.txt";

/// The header of `steamvr_poses.csv`
pub const POSES_HEADER: &str = "t_us,serial,result,valid,connected,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz";
/// The header of `devices.txt`
pub const DEVICES_HEADER: &str = "serial\tclass\trole\tdongle\tmodel\tsystem\tmode";

/// A device as SteamVR listed it
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SteamDevice {
	pub serial: String,
	/// hmd, controller, tracker, station
	pub class: String,
	/// the role SteamVR gives it (waist, left_foot, left hand, ...), or empty
	pub role: String,
	/// the dongle it is connected to (its serial), or empty
	pub dongle: String,
	pub model: String,
	/// "lighthouse", or another tracking system's name
	pub system: String,
	/// a station's mode label (its channel for 2.0 stations)
	pub mode: String,
}

impl SteamDevice {
	pub fn line(&self) -> String {
		let f = |s: &str| s.replace(['\t', '\n', '\r'], " ");
		format!(
			"{}\t{}\t{}\t{}\t{}\t{}\t{}",
			f(&self.serial),
			f(&self.class),
			f(&self.role),
			f(&self.dongle),
			f(&self.model),
			f(&self.system),
			f(&self.mode)
		)
	}

	fn parse(line: &str) -> Option<SteamDevice> {
		let f: Vec<&str> = line.trim_end_matches(['\r', '\n']).split('\t').collect();
		if f.len() < 7 || f[0] == "serial" {
			return None;
		}
		Some(SteamDevice {
			serial: f[0].into(),
			class: f[1].into(),
			role: f[2].into(),
			dongle: f[3].into(),
			model: f[4].into(),
			system: f[5].into(),
			mode: f[6].into(),
		})
	}
}

/// One SteamVR pose sample
#[derive(Clone, Copy, Debug, Default)]
pub struct Pose {
	pub t_us: u64,
	/// ETrackingResult (200 = running OK)
	pub result: i32,
	pub valid: bool,
	pub connected: bool,
	pub pos: [f64; 3],
	/// w, x, y, z
	pub rot: [f64; 4],
	pub vel: [f64; 3],
	pub ang: [f64; 3],
}

impl Pose {
	/// The row of `steamvr_poses.csv` for the device `serial`
	pub fn csv(&self, serial: &str) -> String {
		format!(
			"{},{},{},{},{},{:.5},{:.5},{:.5},{:.6},{:.6},{:.6},{:.6},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}\n",
			self.t_us,
			serial,
			self.result,
			self.valid as u8,
			self.connected as u8,
			self.pos[0],
			self.pos[1],
			self.pos[2],
			self.rot[0],
			self.rot[1],
			self.rot[2],
			self.rot[3],
			self.vel[0],
			self.vel[1],
			self.vel[2],
			self.ang[0],
			self.ang[1],
			self.ang[2]
		)
	}

	fn parse(line: &str) -> Option<(String, Pose)> {
		let f: Vec<&str> = line.trim_end().split(',').collect();
		if f.len() < 18 {
			return None;
		}
		let n = |i: usize| f[i].parse::<f64>().ok();
		Some((
			f[1].to_string(),
			Pose {
				t_us: f[0].parse().ok()?,
				result: f[2].parse().ok()?,
				valid: f[3] == "1",
				connected: f[4] == "1",
				pos: [n(5)?, n(6)?, n(7)?],
				rot: [n(8)?, n(9)?, n(10)?, n(11)?],
				vel: [n(12)?, n(13)?, n(14)?],
				ang: [n(15)?, n(16)?, n(17)?],
			},
		))
	}
}

/// A session folder read back
pub struct Session {
	pub devices: Vec<SteamDevice>,
	/// SteamVR's poses by serial, in time order
	pub poses: HashMap<String, Vec<Pose>>,
	/// sally's SteamVR driver was running (its log lines are in SteamVR's log): SteamVR showed sally's tracking
	pub sally: bool,
}

impl Session {
	pub fn load(dir: &Path) -> Session {
		let devices = fs::read_to_string(dir.join(DEVICES))
			.map(|t| t.lines().filter_map(SteamDevice::parse).collect())
			.unwrap_or_default();
		let mut poses: HashMap<String, Vec<Pose>> = HashMap::new();
		if let Ok(f) = fs::File::open(dir.join(POSES)) {
			for line in BufReader::new(f).lines().map_while(Result::ok) {
				if let Some((serial, p)) = Pose::parse(&line) {
					poses.entry(serial).or_default().push(p);
				}
			}
		}
		for v in poses.values_mut() {
			v.sort_by_key(|p| p.t_us);
		}
		let log = fs::read(dir.join(STEAMVR_LOG)).unwrap_or_default();
		let sally = log.windows(21).any(|w| w == b"sally: sally: driver ");
		Session { devices, poses, sally }
	}

	/// SteamVR's device for a serial
	pub fn steam_device(&self, serial: &str) -> Option<&SteamDevice> {
		self.devices.iter().find(|d| d.serial.eq_ignore_ascii_case(serial))
	}

	/// How high a device was over the session: the median of SteamVR's heights above its floor (m)
	pub fn height(&self, serial: &str) -> Option<f64> {
		let mut y: Vec<f64> = self.poses.get(serial)?.iter().filter(|p| p.valid).map(|p| p.pos[1]).collect();
		if y.len() < 50 {
			return None;
		}
		y.sort_by(|a, b| a.total_cmp(b));
		Some(y[y.len() / 2])
	}

	/// The base station on a gen2 channel (the raw channel: SteamVR's mode label is one higher), by its serial
	pub fn station(&self, channel: usize) -> Option<&str> {
		self.devices
			.iter()
			.find(|d| d.class == "station" && d.mode.trim().parse::<usize>().ok() == Some(channel + 1))
			.map(|d| d.serial.as_str())
	}
}

/// The session's recording, packed or as text; None for a folder that is not a session
pub fn tap(dir: &Path) -> Option<std::path::PathBuf> {
	[TAP, TAP_TEXT].iter().map(|n| dir.join(n)).find(|p| p.is_file())
}

/// What SteamVR's devices are, for the session's devices.txt (base stations included, for their channels)
pub fn devices_text(devices: &[SteamDevice]) -> String {
	let mut s = String::from(DEVICES_HEADER);
	s.push('\n');
	for d in devices {
		s.push_str(&d.line());
		s.push('\n');
	}
	s
}

/// The file in the session's folder a dongle's device config is kept in (`<dongle serial>_config.json`), for a serial
/// that can only name a file there: the serials of a recording are read from it, and a recording to check may come
/// from anyone (one with "../" or a drive in a serial read configs from anywhere on the disk)
pub fn config_file(dongle: &str) -> Option<String> {
	(!dongle.is_empty() && !dongle.contains(['/', '\\', ':', '.'])).then(|| format!("{dongle}_config.json"))
}

/// Replays a session's tap recording into an analyzer, the configs from the folder
pub fn analyze(dir: &Path) -> std::io::Result<Analyzer> {
	let cfg_dir = dir.to_path_buf();
	let mut a = Analyzer::new(Box::new(move |dongle: &str| fs::read(cfg_dir.join(config_file(dongle)?)).ok()));
	let tap = tap(dir).ok_or_else(|| std::io::Error::new(std::io::ErrorKind::NotFound, "no recording"))?;
	let text = sally_hid::pack::open(&tap)?;
	let options = Options { playback_factor: 0.0, allow_fs_read: true, config_fetch: false, ..Options::default() };
	let mut replay = Replay::new(&mut a, options, &tap.to_string_lossy(), None);
	replay.run(&mut a, &mut BufReader::with_capacity(1 << 20, text))?;
	replay.close(&mut a);
	a.finish();
	Ok(a)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_dongles_config_is_a_file_in_the_folder_or_none() {
		assert_eq!(config_file("D5096F43FE-1YX").as_deref(), Some("D5096F43FE-1YX_config.json"));
		for serial in ["", "../../x", "..", "a/b", "a\\b", "C:x", "/etc/x", "x.y"] {
			assert_eq!(config_file(serial), None, "{serial:?}");
		}
	}
}
