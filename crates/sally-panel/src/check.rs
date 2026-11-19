//! The Check tab's work: a stretch of the running session checked and made into one file to send.
//!
//! The runtime records every session, so a check records nothing of the devices itself. Between Start and Stop it
//! notes the time (the recording's clock, which is the PC's) and writes down what SteamVR shows for each device
//! (`PoseLog`, fed by Valve's side as it reads the poses). At Stop the stretch is handed to `sally_check::share` on
//! a thread of its own: cut from the session's recording, checked, and zipped with the room's files and the logs,
//! every serial a stand-in and nothing in it naming the PC or its user. What comes back is the zip, the page of what
//! the check found (which also says which stand-in is which device, and stays on the PC), and the check in a few
//! lines for the tab.
//!
//! Where things are is `replay::Paths`' to say: the session's recording is the newest in the logs folder that is
//! still being written; the devices' configs are the runtime's copies and SteamVR's; the room's files are the ones
//! the driver's settings give the runtime; the shares go beside sally-check's recordings of old, where the Replay tab
//! lists them.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::thread::JoinHandle;

use sally_check::session::{self, SteamDevice};
use sally_check::share::{self, Private, Request, Shared};

use crate::openvr::{
	TrackedDeviceClass_Controller, TrackedDeviceClass_GenericTracker, TrackedDeviceClass_HMD,
	TrackedDeviceClass_TrackingReference, TrackedDevicePose_t,
};
use crate::replay::Paths;
use crate::valve::ValveDevice;

/// The longest check (s): it stops by itself then
pub const MAX_S: f64 = 600.0;
/// Without the runtime's word a session's recording counts as being written when its last report is no older than
/// this (us)
const LIVE_US: u64 = 10_000_000;
/// How long after Stop the recording is read: the runtime writes it in blocks of half a second
const SETTLE: std::time::Duration = std::time::Duration::from_millis(1500);
/// The shares kept
const KEEP: usize = 5;

/// A device's class as a session's files name it
pub fn class_word(cls: i32) -> &'static str {
	if cls == TrackedDeviceClass_HMD {
		"hmd"
	} else if cls == TrackedDeviceClass_Controller {
		"controller"
	} else if cls == TrackedDeviceClass_GenericTracker {
		"tracker"
	} else if cls == TrackedDeviceClass_TrackingReference {
		"station"
	} else {
		"other"
	}
}

/// What SteamVR shows, written as `steamvr_poses.csv`'s rows while a check records: the headset and every device
/// tracked by the base stations, each time Valve's side reads the poses
pub struct PoseLog {
	out: BufWriter<File>,
	pub rows: u64,
}

impl PoseLog {
	fn new(path: &Path) -> std::io::Result<PoseLog> {
		Ok(PoseLog { out: BufWriter::with_capacity(1 << 16, File::create(path)?), rows: 0 })
	}

	/// The poses just read, by the devices' indices
	pub fn rows(&mut self, by_index: &BTreeMap<i32, ValveDevice>, poses: &[TrackedDevicePose_t]) {
		let t_us = sally_os::absolute_time_us();
		for d in by_index.values() {
			let Some(p) = poses.get(d.index as usize) else { continue };
			let tracked = d.cls == TrackedDeviceClass_HMD
				|| (d.system == "lighthouse"
					&& (d.cls == TrackedDeviceClass_GenericTracker || d.cls == TrackedDeviceClass_Controller));
			if !tracked || d.serial.is_empty() || p.bDeviceIsConnected == 0 {
				continue;
			}
			let m = &p.mDeviceToAbsoluteTracking.m;
			let q = crate::valve::from_mat34(&p.mDeviceToAbsoluteTracking).q;
			let pose = session::Pose {
				t_us,
				result: p.eTrackingResult,
				valid: p.bPoseIsValid != 0,
				connected: true,
				pos: [m[0][3] as f64, m[1][3] as f64, m[2][3] as f64],
				rot: [q.w, q.x, q.y, q.z],
				vel: p.vVelocity.v.map(|v| v as f64),
				ang: p.vAngularVelocity.v.map(|v| v as f64),
			};
			let _ = self.out.write_all(pose.csv(&d.serial).as_bytes());
			self.rows += 1;
		}
	}
}

/// A check that is recording
pub struct Recording {
	/// when it started: the recording's clock, and the panel's
	pub from_us: u64,
	pub since: f64,
	stamp: String,
	when: String,
	session: PathBuf,
	poses: PathBuf,
	devices: Vec<SteamDevice>,
}

/// Where a check stands
#[derive(Default)]
pub enum Check {
	#[default]
	Idle,
	Recording(Recording),
	/// since when (the panel's clock), how long the stretch is, and the thread making the share
	Making(f64, f64, JoinHandle<Result<Shared, String>>),
	Done(Result<Shared, String>),
}

/// The session being recorded: the newest recording in the logs folder, when the runtime says it is recording
/// (`recording`) or the file's last report is from just now. With no device sending, the recording stands still (a
/// session's may have no report at all), and only the runtime's word says it is the one being written
fn live_session(logs: &Path, recording: bool) -> Option<PathBuf> {
	let now = sally_os::absolute_time_us();
	let mut taps: Vec<PathBuf> = std::fs::read_dir(logs)
		.ok()?
		.flatten()
		.map(|e| e.path())
		.filter(|p| {
			p.file_name().and_then(|n| n.to_str()).is_some_and(|n| {
				n.strip_prefix("sallytap_").and_then(sally_hid::pack::stem).is_some_and(|s| s.len() == 15)
			})
		})
		.collect();
	taps.sort();
	let newest = taps.pop()?;
	if recording {
		return Some(newest);
	}
	let (_, last) = crate::replay::tap_span(&newest)?;
	let last_us = (last * 1e6) as u64;
	(last_us <= now.saturating_add(LIVE_US) && now.saturating_sub(last_us) <= LIVE_US).then_some(newest)
}

/// The value the runtime's arguments give an option
fn arg(args: &[String], name: &str) -> Option<PathBuf> {
	args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).map(PathBuf::from)
}

/// The devices listed at the start and at the end as one list: a device there at either, what is known of it at the
/// end over what was known at the start
fn merged(mut first: Vec<SteamDevice>, last: Vec<SteamDevice>) -> Vec<SteamDevice> {
	for d in last {
		match first.iter_mut().find(|o| o.serial == d.serial) {
			Some(o) => {
				let keep = |new: String, old: &mut String| {
					if !new.is_empty() {
						*old = new;
					}
				};
				keep(d.role, &mut o.role);
				keep(d.dongle, &mut o.dongle);
				keep(d.model, &mut o.model);
				keep(d.mode, &mut o.mode);
			}
			None => first.push(d),
		}
	}
	first
}

impl Check {
	pub fn recording(&self) -> Option<&Recording> {
		match self {
			Check::Recording(r) => Some(r),
			_ => None,
		}
	}

	/// Start: Err says why there is nothing to check (no session being recorded). `recording`: the runtime answers and
	/// says it is a live one with a recording
	pub fn start(
		paths: &Paths, devices: Vec<SteamDevice>, recording: bool, now: f64,
	) -> Result<(Recording, PoseLog), String> {
		let session = live_session(&paths.logs, recording).ok_or_else(|| match recording {
			true => format!("sally's recording of this session is not in {}.", paths.logs.display()),
			false => "sally is not recording a session right now. It records while SteamVR runs with sally.".into(),
		})?;
		std::fs::create_dir_all(&paths.checks).map_err(|e| format!("cannot make {}: {e}", paths.checks.display()))?;
		// what a check that was cut short left behind (the panel closed while it was being made)
		for e in std::fs::read_dir(&paths.checks).into_iter().flatten().flatten() {
			let name = e.file_name().to_string_lossy().into_owned();
			if name.starts_with("sally-check_") && (name.ends_with(".poses") || name.ends_with(".cut")) {
				let _ = std::fs::remove_file(e.path());
			}
		}
		let t = sally_os::localtime::local_now();
		let stamp = t.as_ref().map(|t| t.stamp()).unwrap_or_else(|| "00000000_000000".into());
		let when = match stamp.as_bytes() {
			s if s.len() == 15 => {
				format!("{}-{}-{} {}:{}", &stamp[..4], &stamp[4..6], &stamp[6..8], &stamp[9..11], &stamp[11..13])
			}
			_ => stamp.clone(),
		};
		let poses = paths.checks.join(format!("sally-check_{stamp}.poses"));
		let log = PoseLog::new(&poses).map_err(|e| format!("cannot write {}: {e}", poses.display()))?;
		let from_us = sally_os::absolute_time_us();
		Ok((Recording { from_us, since: now, stamp, when, session, poses, devices }, log))
	}

	/// Stop: the stretch from the start to now made into a share, on a thread of its own
	pub fn stop(
		rec: Recording, log: Option<PoseLog>, paths: &Paths, devices: Vec<SteamDevice>, version: &str, now: f64,
	) -> Check {
		drop(log);
		let to_us = sally_os::absolute_time_us();
		let room_dir = arg(&paths.runtime_args, "--configfile").and_then(|c| c.parent().map(Path::to_path_buf));
		let room: Vec<PathBuf> = ["--configfile", "--station-fcal-file", "--map-solve-file", "--device-learned-file"]
			.iter()
			.filter_map(|n| arg(&paths.runtime_args, n))
			.collect();
		let settings_dir = paths.driver_settings.parent().map(Path::to_path_buf).unwrap_or_default();
		let stamp_of = |p: &Path| {
			p.file_name()
				.and_then(|n| n.to_str())
				.and_then(|n| n.strip_prefix("sallytap_"))
				.and_then(sally_hid::pack::stem)
				.map(str::to_string)
		};
		let request = Request {
			span: Some((rec.from_us, to_us)),
			config_dirs: paths.config_dirs(),
			devices: merged(rec.devices, devices),
			poses: Some(rec.poses.clone()),
			room,
			sally_log: stamp_of(&rec.session).map(|s| paths.logs.join(format!("sallytap_{s}_solver.txt"))),
			settings: ["sally_settings.txt", "sally_output.txt", "sally_user.txt"]
				.iter()
				.map(|n| settings_dir.join(n))
				.collect(),
			steamvr_log: sally_lighthouse::steamvr::Paths::find().server_log(),
			names: room_dir.unwrap_or_else(|| paths.project.clone()).join("share_names.txt"),
			private: Private::here(&paths.project),
			out: paths.checks.clone(),
			stamp: rec.stamp,
			when: rec.when,
			version: version.to_string(),
			keep: KEEP,
			recording: rec.session,
		};
		let poses = rec.poses;
		let seconds = (to_us.saturating_sub(rec.from_us)) as f64 / 1e6;
		let work = move || {
			std::thread::sleep(SETTLE);
			let made = share::share(&request);
			let _ = std::fs::remove_file(&poses);
			made
		};
		match std::thread::Builder::new().name("check".into()).spawn(work) {
			Ok(handle) => Check::Making(now, seconds, handle),
			Err(e) => Check::Done(Err(format!("the check did not start: {e}"))),
		}
	}

	/// The share's thread looked at: Making becomes Done when it has ended
	pub fn poll(&mut self) {
		if matches!(self, Check::Making(_, _, h) if h.is_finished()) {
			if let Check::Making(_, _, handle) = std::mem::take(self) {
				*self = Check::Done(handle.join().unwrap_or_else(|_| Err("the check stopped unexpectedly".into())));
			}
		}
	}
}

/// A file opened with its usual program, or shown selected in its folder
pub fn open(path: &Path, select: bool) {
	if cfg!(windows) {
		let mut c = std::process::Command::new("explorer.exe");
		if select {
			c.arg(format!("/select,{}", path.display()));
		} else {
			c.arg(path);
		}
		let _ = c.spawn();
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_devices_of_the_start_and_the_end_are_one_list() {
		let d = |serial: &str, role: &str, dongle: &str| SteamDevice {
			serial: serial.into(),
			role: role.into(),
			dongle: dongle.into(),
			..SteamDevice::default()
		};
		let all = merged(
			vec![d("LHR-1", "", "D1"), d("LHR-2", "TrackerRole_Waist", "")],
			vec![d("LHR-2", "", "D2"), d("LHR-1", "TrackerRole_Chest", ""), d("LHR-3", "", "D3")],
		);
		assert_eq!(
			all,
			[d("LHR-1", "TrackerRole_Chest", "D1"), d("LHR-2", "TrackerRole_Waist", "D2"), d("LHR-3", "", "D3")]
		);
	}

	/// A check starts on a session no device is sending in: the runtime's word makes the newest recording the
	/// session's, where the recording alone (no report, or none for a while) does not
	#[test]
	fn a_session_no_device_sends_in_is_the_runtimes_to_name() {
		let dir = std::env::temp_dir().join(format!("sally-panel-check-{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(&dir).unwrap();
		assert_eq!(live_session(&dir, true), None);
		// an older session with reports, and the one being written: its header alone
		let old = sally_os::absolute_time_us() - 60_000_000;
		std::fs::write(dir.join("sallytap_20261118_120000.hidtap"), format!("R {old} 0 2 00\nR {} 0 2 00\n", old + 9))
			.unwrap();
		let live = dir.join("sallytap_20261118_130000.hidtapz");
		std::fs::write(&live, sally_hid::pack::Packer::new().header()).unwrap();
		std::fs::write(dir.join("sallytap_20261118_130000_solver.txt"), "").unwrap();
		assert_eq!(live_session(&dir, false), None);
		assert_eq!(live_session(&dir, true), Some(live.clone()));
		// a report from just now says so by itself
		std::fs::remove_file(&live).unwrap();
		let now = sally_os::absolute_time_us();
		std::fs::write(dir.join("sallytap_20261118_130000.hidtap"), format!("R {now} 0 2 00\nR {now} 0 2 00\n"))
			.unwrap();
		assert_eq!(live_session(&dir, false), Some(dir.join("sallytap_20261118_130000.hidtap")));
		let _ = std::fs::remove_dir_all(&dir);
	}

	#[test]
	fn an_option_is_read_from_the_runtimes_arguments() {
		let args: Vec<String> =
			["--hidtap", "1", "--configfile", "C:\\x\\driver_config.json"].map(String::from).to_vec();
		assert_eq!(arg(&args, "--configfile"), Some(PathBuf::from("C:\\x\\driver_config.json")));
		assert_eq!(arg(&args, "--map-solve-file"), None);
	}
}
