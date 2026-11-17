//! sally-poses: every SteamVR device's pose (the headset, trackers, controllers, base stations) written to a CSV file
//! at 250 Hz, as tools\pose_logger.py wrote it: `logs\poses_<yyyyMMdd_HHmmss>_<label>.csv` in the project (two folders
//! up from this program, as the panel finds it; `--logs <folder>` elsewhere). It only watches SteamVR (a background
//! application: SteamVR must be running) and changes nothing. Ctrl+C stops it.
//!
//! The file is the Python's, byte for byte in its layout: CRLF lines (Python's csv writer), a first row
//! `# start_epoch,<Unix time>`, the header row, then per connected device and frame `t` (s since the start, 4 places),
//! its index, class, serial, valid, tracking result, the 3x4 matrix row by row (6 places) and the velocity and angular
//! velocity (4 places).

use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use sally_panel::openvr::{
	k_unMaxTrackedDeviceCount, Interface, OpenVr, Prop_SerialNumber_String, TrackedDevicePose_t,
	VRApplication_Background, Vr,
};

const RATE_HZ: f64 = 250.0;

const TRACKING_UNIVERSE_STANDING: i32 = 1;

/// Cleared by Ctrl+C
static RUNNING: AtomicBool = AtomicBool::new(true);

/// CLASSES: the tracked device classes by their OpenVR value
fn class_name(class: i32) -> String {
	match class {
		1 => "hmd".into(),
		2 => "controller".into(),
		3 => "tracker".into(),
		4 => "basestation".into(),
		c => c.to_string(),
	}
}

/// One device's row
fn row(t: f64, idx: u32, class: &str, serial: &str, p: &TrackedDevicePose_t) -> String {
	let m = &p.mDeviceToAbsoluteTracking.m;
	let (v, a) = (&p.vVelocity.v, &p.vAngularVelocity.v);
	let mut s = format!("{t:.4},{idx},{class},{serial},{},{}", p.bPoseIsValid as i32, p.eTrackingResult);
	for r in m {
		for c in r {
			s += &format!(",{:.6}", *c as f64);
		}
	}
	for x in v.iter().chain(a.iter()) {
		s += &format!(",{:.4}", *x as f64);
	}
	s.push_str("\r\n");
	s
}

/// The project's logs folder: `--logs`, else two up from this program (tools\sally_panel\), else beside it
fn logs_folder(args: &[String]) -> PathBuf {
	if let Some(i) = args.iter().position(|a| a == "--logs") {
		if let Some(p) = args.get(i + 1) {
			return PathBuf::from(p);
		}
	}
	let exe = std::env::current_exe().unwrap_or_default();
	let dir = exe.parent().map(Path::to_path_buf).unwrap_or_default();
	let project = dir.parent().and_then(Path::parent).map(Path::to_path_buf);
	project.filter(|p| p.join("logs").is_dir()).unwrap_or(dir).join("logs")
}

fn main() {
	let args: Vec<String> = std::env::args().skip(1).collect();
	let label = args
		.iter()
		.enumerate()
		.find(|&(i, a)| !a.starts_with("--") && (i == 0 || args[i - 1] != "--logs"))
		.map(|(_, a)| a.clone())
		.unwrap_or_else(|| "session".into());
	let logs = logs_folder(&args);
	let _ = std::fs::create_dir_all(&logs);
	let stamp = sally_os::localtime::local_now().map(|t| t.stamp()).unwrap_or_default();
	let path = logs.join(format!("poses_{stamp}_{label}.csv"));

	let mut vr = OpenVr::new();
	if vr.load().is_err() {
		eprintln!("openvr_api.dll was not found beside this program");
		std::process::exit(1);
	}
	if let Err(e) = vr.init(VRApplication_Background) {
		eprintln!("SteamVR did not answer (OpenVR error {e}): start SteamVR first");
		std::process::exit(1);
	}
	if let Err(e) = vr.interface(Interface::System) {
		eprintln!("SteamVR has no IVRSystem (OpenVR error {e})");
		std::process::exit(1);
	}
	sys::catch_ctrl_c();
	let file = match std::fs::File::create(&path) {
		Ok(f) => f,
		Err(e) => {
			eprintln!("cannot write {}: {e}", path.display());
			std::process::exit(1);
		}
	};
	let mut out = BufWriter::new(file);
	println!("Recording to {}", path.display());
	println!("Press Ctrl+C to stop.");
	let t0 = Instant::now();
	let epoch0 = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
	let period = 1.0 / RATE_HZ;
	let n = k_unMaxTrackedDeviceCount as usize;
	let mut names: Vec<Option<(String, String)>> = vec![None; n];
	let mut rows = 0u64;
	let _ = write!(out, "# start_epoch,{epoch0:.6}\r\n");
	let _ =
		write!(out, "t,idx,class,serial,valid,result,m00,m01,m02,x,m10,m11,m12,y,m20,m21,m22,z,vx,vy,vz,wx,wy,wz\r\n");
	let mut next = 0.0;
	let mut poses = vec![TrackedDevicePose_t::default(); n];
	while RUNNING.load(Ordering::SeqCst) {
		vr.device_to_absolute_tracking_pose(TRACKING_UNIVERSE_STANDING, 0.0, &mut poses);
		let t = t0.elapsed().as_secs_f64();
		for (i, p) in poses.iter().enumerate() {
			if p.bDeviceIsConnected == 0 {
				continue;
			}
			let (class, serial) = names[i]
				.get_or_insert_with(|| {
					let class = class_name(vr.tracked_device_class(i as u32));
					let serial = vr.string_tracked_device_property(i as u32, Prop_SerialNumber_String);
					(class, if serial.is_empty() { format!("dev{i}") } else { serial })
				})
				.clone();
			let _ = out.write_all(row(t, i as u32, &class, &serial, p).as_bytes());
			rows += 1;
		}
		next += period;
		let now = t0.elapsed().as_secs_f64();
		if next > now {
			std::thread::sleep(Duration::from_secs_f64(next - now));
		} else {
			next = now;
		}
		if (t as i64) != ((t - period) as i64) && (t as i64) % 10 == 0 {
			let seen = names.iter().filter(|n| n.is_some()).count();
			println!("  {} s recorded, {seen} devices", t as i64);
			let _ = out.flush();
		}
	}
	let _ = out.flush();
	vr.shutdown();
	println!("Saved {rows} rows to {}", path.display());
}

/// Ctrl+C clears RUNNING, so the file is closed whole (Windows: the console handler; elsewhere Ctrl+C ends the
/// program, the rows written up to the last flush kept)
mod sys {
	#[cfg(windows)]
	pub fn catch_ctrl_c() {
		#[link(name = "kernel32")]
		extern "system" {
			fn SetConsoleCtrlHandler(handler: extern "system" fn(u32) -> i32, add: i32) -> i32;
		}
		extern "system" fn handler(_ctrl_type: u32) -> i32 {
			super::RUNNING.store(false, std::sync::atomic::Ordering::SeqCst);
			// let the loop end and close the file: the handler returns at once, the main thread finishes
			std::thread::sleep(std::time::Duration::from_millis(500));
			1
		}
		// SAFETY: the handler is a function of the signature the call asks for, and lives as long as the program
		unsafe { SetConsoleCtrlHandler(handler, 1) };
	}

	#[cfg(not(windows))]
	pub fn catch_ctrl_c() {}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_row_is_the_pythons() {
		let mut p = TrackedDevicePose_t::default();
		p.mDeviceToAbsoluteTracking.m = [[1.0, 0.0, 0.0, 0.25], [0.0, 1.0, 0.0, 1.5], [0.0, 0.0, 1.0, -0.125]];
		p.vVelocity.v = [0.1, 0.0, -2.5];
		p.vAngularVelocity.v = [0.0, 3.0, 0.0];
		p.eTrackingResult = 200;
		p.bPoseIsValid = 1;
		p.bDeviceIsConnected = 1;
		assert_eq!(
			row(1.23456, 3, "tracker", "LHR-2D779713", &p),
			"1.2346,3,tracker,LHR-2D779713,1,200,1.000000,0.000000,0.000000,0.250000,0.000000,1.000000,0.000000,\
			 1.500000,0.000000,0.000000,1.000000,-0.125000,0.1000,0.0000,-2.5000,0.0000,3.0000,0.0000\r\n"
		);
		assert_eq!(class_name(4), "basestation");
		assert_eq!(class_name(7), "7");
	}
}
