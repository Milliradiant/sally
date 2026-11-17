//! sally check: one program, no install. Double-clicked, it records the lighthouse devices next to SteamVR (read
//! only: SteamVR keeps tracking as usual and nothing is sent to any device), then says per device whether its trouble
//! comes from the light (which base stations saw it, on which sensors), the radio (its data arriving), or the device
//! (a dead or covered sensor), as a page in the browser and a zip to send. Given a session folder or its zip (dropped
//! on it, or on the command line), it checks that again; that part runs anywhere, the recording only on Windows.
//!
//! Everything it writes stays in one folder (`sally-check recordings` beside the program); deleting the program and
//! that folder removes it completely.

mod analysis;
mod console;
mod events;
#[cfg(windows)]
mod record;
mod report;
mod session;
mod steamvr;
mod zip;

use std::path::{Path, PathBuf};
use std::process::exit;

/// The folder the recordings go in, beside the program
#[cfg_attr(not(windows), allow(dead_code))]
const RECORDINGS: &str = "sally-check recordings";
/// The longest recording (s)
#[cfg(windows)]
const MAX_SECONDS: u64 = 20 * 60;

fn version() -> &'static str {
	env!("CARGO_PKG_VERSION")
}

fn main() {
	let args: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
	let code = match args.first() {
		Some(a) if a.as_os_str() == "--help" || a.as_os_str() == "-h" => {
			usage();
			0
		}
		Some(p) => check_existing(p),
		None => record(),
	};
	if console::own_window() {
		println!("\nPress Enter to close.");
		console::wait_enter();
	}
	exit(code);
}

fn usage() {
	println!("sally check {}", version());
	println!();
	println!("  sally-check                  record next to SteamVR, then check (Windows; or just double-click it)");
	println!("  sally-check <folder | zip>   check a recording again (writes report.html into its folder)");
}

/// A recording made before, checked again
fn check_existing(p: &Path) -> i32 {
	console::setup("sally check");
	let dir = if p.extension().is_some_and(|e| e.eq_ignore_ascii_case("zip")) {
		let dir = p.with_extension("");
		if !dir.join(session::TAP).exists() {
			if let Err(e) = zip::unzip(p, &dir) {
				eprintln!("Cannot open {}: {e}", p.display());
				return 1;
			}
		}
		dir
	} else if p.is_file() {
		p.parent().unwrap_or(Path::new(".")).to_path_buf()
	} else {
		p.to_path_buf()
	};
	if !dir.join(session::TAP).exists() {
		eprintln!("No {} in {}: not a sally-check recording.", session::TAP, dir.display());
		return 1;
	}
	println!("Checking {} ...", dir.display());
	match write_report(&dir, &dir.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()) {
		Ok(text) => {
			println!("\n{text}");
			println!("Report: {}", dir.join(session::REPORT).display());
			console::open(&dir.join(session::REPORT), false);
			0
		}
		Err(e) => {
			eprintln!("The check failed: {e}");
			1
		}
	}
}

/// The session replayed and analyzed, report.html written; the console's summary returned
fn write_report(dir: &Path, title: &str) -> std::io::Result<String> {
	let a = session::analyze(dir)?;
	let s = session::Session::load(dir);
	let r = report::Report::new(&a, &s);
	std::fs::write(dir.join(session::REPORT), r.html(title))?;
	Ok(r.text())
}

/// Where the recordings go: beside the program, unless it runs from somewhere temporary (opened inside a zip) or
/// read-only; then in Documents
#[cfg_attr(not(windows), allow(dead_code))]
fn recordings_dir() -> PathBuf {
	let beside = std::env::current_exe().ok().and_then(|e| e.parent().map(|p| p.join(RECORDINGS)));
	if let Some(b) = beside {
		let s = b.to_string_lossy().to_ascii_lowercase();
		let temporary = s.contains("\\temp\\") || s.contains("\\temporary internet files\\");
		if !temporary && std::fs::create_dir_all(&b).is_ok() {
			return b;
		}
	}
	let home =
		std::env::var_os("USERPROFILE").or_else(|| std::env::var_os("HOME")).map(PathBuf::from).unwrap_or_default();
	home.join("Documents").join(RECORDINGS)
}

#[cfg(not(windows))]
fn record() -> i32 {
	usage();
	println!("\nRecording needs Windows (SteamVR's dongles); checking a recording works here.");
	2
}

#[cfg(windows)]
fn record() -> i32 {
	use std::time::{Duration, Instant};

	use crate::record::Recording;
	use crate::steamvr::{Client, Paths};

	let vt = console::setup("sally check");
	println!("sally check {}", version());
	println!();
	println!("Records your lighthouse devices next to SteamVR and checks what goes wrong.");
	println!("Read-only: SteamVR keeps tracking as usual, nothing is sent to your devices,");
	println!("and nothing is installed. Recordings stay in the \"{RECORDINGS}\" folder.");
	println!();

	let paths = Paths::find();
	if paths.runtime.is_none() {
		println!("SteamVR doesn't seem to be installed on this PC (no OpenVR path registry).");
		return 1;
	}
	console::stop_on_enter();
	let mut waiting = false;
	let client = loop {
		match Client::connect(&paths) {
			Ok(c) => break c,
			Err(e) => {
				if console::stop_requested() {
					return 1;
				}
				if !waiting {
					println!("Waiting for SteamVR: {e}. Start SteamVR and turn your devices on.");
					println!("(Enter or Ctrl+C to give up.)");
					waiting = true;
				}
				std::thread::sleep(Duration::from_secs(2));
			}
		}
	};
	println!("Connected to SteamVR.");

	let (stamp, when) = console::local_stamp();
	let base = recordings_dir();
	let dir = base.join(format!("sally-check_{stamp}"));
	let rec = match Recording::start(&dir, client, paths.clone()) {
		Ok(r) => r,
		Err(e) => {
			println!("Can't start recording: {e}");
			return 1;
		}
	};
	println!();
	println!("Recording. Do whatever goes wrong (turn the way that loses tracking, move around,");
	println!("play for a bit). A minute or two is plenty. Press Enter (or Ctrl+C) to stop.");
	println!();

	let mut drawn = 0usize;
	while !console::stop_requested() && rec.started.elapsed() < Duration::from_secs(MAX_SECONDS) {
		std::thread::sleep(Duration::from_millis(500));
		let lines = live_view(&rec);
		if vt {
			if drawn > 0 {
				print!("\x1b[{drawn}F");
			}
			for l in &lines {
				println!("\x1b[2K{l}");
			}
			// a device that went quiet leaves no line behind
			print!("\x1b[J");
			drawn = lines.len();
		} else if rec.started.elapsed().as_secs() % 15 == 0 {
			for l in &lines {
				println!("{l}");
			}
			println!();
		}
	}
	let seconds = rec.started.elapsed().as_secs_f64();
	println!();
	println!("Stopped after {:.0} s. Saving and checking...", seconds);
	let started = Instant::now();
	let (dir, known) = rec.stop();
	if let Err(e) = save_extras(&dir, &paths, &known, &when) {
		println!("(Couldn't save everything: {e})");
	}
	let title = format!("{when}, {:.0} s", seconds);
	let text = match write_report(&dir, &title) {
		Ok(t) => t,
		Err(e) => {
			println!("The check failed: {e}. The recording is still in {}", dir.display());
			return 1;
		}
	};
	let zip_path = dir.with_extension("zip");
	let zipped = zip::zip_folder(&dir, &zip_path);
	println!("Checked in {:.1} s.\n", started.elapsed().as_secs_f64());
	println!("{text}");
	println!("The full report is open in your browser: {}", dir.join(session::REPORT).display());
	match zipped {
		Ok(()) => {
			let mb = std::fs::metadata(&zip_path).map_or(0.0, |m| m.len() as f64 / 1e6);
			println!("To share it, send {} ({:.1} MB).", zip_path.display(), mb);
			console::open(&zip_path, true);
		}
		Err(e) => println!("(Couldn't make the zip: {e}; the folder {} has everything.)", dir.display()),
	}
	console::open(&dir.join(session::REPORT), false);
	0
}

/// The files besides the two recordings: SteamVR's devices and their configs (by dongle, as sally replays them), its
/// log, and what the folder is
#[cfg(windows)]
fn save_extras(
	dir: &Path, paths: &steamvr::Paths, known: &std::sync::Mutex<record::Known>, when: &str,
) -> std::io::Result<()> {
	let k = known.lock().unwrap_or_else(|e| e.into_inner());
	std::fs::write(dir.join(session::DEVICES), steamvr::devices_text(&k.devices))?;
	for (dongle, json) in &k.configs {
		if let Some(name) = session::config_file(dongle) {
			std::fs::write(dir.join(name), json)?;
		}
	}
	if let Some(log) = paths.server_log() {
		let _ = std::fs::copy(log, dir.join(session::STEAMVR_LOG));
	}
	std::fs::write(
		dir.join(session::ABOUT),
		format!(
			"sally check {} recording, {when}\r\n\r\n\
			report.html          what the check found (open it in a browser)\r\n\
			tap.hidtap           every report the dongles sent (sally's tap format)\r\n\
			steamvr_poses.csv    what SteamVR showed for each device, 250 times a second\r\n\
			devices.txt          SteamVR's devices: serial, class, role, dongle, model\r\n\
			*_config.json        each device's config (SteamVR's copy), by the dongle it is on\r\n\
			steamvr_log.txt      SteamVR's server log (vrserver.txt)\r\n\r\n\
			The serials of your devices are in these files, and SteamVR's log can contain your Windows user name\r\n\
			in file paths. Replay with sally: sally replay --hidtap-playback tap.hidtap --hidtap-allow-fs-config 1\r\n",
			version()
		),
	)
}

/// The live view: a line per device that sends data
#[cfg(windows)]
fn live_view(rec: &record::Recording) -> Vec<String> {
	let secs = rec.started.elapsed().as_secs();
	let mut out = vec![
		format!("Recording {}:{:02}   (Enter or Ctrl+C to stop)", secs / 60, secs % 60),
		String::new(),
		format!("  {:<24} {:<10} {:<34} {}", "device", "radio", "base stations: sensors lit", "SteamVR"),
	];
	// what SteamVR says of the devices, copied and let go first: the tap's reader holds `shared` when it asks for a
	// device's config, which takes `known` (the analyzer's config source), so `known` held while waiting for `shared`
	// could leave the two waiting on each other, and the recording with them
	let (steam_devices, results) = {
		let known = rec.known.lock().unwrap_or_else(|e| e.into_inner());
		(known.devices.clone(), known.results.clone())
	};
	let shared = rec.shared.lock().unwrap_or_else(|e| e.into_inner());
	let a = &shared.analyzer;
	let now = a.bins();
	for d in &a.devices {
		// the last complete second
		if now < 6 || d.bins.len() + 2 < now {
			continue;
		}
		let window = &d.bins[d.bins.len().saturating_sub(5)..d.bins.len().saturating_sub(1)];
		if window.iter().all(|b| b.reports == 0) {
			continue;
		}
		let step = d.imu_step().unwrap_or(192_000.0);
		let (got, lost) = window.iter().fold((0.0, 0.0), |(g, l), b| (g + b.imu as f64, l + d.imu_lost(b, step)));
		let radio = if got + lost > 0.0 { format!("{:.0}%", got / (got + lost) * 100.0) } else { "-".into() };
		let last = &window[window.len() - 1];
		let mut light: Vec<String> =
			last.stations().map(|c| format!("{}:{}", events::station_name(c), last.sensors[c].count_ones())).collect();
		if light.is_empty() {
			light.push(if last.gen1_hits > 0 {
				format!("{} sensors (gen1)", last.gen1_sensors.count_ones())
			} else {
				"none".into()
			});
		}
		let serial = d.serial().to_string();
		let steam = steam_devices.iter().find(|s| s.serial == serial);
		let name = match steam {
			Some(s) if !s.role.is_empty() => s.role.trim_start_matches("TrackerRole_").to_string(),
			Some(s) => s.class.clone(),
			None => d.config.as_ref().map_or("device", |c| c.kind()).to_string(),
		};
		let state = match results.get(&serial) {
			Some(200) => "tracking",
			Some(201) => "out of range",
			Some(100 | 101) => "calibrating",
			Some(300) => "rotation only",
			Some(_) => "not tracking",
			None => "-",
		};
		out.push(format!(
			"  {:<24} {:<10} {:<34} {}",
			format!("{name} {}", short(&serial)),
			radio,
			light.join(" "),
			state
		));
	}
	if out.len() == 3 {
		out.push("  (no devices sending yet: are they on and paired?)".into());
	}
	out
}

/// A serial's last four characters, to tell devices apart in the narrow view (cut between characters: a serial is the
/// device's own text)
#[cfg_attr(not(windows), allow(dead_code))]
fn short(serial: &str) -> &str {
	serial.char_indices().nth_back(3).map_or(serial, |(i, _)| &serial[i..])
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_serials_tail_is_cut_between_characters() {
		assert_eq!(short("LHR-0A1B2C3D"), "2C3D");
		assert_eq!(short("a\u{e9}123"), "\u{e9}123");
		assert_eq!(short("\u{fffd}AB"), "\u{fffd}AB");
		assert_eq!(short(""), "");
	}
}
