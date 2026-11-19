//! A session put together to send: one zip that holds a check's session (`crate::session`) with a stand-in for every
//! serial, and beside it the page of what the check found, for the one who made it.
//!
//! Nothing is copied into a share as it is. Each file is one sally knows how to make clean, and is made again:
//!
//! | file | made from | how |
//! |---|---|---|
//! | `tap.hidtapz` | the session's recording | `recording`: the span, stand-ins, the stations' ids in the light, zeros where nothing reads |
//! | `<dongle>_config.json`, `<device>_config.json` | the devices' configs | `config`: what sally reads of one |
//! | `devices.txt`, `steamvr_poses.csv` | what SteamVR listed and showed | written here, by stand-in |
//! | `driver_config.json`, `station_fcal.txt`, `map_solve.txt`, `device_learned.txt` | the room's files | `text` |
//! | `sally_log.txt`, `sally_settings.txt` | the runtime's log, the driver's settings | `text` |
//! | `steamvr_log.txt` | SteamVR's log | the lines of Valve's lighthouse driver and of sally's alone, `text` |
//! | `report.html`, `about.txt` | the check | written here |
//!
//! Then every file is checked for what it must not hold (`recording::only_stand_ins`, `text::clean`): a serial that
//! is not a stand-in, the user's folder or name, the PC's name. If anything is found there is no zip.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use super::names::{Kind, Names};
use super::text::{self, Private};
use super::{config, recording};
use crate::analysis::{Analyzer, ConfigSource};
use crate::report::Report;
use crate::session::{self, SteamDevice};

/// The most of a log that goes into a share: its start (where the devices are set up) and its end
const LOG_HEAD: usize = 300_000;
const LOG_TAIL: usize = 700_000;

/// What to share
#[derive(Clone, Debug, Default)]
pub struct Request {
	/// the session's recording, and the span of it to share (the recording's clock, microseconds; None: all of it)
	pub recording: PathBuf,
	pub span: Option<(u64, u64)>,
	/// where the devices' configs are: `<serial>_config.json` by a device's or a dongle's serial, or SteamVR's
	/// `<serial in lower case>\config.json`, in the first folder that has one
	pub config_dirs: Vec<PathBuf>,
	/// SteamVR's devices, as it listed them
	pub devices: Vec<SteamDevice>,
	/// what SteamVR showed (`steamvr_poses.csv`'s rows, with the real serials), if it was recorded
	pub poses: Option<PathBuf>,
	/// the room's files, the runtime's log, the driver's settings, SteamVR's log: each where it is, left out where
	/// it is not
	pub room: Vec<PathBuf>,
	pub sally_log: Option<PathBuf>,
	pub settings: Vec<PathBuf>,
	pub steamvr_log: Option<PathBuf>,
	/// the stand-ins' table
	pub names: PathBuf,
	pub private: Private,
	/// the folder the zip and the page go into, the session's stamp (`20261118_142501`) and its time as people read
	/// it, sally's version
	pub out: PathBuf,
	pub stamp: String,
	pub when: String,
	pub version: String,
	/// how many shares stay in `out` (the newest; 0: all)
	pub keep: usize,
}

/// What was made
#[derive(Clone, Debug, Default)]
pub struct Shared {
	pub zip: PathBuf,
	pub bytes: u64,
	/// the page for the one who made it: the report, and which of their devices each stand-in is
	pub report: PathBuf,
	/// the check in a few lines
	pub text: String,
	pub seconds: f64,
	/// what there is to say about the share (a device without its config, station blocks lost)
	pub notes: Vec<String>,
	/// what was done to the recording
	pub recording: recording::Scrubbed,
}

fn io(what: &str) -> impl Fn(std::io::Error) -> String + '_ {
	move |e| format!("{what}: {e}")
}

/// A device's config by its serial in the folders
fn find_config(dirs: &[PathBuf], serial: &str) -> Option<Vec<u8>> {
	if serial.is_empty() || serial.contains(['/', '\\', ':']) || serial.starts_with('.') {
		return None;
	}
	dirs.iter()
		.flat_map(|d| [d.join(format!("{serial}_config.json")), d.join(serial.to_lowercase()).join("config.json")])
		.find_map(|p| std::fs::read(p).ok())
}

fn source(map: HashMap<String, Vec<u8>>) -> ConfigSource {
	Box::new(move |key: &str| map.iter().find(|(k, _)| k.eq_ignore_ascii_case(key)).map(|(_, v)| v.clone()))
}

/// A log's start and end, by whole lines, when it is longer than a share holds
fn capped(text: &[u8]) -> Vec<u8> {
	if text.len() <= LOG_HEAD + LOG_TAIL {
		return text.to_vec();
	}
	let head = text[..LOG_HEAD].iter().rposition(|&c| c == b'\n').map_or(LOG_HEAD, |i| i + 1);
	let from = text.len() - LOG_TAIL;
	let tail = text[from..].iter().position(|&c| c == b'\n').map_or(from, |i| from + i + 1);
	let mut out = text[..head].to_vec();
	out.extend_from_slice(format!("[... {} bytes left out ...]\n", tail - head).as_bytes());
	out.extend_from_slice(&text[tail..]);
	out
}

/// The base stations' ids a room's file names, as numbers: a config's `"id":"<n>"`, and the first number of an
/// `FCAL`, `LH` or `ANCHOR` line and the first two of a `LAM` line
fn room_stations(text: &str) -> Vec<u32> {
	let mut ids = Vec::new();
	for line in text.lines() {
		let f: Vec<&str> = line.split_ascii_whitespace().collect();
		match f.first().copied() {
			Some("FCAL" | "LH" | "ANCHOR") => ids.extend(f.get(1).and_then(|n| n.parse::<u32>().ok())),
			Some("LAM") => ids.extend(f.iter().skip(1).take(2).filter_map(|n| n.parse::<u32>().ok())),
			_ => {
				if let Some(n) = line.trim().strip_prefix("\"id\":\"").and_then(|r| r.split('"').next()) {
					ids.extend(n.parse::<u32>().ok());
				}
			}
		}
	}
	ids
}

/// SteamVR's log as a share holds it: the lines of Valve's lighthouse driver and of sally's driver, scrubbed
fn steamvr_lines(log: &[u8], names: &mut Names, private: &Private) -> Vec<u8> {
	let mut out = Vec::new();
	for line in log.split(|&c| c == b'\n') {
		let text = String::from_utf8_lossy(line);
		let said = text.split_once(" - ").map_or("", |(_, r)| r);
		let valve = said.starts_with("lighthouse:");
		if valve || said.starts_with("sally:") {
			out.extend_from_slice(&text::scrub(text.trim_end_matches('\r').as_bytes(), names, private, valve));
			out.push(b'\n');
		}
	}
	out
}

/// `steamvr_poses.csv` with the stand-ins, the rows of the span alone
fn poses(rows: &[u8], span: (u64, u64), devices: &[SteamDevice], names: &mut Names) -> Vec<u8> {
	let mut out = format!("{}\n", session::POSES_HEADER).into_bytes();
	let mut by: HashMap<String, String> = HashMap::new();
	for line in rows.split(|&c| c == b'\n') {
		let Ok(line) = std::str::from_utf8(line) else { continue };
		let mut f = line.trim_end().splitn(3, ',');
		let (Some(t), Some(serial), Some(rest)) = (f.next(), f.next(), f.next()) else { continue };
		let Ok(t_us) = t.parse::<u64>() else { continue };
		if t_us < span.0 || t_us > span.1 {
			continue;
		}
		let stand_in = by.entry(serial.to_string()).or_insert_with(|| {
			let class = devices.iter().find(|d| d.serial == serial).map_or("device", |d| d.class.as_str());
			names.serial(serial, class)
		});
		out.extend_from_slice(format!("{t},{stand_in},{rest}\n").as_bytes());
	}
	out
}

/// The page's last part, for the one who made the share: which of their devices each stand-in is
fn own_devices(devices: &[SteamDevice], names: &Names) -> String {
	let mut rows = String::new();
	for d in devices {
		let (kind, digits) = if let Some(x) = super::names::device_digits(&d.serial) {
			(Kind::Device, x)
		} else if let Some(id) = super::names::station_id(&d.serial) {
			(Kind::Station, format!("{id:08X}"))
		} else {
			continue;
		};
		let Some(stand_in) = names.known(kind, &digits) else { continue };
		let prefix = if kind == Kind::Device { "LHR" } else { "LHB" };
		let what = match d.role.trim_start_matches("TrackerRole_") {
			"" => d.class.clone(),
			role => format!("{} ({role})", d.class),
		};
		rows.push_str(&format!(
			"<tr><td><code>{prefix}-{stand_in}</code></td><td>{what}</td><td>{}</td><td>ends in <code>{}</code></td></tr>",
			d.model,
			&digits[4..]
		));
	}
	format!(
		"<section><h2>Your devices</h2><p>The file to send names your devices by stand-ins, so that it does not say \
		 whose they are. This table is only in this page, which stays on your PC.</p><table><tr><th>in the file</th>\
		 <th>what</th><th>model</th><th>its serial</th></tr>{rows}</table></section>"
	)
}

/// The shares in `out` beyond the newest `keep`, removed
fn prune(out: &Path, keep: usize) {
	if keep == 0 {
		return;
	}
	let mut stamps: Vec<String> = std::fs::read_dir(out)
		.map(|rd| {
			rd.flatten()
				.filter_map(|e| e.file_name().to_str().map(str::to_string))
				.filter_map(|n| n.strip_prefix("sally-check_")?.strip_suffix(".zip").map(str::to_string))
				.filter(|s| s.len() == 15)
				.collect()
		})
		.unwrap_or_default();
	stamps.sort();
	stamps.reverse();
	for s in stamps.iter().skip(keep) {
		let _ = std::fs::remove_file(out.join(format!("sally-check_{s}.zip")));
		let _ = std::fs::remove_file(out.join(format!("sally-check_{s}.html")));
	}
}

/// The session `r` names made into a zip to send and a page to read. On any failure nothing is left but the error
pub fn share(r: &Request) -> Result<Shared, String> {
	std::fs::create_dir_all(&r.out).map_err(io("cannot make the folder"))?;
	let work = r.out.join(format!("sally-check_{}", r.stamp));
	let _ = std::fs::remove_dir_all(&work);
	std::fs::create_dir_all(&work).map_err(io("cannot make the folder"))?;
	let made = make(r, &work);
	let _ = std::fs::remove_dir_all(&work);
	let _ = std::fs::remove_file(r.out.join(format!("sally-check_{}.cut", r.stamp)));
	if made.is_ok() {
		prune(&r.out, r.keep);
	}
	made
}

fn make(r: &Request, work: &Path) -> Result<Shared, String> {
	let mut names = Names::load(&r.names);
	let mut notes = Vec::new();
	let write = |name: &str, bytes: &[u8]| std::fs::write(work.join(name), bytes).map_err(io(name));

	// the span of the recording, as it was recorded: beside the folder, never in it
	let cut_path = r.out.join(format!("sally-check_{}.cut", r.stamp));
	let cut = recording::cut(&r.recording, r.span, &cut_path).map_err(io("the recording"))?;
	if cut.reports == 0 {
		return Err("Nothing was recorded in that time: no device sent anything.".into());
	}
	let seconds = (cut.to_us - cut.from_us) as f64 / 1e6;

	// the devices' configs: each dongle's by the device it carries, and each device's a line names by its own serial
	let mut real: HashMap<String, Vec<u8>> = HashMap::new();
	for d in &cut.dongles {
		let device = cut.pairs.iter().rev().find(|(dongle, _)| dongle == d).map(|(_, device)| device.as_str());
		match device.and_then(|s| find_config(&r.config_dirs, s)).or_else(|| find_config(&r.config_dirs, d)) {
			Some(json) => {
				real.insert(d.clone(), json);
			}
			None => notes.push("A device's config was not found: its part of the recording is empty.".to_string()),
		}
	}
	for (_, device) in &cut.pairs {
		if !real.contains_key(device) {
			if let Some(json) = find_config(&r.config_dirs, device) {
				real.insert(device.clone(), json);
			}
		}
	}
	let mut shared: HashMap<String, Vec<u8>> = HashMap::new();
	for (key, json) in &real {
		if let Some(clean) = config::scrub(json, &mut names) {
			shared.insert(names.serial(key, "device"), clean);
		}
	}

	// the stations the room's files and SteamVR name, so that their ids are known wherever they stand
	let room: Vec<(String, Vec<u8>)> = r
		.room
		.iter()
		.chain(&r.settings)
		.filter_map(|p| Some((p.file_name()?.to_str()?.to_string(), std::fs::read(p).ok()?)))
		.collect();
	for (_, bytes) in &room {
		for id in room_stations(&String::from_utf8_lossy(bytes)) {
			names.station(id);
		}
	}
	let devices: Vec<SteamDevice> = r
		.devices
		.iter()
		.map(|d| SteamDevice {
			serial: names.serial(&d.serial, &d.class),
			dongle: if d.dongle.is_empty() { String::new() } else { names.serial(&d.dongle, "device") },
			..d.clone()
		})
		.collect();

	// the recording
	let did = recording::scrub(
		&cut_path,
		source(real.clone()),
		source(shared.clone()),
		&mut names,
		&work.join(session::TAP),
	)?;
	if did.blocks_written < did.blocks {
		notes.push(format!(
			"{} of the base stations' {} info blocks did not come through whole.",
			did.blocks - did.blocks_written,
			did.blocks
		));
	}
	for (key, json) in &shared {
		if let Some(name) = session::config_file(key) {
			write(&name, json)?;
		}
	}

	// what SteamVR listed and showed
	write(session::DEVICES, session::devices_text(&devices).as_bytes())?;
	if let Some(rows) = r.poses.as_ref().and_then(|p| std::fs::read(p).ok()) {
		write(session::POSES, &poses(&rows, (cut.from_us, cut.to_us), &r.devices, &mut names))?;
	}

	// the logs, then the room's files and the settings: a station that only a log names (Valve's driver by its bare
	// id, sally by its serial) is known by then, where a room's file has its id as a number
	let steamvr = r.steamvr_log.as_ref().and_then(|p| std::fs::read(p).ok()).unwrap_or_default();
	write(session::STEAMVR_LOG, &capped(&steamvr_lines(&steamvr, &mut names, &r.private)))?;
	if let Some(log) = r.sally_log.as_ref().and_then(|p| std::fs::read(p).ok()) {
		write("sally_log.txt", &text::scrub(&capped(&log), &mut names, &r.private, false))?;
	}
	for (name, bytes) in &room {
		write(name, &text::scrub(bytes, &mut names, &r.private, false))?;
	}
	write(
		session::ABOUT,
		format!(
			"sally {} check, {}\r\n\r\n\
			 report.html          what the check found (open it in a browser)\r\n\
			 tap.hidtapz          every report the devices sent in that time (sally's tap format, packed)\r\n\
			 *_config.json        each device's config, as far as sally reads it\r\n\
			 devices.txt          SteamVR's devices: class, role, dongle, model\r\n\
			 steamvr_poses.csv    what SteamVR showed for each device\r\n\
			 driver_config.json, station_fcal.txt, map_solve.txt, device_learned.txt\r\n\
			 \x20                    sally's picture of the room and what it learned of the devices\r\n\
			 sally_log.txt, sally_settings.txt    the session's log and settings\r\n\
			 steamvr_log.txt      SteamVR's log: the lines of its lighthouse driver and of sally\r\n\r\n\
			 Every serial in these files is a stand-in (LHR-00000001, LHB-00000001, ...), also the base stations'\r\n\
			 ids in the recorded light, and no file names the PC or its user.\r\n\
			 Replay: sally replay --hidtap-playback tap.hidtapz --hidtap-allow-fs-config 1 --configfile\r\n\
			 driver_config.json\r\n",
			r.version, r.when
		)
		.as_bytes(),
	)?;

	// the check, on what was recorded, named by the stand-ins
	let mut a = Analyzer::new(source(real));
	{
		let text = sally_hid::pack::open(&cut_path).map_err(io("the recording"))?;
		let options = sally_hid::tap::Options {
			playback_factor: 0.0,
			allow_fs_read: true,
			config_fetch: false,
			..sally_hid::tap::Options::default()
		};
		let mut replay = sally_hid::replay::Replay::new(&mut a, options, &cut_path.to_string_lossy(), None);
		replay.run(&mut a, &mut std::io::BufReader::with_capacity(1 << 20, text)).map_err(io("the recording"))?;
		replay.close(&mut a);
		a.finish();
	}
	for d in &mut a.devices {
		d.dongle = names.serial(&d.dongle, "device");
		if let Some(c) = &mut d.config {
			c.serial = names.serial(&c.serial, "device");
		}
	}
	for l in &mut a.log {
		*l = String::from_utf8_lossy(&text::scrub(l.as_bytes(), &mut names, &r.private, false)).into_owned();
	}
	let session = session::Session::load(work);
	let report = Report::new(&a, &session);
	let page = report.html(&format!("{}, {:.0} s", r.when, seconds));
	let said = report.text();
	write(session::REPORT, page.as_bytes())?;

	// nothing in the folder says whose it is
	recording::only_stand_ins(&work.join(session::TAP), &names)?;
	let mut files: Vec<PathBuf> =
		std::fs::read_dir(work).map_err(io("the folder"))?.flatten().map(|e| e.path()).collect();
	files.sort();
	for f in &files {
		let name = f.file_name().and_then(|n| n.to_str()).unwrap_or("").to_string();
		if name == session::TAP {
			continue;
		}
		let free = matches!(name.as_str(), "sally_log.txt" | session::STEAMVR_LOG)
			|| r.settings.iter().any(|s| s.ends_with(&name));
		let bytes = std::fs::read(f).map_err(io(&name))?;
		if !text::clean(name.as_bytes(), &names, &r.private, false) || !text::clean(&bytes, &names, &r.private, free) {
			return Err(format!("{name} was not made fit to share: nothing was written."));
		}
	}

	names.save().map_err(io("the stand-ins' table"))?;
	let zip = r.out.join(format!("sally-check_{}.zip", r.stamp));
	crate::zip::zip_folder(work, &zip).map_err(io("the zip"))?;
	let own = r.out.join(format!("sally-check_{}.html", r.stamp));
	let own_page = match page.rfind("</body>") {
		Some(at) => format!("{}{}{}", &page[..at], own_devices(&r.devices, &names), &page[at..]),
		None => page,
	};
	std::fs::write(&own, own_page).map_err(io("the report"))?;
	let bytes = std::fs::metadata(&zip).map_or(0, |m| m.len());
	Ok(Shared { zip, bytes, report: own, text: said, seconds, notes, recording: did })
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn a_rooms_stations_are_read_from_its_files() {
		let config = "\"lighthouse0\":{\n\"index\":\"0\",\n\"id\":\"123456789\",\n\"mode\":\"7\",\n}\n";
		assert_eq!(room_stations(config), [123456789]);
		let map =
			"# sally map-solve\nLH 123456789 -1.8 2.2\nLAM 123456789 2982335460 1.1e9 -3\nFCAL 7 0 1\nANCHOR 8 0.5 1\n";
		assert_eq!(room_stations(map), [123456789, 123456789, 2982335460, 7, 8]);
	}

	#[test]
	fn a_long_log_keeps_its_start_and_its_end() {
		let line = "Info: a line of the log that says something\n";
		let log = line.repeat((LOG_HEAD + LOG_TAIL) / line.len() + 5_000);
		let out = capped(log.as_bytes());
		assert!(out.len() < LOG_HEAD + LOG_TAIL + 100 && out.len() > LOG_HEAD + LOG_TAIL - 200);
		let text = String::from_utf8(out).unwrap();
		assert!(text.lines().all(|l| l == line.trim_end() || l.starts_with("[... ")), "whole lines only");
		assert_eq!(capped(b"short"), b"short");
	}

	#[test]
	fn steamvrs_log_keeps_two_drivers_lines() {
		let mut n = Names::default();
		let p = Private { user: "Robin".into(), home: "C:\\Users\\Robin".into(), ..Private::default() };
		let log =
			"Sat Nov 07 2026 15:03:22.318 [Info] - lighthouse: LHR-1A2B3C4D: Connected to receiver 0A1B2C3D4E-6YX\r\n\
		           Sat Nov 07 2026 15:03:22.400 [Info] - vrlink: something about C:\\Users\\Robin\r\n\
		           Sat Nov 07 2026 15:03:23.000 [Info] - sally: sally: driver 0.22.0 starting\r\n\
		           Sat Nov 07 2026 15:03:24.000 [Info] - lighthouse: LHR-1A2B3C4D C: ----- CALIBRATED base 7C8D9E0F -----\r\n\
		           Sat Nov 07 2026 15:03:25.000 [Info] - Loaded C:\\Users\\Robin\\x - lighthouse: no\r\n";
		let out = String::from_utf8(steamvr_lines(log.as_bytes(), &mut n, &p)).unwrap();
		assert_eq!(
			out,
			"Sat Nov 07 2026 15:03:22.318 [Info] - lighthouse: LHR-00000001: Connected to receiver 0000000001-6YX\n\
			 Sat Nov 07 2026 15:03:23.000 [Info] - sally: sally: driver 0.22.0 starting\n\
			 Sat Nov 07 2026 15:03:24.000 [Info] - lighthouse: LHR-00000001 C: ----- CALIBRATED base 00000001 -----\n"
		);
	}

	#[test]
	fn poses_are_the_spans_by_stand_in() {
		let mut n = Names::default();
		let devices = [SteamDevice { serial: "QUEST-1".into(), class: "hmd".into(), ..SteamDevice::default() }];
		let rows = "t_us,serial,result\n100,LHR-1A2B3C4D,200,1\n200,QUEST-1,200,1\n300,LHR-1A2B3C4D,200,1\n400,X,1\n";
		let out = String::from_utf8(poses(rows.as_bytes(), (150, 300), &devices, &mut n)).unwrap();
		assert_eq!(out, format!("{}\n200,hmd-1,200,1\n300,LHR-00000001,200,1\n", session::POSES_HEADER));
	}
}
