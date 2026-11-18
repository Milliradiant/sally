//! The replay viewer's data (docs/sally_design.md 9.3): a session's recordings (the tap the live runtime recorded,
//! the driver's pose log, the runtime's log), sally run on the tap again, and all of it on one timeline: the run's
//! time, the tap's first report its zero.
//!
//! - **The replay** is the live runtime (sally_settings.txt's `driver.runtime_exe`) with the live tracking arguments
//!   (the rest of that file's lines, less the live-only ones: the tap, the ports, the recording) on copies of the map,
//!   the accelerometer calibration and the stations' config, in replays\<session> beside the panel, with the device
//!   configs the session had (the tap's `# cache` lines, else its runtime log's "device <dongle> is KNn" and the
//!   config it wrote, else the live caches). It writes run.rec (POSE, CONFIG, LH_POSE) and run.log (the tracker's
//!   events).
//! - **Live** is the pose log: what the driver sent SteamVR for each device (sally's pose, in Valve's head frame) and
//!   Valve's own pose, both in SteamVR's world (Y up, before Space Calibrator). Its clock is the tap's (the PC's
//!   performance counter): run time = t - the tap's first report.
//! - **The frame**: Valve's world into the runtime's (Z up, where the stations are) by a rigid fit of what the driver
//!   sent to the replay's poses, one per 30 s (the driver's own fit can move), and each device's head offset (the
//!   replay's head frame to Valve's: 180 deg on a Tundra) from the same pairs.
//! - **Events**: the tracker's lines ("KN4 track warm restart at 536.951 s: ...") from the live run's log and the
//!   replay's, their time the device's clock (its timecodes, unwrapped from the replay's POSE lines) put on the run's.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Instant;

use sally_math::math3d::{compose, fit_rigid_robust, inverse, len, qconj, qmul, qnorm, qrot, xform, Pose, Q4, V3};

use crate::models::json_string;

/// A device's clock: 48 MHz, 32 bits (89.48 s a wrap)
const TIMEBASE: f64 = 48e6;
const WRAP_S: f64 = 4294967296.0 / TIMEBASE;
/// A gap in a track: samples further apart
pub const GAP_S: f32 = 0.25;

/// A recording line more than this off the recording's clock (s) is one two threads wrote into each other (load)
const LINE_CLOCK_S: f64 = 60.0;
/// The frame's windows
const FRAME_WINDOW_S: f64 = 10.0;
/// "off" in the error summary: over this far from Valve (m)
pub const OFF_M: f32 = 0.03;

/// Where things are: the project folder, its logs, the driver's settings and what they say about the live runtime, and
/// where replays go
#[derive(Clone, Debug, Default)]
pub struct Paths {
	pub project: PathBuf,
	pub logs: PathBuf,
	pub driver_settings: PathBuf,
	pub runtime_exe: PathBuf,
	pub runtime_args: Vec<String>,
	/// the live runtime's folder: the dongles' config caches and the devices' LHR-*_config.json
	pub devcfg: PathBuf,
	/// SteamVR's own copies of the device configs (<steam>\config\lighthouse\<serial>\config.json)
	pub steam_lighthouse: Option<PathBuf>,
	pub replays: PathBuf,
	/// sally-check's recordings (tools\sally-check\sally-check recordings\sally-check_<stamp>)
	pub checks: PathBuf,
}

impl Paths {
	/// From the panel's folder (tools\sally_panel: the project is two up; or the installer's folder, the driver's
	/// folder beside the panel) or sally-panel.cfg's `replay_project`
	pub fn discover(panel_dir: &Path, project_override: &str) -> Paths {
		if project_override.is_empty() && panel_dir.join("driver").join("driver.vrdrivermanifest").is_file() {
			return Paths::installed(panel_dir);
		}
		let project = if !project_override.is_empty() {
			PathBuf::from(project_override)
		} else {
			let up = panel_dir.parent().and_then(Path::parent).map(Path::to_path_buf);
			match up.filter(|u| u.join("logs").is_dir()) {
				Some(u) => u,
				None => panel_dir.to_path_buf(),
			}
		};
		let driver_settings =
			project.join("steamvr_driver").join("sally").join("bin").join("win64").join("sally_settings.txt");
		let mut p = Paths {
			logs: project.join("logs"),
			devcfg: project.join("survive").join("hidtap").join("devcfg"),
			runtime_exe: project.join("survive").join("hidtap").join(if cfg!(windows) { "sally.exe" } else { "sally" }),
			replays: panel_dir.join("replays"),
			checks: project.join("tools").join("sally-check").join("sally-check recordings"),
			project,
			driver_settings,
			..Paths::default()
		};
		if let Ok(b) = std::fs::read(&p.driver_settings) {
			p.read_driver_settings(&String::from_utf8_lossy(&b));
		}
		p.steam_lighthouse = crate::models::openvrpaths()
			.and_then(|t| crate::models::json_strings(&t, "config").into_iter().next())
			.map(|c| PathBuf::from(c).join("lighthouse"))
			.filter(|d| d.is_dir());
		p
	}

	/// The installer's folder: sally.exe, the panel and the driver's folder side by side, data\ and logs\ beside them
	fn installed(root: &Path) -> Paths {
		let mut p = Paths {
			project: root.to_path_buf(),
			logs: root.join("logs"),
			driver_settings: root.join("driver").join("bin").join("win64").join("sally_settings.txt"),
			devcfg: root.join("data").join("devices"),
			runtime_exe: root.join(if cfg!(windows) { "sally.exe" } else { "sally" }),
			replays: root.join("replays"),
			checks: root.join("sally-check recordings"),
			..Paths::default()
		};
		if let Ok(b) = std::fs::read(&p.driver_settings) {
			p.read_driver_settings(&String::from_utf8_lossy(&b));
		}
		p.steam_lighthouse = crate::models::openvrpaths()
			.and_then(|t| crate::models::json_strings(&t, "config").into_iter().next())
			.map(|c| PathBuf::from(c).join("lighthouse"))
			.filter(|d| d.is_dir());
		p
	}

	/// sally_settings.txt as the driver reads it: '#' and blank lines skipped, "driver." options, the rest the
	/// runtime's arguments one a line
	pub fn read_driver_settings(&mut self, text: &str) {
		self.runtime_args.clear();
		for line in text.lines() {
			let s = line.trim();
			if s.is_empty() || s.starts_with('#') {
				continue;
			}
			if let Some(o) = s.strip_prefix("driver.") {
				let Some((k, v)) = o.split_once('=') else { continue };
				let v = v.trim();
				match k.trim() {
					"runtime_exe" => self.runtime_exe = PathBuf::from(v),
					"runtime_cwd" => self.devcfg = PathBuf::from(v),
					"runtime_log" => {
						if let Some(d) = Path::new(v).parent() {
							self.logs = d.to_path_buf();
						}
					}
					_ => {}
				}
				continue;
			}
			// the runtime's subcommand (`run`, the first argument for sally.exe) is the replay's to choose
			if self.runtime_args.is_empty() && s == "run" {
				continue;
			}
			self.runtime_args.push(s.to_string());
		}
	}

	/// Where device configs are looked for by serial
	pub fn config_dirs(&self) -> Vec<PathBuf> {
		let mut v = vec![self.devcfg.clone()];
		if let Ok(rd) = std::fs::read_dir(&self.logs) {
			let mut snaps: Vec<PathBuf> = rd
				.flatten()
				.map(|e| e.path())
				.filter(|p| {
					p.is_dir() && p.file_name().and_then(|f| f.to_str()).is_some_and(|f| f.starts_with("steamcfg_"))
				})
				.collect();
			snaps.sort();
			snaps.reverse();
			v.extend(snaps);
		}
		v.extend(self.steam_lighthouse.clone());
		v
	}
}

/// A recorded session: the tap, and the files of the same run
#[derive(Clone, Debug, Default)]
pub struct Session {
	/// "20261107_123001"
	pub stamp: String,
	pub tap: PathBuf,
	pub bytes: u64,
	/// the tap's first and last report on the PC's clock (s)
	pub t0: f64,
	pub t1: f64,
	pub solver: Option<PathBuf>,
	pub poselog: Option<PathBuf>,
	/// the replay's done.txt, when one has finished
	pub replayed: Option<String>,
	/// its recording's size (they are some 300 MB an hour)
	pub replay_bytes: u64,
	/// a sally-check recording's folder (its tap, its device configs, SteamVR's poses, maybe its room's stations)
	pub check: Option<PathBuf>,
}

impl Session {
	pub fn duration(&self) -> f64 {
		(self.t1 - self.t0).max(0.0)
	}
	/// What names it (the picker's, `--replay-check`'s, its replay folder's): the stamp, "check_<stamp>" for a
	/// sally-check recording
	pub fn key(&self) -> String {
		match self.check {
			Some(_) => format!("check_{}", self.stamp),
			None => self.stamp.clone(),
		}
	}
	pub fn replay_dir(&self, paths: &Paths) -> PathBuf {
		paths.replays.join(self.key())
	}
}

/// A tap line's time: "R <us> ..." (s)
fn r_time(line: &[u8]) -> Option<f64> {
	let l = line.strip_prefix(b"R ")?;
	let end = l.iter().position(|&c| c == b' ')?;
	let us: f64 = std::str::from_utf8(&l[..end]).ok()?.parse().ok()?;
	Some(us / 1e6)
}

/// The first and last report times of a tap (its first 64 KB, its last 256 KB; a packed one's as its blocks give them)
pub fn tap_span(path: &Path) -> Option<(f64, f64)> {
	let mut f = File::open(path).ok()?;
	if sally_hid::pack::packed(&mut f).ok()? {
		let (t0, t1) = sally_hid::pack::span(&mut f).ok()??;
		return Some((t0 as f64 / 1e6, t1 as f64 / 1e6));
	}
	let size = f.metadata().ok()?.len();
	let mut head = vec![0u8; 65536.min(size as usize)];
	f.read_exact(&mut head).ok()?;
	let t0 = head.split(|&c| c == b'\n').find_map(r_time)?;
	let tail_n = 262144.min(size);
	f.seek(SeekFrom::Start(size - tail_n)).ok()?;
	let mut tail = Vec::with_capacity(tail_n as usize);
	f.read_to_end(&mut tail).ok()?;
	let t1 = tail.split(|&c| c == b'\n').rev().skip(1).find_map(r_time).unwrap_or(t0);
	Some((t0, t1.max(t0)))
}

/// A pose log's first row's time
fn poselog_start(path: &Path) -> Option<f64> {
	let f = File::open(path).ok()?;
	let mut r = BufReader::new(f);
	let mut line = String::new();
	r.read_line(&mut line).ok()?;
	line.clear();
	r.read_line(&mut line).ok()?;
	line.split(',').next()?.parse().ok()
}

/// The sessions in the logs folder, newest first: each sallytap_<stamp>.hidtap (or .hidtapz, the recording packed)
/// with its runtime log, and the pose log whose first row falls in the tap's span
pub fn sessions(paths: &Paths) -> Vec<Session> {
	let Ok(rd) = std::fs::read_dir(&paths.logs) else { return Vec::new() };
	let mut taps: Vec<(String, PathBuf)> = Vec::new();
	let mut plogs = Vec::new();
	for e in rd.flatten() {
		let name = e.file_name().to_string_lossy().into_owned();
		if let Some(stamp) = name.strip_prefix("sallytap_").and_then(sally_hid::pack::stem) {
			if stamp.len() == 15 && stamp.as_bytes()[8] == b'_' {
				taps.push((stamp.to_string(), e.path()));
			}
		} else if name.starts_with("sally_poses_") && name.ends_with(".csv") {
			plogs.push(e.path());
		}
	}
	// a session there in both forms is listed once, as its text
	taps.sort();
	taps.dedup_by(|b, a| a.0 == b.0);
	let plog_t: Vec<(f64, PathBuf)> = plogs.into_iter().filter_map(|p| poselog_start(&p).map(|t| (t, p))).collect();
	let mut out: Vec<Session> = taps
		.into_iter()
		.filter_map(|(stamp, tap)| {
			let (t0, t1) = tap_span(&tap)?;
			let bytes = std::fs::metadata(&tap).map(|m| m.len()).unwrap_or(0);
			let solver = Some(paths.logs.join(format!("sallytap_{stamp}_solver.txt"))).filter(|p| p.is_file());
			let poselog = plog_t.iter().find(|(t, _)| *t >= t0 - 60.0 && *t <= t1).map(|(_, p)| p.clone());
			let mut s = Session { stamp, tap, bytes, t0, t1, solver, poselog, ..Session::default() };
			let dir = s.replay_dir(paths);
			s.replayed = std::fs::read_to_string(dir.join("done.txt")).ok().map(|t| t.trim().to_string());
			s.replay_bytes = std::fs::metadata(dir.join("run.rec")).map(|m| m.len()).unwrap_or(0);
			Some(s)
		})
		.collect();
	// sally-check's recordings: the tap, SteamVR's poses beside it
	if let Ok(rd) = std::fs::read_dir(&paths.checks) {
		for e in rd.flatten() {
			let name = e.file_name().to_string_lossy().into_owned();
			let Some(stamp) = name.strip_prefix("sally-check_").filter(|s| s.len() == 15) else { continue };
			// packed in a session the panel's Check shared, text in an older sally-check's own
			let Some(tap) = sally_check::session::tap(&e.path()) else { continue };
			let Some((t0, t1)) = tap_span(&tap) else { continue };
			let bytes = std::fs::metadata(&tap).map(|m| m.len()).unwrap_or(0);
			let poselog = Some(e.path().join("steamvr_poses.csv")).filter(|p| p.is_file());
			let mut s = Session {
				stamp: stamp.into(),
				tap,
				bytes,
				t0,
				t1,
				poselog,
				check: Some(e.path()),
				..Session::default()
			};
			let dir = s.replay_dir(paths);
			s.replayed = std::fs::read_to_string(dir.join("done.txt")).ok().map(|t| t.trim().to_string());
			s.replay_bytes = std::fs::metadata(dir.join("run.rec")).map(|m| m.len()).unwrap_or(0);
			out.push(s);
		}
	}
	out.sort_by(|a, b| b.stamp.cmp(&a.stamp));
	out
}

/// A session's stamp: "20261107_123001"
fn is_stamp(s: &str) -> bool {
	s.len() == 15 && s.bytes().enumerate().all(|(i, c)| if i == 8 { c == b'_' } else { c.is_ascii_digit() })
}

/// The replays whose session is gone, removed; how many went. A replay is made from its session's recording and
/// reached through it alone (the picker lists the recordings), so once the recording is deleted (the driver keeps
/// the newest few sessions' files) its replay is hundreds of megabytes that nothing shows any more. A replay is a
/// folder in `replays` named as a session's key (`Session::key`) with a replay's args.txt in it: nothing named or
/// filled any other way is touched, and nothing at all while the recordings cannot be listed.
pub fn prune(paths: &Paths) -> usize {
	let Ok(logs) = std::fs::read_dir(&paths.logs) else { return 0 };
	let mut kept: Vec<String> = logs
		.flatten()
		.filter_map(|e| {
			let name = e.file_name().to_string_lossy().into_owned();
			let stamp = name.strip_prefix("sallytap_").and_then(sally_hid::pack::stem)?;
			is_stamp(stamp).then(|| stamp.to_string())
		})
		.collect();
	// sally-check's recordings: none where their folder is not there, unknown (their replays left) where it cannot be read
	let checks_known = match std::fs::read_dir(&paths.checks) {
		Ok(rd) => {
			kept.extend(rd.flatten().filter_map(|e| {
				let name = e.file_name().to_string_lossy().into_owned();
				name.strip_prefix("sally-check_").filter(|s| is_stamp(s)).map(|s| format!("check_{s}"))
			}));
			true
		}
		Err(e) => e.kind() == std::io::ErrorKind::NotFound,
	};
	let Ok(rd) = std::fs::read_dir(&paths.replays) else { return 0 };
	let mut gone = 0;
	for e in rd.flatten() {
		let name = e.file_name().to_string_lossy().into_owned();
		let check = name.strip_prefix("check_");
		if !is_stamp(check.unwrap_or(&name)) || (check.is_some() && !checks_known) || kept.contains(&name) {
			continue;
		}
		let dir = e.path();
		if dir.join("args.txt").is_file() && std::fs::remove_dir_all(&dir).is_ok() {
			gone += 1;
		}
	}
	gone
}

/// A sally-check recording's stations (its devices.txt: "LHB-15B80A74 ... station"), as the ids a config has them
fn check_stations(folder: &Path) -> Vec<u32> {
	let text = std::fs::read_to_string(folder.join("devices.txt")).unwrap_or_default();
	text.lines()
		.filter(|l| l.split('\t').nth(1) == Some("station"))
		.filter_map(|l| u32::from_str_radix(l.split('\t').next()?.strip_prefix("LHB-")?, 16).ok())
		.collect()
}

/// The stations of a driver config (its "id"s)
fn config_stations(path: &Path) -> Vec<u32> {
	let text = std::fs::read_to_string(path).unwrap_or_default();
	crate::models::json_strings(&text, "id").iter().filter_map(|s| s.parse().ok()).collect()
}

/// Options of this room the replay of another room's recording leaves out: its stations' factory corrections and its
/// gravity's tilt (under the tilt's names of now and its earlier ones)
const THIS_ROOM: &[&str] = &[
	"--station-fcal-file",
	"--track-gravity-tilt",
	"--track-gravity-tilt-az",
	"--core2-gravity-tilt",
	"--core2-gravity-tilt-az",
];

/// The live-only arguments the replay leaves out (with their values)
const LIVE_ONLY: &[&str] = &[
	"--hidtap",
	"--hidtap-config-fetch",
	"--hidtap-record",
	"--pose-udp",
	"--runtime-port",
	"--serialize-device-config",
	"--record",
	"--configfile",
	"--kalman-acc-cal-file",
	"--map-solve-file",
	"--map-solve-test-load",
	"--device-learned-file",
	"--hidtap-playback",
	"--playback-factor",
	"--stop-after",
];

/// The live arguments as the replay takes them: pairs of option and value (a line that is no option is the value of
/// the one before), the live-only ones left out
/// sally (`sally run ...`) rather than an older runtime named survive-cli, which took survive-cli's command line as it
/// was
pub fn takes_subcommand(exe: &Path) -> bool {
	let text = exe.to_string_lossy();
	let name = text.rsplit(['\\', '/']).next().unwrap_or_default();
	let stem = name.rsplit_once('.').map_or(name, |(s, _)| s);
	!stem.eq_ignore_ascii_case("survive-cli")
}

pub fn replay_args(live: &[String]) -> Vec<String> {
	let mut out = Vec::new();
	let mut i = 0;
	while i < live.len() {
		let a = &live[i];
		let has_value = i + 1 < live.len() && !live[i + 1].starts_with("--");
		let n = if has_value { 2 } else { 1 };
		if !LIVE_ONLY.contains(&a.as_str()) {
			out.extend(live[i..i + n].iter().cloned());
		}
		i += n;
	}
	out
}

/// The device configs a replay of `tap` needs, as (file name, source): the tap's "# cache <dongle> <serial>" lines and
/// the devices its A lines name, else the runtime log's "device <dongle> is KNn" with the LHR-<serial>_config.json
/// the run wrote for KNn
pub fn configs_needed(tap_head: &[u8], solver: Option<&str>) -> Vec<(String, String)> {
	let mut out: Vec<(String, String)> = Vec::new();
	for l in tap_head.split(|&c| c == b'\n') {
		let l = String::from_utf8_lossy(l);
		let w: Vec<&str> = l.split_ascii_whitespace().collect();
		if w.len() == 4 && w[0] == "#" && w[1] == "cache" {
			out.push((format!("{}_config.json", w[2]), w[3].to_string()));
		} else if w.len() == 4 && w[0] == "A" && w[3] != "-" {
			out.push((format!("{}_config.json", w[3]), w[3].to_string()));
		}
	}
	if !out.is_empty() {
		out.dedup();
		return out;
	}
	if let Some(log) = solver {
		// "Adding tracked object KN0", "Wrote out 8075 bytes to LHR-2D779713_config.json", "device 4BCAD73D0E-6YX is KN0"
		let mut serial_of: HashMap<String, String> = HashMap::new();
		let mut last_obj = String::new();
		for l in log.lines() {
			if let Some(r) = l.split("Adding tracked object ").nth(1) {
				last_obj = r.split_ascii_whitespace().next().unwrap_or("").to_string();
			} else if let Some(r) = l.split(" bytes to ").nth(1) {
				if let Some(s) = r.trim().strip_suffix("_config.json").filter(|s| s.starts_with("LHR-")) {
					if !last_obj.is_empty() {
						serial_of.entry(last_obj.clone()).or_insert_with(|| s.to_string());
					}
				}
			} else if let Some(r) = l.split("hidtap: device ").nth(1) {
				let w: Vec<&str> = r.split_ascii_whitespace().collect();
				if w.len() >= 3 && w[1] == "is" {
					if let Some(s) = serial_of.get(w[2]) {
						out.push((format!("{}_config.json", w[0]), s.clone()));
					}
				}
			}
		}
	}
	out.dedup();
	out
}

/// A device's config by serial in the folders (LHR-<serial>_config.json, or SteamVR's <serial>\config.json)
fn config_by_serial(dirs: &[PathBuf], serial: &str) -> Option<Vec<u8>> {
	for d in dirs {
		for p in [d.join(format!("{serial}_config.json")), d.join(serial.to_lowercase()).join("config.json")] {
			if let Ok(b) = std::fs::read(&p) {
				return Some(b);
			}
		}
	}
	None
}

/// A replay running
pub struct Job {
	child: Child,
	pub stamp: String,
	pub dir: PathBuf,
	pub started: Instant,
	pub dur: f64,
	/// the run time the replay has reached, of `dur`
	pub at: f64,
	last_check: Instant,
}

/// Starts sally on a session's tap in its replay folder (emptied first)
pub fn start(paths: &Paths, s: &Session) -> Result<Job, String> {
	if !paths.runtime_exe.is_file() {
		return Err(format!("no runtime at {}", paths.runtime_exe.display()));
	}
	let dir = s.replay_dir(paths);
	if dir.is_dir() {
		for f in [
			"run.rec",
			"run.rec.tmp",
			"light.bin",
			"run.log",
			"run.err",
			"done.txt",
			"map.txt",
			"acc.txt",
			"check.txt",
			"learned.txt",
		] {
			let _ = std::fs::remove_file(dir.join(f));
		}
	}
	std::fs::create_dir_all(&dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
	let live = &paths.runtime_args;
	let value_of = |opt: &str| live.iter().position(|a| a == opt).and_then(|i| live.get(i + 1)).cloned();
	// another room's recording: its own stations (a driver_config.json in its folder), none of this room's files
	let check_cfg = s.check.as_ref().map(|c| c.join("driver_config.json")).filter(|p| p.is_file());
	let other_room = match &s.check {
		Some(c) => {
			let theirs = check_stations(c);
			let ours = value_of("--configfile").map(|p| config_stations(Path::new(&p))).unwrap_or_default();
			check_cfg.is_some() || (!theirs.is_empty() && !theirs.iter().any(|t| ours.contains(t)))
		}
		None => false,
	};
	if other_room && check_cfg.is_none() {
		let theirs: Vec<String> =
			s.check.as_deref().map(check_stations).unwrap_or_default().iter().map(|i| format!("LHB-{i:08X}")).collect();
		return Err(format!(
			"this recording is from another room (stations {}): sally cannot place a room's stations yet; a \
			 driver_config.json with them in its folder replays it",
			theirs.join(", ")
		));
	}
	let mut notes: Vec<String> = Vec::new();
	// the device configs as the session had them
	let mut head = Vec::new();
	if let Ok(f) = sally_hid::pack::open(&s.tap) {
		// (what was read before an error still counts)
		let _ = f.take(1 << 20).read_to_end(&mut head);
	}
	let solver =
		s.solver.as_ref().and_then(|p| std::fs::read(p).ok()).map(|b| String::from_utf8_lossy(&b).into_owned());
	let need = if s.check.is_some() { Vec::new() } else { configs_needed(&head, solver.as_deref()) };
	let dirs = paths.config_dirs();
	let mut missing = Vec::new();
	if let Some(c) = &s.check {
		// sally-check keeps each device's config by the dongle it is on, as the caches are named
		for e in std::fs::read_dir(c).map_err(|e| format!("cannot read {}: {e}", c.display()))?.flatten() {
			let n = e.file_name().to_string_lossy().into_owned();
			if n.ends_with("_config.json") && n != "driver_config.json" {
				std::fs::copy(e.path(), dir.join(&n)).map_err(|e| format!("cannot copy {n}: {e}"))?;
			}
		}
	}
	for (file, serial) in &need {
		match config_by_serial(&dirs, serial) {
			Some(b) => {
				std::fs::write(dir.join(file), b).map_err(|e| format!("cannot write {file}: {e}"))?;
			}
			None => missing.push(serial.clone()),
		}
	}
	if need.is_empty() && s.check.is_none() {
		// nothing says which device was on which dongle: the live caches
		notes.push(
			"the device configs are today's dongle caches: nothing in the session says which device was on which dongle"
				.into(),
		);
		if let Ok(rd) = std::fs::read_dir(&paths.devcfg) {
			for e in rd.flatten() {
				let n = e.file_name().to_string_lossy().into_owned();
				if n.ends_with("_config.json") && !n.starts_with("LHR-") && !n.starts_with("KN") {
					let _ = std::fs::copy(e.path(), dir.join(&n));
				}
			}
		}
	}
	// the live files the run reads, copied: a replay never writes the live map or calibration. Another room's: its
	// stations, the map solve starting from them
	let mut args: Vec<String> = Vec::new();
	let cfg = if other_room { check_cfg.map(|p| p.to_string_lossy().into_owned()) } else { value_of("--configfile") };
	if let Some(cfg) = cfg {
		std::fs::copy(&cfg, dir.join("driver_config.json")).map_err(|e| format!("cannot copy {cfg}: {e}"))?;
		// "./": a bare name would be a file in libsurvive's config folder, not the run's
		args.extend(["--configfile".into(), "./driver_config.json".into()]);
	}
	let map_file = value_of("--map-solve-file");
	if let Some(map) = &map_file {
		if !other_room {
			let _ = std::fs::copy(map, dir.join("map.txt"));
		}
		args.extend(["--map-solve-file".into(), "map.txt".into()]);
	}
	// what the tracker learned of the devices: the replay starts from the live history and writes into its copy (a
	// replay is no session of the devices; another room's recording has its own devices, and no history of ours)
	if let Some(learned) = value_of("--device-learned-file") {
		if !other_room {
			let _ = std::fs::copy(&learned, dir.join("learned.txt"));
		}
		args.extend(["--device-learned-file".into(), "learned.txt".into()]);
	}
	if let Some(acc) = value_of("--kalman-acc-cal-file") {
		if !other_room {
			let _ = std::fs::copy(&acc, dir.join("acc.txt"));
		}
		args.extend(["--kalman-acc-cal-file".into(), "acc.txt".into()]);
	}
	let mut rest = replay_args(live);
	if other_room {
		notes.push("another room's recording: its stations from its folder's driver_config.json".into());
		let mut kept = Vec::new();
		let mut i = 0;
		while i < rest.len() {
			let n = if i + 1 < rest.len() && !rest[i + 1].starts_with("--") { 2 } else { 1 };
			if !THIS_ROOM.contains(&rest[i].as_str()) {
				kept.extend(rest[i..i + n].iter().cloned());
			}
			i += n;
		}
		rest = kept;
	}
	args.extend(rest);
	args.extend(
		[
			"--hidtap-playback",
			&s.tap.to_string_lossy(),
			"--hidtap-allow-fs-config",
			"1",
			"--playback-factor",
			"0",
			"--htcvive",
			"0",
			"--record",
			"run.rec",
			"--record-imu",
			"0",
			"--record-rawlight",
			"0",
			// the light per sensor (load turns it into light.bin)
			"--record-angle",
			"1",
		]
		.map(String::from),
	);
	if map_file.is_some() && !other_room {
		// the stations where the saved map has them, as the live run had them: the map solve still runs (its lines in
		// the log) but moves nothing. Re-solved in a replay, the map can drift (stations 80-100 mm and 30-40 mrad from
		// where the live run kept them, 13-35 % of the hits rejected against the live run's 0.2-1.3 %). Last, so it
		// outranks any live setting
		args.extend(["--map-solve-max-move", "0", "--map-solve-max-rot", "0"].map(String::from));
		notes.push("the map as saved, not moved by the replay's own map solve".into());
	}
	let mut note = format!("runtime {}\n", paths.runtime_exe.display());
	if takes_subcommand(&paths.runtime_exe) {
		note += "run\n";
	}
	for a in &args {
		note += a;
		note.push('\n');
	}
	if !missing.is_empty() {
		notes.push(format!("no config found for {}", missing.join(" ")));
	}
	for n in &notes {
		note += &format!("# {n}\n");
	}
	let _ = std::fs::write(dir.join("args.txt"), note);
	let log = File::create(dir.join("run.log")).map_err(|e| e.to_string())?;
	let err = File::create(dir.join("run.err")).map_err(|e| e.to_string())?;
	let mut cmd = Command::new(&paths.runtime_exe);
	if takes_subcommand(&paths.runtime_exe) {
		cmd.arg("run");
	}
	cmd.args(&args).current_dir(&dir).stdin(Stdio::null()).stdout(log).stderr(err);
	#[cfg(windows)]
	{
		use std::os::windows::process::CommandExt;
		// CREATE_NO_WINDOW: no console flashing up, BELOW_NORMAL_PRIORITY_CLASS: SteamVR first
		cmd.creation_flags(0x0800_0000 | 0x0000_4000);
	}
	let child = cmd.spawn().map_err(|e| format!("cannot start {}: {e}", paths.runtime_exe.display()))?;
	Ok(Job {
		child,
		stamp: s.stamp.clone(),
		dir,
		started: Instant::now(),
		dur: s.duration(),
		at: 0.0,
		last_check: Instant::now(),
	})
}

impl Job {
	/// Ok(true) once the run has ended well, Err when it failed; its progress in `at`
	pub fn poll(&mut self) -> Result<bool, String> {
		if self.last_check.elapsed().as_secs_f64() > 0.5 {
			self.last_check = Instant::now();
			if let Some(t) = last_line_time(&self.dir.join("run.rec")) {
				self.at = t;
			}
		}
		match self.child.try_wait() {
			Ok(None) => Ok(false),
			Ok(Some(st)) => {
				let secs = self.started.elapsed().as_secs_f64();
				let ok = st.success();
				let _ = std::fs::write(
					self.dir.join("done.txt"),
					format!("exit {} after {:.0} s\n", st.code().unwrap_or(-1), secs),
				);
				if ok && self.dir.join("run.rec").is_file() {
					Ok(true)
				} else {
					let err = std::fs::read(self.dir.join("run.err")).unwrap_or_default();
					let err = String::from_utf8_lossy(&err);
					let last = err.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("").to_string();
					Err(format!("the replay ended with exit {}: {last}", st.code().unwrap_or(-1)))
				}
			}
			Err(e) => Err(e.to_string()),
		}
	}

	pub fn cancel(&mut self) {
		let _ = self.child.kill();
		let _ = self.child.wait();
	}
}

/// A replay the panel let go of (another session, the panel closed) is stopped, not left running
impl Drop for Job {
	fn drop(&mut self) {
		if matches!(self.child.try_wait(), Ok(None)) {
			self.cancel();
		}
	}
}

/// The run time of a recording's last line
fn last_line_time(path: &Path) -> Option<f64> {
	let mut f = File::open(path).ok()?;
	let size = f.metadata().ok()?.len();
	let n = 4096.min(size);
	f.seek(SeekFrom::Start(size - n)).ok()?;
	let mut b = Vec::new();
	f.read_to_end(&mut b).ok()?;
	let s = String::from_utf8_lossy(&b);
	s.lines().rev().skip(1).find_map(|l| l.split(' ').next()?.parse::<f64>().ok())
}

/// A pose stream on the run's time: samples in time order, f32 to keep an hour of nine devices in memory
#[derive(Clone, Debug, Default)]
pub struct Track {
	pub t: Vec<f32>,
	pub p: Vec<[f32; 3]>,
	pub q: Vec<[f32; 4]>,
}

fn v3(a: [f32; 3]) -> V3 {
	V3::new(a[0] as f64, a[1] as f64, a[2] as f64)
}
fn q4(a: [f32; 4]) -> Q4 {
	Q4::new(a[0] as f64, a[1] as f64, a[2] as f64, a[3] as f64)
}

impl Track {
	pub fn push(&mut self, t: f64, p: &Pose) {
		self.t.push(t as f32);
		self.p.push([p.p.x as f32, p.p.y as f32, p.p.z as f32]);
		self.q.push([p.q.w as f32, p.q.x as f32, p.q.y as f32, p.q.z as f32]);
	}

	pub fn len(&self) -> usize {
		self.t.len()
	}

	pub fn is_empty(&self) -> bool {
		self.t.is_empty()
	}

	/// The pose at `t`, between the samples around it; None outside the track or in a gap
	pub fn at(&self, t: f64) -> Option<Pose> {
		let t = t as f32;
		let i = self.t.partition_point(|&x| x <= t);
		if i == 0 || i >= self.t.len() {
			// the last sample still holds a moment after it
			if i == self.t.len() && i > 0 && t - self.t[i - 1] < 0.02 {
				return Some(Pose { p: v3(self.p[i - 1]), q: q4(self.q[i - 1]) });
			}
			return None;
		}
		let (a, b) = (i - 1, i);
		let dt = self.t[b] - self.t[a];
		if dt > GAP_S {
			return None;
		}
		let k = if dt > 0.0 { ((t - self.t[a]) / dt) as f64 } else { 0.0 };
		let (pa, pb) = (v3(self.p[a]), v3(self.p[b]));
		let (qa, mut qb) = (q4(self.q[a]), q4(self.q[b]));
		if qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z < 0.0 {
			qb = Q4::new(-qb.w, -qb.x, -qb.y, -qb.z);
		}
		let q = qnorm(Q4::new(
			qa.w + (qb.w - qa.w) * k,
			qa.x + (qb.x - qa.x) * k,
			qa.y + (qb.y - qa.y) * k,
			qa.z + (qb.z - qa.z) * k,
		));
		Some(Pose { p: pa + (pb - pa) * k, q })
	}

	/// The samples from t0 to t1 (positions), for a trail
	pub fn span(&self, t0: f64, t1: f64) -> (usize, usize) {
		let a = self.t.partition_point(|&x| (x as f64) < t0);
		let b = self.t.partition_point(|&x| (x as f64) <= t1);
		(a, b)
	}

	pub fn pos(&self, i: usize) -> V3 {
		v3(self.p[i])
	}

	/// Its gaps (from, to)
	pub fn gaps(&self) -> Vec<(f32, f32)> {
		self.t.windows(2).filter(|w| w[1] - w[0] > GAP_S).map(|w| (w[0], w[1])).collect()
	}
}

/// A line from the tracker: the device, what ("warm restart"), its run time, the whole text; from the live run's log
/// or the replay's
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
	pub t: f32,
	pub dev: String,
	pub what: String,
	pub text: String,
	pub live: bool,
	/// 0 fine (a start), 1 worth a look, 2 a restart or worse
	pub level: u8,
}

/// "KN4 track warm restart at 536.951 s: SOLVED ..." -> (KN4, warm restart, 536.951, the text after "Info: "). In an
/// older log the tracker's lines say "core2" where they now say "track": read the same, and the text given as it would
/// be written now
pub fn parse_event(line: &str) -> Option<(String, String, f64, String)> {
	let l = line.trim_end();
	let l = l.strip_prefix("Info: ").or_else(|| l.strip_prefix("Warning: ")).unwrap_or(l);
	let (dev, rest) = l.split_once(' ')?;
	let rest = rest.strip_prefix("track").or_else(|| rest.strip_prefix("core2"))?;
	let text = format!("{dev} track{rest}");
	let rest = rest.strip_prefix(':').unwrap_or(rest).trim_start();
	let i = rest.find(" at ")?;
	let what = &rest[..i];
	let after = &rest[i + 4..];
	let j = after.find(" s")?;
	let t: f64 = after[..j].parse().ok()?;
	if what.is_empty() || !t.is_finite() {
		return None;
	}
	Some((dev.to_string(), what.to_string(), t, text))
}

/// An event's weight: 2 (red) where the tracker lost the device or found its state wrong while tracking (ran away,
/// diverged, a warm restart, stations disagreeing), 0 (green) where it got it back (a start confirmed, a cold
/// restart solved), 1 (amber) the rest: the attempts while it is lost, the ties, a start dropped
pub fn event_level(what: &str, text: &str) -> u8 {
	let solved = text.contains(": SOLVED");
	if what.contains("ran away")
		|| what.contains("diverged")
		|| what.starts_with("warm restart")
		|| what.contains("disagree")
		|| text.contains("NOT fit")
	{
		2
	} else if what.contains("confirmed") || (what.starts_with("cold restart") && solved) {
		0
	} else {
		1
	}
}

/// A device's light in a replay: each sweep that lit it (its time, the station's channel, the plane, the sensors it
/// lit)
#[derive(Clone, Debug, Default)]
pub struct Light {
	pub t: Vec<f32>,
	pub ch: Vec<u8>,
	pub plane: Vec<u8>,
	pub mask: Vec<u64>,
}

impl Light {
	pub fn len(&self) -> usize {
		self.t.len()
	}

	pub fn is_empty(&self) -> bool {
		self.t.is_empty()
	}

	fn push(&mut self, t: f32, ch: u8, plane: u8, mask: u64) {
		self.t.push(t);
		self.ch.push(ch);
		self.plane.push(plane);
		self.mask.push(mask);
	}

	/// The sensors each station lit in (t - window, t], by channel: (channel, both planes, either plane)
	pub fn lit(&self, t: f64, window: f64) -> Vec<(u8, u64, u64)> {
		let a = self.t.partition_point(|&x| (x as f64) <= t - window);
		let b = self.t.partition_point(|&x| (x as f64) <= t);
		let mut out: Vec<(u8, [u64; 2])> = Vec::new();
		for i in a..b {
			let k = match out.iter().position(|o| o.0 == self.ch[i]) {
				Some(k) => k,
				None => {
					out.push((self.ch[i], [0, 0]));
					out.len() - 1
				}
			};
			out[k].1[(self.plane[i] & 1) as usize] |= self.mask[i];
		}
		out.sort_by_key(|o| o.0);
		out.into_iter().map(|(c, m)| (c, m[0] & m[1], m[0] | m[1])).collect()
	}
}

/// The runtime's diagnosis of a device each half second (its DIAG lines): radio loss, stations and sensors in view,
/// whether it was dark
#[derive(Clone, Copy, Debug, Default)]
pub struct Diag {
	pub t: f32,
	pub loss: f32,
	pub stations: u8,
	pub sensors: u8,
	pub dark: bool,
}

/// The light lane's bins (s)
pub const LANE_S: f64 = 0.1;

/// One device in a replay
#[derive(Clone, Debug, Default)]
pub struct Dev {
	pub codename: String,
	pub serial: String,
	/// its render model ("{htc}vr_tracker_vive_3_0") and its type ("VIVE Tracker 3.0 MV")
	pub model: String,
	pub kind: String,
	pub replay: Track,
	pub live: Track,
	pub valve: Track,
	/// where Valve called its own pose not valid (from, to)
	pub valve_bad: Vec<(f32, f32)>,
	/// the replay's head frame into Valve's (a Tundra: 180 deg)
	pub c: Pose,
	/// (t, distance from Valve in m): live, replay
	pub err: [Vec<(f32, f32)>; 2],
	/// median, 90th percentile, largest (m), and the spans over OFF_M: live, replay
	pub stats: [(f32, f32, f32, usize); 2],
	/// its sensors in its IMU's frame (the config's, as the runtime reads it), and the IMU's frame in its head's
	pub sensors: Vec<V3>,
	pub normals: Vec<V3>,
	pub imu_in_head: Pose,
	/// the light that reached it (a replay that recorded it), the diagnosis's half seconds
	pub light: Light,
	pub diag: Vec<Diag>,
	/// stations lighting it per LANE_S (3 sensors or more from the light; else the diagnosis's), 255 where it was not
	/// posing
	pub lane: Vec<u8>,
}

impl Dev {
	/// Its sensors in the room at a pose of its head (the replay's): where they are and which way they face
	pub fn sensors_at(&self, head: &Pose) -> Vec<(V3, V3)> {
		let imu = compose(head, &self.imu_in_head);
		self.sensors
			.iter()
			.enumerate()
			.map(|(k, s)| (xform(&imu, *s), self.normals.get(k).map(|n| qrot(imu.q, *n)).unwrap_or_default()))
			.collect()
	}
}

/// A replay loaded: its devices, the stations, the events
#[derive(Debug, Default)]
pub struct Data {
	pub devs: Vec<Dev>,
	pub stations: Vec<(i32, Pose)>,
	pub events: Vec<Event>,
	pub dur: f64,
	/// SteamVR's world into the runtime's: from each time on, fitted to what the driver sent (`frames`) and to
	/// Valve's own poses (`vframes`); the two are the same where sally was aligned to Valve
	pub frames: Vec<(f32, Pose)>,
	pub vframes: Vec<(f32, Pose)>,
	pub have_live: bool,
	/// what was not there or did not work out
	pub notes: Vec<String>,
}

/// The frame at a time
pub fn frame_at(frames: &[(f32, Pose)], t: f64) -> Pose {
	let i = frames.partition_point(|f| (f.0 as f64) <= t);
	frames.get(i.saturating_sub(1)).map(|f| f.1).unwrap_or_default()
}

/// The frame for a pose at a time: its window's, or the one before or after it where that puts it nearer the replay
/// (`near`). A recenter (SteamVR's world moved at once, Space Calibrator after it) falls inside a window, whose fit
/// is the side with more of it: the poses of the other side go by the neighbouring window's. In steady tracking the
/// three are within a millimetre or two of each other
fn frame_for(frames: &[(f32, Pose)], t: f64, p: V3, near: Option<V3>) -> Pose {
	let i = frames.partition_point(|f| (f.0 as f64) <= t).saturating_sub(1);
	let Some(f) = frames.get(i) else { return Pose::default() };
	let Some(n) = near else { return f.1 };
	// the first entry is the whole session's, not a window's
	let lo = i.saturating_sub(1).max(1).min(i);
	frames[lo..(i + 2).min(frames.len())]
		.iter()
		.map(|f| f.1)
		.min_by(|a, b| len(xform(a, p) - n).total_cmp(&len(xform(b, p) - n)))
		.unwrap_or(f.1)
}

fn num(w: &[&str], i: usize) -> f64 {
	w.get(i).and_then(|s| s.parse().ok()).unwrap_or(f64::NAN)
}

/// A pose's 7 numbers from words (x y z qw qx qy qz)
fn pose_words(w: &[&str], i: usize) -> Option<Pose> {
	let v: Vec<f64> = (i..i + 7).map(|k| num(w, k)).collect();
	if v.iter().any(|x| !x.is_finite()) {
		return None;
	}
	Some(Pose { p: V3::new(v[0], v[1], v[2]), q: qnorm(Q4::new(v[3], v[4], v[5], v[6])) })
}

/// A device's clock (its timecodes unwrapped) against the run's, every second or so
#[derive(Default)]
struct Clock {
	last_tc: Option<u32>,
	wraps: u64,
	anchors: Vec<(f64, f64)>,
}

impl Clock {
	fn add(&mut self, run_t: f64, tc: u32) {
		if let Some(l) = self.last_tc {
			if tc < l && l - tc > 0x8000_0000 {
				self.wraps += 1;
			}
		}
		self.last_tc = Some(tc);
		let dev_t = (tc as u64 + (self.wraps << 32)) as f64 / TIMEBASE;
		if self.anchors.last().is_none_or(|a| run_t - a.0 > 1.0) {
			self.anchors.push((run_t, dev_t));
		}
	}

	/// The run time of a device time
	fn run_time(&self, dev_t: f64) -> Option<f64> {
		let i = self.anchors.partition_point(|a| a.1 <= dev_t);
		let a = self.anchors.get(i.saturating_sub(1)).or(self.anchors.first())?;
		Some(a.0 + (dev_t - a.1))
	}
}

/// What loading tells the UI meanwhile
pub type Progress = std::sync::mpsc::Sender<String>;

/// Reads a finished replay and the session's live files
pub fn load(dir: &Path, s: &Session, progress: Option<Progress>) -> Result<Data, String> {
	let say = |t: &str| {
		if let Some(p) = &progress {
			let _ = p.send(t.to_string());
		}
	};
	let mut data = Data { dur: s.duration(), ..Data::default() };
	let mut devs: BTreeMap<String, Dev> = BTreeMap::new();
	let mut clocks: HashMap<String, Clock> = HashMap::new();
	let mut stations: BTreeMap<i32, Pose> = BTreeMap::new();

	say("reading the replay");
	let f = File::open(dir.join("run.rec")).map_err(|e| format!("no run.rec: {e}"))?;
	let mut r = BufReader::with_capacity(1 << 20, f);
	// a replay that recorded the light (Y, W and B lines, some 4 GB an hour): its sweeps go to light.bin and the
	// rest of the recording is written again without them, once
	let light_file = dir.join("light.bin");
	let have_light = light_file.is_file();
	let raw = !have_light && recorded_angles(dir);
	let tmp_path = dir.join("run.rec.tmp");
	let mut tmp = if raw {
		Some(std::io::BufWriter::with_capacity(1 << 20, File::create(&tmp_path).map_err(|e| e.to_string())?))
	} else {
		None
	};
	// each device's sweep being gathered: its time, channel, plane, sensors
	let mut sweep: HashMap<String, (f32, u8, u8, u64)> = HashMap::new();
	let mut lights: BTreeMap<String, Light> = BTreeMap::new();
	let mut line = String::new();
	let mut n = 0u64;
	let mut file_t: Option<f64> = None;
	let mut skipped = 0u64;
	loop {
		line.clear();
		match r.read_line(&mut line) {
			Ok(0) => break,
			Ok(_) => {}
			Err(_) => break,
		}
		n += 1;
		if n.is_multiple_of(2_000_000) {
			say(&format!(
				"{} the replay ({} M lines)",
				if raw { "gathering the light of" } else { "reading" },
				n / 1_000_000
			));
		}
		let mut it = line.splitn(4, ' ');
		let (Some(t), Some(who), Some(kind)) = (it.next(), it.next(), it.next()) else { continue };
		// the recording's clock: two of the runtime's threads sometimes write into each other's lines, and a line can
		// then carry another's digits in front of its time (1611610.75 for 1610.75). Taken, such a sample put a
		// device's track far ahead of its clock and every sample after it was refused (a trail that stopped, or one
		// line across the room): a line more than a minute off the recording's clock is not taken
		match t.parse::<f64>() {
			Ok(tl) if file_t.is_none_or(|f| (tl - f).abs() <= LINE_CLOCK_S) => {
				file_t = Some(file_t.map_or(tl, |f| f.max(tl)));
			}
			_ => {
				skipped += 1;
				continue;
			}
		}
		if raw {
			match kind {
				"B" => {
					// "<t> KN0 B <channel> <sensor> <timecode> <plane> <angle>"
					let w: Vec<&str> = line.split_ascii_whitespace().collect();
					let (Ok(t), Some(ch), Some(sensor), Some(plane)) = (
						t.parse::<f32>(),
						w.get(3).and_then(|v| v.parse::<u8>().ok()),
						w.get(4).and_then(|v| v.parse::<u32>().ok()).filter(|&v| v < 64),
						w.get(6).and_then(|v| v.parse::<i32>().ok()),
					) else {
						continue;
					};
					let plane = (plane & 1) as u8;
					let cur = sweep.entry(who.to_string()).or_insert((t, ch, plane, 0));
					if cur.0 != t || cur.1 != ch || cur.2 != plane {
						if cur.3 != 0 {
							lights.entry(who.to_string()).or_default().push(cur.0, cur.1, cur.2, cur.3);
						}
						*cur = (t, ch, plane, 0);
					}
					cur.3 |= 1u64 << sensor;
					continue;
				}
				"W" | "Y" => continue,
				_ => {
					if let Some(w) = tmp.as_mut() {
						w.write_all(line.as_bytes()).map_err(|e| format!("cannot write the recording again: {e}"))?;
					}
				}
			}
		}
		match kind {
			"POSE" => {
				let w: Vec<&str> = line.split_ascii_whitespace().collect();
				let Ok(t) = t.parse::<f64>() else { continue };
				let Some(p) = pose_words(&w, 3) else { continue };
				let d =
					devs.entry(who.to_string()).or_insert_with(|| Dev { codename: who.to_string(), ..Dev::default() });
				// at most 240 a second
				if d.replay.t.last().is_none_or(|&l| t as f32 - l >= 1.0 / 240.0) {
					d.replay.push(t, &p);
				}
				if let Some(tc) = w.get(10).and_then(|s| s.parse::<u32>().ok()) {
					clocks.entry(who.to_string()).or_default().add(t, tc);
				}
			}
			"LH_POSE" => {
				let w: Vec<&str> = line.split_ascii_whitespace().collect();
				if let (Ok(idx), Some(p)) = (who.parse::<i32>(), pose_words(&w, 3)) {
					stations.insert(idx, p);
				}
			}
			"CONFIG" => {
				let json = it.next().unwrap_or("");
				let d =
					devs.entry(who.to_string()).or_insert_with(|| Dev { codename: who.to_string(), ..Dev::default() });
				d.serial = json_string(json, "device_serial_number");
				d.model = json_string(json, "render_model");
				d.kind = json_string(json, "model_number");
				let opts = sally_lighthouse::device::ConfigOptions::default();
				if let Ok(cfg) = sally_lighthouse::device::DeviceConfig::parse(json.trim_end().as_bytes(), who, &opts) {
					if let Some(l) = cfg.sensor_locations.filter(|_| cfg.has_sensor_locations) {
						d.sensors = l.iter().take(cfg.sensor_ct).map(|v| V3::new(v[0], v[1], v[2])).collect();
					}
					if let Some(l) = cfg.sensor_normals {
						d.normals = l.iter().take(cfg.sensor_ct).map(|v| V3::new(v[0], v[1], v[2])).collect();
					}
					// the runtime's head is its IMU's pose times head2imu: the IMU in the head's frame is its inverse
					let h = cfg.head2imu;
					let h = Pose {
						p: V3::new(h.pos[0], h.pos[1], h.pos[2]),
						q: Q4::new(h.rot[0], h.rot[1], h.rot[2], h.rot[3]),
					};
					d.imu_in_head = inverse(&Pose { p: h.p, q: qnorm(h.q) });
				}
			}
			"DIAG" => {
				// "<t> KN0 DIAG <now> <headline> <severity> <bars> <loss %> <gap ms> <imu n> <stations> <sensors> ..."
				let w: Vec<&str> = line.split_ascii_whitespace().collect();
				if let (Ok(t), Some(d)) = (t.parse::<f32>(), devs.get_mut(who)) {
					d.diag.push(Diag {
						t,
						loss: num(&w, 7) as f32,
						stations: num(&w, 10).clamp(0.0, 32.0) as u8,
						sensors: num(&w, 11).clamp(0.0, 64.0) as u8,
						dark: w.get(15) == Some(&"dark"),
					});
				}
			}
			_ => {}
		}
	}
	if raw {
		for (who, cur) in sweep {
			if cur.3 != 0 {
				lights.entry(who).or_default().push(cur.0, cur.1, cur.2, cur.3);
			}
		}
		say("writing the light");
		let mut w = tmp.take().expect("raw has it");
		w.flush().map_err(|e| format!("cannot write the recording again: {e}"))?;
		drop(w);
		drop(r);
		write_light(&light_file, &lights)?;
		std::fs::rename(&tmp_path, dir.join("run.rec")).map_err(|e| format!("cannot replace run.rec: {e}"))?;
	} else if have_light {
		lights = read_light(&light_file);
	} else {
		data.notes.push("no light per sensor in this replay (an older one): Replay again records it".into());
	}
	if skipped > 0 {
		let s = if skipped == 1 { "" } else { "s" };
		data.notes.push(format!("{skipped} line{s} the runtime's threads wrote into each other left out"));
	}
	for (who, l) in lights {
		if let Some(d) = devs.get_mut(&who) {
			d.light = l;
		}
	}
	data.stations = stations.into_iter().collect();
	// devices that never posed are left out
	devs.retain(|_, d| !d.replay.is_empty());

	// what the replay ran with and what did not fit (its args.txt's comments)
	if let Ok(a) = std::fs::read_to_string(dir.join("args.txt")) {
		data.notes.extend(a.lines().filter_map(|l| l.strip_prefix("# ")).map(String::from));
		if map_drifted(&a) {
			data.notes.push("this replay's map solve moved the stations: Replay again holds them".into());
		}
	}
	// the live poses: the driver's pose log, or SteamVR's poses as sally-check recorded them
	if let Some(pl) = &s.poselog {
		say("reading SteamVR's poses");
		let read = if s.check.is_some() { read_steamvr_poses(pl, s.t0, s.t1) } else { read_poselog(pl, s.t0, s.t1) };
		match read {
			Ok(rows) => {
				data.have_live = rows.iter().any(|r| r.sent.is_some());
				let by_serial: HashMap<String, String> =
					devs.values().map(|d| (d.serial.clone(), d.codename.clone())).collect();
				// the frames: what the driver sent and Valve's poses (SteamVR's world) against the replay at the
				// same time, each its own
				say("fitting the frame");
				data.frames = fit_frames(&rows, &devs, &by_serial, |r| r.sent.map(|p| p.p));
				data.vframes = fit_frames(&rows, &devs, &by_serial, |r| r.valid.then_some(r.valve.p));
				if data.frames.is_empty() && data.vframes.is_empty() {
					data.notes.push("the live poses could not be put in the replay's frame".into());
				}
				// Valve's poses in the frame of what the driver sent, so that the replay and what SteamVR got are
				// compared alike; in their own where there was nothing sent, or where sally was not aligned to Valve
				// (the two frames put the devices more than 5 cm apart)
				let mut own = data.frames.is_empty();
				if let (Some(a), Some(b)) = (data.frames.first(), data.vframes.first()) {
					let mut d: Vec<f64> = rows
						.iter()
						.step_by(50)
						.filter_map(|r| r.sent)
						.map(|p| len(xform(&a.1, p.p) - xform(&b.1, p.p)))
						.collect();
					d.sort_by(f64::total_cmp);
					let off = d.get(d.len() / 2).copied().unwrap_or(0.0);
					if off > 0.05 {
						own = true;
						data.notes.push(format!(
							"what SteamVR got was {:.0} cm from Valve's place: sally was not aligned to Valve (the \
							 replay is compared in Valve's own frame)",
							off * 100.0
						));
					}
				}
				if !own {
					data.vframes = data.frames.clone();
				}
				for row in &rows {
					let Some(code) = by_serial.get(&row.serial) else { continue };
					let Some(d) = devs.get_mut(code) else { continue };
					let near = d.replay.at(row.t).map(|p| p.p);
					if let Some(sent) = &row.sent {
						d.live.push(row.t, &compose(&frame_for(&data.frames, row.t, sent.p, near), sent));
					}
					if row.valid {
						d.valve.push(row.t, &compose(&frame_for(&data.vframes, row.t, row.valve.p, near), &row.valve));
					}
				}
				for (serial, spans) in valve_bad_spans(&rows) {
					if let Some(d) = by_serial.get(&serial).and_then(|c| devs.get_mut(c)) {
						d.valve_bad = spans;
					}
				}
				for d in devs.values_mut() {
					d.c = head_offset(d);
					errors(d, &rows);
				}
			}
			Err(e) => data.notes.push(e),
		}
	} else {
		data.notes.push("no pose log for this session: Valve's poses and what SteamVR got are not shown".into());
	}
	// with nothing to measure the head offset by, the model's kind gives it; no distances, no summary
	for d in devs.values_mut() {
		if d.live.len() < 10 && d.valve.len() < 10 {
			d.c = Pose { p: V3::default(), q: crate::models::head_turn(&d.model) };
		}
		for k in 0..2 {
			if d.err[k].is_empty() {
				d.stats[k] = summary(&[]);
			}
		}
	}

	// the events: the live run's and the replay's, on the run's time
	say("reading the events");
	let mut fix_k: HashMap<String, f64> = HashMap::new();
	for (live, path) in [(true, s.solver.clone()), (false, Some(dir.join("run.log")))] {
		let Some(path) = path else { continue };
		let Ok(b) = std::fs::read(&path) else { continue };
		let text = String::from_utf8_lossy(&b);
		for l in text.lines() {
			let Some((dev, what, dev_t, text)) = parse_event(l) else { continue };
			let Some(clock) = clocks.get(&dev) else { continue };
			let Some(mut t) = clock.run_time(dev_t) else { continue };
			// the device's clock may have wrapped between its first sample and its first pose: its first start
			// says so (it is next to its first pose)
			if !fix_k.contains_key(&dev) && what.contains("start confirmed") {
				let first = devs.get(&dev).and_then(|d| d.replay.t.first().copied()).unwrap_or(0.0) as f64;
				let k = ((t - first) / WRAP_S).round();
				fix_k.insert(dev.clone(), if (t - first - k * WRAP_S).abs() < 10.0 { -k * WRAP_S } else { 0.0 });
			}
			t += fix_k.get(&dev).copied().unwrap_or(0.0);
			if !(-5.0..=data.dur + 5.0).contains(&t) {
				continue;
			}
			let level = event_level(&what, &text);
			data.events.push(Event { t: t as f32, dev, what, text, live, level });
		}
	}
	data.events.sort_by(|a, b| a.t.total_cmp(&b.t));
	for d in devs.values_mut() {
		d.lane = light_lane(d, data.dur);
	}
	data.devs = devs.into_values().collect();
	say("");
	Ok(data)
}

/// Whether a replay ran with the map solve free to move the stations (an older panel's replay: `start` holds them now)
fn map_drifted(args: &str) -> bool {
	let l: Vec<&str> = args.lines().collect();
	let on = l.iter().rposition(|x| *x == "--map-solve").and_then(|i| l.get(i + 1)).is_none_or(|v| *v != "0");
	on && l.contains(&"--map-solve-file") && !l.contains(&"--map-solve-max-move")
}

/// Whether a replay recorded the light: its args.txt's last --record-angle is 1
fn recorded_angles(dir: &Path) -> bool {
	let a = std::fs::read_to_string(dir.join("args.txt")).unwrap_or_default();
	let l: Vec<&str> = a.lines().collect();
	l.iter().rposition(|x| *x == "--record-angle").and_then(|i| l.get(i + 1)) == Some(&"1")
}

/// light.bin: "SALLYLIGHT 1\n", then per device "<codename> <sweeps>\n" and that many 14-byte sweeps (time f32,
/// channel, plane, sensors u64; little-endian)
fn write_light(path: &Path, lights: &BTreeMap<String, Light>) -> Result<(), String> {
	let mut b: Vec<u8> = b"SALLYLIGHT 1\n".to_vec();
	for (who, l) in lights {
		b.extend_from_slice(format!("{who} {}\n", l.len()).as_bytes());
		for i in 0..l.len() {
			b.extend_from_slice(&l.t[i].to_le_bytes());
			b.push(l.ch[i]);
			b.push(l.plane[i]);
			b.extend_from_slice(&l.mask[i].to_le_bytes());
		}
	}
	std::fs::write(path, b).map_err(|e| format!("cannot write {}: {e}", path.display()))
}

fn read_light(path: &Path) -> BTreeMap<String, Light> {
	let mut out = BTreeMap::new();
	let Ok(b) = std::fs::read(path) else { return out };
	let Some(mut at) = b.strip_prefix(b"SALLYLIGHT 1\n").map(|_| 13) else { return out };
	while at < b.len() {
		let Some(e) = b[at..].iter().position(|&c| c == b'\n') else { break };
		let head = String::from_utf8_lossy(&b[at..at + e]).into_owned();
		at += e + 1;
		let Some((who, n)) = head.split_once(' ') else { break };
		let Ok(n) = n.parse::<usize>() else { break };
		if at + n * 14 > b.len() {
			break;
		}
		let mut l = Light::default();
		for k in 0..n {
			let r = &b[at + k * 14..at + k * 14 + 14];
			let t = f32::from_le_bytes([r[0], r[1], r[2], r[3]]);
			let mask = u64::from_le_bytes([r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13]]);
			l.push(t, r[4], r[5], mask);
		}
		at += n * 14;
		out.insert(who.to_string(), l);
	}
	out
}

/// A device's stations in view per LANE_S: from its light (a station that lit 3 sensors or more in the bin), else
/// from the diagnosis's half seconds (0 while dark); 255 where it was not posing
fn light_lane(d: &Dev, dur: f64) -> Vec<u8> {
	let n = (dur / LANE_S).ceil().max(0.0) as usize + 1;
	let mut lane = vec![255u8; n];
	let (Some(&first), Some(&last)) = (d.replay.t.first(), d.replay.t.last()) else { return lane };
	let bin = |t: f32| ((t as f64 / LANE_S) as usize).min(n - 1);
	for v in lane.iter_mut().take(bin(last) + 1).skip(bin(first)) {
		*v = 0;
	}
	if !d.light.is_empty() {
		let mut i = 0;
		while i < d.light.len() {
			let b = bin(d.light.t[i]);
			let mut per: Vec<(u8, u64)> = Vec::new();
			while i < d.light.len() && bin(d.light.t[i]) == b {
				match per.iter_mut().find(|p| p.0 == d.light.ch[i]) {
					Some(p) => p.1 |= d.light.mask[i],
					None => per.push((d.light.ch[i], d.light.mask[i])),
				}
				i += 1;
			}
			lane[b] = per.iter().filter(|p| p.1.count_ones() >= 3).count() as u8;
		}
	} else {
		for g in &d.diag {
			let b1 = bin(g.t);
			let b0 = bin((g.t - 0.5).max(0.0));
			for v in lane.iter_mut().take(b1 + 1).skip(b0) {
				if *v != 255 {
					*v = if g.dark { 0 } else { g.stations };
				}
			}
		}
	}
	lane
}

/// A pose log row: the run time, the device, what the driver sent and Valve's pose (both SteamVR's world, the
/// head), Valve's flag
#[derive(Clone, Debug)]
pub struct Row {
	pub t: f64,
	pub serial: String,
	/// what the driver sent (none in sally-check's recordings: SteamVR showed Valve's)
	pub sent: Option<Pose>,
	pub valve: Pose,
	pub valid: bool,
}

/// The pose log's rows in [t0, t1] (the PC's clock), on the run's time
pub fn read_poselog(path: &Path, t0: f64, t1: f64) -> Result<Vec<Row>, String> {
	let f = File::open(path).map_err(|e| format!("cannot read the pose log: {e}"))?;
	let mut r = BufReader::with_capacity(1 << 20, f);
	let mut head = String::new();
	r.read_line(&mut head).map_err(|e| e.to_string())?;
	let cols: Vec<&str> = head.trim().split(',').collect();
	let ix = |k: &str| cols.iter().position(|c| *c == k);
	let need = |ks: &[&str]| -> Option<Vec<usize>> { ks.iter().map(|k| ix(k)).collect() };
	let (Some(it), Some(iser), Some(ival)) = (ix("t"), ix("serial"), ix("vvalid")) else {
		return Err("the pose log has no t / serial / vvalid".into());
	};
	let sent = need(&["sx", "sy", "sz", "sqw", "sqx", "sqy", "sqz"]).ok_or("the pose log has no sent pose")?;
	let valve = need(&["vx", "vy", "vz", "vqw", "vqx", "vqy", "vqz"]).ok_or("the pose log has no Valve pose")?;
	let vh = need(&["vhx", "vhy", "vhz", "vhqw", "vhqx", "vhqy", "vhqz"]);
	let vf = need(&["vfx", "vfy", "vfz", "vfqw", "vfqx", "vfqy", "vfqz"]);
	let sf = need(&["sfx", "sfy", "sfz", "sfqw", "sfqx", "sfqy", "sfqz"]);
	let pose_at = |w: &[&str], i: &[usize]| -> Option<Pose> {
		let v: Vec<f64> = i.iter().map(|&k| num(w, k)).collect();
		if v.iter().any(|x| !x.is_finite()) || v[3..].iter().all(|x| *x == 0.0) {
			return None;
		}
		Some(Pose { p: V3::new(v[0], v[1], v[2]), q: qnorm(Q4::new(v[3], v[4], v[5], v[6])) })
	};
	let mut rows = Vec::new();
	let mut line = String::new();
	loop {
		line.clear();
		match r.read_line(&mut line) {
			Ok(0) | Err(_) => break,
			Ok(_) => {}
		}
		let w: Vec<&str> = line.trim_end().split(',').collect();
		let t = num(&w, it);
		if !(t >= t0 && t <= t1) {
			continue;
		}
		let (Some(s), Some(v)) = (pose_at(&w, &sent), pose_at(&w, &valve)) else { continue };
		let ident = Pose::default();
		let vh = vh.as_ref().and_then(|i| pose_at(&w, i)).unwrap_or(ident);
		let vf = vf.as_ref().and_then(|i| pose_at(&w, i)).unwrap_or(ident);
		let sf = sf.as_ref().and_then(|i| pose_at(&w, i)).unwrap_or(ident);
		rows.push(Row {
			t: t - t0,
			serial: w.get(iser).unwrap_or(&"").to_string(),
			// the driver sends sally's pose at the head (no driver-from-head); Valve's is its IMU and its head offset
			sent: Some(compose(&sf, &s)),
			valve: compose(&vf, &compose(&v, &vh)),
			valid: w.get(ival) == Some(&"1"),
		});
	}
	Ok(rows)
}

/// SteamVR's poses as sally-check recorded them (steamvr_poses.csv: t_us on the tap's clock, serial, result, valid,
/// x y z qw qx qy qz in SteamVR's world, the head frame): the trackers' and controllers' rows in [t0, t1], each Valve's
/// pose (SteamVR showed Valve's). A pose far off (a device an app hid at 9001 m) is not valid
pub fn read_steamvr_poses(path: &Path, t0: f64, t1: f64) -> Result<Vec<Row>, String> {
	let f = File::open(path).map_err(|e| format!("cannot read SteamVR's poses: {e}"))?;
	let mut r = BufReader::with_capacity(1 << 20, f);
	let mut head = String::new();
	r.read_line(&mut head).map_err(|e| e.to_string())?;
	let cols: Vec<&str> = head.trim().split(',').collect();
	let ix = |k: &str| cols.iter().position(|c| *c == k);
	let (Some(it), Some(iser), Some(ival)) = (ix("t_us"), ix("serial"), ix("valid")) else {
		return Err("steamvr_poses.csv has no t_us / serial / valid".into());
	};
	let ip: Option<Vec<usize>> = ["x", "y", "z", "qw", "qx", "qy", "qz"].iter().map(|k| ix(k)).collect();
	let ip = ip.ok_or("steamvr_poses.csv has no pose")?;
	let mut rows = Vec::new();
	let mut line = String::new();
	loop {
		line.clear();
		match r.read_line(&mut line) {
			Ok(0) | Err(_) => break,
			Ok(_) => {}
		}
		let w: Vec<&str> = line.trim_end().split(',').collect();
		if !w.get(iser).is_some_and(|s| s.starts_with("LHR-")) {
			continue;
		}
		let t = num(&w, it) / 1e6;
		if !(t >= t0 && t <= t1) {
			continue;
		}
		let v: Vec<f64> = ip.iter().map(|&k| num(&w, k)).collect();
		if v.iter().any(|x| !x.is_finite()) || v[3..].iter().all(|x| *x == 0.0) {
			continue;
		}
		let valve = Pose { p: V3::new(v[0], v[1], v[2]), q: qnorm(Q4::new(v[3], v[4], v[5], v[6])) };
		let valid = w.get(ival) == Some(&"1") && len(valve.p) < 100.0;
		rows.push(Row { t: t - t0, serial: w[iser].to_string(), sent: None, valve, valid });
	}
	Ok(rows)
}

/// Valve's not-valid spans per serial
fn valve_bad_spans(rows: &[Row]) -> HashMap<String, Vec<(f32, f32)>> {
	let mut out: HashMap<String, Vec<(f32, f32)>> = HashMap::new();
	let mut open: HashMap<&str, f64> = HashMap::new();
	for r in rows {
		match (r.valid, open.get(r.serial.as_str()).copied()) {
			(false, None) => {
				open.insert(&r.serial, r.t);
			}
			(true, Some(a)) => {
				open.remove(r.serial.as_str());
				out.entry(r.serial.clone()).or_default().push((a as f32, r.t as f32));
			}
			_ => {}
		}
	}
	let end = rows.last().map(|r| r.t).unwrap_or(0.0);
	for (s, a) in open {
		out.entry(s.to_string()).or_default().push((a as f32, end as f32));
	}
	out
}

/// Valve's world into the runtime's: a robust rigid fit of what the driver sent to the replay's pose at the same
/// time, every FRAME_WINDOW_S (pairs of every device, every 4th row), the whole session's where a window has too few
/// (the first entry, from f32::MIN)
fn fit_frames(
	rows: &[Row], devs: &BTreeMap<String, Dev>, by_serial: &HashMap<String, String>, pick: impl Fn(&Row) -> Option<V3>,
) -> Vec<(f32, Pose)> {
	let mut src_all = Vec::new();
	let mut dst_all = Vec::new();
	let mut win: BTreeMap<i64, (Vec<V3>, Vec<V3>)> = BTreeMap::new();
	for (i, r) in rows.iter().enumerate() {
		if i % 4 != 0 {
			continue;
		}
		let Some(d) = by_serial.get(&r.serial).and_then(|c| devs.get(c)) else { continue };
		let Some(src) = pick(r) else { continue };
		let Some(p) = d.replay.at(r.t) else { continue };
		src_all.push(src);
		dst_all.push(p.p);
		let w = win.entry((r.t / FRAME_WINDOW_S) as i64).or_default();
		w.0.push(src);
		w.1.push(p.p);
	}
	if src_all.len() < 30 {
		return Vec::new();
	}
	let Some((global, _, _)) = fit_rigid_robust(&src_all, &dst_all, 0.01) else { return Vec::new() };
	let mut out = vec![(f32::MIN, global)];
	for (k, (src, dst)) in win {
		let fit = if src.len() >= 120 { fit_rigid_robust(&src, &dst, 0.01).map(|f| f.0) } else { None };
		out.push(((k as f64 * FRAME_WINDOW_S) as f32, fit.unwrap_or(global)));
	}
	out
}

/// The replay's head frame into Valve's for one device: the rotation (and offset) that takes the replay's pose onto
/// Valve's (in its frame), else onto what the driver sent, the median of the pairs. Valve's first: an old driver sent a
/// Tundra in the runtime's head frame, upside down in SteamVR
fn head_offset(d: &Dev) -> Pose {
	let other = if d.valve.len() >= 10 { &d.valve } else { &d.live };
	let mut qs: Vec<Q4> = Vec::new();
	let mut ps: Vec<V3> = Vec::new();
	for i in (0..other.len()).step_by(7) {
		let t = other.t[i] as f64;
		let (Some(a), Some(b)) = (d.replay.at(t), other.at(t)) else { continue };
		let c = compose(&inverse(&a), &b);
		if len(c.p) < 0.05 {
			qs.push(c.q);
			ps.push(c.p);
		}
	}
	if qs.len() < 10 {
		return Pose::default();
	}
	// the quaternions on one side of the first, averaged; the offset the median of each axis
	let q0 = qs[0];
	let mut acc = Q4::new(0.0, 0.0, 0.0, 0.0);
	for q in &qs {
		let s = if q0.w * q.w + q0.x * q.x + q0.y * q.y + q0.z * q.z < 0.0 { -1.0 } else { 1.0 };
		acc = Q4::new(acc.w + s * q.w, acc.x + s * q.x, acc.y + s * q.y, acc.z + s * q.z);
	}
	let med = |mut v: Vec<f64>| {
		v.sort_by(f64::total_cmp);
		v[v.len() / 2]
	};
	let p = V3::new(
		med(ps.iter().map(|p| p.x).collect()),
		med(ps.iter().map(|p| p.y).collect()),
		med(ps.iter().map(|p| p.z).collect()),
	);
	// a head offset of more than 5 mm is the driver's limit (K_HEAD_FIX_M): more is tracking, not the frame
	Pose { p: if len(p) <= 0.005 { p } else { V3::default() }, q: qnorm(acc) }
}

/// The distances from Valve: what SteamVR got (both in Valve's world) and the replay (in the frame), where Valve's
/// pose is valid; and their summary
fn errors(d: &mut Dev, rows: &[Row]) {
	let mut live = Vec::new();
	let mut rep = Vec::new();
	for r in rows.iter().filter(|r| r.valid && r.serial == d.serial) {
		if let Some(sent) = &r.sent {
			live.push((r.t as f32, len(sent.p - r.valve.p) as f32));
		}
		if let (Some(p), Some(v)) = (d.replay.at(r.t), d.valve.at(r.t)) {
			let head = xform(&p, d.c.p);
			rep.push((r.t as f32, len(head - v.p) as f32));
		}
	}
	d.stats = [summary(&live), summary(&rep)];
	d.err = [live, rep];
}

fn summary(e: &[(f32, f32)]) -> (f32, f32, f32, usize) {
	if e.is_empty() {
		return (-1.0, -1.0, -1.0, 0);
	}
	let mut v: Vec<f32> = e.iter().map(|x| x.1).collect();
	v.sort_by(f32::total_cmp);
	let spans = e.windows(2).filter(|w| w[0].1 <= OFF_M && w[1].1 > OFF_M).count() + usize::from(e[0].1 > OFF_M);
	(v[v.len() / 2], v[v.len() * 9 / 10], v[v.len() - 1], spans)
}

/// The model's pose of a device's track at a time: the replay's in Valve's head frame (by the head offset)
pub fn model_pose(d: &Dev, which: usize, t: f64) -> Option<Pose> {
	match which {
		0 => d.valve.at(t),
		1 => d.live.at(t),
		_ => d.replay.at(t).map(|p| compose(&p, &d.c)),
	}
}

/// A rotation turned by a quaternion (the room view's Z up from a model's frame): kept for the UI's axes
pub fn turn(q: Q4, v: V3) -> V3 {
	qrot(q, v)
}

/// The angle between two orientations (deg)
pub fn angle_deg(a: Q4, b: Q4) -> f64 {
	let d = qmul(qconj(a), b);
	2.0 * d.w.abs().min(1.0).acos() * 57.29578
}

/// Writes a line to a file (the replay folder's notes), ignoring failure
pub fn note(dir: &Path, text: &str) {
	if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(dir.join("notes.txt")) {
		let _ = writeln!(f, "{text}");
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A session is listed whichever form its recording is in, with the same span; one there in both forms is listed
	/// once
	#[test]
	fn a_replay_goes_when_its_session_has() {
		let dir = std::env::temp_dir().join(format!("sally_panel_prune_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		let (logs, replays, checks) = (dir.join("logs"), dir.join("replays"), dir.join("checks"));
		let replay = |name: &str, marked: bool| {
			let d = replays.join(name);
			std::fs::create_dir_all(&d).unwrap();
			std::fs::write(d.join("run.rec"), "POSE").unwrap();
			if marked {
				std::fs::write(d.join("args.txt"), "--hidtap-playback").unwrap();
			}
		};
		let left = |replays: &Path| {
			let mut names: Vec<String> = std::fs::read_dir(replays)
				.unwrap()
				.flatten()
				.map(|e| e.file_name().to_string_lossy().into_owned())
				.collect();
			names.sort();
			names
		};
		let paths = Paths { logs: logs.clone(), replays: replays.clone(), checks: checks.clone(), ..Paths::default() };
		// a session there as text, one packed, one gone; a sally-check recording there, one gone; and what is no replay
		for name in
			["20261101_100000", "20261102_100000", "20261103_100000", "check_20261104_100000", "check_20261105_100000"]
		{
			replay(name, true);
		}
		replay("20261106_100000", false);
		replay("notes", true);
		replay("20261107_100000_mine", true);
		std::fs::write(replays.join("20261108_100000"), "a file").unwrap();
		// the recordings cannot be listed: nothing goes
		assert_eq!(prune(&paths), 0);
		assert_eq!(left(&replays).len(), 9);
		std::fs::create_dir_all(&logs).unwrap();
		std::fs::write(logs.join("sallytap_20261101_100000.hidtap"), "R").unwrap();
		std::fs::write(logs.join("sallytap_20261102_100000.hidtapz"), "R").unwrap();
		std::fs::write(logs.join("sallytap_20261103_100000_solver.txt"), "its log alone").unwrap();
		std::fs::create_dir_all(checks.join("sally-check_20261104_100000")).unwrap();
		assert_eq!(prune(&paths), 2);
		assert_eq!(
			left(&replays),
			[
				"20261101_100000",
				"20261102_100000",
				"20261106_100000",
				"20261107_100000_mine",
				"20261108_100000",
				"check_20261104_100000",
				"notes"
			]
		);
		assert_eq!(prune(&paths), 0);
		// no folder of sally-check recordings at all: there are none, and their replays go
		std::fs::remove_dir_all(&checks).unwrap();
		assert_eq!(prune(&paths), 1);
		assert!(!replays.join("check_20261104_100000").exists());
		let _ = std::fs::remove_dir_all(&dir);
	}

	#[test]
	fn a_packed_recording_is_a_session_as_a_text_one_is() {
		let dir = std::env::temp_dir().join(format!("sally_panel_sessions_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		let logs = dir.join("logs");
		std::fs::create_dir_all(&logs).unwrap();
		let text: String = (0..3000u64)
			.map(|n| format!("R {} 4BCAD73D0E-6YX 2102 -1 -1 I 23{:0116x}\r\n", 5_000_000 + n * 2000, n))
			.collect();
		let text = format!("# cache 4BCAD73D0E-6YX LHR-00000001\r\n{text}");
		let packed = |to: &Path| {
			let mut w = sally_hid::pack::PackWriter::new(File::create(to).unwrap()).unwrap();
			w.write(text.as_bytes()).unwrap();
			w.finish().unwrap();
		};
		std::fs::write(logs.join("sallytap_20261101_100000.hidtap"), &text).unwrap();
		packed(&logs.join("sallytap_20261102_100000.hidtapz"));
		std::fs::write(logs.join("sallytap_20261103_100000.hidtap"), &text).unwrap();
		packed(&logs.join("sallytap_20261103_100000.hidtapz"));
		std::fs::write(logs.join("sallytap_20261104_100000.hidtapz.part"), &text).unwrap();
		let paths = Paths { logs: logs.clone(), replays: dir.join("replays"), ..Paths::default() };
		let found = sessions(&paths);
		let names: Vec<String> =
			found.iter().map(|s| s.tap.file_name().unwrap().to_string_lossy().into_owned()).collect();
		assert_eq!(
			names,
			["sallytap_20261103_100000.hidtap", "sallytap_20261102_100000.hidtapz", "sallytap_20261101_100000.hidtap"]
		);
		for s in &found {
			assert_eq!((s.t0, s.t1), (5.0, 10.998), "{}", s.stamp);
		}
		assert!(found[1].bytes * 4 < found[2].bytes);
		// and the devices it names are read out of either
		let head = |s: &Session| {
			let mut head = Vec::new();
			sally_hid::pack::open(&s.tap).unwrap().take(1 << 20).read_to_end(&mut head).unwrap();
			configs_needed(&head, None)
		};
		assert_eq!(head(&found[1]), head(&found[2]));
		assert_eq!(head(&found[1]), [("4BCAD73D0E-6YX_config.json".to_string(), "LHR-00000001".to_string())]);
		let _ = std::fs::remove_dir_all(&dir);
	}

	#[test]
	fn a_tracker_line_is_an_event() {
		let (d, w, t, _) = parse_event("Info: KN4 track warm restart at 536.951 s: SOLVED, 21 of 24 hits").unwrap();
		assert_eq!((d.as_str(), w.as_str(), t), ("KN4", "warm restart", 536.951));
		let (_, w, t, _) = parse_event("Info: KN4 track: the vote ties at 89.5 s (2 sweep planes ...)").unwrap();
		assert_eq!((w.as_str(), t), ("the vote ties", 89.5));
		let (_, w, _, text) = parse_event(
			"Info: KN2 track: light back after a blackout at 12.25 s (0.8 s): the coasted state does NOT fit",
		)
		.unwrap();
		assert_eq!(event_level(&w, &text), 2);
		assert_eq!(event_level("start confirmed", ""), 0);
		assert_eq!(event_level("warm restart", "KN4 track warm restart at 1 s: SOLVED, 21 of 24 hits"), 2);
		assert_eq!(event_level("cold restart", "KN4 track cold restart at 1 s: SOLVED, 21 of 24 hits"), 0);
		assert_eq!(event_level("cold restart", "KN4 track cold restart at 1 s: refused, 3 of 24 hits"), 1);
		assert_eq!(event_level("stations disagree", ""), 2);
		assert_eq!(event_level("the vote ties", ""), 1);
		assert!(parse_event("Info: KN0 track: 989778 hits in 152242 sweeps, used 979229").is_none());
		assert!(parse_event("Info: map-solve: 4 stations at 1.0 s").is_none());
		// an older log names the tracker "core2": the same event, its text as written now
		let (d, w, t, text) = parse_event("Info: KN4 core2 warm restart at 536.951 s: SOLVED, 21 of 24 hits").unwrap();
		assert_eq!((d.as_str(), w.as_str(), t), ("KN4", "warm restart", 536.951));
		assert_eq!(text, "KN4 track warm restart at 536.951 s: SOLVED, 21 of 24 hits");
		let (_, w, _, text) = parse_event("Info: KN4 core2: the vote ties at 89.5 s (2 sweep planes ...)").unwrap();
		assert_eq!(
			(w.as_str(), text.as_str()),
			("the vote ties", "KN4 track: the vote ties at 89.5 s (2 sweep planes ...)")
		);
	}

	#[test]
	fn the_live_arguments_less_the_live_ones() {
		let live: Vec<String> = [
			"--hidtap",
			"1",
			"--hidtap-record",
			"C:\\logs\\x.hidtap",
			"--configfile",
			"C:\\cfg.json",
			"--map-solve",
			"1",
			"--core2",
			"1",
			"--pose-udp",
			"7001",
			"--threaded-posers",
			"0",
			"--device-learned-file",
			"C:\\survive\\device_learned.txt",
			"--some-flag",
		]
		.map(String::from)
		.to_vec();
		assert_eq!(replay_args(&live), ["--map-solve", "1", "--core2", "1", "--threaded-posers", "0", "--some-flag"]);
		let mut p = Paths::default();
		p.read_driver_settings(
			"# x\ndriver.runtime_exe=C:\\a\\survive-cli.exe\ndriver.runtime_log=C:\\proj\\logs\\sallytap_{time}_solver.txt\n\
			 --core2\n1\n\n# c\n--map-solve-file\nC:\\proj\\map.txt\n",
		);
		assert_eq!(p.runtime_args, ["--core2", "1", "--map-solve-file", "C:\\proj\\map.txt"]);
		assert_eq!(p.runtime_exe, PathBuf::from("C:\\a\\survive-cli.exe"));
		assert!(!takes_subcommand(&p.runtime_exe));
		// sally.exe: its `run` is the replay's to give, not one of the live arguments
		let mut p = Paths::default();
		p.read_driver_settings("driver.runtime_exe=C:\\a\\sally.exe\nrun\n--hidtap\n1\n--map-solve\n1\n");
		assert_eq!(p.runtime_args, ["--hidtap", "1", "--map-solve", "1"]);
		assert!(takes_subcommand(&p.runtime_exe));
	}

	#[test]
	fn the_configs_a_session_had() {
		let tap = b"R 1 x\n# cache A9DDD3B47A-LYM LHR-9365E531\nA 12.5 00A4AE2B50-RYB LHR-C57C5A45\nA 13 X -\n";
		assert_eq!(
			configs_needed(tap, None),
			vec![
				("A9DDD3B47A-LYM_config.json".into(), "LHR-9365E531".into()),
				("LHR-C57C5A45_config.json".into(), "LHR-C57C5A45".into())
			]
		);
		let log = "Info: hidtap: opened 4BCAD73D0E-6YX (2102, mi -1, 59-byte reports)\n\
		           Info: Adding tracked object KN0 from HID\nInfo: Wrote out 8075 bytes to LHR-2D779713_config.json\n\
		           Info: Wrote out 8075 bytes to KN0_config.json\nInfo: hidtap: device 4BCAD73D0E-6YX is KN0 (28de:2102)\n\
		           Info: Adding tracked object KN3 from HID\nInfo: hidtap: device 6FE9771ADC-7YX is KN3 (28de:2102)\n";
		assert_eq!(
			configs_needed(b"R 1 x\n", Some(log)),
			vec![("4BCAD73D0E-6YX_config.json".into(), "LHR-2D779713".into())]
		);
	}

	#[test]
	fn a_track_between_its_samples_and_its_gaps() {
		let mut tr = Track::default();
		let q = Q4::new(1.0, 0.0, 0.0, 0.0);
		for (t, x) in [(0.0, 0.0), (0.1, 1.0), (0.2, 2.0), (1.0, 3.0)] {
			tr.push(t, &Pose { p: V3::new(x, 0.0, 0.0), q });
		}
		assert!((tr.at(0.05).unwrap().p.x - 0.5).abs() < 1e-6);
		assert!(tr.at(0.5).is_none(), "a gap");
		assert!(tr.at(-0.1).is_none());
		assert!(tr.at(1.01).is_some() && tr.at(1.5).is_none());
		assert_eq!(tr.gaps(), vec![(0.2f32, 1.0f32)]);
		assert_eq!(tr.span(0.05, 0.21), (1, 3));
	}

	#[test]
	fn a_device_clock_on_the_run_s_time_across_its_wrap() {
		let mut c = Clock::default();
		// the device's clock 80 s on at run time 0, wrapping 9.48 s later
		for i in 0..200 {
			let run = i as f64 * 0.1;
			let dev = 80.0 + run;
			let tc = ((dev * TIMEBASE) as u64 % (1u64 << 32)) as u32;
			c.add(run, tc);
		}
		assert!((c.run_time(85.0).unwrap() - 5.0).abs() < 1e-3);
		// past the wrap: the device time goes on, it does not start again
		assert!((c.run_time(95.0).unwrap() - 15.0).abs() < 1e-3);
	}

	#[test]
	fn the_frame_and_the_head_offset_from_the_pose_log() {
		// Valve's world turned 90 deg about x and moved from the runtime's; a Tundra-like 180 deg head offset
		let f = Pose { p: V3::new(0.5, -1.0, 2.0), q: qnorm(Q4::new(1.0, 1.0, 0.0, 0.0)) };
		let c = Pose { p: V3::new(0.0, 0.0, 0.0015), q: Q4::new(0.0, 0.0, 1.0, 0.0) };
		let mut devs = BTreeMap::new();
		let mut rows = Vec::new();
		let mut d = Dev { codename: "KN0".into(), serial: "LHR-1".into(), ..Dev::default() };
		for i in 0..2000 {
			let t = i as f64 * 0.01;
			let p = Pose {
				p: V3::new((t * 0.7).sin(), (t * 1.3).cos(), 1.0 + 0.3 * (t * 0.4).sin()),
				q: qnorm(Q4::new(1.0, 0.2 * t.sin(), 0.1, 0.3 * t.cos())),
			};
			d.replay.push(t, &p);
			let sent = compose(&inverse(&f), &compose(&p, &c));
			rows.push(Row { t, serial: "LHR-1".into(), sent: Some(sent), valve: sent, valid: true });
		}
		devs.insert("KN0".to_string(), d);
		let by_serial: HashMap<String, String> = [("LHR-1".to_string(), "KN0".to_string())].into();
		let frames = fit_frames(&rows, &devs, &by_serial, |r| r.sent.map(|p| p.p));
		let fr = frames.last().unwrap().1;
		assert!(len(fr.p - f.p) < 0.003, "{:?}", fr);
		let d = devs.get_mut("KN0").unwrap();
		for r in &rows {
			d.live.push(r.t, &compose(&fr, &r.sent.unwrap()));
			d.valve.push(r.t, &compose(&fr, &r.valve));
		}
		d.c = head_offset(d);
		assert!(angle_deg(d.c.q, c.q) < 0.5, "{:?}", d.c);
		errors(d, &rows);
		assert!(d.stats[0].2 < 1e-6 && d.stats[1].2 < 0.004, "{:?}", d.stats);
	}

	#[test]
	fn a_sally_check_recording_s_stations_and_steamvr_poses() {
		let dir = std::env::temp_dir().join(format!("sally_panel_check_{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		std::fs::write(
			dir.join("devices.txt"),
			"serial\tclass\trole\tdongle\tmodel\tsystem\tmode\nLHR-4B02A3E0\ttracker\tTrackerRole_Waist\t08F8F6307C-3YX\t\
			 VIVE Tracker 3.0 MV\tlighthouse\t\nLHB-15B80A74\tstation\t\t\tValve SR Imp\tlighthouse\t1\n",
		)
		.unwrap();
		assert_eq!(check_stations(&dir), vec![0x15B80A74]);
		std::fs::write(
			dir.join("steamvr_poses.csv"),
			"t_us,serial,result,valid,connected,x,y,z,qw,qx,qy,qz\n\
			 10000000,1PASHWWE5PFP07,200,1,1,0,1.5,0,1,0,0,0\n\
			 10000000,LHR-4B02A3E0,200,1,1,0.1,1.0,0.2,1,0,0,0\n\
			 10004000,LHR-775B4248,200,1,1,0,9001,0,1,0,0,0\n\
			 10008000,LHR-4B02A3E0,101,0,1,0.1,1.0,0.2,1,0,0,0\n\
			 99000000,LHR-4B02A3E0,200,1,1,0.1,1.0,0.2,1,0,0,0\n",
		)
		.unwrap();
		let rows = read_steamvr_poses(&dir.join("steamvr_poses.csv"), 9.0, 20.0).unwrap();
		std::fs::remove_dir_all(&dir).unwrap();
		// the headset's row and the one after t1 left out; the one at 9001 m and the one not valid kept, not valid
		let v: Vec<(String, bool)> = rows.iter().map(|r| (r.serial.clone(), r.valid)).collect();
		assert_eq!(v, [("LHR-4B02A3E0".into(), true), ("LHR-775B4248".into(), false), ("LHR-4B02A3E0".into(), false)]);
		assert!((rows[0].t - 1.0).abs() < 1e-9 && rows.iter().all(|r| r.sent.is_none()));
	}

	#[test]
	fn the_light_at_a_time_by_station_both_planes_and_either() {
		let mut l = Light::default();
		l.push(1.00, 7, 0, 0b0111);
		l.push(1.01, 7, 1, 0b0110);
		l.push(1.02, 9, 0, 0b1000);
		l.push(1.50, 9, 1, 0b0001);
		// at 1.05 over 0.1 s: station 7 both planes on sensors 1 and 2, sensor 0 on one; station 9 one plane
		assert_eq!(l.lit(1.05, 0.1), vec![(7, 0b0110, 0b0111), (9, 0, 0b1000)]);
		// the window leaves the earlier sweeps out
		assert_eq!(l.lit(1.55, 0.1), vec![(9, 0, 0b0001)]);
		assert!(l.lit(3.0, 0.1).is_empty());
		let mut m = BTreeMap::new();
		m.insert("KN0".to_string(), l.clone());
		let path = std::env::temp_dir().join(format!("sally_panel_light_{}.bin", std::process::id()));
		write_light(&path, &m).unwrap();
		let back = read_light(&path);
		std::fs::remove_file(&path).unwrap();
		assert_eq!((back["KN0"].mask.clone(), back["KN0"].ch.clone()), (l.mask, l.ch));
	}

	#[test]
	fn a_pose_across_a_recenter_takes_the_neighbouring_window_s_frame() {
		let moved = Pose { p: V3::new(0.2, 0.0, 0.0), q: Q4::default() };
		let frames = vec![(f32::MIN, Pose::default()), (0.0, Pose::default()), (10.0, moved), (20.0, moved)];
		let p = V3::new(1.0, 2.0, 1.0);
		// in the window after the recenter, a pose from before it: the window before's frame puts it on the replay
		assert_eq!(frame_for(&frames, 12.0, p, Some(p)).p, V3::default());
		assert_eq!(frame_for(&frames, 12.0, p, Some(p + moved.p)).p, moved.p);
		assert_eq!(frame_for(&frames, 12.0, p, None).p, moved.p);
		assert_eq!(frame_for(&frames, 25.0, p, Some(p)).p, moved.p, "no window after the last but itself");
	}

	#[test]
	fn a_replay_without_a_pose_log_loads_its_devices_and_events() {
		let dir = std::env::temp_dir().join(format!("sally_panel_replay_{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		// a Tundra posing for 10 s from run time 1 s, its clock at 20.833 s then
		let mut rec = String::from("0.5 KN0 CONFIG {\"device_serial_number\":\"LHR-T1\",");
		rec += "\"render_model\":\"{tundra_labs}tundra_tracker\",\"model_number\":\"Tundra Tracker\"}\n";
		rec += "0.6 0 LH_POSE 1 2 3 1 0 0 0\n";
		for i in 0..1000 {
			let t = 1.0 + i as f64 * 0.01;
			let tc = 1_000_000_000u64 + (i as u64) * 480_000;
			rec += &format!("{t:.6} KN0 POSE {:.4} 0.5 1.0 1 0 0 0 {tc}\n", t * 0.1);
		}
		std::fs::write(dir.join("run.rec"), rec).unwrap();
		std::fs::write(dir.join("run.log"), "Info: KN0 track warm restart at 25.833 s: SOLVED\n").unwrap();
		let s = Session { stamp: "20261108_000000".into(), t0: 100.0, t1: 112.0, ..Session::default() };
		let d = load(&dir, &s, None).unwrap();
		std::fs::remove_dir_all(&dir).unwrap();
		assert_eq!((d.devs.len(), d.stations.len(), d.dur), (1, 1, 12.0));
		let x = &d.devs[0];
		assert_eq!((x.serial.as_str(), x.kind.as_str(), x.replay.len()), ("LHR-T1", "Tundra Tracker", 1000));
		// no Valve to measure the head offset by: the Tundra's turn; no distances
		assert!((angle_deg(Q4::default(), x.c.q) - 180.0).abs() < 1e-3);
		assert_eq!(x.stats, [(-1.0, -1.0, -1.0, 0); 2]);
		assert!(d.notes.iter().any(|n| n.contains("no pose log")));
		// the event 5 s on the device's clock after its first pose
		assert_eq!(d.events.len(), 1);
		assert!((d.events[0].t - 6.0).abs() < 0.01 && !d.events[0].live, "{:?}", d.events);
	}

	#[test]
	fn a_replay_whose_map_moved_is_told() {
		let old = "runtime x\n--configfile\n./driver_config.json\n--map-solve-file\nmap.txt\n--map-solve\n1\n";
		assert!(map_drifted(old));
		assert!(!map_drifted(&format!("{old}--map-solve-max-move\n0\n--map-solve-max-rot\n0\n")));
		assert!(!map_drifted("runtime x\n--map-solve-file\nmap.txt\n--map-solve\n0\n"));
		assert!(!map_drifted("runtime x\n--configfile\n./driver_config.json\n"));
	}

	#[test]
	fn a_line_two_threads_wrote_into_each_other_is_left_out() {
		let dir = std::env::temp_dir().join(format!("sally_panel_replay_race_{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let mut rec = String::new();
		for i in 0..400 {
			let t = 1610.0 + i as f64 * 0.01;
			let tc = 1_000_000_000u64 + (i as u64) * 480_000;
			// at 2 s in, another line's "16" in front of this one's time (1611612.0): taken, the track stopped there
			let lead = if i == 200 { "16" } else { "" };
			rec += &format!("{lead}{t:.6} KN0 POSE {:.4} 0.5 1.0 1 0 0 0 {tc}\n", i as f64 * 0.001);
		}
		std::fs::write(dir.join("run.rec"), rec).unwrap();
		let s = Session { stamp: "20261108_000001".into(), t0: 100.0, t1: 104.0, ..Session::default() };
		let d = load(&dir, &s, None).unwrap();
		std::fs::remove_dir_all(&dir).unwrap();
		let x = &d.devs[0];
		assert_eq!(x.replay.len(), 399);
		assert!(x.replay.t.windows(2).all(|w| w[1] > w[0] && w[1] - w[0] < 0.03), "{:?}", x.replay.t);
		assert!(d.notes.iter().any(|n| n.starts_with("1 line the")), "{:?}", d.notes);
	}
}
