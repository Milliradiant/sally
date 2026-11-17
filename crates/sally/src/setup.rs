//! SteamVR set up for sally and back:
//! - `sally setup`: the sally driver put on SteamVR's list of drivers (vrpathreg adddriver; another folder's sally taken
//!   off it), its settings written (`sally_settings.txt`: the driver's options, the runtime and the panel it starts,
//!   the runtime's arguments), then the panel started. SteamVR may be running: sally starts with its next start. With
//!   SteamVR closed, what in SteamVR's settings would keep sally from loading is set right (a copy of them kept first)
//!   and a runtime left over from an earlier session is stopped. From then on starting SteamVR is all a session needs:
//!   the driver starts the runtime and the panel, restarts the runtime if it exits, stops it with SteamVR. No device is
//!   asked for anything: the runtime takes each device's config from SteamVR's own copy as the device shows up.
//! - `sally off`: with SteamVR closed, the sally driver off and Valve's lighthouse driver on (Valve's tracking alone).
//! - `sally uninstall`: with SteamVR closed, sally out of SteamVR: a leftover runtime and the panel stopped, the driver
//!   taken off SteamVR's list and out of its settings, a sally this one took the place of back on the list. The folder
//!   is left as it is: deleting it is all that remains (the installer's uninstall does that).
//! - `sally stop`: a runtime still running after SteamVR closed, stopped (it would answer the next SteamVR start in
//!   place of a new one, and holds the runtime's files).
//! - `sally copy-log`: SteamVR's server log (vrserver.txt, the driver's lines in it) copied into sally's logs.
//!
//! sally's files are kept one of two ways (`Layout`): in the project folder it is built into (survive\ with the room's
//! files, survive\hidtap\ with the runtime, steamvr_driver\sally\, tools\sally_panel\, logs\), or in the folder the
//! installer makes (sally.exe, the panel and the driver's folder side by side, data\ and logs\ beside them). `--home`
//! names the folder; without it, it is found from where this program is.

use std::path::{Path, PathBuf};

use sally_lighthouse::steamvr;

/// The driver's options (docs/sally_driver.md, "Options")
const DRIVER_OPTIONS: &[&str] = &[
	"# the driver's options (docs/sally_driver.md, \"Options\"): a device lost after 150 ms without a pose, switches",
	"# blended over 150 ms, the carry on. The pose's declared age (9 ms over the radio) is the output layer's",
	"# prediction_ms now, per class and device: sally_output.txt beside this file, the panel's OUTPUT",
	"driver.lost_timeout_ms=150",
	"driver.runtime_port=9100",
	"driver.valve_udp=9102,9103",
	"driver.cmd_port=9104",
	"driver.seen=sally",
	"driver.fallback=1",
	"driver.blend_ms=150",
	"driver.valve_settle_s=2",
	"driver.valve_agree_m=0.3",
	"driver.carry=1",
];

/// The runtime's options for live play, after the files it is given
const RUNTIME_OPTIONS: &[&str] = &[
	"--disable-calibrate",
	"1",
	"--lock-lighthouses",
	"1",
	"--serialize-device-config",
	"1",
	"--record-rawlight",
	"0",
	"--record-angle",
	"0",
	"--map-solve",
	"1",
	"--gen2-plane-predict",
	"0.35",
	"--light-facing-deg",
	"115",
];

/// Where sally's files are under its folder, and what a new install of that kind keeps
struct Layout {
	/// the runtime's folder (sally.exe)
	runtime: &'static str,
	/// where the runtime runs: its own copies of the devices' configs
	devices: &'static str,
	/// the room's files: its calibration, the map solve's, what was learned of the devices and stations
	room: &'static str,
	/// the driver's folder, the one SteamVR is given
	driver: &'static str,
	/// the panel's folder (sally-panel.exe)
	panel: &'static str,
	/// copies of SteamVR's settings from before setup changed them
	backups: &'static str,
	/// whether each session writes a pose log (every pose the driver saw and sent: large, for looking into the
	/// tracking)
	pose_log: bool,
	/// how many of each session file a new install keeps (`driver.keep_sessions`; the driver deletes the older ones
	/// as SteamVR starts): a session leaves a recording and the runtime's log in logs\, and nothing else ever removes
	/// them. The calibration needs the session's own recording and the one before it at most
	keep_sessions: i32,
	/// whether the folder stays after `sally uninstall` (the installer's is deleted with it)
	stays: bool,
}

/// The project folder
const PROJECT: Layout = Layout {
	runtime: "survive\\hidtap",
	devices: "survive\\hidtap\\devcfg",
	room: "survive",
	driver: "steamvr_driver\\sally",
	panel: "tools\\sally_panel",
	backups: "steamvr_config",
	pose_log: true,
	keep_sessions: 10,
	stays: true,
};

/// The installer's folder
const INSTALLED: Layout = Layout {
	runtime: "",
	devices: "data\\devices",
	room: "data",
	driver: "driver",
	panel: "",
	backups: "data\\steamvr_settings",
	pose_log: false,
	keep_sessions: 2,
	stays: false,
};

/// A file's place in one of the layout's folders ("" is sally's folder itself)
fn within(dir: &str, name: &str) -> String {
	if dir.is_empty() {
		name.to_string()
	} else {
		format!("{dir}\\{name}")
	}
}

/// sally's folder and how it is laid out
struct Home {
	root: PathBuf,
	at: &'static Layout,
}

impl Home {
	/// The folder given, else the one this program is in: the installer's folder when the driver's folder is beside
	/// the program, else the project folder two up from it (survive\hidtap\sally.exe)
	fn find(given: Option<PathBuf>) -> Result<Home, String> {
		let installed = |d: &Path| d.join("driver").join("driver.vrdrivermanifest").is_file();
		if let Some(root) = given {
			let at = if installed(&root) { &INSTALLED } else { &PROJECT };
			return Ok(Home { root, at });
		}
		let exe = std::env::current_exe().map_err(|e| format!("cannot find this program: {e}"))?;
		let dir = exe.parent().ok_or("cannot find this program's folder")?;
		if installed(dir) {
			return Ok(Home { root: dir.to_path_buf(), at: &INSTALLED });
		}
		dir.parent()
			.and_then(Path::parent)
			.map(|root| Home { root: root.to_path_buf(), at: &PROJECT })
			.ok_or_else(|| "cannot find sally's folder: give it with --home".to_string())
	}

	fn path(&self, rel: &str) -> PathBuf {
		rel.split('\\').filter(|part| !part.is_empty()).fold(self.root.clone(), |p, part| p.join(part))
	}

	/// A path as text for the driver's settings: Windows' separators unless the folder is written with '/'
	fn text(&self, rel: &str) -> String {
		let root = self.root.to_string_lossy();
		let sep = if root.contains('/') && !root.contains('\\') { "/" } else { "\\" };
		format!("{}{sep}{}", root.trim_end_matches(['/', '\\']), rel.replace('\\', sep))
	}

	fn runtime_dir(&self) -> PathBuf {
		self.path(self.at.runtime)
	}

	fn panel_dir(&self) -> PathBuf {
		self.path(self.at.panel)
	}

	fn driver_dir(&self) -> PathBuf {
		self.path(self.at.driver)
	}
}

/// What a command was given: sally's folder (`--home <folder>`), and for setup whether the panel is started
/// (`--no-panel`: the installer starts it itself, so that the panel is not a child of the setup it waits for), and
/// whether one already running from the folder is stopped first (`--stop-panel`: that one is from before the
/// installer changed the files)
struct Asked {
	home: Home,
	panel: bool,
	stop: bool,
}

fn asked(args: &[String]) -> Result<Asked, String> {
	let mut i = 0;
	let mut root = None;
	let mut panel = true;
	let mut stop = false;
	while i < args.len() {
		match args[i].as_str() {
			"--home" if i + 1 < args.len() => {
				root = Some(PathBuf::from(&args[i + 1]));
				i += 2;
			}
			"--no-panel" => {
				panel = false;
				i += 1;
			}
			"--stop-panel" => {
				stop = true;
				i += 1;
			}
			a => return Err(format!("unknown argument {a}")),
		}
	}
	Ok(Asked { home: Home::find(root)?, panel, stop })
}

/// The settings the driver reads, `sally_settings.txt`: its options, the runtime and the panel it starts, the runtime's
/// arguments one a line (`run` first). CRLF, as Notepad writes it. `kept`: the driver's options the file had that
/// setup does not write itself (`kept_options`); `keep_sessions`: how many of each session file stay (`keep_sessions`)
fn driver_settings(home: &Home, stamp: &str, kept: &[String], keep_sessions: i32) -> String {
	let l = home.at;
	let mut lines: Vec<String> =
		vec![format!("# written by sally setup {stamp}; setting sally up again writes it again")];
	lines.extend(DRIVER_OPTIONS.iter().map(|s| s.to_string()));
	if l.pose_log {
		lines.push(format!("driver.pose_log={}", home.text("logs\\sally_poses_{time}.csv")));
	}
	lines.push(
		"# the newest this many of each session file in logs\\ are kept (the recording, the runtime's log, the".into(),
	);
	lines.push("# pose log), the older ones deleted as SteamVR starts; 0 keeps them all. The panel's choice".into());
	lines.push("# (Settings, Recordings) stands over this one".into());
	lines.push(format!("driver.keep_sessions={keep_sessions}"));
	if !kept.is_empty() {
		lines.push("# kept from the settings before this setup (the virtual trackers, ...)".into());
		lines.extend(kept.iter().cloned());
	}
	lines.push("# the panel the driver starts with SteamVR".into());
	lines.push(format!("driver.panel_exe={}", home.text(&within(l.panel, "sally-panel.exe"))));
	lines.push("# the runtime the driver starts with SteamVR ({time} = the session's start)".into());
	lines.push(format!("driver.runtime_exe={}", home.text(&within(l.runtime, "sally.exe"))));
	lines.push(format!("driver.runtime_cwd={}", home.text(l.devices)));
	lines.push(format!("driver.runtime_log={}", home.text("logs\\sallytap_{time}_solver.txt")));
	lines.push("# its arguments, one a line".into());
	for a in [
		"run".into(),
		"--hidtap".into(),
		"1".into(),
		"--hidtap-config-fetch".into(),
		"0".into(),
		"--hidtap-record".into(),
		home.text("logs\\sallytap_{time}.hidtap"),
		"--configfile".into(),
		home.text(&within(l.room, "driver_config.json")),
		"--runtime-port".into(),
		"9100".into(),
		"--map-solve-file".into(),
		home.text(&within(l.room, "map_solve.txt")),
		"--device-learned-file".into(),
		home.text(&within(l.room, "device_learned.txt")),
		"--station-fcal-file".into(),
		home.text(&within(l.room, "station_fcal.txt")),
	] {
		lines.push(a);
	}
	lines.extend(RUNTIME_OPTIONS.iter().map(|s| s.to_string()));
	let mut out = lines.join("\r\n");
	out.push_str("\r\n");
	out
}

/// The driver's options setup writes itself, by name (`driver.<name>`)
fn written_options() -> Vec<&'static str> {
	let mut names: Vec<&str> = DRIVER_OPTIONS
		.iter()
		.filter_map(|l| l.strip_prefix("driver."))
		.map(|l| l.split('=').next().unwrap_or(l).trim())
		.collect();
	names.extend(["pose_log", "keep_sessions", "panel_exe", "runtime_exe", "runtime_cwd", "runtime_log"]);
	// an option setup once wrote, replaced by the output layer's prediction_ms: not kept from an older settings file
	names.push("extra_latency_ms");
	names
}

/// The `driver.<name>` lines of the settings as they stand that setup does not write itself: options set since the
/// last setup (driver.hmd, the virtual trackers' driver.virtual and its files) stay through the next one, as they
/// are. The last line of a name is the one kept (the driver takes the last too)
fn kept_options(old: &str) -> Vec<String> {
	let written = written_options();
	let mut kept: Vec<(String, String)> = Vec::new();
	for line in old.lines().map(str::trim) {
		let Some(rest) = line.strip_prefix("driver.") else { continue };
		let name = rest.split('=').next().unwrap_or(rest).trim().to_string();
		if name.is_empty() || written.contains(&name.as_str()) {
			continue;
		}
		kept.retain(|(n, _)| *n != name);
		kept.push((name, line.to_string()));
	}
	kept.into_iter().map(|(_, l)| l).collect()
}

/// How many of each session file stay, from the settings as they stand (None: there is no settings file yet, and the
/// install is new: `new`, the layout's own). What the file says is kept; a file that does not say (one from before the
/// option) keeps them all, as it did, so a setup never starts deleting an install's recordings
fn keep_sessions(old: Option<&str>, new: i32) -> i32 {
	let Some(old) = old else { return new };
	old.lines()
		.filter_map(|l| l.trim().strip_prefix("driver.keep_sessions"))
		.filter_map(|rest| rest.trim_start().strip_prefix('='))
		.filter_map(|v| v.trim().parse::<i32>().ok())
		.next_back()
		.map_or(0, |n| n.max(0))
}

/// `sally setup`
pub fn setup(args: &[String]) -> i32 {
	let asked = match asked(args) {
		Ok(a) => a,
		Err(e) => return fail(&e),
	};
	let home = &asked.home;
	let vr_running = sys::running(&["vrserver.exe"]);
	let notes = match set_up(home, vr_running) {
		Ok(n) => n,
		Err(e) => return fail(&e),
	};
	if asked.stop {
		stop_panel(home);
	}
	if asked.panel {
		let panel = home.panel_dir().join("sally-panel.exe");
		if !asked.stop && sys::running(&["sally-panel.exe"]) {
			println!("the panel is running");
		} else if panel.is_file() {
			match sys::start_detached(&panel) {
				Ok(()) => println!("the panel started"),
				Err(e) => println!("the panel did not start: {e}"),
			}
		}
	}
	println!();
	println!("sally is set up. Starting SteamVR is all a session needs: the sally driver starts the runtime and the");
	println!(
		"panel with it, restarts the runtime if it exits, and stops it when SteamVR closes. In the headset sally's"
	);
	println!(
		"tracking shows once it has lined up with Valve's (about 15 s of normal moving); until then, and whenever"
	);
	println!("sally has no pose for a device, SteamVR shows Valve's. The panel says what sally is doing and switches");
	println!("between sally's tracking and Valve's.");
	// (the installer shows these lines to the user)
	for n in &notes {
		println!("note: {n}");
	}
	pause();
	0
}

/// What `sally setup` does; what the user should know of it afterwards (SteamVR to restart, something SteamVR has
/// switched off that could not be switched on while it runs)
fn set_up(home: &Home, vr_running: bool) -> Result<Vec<String>, String> {
	let l = home.at;
	let runtime = home.runtime_dir().join("sally.exe");
	if !runtime.is_file() {
		return Err(format!("missing: {}", runtime.display()));
	}
	let driver_dir = home.driver_dir();
	let driver_bin = driver_dir.join("bin").join("win64");
	if !driver_bin.join("driver_sally.dll").is_file() {
		return Err(format!("the sally driver is not in {}", driver_bin.display()));
	}
	let vr = steamvr::Paths::find();
	if vr.runtime.is_none() && vr.config.is_none() {
		return Err("SteamVR was not found on this PC. Install SteamVR, start it once, then set sally up again.".into());
	}
	// a room without its config (a new install): the runtime calibrates it during its first session and writes the
	// config then (autocal); until then it runs without tracking and SteamVR shows Valve's
	let new_room = !home.path(&within(l.room, "driver_config.json")).is_file();
	// (an unpacked release may have lost its empty folders: the sessions' files go in logs\)
	for d in [l.devices, l.room, "logs"] {
		let d = home.path(d);
		std::fs::create_dir_all(&d).map_err(|e| format!("cannot make {}: {e}", d.display()))?;
	}
	// a runtime left from an earlier session would answer on its port; with SteamVR running the runtime is the
	// driver's own and stays
	if !vr_running {
		stop_leftover(home);
	}
	let others = register(&vr, &driver_dir)?;
	for o in &others {
		println!("another sally was on SteamVR's list of drivers and is off it now: {}", o.display());
	}
	remember_replaced(home, &others);

	let settings = driver_bin.join("sally_settings.txt");
	// (a file that is there but is not text has nothing to keep, and is still an install from before)
	let old = settings.is_file().then(|| std::fs::read_to_string(&settings).unwrap_or_default());
	let kept = kept_options(old.as_deref().unwrap_or_default());
	let keep = keep_sessions(old.as_deref(), l.keep_sessions);
	write_whole(&settings, driver_settings(home, &stamp(), &kept, keep).as_bytes())?;
	println!("written: {}", settings.display());
	match keep {
		0 => println!("  every session's recording and logs are kept (driver.keep_sessions=0)"),
		n => println!("  the newest {n} sessions' recordings and logs are kept (driver.keep_sessions={n})"),
	}
	for k in &kept {
		println!("  kept: {k}");
	}
	if new_room {
		println!("this room is not calibrated yet: the first session does it by itself, a few minutes in. Walk the");
		println!(
			"  devices around the room and hold them still now and then; until then SteamVR shows Valve's tracking"
		);
	}

	let mut notes = Vec::new();
	let sets = vrsettings(&vr).map(|doc| blocking(&doc)).unwrap_or_default();
	if sets.is_empty() {
		println!("SteamVR's settings need no change: the sally driver loads with SteamVR.");
	} else if vr_running {
		notes.push(
			"SteamVR has sally or its own lighthouse tracking switched off. Close SteamVR and set sally up again, or \
			 switch them on in SteamVR: Settings, Startup / Shutdown, Manage add-ons."
				.to_string(),
		);
	} else {
		edit_vrsettings(home, &vr, &sets, &[], true)?;
		println!("SteamVR set up: Valve's lighthouse driver on, the sally driver on.");
	}
	if vr_running {
		notes.push("SteamVR is running now: restart it to start sally.".to_string());
	}
	Ok(notes)
}

/// What in SteamVR's settings keeps sally from loading, as the (section, key, value) that set it right: Valve's
/// lighthouse driver off (sally's poses go out on its devices), drivers besides the headset's off, the sally driver
/// off, or blocked since a crash of SteamVR's (its safe mode). Settings that say nothing of these are left alone
fn blocking(doc: &json::Value) -> Vec<(&'static str, &'static str, &'static str)> {
	[
		("driver_lighthouse", "enable", "false", "true"),
		("steamvr", "activateMultipleDrivers", "false", "true"),
		("driver_sally", "enable", "false", "true"),
		("driver_sally", "blocked_by_safe_mode", "true", "false"),
	]
	.into_iter()
	.filter(|(section, key, bad, _)| doc.get(section, key) == Some(*bad))
	.map(|(section, key, _, good)| (section, key, good))
	.collect()
}

/// SteamVR's settings (steamvr.vrsettings) as a document; None where there are none to read (a SteamVR that has not
/// run yet has no file: nothing in it stands in sally's way)
fn vrsettings(vr: &steamvr::Paths) -> Option<json::Value> {
	let file = vr.config.as_ref()?.join("steamvr.vrsettings");
	json::parse(&std::fs::read_to_string(file).ok()?)
}

/// SteamVR's vrpathreg run with `verb` on a driver's folder (adddriver, removedriver); None where there is no
/// SteamVR, or no vrpathreg in it
fn vrpathreg(vr: &steamvr::Paths, verb: &str, driver_dir: &Path) -> Option<bool> {
	let exe = vr.runtime.as_ref()?.join("bin").join("win64").join("vrpathreg.exe");
	if !exe.is_file() {
		return None;
	}
	let status =
		std::process::Command::new(&exe).arg(verb).arg(driver_dir).stdout(std::process::Stdio::null()).status();
	Some(status.map(|s| s.success()).unwrap_or(false))
}

/// SteamVR's path registry (openvrpaths.vrpath) and the drivers' folders it lists ("external_drivers")
struct Registry {
	file: PathBuf,
	doc: json::Value,
}

impl Registry {
	fn read() -> Option<Registry> {
		let file = steamvr::registry_file()?;
		let doc = json::parse(&std::fs::read_to_string(&file).ok()?)?;
		Some(Registry { file, doc })
	}

	fn drivers(&self) -> Vec<PathBuf> {
		match self.doc.member("external_drivers") {
			Some(json::Value::Array(items)) => items
				.iter()
				.filter_map(|v| match v {
					json::Value::Text(t) => json::unquote(t).map(PathBuf::from),
					_ => None,
				})
				.collect(),
			_ => Vec::new(),
		}
	}

	fn write(&mut self, drivers: &[PathBuf]) -> Result<(), String> {
		let items = drivers.iter().map(|d| json::Value::Text(json::quote(&d.to_string_lossy()))).collect();
		self.doc.set_member("external_drivers", json::Value::Array(items));
		write_whole(&self.file, self.doc.write().as_bytes())
	}
}

/// Whether a driver's folder holds a sally (its manifest's name): another install's, told from other drivers
fn is_sally(dir: &Path) -> bool {
	std::fs::read_to_string(dir.join("driver.vrdrivermanifest"))
		.ok()
		.and_then(|t| steamvr::json_first_string(&t, "name"))
		.is_some_and(|n| n == "sally")
}

/// SteamVR's list of drivers with `add` on it and each of `remove` off it, the rest as they are. vrpathreg is asked
/// first; the list is written here only where that left it otherwise (no vrpathreg, or it did not succeed)
fn set_listed(vr: &steamvr::Paths, add: Option<&Path>, remove: &[PathBuf]) -> Result<(), String> {
	for r in remove {
		vrpathreg(vr, "removedriver", r);
	}
	if let Some(a) = add {
		vrpathreg(vr, "adddriver", a);
	}
	let mut reg = Registry::read().ok_or("cannot read SteamVR's list of drivers (openvrpaths.vrpath)")?;
	let now = reg.drivers();
	let mut want: Vec<PathBuf> = now
		.iter()
		.filter(|d| !remove.iter().any(|r| same_path(r, d)) && !add.is_some_and(|a| same_path(a, d)))
		.cloned()
		.collect();
	want.extend(add.map(Path::to_path_buf));
	let same = want.len() == now.len() && want.iter().all(|w| now.iter().any(|n| same_path(n, w)));
	if same {
		Ok(())
	} else {
		reg.write(&want)
	}
}

/// The sally driver's folder on SteamVR's list of drivers, and any other folder's sally off it (two drivers of one
/// name: SteamVR loads one of them, not always this one). The others taken off
fn register(vr: &steamvr::Paths, driver_dir: &Path) -> Result<Vec<PathBuf>, String> {
	let listed = Registry::read().map(|r| r.drivers()).unwrap_or_default();
	let others: Vec<PathBuf> = listed.into_iter().filter(|d| !same_path(d, driver_dir) && is_sally(d)).collect();
	set_listed(vr, Some(driver_dir), &others)?;
	Ok(others)
}

/// The file naming the sallys this one took the place of on SteamVR's list, one folder a line: uninstall puts them
/// back
fn replaced_file(home: &Home) -> PathBuf {
	home.path(&within(home.at.room, "replaced_drivers.txt"))
}

fn replaced(home: &Home) -> Vec<PathBuf> {
	std::fs::read_to_string(replaced_file(home))
		.map(|t| t.lines().map(str::trim).filter(|l| !l.is_empty()).map(PathBuf::from).collect())
		.unwrap_or_default()
}

fn remember_replaced(home: &Home, others: &[PathBuf]) {
	let mut all = replaced(home);
	let before = all.len();
	for o in others {
		if !all.iter().any(|a| same_path(a, o)) {
			all.push(o.clone());
		}
	}
	if all.len() != before {
		let text: String = all.iter().map(|p| format!("{}\r\n", p.display())).collect();
		let _ = write_whole(&replaced_file(home), text.as_bytes());
	}
}

/// `sally uninstall`
pub fn uninstall(args: &[String]) -> i32 {
	let asked = match asked(args) {
		Ok(a) => a,
		Err(e) => return fail(&e),
	};
	let home = &asked.home;
	if sys::running(&["vrserver.exe", "vrmonitor.exe", "vrcompositor.exe"]) {
		return fail("SteamVR is running. Close SteamVR completely, then run this again.");
	}
	stop_leftover(home);
	stop_panel(home);
	let vr = steamvr::Paths::find();
	match set_listed(&vr, None, &[home.driver_dir()]) {
		Ok(()) => println!("the sally driver is off SteamVR's list of drivers"),
		Err(e) => println!("{e}: the driver may still be on it"),
	}
	for d in replaced(home).into_iter().filter(|d| is_sally(d)) {
		if set_listed(&vr, Some(&d), &[]).is_ok() {
			println!("the sally this one took the place of is back on SteamVR's list: {}", d.display());
		}
	}
	// sally's section out of SteamVR's settings; activateMultipleDrivers stays, other drivers go by it too
	if let Some(doc) = vrsettings(&vr) {
		let lighthouse_off = doc.get("driver_lighthouse", "enable") == Some("false");
		let sets: &[(&str, &str, &str)] = if lighthouse_off { &[("driver_lighthouse", "enable", "true")] } else { &[] };
		if lighthouse_off || doc.has("driver_sally") {
			if let Err(e) = edit_vrsettings(home, &vr, sets, &["driver_sally"], home.at.stays) {
				return fail(&e);
			}
		}
	}
	println!("sally is out of SteamVR: Valve's lighthouse driver alone, as before sally was set up.");
	if home.at.stays {
		println!("What is left is this folder, with its recordings, logs and the room's calibration:");
		println!("  {}", home.root.display());
		println!("Delete it to remove sally entirely, or run sally setup from it to set sally up again.");
	}
	pause();
	0
}

/// `sally` with no command, started from Explorer or a shortcut (its console its own): the panel opened, which is
/// what a double click on sally.exe is for. False when that is not how it was started, or there is no panel
pub fn open_panel() -> bool {
	if !sys::console_is_ours() {
		return false;
	}
	let Ok(home) = Home::find(None) else { return false };
	let panel = home.panel_dir().join("sally-panel.exe");
	panel.is_file() && sys::start_detached(&panel).is_ok()
}

/// `sally off`
pub fn off(args: &[String]) -> i32 {
	let asked = match asked(args) {
		Ok(a) => a,
		Err(e) => return fail(&e),
	};
	if sys::running(&["vrserver.exe", "vrmonitor.exe", "vrcompositor.exe"]) {
		return fail("SteamVR is running. Close SteamVR completely, then run this again.");
	}
	let vr = steamvr::Paths::find();
	let sets = [("driver_lighthouse", "enable", "true"), ("driver_sally", "enable", "false")];
	if let Err(e) = edit_vrsettings(&asked.home, &vr, &sets, &[], true) {
		return fail(&e);
	}
	println!("Back to Valve's lighthouse driver alone: the sally driver is off. sally setup turns it on again.");
	pause();
	0
}

/// `sally stop`
pub fn stop(args: &[String]) -> i32 {
	let asked = match asked(args) {
		Ok(a) => a,
		Err(e) => return fail(&e),
	};
	if stop_leftover(&asked.home) == 0 {
		println!("no leftover runtime running");
	}
	pause();
	0
}

/// `sally copy-log`
pub fn copy_log(args: &[String]) -> i32 {
	let asked = match asked(args) {
		Ok(a) => a,
		Err(e) => return fail(&e),
	};
	let Some(src) = steamvr::Paths::find().server_log() else { return fail("cannot find SteamVR's logs") };
	let dst = asked.home.path("logs").join(format!("vrserver_{}.txt", stamp()));
	match std::fs::copy(&src, &dst) {
		Ok(_) => println!("copied {} to {}", src.display(), dst.display()),
		Err(e) => return fail(&format!("cannot copy {}: {e}", src.display())),
	}
	pause();
	0
}

/// The programs of these names running from a folder stopped, this program left alone; how many
fn stop_in(dir: &Path, names: &[&str], what: &str) -> usize {
	let mut n = 0;
	for p in sys::processes() {
		let in_dir = p.path.parent().is_some_and(|d| same_path(d, dir));
		let name = p.path.file_name().map(|f| f.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
		if in_dir && names.contains(&name.as_str()) && p.pid != std::process::id() {
			println!("stopping {what} {} (pid {})", p.path.display(), p.pid);
			if sys::kill(p.pid) {
				n += 1;
			} else {
				println!("  it did not stop");
			}
		}
	}
	n
}

/// The runtimes running from sally's runtime folder (sally.exe, or an older survive-cli.exe) stopped; how many
fn stop_leftover(home: &Home) -> usize {
	stop_in(&home.runtime_dir(), &["sally.exe", "survive-cli.exe"], "the leftover runtime")
}

/// The panel running from sally's folder stopped (it holds its files open)
fn stop_panel(home: &Home) -> usize {
	stop_in(&home.panel_dir(), &["sally-panel.exe"], "the panel")
}

fn same_path(a: &Path, b: &Path) -> bool {
	let norm = |p: &Path| p.to_string_lossy().replace('/', "\\").trim_end_matches('\\').to_ascii_lowercase();
	norm(a) == norm(b)
}

/// SteamVR's settings (steamvr.vrsettings) with each (section, key, value) set and each section of `without` taken
/// out. `backup`: a copy of them kept first in sally's folder. What is written is read back first: the same
/// settings, less the sections taken out, or nothing is written
fn edit_vrsettings(
	home: &Home, vr: &steamvr::Paths, sets: &[(&str, &str, &str)], without: &[&str], backup: bool,
) -> Result<(), String> {
	let config = vr.config.as_ref().ok_or("cannot find SteamVR's config folder (openvrpaths.vrpath)")?;
	let file = config.join("steamvr.vrsettings");
	let text = std::fs::read_to_string(&file).map_err(|e| format!("cannot read {}: {e}", file.display()))?;
	let mut doc = json::parse(&text).ok_or_else(|| format!("{} is not JSON", file.display()))?;
	if backup {
		let backup_dir = home.path(home.at.backups);
		std::fs::create_dir_all(&backup_dir).map_err(|e| format!("cannot make {}: {e}", backup_dir.display()))?;
		let backup = backup_dir.join(format!("steamvr.vrsettings.backup_{}", stamp()));
		std::fs::copy(&file, &backup).map_err(|e| format!("cannot back up {}: {e}", file.display()))?;
		println!("SteamVR's settings backed up to {}", backup.display());
	}
	let mut sections = doc.sections();
	sections.retain(|s| !without.contains(&s.as_str()));
	for (section, key, value) in sets {
		doc.set(section, key, value);
		if !sections.iter().any(|s| s == section) {
			sections.push(section.to_string());
		}
	}
	for section in without {
		doc.remove(section);
	}
	let out = doc.write();
	if json::parse(&out).map(|d| d.sections()) != Some(sections) {
		return Err(format!("{} would not be written whole: left as it is", file.display()));
	}
	write_whole(&file, out.as_bytes())
}

/// A file written whole: to a temporary beside it, then renamed over it
fn write_whole(path: &Path, bytes: &[u8]) -> Result<(), String> {
	let tmp = path.with_extension("tmp");
	std::fs::write(&tmp, bytes).map_err(|e| format!("cannot write {}: {e}", tmp.display()))?;
	std::fs::rename(&tmp, path).map_err(|e| format!("cannot write {}: {e}", path.display()))
}

/// yyyyMMdd_HHmmss, local time
fn stamp() -> String {
	sally_os::localtime::local_now().map(|t| t.stamp()).unwrap_or_else(|| "00000000_000000".into())
}

fn fail(what: &str) -> i32 {
	eprintln!("{what}");
	pause();
	1
}

/// When this program has the console to itself (started from Explorer or a shortcut), it waits for Enter so what it
/// printed stays readable
fn pause() {
	if sys::console_is_ours() {
		println!("Press Enter to close");
		let mut s = String::new();
		let _ = std::io::stdin().read_line(&mut s);
	}
}

/// SteamVR's settings file as a JSON document, rewritten in SteamVR's own layout (three spaces, `"key" : value`); the
/// values' text kept as it was
mod json {
	pub enum Value {
		/// a string, number, true, false or null, as its text
		Text(String),
		Object(Vec<(String, Value)>),
		Array(Vec<Value>),
	}

	pub fn parse(text: &str) -> Option<Value> {
		let b = text.as_bytes();
		let mut i = 0;
		let v = value(b, &mut i)?;
		skip(b, &mut i);
		(i == b.len()).then_some(v)
	}

	fn skip(b: &[u8], i: &mut usize) {
		while *i < b.len() && b[*i].is_ascii_whitespace() {
			*i += 1;
		}
	}

	fn string(b: &[u8], i: &mut usize) -> Option<String> {
		let start = *i;
		*i += 1;
		while *i < b.len() {
			match b[*i] {
				b'\\' => *i += 2,
				b'"' => {
					*i += 1;
					return String::from_utf8(b[start..*i].to_vec()).ok();
				}
				_ => *i += 1,
			}
		}
		None
	}

	fn value(b: &[u8], i: &mut usize) -> Option<Value> {
		skip(b, i);
		match *b.get(*i)? {
			b'{' => {
				*i += 1;
				let mut members = Vec::new();
				skip(b, i);
				if b.get(*i) == Some(&b'}') {
					*i += 1;
					return Some(Value::Object(members));
				}
				loop {
					skip(b, i);
					if b.get(*i) != Some(&b'"') {
						return None;
					}
					let k = string(b, i)?;
					skip(b, i);
					if b.get(*i) != Some(&b':') {
						return None;
					}
					*i += 1;
					let v = value(b, i)?;
					members.push((k, v));
					skip(b, i);
					match b.get(*i)? {
						b',' => *i += 1,
						b'}' => {
							*i += 1;
							return Some(Value::Object(members));
						}
						_ => return None,
					}
				}
			}
			b'[' => {
				*i += 1;
				let mut items = Vec::new();
				skip(b, i);
				if b.get(*i) == Some(&b']') {
					*i += 1;
					return Some(Value::Array(items));
				}
				loop {
					items.push(value(b, i)?);
					skip(b, i);
					match b.get(*i)? {
						b',' => *i += 1,
						b']' => {
							*i += 1;
							return Some(Value::Array(items));
						}
						_ => return None,
					}
				}
			}
			b'"' => string(b, i).map(Value::Text),
			_ => {
				let start = *i;
				while *i < b.len() && !matches!(b[*i], b',' | b'}' | b']') && !b[*i].is_ascii_whitespace() {
					*i += 1;
				}
				(*i > start).then(|| Value::Text(String::from_utf8_lossy(&b[start..*i]).into_owned()))
			}
		}
	}

	/// Text as a JSON string
	pub fn quote(text: &str) -> String {
		let mut out = String::from("\"");
		for c in text.chars() {
			match c {
				'"' => out.push_str("\\\""),
				'\\' => out.push_str("\\\\"),
				'\n' => out.push_str("\\n"),
				'\r' => out.push_str("\\r"),
				'\t' => out.push_str("\\t"),
				c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
				c => out.push(c),
			}
		}
		out.push('"');
		out
	}

	/// A JSON string's text (None: not a string, or an escape that is none)
	pub fn unquote(token: &str) -> Option<String> {
		let inner = token.strip_prefix('"')?.strip_suffix('"')?;
		let mut units: Vec<u16> = Vec::new();
		let mut chars = inner.chars();
		while let Some(c) = chars.next() {
			if c != '\\' {
				units.extend(c.encode_utf16(&mut [0; 2]).iter());
				continue;
			}
			let plain = match chars.next()? {
				'"' => '"',
				'\\' => '\\',
				'/' => '/',
				'b' => '\u{8}',
				'f' => '\u{c}',
				'n' => '\n',
				'r' => '\r',
				't' => '\t',
				'u' => {
					let hex: String = chars.by_ref().take(4).collect();
					if hex.len() != 4 {
						return None;
					}
					units.push(u16::from_str_radix(&hex, 16).ok()?);
					continue;
				}
				_ => return None,
			};
			units.extend(plain.encode_utf16(&mut [0; 2]).iter());
		}
		String::from_utf16(&units).ok()
	}

	impl Value {
		/// `section.key = value` (value as JSON text), the section made when it is not there
		pub fn set(&mut self, section: &str, key: &str, value: &str) {
			let Value::Object(top) = self else { return };
			let sk = format!("\"{section}\"");
			let kk = format!("\"{key}\"");
			let sec = match top.iter().position(|(k, _)| *k == sk) {
				Some(p) => &mut top[p].1,
				None => {
					top.push((sk, Value::Object(Vec::new())));
					&mut top.last_mut().unwrap().1
				}
			};
			if !matches!(sec, Value::Object(_)) {
				*sec = Value::Object(Vec::new());
			}
			let Value::Object(members) = sec else { return };
			match members.iter_mut().find(|(k, _)| *k == kk) {
				Some((_, v)) => *v = Value::Text(value.into()),
				None => members.push((kk, Value::Text(value.into()))),
			}
		}

		/// `section.key`'s value as its JSON text, where it is there
		pub fn get(&self, section: &str, key: &str) -> Option<&str> {
			let Some(Value::Object(members)) = self.member(section) else { return None };
			let kk = format!("\"{key}\"");
			match &members.iter().find(|(k, _)| *k == kk)?.1 {
				Value::Text(t) => Some(t),
				_ => None,
			}
		}

		pub fn has(&self, section: &str) -> bool {
			self.member(section).is_some()
		}

		/// A member of the top object
		pub fn member(&self, key: &str) -> Option<&Value> {
			let Value::Object(top) = self else { return None };
			let kk = format!("\"{key}\"");
			top.iter().find(|(k, _)| *k == kk).map(|(_, v)| v)
		}

		/// A member of the top object set, made when it is not there
		pub fn set_member(&mut self, key: &str, value: Value) {
			let Value::Object(top) = self else { return };
			let kk = format!("\"{key}\"");
			match top.iter_mut().find(|(k, _)| *k == kk) {
				Some((_, v)) => *v = value,
				None => top.push((kk, value)),
			}
		}

		/// The top object's members' names, in order
		pub fn sections(&self) -> Vec<String> {
			let Value::Object(top) = self else { return Vec::new() };
			top.iter().map(|(k, _)| unquote(k).unwrap_or_else(|| k.clone())).collect()
		}

		/// The section taken out, where there is one
		pub fn remove(&mut self, section: &str) {
			let Value::Object(top) = self else { return };
			let sk = format!("\"{section}\"");
			top.retain(|(k, _)| *k != sk);
		}

		pub fn write(&self) -> String {
			let mut out = String::new();
			self.write_at(&mut out, 0);
			out.push('\n');
			out
		}

		fn write_at(&self, out: &mut String, depth: usize) {
			let pad = |n: usize| "   ".repeat(n);
			match self {
				Value::Text(t) => out.push_str(t),
				Value::Object(m) if m.is_empty() => out.push_str("{}"),
				Value::Object(m) => {
					out.push_str("{\n");
					for (n, (k, v)) in m.iter().enumerate() {
						out.push_str(&pad(depth + 1));
						out.push_str(k);
						out.push_str(" : ");
						v.write_at(out, depth + 1);
						if n + 1 < m.len() {
							out.push(',');
						}
						out.push('\n');
					}
					out.push_str(&pad(depth));
					out.push('}');
				}
				Value::Array(a) if a.is_empty() => out.push_str("[]"),
				Value::Array(a) => {
					out.push_str("[\n");
					for (n, v) in a.iter().enumerate() {
						out.push_str(&pad(depth + 1));
						v.write_at(out, depth + 1);
						if n + 1 < a.len() {
							out.push(',');
						}
						out.push('\n');
					}
					out.push_str(&pad(depth));
					out.push(']');
				}
			}
		}
	}
}

/// What setup needs of Windows, declared by hand as the rest of the workspace does: the processes running (Toolhelp),
/// stopping one, a program started on its own, whether the console is this program's alone. Elsewhere: none of it
mod sys {
	use std::path::{Path, PathBuf};

	pub struct Process {
		pub pid: u32,
		pub path: PathBuf,
	}

	/// Whether a process of one of these names (case aside) runs
	pub fn running(names: &[&str]) -> bool {
		processes().iter().any(|p| {
			let n = p.path.file_name().map(|f| f.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
			names.iter().any(|m| n == m.to_ascii_lowercase())
		})
	}

	/// A program started with its folder as the working directory, not waited for, on its own (no console of ours)
	pub fn start_detached(exe: &Path) -> std::io::Result<()> {
		let mut cmd = std::process::Command::new(exe);
		if let Some(d) = exe.parent() {
			cmd.current_dir(d);
		}
		#[cfg(windows)]
		{
			use std::os::windows::process::CommandExt;
			// DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP
			cmd.creation_flags(0x0000_0008 | 0x0000_0200);
		}
		cmd.stdin(std::process::Stdio::null()).stdout(std::process::Stdio::null()).stderr(std::process::Stdio::null());
		cmd.spawn().map(|_| ())
	}

	#[cfg(windows)]
	mod imp {
		use std::ffi::c_void;
		use std::path::PathBuf;

		use super::Process;

		type Handle = *mut c_void;
		const INVALID_HANDLE_VALUE: Handle = -1isize as Handle;
		const TH32CS_SNAPPROCESS: u32 = 0x2;
		const PROCESS_QUERY_LIMITED_INFORMATION: u32 = 0x1000;
		const PROCESS_TERMINATE: u32 = 0x1;
		const SYNCHRONIZE: u32 = 0x0010_0000;
		const MAX_PATH: usize = 260;

		#[repr(C)]
		struct ProcessEntry32W {
			dw_size: u32,
			cnt_usage: u32,
			th32_process_id: u32,
			th32_default_heap_id: usize,
			th32_module_id: u32,
			cnt_threads: u32,
			th32_parent_process_id: u32,
			pc_pri_class_base: i32,
			dw_flags: u32,
			sz_exe_file: [u16; MAX_PATH],
		}

		#[link(name = "kernel32")]
		extern "system" {
			fn CreateToolhelp32Snapshot(flags: u32, pid: u32) -> Handle;
			fn Process32FirstW(snap: Handle, entry: *mut ProcessEntry32W) -> i32;
			fn Process32NextW(snap: Handle, entry: *mut ProcessEntry32W) -> i32;
			fn OpenProcess(access: u32, inherit: i32, pid: u32) -> Handle;
			fn QueryFullProcessImageNameW(process: Handle, flags: u32, name: *mut u16, size: *mut u32) -> i32;
			fn TerminateProcess(process: Handle, code: u32) -> i32;
			fn WaitForSingleObject(handle: Handle, ms: u32) -> u32;
			fn CloseHandle(handle: Handle) -> i32;
			fn GetConsoleProcessList(list: *mut u32, count: u32) -> u32;
		}

		pub fn processes() -> Vec<Process> {
			let mut out = Vec::new();
			// SAFETY: the snapshot is closed below; the entry is a ProcessEntry32W with its size set, as the calls ask
			unsafe {
				let snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
				if snap == INVALID_HANDLE_VALUE {
					return out;
				}
				let mut e: ProcessEntry32W = std::mem::zeroed();
				e.dw_size = std::mem::size_of::<ProcessEntry32W>() as u32;
				let mut more = Process32FirstW(snap, &mut e) != 0;
				while more {
					let n = e.sz_exe_file.iter().position(|&c| c == 0).unwrap_or(MAX_PATH);
					let name = String::from_utf16_lossy(&e.sz_exe_file[..n]);
					let path = full_path(e.th32_process_id).unwrap_or_else(|| PathBuf::from(name));
					out.push(Process { pid: e.th32_process_id, path });
					more = Process32NextW(snap, &mut e) != 0;
				}
				CloseHandle(snap);
			}
			out
		}

		/// The process's program, where it may be asked (not for other users' or protected processes)
		fn full_path(pid: u32) -> Option<PathBuf> {
			// SAFETY: the handle is closed before returning; the buffer and its size are ours
			unsafe {
				let h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
				if h.is_null() {
					return None;
				}
				let mut buf = [0u16; 1024];
				let mut size = buf.len() as u32;
				let ok = QueryFullProcessImageNameW(h, 0, buf.as_mut_ptr(), &mut size) != 0;
				CloseHandle(h);
				ok.then(|| PathBuf::from(String::from_utf16_lossy(&buf[..size as usize])))
			}
		}

		pub fn kill(pid: u32) -> bool {
			// SAFETY: the handle is closed before returning
			unsafe {
				let h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, 0, pid);
				if h.is_null() {
					return false;
				}
				let ok = TerminateProcess(h, 1) != 0;
				if ok {
					WaitForSingleObject(h, 5000);
				}
				CloseHandle(h);
				ok
			}
		}

		pub fn console_is_ours() -> bool {
			let mut list = [0u32; 4];
			// SAFETY: the list and its length are ours
			unsafe { GetConsoleProcessList(list.as_mut_ptr(), list.len() as u32) == 1 }
		}
	}

	#[cfg(windows)]
	pub use imp::{console_is_ours, kill, processes};

	#[cfg(not(windows))]
	pub fn processes() -> Vec<Process> {
		Vec::new()
	}

	#[cfg(not(windows))]
	pub fn kill(_pid: u32) -> bool {
		false
	}

	#[cfg(not(windows))]
	pub fn console_is_ours() -> bool {
		false
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn steamvrs_settings_keep_their_values_and_take_ours() {
		let text = "{\n   \"driver_lighthouse\" : {\n      \"enable\" : false,\n      \"x\" : 1.50\n   },\n   \"steamvr\" : {\n      \"list\" : [ \"a\\\"b\", 2 ]\n   }\n}\n";
		let mut doc = json::parse(text).unwrap();
		doc.set("driver_lighthouse", "enable", "true");
		doc.set("steamvr", "activateMultipleDrivers", "true");
		doc.set("driver_sally", "enable", "true");
		let out = doc.write();
		assert_eq!(
			out,
			"{\n   \"driver_lighthouse\" : {\n      \"enable\" : true,\n      \"x\" : 1.50\n   },\n   \"steamvr\" : {\n      \"list\" : [\n         \"a\\\"b\",\n         2\n      ],\n      \"activateMultipleDrivers\" : true\n   },\n   \"driver_sally\" : {\n      \"enable\" : true\n   }\n}\n"
		);
		assert!(json::parse(&out).is_some());
	}

	#[test]
	fn setup_keeps_the_options_it_does_not_write() {
		let home = Home { root: PathBuf::from(r"C:\P"), at: &PROJECT };
		let old = "driver.extra_latency_ms=4\r\ndriver.hmd=0\r\ndriver.hmd=1\r\n driver.virtual=1\r\n\
		           driver.virtual_trackers=lknee,rknee,lelbow,relbow\r\ndriver.runtime_exe=D:\\old.exe\r\n# driver.x=1\r\nrun\r\n";
		let kept = kept_options(old);
		assert_eq!(kept, ["driver.hmd=1", "driver.virtual=1", "driver.virtual_trackers=lknee,rknee,lelbow,relbow"]);
		let s = driver_settings(&home, "20261111_120000", &kept, 0);
		// setup's own values stand, the others come through once, before the runtime's arguments; the pose's age,
		// which the driver no longer reads there, goes
		assert!(s.contains("driver.lost_timeout_ms=150\r\n") && !s.contains("extra_latency_ms"));
		assert!(!s.contains("old.exe"));
		assert_eq!(s.matches("driver.hmd=1\r\n").count(), 1);
		assert!(s.find("driver.virtual=1").unwrap() < s.find("\r\nrun\r\n").unwrap());
		// and a second setup keeps them again, no more of them
		assert_eq!(kept_options(&s), kept);
	}

	/// A new install keeps the newest few sessions' files; an install from before the option keeps them all, as it
	/// did, until its settings say otherwise, and what they say stays through every later setup
	#[test]
	fn a_setup_never_starts_deleting_an_installs_recordings() {
		let home = Home { root: PathBuf::from(r"C:\P"), at: &PROJECT };
		assert_eq!(keep_sessions(None, PROJECT.keep_sessions), PROJECT.keep_sessions);
		assert_eq!(keep_sessions(None, INSTALLED.keep_sessions), INSTALLED.keep_sessions);
		let before = "driver.hmd=1\r\ndriver.pose_log=C:\\P\\logs\\sally_poses_{time}.csv\r\nrun\r\n";
		assert_eq!(keep_sessions(Some(before), 10), 0);
		assert_eq!(keep_sessions(Some(""), 10), 0);
		for (text, n) in [
			("driver.keep_sessions=25\r\n", 25),
			(" driver.keep_sessions = 3 \r\n", 3),
			("driver.keep_sessions=0\r\n", 0),
			("driver.keep_sessions=-4\r\n", 0),
			("driver.keep_sessions=many\r\n", 0),
			("driver.keep_sessions=5\r\ndriver.keep_sessions=7\r\n", 7),
			("# driver.keep_sessions=9\r\ndriver.keep_sessions_x=9\r\n", 0),
		] {
			assert_eq!(keep_sessions(Some(text), 10), n, "{text:?}");
		}
		// the line is setup's own: written once with the value carried, never among the kept ones
		for (old, n) in [(None, PROJECT.keep_sessions), (Some(before), 0), (Some("driver.keep_sessions=25\r\n"), 25)] {
			let kept = kept_options(old.unwrap_or_default());
			let s = driver_settings(&home, "20261116_120000", &kept, keep_sessions(old, PROJECT.keep_sessions));
			assert_eq!(s.matches("driver.keep_sessions=").count(), 1, "{old:?}");
			assert!(s.contains(&format!("driver.keep_sessions={n}\r\n")), "{old:?}");
			assert_eq!(keep_sessions(Some(&s), 10), n, "a second setup keeps it");
		}
	}

	#[test]
	fn uninstall_takes_sallys_section_out_of_steamvrs_settings() {
		let text = "{\n   \"driver_sally\" : {\n      \"enable\" : true\n   },\n   \"steamvr\" : {\n      \"activateMultipleDrivers\" : true\n   }\n}\n";
		let mut doc = json::parse(text).unwrap();
		doc.set("driver_lighthouse", "enable", "true");
		doc.remove("driver_sally");
		doc.remove("driver_none");
		assert_eq!(
			doc.write(),
			"{\n   \"steamvr\" : {\n      \"activateMultipleDrivers\" : true\n   },\n   \"driver_lighthouse\" : {\n      \"enable\" : true\n   }\n}\n"
		);
	}

	#[test]
	fn the_driver_settings_name_sally_run() {
		let home = Home { root: PathBuf::from(r"C:\P"), at: &PROJECT };
		let s = driver_settings(&home, "20261109_150000", &[], PROJECT.keep_sessions);
		assert!(s.contains("driver.runtime_exe=C:\\P\\survive\\hidtap\\sally.exe\r\n"));
		assert!(s.contains("# its arguments, one a line\r\nrun\r\n--hidtap\r\n1\r\n"));
		assert!(s.ends_with("--light-facing-deg\r\n115\r\n"));
		for gone in ["--core2", "--pose-udp", "--kalman-acc-cal-file", "--threaded-posers", "--gss-threaded"] {
			assert!(!s.contains(&format!("{gone}\r\n")), "{gone}");
		}
	}
	/// The installer's folder: everything beside sally.exe, no pose log, the panel started by the driver
	#[test]
	fn the_installed_settings_name_the_installers_folder() {
		let home = Home { root: PathBuf::from(r"C:\Users\u\AppData\Local\sally"), at: &INSTALLED };
		let s = driver_settings(&home, "20261117_120000", &[], INSTALLED.keep_sessions);
		let root = r"C:\Users\u\AppData\Local\sally";
		for line in [
			format!("driver.runtime_exe={root}\\sally.exe"),
			format!("driver.runtime_cwd={root}\\data\\devices"),
			format!("driver.panel_exe={root}\\sally-panel.exe"),
			format!("--configfile\r\n{root}\\data\\driver_config.json"),
			format!("--hidtap-record\r\n{root}\\logs\\sallytap_{{time}}.hidtap"),
			"driver.keep_sessions=2".to_string(),
			"--hidtap-config-fetch\r\n0".to_string(),
		] {
			assert!(s.contains(&format!("{line}\r\n")), "{line}");
		}
		assert!(!s.contains("driver.pose_log"));
		assert_eq!(home.path(""), PathBuf::from(root));
		// the panel's line is setup's own: a second setup does not keep a copy of it
		assert!(kept_options(&s).is_empty());
	}

	#[test]
	fn only_what_keeps_sally_from_loading_is_changed_in_steamvrs_settings() {
		let fine = json::parse(
			"{ \"steamvr\" : { \"activateMultipleDrivers\" : true }, \"driver_sally\" : { \"enable\" : true } }",
		);
		assert!(blocking(&fine.unwrap()).is_empty());
		assert!(blocking(&json::parse("{}").unwrap()).is_empty());
		let off = json::parse(
			"{ \"driver_lighthouse\" : { \"enable\" : false }, \"driver_sally\" : { \"blocked_by_safe_mode\" : true, \"enable\" : false } }",
		);
		assert_eq!(
			blocking(&off.unwrap()),
			[
				("driver_lighthouse", "enable", "true"),
				("driver_sally", "enable", "true"),
				("driver_sally", "blocked_by_safe_mode", "false")
			]
		);
	}

	#[test]
	fn steamvrs_list_of_drivers_reads_and_writes() {
		let text = "{\n\t\"config\" : [\n\t\t\"C:\\\\Program Files (x86)\\\\Steam\\\\config\"\n\t],\n\t\"external_drivers\" : [\n\t\t\"C:\\\\a b\\\\sally\",\n\t\t\"D:\\\\caf\\u00e9\\\\other\"\n\t],\n\t\"jsonid\" : \"vrpathreg\",\n\t\"version\" : 1\n}\n";
		let mut reg = Registry { file: PathBuf::new(), doc: json::parse(text).unwrap() };
		assert_eq!(reg.drivers(), [PathBuf::from(r"C:\a b\sally"), PathBuf::from("D:\\caf\u{e9}\\other")]);
		reg.doc
			.set_member("external_drivers", json::Value::Array(vec![json::Value::Text(json::quote(r"C:\new\driver"))]));
		assert_eq!(reg.drivers(), [PathBuf::from(r"C:\new\driver")]);
		assert_eq!(reg.doc.sections(), ["config", "external_drivers", "jsonid", "version"]);
		// a list that is null (no drivers yet), and none at all
		let none = Registry { file: PathBuf::new(), doc: json::parse("{ \"external_drivers\" : null }").unwrap() };
		assert!(none.drivers().is_empty());
		assert_eq!(json::unquote(&json::quote("a\"b\\c\n")).as_deref(), Some("a\"b\\c\n"));
		assert_eq!(json::unquote("\"\\ud83d\\ude00\"").as_deref(), Some("\u{1f600}"));
	}
}
