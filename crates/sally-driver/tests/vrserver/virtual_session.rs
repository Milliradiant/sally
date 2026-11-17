//! The virtual trackers' sessions (driver.virtual, virtual_trackers.rs): the driver in the stand-in vrserver with the
//! virtual trackers and the model process started by the driver (this test binary, scripted: model.rs, which answers
//! with all five), the headset and Valve's four devices reporting as in the other sessions, no runtime. Three run: two
//! with the switch kept on (sally_virtual.txt `on=1`), one with all five (driver.virtual_trackers unset) and one with
//! four of them (no chest: the model's chest lines dropped); and one without the file, off at the start (the default),
//! which first runs 1.5 s with none of them added and no model process, then is switched on (`virtual on`) and goes on
//! as the others. From the first pose of the ones shown that vrserver gets (V):
//!
//! - V + 1: `virtual` (the X line); V + 1.5 `virtual off` (the process stopped); V + 2.5 `virtual on` (started again);
//!   V + 3 `virtual calibrate`
//! - V + 3.5 to V + 4.5: the model quiet (it answers nothing)
//! - V + 5: the model process killed; the driver starts it again
//! - 3 s after its poses are back: the end, Cleanup
//!
//! The checks: the ones shown added and activated with their properties, none other, the R lines the model got (the
//! headset's and Valve's devices, by class; not the base station, not the five), the poses SteamVR got (the model's,
//! handed over as they came: identity world-from-driver, their age declared), off reported disconnected with the process
//! stopped and on tracking again, quiet out of range, calibrate passed on, the process started again, the switch kept
//! in sally_virtual.txt, GetPose, and nothing left running; for the one off at the start, nothing of them before on.

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
use std::sync::{Mutex, RwLock};
use std::time::Duration;

use driver_sally::openvr::DriverPose_t;
use sally_math::math3d::{len, V3};

use crate::checks::Checks;
use crate::host::{device_get_pose, v3, Call, Server, FIRST_ADDED};
use crate::model::path;
use crate::scene::DEVICES;
use crate::session::{command_lines, free_port, install, open, other_drivers, sleep_until, wait_for, Fed, Schedule};

/// A virtual session: its name, which of the five it shows (the model's order: chest, lknee, rknee, lelbow, relbow) and
/// the driver.virtual_trackers line that says so (None: unset, all five)
#[derive(Clone, Copy)]
pub struct VConfig {
	pub name: &'static str,
	pub shown: &'static [usize],
	pub option: Option<&'static str>,
	/// sally_virtual.txt `on=1` written before the start (else no file: off at the start)
	pub start_on: bool,
}

const SERIALS: [&str; 5] = ["SALLY-CHEST", "SALLY-LKNEE", "SALLY-RKNEE", "SALLY-LELBOW", "SALLY-RELBOW"];
const NAMES: [&str; 5] = ["chest", "lknee", "rknee", "lelbow", "relbow"];

pub struct VRecord {
	pub config: VConfig,
	pub server: &'static Server,
	pub t0: i64,
	pub dir: PathBuf,
	/// when the first pose of the five came (V), and the schedule's moments
	pub first: Option<f64>,
	pub off: f64,
	pub on: f64,
	pub quiet: (f64, f64),
	pub kill: f64,
	pub back: Option<f64>,
	pub end: f64,
	/// (command, reply)
	pub replies: Vec<(String, String)>,
	/// the model's consoles and its runs' starts (session time)
	pub consoles: Vec<String>,
	pub starts: Vec<f64>,
	/// GetPose for each of the five after the end, and the last pose vrserver got for it
	pub get_pose: Vec<(DriverPose_t, Option<DriverPose_t>)>,
	pub left_over: Vec<u32>,
	/// off at the start: after 1.5 s, the X line, the devices added and the model processes started by then
	pub prelude: Option<(String, usize, usize)>,
	/// sally_virtual.txt at the end
	pub switch_file: String,
}

/// The ids of the ones shown: the first devices added, in order
fn ids(cfg: &VConfig) -> Vec<u32> {
	(0..cfg.shown.len()).map(|i| FIRST_ADDED + i as u32).collect()
}

pub fn run(cfg: VConfig) -> VRecord {
	let t0 = sally_os::chrono::steady_clock_now();
	let server = Server::new(t0);
	let now = || server.now();
	let exe = std::env::current_exe().unwrap();
	let (dir, drv, library) = install(cfg.name, &exe);
	std::fs::write(dir.join("t0"), t0.to_string()).unwrap();
	let cmd_port = free_port();
	let model_port = free_port();
	let panel = UdpSocket::bind("127.0.0.1:0").unwrap();
	let mut settings = vec![
		"# the vrserver test's: the virtual trackers, no runtime".to_string(),
		format!("driver.runtime_port={}", free_port()),
		format!("driver.valve_udp={}", panel.local_addr().unwrap().port()),
		format!("driver.cmd_port={cmd_port}"),
		"driver.virtual=1".into(),
		format!("driver.virtual_port={model_port}"),
		format!("driver.virtual_exe={}", exe.display()),
		format!("driver.virtual_model={}", dir.display()),
		format!("driver.virtual_log={}", dir.join("model_{time}.log").display()),
	];
	settings.extend(cfg.option.map(|o| format!("driver.virtual_trackers={o}")));
	std::fs::write(drv.join("bin/linux64/sally_settings.txt"), settings.join("\n") + "\n").unwrap();
	let switch = drv.join("bin/linux64/sally_virtual.txt");
	if cfg.start_on {
		std::fs::write(&switch, "on=1\n").unwrap();
	} else {
		let _ = std::fs::remove_file(&switch);
	}
	{
		let mut props = server.props.lock().unwrap();
		let mut dongles = server.dongles.lock().unwrap();
		let mut classes = server.classes.lock().unwrap();
		for d in &DEVICES {
			props.insert(d.id, ("lighthouse", d.serial));
			dongles.insert(d.id, d.dongle);
			classes.insert(d.id, if d.subtype.starts_with("knuckles") { 2 } else { 3 });
		}
		props.insert(0, ("oculus", "1WMHH000X00000"));
		classes.insert(0, 1);
		props.insert(5, ("lighthouse", "LHB-1A2B3C4D"));
		classes.insert(5, 4);
	}
	let (provider, _) = open(&library);
	assert_eq!(provider.init(server.context()), 0, "Init");

	let running = AtomicBool::new(true);
	let schedule: RwLock<Option<Schedule>> = RwLock::new(None);
	let fed: Mutex<Vec<Fed>> = Mutex::new(Vec::new());
	let client = UdpSocket::bind("127.0.0.1:0").unwrap();
	client.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
	let ids = ids(&cfg);
	let ours = |c: &Call| ids.contains(&c.id);
	let first_valid = |after: f64| {
		server.calls.lock().unwrap().iter().find(|c| ours(c) && c.t > after && c.pose.poseIsValid != 0).map(|c| c.t)
	};
	let mut replies = Vec::new();
	let mut prelude = None;
	let (mut first, mut back) = (None, None);
	let (mut off, mut on, mut quiet, mut kill) = (f64::NAN, f64::NAN, (f64::NAN, f64::NAN), f64::NAN);
	std::thread::scope(|s| {
		let frames = s.spawn(|| {
			while running.load(SeqCst) {
				provider.run_frame();
				std::thread::sleep(Duration::from_micros(11111));
			}
		});
		let others = s.spawn(|| other_drivers(server, &schedule, &fed, &running));
		if !cfg.start_on {
			sleep_until(&now, 1.5);
			let x = x_line(&client, cmd_port, "virtual");
			prelude = Some((x, server.added_devices.lock().unwrap().len(), model_pids(&dir).len()));
			replies.push(("virtual on (at the start)".to_string(), x_line(&client, cmd_port, "virtual on")));
		}
		if wait_for(20.0, || first_valid(0.0).is_some()) {
			let v = first_valid(0.0).unwrap();
			first = Some(v);
			sleep_until(&now, v + 1.0);
			replies.push(("virtual".to_string(), x_line(&client, cmd_port, "virtual")));
			sleep_until(&now, v + 1.5);
			off = now();
			replies.push(("virtual off".to_string(), x_line(&client, cmd_port, "virtual off")));
			sleep_until(&now, v + 2.5);
			on = now();
			replies.push(("virtual on".to_string(), x_line(&client, cmd_port, "virtual on")));
			sleep_until(&now, v + 3.0);
			replies.push(("virtual calibrate".to_string(), x_line(&client, cmd_port, "virtual calibrate")));
			sleep_until(&now, v + 3.5);
			std::fs::write(dir.join("quiet"), "").unwrap();
			quiet.0 = now();
			sleep_until(&now, v + 4.5);
			std::fs::remove_file(dir.join("quiet")).unwrap();
			quiet.1 = now();
			sleep_until(&now, v + 5.0);
			kill = now();
			if let Some(pid) = model_pids(&dir).last() {
				// SAFETY: a signal to the model process
				unsafe { libc_kill(*pid as i32, 9) };
			}
			// back: valid poses again after the driver started it again (at most 5 s after its last start)
			if wait_for(15.0, || model_pids(&dir).len() >= 2 && first_valid(kill + 0.5).is_some()) {
				back = first_valid(kill + 0.5);
				sleep_until(&now, back.unwrap() + 3.0);
			}
		}
		running.store(false, SeqCst);
		others.join().unwrap();
		frames.join().unwrap();
	});
	let end = now();
	// GetPose through each device's vtable, against the last pose vrserver got for it
	let added = server.added_devices.lock().unwrap().clone();
	let calls = server.calls.lock().unwrap().clone();
	let get_pose = added
		.iter()
		.take(cfg.shown.len())
		.map(|a| (device_get_pose(a.device), calls.iter().rev().find(|c| c.id == a.id).map(|c| c.pose)))
		.collect();
	provider.cleanup();
	let pids = model_pids(&dir);
	let left_over = pids.iter().filter(|p| Path::new(&format!("/proc/{p}")).exists()).copied().collect();
	let text: String = server.log.lock().unwrap().iter().map(|(t, l)| format!("{t:10.4} {l}\n")).collect();
	std::fs::write(dir.join("vrserver.txt"), text).unwrap();
	let mut consoles: Vec<(String, String)> = std::fs::read_dir(&dir)
		.unwrap()
		.flatten()
		.filter_map(|e| {
			let name = e.file_name().to_string_lossy().into_owned();
			name.starts_with("model_").then(|| (name, std::fs::read_to_string(e.path()).unwrap_or_default()))
		})
		.collect();
	consoles.sort();
	let mut starts: Vec<f64> = pids
		.iter()
		.filter_map(|p| std::fs::read_to_string(dir.join(format!("model-{p}.txt"))).ok())
		.filter_map(|t| t.trim().strip_prefix("start ").and_then(|x| x.parse().ok()))
		.collect();
	starts.sort_by(f64::total_cmp);
	let switch_file = std::fs::read_to_string(&switch).unwrap_or_default();
	VRecord {
		config: cfg,
		server,
		t0,
		dir,
		first,
		off,
		on,
		quiet,
		kill,
		back,
		end,
		replies,
		consoles: consoles.into_iter().map(|c| c.1).collect(),
		starts,
		get_pose,
		left_over,
		prelude,
		switch_file,
	}
}

/// A `virtual` command's X line (its reply is the X line, then the status line)
fn x_line(sock: &UdpSocket, port: u16, cmd: &str) -> String {
	command_lines(sock, port, cmd).into_iter().find(|l| l.starts_with("X ")).unwrap_or_default()
}

extern "C" {
	#[link_name = "kill"]
	fn libc_kill(pid: i32, sig: i32) -> i32;
}

/// The model processes' pids, from their markers, in the order they started
fn model_pids(dir: &Path) -> Vec<u32> {
	let mut v: Vec<(f64, u32)> = std::fs::read_dir(dir)
		.unwrap()
		.flatten()
		.filter_map(|e| {
			let name = e.file_name().to_string_lossy().into_owned();
			let pid: u32 = name.strip_prefix("model-")?.strip_suffix(".txt")?.parse().ok()?;
			let t: f64 = std::fs::read_to_string(e.path()).ok()?.trim().strip_prefix("start ")?.parse().ok()?;
			Some((t, pid))
		})
		.collect();
	v.sort_by(|a, b| a.0.total_cmp(&b.0));
	v.into_iter().map(|x| x.1).collect()
}

pub fn check(r: &VRecord) -> Checks {
	let mut c = Checks { lines: Vec::new() };
	let s = r.server;
	let log = s.log.lock().unwrap().clone();
	let calls = s.calls.lock().unwrap().clone();
	let has = |text: &str| log.iter().any(|l| l.1.contains(text));
	let cfg = &r.config;
	let ids = ids(cfg);
	let n = cfg.shown.len();
	let added = s.added_devices.lock().unwrap().clone();
	let written = s.written.lock().unwrap().clone();

	let serials: Vec<&str> = added.iter().map(|a| a.serial.as_str()).collect();
	let shown: Vec<&str> = cfg.shown.iter().map(|&k| SERIALS[k]).collect();
	c.check(
		serials == shown && added.iter().all(|a| a.class == 3 && a.activated == 0),
		format!("the ones shown added as generic trackers and activated, no other: {serials:?}"),
	);
	// each one's properties, in its container
	let prop = |id: u32, p: i32| -> Option<Vec<u8>> {
		written.get(&(1000 + id as u64)).and_then(|w| w.iter().rev().find(|x| x.0 == p).map(|x| x.2.clone()))
	};
	let string =
		|id: u32, p: i32| prop(id, p).map(|b| String::from_utf8_lossy(&b[..b.len().saturating_sub(1)]).into_owned());
	let types = [
		"vive_tracker_chest",
		"vive_tracker_left_knee",
		"vive_tracker_right_knee",
		"vive_tracker_left_elbow",
		"vive_tracker_right_elbow",
	];
	let props_ok = ids.iter().zip(cfg.shown).all(|(&id, &k)| {
		string(id, 1000).as_deref() == Some("oculus")
			&& string(id, 1002).as_deref() == Some(SERIALS[k])
			&& string(id, 1003).as_deref() == Some("{htc}vr_tracker_vive_1_0")
			&& string(id, 1037).as_deref() == Some("{htc}/input/vive_tracker_profile.json")
			&& string(id, 7000).as_deref() == Some(types[k])
			&& string(id, 1005).as_deref() == Some("sally")
			&& prop(id, 3007) == Some(3i32.to_ne_bytes().to_vec())
			&& [1026, 1004, 1043].iter().all(|&p| prop(id, p) == Some(vec![0]))
			&& (5001..=5008).all(|p| string(id, p).is_some_and(|v| v.starts_with("{htc}/icons/tracker_status_")))
	});
	c.check(
		props_ok,
		"each one's properties: the headset's tracking system (oculus), its serial, Valve's tracker render model, \
		 input profile and icons, its role's controller type, opted out of the hands, no battery"
			.into(),
	);

	// what the model got
	let console = r.consoles.join("\n");
	let want = [
		"R hmd 1WMHH000X00000",
		"R tracker LHR-0A1B2C3D",
		"R tracker LHR-11223344",
		"R controller LHR-55667788",
		"R controller LHR-99AABBCC",
	];
	let leaked =
		console.contains("SALLY-") || console.contains("LHB-1A2B3C4D") || console.contains("got something else");
	c.check(
		want.iter().all(|w| console.contains(w)) && !leaked,
		"the model got SteamVR's poses of the headset and Valve's four, each with its class (no virtual one, no base \
		 station, nothing it could not read)"
			.into(),
	);

	// the poses SteamVR got for the ones shown: the model's, as they came (the model answers for all five)
	let t0s = r.t0 as f64 / 1e9;
	let mine: Vec<&Call> = calls.iter().filter(|x| ids.contains(&x.id)).collect();
	let valid: Vec<&Call> = mine.iter().copied().filter(|x| x.pose.poseIsValid != 0).collect();
	let mut errs = Vec::new();
	let mut frames_ok = true;
	for x in &valid {
		let k = cfg.shown[(x.id - FIRST_ADDED) as usize];
		let p = &x.pose;
		let id_q = |q: &driver_sally::openvr::HmdQuaternion_t| q.w == 1.0 && q.x == 0.0 && q.y == 0.0 && q.z == 0.0;
		frames_ok &= id_q(&p.qWorldFromDriverRotation)
			&& p.vecWorldFromDriverTranslation == [0.0; 3]
			&& id_q(&p.qDriverFromHeadRotation)
			&& p.vecDriverFromHeadTranslation == [0.0; 3]
			&& p.deviceIsConnected == 1
			&& p.result == 200
			&& (-0.05..=0.0).contains(&p.poseTimeOffset);
		// the frame's time, the driver's clock: when vrserver got it, less the age declared
		let (want, _, _) = path(k, x.t + t0s + p.poseTimeOffset);
		errs.push(len(v3(&p.vecPosition) - want));
	}
	errs.sort_by(f64::total_cmp);
	let p99 = errs.get(errs.len() * 99 / 100).copied().unwrap_or(f64::NAN);
	c.check(
		valid.len() > 500 && frames_ok && p99 < 2e-3,
		format!(
			"SteamVR got the model's poses for the ones shown ({} valid): identity world-from-driver and \
			 driver-from-head, connected, their age declared (at most 50 ms), where the model put them at that age (p99 {:.2} mm)",
			valid.len(),
			1e3 * p99
		),
	);

	// off, on: disconnected at once (and again every second), nothing else until on
	let between = |a: f64, b: f64| -> Vec<&Call> { mine.iter().copied().filter(|x| x.t > a && x.t < b).collect() };
	let off_calls = between(r.off + 0.03, r.on);
	let all_off = off_calls.iter().all(|x| x.pose.deviceIsConnected == 0 && x.pose.poseIsValid == 0)
		&& ids
			.iter()
			.all(|id| between(r.off, r.off + 0.05).iter().any(|x| x.id == *id && x.pose.deviceIsConnected == 0));
	c.check(
		all_off,
		format!(
			"switched off: each one reported disconnected within 50 ms, and nothing else until on ({} poses)",
			off_calls.len()
		),
	);
	c.check(
		has("virtual trackers switched off (disconnected until SteamVR starts again)")
			&& has("virtual trackers: stopping the model process"),
		"switched off: the model process stopped, the log saying they stay disconnected until SteamVR starts again"
			.into(),
	);
	let on_again = ids
		.iter()
		.all(|id| mine.iter().any(|x| x.id == *id && x.t > r.on && x.t < r.on + 2.0 && x.pose.poseIsValid != 0));
	c.check(on_again, "switched on again: the model process started again, each tracking within 2 s".into());
	c.check(
		r.switch_file.lines().any(|l| l == "on=1") && !r.switch_file.lines().any(|l| l == "on=0"),
		format!("the switch kept in sally_virtual.txt (on, as last switched): {:?}", r.switch_file),
	);
	if let Some((x, added_then, pids_then)) = &r.prelude {
		c.check(
			x.starts_with("X off devices 0 tracking 0 ")
				&& *added_then == 0
				&& *pids_then == 0
				&& has("virtual trackers off (")
				&& r.first.is_some_and(|v| v > 1.5),
			format!(
				"off at the start (no sally_virtual.txt): none added, no model process, until switched on ({x:?}, {added_then} added, {pids_then} processes)"
			),
		);
	}
	let x_lines: Vec<&str> =
		r.replies.iter().filter(|x| x.0 != "virtual on (at the start)").map(|x| x.1.as_str()).collect();
	let names = cfg.shown.iter().map(|&k| NAMES[k]).collect::<Vec<_>>().join(",");
	c.check(
		x_lines
			.first()
			.is_some_and(|l| l.starts_with(&format!("X on devices {n} tracking {n} trackers {names} model tracking")))
			&& x_lines
				.get(1)
				.is_some_and(|l| l.starts_with(&format!("X off devices {n} tracking 0 ")) && l.ends_with("model none"))
			&& x_lines.get(2).is_some_and(|l| l.starts_with(&format!("X on devices {n} "))),
		format!("the X lines answer `virtual`: {x_lines:?}"),
	);
	c.check(
		console.contains("got C calibrate") && has("the T-pose asked for again"),
		"`virtual calibrate` passed on to the model".into(),
	);

	// quiet: out of range once it is stale (0.25 s), and nothing else until the model answers again
	let q = between(r.quiet.0 + 0.35, r.quiet.1);
	let out_of_range =
		q.iter().all(|x| x.pose.deviceIsConnected == 1 && x.pose.poseIsValid == 0 && x.pose.result == 201)
			&& ids.iter().all(|id| {
				between(r.quiet.0 + 0.2, r.quiet.0 + 0.35).iter().any(|x| x.id == *id && x.pose.result == 201)
			});
	c.check(
		out_of_range,
		format!(
			"the model quiet: each one out of range (connected) 0.25 s on, nothing else until it answers ({} poses)",
			q.len()
		),
	);
	// the process: started when on, again when switched on after off, again after the kill, none left
	c.check(
		r.starts.len() >= 3
			&& r.starts.iter().any(|&t| t > r.on && t < r.kill)
			&& r.starts.iter().any(|&t| t > r.kill)
			&& has("the model process exited")
			&& r.back.is_some(),
		format!(
			"the model process started by the driver, again when switched on, and again after it was killed ({:?}; tracking again at {:?})",
			r.starts, r.back
		),
	);
	c.check(r.left_over.is_empty(), format!("no model process left after Cleanup: {:?}", r.left_over));
	let gp_ok = r.get_pose.len() == n
		&& r.get_pose
			.iter()
			.all(|(g, last)| last.is_some_and(|l| l.vecPosition == g.vecPosition && l.qRotation == g.qRotation));
	c.check(gp_ok, "GetPose: each one's last pose".into());
	let named: Vec<String> = (cfg.start_on)
		.then(|| format!("sally: virtual trackers on: {} from the model process", shown.join(", ")))
		.into_iter()
		.chain(shown.iter().enumerate().map(|(i, w)| {
			format!(
				"sally: virtual tracker {w} is device {} (tracking system oculus, the headset's)",
				FIRST_ADDED as usize + i
			)
		}))
		.collect();
	let missing: Vec<&String> = named.iter().filter(|l| !has(l)).collect();
	c.check(missing.is_empty(), format!("the log names the ones shown, and each one's device (missing: {missing:?})"));
	// the ones not shown: never added, so their lines from the model go nowhere (no pose for any other device id)
	let strays: Vec<u32> = calls.iter().map(|x| x.id).filter(|&id| id >= FIRST_ADDED && !ids.contains(&id)).collect();
	c.check(strays.is_empty(), format!("no pose for a virtual tracker not shown: {} ({strays:?})", 5 - n));
	let _ = V3::default();
	let _ = &r.dir;
	let _ = r.end;
	let _ = r.first;
	c
}
