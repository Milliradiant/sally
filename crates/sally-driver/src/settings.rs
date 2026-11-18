//! sally_settings.txt, next to the driver (`sally setup` writes it): `driver.<name>=<value>` lines for the driver, and
//! every other line that is not a comment one argument for the runtime the driver starts. g_opt is read when SteamVR
//! starts the driver (Init) and only read after; it keeps what it read across Inits.

use std::sync::{Arc, LazyLock, RwLock};

use sally_math::printf::BArg;
use sally_math::{c_atof, c_atoi};

use crate::cstdio::CFile;
use crate::helpers::{log, parse_bool, trim, K_SEP};

/// The driver's options (Options), their defaults the C++'s
#[derive(Clone, Debug, PartialEq)]
pub struct Options {
	// pose math on the runtime's reports
	pub velocity: bool,
	pub angular_velocity: bool,
	pub time_offset: bool,
	pub lever_arm: bool,
	pub stats: bool,
	/// carry the pose over its age at the hand-over with the IMU's acceleration (the runtime's Q line), and hand it
	/// to SteamVR as of now: SteamVR's straight line then covers only its own prediction (docs/sally_driver.md, 0.8)
	pub carry: bool,
	/// Valve's driver stops updating a controller while its accelerometer is at full scale (8 g: the stop of a
	/// punch), 8-40 ms; SteamVR then carries the last pose on at full speed and the hand flies past the stop and snaps
	/// back. With sally shown steadily, her pose is pushed when Valve has not called for this long (ms; 0 = only after
	/// 30 ms)
	pub push_quiet_ms: f64,
	/// a lighthouse headset (vrserver's device 0) gets sally's pose too, unless the panel's Headset tracking says
	/// Valve's (user.rs); off, SteamVR always has Valve's for the headset and nothing of the runtime's is taken
	pub hmd: bool,
	/// no runtime pose for this long: sally has lost the device
	pub lost_timeout_ms: f64,
	// runtime link
	pub runtime_host: Vec<u8>,
	pub runtime_port: i32,
	// panels: Valve's poses and the status go out, "seen ..." commands come in
	/// ports on 127.0.0.1 that get Valve's poses ("V" lines) and the status ("C" lines)
	pub valve_udp: Vec<u8>,
	/// commands: "seen sally", "seen valve", "seen toggle", "status"
	pub cmd_port: i32,
	/// whose poses SteamVR shows at start
	pub seen: Vec<u8>,
	/// seconds of sally/Valve pairs the frame fit uses
	pub fit_window_s: f64,
	/// the frame fit's tilt from gravity, not from the pairs (`level` in pose_override.rs): the runtime's up on the
	/// apps' up with a headset in its own space, on the room setup's long-run up with a lighthouse headset
	pub level: bool,
	/// sally's pose on Valve's own record (`valve_record` in pose_override.rs): the driver pose at Valve's IMU origin
	/// with Valve's driver-from-head, not the head pose with none. What SteamVR shows is the same either way; what
	/// reads the driver pose without the driver-from-head (Space Calibrator) then sees the device where Valve's own
	/// pose has it
	pub valve_record: bool,
	/// fallback (design doc 5): when sally has no fresh pose for a device, SteamVR gets Valve's pose for it instead of
	/// "out of range"; every switch between the two sources is blended over blend_ms rather than cut
	pub fallback: bool,
	/// sally pose older than this = sally has nothing for the device right now
	pub fallback_after_ms: f64,
	pub blend_ms: f64,
	/// Valve's pose counts as a usable fallback only once it has been plausible for this long (right after SteamVR
	/// starts, Valve's poses sat 3 m below the floor for ~20 s)
	pub valve_settle_s: f64,
	/// while we have a fresh pose, Valve's must be within this of it to count as plausible
	pub valve_agree_m: f64,
	/// sally stale for this long: Valve's valid pose is used even if the plausibility gate says no (a frozen tracker
	/// is worse than any jump)
	pub fallback_force_ms: f64,
	/// the newest this many of each kind of session file are kept when SteamVR starts, the older ones deleted
	/// (`sessions`: the pose log, the runtime's recording and log, each by its `{time}` name); 0 = all are kept
	pub keep_sessions: i32,
	/// per frame what we sent, what SteamVR predicts from it, and Valve's own pose (csv; {time} = session start)
	pub pose_log: Vec<u8>,
	// the runtime process (0.7): started by the driver when SteamVR starts, unless one already answers
	/// the runtime, sally.exe (its first argument, `run`, is the first of runtime_args); empty = the driver never starts
	/// a runtime
	pub runtime_exe: Vec<u8>,
	/// its working directory (the tap loads <dongle>_config.json from there)
	pub runtime_cwd: Vec<u8>,
	/// its console output ({time} = session start); empty = discarded
	pub runtime_log: Vec<u8>,
	/// start it again when it exits while SteamVR runs
	pub runtime_restart: bool,
	/// send it "quit" when SteamVR exits (it closes its recordings)
	pub runtime_stop: bool,
	/// every non-driver line of sally_settings.txt, {time} expanded at each start
	pub runtime_args: Vec<Vec<u8>>,
	/// the panel, started with SteamVR (panel.rs); empty = the driver starts none
	pub panel_exe: Vec<u8>,
}

impl Default for Options {
	fn default() -> Options {
		Options {
			velocity: true,
			angular_velocity: true,
			time_offset: true,
			lever_arm: true,
			stats: true,
			carry: false,
			push_quiet_ms: 6.0,
			hmd: true,
			lost_timeout_ms: 500.0,
			runtime_host: b"127.0.0.1".to_vec(),
			runtime_port: 9100,
			valve_udp: b"9102,9103".to_vec(),
			cmd_port: 9104,
			seen: b"sally".to_vec(),
			fit_window_s: 60.0,
			level: true,
			valve_record: true,
			fallback: true,
			fallback_after_ms: 100.0,
			blend_ms: 150.0,
			valve_settle_s: 2.0,
			valve_agree_m: 0.3,
			fallback_force_ms: 2000.0,
			keep_sessions: 0,
			pose_log: Vec::new(),
			runtime_exe: Vec::new(),
			runtime_cwd: Vec::new(),
			runtime_log: Vec::new(),
			runtime_restart: true,
			runtime_stop: true,
			runtime_args: Vec::new(),
			panel_exe: Vec::new(),
		}
	}
}

static G_OPT: LazyLock<RwLock<Arc<Options>>> = LazyLock::new(|| RwLock::new(Arc::new(Options::default())));

/// g_opt as it stands
pub fn g_opt() -> Arc<Options> {
	G_OPT.read().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The options from now on (what was read from the file, or a test's)
pub fn set_g_opt(o: Options) {
	*G_OPT.write().unwrap_or_else(|e| e.into_inner()) = Arc::new(o);
}

/// One "driver.<name>=<value>" line (s starts with "driver."); no '=' means "1"
pub fn parse_option(o: &mut Options, s: &[u8]) {
	let eq = s.iter().position(|&c| c == b'=');
	let k = trim(&s[7..eq.unwrap_or(s.len())]);
	let v: &[u8] = match eq {
		None => b"1",
		Some(eq) => trim(&s[eq + 1..]),
	};
	match k {
		b"velocity" => o.velocity = parse_bool(v),
		b"angular_velocity" => o.angular_velocity = parse_bool(v),
		b"time_offset" => o.time_offset = parse_bool(v),
		b"lever_arm" => o.lever_arm = parse_bool(v),
		b"carry" => o.carry = parse_bool(v),
		b"push_quiet_ms" => o.push_quiet_ms = c_atof(v),
		b"stats" => o.stats = parse_bool(v),
		// the pose's declared age is the output layer's prediction_ms since 0.11 (sally_output.txt, per class and
		// device); a settings file from before still names it
		b"extra_latency_ms" => log(
			"sally: driver.extra_latency_ms=%s is not read any more: the pose's age is the output layer's prediction_ms \
			 (sally_output.txt, the panel's OUTPUT)\n",
			&[BArg::S(v)],
		),
		b"hmd" => o.hmd = parse_bool(v),
		b"lost_timeout_ms" => o.lost_timeout_ms = c_atof(v),
		b"runtime_host" => o.runtime_host = v.to_vec(),
		b"runtime_port" => o.runtime_port = c_atoi(v),
		b"valve_udp" => o.valve_udp = v.to_vec(),
		b"cmd_port" => o.cmd_port = c_atoi(v),
		b"seen" => o.seen = v.to_vec(),
		b"fit_window_s" => o.fit_window_s = c_atof(v),
		b"level" => o.level = parse_bool(v),
		b"valve_record" => o.valve_record = parse_bool(v),
		b"fallback" => o.fallback = parse_bool(v),
		b"fallback_after_ms" => o.fallback_after_ms = c_atof(v),
		b"blend_ms" => o.blend_ms = c_atof(v),
		b"valve_settle_s" => o.valve_settle_s = c_atof(v),
		b"valve_agree_m" => o.valve_agree_m = c_atof(v),
		b"fallback_force_ms" => o.fallback_force_ms = c_atof(v),
		b"keep_sessions" => o.keep_sessions = c_atoi(v),
		b"pose_log" => o.pose_log = v.to_vec(),
		b"panel_exe" => o.panel_exe = v.to_vec(),
		b"runtime_exe" => o.runtime_exe = v.to_vec(),
		b"runtime_cwd" => o.runtime_cwd = v.to_vec(),
		b"runtime_log" => o.runtime_log = v.to_vec(),
		b"runtime_restart" => o.runtime_restart = parse_bool(v),
		b"runtime_stop" => o.runtime_stop = parse_bool(v),
		_ => log("sally: unknown option driver.%s\n", &[BArg::S(k)]),
	}
}

/// ReadSettings(dir): <dir>/sally_settings.txt line by line (fgets, 4096 bytes at a time), each trimmed: blank lines
/// and '#' comments skipped, "driver." lines parsed, the rest the runtime's arguments ({time} filled in at every
/// start, RuntimeProcess::start)
pub fn read_settings(o: &mut Options, dir: &[u8]) {
	let mut path = dir.to_vec();
	path.push(K_SEP);
	path.extend_from_slice(b"sally_settings.txt");
	let Some(mut f) = CFile::open(&path, c"rb") else {
		log("sally: no %s, using defaults\n", &[BArg::S(&path)]);
		return;
	};
	let mut line = [0u8; 4096];
	while let Some(l) = f.gets(&mut line) {
		let s = trim(l);
		if s.is_empty() || s[0] == b'#' {
			continue;
		}
		if s.starts_with(b"driver.") {
			parse_option(o, s);
			continue;
		}
		o.runtime_args.push(s.to_vec());
	}
	f.close();
	log("sally: read %s (%d runtime arguments)\n", &[BArg::S(&path), BArg::I(o.runtime_args.len() as i32 as i64)]);
}
