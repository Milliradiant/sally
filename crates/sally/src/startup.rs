//! What `sally run` and `sally replay` start alike: survive_init_internal (the options, the command line and the
//! environment, the log, the config file, the stations), and of survive_startup the recording, the drivers before the
//! tap, the options it applies to the stations it read, and the broadcast of the stations it knows.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::{Mutex, Weak};

use sally_runtime::config::{
	self, Config, PLAYBACK_CONFIG_PREFIX_FIELDS, RECORD_CONFIG_PREFIX_FIELDS, SC_GET, SC_SETCONFIG,
};
use sally_runtime::context::Context;
use sally_runtime::diagnosis::Diagnosis;
use sally_runtime::diagrec::DiagRec;
use sally_runtime::log::Log;
use sally_runtime::map::MapSolveDriver;
use sally_runtime::protocol::{Protocol, Reg};
use sally_runtime::recording::{Recording, RecordingOptions};
use sally_runtime::{options, Runtime};

/// SurviveDeviceDriverReturn (survive_types.h): a driver started and is a device source
pub const DRIVER_NORMAL: i32 = 0;
/// ... it could not start
pub const DRIVER_ERROR: i32 = -1;
/// ... it started and is not a source
pub const DRIVER_PASSIVE: i32 = 1;

/// A C string as text, for the lines that name it
pub fn text(s: &[u8]) -> String {
	String::from_utf8_lossy(s).into_owned()
}

/// A path the C would hand fopen
pub fn path(p: &[u8]) -> std::path::PathBuf {
	#[cfg(unix)]
	{
		use std::os::unix::ffi::OsStrExt;
		std::path::PathBuf::from(std::ffi::OsStr::from_bytes(p))
	}
	#[cfg(not(unix))]
	{
		std::path::PathBuf::from(text(p))
	}
}

/// survive_init_internal: the options registered, the command line and the environment, the log, the config file,
/// the stations. None when the run stops here (the help, the version, the completion list), with its exit code
pub fn init(argv: &[Vec<u8>]) -> Result<Runtime, i32> {
	let mut config = Config::default();
	options::register(&mut config);
	let args = config.apply_args(argv);
	config.apply_env(config::environment());

	let log_file = config.configs("log", SC_GET, None);
	let record_to_stdout = config.configi("record-stdout", SC_GET, 0) != 0;
	let target: Box<dyn Write + Send> = match &log_file {
		Some(p) => match File::create(path(p)) {
			Ok(f) => Box::new(BufWriter::new(f)),
			Err(e) => {
				// the C's log target would be NULL, and its first line a crash
				eprintln!("sally: cannot open the log {}: {e}", text(p));
				return Err(1);
			}
		},
		None => Box::new(std::io::stdout()),
	};
	let mut log = Log::new(Some(target), log_file.is_none() && !record_to_stdout);

	let user_set_configfile = config.is_set("configfile");
	if !user_set_configfile {
		config.find_correct_config_file(&PLAYBACK_CONFIG_PREFIX_FIELDS);
	}
	config.configi("v", SC_SETCONFIG, 0);
	let init_config = config.configs("init-configfile", SC_GET, None);
	if let Some(p) = &init_config {
		log.info(&format!("Initial config file is {}", text(p)));
	}
	config.read(init_config.as_deref());
	if !user_set_configfile && config.find_correct_config_file(&RECORD_CONFIG_PREFIX_FIELDS) {
		// survive_config_file_path (it makes the file's directory), then config_save: no station is read yet
		config.file_path();
		let outcome = config.save(&[false; config::NUM_GEN2_LIGHTHOUSES]);
		if let (config::SaveOutcome::OpenFailed { .. }, Some(msg)) = (&outcome, outcome.message()) {
			log.warn(&text(&msg));
		}
	}

	let mut rt = Runtime::new(config, log);
	rt.load_lighthouses();

	if let Some(matches) = &args.autocomplete {
		// -m: the parameters for shell completion, or the help of the one named
		let last = matches[if matches[2].is_none() { 1 } else { 2 }].clone();
		let matching = if matches[2].is_none() { None } else { matches[1].clone() };
		let check = matching.or(last.clone());
		if let Some(c) = check.filter(|c| c.len() > 2) {
			if let Some(help) = rt.config.help_for_parameter(&c[2..]) {
				let _ = std::io::stderr().write_all(&help);
				return Err(0);
			}
		}
		let mut out = b"-h -m -p -l -c ".to_vec();
		out.extend_from_slice(&rt.config.known_configs_text(false));
		out.push(b'\n');
		let _ = std::io::stdout().write_all(&out);
		return Err(0);
	}
	if args.showhelp {
		let mut e = std::io::stderr();
		let _ = write!(
			e,
			"sally version {}\nAvailable flags:\n -h                      - shows help.\n -m                      - list \
			 parameters, for autocomplete.\n -p [poser]              - use a specific poser.\n -l [lighthouse count]   - \
			 use a specific number of lighthouses.\n -c [config file]        - set config file\n\n",
			env!("CARGO_PKG_VERSION")
		);
		let _ = std::io::stdout().write_all(&rt.config.known_configs_text(true));
		return Err(0);
	}
	if args.showversion {
		eprintln!("sally version {}", env!("CARGO_PKG_VERSION"));
		return Err(0);
	}
	Ok(rt)
}

/// survive_install_recording: the recording opened (the file, stdout or both), and every option written into it
pub fn install_recording(rt: &mut Runtime) {
	let record = rt.config.configs("record", SC_GET, Some(b"")).unwrap_or_default();
	let record_stdout = rt.config.configi("record-stdout", SC_GET, 0) != 0;
	if !record.is_empty() || record_stdout {
		let file = (!record.is_empty()).then(|| text(&record));
		// SurviveRecordingData_attach_config, before the file is opened: when it does not open, the C frees the
		// recording with its variables still bound, so they stay counted (and nothing reads them)
		let options = RecordingOptions::default();
		options.attach(&mut rt.config);
		match Recording::open(file.as_deref(), record_stdout, options) {
			Ok(r) => {
				rt.recording = Some(r);
				if let Some(p) = &file {
					rt.info(&format!("Recording to '{p}' Compression: {}", p.ends_with(".gz") as i32));
				}
				if record_stdout {
					rt.info("Recording to stdout");
				}
			}
			Err(_) => {
				if let Some(p) = &file {
					rt.info(&format!("Could not open {p} for writing"));
				}
			}
		}
	}
	// survive_config_iterate(ctx, survive_record_config, ..): "OPTION %s %c %s\n" for each registered option
	if rt.recording.is_some() {
		let lines: Vec<(Vec<u8>, u8, Vec<u8>)> = rt
			.config
			.options()
			.iter()
			.map(|o| (o.name().to_vec(), o.type_char(), rt.config.as_str(o.name(), Some(b""), 128)))
			.collect();
		for (name, type_char, value) in lines {
			let t = rt.run_time();
			if let Some(r) = rt.recording.as_mut() {
				r.option(t, &name, type_char, &value);
			}
		}
	}
}

/// Whether survive_startup's driver loop starts the driver of this option: its value read as a number or as text is
/// set
pub fn enabled(rt: &mut Runtime, option: &str) -> bool {
	rt.config.configi(option, SC_GET, 0) != 0 || rt.config.configs(option, SC_GET, None).is_some()
}

/// callDriver for a driver before the tap: the driver started (with its log lines) and loaded, its return
/// (SurviveDeviceDriverReturn); None for one that is not ported (and the tap's, which each program starts its own way).
/// `protocol_started` is the runtime protocol's own note that its first registration has set it up. `threads`: the
/// context a driver's own thread takes the lock of (`sally run`'s), None on one thread (`sally replay`'s), where the
/// map solve solves inline
pub fn start_driver(
	rt: &mut Runtime, name: &'static str, protocol_started: &mut bool, threads: Option<&Weak<Mutex<Context>>>,
) -> Option<i32> {
	match name {
		"Diagnosis" => {
			// the diagnosis checks "diagnosis" again itself: at 0 it adds no driver, but it is named all the same
			if let Some(d) = Diagnosis::reg(rt) {
				rt.add_driver(name, Box::new(d));
			}
			Some(DRIVER_PASSIVE)
		}
		"DiagRec" => {
			rt.add_driver(name, Box::new(DiagRec));
			Some(DRIVER_PASSIVE)
		}
		"Map_Solve" => {
			let mut ms = MapSolveDriver::start(rt);
			if let Some(ctx) = threads {
				ms.start_thread(ctx.clone());
			}
			rt.add_driver(name, Box::new(ms));
			Some(DRIVER_PASSIVE)
		}
		"Pose_Udp" | "Runtime_Port" | "Runtime_Driver_File" => Some(match Protocol::reg(rt, *protocol_started) {
			Reg::Started(p) => {
				rt.add_driver(name, p);
				*protocol_started = true;
				DRIVER_PASSIVE
			}
			Reg::Passive => DRIVER_PASSIVE,
			Reg::Error(r) => r,
		}),
		_ => None,
	}
}

/// survive_startup's options on the stations it read: force-calibrate forgets their poses (and the floor offset),
/// force-ootx their calibration, use-ootx 0 the calibration's use. random-bsd-noise is not ported (it adds glibc
/// rand() noise to the poses: a test of the old solver's)
pub fn startup_options(rt: &mut Runtime) {
	if rt.config.configi("force-calibrate", SC_GET, 0) != 0 {
		rt.info("Force calibrate flag set -- clearing position on all lighthouses");
		rt.floor_offset = 0.0;
		for i in 0..rt.active_lighthouses.max(0) as usize {
			rt.bsd[i].position_set = false;
			rt.bsd[i].pose = Default::default();
		}
	}
	if rt.config.configf("random-bsd-noise", SC_GET, -1.0) > 0.0 {
		rt.warn("random-bsd-noise is not supported; the stations are left as they are");
	}
	if rt.config.configi("force-ootx", SC_GET, 0) != 0 {
		rt.info("Force ootx flag set -- clearing ootx on all lighthouses");
		for i in 0..rt.active_lighthouses.max(0) as usize {
			rt.bsd[i].ootx_set = false;
			rt.bsd[i].fcal = Default::default();
		}
	}
	if rt.config.configi("use-ootx", SC_GET, 1) == 0 {
		for i in 0..rt.active_lighthouses.max(0) as usize {
			rt.bsd[i].fcal = Default::default();
		}
	}
}

/// survive_startup's "If lighthouse positions are known, broadcast them": each known station's OOTX and pose through
/// their processes
pub fn broadcast_stations(rt: &mut Runtime) {
	for i in 0..rt.active_lighthouses.max(0) as usize {
		if rt.bsd[i].ootx_set {
			rt.ootx_received(i);
		}
		if rt.bsd[i].position_set {
			let pose = rt.bsd[i].pose;
			rt.raw_lighthouse_pose(i, Some(&pose));
		}
	}
}
