//! The live tap on its own, to hold it against the C on the dongles: `live_tap <seconds> <recording> [--<option>
//! <value>]...` runs the reader for that long and writes the live recording to `<recording>`, the lines the C's
//! `--hidtap 1 --hidtap-record` writes for the same reports. Next to the C runtime (both read every report, shared),
//! the two recordings hold the same reports, each stamped by its own reader.
//!
//! The runtime is a minimal one: the devices are the tap's (named from sally-hid's device table), a device is
//! configured when its `<serial>_config.json` is in the working directory (or, with config fetch on, by any config
//! that streams in and inflates), and the reports its parser would get are counted, not parsed. The tap's info and
//! warning lines are printed as the C logs them. At the end, each device's reports: those recorded (from the
//! recording) and those that went to its parser.
//!
//! The options are the tap's, with the C's defaults but config fetch off: `--hidtap-config-fetch 0|1`,
//! `--hidtap-config-delay <s>`, `--hidtap-lightcap-mode <-2..2>`, `--hidtap-allow-fs-config 0|1`, `--hidtap-vid <hex>`.

use std::collections::BTreeMap;
use std::process::exit;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sally_hid::line::{self, Line};
use sally_hid::live::{Context, Live};
use sally_hid::replay::PlaybackTime;
use sally_hid::report::{Globals, Parser, BUFFER_LEN};
use sally_hid::tap::{Mode, ObjectId, Options, Runtime, Tap};

/// A device as the runtime sees it
struct Object {
	name: String,
	serial: String,
	reports: u64,
}

/// The minimal runtime
struct Sink {
	start: Instant,
	objects: Vec<Object>,
}

impl Runtime for Sink {
	fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId> {
		self.objects.push(Object { name: name.to_string(), serial: serial.to_string(), reports: 0 });
		Some(self.objects.len() - 1)
	}
	fn load_config_file(&mut self, _obj: ObjectId, filename: &str) -> bool {
		std::fs::File::open(filename).is_ok()
	}
	fn config(&mut self, _obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
		if json.is_some_and(|j| !j.is_empty()) {
			0
		} else {
			-1
		}
	}
	fn report(
		&mut self, obj: ObjectId, _parser: &mut Parser, _globals: &mut Globals, _time_us: u64, _iface: i32,
		_buffer: &[u8; BUFFER_LEN], _len: usize,
	) {
		self.objects[obj].reports += 1;
	}
	fn setup_write(
		&mut self, _obj: ObjectId, _parser: &mut Parser, _bm_request_type: u8, _b_request: u8, _w_value: u16,
		_data: &[u8],
	) {
	}
	fn feature_read(&mut self, _obj: ObjectId, _parser: &Parser, _data: &[u8]) {}
	fn install_run_time_fn(&mut self, _time: Option<Arc<PlaybackTime>>) {}
	fn run_time(&self) -> f64 {
		self.start.elapsed().as_secs_f64()
	}
	fn info(&mut self, text: &str) {
		println!("Info: {text}");
	}
	fn warn(&mut self, text: &str) {
		println!("Warning: {text}");
	}
	fn error(&mut self, _code: i32, text: &str) {
		// SV_ERROR logs at the info level
		println!("Info: {text}");
	}
}

/// What the live tap's threads lock
struct Probe {
	tap: Tap,
	sink: Sink,
}

impl Context for Probe {
	fn tap(&mut self) -> (&mut Tap, &mut dyn Runtime) {
		(&mut self.tap, &mut self.sink)
	}
}

fn usage() -> ! {
	eprintln!("usage: live_tap <seconds> <recording> [--hidtap-config-fetch 0|1] [--hidtap-config-delay <s>]");
	eprintln!("       [--hidtap-lightcap-mode <-2..2>] [--hidtap-allow-fs-config 0|1] [--hidtap-vid <hex>]");
	exit(2);
}

fn main() {
	let args: Vec<String> = std::env::args().skip(1).collect();
	if args.len() < 2 || !args.len().is_multiple_of(2) {
		usage();
	}
	let Ok(seconds) = args[0].parse::<f64>() else { usage() };
	let recording = args[1].clone();
	let mut options = Options { config_fetch: false, ..Options::default() };
	for pair in args[2..].chunks(2) {
		let (name, value) = (pair[0].as_str(), pair[1].as_str());
		let flag = || value != "0";
		match name {
			"--hidtap-config-fetch" => options.config_fetch = flag(),
			"--hidtap-config-delay" => options.config_delay = value.parse().unwrap_or_else(|_| usage()),
			"--hidtap-lightcap-mode" => options.lightcap_mode = value.parse().unwrap_or_else(|_| usage()),
			"--hidtap-allow-fs-config" => options.allow_fs_read = flag(),
			"--hidtap-vid" => {
				options.vid = u16::from_str_radix(value.trim_start_matches("0x"), 16).unwrap_or_else(|_| usage())
			}
			_ => usage(),
		}
	}

	let probe = Probe { tap: Tap::new(options, Mode::Live), sink: Sink { start: Instant::now(), objects: Vec::new() } };
	let ctx = Arc::new(Mutex::new(probe));
	let live = {
		let mut g = ctx.lock().unwrap_or_else(|e| e.into_inner());
		Live::start(&ctx, &mut g, Some(&recording))
	};
	let Some(live) = live else { exit(1) };
	std::thread::sleep(Duration::from_secs_f64(seconds.max(0.0)));
	live.stop();

	// the reports recorded, by serial, as a replay would read them
	let mut recorded: BTreeMap<String, u64> = BTreeMap::new();
	for l in std::fs::read(&recording).unwrap_or_default().split_inclusive(|&c| c == b'\n') {
		if let Line::Report { serial, .. } = line::parse(l) {
			*recorded.entry(serial).or_default() += 1;
		}
	}
	let g = ctx.lock().unwrap_or_else(|e| e.into_inner());
	for (serial, n) in &recorded {
		let o = g.sink.objects.iter().find(|o| &o.serial == serial);
		let name = o.map_or("-", |o| o.name.as_str());
		let parsed = o.map_or(0, |o| o.reports);
		println!("live_tap: {name} {serial}: {n} reports recorded, {parsed} to its parser");
	}
	for o in g.sink.objects.iter().filter(|o| !recorded.contains_key(&o.serial)) {
		println!("live_tap: {} {}: no reports recorded, {} to its parser", o.name, o.serial, o.reports);
	}
}
