//! The recording, live next to SteamVR: the tap on the dongles (sally-hid's live reader, read-only: shared handles
//! opened for reading, config fetch off, so nothing is ever sent to a device) into the session's tap recording and an
//! analyzer for the live view, and SteamVR's poses polled 250 times a second into steamvr_poses.csv on the tap's
//! clock. The device configs come from SteamVR's own copies, by the dongle SteamVR says each device is on.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use sally_hid::live::{Context, Live};
use sally_hid::tap::{Mode, Options, Runtime, Tap};

use crate::analysis::Analyzer;
use crate::session::{self, SteamDevice};
use crate::steamvr::{Client, Paths, MAX_DEVICES};

/// SteamVR's devices are listed again this often
const LIST_EVERY: Duration = Duration::from_secs(2);
/// SteamVR's poses are read this often
const POLL_EVERY: Duration = Duration::from_millis(4);

/// What the live tap's threads lock
pub struct Shared {
	tap: Tap,
	pub analyzer: Analyzer,
}

impl Context for Shared {
	fn tap(&mut self) -> (&mut Tap, &mut dyn Runtime) {
		(&mut self.tap, &mut self.analyzer)
	}
}

/// What the pose thread learns about SteamVR's devices
#[derive(Default)]
pub struct Known {
	/// every device SteamVR listed during the session, by serial
	pub devices: Vec<SteamDevice>,
	/// SteamVR's config of the device on each dongle
	pub configs: HashMap<String, Vec<u8>>,
	/// SteamVR's latest pose result per serial (200 = tracking)
	pub results: HashMap<String, i32>,
}

/// A recording running
pub struct Recording {
	pub dir: PathBuf,
	pub shared: Arc<Mutex<Shared>>,
	pub known: Arc<Mutex<Known>>,
	pub started: Instant,
	live: Live<Shared>,
	stop: Arc<AtomicBool>,
	poller: Option<JoinHandle<()>>,
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
	m.lock().unwrap_or_else(|e| e.into_inner())
}

impl Recording {
	/// Starts both readers into `dir`, with SteamVR's devices as they are now
	pub fn start(dir: &Path, client: Client, paths: Paths) -> Result<Recording, String> {
		std::fs::create_dir_all(dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
		let known = Arc::new(Mutex::new(Known::default()));
		let _ = refresh(&client, &paths, &known);

		let configs = {
			let known = known.clone();
			Box::new(move |dongle: &str| lock(&known).configs.get(dongle).cloned())
		};
		let options = Options {
			config_fetch: false,
			allow_fs_read: true,
			lightcap_mode: -2,
			playback_factor: 0.0,
			..Options::default()
		};
		let shared =
			Arc::new(Mutex::new(Shared { tap: Tap::new(options, Mode::Live), analyzer: Analyzer::new(configs) }));
		let tap_path = dir.join(session::TAP);
		let live = {
			let mut g = lock(&shared);
			Live::start(&shared, &mut g, Some(&tap_path.to_string_lossy()))
		}
		.ok_or("cannot read the dongles")?;

		let stop = Arc::new(AtomicBool::new(false));
		let poses = File::create(dir.join(session::POSES)).map_err(|e| format!("cannot write the poses: {e}"))?;
		let poller = {
			let (stop, known) = (stop.clone(), known.clone());
			std::thread::Builder::new()
				.name("steamvr_poses".into())
				.spawn(move || poll(client, paths, known, poses, stop))
				.map_err(|e| e.to_string())?
		};
		Ok(Recording {
			dir: dir.to_path_buf(),
			shared,
			known,
			started: Instant::now(),
			live,
			stop,
			poller: Some(poller),
		})
	}

	/// Stops both readers; the files are complete after
	pub fn stop(mut self) -> (PathBuf, Arc<Mutex<Known>>) {
		self.stop.store(true, Ordering::SeqCst);
		if let Some(p) = self.poller.take() {
			let _ = p.join();
		}
		self.live.stop();
		(self.dir, self.known)
	}
}

/// SteamVR's devices listed (by index, as `Client::devices` gives them), and the config of each new one on a dongle
/// read from SteamVR's copy
fn refresh(client: &Client, paths: &Paths, known: &Mutex<Known>) -> Vec<Option<SteamDevice>> {
	let settings = paths.settings();
	let mut listed = client.devices(paths, &settings);
	// a lighthouse device without the dongle property: its dongle from SteamVR's log ("LHR-...: Connected to receiver
	// <dongle>"), read only when one is missing
	if listed.iter().flatten().any(|d| d.dongle.is_empty() && d.system == "lighthouse" && d.class != "station") {
		let log = paths.server_log().and_then(|l| std::fs::read(l).ok()).unwrap_or_default();
		let log = String::from_utf8_lossy(&log);
		for d in listed.iter_mut().flatten().filter(|d| d.dongle.is_empty() && d.class != "station") {
			let key = format!("{}: Connected to receiver ", d.serial);
			if let Some(i) = log.rfind(&key) {
				d.dongle = log[i + key.len()..].split_whitespace().next().unwrap_or("").to_string();
			}
		}
	}
	let mut k = lock(known);
	for d in listed.iter().flatten().cloned() {
		if !d.dongle.is_empty() && !k.configs.contains_key(&d.dongle) {
			if let Some(c) = paths.device_config(&d.serial) {
				k.configs.insert(d.dongle.clone(), c);
			}
		}
		match k.devices.iter_mut().find(|o| o.serial == d.serial) {
			Some(o) => {
				// a role or dongle can show up late; keep what is known
				if !d.role.is_empty() {
					o.role = d.role.clone();
				}
				if !d.dongle.is_empty() {
					o.dongle = d.dongle.clone();
				}
			}
			None => k.devices.push(d),
		}
	}
	listed
}

/// The pose thread: SteamVR's poses every 4 ms for every device but the base stations, and its device list every 2 s
fn poll(client: Client, paths: Paths, known: Arc<Mutex<Known>>, file: File, stop: Arc<AtomicBool>) {
	let mut out = BufWriter::with_capacity(1 << 20, file);
	let _ = writeln!(out, "{}", session::POSES_HEADER);
	let mut serials: Vec<Option<String>> = vec![None; MAX_DEVICES];
	let mut listed: Option<Instant> = None;
	while !stop.load(Ordering::SeqCst) {
		if listed.is_none_or(|l| l.elapsed() >= LIST_EVERY) {
			listed = Some(Instant::now());
			for (i, d) in refresh(&client, &paths, &known).into_iter().enumerate() {
				serials[i] = d.filter(|d| d.class != "station").map(|d| d.serial);
			}
		}
		// the tap stamps its reports with this clock
		let t_us = (sally_os::absolute_time() * 1e6) as u64;
		let poses = client.poses(t_us);
		let mut results = Vec::new();
		for (i, p) in poses.iter().enumerate() {
			if let Some(serial) = &serials[i] {
				let _ = out.write_all(p.csv(serial).as_bytes());
				results.push((serial.clone(), if p.valid { p.result } else { 0 }));
			}
		}
		{
			let mut k = lock(&known);
			for (s, r) in results {
				k.results.insert(s, r);
			}
		}
		std::thread::sleep(POLL_EVERY);
	}
	let _ = out.flush();
	drop(client);
}
