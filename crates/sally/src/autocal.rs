//! The room's calibration kept up by itself, in the live runtime (`sally run` on the dongles, with a config file and
//! --hidtap-record; docs/sally_design.md 4.2). Nothing to set and nothing to start:
//!
//! - each session's recording is calibrated when the session ends: `sally calibrate` (the same program, its first
//!   argument `calibrate`), at idle priority, on its own, so it goes on after the runtime has stopped. A session that
//!   ended without its close (a crash, a kill) is calibrated when the next one starts;
//! - a session's calibration waits (pending) for the next sessions' word: each later session's calibration is given
//!   it, and it and the room's calibration each predict that session's light, which neither was solved from
//!   (`sally calibrate --calibrate-candidate`). When it predicts two sessions in a row better (`calibrate::judge`,
//!   `WINS`), it is ready, and the next start takes it up; the first session it does not, it is dropped. One
//!   session's calibration fits that session's own light better than another's does, whatever the room, and one
//!   session is a poor judge of another's (findings.md), so a session never judges its own and never decides alone;
//! - a room the config does not place (a new one, or a station added or replaced) is calibrated during its first
//!   session, from what has been recorded so far, after 2 minutes, then 4, 8 and so on until one is made; the runtime
//!   then stops, and the sally driver starts it again on the new calibration (until then SteamVR shows Valve's
//!   tracking). A calibration that places stations the room's does not is ready at once too;
//! - the runtime protocol's `calibrate`: one of the session so far, judged on the last quarter of its own light (the
//!   one check a single session gives), `calibrate status`, and `calibrate apply` (the runtime stops for the sally
//!   driver to start it again, which takes the calibration up).
//!
//! Taking one up, before the config is read: the config, the stations' corrections and the live map (in the old
//! calibration's frame) put aside as `<file>.bak_<time>`, the new config and corrections in their places, and the
//! calibration's files kept in `calibration\adopted_<time>`. A ready one made from a config that is no longer the
//! room's (another taken up since, or one written by hand) is dropped. It all lives in `calibration\` beside the
//! config: `log.txt` (a line a step), `work\` (the one running: its copies, `status.txt`, `calibrate.log`), `pending\`,
//! `ready\`, `last\` (the last one's files, whatever it found).

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};

use sally_runtime::runtime::Calibrator;
use sally_runtime::Runtime;

/// The live options a calibration does not take: the live runtime's own files and ports (the calibration replays
/// with its own copies), and what replaying sets
const LIVE_ONLY: &[&str] = &[
	"--hidtap",
	"--hidtap-config-fetch",
	"--hidtap-record",
	"--hidtap-playback",
	"--pose-udp",
	"--runtime-port",
	"--serialize-device-config",
	"--record",
	"--record-stdout",
	"--log",
	"--configfile",
	"--init-configfile",
	"--station-fcal-file",
	"--kalman-acc-cal-file",
	"--map-solve-file",
	"--map-solve-test-load",
	"--device-learned-file",
	"--playback-factor",
	"--stop-after",
];

/// A new room's first calibration, after this much of its first session (s), then after twice as long, and so on
const NEW_ROOM_FIRST: f64 = 120.0;

/// A session's calibration takes the room's place when it predicts this many sessions after it better than the room's,
/// one after the other: one session is a poor judge. With 1, a session whose light no other session's calibration
/// predicts well takes up a calibration that the sessions after it then do worse on (findings.md section 2)
const WINS: u32 = 2;

/// The most calibrations waiting at once (the oldest goes): `WINS` and the one just made
const MAX_PENDING: usize = WINS as usize + 1;

/// What a calibration is of
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
	/// a whole session's recording, its end: it judges the pending calibration and becomes the next
	Session,
	/// the session so far, asked for (`calibrate`): judged on its own last quarter
	SoFar,
	/// the session so far in a room the config does not place: ready once made
	NewRoom,
}

impl Kind {
	fn word(self) -> &'static str {
		match self {
			Kind::Session => "session",
			Kind::SoFar => "so-far",
			Kind::NewRoom => "new-room",
		}
	}

	fn from_word(w: &str) -> Kind {
		match w {
			"so-far" => Kind::SoFar,
			"new-room" => Kind::NewRoom,
			_ => Kind::Session,
		}
	}
}

/// The live runtime's calibration: its files, the one running, the new room's schedule
pub struct Auto {
	/// calibration\ beside the config
	dir: PathBuf,
	config: PathBuf,
	fcal: PathBuf,
	map: Option<PathBuf>,
	/// this session's recording
	record: PathBuf,
	/// the program (`sally calibrate` is it with `calibrate` first) and the folder it runs in (the device configs')
	exe: PathBuf,
	cwd: PathBuf,
	/// the live options the calibration replays with
	pass: Vec<Vec<u8>>,
	/// the calibration this runtime started
	child: Option<Child>,
	/// the stations the config placed when the runtime started
	known: Vec<u32>,
	/// stations the config does not place: when they were first seen (run time), the next calibration of the session
	/// so far, how many there were
	unknown_since: Option<f64>,
	next_try: f64,
	/// LH1 base stations were seen this run (the runtime tracks with LH2 alone): said in the status, for the panel
	lh1: bool,
	tries: u32,
	/// a ready calibration of this session's new room: the runtime is to stop for it
	stop: bool,
	/// what to say once the log is open (warning, text)
	notes: Vec<(bool, String)>,
	last_tick: f64,
}

/// The protocol's way in (`Runtime::calibrator`): the main loop holds the same `Auto`
pub struct Handle(pub Arc<Mutex<Auto>>);

impl Calibrator for Handle {
	fn command(&mut self, rt: &mut Runtime, args: &[&[u8]]) -> (Vec<u8>, bool) {
		let mut a = self.0.lock().unwrap_or_else(|e| e.into_inner());
		let r = a.command(rt, args);
		a.flush(rt);
		r
	}
}

/// The value of an option on the command line (the last one: later ones override earlier ones)
fn value_of(argv: &[Vec<u8>], name: &str) -> Option<String> {
	let mut v = None;
	for w in argv.windows(2) {
		if w[0] == name.as_bytes() {
			v = Some(String::from_utf8_lossy(&w[1]).into_owned());
		}
	}
	v
}

/// The command line without the options `drop` names (an option's value is the word after it, unless that is an
/// option)
fn without(argv: &[Vec<u8>], drop: &[&str]) -> Vec<Vec<u8>> {
	let mut out = Vec::new();
	let mut i = 0;
	while i < argv.len() {
		let a = &argv[i];
		let n = if i + 1 < argv.len() && !argv[i + 1].starts_with(b"--") { 2 } else { 1 };
		if !(a.starts_with(b"--") && drop.iter().any(|d| d.as_bytes() == a.as_slice())) {
			out.extend(argv[i..i + n].iter().cloned());
		}
		i += n;
	}
	out
}

/// The stations a config places, as a line: `<id>:<x>,<y>,<z>,<qw>,<qx>,<qy>,<qz>` each, by id ("none" for none).
/// What a calibration was made from, compared with `same_stations`: the runtime writes the config again as it
/// starts and closes (unless the stations are locked), in its own number format
fn stations_of(config: &Path) -> String {
	let mut c = sally_runtime::config::Config::default();
	c.read(Some(config.to_string_lossy().as_bytes()));
	let mut st = Vec::new();
	for i in 0..sally_runtime::config::NUM_GEN2_LIGHTHOUSES {
		let mut r = sally_runtime::config::LighthouseRecord::default();
		if c.read_lighthouse(i as u8, &mut r) && r.position_set && r.base_station_id != 0 {
			let (p, q) = (r.pose.pos, r.pose.rot);
			st.push((
				r.base_station_id,
				format!("{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}", p[0], p[1], p[2], q[0], q[1], q[2], q[3]),
			));
		}
	}
	st.sort();
	if st.is_empty() {
		return "none".into();
	}
	st.iter().map(|(id, v)| format!("{id}:{v}")).collect::<Vec<_>>().join(" ")
}

/// Two `stations_of` lines name the same stations at the same places (within 0.1 mm and 1e-4 of a quaternion's
/// terms, either sign)
fn same_stations(a: &str, b: &str) -> bool {
	let parse = |t: &str| -> Vec<(String, Vec<f64>)> {
		t.split(' ')
			.filter_map(|w| {
				let (id, v) = w.split_once(':')?;
				Some((id.to_string(), v.split(',').filter_map(|x| x.parse().ok()).collect()))
			})
			.collect()
	};
	if a == b {
		return true;
	}
	let (x, y) = (parse(a), parse(b));
	x.len() == y.len()
		&& x.iter().zip(&y).all(|((ia, va), (ib, vb))| {
			if ia != ib || va.len() != 7 || vb.len() != 7 {
				return false;
			}
			let pos = (0..3).all(|k| (va[k] - vb[k]).abs() < 1e-4);
			let same = (3..7).all(|k| (va[k] - vb[k]).abs() < 1e-4);
			let flipped = (3..7).all(|k| (va[k] + vb[k]).abs() < 1e-4);
			pos && (same || flipped)
		})
}

/// FNV-1a of a file's bytes ("none" when it cannot be read): whether the room's corrections are still the ones a
/// calibration was made from
fn fingerprint(path: &Path) -> String {
	match std::fs::read(path) {
		Ok(b) => {
			let mut h: u64 = 0xcbf2_9ce4_8422_2325;
			for c in b {
				h ^= c as u64;
				h = h.wrapping_mul(0x0000_0100_0000_01b3);
			}
			format!("{h:016x}")
		}
		Err(_) => "none".into(),
	}
}

fn stamp() -> String {
	sally_os::localtime::local_now().map(|t| t.stamp()).unwrap_or_else(|| "now".into())
}

/// A key-value file's value for `key` (`key value...` lines)
fn field(text: &str, key: &str) -> Option<String> {
	text.lines().find_map(|l| {
		let (k, v) = l.split_once(' ').unwrap_or((l, ""));
		(k == key).then(|| v.trim().to_string())
	})
}

fn read(path: &Path) -> String {
	std::fs::read_to_string(path).unwrap_or_default()
}

/// A folder replaced by another: `to` removed, `from` renamed to it
fn replace_dir(from: &Path, to: &Path) -> std::io::Result<()> {
	if to.exists() {
		std::fs::remove_dir_all(to)?;
	}
	std::fs::rename(from, to)
}

/// A file copied into place through a temporary beside it, the old one put aside as `<file>.bak_<stamp>` first
fn put_in_place(from: &Path, to: &Path, stamp: &str) -> std::io::Result<Option<PathBuf>> {
	let tmp = to.with_extension("tmp_adopt");
	std::fs::copy(from, &tmp)?;
	let aside = to.is_file().then(|| bak(to, stamp));
	if let Some(a) = &aside {
		std::fs::rename(to, a)?;
	}
	std::fs::rename(&tmp, to)?;
	Ok(aside)
}

fn bak(p: &Path, stamp: &str) -> PathBuf {
	let mut s = p.as_os_str().to_os_string();
	s.push(format!(".bak_{stamp}"));
	PathBuf::from(s)
}

/// The name a recording goes by in log.txt
fn name_of(p: &Path) -> String {
	p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
}

impl Auto {
	/// For a live runtime with a config file and a recording: `--hidtap 1`, `--configfile`, `--hidtap-record`. None
	/// otherwise (a replay, the bench, a runtime started by hand without them)
	pub fn new(argv: &[Vec<u8>]) -> Option<Auto> {
		if value_of(argv, "--hidtap").as_deref() != Some("1") {
			return None;
		}
		let cwd = std::env::current_dir().ok()?;
		let config = cwd.join(value_of(argv, "--configfile")?);
		let record = cwd.join(value_of(argv, "--hidtap-record").filter(|r| !r.is_empty())?);
		let dir = config.parent()?.join("calibration");
		let fcal = value_of(argv, "--station-fcal-file")
			.filter(|f| !f.is_empty())
			.map(|f| cwd.join(f))
			.unwrap_or_else(|| config.with_file_name("station_fcal.txt"));
		let map = value_of(argv, "--map-solve-file").filter(|m| !m.is_empty()).map(|m| cwd.join(m));
		let exe = std::env::current_exe().ok()?;
		let pass = without(argv.get(1..).unwrap_or(&[]), LIVE_ONLY);
		Some(Auto {
			dir,
			config,
			fcal,
			map,
			record,
			exe,
			cwd,
			pass,
			child: None,
			known: Vec::new(),
			unknown_since: None,
			next_try: 0.0,
			lh1: false,
			tries: 0,
			stop: false,
			notes: Vec::new(),
			last_tick: -1e9,
		})
	}

	fn work(&self) -> PathBuf {
		self.dir.join("work")
	}
	fn pending(&self) -> PathBuf {
		self.dir.join("pending")
	}

	/// The calibrations waiting for sessions' word (pending\<its recording's name>), oldest first
	fn pendings(&self) -> Vec<PathBuf> {
		let mut v: Vec<PathBuf> = std::fs::read_dir(self.pending())
			.map(|d| d.flatten().map(|e| e.path()).filter(|p| p.join("driver_config.json").is_file()).collect())
			.unwrap_or_default();
		v.sort();
		v
	}
	fn ready(&self) -> PathBuf {
		self.dir.join("ready")
	}

	fn note(&mut self, warn: bool, text: String) {
		self.notes.push((warn, text));
	}

	/// What was noted, into the runtime's log
	pub fn flush(&mut self, rt: &mut Runtime) {
		for (warn, t) in std::mem::take(&mut self.notes) {
			if warn {
				rt.warn(&format!("calibration: {t}"));
			} else {
				rt.info(&format!("calibration: {t}"));
			}
		}
	}

	/// A line into calibration\log.txt
	fn log(&self, what: &str, rec: &str, text: &str) {
		use std::io::Write;
		let _ = std::fs::create_dir_all(&self.dir);
		if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(self.dir.join("log.txt")) {
			let _ = writeln!(f, "{} {what} {rec} {text}", stamp());
		}
	}

	/// Whether log.txt has a line `what` for the recording (any step when `what` is None)
	fn logged(&self, rec: &str, what: Option<&str>) -> bool {
		read(&self.dir.join("log.txt")).lines().any(|l| {
			let w: Vec<&str> = l.splitn(4, ' ').collect();
			w.len() >= 3 && w[2] == rec && what.is_none_or(|x| w[1] == x)
		})
	}

	/// Before the config is read: a calibration that finished while no runtime ran, then a ready one taken up
	pub fn adopt(&mut self) {
		self.finish_if_done();
		let ready = self.ready();
		if ready.join("driver_config.json").is_file() {
			self.take_up(&ready);
		}
	}

	/// The ready calibration into the room's files, unless the room's calibration is no longer the one it was made
	/// from
	fn take_up(&mut self, from: &Path) {
		let info = read(&from.join("info.txt"));
		let rec = field(&info, "recording").unwrap_or_default();
		let base_ok = same_stations(&field(&info, "config").unwrap_or_default(), &stations_of(&self.config))
			&& field(&info, "fcal").as_deref() == Some(&fingerprint(&self.fcal));
		let s = stamp();
		if !base_ok {
			let to = self.dir.join(format!("dropped_{s}"));
			let _ = replace_dir(from, &to);
			self.log("dropped", &rec, "the room's calibration is no longer the one it was made from");
			self.note(
				true,
				format!("a ready calibration (of {rec}) was made from a config that is no longer the room's: dropped"),
			);
			return;
		}
		let mut aside = Vec::new();
		let r = (|| -> std::io::Result<()> {
			if let Some(a) = put_in_place(&from.join("driver_config.json"), &self.config, &s)? {
				aside.push(a);
			}
			let fc = from.join("station_fcal.txt");
			if fc.is_file() {
				if let Some(a) = put_in_place(&fc, &self.fcal, &s)? {
					aside.push(a);
				}
			}
			if let Some(m) = self.map.as_ref().filter(|m| m.is_file()) {
				let a = bak(m, &s);
				std::fs::rename(m, &a)?;
				aside.push(a);
			}
			Ok(())
		})();
		match r {
			Ok(()) => {
				let to = self.dir.join(format!("adopted_{s}"));
				let _ = replace_dir(from, &to);
				let why = field(&info, "why").unwrap_or_default();
				self.log("adopted", &rec, &why);
				let old: Vec<String> = aside.iter().map(|a| a.display().to_string()).collect();
				self.note(
					false,
					format!(
						"the room's new calibration (of {rec}: {why}) taken up; the old files are {}",
						old.join(", ")
					),
				);
			}
			Err(e) => {
				self.log("failed", &rec, &format!("taking it up: {e}"));
				self.note(true, format!("cannot take the ready calibration up: {e}"));
			}
		}
	}

	/// Once the runtime is made: the stations the config places
	pub fn after_init(&mut self, rt: &mut Runtime) {
		let n = rt.active_lighthouses.max(0) as usize;
		self.known = rt
			.bsd
			.iter()
			.take(n)
			.filter(|b| b.position_set && b.base_station_id != 0)
			.map(|b| b.base_station_id)
			.collect();
		self.flush(rt);
	}

	/// Once the runtime runs: the last session calibrated if its close did not (its recording has no line in
	/// log.txt), unless one is running
	pub fn after_start(&mut self, rt: &mut Runtime) {
		if !self.running() {
			if let Some(prev) = self.previous_recording() {
				let n = name_of(&prev);
				if !self.logged(&n, None) {
					self.note(false, format!("{n} was not calibrated at its end: calibrating it now"));
					self.start(&prev, None, Kind::Session);
				}
			}
		}
		self.flush(rt);
	}

	/// The newest recording beside this session's and named like it (its name up to the first digit, its extension)
	fn previous_recording(&self) -> Option<PathBuf> {
		let dir = self.record.parent()?;
		let me = name_of(&self.record);
		let stem: String = me.chars().take_while(|c| !c.is_ascii_digit()).collect();
		let ext = self.record.extension().map(|e| e.to_string_lossy().into_owned());
		let mut best: Option<String> = None;
		for e in std::fs::read_dir(dir).ok()?.flatten() {
			let n = e.file_name().to_string_lossy().into_owned();
			let p = Path::new(&n);
			if n == me || !n.starts_with(&stem) || p.extension().map(|e| e.to_string_lossy().into_owned()) != ext {
				continue;
			}
			if !n[stem.len()..].starts_with(|c: char| c.is_ascii_digit()) {
				continue;
			}
			// the names carry their start's time: the latest sorts last
			if best.as_ref().is_none_or(|b| n > *b) && n < me {
				best = Some(n);
			}
		}
		best.map(|b| dir.join(b))
	}

	/// A calibration running: this runtime's, or one a runtime before it started that has not finished
	fn running(&mut self) -> bool {
		if let Some(c) = self.child.as_mut() {
			if matches!(c.try_wait(), Ok(None)) {
				return true;
			}
		}
		let work = self.work();
		if !work.join("job.txt").is_file() || self.child.is_some() {
			return false;
		}
		let pid = read(&work.join("pid.txt")).trim().parse::<u32>().unwrap_or(0);
		pid != 0 && sys::alive(pid) && !matches!(self.state().as_str(), "done" | "kept" | "failed")
	}

	/// The running calibration's state as it writes it (replaying, solving, done, kept, failed)
	fn state(&self) -> String {
		field(&read(&self.work().join("status.txt")), "state").unwrap_or_default()
	}

	/// A calibration of `rec` (its first `bytes` when it is being recorded) started, on copies of the room's files
	fn start(&mut self, rec: &Path, bytes: Option<u64>, kind: Kind) -> bool {
		let work = self.work();
		let r = (|| -> std::io::Result<Child> {
			if work.exists() {
				std::fs::remove_dir_all(&work)?;
			}
			std::fs::create_dir_all(&work)?;
			std::fs::copy(&self.config, work.join("driver_config.json"))?;
			if self.fcal.is_file() {
				std::fs::copy(&self.fcal, work.join("station_fcal.txt"))?;
			}
			let mut args: Vec<std::ffi::OsString> = vec!["calibrate".into(), "--hidtap-playback".into(), rec.into()];
			if let Some(b) = bytes {
				args.extend(["--calibrate-bytes".into(), b.to_string().into()]);
			}
			let mut judged = String::new();
			if kind == Kind::Session {
				for (k, p) in self.pendings().iter().enumerate() {
					let cand = work.join(format!("candidate{k}"));
					std::fs::create_dir_all(&cand)?;
					std::fs::copy(p.join("driver_config.json"), cand.join("driver_config.json"))?;
					if p.join("station_fcal.txt").is_file() {
						std::fs::copy(p.join("station_fcal.txt"), cand.join("station_fcal.txt"))?;
					}
					args.extend(["--calibrate-candidate".into(), cand.join("driver_config.json").into()]);
					judged.push_str(&format!("candidate{k} {}\n", name_of(p)));
				}
			}
			if kind == Kind::Session {
				args.extend(["--calibrate-session".into(), "1".into()]);
			}
			std::fs::write(
				work.join("job.txt"),
				format!(
					"kind {}\nrecording {}\nbytes {}\nconfig {}\nfcal {}\nstarted {}\n{judged}",
					kind.word(),
					rec.display(),
					bytes.map_or("all".to_string(), |b| b.to_string()),
					stations_of(&self.config),
					fingerprint(&self.fcal),
					stamp()
				),
			)?;
			args.extend([
				"--calibrate-status".into(),
				work.join("status.txt").into(),
				"--configfile".into(),
				work.join("driver_config.json").into(),
				"--station-fcal-file".into(),
				work.join("station_fcal.txt").into(),
				"--hidtap-allow-fs-config".into(),
				"1".into(),
			]);
			for a in &self.pass {
				args.push(sys::os_string(a));
			}
			let log = std::fs::File::create(work.join("calibrate.log"))?;
			let err = log.try_clone()?;
			let mut cmd = Command::new(&self.exe);
			cmd.args(&args).current_dir(&self.cwd).stdin(Stdio::null()).stdout(log).stderr(err);
			let child = sys::spawn(&mut cmd)?;
			std::fs::write(work.join("pid.txt"), child.id().to_string())?;
			Ok(child)
		})();
		let n = name_of(rec);
		match r {
			Ok(c) => {
				self.child = Some(c);
				self.log("started", &n, kind.word());
				self.note(false, format!("calibrating {n} ({}) in the background", kind.word()));
				true
			}
			Err(e) => {
				self.log("failed", &n, &format!("cannot start: {e}"));
				self.note(true, format!("cannot start the calibration of {n}: {e}"));
				false
			}
		}
	}

	/// A calibration that has finished (or died) taken in: log.txt, and its result where it goes. Whether one was
	fn finish_if_done(&mut self) -> bool {
		let work = self.work();
		if !work.join("job.txt").is_file() || self.running() {
			return false;
		}
		self.child = None;
		let job = read(&work.join("job.txt"));
		let status = read(&work.join("status.txt"));
		let kind = Kind::from_word(&field(&job, "kind").unwrap_or_default());
		let rec_path = field(&job, "recording").unwrap_or_default();
		let rec = name_of(Path::new(&rec_path));
		let state = field(&status, "state").unwrap_or_else(|| "interrupted".into());
		let text = field(&status, "text").unwrap_or_default();
		let written = field(&status, "written").as_deref() == Some("1");
		// where the new calibration goes: ready when it was asked for and judged better on its own last quarter, or
		// when it places stations the room's did not (a new or changed room); a session's otherwise pending, for the
		// next session to judge. A new room's that does not place the stations it was made for goes
		let places_new = field(&status, "ready").as_deref() == Some("1");
		let ready_now = written && (kind == Kind::SoFar || places_new);
		let written = written && (kind != Kind::NewRoom || places_new);
		// a calibration's info.txt: its recording, the room's files as they were when it was judged (or made, when it
		// is ready at once), and why
		let info = |of: &str, why: &str| {
			format!(
				"recording {of}\nconfig {}\nfcal {}\nwhy {why}\n",
				field(&job, "config").unwrap_or_default(),
				field(&job, "fcal").unwrap_or_default()
			)
		};
		// the waiting calibrations this session judged: one more win each that predicted it better than the room's, and
		// the first with WINS of them ready (made from what the room's calibration was when this one started; the
		// others, judged against the room's that it replaces, go); one that did not goes
		if kind == Kind::Session {
			let mut ready_one = false;
			for k in 0.. {
				let Some(name) = field(&job, &format!("candidate{k}")) else { break };
				let p = self.pending().join(&name);
				let verdict = field(&status, &format!("candidate{k}")).unwrap_or_default();
				let why = field(&status, &format!("candidate{k}_text")).unwrap_or_default();
				let pinfo = read(&p.join("info.txt"));
				let wins = field(&pinfo, "wins").and_then(|w| w.parse::<u32>().ok()).unwrap_or(0);
				match verdict.as_str() {
					"better" if !ready_one && wins + 1 >= WINS => {
						let _ = std::fs::write(p.join("info.txt"), info(&name, &why) + &format!("wins {}\n", wins + 1));
						if replace_dir(&p, &self.ready()).is_ok() {
							ready_one = true;
							self.log("ready", &name, &format!("judged on {rec}: {why}"));
							self.note(
								false,
								format!(
									"the calibration of {name} predicted {WINS} sessions better than the room's, the last {rec} ({why}): it is taken up at the next start"
								),
							);
						}
					}
					"better" => {
						let t = pinfo.replace(&format!("wins {wins}\n"), "") + &format!("wins {}\n", wins + 1);
						let _ = std::fs::write(p.join("info.txt"), t);
						self.log("judged", &name, &format!("better on {rec}: {why}"));
					}
					"" | "unscored" => {}
					_ => {
						let _ = std::fs::remove_dir_all(&p);
						self.log("dropped", &name, &format!("judged on {rec}: {verdict}: {why}"));
					}
				}
			}
			if ready_one {
				for p in self.pendings() {
					let _ = std::fs::remove_dir_all(&p);
				}
			}
		}
		if written {
			let dest = if ready_now { self.ready() } else { self.pending().join(&rec) };
			let tmp = self.dir.join("next");
			let r = (|| -> std::io::Result<()> {
				if tmp.exists() {
					std::fs::remove_dir_all(&tmp)?;
				}
				std::fs::create_dir_all(&tmp)?;
				std::fs::copy(work.join("driver_config.json"), tmp.join("driver_config.json"))?;
				if work.join("station_fcal.txt").is_file() {
					std::fs::copy(work.join("station_fcal.txt"), tmp.join("station_fcal.txt"))?;
				}
				std::fs::write(tmp.join("info.txt"), info(&rec, &text) + if ready_now { "" } else { "wins 0\n" })?;
				if let Some(d) = dest.parent() {
					std::fs::create_dir_all(d)?;
				}
				replace_dir(&tmp, &dest)
			})();
			let moved = r.is_ok();
			if ready_now && moved {
				self.log("ready", &rec, &text);
				self.note(
					false,
					format!("the calibration of {rec} is ready ({text}): it is taken up at the next start"),
				);
				if kind == Kind::NewRoom {
					self.stop = true;
				}
			} else if moved {
				self.log("pending", &rec, &text);
				self.note(false, format!("the calibration of {rec} waits for the next {WINS} sessions' word ({text})"));
				let p = self.pendings();
				for old in p.iter().take(p.len().saturating_sub(MAX_PENDING)) {
					let _ = std::fs::remove_dir_all(old);
				}
			}
		} else if state == "done" {
			// a new room's that did not place the stations it was made for
			let why = "it does not place the stations the room's calibration lacks";
			self.log("dropped", &rec, why);
			self.note(false, format!("the calibration of {rec} so far: {why}; another one later"));
		} else {
			self.log(&state, &rec, &text);
			if state != "kept" {
				self.note(true, format!("the calibration of {rec}: {state} {text}"));
			} else {
				self.note(false, format!("the calibration of {rec}: {text}"));
			}
		}
		let _ = replace_dir(&work, &self.dir.join("last"));
		true
	}

	/// Each pass of the main loop (once a second of run time): a calibration that has finished taken in, and a new
	/// room's calibrations. True when the runtime is to stop (a new room's calibration is ready)
	pub fn tick(&mut self, rt: &mut Runtime) -> bool {
		let now = rt.run_time();
		if now - self.last_tick < 1.0 && now >= self.last_tick {
			return false;
		}
		self.last_tick = now;
		self.lh1 = rt.saw_lh1();
		self.finish_if_done();
		// stations with OOTX that the config does not place: the room is new, or changed
		let n = rt.active_lighthouses.max(0) as usize;
		let unknown: Vec<u32> = rt
			.bsd
			.iter()
			.take(n)
			.filter(|b| b.ootx_set && b.base_station_id != 0 && !self.known.contains(&b.base_station_id))
			.map(|b| b.base_station_id)
			.collect();
		if !unknown.is_empty() && self.unknown_since.is_none() {
			self.unknown_since = Some(now);
			self.next_try = now + NEW_ROOM_FIRST;
			let ids: Vec<String> = unknown.iter().map(|i| format!("{i:08x}")).collect();
			self.note(
				false,
				format!(
					"stations the room's calibration does not place ({}): the session is calibrated in {:.0} s",
					ids.join(" "),
					NEW_ROOM_FIRST
				),
			);
		}
		if self.unknown_since.is_some()
			&& now >= self.next_try
			&& !self.stop
			&& !self.ready().join("driver_config.json").is_file()
		{
			if !self.running() {
				let bytes = std::fs::metadata(&self.record).map(|m| m.len()).ok();
				let rec = self.record.clone();
				self.start(&rec, bytes, Kind::NewRoom);
			}
			self.tries += 1;
			self.next_try = now + NEW_ROOM_FIRST * f64::powi(2.0, self.tries as i32);
		}
		self.flush(rt);
		self.stop
	}

	/// The runtime has closed (the recording with it): this session calibrated, unless one runs or this session's
	/// recording has been already
	pub fn close(&mut self) {
		self.finish_if_done();
		let rec = self.record.clone();
		let n = name_of(&rec);
		if self.running() || self.logged_kind(&n, Kind::Session) || self.ready_of(&n) {
			return;
		}
		if rec.is_file() {
			self.start(&rec, None, Kind::Session);
		}
	}

	/// Whether log.txt says a calibration of the recording of this kind started
	fn logged_kind(&self, rec: &str, kind: Kind) -> bool {
		read(&self.dir.join("log.txt")).lines().any(|l| {
			let w: Vec<&str> = l.split(' ').collect();
			w.len() >= 4 && w[1] == "started" && w[2] == rec && w[3] == kind.word()
		})
	}

	/// Whether the ready calibration is of this recording (a new room's, or one asked for, this session)
	fn ready_of(&self, rec: &str) -> bool {
		field(&read(&self.ready().join("info.txt")), "recording").as_deref() == Some(rec)
	}

	/// The protocol's `calibrate [start | status | apply]`
	fn command(&mut self, rt: &mut Runtime, args: &[&[u8]]) -> (Vec<u8>, bool) {
		self.finish_if_done();
		let what = args.first().copied().unwrap_or(b"start");
		match what {
			b"start" => {
				if self.running() {
					return (format!("ERR calibrate already running: {}", self.status_text()).into_bytes(), false);
				}
				let bytes = std::fs::metadata(&self.record).map(|m| m.len()).ok();
				let rec = self.record.clone();
				if self.start(&rec, bytes, Kind::SoFar) {
					(format!("OK calibrate started on {} so far", name_of(&rec)).into_bytes(), false)
				} else {
					(b"ERR calibrate cannot start (see the runtime's log)".to_vec(), false)
				}
			}
			b"status" => (format!("OK calibrate {}", self.status_text()).into_bytes(), false),
			b"apply" => {
				if self.ready().join("driver_config.json").is_file() {
					rt.info("calibration: stopping to start again on the ready calibration");
					(b"OK calibrate apply: the runtime starts again on the new calibration".to_vec(), true)
				} else {
					(b"ERR calibrate apply: no calibration is ready".to_vec(), false)
				}
			}
			_ => (b"ERR calibrate usage: calibrate [start|status|apply]".to_vec(), false),
		}
	}

	/// `calibrate status`'s words: `running <kind> <recording> <state> <progress> <moments>`, then `ready <recording>
	/// <why>` when one waits for the next start, `pending <recording>:<wins> ...` for those waiting for sessions' word,
	/// and `last <log.txt's last line>`, joined by " | "
	fn status_text(&mut self) -> String {
		let mut parts = Vec::new();
		if self.running() {
			let job = read(&self.work().join("job.txt"));
			let st = read(&self.work().join("status.txt"));
			let rec = field(&job, "recording").map(|r| name_of(Path::new(&r))).unwrap_or_default();
			parts.push(format!(
				"running {} {rec} {} {} {}",
				field(&job, "kind").unwrap_or_default(),
				field(&st, "state").unwrap_or_else(|| "starting".into()),
				field(&st, "progress").unwrap_or_else(|| "0".into()),
				field(&st, "moments").unwrap_or_else(|| "0".into())
			));
		}
		// a room the calibration does not place, before its first calibration runs: how long until it starts
		if self.unknown_since.is_some() && !self.running() && !self.ready().join("driver_config.json").is_file() {
			parts.push(format!("newroom {:.0}", (self.next_try - self.last_tick).max(0.0)));
		}
		let ready = read(&self.ready().join("info.txt"));
		if !ready.is_empty() {
			parts.push(format!(
				"ready {} {}",
				field(&ready, "recording").unwrap_or_default(),
				field(&ready, "why").unwrap_or_default()
			));
		}
		let pending: Vec<String> = self
			.pendings()
			.iter()
			.map(|p| {
				let wins = field(&read(&p.join("info.txt")), "wins").unwrap_or_else(|| "0".into());
				format!("{}:{wins}", name_of(p))
			})
			.collect();
		if !pending.is_empty() {
			parts.push(format!("pending {}", pending.join(" ")));
		}
		if let Some(l) =
			read(&self.dir.join("log.txt")).lines().rev().find(|l| !l.split(' ').nth(1).is_some_and(|w| w == "started"))
		{
			parts.push(format!("last {}", l));
		}
		if self.lh1 {
			parts.push("lh1".into());
		}
		if parts.is_empty() {
			"idle".into()
		} else {
			parts.join(" | ")
		}
	}
}

mod sys {
	use std::process::{Child, Command};

	/// A command-line word as the OS takes it
	pub fn os_string(b: &[u8]) -> std::ffi::OsString {
		#[cfg(unix)]
		{
			use std::os::unix::ffi::OsStringExt;
			std::ffi::OsString::from_vec(b.to_vec())
		}
		#[cfg(not(unix))]
		{
			String::from_utf8_lossy(b).into_owned().into()
		}
	}

	/// The calibration started on its own: on Windows at idle priority, without a window, in a process group of its
	/// own (the runtime's console's Ctrl+C is not its), and out of the runtime's job when the job lets it, so it goes on
	/// whatever stops the runtime
	pub fn spawn(cmd: &mut Command) -> std::io::Result<Child> {
		#[cfg(windows)]
		{
			use std::os::windows::process::CommandExt;
			const IDLE_PRIORITY_CLASS: u32 = 0x0000_0040;
			const CREATE_NEW_PROCESS_GROUP: u32 = 0x0000_0200;
			const CREATE_BREAKAWAY_FROM_JOB: u32 = 0x0100_0000;
			const CREATE_NO_WINDOW: u32 = 0x0800_0000;
			let flags = IDLE_PRIORITY_CLASS | CREATE_NEW_PROCESS_GROUP | CREATE_NO_WINDOW;
			cmd.creation_flags(flags | CREATE_BREAKAWAY_FROM_JOB);
			match cmd.spawn() {
				Ok(c) => Ok(c),
				Err(_) => {
					cmd.creation_flags(flags);
					cmd.spawn()
				}
			}
		}
		#[cfg(not(windows))]
		{
			cmd.spawn()
		}
	}

	/// Whether a process runs
	pub fn alive(pid: u32) -> bool {
		imp::alive(pid)
	}

	#[cfg(windows)]
	mod imp {
		use std::ffi::c_void;

		const PROCESS_QUERY_LIMITED_INFORMATION: u32 = 0x1000;
		const STILL_ACTIVE: u32 = 259;

		#[link(name = "kernel32")]
		extern "system" {
			fn OpenProcess(access: u32, inherit: i32, pid: u32) -> *mut c_void;
			fn GetExitCodeProcess(h: *mut c_void, code: *mut u32) -> i32;
			fn CloseHandle(h: *mut c_void) -> i32;
		}

		pub fn alive(pid: u32) -> bool {
			// SAFETY: a handle opened here, asked its exit code and closed; a pid that names no process gives none
			unsafe {
				let h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
				if h.is_null() {
					return false;
				}
				let mut code = 0u32;
				let ok = GetExitCodeProcess(h, &mut code) != 0;
				CloseHandle(h);
				ok && code == STILL_ACTIVE
			}
		}
	}

	#[cfg(not(windows))]
	mod imp {
		pub fn alive(pid: u32) -> bool {
			std::path::Path::new(&format!("/proc/{pid}")).exists()
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_live_options_are_left_out_with_their_values() {
		let argv: Vec<Vec<u8>> = [
			"--hidtap",
			"1",
			"--map-solve",
			"1",
			"--configfile",
			"c.json",
			"--core2",
			"--pose-udp",
			"7001",
			"--x",
			"-1",
		]
		.iter()
		.map(|s| s.as_bytes().to_vec())
		.collect();
		let out: Vec<String> =
			without(&argv, LIVE_ONLY).iter().map(|a| String::from_utf8_lossy(a).into_owned()).collect();
		assert_eq!(out, ["--map-solve", "1", "--core2", "--x", "-1"]);
		assert_eq!(value_of(&argv, "--configfile").as_deref(), Some("c.json"));
		assert_eq!(value_of(&argv, "--nothing"), None);
	}

	/// A room's files in a folder of their own, and the live runtime's command line for them
	fn room(name: &str) -> (PathBuf, Vec<Vec<u8>>) {
		let root = std::env::temp_dir().join(format!("sally_autocal_{name}_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&root);
		std::fs::create_dir_all(root.join("survive")).unwrap();
		std::fs::create_dir_all(root.join("logs")).unwrap();
		std::fs::write(root.join("survive/driver_config.json"), "A").unwrap();
		std::fs::write(root.join("survive/station_fcal.txt"), "FA").unwrap();
		std::fs::write(root.join("survive/map_solve.txt"), "M").unwrap();
		for r in [
			"sallytap_20261108_220000.hidtap",
			"sallytap_20261109_010000.hidtap",
			"sallytap_20261109_010000_solver.txt",
		] {
			std::fs::write(root.join("logs").join(r), "x\n").unwrap();
		}
		let s = |p: &Path| p.to_string_lossy().into_owned().into_bytes();
		let argv = vec![
			b"sally run".to_vec(),
			b"--hidtap".to_vec(),
			b"1".to_vec(),
			b"--configfile".to_vec(),
			s(&root.join("survive/driver_config.json")),
			b"--hidtap-record".to_vec(),
			s(&root.join("logs/sallytap_20261109_010000.hidtap")),
			b"--map-solve-file".to_vec(),
			s(&root.join("survive/map_solve.txt")),
		];
		(root, argv)
	}

	/// A calibration as the calibrate process leaves it in work\ (finished, no process), the waiting ones it was given
	fn finished(a: &Auto, kind: Kind, rec: &str, status: &str, config: &str) {
		let w = a.work();
		std::fs::create_dir_all(&w).unwrap();
		let mut job = format!(
			"kind {}\nrecording {}\nbytes all\nconfig {}\nfcal {}\nstarted x\n",
			kind.word(),
			a.record.with_file_name(rec).display(),
			stations_of(&a.config),
			fingerprint(&a.fcal)
		);
		if kind == Kind::Session {
			for (k, p) in a.pendings().iter().enumerate() {
				job.push_str(&format!("candidate{k} {}\n", name_of(p)));
			}
		}
		std::fs::write(w.join("job.txt"), job).unwrap();
		std::fs::write(w.join("status.txt"), status).unwrap();
		std::fs::write(w.join("driver_config.json"), config).unwrap();
		std::fs::write(w.join("station_fcal.txt"), format!("F{config}")).unwrap();
	}

	const R1: &str = "sallytap_20261108_220000.hidtap";
	const R2: &str = "sallytap_20261109_010000.hidtap";
	const R3: &str = "sallytap_20261109_020000.hidtap";

	#[test]
	fn a_session_waits_for_two_more_then_is_taken_up_at_the_start() {
		let (root, argv) = room("flow");
		let mut a = Auto::new(&argv).unwrap();
		assert_eq!(a.previous_recording().map(|p| name_of(&p)).as_deref(), Some(R1));
		// the last session's calibration: pending
		finished(&a, Kind::Session, R1, "state done\nwritten 1\nready 0\ntext 4 stations\n", "B");
		assert!(a.finish_if_done());
		assert_eq!(a.pendings().len(), 1);
		assert!(!a.ready().exists());
		// the next session's: the pending one predicted it better (one win); this one waits beside it
		let st =
			"state done\nwritten 1\nready 0\ncandidate0 better\ncandidate0_text it 0.05, the room's 0.07\ntext 4\n";
		finished(&a, Kind::Session, R2, st, "C");
		assert!(a.finish_if_done());
		assert!(!a.ready().exists());
		let p = a.pendings();
		assert_eq!(p.iter().map(|p| name_of(p)).collect::<Vec<_>>(), [R1, R2]);
		assert_eq!(field(&read(&p[0].join("info.txt")), "wins").as_deref(), Some("1"));
		// and the one after: the first's second win makes it ready; the other, judged against the room's it replaces,
		// goes; this session's waits
		let st = "state done\nwritten 1\nready 0\ncandidate0 better\ncandidate0_text x\ncandidate1 better\ncandidate1_text y\ntext 4\n";
		finished(&a, Kind::Session, R3, st, "D");
		assert!(a.finish_if_done());
		assert_eq!(read(&a.ready().join("driver_config.json")), "B");
		assert_eq!(a.pendings().iter().map(|p| name_of(p)).collect::<Vec<_>>(), [R3]);
		// the next start takes it up: the old files put aside, the map with them
		let mut b = Auto::new(&argv).unwrap();
		b.adopt();
		assert_eq!(read(&root.join("survive/driver_config.json")), "B");
		assert_eq!(read(&root.join("survive/station_fcal.txt")), "FB");
		assert!(!root.join("survive/map_solve.txt").exists());
		let names: Vec<String> = std::fs::read_dir(root.join("survive"))
			.unwrap()
			.flatten()
			.map(|e| e.file_name().to_string_lossy().into_owned())
			.collect();
		assert!(names.iter().any(|n| n.starts_with("driver_config.json.bak_")));
		assert!(names.iter().any(|n| n.starts_with("map_solve.txt.bak_")));
		assert!(!b.ready().exists());
		assert!(read(&b.dir.join("log.txt")).contains(&format!(" adopted {R1} ")));
		let _ = std::fs::remove_dir_all(&root);
	}

	#[test]
	fn a_ready_one_made_from_another_config_is_dropped_and_a_judged_one_that_lost_goes() {
		let (root, argv) = room("drop");
		let mut a = Auto::new(&argv).unwrap();
		finished(&a, Kind::Session, R1, "state done\nwritten 1\nready 1\ntext new room\n", "B");
		assert!(a.finish_if_done());
		assert!(a.ready().exists(), "a calibration placing stations the room's did not is ready at once");
		// the room's config changed by hand since: a station placed
		let placed = "\"lighthouse0\":{\n\"index\":\"0\",\n\"id\":\"609880014\",\n\"mode\":\"7\",\n\"pose\":[\"1\",\"2\",\"3\",\"1\",\"0\",\"0\",\"0\"],\n\"OOTXSet\":\"1\",\n\"PositionSet\":\"1\"\n}\n";
		std::fs::write(root.join("survive/driver_config.json"), placed).unwrap();
		let mut b = Auto::new(&argv).unwrap();
		b.adopt();
		assert_eq!(read(&root.join("survive/driver_config.json")), placed);
		assert!(!b.ready().exists());
		assert!(read(&b.dir.join("log.txt")).contains(" dropped "));
		// a pending one that did not predict the next session better goes
		finished(&b, Kind::Session, R1, "state done\nwritten 1\nready 0\ntext x\n", "P");
		assert!(b.finish_if_done());
		let st = "state failed\nwritten 0\ncandidate0 not-better\ncandidate0_text it 0.08, the room's 0.06\n";
		finished(&b, Kind::Session, R2, st, "Q");
		assert!(b.finish_if_done());
		assert!(b.pendings().is_empty() && !b.ready().exists());
		// a calibration asked for that the room's still beats changes nothing
		finished(&b, Kind::SoFar, R2, "state kept\nwritten 0\ntext the room's predicts as well\n", "R");
		assert!(b.finish_if_done());
		assert!(!b.ready().exists());
		// a new room's that does not place the stations it was made for goes
		finished(&b, Kind::NewRoom, R2, "state done\nwritten 1\nready 0\ntext 3 stations\n", "S");
		assert!(b.finish_if_done());
		assert!(!b.ready().exists() && b.pendings().is_empty() && !b.stop);
		// one that does is ready, and the runtime is to stop for it
		finished(&b, Kind::NewRoom, R2, "state done\nwritten 1\nready 1\ntext 4 stations\n", "T");
		assert!(b.finish_if_done());
		assert_eq!(read(&b.ready().join("driver_config.json")), "T");
		assert!(b.stop);
		assert_eq!(read(&root.join("survive/driver_config.json")), placed);
		let _ = std::fs::remove_dir_all(&root);
	}

	#[test]
	fn the_same_stations_survive_the_runtime_writing_them_again() {
		let a = "609880014:-1.837475,-1.174987,2.277723,0.655081,0.400424,-0.296132,-0.568186";
		let b = "609880014:-1.837475,-1.174986,2.277723,0.655081,0.400424,-0.296132,-0.568186";
		assert!(same_stations(a, b));
		let flipped = "609880014:-1.837475,-1.174986,2.277723,-0.655081,-0.400424,0.296132,0.568186";
		assert!(same_stations(a, flipped));
		let moved = "609880014:-1.837975,-1.174986,2.277723,0.655081,0.400424,-0.296132,-0.568186";
		assert!(!same_stations(a, moved));
		assert!(!same_stations(a, "none"));
		assert!(same_stations("none", "none"));
	}

	#[test]
	fn a_key_value_file_reads_by_key() {
		let t = "state done\nwritten 1\ntext the new one 0.050 mrad\n";
		assert_eq!(field(t, "state").as_deref(), Some("done"));
		assert_eq!(field(t, "text").as_deref(), Some("the new one 0.050 mrad"));
		assert_eq!(field(t, "nope"), None);
	}
}
