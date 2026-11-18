//! `sally calibrate`: a room's base stations placed from recordings, every device solving them together, and the map
//! levelled with the devices' accelerometers (`sally_map::calibrate`, docs/sally_design.md 4.2). It replays each
//! recording (`--hidtap-playback`, once or more) as `sally replay` does, without tracking (the stations' saved poses
//! are not used: force-calibrate), takes every device's light and IMU as they come, finds the moments each device held
//! still, pools them (devices by serial, stations by their id), solves, and writes the stations into the config file:
//! the calibration every session's map starts from and is anchored to. The live runtime runs it by itself (`autocal`).
//!
//! The recording wants every device still now and then in many places and at many headings: walking the room and
//! stopping, a tracker set down here and there, or turning slowly on the spot and pausing. Gravity needs each device
//! still at several headings; turning on the spot gives that.
//!
//! The stations' corrections (their factory calibration's errors) are solved with them, from the factory values the
//! stations broadcast, and written to `station-fcal-file` (or `station_fcal.txt` beside the config). The old one, and
//! the map file (`map-solve-file`, whose frame was the old calibration's), are put aside as `<file>.bak_<time>`: the
//! next session starts from the new calibration with nothing to set by hand. A station of the room's calibration that
//! is not in the recordings keeps its place and its corrections.
//!
//! The check: a calibration is written only when it predicts light it was not solved from better than the room's
//! calibration so far (`judge`): by default the last quarter of the recordings' still moments, solved from the rest.
//! For the live runtime's:
//! - `--calibrate-candidate <config>` (its corrections the station_fcal.txt beside it; once or more):
//!   the candidate and the room's calibration each predict all of the recordings' light, which neither was solved
//!   from, and the verdict on each goes into the status (`candidate<k>`, `candidate<k>_text`);
//! - `--calibrate-session 1`: no check of the new one (the next session's recording is its judge): it is written
//!   whatever it scores, and the status says `ready 1` when the room's calibration placed none of its stations, or
//!   not all;
//! - `--calibrate-status <file>`: where it stands, written whole as it goes (`state replaying|solving|done|kept|failed`
//!   and the rest, `key value` lines);
//! - `--calibrate-bytes <n>`: the recording's first n bytes only (one still being written).
//!
//! The exit code: 0 written, 3 kept (the room's calibration predicts as well), 1 failed, 2 usage.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use sally_map::calibrate::Scored;
use sally_map::calibrate::{self, Device, Hit, Imu, Moment, Options, StationIn, StillFinder};
use sally_map::fcal_solve::{fcal_moved, fcal_terms};
use sally_map::map_file::text_mode_out;
use sally_runtime::config::{Config, LighthouseRecord, SC_GET, SC_OVERRIDE, SC_SETCONFIG};
use sally_runtime::runtime::Observer;
use sally_runtime::{ObjectId, Runtime};

use crate::replay;

/// The device clock (LH2 devices: 48 MHz), which the still moments are timed on until the devices' configs say
const HZ: f64 = 48e6;

/// What the replay hands over: each device's still moments as they complete
#[derive(Default)]
struct Collected {
	finders: Vec<Option<StillFinder>>,
	moments: Vec<Moment>,
	opt: Options,
	/// the status, and the moments the recordings before this one gave
	status: Option<Arc<Mutex<Status>>>,
	before: usize,
}

/// `--calibrate-status`: where the calibration stands, for the runtime that started it
struct Status {
	path: String,
	fields: Vec<(String, String)>,
	written: Option<Instant>,
}

impl Status {
	fn set(&mut self, key: &str, value: impl Into<String>) {
		let v = value.into().replace('\n', " ");
		match self.fields.iter_mut().find(|(k, _)| k == key) {
			Some(f) => f.1 = v,
			None => self.fields.push((key.to_string(), v)),
		}
	}

	/// Written whole (through a temporary beside it), at most every 2 s unless `now`
	fn write(&mut self, now: bool) {
		if !now && self.written.is_some_and(|t| t.elapsed().as_secs_f64() < 2.0) {
			return;
		}
		self.written = Some(Instant::now());
		let mut t = String::new();
		for (k, v) in &self.fields {
			t.push_str(&format!("{k} {v}\n"));
		}
		let tmp = format!("{}.tmp", self.path);
		if std::fs::write(&tmp, t).is_ok() {
			let _ = std::fs::rename(&tmp, &self.path);
		}
	}
}

/// The status set and written at once, when there is one
fn report(status: &Option<Arc<Mutex<Status>>>, fields: &[(&str, String)]) {
	if let Some(s) = status {
		let mut s = s.lock().unwrap_or_else(|e| e.into_inner());
		for (k, v) in fields {
			s.set(k, v.clone());
		}
		s.write(true);
	}
}

impl Collected {
	fn finder(&mut self, obj: ObjectId) -> &mut StillFinder {
		if self.finders.len() <= obj {
			self.finders.resize_with(obj + 1, || None);
		}
		let opt = self.opt.clone();
		let dev = Device { hz: HZ, ..Device::default() };
		self.finders[obj].get_or_insert_with(|| StillFinder::new(obj, &dev, &opt))
	}
}

struct Collector(Arc<Mutex<Collected>>);

impl Observer for Collector {
	fn hit(&mut self, obj: ObjectId, lh: usize, plane: i8, sensor: i32, timecode: u64, angle: f64) {
		if !(0..=255).contains(&sensor) || lh > 255 || !angle.is_finite() {
			return;
		}
		let mut c = self.0.lock().unwrap_or_else(|e| e.into_inner());
		c.finder(obj).hit(Hit { tc: timecode, lh: lh as u8, sensor: sensor as u8, plane, angle });
	}
	fn imu(&mut self, obj: ObjectId, timecode: u64, accel: &[f64; 3], gyro: &[f64; 3]) {
		let mut c = self.0.lock().unwrap_or_else(|e| e.into_inner());
		if let Some(m) = c.finder(obj).imu(Imu { tc: timecode, acc: *accel, gyro: *gyro }) {
			c.moments.push(m);
			if let Some(s) = &c.status {
				let n = c.before + c.moments.len();
				let mut s = s.lock().unwrap_or_else(|e| e.into_inner());
				s.set("progress", format!("{:.3}", replay::progress()));
				s.set("moments", n.to_string());
				s.write(false);
			}
		}
	}
}

/// The options calibrate sets over the command line's: the saved station poses not used, nothing locked (the config
/// is written), and nothing but the replay itself running
const FORCED: [(&str, &str); 8] = [
	("--force-calibrate", "1"),
	("--lock-lighthouses", "0"),
	("--map-solve", "0"),
	("--diagnosis", "0"),
	("--diagrec", "0"),
	("--pose-udp", "0"),
	("--runtime-port", "0"),
	("--playback-factor", "0"),
];

/// What the recordings gave so far: the devices (by serial) and stations (by id) of all of them, and every still
/// moment, its device and its hits' stations the pool's
#[derive(Default)]
struct Pool {
	devs: Vec<Device>,
	stations: Vec<StationIn>,
	moments: Vec<Moment>,
	odd_clocks: Vec<String>,
	/// the config's calibration as the first recording found it (per station id): the new frame is tied to it
	anchors: Vec<(u32, sally_math::Pose)>,
}

impl Pool {
	/// The recording's still moments added: its devices and stations found in the pool or added to it
	fn add(&mut self, rt: &mut Runtime, moments: Vec<Moment>) {
		let mut dev_of = Vec::new();
		for o in &rt.objects {
			let flat = |v: &Option<Vec<f64>>| -> Vec<[f64; 3]> {
				v.as_ref().map(|v| v.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect()).unwrap_or_default()
			};
			let n = o.sensor_ct.max(0) as usize;
			let mut sensors = flat(&o.sensor_locations);
			let mut normals = flat(&o.sensor_normals);
			sensors.truncate(n);
			normals.truncate(n);
			let name = if o.serial_number.is_empty() { o.codename.clone() } else { o.serial_number.clone() };
			if o.timebase_hz > 0 && o.timebase_hz as f64 != HZ {
				self.odd_clocks.push(format!(
					"calibrate: {name} runs at {} Hz, not 48 MHz: its moments are mistimed",
					o.timebase_hz
				));
			}
			let i = match self.devs.iter().position(|d| d.name == name && d.sensors.len() == sensors.len()) {
				Some(i) => i,
				None => {
					self.devs.push(Device { name, hz: HZ, sensors, normals });
					self.devs.len() - 1
				}
			};
			dev_of.push(i);
		}
		let mut st_of = [usize::MAX; 256];
		let n = rt.active_lighthouses.max(0) as usize;
		for (l, slot) in st_of.iter_mut().enumerate().take(n) {
			let b = &rt.bsd[l];
			if !b.ootx_set {
				let ch = b.mode;
				rt.warn(&format!(
					"calibrate: the station on channel {ch} sent no OOTX in this recording: its light left out"
				));
				continue;
			}
			let i = match self.stations.iter().position(|s| s.id == b.base_station_id) {
				Some(i) => i,
				None => {
					self.stations.push(StationIn {
						lh: self.stations.len(),
						channel: b.mode,
						id: b.base_station_id,
						fcal: b.fcal,
					});
					self.stations.len() - 1
				}
			};
			*slot = i;
		}
		for mut m in moments {
			let Some(&d) = dev_of.get(m.dev) else { continue };
			m.dev = d;
			m.hits.retain_mut(|h| {
				let s = st_of[h.lh as usize];
				h.lh = s as u8;
				s != usize::MAX
			});
			if !m.hits.is_empty() {
				self.moments.push(m);
			}
		}
	}
}

/// What the live runtime asks of a calibration (none of it by hand): the module's `--calibrate-*`
#[derive(Default)]
struct Job {
	candidates: Vec<String>,
	session: bool,
	bytes: Option<u64>,
	status: Option<Arc<Mutex<Status>>>,
}

/// A calibration's predictions take the room's place only when they beat the room's on this many more of the moments
/// than they lose, in standard deviations of a coin's (a sign test): the moments of a session are not each other's
/// equals in what they say, so a handful more is no verdict
const JUDGE_Z: f64 = 3.0;

/// How two calibrations predict the same moments: each moment's median |residual| under each, the moments where
/// `a`'s is the smaller and where it is the larger, and the median |residual| over all the moments both place
#[derive(Clone, Copy, Debug, Default)]
pub struct Judged {
	pub moments: usize,
	pub wins: usize,
	pub losses: usize,
	pub med_a: f64,
	pub med_b: f64,
}

impl Judged {
	/// The sign test's z: how far a's wins are above half, in a coin's standard deviations
	pub fn z(&self) -> f64 {
		let n = (self.wins + self.losses) as f64;
		if n == 0.0 {
			0.0
		} else {
			(self.wins as f64 - n / 2.0) / (n / 4.0).sqrt()
		}
	}

	/// `a` predicts better: more of the moments, by the sign test, and the median residual too
	pub fn better(&self) -> bool {
		self.z() >= JUDGE_Z && self.med_a < self.med_b
	}

	/// `b` predicts better
	pub fn worse(&self) -> bool {
		-self.z() >= JUDGE_Z && self.med_b < self.med_a
	}

	pub fn text(&self, a: &str, b: &str) -> String {
		format!(
			"{a} {:.3} mrad, {b} {:.3} mrad, {a} closer on {} of {} moments (z {:+.1})",
			self.med_a * 1e3,
			self.med_b * 1e3,
			self.wins,
			self.moments,
			self.z()
		)
	}
}

/// A calibration's residuals moment by moment (`calibrate::score_moments`'s)
type Predictions = Vec<Option<Vec<f64>>>;

/// Two calibrations' predictions of the same moments compared
pub fn judge(a: &[Option<Vec<f64>>], b: &[Option<Vec<f64>>]) -> Judged {
	let med = |v: &[f64]| -> f64 {
		let mut v = v.to_vec();
		v.sort_by(|x, y| x.total_cmp(y));
		if v.is_empty() {
			f64::NAN
		} else {
			v[v.len() / 2]
		}
	};
	let mut j = Judged::default();
	let (mut all_a, mut all_b) = (Vec::new(), Vec::new());
	for (x, y) in a.iter().zip(b) {
		let (Some(x), Some(y)) = (x, y) else { continue };
		if x.is_empty() || y.is_empty() {
			continue;
		}
		j.moments += 1;
		let (mx, my) = (med(x), med(y));
		if mx < my {
			j.wins += 1;
		} else if mx > my {
			j.losses += 1;
		}
		all_a.extend_from_slice(x);
		all_b.extend_from_slice(y);
	}
	j.med_a = med(&all_a);
	j.med_b = med(&all_b);
	j
}

/// `sally calibrate --hidtap-playback <tap recording> [--hidtap-playback <another> ...] --configfile <config>
/// [--station-fcal-file <file>] [--map-solve-file <file>] [--calibrate-...] ...`
pub fn main(argv: &[Vec<u8>]) -> i32 {
	// the recordings, the job, and the rest of the command line for each
	let mut taps = Vec::new();
	let mut job = Job::default();
	let mut rest = Vec::new();
	let mut i = 0;
	let text = |b: &[u8]| String::from_utf8_lossy(b).into_owned();
	while i < argv.len() {
		let a = argv[i].as_slice();
		let v = argv.get(i + 1);
		match (a, v) {
			(b"--hidtap-playback", Some(v)) => taps.push(v.clone()),
			(b"--calibrate-candidate", Some(v)) => job.candidates.push(text(v)),
			(b"--calibrate-session", Some(v)) => job.session = v.as_slice() != b"0",
			(b"--calibrate-bytes", Some(v)) => job.bytes = text(v).parse().ok(),
			(b"--calibrate-status", Some(v)) => {
				job.status = Some(Arc::new(Mutex::new(Status { path: text(v), fields: Vec::new(), written: None })))
			}
			_ => {
				rest.push(argv[i].clone());
				i += 1;
				continue;
			}
		}
		i += 2;
	}
	if taps.is_empty() {
		eprintln!("sally calibrate: --hidtap-playback <tap recording> is required (once or more)");
		return 2;
	}
	let status = job.status.clone();
	report(&status, &[("state", "replaying".into()), ("progress", "0".into()), ("moments", "0".into())]);
	let job = Arc::new(job);
	let pool = Arc::new(Mutex::new(Pool::default()));
	let last = taps.len() - 1;
	for (k, tap) in taps.iter().enumerate() {
		let mut args = rest.clone();
		args.push(b"--hidtap-playback".to_vec());
		args.push(tap.clone());
		for (key, v) in FORCED {
			args.push(key.as_bytes().to_vec());
			args.push(v.as_bytes().to_vec());
		}
		let before = pool.lock().unwrap_or_else(|e| e.into_inner()).moments.len();
		let collected = Arc::new(Mutex::new(Collected { status: status.clone(), before, ..Collected::default() }));
		let c2 = collected.clone();
		let pool2 = pool.clone();
		let pool3 = pool.clone();
		let job2 = job.clone();
		// the last recording only, when it is being recorded
		let limit = if k == last { job.bytes } else { None };
		let code = replay::run(
			&args,
			limit,
			move |rt| {
				// the room's calibration so far, before anything is written over it
				if k == 0 {
					let mut p = pool3.lock().unwrap_or_else(|e| e.into_inner());
					for b in rt.bsd.iter().take(rt.active_lighthouses.max(0) as usize) {
						if b.position_set && b.base_station_id != 0 {
							p.anchors.push((b.base_station_id, b.pose));
						}
					}
				}
				rt.info(&format!(
					"calibrate: recording {} of {}: every device's still moments are collected; the stations are placed at the end",
					k + 1,
					last + 1
				));
				rt.observer = Some(Box::new(Collector(c2)));
			},
			move |rt| {
				rt.observer = None;
				let (moments, opt) = {
					let mut c = collected.lock().unwrap_or_else(|e| e.into_inner());
					(std::mem::take(&mut c.moments), c.opt.clone())
				};
				let mut pool = pool2.lock().unwrap_or_else(|e| e.into_inner());
				let n = moments.len();
				pool.add(rt, moments);
				rt.info(&format!("calibrate: recording {} of {}: {n} still moments", k + 1, last + 1));
				if k == last {
					finish(rt, &mut pool, &opt, &job2)
				} else {
					0
				}
			},
		);
		if code != 0 {
			// a replay that ended before the calibration said how it went
			if status.as_ref().is_some_and(|s| !matches!(field_of(s, "state").as_str(), "done" | "kept" | "failed")) {
				let t = format!("the replay of {} ended with {code}", text(tap));
				report(&status, &[("state", "failed".into()), ("written", "0".into()), ("text", t)]);
			}
			return code;
		}
	}
	0
}

/// A status field's value (empty when unset)
fn field_of(s: &Arc<Mutex<Status>>, key: &str) -> String {
	let s = s.lock().unwrap_or_else(|e| e.into_inner());
	s.fields.iter().find(|(k, _)| k == key).map(|(_, v)| v.clone()).unwrap_or_default()
}

/// A calibration as a config file and the corrections beside it have it, as `score` takes it: the stations it places
/// (by id) with the factory calibration the recordings' stations broadcast, corrected where its corrections are to
/// those same factory values
fn scored_from(
	stations: &[StationIn], placed: &[(u32, sally_math::Pose)], fc: &[(u32, [f64; 10], [f64; 10])],
) -> Vec<Scored> {
	placed
		.iter()
		.filter_map(|(id, pose)| {
			let s = stations.iter().find(|s| s.id == *id)?;
			let f = fc
				.iter()
				.find(|(i, f, _)| i == id && f.iter().zip(fcal_terms(&s.fcal)).all(|(a, b)| (a - b).abs() < 1e-6))
				.map(|(_, _, d)| fcal_moved(&s.fcal, d))
				.unwrap_or(s.fcal);
			Some(Scored { id: *id, pose: *pose, fcal: f })
		})
		.collect()
}

/// The stations a config file places (id, pose)
fn placed_in(config: &str) -> Vec<(u32, sally_math::Pose)> {
	let mut c = Config::default();
	c.read(Some(config.as_bytes()));
	let mut out = Vec::new();
	for i in 0..sally_runtime::config::NUM_GEN2_LIGHTHOUSES {
		let mut r = LighthouseRecord::default();
		if c.read_lighthouse(i as u8, &mut r) && r.position_set && r.base_station_id != 0 {
			out.push((r.base_station_id, r.pose));
		}
	}
	out
}

/// Two calibrations judged on the moments, each on the stations both place
fn judge_sets(pool: &Pool, moments: &[Moment], a: &[Scored], b: &[Scored], opt: &Options) -> Judged {
	let both = |x: &[Scored], y: &[Scored]| -> Vec<Scored> {
		x.iter().filter(|s| y.iter().any(|t| t.id == s.id)).copied().collect()
	};
	let (a2, b2) = (both(a, b), both(b, a));
	let sa = calibrate::score_moments(&pool.devs, moments, &pool.stations, &a2, opt);
	let sb = calibrate::score_moments(&pool.devs, moments, &pool.stations, &b2, opt);
	judge(&sa, &sb)
}

/// The solve over the pool, its report, and the stations into the config (through the last recording's runtime)
fn finish(rt: &mut Runtime, pool: &mut Pool, opt: &Options, job: &Job) -> i32 {
	let status = &job.status;
	report(status, &[("state", "solving".into()), ("progress", "1".into())]);
	for w in std::mem::take(&mut pool.odd_clocks) {
		rt.warn(&w);
	}
	let mut opt = opt.clone();
	opt.anchors = pool.anchors.clone();
	let opt = &opt;
	let moments = std::mem::take(&mut pool.moments);
	rt.info(&format!(
		"calibrate: {} still moments of {} devices, {} stations with OOTX",
		moments.len(),
		pool.devs.len(),
		pool.stations.len()
	));
	report(status, &[("moments", moments.len().to_string()), ("anchors", pool.anchors.len().to_string())]);
	let fc_file = corrections_file(rt);
	let old_fc = read_corrections(&fc_file);
	let room = scored_from(&pool.stations, &pool.anchors, &old_fc);
	// the stations the recordings saw (with OOTX) that the room's calibration places
	let seen_placed: Vec<u32> = room.iter().map(|s| s.id).collect();

	// the candidates, each against the room's calibration on all of the moments, neither solved from them (the room's
	// scored once for each set of stations it is judged on)
	let mut room_scores: Vec<(Vec<u32>, Predictions)> = Vec::new();
	for (k, cand) in job.candidates.iter().enumerate() {
		let placed = placed_in(cand);
		let dir = std::path::Path::new(cand).parent().map(|d| d.to_path_buf()).unwrap_or_default();
		let cfc = read_corrections(&dir.join("station_fcal.txt").to_string_lossy());
		let set = scored_from(&pool.stations, &placed, &cfc);
		let missing: Vec<String> =
			seen_placed.iter().filter(|id| !set.iter().any(|s| s.id == **id)).map(|id| format!("{id:08x}")).collect();
		let (verdict, why) = if room.is_empty() {
			("unscored", "the room's calibration places none of the recordings' stations".to_string())
		} else if !missing.is_empty() {
			("incomplete", format!("it does not place {} of the room's stations", missing.join(" ")))
		} else {
			let mut ids: Vec<u32> = set.iter().map(|s| s.id).filter(|id| seen_placed.contains(id)).collect();
			ids.sort_unstable();
			let mine: Vec<Scored> = set.iter().filter(|s| ids.contains(&s.id)).copied().collect();
			if !room_scores.iter().any(|(i, _)| *i == ids) {
				let theirs: Vec<Scored> = room.iter().filter(|s| ids.contains(&s.id)).copied().collect();
				room_scores
					.push((ids.clone(), calibrate::score_moments(&pool.devs, &moments, &pool.stations, &theirs, opt)));
			}
			let rs = &room_scores.iter().find(|(i, _)| *i == ids).expect("scored above").1;
			let cs = calibrate::score_moments(&pool.devs, &moments, &pool.stations, &mine, opt);
			let j = judge(&cs, rs);
			let t = j.text("it", "the room's");
			if j.moments == 0 {
				("unscored", "no moment both place".to_string())
			} else if j.better() {
				("better", t)
			} else {
				("not-better", t)
			}
		};
		rt.info(&format!("calibrate: the candidate {cand}: {verdict}: {why}"));
		report(status, &[(&format!("candidate{k}"), verdict.into()), (&format!("candidate{k}_text"), why)]);
	}

	// the check (not for a session's own calibration, which the next session judges): solved from the first three
	// quarters of the moments, it and the room's calibration each predict the last quarter. A new one that places
	// stations the room's does not is kept back only when it predicts the stations both place worse
	let new_ids: Vec<u32> = pool.stations.iter().map(|s| s.id).collect();
	let adds = new_ids.iter().any(|id| !seen_placed.contains(id));
	if !job.session && !room.is_empty() {
		let n_test = moments.len() / 4;
		let (train, test) = moments.split_at(moments.len() - n_test);
		match calibrate::calibrate(&pool.devs, train.to_vec(), &pool.stations, opt) {
			Ok(c) => {
				let new: Vec<Scored> = c
					.stations
					.iter()
					.map(|p| Scored {
						id: p.id,
						pose: p.pose,
						fcal: fcal_moved(&pool.stations[p.lh].fcal, &p.correction),
					})
					.collect();
				let j = judge_sets(pool, test, &new, &room, opt);
				let t = j.text("the new calibration", "the room's so far");
				rt.info(&format!(
					"calibrate: the check, on the last quarter of the moments ({} of them, not solved from): {t}",
					test.len()
				));
				let keep = if adds { j.worse() } else { !j.better() };
				if keep {
					rt.warn("calibrate: the new calibration does not predict the light better than the room's so far: the config is left as it was");
					report(status, &[("state", "kept".into()), ("written", "0".into()), ("text", t)]);
					return 3;
				}
				report(status, &[("check", t)]);
			}
			Err(e) => rt.warn(&format!("calibrate: the check could not be made ({e}); going on without it")),
		}
	}
	let cal = match calibrate::calibrate(&pool.devs, moments, &pool.stations, opt) {
		Ok(c) => c,
		Err(e) => {
			rt.warn(&format!("calibrate: {e}"));
			rt.warn("calibrate: the config is left as it was");
			report(status, &[("state", "failed".into()), ("written", "0".into()), ("text", e.to_string())]);
			return 1;
		}
	};
	for l in &cal.lines {
		rt.info(&format!("calibrate: {l}"));
	}
	// the stations into the config, by id in the last recording's stations: placed ones with their pose, the room's
	// ones the recordings did not see where the room's calibration has them, the rest without; all with the factory
	// calibration they broadcast (the corrections apply to it each session)
	let n = rt.active_lighthouses.max(0) as usize;
	for b in rt.bsd.iter_mut().take(n) {
		b.position_set = false;
	}
	let mut written = Vec::new();
	for p in &cal.stations {
		match (0..n).find(|&l| rt.bsd[l].ootx_set && rt.bsd[l].base_station_id == p.id) {
			Some(l) => {
				rt.bsd[l].pose = p.pose;
				rt.bsd[l].position_set = true;
				rt.bsd[l].fcal = pool.stations[p.lh].fcal;
				rt.config_set_lighthouse(l);
				written.push(p);
			}
			None => rt.warn(&format!(
				"calibrate: the station {:08x} (ch {}) is not in the last recording: not written",
				p.id, p.channel
			)),
		}
	}
	let mut kept = Vec::new();
	for l in 0..n {
		let id = rt.bsd[l].base_station_id;
		// (the same room: the recordings saw one of its stations at least)
		if !rt.bsd[l].position_set && !room.is_empty() && !pool.stations.iter().any(|s| s.id == id) {
			if let Some((_, pose)) = pool.anchors.iter().find(|(a, _)| *a == id) {
				rt.bsd[l].pose = *pose;
				rt.bsd[l].position_set = true;
				kept.push(id);
				rt.info(&format!(
					"calibrate: the station {id:08x} (ch {}) is not in the recordings: kept where the room's calibration has it",
					rt.bsd[l].mode
				));
			}
		}
		if !written.iter().any(|p| p.id == id) {
			rt.config_set_lighthouse(l);
		}
	}
	// the frame's z is up and its floor at 0
	rt.floor_offset = 0.0;
	rt.config.configf("floor-offset", SC_SETCONFIG | SC_OVERRIDE, 0.0);
	rt.config_save();
	let stamp = sally_os::localtime::local_now().map(|t| t.stamp()).unwrap_or_else(|| "old".into());
	let file = String::from_utf8_lossy(&rt.config.file_path()).into_owned();
	// the stations' corrections, beside the old ones put aside; a station kept, its old ones
	if opt.corrections {
		let fc = fc_file;
		let mut lines: Vec<(u32, [f64; 10], [f64; 10])> =
			written.iter().map(|p| (p.id, fcal_terms(&pool.stations[p.lh].fcal), p.correction)).collect();
		lines.extend(old_fc.iter().filter(|(id, _, _)| kept.contains(id)).copied());
		let aside = put_aside(&fc, &stamp);
		match write_corrections(&fc, &lines, cal.snapshots) {
			Ok(()) => rt.info(&format!(
				"calibrate: the {} stations' corrections written to {fc}{}",
				lines.len(),
				aside.map(|a| format!(" (the old ones are {a})")).unwrap_or_default()
			)),
			Err(e) => rt.warn(&format!("calibrate: cannot write the corrections to {fc}: {e}")),
		}
	}
	// the map the live solve kept was in the old calibration's frame: the next session starts from this one
	let map = String::from_utf8_lossy(&rt.config.configs("map-solve-file", SC_GET, Some(b"")).unwrap_or_default())
		.into_owned();
	if !map.is_empty() {
		if let Some(a) = put_aside(&map, &stamp) {
			rt.info(&format!(
				"calibrate: the live map {map} put aside as {a}: the next session solves from this calibration"
			));
		}
	}
	let (t, az) = cal.up_tilt;
	rt.info(&format!(
		"calibrate: {} stations placed from {} snapshots (light residual median {:.3} mrad), z levelled to gravity (it was {:.2} mrad off the first station's frame toward {:.1} deg), floor at 0: written to {file}",
		written.len(),
		cal.snapshots,
		cal.med_res * 1e3,
		t * 1e3,
		az
	));
	rt.info("calibrate: its z is up: a session on it wants no --track-gravity-tilt");
	// ready at once: the room's calibration placed none of these stations, or not all of them
	let ready = room.is_empty() || written.iter().any(|p| !seen_placed.contains(&p.id));
	let summary = format!(
		"{} stations from {} snapshots, light residual median {:.3} mrad",
		written.len(),
		cal.snapshots,
		cal.med_res * 1e3
	);
	report(
		status,
		&[
			("state", "done".into()),
			("written", "1".into()),
			("ready", if ready { "1" } else { "0" }.into()),
			("text", summary),
		],
	);
	0
}

/// station-fcal-file, or station_fcal.txt beside the config
fn corrections_file(rt: &mut Runtime) -> String {
	let fc = String::from_utf8_lossy(&rt.config.configs("station-fcal-file", SC_GET, Some(b"")).unwrap_or_default())
		.into_owned();
	if !fc.is_empty() {
		return fc;
	}
	let file = String::from_utf8_lossy(&rt.config.file_path()).into_owned();
	let dir = std::path::Path::new(&file).parent().map(|d| d.to_path_buf()).unwrap_or_default();
	dir.join("station_fcal.txt").to_string_lossy().into_owned()
}

/// The corrections a station-fcal-file holds: per station id the factory values and the correction (none when it
/// cannot be read)
fn read_corrections(path: &str) -> Vec<(u32, [f64; 10], [f64; 10])> {
	let Ok(t) = std::fs::read_to_string(path) else { return Vec::new() };
	let mut out = Vec::new();
	for line in t.lines() {
		let f: Vec<&str> = line.split_whitespace().collect();
		if f.len() != 22 || f[0] != "FCAL" {
			continue;
		}
		let Ok(id) = f[1].parse::<u32>() else { continue };
		let v: Vec<f64> = f[2..].iter().filter_map(|x| x.parse().ok()).collect();
		if v.len() != 20 {
			continue;
		}
		let mut a = [0.0; 10];
		let mut d = [0.0; 10];
		a.copy_from_slice(&v[..10]);
		d.copy_from_slice(&v[10..]);
		out.push((id, a, d));
	}
	out
}

/// The file renamed to `<file>.bak_<stamp>` (the name it has now), None when there is none to put aside
fn put_aside(path: &str, stamp: &str) -> Option<String> {
	if !std::path::Path::new(path).is_file() {
		return None;
	}
	let to = format!("{path}.bak_{stamp}");
	std::fs::rename(path, &to).ok().map(|()| to)
}

/// station-fcal-file's text: per station its id, the factory values the correction is to (the file's order: phase,
/// tilt, curve, gibbous phase, gibbous magnitude, each axis 0 then 1) and the correction (corrections.rs reads it)
fn write_corrections(path: &str, stations: &[(u32, [f64; 10], [f64; 10])], snapshots: usize) -> std::io::Result<()> {
	let stamp = sally_os::localtime::local_now().map(|t| t.stamp()).unwrap_or_default();
	let mut t = format!(
		"# sally station-fcal-file (sally calibrate, {stamp}, solved with the stations from {snapshots} snapshots)\n\
		 # FCAL <station id> <factory phase0 phase1 tilt0 tilt1 curve0 curve1 gibpha0 gibpha1 gibmag0 gibmag1> <the same 10 deltas>\n"
	);
	for (id, f, d) in stations {
		t.push_str(&format!("FCAL {id}"));
		for v in f.iter().chain(d.iter()) {
			t.push_str(&format!(" {v}"));
		}
		t.push('\n');
	}
	let tmp = format!("{path}.tmp");
	std::fs::write(&tmp, text_mode_out(&t))?;
	std::fs::rename(&tmp, path)
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Moments' residuals: each moment's hits around its value
	fn moments(v: &[f64]) -> Vec<Option<Vec<f64>>> {
		v.iter().map(|&x| Some(vec![x * 0.9, x, x * 1.1])).collect()
	}

	#[test]
	fn a_calibration_is_better_only_on_most_moments_and_the_median() {
		// a closer on 70 of 100 moments, its median lower: z = (70 - 50) / 5 = 4
		let a: Vec<f64> = (0..100).map(|i| if i < 70 { 0.05 } else { 0.08 }).collect();
		let b = vec![0.06; 100];
		let j = judge(&moments(&a), &moments(&b));
		assert_eq!((j.moments, j.wins, j.losses), (100, 70, 30));
		assert!((j.z() - 4.0).abs() < 1e-12);
		assert!(j.better() && !j.worse());
		// 55 of 100 is a coin's
		let a: Vec<f64> = (0..100).map(|i| if i < 55 { 0.05 } else { 0.08 }).collect();
		let j = judge(&moments(&a), &moments(&b));
		assert!(!j.better() && !j.worse());
		// a moment only one of them places is not judged
		let mut a = moments(&[0.05; 10]);
		a[3] = None;
		let j = judge(&a, &moments(&[0.06; 10]));
		assert_eq!(j.moments, 9);
		// the other way round
		let j = judge(&moments(&b), &moments(&(0..100).map(|i| if i < 70 { 0.05 } else { 0.08 }).collect::<Vec<_>>()));
		assert!(j.worse() && !j.better());
	}

	#[test]
	fn a_config_says_which_stations_it_places() {
		let dir = std::env::temp_dir().join(format!("sally_cal_test_{}", std::process::id()));
		let _ = std::fs::create_dir_all(&dir);
		let p = dir.join("c.json");
		std::fs::write(
			&p,
			"\"v\":\"0\",\n\"lighthouse0\":{\n\"index\":\"0\",\n\"id\":\"287454020\",\n\"mode\":\"7\",\n\"pose\":[\"1.0\",\"2.0\",\"3.0\",\"1.0\",\"0.0\",\"0.0\",\"0.0\"],\n\"OOTXSet\":\"1\",\n\"PositionSet\":\"1\"\n}\n\"lighthouse1\":{\n\"index\":\"1\",\n\"id\":\"2578103244\",\n\"mode\":\"9\",\n\"pose\":[\"0\",\"0\",\"0\",\"1\",\"0\",\"0\",\"0\"],\n\"OOTXSet\":\"1\",\n\"PositionSet\":\"0\"\n}\n",
		)
		.unwrap();
		let placed = placed_in(&p.to_string_lossy());
		let _ = std::fs::remove_dir_all(&dir);
		assert_eq!(placed.len(), 1);
		assert_eq!(placed[0].0, 287454020);
		assert_eq!(placed[0].1.pos, [1.0, 2.0, 3.0]);
	}
}
