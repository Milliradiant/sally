//! A small test host (a runtime in memory: stations, devices, a clock, the log and the recording collected) and the
//! map solve run on synthetic light: stations around a room, a tracker resting at a few places, its hits made by the
//! same reprojection the solve uses; inline, and on the solve thread behind a lock of the test's.

use sally_lighthouse::BaseStationCal;
use sally_math::Pose;

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, Weak};

use crate::host::{DeviceId, Host, Velocity};
use crate::linalg::{mv3, q2r};
use crate::model::{model_angle, Snapshot, Station};
use crate::{Locked, MapSolve, Settings};

/// The map solve's options as the C registers them (STRUCT_CONFIG_ITEM and STATIC_CONFIG_ITEM), for the test host's
/// store: the runtime's registry has them
const DEFAULTS: &[(&str, &str)] = &[
	("map-solve-batch", "400"),
	("map-solve-step", "0.25"),
	("map-solve-window", "0.022"),
	("map-solve-vmax", "0.05"),
	("map-solve-wmax", "0.15"),
	("map-solve-min-hits", "6"),
	("map-solve-min-snaps", "40"),
	("map-solve-min-station-hits", "150"),
	("map-solve-first", "10"),
	("map-solve-period", "5"),
	("map-solve-tau", "600"),
	("map-solve-iters", "8"),
	("map-solve-prior-pos", "0.05"),
	("map-solve-prior-rot", "0.01"),
	("map-solve-rate-pos", "0.002"),
	("map-solve-rate-rot", "0.0005"),
	("map-solve-huber", "0.001"),
	("map-solve-reject", "0.008"),
	("map-solve-max-move", "0.05"),
	("map-solve-max-rot", "0.02"),
	("map-solve-load-weight", "0.25"),
	("map-solve-save-period", "60"),
	("map-solve-relocate", "1"),
	("map-solve-move-resid", "0.003"),
	("map-solve-move-fit", "0.0004"),
	("map-solve-move-ratio", "3"),
	("map-solve-move-rest-ratio", "2"),
	("map-solve-move-min-snaps", "12"),
	("map-solve-move-agree-pos", "0.01"),
	("map-solve-move-agree-rot", "0.003"),
	("map-solve-move-min", "0.005"),
	("map-solve-move-min-rot", "0.002"),
	("map-solve-move-pending-max", "60"),
	("map-solve-threaded", "1"),
	("map-solve-file", ""),
	("map-solve-test-move", ""),
	("map-solve-test-load", ""),
	("station-fcal-file", ""),
	("device-calibration-file", ""),
];

/// A station of the test host (ctx->bsd[lh])
#[derive(Clone, Default)]
struct TestStation {
	pose: Pose,
	position_set: bool,
	ootx_set: bool,
	id: u32,
	mode: u8,
	fcal: [BaseStationCal; 2],
	moved: i8,
	moved_time: f64,
}

/// A device of the test host (a SurviveObject)
#[derive(Clone, Default)]
struct TestDevice {
	locations: Vec<f64>,
	normals: Vec<f64>,
	velocity: Velocity,
	serial: String,
	codename: String,
}

/// The runtime in memory
#[derive(Default)]
struct TestHost {
	now: f64,
	stations: Vec<TestStation>,
	devices: Vec<TestDevice>,
	infos: Vec<String>,
	warns: Vec<String>,
	records: Vec<String>,
	/// the poses raw_lighthouse_pose was given, per station
	lh_poses: Vec<(usize, Pose)>,
	/// the options set (the command line's), over DEFAULTS
	config: HashMap<String, String>,
}

impl TestHost {
	/// An option's text: as set, or its registered default
	fn option(&self, tag: &str) -> Option<String> {
		self.config.get(tag).cloned().or_else(|| DEFAULTS.iter().find(|d| d.0 == tag).map(|d| d.1.to_string()))
	}
}

impl Host for TestHost {
	fn run_time(&mut self) -> f64 {
		self.now
	}
	fn relative_time(&mut self) -> f64 {
		0.0
	}
	fn active_lighthouses(&self) -> i32 {
		self.stations.len() as i32
	}
	fn get_bsd_idx(&mut self, channel: u8) -> i8 {
		self.stations.iter().position(|s| s.mode == channel).map(|i| i as i8).unwrap_or(-1)
	}
	fn bsd_pose(&self, lh: usize) -> Pose {
		self.stations[lh].pose
	}
	fn bsd_position_set(&self, lh: usize) -> bool {
		self.stations[lh].position_set
	}
	fn bsd_ootx_set(&self, lh: usize) -> bool {
		self.stations[lh].ootx_set
	}
	fn bsd_base_station_id(&self, lh: usize) -> u32 {
		self.stations[lh].id
	}
	fn bsd_mode(&self, lh: usize) -> u8 {
		self.stations[lh].mode
	}
	fn bsd_fcal(&self, lh: usize) -> [BaseStationCal; 2] {
		self.stations[lh].fcal
	}
	fn set_bsd_fcal(&mut self, lh: usize, fcal: &[BaseStationCal; 2]) {
		self.stations[lh].fcal = *fcal;
	}
	fn bsd_sally_moved(&self, lh: usize) -> i8 {
		self.stations[lh].moved
	}
	fn set_bsd_sally_moved(&mut self, lh: usize, moved: i8) {
		self.stations[lh].moved = moved;
	}
	fn set_bsd_sally_moved_time(&mut self, lh: usize, t: f64) {
		self.stations[lh].moved_time = t;
	}
	fn raw_lighthouse_pose(&mut self, lh: usize, pose: &Pose) {
		self.stations[lh].pose = *pose;
		self.stations[lh].position_set = true;
		self.lh_poses.push((lh, *pose));
	}
	fn sensor_ct(&self, dev: DeviceId) -> i32 {
		(self.devices[dev].locations.len() / 3) as i32
	}
	fn sensor_locations_set(&self, dev: DeviceId) -> bool {
		!self.devices[dev].locations.is_empty()
	}
	fn sensor_normals_set(&self, dev: DeviceId) -> bool {
		!self.devices[dev].normals.is_empty()
	}
	fn sensor_location(&self, dev: DeviceId, sensor: usize) -> [f64; 3] {
		let l = &self.devices[dev].locations[3 * sensor..3 * sensor + 3];
		[l[0], l[1], l[2]]
	}
	fn set_sensor_location(&mut self, dev: DeviceId, sensor: usize, loc: &[f64; 3]) {
		self.devices[dev].locations[3 * sensor..3 * sensor + 3].copy_from_slice(loc);
	}
	fn sensor_normal(&self, dev: DeviceId, sensor: usize) -> [f64; 3] {
		let n = &self.devices[dev].normals[3 * sensor..3 * sensor + 3];
		[n[0], n[1], n[2]]
	}
	fn timebase_hz(&self, _dev: DeviceId) -> i32 {
		48_000_000
	}
	fn velocity(&self, dev: DeviceId) -> Velocity {
		self.devices[dev].velocity
	}
	fn serial_number(&self, dev: DeviceId) -> String {
		self.devices[dev].serial.clone()
	}
	fn codename(&self, dev: DeviceId) -> String {
		self.devices[dev].codename.clone()
	}
	fn long_timecode_light(&self, _dev: DeviceId, timecode: u32) -> u64 {
		timecode as u64
	}
	fn configf(&mut self, tag: &str, def: f64) -> f64 {
		self.option(tag).map(|v| v.parse().unwrap()).unwrap_or(def)
	}
	fn configi(&mut self, tag: &str, def: u32) -> u32 {
		self.option(tag).map(|v| v.parse().unwrap()).unwrap_or(def)
	}
	fn configs(&mut self, tag: &str, def: &str) -> String {
		self.option(tag).unwrap_or_else(|| def.to_string())
	}
	fn log_level(&self) -> i32 {
		0
	}
	fn info(&mut self, msg: &str) {
		self.infos.push(msg.to_string());
	}
	fn warn(&mut self, msg: &str) {
		self.warns.push(msg.to_string());
	}
	fn record(&mut self, line: &str) {
		self.records.push(line.to_string());
	}
}

/// A station at `pos` looking at the middle of the room (its -z toward the origin, its y up)
fn station_pose(pos: [f64; 3]) -> Pose {
	let n = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
	let z = [pos[0] / n, pos[1] / n, pos[2] / n]; // -z looks at the origin
	let up = [0.0, 0.0, 1.0];
	let mut x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]];
	let xn = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
	x = [x[0] / xn, x[1] / xn, x[2] / xn];
	let y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
	// the columns of lh2world are the station's axes in the world
	let r = [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
	Pose { pos, rot: crate::linalg::r2q(&r) }
}

/// A tracker's 18 sensors on a 5 cm sphere's upper half, facing out
fn sensors() -> (Vec<f64>, Vec<f64>) {
	let (mut loc, mut nrm) = (Vec::new(), Vec::new());
	for i in 0..18 {
		let a = i as f64 * 2.399963;
		let z = 0.9 - 0.05 * i as f64;
		let r = (1.0 - z * z).sqrt();
		let n = [r * a.cos(), r * a.sin(), z];
		nrm.extend_from_slice(&n);
		loc.extend_from_slice(&[0.05 * n[0], 0.05 * n[1], 0.05 * n[2]]);
	}
	(loc, nrm)
}

/// The test room: four stations, one tracker
fn room() -> (TestHost, Vec<Pose>) {
	let truth = vec![
		station_pose([2.5, 2.0, 2.2]),
		station_pose([-2.4, 2.1, 2.1]),
		station_pose([-2.2, -2.3, 2.3]),
		station_pose([2.3, -2.2, 2.0]),
	];
	let mut host = TestHost::default();
	for (i, p) in truth.iter().enumerate() {
		host.stations.push(TestStation {
			pose: *p,
			position_set: true,
			ootx_set: true,
			id: 1000 + i as u32,
			mode: [1u8, 3, 7, 9][i],
			..TestStation::default()
		});
	}
	let (locations, normals) = sensors();
	host.devices.push(TestDevice {
		locations,
		normals,
		serial: "LHR-TEST0001".into(),
		codename: "T20".into(),
		..TestDevice::default()
	});
	(host, truth)
}

/// The angle station `st` (true pose) sees sensor `k` of the device at `dev_pose` on `plane`, as the solve models it
fn true_angle(host: &TestHost, st: &Pose, dev_pose: &Pose, k: usize, plane: i8) -> f64 {
	let mut s = Snapshot::default();
	s.pt[0] = host.sensor_location(0, k);
	s.plane[0] = plane;
	let station = Station { r: q2r(&st.rot), p: st.pos, ..Station::default() };
	model_angle(&s, 0, &q2r(&dev_pose.rot), &dev_pose.pos, &station)
}

/// The device's first light timecode for a run from `t0`
fn first_tc(t0: f64) -> u64 {
	(t0 * 48e6) as u64 + 1
}

/// Step i of a run from `t0`: the device resting at one of a few places, every station's sweeps every fifth step (50
/// Hz), then its pose (250 Hz); the stations' light from `truth`
fn step(host: &mut TestHost, ms: &mut MapSolve, truth: &[Pose], t0: f64, i: usize, tc: &mut u64) {
	let places = [[0.3, 0.2, 1.0], [-0.4, 0.5, 1.2], [0.1, -0.6, 0.9], [-0.2, -0.1, 1.4]];
	host.now = t0 + i as f64 / 250.0;
	let place = places[(i / 1000) % places.len()];
	let dev_pose = Pose { pos: place, rot: [0.98, 0.1, 0.05, 0.15] };
	let n = (dev_pose.rot.iter().map(|v| v * v).sum::<f64>()).sqrt();
	let dev_pose = Pose { pos: dev_pose.pos, rot: dev_pose.rot.map(|v| v / n) };
	// every fifth pose a sweep of each station: the sensors facing it
	if i.is_multiple_of(5) {
		for (lh, st) in truth.iter().enumerate() {
			let r = q2r(&dev_pose.rot);
			for k in 0..18 {
				let p = mv3(&r, &host.sensor_location(0, k));
				let nrm = mv3(&r, &host.sensor_normal(0, k));
				let to = [st.pos[0] - p[0] - place[0], st.pos[1] - p[1] - place[1], st.pos[2] - p[2] - place[2]];
				if nrm[0] * to[0] + nrm[1] * to[1] + nrm[2] * to[2] <= 0.0 {
					continue;
				}
				for plane in 0..2i8 {
					let a = true_angle(host, st, &dev_pose, k, plane);
					let ch = host.stations[lh].mode;
					ms.sweep_angle(host, 0, ch, k as i32, (*tc as u32).wrapping_add(k as u32), plane, a);
				}
			}
		}
	}
	*tc += 48_000_000 / 250;
	ms.imupose(host, 0, *tc, &dev_pose);
}

/// Runs the map solve on the device resting at a few places for `secs` s of light, the stations' light from `truth`
fn run(host: &mut TestHost, ms: &mut MapSolve, truth: &[Pose], t0: f64, secs: f64) {
	let mut tc = first_tc(t0);
	for i in 0..(secs * 250.0) as usize {
		step(host, ms, truth, t0, i, &mut tc);
	}
}

#[test]
fn solves_a_resting_device_and_holds_the_map() {
	let (mut host, truth) = room();
	host.config.insert("map-solve-min-snaps".into(), "20".into());
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 40.0);
	let st = ms.stats();
	assert!(st.solves >= 4, "{st:?}");
	assert_eq!(st.applied, st.solves, "{st:?} {:?}", host.records);
	assert!(host.records.iter().any(|r| r.starts_with("MAPSOLVE 1 ")), "{:?}", host.records);
	// the light fits the map: the solved stations stay where they are
	for lh in 0..4 {
		let t = ms.target(lh).expect("a solved pose");
		let d = sally_math::dist3d(&t.pos, &truth[lh].pos);
		assert!(d < 1e-4, "station {lh} {d}");
	}
	assert!(host.warns.is_empty(), "{:?}", host.warns);
	ms.close(&mut host);
	assert!(host.infos.iter().any(|m| m.starts_with("map-solve: 4 solves") || m.contains(" solves, ")));
}

#[test]
fn finds_and_relocates_a_moved_station() {
	let (mut host, truth) = room();
	// the map has station 2 moved by 3 cm and 15 mrad from where its light comes from
	host.config.insert("map-solve-min-snaps".into(), "20".into());
	host.config.insert("map-solve-test-move".into(), "12,2,0.03,0,0,0,0.015,0".into());
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 60.0);
	assert!(host.infos.iter().any(|m| m.starts_with("map-solve: TEST: station 2 (ch 7)")), "{:?}", host.infos);
	let moves: Vec<&String> = host.records.iter().filter(|r| r.starts_with("MAPMOVE")).collect();
	assert!(moves.iter().any(|r| r.contains(" pending ")), "{moves:?}");
	assert!(moves.iter().any(|r| r.contains(" applied ")), "{moves:?}");
	assert_eq!(ms.stats().relocations, 1);
	assert_eq!(host.stations[2].moved, 2);
	// back where its light comes from
	let t = ms.target(2).expect("a solved pose");
	let d = sally_math::dist3d(&t.pos, &truth[2].pos);
	assert!(d < 2e-3, "{d}");
}

/// The test's context lock: the map solve and the host behind one mutex, as a runtime keeps them
struct World {
	ms: Option<MapSolve>,
	host: TestHost,
}

/// The solve thread's way to the world, counting the solves it starts
struct WorldLock {
	world: Weak<Mutex<World>>,
	settings: Settings,
	starts: Arc<AtomicUsize>,
}

impl Locked for WorldLock {
	fn settings(&mut self) -> Settings {
		// run_solve's first step, with the solver locked
		self.starts.fetch_add(1, Ordering::SeqCst);
		self.settings.clone()
	}
	fn relative_time(&mut self) -> f64 {
		0.0
	}
	fn locked<R>(&mut self, f: impl FnOnce(&mut MapSolve, &mut dyn Host) -> R) -> Option<R> {
		let world = self.world.upgrade()?;
		let mut g = world.lock().unwrap();
		let World { ms, host } = &mut *g;
		Some(f(ms.as_mut()?, host))
	}
}

/// The map solve with `options` on `secs` s of light, inline and on its thread, each solve the thread is asked for
/// waited for: the two do the same, to the bit. The inline run's host, for the scenario's own checks
fn inline_and_threaded(options: &[(&str, &str)], secs: f64) -> TestHost {
	let t0 = 0.001;
	let (mut host, truth) = room();
	for (k, v) in options {
		host.config.insert(k.to_string(), v.to_string());
	}
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, t0, secs);
	ms.close(&mut host);
	let inline = host;

	// the same light with the solve thread: each step under the lock, and each solve the hooks ask for waited for (the
	// thread starts it with the solver locked, which it holds to the end), so that it lands where the inline one did
	let (mut host, _) = room();
	for (k, v) in options {
		host.config.insert(k.to_string(), v.to_string());
	}
	let ms = MapSolve::start(&mut host);
	let (first, period) = (ms.s.first, ms.s.period);
	let (settings, solver) = (ms.s.clone(), ms.solver.clone());
	let world = Arc::new(Mutex::new(World { ms: Some(ms), host }));
	let starts = Arc::new(AtomicUsize::new(0));
	{
		let lock = WorldLock { world: Arc::downgrade(&world), settings, starts: starts.clone() };
		world.lock().unwrap().ms.as_mut().unwrap().start_thread(lock);
	}
	let mut tc = first_tc(t0);
	let (mut last_solve, mut asked) = (0.0, 0);
	for i in 0..(secs * 250.0) as usize {
		let now = {
			let mut g = world.lock().unwrap();
			let World { ms, host } = &mut *g;
			step(host, ms.as_mut().unwrap(), &truth, t0, i, &mut tc);
			host.now
		};
		// imupose_fn's test for a due solve
		if now > first && now - last_solve >= period {
			last_solve = now;
			asked += 1;
			while starts.load(Ordering::SeqCst) < asked {
				std::thread::yield_now();
			}
			drop(solver.lock().unwrap());
		}
	}
	// a solve every period from first on
	assert!(asked as f64 >= (secs - first) / period, "{asked}");
	// survive_close: the thread stopped and joined without the lock, then the close
	let thread = world.lock().unwrap().ms.as_mut().unwrap().stop_thread().expect("the solve thread");
	thread.join().unwrap();
	let mut g = world.lock().unwrap();
	let World { ms, host } = &mut *g;
	ms.take().unwrap().close(host);
	assert_eq!(starts.load(Ordering::SeqCst), asked);

	assert_eq!(host.records, inline.records);
	assert_eq!(host.infos, inline.infos);
	assert_eq!(host.warns, inline.warns);
	let bits = |p: &[(usize, Pose)]| -> Vec<(usize, Vec<u64>)> {
		p.iter().map(|(l, p)| (*l, p.pos.iter().chain(&p.rot).map(|v| v.to_bits()).collect())).collect()
	};
	assert_eq!(bits(&host.lh_poses), bits(&inline.lh_poses));
	let moved =
		|h: &TestHost| -> Vec<(i8, u64)> { h.stations.iter().map(|s| (s.moved, s.moved_time.to_bits())).collect() };
	assert_eq!(moved(host), moved(&inline));
	inline
}

#[test]
fn solves_on_its_thread_as_inline() {
	let snaps = ("map-solve-min-snaps", "20");
	// a moved station found, flagged and relocated
	let h = inline_and_threaded(&[snaps, ("map-solve-test-move", "12,2,0.03,0,0,0,0.015,0")], 60.0);
	assert!(h.records.iter().any(|r| r.starts_with("MAPMOVE ") && r.contains(" applied ")), "{:?}", h.records);
	// a relocation not confirmed in time: back to the map's place
	let h = inline_and_threaded(
		&[snaps, ("map-solve-move-pending-max", "4"), ("map-solve-test-move", "12,0,0.03,0,0.01,0,0.015,0.01")],
		30.0,
	);
	assert!(h.infos.iter().any(|m| m.contains("relocation not confirmed within 4 s")), "{:?}", h.infos);
	// solves that move a station too far, not applied
	let h = inline_and_threaded(
		&[
			snaps,
			("map-solve-max-move", "0.0001"),
			("map-solve-max-rot", "0.00001"),
			("map-solve-test-move", "12,2,0.01,0,0,0,0.005,0"),
		],
		25.0,
	);
	assert!(h.records.iter().any(|r| r.starts_with("MAPSOLVE ") && r.contains(" 0 LH")), "{:?}", h.records);
}

#[test]
fn reads_and_writes_the_map_file() {
	let dir = std::env::temp_dir().join(format!("sally-map-test-{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let file = dir.join("map.txt").to_string_lossy().into_owned();
	let (mut host, truth) = room();
	host.config.insert("map-solve-min-snaps".into(), "20".into());
	host.config.insert("map-solve-file".into(), file.clone());
	let config = host.config.clone();
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 25.0);
	ms.close(&mut host);
	let text = std::fs::read_to_string(&file).unwrap();
	assert!(text.starts_with("# sally map-solve: live-solved station poses"));
	assert_eq!(text.lines().filter(|l| l.starts_with("LH ")).count(), 4);
	assert_eq!(text.lines().filter(|l| l.starts_with("LAM ")).count(), 16);
	// the next session starts from it
	let (mut host, _) = room();
	host.config = config;
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 0.1);
	assert!(
		host.infos.iter().any(|m| m.starts_with("map-solve: 4 station(s) taken from the saved map")),
		"{:?}",
		host.infos
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// A pose's seven values as the map file writes them ("%.9f")
fn pose9(p: &Pose) -> String {
	let v: Vec<String> = p.pos.iter().chain(&p.rot).map(|v| format!("{v:.9}")).collect();
	v.join(" ")
}

/// Every value of every pose finite
fn all_finite(poses: &[(usize, Pose)]) -> bool {
	poses.iter().all(|(_, p)| p.pos.iter().chain(&p.rot).all(|v| v.is_finite()))
}

/// A number in a text that is not finite: a nan or an inf as printf writes them (a unit after it, mm or mrad)
fn has_not_finite(text: &str) -> bool {
	text.split(|c: char| !(c.is_ascii_alphanumeric() || c == '.' || c == '-' || c == '+')).any(|t| {
		let t = t.trim_end_matches("mrad").trim_end_matches("mm").trim_start_matches(['-', '+']);
		t == "nan" || t == "inf"
	})
}

#[test]
fn leaves_out_saved_lines_that_are_not_finite() {
	let dir = std::env::temp_dir().join(format!("sally-map-test-nan-{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let file = dir.join("map.txt").to_string_lossy().into_owned();
	let (mut host, truth) = room();
	// a damaged file: one station's frame anchor with a NaN (the C held the frame with it: every station NaN, in the
	// tracker and back in the file), an LH line with one, and an information entry past a double (1e400: inf)
	let mut text = String::from("# sally map-solve: damaged\n");
	for (i, p) in truth.iter().enumerate() {
		let mut anchor = pose9(p);
		if i == 1 {
			anchor = anchor.replacen(&format!("{:.9}", p.pos[0]), "nan", 1);
		}
		text.push_str(&format!("ANCHOR {} {} {} 1\n", 1000 + i, pose9(p), anchor));
	}
	let mut lh = pose9(&truth[2]).split(' ').map(str::to_string).collect::<Vec<_>>();
	lh[2] = "-nan".into();
	text.push_str(&format!("LH 1002 {}\n", lh.join(" ")));
	text.push_str("LAM 1000 1000 1e400 1 2 3\n");
	std::fs::write(&file, text).unwrap();
	host.config.insert("map-solve-min-snaps".into(), "20".into());
	host.config.insert("map-solve-file".into(), file.clone());
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 25.0);
	// each left out, with a warning (stations 0, 1 and 2 are channels 1, 3 and 7)
	for what in ["ANCHOR line of LH ch 3 ", "LH line of LH ch 7 ", "LAM line of LH ch 1 and ch 1 "] {
		assert!(
			host.warns.iter().any(|w| w.contains(what) && w.ends_with("not finite: left out")),
			"{what}: {:?}",
			host.warns
		);
	}
	assert_eq!(host.warns.len(), 3, "{:?}", host.warns);
	// and nothing that is not finite reached the tracker, the solves or the file
	assert!(all_finite(&host.lh_poses));
	let solves: Vec<&String> = host.records.iter().filter(|r| r.starts_with("MAPSOLVE ")).collect();
	assert!(!solves.is_empty(), "{:?}", host.records);
	assert!(solves.iter().all(|r| !has_not_finite(r)), "{solves:?}");
	let st = ms.stats();
	assert_eq!(st.applied, st.solves, "{st:?}");
	for lh in 0..4 {
		let d = sally_math::dist3d(&ms.target(lh).expect("a solved pose").pos, &truth[lh].pos);
		assert!(d < 1e-4, "station {lh} {d}");
	}
	ms.close(&mut host);
	let back = std::fs::read_to_string(&file).unwrap();
	assert!(back.starts_with("# sally map-solve: live-solved station poses"), "{back}");
	assert!(!has_not_finite(&back), "{back}");
	assert!(has_not_finite("LH 1 -nan 0") && has_not_finite("x infmm") && has_not_finite("(info inf)"));
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_solve_that_is_not_finite_is_not_applied() {
	let (mut host, truth) = room();
	host.config.insert("map-solve-min-snaps".into(), "20".into());
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 20.0);
	let before = ms.stats();
	assert!(before.applied >= 1 && before.rejected_moves == 0, "{before:?}");
	// a station's solved pose that is not a number (the solve's NaN: the C applied it, the frame hold spreading it to
	// every station): from here on the solves are not applied, and the other stations keep their maps
	ms.target[2].pos[0] = f64::NAN;
	let n = host.records.len();
	run(&mut host, &mut ms, &truth, 20.001, 12.0);
	let after = ms.stats();
	assert!(after.solves > before.solves, "{after:?}");
	assert_eq!(after.applied, before.applied, "{after:?}");
	assert_eq!(after.rejected_moves, after.solves - before.solves, "{after:?}");
	let solves: Vec<&String> = host.records[n..].iter().filter(|r| r.starts_with("MAPSOLVE ")).collect();
	assert!(!solves.is_empty() && solves.iter().all(|r| r.split(' ').nth(6) == Some("0")), "{solves:?}");
	for lh in [0, 1, 3] {
		let t = ms.target(lh).expect("a solved pose");
		assert!(t.pos.iter().chain(&t.rot).all(|v| v.is_finite()), "station {lh} {t:?}");
	}
}

#[test]
fn rigid_fit_refuses_points_that_are_not_finite() {
	let from = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
	let mut to = from;
	for p in to.iter_mut() {
		p[0] += 0.5;
	}
	let (r, t) = crate::relocate::rigid_fit(&from, &to).expect("a fit");
	assert!((t[0] - 0.5).abs() < 1e-9 && t[1].abs() < 1e-9 && (r[0] - 1.0).abs() < 1e-9, "{r:?} {t:?}");
	for bad in [f64::NAN, f64::INFINITY] {
		let mut to = to;
		to[2][1] = bad;
		assert!(crate::relocate::rigid_fit(&from, &to).is_none());
		let mut from = from;
		from[3][2] = -bad;
		assert!(crate::relocate::rigid_fit(&from, &to).is_none());
	}
}

/// A write of the map file that fails (a full disk; here the temporary file is a link to /dev/full) leaves the saved
/// map as it was: the C moved what it had written over it
#[cfg(target_os = "linux")]
#[test]
fn a_failed_write_keeps_the_saved_map() {
	use std::os::unix::fs::FileTypeExt;
	if !std::fs::metadata("/dev/full").is_ok_and(|m| m.file_type().is_char_device()) {
		return;
	}
	let dir = std::env::temp_dir().join(format!("sally-map-test-full-{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).unwrap();
	let file = dir.join("map.txt");
	std::fs::write(&file, "# the map before\n").unwrap();
	std::os::unix::fs::symlink("/dev/full", dir.join("map.txt.tmp")).unwrap();
	let (mut host, truth) = room();
	host.config.insert("map-solve-min-snaps".into(), "20".into());
	host.config.insert("map-solve-file".into(), file.to_string_lossy().into_owned());
	let mut ms = MapSolve::start(&mut host);
	run(&mut host, &mut ms, &truth, 0.001, 25.0);
	assert!(ms.stats().applied >= 1);
	ms.close(&mut host);
	assert!(
		host.warns
			.iter()
			.any(|w| w.starts_with("map-solve: cannot write ") && w.ends_with("; the saved map stays as it was")),
		"{:?}",
		host.warns
	);
	assert!(!host.infos.iter().any(|m| m.contains("solved map written")), "{:?}", host.infos);
	assert!(!std::fs::symlink_metadata(&file).unwrap().file_type().is_symlink());
	assert_eq!(std::fs::read_to_string(&file).unwrap(), "# the map before\n");
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn model_angle_with_the_stations_terms_is_the_plain_reprojection() {
	use sally_lighthouse::{survive_reproject_axis_x_gen2, survive_reproject_axis_y_gen2};
	// a station at the origin, unturned, and a hit with no motion: model_angle is the reprojection of the sensor's
	// place, which it now takes with the station's terms (Station::set_fcal); to the bit as the plain calls
	let mut x: u64 = 0x2545_f491_4f6c_dd1d;
	let mut next = || {
		x ^= x << 13;
		x ^= x >> 7;
		x ^= x << 17;
		(x >> 11) as f64 / (1u64 << 53) as f64 - 0.5
	};
	let mut st = Station { r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], ..Station::default() };
	for round in 0..20000 {
		if round % 100 != 0 {
			let mut fcal = [BaseStationCal::default(); 2];
			for c in fcal.iter_mut() {
				(c.phase, c.tilt, c.curve, c.gibpha, c.gibmag) =
					(next() * 0.1, next() * 0.1, next() * 0.1, next() * 6.0, next() * 0.02);
				(c.ogeephase, c.ogeemag) = (next() * 6.0, next() * 0.2);
			}
			st.set_fcal(fcal);
		} else {
			// the zero calibration of Station::default
			st = Station { r: st.r, ..Station::default() };
		}
		let mut s = Snapshot::default();
		s.pt[0] = [next() * 4.0, next() * 4.0, next() * 4.0 - 2.5];
		for plane in 0..2i8 {
			s.plane[0] = plane;
			let m = model_angle(&s, 0, &st.r, &[0.0; 3], &st);
			let plain = if plane == 0 {
				survive_reproject_axis_x_gen2(&st.fcal, &s.pt[0])
			} else {
				survive_reproject_axis_y_gen2(&st.fcal, &s.pt[0])
			};
			assert_eq!(m.to_bits(), plain.to_bits(), "{st:?} {:?} {plane}", s.pt[0]);
		}
	}
}
