//! The map solve (sally-map, the C runtime's driver_map_solve.c) as one of the runtime's drivers: the host it reads the
//! runtime through and writes it through (its stations, its devices, the config store, the text), its hooks, the
//! variables bound to its options, and its solve thread's way to the context lock.
//!
//! The thread is the C's: map-solve-threaded, unless the playback factor is 0, and only where the runtime runs with
//! the context lock (`sally run`; `sally replay` runs on one thread and always solves inline). It reaches the map solve
//! where the runtime keeps it, in its driver's slot, with the lock (`ThreadLock`). survive_close stops it first
//! (`Driver::stop`: the hooks pass every call on from then, as the C's do once g_ms is cleared) and joins it without
//! the lock, before the map solve's own close.

use std::sync::{Mutex, Weak};
use std::thread::JoinHandle;

use sally_lighthouse::BaseStationCal;
use sally_map::{DeviceId, MapSolve, Settings, Source, Velocity};
use sally_math::Pose;
use sally_os::StartTime;

use crate::config::{Binding, Config, VarType, SC_GET};
use crate::context::{lock, Context};
use crate::driver::Driver;
use crate::runtime::Runtime;
use crate::ObjectId;

/// The name the map solve is loaded under (survive_startup's driver loop)
const NAME: &str = "Map_Solve";

/// driver_map_solve.c's OGStartTimeS, for its OGRelativeTime (the time a solve took). The C's is taken at the file's
/// first timed call (its first log line, as the plugin starts), this one at the first solve's: only the difference of
/// two readings is used, which is the same from either start (both readings are exact differences from it while the
/// clock reads under twice the start). The solve thread's readings are the same clock's
static START: StartTime = StartTime::new();

/// The runtime as the map solve sees it: a device is its ObjectId
pub struct MapHost<'a>(pub &'a mut Runtime);

/// &array[3 * sensor ..][..3] of a device's sensor array, for a sensor the caller checked
fn vec3(v: Option<&Vec<f64>>, sensor: usize, what: &str) -> [f64; 3] {
	match v {
		Some(v) if v.len() >= 3 * (sensor + 1) => [v[3 * sensor], v[3 * sensor + 1], v[3 * sensor + 2]],
		_ => panic!("map-solve: so->{what}[{sensor}] read out of bounds"),
	}
}

impl sally_map::Host for MapHost<'_> {
	fn run_time(&mut self) -> f64 {
		self.0.run_time()
	}
	fn relative_time(&mut self) -> f64 {
		START.relative_time()
	}

	fn active_lighthouses(&self) -> i32 {
		self.0.active_lighthouses
	}
	fn get_bsd_idx(&mut self, channel: u8) -> i8 {
		self.0.get_bsd_idx(channel)
	}
	fn bsd_pose(&self, lh: usize) -> Pose {
		self.0.bsd[lh].pose
	}
	fn bsd_position_set(&self, lh: usize) -> bool {
		self.0.bsd[lh].position_set
	}
	fn bsd_ootx_set(&self, lh: usize) -> bool {
		self.0.bsd[lh].ootx_set
	}
	fn bsd_base_station_id(&self, lh: usize) -> u32 {
		self.0.bsd[lh].base_station_id
	}
	fn bsd_mode(&self, lh: usize) -> u8 {
		self.0.bsd[lh].mode
	}
	fn bsd_fcal(&self, lh: usize) -> [BaseStationCal; 2] {
		self.0.bsd[lh].fcal
	}
	fn set_bsd_fcal(&mut self, lh: usize, fcal: &[BaseStationCal; 2]) {
		self.0.bsd[lh].fcal = *fcal;
	}
	fn bsd_sally_moved(&self, lh: usize) -> i8 {
		self.0.bsd[lh].sally_moved
	}
	fn set_bsd_sally_moved(&mut self, lh: usize, moved: i8) {
		self.0.bsd[lh].sally_moved = moved;
	}
	fn set_bsd_sally_moved_time(&mut self, lh: usize, t: f64) {
		self.0.bsd[lh].sally_moved_time = t;
	}
	fn raw_lighthouse_pose(&mut self, lh: usize, pose: &Pose) {
		self.0.raw_lighthouse_pose(lh, Some(pose));
	}

	fn sensor_ct(&self, dev: DeviceId) -> i32 {
		self.0.objects[dev].sensor_ct as i32
	}
	fn sensor_locations_set(&self, dev: DeviceId) -> bool {
		self.0.objects[dev].sensor_locations.is_some()
	}
	fn sensor_normals_set(&self, dev: DeviceId) -> bool {
		self.0.objects[dev].sensor_normals.is_some()
	}
	fn sensor_location(&self, dev: DeviceId, sensor: usize) -> [f64; 3] {
		vec3(self.0.objects[dev].sensor_locations.as_ref(), sensor, "sensor_locations")
	}
	fn set_sensor_location(&mut self, dev: DeviceId, sensor: usize, loc: &[f64; 3]) {
		match self.0.objects[dev].sensor_locations.as_mut() {
			Some(v) if v.len() >= 3 * (sensor + 1) => v[3 * sensor..3 * sensor + 3].copy_from_slice(loc),
			_ => panic!("map-solve: so->sensor_locations[{sensor}] written out of bounds"),
		}
	}
	fn sensor_normal(&self, dev: DeviceId, sensor: usize) -> [f64; 3] {
		vec3(self.0.objects[dev].sensor_normals.as_ref(), sensor, "sensor_normals")
	}
	fn timebase_hz(&self, dev: DeviceId) -> i32 {
		self.0.objects[dev].timebase_hz
	}
	fn velocity(&self, dev: DeviceId) -> Velocity {
		let v = &self.0.objects[dev].velocity;
		Velocity { pos: v.pos, axis_angle_rot: v.axis_angle_rot }
	}
	fn serial_number(&self, dev: DeviceId) -> String {
		self.0.objects[dev].serial_number.clone()
	}
	fn codename(&self, dev: DeviceId) -> String {
		self.0.objects[dev].codename.clone()
	}
	fn long_timecode_light(&self, dev: DeviceId, timecode: u32) -> u64 {
		self.0.objects[dev].activations.long_timecode_light(timecode)
	}

	fn configf(&mut self, tag: &str, def: f64) -> f64 {
		self.0.config.configf(tag, SC_GET, def)
	}
	fn configi(&mut self, tag: &str, def: u32) -> u32 {
		self.0.config.configi(tag, SC_GET, def)
	}
	fn configs(&mut self, tag: &str, def: &str) -> String {
		let v = self.0.config.configs(tag, SC_GET, Some(def.as_bytes()));
		String::from_utf8_lossy(&v.unwrap_or_else(|| def.as_bytes().to_vec())).into_owned()
	}
	fn log_level(&self) -> i32 {
		self.0.settings.log_level
	}

	fn info(&mut self, msg: &str) {
		self.0.info(msg);
	}
	fn warn(&mut self, msg: &str) {
		self.0.warn(msg);
	}
	fn record(&mut self, line: &str) {
		self.0.record_line(line);
	}
}

/// The variables map_solve_attach_config binds to the map solve's options, each with its tag, in the C's order (a
/// clone is the same variables)
#[derive(Clone)]
struct Bound(Vec<(&'static str, Binding)>);

/// map_solve_attach_config: each option given a variable of its C type, bound in the config store, which gives the
/// variable the option's value
struct Attach<'a> {
	config: &'a mut Config,
	bound: Vec<(&'static str, Binding)>,
}

impl Attach<'_> {
	fn bind(&mut self, tag: &'static str, var_type: VarType) -> Binding {
		let var = Binding::of(var_type);
		self.config.attach(tag, &var);
		self.bound.push((tag, var.clone()));
		var
	}
}

impl Source for Attach<'_> {
	fn int(&mut self, tag: &'static str) -> i32 {
		self.bind(tag, VarType::Int).int()
	}
	fn float(&mut self, tag: &'static str) -> f64 {
		self.bind(tag, VarType::Float).float()
	}
}

/// The bound variables as they are now, read in the order they were bound
struct Read<'a> {
	bound: &'a [(&'static str, Binding)],
	next: usize,
}

impl Read<'_> {
	fn var(&mut self, tag: &'static str) -> &Binding {
		let (t, var) = &self.bound[self.next];
		debug_assert_eq!(*t, tag, "map-solve: the options read in another order than they were bound");
		self.next += 1;
		var
	}
}

impl Source for Read<'_> {
	fn int(&mut self, tag: &'static str) -> i32 {
		self.var(tag).int()
	}
	fn float(&mut self, tag: &'static str) -> f64 {
		self.var(tag).float()
	}
}

impl Bound {
	/// The options' values as their variables hold them now
	fn settings(&self) -> Settings {
		Settings::read_from(&mut Read { bound: &self.0, next: 0 })
	}
}

/// DriverRegMap_Solve's driver: the map solve, and the variables bound to its options. The C's solve reads those
/// fields of its struct where it uses them, so a live set reaches its next read; inline every read is in one of its
/// hooks, so the variables are handed to it as each hook starts, and its reads see what the C's see (a solve on the
/// thread reads them as it starts)
pub struct MapSolveDriver {
	ms: Option<MapSolve>,
	bound: Bound,
	/// the bound variables' writes (`Binding::writes`) when the map solve was last handed their values: until they move,
	/// the values it has are theirs (None: not handed yet)
	bound_read: Option<u64>,
	/// ms->threaded: map-solve-threaded, off at playback factor 0
	threaded: bool,
	/// the close has begun (g_ms cleared): the hooks pass every call on
	closing: bool,
}

impl MapSolveDriver {
	/// The options bound (map_solve_attach_config), then the map solve started with them (its files, its log lines),
	/// solving inline until `start_thread`
	pub fn start(rt: &mut Runtime) -> MapSolveDriver {
		let mut attach = Attach { config: &mut rt.config, bound: Vec::new() };
		let settings = Settings::read_from(&mut attach);
		let bound = Bound(attach.bound);
		// ms->threaded = ms->threaded_opt, and off when playback-factor is 0 (the C reads it after the files and the
		// test move, which reading it does not change)
		let threaded = settings.threaded != 0 && rt.config.configf("playback-factor", SC_GET, 1.0) != 0.0;
		let ms = MapSolve::start_with(&mut MapHost(rt), settings);
		MapSolveDriver { ms: Some(ms), bound, bound_read: None, threaded, closing: false }
	}

	/// The C's solve thread ("map-solve", OGCreateThread's stack), if ms->threaded: from now on a due solve wakes it,
	/// and it reaches the map solve and the runtime with `ctx`'s lock. Called with the context locked, as the map solve
	/// starts; the thread waits for its first solve. A runtime on one thread never calls it, and solves inline
	pub fn start_thread(&mut self, ctx: Weak<Mutex<Context>>) {
		if !self.threaded {
			return;
		}
		if let Some(ms) = self.ms.as_mut() {
			ms.start_thread(ThreadLock { ctx, bound: self.bound.clone() });
		}
	}

	/// The map solve, with its options' values as they are now (read again only after a write to a bound variable: they
	/// are the values it has otherwise); None once the close has begun
	fn map_solve(&mut self) -> Option<&mut MapSolve> {
		if self.closing {
			return None;
		}
		let ms = self.ms.as_mut()?;
		// the count before the values: a write while they are read is read again next time
		let writes = Binding::writes();
		if self.bound_read != Some(writes) {
			ms.set_settings(self.bound.settings());
			self.bound_read = Some(writes);
		}
		Some(ms)
	}
}

/// The solve thread's way to the context lock (sally_map::Locked): the context, held weakly (survive_close joins the
/// thread before the context goes), where the map solve sits in its driver's slot, and the variables bound to its
/// options, which the thread reads without the lock, as the C's reads its struct's fields
struct ThreadLock {
	ctx: Weak<Mutex<Context>>,
	bound: Bound,
}

impl sally_map::Locked for ThreadLock {
	fn settings(&mut self) -> Settings {
		self.bound.settings()
	}

	fn relative_time(&mut self) -> f64 {
		START.relative_time()
	}

	/// survive_get_ctx_lock, `f` on the map solve and the runtime, survive_release_ctx_lock
	fn locked<R>(&mut self, f: impl FnOnce(&mut MapSolve, &mut dyn sally_map::Host) -> R) -> Option<R> {
		let ctx = self.ctx.upgrade()?;
		let mut g = lock(&ctx);
		g.rt.with_driver(NAME, |d: &mut MapSolveDriver, rt| d.ms.as_mut().map(|ms| f(ms, &mut MapHost(rt)))).flatten()
	}
}

impl Driver for MapSolveDriver {
	fn sweep_angle(
		&mut self, rt: &mut Runtime, obj: ObjectId, channel: u8, sensor_id: i32, timecode: u32, plane: i8, angle: f64,
	) {
		if let Some(ms) = self.map_solve() {
			ms.sweep_angle(&mut MapHost(rt), obj, channel, sensor_id, timecode, plane, angle);
		}
	}

	fn imupose(&mut self, rt: &mut Runtime, obj: ObjectId, timecode: u64, pose: &Pose) {
		if let Some(ms) = self.map_solve() {
			ms.imupose(&mut MapHost(rt), obj, timecode, pose);
		}
	}

	/// sally: an object that is another device now leaves nothing of the old one in the solve
	fn device_reset(&mut self, _rt: &mut Runtime, obj: ObjectId) {
		if let Some(ms) = self.map_solve() {
			ms.forget_device(obj);
		}
	}

	/// DriverRegMap_SolveClose's first half: g_ms cleared (the hooks pass every call on), and the solve thread, if
	/// there is one, told to stop (active cleared, the cv signalled) and handed back to be joined without the context
	/// lock (it may be in a solve that still takes it)
	fn stop(&mut self) -> Option<JoinHandle<()>> {
		self.closing = true;
		self.ms.as_mut()?.stop_thread()
	}

	/// DriverRegMap_SolveClose, the rest: its summary, the solved stations, the map file, then the options unbound
	/// (map_solve_detach_config)
	fn close(&mut self, rt: &mut Runtime) {
		if let Some(mut ms) = self.ms.take() {
			ms.set_settings(self.bound.settings());
			ms.close(&mut MapHost(rt));
			for (tag, var) in &self.bound.0 {
				rt.detach_config(tag, var);
			}
		}
	}
}
