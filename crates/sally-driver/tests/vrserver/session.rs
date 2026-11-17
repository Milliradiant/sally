//! One session in the stand-in vrserver (test_compare.py's, scripted): the driver folder laid out as SteamVR has it
//! installed (the package's manifest and settings, the library where the manifest's name puts it, sally_settings.txt
//! beside it), the library loaded and started as vrserver does, then Valve's driver and the runtime played to a
//! schedule set from the moment the frame fit is ready:
//!
//! - from the start to R + 2: Valve's driver has the first device in a driver space of its own (as it boots one), its
//!   world-from-driver putting it where it is
//! - R + 1: "status" on the command port
//! - R + 3 to R + 7: Valve's driver pauses its updates of one device 20 ms in every 100 ms (as it does while a
//!   controller's accelerometer is at full scale), loses another (not tracking for 2 s, then 1 m off with its
//!   world-from-driver moved), and from R + 3.5 goes quiet on a third for 3 s
//! - R + 8 "seen valve", R + 9 a command the driver does not know, R + 9.5 "refit" (the panel's re-align), R + 10.5
//!   "seen sally"
//! - R + 12: Valve's world moves: every device's world-from-driver, their poses in its driver space as they were
//! - R + 13: the runtime killed; it comes back (started again by the test, or by the driver), taking 3 s to come up,
//!   with two devices' configs on each other's dongles until the driver's `pair` puts them right
//! - 6 s after it is back: the end
//!
//! Every pose vrserver got, every pose fed to it and every line the panels and the command client got is recorded, with
//! the session time it came at, for the checks.

use std::ffi::{c_char, c_int, c_void, CStr, CString};
use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
use std::sync::{Mutex, RwLock};
use std::time::Duration;

use driver_sally::openvr::{DriverPose_t, HmdQuaternion_t};
use sally_math::math3d::{compose, inverse, qconj, qrot, xform, Pose, V3};

use crate::host::Server;
use crate::scene::{
	axis_angle, boot_frame, driver_from_head, lost_frame, valve_driver_from_head, valve_frame, valve_head,
	world_from_driver, world_from_driver_moved, yup, DEVICES,
};

/// What a session differs in
#[derive(Clone, Copy, Debug)]
pub struct Config {
	pub name: &'static str,
	/// the driver starts the runtime (driver.runtime_exe), starts it again when it exits, and stops it at Cleanup; or
	/// the test runs it, as one started by hand
	pub driver_starts_runtime: bool,
	/// driver.carry: the hand-over carries the pose to the time SteamVR gets it
	pub carry: bool,
}

/// The schedule, set when the frame fit is ready (session times, s)
#[derive(Clone, Copy, Debug)]
pub struct Schedule {
	pub ready: f64,
	pub status: f64,
	/// Valve's driver pauses its updates of a device 20 ms in every 100 ms (the device, from, to)
	pub pause: (usize, f64, f64),
	/// Valve's driver loses a device: not tracking for the first half, then 1 m off in a moved driver space
	pub lose: (usize, f64, f64),
	/// Valve's driver stops reporting a device
	pub quiet: (usize, f64, f64),
	pub seen_valve: f64,
	pub unknown_command: f64,
	pub refit: f64,
	pub seen_sally: f64,
	/// Valve's world moves (world_from_driver_moved from then on)
	pub world_move: f64,
	pub kill: f64,
	/// until when Valve's driver has device BOOTED in a driver space of its own (from the start)
	pub booted_until: f64,
}

/// The device Valve's driver boots in a driver space of its own (Schedule::booted_until)
pub const BOOTED: usize = 0;

impl Schedule {
	fn new(ready: f64) -> Schedule {
		let r = ready;
		Schedule {
			ready,
			status: r + 1.0,
			pause: (0, r + 3.0, r + 7.0),
			lose: (1, r + 3.0, r + 7.0),
			quiet: (2, r + 3.5, r + 6.5),
			seen_valve: r + 8.0,
			unknown_command: r + 9.0,
			refit: r + 9.5,
			seen_sally: r + 10.5,
			world_move: r + 12.0,
			kill: r + 13.0,
			booted_until: r + 2.0,
		}
	}

	pub fn paused(&self, dev: usize, t: f64) -> bool {
		let (d, a, b) = self.pause;
		d == dev && a <= t && t < b && (t - a) % 0.1 < 0.02
	}

	pub fn quiet(&self, dev: usize, t: f64) -> bool {
		let (d, a, b) = self.quiet;
		d == dev && a <= t && t < b
	}

	/// Whether Valve has lost the device at t, and if so whether it calls it valid again
	pub fn lost(&self, dev: usize, t: f64) -> Option<bool> {
		let (d, a, b) = self.lose;
		(d == dev && a <= t && t < b).then_some(t >= (a + b) / 2.0)
	}
}

/// Whether Valve's driver has the device in the driver space it boots it in, at t (before the schedule is set: from the
/// start)
pub fn booted(sch: Option<&Schedule>, dev: usize, t: f64) -> bool {
	dev == BOOTED && sch.is_none_or(|s| t < s.booted_until)
}

/// Valve's world-from-driver for the devices on its driver space at t: the scene's, moved at Schedule::world_move
pub fn wfd_at(sch: Option<&Schedule>, t: f64) -> Pose {
	if sch.is_some_and(|s| t >= s.world_move) {
		world_from_driver_moved()
	} else {
		world_from_driver()
	}
}

/// One pose fed to vrserver by another driver (Valve's, for the four devices; the headset's, a base station's and a
/// device's whose properties do not read)
#[derive(Clone, Copy, Debug)]
pub struct Fed {
	pub t: f64,
	/// the scene's device, None for the others
	pub dev: Option<usize>,
	pub pose: DriverPose_t,
	/// Valve's head position in the world: what SteamVR shows while it shows Valve's pose
	pub head_world: V3,
}

/// A runtime's run, from its marker
#[derive(Clone, Debug, Default)]
pub struct RuntimeRun {
	pub pid: u32,
	pub start: f64,
	pub serving: Option<f64>,
	pub quit: Option<f64>,
	/// when a `pair` put the devices of a restart's stale dongles right
	pub fixed: Option<f64>,
}

/// Everything a session saw
pub struct Record {
	pub config: Config,
	pub server: &'static Server,
	/// the library loaded, from the manifest's name
	pub library: PathBuf,
	/// what HmdDriverFactory said to IServerTrackedDeviceProvider_003 (null, the error); _004's provider is the one
	/// the session ran
	pub factory_other: (bool, i32),
	pub versions: Vec<String>,
	pub init: i32,
	pub fed: Vec<Fed>,
	/// (session time, line) the panel port got: V and C lines
	pub panel: Vec<(f64, String)>,
	/// (command, reply)
	pub replies: Vec<(String, String)>,
	/// the output layer's command (`output trackers recovery_blend_ms 80`): its reply's lines, and sally_output.txt after
	/// it
	pub output_reply: Vec<String>,
	pub output_file: String,
	pub schedule: Option<Schedule>,
	/// when the runtime that came back after the kill was up
	pub back: Option<f64>,
	pub end: f64,
	pub runs: Vec<RuntimeRun>,
	/// each run's console output
	pub consoles: Vec<String>,
	/// what the hooked slots hold after Cleanup, and another driver's hook (the session where one hooks after us)
	pub slots_after: [usize; 3],
	pub other_hook: Option<usize>,
	/// a pose through the hooks after Cleanup: (sent, what vrserver got)
	pub after_cleanup: Option<(DriverPose_t, Option<DriverPose_t>)>,
	pub pose_log: Option<String>,
	/// runtimes still running after the session
	pub left_over: Vec<u32>,
}

extern "C" {
	fn dlopen(file: *const c_char, flags: c_int) -> *mut c_void;
	fn dlsym(lib: *mut c_void, name: *const c_char) -> *mut c_void;
	fn dlerror() -> *const c_char;
	fn kill(pid: i32, sig: c_int) -> c_int;
	fn fflush(f: *mut c_void) -> c_int;
}
const RTLD_NOW: c_int = 2;
const SIGKILL: c_int = 9;

type Factory = unsafe extern "C" fn(*const c_char, *mut i32) -> *mut c_void;

/// The provider the factory handed out, called through its vtable (IServerTrackedDeviceProvider: Init, Cleanup,
/// GetInterfaceVersions, RunFrame, ShouldBlockStandbyMode, EnterStandby, LeaveStandby)
#[derive(Clone, Copy)]
pub struct Provider(pub *mut c_void);
// SAFETY: vrserver calls a provider from its own threads; the driver's are made for that
unsafe impl Send for Provider {}
unsafe impl Sync for Provider {}

type InitFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32;
type VoidFn = unsafe extern "C" fn(*mut c_void);
type VersionsFn = unsafe extern "C" fn(*mut c_void) -> *const *const c_char;

impl Provider {
	fn slot(&self, k: usize) -> *const c_void {
		// SAFETY: the provider object starts with its vtable, of 7 slots
		unsafe { (*(self.0 as *const *const *const c_void)).add(k).read() }
	}

	pub fn init(&self, ctx: *mut c_void) -> i32 {
		// SAFETY: slot 0 is Init(this, IVRDriverContext *)
		unsafe { std::mem::transmute::<*const c_void, InitFn>(self.slot(0))(self.0, ctx) }
	}

	pub fn cleanup(&self) {
		// SAFETY: slot 1 is Cleanup(this)
		unsafe { std::mem::transmute::<*const c_void, VoidFn>(self.slot(1))(self.0) }
	}

	fn interface_versions(&self) -> Vec<String> {
		// SAFETY: slot 2 is GetInterfaceVersions(this): a null-ended array of NUL-terminated strings
		unsafe {
			let mut p = std::mem::transmute::<*const c_void, VersionsFn>(self.slot(2))(self.0);
			let mut out = Vec::new();
			while !(*p).is_null() {
				out.push(CStr::from_ptr(*p).to_string_lossy().into_owned());
				p = p.add(1);
			}
			out
		}
	}

	pub fn run_frame(&self) {
		// SAFETY: slot 3 is RunFrame(this)
		unsafe { std::mem::transmute::<*const c_void, VoidFn>(self.slot(3))(self.0) }
	}
}

/// The session folder (target/tmp/vrserver-<name>, emptied), and the driver folder in it as SteamVR has it installed:
/// the package's manifest and settings, the library where the manifest's name puts it (the one cargo built beside
/// this test, or VRSERVER_DRIVER's). The folder, the driver folder, the library
pub fn install(name: &str, exe: &Path) -> (PathBuf, PathBuf, PathBuf) {
	let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("vrserver-{name}"));
	let _ = std::fs::remove_dir_all(&dir);
	let drv = dir.join("sally");
	let package = Path::new(env!("CARGO_MANIFEST_DIR")).join("package");
	std::fs::create_dir_all(drv.join("resources/settings")).unwrap();
	std::fs::create_dir_all(drv.join("bin/linux64")).unwrap();
	std::fs::copy(package.join("driver.vrdrivermanifest"), drv.join("driver.vrdrivermanifest")).unwrap();
	std::fs::copy(
		package.join("resources/settings/default.vrsettings"),
		drv.join("resources/settings/default.vrsettings"),
	)
	.unwrap();
	// vrserver loads bin/<platform>/driver_<the manifest's name>
	let manifest = std::fs::read_to_string(drv.join("driver.vrdrivermanifest")).unwrap();
	let lib_name = manifest
		.split_once("\"name\"")
		.and_then(|(_, r)| r.split('"').nth(1))
		.expect("the manifest's name")
		.to_string();
	let library = drv.join(format!("bin/linux64/driver_{lib_name}.so"));
	// the library cargo built beside this test, or another build of the driver (VRSERVER_DRIVER: the C++ driver built
	// from git, say)
	let built = std::env::var_os("VRSERVER_DRIVER")
		.map(PathBuf::from)
		.unwrap_or_else(|| exe.parent().unwrap().join("libdriver_sally.so"));
	std::fs::copy(&built, &library).unwrap_or_else(|e| panic!("the driver to load, {}: {e}", built.display()));
	(dir, drv, library)
}

/// The driver, loaded as vrserver loads it (dlopen, HmdDriverFactory): its IServerTrackedDeviceProvider_004, and
/// what the factory said to _003 (null, the error)
pub fn open(library: &Path) -> (Provider, (bool, i32)) {
	let lib = CString::new(library.to_string_lossy().as_bytes()).unwrap();
	// SAFETY: dlopen and dlsym on the driver's library, the names NUL-terminated
	let factory: Factory = unsafe {
		let h = dlopen(lib.as_ptr(), RTLD_NOW);
		assert!(!h.is_null(), "dlopen: {}", CStr::from_ptr(dlerror()).to_string_lossy());
		let f = dlsym(h, c"HmdDriverFactory".as_ptr());
		assert!(!f.is_null(), "no HmdDriverFactory");
		std::mem::transmute::<*mut c_void, Factory>(f)
	};
	let mut rc = 0;
	// SAFETY: the factory takes an interface name and a place for the error
	let other = unsafe { factory(c"IServerTrackedDeviceProvider_003".as_ptr(), &mut rc) };
	// SAFETY: as above
	let p = unsafe { factory(c"IServerTrackedDeviceProvider_004".as_ptr(), std::ptr::null_mut()) };
	assert!(!p.is_null(), "the factory has no IServerTrackedDeviceProvider_004");
	(Provider(p), (other.is_null(), rc))
}

/// A UDP port nothing listens on now
pub fn free_port() -> u16 {
	UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

pub fn sleep_until(now: &dyn Fn() -> f64, t: f64) {
	let dt = t - now();
	if dt > 0.0 {
		std::thread::sleep(Duration::from_secs_f64(dt));
	}
}

/// The runs' markers in the session folder
fn runs(dir: &Path) -> Vec<RuntimeRun> {
	let mut out: Vec<RuntimeRun> = Vec::new();
	for e in std::fs::read_dir(dir).unwrap().flatten() {
		let name = e.file_name().to_string_lossy().into_owned();
		let Some(pid) = name.strip_prefix("runtime-").and_then(|s| s.strip_suffix(".txt")).and_then(|s| s.parse().ok())
		else {
			continue;
		};
		let mut run = RuntimeRun { pid, ..Default::default() };
		for line in std::fs::read_to_string(e.path()).unwrap_or_default().lines() {
			let (what, t) = line.split_once(' ').unwrap_or((line, ""));
			let t: f64 = t.parse().unwrap_or(f64::NAN);
			match what {
				"start" => run.start = t,
				"serving" => run.serving = Some(t),
				"quit" => run.quit = Some(t),
				"fixed" => run.fixed = Some(t),
				_ => {}
			}
		}
		out.push(run);
	}
	out.sort_by(|a, b| a.start.total_cmp(&b.start));
	out
}

/// splitmix64: Valve's update times' jitter
struct Rng(u64);
impl Rng {
	fn uniform(&mut self) -> f64 {
		self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
		let mut z = self.0;
		z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
		z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
		((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
	}
}

fn hmd_q(q: sally_math::math3d::Q4) -> HmdQuaternion_t {
	HmdQuaternion_t { w: q.w, x: q.x, y: q.y, z: q.z }
}

/// Valve's driver pose for a head pose with driver-from-head `h`: D = head * h^-1 (so that D * h = head), its
/// world-from-driver and velocity
fn driver_pose(head: &Pose, wfd: &Pose, h: &Pose, vel: V3, valid: bool) -> DriverPose_t {
	let d = compose(head, &inverse(h));
	DriverPose_t {
		poseTimeOffset: 0.0,
		qWorldFromDriverRotation: hmd_q(wfd.q),
		vecWorldFromDriverTranslation: [wfd.p.x, wfd.p.y, wfd.p.z],
		qDriverFromHeadRotation: hmd_q(h.q),
		vecDriverFromHeadTranslation: [h.p.x, h.p.y, h.p.z],
		vecPosition: [d.p.x, d.p.y, d.p.z],
		vecVelocity: [vel.x, vel.y, vel.z],
		qRotation: hmd_q(d.q),
		result: if valid { 200 } else { 201 }, // TrackingResult_Running_OK, _OutOfRange
		poseIsValid: valid as u8,
		deviceIsConnected: 1,
		..DriverPose_t::default()
	}
}

/// The other drivers' devices vrserver has: the headset (another tracking system, through _005), a base station
/// (Valve's, not a tracker), a device whose properties do not read; each (id, interface, period)
pub const OTHERS: [(u32, usize, f64); 3] = [(0, 1, 1.0 / 90.0), (5, 0, 0.1), (6, 0, 0.01)];

/// Valve's driver and the others: each of the scene's devices every 4 ms (jittered), the others at their periods,
/// through the server's vtables, while `running`
pub fn other_drivers(
	server: &'static Server, schedule: &RwLock<Option<Schedule>>, fed: &Mutex<Vec<Fed>>, running: &AtomicBool,
) {
	let now = || server.now();
	let mut rng = Rng(0x5a11_7e57);
	let t = now();
	let mut next: Vec<f64> = (0..DEVICES.len()).map(|i| t + 0.001 * i as f64).collect();
	let mut next_other: Vec<f64> = OTHERS.iter().map(|_| t).collect();
	let (k, l, b) = (valve_frame(), lost_frame(), boot_frame());
	// a pose fed: recorded, then handed to vrserver through the interface's vtable
	let feed = |f: Fed, iface: usize, id: u32| {
		let index = {
			let mut fed = fed.lock().unwrap();
			fed.push(f);
			fed.len() - 1
		};
		server.report(iface, id, &f.pose, Some(index));
	};
	while running.load(SeqCst) {
		let t = now();
		let sch = *schedule.read().unwrap();
		for i in 0..DEVICES.len() {
			if t < next[i] {
				continue;
			}
			next[i] += 0.004 + 0.0006 * (rng.uniform() - 0.5);
			if sch.is_some_and(|s| s.paused(i, t) || s.quiet(i, t)) {
				continue;
			}
			let head = valve_head(i, t);
			let vel = qrot(k.q, yup(DEVICES[i].at(t).v));
			let w = wfd_at(sch.as_ref(), t);
			// lost, or not yet out of the driver space it was booted in: the same world pose written in another
			// driver space (its world-from-driver moved with it)
			let (head_d, wfd, vel, valid) = match sch.and_then(|s| s.lost(i, t)) {
				Some(valid) => (compose(&inverse(&l), &head), compose(&w, &l), qrot(qconj(l.q), vel), valid),
				None if booted(sch.as_ref(), i, t) => {
					(compose(&inverse(&b), &head), compose(&w, &b), qrot(qconj(b.q), vel), true)
				}
				None => (head, w, vel, true),
			};
			let pose = driver_pose(&head_d, &wfd, &driver_from_head(i), vel, valid);
			feed(Fed { t, dev: Some(i), pose, head_world: xform(&w, head.p) }, 0, DEVICES[i].id);
		}
		for (j, &(id, iface, period)) in OTHERS.iter().enumerate() {
			if t < next_other[j] {
				continue;
			}
			next_other[j] += period;
			// the base station stands still; the others move a little
			let head = match id {
				5 => Pose { p: V3::new(2.0, 2.4, -1.5), q: axis_angle(V3::new(0.0, 1.0, 0.0), 2.3) },
				_ => Pose {
					p: V3::new(0.1 * (0.7 * t).sin(), 1.6 + 0.02 * (1.3 * t).sin(), 0.1 * (0.5 * t).cos()),
					q: axis_angle(V3::new(0.0, 1.0, 0.0), 0.3 * (0.4 * t).sin()),
				},
			};
			let pose = driver_pose(&head, &Pose::default(), &valve_driver_from_head(), V3::default(), true);
			feed(Fed { t, dev: None, pose, head_world: head.p }, iface, id);
		}
		let wake = next.iter().chain(&next_other).copied().fold(f64::INFINITY, f64::min);
		sleep_until(&now, wake.min(now() + 0.002));
	}
}

/// A command whose reply is several lines (the output layer's O lines): every datagram until the status line
pub fn command_lines(sock: &UdpSocket, port: u16, c: &str) -> Vec<String> {
	sock.send_to(c.as_bytes(), ("127.0.0.1", port)).unwrap();
	let mut out = Vec::new();
	let mut buf = [0u8; 512];
	while let Ok(n) = sock.recv(&mut buf) {
		let l = String::from_utf8_lossy(&buf[..n]).trim_end().to_string();
		let status = l.starts_with("C ");
		out.push(l);
		if status {
			break;
		}
	}
	out
}

pub fn command(sock: &UdpSocket, port: u16, c: &str) -> String {
	sock.send_to(c.as_bytes(), ("127.0.0.1", port)).unwrap();
	let mut buf = [0u8; 512];
	match sock.recv(&mut buf) {
		Ok(n) => String::from_utf8_lossy(&buf[..n]).trim_end().to_string(),
		Err(_) => String::new(),
	}
}

/// The runtime as the test starts it (as by hand), its console to a file of its own: the process and that file
fn start_runtime(exe: &Path, dir: &Path, t0: i64, port: u16, n: usize) -> (Child, PathBuf) {
	let path = dir.join(format!("console-{n}.txt"));
	let console = std::fs::File::create(&path).unwrap();
	let child = Command::new(exe)
		.args(["--scripted-runtime", &dir.to_string_lossy(), &t0.to_string(), "--runtime-port", &port.to_string()])
		.current_dir(dir)
		.stdin(Stdio::null())
		.stdout(console)
		.stderr(Stdio::null())
		.spawn()
		.expect("the scripted runtime");
	(child, path)
}

/// The moment the frame fit is ready, from the driver's log (up to 60 s), and the schedule from it, set for the other
/// drivers
fn play(server: &Server, schedule: &RwLock<Option<Schedule>>) -> Option<Schedule> {
	let ready = || server.log.lock().unwrap().iter().find(|l| l.1.contains("frame fit ready")).map(|l| l.0);
	if !wait_for(60.0, || ready().is_some()) {
		return None;
	}
	let plan = Schedule::new(ready()?);
	*schedule.write().unwrap() = Some(plan);
	Some(plan)
}

/// Until `f` holds or `timeout` s have passed: whether it held
pub fn wait_for(timeout: f64, f: impl Fn() -> bool) -> bool {
	let end = std::time::Instant::now() + Duration::from_secs_f64(timeout);
	while std::time::Instant::now() < end {
		if f() {
			return true;
		}
		std::thread::sleep(Duration::from_millis(20));
	}
	f()
}

pub fn run(config: Config) -> Record {
	let t0 = sally_os::chrono::steady_clock_now();
	let server = Server::new(t0);
	let now = || server.now();
	let exe = std::env::current_exe().unwrap();

	let (dir, drv, library) = install(config.name, &exe);

	// the ports: the runtime's, the command port the driver binds, the panel port the test listens on
	let rt_port = free_port();
	let cmd_port = free_port();
	let panel = UdpSocket::bind("127.0.0.1:0").unwrap();
	panel.set_read_timeout(Some(Duration::from_millis(100))).unwrap();
	let panel_port = panel.local_addr().unwrap().port();
	// sally_settings.txt: test_compare.py's (the ports, lost_timeout_ms, the pose log; the pose's age is the output
	// layer's default, the live 9 ms); the carry
	// and the runtime's start where the session has them
	let mut settings = vec![
		"# the vrserver test's".to_string(),
		format!("driver.runtime_port={rt_port}"),
		"driver.lost_timeout_ms=150".into(),
		format!("driver.valve_udp={panel_port}"),
		format!("driver.cmd_port={cmd_port}"),
		format!("driver.pose_log={}", dir.join("poses_{time}.csv").display()),
	];
	if config.carry {
		settings.push("driver.carry=1".into());
	}
	if config.driver_starts_runtime {
		settings.push(format!("driver.runtime_exe={}", exe.display()));
		settings.push(format!("driver.runtime_cwd={}", dir.display()));
		settings.push(format!("driver.runtime_log={}", dir.join("runtime_{time}.log").display()));
		// its arguments, one a line (the driver adds --runtime-port)
		settings.extend([
			"# its arguments".into(),
			"--scripted-runtime".into(),
			dir.display().to_string(),
			t0.to_string(),
		]);
	}
	std::fs::write(drv.join("bin/linux64/sally_settings.txt"), settings.join("\n") + "\n").unwrap();

	// the vrserver's devices: Valve's four (each on a dongle of its own), the headset, a base station; the sixth's
	// properties do not read
	{
		let mut props = server.props.lock().unwrap();
		let mut dongles = server.dongles.lock().unwrap();
		for d in &DEVICES {
			props.insert(d.id, ("lighthouse", d.serial));
			dongles.insert(d.id, d.dongle);
		}
		props.insert(0, ("oculus", "1WMHH000X00000"));
		props.insert(5, ("lighthouse", "LHB-1A2B3C4D"));
	}

	// a runtime started by hand comes first
	let mut child = None;
	if !config.driver_starts_runtime {
		child = Some(start_runtime(&exe, &dir, t0, rt_port, 0));
		assert!(wait_for(5.0, || runs(&dir).iter().any(|r| r.serving.is_some())), "the runtime came up");
	}

	let (provider, factory_other) = open(&library);
	let versions = provider.interface_versions();
	let init = provider.init(server.context());

	// vrserver at work: the frames, the other drivers, the panel port's listener; the schedule from the fit on
	let (running, listening) = (AtomicBool::new(true), AtomicBool::new(true));
	let schedule: RwLock<Option<Schedule>> = RwLock::new(None);
	let fed: Mutex<Vec<Fed>> = Mutex::new(Vec::new());
	let lines: Mutex<Vec<(f64, String)>> = Mutex::new(Vec::new());
	let client = UdpSocket::bind("127.0.0.1:0").unwrap();
	client.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
	let mut replies = Vec::new();
	let mut output_reply = Vec::new();
	let mut back = None;
	std::thread::scope(|s| {
		s.spawn(|| {
			let mut buf = [0u8; 4096];
			while listening.load(SeqCst) {
				if let Ok(n) = panel.recv(&mut buf) {
					lines.lock().unwrap().push((now(), String::from_utf8_lossy(&buf[..n]).trim_end().to_string()));
				}
			}
		});
		let frames = s.spawn(|| {
			while running.load(SeqCst) {
				provider.run_frame();
				std::thread::sleep(Duration::from_micros(11111));
			}
		});
		let others = s.spawn(|| other_drivers(server, &schedule, &fed, &running));
		if let Some(plan) = play(server, &schedule) {
			for (t, c) in [
				(plan.status, "status"),
				(plan.seen_valve, "seen valve"),
				(plan.unknown_command, "garbage"),
				(plan.refit, "refit"),
				(plan.seen_sally, "seen sally"),
			] {
				sleep_until(&now, t);
				replies.push((c.to_string(), command(&client, cmd_port, c)));
			}
			output_reply = command_lines(&client, cmd_port, "output trackers recovery_blend_ms 80");
			// a toy switched on (the headset's, which the toys leave alone: the poses stay as they are), to be off
			// again after Cleanup
			command_lines(&client, cmd_port, "output headset rail 1");
			sleep_until(&now, plan.kill);
			match child.as_mut() {
				Some((c, _)) => {
					c.kill().unwrap();
					c.wait().unwrap();
					child = Some(start_runtime(&exe, &dir, t0, rt_port, 1));
				}
				None => {
					let run = runs(&dir).into_iter().next_back().expect("the runtime the driver started");
					// SAFETY: a signal to the runtime process
					unsafe { kill(run.pid as i32, SIGKILL) };
				}
			}
			let came_back = || runs(&dir).iter().filter_map(|r| r.serving).find(|&t| t > plan.kill);
			if wait_for(15.0, || came_back().is_some()) {
				back = came_back();
				sleep_until(&now, back.unwrap() + 6.0);
			}
		}
		running.store(false, SeqCst);
		others.join().unwrap();
		frames.join().unwrap();
		listening.store(false, SeqCst);
	});
	let end = now();

	// another driver hooks _006 after us (the session the driver does not start the runtime in), then Cleanup
	let other_hook = (!config.driver_starts_runtime).then(|| server.other_driver_hooks());
	provider.cleanup();
	let slots_after = [server.slot_value(0), server.slot_value(1), server.slot_value(2)];
	// after Cleanup, a pose through _006's hooks: ours passes it through
	let after_cleanup = other_hook.map(|_| {
		let pose = driver_pose(&valve_head(3, end), &world_from_driver(), &driver_from_head(3), V3::default(), true);
		let before = server.calls.lock().unwrap().len();
		server.report(0, DEVICES[3].id, &pose, None);
		let calls = server.calls.lock().unwrap();
		(pose, calls.get(before).map(|c| c.pose))
	});
	// the pose log is the C library's stream, never closed: what it holds goes out as at the process's exit
	// SAFETY: fflush(NULL) flushes every output stream
	unsafe { fflush(std::ptr::null_mut()) };
	let pose_log = std::fs::read_dir(&dir)
		.unwrap()
		.flatten()
		.find(|e| e.file_name().to_string_lossy().starts_with("poses_"))
		.and_then(|e| std::fs::read_to_string(e.path()).ok());
	// the runtime started by hand stays until it has read what Cleanup sent it (it unsubscribes), then goes
	if let Some((mut c, console)) = child {
		wait_for(2.0, || std::fs::read_to_string(&console).is_ok_and(|t| t.contains("got unsub")));
		c.kill().unwrap();
		c.wait().unwrap();
	}
	// the driver's log as vrserver's would have it, kept in the folder
	let text: String = server.log.lock().unwrap().iter().map(|(t, l)| format!("{t:10.4} {l}\n")).collect();
	std::fs::write(dir.join("vrserver.txt"), text).unwrap();
	let runs = runs(&dir);
	let left_over = runs.iter().filter(|r| Path::new(&format!("/proc/{}", r.pid)).exists()).map(|r| r.pid).collect();
	let mut consoles: Vec<(String, String)> = std::fs::read_dir(&dir)
		.unwrap()
		.flatten()
		.filter_map(|e| {
			let name = e.file_name().to_string_lossy().into_owned();
			(name.starts_with("console-") || name.starts_with("runtime_"))
				.then(|| (name, std::fs::read_to_string(e.path()).unwrap_or_default()))
		})
		.collect();
	consoles.sort();
	Record {
		config,
		server,
		library,
		factory_other,
		versions,
		init,
		fed: fed.into_inner().unwrap(),
		panel: lines.into_inner().unwrap(),
		replies,
		output_reply,
		output_file: std::fs::read_to_string(drv.join("bin/linux64/sally_output.txt")).unwrap_or_default(),
		schedule: schedule.into_inner().unwrap(),
		back,
		end,
		runs,
		consoles: consoles.into_iter().map(|c| c.1).collect(),
		slots_after,
		other_hook,
		after_cleanup,
		pose_log,
		left_over,
	}
}
