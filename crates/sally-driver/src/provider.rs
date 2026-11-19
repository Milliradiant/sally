//! The provider vrserver loads (Provider: IServerTrackedDeviceProvider): Init reads the settings, opens the socket to
//! the runtime (starting one if none answers and runtime_exe is set), starts the override and the receive thread;
//! RunFrame ticks the override and notices lost devices; Cleanup stops it all, the runtime it started included.
//!
//! The receive thread subscribes to the runtime's `driver` stream (docs/sally_runtime_protocol.md), re-subscribing and
//! asking for the device list every second, and turns its lines into RuntimeDevice updates. With a runtime of protocol 5
//! on, it also tells it every second (and at once when one changes) which device each dongle carries, as Valve's
//! driver says (`pair`, PoseOverride::pairs).

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::c_char;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicI64, AtomicU64, Ordering::SeqCst};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;

use sally_math::printf::BArg;
use sally_math::scanf::{sscanf, Value};
use sally_math::{c_atof, c_atoi};

use crate::helpers::{dll_dir, lines, log, now_seconds, split};
use crate::net::{self, SockAddrIn, SOCKADDR_IN_LEN, SOCK_BAD};
use crate::openvr::*;
use crate::pose_override::G_OVERRIDE;
use crate::runtime_device::{Kind, Report, RuntimeDevice};
use crate::runtime_process::RuntimeProcess;
use crate::settings::{g_opt, read_settings, set_g_opt};

pub const SALLY_DRIVER_VERSION: &str = env!("CARGO_PKG_VERSION");
pub const SALLY_MIN_PROTOCOL: i32 = 2;
/// The runtime protocol with `pair`
pub const SALLY_PAIR_PROTOCOL: i32 = 5;

/// The devices the runtime tracks, by serial, and its codenames for them (under the C++'s lock_)
#[derive(Default)]
pub struct Devices {
	pub devices: BTreeMap<Vec<u8>, Arc<RuntimeDevice>>,
	/// runtime codename -> serial
	pub codenames: BTreeMap<Vec<u8>, Vec<u8>>,
}

/// The runtime process and its timers (the receive thread's, and Init's before it runs)
struct Process {
	proc: RuntimeProcess,
	last_start: f64,
	init_t: f64,
	/// the starts in a row that did not hold: the runtime gone again within RUN_HELD_S, or not started at all
	short_runs: u32,
	/// ours has exited and is to be started again once its wait has passed
	down: bool,
}

/// How long Init waits for a runtime that is already up to answer (one started by hand): two hellos' worth. A runtime
/// answers within a poll, 10 ms
const PROBE_S: f64 = 0.3;
/// A runtime that has exited is started again this long after its last start at the earliest (s)
const RESTART_WAIT_S: f64 = 5.0;
/// and, when the starts before it did not hold either, twice as long each time, up to this
const RESTART_WAIT_MAX_S: f64 = 300.0;
/// A start has held when the runtime ran this long (s): the next exit is started again at once
const RUN_HELD_S: f64 = 60.0;

/// How long after its last start the runtime may be started again: RESTART_WAIT_S, doubled for every start in a row
/// before it that did not hold, up to RESTART_WAIT_MAX_S. A runtime that cannot come up (its port taken, its program
/// damaged) is so not started, with a new log and recording each time, every 5 s for as long as SteamVR runs
fn restart_wait(short_runs: u32) -> f64 {
	(RESTART_WAIT_S * 2f64.powi(short_runs.saturating_sub(1).min(16) as i32)).min(RESTART_WAIT_MAX_S)
}

pub struct Provider {
	dir: Mutex<Vec<u8>>,
	process: Mutex<Process>,
	sock: AtomicI64,
	runtime: Mutex<SockAddrIn>,
	running: AtomicBool,
	hello_ok: AtomicBool,
	/// the protocol the runtime's hello said
	proto: AtomicI32,
	proto_warned: AtomicBool,
	/// last_heard_, a double's bits
	last_heard: AtomicU64,
	thread: Mutex<Option<JoinHandle<()>>>,
	lock: Mutex<Devices>,
	skipped: Mutex<BTreeSet<Vec<u8>>>,
	/// per serial: the runtime's lines with a value that is not a number, dropped (how many, when last logged)
	not_finite: Mutex<BTreeMap<Vec<u8>, (u64, f64)>>,
}

/// The provider (g_provider)
pub static G_PROVIDER: Provider = Provider::new();

/// A lock taken (a panic while it was held aborts the process, so a poisoned lock never reaches here)
fn locked<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
	m.lock().unwrap_or_else(|e| e.into_inner())
}

impl Provider {
	const fn new() -> Provider {
		Provider {
			dir: Mutex::new(Vec::new()),
			process: Mutex::new(Process {
				proc: RuntimeProcess::NONE,
				last_start: 0.0,
				init_t: 0.0,
				short_runs: 0,
				down: false,
			}),
			sock: AtomicI64::new(SOCK_BAD),
			runtime: Mutex::new(SockAddrIn { sin_family: 0, sin_port: 0, sin_addr: 0, sin_zero: [0; 8] }),
			running: AtomicBool::new(false),
			hello_ok: AtomicBool::new(false),
			proto: AtomicI32::new(0),
			proto_warned: AtomicBool::new(false),
			last_heard: AtomicU64::new(0),
			thread: Mutex::new(None),
			lock: Mutex::new(Devices { devices: BTreeMap::new(), codenames: BTreeMap::new() }),
			skipped: Mutex::new(BTreeSet::new()),
			not_finite: Mutex::new(BTreeMap::new()),
		}
	}

	/// A runtime line with a value that is not a number (a filter gone bad prints "nan"): dropped, before it reaches
	/// SteamVR or the frame fit, where one NaN pair keeps the fit from being taken for its whole window; logged at
	/// most every 10 s per device
	fn drop_not_finite(&self, serial: &[u8], what: &str) {
		let now = now_seconds();
		let mut m = locked(&self.not_finite);
		let e = m.entry(serial.to_vec()).or_insert((0, -1e9));
		e.0 += 1;
		if now - e.1 >= 10.0 {
			let n = e.0;
			e.1 = now;
			drop(m);
			log(
				"sally: %s: %s with a value that is not a number, dropped (%llu so far)\n",
				&[BArg::S(serial), BArg::S(what.as_bytes()), BArg::U(n)],
			);
		}
	}

	fn last_heard(&self) -> f64 {
		f64::from_bits(self.last_heard.load(SeqCst))
	}

	/// The devices (the tests read them)
	pub fn devices(&self) -> MutexGuard<'_, Devices> {
		locked(&self.lock)
	}

	pub fn init(&'static self, ctx: *mut IVRDriverContext) -> EVRInitError {
		// VR_INIT_SERVER_DRIVER_CONTEXT(ctx)
		let e = CONTEXT.init_server(ctx);
		if e != VRInitError_None {
			return e;
		}
		let dir = dll_dir();
		*locked(&self.dir) = dir.clone();
		log("sally: driver %s starting from %s\n", &[BArg::S(SALLY_DRIVER_VERSION.as_bytes()), BArg::S(&dir)]);
		let mut o = (*g_opt()).clone();
		read_settings(&mut o, &dir);
		// the panel's choices over the settings' (user.rs)
		if let Some(keep) = crate::user::read(&dir).keep_sessions {
			o.keep_sessions = keep;
		}
		set_g_opt(o);
		crate::output::read_output(&dir);
		let o = g_opt();
		// this session noted, and the files of all but the newest few before it deleted
		let gone = crate::sessions::start(&o, &dir, &crate::helpers::time_stamp());
		if o.keep_sessions > 0 {
			log(
				"sally: the files of the newest %d sessions kept, %d older files deleted\n",
				&[BArg::I(o.keep_sessions as i64), BArg::I(gone as i64)],
			);
		}

		net::wsa_startup();
		let sock = net::socket_udp();
		self.sock.store(sock, SeqCst);
		if sock == SOCK_BAD {
			log("sally: cannot create a UDP socket\n", &[]);
			return VRInitError_Driver_Failed;
		}
		net::bind(sock, &SockAddrIn::loopback(0));
		net::set_rcvbuf(sock, 1 << 20);
		net::set_recv_timeout_50ms(sock);
		{
			let mut rt = locked(&self.runtime);
			*rt = SockAddrIn::default();
			rt.sin_family = net::AF_INET;
			rt.sin_port = net::htons(o.runtime_port as u16);
			net::inet_pton(&o.runtime_host, &mut rt.sin_addr);
		}

		log(
			"sally: runtime at %s:%d, velocity %d, angular velocity %d, lever arm %d, time offset %d\n",
			&[
				BArg::S(&o.runtime_host),
				BArg::I(o.runtime_port as i64),
				BArg::I(o.velocity as i64),
				BArg::I(o.angular_velocity as i64),
				BArg::I(o.lever_arm as i64),
				BArg::I(o.time_offset as i64),
			],
		);

		// a runtime that already answers (one started by hand) is used as it is; otherwise the driver starts one.
		// SteamVR's start does not wait for it to come up: the receive thread asks it every second and takes its
		// answer, and until then SteamVR shows Valve's tracking
		if !o.runtime_exe.is_empty() {
			if !self.handshake(PROBE_S) && !self.start_runtime() {
				log("sally: no runtime; Valve's tracking until one answers\n", &[]);
			}
		} else {
			log("sally: driver.runtime_exe not set: the runtime is started separately\n", &[]);
		}
		G_OVERRIDE.start(ctx);
		crate::panel::start();
		self.running.store(true, SeqCst);
		*locked(&self.thread) = Some(std::thread::spawn(|| G_PROVIDER.receive_thread()));
		VRInitError_None
	}

	pub fn cleanup(&self) {
		self.running.store(false, SeqCst);
		if let Some(t) = locked(&self.thread).take() {
			let _ = t.join();
		}
		// a toy is for the session it was switched on in (output.rs); what else the panel set is in the file
		crate::output::toys_off("SteamVR closes");
		crate::output::save_due(None);
		{
			let mut p = locked(&self.process);
			if p.proc.started() && g_opt().runtime_stop {
				log("sally: stopping the runtime\n", &[]);
				self.send(b"quit");
				p.proc.wait_or_kill(8.0); // the runtime closes its recordings on the way out
			}
		}
		let sock = self.sock.load(SeqCst);
		if sock != SOCK_BAD {
			self.send(b"unsub");
			net::close(sock);
			self.sock.store(SOCK_BAD, SeqCst);
		}
		G_OVERRIDE.stop();
		let mut l = locked(&self.lock);
		l.devices.clear();
		// VR_CLEANUP_SERVER_DRIVER_CONTEXT()
		CONTEXT.cleanup();
		drop(l);
	}

	pub fn run_frame(&self) {
		let now = now_seconds();
		G_OVERRIDE.tick(now);
		crate::output::save_due(Some(now));
		// the devices' list let go before the checks: a check that finds a device lost pushes its pose into vrserver
		let devices: Vec<Arc<RuntimeDevice>> = locked(&self.lock).devices.values().cloned().collect();
		for d in devices {
			d.check_lost(now);
		}
	}

	/// Send(cmd): one line to the runtime
	fn send(&self, cmd: &[u8]) {
		let sock = self.sock.load(SeqCst);
		if sock == SOCK_BAD {
			return;
		}
		let mut line = cmd.to_vec();
		line.push(b'\n');
		let rt = *locked(&self.runtime);
		net::sendto(sock, &line, &rt, SOCKADDR_IN_LEN);
	}

	/// SendPairs: which device each dongle carries, as Valve's driver says, to a runtime that takes them
	fn send_pairs(&self, now: f64) {
		if !self.hello_ok.load(SeqCst) || self.proto.load(SeqCst) < SALLY_PAIR_PROTOCOL {
			return;
		}
		for line in G_OVERRIDE.pairs(now) {
			self.send(&line);
		}
	}

	/// Handshake(seconds): up to `seconds` for a runtime to answer "#h hello" (before the receive thread runs)
	fn handshake(&self, seconds: f64) -> bool {
		let end = now_seconds() + seconds;
		let mut last = 0.0;
		let mut buf = vec![0u8; 65536];
		while now_seconds() < end {
			if now_seconds() - last > 0.1 {
				self.send(b"#h hello");
				last = now_seconds();
			}
			let n = net::recv(self.sock.load(SeqCst), &mut buf[..65535]);
			if n <= 0 {
				continue;
			}
			buf[n as usize] = 0;
			for line in lines(&buf) {
				self.handle_line(line);
			}
			if self.hello_ok.load(SeqCst) {
				return true;
			}
		}
		false
	}

	/// The runtime started (a start that fails counts as one that did not hold: restart_wait)
	fn start_runtime_locked(&self, p: &mut Process) -> bool {
		p.last_start = now_seconds();
		p.down = false;
		let dir = locked(&self.dir).clone();
		let started = p.proc.start(&dir);
		if !started {
			p.short_runs = p.short_runs.saturating_add(1);
		}
		started
	}

	fn start_runtime(&self) -> bool {
		let mut p = locked(&self.process);
		self.start_runtime_locked(&mut p)
	}

	/// ManageRuntimeProcess(now): our runtime exited: start it again, restart_wait after its last start at the
	/// earliest (at once after a run that held). A runtime someone else started is left alone; if none has been
	/// heard for 10 s and we have none, start ours (the driver keeps one running).
	fn manage_runtime_process(&self, now: f64) {
		let o = g_opt();
		let mut p = locked(&self.process);
		if o.runtime_exe.is_empty() || !o.runtime_restart {
			return;
		}
		let mut code = 0;
		if p.proc.started() {
			if p.proc.alive(&mut code) {
				return;
			}
			let ran = now - p.last_start;
			p.short_runs = if ran < RUN_HELD_S { p.short_runs.saturating_add(1) } else { 0 };
			self.hello_ok.store(false, SeqCst);
			let wait = restart_wait(p.short_runs) - ran;
			if wait <= 0.0 {
				log("sally: the runtime exited (code %d), starting it again\n", &[BArg::I(code as i64)]);
				self.start_runtime_locked(&mut p);
			} else {
				p.down = true;
				log(
					"sally: the runtime exited (code %d) %.0f s after its start (%d starts in a row that did not hold): \
					 starting it again in %.0f s\n",
					&[BArg::I(code as i64), BArg::F(ran), BArg::I(p.short_runs as i64), BArg::F(wait)],
				);
			}
			return;
		}
		if now - p.last_start < restart_wait(p.short_runs) {
			return;
		}
		if p.down {
			log("sally: starting the runtime again\n", &[]);
			self.start_runtime_locked(&mut p);
		} else if now - self.last_heard() > 10.0 && now - p.init_t > 10.0 {
			let silent = if self.last_heard() > 0.0 { now - self.last_heard() } else { now - p.init_t };
			log("sally: no runtime heard for %.0f s, starting one\n", &[BArg::F(silent)]);
			self.start_runtime_locked(&mut p);
		}
	}

	fn receive_thread(&self) {
		let mut buf = vec![0u8; 65536];
		let mut last_keepalive = 0.0;
		let mut last_heard_log = 0.0;
		locked(&self.process).init_t = now_seconds();
		while self.running.load(SeqCst) {
			let now = now_seconds();
			let pairs_changed = G_OVERRIDE.take_pairs_changed();
			if now - last_keepalive >= 1.0 {
				last_keepalive = now;
				if !self.hello_ok.load(SeqCst) {
					self.send(b"#h hello");
				}
				// the device list first: a restarted runtime's devices and dongles reach the identity check before its
				// poses do
				self.send(b"#d devices"); // device list: new devices, serials, lever arms, dongles
				self.send(b"sub driver"); // (re)subscribe; also re-attaches to a restarted runtime
				self.send_pairs(now); // which device each dongle carries
				self.manage_runtime_process(now);
				if self.last_heard() > 0.0 && now - self.last_heard() > 3.0 && now - last_heard_log > 10.0 {
					last_heard_log = now;
					log("sally: runtime silent for %.0f s\n", &[BArg::F(now - self.last_heard())]);
					self.hello_ok.store(false, SeqCst);
				}
			} else if pairs_changed {
				self.send_pairs(now);
			}
			let n = net::recv(self.sock.load(SeqCst), &mut buf[..65535]);
			if n <= 0 {
				continue;
			}
			buf[n as usize] = 0;
			self.last_heard.store(now_seconds().to_bits(), SeqCst);
			// one line per datagram, but tolerate several
			for line in lines(&buf) {
				self.handle_line(line);
			}
		}
	}

	/// HandleLine(line): Q and P lines are poses, "#h" the hello's reply, "#d DEV" the device list's lines
	pub fn handle_line(&self, line: &[u8]) {
		let t = line.first().copied().unwrap_or(0);
		if (t == b'Q' || t == b'P') && line.get(1) == Some(&b' ') {
			self.handle_pose(line, t == b'Q');
			return;
		}
		if t != b'#' {
			return; // L and anything else: not used
		}
		let f = split(line);
		if f.len() < 2 {
			return;
		}
		if f[0] == b"#h" && f.len() >= 5 && f[1] == b"OK" && f[2] == b"hello" {
			let proto = c_atoi(f[4]);
			if proto < SALLY_MIN_PROTOCOL {
				if !self.proto_warned.load(SeqCst) {
					log(
						"sally: the runtime speaks protocol %d, this driver needs %d or later (update the runtime)\n",
						&[BArg::I(proto as i64), BArg::I(SALLY_MIN_PROTOCOL as i64)],
					);
				}
				self.proto_warned.store(true, SeqCst);
				return;
			}
			if !self.hello_ok.load(SeqCst) {
				log(
					"sally: connected to %s protocol %d (%s devices, %s stations)\n",
					&[
						BArg::S(f[3]),
						BArg::I(proto as i64),
						BArg::S(if f.len() > 6 { f[6] } else { b"?" }),
						BArg::S(if f.len() > 7 { f[7] } else { b"?" }),
					],
				);
			}
			self.proto.store(proto, SeqCst);
			self.hello_ok.store(true, SeqCst);
			self.send(b"#d devices");
			self.send(b"sub driver");
			self.send_pairs(now_seconds());
		} else if f[0] == b"#d" && f[1] == b"DEV" && f.len() >= 16 {
			self.handle_device(&f);
		}
	}

	/// HandleDevice(f): #d DEV codename serial sensors state light_age subtype timebase_hz h2i_x y z qw qx qy qz
	/// [charge charging dongle]
	fn handle_device(&self, f: &[&[u8]]) {
		let (codename, serial, subtype) = (f[2], f[3], f[7]);
		let configured = serial != b"-" && subtype != b"generic";
		// the dongle it is on, for the identity check (PoseOverride::same_device): every dongle the runtime has, and
		// the device's serial on it once its config is read
		if let Some(&dongle) = f.get(18) {
			G_OVERRIDE.runtime_dongle(dongle, configured.then_some(serial));
		}
		if !configured {
			return; // config not read yet
		}
		// the headset unless driver.hmd=0: with that nothing of the runtime's headset reaches the driver
		let Some(kind) = Kind::of_subtype(subtype, g_opt().hmd) else {
			if locked(&self.skipped).insert(serial.to_vec()) {
				log("sally: ignoring %s (%s, %s)\n", &[BArg::S(serial), BArg::S(codename), BArg::S(subtype)]);
			}
			return;
		};
		let mut h2i = [0.0f64; 7];
		for (i, h) in h2i.iter_mut().enumerate() {
			*h = c_atof(f[9 + i]);
		}
		// a head-to-IMU that is not a number is not taken: the lever arm is kept from the first one (its NaN would be
		// in every velocity after), and HeadFix would work it out again at every pose of Valve's
		let h2i_ok = h2i.iter().all(|x| x.is_finite());
		if h2i_ok {
			// where the runtime puts the head, against Valve's driver-from-head (HeadFix)
			G_OVERRIDE.runtime_head(serial, &h2i);
		} else {
			self.drop_not_finite(serial, "a head-to-IMU");
		}
		let mut l = locked(&self.lock);
		l.codenames.insert(codename.to_vec(), serial.to_vec()); // codenames can change when the runtime restarts; serials do not
		let dev = match l.devices.get(serial) {
			Some(d) => d.clone(),
			None => {
				let dev = Arc::new(RuntimeDevice::new(serial, kind));
				log("sally: runtime device %s (%s)\n", &[BArg::S(serial), BArg::S(dev.kind_name())]);
				l.devices.insert(serial.to_vec(), dev.clone());
				dev
			}
		};
		drop(l);
		if h2i_ok {
			dev.set_device_info(&h2i);
		}
	}

	/// A Q line (docs/sally_runtime_protocol.md) with the tails a runtime may send after it: ten fields for the
	/// hand-over look-ahead, three for the angular acceleration, nine for the corrections and the light's return (the
	/// output layer); or a P line
	///   Q codename serial timecode run_time x y z qw qx qy qz vx vy vz wx wy wz [ax ay az lead dx dy dz dvx dvy dvz
	///     [alx aly alz [cx cy cz cqw cqx cqy cqz ret_age ret_gap]]]
	///   P codename serial timecode run_time wall x y z qw qx qy qz
	fn handle_pose(&self, line: &[u8], q: bool) {
		let (code, serial, v, ho, al, co) = match parse_pose(line, q) {
			Some(x) => x,
			None => return,
		};
		let used = if !q {
			7
		} else if al {
			26
		} else if ho {
			23
		} else {
			13
		};
		if !v[..used].iter().all(|x| x.is_finite()) {
			self.drop_not_finite(&serial, "a runtime pose");
			return;
		}
		let d = {
			let l = locked(&self.lock);
			// by serial: a restarted runtime can hand out the codenames in another order, and until its device lines
			// arrive the codename map still has the old run's (poses went to the wrong device). "-" = serial unknown
			// yet
			let found = if serial != b"-" {
				l.devices.get(&serial)
			} else {
				l.codenames.get(&code).and_then(|s| l.devices.get(s))
			};
			match found {
				Some(d) => d.clone(),
				None => return,
			}
		};
		let pos: [f64; 3] = [v[0], v[1], v[2]];
		let rot: [f64; 4] = [v[3], v[4], v[5], v[6]];
		let vel: [f64; 3] = [v[7], v[8], v[9]];
		let angvel: [f64; 3] = [v[10], v[11], v[12]];
		let hov: [f64; 10] = [v[13], v[14], v[15], v[16], v[17], v[18], v[19], v[20], v[21], v[22]];
		let alv: [f64; 3] = [v[23], v[24], v[25]];
		let cov: [f64; 9] = [v[26], v[27], v[28], v[29], v[30], v[31], v[32], v[33], v[34]];
		d.update(&Report {
			pos: &pos,
			rot: &rot,
			vel: &vel,
			angvel: &angvel,
			has_vel: q,
			ho: if ho { Some(&hov) } else { None },
			al: if al { Some(&alv) } else { None },
			corr: if co { Some(&cov) } else { None },
		});
	}
}

/// The Q line's format: two names, two skipped, 35 doubles
pub const Q_FORMAT: &str = "Q %31s %63s %*s %*s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf \
                            %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf";
/// The P line's format: two names, three skipped, 7 doubles
pub const P_FORMAT: &str = "P %31s %63s %*s %*s %*s %lf %lf %lf %lf %lf %lf %lf";

/// HandlePose's sscanf: the codename, the serial, the 35 values (0 where none was read) and whether the look-ahead,
/// the angular acceleration and the corrections came; None for a line it does not take (a Q line of other than 15,
/// 25, 28 or 37 fields read, a P line of other than 9)
#[allow(clippy::type_complexity)]
pub fn parse_pose(line: &[u8], q: bool) -> Option<(Vec<u8>, Vec<u8>, [f64; 35], bool, bool, bool)> {
	let (got, values) = sscanf(line, if q { Q_FORMAT } else { P_FORMAT });
	let (ho, al, co) = if q {
		if got != 15 && got != 25 && got != 28 && got != 37 {
			return None;
		}
		(got >= 25, got >= 28, got == 37)
	} else {
		if got != 9 {
			return None;
		}
		(false, false, false)
	};
	let mut it = values.into_iter();
	let (Some(Value::S(code)), Some(Value::S(serial))) = (it.next(), it.next()) else { return None };
	let mut v = [0.0f64; 35];
	for (k, x) in it.enumerate() {
		if let Value::Lf(x) = x {
			v[k] = x;
		}
	}
	Some((code, serial, v, ho, al, co))
}

// ---------------------------------------------------------------- the provider's vtable, as vrserver calls it

unsafe extern "C" fn provider_init(
	_this: *mut IServerTrackedDeviceProvider, ctx: *mut IVRDriverContext,
) -> EVRInitError {
	G_PROVIDER.init(ctx)
}

unsafe extern "C" fn provider_cleanup(_this: *mut IServerTrackedDeviceProvider) {
	G_PROVIDER.cleanup();
}

unsafe extern "C" fn provider_get_interface_versions(_this: *mut IServerTrackedDeviceProvider) -> *const *const c_char {
	k_InterfaceVersions.0.as_ptr()
}

unsafe extern "C" fn provider_run_frame(_this: *mut IServerTrackedDeviceProvider) {
	G_PROVIDER.run_frame();
}

unsafe extern "C" fn provider_should_block_standby_mode(_this: *mut IServerTrackedDeviceProvider) -> bool {
	false
}

unsafe extern "C" fn provider_enter_standby(_this: *mut IServerTrackedDeviceProvider) {}

unsafe extern "C" fn provider_leave_standby(_this: *mut IServerTrackedDeviceProvider) {}

static PROVIDER_VTBL: IServerTrackedDeviceProviderVtbl = IServerTrackedDeviceProviderVtbl {
	Init: provider_init,
	Cleanup: provider_cleanup,
	GetInterfaceVersions: provider_get_interface_versions,
	RunFrame: provider_run_frame,
	ShouldBlockStandbyMode: provider_should_block_standby_mode,
	EnterStandby: provider_enter_standby,
	LeaveStandby: provider_leave_standby,
};

/// &g_provider as vrserver sees it: an IServerTrackedDeviceProvider
pub static PROVIDER_OBJECT: IServerTrackedDeviceProvider = Interface { vtbl: &PROVIDER_VTBL };
