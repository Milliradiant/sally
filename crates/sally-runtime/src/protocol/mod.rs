//! The runtime protocol (docs/sally_runtime_protocol.md, protocol 5). The runtime's streams go out as text lines over
//! UDP, one per datagram, and its control commands come in on the same socket, for the SteamVR driver and the panel.
//! Passive: it never creates devices, and a line whose send would block is dropped and counted, so the tracker never
//! waits on a client.
//!
//! - `--runtime-port PORT` (`--runtime-bind ADDR`): the socket bound there. A client that sends `sub pose diag` (and
//!   repeats it, or `ping`, within runtime-sub-timeout s) gets those streams on the socket it sent from, and every
//!   command its reply there.
//! - `--pose-udp PORT` (`--pose-udp-host H`): the legacy push of the P, V and L lines to H:PORT.
//! - `--runtime-driver-file FILE`: the driver stream written to FILE, every line of it (a file drops none), with each
//!   device's DEV line before its first: what the SteamVR driver is sent on a replay, for the bench (`sally-bench
//!   output`). On its own it opens no port.
//!
//! The pose stream: P (each report's pose), V (its velocity), L (a station's pose set or moved), Q (pose and velocity
//! of one report, with the tracker's hand-over look-ahead), S (the tracker's state, at most runtime-state-hz per
//! device). The diagnostics stream: H (light hits per sensor per station), T (the tracker's state and counters), M
//! (the map), F (a station seen or lost) every runtime-diag-period s of run time, and D and C (each diagnosis verdict
//! and its causes). The control channel is in `commands`.
//!
//! Not sent:
//! - R lines (one per solve) and the T lines' solve counts: nothing reports solves, so there are no R lines and the
//!   counts are 0.
//! - LH1 light's hit counts: gen2 hits are counted on the sweep_angle hook.
//! - a device's slot is not freed as it goes: devices go only as the runtime closes.
//!
//! The poll is the runtime's (`Runtime::poll`): `sally run` polls every 10 ms of wall time on its main thread, `sally
//! replay` every 10 ms of the playback's pace on the recording's clock.

mod commands;
mod sys;
#[cfg(test)]
mod tests;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};

use sally_lighthouse::activations::SENSORS_PER_OBJECT;
use sally_lighthouse::NUM_GEN2_LIGHTHOUSES;
use sally_math::printf::{sprintf_into, Arg};
use sally_math::{c_cos, c_sin, cross3d, dot3d, norm3d, quatgetreciprocal, quatrotateabout, quatrotatevector, Pose};
use sally_os::StartTime;
use sally_track::host::SurviveVelocity;

use crate::config::SC_GET;
use crate::diagnosis::DeviceDiagnosis;
use crate::driver::Driver;
use crate::object::TrackedObject;
use crate::runtime::Runtime;
use crate::ObjectId;
use sys::wall_epoch;

/// driver_pose_udp.c's OGStartTimeS, for the subscribers' clock (OGRelativeTime: when each was last seen, against now).
/// The C's is taken at the file's first timed call (its first log line, as the protocol starts), this one at the first
/// subscriber's: only differences of two readings are used, the same from either start (map.rs's START says why)
static START: StartTime = StartTime::new();

/// RUNTIME_PROTOCOL_VERSION (sally: 5, `pair`)
const PROTOCOL_VERSION: i32 = 5;

/// The pose stream: P V L S
const STREAM_POSE: u32 = 1;
/// The diagnostics stream: H T M F (and the C's R, never sent here), and D C
const STREAM_DIAG: u32 = 2;
/// P V L only: the --pose-udp push target's
const STREAM_POSE_LEGACY: u32 = 4;
/// The driver stream: Q and L, plus P for a device that has no Q (what the SteamVR driver needs, nothing more)
const STREAM_DRIVER: u32 = 8;
/// The state stream: S only (the panel takes driver, state and diag instead of the full pose stream)
const STREAM_STATE: u32 = 16;
/// The streams a subscriber can have
const STREAM_SUBSCRIBED: u32 = STREAM_POSE | STREAM_DIAG | STREAM_DRIVER | STREAM_STATE;
/// A device without a Q for this long (s of run time) gets its P lines on the driver stream
const Q_STALE_S: f64 = 0.5;

const MAX_SUBS: usize = 16;
const MAX_OBJS: usize = 32;
const MAX_CMDS_PER_POLL: usize = 64;

/// The size of the C's line buffers: the pose stream's, the diag summaries'
const LINE: usize = 512;
const DIAG_LINE: usize = 1024;

/// The Q line's format
const Q_FMT: &str = concat!(
	"Q %s %s %llu %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f %.5f %.5f %.5f %.5f %.5f %.5f %.4f %.4f %.4f %.5f ",
	"%.6f %.6f %.6f %.5f %.5f %.5f %.3f %.3f %.3f %.6f %.6f %.6f %.9f %.9f %.9f %.9f %.4f %.3f\n",
);

/// RuntimeSub: a client subscribed from its socket
#[derive(Clone, Copy, Debug)]
struct Sub {
	addr: SocketAddrV4,
	/// wall seconds (OGRelativeTime) of its last sub or ping
	last_seen: f64,
	mask: u32,
	used: bool,
}

impl Default for Sub {
	/// A free slot (zeroed)
	fn default() -> Self {
		Sub { addr: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0), last_seen: 0.0, mask: 0, used: false }
	}
}

/// RuntimeObj: a device's slot
#[derive(Clone, Debug)]
struct Slot {
	so: Option<ObjectId>,
	/// angle events in the current diag period
	hits: [[u32; SENSORS_PER_OBJECT]; NUM_GEN2_LIGHTHOUSES],
	/// run time of the last S line
	last_state_rt: f64,
	/// the latest P, paired with the V of the same report into Q
	pose_timecode: u64,
	pose: Pose,
	have_pose: bool,
	/// run time of the last Q (the driver stream: P stands in while this is stale)
	last_q_rt: f64,
	/// the Q lines' corrections (Protocol::corrections): the head point's summed since the slot began, and the
	/// tracker's sums as the last Q had them
	corr_head: [f64; 3],
	corr_p_last: [f64; 3],
	corr_q_last: [f64; 4],
	have_corr: bool,
	/// the device's DEV line is in the driver file
	dev_filed: bool,
}

impl Default for Slot {
	/// A free slot (zeroed)
	fn default() -> Self {
		Slot {
			so: None,
			hits: [[0; SENSORS_PER_OBJECT]; NUM_GEN2_LIGHTHOUSES],
			last_state_rt: 0.0,
			pose_timecode: 0,
			pose: Pose::default(),
			have_pose: false,
			last_q_rt: 0.0,
			corr_head: [0.0; 3],
			corr_p_last: [0.0; 3],
			corr_q_last: [1.0, 0.0, 0.0, 0.0],
			have_corr: false,
			dev_filed: false,
		}
	}
}

/// SurviveRuntime: the protocol's state
pub struct Protocol {
	sock: UdpSocket,
	/// the runtime port is open
	bound: bool,
	/// the --pose-udp push target
	legacy: Option<SocketAddrV4>,
	/// --runtime-driver-file: the driver stream's lines, all of them
	file: Option<BufWriter<File>>,
	subs: [Sub; MAX_SUBS],
	objs: Vec<Slot>,
	state_hz: f64,
	diag_period: f64,
	sub_timeout: f64,
	last_diag_rt: f64,
	station_seen: [bool; NUM_GEN2_LIGHTHOUSES],
	station_known: [bool; NUM_GEN2_LIGHTHOUSES],
	sent: u64,
	dropped: u64,
	cmds: u64,
	/// a client asked the runtime to exit; the next poll ends the session
	quit: bool,
	/// the line being sent: one buffer, kept from line to line
	buf: String,
}

/// What DriverRegPose_Udp returns: the protocol started, or SURVIVE_DRIVER_PASSIVE for the second of its two
/// registrations (the first set everything up), or an error status
pub enum Reg {
	Started(Box<Protocol>),
	Passive,
	Error(i32),
}

/// serial_of: the device's serial, "-" when unknown
fn serial_of(so: &TrackedObject) -> &str {
	if so.serial_number.is_empty() {
		"-"
	} else {
		&so.serial_number
	}
}

/// dev_state's names of the tracker's states (SURVIVE_DIAG_STATE_*), "none" past them
fn state_name(state: i32) -> &'static str {
	const NAMES: [&str; 5] = ["none", "tracking", "dark", "coasting", "reacquiring"];
	if state >= 0 && state < 5 {
		NAMES[state as usize]
	} else {
		"none"
	}
}

/// A line as the C's snprintf into char[size] leaves it: its first size - 1 bytes (a byte cut, as the C's)
fn cut(line: &str, size: usize) -> &[u8] {
	&line.as_bytes()[..line.len().min(size - 1)]
}

/// Doubles as printf arguments, into `args` from its start
fn floats(args: &mut [Arg<'_>], values: impl IntoIterator<Item = f64>) {
	for (a, v) in args.iter_mut().zip(values) {
		*a = Arg::F(v);
	}
}

/// format_lh: a station's L line (its pose as given), appended to `out`
fn format_lh(out: &mut String, rt: &Runtime, bsd_idx: usize, pose: &Pose) {
	let mut args = [Arg::F(0.0); 10];
	args[..3].copy_from_slice(&[
		Arg::I(bsd_idx as u8 as i64),
		Arg::U(rt.bsd[bsd_idx].base_station_id as u64),
		Arg::F(wall_epoch()),
	]);
	floats(&mut args[3..], pose.pos.iter().chain(&pose.rot).copied());
	sprintf_into(out, "L %d %u %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", &args);
}

/// format_map_line: a station's M line, appended to `out`
fn format_map_line(out: &mut String, rt: &Runtime, lh: usize, locked: bool) {
	let b = &rt.bsd[lh];
	let mut args = [Arg::F(0.0); 12];
	args[..5].copy_from_slice(&[
		Arg::I(lh as i64),
		Arg::U(b.base_station_id as u64),
		Arg::I(b.mode as i64),
		Arg::I(b.position_set as i64),
		Arg::I(locked as i64),
	]);
	floats(&mut args[5..], b.pose.pos.iter().chain(&b.pose.rot).copied());
	sprintf_into(out, "M %d %u %d %d %d %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", &args);
}

/// The stations the M lines and the new subscriber's map cover: the active ones, NUM_GEN2_LIGHTHOUSES at most
fn stations(rt: &Runtime) -> usize {
	(rt.active_lighthouses.max(0) as usize).min(NUM_GEN2_LIGHTHOUSES)
}

impl Protocol {
	/// DriverRegPose_Udp, which DriverRegRuntime_Port is too: libsurvive starts a driver whose name's option is set, so
	/// "pose-udp" starts the legacy push and "runtime-port" the runtime, and the second registration finds the first's
	/// protocol (`started`). The socket made, the push target and the runtime port set up with their log lines; an
	/// error status when neither works
	pub fn reg(rt: &mut Runtime, started: bool) -> Reg {
		let legacy_port = rt.config.configi("pose-udp", SC_GET, 0) as i32;
		let port = rt.config.configi("runtime-port", SC_GET, 0) as i32;
		let file_name = rt.config.configs("runtime-driver-file", SC_GET, None).filter(|f| !f.is_empty());
		if legacy_port <= 0 && port <= 0 && file_name.is_none() {
			return Reg::Error(-1);
		}
		if started {
			return Reg::Passive;
		}
		// socket(): an unbound socket's first sendto binds it to an ephemeral port, which this does now (and on
		// Windows it starts Winsock, which the C's WSAStartup does)
		let Ok(mut sock) = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0)) else {
			rt.warn("runtime: cannot create a UDP socket");
			return Reg::Error(-1);
		};
		let state_hz = rt.config.configf("runtime-state-hz", SC_GET, 100.0);
		let diag_period = rt.config.configf("runtime-diag-period", SC_GET, 1.0);
		let sub_timeout = rt.config.configf("runtime-sub-timeout", SC_GET, 5.0);

		let mut legacy = None;
		if legacy_port > 0 {
			let host = rt.config.configs("pose-udp-host", SC_GET, Some(b"127.0.0.1")).unwrap_or_default();
			let host_text = String::from_utf8_lossy(&host).into_owned();
			match sys::inet_addr_of(&host) {
				None => rt.warn(&format!("pose-udp: bad host '{host_text}'")),
				Some(ip) => {
					// htons((unsigned short)legacy_port)
					legacy = Some(SocketAddrV4::new(ip, legacy_port as u16));
					rt.info(&format!("pose-udp: publishing poses to {host_text}:{legacy_port}"));
				}
			}
		}

		let mut bound = false;
		if port > 0 {
			let bind_host = rt.config.configs("runtime-bind", SC_GET, Some(b"127.0.0.1")).unwrap_or_default();
			let bind_text = String::from_utf8_lossy(&bind_host).into_owned();
			// INADDR_NONE is also 255.255.255.255's, which is bound (and fails)
			let ip = sys::inet_addr_of(&bind_host).or((bind_host == b"255.255.255.255").then_some(Ipv4Addr::BROADCAST));
			match ip {
				None => rt.warn(&format!("runtime: bad bind address '{bind_text}'")),
				Some(ip) => match UdpSocket::bind(SocketAddrV4::new(ip, port as u16)) {
					Err(_) => rt.warn(&format!("runtime: cannot bind {bind_text}:{port} (another runtime running?)")),
					Ok(s) => {
						sock = s;
						if !sys::set_nonblocking(&sock) {
							rt.warn("runtime: cannot make the socket non-blocking");
						} else {
							bound = true;
							rt.info(&format!("runtime: listening on {bind_text}:{port} (protocol {PROTOCOL_VERSION})"));
						}
					}
				},
			}
		}
		if !bound {
			// the push target alone must never block the solver either
			sys::set_nonblocking(&sock);
		}
		let mut file = None;
		if let Some(name) = file_name {
			let name = String::from_utf8_lossy(&name).into_owned();
			match File::create(&name) {
				Ok(f) => {
					file = Some(BufWriter::new(f));
					rt.info(&format!("runtime: the driver stream written to {name}"));
				}
				Err(e) => rt.warn(&format!("runtime: cannot write the driver stream to {name}: {e}")),
			}
		}
		if !bound && legacy.is_none() && file.is_none() {
			return Reg::Error(-1);
		}
		Reg::Started(Box::new(Protocol {
			sock,
			bound,
			legacy,
			file,
			subs: [Sub::default(); MAX_SUBS],
			objs: vec![Slot::default(); MAX_OBJS],
			state_hz,
			diag_period,
			sub_timeout,
			last_diag_rt: 0.0,
			station_seen: [false; NUM_GEN2_LIGHTHOUSES],
			station_known: [false; NUM_GEN2_LIGHTHOUSES],
			sent: 0,
			dropped: 0,
			cmds: 0,
			quit: false,
			buf: String::new(),
		}))
	}

	/// send_to: one datagram, counted as sent, or as dropped when the send would block or fails
	fn send_to(&mut self, addr: SocketAddrV4, line: &[u8]) {
		match self.sock.send_to(line, addr) {
			Ok(n) if n == line.len() => self.sent += 1,
			_ => self.dropped += 1,
		}
	}

	/// emit: one line to every receiver of the stream, the legacy push target for P V L first, then the subscribers by
	/// their masks
	fn emit(&mut self, stream: u32, line: &[u8]) {
		if stream & STREAM_DRIVER != 0 {
			if let Some(f) = self.file.as_mut() {
				let _ = f.write_all(line);
			}
		}
		if let Some(addr) = self.legacy {
			if stream & STREAM_POSE_LEGACY != 0 {
				self.send_to(addr, line);
			}
		}
		for i in 0..MAX_SUBS {
			let s = self.subs[i];
			if s.used && s.mask & stream & STREAM_SUBSCRIBED != 0 {
				self.send_to(s.addr, line);
			}
		}
	}

	/// One line to every receiver of the stream, as `write` writes it into the line buffer, cut where the C's char[size]
	/// cuts it
	fn emit_line(&mut self, stream: u32, size: usize, write: impl FnOnce(&mut String)) {
		let mut buf = std::mem::take(&mut self.buf);
		buf.clear();
		write(&mut buf);
		self.emit(stream, cut(&buf, size));
		self.buf = buf;
	}

	/// has_subscribers: a subscriber takes one of the streams (the driver file takes the driver stream)
	fn has_subscribers(&self, stream: u32) -> bool {
		(self.file.is_some() && stream & STREAM_DRIVER != 0) || self.subs.iter().any(|s| s.used && s.mask & stream != 0)
	}

	/// obj_slot: the device's slot, a free one taken for a device not seen yet (None once all are taken)
	fn obj_slot(&mut self, obj: ObjectId) -> Option<usize> {
		let mut free = None;
		for (i, o) in self.objs.iter().enumerate() {
			if o.so == Some(obj) {
				return Some(i);
			}
			if free.is_none() && o.so.is_none() {
				free = Some(i);
			}
		}
		let i = free?;
		self.objs[i] = Slot { so: Some(obj), last_state_rt: -1e9, last_q_rt: -1e9, ..Slot::default() };
		Some(i)
	}

	/// count_hit: a hit from station lh on a sensor, counted for the H lines
	fn count_hit(&mut self, obj: ObjectId, lh: i32, sensor_id: i32) {
		if lh < 0 || lh >= NUM_GEN2_LIGHTHOUSES as i32 || sensor_id < 0 || sensor_id >= SENSORS_PER_OBJECT as i32 {
			return;
		}
		if let Some(i) = self.obj_slot(obj) {
			let h = &mut self.objs[i].hits[lh as usize][sensor_id as usize];
			*h = h.wrapping_add(1);
		}
	}

	/// diag_flush: the period's summaries: per device its H lines (a station's hits on each sensor, for every station
	/// that lit it) and its T line; per station its M line and, when it went from seen to lost or back, its F line
	fn diag_flush(&mut self, rt: &mut Runtime, t: f64) {
		let period = t - self.last_diag_rt;
		self.last_diag_rt = t;
		let send = self.has_subscribers(STREAM_DIAG);
		let mut seen_now = [false; NUM_GEN2_LIGHTHOUSES];

		for i in 0..MAX_OBJS {
			let Some(obj) = self.objs[i].so else { continue };
			let so = &rt.objects[obj];
			let sensors = if so.sensor_ct > 0 && so.sensor_ct as usize <= SENSORS_PER_OBJECT {
				so.sensor_ct as usize
			} else {
				SENSORS_PER_OBJECT
			};
			let codename = so.codename.clone();
			for lh in 0..NUM_GEN2_LIGHTHOUSES {
				let hits = self.objs[i].hits[lh];
				let total = hits[..sensors].iter().fold(0u32, |a, &h| a.wrapping_add(h));
				if total == 0 {
					continue;
				}
				seen_now[lh] = true;
				if !send {
					continue;
				}
				let head = [Arg::S(&codename), Arg::I(lh as i64), Arg::F(period), Arg::I(sensors as i64)];
				self.emit_line(STREAM_DIAG, DIAG_LINE, |b| {
					sprintf_into(b, "H %s %d %.3f %d", &head);
					for &h in &hits[..sensors] {
						sprintf_into(b, " %u", &[Arg::U(h as u64)]);
					}
					b.push('\n');
				});
			}
			self.objs[i].hits = [[0; SENSORS_PER_OBJECT]; NUM_GEN2_LIGHTHOUSES];

			if send {
				let d = rt.tracker_diag(obj);
				let head = [Arg::S(&codename), Arg::F(t), Arg::S(state_name(d.state)), Arg::F(d.light_age)];
				// the solve counts: per outcome, this period (the ok ones, the rejected ones, then each gate's), with the
				// gates added after protocol 1 after light_res so field positions stay; they come from the
				// solve_report hook, which nothing raises (see the top), so all are 0
				let mut counts = [Arg::U(0); 12];
				counts[7] = Arg::F(0.0);
				self.emit_line(STREAM_DIAG, DIAG_LINE, |b| {
					sprintf_into(b, "T %s %.6f %s %.4f", &head);
					// counters of a tracker that is gone (reported / dropped poses, observations, single-station
					// observations, two removed, IMU-only coasting) and, after the solve counts, its running light
					// residual and per-station residual list stay in the line as zeros: field positions stay
					b.push_str(" 0 0 0 0 0 0 0");
					sprintf_into(b, " %u %u %u %u %u %u %u %.6g %u %u %u %u", &counts);
					b.push_str(" 0");
					b.push('\n');
				});
			}
		}

		let locked = rt.config.configi("lock-lighthouses", SC_GET, 0) != 0;
		for lh in 0..stations(rt) {
			if send {
				self.emit_line(STREAM_DIAG, DIAG_LINE, |b| format_map_line(b, rt, lh, locked));
			}
			// a station counts as seen while any device got light from it this period
			if !self.station_known[lh] || self.station_seen[lh] != seen_now[lh] {
				if self.station_known[lh] && send {
					let state = if seen_now[lh] { "seen" } else { "lost" };
					let args = [Arg::I(lh as i64), Arg::S(state), Arg::F(t)];
					self.emit_line(STREAM_DIAG, DIAG_LINE, |b| sprintf_into(b, "F %d %s %.6f\n", &args));
				}
				self.station_known[lh] = true;
				self.station_seen[lh] = seen_now[lh];
			}
		}
	}

	/// The hand-over look-ahead the Q line carries (docs/sally_driver.md): the IMU point's acceleration at the
	/// end of the lead (world, gravity out), the lead (s: how far the accelerometer's newest samples are past the pose),
	/// the head point's displacement over the lead and the IMU point's velocity change, then the angular
	/// acceleration. All 0 without a seeded tracker. The driver declares the pose `lead` younger, applies the
	/// displacement and the change, and turns the orientation by the rate over the lead
	fn look_ahead(rt: &Runtime, obj: ObjectId, q: &Pose) -> [f64; 13] {
		let mut lead = 0.0;
		let (mut pimu, mut dp, mut dv, mut acc, mut w, mut dph) =
			([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]);
		let al = rt.tracker_handover_alpha(obj).unwrap_or([0.0; 3]);
		let handover = rt.tracker_handover(obj);
		if let Some(h) = &handover {
			(lead, pimu, dp, dv, acc, w) = (h.lead, h.p_imu, h.dp, h.dv, h.a, h.w);
		}
		if handover.is_some() && lead > 0.0 {
			// the head point turns about the IMU point with the device: dp_head = dp_imu + (exp([w] lead) - I)(p_head -
			// p_imu) (the head pose has the floor offset taken off its height, survive_default_imupose_process: put back
			// for the lever)
			let r = [q.pos[0] - pimu[0], q.pos[1] - pimu[1], q.pos[2] + rt.floor_offset - pimu[2]];
			let th = norm3d(&w) * lead;
			let mut rr = r;
			if th > 1e-9 {
				let k = [w[0] * lead / th, w[1] * lead / th, w[2] * lead / th];
				let kd = dot3d(&k, &r);
				let c = c_cos(th);
				let sn = c_sin(th);
				let kxr = cross3d(&k, &r);
				for i in 0..3 {
					rr[i] = r[i] * c + kxr[i] * sn + k[i] * kd * (1.0 - c);
				}
			}
			for i in 0..3 {
				dph[i] = dp[i] + rr[i] - r[i];
			}
			// the angular acceleration's lever term: the head point's acceleration is a_imu + alpha x r + w x (w x r),
			// and the driver adds w x (w x r) to `a`: alpha x r is folded in here, into `a`, the velocity change and the
			// displacement over the lead
			let axr = cross3d(&al, &rr);
			for i in 0..3 {
				acc[i] += axr[i];
				dv[i] += axr[i] * lead;
				dph[i] += 0.5 * axr[i] * lead * lead;
			}
		} else {
			lead = 0.0;
		}
		[acc[0], acc[1], acc[2], lead, dph[0], dph[1], dph[2], dv[0], dv[1], dv[2], al[0], al[1], al[2]]
	}

	/// The output layer's facts the Q line carries after the look-ahead (docs/sally_runtime_protocol.md): the state's
	/// corrections, which a consumer tells from its motion by their change between two Q lines, and the light's return.
	/// The head point's corrections summed since the slot began: each report's correction of the IMU point plus what
	/// the orientation's correction did to the lever (r = IMU point to head point as it stands now: the head point
	/// moved dp + r - e^-1 r); the orientation's corrections composed on the world side, as the tracker sums them
	/// (the same for every point of the device); then how long ago the latest return began (s, -1 before the first) and
	/// how long the device was dark before it (s). Without a seeded tracker the sums stand and the return is -1
	fn corrections(rt: &Runtime, obj: ObjectId, q: &Pose, slot: &mut Slot) -> [f64; 9] {
		let (mut ret_age, mut ret_gap) = (-1.0, 0.0);
		if let Some(h) = rt.tracker_handover(obj) {
			if slot.have_corr {
				let dp = [
					h.corr_p[0] - slot.corr_p_last[0],
					h.corr_p[1] - slot.corr_p_last[1],
					h.corr_p[2] - slot.corr_p_last[2],
				];
				let e = quatrotateabout(&h.corr_q, &quatgetreciprocal(&slot.corr_q_last));
				let r = [q.pos[0] - h.p_imu[0], q.pos[1] - h.p_imu[1], q.pos[2] + rt.floor_offset - h.p_imu[2]];
				let rb = quatrotatevector(&quatgetreciprocal(&e), &r);
				for i in 0..3 {
					slot.corr_head[i] += dp[i] + r[i] - rb[i];
				}
			}
			slot.corr_p_last = h.corr_p;
			slot.corr_q_last = h.corr_q;
			slot.have_corr = true;
			(ret_age, ret_gap) = (h.ret_age, h.ret_gap);
		}
		let (c, e) = (slot.corr_head, slot.corr_q_last);
		[c[0], c[1], c[2], e[0], e[1], e[2], e[3], ret_age, ret_gap]
	}
}

impl Driver for Protocol {
	/// sally: a device started over (another one on its dongle): its slot as a new device's
	fn device_reset(&mut self, _rt: &mut Runtime, obj: ObjectId) {
		if let Some(slot) = self.objs.iter_mut().find(|o| o.so == Some(obj)) {
			*slot = Slot { so: Some(obj), last_state_rt: -1e9, last_q_rt: -1e9, ..Slot::default() };
		}
	}

	/// rt_pose: the P line, to whoever gets it (the pose stream, the push target, and the driver stream for a device
	/// whose Q lines are stale), the pose kept for the report's Q line, then the S line at most runtime-state-hz
	fn pose(&mut self, rt: &mut Runtime, obj: ObjectId, timecode: u64, pose: &Pose) {
		let t = rt.run_time();
		let want_pose = self.has_subscribers(STREAM_POSE);
		let want_driver = self.has_subscribers(STREAM_DRIVER);
		let want_state = want_pose || self.has_subscribers(STREAM_STATE);
		let o = if want_state || want_driver { self.obj_slot(obj) } else { None };
		let mut stream = STREAM_POSE | STREAM_POSE_LEGACY;
		if o.is_some_and(|i| t - self.objs[i].last_q_rt > Q_STALE_S) {
			stream |= STREAM_DRIVER;
		}
		// formatted only when someone gets it (a P line at every report of every device)
		if want_pose || self.legacy.is_some() || (stream & STREAM_DRIVER != 0 && want_driver) {
			let so = &rt.objects[obj];
			let mut args = [Arg::F(0.0); 12];
			args[..5].copy_from_slice(&[
				Arg::S(&so.codename),
				Arg::S(serial_of(so)),
				Arg::U(timecode),
				Arg::F(t),
				Arg::F(wall_epoch()),
			]);
			floats(&mut args[5..], pose.pos.iter().chain(&pose.rot).copied());
			self.emit_line(stream, LINE, |b| {
				sprintf_into(b, "P %s %s %llu %.6f %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", &args)
			});
		}

		let Some(i) = o else { return };
		let slot = &mut self.objs[i];
		slot.pose = *pose;
		slot.pose_timecode = timecode;
		slot.have_pose = true;
		if !want_state || (self.state_hz > 0.0 && t - slot.last_state_rt < 1.0 / self.state_hz) {
			return;
		}
		slot.last_state_rt = t;
		// dev_state: the tracker's state, position sd and station mask; it reports no rotation sd (-1); then the
		// light's residual (mrad: an eleventh field, which older consumers do not read)
		let d = rt.tracker_diag(obj);
		let so = &rt.objects[obj];
		let args = [
			Arg::S(&so.codename),
			Arg::S(serial_of(so)),
			Arg::U(timecode),
			Arg::F(t),
			Arg::S(state_name(d.state)),
			Arg::F(d.light_age),
			Arg::F(d.pos_sd),
			Arg::F(-1.0),
			Arg::U(d.lh_mask as u64),
			Arg::F(d.res_mrad),
		];
		self.emit_line(STREAM_POSE | STREAM_STATE, LINE, |b| {
			sprintf_into(b, "S %s %s %llu %.6f %s %.4f %.6f %.6f %x %.3f\n", &args)
		});
	}

	/// rt_velocity: the V line, then the Q line: the pose and the velocity of the same report in one line, so that a
	/// consumer (the SteamVR driver) never pairs a pose with the previous report's velocity (the tracker reports the
	/// pose first, then the velocity)
	fn velocity(&mut self, rt: &mut Runtime, obj: ObjectId, timecode: u64, v: &SurviveVelocity) {
		if self.legacy.is_some() || self.has_subscribers(STREAM_POSE) {
			let t = rt.run_time();
			let so = &rt.objects[obj];
			let mut args = [Arg::F(0.0); 11];
			args[..5].copy_from_slice(&[
				Arg::S(&so.codename),
				Arg::S(serial_of(so)),
				Arg::U(timecode),
				Arg::F(t),
				Arg::F(wall_epoch()),
			]);
			floats(&mut args[5..], v.pos.iter().chain(&v.axis_angle_rot).copied());
			self.emit_line(STREAM_POSE | STREAM_POSE_LEGACY, LINE, |b| {
				sprintf_into(b, "V %s %s %llu %.6f %.6f %.5f %.5f %.5f %.5f %.5f %.5f\n", &args)
			});
		}

		if !self.has_subscribers(STREAM_POSE | STREAM_DRIVER) {
			return;
		}
		let Some(i) = self.obj_slot(obj) else { return };
		if !self.objs[i].have_pose || self.objs[i].pose_timecode != timecode {
			return;
		}
		self.objs[i].last_q_rt = rt.run_time();
		if !self.objs[i].dev_filed {
			// the driver file: who the device is and where its head sits against its IMU, before its first Q
			self.objs[i].dev_filed = true;
			if let Some(f) = self.file.as_mut() {
				let _ = writeln!(f, "{}", commands::dev_line(rt, obj));
			}
		}
		let q = self.objs[i].pose;
		let ahead = Protocol::look_ahead(rt, obj, &q);
		let corr = Protocol::corrections(rt, obj, &q, &mut self.objs[i]);
		let t = rt.run_time();
		let so = &rt.objects[obj];
		let mut args = [Arg::F(0.0); 39];
		args[..4].copy_from_slice(&[Arg::S(&so.codename), Arg::S(serial_of(so)), Arg::U(timecode), Arg::F(t)]);
		let fields = q.pos.iter().chain(&q.rot).chain(&v.pos).chain(&v.axis_angle_rot).chain(&ahead).chain(&corr);
		floats(&mut args[4..], fields.copied());
		self.emit_line(STREAM_POSE | STREAM_DRIVER, LINE, |b| {
			sprintf_into(b, Q_FMT, &args);
		});
	}

	/// rt_lighthouse: the L line of a station's pose set or moved
	fn lighthouse_pose(&mut self, rt: &mut Runtime, bsd_idx: usize, pose: &Pose) {
		self.emit_line(STREAM_POSE | STREAM_POSE_LEGACY | STREAM_DRIVER, LINE, |b| format_lh(b, rt, bsd_idx, pose));
	}

	/// rt_sweep_angle: a gen2 hit counted for the H lines (with the runtime port open)
	fn sweep_angle(
		&mut self, rt: &mut Runtime, obj: ObjectId, channel: u8, sensor_id: i32, _timecode: u32, _plane: i8,
		_angle: f64,
	) {
		if self.bound {
			let lh = rt.get_bsd_idx(channel);
			self.count_hit(obj, lh as i32, sensor_id);
		}
	}

	/// rt_diagnosis: a device's verdict as a D line, then a C line per cause, the plain-language text running to the
	/// end of the line (with the runtime port open, to the diagnostics stream)
	fn diagnosis(&mut self, rt: &mut Runtime, obj: ObjectId, g: &DeviceDiagnosis) {
		if !self.bound || !self.has_subscribers(STREAM_DIAG) {
			return;
		}
		let codename = rt.objects[obj].codename.clone();
		let state = if g.state >= 0 && g.state < 5 { state_name(g.state) } else { "?" };
		let args = [
			Arg::S(&codename),
			Arg::F(g.run_time),
			Arg::S(cause_name(g.headline)),
			Arg::I(g.severity as i64),
			Arg::I(g.bars as i64),
			Arg::F(100.0 * g.loss),
			Arg::F(1e3 * g.gap_max),
			Arg::I(g.stations as i64),
			Arg::I(g.sensors as i64),
			Arg::F(100.0 * g.back),
			Arg::I(g.gated as i64),
			Arg::I(g.discards as i64),
			Arg::S(state),
			Arg::F(g.light_age),
			Arg::I(g.charge as i64),
			Arg::I(g.causes.len() as i64),
		];
		self.emit_line(STREAM_DIAG, LINE, |b| {
			sprintf_into(b, "D %s %.3f %s %d %d %.1f %.0f %d %d %.1f %d %d %s %.2f %d %d\n", &args)
		});
		for c in &g.causes {
			let args = [
				Arg::S(&codename),
				Arg::F(g.run_time),
				Arg::S(cause_name(c.cause)),
				Arg::I(c.severity as i64),
				Arg::S(&c.text),
			];
			self.emit_line(STREAM_DIAG, LINE, |b| sprintf_into(b, "C %s %.3f %s %d %s\n", &args));
		}
	}

	fn polls(&self) -> bool {
		true
	}

	/// rt_poll: the commands waiting (64 at most), the subscribers that stopped refreshing dropped, the summaries
	/// once a diag period of run time has passed; 1 once a client asked to quit (survive_poll returns it and the
	/// main loop ends: survive_close runs as for --stop-after)
	fn poll(&mut self, rt: &mut Runtime) -> i32 {
		if self.bound {
			let mut buf = [0u8; 2048];
			for _ in 0..MAX_CMDS_PER_POLL {
				// would block (or a stray error): nothing more this poll
				let Ok((n, SocketAddr::V4(from))) = self.sock.recv_from(&mut buf[..2047]) else { break };
				if n == 0 {
					break;
				}
				self.handle_command(rt, &buf[..n], from);
			}
			let now = START.relative_time();
			for i in 0..MAX_SUBS {
				let s = self.subs[i];
				if s.used && now - s.last_seen > self.sub_timeout {
					if rt.settings.log_level >= 10 {
						rt.info(&format!("runtime: subscriber {}:{} timed out", s.addr.ip(), s.addr.port()));
					}
					self.subs[i].used = false;
				}
			}
		}

		let t = rt.run_time();
		if self.diag_period > 0.0 && t - self.last_diag_rt >= self.diag_period {
			self.diag_flush(rt, t);
		}
		if self.quit {
			rt.info("runtime: stopping (quit)");
			return 1;
		}
		0
	}

	/// rt_close: the session's summary (the socket closes as the driver goes)
	fn close(&mut self, rt: &mut Runtime) {
		rt.info(&format!("runtime: sent {} datagrams, {} dropped, {} commands", self.sent, self.dropped, self.cmds));
		if let Some(f) = self.file.as_mut() {
			if f.flush().is_err() {
				rt.warn("runtime: the driver stream's file did not take its last lines");
			}
		}
	}
}

/// diag_cause_name: a diagnosis cause as the D and C lines name it
fn cause_name(c: i32) -> &'static str {
	const NAMES: [&str; 6] = ["fine", "radio", "station", "device", "light", "reflect"];
	if c >= 0 && c < NAMES.len() as i32 {
		NAMES[c as usize]
	} else {
		"?"
	}
}
