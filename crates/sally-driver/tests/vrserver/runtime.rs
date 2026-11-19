//! The scripted runtime: what `sally run` sends the driver over the runtime protocol (docs/sally_runtime_protocol.md),
//! scripted from the scene instead of read from the dongles. The test binary is this runtime when started with
//! `--scripted-runtime <session folder> <the session's clock origin, steady_clock ns>`, by the test or by the driver
//! from sally_settings.txt's runtime arguments (the driver adds `--runtime-port <port>`, which both have, and
//! `--stop-with <its process>`, which the console then names).
//!
//! It answers hello, devices, sub, unsub, ping and quit as the runtime does, and sends each device's Q line every 4 ms
//! to the driver stream's subscribers: the state 9 ms old (the age the driver declares, extra_latency_ms), the
//! look-ahead's acceleration, displacement and velocity change over the device's lead, and the angular acceleration,
//! each as the protocol defines it, so that the driver's hand-over gets the device where it is. A start that finds an
//! earlier start's marker in the folder is a restart: it takes 3 s to come up (the dongles opened again), hands out
//! the codenames in the other order, and has the first two devices' configs on each other's dongles, as a runtime
//! whose dongle caches are stale has (each one's serial on the other's dongle, with the other's motion), until the
//! driver's `pair` for one of them puts them right. Its console (its stdout) says what it got.

use std::io::Write;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::time::Duration;

use sally_math::math3d::{cross, qrot};
use sally_math::printf::{bsprintf, BArg};

use crate::scene::DEVICES;

/// The pipeline's age: the state is this old when its Q line goes out (the driver's extra_latency_ms)
pub const AGE: f64 = 0.009;
/// How long a restarted runtime takes before it answers
pub const RESTART_TIME: f64 = 3.0;
/// Each device's Q lines come this often
const PERIOD: f64 = 0.004;
/// A subscriber that has not re-sent sub for this long is dropped (--runtime-sub-timeout)
const SUB_TIMEOUT: f64 = 5.0;

/// The runtime's marker in the session folder: when it started, came up and quit (the test reads them)
pub fn marker(dir: &Path, pid: u32) -> PathBuf {
	dir.join(format!("runtime-{pid}.txt"))
}

fn note(path: &Path, what: &str, t: f64) {
	let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).expect("the runtime's marker");
	writeln!(f, "{what} {t:.6}").expect("the runtime's marker");
}

pub fn main(args: &[String]) -> ! {
	let dir = PathBuf::from(&args[2]);
	let t0: i64 = args[3].parse().expect("the clock origin");
	let port: u16 = args
		.iter()
		.position(|a| a == "--runtime-port")
		.and_then(|i| args.get(i + 1))
		.and_then(|p| p.parse().ok())
		.expect("--runtime-port <port>");
	let now = || (sally_os::chrono::steady_clock_now() - t0) as f64 / 1e9;
	let pid = std::process::id();
	let parent = std::os::unix::process::parent_id();
	let restart = std::fs::read_dir(&dir)
		.expect("the session folder")
		.any(|e| e.is_ok_and(|e| e.file_name().to_string_lossy().starts_with("runtime-")));
	let mark = marker(&dir, pid);
	note(&mark, "start", now());
	if restart {
		std::thread::sleep(Duration::from_secs_f64(RESTART_TIME));
	}
	let sock = (0..40)
		.find_map(|_| {
			UdpSocket::bind(("127.0.0.1", port)).ok().or_else(|| {
				std::thread::sleep(Duration::from_millis(50));
				None
			})
		})
		.expect("the runtime port");
	note(&mark, "serving", now());
	println!("scripted runtime {pid} on 127.0.0.1:{port}{}", if restart { " (a restart)" } else { "" });
	// whose it is, as the driver says it to a runtime it starts (--stop-with)
	if let Some(owner) = args.iter().position(|a| a == "--stop-with").and_then(|i| args.get(i + 1)) {
		println!("started for {owner}");
	}
	let started = now();
	// the codenames this run hands out (a restart: the other way round)
	let codename = |i: usize| format!("KN{}", if restart { DEVICES.len() - 1 - i } else { i });
	let mut subs: Vec<(SocketAddr, f64)> = Vec::new();
	let mut next: Vec<f64> = (0..DEVICES.len()).map(|i| now() + 0.001 * i as f64).collect();
	let mut timecode = [1_000_000u64; DEVICES.len()];
	// the first two devices on each other's dongles (a restart's stale caches), until a `pair` says otherwise
	let mut swapped = restart;
	// the device whose motion and dongle a serial's lines carry
	let on = |i: usize, swapped: bool| if swapped && i < 2 { 1 - i } else { i };
	let mut buf = [0u8; 2048];
	let mut last_parent_check = now();
	// non-blocking, and asleep in between (a socket's receive timeout counts in the kernel's ticks, which would send
	// the Q lines in bursts)
	sock.set_nonblocking(true).unwrap();
	loop {
		let wait = next.iter().copied().fold(f64::INFINITY, f64::min) - now();
		if wait > 0.0 {
			std::thread::sleep(Duration::from_secs_f64(wait.min(0.001)));
		}
		// the commands that came
		while let Ok((n, from)) = sock.recv_from(&mut buf) {
			let t = now();
			let text = String::from_utf8_lossy(&buf[..n]).into_owned();
			let mut cmd = text.trim_end_matches(['\n', '\r', ' ']);
			println!("got {cmd}");
			// "#<id> <command>": the reply carries the id
			let mut id = String::new();
			if cmd.starts_with('#') {
				let (a, b) = cmd.split_once(' ').unwrap_or((cmd, ""));
				id = format!("{a} ");
				cmd = b;
			}
			let words: Vec<&str> = cmd.split([' ', '\t']).filter(|w| !w.is_empty()).collect();
			let reply = |line: &[u8]| {
				let mut l = id.as_bytes().to_vec();
				l.extend_from_slice(line);
				l.push(b'\n');
				let _ = sock.send_to(&l, from);
			};
			match words.first().copied() {
				Some("hello") => reply(&bsprintf(
					"OK hello sally-runtime 5 %.6f %d 2",
					&[BArg::F(t - started), BArg::I(DEVICES.len() as i64)],
				)),
				Some("devices") => {
					for (i, d) in DEVICES.iter().enumerate() {
						let code = codename(i);
						let (l, s) = (d.lever, d.subtype.as_bytes());
						reply(&bsprintf(
							"DEV %s %s 24 tracking 0.0040 %s 48000000 %.6f %.6f %.6f %.7f %.7f %.7f %.7f 80 0 %s",
							&[
								BArg::S(code.as_bytes()),
								BArg::S(d.serial.as_bytes()),
								BArg::S(s),
								BArg::F(l.x),
								BArg::F(l.y),
								BArg::F(l.z),
								BArg::F(1.0),
								BArg::F(0.0),
								BArg::F(0.0),
								BArg::F(0.0),
								BArg::S(DEVICES[on(i, swapped)].dongle.as_bytes()),
							],
						));
					}
					reply(&bsprintf("OK devices %d", &[BArg::I(DEVICES.len() as i64)]));
				}
				Some("pair") if words.len() == 3 => {
					// the runtime's tap: the dongle gets that device's config, and the device leaves any other dongle
					let named = DEVICES.iter().position(|d| d.dongle == words[1] && d.serial == words[2]);
					if swapped && named.is_some_and(|k| k < 2) {
						swapped = false;
						note(&mark, "fixed", t);
						println!("put {} and {} on their own dongles", DEVICES[0].serial, DEVICES[1].serial);
					}
					reply(&bsprintf("OK pair %s %s", &[BArg::S(words[1].as_bytes()), BArg::S(words[2].as_bytes())]));
				}
				Some("sub") => {
					let fresh = !subs.iter().any(|s| s.0 == from);
					match subs.iter_mut().find(|s| s.0 == from) {
						Some(s) => s.1 = t,
						None => subs.push((from, t)),
					}
					reply(b"OK sub driver");
					// a new subscriber gets the map: the stations' L lines (the driver takes none of them)
					if fresh {
						for l in [
							"L 0 3220893721 1762516800.000000 1.512000 -1.823000 2.410000 \
							 0.7071068 0.0000000 0.0000000 0.7071068\n",
							"L 1 1103245517 1762516800.000000 -1.677000 1.932000 2.388000 \
							 0.7071068 0.0000000 0.0000000 -0.7071068\n",
						] {
							let _ = sock.send_to(l.as_bytes(), from);
						}
					}
				}
				Some("unsub") => {
					subs.retain(|s| s.0 != from);
					reply(b"OK unsub");
				}
				Some("ping") => {
					let state: &[u8] = if subs.iter().any(|s| s.0 == from) { b"subscribed" } else { b"not_subscribed" };
					reply(&bsprintf("OK ping %.6f %s", &[BArg::F(t - started), BArg::S(state)]));
				}
				Some("quit") => {
					reply(b"OK quit");
					note(&mark, "quit", t);
					println!("quit");
					std::process::exit(0);
				}
				Some(w) => reply(&bsprintf("ERR %s unknown command (try help)", &[BArg::S(w.as_bytes())])),
				None => {}
			}
		}
		// the Q lines due
		let t = now();
		subs.retain(|s| t - s.1 < SUB_TIMEOUT);
		for i in 0..DEVICES.len() {
			if t < next[i] {
				continue;
			}
			next[i] = if t - next[i] > 0.1 { t + PERIOD } else { next[i] + PERIOD };
			timecode[i] += 192_000; // 4 ms of the 48 MHz device clock
			let line = q_line(on(i, swapped), DEVICES[i].serial, &codename(i), timecode[i], t, t - started);
			for (to, _) in &subs {
				let _ = sock.send_to(&line, to);
			}
		}
		// a test that went away takes its runtime with it
		if t - last_parent_check > 1.0 {
			last_parent_check = t;
			if std::os::unix::process::parent_id() != parent {
				std::process::exit(1);
			}
		}
	}
}

/// The device's Q line sent at session time `t` (docs/sally_runtime_protocol.md): the state `AGE` old, the IMU point's
/// velocity, the look-ahead over the device's lead (the IMU point's acceleration with alpha x r at its end, the head
/// point's displacement, the IMU point's velocity change with the change of w x r), the angular acceleration; in the
/// runtime's own format, under `serial` (another device's, where the runtime has the configs on the wrong dongles)
fn q_line(i: usize, serial: &str, codename: &str, timecode: u64, t: f64, run_time: f64) -> Vec<u8> {
	let d = &DEVICES[i];
	let ts = t - AGE;
	let (m, m2) = (d.at(ts), d.at(ts + d.lead));
	let (r, r2) = (qrot(m.q, d.lever), qrot(m2.q, d.lever));
	let v_imu = m.v - cross(m.w, r);
	let acc = m2.a - cross(m2.w, cross(m2.w, r2));
	let disp = m2.p - m.p;
	let dv = m2.v - cross(m.w, r2) - v_imu;
	let f = |x: f64| BArg::F(x);
	bsprintf(
		concat!(
			"Q %s %s %llu %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f %.5f %.5f %.5f %.5f %.5f %.5f %.4f %.4f %.4f %.5f ",
			"%.6f %.6f %.6f %.5f %.5f %.5f %.3f %.3f %.3f\n",
		),
		&[
			BArg::S(codename.as_bytes()),
			BArg::S(serial.as_bytes()),
			BArg::U(timecode),
			f(run_time),
			f(m.p.x),
			f(m.p.y),
			f(m.p.z),
			f(m.q.w),
			f(m.q.x),
			f(m.q.y),
			f(m.q.z),
			f(v_imu.x),
			f(v_imu.y),
			f(v_imu.z),
			f(m.w.x),
			f(m.w.y),
			f(m.w.z),
			f(acc.x),
			f(acc.y),
			f(acc.z),
			f(d.lead),
			f(disp.x),
			f(disp.y),
			f(disp.z),
			f(dv.x),
			f(dv.y),
			f(dv.z),
			f(m.al.x),
			f(m.al.y),
			f(m.al.z),
		],
	)
}
