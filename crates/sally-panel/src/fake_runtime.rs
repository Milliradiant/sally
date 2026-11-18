//! A runtime that is not one, for the panel's tests and screenshots without hardware: speaks protocol 4 over UDP with
//! six devices (twelve with `start_with`'s `full`: a whole body's set) and four stations, a body rocking about, a left
//! foot that keeps losing a station (diagnosis: light), a waist with a weak radio and a flat battery, a tracker whose
//! light comes from behind its sensors, and a controller that is paired (its dongle is listed) but switched off, listed
//! and never posed. Its S lines carry the light's residual (an eleventh field). `drop` makes a device go quiet at a
//! time, as one switched off in the middle of a session.
//!
//! The stations look at the middle of the room; each device's sensors are a dome (`sensor`), lit by a station they
//! face (H lines), except the head's sensor COVERED, which never is, and the tracker whose light comes from behind:
//! its sensors that neither of its stations lights get station 1's light from behind them.

use std::collections::HashMap;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// A station: index, id, mode (the channel less one), position
type Station = (i32, u32, i32, (f64, f64, f64));

const STATIONS: [Station; 4] = [
	(0, 0xFB47A06D, 13, (-2.2, -2.0, 2.4)),
	(1, 0x4FB344F8, 3, (2.2, -2.0, 2.4)),
	(2, 0xBDE5E55A, 9, (-2.2, 2.0, 2.3)),
	(3, 0xD0875DAE, 7, (2.2, 2.0, 2.3)),
];

/// codename, serial, subtype, sensors, where it rocks about, battery, dongle
struct Dev {
	code: &'static str,
	serial: &'static str,
	subtype: &'static str,
	sensors: usize,
	base: (f64, f64, f64),
	charge: i32,
	dongle: &'static str,
}

const DEVS: [Dev; 6] = [
	Dev {
		code: "KN0",
		serial: "LHR-0334A73B",
		subtype: "tracker_gen2",
		sensors: 22,
		base: (0.0, 0.0, 1.75),
		charge: 49,
		dongle: "D5096F43FE-1YX",
	},
	Dev {
		code: "KN3",
		serial: "LHR-2D779713",
		subtype: "tracker_gen2",
		sensors: 22,
		base: (0.0, 0.05, 1.05),
		charge: 9,
		dongle: "4BCAD73D0E-6YX",
	},
	Dev {
		code: "KN4",
		serial: "LHR-B0D67561",
		subtype: "tracker_gen2",
		sensors: 22,
		base: (0.12, 0.0, 0.08),
		charge: 98,
		dongle: "0A0E4D6852-2YX",
	},
	Dev {
		code: "KN5",
		serial: "LHR-1B80952C",
		subtype: "tracker_gen2",
		sensors: 22,
		base: (-0.12, 0.0, 0.08),
		charge: 81,
		dongle: "EBF4140A47-4YX",
	},
	Dev {
		code: "KN1",
		serial: "LHR-C57C5A45",
		subtype: "knuckles_r",
		sensors: 24,
		base: (0.35, 0.2, 1.1),
		charge: 83,
		dongle: "2829C4CAEA-5YX",
	},
	Dev {
		code: "KN2",
		serial: "LHR-9365E531",
		subtype: "knuckles_l",
		sensors: 24,
		base: (-0.35, 0.2, 1.1),
		charge: 81,
		dongle: "1B7A62914E-3YX",
	},
];

/// The rest of a whole body's set (`FakeRuntime::start_with`'s `full`): an Index headset on its cable and five more
/// trackers, all well. Twelve devices: what the panel's lists have to hold
const MORE: [Dev; 6] = [
	Dev {
		code: "T20",
		serial: "LHR-0A0A0020",
		subtype: "index_hmd",
		sensors: 32,
		base: (0.0, -0.03, 1.66),
		charge: -1,
		dongle: "-",
	},
	Dev {
		code: "KN7",
		serial: "LHR-0A0A0007",
		subtype: "tracker_gen2",
		sensors: 18,
		base: (0.0, 0.07, 1.36),
		charge: 64,
		dongle: "6FE9771ADC-7YX",
	},
	Dev {
		code: "KN8",
		serial: "LHR-0A0A0008",
		subtype: "tracker_gen2",
		sensors: 18,
		base: (-0.3, 0.1, 1.32),
		charge: 72,
		dongle: "B08A0CBD70-DYX",
	},
	Dev {
		code: "KN9",
		serial: "LHR-0A0A0009",
		subtype: "tracker_gen2",
		sensors: 18,
		base: (0.3, 0.1, 1.32),
		charge: 55,
		dongle: "00A4AE2B50-RYB",
	},
	Dev {
		code: "KN10",
		serial: "LHR-0A0A0010",
		subtype: "tracker_gen2",
		sensors: 18,
		base: (-0.13, 0.03, 0.5),
		charge: 91,
		dongle: "A9DDD3B47A-LYM",
	},
	Dev {
		code: "KN11",
		serial: "LHR-0A0A0011",
		subtype: "tracker_gen2",
		sensors: 18,
		base: (0.13, 0.03, 0.5),
		charge: 100,
		dongle: "2A1F10B651-DYX",
	},
];

/// the one paired to its dongle and switched off
const OFF: (&str, &str, &str, usize, &str) = ("KN6", "LHR-7E87BF59", "knuckles_r", 24, "21879ADDC6-7YX");

/// The head's (KN0's) sensor that something covers: it faces station 0 (22 deg off its normal) and is never lit
pub const COVERED: usize = 15;

/// Sensor k of n on a device: a dome of 4 cm, in rings at 10, 40 and 70 deg up, facing out (its place and normal)
pub fn sensor(k: usize, n: usize) -> ((f64, f64, f64), (f64, f64, f64)) {
	let e = [10.0f64, 40.0, 70.0][k % 3].to_radians();
	let a = k as f64 / n as f64 * std::f64::consts::TAU + (k % 3) as f64 * 0.4;
	let u = (e.cos() * a.cos(), e.cos() * a.sin(), e.sin());
	((0.04 * u.0, 0.04 * u.1, 0.04 * u.2), u)
}

/// A station's turn (w x y z): its -Z toward the middle of the room, a metre up
fn station_q(p: (f64, f64, f64)) -> (f64, f64, f64, f64) {
	let n = |v: (f64, f64, f64)| {
		let l = (v.0 * v.0 + v.1 * v.1 + v.2 * v.2).sqrt();
		(v.0 / l, v.1 / l, v.2 / l)
	};
	let cr =
		|a: (f64, f64, f64), b: (f64, f64, f64)| (a.1 * b.2 - a.2 * b.1, a.2 * b.0 - a.0 * b.2, a.0 * b.1 - a.1 * b.0);
	let z = n((p.0, p.1, p.2 - 1.0));
	let x = n(cr((0.0, 0.0, 1.0), z));
	let y = cr(z, x);
	// the rotation whose columns are x, y and z
	let w = (1.0 + x.0 + y.1 + z.2).max(1e-12).sqrt() / 2.0;
	(w, (y.2 - z.1) / (4.0 * w), (z.0 - x.2) / (4.0 * w), (x.1 - y.0) / (4.0 * w))
}

/// The sensors station `i` lights on a device at `at` (its sensors facing it within 72 deg), unturned as its poses are
fn faces(d: &Dev, at: (f64, f64, f64), i: usize) -> Vec<bool> {
	let sp = STATIONS[i].3;
	(0..d.sensors)
		.map(|k| {
			let (p, u) = sensor(k, d.sensors);
			let v = (sp.0 - at.0 - p.0, sp.1 - at.1 - p.1, sp.2 - at.2 - p.2);
			let l = (v.0 * v.0 + v.1 * v.1 + v.2 * v.2).sqrt();
			(u.0 * v.0 + u.1 * v.1 + u.2 * v.2) / l > 72f64.to_radians().cos()
		})
		.collect()
}

pub struct FakeRuntime {
	pub port: u16,
	stop: Arc<AtomicBool>,
	thread: Option<JoinHandle<()>>,
}

impl Drop for FakeRuntime {
	fn drop(&mut self) {
		self.stop.store(true, Ordering::Relaxed);
		if let Some(t) = self.thread.take() {
			let _ = t.join();
		}
	}
}

impl FakeRuntime {
	/// On `port` (0: any free one); `drop`: (codename, seconds) of devices that go quiet then
	pub fn start(port: u16, drop: Vec<(String, f64)>) -> std::io::Result<FakeRuntime> {
		FakeRuntime::start_with(port, drop, false)
	}

	/// As `start`; `full`: with the rest of a whole body's set (twelve devices: a headset, two controllers, nine
	/// trackers)
	pub fn start_with(port: u16, drop: Vec<(String, f64)>, full: bool) -> std::io::Result<FakeRuntime> {
		let sock = UdpSocket::bind(("127.0.0.1", port))?;
		sock.set_nonblocking(true)?;
		let port = sock.local_addr()?.port();
		let stop = Arc::new(AtomicBool::new(false));
		let s2 = stop.clone();
		let thread = std::thread::spawn(move || run(sock, drop, s2, full));
		Ok(FakeRuntime { port, stop, thread: Some(thread) })
	}
}

fn pose(d: &Dev, t: f64) -> (f64, f64, f64) {
	let (bx, by, bz) = d.base;
	let sway = 0.08 * (t * 1.3).sin();
	let bob = 0.02 * (t * 2.6).sin();
	let (mut x, y, mut z) = (bx + sway + 0.3 * (t * 0.2).sin(), by + 0.3 * (t * 0.2).cos(), bz + bob);
	if d.subtype.starts_with("knuckles") {
		x += 0.15 * (t * 3.0 + if d.code == "KN1" { 1.0 } else { 0.0 }).sin();
		z += 0.1 * (t * 3.0).cos();
	}
	(x, y, z)
}

fn lh_mask(d: &Dev, t: f64) -> u32 {
	match d.code {
		// the left foot: station 3 blocked most of the time, now and then all
		"KN5" if t % 9.0 > 5.5 => {
			if t % 9.0 > 6.5 {
				0b0001
			} else {
				0b0011
			}
		}
		"KN4" => 0b0011,
		"KN3" => 0b0111,
		_ => 0b1111,
	}
}

fn run(sock: UdpSocket, drop: Vec<(String, f64)>, stop: Arc<AtomicBool>, full: bool) {
	let devs: Vec<&Dev> = DEVS.iter().chain(MORE.iter().filter(|_| full)).collect();
	let t0 = Instant::now();
	let mut subs: HashMap<SocketAddr, f64> = HashMap::new();
	let quiet = |code: &str, t: f64| drop.iter().any(|(c, at)| c == code && t >= *at);
	let (mut last_q, mut last_h, mut last_diag) = (-1.0, -1.0, -1.0);
	let mut rng: u64 = 0x2545F4914F6CDD1D;
	let mut rand = move |n: u64| {
		rng ^= rng << 13;
		rng ^= rng >> 7;
		rng ^= rng << 17;
		rng % n
	};
	let mut buf = [0u8; 4096];
	while !stop.load(Ordering::Relaxed) {
		let t = t0.elapsed().as_secs_f64();
		let wall =
			std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0);
		let send = |line: &str, to: SocketAddr| {
			let _ = sock.send_to(format!("{line}\n").as_bytes(), to);
		};
		// the commands
		while let Ok((n, addr)) = sock.recv_from(&mut buf) {
			let line = String::from_utf8_lossy(&buf[..n]).trim().to_string();
			let (pid, line) = match line.strip_prefix('#') {
				Some(rest) => {
					let (id, cmd) = rest.split_once(' ').unwrap_or((rest, ""));
					(format!("#{id} "), cmd.to_string())
				}
				None => (String::new(), line),
			};
			let f: Vec<&str> = line.split_whitespace().collect();
			let Some(&c) = f.first() else { continue };
			let reply = |r: &str| send(&format!("{pid}{r}"), addr);
			match c {
				"sub" => {
					let new = !subs.contains_key(&addr);
					subs.insert(addr, t);
					reply("OK sub driver state diag");
					if new {
						for (i, sid, _, p) in STATIONS {
							let q = station_q(p);
							send(
								&format!(
									"L {i} {sid} {wall:.3} {:.3} {:.3} {:.3} {:.6} {:.6} {:.6} {:.6}",
									p.0, p.1, p.2, q.0, q.1, q.2, q.3
								),
								addr,
							);
						}
					}
				}
				"hello" => reply(&format!("OK hello sally-runtime 4 {t:.3} {} {}", devs.len(), STATIONS.len())),
				"ping" => {
					subs.insert(addr, t);
					reply(&format!("OK ping {t:.3} subscribed"));
				}
				"devices" => {
					for &d in &devs {
						reply(&format!(
							"DEV {} {} {} tracking 0.004 {} 48000000 0 0 0 1 0 0 0 {} 0 {}",
							d.code, d.serial, d.sensors, d.subtype, d.charge, d.dongle
						));
					}
					reply(&format!(
						"DEV {} {} {} none -1 {} 48000000 0 0 0 1 0 0 0 -1 0 {}",
						OFF.0, OFF.1, OFF.3, OFF.2, OFF.4
					));
					reply(&format!("OK devices {}", devs.len() + 1));
				}
				"sensors" => {
					let code = f.get(1).copied().unwrap_or("");
					match devs.iter().find(|d| d.code == code) {
						Some(d) => {
							for i in 0..d.sensors {
								let (p, u) = sensor(i, d.sensors);
								reply(&format!(
									"SENS {} {i} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}",
									d.code, p.0, p.1, p.2, u.0, u.1, u.2
								));
							}
							reply(&format!("OK sensors {} {}", d.code, d.sensors));
						}
						None => reply(&format!("OK sensors {code} 0")),
					}
				}
				"map" => {
					for (i, sid, ch, p) in STATIONS {
						let q = station_q(p);
						reply(&format!(
							"M {i} {sid} {ch} 1 1 {:.3} {:.3} {:.3} {:.6} {:.6} {:.6} {:.6}",
							p.0, p.1, p.2, q.0, q.1, q.2, q.3
						));
					}
					reply(&format!("OK map locked {}", STATIONS.len()));
				}
				"quit" => reply("OK quit"),
				"calibrate" => match f.get(1).copied() {
					Some("status") => reply("OK calibrate pending sallytap_20261108_220747.hidtap"),
					Some("apply") => reply("ERR calibrate apply: no calibration is ready"),
					_ => reply("OK calibrate started on sallytap_20261109_010441.hidtap so far"),
				},
				_ => reply(&format!("ERR {c} unknown")),
			}
		}
		subs.retain(|_, at| t - *at <= 5.0);
		let to_all = |line: &str| {
			for a in subs.keys() {
				let _ = sock.send_to(format!("{line}\n").as_bytes(), a);
			}
		};
		// the streams at about 125 Hz
		if t - last_q >= 0.008 {
			last_q = t;
			for &d in &devs {
				if quiet(d.code, t) {
					continue;
				}
				let (x, y, z) = pose(d, t);
				let tc = ((t * 48e6) as u64) & 0xffff_ffff;
				to_all(&format!("Q {} {} {tc} {t:.3} {x:.4} {y:.4} {z:.4} 1 0 0 0 0.01 0 0 0 0 0", d.code, d.serial));
				let m = lh_mask(d, t);
				let st = if m != 0 { "tracking" } else { "dark" };
				let la = if m != 0 { 0.004 } else { (t % 9.0 - 6.5).min(2.0) };
				let n = m.count_ones();
				let sd = if n >= 2 {
					0.0012
				} else if n == 1 {
					0.014
				} else {
					0.03 + 0.1 * la
				};
				// the light's residual (mrad): a fraction of a milliradian while lit, more with one station, -1 dark
				let res = match n {
					0 => -1.0,
					1 => 1.2 + 0.4 * (t * 2.3).sin().abs(),
					_ => 0.25 + 0.1 * (t * 1.7 + d.base.0).sin().abs(),
				};
				to_all(&format!("S {} {} {tc} {t:.3} {st} {la:.3} {sd:.4} 0.004 {m:x} {res:.3}", d.code, d.serial));
			}
		}
		if t - last_h >= 1.0 {
			last_h = t;
			for &d in &devs {
				if quiet(d.code, t) {
					continue;
				}
				let m = lh_mask(d, t);
				let at = pose(d, t);
				let lit: Vec<Vec<bool>> = (0..STATIONS.len()).map(|i| faces(d, at, i)).collect();
				for (i, _, _, _) in STATIONS {
					if (m >> i) & 1 == 0 {
						continue;
					}
					let hits: Vec<String> = (0..d.sensors)
						.map(|k| {
							let covered = d.code == "KN0" && k == COVERED;
							// the reflections: what none of its stations lights gets station 1's light from behind
							let reflected = d.code == "KN4"
								&& i == 1 && !(0..STATIONS.len()).any(|j| (m >> j) & 1 != 0 && lit[j][k]);
							if (lit[i as usize][k] && !covered) || reflected {
								(1 + rand(12)).to_string()
							} else {
								"0".into()
							}
						})
						.collect();
					to_all(&format!("H {} {i} 1.0 {} {}", d.code, d.sensors, hits.join(" ")));
				}
			}
			for (i, sid, ch, p) in STATIONS {
				let q = station_q(p);
				to_all(&format!("F {i} seen {t:.3}"));
				to_all(&format!(
					"M {i} {sid} {ch} 1 1 {:.3} {:.3} {:.3} {:.6} {:.6} {:.6} {:.6}",
					p.0, p.1, p.2, q.0, q.1, q.2, q.3
				));
			}
		}
		if t - last_diag >= 0.5 {
			last_diag = t;
			// the runtime's cause texts (add_cause, crates/sally-runtime/src/diagnosis.rs)
			to_all(&format!("D {} {t:.3} device 2 0 100.0 {:.0} 0 0 0.0 0 0 none -1.00 -1 1", OFF.0, 1e3 * t));
			to_all(&format!(
				"C {} {t:.3} device 2 no reports for {:.0} s (switched off, flat, or out of range)",
				OFF.0,
				120.0 + t
			));
			for &d in &devs {
				let m = lh_mask(d, t);
				let n = m.count_ones();
				let mut causes: Vec<(&str, i32, String)> = Vec::new();
				if quiet(d.code, t) {
					let at = drop.iter().find(|(c, _)| c == d.code).map(|x| x.1).unwrap_or(0.0);
					causes.push((
						"device",
						2,
						format!("no reports for {:.0} s (switched off, flat, or out of range)", t - at),
					));
				} else if d.code == "KN5" && n <= 1 {
					let text = if n == 1 {
						"only one base station sees it".to_string()
					} else {
						format!("no light for {:.1} s", t % 9.0 - 6.5)
					};
					causes.push(("light", if n == 1 { 1 } else { 2 }, text));
				}
				if d.code == "KN3" {
					causes.push(("radio", 1, "6% of reports lost".into()));
					causes.push(("device", 1, "battery at 9%".into()));
				}
				if d.code == "KN4" {
					causes.push((
						"reflect",
						1,
						"mostly with base station ch 7 (8% of hits from behind sensors)".into(),
					));
				}
				causes.sort_by_key(|c| -c.1);
				let (head, sev) = causes.first().map(|c| (c.0, c.1)).unwrap_or(("fine", 0));
				let (loss, bars) = if d.code == "KN3" { (6.0, 2) } else { (0.1, 4) };
				let back = if d.code == "KN4" { 8.0 } else { 0.0 };
				let st = if m != 0 { "tracking" } else { "dark" };
				to_all(&format!(
					"D {} {t:.3} {head} {sev} {bars} {loss:.1} 40 {n} 14 {back:.1} 3 0 {st} 0.004 {} {}",
					d.code,
					d.charge,
					causes.len()
				));
				for (c, s, text) in &causes {
					to_all(&format!("C {} {t:.3} {c} {s} {text}", d.code));
				}
			}
		}
		std::thread::sleep(Duration::from_millis(2));
	}
}
