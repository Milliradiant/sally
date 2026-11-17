//! The model process as the driver starts it (`<virtual_exe> run --model <dir> --port <port>`), scripted: it speaks
//! `sally-model run`'s side of the protocol (virtual_trackers.rs) on the port. Each F datagram from the driver is answered
//! with the five trackers on a known path (`path`) at the F line's time, and every second an S line; what it is sent is
//! said on its console, which the driver keeps (driver.virtual_log): every device the R lines name, once each, with its
//! class, and each C line. While `<dir>/quiet` exists it answers nothing. A marker `<dir>/model-<pid>.txt` has its
//! start (session time, from the `<dir>/t0` the test leaves).

use std::collections::BTreeSet;
use std::io::Write;
use std::net::UdpSocket;
use std::path::PathBuf;

use sally_math::math3d::{Q4, V3};

/// Tracker k's place at the driver's time t (s), its velocity, its orientation (a turn about +Y)
pub fn path(k: usize, t: f64) -> (V3, V3, Q4) {
	let kf = k as f64;
	let p = V3::new(0.1 * kf, 1.0 + 0.05 * (2.0 * t + kf).sin(), -0.2 + 0.03 * t.cos());
	let v = V3::new(0.0, 0.1 * (2.0 * t + kf).cos(), -0.03 * t.sin());
	let a = 0.2 * (t + kf).sin();
	(p, v, Q4::new((a / 2.0).cos(), 0.0, (a / 2.0).sin(), 0.0))
}

pub const NAMES: [&str; 5] = ["chest", "lknee", "rknee", "lelbow", "relbow"];

pub fn main(args: &[String]) -> ! {
	let get = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
	let dir = PathBuf::from(get("--model").expect("--model"));
	let port: u16 = get("--port").and_then(|p| p.parse().ok()).expect("--port");
	let t0: i64 = std::fs::read_to_string(dir.join("t0")).ok().and_then(|s| s.trim().parse().ok()).unwrap_or(0);
	let session_now = || (sally_os::chrono::steady_clock_now() - t0) as f64 / 1e9;
	let _ = std::fs::write(dir.join(format!("model-{}.txt", std::process::id())), format!("start {}\n", session_now()));
	let sock = UdpSocket::bind(("127.0.0.1", port)).expect("the model's port");
	let mut seen = BTreeSet::new();
	let mut last_s = f64::NEG_INFINITY;
	let mut buf = vec![0u8; 65536];
	println!("scripted model on {port}");
	loop {
		let Ok((n, from)) = sock.recv_from(&mut buf) else { continue };
		let text = String::from_utf8_lossy(&buf[..n]).into_owned();
		let mut t = None;
		for l in text.lines() {
			let w: Vec<&str> = l.split_whitespace().collect();
			match w.as_slice() {
				["F", x] => t = x.parse::<f64>().ok(),
				["C", what] => println!("got C {what}"),
				["R", _, class, serial, ..] if w.len() == 12 => {
					if seen.insert((class.to_string(), serial.to_string())) {
						println!("R {class} {serial}");
					}
				}
				_ => println!("got something else: {l}"),
			}
		}
		let _ = std::io::stdout().flush();
		let Some(t) = t else { continue };
		if dir.join("quiet").exists() {
			continue;
		}
		let mut out = format!("F {t:.4}\n");
		for (k, name) in NAMES.iter().enumerate() {
			let (p, v, q) = path(k, t);
			out += &format!(
				"T {name} 1 {:.5} {:.5} {:.5} {:.6} {:.6} {:.6} {:.6} {:.4} {:.4} {:.4} 0 0 0\n",
				p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z
			);
		}
		if t - last_s >= 1.0 {
			last_s = t;
			out += "S tracking 12.5\n";
		}
		let _ = sock.send_to(out.as_bytes(), from);
	}
}
