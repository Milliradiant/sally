//! A stand-in for the sally driver's side of the panel (docs/sally_driver.md), for the tests and the headless
//! renderer: it sends the panel what the driver sends while SteamVR shows sally's tracking (the V lines with Valve's
//! pose of each device, the C line, the F line with its fit) and answers the panel's commands as the driver does:
//! `seen sally|valve`, `refit`, `output` and `output <scope> <name> <value|->`, and the panel's choices it keeps
//! (`headset sally|valve`, `keep <n>`, `pose_log on|off`).
//!
//! Valve's poses are the runtime's own, in the driver's space (OpenVR's axes, Y up) with a few millimetres of noise, and
//! the fit is the identity: the panel compares through the driver's alignment and finds the two close.

use std::collections::BTreeMap;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};

use sally_math::math3d::{qmul, Q4};

use super::ValveCompare;
use crate::runtime::Runtime;

/// The classes' settings: the name, the defaults (headset, controllers, trackers) and the range, as the driver's
/// output.rs has them
pub const SETTINGS: [(&str, [f64; 3], f64, f64); 16] = [
	("recovery_blend_ms", [0.0, 0.0, 0.0], 0.0, 1000.0),
	("smoothing_ms", [0.0, 0.0, 0.0], 0.0, 1000.0),
	("prediction_ms", [0.0, 9.0, 9.0], 0.0, 40.0),
	("velocity_boost", [0.0, 0.0, 0.0], 0.0, 200.0),
	("offset_x_mm", [0.0, 0.0, 0.0], -500.0, 500.0),
	("offset_y_mm", [0.0, 0.0, 0.0], -500.0, 500.0),
	("offset_z_mm", [0.0, 0.0, 0.0], -500.0, 500.0),
	("offset_rx_deg", [0.0, 0.0, 0.0], -180.0, 180.0),
	("offset_ry_deg", [0.0, 0.0, 0.0], -180.0, 180.0),
	("offset_rz_deg", [0.0, 0.0, 0.0], -180.0, 180.0),
	("rail", [0.0, 0.0, 0.0], 0.0, 1.0),
	("rail_speed", [1.5, 1.5, 1.5], 0.3, 10.0),
	("rail_boost", [50.0, 50.0, 50.0], 0.0, 500.0),
	("dash", [0.0, 0.0, 0.0], 0.0, 1.0),
	("dash_percent", [100.0, 100.0, 100.0], 0.0, 100.0),
	("dash_back_ms", [400.0, 400.0, 400.0], 50.0, 2000.0),
];
pub const CLASSES: [&str; 3] = ["headset", "controllers", "trackers"];

pub struct FakeDriver {
	/// the commands arrive here, the lines leave from it
	sock: UdpSocket,
	/// the panel's port for the driver's lines
	to: SocketAddrV4,
	/// what SteamVR is shown: "sally" or "valve"
	pub seen: String,
	/// the panel's choices: whose pose a lighthouse headset gets, how many sessions' files stay, whether the pose log
	/// is written
	pub headset_sally: bool,
	pub keep_sessions: i32,
	pub pose_log: bool,
	/// the output layer's values by (scope, name): every class's, and the devices' own
	pub output: BTreeMap<(String, String), f64>,
	/// every command the panel sent, in order
	pub commands: Vec<String>,
	/// the pairs a re-align has (150 and it is done)
	refit: Option<i32>,
	next_status: f64,
	frames: u64,
}

impl FakeDriver {
	/// A driver for `vc`, whose two ports are set to this one's (before the panel's first poll)
	pub fn new(vc: &mut ValveCompare) -> std::io::Result<FakeDriver> {
		let sock = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0))?;
		sock.set_nonblocking(true)?;
		// a port for the panel to bind: one that is free now
		let probe = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0))?;
		let panel_port = probe.local_addr()?.port();
		drop(probe);
		vc.driver_port = panel_port;
		vc.driver_cmd_port = sock.local_addr()?.port();
		let mut output = BTreeMap::new();
		for (name, defaults, _, _) in SETTINGS {
			for (class, v) in CLASSES.iter().zip(defaults) {
				output.insert((class.to_string(), name.to_string()), v);
			}
		}
		Ok(FakeDriver {
			sock,
			to: SocketAddrV4::new(Ipv4Addr::LOCALHOST, panel_port),
			seen: "sally".into(),
			headset_sally: true,
			keep_sessions: 2,
			pose_log: false,
			output,
			commands: Vec::new(),
			refit: None,
			next_status: 0.0,
			frames: 0,
		})
	}

	fn send(&self, line: &str) {
		let _ = self.sock.send_to(line.as_bytes(), self.to);
	}

	fn send_output(&self) {
		for ((scope, name), v) in &self.output {
			self.send(&format!("O {scope} {name} {v}"));
		}
		self.send("O end");
	}

	fn status(&self) {
		self.send(&format!(
			"C seen {} fit 1 median_mm 0.8 pairs 150 hooks 3 valve_devices 0 sally_devices 0 headset {} \
			 lighthouse_headset 0 keep_sessions {} pose_log {}",
			self.seen,
			if self.headset_sally { "sally" } else { "valve" },
			self.keep_sessions,
			if self.pose_log { "on" } else { "off" },
		));
	}

	/// Once a loop: the panel's commands answered, and the driver's lines for this frame
	pub fn poll(&mut self, now: f64, rt: &Runtime) {
		let mut buf = [0u8; 1024];
		while let Ok((n, _)) = self.sock.recv_from(&mut buf) {
			let cmd = String::from_utf8_lossy(&buf[..n]).trim_end_matches('\0').trim().to_string();
			self.commands.push(cmd.clone());
			let f: Vec<&str> = cmd.split_whitespace().collect();
			match f.as_slice() {
				["seen", who @ ("sally" | "valve")] => self.seen = who.to_string(),
				["refit"] => self.refit = Some(0),
				["headset", who @ ("sally" | "valve")] => self.headset_sally = *who == "sally",
				["keep", n] => self.keep_sessions = n.parse::<i32>().unwrap_or(0).max(0),
				["pose_log", to @ ("on" | "off")] => self.pose_log = *to == "on",
				["output"] => self.send_output(),
				["output", scope, name, value] => {
					let known = SETTINGS.iter().find(|s| s.0 == *name);
					match (known, *value, value.parse::<f64>()) {
						(Some(_), "-", _) if !CLASSES.contains(scope) => {
							self.output.remove(&(scope.to_string(), name.to_string()));
							self.send_output();
						}
						(Some(s), _, Ok(v)) if (s.2..=s.3).contains(&v) => {
							self.output.insert((scope.to_string(), name.to_string()), v);
							self.send_output();
						}
						(Some(_), _, _) => self.send(&format!("O error {name}: {value} is out of range")),
						(None, _, _) => self.send(&format!("O error no setting {name}")),
					}
				}
				_ => {}
			}
			self.status();
		}

		// Valve's pose of each device that poses, in the driver's space: the runtime's world in OpenVR's axes
		// ((x, y, z) -> (x, z, -y)), a little off
		self.frames += 1;
		let s = 0.5f64.sqrt();
		let yup = Q4::new(s, -s, 0.0, 0.0);
		for (k, d) in rt.devices.values().filter(|d| d.active && d.have_pose && d.serial != "-").enumerate() {
			let w = self.frames as f64 * 0.11 + k as f64 * 1.7;
			let (nx, ny, nz) = (0.0016 * w.sin(), 0.0012 * (w * 1.31).cos(), 0.0014 * (w * 0.77).sin());
			let (x, y, z) = (d.pos.x + nx, d.pos.z + nz, -d.pos.y + ny);
			let q = qmul(yup, d.rot);
			self.send(&format!(
				"V {} {now:.4} 1 {x:.6} {y:.6} {z:.6} {:.6} {:.6} {:.6} {:.6} {}",
				d.serial,
				q.w,
				q.x,
				q.y,
				q.z,
				(self.seen == "sally") as i32
			));
		}
		if now >= self.next_status {
			self.next_status = now + 0.5;
			self.status();
			if let Some(n) = &mut self.refit {
				*n += 40;
			}
			let (samples, refitting) = match self.refit {
				Some(n) if n < 150 => (n, 1),
				_ => (150, 0),
			};
			if refitting == 0 {
				self.refit = None;
			}
			// the fit and the frame's world-from-driver: both the identity
			self.send(&format!("F 1 {samples} {refitting} 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0"));
		}
	}
}
