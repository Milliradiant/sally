use std::net::UdpSocket;
use std::time::{Duration, Instant};

use sally_math::math3d::{inverse, norm, qrot};

use super::fake::{self, event, keyboard_event, mouse_event, scroll_event, FakeVr, SteamVr, MAIN, THUMB};
use super::*;
use crate::runtime::Event;

const DT: f64 = 1.0 / 30.0;

/// A rotation of `angle` (radians) about `axis`
fn qaxis(axis: V3, angle: f64) -> Q4 {
	let a = norm(axis);
	let s = (angle / 2.0).sin();
	Q4::new((angle / 2.0).cos(), a.x * s, a.y * s, a.z * s)
}

fn degrees(q: Q4) -> f64 {
	qangle(q).to_degrees()
}

/// A runtime with the devices (codename, serial), made the runtime's own way: a Q line each over its socket, from a
/// stand-in for the runtime (a Device has no public constructor). They are put in the tracking state; the tests move
/// them by writing their poses.
fn runtime_with(devices: &[(&str, &str)]) -> Runtime {
	let server = UdpSocket::bind("127.0.0.1:0").unwrap();
	server.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
	let mut rt = Runtime::new();
	rt.set_target("127.0.0.1", server.local_addr().unwrap().port());
	rt.poll(0.0);
	let mut buf = [0u8; 2048];
	let (_, client) = server.recv_from(&mut buf).expect("the runtime client subscribes");
	for (code, serial) in devices {
		let q = format!("Q {code} {serial} 1000 1.0 0 0 0 1 0 0 0 0 0 0 0 0 0\n");
		server.send_to(q.as_bytes(), client).unwrap();
	}
	let t0 = Instant::now();
	while devices.iter().any(|(code, _)| !rt.devices.contains_key(*code)) {
		assert!(t0.elapsed() < Duration::from_secs(5), "the runtime client got no Q lines");
		std::thread::sleep(Duration::from_millis(2));
		rt.poll(0.0);
	}
	for d in rt.devices.values_mut() {
		d.state = "tracking".into();
	}
	rt
}

/// The comparison against a fake SteamVR; the driver's socket on a port of the system's choosing, which nothing
/// reaches (a test of the compare mode sets its own)
fn compare() -> (ValveCompare, SteamVr) {
	let (vr, steamvr) = FakeVr::new();
	let mut vc = ValveCompare::with_vr(Box::new(vr));
	vc.driver_port = 0;
	(vc, steamvr)
}

/// A free UDP port on 127.0.0.1
fn free_port() -> u16 {
	UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

/// Where device `k` is at time t (sally's frame: metres, Z up): a slow figure through the play area, about 0.5 m/s,
/// turning as it goes
fn path(k: f64, t: f64) -> Pose {
	let p =
		V3::new(0.9 * (0.31 * t + k).sin(), 0.8 * (0.43 * t + 2.0 * k).sin(), 1.0 + 0.4 * (0.57 * t + 3.0 * k).sin());
	let axis = V3::new((0.2 * t + k).sin(), (0.3 * t).cos(), 1.0);
	Pose { p, q: qaxis(axis, 0.7 * (0.25 * t + k).sin() + k) }
}

/// The comparison with a runtime and a fake SteamVR that both see the same devices move
struct Bench {
	rt: Runtime,
	vc: ValveCompare,
	steamvr: SteamVr,
	/// (codename, serial, class)
	devices: Vec<(String, String, ETrackedDeviceClass)>,
	t: f64,
}

impl Bench {
	fn new(devices: &[(&str, &str, ETrackedDeviceClass)]) -> Bench {
		let pairs: Vec<(&str, &str)> = devices.iter().map(|&(c, s, _)| (c, s)).collect();
		let rt = runtime_with(&pairs);
		let (vc, steamvr) = compare();
		let devices: Vec<_> = devices.iter().map(|&(c, s, k)| (c.to_string(), s.to_string(), k)).collect();
		for (_, serial, class) in &devices {
			steamvr.lock().set_pose(serial, *class, None);
		}
		Bench { rt, vc, steamvr, devices, t: 0.0 }
	}

	/// Every device at its place on its path at time t, on both sides, ours tracking
	fn place(&mut self, t: f64) {
		for (k, (code, serial, class)) in self.devices.iter().enumerate() {
			let pose = path(k as f64, t);
			let d = self.rt.devices.get_mut(code).unwrap();
			d.pos = pose.p;
			d.rot = pose.q;
			d.have_pose = true;
			d.last_pose_wall = t;
			self.steamvr.lock().set_pose(serial, *class, Some(pose));
		}
	}

	fn step(&mut self) {
		self.t += DT;
		let t = self.t;
		self.place(t);
		self.vc.poll(t, &mut self.rt);
	}

	fn run(&mut self, seconds: f64) {
		let until = self.t + seconds;
		while self.t < until {
			self.step();
		}
	}

	/// The comparison's own log lines
	fn steamvr_events(&self) -> Vec<Event> {
		self.rt.events.iter().filter(|e| e.who == "SteamVR").cloned().collect()
	}
}

/// How far apart two transforms put a point
fn apart(a: &Pose, b: &Pose, at: V3) -> f64 {
	len(xform(a, at) - xform(b, at))
}

#[test]
fn the_fit_finds_the_fakes_frame_and_then_measures_a_few_mm() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-C1", TrackedDeviceClass_Controller),
	]);
	// Valve's frame of the tracker is not sally's: a constant rotation between the two, which is learnt
	let offset = qaxis(V3::new(1.0, 0.2, 0.0), 0.6);
	b.steamvr.lock().device("LHR-T1").rot_offset = offset;
	// a headset Valve's driver has (never compared)
	let head = Pose { p: V3::new(0.2, -0.4, 1.7), q: qaxis(V3::new(0.0, 0.0, 1.0), 0.3) };
	b.steamvr.lock().set_pose("LHR-H1", TrackedDeviceClass_HMD, Some(head));
	b.run(2.0);
	assert!(!b.vc.fit_ok);
	assert!(b.vc.status.starts_with("aligning to Valve's frame ("), "{}", b.vc.status);
	assert!(
		b.vc.status.contains("/100 samples from ") && b.vc.status.ends_with(" devices, move around)"),
		"{}",
		b.vc.status
	);
	b.run(30.0);
	let vc = &b.vc;
	assert!(vc.fit_ok);
	assert_eq!(vc.status, "comparing 2 devices with Valve's");
	assert_eq!(vc.compared, 2);
	// the fake's frame found: raw -> ours is the inverse of its ours -> raw
	let frame = b.steamvr.lock().frame;
	let truth = inverse(&frame);
	let mid = xform(&frame, V3::new(0.0, 0.0, 1.0));
	assert!(apart(&vc.fit, &truth, mid) < 0.001, "{:?} against {:?}", vc.fit, truth);
	assert!(degrees(qmul(qconj(vc.fit.q), truth.q)) < 0.05);
	// and then a few mm: 2 mm of noise on each axis is 3.1 mm at the median
	assert!(vc.fit_median > 0.002 && vc.fit_median < 0.004, "median {}", vc.fit_median);
	assert!(vc.fit_kept > 0 && vc.fit_kept <= vc.fit_n);
	for serial in ["LHR-T1", "LHR-C1"] {
		let d = &vc.devs[serial];
		assert!(d.have_ours && d.valid);
		assert!(d.err_med > 2.0 && d.err_med < 4.5, "{serial}: median {}", d.err_med);
		assert!(d.err_p90 > d.err_med && d.err_p90 < 7.0, "{serial}: p90 {}", d.err_p90);
		assert!(d.err_window.len() > 250 && d.err_window.len() <= 302, "{}", d.err_window.len());
		assert!(d.err_deg_ema >= 0.0 && d.err_deg_ema < 0.2, "{serial}: {} degrees", d.err_deg_ema);
		// the trail: the last trail_seconds of it, a point a poll (30 Hz is under 90)
		assert!(d.trail.len() > 170 && d.trail.len() <= 182, "{}", d.trail.len());
		assert!(b.t - d.trail.front().unwrap().t <= vc.trail_seconds);
	}
	assert_eq!(vc.devs["LHR-T1"].rot_offset_n, 300);
	assert!(degrees(qmul(qconj(vc.devs["LHR-T1"].rot_offset), offset)) < 0.1);
	assert!(degrees(vc.devs["LHR-C1"].rot_offset) < 0.1);
	// the headset in our frame (not a lighthouse device of the comparison)
	assert!(vc.have_hmd && !vc.devs.contains_key("LHR-H1"));
	assert!(len(vc.hmd_ours.p - head.p) < 0.01);
	// one line in the log for it
	let ev = b.steamvr_events();
	assert_eq!(ev.len(), 1, "{ev:?}");
	assert!(ev[0].text.starts_with("aligned Valve's frame to ours ("), "{}", ev[0].text);
	assert!(ev[0].text.ends_with(" mm)"), "{}", ev[0].text);
	let med: i32 = ev[0].text.split("median ").nth(1).unwrap().trim_end_matches(" mm)").parse().unwrap();
	assert!((2..=4).contains(&med), "{}", ev[0].text);
	assert_eq!((ev[0].level, ev[0].matters, ev[0].run_time), (EvLevel::Good, false, -1.0));

	// re-aligned on request: the pairs forgotten, then the same frame found again
	b.vc.refit();
	assert!(!b.vc.fit_ok && b.vc.devs["LHR-T1"].err_med < 0.0);
	b.run(6.0);
	assert!(b.vc.fit_ok);
	assert_eq!(b.steamvr_events().len(), 2);
	assert!(apart(&b.vc.fit, &truth, mid) < 0.002);
}

#[test]
fn the_chaperones_walls_are_put_in_sallys_frame_with_the_fit() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-C1", TrackedDeviceClass_Controller),
	]);
	b.run(2.0);
	// read, in OpenVR's raw universe; none in sally's frame before the fit
	assert_eq!(b.vc.walls_raw.len(), 4);
	assert!(b.vc.walls.is_empty());
	b.run(30.0);
	assert!(b.vc.fit_ok);
	// a wall's corner where the fake has it: its standing universe into the raw one, then out of the fake's frame
	let (frame, standing_from_raw, quad) = {
		let s = b.steamvr.lock();
		(s.frame, s.standing_from_raw, s.walls[1])
	};
	let c = quad.vCorners[2].v;
	let raw = xform(&inverse(&standing_from_raw), V3::new(c[0] as f64, c[1] as f64, c[2] as f64));
	let truth = xform(&inverse(&frame), raw);
	assert_eq!(b.vc.walls.len(), 4);
	assert!(len(b.vc.walls[1][2] - truth) < 0.005, "{:?} against {truth:?}", b.vc.walls[1][2]);
	// the floor corners at one height in sally's Z up, the top ones 2.4 m over them
	let w = &b.vc.walls[0];
	assert!((w[0].z - w[1].z).abs() < 0.005 && (w[3].z - w[0].z - 2.4).abs() < 0.005, "{w:?}");
	// a SteamVR without the interface: no walls
	b.steamvr.lock().missing.push(Interface::ChaperoneSetup);
	b.vc.shutdown();
	b.vc.next_try = 0.0;
	b.run(6.0);
	assert!(b.vc.running && b.vc.walls.is_empty() && b.vc.walls_raw.is_empty());
}

#[test]
fn locate_buzzes_a_controller_three_times() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-C1", TrackedDeviceClass_Controller),
	]);
	b.run(3.0);
	// a tracker has nothing to buzz with
	assert!(!b.vc.buzz("LHR-T1", b.t));
	assert!(b.vc.buzz("LHR-C1", b.t));
	b.run(1.5);
	let index = b.vc.devs["LHR-C1"].index as u32;
	let pulses = b.steamvr.lock().pulses.clone();
	assert!(pulses.iter().all(|&p| p == (index, 0, 3999)), "{pulses:?}");
	// a pulse a poll while each buzz is on: three of 0.15 s at 30 polls a second, 4 or 5 each
	assert!((12..=16).contains(&pulses.len()), "{}", pulses.len());
}

#[test]
fn a_moved_frame_is_logged_and_followed() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-T2", TrackedDeviceClass_GenericTracker),
	]);
	b.run(20.0);
	assert!(b.vc.fit_ok);
	// OpenVR's raw universe moves 20 cm (Valve's frame can sit off for ~15 s after SteamVR starts)
	b.steamvr.lock().frame.p.x += 0.2;
	b.run(70.0);
	let ev = b.steamvr_events();
	let moved: Vec<&Event> = ev.iter().filter(|e| e.text.starts_with("Valve's frame moved ")).collect();
	assert!(!moved.is_empty(), "{ev:?}");
	for e in &moved {
		assert!(e.text.ends_with(" mm against ours (re-aligned)"), "{}", e.text);
		assert_eq!((e.level, e.matters), (EvLevel::Warn, false));
	}
	// only the new frame's pairs are left in the minute's window: found
	let frame = b.steamvr.lock().frame;
	assert!(apart(&b.vc.fit, &inverse(&frame), xform(&frame, V3::new(0.0, 0.0, 1.0))) < 0.001);
	assert!(b.vc.fit_median < 0.004);
}

#[test]
fn tracker_roles_come_from_steamvrs_settings() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-T2", TrackedDeviceClass_GenericTracker),
		("KN2", "LHR-T3", TrackedDeviceClass_GenericTracker),
		("KN3", "LHR-C1", TrackedDeviceClass_Controller),
	]);
	{
		let mut s = b.steamvr.lock();
		s.set_tracker_role("LHR-T1", "TrackerRole_Waist");
		s.set_tracker_role("LHR-T2", "TrackerRole_None");
		// only a tracker's is read
		s.set_tracker_role("LHR-C1", "TrackerRole_LeftFoot");
	}
	b.step();
	let roles: Vec<(&str, &str)> = b.vc.roles_by_serial.iter().map(|(s, r)| (s.as_str(), r.as_str())).collect();
	assert_eq!(roles, [("LHR-T1", "TrackerRole_Waist")]);
	let role = |b: &Bench, s: &str| b.vc.devs[s].role.clone();
	assert_eq!(role(&b, "LHR-T1"), "TrackerRole_Waist");
	assert_eq!(role(&b, "LHR-T2"), "TrackerRole_None");
	assert_eq!(role(&b, "LHR-T3"), "");
	assert_eq!(role(&b, "LHR-C1"), "");
	// read again every 2 s
	b.steamvr.lock().set_tracker_role("LHR-T3", "TrackerRole_RightFoot");
	b.run(1.5);
	assert!(!b.vc.roles_by_serial.contains_key("LHR-T3"));
	b.run(1.0);
	assert_eq!(b.vc.roles_by_serial["LHR-T3"], "TrackerRole_RightFoot");
	assert_eq!(role(&b, "LHR-T3"), "TrackerRole_RightFoot");
}

#[test]
fn sallys_own_devices_are_never_compared() {
	let mut b = Bench::new(&[
		("KN0", "LHR-S1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-S2", TrackedDeviceClass_GenericTracker),
		("KN2", "LHR-Q1", TrackedDeviceClass_GenericTracker),
		("KN3", "LHR-V1", TrackedDeviceClass_GenericTracker),
	]);
	{
		let mut s = b.steamvr.lock();
		// the sally driver's, which it can call "lighthouse" (for Space Calibrator): by its resource root, by its
		// registered type
		s.device("LHR-S1").resource_root = "sally".into();
		s.device("LHR-S2").registered_type = "sally/LHR-S2".into();
		// another tracking system's
		s.device("LHR-Q1").system = "oculus".into();
	}
	b.run(20.0);
	assert_eq!(b.vc.devs.keys().collect::<Vec<_>>(), ["LHR-V1"]);
	assert_eq!(b.vc.compared, 1);
	assert!(b.vc.fit_ok);
	assert_eq!(b.vc.status, "comparing 1 devices with Valve's");
	// the last of Valve's turns out to be sally's (read again within 2 s): nothing to compare
	b.steamvr.lock().device("LHR-V1").registered_type = "sally/LHR-V1".into();
	b.run(2.5);
	assert_eq!(b.vc.compared, 0);
	assert!(!b.vc.devs["LHR-V1"].valid);
	assert_eq!(b.vc.status, "SteamVR connected, Valve's lighthouse driver has no devices");
}

#[test]
fn in_the_drivers_compare_mode_valves_poses_come_from_v_lines() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-C1", TrackedDeviceClass_Controller),
	]);
	let driver = UdpSocket::bind("127.0.0.1:0").unwrap();
	let cmd = UdpSocket::bind("127.0.0.1:0").unwrap();
	cmd.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
	let port = free_port();
	b.vc.driver_port = port;
	b.vc.driver_cmd_port = cmd.local_addr().unwrap().port();
	// SteamVR shows sally's poses on Valve's entries (here 15 mm of noise in the fake's frame); Valve's own arrive as
	// V lines, in another frame (Z up to Y up, -45 degrees of yaw, shifted) with 1 mm of noise (test_panel.py's)
	b.steamvr.lock().noise_m = 0.015;
	let s = 0.5f64.sqrt();
	let valve_frame = Pose {
		p: V3::new(-0.3, 0.1, 0.7),
		q: qmul(qaxis(V3::new(0.0, 1.0, 0.0), (-45.0f64).to_radians()), Q4::new(s, -s, 0.0, 0.0)),
	};
	let mut noise = fake::Gaussian::new(7);
	let send_v = |serial: &str, pose: &Pose, noise: &mut fake::Gaussian| -> V3 {
		let raw = compose(&valve_frame, pose);
		let p = raw.p + V3::new(noise.sample(), noise.sample(), noise.sample()) * 0.001;
		let line = format!(
			"V {serial} 1762500000.000000 1 {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} 1\n",
			p.x, p.y, p.z, raw.q.w, raw.q.x, raw.q.y, raw.q.z
		);
		driver.send_to(line.as_bytes(), ("127.0.0.1", port)).unwrap();
		p
	};
	// the panel's socket is open from the first poll
	b.step();
	assert!(!b.vc.hook_mode);
	let mut last = V3::default();
	for _ in 0..(40.0 / DT) as usize {
		b.t += DT;
		let t = b.t;
		b.place(t);
		for (k, serial) in ["LHR-T1", "LHR-C1"].iter().enumerate() {
			let p = send_v(serial, &path(k as f64, t), &mut noise);
			if k == 0 {
				last = p;
			}
		}
		b.vc.poll(t, &mut b.rt);
	}
	assert!(b.vc.hook_mode);
	// Valve's poses are the V lines' (to their printed digits), not OpenVR's
	assert!(len(b.vc.devs["LHR-T1"].raw.p - last) < 1e-5);
	assert!(b.vc.fit_ok);
	let truth = inverse(&valve_frame);
	let mid = xform(&valve_frame, V3::new(0.0, 0.0, 1.0));
	assert!(apart(&b.vc.fit, &truth, mid) < 0.0015, "{:?} against {:?}", b.vc.fit, truth);
	assert!(b.vc.fit_median < 0.003, "median {}", b.vc.fit_median);
	let ev = b.steamvr_events();
	assert!(ev[0].text.starts_with("aligned Valve's frame to ours ("), "{ev:?}");

	// the driver's state, and a command to it
	driver.send_to(b"C seen sally fit 1 median_mm 3.5\n", ("127.0.0.1", port)).unwrap();
	let t0 = Instant::now();
	while b.vc.hook_seen.is_empty() && t0.elapsed() < Duration::from_secs(5) {
		std::thread::sleep(Duration::from_millis(2));
		b.vc.poll(b.t, &mut b.rt);
	}
	assert_eq!((b.vc.hook_seen.as_str(), b.vc.hook_fit, b.vc.hook_fit_mm), ("sally", true, 3.5));
	b.vc.driver_cmd("seen valve");
	let mut buf = [0u8; 64];
	let (n, _) = cmd.recv_from(&mut buf).unwrap();
	assert_eq!(&buf[..n], b"seen valve");

	// the driver's fit (its F line): the comparison goes through it, not the panel's own. Its fit maps the runtime's
	// world in OpenVR's axes (Y up) into the V lines' space; here the scene's frame and 5 cm, so that its use shows
	let s = 0.5f64.sqrt();
	let ydown = Pose { p: V3::default(), q: Q4::new(s, s, 0.0, 0.0) };
	let t = compose(&Pose { p: V3::new(0.05, 0.0, 0.0), q: Q4::default() }, &compose(&valve_frame, &ydown));
	let wfd = Pose { p: V3::new(0.1, 0.2, 0.3), q: qaxis(V3::new(0.0, 1.0, 0.0), 0.4) };
	let f_line = |samples: i32, refitting: i32| {
		format!(
			"F 1 {samples} {refitting} {:.5} {:.5} {:.5} {:.6} {:.6} {:.6} {:.6} 1 {:.5} {:.5} {:.5} {:.6} {:.6} {:.6} {:.6}\n",
			t.p.x, t.p.y, t.p.z, t.q.w, t.q.x, t.q.y, t.q.z, wfd.p.x, wfd.p.y, wfd.p.z, wfd.q.w, wfd.q.x, wfd.q.y, wfd.q.z
		)
	};
	let steps = |b: &mut Bench, n: usize, noise: &mut fake::Gaussian| {
		for _ in 0..n {
			b.t += DT;
			let t = b.t;
			b.place(t);
			for (k, serial) in ["LHR-T1", "LHR-C1"].iter().enumerate() {
				send_v(serial, &path(k as f64, t), noise);
			}
			std::thread::sleep(Duration::from_millis(1));
			b.vc.poll(t, &mut b.rt);
		}
	};
	driver.send_to(f_line(412, 0).as_bytes(), ("127.0.0.1", port)).unwrap();
	steps(&mut b, 10, &mut noise);
	assert!(b.vc.fit_from_driver && b.vc.fit_ok);
	// not the panel's own any more (which found the scene's frame)
	assert!(apart(&b.vc.fit, &truth, mid) > 0.04);
	let want = compose(&ydown, &inverse(&t));
	assert!(len(b.vc.fit.p - want.p) < 1e-4 && degrees(qmul(qconj(b.vc.fit.q), want.q)) < 0.01, "{:?}", b.vc.fit);
	assert!((b.vc.fit_median - 0.0035).abs() < 1e-9 && b.vc.fit_n == 412);
	assert!(b.vc.status.ends_with("through the driver's alignment"), "{}", b.vc.status);
	// the 5 cm the driver's fit is off this scene shows in each device's comparison
	let e = b.vc.devs["LHR-T1"].err_mm;
	assert!(e > 40.0 && e < 60.0, "{e}");
	// re-align: asked of the driver, which says how far it is in its F lines; then done
	b.vc.refit();
	let (n, _) = cmd.recv_from(&mut buf).unwrap();
	assert_eq!(&buf[..n], b"refit");
	assert!(b.vc.fit_from_driver && b.vc.drv_fit.unwrap().refitting);
	// an F line the driver sent before it had the command says it is not re-aligning: still under way, and nothing
	// logged for it (it was "the driver re-aligned")
	let logged = b.steamvr_events().len();
	driver.send_to(f_line(412, 0).as_bytes(), ("127.0.0.1", port)).unwrap();
	steps(&mut b, 5, &mut noise);
	assert_eq!(b.vc.drv_fit.map(|f| (f.samples, f.refitting)), Some((412, true)));
	assert_eq!(b.steamvr_events().len(), logged);
	driver.send_to(f_line(37, 1).as_bytes(), ("127.0.0.1", port)).unwrap();
	steps(&mut b, 5, &mut noise);
	assert_eq!(b.vc.drv_fit.map(|f| (f.samples, f.refitting)), Some((37, true)));
	// a frame a step, through the driver's alignment since it came (20 steps)
	let t1 = &b.vc.devs["LHR-T1"];
	assert!(t1.err_window.len() >= 15 && t1.rot_offset_n >= 15, "{} {}", t1.err_window.len(), t1.rot_offset_n);
	driver.send_to(f_line(160, 0).as_bytes(), ("127.0.0.1", port)).unwrap();
	steps(&mut b, 5, &mut noise);
	let ev = b.steamvr_events();
	assert!(ev.iter().any(|e| e.text.starts_with("comparing through the driver's alignment")), "{ev:?}");
	assert!(ev.last().unwrap().text.starts_with("the driver re-aligned"), "{ev:?}");
	// what was measured through the old alignment went with it: the distances and the rotation offset start over
	let t1 = &b.vc.devs["LHR-T1"];
	assert!(t1.err_window.len() <= 5 && t1.rot_offset_n <= 5, "{} {}", t1.err_window.len(), t1.rot_offset_n);
	// a refit the driver never takes up (its command lost): its F lines go on saying it is not re-aligning, which is
	// taken as it is after a while, and no "re-aligned" is logged for it
	let logged = b.steamvr_events().len();
	b.vc.refit();
	let _ = cmd.recv_from(&mut buf);
	for _ in 0..5 {
		driver.send_to(f_line(160, 0).as_bytes(), ("127.0.0.1", port)).unwrap();
		steps(&mut b, 30, &mut noise);
	}
	assert!(!b.vc.drv_fit.unwrap().refitting);
	assert_eq!(b.steamvr_events().len(), logged);

	// a device whose V lines stop is not valid 0.1 s on
	for _ in 0..6 {
		b.t += DT;
		let t = b.t;
		b.place(t);
		send_v("LHR-T1", &path(0.0, t), &mut noise);
		b.vc.poll(t, &mut b.rt);
	}
	assert!(b.vc.hook_mode && b.vc.devs["LHR-T1"].valid && !b.vc.devs["LHR-C1"].valid);
	// no V lines for 2 s: OpenVR's poses again, and the panel's own alignment, anew
	b.run(2.5);
	assert!(!b.vc.hook_mode);
	assert!(!b.vc.fit_from_driver && !b.vc.fit_ok);
	assert!(b.vc.devs["LHR-C1"].valid);
	let frame = b.steamvr.lock().frame;
	let openvr = compose(&frame, &path(1.0, b.t));
	assert!(len(b.vc.devs["LHR-C1"].raw.p - openvr.p) < 0.1);
}

#[test]
fn a_device_no_longer_compared_shows_nothing_old() {
	let mut b = Bench::new(&[
		("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker),
		("KN1", "LHR-T2", TrackedDeviceClass_GenericTracker),
	]);
	b.run(30.0);
	assert!(b.vc.fit_ok && b.vc.devs["LHR-T2"].err_med > 0.0);
	// ours stops tracking it: no longer compared; its last 10 s still say something for 10 s, then nothing does (its
	// median and p90 stayed, as if they were now's)
	b.rt.devices.get_mut("KN1").unwrap().state = "dark".into();
	b.run(5.0);
	assert!(b.vc.devs["LHR-T2"].err_med > 0.0);
	b.run(6.0);
	let d = &b.vc.devs["LHR-T2"];
	assert!(d.err_window.is_empty() && d.err_med < 0.0 && d.err_p90 < 0.0, "{} {}", d.err_med, d.err_p90);
	assert!(b.vc.devs["LHR-T1"].err_med > 0.0);
}

#[test]
fn the_driver_ports_serials_are_kept_to_the_ones_it_is_sending() {
	let mut b = Bench::new(&[("KN0", "LHR-T1", TrackedDeviceClass_GenericTracker)]);
	let driver = UdpSocket::bind("127.0.0.1:0").unwrap();
	let port = free_port();
	b.vc.driver_port = port;
	b.step();
	// a hundred serials at once (anything on the machine can send to the port): as many as SteamVR has devices
	for k in 0..100 {
		let line = format!("V LHR-X{k:03} 1762500000.000000 1 0 0 1 1 0 0 0 1\n");
		driver.send_to(line.as_bytes(), ("127.0.0.1", port)).unwrap();
	}
	let t0 = Instant::now();
	while b.vc.driver_valve.len() < K_DRIVER_SERIALS && t0.elapsed() < Duration::from_secs(5) {
		std::thread::sleep(Duration::from_millis(2));
		b.step();
	}
	assert_eq!(b.vc.driver_valve.len(), K_DRIVER_SERIALS);
	assert!(b.vc.hook_mode && b.vc.devs.contains_key("LHR-X000"));
	// none heard for 10 s: forgotten, and Valve's devices made for them with them
	b.run(13.0);
	assert!(b.vc.driver_valve.is_empty() && b.vc.driver_valve_t.is_empty());
	assert!(b.vc.devs.keys().all(|s| !s.starts_with("LHR-X")), "{:?}", b.vc.devs.keys().collect::<Vec<_>>());
	assert!(b.vc.devs.contains_key("LHR-T1"));
}

/// The comparison with its overlay made, after a poll
fn with_overlay(visible: bool) -> (ValveCompare, SteamVr, Runtime) {
	let (mut vc, steamvr) = compare();
	steamvr.lock().overlay_visible = visible;
	let mut rt = Runtime::new();
	vc.poll(0.0, &mut rt);
	assert!(vc.overlay_ok);
	(vc, steamvr, rt)
}

fn ev(kind: OverlayEventKind, x: f32, y: f32, button: i32, down: bool, wheel: f32) -> OverlayEvent {
	OverlayEvent { kind, x, y, button, down, wheel }
}

#[test]
fn the_lasers_events_are_queued_as_steamvr_sent_them() {
	use OverlayEventKind::*;
	let (mut vc, steamvr, mut rt) = with_overlay(true);
	assert!(vc.overlay_visible);
	assert_eq!(vc.overlay_status, "visible");
	{
		let mut s = steamvr.lock();
		s.post_overlay_event(MAIN, mouse_event(VREvent_MouseMove, 100.0, 200.0, 0));
		s.post_overlay_event(MAIN, mouse_event(VREvent_MouseButtonDown, 110.0, 210.0, VRMouseButton_Left));
		s.post_overlay_event(MAIN, mouse_event(VREvent_MouseButtonUp, 111.0, 211.0, VRMouseButton_Right));
		s.post_overlay_event(MAIN, mouse_event(VREvent_MouseButtonDown, 112.0, 212.0, VRMouseButton_Middle));
		// a thumb resting on the stick
		s.post_overlay_event(MAIN, scroll_event(VREvent_ScrollSmooth, 0.3, 0.0));
		s.post_overlay_event(MAIN, scroll_event(VREvent_ScrollDiscrete, 0.0, -1.0));
		s.post_overlay_event(MAIN, scroll_event(VREvent_ScrollSmooth, 0.0, 0.25));
		s.post_overlay_event(MAIN, event(VREvent_FocusLeave));
	}
	vc.poll(0.1, &mut rt);
	assert_eq!(
		vc.ov_events,
		[
			ev(Move, 100.0, 200.0, 0, false, 0.0),
			ev(Move, 110.0, 210.0, 0, false, 0.0),
			ev(Button, 110.0, 210.0, 0, true, 0.0),
			ev(Move, 111.0, 211.0, 0, false, 0.0),
			ev(Button, 111.0, 211.0, 1, false, 0.0),
			ev(Move, 112.0, 212.0, 0, false, 0.0),
			ev(Button, 112.0, 212.0, 2, true, 0.0),
			ev(Wheel, 0.0, 0.0, 0, false, -1.0),
			ev(Wheel, 0.0, 0.0, 0, false, 0.25),
			ev(Leave, 0.0, 0.0, 0, false, 0.0),
		]
	);
	// the host takes them; the pointer's y counted from the bottom when the cfg says so
	vc.ov_events.clear();
	vc.overlay_flip_pointer = true;
	{
		let mut s = steamvr.lock();
		s.post_overlay_event(MAIN, mouse_event(VREvent_MouseMove, 10.0, 100.0, 0));
		s.post_overlay_event(MAIN, mouse_event(VREvent_MouseButtonUp, 10.0, 175.0, VRMouseButton_Left));
	}
	vc.poll(0.2, &mut rt);
	assert_eq!(
		vc.ov_events,
		[
			ev(Move, 10.0, 1075.0, 0, false, 0.0),
			ev(Move, 10.0, 1000.0, 0, false, 0.0),
			ev(Button, 10.0, 1000.0, 0, false, 0.0)
		]
	);
	// at most 512 wait for the host
	vc.ov_events.clear();
	vc.overlay_flip_pointer = false;
	for i in 0..600 {
		steamvr.lock().post_overlay_event(MAIN, mouse_event(VREvent_MouseMove, i as f32, 5.0, 0));
	}
	vc.poll(0.3, &mut rt);
	assert_eq!(vc.ov_events.len(), 512);
	assert!(vc.ov_events.iter().enumerate().all(|(i, e)| e.kind == Move && e.x == i as f32));
	assert!(steamvr.lock().overlay_events.is_empty());
	// hidden: what is queued goes, and the pointer is let go of
	steamvr.lock().overlay_visible = false;
	steamvr.lock().post_overlay_event(MAIN, mouse_event(VREvent_MouseMove, 1.0, 2.0, 0));
	vc.poll(0.4, &mut rt);
	assert!(!vc.overlay_visible);
	assert_eq!(vc.ov_events, [ev(Leave, 0.0, 0.0, 0, false, 0.0)]);
	assert_eq!(vc.overlay_status, "in the dashboard");
	// the button's visibility
	steamvr.lock().thumb_visible = true;
	vc.poll(0.5, &mut rt);
	assert!(vc.thumb_visible && !vc.overlay_visible);
}

#[test]
fn the_overlay_is_made_for_the_dashboard_and_made_again() {
	let (mut vc, steamvr, mut rt) = with_overlay(false);
	assert_eq!(
		steamvr.lock().calls,
		[
			"init background",
			"shutdown",
			"init overlay",
			"interface IVRSystem_026",
			"interface IVROverlay_028",
			"interface IVRSettings_003",
			"interface IVRChaperoneSetup_006",
			"create sally.panel sally",
			"width 1 2.5",
			"input method 1 1",
			"mouse scale 1 2000 1175",
			"flag 1 131072 true",
			"flag 2 2097152 true",
		]
	);
	assert_eq!((vc.overlay_gen, vc.overlay_status.as_str()), (1, "in the dashboard"));
	// its width follows the settings
	vc.overlay_set_width(3.0);
	assert_eq!(steamvr.lock().calls.last().unwrap(), "width 1 3");
	// switched off it goes (the Overlay session stays); on again it is made again
	vc.overlay_wanted = false;
	vc.poll(1.0, &mut rt);
	assert!(!vc.overlay_ok && vc.running);
	assert_eq!(vc.overlay_status, "off");
	assert_eq!(steamvr.lock().calls.last().unwrap(), "destroy 1");
	vc.overlay_wanted = true;
	vc.poll(2.0, &mut rt);
	assert!(vc.overlay_ok);
	assert_eq!(vc.overlay_gen, 2);
	assert!(steamvr.lock().calls.iter().any(|c| c == "width 1 3"));
	// one that cannot be made is tried again every 3 s
	let creates = |s: &SteamVr| s.lock().calls.iter().filter(|c| c.starts_with("create ")).count();
	steamvr.lock().create_error = VROverlayError_KeyInUse;
	vc.overlay_wanted = false;
	vc.poll(3.0, &mut rt);
	vc.overlay_wanted = true;
	let n = creates(&steamvr);
	vc.poll(4.9, &mut rt);
	assert_eq!(creates(&steamvr), n);
	vc.poll(5.0, &mut rt);
	assert_eq!(creates(&steamvr), n + 1);
	assert_eq!(vc.overlay_status, "create error 17 (key in use: another panel running?)");
	vc.poll(7.9, &mut rt);
	assert_eq!(creates(&steamvr), n + 1);
	vc.poll(8.0, &mut rt);
	assert_eq!(creates(&steamvr), n + 2);
	steamvr.lock().create_error = VROverlayError_None;
	vc.poll(11.0, &mut rt);
	assert!(vc.overlay_ok);
	assert_eq!((vc.overlay_gen, vc.overlay_status.as_str()), (3, "in the dashboard"));
}

#[test]
fn textures_go_to_steamvr_as_gl_textures_in_gamma() {
	let (mut vc, steamvr, mut rt) = with_overlay(true);
	vc.overlay_submit(7);
	vc.overlay_submit_thumb(8);
	let handed = steamvr.lock().textures.clone();
	let gl = |overlay, gl_texture| fake::HandedTexture {
		overlay,
		gl_texture,
		texture_type: TextureType_OpenGL,
		color_space: ColorSpace_Gamma,
	};
	assert_eq!(handed, [gl(MAIN, 7), gl(THUMB, 8)]);
	assert_eq!(vc.overlay_error, "");
	// a failure shows in the status until a hand-over works
	steamvr.lock().texture_error = 11;
	vc.overlay_submit(7);
	assert_eq!(vc.overlay_error, "texture error 11");
	vc.poll(0.1, &mut rt);
	assert_eq!(vc.overlay_status, "texture error 11");
	vc.overlay_submit_thumb(8);
	assert_eq!(vc.overlay_error, "button texture error 11");
	steamvr.lock().texture_error = VROverlayError_None;
	vc.overlay_submit(7);
	vc.poll(0.2, &mut rt);
	assert_eq!(vc.overlay_status, "visible");
	// nothing is handed over without the overlay
	vc.overlay_wanted = false;
	vc.poll(0.3, &mut rt);
	let n = steamvr.lock().textures.len();
	vc.overlay_submit(7);
	vc.overlay_submit_thumb(8);
	assert_eq!(steamvr.lock().textures.len(), n);
}

#[test]
fn the_keyboards_text_is_taken_once_whichever_queue_its_done_comes_on() {
	let (mut vc, steamvr, mut rt) = with_overlay(true);
	steamvr.lock().keyboard_on_system_queue = true;
	assert!(vc.overlay_keyboard("LHR-T1", "Name for KN0", "Tracker 3545"));
	assert!(vc.kb_open && !vc.kb_done);
	assert_eq!((vc.kb_tag.as_str(), vc.kb_error.as_str()), ("LHR-T1", ""));
	let asked = KeyboardRequest {
		input_mode: k_EGamepadTextInputModeNormal,
		line_mode: k_EGamepadTextInputLineModeSingleLine,
		flags: KeyboardFlag_Modal,
		description: b"Name for KN0".to_vec(),
		char_max: 32,
		existing_text: b"Tracker 3545".to_vec(),
		user_value: 1,
	};
	assert_eq!(steamvr.lock().keyboard_requests, [(MAIN, asked)]);
	// its Done on both queues: taken at the first, once
	vc.poll(0.1, &mut rt);
	assert!(vc.kb_done && !vc.kb_open);
	assert_eq!(vc.kb_text, "Bench tracker");
	assert_eq!(steamvr.lock().keyboard_reads, 1);
	// the panel takes the name; nothing comes again
	vc.kb_done = false;
	vc.poll(0.2, &mut rt);
	assert!(!vc.kb_done);
	assert_eq!(steamvr.lock().keyboard_reads, 1);
	// texts as long as the C++'s 128-byte buffers take
	let long = "x".repeat(200);
	assert!(vc.overlay_keyboard("LHR-T1", &long, &long));
	let (_, r) = steamvr.lock().keyboard_requests[1].clone();
	assert_eq!((r.description.len(), r.existing_text.len(), r.user_value), (127, 127, 2));
}

#[test]
fn a_done_for_an_older_request_is_ignored() {
	let (mut vc, steamvr, mut rt) = with_overlay(true);
	steamvr.lock().keyboard_answers = false;
	assert!(vc.overlay_keyboard("LHR-A", "Name for KN0", "a"));
	assert!(vc.overlay_keyboard("LHR-B", "Name for KN1", "b"));
	steamvr.lock().keyboard_text = "Left shin".into();
	steamvr.lock().post_overlay_event(MAIN, keyboard_event(VREvent_KeyboardDone, 1, MAIN));
	vc.poll(0.1, &mut rt);
	assert!(!vc.kb_done && vc.kb_open);
	assert_eq!(steamvr.lock().keyboard_reads, 0);
	steamvr.lock().post_overlay_event(MAIN, keyboard_event(VREvent_KeyboardDone, 2, MAIN));
	vc.poll(0.2, &mut rt);
	assert!(vc.kb_done && !vc.kb_open);
	assert_eq!((vc.kb_tag.as_str(), vc.kb_text.as_str()), ("LHR-B", "Left shin"));
	// a Done without its request's number counts for the request open; on the button's queue too
	vc.kb_done = false;
	assert!(vc.overlay_keyboard("LHR-C", "Name for KN2", "c"));
	steamvr.lock().post_overlay_event(THUMB, keyboard_event(VREvent_KeyboardDone, 0, THUMB));
	vc.poll(0.3, &mut rt);
	assert!(vc.kb_done);
	assert_eq!(vc.kb_tag, "LHR-C");
	// closed without a Done: cancelled
	vc.kb_done = false;
	assert!(vc.overlay_keyboard("LHR-D", "Name for KN3", "d"));
	steamvr.lock().post_system_event(keyboard_event(VREvent_KeyboardClosed_Global, 4, MAIN));
	vc.poll(0.4, &mut rt);
	assert!(!vc.kb_open && !vc.kb_done);
	assert_eq!(steamvr.lock().keyboard_reads, 2);
}

#[test]
fn a_keyboard_left_up_is_taken_over_and_failures_say_why() {
	let (mut vc, steamvr) = compare();
	let mut rt = Runtime::new();
	assert!(!vc.overlay_keyboard("LHR-A", "Name for KN0", "a"));
	assert_eq!(vc.kb_error, "no SteamVR overlay");
	vc.poll(0.0, &mut rt);
	// still up from an earlier request whose Close never came: hidden, and shown again
	steamvr.lock().keyboard_in_use = true;
	assert!(vc.overlay_keyboard("LHR-A", "Name for KN0", "a"));
	assert_eq!(vc.kb_error, "");
	let calls: Vec<String> = steamvr.lock().calls.iter().rev().take(3).rev().cloned().collect();
	assert_eq!(calls, ["keyboard 1 #1", "hide keyboard", "keyboard 1 #1"]);
	vc.poll(0.1, &mut rt);
	assert!(vc.kb_done);
	// in use for good
	steamvr.lock().keyboard_error = VROverlayError_KeyboardAlreadyInUse;
	assert!(!vc.overlay_keyboard("LHR-B", "Name for KN1", "b"));
	assert_eq!(vc.kb_error, "keyboard in use");
	// another failure (not retried)
	steamvr.lock().keyboard_error = VROverlayError_RequestFailed;
	let n = steamvr.lock().keyboard_requests.len();
	assert!(!vc.overlay_keyboard("LHR-B", "Name for KN1", "b"));
	assert_eq!(vc.kb_error, "keyboard error 23");
	assert_eq!(steamvr.lock().keyboard_requests.len(), n + 1);
	// a failed request takes no number: the next is #2
	steamvr.lock().keyboard_error = VROverlayError_None;
	assert!(vc.overlay_keyboard("LHR-B", "Name for KN1", "b"));
	assert_eq!(steamvr.lock().keyboard_requests.last().unwrap().1.user_value, 2);
}

#[test]
fn steamvrs_quit_is_acknowledged_and_it_is_tried_again_after_10_s() {
	let (mut vc, steamvr, mut rt) = with_overlay(true);
	assert!(vc.running);
	steamvr.lock().post_system_event(event(VREvent_Quit));
	vc.poll(1.0, &mut rt);
	assert!(!vc.running && !vc.overlay_ok);
	assert_eq!(vc.status, "SteamVR exited");
	assert_eq!(vc.overlay_status, "SteamVR not running");
	assert_eq!(steamvr.lock().quits_acknowledged, 1);
	let tail: Vec<String> = steamvr.lock().calls.iter().rev().take(3).rev().cloned().collect();
	assert_eq!(tail, ["acknowledge quit", "destroy 1", "shutdown"]);
	// SteamVR gone: tried again 10 s after it left, then every 3 s
	steamvr.lock().running = false;
	let inits = |s: &SteamVr| s.lock().calls.iter().filter(|c| *c == "init background").count();
	let n = inits(&steamvr);
	vc.poll(5.0, &mut rt);
	vc.poll(10.9, &mut rt);
	assert_eq!(inits(&steamvr), n);
	assert_eq!(vc.status, "SteamVR exited");
	vc.poll(11.0, &mut rt);
	assert_eq!(inits(&steamvr), n + 1);
	assert_eq!((vc.status.as_str(), vc.overlay_status.as_str()), ("SteamVR not running", "SteamVR not running"));
	vc.poll(13.9, &mut rt);
	assert_eq!(inits(&steamvr), n + 1);
	vc.poll(14.0, &mut rt);
	assert_eq!(inits(&steamvr), n + 2);
	// back: attached again, with the overlay
	steamvr.lock().running = true;
	vc.poll(17.0, &mut rt);
	assert!(vc.running && vc.overlay_ok);
	assert_eq!(vc.overlay_gen, 2);
}

#[test]
fn it_attaches_as_a_background_app_and_comes_back_as_an_overlay_app() {
	// no overlay wanted: a Background session and nothing more
	let (mut vc, steamvr) = compare();
	vc.overlay_wanted = false;
	let mut rt = Runtime::new();
	vc.poll(0.0, &mut rt);
	assert!(vc.running);
	assert_eq!(
		steamvr.lock().calls,
		["init background", "interface IVRSystem_026", "interface IVRSettings_003", "interface IVRChaperoneSetup_006"]
	);
	assert_eq!(steamvr.lock().session, Some(VRApplication_Background));
	assert_eq!(vc.overlay_status, "off");
	assert_eq!(vc.status, "SteamVR connected, Valve's lighthouse driver has no devices");
	// the overlay turned on: shut down and back, a Background app first again (it never starts SteamVR)
	steamvr.lock().calls.clear();
	vc.overlay_wanted = true;
	vc.poll(1.0, &mut rt);
	let calls = steamvr.lock().calls.clone();
	assert_eq!(calls[..4], ["shutdown", "init background", "shutdown", "init overlay"]);
	assert_eq!(steamvr.lock().session, Some(VRApplication_Overlay));
	assert!(vc.overlay_ok);
	// SteamVR not there: nothing but the Background app's attempt
	let (mut vc, steamvr) = compare();
	steamvr.lock().running = false;
	vc.poll(0.0, &mut rt);
	assert_eq!(steamvr.lock().calls, ["init background"]);
	assert!(!vc.running);
}

#[test]
fn failures_to_attach_say_why_and_are_tried_again() {
	let (mut vc, steamvr) = compare();
	let mut rt = Runtime::new();
	steamvr.lock().library = fake::Library::Missing;
	vc.poll(0.0, &mut rt);
	assert_eq!(vc.status, "openvr_api library not found next to the panel");
	assert_eq!(vc.overlay_status, "no openvr_api library");
	// every 3 s
	vc.overlay_wanted = false;
	vc.poll(2.9, &mut rt);
	assert_eq!(vc.overlay_status, "off");
	vc.poll(3.0, &mut rt);
	assert_eq!(vc.overlay_status, "off");
	steamvr.lock().library = fake::Library::Unusable;
	vc.poll(6.0, &mut rt);
	assert_eq!(vc.status, "openvr_api library is not usable");
	steamvr.lock().library = fake::Library::Usable;
	steamvr.lock().running = false;
	vc.overlay_wanted = true;
	vc.poll(9.0, &mut rt);
	assert_eq!((vc.status.as_str(), vc.overlay_status.as_str()), ("SteamVR not running", "SteamVR not running"));
	steamvr.lock().running = true;
	steamvr.lock().init_error = Some(108);
	vc.poll(12.0, &mut rt);
	assert_eq!((vc.status.as_str(), vc.overlay_status.as_str()), ("SteamVR init error 108", "SteamVR init error 108"));
	steamvr.lock().init_error = None;
	steamvr.lock().overlay_init_error = Some(110);
	vc.poll(15.0, &mut rt);
	assert!(!vc.running);
	assert_eq!((vc.status.as_str(), vc.overlay_status.as_str()), ("SteamVR overlay init error 110", "init error 110"));
	// a SteamVR without IVRSystem: let go, and tried again only 15 s on
	steamvr.lock().overlay_init_error = None;
	steamvr.lock().missing = vec![Interface::System];
	vc.poll(18.0, &mut rt);
	assert!(!vc.running);
	assert_eq!(vc.status, "SteamVR has no IVRSystem_026");
	assert_eq!(steamvr.lock().calls.last().unwrap(), "shutdown");
	steamvr.lock().missing.clear();
	vc.poll(32.9, &mut rt);
	assert!(!vc.running);
	vc.poll(33.0, &mut rt);
	assert!(vc.running && vc.overlay_ok);
	// one without IVROverlay: the comparison runs, the overlay says why it does not
	let (mut vc, steamvr) = compare();
	steamvr.lock().missing = vec![Interface::Overlay];
	vc.poll(0.0, &mut rt);
	assert!(vc.running && !vc.overlay_ok);
	assert_eq!(vc.overlay_status, "SteamVR has no IVROverlay_028 (error 105)");
	assert!(!vc.overlay_keyboard("LHR-A", "Name for KN0", "a"));
}

#[test]
fn with_the_comparison_and_the_overlay_off_steamvr_is_let_go() {
	let (mut vc, steamvr, mut rt) = with_overlay(true);
	vc.enabled = false;
	vc.poll(1.0, &mut rt);
	// the overlay keeps the session
	assert!(vc.running && vc.overlay_ok);
	vc.overlay_wanted = false;
	vc.poll(2.0, &mut rt);
	assert!(!vc.running);
	assert_eq!(vc.status, "comparison off");
	let tail: Vec<String> = steamvr.lock().calls.iter().rev().take(2).rev().cloned().collect();
	assert_eq!(tail, ["destroy 1", "shutdown"]);
	let n = steamvr.lock().calls.len();
	vc.poll(10.0, &mut rt);
	assert_eq!(steamvr.lock().calls.len(), n);
	// dropped, the panel lets go of a session it holds
	let (vc, steamvr, _) = with_overlay(true);
	drop(vc);
	let tail: Vec<String> = steamvr.lock().calls.iter().rev().take(2).rev().cloned().collect();
	assert_eq!(tail, ["destroy 1", "shutdown"]);
}

#[test]
fn the_fakes_follow_a_runtimes_devices() {
	let mut rt = runtime_with(&[("KN0", "LHR-T1"), ("KN1", "LHR-C1")]);
	rt.devices.get_mut("KN1").unwrap().subtype = "knuckles_r".into();
	let (_, steamvr) = FakeVr::new();
	steamvr.lock().follow(&rt);
	let s = steamvr.lock();
	assert_eq!(s.devices.len(), 2);
	assert_eq!((s.devices[0].serial.as_str(), s.devices[0].class), ("LHR-T1", TrackedDeviceClass_GenericTracker));
	assert_eq!((s.devices[1].serial.as_str(), s.devices[1].class), ("LHR-C1", TrackedDeviceClass_Controller));
	assert_eq!(s.devices[0].pose, Some(Pose::default()));
	// the C++ fake's frame turns Z up to Y up
	let up = qrot(s.frame.q, V3::new(0.0, 0.0, 1.0));
	assert!(len(up - V3::new(0.0, 1.0, 0.0)) < 1e-12);
}
