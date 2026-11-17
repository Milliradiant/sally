//! The driver's hand-over with the angular acceleration (Update, Substitute and the Q line parser): the rate handed to
//! SteamVR and the orientation, controllers and trackers, with and without the Q line's last three fields, the clamp, P
//! lines, a nan. One test: the driver's state is its globals.

use driver_sally::helpers::now_seconds;
use driver_sally::openvr::DriverPose_t;
use driver_sally::output::Shown;
use driver_sally::pose_override::{substitute, SallyLatest, ValveDev};
use driver_sally::provider::{parse_pose, G_PROVIDER};
use driver_sally::runtime_device::{Kind, Report, RuntimeDevice};
use driver_sally::settings::{g_opt, set_g_opt, Options};
use sally_math::math3d::{qmul, Pose, Q4, V3};

struct Checks {
	fails: i32,
}

impl Checks {
	fn check(&mut self, ok: bool, what: &str, got: f64, want: f64) {
		println!("{} {:<62} got {:+.5} want {:+.5}", if ok { "PASS" } else { "FAIL" }, what, got, want);
		self.fails += if ok { 0 } else { 1 };
	}
}

fn near(a: f64, b: f64, tol: f64) -> bool {
	(a - b).abs() <= tol
}

/// One report into a device, then the override's hand-over of it as of now; the pose handed over and the one stored
fn handover(
	dev: &RuntimeDevice, q: &[f64; 23], al: Option<&[f64; 3]>, t_recv_ago: f64,
) -> (DriverPose_t, DriverPose_t) {
	update(dev, q, true, al);
	let stored = dev.state().pose;
	let s = SallyLatest {
		have: true,
		lost: false,
		t_recv: now_seconds() - t_recv_ago,
		pose: stored,
		shown: Shown::default(),
	};
	let mut valve = DriverPose_t::default();
	valve.qRotation.w = 1.0;
	valve.qWorldFromDriverRotation.w = 1.0;
	valve.qDriverFromHeadRotation.w = 1.0;
	let mut d = ValveDev::default();
	let mut o = DriverPose_t::default();
	let t = Pose { p: V3::new(0.0, 0.0, 0.0), q: Q4::new(1.0, 0.0, 0.0, 0.0) };
	substitute(&g_opt(), &t, &s, &valve, None, now_seconds(), &mut o, &mut d);
	(o, stored)
}

/// dev.Update(q, q + 3, q + 7, q + 10, has_vel, has_vel ? q + 13 : nullptr, al)
fn update(dev: &RuntimeDevice, q: &[f64; 23], has_vel: bool, al: Option<&[f64; 3]>) {
	let (pos, rot, vel, angvel) =
		([q[0], q[1], q[2]], [q[3], q[4], q[5], q[6]], [q[7], q[8], q[9]], [q[10], q[11], q[12]]);
	let ho: [f64; 10] = std::array::from_fn(|i| q[13 + i]);
	dev.update(&Report {
		pos: &pos,
		rot: &rot,
		vel: &vel,
		angvel: &angvel,
		has_vel,
		ho: if has_vel { Some(&ho) } else { None },
		al,
		corr: None,
	});
}

#[test]
fn test_handover() {
	let mut c = Checks { fails: 0 };
	set_g_opt(Options { carry: true, stats: false, ..Options::default() }); // the pose 9 ms old: prediction_ms as it stands
	let h2i = [0.0, 0.0, -0.16, 1.0, 0.0, 0.0, 0.0]; // IMU -> origin 16 cm along -z (device frame)
												  // a report: head at the origin, identity orientation (Z-up), turning about world z at 10 rad/s, decelerating at 500
												  // rad/s^2; look-ahead lead 2.5 ms, nothing else
	let q: [f64; 23] = [
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0,
	];
	let al = [0.0, 0.0, -500.0];
	// world z (Z-up) is the device's z here (identity orientation); the Z-up -> Y-up change does not touch the device
	// frame
	{
		let dev = RuntimeDevice::new(b"LHR-R", Kind::RightController);
		dev.set_device_info(&h2i);
		let (o, st) = handover(&dev, &q, Some(&al), 0.002);
		let want = 10.0 - 500.0 * 0.0025;
		c.check(
			near(st.vecAngularVelocity[2], want, 1e-6),
			"controller: rate at the lead's end (body z)",
			st.vecAngularVelocity[2],
			want,
		);
		c.check(
			near(st.vecAngularAcceleration[2], -500.0, 1e-6),
			"controller: angular acceleration kept for the carry (body z)",
			st.vecAngularAcceleration[2],
			-500.0,
		);
		let age = 0.009 - 0.0025 + 0.002; // declared age + the time since the report (fresh)
		let want = 10.0 - 500.0 * (0.0025 + age);
		c.check(
			near(o.vecAngularVelocity[2], want, 2e-3),
			"controller: rate handed to SteamVR, w + alpha (lead + age)",
			o.vecAngularVelocity[2],
			want,
		);
		c.check(
			o.vecAngularAcceleration[2] == 0.0 && o.vecAcceleration[2] == 0.0,
			"controller: SteamVR gets no accelerations",
			o.vecAngularAcceleration[2],
			0.0,
		);
		// orientation: turned about z by w T - alpha T^2 / 2, T = lead + age; o.qRotation is Y-up: Rx(-90) q, so compare
		// the angle about the device's z through q_dev = Rx(+90) o.q
		let t = 0.0025 + age;
		let want = 10.0 * t - 0.5 * 500.0 * t * t;
		let s2 = 0.5f64.sqrt();
		let oy = Q4::new(o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z);
		let qd = qmul(Q4::new(s2, s2, 0.0, 0.0), oy);
		let got = 2.0 * qd.z.atan2(qd.w);
		c.check(near(got, want, 2e-4), "controller: turn over the hand-over w T + alpha T^2 / 2 (rad)", got, want);
		c.check(near(o.poseTimeOffset, 0.0, 1e-9), "controller: the pose declared current", o.poseTimeOffset, 0.0);
	}
	{
		let dev = RuntimeDevice::new(b"LHR-T", Kind::Tracker);
		dev.set_device_info(&h2i);
		let (o, st) = handover(&dev, &q, Some(&al), 0.002);
		c.check(
			near(o.vecAngularVelocity[2], 10.0, 1e-6),
			"tracker: the rate as before (no angular acceleration)",
			o.vecAngularVelocity[2],
			10.0,
		);
		c.check(
			near(st.vecAngularAcceleration[2], 0.0, 1e-12),
			"tracker: nothing kept for the carry",
			st.vecAngularAcceleration[2],
			0.0,
		);
	}
	{
		let dev = RuntimeDevice::new(b"LHR-O", Kind::RightController);
		dev.set_device_info(&h2i);
		let (o, _) = handover(&dev, &q, None, 0.002); // a runtime that sends no angular acceleration: 28 fields
		c.check(
			near(o.vecAngularVelocity[2], 10.0, 1e-6),
			"older runtime (no alpha): the rate as 0.8.1",
			o.vecAngularVelocity[2],
			10.0,
		);
	}
	{
		// the clamp: 5000 rad/s^2 against 10 rad/s would turn it back within the hand-over: it stops at 0 instead
		let big = [0.0, 0.0, -5000.0];
		let dev = RuntimeDevice::new(b"LHR-C", Kind::LeftController);
		dev.set_device_info(&h2i);
		let (o, st) = handover(&dev, &q, Some(&big), 0.002);
		c.check(
			st.vecAngularVelocity[2] > 0.0 && o.vecAngularVelocity[2] > -1e-6 && o.vecAngularVelocity[2] < 0.5,
			"clamp: the rate slows to about 0, does not reverse",
			o.vecAngularVelocity[2],
			0.0,
		);
		let spin_up = [0.0, 0.0, 5000.0]; // the same size the other way: at most doubled
		let (o2, _) = handover(&dev, &q, Some(&spin_up), 0.002);
		// bounded over the lead and the declared age (9 ms); the 2 ms since the report come on top: 10 (1 + 11 / 9), and
		// the time the test takes from the report to the hand-over (1.1 rad/s per ms: 22.22325 against 22.22322 once,
		// with the workspace's other tests running)
		c.check(
			o2.vecAngularVelocity[2] <= 10.0 * (1.0 + 11.0 / 9.0) + 0.05 && o2.vecAngularVelocity[2] > 15.0,
			"clamp: a spin-up grows by about the rate over the hand-over",
			o2.vecAngularVelocity[2],
			10.0 * (1.0 + 11.0 / 9.0),
		);
	}
	{
		// position and velocity: unchanged by alpha (the runtime folds alpha x r; the driver uses the state's rate for
		// the lever)
		let a = RuntimeDevice::new(b"LHR-A", Kind::RightController);
		let b = RuntimeDevice::new(b"LHR-B", Kind::RightController);
		a.set_device_info(&h2i);
		b.set_device_info(&h2i);
		let (_, sa) = handover(&a, &q, Some(&al), 0.0);
		let (_, sb) = handover(&b, &q, None, 0.0);
		let (mut dp, mut dv) = (0.0f64, 0.0f64);
		for i in 0..3 {
			dp = dp.max((sa.vecPosition[i] - sb.vecPosition[i]).abs());
			dv = dv.max((sa.vecVelocity[i] - sb.vecVelocity[i]).abs());
		}
		c.check(
			dp < 1e-12 && dv < 1e-12,
			"the head point's stored position / velocity the same with and without alpha",
			dp + dv,
			0.0,
		);
	}
	{
		// a P line (no velocity): nothing carried, nothing kept
		let dev = RuntimeDevice::new(b"LHR-P", Kind::RightController);
		dev.set_device_info(&h2i);
		update(&dev, &q, false, None);
		let st = dev.state().pose;
		c.check(
			st.vecAngularAcceleration[2] == 0.0,
			"P line: no angular acceleration kept",
			st.vecAngularAcceleration[2],
			0.0,
		);
	}
	{
		// the protocol lines through the provider's parser: a DEV line, then Q lines with 31, 28 and 18 fields
		G_PROVIDER.handle_line(
			b"#d DEV KN4 LHR-PARSE 24 tracking 0.004 knuckles_r 48000000 0 0 -0.16 1 0 0 0 83 2829C4CAEA-5YX",
		);
		let it = G_PROVIDER.devices().devices.get(&b"LHR-PARSE"[..]).cloned();
		c.check(
			it.as_ref().is_some_and(|d| d.kind == Kind::RightController),
			"parser: DEV line makes a right controller",
			it.is_some() as i32 as f64,
			1.0,
		);
		let it = it.expect("the DEV line's device");
		let pose = || it.state().pose;
		G_PROVIDER.handle_line(
			b"Q KN4 LHR-PARSE 123456 12.5 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10 0 0 0 0.0025 0 0 0 0 0 0 0 0 -500",
		);
		let p = pose();
		c.check(
			near(p.vecAngularAcceleration[2], -500.0, 1e-9),
			"parser: 31-field Q line hands alpha to Update",
			p.vecAngularAcceleration[2],
			-500.0,
		);
		let want = 10.0 - 500.0 * 0.0025;
		c.check(
			near(p.vecAngularVelocity[2], want, 1e-9),
			"parser: ... and the rate at the lead's end",
			p.vecAngularVelocity[2],
			want,
		);
		G_PROVIDER
			.handle_line(b"Q KN4 LHR-PARSE 123457 12.6 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10 0 0 0 0.0025 0 0 0 0 0 0");
		let p = pose();
		c.check(
			near(p.vecAngularAcceleration[2], 0.0, 1e-12) && near(p.vecAngularVelocity[2], 10.0, 1e-9),
			"parser: 28-field Q line (no angular acceleration): no alpha",
			p.vecAngularVelocity[2],
			10.0,
		);
		G_PROVIDER.handle_line(b"Q KN4 LHR-PARSE 123458 12.7 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10");
		let p = pose();
		c.check(
			near(p.vecAngularVelocity[2], 10.0, 1e-9),
			"parser: 18-field Q line (no look-ahead) still taken",
			p.vecAngularVelocity[2],
			10.0,
		);
		G_PROVIDER.handle_line(
			b"Q KN4 LHR-PARSE 123459 12.8 0.1 0.2 0.3 1 0 0 0 0 0 0 0 0 10 0 0 0 0.0025 0 0 0 0 0 0 nan 0 0",
		);
		let p = pose();
		c.check(
			p.vecAngularVelocity[0].is_finite() && p.vecAngularVelocity[2].is_finite(),
			"parser: a nan alpha does not reach the pose",
			p.vecAngularVelocity[2],
			10.0,
		);
		// a position that is not a number: the line is read (nan is a number to sscanf) and dropped, the pose as it was
		let line = b"Q KN4 LHR-PARSE 123460 12.9 nan 0.2 0.3 1 0 0 0 0 0 0 0 0 10";
		let read = parse_pose(line, true).is_some_and(|(_, _, v, _, _, _)| v[0].is_nan());
		let before = pose().vecPosition;
		G_PROVIDER.handle_line(line);
		let p = pose();
		c.check(
			read && p.vecPosition == before,
			"parser: a Q line with a nan position is dropped",
			p.vecPosition[0],
			before[0],
		);
		// a head-to-IMU that is not a number: no lever arm from it, the next good one taken
		G_PROVIDER.handle_line(
			b"#d DEV KN5 LHR-NANH 24 tracking 0.004 knuckles_l 48000000 nan 0 -0.16 1 0 0 0 83 2829C4CAEA-5YX",
		);
		let d = G_PROVIDER.devices().devices.get(&b"LHR-NANH"[..]).cloned().expect("the DEV line's device");
		c.check(!d.state().have_lever, "parser: a nan head-to-IMU is not the lever arm", 1.0, 0.0);
		G_PROVIDER.handle_line(
			b"#d DEV KN5 LHR-NANH 24 tracking 0.004 knuckles_l 48000000 0 0 -0.16 1 0 0 0 83 2829C4CAEA-5YX",
		);
		let st = d.state();
		c.check(
			st.have_lever && near(st.lever_head[2], -0.16, 1e-12),
			"parser: ... the next good one is",
			st.lever_head[2],
			-0.16,
		);
	}
	// the headset: the runtime's index_hmd is a device of the driver's only with driver.hmd
	{
		let o = g_opt();
		set_g_opt(Options { hmd: false, ..(*o).clone() });
		G_PROVIDER.handle_line(b"#d DEV T20 LHR-HMD0 32 tracking 0.004 index_hmd 48000000 0 0 0 1 0 0 0 -1 LHR-HMD0");
		let off = !G_PROVIDER.devices().devices.contains_key(&b"LHR-HMD0"[..]);
		c.check(off, "headset: without driver.hmd the runtime's headset is no device of the driver's", 1.0, 0.0);
		set_g_opt(Options { hmd: true, ..(*o).clone() });
		G_PROVIDER.handle_line(b"#d DEV T20 LHR-HMD1 32 tracking 0.004 index_hmd 48000000 0 0 0 1 0 0 0 -1 LHR-HMD1");
		let kind = G_PROVIDER.devices().devices.get(&b"LHR-HMD1"[..]).map(|d| d.kind_name());
		c.check(
			kind == Some(&b"headset"[..]),
			"headset: with driver.hmd it is the driver's headset",
			kind.is_some() as i32 as f64,
			1.0,
		);
		set_g_opt((*o).clone());
	}
	println!("{}: {} failed", if c.fails != 0 { "FAILED" } else { "ALL PASSED" }, c.fails);
	assert_eq!(c.fails, 0);
}
