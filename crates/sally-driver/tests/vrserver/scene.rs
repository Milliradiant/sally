//! The scene both sides track: four devices on known paths. The runtime sees them in its world (libsurvive's: metres,
//! Z up); Valve's driver in its driver space, which is the runtime's world in OpenVR's axes (Y up) turned 30 deg about
//! the vertical and moved (`valve_frame`), with a world-from-driver of its own (`world_from_driver`), a
//! driver-from-head offset its driver pose has to be composed with (`driver_from_head`), and a wobble of 2.5 cm, so
//! that its poses differ from the runtime's as two trackings do and what SteamVR got tells whose pose it was. The
//! devices move at up to about 0.9 m/s and turn at up to 1.9 rad/s. One of them is a Tundra's: Valve's head frame is
//! not the runtime's (`Device::head_fix`), and SteamVR is to show it in Valve's.

use std::f64::consts::PI;

use sally_math::math3d::{compose, cross, len, norm, qmul, qrot, xform, Pose, Q4, V3};

/// A device: its head point on `c + a sin(w t + ph)` per axis, turned `0.6 sin(wr t)` rad about `axis`
pub struct Device {
	/// vrserver's id for it (Valve's driver added it)
	pub id: u32,
	pub serial: &'static str,
	/// the dongle it talks through (Valve's driver's Prop_ConnectedWirelessDongle_String, the runtime's DEV line's)
	pub dongle: &'static str,
	/// the runtime's subtype for it (its DEV line's)
	pub subtype: &'static str,
	/// the runtime's look-ahead (s): the accelerometer's newest samples past the gyro's (7 ms on trackers, 2.5 on
	/// controllers)
	pub lead: f64,
	/// IMU point to head point in the device frame (the DEV line's head2imu, its rotation the identity)
	pub lever: V3,
	/// Valve's head against the runtime's (Valve's head pose = the runtime's * this): a Tundra's, 180 deg about the
	/// head's Y and 1.5 mm off, on the second tracker (its driver-from-head the DEV line's head2imu * this); the
	/// identity on the others, whose driver-from-head is not at the runtime's IMU (`driver_from_head`)
	pub head_fix: Pose,
	c: V3,
	a: V3,
	w: V3,
	ph: V3,
	axis: V3,
	wr: f64,
}

const IDENTITY: Pose = Pose { p: V3::new(0.0, 0.0, 0.0), q: Q4::new(1.0, 0.0, 0.0, 0.0) };

/// Two trackers and two controllers (a tracker's IMU sits 1.5 cm from its head point, a controller's 16 cm)
pub const DEVICES: [Device; 4] = [
	Device {
		id: 1,
		serial: "LHR-0A1B2C3D",
		dongle: "2829C4CAEA-5YX",
		subtype: "tracker_gen2",
		lead: 0.007,
		lever: V3::new(0.0, 0.0, -0.015),
		head_fix: IDENTITY,
		c: V3::new(0.1, 0.2, 0.9),
		a: V3::new(0.3, 0.2, 0.1),
		w: V3::new(1.1, 0.7, 1.9),
		ph: V3::new(0.0, 1.0, 2.0),
		axis: V3::new(0.0, 0.0, 1.0),
		wr: 0.9,
	},
	Device {
		id: 2,
		serial: "LHR-11223344",
		dongle: "0A0E4D6852-2YX",
		subtype: "tracker",
		lead: 0.007,
		lever: V3::new(0.0, 0.0, -0.015),
		head_fix: Pose { p: V3::new(0.0, 0.0, 0.0015), q: Q4::new(0.0, 0.0, 1.0, 0.0) },
		c: V3::new(-0.4, 0.1, 0.1),
		a: V3::new(0.2, 0.3, 0.05),
		w: V3::new(0.6, 1.3, 2.3),
		ph: V3::new(1.0, 0.0, 3.0),
		axis: V3::new(1.0, 0.0, 0.0),
		wr: 1.7,
	},
	Device {
		id: 3,
		serial: "LHR-55667788",
		dongle: "00A4AE2B50-RYB",
		subtype: "knuckles_l",
		lead: 0.0025,
		lever: V3::new(0.0, 0.01, -0.16),
		head_fix: IDENTITY,
		c: V3::new(0.3, -0.2, 1.2),
		a: V3::new(0.3, 0.25, 0.2),
		w: V3::new(2.1, 1.7, 1.3),
		ph: V3::new(2.0, 1.0, 0.0),
		axis: V3::new(0.0, 1.0, 0.0),
		wr: 3.1,
	},
	Device {
		id: 4,
		serial: "LHR-99AABBCC",
		dongle: "A9DDD3B47A-LYM",
		subtype: "knuckles_r",
		lead: 0.0025,
		lever: V3::new(0.0, 0.01, -0.16),
		head_fix: IDENTITY,
		c: V3::new(-0.3, -0.3, 1.1),
		a: V3::new(0.3, 0.3, 0.2),
		w: V3::new(1.9, 2.3, 1.1),
		ph: V3::new(0.0, 2.0, 1.0),
		axis: V3::new(0.6, 0.8, 0.0),
		wr: 2.7,
	},
];

/// The head point at a time, in the runtime's world: position, orientation, velocity, acceleration, and the angular
/// velocity and acceleration (world; about a fixed axis, so the same in the device frame)
#[derive(Clone, Copy, Debug)]
pub struct Motion {
	pub p: V3,
	pub q: Q4,
	pub v: V3,
	pub a: V3,
	pub w: V3,
	pub al: V3,
}

impl Device {
	pub fn at(&self, t: f64) -> Motion {
		let arg = |i: usize| self.w.get(i) * t + self.ph.get(i);
		let (a, w) = (self.a, self.w);
		let n = norm(self.axis);
		let th = 0.6 * (self.wr * t).sin();
		Motion {
			p: self.c + V3::new(a.x * arg(0).sin(), a.y * arg(1).sin(), a.z * arg(2).sin()),
			q: axis_angle(n, th),
			v: V3::new(a.x * w.x * arg(0).cos(), a.y * w.y * arg(1).cos(), a.z * w.z * arg(2).cos()),
			a: V3::new(
				-a.x * w.x * w.x * arg(0).sin(),
				-a.y * w.y * w.y * arg(1).sin(),
				-a.z * w.z * w.z * arg(2).sin(),
			),
			w: n * (0.6 * self.wr * (self.wr * t).cos()),
			al: n * (-0.6 * self.wr * self.wr * (self.wr * t).sin()),
		}
	}
}

pub fn axis_angle(n: V3, th: f64) -> Q4 {
	let s = (0.5 * th).sin();
	Q4::new((0.5 * th).cos(), n.x * s, n.y * s, n.z * s)
}

/// A rotation's angle (0..pi), without the C library's acos (the checks' own measure)
pub fn angle(q: Q4) -> f64 {
	2.0 * len(V3::new(q.x, q.y, q.z)).atan2(q.w.abs())
}

/// The runtime's world in OpenVR's axes, as the driver turns it: (x, y, z) -> (x, z, -y)
pub fn yup(v: V3) -> V3 {
	V3::new(v.x, v.z, -v.y)
}

/// ... and a rotation: q -> Rx(-90) q
pub fn yup_q(q: Q4) -> Q4 {
	let s = 0.5f64.sqrt();
	Q4::new(s * (q.w + q.x), s * (q.x - q.w), s * (q.y + q.z), s * (q.z - q.y))
}

/// Valve's driver space: the runtime's world (Y up) turned 30 deg about Y, then moved (test_compare.py's frame)
pub fn valve_frame() -> Pose {
	Pose { p: V3::new(0.5, -0.2, 1.0), q: axis_angle(V3::new(0.0, 1.0, 0.0), 30.0f64.to_radians()) }
}

/// Valve's world-from-driver (Space Calibrator's offset rides on it)
pub fn world_from_driver() -> Pose {
	Pose { p: V3::new(0.1, 0.2, 0.3), q: axis_angle(V3::new(0.0, 1.0, 0.0), 0.17) }
}

/// Valve's driver-from-head: 2 cm, 1 cm, -3 cm and 10 deg about x (fake_vrserver's), which puts the head 27 mm (a
/// tracker's) and 131 mm (a controller's) from where the runtime's head-to-IMU does: Valve's driver pose is not at the
/// runtime's IMU, and the driver leaves the runtime's head frame as it is. The Tundra's is the runtime's head-to-IMU
/// with Valve's head on it (`Device::head_fix`)
pub fn driver_from_head(dev: usize) -> Pose {
	let d = &DEVICES[dev];
	if d.head_fix == IDENTITY {
		valve_driver_from_head()
	} else {
		compose(&Pose { p: d.lever, q: Q4::default() }, &d.head_fix)
	}
}

/// fake_vrserver's driver-from-head (the other drivers' devices have it too)
pub fn valve_driver_from_head() -> Pose {
	Pose { p: V3::new(0.02, 0.01, -0.03), q: Q4::new(0.0873f64.cos(), 0.0873f64.sin(), 0.0, 0.0) }
}

/// The head point's motion in Valve's head frame (`Device::head_fix`), in the runtime's world: what SteamVR is to show
/// under sally, the frame fit applied
pub fn shown(dev: usize, t: f64) -> Motion {
	let d = &DEVICES[dev];
	let m = d.at(t);
	let r = qrot(m.q, d.head_fix.p);
	let wr = cross(m.w, r);
	Motion {
		p: m.p + r,
		q: qmul(m.q, d.head_fix.q),
		v: m.v + wr,
		a: m.a + cross(m.al, r) + cross(m.w, wr),
		w: m.w,
		al: m.al,
	}
}

/// Where Valve's driver space goes for a device it has lost (a long squat): 25 deg about Y and about 1 m away, its
/// world-from-driver moved with it
pub fn lost_frame() -> Pose {
	Pose { p: V3::new(-0.66, 0.34, 0.57), q: axis_angle(V3::new(0.0, 1.0, 0.0), 25.0f64.to_radians()) }
}

/// The device's pose in Valve's driver space as the runtime tracks it: where the frame fit should put sally's pose
pub fn clean(dev: usize, t: f64) -> V3 {
	xform(&valve_frame(), yup(shown(dev, t).p))
}

/// Valve's head pose of the device in its driver space: the clean pose with Valve's own wobble, 2.5 cm round at 0.5 Hz
/// (each device its own phase). test_compare.py's was 3 cm; 2.5 keeps every pair under the frame fit's 3 cm floor, so
/// its trimming does not keep one side of the circle and pull the fit 1 cm along it
pub fn valve_head(dev: usize, t: f64) -> Pose {
	let m = shown(dev, t);
	let k = valve_frame();
	let ph = 0.7 * (dev as f64 + 1.0);
	let wobble = V3::new(0.025 * (PI * t + ph).sin(), 0.0, 0.025 * (PI * t + ph).cos());
	Pose { p: xform(&k, yup(m.p)) + wobble, q: qmul(k.q, yup_q(m.q)) }
}

/// Where Valve's driver boots a device (the first device, until 2 s after the frame fit is ready): a driver space of
/// its own, its world-from-driver putting it where it is (seen 5.3 m / 151 deg off the others', for over a minute)
pub fn boot_frame() -> Pose {
	Pose { p: V3::new(2.1, -0.3, -1.7), q: axis_angle(V3::new(0.0, 1.0, 0.0), 2.6) }
}

/// Valve's world moved (Space Calibrator's continuous mode, a room setup): every device's world-from-driver from then
/// on, their poses in the driver space as they were (seen: 77 mm / 1 deg)
pub fn world_from_driver_moved() -> Pose {
	compose(
		&Pose { p: V3::new(0.06, -0.01, 0.05), q: axis_angle(V3::new(0.0, 1.0, 0.0), 0.0175) },
		&world_from_driver(),
	)
}
