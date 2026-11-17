//! One device the runtime tracks (RuntimeDevice): turns its reports into a DriverPose_t for the override and notices
//! when it stops.
//!
//! Pose math on the runtime's reports:
//!  * linear velocity is moved from the IMU point (what libsurvive reports) to the pose origin: v + w x r. On Index
//!    controllers the IMU sits ~16 cm from the origin; without this every wrist turn sent up to ~1 m/s of fake
//!    velocity and SteamVR's prediction swung the controller around.
//!  * angular velocity goes from the world frame to the device frame (what SteamVR expects).
//!  * libsurvive's world is Z-up, OpenVR's is Y-up: (x, y, z) -> (x, z, -y), q -> Rx(-90) * q.
//!  * pose age = the device's prediction_ms (the output layer's, output.rs: the pipeline's measured age by default).

use std::sync::{Mutex, MutexGuard};

use sally_math::printf::BArg;

use crate::helpers::{cross, log, norm, now_seconds, quat_conj, quat_mul, quat_rotate, sin_cos, std_max};
use crate::openvr::{DriverPose_t, HmdQuaternion_t, TrackingResult_Running_OK, TrackingResult_Running_OutOfRange};
use crate::output::{self, Blend, Shown};
use crate::pose_override::G_OVERRIDE;
use crate::settings::{g_opt, Options};
use crate::toys::{carried, Kin, Toys};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
	Tracker,
	LeftController,
	RightController,
	/// a lighthouse headset (the runtime's index_hmd or vive_hmd), taken only with driver.hmd
	Headset,
}

impl Kind {
	/// A device's kind from its subtype on the runtime's DEV line; the headset's (index_hmd, vive_hmd) only where it
	/// is taken (`hmd`: driver.hmd). None for anything else
	pub fn of_subtype(subtype: &[u8], hmd: bool) -> Option<Kind> {
		match subtype {
			b"knuckles_l" => Some(Kind::LeftController),
			b"knuckles_r" => Some(Kind::RightController),
			b"tracker" | b"tracker_gen2" | b"wand" => Some(Kind::Tracker),
			b"index_hmd" | b"vive_hmd" if hmd => Some(Kind::Headset),
			_ => None,
		}
	}
}

/// The IMU -> origin offset (device frame) from a DEV line's head-to-IMU (translation, then the rotation w x y z);
/// None for an identity one (the runtime has not read the device's config)
pub fn lever_of(h2i: &[f64; 7]) -> Option<[f64; 3]> {
	let identity = norm(h2i) < 1e-9 && ((h2i[3]).abs() - 1.0).abs() < 1e-9;
	if identity {
		return None;
	}
	let c = quat_conj(&[h2i[3], h2i[4], h2i[5], h2i[6]]);
	Some(quat_rotate(&c, &[h2i[0], h2i[1], h2i[2]]))
}

/// What RuntimeDevice keeps under its m_
#[derive(Clone, Copy, Debug)]
pub struct DeviceState {
	pub pose: DriverPose_t,
	pub lever_head: [f64; 3],
	pub have_lever: bool,
	pub last_update: f64,
	pub lost: bool,
	pub stats_t0: f64,
	pub age_sum: f64,
	pub corr_sum: f64,
	pub n: i32,
	pub n_novel: i32,
	/// the output layer's glides (output.rs)
	pub glides: Glides,
}

/// The output layer's state of a device between its reports: the recovery blend, the smoothing's glide that every
/// other correction is let in by, and the toys (toys.rs: the punch rail among them)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Glides {
	pub blend: Blend,
	pub smooth: Blend,
	pub toys: Toys,
}

pub struct RuntimeDevice {
	pub serial: Vec<u8>,
	pub kind: Kind,
	m: Mutex<DeviceState>,
}

/// One report from the runtime, as Update takes it: pos / rot the head pose (Z-up world), vel / angvel libsurvive's
/// velocity (IMU point, world frame), has_vel false for a P line (no velocity this report). The rest are the Q line's
/// optional tails, None when the runtime sent none. ho: the hand-over look-ahead, 10 values: the IMU point's
/// acceleration at its end (world, gravity out), the lead (s), the head point's displacement over it, the IMU point's
/// velocity change over it. al: the angular acceleration (world, rad/s^2), for the orientation and the rate. corr: the
/// summed corrections (head point, world; orientation, world side) and the light's return (how long ago it began, how
/// long the device was dark before), for the output layer's blend
#[derive(Clone, Copy, Debug)]
pub struct Report<'a> {
	pub pos: &'a [f64; 3],
	pub rot: &'a [f64; 4],
	pub vel: &'a [f64; 3],
	pub angvel: &'a [f64; 3],
	pub has_vel: bool,
	pub ho: Option<&'a [f64; 10]>,
	pub al: Option<&'a [f64; 3]>,
	pub corr: Option<&'a [f64; 9]>,
}

/// What Update makes of a report besides the pose: its declared age and the lever's velocity correction (the stats'
/// sums), and what the override is to do with the pose as it hands it to SteamVR (the output layer's: `report_pose`
/// fills it in)
pub struct Handed {
	pub age: f64,
	pub corr: f64,
	pub shown: Shown,
}

impl RuntimeDevice {
	pub fn new(serial: &[u8], kind: Kind) -> RuntimeDevice {
		let mut pose = DriverPose_t::default();
		pose.qWorldFromDriverRotation.w = 1.0;
		pose.qDriverFromHeadRotation.w = 1.0;
		pose.qRotation.w = 1.0;
		pose.deviceIsConnected = 1;
		pose.result = TrackingResult_Running_OutOfRange;
		pose.poseIsValid = 0;
		RuntimeDevice {
			serial: serial.to_vec(),
			kind,
			m: Mutex::new(DeviceState {
				pose,
				lever_head: [0.0; 3],
				have_lever: false,
				last_update: 0.0,
				lost: true,
				stats_t0: 0.0,
				age_sum: 0.0,
				corr_sum: 0.0,
				n: 0,
				n_novel: 0,
				glides: Glides::default(),
			}),
		}
	}

	fn lock(&self) -> MutexGuard<'_, DeviceState> {
		self.m.lock().unwrap_or_else(|e| e.into_inner())
	}

	/// The device's state as it stands (the tests read it)
	pub fn state(&self) -> DeviceState {
		*self.lock()
	}

	pub fn kind_name(&self) -> &'static [u8] {
		match self.kind {
			Kind::LeftController => b"left controller",
			Kind::RightController => b"right controller",
			Kind::Tracker => b"tracker",
			Kind::Headset => b"headset",
		}
	}

	/// SetDeviceInfo: the IMU -> origin offset from head2imu (DEV line), as the survive_fork driver computed it
	pub fn set_device_info(&self, h2i: &[f64; 7]) {
		let mut st = self.lock();
		if st.have_lever {
			return;
		}
		let Some(lever) = lever_of(h2i) else { return };
		st.lever_head = lever;
		st.have_lever = true;
		log(
			"sally: %s lever arm (IMU -> pose origin, device frame) %.1f %.1f %.1f mm\n",
			&[
				BArg::S(&self.serial),
				BArg::F(st.lever_head[0] * 1e3),
				BArg::F(st.lever_head[1] * 1e3),
				BArg::F(st.lever_head[2] * 1e3),
			],
		);
	}

	/// Update: one report from the runtime, handed to the override (shown on Valve's device entry of the same serial).
	/// The device's lock is let go before: the override can push the pose into vrserver (Valve quiet), and RunFrame's
	/// check_lost, on vrserver's main thread, waits for this lock (the receive thread is the only caller, so the poses
	/// still reach the override in order)
	pub fn update(&self, r: &Report) {
		let mut st = self.lock();
		let now = now_seconds();
		let opt = g_opt();
		let lever = st.lever_head;
		let have_lever = st.have_lever;
		let out = output::values(&self.serial, self.kind);
		let lever = have_lever.then_some(&lever);
		let h = {
			let st = &mut *st;
			report_pose(&opt, self.kind, lever, r, &out, now, &mut st.glides, &mut st.pose)
		};
		let pose = st.pose;
		drop(st);
		G_OVERRIDE.on_sally_pose(&self.serial, &pose, h.shown, now);

		let mut st = self.lock();
		st.last_update = now;
		if st.lost {
			st.lost = false;
			log("sally: %s tracking again\n", &[BArg::S(&self.serial)]);
		}

		// ---- stats
		st.n += 1;
		if !r.has_vel {
			st.n_novel += 1;
		}
		st.age_sum += h.age;
		st.corr_sum += h.corr;
		if opt.stats && now - st.stats_t0 >= 10.0 {
			if st.stats_t0 > 0.0 {
				let dt = now - st.stats_t0;
				log(
					"sally: %s %.0f poses/s, %.0f%% with velocity, age %.1f ms, lever %.0f mm, vel corr avg %.0f mm/s\n",
					&[
						BArg::S(&self.serial),
						BArg::F(st.n as f64 / dt),
						BArg::F(if st.n != 0 { 100.0 * (st.n - st.n_novel) as f64 / st.n as f64 } else { 0.0 }),
						BArg::F(if st.n != 0 { st.age_sum / st.n as f64 * 1e3 } else { 0.0 }),
						BArg::F(if st.have_lever { norm(&st.lever_head) * 1e3 } else { -1.0 }),
						BArg::F(if st.n != 0 { st.corr_sum / st.n as f64 * 1e3 } else { 0.0 }),
					],
				);
			}
			st.stats_t0 = now;
			st.n = 0;
			st.n_novel = 0;
			st.age_sum = 0.0;
			st.corr_sum = 0.0;
		}
	}

	/// CheckLost: no pose for lost_timeout_ms: sally has lost the device (the override falls back to Valve's pose for it).
	/// The override is told without this device's lock held (it pushes into vrserver); a pose that comes meanwhile
	/// keeps the device (on_sally_lost's `since`)
	pub fn check_lost(&self, now: f64) {
		let mut st = self.lock();
		if st.lost || now - st.last_update < g_opt().lost_timeout_ms * 1e-3 {
			return;
		}
		st.lost = true;
		let last = st.last_update;
		drop(st);
		G_OVERRIDE.on_sally_lost(&self.serial, now, last);
		if last > 0.0 {
			log("sally: %s lost (no pose for %.0f ms)\n", &[BArg::S(&self.serial), BArg::F((now - last) * 1e3)]);
		}
	}
}

/// What a report hands the override: the output layer's glides (output.rs: the light's return shown as a glide, every
/// other correction let in by the smoothing's; the rest of the report as it came), then the hand-over, the pose
/// declared the device's prediction_ms old; and with it (`Handed::shown`) what the override does as it hands the pose
/// to SteamVR: what the toys show in its place (toys.rs), the offset, the velocity's scale. `out` the device's output
/// settings, `now` the driver's clock, `glides` and `o` the device's own between reports. (The bench runs a replay's
/// driver stream through this: `sally-bench output`)
#[allow(clippy::too_many_arguments)]
pub fn report_pose(
	opt: &Options, kind: Kind, lever: Option<&[f64; 3]>, r: &Report, out: &output::Values, now: f64,
	glides: &mut Glides, o: &mut DriverPose_t,
) -> Handed {
	let (mut pos, mut rot) = (*r.pos, *r.rot);
	if let Some(c) = r.corr {
		let gap = opt.fallback_after_ms * 1e-3;
		glides.blend.show(&mut pos, &mut rot, c, now, out[output::RECOVERY_BLEND_MS], gap);
		let (speed, turn) = if r.has_vel { (norm(r.vel), norm(r.angvel)) } else { (0.0, 0.0) };
		glides.smooth.smooth(&mut pos, &mut rot, c, now, out[output::SMOOTHING_MS], speed, turn, gap);
	}
	let r = &Report { pos: &pos, rot: &rot, ..*r };
	let h = hand_over(opt, kind, lever, r, out[output::PREDICTION_MS], o);
	// what the override does with the pose: the device's offset and velocity scale, and what the toys show in its
	// place, which go by the pose as it is handed over, carried to now as the override carries it (driver.carry)
	let raw = Kin::of(o);
	let own = if opt.carry { carried(&raw) } else { raw };
	let reshaped = glides.toys.step(kind, out, now, &raw, &own);
	Handed { shown: Shown { reshaped, dash: glides.toys.dash, ..Shown::of(out) }, ..h }
}

/// Update's pose math: the pose a report hands the override, written into the device's pose `o` (pose_, whose
/// world-from-driver, driver-from-head and the two flags it never writes stay as the constructor set them), with
/// `lever` the device's IMU -> origin offset (device frame) once its DEV line gave one, `prediction_ms` the age the
/// pose is declared (the output layer's, per device)
pub fn hand_over(
	opt: &Options, kind: Kind, lever: Option<&[f64; 3]>, r: &Report, prediction_ms: f64, o: &mut DriverPose_t,
) -> Handed {
	// ---- the angular acceleration: the rate changes over the hand-over too, so the orientation is turned with it and
	// the rate handed on is the one at the end. Controllers only (not the headset either): a held controller's angular
	// accelerations are the wrist's, which last (the stop of a punch: tens of ms); a strapped tracker's are mostly
	// impacts and the strap's ringing, over before the hand-over is (the trackers' worst 1 % get worse with it).
	// Clamped so the change over the whole hand-over (the lead and the declared age) is no larger than the rate itself:
	// a turn that is stopping stops, the carry does not turn it back (without the clamp the hands' rare largest
	// accelerations, impacts and violent swings over 2000 rad/s^2, come out worse). The lever terms below keep the
	// state's rate: the runtime has put the angular acceleration's part of the head point's motion into the
	// acceleration, the velocity change and the displacement already
	let mut al = [0.0f64; 3];
	if let Some(al_in) = r.al {
		if r.has_vel && matches!(kind, Kind::LeftController | Kind::RightController) {
			for i in 0..3 {
				al[i] = if al_in[i].is_finite() { al_in[i] } else { 0.0 };
			}
			let lead_in = match r.ho {
				Some(ho) if ho[3] > 0.0 && ho[3] < 0.02 => ho[3],
				_ => 0.0,
			};
			let span = std_max(lead_in, prediction_ms * 1e-3);
			let change = norm(&al) * span;
			let rate = norm(r.angvel);
			if change > rate {
				for a in al.iter_mut() {
					*a *= if change > 0.0 { rate / change } else { 0.0 };
				}
			}
		}
	}

	// ---- the look-ahead: the state is on the gyro's time, the accelerometer's newest samples belong `lead` past it;
	// the runtime integrated them (measured, not predicted): the pose moves by the head point's displacement, turns by
	// the body rate over the lead, the velocity changes by the measured change, and the pose is `lead` younger
	let mut pos = *r.pos;
	let mut rot = *r.rot;
	let mut vel_imu = *r.vel;
	let mut acc_imu = [0.0f64; 3];
	let mut lead = 0.0;
	if let Some(ho) = r.ho {
		if r.has_vel && ho[3] > 0.0 && ho[3] < 0.02 {
			lead = ho[3];
			for i in 0..3 {
				acc_imu[i] = ho[i];
				pos[i] += ho[4 + i];
				vel_imu[i] += ho[7 + i];
			}
			// the turn over the lead: w lead + alpha lead^2 / 2 (world)
			let angvel = r.angvel;
			let rv = [
				angvel[0] * lead + 0.5 * al[0] * lead * lead,
				angvel[1] * lead + 0.5 * al[1] * lead * lead,
				angvel[2] * lead + 0.5 * al[2] * lead * lead,
			];
			let th = norm(&rv);
			if th > 1e-9 {
				let (s, c) = sin_cos(0.5 * th);
				let sh = s / th;
				let dq = [c, rv[0] * sh, rv[1] * sh, rv[2] * sh];
				let r2 = quat_mul(&dq, &rot); // the world-frame turn, on the left
				let n = (r2[0] * r2[0] + r2[1] * r2[1] + r2[2] * r2[2] + r2[3] * r2[3]).sqrt();
				for i in 0..4 {
					rot[i] = r2[i] / n;
				}
			}
		}
	}

	// ---- pose age: the declared age, less what the look-ahead measured
	let age = std_max(0.0, prediction_ms * 1e-3 - lead);

	// ---- lever arm: pose origin velocity = v_imu + w x r, r = R_head2world * (IMU -> origin in the device frame) and
	// its acceleration a_imu + w x (w x r) (the angular acceleration's term, alpha x r, comes folded into a_imu, the
	// velocity change and the displacement from the runtime)
	let mut vel = [0.0f64; 3];
	let mut w = [0.0f64; 3];
	let mut accel = [0.0f64; 3];
	let mut corr = 0.0;
	if r.has_vel {
		for i in 0..3 {
			vel[i] = vel_imu[i];
			w[i] = r.angvel[i];
			accel[i] = acc_imu[i];
		}
		if let Some(lever_head) = lever.filter(|_| opt.lever_arm) {
			let r_world = quat_rotate(&rot, lever_head);
			let wxr = cross(&w, &r_world);
			let wwxr = cross(&w, &wxr);
			for i in 0..3 {
				vel[i] += wxr[i];
				if r.ho.is_some() {
					accel[i] += wwxr[i];
				}
			}
			corr = norm(&wxr);
		}
	}

	// ---- angular velocity: world -> device frame; the rate at the end of the lead (with the angular acceleration),
	// and the angular acceleration in the device frame for the carry
	let mut w_out = [0.0f64; 3];
	for i in 0..3 {
		w_out[i] = w[i] + al[i] * lead;
	}
	let qinv = quat_conj(&rot);
	let w_body = quat_rotate(&qinv, &w_out);
	let al_body = quat_rotate(&qinv, &al);

	// ---- Z-up -> Y-up
	o.poseTimeOffset = if opt.time_offset { -age } else { 0.0 };
	o.vecPosition = [pos[0], pos[2], -pos[1]];
	let s = 0.5f64.sqrt();
	let (qw, qx, qy, qz) = (rot[0], rot[1], rot[2], rot[3]);
	o.qRotation = HmdQuaternion_t { w: s * (qw + qx), x: s * (qx - qw), y: s * (qy + qz), z: s * (qz - qy) };
	o.vecVelocity = if opt.velocity { [vel[0], vel[2], -vel[1]] } else { [0.0; 3] };
	for i in 0..3 {
		o.vecAngularVelocity[i] = if opt.angular_velocity { w_body[i] } else { 0.0 };
	}
	// the head point's acceleration, kept here for the hand-over carry only (driver.carry). The override sends SteamVR
	// 0: its prediction does not use vecAcceleration (what it shows 40 ms ahead is the pose plus velocity x 40 ms, with
	// or without it), and Valve's driver sends 0 too
	o.vecAcceleration[0] = if opt.velocity { accel[0] } else { 0.0 };
	o.vecAcceleration[1] = if opt.velocity { accel[2] } else { 0.0 };
	o.vecAcceleration[2] = if opt.velocity { -accel[1] } else { 0.0 };
	// ... and the angular acceleration (device frame, so unchanged by Z-up -> Y-up), also for the carry only
	for i in 0..3 {
		o.vecAngularAcceleration[i] = if opt.angular_velocity { al_body[i] } else { 0.0 };
	}
	o.poseIsValid = 1;
	o.result = TrackingResult_Running_OK;
	o.deviceIsConnected = 1;
	Handed { age, corr, shown: Shown::default() }
}
