//! Dash punch: the whole body moved with a fist on its punch rail, the headset with it (docs/sally_driver.md, "Dash
//! punch"). Play, not tracking: a switch among the output settings, off until it is switched on, and off again when
//! SteamVR closes. With it off and nothing left to ease away there is no move ([`Body::at`] is None) and a pose
//! passes as it is, to the bit.
//!
//! The move is one slide of SteamVR's world, the same for every device at the same moment. It is put on a pose's
//! world-from-driver as the pose goes to SteamVR ([`Move::apply`]; pose_override.rs's hook, for every device of
//! every driver: the headset, the hands, the trackers, the base stations), with the slide's own velocity on the
//! pose's, so SteamVR's straight line goes on where the body goes. To an app it is what a playspace mover does. The
//! frame fit, the fallback, the room view and the recordings go by the poses before it.
//!
//! **Dash punch** (`dash`, `dash_percent`, `dash_back_ms`; a hand's settings, with its punch rail on; [`Slide`]):
//! during a punch on its rail the body goes with the fist. That share of what the rail shows the hand ahead by (the
//! height left out) is the body's to slide by, and the hand's own pose keeps the rest (toys.rs), so at 100 % the fist
//! is where the rail puts it with the arm as the wearer holds it: an avatar's arm does not have to reach. The body is
//! eased back in `dash_back_ms`.
//!
//! What moves is the devices against SteamVR's standing space, so SteamVR's chaperone stays where the standing space
//! has it: the walls are off by the slide for as long as it lasts.

use std::collections::BTreeMap;

use crate::helpers::{quat_conj, quat_rotate};
use crate::openvr::{DriverPose_t, HmdQuaternion_t};
use crate::output::{settle, SETTLE_WT};

/// Dash punch: the body goes out with the fist in this long (s, 95 % there) ...
const DASH_OUT_S: f64 = 0.03;
/// ... and a hand that has said nothing for this long (s) has no punch on
const DASH_GAP_S: f64 = 0.1;

/// What a hand's punch rail gives the body to slide by (toys.rs: `dash_percent` of what the rail shows the hand
/// ahead by, the height left out), its rate, and how long the body takes back (s). In the frame of whoever holds it:
/// the runtime's world at the report, SteamVR's world at [`Body::dash`]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dash {
	pub shift: [f64; 3],
	pub rate: [f64; 3],
	pub back_s: f64,
}

/// Dash punch's state of one hand: what its rail gives the body to slide by, and the slide that is shown for it
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Slide {
	/// the driver's clock the slide is at, and when the hand last said what its rail shows
	t: f64,
	heard: f64,
	goal: Dash,
	x: [f64; 3],
	v: [f64; 3],
}

impl Slide {
	/// On to `now`: while what the rail gives is further out than the slide, the slide catches up with it and goes as
	/// it goes (the body with the fist); once it is not, the slide comes back toward it as a critically damped glide,
	/// in `back_s`
	fn advance(&mut self, now: f64) {
		if now - self.heard > DASH_GAP_S {
			(self.goal.shift, self.goal.rate) = ([0.0; 3], [0.0; 3]);
		}
		let dt = now - self.t;
		if !(dt > 0.0) {
			return;
		}
		self.t = now;
		let sq = |a: &[f64; 3]| a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
		// the goal goes on at its own rate meanwhile (the hand reports every few ms)
		let carry = dt.min(DASH_GAP_S);
		let goal: [f64; 3] = std::array::from_fn(|i| self.goal.shift[i] + self.goal.rate[i] * carry);
		if sq(&self.goal.shift) > sq(&self.x) {
			// out: what the slide is behind the goal by, and slower than it, dies away, so it goes as the fist goes
			let mut d: [f64; 3] = std::array::from_fn(|i| self.x[i] - self.goal.shift[i]);
			let mut dv: [f64; 3] = std::array::from_fn(|i| self.v[i] - self.goal.rate[i]);
			settle(&mut d, &mut dv, SETTLE_WT / DASH_OUT_S, dt);
			self.x = std::array::from_fn(|i| goal[i] + d[i]);
			self.v = std::array::from_fn(|i| self.goal.rate[i] + dv[i]);
		} else {
			// back: a glide toward where the goal is, however fast that went there
			let mut d: [f64; 3] = std::array::from_fn(|i| self.x[i] - goal[i]);
			settle(&mut d, &mut self.v, SETTLE_WT / self.goal.back_s.max(DASH_OUT_S), dt);
			self.x = std::array::from_fn(|i| goal[i] + d[i]);
		}
		self.goal.shift = goal;
	}

	fn none(&self) -> bool {
		let small = |a: &[f64; 3], lim: f64| a.iter().all(|x| x.abs() < lim);
		small(&self.goal.shift, 1e-9) && small(&self.x, 1e-6) && small(&self.v, 1e-5)
	}
}

/// The body's move at one moment: SteamVR's world is shown `slide` from where it is, going at `slide_rate`
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Move {
	pub slide: [f64; 3],
	pub slide_rate: [f64; 3],
}

impl Move {
	/// Where a point of the world is shown
	pub fn point(&self, x: &[f64; 3]) -> [f64; 3] {
		[x[0] + self.slide[0], x[1] + self.slide[1], x[2] + self.slide[2]]
	}

	/// The move on a pose as it goes to SteamVR: on its world-from-driver's translation (the pose in its driver
	/// space stays; the world-from-driver's rotation goes out at unit length), with the slide's velocity on the
	/// pose's (driver space)
	pub fn apply(&self, o: &mut DriverPose_t) {
		let q = [
			o.qWorldFromDriverRotation.w,
			o.qWorldFromDriverRotation.x,
			o.qWorldFromDriverRotation.y,
			o.qWorldFromDriverRotation.z,
		];
		let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
		let q = [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
		o.qWorldFromDriverRotation = HmdQuaternion_t { w: q[0], x: q[1], y: q[2], z: q[3] };
		o.vecWorldFromDriverTranslation = self.point(&o.vecWorldFromDriverTranslation);
		let v = quat_rotate(&quat_conj(&q), &self.slide_rate);
		for i in 0..3 {
			o.vecVelocity[i] += v[i];
		}
	}
}

/// The body's slides between poses: dash punch's, by the hand's serial
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Body {
	pub slides: BTreeMap<Vec<u8>, Slide>,
}

impl Body {
	pub const fn new() -> Body {
		Body { slides: BTreeMap::new() }
	}

	/// A hand's report at `now`: what its punch rail gives the body to slide by (SteamVR's world), None where it
	/// gives nothing (dash punch off, or no rail)
	pub fn dash(&mut self, now: f64, serial: &[u8], d: Option<Dash>) {
		match (self.slides.get_mut(serial), d) {
			(Some(s), d) => {
				s.advance(now);
				s.heard = now;
				s.goal = d.unwrap_or(Dash { shift: [0.0; 3], rate: [0.0; 3], back_s: s.goal.back_s });
			}
			(None, Some(d)) if d.shift.iter().chain(&d.rate).all(|x| x.is_finite()) => {
				self.slides.insert(serial.to_vec(), Slide { t: now, heard: now, goal: d, x: [0.0; 3], v: [0.0; 3] });
			}
			_ => {}
		}
	}

	/// Every hand's slide on to `now`, summed, and its rate; a slide that has come back is dropped
	fn slide(&mut self, now: f64) -> ([f64; 3], [f64; 3]) {
		let (mut x, mut v) = ([0.0; 3], [0.0; 3]);
		for s in self.slides.values_mut() {
			s.advance(now);
			for i in 0..3 {
				x[i] += s.x[i];
				v[i] += s.v[i];
			}
		}
		self.slides.retain(|_, s| !s.none());
		if self.slides.is_empty() {
			return ([0.0; 3], [0.0; 3]);
		}
		(x, v)
	}

	/// The body's move at `now`; None where there is none: every device's pose goes out as it is
	pub fn at(&mut self, now: f64) -> Option<Move> {
		let (slide, slide_rate) = self.slide(now);
		if slide == [0.0; 3] && slide_rate == [0.0; 3] {
			return None;
		}
		Some(Move { slide, slide_rate })
	}

	/// Something of a slide is still shown: the move is to be asked for
	pub fn busy(&self) -> bool {
		!self.slides.is_empty()
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// with no punch there is no move
	#[test]
	fn without_a_punch_there_is_no_move() {
		let mut body = Body::default();
		body.dash(100.0, b"LHR-HAND", None);
		assert_eq!(body.at(100.0), None);
		assert!(!body.busy());
	}

	/// dash punch: the body goes out with what the rail gives within a few centimetres, is eased back in the time
	/// set once the punch is over, and nothing is left
	#[test]
	fn the_body_goes_out_with_the_fist_and_is_eased_back() {
		let mut body = Body::default();
		let back_s = 0.4;
		// the rail's share: out to 0.8 m along x over 0.1 s, held 50 ms, gone in the rail's own 0.15 s
		let rail = |s: f64| {
			if s < 0.1 {
				(8.0 * s, 8.0)
			} else if s < 0.15 {
				(0.8, 0.0)
			} else {
				(0.8 * (1.0 - (s - 0.15) / 0.15).max(0.0), if s < 0.3 { -0.8 / 0.15 } else { 0.0 })
			}
		};
		let mut most = 0.0f64;
		let (mut at_back, mut last) = (f64::NAN, 0.0);
		for i in 0..2000 {
			let s = i as f64 * 0.004;
			let t = 100.0 + s;
			let (x, v) = rail(s);
			let d = (s < 0.4).then_some(Dash { shift: [x, 0.0, 0.0], rate: [v, 0.0, 0.0], back_s });
			body.dash(t, b"LHR-HAND", d);
			let shown = body.at(t).map_or(0.0, |m| m.slide[0]);
			if s <= 0.15 {
				assert!((shown - x).abs() < 0.12, "at {s}: {shown} against {x}");
			}
			if s > 0.2 {
				assert!(shown <= last + 1e-9, "out again at {s}");
			}
			if (s - (0.15 + back_s)).abs() < 0.003 {
				at_back = shown;
			}
			(most, last) = (most.max(shown), shown);
		}
		assert!(most > 0.75, "{most}");
		// 95 % back in the time set, from where the punch ended
		assert!(at_back < 0.12 && at_back > 0.0, "{at_back}");
		assert_eq!(body.at(110.0), None);
		assert!(!body.busy());
	}

	/// a hand that stops reporting has no punch on: its slide comes back
	#[test]
	fn a_hand_that_falls_silent_lets_the_body_back() {
		let mut body = Body::default();
		for i in 0..50 {
			let d = Dash { shift: [0.0, 0.0, -0.5], rate: [0.0; 3], back_s: 0.2 };
			body.dash(100.0 + i as f64 * 0.004, b"LHR-HAND", Some(d));
		}
		assert!(body.at(100.2).unwrap().slide[2] < -0.45);
		assert_eq!(body.at(103.0), None);
	}

	/// the move on a pose: the device is shown where the slide puts its place in the world, its rotation as it was,
	/// and the velocity SteamVR is given has the slide's own on it, in the pose's driver space
	#[test]
	fn a_pose_goes_out_moved_with_the_moves_velocity() {
		// a driver space turned a quarter about the world's up
		let h = std::f64::consts::FRAC_1_SQRT_2;
		let mut o = DriverPose_t {
			qRotation: HmdQuaternion_t { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
			vecPosition: [0.2, 1.0, 0.3],
			qWorldFromDriverRotation: HmdQuaternion_t { w: h, x: 0.0, y: h, z: 0.0 },
			vecWorldFromDriverTranslation: [1.0, 0.0, -2.0],
			vecVelocity: [0.1, 0.0, 0.0],
			..DriverPose_t::default()
		};
		let before = o;
		let m = Move { slide: [0.5, 0.0, -0.25], slide_rate: [2.0, 0.0, 0.0] };
		m.apply(&mut o);
		assert_eq!(o.vecWorldFromDriverTranslation, [1.5, 0.0, -2.25]);
		assert_eq!((o.vecPosition, o.qRotation), (before.vecPosition, before.qRotation));
		let q = [h, 0.0, h, 0.0];
		let world = quat_rotate(&q, &o.vecVelocity);
		let own = quat_rotate(&q, &before.vecVelocity);
		for i in 0..3 {
			assert!((world[i] - own[i] - m.slide_rate[i]).abs() < 1e-12, "{world:?}");
		}
	}
}
