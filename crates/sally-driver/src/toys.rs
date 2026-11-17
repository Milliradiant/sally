//! The output layer's toys (docs/sally_driver.md, "The punch rail", "Dash punch"): what SteamVR is shown of a hand,
//! reshaped for play. Each is a switch among the output settings (sally_output.txt: per class and per device, as all
//! of them), off until it is switched on; with both off a report passes as it is. Play, not tracking: they run after
//! the hand-over, on the pose as it goes to the override ([`Kin`]) carried to the report's own moment ([`carried`]:
//! what the override does with a device's own pose at the hand-over), so what they show is the device now; the frame
//! fit and the fallback go by the device's own pose, and the room view and the recordings keep the tracker's own.
//!
//! - **the punch rail** (`rail`, `rail_speed`, `rail_boost`; output.rs's [`Rail`]): a fast hand on a straight line,
//!   further and faster along it.
//! - **dash punch** (`dash`, `dash_percent`; body.rs): that share of what the rail shows the hand ahead by, the
//!   height left out, is the whole body's to slide by ([`Toys::dash`]), and the hand's own pose keeps the rest: the
//!   body's slide carries the hand as it carries every device. The headset is the wearer's view: it has no share.
//!
//! A toy that is switched off, or whose share is set anew, leaves no step: what the shown pose was off the device's
//! own by is eased away ([`EASE_S`]).

use crate::body::Dash;
use crate::helpers::{norm, quat_conj, quat_mul, quat_rotate, std_max, std_min};
use crate::openvr::{DriverPose_t, HmdQuaternion_t};
use crate::output::{self, arr, qexp, qlog, settle, v3, Rail, Values, SETTLE_WT};
use crate::runtime_device::Kind;

/// No report for this long (s) starts the toys over, from the device's own pose (the override has shown Valve's or a
/// held pose meanwhile)
const TOY_GAP_S: f64 = 0.1;
/// A toy switched off, or its strength set anew: what the shown pose was off by is eased away in this long (s, 95 %)
pub const EASE_S: f64 = 0.15;

/// A device's kinematics as a report hands them to the override (the runtime's world in OpenVR's axes, Y up): the
/// head point, the rotation (device to world, w x y z), the point's velocity and acceleration, the body rate and its
/// change (device frame), and how old they are declared (s)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Kin {
	pub p: [f64; 3],
	pub q: [f64; 4],
	pub v: [f64; 3],
	pub a: [f64; 3],
	pub w: [f64; 3],
	pub al: [f64; 3],
	pub age: f64,
}

impl Kin {
	pub fn of(o: &DriverPose_t) -> Kin {
		Kin {
			p: o.vecPosition,
			q: [o.qRotation.w, o.qRotation.x, o.qRotation.y, o.qRotation.z],
			v: o.vecVelocity,
			a: o.vecAcceleration,
			w: o.vecAngularVelocity,
			al: o.vecAngularAcceleration,
			age: std_max(0.0, -o.poseTimeOffset),
		}
	}

	/// These kinematics on a pose record (the rest of it kept)
	pub fn put(&self, o: &mut DriverPose_t) {
		o.vecPosition = self.p;
		o.qRotation = HmdQuaternion_t { w: self.q[0], x: self.q[1], y: self.q[2], z: self.q[3] };
		o.vecVelocity = self.v;
		o.vecAcceleration = self.a;
		o.vecAngularVelocity = self.w;
		o.vecAngularAcceleration = self.al;
		o.poseTimeOffset = -self.age;
	}

	fn finite(&self) -> bool {
		let all = |a: &[f64]| a.iter().all(|x| x.is_finite());
		all(&self.p)
			&& all(&self.q)
			&& all(&self.v)
			&& all(&self.a)
			&& all(&self.w)
			&& all(&self.al)
			&& self.age.is_finite()
	}
}

/// The hand-over carry, at the report (pose_override.rs's `substitute` does it for a device's own pose at the
/// hand-over, in Valve's frame; this is the same in the runtime's): the pose is `age` old, and is carried to now as a
/// rigid body, the point with its acceleration, the rotation with the body rate and its change (no larger over the
/// carry than the rate it starts from), at most 50 ms; what is left of its age stays declared
pub fn carried(k: &Kin) -> Kin {
	let age = std_min(0.05, k.age);
	let mut o = *k;
	for i in 0..3 {
		o.p[i] += k.v[i] * age + 0.5 * k.a[i] * age * age;
		o.v[i] += k.a[i] * age;
	}
	let mut al = k.al;
	let (change, rate) = (norm(&al) * age, norm(&k.w));
	if change > rate {
		for a in al.iter_mut() {
			*a *= if change > 0.0 { rate / change } else { 0.0 };
		}
	}
	let rv: [f64; 3] = std::array::from_fn(|i| k.w[i] * age + 0.5 * al[i] * age * age);
	if norm(&rv) > 1e-9 {
		// the body's own turn: on the right
		o.q = unit(quat_mul(&k.q, &qexp(&rv)));
	}
	o.w = std::array::from_fn(|i| k.w[i] + al[i] * age);
	o.al = al;
	o.age = k.age - age;
	o
}

fn unit(q: [f64; 4]) -> [f64; 4] {
	let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
	[q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

/// The toys' state of a device between its reports
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Toys {
	have: bool,
	/// the driver's clock at the last report
	t: f64,
	pub rail: Rail,
	/// which of the toys were on at the last report, the dash's share, and what they showed the pose off the
	/// device's own by (the point; the turn, on the world's side)
	on: u8,
	share: f64,
	/// dash punch: what this report's rail gives the body to slide by (the runtime's world), None where nothing
	pub dash: Option<Dash>,
	off_p: [f64; 3],
	off_q: [f64; 4],
	/// what is being eased away after a switch: the point's offset and the turn's (a rotation vector), their rates
	ease: [f64; 3],
	ease_v: [f64; 3],
	ease_r: [f64; 3],
	ease_rv: [f64; 3],
}

impl Default for Toys {
	fn default() -> Toys {
		Toys {
			have: false,
			t: 0.0,
			rail: Rail::default(),
			on: 0,
			share: 0.0,
			dash: None,
			off_p: [0.0; 3],
			off_q: [1.0, 0.0, 0.0, 0.0],
			ease: [0.0; 3],
			ease_v: [0.0; 3],
			ease_r: [0.0; 3],
			ease_rv: [0.0; 3],
		}
	}
}

impl Toys {
	/// One report: `raw` the device's kinematics as they are handed over and `own` the same carried to now
	/// (`carried`; the rail takes its punches from `raw`, as it did before it was one of these), `out` the device's
	/// output settings, `now` the driver's clock. What the override is to
	/// show in the device's place, None where that is its own pose
	pub fn step(&mut self, kind: Kind, out: &Values, now: f64, raw: &Kin, own: &Kin) -> Option<Kin> {
		let on = |i: usize| out[i] > 0.0;
		let rail_speed = if on(output::RAIL) { out[output::RAIL_SPEED] } else { 0.0 };
		self.dash = None;
		if !own.finite() || !raw.finite() {
			*self = Toys::default();
			return None;
		}
		// the headset is the wearer's view
		let body = kind != Kind::Headset;
		// dash punch: the share of the rail's reach that is the body's
		let share = if body && on(output::DASH) { out[output::DASH_PERCENT].clamp(0.0, 100.0) * 1e-2 } else { 0.0 };
		let fresh = self.have && now >= self.t && now - self.t <= TOY_GAP_S;
		if !fresh {
			// the rail has its own count of the gap (and keeps whether the hand has slowed)
			*self = Toys { rail: self.rail, ..Toys::default() };
		}
		let dt = if fresh { now - self.t } else { 0.0 };
		(self.have, self.t) = (true, now);

		let mut k = *own;
		if let Some(rail) = self.rail.step(now, rail_speed, out[output::RAIL_BOOST], &raw.p, &raw.v) {
			// the point, its velocity and its acceleration as the rail shows them
			let (p, v, a) = (v3(&k.p), v3(&k.v), v3(&k.a));
			let (sp, sv) = rail.map(p, v);
			let sa = rail.map_acc(p, v, a);
			(k.p, k.v, k.a) = (arr(sp), arr(sv), arr(sa));
			if share > 0.0 {
				// the body goes with the fist: that share of what the rail shows the hand ahead by is the body's
				// slide (the height stays the hand's: the feet keep the floor), and is not shown on the hand twice
				let flat = |x: sally_math::math3d::V3| [share * x.x, 0.0, share * x.z];
				let (ds, dv, da) = (flat(sp - p), flat(sv - v), flat(sa - a));
				for i in 0..3 {
					k.p[i] -= ds[i];
					k.v[i] -= dv[i];
					k.a[i] -= da[i];
				}
				self.dash = Some(Dash { shift: ds, rate: dv, back_s: out[output::DASH_BACK_MS].max(1.0) * 1e-3 });
			}
		}

		// a toy switched, or the dash's share set anew: what the pose was shown off by a report ago is kept, and
		// eased away from there, so nothing steps
		let bit = |on: bool, n: u8| (on as u8) << n;
		let now_on = bit(rail_speed > 0.0, 1) | bit(share > 0.0, 4);
		let off_p: [f64; 3] = std::array::from_fn(|i| k.p[i] - own.p[i]);
		let off_q = quat_mul(&k.q, &quat_conj(&own.q));
		settle(&mut self.ease, &mut self.ease_v, SETTLE_WT / EASE_S, dt);
		settle(&mut self.ease_r, &mut self.ease_rv, SETTLE_WT / EASE_S, dt);
		if fresh && (now_on != self.on || share != self.share) {
			for i in 0..3 {
				self.ease[i] += self.off_p[i] - off_p[i];
			}
			let was = quat_mul(&qexp(&self.ease_r), &self.off_q);
			self.ease_r = qlog(&quat_mul(&was, &quat_conj(&off_q)));
		}
		(self.on, self.share, self.off_p, self.off_q) = (now_on, share, off_p, off_q);
		let small = |a: &[f64; 3], lim: f64| a.iter().all(|x| x.abs() < lim);
		if small(&self.ease, 1e-6)
			&& small(&self.ease_v, 1e-5)
			&& small(&self.ease_r, 1e-6)
			&& small(&self.ease_rv, 1e-5)
		{
			// eased away to within a micrometre: nothing left of it
			(self.ease, self.ease_v, self.ease_r, self.ease_rv) = ([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]);
		} else {
			let w = quat_rotate(&k.q, &k.w);
			for i in 0..3 {
				k.p[i] += self.ease[i];
				k.v[i] += self.ease_v[i];
			}
			k.q = unit(quat_mul(&qexp(&self.ease_r), &k.q));
			let ws = [w[0] + self.ease_rv[0], w[1] + self.ease_rv[1], w[2] + self.ease_rv[2]];
			k.w = quat_rotate(&quat_conj(&k.q), &ws);
		}

		(k != *own).then_some(k)
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::output::Output;

	const DT: f64 = 0.004;

	/// A tracker's settings with these set
	fn settings(set: &[(usize, f64)]) -> Values {
		let mut v = Output::default().class[2];
		for &(i, x) in set {
			v[i] = x;
		}
		v
	}

	/// A device that does not turn, at `p` going `v`
	fn at(p: [f64; 3], v: [f64; 3]) -> Kin {
		Kin { p, q: [1.0, 0.0, 0.0, 0.0], v, a: [0.0; 3], w: [0.0; 3], al: [0.0; 3], age: 0.0 }
	}

	fn yaw_of(q: &[f64; 4]) -> f64 {
		2.0 * q[2].atan2(q[0])
	}

	/// `n` reports 4 ms apart from `t0` through the toys: each one's time, the device's own, what is shown
	fn run(toys: &mut Toys, out: &Values, t0: f64, n: usize, own: impl Fn(f64) -> Kin) -> Vec<(f64, Kin, Kin)> {
		(0..n)
			.map(|i| {
				let t = t0 + i as f64 * DT;
				let k = own(t);
				(t, k, toys.step(Kind::Tracker, out, t, &k, &k).unwrap_or(k))
			})
			.collect()
	}

	/// the carry at the report: a pose 10 ms old goes on to now by its velocity and its acceleration, and turns by its
	/// body rate; what the toys are given is the device now
	#[test]
	fn a_report_is_carried_to_its_own_moment() {
		let k = Kin {
			p: [1.0, 0.0, 0.0],
			q: [1.0, 0.0, 0.0, 0.0],
			v: [1.0, 0.0, 0.0],
			a: [0.0, 10.0, 0.0],
			w: [0.0, 2.0, 0.0],
			al: [0.0; 3],
			age: 0.01,
		};
		let c = carried(&k);
		let near = |a: &[f64], b: &[f64]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12);
		assert!(near(&c.p, &[1.01, 0.0005, 0.0]) && near(&c.v, &[1.0, 0.1, 0.0]), "{c:?}");
		assert!((yaw_of(&c.q) - 0.02).abs() < 1e-12 && c.w == k.w && c.age == 0.0);
		// 80 ms old: 50 ms of it carried, the rest still declared
		let c = carried(&Kin { age: 0.08, a: [0.0; 3], ..k });
		assert!(near(&c.p, &[1.05, 0.0, 0.0]) && (c.age - 0.03).abs() < 1e-12, "{c:?}");
		// a pose that is current is itself
		assert_eq!(carried(&at([1.0, 2.0, 3.0], [1.0, 0.0, 0.0])), at([1.0, 2.0, 3.0], [1.0, 0.0, 0.0]));
	}

	/// with every toy off a report passes as it is, however the device moves
	#[test]
	fn with_the_toys_off_a_report_is_the_devices_own() {
		let mut toys = Toys::default();
		let out = settings(&[]);
		for i in 0..500 {
			let t = i as f64 * DT;
			let k = at([t.sin(), 0.3 * t, 1.0], [t.cos(), 0.3, 0.0]);
			assert_eq!(toys.step(Kind::Tracker, &out, t, &k, &k), None);
		}
		assert_eq!(toys, Toys { have: true, t: 499.0 * DT, ..Toys::default() });
	}

	/// the rail switched off leaves no step: what it showed the hand ahead by is eased away, and then the report is
	/// the device's own again
	#[test]
	fn a_toy_switched_off_is_eased_away() {
		let on = settings(&[(output::RAIL, 1.0), (output::RAIL_SPEED, 1.5), (output::RAIL_BOOST, 50.0)]);
		let off = settings(&[]);
		let mut toys = Toys::default();
		// still for 0.1 s, then 3 m/s along x
		let own = |t: f64| {
			if t < 0.1 {
				at([0.0, 0.0, 1.0], [0.0; 3])
			} else {
				at([3.0 * (t - 0.1), 0.0, 1.0], [3.0, 0.0, 0.0])
			}
		};
		let mut path = run(&mut toys, &on, 0.0, 60, own);
		let ahead = path[59].2.p[0] - path[59].1.p[0];
		assert!(ahead > 0.1, "{ahead}");
		path.extend(run(&mut toys, &off, 60.0 * DT, 200, own));
		// no step as it is switched off: the shown point goes on from where it was
		let step = path[60].2.p[0] - path[59].2.p[0];
		assert!((0.0..0.03).contains(&step), "{step}");
		// most of the way back after EASE_S, the device's own soon after
		let back = &path[60 + (EASE_S / DT) as usize];
		assert!(back.2.p[0] - back.1.p[0] < 0.1 * ahead, "{}", back.2.p[0] - back.1.p[0]);
		let end = path.last().unwrap();
		assert_eq!(end.1, end.2);
	}

	/// the headset is the wearer's view: the dash takes no share of its rail
	#[test]
	fn the_dash_is_not_the_headsets() {
		let out = settings(&[(output::RAIL, 1.0), (output::DASH, 1.0)]);
		let mut toys = Toys::default();
		for i in 0..100 {
			let t = i as f64 * DT;
			let k =
				if t < 0.1 { at([0.0, 0.0, 1.0], [0.0; 3]) } else { at([3.0 * (t - 0.1), 0.0, 1.0], [3.0, 0.0, 0.0]) };
			toys.step(Kind::Headset, &out, t, &k, &k);
			assert_eq!(toys.dash, None);
		}
	}

	/// the punch rail runs among the toys: a hand that gets fast is shown further along its line, and faster
	#[test]
	fn the_rail_is_one_of_the_toys() {
		let out = settings(&[(output::RAIL, 1.0), (output::RAIL_SPEED, 1.5), (output::RAIL_BOOST, 50.0)]);
		let mut toys = Toys::default();
		// still for 0.1 s, then 3 m/s along x, a little aside too
		let own = |t: f64| {
			if t < 0.1 {
				at([0.0, 0.0, 1.0], [0.0; 3])
			} else {
				at([3.0 * (t - 0.1), 0.2 * (t - 0.1), 1.0], [3.0, 0.2, 0.0])
			}
		};
		let path = run(&mut toys, &out, 0.0, 60, own);
		assert!(toys.rail.on());
		let (_, own, shown) = path.last().unwrap();
		// 1.5 times as far and as fast along the line it was taken on
		let along = (shown.p[0].powi(2) + shown.p[1].powi(2)).sqrt() / (own.p[0].powi(2) + own.p[1].powi(2)).sqrt();
		assert!((along - 1.5).abs() < 0.02, "{along}");
		assert!((shown.v[0] - 4.5).abs() < 0.05 && (shown.v[1] - 0.3).abs() < 0.05, "{:?}", shown.v);
	}
	/// dash punch: that share of what the rail shows the hand ahead by is the body's to slide by (the height left
	/// out), and the hand's own pose keeps the rest, so the two together are what the rail alone shows
	#[test]
	fn with_dash_punch_the_rails_reach_is_the_bodys() {
		let rail = [(output::RAIL, 1.0), (output::RAIL_SPEED, 1.5), (output::RAIL_BOOST, 100.0)];
		// still for 0.1 s, then 3 m/s along x and 1 m/s up
		let own = |t: f64| {
			if t < 0.1 {
				at([0.0, 0.0, 1.0], [0.0; 3])
			} else {
				at([3.0 * (t - 0.1), 1.0 * (t - 0.1), 1.0], [3.0, 1.0, 0.0])
			}
		};
		let mut plain = Toys::default();
		let alone = run(&mut plain, &settings(&rail), 0.0, 60, own);
		assert_eq!(plain.dash, None);
		for share in [1.0, 0.4] {
			let mut set = rail.to_vec();
			set.extend([(output::DASH, 1.0), (output::DASH_PERCENT, 1e2 * share), (output::DASH_BACK_MS, 250.0)]);
			let mut toys = Toys::default();
			let path = run(&mut toys, &settings(&set), 0.0, 60, own);
			let d = toys.dash.expect("the rail gives the body a slide");
			let ((_, own, shown), (_, _, railed)) = (path.last().unwrap(), alone.last().unwrap());
			// the body's: the share of the rail's reach, level, with its rate; back in the time set
			assert!((d.shift[0] - share * (railed.p[0] - own.p[0])).abs() < 1e-12 && d.shift[1] == 0.0, "{d:?}");
			assert!((d.rate[0] - share * (railed.v[0] - own.v[0])).abs() < 1e-12 && d.back_s == 0.25);
			assert!(d.shift[0] > 0.3 * share);
			// the hand's own pose and the body's slide together: the rail's
			for i in 0..3 {
				assert!((shown.p[i] + d.shift[i] - railed.p[i]).abs() < 1e-12);
				assert!((shown.v[i] + d.rate[i] - railed.v[i]).abs() < 1e-12);
			}
			// its height is the rail's
			assert_eq!(shown.p[1], railed.p[1]);
		}
		// a headset's rail gives the body nothing, and no rail no dash
		let mut set = rail.to_vec();
		set.push((output::DASH, 1.0));
		let mut toys = Toys::default();
		for i in 0..60 {
			let k = own(i as f64 * DT);
			toys.step(Kind::Headset, &settings(&set), i as f64 * DT, &k, &k);
		}
		assert!(toys.rail.on() && toys.dash.is_none());
		let mut toys = Toys::default();
		run(&mut toys, &settings(&[(output::DASH, 1.0)]), 0.0, 60, own);
		assert_eq!(toys.dash, None);
	}
}
