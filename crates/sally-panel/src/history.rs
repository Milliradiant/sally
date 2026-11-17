//! A device's last minutes, for the device page: its pose, speed, position sd, the stations whose light it got and
//! its pose rate, sampled as the streams bring them (at most `RATE` a second), kept for `KEEP` seconds. The samples
//! are on the runtime's clock (the lines' run time), so a panel that stalls for a moment (a window being dragged)
//! loses nothing of their spacing; a clock that goes back (the runtime started again) starts the history over.

use std::collections::VecDeque;

use sally_math::math3d::{Q4, V3};

/// Samples a second at most
pub const RATE: f64 = 120.0;
/// Seconds kept: the longest range the page shows
pub const KEEP: f64 = 120.0;

/// What a sample says about the device's state (the runtime's, as on S lines)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Light {
	Tracking,
	Dark,
	Coasting,
	/// reacquiring, none, or anything else the runtime says
	Other,
}

impl Light {
	pub fn of(state: &str) -> Light {
		match state {
			"tracking" => Light::Tracking,
			"dark" => Light::Dark,
			"coasting" => Light::Coasting,
			_ => Light::Other,
		}
	}
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
	/// the runtime's clock (s)
	pub t: f64,
	pub pos: V3,
	pub rot: Q4,
	/// m/s, the IMU point's (as Q lines carry it; P lines carry none and the last stays)
	pub speed: f64,
	/// the position sd (mm), negative when unknown
	pub sd_mm: f64,
	/// the stations whose sweeps the tracker got in the last 0.5 s (S lines' lh_mask)
	pub lh_mask: u32,
	/// the light's residual (mrad), negative when unknown
	pub res_mrad: f64,
	pub light: Light,
	/// poses a second, as last counted
	pub rate: f64,
}

#[derive(Clone, Debug, Default)]
pub struct History {
	pub samples: VecDeque<Sample>,
}

impl History {
	/// Adds a sample unless the last one is younger than 1 / RATE s, and forgets what is older than KEEP. A sample whose
	/// time is not a number is left out: compared with nothing, it was never thinned nor forgotten, and held everything
	/// after it from being forgotten (the history grew for good)
	pub fn push(&mut self, s: Sample) {
		if !s.t.is_finite() {
			return;
		}
		if let Some(last) = self.samples.back() {
			if s.t < last.t - 1.0 {
				self.samples.clear();
			} else if s.t - last.t < 1.0 / RATE {
				return;
			}
		}
		self.samples.push_back(s);
		while self.samples.front().is_some_and(|f| s.t - f.t > KEEP) {
			self.samples.pop_front();
		}
	}

	/// The samples from `t0` on
	pub fn since(&self, t0: f64) -> impl Iterator<Item = &Sample> {
		let start = self.samples.partition_point(|s| s.t < t0);
		self.samples.range(start..)
	}

	pub fn clear(&mut self) {
		self.samples.clear();
	}
}

/// Yaw, pitch and roll (degrees) of a rotation in the Z-up frame: yaw about Z, pitch about Y, roll about X (the C++
/// panel's yaw_pitch_roll)
pub fn yaw_pitch_roll(q: &Q4) -> (f64, f64, f64) {
	let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 57.29578;
	let sp = 2.0 * (q.w * q.y - q.z * q.x);
	let pitch = sp.clamp(-1.0, 1.0).asin() * 57.29578;
	let roll = (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * 57.29578;
	(yaw, pitch, roll)
}

#[cfg(test)]
mod tests {
	use super::*;

	fn at(t: f64) -> Sample {
		Sample {
			t,
			pos: V3::default(),
			rot: Q4::default(),
			speed: 0.0,
			sd_mm: 1.0,
			lh_mask: 1,
			res_mrad: -1.0,
			light: Light::Tracking,
			rate: 0.0,
		}
	}

	#[test]
	fn samples_are_thinned_and_forgotten() {
		let mut h = History::default();
		for i in 0..100_000 {
			h.push(at(i as f64 * 0.002)); // 500 a second for 200 s
		}
		// 120 a second at most (every 5th at 2 ms steps: 100 a second), the last 120 s
		let n = h.samples.len();
		assert!((11_900..=12_100).contains(&n), "{n}");
		let t_last = h.samples.back().unwrap().t;
		assert!(t_last - h.samples.front().unwrap().t <= KEEP);
		assert_eq!(h.since(t_last - 1.0).count(), 101);
		// the clock back: the history starts over
		h.push(at(3.0));
		assert_eq!(h.samples.len(), 1);
	}

	#[test]
	fn a_time_that_is_not_a_number_is_left_out() {
		let mut h = History::default();
		h.push(at(1.0));
		h.push(at(f64::NAN));
		h.push(at(f64::INFINITY));
		h.push(at(1.5));
		assert_eq!(h.samples.iter().map(|s| s.t).collect::<Vec<_>>(), [1.0, 1.5]);
		// and what is older than KEEP still goes
		h.push(at(400.0));
		assert_eq!(h.samples.len(), 1);
	}

	#[test]
	fn yaw_pitch_roll_of_quarter_turns() {
		let s = std::f64::consts::FRAC_1_SQRT_2;
		let (y, p, r) = yaw_pitch_roll(&Q4::new(s, 0.0, 0.0, s));
		assert!((y - 90.0).abs() < 1e-3 && p.abs() < 1e-9 && r.abs() < 1e-9);
		let (y, p, r) = yaw_pitch_roll(&Q4::new(s, s, 0.0, 0.0));
		assert!(y.abs() < 1e-9 && p.abs() < 1e-9 && (r - 90.0).abs() < 1e-3);
	}
}
