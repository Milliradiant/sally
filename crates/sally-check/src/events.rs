//! What SteamVR showed going wrong, from its own poses, and what the device's light and radio did at that moment.
//!
//! Three kinds of trouble, as a player sees them:
//! - lost: SteamVR stopped calling the pose valid / running (the device greys out or drops)
//! - froze: the position held still while the device kept turning (SteamVR coasting on the IMU without light), which
//!   usually ends in a jump when the light comes back
//! - jumped: the position moved further in one step than the device's own velocity allows
//!
//! Each is then put against the device's quarter seconds around it: was its data arriving (the radio), which base
//! stations saw it and on how many sensors (the light), and was the light late (the radio holding it back).

use crate::analysis::{Analyzer, Device, BIN_US};
use crate::session::Pose;

/// Position steps smaller than this count as held (m)
const HELD_STEP: f64 = 0.000_02;
/// A hold this long is a freeze (us)
const FREEZE_US: u64 = 80_000;
/// ... when the device turned at least this much meanwhile (rad), or the hold ends in a jump
const FREEZE_TURN: f64 = 0.05;
/// A step this long is a jump (m), when it is also far more than the velocity explains
const JUMP_STEP: f64 = 0.015;
/// Not running for this long is a loss (us)
const LOST_US: u64 = 40_000;
/// Events closer than this are one (us)
const MERGE_US: u64 = 150_000;
/// The radio counts as the cause above this share of IMU samples lost
pub const RADIO_LOST: f64 = 0.3;
/// Light later than this (ms) is held back by the radio
pub const LATE_MS: f32 = 60.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Kind {
	Lost,
	Froze,
	Jumped,
}

impl Kind {
	pub fn word(self) -> &'static str {
		match self {
			Kind::Lost => "lost",
			Kind::Froze => "froze",
			Kind::Jumped => "jumped",
		}
	}
}

/// What the light and the radio say about a moment
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Cause {
	/// the device's data stopped arriving
	Radio,
	/// no station saw it
	NoLight,
	/// one station saw it
	OneStation,
	/// the light arrived late
	Late,
	/// light and radio looked fine
	Unexplained,
}

impl Cause {
	pub fn short(self) -> &'static str {
		match self {
			Cause::Radio => "radio",
			Cause::NoLight => "no light",
			Cause::OneStation => "one station",
			Cause::Late => "late light",
			Cause::Unexplained => "unexplained",
		}
	}
}

/// One moment of trouble
#[derive(Clone, Debug)]
pub struct Event {
	pub kind: Kind,
	pub start_us: u64,
	pub end_us: u64,
	/// the jump at its end (m), if any
	pub jump: f64,
	pub cause: Cause,
	/// in words, for the report
	pub detail: String,
	/// "during" or "right after" a room-wide blackout (every device dark at once), set by the report: then the device
	/// is not what went wrong
	pub room: Option<&'static str>,
}

/// The trouble in one device's SteamVR poses (in time order), with the causes from its light and radio
pub fn find(poses: &[Pose], a: &Analyzer, dev: Option<&Device>) -> Vec<Event> {
	let mut raw: Vec<(Kind, u64, u64, f64)> = Vec::new();
	// lost
	let mut lost_from: Option<u64> = None;
	for p in poses {
		let bad = !(p.valid && p.result == 200);
		match (bad, lost_from) {
			(true, None) => lost_from = Some(p.t_us),
			(false, Some(t0)) => {
				if p.t_us - t0 >= LOST_US {
					raw.push((Kind::Lost, t0, p.t_us, 0.0));
				}
				lost_from = None;
			}
			_ => {}
		}
	}
	if let (Some(t0), Some(last)) = (lost_from, poses.last()) {
		if last.t_us - t0 >= LOST_US {
			raw.push((Kind::Lost, t0, last.t_us, 0.0));
		}
	}
	// held and jumped, on valid poses only
	let valid: Vec<&Pose> = poses.iter().filter(|p| p.valid && p.connected).collect();
	let mut i = 0;
	while i + 1 < valid.len() {
		let (p, q) = (valid[i], valid[i + 1]);
		let step = dist(p.pos, q.pos);
		if step < HELD_STEP {
			// a hold: as long as the steps stay tiny
			let mut j = i + 1;
			while j + 1 < valid.len() && dist(valid[j].pos, valid[j + 1].pos) < HELD_STEP {
				j += 1;
			}
			let (t0, t1) = (valid[i].t_us, valid[j].t_us);
			let turned = angle(valid[i].rot, valid[j].rot);
			let jump = valid.get(j + 1).map_or(0.0, |n| dist(valid[j].pos, n.pos));
			if t1 - t0 >= FREEZE_US && (turned > FREEZE_TURN || jump > JUMP_STEP) {
				raw.push((Kind::Froze, t0, t1, if jump > JUMP_STEP { jump } else { 0.0 }));
			}
			i = j;
			continue;
		}
		let dt = (q.t_us.saturating_sub(p.t_us)) as f64 / 1e6;
		let speed = norm(p.vel).max(norm(q.vel));
		if step > JUMP_STEP && step > 3.0 * speed * dt + 0.005 {
			raw.push((Kind::Jumped, p.t_us, q.t_us, step));
		}
		i += 1;
	}
	raw.sort_by_key(|e| e.1);

	// one event per moment: a freeze ending in a jump, or a loss with a jump after it, is one
	let mut merged: Vec<(Kind, u64, u64, f64)> = Vec::new();
	for e in raw {
		match merged.last_mut() {
			Some(m) if e.1 <= m.2 + MERGE_US => {
				m.0 = m.0.min(e.0);
				m.2 = m.2.max(e.2);
				m.3 = m.3.max(e.3);
			}
			_ => merged.push(e),
		}
	}
	merged
		.into_iter()
		.map(|(kind, start_us, end_us, jump)| {
			let (cause, detail) = explain(a, dev, start_us, end_us);
			Event { kind, start_us, end_us, jump, cause, detail, room: None }
		})
		.collect()
}

/// The device's light and radio from a quarter second before a moment to its end
pub fn explain(a: &Analyzer, dev: Option<&Device>, start_us: u64, end_us: u64) -> (Cause, String) {
	let Some(dev) = dev else {
		return (Cause::Unexplained, "no radio data for this device".into());
	};
	let Some(t0) = a.start_us else {
		return (Cause::Unexplained, "no radio data".into());
	};
	let bin = |t: u64| (t.saturating_sub(t0) / BIN_US) as usize;
	let (b0, b1) = (bin(start_us.saturating_sub(BIN_US)), bin(end_us));
	if b0 >= dev.bins.len() {
		return (Cause::Radio, "no reports from the device at all".into());
	}
	let bins = &dev.bins[b0..=b1.min(dev.bins.len() - 1)];
	let step = dev.imu_step().unwrap_or(192_000.0);
	let (got, lost) = bins.iter().fold((0.0, 0.0), |(g, l), b| (g + b.imu as f64, l + dev.imu_lost(b, step)));
	let lost_share = if got + lost > 0.0 { lost / (got + lost) } else { 1.0 };
	if lost_share > RADIO_LOST {
		return (Cause::Radio, format!("{:.0}% of its data never arrived", lost_share * 100.0));
	}
	// the worst quarter second for light
	let worst = bins.iter().min_by_key(|b| (b.station_count(), b.sensors_lit())).expect("bins");
	let stations: Vec<String> = worst.stations().map(station_name).collect();
	let late = bins.iter().map(|b| b.delay_max).fold(0.0f32, f32::max);
	// gen1 light is not told apart by station: how many sensors it lit is what there is
	if worst.station_count() == 0 && worst.gen1_hits > 0 {
		let lit = worst.sensors_lit();
		return if lit < 4 {
			(Cause::OneStation, format!("gen1 light on only {lit} sensors"))
		} else if late > LATE_MS {
			(Cause::Late, format!("its light arrived up to {late:.0} ms late"))
		} else {
			(Cause::Unexplained, format!("gen1 light on {lit} sensors, data arriving"))
		};
	}
	match worst.station_count() {
		0 => (Cause::NoLight, "no base station saw it".to_string()),
		1 => (Cause::OneStation, format!("only {} saw it, on {} sensors", stations.join(""), worst.sensors_lit())),
		_ if late > LATE_MS => (Cause::Late, format!("its light arrived up to {late:.0} ms late")),
		_ => (
			Cause::Unexplained,
			format!("{} stations saw it ({}), data arriving", worst.station_count().max(1), stations.join(", ")),
		),
	}
}

/// A station as SteamVR numbers its channel (one above the raw channel)
pub fn station_name(channel: usize) -> String {
	format!("ch {}", channel + 1)
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
	norm([a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

fn norm(a: [f64; 3]) -> f64 {
	(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// The angle between two orientations (w, x, y, z)
fn angle(a: [f64; 4], b: [f64; 4]) -> f64 {
	let d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).abs().min(1.0);
	2.0 * d.acos()
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::analysis::Analyzer;

	fn pose(t_ms: u64, x: f64, yaw: f64) -> Pose {
		Pose {
			t_us: t_ms * 1000,
			result: 200,
			valid: true,
			connected: true,
			pos: [x, 1.0, 0.0],
			rot: [(yaw / 2.0).cos(), 0.0, (yaw / 2.0).sin(), 0.0],
			..Pose::default()
		}
	}

	#[test]
	fn a_hold_while_turning_then_a_jump_is_one_freeze() {
		let a = Analyzer::new(Box::new(|_| None));
		let mut p = Vec::new();
		// moving, then held for 200 ms while turning, then 5 cm away at once
		for k in 0..50 {
			p.push(pose(k * 4, k as f64 * 0.001, 0.0));
		}
		for k in 50..100 {
			p.push(pose(k * 4, 0.049, (k - 50) as f64 * 0.01));
		}
		for k in 100..120 {
			p.push(pose(k * 4, 0.1 + (k - 100) as f64 * 0.001, 0.5));
		}
		let e = find(&p, &a, None);
		assert_eq!(e.len(), 1, "{e:?}");
		assert_eq!(e[0].kind, Kind::Froze);
		assert!(e[0].jump > 0.04);
	}

	#[test]
	fn standing_still_is_not_a_freeze() {
		let a = Analyzer::new(Box::new(|_| None));
		let p: Vec<Pose> = (0..500).map(|k| pose(k * 4, 0.3, 0.2)).collect();
		assert!(find(&p, &a, None).is_empty());
	}
}
