//! The gen2 light's timing, from a station's syncs and a sensor's sweep hit to the rotor angle of the hit and the sweep
//! plane it belongs to (the arithmetic of survive_process_gen2.c's sync and sweep processes and determine_plane,
//! operation for operation; the runtime keeps the state and does the rest of those processes around it).

use sally_math::{linmath_max, linmath_min, LINMATHPI};

use crate::NUM_GEN2_LIGHTHOUSES;

/// freq_per_channel: each channel's rotor frequency (Hz)
pub const FREQ_PER_CHANNEL: [f64; NUM_GEN2_LIGHTHOUSES] = [
	50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182, 51.2273, 51.6685, 52.2307, 52.6894,
	52.9217, 53.2741, 53.7514, 54.1150,
];

/// survive_timecode_difference: the distance between two 32-bit timecodes, the short way round
pub fn timecode_difference(most_recent: u32, least_recent: u32) -> u32 {
	let diff = most_recent.abs_diff(least_recent);
	if diff > 0xFFFFFFFF / 2 {
		return 0xFFFFFFFF - diff;
	}
	diff
}

/// A channel's nominal rotation in ticks, as a survive_timecode (`48000000. / freq` truncated)
fn predicted_time(channel: usize) -> u32 {
	(48000000.0 / FREQ_PER_CHANNEL[channel]) as u32
}

/// What survive_default_sync_process works out from the time since a station's last sync
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SyncInterval {
	/// whole rotations in the interval (rounded); the syncs skipped are one fewer
	pub rotations_since: i32,
	/// the interval per rotation (the whole interval over the rotations it spanned)
	pub time_delta: u32,
	/// the rotor frequency that gives, and its distance from the channel's
	pub hz: f64,
	pub err: f64,
}

/// The interval between a station's syncs: `time_delta` ticks on `channel`
pub fn sync_interval(time_delta: u32, channel: usize) -> SyncInterval {
	let predicted_time = predicted_time(channel);
	let rotations_since = (time_delta.wrapping_add(predicted_time / 2) / predicted_time) as i32;
	let mut time_delta = time_delta;
	// sally: the rotor period over a gap is the whole interval divided by the rotations it spanned (see the C)
	if rotations_since > 1 {
		time_delta = time_delta.wrapping_add((rotations_since / 2) as u32) / rotations_since as u32;
	}
	let hz = 48000000.0 / time_delta as f64;
	let err = (hz - FREQ_PER_CHANNEL[channel]).abs();
	SyncInterval { rotations_since, time_delta, hz, err }
}

/// A hit's place in its station's rotation (survive_default_sweep_process)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweepTiming {
	/// whole rotations since the sync the hit is timed from (more than 10: the hit is dropped)
	pub rotations_since: i32,
	/// seconds into the current rotation
	pub time_since_sync: f64,
	/// the rotor angle (rad, 0 at the sync)
	pub angle: f64,
}

/// A hit `time_delta` ticks after its station's last sync on `channel` (the half-clock flag adds half a tick), with
/// the last measured sync interval `last_time_between_sync` (the nominal rotation when that is more than 1 Hz off)
pub fn sweep_timing(time_delta: u32, half_clock: bool, last_time_between_sync: u32, channel: usize) -> SweepTiming {
	let mut time_since_sync = time_delta as f64 / 48000000.0;
	if half_clock {
		time_since_sync += 0.5 / 48000000.0;
	}
	let mut hz = 48000000.0 / last_time_between_sync as f64;
	if (hz - FREQ_PER_CHANNEL[channel]).abs() > 1.0 {
		hz = FREQ_PER_CHANNEL[channel];
	}
	let rotations_since = (time_delta / predicted_time(channel)) as i32;
	let time_per_rot = 1.0 / hz;
	time_since_sync -= time_per_rot * rotations_since as f64;
	let angle = time_since_sync / time_per_rot * 2.0 * LINMATHPI;
	SweepTiming { rotations_since, time_since_sync, angle }
}

/// The angle on each sweep plane a rotor angle stands for (the planes are 120 and 240 degrees round the rotor)
pub fn angle_for_axis(angle: f64) -> [f64; 2] {
	[angle - 2.0 / 3.0 * LINMATHPI, angle - 4.0 / 3.0 * LINMATHPI]
}

/// determine_plane's counters (SurviveObject's stats)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlaneStats {
	/// near the split, the means picked the plane the plain rule would not
	pub extent_hits: u32,
	/// right at the split, nothing picked a plane (the hit is dropped)
	pub extent_misses: u32,
	/// near the split, the means agreed with the plain rule
	pub naive_hits: u32,
	/// the furthest the means moved the split, below and above pi
	pub min_extent: f64,
	pub max_extent: f64,
}

/// determine_plane without the pose (naive-plane-only, or no usable prediction): the plain rule splits the rotor at pi;
/// within 10 % of pi the plane whose mean angle at the last sync (`center`, the station's angles_center_x) lies within
/// 0.1 rad decides, and within 5 % with neither, the hit gets no plane (-1). `naive_plane_only` returns the plain rule
/// alone.
pub fn plane_from_angle(angle: f64, center: &[f64; 2], naive_plane_only: bool, stats: &mut PlaneStats) -> i8 {
	let naive_plane = (angle > LINMATHPI) as i8;
	if naive_plane_only {
		return naive_plane;
	}
	let mut plane = naive_plane;
	let m = 0.05;
	let border_line = angle > LINMATHPI * (1.0 - 2.0 * m) && angle < LINMATHPI * (1.0 + 2.0 * m);
	let very_border_line = angle > LINMATHPI * (1.0 - m) && angle < LINMATHPI * (1.0 + m);
	let angle_for_axis = angle_for_axis(angle);
	if border_line {
		if very_border_line {
			plane = -1;
		}
		let mut err = [0.0f64; 2];
		for i in 0..2 {
			// fabsf((float)...): the distance in floats
			err[i] = ((angle_for_axis[i] - center[i]) as f32).abs() as f64;
			if err[i].is_nan() {
				err[i] = 1.0;
			}
		}
		if err[0] <= 0.1 || err[1] <= 0.1 {
			plane = (err[1] < err[0]) as i8;
		}
		if plane != -1 && naive_plane != plane {
			stats.extent_hits = stats.extent_hits.wrapping_add(1);
			stats.min_extent = linmath_min(angle - LINMATHPI, stats.min_extent);
			stats.max_extent = linmath_max(angle - LINMATHPI, stats.max_extent);
		} else if plane == -1 {
			stats.extent_misses = stats.extent_misses.wrapping_add(1);
		} else if plane == naive_plane {
			stats.naive_hits = stats.naive_hits.wrapping_add(1);
		}
	}
	plane
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn timecode_difference_goes_the_short_way() {
		assert_eq!(timecode_difference(10, 3), 7);
		assert_eq!(timecode_difference(3, 10), 7);
		assert_eq!(timecode_difference(5, 0xFFFF_FFFA), 0xFFFF_FFFF - (0xFFFF_FFFA - 5));
	}

	#[test]
	fn a_gap_is_shared_over_its_rotations() {
		let p = predicted_time(0);
		assert_eq!(p, 959000);
		let s = sync_interval(3 * 959001, 0);
		assert_eq!(s.rotations_since, 3);
		assert_eq!(s.time_delta, 959001);
		assert!(s.err < 1.0);
	}

	#[test]
	fn the_plain_rule_splits_at_pi() {
		let mut st = PlaneStats::default();
		let c = [f64::NAN; 2];
		assert_eq!(plane_from_angle(1.0, &c, false, &mut st), 0);
		assert_eq!(plane_from_angle(5.0, &c, false, &mut st), 1);
		// right at the split with no means: no plane
		assert_eq!(plane_from_angle(LINMATHPI, &c, false, &mut st), -1);
		assert_eq!(st.extent_misses, 1);
		// near the split, a mean decides: plane 0's angle there is pi/3
		let c = [LINMATHPI * 1.08 - 2.0 / 3.0 * LINMATHPI, 9.0];
		assert_eq!(plane_from_angle(LINMATHPI * 1.08, &c, false, &mut st), 0);
		assert_eq!(st.extent_hits, 1);
	}
}
