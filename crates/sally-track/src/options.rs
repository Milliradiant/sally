//! The tracker's options: the 44 `track-*` settings with their descriptions and defaults, in the order of the settings'
//! fields, which is the order they are registered in. Their earlier `core2-*` names are still read (sally-runtime's
//! `Config::current_name`). sally-runtime's registry test holds them to tests/data/config/registry.txt: each option's
//! type, tag, default and description, in order.

use crate::host::Settings;

/// An option's value, in its C type: an FLT ('f') or an int ('i')
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OptionValue {
	Float(f64),
	Int(i32),
}

impl OptionValue {
	/// The value as a float (an int converted)
	pub fn float(self) -> f64 {
		match self {
			OptionValue::Float(v) => v,
			OptionValue::Int(v) => v as f64,
		}
	}
	/// The value as an int (a float truncated, as a C cast does)
	pub fn int(self) -> i32 {
		match self {
			OptionValue::Int(v) => v,
			OptionValue::Float(v) => v as i32,
		}
	}
}

/// One option: its tag, its description (for the help), its default, and the default's text as registered
#[derive(Clone, Copy, Debug)]
pub struct TrackerOption {
	pub tag: &'static str,
	pub description: &'static str,
	pub default: OptionValue,
	pub default_text: &'static str,
}

/// The options, in the order of `Settings`' fields
pub const OPTIONS: [TrackerOption; 44] = [
	TrackerOption {
		tag: "track-hit-sd",
		description: "sally tracker: sd of one hit's angle (rad) at normal incidence, on top of track-hit-common-sd",
		default: OptionValue::Float(2e-4),
		default_text: "2e-4",
	},
	TrackerOption {
		tag: "track-hit-common-sd",
		description: concat!(
			"sally tracker: sd of the white angle error common to all hits of one sweep (rad; measured 0.13 ",
			"mrad sweep to sweep against the all-station track). The common error is 0.5 mrad rms in all, but ",
			"most of it is slow (0.3-0.55 mrad, varying over seconds with where the device is in the station's ",
			"field: the station model), which a white term must not stand in for",
		),
		default: OptionValue::Float(1.5e-4),
		default_text: "1.5e-4",
	},
	TrackerOption {
		tag: "track-hit-join-sd",
		description: concat!(
			"sally tracker: sd of the angle error common to a sweep's hits when the station has only just come ",
			"into view (rad), fading with exp(-t / JOIN_TAU) and exp(-turned / JOIN_DEG) after its light ",
			"returned (no sweep from it for JOIN_GAP s before), and only while another station holds the pose ",
			"(accepted hits within JOIN_HOLD s). A station coming into view lights the sensors at the edge of ",
			"the device's visible side first, partly shadowed (by the body, by the housing): its first sweeps ",
			"are off by a common 1-3 mrad (0.4-0.5 after 0.25 s, 0.3-0.5 steady); taken at the steady weight ",
			"they yank the estimate by a cm on every handover. 0 = off",
		),
		default: OptionValue::Float(1.1e-3),
		default_text: "1.1e-3",
	},
	TrackerOption {
		tag: "track-hit-knee",
		description: concat!(
			"sally tracker: the hit sd grows as 1 + (incidence angle / this)^6 (deg): 0.2 mrad below 70 deg, ",
			"0.45 at 75, 1.2-1.9 beyond 80",
		),
		default: OptionValue::Float(70.0),
		default_text: "70.",
	},
	TrackerOption {
		tag: "track-gate",
		description: "sally tracker: a hit is dropped when its residual exceeds this many predicted innovation sds",
		default: OptionValue::Float(8.0),
		default_text: "8.",
	},
	TrackerOption {
		tag: "track-gate-floor",
		description: concat!(
			"sally tracker: ... or this many rad, whichever is larger (a floor so a confident filter still takes ",
			"hits after a short blackout)",
		),
		default: OptionValue::Float(3e-3),
		default_text: "3e-3",
	},
	TrackerOption {
		tag: "track-zupt-sd",
		description: concat!(
			"sally tracker: while the IMU says the device is at rest (|gyro| < track-zupt-gyro rad/s and the ",
			"accelerometer's fast part < track-zupt-vib m/s^2 for track-zupt-time s) a zero-velocity measurement ",
			"with this sd (m/s) is applied every sample: the one thing that holds the depth with one station when",
			" the device does not move (a foot on the floor). <= 0 = off",
		),
		default: OptionValue::Float(0.01),
		default_text: "0.01",
	},
	TrackerOption {
		tag: "track-zupt-gyro",
		description: "sally tracker: see track-zupt-sd (rad/s)",
		default: OptionValue::Float(0.02),
		default_text: "0.02",
	},
	TrackerOption {
		tag: "track-zupt-vib",
		description: "sally tracker: see track-zupt-sd (m/s^2)",
		default: OptionValue::Float(0.05),
		default_text: "0.05",
	},
	TrackerOption {
		tag: "track-zupt-time",
		description: "sally tracker: see track-zupt-sd (s)",
		default: OptionValue::Float(0.2),
		default_text: "0.2",
	},
	TrackerOption {
		tag: "track-hit-shift",
		description: concat!(
			"sally tracker: seconds added to every hit's time to put it on the IMU clock (the light leads the IMU ",
			"by ~4.5 ms on the trackers)",
		),
		default: OptionValue::Float(0.0045),
		default_text: "0.0045",
	},
	TrackerOption {
		tag: "track-acc-sd",
		description: "sally tracker: accelerometer process noise (m/s^2 per step at 250 Hz): base velocity random walk",
		default: OptionValue::Float(0.05),
		default_text: "0.05",
	},
	TrackerOption {
		tag: "track-acc-knoise",
		description: concat!(
			"sally tracker: accelerometer process noise proportional to the specific force it reads beyond ",
			"gravity: sd = sqrt(acc-sd^2 + (this * max(0, |f| - g))^2 (+ the vibration term)). The sensor is ",
			"off by a fraction of what it measures (scale and cross-axis errors, the point it sits at under the ",
			"body rate, its timing against the light), and the fraction is of the motion, not of gravity, whose ",
			"constant error the bias state holds. Flat from 0.08 to 0.12 on rigid-body truths; applied to |f| ",
			"itself it adds snaps at rest. 0 = off",
		),
		default: OptionValue::Float(0.1),
		default_text: "0.1",
	},
	TrackerOption {
		tag: "track-gyr-sd",
		description: concat!(
			"sally tracker: orientation process noise (rad/s per step at 250 Hz): the gyro's own noise. What ",
			"the gyro does not deliver is modelled where it arises: its scale error turns the estimate about ",
			"the axis it turns about (GYRO_SCALE), and the light's orientation carries an offset from the ",
			"sensor calibration of the face it sees (OB_SD). Raised to stand in for both (0.1), the orientation ",
			"follows the light's offset and the tilt leaks gravity along the one line a single station cannot ",
			"see; at 0.01 one-station tracking is worse at rest and a fast spin loses the orientation",
		),
		default: OptionValue::Float(0.003),
		default_text: "0.003",
	},
	TrackerOption {
		tag: "track-acc-bias-sd",
		description: "sally tracker: accelerometer bias random walk (m/s^2 per step)",
		default: OptionValue::Float(0.01),
		default_text: "0.01",
	},
	TrackerOption {
		tag: "track-gyr-bias-sd",
		description: "sally tracker: gyro bias random walk (rad/s per step)",
		default: OptionValue::Float(5e-4),
		default_text: "5e-4",
	},
	TrackerOption {
		tag: "track-layout-sd",
		description: concat!(
			"sally tracker: sensor-layout scale random walk (per step); the scale fixes the one-station depth ",
			"bias of a device whose layout is a little off (+0.5 % on some devices). <= 0 = no scale state",
		),
		default: OptionValue::Float(1e-4),
		default_text: "1e-4",
	},
	TrackerOption {
		tag: "track-layout-max",
		description: "sally tracker: the layout scale is clamped to +- this (the real values are 0.1-0.6 %)",
		default: OptionValue::Float(0.015),
		default_text: "0.015",
	},
	TrackerOption {
		tag: "track-adapt",
		description: concat!(
			"sally tracker (controllers only unless track-adapt-trackers): accelerometer noise adaptive to the ",
			"measured vibration: sd = sqrt(acc-sd^2 + (this * max(0, vib - track-vib0))^2), vib = rms of the ",
			"fast part of the accelerometer over the last 40 ms. On a vibrating mount (0.8-1.7 m/s^2 rms) a ",
			"fixed noise lets the velocity follow the vibration. 0 = fixed",
		),
		default: OptionValue::Float(6.0),
		default_text: "6.",
	},
	TrackerOption {
		tag: "track-acc-tnoise",
		description: concat!(
			"sally tracker: the accelerometer's timing against the gyro's, sd (s): accelerometer process noise ",
			"|f| |w| this per step (the acceleration vector turns at the body rate, so a reading a little off ",
			"its time is off by that much). 6 and 10 ms hold a tracker whirled at 25-34 rad/s, 3 does not. 0 = ",
			"off",
		),
		default: OptionValue::Float(0.010),
		default_text: "0.010",
	},
	TrackerOption {
		tag: "track-adapt-trackers",
		description: "sally tracker: track-adapt on the trackers too (1), or on the controllers only (0)",
		default: OptionValue::Int(0),
		default_text: "0",
	},
	TrackerOption {
		tag: "track-vib0",
		description: "sally tracker: vibration below this (m/s^2 rms) is not counted",
		default: OptionValue::Float(0.1),
		default_text: "0.1",
	},
	TrackerOption {
		tag: "track-facing-deg",
		description: concat!(
			"sally tracker: a hit on a sensor whose normal is more than this many degrees from the direction to ",
			"the station is dropped (a sensor cannot see light from behind; what arrives there is a reflection, ",
			"and the hits at 90-110 deg measured 1-2 mrad off)",
		),
		default: OptionValue::Float(90.0),
		default_text: "90.",
	},
	TrackerOption {
		tag: "track-blink",
		description: concat!(
			"sally tracker, bench measurement only: a one-station handover test. Every track-blink + 2.5 s the ",
			"device keeps for track-blink s only the station that gave it the most accepted hits in the 0.1 s ",
			"before (the others' sweeps are dropped, as with track-only), then all stations again. Replayed ",
			"beside the same run without it, the error along the kept station's line of sight at the window's end",
			" and the correction when the others return are the handover snap, against a truth, on every device ",
			"and recording (TRACK_BLINK line per window). Negative: a blackout of -track-blink s instead, every station ",
			"dropped (dead reckoning, where some station was lit). 0 = off",
		),
		default: OptionValue::Float(0.0),
		default_text: "0.",
	},
	TrackerOption {
		tag: "track-probe-ch",
		description: concat!(
			"sally tracker, bench measurement only: the base station CHANNEL whose sweeps are evaluated against ",
			"the state (TRACK_HIT / TRACK_DEPTH at track-verbose 3) but never used, neither in the update nor in a seed solve: ",
			"its residuals then carry nothing the state absorbed from them. -1 = off",
		),
		default: OptionValue::Int(-1),
		default_text: "-1",
	},
	TrackerOption {
		tag: "track-only",
		description: concat!(
			"sally tracker: bit mask of base station CHANNELS the tracker may use (bit n = channel n), 0 = all. For the",
			" one-station bench",
		),
		default: OptionValue::Int(0),
		default_text: "0",
	},
	TrackerOption {
		tag: "track-reseed-frac",
		description: concat!(
			"sally tracker: the state is lost (and re-solved from the hits) when more than this fraction of the ",
			"hits of the last 0.5 s were rejected",
		),
		default: OptionValue::Float(0.8),
		default_text: "0.8",
	},
	TrackerOption {
		tag: "track-gap-coast",
		description: concat!(
			"sally tracker: seconds without an accepted hit after which the device coasts: the accelerometer no ",
			"longer drives the velocity, which decays with track-gap-tau (dead reckoning on the accelerometer ",
			"alone runs off by metres in a few seconds of blackout). 0.6 s: a body keeps moving through a ",
			"blackout and the accelerometer follows it; longer costs a hidden tracker's frozen point (36 mm ",
			"from its last light at 0.3 s, 137 at 0.6, 353 at 1.0). <= 0 = off",
		),
		default: OptionValue::Float(0.6),
		default_text: "0.6",
	},
	TrackerOption {
		tag: "track-gap-tau",
		description: "sally tracker: velocity decay time constant (s) while coasting",
		default: OptionValue::Float(0.5),
		default_text: "0.5",
	},
	TrackerOption {
		tag: "track-gap-reseed",
		description: concat!(
			"sally tracker: after this many seconds without an accepted hit, a returning sweep that the coasted ",
			"state does not fit (median |residual| over track-seed-fit) means the state is lost: it is re-solved ",
			"from the hits at once rather than after the half-second divergence test. <= 0 = off",
		),
		default: OptionValue::Float(0.5),
		default_text: "0.5",
	},
	TrackerOption {
		tag: "track-seed-fit",
		description: concat!(
			"sally tracker: a seed solve (the start, or the restart after a loss) is taken only when its pose fits",
			" its own hits to this median |residual| (rad); a wrong solution (a mirror pose, a reflected station)",
			" sits at tens of mrad, a right one at the hit noise. Also the fit of a returning sweep to the ",
			"coasted state (track-gap-reseed)",
		),
		default: OptionValue::Float(3e-3),
		default_text: "3e-3",
	},
	TrackerOption {
		tag: "track-seed-window",
		description: concat!(
			"sally tracker: the seed solve fits position, orientation and velocity to the sweeps of the last this ",
			"many seconds (constant velocity over the window, the gyro carrying the orientation between the ",
			"sweeps): long enough for both planes of a station and some motion, short enough for the acceleration",
			" not to matter (at 1 g 0.06 s is 18 mm at the edge of the window)",
		),
		default: OptionValue::Float(0.06),
		default_text: "0.06",
	},
	TrackerOption {
		tag: "track-clip-sd",
		description: concat!(
			"sally tracker: accelerometer process noise (m/s^2 per step) while an axis is at full scale ",
			"(CLIP_G). A clipped sample is a lower bound, not nothing: the true value is on average a few tens ",
			"of m/s^2 above the range (a tracker at 14-18 rad/s on a 0.35 m radius: 70-115 m/s^2 against the 78 ",
			"it reads), and a sample taken as unknown (100) lets a few poor sweeps carry the velocity. 20 to 50 ",
			"do alike; 10 loses a tracker in a fast spin",
		),
		default: OptionValue::Float(30.0),
		default_text: "30.",
	},
	TrackerOption {
		tag: "track-reseed-min-hits",
		description: "sally tracker: ... and there were at least this many hits",
		default: OptionValue::Int(200),
		default_text: "200",
	},
	TrackerOption {
		tag: "track-p0-pos",
		description: concat!(
			"sally tracker: position variance at a seed (m^2) is at least this (the seed solve's own covariance, ",
			"if larger)",
		),
		default: OptionValue::Float(1e-4),
		default_text: "1e-4",
	},
	TrackerOption {
		tag: "track-p0-vel",
		description: concat!(
			"sally tracker: velocity variance (m^2/s^2) at a seed is at least this (the seed solve's own ",
			"covariance if larger: with one station the velocity along the line of sight is not observed and ",
			"keeps the solve's 3 m/s prior)",
		),
		default: OptionValue::Float(0.1),
		default_text: "0.1",
	},
	TrackerOption {
		tag: "track-p0-rot",
		description: "sally tracker: orientation variance (rad^2) at a seed is at least this",
		default: OptionValue::Float(1e-3),
		default_text: "1e-3",
	},
	TrackerOption {
		tag: "track-p0-gyr-bias",
		description: "sally tracker: initial gyro bias variance (rad^2/s^2)",
		default: OptionValue::Float(1e-4),
		default_text: "1e-4",
	},
	TrackerOption {
		tag: "track-p0-acc-bias",
		description: "sally tracker: initial accelerometer bias variance (m^2/s^4)",
		default: OptionValue::Float(1e-2),
		default_text: "1e-2",
	},
	TrackerOption {
		tag: "track-p0-layout",
		description: "sally tracker: initial layout scale variance",
		default: OptionValue::Float(1e-4),
		default_text: "1e-4",
	},
	TrackerOption {
		tag: "track-gravity-tilt",
		description: concat!(
			"sally tracker: where gravity points in the map's frame: its tilt from the map's z (rad), toward ",
			"the azimuth track-gravity-tilt-az (deg from +x toward +y; both positive, a value starting with '-' ",
			"is taken for an option). The map's frame is the calibrated stations' and owes nothing to gravity: ",
			"a map can be 10 mrad off, and with gravity taken along z every device's orientation sits that far ",
			"from the light's, the light's residuals carry it times the sensors' lever, and a tracker's ",
			"position wanders by millimetres in its own frame over a turn",
		),
		default: OptionValue::Float(0.0),
		default_text: "0.",
	},
	TrackerOption {
		tag: "track-gravity-tilt-az",
		description: "sally tracker: see track-gravity-tilt (deg)",
		default: OptionValue::Float(0.0),
		default_text: "0.",
	},
	TrackerOption {
		tag: "track-gravity-tilt-sd",
		description: concat!(
			"sally tracker: the tilt above as two filter states with this initial sd (rad), learned from the ",
			"light's orientation against the accelerometer as the device turns (a body-fixed bias cannot follow a",
			" world-fixed direction; at rest the two cannot be told apart, so the states only move while the ",
			"device turns). No process noise: the map's gravity does not change. The estimate is printed with the",
			" summary. 0 = fixed at the values given",
		),
		default: OptionValue::Float(0.0),
		default_text: "0.",
	},
	TrackerOption {
		tag: "track-verbose",
		description: concat!(
			"sally tracker: 1 = log seeds / reseeds, 2 = TRACK_SWEEP and TRACK_UPDATE lines per sweep and TRACK_BIAS per ",
			"second in the recording, 3 = also TRACK_HIT per hit and TRACK_IMU per IMU sample",
		),
		default: OptionValue::Int(1),
		default_text: "1",
	},
];

impl Settings {
	/// The settings with each option's value from `value` (called in the table's order)
	pub fn from_options(mut value: impl FnMut(&TrackerOption) -> OptionValue) -> Settings {
		let v: [OptionValue; 44] = std::array::from_fn(|i| value(&OPTIONS[i]));
		Settings {
			sig_hit: v[0].float(),
			sig_common: v[1].float(),
			sig_join: v[2].float(),
			hit_knee: v[3].float(),
			gate: v[4].float(),
			gate_floor: v[5].float(),
			zupt_sd: v[6].float(),
			zupt_gyro: v[7].float(),
			zupt_vib: v[8].float(),
			zupt_time: v[9].float(),
			hit_shift: v[10].float(),
			sig_acc: v[11].float(),
			acc_knoise: v[12].float(),
			sig_gyr: v[13].float(),
			sig_ba: v[14].float(),
			sig_bg: v[15].float(),
			sig_ls: v[16].float(),
			ls_max: v[17].float(),
			adapt: v[18].float(),
			acc_tnoise: v[19].float(),
			adapt_trackers: v[20].int(),
			vib0: v[21].float(),
			facing_deg: v[22].float(),
			blink: v[23].float(),
			probe_ch: v[24].int(),
			only_mask: v[25].int(),
			reseed_frac: v[26].float(),
			gap_coast: v[27].float(),
			gap_tau: v[28].float(),
			gap_reseed: v[29].float(),
			seed_fit: v[30].float(),
			seed_window: v[31].float(),
			clip_sd: v[32].float(),
			reseed_min_hits: v[33].int(),
			p0_pos: v[34].float(),
			p0_vel: v[35].float(),
			p0_rot: v[36].float(),
			p0_bg: v[37].float(),
			p0_ba: v[38].float(),
			p0_ls: v[39].float(),
			gt_mag: v[40].float(),
			gt_az: v[41].float(),
			gt_sd: v[42].float(),
			verbose: v[43].int(),
		}
	}

	/// The defaults (the values a device's tracker starts with when no option is set)
	pub fn defaults() -> Settings {
		Settings::from_options(|o| o.default)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn defaults_are_the_c_literals() {
		for o in &OPTIONS {
			match o.default {
				OptionValue::Float(v) => assert_eq!(v, o.default_text.parse::<f64>().unwrap(), "{}", o.tag),
				OptionValue::Int(v) => assert_eq!(v, o.default_text.parse::<i32>().unwrap(), "{}", o.tag),
			}
		}
		let s = Settings::defaults();
		assert_eq!((s.sig_hit, s.verbose, s.probe_ch, s.acc_tnoise), (2e-4, 1, -1, 0.010));
	}
}
