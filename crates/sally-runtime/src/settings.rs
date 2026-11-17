//! The options the runtime's processes read, as the C reads them: each with its C lookup (survive_configi / f / b / s
//! with SC_GET and the default the C passes there). The C reads most of them once, into a static, at their first use;
//! nothing changes them before that, so they are read here once, when the runtime is made. Those an option set live
//! can change, and those whose read writes the config, are read where the C reads them; the context's own three are
//! bound to their options (`StationMoves`), as the C binds them.

use sally_lighthouse::activations::Params;
use sally_track::host::Settings as TrackerSettings;
use sally_track::options::OptionValue;

use crate::config::{Binding, Config, VarType, SC_GET};

/// The settings of the runtime's processes
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
	/// v: the log level (verbose lines are not written)
	pub log_level: i32,
	/// configed-lighthouse-gen - 1: the generation the config file was valid for (-1 none)
	pub lh_version_configed: i32,
	/// lighthouse-gen - 1: the generation forced (-1 detect)
	pub lh_version_forced: i32,
	/// blacklist-devs: devices not to create (a comma-separated list; "-" in survive_create_device when unset)
	pub blacklist_devs: String,
	/// naive-plane-only: the plane of a gen2 hit by the plain rule alone
	pub naive_plane_only: bool,
	/// gen2-plane-predict / -tol: the plane from the predicted pose near the plain rule's limit (<= 0 off)
	pub gen2_plane_predict: f64,
	pub gen2_plane_predict_tol: f64,
	/// light-facing-deg / -max-age: drop hits on sensors facing away from their station (<= 0 off)
	pub light_facing_deg: f64,
	pub light_facing_max_age: f64,
	/// record-facing: a FAC line per gen2 hit
	pub record_facing: bool,
	/// sim-gap-period / -len / -start: bench only, simulated occlusions (period 0 off)
	pub sim_gap_period: f64,
	pub sim_gap_len: f64,
	pub sim_gap_start: f64,
	/// serialize-ootx (== 1): write each OOTX packet to LH<ch>_<id>.ootx
	pub serialize_ootx: bool,
	/// serialize-device-config: write each device's config to <serial>_config.json and <codename>_config.json
	pub serialize_device_config: bool,
	/// imu-acc-bias-units: the device config's acc_bias in m/s^2 when 1 (else upstream's guess: ignored)
	pub imu_acc_bias_units: u32,
	/// ignore-config-imu-bias: the device config's IMU biases zeroed
	pub ignore_config_imu_bias: bool,
	/// the light's outlier filter and the movement thresholds (move-threshold-*, filter-*), each device's activations
	/// starting with them
	pub activations: Params,
	/// report-in-imu: the pose process gets the IMU's pose instead of the tracked point's (a debug option)
	pub report_in_imu: bool,
	/// track-*: the tracker's options, each device's tracker starting with them
	pub tracker: TrackerSettings,
}

impl Settings {
	/// The settings in `config` (registered, with the command line, the environment and the config file read in)
	pub fn from_config(c: &mut Config) -> Settings {
		let s = |c: &mut Config, tag: &str, def: &[u8]| {
			String::from_utf8_lossy(&c.configs(tag, SC_GET, Some(def)).unwrap_or_default()).into_owned()
		};
		Settings {
			log_level: c.configi("v", SC_GET, 0) as i32,
			// survive_init: survive_configi(..) - 1, a uint32 into an int
			lh_version_configed: c.configi("configed-lighthouse-gen", SC_GET, 0).wrapping_sub(1) as i32,
			lh_version_forced: c.configi("lighthouse-gen", SC_GET, 0).wrapping_sub(1) as i32,
			blacklist_devs: s(c, "blacklist-devs", b"-"),
			naive_plane_only: c.configi("naive-plane-only", SC_GET, 0) != 0,
			gen2_plane_predict: c.configf("gen2-plane-predict", SC_GET, -1.0),
			gen2_plane_predict_tol: c.configf("gen2-plane-predict-tol", SC_GET, 0.5),
			light_facing_deg: c.configf("light-facing-deg", SC_GET, -1.0),
			light_facing_max_age: c.configf("light-facing-max-age", SC_GET, 0.5),
			record_facing: c.configi("record-facing", SC_GET, 0) != 0,
			sim_gap_period: c.configf("sim-gap-period", SC_GET, 0.0),
			sim_gap_len: c.configf("sim-gap-len", SC_GET, 2.0),
			sim_gap_start: c.configf("sim-gap-start", SC_GET, 20.0),
			serialize_ootx: c.configi("serialize-ootx", SC_GET, 0) == 1,
			serialize_device_config: c.configi("serialize-device-config", SC_GET, 0) != 0,
			imu_acc_bias_units: c.configi("imu-acc-bias-units", SC_GET, 0),
			ignore_config_imu_bias: c.configb("ignore-config-imu-bias", SC_GET, false),
			activations: activation_params(c),
			report_in_imu: c.configi("report-in-imu", SC_GET, 0) != 0,
			tracker: tracker_settings(c),
		}
	}
}

/// SurviveContext's bound settings (its STRUCT_CONFIG_SECTION, bound by SurviveContext_attach_config in survive_init
/// and unbound in survive_close): a known station's new pose is taken at once when it moved less than
/// lighthouse-max-update or more than lighthouse-max-nudge-distance (m), and slewed to at lighthouse-update-velocity
/// (m/s) in between
#[derive(Clone, Debug)]
pub struct StationMoves {
	pub max_update: Binding,
	pub max_nudge_distance: Binding,
	pub update_velocity: Binding,
}

impl StationMoves {
	/// The section's items, in the C's order
	pub fn items(&self) -> [(&'static str, &Binding); 3] {
		[
			("lighthouse-max-update", &self.max_update),
			("lighthouse-max-nudge-distance", &self.max_nudge_distance),
			("lighthouse-update-velocity", &self.update_velocity),
		]
	}

	/// SurviveContext_attach_config: each variable bound, taking its option's value
	pub fn attach(c: &mut Config) -> StationMoves {
		let m = StationMoves {
			max_update: Binding::of(VarType::Float),
			max_nudge_distance: Binding::of(VarType::Float),
			update_velocity: Binding::of(VarType::Float),
		};
		for (tag, var) in m.items() {
			c.attach(tag, var);
		}
		m
	}
}

/// The light filter's options (SurviveSensorActivations' STRUCT_CONFIG_SECTION), as a device's activations bind them
/// when the device is made (SurviveSensorActivations_attach_config, in survive_create_device): the value each tag has
/// in `c`, or its registered default
pub(crate) fn activation_params(c: &mut Config) -> Params {
	Params {
		move_threshold_gyro: c.configf("move-threshold-gyro", SC_GET, 0.0),
		move_threshold_acc: c.configf("move-threshold-acc", SC_GET, 0.0),
		move_threshold_ang: c.configf("move-threshold-ang", SC_GET, 0.0),
		filter_light_change: c.configf("filter-threshold-ang-per-sec", SC_GET, 0.0),
		filter_outlier_criteria: c.configf("filter-light-outlier-criteria", SC_GET, 0.0),
		filter_variance_min: c.configf("filter-variance-minimum", SC_GET, 0.0),
		filter_outlier_min_count: c.configi("filter-outlier-minimum-count", SC_GET, 0) as i32,
	}
}

/// The light filter's options' tags
pub(crate) const ACTIVATION_OPTIONS: [&str; 7] = [
	"move-threshold-gyro",
	"move-threshold-acc",
	"move-threshold-ang",
	"filter-threshold-ang-per-sec",
	"filter-light-outlier-criteria",
	"filter-variance-minimum",
	"filter-outlier-minimum-count",
];

/// SurviveCore2RsSettings_attach_config's reads: the tracker's options as a device's tracker gets them when it is made
/// (survive_c2_create), each the value its tag has in `c`, or its registered default
pub(crate) fn tracker_settings(c: &mut Config) -> TrackerSettings {
	TrackerSettings::from_options(|o| match o.default {
		OptionValue::Float(d) => OptionValue::Float(c.configf(o.tag, SC_GET, d)),
		OptionValue::Int(d) => OptionValue::Int(c.configi(o.tag, SC_GET, d as u32) as i32),
	})
}

/// check_str: whether `name` is one of the comma-separated entries of `list`
pub fn in_list(list: &str, name: &str) -> bool {
	// strstr finds the first occurrence only, and the C judges that one
	if let Some(from) = list.find(name) {
		let end = from + name.len();
		let after_ok = end == list.len() || list.as_bytes()[end] == b',';
		let before_ok = from == 0 || list.as_bytes()[from - 1] == b',';
		return after_ok && before_ok;
	}
	false
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn blacklists_match_whole_entries() {
		assert!(in_list("KN1,KN3", "KN3"));
		assert!(in_list("KN1", "KN1"));
		assert!(!in_list("-", "KN1"));
		assert!(!in_list("KN10", "KN1"));
		// the first occurrence decides, as with strstr
		assert!(!in_list("XKN1,KN1", "KN1"));
	}
}
