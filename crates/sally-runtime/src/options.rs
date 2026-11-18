//! The options the runtime registers, as survive-cli registers them: each with the C's tag, default and description,
//! one group per C file (in the file's order), and the groups in the order survive-cli's registry has them on each
//! platform, which is the order of a recording's OPTION lines and of the help. That order is the C's constructors' order
//! as each platform links and loads them: on Linux the library's files and then its plugins as the C build lists them;
//! on Windows the library's files in reverse, then survive-cli.c's own option, then the plugins in their load order.
//! The tracker's options come from sally-track (`sally_track::options`). An option moves into the piece that reads it
//! when that piece is ported.
#![forbid(unsafe_code)]

use sally_track::options::{OptionValue, OPTIONS as TRACKER_OPTIONS};

use crate::config::{Config, OptionDefault};

/// An option's default as written here: a string default is static text
#[derive(Clone, Copy)]
enum Def {
	Int(i32),
	Float(f64),
	Bool(bool),
	Str(Option<&'static [u8]>),
}

use Def::{Bool, Float, Int, Str};

impl Def {
	fn option_default(self) -> OptionDefault {
		match self {
			Int(v) => OptionDefault::Int(v),
			Float(v) => OptionDefault::Float(v),
			Bool(v) => OptionDefault::Bool(v),
			Str(v) => OptionDefault::Str(v.map(<[u8]>::to_vec)),
		}
	}
}

/// One option: its tag, its default, its description
type Opt = (&'static str, Def, &'static str);

/// survive.c
const SURVIVE: [Opt; 17] = [
	("v", Int(0), "Verbosity level"),
	("blacklist-devs", Str(Some(b"")), "List any devs (or substrings of devs) to blacklist."),
	("configfile", Str(Some(b"config.json")), "Default configuration file"),
	("init-configfile", Str(None), "Initial configuration file"),
	("disable-calibrate", Bool(false), "Enables or disables calibration"),
	("force-calibrate", Bool(false), "Forces calibration even if one exists."),
	("force-ootx", Bool(false), "Forces ootx capture even if its in the config file."),
	("lighthousecount", Int(0), "How many lighthouses to look for."),
	("lighthouse-gen", Int(0), "Which lighthouse gen to use -- 1 for LH1, 2 for LH2, 0 (default) for auto-detect"),
	("output-callback-stats", Float(0.0), "Print cb stats every given number of seconds. 0 disables this output."),
	("lighthouse-0-disable", Bool(false), "Disable lh at idx 0"),
	("lighthouse-1-disable", Bool(false), "Disable lh at idx 1"),
	("lighthouse-2-disable", Bool(false), "Disable lh at idx 2"),
	("lighthouse-3-disable", Bool(false), "Disable lh at idx 3"),
	("lighthouse-max-update", Float(0.01), "Maximum instant move for a lighthouse"),
	(
		"lighthouse-max-nudge-distance",
		Float(0.1),
		"If a proscribed LH move is more than this; do it instantly -- it's too far off to slerp over",
	),
	("lighthouse-update-velocity", Float(0.1), "Allowable velocity to update a lighthouse"),
];

/// survive_config.c
const SURVIVE_CONFIG: [Opt; 1] = [(
	"config-readonly",
	Bool(false),
	concat!(
		"sally: never write the config file back (also implied by lock-lighthouses: a wiped or re-solved map ",
		"must not replace the calibrated one on disk)",
	),
)];

/// survive_default_devices.c
const SURVIVE_DEFAULT_DEVICES: [Opt; 2] = [
	(
		"imu-acc-bias-units",
		Int(0),
		concat!(
			"sally: units of acc_bias in the device JSON: 0 = upstream guess (divide by 1000, i.e. ignored), 1 = ",
			"m/s^2 (divide by 9.80665 to get g)",
		),
	),
	("ignore-config-imu-bias", Bool(false), "Ignore the bias set for imu devices in the config"),
];

/// survive_recording.c
const SURVIVE_RECORDING: [Opt; 6] = [
	("record-rawlight", Bool(true), "Whether or not to output raw light data"),
	("record-imu", Bool(true), "Whether or not to output imu data"),
	("record-cal-imu", Bool(false), "Whether or not to output calibrated imu data"),
	("record-angle", Bool(true), "Whether or not to output angle data"),
	("record", Str(Some(b"")), "File to record to if you wish to make a recording."),
	("record-stdout", Bool(false), "Whether or not to dump recording data to stdout"),
];

/// survive_process.c
const SURVIVE_PROCESS: [Opt; 2] = [
	("report-in-imu", Bool(false), "Debug option to output poses in IMU space."),
	("serialize-device-config", Bool(false), "Serialize device config files"),
];

/// survive_process_gen2.c
const SURVIVE_PROCESS_GEN2: [Opt; 10] = [
	("serialize-ootx", Bool(false), "Serialize out ootx"),
	(
		"gen2-plane-predict",
		Float(-1.0),
		concat!(
			"sally: decide which of a gen2 station's two laser planes a pulse belongs to from where the device ",
			"is (the filter's pose, reprojected) whenever the device sits within this margin (rad) of the place ",
			"where the plain rule fails. The plain rule splits the rotor angle at pi, which holds only while ",
			"the plane-0 angle stays under +60 deg and the plane-1 angle over -60 deg. Further off-axis (the ",
			"station's field of view reaches about 80 deg) BOTH pulses of a rotation land on the same side and ",
			"get the same plane: near the sides of the play space every hit of a device is then labeled with ",
			"one plane, and with one axis no pose is solved while the light is there. A pulse further than ",
			"gen2-plane-predict-tol from both predictions is dropped. <= 0 = off",
		),
	),
	(
		"light-facing-deg",
		Float(-1.0),
		concat!(
			"sally: reflections. A gen2 hit on a sensor that faces away from the station that swept it cannot be ",
			"direct light: drop it when the angle between the sensor's normal and the direction to that station ",
			"(both from the filter's pose) is above this (deg). The pose is used only while the device had an ",
			"accepted solve within light-facing-max-age. Real hits reach 114 deg on two Index controller sensors ",
			"(their windows see past 90); reflected hits go up to 180. <= 0 = off",
		),
	),
	("light-facing-max-age", Float(0.5), "sally: see light-facing-deg (s since the device's last accepted solve)"),
	(
		"record-facing",
		Int(0),
		concat!(
			"sally: diagnostic: write a FAC line per gen2 hit (lh sensor plane angle, residual against the filter",
			" pose, incidence cosine)",
		),
	),
	("gen2-plane-predict-tol", Float(0.5), "sally: see gen2-plane-predict (rad)"),
	("sim-gap-period", Float(0.0), "sally bench: simulated full occlusion every this many seconds per device; 0 = off"),
	("sim-gap-len", Float(2.0), "sally bench: length (s) of each simulated occlusion"),
	("sim-gap-start", Float(20.0), "sally bench: no simulated occlusion before this run time (s)"),
	(
		"lock-lighthouses",
		Bool(false),
		"Never move/forget a known lighthouse position because of a new OOTX packet from the same station",
	),
];

/// survive_sensor_activations.c
const SURVIVE_SENSOR_ACTIVATIONS: [Opt; 7] = [
	("move-threshold-gyro", Float(0.075), "Threshold to count gyro norms as moving"),
	("move-threshold-acc", Float(0.03), "Threshold to count acc diff norms as moving"),
	("move-threshold-ang", Float(0.02), "Threshold to count light angle diffs as moving"),
	("filter-threshold-ang-per-sec", Float(50.0), "Threshold to filter light which changes too fast"),
	("filter-light-outlier-criteria", Float(0.5), "Threshold to filter outlier light strikes"),
	("filter-variance-minimum", Float(0.05), "Minimum variance to use in outlier detection"),
	("filter-outlier-minimum-count", Int(11), "Assumed minimum population for outlier test"),
];

/// survive-cli.c
const SURVIVE_CLI: [Opt; 2] = [
	("stop-after", Float(-1.0), "sally: stop a live run cleanly after this many seconds (-1 = never)"),
	DEVICE_LEARNED,
];

/// sally: the tracker's device calibration carried between sessions (learned.rs)
const DEVICE_LEARNED: Opt = (
	"device-learned-file",
	Str(Some(b"")),
	concat!(
		"sally: file of what the tracker learned of each device (gyro and accelerometer biases, sensor-layout ",
		"scale), by serial, session by session: each device's tracker starts from it, and it is written every ",
		"minute and at exit. Empty = every session starts from the factory prior",
	),
);

/// driver_diagnosis.c
const DRIVER_DIAGNOSIS: [Opt; 4] = [
	(
		"diagnosis",
		Int(1),
		"sally: diagnosis (what is wrong with each device, design 9.2): DIAG lines and the diagnosis hook. 0 = off",
	),
	("diagnosis-period", Float(0.5), "sally: diagnosis: window length (s of run time)"),
	(
		"diagnosis-back-deg",
		Float(115.0),
		concat!(
			"sally: diagnosis: when light-facing-deg is off, a hit on a sensor whose normal is further than this ",
			"from the direction to its station counts as a reflection (deg; real hits reach 114 on Index ",
			"controllers)",
		),
	),
	(
		"diagnosis-discard-vmax",
		Float(0.5),
		concat!(
			"sally: diagnosis: a station the solver dropped counts toward reflections only while the device is ",
			"slower than this (m/s)",
		),
	),
];

/// driver_vive.c
const DRIVER_VIVE: [Opt; 1] = [(
	"vive-test-drop",
	Str(Some(b"")),
	concat!(
		"sally: TEST ONLY, drop reports on a replay as a bad radio link would: \"T0,T1,CODENAME,FRACTION\" ",
		"drops that share of the device's reports (evenly spread; 1 = all) between run times T0 and T1. Empty",
		" = off",
	),
)];

/// driver_usbmon.c
const DRIVER_USBMON: [Opt; 8] = [
	("usbmon-record", Str(None), "File to save .pcap to."),
	(
		"usbmon-playback",
		Str(None),
		"File to replay .pcap from ('stdin' = standard input, e.g. piped from USBPcapCMD -o - for live use).",
	),
	(
		"usbmon-devs",
		Str(None),
		"sally: device listing file (vid pid bus dev codename per line); default <playback>.usbdevs",
	),
	("usbmon-record-all", Bool(false), "Whether or not to record all usb traffic"),
	("usbmon-output-all", Bool(false), "Whether or not to log all usb traffic"),
	("usbmon-output", Bool(false), "Whether or not to log any generic usb traffic"),
	("usbmon-only-record", Bool(false), "Record only; don't forward to libsurvive"),
	(
		"usbmon-allow-fs-config",
		Bool(false),
		"If we dont see a config section; try to read it from filesystem -- could be very wrong",
	),
];

/// driver_playback.c
const DRIVER_PLAYBACK: [Opt; 7] = [
	("playback-replay-pose", Bool(false), "Whether or not to output pose"),
	("playback-replay-external-pose", Bool(false), "Whether or not to output external pose"),
	("playback", Str(None), "File to be used for playback if playing a recording."),
	(
		"playback-factor",
		Float(1.0),
		"Time factor of playback -- 1 is run at the same timing as original, 0 is run as fast as possible.",
	),
	("playback-start-time", Float(-1.0), "Start time of playback"),
	("playback-time", Float(-1.0), "End time of playback"),
	("run-time", Float(-1.0), "How long to run for"),
];

/// disambiguator_statebased.c
const DISAMBIGUATOR_STATEBASED: [Opt; 2] = [
	("light-min-length", Int(100), "Minimum length of V1 light to accept."),
	("disambiguator-verbosity", Int(1000), "Verbosity of disambiguator"),
];

/// driver_map_solve.c
const DRIVER_MAP_SOLVE: [Opt; 40] = [
	(
		"map-solve",
		Int(0),
		concat!(
			"sally: live joint map solve: all devices solve the station poses together from their raw sweep ",
			"angles; the map moves (slowly, frame held) instead of staying locked. 0 = off",
		),
	),
	(
		"map-solve-file",
		Str(Some(b"")),
		concat!(
			"sally: file the live-solved map is written to at exit and read from at start (by station id). Empty ",
			"= every session starts from the calibrated map",
		),
	),
	(
		"map-solve-test-move",
		Str(Some(b"")),
		concat!(
			"sally: map-solve: TEST ONLY, simulates a bumped station on a replay: \"T,LH,dx,dy,dz,rx,ry,rz\" ",
			"moves station index LH in the map (not its light) by (dx dy dz) m and the rotation vector (rx ry rz)",
			" rad (world frame, about the station) at run time T s. Empty = off",
		),
	),
	(
		"map-solve-test-load",
		Str(Some(b"")),
		concat!(
			"sally: map-solve: TEST ONLY, start from this map file instead of map-solve-file (which is still ",
			"where the map is written). Empty = off",
		),
	),
	(
		"station-fcal-file",
		Str(Some(b"")),
		concat!(
			"sally: corrections to the base stations' factory calibration (the OOTX fcal terms phase, tilt, ",
			"curve, gibpha, gibmag), solved from recordings (sally calibrate). Lines 'FCAL <station id> <the 10 ",
			"factory values it was solved against> <the 10 deltas>', each pair as axis 0, axis 1. A correction ",
			"is applied only to a station whose factory values match, and again whenever an OOTX packet ",
			"rewrites them. Needs map-solve 1. Empty = off",
		),
	),
	(
		"device-calibration-file",
		Str(Some(b"")),
		concat!(
			"sally: corrections to the devices' factory sensor layout, solved offline from recordings: 'SENSOR ",
			"<serial> <sensor index> <dx> <dy> <dz>' moves one sensor (m, config frame), 'SCALE <serial> ",
			"<scale>' scales the whole layout. One base station measures distance from where the few sensors it ",
			"sees are, so a few tenths of a mm per sensor put a device seen by one station 1-2 cm too near or ",
			"too far, and it snaps when a second station comes. Empty = off",
		),
	),
	(
		"map-solve-batch",
		Int(400),
		"sally: map-solve: most snapshots one solve takes (the oldest are dropped beyond this)",
	),
	("map-solve-step", Float(0.25), "sally: map-solve: at most one snapshot per device every this many s"),
	(
		"map-solve-window",
		Float(0.022),
		"sally: map-solve: a snapshot's hits come from this many s before the pose (about one LH2 rotor turn)",
	),
	("map-solve-vmax", Float(0.05), "sally: map-solve: snapshots only while the filter speed is below this (m/s)"),
	("map-solve-wmax", Float(0.15), "sally: map-solve: ... and the turn rate below this (rad/s)"),
	(
		"map-solve-min-hits",
		Int(6),
		"sally: map-solve: hits a station must give a snapshot to count; a snapshot needs two such stations",
	),
	("map-solve-min-snaps", Int(40), "sally: map-solve: snapshots a batch needs before it is solved"),
	(
		"map-solve-min-station-hits",
		Int(150),
		"sally: map-solve: a station takes part in a solve with at least this many hits in the batch",
	),
	("map-solve-first", Float(10.0), "sally: map-solve: no solve before this many s of run time"),
	("map-solve-period", Float(5.0), "sally: map-solve: s between joint solves"),
	("map-solve-tau", Float(600.0), "sally: map-solve: what earlier solves learned fades with this time constant (s)"),
	("map-solve-iters", Int(8), "sally: map-solve: Gauss-Newton iterations per solve"),
	(
		"map-solve-prior-pos",
		Float(0.05),
		"sally: map-solve: sd (m) of the weak prior keeping each station near the calibrated map",
	),
	("map-solve-prior-rot", Float(0.01), "sally: map-solve: sd (rad) of the same prior on station orientation"),
	(
		"map-solve-rate-pos",
		Float(0.002),
		"sally: map-solve: stations in use move toward the solution at most this fast (m/s)",
	),
	("map-solve-rate-rot", Float(0.0005), "sally: map-solve: ... and turn at most this fast (rad/s)"),
	("map-solve-huber", Float(0.001), "sally: map-solve: robust (Huber) scale of a hit's angle residual (rad)"),
	("map-solve-reject", Float(0.008), "sally: map-solve: hits further off than this (rad) are left out of a solve"),
	(
		"map-solve-max-move",
		Float(0.05),
		"sally: map-solve: a solve that moves a station further than this (m) from the map in use is not applied",
	),
	("map-solve-max-rot", Float(0.02), "sally: map-solve: ... or turns it further than this (rad)"),
	(
		"map-solve-load-weight",
		Float(0.25),
		concat!(
			"sally: map-solve: share of the saved map's information kept at the next start (the map may have ",
			"shifted since)",
		),
	),
	(
		"map-solve-save-period",
		Float(60.0),
		concat!(
			"sally: map-solve: also write the map file every this many seconds after an applied solve (not only ",
			"at exit): a runtime that is killed never runs its exit save, and its next session would start from ",
			"the calibrated map again. <= 0 = only at exit",
		),
	),
	(
		"map-solve-relocate",
		Int(1),
		concat!(
			"sally: map-solve: handle a moved (bumped) station: a station whose light disagrees with the map ",
			"while the others agree is left out of the solve, solved on its own from the others' snapshot poses, ",
			"and once two batches agree on where it is now, it is put there at once (no slew), its old ",
			"information is forgotten and the calibrated anchor follows it. 0 = off (it stays where the map had ",
			"it)",
		),
	),
	(
		"map-solve-move-resid",
		Float(0.003),
		concat!(
			"sally: map-solve: a station counts as moved when the median angle residual of its hits, from ",
			"snapshot poses fitted to the other stations only, is above this (rad; normal sessions reach 1-2, a ",
			"bump of a few cm or 10+ mrad gives 5-20; below this the normal solve handles it better than a ",
			"relocation from one or two batches, which leaves millimeters along a pivot) ...",
		),
	),
	(
		"map-solve-move-fit",
		Float(0.0004),
		concat!(
			"sally: map-solve: a relocation counts only when it explains its station's light (median residual ",
			"from the other stations' snapshot poses) to this (rad); it is confirmed when it explains the next ",
			"batch to this and twice as well as the map, and dropped when the map explains it to twice this again",
		),
	),
	(
		"map-solve-move-ratio",
		Float(3.0),
		"sally: map-solve: ... and this many times the residual of the other stations in that fit ...",
	),
	(
		"map-solve-move-rest-ratio",
		Float(2.0),
		concat!(
			"sally: map-solve: ... and leaving any other station out instead leaves the rest at least this many ",
			"times worse (it is this station that disagrees; needs three stations with enough light)",
		),
	),
	(
		"map-solve-move-min-snaps",
		Int(12),
		concat!(
			"sally: map-solve: snapshots seen by a station and two others that the moved-station test and a ",
			"relocation need",
		),
	),
	(
		"map-solve-move-agree-pos",
		Float(0.01),
		"sally: map-solve: two relocations from consecutive batches must agree to within this (m) to be applied ...",
	),
	("map-solve-move-agree-rot", Float(0.003), "sally: map-solve: ... and this (rad)"),
	("map-solve-move-min", Float(0.005), "sally: map-solve: a relocation smaller than this (m) ..."),
	("map-solve-move-min-rot", Float(0.002), "sally: map-solve: ... and this (rad) is left to the normal solve"),
	(
		"map-solve-move-pending-max",
		Float(60.0),
		"sally: map-solve: a relocation waiting for its confirmation is dropped after this many s",
	),
	(
		"map-solve-threaded",
		Int(1),
		"sally: map-solve: solve in a background thread (live); 0 = inline, deterministic (replays)",
	),
];

/// driver_diagrec.c
const DRIVER_DIAGREC: [Opt; 1] =
	[("diagrec", Int(0), "sally: analysis diagnostics (RPT / DST lines) into the recording")];

/// driver_simulator.c
const DRIVER_SIMULATOR: [Opt; 17] = [
	("simulator", Bool(false), "Load a Simulator driver for testing."),
	("simulator-attractors", Int(3), "Number on gravity attractors in simulation"),
	("simulator-obj-sensors", Int(20), "Number on sensors on the simulated object"),
	("simulator-fcal-noise", Float(0.001), "Noise to apply to BSD fcal parameters"),
	("simulator-init-time", Float(2.0), "Init time -- object wont move for this long"),
	("simulator-gyro-noise", Float(0.001), "Variance of noise to apply to gyro"),
	("simulator-acc-noise", Float(0.0001), "Variance of noise to apply to accelerometer"),
	("simulator-gyro-bias", Float(0.1), "Scale of bias to apply to gyro"),
	("simulator-sensor-time-jitter", Float(0.01), "Variance of time jitter to apply to light sensors"),
	("simulator-show-gt", Int(1), "0: No GT device, 1: Show GT device, 2: Only GT device"),
	("simulator-sensor-noise", Float(0.0001), "Variance of noise to apply to light sensors"),
	("simulator-obj-radius", Float(0.05), "Radius of the simulated object"),
	("simulator-time", Float(0.0), "Seconds to run simulator for."),
	("simulator-sensor-droprate", Float(0.2), "Chance to drop a sensor reading"),
	("simulator-noise-scale", Float(1.0), ""),
	("simulator-lh-gen", Int(1), "Lighthouse generation"),
	("simulator-lh-duty-cycle", Float(1.0), "Duty cycle of lighthouses"),
];

/// driver_pose_udp.c
const DRIVER_POSE_UDP: [Opt; 8] = [
	("pose-udp", Int(0), "sally: UDP port to push the pose stream (P/V/L lines) to (0 = off)"),
	("pose-udp-host", Str(Some(b"127.0.0.1")), "sally: host to push the pose stream to"),
	("runtime-port", Int(0), "sally: UDP port the runtime binds for subscriptions and control commands (0 = off)"),
	(
		"runtime-bind",
		Str(Some(b"127.0.0.1")),
		concat!(
			"sally: address the runtime port binds to. Anyone who can reach it can change parameters and write ",
			"recordings, so keep it on loopback unless the network is yours",
		),
	),
	("runtime-state-hz", Float(100.0), "sally: maximum rate of S (tracker state) lines per device; 0 = every pose"),
	(
		"runtime-diag-period",
		Float(1.0),
		"sally: seconds of run time between H (hit counts), T (tracker stats) and M (map) summaries",
	),
	(
		"runtime-sub-timeout",
		Float(5.0),
		"sally: a subscriber that has not re-sent sub or ping for this many seconds is dropped",
	),
	(
		"runtime-driver-file",
		Str(None),
		concat!(
			"sally: file the driver stream is written to, every line (Q, L, P for a device without Q, each device's DEV ",
			"line before its first): what the SteamVR driver is sent, for the bench's output scores",
		),
	),
];

/// driver_hidtap.c
const DRIVER_HIDTAP: [Opt; 10] = [
	("hidtap", Bool(false), "sally: read the lighthouse HID devices directly, next to Valve's driver (Windows)"),
	(
		"hidtap-lightcap-mode",
		Int(-2),
		concat!(
			"sally: raw lightcap mode of the devices: -2 = detect from the packets (devices power on in raw0, ",
			"Valve switches them to raw1 when SteamVR starts), 0/1/2 = fixed, -1 = don't touch",
		),
	),
	(
		"hidtap-config-fetch",
		Bool(true),
		"sally: live mode: fetch a device's config over feature reports when no <serial>_config.json is cached",
	),
	(
		"hidtap-config-delay",
		Float(5.0),
		"sally: live mode: seconds a device must have been streaming before its config is fetched",
	),
	("hidtap-playback", Str(None), "sally: HID tap recording to replay"),
	("hidtap-record", Str(None), "sally: file to write the HID tap stream to"),
	(
		"hidtap-record-strip",
		Bool(true),
		concat!(
			"sally: live mode: the recording is written without the bytes of a radio dongle's report that follow its ",
			"packets (the dongle's own four and leftovers of earlier packets), which nothing reads: zeros in their ",
			"place. 0 keeps every byte",
		),
	),
	(
		"hidtap-past-end",
		Int(2),
		concat!(
			"sally: a radio packet that is not laid out as its flags say: 2 stops the parser's reading of it at the ",
			"packet's own end, and the session's end says how many there were; 1 only counts them; 0 reads on into the ",
			"dongle's bytes after the packet, as libsurvive does",
		),
	),
	(
		"hidtap-allow-fs-config",
		Bool(false),
		"sally: if no config stream is seen, try <serial>_config.json from the working directory",
	),
	("hidtap-vid", Int(10462), "sally: USB vendor id assumed for tap devices"),
];

/// The C files' order in survive-cli's registry on Linux
const LINUX_ORDER: [&str; 19] = [
	"survive.c",
	"survive_config.c",
	"survive_default_devices.c",
	"survive_core2_rs.c",
	"survive_recording.c",
	"survive_process.c",
	"survive_process_gen2.c",
	"survive_sensor_activations.c",
	"survive-cli.c",
	"driver_diagnosis.c",
	"driver_vive.c",
	"driver_usbmon.c",
	"driver_playback.c",
	"disambiguator_statebased.c",
	"driver_map_solve.c",
	"driver_diagrec.c",
	"driver_simulator.c",
	"driver_pose_udp.c",
	"driver_hidtap.c",
];

/// The C files' order in survive-cli's registry on Windows
const WINDOWS_ORDER: [&str; 19] = [
	"survive_sensor_activations.c",
	"survive_process_gen2.c",
	"survive_process.c",
	"survive_recording.c",
	"survive_core2_rs.c",
	"survive_default_devices.c",
	"survive_config.c",
	"survive.c",
	"survive-cli.c",
	"disambiguator_statebased.c",
	"driver_diagnosis.c",
	"driver_diagrec.c",
	"driver_map_solve.c",
	"driver_playback.c",
	"driver_pose_udp.c",
	"driver_simulator.c",
	"driver_vive.c",
	"driver_hidtap.c",
	"driver_usbmon.c",
];

/// A C file's options
fn group(file: &str) -> &'static [Opt] {
	match file {
		"survive.c" => &SURVIVE,
		"survive_config.c" => &SURVIVE_CONFIG,
		"survive_default_devices.c" => &SURVIVE_DEFAULT_DEVICES,
		"survive_recording.c" => &SURVIVE_RECORDING,
		"survive_process.c" => &SURVIVE_PROCESS,
		"survive_process_gen2.c" => &SURVIVE_PROCESS_GEN2,
		"survive_sensor_activations.c" => &SURVIVE_SENSOR_ACTIVATIONS,
		"survive-cli.c" => &SURVIVE_CLI,
		"driver_diagnosis.c" => &DRIVER_DIAGNOSIS,
		"driver_vive.c" => &DRIVER_VIVE,
		"driver_usbmon.c" => &DRIVER_USBMON,
		"driver_playback.c" => &DRIVER_PLAYBACK,
		"disambiguator_statebased.c" => &DISAMBIGUATOR_STATEBASED,
		"driver_map_solve.c" => &DRIVER_MAP_SOLVE,
		"driver_diagrec.c" => &DRIVER_DIAGREC,
		"driver_simulator.c" => &DRIVER_SIMULATOR,
		"driver_pose_udp.c" => &DRIVER_POSE_UDP,
		"driver_hidtap.c" => &DRIVER_HIDTAP,
		_ => &[],
	}
}

/// Registers every option: the groups in `order` (each named for the C file its options came from), the tracker's own
/// from sally-track
fn register_in(config: &mut Config, order: &[&str]) {
	for file in order {
		if *file == "survive_core2_rs.c" {
			for o in &TRACKER_OPTIONS {
				let default = match o.default {
					OptionValue::Float(v) => OptionDefault::Float(v),
					OptionValue::Int(v) => OptionDefault::Int(v),
				};
				config.register(o.tag, o.description, default);
			}
			continue;
		}
		for (name, default, description) in group(file) {
			config.register(name, description, default.option_default());
		}
	}
}

/// Registers every option, in survive-cli's order on this platform
pub fn register(config: &mut Config) {
	register_in(config, if cfg!(windows) { &WINDOWS_ORDER } else { &LINUX_ORDER });
}

#[cfg(test)]
mod tests {
	use super::*;

	fn quote(s: &[u8]) -> String {
		let mut out = String::from("\"");
		for &c in s {
			if (32..127).contains(&c) && c != b'"' && c != b'\\' {
				out.push(c as char);
			} else {
				out.push_str(&format!("\\x{c:02x}"));
			}
		}
		out.push('"');
		out
	}

	/// On Linux the registry is the C build's, option for option (tests/data/config/registry.txt, dumped from its
	/// library), with survive-cli's own option where survive-cli's recordings list it
	#[test]
	fn the_linux_registry_is_the_c_builds() {
		let mut config = Config::default();
		register_in(&mut config, &LINUX_ORDER);
		let ours: Vec<String> = config
			.options()
			.iter()
			.map(|o| {
				let default = match o.default_value() {
					OptionDefault::Int(v) => v.to_string(),
					OptionDefault::Float(v) => format!("{:016x}", v.to_bits()),
					OptionDefault::Bool(v) => (v as u8).to_string(),
					OptionDefault::Str(None) => "NULL".to_string(),
					OptionDefault::Str(Some(s)) => quote(&s),
				};
				format!("option {} {} {} {}", o.type_char() as char, quote(o.name()), default, quote(o.description()))
			})
			.collect();
		let mut c: Vec<&str> = include_str!("../tests/data/config/registry.txt").lines().collect();
		let at = c.iter().position(|l| l.contains(r#""filter-outlier-minimum-count""#)).unwrap() + 1;
		let stop_after = r#"option f "stop-after" bff0000000000000 "sally: stop a live run cleanly after this many seconds (-1 = never)""#;
		c.insert(at, stop_after);
		let learned = format!(
			"option s {} {} {}",
			quote(DEVICE_LEARNED.0.as_bytes()),
			quote(b""),
			quote(DEVICE_LEARNED.2.as_bytes())
		);
		c.insert(at + 1, &learned);
		// the runtime protocol's file of the driver stream (sally's, after the C's last option of that file)
		let at = c.iter().position(|l| l.contains(r#""runtime-sub-timeout""#)).unwrap() + 1;
		let (name, _, says) = DRIVER_POSE_UDP[7];
		let driver_file = format!("option s {} NULL {}", quote(name.as_bytes()), quote(says.as_bytes()));
		c.insert(at, &driver_file);
		// the recording without what nothing reads (sally's, after the tap's record file)
		let at = c.iter().position(|l| l.contains(r#""hidtap-record""#)).unwrap() + 1;
		let (name, _, says) = DRIVER_HIDTAP[6];
		let record_strip = format!("option b {} 1 {}", quote(name.as_bytes()), quote(says.as_bytes()));
		assert_eq!(name, "hidtap-record-strip");
		c.insert(at, &record_strip);
		let (name, _, says) = DRIVER_HIDTAP[7];
		let past_end = format!("option i {} 2 {}", quote(name.as_bytes()), quote(says.as_bytes()));
		c.insert(at + 1, &past_end);
		assert_eq!(ours, c);
	}

	/// On Windows the options come in the order of a Windows survive-cli's recording's OPTION lines
	/// (tests/data/options/windows_order.txt, from a recording made by the Windows build)
	#[test]
	fn the_windows_order_is_the_windows_builds() {
		let mut config = Config::default();
		register_in(&mut config, &WINDOWS_ORDER);
		let ours: Vec<String> =
			config.options().iter().map(|o| String::from_utf8_lossy(o.name()).into_owned()).collect();
		let mut c: Vec<&str> = include_str!("../tests/data/options/windows_order.txt").split_whitespace().collect();
		// sally's options since that build
		let at = c.iter().position(|&o| o == "stop-after").unwrap() + 1;
		c.insert(at, "device-learned-file");
		let at = c.iter().position(|&o| o == "runtime-sub-timeout").unwrap() + 1;
		c.insert(at, "runtime-driver-file");
		let at = c.iter().position(|&o| o == "hidtap-record").unwrap() + 1;
		c.insert(at, "hidtap-record-strip");
		c.insert(at + 1, "hidtap-past-end");
		assert_eq!(ours, c);
	}
}
