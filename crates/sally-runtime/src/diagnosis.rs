//! sally's diagnosis (docs/sally_design.md 9.2): what is wrong with each device right now, and why.
//!
//! One module turns the runtime's own data into a short list of causes per device, each with a severity (0 fine, 1
//! hurting a little, 2 hurting), so the live status view, the replay viewer, the exported bundle and the bench all
//! read the same verdicts. It runs the same live and on a replay. Every diagnosis-period seconds of run time (0.5) a
//! window closes for every known device and its verdict goes into the recording as a DIAG line; at close, a line per
//! device in the log says how late its light came against its IMU stream over the session. Causes, first version:
//!
//! - light: "can't see the base stations": the filter has had no light for a while (dark / coasting), or only one
//!   station lit the device all window (one station gives weak depth). Not raised when the radio explains it.
//! - radio: reports lost between the device and its dongle: gaps in the device's IMU timecodes against its own usual
//!   sample interval (the same live, on tap recordings and on old USB captures, since every report carries IMU).
//!   Counted on the device's clock, not as samples per second of run time: a rate learned from the run time takes the
//!   burst after the runtime's start (the reports the host held during the start's ~0.5 s stall) for the usual rate,
//!   and shows every later window as 15-30 % lost with nothing lost. Also a signal level 0-4, like wifi bars. Or its
//!   light held back: how far the device's IMU stream runs ahead of its light when the light arrives, the median of the
//!   windows' medians over the last 60 s at 19 ms or more (off again under 15). Healthy devices sit at 0-3 ms or in a
//!   held mode at 14-18, which a shorter median or a lower mark keeps flagging; a device whose radio holds its light
//!   back sits at 18-25 ms.
//! - reflect: light from impossible directions: hits on sensors that face away from the station that lit them (the
//!   light-facing gate's own counts, or our test at diagnosis-back-deg when that gate is off), hits the filter's
//!   per-hit gate threw out, and stations the solver dropped as inconsistent while the device was slow. Mild
//!   (severity 1) at most: present and filtered is the common case.
//! - station: a base station the device was using went dark for every device, or one station disagrees with the
//!   others for several devices (moved or unstable; map-solve relocates a moved one).
//! - device: battery low, or the device has no sensor layout (missing config).
//!
//! The headline is the worst cause; on a tie the root cause wins: radio, station, device, then light, reflect.
//!
//! DIAG line (recording): codename DIAG run_t headline sev bars loss_pct gap_ms imu_n stations sensors back_pct gated
//! discards state light_age charge vmax, then the causes as name:sev[:detail] tokens ("fine" when none).
//!
//! Each verdict goes out through the diagnosis hook after its DIAG line: the runtime protocol's D and C lines.
//!
//! Not done here:
//! - the stations a solver dropped as inconsistent (for the reflections): nothing reports solves, so no station is ever
//!   dropped and the DIAG line's discards are 0.
//! - a device's state is not dropped when the device goes: devices go only as the runtime closes.
//! - no lock: the runtime is single-threaded. The hooks' chain is the runtime's (driver.rs).

// R is the pose's rotation as a matrix
#![allow(non_snake_case)]

use sally_lighthouse::NUM_GEN2_LIGHTHOUSES;
use sally_math::printf::{sprintf, Arg};
use sally_math::{c_atoi, c_fmax, norm3d, quatiszero, Pose, Quat, LINMATHPI};
use sally_track::host::{
	SURVIVE_DIAG_STATE_COASTING, SURVIVE_DIAG_STATE_DARK, SURVIVE_DIAG_STATE_REACQUIRING, SURVIVE_DIAG_STATE_TRACKING,
};

use crate::config::{Binding, VarType, SC_GET};
use crate::driver::Driver;
use crate::runtime::Runtime;
use crate::ObjectId;

/// SURVIVE_DIAG_* (survive_types.h): the causes, root first (on equal severity the lower number is the headline)
pub mod cause {
	pub const FINE: i32 = 0;
	/// reports lost between the device and its dongle
	pub const RADIO: i32 = 1;
	/// a base station went dark for everyone, or disagrees with the others
	pub const STATION: i32 = 2;
	/// battery, missing config
	pub const DEVICE: i32 = 3;
	/// can't see the base stations (occlusion), or only one
	pub const LIGHT: i32 = 4;
	/// light from impossible directions
	pub const REFLECT: i32 = 5;
	/// SURVIVE_DIAG_CAUSES
	pub const COUNT: i32 = 6;
}

/// SURVIVE_DIAG_MAX_CAUSES
pub const MAX_CAUSES: usize = 8;
/// SurviveDiagnosisCause's char detail[96] and text[160]
const DETAIL_SIZE: usize = 96;
const TEXT_SIZE: usize = 160;

const DG_MAXDEV: usize = 32;
const DG_MAXLH: usize = NUM_GEN2_LIGHTHOUSES;
// the light's lateness (radio): 1 ms bins (the last one holds 63 ms and more), a window needs this many hits to be
// judged, its median joins the last DG_LATEWIN windows' (60 s at 0.5 s), and the verdict is on at DG_LATE_ON, off
// under DG_LATE_OFF, hurting (2) at DG_LATE_BAD
const DG_LATEBINS: usize = 64;
const DG_LATE_MINHITS: i32 = 50;
const DG_LATEWIN: usize = 120;
const DG_LATE_ON: f64 = 0.019;
const DG_LATE_OFF: f64 = 0.015;
const DG_LATE_BAD: f64 = 0.040;

/// SurviveDiagnosisCause: one cause of a device's trouble
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Cause {
	/// `cause::*`
	pub cause: i32,
	/// 1 = hurting a little, 2 = hurting
	pub severity: i32,
	/// e.g. "base station ch 9 went dark for every device" (95 bytes at most)
	pub detail: String,
	/// the plain-language line: cause and detail (159 bytes at most)
	pub text: String,
}

/// SurviveDiagnosis: what is wrong with one device in one window, and why
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DeviceDiagnosis {
	pub run_time: f64,
	pub period: f64,
	/// the worst cause (`cause::FINE` / 0 when none)
	pub headline: i32,
	pub severity: i32,
	/// radio signal 0-4
	pub bars: i32,
	/// share of reports lost, longest gap (s)
	pub loss: f64,
	pub gap_max: f64,
	/// IMU samples; stations and sensors that lit it
	pub imu_n: i32,
	pub stations: i32,
	pub sensors: i32,
	/// share of judged hits that came from behind their sensor
	pub back: f64,
	/// hits the filter's gate threw out; stations the solver dropped
	pub gated: i32,
	pub discards: i32,
	/// SURVIVE_DIAG_STATE_*
	pub state: i32,
	pub light_age: f64,
	pub charge: i32,
	/// fastest the filter had it in the window (m/s)
	pub vmax: f64,
	/// causes[..n_causes], MAX_CAUSES at most
	pub causes: Vec<Cause>,
}

/// dg_dev: one device's state
#[derive(Clone, Debug)]
struct Device {
	so: ObjectId,
	// this window
	hits: [i32; DG_MAXLH],
	/// sensors lit, per station
	lit: [u64; DG_MAXLH],
	/// hits on sensors facing away from their station / hits judged (pose known)
	back: i32,
	judged: i32,
	/// the light-facing gate's counters at window start (it drops those hits before we see them)
	fchk0: u32,
	fdrop0: u32,
	/// fastest the filter had the device in this window (m/s)
	vmax: f64,
	/// run time of the device's first report
	first_rt: f64,
	imu_n: i32,
	/// IMU samples the device sent this window, by its own clock: imu_n plus the ones a gap skipped
	imu_expect: i32,
	/// longest IMU gap (s)
	imu_gap_max: f64,
	// across windows
	imu_last_raw: u32,
	/// usual IMU sample interval (s), learned
	imu_dt: f64,
	/// the last four windows' loss, for steady signal bars
	loss_hist: [f64; 4],
	imu_seen: i32,
	/// run time of the last IMU sample
	last_report_rt: f64,
	have_pose: bool,
	/// imu2world of the latest pose
	R: [f64; 9],
	p: [f64; 3],
	/// run time this device last got light from each station
	seen_lh: [f64; DG_MAXLH],
	any_report: bool,
	// the light's lateness against the device's own IMU stream: this window's histogram, the judged windows' medians
	// with their run times, the verdict, and the session's counts for the summary at close
	late_hist: [i32; DG_LATEBINS],
	late_n: i32,
	late_med: [f64; DG_LATEWIN],
	late_rt: [f64; DG_LATEWIN],
	late_i: usize,
	late_on: bool,
	late_windows: i32,
	late_flagged: i32,
	late_sess: [i32; DG_LATEBINS],
}

impl Device {
	/// get_dev's new device: all zero (SV_CALLOC), no report yet, no station seen
	fn new(so: ObjectId) -> Self {
		Device {
			so,
			hits: [0; DG_MAXLH],
			lit: [0; DG_MAXLH],
			back: 0,
			judged: 0,
			fchk0: 0,
			fdrop0: 0,
			vmax: 0.0,
			first_rt: -1.0,
			imu_n: 0,
			imu_expect: 0,
			imu_gap_max: 0.0,
			imu_last_raw: 0,
			imu_dt: 0.0,
			loss_hist: [0.0; 4],
			imu_seen: 0,
			last_report_rt: -1.0,
			have_pose: false,
			R: [0.0; 9],
			p: [0.0; 3],
			seen_lh: [-1e9; DG_MAXLH],
			any_report: false,
			late_hist: [0; DG_LATEBINS],
			late_n: 0,
			late_med: [0.0; DG_LATEWIN],
			late_rt: [0.0; DG_LATEWIN],
			late_i: 0,
			late_on: false,
			late_windows: 0,
			late_flagged: 0,
			late_sess: [0; DG_LATEBINS],
		}
	}
}

/// DriverRegDiagnosis's driver (the C's `diagnosis`)
pub struct Diagnosis {
	/// diagnosis-period, diagnosis-back-deg and diagnosis-discard-vmax, bound to their options (STRUCT_CONFIG_SECTION),
	/// so that a set while running reaches them as in the C. discard_vmax is read only by the solve path (see the top)
	period: Binding,
	back_deg: Binding,
	discard_vmax: Binding,
	cos_back: f64,
	last_close: f64,
	/// devs[..dev_ct]: the devices in the order they were first seen (DG_MAXDEV at most)
	devs: Vec<Device>,
	// stations, ctx-wide
	lh_hits: [i32; DG_MAXLH],
	lh_last_seen: [f64; DG_MAXLH],
}

/// cause_text: a cause in plain language
fn cause_text(cause: i32) -> &'static str {
	const T: [&str; 6] = [
		"fine",
		"radio: weak link to the device",
		"base station problem",
		"device problem",
		"can't see the base stations",
		"reflections: light arriving from impossible directions",
	];
	if cause >= 0 && cause < cause::COUNT {
		T[cause as usize]
	} else {
		"?"
	}
}

/// cause_name: a cause as the DIAG line names it
fn cause_name(cause: i32) -> &'static str {
	const T: [&str; 6] = ["fine", "radio", "station", "device", "light", "reflect"];
	if cause >= 0 && cause < cause::COUNT {
		T[cause as usize]
	} else {
		"?"
	}
}

/// state_name: the tracker's state (SURVIVE_DIAG_STATE_*) as the DIAG line names it
fn state_name(s: i32) -> &'static str {
	const T: [&str; 5] = ["none", "tracking", "dark", "coasting", "reacquiring"];
	if s >= 0 && s < 5 {
		T[s as usize]
	} else {
		"?"
	}
}

/// mv3: y = A x, A a 3x3 matrix by rows
fn mv3(A: &[f64; 9], x: &[f64; 3]) -> [f64; 3] {
	let mut t = [0.0; 3];
	for i in 0..3 {
		t[i] = A[i * 3] * x[0] + A[i * 3 + 1] * x[1] + A[i * 3 + 2] * x[2];
	}
	t
}

/// q2R: a quaternion (w x y z, normalized first when it is not zero) as a rotation matrix by rows
fn q2R(q: &Quat) -> [f64; 9] {
	let (mut w, mut x, mut y, mut z) = (q[0], q[1], q[2], q[3]);
	let mut n = w * w + x * x + y * y + z * z;
	if n > 0.0 {
		n = 1.0 / n.sqrt();
		w *= n;
		x *= n;
		y *= n;
		z *= n;
	}
	[
		1.0 - 2.0 * (y * y + z * z),
		2.0 * (x * y - z * w),
		2.0 * (x * z + y * w),
		2.0 * (x * y + z * w),
		1.0 - 2.0 * (x * x + z * z),
		2.0 * (y * z - x * w),
		2.0 * (x * z - y * w),
		2.0 * (y * z + x * w),
		1.0 - 2.0 * (x * x + y * y),
	]
}

/// snprintf into a char[size]: the text cut to size - 1 bytes (the texts are ASCII; a cut inside a character of
/// another text would back up to its start)
fn cut(mut s: String, size: usize) -> String {
	let max = size.saturating_sub(1);
	if s.len() > max {
		let mut end = max;
		while !s.is_char_boundary(end) {
			end -= 1;
		}
		s.truncate(end);
	}
	s
}

fn snprintf(size: usize, fmt: &str, args: &[Arg]) -> String {
	cut(sprintf(fmt, args), size)
}

/// add_cause: a cause with its detail (`fmt` formatted into the C's 96 bytes) and its plain-language text (cause and
/// detail, into 160 bytes); nothing at severity 0, or once the list is full
fn add_cause(g: &mut DeviceDiagnosis, cause: i32, sev: i32, fmt: &str, args: &[Arg]) {
	if sev <= 0 || g.causes.len() >= MAX_CAUSES {
		return;
	}
	let detail = snprintf(DETAIL_SIZE, fmt, args);
	let sep = if detail.is_empty() { "" } else { ": " };
	let text = snprintf(TEXT_SIZE, "%s%s%s", &[Arg::S(cause_text(cause)), Arg::S(sep), Arg::S(&detail)]);
	g.causes.push(Cause { cause, severity: sev, detail, text });
}

/// The DIAG line as the C builds it in char line[1200]: each piece snprintf'd at offset n, where n adds up what each
/// call returned (the length it would have written), so a line cut once stays cut. Past the buffer's end the C's size
/// (sizeof line - n) would wrap and write out of bounds (undefined; no line comes near it): nothing is written there
#[derive(Default)]
struct Line {
	text: String,
	n: usize,
}

impl Line {
	const SIZE: usize = 1200;

	fn push(&mut self, s: &str) {
		if self.n < Self::SIZE {
			self.text.push_str(&cut(s.to_string(), Self::SIZE - self.n));
		}
		self.n += s.len();
	}
}

/// The DIAG line of a device's verdict, its newline included (late_sev and late10: the radio cause's lateness token)
fn diag_line(codename: &str, now: f64, g: &DeviceDiagnosis, late_sev: i32, late10: f64) -> String {
	let mut line = Line::default();
	line.push(&sprintf(
		"%s DIAG %.3f %s %d %d %.1f %.0f %d %d %d %.1f %d %d %s %.2f %d %.2f",
		&[
			Arg::S(codename),
			Arg::F(now),
			Arg::S(cause_name(g.headline)),
			Arg::I(g.severity as i64),
			Arg::I(g.bars as i64),
			Arg::F(100.0 * g.loss),
			Arg::F(1e3 * g.gap_max),
			Arg::I(g.imu_n as i64),
			Arg::I(g.stations as i64),
			Arg::I(g.sensors as i64),
			Arg::F(100.0 * g.back),
			Arg::I(g.gated as i64),
			Arg::I(g.discards as i64),
			Arg::S(state_name(g.state)),
			Arg::F(g.light_age),
			Arg::I(g.charge as i64),
			Arg::F(g.vmax),
		],
	));
	if g.causes.is_empty() {
		line.push(" fine");
	}
	let mut c = 0;
	while c < g.causes.len() && line.n < Line::SIZE - 80 {
		let k = &g.causes[c];
		line.push(&sprintf(" %s:%d", &[Arg::S(cause_name(k.cause)), Arg::I(k.severity as i64)]));
		if k.cause == cause::RADIO && late_sev != 0 {
			line.push(&sprintf(":late%.0f", &[Arg::F(late10 * 1e3)]));
		}
		if k.cause == cause::REFLECT || k.cause == cause::STATION {
			// the station, when the cause names one
			if let Some(ch) = k.detail.find("ch ") {
				line.push(&sprintf(":ch%d", &[Arg::I(c_atoi(&k.detail.as_bytes()[ch + 3..]) as i64)]));
			}
		}
		c += 1;
	}
	line.push("\n");
	line.text
}

/// late_update: the light's lateness. This window's median joins the last DG_LATEWIN windows' when it had light
/// enough to judge, and the median of those (judged within the last DG_LATEWIN periods) is the verdict's measure; -1
/// when fewer than half of them were judged. Also whether this window was judged
fn late_update(v: &mut Device, now: f64, period: f64) -> (f64, bool) {
	let judged = v.late_n >= DG_LATE_MINHITS;
	if judged {
		let half = (v.late_n + 1) / 2;
		let mut acc = 0;
		let mut b = 0;
		while b < DG_LATEBINS - 1 {
			acc += v.late_hist[b];
			if acc >= half {
				break;
			}
			b += 1;
		}
		v.late_med[v.late_i] = (b as f64 + 0.5) * 1e-3;
		v.late_rt[v.late_i] = now;
		v.late_i = (v.late_i + 1) % DG_LATEWIN;
		v.late_windows += 1;
		v.late_sess[b] += 1;
	}
	let mut s = [0.0; DG_LATEWIN];
	let mut k = 0;
	for i in 0..DG_LATEWIN {
		if v.late_rt[i] > 0.0 && now - v.late_rt[i] < (DG_LATEWIN as f64 + 0.5) * period {
			s[k] = v.late_med[i];
			k += 1;
		}
	}
	if k < DG_LATEWIN / 2 {
		return (-1.0, judged);
	}
	for a in 1..k {
		let mut c = a;
		while c > 0 && s[c - 1] > s[c] {
			s.swap(c - 1, c);
			c -= 1;
		}
	}
	(s[k / 2], judged)
}

impl Diagnosis {
	/// The driver with its options bound, or None when "diagnosis" is 0 (the C then adds no driver but returns
	/// SURVIVE_DRIVER_PASSIVE all the same, so survive_startup names it among the loaded drivers)
	pub fn reg(rt: &mut Runtime) -> Option<Diagnosis> {
		if rt.config.configi("diagnosis", SC_GET, 1) == 0 {
			return None;
		}
		let mut d = Diagnosis {
			period: Binding::of(VarType::Float),
			back_deg: Binding::of(VarType::Float),
			discard_vmax: Binding::of(VarType::Float),
			cos_back: 0.0,
			last_close: 0.0,
			devs: Vec::new(),
			lh_hits: [0; DG_MAXLH],
			lh_last_seen: [0.0; DG_MAXLH],
		};
		// diagnosis_attach_config: each variable takes its option's value as it is bound (the defaults the C writes
		// into them first are the options' own, and are overwritten)
		for (tag, var) in d.config_items() {
			rt.config.attach(tag, var);
		}
		if d.period.float() < 0.05 {
			d.period.set_float(0.05);
		}
		d.cos_back = (d.back_deg.float() * LINMATHPI / 180.0).cos();
		Some(d)
	}

	/// The section's items: diagnosis_attach_config binds them, diagnosis_detach_config unbinds them
	fn config_items(&self) -> [(&'static str, &Binding); 3] {
		[
			("diagnosis-period", &self.period),
			("diagnosis-back-deg", &self.back_deg),
			("diagnosis-discard-vmax", &self.discard_vmax),
		]
	}

	/// get_dev: the device's state, made when it is new; None once DG_MAXDEV devices are known
	fn get_dev(&mut self, so: ObjectId) -> Option<usize> {
		if let Some(i) = self.devs.iter().position(|v| v.so == so) {
			return Some(i);
		}
		if self.devs.len() >= DG_MAXDEV {
			return None;
		}
		self.devs.push(Device::new(so));
		Some(self.devs.len() - 1)
	}

	/// tick: the window closes once a period of run time has passed since the last close
	fn tick(&mut self, rt: &mut Runtime) {
		let now = rt.run_time();
		if self.last_close <= 0.0 {
			self.last_close = now;
		}
		if now - self.last_close >= self.period.float() {
			self.close_window(rt, now);
		}
	}

	/// close_window: the window closed for every device: its verdict into the recording (DIAG) and onto the device
	fn close_window(&mut self, rt: &mut Runtime, now: f64) {
		let period = now - self.last_close;
		self.last_close = now;
		if period <= 0.0 {
			return;
		}
		// back_deg can be set live ("set diagnosis-back-deg"): keep the cosine with it
		self.cos_back = (self.back_deg.float() * LINMATHPI / 180.0).cos();
		let d_period = self.period.float();

		// stations: dark for every device for a second after being seen within the last 10 s; disagreeing with the map
		// (map-solve's moved-station verdict) or relocated in the last 30 s
		let mut lh_gone = [false; DG_MAXLH];
		let mut lh_bad = [false; DG_MAXLH];
		let mut lh_moved = [false; DG_MAXLH];
		for l in 0..DG_MAXLH {
			if l as i32 >= rt.active_lighthouses {
				break;
			}
			if self.lh_hits[l] > 0 {
				self.lh_last_seen[l] = now;
			} else if self.lh_last_seen[l] > 0.0
				&& now - self.lh_last_seen[l] >= 1.0
				&& now - self.lh_last_seen[l] < 10.0
			{
				lh_gone[l] = true;
			}
			lh_bad[l] = rt.bsd[l].sally_moved == 1;
			lh_moved[l] = rt.bsd[l].sally_moved == 2 && now - rt.bsd[l].sally_moved_time < 30.0;
		}

		for i in 0..self.devs.len() {
			let v = &mut self.devs[i];
			let obj = v.so;
			if !v.any_report {
				continue;
			}
			let mut g = DeviceDiagnosis { run_time: now, period, ..Default::default() };
			let st = rt.tracker_diag(obj);
			g.state = st.state;
			g.light_age = st.light_age;
			let so = &rt.objects[obj];
			g.charge = so.charge as i32;
			let (codename, serial) = (so.codename.clone(), so.serial_number.clone());

			// radio: the samples received against the ones the device sent by its own clock (the gaps in its IMU
			// timecodes), the longest such gap, and the open gap since its last sample. A window without any sample from
			// a device that was reporting counts as all lost (its gap is counted again when the samples resume, so a long
			// dropout shows a little longer than it lasted: the bars average the last 2 s anyway)
			let mut gap = v.imu_gap_max;
			if v.last_report_rt >= 0.0 && now - v.last_report_rt > gap {
				gap = now - v.last_report_rt;
			}
			g.imu_n = v.imu_n;
			let mut wloss =
				if v.imu_expect > 0 { c_fmax(0.0, 1.0 - v.imu_n as f64 / v.imu_expect as f64) } else { 0.0 };
			if v.imu_n == 0 && v.last_report_rt >= 0.0 && now - v.last_report_rt >= period {
				wloss = 1.0;
			}
			if wloss < 0.02 {
				wloss = 0.0; // a sample or two either side of the window's edge
			}
			// a share of reports lost reads noisily over half a second: the bars and the mild level use the last 2 s
			v.loss_hist.copy_within(0..3, 1);
			v.loss_hist[0] = wloss;
			g.loss = 0.25 * (v.loss_hist[0] + v.loss_hist[1] + v.loss_hist[2] + v.loss_hist[3]);
			g.gap_max = gap;
			g.bars = if g.loss < 0.01 {
				4
			} else if g.loss < 0.05 {
				3
			} else if g.loss < 0.15 {
				2
			} else if g.loss < 0.4 {
				1
			} else {
				0
			};
			let silent = if v.last_report_rt >= 0.0 { now - v.last_report_rt } else { 0.0 };
			// not a weak link any more: switched off, flat, or out of range
			let off = silent > 3.0;
			// ... and the light held back (see the top): logged when the verdict changes, a cause while it holds
			let (late10, judged) = late_update(v, now, d_period);
			let was_late = v.late_on;
			if late10 >= DG_LATE_ON {
				v.late_on = true;
			} else if late10 >= 0.0 && late10 < DG_LATE_OFF {
				v.late_on = false;
			}
			if judged && v.late_on {
				v.late_flagged += 1;
			}
			if v.late_on && !was_late {
				rt.info(&sprintf(
					concat!(
						"%s (%s) diagnosis: its light comes %.0f ms late (the median over 60 s; usually 0-3, or 14-18 ",
						"when held): its radio is holding the light back",
					),
					&[Arg::S(&codename), Arg::S(&serial), Arg::F(late10 * 1e3)],
				));
			} else if !v.late_on && was_late {
				rt.info(&sprintf(
					"%s (%s) diagnosis: its light is on time again (%.0f ms)",
					&[Arg::S(&codename), Arg::S(&serial), Arg::F(late10 * 1e3)],
				));
			}
			let loss_sev = if off {
				0
			} else if g.loss >= 0.2 || wloss >= 0.5 || gap >= 0.1 {
				2
			} else if g.loss >= 0.05 {
				1
			} else {
				0
			};
			let late_sev = if off || !v.late_on || late10 < 0.0 {
				0
			} else if late10 >= DG_LATE_BAD {
				2
			} else {
				1
			};
			let radio = if loss_sev > late_sev { loss_sev } else { late_sev };
			let mut lost = String::new();
			let mut held = String::new();
			if loss_sev != 0 {
				let gaps = if gap >= 0.1 { ", gaps up to a tenth of a second or more" } else { "" };
				lost = snprintf(96, "%.0f%% of reports lost%s", &[Arg::F(100.0 * g.loss), Arg::S(gaps)]);
			}
			if late_sev != 0 {
				held = snprintf(64, "its light comes %.0f ms late", &[Arg::F(5.0 * (late10 * 1e3 / 5.0).round())]);
			}
			let both = if loss_sev != 0 && late_sev != 0 { ", " } else { "" };
			add_cause(&mut g, cause::RADIO, radio, "%s%s%s", &[Arg::S(&lost), Arg::S(both), Arg::S(&held)]);
			if off {
				add_cause(
					&mut g,
					cause::DEVICE,
					2,
					"no reports for %.0f s (switched off, flat, or out of range)",
					&[Arg::F(silent)],
				);
			}

			// light
			let mut nst = 0;
			let mut nsens = 0;
			for l in 0..DG_MAXLH {
				if v.hits[l] >= 4 {
					nst += 1;
				}
				nsens += v.lit[l].count_ones() as i32;
				if v.hits[l] > 0 {
					v.seen_lh[l] = now;
				}
			}
			g.stations = nst;
			g.sensors = nsens;
			let mut light = 0;
			if g.state == SURVIVE_DIAG_STATE_COASTING || (g.state == SURVIVE_DIAG_STATE_DARK && g.light_age > 0.1) {
				light = 2;
			} else if (g.state == SURVIVE_DIAG_STATE_TRACKING || g.state == SURVIVE_DIAG_STATE_REACQUIRING) && nst <= 1
			{
				light = 1;
			}
			if radio < 2 && !off {
				let fmt = if light == 2 {
					"no light for %.1f s"
				} else if nst == 1 {
					"only one base station sees it"
				} else {
					"barely any light reaches it"
				};
				let light_age = g.light_age;
				add_cause(&mut g, cause::LIGHT, light, fmt, &[Arg::F(light_age)]);
			}

			// reflections
			let so = &rt.objects[obj];
			let fchk = so.stats.facing_checks.wrapping_sub(v.fchk0);
			let fdrop = so.stats.facing_drops.wrapping_sub(v.fdrop0);
			if fchk > 20 {
				g.back = fdrop as f64 / fchk as f64; // light-facing-deg is on: it judged (and dropped) them
			} else {
				g.back = if v.judged > 20 { v.back as f64 / v.judged as f64 } else { 0.0 };
			}
			g.vmax = v.vmax;
			// the tracker has no per-hit gate count: 0
			g.gated = 0;
			// the stations the solver dropped (and the one it dropped most, which the cause names) came from the
			// solve_report hook, which nothing raises (see the top): none
			g.discards = 0;
			// Reflections are mild (1) at most: measured against Valve (p90 error per window, normal 7 mm), windows with
			// them run 10-26 mm on some sessions, but on another the filter's gate threw out 20+ hits in hundreds of
			// windows at 7.7 mm, i.e. present and filtered. How many hits it takes says nothing reliable about the harm.
			let refl = if g.back > 0.05 || g.discards >= 1 || g.gated >= 5 { 1 } else { 0 };
			if refl != 0 {
				let args = [Arg::F(100.0 * g.back), Arg::I(g.gated as i64)];
				add_cause(
					&mut g,
					cause::REFLECT,
					refl,
					"%.0f%% of hits from behind sensors, %d rejected as off the pose",
					&args,
				);
			}

			// base stations this device was using
			for l in 0..DG_MAXLH {
				if l as i32 >= rt.active_lighthouses {
					break;
				}
				let using = now - v.seen_lh[l] < 5.0;
				if !using {
					continue;
				}
				let mode = [Arg::I(rt.bsd[l].mode as i64)];
				if lh_bad[l] {
					add_cause(
						&mut g,
						cause::STATION,
						2,
						"base station ch %d disagrees with the map (moved?); re-solving it",
						&mode,
					);
				} else if lh_moved[l] {
					add_cause(&mut g, cause::STATION, 1, "base station ch %d was moved; sally re-solved it", &mode);
				} else if lh_gone[l] {
					let sev = if nst >= 2 { 1 } else { 2 };
					add_cause(&mut g, cause::STATION, sev, "base station ch %d went dark for every device", &mode);
				}
			}

			// the device itself
			let so = &rt.objects[obj];
			if so.sensor_locations.is_none() || so.sensor_ct == 0 {
				add_cause(&mut g, cause::DEVICE, 2, "no sensor layout (device config missing)", &[]);
			} else if so.charge > 0 && so.charge < 10 && v.first_rt >= 0.0 && now - v.first_rt > 10.0 {
				let sev = if so.charge < 5 { 2 } else { 1 };
				add_cause(&mut g, cause::DEVICE, sev, "battery at %d%%", &[Arg::I(so.charge as i64)]);
			}

			// headline: the worst cause, earlier in the list on a tie
			for c in 0..g.causes.len() {
				let (sev, cause) = (g.causes[c].severity, g.causes[c].cause);
				if sev > g.severity || (sev == g.severity && cause < g.headline) {
					g.severity = sev;
					g.headline = cause;
				}
			}

			let line = diag_line(&codename, now, &g, late_sev, late10);
			rt.record_line(&line);
			rt.diagnosis_hook(obj, &g);

			// next window
			let so = &rt.objects[obj];
			v.hits = [0; DG_MAXLH];
			v.lit = [0; DG_MAXLH];
			v.back = 0;
			v.judged = 0;
			v.imu_n = 0;
			v.imu_expect = 0;
			v.imu_gap_max = 0.0;
			v.fchk0 = so.stats.facing_checks;
			v.fdrop0 = so.stats.facing_drops;
			v.vmax = 0.0;
			v.late_hist = [0; DG_LATEBINS];
			v.late_n = 0;
		}
		self.lh_hits = [0; DG_MAXLH];
	}
}

impl Driver for Diagnosis {
	/// sally: a device started over (another one on its dongle): what was kept of it goes, its place in the order stays
	fn device_reset(&mut self, _rt: &mut Runtime, obj: ObjectId) {
		if let Some(v) = self.devs.iter_mut().find(|v| v.so == obj) {
			*v = Device::new(obj);
		}
	}

	/// imupose_fn: the device's latest pose in IMU space, and its speed then (the filter's velocity, before this
	/// report's: the velocity hook comes after this one)
	fn imupose(&mut self, rt: &mut Runtime, obj: ObjectId, _timecode: u64, pose: &Pose) {
		let Some(i) = self.get_dev(obj) else { return };
		if quatiszero(&pose.rot) {
			return;
		}
		let v = &mut self.devs[i];
		v.R = q2R(&pose.rot);
		v.p = pose.pos;
		v.have_pose = true;
		let sp = norm3d(&rt.objects[obj].velocity.pos);
		if sp > v.vmax {
			v.vmax = sp;
		}
	}

	/// sweep_angle_fn: a hit from a station: counted, its lateness against the device's IMU stream binned, and, while
	/// the filter tracks, judged by the pose (a sensor facing away from its station cannot have seen it)
	fn sweep_angle(
		&mut self, rt: &mut Runtime, obj: ObjectId, channel: u8, sensor_id: i32, timecode: u32, _plane: i8, _angle: f64,
	) {
		let lh = rt.get_bsd_idx(channel);
		if lh < 0 || lh as usize >= DG_MAXLH {
			return;
		}
		let lh = lh as usize;
		if let Some(i) = self.get_dev(obj) {
			let v = &mut self.devs[i];
			v.any_report = true;
			v.hits[lh] += 1;
			self.lh_hits[lh] += 1;
			if sensor_id >= 0 && sensor_id < 64 {
				v.lit[lh] |= 1u64 << sensor_id;
			}
			// how far the device's IMU stream is ahead of this hit as it arrives (both on the device's clock): the radio
			// holding its light back
			let so = &rt.objects[obj];
			if v.imu_seen != 0 && so.timebase_hz > 0 {
				let late = v.imu_last_raw.wrapping_sub(timecode) as i32 as f64 / so.timebase_hz as f64;
				if late > -0.05 && late < 0.5 {
					let bin = if late <= 0.0 { 0 } else { (late * 1e3) as i32 };
					v.late_hist[if bin < DG_LATEBINS as i32 { bin as usize } else { DG_LATEBINS - 1 }] += 1;
					v.late_n += 1;
				}
			}
			// facing: only while the filter tracks, so the pose is worth judging by. The C checks the normals only: a
			// device with normals and no locations would read through a NULL pointer there, and is not judged here
			let judge = v.have_pose
				&& so.sensor_normals.is_some()
				&& sensor_id >= 0
				&& sensor_id < so.sensor_ct as i32
				&& rt.bsd[lh].position_set;
			if judge && rt.tracker_diag(obj).state == SURVIVE_DIAG_STATE_TRACKING {
				let so = &rt.objects[obj];
				if let (Some(loc), Some(nrm)) = (so.sensor_locations.as_deref(), so.sensor_normals.as_deref()) {
					let j = sensor_id as usize * 3;
					let wp = mv3(&v.R, &[loc[j], loc[j + 1], loc[j + 2]]);
					let wn = mv3(&v.R, &[nrm[j], nrm[j + 1], nrm[j + 2]]);
					let mut to = [0.0; 3];
					for k in 0..3 {
						to[k] = rt.bsd[lh].pose.pos[k] - (wp[k] + v.p[k]);
					}
					let nt = (to[0] * to[0] + to[1] * to[1] + to[2] * to[2]).sqrt();
					let nn = (wn[0] * wn[0] + wn[1] * wn[1] + wn[2] * wn[2]).sqrt();
					if nt > 0.0 && nn > 0.0 {
						v.judged += 1;
						if (wn[0] * to[0] + wn[1] * to[1] + wn[2] * to[2]) / (nt * nn) < self.cos_back {
							v.back += 1;
						}
					}
				}
			}
		}
		self.tick(rt);
	}

	/// imu_fn: an IMU sample, on the device's own clock: the samples it stands for (itself and the ones its gap skipped),
	/// and the usual interval learned
	fn imu(&mut self, rt: &mut Runtime, obj: ObjectId, _mask: i32, _agm: &[f64; 9], timecode: u32, _id: i32) {
		let timebase_hz = rt.objects[obj].timebase_hz;
		if timebase_hz <= 0 {
			return;
		}
		if let Some(i) = self.get_dev(obj) {
			let v = &mut self.devs[i];
			v.any_report = true;
			// the step on the device's own clock (32 bits, signed: the difference wraps correctly). A sample at or a
			// little behind the last one (the controllers send a few a minute out of order or twice) is received but
			// does not move the clock; after a silence of 5 s or more, or a clock that went back (a device restart), the
			// clock is taken up again without counting the gap (the silent windows were counted as lost already)
			let dt = if v.imu_seen != 0 {
				timecode.wrapping_sub(v.imu_last_raw) as i32 as f64 / timebase_hz as f64
			} else {
				0.0
			};
			let ahead = v.imu_seen != 0 && dt > 0.0 && dt < 5.0;
			let behind = v.imu_seen != 0 && dt <= 0.0 && dt > -0.1;
			if !behind {
				v.imu_last_raw = timecode;
			}
			let mut k = if behind { 0 } else { 1 }; // the samples this one stands for: itself and the ones its gap skipped
			if ahead {
				// the usual interval: the shortest of the first 50 steps (the first step can be a skipped sample), then
				// it follows the common case and ignores gaps
				if v.imu_seen <= 50 {
					if dt > 0.0015 && dt < 0.05 && (v.imu_dt <= 0.0 || dt < v.imu_dt) {
						v.imu_dt = dt;
					}
				} else if dt > 0.7 * v.imu_dt && dt < 1.3 * v.imu_dt {
					v.imu_dt += 0.01 * (dt - v.imu_dt);
				}
				if v.imu_seen > 50 && v.imu_dt > 0.0 && dt > 5.0 * v.imu_dt && dt > 0.02 && dt > v.imu_gap_max {
					v.imu_gap_max = dt;
				}
				// (dt is under 5 s and the interval no shorter than a fraction of the first steps': in an int's range)
				k = if v.imu_dt > 0.0 { (dt / v.imu_dt + 0.5).floor() as i32 } else { 1 };
			}
			v.imu_expect += k;
			v.imu_seen = v.imu_seen.wrapping_add(1);
			v.imu_n += 1;
			v.last_report_rt = rt.run_time();
			if v.first_rt < 0.0 {
				v.first_rt = v.last_report_rt;
			}
		}
		self.tick(rt);
	}

	/// At close: the options unbound, and the session's light lateness per device: the windows' median and p90, and how
	/// much of it was held back
	fn close(&mut self, rt: &mut Runtime) {
		for (tag, var) in self.config_items() {
			rt.detach_config(tag, var);
		}
		for v in &self.devs {
			if v.late_windows < 20 {
				continue;
			}
			let (mut b50, mut b90, mut acc) = (-1i32, -1i32, 0);
			for b in 0..DG_LATEBINS {
				acc += v.late_sess[b];
				if b50 < 0 && 2 * acc >= v.late_windows {
					b50 = b as i32;
				}
				if b90 < 0 && 10 * acc >= 9 * v.late_windows {
					b90 = b as i32;
				}
			}
			let so = &rt.objects[v.so];
			let msg = sprintf(
				concat!(
					"%s (%s) diagnosis: its light came %d ms after its IMU stream (the median of %d half-second windows, ",
					"p90 %d), held back %.0f ms or more (the 60 s median) in %.0f %% of them",
				),
				&[
					Arg::S(&so.codename),
					Arg::S(&so.serial_number),
					Arg::I(b50 as i64),
					Arg::I(v.late_windows as i64),
					Arg::I(b90 as i64),
					Arg::F(DG_LATE_ON * 1e3),
					Arg::F(100.0 * v.late_flagged as f64 / v.late_windows as f64),
				],
			);
			rt.info(&msg);
		}
	}
}

#[cfg(test)]
mod tests {
	use std::sync::{Arc, Mutex};

	use sally_math::quatrotatevector;

	use super::*;
	use crate::config::Config;
	use crate::log::{Log, LogLevel};
	use crate::options;

	/// 2 ms of a device's 48 MHz clock: the IMU's interval in these tests
	const TICKS: u64 = 96_000;

	/// What a test sees: the log's lines, and the verdicts the diagnosis hook hands on
	#[derive(Default)]
	struct Seen {
		log: Mutex<Vec<String>>,
		verdicts: Mutex<Vec<(ObjectId, DeviceDiagnosis)>>,
	}

	type Lines = Arc<Seen>;

	/// A driver on the diagnosis hook, as the runtime protocol is: it keeps every verdict
	struct Verdicts(Lines);

	impl Driver for Verdicts {
		fn diagnosis(&mut self, _rt: &mut Runtime, obj: ObjectId, g: &DeviceDiagnosis) {
			self.0.verdicts.lock().unwrap().push((obj, g.clone()));
		}
	}

	/// A runtime with the options registered and `args` on its command line, on the replay's clock, its log lines
	/// kept
	fn runtime(args: &[&str]) -> (Runtime, Lines) {
		let mut config = Config::default();
		options::register(&mut config);
		let argv: Vec<&str> = std::iter::once("sally").chain(args.iter().copied()).collect();
		config.apply_args(&argv);
		let mut rt = Runtime::new(config, Log::new(None, false));
		let seen: Lines = Arc::default();
		let sink = Arc::clone(&seen);
		rt.install_log_fn(Some(Box::new(move |_: LogLevel, msg: &str| sink.log.lock().unwrap().push(msg.to_string()))));
		rt.set_replay_time(0.0);
		(rt, seen)
	}

	/// runtime(), the diagnosis registered, a driver on its hook, and `n` devices (KN0 with serial SER0, ...) with a
	/// sensor layout, so that only the causes a test makes show (none has a tracker that is fed: their state stays
	/// none)
	fn setup(args: &[&str], n: usize) -> (Runtime, Diagnosis, Vec<ObjectId>, Lines) {
		let (mut rt, seen) = runtime(args);
		let d = Diagnosis::reg(&mut rt).unwrap();
		rt.add_driver("Verdicts", Box::new(Verdicts(Arc::clone(&seen))));
		let objs = (0..n)
			.map(|i| {
				let obj = rt.create_object("HID", &format!("KN{i}"), &format!("SER{i}"), &format!("SER{i}")).unwrap();
				let so = &mut rt.objects[obj];
				so.sensor_ct = 1;
				so.sensor_locations = Some(vec![0.0; 96]);
				obj
			})
			.collect();
		(rt, d, objs, seen)
	}

	/// A device's sample i, at run time i * 2 ms: its IMU (timecode i * TICKS), then, with `late`, a hit `late` s
	/// behind it from the station on channel 5
	fn sample(d: &mut Diagnosis, rt: &mut Runtime, obj: ObjectId, i: u64, late: Option<f64>) {
		rt.set_replay_time(i as f64 * 0.002);
		let tc = (i * TICKS) as u32;
		d.imu(rt, obj, 3, &[0.0; 9], tc, 0);
		if let Some(late) = late {
			d.sweep_angle(rt, obj, 5, 0, tc.wrapping_sub((late * 48e6) as u32), 0, 0.0);
		}
	}

	/// Samples from..=to of one device
	fn run(d: &mut Diagnosis, rt: &mut Runtime, obj: ObjectId, from: u64, to: u64, late: Option<f64>) {
		for i in from..=to {
			sample(d, rt, obj, i, late);
		}
	}

	/// The device's latest verdict, as the diagnosis hook handed it on
	fn latest(seen: &Lines, obj: ObjectId) -> DeviceDiagnosis {
		let verdicts = seen.verdicts.lock().unwrap();
		verdicts.iter().rev().find(|(o, _)| *o == obj).map(|(_, g)| g.clone()).unwrap()
	}

	fn logged(seen: &Lines, what: &str) -> Vec<String> {
		seen.log.lock().unwrap().iter().filter(|l| l.contains(what)).cloned().collect()
	}

	/// late_update's median of a bin: (b + .5) ms
	fn med(b: usize) -> f64 {
		(b as f64 + 0.5) * 1e-3
	}

	/// Lines of the C's recordings, and the tokens the causes add
	#[test]
	fn the_diag_line_is_the_cs() {
		// a recorded window: KN0 at 2.508 s
		let mut g = DeviceDiagnosis {
			headline: cause::RADIO,
			severity: 1,
			bars: 2,
			loss: 0.083,
			gap_max: 0.002,
			imu_n: 125,
			stations: 4,
			sensors: 39,
			state: SURVIVE_DIAG_STATE_TRACKING,
			light_age: 0.02,
			vmax: 1.03,
			..Default::default()
		};
		add_cause(&mut g, cause::RADIO, 1, "%s%s%s", &[Arg::S("8% of reports lost"), Arg::S(""), Arg::S("")]);
		assert_eq!(
			diag_line("KN0", 2.508062, &g, 0, -1.0),
			"KN0 DIAG 2.508 radio 1 2 8.3 2 125 4 39 0.0 0 0 tracking 0.02 0 1.03 radio:1\n"
		);
		assert_eq!(g.causes[0].text, "radio: weak link to the device: 8% of reports lost");
		// a recorded window: KN0 at 3.508 s
		let g = DeviceDiagnosis {
			bars: 4,
			imu_n: 126,
			stations: 4,
			sensors: 29,
			state: SURVIVE_DIAG_STATE_TRACKING,
			vmax: 0.64,
			..Default::default()
		};
		assert_eq!(
			diag_line("KN0", 3.508111, &g, 0, -1.0),
			"KN0 DIAG 3.508 fine 0 4 0.0 0 126 4 29 0.0 0 0 tracking 0.00 0 0.64 fine\n"
		);
		// the radio's lateness, and the station a cause names
		let mut g = DeviceDiagnosis {
			headline: cause::STATION,
			severity: 2,
			bars: 4,
			state: SURVIVE_DIAG_STATE_DARK,
			light_age: 1.5,
			charge: 67,
			..Default::default()
		};
		add_cause(&mut g, cause::RADIO, 1, "%s%s%s", &[Arg::S(""), Arg::S(""), Arg::S("its light comes 25 ms late")]);
		add_cause(&mut g, cause::STATION, 2, "base station ch %d went dark for every device", &[Arg::I(13)]);
		assert_eq!(
			diag_line("KN3", 61.0, &g, 1, 0.0243),
			"KN3 DIAG 61.000 station 2 4 0.0 0 0 0 0 0.0 0 0 dark 1.50 67 0.00 radio:1:late24 station:2:ch13\n"
		);
		assert_eq!(g.causes[1].text, "base station problem: base station ch 13 went dark for every device");
	}

	/// snprintf's cuts: the line's 1200 bytes (and nothing past them), a cause's detail (96; the text's 160 hold the
	/// longest name and detail, 151 bytes); no cause at severity 0 or past the eighth
	#[test]
	fn texts_are_cut_as_the_cs_buffers_cut_them() {
		let mut line = Line::default();
		line.push(&"x".repeat(1198));
		line.push("abc");
		assert_eq!((line.text.len(), line.n), (1199, 1201));
		assert!(line.text.ends_with("xa"));
		line.push("d");
		assert_eq!((line.text.len(), line.n), (1199, 1202));

		let mut g = DeviceDiagnosis::default();
		add_cause(&mut g, cause::DEVICE, 1, "%s", &[Arg::S(&"y".repeat(200))]);
		assert_eq!((g.causes[0].detail.len(), g.causes[0].text.len()), (95, 111));
		assert!(g.causes[0].text.starts_with("device problem: yyy"));
		add_cause(&mut g, cause::LIGHT, 0, "x", &[]);
		for _ in 0..10 {
			add_cause(&mut g, cause::REFLECT, 1, "x", &[]);
		}
		assert_eq!(g.causes.len(), MAX_CAUSES);
		assert_eq!((cause_name(6), cause_text(-1), state_name(5)), ("?", "?", "?"));
	}

	/// A window's median bin, and the verdict's measure: the median of the judged windows of the last 120.5 periods,
	/// once there are 60
	#[test]
	fn the_lateness_is_the_median_of_the_windows_medians() {
		let window = |v: &mut Device, bins: &[(usize, i32)]| {
			v.late_hist = [0; DG_LATEBINS];
			v.late_n = 0;
			for &(b, n) in bins {
				v.late_hist[b] += n;
				v.late_n += n;
			}
		};
		let mut v = Device::new(0);
		window(&mut v, &[(30, 49)]);
		assert_eq!(late_update(&mut v, 0.25, 0.5), (-1.0, false));
		assert_eq!(v.late_windows, 0);
		for i in 1..60 {
			window(&mut v, &[(25, 50)]);
			assert_eq!(late_update(&mut v, i as f64 * 0.5, 0.5), (-1.0, true));
		}
		window(&mut v, &[(2, 30), (30, 30)]);
		assert_eq!(late_update(&mut v, 30.0, 0.5), (med(25), true));
		assert_eq!(v.late_med[59], med(2));
		window(&mut v, &[(2, 29), (30, 31)]);
		late_update(&mut v, 30.5, 0.5);
		assert_eq!(v.late_med[60], med(30));
		// 63 ms and more: the last bin
		window(&mut v, &[(63, 80)]);
		late_update(&mut v, 31.0, 0.5);
		assert_eq!((v.late_med[61], v.late_sess[63], v.late_windows), (med(63), 1, 62));
		// none judged within the last 60.25 s
		window(&mut v, &[]);
		assert_eq!(late_update(&mut v, 91.5, 0.5), (-1.0, false));
	}

	/// Light 25.5 ms behind the IMU stream: flagged once 60 judged windows (30 s) say so, with its log line and a mild
	/// radio cause; on time again once those are the majority of the last 60 s; and the session's summary at close
	#[test]
	fn light_held_back_is_flagged_and_cleared() {
		let (mut rt, mut d, objs, seen) = setup(&[], 1);
		let kn0 = objs[0];
		let mut on_at = None;
		for i in 1..=35_000 {
			sample(&mut d, &mut rt, kn0, i, Some(0.0255));
			if on_at.is_none() && d.devs[0].late_on {
				on_at = Some(i);
			}
		}
		assert!((14_990..=15_100).contains(&on_at.unwrap()), "{on_at:?}");
		let held = format!(
			concat!(
				"KN0 (SER0) diagnosis: its light comes {:.0} ms late (the median over 60 s; usually 0-3, or 14-18 when ",
				"held): its radio is holding the light back",
			),
			med(25) * 1e3
		);
		assert_eq!(logged(&seen, " diagnosis: "), std::slice::from_ref(&held));
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity, g.bars, g.causes.len()), (cause::RADIO, 1, 4, 1));
		assert_eq!(g.causes[0].detail, "its light comes 25 ms late");
		let token = format!(" radio:1:late{:.0}\n", med(25) * 1e3);
		assert!(diag_line("KN0", g.run_time, &g, 1, med(25)).ends_with(&token));

		run(&mut d, &mut rt, kn0, 35_001, 55_000, Some(0.0005));
		assert!(!d.devs[0].late_on);
		let on_time = format!("KN0 (SER0) diagnosis: its light is on time again ({:.0} ms)", med(0) * 1e3);
		assert_eq!(logged(&seen, " diagnosis: "), [held, on_time]);
		assert!(latest(&seen, kn0).causes.is_empty());

		let (n, flagged) = (d.devs[0].late_windows, d.devs[0].late_flagged);
		assert!(n > 200 && flagged > 0);
		d.close(&mut rt);
		let summary = format!(
			concat!(
				"KN0 (SER0) diagnosis: its light came 25 ms after its IMU stream (the median of {} half-second windows, ",
				"p90 25), held back 19 ms or more (the 60 s median) in {:.0} % of them",
			),
			n,
			100.0 * flagged as f64 / n as f64
		);
		assert_eq!(logged(&seen, " diagnosis: ").last(), Some(&summary));
	}

	/// Light 40 ms late or more hurts (2)
	#[test]
	fn light_held_back_long_hurts() {
		let (mut rt, mut d, objs, seen) = setup(&[], 1);
		run(&mut d, &mut rt, objs[0], 1, 16_000, Some(0.0455));
		let g = latest(&seen, objs[0]);
		assert_eq!((g.headline, g.severity), (cause::RADIO, 2));
		assert_eq!(g.causes[0].detail, "its light comes 45 ms late");
	}

	/// Every fourth report lost: a quarter of the samples by the device's own clock, one bar, the radio hurting. Then
	/// silent: the windows count as lost, with the open gap; past 3 s the device is off, not a weak link
	#[test]
	fn reports_lost_and_a_device_gone_silent() {
		let (mut rt, mut d, objs, seen) = setup(&[], 2);
		let (kn0, kn1) = (objs[0], objs[1]);
		for i in 1..=5_000 {
			if i % 4 != 0 {
				sample(&mut d, &mut rt, kn0, i, None);
			}
			sample(&mut d, &mut rt, kn1, i, None);
		}
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity, g.bars), (cause::RADIO, 2, 1));
		assert_eq!(g.causes[0].detail, "25% of reports lost");
		assert!(latest(&seen, kn1).causes.is_empty());

		run(&mut d, &mut rt, kn1, 5_001, 5_700, None);
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity, g.imu_n), (cause::RADIO, 2, 0));
		assert!(g.causes[0].detail.ends_with("% of reports lost, gaps up to a tenth of a second or more"));

		run(&mut d, &mut rt, kn1, 5_701, 7_500, None);
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity, g.bars, g.causes.len()), (cause::DEVICE, 2, 0, 1));
		assert!(g.causes[0].detail.starts_with("no reports for "));
		assert!(g.causes[0].detail.ends_with(" s (switched off, flat, or out of range)"));
	}

	/// A low battery is a device cause once the device has reported for 10 s: mild under 10 %, hurting under 5 %
	#[test]
	fn a_low_battery() {
		let (mut rt, mut d, objs, seen) = setup(&[], 1);
		let kn0 = objs[0];
		rt.objects[kn0].charge = 7;
		run(&mut d, &mut rt, kn0, 1, 4_750, None);
		assert!(latest(&seen, kn0).causes.is_empty());
		run(&mut d, &mut rt, kn0, 4_751, 5_300, None);
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity, g.charge), (cause::DEVICE, 1, 7));
		assert_eq!(g.causes[0].text, "device problem: battery at 7%");
		rt.objects[kn0].charge = 3;
		run(&mut d, &mut rt, kn0, 5_301, 5_600, None);
		assert_eq!(latest(&seen, kn0).severity, 2);
		rt.objects[kn0].charge = 10;
		run(&mut d, &mut rt, kn0, 5_601, 5_900, None);
		assert!(latest(&seen, kn0).causes.is_empty());
	}

	/// The stations: one gone dark for every device (seen in the last 10 s, not for 1 s), and map-solve's verdicts,
	/// each naming the station's channel
	#[test]
	fn station_causes() {
		let (mut rt, mut d, objs, seen) = setup(&[], 1);
		let kn0 = objs[0];
		run(&mut d, &mut rt, kn0, 1, 1_000, Some(0.0005));
		assert!(latest(&seen, kn0).causes.is_empty());
		run(&mut d, &mut rt, kn0, 1_001, 2_000, None);
		let g = latest(&seen, kn0);
		assert_eq!(g.causes.len(), 1);
		assert_eq!((g.causes[0].cause, g.causes[0].severity), (cause::STATION, 2));
		assert_eq!(g.causes[0].text, "base station problem: base station ch 5 went dark for every device");
		assert!(diag_line("KN0", g.run_time, &g, 0, -1.0).ends_with(" station:2:ch5\n"));

		rt.bsd[0].sally_moved = 1;
		run(&mut d, &mut rt, kn0, 2_001, 2_300, Some(0.0005));
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity), (cause::STATION, 2));
		assert_eq!(g.causes[0].detail, "base station ch 5 disagrees with the map (moved?); re-solving it");

		rt.bsd[0].sally_moved = 2;
		rt.bsd[0].sally_moved_time = 4.0;
		run(&mut d, &mut rt, kn0, 2_301, 2_600, Some(0.0005));
		let g = latest(&seen, kn0);
		assert_eq!((g.headline, g.severity), (cause::STATION, 1));
		assert_eq!(g.causes[0].detail, "base station ch 5 was moved; sally re-solved it");
		rt.bsd[0].sally_moved_time = -30.0;
		run(&mut d, &mut rt, kn0, 2_601, 2_900, Some(0.0005));
		assert!(latest(&seen, kn0).causes.is_empty());
	}

	/// The options are bound in the store: the period clamped at registration, a set while running reaching the
	/// variables (unclamped, as in the C) and the cosine at the next window, nothing reaching them once closed; at
	/// diagnosis 0 no driver
	#[test]
	fn the_options_are_bound() {
		let (mut rt, mut d, objs, _) = setup(&["--diagnosis-period", "0.01", "--diagnosis-back-deg", "60"], 1);
		assert_eq!(d.period.float(), 0.05);
		assert_eq!(d.cos_back, (60.0 * LINMATHPI / 180.0).cos());
		assert_eq!(rt.set_live("diagnosis-period", "0.01"), 1);
		assert_eq!(rt.set_live("diagnosis-back-deg", "90"), 1);
		assert_eq!((d.period.float(), d.back_deg.float()), (0.01, 90.0));
		run(&mut d, &mut rt, objs[0], 1, 10, None);
		assert_eq!(d.cos_back, (90.0 * LINMATHPI / 180.0).cos());
		d.close(&mut rt);
		assert_eq!(rt.set_live("diagnosis-period", "1"), 0);
		assert_eq!(d.period.float(), 0.01);

		let (mut rt, _) = runtime(&["--diagnosis", "0"]);
		assert!(Diagnosis::reg(&mut rt).is_none());
	}

	/// The facing test's rotation: the quaternion's matrix by rows, as quatrotatevector turns a vector
	#[test]
	fn the_rotation_matrix_turns_as_the_quaternion() {
		let q: Quat = [0.9, 0.1, -0.3, 0.2];
		let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
		let unit = [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
		let x = [0.3, -1.2, 2.5];
		let (a, b) = (mv3(&q2R(&q), &x), quatrotatevector(&unit, &x));
		for k in 0..3 {
			assert!((a[k] - b[k]).abs() < 1e-12, "{a:?} {b:?}");
		}
	}
}
