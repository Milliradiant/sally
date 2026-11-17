//! The tracker (docs/tracking.md): one error-state EKF per device, driven by its IMU, with every sweep of light applied
//! as a measurement at its own time.
//!
//! 21 error states: position (3), velocity (3), orientation (3, a small rotation applied on the world side of the
//! quaternion), gyro bias (3), accelerometer bias (3), sensor-layout scale (1), the light's orientation offset (3,
//! OB_SD), gravity's tilt in the map frame (2). The state point is the IMU itself: the accelerometer measures that
//! point, so nothing is transferred (moving it to the head point takes the angular acceleration from differenced gyro
//! samples, 80-300 m/s^2 of noise on a shaken controller). Propagated by every IMU sample, trapezoidal: the mean of the
//! two gyro samples turns the orientation, and the two accelerations are each rotated by the orientation of their own
//! time (rotating the new one by the step's start orientation puts the step's rotation, 0.1 rad at 25 rad/s, into the
//! acceleration, gravity included). Every sweep (the hits of one station and plane within 1.5 ms of each other) is one
//! update, each hit evaluated at its own time, with a per-hit gate on the predicted innovation variance. The light
//! arrives after the IMU (a hit's report comes 5-15 ms after the IMU sample of its time), so a sweep is evaluated
//! against the kept state of the IMU sample at or before its time and the correction applied to the current state and
//! to that history. Starts and recovers on its own: a pose solve on its own recent sweeps (seed_solve: a
//! barycentric-SVD pose from the best station as the first guess, then a Gauss-Newton fit of position, orientation and
//! velocity over every station's hits of the last track-seed-window s, with the gyro carrying the orientation between
//! the sweeps), taken when it fits the hits to track-seed-fit. The same solve restarts the state after a divergence
//! (more than 80 % of the last half second's hits rejected) or when the light returns after a blackout and the coasted
//! state no longer fits. Reports the state at every IMU sample through the imupose / velocity hooks (invoked by the
//! host with what integrate_imu returns), so everything downstream (recording, driver, diagnosis) sees the tracker and
//! nothing else.
//!
//! Process noise is per IMU step and scales with the step (sd = sigma * dt per step); the settings are tuned at 250 Hz,
//! and a 1000 Hz device needs retuning.
//!
//! Exactness: the arithmetic is that of the C this was ported from, operation for operation, so that a replay computes
//! the same bits (docs/checking.md). `int` is i32 with wrapping where the C would wrap, `uint64_t` u64, `long` c_long;
//! C's implicit int-to-double conversions are written where the C makes them (`x + 0` adds 0.0: it turns -0 into +0);
//! fmax / fmin are the C library's (sally_math::c_fmax); the medians keep an insertion sort (with a NaN in the list the
//! order is the algorithm's); every matrix product keeps its zero terms (a zero times a NaN is a NaN).
// the matrices keep their usual names (P, F, H, K, R, N ...)
#![allow(non_snake_case)]

use std::ffi::{c_int, c_long, c_uint};

use crate::cold_start::BarycentricSvd;
use crate::host::Host;
use crate::host::{
	BaseStationCal, Settings, SurvivePose, SurviveVelocity, NUM_GEN2_LIGHTHOUSES, SURVIVE_DIAG_STATE_COASTING,
	SURVIVE_DIAG_STATE_DARK, SURVIVE_DIAG_STATE_NONE, SURVIVE_DIAG_STATE_REACQUIRING, SURVIVE_DIAG_STATE_TRACKING,
	SURVIVE_OBJECT_TYPE_CONTROLLER,
};
use crate::learned::{Est, Learned, Limits, Prior, NQ};
use sally_lighthouse::{bc_fill_m, gen2_axis_consts, survive_reproject_axis_gen2_with, Gen2AxisConsts};
use sally_math::{
	apply_pose_to_point, apply_pose_to_pose, c_fmax, c_fmin, c_pow, dot3d, invert_pose, norm3d, normalize3d,
	quatfrommatrix33, quatgetreciprocal, quatiszero, quatrotateabout, quatrotatevector, sub3d, Quat, LINMATHPI,
};

const NLH: usize = NUM_GEN2_LIGHTHOUSES;

const NS: usize = 21; // error-state size
const IP: usize = 0; // position
const IV: usize = 3; // velocity
const IQ: usize = 6; // orientation
const IBG: usize = 9; // gyro bias
const IBA: usize = 12; // accel bias
const ILS: usize = 15; // layout scale
const IOB: usize = 16; // the light's orientation offset (world-frame rotation, rad): see OB_SD
const IGT: usize = 19; // gravity's direction in the map frame: its tilt from the map's z about x and y (rad): see track-gravity-tilt-sd
const HMAX: usize = 32; // hits per sweep (sensors per device)
const SWMAX: usize = 32; // pending sweeps
const VIBN: usize = 11; // vibration history (IMU samples): 10 sample-to-sample differences
const STUCKN: usize = 16; // gyro history (samples, 64 ms) for the stuck-axis test
const RECENT: usize = 256; // sweep outcomes kept for the divergence test
const HISTN: usize = 64; // state history (0.25 s at 250 Hz): the light arrives 5-150 ms after its IMU sample
const SEEDN: usize = 96; // closed sweeps kept for the seed solve (4 stations x 2 planes x ~50 Hz = 0.24 s)
const SEEDX: usize = 9; // seed solve parameters: position, orientation, velocity
const ACCQN: usize = 8; // accelerometer samples kept for the accelerometer's own time, ACC_LEAD_* (32 ms at 250 Hz)
const GRAV: f64 = 9.80665;

// the ring buffers' sizes as i32, for the index arithmetic
const HISTN_I: i32 = HISTN as i32;
const SEEDN_I: i32 = SEEDN as i32;
const STUCKN_I: i32 = STUCKN as i32;
const VIBN_I: i32 = VIBN as i32;
const ACCQN_I: i32 = ACCQN as i32;
const RECENT_I: i32 = RECENT as i32;

#[derive(Clone, Copy, Default)]
struct Hist {
	t: f64,
	p: [f64; 3],
	v: [f64; 3],
	w: [f64; 3],
	dr_p: [f64; 3],
	dr_v: [f64; 3], // the IMU-only motion accumulator at this time (propagate_state)
	q: Quat,
	// the process noise of the IMU step that ended here (propagate_P; none where P was not propagated): the velocity's
	// per axis, the orientation's (world, 3 x 3), the light offset's per axis, and the offset's decay in the step (STALE_Q)
	nq_v: f64,
	nq_th: [f64; 9],
	nq_ob: f64,
	kob: f64,
}

/// the pose the hits of a sweep are evaluated against: a state at a time at or before the sweep, carried to each hit's
/// own time with its velocity and body rate
#[derive(Clone, Copy, Default)]
struct Base {
	t: f64,
	p: [f64; 3],
	v: [f64; 3],
	w: [f64; 3],
	dr_p: [f64; 3],
	dr_v: [f64; 3],
	q: Quat,
}

/// what the predictions of one sweep's hits from one base share, whatever their perturbation (predict's): each hit's
/// time from the base, the body's rotation over that time, and the sensor (as given, and turned by that rotation at
/// the state's layout scale); and the station's terms of the sweep's plane in the gen2 reprojection. Worked out once a
/// sweep and base (hits_at), not in each of the sweep's eight predictions
#[derive(Clone, Copy)]
struct HitsAt {
	dt: [f64; HMAX],
	e: [Quat; HMAX],
	pt: [[f64; 3]; HMAX],
	ptl: [[f64; 3]; HMAX],
	k: Gen2AxisConsts,
}
impl HitsAt {
	const ZERO: HitsAt = HitsAt {
		dt: [0.0; HMAX],
		e: [[0.0; 4]; HMAX],
		pt: [[0.0; 3]; HMAX],
		ptl: [[0.0; 3]; HMAX],
		k: Gen2AxisConsts { tan_a: 0.0, sin_ydeg: 0.0, cos_ydeg: 0.0 },
	};
}

/// a closed sweep kept for the seed solve, with the gyro-integrated orientation and body rate at its time
#[derive(Clone, Copy)]
struct SeedSweep {
	lh: i32,
	plane: i32,
	n: i32,
	t: f64, // sweep time on the IMU clock
	q: Quat,
	w: [f64; 3],
	dp: [f64; 3],
	dv: [f64; 3], // the IMU-only motion accumulator at the sweep's time: differences between sweeps are the motion inside the window
	ht: [f64; HMAX],
	angle: [f64; HMAX],
	sensor: [i32; HMAX],
}
impl SeedSweep {
	const ZERO: SeedSweep = SeedSweep {
		lh: 0,
		plane: 0,
		n: 0,
		t: 0.0,
		q: [0.0; 4],
		w: [0.0; 3],
		dp: [0.0; 3],
		dv: [0.0; 3],
		ht: [0.0; HMAX],
		angle: [0.0; HMAX],
		sensor: [0; HMAX],
	};
}

/// a pending sweep (struct core2_sweep)
#[derive(Clone, Copy)]
struct Sweep {
	lh: i32,
	plane: i32,
	n: i32,
	t_min: f64,
	t_max: f64,
	t: [f64; HMAX],
	angle: [f64; HMAX],
	sensor: [i32; HMAX],
}
impl Sweep {
	const ZERO: Sweep =
		Sweep { lh: 0, plane: 0, n: 0, t_min: 0.0, t_max: 0.0, t: [0.0; HMAX], angle: [0.0; HMAX], sensor: [0; HMAX] };
}

/// a sweep's outcome for the divergence test
#[derive(Clone, Copy, Default)]
struct Recent {
	t: f64,
	used: i32,
	rej: i32,
	lh: i32,
	plane: i32, // the station and sweep plane, and whether the sweep was coherently rejected (every hit out, one common residual)
	coherent: bool,
}

/// What a seed solve found (seed_fit)
#[derive(Clone, Copy)]
struct SeedFit {
	ok: bool,      // a pose, and as a tie's trial one with both sides in it
	fit_ok: bool,  // a pose, whatever a tie's sides
	med: f64,      // the used hits' median |residual| (rad)
	stations: u32, // the stations with hits used (bsd indices)
	applied: bool, // the state was set from it
}

#[derive(Clone, Copy, Default)]
struct Stats {
	hits: u64,
	used: u64,
	rejected: u64,
	facing_drops: u64,
	sweeps: u64,
	sweeps_dropped: u64,
	reports: u64,
	stale: u64,
	seeds: i32,
	reseeds: i32,
	gap_reseeds: i32,
	seed_tries: i32,
	seed_refused: i32,
	disagreements: i32,
	clipped: u64,
	zupts: u64,
	stale_q: [u64; 4], // late sweeps by the share of their interval's noise taken out (STALE_Q): all, 1/2, 1/4, none
}

/// the hand-over look-ahead, and what the output layer needs to tell the state's corrections from its motion
pub struct Handover {
	pub lead: f64,
	pub p_imu: [f64; 3],
	pub dp: [f64; 3],
	pub dv: [f64; 3],
	pub a: [f64; 3],
	pub w: [f64; 3],
	/// every correction the state has taken since the tracker began (RETURN_DARK): the IMU point's position
	/// corrections summed (m, world) and the orientation's composed on the world side. The state moves by the IMU and
	/// by these; what a consumer sees change between two reports beyond the motion is the change of these
	pub corr_p: [f64; 3],
	pub corr_q: Quat,
	/// how long ago the light's latest return began (s), -1 before the first; and how long the device was dark before
	/// it (s)
	pub ret_age: f64,
	pub ret_gap: f64,
}

/// the device's tracking state
pub struct Diag {
	pub state: c_int,
	pub light_age: f64,
	pub pos_sd: f64,
	pub lh_mask: c_uint,
	/// the light's residual (mrad): the median |residual| of the sweeps the update takes, smoothed; -1 without one in
	/// the last 0.5 s (for the runtime protocol's S lines, the panel's; nothing in the tracker reads it)
	pub res_mrad: f64,
}

// ++ on a counter, wrapping
fn inc(x: &mut u64) {
	*x = x.wrapping_add(1);
}
fn inci(x: &mut i32) {
	*x = x.wrapping_add(1);
}

/// A ring buffer's index from its i32 arithmetic (a negative one is out of bounds: the index check stops)
fn ix(i: i32) -> usize {
	i as usize
}

/// a zeroed array on the heap (SV_CALLOC), without a stack copy of it
fn boxed<T: Copy, const N: usize>(v: T) -> Box<[T; N]> {
	match vec![v; N].into_boxed_slice().try_into() {
		Ok(b) => b,
		Err(_) => unreachable!(),
	}
}

/// Insertion sort for a median (kept: with a NaN in the list the result is the algorithm's)
fn insertion_sort(ar: &mut [f64]) {
	for a in 1..ar.len() {
		let mut b = a;
		while b > 0 && ar[b - 1] > ar[b] {
			ar.swap(b, b - 1);
			b -= 1;
		}
	}
}

/// The work arrays of the update, of the rest test's updates and of the covariance's propagation, kept with the
/// tracker rather than made at every call: zero-filled there, they were 52 KB an update and 21 KB a sample at rest
/// (the replays' memset, 11 % of their instructions). Every one is written before it is read, so a call clears only
/// H's rows of its sweep (the columns no hit's Jacobian sets are read as zeros) and never F (the identity, and the
/// entries every step writes). Out of the tracker for the call that uses them (take_work), so the tracker's own
/// methods stay free to borrow it
struct Work {
	at: HitsAt,
	pw: [f64; HMAX * 3],
	H: [f64; HMAX * NS],
	PH: [f64; HMAX * NS], // the gate's P H^T entries (a hit's row, where its H is not zero)
	Pq: [f64; NS * NS],
	PHt: [f64; NS * HMAX],
	S: [f64; HMAX * HMAX],
	S0: [f64; HMAX * HMAX],
	X: [f64; HMAX * NS],
	K: [f64; NS * HMAX],
	KS: [f64; NS * HMAX],
	KW: [f64; NS * HMAX], // the gain times each hit's own variance (the Joseph form's)
	IKH: [f64; NS * NS],
	T: [f64; NS * NS],
	Pn: [f64; NS * NS],
	L: [f64; NS * NS], // is_pd's
	F: [f64; NS * NS], // propagate_P's: the identity where no step writes
	FP: [f64; NS * NS],
	Mt: [f64; NS * NS], // a product's right factor transposed (F's, IKH's)
}
impl Work {
	fn boxed() -> Box<Work> {
		let mut w = Box::new(Work {
			at: HitsAt::ZERO,
			pw: [0.0; HMAX * 3],
			H: [0.0; HMAX * NS],
			PH: [0.0; HMAX * NS],
			Pq: [0.0; NS * NS],
			PHt: [0.0; NS * HMAX],
			S: [0.0; HMAX * HMAX],
			S0: [0.0; HMAX * HMAX],
			X: [0.0; HMAX * NS],
			K: [0.0; NS * HMAX],
			KS: [0.0; NS * HMAX],
			KW: [0.0; NS * HMAX],
			IKH: [0.0; NS * NS],
			T: [0.0; NS * NS],
			Pn: [0.0; NS * NS],
			L: [0.0; NS * NS],
			F: [0.0; NS * NS],
			FP: [0.0; NS * NS],
			Mt: [0.0; NS * NS],
		});
		for i in 0..NS {
			w.F[i * NS + i] = 1.0;
		}
		w
	}
}

pub struct Tracker {
	// settings: the tracker's copy of its options, read at every use
	s: Settings,
	gyro_clip_t: f64,
	gt0x: f64,
	gt0y: f64, // the tilt above as components about x and y
	// the accelerometer's recent samples with their times (ACC_LEAD_*)
	accq_t: [f64; ACCQN],
	accq: [[f64; 3]; ACCQN],
	accq_clip: [bool; ACCQN],
	accq_n: i32,
	accq_i: i32,
	a_prev: [f64; 3],
	g_prev: [f64; 3], // the previous IMU sample (acceleration as used, raw gyro), for the trapezoid
	have_prev: bool,
	ob: [f64; 3], // the light's orientation offset (OB_SD)
	// the handover test (track-blink): the window index and the one station kept in it
	blink_k: c_long,
	blink_lh: i32,

	// state
	seeded: bool,
	ever_seeded: bool,
	confirmed: bool, // confirmed: the sweeps after the seed solve agreed with it (confirm)
	seed_t: f64,     // time of the seed solve (device clock)
	t: f64,          // state time (s, device clock)
	p: [f64; 3],
	v: [f64; 3],
	bg: [f64; 3],
	ba: [f64; 3],
	q: Quat,
	ls: f64,
	// what earlier sessions learned of the device (learned.rs): the calibration states' start at the first seed; and
	// where each started (that, or where a reset of the states put it) and when the first seed was, for what this run
	// learned (`learned`)
	prior: Option<Prior>,
	cal_start: [Est; NQ],
	first_t: f64,
	gt: [f64; 2], // gravity's tilt from the map's z (rad, about x and y): g = GRAV * normalize(gt[0], gt[1], 1)
	gvec: [f64; 3], // ... as the vector the propagation subtracts (gravity)
	// the IMU-only motion: the world acceleration (orientation from the state, bias and gravity out) integrated twice
	// from the start and never corrected, so that the difference between two times is what the IMU says the device did
	// between them. A velocity error in it grows with the time, but the difference over a seed window (0.06 s) only
	// carries the bias over that window
	dr_p: [f64; 3],
	dr_v: [f64; 3],
	a_now: [f64; 3], // the IMU point's acceleration at the state's time (world, gravity and bias out; while coasting the decay)
	// the hand-over look-ahead (lookahead): the accelerometer's samples newer than the state carry it ho_lead ahead
	ho_lead: f64,
	ho_dp: [f64; 3],
	ho_dv: [f64; 3],
	ho_a: [f64; 3],
	ho_w: [f64; 3],
	ho_al: [f64; 3], // ... and the angular acceleration (world, rad/s^2) from the gyro's newest samples (al_b)
	P: [f64; NS * NS],
	step_qv: f64,
	step_qth: [f64; 9],
	step_qob: f64,
	step_kob: f64, // the process noise of the last IMU step (propagate_P), for the history
	a_lp: [f64; 3],
	moving_t: f64, // the accelerometer's motion: the world acceleration (gravity and bias out) without its noise
	// and vibration (DR_LP), and the last time it was over DR_MOTION
	stale_qv: f64,
	stale_qth: f64, // the last stale sweep: the share of the velocity's / orientation's variance its interval's noise was (TRACK_SWEEP)
	facing_cos: f64,

	// state history, one entry per IMU sample after its propagation
	hist: Box<[Hist; HISTN]>,
	hist_n: i32,
	hist_i: i32,

	// IMU
	acc_hist: [[f64; 3]; VIBN],
	acc_hist_n: i32,
	acc_hist_i: i32,
	gyr_hist: [[f64; 3]; STUCKN],
	gyr_mag: [[f64; 3]; STUCKN], // ... and per axis the rate of the other two (hypot), once a sample for the stuck test
	gyr_hist_n: i32,
	gyr_hist_i: i32,
	gyro_stuck: bool, // a gyro axis has stopped reporting (STUCK_*)
	stuck_ax: i32,    // ... which (-1 = none): latched until that axis reads differently again
	stuck_val: f64,
	last_acc: [f64; 3],
	acc_use: [f64; 3],
	last_gyr: [f64; 3],
	last_imu_t: f64,
	al_b: [f64; 3], // the body rate's change since the previous sample (rad/s^2, body frame): the hand-over's angular acceleration
	gyr_prev_clip: bool, // the previous sample's gyro was at full scale
	vib: f64,
	last_stale: f64,
	// the light's residual for Diag: each sweep the update takes, its median |residual| (TRACK_SWEEP's, mrad) smoothed 0.8 /
	// 0.2, started over after 0.5 s without one; and that sweep's time
	res_mrad: f64,
	res_t: f64,
	light_lag: f64,
	last_light_t: f64,
	rest_since: f64,
	// the hold along the line the light does not see (HOLD_*): since when its conditions have held, whether it holds
	// now, and that line as last found (where blind_line's iteration starts), with the position's sd along it
	hold_since: f64,
	holding: bool,
	hold_line: [f64; 3],
	hold_sd: f64,
	pin_logged: bool, // the pinned levels logged once (pinned)
	coasting: bool,
	clipped: bool,
	gyro_clipped: bool,
	want_reseed: bool,
	lost_warm: bool, // lost_warm: lost at the light's return, the orientation still good
	imu_count: i32,  // the samples so far, for the lines' and the divergence test's cadence (an int: it wraps)
	had_imu: bool,   // a sample has been taken

	// pending sweeps
	sw: Box<[Sweep; SWMAX]>,
	sw_n: i32,

	last_sweep_t: [f64; NLH], // per station, for the layout-scale gating
	light_since: [f64; NLH],  // per station: when its light came back (no sweep for JOIN_GAP s before)
	rot_at_light: [f64; NLH], // per station: rot_cum when its light came back
	rot_cum: f64,             // the device's total rotation so far (rad, |gyro - bias| integrated)
	last_used_t: [f64; NLH],  // per station: its last sweep with an accepted hit

	// the seed solve
	seedbuf: Box<[SeedSweep; SEEDN]>,
	seed_n: i32,
	seed_i: i32,
	seed_try_t: f64,            // time of the last attempt (rate limit)
	bc: Option<BarycentricSvd>, // the cold start's solver, set up at the first cold start

	// divergence test
	recent: Box<[Recent; RECENT]>,
	recent_n: i32,
	recent_i: i32,
	disagree_t: f64, // the last station-disagreement restart (rate limit)
	tie_t: f64,      // the last joint solve tried on a tie in the vote (rate limit)

	// for the output layer (RETURN_DARK), read by nothing in the tracker
	corr_p: [f64; 3],     // the state's corrections so far: positions summed (IMU point, world)
	corr_q: Quat,         // ... orientations composed on the world side
	corr_rep_p: [f64; 3], // ... as they stood at the last report (the TRACK_RETURN lines' per-report corrections)
	corr_rep_q: Quat,
	sweep_in_t: f64, // the newest sweep the update was given (any station, used or not)
	ret_armed: bool, // a dark spell has ended: the next correction begins a return
	ret_t: f64,      // when the latest return began (state time), -1e9 before the first
	ret_gap: f64,    // ... and how long the device was dark before it

	stats: Stats,
	work: Option<Box<Work>>, // the work arrays, while no call has them out
}

// the vibration measure saturates here (m/s^2 rms of the sample-to-sample accelerometer difference): a rattling tracker
// reads 1-2, a shaken controller 2-6, and the shake is real motion. Below the cap the adaptation is needed as it is
// (with half of it a tracker spun at 14 rad/s is lost), above it the hands run away
const VIB_CAP: f64 = 2.0;
// an accelerometer axis beyond this (g) is at full scale (the trackers and controllers clip at 8 g): the sample gets
// track-clip-sd as its noise
const CLIP_G: f64 = 7.8;
// ... and on a controller the excess beyond the reading is taken as this much (m/s^2, on the side the axis is pinned
// to), not as zero. A punch's stop runs 10-25 g for 10-40 ms; integrating the 8 g that is read, the state runs on past
// the stop and the light pulls it back 20-40 ms later (the position error grows with the square of the time), a lurch
// and a rubber band. Only on readings truly at the end of the range (PIN_TOL). Estimating the excess from the stretch's
// shape counted it twice with the light, and a state for it diverged. Trackers keep 0: one held at full scale by a
// spin's centripetal pull got slightly worse with it (findings.md section 6)
const CLIP_EXCESS_CONTROLLER: f64 = 40.;
// ... and the velocity process noise while at full scale on a controller (m/s^2 per step; the trackers keep
// track-clip-sd, 30). With the excess modelled, 30 lets the light's few sweeps after a hard stop set the velocity in
// kicks of 1-3 m/s: the state runs on past the stop, the sweeps are then rejected coherently and the state restarts
// with a jump. 15 is the base noise at those accelerations. Noise that grows with the length of the stretch, or a wider
// hit gate after full scale, did not reduce the restarts
const CLIP_SD_CONTROLLER: f64 = 15.;
// a gyro axis at full scale (2000 dps on the trackers and controllers; a tracker shaken by hand saturates both
// sensors): orientation process noise (rad/s per step) while it lasts, and the accelerometer does not drive the
// velocity through an orientation that is not known. 5 and 100 replay alike, 20 once ran away
const GYRO_CLIP_SD: f64 = 5.0;
// at rest (the zero-velocity update's condition) the device does not turn either: what its gyro reads then is its bias.
// A zero-rate measurement of every sample with this sd (rad/s): the rest test only says the device turns slower than
// track-zupt-gyro, so that is the sd (at 0.004, the gyro's own noise, a resting hand's slow real turning went into the
// bias). Without it nothing but the light holds the bias, and in the dark a resting device's orientation runs off at
// the bias error (0.3-0.6 deg/s): after a long blackout the warm restart, pinned to that orientation, starts metres
// away
const ZARU_SD: f64 = 0.02;
// ... and on a tracker the accelerometer stays off the velocity for this long after the last saturated sample (the
// orientation error it left takes the light a few sweeps to fix). Not on a controller: a punch saturates the gyro for a
// sample or two at its fastest, with the orientation a few degrees off at most, while the accelerometer reads 8-11 g;
// holding it off for 20 ms there loses ~2 m/s that the light then has to put back
const GYRO_CLIP_HOLD: f64 = 0.02;
// a gyro axis that has stopped reporting (see integrate_imu): sample-to-sample change within this (rad/s,
// 4 LSB at 2000 dps) for STUCKN samples while the other two axes exceed STUCK_FAST (rad/s) and turn; released once the
// axis moves by STUCK_RELEASE from the value it sat at
const STUCK_FLAT: f64 = 0.004;
const STUCK_FAST: f64 = 3.0;
const STUCK_RELEASE: f64 = 0.02;
// the light's orientation offset (state IOB): the orientation the hits imply differs from the device's by the sensor
// calibration of the face a station sees (positions to ~0.3 mm over the 57 mm a tracker spans = ~5 mrad), so the hits
// see the orientation turned by an offset that the gyro and gravity do not see. Without the state a white hit model
// lets the light's offset into the tilt, and gravity leaks into the velocity along the line a single station cannot
// see. The offset belongs to the face: it changes as the device turns (another set of sensors, other incidence angles;
// Markov over OB_RAD of turn) and slowly as it moves across the station's view (OB_TAU). A device that does not turn
// keeps its offset, so the light's orientation does not average below it: with a time constant of 0.5 s a foot lit by
// one station on 3 sensors believed its tilt to 0.8 mrad while it was off ~10 mrad, and drifted along the station's
// line
const OB_SD: f64 = 0.005;
const OB_RAD: f64 = 0.3;
const OB_TAU: f64 = 5.0;
// the gyro's scale error (state-free): orientation process noise proportional to the body rate and about the rotation
// axis, Q += (GYRO_SCALE * dt)^2 * w w^T in the world frame. The trackers carry no gyro scale calibration (gyro_scale 1
// 1 1), and the light's orientation corrections against the gyro's rotation say they read ~1 % over (0.2-1.8 % per
// device and axis). About the rotation axis only: noise across it lets the light's tilt offset in while a device twists
// at 1-2 rad/s; without the term a tracker spun at 10-35 rad/s loses its orientation on one station
const GYRO_SCALE: f64 = 0.05;
// the accelerometer's samples are fresher than the gyro's: the reading that belongs to the gyro's time t (the state's;
// track-hit-shift puts the light there, which the orientation fits) is the one of t - this. Integrated as reported, the
// accelerometer runs the velocity a few ms ahead of the light and every sweep pulls it back: the sweeps' velocity kicks
// follow the IMU's acceleration with a coherence that grows in proportion to the frequency (a timing error), while the
// gyro's orientation kicks show no timing at all. Each class's value is where that coherence is lowest: 7 ms on the
// trackers, 2.5 ms on the controllers. The newest samples are then this much ahead of the state: the hand-over uses
// them (lookahead)
const ACC_LEAD_TRACKER: f64 = 0.007;
// the hand-over's angular acceleration (al_b: the body rate's change between two gyro samples, for the runtime's Q
// line: its lever term alpha x r in the head point's acceleration, and the driver's orientation carry). Without it a
// hand's handed-over velocity lags its own path by ~8 ms wherever |alpha x r| is over 30 m/s^2. To judge it, compare
// the velocity with the path's at the time it is handed over for: against the path's velocity at the state's time, any
// carried acceleration counts as error
const HANDOVER_ALPHA: bool = true;
const ACC_LEAD_CONTROLLER: f64 = 0.0025;
// a sweep whose light came late (the radio held it up; the base is the state's history) measures the device where it
// was then: the update takes the velocity's part of that (H's velocity columns, update, while two or more stations
// light the device) and the history is corrected back to its own times with it (STALE_HIST). Off, the hits are taken as
// measuring the current position, and a shaken controller whose radio holds its light 15-18 ms is restarted again and
// again. Only with two or more stations: with one, it costs the controllers accuracy. The history part needs the
// velocity columns: without them the state runs off
const STALE_H: bool = true;
const STALE_HIST: bool = true;
// the late light's covariance (update): the process noise between a sweep's time and now taken out of what it measured
const STALE_Q: bool = true;
// dead reckoning through a blackout (propagate_state): past track-gap-coast a device coasts (the accelerometer no
// longer drives the velocity, which decays) unless the accelerometer still sees it moving: the world acceleration
// (gravity and bias out; its sample noise and vibration out with a DR_LP time constant) above DR_MOTION within the last
// DR_WIN, 5-10x the 0.05-0.1 m/s^2 that a tilt or bias error gives a still device; then it is followed up to DR_MAX of
// darkness. A body keeps moving through a blackout (a waist that crouches and stands up in the dark is frozen at the
// bottom by coasting), and a still device's dead reckoning only drifts. Its limit: a device that went dark out of
// sparse, grazing light dead-reckons further off than coasting would leave it, and the state's velocity sd does not
// tell the two apart. (A mean square over the window keeps a hard stop from a second before in it: a resting tracker
// dead-reckons on.)
const DR_MOTION: f64 = 0.5;
const DR_WIN: f64 = 0.25;
const DR_LP: f64 = 0.05;
const DR_MAX: f64 = 1.5;
// The hold along the line the light does not see. One station with a few hits a sweep places a device across its line
// of sight and hardly along it, so along that line the velocity is the IMU's alone, and what the accelerometer gets
// wrong there is a bias, not noise (a tilt off by a few mrad leaks 0.05-0.15 m/s^2 of gravity): a planted foot hidden
// from all but one station creeps 3-4 cm/s along the line until the zero-velocity update comes, and a foot that rocks
// over its ankle at 0.03-0.2 rad/s does not get that one (findings.md section 4). The rest test cannot simply be
// loosened to reach it: with the gyro's bound at 0.1 rad/s a slowly moving waist is pinned and then snaps. What tells
// the two apart is the light: where it sees, it decides. So the velocity is held at zero (track-zupt-sd) only along the
// one line the light leaves open, and only while everything that can be seen says rest: the IMU nearly quiet (turning
// under HOLD_GYRO, no vibration, the accelerometer's motion under DR_MOTION, as in a blackout), lit within HOLD_LIT,
// and the velocity across the line, which the light does measure, under HOLD_ACROSS (a line through 0.15 s of light
// tells still, p90 0.7 cm/s, from slow, p10 1.6 cm/s). It starts once that has lasted track-zupt-time with the position
// along the line known no better than HOLD_SD (two or more stations leave about 1 mm; one station 11-65 mm), and lasts
// until one of the others fails (held, the line's own sd shrinks: a threshold on it there makes the hold start and stop
// thousands of times in a session)
const HOLD_GYRO: f64 = 0.1;
const HOLD_LIT: f64 = 0.1;
const HOLD_ACROSS: f64 = 0.01;
const HOLD_SD: f64 = 0.01;
// stations disagree (update): a sweep is coherently rejected when every hit is out and all residuals are one-sided
// beyond this (mrad)
const DISAGREE_MRAD: f64 = 5.0;
// the largest position sd (m, chi-square scaled) a seed solve may claim and still be taken (seed_solve): twice the
// warm solve's 1 m prior
const SEED_MAX_SD: f64 = 2.0;
// Reflections. A station's light off glass or a mirror comes from somewhere else: it fits no rigid pose together with
// the direct light of the other stations, and a solve or an update that takes it in lands between the device and its
// reflection. Whole reflected sweeps (every hit tens to hundreds of mrad off) the gate already keeps out; what gets
// through is a tie's trial fitting both, and single hits a mrad or two off where a sensor sees the glass beside the
// station (findings.md section 3). A tie's trial (seed_solve) whose hits fit worse than REFL_SOLVE_FIT (their median
// |residual|, rad) is tried again without each of its stations; where it fits REFL_SOLVE_GAIN times better without one,
// and within REFL_SOLVE_FIT, the trial's pose was a compromise with that station's light (0.6-5.6 mrad with all,
// 0.02-0.18 without). Only for a trial: on every solve it fires at the 0.5-1.5 mrad by which a room's calibration
// leaves the stations apart, where the station left out is no reflection, its light then fails the solve's
// confirmation, and the solves go round
const REFL_SOLVE_FIT: f64 = 0.5e-3;
const REFL_SOLVE_GAIN: f64 = 4.0;
// ... and in the update, while two or more stations light the device, each hit the gate let through is tested against
// the others of its sweep: its leave-one-out residual under the whole innovation covariance (the state's uncertainty,
// the sweep's common error, the hits' own), z = (S^-1 r)_a / sqrt((S^-1)_aa). The worst beyond REFL_HIT_Z sds is left
// out, again until none is, while REFL_HIT_MIN or more hits are in (loo_worst); and the hits left out stay out only if
// the ones left then fit their noise (chi-square, 99 %). A reflected hit is one or two off in a sweep that otherwise
// fits; in fast motion, or with the state off, every hit is off more than its noise says and the worst one is no
// outlier (without that condition a shaken hand gets worse). Not with one station: the depth along its line of sight is
// measured only through the apparent size, and a hit at the edge of the device that disagrees with the others is as
// often the state's depth error as the hit's
const REFL_HIT_Z: f64 = 3.0;
const REFL_HIT_MIN: usize = 4;
// a device's calibration states beyond these are not a device's (reset_P): rad/s, m/s^2. The real gyro biases are
// 0.002-0.03 rad/s, the accelerometer's 0.02-0.2 m/s^2
const BG_MAX: f64 = 0.2;
const BA_MAX: f64 = 2.0;
/// What a device's calibration states carried between sessions are held to (learned.rs): the factory priors the tracker
/// starts them with (track-p0-*) and the bounds beyond which a value is not a device's (a start's reset: BG_MAX,
/// BA_MAX; the layout scale's clamp)
pub fn learned_limits(s: &Settings) -> Limits {
	Limits {
		factory: [s.p0_bg, s.p0_bg, s.p0_bg, s.p0_ba, s.p0_ba, s.p0_ba, s.p0_ls],
		bound: [BG_MAX, BG_MAX, BG_MAX, BA_MAX, BA_MAX, BA_MAX, s.ls_max],
	}
}
// the device's clock going back by more than this (s) is the device restarting (integrate_imu)
const CLOCK_BACK: f64 = 0.5;
// an accelerometer reading at full scale, with the excess the device gets beyond it (CLIP_EXCESS_CONTROLLER). Only a
// reading at the end of the range is pinned: the calibrated value of the raw count's limit (+32767 / -32768), within
// PIN_TOL. Readings between CLIP_G and that are real (the edges of a stretch at full scale) and get no excess. Without
// the device's raw scale the test is CLIP_G
const PIN_TOL: f64 = 0.01; // g

// first light (track-hit-join-sd): a station counts as returning after this long without a sweep from it, and its extra
// sweep-common error fades with this time constant (the first-light excess is gone by 0.25 s)
const JOIN_GAP: f64 = 0.3;
const JOIN_TAU: f64 = 0.1;
// ... and with the angle the device has turned since (deg): the edge of a station's view, where the sensors are
// shadowed and lit at grazing incidence (the hit noise grows from 70 to 80+ deg), is a band about this wide, which a
// swaying device crosses in a second and a spinning one in milliseconds. Time alone holds the caution too long in fast
// motion
const JOIN_DEG: i32 = 10;
// ... and it applies only while another station's hits were accepted within this long (several of its sweeps)
const JOIN_HOLD: f64 = 0.1;

// The light's return, for the output layer (docs/sally_driver.md, "The output layer"; nothing in the tracker reads it):
// the light is back after a dark spell when no sweep at all reached the update for at least this long (s). The device
// was hidden, not rejected: a state the light disagrees with gets its sweeps all along, and its restart is a
// correction like any other. The return begins with the first correction after the spell (the coasted state taking
// the light, or the restart when it does not fit) and the hand-over says how long ago that was; which spells and how
// much of what follows the driver shows blended is the output layer's choice
const RETURN_DARK: f64 = 0.1;
// ... and the recordings carry each report's correction for this long after a return began (TRACK_RETURN lines, verbose 2)
const RETURN_RECORD: f64 = 0.6;

// the layout scale learns only from sweeps that fit the state to their noise (update; fits_noise)
const LS_FIT: bool = true;
// ... and a station plane shows the device's size with an error of its own (its calibration: tilt, gibbous, curve), the
// same sweep after sweep, in the noise of every sweep the scale is measured in (update). The sizes that a room's
// station planes show sit about +-1 % apart (sd 0.47 %), the two planes of one station as far apart as two stations.
// Taken as independent hit noise, the plane with the most hits and the closest station decides the size: a station
// under a metre away takes the scale to its clamp in 0.4 s. 0 = off
const LS_STATION_SD: f64 = 0.005;

/// The hit of `sel` that disagrees most with the others of its sweep, and by how much (REFL_HIT_Z): each one's
/// leave-one-out residual under the innovation covariance the update takes them with (H P H^T, each hit's own
/// variance, the common part, the layout scale's station term sb2 along its column), z_a = (S^-1 r)_a /
/// sqrt((S^-1)_aa): the hit's residual against what the others predict for it, in sds of that prediction; and the
/// hits' chi-square r' S^-1 r (less z_a^2 without hit a). `ph`, `s`, `w`: work arrays (HMAX * NS, HMAX * HMAX each).
/// None where S is not positive definite
#[allow(clippy::too_many_arguments)]
fn loo_worst(
	H: &[f64], P: &[f64], var: &[f64], common: f64, sb2: f64, r: &[f64], sel: &[usize], ph: &mut [f64], s: &mut [f64],
	w: &mut [f64],
) -> Option<(usize, f64, f64)> {
	let m = sel.len();
	for (a, &k) in sel.iter().enumerate() {
		let h = &H[k * NS..k * NS + NS];
		for i in 0..NS {
			let mut acc = 0.0;
			for j in 0..NS {
				if h[j] != 0.0 {
					acc += P[i * NS + j] * h[j];
				}
			}
			ph[a * NS + i] = acc;
		}
	}
	for a in 0..m {
		let ha = &H[sel[a] * NS..sel[a] * NS + NS];
		for b in 0..m {
			let mut acc = 0.0;
			for i in 0..NS {
				acc += ha[i] * ph[b * NS + i];
			}
			s[a * m + b] =
				acc + (if a == b { var[sel[a]] } else { 0.0 }) + common + sb2 * ha[ILS] * H[sel[b] * NS + ILS];
		}
	}
	w[..m * m].fill(0.0);
	for a in 0..m {
		w[a * m + a] = 1.0;
	}
	if !chol_solve(&mut s[..m * m], m, &mut w[..m * m], m) {
		return None;
	}
	let mut worst: Option<(usize, f64)> = None;
	let mut chi2 = 0.0;
	for a in 0..m {
		let mut wr = 0.0;
		for b in 0..m {
			wr += w[a * m + b] * r[sel[b]];
		}
		chi2 += r[sel[a]] * wr;
		let z = (wr / w[a * m + a].max(1e-30).sqrt()).abs();
		if worst.is_none_or(|x| z > x.1) {
			worst = Some((a, z));
		}
	}
	worst.map(|(a, z)| (a, z, chi2))
}

/// The hits' residuals `r` fit their noise: each hit's own variance `var` and a part `common` all of them share. The
/// chi-square r' R^-1 r for R = diag(var) + common 11' (Sherman-Morrison: the scatter about the weighted mean against
/// the hits' own noise, plus the mean against its own variance) within the 99 % point of chi-square with n degrees of
/// freedom (Wilson-Hilferty)
fn fits_noise(r: &[f64], var: &[f64], common: f64) -> bool {
	let n = r.len();
	if n == 0 {
		return false;
	}
	let (mut sw, mut swr) = (0.0, 0.0);
	for k in 0..n {
		sw += 1.0 / var[k];
		swr += r[k] / var[k];
	}
	let mean = swr / sw;
	let mut q = mean * mean / (1.0 / sw + common);
	for k in 0..n {
		q += (r[k] - mean) * (r[k] - mean) / var[k];
	}
	q <= chi2_99(n as f64)
}

/// The 99 % point of chi-square with `dof` degrees of freedom (Wilson-Hilferty)
fn chi2_99(dof: f64) -> f64 {
	let a = 2.0 / (9.0 * dof);
	dof * (1.0 - a + 2.326_347_874 * a.sqrt()).powi(3)
}

// ---- small quaternion / matrix helpers (w x y z quaternions, rotation = q v q*)
fn q2R(q: &Quat) -> [f64; 9] {
	let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
	[
		1.0 - 2.0 * (y * y + z * z),
		2.0 * (x * y - w * z),
		2.0 * (x * z + w * y),
		2.0 * (x * y + w * z),
		1.0 - 2.0 * (x * x + z * z),
		2.0 * (y * z - w * x),
		2.0 * (x * z - w * y),
		2.0 * (y * z + w * x),
		1.0 - 2.0 * (x * x + y * y),
	]
}
fn Rv(R: &[f64; 9], v: &[f64; 3]) -> [f64; 3] {
	let a = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
	let b = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
	let c = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
	[a, b, c]
}
fn qmul(a: &Quat, b: &Quat) -> Quat {
	let (w1, x1, y1, z1, w2, x2, y2, z2) = (a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]);
	[
		w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
		w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
		w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
		w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
	]
}
fn qexp(v: &[f64; 3]) -> Quat {
	let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
	if n < 1e-12 {
		return [1.0, 0.0, 0.0, 0.0];
	}
	// sin and cos of one argument: LLVM makes them one sincos call, the same bits as sin and cos (docs/checking.md)
	let s = (n / 2.0).sin() / n;
	[(n / 2.0).cos(), v[0] * s, v[1] * s, v[2] * s]
}
fn qnorm(q: &mut Quat) {
	let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
	if n > 0.0 {
		for x in q.iter_mut() {
			*x /= n;
		}
	}
}
// q <- exp(dth) * q  (rotation error on the world side)
fn qrot_left(q: &mut Quat, dth: &[f64; 3]) {
	let e = qexp(dth);
	qrot_left_by(q, &e);
}
// ... with exp(dth) given: one correction to many orientations
fn qrot_left_by(q: &mut Quat, e: &Quat) {
	*q = qmul(e, q);
	qnorm(q);
}
// q*: the inverse of a unit quaternion
fn qconj(q: &Quat) -> Quat {
	[q[0], -q[1], -q[2], -q[3]]
}
// the rotation vector of a unit quaternion (rad), the shorter way round
fn qlog(q: &Quat) -> [f64; 3] {
	let (w, v) = if q[0] < 0.0 { (-q[0], [-q[1], -q[2], -q[3]]) } else { (q[0], [q[1], q[2], q[3]]) };
	let s = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
	if s < 1e-12 {
		return [2.0 * v[0], 2.0 * v[1], 2.0 * v[2]];
	}
	let k = 2.0 * s.atan2(w) / s;
	[v[0] * k, v[1] * k, v[2] * k]
}
// q <- q * exp(w dt)  (body rate)
fn qrot_right(q: &mut Quat, w: &[f64; 3], dt: f64) {
	let v = [w[0] * dt, w[1] * dt, w[2] * dt];
	let e = qexp(&v);
	*q = qmul(q, &e);
	qnorm(q);
}
fn v3(x: &[f64]) -> [f64; 3] {
	[x[0], x[1], x[2]]
}
fn finite(x: &[f64]) -> bool {
	x.iter().all(|v| v.is_finite())
}
// finite() over a whole matrix, every entry looked at (no early exit, so the loop runs on vectors)
fn all_finite(x: &[f64]) -> bool {
	!x.iter().fold(false, |bad, v| bad | !v.is_finite())
}

// C = A B (NS x NS, row-major): each element's sum in k order from 0, as `s = 0; for k: s += A[i][k] B[k][j]`, with
// the loops in the order i, k, j so that the inner one runs along the rows of B and C (the same operations on each
// element, so the same bits)
fn mul_ikj(A: &[f64; NS * NS], B: &[f64; NS * NS], C: &mut [f64; NS * NS]) {
	for i in 0..NS {
		let c = &mut C[i * NS..i * NS + NS];
		c.fill(0.0);
		for k in 0..NS {
			let a = A[i * NS + k];
			let b = &B[k * NS..k * NS + NS];
			for j in 0..NS {
				c[j] += a * b[j];
			}
		}
	}
}
// Where each row of a matrix is not zero, in order: propagate_P's F (I + dt A) has about 2.4 such entries a row
fn row_nz(B: &[f64; NS * NS], nz: &mut [[u8; NS]; NS], nn: &mut [usize; NS]) {
	for j in 0..NS {
		nn[j] = 0;
		for k in 0..NS {
			if B[j * NS + k] != 0.0 {
				nz[j][nn[j]] = k as u8;
				nn[j] += 1;
			}
		}
	}
}
// C = A B as mul_ikj, with A's rows (row_nz) only where they are not zero, for a B whose every entry is finite: a term
// 0 * b is then +0 or -0, and a sum begun at +0 is never -0 (rounding to nearest, x + -x is +0), so leaving it out
// changes no bit
fn mul_sparse_a(A: &[f64; NS * NS], nz: &[[u8; NS]; NS], nn: &[usize; NS], B: &[f64; NS * NS], C: &mut [f64; NS * NS]) {
	for i in 0..NS {
		let c = &mut C[i * NS..i * NS + NS];
		c.fill(0.0);
		for &k in &nz[i][..nn[i]] {
			let a = A[i * NS + k as usize];
			let b = &B[k as usize * NS..k as usize * NS + NS];
			for j in 0..NS {
				c[j] += a * b[j];
			}
		}
	}
}
// C = A B^T, each element's sum in k order from 0 as mul_ikj(A, B^T) has it, with B's rows (row_nz) only where they
// are not zero, for an A whose every entry is finite (as mul_sparse_a: the same bits)
fn mul_abt_sparse_b(
	A: &[f64; NS * NS], B: &[f64; NS * NS], nz: &[[u8; NS]; NS], nn: &[usize; NS], C: &mut [f64; NS * NS],
) {
	for i in 0..NS {
		let a = &A[i * NS..i * NS + NS];
		for j in 0..NS {
			let b = &B[j * NS..j * NS + NS];
			let mut s = 0.0;
			for &k in &nz[j][..nn[j]] {
				s += a[k as usize] * b[k as usize];
			}
			C[i * NS + j] = s;
		}
	}
}
// At = A^T (NS x NS): for a product with a transposed factor (A B^T is mul_ikj(A, B^T))
fn transpose(A: &[f64; NS * NS], At: &mut [f64; NS * NS]) {
	for i in 0..NS {
		for j in 0..NS {
			At[j * NS + i] = A[i * NS + j];
		}
	}
}

// Cholesky solve of S X = B for symmetric positive definite S (n x n), B n x m, in place; false if not SPD
fn chol_solve(S: &mut [f64], n: usize, B: &mut [f64], m: usize) -> bool {
	for j in 0..n {
		let mut d = S[j * n + j];
		for k in 0..j {
			d -= S[j * n + k] * S[j * n + k];
		}
		if d <= 0.0 {
			return false;
		}
		d = d.sqrt();
		S[j * n + j] = d;
		for i in j + 1..n {
			let mut s = S[i * n + j];
			for k in 0..j {
				s -= S[i * n + k] * S[j * n + k];
			}
			S[i * n + j] = s / d;
		}
	}
	// L y = B, then L^T x = y
	for col in 0..m {
		for i in 0..n {
			let mut s = B[i * m + col];
			for k in 0..i {
				s -= S[i * n + k] * B[k * m + col];
			}
			B[i * m + col] = s / S[i * n + i];
		}
		for i in (0..n).rev() {
			let mut s = B[i * m + col];
			for k in i + 1..n {
				s -= S[k * n + i] * B[k * m + col];
			}
			B[i * m + col] = s / S[i * n + i];
		}
	}
	true
}

// whether the symmetric n x n matrix A is positive definite on the states it carries (Cholesky on a copy; a state
// with zero variance, such as a fixed gravity tilt, is left out)
fn is_pd(A: &[f64], n: usize, L: &mut [f64; NS * NS]) -> bool {
	// (L's entries are each written before they are read: no clearing)
	let mut on = [false; NS];
	if n > NS {
		return false;
	}
	for j in 0..n {
		on[j] = A[j * n + j] != 0.0;
	}
	for j in 0..n {
		if !on[j] {
			continue;
		}
		let mut d = A[j * n + j];
		for k in 0..j {
			if on[k] {
				d -= L[j * n + k] * L[j * n + k];
			}
		}
		if !(d > 0.0) {
			return false;
		}
		d = d.sqrt();
		L[j * n + j] = d;
		for i in j + 1..n {
			if !on[i] {
				continue;
			}
			let mut s = A[i * n + j];
			for k in 0..j {
				if on[k] {
					s -= L[i * n + k] * L[j * n + k];
				}
			}
			L[i * n + j] = s / d;
		}
	}
	true
}

impl Tracker {
	fn acc_lead<H: Host>(&self, h: &H) -> f64 {
		if h.object_type() == SURVIVE_OBJECT_TYPE_CONTROLLER {
			ACC_LEAD_CONTROLLER
		} else {
			ACC_LEAD_TRACKER
		}
	}
	fn is_controller<H: Host>(&self, h: &H) -> bool {
		h.object_type() == SURVIVE_OBJECT_TYPE_CONTROLLER
	}
	fn pinned<H: Host>(&self, h: &H, i: usize, a: f64) -> bool {
		if !(h.raw_acc_scale() > 0.0) || h.acc_scale(i) == 0.0 {
			return a.abs() > CLIP_G * GRAV;
		}
		let mut hi = (32767.0 * h.raw_acc_scale() - h.acc_bias(i)) * h.acc_scale(i) * GRAV;
		let mut lo = (-32768.0 * h.raw_acc_scale() - h.acc_bias(i)) * h.acc_scale(i) * GRAV;
		if hi < lo {
			std::mem::swap(&mut hi, &mut lo);
		}
		a >= hi - PIN_TOL * GRAV || a <= lo + PIN_TOL * GRAV
	}
	fn clip_add<H: Host>(&self, h: &H, raw: &[f64; 3], am: &mut [f64; 3]) {
		if !self.is_controller(h) {
			return;
		}
		for i in 0..3 {
			if raw[i].abs() > CLIP_G * GRAV && self.pinned(h, i, raw[i]) {
				am[i] += if raw[i] > 0.0 { CLIP_EXCESS_CONTROLLER } else { -CLIP_EXCESS_CONTROLLER };
			}
		}
	}

	// ---- lifecycle
	/// A tracker with its settings
	pub fn create(s: Settings) -> Box<Tracker> {
		let mut c = Box::new(Tracker {
			s,
			gyro_clip_t: 0.0,
			gt0x: 0.0,
			gt0y: 0.0,
			accq_t: [0.0; ACCQN],
			accq: [[0.0; 3]; ACCQN],
			accq_clip: [false; ACCQN],
			accq_n: 0,
			accq_i: 0,
			a_prev: [0.0; 3],
			g_prev: [0.0; 3],
			have_prev: false,
			ob: [0.0; 3],
			blink_k: 0,
			blink_lh: 0,
			seeded: false,
			ever_seeded: false,
			confirmed: false,
			seed_t: 0.0,
			t: 0.0,
			p: [0.0; 3],
			v: [0.0; 3],
			bg: [0.0; 3],
			ba: [0.0; 3],
			q: [0.0; 4],
			ls: 0.0,
			prior: None,
			cal_start: [Est { mean: 0.0, var: 0.0 }; NQ],
			first_t: 0.0,
			gt: [0.0; 2],
			gvec: [0.0; 3],
			dr_p: [0.0; 3],
			dr_v: [0.0; 3],
			a_now: [0.0; 3],
			ho_lead: 0.0,
			ho_dp: [0.0; 3],
			ho_dv: [0.0; 3],
			ho_a: [0.0; 3],
			ho_w: [0.0; 3],
			ho_al: [0.0; 3],
			P: [0.0; NS * NS],
			step_qv: 0.0,
			step_qth: [0.0; 9],
			step_qob: 0.0,
			step_kob: 0.0,
			a_lp: [0.0; 3],
			moving_t: 0.0,
			stale_qv: 0.0,
			stale_qth: 0.0,
			facing_cos: 0.0,
			hist: boxed(Hist::default()),
			hist_n: 0,
			hist_i: 0,
			acc_hist: [[0.0; 3]; VIBN],
			acc_hist_n: 0,
			acc_hist_i: 0,
			gyr_hist: [[0.0; 3]; STUCKN],
			gyr_mag: [[0.0; 3]; STUCKN],
			gyr_hist_n: 0,
			gyr_hist_i: 0,
			gyro_stuck: false,
			stuck_ax: 0,
			stuck_val: 0.0,
			last_acc: [0.0; 3],
			acc_use: [0.0; 3],
			last_gyr: [0.0; 3],
			last_imu_t: 0.0,
			al_b: [0.0; 3],
			gyr_prev_clip: false,
			vib: 0.0,
			last_stale: 0.0,
			res_mrad: -1.0,
			res_t: f64::NEG_INFINITY,
			light_lag: 0.0,
			last_light_t: 0.0,
			rest_since: 0.0,
			hold_since: 0.0,
			holding: false,
			hold_line: [0.0, 0.0, 1.0],
			hold_sd: 0.0,
			pin_logged: false,
			coasting: false,
			clipped: false,
			gyro_clipped: false,
			want_reseed: false,
			lost_warm: false,
			imu_count: 0,
			had_imu: false,
			sw: boxed(Sweep::ZERO),
			sw_n: 0,
			last_sweep_t: [0.0; NLH],
			light_since: [0.0; NLH],
			rot_at_light: [0.0; NLH],
			rot_cum: 0.0,
			last_used_t: [0.0; NLH],
			seedbuf: boxed(SeedSweep::ZERO),
			seed_n: 0,
			seed_i: 0,
			seed_try_t: 0.0,
			bc: None,
			recent: boxed(Recent::default()),
			recent_n: 0,
			recent_i: 0,
			disagree_t: 0.0,
			tie_t: 0.0,
			corr_p: [0.0; 3],
			corr_q: [1.0, 0.0, 0.0, 0.0],
			corr_rep_p: [0.0; 3],
			corr_rep_q: [1.0, 0.0, 0.0, 0.0],
			sweep_in_t: 0.0,
			ret_armed: false,
			ret_t: -1e9,
			ret_gap: 0.0,
			stats: Stats::default(),
			work: Some(Work::boxed()),
		});
		c.facing_cos = (c.s.facing_deg * LINMATHPI / 180.0).cos();
		c.gt0x = c.s.gt_mag * (c.s.gt_az * LINMATHPI / 180.0).cos();
		c.gt0y = c.s.gt_mag * (c.s.gt_az * LINMATHPI / 180.0).sin();
		c.gt[0] = c.gt0x;
		c.gt[1] = c.gt0y;
		c.gravity();
		c.q[0] = 1.0; // the orientation integrates the gyro from the start; the seed solve supplies the world-side correction
		c.stuck_ax = -1;
		c.blink_k = -1;
		c
	}

	/// The tracker's copy of its settings replaced (the runtime's live set). What create derived from the settings (the
	/// facing cosine, the gravity tilt) is kept
	pub fn set_settings(&mut self, s: Settings) {
		self.s = s;
	}

	/// The device's calibration states (gyro bias, accelerometer bias, layout scale) start from `p` instead of zero
	/// and the factory prior (learned.rs: what earlier sessions measured), each state that has one, its variance no
	/// wider than the factory prior's. Only before the first seed (false after: the states are this run's by then).
	/// The gyro's integration before the first seed runs on the prior's bias too. A layout scale is taken only with
	/// the scale state on (track-layout-sd > 0)
	pub fn set_prior(&mut self, p: Prior) -> bool {
		if self.ever_seeded {
			return false;
		}
		self.prior = Some(p);
		self.take_prior_means();
		true
	}

	/// The calibration states set to the prior's means, where it has them (the rest at zero)
	fn take_prior_means(&mut self) {
		self.bg = [0.0; 3];
		self.ba = [0.0; 3];
		self.ls = 0.0;
		let Some(p) = self.prior else { return };
		for i in 0..3 {
			if let Some(e) = p.est[i] {
				self.bg[i] = e.mean;
			}
			if let Some(e) = p.est[3 + i] {
				self.ba[i] = e.mean;
			}
		}
		if let (Some(e), true) = (p.est[6], self.s.sig_ls > 0.0) {
			self.ls = e.mean;
		}
	}

	/// The calibration states now: mean and variance (the covariance's diagonal), in learned.rs's order
	fn cal_now(&self) -> [Est; NQ] {
		let mut c = [Est { mean: 0.0, var: 0.0 }; NQ];
		for i in 0..3 {
			c[i] = Est { mean: self.bg[i], var: self.P[(IBG + i) * NS + IBG + i] };
			c[3 + i] = Est { mean: self.ba[i], var: self.P[(IBA + i) * NS + IBA + i] };
		}
		c[6] = Est { mean: self.ls, var: self.P[ILS * NS + ILS] };
		c
	}

	/// What this run learned of the device: where its calibration states started and where they are now (None before
	/// the first seed)
	pub fn learned(&self) -> Option<Learned> {
		if !self.ever_seeded {
			return None;
		}
		Some(Learned { start: self.cal_start, now: self.cal_now(), tracked_s: self.t - self.first_t })
	}

	/// Whether the tracker has a state to report
	pub fn reporting(&self) -> bool {
		self.seeded
	}

	/// The hand-over look-ahead, None while there is no state
	pub fn handover(&self) -> Option<Handover> {
		if !self.seeded {
			return None;
		}
		Some(Handover {
			lead: self.ho_lead,
			p_imu: self.p,
			dp: self.ho_dp,
			dv: self.ho_dv,
			a: self.ho_a,
			w: self.ho_w,
			corr_p: self.corr_p,
			corr_q: self.corr_q,
			ret_age: if self.ret_t > -1e8 { self.t - self.ret_t } else { -1.0 },
			ret_gap: self.ret_gap,
		})
	}

	/// A correction of the state, for the output layer: the IMU point moved `dp` and the orientation turned by `e` on the
	/// world side, outside the motion
	fn corrected(&mut self, dp: &[f64; 3], e: &Quat) {
		for i in 0..3 {
			self.corr_p[i] += dp[i];
		}
		self.corr_q = qmul(e, &self.corr_q);
		qnorm(&mut self.corr_q);
	}

	/// ... by the light (an update, a restart): the first such correction after a dark spell begins the light's return
	/// (RETURN_DARK). A zero-velocity update is no light and begins nothing
	fn corrected_by_light(&mut self, dp: &[f64; 3], e: &Quat) {
		self.corrected(dp, e);
		if self.ret_armed {
			self.ret_armed = false;
			self.ret_t = self.t;
		}
	}

	/// The hand-over's angular acceleration, None while there is no state
	pub fn handover_alpha(&self) -> Option<[f64; 3]> {
		if !self.seeded {
			return None;
		}
		Some(self.ho_al)
	}

	/// The device's tracking state, for the diagnosis and the protocol
	pub fn diag_state<H: Host>(&self, h: &H) -> Diag {
		let mut d = Diag { state: SURVIVE_DIAG_STATE_NONE, light_age: -1.0, pos_sd: -1.0, lh_mask: 0, res_mrad: -1.0 };
		if !self.ever_seeded {
			return d;
		}
		if self.t - self.res_t <= 0.5 {
			d.res_mrad = self.res_mrad;
		}
		let age = self.t - self.last_light_t;
		d.light_age = if age < 0.0 { 0.0 } else { age };
		d.pos_sd =
			c_fmax(0., self.P[IP * NS + IP] + self.P[(IP + 1) * NS + IP + 1] + self.P[(IP + 2) * NS + IP + 2]).sqrt();
		let mut i = 0;
		while i < NLH && (i as i32) < h.active_lighthouses() {
			if self.last_sweep_t[i] > self.t - 0.5 {
				d.lh_mask |= 1u32 << i;
			}
			i += 1;
		}
		d.state = if !self.seeded || self.want_reseed {
			SURVIVE_DIAG_STATE_REACQUIRING
		} else if self.coasting {
			SURVIVE_DIAG_STATE_COASTING
		} else if age > 0.1 {
			SURVIVE_DIAG_STATE_DARK
		} else {
			SURVIVE_DIAG_STATE_TRACKING
		};
		d
	}

	/// The session's counts, as log lines
	pub fn print_stats<H: Host>(&self, h: &mut H) {
		if self.stats.hits == 0 && !self.ever_seeded {
			return;
		}
		let st = &self.stats;
		let ur = st.used.wrapping_add(st.rejected);
		h.sv_info(
			c"%s track: %llu hits in %llu sweeps, used %llu rejected %llu (%.1f %%), facing drops %llu, sweeps dropped %llu, stale %llu, seeds %d (%d restarts, %d losses at the light's return; %d solve attempts, %d refused, %d disagreements), clipped samples %llu, zupts %llu, reports %llu; biases gyro %+.4f %+.4f %+.4f rad/s accel %+.3f %+.3f %+.3f m/s^2 layout %+.3f %%",
			(
				h.codename(),
				st.hits,
				st.sweeps,
				st.used,
				st.rejected,
				100. * st.rejected as f64 / ur.wrapping_add((ur == 0) as u64) as f64,
				st.facing_drops,
				st.sweeps_dropped,
				st.stale,
				st.seeds,
				st.reseeds,
				st.gap_reseeds,
				st.seed_tries,
				st.seed_refused,
				st.disagreements,
				st.clipped,
				st.zupts,
				st.reports,
				self.bg[0],
				self.bg[1],
				self.bg[2],
				self.ba[0],
				self.ba[1],
				self.ba[2],
				self.ls * 100.0,
			),
		);
		if self.s.gt_sd > 0.0 || self.s.gt_mag != 0.0 {
			h.sv_info(
				c"%s track: gravity's tilt from the map's z: %+.2f %+.2f mrad about x / y (%.2f mrad toward %.0f deg; sd %.2f %.2f; started at %+.2f %+.2f)",
				(
					h.codename(),
					self.gt[0] * 1e3,
					self.gt[1] * 1e3,
					self.gt[0].hypot(self.gt[1]) * 1e3,
					self.gt[1].atan2(self.gt[0]) * 180. / LINMATHPI,
					c_fmax(self.P[IGT * NS + IGT], 0.0).sqrt() * 1e3,
					c_fmax(self.P[(IGT + 1) * NS + IGT + 1], 0.0).sqrt() * 1e3,
					self.gt0x * 1e3,
					self.gt0y * 1e3,
				),
			);
		}
		if STALE_Q {
			h.sv_info(
				c"%s track: late sweeps with their interval's noise out: all %llu, half %llu, a quarter %llu, none %llu",
				(h.codename(), st.stale_q[0], st.stale_q[1], st.stale_q[2], st.stale_q[3]),
			);
		}
	}

	// gravity in the map frame from the tilt states (track-gravity-tilt-*)
	fn gravity(&mut self) {
		let g = [self.gt[0], self.gt[1], 1.0];
		let n = (g[0] * g[0] + g[1] * g[1] + 1.0).sqrt();
		for i in 0..3 {
			self.gvec[i] = GRAV * g[i] / n;
		}
	}

	// ---- the seed: the tracker's own start and recovery, a pose solve on its own recent sweeps
	fn reset_P<H: Host>(&mut self, h: &mut H, C9: &[f64; SEEDX * SEEDX]) {
		// a seed re-initialises the pose and the velocity, not the device's calibration: the bias and layout-scale
		// variances the filter has reached are kept (a re-seed every few seconds with one station put the gyro bias back
		// to its 0.01 rad/s prior each time, and with one station the light cannot tell a bias from a depth: the bias was
		// yanked by 0.03 rad/s within a second, the attitude with it, gravity leaked into the velocity).
		// C9: the seed solve's covariance of (position, orientation, velocity). Only its diagonal is taken: the solve's
		// correlations (position against velocity over a 60 ms window are near +-1) make the first accepted sweeps
		// after a restart drag the velocity to 8 m/s with a 100 mm position correction, and the floors (track-p0-*)
		// apply where the solve is more confident than that
		let mut keep = [0.0; NS];
		for i in 0..NS {
			keep[i] = if self.ever_seeded && self.P[i * NS + i].is_finite() && self.P[i * NS + i] > 0.0 {
				self.P[i * NS + i]
			} else {
				1e9
			};
		}
		// the first seed: the calibration states' variances what earlier sessions learned, where they did (set_prior;
		// the factory prior still the widest)
		if let (false, Some(p)) = (self.ever_seeded, self.prior) {
			for i in 0..3 {
				if let Some(e) = p.est[i] {
					keep[IBG + i] = e.var;
				}
				if let Some(e) = p.est[3 + i] {
					keep[IBA + i] = e.var;
				}
			}
			if let Some(e) = p.est[6] {
				keep[ILS] = e.var;
			}
		}
		let mut cal_reset = !self.ever_seeded;
		// calibration states that are not finite or beyond any device's start over from their priors: kept as they were
		// (a gyro bias of 1e51 rad/s after a device's clock went back), every start after that runs away within 3 ms
		{
			let mut bad = !self.ls.is_finite() || self.ls.abs() > 2.0 * self.s.ls_max;
			for i in 0..3 {
				bad |= !self.bg[i].is_finite()
					|| self.bg[i].abs() > BG_MAX
					|| !self.ba[i].is_finite()
					|| self.ba[i].abs() > BA_MAX;
			}
			if bad {
				h.sv_warn(
					c"%s track: calibration states out of bounds at a start (gyro bias %g %g %g rad/s, accelerometer %g %g %g m/s^2, layout %g): reset",
					(
						h.codename(),
						self.bg[0],
						self.bg[1],
						self.bg[2],
						self.ba[0],
						self.ba[1],
						self.ba[2],
						self.ls,
					),
				);
				self.bg = [0.0; 3];
				self.ba = [0.0; 3];
				self.ls = 0.0;
				for i in 0..3 {
					keep[IBG + i] = 1e9;
					keep[IBA + i] = 1e9;
				}
				keep[ILS] = 1e9;
				cal_reset = true;
			}
		}
		// the IMU-only motion (dr_p, dr_v; a_lp, the dead reckoning's motion test) and the turn count (rot_cum, the
		// first-light fade's) integrate from the first sample and nothing else starts them over: one that is not finite
		// stayed so for the rest of the run (a NaN accelerometer sample: every warm solve's shape term NaN, no dead
		// reckoning in the dark; a NaN gyro sample: the first-light term NaN whenever another station holds the device, and
		// with it every update). Only their differences over the seed window matter, and a start empties the history and the
		// seed buffer that hold their older values
		if !(finite(&self.dr_p) && finite(&self.dr_v) && finite(&self.a_lp)) {
			self.dr_p = [0.0; 3];
			self.dr_v = [0.0; 3];
			self.a_lp = [0.0; 3];
		}
		if !(self.rot_cum.is_finite() && finite(&self.rot_at_light)) {
			self.rot_cum = 0.0;
			self.rot_at_light = [0.0; NLH];
		}
		self.P = [0.0; NS * NS];
		let map: [usize; SEEDX] = [IP, IP + 1, IP + 2, IQ, IQ + 1, IQ + 2, IV, IV + 1, IV + 2];
		let floor9: [f64; SEEDX] = [
			self.s.p0_pos,
			self.s.p0_pos,
			self.s.p0_pos,
			self.s.p0_rot,
			self.s.p0_rot,
			self.s.p0_rot,
			self.s.p0_vel,
			self.s.p0_vel,
			self.s.p0_vel,
		];
		for a in 0..SEEDX {
			self.P[map[a] * NS + map[a]] = c_fmax(floor9[a], C9[a * SEEDX + a]);
		}
		for i in 0..3 {
			self.P[(IBG + i) * NS + IBG + i] = c_fmin(self.s.p0_bg, keep[IBG + i]);
			self.P[(IBA + i) * NS + IBA + i] = c_fmin(self.s.p0_ba, keep[IBA + i]);
		}
		self.P[ILS * NS + ILS] = if self.s.sig_ls > 0.0 { c_fmin(self.s.p0_ls, keep[ILS]) } else { 0.0 };
		for i in 0..2 {
			self.P[(IGT + i) * NS + IGT + i] = c_fmin(self.s.gt_sd * self.s.gt_sd, keep[IGT + i]);
		}
		// the light's orientation offset belongs to the face seen now: a seed solved the orientation from the light, so it
		// starts at zero with its stationary variance
		for i in 0..3 {
			self.ob[i] = 0.0;
			self.P[(IOB + i) * NS + IOB + i] = OB_SD * OB_SD;
		}
		// where the calibration states start, for what this run learns of them (`learned`)
		if cal_reset {
			self.cal_start = self.cal_now();
		}
		if !self.ever_seeded {
			self.first_t = self.t;
		}
	}

	// the first guess: the device's pose in the world from one station's hits in both planes (barycentric SVD, the same
	// solver the old pipeline starts from). None when the solve is degenerate. sws: the window's sweeps (indices into
	// seedbuf), newest first
	fn bc_pose<H: Host>(&mut self, h: &mut H, lh: usize, sws: &[usize]) -> Option<SurvivePose> {
		if self.bc.is_none() {
			let layout = (0..h.sensor_ct()).map(|i| h.sensor_location(i)).collect();
			self.bc = Some(BarycentricSvd::new(layout));
		}
		let lh_version = h.lh_version();
		let cal = h.bsd_fcal(lh as i32);
		let bc = self.bc.as_mut().expect("set up above");
		bc.reset();
		// the newest angle per sensor and plane in the window (the sweeps are newest first)
		let mut have = [[false; 2]; HMAX];
		for &s in sws {
			let sw = &self.seedbuf[s];
			if sw.lh != lh as i32 {
				continue;
			}
			for k in 0..sw.n as usize {
				if sw.sensor[k] >= HMAX as i32 || have[ix(sw.sensor[k])][ix(sw.plane)] {
					continue;
				}
				have[ix(sw.sensor[k])][ix(sw.plane)] = true;
				bc.add(sw.sensor[k] as usize, sw.plane, sw.angle[k] + cal[ix(sw.plane)].phase);
			}
		}
		if bc.meas_cnt() < 8 {
			return None;
		}
		let mut r = [[0.0; 3]; 3];
		let mut t = [0.0; 3];
		let err = bc.compute_pose(lh_version, &mut r, &mut t);
		if err < 0.0 || err > 1.0 || norm3d(&t) < 0.25 || norm3d(&t) > 25.0 {
			return None;
		}
		// lh2obj: the solver's R, t describe the camera in the object's frame once transposed; the station's z points out
		// of its back (a 180 deg turn about y), exactly as poser_barycentric_svd does it
		let Rt = [r[0][0], r[1][0], r[2][0], r[0][1], r[1][1], r[2][1], r[0][2], r[1][2], r[2][2]];
		let mut lh2obj = SurvivePose::default();
		let tt = Rv(&Rt, &t);
		for i in 0..3 {
			lh2obj.pos[i] = -tt[i];
		}
		{
			let tmp = quatfrommatrix33(&Rt);
			let rt: Quat = [0.0, 0.0, 1.0, 0.0];
			lh2obj.rot = quatrotateabout(&tmp, &rt);
		}
		let lh2world = h.lighthouse_position(lh as i32);
		let obj2lh = invert_pose(&lh2obj);
		let obj2world = apply_pose_to_pose(&lh2world, &obj2lh);
		for i in 0..3 {
			if !obj2world.pos[i].is_finite() {
				return None;
			}
		}
		if quatiszero(&obj2world.rot) {
			None
		} else {
			Some(obj2world)
		}
	}

	// solve position, orientation and velocity from the sweeps of the last seed-window seconds: a Gauss-Newton fit of
	// the hits with the tracker's own measurement model (every hit at its own time, the gyro-integrated orientation of each
	// sweep corrected by one world-side rotation, constant velocity over the window, a 3 m/s prior on the velocity so a
	// one-station window, which does not observe the velocity along the line of sight, stays well posed).
	// Cold (the start, or after a divergence, when nothing of the state can be trusted): the first guess is a
	// barycentric-SVD pose from the station with the most hits in both planes, and 16 hits are needed for the fit to
	// mean anything with 9 unknowns. Warm (the light back after a blackout, the orientation carried through it by the
	// gyro to within what the filter's covariance says): the coasted orientation is the prior, which leaves the position
	// a linear problem (each hit puts the sensor on a plane through the station, no mirror pose possible) solved from
	// the first few hits, so the restart comes with the first sweeps back rather than half a second later.
	// On success the state and its covariance (the solve's, scaled by its reduced chi-square) are set and true is returned
	// tie_a / tie_b (station bitmasks, both 0 for an ordinary start or restart): a tie in the vote (update). The solve is
	// then a trial: it is taken only if its pose uses at least 3 hits of a station in each mask (the stations whose planes
	// accept the state and those whose planes coherently reject it), and otherwise leaves everything as it was.
	// A trial whose light fits poorly is tried without each of its stations, and where it fits without one the trial's
	// pose was a compromise with that station's light (REFL_SOLVE_FIT): the pose without it is taken if both sides are
	// still in it, and otherwise the tie stands
	fn seed_solve<H: Host>(&mut self, h: &mut H, now: f64, warm: bool, tie_a: c_uint, tie_b: c_uint) -> bool {
		let trial = tie_a != 0 && tie_b != 0;
		let all = if trial { self.seed_fit(h, now, warm, tie_a, tie_b, 0, true) } else { None };
		if let Some(all) = all {
			if all.stations.count_ones() >= 2 && all.med > REFL_SOLVE_FIT {
				let mut best: Option<(u32, SeedFit)> = None;
				for lh in 0..NLH as u32 {
					if (all.stations >> lh) & 1 == 0 {
						continue;
					}
					if let Some(f) = self.seed_fit(h, now, warm, tie_a, tie_b, 1 << lh, true) {
						if f.fit_ok
							&& f.med < all.med / REFL_SOLVE_GAIN
							&& f.med < REFL_SOLVE_FIT
							&& best.is_none_or(|b| f.med < b.1.med)
						{
							best = Some((lh, f));
						}
					}
				}
				if let Some((lh, f)) = best {
					if self.s.verbose >= 1 {
						h.sv_info(
							c"%s track at %.3f s: the light fits %.2f mrad, without ch %d %.2f mrad: a reflection, %s",
							(
								h.codename(),
								now,
								all.med * 1e3,
								h.bsd_mode(lh as i32),
								f.med * 1e3,
								if f.ok { c"solved without it" } else { c"the tie stands" },
							),
						);
					}
					if !f.ok {
						return false;
					}
					return self.seed_fit(h, now, warm, tie_a, tie_b, 1 << lh, false).is_some_and(|f| f.applied);
				}
			}
		}
		self.seed_fit(h, now, warm, tie_a, tie_b, 0, false).is_some_and(|f| f.applied)
	}

	/// The seed solve (seed_solve) without the stations in `exclude` (bsd indices), and with `dry` nothing changed (the
	/// state, the counts, the log): what it found, None where it gave up before a fit
	#[allow(clippy::too_many_arguments)]
	fn seed_fit<H: Host>(
		&mut self, h: &mut H, now: f64, mut warm: bool, tie_a: c_uint, tie_b: c_uint, exclude: u32, dry: bool,
	) -> Option<SeedFit> {
		let trial = tie_a != 0 && tie_b != 0;
		if !h.sensor_locations_set() || !h.sensor_normals_set() || self.seed_n == 0 {
			return None;
		}
		if !trial && !dry {
			self.seed_try_t = now;
			inci(&mut self.stats.seed_tries);
		}
		// the window, newest first (indices into seedbuf)
		let mut sws = [0usize; SEEDN];
		let mut nsw: usize = 0;
		let mut hits: i32 = 0;
		let mut per_lh = [[0i32; 2]; NLH];
		let mut t_ref = 0.0;
		for j in 0..self.seed_n {
			let k = ix((self.seed_i - 1 - j + 2 * SEEDN_I) % SEEDN_I);
			let sw = &self.seedbuf[k];
			if sw.t < now - self.s.seed_window {
				break;
			}
			if sw.lh < 0 || sw.lh >= h.active_lighthouses() || sw.lh >= NLH as i32 || !h.bsd_position_set(sw.lh) {
				continue;
			}
			if self.s.only_mask != 0 && (self.s.only_mask.wrapping_shr(h.bsd_mode(sw.lh) as u32) & 1) == 0 {
				continue;
			}
			if self.s.probe_ch >= 0 && h.bsd_mode(sw.lh) == self.s.probe_ch {
				continue;
			}
			if (exclude >> sw.lh) & 1 != 0 {
				continue;
			}
			if nsw == 0 {
				t_ref = sw.t;
			}
			sws[nsw] = k;
			nsw += 1;
			hits += sw.n;
			per_lh[ix(sw.lh)][ix(sw.plane)] += sw.n;
		}
		// the orientation prior of a warm restart: the filter's own covariance of it, if it is still small
		let mut Qinv = [0.0; 9];
		if warm {
			let mut Pq = [0.0; 9];
			for a in 0..3 {
				for b in 0..3 {
					Pq[a * 3 + b] = self.P[(IQ + a) * NS + IQ + b];
				}
			}
			let mut I3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
			warm = Pq[0] + Pq[4] + Pq[8] < 0.04 && chol_solve(&mut Pq, 3, &mut I3, 3);
			if warm {
				Qinv = I3;
			}
		}
		let min_used = if warm { 10 } else { 16 };
		if hits < min_used {
			return None;
		}
		// the motion inside the window: the constant-velocity model would put the acceleration's effect into the
		// residuals (at 1 g 18 mm at the edge of a 60 ms window; a punch reversal is 10-15 g, 50-60 mm, and such solves
		// "fit" at 2.5-3.9 mrad while landing 100-400 mm off). The IMU measures that motion: with the orientation
		// trusted (a warm restart) each sweep's position is p + v dt + the IMU-integrated displacement since t_ref with
		// the velocity at t_ref taken out, so p and v stay the unknowns and the shape between the sweeps is the
		// accelerometer's. Cold: the orientation is not trusted, the gravity would be removed wrongly, constant
		// velocity stands
		let (ref_dp, ref_dv) = (self.seedbuf[sws[0]].dp, self.seedbuf[sws[0]].dv); // ref = sws[0] (hits > 0: nsw > 0)
		let shape = warm;
		// SHAPE_P / SHAPE_V (a 0 otherwise: the double 0, added)
		let shape_p = |sw: &SeedSweep, dt: f64, i: usize| -> f64 {
			if shape {
				sw.dp[i] - ref_dp[i] - ref_dv[i] * dt
			} else {
				0.0
			}
		};
		let shape_v = |sw: &SeedSweep, i: usize| -> f64 {
			if shape {
				sw.dv[i] - ref_dv[i]
			} else {
				0.0
			}
		};
		let mut p = [0.0; 3];
		let mut v = [0.0; 3];
		let mut qc: Quat; // the correction: q(t) = qc * q_dr(t)
		if warm {
			// the coasted orientation stands; the position from the planes the hits put the sensors on (linear, with the
			// velocity along, 3 m/s prior): eq . (W (p + v dt + R s) + t) = 0 per hit, eq the plane in the station frame
			qc = [1.0, 0.0, 0.0, 0.0];
			let mut A = [0.0; 36];
			let mut rhs = [0.0; 6];
			for a in 3..6 {
				A[a * 6 + a] = 1.0 / 9.;
			}
			// with one station the planes give no depth (that is in the device's apparent size, a second-order effect the
			// Gauss-Newton fit below sees): a prior of 1 m around the coasted position keeps the solution on the right
			// line of sight (without it a one-station window solved to kilometres)
			for a in 0..3 {
				A[a * 6 + a] += 1.0;
				rhs[a] += self.p[a];
			}
			let mut rows = 0;
			for &s in &sws[..nsw] {
				let sw = &self.seedbuf[s];
				let lh2world = h.lighthouse_position(sw.lh);
				let world2lh = invert_pose(&lh2world);
				let W = q2R(&world2lh.rot);
				let Rk = q2R(&sw.q);
				let cal = h.bsd_fcal(sw.lh);
				// o = 0: the state point is the IMU: the sensor locations are already relative to it
				for k in 0..sw.n as usize {
					let eq = bc_fill_m(h.lh_version(), sw.plane, sw.angle[k] + cal[ix(sw.plane)].phase);
					let n = [
						W[0] * eq[0] + W[3] * eq[1] + W[6] * eq[2], // W^T eq
						W[1] * eq[0] + W[4] * eq[1] + W[7] * eq[2],
						W[2] * eq[0] + W[5] * eq[1] + W[8] * eq[2],
					];
					let pt = h.sensor_location(sw.sensor[k]);
					let sl = [pt[0] - 0.0, pt[1] - 0.0, pt[2] - 0.0];
					let sr = Rv(&Rk, &sl);
					let dt = sw.ht[k] + self.s.hit_shift - t_ref;
					let row = [n[0], n[1], n[2], n[0] * dt, n[1] * dt, n[2] * dt];
					let shp = [shape_p(sw, dt, 0), shape_p(sw, dt, 1), shape_p(sw, dt, 2)];
					let b = -dot3d(&eq, &world2lh.pos) - dot3d(&n, &sr) - dot3d(&n, &shp);
					for a in 0..6 {
						rhs[a] += row[a] * b;
						for bb in 0..6 {
							A[a * 6 + bb] += row[a] * row[bb];
						}
					}
					rows += 1;
				}
			}
			if rows < 6 || !chol_solve(&mut A, 6, &mut rhs, 1) {
				return None;
			}
			for i in 0..3 {
				p[i] = rhs[i];
				v[i] = rhs[3 + i];
			}
			for i in 0..6 {
				if !rhs[i].is_finite() {
					if !trial && !dry {
						// a warm solve that gives no number (the IMU-only motion not finite) would give none at the
						// next attempt either: that one is cold, as after a refusal
						self.lost_warm = false;
					}
					return None;
				}
			}
			if norm3d(&p) > 20.0 {
				if trial || dry {
					return None;
				}
				inci(&mut self.stats.seed_refused);
				self.lost_warm = false;
				return None;
			}
		} else {
			// the first guess from the station with the most hits in both planes
			let mut best: i32 = -1;
			let mut best_n = 0;
			for lh in 0..NLH {
				let m = if per_lh[lh][0] < per_lh[lh][1] { per_lh[lh][0] } else { per_lh[lh][1] };
				if m >= 3 && per_lh[lh][0] + per_lh[lh][1] > best_n {
					best = lh as i32;
					best_n = per_lh[lh][0] + per_lh[lh][1];
				}
			}
			if best < 0 {
				return None;
			}
			let Some(guess) = self.bc_pose(h, best as usize, &sws[..nsw]) else {
				if !trial && !dry {
					inci(&mut self.stats.seed_refused);
				}
				return None;
			};
			// the state point and the world-side correction of the gyro-integrated orientation at t_ref
			let qinv = quatgetreciprocal(&self.seedbuf[sws[0]].q);
			qc = qmul(&guess.rot, &qinv);
			qnorm(&mut qc);
			p = guess.pos; // the IMU point: the state point
		}
		let mut nst = 0;
		for lh in 0..NLH {
			nst += (per_lh[lh][0] + per_lh[lh][1] > 0) as i32;
		}
		// the velocity needs time separation to be observed: two sweeps of the same station and plane at different
		// times give the motion along that plane's normal, and nothing else does (with few sweeps it absorbs whatever
		// the window's planes disagree about: one sweep of one plane family fits perfectly with 2.8 m/s and the depth
		// 160 mm off). So the velocity is solved only in the span of the normals of the plane families seen twice or
		// more, held at zero in the rest (with one station: the line of sight), and the filter learns the held part
		// from the sweeps that follow, starting from the prior
		// the held directions as a projector
		let vhold = {
			let mut M = [0.0; 9];
			let mut fam_n = [[0i32; 2]; NLH];
			for &s in &sws[..nsw] {
				let sw = &self.seedbuf[s];
				fam_n[ix(sw.lh)][ix(sw.plane)] += 1;
			}
			for &s in &sws[..nsw] {
				let sw = &self.seedbuf[s];
				if fam_n[ix(sw.lh)][ix(sw.plane)] < 2 {
					continue;
				}
				let lh2world = h.lighthouse_position(sw.lh);
				let eq =
					bc_fill_m(h.lh_version(), sw.plane, sw.angle[ix(sw.n / 2)] + h.bsd_fcal(sw.lh)[ix(sw.plane)].phase);
				let Rl = q2R(&lh2world.rot);
				let n = Rv(&Rl, &eq);
				let n = normalize3d(&n);
				for a in 0..3 {
					for bb in 0..3 {
						M[a * 3 + bb] += n[a] * n[bb] / fam_n[ix(sw.lh)][ix(sw.plane)] as f64;
					}
				}
			}
			// eigenvectors of M (Jacobi): a direction is observed when the repeated families' normals have weight >= 0.2 along it (two stations in different corners of the room see everything; one station nothing along its line of sight)
			let mut V = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
			for _sweep in 0..20 {
				let pq = [[0, 1], [0, 2], [1, 2]];
				for k in 0..3 {
					let (pi, qi) = (pq[k][0], pq[k][1]);
					if M[pi * 3 + qi].abs() < 1e-12 {
						continue;
					}
					let th = 0.5 * (2.0 * M[pi * 3 + qi]).atan2(M[qi * 3 + qi] - M[pi * 3 + pi]);
					let cs = th.cos();
					let sn = th.sin();
					for i in 0..3 {
						// rotate columns p, q of M and V
						let mp = M[i * 3 + pi];
						let mq = M[i * 3 + qi];
						M[i * 3 + pi] = cs * mp - sn * mq;
						M[i * 3 + qi] = sn * mp + cs * mq;
					}
					for i in 0..3 {
						let mp = M[pi * 3 + i];
						let mq = M[qi * 3 + i];
						M[pi * 3 + i] = cs * mp - sn * mq;
						M[qi * 3 + i] = sn * mp + cs * mq;
					}
					for i in 0..3 {
						let vp = V[i * 3 + pi];
						let vq = V[i * 3 + qi];
						V[i * 3 + pi] = cs * vp - sn * vq;
						V[i * 3 + qi] = sn * vp + cs * vq;
					}
				}
			}
			let mut vhold = [0.0; 9];
			for k in 0..3 {
				if M[k * 3 + k] < 0.2 {
					for a in 0..3 {
						for bb in 0..3 {
							vhold[a * 3 + bb] += V[a * 3 + k] * V[bb * 3 + k];
						}
					}
				}
			}
			vhold
		};
		// Gauss-Newton on x = (dp, dth, dv) with the velocity prior; hits weighted by their incidence-angle noise, the
		// back-facing and the far-off ones dropped after the first iterations
		let vprior = 3.;
		let mut N = [0.0; SEEDX * SEEDX]; // (and g, C9: set in each iteration)
		let mut used: i32 = 0;
		let mut nplanes: i32 = 0;
		let mut used_lh = [0i32; NLH];
		let mut med = 1.0;
		let mut chi2 = 1.0;
		let mut th_acc = [0.0, 0.0, 0.0]; // th_acc: the orientation correction accumulated so far (the warm prior is on it)
		let mut robust = false;
		const NAR: usize = if SEEDN * HMAX > 2048 { 2048 } else { SEEDN * HMAX };
		let mut ar = [0.0; NAR];
		let mut at = HitsAt::ZERO;
		for it in 0..12 {
			N = [0.0; SEEDX * SEEDX];
			let mut g = [0.0; SEEDX];
			for a in 0..3 {
				N[(6 + a) * SEEDX + 6 + a] = 1.0 / (vprior * vprior);
			}
			for a in 0..3 {
				N[a * SEEDX + a] += 1e-2; // a weak prior on position (10 m) and orientation (10 rad): conditioning only
				N[(3 + a) * SEEDX + 3 + a] += 1e-2;
			}
			for a in 0..3 {
				// the held velocity directions (sd 0.01 m/s around zero)
				for bb in 0..3 {
					N[(6 + a) * SEEDX + 6 + bb] += 1e4 * vhold[a * 3 + bb];
					g[6 + a] -= 1e4 * vhold[a * 3 + bb] * v[bb];
				}
			}
			if warm {
				// the orientation prior (the filter's covariance), the position prior (1 m around the coasted one)
				for a in 0..3 {
					for bb in 0..3 {
						N[(3 + a) * SEEDX + 3 + bb] += Qinv[a * 3 + bb];
						g[3 + a] -= Qinv[a * 3 + bb] * th_acc[bb];
					}
					N[a * SEEDX + a] += 1.0;
					g[a] += self.p[a] - p[a];
				}
			}
			let mut wrr = 0.0;
			let mut na: usize = 0;
			used = 0;
			used_lh = [0; NLH];
			let mut plane_seen = [false, false];
			for &s in &sws[..nsw] {
				let sw = &self.seedbuf[s];
				let n = sw.n as usize;
				let mut hs = Sweep { lh: sw.lh, plane: sw.plane, n: sw.n, ..Sweep::ZERO };
				hs.t[..n].copy_from_slice(&sw.ht[..n]);
				hs.angle[..n].copy_from_slice(&sw.angle[..n]);
				hs.sensor[..n].copy_from_slice(&sw.sensor[..n]);
				let mut b = Base::default();
				let dt = sw.t - t_ref;
				b.t = sw.t;
				for i in 0..3 {
					b.p[i] = p[i] + v[i] * dt + shape_p(sw, dt, i);
					b.v[i] = v[i] + shape_v(sw, i);
					b.w[i] = sw.w[i];
				}
				b.q = qmul(&qc, &sw.q);
				qnorm(&mut b.q);
				let R = q2R(&b.q);
				let lh2world = h.lighthouse_position(sw.lh);
				let world2lh = invert_pose(&lh2world);
				let bcal = h.bsd_fcal(sw.lh);
				let mut pred = [0.0; HMAX];
				let mut pw = [0.0; HMAX * 3];
				let mut pd = [0.0; HMAX];
				let mut H = [0.0; HMAX * SEEDX];
				self.hits_at(h, &hs, &b, &bcal, &mut at);
				self.predict(&hs, &b, &at, None, None, 0.0, &R, &world2lh, &bcal, &mut pred, Some(&mut pw));
				let eps = 1e-5;
				for cc in 0..3 {
					let mut d = [0.0, 0.0, 0.0];
					d[cc] = eps;
					self.predict(&hs, &b, &at, Some(&d), None, 0.0, &R, &world2lh, &bcal, &mut pd, None);
					for k in 0..n {
						H[k * SEEDX + cc] = (pd[k] - pred[k]) / eps;
						H[k * SEEDX + 6 + cc] = H[k * SEEDX + cc] * dt;
					}
					self.predict(&hs, &b, &at, None, Some(&d), 0.0, &R, &world2lh, &bcal, &mut pd, None);
					for k in 0..n {
						H[k * SEEDX + 3 + cc] = (pd[k] - pred[k]) / eps;
					}
				}
				for k in 0..n {
					let r = sw.angle[k] - pred[k];
					if !r.is_finite() {
						continue;
					}
					let nw = Rv(&R, &h.sensor_normal(sw.sensor[k]));
					let dir = sub3d(&lh2world.pos, &v3(&pw[3 * k..]));
					let dir = normalize3d(&dir);
					let cosi = dot3d(&nw, &dir);
					if robust {
						if cosi <= self.facing_cos {
							continue;
						}
						if r.abs() > c_fmax(self.s.gate_floor, 4.0 * med) {
							continue;
						}
					}
					let ang = c_fmax(-1., c_fmin(1., cosi)).acos() / (self.s.hit_knee * LINMATHPI / 180.);
					let f = 1.0 + ang * ang * ang * ang * ang * ang;
					let w = if robust {
						1.0 / (self.s.sig_hit * self.s.sig_hit * f * f + self.s.sig_common * self.s.sig_common)
					} else {
						1.0 / (1e-3 * 1e-3)
					};
					let h = &H[k * SEEDX..k * SEEDX + SEEDX];
					for a in 0..SEEDX {
						g[a] += w * h[a] * r;
						for bb in 0..SEEDX {
							N[a * SEEDX + bb] += w * h[a] * h[bb];
						}
					}
					wrr += w * r * r;
					if na < NAR {
						ar[na] = r.abs();
						na += 1;
					}
					used += 1;
					used_lh[ix(sw.lh)] += 1;
					plane_seen[ix(sw.plane)] = true;
				}
			}
			if used < min_used {
				return None;
			}
			nplanes = plane_seen[0] as i32 + plane_seen[1] as i32;
			chi2 = if robust && used > SEEDX as i32 { wrr / (used - SEEDX as i32) as f64 } else { 1.0 };
			insertion_sort(&mut ar[..na]);
			med = ar[na / 2];
			let mut dx = g;
			let mut C9 = N;
			if !chol_solve(&mut C9, SEEDX, &mut dx, 1) {
				return None;
			}
			for i in 0..3 {
				p[i] += dx[i];
				v[i] += dx[6 + i];
				th_acc[i] += dx[3 + i];
			}
			qrot_left(&mut qc, &v3(&dx[3..]));
			let mut step = 0.0;
			for i in 0..SEEDX {
				step += dx[i] * dx[i];
			}
			if it >= 2 {
				robust = true; // from the third iteration: the real noise model, the far-off and back-facing hits out
			}
			if robust && step < 1e-10 && it >= 4 {
				break;
			}
		}
		// the covariance of the solve: N^-1
		let mut C9 = N;
		let mut I9 = [0.0; SEEDX * SEEDX];
		for a in 0..SEEDX {
			I9[a * SEEDX + a] = 1.0;
		}
		if !chol_solve(&mut C9, SEEDX, &mut I9, SEEDX) {
			return None;
		}
		// the fit means something only with redundancy (a 14-hit cold solve with 9 unknowns once fit to 0.02 mrad)
		let mut ok = med < self.s.seed_fit
			&& used >= min_used
			&& used * 10 >= hits * 6
			&& nplanes == 2
			&& norm3d(&p) < 20.0
			&& norm3d(&v) < 20.0;
		for i in 0..3 {
			ok = ok && p[i].is_finite() && v[i].is_finite();
		}
		for i in 0..4 {
			ok = ok && qc[i].is_finite();
		}
		// a pose places the device: the solve's own position sd, scaled by its reduced chi-square as the filter takes
		// it (below), must be within SEED_MAX_SD. Hits that disagree among themselves (chi-square 70-100) and hardly
		// locate the device (sd 0.5-0.8 m, the warm prior's 1 m doing the work) are not a pose: such a warm restart
		// from one station lands metres off and runs away. Well-fitting solves from one station reach 0.6 m unscaled
		// (1.2 m scaled); a punching hand's chi-square of 30-250 comes with a few mm
		{
			let sd2 = (I9[0] + I9[SEEDX + 1] + I9[2 * SEEDX + 2]) * 4.0 * c_fmax(1., chi2);
			ok = ok && sd2 < SEED_MAX_SD * SEED_MAX_SD;
		}
		let fit_ok = ok;
		if trial {
			let mut ua = false;
			let mut ub = false;
			for lh in 0..NLH {
				if used_lh[lh] < 3 {
					continue;
				}
				ua = ua || ((tie_a >> lh) & 1) != 0;
				ub = ub || ((tie_b >> lh) & 1) != 0;
			}
			ok = ok && ua && ub;
		}
		let mut stations = 0u32;
		for lh in 0..NLH {
			if used_lh[lh] > 0 {
				stations |= 1 << lh;
			}
		}
		let found = SeedFit { ok, fit_ok, med, stations, applied: false };
		if dry {
			return Some(found);
		}
		if self.s.verbose >= 1 {
			h.sv_info(
				c"%s track %s at %.3f s: %s, %d of %d hits of %d sweeps fit to %.2f mrad (chi2 %.1f; %s, %d planes, %d stations, v held in %d of 3), p %.3f %.3f %.3f, |v| %.2f m/s, sd pos %.1f mm rot %.1f mrad vel %.2f m/s",
				(
					h.codename(),
					if trial {
						c"joint solve on the tie"
					} else if self.ever_seeded {
						if warm {
							c"warm restart"
						} else {
							c"cold restart"
						}
					} else {
						c"start"
					},
					now,
					if ok { c"SOLVED" } else { c"refused" },
					used,
					hits,
					nsw as i32,
					med * 1e3,
					chi2,
					if warm { c"orientation from the gyro" } else { c"first guess from one station" },
					nplanes,
					nst,
					(vhold[0] + vhold[4] + vhold[8] + 0.5) as i32,
					p[0],
					p[1],
					p[2],
					norm3d(&v),
					(I9[0] + I9[SEEDX + 1] + I9[2 * SEEDX + 2]).sqrt() * 1e3,
					(I9[3 * SEEDX + 3] + I9[4 * SEEDX + 4] + I9[5 * SEEDX + 5]).sqrt() * 1e3,
					(I9[6 * SEEDX + 6] + I9[7 * SEEDX + 7] + I9[8 * SEEDX + 8]).sqrt(),
				),
			);
		}
		if !ok && trial {
			return Some(found); // the tie stands: nothing changed
		}
		if !ok {
			inci(&mut self.stats.seed_refused);
			self.lost_warm = false; // the orientation was not as good as its covariance said (a long blackout, the gyro's scale over many turns): cold next
			return Some(found);
		}
		// the state now: the solved pose carried from t_ref to the current state time with the solved velocity, the
		// orientation correction applied to the gyro-integrated orientation of now
		let before = if self.seeded { Some((self.p, self.q)) } else { None };
		let dtn = self.t - t_ref;
		for i in 0..3 {
			self.p[i] = p[i] + v[i] * dtn + (if shape { self.dr_p[i] - ref_dp[i] - ref_dv[i] * dtn } else { 0.0 });
			self.v[i] = v[i] + (if shape { self.dr_v[i] - ref_dv[i] } else { 0.0 });
		}
		{
			let q = qmul(&qc, &self.q);
			self.q = q;
			qnorm(&mut self.q);
		}
		// for the output layer: a restart of a state that was reported is a correction (the jump from where the device
		// was shown); a start from nothing reported is not (a consumer sees the reports begin)
		match before {
			Some((p0, q0)) => {
				let dp = [self.p[0] - p0[0], self.p[1] - p0[1], self.p[2] - p0[2]];
				let e = qmul(&self.q, &qconj(&q0));
				self.corrected_by_light(&dp, &e);
			}
			None => {
				self.sweep_in_t = now;
				self.ret_armed = false;
			}
		}
		if !self.ever_seeded {
			// zero, or what earlier sessions learned (set_prior)
			self.take_prior_means();
		}
		// the solve's covariance scaled by its reduced chi-square (hits that do not agree with each other as well as the
		// noise model says: a thin window, a station the map has slightly wrong), and by 4: the constant-velocity model
		// over the window is not the whole truth
		for a in 0..SEEDX * SEEDX {
			I9[a] *= 4.0 * c_fmax(1., chi2);
		}
		for a in 0..3 {
			// the held velocity directions get the prior back, for the filter to learn them from the sweeps that follow
			I9[(6 + a) * SEEDX + 6 + a] += vprior * vprior * vhold[a * 3 + a];
		}
		self.reset_P(h, &I9);
		self.last_light_t = now;
		self.recent_n = 0;
		self.recent_i = 0;
		self.sw_n = 0;
		self.hist_n = 0;
		self.hist_i = 0;
		self.seed_n = 0;
		self.seed_i = 0;
		self.want_reseed = false;
		self.coasting = false;
		self.seeded = true;
		self.ever_seeded = true;
		self.confirmed = false; // a hypothesis until the next sweeps agree (confirm); reported meanwhile
		self.seed_t = now;
		inci(&mut self.stats.seeds);
		Some(SeedFit { applied: true, ..found })
	}

	// a seed solve is a hypothesis: it stands once the sweeps of the next 0.2 s (at least 30 hits, up to 1 s) have been
	// accepted at 60 % or more; otherwise the start was wrong (while a tracker is being strapped on, a 60 ms window can
	// fit to 0.8 mrad 140 mm off, and the filter then wanders for seconds before the divergence test fires) and it is
	// dropped for a new solve. It is reported meanwhile: a hole in the reports reads as a pause downstream
	fn confirm<H: Host>(&mut self, h: &mut H, t: f64) {
		if self.confirmed || t - self.seed_t < 0.2 {
			return;
		}
		let mut used: i32 = 0;
		let mut rej: i32 = 0;
		for k in 0..ix(self.recent_n) {
			if self.recent[k].t > self.seed_t {
				used = used.wrapping_add(self.recent[k].used);
				rej = rej.wrapping_add(self.recent[k].rej);
			}
		}
		if used + rej < 30 && t - self.seed_t < 1.0 {
			return;
		}
		if used as f64 >= 0.6 * (used + rej) as f64 {
			// (no hits at all in a second: not contradicted either)
			self.confirmed = true;
			if self.s.verbose >= 1 {
				h.sv_info(
					c"%s track start confirmed at %.3f s: %d of %d hits since the solve fit",
					(h.codename(), t, used, used + rej),
				);
			}
		} else {
			if self.s.verbose >= 1 {
				h.sv_info(
					c"%s track start dropped at %.3f s: only %d of %d hits since the solve fit",
					(h.codename(), t, used, used + rej),
				);
			}
			self.want_reseed = true;
			self.lost_warm = false;
			self.confirmed = true; // decided (the state is lost; the next solve starts a new hypothesis)
			inci(&mut self.stats.seed_refused);
		}
	}

	// a closed sweep goes into the seed buffer with the orientation and body rate the gyro integration had at its time
	fn seed_keep(&mut self, sw: &Sweep, sw_t: f64, b: &Base) {
		let k = &mut self.seedbuf[ix(self.seed_i)];
		let n = sw.n as usize;
		k.lh = sw.lh;
		k.plane = sw.plane;
		k.n = sw.n;
		k.t = sw_t;
		k.ht[..n].copy_from_slice(&sw.t[..n]);
		k.angle[..n].copy_from_slice(&sw.angle[..n]);
		k.sensor[..n].copy_from_slice(&sw.sensor[..n]);
		// the orientation at the sweep's time: the base entry's, carried by the body rate
		let wdt = [b.w[0] * (sw_t - b.t), b.w[1] * (sw_t - b.t), b.w[2] * (sw_t - b.t)];
		let e = qexp(&wdt);
		k.q = qmul(&b.q, &e);
		qnorm(&mut k.q);
		k.w = b.w;
		for i in 0..3 {
			k.dp[i] = b.dr_p[i] + b.dr_v[i] * (sw_t - b.t);
			k.dv[i] = b.dr_v[i];
		}
		self.seed_i = (self.seed_i + 1) % SEEDN_I;
		if self.seed_n < SEEDN_I {
			self.seed_n += 1;
		}
	}

	// ---- propagation (state only; P separately, once per IMU sample)
	fn propagate_state<H: Host>(&mut self, h: &mut H, dt: f64) {
		let R = q2R(&self.q);
		let mut am = [self.acc_use[0] - self.ba[0], self.acc_use[1] - self.ba[1], self.acc_use[2] - self.ba[2]];
		self.clip_add(h, &self.acc_use, &mut am);
		let mut aw: [f64; 3];
		let mut q_end: Quat = [0.0; 4];
		let trap = self.have_prev;
		if trap {
			// the step from the previous sample to this one: the mean rate turns the orientation, and the two accelerations
			// are each rotated by the orientation of their own time
			let mut wbar = [0.0; 3];
			let mut ap = [self.a_prev[0] - self.ba[0], self.a_prev[1] - self.ba[1], self.a_prev[2] - self.ba[2]];
			self.clip_add(h, &self.a_prev, &mut ap);
			for i in 0..3 {
				wbar[i] = 0.5 * (self.g_prev[i] + self.last_gyr[i]) - self.bg[i];
			}
			q_end = self.q;
			qrot_right(&mut q_end, &wbar, dt);
			let R1 = q2R(&q_end);
			let a0 = Rv(&R, &ap);
			let a1 = Rv(&R1, &am);
			aw = [0.0; 3];
			for i in 0..3 {
				aw[i] = 0.5 * (a0[i] + a1[i]);
			}
			self.a_now = a1; // the newest sample at the step's end orientation: the acceleration now
		} else {
			aw = Rv(&R, &am);
			self.a_now = aw;
		}
		for i in 0..3 {
			aw[i] -= self.gvec[i];
			self.a_now[i] -= self.gvec[i];
		}
		{
			let kl = (-dt / DR_LP).exp();
			for i in 0..3 {
				self.a_lp[i] = self.a_lp[i] * kl + aw[i] * (1.0 - kl);
			}
			if norm3d(&self.a_lp) > DR_MOTION {
				self.moving_t = self.t + dt;
			}
		}
		for i in 0..3 {
			self.dr_p[i] += self.dr_v[i] * dt + 0.5 * aw[i] * dt * dt;
			self.dr_v[i] += aw[i] * dt;
		}
		{
			let wr = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			let k = (-norm3d(&wr) * dt / OB_RAD - dt / OB_TAU).exp();
			for i in 0..3 {
				self.ob[i] *= k;
			}
		}
		let dark = self.t - self.last_light_t;
		let mut coast = self.s.gap_coast > 0.0 && dark > self.s.gap_coast;
		// ... unless the accelerometer still sees the body moving well beyond its own error (DR_MOTION), up to DR_MAX
		let vsd =
			c_fmax(0., (self.P[IV * NS + IV] + self.P[(IV + 1) * NS + IV + 1] + self.P[(IV + 2) * NS + IV + 2]) / 3.0)
				.sqrt();
		if coast && DR_MOTION > 0.0 && dark < DR_MAX && self.t - self.moving_t < DR_WIN {
			coast = false;
		}
		if self.s.verbose >= 2 && dark > 0.4 && dark < 2.0 && h.recording() && (self.imu_count % 10) == 0 {
			// TRACK_DARK t dark vsd |a_lp| coast: dead reckoning in the dark
			h.record(
				c"%s TRACK_DARK %.4f %.3f %.4f %.3f %d\r\n",
				(h.codename(), self.t, dark, vsd, norm3d(&self.a_lp), coast as i32),
			);
		}
		if self.gyro_clip_t > 0.0
			&& self.t - self.gyro_clip_t < (if self.is_controller(h) { 0. } else { GYRO_CLIP_HOLD })
		{
			coast = true; // the gyro was at full scale just now: the orientation, and so the accelerometer's direction in the world, is not known
		}
		if coast {
			// coasting through a blackout: the accelerometer no longer drives the velocity (dead reckoning on it runs
			// off by metres within seconds), the velocity decays
			let k = if self.s.gap_tau > 0.0 { (-dt / self.s.gap_tau).exp() } else { 1.0 };
			for i in 0..3 {
				self.p[i] += self.v[i] * dt;
				self.v[i] *= k;
			}
			self.coasting = true;
			for i in 0..3 {
				self.a_now[i] = -self.v[i] * (1.0 - k) / dt; // what the decay does to the velocity
			}
		} else {
			self.coasting = false;
			for i in 0..3 {
				self.p[i] += self.v[i] * dt + 0.5 * aw[i] * dt * dt;
				self.v[i] += aw[i] * dt;
			}
		}
		if trap {
			self.q = q_end;
		} else {
			let wm = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			qrot_right(&mut self.q, &wm, dt);
		}
	}

	fn propagate_P<H: Host>(&mut self, h: &mut H, dt: f64, R: &[f64; 9], am: &[f64; 3]) {
		let mut w = self.take_work();
		self.propagate_P_in(h, dt, R, am, &mut w);
		self.work = Some(w);
	}
	fn propagate_P_in<H: Host>(&mut self, h: &mut H, dt: f64, R: &[f64; 9], am: &[f64; 3], w: &mut Work) {
		// F = I + dt * A with A[p,v] = I, A[v,th] = -skew(R am), A[v,ba] = -R, A[th,bg] = -R (the identity is the work
		// array's, and every entry it does not hold is written here at every step)
		let Work { F, FP, Mt, .. } = w;
		let aw = Rv(R, am);
		for i in 0..3 {
			F[(IP + i) * NS + IV + i] = dt;
		}
		// -skew(aw) * dt
		F[(IV) * NS + IQ + 1] = aw[2] * dt;
		F[(IV) * NS + IQ + 2] = -aw[1] * dt;
		F[(IV + 1) * NS + IQ] = -aw[2] * dt;
		F[(IV + 1) * NS + IQ + 2] = aw[0] * dt;
		F[(IV + 2) * NS + IQ] = aw[1] * dt;
		F[(IV + 2) * NS + IQ + 1] = -aw[0] * dt;
		for i in 0..3 {
			for j in 0..3 {
				F[(IV + i) * NS + IBA + j] = -R[i * 3 + j] * dt;
				F[(IQ + i) * NS + IBG + j] = -R[i * 3 + j] * dt;
			}
		}
		// gravity's tilt (IGT): d v / d gt = -GRAV dt on the x / y axes (the tilt small)
		F[(IV) * NS + IGT] = -GRAV * dt;
		F[(IV + 1) * NS + IGT + 1] = -GRAV * dt;
		let kob; // the light's orientation offset: Markov over the turn and the time (OB_RAD, OB_TAU)
		{
			let wr = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			kob = (-norm3d(&wr) * dt / OB_RAD - dt / OB_TAU).exp();
		}
		for i in 0..3 {
			F[(IOB + i) * NS + IOB + i] = kob;
		}
		// P = F P F^T + Q (F P F^T into P itself: P is not read after F P)
		// (F has about 2.4 entries a row that are not zero: while the other factor is finite the products leave out the
		// rest, the same bits; one that is not, dense as before)
		let mut nz = [[0u8; NS]; NS];
		let mut nn = [0usize; NS];
		row_nz(F, &mut nz, &mut nn);
		if all_finite(&self.P) {
			mul_sparse_a(F, &nz, &nn, &self.P, FP);
		} else {
			mul_ikj(F, &self.P, FP);
		}
		if all_finite(&FP[..]) {
			mul_abt_sparse_b(FP, F, &nz, &nn, &mut self.P);
		} else {
			transpose(F, Mt);
			mul_ikj(FP, Mt, &mut self.P);
		}
		let mut sacc = self.s.sig_acc;
		if self.s.acc_knoise > 0.0 {
			// the error that grows with the motion (track-acc-knoise)
			let e = self.s.acc_knoise * c_fmax(0., norm3d(am) - GRAV);
			sacc = (sacc * sacc + e * e).sqrt();
		}
		if self.s.acc_tnoise > 0.0 {
			// the accelerometer under rotation (track-acc-tnoise): its reading belongs to a time within tnoise of the
			// gyro's (the two sensors' filters' group delays differ with frequency; the nominal difference is
			// ACC_LEAD_*), and the acceleration vector turns at the body rate, so the world acceleration is off by |f|
			// |w| tnoise. Nothing at rest, 0.75 m/s^2 per step in a fast head turn (|f| 15, 5 rad/s), 17 on a tracker
			// whirled at 34 rad/s on a 32 mm arm (|f| 50)
			let wr = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			let et = self.s.acc_tnoise * norm3d(am) * norm3d(&wr);
			sacc = (sacc * sacc + et * et).sqrt();
		}
		if self.s.adapt > 0.0 && (self.s.adapt_trackers != 0 || self.is_controller(h)) {
			// controllers only (track-adapt-trackers): on a tracker the "vibration" of this measure is the strap's
			// ringing (a head tracker's accelerometer in fast turns: 2.6 m/s^2 rms sample to sample, half of it at
			// 15-40 Hz, real and 0.1 mm of motion) and the jerk of a hard stop; it opens the velocity to the light's
			// noise exactly in fast turns, where the light then kinks the track by millimetres the IMU does not
			// support. The controllers keep it: after a punch that pinned both sensors the velocity's error is beyond
			// its covariance, and the roughness that follows is what keeps it open for the light
			// the vibration counts up to VIB_CAP and no further: a shaken controller's smooth 50 m/s^2 swings read 2-6
			// on this measure and are the motion itself, which the accelerometer reports correctly; past the cap the
			// noise is no longer the accelerometer's error
			let e = self.s.adapt * c_fmax(0., c_fmin(self.vib, VIB_CAP) - self.s.vib0);
			sacc = (sacc * sacc + e * e).sqrt();
		}
		if self.clipped {
			// the accelerometer is at full scale: the velocity is the light's
			sacc = c_fmax(sacc, if self.is_controller(h) { CLIP_SD_CONTROLLER } else { self.s.clip_sd });
		}
		let mut sgyr = self.s.sig_gyr;
		{
			// the gyro's scale error: about the rotation axis (GYRO_SCALE)
			let wr = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			let ww = Rv(R, &wr);
			let k2 = (GYRO_SCALE * dt) * (GYRO_SCALE * dt);
			for i in 0..3 {
				for j in 0..3 {
					self.P[(IQ + i) * NS + IQ + j] += k2 * ww[i] * ww[j];
					self.step_qth[i * 3 + j] = k2 * ww[i] * ww[j];
				}
			}
		}
		if self.gyro_clipped {
			sgyr = c_fmax(sgyr, GYRO_CLIP_SD); // the gyro is at full scale: the rate beyond it is unknown, the orientation is the light's
		}
		for i in 0..3 {
			self.P[(IV + i) * NS + IV + i] += (sacc * dt) * (sacc * dt);
			self.P[(IQ + i) * NS + IQ + i] += (sgyr * dt) * (sgyr * dt);
			self.P[(IBG + i) * NS + IBG + i] += (self.s.sig_bg * dt) * (self.s.sig_bg * dt);
			self.P[(IBA + i) * NS + IBA + i] += (self.s.sig_ba * dt) * (self.s.sig_ba * dt);
		}
		if self.s.sig_ls > 0.0 {
			self.P[ILS * NS + ILS] += (self.s.sig_ls * dt) * (self.s.sig_ls * dt);
		}
		for i in 0..3 {
			self.P[(IOB + i) * NS + IOB + i] += OB_SD * OB_SD * (1.0 - kob * kob);
		}
		self.step_qv = (sacc * dt) * (sacc * dt);
		for i in 0..3 {
			self.step_qth[i * 4] += (sgyr * dt) * (sgyr * dt);
		}
		self.step_qob = OB_SD * OB_SD * (1.0 - kob * kob);
		self.step_kob = kob;
	}

	/// the work arrays out of the tracker for a call (put back by it: a call inside another's would get new ones,
	/// and none is)
	fn take_work(&mut self) -> Box<Work> {
		self.work.take().unwrap_or_else(Work::boxed)
	}

	// ---- the sweep update
	// the newest state at or before time t: the current one when the state was propagated to it, else the history entry
	// (the sweep's hits arrived after the IMU samples of their time); the oldest entry if t is older than the history.
	// t = -1 when no state of its time exists (the rest is zero and unread)
	fn base_at(&self, t: f64) -> Base {
		let mut b = Base::default();
		if t >= self.t - 1e-9 || self.hist_n == 0 {
			b.t = self.t;
			b.p = self.p;
			b.v = self.v;
			b.q = self.q;
			b.dr_p = self.dr_p;
			b.dr_v = self.dr_v;
			for i in 0..3 {
				b.w[i] = self.last_gyr[i] - self.bg[i];
			}
			return b;
		}
		let mut k = 0;
		for j in 0..self.hist_n {
			let h = &self.hist[ix((self.hist_i - 1 - j + 2 * HISTN_I) % HISTN_I)];
			k = j;
			if h.t <= t {
				break;
			}
		}
		let h = &self.hist[ix((self.hist_i - 1 - k + 2 * HISTN_I) % HISTN_I)];
		if h.t > t + 0.01 {
			// older than the history: no state of its time exists
			b.t = -1.0;
			return b;
		}
		b.t = h.t;
		b.p = h.p;
		b.v = h.v;
		b.q = h.q;
		b.w = h.w;
		b.dr_p = h.dr_p;
		b.dr_v = h.dr_v;
		b
	}

	// predict's terms that no perturbation changes (HitsAt), for the sweep's hits from base b, the station's calibration
	// bcal
	fn hits_at<H: Host>(&self, h: &H, sw: &Sweep, b: &Base, bcal: &[BaseStationCal; 2], at: &mut HitsAt) {
		at.k = gen2_axis_consts(bcal, ix(sw.plane));
		let scale = 1.0 + self.ls + 0.0; // predict's, with ds 0
		for k in 0..sw.n as usize {
			let dt = sw.t[k] + self.s.hit_shift - b.t;
			let pt = h.sensor_location(sw.sensor[k]);
			let ptl = [(pt[0] - 0.0) * scale, (pt[1] - 0.0) * scale, (pt[2] - 0.0) * scale];
			// the body rotation between the base time and the hit's own time (the controller's sensors are up to 16 cm
			// from the IMU: at 7 rad/s a hit 4 ms away is 4 mm off without it)
			let wdt = [b.w[0] * dt, b.w[1] * dt, b.w[2] * dt];
			let e = qexp(&wdt);
			at.dt[k] = dt;
			at.e[k] = e;
			at.pt[k] = pt;
			at.ptl[k] = quatrotatevector(&e, &ptl);
		}
	}

	// predicted angle of hit k for a perturbed base: dp (position), dth (orientation, world side), ds (layout scale); at:
	// hits_at's terms of this sweep and base
	#[allow(clippy::too_many_arguments)]
	fn predict(
		&self, sw: &Sweep, b: &Base, at: &HitsAt, dp: Option<&[f64; 3]>, dth: Option<&[f64; 3]>, ds: f64,
		R0: &[f64; 9], world2lh: &SurvivePose, bcal: &[BaseStationCal; 2], pred: &mut [f64; HMAX],
		mut pw_out: Option<&mut [f64; HMAX * 3]>,
	) {
		let R = match dth {
			Some(dth) => {
				let mut q = b.q;
				qrot_left(&mut q, dth);
				q2R(&q)
			}
			None => *R0,
		};
		let scale = 1.0 + self.ls + ds;
		// o = 0: the state point in the IMU frame: the IMU itself
		for k in 0..sw.n as usize {
			let dt = at.dt[k];
			// the sensor turned by the body's rotation since the base: hits_at's, or at the perturbed scale
			let ptl = if ds == 0.0 {
				at.ptl[k]
			} else {
				let pt = at.pt[k];
				quatrotatevector(&at.e[k], &[(pt[0] - 0.0) * scale, (pt[1] - 0.0) * scale, (pt[2] - 0.0) * scale])
			};
			let pr = Rv(&R, &ptl);
			let mut pw = [0.0; 3];
			for i in 0..3 {
				pw[i] = b.p[i]
					+ (match dp {
						Some(dp) => dp[i],
						None => 0.0,
					}) + b.v[i] * dt
					+ pr[i];
			}
			let pl = apply_pose_to_point(world2lh, &pw);
			// REPROJECT_AXIS_FN_GEN2[plane](bcal, pl), with the plane's terms from hits_at
			pred[k] = survive_reproject_axis_gen2_with(bcal, &at.k, ix(sw.plane), &pl);
			if let Some(out) = pw_out.as_deref_mut() {
				out[3 * k..3 * k + 3].copy_from_slice(&pw);
			}
		}
	}

	// the handover test (track-blink): whether this station's sweep is dropped now. At each window's start the kept station is
	// the one with the most accepted hits in the last 0.1 s (TRACK_BLINK t kept-station channel stations-lit; kept -1 = none lit,
	// nothing dropped in that window)
	fn blink_drop<H: Host>(&mut self, h: &mut H, lh: i32, t: f64) -> bool {
		if self.s.blink == 0.0 {
			return false;
		}
		let len = self.s.blink.abs(); // < 0: a blackout test, every station dropped (pure dead reckoning)
		let period = len + 2.5;
		let k = (t / period).floor() as c_long;
		if t - k as f64 * period >= len {
			return false;
		}
		if k != self.blink_k {
			self.blink_k = k;
			let mut used = [0i32; NLH];
			let mut lit = 0;
			for j in 0..ix(self.recent_n) {
				let r = &self.recent[j];
				if r.t > t - 0.1 && r.t <= t && r.lh >= 0 && r.lh < NLH as i32 {
					used[ix(r.lh)] += r.used;
				}
			}
			self.blink_lh = -1;
			for i in 0..NLH {
				if used[i] > 0 {
					lit += 1;
				}
				if used[i] > 0 && (self.blink_lh < 0 || used[i] > used[ix(self.blink_lh)]) {
					self.blink_lh = i as i32;
				}
			}
			if self.s.verbose >= 1 && h.recording() {
				h.record(
					c"%s TRACK_BLINK %.4f %d %d %d\r\n",
					(
						h.codename(),
						t,
						self.blink_lh,
						if self.blink_lh >= 0 { h.bsd_mode(self.blink_lh) } else { -1 },
						lit,
					),
				);
			}
		}
		if self.s.blink < 0.0 {
			return self.blink_lh >= 0; // a blackout window (only where some station was lit, as the TRACK_BLINK line says)
		}
		self.blink_lh >= 0 && lh != self.blink_lh
	}

	/// one sweep's outcome into the divergence test's ring
	fn recent_push(&mut self, t: f64, used: i32, rej: i32, lh: i32, plane: i32, coherent: bool) {
		self.recent[ix(self.recent_i)] = Recent { t, used, rej, lh, plane, coherent };
		self.recent_i = (self.recent_i + 1) % RECENT_I;
		if self.recent_n < RECENT_I {
			self.recent_n += 1;
		}
	}

	/// TRACK_HIT t ch plane sensor residual_mrad cos_incidence used(1/0), the sensor point (m) and normal in the station frame
	#[allow(clippy::too_many_arguments)]
	fn record_c2h<H: Host>(
		&self, h: &mut H, sw: &Sweep, sw_t: f64, mode: i32, R: &[f64; 9], lh2world: &SurvivePose,
		world2lh: &SurvivePose, pw: &[f64; HMAX * 3], r: &[f64; HMAX], ok: &[bool; HMAX],
	) {
		for k in 0..sw.n as usize {
			let nw = Rv(R, &h.sensor_normal(sw.sensor[k]));
			let dir = sub3d(&lh2world.pos, &v3(&pw[3 * k..]));
			let dir = normalize3d(&dir);
			let plh = apply_pose_to_point(world2lh, &v3(&pw[3 * k..]));
			let nlh = quatrotatevector(&world2lh.rot, &nw);
			h.record(
				c"%s TRACK_HIT %.4f %d %d %d %+.3f %+.3f %d %.4f %.4f %.4f %+.4f %+.4f %+.4f\r\n",
				(
					h.codename(),
					sw_t,
					mode,
					sw.plane,
					sw.sensor[k],
					r[k] * 1e3,
					dot3d(&nw, &dir),
					ok[k] as i32,
					plh[0],
					plh[1],
					plh[2],
					nlh[0],
					nlh[1],
					nlh[2],
				),
			);
		}
	}

	fn update<H: Host>(&mut self, h: &mut H, sw: &Sweep, sw_t: f64) {
		let mut w = self.take_work();
		self.update_in(h, sw, sw_t, &mut w);
		self.work = Some(w);
	}
	fn update_in<H: Host>(&mut self, h: &mut H, sw: &Sweep, sw_t: f64, w: &mut Work) {
		let Work { at, pw, H, PH, Pq, PHt, S, S0, X, K, KS, KW, IKH, T, Pn, L, Mt, .. } = w;
		let lh = sw.lh;
		if lh < 0
			|| lh >= h.active_lighthouses()
			|| !h.bsd_position_set(lh)
			|| !h.sensor_locations_set()
			|| !h.sensor_normals_set()
		{
			return;
		}
		if self.s.only_mask != 0 && (self.s.only_mask.wrapping_shr(h.bsd_mode(lh) as u32) & 1) == 0 {
			return;
		}
		let bcal = h.bsd_fcal(lh);
		let lh2world = h.lighthouse_position(lh);
		let world2lh = invert_pose(&lh2world);
		let n = sw.n as usize;
		// the base: the newest state at or before the sweep time (the current one when the state was propagated to it)
		let mut b = self.base_at(sw_t);
		if b.t < 0.0 {
			// the sweep is older than the state history (a burst of light held up on the radio): nothing to evaluate it against
			inc(&mut self.stats.sweeps_dropped);
			return;
		}
		if b.t < self.t {
			self.last_stale = self.t - b.t;
		}
		// the light's return (RETURN_DARK): this sweep reached the update after the device had been dark
		if sw_t - self.sweep_in_t >= RETURN_DARK {
			self.ret_armed = true;
			self.ret_gap = sw_t - self.sweep_in_t;
		}
		if sw_t > self.sweep_in_t {
			self.sweep_in_t = sw_t;
		}
		qrot_left(&mut b.q, &self.ob); // the hits see the orientation turned by the light's offset (OB_SD)
		let R = q2R(&b.q);
		let mut pred = [0.0; HMAX];
		let mut r = [0.0; HMAX];
		let mut sk2 = [0.0; HMAX];
		let mut ok = [false; HMAX];
		let mut ok_facing = [false; HMAX];
		self.hits_at(h, sw, &b, &bcal, at);
		self.predict(sw, &b, at, None, None, 0.0, &R, &world2lh, &bcal, &mut pred, Some(&mut *pw));
		// back-facing hits (reflections) out
		let mut nok: i32 = 0;
		for k in 0..n {
			let nw = Rv(&R, &h.sensor_normal(sw.sensor[k]));
			let dir = sub3d(&lh2world.pos, &v3(&pw[3 * k..]));
			let dir = normalize3d(&dir);
			let cosi = dot3d(&nw, &dir);
			ok[k] = cosi > self.facing_cos;
			// hit noise grows with the incidence angle at the sensor (measured against the all-station track: 0.2 mrad
			// below 70 deg, 0.45 at 70-80, 1.2-1.9 beyond 80): sd = hit-sd * (1 + (angle / knee)^6)
			{
				let ang = c_fmax(-1., c_fmin(1., cosi)).acos() / (self.s.hit_knee * LINMATHPI / 180.);
				let f = 1.0 + ang * ang * ang * ang * ang * ang;
				sk2[k] = self.s.sig_hit * self.s.sig_hit * f * f;
			}
			if !ok[k] {
				inc(&mut self.stats.facing_drops);
			} else {
				nok += 1;
			}
			ok_facing[k] = ok[k];
			r[k] = sw.angle[k] - pred[k];
		}
		if self.s.probe_ch >= 0 && h.bsd_mode(lh) == self.s.probe_ch {
			// the probe station: logged, never used
			if self.s.verbose >= 3 && h.recording() && self.seeded && !self.want_reseed {
				self.record_c2h(h, sw, sw_t, h.bsd_mode(lh), &R, &lh2world, &world2lh, pw, &r, &ok);
			}
			return;
		}
		if nok == 0 {
			self.stats.rejected = self.stats.rejected.wrapping_add(sw.n as u64);
			self.recent_push(sw_t, 0, sw.n, lh, sw.plane, false);
			return;
		}
		// numerical Jacobian wrt position, orientation and layout scale (into H's rows of this sweep, cleared: the
		// columns no Jacobian below sets stay zero)
		H[..n * NS].fill(0.0);
		let eps = 1e-5;
		let mut pd = [0.0; HMAX];
		for cc in 0..3 {
			let mut d = [0.0, 0.0, 0.0];
			d[cc] = eps;
			self.predict(sw, &b, at, Some(&d), None, 0.0, &R, &world2lh, &bcal, &mut pd, None);
			for k in 0..n {
				H[k * NS + IP + cc] = (pd[k] - pred[k]) / eps;
			}
			self.predict(sw, &b, at, None, Some(&d), 0.0, &R, &world2lh, &bcal, &mut pd, None);
			for k in 0..n {
				H[k * NS + IQ + cc] = (pd[k] - pred[k]) / eps;
			}
		}
		for k in 0..n {
			// the offset turns the orientation the hits see exactly as an orientation error does
			for i in 0..3 {
				H[k * NS + IOB + i] = H[k * NS + IQ + i];
			}
		}
		// first light: the station's light is back after JOIN_GAP without a sweep from it. Its extra sweep-common error
		// fades with the time and with the angle the device has turned since (both), and applies only while another
		// station's hits are being accepted: it protects a pose that other light holds; with nothing else holding the
		// pose the joining station is all there is, and the caution would only delay finding a pose that is off (the
		// joining station's sweeps, let in through the caution's wider gate, keep the lost test from firing)
		let l = ix(lh);
		if sw_t - self.last_sweep_t[l] > JOIN_GAP {
			self.light_since[l] = sw_t;
			self.rot_at_light[l] = self.rot_cum;
		}
		let mut join = 0.0;
		if self.s.sig_join > 0.0 {
			let mut held = false;
			let mut i = 0;
			while (i as i32) < h.active_lighthouses() && i < NLH {
				if i != l && sw_t - self.last_used_t[i] < JOIN_HOLD {
					held = true;
				}
				i += 1;
			}
			if held {
				join = self.s.sig_join
					* (-(sw_t - self.light_since[l]) / JOIN_TAU).exp()
					* (-(self.rot_cum - self.rot_at_light[l]) / (JOIN_DEG as f64 * LINMATHPI / 180.)).exp();
			}
		}
		// the layout scale is learned only while two or more stations light the device (within 0.5 s): with one station
		// the scale and the depth are the same measurement, and the scale would absorb the depth error (-2.4 % = -11 cm
		// at 4.5 m in a one-station replay). With one station it is a consider state: left out of the update (its gain
		// row zero), its correlations carried along
		self.last_sweep_t[l] = sw_t;
		let mut stations = 0;
		let mut i = 0;
		while (i as i32) < h.active_lighthouses() && i < NLH {
			stations += (self.last_sweep_t[i] > sw_t - 0.5) as i32;
			i += 1;
		}
		let ls_possible = self.s.sig_ls > 0.0 && stations >= 2;
		if ls_possible {
			self.predict(sw, &b, at, None, None, eps, &R, &world2lh, &bcal, &mut pd, None);
			for k in 0..n {
				H[k * NS + ILS] = (pd[k] - pred[k]) / eps;
			}
		}
		// the velocity's part (STALE_H): a hit measures the device at its own time, the state is at c->t, and under the
		// state's own error model a velocity error moves the position by itself times the time between, so d angle / d v =
		// H_p (t_hit - c->t). On time that is the hit's place in its sweep (+- 1 ms); for a sweep whose light the radio held
		// up (the base from the history) it is minus its age, and without it the position's correlation with the velocity
		// put a residual of then into the velocity of now: with the light 20 ms late it turned the velocity the wrong way.
		// Only while two or more stations light the device, as the layout scale: with one, the depth along its line of
		// sight is measured only through the apparent size, and that measurement's bias would drive the velocity along it
		let hv = STALE_H && stations >= 2;
		if hv {
			for k in 0..n {
				let tk = sw.t[k] + self.s.hit_shift - self.t;
				for cc in 0..3 {
					H[k * NS + IV + cc] = tk * H[k * NS + IP + cc];
				}
			}
		}
		// ... and what the late light cannot know (STALE_Q): the state took on process noise between the sweep's time and
		// now (the accelerometer's, the gyro's scale error about the turn, the light offset's change with the turn), and that
		// noise is in P but not in what the sweep measured. Taken as measuring the state of now with all of P, the sweep
		// corrected that noise too: P's velocity and orientation as of now are partly the motion after the sweep, and the
		// update put the sweep's residual into them. The exact delayed measurement: with x_now = F x_then + w, w ~ Q (the
		// interval's noise mapped to now), the sweep's covariance with the state is (P - Q) H^T and its innovation
		// covariance H (P - Q) H^T + R; the light offset measured is the one of the sweep's time, decayed since (H / decay).
		// The interval's noise: each history entry holds its step's; velocity noise in a step at t_k carries into the
		// position by (now - t_k). With the velocity columns only (hv): without them the model takes the hits as
		// measuring the current state, and half of the delayed model (the noise out, the velocity's part not in) is
		// worse on one station. The interval's noise is 20-45 % of the velocity's variance for the late sweeps of a
		// device in motion
		let mut use_pq = false; // Pu: Pq when set, else c->P (Pq is the work array's, set below before it is read)
		self.stale_qv = 0.0;
		self.stale_qth = 0.0;
		if STALE_Q && hv && b.t < self.t - 1e-9 && self.hist_n > 1 {
			let (mut qpp, mut qpv, mut qvv, mut qth, mut qob, mut prod, mut kint) =
				(0.0, 0.0, 0.0, [0.0; 9], 0.0, 1.0, 1.0);
			let mut j = 0;
			while j + 1 < self.hist_n {
				// newest first; each entry's step began at the next older entry
				let h = &self.hist[ix((self.hist_i - 1 - j + 2 * HISTN_I) % HISTN_I)];
				let h0 = &self.hist[ix((self.hist_i - 2 - j + 2 * HISTN_I) % HISTN_I)];
				j += 1;
				if h.t <= b.t + 1e-9 {
					break;
				}
				if h.t > self.t + 1e-9 || h.t <= h0.t {
					continue;
				}
				let f = c_fmin(1., c_fmax(0., (h.t - sw_t) / (h.t - h0.t))); // the part of the step after the sweep's time
				if f <= 0.0 {
					continue;
				}
				let a = self.t - h.t;
				qvv += f * h.nq_v;
				qpv += f * h.nq_v * a;
				qpp += f * h.nq_v * a * a;
				for i in 0..9 {
					qth[i] += f * h.nq_th[i];
				}
				qob += f * h.nq_ob * prod * prod;
				let kd = c_pow(c_fmax(h.kob, 1e-6), f);
				prod *= kd;
				kint *= kd;
			}
			// P - Q must stay a covariance: Q here is the interval's noise without its cross terms (the orientation noise
			// that the acceleration turns into velocity in the steps after it, ...), and a sweep older than one already
			// taken (out of time order) finds part of its interval's noise taken out already. The largest share of Q
			// (1, 1/2, 1/4) that leaves P - Q positive definite, else P as it is
			let mut okq = false;
			let mut al = 1.0;
			let mut tr = 0;
			while tr < 3 && kint > 0.05 {
				*Pq = self.P;
				for i in 0..3 {
					Pq[(IP + i) * NS + IP + i] -= al * qpp;
					Pq[(IP + i) * NS + IV + i] -= al * qpv;
					Pq[(IV + i) * NS + IP + i] -= al * qpv;
					Pq[(IV + i) * NS + IV + i] -= al * qvv;
					Pq[(IOB + i) * NS + IOB + i] -= al * qob;
					for j in 0..3 {
						Pq[(IQ + i) * NS + IQ + j] -= al * qth[i * 3 + j];
					}
				}
				if is_pd(&Pq[..], NS, L) {
					okq = true;
					break;
				}
				tr += 1;
				al *= 0.5;
			}
			if okq {
				qvv *= al;
				qth[0] *= al;
				qth[4] *= al;
				qth[8] *= al;
				inc(&mut self.stats.stale_q[if al > 0.75 {
					0
				} else if al > 0.375 {
					1
				} else {
					2
				}]);
			} else if kint > 0.05 {
				inc(&mut self.stats.stale_q[3]);
			}
			if okq {
				use_pq = true;
				for k in 0..n {
					for i in 0..3 {
						H[k * NS + IOB + i] /= kint;
					}
				}
				let pv = self.P[IV * NS + IV] + self.P[(IV + 1) * NS + IV + 1] + self.P[(IV + 2) * NS + IV + 2];
				let pt = self.P[IQ * NS + IQ] + self.P[(IQ + 1) * NS + IQ + 1] + self.P[(IQ + 2) * NS + IQ + 2];
				self.stale_qv = if pv > 0.0 { 3.0 * qvv / pv } else { 0.0 };
				self.stale_qth = if pt > 0.0 { (qth[0] + qth[4] + qth[8]) / pt } else { 0.0 };
			}
		}
		// per-hit gate on the predicted innovation variance (the first-light term included: a joining station's biased
		// hits are taken at a low weight rather than dropped, which in fast motion is most of the light there is)
		let sc2 = self.s.sig_common * self.s.sig_common + join * join;
		{
			let Pu = if use_pq { &*Pq } else { &self.P };
			for k in 0..n {
				if !ok[k] {
					continue;
				}
				let mut Sd = sk2[k] + sc2;
				for i in 0..NS {
					let hi = H[k * NS + i];
					if hi == 0.0 {
						continue;
					}
					let mut s = 0.0;
					for j in 0..NS {
						s += Pu[i * NS + j] * H[k * NS + j];
					}
					PH[k * NS + i] = s; // (P H^T's entry, kept for PHt below)
					Sd += hi * s;
				}
				let lim = c_fmax(self.s.gate * Sd.sqrt(), self.s.gate_floor);
				ok[k] = r[k].abs() < lim;
			}
		}
		// ... and, while two or more stations light the device, a hit that disagrees with the others of its sweep: a
		// sensor that saw the station's light off glass beside it (REFL_HIT_Z). Taken out only where the hits left then
		// fit their noise: in a sweep that fits nothing well (fast motion, the state off) the worst hit is no outlier
		if stations >= 2 {
			let Pu = if use_pq { &*Pq } else { &self.P };
			let sb2 = if ls_possible { LS_STATION_SD * LS_STATION_SD } else { 0.0 };
			let mut left = ok;
			let mut dropped = false;
			let mut clean = false;
			loop {
				let mut sel = [0usize; HMAX];
				let mut ns = 0;
				for k in 0..n {
					if left[k] {
						sel[ns] = k;
						ns += 1;
					}
				}
				if ns < REFL_HIT_MIN {
					break;
				}
				let Some((a, z, chi2)) = loo_worst(
					&H[..],
					&Pu[..],
					&sk2[..],
					sc2,
					sb2,
					&r[..],
					&sel[..ns],
					&mut PHt[..],
					&mut S[..],
					&mut S0[..],
				) else {
					break;
				};
				if z <= REFL_HIT_Z {
					clean = chi2 <= chi2_99(ns as f64);
					break;
				}
				left[sel[a]] = false;
				dropped = true;
			}
			if dropped && clean {
				ok = left;
			}
		}
		let mut m = 0;
		let mut idx = [0usize; HMAX];
		for k in 0..n {
			if ok[k] {
				idx[m] = k;
				m += 1;
			}
		}
		let rej = sw.n - m as i32; // facing drops count as rejected too
		let mut ls_active = ls_possible && m == n; // and a clean sweep (checked below on the median residual)
		if ls_active {
			let mut ar = [0.0; HMAX];
			for k in 0..n {
				ar[k] = r[k].abs();
			}
			insertion_sort(&mut ar[..n]);
			ls_active = ar[n / 2] < 2e-3;
		}
		// ... and a sweep that fits the state as well as its noise says it should: its residuals within the 99 % point
		// of chi-square against the sweep's noise (each hit's own and their common part, first light's in it). A sweep
		// the pose does not fit yet (a start, a station come into view, light at grazing incidence) carries the pose's
		// error, and the scale takes it, directly or through its correlation with the position: at a start it runs to
		// its clamp within a second and takes minutes back. A scale error itself is far below that (1 % = 0.15 mrad
		// across a tracker 3 m away), so a settled sweep still corrects it
		if ls_active && LS_FIT {
			ls_active = fits_noise(&r[..n], &sk2[..n], sc2);
		}
		// the light is back after a blackout (track-gap-reseed s without an accepted hit): the coasted state is kept if
		// this sweep still fits it (median |residual| within track-seed-fit; the gate has opened with the covariance),
		// and lost otherwise. Lost (that, or the divergence test): no update until the tracker's own seed solve fits
		// the sweeps in hand. An EKF update from a pose that is far off (after a blackout the covariance is huge, the
		// gate lets 100 mrad residuals through) only runs the state further away
		let gap_back = self.s.gap_reseed > 0.0 && sw_t - self.last_light_t > self.s.gap_reseed;
		if gap_back && !self.want_reseed && nok < 3 {
			return; // too few hits to judge the coasted state: no update from a sweep that could be anything, wait for a fuller one
		}
		if gap_back && !self.want_reseed {
			let mut ar = [0.0; HMAX];
			let mut na = 0;
			for k in 0..n {
				if ok[k] || r[k].abs() < self.s.seed_fit {
					ar[na] = r[k].abs();
					na += 1;
				}
			}
			insertion_sort(&mut ar[..na]);
			let fits = na * 2 > n && ar[na / 2] < self.s.seed_fit;
			if self.s.verbose >= 1 {
				h.sv_info(
					c"%s track: light back after a blackout at %.3f s (%.2f s): the coasted state %s this sweep (%d of %d hits within %.1f mrad)",
					(
						h.codename(),
						sw_t,
						sw_t - self.last_light_t,
						if fits { c"fits" } else { c"does NOT fit, lost" },
						na as i32,
						sw.n,
						self.s.seed_fit * 1e3,
					),
				);
			}
			if !fits {
				self.want_reseed = true;
				self.lost_warm = true;
				inci(&mut self.stats.gap_reseeds);
			}
		}
		if self.want_reseed {
			self.stats.rejected = self.stats.rejected.wrapping_add(sw.n as u64);
			self.recent_push(sw_t, 0, sw.n, lh, sw.plane, false);
			// lost: the seed solve on the buffered sweeps (this one included), rate-limited; on success the state is new
			// and the pending sweeps are dropped
			if sw_t - self.seed_try_t >= 0.02 && self.seed_solve(h, sw_t, self.lost_warm, 0, 0) {
				inci(&mut self.stats.reseeds);
			}
			return;
		}
		// (as they were, for an update that cannot be made after all: below)
		let light_was = (self.last_light_t, self.last_used_t[l]);
		if m > 0 {
			self.last_light_t = c_fmax(self.last_light_t, sw_t);
			self.last_used_t[l] = c_fmax(self.last_used_t[l], sw_t);
		}
		if self.s.verbose >= 3 && h.recording() {
			// TRACK_HIT t ch plane sensor residual_mrad cos_incidence used(1/0), the sensor point (m) and normal in the station frame: every hit of the sweep
			self.record_c2h(h, sw, sw_t, h.bsd_mode(lh), &R, &lh2world, &world2lh, pw, &r, &ok);
		}
		if self.s.verbose >= 3 && h.recording() {
			// TRACK_DEPTH t lh plane used: along the station's line of sight, position sd mm, velocity sd mm/s; this
			// sweep's depth estimate from the used hits' residuals (least squares on the position Jacobian projected on
			// the line, mm) and its sd (mm); the fraction of the sweep's position information that lies along the line;
			// then the orientation sd about the world axes (mrad) and the accelerometer bias sd (mm/s^2). The
			// one-station depth diagnostic: a device can drift along a station's line while that station fits at 0.2
			// mrad and the filter believes the depth to 1-3 mm
			let dvec = sub3d(&b.p, &lh2world.pos);
			let dvec = normalize3d(&dvec);
			let (mut pdd, mut pvv) = (0.0, 0.0);
			for i in 0..3 {
				for j in 0..3 {
					pdd += dvec[i] * self.P[(IP + i) * NS + IP + j] * dvec[j];
					pvv += dvec[i] * self.P[(IV + i) * NS + IV + j] * dvec[j];
				}
			}
			let (mut shh, mut shr, mut sall) = (0.0, 0.0, 0.0);
			for k in 0..n {
				let mut hd = 0.0;
				for i in 0..3 {
					hd += H[k * NS + IP + i] * dvec[i];
				}
				for i in 0..3 {
					sall += H[k * NS + IP + i] * H[k * NS + IP + i];
				}
				if !ok[k] {
					continue;
				}
				shh += hd * hd;
				shr += hd * r[k];
			}
			h.record(
				c"%s TRACK_DEPTH %.4f %d %d %d %.2f %.1f %+.1f %.1f %.3f %.2f %.2f %.2f %.1f %.1f %.1f\r\n",
				(
					h.codename(),
					sw_t,
					h.bsd_mode(lh),
					sw.plane,
					m as i32,
					c_fmax(pdd, 0.0).sqrt() * 1e3,
					c_fmax(pvv, 0.0).sqrt() * 1e3,
					if shh > 0.0 { shr / shh * 1e3 } else { 0. },
					if shh > 0.0 { self.s.sig_hit / shh.sqrt() * 1e3 } else { 0. },
					if sall > 0.0 { shh / sall } else { 0. },
					c_fmax(self.P[IQ * NS + IQ], 0.0).sqrt() * 1e3,
					c_fmax(self.P[(IQ + 1) * NS + IQ + 1], 0.0).sqrt() * 1e3,
					c_fmax(self.P[(IQ + 2) * NS + IQ + 2], 0.0).sqrt() * 1e3,
					c_fmax(self.P[IBA * NS + IBA], 0.0).sqrt() * 1e3,
					c_fmax(self.P[(IBA + 1) * NS + IBA + 1], 0.0).sqrt() * 1e3,
					c_fmax(self.P[(IBA + 2) * NS + IBA + 2], 0.0).sqrt() * 1e3,
				),
			);
		}
		{
			// the light's residual (Diag's): this sweep's median |residual|, TRACK_SWEEP's quantity, smoothed
			let mut ar = [0.0; HMAX];
			for k in 0..n {
				ar[k] = r[k].abs();
			}
			insertion_sort(&mut ar[..n]);
			let med = ar[n / 2] * 1e3;
			let fresh = self.res_mrad < 0.0 || sw_t - self.res_t > 0.5;
			self.res_mrad = if fresh { med } else { 0.8 * self.res_mrad + 0.2 * med };
			self.res_t = sw_t;
		}
		if self.s.verbose >= 2 && h.recording() {
			// TRACK_SWEEP t lh plane n used median|resid| mrad, gate sd mrad, position sd mm, |v|, vib, median signed resid
			let mut ar = [0.0; HMAX];
			let mut sr = [0.0; HMAX];
			for k in 0..n {
				ar[k] = r[k].abs();
				sr[k] = r[k];
			}
			insertion_sort(&mut ar[..n]);
			insertion_sort(&mut sr[..n]);
			let psd = (self.P[IP * NS + IP] + self.P[(IP + 1) * NS + IP + 1] + self.P[(IP + 2) * NS + IP + 2]).sqrt();
			h.record(
				c"%s TRACK_SWEEP %.4f %d %d %d %d %.3f %.1f %.2f %.3f %+.3f %.1f %.3f %.3f\r\n",
				(
					h.codename(),
					sw_t,
					h.bsd_mode(lh),
					sw.plane,
					sw.n,
					m as i32,
					ar[n / 2] * 1e3,
					psd * 1e3,
					norm3d(&self.v),
					self.vib,
					sr[n / 2] * 1e3,
					self.last_stale * 1e3,
					self.stale_qv,
					self.stale_qth,
				),
			);
		}
		self.stats.used = self.stats.used.wrapping_add(m as u64);
		self.stats.rejected = self.stats.rejected.wrapping_add(rej as u64);
		{
			// coherently rejected: three or more hits, none used, their signed residuals one-sided beyond DISAGREE_MRAD
			let mut coh = m == 0 && nok >= 3;
			if coh {
				let (mut pos, mut neg) = (0, 0);
				for k in 0..n {
					if !ok_facing[k] {
						continue;
					}
					if r[k] > DISAGREE_MRAD * 1e-3 {
						pos += 1;
					} else if r[k] < -DISAGREE_MRAD * 1e-3 {
						neg += 1;
					}
				}
				coh = pos == nok || neg == nok;
			}
			self.recent_push(sw_t, m as i32, rej, lh, sw.plane, coh);
		}
		if m == 0 && sw_t - self.disagree_t > 0.5 {
			// the light disagrees: over the last 0.1 s, sweep planes coherently rejected (two sweeps or more, none
			// accepted) outnumber sweep planes accepted (two sweeps or more), and there are two of them. A sweep plane
			// measures one direction, so a plane that fits says nothing about the state along the plane: a state
			// sliding along the line where one plane of one station meets one plane of another fits both exactly while
			// every other plane rejects (counted per station, those two "accept" and tie the two rejecting stations
			// 2:2). Two independent reflections do not agree, so two planes against are required. A warm restart from
			// the buffered sweeps of every station, at most once per 0.5 s
			let mut coh = [0i32; NLH * 2];
			let mut acc = [0i32; NLH * 2];
			for k in 0..ix(self.recent_n) {
				let rc = &self.recent[k];
				if rc.t <= sw_t - 0.1 || rc.t > sw_t || rc.lh < 0 || rc.lh >= NLH as i32 {
					continue;
				}
				let f = ix(rc.lh * 2 + (rc.plane & 1));
				if rc.used > 0 {
					acc[f] += 1;
				} else if rc.coherent {
					coh[f] += 1;
				}
			}
			let (mut against, mut forr) = (0, 0);
			for i in 0..NLH * 2 {
				if acc[i] >= 2 {
					forr += 1;
				} else if coh[i] >= 2 {
					against += 1;
				}
			}
			// one station against itself: its one plane accepted, its other plane coherently rejected, and no other
			// plane holding the state. The two planes of a station are swept by the same rotor through the same optics,
			// so a reflection moves both (they then fit a mirror pose together, or the facing test drops them); a
			// station whose planes split means the state has left the plane that rejects. With one station lit the vote
			// above can never reach two planes against, and the state slides along the plane that still fits at the
			// speed a restart's first sweeps gave it, for seconds and metres (findings.md section 4). With every
			// station lit it fires rarely (hands in punches).
			// ... and with more stations lit when every accepting plane's own station rejects with its other plane (two
			// stations split, two planes against two): the tie below has no second side to try then (every station that
			// rejects also accepts), and the state slid along where the accepting planes meet, as with one station
			let all_split = (0..NLH).all(|i| {
				let (a0, a1) = (acc[2 * i] >= 2, acc[2 * i + 1] >= 2);
				(!a0 && !a1) || (a0 && !a1 && coh[2 * i + 1] >= 2) || (a1 && !a0 && coh[2 * i] >= 2)
			});
			let mut split: i32 = -1;
			if forr == 1 || all_split {
				let mut i = 0;
				while i < NLH && split < 0 {
					if (acc[2 * i] >= 2 && acc[2 * i + 1] < 2 && coh[2 * i + 1] >= 2)
						|| (acc[2 * i + 1] >= 2 && acc[2 * i] < 2 && coh[2 * i] >= 2)
					{
						split = i as i32;
					}
					i += 1;
				}
			}
			// a tie (as many planes against the state as for it, two or more each, no station split): the vote cannot
			// say which side is right, and the state would stand until a third plane joined. A restart from one station
			// alone can sit metres off along that station's line of sight (a station's two planes fix the directions to
			// the sensors from that station, not the distance along them, so it keeps fitting) while another station's
			// planes reject it coherently, two against two. The light decides: the warm solve on the buffered sweeps of
			// every station is tried, and taken only if its pose fits hits of both sides (3 or more of a station that
			// accepts and of one that rejects). A reflection cannot share one rigid pose with another station's direct
			// light, so such a pose means the rejecting station was right and the state was off where the accepting
			// station is blind. With no such pose the tie stands. At most every 0.1 s
			if against >= 2 && against == forr && split < 0 && sw_t - self.tie_t > 0.1 {
				let (mut ma, mut mb): (c_uint, c_uint) = (0, 0);
				for i in 0..NLH * 2 {
					if acc[i] >= 2 {
						ma |= 1u32 << (i / 2);
					} else if coh[i] >= 2 {
						mb |= 1u32 << (i / 2);
					}
				}
				mb &= !ma; // a station on both sides is not a second witness
				self.tie_t = sw_t;
				if self.s.verbose >= 1 {
					h.sv_info(
						c"%s track: the vote ties at %.3f s (%d sweep planes coherently rejected against %d accepted)",
						(h.codename(), sw_t, against, forr),
					);
				}
				if ma != 0 && mb != 0 && self.seed_solve(h, sw_t, true, ma, mb) {
					self.disagree_t = sw_t;
					inci(&mut self.stats.disagreements);
					inci(&mut self.stats.reseeds);
					return;
				}
			}
			if (against >= 2 && against > forr) || split >= 0 {
				self.disagree_t = sw_t;
				inci(&mut self.stats.disagreements);
				self.want_reseed = true;
				self.lost_warm = true;
				if self.s.verbose >= 1 {
					if split >= 0 {
						h.sv_info(
							c"%s track: a station disagrees with itself at %.3f s (ch %d: one sweep plane accepted, the other coherently rejected, no other plane holding the state): the state is wrong, warm restart",
							(h.codename(), sw_t, h.bsd_mode(split)),
						);
					} else {
						h.sv_info(
							c"%s track: stations disagree at %.3f s (%d sweep planes coherently rejected against %d accepted): the state is wrong, warm restart",
							(h.codename(), sw_t, against, forr),
						);
					}
				}
				return;
			}
		}
		if m == 0 {
			return;
		}
		// K = P H^T (H P H^T + s2 I)^-1 over the accepted hits. Where the layout scale is in H (two or more stations), the
		// noise has the station plane's own error of the device's size in it (LS_STATION_SD: along the scale's column,
		// for every state, the depth a station's apparent size gives as much as the scale)
		let Pu = if use_pq { &*Pq } else { &self.P };
		let sb2 = if ls_possible { LS_STATION_SD * LS_STATION_SD } else { 0.0 };
		// PHt NS x m, S m x m, S0 (the innovation covariance, kept for the late light's covariance update: STALE_Q),
		// X m x NS, K NS x m: the work arrays' first parts, each written before it is read
		// (the entries where the hit's H is not zero are the gate's sums: the same sum, worked out once)
		for i in 0..NS {
			for a in 0..m {
				let k = idx[a];
				PHt[i * m + a] = if H[k * NS + i] == 0.0 {
					let mut s = 0.0;
					let h = &H[k * NS..k * NS + NS];
					for j in 0..NS {
						s += Pu[i * NS + j] * h[j];
					}
					s
				} else {
					PH[k * NS + i]
				};
			}
		}
		for a in 0..m {
			for b in 0..m {
				let mut s = 0.0;
				let h = &H[idx[a] * NS..idx[a] * NS + NS];
				for i in 0..NS {
					s += h[i] * PHt[i * m + b];
				}
				S[a * m + b] = s
					+ (if a == b { sk2[idx[a]] } else { 0.0 })
					+ sc2 + sb2 * H[idx[a] * NS + ILS] * H[idx[b] * NS + ILS];
			}
		}
		if use_pq {
			S0[..m * m].copy_from_slice(&S[..m * m]);
		}
		// X = S^-1 (P H^T)^T  (m x NS), K = X^T
		for a in 0..m {
			for i in 0..NS {
				X[a * NS + i] = PHt[i * m + a];
			}
		}
		if !chol_solve(&mut S[..m * m], m, &mut X[..m * NS], NS) {
			h.sv_warn(c"%s track: innovation matrix not positive definite (%d hits)", (h.codename(), m as i32));
			// sally: the sweep was counted as light the state fits (the light's time, the hits used, the divergence
			// test's ring) before the update could fail, so a P no longer positive definite went on dead-reckoning
			// while every sweep "fit". Not used after all: its hits rejected, so the divergence test and the light's
			// timeout see what happened, and the restart they lead to starts P again
			(self.last_light_t, self.last_used_t[l]) = light_was;
			self.stats.used = self.stats.used.wrapping_sub(m as u64);
			self.stats.rejected = self.stats.rejected.wrapping_add(m as u64);
			let k = ix((self.recent_i - 1 + RECENT_I) % RECENT_I);
			self.recent[k].used = 0;
			self.recent[k].rej += m as i32;
			return;
		}
		for i in 0..NS {
			for a in 0..m {
				K[i * m + a] = if i == ILS && !ls_active { 0.0 } else { X[a * NS + i] };
			}
		}
		let mut dx = [0.0; NS];
		for i in 0..NS {
			let mut s = 0.0;
			for a in 0..m {
				s += K[i * m + a] * r[idx[a]];
			}
			dx[i] = s;
		}
		if self.s.verbose >= 2 && h.recording() {
			// TRACK_UPDATE t ch dv(3) dp(3) dth(3) world frame (m/s, m, rad), q(4) before: what this sweep corrected
			h.record(
				c"%s TRACK_UPDATE %.4f %d %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f %.5f %.5f %.5f %.5f\r\n",
				(
					h.codename(),
					sw_t,
					h.bsd_mode(lh),
					dx[IV],
					dx[IV + 1],
					dx[IV + 2],
					dx[IP],
					dx[IP + 1],
					dx[IP + 2],
					dx[IQ],
					dx[IQ + 1],
					dx[IQ + 2],
					self.q[0],
					self.q[1],
					self.q[2],
					self.q[3],
				),
			);
		}
		for i in 0..3 {
			self.p[i] += dx[IP + i];
			self.v[i] += dx[IV + i];
			self.bg[i] += dx[IBG + i];
			self.ba[i] += dx[IBA + i];
			self.ob[i] += dx[IOB + i];
		}
		for i in 0..2 {
			self.gt[i] += dx[IGT + i];
		}
		self.gravity();
		// (exp of the orientation correction once, for the state and every history entry)
		let edq = qexp(&v3(&dx[IQ..]));
		qrot_left_by(&mut self.q, &edq);
		self.corrected_by_light(&v3(&dx[IP..]), &edq);
		// the same correction to the history, so a later sweep evaluated against an older entry does not see and correct it
		// again: the correction is the current state's, and an older entry's position takes the velocity's part back to its
		// own time (STALE_HIST; before, the error was taken as constant over the samples it spans)
		for j in 0..ix(self.hist_n) {
			let back = if STALE_HIST && hv { self.t - self.hist[j].t } else { 0.0 };
			let h = &mut self.hist[j];
			for i in 0..3 {
				h.p[i] += dx[IP + i] - dx[IV + i] * back;
				h.v[i] += dx[IV + i];
				h.w[i] -= dx[IBG + i];
			}
			qrot_left_by(&mut h.q, &edq);
		}
		if ls_active {
			self.ls += dx[ILS];
		}
		if self.ls > self.s.ls_max {
			self.ls = self.s.ls_max; // a wrong scale is a wrong depth with one station (-7.8 % was learned once from a reacquisition)
		}
		if self.ls < -self.s.ls_max {
			self.ls = -self.s.ls_max;
		}
		if use_pq {
			// P = P - K C^T - C K^T + K S K^T with C = (P - Q) H^T (PHt) and S its innovation covariance: the Joseph form for
			// a gain that need not be the optimal one (the layout scale's row may be zero) with the measurement's covariance
			// with the state as the late light has it. Equal to the Joseph form below when Q = 0
			for i in 0..NS {
				for a in 0..m {
					let mut s = 0.0;
					for b2 in 0..m {
						s += K[i * m + b2] * S0[b2 * m + a];
					}
					KS[i * m + a] = s;
				}
			}
			for i in 0..NS {
				for j in 0..NS {
					let mut s = self.P[i * NS + j];
					for a in 0..m {
						s += -K[i * m + a] * PHt[j * m + a] - PHt[i * m + a] * K[j * m + a]
							+ KS[i * m + a] * K[j * m + a];
					}
					Pn[i * NS + j] = s;
				}
			}
			for i in 0..NS {
				for j in 0..NS {
					self.P[i * NS + j] = 0.5 * (Pn[i * NS + j] + Pn[j * NS + i]);
				}
			}
			return;
		}
		// P = (I - K H) P (I - K H)^T + s2 K K^T
		for i in 0..NS {
			for j in 0..NS {
				let mut s = if i == j { 1.0 } else { 0.0 };
				for a in 0..m {
					s -= K[i * m + a] * H[idx[a] * NS + j];
				}
				IKH[i * NS + j] = s;
			}
		}
		mul_ikj(IKH, &self.P, T);
		// the noise terms' factors of a row, once a row rather than for every element (the same products and the same
		// sums): sk2 K, and K's row sums with sc2 times them
		let mut ksum = [0.0; NS];
		let mut ksc = [0.0; NS];
		for i in 0..NS {
			for a in 0..m {
				KW[i * m + a] = sk2[idx[a]] * K[i * m + a];
			}
			if sc2 > 0.0 {
				let mut ki = 0.0;
				for a in 0..m {
					ki += K[i * m + a];
				}
				ksum[i] = ki;
				ksc[i] = sc2 * ki;
			}
		}
		// ... and the station plane's size error along the scale's column (LS_STATION_SD): K h, h the scale's column
		let mut kh = [0.0; NS];
		if sb2 > 0.0 {
			for i in 0..NS {
				let mut ki = 0.0;
				for a in 0..m {
					ki += K[i * m + a] * H[idx[a] * NS + ILS];
				}
				kh[i] = ki;
			}
		}
		transpose(IKH, Mt);
		mul_ikj(T, Mt, Pn);
		for i in 0..NS {
			for j in 0..NS {
				let mut s = Pn[i * NS + j];
				for a in 0..m {
					s += KW[i * m + a] * K[j * m + a];
				}
				if sc2 > 0.0 {
					s += ksc[i] * ksum[j];
				}
				if sb2 > 0.0 {
					s += sb2 * kh[i] * kh[j];
				}
				Pn[i * NS + j] = s;
			}
		}
		// keep it symmetric
		for i in 0..NS {
			for j in 0..NS {
				self.P[i * NS + j] = 0.5 * (Pn[i * NS + j] + Pn[j * NS + i]);
			}
		}
	}

	// ---- hits
	/// Every gen2 sweep hit (after the reflection gate, before the activations filter)
	pub fn integrate_hit<H: Host>(
		&mut self, h: &mut H, timecode: u64, lh: c_int, plane: c_int, sensor: c_int, angle: f64,
	) {
		if !self.had_imu {
			return; // the sweeps are grouped and closed on the IMU clock
		}
		inc(&mut self.stats.hits);
		let t = timecode as f64 / h.timebase_hz() as f64;
		if sensor < 0 || sensor >= h.sensor_ct() || plane < 0 || plane > 1 {
			return;
		}
		// how far the IMU stream runs ahead of the light (the reports of a hit come ~5-15 ms after the IMU sample of its
		// time): a slowly decaying maximum, so a sweep is not closed by an IMU sample while its hits are still coming
		{
			let lag = self.t - t;
			self.light_lag = c_fmax(lag, self.light_lag * 0.999);
		}
		// the open sweep continues while the station and plane match and the hit is within 1.5 ms of its span (the
		// reports deliver a sweep's hits slightly out of time order)
		let open = if self.sw_n != 0 { Some(ix(self.sw_n - 1)) } else { None };
		let continues = match open {
			Some(o) => {
				let sw = &self.sw[o];
				!(sw.lh != lh
					|| sw.plane != plane
					|| t > sw.t_max + 1.5e-3
					|| t < sw.t_min - 1.5e-3
					|| sw.n >= HMAX as i32)
			}
			None => false,
		};
		let s = if continues {
			ix(self.sw_n - 1)
		} else {
			if self.sw_n == SWMAX as i32 {
				// no IMU sample closed the pending sweeps: drop the oldest
				self.sw.copy_within(1..SWMAX, 0);
				self.sw_n -= 1;
				inc(&mut self.stats.sweeps_dropped);
			}
			let s = ix(self.sw_n);
			self.sw_n += 1;
			let sw = &mut self.sw[s];
			sw.lh = lh;
			sw.plane = plane;
			sw.n = 0;
			sw.t_min = t;
			sw.t_max = t;
			inc(&mut self.stats.sweeps);
			s
		};
		let sw = &mut self.sw[s];
		if t < sw.t_min {
			sw.t_min = t;
		}
		if t > sw.t_max {
			sw.t_max = t;
		}
		sw.t[ix(sw.n)] = t;
		sw.angle[ix(sw.n)] = angle;
		sw.sensor[ix(sw.n)] = sensor;
		sw.n += 1;
	}

	// ---- IMU samples: close and apply the pending sweeps, propagate, report
	// The hand-over look-ahead: the state is on the gyro's time, but the accelerometer's newest samples belong up to
	// ACC_LEAD_* past it (see there). Integrated forward from the state (the orientation turned by the body rate, constant
	// over the few ms; bias and gravity out) they say where the device went in that time: measured, not predicted. The
	// runtime hands this to the SteamVR driver with the Q line, which declares the pose that much younger (and carries the
	// rest of its age with ho_a, the newest sample's acceleration). Nothing here feeds back into the state. Not while
	// coasting (the accelerometer does not drive the velocity then) or with the gyro at full scale
	fn lookahead<H: Host>(&mut self, h: &mut H) {
		let lead = self.acc_lead(h);
		let wb = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
		let R = q2R(&self.q);
		self.ho_w = Rv(&R, &wb);
		self.ho_al = Rv(&R, &self.al_b); // (d/dt of the world rate R w_b is R dw_b/dt)
		self.ho_a = self.a_now;
		self.ho_dp = [0.0; 3];
		self.ho_dv = [0.0; 3];
		self.ho_lead = 0.0;
		if lead <= 0.0 || self.coasting || self.gyro_clipped || self.accq_n < 2 {
			return;
		}
		// the points: s = 0 is the state (the accelerometer as the propagation took it, interpolated at ti - lead), then
		// every kept sample newer than ti - lead, at s = its time - (ti - lead)
		let ti = self.t;
		let mut s_ = [0.0; ACCQN + 1];
		let mut a_ = [[0.0; 3]; ACCQN + 1];
		let mut n = 0;
		s_[n] = 0.0;
		a_[n] = self.acc_use;
		n += 1;
		let mut u = self.accq_n - 1;
		while u >= 0 {
			// oldest first
			let i = ix((self.accq_i - 1 - u + 2 * ACCQN_I) % ACCQN_I);
			u -= 1;
			let s = self.accq_t[i] - (ti - lead);
			if s <= 1e-6 || s > lead + 1e-6 {
				continue;
			}
			if s - s_[n - 1] > 0.05 {
				return; // a gap: nothing to carry
			}
			s_[n] = s;
			a_[n] = self.accq[i];
			n += 1;
		}
		if n < 2 {
			return;
		}
		let mut aw = [[0.0; 3]; ACCQN + 1];
		for j in 0..n {
			let mut am = [a_[j][0] - self.ba[0], a_[j][1] - self.ba[1], a_[j][2] - self.ba[2]];
			self.clip_add(h, &a_[j], &mut am); // as the state takes it
			let mut qj = self.q;
			qrot_right(&mut qj, &wb, s_[j]);
			let Rj = q2R(&qj);
			aw[j] = Rv(&Rj, &am);
			for i in 0..3 {
				aw[j][i] -= self.gvec[i];
			}
		}
		let mut dv = [0.0, 0.0, 0.0];
		let mut dp = [0.0, 0.0, 0.0];
		for j in 0..n - 1 {
			let h = s_[j + 1] - s_[j];
			for k in 0..3 {
				dp[k] += (self.v[k] + dv[k]) * h + (2.0 * aw[j][k] + aw[j + 1][k]) / 6. * h * h; // exact for a linear acceleration
				dv[k] += 0.5 * (aw[j][k] + aw[j + 1][k]) * h;
			}
		}
		self.ho_lead = s_[n - 1];
		self.ho_dp = dp;
		self.ho_dv = dv;
		self.ho_a = aw[n - 1];
	}

	/// The pose for the imupose hook and the velocity for the velocity hook, which the host invokes with them (the
	/// hooks can re-enter the read-only API: nothing of the tracker is borrowed while they run)
	fn report<H: Host>(&mut self, h: &mut H) -> Option<(SurvivePose, SurviveVelocity)> {
		let pose = SurvivePose { pos: self.p, rot: self.q };
		// the imupose hook takes the IMU point: the state point
		for x in pose.pos.iter().chain(pose.rot.iter()) {
			if !x.is_finite() {
				return None;
			}
		}
		inc(&mut self.stats.reports);
		if self.s.verbose >= 2 && h.recording() && self.ret_t > -1e8 && self.t - self.ret_t <= RETURN_RECORD {
			// TRACK_RETURN t since dark dp(3) dth(3): after the light's return (RETURN_DARK), this report's correction of the
			// IMU point (m, world) and of the orientation (rad, world side), how long ago the return began and how long
			// the device was dark before it: what the output layer blends, for the bench
			let e = qlog(&qmul(&self.corr_q, &qconj(&self.corr_rep_q)));
			h.record(
				c"%s TRACK_RETURN %.4f %.4f %.3f %+.6f %+.6f %+.6f %+.6f %+.6f %+.6f\r\n",
				(
					h.codename(),
					self.t,
					self.t - self.ret_t,
					self.ret_gap,
					self.corr_p[0] - self.corr_rep_p[0],
					self.corr_p[1] - self.corr_rep_p[1],
					self.corr_p[2] - self.corr_rep_p[2],
					e[0],
					e[1],
					e[2],
				),
			);
		}
		self.corr_rep_p = self.corr_p;
		self.corr_rep_q = self.corr_q;
		self.lookahead(h); // before the hooks: the runtime's Q line takes it with the velocity
					 // (the imupose hook sets so->OutPoseIMU: the device's pose for the reflection gate and the scene solver)
		let mut vel = SurviveVelocity::default();
		vel.pos = self.v;
		let wm = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
		let R = q2R(&self.q);
		vel.axis_angle_rot = Rv(&R, &wm);
		// the velocity hook carries the IMU point's velocity: the state's
		Some((pose, vel))
	}

	/// Every calibrated IMU sample (g, rad/s), in device time. Returns the report (the imupose hook's pose, the
	/// velocity hook's velocity) when there is one for the host's hooks
	pub fn integrate_imu<H: Host>(
		&mut self, h: &mut H, timecode: u64, accel: &[f64; 3], gyro: &[f64; 3],
	) -> Option<(SurvivePose, SurviveVelocity)> {
		let ti = timecode as f64 / h.timebase_hz() as f64;
		if self.had_imu && ti < self.t - CLOCK_BACK {
			// the device's clock went back: it restarted. Nothing buffered belongs to the new time line: with the
			// state's history and the pending sweeps from the old one, a few seconds of updates break the covariance
			// ("not positive definite") and run the calibration states off. Lost now, every buffer emptied, every time
			// mark moved before the new clock
			if self.s.verbose >= 1 {
				h.sv_info(
					c"%s track: the device's clock went back %.3f s at %.3f s (the device restarted): lost, starting over",
					(h.codename(), self.t - ti, ti),
				);
			}
			self.seeded = false;
			self.want_reseed = false;
			self.coasting = false;
			self.lost_warm = false;
			self.confirmed = false;
			self.p = [0.0; 3];
			self.v = [0.0; 3];
			self.hist_n = 0;
			self.hist_i = 0;
			self.seed_n = 0;
			self.seed_i = 0;
			self.sw_n = 0;
			self.recent_n = 0;
			self.recent_i = 0;
			self.accq_n = 0;
			self.accq_i = 0;
			self.gyr_hist_n = 0;
			self.gyr_hist_i = 0;
			self.have_prev = false;
			self.rest_since = 0.0;
			self.hold_since = 0.0;
			self.holding = false;
			let past = ti - 1e3;
			self.first_t = ti - (self.t - self.first_t); // the time tracked so far kept
			self.t = ti;
			self.last_imu_t = ti;
			self.seed_t = past;
			self.seed_try_t = past;
			self.disagree_t = past;
			self.tie_t = past;
			self.last_light_t = past;
			self.sweep_in_t = past;
			self.ret_armed = false;
			self.ret_t = -1e9;
			if self.gyro_clip_t > 0.0 {
				self.gyro_clip_t = past;
			}
			for i in 0..NLH {
				self.last_sweep_t[i] = past;
				self.light_since[i] = past;
				self.last_used_t[i] = past;
			}
		}
		self.clipped = false;
		for i in 0..3 {
			self.last_acc[i] = accel[i] * GRAV;
			self.last_gyr[i] = gyro[i];
			if accel[i].abs() > CLIP_G {
				self.clipped = true; // at full scale (8 g)
			}
		}
		if self.clipped && !self.pin_logged && self.s.verbose >= 1 && self.is_controller(h) {
			self.pin_logged = true;
			h.sv_info(
				c"%s track: accelerometer beyond %.1f g at %.3f s (%+.3f %+.3f %+.3f g); pinned: %d %d %d (raw scale %g)",
				(
					h.codename(),
					CLIP_G,
					ti,
					accel[0],
					accel[1],
					accel[2],
					self.pinned(h, 0, accel[0] * GRAV) as i32,
					self.pinned(h, 1, accel[1] * GRAV) as i32,
					self.pinned(h, 2, accel[2] * GRAV) as i32,
					h.raw_acc_scale(),
				),
			);
		}
		if self.clipped {
			inc(&mut self.stats.clipped);
		}
		{
			// the gyro at full scale (the device's range: 2000 dps on the trackers and controllers)
			let fs = if h.raw_gyro_scale() > 0.0 { h.raw_gyro_scale() * 32767.0 } else { 34.9 };
			self.gyro_clipped = false;
			for i in 0..3 {
				if gyro[i].abs() > 0.97 * fs {
					self.gyro_clipped = true;
				}
			}
			if self.gyro_clipped {
				self.gyro_clip_t = ti;
			}
		}
		// a gyro axis that has stopped reporting: constant to STUCK_FLAT over STUCKN samples while the other two axes
		// are fast and the direction of their rate turns by more than 90 degrees (a rigid turn about one fixed axis
		// keeps an axis constant for real). Seen on a controller after a hard impact: one axis flat for 0.43 s while
		// the other two swung +-8 rad/s; the orientation integrated wrong, gravity leaked into the velocity and the
		// device was lost. Handled as a saturated gyro (the orientation is the light's, the accelerometer does not
		// drive the velocity) until the axis reads differently
		self.gyr_hist[ix(self.gyr_hist_i)] = self.last_gyr;
		{
			// the test's x[o1].hypot(x[o2]) of this sample for each axis, worked out once rather than at every use (up to 136
			// times an axis while one turns and two sit flat)
			let g = &self.last_gyr;
			self.gyr_mag[ix(self.gyr_hist_i)] = [g[1].hypot(g[2]), g[2].hypot(g[0]), g[0].hypot(g[1])];
		}
		self.gyr_hist_i = (self.gyr_hist_i + 1) % STUCKN_I;
		if self.gyr_hist_n < STUCKN_I {
			self.gyr_hist_n += 1;
		}
		self.gyro_stuck = false;
		if self.stuck_ax >= 0 {
			// latched: until the axis reads differently again
			if (self.last_gyr[ix(self.stuck_ax)] - self.stuck_val).abs() > STUCK_RELEASE {
				self.stuck_ax = -1;
			} else {
				self.gyro_stuck = true;
			}
		}
		if self.stuck_ax < 0 && self.gyr_hist_n == STUCKN_I {
			let mut ax = 0;
			while ax < 3 && !self.gyro_stuck {
				let o1 = (ax + 1) % 3;
				let o2 = (ax + 2) % 3;
				let mut flat = true;
				let mut fast = false;
				let mut mind = 1.0;
				let mut u = 0;
				while u < STUCKN_I && flat {
					let xi = ix((self.gyr_hist_i - 1 - u + 2 * STUCKN_I) % STUCKN_I);
					let x = &self.gyr_hist[xi];
					let y = &self.gyr_hist[ix((self.gyr_hist_i - 2 - u + 2 * STUCKN_I) % STUCKN_I)];
					if u < STUCKN_I - 1 && (x[ax] - y[ax]).abs() > STUCK_FLAT {
						flat = false;
					}
					let m = self.gyr_mag[xi][ax]; // x[o1].hypot(x[o2])
					if m > STUCK_FAST {
						fast = true;
					}
					if m > 1.0 {
						// the direction of the other two axes' rate against every earlier sample in the window
						for w in u + 1..STUCKN_I {
							let zi = ix((self.gyr_hist_i - 1 - w + 2 * STUCKN_I) % STUCKN_I);
							let z = &self.gyr_hist[zi];
							let mz = self.gyr_mag[zi][ax]; // z[o1].hypot(z[o2])
							if mz > 1.0 {
								let d = (x[o1] * z[o1] + x[o2] * z[o2]) / (m * mz);
								if d < mind {
									mind = d;
								}
							}
						}
					}
					u += 1;
				}
				if flat && fast && mind < 0.0 {
					// turned by more than 90 degrees
					self.gyro_stuck = true;
					self.stuck_ax = ax as i32;
					self.stuck_val = self.last_gyr[ax];
					if self.s.verbose >= 1 {
						h.sv_info(
							c"%s track: gyro axis %d stopped reporting at %.3f s (%+.3f rad/s while the others turn): the orientation is the light's until it moves",
							(h.codename(), ax as i32, ti, self.stuck_val),
						);
					}
				}
				ax += 1;
			}
		}
		if self.gyro_stuck {
			// handled as a saturated gyro
			self.gyro_clipped = true;
			self.gyro_clip_t = ti;
			if self.s.verbose >= 2 && h.recording() {
				h.record(
					c"%s TRACK_STUCK %.4f %d %+.4f\r\n",
					(h.codename(), ti, self.stuck_ax, self.last_gyr[ix(self.stuck_ax)]),
				);
			}
		}
		// the angular acceleration for the hand-over (lookahead, the runtime's Q line): the body rate's change since the
		// previous sample (the bias drops out). Measured, like the accelerometer's newest sample the hand-over carries: 0
		// across a gap or with either sample at full scale (the rate beyond it is not known). Nothing here feeds the state
		self.al_b = [0.0; 3];
		if HANDOVER_ALPHA
			&& self.had_imu
			&& self.gyr_hist_n >= 2
			&& ti > self.last_imu_t
			&& ti - self.last_imu_t < 0.05
			&& !self.gyro_clipped
			&& !self.gyr_prev_clip
		{
			let g0 = self.gyr_hist[ix((self.gyr_hist_i - 2 + 2 * STUCKN_I) % STUCKN_I)];
			for i in 0..3 {
				self.al_b[i] = (self.last_gyr[i] - g0[i]) / (ti - self.last_imu_t);
			}
		}
		self.gyr_prev_clip = self.gyro_clipped;
		// the accelerometer is taken as it is: the state point is its own (see the top of the file)
		{
			let w = [self.last_gyr[0], self.last_gyr[1], self.last_gyr[2]];
			self.accq_t[ix(self.accq_i)] = ti;
			self.accq[ix(self.accq_i)] = self.last_acc;
			self.accq_clip[ix(self.accq_i)] = self.clipped;
			self.accq_i = (self.accq_i + 1) % ACCQN_I;
			if self.accq_n < ACCQN_I {
				self.accq_n += 1;
			}
			self.acc_use = self.last_acc;
			let ashift = self.acc_lead(h);
			if ashift > 0.0 && self.accq_n >= 2 {
				// the accelerometer at the gyro's time (ACC_LEAD_*): interpolated between the kept samples around
				// ti - lead (newest pair first); across a gap in the samples the newest is taken as it is
				let ts = ti - ashift;
				let mut u = 0;
				while u + 1 < self.accq_n {
					let i1 = ix((self.accq_i - 1 - u + 2 * ACCQN_I) % ACCQN_I);
					let i0 = ix((self.accq_i - 2 - u + 2 * ACCQN_I) % ACCQN_I);
					let t1 = self.accq_t[i1];
					let t0 = self.accq_t[i0];
					if t1 <= t0 || t1 - t0 > 0.05 {
						break;
					}
					if t0 <= ts || u + 2 == self.accq_n {
						let mut f = (ts - t0) / (t1 - t0);
						f = if f < 0.0 {
							0.0
						} else if f > 1.0 {
							1.0
						} else {
							f
						};
						for k in 0..3 {
							self.acc_use[k] = self.accq[i0][k] + f * (self.accq[i1][k] - self.accq[i0][k]);
						}
						self.clipped = (f < 1.0 && self.accq_clip[i0]) || (f > 0.0 && self.accq_clip[i1]);
						break;
					}
					u += 1;
				}
			}
			if self.had_imu && ti > self.last_imu_t && ti - self.last_imu_t < 0.1 {
				let wb = [w[0] - self.bg[0], w[1] - self.bg[1], w[2] - self.bg[2]];
				self.rot_cum += norm3d(&wb) * (ti - self.last_imu_t); // for the first-light fade (JOIN_DEG)
			} else {
				self.have_prev = false; // no previous sample to integrate from
			}
			self.last_imu_t = ti;
		}
		if self.s.verbose >= 3 && h.recording() {
			h.record(
				c"%s TRACK_IMU %.5f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.3f\r\n",
				(
					h.codename(),
					ti,
					self.last_acc[0],
					self.last_acc[1],
					self.last_acc[2],
					self.acc_use[0],
					self.acc_use[1],
					self.acc_use[2],
					self.last_gyr[0],
					self.last_gyr[1],
					self.last_gyr[2],
					self.vib,
				),
			);
		}
		// the vibration: rms of the sample-to-sample accelerometer difference (/ sqrt 2) over the last 10 samples,
		// scaled (x0.65) to the centred-window measure the adapt setting was tuned on. Causal and immediate: it reacts
		// within one sample. On the accelerometer as it is, the signal the state takes: moved to the head point it
		// would carry the differenced-gyro angular acceleration, which on a shaken controller (head 16 cm from the IMU)
		// reads as vibration and turns the accelerometer off when it carries the motion
		self.acc_hist[ix(self.acc_hist_i)] = self.last_acc;
		self.acc_hist_i = (self.acc_hist_i + 1) % VIBN_I;
		if self.acc_hist_n < VIBN_I {
			self.acc_hist_n += 1;
		}
		if self.acc_hist_n >= 11 {
			let mut s = 0.0;
			for u in 0..10 {
				let x = &self.acc_hist[ix((self.acc_hist_i - 1 - u + 4 * VIBN_I) % VIBN_I)];
				let y = &self.acc_hist[ix((self.acc_hist_i - 2 - u + 4 * VIBN_I) % VIBN_I)];
				for i in 0..3 {
					s += (x[i] - y[i]) * (x[i] - y[i]) / 2.0;
				}
			}
			self.vib = 0.65 * (s / 10.0).sqrt();
		} else {
			self.vib = 0.0;
		}
		let first = !self.had_imu;
		self.had_imu = true;
		self.imu_count = self.imu_count.wrapping_add(1);
		if first {
			self.t = ti;
		}
		// sweeps closed by this sample (no later hit can join: 1.5 ms plus the light's lag past their last hit), in time order; a sweep whose
		// shifted time is still ahead of this sample waits for the next one. Every closed sweep goes into the seed buffer
		// with the orientation the gyro integration had at its time; while the state is seeded it is applied as an update,
		// otherwise it feeds the start solve
		let mut applied: i32 = 0;
		let mut s = 0;
		while s < self.sw_n {
			// (a copy: the update does not change the pending sweeps, a seed only empties them)
			let sw = self.sw[ix(s)];
			let closed = (s < self.sw_n - 1) || (ti >= sw.t_max + 1.5e-3 + c_fmax(0., self.light_lag));
			if !closed {
				break;
			}
			let mut sw_t = 0.0;
			for k in 0..sw.n as usize {
				sw_t += sw.t[k];
			}
			sw_t = sw_t / sw.n as f64 + self.s.hit_shift;
			if sw_t > ti {
				break;
			}
			let dt = sw_t - self.t;
			if dt < 0.0 {
				inc(&mut self.stats.stale); // the state is already past the sweep (its hits arrived after the IMU samples of their time): evaluated against the history
			}
			if dt > 0.0 && dt < 0.1 {
				self.propagate_state(h, dt);
				self.t = sw_t;
			} else if dt >= 0.1 {
				self.t = sw_t;
			}
			self.last_stale = 0.0;
			if self.blink_drop(h, sw.lh, sw_t) {
				// the handover test: this station is out for now, as if it could not see the device
				applied += 1;
				s += 1;
				continue;
			}
			{
				let b = self.base_at(sw_t);
				if b.t >= 0.0 {
					self.seed_keep(&sw, sw_t, &b);
				}
			}
			if self.seeded {
				self.update(h, &sw, sw_t);
				if self.seeded {
					self.confirm(h, sw_t);
				}
			} else if sw_t - self.seed_try_t >= 0.02 {
				self.seed_solve(h, sw_t, false, 0, 0);
			}
			if self.sw_n == 0 {
				// a seed inside the update cleared the pending sweeps
				applied = 0;
				break;
			}
			applied += 1;
			s += 1;
		}
		if applied != 0 {
			self.sw.copy_within(ix(applied)..ix(self.sw_n), 0);
			self.sw_n -= applied;
		}
		let dt = ti - self.t;
		self.step_qv = 0.0;
		self.step_qob = 0.0;
		self.step_qth = [0.0; 9];
		self.step_kob = 1.0;
		if dt > 0.0 && dt < 0.1 {
			let R = q2R(&self.q);
			let am = [self.acc_use[0] - self.ba[0], self.acc_use[1] - self.ba[1], self.acc_use[2] - self.ba[2]];
			self.propagate_state(h, dt);
			if self.seeded {
				self.propagate_P(h, dt, &R, &am);
			}
		}
		self.t = ti;
		self.a_prev = self.acc_use;
		self.g_prev = self.last_gyr;
		self.have_prev = true;
		if self.seeded {
			self.confirm(h, ti); // on the clock too: with the light gone again right after a restart, the decision would wait for the next sweep
		}
		if !self.seeded {
			// not started (or lost beyond recovery by the solve so far): only the orientation integrates the gyro, for the
			// seed solve's window; position and velocity mean nothing until the solve sets them. One that is not finite
			// starts over here as at the run-away below: carried on, every buffered sweep's orientation was NaN and no
			// solve could start the device again
			if !finite(&self.q) {
				self.q = [1.0, 0.0, 0.0, 0.0];
			}
			self.p = [0.0; 3];
			self.v = [0.0; 3];
			self.hist_push(ti);
			return None;
		}
		// at rest (the gyro and the accelerometer's fast part quiet for track-zupt-time s): a zero-velocity measurement
		// (sd track-zupt-sd). The one thing that holds the depth with one station when the device is not moving (a foot
		// on the floor, a waist standing still)
		if self.s.zupt_sd > 0.0 {
			let w = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			let quiet = norm3d(&w) < self.s.zupt_gyro && self.vib < self.s.zupt_vib && !self.clipped;
			self.rest_since = if quiet {
				if self.rest_since > 0.0 {
					self.rest_since
				} else {
					ti
				}
			} else {
				0.0
			};
			// ... and near rest, where the light does not see: the velocity along that line held (HOLD_*)
			let mut line = None;
			if norm3d(&w) < HOLD_GYRO
				&& self.vib < self.s.zupt_vib
				&& !self.clipped
				&& ti - self.moving_t >= DR_WIN
				&& ti - self.last_light_t < HOLD_LIT
			{
				if let Some(u) = self.blind_line() {
					let along = self.v[0] * u[0] + self.v[1] * u[1] + self.v[2] * u[2];
					let across = [self.v[0] - along * u[0], self.v[1] - along * u[1], self.v[2] - along * u[2]];
					if norm3d(&across) < HOLD_ACROSS && (self.holding || self.hold_sd >= HOLD_SD) {
						line = Some((u, along));
					}
				}
			}
			self.hold_since = match line {
				Some(_) if self.hold_since > 0.0 => self.hold_since,
				Some(_) => ti,
				None => 0.0,
			};
			let was_holding = self.holding;
			self.holding = false;
			if self.rest_since > 0.0 && ti - self.rest_since >= self.s.zupt_time {
				inc(&mut self.stats.zupts);
				self.zupt();
			} else if let Some((u, along)) = line {
				if ti - self.hold_since >= self.s.zupt_time {
					if !was_holding && self.s.verbose >= 1 {
						h.sv_info(
							c"%s track: held along the line the light does not see from %.3f s (the velocity along it %.3f m/s, the position's sd %.1f mm)",
							(h.codename(), ti, along, 1e3 * self.hold_sd),
						);
					}
					self.holding = true;
					self.velocity_along(&u, 0.0, self.s.zupt_sd);
				}
			}
		}
		self.hist_push(ti);
		// a state that is not finite or far outside any room: lost at once, the reports stop until the seed solve has a
		// new start (the orientation keeps integrating the gyro; a non-finite one starts over)
		{
			let mut bad = false;
			for i in 0..3 {
				bad |= !self.p[i].is_finite()
					|| self.p[i].abs() > 20.0
					|| !self.v[i].is_finite()
					|| self.v[i].abs() > 50.0;
			}
			let mut qbad = false;
			for i in 0..4 {
				qbad |= !self.q[i].is_finite();
			}
			if bad || qbad {
				if self.s.verbose >= 1 {
					h.sv_info(
						c"%s track ran away at %.3f s (%.1f %.1f %.1f m): lost",
						(h.codename(), ti, self.p[0], self.p[1], self.p[2]),
					);
				}
				self.seeded = false;
				self.want_reseed = false;
				self.p = [0.0; 3];
				self.v = [0.0; 3];
				if qbad {
					self.q = [1.0, 0.0, 0.0, 0.0];
				}
				// ... and what the next start would take over that is not finite starts over now, not at that start
				// (reset_P's reset comes only with a solve, and none could succeed): the gyro's integration while not
				// seeded runs on bg and on the previous sample, and NaN in either makes the orientation NaN again at
				// the next sample, every buffered sweep's with it; a NaN layout scale is in every prediction; a NaN
				// tilt is in gravity, and every start runs away at its first sample. Without this, one NaN gyro sample
				// (or one NaN accelerometer sample while coasting) and the device never starts again
				for i in 0..3 {
					if !self.bg[i].is_finite() {
						self.bg[i] = 0.0;
					}
					if !self.ba[i].is_finite() {
						self.ba[i] = 0.0;
					}
					if !self.ob[i].is_finite() {
						self.ob[i] = 0.0;
					}
				}
				if !self.ls.is_finite() {
					self.ls = 0.0;
				}
				if !finite(&self.gt) {
					self.gt = [self.gt0x, self.gt0y];
					self.gravity();
				}
				if !(finite(&self.g_prev) && finite(&self.a_prev)) {
					self.have_prev = false;
				}
				self.hist_n = 0;
				self.hist_i = 0;
				self.seed_n = 0;
				self.seed_i = 0;
				self.sw_n = 0;
				return None;
			}
		}
		if self.s.verbose >= 2 && h.recording() && self.imu_count % 250 == 0 {
			h.record(
				c"%s TRACK_BIAS %.3f %+.4f %+.4f %+.4f %+.3f %+.3f %+.3f %+.3f %.3f %.3f %.3f %.3f\r\n",
				(
					h.codename(),
					ti,
					self.bg[0],
					self.bg[1],
					self.bg[2],
					self.ba[0],
					self.ba[1],
					self.ba[2],
					self.ls * 100.0,
					(self.P[0] + self.P[NS + 1] + self.P[2 * NS + 2]).sqrt() * 1e3,
					(self.P[IV * NS + IV] + self.P[(IV + 1) * NS + IV + 1] + self.P[(IV + 2) * NS + IV + 2]).sqrt(),
					(self.P[IBA * NS + IBA] + self.P[(IBA + 1) * NS + IBA + 1] + self.P[(IBA + 2) * NS + IBA + 2])
						.sqrt(),
					self.P[ILS * NS + ILS].sqrt() * 100.0,
				),
			);
		}
		// divergence: of the last 0.5 s of sweeps, more than reseed-frac of the hits rejected with at least reseed-min-hits
		if self.imu_count % 125 == 0 && self.recent_n != 0 {
			let (mut used, mut rej): (i32, i32) = (0, 0);
			for k in 0..ix(self.recent_n) {
				if self.recent[k].t > ti - 0.5 {
					used = used.wrapping_add(self.recent[k].used);
					rej = rej.wrapping_add(self.recent[k].rej);
				}
			}
			if used + rej >= self.s.reseed_min_hits
				&& rej as f64 > self.s.reseed_frac * (used + rej) as f64
				&& !self.want_reseed
			{
				self.want_reseed = true; // lost: the seed solve restarts the state at the next sweeps it fits
				self.lost_warm = false; // a divergence: nothing of the state is trusted, the restart is cold
				if self.s.verbose >= 1 {
					h.sv_info(
						c"%s track diverged at %.3f s (%d of %d hits of the last 0.5 s rejected): lost",
						(h.codename(), ti, rej, used + rej),
					);
				}
			}
		}
		self.report(h)
	}

	/// The state after this sample into the history
	fn hist_push(&mut self, ti: f64) {
		let h = &mut self.hist[ix(self.hist_i)];
		h.t = ti;
		h.p = self.p;
		h.v = self.v;
		h.q = self.q;
		h.dr_p = self.dr_p;
		h.dr_v = self.dr_v;
		for i in 0..3 {
			h.w[i] = self.last_gyr[i] - self.bg[i];
		}
		h.nq_v = self.step_qv;
		h.nq_th = self.step_qth;
		h.nq_ob = self.step_qob;
		h.kob = self.step_kob;
		self.hist_i = (self.hist_i + 1) % HISTN_I;
		if self.hist_n < HISTN_I {
			self.hist_n += 1;
		}
	}

	/// the rest test's measurements (integrate_imu's at-rest block): zero velocity, then zero rotation
	fn zupt(&mut self) {
		let mut w = self.take_work();
		self.zupt_in(&mut w);
		self.work = Some(w);
	}
	fn zupt_in(&mut self, w: &mut Work) {
		let Work { IKH, T, Pn, Mt, .. } = w;
		// K = P H^T (H P H^T + R)^-1 with H = the velocity rows
		let mut S = [0.0; 9];
		let mut X = [0.0; 3 * NS];
		for a in 0..3 {
			for b in 0..3 {
				S[a * 3 + b] =
					self.P[(IV + a) * NS + IV + b] + (if a == b { self.s.zupt_sd * self.s.zupt_sd } else { 0.0 });
			}
		}
		for a in 0..3 {
			for i in 0..NS {
				X[a * NS + i] = self.P[i * NS + IV + a]; // (P H^T)^T rows
			}
		}
		if chol_solve(&mut S, 3, &mut X, NS) {
			let mut K = [0.0; NS * 3];
			let mut dx = [0.0; NS];
			for i in 0..NS {
				for a in 0..3 {
					K[i * 3 + a] = X[a * NS + i];
				}
			}
			for i in 0..NS {
				let mut acc = 0.0;
				for a in 0..3 {
					acc += K[i * 3 + a] * (0.0 - self.v[a]);
				}
				dx[i] = if i == ILS { 0.0 } else { acc };
			}
			self.apply_dx(&dx);
			for i in 0..NS {
				for j in 0..NS {
					IKH[i * NS + j] = (if i == j { 1.0 } else { 0.0 })
						- (if (IV..IV + 3).contains(&j) && i != ILS { K[i * 3 + (j - IV)] } else { 0.0 });
				}
			}
			mul_ikj(IKH, &self.P, T);
			// (the noise term's first product once a row: the same products as for every element)
			let mut KR = [0.0; NS * 3];
			for i in 0..NS * 3 {
				KR[i] = self.s.zupt_sd * self.s.zupt_sd * K[i];
			}
			transpose(IKH, Mt);
			mul_ikj(T, Mt, Pn);
			for i in 0..NS {
				for j in 0..NS {
					let mut acc = Pn[i * NS + j];
					if i != ILS && j != ILS {
						for a in 0..3 {
							acc += KR[i * 3 + a] * K[j * 3 + a];
						}
					}
					Pn[i * NS + j] = acc;
				}
			}
			for i in 0..NS {
				for j in 0..NS {
					self.P[i * NS + j] = 0.5 * (Pn[i * NS + j] + Pn[j * NS + i]);
				}
			}
		}
		// ... and zero rotation (ZARU_SD): the residual 0 - (gyro - bg), the bias rows H = -I
		for a in 0..3 {
			for b in 0..3 {
				S[a * 3 + b] = self.P[(IBG + a) * NS + IBG + b] + (if a == b { ZARU_SD * ZARU_SD } else { 0.0 });
			}
		}
		for a in 0..3 {
			for i in 0..NS {
				X[a * NS + i] = self.P[i * NS + IBG + a]; // (P H^T)^T rows, sign below
			}
		}
		if chol_solve(&mut S, 3, &mut X, NS) {
			let mut K = [0.0; NS * 3];
			let mut dx = [0.0; NS];
			let wr = [self.last_gyr[0] - self.bg[0], self.last_gyr[1] - self.bg[1], self.last_gyr[2] - self.bg[2]];
			for i in 0..NS {
				for a in 0..3 {
					K[i * 3 + a] = -X[a * NS + i]; // K = P H^T S^-1, H = -I on the bias
				}
			}
			for i in 0..NS {
				let mut acc = 0.0;
				for a in 0..3 {
					acc += K[i * 3 + a] * (0.0 - wr[a]);
				}
				dx[i] = if i == ILS { 0.0 } else { acc };
			}
			self.apply_dx(&dx);
			// Joseph form with H = -I on the bias columns: I - K H = I + K on them
			for i in 0..NS {
				for j in 0..NS {
					IKH[i * NS + j] = (if i == j { 1.0 } else { 0.0 })
						+ (if (IBG..IBG + 3).contains(&j) && i != ILS { K[i * 3 + (j - IBG)] } else { 0.0 });
				}
			}
			mul_ikj(IKH, &self.P, T);
			let mut KR = [0.0; NS * 3];
			for i in 0..NS * 3 {
				KR[i] = ZARU_SD * ZARU_SD * K[i];
			}
			transpose(IKH, Mt);
			mul_ikj(T, Mt, Pn);
			for i in 0..NS {
				for j in 0..NS {
					let mut acc = Pn[i * NS + j];
					if i != ILS && j != ILS {
						for a in 0..3 {
							acc += KR[i * 3 + a] * K[j * 3 + a];
						}
					}
					Pn[i * NS + j] = acc;
				}
			}
			for i in 0..NS {
				for j in 0..NS {
					self.P[i * NS + j] = 0.5 * (Pn[i * NS + j] + Pn[j * NS + i]);
				}
			}
		}
	}

	/// The line along which the light places the device worst (the longest axis of the position's covariance, by
	/// power iteration from the line last found), with the position's sd along it in `hold_sd`
	fn blind_line(&mut self) -> Option<[f64; 3]> {
		let mut u = self.hold_line;
		let mut l = 0.0;
		for _ in 0..12 {
			let mut y = [0.0; 3];
			for a in 0..3 {
				for b in 0..3 {
					y[a] += self.P[(IP + a) * NS + IP + b] * u[b];
				}
			}
			l = norm3d(&y);
			if !(l > 0.0) || !l.is_finite() {
				return None;
			}
			u = [y[0] / l, y[1] / l, y[2] / l];
		}
		self.hold_line = u;
		self.hold_sd = l.sqrt();
		Some(u)
	}

	/// A measurement of the velocity along `u` (a unit vector): `rate`, sd `sd`
	fn velocity_along(&mut self, u: &[f64; 3], rate: f64, sd: f64) {
		let r = rate - (self.v[0] * u[0] + self.v[1] * u[1] + self.v[2] * u[2]);
		// K = P H^T / (H P H^T + R), H = u on the velocity
		let mut ph = [0.0; NS];
		for i in 0..NS {
			for a in 0..3 {
				ph[i] += self.P[i * NS + IV + a] * u[a];
			}
		}
		let s = ph[IV] * u[0] + ph[IV + 1] * u[1] + ph[IV + 2] * u[2] + sd * sd;
		if !(s > 0.0) || !s.is_finite() {
			return;
		}
		let mut k = [0.0; NS];
		let mut dx = [0.0; NS];
		for i in 0..NS {
			k[i] = if i == ILS { 0.0 } else { ph[i] / s };
			dx[i] = k[i] * r;
		}
		self.apply_dx(&dx);
		// P - K (H P) - (P H^T) K^T + K S K^T: the Joseph form, for a gain with the layout scale's row zero
		let mut w = self.take_work();
		{
			let Pn = &mut w.Pn;
			for i in 0..NS {
				for j in 0..NS {
					Pn[i * NS + j] = self.P[i * NS + j] - k[i] * ph[j] - ph[i] * k[j] + k[i] * s * k[j];
				}
			}
			for i in 0..NS {
				for j in 0..NS {
					self.P[i * NS + j] = 0.5 * (Pn[i * NS + j] + Pn[j * NS + i]);
				}
			}
		}
		self.work = Some(w);
	}

	/// The rest test's correction applied to the state
	fn apply_dx(&mut self, dx: &[f64; NS]) {
		for i in 0..3 {
			self.p[i] += dx[IP + i];
			self.v[i] += dx[IV + i];
			self.bg[i] += dx[IBG + i];
			self.ba[i] += dx[IBA + i];
			self.ob[i] += dx[IOB + i];
		}
		for i in 0..2 {
			self.gt[i] += dx[IGT + i];
		}
		self.gravity();
		qrot_left(&mut self.q, &v3(&dx[IQ..]));
		self.corrected(&v3(&dx[IP..]), &qexp(&v3(&dx[IQ..])));
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::host::Settings;
	use crate::text::Name;
	use sally_lighthouse::REPROJECT_AXIS_FN_GEN2;

	/// a device and its stations, as a runtime would host them, with the lines the tracker writes
	struct TestDevice {
		sensors: Vec<[f64; 3]>,
		normals: Vec<[f64; 3]>,
		controller: bool,
		stations: Vec<(SurvivePose, [BaseStationCal; 2])>,
		lines: Vec<String>,
	}

	impl Host for TestDevice {
		fn sensor_ct(&self) -> i32 {
			self.sensors.len() as i32
		}
		fn sensor_locations_set(&self) -> bool {
			true
		}
		fn sensor_normals_set(&self) -> bool {
			true
		}
		fn sensor_location(&self, i: i32) -> [f64; 3] {
			self.sensors[i as usize]
		}
		fn sensor_normal(&self, i: i32) -> [f64; 3] {
			self.normals[i as usize]
		}
		fn object_type(&self) -> i32 {
			if self.controller {
				SURVIVE_OBJECT_TYPE_CONTROLLER
			} else {
				3
			}
		}
		fn timebase_hz(&self) -> i32 {
			48_000_000
		}
		fn raw_acc_scale(&self) -> f64 {
			0.0
		}
		fn raw_gyro_scale(&self) -> f64 {
			0.0
		}
		fn acc_scale(&self, _i: usize) -> f64 {
			1.0
		}
		fn acc_bias(&self, _i: usize) -> f64 {
			0.0
		}
		fn codename(&self) -> Name {
			Name::new("TST")
		}
		fn active_lighthouses(&self) -> i32 {
			self.stations.len() as i32
		}
		fn lh_version(&self) -> i32 {
			1
		}
		fn bsd_mode(&self, lh: i32) -> i32 {
			lh
		}
		fn bsd_position_set(&self, lh: i32) -> bool {
			(lh as usize) < self.stations.len()
		}
		fn bsd_fcal(&self, lh: i32) -> [BaseStationCal; 2] {
			self.stations[lh as usize].1
		}
		fn lighthouse_position(&mut self, lh: i32) -> SurvivePose {
			self.stations[lh as usize].0
		}
		fn recording(&self) -> bool {
			true
		}
		fn log(&mut self, warn: bool, text: &str) {
			self.lines.push(if warn { format!("WARN {text}") } else { text.to_string() });
		}
		fn record_line(&mut self, line: &str) {
			self.lines.push(line.to_string());
		}
	}

	/// a tracker-like layout: 16 sensors on a 5 cm sphere, normals outward
	fn device(controller: bool) -> TestDevice {
		let mut sensors = Vec::new();
		let mut normals = Vec::new();
		for i in 0..16 {
			let th = i as f64 * 2.399963; // golden angle
			let z = 1.0 - (i as f64 + 0.5) / 8.0;
			let r = (1.0 - z * z).max(0.0).sqrt();
			let n = [r * th.cos(), r * th.sin(), z];
			sensors.push([n[0] * 0.05, n[1] * 0.05, n[2] * 0.05]);
			normals.push(n);
		}
		// two stations 2.5 m up in two corners, looking at the middle of the room (their -z toward the device)
		let look = |pos: [f64; 3]| -> SurvivePose {
			// z axis of the station = from the origin to the station (the device is in front, along -z)
			let z = normalize3d(&pos);
			let x = normalize3d(&sally_math::cross3d(&[0.0, 0.0, 1.0], &z));
			let y = sally_math::cross3d(&z, &x);
			let m = [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
			SurvivePose { pos, rot: quatfrommatrix33(&m) }
		};
		let cal =
			[BaseStationCal { tilt: 0.01, ..Default::default() }, BaseStationCal { tilt: -0.01, ..Default::default() }];
		TestDevice {
			sensors,
			normals,
			controller,
			stations: vec![(look([2.0, 2.0, 2.5]), cal), (look([-2.0, 1.5, 2.4]), cal)],
			lines: Vec::new(),
		}
	}

	fn core(s: Settings) -> Box<Tracker> {
		Tracker::create(s)
	}

	/// feed a device at rest at p0 (identity orientation): IMU at 250 Hz, every sensor's hits from both stations
	fn run(c: &mut Tracker, d: &mut TestDevice, secs: f64) -> usize {
		feed(c, d, 1, (secs * 250.0) as u64, |_| Some([0.1, -0.2, 1.0]), |_, _, _| {}).0
	}

	/// feed the IMU steps s0 ..= s1 (250 Hz) of a device at rest (identity orientation) at at(step), dark where that is
	/// None, with every sensor's hits from both stations; imu(step, accel, gyro) may change the sample. The reports, the
	/// reports of the last 250 steps, and the last reported position
	fn feed(
		c: &mut Tracker, d: &mut TestDevice, s0: u64, s1: u64, at: impl Fn(u64) -> Option<[f64; 3]>,
		imu: impl Fn(u64, &mut [f64; 3], &mut [f64; 3]),
	) -> (usize, usize, [f64; 3]) {
		let tb = 48_000_000.0;
		let (mut reports, mut last, mut p_last) = (0, 0, [f64::NAN; 3]);
		for s in s0..=s1 {
			let t = s as f64 / 250.0;
			// the sweeps of the last 4 ms: each station, alternating planes
			for lh in 0..d.stations.len() {
				let Some(p0) = at(s) else {
					break;
				};
				let (pose, cal) = d.stations[lh];
				let plane = (s % 2) as i32;
				let w2l = invert_pose(&pose);
				for i in 0..d.sensors.len() {
					let sp = d.sensors[i];
					let pw = [p0[0] + sp[0], p0[1] + sp[1], p0[2] + sp[2]];
					let pl = apply_pose_to_point(&w2l, &pw);
					let dir = normalize3d(&sub3d(&pose.pos, &pw));
					if dot3d(&d.normals[i], &dir) < 0.2 {
						continue;
					}
					let ang = REPROJECT_AXIS_FN_GEN2[plane as usize](&cal, &pl);
					let th = t - 0.002 - 0.0045 + i as f64 * 1e-5;
					c.integrate_hit(d, (th * tb) as u64, lh as c_int, plane, i as c_int, ang);
				}
			}
			let (mut accel, mut gyro) = ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
			imu(s, &mut accel, &mut gyro);
			if let Some((pose, _)) = c.integrate_imu(d, (t * tb) as u64, &accel, &gyro) {
				reports += 1;
				if s + 250 > s1 {
					last += 1;
				}
				p_last = pose.pos;
			}
		}
		(reports, last, p_last)
	}

	#[test]
	fn starts_cold_and_tracks_a_still_device() {
		// the cold start (the barycentric SVD) gives the first guess, the seed solve takes it and the reports begin
		let mut d = device(false);
		let mut c = core(Settings::defaults());
		let reports = run(&mut c, &mut d, 1.0);
		assert!(reports > 0, "started from cold");
		assert!(c.bc.is_some(), "the cold start's solver was set up");
		assert!(c.stats.seeds >= 1 && c.stats.hits > 0 && c.stats.sweeps > 0);
		assert_eq!(c.imu_count, 250);
		let p0 = [0.1, -0.2, 1.0];
		assert!(norm3d(&sub3d(&c.p, &p0)) < 0.01, "where the device is: {:?}", c.p);
		c.print_stats(&mut d);
		assert!(d.lines.iter().any(|l| l.starts_with("TST track: ") && l.contains(" hits in ")), "{:?}", d.lines);
	}

	#[test]
	fn holds_the_velocity_along_the_line_the_light_does_not_see() {
		// a position known to 1 mm across a line and to 30 mm along it; a velocity of 0.15 m/s along the line and 4 mm/s
		// across it: the line is found, the hold takes the velocity along it and leaves the rest
		let mut c = core(Settings::defaults());
		let u = normalize3d(&[1.0, 2.0, -0.5]);
		let x = normalize3d(&[2.0, -1.0, 0.0]); // across u
		for i in 0..NS * NS {
			c.P[i] = 0.0;
		}
		for a in 0..3 {
			for b in 0..3 {
				c.P[(IP + a) * NS + IP + b] = (if a == b { 1e-6 } else { 0.0 }) + (0.03 * 0.03 - 1e-6) * u[a] * u[b];
				c.P[(IV + a) * NS + IV + b] = if a == b { 0.05 * 0.05 } else { 0.0 };
			}
		}
		for a in 0..3 {
			c.v[a] = 0.15 * u[a] + 0.004 * x[a];
		}
		let l = c.blind_line().expect("a line");
		assert!(dot3d(&l, &u).abs() > 0.999_999, "the line {l:?} for {u:?}");
		assert!((c.hold_sd - 0.03).abs() < 1e-6, "the sd along it {}", c.hold_sd);
		let p = c.p;
		for _ in 0..50 {
			c.velocity_along(&l, 0.0, 0.01);
		}
		assert!(dot3d(&c.v, &u).abs() < 1e-3, "the velocity along the line {:?}", c.v);
		assert!((dot3d(&c.v, &x) - 0.004).abs() < 1e-9, "the velocity across it {:?}", c.v);
		assert!(norm3d(&sub3d(&c.p, &p)) < 1e-12, "the position {:?}", c.p);
		let mut along = 0.0;
		for a in 0..3 {
			for b in 0..3 {
				along += u[a] * c.P[(IV + a) * NS + IV + b] * u[b];
				assert_eq!(c.P[(IV + a) * NS + IV + b], c.P[(IV + b) * NS + IV + a]);
			}
		}
		assert!(along > 0.0 && along.sqrt() < 0.002, "the velocity's sd along the line {}", along.sqrt());
		assert!(
			(c.P[IV * NS + IV] + c.P[(IV + 1) * NS + IV + 1] + c.P[(IV + 2) * NS + IV + 2] - along - 2.0 * 0.0025)
				.abs() < 1e-9
		);
	}

	#[test]
	fn seeded_by_hand_tracks_and_reports() {
		// the state set as a solve would set it: the update, the propagation, the rest test and the report run
		let mut d = device(true);
		let mut c = core(Settings::defaults());
		run(&mut c, &mut d, 0.1);
		c.p = [0.1, -0.2, 1.0];
		c.v = [0.0; 3];
		c.q = [1.0, 0.0, 0.0, 0.0];
		let mut I9 = [0.0; SEEDX * SEEDX];
		for a in 0..SEEDX {
			I9[a * SEEDX + a] = 1e-6;
		}
		c.reset_P(&mut d, &I9);
		c.seeded = true;
		c.ever_seeded = true;
		c.last_light_t = c.t;
		c.seed_t = c.t;
		let reports = run(&mut c, &mut d, 2.0);
		assert_eq!(reports, 500);
		assert!(c.stats.used > 1000, "used {}", c.stats.used);
		assert!(c.stats.zupts > 0);
		for i in 0..3 {
			assert!((c.p[i] - [0.1, -0.2, 1.0][i]).abs() < 1e-3, "p {:?}", c.p);
		}
		let dg = c.diag_state(&d);
		assert_eq!(dg.state, SURVIVE_DIAG_STATE_TRACKING);
		assert_eq!(dg.lh_mask, 3);
		assert!(c.handover().is_some() && c.handover_alpha().is_some());
		assert!(c.confirmed);
	}

	#[test]
	fn records_lines_as_the_c_printf_wrote_them() {
		let mut d = device(false);
		let mut s = Settings::defaults();
		s.verbose = 3;
		let mut c = core(s);
		run(&mut c, &mut d, 0.1);
		let imu = d.lines.iter().find(|l| l.starts_with("TST TRACK_IMU ")).expect("TRACK_IMU lines at verbose 3");
		assert!(imu.ends_with("\r\n"));
		assert!(imu.contains(" 0.0000 0.0000 9.8066 "), "{imu}");
	}

	#[test]
	fn clock_back_restarts() {
		let mut d = device(false);
		let mut c = core(Settings::defaults());
		run(&mut c, &mut d, 1.0);
		c.integrate_imu(&mut d, 48_000_000 / 100, &[0.0, 0.0, 1.0], &[0.0; 3]);
		assert_eq!(c.t, 0.01);
		assert!(d.lines.iter().any(|l| l.contains("the device's clock went back")));
	}

	/// An update whose innovation matrix is not positive definite (P no longer is) uses none of its sweep: the hits go
	/// down as rejected, the light's time stays, and the light's timeout restarts the state, P with it. Counted as
	/// used, the tracker would dead-reckon while every sweep "fit".
	#[test]
	fn a_sweep_the_update_cannot_take_is_not_counted_as_used() {
		let mut d = device(false);
		let mut c = core(Settings::defaults());
		assert!(run(&mut c, &mut d, 1.0) > 0, "started");
		let seeds = c.stats.seeds + c.stats.reseeds;
		// P made indefinite: the position's variance negative
		for i in 0..3 {
			c.P[(IP + i) * NS + IP + i] = -1.0;
		}
		let (used, rejected, light) = (c.stats.used, c.stats.rejected, c.last_light_t);
		d.lines.clear();
		feed(&mut c, &mut d, 251, 262, |_| Some([0.1, -0.2, 1.0]), |_, _, _| {});
		let failed = d.lines.iter().filter(|l| l.contains("innovation matrix not positive definite")).count();
		assert!(failed > 0, "the update failed: {:?}", &d.lines[..d.lines.len().min(5)]);
		assert_eq!(c.stats.used, used, "no hit of those sweeps counted as used");
		assert!(c.stats.rejected > rejected);
		assert_eq!(c.last_light_t, light, "the light's time where it was");
		assert!(c.recent[..ix(c.recent_n)].iter().filter(|r| r.t > light).all(|r| r.used == 0));
		// the light's timeout restarts the state, and the device tracks again
		let (_, last, p) = feed(&mut c, &mut d, 263, 1000, |_| Some([0.1, -0.2, 1.0]), |_, _, _| {});
		assert!(c.stats.seeds + c.stats.reseeds > seeds, "started again");
		assert!(last > 200, "reports in the last second: {last}");
		assert!(norm3d(&sub3d(&p, &[0.1, -0.2, 1.0])) < 0.01, "where the device is: {p:?}");
		assert!((0..NS).all(|i| c.P[i * NS + i] >= 0.0), "P started again");
	}

	/// propagate_P's sparse products: the same bits as the dense ones, for a sparse left factor (zeros of both signs among
	/// its entries) and any finite right one; all_finite tells the matrices they may take
	#[test]
	fn the_sparse_products_are_the_dense_ones_bit_for_bit() {
		let mut r = 0x2545_f491_4f6c_dd1du64;
		let mut next = move || {
			r ^= r << 13;
			r ^= r >> 7;
			r ^= r << 17;
			r
		};
		for case in 0..400 {
			let mut F = [0.0; NS * NS];
			let mut P = [0.0; NS * NS];
			for i in 0..NS * NS {
				let u = next();
				F[i] = match u % 8 {
					0 => (u >> 11) as f64 / (1u64 << 53) as f64 - 0.5,
					1 => -0.0,
					_ => 0.0,
				};
				// P: values over many magnitudes and both signs, -0 and subnormals among them
				let v = next();
				P[i] = match v % 16 {
					0 => -0.0,
					1 => f64::from_bits(v >> 13),
					_ => ((v >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 10f64.powi((v % 40) as i32 - 20),
				};
			}
			for i in 0..NS {
				F[i * NS + i] = if case % 5 == 0 { 1.0 } else { F[i * NS + i] };
			}
			assert!(all_finite(&P));
			let (mut nz, mut nn) = ([[0u8; NS]; NS], [0usize; NS]);
			row_nz(&F, &mut nz, &mut nn);
			let (mut d1, mut s1, mut d2, mut s2, mut Ft) =
				([0.0; NS * NS], [0.0; NS * NS], [0.0; NS * NS], [0.0; NS * NS], [0.0; NS * NS]);
			mul_ikj(&F, &P, &mut d1);
			mul_sparse_a(&F, &nz, &nn, &P, &mut s1);
			transpose(&F, &mut Ft);
			mul_ikj(&P, &Ft, &mut d2);
			mul_abt_sparse_b(&P, &F, &nz, &nn, &mut s2);
			for i in 0..NS * NS {
				assert_eq!(d1[i].to_bits(), s1[i].to_bits(), "F P, case {case}, entry {i}");
				assert_eq!(d2[i].to_bits(), s2[i].to_bits(), "P F^T, case {case}, entry {i}");
			}
		}
		let mut P = [1.0; NS * NS];
		P[200] = f64::INFINITY;
		assert!(!all_finite(&P));
		P[200] = f64::NAN;
		assert!(!all_finite(&P));
	}

	/// A NaN sample loses the state, and the light starts it again. Without the guards, one NaN gyro sample (or one NaN
	/// accelerometer sample while coasting) and the device never starts again, and after a NaN accelerometer sample the
	/// next warm restart never comes.
	#[test]
	fn a_nan_sample_does_not_lose_the_device_for_good() {
		let p0 = [0.1, -0.2, 1.0];
		let p1 = [0.35, -0.2, 1.0];
		let tracking = |c: &Tracker, d: &TestDevice, last: usize, p: [f64; 3], want: [f64; 3]| {
			assert!(d.lines.iter().any(|l| l.contains("ran away")), "the NaN sample lost the state: {:?}", d.lines);
			assert_eq!(last, 250, "reported throughout the last second: {:?}", d.lines);
			assert!(norm3d(&sub3d(&p, &want)) < 0.01, "where the device is: {p:?}");
			assert_eq!(c.diag_state(d).state, SURVIVE_DIAG_STATE_TRACKING);
		};
		// one NaN gyro sample while tracking (two stations: the first-light term took the NaN of the turn count too)
		let mut d = device(false);
		let mut c = core(Settings::defaults());
		let (_, last, p) = feed(
			&mut c,
			&mut d,
			1,
			1250,
			|_| Some(p0),
			|s, _, g| {
				if s == 250 {
					g[0] = f64::NAN;
				}
			},
		);
		tracking(&c, &d, last, p, p0);
		// one NaN accelerometer sample while coasting in the dark (2 to 4 s): P took the NaN, the zero-velocity update
		// 0.24 s later put it in the state, which ran away; the light's return starts it again
		let mut d = device(false);
		let mut c = core(Settings::defaults());
		let (_, last, p) = feed(
			&mut c,
			&mut d,
			1,
			1750,
			|s| if (500..1000).contains(&s) { None } else { Some(p0) },
			|s, a, _| {
				if s == 700 {
					a[2] = f64::NAN;
				}
			},
		);
		tracking(&c, &d, last, p, p0);
		// one NaN accelerometer sample while lit, then the light gone for 1 s and the device 0.25 m away at its return:
		// the warm restart (the IMU-only motion is not NaN since the start after the NaN sample)
		let mut d = device(false);
		let mut c = core(Settings::defaults());
		let at = |s: u64| match s {
			500..=749 => None,
			750.. => Some(p1),
			_ => Some(p0),
		};
		let (_, last, p) = feed(&mut c, &mut d, 1, 1250, at, |s, a, _| {
			if s == 250 {
				a[2] = f64::NAN;
			}
		});
		tracking(&c, &d, last, p, p1);
		assert!(d.lines.iter().any(|l| l.contains("warm restart") && l.contains("SOLVED")), "{:?}", d.lines);
	}
}
