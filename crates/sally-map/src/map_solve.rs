//! The map solve as the runtime runs it: the sweep hits of every device kept in a ring, a snapshot whenever a device is
//! slow and two stations light it, a joint solve every map-solve-period s over the snapshots since the last, its result
//! slewed into the poses in use with the frame held, moved stations found and relocated, and the map file.
//!
//! It sits on two hooks, sweep_angle and imupose, each after the ones installed before it: a sweep angle goes through
//! the default process, then diagnosis's hook, then `sweep_angle`; a pose through the default imupose process, then
//! diagnosis's, then `imupose`, then diagrec's. (The tracker reports a pose before its velocity, so the velocity
//! `imupose` reads is the one reported with the previous pose.)
//!
//! What a `MapSolve` holds is the hooks' side, which a threaded solve reads and writes only with the context lock
//! (`solving::Locked`). The snapshots since the last solve are the batch, behind a lock of its own, and the solve's own
//! state is its `Solver` (`solving`). A solve runs inline, in the imupose call whose run time first reaches
//! map-solve-period after the last solve (that call applies the result: the slew of the next pose moves toward it), or,
//! once `start_thread` has started the solve thread, on that thread, which the call only wakes. There is no thread when
//! the playback factor is 0 (a replay as fast as it goes), so every replay the bench makes solves inline,
//! deterministically; a live runtime's thread applies a solve's result whenever it gets the context lock, which no two
//! runs repeat.

use std::sync::{Arc, Mutex};

use sally_math::printf::Arg;
use sally_math::{c_acos, dist3d, norm3d, pose_slerp, quatiszero, Pose};

use crate::corrections::{DeviceCorrections, StationCorrections, MS_MAXDEV};
use crate::host::{DeviceId, Host};
use crate::linalg::{mm3, mv3, q2r, quatinnerproduct, r2q, rv2r};
use crate::model::{Snapshot, MS_MAXG, MS_MAXH, MS_MAXLH};
use crate::settings::Settings;
use crate::solving::{lock, Locked, SolveThread, Solver};
use crate::text::{cut, info};
use sally_math::scanf::sscanf;

/// MS_RING: the sweep hits kept per device
pub const MS_RING: usize = 2048;

/// ms_hit: one sweep hit as the ring keeps it
#[derive(Clone, Copy, Default)]
struct Hit {
	tc: u64,
	angle: f64,
	lh: u8,
	sensor: u8,
	plane: i8,
}

/// ms_dev: a device's recent hits
struct Device {
	id: DeviceId,
	ring: Box<[Hit; MS_RING]>,
	head: u32,
	last_snap: f64,
}

/// The counts the map solve keeps (its close line)
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
	pub solves: i32,
	pub applied: i32,
	/// solves not applied because they moved a station too far
	pub rejected_moves: i32,
	pub snapshots: i64,
	pub slow_poses: i64,
	/// moved-station flags and relocations
	pub suspects: i32,
	pub relocations: i32,
}

/// map_solve: the live joint map solve, its hooks' side
pub struct MapSolve {
	pub(crate) s: Settings,
	/// taken from the map file already (load_map_file)
	pub(crate) file_loaded: [bool; MS_MAXLH],
	/// map-solve-batch as the plugin started (a live change would overrun the batch)
	batch_max: usize,
	pub(crate) last_save: f64,

	// moved stations (relocate): the solve's own part is the Solver's
	/// relocated: no longer part of the frame hold, no prior to the calibrated map
	pub(crate) unanchored: [bool; MS_MAXLH],
	/// the anchors came from the map file (so they are written back to it)
	pub(crate) anchors_saved: bool,
	/// the calibration (config) the anchors were derived from, for the map file
	pub(crate) cal: [Pose; MS_MAXLH],
	have_cal: [bool; MS_MAXLH],
	pub(crate) relocations: i32,

	// map-solve-test-move
	test_t: f64,
	test_d: [f64; 6],
	test_lh: i32,
	test_done: bool,

	devs: Vec<Device>,
	/// the snapshots since the last solve (ms->batch, under batch_lock), which the solve takes
	pub(crate) batch: Arc<Mutex<Vec<Snapshot>>>,

	pub(crate) have_start: [bool; MS_MAXLH],
	/// calibrated map (lh2world), the frame anchor
	pub(crate) start: [Pose; MS_MAXLH],
	pub(crate) have_target: [bool; MS_MAXLH],
	/// the solved map (what the poses in use slew toward)
	pub(crate) target: [Pose; MS_MAXLH],
	/// station information (tangent space at target, index 6*bsd+k)
	pub(crate) lam: Vec<f64>,
	pub(crate) lam_time: f64,
	loaded: bool,
	last_solve: f64,
	last_slew: f64,
	pub(crate) applied: i32,
	pub(crate) rejected_moves: i32,
	n_slow: i64,
	n_snap: i64,

	fcal: StationCorrections,
	dscale: DeviceCorrections,

	/// the solve's own state, locked by whoever solves: the hook inline, the solve thread when there is one (and the
	/// close, after the thread is joined)
	pub(crate) solver: Arc<Mutex<Solver>>,
	/// the solve thread (ms->threaded), once started
	thread: Option<SolveThread>,
}

impl MapSolve {
	/// The map solve started: its bound options read from the config store, then the rest of `start_with`
	pub fn start(host: &mut (impl Host + ?Sized)) -> MapSolve {
		let settings = Settings::read(host);
		MapSolve::start_with(host, settings)
	}

	/// The start, with the bound options' values: the stations' calibration as the config has it, station-fcal-file and
	/// device-calibration-file read, and map-solve-test-move parsed; the solves inline until `start_thread`
	/// (map-solve-threaded, unless the playback factor is 0)
	pub fn start_with(host: &mut (impl Host + ?Sized), settings: Settings) -> MapSolve {
		let batch_max = if settings.batch < 50 { 50 } else { settings.batch } as usize;
		let batch = Arc::new(Mutex::new(Vec::with_capacity(batch_max)));
		let solver = Arc::new(Mutex::new(Solver::new(batch.clone(), batch_max)));
		let mut ms = MapSolve {
			s: settings,
			file_loaded: [false; MS_MAXLH],
			batch_max,
			last_save: 0.0,
			unanchored: [false; MS_MAXLH],
			anchors_saved: false,
			cal: [Pose::default(); MS_MAXLH],
			have_cal: [false; MS_MAXLH],
			relocations: 0,
			test_t: 0.0,
			test_d: [0.0; 6],
			test_lh: 0,
			test_done: false,
			devs: Vec::new(),
			batch,
			have_start: [false; MS_MAXLH],
			start: [Pose::default(); MS_MAXLH],
			have_target: [false; MS_MAXLH],
			target: [Pose::default(); MS_MAXLH],
			lam: vec![0.0; MS_MAXG * MS_MAXG],
			lam_time: 0.0,
			loaded: false,
			last_solve: 0.0,
			last_slew: 0.0,
			applied: 0,
			rejected_moves: 0,
			n_slow: 0,
			n_snap: 0,
			fcal: StationCorrections::empty(),
			dscale: DeviceCorrections::empty(),
			solver,
			thread: None,
		};
		// the calibration as the config has it, before any saved map moves the poses in use (a station whose first
		// light comes after the map file is loaded would otherwise report the loaded pose as its calibration) (the
		// config's PositionSet flag is usually absent, so a pose read from it is recognized by its rotation)
		let mut l = 0;
		while l < MS_MAXLH && (l as i32) < host.active_lighthouses() {
			let p = host.bsd_pose(l);
			if !quatiszero(&p.rot) {
				ms.cal[l] = p;
				ms.have_cal[l] = true;
			}
			l += 1;
		}
		ms.test_lh = -1;
		ms.fcal = StationCorrections::load(host);
		ms.dscale = DeviceCorrections::load(host);
		let tm = host.configs("map-solve-test-move", "");
		if !tm.is_empty() {
			// commas allowed, so the value is one argument
			let buf = cut(tm.clone(), 256).replace(',', " ");
			let (n, v) = sscanf(buf.as_bytes(), "%lf %d %lf %lf %lf %lf %lf %lf");
			if n == 8 {
				use sally_math::scanf::Value;
				let (Value::Lf(t), Value::D(lh)) = (&v[0], &v[1]) else { unreachable!() };
				ms.test_t = *t;
				ms.test_lh = *lh;
				for k in 0..6 {
					let Value::Lf(d) = v[2 + k] else { unreachable!() };
					ms.test_d[k] = d;
				}
			} else {
				crate::text::warn(
					host,
					"map-solve: map-solve-test-move '%s' not understood (T LH dx dy dz rx ry rz)",
					&[Arg::S(&tm)],
				);
			}
		}
		info(
			host,
			"map-solve: live joint map solve on (every %.1f s, memory tau %.0f s, slew %.1f mm/s %.2f mrad/s)",
			&[Arg::F(ms.s.period), Arg::F(ms.s.tau), Arg::F(ms.s.rate_pos * 1e3), Arg::F(ms.s.rate_rot * 1e3)],
		);
		ms
	}

	/// ms->threaded: from now on the solves run on a thread of their own ("map-solve", with OGCreateThread's stack),
	/// which `lock` takes the context lock for where the C's takes it. The imupose hook only wakes it (pending, cv);
	/// `stop_thread` ends it
	pub fn start_thread(&mut self, lock: impl Locked + Send + 'static) {
		self.thread = Some(SolveThread::start(self.solver.clone(), lock));
	}

	/// The first half of the close for a threaded solve: the thread told to stop (active cleared and the cv signalled),
	/// handed back to be joined without the context lock, which the thread may still take to finish the solve it is on.
	/// None without a thread
	pub fn stop_thread(&mut self) -> Option<std::thread::JoinHandle<()>> {
		self.thread.as_mut()?.stop()
	}

	/// The bound options as their variables hold them now: the C's map_solve fields, which a live set writes and the
	/// solve reads where it uses them. A runtime that binds variables to the options hands them in before each call
	/// (map-solve-batch's batch size stays the one the plugin started with, as in the C)
	pub fn set_settings(&mut self, settings: Settings) {
		self.s = settings;
	}

	/// The counts kept so far (a threaded solve's, as of its last)
	pub fn stats(&self) -> Stats {
		let solver = lock(&self.solver);
		Stats {
			solves: solver.solves,
			applied: self.applied,
			rejected_moves: self.rejected_moves,
			snapshots: self.n_snap,
			slow_poses: self.n_slow,
			suspects: solver.suspects,
			relocations: self.relocations,
		}
	}

	/// The solved map (what the poses in use slew toward) of station lh, once there is one
	pub fn target(&self, lh: usize) -> Option<Pose> {
		self.have_target[lh].then_some(self.target[lh])
	}

	/// sally: a device that is another one from here on (its dongle carries another device: the runtime's
	/// device_reset): its recent hits and its snapshots not solved yet go, since they were measured on the other device
	/// (its layout and pose)
	pub fn forget_device(&mut self, dev: DeviceId) {
		// its layout is the new device's config's: that device's corrections are still to be made
		self.dscale.forget(dev);
		let Some(di) = self.devs.iter().position(|d| d.id == dev) else { return };
		let d = &mut self.devs[di];
		d.ring.fill(Hit::default());
		d.head = 0;
		d.last_snap = -1e9;
		lock(&self.batch).retain(|s| s.dev != di as i32);
	}

	/// get_dev: the device's ring, made on its first hit (at most MS_MAXDEV devices)
	fn get_dev(&mut self, dev: DeviceId) -> Option<usize> {
		if let Some(i) = self.devs.iter().position(|d| d.id == dev) {
			return Some(i);
		}
		if self.devs.len() >= MS_MAXDEV {
			return None;
		}
		self.devs.push(Device { id: dev, ring: Box::new([Hit::default(); MS_RING]), head: 0, last_snap: -1e9 });
		Some(self.devs.len() - 1)
	}

	/// sweep_angle_fn: a device's sweep hit, into its ring (the runtime has called the hooks before this one)
	#[allow(clippy::too_many_arguments)] // the hook's signature
	pub fn sweep_angle(
		&mut self, host: &mut (impl Host + ?Sized), dev: DeviceId, channel: u8, sensor_id: i32, timecode: u32,
		plane: i8, angle: f64,
	) {
		let lh = host.get_bsd_idx(channel);
		if lh < 0 || !(0..=1).contains(&plane) {
			return;
		}
		let Some(d) = self.get_dev(dev) else { return };
		let tc = host.long_timecode_light(dev, timecode);
		let d = &mut self.devs[d];
		let h = &mut d.ring[d.head as usize & (MS_RING - 1)];
		h.tc = tc;
		h.lh = lh as u8;
		h.sensor = sensor_id as u8;
		h.plane = plane;
		h.angle = angle;
		d.head = d.head.wrapping_add(1);
	}

	/// imupose_fn: a device's pose (the runtime has called the hooks before this one): the corrections, the stations
	/// that became ready, the map file, a test move, the device's snapshot, the slew and, when due, a solve (inline, or
	/// the solve thread woken)
	pub fn imupose(&mut self, host: &mut impl Host, dev: DeviceId, timecode: u64, pose: &Pose) {
		let now = host.run_time();
		let mut l = 0;
		while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
			self.fcal.apply(host, l);
			l += 1;
		}
		self.dscale.apply(host, dev);
		l = 0;
		while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
			if !self.have_start[l] && host.bsd_position_set(l) && host.bsd_ootx_set(l) {
				let p = host.bsd_pose(l);
				self.start[l] = p;
				if !self.have_cal[l] {
					self.cal[l] = p;
					self.have_cal[l] = true;
				}
				self.have_start[l] = true;
				self.loaded = false;
			}
			l += 1;
		}
		if !self.loaded {
			self.loaded = true;
			self.load_map_file(host);
		}
		if self.test_lh >= 0
			&& !self.test_done
			&& now >= self.test_t
			&& (self.test_lh as usize) < MS_MAXLH
			&& self.have_start[self.test_lh as usize]
		{
			// map-solve-test-move: the map (in use, solved and calibrated) moves while the light does not, which to
			// the solver is the same as a bumped station
			let l = self.test_lh as usize;
			self.test_done = true;
			let dr = rv2r(&[self.test_d[3], self.test_d[4], self.test_d[5]]);
			let with_target = self.have_target[l];
			let mut cur = host.bsd_pose(l);
			let test_d = self.test_d;
			let moved = |p: &mut Pose| {
				let r = mm3(&dr, &q2r(&p.rot));
				p.rot = r2q(&r);
				for a in 0..3 {
					p.pos[a] += test_d[a];
				}
			};
			moved(&mut cur);
			moved(&mut self.start[l]);
			if with_target {
				moved(&mut self.target[l]);
			}
			host.raw_lighthouse_pose(l, &cur);
			let mode = host.bsd_mode(l);
			info(
				host,
				"map-solve: TEST: station %d (ch %d) moved in the map by %.1f %.1f %.1f mm, %.2f %.2f %.2f mrad at %.1f s",
				&[
					Arg::I(l as i64),
					Arg::I(mode as i64),
					Arg::F(test_d[0] * 1e3),
					Arg::F(test_d[1] * 1e3),
					Arg::F(test_d[2] * 1e3),
					Arg::F(test_d[3] * 1e3),
					Arg::F(test_d[4] * 1e3),
					Arg::F(test_d[5] * 1e3),
					Arg::F(now),
				],
			);
		}
		if let Some(d) = self.get_dev(dev) {
			if now - self.devs[d].last_snap >= self.s.step {
				self.try_snapshot(host, d, timecode, pose);
			}
		}
		self.slew(host, now);
		if now > self.s.first && now - self.last_solve >= self.s.period {
			self.last_solve = now;
			match &self.thread {
				Some(t) => t.wake(),
				None => self.solve_inline(host),
			}
		}
	}

	/// batch_add: a snapshot into the batch, under batch_lock; a full batch drops its oldest
	fn batch_add(&mut self, s: Snapshot) {
		let mut batch = lock(&self.batch);
		if batch.len() < self.batch_max {
			batch.push(s);
		} else {
			let mut slot = 0;
			for i in 1..batch.len() {
				if batch[i].time < batch[slot].time {
					slot = i;
				}
			}
			batch[slot] = s;
		}
	}

	/// try_snapshot: the device's hits of the last map-solve-window s as a snapshot, if it is slow and two stations
	/// each give it map-solve-min-hits front-facing hits
	fn try_snapshot(&mut self, host: &mut (impl Host + ?Sized), di: usize, tc0: u64, pose: &Pose) {
		let dev = self.devs[di].id;
		if !host.sensor_locations_set(dev) || !host.sensor_normals_set(dev) {
			return;
		}
		let hz = host.timebase_hz(dev);
		if hz == 0 {
			return;
		}
		let vel = host.velocity(dev);
		let v = norm3d(&vel.pos);
		let w = norm3d(&vel.axis_angle_rot);
		if v > self.s.vmax || w > self.s.wmax {
			return;
		}
		self.n_slow += 1;
		let hz = hz as f64;
		let mut s = Snapshot { dev: di as i32, time: host.run_time(), ..Snapshot::default() };
		s.r0 = q2r(&pose.rot);
		s.p0 = pose.pos;
		s.v = vel.pos;
		s.w = vel.axis_angle_rot;
		let mut per = [0i32; MS_MAXLH];
		let mut tmp: Vec<Hit> = Vec::with_capacity(MS_MAXH * 2);
		let d = &self.devs[di];
		let mut k = 0u32;
		while (k as usize) < MS_RING && tmp.len() < MS_MAXH * 2 {
			let h = d.ring[d.head.wrapping_sub(1).wrapping_sub(k) as usize & (MS_RING - 1)];
			k += 1;
			if h.tc == 0 {
				break;
			}
			let dt = (h.tc as f64 - tc0 as f64) / hz;
			if dt < -self.s.window {
				break;
			}
			if dt > 0.004 {
				continue;
			}
			let lh = h.lh as usize;
			if h.lh as i32 >= host.active_lighthouses()
				|| !host.bsd_position_set(lh)
				|| h.sensor as i32 >= host.sensor_ct(dev)
			{
				continue;
			}
			// back-facing (reflection) test with the seed pose
			let wp = mv3(&s.r0, &host.sensor_location(dev, h.sensor as usize));
			let wn = mv3(&s.r0, &host.sensor_normal(dev, h.sensor as usize));
			let lp = host.bsd_pose(lh).pos;
			let mut to = [0.0; 3];
			for k2 in 0..3 {
				to[k2] = lp[k2] - (wp[k2] + s.p0[k2]);
			}
			if wn[0] * to[0] + wn[1] * to[1] + wn[2] * to[2] <= 0.0 {
				continue;
			}
			tmp.push(h);
			per[lh] += 1;
		}
		let mut nst = 0;
		for l in 0..MS_MAXLH {
			if per[l] >= self.s.min_hits {
				nst += 1;
			}
		}
		if nst < 2 {
			return;
		}
		for h in &tmp {
			if s.n >= MS_MAXH {
				break;
			}
			if per[h.lh as usize] < self.s.min_hits {
				continue;
			}
			s.lh[s.n] = h.lh;
			s.plane[s.n] = h.plane;
			s.angle[s.n] = h.angle;
			s.dt[s.n] = (h.tc as f64 - tc0 as f64) / hz;
			s.pt[s.n] = host.sensor_location(dev, h.sensor as usize);
			s.n += 1;
		}
		self.devs[di].last_snap = s.time;
		self.n_snap += 1;
		self.batch_add(s);
	}

	/// slew: the poses in use toward the solved map, at most map-solve-rate-pos m/s and map-solve-rate-rot rad/s
	/// (every 0.05 s at most)
	fn slew(&mut self, host: &mut (impl Host + ?Sized), now: f64) {
		let mut dt = now - self.last_slew;
		if dt < 0.05 {
			return;
		}
		self.last_slew = now;
		if dt > 0.5 {
			dt = 0.5;
		}
		let mut l = 0;
		while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
			if !self.have_target[l] || !host.bsd_position_set(l) {
				l += 1;
				continue;
			}
			let cur = host.bsd_pose(l);
			let tgt = self.target[l];
			let dp = dist3d(&cur.pos, &tgt.pos);
			let qa = quatinnerproduct(&cur.rot, &tgt.rot).abs();
			let dq = 2.0 * c_acos(if qa > 1.0 { 1.0 } else { qa });
			// at the target: 2 acos(|q.q|) of one quaternion is up to ~4e-8 from rounding, and a tighter test re-sent
			// every station at 20 Hz for good (a lighthouse update, a recording line and L lines each time)
			if dp < 1e-7 && dq < 1e-6 {
				l += 1;
				continue;
			}
			let mut f = 1.0;
			if dp > 0.0 && self.s.rate_pos * dt / dp < f {
				f = self.s.rate_pos * dt / dp;
			}
			if dq > 0.0 && self.s.rate_rot * dt / dq < f {
				f = self.s.rate_rot * dt / dq;
			}
			let np = if f >= 1.0 { tgt } else { pose_slerp(&cur, &tgt, f) };
			host.raw_lighthouse_pose(l, &np);
			l += 1;
		}
	}

	/// set_in_use: the pose the tracker uses for station l, now (the slew follows the target, so both are set)
	pub(crate) fn set_in_use(&mut self, host: &mut (impl Host + ?Sized), l: usize, p: &Pose) {
		self.target[l] = *p;
		self.have_target[l] = true;
		host.raw_lighthouse_pose(l, p);
	}

	/// SV_VERBOSE(level, ...): SV_INFO when the log level is at least `level`
	pub(crate) fn verbose(&self, host: &mut (impl Host + ?Sized), level: i32, fmt: &str, args: &[Arg]) {
		if host.log_level() >= level {
			info(host, fmt, args);
		}
	}

	/// The close (its thread, if it had one, stopped and joined already): the map solve's summary, each station's
	/// solved pose against the calibrated map, and the map file written
	pub fn close(mut self, host: &mut (impl Host + ?Sized)) {
		let st = self.stats();
		info(
			host,
			"map-solve: %d solves, %d applied, %d not applied (moved too far); %ld snapshots from %ld slow poses; %d \
			 moved-station flags, %d relocations",
			&[
				Arg::I(st.solves as i64),
				Arg::I(st.applied as i64),
				Arg::I(st.rejected_moves as i64),
				Arg::I(st.snapshots),
				Arg::I(st.slow_poses),
				Arg::I(st.suspects as i64),
				Arg::I(st.relocations as i64),
			],
		);
		let mut l = 0;
		while (l as i32) < host.active_lighthouses() && l < MS_MAXLH {
			if !self.have_target[l] || !self.have_start[l] {
				l += 1;
				continue;
			}
			let (t, s) = (self.target[l], self.start[l]);
			let dp = dist3d(&t.pos, &s.pos);
			let qa = quatinnerproduct(&t.rot, &s.rot).abs();
			let mode = host.bsd_mode(l);
			info(
				host,
				"map-solve: LH %d (ch %d) solved pose %.1f mm / %.2f mrad from the calibrated map %+le   %+le   %+le\t%+le   %+le   %+le   %+le",
				&[
					Arg::I(l as i64),
					Arg::I(mode as i64),
					Arg::F(dp * 1e3),
					Arg::F(2e3 * c_acos(if qa > 1.0 { 1.0 } else { qa })),
					Arg::F(t.pos[0]),
					Arg::F(t.pos[1]),
					Arg::F(t.pos[2]),
					Arg::F(t.rot[0]),
					Arg::F(t.rot[1]),
					Arg::F(t.rot[2]),
					Arg::F(t.rot[3]),
				],
			);
			l += 1;
		}
		self.save_map_file(host, false);
	}
}
