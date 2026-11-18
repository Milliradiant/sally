//! The checks on what a session recorded: test_compare.py's (its names, its windows and its limits where it compared
//! by distance), exact where the record allows it (a pose vrserver got against the pose fed in the same call, instead
//! of the fed poses interpolated at its time), and those the scripted scene makes possible (where each device was when
//! SteamVR got it).

use std::sync::atomic::Ordering::SeqCst;

use driver_sally::openvr::DriverPose_t;
use sally_math::math3d::{cross, fit_rigid, inverse, len, qconj, qmul, qrot, xform, Pose, Q4, V3};

use crate::host::{quat, v3, world_head, Call, Server};
use crate::runtime::AGE;
use crate::scene::{
	angle, clean, shown, valve_frame, valve_head, world_from_driver, world_from_driver_moved, yup, yup_q, DEVICES,
};
use crate::session::{wfd_at, Record, Schedule, BOOTED, OTHERS};

pub struct Checks {
	pub lines: Vec<(bool, String)>,
}

impl Checks {
	pub fn check(&mut self, ok: bool, what: String) {
		self.lines.push((ok, what));
	}

	pub fn failures(&self) -> usize {
		self.lines.iter().filter(|l| !l.0).count()
	}
}

fn median(v: &[f64]) -> f64 {
	if v.is_empty() {
		return f64::NAN;
	}
	let mut s = v.to_vec();
	s.sort_by(f64::total_cmp);
	s[s.len() / 2]
}

fn max(v: &[f64]) -> f64 {
	v.iter().copied().fold(f64::NAN, f64::max)
}

fn mm(x: f64) -> f64 {
	1e3 * x
}

/// The bytes of a pose
fn bytes(p: &DriverPose_t) -> &[u8] {
	// SAFETY: a DriverPose_t is plain data, every byte of it written (the defaults and copies made in this test)
	unsafe { std::slice::from_raw_parts((p as *const DriverPose_t).cast::<u8>(), std::mem::size_of::<DriverPose_t>()) }
}

/// Where SteamVR should show a device at a time when it shows the runtime's pose: the clean pose with Valve's
/// world-from-driver (for the devices on its driver space: the frame's)
fn clean_world(sch: &Schedule, dev: usize, t: f64) -> V3 {
	xform(&wfd_at(Some(sch), t), clean(dev, t))
}

/// How far a pose of the runtime's that SteamVR got is from where it should show the device at the pose's own time
/// (its time plus its poseTimeOffset: the hand-over's timing is checked apart, this is the mapping)
fn off_runtime(sch: &Schedule, dev: usize, x: &Call) -> f64 {
	len(world_head(&x.pose) - clean_world(sch, dev, x.t + x.pose.poseTimeOffset))
}

/// A pose's world-from-driver
fn wfd_of(p: &DriverPose_t) -> Pose {
	Pose { p: v3(&p.vecWorldFromDriverTranslation), q: quat(&p.qWorldFromDriverRotation) }
}

/// The head's rotation in the world: world-from-driver * driver pose * driver-from-head
fn world_q(p: &DriverPose_t) -> Q4 {
	qmul(qmul(quat(&p.qWorldFromDriverRotation), quat(&p.qRotation)), quat(&p.qDriverFromHeadRotation))
}

pub fn run(r: &Record) -> Checks {
	let mut c = Checks { lines: Vec::new() };
	let s: &Server = r.server;
	let log = s.log.lock().unwrap().clone();
	let calls = s.calls.lock().unwrap().clone();
	let has = |text: &str| log.iter().any(|l| l.1.contains(text));
	let of = |id: u32, a: f64, b: f64| -> Vec<&Call> {
		calls.iter().filter(|x| x.id == id && a <= x.t && x.t < b).collect()
	};
	let fed_of = |x: &Call| x.fed.map(|i| &r.fed[i]);
	let untouched = |x: &Call| fed_of(x).is_some_and(|f| bytes(&f.pose) == bytes(&x.pose));

	// loading
	c.check(
		r.factory_other == (true, 105),
		"HmdDriverFactory: IServerTrackedDeviceProvider_004, and nothing for _003 (VRInitError_Init_InterfaceNotFound)"
			.into(),
	);
	let header = [
		"IVRSettings_003",
		"ITrackedDeviceServerDriver_005",
		"IVRDisplayComponent_003",
		"IVRDriverDirectModeComponent_009",
		"IVRCameraComponent_003",
		"IServerTrackedDeviceProvider_004",
		"IVRWatchdogProvider_001",
		"IVRVirtualDisplay_002",
		"IVRDriverManager_001",
		"IVRResources_001",
		"IVRCompositorPluginProvider_001",
		"IVRIPCResourceManagerClient_003",
	];
	c.check(r.versions == header, format!("GetInterfaceVersions: OpenVR 2.15.6's ({} interfaces)", r.versions.len()));
	c.check(r.init == 0, format!("Init: {} (VRInitError_None)", r.init));
	let hooks = ["006", "005", "004"].map(|v| format!("hooked IVRServerDriverHost_{v}::TrackedDevicePoseUpdated"));
	c.check(hooks.iter().all(|h| has(h)), "pose hook installed (IVRServerDriverHost_006, _005 and _004)".into());
	let identified = DEVICES.iter().all(|d| has(&format!("Valve device {} = {} (lighthouse)", d.id, d.serial)));
	let others = OTHERS.iter().any(|o| has(&format!("Valve device {} ", o.0)));
	c.check(
		identified && !others,
		"Valve devices identified (the four trackers and controllers; not the headset, the base station or the device \
		 whose properties do not read)"
			.into(),
	);
	c.check(
		DEVICES.iter().all(|d| has(&format!("{} is on dongle {}", d.serial, d.dongle)))
			&& has("Valve's driver and the runtime name the dongles alike"),
		"each device's dongle read from Valve's driver, and the runtime's found to be named alike".into(),
	);
	// which device each dongle carries, to the runtime: the four, each with Valve's dongle for it, and nothing else
	let consoles = r.consoles.join("\n");
	let pairs: Vec<&str> = consoles.lines().filter(|l| l.starts_with("got #p pair ")).collect();
	let pair_of = |d: &crate::scene::Device| format!("got #p pair {} {}", d.dongle, d.serial);
	c.check(
		DEVICES.iter().all(|d| pairs.contains(&pair_of(d).as_str()))
			&& pairs.iter().all(|l| DEVICES.iter().any(|d| *l == pair_of(d)))
			&& pairs.len() > 40,
		format!(
			"the runtime told which device each dongle carries, as Valve's driver says ({} pair lines, the four \
			 devices' and no others)",
			pairs.len()
		),
	);
	let Some(sch) = r.schedule else {
		c.check(false, "frame fit ready: never".into());
		return c;
	};
	let ready = log.iter().find(|l| l.1.contains("frame fit ready")).map(|l| l.1.clone()).unwrap_or_default();
	let first = calls.iter().find(|x| DEVICES.iter().any(|d| d.id == x.id)).map_or(f64::NAN, |x| x.t);
	c.check(
		sch.ready - first >= 20.0,
		format!(
			"frame fit ready {:.1} s after Valve's first pose (pairs only after 20 s of Valve's tracking): {}",
			sch.ready - first,
			ready.trim_start_matches("sally: ")
		),
	);
	let counts: Vec<usize> = DEVICES.iter().map(|d| of(d.id, 0.0, r.end).len()).collect();
	c.check(counts.iter().all(|&n| n > 100), format!("devices seen on both sides ({counts:?} poses)"));
	c.check(
		calls.iter().all(|x| x.size as usize == std::mem::size_of::<DriverPose_t>()),
		format!("every pose came with a DriverPose_t's size ({} poses)", calls.len()),
	);

	// the windows SteamVR shows sally steadily in, Valve's ("seen valve"), the runtime gone and back
	let sally_windows: Vec<(f64, f64)> = [
		Some((sch.ready + 1.0, sch.ready + 3.0)),
		Some((sch.seen_sally + 1.0, sch.kill)),
		r.back.map(|b| (b + 1.5, r.end)),
	]
	.into_iter()
	.flatten()
	.collect();
	let in_sally = |t: f64| sally_windows.iter().any(|&(a, b)| a <= t && t < b);
	let back = r.back.unwrap_or(f64::NAN);
	for (i, d) in DEVICES.iter().enumerate() {
		let before = of(d.id, 0.0, sch.ready);
		c.check(
			!before.is_empty() && before.iter().all(|x| untouched(x)),
			format!(
				"{} before the fit: Valve's poses passed through untouched ({} of {}, byte for byte)",
				d.serial,
				before.iter().filter(|x| untouched(x)).count(),
				before.len()
			),
		);
		let shown: Vec<&Call> =
			of(d.id, 0.0, r.end).into_iter().filter(|x| in_sally(x.t) && x.fed.is_some() && x.t < sch.kill).collect();
		let e_clean: Vec<f64> = shown.iter().map(|x| off_runtime(&sch, i, x)).collect();
		let e_valve: Vec<f64> =
			shown.iter().map(|x| len(world_head(&x.pose) - fed_of(x).unwrap().head_world)).collect();
		c.check(
			shown.len() > 100 && median(&e_clean) < 0.02 && median(&e_valve) > 0.015,
			format!(
				"{} sally shown, mapped into Valve's frame (median {:.1} mm from the runtime's pose at its time, \
				 {:.1} mm from Valve's)",
				d.serial,
				mm(median(&e_clean)),
				mm(median(&e_valve))
			),
		);
		let valve = of(d.id, sch.seen_valve + 0.5, sch.seen_sally - 0.5);
		c.check(
			!valve.is_empty() && valve.iter().all(|x| untouched(x)),
			format!("{} \"seen valve\": Valve's poses again, untouched ({} poses)", d.serial, valve.len()),
		);
		// V lines: each against the fed pose of its call (the last one fed before the line came), in the frame's driver
		// space (Valve's head in the world, its own world-from-driver applied, without the frame's): where it is also
		// while Valve has it in a driver space of its own (booted, lost)
		let fed_dev: Vec<_> = r.fed.iter().filter(|f| f.dev == Some(i)).collect();
		let ev: Vec<f64> = r
			.panel
			.iter()
			.filter_map(|(t, l)| {
				let f: Vec<&str> = l.split(' ').collect();
				(f.len() >= 11 && f[0] == "V" && f[1] == d.serial).then(|| {
					let k = fed_dev.partition_point(|x| x.t <= *t);
					let p = V3::new(f[4].parse().unwrap(), f[5].parse().unwrap(), f[6].parse().unwrap());
					if k == 0 {
						f64::INFINITY
					} else {
						let f = fed_dev[k - 1];
						len(p - xform(&inverse(&wfd_at(Some(&sch), f.t)), f.head_world))
					}
				})
			})
			.collect();
		c.check(
			ev.len() > 50 && median(&ev) < 0.0001,
			format!(
				"{} V lines carry Valve's head pose, in the frame's driver space ({} lines, median {:.3} mm)",
				d.serial,
				ev.len(),
				mm(median(&ev))
			),
		);
	}

	// Valve loses a device: 1 m off in its driver space, its world-from-driver moved with it
	let (ld, la, lb) = sch.lose;
	let lost = of(DEVICES[ld].id, la + 0.2, lb);
	let el: Vec<f64> = lost.iter().map(|x| off_runtime(&sch, ld, x)).collect();
	c.check(
		lost.len() > 100 && max(&el) < 0.05,
		format!(
			"{} Valve lost it (pose 1 m off, world-from-driver moved): SteamVR stays on the runtime's pose \
			 (max {:.1} mm)",
			DEVICES[ld].serial,
			mm(max(&el))
		),
	);
	c.check(
		has(&format!("{} Valve's world-from-driver is", DEVICES[ld].serial)),
		format!("{} the moved world-from-driver is logged and not followed", DEVICES[ld].serial),
	);

	// Valve boots a device in a driver space of its own: sally's pose on the frame's world-from-driver all the same (on
	// the one Valve has the device on it would show metres from where it is)
	let bd = &DEVICES[BOOTED];
	let boot: Vec<f64> = of(bd.id, sch.ready + 1.0, sch.booted_until)
		.iter()
		.filter(|x| x.fed.is_some())
		.map(|x| off_runtime(&sch, BOOTED, x))
		.collect();
	c.check(
		boot.len() > 100
			&& max(&boot) < 0.05
			&& has(&format!("the frame: Valve's world-from-driver of {}", bd.serial))
			&& has("the frame: most of Valve's devices are on another world-from-driver (2 against 1"),
		format!(
			"{} booted by Valve in a driver space of its own, the frame first on its world-from-driver and then on the \
			 others': SteamVR shows sally's pose where it is (median {:.1} mm, max {:.1} mm)",
			bd.serial,
			mm(median(&boot)),
			mm(max(&boot))
		),
	);

	// a Tundra's head frame: SteamVR gets sally's pose in Valve's (turned 180 deg about its Y from the runtime's); where
	// Valve's driver pose is not at the runtime's IMU, the runtime's head frame stays
	let (ti, td) = DEVICES.iter().enumerate().find(|(_, d)| d.head_fix.q.w.abs() < 0.5).unwrap();
	let tq: Vec<(f64, f64)> = of(td.id, 0.0, sch.kill)
		.iter()
		.filter(|x| in_sally(x.t) && x.fed.is_some() && !untouched(x))
		.map(|x| {
			// Valve's head axes in the world at the pose's own time, and the runtime's there
			let at = x.t + x.pose.poseTimeOffset;
			let valve = qmul(wfd_at(Some(&sch), at).q, valve_head(ti, at).q);
			let runtime = qmul(valve, qconj(td.head_fix.q));
			(angle(qmul(world_q(&x.pose), qconj(valve))), angle(qmul(world_q(&x.pose), qconj(runtime))))
		})
		.collect();
	let (tv, tr): (Vec<f64>, Vec<f64>) = tq.into_iter().unzip();
	c.check(
		tv.len() > 100
			&& median(&tv) < 0.035
			&& median(&tr) > 3.0
			&& has(&format!("{} head frame: Valve's is turned 180.0 deg", td.serial))
			&& DEVICES
				.iter()
				.filter(|d| d.serial != td.serial)
				.all(|d| has(&format!("{} head frame: Valve's driver-from-head puts the head", d.serial))),
		format!(
			"{} (a Tundra's head frame): SteamVR gets sally's pose in Valve's head frame ({:.1} deg from Valve's head \
			 axes, {:.0} from the runtime's); the others', whose driver pose is not at the runtime's IMU, kept",
			td.serial,
			median(&tv).to_degrees(),
			median(&tr).to_degrees()
		),
	);

	// Valve's world moves: sally's poses move with it
	let moved: Vec<f64> = DEVICES
		.iter()
		.enumerate()
		.flat_map(|(i, d)| {
			of(d.id, sch.world_move + 0.05, sch.kill).into_iter().filter(|x| x.fed.is_some()).map(move |x| (i, x))
		})
		.map(|(i, x)| off_runtime(&sch, i, x))
		.collect();
	c.check(
		moved.len() > 400 && median(&moved) < 0.02 && has("Valve's world moved"),
		format!(
			"Valve's world moved (every device's world-from-driver, their poses in its driver space as they were): \
			 followed, SteamVR shows sally's poses in the moved world (median {:.1} mm)",
			mm(median(&moved))
		),
	);
	c.check(
		!has("more than the pose we sent") && !has("although we sent it tracking"),
		"what SteamVR has matches what we sent (no \"SteamVR moved / shows\" lines)".into(),
	);

	// Valve pauses a device 20 ms in every 100 ms
	let (pd, pa, pb) = sch.pause;
	let times: Vec<f64> = of(DEVICES[pd].id, pa, pb + 0.01).iter().map(|x| x.t).collect();
	let mut gaps = Vec::new();
	let mut k = 0;
	while pa + 0.1 * k as f64 + 0.02 <= pb {
		let a = pa + 0.1 * k as f64;
		let b = a + 0.02 + 0.004;
		let mut ts: Vec<f64> = vec![a];
		ts.extend(times.iter().copied().filter(|&t| a <= t && t < b));
		ts.push(b);
		gaps.push(ts.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max));
		k += 1;
	}
	let ep: Vec<f64> = of(DEVICES[pd].id, pa + 0.5, pb).iter().map(|x| off_runtime(&sch, pd, x)).collect();
	c.check(
		median(&gaps) < 0.011 && median(&ep) < 0.02,
		format!(
			"{} Valve pausing 20 ms at a time: the runtime's pose goes on being handed over (longest wait median {:.1} \
			 ms, max {:.1} ms; pose vs runtime median {:.1} mm)",
			DEVICES[pd].serial,
			mm(median(&gaps)),
			mm(max(&gaps)),
			mm(median(&ep))
		),
	);

	// Valve goes quiet on a device
	let (qd, qa, qb) = sch.quiet;
	let quiet = of(DEVICES[qd].id, qa + 0.3, qb - 0.3);
	let rate = quiet.len() as f64 / (qb - qa - 0.6);
	c.check(
		rate > 50.0 && quiet.iter().all(|x| x.pose.poseIsValid != 0 && x.fed.is_none()),
		format!("{} Valve quiet for 3 s: runtime poses pushed ({rate:.0}/s, all valid)", DEVICES[qd].serial),
	);

	// the runtime killed: Valve's poses, slid onto; and back
	for d in &DEVICES {
		let gone = of(d.id, sch.kill + 0.5, back - 0.2);
		c.check(
			gone.len() > 50 && gone.iter().all(|x| x.pose.poseIsValid != 0),
			format!(
				"{} runtime gone: SteamVR keeps getting valid poses ({} rows, {:.0}% valid)",
				d.serial,
				gone.len(),
				100.0 * gone.iter().filter(|x| x.pose.poseIsValid != 0).count() as f64 / gone.len().max(1) as f64
			),
		);
		c.check(
			!gone.is_empty() && gone.iter().all(|x| untouched(x)),
			format!("{} runtime gone: SteamVR shows Valve's pose (untouched)", d.serial),
		);
		// the slide onto it: what SteamVR got against Valve's pose goes from their offset to 0 without a step over
		// what a blend step and the device's own motion make
		let slide: Vec<V3> = of(d.id, sch.kill - 0.3, sch.kill + 0.6)
			.iter()
			.filter(|x| x.fed.is_some() && x.pose.poseIsValid != 0)
			.map(|x| world_head(&x.pose) - fed_of(x).unwrap().head_world)
			.collect();
		let steps: Vec<f64> = slide.windows(2).map(|w| len(w[1] - w[0])).collect();
		c.check(
			slide.len() > 5 && max(&steps) < 0.012,
			format!(
				"{} transition to Valve is blended (largest residual step {:.1} mm, offset {:.0} -> {:.0} mm)",
				d.serial,
				mm(max(&steps)),
				mm(slide.first().map_or(f64::NAN, |v| len(*v))),
				mm(slide.last().map_or(f64::NAN, |v| len(*v)))
			),
		);
	}
	c.check(has("SteamVR gets Valve's pose (sally stale"), "fallback logged".into());
	// the restarted runtime has the first two devices on each other's dongles (each one's serial with the other's
	// motion) until the driver's pair puts them right: SteamVR keeps Valve's poses for them, then has the runtime's
	let fixed = r.runs.iter().filter_map(|x| x.fixed).find(|&t| t > sch.kill);
	let fx = fixed.unwrap_or(f64::NAN);
	c.check(
		fixed.is_some_and(|t| t - back < 3.0),
		format!(
			"the restarted runtime's stale dongles put right by the driver's pair {:.1} s after it came back",
			fx - back
		),
	);
	for (i, d) in DEVICES.iter().enumerate().take(2) {
		let stale = of(d.id, back, fx);
		c.check(
			!stale.is_empty() && stale.iter().all(|x| untouched(x)),
			format!(
				"{} on the other device's dongle in the runtime: SteamVR shows Valve's pose, untouched ({} poses), not \
				 the runtime's (the other device's motion)",
				d.serial,
				stale.len()
			),
		);
		c.check(
			has(&format!("{} is on dongle {}, the runtime has it on {}", d.serial, d.dongle, DEVICES[1 - i].dongle))
				&& has(&format!("{}: the runtime has it on dongle {} too", d.serial, d.dongle)),
			format!("{} the identity check logged, closed and open again", d.serial),
		);
	}
	// from when a device has the runtime's pose again: 1.5 s after the runtime came back, and for the two on the
	// wrong dongles 1.5 s after the pair (its device list follows within 1 s, then the blend)
	let shown_from = |i: usize| if i < 2 { fx + 1.5 } else { back + 1.5 };
	let e_back: Vec<f64> = DEVICES
		.iter()
		.enumerate()
		.map(|(i, d)| {
			let e: Vec<f64> = of(d.id, shown_from(i), r.end)
				.iter()
				.filter(|x| x.fed.is_some())
				.map(|x| off_runtime(&sch, i, x))
				.collect();
			if e.len() > 100 {
				median(&e)
			} else {
				f64::NAN
			}
		})
		.collect();
	c.check(
		e_back.iter().all(|&e| e < 0.02),
		format!(
			"runtime back (its codenames handed out the other way round, two devices on each other's dongles until the \
			 pair): SteamVR on the runtime's poses again for all four ({} mm)",
			e_back.iter().map(|e| format!("{:.1}", mm(*e))).collect::<Vec<_>>().join(", ")
		),
	);

	// the panels and the command port
	let clines = r.panel.iter().filter(|l| l.1.starts_with("C ")).count();
	c.check(
		r.panel.iter().any(|l| l.1.starts_with("C seen sally fit 1")),
		format!("status lines reach the panel port ({clines})"),
	);
	let want = [
		("status", "C seen sally fit 1 "),
		("seen valve", "C seen valve fit 1 "),
		("garbage", "C seen valve fit 1 "),
		("refit", "C seen valve fit 1 "),
		("seen sally", "C seen sally fit 1 "),
	];
	c.check(
		r.replies.len() == want.len() && r.replies.iter().zip(want).all(|(a, b)| a.0 == b.0 && a.1.starts_with(b.1)),
		format!(
			"the command port answers each command with the status after it ({})",
			r.replies.iter().map(|x| format!("{} -> \"{}\"", x.0, x.1)).collect::<Vec<_>>().join("; ")
		),
	);
	c.check(
		has("SteamVR now shows Valve's poses") && has("SteamVR now shows sally's poses"),
		"\"seen valve\" and \"seen sally\" logged".into(),
	);
	// the output layer's settings: set from the command port, the list back, the file written
	if r.schedule.is_some() {
		// every class has every setting, at its default: nothing on (the prediction is the pose's own age)
		let want = [
			"O trackers recovery_blend_ms 80",
			"O controllers recovery_blend_ms 0",
			"O headset prediction_ms 0",
			"O controllers prediction_ms 9",
			"O trackers smoothing_ms 0",
			"O headset smoothing_ms 0",
			"O controllers velocity_boost 0",
			"O trackers offset_rz_deg 0",
			"O controllers rail 0",
			"O controllers rail_speed 1.5",
			"O trackers rail_boost 50",
			"O controllers dash 0",
			"O controllers dash_percent 100",
			"O controllers dash_back_ms 400",
			"O end",
		];
		c.check(
			want.iter().all(|w| r.output_reply.iter().any(|l| l == w))
				&& r.output_reply.last().is_some_and(|l| l.starts_with("C "))
				&& r.output_file.contains("trackers.recovery_blend_ms=80\n")
				&& r.output_file.contains("headset.prediction_ms=0\n")
				&& has("sally: output trackers.recovery_blend_ms = 80"),
			format!(
				"\"output trackers recovery_blend_ms 80\": the O lines of every class and setting and the status back \
				 ({} lines), sally_output.txt written",
				r.output_reply.len()
			),
		);
		// a toy is for the session it was switched on in: off again as SteamVR closes
		c.check(
			has("sally: output headset.rail = 1")
				&& has("sally: the toys off as SteamVR closes (headset.rail)")
				&& r.output_file.contains("headset.rail=0\n")
				&& r.output_file.contains("trackers.recovery_blend_ms=80\n"),
			"a toy switched on in the session (\"output headset rail 1\") is off again after Cleanup, in the log and \
			 in sally_output.txt, the other settings kept"
				.into(),
		);
	}
	// the frame lines: the fit SteamVR shows sally through (the runtime's world in OpenVR's axes to Valve's driver
	// space: the scene's valve_frame) and the frame's world-from-driver, the scene's and then the moved one
	let frames: Vec<(f64, Vec<f64>)> = r
		.panel
		.iter()
		.filter(|l| l.1.starts_with("F "))
		.map(|(t, l)| (*t, l.split(' ').skip(1).map(|x| x.parse().unwrap_or(f64::NAN)).collect()))
		.filter(|(_, v): &(f64, Vec<f64>)| v.len() == 18)
		.collect();
	let pose = |v: &[f64]| Pose { p: V3::new(v[0], v[1], v[2]), q: Q4::new(v[3], v[4], v[5], v[6]) };
	let frame_ok = |a: f64, b: f64, wfd: Pose| {
		let f: Vec<&Vec<f64>> = frames.iter().filter(|x| a <= x.0 && x.0 < b).map(|x| &x.1).collect();
		!f.is_empty()
			&& f.iter().all(|v| {
				let (t, w, k) = (pose(&v[3..10]), pose(&v[11..18]), valve_frame());
				v[0] == 1.0
					&& v[10] == 1.0 && len(t.p - k.p) < 0.02
					&& angle(qmul(t.q, qconj(k.q))) < 0.02
					&& len(w.p - wfd.p) < 1e-4
					&& angle(qmul(w.q, qconj(wfd.q))) < 1e-4
			})
	};
	let refitting = frames.iter().any(|(t, v)| *t > sch.refit && v[2] == 1.0);
	c.check(
		frame_ok(sch.ready + 1.5, sch.refit, world_from_driver())
			&& frame_ok(sch.world_move + 0.1, r.end, world_from_driver_moved())
			&& refitting
			&& has("re-align asked")
			&& has("re-aligned ("),
		format!(
			"frame lines reach the panel port ({}): the fit SteamVR shows sally through and the frame's \
			 world-from-driver, the moved one after Valve's world moved; \"refit\" (the panel's re-align): the pairs anew, \
			 then their fit",
			frames.len()
		),
	);

	// the hand-over: each pose of sally's as SteamVR got it is the device as it was at the time the pose declares
	// (with the carry: the time SteamVR got it). Between two refits the frame fit is one rigid transform (it changes
	// only at a refit, which the status line to the panels follows at once): over each such stretch of the windows,
	// after one rigid fit of the poses to the scene's, the residuals of the position, the orientation, the velocity and
	// the body rate
	let refits: Vec<f64> = r.panel.iter().filter(|l| l.1.starts_with("C ")).map(|l| l.0).collect();
	let mut stretches = Vec::new();
	for &(a, b) in &sally_windows {
		let mut cuts = vec![a];
		cuts.extend(refits.iter().copied().filter(|&t| a < t && t < b));
		cuts.push(b);
		stretches.extend(cuts.windows(2).map(|w| (w[0] + 0.005, w[1] - 0.005)));
	}
	// (medians over the poses: a report the receive thread got late is stamped late, and is a few ms off by that alone)
	let mut res = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
	// and the record each is on (driver.valve_record): Valve's driver-from-head for the device, the driver pose where
	// Valve's own of the same call has the device, in the world
	let mut rec = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
	let mut current = (0, 0);
	for (a, b) in stretches {
		let mut src = Vec::new();
		let mut dst = Vec::new();
		let mut rows = Vec::new();
		for (i, d) in DEVICES.iter().enumerate() {
			// after the restart, from when the device has the runtime's pose again
			let a = if a > sch.kill { a.max(shown_from(i)) } else { a };
			for x in of(d.id, a, b).into_iter().filter(|x| x.fed.is_some() && !untouched(x)) {
				let m = shown(i, x.t + x.pose.poseTimeOffset);
				src.push(yup(m.p));
				// the head, in the driver space: the driver pose with its driver-from-head on top
				dst.push(
					v3(&x.pose.vecPosition) + qrot(quat(&x.pose.qRotation), v3(&x.pose.vecDriverFromHeadTranslation)),
				);
				if let Some(f) = fed_of(x).filter(|f| f.pose.poseIsValid != 0) {
					let (xw, fw) = (wfd_of(&x.pose), wfd_of(&f.pose));
					rec.0
						.push(len(v3(&x.pose.vecDriverFromHeadTranslation) - v3(&f.pose.vecDriverFromHeadTranslation)));
					rec.1.push(angle(qmul(
						quat(&x.pose.qDriverFromHeadRotation),
						qconj(quat(&f.pose.qDriverFromHeadRotation)),
					)));
					rec.2.push(len(xform(&xw, v3(&x.pose.vecPosition)) - xform(&fw, v3(&f.pose.vecPosition))));
					rec.3.push(angle(qmul(
						qmul(xw.q, quat(&x.pose.qRotation)),
						qconj(qmul(fw.q, quat(&f.pose.qRotation))),
					)));
				}
				rows.push((x, m));
				current.0 += (x.pose.poseTimeOffset == 0.0) as usize;
				current.1 += 1;
			}
		}
		if rows.len() < 50 {
			continue;
		}
		let Some((t, _)) = fit_rigid(&src, &dst) else { continue };
		for (k, (x, m)) in rows.into_iter().enumerate() {
			// the head's orientation, the head point's velocity (the driver pose's origin's, carried: v + w x r) and
			// the body rate in the head's axes
			let (qd, qh) = (quat(&x.pose.qRotation), quat(&x.pose.qDriverFromHeadRotation));
			let w_drv = v3(&x.pose.vecAngularVelocity);
			let arm = qrot(qd, v3(&x.pose.vecDriverFromHeadTranslation));
			let head_v = v3(&x.pose.vecVelocity) + cross(qrot(qd, w_drv), arm);
			res.0.push(len(xform(&t, src[k]) - dst[k]));
			res.1.push(angle(qmul(qmul(qd, qh), qconj(qmul(t.q, yup_q(m.q))))));
			res.2.push(len(head_v - qrot(t.q, yup(m.v))));
			res.3.push(len(qrot(qconj(qh), w_drv) - qrot(qconj(m.q), m.w)));
		}
	}
	let current_share = current.0 as f64 / current.1.max(1) as f64;
	c.check(
		res.0.len() > 1000
			&& median(&res.0) < 0.0005
			&& median(&res.1) < 0.001
			&& median(&res.2) < 0.005
			&& median(&res.3) < 0.005
			&& (!r.config.carry || current_share > 0.9),
		format!(
			"the hand-over: SteamVR gets each device as it was at the pose's time ({}: {:.0}% declared current): \
			 position {:.2} mm after a rigid fit, orientation {:.2} mrad, velocity {:.1} mm/s, body rate {:.1} mrad/s \
			 (medians of {} poses; the runtime's state is {:.0} ms old)",
			if r.config.carry { "carried to it" } else { "declared as its age" },
			100.0 * current_share,
			mm(median(&res.0)),
			mm(median(&res.1)),
			mm(median(&res.2)),
			mm(median(&res.3)),
			res.0.len(),
			mm(AGE)
		),
	);

	c.check(
		rec.0.len() > 1000
			&& max(&rec.0) < 1e-9
			&& max(&rec.1) < 1e-9
			&& median(&rec.2) < 0.04
			&& median(&rec.3) < 0.03,
		format!(
			"sally's poses are on Valve's own record: Valve's driver-from-head for the device (within {:.1e} mm, {:.1e} \
			 mrad of it), the driver pose where Valve's own of that call has the device ({:.1} mm, {:.1} mrad, medians of \
			 {} poses; Valve's wobbles 25 mm), so a reader of the driver pose alone sees no turn between the two",
			mm(max(&rec.0)),
			mm(max(&rec.1)),
			mm(median(&rec.2)),
			mm(median(&rec.3)),
			rec.0.len()
		),
	);

	// the others' devices
	for &(id, iface, _) in &OTHERS {
		let all = of(id, 0.0, r.end);
		c.check(
			all.len() > 50 && all.iter().all(|x| untouched(x) && x.iface == iface),
			format!(
				"device {id} ({}) passes through untouched ({} poses through IVRServerDriverHost_{})",
				match id {
					0 => "the headset, another tracking system",
					5 => "a base station",
					_ => "its properties do not read",
				},
				all.len(),
				["006", "005", "004"][iface]
			),
		);
	}
	// the property reads (two a look: the tracking system, the serial): once for each of Valve's devices, and its
	// dongle at its first pose and then every second (once more when it comes back after a silence); for the others
	// again at the first pose `retry` s after the last look (0.25 s when they did not read, 2 s when they did but the
	// device is not one of ours), not on every pose
	let reads = s.reads.lock().unwrap().clone();
	let n = |id: u32| reads.get(&(1000 + id as u64)).copied().unwrap_or(0);
	let span = |id: u32| {
		let t: Vec<f64> = calls.iter().filter(|x| x.id == id).map(|x| x.t).collect();
		t.last().unwrap_or(&0.0) - t.first().unwrap_or(&0.0)
	};
	let expected = |id: u32, period: f64, retry: f64| 2.0 * (1.0 + (span(id) / (retry + period / 2.0)).floor());
	let others_ok = OTHERS.iter().all(|&(id, _, period)| {
		let retry = if id == 6 { 0.25 } else { 2.0 };
		(n(id) as f64 - expected(id, period, retry)).abs() <= 4.0
	});
	let dongle_reads: Vec<u32> = DEVICES.iter().map(|d| n(d.id).saturating_sub(2)).collect();
	let valve_ok = DEVICES.iter().zip(&dongle_reads).all(|(d, &k)| {
		let k = k as f64;
		k >= 0.9 * span(d.id) - 3.0 && k <= span(d.id) + 6.0
	});
	c.check(
		valve_ok && others_ok,
		format!(
			"properties read once for each of Valve's devices and its dongle about every second ({:?} dongle reads in \
			 {:.0} s), every 0.25 s for the device whose properties do not read ({} reads), every 2 s for the headset and \
			 the base station ({}, {}): not on every pose",
			dongle_reads,
			span(DEVICES[0].id),
			n(6),
			n(0),
			n(5)
		),
	);
	c.check(
		s.added.load(SeqCst) == 0 && s.writes.load(SeqCst) == 0,
		"the driver adds no devices and writes no properties (and calls none of vrserver's it should not: a call \
		 would have stopped the test)"
			.into(),
	);

	// the runtime: started, restarted, stopped
	if r.config.driver_starts_runtime {
		c.check(has("starting the runtime"), "driver started the runtime".into());
		c.check(
			has("the runtime exited (code -9), starting it again"),
			"driver restarted the runtime after the kill".into(),
		);
		let quit = r.runs.last().is_some_and(|x| x.quit.is_some());
		c.check(
			has("stopping the runtime") && has("runtime exited (code 0)") && quit && r.left_over.is_empty(),
			format!("driver stopped the runtime at exit (quit, exit code 0; left over: {:?})", r.left_over),
		);
		c.check(
			r.consoles.len() == r.runs.len() && r.consoles.iter().all(|t| t.starts_with("scripted runtime ")),
			format!("the runtime's console goes to driver.runtime_log, a file per start ({} starts)", r.runs.len()),
		);
	} else {
		c.check(has("driver.runtime_exe not set"), "the runtime started separately: the driver starts none".into());
		c.check(
			r.consoles.last().is_some_and(|t| t.contains("got unsub")),
			"the driver unsubscribes at Cleanup".into(),
		);
	}

	// the hooks at Cleanup
	let own = Server::own_pose_fn();
	match r.other_hook {
		None => c.check(
			hooks.iter().all(|h| has(&h.replace("hooked", "restored"))) && r.slots_after == [own; 3],
			"hook removed at Cleanup (restored in all three, the slots vrserver's again)".into(),
		),
		Some(other) => {
			c.check(
				has("IVRServerDriverHost_006::TrackedDevicePoseUpdated was hooked again after us; ours stays as a \
				     pass-through")
					&& has("restored IVRServerDriverHost_005")
					&& has("restored IVRServerDriverHost_004")
					&& r.slots_after == [other, own, own],
				"another driver hooked _006 after us: ours stays in its chain, _005 and _004 restored at Cleanup"
					.into(),
			);
			let ok = r.after_cleanup.is_some_and(|(sent, got)| got.is_some_and(|g| bytes(&g) == bytes(&sent)));
			c.check(ok, "after Cleanup, a pose through that chain reaches vrserver untouched".into());
		}
	}

	// the pose log: a row per frame and device, every column filled, our rotation a unit one
	let text = r.pose_log.as_deref().unwrap_or("");
	let header: Vec<&str> = text.lines().next().unwrap_or("").split(',').collect();
	let rows: Vec<Vec<&str>> = text.lines().skip(1).map(|l| l.split(',').collect()).collect();
	let col = |name: &str| header.iter().position(|&h| h == name);
	let whole = header.last() == Some(&"sfz") && rows.iter().all(|f| f.len() == header.len());
	let unit = rows.iter().all(|f| {
		let q: f64 = ["sqw", "sqx", "sqy", "sqz"]
			.iter()
			.map(|k| col(k).and_then(|i| f.get(i)).and_then(|x| x.parse::<f64>().ok()).unwrap_or(f64::NAN).powi(2))
			.sum();
		(1.0 - q).abs() < 1e-3
	});
	c.check(
		rows.len() > 100 && whole && unit,
		format!("pose log written ({} rows, rotations and SteamVR matrices in every row)", rows.len()),
	);
	c
}
