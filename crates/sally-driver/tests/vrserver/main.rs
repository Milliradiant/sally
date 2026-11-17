//! The driver in a stand-in vrserver (fake_vrserver.cpp and test_compare.py, which left with the C++ driver;
//! docs/sally_driver.md): the library cargo built for the tests, loaded as vrserver loads it (dlopen of
//! `bin/linux64/driver_<the manifest's name>.so` in a driver folder laid out from `package/`, HmdDriverFactory,
//! GetInterfaceVersions, Init, RunFrame at 90 Hz, Cleanup), with Valve's driver and the runtime played. Linux only, as
//! fake_vrserver was: the Windows-only paths (Winsock, VirtualProtect, CreateProcessW) and SteamVR itself are not
//! tested here.
//!
//! Valve's driver reports four devices through the hooked vtable, each pose the runtime's in a known frame with a 2.5
//! cm wobble and a driver-from-head offset; the headset, a base station and a device whose properties do not read
//! report too. The runtime is this test binary run as a scripted runtime (`runtime.rs`): it speaks the runtime
//! protocol over loopback UDP as `sally run` does, its Q lines made from the same scene, where test_compare.py replayed
//! a tap recording through the real runtime in real time (and derived Valve's poses from the runtime's). Two sessions
//! run at once, each with its own copy of the library (so its own state): one where the test runs the runtime, as by
//! hand, and another driver hooks the pose update after ours; one where the driver starts the runtime, starts it again
//! after the kill and stops it at Cleanup, with the hand-over's carry on (the live settings). See session.rs for the
//! schedule and checks.rs for what is checked. Three more run beside them with the virtual trackers (driver.virtual):
//! kept on, all five and the four without the chest, and off at the start (the default) then switched on, the model
//! process the driver starts played by this binary too (`run`, model.rs): virtual_session.rs. About 50 s.
//!
//! Each session's folder stays in cargo's target/tmp (vrserver-<session>): the driver folder as loaded, the driver's
//! log (vrserver.txt), the pose log, the runtimes' markers and consoles. VRSERVER_DRIVER names another build of the
//! driver to load instead of the one cargo built (the C++ driver built from git, say).

#[cfg(target_os = "linux")]
mod checks;
#[cfg(target_os = "linux")]
mod host;
#[cfg(target_os = "linux")]
mod model;
#[cfg(target_os = "linux")]
mod runtime;
#[cfg(target_os = "linux")]
mod scene;
#[cfg(target_os = "linux")]
mod session;
#[cfg(target_os = "linux")]
mod virtual_session;

/// Whether a test harness given these arguments would run a test named "vrserver" (filters, --exact, --skip,
/// --ignored; the values of the options that take one are not filters)
#[cfg(target_os = "linux")]
fn selected(args: &[String]) -> bool {
	let (mut filters, mut skips, mut exact, mut ignored) = (Vec::new(), Vec::new(), false, false);
	let mut it = args.iter().skip(1);
	while let Some(a) = it.next() {
		match a.as_str() {
			"--skip" => skips.extend(it.next()),
			"--test-threads" | "--logfile" | "--format" | "--color" | "-Z" => {
				it.next();
			}
			"--exact" => exact = true,
			"--ignored" => ignored = true,
			_ if !a.starts_with('-') => filters.push(a.as_str()),
			_ => {}
		}
	}
	let names = |f: &&str| if exact { *f == "vrserver" } else { "vrserver".contains(*f) };
	!ignored && (filters.is_empty() || filters.iter().any(names)) && !skips.iter().any(|s| names(&s.as_str()))
}

#[cfg(target_os = "linux")]
fn main() {
	let args: Vec<String> = std::env::args().collect();
	if args.get(1).map(String::as_str) == Some("--scripted-runtime") {
		runtime::main(&args);
	}
	// the virtual trackers' model process, as the driver starts it
	if args.get(1).map(String::as_str) == Some("run") {
		model::main(&args);
	}
	if !selected(&args) {
		return;
	}
	if args.iter().any(|a| a == "--list") {
		println!("vrserver: test");
		return;
	}
	let configs = [
		session::Config { name: "by-hand", driver_starts_runtime: false, carry: false },
		session::Config { name: "driver-started", driver_starts_runtime: true, carry: true },
	];
	let virtuals = [
		virtual_session::VConfig { name: "virtual", shown: &[0, 1, 2, 3, 4], option: None, start_on: true },
		virtual_session::VConfig {
			name: "virtual-four",
			shown: &[1, 2, 3, 4],
			option: Some("lknee,rknee,lelbow,relbow"),
			start_on: true,
		},
		virtual_session::VConfig {
			name: "virtual-off-at-start",
			shown: &[1, 2, 3, 4],
			option: Some("lknee,rknee,lelbow,relbow"),
			start_on: false,
		},
	];
	let (records, virtual_records): (Vec<session::Record>, Vec<virtual_session::VRecord>) = std::thread::scope(|s| {
		let vs: Vec<_> = virtuals.iter().map(|&c| s.spawn(move || virtual_session::run(c))).collect();
		let runs: Vec<_> = configs.iter().map(|&c| s.spawn(move || session::run(c))).collect();
		(runs.into_iter().map(|r| r.join().unwrap()).collect(), vs.into_iter().map(|r| r.join().unwrap()).collect())
	});
	let mut failures = 0;
	for r in &records {
		let c = checks::run(r);
		println!(
			"\n== session {} (the {}, carry {}): {}",
			r.config.name,
			if r.config.driver_starts_runtime { "driver starting the runtime" } else { "runtime started by hand" },
			if r.config.carry { "on" } else { "off" },
			r.library.display()
		);
		for (ok, what) in &c.lines {
			println!("{} {what}", if *ok { "PASS" } else { "FAIL" });
		}
		println!("{}: {} checks, {} failed", r.config.name, c.lines.len(), c.failures());
		failures += c.failures();
	}
	for v in &virtual_records {
		let c = virtual_session::check(v);
		println!(
			"\n== session {} (the virtual trackers, the driver starting the model process; driver.virtual_trackers {}; {})",
			v.config.name,
			v.config.option.unwrap_or("unset"),
			if v.config.start_on { "kept on" } else { "off at the start" }
		);
		for (ok, what) in &c.lines {
			println!("{} {what}", if *ok { "PASS" } else { "FAIL" });
		}
		println!("{}: {} checks, {} failed", v.config.name, c.lines.len(), c.failures());
		failures += c.failures();
	}
	let folders = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("vrserver-*");
	if failures > 0 {
		println!("\nvrserver: FAILED ({failures}); what each session saw is in {}", folders.display());
		std::process::exit(1);
	}
	println!("\nvrserver: ok ({})", folders.display());
}

#[cfg(not(target_os = "linux"))]
fn main() {}
