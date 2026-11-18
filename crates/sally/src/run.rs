//! `sally run`: survive-cli (survive-cli.c) with the C's threads. survive_init, survive_startup (the drivers in
//! the C's load order, a tap starting its own thread: the dongles' reader live, on Windows, or the recording's
//! playback), then survive-cli's main loop on the main thread: survive_poll and the --stop-after check, until a
//! driver's poll ends it (the protocol's quit, a playback that has finished), an error is raised, --stop-after's run
//! time passes or a stop comes (Ctrl+C or the console closing on Windows; SIGINT or SIGTERM on Linux, where a second
//! one exits at once with -1); then survive_close, which joins each thread without the context lock.
//!
//! The main thread holds the context lock (`sally_runtime::context`) from survive_init on, as the C's semaphore starts
//! taken, so a driver's thread waits for the main loop's first poll; survive_poll lets it go only for what is left of
//! its 10 ms (poll_min_time_ms) on os_generic's clock, sleeping with OGUSleep (on Windows, Sleep's milliseconds on the
//! system's timer tick). The threads do their work while it sleeps.
//!
//! The map solve solves on a thread of its own (map-solve-threaded, as the C's), unless the playback factor is 0: a
//! due solve wakes it, and it takes the context lock where the C's takes it, so its results land wherever it gets the
//! lock, as in the C.
//!
//! What a run records depends on where the polls land only through what polls: the runtime protocol (its commands and
//! summaries), --stop-after and the stops, and an error raised (the run ends at the next poll). At playback-factor 0,
//! as the bench replays, the map solve solves inline and the playback reads as fast as it can, and a run gives the C's
//! recording byte for byte, as `sally replay` does.
//!
//! Not here: the button servicer's thread (`Runtime::button` runs where the C posts the event to it), the callback
//! statistics (output-callback-stats: the hooks are not timed), the drivers not ported (the .rec playback, the USB
//! capture replay, the simulator) and the LH1 disambiguator's flush at close.

use std::fs::File;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use sally_hid::live::Live;
use sally_hid::replay::Playback;
use sally_hid::tap::{Mode, Tap};
use sally_math::printf::{sprintf, Arg};
use sally_runtime::config::{SC_GET, SC_SETCONFIG};
use sally_runtime::context::{lock, Context};
use sally_runtime::runtime::error;
use sally_runtime::{driver, tap};

use crate::autocal::{self, Auto};
use crate::startup::{self, text, DRIVER_ERROR, DRIVER_NORMAL, DRIVER_PASSIVE};

mod sys;

/// survive-cli's keepRunning: cleared by a stop
static KEEP_RUNNING: AtomicBool = AtomicBool::new(true);

/// ctx->poll_min_time_ms, as survive_init_internal sets it
const POLL_MIN_TIME_MS: u64 = 10;

/// A tap driver's thread, as the main thread holds it (survive_add_threaded_driver's driver)
enum TapThread {
	Live(Live<Context>),
	Playback(Playback<Context>),
}

impl TapThread {
	/// threaded_driver_poll: -1 once the thread has stopped (a playback at its end or past run-time; the live reader
	/// stops only at the close)
	fn poll(&self) -> i32 {
		match self {
			TapThread::Live(_) => 0,
			TapThread::Playback(p) => {
				if p.running() {
					0
				} else {
					-1
				}
			}
		}
	}

	/// The tap's thread stopped and joined, and the tap closed; called without the context lock
	fn stop(self) {
		match self {
			TapThread::Live(l) => l.stop(),
			TapThread::Playback(p) => p.stop(),
		}
	}
}

/// survive-cli's main: the stops caught, survive_init, survive_startup, the main loop, "Shutting down cleanly",
/// survive_close. `argv` is the command line as main's argv has it (the program first)
pub fn main(argv: &[Vec<u8>]) -> i32 {
	sys::catch_stops();
	// sally: the room's calibration kept up (live, with a config file and a recording): a ready one taken up before the
	// config is read
	let auto = Auto::new(argv).map(|mut a| {
		a.adopt();
		Arc::new(Mutex::new(a))
	});
	let rt = match startup::init(argv) {
		Ok(rt) => rt,
		Err(code) => return code,
	};
	let ctx = Arc::new(Mutex::new(Context::new(rt)));
	let mut g = lock(&ctx);
	if let Some(a) = &auto {
		locked(a).after_init(&mut g.rt);
		g.rt.calibrator = Some(Box::new(autocal::Handle(a.clone())));
	}
	let taps = survive_startup(&ctx, &mut g);
	if let Some(a) = &auto {
		locked(a).after_start(&mut g.rt);
	}
	// survive_install_button_fn(ctx, button_process): the menu button's process is the runtime's own
	let stop_after = g.rt.config.configf("stop-after", SC_GET, -1.0);
	while KEEP_RUNNING.load(Ordering::SeqCst) {
		let r;
		(g, r) = survive_poll(&ctx, g, &taps);
		if r != 0 {
			break;
		}
		if stop_after > 0.0 && g.rt.run_time() > stop_after {
			g.rt.info(&sprintf("Stopping after %.0f s (--stop-after)", &[Arg::F(stop_after)]));
			break;
		}
		if auto.as_ref().is_some_and(|a| locked(a).tick(&mut g.rt)) {
			g.rt.info("calibration: the new room's calibration is ready: stopping to start again on it");
			break;
		}
	}
	g.rt.info("Shutting down cleanly");
	g.rt.calibrator = None;
	survive_close(&ctx, g, taps);
	// this session's recording is closed: its calibration
	if let Some(a) = &auto {
		locked(a).close();
	}
	0
}

fn locked(a: &Mutex<Auto>) -> MutexGuard<'_, Auto> {
	a.lock().unwrap_or_else(|e| e.into_inner())
}

/// survive_startup, with the context locked: the recording, the drivers in the C's load order (each one whose option is
/// set; a tap's thread starts and waits for the lock), "Loaded drivers", the config saved, the startup options, the
/// known stations broadcast, and the error for a runtime with nothing to track. An error raised while the drivers start
/// cuts it short after them, as in the C (the main loop's first poll then ends the run). The taps' threads, in load
/// order
fn survive_startup(ctx: &Arc<Mutex<Context>>, c: &mut Context) -> Vec<TapThread> {
	startup::install_recording(&mut c.rt);
	// the LH1 disambiguator is not part of sally, but its option is still written into the config
	c.rt.config.configs("disambiguator", SC_SETCONFIG, Some(b"StateBased"));

	let mut taps = Vec::new();
	let mut loaded = Vec::new();
	// a device source started: a driver asked for by name that is not passive (a tap, live or replaying)
	let mut no_source = true;
	let mut protocol_started = false;
	let threads = Arc::downgrade(ctx);
	for (name, option) in driver::ORDER {
		if !startup::enabled(&mut c.rt, option) {
			continue;
		}
		let manually_enabled = c.rt.config.is_set(option);
		let r = match name {
			"HIDTap" => start_live(ctx, c, &mut taps),
			"HIDTap_Playback" => start_playback(ctx, c, &mut taps),
			_ => match startup::start_driver(&mut c.rt, name, &mut protocol_started, Some(&threads)) {
				Some(r) => r,
				None => continue,
			},
		};
		// callDriver: a driver that reports an error is not named
		if r < 0 {
			c.rt.warn(&format!("Driver {name} reports status {r}"));
		} else {
			loaded.push(name);
		}
		if manually_enabled && r != DRIVER_PASSIVE {
			no_source = false;
		}
	}
	if c.rt.current_error != error::OK {
		return taps;
	}
	if no_source {
		c.rt.warn("No playback / tap driver loaded: no device source");
	}
	// "Loaded drivers: " and each name with ", ", the last two characters cut (with no driver, the prefix's ": ")
	let mut line = String::from("Loaded drivers: ");
	for name in loaded {
		line.push_str(name);
		line.push_str(", ");
	}
	line.truncate(line.len() - 2);
	c.rt.info(&line);
	// the config saved once more, so that there is a config file to edit
	c.rt.config_save();
	startup::startup_options(&mut c.rt);
	startup::broadcast_stations(&mut c.rt);
	if c.rt.objects.is_empty() && c.rt.drivers.is_empty() && taps.is_empty() {
		c.rt.error(error::NO_TRACKABLE_OBJECTS, "No trackable objects provided and no drivers are registered.");
	}
	taps
}

/// The live tap: on Windows the dongles read on their own threads (`Live`), elsewhere an error
fn start_live(ctx: &Arc<Mutex<Context>>, c: &mut Context, taps: &mut Vec<TapThread>) -> i32 {
	let record = c.rt.config.configs("hidtap-record", SC_GET, None).map(|r| text(&r));
	let options = tap::options(&mut c.rt);
	c.live = Some(Tap::new(options, Mode::Live));
	match Live::start(ctx, c, record.as_deref()) {
		Some(l) => {
			taps.push(TapThread::Live(l));
			DRIVER_NORMAL
		}
		None => DRIVER_ERROR,
	}
}

/// The playback: the recording opened (an empty name is no recording, one that does not open an error), and played on
/// its own thread (`Playback`)
fn start_playback(ctx: &Arc<Mutex<Context>>, c: &mut Context, taps: &mut Vec<TapThread>) -> i32 {
	let playback = c.rt.config.configs("hidtap-playback", SC_GET, None).filter(|p| !p.is_empty());
	let record = c.rt.config.configs("hidtap-record", SC_GET, None).map(|r| text(&r));
	let Some(playback) = playback else { return DRIVER_ERROR };
	let options = tap::options(&mut c.rt);
	let path = text(&playback);
	// (a packed recording is read as the text it holds)
	let file = match File::open(startup::path(&playback)).and_then(sally_hid::pack::text) {
		Ok(f) => f,
		Err(_) => {
			c.rt.error(error::INVALID_CONFIG, &format!("hidtap: cannot open {path}"));
			return DRIVER_ERROR;
		}
	};
	c.playback = Some(Tap::new(options, Mode::Playback));
	let p = Playback::start(ctx, c, file, &path, record.as_deref());
	taps.push(TapThread::Playback(p));
	DRIVER_NORMAL
}

/// survive_poll, with the context lock held and returning with it: an error raised ends the run, then each driver that
/// polls, in load order (the taps last), the first nonzero ending it ("Driver reported %d"); then the lock let go for
/// what is left of poll_min_time_ms since the poll began (OGGetAbsoluteTimeMS, rounded up to the millisecond, then
/// OGUSleep: poll_wait_us), and taken again
fn survive_poll<'a>(
	ctx: &'a Mutex<Context>, mut g: MutexGuard<'a, Context>, taps: &[TapThread],
) -> (MutexGuard<'a, Context>, i32) {
	let time_start = sally_os::absolute_time_ms();
	let r = g.rt.poll();
	if r != 0 {
		return (g, r);
	}
	// sally: what the protocol's `pair` told the runtime, to the live tap's dongles
	g.apply_pairs();
	for t in taps {
		let r = t.poll();
		if r != 0 {
			g.rt.warn(&format!("Driver reported {r}"));
			return (g, r);
		}
	}
	drop(g);
	if let Some(us) = poll_wait_us(time_start, sally_os::absolute_time_ms()) {
		sally_os::usleep(us);
	}
	(lock(ctx), 0)
}

/// survive_poll's wait, in uint64_t arithmetic, its microseconds an int: what is left of poll_min_time_ms after
/// time_start, if any. sally: never more than poll_min_time_ms. A clock that stepped back since time_start (the Linux
/// clock is the wall clock) made the C wait the step too, and on Windows a step back of the C's wrapped clock during a
/// poll longer than poll_min_time_ms made its int microseconds negative, which OGUSleep's Sleep takes as 49 days
fn poll_wait_us(time_start: u64, time_now: u64) -> Option<i32> {
	let end = time_start.wrapping_add(POLL_MIN_TIME_MS);
	if end > time_now && end - time_now <= POLL_MIN_TIME_MS {
		Some(end.wrapping_sub(time_now).wrapping_mul(1000) as i32)
	} else {
		None
	}
}

/// survive_close: the drivers closed in load order, the taps last (they are loaded last): a driver with a thread of its
/// own (the map solve's, the taps') has it told to stop and joined without the context lock, which the thread may be
/// waiting for, then closes under it. While a thread is joined the others go on, and their calls reach the drivers not
/// closed yet. Then the devices, the recording and the options
fn survive_close<'a>(ctx: &'a Mutex<Context>, mut g: MutexGuard<'a, Context>, taps: Vec<TapThread>) {
	loop {
		if let Some(thread) = g.rt.stop_first_driver() {
			drop(g);
			let _ = thread.join();
			g = lock(ctx);
		}
		if !g.rt.close_first_driver() {
			break;
		}
	}
	for t in taps {
		drop(g);
		t.stop();
		g = lock(ctx);
	}
	g.rt.close();
}

#[cfg(test)]
mod tests {
	use super::{poll_wait_us, POLL_MIN_TIME_MS};

	#[test]
	fn the_poll_waits_what_is_left_of_its_time() {
		let t = 1_000_000;
		assert_eq!(poll_wait_us(t, t), Some(POLL_MIN_TIME_MS as i32 * 1000));
		assert_eq!(poll_wait_us(t, t + 3), Some((POLL_MIN_TIME_MS as i32 - 3) * 1000));
		assert_eq!(poll_wait_us(t, t + POLL_MIN_TIME_MS), None);
		assert_eq!(poll_wait_us(t, t + 500), None);
		// a clock that stepped back during the poll: no wait (the C's was the step, or 49 days)
		assert_eq!(poll_wait_us(t, t - 1), None);
		assert_eq!(poll_wait_us(t, 0), None);
		assert_eq!(poll_wait_us(u64::MAX / 1000, 1), None);
	}
}
