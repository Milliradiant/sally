//! `sally replay`: survive-cli's run of a tap recording (survive_init, survive_startup, the HID tap's playback thread
//! with survive-cli's main loop beside it, and survive_close), single-threaded: the C's playback thread and its main
//! thread take turns under one lock, so one thread doing both in the C's order gives the same output (`play` says
//! where the main loop's polls come). Its polls come at the same lines on every run, which the protocol's checks rely
//! on; `sally run` is the C's threads.

use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::sync::atomic::{AtomicU64, Ordering};

use sally_hid::pack;
use sally_hid::replay::{Replay, MAX_LINE};
use sally_math::printf::{sprintf, Arg};
use sally_runtime::config::{SC_GET, SC_SETCONFIG};
use sally_runtime::runtime::error;
use sally_runtime::{driver, tap, Runtime};

use crate::startup::{self, text};

/// survive-cli's main on a tap recording: survive_init, survive_startup (the recording, the HID tap's playback
/// driver, the known stations broadcast), the playback to its end, survive_close. `argv` is the command line as
/// main's argv has it (the program first)
pub fn main(argv: &[Vec<u8>]) -> i32 {
	run(argv, None, |_| {}, |_| 0)
}

/// How far the replay has read its recording, and how much it reads (bytes): what `sally calibrate` reports while it
/// replays
static READ: AtomicU64 = AtomicU64::new(0);
static LENGTH: AtomicU64 = AtomicU64::new(0);

/// The share of the recording read so far (0 to 1)
pub fn progress() -> f64 {
	let n = LENGTH.load(Ordering::Relaxed);
	if n == 0 {
		0.0
	} else {
		(READ.load(Ordering::Relaxed) as f64 / n as f64).min(1.0)
	}
}

/// A recording's first `limit` bytes up to their last whole line (a recording being written ends in a line cut
/// short), or all of it. A packed recording's first `limit` bytes as they are: it is read to its last whole block
fn length_to_read(file: &mut File, limit: Option<u64>, packed: bool) -> u64 {
	use std::io::{Seek, SeekFrom};
	let all = file.metadata().map(|m| m.len()).unwrap_or(u64::MAX);
	let Some(limit) = limit.filter(|&l| l < all) else { return all };
	if packed {
		return limit;
	}
	let back = limit.min(1 << 16);
	let mut tail = vec![0u8; back as usize];
	let whole = file.seek(SeekFrom::Start(limit - back)).is_ok() && file.read_exact(&mut tail).is_ok();
	let _ = file.seek(SeekFrom::Start(0));
	if !whole {
		return 0;
	}
	match tail.iter().rposition(|&c| c == b'\n') {
		Some(i) => limit - back + i as u64 + 1,
		None => 0,
	}
}

/// The recording as the replay reads it, counted into READ
struct Counted<R>(R);

impl<R: Read> Read for Counted<R> {
	fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
		let n = self.0.read(buf)?;
		READ.fetch_add(n as u64, Ordering::Relaxed);
		Ok(n)
	}
}

/// `main` with two hooks for what is built on a replay (`sally calibrate`): `setup` once the runtime is made, before
/// the recording and the drivers; `finish` once the playback has ended, before the runtime closes (its code is the
/// run's when it is not 0). `main`'s hooks do nothing. `limit`: the recording's first bytes only, to their last whole
/// line (one still being written)
pub fn run(
	argv: &[Vec<u8>], limit: Option<u64>, setup: impl FnOnce(&mut Runtime), finish: impl FnOnce(&mut Runtime) -> i32,
) -> i32 {
	let mut rt = match startup::init(argv) {
		Ok(rt) => rt,
		Err(code) => return code,
	};
	setup(&mut rt);

	// survive_startup
	startup::install_recording(&mut rt);
	// the LH1 disambiguator is not part of sally (LH1 light is dropped once the generation is known), but its option is
	// still written into the config
	rt.config.configs("disambiguator", SC_SETCONFIG, Some(b"StateBased"));

	// survive_startup's driver loop: each driver in the C's load order whose option is set (the drivers not ported yet
	// are not loaded, nor the live tap), the tap replay last
	let mut loaded = Vec::new();
	let mut protocol_started = false;
	for (name, option) in driver::ORDER {
		if !startup::enabled(&mut rt, option) {
			continue;
		}
		match startup::start_driver(&mut rt, name, &mut protocol_started, None) {
			// callDriver: a driver that reports an error is not named
			Some(r) if r < 0 => rt.warn(&format!("Driver {name} reports status {r}")),
			Some(_) => loaded.push(name),
			None => {}
		}
	}
	let Some(playback) = rt.config.configs("hidtap-playback", SC_GET, None) else {
		rt.warn("No playback / tap driver loaded: no device source");
		eprintln!("sally replay: --hidtap-playback <tap recording> is required");
		return 2;
	};
	let playback = text(&playback);
	let record = rt.config.configs("hidtap-record", SC_GET, None).map(|r| text(&r));
	let options = tap::options(&mut rt);
	let playback_factor = options.playback_factor;
	// the recording's text: the file itself, or what a packed one holds (how far it has been read is the file's
	// bytes either way)
	let opened = File::open(&playback).and_then(|mut file| {
		let packed = pack::packed(&mut file)?;
		let length = length_to_read(&mut file, limit, packed);
		let file = Counted(file.take(length));
		let text: Box<dyn Read> = match packed {
			true => Box::new(pack::Unpacker::new(BufReader::with_capacity(1 << 16, file))?),
			false => Box::new(file),
		};
		Ok((text, length))
	});
	let (text, length) = match opened {
		Ok(o) => o,
		Err(_) => {
			rt.error(error::INVALID_CONFIG, &format!("hidtap: cannot open {playback}"));
			return 1;
		}
	};
	let mut replay = Replay::new(&mut rt, options, &playback, record.as_deref());
	loaded.push("HIDTap_Playback");
	rt.info(&format!("Loaded drivers: {}", loaded.join(", ")));
	// the config saved once more, so that there is a config file to edit
	rt.config_save();
	startup::startup_options(&mut rt);
	startup::broadcast_stations(&mut rt);

	let stop_after = rt.config.configf("stop-after", SC_GET, -1.0);
	READ.store(0, Ordering::Relaxed);
	LENGTH.store(length, Ordering::Relaxed);
	let mut input = BufReader::with_capacity(1 << 20, text);
	let end = play(&mut rt, &mut replay, &mut input, stop_after, playback_factor);
	match &end {
		End::Finished | End::ReadError(_) => {
			if let End::ReadError(e) = &end {
				rt.warn(&format!("hidtap: reading {playback}: {e}"));
			} else {
				// the playback thread's end: its last line, with nothing left to read
				let _ = replay.run(&mut rt, &mut std::io::empty());
			}
			// the main loop's next survive_poll: the drivers before the tap's polled once more, then the tap's driver
			// found stopped, and survive-cli's loop ends
			if rt.poll() == 0 {
				rt.warn("Driver reported -1");
			}
		}
		// survive_poll returned it ("Driver reported", or the error raised): survive-cli's loop ends
		End::Poll => {}
		End::StopAfter => rt.info(&sprintf("Stopping after %.0f s (--stop-after)", &[Arg::F(stop_after)])),
	}
	let code = finish(&mut rt);
	rt.info("Shutting down cleanly");

	// survive_close: the drivers closed in load order, the tap replay last (the playback thread stopped if it still
	// ran, its summary, the clock frozen), then the devices
	rt.close_drivers();
	if matches!(end, End::Poll | End::StopAfter) {
		let _ = replay.run(&mut rt, &mut std::io::empty());
	}
	replay.close(&mut rt);
	rt.close();
	code
}

/// How the playback ended
enum End {
	/// the recording's end, or its run time (run-time)
	Finished,
	/// the recording could not be read on
	ReadError(std::io::Error),
	/// survive_poll returned nonzero: a driver's poll (the protocol's quit), or an error raised
	Poll,
	/// --stop-after's run time passed
	StopAfter,
}

/// survive_poll's least time from one poll to the next (ctx->poll_min_time_ms), in seconds of wall time
const POLL_MIN_TIME: f64 = 0.010;

/// The playback with the main loop beside it: the poll (`Runtime::poll`: the error check, then the polls of the drivers
/// that have one) and the --stop-after check. `sally run` runs them every 10 ms of wall time, wherever the playback has
/// got to; here they run after the first line 10 ms of the playback's pace past the last poll (10 ms / playback-factor
/// of run time; 10 ms unpaced, as at factor 1). That is the same cadence on the recording's clock instead of the
/// wall's: a function of the recording alone, so a replay polls at the same lines on every run, and the protocol's
/// summaries (each once a diag period of run time has passed at a poll) come at the live pace. A poll costs a recvfrom
/// that finds nothing; a gap in a paced recording holds the commands back to its end (the playback sleeps through it)
fn play(rt: &mut Runtime, replay: &mut Replay, input: &mut impl BufRead, stop_after: f64, factor: f64) -> End {
	let interval = if factor > 0.0 { POLL_MIN_TIME / factor } else { POLL_MIN_TIME };
	let mut last_poll = rt.run_time();
	let mut piece = Vec::with_capacity(MAX_LINE);
	while !replay.stopped() {
		// fgets(buf, HIDTAP_MAX_LINE, f): the next piece of a line, up to its newline and at most MAX_LINE - 1 bytes
		piece.clear();
		match input.by_ref().take(MAX_LINE as u64 - 1).read_until(b'\n', &mut piece) {
			Ok(0) => break,
			Ok(_) => replay.line(rt, &piece),
			Err(e) => return End::ReadError(e),
		}
		// a run time that went back (a recording's clock that did) polls at once and counts on from there
		let now = rt.run_time();
		if now >= last_poll && now - last_poll < interval {
			continue;
		}
		last_poll = now;
		if rt.poll() != 0 {
			return End::Poll;
		}
		if stop_after > 0.0 && now > stop_after {
			return End::StopAfter;
		}
	}
	End::Finished
}
