//! A HID tap recording replayed (the playback half of the C runtime's driver_hidtap.c): each line goes to the device it
//! names in the tap (`tap::Tap`), copied to the `--hidtap-record` file as it was read. The run clock is the recording's:
//! the time of the line being replayed since the first (`PlaybackTime`, which the replay installs as the run clock).
//! At a playback factor above 0 the replay keeps to real time on os_generic's clock and sleep, as the C does.
//!
//! Two ways to run it. `Playback` is the C's: the playback thread (hidtap_thread_fn) reads the recording beside
//! survive-cli's main loop and takes the context lock (a `Context`) where hidtap_line takes it: once for a device line,
//! and for a report after its pace, to find its device; for an input report it lets the lock go once the device is
//! found and takes it again to hand the report on (the C's survive_data_cb takes it there). What a line does without
//! the lock is the C's too: it is counted (into the tap at the next lock), copied to the record file, parsed, and paced,
//! the pace's sleep outside the lock; a line the C ignores takes no lock. A malformed line's warning takes it, which the
//! C logs without. `Replay` does the same work on the caller's thread with a tap of its own (`sally replay`).

use std::io::{BufRead, BufReader, Read};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread::JoinHandle;

use crate::line::{self, Line};
use crate::tap::{DeviceId, Mode, Options, Record, Runtime, Tap};

/// HIDTAP_MAX_LINE: fgets reads a line in pieces of at most this many bytes less one
pub const MAX_LINE: usize = 4096;

/// d->time_now, the run time hidtap_run_time gives: the time of the line being replayed since the first, set by the
/// playback as it paces each line (outside the context lock, as the C's hidtap_pace sets it) and read by the run clock
/// from any thread
#[derive(Debug, Default)]
pub struct PlaybackTime(AtomicU64);

impl PlaybackTime {
	pub fn new(t: f64) -> PlaybackTime {
		PlaybackTime(AtomicU64::new(t.to_bits()))
	}

	pub fn get(&self) -> f64 {
		f64::from_bits(self.0.load(Ordering::Relaxed))
	}

	pub fn set(&self, t: f64) {
		self.0.store(t.to_bits(), Ordering::Relaxed);
	}
}

/// What the playback thread locks, as the C's context lock: the playback's tap state and the runtime it feeds (the
/// live tap's is `live::Context`: a runtime with both taps keeps two states, as the C's two drivers do)
pub trait Context: Send + 'static {
	fn tap(&mut self) -> (&mut Tap, &mut dyn Runtime);
}

/// The playback's own state, which the C keeps in SurviveDriverHIDTap and touches without the context lock:
/// hidtap_pace's (the first report's time, the real time it was read at, the time reached), the record file
/// (d->record), the threaded driver's keep_running, and the lines read since the tap's count was last brought up
struct Player {
	playback_factor: f64,
	run_time: f64,
	first_time_us: u64,
	first_real_time: f64,
	time: Arc<PlaybackTime>,
	record: Option<Record>,
	keep_running: Arc<AtomicBool>,
	uncounted: u64,
}

/// An input report whose device was found under the lock, to hand on (hidtap_line's 'I' case, after its release)
struct Input {
	dev: DeviceId,
	pid: u16,
	mi: i32,
	col: i32,
	time_us: u64,
	data: Vec<u8>,
}

impl Player {
	/// DriverRegHIDTap_ in playback mode, once the file is open: "hidtap: replaying %s (factor %.2f)", then the
	/// recording of the stream when `record` names a file; the playback's time not installed yet
	fn new(rt: &mut dyn Runtime, options: &Options, path: &str, record: Option<&str>) -> Player {
		rt.info(&format!("hidtap: replaying {} (factor {:.2})", path, options.playback_factor));
		Player {
			playback_factor: options.playback_factor,
			run_time: options.run_time,
			first_time_us: 0,
			first_real_time: 0.0,
			time: Arc::new(PlaybackTime::default()),
			record: record.filter(|r| !r.is_empty()).and_then(|r| Record::open(rt, r)),
			keep_running: Arc::new(AtomicBool::new(true)),
			uncounted: 0,
		}
	}

	/// The playback goes on
	fn running(&self) -> bool {
		self.keep_running.load(Ordering::SeqCst)
	}

	/// hidtap_line before the lock: the line counted (d->lines++, into the tap at the next lock), copied to the record
	/// file, parsed, and a report paced
	fn read(&mut self, line: &[u8]) -> Line {
		self.uncounted += 1;
		if let Some(r) = self.record.as_mut() {
			r.line_as_read(line);
		}
		let parsed = line::parse(line);
		if let Line::Report { time_us, .. } = parsed {
			self.pace(time_us);
		}
		parsed
	}

	/// The run clock from the line's time; at playback-factor > 0 the replay keeps to real time (OGGetAbsoluteTime,
	/// OGUSleep); past run-time the replay stops (after this line)
	fn pace(&mut self, time_us: u64) {
		if self.first_time_us == 0 {
			self.first_time_us = time_us;
			self.first_real_time = sally_os::absolute_time();
		}
		let time_now = time_us.wrapping_sub(self.first_time_us) as f64 * 1e-6;
		self.time.set(time_now);
		if self.playback_factor > 0.0 {
			let target = self.first_real_time + time_now * self.playback_factor;
			let mut now = sally_os::absolute_time();
			while now < target {
				sally_os::usleep(c_int(1000.0 + (target - now) * 1e6));
				now = sally_os::absolute_time();
			}
		}
		if self.run_time > 0.0 && time_now > self.run_time {
			// hidtap_stop
			self.keep_running.store(false, Ordering::SeqCst);
		}
	}

	/// The lines read since the tap's count was brought up, into it (under the lock)
	fn count(&mut self, tap: &mut Tap) {
		tap.lines += std::mem::take(&mut self.uncounted);
	}

	/// The end of hidtap_thread_fn: "hidtap: playback finished after %llu lines", then hidtap_stop
	fn finished(&mut self, tap: &mut Tap, rt: &mut dyn Runtime) {
		self.count(tap);
		rt.info(&format!("hidtap: playback finished after {} lines", tap.lines));
		self.keep_running.store(false, Ordering::SeqCst);
	}
}

impl Tap {
	/// hidtap_line with the context locked, after what it does without: a device line's device, a malformed line's
	/// warning, a report's device (the report counted) and what a feature or output report asks of it. An input report
	/// is given back, to hand on after the lock is let go (`replay_input`)
	fn replay_line(&mut self, rt: &mut dyn Runtime, parsed: Line, line: &[u8]) -> Option<Input> {
		match parsed {
			Line::Device { serial, pid, .. } => {
				self.device(rt, &serial, pid);
			}
			// sally: the device the dongle carries from here on, as the live run was told
			Line::Assign { time_us, serial, device } => {
				self.assign(rt, &serial, &device, time_us);
			}
			Line::Ignored => {}
			Line::BadLine => self.bad_line(rt, "hidtap: bad line", line),
			Line::BadHex => self.bad_line(rt, "hidtap: bad hex", line),
			Line::Report { time_us, serial, pid, mi, col, kind, data } => {
				let dev = self.device(rt, &serial, pid)?;
				self.reports += 1;
				match kind {
					b'I' => return Some(Input { dev, pid, mi, col, time_us, data }),
					b'F' => {
						if !self.config_report(rt, dev, &data) {
							self.feature_read(rt, dev, &data);
						}
					}
					b'W' | b'O' if !data.is_empty() => {
						let w_value = if kind == b'W' { 0x300 } else { 0x200 } | data[0] as u16;
						self.setup_write(rt, dev, 0x21, 0x09, w_value, &data);
					}
					b'W' | b'O' => {}
					_ => {
						if self.bad_lines < 5 {
							rt.warn(&format!("hidtap: unknown kind '{}'", kind as char));
						}
						self.bad_lines += 1;
					}
				}
			}
		}
		None
	}

	/// The first five malformed lines are logged, as "%.80s" of the line (its newline too, when it is short)
	fn bad_line(&mut self, rt: &mut dyn Runtime, what: &str, line: &[u8]) {
		if self.bad_lines < 5 {
			let shown: String = line.iter().take(80).map(|&c| c as char).collect();
			rt.warn(&format!("{what}: {shown}"));
		}
		self.bad_lines += 1;
	}

	/// hidtap_line's input report, handed on: its light packets sniffed for the mode, then the report to the parser
	fn replay_input(&mut self, rt: &mut dyn Runtime, i: &Input) {
		self.input_report(rt, i.dev, i.pid, i.mi, i.col, i.time_us, &i.data);
	}
}

/// The replay on the caller's thread (SurviveDriverHIDTap in playback mode, with its tap): `sally replay`'s, whose one
/// thread does the playback thread's work and the main loop's in the C's order
pub struct Replay {
	tap: Tap,
	player: Player,
}

impl Replay {
	/// DriverRegHIDTap_Playback, once the file is open: "hidtap: replaying %s (factor %.2f)", the recording of the
	/// stream when `record` names a file, and the playback's time installed as the run clock (at 0 until the first
	/// line)
	pub fn new(rt: &mut dyn Runtime, options: Options, path: &str, record: Option<&str>) -> Self {
		let player = Player::new(rt, &options, path, record);
		rt.install_run_time_fn(Some(player.time.clone()));
		Replay { tap: Tap::new(options, Mode::Playback), player }
	}

	/// The tap's state
	pub fn tap(&self) -> &Tap {
		&self.tap
	}

	/// The replay is over: the recording ended, or the run time was reached
	pub fn stopped(&self) -> bool {
		!self.player.running()
	}

	/// Every line of the recording, in fgets's pieces, until the end or the run time; then "hidtap: playback finished
	/// after %llu lines"
	pub fn run(&mut self, rt: &mut dyn Runtime, input: &mut dyn BufRead) -> std::io::Result<()> {
		let mut piece = Vec::with_capacity(MAX_LINE);
		while self.player.running() && read_piece(input, &mut piece)? {
			self.line(rt, &piece);
		}
		self.player.finished(&mut self.tap, rt);
		Ok(())
	}

	/// The summary, the recording closed, the run clock frozen
	pub fn close(self, rt: &mut dyn Runtime) {
		self.tap.close(rt, self.player.record);
	}

	/// One line of the recording (as fgets gives it, its newline included)
	pub fn line(&mut self, rt: &mut dyn Runtime, line: &[u8]) {
		let parsed = self.player.read(line);
		self.player.count(&mut self.tap);
		if let Some(i) = self.tap.replay_line(rt, parsed, line) {
			self.tap.replay_input(rt, &i);
		}
	}
}

/// The playback on its own thread (the C's threaded driver "hidtap_thread"), as the main thread holds it
pub struct Playback<C: Context> {
	ctx: Arc<Mutex<C>>,
	/// the threaded driver's keep_running (the thread clears it at its end, survive_close before it joins)
	keep_running: Arc<AtomicBool>,
	/// the playback thread, which gives the record file back when it ends
	thread: Option<JoinHandle<Option<Record>>>,
}

impl<C: Context> Playback<C> {
	/// DriverRegHIDTap_Playback once the file is open, with its tap state in the context: "hidtap: replaying %s
	/// (factor %.2f)", the recording of the stream when `record` names a file, the playback thread started
	/// (survive_add_threaded_driver), then its time installed as the run clock. Called with the context locked, as
	/// survive_startup calls it (`locked` is the locked context): the thread waits for the lock at its first device
	pub fn start(
		ctx: &Arc<Mutex<C>>, locked: &mut C, file: Box<dyn Read + Send>, path: &str, record: Option<&str>,
	) -> Playback<C> {
		let (tap, rt) = locked.tap();
		let player = Player::new(rt, tap.options(), path, record);
		let (time, keep_running) = (player.time.clone(), player.keep_running.clone());
		let thread = {
			let ctx = ctx.clone();
			sally_os::create_thread("hidtap_thread", move || play(&ctx, player, BufReader::new(file)))
		};
		rt.install_run_time_fn(Some(time));
		Playback { ctx: ctx.clone(), keep_running, thread }
	}

	/// What threaded_driver_poll asks: whether the playback goes on (its poll is -1 once it has stopped)
	pub fn running(&self) -> bool {
		self.keep_running.load(Ordering::SeqCst)
	}

	/// The stop: the keep-running flag cleared and the playback thread joined (it ends after the line it is on, whose
	/// pace may still sleep), then with the context locked the summary logged, the record file closed and the run clock
	/// frozen. Called without the context locked: the thread takes the lock for its last line
	pub fn stop(mut self) {
		self.keep_running.store(false, Ordering::SeqCst);
		let record = self.thread.take().and_then(|t| t.join().ok()).flatten();
		let mut g = lock(&self.ctx);
		let (tap, rt) = g.tap();
		tap.close(rt, record);
	}
}

/// The context lock (survive_get_ctx_lock)
fn lock<C>(ctx: &Mutex<C>) -> MutexGuard<'_, C> {
	ctx.lock().unwrap_or_else(|e| e.into_inner())
}

/// Every line of the recording, in fgets's pieces, until its end (or a read error: fgets's NULL), the run time, or the
/// driver stopped; then "hidtap: playback finished after %llu lines" (with the lock, which the C logs without) and the
/// driver stopped. The record file is given back for the close
fn play<C: Context>(ctx: &Mutex<C>, mut player: Player, mut input: impl BufRead) -> Option<Record> {
	let mut piece = Vec::with_capacity(MAX_LINE);
	while player.running() && read_piece(&mut input, &mut piece).unwrap_or(false) {
		played_line(ctx, &mut player, &piece);
	}
	let mut g = lock(ctx);
	let (tap, rt) = g.tap();
	player.finished(tap, rt);
	player.record
}

/// hidtap_line on the playback thread, the lock taken where the C takes it (see the module)
fn played_line<C: Context>(ctx: &Mutex<C>, player: &mut Player, line: &[u8]) {
	let parsed = player.read(line);
	if matches!(parsed, Line::Ignored) {
		return;
	}
	let input = {
		let mut g = lock(ctx);
		let (tap, rt) = g.tap();
		player.count(tap);
		tap.replay_line(rt, parsed, line)
	};
	// survive_release_ctx_lock, then hidtap_sniff_mode and survive_vive_report_input, whose survive_data_cb takes it
	if let Some(i) = input {
		let mut g = lock(ctx);
		let (tap, rt) = g.tap();
		tap.replay_input(rt, &i);
	}
}

/// (int)x as the C's conversion of a double compiles on x86-64 (cvttsd2si): toward zero, and INT_MIN for a value past
/// the int's range or NaN, which the C leaves undefined. A wait of 36 minutes reaches it (a gap that long in a
/// recording replayed at factor 1): the C's OGUSleep then fails at once on Linux, so the pace spins, and sleeps 49 days
/// on Windows; kept
fn c_int(x: f64) -> i32 {
	if x > -2147483649.0 && x < 2147483648.0 {
		x as i32
	} else {
		i32::MIN
	}
}

/// fgets(buf, MAX_LINE, f): the next piece of a line, up to its newline and at most MAX_LINE - 1 bytes; false at the end
fn read_piece(input: &mut dyn BufRead, piece: &mut Vec<u8>) -> std::io::Result<bool> {
	piece.clear();
	while piece.len() < MAX_LINE - 1 {
		let buf = input.fill_buf()?;
		if buf.is_empty() {
			break;
		}
		let room = MAX_LINE - 1 - piece.len();
		let take = buf.iter().take(room).position(|&c| c == b'\n').map(|p| p + 1).unwrap_or(buf.len().min(room));
		piece.extend_from_slice(&buf[..take]);
		input.consume(take);
		if piece.last() == Some(&b'\n') {
			break;
		}
	}
	Ok(!piece.is_empty())
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::report::{Globals, Parser, BUFFER_LEN};
	use crate::tap::ObjectId;

	#[test]
	fn a_double_becomes_an_int_as_the_c_converts_it() {
		assert_eq!(c_int(1999.9), 1999);
		assert_eq!(c_int(-1999.9), -1999);
		assert_eq!(c_int(2147483647.9), i32::MAX);
		assert_eq!(c_int(-2147483648.9), i32::MIN);
		// past the range, and NaN: cvttsd2si's "integer indefinite"
		assert_eq!(c_int(2147483648.0), i32::MIN);
		assert_eq!(c_int(-2147483649.0), i32::MIN);
		assert_eq!(c_int(f64::NAN), i32::MIN);
		assert_eq!(c_int(f64::INFINITY), i32::MIN);
	}

	#[test]
	fn the_playback_time_is_shared() {
		let t = Arc::new(PlaybackTime::default());
		let seen = t.clone();
		assert_eq!(seen.get(), 0.0);
		t.set(12.25);
		assert_eq!(seen.get(), 12.25);
	}

	#[test]
	fn lines_come_in_fgets_pieces() {
		let long = vec![b'7'; 5000];
		let mut text = b"R 1 a 2102 -1 -1 I 23\n".to_vec();
		text.extend_from_slice(&long);
		text.push(b'\n');
		let mut r = std::io::Cursor::new(text);
		let mut p = Vec::new();
		assert!(read_piece(&mut r, &mut p).unwrap());
		assert_eq!(p, b"R 1 a 2102 -1 -1 I 23\n");
		assert!(read_piece(&mut r, &mut p).unwrap());
		assert_eq!(p.len(), MAX_LINE - 1);
		assert!(read_piece(&mut r, &mut p).unwrap());
		assert_eq!(p.len(), 5000 - (MAX_LINE - 1) + 1);
		assert!(!read_piece(&mut r, &mut p).unwrap());
	}

	/// A runtime that notes what the tap asks of it, its run time the playback's
	#[derive(Default)]
	struct Notes {
		said: Vec<String>,
		objects: usize,
		time: Option<Arc<PlaybackTime>>,
	}

	impl Runtime for Notes {
		fn create_object(&mut self, name: &str, serial: &str) -> Option<ObjectId> {
			self.said.push(format!("object {name} {serial}"));
			self.objects += 1;
			Some(self.objects - 1)
		}
		fn load_config_file(&mut self, _obj: ObjectId, filename: &str) -> bool {
			self.said.push(format!("load {filename}"));
			false
		}
		fn config(&mut self, _obj: ObjectId, json: Option<Vec<u8>>) -> i32 {
			self.said.push(format!("config {}", json.map_or(0, |j| j.len())));
			0
		}
		fn report(
			&mut self, obj: ObjectId, _parser: &mut Parser, _globals: &mut Globals, time_us: u64, iface: i32,
			_buffer: &[u8; BUFFER_LEN], len: usize,
		) {
			let t = self.run_time();
			self.said.push(format!("report {obj} {time_us} {iface} {len} at {t}"));
		}
		fn setup_write(
			&mut self, obj: ObjectId, _parser: &mut Parser, _bm_request_type: u8, _b_request: u8, w_value: u16,
			_data: &[u8],
		) {
			self.said.push(format!("setup {obj} {w_value:x}"));
		}
		fn feature_read(&mut self, obj: ObjectId, _parser: &Parser, data: &[u8]) {
			self.said.push(format!("feature {obj} {data:?}"));
		}
		fn install_run_time_fn(&mut self, time: Option<Arc<PlaybackTime>>) {
			self.said.push(format!("clock {}", if time.is_some() { "installed" } else { "frozen" }));
			if time.is_some() {
				self.time = time;
			}
		}
		fn run_time(&self) -> f64 {
			self.time.as_ref().map_or(-1.0, |t| t.get())
		}
		fn info(&mut self, text: &str) {
			self.said.push(format!("Info: {text}"));
		}
		fn warn(&mut self, text: &str) {
			self.said.push(format!("Warning: {text}"));
		}
		fn error(&mut self, code: i32, text: &str) {
			self.said.push(format!("Error {code}: {text}"));
		}
	}

	struct Ctx {
		tap: Tap,
		rt: Notes,
	}

	impl Context for Ctx {
		fn tap(&mut self) -> (&mut Tap, &mut dyn Runtime) {
			(&mut self.tap, &mut self.rt)
		}
	}

	/// A recording with a comment, a device line, a configured device's config stream and input reports, malformed
	/// lines, a feature report that is not config, an output report and a kind the tap does not know, then two lines
	/// past run-time
	fn recording() -> Vec<u8> {
		let json = b"{}";
		let z = miniz_oxide::deflate::compress_to_vec_zlib(json, 6);
		let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
		let mut chunk = vec![0x11, z.len() as u8];
		chunk.extend_from_slice(&z);
		let mut light = vec![0x77u8; 59];
		light[0] = 0x23;
		let lines = [
			"# a tap recording".to_string(),
			"D 900 nosn-0 2102".into(),
			"R 1000 nosn-0 2102 -1 -1 F 10".into(),
			format!("R 1100 nosn-0 2102 -1 -1 F {}", hex(&chunk)),
			"R 1200 nosn-0 2102 -1 -1 F 1100".into(),
			format!("R 2000 nosn-0 2102 -1 -1 I {}", hex(&light)),
			"R 2500 nosn-0".into(),
			"R 2600 nosn-0 2102 -1 -1 I 2x".into(),
			"R 3000 nosn-0 2102 -1 -1 F 05aa".into(),
			"R 3500 nosn-0 2102 -1 -1 O 0102".into(),
			"R 4000 nosn-0 2102 -1 -1 Z 00".into(),
			"R 4500 nosn-1 2613 -1 -1 I 00".into(),
			format!("R 5000 nosn-0 2102 -1 -1 I {}", hex(&light)),
			format!("R 9000 nosn-0 2102 -1 -1 I {}", hex(&light)),
			format!("R 9500 nosn-0 2102 -1 -1 I {}", hex(&light)),
		];
		lines.iter().flat_map(|l| l.bytes().chain(Some(b'\n'))).collect()
	}

	/// The playback on its own thread does what the replay on the caller's thread does, line for line, and stops at
	/// run-time with its line count; its poll's question turns once it has stopped. A packed recording plays as the
	/// text it holds
	#[test]
	fn the_playback_thread_replays_as_the_replay_does() {
		use std::fs::File;
		let path = std::env::temp_dir().join(format!("sally-hid-playback-{}.hidtap", std::process::id()));
		std::fs::write(&path, recording()).unwrap();
		let packed = path.with_extension("hidtapz");
		let mut w = crate::pack::PackWriter::new(File::create(&packed).unwrap()).unwrap();
		w.write(&recording()).unwrap();
		w.finish().unwrap();
		assert!(std::fs::read(&packed).unwrap().starts_with(crate::pack::MAGIC));
		let options = Options { playback_factor: 0.0, run_time: 0.007, ..Options::default() };

		let mut rt = Notes::default();
		let mut replay = Replay::new(&mut rt, options.clone(), "the recording", None);
		replay.run(&mut rt, &mut BufReader::new(File::open(&path).unwrap())).unwrap();
		assert!(replay.stopped());
		replay.close(&mut rt);
		assert!(rt.said.contains(&"Info: hidtap: playback finished after 14 lines".to_string()), "{:?}", rt.said);
		// (the unknown device's report is not counted: it has no device)
		assert!(rt.said.contains(&"Info: hidtap: 14 lines, 9 reports, 3 bad lines".to_string()), "{:?}", rt.said);
		assert_eq!(rt.said.last().map(String::as_str), Some("clock frozen"));

		for file in [&path, &packed] {
			let ctx =
				Arc::new(Mutex::new(Ctx { tap: Tap::new(options.clone(), Mode::Playback), rt: Notes::default() }));
			let playback = {
				let mut g = lock(&ctx);
				Playback::start(&ctx, &mut g, crate::pack::open(file).unwrap(), "the recording", None)
			};
			while playback.running() {
				std::thread::sleep(std::time::Duration::from_millis(1));
			}
			playback.stop();
			assert_eq!(lock(&ctx).rt.said, rt.said, "{}", file.display());
			let _ = std::fs::remove_file(file);
		}
	}
}
