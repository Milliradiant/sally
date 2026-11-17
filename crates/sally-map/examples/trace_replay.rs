//! The map solve checked against the C on a trace of it: `trace_replay <trace> [--init-map <file>]`.
//!
//! The trace comes from an instrumented copy of the C runtime's driver_map_solve.c (kept outside the repo): every call
//! into the map solve (`C start|sweep|imupose|close ...` up to `E`), the options it starts with (`S`), every value it
//! reads from the runtime (`R`) and everything it does there (`O`), in order; doubles as their 64 bits in hex, strings
//! as one token, '=' and the bytes with '\\' and the bytes below 33 or above 126 as \xHH. This host answers the port's
//! reads from the trace and stops at the first read or output that is not the C's: the same kind, the same arguments,
//! the same bits and the same text, in the same order. The config store's options (and the log level, and
//! SALLY_MAPSOLVE_DUMP) are the C's from the `S` lines; asking for one the C did not have stops it too. The map file
//! and the debug dump the C wrote (`O mapsave`, `O dump`, with the file's bytes) are compared with the files the port
//! wrote at the same moment; --init-map puts a map file's first content in place for a trace that reads map-solve-file.

use std::cell::RefCell;
use std::collections::HashMap;
use std::io::{BufRead, BufReader};
use std::process::exit;

use sally_lighthouse::BaseStationCal;
use sally_map::{DeviceId, Host, MapSolve, Velocity};
use sally_math::Pose;

/// The trace, one event at a time
struct Trace {
	lines: std::io::Lines<BufReader<std::fs::File>>,
	line_no: usize,
	/// an event read ahead
	peeked: Option<String>,
	reads: u64,
	outputs: u64,
	files: u64,
}

impl Trace {
	fn next(&mut self) -> Option<String> {
		if let Some(l) = self.peeked.take() {
			return Some(l);
		}
		let l = self.lines.next()?.expect("reading the trace");
		self.line_no += 1;
		Some(l)
	}
	fn peek(&mut self) -> Option<&String> {
		if self.peeked.is_none() {
			self.peeked = self.next();
		}
		self.peeked.as_ref()
	}
}

fn fail(line_no: usize, expected: &str, got: &str) -> ! {
	eprintln!("MISMATCH at trace line {line_no}\n  C:    {expected}\n  Rust: {got}");
	exit(1);
}

/// A string as the trace writes it
fn esc(s: &[u8]) -> String {
	let mut o = String::with_capacity(s.len() + 1);
	o.push('=');
	for &c in s {
		if c == b'\\' {
			o.push_str("\\\\");
		} else if !(33..=126).contains(&c) {
			o.push_str(&format!("\\x{c:02x}"));
		} else {
			o.push(c as char);
		}
	}
	o
}

/// A string token back to its bytes
fn unesc(t: &str) -> Vec<u8> {
	let b = t.strip_prefix('=').expect("a string token").as_bytes();
	let mut o = Vec::with_capacity(b.len());
	let mut i = 0;
	while i < b.len() {
		if b[i] == b'\\' && b.get(i + 1) == Some(&b'\\') {
			o.push(b'\\');
			i += 2;
		} else if b[i] == b'\\' && b.get(i + 1) == Some(&b'x') {
			o.push(u8::from_str_radix(std::str::from_utf8(&b[i + 2..i + 4]).unwrap(), 16).unwrap());
			i += 4;
		} else {
			o.push(b[i]);
			i += 1;
		}
	}
	o
}

fn hex(v: f64) -> String {
	format!("{:016x}", v.to_bits())
}

fn dbl(t: &str) -> f64 {
	f64::from_bits(u64::from_str_radix(t, 16).expect("a double's bits"))
}

fn pose_hex(p: &Pose) -> String {
	p.pos.iter().chain(&p.rot).map(|&v| hex(v)).collect::<Vec<_>>().join(" ")
}

fn fcal_hex(f: &[BaseStationCal; 2]) -> String {
	let mut v = Vec::new();
	for c in f {
		for x in [c.phase, c.tilt, c.curve, c.gibpha, c.gibmag, c.ogeephase, c.ogeemag] {
			v.push(hex(x));
		}
	}
	v.join(" ")
}

/// The runtime the trace recorded
struct TraceHost {
	t: RefCell<Trace>,
	/// the options the C started with (the S lines: an integer, a double's bits, or a string token)
	options: HashMap<String, String>,
}

impl TraceHost {
	/// The next event that is not a file the C wrote; a file event is checked against the port's file on its way
	fn event(&self) -> (usize, String) {
		let mut t = self.t.borrow_mut();
		loop {
			let Some(l) = t.next() else {
				let n = t.line_no;
				drop(t);
				fail(n, "(the trace ends)", "a read or an output");
			};
			let n = t.line_no;
			if l.starts_with("O mapsave ") || l.starts_with("O dump ") {
				t.files += 1;
				check_file(n, &l);
				continue;
			}
			return (n, l);
		}
	}

	/// A read: the next event must be `R <what>`, and its value is the rest
	fn read(&self, what: &str) -> String {
		let (n, l) = self.event();
		let want = format!("R {what} ");
		if !l.starts_with(&want) {
			fail(n, &l, &format!("R {what} ..."));
		}
		self.t.borrow_mut().reads += 1;
		l[want.len()..].to_string()
	}

	/// An output: the next event must be exactly it
	fn output(&self, got: String) {
		let (n, l) = self.event();
		if l != got {
			fail(n, &l, &got);
		}
		self.t.borrow_mut().outputs += 1;
	}
}

/// A file event of the C (`O mapsave|dump =path =bytes|none`, `O dump =bytes`): the port's file must have the bytes
fn check_file(n: usize, l: &str) {
	let tok: Vec<&str> = l.split(' ').collect();
	let (path, content) = (String::from_utf8(unesc(tok[2])).unwrap(), tok[3]);
	let ours = std::fs::read(&path).ok();
	match (content, ours) {
		("none", None) => {}
		("none", Some(_)) => fail(n, &format!("{} {} none", tok[1], tok[2]), "the file exists"),
		(c, Some(b)) if unesc(c) == b => {}
		(c, b) => {
			let want = unesc(c);
			let got = b.unwrap_or_default();
			let at = want.iter().zip(&got).position(|(a, b)| a != b).unwrap_or(want.len().min(got.len()));
			let ctx =
				|v: &[u8]| String::from_utf8_lossy(&v[at.saturating_sub(60)..(at + 60).min(v.len())]).into_owned();
			fail(n, &format!("{} {}: ...{}...", tok[1], tok[2], ctx(&want)), &format!("...{}...", ctx(&got)));
		}
	}
}

impl Host for TraceHost {
	fn run_time(&mut self) -> f64 {
		dbl(&self.read("rt"))
	}
	fn relative_time(&mut self) -> f64 {
		dbl(&self.read("wall"))
	}
	fn active_lighthouses(&self) -> i32 {
		self.read("alh").parse().unwrap()
	}
	fn get_bsd_idx(&mut self, channel: u8) -> i8 {
		let v = self.read(&format!("bsdidx {channel}"));
		v.parse().unwrap()
	}
	fn bsd_pose(&self, lh: usize) -> Pose {
		let v: Vec<f64> = self.read(&format!("pose {lh}")).split(' ').map(dbl).collect();
		Pose { pos: [v[0], v[1], v[2]], rot: [v[3], v[4], v[5], v[6]] }
	}
	fn bsd_position_set(&self, lh: usize) -> bool {
		self.read(&format!("posset {lh}")) == "1"
	}
	fn bsd_ootx_set(&self, lh: usize) -> bool {
		self.read(&format!("ootx {lh}")) == "1"
	}
	fn bsd_base_station_id(&self, lh: usize) -> u32 {
		self.read(&format!("id {lh}")).parse().unwrap()
	}
	fn bsd_mode(&self, lh: usize) -> u8 {
		self.read(&format!("mode {lh}")).parse().unwrap()
	}
	fn bsd_fcal(&self, lh: usize) -> [BaseStationCal; 2] {
		let v: Vec<f64> = self.read(&format!("fcal {lh}")).split(' ').map(dbl).collect();
		let c = |o: usize| BaseStationCal {
			phase: v[o],
			tilt: v[o + 1],
			curve: v[o + 2],
			gibpha: v[o + 3],
			gibmag: v[o + 4],
			ogeephase: v[o + 5],
			ogeemag: v[o + 6],
		};
		[c(0), c(7)]
	}
	fn set_bsd_fcal(&mut self, lh: usize, fcal: &[BaseStationCal; 2]) {
		self.output(format!("O setfcal {lh} {}", fcal_hex(fcal)));
	}
	fn bsd_sally_moved(&self, lh: usize) -> i8 {
		self.read(&format!("moved {lh}")).parse().unwrap()
	}
	fn set_bsd_sally_moved(&mut self, lh: usize, moved: i8) {
		self.output(format!("O setmoved {lh} {moved}"));
	}
	fn set_bsd_sally_moved_time(&mut self, lh: usize, t: f64) {
		self.output(format!("O setmovedtime {lh} {}", hex(t)));
	}
	fn raw_lighthouse_pose(&mut self, lh: usize, pose: &Pose) {
		self.output(format!("O lhpose {lh} {}", pose_hex(pose)));
	}
	fn sensor_ct(&self, dev: DeviceId) -> i32 {
		self.read(&format!("sct {dev}")).parse().unwrap()
	}
	fn sensor_locations_set(&self, dev: DeviceId) -> bool {
		self.read(&format!("hasloc {dev}")) == "1"
	}
	fn sensor_normals_set(&self, dev: DeviceId) -> bool {
		self.read(&format!("hasnorm {dev}")) == "1"
	}
	fn sensor_location(&self, dev: DeviceId, sensor: usize) -> [f64; 3] {
		let v: Vec<f64> = self.read(&format!("loc {dev} {sensor}")).split(' ').map(dbl).collect();
		[v[0], v[1], v[2]]
	}
	fn set_sensor_location(&mut self, dev: DeviceId, sensor: usize, loc: &[f64; 3]) {
		self.output(format!("O setloc {dev} {sensor} {} {} {}", hex(loc[0]), hex(loc[1]), hex(loc[2])));
	}
	fn sensor_normal(&self, dev: DeviceId, sensor: usize) -> [f64; 3] {
		let v: Vec<f64> = self.read(&format!("norm {dev} {sensor}")).split(' ').map(dbl).collect();
		[v[0], v[1], v[2]]
	}
	fn timebase_hz(&self, dev: DeviceId) -> i32 {
		self.read(&format!("hz {dev}")).parse().unwrap()
	}
	fn velocity(&self, dev: DeviceId) -> Velocity {
		let v: Vec<f64> = self.read(&format!("vel {dev}")).split(' ').map(dbl).collect();
		Velocity { pos: [v[0], v[1], v[2]], axis_angle_rot: [v[3], v[4], v[5]] }
	}
	fn serial_number(&self, dev: DeviceId) -> String {
		String::from_utf8(unesc(&self.read(&format!("serial {dev}")))).unwrap()
	}
	fn codename(&self, dev: DeviceId) -> String {
		String::from_utf8(unesc(&self.read(&format!("code {dev}")))).unwrap()
	}
	fn long_timecode_light(&self, dev: DeviceId, timecode: u32) -> u64 {
		self.read(&format!("ltc {dev} {timecode}")).parse().unwrap()
	}
	fn configf(&mut self, tag: &str, _def: f64) -> f64 {
		dbl(self.option(tag))
	}
	fn configi(&mut self, tag: &str, _def: u32) -> u32 {
		self.option(tag).parse::<i32>().unwrap() as u32
	}
	fn configs(&mut self, tag: &str, _def: &str) -> String {
		String::from_utf8(unesc(self.option(tag))).unwrap()
	}
	fn log_level(&self) -> i32 {
		self.option("v").parse().unwrap()
	}
	fn info(&mut self, msg: &str) {
		self.output(format!("O info {}", esc(msg.as_bytes())));
	}
	fn warn(&mut self, msg: &str) {
		self.output(format!("O warn {}", esc(msg.as_bytes())));
	}
	fn record(&mut self, line: &str) {
		self.output(format!("O rec {}", esc(line.as_bytes())));
	}
}

impl TraceHost {
	/// An option the C started with: one it did not have stops the replay
	fn option(&self, tag: &str) -> &str {
		match self.options.get(tag) {
			Some(v) => v,
			None => {
				let n = self.t.borrow().line_no;
				fail(n, "(the options the C read)", &format!("an option {tag}"));
			}
		}
	}
}

fn main() {
	let args: Vec<String> = std::env::args().collect();
	if args.len() < 2 {
		eprintln!("usage: trace_replay <trace> [--init-map <file>]");
		exit(2);
	}
	let init_map = args.iter().position(|a| a == "--init-map").map(|i| args[i + 1].clone());
	let f = std::fs::File::open(&args[1]).expect("the trace");
	let mut host = TraceHost {
		t: RefCell::new(Trace {
			lines: BufReader::with_capacity(1 << 20, f).lines(),
			line_no: 0,
			peeked: None,
			reads: 0,
			outputs: 0,
			files: 0,
		}),
		options: HashMap::new(),
	};
	let mut ms: Option<MapSolve> = None;
	let mut calls = [0u64; 4];
	let mut threaded_in_use = None;
	loop {
		let Some(l) = host.t.borrow_mut().next() else { break };
		let n = host.t.borrow().line_no;
		let tok: Vec<&str> = l.split(' ').collect();
		match (tok[0], tok.get(1).copied()) {
			("C", Some("start")) => {
				loop {
					let next = host.t.borrow_mut().peek().cloned();
					match next {
						Some(sl) if sl.starts_with("S ") && !sl.starts_with("S threaded-in-use") => {
							host.t.borrow_mut().next();
							let p: Vec<&str> = sl.splitn(3, ' ').collect();
							host.options.insert(p[1].to_string(), p[2].to_string());
						}
						_ => break,
					}
				}
				// the C's environment: SALLY_MAPSOLVE_DUMP as it was
				match host.options.get("dump").map(|v| v.as_str()) {
					Some("none") | None => std::env::remove_var("SALLY_MAPSOLVE_DUMP"),
					Some(v) => std::env::set_var("SALLY_MAPSOLVE_DUMP", String::from_utf8(unesc(v)).unwrap()),
				}
				if let Some(m) = &init_map {
					let file = String::from_utf8(unesc(host.option("map-solve-file"))).unwrap();
					std::fs::copy(m, file).expect("--init-map");
				}
				ms = Some(MapSolve::start(&mut host));
				calls[0] += 1;
			}
			("C", Some("sweep")) => {
				let m = ms.as_mut().expect("a map solve");
				m.sweep_angle(
					&mut host,
					tok[2].parse().unwrap(),
					tok[3].parse().unwrap(),
					tok[4].parse().unwrap(),
					tok[5].parse().unwrap(),
					tok[6].parse().unwrap(),
					dbl(tok[7]),
				);
				calls[1] += 1;
			}
			("C", Some("imupose")) => {
				let m = ms.as_mut().expect("a map solve");
				let v: Vec<f64> = tok[4..11].iter().map(|t| dbl(t)).collect();
				let pose = Pose { pos: [v[0], v[1], v[2]], rot: [v[3], v[4], v[5], v[6]] };
				m.imupose(&mut host, tok[2].parse().unwrap(), tok[3].parse().unwrap(), &pose);
				calls[2] += 1;
			}
			("C", Some("close")) => {
				ms.take().expect("a map solve").close(&mut host);
				calls[3] += 1;
			}
			_ => fail(n, &l, "a call"),
		}
		// the call's end: what the C did after the port's last read or output must be a file it wrote, then E
		loop {
			let next = host.t.borrow_mut().next();
			let n = host.t.borrow().line_no;
			match next {
				Some(e) if e == "E" => break,
				Some(e) if e.starts_with("O mapsave ") || e.starts_with("O dump ") => {
					host.t.borrow_mut().files += 1;
					check_file(n, &e);
				}
				Some(e) if e.starts_with("S threaded-in-use ") => threaded_in_use = Some(e[18..].to_string()),
				Some(e) => fail(n, &e, "the call's end"),
				None => fail(n, "E", "(the trace ends)"),
			}
		}
	}
	let t = host.t.borrow();
	println!(
		"{}: the same: {} calls (start {}, sweep {}, imupose {}, close {}), {} reads, {} outputs, {} files; the C solved {}",
		args[1],
		calls.iter().sum::<u64>(),
		calls[0],
		calls[1],
		calls[2],
		calls[3],
		t.reads,
		t.outputs,
		t.files,
		match threaded_in_use.as_deref() {
			Some("0") => "inline",
			Some(_) => "on its thread",
			None => "(not traced)",
		}
	);
}
