//! The control channel (driver_pose_udp.c's handle_command): one command per datagram, its reply to the socket it
//! came from, one line per datagram, a multi-line reply ending with its OK line. A command may start with `#<id> `,
//! which every line of its reply then starts with. The texts are the C's to the byte: a command's words are bytes, as
//! the C's are, and come back in the replies as they came.

use std::net::SocketAddrV4;

use sally_math::printf::{sprintf, Arg};

use super::{
	cut, format_lh, format_map_line, serial_of, state_name, stations, Protocol, LINE, PROTOCOL_VERSION, START,
	STREAM_DIAG, STREAM_DRIVER, STREAM_POSE, STREAM_STATE,
};
use sally_hid::tap::MAX_PAIRS;

use crate::config::{Config, SC_GET};
use crate::runtime::Runtime;
use crate::ObjectId;

/// A printf argument whose strings are bytes
#[derive(Clone, Copy)]
enum A<'a> {
	S(&'a [u8]),
	I(i64),
	F(f64),
}

/// sprintf with byte strings, as the C's printf takes them: each byte goes through as one char (U+0000 to U+00FF), so
/// a width or a precision counts bytes, and comes back as that byte
fn bsprintf(fmt: &str, args: &[A]) -> Vec<u8> {
	let texts: Vec<String> = args
		.iter()
		.map(|a| match a {
			A::S(b) => b.iter().map(|&c| c as char).collect(),
			_ => String::new(),
		})
		.collect();
	let args: Vec<Arg> = args
		.iter()
		.zip(&texts)
		.map(|(a, t)| match *a {
			A::S(_) => Arg::S(t),
			A::I(v) => Arg::I(v),
			A::F(v) => Arg::F(v),
		})
		.collect();
	sprintf(fmt, &args).chars().map(|c| c as u32 as u8).collect()
}

/// type_name: an option's type char as the replies name it
fn type_name(type_char: u8) -> &'static [u8] {
	match type_char {
		b'f' => b"float",
		b'i' => b"int",
		b'b' => b"bool",
		b's' => b"string",
		_ => b"unknown",
	}
}

/// format_value: an option's value as the replies give it, in char out[256]: a float by "%.9g", an int or a bool by
/// "%d", a string as it is ("" when empty or none) with its spaces, tabs, CRs and LFs made '_' (one token on the wire)
fn format_value(config: &mut Config, name: &[u8], type_char: u8) -> Vec<u8> {
	let mut out = match type_char {
		b'f' => sprintf("%.9g", &[Arg::F(config.configf(name, SC_GET, 0.0))]).into_bytes(),
		b'i' => sprintf("%d", &[Arg::I(config.configi(name, SC_GET, 0) as i32 as i64)]).into_bytes(),
		b'b' => sprintf("%d", &[Arg::I(config.configb(name, SC_GET, false) as i64)]).into_bytes(),
		b's' => match config.configs(name, SC_GET, Some(b"")) {
			Some(s) if !s.is_empty() => s,
			_ => b"\"\"".to_vec(),
		},
		_ => b"?".to_vec(),
	};
	out.truncate(255);
	if type_char == b's' {
		for c in out.iter_mut() {
			if matches!(*c, b' ' | b'\t' | b'\r' | b'\n') {
				*c = b'_';
			}
		}
	}
	out
}

/// subtype_name: a device's subtype as the DEV lines name it
fn subtype_name(subtype: i32) -> &'static str {
	const NAMES: [&str; 8] =
		["generic", "vive_hmd", "index_hmd", "wand", "knuckles_r", "knuckles_l", "tracker", "tracker_gen2"];
	if subtype >= 0 && (subtype as usize) < NAMES.len() {
		NAMES[subtype as usize]
	} else {
		"generic"
	}
}

/// A device's DEV line (`devices`' reply, and the driver file's line before a device's first Q): who it is, its
/// state, its head against its IMU, its battery and its dongle
pub(super) fn dev_line(rt: &mut Runtime, obj: ObjectId) -> String {
	let d = rt.tracker_diag(obj);
	let so = &rt.objects[obj];
	let h = &so.head2imu;
	let dongle = if so.sally_dongle.is_empty() { "-" } else { &so.sally_dongle };
	let mut args = vec![
		Arg::S(&so.codename),
		Arg::S(serial_of(so)),
		Arg::I(so.sensor_ct as i64),
		Arg::S(state_name(d.state)),
		Arg::F(d.light_age),
		Arg::S(subtype_name(so.object_subtype)),
		Arg::I(so.timebase_hz as i64),
	];
	args.extend(h.pos.iter().chain(h.rot.iter()).map(|&v| Arg::F(v)));
	args.push(Arg::I(if so.charge > 0 { so.charge as i64 } else { -1 }));
	args.push(Arg::I(so.charging as i64));
	args.push(Arg::S(dongle));
	sprintf("DEV %s %s %d %s %.4f %s %d %.6f %.6f %.6f %.7f %.7f %.7f %.7f %d %d %s", &args)
}

/// Reply: where a command's reply goes, and the "#<id> " its lines start with ("" without one)
struct Reply {
	to: SocketAddrV4,
	id: Vec<u8>,
}

impl Protocol {
	/// reply: one line of a command's reply, as char line[1400] holds it: the id and the text (cut to 1398 bytes
	/// when the two do not fit in 1399), then its newline
	fn reply(&mut self, r: &Reply, text: &[u8]) {
		let mut line = r.id.clone();
		line.extend_from_slice(text);
		if line.len() >= 1399 {
			line.truncate(1398);
		}
		line.push(b'\n');
		self.send_to(r.to, &line);
	}

	/// find_sub: the subscriber at this address, or with `create` a new one in a free slot (all taken: in the stalest
	/// subscriber's)
	fn find_sub(&mut self, from: SocketAddrV4, create: bool) -> Option<usize> {
		let (mut free, mut oldest): (Option<usize>, Option<usize>) = (None, None);
		for i in 0..self.subs.len() {
			let s = &self.subs[i];
			if s.used && s.addr == from {
				return Some(i);
			}
			if !s.used && free.is_none() {
				free = Some(i);
			}
			if s.used && oldest.is_none_or(|o| s.last_seen < self.subs[o].last_seen) {
				oldest = Some(i);
			}
		}
		if !create {
			return None;
		}
		let i = free.or(oldest)?;
		self.subs[i] = super::Sub { addr: from, used: true, ..Default::default() };
		Some(i)
	}

	/// handle_command: a command (a C string: it ends at a NUL), without its trailing newlines and spaces, its
	/// "#<id> " (38 bytes at most kept) taken off, split at spaces and tabs into 8 words at most (strtok)
	pub(super) fn handle_command(&mut self, rt: &mut Runtime, msg: &[u8], from: SocketAddrV4) {
		self.cmds += 1;
		let msg = &msg[..msg.iter().position(|&c| c == 0).unwrap_or(msg.len())];
		let mut len = msg.len();
		while len > 0 && matches!(msg[len - 1], b'\n' | b'\r' | b' ') {
			len -= 1;
		}
		let mut s = &msg[..len];
		let mut id = Vec::new();
		if s.first() == Some(&b'#') {
			let sp = s.iter().position(|&c| c == b' ');
			let idlen = sp.unwrap_or(s.len()).min(38);
			id.extend_from_slice(&s[..idlen]);
			id.push(b' ');
			s = match sp {
				Some(p) => &s[p + 1..],
				None => &s[s.len()..],
			};
		}
		let r = Reply { to: from, id };
		let argv: Vec<&[u8]> = s.split(|&c| c == b' ' || c == b'\t').filter(|t| !t.is_empty()).take(8).collect();
		let Some(&cmd) = argv.first() else { return };
		let argc = argv.len();

		match cmd {
			b"hello" => {
				let t = rt.run_time();
				let objs = rt.listed_objects().count() as i64;
				let text = bsprintf(
					"OK hello sally-runtime %d %.6f %d %d",
					&[A::I(PROTOCOL_VERSION as i64), A::F(t), A::I(objs), A::I(rt.active_lighthouses as i64)],
				);
				self.reply(&r, &text);
			}
			b"help" => self.reply(
				&r,
				b"OK help hello ping sub unsub get set params describe map devices sensors pair record quit reload calibrate",
			),
			b"ping" => {
				let sub = self.find_sub(from, false);
				if let Some(i) = sub {
					self.subs[i].last_seen = START.relative_time();
				}
				let t = rt.run_time();
				let state: &[u8] = if sub.is_some() { b"subscribed" } else { b"not_subscribed" };
				self.reply(&r, &bsprintf("OK ping %.6f %s", &[A::F(t), A::S(state)]));
			}
			b"sub" => self.sub(rt, &r, &argv, from),
			b"unsub" => {
				if let Some(i) = self.find_sub(from, false) {
					self.subs[i].used = false;
				}
				self.reply(&r, b"OK unsub");
			}
			b"get" => {
				if argc < 2 {
					return self.reply(&r, b"ERR get usage: get <name>");
				}
				let name = rt.config.current_name(argv[1]);
				let name: &[u8] = &name;
				let Some(type_char) = rt.config.value_type(name) else {
					return self.reply(&r, &bsprintf("ERR get unknown parameter '%s'", &[A::S(name)]));
				};
				let value = format_value(&mut rt.config, name, type_char);
				let bound = rt.bound_count(name) as i64;
				let text = bsprintf(
					"OK get %s %s %s %d",
					&[A::S(name), A::S(type_name(type_char)), A::S(&value), A::I(bound)],
				);
				self.reply(&r, &text);
			}
			b"set" => self.set(rt, &r, &argv),
			b"params" => {
				let prefix = argv.get(1).copied();
				let options: Vec<(Vec<u8>, u8)> =
					rt.config.options().iter().map(|o| (o.name().to_vec(), o.type_char())).collect();
				let mut count = 0;
				for (name, type_char) in options {
					if prefix.is_some_and(|p| !name.starts_with(p)) {
						continue;
					}
					let value = format_value(&mut rt.config, &name, type_char);
					let bound = rt.bound_count(&name) as i64;
					let text = bsprintf(
						"PARAM %s %s %s %d",
						&[A::S(&name), A::S(type_name(type_char)), A::S(&value), A::I(bound)],
					);
					self.reply(&r, &text);
					count += 1;
				}
				self.reply(&r, &bsprintf("OK params %d", &[A::I(count)]));
			}
			b"describe" => {
				if argc < 2 {
					return self.reply(&r, b"ERR describe usage: describe <name>");
				}
				let name = rt.config.current_name(argv[1]);
				let name: &[u8] = &name;
				// describe_iter: the last option of that name (char default_value[128] holds the iterator's text)
				let found = rt.config.options().iter().rev().find(|o| o.name() == name).map(|o| {
					let mut default = Config::default_text(o);
					default.truncate(127);
					(o.type_char(), default, o.description().to_vec())
				});
				let Some((type_char, default, description)) = found else {
					return self.reply(&r, &bsprintf("ERR describe unknown parameter '%s'", &[A::S(name)]));
				};
				let default: &[u8] = if default.is_empty() { b"\"\"" } else { &default };
				let text = bsprintf(
					"OK describe %s %s %s %s",
					&[A::S(name), A::S(type_name(type_char)), A::S(default), A::S(&description)],
				);
				self.reply(&r, &text);
			}
			b"map" => {
				if argc > 1 && (argv[1] == b"lock" || argv[1] == b"unlock") {
					let lock = argv[1] == b"lock";
					rt.set_live("lock-lighthouses", if lock { "1" } else { "0" });
					rt.info(if lock { "runtime: map locked" } else { "runtime: map unlocked" });
				} else if argc > 1 {
					return self.reply(&r, b"ERR map usage: map [lock|unlock]");
				}
				let locked = rt.config.configi("lock-lighthouses", SC_GET, 0) != 0;
				let mut m = String::new();
				for lh in 0..stations(rt) {
					m.clear();
					format_map_line(&mut m, rt, lh, locked);
					// the line as char line[LINE] holds it, without its last byte (its newline)
					let line = cut(&m, LINE);
					self.reply(&r, &line[..line.len().saturating_sub(1)]);
				}
				let state: &[u8] = if locked { b"locked" } else { b"unlocked" };
				self.reply(&r, &bsprintf("OK map %s %d", &[A::S(state), A::I(rt.active_lighthouses as i64)]));
			}
			b"devices" => {
				let objs: Vec<_> = rt.listed_objects().collect();
				for &obj in &objs {
					let text = dev_line(rt, obj);
					self.reply(&r, text.as_bytes());
				}
				self.reply(&r, &bsprintf("OK devices %d", &[A::I(objs.len() as i64)]));
			}
			b"sensors" => {
				if argc < 2 {
					return self.reply(&r, b"ERR sensors usage: sensors <codename>");
				}
				// the last device of that name
				let Some(obj) = rt.listed_objects().filter(|&o| rt.objects[o].codename.as_bytes() == argv[1]).last()
				else {
					return self.reply(&r, &bsprintf("ERR sensors unknown device '%s'", &[A::S(argv[1])]));
				};
				let so = &rt.objects[obj];
				let n = if so.sensor_locations.is_some() { so.sensor_ct as i64 } else { 0 };
				// (the config gives the locations and the normals for each of its sensor_ct sensors)
				let at = |v: Option<&Vec<f64>>, j: usize| v.and_then(|v| v.get(j).copied()).unwrap_or(0.0);
				let mut lines = Vec::new();
				for i in 0..n.max(0) as usize {
					let (l, nm) = (so.sensor_locations.as_ref(), so.sensor_normals.as_ref());
					let args = [
						Arg::S(&so.codename),
						Arg::I(i as i64),
						Arg::F(at(l, 3 * i)),
						Arg::F(at(l, 3 * i + 1)),
						Arg::F(at(l, 3 * i + 2)),
						Arg::F(at(nm, 3 * i)),
						Arg::F(at(nm, 3 * i + 1)),
						Arg::F(at(nm, 3 * i + 2)),
					];
					lines.push(sprintf("SENS %s %d %.5f %.5f %.5f %.4f %.4f %.4f", &args));
				}
				lines.push(sprintf("OK sensors %s %d", &[Arg::S(&so.codename), Arg::I(n)]));
				for l in lines {
					self.reply(&r, l.as_bytes());
				}
			}
			b"pair" => {
				// sally: the device a dongle carries, as SteamVR says (the driver's): the main loop gives it to the
				// live tap (Context::apply_pairs), whose devices and recording follow
				if argc < 3 {
					return self.reply(&r, b"ERR pair usage: pair <dongle serial> <device serial|->");
				}
				// a dongle's serial is kept in 31 bytes (char serial[32]) and a device's in 15 (char serial_number[16]):
				// a longer one is no device's
				if argv[1].len() > 31 || argv[2].len() > 15 {
					return self.reply(&r, b"ERR pair serial too long (a dongle's 31 bytes at most, a device's 15)");
				}
				let text = |b: &[u8]| String::from_utf8_lossy(b).into_owned();
				let (dongle, device) = (text(argv[1]), text(argv[2]));
				rt.pending_pairs.retain(|(d, _)| *d != dongle);
				rt.pending_pairs.push((dongle, device));
				// the latest word on MAX_PAIRS dongles (a client naming made-up ones fills nothing: the main loop takes
				// them every poll, a replay never)
				if rt.pending_pairs.len() > MAX_PAIRS {
					rt.pending_pairs.remove(0);
				}
				self.reply(&r, &bsprintf("OK pair %s %s", &[A::S(argv[1]), A::S(argv[2])]));
			}
			b"record" => {
				if argc > 2 && argv[1] == b"start" {
					let path = String::from_utf8_lossy(argv[2]).into_owned();
					let text = match rt.recording_start(&path) {
						0 => bsprintf("OK record started %s", &[A::S(argv[2])]),
						-1 => b"ERR record already recording".to_vec(),
						-2 => b"ERR record only .rec/.rec.gz can start live (USB captures are set up at startup)".to_vec(),
						_ => bsprintf("ERR record cannot open %s", &[A::S(argv[2])]),
					};
					self.reply(&r, &text);
				} else if argc > 1 && argv[1] == b"stop" {
					let text: &[u8] =
						if rt.recording_stop() == 0 { b"OK record stopped" } else { b"ERR record not recording" };
					self.reply(&r, text);
				} else if argc == 1 {
					let text: &[u8] = if rt.recording.is_some() { b"OK record recording" } else { b"OK record idle" };
					self.reply(&r, text);
				} else {
					self.reply(&r, b"ERR record usage: record [start <file.rec[.gz]>|stop]");
				}
			}
			b"quit" => {
				// used by the SteamVR driver to stop a runtime it started; ends like --stop-after (recordings are closed)
				rt.info(&format!("runtime: quit requested by {}:{}", from.ip(), from.port()));
				self.quit = true;
				self.reply(&r, b"OK quit");
			}
			b"reload" => self.reply(
				&r,
				b"ERR reload not implemented yet (set parameters individually; config file values apply at start)",
			),
			b"calibrate" => {
				// sally: the live runtime's (`sally run`'s calibrator): a calibration of this session so far, where one
				// stands, or the runtime stopped to start again on it
				let Some(mut c) = rt.calibrator.take() else {
					return self.reply(
						&r,
						b"ERR calibrate needs the live runtime (the dongles, a config file and --hidtap-record)",
					);
				};
				let (text, quit) = c.command(rt, &argv[1..argc]);
				rt.calibrator = Some(c);
				if quit {
					rt.info(&format!("runtime: stopping for the new calibration ({}:{})", from.ip(), from.port()));
					self.quit = true;
				}
				self.reply(&r, &text);
			}
			_ => self.reply(&r, &bsprintf("ERR %s unknown command (try help)", &[A::S(cmd)])),
		}
	}

	/// sub: the streams named (none: pose and diag) for the socket the command came from; a new subscriber gets the
	/// map right away instead of waiting for the next summary: an L line per placed station (pose or driver stream),
	/// an M line per station (diag), without the reply's id
	fn sub(&mut self, rt: &mut Runtime, r: &Reply, argv: &[&[u8]], from: SocketAddrV4) {
		let mut mask = 0;
		for &s in &argv[1..] {
			mask |= match s {
				b"pose" => STREAM_POSE,
				b"diag" => STREAM_DIAG,
				b"all" => STREAM_POSE | STREAM_DIAG,
				b"driver" => STREAM_DRIVER,
				b"state" => STREAM_STATE,
				_ => {
					let text = bsprintf("ERR sub unknown stream '%s' (pose, diag, all, driver, state)", &[A::S(s)]);
					return self.reply(r, &text);
				}
			};
		}
		if argv.len() == 1 {
			mask = STREAM_POSE | STREAM_DIAG;
		}
		let Some(i) = self.find_sub(from, true) else { return };
		let fresh = self.subs[i].mask == 0;
		self.subs[i].mask = mask;
		self.subs[i].last_seen = START.relative_time();
		let mut text = b"OK sub".to_vec();
		for (stream, name) in [
			(STREAM_POSE, &b" pose"[..]),
			(STREAM_DIAG, b" diag"),
			(STREAM_DRIVER, b" driver"),
			(STREAM_STATE, b" state"),
		] {
			if mask & stream != 0 {
				text.extend_from_slice(name);
			}
		}
		self.reply(r, &text);
		if fresh {
			let locked = rt.config.configi("lock-lighthouses", SC_GET, 0) != 0;
			let mut line = String::new();
			for lh in 0..stations(rt) {
				if mask & (STREAM_POSE | STREAM_DRIVER) != 0 && rt.bsd[lh].position_set {
					let pose = rt.bsd[lh].pose;
					line.clear();
					format_lh(&mut line, rt, lh, &pose);
					self.send_to(from, cut(&line, LINE));
				}
				if mask & STREAM_DIAG != 0 {
					line.clear();
					format_map_line(&mut line, rt, lh, locked);
					self.send_to(from, cut(&line, LINE));
				}
			}
		}
	}

	/// set: an option set live (survive_config_set_live: the store, and the variables bound to it), its old and new
	/// values and the number of variables it reached, logged and replied
	fn set(&mut self, rt: &mut Runtime, r: &Reply, argv: &[&[u8]]) {
		if argv.len() < 3 {
			return self.reply(r, b"ERR set usage: set <name> <value>");
		}
		let (name, value) = (rt.config.current_name(argv[1]), argv[2]);
		let name: &[u8] = &name;
		let Some(type_char) = rt.config.value_type(name) else {
			return self.reply(r, &bsprintf("ERR set unknown parameter '%s'", &[A::S(name)]));
		};
		let old = format_value(&mut rt.config, name, type_char);
		let bound = rt.set_live(name, value);
		if bound == -2 {
			let text =
				bsprintf("ERR set bad %s value '%s' for %s", &[A::S(type_name(type_char)), A::S(value), A::S(name)]);
			return self.reply(r, &text);
		} else if bound < 0 {
			return self.reply(r, &bsprintf("ERR set could not set %s", &[A::S(name)]));
		}
		let now = format_value(&mut rt.config, name, type_char);
		let log =
			bsprintf("runtime: set %s %s -> %s (%d bound)", &[A::S(name), A::S(&old), A::S(&now), A::I(bound as i64)]);
		rt.info(&String::from_utf8_lossy(&log));
		let text = bsprintf(
			"OK set %s %s %s %s %d",
			&[A::S(name), A::S(type_name(type_char)), A::S(&old), A::S(&now), A::I(bound as i64)],
		);
		self.reply(r, &text);
	}
}
