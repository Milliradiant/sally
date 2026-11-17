//! The protocol on loopback: a runtime with it registered on a free port, a client socket, and the lines the client
//! gets, against the texts driver_pose_udp.c sends

use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use sally_math::printf::Arg;
use sally_math::Pose;
use sally_track::host::SurviveVelocity;

use super::*;
use crate::config::Config;
use crate::diagnosis::Diagnosis;
use crate::log::{Log, LogLevel};
use crate::options;

/// A loopback port nobody holds right now
fn free_port() -> u16 {
	UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

/// A client: a loopback socket connected to the runtime's port
fn client(port: u16) -> UdpSocket {
	let s = UdpSocket::bind("127.0.0.1:0").unwrap();
	s.connect(("127.0.0.1", port)).unwrap();
	s.set_read_timeout(Some(Duration::from_millis(30))).unwrap();
	s
}

/// The datagrams waiting for a client, each as text
fn received(s: &UdpSocket) -> Vec<String> {
	let mut buf = [0u8; 4096];
	let mut out = Vec::new();
	while let Ok(n) = s.recv(&mut buf) {
		out.push(String::from_utf8_lossy(&buf[..n]).into_owned());
	}
	out
}

/// A runtime on the replay's clock with the protocol on a free port (and `args`), its log lines kept, a client
struct Bench {
	rt: Runtime,
	p: Box<Protocol>,
	c: UdpSocket,
	port: u16,
	log: Arc<Mutex<Vec<String>>>,
}

impl Bench {
	fn new(args: &[&str]) -> Bench {
		let port = free_port();
		let mut config = Config::default();
		options::register(&mut config);
		let port_text = port.to_string();
		let mut argv = vec!["sally", "--runtime-port", &port_text];
		argv.extend_from_slice(args);
		config.apply_args(&argv);
		let mut rt = Runtime::new(config, Log::new(None, false));
		let log = Arc::new(Mutex::new(Vec::new()));
		let sink = Arc::clone(&log);
		rt.install_log_fn(Some(Box::new(move |_: LogLevel, msg: &str| sink.lock().unwrap().push(msg.to_string()))));
		rt.set_replay_time(0.0);
		let Reg::Started(p) = Protocol::reg(&mut rt, false) else { panic!("the protocol did not start") };
		Bench { rt, p, c: client(port), port, log }
	}

	/// A command from the client, the poll that takes it, and what the client got
	fn cmd(&mut self, line: &str) -> Vec<String> {
		self.c.send(line.as_bytes()).unwrap();
		self.p.poll(&mut self.rt);
		received(&self.c)
	}

	/// Devices KN0, KN1, ... (serials SER0, ...) with `sensors` sensors at known places
	fn devices(&mut self, n: usize, sensors: i8) -> Vec<ObjectId> {
		(0..n)
			.map(|i| {
				let obj = self.rt.create_object("HID", &format!("KN{i}"), &format!("SER{i}"), "DONGLE").unwrap();
				let so = &mut self.rt.objects[obj];
				so.sensor_ct = sensors;
				so.sensor_locations = Some((0..3 * sensors as usize).map(|k| k as f64 * 0.001).collect());
				so.sensor_normals =
					Some((0..3 * sensors as usize).map(|k| if k % 3 == 2 { 1.0 } else { 0.0 }).collect());
				obj
			})
			.collect()
	}

	/// Two stations: LH 0 on channel 7 placed, LH 1 on channel 9 not
	fn stations(&mut self) {
		self.rt.active_lighthouses = 2;
		for (l, ch) in [(0, 7u8), (1, 9u8)] {
			let b = &mut self.rt.bsd[l];
			b.mode = ch;
			b.base_station_id = 0x1000 + l as u32;
			self.rt.bsd_map[ch as usize] = l as i8;
		}
		self.rt.bsd[0].position_set = true;
		self.rt.bsd[0].pose = Pose { pos: [1.0, -2.0, 2.5], rot: [0.5, 0.5, -0.5, 0.5] };
	}

	fn logged(&self, what: &str) -> Vec<String> {
		self.log.lock().unwrap().iter().filter(|l| l.contains(what)).cloned().collect()
	}
}

/// A line without its wall_epoch field (the `i`th, counting from 0)
fn without_wall(line: &str, i: usize) -> String {
	let mut f: Vec<&str> = line.split(' ').collect();
	assert!(f[i].parse::<f64>().unwrap() > 1.7e9, "{line}");
	f.remove(i);
	f.join(" ")
}

#[test]
fn commands_and_their_replies() {
	let mut b = Bench::new(&[]);
	assert_eq!(b.cmd("hello\n"), ["OK hello sally-runtime 5 0.000000 0 0\n"]);
	assert_eq!(
		b.cmd("help"),
		["OK help hello ping sub unsub get set params describe map devices sensors pair record quit reload calibrate\n"]
	);
	// the request id on every line, trailing newlines and spaces stripped, words split at spaces and tabs
	assert_eq!(b.cmd("#7 ping \r\n"), ["#7 OK ping 0.000000 not_subscribed\n"]);
	assert_eq!(b.cmd("#x\tget\t  nope"), ["#x\tget\t ERR nope unknown command (try help)\n"]);
	assert_eq!(b.cmd("#x get\t  nope"), ["#x ERR get unknown parameter 'nope'\n"]);
	// nothing to do: no reply
	assert!(b.cmd("   ").is_empty());
	assert!(b.cmd("#42").is_empty());
	// a NUL ends the command
	assert_eq!(b.cmd("hello\0junk"), ["OK hello sally-runtime 5 0.000000 0 0\n"]);
	// an id of 38 bytes at most
	let long_id = format!("#{}", "i".repeat(50));
	assert_eq!(b.cmd(&format!("{long_id} help"))[0][..41], format!("{} OK", &long_id[..38]));
	for (cmd, reply) in [
		("bogus", "ERR bogus unknown command (try help)"),
		("get", "ERR get usage: get <name>"),
		("set x", "ERR set usage: set <name> <value>"),
		("describe", "ERR describe usage: describe <name>"),
		("describe nope", "ERR describe unknown parameter 'nope'"),
		("sensors", "ERR sensors usage: sensors <codename>"),
		("sensors KN9", "ERR sensors unknown device 'KN9'"),
		("map bogus", "ERR map usage: map [lock|unlock]"),
		("record start", "ERR record usage: record [start <file.rec[.gz]>|stop]"),
		("record stop", "ERR record not recording"),
		("record start x.pcap", "ERR record only .rec/.rec.gz can start live (USB captures are set up at startup)"),
		("record start /no/such/dir/x.rec", "ERR record cannot open /no/such/dir/x.rec"),
		("record", "OK record idle"),
		("reload", "ERR reload not implemented yet (set parameters individually; config file values apply at start)"),
		("calibrate", "ERR calibrate needs the live runtime (the dongles, a config file and --hidtap-record)"),
		("sub pose nope", "ERR sub unknown stream 'nope' (pose, diag, all, driver, state)"),
	] {
		assert_eq!(b.cmd(cmd), [format!("{reply}\n")], "{cmd}");
	}
	// bytes that are not UTF-8 come back as they came
	b.c.send(b"#\xff hello\xfe").unwrap();
	b.p.poll(&mut b.rt);
	let mut buf = [0u8; 100];
	let n = b.c.recv(&mut buf).unwrap();
	assert_eq!(&buf[..n], b"#\xff ERR hello\xfe unknown command (try help)\n");
	// char line[1400]: the id and the text cut to 1398 bytes, then the newline
	let long = format!("#1 {}", "x".repeat(1500));
	let r = b.cmd(&long);
	assert_eq!(r[0].len(), 1399);
	assert_eq!(r[0], format!("{}\n", &format!("#1 ERR {} unknown command (try help)", "x".repeat(1500))[..1398]));
	assert_eq!(b.p.cmds, 27);
}

#[test]
fn subscribers_get_the_map_and_their_streams() {
	let mut b = Bench::new(&[]);
	b.stations();
	// a new subscriber: the reply, then the map (no id): an L line per placed station, an M line per station
	let r = b.cmd("#1 sub");
	assert_eq!(r.len(), 4, "{r:?}");
	assert_eq!(r[0], "#1 OK sub pose diag\n");
	assert_eq!(
		without_wall(&r[1], 3),
		"L 0 4096 1.000000 -2.000000 2.500000 0.5000000 0.5000000 -0.5000000 0.5000000\n"
	);
	assert_eq!(r[2], "M 0 4096 7 1 0 1.000000 -2.000000 2.500000 0.5000000 0.5000000 -0.5000000 0.5000000\n");
	assert_eq!(r[3], "M 1 4097 9 0 0 0.000000 0.000000 0.000000 0.0000000 0.0000000 0.0000000 0.0000000\n");
	assert_eq!(b.cmd("ping"), ["OK ping 0.000000 subscribed\n"]);
	// the same subscriber again: its streams changed, no map
	assert_eq!(b.cmd("sub driver state"), ["OK sub driver state\n"]);
	assert_eq!(b.p.subs.iter().filter(|s| s.used).count(), 1);
	assert_eq!(b.cmd("unsub"), ["OK unsub\n"]);
	assert_eq!(b.cmd("ping"), ["OK ping 0.000000 not_subscribed\n"]);
	// driver: the L lines only; diag: the M lines only
	let r = b.cmd("sub driver");
	assert_eq!((r.len(), &r[0][..], &r[1][..2]), (2, "OK sub driver\n", "L "));
	b.cmd("unsub");
	let r = b.cmd("sub diag");
	assert_eq!((r.len(), &r[0][..], &r[1][..4], &r[2][..4]), (3, "OK sub diag\n", "M 0 ", "M 1 "));
	// map lock / unlock: lock-lighthouses set live, and the M lines with id
	let r = b.cmd("#m map lock");
	assert_eq!(r[0], "#m M 0 4096 7 1 1 1.000000 -2.000000 2.500000 0.5000000 0.5000000 -0.5000000 0.5000000\n");
	assert_eq!(r[2], "#m OK map locked 2\n");
	assert_eq!(b.cmd("map unlock").last().unwrap(), "OK map unlocked 2\n");
	assert_eq!(b.logged("runtime: map"), ["runtime: map locked", "runtime: map unlocked"]);
}

#[test]
fn parameters_and_what_a_set_reaches() {
	let mut b = Bench::new(&["--diagnosis-back-deg", "100"]);
	b.devices(2, 3);
	let mut d = Diagnosis::reg(&mut b.rt).unwrap();
	// the trackers' and the light filter's options are bound per device, the context's and the diagnosis's once
	for (cmd, reply) in [
		("get track-gate", "OK get track-gate float 8 2"),
		("get track-only", "OK get track-only int 0 2"),
		("get move-threshold-gyro", "OK get move-threshold-gyro float 0.075 2"),
		("get filter-outlier-minimum-count", "OK get filter-outlier-minimum-count int 11 2"),
		("get lighthouse-max-update", "OK get lighthouse-max-update float 0.01 1"),
		("get diagnosis-back-deg", "OK get diagnosis-back-deg float 100 1"),
		("get record-imu", "OK get record-imu bool 1 0"),
		("get runtime-bind", "OK get runtime-bind string 127.0.0.1 0"),
		("get init-configfile", "OK get init-configfile string \"\" 0"),
		("get light-min-length", "OK get light-min-length int 100 0"),
		// an option's old name (core2-*) reaches the option, and the reply names it as it is now
		("get core2-gate", "OK get track-gate float 8 2"),
		("set core2-gate 6", "OK set track-gate float 8 6 2"),
		(
			"describe core2-zupt-gyro",
			"OK describe track-zupt-gyro float        0.020000     sally tracker: see track-zupt-sd (rad/s)",
		),
		("set track-gate 6.5", "OK set track-gate float 6 6.5 2"),
		("set track-only 0x80", "OK set track-only int 0 128 2"),
		("set move-threshold-gyro 0.1", "OK set move-threshold-gyro float 0.075 0.1 2"),
		("set lighthouse-max-update 0.5", "OK set lighthouse-max-update float 0.01 0.5 1"),
		("set diagnosis-back-deg 90", "OK set diagnosis-back-deg float 100 90 1"),
		("set lock-lighthouses on", "OK set lock-lighthouses bool 0 1 0"),
		("set lock-lighthouses false", "OK set lock-lighthouses bool 1 0 0"),
		("set runtime-bind a\tb", "OK set runtime-bind string 127.0.0.1 a 0"),
		("set track-gate 1e", "ERR set bad float value '1e' for track-gate"),
		("set lock-lighthouses yes", "ERR set bad bool value 'yes' for lock-lighthouses"),
		("set nope 1", "ERR set unknown parameter 'nope'"),
		("get track-gate", "OK get track-gate float 6.5 2"),
	] {
		assert_eq!(b.cmd(cmd), [format!("{reply}\n")], "{cmd}");
	}
	// the sets reached the variables
	assert_eq!(b.rt.station_moves.max_update.float(), 0.5);
	assert!(b.rt.objects.iter().all(|o| o.activations.params.move_threshold_gyro == 0.1));
	assert_eq!(b.rt.settings.tracker.gate, 6.5);
	assert_eq!(
		b.logged("runtime: set track-gate"),
		["runtime: set track-gate 8 -> 6 (2 bound)", "runtime: set track-gate 6 -> 6.5 (2 bound)"]
	);
	// a recording binds its four while it is open
	let rec = std::env::temp_dir().join(format!("sally-protocol-test-{}.rec", b.port));
	let rec_text = rec.to_string_lossy().into_owned();
	assert_eq!(b.cmd(&format!("record start {rec_text}")), [format!("OK record started {rec_text}\n")]);
	assert_eq!(b.cmd("record"), ["OK record recording\n"]);
	assert_eq!(b.cmd("set record-cal-imu 1"), ["OK set record-cal-imu bool 0 1 1\n"]);
	assert!(b.rt.recording.as_ref().unwrap().options.cal_imu.bool());
	assert_eq!(b.cmd("record stop"), ["OK record stopped\n"]);
	assert_eq!(b.cmd("get record-cal-imu"), ["OK get record-cal-imu bool 1 0\n"]);
	let _ = std::fs::remove_file(&rec);
	// params: the registered options from the prefix, in registration order
	let r = b.cmd("#p params runtime-");
	assert_eq!(
		r,
		[
			format!("#p PARAM runtime-port int {} 0\n", b.port),
			"#p PARAM runtime-bind string a 0\n".to_string(),
			"#p PARAM runtime-state-hz float 100 0\n".to_string(),
			"#p PARAM runtime-diag-period float 1 0\n".to_string(),
			"#p PARAM runtime-sub-timeout float 5 0\n".to_string(),
			"#p PARAM runtime-driver-file string \"\" 0\n".to_string(),
			"#p OK params 6\n".to_string(),
		]
	);
	let all = b.cmd("params");
	assert_eq!(all.len(), b.rt.config.options().len() + 1);
	// describe: the default as survive_config_iterate gives it, padded
	assert_eq!(
		b.cmd("describe init-configfile"),
		["OK describe init-configfile string          (null)     Initial configuration file\n"]
	);
	assert_eq!(
		b.cmd("describe runtime-state-hz"),
		["OK describe runtime-state-hz float      100.000000     sally: maximum rate of S (tracker state) lines per device; 0 = every pose\n"]
	);
	d.close(&mut b.rt);
	assert_eq!(b.cmd("get diagnosis-back-deg"), ["OK get diagnosis-back-deg float 90 0\n"]);
}

#[test]
fn the_pose_stream() {
	let mut b = Bench::new(&["--runtime-state-hz", "10"]);
	b.stations();
	let kn0 = b.devices(1, 3)[0];
	assert_eq!(b.cmd("sub all driver state").len(), 4);
	let pose = Pose { pos: [0.1, 0.2, 0.3], rot: [1.0, 0.0, 0.0, 0.0] };
	let vel = SurviveVelocity { pos: [0.01, -0.02, 0.03], axis_angle_rot: [0.1, 0.2, -0.3] };
	b.rt.set_replay_time(1.25);
	b.p.pose(&mut b.rt, kn0, 1000, &pose);
	b.p.velocity(&mut b.rt, kn0, 1000, &vel);
	let r = received(&b.c);
	assert_eq!(r.len(), 4, "{r:?}");
	assert_eq!(
		without_wall(&r[0], 5),
		"P KN0 SER0 1000 1.250000 0.100000 0.200000 0.300000 1.0000000 0.0000000 0.0000000 0.0000000\n"
	);
	assert_eq!(r[1], "S KN0 SER0 1000 1.250000 none -1.0000 -1.000000 -1.000000 0 -1.000\n");
	assert_eq!(without_wall(&r[2], 5), "V KN0 SER0 1000 1.250000 0.01000 -0.02000 0.03000 0.10000 0.20000 -0.30000\n");
	assert_eq!(
		r[3],
		concat!(
			"Q KN0 SER0 1000 1.250000 0.100000 0.200000 0.300000 1.0000000 0.0000000 0.0000000 0.0000000 0.01000 -0.02000 ",
			"0.03000 0.10000 0.20000 -0.30000 0.0000 0.0000 0.0000 0.00000 0.000000 0.000000 0.000000 0.00000 0.00000 ",
			"0.00000 0.000 0.000 0.000 0.000000 0.000000 0.000000 1.000000000 0.000000000 0.000000000 0.000000000 ",
			"-1.0000 0.000\n"
		)
	);
	// the next report within 0.1 s: no S line; a velocity of another report: no Q line
	b.rt.set_replay_time(1.3);
	b.p.pose(&mut b.rt, kn0, 1001, &pose);
	b.p.velocity(&mut b.rt, kn0, 1002, &vel);
	let r = received(&b.c);
	assert_eq!(r.iter().map(|l| &l[..1]).collect::<String>(), "PV");
	// a station moved: its L line
	b.p.lighthouse_pose(&mut b.rt, 0, &pose);
	assert_eq!(
		without_wall(&received(&b.c)[0], 3),
		"L 0 4096 0.100000 0.200000 0.300000 1.0000000 0.0000000 0.0000000 0.0000000\n"
	);
	// the driver stream alone: P only once the device has had no Q for 0.5 s
	b.cmd("unsub");
	b.cmd("sub driver");
	b.rt.set_replay_time(1.6);
	b.p.pose(&mut b.rt, kn0, 1003, &pose);
	assert!(received(&b.c).is_empty());
	b.rt.set_replay_time(1.9);
	b.p.pose(&mut b.rt, kn0, 1004, &pose);
	assert_eq!(&received(&b.c)[0][..6], "P KN0 ");
	assert_eq!(b.p.dropped, 0);
}

/// A line as the protocol built it before it had a line buffer (a String of sprintf's own, its bytes cut to size - 1),
/// to hold the buffer's lines to
fn old_line(size: usize, fmt: &str, args: &[Arg]) -> String {
	let mut l = sally_math::printf::sprintf(fmt, args).into_bytes();
	l.truncate(size - 1);
	String::from_utf8_lossy(&l).into_owned()
}

/// The pose stream's lines out of the line buffer are those each line's own String gave, to the byte, the cut at the
/// C's 511 bytes included: random poses and velocities with NaN, the infinities, -0 and values long enough to cut a
/// line, through P, S, V, Q and L
#[test]
fn the_lines_from_the_buffer_are_the_old_ones() {
	let mut b = Bench::new(&["--runtime-state-hz", "0"]);
	b.stations();
	let kn0 = b.devices(1, 3)[0];
	assert_eq!(b.cmd("sub all driver state").len(), 4);
	let mut x: u64 = 0x2545_f491_4f6c_dd1d;
	let mut value = move || {
		x ^= x << 13;
		x ^= x >> 7;
		x ^= x << 17;
		match x % 10 {
			0 => f64::NAN,
			1 => f64::NEG_INFINITY,
			2 => -0.0,
			3 => 1e300,
			4 => -1.7e308,
			_ => (x as i64 as f64) * 1e-18,
		}
	};
	let mut cut = 0;
	for i in 0..300u64 {
		let pose = Pose { pos: [value(), value(), value()], rot: [value(), value(), value(), value()] };
		let vel = SurviveVelocity { pos: [value(), value(), value()], axis_angle_rot: [value(), value(), value()] };
		let (t, tc) = (1.0 + i as f64 * 0.01, 1000 + i);
		b.rt.set_replay_time(t);
		b.p.pose(&mut b.rt, kn0, tc, &pose);
		b.p.velocity(&mut b.rt, kn0, tc, &vel);
		b.p.lighthouse_pose(&mut b.rt, 0, &pose);
		let got = received(&b.c);
		assert_eq!(got.len(), 5, "{got:?}");

		let d = b.rt.tracker_diag(kn0);
		let ahead = Protocol::look_ahead(&b.rt, kn0, &pose);
		let so = &b.rt.objects[kn0];
		// a wall clock of the same length as the one sent: the line is cut at the same byte, and the field is left out
		let wall = Arg::F(1.8e9);
		let head = || vec![Arg::S(&so.codename), Arg::S(serial_of(so)), Arg::U(tc), Arg::F(t)];
		let mut p = head();
		p.push(wall);
		p.extend(pose.pos.iter().chain(pose.rot.iter()).map(|&v| Arg::F(v)));
		let p = old_line(LINE, "P %s %s %llu %.6f %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", &p);
		let mut st = head();
		st.extend([
			Arg::S(state_name(d.state)),
			Arg::F(d.light_age),
			Arg::F(d.pos_sd),
			Arg::F(-1.0),
			Arg::U(d.lh_mask as u64),
			Arg::F(d.res_mrad),
		]);
		let st = old_line(LINE, "S %s %s %llu %.6f %s %.4f %.6f %.6f %x %.3f\n", &st);
		let mut v = head();
		v.push(wall);
		v.extend(vel.pos.iter().chain(vel.axis_angle_rot.iter()).map(|&x| Arg::F(x)));
		let v = old_line(LINE, "V %s %s %llu %.6f %.6f %.5f %.5f %.5f %.5f %.5f %.5f\n", &v);
		let mut q = head();
		q.extend(pose.pos.iter().chain(pose.rot.iter()).map(|&x| Arg::F(x)));
		q.extend(vel.pos.iter().chain(vel.axis_angle_rot.iter()).map(|&x| Arg::F(x)));
		q.extend(ahead.iter().map(|&x| Arg::F(x)));
		// no seeded tracker: no corrections and no return
		q.extend([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0].map(Arg::F));
		let q = old_line(LINE, Q_FMT, &q);
		let mut l = vec![Arg::I(0), Arg::U(b.rt.bsd[0].base_station_id as u64), wall];
		l.extend(pose.pos.iter().chain(pose.rot.iter()).map(|&v| Arg::F(v)));
		let l = old_line(LINE, "L %d %u %.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", &l);

		assert_eq!(without_wall(&got[0], 5), without_wall(&p, 5));
		assert_eq!(got[1], st);
		assert_eq!(without_wall(&got[2], 5), without_wall(&v, 5));
		assert_eq!(got[3], q);
		assert_eq!(without_wall(&got[4], 3), without_wall(&l, 3));
		cut += got.iter().filter(|g| g.len() == LINE - 1).count();
	}
	// lines long enough to be cut came through
	assert!(cut > 500, "{cut}");
	assert_eq!(b.p.dropped, 0);
}

#[test]
fn the_summaries_and_the_diagnosis() {
	let mut b = Bench::new(&[]);
	b.stations();
	let kn0 = b.devices(1, 3)[0];
	b.cmd("sub diag");
	for (sensor, n) in [(0, 2), (2, 1), (5, 4)] {
		for _ in 0..n {
			b.p.sweep_angle(&mut b.rt, kn0, 7, sensor, 0, 0, 0.0);
		}
	}
	// a period of run time: per device its H lines (the sensors it has) and its T line, per station its M line
	b.rt.set_replay_time(1.0);
	b.p.poll(&mut b.rt);
	let r = received(&b.c);
	assert_eq!(r[0], "H KN0 0 1.000 3 2 0 1\n");
	assert_eq!(r[1], "T KN0 1.000000 none -1.0000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n");
	assert_eq!((&r[2][..4], &r[3][..4], r.len()), ("M 0 ", "M 1 ", 4));
	// a station lost since: its F line, after its M line
	b.rt.set_replay_time(2.5);
	b.p.poll(&mut b.rt);
	let r = received(&b.c);
	assert_eq!(
		(r.len(), &r[0][..2], &r[1][..4], &r[2][..], &r[3][..4]),
		(4, "T ", "M 0 ", "F 0 lost 2.500000\n", "M 1 ")
	);
	// a verdict: its D line and a C line per cause
	let g = DeviceDiagnosis {
		run_time: 2.5,
		headline: 4,
		severity: 2,
		bars: 3,
		loss: 0.061,
		gap_max: 0.013,
		stations: 1,
		sensors: 3,
		back: 0.5,
		state: 2,
		light_age: 1.5,
		charge: 80,
		causes: vec![crate::diagnosis::Cause {
			cause: 4,
			severity: 2,
			detail: "no light for 1.5 s".into(),
			text: "can't see the base stations: no light for 1.5 s".into(),
		}],
		..Default::default()
	};
	b.p.diagnosis(&mut b.rt, kn0, &g);
	assert_eq!(
		received(&b.c),
		[
			"D KN0 2.500 light 2 3 6.1 13 1 3 50.0 0 0 dark 1.50 80 1\n",
			"C KN0 2.500 light 2 can't see the base stations: no light for 1.5 s\n"
		]
	);
}

#[test]
fn subscribers_time_out_and_the_stalest_makes_room() {
	let mut b = Bench::new(&["--runtime-sub-timeout", "0.05", "--runtime-diag-period", "0"]);
	b.cmd("sub pose");
	assert!(b.p.has_subscribers(STREAM_POSE));
	std::thread::sleep(Duration::from_millis(100));
	b.p.poll(&mut b.rt);
	assert!(!b.p.has_subscribers(STREAM_POSE));

	let mut b = Bench::new(&[]);
	let clients: Vec<UdpSocket> = (0..17).map(|_| client(b.port)).collect();
	for c in &clients {
		c.send(b"sub state").unwrap();
		b.p.poll(&mut b.rt);
	}
	let first = clients[0].local_addr().unwrap();
	let last = clients[16].local_addr().unwrap();
	assert!(b.p.subs.iter().all(|s| s.used && SocketAddr::V4(s.addr) != first));
	assert_eq!(SocketAddr::V4(b.p.subs[0].addr), last);
}

#[test]
fn the_devices_and_their_sensors() {
	let mut b = Bench::new(&[]);
	let kn = b.devices(2, 2);
	b.rt.objects[kn[1]].charge = 55;
	b.rt.objects[kn[1]].charging = true;
	b.rt.objects[kn[1]].object_subtype = 7;
	b.rt.objects[kn[1]].head2imu = Pose { pos: [0.01, 0.02, -0.03], rot: [0.0, 1.0, 0.0, 0.0] };
	assert_eq!(
		b.cmd("devices"),
		[
			"DEV KN0 SER0 2 none -1.0000 generic 48000000 0.000000 0.000000 0.000000 0.0000000 0.0000000 0.0000000 0.0000000 -1 0 DONGLE\n",
			"DEV KN1 SER1 2 none -1.0000 tracker_gen2 48000000 0.010000 0.020000 -0.030000 0.0000000 1.0000000 0.0000000 0.0000000 55 1 DONGLE\n",
			"OK devices 2\n",
		]
	);
	assert_eq!(
		b.cmd("sensors KN1"),
		[
			"SENS KN1 0 0.00000 0.00100 0.00200 0.0000 0.0000 1.0000\n",
			"SENS KN1 1 0.00300 0.00400 0.00500 0.0000 0.0000 1.0000\n",
			"OK sensors KN1 2\n",
		]
	);
	b.rt.objects[kn[0]].sensor_locations = None;
	assert_eq!(b.cmd("sensors KN0"), ["OK sensors KN0 0\n"]);
	assert_eq!(b.cmd("hello"), ["OK hello sally-runtime 5 0.000000 2 0\n"]);
}

#[test]
fn pair_hands_the_dongles_devices_to_the_main_loop() {
	let mut b = Bench::new(&[]);
	assert_eq!(b.cmd("#p pair 00A4AE2B50-RYB LHR-C57C5A45"), ["#p OK pair 00A4AE2B50-RYB LHR-C57C5A45\n"]);
	assert_eq!(b.cmd("pair A9DDD3B47A-LYM -"), ["OK pair A9DDD3B47A-LYM -\n"]);
	// a dongle named again: the newest device, once
	assert_eq!(b.cmd("pair 00A4AE2B50-RYB LHR-9365E531"), ["OK pair 00A4AE2B50-RYB LHR-9365E531\n"]);
	assert_eq!(b.cmd("pair 00A4AE2B50-RYB"), ["ERR pair usage: pair <dongle serial> <device serial|->\n"]);
	let pairs: Vec<(&str, &str)> = b.rt.pending_pairs.iter().map(|(d, s)| (d.as_str(), s.as_str())).collect();
	assert_eq!(pairs, [("A9DDD3B47A-LYM", "-"), ("00A4AE2B50-RYB", "LHR-9365E531")]);
	// a serial longer than the 31 / 15 bytes kept is no device's
	let too_long = ["ERR pair serial too long (a dongle's 31 bytes at most, a device's 15)\n"];
	assert_eq!(b.cmd(&format!("pair {} LHR-9365E531", "D".repeat(32))), too_long);
	assert_eq!(b.cmd(&format!("pair 00A4AE2B50-RYB {}", "L".repeat(16))), too_long);
	assert_eq!(b.cmd(&format!("pair {} {}", "D".repeat(31), "L".repeat(15))).len(), 1);
	assert_eq!(b.rt.pending_pairs.len(), 3);
	// the latest word on MAX_PAIRS dongles (the oldest goes)
	for i in 0..100 {
		b.cmd(&format!("pair DONGLE{i} LHR-{i}"));
	}
	assert_eq!(b.rt.pending_pairs.len(), sally_hid::tap::MAX_PAIRS);
	assert_eq!(b.rt.pending_pairs[0].0, "DONGLE36");
	assert_eq!(b.rt.pending_pairs.last().unwrap().0, "DONGLE99");
}

#[test]
fn quit_ends_the_session_and_close_sums_it_up() {
	let mut b = Bench::new(&[]);
	b.c.send(b"#q quit").unwrap();
	assert_eq!(b.p.poll(&mut b.rt), 1);
	assert_eq!(received(&b.c), ["#q OK quit\n"]);
	let from = b.c.local_addr().unwrap();
	assert_eq!(b.logged("quit"), [format!("runtime: quit requested by {from}"), "runtime: stopping (quit)".into()]);
	assert_eq!(b.logged("listening"), [format!("runtime: listening on 127.0.0.1:{} (protocol 5)", b.port)]);
	b.p.close(&mut b.rt);
	assert_eq!(b.logged("runtime: sent"), ["runtime: sent 1 datagrams, 0 dropped, 1 commands"]);
}

#[test]
fn the_legacy_push_and_what_does_not_start() {
	let target = UdpSocket::bind("127.0.0.1:0").unwrap();
	target.set_read_timeout(Some(Duration::from_millis(30))).unwrap();
	let target_port = target.local_addr().unwrap().port().to_string();
	let mut config = Config::default();
	options::register(&mut config);
	config.apply_args(&["sally", "--pose-udp", &target_port, "--pose-udp-host", "127.1"]);
	let mut rt = Runtime::new(config, Log::new(None, false));
	rt.set_replay_time(0.5);
	let Reg::Started(mut p) = Protocol::reg(&mut rt, false) else { panic!() };
	assert!(!p.bound);
	assert!(matches!(Protocol::reg(&mut rt, true), Reg::Passive));
	let obj = rt.create_object("HID", "KN0", "", "").unwrap();
	let pose = Pose { pos: [1.0, 2.0, 3.0], rot: [1.0, 0.0, 0.0, 0.0] };
	p.pose(&mut rt, obj, 5, &pose);
	p.velocity(&mut rt, obj, 5, &SurviveVelocity::default());
	let r = received(&target);
	assert_eq!(r.len(), 2);
	assert_eq!(
		without_wall(&r[0], 5),
		"P KN0 - 5 0.500000 1.000000 2.000000 3.000000 1.0000000 0.0000000 0.0000000 0.0000000\n"
	);
	assert_eq!(&r[1][..9], "V KN0 - 5");

	// neither on, a bad address, a port in use: no protocol
	let start = |args: &[&str]| {
		let mut config = Config::default();
		options::register(&mut config);
		let mut argv = vec!["sally"];
		argv.extend_from_slice(args);
		config.apply_args(&argv);
		let mut rt = Runtime::new(config, Log::new(None, false));
		let log = Arc::new(Mutex::new(Vec::new()));
		let sink = Arc::clone(&log);
		rt.install_log_fn(Some(Box::new(move |_: LogLevel, msg: &str| sink.lock().unwrap().push(msg.to_string()))));
		let r = Protocol::reg(&mut rt, false);
		let lines = log.lock().unwrap().clone();
		(matches!(r, Reg::Error(-1)), lines)
	};
	assert_eq!(start(&[]), (true, vec![]));
	assert_eq!(
		start(&["--runtime-port", "9", "--runtime-bind", "1.2.3.256"]),
		(true, vec!["runtime: bad bind address '1.2.3.256'".to_string()])
	);
	let held = UdpSocket::bind("127.0.0.1:0").unwrap();
	let port = held.local_addr().unwrap().port().to_string();
	let msg = format!("runtime: cannot bind 127.0.0.1:{port} (another runtime running?)");
	assert_eq!(start(&["--runtime-port", &port]), (true, vec![msg]));
	assert_eq!(
		start(&["--pose-udp", "9", "--pose-udp-host", "nowhere"]),
		(true, vec!["pose-udp: bad host 'nowhere'".to_string()])
	);
}

/// `--runtime-driver-file` alone starts the protocol with no port, and the file gets the driver stream: a device's DEV
/// line before its first Q, every Q line, and the P line of a device without a velocity
#[test]
fn the_driver_stream_as_a_file() {
	let file = std::env::temp_dir().join(format!("sally-driver-stream-test-{}.txt", std::process::id()));
	let name = file.to_string_lossy().into_owned();
	let mut config = Config::default();
	options::register(&mut config);
	config.apply_args(&["sally", "--runtime-driver-file", &name]);
	let mut rt = Runtime::new(config, Log::new(None, false));
	rt.set_replay_time(0.5);
	let Reg::Started(mut p) = Protocol::reg(&mut rt, false) else { panic!("the protocol did not start") };
	assert!(!p.bound);
	let kn0 = rt.create_object("HID", "KN0", "SER0", "DONGLE").unwrap();
	let kn1 = rt.create_object("HID", "KN1", "SER1", "DONGLE").unwrap();
	let pose = Pose { pos: [1.0, 2.0, 3.0], rot: [1.0, 0.0, 0.0, 0.0] };
	for tc in [5, 6] {
		p.pose(&mut rt, kn0, tc, &pose);
		p.velocity(&mut rt, kn0, tc, &SurviveVelocity::default());
	}
	// a pose alone: no Q line of this device yet, so its P line is the driver stream's
	p.pose(&mut rt, kn1, 7, &pose);
	p.close(&mut rt);
	let text = std::fs::read_to_string(&file).unwrap();
	let _ = std::fs::remove_file(&file);
	let lines: Vec<&str> = text.lines().collect();
	let starts: Vec<&str> = lines.iter().map(|l| &l[..l.len().min(13)]).collect();
	assert_eq!(starts, ["P KN0 SER0 5 ", "DEV KN0 SER0 ", "Q KN0 SER0 5 ", "Q KN0 SER0 6 ", "P KN1 SER1 7 "], "{text}");
	assert_eq!(lines[2].split(' ').count(), 40, "{}", lines[2]);
}

#[test]
fn addresses_as_inet_addr_reads_them() {
	let a = |s: &str| sys::inet_addr_of(s.as_bytes()).map(|ip| ip.to_string());
	assert_eq!(a("127.0.0.1").as_deref(), Some("127.0.0.1"));
	assert_eq!(a("127.1").as_deref(), Some("127.0.0.1"));
	assert_eq!(a("0x7f.0.0.010").as_deref(), Some("127.0.0.8"));
	assert_eq!(a("10.1.65535").as_deref(), Some("10.1.255.255"));
	assert_eq!(a("1.2.3.256"), None);
	assert_eq!(a("255.255.255.255"), None);
	assert_eq!(a(""), None);
	assert_eq!(a("localhost"), None);
}
