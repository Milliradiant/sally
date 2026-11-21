use super::*;

/// A runtime fed lines by hand at the panel time `t` (no socket)
fn fed(t: f64, lines: &[&str]) -> Runtime {
	let mut rt = Runtime::new();
	feed(&mut rt, t, lines);
	rt
}

fn feed(rt: &mut Runtime, t: f64, lines: &[&str]) {
	rt.now = t;
	for l in lines {
		rt.handle_line(l);
	}
}

const Q0: &str = "Q KN0 LHR-0334A73B 1000 12.5 0.1 0.2 1.0 1 0 0 0 0.3 0.4 0 0 0 1 0 0 0 0.007 0 0 0 0 0 0 0 0 0";

#[test]
fn a_pose_line_makes_a_device_with_its_pose() {
	let rt = fed(1.0, &[Q0]);
	let d = &rt.devices["KN0"];
	assert_eq!(d.serial, "LHR-0334A73B");
	assert!(d.have_pose);
	assert_eq!((d.pos.x, d.pos.y, d.pos.z), (0.1, 0.2, 1.0));
	assert!((len(d.vel) - 0.5).abs() < 1e-12);
	assert_eq!(rt.run_time, 12.5);
	assert_eq!(d.trail.len(), 1);
	assert_eq!(d.history.samples.len(), 1);
	assert_eq!(d.history.samples[0].speed, 0.5);
}

#[test]
fn p_lines_are_ignored_while_q_lines_come() {
	let mut rt = fed(1.0, &[Q0]);
	feed(&mut rt, 1.2, &["P KN0 LHR-0334A73B 1000 12.6 1.7e9 5 5 5 1 0 0 0"]);
	assert_eq!(rt.devices["KN0"].pos.x, 0.1);
	feed(&mut rt, 1.6, &["P KN0 LHR-0334A73B 1000 12.6 1.7e9 5 5 5 1 0 0 0"]);
	assert_eq!(rt.devices["KN0"].pos.x, 5.0);
}

#[test]
fn state_changes_are_events_and_drops() {
	let mut rt = fed(1.0, &[Q0, "S KN0 LHR-0334A73B 1000 12.5 tracking 0.004 0.0012 -1 3"]);
	assert_eq!(rt.events.back().unwrap().text, "tracking");
	let d = &rt.devices["KN0"];
	assert_eq!((d.lh_mask, d.pos_sd, d.res_mrad), (3, 0.0012, -1.0));
	// a brief gap in its light (a hand over it): nothing while it lasts, and a line that does not matter after
	let n = rt.events.len();
	feed(&mut rt, 2.0, &["S KN0 LHR-0334A73B 1000 13.5 coasting 0.7 0.01 -1 0"]);
	assert_eq!(rt.events.len(), n);
	feed(&mut rt, 2.5, &["S KN0 LHR-0334A73B 1000 14.0 tracking 0.004 0.0012 -1 f"]);
	let e = rt.events.back().unwrap();
	assert_eq!((e.text.as_str(), e.level, e.matters), ("0.70 s without light", EvLevel::Info, false));
	// the residual, the eleventh field
	feed(&mut rt, 2.6, &["S KN0 LHR-0334A73B 1000 14.1 tracking 0.004 0.0012 -1 f 1.375"]);
	assert_eq!(rt.devices["KN0"].res_mrad, 1.375);
	let d = &rt.devices["KN0"];
	assert_eq!(d.drops.len(), 1);
	assert_eq!(d.drops_last(600.0, 3.0), 1);
	assert_eq!(d.lh_mask, 0xf);
	// a short dark spell is no event
	let n = rt.events.len();
	feed(&mut rt, 3.0, &["S KN0 LHR-0334A73B 1000 14.5 dark 0.15 0.002 -1 1 -1.000"]);
	feed(&mut rt, 3.1, &["S KN0 LHR-0334A73B 1000 14.6 tracking 0.004 0.0012 -1 3"]);
	assert_eq!(rt.events.len(), n);
	assert_eq!(rt.devices["KN0"].drops.len(), 2);
	// a gap that holds is said once, as a warning and no more, and once more when the light is back
	feed(&mut rt, 4.0, &["S KN0 LHR-0334A73B 1000 15.5 coasting 0.9 0.01 -1 0"]);
	feed(&mut rt, 5.2, &["S KN0 LHR-0334A73B 1000 16.7 coasting 2.1 0.03 -1 0"]);
	let e = rt.events.back().unwrap();
	assert_eq!((e.text.as_str(), e.level, e.matters), ("no light for 2 s, coasting on the IMU", EvLevel::Warn, true));
	let n = rt.events.len();
	feed(&mut rt, 6.5, &["S KN0 LHR-0334A73B 1000 18.0 coasting 3.4 0.05 -1 0"]);
	assert_eq!(rt.events.len(), n);
	feed(&mut rt, 6.6, &["S KN0 LHR-0334A73B 1000 18.1 tracking 0.004 0.0012 -1 3"]);
	let e = rt.events.back().unwrap();
	assert_eq!(
		(e.text.as_str(), e.level, e.matters),
		("tracking again after 3.4 s without light", EvLevel::Good, true)
	);
}

#[test]
fn the_heat_map_counts_floor_squares() {
	let mut rt = fed(1.0, &[Q0]);
	for _ in 0..3 {
		feed(&mut rt, 1.0, &["S KN0 LHR-0334A73B 1000 12.5 tracking 0.004 0.0012 -1 7"]);
	}
	feed(&mut rt, 1.0, &["S KN0 LHR-0334A73B 1000 12.5 dark 0.4 0.0012 -1 0"]);
	let v = rt.voxels[&(0, 0)];
	assert_eq!((v.n, v.stations_sum, v.dark), (4, 9, 1));
}

#[test]
fn hits_per_station_and_sensor() {
	let mut rt = fed(1.0, &["H KN0 2 1.0 4 0 3 5 0", "H KN0 1 1.0 4 1 0 0 0"]);
	let d = &rt.devices["KN0"];
	assert_eq!(d.sensor_hits, vec![1, 3, 5, 0]);
	assert_eq!((d.light[&2].hits, d.light[&2].sensors_lit), (8, 2));
	assert_eq!(d.lit_sensors(1.5), 3);
	assert_eq!(d.lit_sensors(4.0), 0);
	// a new period forgets the last one
	feed(&mut rt, 2.0, &["H KN0 1 1.0 4 0 0 0 2"]);
	let d = &rt.devices["KN0"];
	assert_eq!(d.sensor_hits, vec![0, 0, 0, 2]);
	assert_eq!(d.light[&2].hits, 0);
	// an absurd sensor count is capped
	feed(&mut rt, 3.0, &["H KN0 1 1.0 100000 1"]);
	assert_eq!(rt.devices["KN0"].sensor_hits.len(), 64);
	// a station index is kept to 0..31 as the stations' are, and a count that would overflow the sums stops at the most
	feed(&mut rt, 4.0, &["H KN0 99 1.0 2 2147483647 5", "H KN0 -4 1.0 2 2147483647 2147483647"]);
	let d = &rt.devices["KN0"];
	assert_eq!(d.light.keys().copied().collect::<Vec<_>>(), [0, 1, 2, 31]);
	assert_eq!((d.light[&31].hits, d.light[&0].hits, d.sensor_hits[0]), (i32::MAX, i32::MAX, i32::MAX));
}

#[test]
fn a_device_lists_sensor_count_is_kept_to_what_an_h_line_takes() {
	let mut rt = Runtime::new();
	rt.command("devices", Want::Devices);
	let id = rt.pending.keys().next().unwrap().clone();
	rt.handle_line(&format!("#{id} DEV KN9 LHR-0000000A 2000000000 tracking 0 tracker 48000000 0 0 0 1 0 0 0"));
	rt.handle_line(&format!("#{id} OK devices 1"));
	assert_eq!(rt.devices["KN9"].sensor_ct, 64);
	// every H period is sized to it: 64, not two billion
	feed(&mut rt, 1.0, &["H KN9 0 1.0 3 1 2 3"]);
	assert_eq!(rt.devices["KN9"].sensor_hits.len(), 64);
}

#[test]
fn lines_without_end_have_ends() {
	let mut rt = fed(1.0, &[Q0]);
	// codenames: 64 at most, the ones known still taken
	for k in 0..100 {
		feed(&mut rt, 1.0, &[&format!("S KX{k} LHR-X 1 1.0 tracking 0 0 -1 1")]);
	}
	assert_eq!(rt.devices.len(), MAX_DEVICES);
	assert!(rt.devices.contains_key("KN0") && !rt.devices.contains_key("KX99"));
	feed(&mut rt, 1.1, &[&Q0.replace(" 12.5 ", " 12.6 ")]);
	assert_eq!(rt.devices["KN0"].run_time, 12.6);
	// C lines with no D line after them
	for _ in 0..1000 {
		feed(&mut rt, 1.2, &["C KN0 12.6 radio 1 6% of reports lost"]);
	}
	assert_eq!(rt.devices["KN0"].diag.causes_next.len(), MAX_CAUSES);
	// a position that is not a number is on no floor square (it was the origin's)
	feed(&mut rt, 1.3, &["Q KN0 LHR-0334A73B 1000 12.7 nan nan 1 1 0 0 0 0 0 0 0 0 0"]);
	feed(&mut rt, 1.3, &["S KN0 LHR-0334A73B 1000 12.7 tracking 0.004 0.0012 -1 3"]);
	assert!(rt.voxels.is_empty());
	// a position run away, a square a line: 65536 at most
	for k in 0..70_000 {
		let q = format!("Q KN0 LHR-0334A73B 1000 13 {} 0 1 1 0 0 0 0 0 0 0 0 0", k as f64);
		feed(&mut rt, 1.4, &[&q, "S KN0 LHR-0334A73B 1000 13 tracking 0.004 0.0012 -1 3"]);
	}
	assert_eq!(rt.voxels.len(), MAX_VOXELS);
}

#[test]
fn a_light_copy_has_all_but_the_history_and_the_trail() {
	let mut rt = fed(1.0, &[Q0, "S KN0 LHR-0334A73B 1000 12.5 tracking 0.004 0.0012 -1 3"]);
	feed(&mut rt, 2.0, &[&Q0.replace(" 12.5 ", " 13.5 ")]);
	let d = rt.devices.get_mut("KN0").unwrap();
	let c = d.copy_light();
	assert!(c.history.samples.is_empty() && c.trail.is_empty());
	assert_eq!(
		(c.codename.as_str(), c.serial.as_str(), c.state.as_str(), c.lh_mask),
		("KN0", "LHR-0334A73B", "tracking", 3)
	);
	assert_eq!((c.pos.x, c.run_time), (d.pos.x, d.run_time));
	// the device keeps its own
	assert_eq!((d.history.samples.len(), d.trail.len()), (2, 2));
}

#[test]
fn stations_from_map_lines() {
	let mut rt = fed(
		1.0,
		&[
			"M 0 439041101 13 1 1 -2.2 -2.0 2.4 1 0 0 0",
			"L 1 8 1.7e9 2.2 -2.0 2.4 1 0 0 0",
			"M 7 1 1 1 1 0 0 0 1 0 0 0",
		],
	);
	assert!(rt.map_locked);
	let s = &rt.stations[&0];
	assert_eq!((s.id, s.mode, s.have_pose), (439041101, 13, true));
	assert_eq!(s.name(), "LHB-1A2B3C4D");
	assert_eq!(s.channel_label(), "ch 14");
	assert_eq!(rt.stations[&1].name(), "LHB-00000008");
	assert!(rt.events.iter().any(|e| e.who == "LHB-00000008" && e.text == "placed in the map"));
	feed(&mut rt, 2.0, &["L 1 8 1.7e9 2.25 -2.0 2.4 1 0 0 0"]);
	assert_eq!(rt.events.back().unwrap().text, "moved 50 mm in the map");
	// an index out of range is kept to the 32 stations a mask has
	feed(&mut rt, 2.0, &["M 99 5 1 1 1 0 0 0 1 0 0 0"]);
	assert!(rt.stations.contains_key(&31));
	feed(&mut rt, 2.0, &["F 0 lost 20.0", "F 0 lost 21.0", "F 0 seen 22.0"]);
	let lost: Vec<&str> = rt.events.iter().filter(|e| e.who == "LHB-1A2B3C4D").map(|e| e.text.as_str()).collect();
	assert_eq!(lost, ["light lost (no device sees it)", "light seen"]);
}

/// D and C lines for KN0 at panel time t
fn diag(rt: &mut Runtime, t: f64, head: &str, sev: i32, causes: &[(&str, i32, &str)]) {
	let d = format!("D KN0 {t:.3} {head} {sev} 4 0.1 40 2 14 0.0 0 0 tracking 0.004 80 {}", causes.len());
	let mut lines = vec![d];
	for (c, s, text) in causes {
		lines.push(format!("C KN0 {t:.3} {c} {s} {text}"));
	}
	let refs: Vec<&str> = lines.iter().map(String::as_str).collect();
	feed(rt, t, &refs);
}

#[test]
fn verdicts_are_logged_once_they_hold() {
	let mut rt = fed(0.0, &[Q0]);
	rt.devices.get_mut("KN0").unwrap().was_active = true;
	rt.devices.get_mut("KN0").unwrap().active = true;
	let radio = [("radio", 1, "6% of reports lost")];
	let mut t = 0.0;
	while t < 1.9 {
		diag(&mut rt, t, "radio", 1, &radio);
		t += 0.5;
	}
	assert!(!rt.events.iter().any(|e| e.text.contains("reports lost")));
	for _ in 0..3 {
		diag(&mut rt, t, "radio", 1, &radio);
		t += 0.5;
	}
	let logged: Vec<&Event> = rt.events.iter().filter(|e| e.text == "6% of reports lost").collect();
	assert_eq!(logged.len(), 1);
	// (a mild one is said without the warning's mark)
	assert_eq!(logged[0].level, EvLevel::Info);
	// the causes of a window are those of the C lines before its D line
	assert_eq!(rt.devices["KN0"].diag.causes, vec![("radio".to_string(), "6% of reports lost".to_string())]);
	// fine for 5 s: "fine again"
	for _ in 0..12 {
		diag(&mut rt, t, "fine", 0, &[]);
		t += 0.5;
	}
	assert_eq!(rt.events.back().unwrap().text, "fine again");
	assert!(rt.devices["KN0"].diag.causes.is_empty());
	// one that hurts the tracking has the warning's mark, and none is more than a warning
	for _ in 0..6 {
		diag(&mut rt, t, "radio", 2, &[("radio", 2, "45% of reports lost")]);
		t += 0.5;
	}
	assert_eq!(rt.events.back().map(|e| (e.text.as_str(), e.level)), Some(("45% of reports lost", EvLevel::Warn)));
	// a light verdict is never an event: the device's own state says a gap in its light
	let n = rt.events.len();
	for (sev, text) in [(1, "only one base station sees it"), (2, "no light for 2.5 s")] {
		for _ in 0..10 {
			diag(&mut rt, t, "light", sev, &[("light", sev, text)]);
			t += 0.5;
		}
	}
	assert_eq!(rt.events.len(), n);
}

#[test]
fn a_device_list_reply_sets_the_devices_and_who_left() {
	let mut rt = Runtime::new();
	rt.command("devices", Want::Devices);
	let id = rt.pending.keys().next().unwrap().clone();
	let r = |rt: &mut Runtime, l: &str| rt.handle_line(&format!("#{id} {l}"));
	r(&mut rt, "DEV KN0 LHR-0334A73B 22 tracking 0.004 tracker_gen2 48000000 0 0 0 1 0 0 0 49 1 D5096F43FE-1YX");
	r(&mut rt, "DEV KN6 LHR-7E87BF59 24 none -1 knuckles_r 48000000 0 0 0 1 0 0 0 -1 0 21879ADDC6-7YX");
	r(&mut rt, "OK devices 2");
	assert!(rt.pending.is_empty());
	let d = &rt.devices["KN0"];
	assert_eq!((d.sensor_ct, d.subtype.as_str(), d.charge, d.charging), (22, "tracker_gen2", 49, true));
	assert_eq!(d.dongle, "D5096F43FE-1YX");
	assert!(rt.devices["KN6"].present);
	assert_eq!(rt.events.iter().filter(|e| e.text.starts_with("device ")).count(), 2);
	rt.command("devices", Want::Devices);
	let id = rt.pending.keys().next().unwrap().clone();
	rt.handle_line(&format!("#{id} DEV KN0 LHR-0334A73B 22 tracking 0.004 tracker_gen2 48000000 0 0 0 1 0 0 0 49 1 -"));
	rt.handle_line(&format!("#{id} OK devices 1"));
	assert!(!rt.devices["KN6"].present);
	assert_eq!(rt.events.back().unwrap().text, "left the runtime's device list");
}

#[test]
fn sensors_come_in_a_reply_nobody_waits_on() {
	let mut rt = fed(0.0, &[Q0]);
	rt.command("sensors KN0", Want::Nothing);
	let id = rt.pending.keys().next().unwrap().clone();
	rt.handle_line(&format!("#{id} SENS KN0 1 0.01 0.02 0.03 0 0 1"));
	rt.handle_line(&format!("#{id} SENS KN0 0 0.04 0.05 0.06 1 0 0"));
	rt.handle_line(&format!("#{id} OK sensors KN0 2"));
	let d = &rt.devices["KN0"];
	assert_eq!(d.sensor_pos.len(), 2);
	assert_eq!(d.sensor_pos[1].z, 0.03);
	assert_eq!(d.sensor_nrm[0].x, 1.0);
}

#[test]
fn hello_sets_the_protocol() {
	let mut rt = Runtime::new();
	rt.command("hello", Want::Hello);
	let id = rt.pending.keys().next().unwrap().clone();
	rt.handle_line(&format!("#{id} OK hello sally-runtime 4 637.9 6 4"));
	assert_eq!(rt.protocol, 4);
	assert_eq!(rt.events.back().unwrap().text, "runtime protocol 4");
}

#[test]
fn malformed_lines_change_nothing() {
	let rt = fed(1.0, &["Q KN0 x", "S", "QQ KN0", "H KN0", "D KN0 1", "C KN0", "M 1", "L 1", "F 1", "#", "#c9 OK"]);
	assert!(rt.devices.is_empty() && rt.stations.is_empty() && rt.events.is_empty());
}

/// A runtime on a UDP port of its own: answers hello and devices, sends a pose to whoever subscribes
fn tiny_runtime() -> (u16, std::thread::JoinHandle<Vec<String>>) {
	let s = UdpSocket::bind("127.0.0.1:0").unwrap();
	let port = s.local_addr().unwrap().port();
	s.set_read_timeout(Some(std::time::Duration::from_millis(100))).unwrap();
	let h = std::thread::spawn(move || {
		let mut got = Vec::new();
		let t0 = std::time::Instant::now();
		let mut buf = [0u8; 2048];
		while t0.elapsed().as_secs_f64() < 2.0 {
			let Ok((n, from)) = s.recv_from(&mut buf) else { continue };
			let line = String::from_utf8_lossy(&buf[..n]).trim().to_string();
			let (id, cmd) = line.split_once(' ').unwrap_or(("", &line));
			got.push(cmd.to_string());
			let reply = |r: &str| s.send_to(format!("{id} {r}\n").as_bytes(), from).unwrap();
			match cmd.split(' ').next().unwrap() {
				"hello" => {
					reply("OK hello sally-runtime 4 5.0 1 0");
				}
				"devices" => {
					reply("DEV KN0 LHR-0334A73B 22 tracking 0.004 tracker_gen2 48000000 0 0 0 1 0 0 0 49 0 -");
					reply("OK devices 1");
				}
				"sub" => {
					reply("OK sub driver state diag");
					s.send_to(format!("{Q0}\n").as_bytes(), from).unwrap();
				}
				_ => {}
			}
		}
		got
	});
	(port, h)
}

#[test]
fn over_a_socket_it_subscribes_and_asks() {
	let (port, h) = tiny_runtime();
	let mut rt = Runtime::new();
	rt.set_target("127.0.0.1", port);
	let t0 = std::time::Instant::now();
	while t0.elapsed().as_secs_f64() < 1.5 {
		rt.poll(t0.elapsed().as_secs_f64());
		std::thread::sleep(std::time::Duration::from_millis(10));
	}
	let got = h.join().unwrap();
	assert!(got.iter().any(|c| c == "sub all"), "{got:?}");
	assert!(got.iter().any(|c| c == "sub driver state diag"), "{got:?}");
	assert!(got.iter().any(|c| c == "hello") && got.iter().any(|c| c == "devices"), "{got:?}");
	assert_eq!(rt.protocol, 4);
	assert!(rt.devices["KN0"].active);
	assert_eq!(rt.devices["KN0"].charge, 49);
	assert!(rt.events.iter().any(|e| e.text.starts_with("connected to 127.0.0.1:")));
}

#[test]
fn the_calibration_status_reads_its_parts() {
	let c = Calib::parse(
		"OK calibrate running session sallytap_20261109_010441.hidtap replaying 0.420 1234 | ready sallytap_20261108_220747.hidtap it 0.061 mrad, the room's 0.065 mrad | last 20261109_011500 pending sallytap_20261108_220747.hidtap 4 stations",
	);
	assert_eq!(c.available, Some(true));
	let r = c.running.as_ref().unwrap();
	assert_eq!((r.kind.as_str(), r.state.as_str(), r.progress, r.moments), ("session", "replaying", 0.42, 1234));
	assert_eq!(r.recording, "sallytap_20261109_010441.hidtap");
	let (rec, why) = c.ready.as_ref().unwrap();
	assert_eq!(rec, "sallytap_20261108_220747.hidtap");
	assert_eq!(why, "it 0.061 mrad, the room's 0.065 mrad");
	assert_eq!(c.pending, None);
	assert!(c.last.as_ref().unwrap().starts_with("20261109_011500 pending"));
	assert_eq!(c.failed, None);
	// a room that waits for its calibration, after one of this session that failed
	let c = Calib::parse(
		"OK calibrate newroom 79 | failed only 9 still moments were seen by two placed stations: too few | last 20261119_230915 failed sallytap_20261119_230704.hidtapz only 9 still moments were seen by two placed stations: too few",
	);
	assert_eq!(c.new_room, Some(79.0));
	assert_eq!(c.failed.as_deref(), Some("only 9 still moments were seen by two placed stations: too few"));
	assert_eq!(Calib::parse("OK calibrate idle"), Calib { available: Some(true), ..Calib::default() });
	let replay = Calib::parse("ERR calibrate needs the live runtime (the dongles, a config file and --hidtap-record)");
	assert_eq!(replay.available, Some(false));
}
