//! The panel without a display (Linux): runs it against a runtime (or the fake one) and writes screenshots, for tests
//! and for a look at what changed.
//!
//!   headless [--port P | --fake [--full] [--drop KN2@12]] [--size 1600x940] [--seconds 12] [--shot out.png@10]...
//!            [--overlay-shot ov.png@10] [--thumb-shot thumb.png@5] [--select KN0] [--select-station 1]
//!            [--page KN0@3] [--tab 3@2] [--click x,y@t] [--overlay-click x,y@t] [--type text@t] [--key enter@t]
//!            [--hover x,y@t] [--drag x,y:x,y@t]
//!            [--cfg panel.cfg [--save]] [--fake-vr [--keyboard text]]
//!            [--fake-driver [--output scope.name=value]... [--virtual on|off|none]]
//!            [--project dir] [--steamvr dir] [--replay key@t] [--replay-seek secs[:device[:metres]]@t]
//!            [--replay-view from:to@t] [--replay-focus track|-:station|-@t] [--replay-cam yaw:pitch@t]
//!
//! `--full` gives the fake runtime a whole body's set of twelve devices. The overlay is the same panel through a second
//! egui context at 1.25 pixels a point (2000 x 1175 for the 1600 x 940 layout), as the window host draws it. Clicks are
//! a press and, a frame later, a release. `--fake-vr` is SteamVR with Valve's driver seeing what the runtime sees (in
//! another frame, with noise: valve::fake), its overlay up, its keyboard answering with `--keyboard`'s text.
//! `--fake-driver` is the sally driver's side (valve::fake_driver): SteamVR shown sally's tracking, the output layer's
//! settings (`--output` sets one before the start), the virtual trackers. `--replay` opens a session in the Replay tab
//! (`--project`: the project folder, `--steamvr`: where the render models are); the script's clock stops while it
//! replays and loads, so the times after it are from when it is there.

#[cfg(target_os = "linux")]
fn main() {
	linux::main();
}

#[cfg(not(target_os = "linux"))]
fn main() {
	eprintln!("the headless panel is Linux only (EGL's software device)");
	std::process::exit(2);
}

#[cfg(target_os = "linux")]
mod linux {
	use std::path::PathBuf;
	use std::time::Instant;

	use egui::{Event, Key, Modifiers, PointerButton, Pos2};
	use sally_panel::fake_runtime::FakeRuntime;
	use sally_panel::headless::Headless;
	use sally_panel::offscreen::{save_png, Offscreen};
	use sally_panel::settings::Settings;
	use sally_panel::ui::{Panel, View};
	use sally_panel::valve::fake::FakeVr;
	use sally_panel::valve::fake_driver::FakeDriver;
	use sally_panel::valve::ValveCompare;

	enum Input {
		Click(f32, f32, bool),
		Hover(f32, f32),
		Drag(f32, f32, f32, f32),
		Text(String),
		Key(Key),
		Tab(i32),
		Page(String),
		Replay(String),
		Seek(f64, Option<usize>, Option<f64>),
		View(f64, f64),
		Focus(Option<usize>, Option<i32>),
		Cam(f64, f64),
	}

	fn at(v: &str) -> (String, f64) {
		match v.rsplit_once('@') {
			Some((a, t)) => (a.to_string(), t.parse().unwrap_or(0.0)),
			None => (v.to_string(), 0.0),
		}
	}

	fn xy(v: &str) -> (f32, f32) {
		let (x, y) = v.split_once(',').unwrap_or(("0", "0"));
		(x.parse().unwrap_or(0.0), y.parse().unwrap_or(0.0))
	}

	pub fn main() {
		let mut args = std::env::args().skip(1);
		let (mut port, mut fake, mut full, mut drops) = (9100u16, false, false, Vec::new());
		let (mut w, mut h, mut seconds) = (1600u32, 940u32, 12.0f64);
		let (mut shots, mut ov_shots, mut thumb_shots) = (Vec::new(), Vec::new(), Vec::new());
		let mut inputs: Vec<(f64, Input)> = Vec::new();
		let (mut cfg, mut save) = (None::<PathBuf>, false);
		let (mut select, mut select_station) = (String::new(), String::new());
		let (mut fake_vr, mut keyboard) = (false, String::from("Spare"));
		let (mut project, mut steamvr_dir) = (None::<String>, None::<String>);
		let (mut fake_driver, mut outputs, mut virtual_on) = (false, Vec::<String>::new(), String::from("off"));
		while let Some(a) = args.next() {
			let mut next = || args.next().unwrap_or_default();
			match a.as_str() {
				"--port" => port = next().parse().unwrap_or(9100),
				"--fake" => fake = true,
				"--full" => full = true,
				"--drop" => drops.push(at(&next())),
				"--size" => {
					let v = next();
					let (a, b) = v.split_once('x').unwrap_or(("1600", "940"));
					(w, h) = (a.parse().unwrap_or(1600), b.parse().unwrap_or(940));
				}
				"--seconds" => seconds = next().parse().unwrap_or(12.0),
				"--shot" => shots.push(at(&next())),
				"--overlay-shot" => ov_shots.push(at(&next())),
				"--thumb-shot" => thumb_shots.push(at(&next())),
				"--select" => select = next(),
				"--select-station" => select_station = next(),
				"--page" => {
					let (c, t) = at(&next());
					inputs.push((t, Input::Page(c)));
				}
				"--tab" => {
					let (c, t) = at(&next());
					inputs.push((t, Input::Tab(c.parse().unwrap_or(0))));
				}
				"--click" | "--overlay-click" => {
					let (c, t) = at(&next());
					let (x, y) = xy(&c);
					inputs.push((t, Input::Click(x, y, a == "--overlay-click")));
				}
				"--hover" => {
					let (c, t) = at(&next());
					let (x, y) = xy(&c);
					inputs.push((t, Input::Hover(x, y)));
				}
				"--drag" => {
					let (c, t) = at(&next());
					let (a, b) = c.split_once(':').unwrap_or(("0,0", "0,0"));
					let ((x0, y0), (x1, y1)) = (xy(a), xy(b));
					inputs.push((t, Input::Drag(x0, y0, x1, y1)));
				}
				"--fake-driver" => fake_driver = true,
				"--output" => outputs.push(next()),
				"--virtual" => virtual_on = next(),
				"--type" => {
					let (c, t) = at(&next());
					inputs.push((t, Input::Text(c)));
				}
				"--key" => {
					let (c, t) = at(&next());
					let k = match c.as_str() {
						"enter" => Key::Enter,
						"escape" => Key::Escape,
						"backspace" => Key::Backspace,
						_ => Key::Space,
					};
					inputs.push((t, Input::Key(k)));
				}
				"--cfg" => cfg = Some(PathBuf::from(next())),
				"--save" => save = true,
				"--fake-vr" => fake_vr = true,
				"--keyboard" => keyboard = next(),
				"--project" => project = Some(next()),
				"--steamvr" => steamvr_dir = Some(next()),
				"--replay" => {
					let (c, t) = at(&next());
					inputs.push((t, Input::Replay(c)));
				}
				"--replay-focus" => {
					let (c, t) = at(&next());
					let (a, b) = c.split_once(':').unwrap_or(("-", "-"));
					inputs.push((t, Input::Focus(a.parse().ok(), b.parse().ok())));
				}
				"--replay-view" => {
					let (c, t) = at(&next());
					let (a, b) = c.split_once(':').unwrap_or(("0", "0"));
					inputs.push((t, Input::View(a.parse().unwrap_or(0.0), b.parse().unwrap_or(0.0))));
				}
				"--replay-cam" => {
					let (c, t) = at(&next());
					let (a, b) = c.split_once(':').unwrap_or(("0", "0"));
					inputs.push((t, Input::Cam(a.parse().unwrap_or(0.0), b.parse().unwrap_or(0.0))));
				}
				"--replay-seek" => {
					let (c, t) = at(&next());
					let mut w = c.split(':');
					let secs = w.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
					let dev = w.next().and_then(|v| v.parse().ok());
					let dist = w.next().and_then(|v| v.parse().ok());
					inputs.push((t, Input::Seek(secs, dev, dist)));
				}
				_ => {
					eprintln!("unknown option {a}");
					std::process::exit(2);
				}
			}
		}
		inputs.sort_by(|a, b| a.0.total_cmp(&b.0));
		let _fake = if fake {
			let f = FakeRuntime::start_with(0, drops, full).expect("fake runtime");
			port = f.port;
			Some(f)
		} else {
			None
		};
		let mut st = match &cfg {
			Some(p) => Settings::load(p),
			None => Settings::default(),
		};
		st.port = port;
		st.host = "127.0.0.1".into();
		if let Some(p) = project {
			st.replay_project = p;
		}
		if let Some(d) = steamvr_dir {
			st.steamvr = d;
		}
		if st.path.as_os_str().is_empty() {
			st.path = std::env::current_dir().unwrap_or_default().join("sally-panel.cfg");
		}
		let (vc, steamvr) = if fake_vr {
			let (vr, steamvr) = FakeVr::new();
			{
				let mut s = steamvr.lock();
				s.overlay_visible = true;
				s.keyboard_text = keyboard;
			}
			(ValveCompare::with_vr(Box::new(vr)), Some(steamvr))
		} else {
			(ValveCompare::new(), None)
		};
		let mut vc = vc;
		let mut driver = fake_driver.then(|| {
			let mut d = FakeDriver::new(&mut vc).expect("fake driver");
			for o in &outputs {
				// scope.name=value
				if let Some(((scope, name), v)) = o.split_once('=').and_then(|(k, v)| Some((k.split_once('.')?, v))) {
					if let Ok(v) = v.parse::<f64>() {
						d.output.insert((scope.to_string(), name.to_string()), v);
					}
				}
			}
			d.virtual_on = match virtual_on.as_str() {
				"on" => Some(true),
				"none" => None,
				_ => Some(false),
			};
			d
		});
		let mut panel = Panel::new(st, vc);
		panel.select_device = select;
		panel.select_station = select_station;

		let gl = Headless::new().unwrap_or_else(|e| {
			eprintln!("no headless GL: {e}");
			std::process::exit(3);
		});
		let mut win = Offscreen::new(gl.gl.clone(), w, h, 1.0).expect("window view");
		Panel::setup(&win.ctx);
		let mut ov = Offscreen::new(gl.gl.clone(), 2000, 1175, 1.25).expect("overlay view");
		Panel::setup(&ov.ctx);
		let mut thumb = Offscreen::new(gl.gl.clone(), 256, 256, 1.0).expect("button view");
		thumb.mirror = true;
		Panel::setup(&thumb.ctx);

		let t0 = Instant::now();
		let (mut win_ev, mut ov_ev): (Vec<Event>, Vec<Event>) = (Vec::new(), Vec::new());
		let (mut win_release, mut ov_release) = (None::<Pos2>, None::<Pos2>);
		let first_ov = ov_shots
			.iter()
			.map(|s| s.1)
			.chain(inputs.iter().filter(|i| matches!(i.1, Input::Click(_, _, true))).map(|i| i.0))
			.fold(f64::INFINITY, f64::min);
		// the window's frame times (ms), for a look at what a view costs
		let mut frame_ms: Vec<[f64; 3]> = Vec::new();
		// the script's clock: stopped while a replay runs and loads
		let (mut held, mut last, mut waiting) = (0.0f64, 0.0f64, false);
		loop {
			let real = t0.elapsed().as_secs_f64();
			if waiting {
				if panel.replay_busy() {
					held += real - last;
				} else {
					waiting = false;
				}
			}
			last = real;
			let now = real - held;
			if now > seconds {
				break;
			}
			if let Some(s) = &steamvr {
				s.lock().follow(&panel.rt);
			}
			if let Some(d) = &mut driver {
				d.poll(real, &panel.rt);
			}
			panel.poll(real);
			if let Some(p) = win_release.take() {
				win_ev.push(Event::PointerButton {
					pos: p,
					button: PointerButton::Primary,
					pressed: false,
					modifiers: Modifiers::NONE,
				});
			}
			if let Some(p) = ov_release.take() {
				ov_ev.push(Event::PointerButton {
					pos: p,
					button: PointerButton::Primary,
					pressed: false,
					modifiers: Modifiers::NONE,
				});
			}
			while inputs.first().is_some_and(|i| i.0 <= now) {
				let (_, i) = inputs.remove(0);
				match i {
					Input::Click(x, y, overlay) => {
						// the overlay's clicks are in its texture's pixels, as SteamVR's laser gives them
						let p = if overlay { Pos2::new(x / ov.ppp, y / ov.ppp) } else { Pos2::new(x, y) };
						let evs = if overlay { &mut ov_ev } else { &mut win_ev };
						evs.push(Event::PointerMoved(p));
						evs.push(Event::PointerButton {
							pos: p,
							button: PointerButton::Primary,
							pressed: true,
							modifiers: Modifiers::NONE,
						});
						if overlay {
							ov_release = Some(p);
						} else {
							win_release = Some(p);
						}
					}
					Input::Hover(x, y) => win_ev.push(Event::PointerMoved(Pos2::new(x, y))),
					Input::Drag(x0, y0, x1, y1) => {
						// a press, the way there in steps, and the release a frame later
						win_ev.push(Event::PointerMoved(Pos2::new(x0, y0)));
						win_ev.push(Event::PointerButton {
							pos: Pos2::new(x0, y0),
							button: PointerButton::Primary,
							pressed: true,
							modifiers: Modifiers::NONE,
						});
						for k in 1..=8 {
							let f = k as f32 / 8.0;
							win_ev.push(Event::PointerMoved(Pos2::new(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f)));
						}
						win_release = Some(Pos2::new(x1, y1));
					}
					Input::Text(t) => win_ev.push(Event::Text(t)),
					Input::Key(k) => {
						for pressed in [true, false] {
							win_ev.push(Event::Key {
								key: k,
								physical_key: None,
								pressed,
								repeat: false,
								modifiers: Modifiers::NONE,
							});
						}
					}
					Input::Tab(t) => panel.st.tab = t,
					Input::Page(c) => panel.open_page = c,
					Input::Replay(stamp) => {
						panel.st.tab = 1;
						panel.replay_open(&stamp, false);
						waiting = true;
					}
					Input::Seek(t, dev, dist) => panel.replay_seek(t, dev, dist),
					Input::View(a, b) => panel.replay_view(a, b),
					Input::Focus(a, b) => panel.replay_focus(a, b),
					Input::Cam(a, b) => panel.replay_cam(a, b),
				}
			}
			win.frame(real, std::mem::take(&mut win_ev), |ui| panel.frame(ui, real, View::Window));
			frame_ms.push(win.last_ms);
			if now + 1.0 >= first_ov {
				ov.frame(real, std::mem::take(&mut ov_ev), |ui| panel.frame(ui, real, View::Overlay));
			}
			let due = |list: &mut Vec<(String, f64)>, v: &Offscreen| {
				list.retain(|(path, t)| {
					if *t > now {
						return true;
					}
					match save_png(&v.read(), std::path::Path::new(path)) {
						Ok(()) => println!("{path} at {now:.1} s"),
						Err(e) => eprintln!("{path}: {e}"),
					}
					false
				});
			};
			due(&mut shots, &win);
			due(&mut ov_shots, &ov);
			if thumb_shots.iter().any(|s| s.1 <= now) {
				thumb.frame(real, Vec::new(), |ui| panel.thumbnail(ui, real));
				due(&mut thumb_shots, &thumb);
			}
			std::thread::sleep(std::time::Duration::from_millis(5));
		}
		if save {
			panel.st.save();
		}
		if !frame_ms.is_empty() {
			let stat = |k: usize| {
				let mut v: Vec<f64> = frame_ms.iter().map(|f| f[k]).collect();
				v.sort_by(f64::total_cmp);
				format!("{:.1} / {:.1} ms", v[v.len() / 2], v[v.len() * 9 / 10])
			};
			println!(
				"window frames {} (median / 90 %): the panel's UI {}, tessellation {}, painting {} (the software GL's)",
				frame_ms.len(),
				stat(0),
				stat(1),
				stat(2)
			);
		}
		let d = &panel.rt;
		println!(
			"connected {} protocol {} devices {} active {} stations {} events {}",
			d.connected(t0.elapsed().as_secs_f64()),
			d.protocol,
			d.devices.len(),
			d.active().count(),
			d.stations.len(),
			d.events.len()
		);
		win.destroy();
		ov.destroy();
		thumb.destroy();
	}
}
