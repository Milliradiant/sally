//! sally-panel, the program (docs/sally_panel.md): the panel's desktop window, and the same panel drawn a second time
//! into a texture for the SteamVR dashboard overlay. On winit (the window and its input), glutin through glutin-winit
//! (WGL on Windows; GLX, else EGL, on Linux), egui-winit and egui_glow.
//!
//!   sally-panel [--host 127.0.0.1] [--port 9100]
//!   sally-panel --replay-check <stamp | check_<stamp> | latest> [--again]
//!
//! The settings are sally-panel.cfg beside the program; --host and --port override them for the run (the file keeps its
//! own). One GL context, the window's, carries every view: the window through its own painter, and the overlay through
//! two `Offscreen` views, the panel at 2000 x 1175 pixels (the 1600 x 940 point layout at 1.25 pixels a point, sharper
//! in the headset) and the dashboard button's 256 x 256 picture, drawn upside down as the dashboard shows it. Each view
//! is an egui context of its own; the panel and its state are one.
//!
//! The loop takes a turn whenever winit has handled what came in:
//! - the runtime's socket and Valve's side are polled, every turn whatever else happens;
//! - the overlay switched on or off in Settings is wanted only when its views could be made;
//! - the dashboard button's picture is drawn when an overlay was made, then once a second while the dashboard is up on
//!   another tab;
//! - the overlay is drawn only while SteamVR shows it, at up to 90 Hz, also with the window minimised: the laser's
//!   events through `overlay_input`, the panel into the texture, the texture handed over;
//! - the window is drawn with vsync; while the overlay is up and the window is in the background, at 30 Hz without
//!   vsync, so the monitor does not hold the overlay back; minimised (or covered) it is not drawn and the loop waits
//!   15 ms, or only until the overlay's next frame while that is up; in the background with nothing in VR it rests
//!   8 ms after each frame;
//! - the settings are written every 2 s while they change.
//!
//! At the end the settings are written, OpenVR lets go of the overlay while the GL context its textures live in is
//! still current, then the views go, then the context, then the window.
//!
//! `--replay-check` runs the Replay tab's work without a window: the session replayed (unless it has been, or
//! `--again`) and loaded, what it found written to replays\<stamp>\check.txt, the exit code 0 when it all worked.
//!
//! For the tests only, `SALLY_PANEL_FAKE_VR=1` puts valve's fake SteamVR where openvr_api would be and plays a
//! dashboard session on it (`FakeSession`).

#![cfg_attr(windows, windows_subsystem = "windows")]

use std::collections::VecDeque;
use std::ffi::OsString;
use std::num::NonZeroU32;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use egui::ViewportId;
use glutin::config::{ConfigTemplateBuilder, GlConfig};
use glutin::context::{
	ContextApi, ContextAttributesBuilder, NotCurrentGlContext, PossiblyCurrentContext, PossiblyCurrentGlContext,
};
use glutin::display::{GetGlDisplay, GlDisplay};
use glutin::surface::{GlSurface, Surface, SurfaceAttributesBuilder, SwapInterval, WindowSurface};
use glutin_winit::{ApiPreference, DisplayBuilder, GlWindow};
use raw_window_handle::HasWindowHandle;
use sally_panel::icon::{self, Dot};
use sally_panel::offscreen::{save_png, Offscreen};
use sally_panel::openvr::{
	ColorSpace_Gamma, EVREventType, TextureType_OpenGL, VREvent_FocusLeave, VREvent_MouseButtonDown,
	VREvent_MouseButtonUp, VREvent_MouseMove, VREvent_ScrollSmooth, VRMouseButton_Left, VrEvent,
};
use sally_panel::overlay_input::OverlayInput;
use sally_panel::runtime::Runtime;
use sally_panel::settings::Settings;
use sally_panel::ui::{theme, Panel, View};
use sally_panel::valve::fake::{self, FakeVr, SteamVr};
use sally_panel::valve::ValveCompare;
use winit::application::ApplicationHandler;
use winit::dpi::{LogicalSize, PhysicalPosition, PhysicalSize};
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::monitor::MonitorHandle;
use winit::window::{Icon, Window, WindowId};

/// The panel's layout (points): the window opens at it times the monitor's scale, and the overlay draws it
const LAYOUT: (f64, f64) = (1600.0, 940.0);
/// The smallest the window may be made (points)
const SMALLEST: (f64, f64) = (1100.0, 700.0);
/// Left free around the window in the work area when the layout does not fit (pixels), main_win32.cpp's
const MARGIN: f64 = 40.0;
/// The overlay's texture: the layout at 1.25 pixels a point
const OVERLAY_PPP: f32 = 1.25;
const OVERLAY_SIZE: (u32, u32) = (2000, 1175);
/// The dashboard button's picture (pixels a side)
const BUTTON_SIZE: u32 = 256;
/// The overlay's frames a second at most, and the window's while it is behind the overlay
const OVERLAY_HZ: f64 = 90.0;
const BEHIND_HZ: f64 = 30.0;
/// How often the button's picture is drawn again while the dashboard is up on another tab (s)
const BUTTON_EVERY: f64 = 1.0;
/// The loop's rest while the window is minimised, and in the background after a frame with nothing in VR (s)
const HIDDEN_REST: f64 = 0.015;
const BACKGROUND_REST: f64 = 0.008;
/// The settings are written at most this often while they change (s)
const SAVE_EVERY: f64 = 2.0;

fn main() {
	#[cfg(windows)]
	{
		// nobody sees a GUI program's stderr: a panic says what it was in a message box before the program goes
		let default = std::panic::take_hook();
		std::panic::set_hook(Box::new(move |info| {
			default(info);
			win::message_box(&format!("sally panel stopped: {info}"));
		}));
	}
	let args = Args::parse(std::env::args_os().skip(1));
	let mut st = Settings::load_dir(&exe_dir());
	if let Some(stamp) = &args.replay_check {
		std::process::exit(replay_check(&st, stamp, args.again));
	}
	// one panel at a time: a second one brings the first's window up and goes (one started with SteamVR just goes).
	// Not for a panel given a runtime of its own to watch, nor the tests'
	#[cfg(windows)]
	if args.host.is_none()
		&& args.port.is_none()
		&& std::env::var_os("SALLY_PANEL_FAKE_VR").is_none()
		&& !win::only_panel()
	{
		if args.steamvr.is_none() {
			win::show_running();
		}
		return;
	}
	if args.host.is_some() || args.port.is_some() {
		// for this run only: the file keeps its own
		st.file_target = Some((st.host.clone(), st.port));
	}
	if let Some(host) = args.host {
		st.host = host;
	}
	if let Some(port) = args.port {
		st.port = port;
	}
	let (vc, session) = if std::env::var_os("SALLY_PANEL_FAKE_VR").is_some_and(|v| v == "1") {
		let (vr, steamvr) = FakeVr::new();
		(ValveCompare::with_vr(Box::new(vr)), Some(FakeSession::new(steamvr)))
	} else {
		(ValveCompare::new(), None)
	};
	let mut panel = Panel::new(st, vc);
	// the texture's size is the laser's scale, which the overlay is made with
	panel.vc.overlay_w = OVERLAY_SIZE.0 as i32;
	panel.vc.overlay_h = OVERLAY_SIZE.1 as i32;
	let event_loop = match EventLoop::new() {
		Ok(el) => el,
		Err(e) => {
			tell(&format!("No window system: {e}"));
			std::process::exit(1);
		}
	};
	let mut host = Host {
		panel,
		start: Instant::now(),
		shown: None,
		error: None,
		session,
		next_turn: 0.0,
		last_save: 0.0,
		minimised: args.steamvr.is_some(),
		#[cfg(windows)]
		steamvr: args.steamvr.and_then(win::Process::open),
		#[cfg(windows)]
		next_look: 0.0,
	};
	if let Err(e) = event_loop.run_app(&mut host) {
		host.error.get_or_insert(e.to_string());
	}
	if let Some(e) = host.error {
		tell(&e);
		std::process::exit(1);
	}
}

/// The folder the program is in, where its settings are
fn exe_dir() -> PathBuf {
	std::env::current_exe().ok().and_then(|p| p.parent().map(Path::to_path_buf)).unwrap_or_else(|| ".".into())
}

/// Says why the panel cannot run: a message box on Windows (there is no console), stderr elsewhere
fn tell(text: &str) {
	#[cfg(windows)]
	win::message_box(text);
	#[cfg(not(windows))]
	eprintln!("sally-panel: {text}");
}

/// The command line: --host and --port, as main_win32.cpp took them; --replay-check and --again
#[derive(Debug, Default, PartialEq)]
struct Args {
	host: Option<String>,
	port: Option<u16>,
	replay_check: Option<String>,
	again: bool,
	/// started by the sally driver with SteamVR (`--steamvr <SteamVR's process>`): the window opens minimised, and
	/// the panel goes when that process has
	steamvr: Option<u32>,
}

impl Args {
	fn parse(args: impl IntoIterator<Item = OsString>) -> Args {
		let mut out = Args::default();
		let mut it = args.into_iter().map(|a| a.to_string_lossy().into_owned());
		while let Some(a) = it.next() {
			// a flag takes the argument after it; at the end of the line it is ignored, as the C++ ignored it
			match a.as_str() {
				"--host" => {
					if let Some(v) = it.next() {
						out.host = Some(v);
					}
				}
				"--port" => {
					// as the C++ read it (_wtoi) and as sally-panel.cfg's port is read: the leading digits
					if let Some(v) = it.next() {
						out.port = Some(sally_math::c_atoi(v.as_bytes()).clamp(0, 65535) as u16);
					}
				}
				"--replay-check" => out.replay_check = it.next(),
				"--again" => out.again = true,
				"--steamvr" => out.steamvr = it.next().and_then(|v| v.parse().ok()),
				// nothing else is the panel's (and there is no console to say so on)
				_ => {}
			}
		}
		out
	}
}

/// The Replay tab's work without a window (`--replay-check`): replays and loads a session ("latest": the newest), and
/// writes what it found to check.txt in its replay folder (and stdout); 0 when the replay ran and loaded with devices
fn replay_check(st: &Settings, stamp: &str, again: bool) -> i32 {
	use sally_panel::replay;
	use std::fmt::Write as _;
	let t0 = Instant::now();
	let panel_dir = st.path.parent().map(Path::to_path_buf).unwrap_or_else(exe_dir);
	let paths = replay::Paths::discover(&panel_dir, &st.replay_project);
	let mut out = String::new();
	let _ = writeln!(out, "panel {} (sally-panel {})", panel_dir.display(), env!("CARGO_PKG_VERSION"));
	let _ = writeln!(out, "project {}\nlogs {}", paths.project.display(), paths.logs.display());
	let _ = writeln!(out, "runtime {} ({} live arguments)", paths.runtime_exe.display(), paths.runtime_args.len());
	let _ = writeln!(
		out,
		"device configs {}",
		paths.config_dirs().iter().map(|d| d.display().to_string()).collect::<Vec<_>>().join(" ; ")
	);
	let sessions = replay::sessions(&paths);
	let _ = writeln!(out, "{} sessions", sessions.len());
	let s = if stamp == "latest" { sessions.first() } else { sessions.iter().find(|s| s.key() == stamp) };
	let Some(s) = s.cloned() else {
		let _ = writeln!(out, "no session {stamp}");
		print!("{out}");
		return 2;
	};
	let dir = s.replay_dir(&paths);
	let _ = writeln!(
		out,
		"session {} {:.1} s, tap {} MB, pose log {}, runtime log {}",
		s.stamp,
		s.duration(),
		s.bytes / 1_000_000,
		s.poselog.as_ref().map_or("none".into(), |p| p.display().to_string()),
		s.solver.as_ref().map_or("none".into(), |p| p.display().to_string())
	);
	let done = s.replayed.as_deref().is_some_and(|d| d.starts_with("exit 0"));
	let mut code = 0;
	if !done || again {
		match replay::start(&paths, &s) {
			Ok(mut job) => loop {
				match job.poll() {
					Ok(false) => std::thread::sleep(Duration::from_millis(250)),
					Ok(true) => {
						let _ = writeln!(out, "replayed in {:.1} s", job.started.elapsed().as_secs_f64());
						break;
					}
					Err(e) => {
						let _ = writeln!(out, "replay failed: {e}");
						code = 1;
						break;
					}
				}
			},
			Err(e) => {
				let _ = writeln!(out, "replay not started: {e}");
				code = 1;
			}
		}
	} else {
		let _ = writeln!(out, "replayed before: {}", s.replayed.as_deref().unwrap_or(""));
	}
	if code == 0 {
		let t1 = Instant::now();
		match replay::load(&dir, &s, None) {
			Ok(d) => {
				let _ = writeln!(
					out,
					"loaded in {:.1} s: {} devices, {} stations, {} events, live poses {}",
					t1.elapsed().as_secs_f64(),
					d.devs.len(),
					d.stations.len(),
					d.events.len(),
					d.have_live
				);
				for n in &d.notes {
					let _ = writeln!(out, "note: {n}");
				}
				let mut models = sally_panel::models::Models::default();
				models.set_steamvr(&st.steamvr);
				for x in &d.devs {
					let m =
						models.get(&x.model).map_or("not found".to_string(), |m| format!("{} triangles", m.tris.len()));
					let st = |k: usize| {
						let (a, b, c, n) = x.stats[k];
						if a < 0.0 {
							"-".to_string()
						} else {
							format!("{:.1} / {:.1} / {:.0} mm, {n} over 3 cm", a * 1e3, b * 1e3, c * 1e3)
						}
					};
					let head = replay::angle_deg(Default::default(), x.c.q);
					let _ = writeln!(
						out,
						"{} {} {} ({}): model {m}, {} replay poses, {} live; head offset {head:.1} deg; \
						 from Valve: live {}, replay {}",
						x.codename,
						x.serial,
						x.kind,
						x.model,
						x.replay.len(),
						x.live.len(),
						st(0),
						st(1)
					);
				}
				if d.devs.is_empty() {
					code = 1;
				}
			}
			Err(e) => {
				let _ = writeln!(out, "load failed: {e}");
				code = 1;
			}
		}
	}
	let _ = writeln!(out, "check done in {:.1} s, exit {code}", t0.elapsed().as_secs_f64());
	let _ = std::fs::create_dir_all(&dir);
	let _ = std::fs::write(dir.join("check.txt"), &out);
	print!("{out}");
	code
}

/// A part of the screen in pixels: a monitor's work area
#[derive(Clone, Copy, Debug, PartialEq)]
struct Area {
	x: i32,
	y: i32,
	w: u32,
	h: u32,
}

/// Where the window opens in a monitor's work area: the layout at the monitor's scale, no closer than the margin to the
/// area's edges, centred; and the smallest it may be made, no larger than that
fn placement(scale: f64, work: Area) -> (PhysicalSize<u32>, PhysicalPosition<i32>, LogicalSize<f64>) {
	let w = (LAYOUT.0 * scale).round().min(work.w as f64 - MARGIN).max(1.0);
	let h = (LAYOUT.1 * scale).round().min(work.h as f64 - MARGIN).max(1.0);
	let x = work.x + ((work.w as f64 - w) / 2.0) as i32;
	let y = work.y + ((work.h as f64 - h) / 2.0) as i32;
	let smallest = LogicalSize::new(SMALLEST.0.min(w / scale), SMALLEST.1.min(h / scale));
	(PhysicalSize::new(w as u32, h as u32), PhysicalPosition::new(x, y), smallest)
}

/// Once the window exists, hidden: made smaller if its frame does not fit in the work area too, and centred there frame
/// and all (the frame's size is known only now)
fn centre(window: &Window, work: Area) {
	let (inner, outer) = (window.inner_size(), window.outer_size());
	let frame = (outer.width.saturating_sub(inner.width), outer.height.saturating_sub(inner.height));
	let room = (work.w.saturating_sub(frame.0), work.h.saturating_sub(frame.1));
	if inner.width > room.0 || inner.height > room.1 {
		let _ = window.request_inner_size(PhysicalSize::new(inner.width.min(room.0), inner.height.min(room.1)));
	}
	let outer = window.outer_size();
	let x = work.x + (work.w as i32 - outer.width as i32).max(0) / 2;
	let y = work.y + (work.h as i32 - outer.height as i32).max(0) / 2;
	window.set_outer_position(PhysicalPosition::new(x, y));
}

/// The whole monitor, where there is no work area to be had
fn whole(monitor: &MonitorHandle) -> Area {
	let (p, s) = (monitor.position(), monitor.size());
	Area { x: p.x, y: p.y, w: s.width, h: s.height }
}

/// The monitor less the taskbar
#[cfg(windows)]
fn work_area(monitor: &MonitorHandle) -> Area {
	win::work_area(monitor).unwrap_or_else(|| whole(monitor))
}

/// The monitor (winit has no work area on X11)
#[cfg(not(windows))]
fn work_area(monitor: &MonitorHandle) -> Area {
	whole(monitor)
}

/// The next time something done every `period` is due: a period after it was last due, so its rate holds when a turn
/// comes a little late; a period from now when the turn came more than a period late (no burst to catch up)
fn next_due(due: f64, period: f64, now: f64) -> f64 {
	if due + period > now {
		due + period
	} else {
		now + period
	}
}

/// The program: the panel, and its window once winit lets it be made
struct Host {
	panel: Panel,
	/// the host's clock (`now`) is the seconds since
	start: Instant,
	shown: Option<Shown>,
	/// why the window could not be made
	error: Option<String>,
	/// the tests' fake SteamVR session (SALLY_PANEL_FAKE_VR=1)
	session: Option<FakeSession>,
	/// when the loop's next turn is due
	next_turn: f64,
	last_save: f64,
	/// the window opens minimised (started with SteamVR)
	minimised: bool,
	/// the SteamVR the panel was started with, and when it is next looked at: the panel goes when it has
	#[cfg(windows)]
	steamvr: Option<win::Process>,
	#[cfg(windows)]
	next_look: f64,
}

impl Host {
	fn now(&self) -> f64 {
		self.start.elapsed().as_secs_f64()
	}

	/// One turn of the loop; when the next is due
	fn turn(&mut self, now: f64) -> f64 {
		let Host { panel, start, shown, session, last_save, .. } = self;
		let Some(shown) = shown.as_mut() else { return now + HIDDEN_REST };
		// the overlay switched on or off in Settings: wanted only where its views could be made
		if panel.toggle_overlay {
			panel.toggle_overlay = false;
			panel.vc.overlay_wanted = panel.st.overlay && shown.overlay.is_some();
		}
		if let Some(s) = session.as_mut() {
			s.play(now, &panel.rt);
		}
		panel.poll(now);
		// the icon's dot says what sally is doing, the window minimised or not
		let dot = panel.dot(now);
		if shown.dot != Some(dot) {
			shown.set_icon(dot);
		}
		if shown.overlay.is_none() {
			panel.vc.overlay_status = "no GL framebuffers for the overlay".into();
		}
		if let Some(s) = session.as_mut() {
			s.waiting(start.elapsed().as_secs_f64(), &panel.vc);
		}
		let overlay_up = shown.overlay.as_mut().is_some_and(|o| o.turn(panel, now));
		let overlay_next = shown.overlay.as_ref().map_or(f64::INFINITY, |o| o.next_frame);
		let mut drawn = false;
		let next = if shown.hidden() {
			// nothing to see: the overlay goes on at its own pace
			if overlay_up {
				overlay_next.max(now + 0.001)
			} else {
				now + HIDDEN_REST
			}
		} else {
			let behind = overlay_up && !shown.focused;
			if behind && now < shown.next_behind {
				shown.next_behind.min(overlay_next)
			} else {
				if behind {
					shown.next_behind = next_due(shown.next_behind, 1.0 / BEHIND_HZ, now);
				}
				shown.set_vsync(!behind);
				shown.draw(panel, now);
				drawn = true;
				if behind {
					shown.next_behind.min(overlay_next)
				} else if !shown.focused && !overlay_up {
					// in the background with nothing in VR: light (the socket is still drained every turn)
					start.elapsed().as_secs_f64() + BACKGROUND_REST
				} else {
					// at once: the swap waits for the monitor
					now
				}
			}
		};
		if panel.st.dirty && now - *last_save > SAVE_EVERY {
			panel.save_settings();
			*last_save = now;
		}
		if let Some(s) = session.as_mut() {
			s.turned(now, start.elapsed().as_secs_f64(), drawn);
		}
		next
	}
}

impl ApplicationHandler for Host {
	fn resumed(&mut self, el: &ActiveEventLoop) {
		if self.shown.is_some() || self.error.is_some() {
			return;
		}
		match Shown::open(el) {
			Ok(shown) => {
				if shown.overlay.is_none() {
					self.panel.vc.overlay_wanted = false;
				}
				let mut shown = shown;
				shown.minimised = self.minimised;
				self.shown = Some(shown);
			}
			Err(e) => {
				self.error = Some(e);
				el.exit();
			}
		}
	}

	fn window_event(&mut self, el: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
		let Some(shown) = self.shown.as_mut() else { return };
		// egui's share first: the input, the scale (ScaleFactorChanged), the focus
		let _ = shown.egui.on_window_event(&shown.window, &event);
		match event {
			WindowEvent::CloseRequested => el.exit(),
			WindowEvent::Resized(_) => shown.window.resize_surface(&shown.surface, &shown.context),
			WindowEvent::Focused(focused) => shown.focused = focused,
			WindowEvent::Occluded(occluded) => shown.occluded = occluded,
			// the system wants it drawn again: uncovered, or resized while Windows holds the loop in its drag
			WindowEvent::RedrawRequested if shown.visible && !shown.hidden() => {
				let now = self.start.elapsed().as_secs_f64();
				shown.draw(&mut self.panel, now);
			}
			_ => {}
		}
	}

	fn about_to_wait(&mut self, el: &ActiveEventLoop) {
		let now = self.now();
		#[cfg(windows)]
		if now >= self.next_look {
			self.next_look = now + 1.0;
			if self.steamvr.as_ref().is_some_and(win::Process::gone) {
				el.exit();
				return;
			}
		}
		if now >= self.next_turn {
			self.next_turn = self.turn(now);
		}
		el.set_control_flow(if self.next_turn > self.now() {
			ControlFlow::WaitUntil(self.start + Duration::from_secs_f64(self.next_turn))
		} else {
			ControlFlow::Poll
		});
	}

	fn exiting(&mut self, _el: &ActiveEventLoop) {
		self.panel.save_settings();
		if let (Some(session), Some(shown)) = (&self.session, &self.shown) {
			session.report(shown.overlay.as_ref());
		}
		// the overlay and the OpenVR session go while the GL context the textures live in is still current
		self.panel.vc.close_vr();
		if let Some(shown) = self.shown.take() {
			shown.close();
		}
	}
}

/// The window and the GL context every view draws on
struct Shown {
	window: Window,
	surface: Surface<WindowSurface>,
	context: PossiblyCurrentContext,
	/// the window's own egui context and its winit side (input, cursor, clipboard, IME)
	ctx: egui::Context,
	egui: egui_winit::State,
	painter: egui_glow::Painter,
	/// none when its views could not be made
	overlay: Option<Overlay>,
	/// the swaps wait for the monitor
	vsync: bool,
	focused: bool,
	occluded: bool,
	/// drawn once and shown (it opens hidden, so nothing shows before the first frame)
	visible: bool,
	/// its first showing is minimised (started with SteamVR)
	minimised: bool,
	/// what the icon's dot says now (none: no icon set yet)
	dot: Option<Dot>,
	/// when the window's next frame is due while it is behind the overlay
	next_behind: f64,
}

impl Shown {
	/// The window's icon with its dot saying `dot`: the title bar's and, on Windows, the taskbar's larger one, each
	/// at the window's scale
	fn set_icon(&mut self, dot: Dot) {
		self.dot = Some(dot);
		let scale = self.window.scale_factor();
		let at = |points: f64| {
			let p = icon::picture((points * scale).round() as usize, dot)?;
			Icon::from_rgba(p.rgba, p.side as u32, p.side as u32).ok()
		};
		#[cfg(windows)]
		{
			use winit::platform::windows::WindowExtWindows;
			self.window.set_window_icon(at(16.0));
			self.window.set_taskbar_icon(at(32.0));
		}
		#[cfg(not(windows))]
		self.window.set_window_icon(at(48.0));
	}

	fn open(el: &ActiveEventLoop) -> Result<Shown, String> {
		let monitor = el.primary_monitor().or_else(|| el.available_monitors().next());
		let work = monitor.as_ref().map(work_area);
		let mut attrs = Window::default_attributes().with_title("sally panel").with_visible(false);
		attrs = match (&monitor, work) {
			(Some(m), Some(work)) => {
				let (size, pos, smallest) = placement(m.scale_factor(), work);
				attrs.with_inner_size(size).with_position(pos).with_min_inner_size(smallest)
			}
			_ => attrs
				.with_inner_size(LogicalSize::new(LAYOUT.0, LAYOUT.1))
				.with_min_inner_size(LogicalSize::new(SMALLEST.0, SMALLEST.1)),
		};
		let template = ConfigTemplateBuilder::new().with_alpha_size(8).with_depth_size(0).with_stencil_size(0);
		let (window, config) = DisplayBuilder::new()
			.with_preference(ApiPreference::FallbackEgl)
			.with_window_attributes(Some(attrs))
			// the fewest samples: egui smooths its own edges
			.build(el, template, |configs| configs.min_by_key(|c| c.num_samples()).expect("an OpenGL configuration"))
			.map_err(|e| format!("No OpenGL for a window: {e}"))?;
		let window = window.ok_or("No window")?;
		#[cfg(windows)]
		win::dark_title_bar(&window);
		if let Some(work) = work {
			centre(&window, work);
		}

		let display = config.display();
		let handle = window.window_handle().map_err(|e| format!("No window handle: {e}"))?.as_raw();
		let gl_attrs = ContextAttributesBuilder::new().build(Some(handle));
		// SAFETY: the handle is the window's, which outlives the context (`close` drops the context first)
		let context = match unsafe { display.create_context(&config, &gl_attrs) } {
			Ok(c) => c,
			Err(_) => {
				// OpenGL ES where there is no desktop OpenGL: egui_glow draws with either
				let gles_attrs =
					ContextAttributesBuilder::new().with_context_api(ContextApi::Gles(None)).build(Some(handle));
				// SAFETY: as above
				unsafe { display.create_context(&config, &gles_attrs) }
					.map_err(|e| format!("No OpenGL context: {e}"))?
			}
		};
		let surface_attrs = window
			.build_surface_attributes(SurfaceAttributesBuilder::new())
			.map_err(|e| format!("No window handle: {e}"))?;
		// SAFETY: the surface is the window's, and goes before it (`close`)
		let surface = unsafe { display.create_window_surface(&config, &surface_attrs) }
			.map_err(|e| format!("No OpenGL surface: {e}"))?;
		let context = context.make_current(&surface).map_err(|e| format!("No current OpenGL context: {e}"))?;
		if let Err(e) = surface.set_swap_interval(&context, SwapInterval::Wait(NonZeroU32::MIN)) {
			eprintln!("sally-panel: no vsync: {e}");
		}
		// SAFETY: the loader gives the functions of the context just made current, which stays current to the end
		let gl = Arc::new(unsafe { glow::Context::from_loader_function_cstr(|s| display.get_proc_address(s)) });
		let painter =
			egui_glow::Painter::new(gl.clone(), "", None, false).map_err(|e| format!("No OpenGL painter: {e}"))?;
		let ctx = egui::Context::default();
		Panel::setup(&ctx);
		let egui = egui_winit::State::new(
			ctx.clone(),
			ViewportId::ROOT,
			el,
			Some(window.scale_factor() as f32),
			window.theme(),
			Some(painter.max_texture_side()),
		);
		let overlay = Overlay::new(&gl).inspect_err(|e| eprintln!("sally-panel: no overlay: {e}")).ok();
		Ok(Shown {
			window,
			surface,
			context,
			ctx,
			egui,
			painter,
			overlay,
			vsync: true,
			focused: false,
			occluded: false,
			visible: false,
			minimised: false,
			dot: None,
			next_behind: 0.0,
		})
	}

	/// Minimised or covered: nothing of it to see (Windows says minimised with a size of 0)
	fn hidden(&self) -> bool {
		let size = self.window.inner_size();
		self.occluded || size.width == 0 || size.height == 0 || self.window.is_minimized() == Some(true)
	}

	/// Swaps that wait for the monitor, or not
	fn set_vsync(&mut self, on: bool) {
		if self.vsync == on {
			return;
		}
		self.vsync = on;
		let interval = if on { SwapInterval::Wait(NonZeroU32::MIN) } else { SwapInterval::DontWait };
		if let Err(e) = self.surface.set_swap_interval(&self.context, interval) {
			eprintln!("sally-panel: swap interval: {e}");
		}
	}

	/// The window's frame: the panel run in the window's egui context, painted and swapped; shown after the first
	fn draw(&mut self, panel: &mut Panel, now: f64) {
		let raw = self.egui.take_egui_input(&self.window);
		let mut out = self.ctx.run_ui(raw, |ui| panel.frame(ui, now, View::Window));
		// the cursor's look, the clipboard, the IME's place, links
		self.egui.handle_platform_output(&self.window, std::mem::take(&mut out.platform_output));
		let prims = self.ctx.tessellate(std::mem::take(&mut out.shapes), out.pixels_per_point);
		let size = self.window.inner_size();
		let px = [size.width, size.height];
		self.painter.clear(px, theme::BG.to_normalized_gamma_f32());
		self.painter.paint_and_update_textures(px, out.pixels_per_point, &prims, &mut out.textures_delta);
		if let Err(e) = self.surface.swap_buffers(&self.context) {
			eprintln!("sally-panel: swap: {e}");
		}
		if !self.visible {
			if self.minimised {
				// started with SteamVR: on the taskbar, not over what is on the screen
				#[cfg(windows)]
				win::show_minimised(&self.window);
				#[cfg(not(windows))]
				{
					self.window.set_minimized(true);
					self.window.set_visible(true);
				}
			} else {
				self.window.set_visible(true);
			}
			self.visible = true;
		}
	}

	/// The end, OpenVR done with the textures: the views and the painter, the GL context, its surface, the window
	fn close(mut self) {
		if let Some(o) = self.overlay.as_mut() {
			o.destroy();
		}
		self.painter.destroy();
		let Shown { window, surface, context, egui, .. } = self;
		drop(egui);
		match context.make_not_current() {
			Ok(not_current) => drop(not_current),
			Err(e) => eprintln!("sally-panel: OpenGL context: {e}"),
		}
		drop(surface);
		drop(window);
	}
}

/// The overlay's side: its two views on the window's GL context, and its laser
struct Overlay {
	/// the panel, the layout at 1.25 pixels a point
	view: Offscreen,
	/// the dashboard button's picture, upside down as the dashboard shows it
	button: Offscreen,
	laser: OverlayInput,
	/// when the next frame is due
	next_frame: f64,
	/// when the button's picture was drawn, and for which of the overlay's creations (`overlay_gen`)
	button_drawn: f64,
	button_gen: i32,
}

impl Overlay {
	fn new(gl: &Arc<glow::Context>) -> Result<Overlay, String> {
		let mut view = Offscreen::new(gl.clone(), OVERLAY_SIZE.0, OVERLAY_SIZE.1, OVERLAY_PPP)?;
		let mut button = match Offscreen::new(gl.clone(), BUTTON_SIZE, BUTTON_SIZE, 1.0) {
			Ok(b) => b,
			Err(e) => {
				view.destroy();
				return Err(e);
			}
		};
		button.mirror = true;
		Panel::setup(&view.ctx);
		Panel::setup(&button.ctx);
		Ok(Overlay { view, button, laser: OverlayInput::default(), next_frame: 0.0, button_drawn: -1e9, button_gen: 0 })
	}

	/// The button's picture when one is due, and the overlay's frame when it is up and due; whether it is up
	fn turn(&mut self, panel: &mut Panel, now: f64) -> bool {
		let vc = &panel.vc;
		let made = self.button_gen != vc.overlay_gen;
		let elsewhere = vc.thumb_visible && !vc.overlay_visible && now - self.button_drawn > BUTTON_EVERY;
		if vc.overlay_ok && (made || elsewhere) {
			// a view of its own: drawn between two of the overlay's frames, it does not end the laser's drag there
			self.button.frame(now, Vec::new(), |ui| panel.thumbnail(ui, now));
			panel.vc.overlay_submit_thumb(self.button.gl_name());
			self.button_drawn = now;
			self.button_gen = panel.vc.overlay_gen;
		}
		let up = panel.vc.overlay_ok && panel.vc.overlay_visible;
		if up && now >= self.next_frame {
			let events = self.laser.take(&mut panel.vc.ov_events, OVERLAY_PPP);
			self.view.frame(now, events, |ui| panel.frame(ui, now, View::Overlay));
			panel.vc.overlay_submit(self.view.gl_name());
			self.next_frame = next_due(self.next_frame, 1.0 / OVERLAY_HZ, now);
		}
		up
	}

	fn destroy(&mut self) {
		self.view.destroy();
		self.button.destroy();
	}
}

/// What the fake SteamVR does in the session, at a time on the host's clock
enum Step {
	/// the dashboard as SteamVR shows it: the sally tab open (the overlay), the bar of tabs up (the button)
	Show { overlay: bool, button: bool },
	/// the laser on the overlay
	Laser(VrEvent),
}

/// For the tests only (`SALLY_PANEL_FAKE_VR=1`): valve's fake SteamVR in openvr_api's place, playing Valve's driver
/// on what the runtime sees, and a dashboard session played on it from the start. For 2 s the dashboard is up on
/// another tab (the button's picture drawn at the overlay's creation and each second); then the sally tab is open:
/// the laser clicks Tracker 3545's row (the move, the press and the release in one go, as SteamVR often sends them),
/// holds and drags across the room view for 3 s with the thumb on the stick (moves and scroll ticks interleaved, 300
/// events a second), clicks the Settings tab and leaves; at 7.5 s the dashboard closes. At the end the panel prints
/// what the fake was handed and writes the overlay's and the button's last pictures in the current folder.
struct FakeSession {
	steamvr: SteamVr,
	script: VecDeque<(f64, Step)>,
	/// when the sally tab opened and closed
	open: Option<f64>,
	closed: Option<f64>,
	/// when each of the overlay's textures was handed over, and each window frame drawn (as the turns saw them)
	overlay_frames: Vec<f64>,
	window_frames: Vec<f64>,
	/// each turn's start, the end of its poll and its end: how busy the loop was, and with what
	turns: Vec<(f64, f64, f64)>,
	polled: f64,
	/// the laser's events most waiting for an overlay frame
	queue_peak: usize,
}

impl FakeSession {
	fn new(steamvr: SteamVr) -> FakeSession {
		FakeSession {
			steamvr,
			script: FakeSession::script(),
			open: None,
			closed: None,
			overlay_frames: Vec::new(),
			window_frames: Vec::new(),
			turns: Vec::new(),
			polled: 0.0,
			queue_peak: 0,
		}
	}

	/// The session, in texture pixels (y down), as the overlay's mouse scale gives the laser
	fn script() -> VecDeque<(f64, Step)> {
		let laser = |t: f64, kind: EVREventType, x: f32, y: f32| {
			(t, Step::Laser(fake::mouse_event(kind, x, y, VRMouseButton_Left)))
		};
		let click = [VREvent_MouseMove, VREvent_MouseButtonDown, VREvent_MouseButtonUp];
		let mut s = VecDeque::new();
		s.push_back((0.0, Step::Show { overlay: false, button: true }));
		s.push_back((2.0, Step::Show { overlay: true, button: true }));
		s.extend(click.map(|kind| laser(2.5, kind, 110.0, 307.0)));
		s.push_back(laser(3.0, VREvent_MouseButtonDown, 1020.0, 520.0));
		let mut at = (1020.0, 520.0);
		for k in 0..900 {
			let t = 3.0 + k as f64 / 300.0;
			if k % 2 == 0 {
				let a = (t - 3.0) as f32;
				at = (1020.0 + 250.0 * (a * 3.0).sin(), 520.0 + 80.0 * (a * 5.0).sin());
				s.push_back(laser(t, VREvent_MouseMove, at.0, at.1));
			} else {
				s.push_back((t, Step::Laser(fake::scroll_event(VREvent_ScrollSmooth, 0.0, 0.01))));
			}
		}
		s.push_back(laser(6.0, VREvent_MouseButtonUp, at.0, at.1));
		s.extend(click.map(|kind| laser(6.5, kind, 532.0, 31.0)));
		s.push_back((7.0, Step::Laser(fake::event(VREvent_FocusLeave))));
		s.push_back((7.5, Step::Show { overlay: false, button: false }));
		s
	}

	/// Before the turn's poll: the steps that are due, and Valve's driver seeing what the runtime sees
	fn play(&mut self, now: f64, rt: &Runtime) {
		// (the fake is locked here only: it locks itself for each of the panel's calls)
		let mut vr = self.steamvr.lock();
		while let Some((_, step)) = self.script.pop_front_if(|(t, _)| *t <= now) {
			match step {
				Step::Show { overlay, button } => {
					vr.overlay_visible = overlay;
					vr.thumb_visible = button;
					if overlay {
						self.open = Some(now);
					} else if self.open.is_some() {
						self.closed = Some(now);
					}
				}
				Step::Laser(ev) => vr.post_overlay_event(fake::MAIN, ev),
			}
		}
		vr.follow(rt);
	}

	/// After the poll (ended at `at`): the laser's events waiting for the overlay's frame
	fn waiting(&mut self, at: f64, vc: &ValveCompare) {
		self.polled = at;
		if vc.overlay_visible {
			self.queue_peak = self.queue_peak.max(vc.ov_events.len());
		}
	}

	/// After the turn (begun at `now`, ended at `end`): the overlay's textures handed over since the last, and the
	/// window's frame if it was drawn
	fn turned(&mut self, now: f64, end: f64, window_drawn: bool) {
		self.turns.push((now, self.polled, end));
		let handed = self.steamvr.lock().textures.iter().filter(|t| t.overlay == fake::MAIN).count();
		let new = handed.saturating_sub(self.overlay_frames.len());
		self.overlay_frames.extend(std::iter::repeat_n(now, new));
		if window_drawn {
			self.window_frames.push(now);
		}
	}

	/// What the fake SteamVR was handed, and the overlay's and the button's last pictures
	fn report(&self, overlay: Option<&Overlay>) {
		let vr = self.steamvr.lock();
		let made = vr.calls.iter().filter(|c| c.starts_with("create ")).count();
		let names = |handle| {
			let mut n: Vec<u32> = vr.textures.iter().filter(|t| t.overlay == handle).map(|t| t.gl_texture).collect();
			n.dedup();
			n
		};
		let (main, thumb) = (names(fake::MAIN), names(fake::THUMB));
		let thumbs = vr.textures.iter().filter(|t| t.overlay == fake::THUMB).count();
		let gl_gamma =
			vr.textures.iter().all(|t| t.texture_type == TextureType_OpenGL && t.color_space == ColorSpace_Gamma);
		let open = self.open.unwrap_or(f64::NAN);
		let closed = self.closed.unwrap_or_else(|| self.overlay_frames.last().copied().unwrap_or(open));
		let within = |times: &[f64]| times.iter().copied().filter(|&t| t >= open && t <= closed).collect::<Vec<_>>();
		let (frames, window) = (within(&self.overlay_frames), within(&self.window_frames));
		let span = closed - open;
		// the longest wait between two frames and when it began (s after the tab opened)
		let (gap, gap_at) = frames
			.windows(2)
			.map(|w| (w[1] - w[0], w[0] - open))
			.max_by(|a, b| a.0.total_cmp(&b.0))
			.unwrap_or((0.0, f64::NAN));
		let turns = self.turns.iter().filter(|t| t.0 >= open && t.0 <= closed);
		let (turn_count, busy) = turns.clone().fold((0, 0.0), |(n, b), t| (n + 1, b + t.2 - t.0));
		// the longest turn: how long, when, and how much of it was the poll
		let (took, took_at, poll) = turns
			.max_by(|a, b| (a.2 - a.0).total_cmp(&(b.2 - b.0)))
			.map_or((0.0, f64::NAN, 0.0), |t| (t.2 - t.0, t.0 - open, t.1 - t.0));
		println!(
			"fake SteamVR: the overlay made {made} time(s); {} overlay frames handed over in all, {} in the {span:.2} \
			 s the sally tab was open ({:.1} a second, the longest gap {:.0} ms at {gap_at:.2} s, the longest turn \
			 {:.0} ms at {took_at:.2} s, {:.0} ms of it the poll), textures {main:?}; {thumbs} button pictures, \
			 textures {thumb:?}; all OpenGL in gamma: {gl_gamma}; the window drawn {:.1} times a second meanwhile; \
			 the loop's {turn_count} turns busy {:.0} % of that time; the laser's queue at most {} events",
			self.overlay_frames.len(),
			frames.len(),
			frames.len() as f64 / span,
			gap * 1000.0,
			took * 1000.0,
			poll * 1000.0,
			window.len() as f64 / span,
			busy / span * 100.0,
			self.queue_peak,
		);
		if let Some(o) = overlay {
			for (view, file) in [(&o.view, "sally-panel-overlay.png"), (&o.button, "sally-panel-button.png")] {
				match save_png(&view.read(), Path::new(file)) {
					Ok(()) => println!("{file}"),
					Err(e) => eprintln!("{file}: {e}"),
				}
			}
		}
	}
}

/// What winit does not do on Windows: the dark title bar, the work area, a message box
#[cfg(windows)]
mod win {
	use std::ffi::c_void;

	use raw_window_handle::{HasWindowHandle, RawWindowHandle};
	use winit::monitor::MonitorHandle;
	use winit::platform::windows::MonitorHandleExtWindows;
	use winit::window::Window;

	use super::Area;

	#[repr(C)]
	#[derive(Default)]
	struct Rect {
		left: i32,
		top: i32,
		right: i32,
		bottom: i32,
	}

	#[repr(C)]
	#[derive(Default)]
	struct MonitorInfo {
		size: u32,
		monitor: Rect,
		work: Rect,
		flags: u32,
	}

	/// DWMWA_USE_IMMERSIVE_DARK_MODE
	const USE_IMMERSIVE_DARK_MODE: u32 = 20;
	/// MB_ICONERROR
	const ICON_ERROR: u32 = 0x10;

	#[link(name = "user32")]
	extern "system" {
		fn GetMonitorInfoW(monitor: isize, info: *mut MonitorInfo) -> i32;
		fn MessageBoxW(owner: isize, text: *const u16, caption: *const u16, kind: u32) -> i32;
	}

	#[link(name = "dwmapi")]
	extern "system" {
		fn DwmSetWindowAttribute(hwnd: isize, attribute: u32, value: *const c_void, size: u32) -> i32;
	}

	/// The title bar dark like the panel (from Windows 10 20H1; an older one keeps its light one)
	pub fn dark_title_bar(window: &Window) {
		let Ok(handle) = window.window_handle() else { return };
		if let RawWindowHandle::Win32(h) = handle.as_raw() {
			let dark: i32 = 1;
			// SAFETY: the window's own handle, alive with `window`; the value is a BOOL, its size given
			unsafe {
				DwmSetWindowAttribute(h.hwnd.get(), USE_IMMERSIVE_DARK_MODE, (&raw const dark).cast(), 4);
			}
		}
	}

	/// The monitor's work area: the monitor less the taskbar
	pub fn work_area(monitor: &MonitorHandle) -> Option<Area> {
		let mut info = MonitorInfo { size: size_of::<MonitorInfo>() as u32, ..MonitorInfo::default() };
		// SAFETY: a monitor's handle as winit holds it; MONITORINFO with its size set, as the call wants it
		let ok = unsafe { GetMonitorInfoW(monitor.hmonitor(), &mut info) } != 0;
		let r = &info.work;
		(ok && r.right > r.left && r.bottom > r.top).then(|| Area {
			x: r.left,
			y: r.top,
			w: (r.right - r.left) as u32,
			h: (r.bottom - r.top) as u32,
		})
	}

	/// ERROR_ALREADY_EXISTS
	const ALREADY_EXISTS: u32 = 183;
	const SW_SHOWMINNOACTIVE: i32 = 7;
	const SW_RESTORE: i32 = 9;
	const SYNCHRONIZE: u32 = 0x0010_0000;
	const WAIT_TIMEOUT: u32 = 0x102;

	#[link(name = "user32")]
	extern "system" {
		fn FindWindowW(class: *const u16, title: *const u16) -> isize;
		fn IsIconic(hwnd: isize) -> i32;
		fn ShowWindow(hwnd: isize, how: i32) -> i32;
		fn SetForegroundWindow(hwnd: isize) -> i32;
	}

	#[link(name = "kernel32")]
	extern "system" {
		fn CreateMutexW(security: *const c_void, owned: i32, name: *const u16) -> isize;
		fn GetLastError() -> u32;
		fn OpenProcess(access: u32, inherit: i32, pid: u32) -> isize;
		fn WaitForSingleObject(handle: isize, ms: u32) -> u32;
		fn CloseHandle(handle: isize) -> i32;
	}

	fn wide(s: &str) -> Vec<u16> {
		s.encode_utf16().chain(Some(0)).collect()
	}

	/// Whether this is the only panel running: the first to make the panel's named mutex, which it holds to its end
	pub fn only_panel() -> bool {
		let name = wide("sally-panel");
		// SAFETY: a NUL-terminated name that outlives the call; the handle is kept for the process's life on purpose
		unsafe {
			let h = CreateMutexW(std::ptr::null(), 0, name.as_ptr());
			h == 0 || GetLastError() != ALREADY_EXISTS
		}
	}

	/// The running panel's window brought up: restored where it is minimised, and in front
	pub fn show_running() {
		let title = wide("sally panel");
		// SAFETY: a NUL-terminated title that outlives the call; the handle is used only while it is one
		unsafe {
			let h = FindWindowW(std::ptr::null(), title.as_ptr());
			if h != 0 {
				if IsIconic(h) != 0 {
					ShowWindow(h, SW_RESTORE);
				}
				SetForegroundWindow(h);
			}
		}
	}

	/// The window shown for the first time minimised, on the taskbar, without taking the focus
	pub fn show_minimised(window: &Window) {
		let Ok(handle) = window.window_handle() else { return };
		if let RawWindowHandle::Win32(h) = handle.as_raw() {
			// SAFETY: the window's own handle, alive with `window`
			unsafe {
				ShowWindow(h.hwnd.get(), SW_SHOWMINNOACTIVE);
			}
		}
	}

	/// Another process, watched for its end
	pub struct Process(isize);

	impl Process {
		/// None where it cannot be watched (it has gone already, or is not ours to ask about)
		pub fn open(pid: u32) -> Option<Process> {
			// SAFETY: the handle, where there is one, is closed when the Process is dropped
			let h = unsafe { OpenProcess(SYNCHRONIZE, 0, pid) };
			(h != 0).then_some(Process(h))
		}

		pub fn gone(&self) -> bool {
			// SAFETY: the handle is open while the Process lives
			unsafe { WaitForSingleObject(self.0, 0) != WAIT_TIMEOUT }
		}
	}

	impl Drop for Process {
		fn drop(&mut self) {
			// SAFETY: the handle OpenProcess gave, closed once
			unsafe {
				CloseHandle(self.0);
			}
		}
	}

	pub fn message_box(text: &str) {
		let (text, caption) = (wide(text), wide("sally panel"));
		// SAFETY: two NUL-terminated UTF-16 strings that outlive the call, and no owner window
		unsafe {
			MessageBoxW(0, text.as_ptr(), caption.as_ptr(), ICON_ERROR);
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn args(list: &[&str]) -> Args {
		Args::parse(list.iter().map(OsString::from))
	}

	#[test]
	fn host_and_port_from_the_command_line() {
		assert_eq!(args(&[]), Args::default());
		assert_eq!(
			args(&["--port", "9101", "--host", "192.168.1.5"]),
			Args { host: Some("192.168.1.5".into()), port: Some(9101), ..Args::default() }
		);
		// _wtoi's reading: the leading digits, 0 for none, and the port's range
		assert_eq!(args(&["--port", "9177x"]).port, Some(9177));
		assert_eq!(args(&["--port", "x"]).port, Some(0));
		assert_eq!(args(&["--port", "70000"]).port, Some(65535));
		// a flag without its value, and what is not the panel's
		assert_eq!(args(&["--verbose", "--host"]), Args::default());
		let a = args(&["--replay-check", "latest", "--again"]);
		assert_eq!((a.replay_check.as_deref(), a.again), (Some("latest"), true));
	}

	#[test]
	fn the_window_opens_centred_within_the_work_area() {
		// 2560 x 1440 at 125 %, the taskbar 48 pixels: the layout fits as it is
		let work = Area { x: 0, y: 0, w: 2560, h: 1392 };
		let (size, pos, smallest) = placement(1.25, work);
		assert_eq!(size, PhysicalSize::new(2000, 1175));
		assert_eq!(pos, PhysicalPosition::new(280, 108));
		assert_eq!(smallest, LogicalSize::new(1100.0, 700.0));
		// 1920 x 1080 at 150 %, on the left of the main monitor: kept within the area less the margin
		let work = Area { x: -1920, y: 0, w: 1920, h: 1040 };
		let (size, pos, smallest) = placement(1.5, work);
		assert_eq!(size, PhysicalSize::new(1880, 1000));
		assert_eq!(pos, PhysicalPosition::new(-1900, 20));
		assert_eq!(smallest.width, 1100.0);
		assert!((smallest.height - 1000.0 / 1.5).abs() < 1e-9);
	}

	#[test]
	fn what_is_due_keeps_its_rate() {
		// on time, and a little late: a period after it was due
		assert!((next_due(1.0, 0.1, 1.0) - 1.1).abs() < 1e-12);
		assert!((next_due(1.0, 0.1, 1.05) - 1.1).abs() < 1e-12);
		// more than a period late: a period from now
		assert!((next_due(1.0, 0.1, 1.25) - 1.35).abs() < 1e-12);
		// 90 Hz from turns at 240 Hz: 90 frames a second on the whole
		let (mut due, mut frames) = (0.0, 0);
		for k in 0..240 {
			let now = 10.0 + k as f64 / 240.0;
			if now >= due {
				due = next_due(due, 1.0 / 90.0, now);
				frames += 1;
			}
		}
		assert!((89..=91).contains(&frames), "{frames}");
	}
}
