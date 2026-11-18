//! The Replay tab (docs/sally_design.md 9.3): a recorded session played back in the room, sally run on it again
//! beside what SteamVR got live and Valve's own tracking, each device as its model; a timeline of how far each is
//! from Valve, with the moments Valve lost the device, the gaps in sally's tracks and the tracker's events; played at
//! any speed, stepped from event to event, or scrubbed. What it reads and runs is `crate::replay`'s.

use std::path::PathBuf;
use std::sync::mpsc;
use std::thread::JoinHandle;

use egui::{Align2, Color32, CornerRadius, Id, Key, PointerButton, Pos2, Rect, Sense, Shape, Stroke, StrokeKind, Vec2};
use sally_math::math3d::{dot, len, xform, Pose, V3};

use super::icons;
use super::kit::{self, Surface, HEADER_H};
use super::room::{camera_input, draw_tags, grid_squares, model_name_for, Cam3, Camera, Tag};
use super::theme;
use super::theme::*;
use super::{Panel, View};
use crate::replay::{self, Data, Dev, Job, Paths, Session};

/// The tracks: Valve's own tracking, what SteamVR got (sally live), sally run again
pub fn tracks() -> [(&'static str, Color32); 3] {
	let t = theme::tk();
	[("Valve", t.valve), ("SteamVR got", t.note), ("Replay", t.good)]
}
const SPEEDS: [f64; 6] = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0];
/// The other devices' models
const OTHERS: Color32 = Color32::from_rgb(212, 216, 226);
/// The stations' colours (the light's rays and sensors), by their order
/// The light: a sensor or station lit, one not, a ghosted track's models
const LIT: Color32 = Color32::from_rgb(250, 250, 252);
const UNLIT: Color32 = Color32::from_rgb(120, 123, 132);
const GHOST: u8 = 115;
/// How far back the light at the cursor reaches (s): a few turns of a station's rotor
const LIGHT_WINDOW: f64 = 0.1;
/// Samples further apart are not joined on the graph (s)
const GRAPH_GAP: f32 = 0.3;

fn station_pose(d: &Data, ch: u8) -> Option<Pose> {
	d.stations.iter().find(|s| s.0 == ch as i32).map(|s| s.1)
}

type Loading = (mpsc::Receiver<String>, JoinHandle<Result<Data, String>>);

/// The tab's state, kept while the panel runs
pub struct ReplayState {
	paths: Option<Paths>,
	/// where device configs are (by serial: the models' names), looked for again now and then
	dirs: Vec<PathBuf>,
	dirs_t: f64,
	sessions: Vec<Session>,
	scanned: f64,
	picking: bool,
	pick_scroll: f32,
	session: Option<Session>,
	job: Option<Job>,
	loading: Option<Loading>,
	status: String,
	error: String,
	data: Option<Data>,
	/// the graph's height per device (m): its distances' 99.5th percentile
	ymax: Vec<f64>,
	/// the lowest a device went, less a tracker's height
	floor: f64,
	t: f64,
	playing: bool,
	speed: f64,
	last_now: f64,
	dev: usize,
	cam: Camera,
	/// the timeline's range (s)
	view: (f64, f64),
	follow: bool,
	scrubbing: bool,
	/// the track drawn solid, the others ghosted (None: all solid)
	primary: Option<usize>,
	/// the station whose light is drawn (None: the light of all, no rays)
	station: Option<i32>,
	/// whether the cursor was on the timeline at the last frame (playing pages on only from there)
	cursor_shown: bool,
}

impl Default for ReplayState {
	fn default() -> ReplayState {
		ReplayState {
			paths: None,
			dirs: Vec::new(),
			dirs_t: -1e9,
			sessions: Vec::new(),
			scanned: -1e9,
			picking: false,
			pick_scroll: 0.0,
			session: None,
			job: None,
			loading: None,
			status: String::new(),
			error: String::new(),
			data: None,
			ymax: Vec::new(),
			floor: 0.0,
			t: 0.0,
			playing: false,
			speed: 1.0,
			last_now: 0.0,
			dev: 0,
			cam: Camera::default(),
			view: (0.0, 1.0),
			follow: false,
			scrubbing: false,
			primary: None,
			station: None,
			cursor_shown: true,
		}
	}
}

/// "20261107_123001" as "7 Nov 12:30:01"
fn stamp_text(s: &str) -> String {
	const MON: [&str; 12] = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
	if s.len() != 15 || !s.is_ascii() || s.as_bytes()[8] != b'_' {
		return s.into();
	}
	let m: usize = s[4..6].parse().unwrap_or(1);
	let d: u32 = s[6..8].parse().unwrap_or(0);
	format!("{d} {} {}:{}:{}", MON[m.clamp(1, 12) - 1], &s[9..11], &s[11..13], &s[13..15])
}

/// A time on the axis: with tenths where the ticks are closer than a second
fn tick_text(s: f64, step: f64) -> String {
	if step >= 1.0 {
		return dur_text(s);
	}
	let s = s.max(0.0);
	format!("{}:{:04.1}", (s / 60.0) as u64, s % 60.0)
}

fn dur_text(s: f64) -> String {
	let s = s.max(0.0) as u64;
	if s >= 3600 {
		format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
	} else {
		format!("{}:{:02}", s / 60, s % 60)
	}
}

/// A distance as the panel shows it
fn mm(m: f32) -> String {
	if m < 0.0 {
		"-".into()
	} else if m < 0.01 {
		format!("{:.1} mm", m * 1e3)
	} else {
		format!("{:.0} mm", m * 1e3)
	}
}

/// The distance at a time from a list of (t, m): the sample nearest, within 0.1 s
fn err_at(e: &[(f32, f32)], t: f64) -> Option<f32> {
	let i = e.partition_point(|q| (q.0 as f64) < t);
	[i.wrapping_sub(1), i]
		.into_iter()
		.filter_map(|k| e.get(k))
		.filter(|q| (q.0 as f64 - t).abs() < 0.1)
		.min_by(|a, b| (a.0 as f64 - t).abs().total_cmp(&(b.0 as f64 - t).abs()))
		.map(|q| q.1)
}

/// Which track a device is labelled and picked by: the replay, else what SteamVR got, else Valve's
fn primary(tracks: [bool; 3]) -> usize {
	if tracks[2] {
		2
	} else if tracks[1] {
		1
	} else {
		0
	}
}

impl Panel {
	/// Where the replays read from (found once)
	pub(super) fn replay_paths(&mut self) -> &Paths {
		if self.rp.paths.is_none() {
			let dir = self.st.path.parent().map(|p| p.to_path_buf()).unwrap_or_default();
			let dir = if dir.as_os_str().is_empty() { std::env::current_dir().unwrap_or_default() } else { dir };
			self.rp.paths = Some(Paths::discover(&dir, &self.st.replay_project));
		}
		self.rp.paths.as_ref().expect("just found")
	}

	/// The replays of sessions that are gone removed (`replay::prune`): when the panel starts, which with SteamVR is
	/// just after the driver has deleted the sessions no longer kept, and whenever the sessions are listed
	pub fn replay_prune(&mut self) {
		let paths = self.replay_paths().clone();
		replay::prune(&paths);
	}

	/// The sessions listed again, the replays of the ones that are gone removed first
	fn replay_scan(&mut self) {
		self.replay_prune();
		let paths = self.replay_paths().clone();
		self.rp.sessions = replay::sessions(&paths);
		self.rp.scanned = self.now;
	}

	/// Where device configs are looked for by serial (the models' names): every 30 s again
	pub(super) fn config_dirs(&mut self) -> Vec<PathBuf> {
		if self.now - self.rp.dirs_t > 30.0 || self.now < self.rp.dirs_t {
			self.rp.dirs = self.replay_paths().config_dirs();
			self.rp.dirs_t = self.now;
			self.symbols.retry();
		}
		self.rp.dirs.clone()
	}

	/// The loaded replay's devices: each one's serial, and what the runtime would call it as far as its model's name
	/// says (for its symbol where its config is not found)
	pub(super) fn replay_serials(&self) -> Vec<(String, String)> {
		let Some(data) = &self.rp.data else { return Vec::new() };
		data.devs
			.iter()
			.map(|d| {
				let m = d.model.to_ascii_lowercase();
				let subtype = if m.contains("knu") && m.contains("left") {
					"knuckles_l"
				} else if m.contains("knu") && m.contains("right") {
					"knuckles_r"
				} else if m.contains("hmd") || m.contains("headset") {
					"index_hmd"
				} else {
					"tracker"
				};
				(d.serial.clone(), subtype.to_string())
			})
			.collect()
	}

	/// The floor grid's lines (every metre, between): on a picture, white veils
	pub(super) fn grid_colors(&self) -> (Color32, Color32) {
		(theme::tk().grid, theme::tk().grid_minor)
	}

	/// Opens a session by its key (`Session::key`): its replay when one has finished (unless `again`), else a replay
	/// started
	pub fn replay_open(&mut self, stamp: &str, again: bool) {
		let paths = self.replay_paths().clone();
		if self.rp.sessions.iter().all(|s| s.key() != stamp) {
			self.replay_scan();
		}
		let Some(s) = self.rp.sessions.iter().find(|s| s.key() == stamp).cloned() else {
			self.rp.error = format!("no session {stamp} in {}", paths.logs.display());
			return;
		};
		if let Some(j) = self.rp.job.as_mut() {
			j.cancel();
		}
		self.rp.job = None;
		self.rp.loading = None;
		self.rp.data = None;
		self.rp.error.clear();
		self.rp.status.clear();
		self.rp.picking = false;
		self.rp.session = Some(s.clone());
		self.rp.t = 0.0;
		self.rp.playing = false;
		self.rp.view = (0.0, s.duration().max(1.0));
		let done_ok = s.replayed.as_deref().is_some_and(|d| d.starts_with("exit 0"));
		if done_ok && !again {
			self.replay_load(s);
		} else {
			match replay::start(&paths, &s) {
				Ok(j) => {
					self.rp.status = "replaying".into();
					self.rp.job = Some(j);
				}
				Err(e) => self.rp.error = e,
			}
		}
	}

	/// The timeline brought to the cursor where it is off it (a jump to an event, play), a third in
	fn show_cursor(&mut self) {
		let (v0, v1) = self.rp.view;
		if !(v0..=v1).contains(&self.rp.t) {
			let span = v1 - v0;
			self.rp.view = (self.rp.t - span / 3.0, self.rp.t + span * 2.0 / 3.0);
		}
		self.rp.cursor_shown = true;
	}

	/// Out of the session: the replay running stopped, what was loaded let go, the tab back to the picker
	pub fn replay_close(&mut self) {
		if let Some(j) = self.rp.job.as_mut() {
			j.cancel();
		}
		self.rp.job = None;
		self.rp.loading = None;
		self.rp.data = None;
		self.rp.session = None;
		self.rp.status.clear();
		self.rp.error.clear();
		self.rp.playing = false;
		self.rp.station = None;
		self.rp.t = 0.0;
	}

	/// Host override (the headless runs): the cursor at `t`, the device at `dev` selected and followed from `dist` m
	pub fn replay_seek(&mut self, t: f64, dev: Option<usize>, dist: Option<f64>) {
		self.rp.t = t.max(0.0);
		self.rp.cursor_shown = true;
		if let Some(d) = dev {
			self.rp.dev = d;
		}
		if let Some(m) = dist {
			self.rp.follow = true;
			self.rp.cam.dist = m.clamp(0.3, 60.0);
		}
	}

	/// Host override (the headless runs): the primary track and the station whose light is drawn
	pub fn replay_focus(&mut self, primary: Option<usize>, station: Option<i32>) {
		self.rp.primary = primary.filter(|k| *k < 3);
		self.rp.station = station;
	}

	/// Host override (the headless runs): the room's camera turned to `yaw` and `pitch` (radians)
	pub fn replay_cam(&mut self, yaw: f64, pitch: f64) {
		self.rp.cam.yaw = yaw;
		self.rp.cam.pitch = pitch.clamp(-1.45, 1.5);
	}

	/// Host override (the headless runs): the timeline's range
	pub fn replay_view(&mut self, v0: f64, v1: f64) {
		if v1 > v0 {
			self.rp.view = (v0, v1);
		}
	}

	/// Whether a replay is running or loading
	pub fn replay_busy(&self) -> bool {
		self.rp.job.is_some() || self.rp.loading.is_some()
	}

	fn replay_load(&mut self, s: Session) {
		let dir = s.replay_dir(self.replay_paths());
		let (tx, rx) = mpsc::channel();
		let h = std::thread::spawn(move || replay::load(&dir, &s, Some(tx)));
		self.rp.loading = Some((rx, h));
		self.rp.status = "loading".into();
	}

	/// The replay's run and its loading, once a frame
	fn replay_poll(&mut self) {
		if let Some(j) = self.rp.job.as_mut() {
			match j.poll() {
				Ok(false) => {
					self.rp.status = if j.dur > 0.0 {
						format!("replaying with this build: {:.0} %", (j.at / j.dur * 100.0).clamp(0.0, 100.0))
					} else {
						"replaying".into()
					};
				}
				Ok(true) => {
					self.rp.job = None;
					if let Some(mut s) = self.rp.session.clone() {
						s.replayed = Some("exit 0".into());
						for x in self.rp.sessions.iter_mut().filter(|x| x.key() == s.key()) {
							x.replayed = s.replayed.clone();
						}
						self.rp.session = Some(s.clone());
						self.replay_load(s);
					}
				}
				Err(e) => {
					self.rp.job = None;
					self.rp.status.clear();
					self.rp.error = e;
				}
			}
		}
		if let Some((rx, _)) = &self.rp.loading {
			while let Ok(m) = rx.try_recv() {
				if !m.is_empty() {
					self.rp.status = m;
				}
			}
		}
		if self.rp.loading.as_ref().is_some_and(|(_, h)| h.is_finished()) {
			let (_, h) = self.rp.loading.take().expect("is there");
			self.rp.status.clear();
			match h.join() {
				Ok(Ok(d)) => self.replay_loaded(d),
				Ok(Err(e)) => self.rp.error = e,
				Err(_) => self.rp.error = "loading the replay failed".into(),
			}
		}
	}

	fn replay_loaded(&mut self, d: Data) {
		// where a device can be: within 8 m of the stations' middle, from 4 m under them to 1.5 m over (a device that
		// ran away falls through the floor before it is lost)
		let mid = if d.stations.is_empty() {
			None
		} else {
			Some(d.stations.iter().fold(V3::default(), |a, s| a + s.1.p) * (1.0 / d.stations.len() as f64))
		};
		let plausible = |p: V3| {
			mid.is_none_or(|m| {
				let h = V3::new(p.x - m.x, p.y - m.y, 0.0);
				len(h) < 8.0 && p.z > m.z - 4.0 && p.z < m.z + 1.5
			})
		};
		// the floor: the lowest 1 % of the devices' heights, a tracker's height below
		let mut zs: Vec<f32> = d
			.devs
			.iter()
			.flat_map(|x| (0..x.replay.len()).step_by(25).map(|i| x.replay.pos(i)))
			.filter(|p| plausible(*p))
			.map(|p| p.z as f32)
			.collect();
		zs.sort_by(f32::total_cmp);
		self.rp.floor = zs.get(zs.len() / 100).map(|z| *z as f64 - 0.03).unwrap_or(0.0);
		// the graph's height: most of the distances, not the largest
		self.rp.ymax = d
			.devs
			.iter()
			.map(|x| {
				let mut v: Vec<f32> = x.err.iter().flat_map(|e| e.iter().map(|q| q.1)).collect();
				v.sort_by(f32::total_cmp);
				let p = v.get(v.len() * 995 / 1000).copied().unwrap_or(0.02);
				(p as f64 * 1.2).clamp(0.01, 0.5)
			})
			.collect();
		self.rp.dev = self.rp.dev.min(d.devs.len().saturating_sub(1));
		// the camera on where the devices were (the stations may be out of the view: the devices are what is looked at)
		let pts: Vec<V3> = d
			.devs
			.iter()
			.filter(|x| !x.replay.is_empty())
			.flat_map(|x| (1..10).map(|k| x.replay.pos(x.replay.len() * k / 10)))
			.filter(|p| len(*p) < 50.0 && plausible(*p))
			.collect();
		if !pts.is_empty() {
			let (mut lo, mut hi) = (pts[0], pts[0]);
			for p in &pts {
				lo = V3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
				hi = V3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
			}
			let mut t = (lo + hi) * 0.5;
			t.z = (t.z + self.rp.floor) * 0.5;
			self.rp.cam.target = t;
			self.rp.cam.dist = (len(hi - lo) * 1.6).clamp(1.8, 8.0);
		}
		self.rp.view = (0.0, d.dur.max(1.0));
		self.rp.data = Some(d);
	}

	pub(super) fn replay_tab(&mut self, ui: &mut egui::Ui, r: Rect) {
		let now = self.now;
		let dt = (now - self.rp.last_now).clamp(0.0, 0.25);
		self.rp.last_now = now;
		self.replay_poll();
		if self.rp.sessions.is_empty() && now - self.rp.scanned > 10.0 {
			self.replay_scan();
		}
		if self.replay_busy() || self.rp.playing {
			ui.ctx().request_repaint();
		}
		let dur = self.rp.data.as_ref().map(|d| d.dur).unwrap_or(0.0);
		if self.rp.playing && !self.rp.scrubbing {
			self.rp.t += dt * self.rp.speed;
			if self.rp.t >= dur {
				self.rp.t = dur;
				self.rp.playing = false;
			}
		}
		if self.view == View::Window && self.rp.data.is_some() {
			let (space, left, right, shift) = ui.input(|i| {
				(
					i.key_pressed(Key::Space),
					i.key_pressed(Key::ArrowLeft),
					i.key_pressed(Key::ArrowRight),
					i.modifiers.shift,
				)
			});
			if space {
				if self.rp.t >= dur {
					self.rp.t = 0.0;
				}
				self.rp.playing = !self.rp.playing;
				if self.rp.playing {
					self.show_cursor();
				}
			}
			let step = if shift { 0.1 } else { 1.0 };
			if left {
				self.rp.t = (self.rp.t - step).max(0.0);
			}
			if right {
				self.rp.t = (self.rp.t + step).min(dur);
			}
		}

		let m = 10.0;
		let head_h = 46.0;
		let opened = self.replay_header(ui, Rect::from_min_size(r.min, Vec2::new(r.width(), head_h)));
		let area = Rect::from_min_max(Pos2::new(r.min.x, r.min.y + head_h + m), r.max);
		let data = self.rp.data.take();
		match &data {
			Some(d) => {
				// the list as wide as the Tracking tab's right column
				let list_w = (area.width() * 0.262).clamp(352.0, 414.0);
				let tl_h = 250.0f32.max(area.height() * 0.3);
				let room = Rect::from_min_max(area.min, Pos2::new(area.max.x - list_w - m, area.max.y - tl_h - m));
				let list = Rect::from_min_max(Pos2::new(room.max.x + m, area.min.y), Pos2::new(area.max.x, room.max.y));
				let tl = Rect::from_min_max(Pos2::new(area.min.x, area.max.y - tl_h), area.max);
				self.replay_room(ui, room, d);
				self.replay_devices(ui, list, d);
				self.replay_timeline(ui, tl, d);
			}
			None => {
				let p = ui.painter().clone();
				self.surface(&p, area, Surface::Well);
				let (text, col) = if !self.rp.error.is_empty() {
					(self.rp.error.clone(), theme::tk().bad)
				} else if !self.rp.status.is_empty() {
					(format!("{}...", self.rp.status), theme::tk().muted)
				} else if self.rp.sessions.is_empty() {
					(format!("no sessions in {}", self.replay_paths().logs.display()), theme::tk().muted)
				} else {
					("Pick a session to replay it with this build".into(), theme::tk().muted)
				};
				self.text_at(&p, area.center(), Align2::CENTER_CENTER, body(), col, &text);
				if let Some(j) = &self.rp.job {
					if j.dur > 0.0 {
						let w = 360.0f32.min(area.width() - 40.0);
						let mr = Rect::from_center_size(area.center() + Vec2::new(0.0, 30.0), Vec2::new(w, 6.0));
						super::kit::meter(&p, mr, (j.at / j.dur).clamp(0.0, 1.0), theme::tk().good);
					}
				}
			}
		}
		self.rp.data = data;
		if self.rp.picking {
			self.replay_picker(ui, area, opened);
		}
	}

	/// The session's line: the picker's button, Replay again, what is going on; whether the picker opened now
	fn replay_header(&mut self, ui: &mut egui::Ui, r: Rect) -> bool {
		let t = theme::tk();
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Card);
		let h = 32.0;
		let y = r.center().y - h * 0.5;
		let says = match &self.rp.session {
			Some(s) => format!(
				"{}{}   {}",
				stamp_text(&s.stamp),
				if s.check.is_some() { "  sally-check" } else { "" },
				dur_text(s.duration())
			),
			None => "Pick a session".into(),
		};
		// the picker's button: the session, a caret
		let sw = kit::text_w(&p, &says, body()) + 54.0;
		let sr = Rect::from_min_size(Pos2::new(r.min.x + 10.0, y), Vec2::new(sw, h));
		let look = if self.rp.session.is_none() && !self.rp.picking { kit::Look::Primary } else { kit::Look::Plain };
		let clicked = kit::button(ui, Id::unique("rp_pick"), sr, "", None, look, true).clicked();
		let ink = if look == kit::Look::Primary { t.on_ink } else { t.text };
		p.text(Pos2::new(sr.min.x + 14.0, sr.center().y), Align2::LEFT_CENTER, &says, body(), ink);
		kit::icon_at(
			&p,
			Pos2::new(sr.max.x - 18.0, sr.center().y),
			if self.rp.picking { icons::CARET_UP } else { icons::CARET_DOWN },
			14.0,
			ink,
		);
		if clicked {
			self.rp.picking = !self.rp.picking;
			if self.rp.picking {
				self.replay_scan();
				self.rp.pick_scroll = 0.0;
			}
		}
		let mut x = sr.max.x + 8.0;
		if let Some(s) = self.rp.session.clone() {
			if !self.replay_busy() {
				let bw = kit::button_w(&p, "Replay again", true);
				let br = Rect::from_min_size(Pos2::new(x, y), Vec2::new(bw, h));
				let resp = kit::button(
					ui,
					Id::unique("rp_again"),
					br,
					"Replay again",
					Some(icons::ARROWS_CLOCKWISE),
					kit::Look::Ghost,
					true,
				)
				.on_hover_text("Runs the session through this build again");
				if resp.clicked() {
					self.replay_open(&s.key(), true);
				}
				x = br.max.x + 4.0;
			}
			// out of the session: what it loaded let go, a replay still running stopped
			let bw = kit::button_w(&p, "Close", true);
			let br = Rect::from_min_size(Pos2::new(x, y), Vec2::new(bw, h));
			if kit::button(ui, Id::unique("rp_close"), br, "Close", Some(icons::X), kit::Look::Ghost, true).clicked() {
				self.replay_close();
				return false;
			}
			x = br.max.x + 16.0;
			let mut info = Vec::new();
			if !self.rp.error.is_empty() {
				info.push(self.rp.error.clone());
			}
			if !self.rp.status.is_empty() {
				info.push(self.rp.status.clone());
			}
			if let Some(d) = &self.rp.data {
				info.extend(d.notes.iter().cloned());
				if d.notes.is_empty() {
					info.push(format!(
						"{} devices, {} stations, {} events",
						d.devs.len(),
						d.stations.len(),
						d.events.len()
					));
				}
			}
			let col = if self.rp.error.is_empty() { t.muted } else { t.bad };
			let clip = Rect::from_min_max(Pos2::new(x, r.min.y), Pos2::new(r.max.x - 10.0, r.max.y));
			kit::text_in(&p, clip, &info.join(";  "), theme::label(), col);
		}
		clicked && self.rp.picking
	}

	/// The session list, over the view: newest first
	fn replay_picker(&mut self, ui: &mut egui::Ui, r: Rect, opened: bool) {
		let w = 760.0f32.min(r.width() - 20.0);
		let row_h = 34.0;
		let n = self.rp.sessions.len();
		let foot = 30.0;
		let hh = (n.max(1) as f32 * row_h + 20.0 + foot).min(r.height() - 10.0).min(640.0);
		let rr = Rect::from_min_size(r.min + Vec2::new(10.0, 0.0), Vec2::new(w, hh));
		let p = ui.painter().clone();
		p.rect_filled(rr, 8.0, theme::tk().float);
		p.rect_stroke(rr, 8.0, Stroke::new(1.0, theme::tk().card_edge), StrokeKind::Inside);
		let inner = Rect::from_min_max(rr.min + Vec2::new(10.0, 10.0), rr.max - Vec2::new(10.0, 10.0 + foot));
		let resp = ui.interact(rr, Id::unique("rp_picker"), Sense::click());
		// the replays' room on the disk: each is kept for as long as its session's recording is
		let used: u64 = self.rp.sessions.iter().map(|s| s.replay_bytes).sum();
		let foot_t = format!("replays kept in {}: {:.1} GB", self.replay_paths().replays.display(), used as f64 / 1e9);
		let fy = rr.max.y - foot * 0.5 - 4.0;
		self.text_at(
			&p.with_clip_rect(rr),
			Pos2::new(rr.min.x + 20.0, fy),
			Align2::LEFT_CENTER,
			theme::label(),
			theme::tk().faint,
			&foot_t,
		);
		if resp.hovered() {
			let wheel = ui.input(|i| i.smooth_scroll_delta.y);
			self.rp.pick_scroll -= wheel;
		}
		let max_scroll = (n as f32 * row_h - inner.height()).max(0.0);
		self.rp.pick_scroll = self.rp.pick_scroll.clamp(0.0, max_scroll);
		let pc = p.with_clip_rect(inner);
		let mut chosen = None;
		let hover = resp.hover_pos();
		for (i, s) in self.rp.sessions.iter().enumerate() {
			let y = inner.min.y + i as f32 * row_h - self.rp.pick_scroll;
			if y + row_h < inner.min.y || y > inner.max.y {
				continue;
			}
			let row = Rect::from_min_size(Pos2::new(inner.min.x, y), Vec2::new(inner.width(), row_h));
			if hover.is_some_and(|q| row.contains(q) && inner.contains(q)) {
				pc.rect_filled(row, 6.0, white(16));
				if resp.clicked() {
					chosen = Some(s.key());
				}
			} else if self.rp.session.as_ref().is_some_and(|c| c.key() == s.key()) {
				pc.rect_filled(row, 6.0, white(8));
			}
			let cy = row.center().y;
			let x0 = row.min.x + 10.0;
			self.text_at(&pc, Pos2::new(x0, cy), Align2::LEFT_CENTER, body(), theme::tk().text, &stamp_text(&s.stamp));
			self.text_at(
				&pc,
				Pos2::new(x0 + 170.0, cy),
				Align2::LEFT_CENTER,
				theme::label(),
				theme::tk().soft,
				&dur_text(s.duration()),
			);
			let mb = format!("{:.0} MB", s.bytes as f64 / 1e6);
			self.text_at(&pc, Pos2::new(x0 + 250.0, cy), Align2::LEFT_CENTER, theme::label(), theme::tk().muted, &mb);
			let (t, c) = match (&s.check, &s.poselog) {
				(Some(_), _) => ("sally-check recording", theme::tk().violet),
				(None, Some(_)) => ("with Valve's poses", theme::tk().note),
				(None, None) => ("no pose log", theme::tk().faint),
			};
			self.text_at(&pc, Pos2::new(x0 + 340.0, cy), Align2::LEFT_CENTER, theme::label(), c, t);
			let (t, c) = match s.replayed.as_deref() {
				Some(d) if d.starts_with("exit 0") => {
					(format!("replayed  {:.0} MB", s.replay_bytes as f64 / 1e6), theme::tk().good)
				}
				Some(_) => ("replay failed".into(), theme::tk().bad),
				None => (String::new(), theme::tk().text),
			};
			self.text_at(&pc, Pos2::new(row.max.x - 10.0, cy), Align2::RIGHT_CENTER, theme::label(), c, &t);
		}
		if n == 0 {
			let t = format!("no sallytap_* recording in {}", self.replay_paths().logs.display());
			self.text_at(&p, inner.center(), Align2::CENTER_CENTER, theme::label(), theme::tk().muted, &t);
		}
		if let Some(s) = chosen {
			self.replay_open(&s, false);
		} else if resp.clicked_elsewhere() && !opened {
			self.rp.picking = false;
		}
	}

	fn replay_room(&mut self, ui: &mut egui::Ui, r: Rect, d: &Data) {
		self.card(ui, r, "Room");
		// what is shown, in the header line (a second row where they do not fit): the three tracks by their colours,
		// then the layers
		let tracks = self.st.replay_tracks;
		let mut items: Vec<(&str, bool, Option<Color32>)> =
			self::tracks().iter().zip(tracks).map(|(t, s)| (t.0, s, Some(t.1))).collect();
		items.extend([
			("Light", self.st.show_rays, None),
			("Models", self.st.show_models, None),
			("Trails", self.st.show_trails, None),
			("Labels", self.st.show_labels, None),
			("Follow", self.rp.follow, None),
		]);
		let (clicked, bottom) = kit::chips(
			ui,
			Id::unique("rp_pills"),
			Pos2::new(r.min.x + 76.0, r.min.y + 6.0),
			r.max.x - 12.0,
			28.0,
			&items,
		);
		if let Some(i) = clicked {
			match i {
				0..=2 => self.st.replay_tracks[i] = !self.st.replay_tracks[i],
				3 => self.st.show_rays = !self.st.show_rays,
				4 => self.st.show_models = !self.st.show_models,
				5 => self.st.show_trails = !self.st.show_trails,
				6 => self.st.show_labels = !self.st.show_labels,
				_ => self.rp.follow = !self.rp.follow,
			}
			self.st.dirty = true;
		}
		let y0 = (bottom.y + 6.0).max(r.min.y + HEADER_H);
		let canvas = Rect::from_min_max(
			Pos2::new(r.min.x + 12.0, y0),
			Pos2::new(r.max.x - 12.0, (r.max.y - 12.0).max(y0 + 20.0)),
		);
		let resp = ui.interact(canvas, Id::unique("rp_room"), Sense::click_and_drag());
		if camera_input(&mut self.rp.cam, ui, &resp, canvas) {
			self.rp.follow = false;
		}
		let t = self.rp.t;
		let sel = self.rp.dev;
		let tracks = self.st.replay_tracks;
		let prim = primary(tracks);
		if self.rp.follow {
			if let Some(pose) = d.devs.get(sel).and_then(|x| replay::model_pose(x, prim, t)) {
				self.rp.cam.target = self.rp.cam.target * 0.85 + pose.p * 0.15;
				ui.ctx().request_repaint();
			}
		}
		let cam = Cam3::new(&self.rp.cam, canvas);
		let p = ui.painter_at(canvas);
		self.surface(&p, canvas, Surface::Well);
		let (major, minor) = self.grid_colors();
		cam.grid(&p, grid_squares(d.stations.iter().map(|s| s.1.p), 0.25), 0.25, self.rp.floor, major, minor);
		// the main track: the primary where one is chosen and shown, else the replay, else what SteamVR got
		let main = self.rp.primary.filter(|k| tracks[*k]).unwrap_or(prim);
		// a track's look: solid, or ghosted beside the primary
		let ghosted = |k: usize| self.rp.primary.is_some_and(|pk| pk != k && tracks[pk]);
		let light_on = self.st.show_rays;
		let sel_station = self.rp.station.filter(|c| d.stations.iter().any(|s| s.0 == *c));

		// the light at the cursor on the selected device: per station the sensors it lit (both planes, either)
		let lit: Vec<(u8, u64, u64)> = match d.devs.get(sel) {
			Some(x) if light_on => x.light.lit(t, LIGHT_WINDOW),
			_ => Vec::new(),
		};
		let lit_by = |ch: i32| lit.iter().find(|l| l.0 as i32 == ch);
		// the labels of what is drawn, placed once all of it is (kept off each other, as the live room's)
		let mut tags: Vec<Tag> = Vec::new();
		// the stations: white where they lit the device, grey where not; with one chosen, the others dimmer
		let mut station_picks: Vec<(Pos2, i32)> = Vec::new();
		for (idx, pose) in &d.stations {
			let lights = lit_by(*idx).is_some_and(|l| l.2 != 0);
			let col = if !light_on {
				Color32::from_rgb(230, 233, 245)
			} else if sel_station == Some(*idx) {
				LIT
			} else if sel_station.is_some() {
				alpha(UNLIT, 140)
			} else if lights {
				LIT
			} else {
				UNLIT
			};
			cam.station(&p, pose, col);
			if let Some(q) = cam.pt(pose.p) {
				station_picks.push((q, *idx));
				if sel_station == Some(*idx) {
					p.circle_stroke(q, 14.0, Stroke::new(1.6, LIT));
				}
				if self.st.show_labels {
					// by its channel, as the Tracking tab names it (the replay's index is the channel less one)
					tags.push(Tag {
						at: q,
						gap: 12.0,
						text: format!("ch {}", idx + 1),
						font: theme::label(),
						color: alpha(col, 230),
						rank: 1,
					});
				}
			}
		}
		// the chosen station's rays to the sensors it lit (both of its planes bright, one faint; a sensor facing away
		// from the camera fainter still)
		let sensors: Vec<(V3, V3)> = match d.devs.get(sel).and_then(|x| x.replay.at(t).map(|h| (x, h))) {
			Some((x, head)) if light_on => x.sensors_at(&head),
			_ => Vec::new(),
		};
		let facing = |s: &(V3, V3)| len(s.1) < 0.5 || dot(s.1, cam.eye() - s.0) > 0.0;
		if let (Some(ch), Some(st)) = (sel_station, sel_station.and_then(|c| station_pose(d, c as u8))) {
			if let Some(&(_, both, either)) = lit_by(ch) {
				for (k, s) in sensors.iter().enumerate().filter(|(k, _)| either >> k & 1 != 0) {
					let a = match (both >> k & 1 != 0, facing(s)) {
						(true, true) => 170,
						(true, false) | (false, true) => 80,
						(false, false) => 40,
					};
					cam.line(&p, st.p, s.0, alpha(LIT, a), 1.2);
				}
			}
		}

		// the trails: the selected device's on every track shown, the others' on the main one
		if self.st.show_trails {
			let ts = self.st.trail_seconds.clamp(0.5, 8.0);
			for (i, x) in d.devs.iter().enumerate() {
				for (k, tr) in [&x.valve, &x.live, &x.replay].into_iter().enumerate() {
					if !tracks[k] || (i != sel && k != main) {
						continue;
					}
					let (a, b) = tr.span(t - ts, t);
					// the selected device's every sample (the track's 240 a second), the others' at most 90 a second as
					// the live trails are. Picked by the clock, not every n-th sample of the window: a hand's shake
					// drawn from a sample in 6 or 16 (40 / 15 a second) was chords across it, a different set every
					// frame as the window moved
					let rate = if i == sel { 240.0 } else { 90.0 };
					let mut bucket = i64::MIN;
					let base = match (i == sel, ghosted(k)) {
						(true, false) => 230.0,
						(true, true) => 80.0,
						(false, _) => 110.0,
					};
					let mut last: Option<(V3, f32)> = None;
					for j in a..b {
						let bj = (tr.t[j] as f64 * rate).floor() as i64;
						if bj == bucket {
							continue;
						}
						bucket = bj;
						let q = tr.pos(j);
						if let Some((lp, lt)) = last {
							if len(q - lp) < 0.5 && tr.t[j] - lt < replay::GAP_S {
								let age = ((t - tr.t[j] as f64) / ts).clamp(0.0, 1.0);
								let al = (base * (1.0 - age)) as i32;
								let col = if i == sel { self::tracks()[k].1 } else { OTHERS };
								cam.line(&p, lp, q, alpha(col, al), if i == sel && !ghosted(k) { 1.8 } else { 1.2 });
							}
						}
						last = Some((q, tr.t[j]));
					}
				}
			}
		}

		// the models, far ones first: the selected device on every track shown (the primary solid, the others ghosted),
		// the others on the main one
		let mut draw: Vec<(f64, usize, usize, Pose)> = Vec::new();
		for (i, x) in d.devs.iter().enumerate() {
			for (k, on) in tracks.into_iter().enumerate() {
				if !on || (i != sel && k != main) {
					continue;
				}
				if let Some(pose) = replay::model_pose(x, k, t) {
					draw.push((dot(pose.p - cam.eye(), cam.forward()), i, k, pose));
				}
			}
		}
		// the ghosts first, under the solid ones where they are in the same place, then far ones first
		let is_ghost = |i: usize, k: usize| i == sel && ghosted(k);
		draw.sort_by(|a, b| is_ghost(b.1, b.2).cmp(&is_ghost(a.1, a.2)).then(b.0.total_cmp(&a.0)));
		let mut picks: Vec<(Pos2, usize)> = Vec::new();
		for (_, i, k, pose) in &draw {
			let x = &d.devs[*i];
			let col = if *i == sel { self::tracks()[*k].1 } else { OTHERS };
			let a: u8 = if *i == sel && ghosted(*k) { GHOST } else { 255 };
			let mut name = x.model.clone();
			if name.is_empty() {
				name = model_name_for("tracker").into();
			}
			let model = if self.st.show_models { self.models.get(&name) } else { None };
			match model {
				Some(m) if a < 255 => cam.model_ghost(&p, &m, pose, col, a),
				Some(m) => cam.model(&p, &m, pose, col, a),
				None => {
					if let Some(q) = cam.pt(pose.p) {
						p.circle_filled(q, 6.0, alpha(col, a as i32));
					}
				}
			}
			if *i == sel && *k == main {
				let al = 0.06;
				for (v, c) in
					[(V3::new(al, 0.0, 0.0), AXIS_X), (V3::new(0.0, al, 0.0), AXIS_Y), (V3::new(0.0, 0.0, al), AXIS_Z)]
				{
					cam.line(&p, pose.p, xform(pose, v), alpha(c, 200), 1.6);
				}
			}
			if *k == main {
				if let Some(q) = cam.pt(pose.p) {
					picks.push((q, *i));
					if self.st.show_labels {
						let (lbl, c) = if *i == sel {
							(self.rp_name(x), theme::tk().text)
						} else {
							(self.rp_short(x), theme::tk().soft)
						};
						tags.push(Tag {
							at: q,
							gap: 14.0,
							text: lbl,
							font: theme::label(),
							color: alpha(c, if *i == sel { 230 } else { 170 }),
							rank: if *i == sel { 0 } else { 2 },
						});
					}
				}
			}
		}
		// the selected device's sensors over its models: white where lit (by the chosen station, else by any), grey
		// where not; facing away from the camera smaller and fainter
		let mask = match sel_station {
			Some(ch) => lit_by(ch).map(|l| l.2).unwrap_or(0),
			None => lit.iter().fold(0, |m, l| m | l.2),
		};
		for (k, s) in sensors.iter().enumerate() {
			let Some((q, depth)) = cam.proj(s.0) else { continue };
			let on = mask >> k & 1 != 0;
			let front = facing(s);
			let rad = (0.006 * cam.focal / depth).clamp(1.5, 4.5) as f32 * if front { 1.0 } else { 0.7 };
			let col = match (on, front) {
				(true, true) => LIT,
				(true, false) => alpha(LIT, 110),
				(false, true) => alpha(UNLIT, 230),
				(false, false) => alpha(UNLIT, 90),
			};
			p.circle_filled(q, rad, col);
		}

		// the bottom bar: the primary track (a click picks it, again leaves all solid), and the stations with the
		// sensors each lit (a click shows its light alone, again all)
		let bh = 24.0;
		let by = canvas.max.y - bh - 8.0;
		// the labels over the view, clear of the bar
		draw_tags(&p, tags, Rect::from_min_max(canvas.min + Vec2::splat(4.0), Pos2::new(canvas.max.x - 4.0, by - 8.0)));
		// a shade under it, so that what is behind does not run through its text
		let shade_r = Rect::from_min_max(Pos2::new(canvas.min.x, by - 6.0), canvas.max);
		p.rect_filled(shade_r, CornerRadius { nw: 0, ne: 0, sw: 6, se: 6 }, Color32::from_black_alpha(120));
		let mut lx = canvas.min.x + 10.0;
		let tr = self.text_at(
			&p,
			Pos2::new(lx, by + bh * 0.5),
			Align2::LEFT_CENTER,
			theme::label(),
			theme::tk().muted,
			"solid",
		);
		lx = tr.max.x + 8.0;
		for (k, (label, col)) in self::tracks().iter().enumerate() {
			if !tracks[k] {
				continue;
			}
			let on = self.rp.primary == Some(k);
			let (pr, c) =
				self.pill_toggle(ui, Id::unique(("rp_primary", k)), Pos2::new(lx, by), bh, &format!("    {label}"), on);
			p.circle_filled(Pos2::new(pr.min.x + 12.0, pr.center().y), 4.0, *col);
			if c {
				self.rp.primary = if on { None } else { Some(k) };
			}
			lx = pr.max.x + 5.0;
		}
		if light_on && sel < d.devs.len() {
			lx += 12.0;
			let x = &d.devs[sel];
			let tr = self.text_at(
				&p,
				Pos2::new(lx, by + bh * 0.5),
				Align2::LEFT_CENTER,
				theme::label(),
				theme::tk().muted,
				"light",
			);
			lx = tr.max.x + 8.0;
			if x.light.is_empty() {
				let t = "none recorded: Replay again";
				self.text_at(
					&p,
					Pos2::new(lx, by + bh * 0.5),
					Align2::LEFT_CENTER,
					theme::label(),
					theme::tk().faint,
					t,
				);
			} else {
				for (idx, _) in &d.stations {
					let n = lit_by(*idx).map_or(0, |l| l.2.count_ones());
					let on = sel_station == Some(*idx);
					let lbl = format!("ch {}: {n}", idx + 1);
					let (pr, c) =
						self.pill_toggle(ui, Id::unique(("rp_station", *idx)), Pos2::new(lx, by), bh, &lbl, on);
					if n == 0 && !on {
						// a station that did not light it: its chip dimmed
						p.rect_filled(
							pr,
							CornerRadius::same(theme::tk().ctl_radius as u8),
							Color32::from_black_alpha(90),
						);
					}
					if c {
						self.rp.station = if on { None } else { Some(*idx) };
					}
					lx = pr.max.x + 5.0;
				}
				if lit.is_empty() {
					self.text_at(
						&p,
						Pos2::new(lx + 6.0, by + bh * 0.5),
						Align2::LEFT_CENTER,
						theme::label(),
						theme::tk().bad,
						"dark",
					);
				}
			}
		}
		let ly = by + bh * 0.5;
		let tt = format!("{} / {}", dur_text(t), dur_text(d.dur));
		self.text_at(
			&p,
			Pos2::new(canvas.max.x - 10.0, ly),
			Align2::RIGHT_CENTER,
			theme::label(),
			theme::tk().muted,
			&tt,
		);
		if d.devs.is_empty() {
			self.text_at(
				&p,
				canvas.center(),
				Align2::CENTER_CENTER,
				body(),
				theme::tk().muted,
				"the replay tracked no device",
			);
		}

		if resp.clicked() {
			if let Some(mp) = resp.interact_pointer_pos() {
				let best = picks
					.iter()
					.map(|(q, i)| (q.distance(mp), *i))
					.filter(|(dd, _)| *dd < 30.0)
					.min_by(|a, b| a.0.total_cmp(&b.0));
				// a station: its light alone (again: all)
				let st = station_picks
					.iter()
					.map(|(q, c)| (q.distance(mp), *c))
					.filter(|(dd, _)| *dd < 22.0)
					.min_by(|a, b| a.0.total_cmp(&b.0));
				match (best, st) {
					(_, Some((ds, c))) if best.is_none_or(|b| ds < b.0) => {
						self.rp.station = if self.rp.station == Some(c) { None } else { Some(c) };
					}
					(Some((_, i)), _) => self.rp.dev = i,
					_ => {}
				}
			}
		}
		if resp.double_clicked() {
			self.rp.follow = true;
		}
	}

	/// A device's name in the replay: the panel's name for its serial, else its type and the end of its serial
	fn rp_name(&self, x: &Dev) -> String {
		match self.st.names.get(&x.serial) {
			Some(n) => format!("{n}  {}", x.codename),
			None => {
				let tail = x.serial.char_indices().nth_back(3).map_or(x.serial.as_str(), |(i, _)| &x.serial[i..]);
				let kind = if x.kind.is_empty() { "Device" } else { x.kind.trim_end_matches(" MV") };
				format!("{kind} {tail}  {}", x.codename)
			}
		}
	}

	/// A device's short name in the room (the devices not selected): the panel's name, else the end of its serial,
	/// and its codename
	fn rp_short(&self, x: &Dev) -> String {
		match self.st.names.get(&x.serial) {
			Some(n) => format!("{n} {}", x.codename),
			None => {
				let tail = x.serial.char_indices().nth_back(3).map_or(x.serial.as_str(), |(i, _)| &x.serial[i..]);
				format!("{tail} {}", x.codename)
			}
		}
	}

	fn replay_devices(&mut self, ui: &mut egui::Ui, r: Rect, d: &Data) {
		let inner = self.boxed(ui, r, "Devices", "from Valve", None);
		let p = ui.painter().clone();
		let tk = theme::tk();
		// the numbers in columns, each to its right edge: the median, 9 in 10, the times over 3 cm
		let cols = [inner.max.x - 138.0, inner.max.x - 82.0, inner.max.x - 10.0];
		for (cx, head) in cols.iter().zip(["median", "9 in 10", "over 3 cm"]) {
			kit::text(&p, Pos2::new(*cx, inner.min.y + 6.0), Align2::RIGHT_CENTER, head, theme::caption(), tk.faint);
		}
		let top = inner.min + Vec2::new(0.0, 18.0);
		let row_h = 70.0;
		let resp = ui.interact(inner, Id::unique("rp_devs"), Sense::click());
		let t = self.rp.t;
		let any_valve = d.devs.iter().any(|x| !x.valve.is_empty());
		for (i, x) in d.devs.iter().enumerate() {
			let row =
				Rect::from_min_size(top + Vec2::new(0.0, i as f32 * row_h), Vec2::new(inner.width(), row_h - 6.0));
			if row.max.y > inner.max.y + 4.0 {
				let more = format!("{} more", d.devs.len() - i);
				self.text(&p, Pos2::new(inner.min.x + 8.0, row.min.y), theme::label(), theme::tk().faint, &more);
				break;
			}
			let sel = i == self.rp.dev;
			let hov = resp.hover_pos().is_some_and(|q| row.contains(q));
			if sel {
				p.rect_filled(row, 6.0, white(18));
			} else if hov {
				p.rect_filled(row, 6.0, white(9));
			}
			if resp.clicked() && resp.interact_pointer_pos().is_some_and(|q| row.contains(q)) {
				// the view to it: followed, from 1.6 m at most
				self.rp.dev = i;
				self.rp.follow = true;
				self.rp.cam.dist = self.rp.cam.dist.min(1.6);
			}
			let name = self.rp_name(x);
			let clip = p.with_clip_rect(row);
			let sym = Rect::from_min_size(row.min + Vec2::new(6.0, 5.0), Vec2::new(26.0, 24.0));
			kit::symbol(
				&p,
				sym,
				self.symbol_of(&x.serial),
				icons::HEXAGON,
				if sel { theme::tk().text } else { theme::tk().soft },
			);
			self.text(
				&clip,
				row.min + Vec2::new(40.0, 6.0),
				body(),
				if sel { theme::tk().text } else { theme::tk().soft },
				&name,
			);
			// whether the replay has a pose at the cursor
			let here = x.replay.at(t).is_some();
			p.circle_filled(
				Pos2::new(row.max.x - 12.0, row.min.y + 15.0),
				4.0,
				if here { theme::tk().good } else { theme::tk().faint },
			);
			let mut y = row.min.y + 32.0;
			let mut any = false;
			for (k, label) in [(0usize, "SteamVR got"), (1, "Replay")] {
				let (med, p90, _, spans) = x.stats[k];
				if med < 0.0 {
					continue;
				}
				any = true;
				p.circle_filled(Pos2::new(row.min.x + 46.0, y + 8.0), 3.5, self::tracks()[k + 1].1);
				self.text(&clip, Pos2::new(row.min.x + 56.0, y), theme::label(), tk.muted, label);
				for (cx, s) in cols.iter().zip([mm(med), mm(p90)]) {
					kit::text(&clip, Pos2::new(*cx, y), Align2::RIGHT_TOP, &s, theme::label(), tk.text);
				}
				if spans > 0 {
					let s = format!("{spans}x");
					kit::text(&clip, Pos2::new(cols[2], y), Align2::RIGHT_TOP, &s, theme::label(), tk.warn);
				}
				y += 16.0;
			}
			if !any {
				let tx = if any_valve { "no Valve poses for it" } else { "no Valve poses (no pose log)" };
				self.text(&clip, Pos2::new(row.min.x + 40.0, y), theme::label(), theme::tk().faint, tx);
			}
		}
	}

	fn replay_timeline(&mut self, ui: &mut egui::Ui, r: Rect, d: &Data) {
		let p = ui.painter().clone();
		self.surface(&p, r, Surface::Card);
		let x = d.devs.get(self.rp.dev);
		let t_now = self.rp.t;

		// the controls
		let h = 30.0;
		let cy = r.min.y + 10.0;
		let mut cx = r.min.x + 12.0;
		let pr = Rect::from_min_size(Pos2::new(cx, cy), Vec2::new(44.0, h));
		let glyph = if self.rp.playing { icons::PAUSE } else { icons::PLAY };
		let resp = kit::button(ui, Id::unique("rp_play"), pr, "", None, kit::Look::Primary, true)
			.on_hover_text(if self.rp.playing { "Pause (the space bar)" } else { "Play (the space bar)" });
		p.text(pr.center(), Align2::CENTER_CENTER, glyph, icon_fill(16.0), theme::tk().on_ink);
		if resp.clicked() {
			if self.rp.t >= d.dur {
				self.rp.t = 0.0;
			}
			self.rp.playing = !self.rp.playing;
			if self.rp.playing {
				self.show_cursor();
			}
		}
		cx = pr.max.x + 10.0;
		let labels: Vec<String> = SPEEDS.iter().map(|s| format!("{s}x")).collect();
		let items: Vec<&str> = labels.iter().map(String::as_str).collect();
		let at = SPEEDS.iter().position(|s| *s == self.rp.speed).unwrap_or(0);
		let (sr, c) = kit::segmented(ui, Id::unique("rp_speed"), Pos2::new(cx, cy), h, &items, at);
		if let Some(i) = c {
			self.rp.speed = SPEEDS[i];
		}
		cx = sr.max.x + 10.0;
		// the selected device's red events (lost, the state wrong): to the one before or after the cursor, a second early
		let evs: Vec<&replay::Event> =
			d.events.iter().filter(|e| x.is_some_and(|x| x.codename == e.dev) && e.level >= 2).collect();
		let br = Rect::from_min_size(Pos2::new(cx, cy), Vec2::splat(h));
		let prev = kit::icon_button(ui, Id::unique("rp_prev"), br, icons::SKIP_BACK, false, !evs.is_empty())
			.on_hover_text("To the device's event before this");
		if prev.clicked() {
			if let Some(e) = evs.iter().rev().find(|e| (e.t as f64) < self.rp.t + 0.9) {
				self.rp.t = (e.t as f64 - 1.0).max(0.0);
				self.rp.playing = false;
				self.show_cursor();
			}
		}
		let nr = Rect::from_min_size(Pos2::new(br.max.x + 2.0, cy), Vec2::splat(h));
		let next = kit::icon_button(ui, Id::unique("rp_next"), nr, icons::SKIP_FORWARD, false, !evs.is_empty())
			.on_hover_text("To the device's next event");
		if next.clicked() {
			if let Some(e) = evs.iter().find(|e| (e.t as f64) > self.rp.t + 1.1) {
				self.rp.t = (e.t as f64 - 1.0).max(0.0);
				self.rp.playing = false;
				self.show_cursor();
			}
		}
		let whole = self.rp.view.0 <= 0.0 && self.rp.view.1 >= d.dur.max(1.0) - 1e-6;
		let zw = kit::button_w(&p, "Whole session", true);
		let zr = Rect::from_min_size(Pos2::new(nr.max.x + 8.0, cy), Vec2::new(zw, h));
		if kit::button(
			ui,
			Id::unique("rp_zoom"),
			zr,
			"Whole session",
			Some(icons::ARROWS_HORIZONTAL),
			kit::Look::Ghost,
			!whole,
		)
		.clicked()
		{
			self.rp.view = (0.0, d.dur.max(1.0));
		}
		// the time and, for the selected device, the distances at the cursor
		let ty0 = cy + h * 0.5;
		let tr = self.text_at(
			&p,
			Pos2::new(r.max.x - 14.0, ty0),
			Align2::RIGHT_CENTER,
			body(),
			theme::tk().text,
			&format!("{t_now:.2} s"),
		);
		let mut right = tr.min.x - 20.0;
		if let Some(x) = x {
			for k in [1usize, 0] {
				if let Some(e) = err_at(&x.err[k], t_now) {
					let lbl = format!("{} {}", self::tracks()[k + 1].0, mm(e));
					let tr = self.text_at(
						&p,
						Pos2::new(right, ty0),
						Align2::RIGHT_CENTER,
						theme::label(),
						self::tracks()[k + 1].1,
						&lbl,
					);
					right = tr.min.x - 16.0;
				}
			}
		}

		// the selected device's summary
		let sy = cy + h + 10.0;
		let mut sx = r.min.x + 14.0;
		if let Some(x) = x {
			let tr = self.text(&p, Pos2::new(sx, sy), theme::label(), theme::tk().soft, &self.rp_name(x));
			sx = tr.max.x + 24.0;
			for (k, label) in [(0usize, "SteamVR got"), (1, "replay")] {
				let (med, p90, max, spans) = x.stats[k];
				if med < 0.0 {
					continue;
				}
				let t = format!(
					"{label}: {} median, {} at 90 %, {} at most, {spans}x over 3 cm",
					mm(med),
					mm(p90),
					mm(max)
				);
				let tr = self.text(&p, Pos2::new(sx, sy), theme::label(), self::tracks()[k + 1].1, &t);
				sx = tr.max.x + 24.0;
			}
			if x.stats[0].0 < 0.0 && x.stats[1].0 < 0.0 {
				self.text(&p, Pos2::new(sx, sy), theme::label(), theme::tk().muted, "no Valve poses to compare with");
			}
		}

		// the graph: lanes for the events above, the distances under them
		let g = Rect::from_min_max(Pos2::new(r.min.x + 70.0, sy + 26.0), Pos2::new(r.max.x - 14.0, r.max.y - 26.0));
		let resp = ui.interact(g, Id::unique("rp_graph"), Sense::click_and_drag());
		let (v0, v1) = self.rp.view;
		let span = (v1 - v0).max(0.5);
		let tx = |t: f64| g.min.x + ((t - v0) / span) as f32 * g.width();
		let xt = |px: f32| v0 + ((px - g.min.x) / g.width()) as f64 * span;
		// zoom about the pointer with the wheel, pan with the right button, scrub with the left
		if resp.hovered() {
			let wheel = ui.input(|i| i.smooth_scroll_delta.y) as f64 / 40.0;
			if wheel != 0.0 {
				if let Some(mp) = resp.hover_pos() {
					let at = xt(mp.x);
					let k = 0.85f64.powf(wheel);
					let (n0, n1) = (at - (at - v0) * k, at + (v1 - at) * k);
					if n1 - n0 > 1.0 && n1 - n0 <= d.dur.max(1.0) * 1.05 {
						self.rp.view = (n0, n1);
					}
				}
			}
		}
		if resp.dragged_by(PointerButton::Secondary) {
			let dl = resp.drag_delta().x as f64 / g.width() as f64 * span;
			self.rp.view = (v0 - dl, v1 - dl);
		}
		self.rp.scrubbing = resp.dragged_by(PointerButton::Primary);
		if resp.clicked() || self.rp.scrubbing {
			if let Some(mp) = resp.interact_pointer_pos() {
				self.rp.t = xt(mp.x).clamp(0.0, d.dur);
			}
		}
		// the cursor kept in view while playing, where playing took it out (a page on); not where the view was moved
		// away from it (dragged or zoomed), which the view keeps
		let in_view = (v0..=v1).contains(&self.rp.t);
		if self.rp.playing && !in_view && self.rp.cursor_shown && !resp.dragged() && self.rp.t > v1 {
			self.rp.view = (self.rp.t - span * 0.05, self.rp.t + span * 0.95);
			self.rp.cursor_shown = true;
		} else {
			self.rp.cursor_shown = in_view;
		}
		let gp = p.with_clip_rect(g.expand(2.0));
		gp.rect_filled(g, CornerRadius::same(4), theme::tk().well);
		// the lanes: the live run's events, the replay's, the stations lighting the device, its radio's loss
		let lane = 15.0;
		let lanes = 4.0;
		let plot =
			Rect::from_min_max(Pos2::new(g.min.x, g.min.y + lanes * lane + 14.0), Pos2::new(g.max.x, g.max.y - 8.0));
		let ry = g.min.y + lanes * lane + 5.0;
		self.rule(&gp, Pos2::new(g.min.x, ry), Pos2::new(g.max.x, ry));
		let lane_y = |k: f32| g.min.y + 3.0 + k * lane;
		let mut hover_text = None;
		let hover = resp.hover_pos();
		if let Some(x) = x {
			// the stations in view, per LANE_S: red none, amber one, green two or more
			let (b0, b1) = (
				((v0.max(0.0) / replay::LANE_S) as usize).min(x.lane.len()),
				((v1.max(0.0) / replay::LANE_S) as usize + 1).min(x.lane.len()),
			);
			let mut i = b0;
			let ly = lane_y(2.0);
			while i < b1 {
				let v = x.lane[i];
				let mut j = i + 1;
				while j < b1 && x.lane[j] == v {
					j += 1;
				}
				if v != 255 {
					let col = match v {
						0 => alpha(theme::tk().bad, 190),
						1 => alpha(theme::tk().warn, 170),
						_ => alpha(theme::tk().good, 110),
					};
					let (xa, xb) = (tx(i as f64 * replay::LANE_S), tx(j as f64 * replay::LANE_S));
					gp.rect_filled(
						Rect::from_min_max(Pos2::new(xa, ly), Pos2::new(xb.max(xa + 1.0), ly + lane - 4.0)),
						0.0,
						col,
					);
				}
				i = j;
			}
			// the radio's loss each half second: the more lost, the redder
			let ly = lane_y(3.0);
			let (a, b) =
				(x.diag.partition_point(|g| (g.t as f64) < v0), x.diag.partition_point(|g| (g.t as f64) <= v1 + 0.5));
			for g in &x.diag[a..b] {
				if g.loss < 0.5 {
					continue;
				}
				let (xa, xb) = (tx(g.t as f64 - 0.5), tx(g.t as f64));
				let al = (40.0 + g.loss.min(40.0) * 5.0) as i32;
				gp.rect_filled(
					Rect::from_min_max(Pos2::new(xa, ly), Pos2::new(xb.max(xa + 1.0), ly + lane - 4.0)),
					0.0,
					alpha(theme::tk().bad, al),
				);
			}
			if let Some(mp) = hover {
				let ht = xt(mp.x);
				if mp.y >= lane_y(2.0) && mp.y < lane_y(3.0) {
					let i = (ht.max(0.0) / replay::LANE_S) as usize;
					hover_text = match x.lane.get(i) {
						Some(255) | None => None,
						Some(0) => Some(format!("{:.1} s: dark (no station lit 3 sensors)", ht)),
						Some(n) => Some(format!("{:.1} s: {n} station{} in view", ht, if *n == 1 { "" } else { "s" })),
					};
				} else if mp.y >= lane_y(3.0) && mp.y < ry {
					let i = x.diag.partition_point(|g| (g.t as f64) < ht);
					hover_text = x.diag.get(i).map(|g| {
						format!(
							"{:.1} s: radio loss {:.0} % in the half second, {} stations, {} sensors",
							g.t, g.loss, g.stations, g.sensors
						)
					});
				}
			}
			// where Valve called its pose not valid: shaded
			for &(a, b) in &x.valve_bad {
				let (xa, xb) = (tx(a as f64), tx(b as f64));
				if xb < g.min.x || xa > g.max.x {
					continue;
				}
				let rr = Rect::from_min_max(Pos2::new(xa, plot.min.y), Pos2::new(xb.max(xa + 1.0), plot.max.y));
				gp.rect_filled(rr, 0.0, alpha(theme::tk().valve, 34));
			}
			// sally's gaps (what SteamVR got, the replay): bars along the bottom
			for (k, tr) in [(1usize, &x.live), (2, &x.replay)] {
				if !self.st.replay_tracks[k] {
					continue;
				}
				let yb = g.max.y - if k == 1 { 6.0 } else { 2.0 };
				for (a, b) in tr.gaps() {
					let (xa, xb) = (tx(a as f64), tx(b as f64));
					if xb < g.min.x || xa > g.max.x {
						continue;
					}
					gp.line_segment(
						[Pos2::new(xa, yb), Pos2::new(xb.max(xa + 2.0), yb)],
						Stroke::new(3.0, self::tracks()[k].1),
					);
				}
			}
			// the distances: the largest in each column of pixels
			let ymax = self.rp.ymax.get(self.rp.dev).copied().unwrap_or(0.05);
			let ty = |m: f32| plot.max.y - ((m as f64 / ymax).min(1.04) as f32) * plot.height();
			for val in [0.0, ymax * 0.5, ymax] {
				let y = ty(val as f32);
				gp.line_segment([Pos2::new(g.min.x, y), Pos2::new(g.max.x, y)], Stroke::new(1.0, white(12)));
				let lbl = if val == 0.0 { "0".to_string() } else { format!("{:.0} mm", val * 1e3) };
				self.text_at(
					&p,
					Pos2::new(g.min.x - 8.0, y),
					Align2::RIGHT_CENTER,
					theme::label(),
					theme::tk().muted,
					&lbl,
				);
			}
			// the replay under what SteamVR got, wider: where they are the same both show
			for k in [1usize, 0] {
				if !self.st.replay_tracks[k + 1] {
					continue;
				}
				let stroke = Stroke::new(if k == 1 { 2.2 } else { 1.2 }, self::tracks()[k + 1].1);
				for line in graph_line(&x.err[k], v0, v1, &tx) {
					let pts: Vec<Pos2> = line.iter().map(|(px, m)| Pos2::new(*px, ty(*m))).collect();
					if pts.len() == 1 {
						gp.circle_filled(pts[0], stroke.width, stroke.color);
					} else {
						gp.add(Shape::line(pts, stroke));
					}
				}
			}
			// the events: the live run's in the upper lane, the replay's in the lower
			let mut hover_d = 5.0;
			for e in d.events.iter().filter(|e| e.dev == x.codename) {
				let ex = tx(e.t as f64);
				if ex < g.min.x - 2.0 || ex > g.max.x + 2.0 {
					continue;
				}
				let y0 = g.min.y + if e.live { 3.0 } else { lane + 3.0 };
				let (col, hh) = match e.level {
					0 => (alpha(theme::tk().good, 150), (lane - 4.0) * 0.5),
					1 => (alpha(theme::tk().warn, 200), (lane - 4.0) * 0.6),
					_ => (theme::tk().bad, lane - 4.0),
				};
				let y0 = y0 + (lane - 4.0 - hh);
				gp.rect_filled(Rect::from_min_size(Pos2::new(ex - 1.0, y0), Vec2::new(2.5, hh)), 1.0, col);
				if let Some(mp) = hover {
					let dd = (mp.x - ex).abs();
					if dd < hover_d && mp.y < lane_y(2.0) {
						hover_d = dd;
						let who = if e.live { "live" } else { "replay" };
						hover_text = Some(format!("{who} at {:.1} s: {}", e.t, e.text));
					}
				}
			}
			let (fc, lx) = (theme::tk().faint, g.min.x - 8.0);
			for (k, name) in ["live", "replay", "light", "radio"].into_iter().enumerate() {
				let y = lane_y(k as f32) + (lane - 4.0) * 0.5;
				self.text_at(&p, Pos2::new(lx, y), Align2::RIGHT_CENTER, medium(12.0), fc, name);
			}
			// on the graph: the distances at the pointer
			if let Some(mp) = hover.filter(|mp| mp.y >= plot.min.y && hover_text.is_none()) {
				let ht = xt(mp.x);
				let mut parts = vec![format!("{ht:.2} s")];
				for k in [0usize, 1] {
					if let Some(e) = err_at(&x.err[k], ht) {
						parts.push(format!("{} {}", self::tracks()[k + 1].0, mm(e)));
					}
				}
				if parts.len() > 1 {
					gp.line_segment(
						[Pos2::new(mp.x, plot.min.y), Pos2::new(mp.x, plot.max.y)],
						Stroke::new(1.0, white(50)),
					);
					hover_text = Some(parts.join("   "));
				}
			}
			if let Some(text) = hover_text {
				let mp = resp.hover_pos().unwrap_or(g.min);
				let tw = Self::text_width(&p, theme::label(), &text).min(r.width() - 40.0);
				let bx = (mp.x + 12.0).min(r.max.x - tw - 26.0).max(r.min.x + 10.0);
				let br = Rect::from_min_size(Pos2::new(bx, mp.y + 16.0), Vec2::new(tw + 16.0, 26.0));
				let tp = ui.ctx().layer_painter(egui::LayerId::new(egui::Order::Tooltip, Id::unique("rp_tip")));
				tp.rect_filled(br, 6.0, theme::tk().float);
				tp.rect_stroke(br, 6.0, Stroke::new(1.0, theme::tk().card_edge), StrokeKind::Inside);
				let tc = tp.with_clip_rect(br);
				tc.text(
					br.left_center() + Vec2::new(8.0, 0.0),
					Align2::LEFT_CENTER,
					&text,
					theme::label(),
					theme::tk().text,
				);
			}
		}
		// the cursor
		let cxp = tx(t_now);
		if cxp >= g.min.x && cxp <= g.max.x {
			gp.line_segment([Pos2::new(cxp, g.min.y), Pos2::new(cxp, g.max.y)], Stroke::new(1.5, theme::tk().text));
		}
		// the time under it
		let step = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 30.0, 60.0, 120.0, 300.0, 600.0, 900.0, 1800.0]
			.into_iter()
			.find(|s| span / s <= 10.0)
			.unwrap_or(3600.0);
		let mut k = (v0.max(0.0) / step).ceil() as i64;
		while (k as f64) * step <= v1.min(d.dur) + 1e-6 {
			let tt = k as f64 * step;
			let xx = tx(tt);
			p.line_segment([Pos2::new(xx, g.max.y), Pos2::new(xx, g.max.y + 4.0)], Stroke::new(1.0, white(60)));
			let lbl = tick_text(tt, step);
			self.text_at(&p, Pos2::new(xx, g.max.y + 5.0), Align2::CENTER_TOP, theme::label(), theme::tk().faint, &lbl);
			k += 1;
		}
	}
}

/// The graph's lines of a list of (t, m) over [v0, v1]: in each column of pixels its largest, at that sample's place;
/// joined from column to column, apart where samples are more than GRAPH_GAP apart (Valve had no valid pose). Zoomed
/// in, where samples are fewer than columns, each is its own point and the line goes through them all
fn graph_line(e: &[(f32, f32)], v0: f64, v1: f64, tx: &dyn Fn(f64) -> f32) -> Vec<Vec<(f32, f32)>> {
	// one sample either side of the view, so the line reaches its edges
	let a = e.partition_point(|q| (q.0 as f64) < v0).saturating_sub(1);
	let b = (e.partition_point(|q| (q.0 as f64) <= v1) + 1).min(e.len());
	let mut out: Vec<Vec<(f32, f32)>> = Vec::new();
	let mut line: Vec<(f32, f32)> = Vec::new();
	// the column being gathered: its pixel, the largest's place and value
	let mut cur: Option<(i64, f32, f32)> = None;
	let mut last_t = f32::NAN;
	for q in &e[a..b] {
		let px = tx(q.0 as f64);
		let c = px.floor() as i64;
		if q.0 - last_t > GRAPH_GAP {
			if let Some((_, x, m)) = cur.take() {
				line.push((x, m));
			}
			if !line.is_empty() {
				out.push(std::mem::take(&mut line));
			}
		}
		last_t = q.0;
		match cur {
			Some((cc, _, m)) if cc == c => {
				if q.1 > m {
					cur = Some((c, px, q.1));
				}
			}
			_ => {
				if let Some((_, x, m)) = cur {
					line.push((x, m));
				}
				cur = Some((c, px, q.1));
			}
		}
	}
	if let Some((_, x, m)) = cur {
		line.push((x, m));
	}
	if !line.is_empty() {
		out.push(line);
	}
	out
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_graph_joins_its_samples_and_breaks_at_gaps() {
		// zoomed in: 100 pixels a second, samples every 10 ms (a pixel each), a gap of a second at 1 s
		let e: Vec<(f32, f32)> =
			(0..300).map(|i| i as f32 * 0.01).filter(|t| !(1.0..2.0).contains(t)).map(|t| (t, t)).collect();
		let tx = |t: f64| (t * 100.0 + 0.5) as f32;
		let lines = graph_line(&e, 0.0, 3.0, &tx);
		assert_eq!(lines.len(), 2);
		assert_eq!((lines[0].len(), lines[1].len()), (100, 100));
		// zoomed far in: 1000 pixels a second, still one line through every sample
		let tx = |t: f64| (t * 1000.0) as f32;
		assert_eq!(graph_line(&e, 0.2, 0.3, &tx).len(), 1);
		// zoomed out: 10 pixels a second, a column's largest
		let tx = |t: f64| (t * 10.0) as f32;
		let lines = graph_line(&e, 0.0, 3.0, &tx);
		assert!(lines[0].len() == 10 && (lines[0][3].1 - 0.39).abs() < 0.015, "{:?}", lines[0]);
		assert_eq!(tick_text(65.25, 0.5), "1:05.2");
		assert_eq!(tick_text(65.25, 1.0), "1:05");
	}

	#[test]
	fn stamps_durations_and_distances_read_as_people_say_them() {
		assert_eq!(stamp_text("20261107_123001"), "7 Nov 12:30:01");
		assert_eq!(stamp_text("x"), "x");
		assert_eq!(dur_text(75.4), "1:15");
		assert_eq!(dur_text(3725.0), "1:02:05");
		assert_eq!(mm(0.0041), "4.1 mm");
		assert_eq!(mm(0.234), "234 mm");
		assert_eq!(mm(-1.0), "-");
	}

	#[test]
	fn the_distance_at_a_time_is_the_nearest_sample_close_enough() {
		let e = [(1.0f32, 0.01f32), (1.05, 0.02), (3.0, 0.5)];
		assert_eq!(err_at(&e, 1.04), Some(0.02));
		assert_eq!(err_at(&e, 1.0), Some(0.01));
		assert_eq!(err_at(&e, 2.0), None);
		assert_eq!(err_at(&[], 2.0), None);
		assert_eq!(primary([true, true, false]), 1);
		assert_eq!(primary([true, false, false]), 0);
	}
}
