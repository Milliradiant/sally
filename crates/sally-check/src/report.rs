//! What the check found, per device: a headline, the radio, the light by base station, the sensors, and every moment
//! SteamVR showed trouble with what the light and radio did then; written as one self-contained HTML page (with a
//! timeline per device) and as a few lines for the console.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::analysis::{Analyzer, Bin, Device, BINS_PER_S, BIN_US, CHANNELS, SEEN_HITS};
use crate::events::{self, station_name, Cause, Event, Kind};
use crate::session::Session;

/// Below this share of its neighbours' sweeps a tracker's sensor is dead or covered; controllers are held, so only
/// the dead mark applies to them
const SENSOR_DEAD: f64 = 0.05;
const SENSOR_WEAK: f64 = 0.2;
/// The fewest sweeps a sensor's verdict needs
const SENSOR_MIN_SWEEPS: u32 = 300;
/// A room-wide moment: at least two thirds of the devices sending at the time, and at least 3 of them, for at least
/// half a second
const ROOM_SHARE: f64 = 0.66;
const ROOM_MIN_DEVICES: usize = 3;
const ROOM_MIN_BINS: usize = 2;
/// Trouble this soon after a room-wide blackout is the light coming back, not the device (us)
const ROOM_AFTER_US: u64 = 1_500_000;

/// Something that happened to every device at once
#[derive(Clone, Debug)]
pub struct Room {
	/// true: no light reached them; false: their data stopped arriving (the host's side: USB, the receiver)
	pub light: bool,
	/// bins, the end excluded
	pub start: usize,
	pub end: usize,
	/// how many of the devices sending were caught by it, at most, and of how many
	pub hit: usize,
	pub of: usize,
}

/// The quarter seconds in which most devices were dark (`light`) or lost most of their data, as runs
fn room_events(a: &Analyzer) -> Vec<Room> {
	let mut out = Vec::new();
	// each device's active bins and usual IMU step, once: each is a pass over the session (the step a sort of its
	// samples), which taken for every device in every bin made a long session's check quadratic
	let per: Vec<(std::ops::Range<usize>, f64)> =
		a.devices.iter().map(|d| (d.active(), d.imu_step().unwrap_or(192_000.0))).collect();
	for light in [true, false] {
		// the run's start, and its most devices caught (of how many)
		let mut run: Option<(usize, usize, usize)> = None;
		for i in 0..=a.bins() {
			let mut sending = 0usize;
			let mut hit = 0usize;
			for (d, (active, step)) in a.devices.iter().zip(&per) {
				let Some(b) = d.bins.get(i).filter(|b| b.reports > 0 && active.contains(&i)) else { continue };
				sending += 1;
				let lost = d.imu_lost(b, *step);
				let bad = if light {
					b.station_count() == 0 && b.gen1_hits < SEEN_HITS
				} else {
					lost > 0.5 * (lost + b.imu as f64)
				};
				hit += bad as usize;
			}
			let room = sending >= ROOM_MIN_DEVICES && hit as f64 >= ROOM_SHARE * sending as f64;
			match (room, run) {
				(true, None) => run = Some((i, hit, sending)),
				(true, Some((s, h, o))) => {
					if hit * o > h * sending || (hit * o == h * sending && sending > o) {
						run = Some((s, hit, sending));
					}
				}
				(false, Some((s, hit, of))) => {
					if i - s >= ROOM_MIN_BINS {
						out.push(Room { light, start: s, end: i, hit, of });
					}
					run = None;
				}
				_ => {}
			}
		}
	}
	out.sort_by_key(|r| r.start);
	out
}

/// A tracker without a role, named by how high it was (SteamVR's height above the floor, median over the session)
fn height_words(h: f64) -> &'static str {
	match h {
		h if h < 0.35 => "foot",
		h if h < 0.7 => "knee",
		h if h < 1.15 => "waist",
		h if h < 1.4 => "chest",
		_ => "head",
	}
}

/// A sensor worth a look
#[derive(Clone, Debug)]
pub struct SensorNote {
	pub sensor: usize,
	/// share of its neighbours' sweeps it was lit in, and how many sweeps that is
	pub rate: f64,
	pub sweeps: u32,
	/// the device's usual share
	pub typical: f64,
	pub dead: bool,
}

/// One station's light on a device
#[derive(Clone, Debug)]
pub struct StationLight {
	pub channel: usize,
	/// share of the session it saw the device, its hits a second while it did, and the sensors per quarter second
	pub seen: f64,
	pub hits_s: f64,
	pub sensors: f64,
}

/// The quarter seconds seen by the same number of stations, summed (the radio load table)
#[derive(Default)]
struct LoadRow {
	bins: f64,
	packets: f64,
	light_bytes: f64,
	lost: f64,
	imu: f64,
	delay_sum: f64,
	delay_n: f64,
}

/// One device, summed up
pub struct DeviceReport<'a> {
	pub dev: &'a Device,
	pub label: String,
	pub serial: String,
	/// how high it was (m), when SteamVR showed it
	pub height: Option<f64>,
	pub reports_s: f64,
	pub packets_s: f64,
	pub doubled: f64,
	pub bytes_s: f64,
	pub light_bytes_s: f64,
	/// IMU samples lost: overall, the worst second, and the longest stretch without data (s)
	pub lost: f64,
	pub lost_worst: f64,
	pub gap: f64,
	pub stations: Vec<StationLight>,
	/// share of the session seen by 0, 1, 2, 3+ stations
	pub seen_by: [f64; 4],
	/// sensors lit when one station alone saw it
	pub one_station_sensors: f64,
	/// the light's age on arrival: median and the 95th percentile of the quarter seconds' means (ms)
	pub delay: f64,
	pub delay_p95: f64,
	/// its light came from gen1 stations (not told apart by station)
	pub gen1: bool,
	pub sensors: Vec<SensorNote>,
	pub events: Vec<Event>,
	pub headline: String,
	pub severity: u8,
}

impl DeviceReport<'_> {
	/// How much of the time stations saw it, in words
	fn seen_words(&self) -> String {
		if self.gen1 {
			format!(
				"gen1 light {} of the time, none {} (gen1 stations aren't told apart)",
				pct(self.seen_by[2]),
				pct(self.seen_by[0])
			)
		} else {
			format!(
				"2+ stations {}, one {}, none {}",
				pct(self.seen_by[2] + self.seen_by[3]),
				pct(self.seen_by[1]),
				pct(self.seen_by[0])
			)
		}
	}
}

/// The whole check
pub struct Report<'a> {
	pub a: &'a Analyzer,
	pub session: &'a Session,
	pub devices: Vec<DeviceReport<'a>>,
	pub room: Vec<Room>,
	pub seconds: f64,
	pub channels: Vec<usize>,
}

fn pct(x: f64) -> String {
	if x > 0.0 && x < 0.005 {
		"<1%".into()
	} else {
		format!("{:.0}%", x * 100.0)
	}
}

fn esc(s: &str) -> String {
	s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

fn median(v: &mut [f64]) -> f64 {
	if v.is_empty() {
		return 0.0;
	}
	v.sort_by(|a, b| a.total_cmp(b));
	v[v.len() / 2]
}

fn quantile(v: &mut [f64], q: f64) -> f64 {
	if v.is_empty() {
		return 0.0;
	}
	v.sort_by(|a, b| a.total_cmp(b));
	v[((v.len() - 1) as f64 * q).round() as usize]
}

/// A role as people say it
fn role_words(role: &str, class: &str, kind: &str) -> String {
	let r = role.trim_start_matches("TrackerRole_");
	let mut out = String::new();
	for (i, c) in r.chars().enumerate() {
		if c.is_ascii_uppercase() && i > 0 {
			out.push(' ');
		}
		out.push(c.to_ascii_lowercase());
	}
	let what = match class {
		"controller" | "tracker" | "hmd" => class,
		_ => kind,
	};
	match (out.as_str(), what) {
		("", w) => w.to_string(),
		("left hand" | "right hand", "controller") => format!("{out} controller"),
		(r, "tracker") => format!("{r} tracker"),
		(r, w) => format!("{r} ({w})"),
	}
}

impl<'a> Report<'a> {
	pub fn new(a: &'a Analyzer, session: &'a Session) -> Report<'a> {
		let seconds = a.start_us.map_or(0.0, |s| (a.last_us.saturating_sub(s)) as f64 / 1e6);
		let mut channels: Vec<usize> = (0..CHANNELS)
			.filter(|&c| a.devices.iter().any(|d| d.bins.iter().any(|b| b.hits[c] >= SEEN_HITS)))
			.collect();
		channels.sort_unstable();
		let mut devices: Vec<DeviceReport> =
			a.devices.iter().filter(|d| d.bins.iter().any(|b| b.reports > 0)).map(|d| device(a, session, d)).collect();
		// the one-station sensor counts compared across devices: a device far below the others is the odd one
		let others: Vec<f64> = devices.iter().map(|d| d.one_station_sensors).filter(|&s| s > 0.0).collect();
		// trouble during a room-wide blackout, or as the light comes back after one, is not the device's
		let room = room_events(a);
		let t0 = a.start_us.unwrap_or(0);
		let us = |b: usize| t0 + b as u64 * BIN_US;
		// a dark sensor that is dark on another device of the same make too (by sensor count) is more likely where they
		// sit (a strap, a mount) than a dead sensor: the devices it is shared with, per device
		let dark: Vec<(usize, Option<usize>, String)> = devices
			.iter()
			.map(|d| {
				let ct = d.dev.config.as_ref().map_or(0, |c| c.sensor_ct);
				(ct, d.sensors.iter().find(|s| s.dead).map(|s| s.sensor), d.label.clone())
			})
			.collect();
		let shared: Vec<Vec<String>> = (0..devices.len())
			.map(|i| match dark[i].1 {
				Some(s) => (0..devices.len())
					.filter(|&j| j != i && dark[j].0 == dark[i].0 && dark[j].1 == Some(s))
					.map(|j| format!("{} ({})", dark[j].2, devices[j].serial))
					.collect(),
				None => Vec::new(),
			})
			.collect();
		for (d, shared) in devices.iter_mut().zip(&shared) {
			for e in &mut d.events {
				for r in room.iter().filter(|r| r.light) {
					let (s, end) = (us(r.start), us(r.end));
					if e.start_us < end && e.end_us > s {
						e.room = Some("during");
					} else if e.room.is_none() && e.start_us >= end && e.start_us < end + ROOM_AFTER_US {
						e.room = Some("right after");
					}
				}
			}
			finish_headline(d, &others, shared);
		}
		devices.sort_by(|x, y| y.severity.cmp(&x.severity).then(x.label.cmp(&y.label)));
		Report { a, session, devices, room, seconds, channels }
	}

	/// A station by its channel, with its serial when SteamVR listed it
	fn station_full(&self, c: usize) -> String {
		match self.session.station(c) {
			Some(s) => format!("{} ({s})", station_name(c)),
			None => station_name(c),
		}
	}

	/// The room-wide moments in words
	fn room_lines(&self) -> Vec<String> {
		self.room
			.iter()
			.map(|r| {
				let who = if r.hit == r.of { "Every device".to_string() } else { format!("{} of {} devices", r.hit, r.of) };
				let when = format!(
					"at {:.1} s for {:.1} s",
					r.start as f64 / BINS_PER_S,
					(r.end - r.start) as f64 / BINS_PER_S
				);
				if r.light {
					format!(
						"{who} lost all light at once {when}: the base stations were blocked or stopped for all of them, or they were all out of view, not something about one device."
					)
				} else {
					format!("{who} stopped getting data through at once {when}: the PC's side (USB, the receiver), not the devices.")
				}
			})
			.collect()
	}

	/// The console's version: a line or two per device
	pub fn text(&self) -> String {
		let mut s = String::new();
		let _ = writeln!(
			s,
			"{:.0} s recorded, {} devices, base stations seen: {}",
			self.seconds,
			self.devices.len(),
			if self.channels.is_empty() {
				"none (gen1 light, or none at all)".to_string()
			} else {
				self.channels.iter().map(|&c| self.station_full(c)).collect::<Vec<_>>().join(", ")
			}
		);
		if self.session.sally {
			let _ = writeln!(s, "SteamVR was showing sally's tracking, so its trouble below is sally's.");
		}
		for l in self.room_lines() {
			let _ = writeln!(s, "{l}");
		}
		for d in &self.devices {
			let _ = writeln!(s, "\n  {} ({})", d.label, d.serial);
			let _ = writeln!(s, "    {}", d.headline);
			let _ = writeln!(
				s,
				"    radio {} lost (worst second {}), light: {}",
				pct(d.lost),
				pct(d.lost_worst),
				d.seen_words()
			);
		}
		s
	}

	/// The page
	pub fn html(&self, title: &str) -> String {
		let mut h = String::new();
		let _ = write!(
			h,
			"<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>{}</title><style>{}</style></head><body><main>",
			esc(title),
			CSS
		);
		let _ = write!(h, "<header><h1>sally check</h1><p class=sub>{}</p>", esc(title));
		let _ = write!(
			h,
			"<p>{:.0} s recorded &middot; {} devices &middot; base stations: {}</p></header>",
			self.seconds,
			self.devices.len(),
			if self.channels.is_empty() {
				"none seen on channels (gen1 stations, or no light)".to_string()
			} else {
				// the stations' serials are the session's devices.txt: text from the recording, as every name here
				esc(&self.channels.iter().map(|&c| self.station_full(c)).collect::<Vec<_>>().join(", "))
			}
		);
		if self.session.sally {
			h.push_str("<p class=note>SteamVR was showing sally's tracking during this recording (sally's driver was running), so the SteamVR trouble below is sally's, not Valve's.</p>");
		}
		for l in self.room_lines() {
			let _ = write!(h, "<p class=\"headline s1\">{}</p>", esc(&l));
		}

		// the overview
		h.push_str("<section><h2>Overview</h2><div class=scroll><table><tr><th>Device</th><th>What it shows</th><th>SteamVR trouble</th><th>Radio lost</th><th>Seen by 2+ / 1 / 0 stations</th></tr>");
		for d in &self.devices {
			let _ = write!(
				h,
				"<tr><td><a href=\"#{}\">{}</a><div class=dim>{}</div></td><td><span class=\"dot s{}\"></span>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>",
				esc(&d.serial),
				esc(&d.label),
				esc(&d.serial),
				d.severity,
				esc(&d.headline),
				d.events.len(),
				pct(d.lost),
				if d.gen1 {
					format!("gen1: {} / - / {}", pct(d.seen_by[2]), pct(d.seen_by[0]))
				} else {
					format!("{} / {} / {}", pct(d.seen_by[2] + d.seen_by[3]), pct(d.seen_by[1]), pct(d.seen_by[0]))
				}
			);
		}
		h.push_str("</table></div></section>");

		for d in &self.devices {
			self.device_html(&mut h, d);
		}
		self.radio_load_html(&mut h);
		h.push_str(HOW_TO_READ);
		h.push_str("</main></body></html>");
		h
	}

	fn device_html(&self, h: &mut String, d: &DeviceReport) {
		let _ = write!(
			h,
			"<section id=\"{}\"><h2>{}</h2><p class=dim>{} &middot; dongle {}{}{}</p><p class=\"headline s{}\">{}</p>",
			esc(&d.serial),
			esc(&d.label),
			esc(&d.serial),
			esc(&d.dev.dongle),
			d.height.map_or(String::new(), |h| format!(" &middot; usually {h:.2} m high")),
			d.dev.battery_min().map_or(String::new(), |b| format!(" &middot; battery down to {b}%")),
			d.severity,
			esc(&d.headline)
		);
		self.timeline(h, d);

		h.push_str("<div class=grid><div><h3>Light</h3><table><tr><th>Station</th><th>Saw it</th><th>Hits/s</th><th>Sensors</th></tr>");
		for s in &d.stations {
			let _ = write!(
				h,
				"<tr><td>{}</td><td>{}</td><td>{:.0}</td><td>{:.1}</td></tr>",
				esc(&self.station_full(s.channel)),
				pct(s.seen),
				s.hits_s,
				s.sensors
			);
		}
		let _ = write!(
			h,
			"</table><p class=dim>Light: {}.{}</p></div>",
			d.seen_words(),
			if d.one_station_sensors > 0.0 {
				format!(" With one station alone, {:.1} sensors lit.", d.one_station_sensors)
			} else {
				String::new()
			}
		);
		let _ = write!(
			h,
			"<div><h3>Radio</h3><table><tr><td>Data lost</td><td>{} (worst second {})</td></tr><tr><td>Longest gap</td><td>{:.0} ms</td></tr><tr><td>Reports / packets</td><td>{:.0} / {:.0} a second</td></tr><tr><td>Light's age on arrival</td><td>{:.1} ms (95%: {:.1} ms)</td></tr></table></div>",
			pct(d.lost),
			pct(d.lost_worst),
			d.gap * 1000.0,
			d.reports_s,
			d.packets_s,
			d.delay,
			d.delay_p95
		);
		h.push_str("<div><h3>Sensors</h3>");
		if d.sensors.is_empty() {
			let n = d.dev.config.as_ref().map_or(0, |c| c.sensor_ct);
			if n == 0 {
				h.push_str("<p class=dim>No config for this device, so its sensors can't be checked.</p>");
			} else {
				let _ = write!(h, "<p>All {n} sensors light up with their neighbours.</p>");
			}
		} else {
			h.push_str("<table><tr><th>Sensor</th><th>Lit with its neighbours</th><th>Usual</th></tr>");
			for s in &d.sensors {
				let _ = write!(
					h,
					"<tr><td>{}{}</td><td>{} of {} sweeps</td><td>{}</td></tr>",
					s.sensor,
					if s.dead { " <b>dead or covered</b>" } else { " rarely lit" },
					pct(s.rate),
					s.sweeps,
					pct(s.typical)
				);
			}
			h.push_str("</table>");
		}
		h.push_str("</div></div>");

		if !d.events.is_empty() {
			let mut by: BTreeMap<Cause, usize> = BTreeMap::new();
			for e in &d.events {
				*by.entry(e.cause).or_default() += 1;
			}
			let _ = write!(
				h,
				"<h3>SteamVR trouble ({})</h3><p class=dim>{}</p><div class=scroll><table><tr><th>At</th><th>What</th><th>For</th><th>Jump</th><th>Light and radio then</th></tr>",
				d.events.len(),
				by.iter().map(|(c, n)| format!("{n} {}", c.short())).collect::<Vec<_>>().join(" &middot; ")
			);
			let t0 = self.a.start_us.unwrap_or(0);
			for e in d.events.iter().take(200) {
				let _ = write!(
					h,
					"<tr><td>{:.1} s</td><td>{}</td><td>{:.0} ms</td><td>{}</td><td><span class=\"tag c{}\">{}</span> {}{}</td></tr>",
					e.start_us.saturating_sub(t0) as f64 / 1e6,
					e.kind.word(),
					(e.end_us - e.start_us) as f64 / 1000.0,
					if e.jump > 0.0 { format!("{:.1} cm", e.jump * 100.0) } else { "-".into() },
					e.cause as u8,
					e.cause.short(),
					esc(&e.detail),
					e.room.map_or(String::new(), |w| format!(" <span class=tag>{w} a room-wide blackout</span>"))
				);
			}
			h.push_str("</table></div>");
		}
		h.push_str("</section>");
	}

	/// The timeline: one lane per station (how many sensors it lit), the radio, the light's lateness, and SteamVR's
	/// trouble
	fn timeline(&self, h: &mut String, d: &DeviceReport) {
		let bins = self.a.bins().max(1);
		let w = 1000.0;
		let lane = 14.0;
		let label_w = 70.0;
		let gen1 = d.dev.bins.iter().any(|b| b.gen1_hits >= SEEN_HITS);
		let rows = self.channels.len() + 3 + gen1 as usize;
		let height = rows as f64 * (lane + 4.0) + 22.0;
		let x = |b: usize| label_w + b as f64 / bins as f64 * (w - label_w);
		let bw = (w - label_w) / bins as f64;
		let _ = write!(
			h,
			"<div class=scroll><svg class=tl viewBox=\"0 0 {w} {height:.0}\" role=img aria-label=\"timeline\">"
		);
		// the room-wide moments, behind every lane
		for r in &self.room {
			let _ = write!(
				h,
				"<rect x={:.1} y=0 width={:.1} height={:.0} class=\"room{}\"><title>every device {} at once</title></rect>",
				x(r.start),
				((r.end - r.start) as f64 * bw).max(2.0),
				height - 20.0,
				if r.light { "" } else { " radio" },
				if r.light { "lost light" } else { "lost data" }
			);
		}
		let mut y = 2.0;
		let step = d.dev.imu_step().unwrap_or(192_000.0);
		let lane_run = |h: &mut String, label: &str, class: &str, f: &dyn Fn(&Bin) -> f64, y: f64| {
			let _ = write!(h, "<text x=0 y={:.1} class=lab>{}</text>", y + lane - 3.0, esc(label));
			let _ = write!(h, "<rect x={label_w} y={y:.1} width={:.1} height={lane} class=bg />", w - label_w);
			let mut i = 0;
			while i < d.dev.bins.len() {
				let v = f(&d.dev.bins[i]);
				if v <= 0.0 {
					i += 1;
					continue;
				}
				let level = (v * 4.0).ceil().min(4.0);
				let mut j = i + 1;
				while j < d.dev.bins.len() && (f(&d.dev.bins[j]) * 4.0).ceil().min(4.0) == level {
					j += 1;
				}
				let _ = write!(
					h,
					"<rect x={:.1} y={y:.1} width={:.2} height={lane} class=\"{class} l{}\" />",
					x(i),
					(j - i) as f64 * bw,
					level as u8
				);
				i = j;
			}
		};
		for &c in &self.channels {
			lane_run(
				h,
				&station_name(c),
				"st",
				&|b: &Bin| {
					if b.hits[c] >= SEEN_HITS {
						(b.sensors[c].count_ones() as f64 / 6.0).min(1.0)
					} else {
						0.0
					}
				},
				y,
			);
			y += lane + 4.0;
		}
		let dev = d.dev;
		if dev.bins.iter().any(|b| b.gen1_hits >= SEEN_HITS) {
			lane_run(
				h,
				"gen1 light",
				"st",
				&|b: &Bin| {
					if b.gen1_hits >= SEEN_HITS {
						(b.gen1_sensors.count_ones() as f64 / 8.0).min(1.0)
					} else {
						0.0
					}
				},
				y,
			);
			y += lane + 4.0;
		}
		lane_run(
			h,
			"radio lost",
			"rd",
			&|b: &Bin| {
				let l = dev.imu_lost(b, step);
				let got = b.imu as f64;
				if b.reports == 0 {
					1.0
				} else if got + l > 0.0 && l / (got + l) > 0.1 {
					l / (got + l)
				} else {
					0.0
				}
			},
			y,
		);
		y += lane + 4.0;
		lane_run(
			h,
			"light late",
			"lt",
			&|b: &Bin| if b.delay_max > events::LATE_MS { (b.delay_max as f64 / 250.0).min(1.0) } else { 0.0 },
			y,
		);
		y += lane + 4.0;
		let _ = write!(h, "<text x=0 y={:.1} class=lab>SteamVR</text><rect x={label_w} y={y:.1} width={:.1} height={lane} class=bg />", y + lane - 3.0, w - label_w);
		let t0 = self.a.start_us.unwrap_or(0);
		for e in &d.events {
			let b0 = e.start_us.saturating_sub(t0) as f64 / BIN_US as f64;
			let b1 = e.end_us.saturating_sub(t0) as f64 / BIN_US as f64;
			let x0 = label_w + b0 / bins as f64 * (w - label_w);
			let x1 = (label_w + b1 / bins as f64 * (w - label_w)).max(x0 + 2.0);
			let _ = write!(
				h,
				"<rect x={x0:.1} y={y:.1} width={:.1} height={lane} class=\"ev k{}\"><title>{} {:.1} s: {}</title></rect>",
				x1 - x0,
				e.kind as u8,
				e.kind.word(),
				b0 / BINS_PER_S,
				esc(&e.detail)
			);
		}
		y += lane + 4.0;
		// seconds
		let secs = bins as f64 / BINS_PER_S;
		let tick = [5.0, 10.0, 30.0, 60.0, 120.0, 300.0].into_iter().find(|t| secs / t <= 12.0).unwrap_or(600.0);
		let mut t = 0.0;
		while t <= secs {
			let _ = write!(
				h,
				"<text x={:.1} y={:.1} class=ax>{:.0}s</text>",
				label_w + t / secs * (w - label_w),
				y + 12.0,
				t
			);
			t += tick;
		}
		let _ = write!(h, "</svg></div><p class=legend><span class=\"key st l3\"></span>station sees it (darker: more sensors) <span class=\"key rd l3\"></span>data not arriving <span class=\"key lt l3\"></span>light late <span class=\"key ev k0\"></span>lost <span class=\"key ev k1\"></span>froze <span class=\"key ev k2\"></span>jumped{}</p>", if self.room.is_empty() { "" } else { " <span class=\"key roomkey\"></span>every device at once" });
	}

	/// The radio load: what the link carries per device, and the same against how many stations saw the device
	fn radio_load_html(&self, h: &mut String) {
		h.push_str("<section><h2>Radio load</h2><p>Each device sends its light and IMU over its own radio link to its dongle. The dongle hands the PC about 500 reports a second; a report carries one or two radio packets. More base stations in view means more light to send. When the link can't keep up, light queues and arrives late (the light's age below), and if it overflows, data goes missing.</p><div class=scroll><table><tr><th>Device</th><th>Reports/s</th><th>Packets/s</th><th>Reports with 2 packets</th><th>Bytes/s</th><th>Light bytes/s</th><th>Light's age</th></tr>");
		for d in &self.devices {
			let _ = write!(
				h,
				"<tr><td>{}</td><td>{:.0}</td><td>{:.0}</td><td>{}</td><td>{:.0}</td><td>{:.0}</td><td>{:.1} ms</td></tr>",
				esc(&d.label),
				d.reports_s,
				d.packets_s,
				pct(d.doubled),
				d.bytes_s,
				d.light_bytes_s,
				d.delay
			);
		}
		h.push_str("</table></div><h3>By stations in view</h3><p class=dim>All devices' quarter seconds, grouped by how many stations saw the device then.</p><div class=scroll><table><tr><th>Stations</th><th>Time</th><th>Packets/s</th><th>Light bytes/s</th><th>Data lost</th><th>Light's age</th></tr>");
		let mut rows: BTreeMap<usize, LoadRow> = BTreeMap::new();
		for d in &self.devices {
			let step = d.dev.imu_step().unwrap_or(192_000.0);
			for b in d.dev.bins[d.dev.active()].iter().filter(|b| b.reports > 0) {
				let r = rows.entry(b.station_count()).or_default();
				r.bins += 1.0;
				r.packets += b.packets as f64;
				r.light_bytes += b.light_bytes as f64;
				r.lost += d.dev.imu_lost(b, step);
				r.imu += b.imu as f64;
				if let Some(m) = b.delay_mean() {
					r.delay_sum += m as f64;
					r.delay_n += 1.0;
				}
			}
		}
		let total: f64 = rows.values().map(|r| r.bins).sum::<f64>().max(1.0);
		for (n, r) in &rows {
			let _ = write!(
				h,
				"<tr><td>{}</td><td>{}</td><td>{:.0}</td><td>{:.0}</td><td>{}</td><td>{}</td></tr>",
				n,
				pct(r.bins / total),
				r.packets / r.bins * BINS_PER_S,
				r.light_bytes / r.bins * BINS_PER_S,
				pct(if r.lost + r.imu > 0.0 { r.lost / (r.lost + r.imu) } else { 0.0 }),
				if r.delay_n > 0.0 { format!("{:.1} ms", r.delay_sum / r.delay_n) } else { "-".into() }
			);
		}
		h.push_str("</table></div></section>");
	}
}

/// One device's numbers
fn device<'a>(a: &Analyzer, session: &Session, d: &'a Device) -> DeviceReport<'a> {
	let serial = d.serial().to_string();
	let steam = session.steam_device(&serial).or_else(|| session.devices.iter().find(|s| s.dongle == d.dongle));
	let kind = d.config.as_ref().map_or("device", |c| c.kind());
	let role = steam.map_or(String::new(), |s| s.role.clone());
	let class = steam.map_or("", |s| s.class.as_str());
	let height = session.height(&serial);
	let label = match height {
		Some(h) if role.is_empty() && (class == "tracker" || kind == "tracker") => {
			format!("tracker at {} height", height_words(h))
		}
		_ => role_words(&role, class, kind),
	};
	let active = d.active();
	let used: Vec<&Bin> = d.bins[active.clone()].iter().collect();
	let n = used.len().max(1) as f64;
	let seconds = n / BINS_PER_S;
	let sum = |f: &dyn Fn(&Bin) -> f64| used.iter().map(|b| f(b)).sum::<f64>();
	let step = d.imu_step().unwrap_or(192_000.0);
	let lost_n = sum(&|b| d.imu_lost(b, step));
	let got_n = sum(&|b| b.imu as f64);
	let reports = sum(&|b| b.reports as f64);

	// worst second, and the longest stretch of quarter seconds without reports or IMU
	let mut lost_worst: f64 = 0.0;
	for w in d.bins[active.clone()].windows(4) {
		let l: f64 = w.iter().map(|b| d.imu_lost(b, step)).sum();
		let g: f64 = w.iter().map(|b| b.imu as f64).sum();
		if l + g > 0.0 {
			lost_worst = lost_worst.max(l / (l + g));
		}
	}
	let (mut gap, mut run) = (0usize, 0usize);
	for b in &d.bins[active.clone()] {
		if b.imu == 0 {
			run += 1;
			gap = gap.max(run);
		} else {
			run = 0;
		}
	}

	let stations: Vec<StationLight> = (0..CHANNELS)
		.filter_map(|c| {
			let seen: Vec<&&Bin> = used.iter().filter(|b| b.hits[c] >= SEEN_HITS).collect();
			if seen.is_empty() {
				return None;
			}
			let k = seen.len() as f64;
			Some(StationLight {
				channel: c,
				seen: k / n,
				hits_s: seen.iter().map(|b| b.hits[c] as f64).sum::<f64>() / k * BINS_PER_S,
				sensors: seen.iter().map(|b| b.sensors[c].count_ones() as f64).sum::<f64>() / k,
			})
		})
		.collect();
	// gen1 light is not split by station: such a device's time counts by whether any light came
	let mut seen_by = [0.0; 4];
	let mut one = Vec::new();
	for b in used.iter().filter(|b| b.reports > 0) {
		let k = if b.gen1_hits >= SEEN_HITS && b.station_count() == 0 { 2 } else { b.station_count() };
		seen_by[k.min(3)] += 1.0;
		if k == 1 {
			one.push(b.sensors_lit() as f64);
		}
	}
	let total = seen_by.iter().sum::<f64>().max(1.0);
	for s in &mut seen_by {
		*s /= total;
	}
	let mut delays: Vec<f64> = d.bins[active.clone()].iter().filter_map(|b| b.delay_mean().map(|m| m as f64)).collect();
	let delay = median(&mut delays);
	let delay_p95 = quantile(&mut delays, 0.95);

	// sensors
	let mut rates: Vec<f64> = (0..d.neighbours.len())
		.filter(|&s| d.eligible[s] >= SENSOR_MIN_SWEEPS)
		.map(|s| d.eligible_lit[s] as f64 / d.eligible[s] as f64)
		.collect();
	let typical = median(&mut rates);
	let tracker = kind == "tracker";
	let sensors: Vec<SensorNote> = (0..d.neighbours.len())
		.filter(|&s| d.eligible[s] >= SENSOR_MIN_SWEEPS)
		.filter_map(|s| {
			let rate = d.eligible_lit[s] as f64 / d.eligible[s] as f64;
			let dead = rate < SENSOR_DEAD;
			(typical > 0.5 && (dead || (tracker && rate < SENSOR_WEAK))).then_some(SensorNote {
				sensor: s,
				rate,
				sweeps: d.eligible[s],
				typical,
				dead,
			})
		})
		.collect();

	// the device's light from start to end: trouble before it (SteamVR still starting it up) and after it (switched
	// off) is not the device's
	let t0 = a.start_us.unwrap_or(0);
	let (from, to) = (t0 + active.start as u64 * BIN_US, (t0 + active.end as u64 * BIN_US).saturating_sub(BIN_US));
	let events: Vec<Event> = session
		.poses
		.get(&serial)
		.map_or(Vec::new(), |p| events::find(p, a, Some(d)))
		.into_iter()
		.filter(|e| e.start_us >= from && e.end_us <= to)
		.collect();

	DeviceReport {
		dev: d,
		label,
		serial,
		height,
		reports_s: reports / seconds,
		packets_s: sum(&|b| b.packets as f64) / seconds,
		doubled: sum(&|b| b.doubled as f64) / reports.max(1.0),
		bytes_s: sum(&|b| b.bytes as f64) / seconds,
		light_bytes_s: sum(&|b| b.light_bytes as f64) / seconds,
		lost: if lost_n + got_n > 0.0 { lost_n / (lost_n + got_n) } else { 0.0 },
		lost_worst,
		gap: gap as f64 / BINS_PER_S,
		stations,
		seen_by,
		one_station_sensors: if one.is_empty() { 0.0 } else { one.iter().sum::<f64>() / one.len() as f64 },
		delay,
		delay_p95,
		gen1: used.iter().map(|b| b.gen1_hits as u64).sum::<u64>()
			> used.iter().map(|b| b.hits.iter().map(|&h| h as u64).sum::<u64>()).sum::<u64>(),
		sensors,
		events,
		headline: String::new(),
		severity: 0,
	}
}

/// The headline: what stands out, most telling first
fn finish_headline(d: &mut DeviceReport, others: &[f64], shared: &[String]) {
	let mut parts: Vec<String> = Vec::new();
	let mut severity = 0u8;
	if d.dev.config.is_none() {
		parts.push("No config for this device, so only its radio was checked.".into());
		severity = severity.max(1);
	}
	// the device's own trouble: not during a room-wide blackout or as the light came back after one
	let own: Vec<&Event> = d.events.iter().filter(|e| e.room.is_none()).collect();
	let room = d.events.len() - own.len();
	if !own.is_empty() {
		let mut by: BTreeMap<Cause, usize> = BTreeMap::new();
		for e in &own {
			*by.entry(e.cause).or_default() += 1;
		}
		let (top, n) =
			by.iter().max_by_key(|(c, n)| (**n, std::cmp::Reverse(**c))).map(|(c, n)| (*c, *n)).expect("events");
		let kinds: BTreeMap<Kind, usize> = own.iter().fold(BTreeMap::new(), |mut m, e| {
			*m.entry(e.kind).or_default() += 1;
			m
		});
		let what = kinds.iter().map(|(k, n)| format!("{} {}x", k.word(), n)).collect::<Vec<_>>().join(", ");
		let why = match top {
			Cause::Radio => "its data stopped arriving over the radio",
			Cause::NoLight => "no base station could see it",
			Cause::OneStation => "only one base station could see it",
			Cause::Late => "its light arrived late (radio backlog)",
			Cause::Unexplained => "its light and radio looked fine",
		};
		let besides =
			if room > 0 { format!(" (and {room} more around the room-wide blackouts)") } else { String::new() };
		parts.push(format!("SteamVR {what}; {n} of {} while {why}{besides}.", own.len()));
		severity = severity.max(if own.len() >= 3 { 2 } else { 1 });
	} else if room > 0 {
		let what = d.events.iter().fold(BTreeMap::<Kind, usize>::new(), |mut m, e| {
			*m.entry(e.kind).or_default() += 1;
			m
		});
		let what = what.iter().map(|(k, n)| format!("{} {}x", k.word(), n)).collect::<Vec<_>>().join(", ");
		parts.push(format!(
			"SteamVR {what}, all during or right after the room-wide blackouts: not this device's doing."
		));
	}
	if let Some(s) = d.sensors.iter().find(|s| s.dead) {
		if shared.is_empty() {
			parts.push(format!(
				"Sensor {} is lit in {} of the sweeps that light its neighbours (usually {}): dead, dirty or covered.",
				s.sensor,
				pct(s.rate),
				pct(s.typical)
			));
			severity = 2;
		} else {
			parts.push(format!(
				"Sensor {} is lit in {} of the sweeps that light its neighbours (usually {}), and the same sensor is dark on the {} too: more likely covered by how they're worn than dead.",
				s.sensor,
				pct(s.rate),
				pct(s.typical),
				shared.join(" and the ")
			));
			severity = severity.max(1);
		}
	} else if let Some(s) = d.sensors.first() {
		parts.push(format!(
			"Sensor {} lights up less than its neighbours ({} vs {}).",
			s.sensor,
			pct(s.rate),
			pct(s.typical)
		));
		severity = severity.max(1);
	}
	// fewer sensors on one station than the other devices get
	if d.one_station_sensors > 0.0 && others.len() >= 2 {
		let mut o: Vec<f64> = others.to_vec();
		let typical = median(&mut o);
		if d.one_station_sensors < typical * 0.6 && d.seen_by[1] > 0.05 {
			parts.push(format!(
				"With one station alone it gets {:.1} sensors lit; the other devices get {:.1}.",
				d.one_station_sensors, typical
			));
			severity = severity.max(1);
		}
	}
	if d.lost > 0.05 || d.lost_worst > 0.3 {
		parts.push(format!("Radio: {} of its data lost (worst second {}).", pct(d.lost), pct(d.lost_worst)));
		severity = severity.max(if d.lost > 0.1 { 2 } else { 1 });
	}
	if parts.is_empty() {
		parts.push("Looks healthy: light from every side it faced, data arriving.".into());
	}
	d.headline = parts.join(" ");
	d.severity = severity;
}

const CSS: &str = r#"
:root{--bg:#16181c;--panel:#1e2126;--line:#2c3038;--text:#e6e8eb;--dim:#9aa1ab;--st:#5aa9e6;--rd:#e5534b;--lt:#d29922;--ok:#3fb950;--warn:#d29922;--bad:#e5534b}
@media (prefers-color-scheme: light){:root{--bg:#f6f7f9;--panel:#fff;--line:#dde1e6;--text:#1c2128;--dim:#5c6570}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1100px;margin:0 auto;padding:24px 16px 64px}h1{margin:0;font-size:26px}h2{margin:0 0 6px;font-size:20px}h3{margin:18px 0 6px;font-size:15px;color:var(--dim);font-weight:600}
header{margin-bottom:20px}.sub{margin:2px 0;color:var(--dim)}section{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:18px;margin:16px 0}
table{border-collapse:collapse;width:100%;font-size:14px}th,td{text-align:left;padding:6px 10px 6px 0;border-bottom:1px solid var(--line);vertical-align:top}th{color:var(--dim);font-weight:600}
.scroll{overflow-x:auto}.dim{color:var(--dim);font-size:13px}a{color:var(--st)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:8px 24px}
.headline{font-size:16px;margin:8px 0 12px;padding-left:10px;border-left:3px solid var(--ok)}.headline.s1{border-color:var(--warn)}.headline.s2{border-color:var(--bad)}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:7px;background:var(--ok)}.dot.s1{background:var(--warn)}.dot.s2{background:var(--bad)}
.tag{display:inline-block;padding:0 6px;border-radius:4px;font-size:12px;background:var(--line)}.tag.c0{background:var(--rd);color:#fff}.tag.c1,.tag.c2{background:var(--st);color:#fff}.tag.c3{background:var(--lt);color:#fff}
svg.tl{width:100%;min-width:640px;display:block}svg .lab{fill:var(--dim);font-size:11px}svg .ax{fill:var(--dim);font-size:10px}svg .bg{fill:var(--line);opacity:.5}
.st{fill:var(--st)}.rd{fill:var(--rd)}.lt{fill:var(--lt)}.l1{opacity:.3}.l2{opacity:.5}.l3{opacity:.75}.l4{opacity:1}
.room{fill:var(--text);opacity:.12}.room.radio{fill:var(--rd)}.note{color:var(--dim);font-size:14px;margin:4px 0}
.ev.k0{fill:var(--rd)}.ev.k1{fill:var(--lt)}.ev.k2{fill:#c678dd}
.legend{font-size:12px;color:var(--dim)}.key{display:inline-block;width:12px;height:10px;margin:0 4px 0 12px;vertical-align:-1px;background:currentColor}
.key.roomkey{background:var(--text);opacity:.3}.key.st{background:var(--st)}.key.rd{background:var(--rd)}.key.lt{background:var(--lt)}.key.ev.k0{background:var(--rd)}.key.ev.k1{background:var(--lt)}.key.ev.k2{background:#c678dd}
"#;

const HOW_TO_READ: &str = "<section><h2>How this was measured</h2><p>Everything comes from what each device sent over its radio, read next to SteamVR without changing anything, and from the poses SteamVR showed at the same time. Nothing here tracks: the light is counted, not solved.</p><ul><li><b>Data lost</b>: the IMU samples that never arrived, from the gaps in the device's own sample clock.</li><li><b>Station sees it</b>: at least 8 hits from that base station in a quarter second. Channels are numbered as SteamVR numbers them.</li><li><b>Sensors</b>: each sweep of a station across the device lights a set of sensors. A sensor whose neighbours (facing within 30 degrees of it) are lit should mostly be lit too. One that almost never is, while they are, is dead, dirty or covered (a strap, a clip, a sticker).</li><li><b>Light's age</b>: how much older the light is than the IMU sample before it when it arrives. A few ms is normal; a backed-up radio link holds light back for tens of ms or more.</li><li><b>SteamVR trouble</b>: from SteamVR's own poses: <i>lost</i> (not tracking), <i>froze</i> (position held while the device kept turning), <i>jumped</i> (moved further in one step than its velocity allows). Each is matched to the light and radio from a quarter second before it.</li></ul></section>";

#[cfg(test)]
mod tests {
	use super::*;

	/// A device lit by one station in every bin but `dark`
	fn device(name: &str, bins: usize, dark: std::ops::Range<usize>) -> Device {
		let mut d = Device::new(name, name);
		d.bins = (0..bins)
			.map(|i| {
				let mut b = Bin { reports: 100, imu: 62, ..Bin::default() };
				if !dark.contains(&i) {
					b.hits[3] = 40;
					b.sensors[3] = 0b1111;
				}
				b
			})
			.collect();
		d
	}

	#[test]
	fn a_blackout_shared_by_most_devices_is_room_wide() {
		let mut a = Analyzer::new(Box::new(|_| None));
		a.devices =
			vec![device("A", 80, 40..48), device("B", 80, 40..48), device("C", 80, 41..47), device("D", 80, 0..0)];
		let r = room_events(&a);
		assert_eq!(r.len(), 1, "{r:?}");
		// three of the four are dark from 41 to 47 (two of four, at 40 and 47, is not enough; the fourth never is)
		assert_eq!((r[0].start, r[0].end, r[0].hit, r[0].of), (41, 47, 3, 4));
		assert!(r[0].light);
	}

	#[test]
	fn one_dark_device_is_not_room_wide() {
		let mut a = Analyzer::new(Box::new(|_| None));
		a.devices = vec![device("A", 80, 40..60), device("B", 80, 0..0), device("C", 80, 0..0)];
		assert!(room_events(&a).is_empty());
	}

	#[test]
	fn heights_name_the_body_part() {
		assert_eq!(height_words(0.09), "foot");
		assert_eq!(height_words(0.96), "waist");
		assert_eq!(height_words(1.52), "head");
	}

	#[test]
	fn the_recordings_own_text_is_escaped_in_the_page() {
		let mut a = Analyzer::new(Box::new(|_| None));
		a.start_us = Some(0);
		a.devices = vec![device("A", 80, 0..0)];
		// the station on channel 3 (SteamVR's mode label 4), as a devices.txt from anyone may name it
		let station = crate::session::SteamDevice {
			serial: "<script>x</script>".into(),
			class: "station".into(),
			mode: "4".into(),
			..Default::default()
		};
		let s = Session { devices: vec![station], poses: Default::default(), sally: false };
		let h = Report::new(&a, &s).html("t");
		assert!(!h.contains("<script>"), "{h}");
		// the header's list of stations, and the light table's
		assert_eq!(h.matches("ch 4 (&lt;script&gt;x&lt;/script&gt;)").count(), 2);
	}
}
