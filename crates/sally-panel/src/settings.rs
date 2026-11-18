//! The panel's settings, kept in `sally-panel.cfg` beside the program (plain "key value" lines, the C++ panel's
//! sally_panel.cfg format). Where there is no sally-panel.cfg yet the C++ panel's sally_panel.cfg is read, so the
//! names, roles and view choices made there carry over; only sally-panel.cfg is ever written.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub const FILE: &str = "sally-panel.cfg";
/// The C++ panel's, read when FILE is not there
pub const CPP_FILE: &str = "sally_panel.cfg";

#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
	pub host: String,
	pub port: u16,
	/// serial -> role key (see ui::roles); missing = auto
	pub roles: BTreeMap<String, String>,
	/// serial -> the name given in the panel; missing = the default name
	pub names: BTreeMap<String, String>,
	/// NaN = auto (the lowest point seen)
	pub floor_z: f64,
	pub frustum_len: f64,
	pub trail_seconds: f64,
	pub show_trails: bool,
	pub show_sensors: bool,
	pub show_frustums: bool,
	pub show_rays: bool,
	pub show_labels: bool,
	pub show_valve: bool,
	/// the coverage heat map on the floor grid
	pub show_heat: bool,
	/// the devices drawn as their shape (the outline of their sensors)
	pub show_models: bool,
	/// SteamVR's chaperone's walls in the room
	pub show_walls: bool,
	pub compare: bool,
	pub follow: bool,
	/// 0 tracking, 1 replay, 2 output, 3 settings (the file has its name)
	pub tab: i32,
	/// 0 important, 1 all
	pub events_filter: i32,
	/// the device page's range (s)
	pub page_range: f64,
	/// the SteamVR dashboard overlay: the same panel, this wide in VR (m)
	pub overlay: bool,
	pub overlay_width: f64,
	/// cfg only: flip the laser's y if SteamVR ever reports it from the bottom
	pub overlay_flip_pointer: bool,
	/// cfg only: SteamVR's folder for the devices' models ("" = the one openvrpaths.vrpath names)
	pub steamvr: String,
	/// cfg only: the project folder the replays read (logs, the driver's settings; "" = two folders up from the panel)
	pub replay_project: String,
	/// the replay's tracks shown: Valve, what SteamVR got, the replay
	pub replay_tracks: [bool; 3],

	/// the file's host and port while the command line points this run at another runtime: written in place of
	/// `host` and `port`, so a command line's choice is for its run only
	pub file_target: Option<(String, u16)>,
	/// where it is written
	pub path: PathBuf,
	pub dirty: bool,
}

impl Default for Settings {
	fn default() -> Settings {
		Settings {
			host: "127.0.0.1".into(),
			port: 9100,
			roles: BTreeMap::new(),
			names: BTreeMap::new(),
			floor_z: f64::NAN,
			frustum_len: 3.0,
			trail_seconds: 6.0,
			show_trails: true,
			show_sensors: true,
			show_frustums: true,
			show_rays: true,
			show_labels: true,
			show_valve: true,
			show_heat: false,
			show_models: true,
			show_walls: true,
			compare: true,
			follow: false,
			tab: 0,
			events_filter: 0,
			page_range: 30.0,
			overlay: true,
			overlay_width: 2.5,
			overlay_flip_pointer: false,
			steamvr: String::new(),
			replay_project: String::new(),
			replay_tracks: [true, true, true],
			file_target: None,
			path: PathBuf::new(),
			dirty: false,
		}
	}
}

/// The tabs as the file names them, by their number
const TABS: [&str; 5] = ["tracking", "replay", "output", "settings", "check"];

/// A file's `tab`: its name, or a number as the panel wrote it until 2.0 (0 tracking, 1 replay, 2 settings, 3
/// advanced, which is in Settings now)
fn tab_of(v: &str) -> i32 {
	let v = v.trim();
	// a file from when the punch settings had a tab of their own: they are the Output tab's
	if v == "fun" {
		return 2;
	}
	match TABS.iter().position(|t| *t == v) {
		Some(i) => i as i32,
		None => match atoi(v) {
			1 => 1,
			2 | 3 => 3,
			_ => 0,
		},
	}
}

fn atof(s: &str) -> f64 {
	sally_math::c_atof(s.as_bytes())
}

fn atoi(s: &str) -> i32 {
	sally_math::c_atoi(s.as_bytes())
}

impl Settings {
	/// The settings in `dir` (sally-panel.cfg, else the C++ panel's sally_panel.cfg), written back to sally-panel.cfg
	pub fn load_dir(dir: &Path) -> Settings {
		let path = dir.join(FILE);
		let mut st = Settings { path: path.clone(), ..Settings::default() };
		if let Ok(text) = std::fs::read(&path).or_else(|_| std::fs::read(dir.join(CPP_FILE))) {
			st.parse(&String::from_utf8_lossy(&text));
		}
		st
	}

	/// The settings in one file
	pub fn load(path: &Path) -> Settings {
		let mut st = Settings { path: path.into(), ..Settings::default() };
		if let Ok(text) = std::fs::read(path) {
			st.parse(&String::from_utf8_lossy(&text));
		}
		st
	}

	fn parse(&mut self, text: &str) {
		for line in text.split('\n') {
			let line = line.split(['\r', '\n']).next().unwrap_or("");
			if line.is_empty() || line.starts_with('#') {
				continue;
			}
			let Some((k, v)) = line.split_once(' ') else { continue };
			let b = |v: &str| v == "1" || v == "true" || v == "on";
			match k {
				"host" => self.host = v.into(),
				"port" => self.port = atoi(v).clamp(0, 65535) as u16,
				"floor" => self.floor_z = if v == "auto" { f64::NAN } else { atof(v) },
				"frustum_len" => self.frustum_len = atof(v),
				"trail_seconds" => self.trail_seconds = atof(v),
				"trails" => self.show_trails = b(v),
				"sensors" => self.show_sensors = b(v),
				"frustums" => self.show_frustums = b(v),
				"rays" => self.show_rays = b(v),
				"labels" => self.show_labels = b(v),
				"valve_trails" => self.show_valve = b(v),
				"heat" => self.show_heat = atoi(v) != 0,
				"models" => self.show_models = b(v),
				"walls" => self.show_walls = b(v),
				"compare" => self.compare = b(v),
				"follow" => self.follow = b(v),
				"tab" => self.tab = tab_of(v),
				"events_filter" => self.events_filter = atoi(v),
				"page_range" => self.page_range = atof(v),
				"overlay" => self.overlay = b(v),
				"overlay_flip_pointer" => self.overlay_flip_pointer = b(v),
				"overlay_width" => self.overlay_width = atof(v).clamp(1.0, 5.0),
				"steamvr" => self.steamvr = v.trim().into(),
				"replay_project" => self.replay_project = v.trim().into(),
				"replay_tracks" => {
					let c: Vec<char> = v.chars().collect();
					for (i, t) in self.replay_tracks.iter_mut().enumerate() {
						*t = c.get(i) != Some(&'0');
					}
				}
				_ => {
					if let Some(serial) = k.strip_prefix("role.") {
						self.roles.insert(serial.into(), v.into());
					} else if let Some(serial) = k.strip_prefix("name.") {
						// the rest of the line: a name may have spaces
						if !v.is_empty() {
							self.names.insert(serial.into(), v.into());
						}
					}
				}
			}
		}
	}

	/// The file's text
	pub fn text(&self) -> String {
		let b = |v: bool| v as i32;
		let mut s = String::from("# sally panel settings (written by the panel)\n");
		let (host, port) = match &self.file_target {
			Some((h, p)) => (h.as_str(), *p),
			None => (self.host.as_str(), self.port),
		};
		s += &format!("host {host}\nport {port}\n");
		if self.floor_z.is_nan() {
			s += "floor auto\n";
		} else {
			s += &format!("floor {:.4}\n", self.floor_z);
		}
		s += &format!("frustum_len {:.2}\ntrail_seconds {:.1}\n", self.frustum_len, self.trail_seconds);
		s += &format!(
			"trails {}\nsensors {}\nfrustums {}\nrays {}\nlabels {}\nvalve_trails {}\nmodels {}\n",
			b(self.show_trails),
			b(self.show_sensors),
			b(self.show_frustums),
			b(self.show_rays),
			b(self.show_labels),
			b(self.show_valve),
			b(self.show_models)
		);
		s += &format!("heat {}\ncompare {}\nfollow {}\n", b(self.show_heat), b(self.compare), b(self.follow));
		s += &format!("walls {}\n", b(self.show_walls));
		s += &format!(
			"tab {}\nevents_filter {}\npage_range {:.0}\n",
			TABS[self.tab.clamp(0, TABS.len() as i32 - 1) as usize],
			self.events_filter,
			self.page_range
		);
		s += &format!(
			"overlay {}\noverlay_width {:.2}\noverlay_flip_pointer {}\n",
			b(self.overlay),
			self.overlay_width,
			b(self.overlay_flip_pointer)
		);
		let tracks: String = self.replay_tracks.iter().map(|&t| if t { '1' } else { '0' }).collect();
		s += &format!("replay_tracks {tracks}\n");
		if !self.steamvr.is_empty() {
			s += &format!("steamvr {}\n", self.steamvr);
		}
		if !self.replay_project.is_empty() {
			s += &format!("replay_project {}\n", self.replay_project);
		}
		for (k, v) in &self.roles {
			s += &format!("role.{k} {v}\n");
		}
		for (k, v) in &self.names {
			s += &format!("name.{k} {v}\n");
		}
		s
	}

	/// Writes the file (nothing without a path); the settings are clean after a write that worked
	pub fn save(&mut self) {
		if self.path.as_os_str().is_empty() {
			return;
		}
		if std::fs::write(&self.path, self.text()).is_ok() {
			self.dirty = false;
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// sally_panel.cfg as the earlier C++ panel wrote it (still read where there is no newer file)
	const CPP: &str = "# sally panel settings (written by the panel)\nhost 127.0.0.1\nport 9100\nfloor 0.0760\n\
		frustum_len 3.00\ntrail_seconds 6.0\ntrails 1\nsensors 1\nfrustums 1\nrays 1\nlabels 1\nvalve_trails 0\n\
		heat 1\ncompare 1\nfollow 0\ntab 1\nevents_filter 0\noverlay 1\noverlay_width 2.50\noverlay_flip_pointer 0\n\
		background sky.jpg\nbackground_dim 0.12\nglass_tint 0.32\nrole.LHR-0334A73B head\n\
		name.LHR-0334A73B Head\nname.LHR-2D779713 Tracker one\n";

	#[test]
	fn the_cpp_panels_file_reads() {
		let mut st = Settings::default();
		st.parse(CPP);
		assert_eq!(st.floor_z, 0.076);
		assert!(st.show_heat && !st.show_valve && st.show_models);
		assert_eq!(st.tab, 1);
		assert_eq!(st.roles["LHR-0334A73B"], "head");
		assert_eq!(st.names["LHR-2D779713"], "Tracker one");
		// and its own lines come back from the text it writes
		let mut again = Settings::default();
		again.parse(&st.text());
		assert_eq!(again, st);
	}

	#[test]
	fn the_tab_is_kept_by_name_and_an_old_files_number_is_read() {
		let mut st = Settings::default();
		for (line, tab) in [
			("tab output", 2),
			("tab settings", 3),
			("tab fun", 2),
			("tab 1", 1),
			("tab 2", 3),
			("tab 3", 3),
			("tab 0", 0),
			("tab x", 0),
		] {
			st.parse(line);
			assert_eq!(st.tab, tab, "{line}");
		}
		st.tab = 2;
		assert!(st.text().contains("\ntab output\n"), "{}", st.text());
	}

	#[test]
	fn crlf_and_bad_values() {
		let mut st = Settings::default();
		st.parse("port 70000\r\noverlay_width 9\r\nfloor auto\r\nname.X \r\nbogus\r\n");
		assert_eq!(st.port, 65535);
		assert_eq!(st.overlay_width, 5.0);
		assert!(st.floor_z.is_nan());
		assert!(st.names.is_empty());
	}

	#[test]
	fn the_old_file_is_read_and_the_new_one_written() {
		let dir = std::env::temp_dir().join(format!("sally-panel-settings-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		std::fs::write(dir.join(CPP_FILE), CPP).unwrap();
		let mut st = Settings::load_dir(&dir);
		assert_eq!(st.names.len(), 2);
		st.names.insert("LHR-1B80952C".into(), "Left foot spare".into());
		st.save();
		assert!(!st.dirty);
		assert_eq!(std::fs::read_to_string(dir.join(CPP_FILE)).unwrap(), CPP);
		let back = Settings::load_dir(&dir);
		assert_eq!(back.names["LHR-1B80952C"], "Left foot spare");
		std::fs::remove_dir_all(&dir).unwrap();
	}

	#[test]
	fn a_command_lines_runtime_is_not_written() {
		let mut st = Settings::default();
		st.parse(CPP);
		// sally-panel --host 10.0.0.2 --port 9177, as main.rs takes it
		st.file_target = Some((st.host.clone(), st.port));
		(st.host, st.port) = ("10.0.0.2".into(), 9177);
		st.names.insert("LHR-1B80952C".into(), "Left foot spare".into());
		let mut back = Settings::default();
		back.parse(&st.text());
		assert_eq!((back.host.as_str(), back.port), ("127.0.0.1", 9100));
		assert_eq!(back.names["LHR-1B80952C"], "Left foot spare");
	}
}
