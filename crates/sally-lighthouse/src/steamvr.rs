//! SteamVR's folders, as its path registry names them (%LOCALAPPDATA%\openvr\openvrpaths.vrpath), and what sally reads
//! in them: its own copy of every lighthouse device's config (Steam\config\lighthouse\<serial>\config.json, which
//! Valve's driver writes when it first connects a device, so any device SteamVR has seen has one), the trackers' roles
//! and the server log. Read-only. Used by sally-check (its recordings) and by the runtime (a device's config by its
//! serial, wherever it is paired).

use std::path::{Path, PathBuf};

/// This user's path registry, where %LOCALAPPDATA% is set (the file may not be there: no SteamVR has run)
pub fn registry_file() -> Option<PathBuf> {
	let local = std::env::var_os("LOCALAPPDATA")?;
	Some(Path::new(&local).join("openvr").join("openvrpaths.vrpath"))
}

/// SteamVR's folders, from the path registry
#[derive(Clone, Debug, Default)]
pub struct Paths {
	pub runtime: Option<PathBuf>,
	pub config: Option<PathBuf>,
	pub log: Option<PathBuf>,
}

impl Paths {
	/// This user's registry (none where %LOCALAPPDATA% or the file is missing)
	pub fn find() -> Paths {
		registry_file().map(|f| Paths::from_registry(&f)).unwrap_or_default()
	}

	/// The folders a registry file names (the first of each list)
	pub fn from_registry(file: &Path) -> Paths {
		let text = std::fs::read_to_string(file).unwrap_or_default();
		let first = |key: &str| json_first_string(&text, key).map(PathBuf::from);
		Paths { runtime: first("runtime"), config: first("config"), log: first("log") }
	}

	/// SteamVR's own copy of a device's config (Steam\config\lighthouse\<serial>\config.json)
	pub fn device_config(&self, serial: &str) -> Option<Vec<u8>> {
		std::fs::read(self.device_config_path(serial)?).ok()
	}

	/// Where SteamVR keeps a device's config (whether it is there or not)
	pub fn device_config_path(&self, serial: &str) -> Option<PathBuf> {
		Some(self.config.as_ref()?.join("lighthouse").join(serial.to_ascii_lowercase()).join("config.json"))
	}

	/// The trackers' roles from SteamVR's settings ("TrackerRole_Waist"), for a serial
	pub fn tracker_role(&self, settings: &str, serial: &str) -> Option<String> {
		let _ = self;
		let key = format!("{serial}\"");
		let mut from = 0;
		while let Some(i) = settings[from..].find(&key) {
			let rest = settings[from + i + key.len()..].trim_start();
			if let Some(rest) = rest.strip_prefix(':') {
				let rest = rest.trim_start();
				if let Some(v) = rest.strip_prefix('"').and_then(|r| r.split('"').next()) {
					if v.starts_with("TrackerRole_") {
						return Some(v.to_string());
					}
				}
			}
			from += i + key.len();
		}
		None
	}

	pub fn settings(&self) -> String {
		self.config
			.as_ref()
			.and_then(|c| std::fs::read_to_string(c.join("steamvr.vrsettings")).ok())
			.unwrap_or_default()
	}

	pub fn server_log(&self) -> Option<PathBuf> {
		self.log.as_ref().map(|l| l.join("vrserver.txt"))
	}
}

/// The first string of a key's array (or the key's string) in a small JSON file, with JSON's escapes undone
pub fn json_first_string(text: &str, key: &str) -> Option<String> {
	let i = text.find(&format!("\"{key}\""))?;
	let rest = &text[i + key.len() + 2..];
	let q = rest.find('"')?;
	// a null or an empty array before the next string means none
	let between = &rest[..q];
	if between.contains("null") || between.contains(']') {
		return None;
	}
	let mut out = String::new();
	let mut chars = rest[q + 1..].chars();
	while let Some(c) = chars.next() {
		match c {
			'"' => return Some(out),
			'\\' => match chars.next()? {
				'n' => out.push('\n'),
				't' => out.push('\t'),
				'u' => {
					let hex: String = chars.by_ref().take(4).collect();
					out.push(char::from_u32(u32::from_str_radix(&hex, 16).ok()?)?);
				}
				c => out.push(c),
			},
			c => out.push(c),
		}
	}
	None
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_path_registry_is_read() {
		let text = r#"{
	"config" : [ "C:\\Program Files (x86)\\Steam\\config" ],
	"external_drivers" : null,
	"jsonid" : "vrpathreg",
	"log" : [ "C:\\Program Files (x86)\\Steam\\logs" ],
	"runtime" : [ "C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR" ],
	"version" : 1
}"#;
		assert_eq!(
			json_first_string(text, "runtime").as_deref(),
			Some(r"C:\Program Files (x86)\Steam\steamapps\common\SteamVR")
		);
		assert_eq!(json_first_string(text, "external_drivers"), None);
	}

	#[test]
	fn a_registry_file_names_the_folders_and_a_device_config_is_found_by_serial() {
		let dir = std::env::temp_dir().join(format!("sally_steamvr_{}", std::process::id()));
		let cfg = dir.join("config");
		std::fs::create_dir_all(cfg.join("lighthouse").join("lhr-239fd4b3")).unwrap();
		std::fs::write(cfg.join("lighthouse").join("lhr-239fd4b3").join("config.json"), b"{}").unwrap();
		let reg = dir.join("openvrpaths.vrpath");
		let text = format!(
			"{{\n\t\"config\" : [ \"{}\" ],\n\t\"version\" : 1\n}}",
			cfg.display().to_string().replace('\\', "\\\\")
		);
		std::fs::write(&reg, text).unwrap();
		let p = Paths::from_registry(&reg);
		assert_eq!(p.config.as_deref(), Some(cfg.as_path()));
		assert_eq!(p.device_config("LHR-239FD4B3").as_deref(), Some(&b"{}"[..]));
		assert_eq!(p.device_config("LHR-00000000"), None);
		assert_eq!(Paths::default().device_config("LHR-239FD4B3"), None);
		let _ = std::fs::remove_dir_all(&dir);
	}
}
