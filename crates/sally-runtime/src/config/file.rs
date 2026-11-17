//! The config file: finding it (survive_config_file_name in survive.c, survive_config_file_path), reading it into the
//! store (config_read and its json callbacks) and writing it back (config_save, write_config_group), from
//! survive_config.c.
//!
//! Paths are bytes, as the C's char strings are; they become a std path only at the file system (on Windows through
//! UTF-8, where the C's are in the ANSI code page).

use std::io::Write;
use std::path::{Path, PathBuf};

use super::cnum::{self, c_str};
use super::json::{self, JsonCallbacks};
use super::store::{Config, ConfigGroup, EntryType, NUM_GEN2_LIGHTHOUSES, SC_GET};

/// FILENAME_MAX: the size of the C's path buffers (glibc's 4096, the ucrt's 260)
#[cfg(windows)]
pub const FILENAME_MAX: usize = 260;
#[cfg(not(windows))]
pub const FILENAME_MAX: usize = 4096;

/// DEFAULT_CONFIG_PATH (survive.c): the "configfile" when none is set
pub const DEFAULT_CONFIG_PATH: &str = "config.json";

/// What config_save did
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SaveOutcome {
	/// The file was written (write errors are ignored, as the C's fprintf results are)
	Written,
	/// config-readonly or lock-lighthouses is set: nothing written. `first`: this is the first time, when the C logs
	/// it (at verbosity 10)
	ReadOnly { first: bool },
	/// The file could not be opened for writing. `warn`: the C warns (once, and never for /dev/null)
	OpenFailed { path: Vec<u8>, warn: bool },
}

impl SaveOutcome {
	/// The line the C logs for this outcome, if any: SV_VERBOSE(10, ..) for ReadOnly, SV_WARN(..) for OpenFailed
	pub fn message(&self) -> Option<Vec<u8>> {
		match self {
			SaveOutcome::ReadOnly { first: true } => {
				Some(b"config file not written (config-readonly / lock-lighthouses)".to_vec())
			}
			SaveOutcome::OpenFailed { path, warn: true } => {
				let mut m = b"Could not open '".to_vec();
				m.extend_from_slice(&path[..path.len().min(512)]);
				m.extend_from_slice(
					b"' for writing; settings and calibration will not persist. This typically happens if the path \
					  doesn't exist or root owns the file.",
				);
				Some(m)
			}
			_ => None,
		}
	}
}

/// sally: `text` into the file at `path` whole or not at all: written to `<path>.tmp`, which is then renamed over the
/// file, so that a run killed while it writes leaves the old file as it was (the C's fopen(path, "w") empties it
/// first). Where that cannot be done it is written in place as the C writes it: a path that is not a plain file (a
/// link, a device), a file fopen could not open for writing (it stays as it is, and the error is the caller's to
/// report), a temporary that cannot be made or written, or a rename that fails (on Windows, while another process has
/// the file open; the temporary is removed). Err only when the file cannot be opened for writing in place either
pub fn write_whole(path: &Path, text: &[u8]) -> std::io::Result<()> {
	let replace = match std::fs::symlink_metadata(path) {
		Ok(m) => m.file_type().is_file() && std::fs::OpenOptions::new().write(true).open(path).is_ok(),
		Err(_) => true,
	};
	if replace {
		let mut tmp = path.as_os_str().to_owned();
		tmp.push(".tmp");
		let tmp = PathBuf::from(tmp);
		if let Ok(mut f) = std::fs::File::create(&tmp) {
			let written = f.write_all(text).is_ok();
			drop(f);
			if let Ok(m) = std::fs::metadata(path) {
				// the file's permissions go with it
				let _ = std::fs::set_permissions(&tmp, m.permissions());
			}
			if written && std::fs::rename(&tmp, path).is_ok() {
				return Ok(());
			}
			let _ = std::fs::remove_file(&tmp);
		}
	}
	// write errors are ignored, as the C's fprintf results are
	let _ = std::fs::File::create(path)?.write_all(text);
	Ok(())
}

/// A path the C would hand fopen or mkdir
fn os_path(path: &[u8]) -> PathBuf {
	#[cfg(unix)]
	{
		use std::os::unix::ffi::OsStrExt;
		PathBuf::from(std::ffi::OsStr::from_bytes(path))
	}
	#[cfg(not(unix))]
	{
		PathBuf::from(String::from_utf8_lossy(path).into_owned())
	}
}

fn getenv(name: &str) -> Option<Vec<u8>> {
	let value = std::env::var_os(name)?;
	#[cfg(unix)]
	{
		use std::os::unix::ffi::OsStringExt;
		Some(value.into_vec())
	}
	#[cfg(not(unix))]
	{
		Some(value.to_string_lossy().into_owned().into_bytes())
	}
}

/// mkdir(path, 0755) on Linux, _mkdir(path) on Windows, nothing elsewhere (as the C), errors ignored
fn make_dir(path: &[u8]) {
	#[cfg(target_os = "linux")]
	{
		use std::os::unix::fs::DirBuilderExt;
		let _ = std::fs::DirBuilder::new().mode(0o755).create(os_path(path));
	}
	#[cfg(windows)]
	{
		let _ = std::fs::create_dir(os_path(path));
	}
	#[cfg(not(any(target_os = "linux", windows)))]
	let _ = path;
}

/// load_file_to_mem: the file's bytes up to the first NUL (json_load_file takes strlen of them), None if fopen fails.
/// On Windows the C reads in text mode, so a CR before an LF is dropped (which only shows inside a quoted value; jsmn
/// takes CR for white space). The ucrt's text mode also stops at a Ctrl-Z byte; that is not reproduced
fn load_file(path: &[u8]) -> Option<Vec<u8>> {
	let bytes = std::fs::read(os_path(path)).ok()?;
	#[cfg(windows)]
	let bytes = {
		let mut text = Vec::with_capacity(bytes.len());
		for (i, &b) in bytes.iter().enumerate() {
			if !(b == b'\r' && bytes.get(i + 1) == Some(&b'\n')) {
				text.push(b);
			}
		}
		text
	};
	Some(c_str(&bytes).to_vec())
}

/// Which group the object being read goes into: an entry of cg_stack
#[derive(Clone, Copy, Debug)]
enum Target {
	Global,
	Lighthouse(usize),
	/// "lighthouseN" for an N outside 0..16: the C points past lh_config there; the port drops what it holds
	Nowhere,
}

/// config_read's callbacks and the statics they share (cg_stack, cg_stack_head, array_size, array_data)
struct Reader<'a> {
	config: &'a mut Config,
	/// cg_stack up to cg_stack_head, the global group at the bottom. The C's stack holds 10; a file nesting objects
	/// deeper overflows it there
	stack: Vec<Target>,
	/// array_size: 0 outside an array, else 1 + the number of values read in it
	array_size: usize,
	/// array_data: the values of the array being read
	array_values: Vec<Vec<u8>>,
}

impl Reader<'_> {
	fn group(config: &mut Config, target: Target) -> Option<&mut ConfigGroup> {
		match target {
			Target::Global => Some(&mut config.global),
			Target::Lighthouse(i) => Some(&mut config.lighthouses[i]),
			Target::Nowhere => None,
		}
	}

	fn target(&self) -> Target {
		self.stack.last().copied().unwrap_or(Target::Global)
	}
}

/// parse_uint32: every value a whole base-10 strtoul (an empty text is 0), then the first value set as a uint32 (an
/// array of integers keeps only its first). With no values the C sets an uninitialized uint32 (alloca(0)); here 0
fn parse_uint32(group: Option<&mut ConfigGroup>, tag: &[u8], values: &[&[u8]]) -> bool {
	if values.iter().any(|v| cnum::strtoul(v, 10).1 != v.len()) {
		return false;
	}
	let first = values.first().map_or(0, |v| cnum::uint32_of_ulong(cnum::strtoul(v, 10).0));
	if let Some(g) = group {
		g.set_uint32(tag, first);
	}
	true
}

/// parse_floats: every value a whole strtof (not strtod: the C builds without USE_DOUBLE, so a value read from the
/// file has a float's precision), then an array of two or more set as a float array, one value as a float. With no
/// values the C sets an uninitialized double; here 0
fn parse_floats(group: Option<&mut ConfigGroup>, tag: &[u8], values: &[&[u8]]) -> bool {
	if values.iter().any(|v| cnum::strtof(v).1 != v.len()) {
		return false;
	}
	let floats: Vec<f64> = values.iter().map(|v| cnum::to_double(cnum::strtof(v).0)).collect();
	if let Some(g) = group {
		if floats.len() > 1 {
			g.set_float_array(tag, &floats);
		} else {
			g.set_float(tag, floats.first().copied().unwrap_or(0.0));
		}
	}
	true
}

impl JsonCallbacks for Reader<'_> {
	/// handle_config_group: an object whose tag sscanf's as "lighthouse%d" reads into that lighthouse's group, any
	/// other into the global group (a nested object's entries land at the top level)
	fn begin_object(&mut self, tag: &[u8]) {
		let target = match tag.strip_prefix(b"lighthouse").and_then(cnum::scan_int) {
			Some(i) if (0..NUM_GEN2_LIGHTHOUSES as i32).contains(&i) => Target::Lighthouse(i as usize),
			Some(_) => Target::Nowhere,
			None => Target::Global,
		};
		self.stack.push(target);
	}

	/// pop_config_group
	fn end_object(&mut self, _tag: &[u8]) {
		self.stack.pop();
	}

	/// handle_array_start
	fn begin_array(&mut self, _tag: &[u8]) {
		self.array_size = 1;
	}

	/// handle_array_end: the values as integers first ("stricter rules"), else as floats; an array of anything else is
	/// not stored. Nothing happens until some array has had a value (the C tests array_data, a static never freed, for
	/// NULL): after that an empty array sets the tag to the uninitialized uint32 of parse_uint32 with no values. The
	/// counts are cut as the C's parameters are (uint16_t for parse_uint32, uint8_t for parse_floats). An array inside
	/// an array leaves the C's array_size at 0 when the outer one ends, and the C then reads far past array_data; the
	/// port stores nothing for it
	fn end_array(&mut self, tag: &[u8]) {
		if self.config.array_value_read && self.array_size > 0 {
			let n = self.array_size - 1;
			let target = self.target();
			let values: Vec<&[u8]> = self.array_values.iter().map(Vec::as_slice).collect();
			let as_integers = &values[..(n as u16 as usize).min(values.len())];
			if !parse_uint32(Reader::group(self.config, target), tag, as_integers) {
				let as_floats = &values[..(n as u8 as usize).min(values.len())];
				parse_floats(Reader::group(self.config, target), tag, as_floats);
			}
		}
		self.array_size = 0;
	}

	/// handle_tag_value: inside an array the value is kept for the array's end (whatever its tag); otherwise it is set
	/// as a uint32 if it is a whole base-10 integer, else as a float if it is a whole float, else as a string
	fn tag_value(&mut self, tag: &[u8], value: &[u8]) {
		if self.array_size > 0 {
			let i = self.array_size - 1;
			self.array_values.truncate(i);
			self.array_values.push(value.to_vec());
			self.array_size += 1;
			self.config.array_value_read = true;
			return;
		}
		let target = self.target();
		if parse_uint32(Reader::group(self.config, target), tag, &[value]) {
			return;
		}
		if parse_floats(Reader::group(self.config, target), tag, &[value]) {
			return;
		}
		// "should probably also handle string arrays"
		if let Some(g) = Reader::group(self.config, target) {
			g.set_str(tag, Some(value));
		}
	}
}

/// write_config_group: every entry as "tag":value, in order, a comma after each but the last, one per line; a named
/// group inside "tag":{ and }
fn write_config_group(out: &mut Vec<u8>, group: &ConfigGroup, tag: Option<&[u8]>) {
	if let Some(tag) = tag {
		out.push(b'"');
		out.extend_from_slice(tag);
		out.extend_from_slice(b"\":{\n");
	}
	let n = group.entries().len();
	for (i, e) in group.entries().iter().enumerate() {
		match e.kind() {
			EntryType::Float => json::write_float(out, e.tag(), cnum::to_float(e.float())),
			EntryType::Uint32 => json::write_uint32(out, e.tag(), e.uint32()),
			EntryType::String => json::write_str(out, e.tag(), &e.data_c_str().unwrap_or_default()),
			EntryType::FloatArray => json::write_double_array(out, e.tag(), &e.floats()),
		}
		if i + 1 < n {
			out.push(b',');
		}
		out.push(b'\n');
	}
	if tag.is_some() {
		out.extend_from_slice(b"}\n");
	}
}

impl Config {
	/// survive_config_file_name (survive.c): "configfile", or DEFAULT_CONFIG_PATH. None where the C's pointer is
	/// NULL ("configfile" set as a number, as `--configfile` with no value after it does)
	pub fn file_name(&self) -> Option<Vec<u8>> {
		self.get_str(b"configfile", Some(DEFAULT_CONFIG_PATH.as_bytes()))
	}

	/// survive_config_file_path: the config file's path. A name starting with a letter is a bare name (sally: on
	/// Windows "C:..." is not) and lives in $XDG_CONFIG_HOME/sally, else $HOME/.config/sally, else $LOCALAPPDATA/sally
	/// (that directory is created; with none of the three the path is "/name"); anything else is a path, used as it is,
	/// its directory created (on Linux; the C's Windows build creates the wrong path there, to no effect). The C
	/// crashes on a NULL name; the port takes it as "". (sally builds without SURVIVE_CONFIG_PATH.)
	pub fn file_path(&self) -> Vec<u8> {
		self.file_path_in(&getenv)
	}

	pub(super) fn file_path_in(&self, getenv: &dyn Fn(&str) -> Option<Vec<u8>>) -> Vec<u8> {
		let configpath = self.file_name().unwrap_or_default();
		#[allow(unused_mut)] // reassigned on Windows only
		let mut bare_name = configpath.first().is_some_and(u8::is_ascii_alphabetic);
		#[cfg(windows)]
		if bare_name && configpath.get(1) == Some(&b':') {
			bare_name = false;
		}
		if bare_name {
			let mut path = Vec::new();
			if let Some(home) = getenv("XDG_CONFIG_HOME") {
				path.extend_from_slice(c_str(&home));
				path.extend_from_slice(b"/sally");
			} else if let Some(home) = getenv("HOME") {
				path.extend_from_slice(c_str(&home));
				path.extend_from_slice(b"/.config/sally");
			} else if let Some(home) = getenv("LOCALAPPDATA") {
				path.extend_from_slice(c_str(&home));
				path.extend_from_slice(b"/sally");
			}
			path.truncate(FILENAME_MAX - 1);
			make_dir(&path);
			path.push(b'/');
			path.extend_from_slice(&configpath);
			path.truncate(FILENAME_MAX - 1);
			return path;
		}
		// the directory: up to the last '/' after the first byte of the name's first 511 bytes
		let pathname = &configpath[..configpath.len().min(511)];
		if let Some(i) = (1..pathname.len()).rev().find(|&i| pathname[i] == b'/') {
			#[cfg(target_os = "linux")]
			make_dir(&pathname[..i]);
			#[cfg(not(target_os = "linux"))]
			let _ = i;
		}
		configpath
	}

	/// config_read: reads the file at `init_path` (cut to FILENAME_MAX - 1 bytes), or at file_path() for None, into
	/// the store. A missing file, or one jsmn cannot tokenize, reads nothing
	pub fn read(&mut self, init_path: Option<&[u8]>) {
		let path = match init_path {
			Some(p) => {
				let p = c_str(p);
				p[..p.len().min(FILENAME_MAX - 1)].to_vec()
			}
			None => self.file_path(),
		};
		if let Some(text) = load_file(&path) {
			self.read_text(&text);
		}
	}

	/// What config_read does with the file's text (json_run_callbacks with its callbacks), the text taken up to its
	/// first NUL as the C's strlen does. The top level goes into the global group, a "lighthouseN" object into
	/// lighthouse N's, any other object into the global group too
	pub fn read_text(&mut self, text: &[u8]) {
		let mut reader = Reader { config: self, stack: vec![Target::Global], array_size: 0, array_values: Vec::new() };
		let _ = json::json_run_callbacks(&mut reader, c_str(text));
	}

	/// The file config_save writes: the global group's entries (no braces, no comma after the last), then the group
	/// of each lighthouse whose OOTXSet is set (`ootx_set`, from the runtime's station state), as "lighthouseN":{..}.
	/// On Windows the C's file is in text mode, so every LF is written as CR LF
	pub fn save_text(&self, ootx_set: &[bool; NUM_GEN2_LIGHTHOUSES]) -> Vec<u8> {
		let mut out = Vec::new();
		write_config_group(&mut out, &self.global, None);
		for (i, set) in ootx_set.iter().enumerate() {
			if *set {
				write_config_group(&mut out, &self.lighthouses[i], Some(format!("lighthouse{i}").as_bytes()));
			}
		}
		#[cfg(windows)]
		let out = {
			let mut text = Vec::with_capacity(out.len() + out.len() / 16);
			for b in out {
				if b == b'\n' {
					text.push(b'\r');
				}
				text.push(b);
			}
			text
		};
		out
	}

	/// config_save: writes save_text to file_path(), unless "config-readonly" or "lock-lighthouses" is set (a wiped or
	/// re-solved map must not replace the calibrated one on disk), and whole or not at all (`write_whole`: a run killed
	/// while it writes keeps the old file)
	pub fn save(&mut self, ootx_set: &[bool; NUM_GEN2_LIGHTHOUSES]) -> SaveOutcome {
		self.save_in(ootx_set, &getenv)
	}

	pub(super) fn save_in(
		&mut self, ootx_set: &[bool; NUM_GEN2_LIGHTHOUSES], getenv: &dyn Fn(&str) -> Option<Vec<u8>>,
	) -> SaveOutcome {
		if self.configi("config-readonly", SC_GET, 0) != 0 || self.configi("lock-lighthouses", SC_GET, 0) != 0 {
			let first = !self.save_readonly_said;
			self.save_readonly_said = true;
			return SaveOutcome::ReadOnly { first };
		}
		let path = self.file_path_in(getenv);
		match write_whole(&os_path(&path), &self.save_text(ootx_set)) {
			Ok(()) => SaveOutcome::Written,
			Err(_) => {
				let warn = !self.save_open_warned && path != b"/dev/null";
				if warn {
					self.save_open_warned = true;
				}
				SaveOutcome::OpenFailed { path, warn }
			}
		}
	}
}
