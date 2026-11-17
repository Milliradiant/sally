//! The command line and the environment, into the temporary group, and the config file a recording or a playback
//! names: survive_init_internal's argument loop, survive_process_env and find_correct_config_file (survive.c).

use super::cnum::{self, c_str};
use super::store::{Config, SC_OVERRIDE, SC_SET};

/// The fields whose value names the config file when none was given ("playback_config_prefix_fields"): checked before
/// the file is read
pub const PLAYBACK_CONFIG_PREFIX_FIELDS: [&str; 4] = ["playback", "usbmon-playback", "hidtap-playback", "simulator"];

/// "record_config_prefix_fields": checked after the file is read, to save it under the recording's name
pub const RECORD_CONFIG_PREFIX_FIELDS: [&str; 3] = ["record", "usbmon-record", "hidtap-record"];

/// What the argument loop leaves for survive_init_internal to act on
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ArgsOutcome {
	/// `-h` (any argument starting so) or `--help`: survive_init_internal prints the help and returns no context
	pub showhelp: bool,
	/// `--version`
	pub showversion: bool,
	/// `-m`: list the parameters for shell completion; the (up to) three arguments after it (autocomplete_match). The
	/// loop stops at -m
	pub autocomplete: Option<[Option<Vec<u8>>; 3]>,
}

/// The process's arguments as main's argv has them (the program first)
pub fn arguments() -> Vec<Vec<u8>> {
	std::env::args_os().map(os_bytes).collect()
}

/// The process's environment as `environ` has it, in its order, each as (name, value)
pub fn environment() -> Vec<(Vec<u8>, Vec<u8>)> {
	std::env::vars_os().map(|(k, v)| (os_bytes(k), os_bytes(v))).collect()
}

/// An OS string's bytes: exact on Unix; on Windows UTF-8, where the C's argv and environ are in the ANSI code page
fn os_bytes(s: std::ffi::OsString) -> Vec<u8> {
	#[cfg(unix)]
	{
		use std::os::unix::ffi::OsStringExt;
		s.into_vec()
	}
	#[cfg(not(unix))]
	{
		s.to_string_lossy().into_owned().into_bytes()
	}
}

/// survive_process_env's reading of one variable: the tag a SURVIVE_ variable sets (its name after the prefix,
/// A-Z lowered; no '_' becomes '-', so SURVIVE_LIGHTHOUSE_GEN sets "lighthouse_gen", not "lighthouse-gen"), None for
/// any other variable, an empty tag, or a name over 32 bytes. At exactly 32 the C's 32-byte tag buffer has no NUL and
/// it reads on into the stack; here the tag is those 32 bytes
fn env_tag(name: &[u8]) -> Option<Vec<u8>> {
	let entry = c_str(name).strip_prefix(b"SURVIVE_")?;
	if entry.is_empty() || entry.len() > 32 {
		return None;
	}
	Some(entry.to_ascii_lowercase())
}

impl Config {
	/// survive_init_internal's loop over the command line, into the temporary group. `argv` is main's (`argv[0]`, the
	/// program, is skipped). Only arguments starting with '-' count: `--name` sets the option `name`, `-p` "poser",
	/// `-l` "lighthousecount", `-c` "configfile", `-v` "v" (other short options are ignored, and so is the rest of
	/// "-pxyz"). An option is a flag when it is the last argument or the next one starts with '-' and atoi reads it as
	/// 0 (so "-0.5" or "--x" after it make it a flag, "-5" is its value): a flag sets the uint32 1, or 0 for
	/// `--no-name` (which sets "name"), whatever the option's type. Otherwise the next argument is its value, set by
	/// survive_configs(SC_OVERRIDE | SC_SET): an int option takes atoi(value), a bool option atoi(value) != 0 (so
	/// "true" is false), a float option atof(value), a string option or an unregistered name the text. `--help` and
	/// `--version` set nothing. An option's name from before a rename sets the option under its name now
	/// (`current_name`: sally's, not the C's)
	pub fn apply_args<A: AsRef<[u8]>>(&mut self, argv: &[A]) -> ArgsOutcome {
		let args: Vec<&[u8]> = argv.iter().map(|a| c_str(a.as_ref())).collect();
		let at = |a: &[u8], i: usize| a.get(i).copied().unwrap_or(0);
		let mut out = ArgsOutcome::default();
		let mut av = 1;
		while av < args.len() {
			let arg = args[av];
			if at(arg, 0) == b'-' {
				let vartoupdate: Option<&[u8]> = match at(arg, 1) {
					b'-' => Some(&arg[2..]),
					b'h' => {
						out.showhelp = true;
						None
					}
					b'p' => Some(b"poser"),
					b'l' => Some(b"lighthousecount"),
					b'm' => {
						let next = |k: usize| args.get(av + k).map(|a| a.to_vec());
						out.autocomplete = Some([next(1), next(2), next(3)]);
						// "Eject immediately after processing -m"
						break;
					}
					b'c' => Some(b"configfile"),
					b'v' => Some(b"v"),
					_ => None,
				};
				if let Some(name) = vartoupdate {
					let is_last = av + 1 == args.len();
					let next_is_option = !is_last && at(args[av + 1], 0) == b'-' && cnum::atoi(args[av + 1]) == 0;
					let flag_argument = is_last || next_is_option;
					if name == b"help" {
						out.showhelp = true;
					} else if name == b"version" {
						out.showversion = true;
					} else if flag_argument {
						let value = !name.starts_with(b"no-");
						let name = self.current_name(if value { name } else { &name[3..] });
						self.configi(&name, SC_OVERRIDE | SC_SET, value as u32);
					} else {
						let name = self.current_name(name);
						self.configs(&name, SC_OVERRIDE | SC_SET, Some(args[av + 1]));
						av += 1;
					}
				}
			}
			av += 1;
		}
		out
	}

	/// survive_process_env(ctx, false): every SURVIVE_ variable (see env_tag) set as a command line value would be,
	/// survive_configs(tag, SC_OVERRIDE | SC_SET, value), in the environment's order
	pub fn apply_env<K: AsRef<[u8]>, V: AsRef<[u8]>>(&mut self, vars: impl IntoIterator<Item = (K, V)>) {
		for (name, value) in vars {
			if let Some(tag) = env_tag(name.as_ref()) {
				self.configs(&tag, SC_OVERRIDE | SC_SET, Some(c_str(value.as_ref())));
			}
		}
	}

	/// survive_process_env(ctx, true): the lines it logs at verbosity 100 instead, `"\t[ENV]'NAME=value'"` for each
	/// variable apply_env would set
	pub fn env_log_lines<K: AsRef<[u8]>, V: AsRef<[u8]>>(vars: impl IntoIterator<Item = (K, V)>) -> Vec<Vec<u8>> {
		let mut lines = Vec::new();
		for (name, value) in vars {
			if env_tag(name.as_ref()).is_some() {
				let mut line = b"\t[ENV]'".to_vec();
				line.extend_from_slice(c_str(name.as_ref()));
				line.push(b'=');
				line.extend_from_slice(c_str(value.as_ref()));
				line.push(b'\'');
				lines.push(line);
			}
		}
		lines
	}

	/// find_correct_config_file: the first of `fields` that is set and names a file (its value's text after the last
	/// '/' or '\\', not empty; a field set as a number names the field itself, "If a non-arg driver is used; just use
	/// the name of the driver") makes "configfile" that name plus ".json" (cut to 255 bytes) in the temporary group,
	/// and true is returned
	pub fn find_correct_config_file(&mut self, fields: &[&str]) -> bool {
		for name in fields {
			if !self.is_set(name) {
				continue;
			}
			let recordname = self.get_str(name.as_bytes(), None).unwrap_or_else(|| name.as_bytes().to_vec());
			let base = match recordname.iter().rposition(|&c| c == b'/' || c == b'\\') {
				Some(i) => &recordname[i + 1..],
				None => &recordname[..],
			};
			if !base.is_empty() {
				let mut configfile = base.to_vec();
				configfile.extend_from_slice(b".json");
				configfile.truncate(255);
				self.configs("configfile", SC_SET | SC_OVERRIDE, Some(&configfile));
				return true;
			}
		}
		false
	}
}
