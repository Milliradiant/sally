//! The store against the C: every case of tests/data/config/cases.txt (a command line, an environment, config files and
//! a script of lookups, sets, bindings, saves and dumps) run the way a probe linked to the C build's libsurvive.so ran
//! it, and the output compared with that probe's, line for line (tests/data/config/expected.txt). The registry is the C
//! build's, dumped by the same probe (registry.txt). The run directory is written as @DIR@ in both. Linux only: the C's
//! outputs are glibc's, with LF line ends and a 64-bit long.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use super::cnum;
use super::store::{Config, ConfigEntry, ConfigGroup, EntryType, OptionDefault};
use super::{
	Binding, LighthouseRecord, VarType, FILENAME_MAX, NUM_GEN2_LIGHTHOUSES, PLAYBACK_CONFIG_PREFIX_FIELDS,
	RECORD_CONFIG_PREFIX_FIELDS, SC_GET, SC_SETCONFIG,
};

const CASES: &str = include_str!("../../tests/data/config/cases.txt");
const EXPECTED: &str = include_str!("../../tests/data/config/expected.txt");
const REGISTRY: &str = include_str!("../../tests/data/config/registry.txt");

fn fixture(name: &str) -> &'static [u8] {
	match name {
		"real.json" => include_bytes!("../../tests/data/config/real.json"),
		"typed.json" => include_bytes!("../../tests/data/config/typed.json"),
		"broken.json" => include_bytes!("../../tests/data/config/broken.json"),
		_ => panic!("no fixture {name}"),
	}
}

/// The probe's quoting: printable ASCII as is but for '"' and '\\', every other byte as \xHH
fn quote(out: &mut String, s: &[u8]) {
	out.push('"');
	for &c in s {
		if (32..127).contains(&c) && c != b'"' && c != b'\\' {
			out.push(c as char);
		} else {
			out.push_str(&format!("\\x{c:02x}"));
		}
	}
	out.push('"');
}

fn quote_opt(out: &mut String, s: Option<&[u8]>) {
	match s {
		Some(s) => quote(out, s),
		None => out.push_str("NULL"),
	}
}

/// A quoted string at the start of `s`: its bytes and the rest
fn unquote(s: &str) -> (Vec<u8>, &str) {
	let b = s.as_bytes();
	assert_eq!(b[0], b'"', "not quoted: {s}");
	let mut out = Vec::new();
	let mut i = 1;
	while b[i] != b'"' {
		if b[i] == b'\\' {
			out.push(u8::from_str_radix(&s[i + 2..i + 4], 16).unwrap());
			i += 4;
		} else {
			out.push(b[i]);
			i += 1;
		}
	}
	(out, &s[i + 1..])
}

/// The C build's registry, in its registration order
fn registered() -> Config {
	let mut config = Config::default();
	for line in REGISTRY.lines() {
		let rest = line.strip_prefix("option ").unwrap();
		let type_char = rest.as_bytes()[0];
		let (name, rest) = unquote(&rest[2..]);
		let rest = rest.trim_start();
		let (default, rest) = match type_char {
			b's' if rest.starts_with("NULL") => (OptionDefault::Str(None), &rest[4..]),
			b's' => {
				let (s, rest) = unquote(rest);
				(OptionDefault::Str(Some(s)), rest)
			}
			_ => {
				let (word, rest) = rest.split_once(' ').unwrap();
				let d = match type_char {
					b'f' => OptionDefault::Float(f64::from_bits(u64::from_str_radix(word, 16).unwrap())),
					b'i' => OptionDefault::Int(word.parse().unwrap()),
					_ => OptionDefault::Bool(word == "1"),
				};
				(d, rest)
			}
		};
		let (description, _) = unquote(rest.trim_start());
		let name = String::from_utf8(name).unwrap();
		config.register(&name, &String::from_utf8(description).unwrap(), default);
	}
	config
}

struct Case {
	name: String,
	args: Vec<String>,
	env: Vec<String>,
	files: Vec<(String, String)>,
	script: Vec<String>,
}

/// The config folder is sally's own, where the C's was libsurvive's: the one difference on purpose, so the cases' files
/// and the C's outputs are read with the one name for the other
const LIBSURVIVE_FOLDER: &str = "libsurvive/";
const SALLY_FOLDER: &str = "sally/";

fn cases() -> Vec<Case> {
	let mut cases = Vec::new();
	let text = CASES.replace(LIBSURVIVE_FOLDER, SALLY_FOLDER);
	let mut lines = text.lines();
	while let Some(line) = lines.next() {
		let Some(name) = line.strip_prefix("case ") else {
			continue;
		};
		let mut c = Case { name: name.into(), args: vec![], env: vec![], files: vec![], script: vec![] };
		for line in lines.by_ref() {
			if line == "script" {
				break;
			} else if let Some(a) = line.strip_prefix("arg ") {
				c.args.push(a.into());
			} else if let Some(e) = line.strip_prefix("env ") {
				c.env.push(e.into());
			} else if let Some(f) = line.strip_prefix("file ") {
				let (path, name) = f.split_once(' ').unwrap();
				c.files.push((path.into(), name.into()));
			}
		}
		for line in lines.by_ref() {
			if line == "end" {
				break;
			}
			c.script.push(line.into());
		}
		cases.push(c);
	}
	cases
}

/// The probe's arguments: "NULL", "EMPTY", "x:<hex>" or the text
fn arg(s: &str) -> Option<Vec<u8>> {
	match s {
		"NULL" => None,
		"EMPTY" => Some(Vec::new()),
		_ => Some(match s.strip_prefix("x:") {
			Some(hex) => (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap()).collect(),
			None => s.as_bytes().to_vec(),
		}),
	}
}

fn parse_double(s: &str) -> f64 {
	match s.strip_prefix("bits:") {
		Some(hex) => f64::from_bits(u64::from_str_radix(hex, 16).unwrap()),
		None => cnum::strtod(s.as_bytes()).0,
	}
}

/// A station as survive_init_internal leaves it before reading the file: mode -1, the rest zero
fn initial_station() -> LighthouseRecord {
	LighthouseRecord { mode: 0xff, ..Default::default() }
}

struct Run {
	config: Config,
	stations: [LighthouseRecord; NUM_GEN2_LIGHTHOUSES],
	env: Vec<(Vec<u8>, Vec<u8>)>,
	vars: HashMap<(char, usize), Binding>,
}

impl Run {
	fn getenv(&self) -> impl Fn(&str) -> Option<Vec<u8>> + '_ {
		|name| self.env.iter().rev().find(|(k, _)| k == name.as_bytes()).map(|(_, v)| v.clone())
	}

	/// survive_init_internal's config steps, in its order (config/mod.rs)
	fn init(argv: &[Vec<u8>], env: Vec<(Vec<u8>, Vec<u8>)>) -> Run {
		let mut run = Run {
			config: registered(),
			stations: [initial_station(); NUM_GEN2_LIGHTHOUSES],
			env,
			vars: HashMap::new(),
		};
		let outcome = run.config.apply_args(argv);
		assert!(!outcome.showhelp && !outcome.showversion && outcome.autocomplete.is_none());
		let env = run.env.clone();
		run.config.apply_env(env.iter().map(|(k, v)| (k, v)));
		run.config.configs("log", SC_GET, None);
		run.config.configi("record-stdout", SC_GET, 0);
		let user_set_configfile = run.config.is_set("configfile");
		if !user_set_configfile {
			run.config.find_correct_config_file(&PLAYBACK_CONFIG_PREFIX_FIELDS);
		}
		run.config.configi("v", SC_SETCONFIG, 0);
		let path = match run.config.configs("init-configfile", SC_GET, None) {
			Some(p) => p[..p.len().min(FILENAME_MAX - 1)].to_vec(),
			None => run.config.file_path_in(&run.getenv()),
		};
		run.config.read(Some(&path));
		if !user_set_configfile && run.config.find_correct_config_file(&RECORD_CONFIG_PREFIX_FIELDS) {
			let getenv = |name: &str| env.iter().rev().find(|(k, _)| k == name.as_bytes()).map(|(_, v)| v.clone());
			run.config.file_path_in(&getenv);
			run.config.save_in(&[false; NUM_GEN2_LIGHTHOUSES], &getenv);
		}
		// SurviveContext_attach_config: the context's three floats
		for tag in ["lighthouse-max-update", "lighthouse-max-nudge-distance", "lighthouse-update-velocity"] {
			let var = Binding::of(VarType::Float);
			run.config.attach(tag, &var);
			run.vars.insert(('c', run.vars.len()), var);
		}
		run.config.configi("configed-lighthouse-gen", SC_GET, 0);
		run.config.configi("lighthouse-gen", SC_GET, 0);
		run.config.configf("floor-offset", SC_GET, 0.0);
		run.config.configf("output-callback-stats", SC_GET, 0.0);
		for i in 0..NUM_GEN2_LIGHTHOUSES {
			run.config.read_lighthouse(i as u8, &mut run.stations[i]);
			run.config.configi(format!("lighthouse-{i}-disable"), SC_GET, 0);
		}
		run
	}

	fn var(&mut self, type_char: &str, slot: &str) -> Binding {
		let t = type_char.as_bytes()[0] as char;
		let var_type = match t {
			'i' => VarType::Int,
			'b' => VarType::Bool,
			'f' => VarType::Float,
			_ => VarType::Str,
		};
		self.vars.entry((t, slot.parse().unwrap())).or_insert_with(|| Binding::of(var_type)).clone()
	}

	fn command(&mut self, line: &str, out: &mut String) {
		out.push_str(line);
		let a: Vec<&str> = line.split(' ').filter(|s| !s.is_empty()).collect();
		let tag = a.get(1).and_then(|t| arg(t)).unwrap_or_default();
		let c = &mut self.config;
		match a[0] {
			"f" => {
				let v = c.configf(&tag, a[2].parse().unwrap(), parse_double(a[3]));
				out.push_str(&format!(" -> {:016x}", v.to_bits()));
			}
			"i" => {
				let def = cnum::uint32_of_ulong(cnum::strtoul(a[3].as_bytes(), 0).0);
				out.push_str(&format!(" -> {}", c.configi(&tag, a[2].parse().unwrap(), def)));
			}
			"b" => {
				let v = c.configb(&tag, a[2].parse().unwrap(), cnum::atoi(a[3].as_bytes()) != 0);
				out.push_str(&format!(" -> {}", v as i32));
			}
			"s" => {
				let v = c.configs(&tag, a[2].parse().unwrap(), arg(a[3]).as_deref());
				out.push_str(" -> ");
				quote_opt(out, v.as_deref());
			}
			"str" => {
				let v = c.as_str(&tag, arg(a[2]).as_deref(), a[3].parse().unwrap());
				out.push_str(" -> ");
				quote(out, &v);
			}
			"isset" => out.push_str(&format!(" -> {}", c.is_set(&tag) as i32)),
			"type" => out.push_str(&format!(" -> {}", c.value_type(&tag).map_or(0, |t| t as i32))),
			"live" => out.push_str(&format!(" -> {}", c.set_live(&tag, arg(a[2]).unwrap_or_default()))),
			"bound" => out.push_str(&format!(" -> {}", c.bound_count(&tag))),
			"attach" => {
				let var = self.var(a[2], a[3]);
				self.config.attach(&tag, &var);
			}
			"detach" => {
				let var = self.var(a[2], a[3]);
				let _ = self.config.detach(&tag, &var);
			}
			"var" => {
				let var = self.var(a[1], a[2]);
				out.push_str(" -> ");
				match var.var_type() {
					VarType::Int => out.push_str(&var.int().to_string()),
					VarType::Bool => out.push_str(&(var.bool() as i32).to_string()),
					VarType::Float => out.push_str(&format!("{:016x}", var.float().to_bits())),
					VarType::Str => quote(out, &var.text()),
				}
			}
			"save" => {
				let ootx = self.stations.map(|s| s.ootx_set);
				let env = self.env.clone();
				let getenv = |name: &str| env.iter().rev().find(|(k, _)| k == name.as_bytes()).map(|(_, v)| v.clone());
				self.config.save_in(&ootx, &getenv);
				let path = self.config.file_path_in(&getenv);
				out.push_str(" -> ");
				cat(out, &path);
			}
			"cat" => {
				out.push_str(" -> ");
				cat(out, &tag);
			}
			"dump" => {
				out.push('\n');
				dump_group(out, "global", &c.global);
				dump_group(out, "temporary", &c.temporary);
				for (i, g) in c.lighthouses.iter().enumerate() {
					dump_group(out, &format!("lh{i}"), g);
				}
				out.push_str("end");
			}
			"bsd" => {
				out.push('\n');
				for (i, s) in self.stations.iter().enumerate() {
					if *s != initial_station() {
						dump_station(out, i, s);
					}
				}
				out.push_str("end");
			}
			"ootx" => self.stations[a[1].parse::<usize>().unwrap()].ootx_set = cnum::atoi(a[2].as_bytes()) & 1 != 0,
			"setbsd" => {
				let s = &mut self.stations[a[1].parse::<usize>().unwrap()];
				match a[2] {
					"pose" => {
						let v: Vec<f64> = a[3].split(',').map(parse_double).collect();
						s.pose.pos.copy_from_slice(&v[..3]);
						s.pose.rot.copy_from_slice(&v[3..7]);
					}
					"variance0" => s.variance[0] = parse_double(a[3]),
					"accel0" => s.accel[0] = parse_double(a[3]),
					"phase1" => s.fcal[1].phase = parse_double(a[3]),
					"ogeemag0" => s.fcal[0].ogeemag = parse_double(a[3]),
					"id" => s.base_station_id = cnum::uint32_of_ulong(cnum::strtoul(a[3].as_bytes(), 0).0),
					"mode" => s.mode = cnum::atoi(a[3].as_bytes()) as u8,
					"unlock" => s.sys_unlock_count = cnum::atoi(a[3].as_bytes()) as u8,
					"PositionSet" => s.position_set = cnum::atoi(a[3].as_bytes()) & 1 != 0,
					_ => panic!("setbsd {}", a[2]),
				}
			}
			"setlh" => {
				let i = a[1].parse::<usize>().unwrap();
				self.config.set_lighthouse(i as u8, &mut self.stations[i]);
			}
			"readlh" => {
				let i = a[1].parse::<usize>().unwrap();
				let r = self.config.read_lighthouse(i as u8, &mut self.stations[i]);
				out.push_str(&format!(" -> {}", r as i32));
			}
			"iterate" => {
				out.push('\n');
				for (o, default) in c.iterate() {
					out.push_str("iterate ");
					quote(out, o.name());
					out.push_str(&format!(" {} ", o.type_char() as char));
					quote(out, &default);
					out.push('\n');
				}
				out.push_str("end");
			}
			"options" => {
				out.push('\n');
				for o in c.options() {
					let mut line = b"OPTION ".to_vec();
					line.extend_from_slice(o.name());
					line.push(b' ');
					line.push(o.type_char());
					line.push(b' ');
					line.extend_from_slice(&c.as_str(o.name(), Some(b""), 128));
					line.push(b'\n');
					quote(out, &line);
					out.push('\n');
				}
				out.push_str("end");
			}
			"path" => {
				let env = self.env.clone();
				let getenv = |name: &str| env.iter().rev().find(|(k, _)| k == name.as_bytes()).map(|(_, v)| v.clone());
				out.push_str(" -> ");
				quote(out, &self.config.file_path_in(&getenv));
			}
			"help" => {
				out.push_str(" -> ");
				quote(out, &c.known_configs_text(a[1] != "0"));
			}
			"helpparam" => {
				let text = c.help_for_parameter(&tag);
				out.push_str(&format!(" -> {} ", text.is_some() as i32));
				quote(out, text.as_deref().unwrap_or(b""));
			}
			_ => out.push_str(" -> unknown command"),
		}
		out.push('\n');
	}
}

fn cat(out: &mut String, path: &[u8]) {
	match std::fs::read(Path::new(std::str::from_utf8(path).unwrap())) {
		Ok(text) => quote(out, &text),
		Err(_) => out.push_str("NOFILE"),
	}
}

fn dump_group(out: &mut String, name: &str, g: &ConfigGroup) {
	for (i, e) in g.entries().iter().enumerate() {
		dump_entry(out, name, i, e);
	}
}

fn dump_entry(out: &mut String, name: &str, i: usize, e: &ConfigEntry) {
	out.push_str(&format!("entry {name} {i} "));
	quote(out, e.tag());
	out.push_str(&format!(" type={}", e.kind() as i32));
	match e.kind() {
		EntryType::Uint32 => out.push_str(&format!(" i={}", e.uint32())),
		EntryType::Float => out.push_str(&format!(" f={:016x}", e.numeric_bits())),
		_ => {}
	}
	out.push_str(&format!(" elements={} data=", e.elements()));
	match (e.data(), e.kind()) {
		(None, _) => out.push_str("NULL"),
		(Some(d), EntryType::FloatArray) => {
			let d = &d[..e.elements() as usize * 8];
			if d.is_empty() {
				out.push('-');
			}
			for b in d {
				out.push_str(&format!("{b:02x}"));
			}
		}
		(Some(_), EntryType::String) => quote(out, &e.data_c_str().unwrap_or_default()),
		(Some(_), _) => out.push_str("stale"),
	}
	out.push_str(&format!(" bound={}\n", e.bound()));
}

fn dump_station(out: &mut String, i: usize, s: &LighthouseRecord) {
	let hex = |v: f64| format!("{:016x},", v.to_bits());
	out.push_str(&format!(
		"bsd {i} PositionSet={} OOTXSet={} id={} mode={} unlock={} pose=",
		s.position_set as i32, s.ootx_set as i32, s.base_station_id, s.mode, s.sys_unlock_count
	));
	for v in s.pose.pos.iter().chain(&s.pose.rot) {
		out.push_str(&hex(*v));
	}
	out.push_str(" variance=");
	for v in &s.variance {
		out.push_str(&hex(*v));
	}
	out.push_str(" accel=");
	for v in &s.accel {
		out.push_str(&hex(*v));
	}
	out.push_str(" fcal=");
	for c in &s.fcal {
		for v in [c.phase, c.tilt, c.curve, c.gibpha, c.gibmag, c.ogeephase] {
			out.push_str(&hex(v));
		}
		out.push_str(&format!("{:016x};", c.ogeemag.to_bits()));
	}
	out.push('\n');
}

/// One case run as the C probe ran it: its output with the run directory written as @DIR@
fn run_case(c: &Case, base: &Path) -> String {
	let dir: PathBuf = base.join(&c.name);
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).unwrap();
	let d = dir.to_str().unwrap().to_string();
	for (path, name) in &c.files {
		let p = dir.join(path);
		std::fs::create_dir_all(p.parent().unwrap()).unwrap();
		std::fs::write(p, fixture(name)).unwrap();
	}
	if c.env.iter().any(|e| e.starts_with("XDG_CONFIG_HOME=")) {
		std::fs::create_dir_all(dir.join("xdg")).unwrap();
	}
	let mut env: Vec<(Vec<u8>, Vec<u8>)> = vec![
		(b"HOME".to_vec(), d.as_bytes().to_vec()),
		(b"PATH".to_vec(), b"/usr/bin:/bin".to_vec()),
		(b"PROBE_SCRIPT".to_vec(), format!("{d}/script.txt").into_bytes()),
		(b"PROBE_OUT".to_vec(), format!("{d}/probe.out").into_bytes()),
	];
	for e in &c.env {
		let (k, v) = e.split_once('=').unwrap();
		env.push((k.as_bytes().to_vec(), v.replace("@DIR@", &d).into_bytes()));
	}
	let mut argv = vec![b"cfgprobe".to_vec()];
	argv.extend(c.args.iter().map(|a| arg(&a.replace("@DIR@", &d)).unwrap()));
	let mut run = Run::init(&argv, env);
	let mut out = String::from("init ctx\n");
	for line in &c.script {
		run.command(&line.replace("@DIR@", &d), &mut out);
	}
	let _ = std::fs::remove_dir_all(&dir);
	out.replace(&d, "@DIR@")
}

#[test]
fn every_case_gives_what_the_c_gave() {
	let base = std::env::temp_dir().join(format!("sally-config-{}", std::process::id()));
	// a case's output: the lines after its "case <name>" line, up to the next one (no output line starts so)
	let mut expected: HashMap<&str, String> = HashMap::new();
	let mut current = None;
	for line in EXPECTED.lines() {
		if let Some(name) = line.strip_prefix("case ") {
			current = Some(name);
			expected.insert(name, String::new());
		} else if let Some(name) = current {
			let text = expected.get_mut(name).unwrap();
			text.push_str(line);
			text.push('\n');
		}
	}
	let mut failures = Vec::new();
	for c in cases() {
		let got = run_case(&c, &base);
		let want = &expected[c.name.as_str()].replace(LIBSURVIVE_FOLDER, SALLY_FOLDER);
		if &got != want {
			let (g, w): (Vec<&str>, Vec<&str>) = (got.lines().collect(), want.lines().collect());
			let n = (0..g.len().max(w.len())).find(|&i| g.get(i) != w.get(i)).unwrap_or(0);
			let (g, w) = (g.get(n).copied().unwrap_or("<none>"), w.get(n).copied().unwrap_or("<none>"));
			failures.push(format!("case {} line {}:\n  port: {}\n  C:    {}", c.name, n + 1, g, w));
		}
	}
	let _ = std::fs::remove_dir_all(&base);
	assert!(failures.is_empty(), "{} of the C's cases differ:\n{}", failures.len(), failures.join("\n"));
}
