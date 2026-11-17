//! The port's choices where the C crashes or reads memory it never wrote (each said at its function), and the paths
//! that end survive_init_internal early: what the comparison with the C (against_c.rs) cannot run.

use super::store::{Config, ConfigGroup, EntryType, OptionDefault};
use super::{Binding, SaveOutcome, VarType, NUM_GEN2_LIGHTHOUSES, SC_GET, SC_SET};

fn read(text: &str) -> Config {
	let mut config = Config::default();
	config.read_text(text.as_bytes());
	config
}

fn tags(g: &ConfigGroup) -> Vec<String> {
	g.entries().iter().map(|e| String::from_utf8_lossy(e.tag()).into_owned()).collect()
}

#[test]
fn a_file_in_braces_reads_as_its_top_level() {
	// the C dereferences a NULL stack entry here
	let c = read("{\"a\":\"1\",\"lighthouse1\":{\"index\":\"1\"}}");
	assert_eq!(tags(&c.global), ["a"]);
	assert_eq!(tags(&c.lighthouses[1]), ["index"]);
}

// a 64-bit long, as glibc's sscanf reads "lighthouse4294967297" (Windows' 32-bit one saturates instead)
#[cfg(all(target_os = "linux", target_pointer_width = "64"))]
#[test]
fn a_lighthouse_past_the_groups_is_dropped() {
	// the C writes past lh_config
	let c = read(
		"\"lighthouse16\":{\"index\":\"16\"}\n\"lighthouse-1\":{\"b\":\"2\"}\n\"lighthouse4294967297\":{\"c\":\"3\"}\n",
	);
	assert!(c.global.entries().is_empty());
	// 4294967297 reads as the int 1, as glibc's sscanf truncates its long
	assert_eq!(tags(&c.lighthouses[1]), ["c"]);
}

#[test]
fn arrays_the_c_reads_from_nowhere() {
	// an empty array before any array value: nothing (array_data is still NULL)
	let c = read("\"e\":[]\n\"a\":[\"1.5\",\"2\"]\n");
	assert_eq!(tags(&c.global), ["a"]);
	// after one: parse_uint32 with no values sets an uninitialized uint32 in the C, 0 here
	let mut c = read("\"a\":[\"1.5\",\"2\"]\n");
	c.read_text(b"\"e\":[]\n");
	assert_eq!(c.global.find("e").map(|e| (e.kind(), e.uint32())), Some((EntryType::Uint32, 0)));
	// an array in an array: the inner one sets "", the outer one's end reads past array_data in the C, nothing here
	let c = read("\"a\":[[\"1\"],\"2\"]\n");
	assert_eq!(tags(&c.global), [""]);
	assert_eq!(c.global.find("").map(|e| e.uint32()), Some(2));
}

#[test]
fn the_text_ends_at_its_first_nul() {
	let c = read("\"a\":\"1\"\n\0\"b\":\"2\"\n");
	assert_eq!(tags(&c.global), ["a"]);
	// inside a string the NUL leaves it unterminated, and jsmn refuses the whole text
	let c = read("\"a\":\"1\"\n\"b\":\"2\0\"\n");
	assert!(c.global.entries().is_empty());
}

#[test]
fn the_runs_that_print_and_stop() {
	let mut c = Config::default();
	c.register("v", "Verbosity level", OptionDefault::Int(0));
	let out = c.apply_args(&["sally", "--v", "2", "-m", "--con", "x"]);
	assert_eq!(out.autocomplete, Some([Some(b"--con".to_vec()), Some(b"x".to_vec()), None]));
	assert_eq!(c.configi("v", SC_GET, 0), 2);
	let out = c.apply_args(&["sally", "-hx", "--version", "--help", "--no-help"]);
	assert!(out.showhelp && out.showversion && out.autocomplete.is_none());
	// "--no-help" is checked against "help" before its "no-" comes off: it sets "help" to 0
	assert_eq!(c.temporary.find("help").map(|e| e.uint32()), Some(0));
}

#[test]
fn an_options_old_name_sets_the_option() {
	// the tracker's options were once core2-*: what was written with those names still sets them
	let mut c = Config::default();
	c.register("track-gate", "", OptionDefault::Float(8.0));
	c.register("track-adapt", "", OptionDefault::Int(1));
	let _ = c.apply_args(&["sally", "--core2-gate", "6.5", "--no-core2-adapt", "--core2-impl", "1", "--core2", "1"]);
	assert_eq!(c.configf("track-gate", SC_GET, 0.0), 6.5);
	assert_eq!(c.configi("track-adapt", SC_GET, 1), 0);
	// a name that was never one of them stays what it is: an unregistered name with its text
	assert_eq!(tags(&c.temporary), ["track-gate", "track-adapt", "core2-impl", "core2"]);
	assert_eq!(&*c.current_name(b"core2-gate"), b"track-gate");
	assert_eq!(&*c.current_name(b"track-gate"), b"track-gate");
	assert_eq!(&*c.current_name(b"core2-"), b"core2-");
}

#[test]
fn an_environment_name_of_exactly_32_bytes() {
	// the C's tag[32] has no NUL then, and the C reads on into its stack
	let mut c = Config::default();
	let name = format!("SURVIVE_{}", "A".repeat(32));
	let too_long = format!("SURVIVE_{}", "B".repeat(33));
	c.apply_env([(name.as_bytes(), b"1".as_slice()), (too_long.as_bytes(), b"2")]);
	assert_eq!(tags(&c.temporary), ["a".repeat(32)]);
	let lines = Config::env_log_lines([(name.as_bytes(), b"1".as_slice()), (b"OTHER".as_slice(), b"2")]);
	assert_eq!(lines, [format!("\t[ENV]'{name}=1'").into_bytes()]);
}

#[test]
fn registering_again_keeps_the_place_and_takes_the_default() {
	let mut c = Config::default();
	c.register("a", "first", OptionDefault::Int(1));
	c.register("b", "", OptionDefault::text("x"));
	c.register("a", "second", OptionDefault::Int(5));
	let names: Vec<&[u8]> = c.options().iter().map(|o| o.name()).collect();
	assert_eq!(names, [b"a".as_slice(), b"b"]);
	assert_eq!(
		c.option("a").map(|o| (o.description().to_vec(), o.default_value())),
		Some((b"first".to_vec(), OptionDefault::Int(5)))
	);
}

#[test]
#[should_panic(expected = "Types disagree [i/f]")]
fn registering_another_type_is_fatal() {
	let mut c = Config::default();
	c.register("a", "", OptionDefault::Int(1));
	c.register("a", "", OptionDefault::Float(1.0));
}

#[test]
fn what_the_c_logs_for_detach_and_save() {
	let mut c = Config::default();
	let var = Binding::of(VarType::Float);
	assert_eq!(c.detach("t", &var).unwrap_err().to_string(), "Configuration item t not initialized.\n");
	c.attach("t", &var);
	assert!(c.detach("t", &var).is_ok());
	assert_eq!(c.detach("t", &var).unwrap_err().to_string(), "Found no config item to detach t");

	let dir = std::env::temp_dir().join(format!("sally-config-save-{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let blocker = dir.join("file");
	std::fs::write(&blocker, b"").unwrap();
	let path = format!("{}/c.json", blocker.display());
	c.configs("configfile", SC_SET, Some(path.as_bytes()));
	let none = [false; NUM_GEN2_LIGHTHOUSES];
	let failed = c.save(&none);
	assert_eq!(failed, SaveOutcome::OpenFailed { path: path.clone().into_bytes(), warn: true });
	assert!(failed.message().unwrap().starts_with(format!("Could not open '{path}' for writing;").as_bytes()));
	assert_eq!(c.save(&none), SaveOutcome::OpenFailed { path: path.into_bytes(), warn: false });
	c.configi("config-readonly", SC_SET, 1);
	let readonly = c.save(&none);
	assert_eq!(readonly.message().unwrap(), b"config file not written (config-readonly / lock-lighthouses)");
	assert_eq!(c.save(&none), SaveOutcome::ReadOnly { first: false });
	let _ = std::fs::remove_dir_all(&dir);
}

/// sally: config_save writes the file whole or not at all (`write_whole`): a new file, an old one replaced, no
/// temporary left; where the temporary cannot be made (a directory in its place) the file is written in place, as the
/// C writes it; through a link the link's file is written, and the link stays
#[test]
fn the_file_is_written_whole() {
	let dir = std::env::temp_dir().join(format!("sally-config-whole-{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).unwrap();
	let path = dir.join("c.json");
	let mut c = Config::default();
	c.configs("configfile", SC_SET, Some(path.to_str().unwrap().as_bytes()));
	let none = [false; NUM_GEN2_LIGHTHOUSES];
	for (tag, v) in [("a", 1), ("b", 2)] {
		c.global.set_uint32(tag, v);
		assert_eq!(c.save(&none), SaveOutcome::Written);
		assert_eq!(std::fs::read(&path).unwrap(), c.save_text(&none));
	}
	assert!(!dir.join("c.json.tmp").exists());
	std::fs::create_dir_all(dir.join("c.json.tmp").join("x")).unwrap();
	c.global.set_uint32("c", 3);
	assert_eq!(c.save(&none), SaveOutcome::Written);
	assert_eq!(std::fs::read(&path).unwrap(), c.save_text(&none));
	#[cfg(unix)]
	{
		let target = dir.join("target.json");
		std::fs::write(&target, b"old").unwrap();
		let link = dir.join("link.json");
		std::os::unix::fs::symlink(&target, &link).unwrap();
		assert!(super::write_whole(&link, b"new").is_ok());
		assert!(std::fs::symlink_metadata(&link).unwrap().file_type().is_symlink());
		assert_eq!(std::fs::read(&target).unwrap(), b"new");
	}
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn the_bare_name_without_a_home() {
	let c = Config::default();
	assert_eq!(c.file_path_in(&|_| None), b"/config.json");
}
