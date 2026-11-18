//! What sessions leave on disk, kept to the newest few (`driver.keep_sessions`). Every start of SteamVR writes a pose
//! log and every start of the runtime a recording and a console log, each named by its start (`{time}` in
//! sally_settings.txt), and nothing else ever removes them.
//!
//! A kind of session file is one settings path with `{time}` in its file name: the pose log, the runtime's log, the
//! model process's log, and every runtime argument that has one (the recording, as text or packed: one kind). Its
//! files are the ones in that folder named exactly as the path with a stamp in `{time}`'s place, so a file named any
//! other way is never touched:
//! a recording made on purpose, a copy with something added to its name. When SteamVR starts, all but the newest
//! `keep_sessions` of each kind are deleted; the session then starting adds its own to those.
//!
//! Only absolute paths are taken (the ones `sally setup` writes): a relative one is the runtime's or vrserver's to
//! resolve, not this module's to guess. A file that cannot be deleted (still open in a runtime that is running) stays.

use std::path::{Path, PathBuf};

use crate::helpers::is_absolute;
use crate::settings::Options;

const TIME: &[u8] = b"{time}";

/// A stamp as `helpers::time_stamp` writes it ("%Y%m%d_%H%M%S"): its text orders as its time does
fn is_stamp(s: &str) -> bool {
	s.len() == 15 && s.bytes().enumerate().all(|(i, c)| if i == 8 { c == b'_' } else { c.is_ascii_digit() })
}

fn is_separator(c: u8) -> bool {
	c == b'/' || (cfg!(windows) && c == b'\\')
}

/// A settings path as a kind of session file: its folder, and its file name's text before and after `{time}`. None
/// for a path that is not absolute or not UTF-8, or that does not have `{time}` exactly once, in its file name
fn kind(path: &[u8]) -> Option<(PathBuf, String, String)> {
	if !is_absolute(path) {
		return None;
	}
	let name_at = path.iter().rposition(|&c| is_separator(c))? + 1;
	let (dir, name) = path.split_at(name_at);
	let has_time = |s: &[u8]| s.windows(TIME.len()).filter(|w| *w == TIME).count();
	if has_time(dir) != 0 || has_time(name) != 1 {
		return None;
	}
	let at = name.windows(TIME.len()).position(|w| w == TIME)?;
	let text = |s: &[u8]| String::from_utf8(s.to_vec()).ok();
	Some((PathBuf::from(text(dir)?), text(&name[..at])?, text(&name[at + TIME.len()..])?))
}

/// The files of one kind, by their stamps, oldest first
fn files_of(dir: &Path, before: &str, after: &str) -> Vec<(String, PathBuf)> {
	let Ok(entries) = std::fs::read_dir(dir) else { return Vec::new() };
	let mut files: Vec<(String, PathBuf)> = entries
		.flatten()
		.filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
		.filter_map(|e| {
			let name = e.file_name().into_string().ok()?;
			let stamp = name.strip_prefix(before)?.strip_suffix(after)?;
			is_stamp(stamp).then(|| (stamp.to_string(), e.path()))
		})
		.collect();
	files.sort();
	files
}

/// A tap recording's name ends in one of these: as text, or packed. The two are one kind of session file, so that
/// the recordings from before a change of form go as the others do
const RECORDING_ENDS: [&str; 2] = [".hidtap", ".hidtapz"];

/// All but the newest `keep` sessions' files of each kind deleted (`keep` 0: nothing is); how many went
pub fn prune(paths: &[&[u8]], keep: usize) -> usize {
	if keep == 0 {
		return 0;
	}
	let mut gone = 0;
	for path in paths {
		let Some((dir, before, after)) = kind(path) else { continue };
		let ends: &[&str] = if RECORDING_ENDS.contains(&after.as_str()) { &RECORDING_ENDS } else { &[after.as_str()] };
		let mut files: Vec<(String, PathBuf)> = ends.iter().flat_map(|end| files_of(&dir, &before, end)).collect();
		files.sort();
		// a session recorded in both forms is one session
		let mut stamps: Vec<&str> = files.iter().map(|(stamp, _)| stamp.as_str()).collect();
		stamps.dedup();
		let Some(&oldest_kept) = stamps.len().checked_sub(keep).and_then(|first| stamps.get(first)) else { continue };
		for (_, file) in files.iter().filter(|(stamp, _)| stamp.as_str() < oldest_kept) {
			if std::fs::remove_file(file).is_ok() {
				gone += 1;
			}
		}
	}
	gone
}

/// The settings' kinds of session file pruned to `driver.keep_sessions`; how many files went
pub fn prune_settings(o: &Options) -> usize {
	let mut paths: Vec<&[u8]> = vec![&o.pose_log, &o.runtime_log];
	paths.extend(o.runtime_args.iter().map(Vec::as_slice));
	prune(&paths, o.keep_sessions.max(0) as usize)
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A folder of its own for one test, with the files named
	fn folder(test: &str, names: &[&str]) -> PathBuf {
		let dir = std::env::temp_dir().join(format!("sally_sessions_{test}_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(&dir).unwrap();
		for n in names {
			std::fs::write(dir.join(n), n).unwrap();
		}
		dir
	}

	fn left(dir: &Path) -> Vec<String> {
		let mut names: Vec<String> =
			std::fs::read_dir(dir).unwrap().flatten().map(|e| e.file_name().into_string().unwrap()).collect();
		names.sort();
		names
	}

	fn in_dir(dir: &Path, name: &str) -> Vec<u8> {
		dir.join(name).to_str().unwrap().as_bytes().to_vec()
	}

	#[test]
	fn the_newest_of_each_kind_stay() {
		let dir = folder(
			"newest",
			&[
				"sallytap_20261101_100000.hidtap",
				"sallytap_20261102_100000.hidtap",
				"sallytap_20261103_100000.hidtap",
				"sallytap_20261101_100000_solver.txt",
				"sallytap_20261102_100000_solver.txt",
				"sallytap_20261103_100000_solver.txt",
				"sally_poses_20261101_100003.csv",
				"sally_poses_20261103_100002.csv",
			],
		);
		let paths = [
			in_dir(&dir, "sallytap_{time}.hidtap"),
			in_dir(&dir, "sallytap_{time}_solver.txt"),
			in_dir(&dir, "sally_poses_{time}.csv"),
		];
		let paths: Vec<&[u8]> = paths.iter().map(Vec::as_slice).collect();
		assert_eq!(prune(&paths, 2), 2);
		assert_eq!(
			left(&dir),
			[
				"sally_poses_20261101_100003.csv",
				"sally_poses_20261103_100002.csv",
				"sallytap_20261102_100000.hidtap",
				"sallytap_20261102_100000_solver.txt",
				"sallytap_20261103_100000.hidtap",
				"sallytap_20261103_100000_solver.txt",
			]
		);
		// nothing more to do, and 0 keeps everything
		assert_eq!(prune(&paths, 2), 0);
		assert_eq!(prune(&paths, 0), 0);
		assert_eq!(left(&dir).len(), 6);
		let _ = std::fs::remove_dir_all(&dir);
	}

	/// A recording counts whichever form it is in: the text ones from before the packed ones go as they age
	#[test]
	fn recordings_of_both_forms_are_one_kind() {
		let names = [
			"sallytap_20261101_100000.hidtap",
			"sallytap_20261102_100000.hidtap",
			"sallytap_20261103_100000.hidtap",
			"sallytap_20261103_100000.hidtapz",
			"sallytap_20261104_100000.hidtapz",
			"sallytap_20261105_100000.hidtapz",
			"sallytap_20261101_100000_solver.txt",
		];
		for settings in ["sallytap_{time}.hidtapz", "sallytap_{time}.hidtap"] {
			let dir = folder("forms", &names);
			let path = in_dir(&dir, settings);
			assert_eq!(prune(&[&path], 3), 2, "{settings}");
			assert_eq!(
				left(&dir),
				[
					"sallytap_20261101_100000_solver.txt",
					"sallytap_20261103_100000.hidtap",
					"sallytap_20261103_100000.hidtapz",
					"sallytap_20261104_100000.hidtapz",
					"sallytap_20261105_100000.hidtapz",
				],
				"{settings}"
			);
			assert_eq!(prune(&[&path], 3), 0);
			assert_eq!(prune(&[&path], 9), 0);
			assert_eq!(prune(&[&path], 1), 3);
			assert_eq!(left(&dir), ["sallytap_20261101_100000_solver.txt", "sallytap_20261105_100000.hidtapz"]);
			let _ = std::fs::remove_dir_all(&dir);
		}
	}

	/// Only a file named exactly as the settings name it is a session's: a recording with something added to its
	/// name, another program's file, a folder, a stamp that is not one all stay
	#[test]
	fn a_file_named_any_other_way_stays() {
		let names = [
			"sallytap_20261101_100000.hidtap",
			"sallytap_20261101_100000_pairs.hidtap",
			"sallytap_20261101_100000.hidtap.bak",
			"my_sallytap_20261101_100000.hidtap",
			"sallytap_2026110a_100000.hidtap",
			"sallytap_20261101-100000.hidtap",
			"sallytap_.hidtap",
			"notes.txt",
		];
		let dir = folder("other", &names);
		std::fs::create_dir(dir.join("sallytap_20261001_100000.hidtap")).unwrap();
		std::fs::write(dir.join("sallytap_20261102_100000.hidtap"), "new").unwrap();
		let path = in_dir(&dir, "sallytap_{time}.hidtap");
		assert_eq!(prune(&[&path], 1), 1);
		let mut want: Vec<String> = names[1..].iter().map(|s| s.to_string()).collect();
		want.push("sallytap_20261001_100000.hidtap".into());
		want.push("sallytap_20261102_100000.hidtap".into());
		want.sort();
		assert_eq!(left(&dir), want);
		let _ = std::fs::remove_dir_all(&dir);
	}

	#[test]
	fn a_path_that_is_not_a_kind_prunes_nothing() {
		let dir = folder("notkind", &["a_20261101_100000.txt", "a_20261102_100000.txt"]);
		let d = dir.to_str().unwrap();
		for path in [
			"a_{time}.txt".to_string(),             // relative
			format!("{d}/a_20261101_100000.txt"),   // no {time}
			format!("{d}/a_{{time}}_{{time}}.txt"), // twice
			format!("{d}_{{time}}/a_{{time}}.txt"), // in a folder's name
			format!("{d}/missing/a_{{time}}.txt"),  // no such folder
			String::new(),
		] {
			assert_eq!(prune(&[path.as_bytes()], 1), 0, "{path}");
		}
		assert_eq!(prune(&[b"/\xff/a_{time}.txt"], 1), 0);
		assert_eq!(left(&dir).len(), 2);
		let _ = std::fs::remove_dir_all(&dir);
	}

	#[test]
	fn the_settings_kinds_are_the_logs_and_the_runtime_arguments() {
		let dir = folder(
			"settings",
			&["p_20261101_100000.csv", "p_20261102_100000.csv", "r_20261101_100000.hidtap", "r_20261102_100000.hidtap"],
		);
		let mut o = Options {
			pose_log: in_dir(&dir, "p_{time}.csv"),
			runtime_args: vec![b"--hidtap-record".to_vec(), in_dir(&dir, "r_{time}.hidtap"), b"run".to_vec()],
			..Options::default()
		};
		assert_eq!(prune_settings(&o), 0, "off unless it is set");
		o.keep_sessions = -3;
		assert_eq!(prune_settings(&o), 0);
		o.keep_sessions = 1;
		assert_eq!(prune_settings(&o), 2);
		assert_eq!(left(&dir), ["p_20261102_100000.csv", "r_20261102_100000.hidtap"]);
		let _ = std::fs::remove_dir_all(&dir);
	}
}
