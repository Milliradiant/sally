//! What sessions leave on disk, kept to the newest few (`driver.keep_sessions`). Every start of SteamVR writes a pose
//! log and every start of the runtime a recording and a console log, each named by its start (`{time}` in
//! sally_settings.txt), and nothing else ever removes them.
//!
//! A session is one start of SteamVR and everything written until the next. The runtime can start more than once in
//! it, each time with a recording and a log of their own: a room's first calibration ends in a restart, so does one
//! taken up from the panel, and so does a runtime that went down. The driver notes each session's start
//! (`sally_sessions.txt` beside its settings), and a file is of the session whose start is the last at or before the
//! file's own stamp. A file from before the first start noted (written by a sally that noted none) is a session by
//! itself, as every file was then.
//!
//! A kind of session file is one settings path with `{time}` in its file name: the pose log, the runtime's log, and
//! every runtime argument that has one (the recording); a recording and a pose log are each one kind as text and
//! packed. Its files are the ones in that folder named exactly as the path with a stamp in `{time}`'s place, so a file
//! named any other way is never touched: a recording made on purpose, a copy with something added to its name. When
//! SteamVR starts, the files of all but the newest `keep_sessions` sessions are deleted; the session then starting
//! adds its own to those.
//!
//! Only absolute paths are taken (the ones `sally setup` writes): a relative one is the runtime's or vrserver's to
//! resolve, not this module's to guess. A file that cannot be deleted (still open in a runtime that is running) stays.

use std::path::{Path, PathBuf};

use sally_math::printf::BArg;

use crate::helpers::{is_absolute, log, write_whole};
use crate::settings::Options;

const TIME: &[u8] = b"{time}";

/// The file of the sessions' starts, beside the driver's settings: a stamp a line
const STARTS: &str = "sally_sessions.txt";

/// The newest this many starts are kept in it
const STARTS_KEPT: usize = 512;

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

/// A tap recording's name, and a pose log's, ends in one of two: as text, or packed. The two are one kind of session
/// file, so that the files from before a change of form go as the others do
const FORMS: [[&str; 2]; 2] = [[".hidtap", ".hidtapz"], [".csv", ".csvz"]];

/// The session a file with this stamp is of: the last of the starts (oldest first) at or before it, and the stamp
/// itself where there is none
fn session_of<'a>(stamp: &'a str, starts: &'a [String]) -> &'a str {
	match starts.partition_point(|start| start.as_str() <= stamp) {
		0 => stamp,
		after => &starts[after - 1],
	}
}

/// The files of all but the newest `keep` sessions deleted, kind by kind (`keep` 0: nothing is); how many went.
/// `starts` are the sessions' starts as noted, oldest first
pub fn prune(paths: &[&[u8]], keep: usize, starts: &[String]) -> usize {
	if keep == 0 {
		return 0;
	}
	let mut gone = 0;
	for path in paths {
		let Some((dir, before, after)) = kind(path) else { continue };
		let own = [after.as_str()];
		let ends: &[&str] = FORMS.iter().find(|forms| forms.contains(&after.as_str())).map_or(&own, |forms| forms);
		let mut files: Vec<(String, PathBuf)> = ends.iter().flat_map(|end| files_of(&dir, &before, end)).collect();
		files.sort();
		// the sessions there are, oldest first: each file from before the first start noted (one recorded in both
		// forms is one), then the starts
		let noted = |stamp: &str| starts.first().is_some_and(|first| first.as_str() <= stamp);
		let mut sessions: Vec<&str> =
			files.iter().map(|(stamp, _)| stamp.as_str()).filter(|stamp| !noted(stamp)).collect();
		sessions.dedup();
		sessions.extend(starts.iter().map(String::as_str));
		let Some(&oldest_kept) = sessions.len().checked_sub(keep).and_then(|first| sessions.get(first)) else {
			continue;
		};
		for (_, file) in files.iter().filter(|(stamp, _)| session_of(stamp, starts) < oldest_kept) {
			if std::fs::remove_file(file).is_ok() {
				gone += 1;
			}
		}
	}
	gone
}

/// The settings' kinds of session file pruned to `driver.keep_sessions`; how many files went
fn prune_settings(o: &Options, starts: &[String]) -> usize {
	let mut paths: Vec<&[u8]> = vec![&o.pose_log, &o.runtime_log];
	paths.extend(o.runtime_args.iter().map(Vec::as_slice));
	prune(&paths, o.keep_sessions.max(0) as usize, starts)
}

/// The starts noted in `file`, oldest first (none where it does not read)
fn read_starts(file: &Path) -> Vec<String> {
	let text = std::fs::read_to_string(file).unwrap_or_default();
	let mut starts: Vec<String> = text.lines().map(str::trim).filter(|l| is_stamp(l)).map(str::to_string).collect();
	starts.sort();
	starts.dedup();
	starts
}

/// A session starts, at `now` (a stamp), with the driver's settings in `dir`: the files of all but the newest
/// `driver.keep_sessions` sessions before it are deleted (how many went), and its start is noted for the ones to come
pub fn start(o: &Options, dir: &[u8], now: &[u8]) -> usize {
	let file = std::str::from_utf8(dir).ok().map(|dir| Path::new(dir).join(STARTS));
	let mut starts = file.as_deref().map(read_starts).unwrap_or_default();
	let gone = prune_settings(o, &starts);
	let (Some(file), Some(now)) = (file, std::str::from_utf8(now).ok().filter(|now| is_stamp(now))) else {
		return gone;
	};
	starts.push(now.to_string());
	starts.sort();
	starts.dedup();
	let newest = &starts[starts.len().saturating_sub(STARTS_KEPT)..];
	let text: String = newest.iter().map(|start| format!("{start}\n")).collect();
	if !file.to_str().is_some_and(|file| write_whole(file.as_bytes(), text.as_bytes())) {
		log("sally: cannot write %s: this session's files count with the one before\n", &[BArg::S(STARTS.as_bytes())]);
	}
	gone
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
		assert_eq!(prune(&paths, 2, &[]), 2);
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
		assert_eq!(prune(&paths, 2, &[]), 0);
		assert_eq!(prune(&paths, 0, &[]), 0);
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
			assert_eq!(prune(&[&path], 3, &[]), 2, "{settings}");
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
			assert_eq!(prune(&[&path], 3, &[]), 0);
			assert_eq!(prune(&[&path], 9, &[]), 0);
			assert_eq!(prune(&[&path], 1, &[]), 3);
			assert_eq!(left(&dir), ["sallytap_20261101_100000_solver.txt", "sallytap_20261105_100000.hidtapz"]);
			let _ = std::fs::remove_dir_all(&dir);
		}
	}

	/// So does a pose log: the text ones from before the packed ones go as they age
	#[test]
	fn pose_logs_of_both_forms_are_one_kind() {
		let names = [
			"sally_poses_20261101_100000.csv",
			"sally_poses_20261102_100000.csv",
			"sally_poses_20261103_100000.csvz",
			"sally_poses_20261104_100000.csvz",
			"other_20261101_100000.csv",
		];
		for settings in ["sally_poses_{time}.csvz", "sally_poses_{time}.csv"] {
			let dir = folder("pose_forms", &names);
			assert_eq!(prune(&[&in_dir(&dir, settings)], 2, &[]), 2, "{settings}");
			assert_eq!(
				left(&dir),
				["other_20261101_100000.csv", "sally_poses_20261103_100000.csvz", "sally_poses_20261104_100000.csvz"],
				"{settings}"
			);
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
		assert_eq!(prune(&[&path], 1, &[]), 1);
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
			assert_eq!(prune(&[path.as_bytes()], 1, &[]), 0, "{path}");
		}
		assert_eq!(prune(&[b"/\xff/a_{time}.txt"], 1, &[]), 0);
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
		assert_eq!(prune_settings(&o, &[]), 0, "off unless it is set");
		o.keep_sessions = -3;
		assert_eq!(prune_settings(&o, &[]), 0);
		o.keep_sessions = 1;
		assert_eq!(prune_settings(&o, &[]), 2);
		assert_eq!(left(&dir), ["p_20261102_100000.csv", "r_20261102_100000.hidtap"]);
		let _ = std::fs::remove_dir_all(&dir);
	}

	/// What is kept is sessions, not files: a session in which the runtime started twice has two recordings, and they
	/// stay and go together, with its pose log. The files from before the driver noted the sessions' starts count one
	/// each, as they did
	#[test]
	fn a_session_with_two_recordings_is_one_session() {
		let dir = folder("sessions", &["r_20261101_100000.hidtap", "r_20261101_200000.hidtap"]);
		let settings = dir.to_str().unwrap().as_bytes().to_vec();
		let o = Options {
			keep_sessions: 2,
			pose_log: in_dir(&dir, "p_{time}.csv"),
			runtime_args: vec![in_dir(&dir, "r_{time}.hidtap")],
			..Options::default()
		};
		let put = |names: &[&str]| names.iter().for_each(|n| std::fs::write(dir.join(n), n).unwrap());
		// the first session noted: the two from before are the two kept
		assert_eq!(start(&o, &settings, b"20261102_090000"), 0);
		put(&["r_20261102_090005.hidtap", "r_20261102_093000.hidtap", "p_20261102_090001.csv"]);
		// the second: the session before it and the newer of the two from before
		assert_eq!(start(&o, &settings, b"20261103_090000"), 1);
		put(&["r_20261103_090004.hidtap"]);
		// the third: the two noted, whole
		assert_eq!(start(&o, &settings, b"20261104_090000"), 1);
		assert_eq!(
			left(&dir),
			[
				"p_20261102_090001.csv",
				"r_20261102_090005.hidtap",
				"r_20261102_093000.hidtap",
				"r_20261103_090004.hidtap",
				"sally_sessions.txt",
			]
		);
		// the fourth: the session of 2 Nov goes whole, its two recordings and its pose log
		assert_eq!(start(&o, &settings, b"20261105_090000"), 3);
		assert_eq!(left(&dir), ["r_20261103_090004.hidtap", "sally_sessions.txt"]);
		assert_eq!(
			std::fs::read_to_string(dir.join(STARTS)).unwrap(),
			"20261102_090000\n20261103_090000\n20261104_090000\n20261105_090000\n"
		);
		let _ = std::fs::remove_dir_all(&dir);
	}
}
