//! What the user chose in the panel that the driver keeps between sessions: sally_user.txt next to the driver,
//! `<name>=<value>` lines, written whenever a choice changes.
//! - `headset=sally|valve`: whose pose a lighthouse headset gets while SteamVR shows sally's tracking. sally's unless
//!   Valve's is chosen (`driver.hmd=0` keeps the headset on Valve's whatever is chosen here)
//! - `keep_sessions=<n>`: how many sessions' recordings and logs stay, over `driver.keep_sessions` (0 keeps them all)

use std::sync::{LazyLock, Mutex, MutexGuard};

use sally_math::c_atoi;
use sally_math::printf::{bsprintf, BArg};

use crate::cstdio::CFile;
use crate::helpers::{log, trim, K_SEP};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct User {
	pub headset_sally: bool,
	pub keep_sessions: Option<i32>,
}

impl Default for User {
	fn default() -> User {
		User { headset_sally: true, keep_sessions: None }
	}
}

impl User {
	/// One `<name>=<value>` line; anything else is left alone
	pub fn parse_line(&mut self, line: &[u8]) {
		let l = trim(line);
		let Some(eq) = l.iter().position(|&c| c == b'=') else { return };
		let (k, v) = (trim(&l[..eq]), trim(&l[eq + 1..]));
		match k {
			b"headset" => self.headset_sally = v != b"valve",
			b"keep_sessions" => self.keep_sessions = Some(c_atoi(v).max(0)),
			_ => {}
		}
	}

	pub fn text(&self) -> Vec<u8> {
		let mut out = bsprintf("headset=%s\r\n", &[BArg::S(if self.headset_sally { b"sally" } else { b"valve" })]);
		if let Some(n) = self.keep_sessions {
			out.extend(bsprintf("keep_sessions=%d\r\n", &[BArg::I(n as i64)]));
		}
		out
	}
}

struct Store {
	user: User,
	/// sally_user.txt, once the driver knows its folder
	path: Option<Vec<u8>>,
}

static G_USER: LazyLock<Mutex<Store>> =
	LazyLock::new(|| Mutex::new(Store { user: User { headset_sally: true, keep_sessions: None }, path: None }));

fn store() -> MutexGuard<'static, Store> {
	G_USER.lock().unwrap_or_else(|e| e.into_inner())
}

/// <dir>/sally_user.txt read (its choices over the defaults), and kept as the file the panel's choices go to
pub fn read(dir: &[u8]) -> User {
	let mut path = dir.to_vec();
	path.push(K_SEP);
	path.extend_from_slice(b"sally_user.txt");
	let mut u = User::default();
	if let Some(mut f) = CFile::open(&path, c"rb") {
		let mut line = [0u8; 256];
		while let Some(l) = f.gets(&mut line) {
			u.parse_line(l);
		}
		f.close();
		log("sally: read %s\n", &[BArg::S(&path)]);
	}
	let mut s = store();
	s.user = u;
	s.path = Some(path);
	u
}

pub fn get() -> User {
	store().user
}

/// A choice changed: kept, and the file written (once the driver knows its folder)
pub fn set(change: impl FnOnce(&mut User)) {
	let mut s = store();
	change(&mut s.user);
	if let Some(path) = &s.path {
		match CFile::open(path, c"wb") {
			Some(mut f) => {
				f.write(&s.user.text());
				f.close();
			}
			None => log("sally: cannot write %s\n", &[BArg::S(path)]),
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn the_choices_read_back_as_written() {
		let mut u = User::default();
		assert!(u.headset_sally && u.keep_sessions.is_none());
		assert_eq!(u.text(), b"headset=sally\r\n");
		for l in [&b"headset = valve\r\n"[..], b"keep_sessions=3\n", b"# a comment", b"other=1"] {
			u.parse_line(l);
		}
		assert_eq!(u, User { headset_sally: false, keep_sessions: Some(3) });
		let mut back = User::default();
		for l in u.text().split(|&c| c == b'\n') {
			back.parse_line(l);
		}
		assert_eq!(back, u);
		u.parse_line(b"keep_sessions=-2");
		assert_eq!(u.keep_sessions, Some(0));
	}
}
