//! What sally is doing while its tracking is not what SteamVR is shown: a line in the bar that says why and, where
//! there is something to do about it, what. Stated as it is, without advice beyond the one step that is the user's.
//! While all is as it should be the bar has no line. It has its place in the bar whether it is there or not, so
//! nothing under the bar moves when it comes and goes.

use super::topbar::clock;
use super::Panel;
use crate::icon::Dot;

/// How a state reads: something to know, something under way, something that needs the user
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tone {
	Note,
	Wait,
	Stop,
}

/// What the state is read from
#[derive(Clone, Debug, Default)]
pub struct Facts {
	/// SteamVR runs (the panel is connected to it, or the sally driver in it is heard), and for how long (s)
	pub steamvr: bool,
	pub steamvr_for: f64,
	/// the sally driver is heard (its status, every second)
	pub driver: bool,
	/// how many of SteamVR's pose interfaces the driver could take (None: a driver that does not say)
	pub hooks: Option<i32>,
	/// the switch: SteamVR is shown sally's tracking
	pub seen_sally: bool,
	/// sally is lined up with Valve's frame
	pub fit: bool,
	/// the devices sally has a pose for
	pub sally_devices: i32,
	/// the runtime answers, and for how long it has run (s)
	pub runtime: bool,
	pub runtime_for: f64,
	/// the base stations the runtime knows of
	pub stations: usize,
	/// LH1 base stations were seen
	pub lh1: bool,
	/// the room's first calibration: the seconds until it starts
	pub new_room: Option<f64>,
	/// ... and how far it has got once it runs (0 to 1)
	pub calibrating: Option<f64>,
	/// this session's calibration failed (it could not start, or broke off): why, as it said. Not an earlier
	/// session's: one that ended before a station was seen fails for that alone, and says nothing of this one
	pub calibration_failed: Option<String>,
}

/// The state in a line; None while SteamVR is shown sally's tracking and nothing is in its way
pub fn state(f: &Facts) -> Option<(Tone, String)> {
	let valve = "SteamVR is shown Valve's tracking";
	let said = |tone: Tone, text: String| Some((tone, text));
	if !f.steamvr {
		return said(Tone::Note, "SteamVR is not running.".into());
	}
	if !f.driver {
		// SteamVR loads its drivers in its first seconds
		if f.steamvr_for < 15.0 {
			return said(Tone::Wait, "SteamVR is starting.".into());
		}
		return said(
			Tone::Stop,
			"SteamVR is running without the sally driver. Switch sally on in SteamVR's Settings, Startup / Shutdown, \
			 Manage add-ons, then restart SteamVR."
				.into(),
		);
	}
	if f.hooks == Some(0) {
		return said(Tone::Stop, format!("This SteamVR version does not take sally's poses: {valve}."));
	}
	if !f.seen_sally {
		return said(Tone::Note, format!("The switch is on Valve: {valve}."));
	}
	if !f.runtime {
		return said(Tone::Wait, format!("The runtime is not answering yet: {valve}."));
	}
	if let Some(done) = f.calibrating {
		return said(Tone::Wait, format!("Calibrating this room, {:.0} %: {valve} until it is done.", done * 100.0));
	}
	if let (Some(s), Some(why)) = (f.new_room, &f.calibration_failed) {
		// the wait would otherwise start over without a word: said, with why (a picture of the panel then tells)
		let why = if why.is_empty() { String::new() } else { format!(" ({why})") };
		return said(Tone::Stop, format!("Room calibration failed{why}; next try in {}. {valve}.", clock(s)));
	}
	if let Some(s) = f.new_room {
		return said(
			Tone::Wait,
			format!(
				"New room: its calibration starts in {}, from devices that are on and moved around the room. {valve}.",
				clock(s)
			),
		);
	}
	if f.stations == 0 && f.runtime_for > 30.0 {
		return if f.lh1 {
			said(Tone::Stop, format!("1.0 base stations: sally reads 2.0 only. {valve}."))
		} else {
			said(Tone::Wait, "No base station seen yet (2.0 only).".into())
		};
	}
	if f.sally_devices == 0 {
		return said(Tone::Wait, "No tracker or controller has a pose yet.".into());
	}
	if !f.fit {
		return said(Tone::Wait, format!("Lining up with Valve's frame, from devices that move: about 15 s. {valve}."));
	}
	None
}

impl Panel {
	fn facts(&mut self, now: f64) -> Facts {
		let driver = self.vc.driver_heard(now);
		let steamvr = self.vc.running || driver;
		let since = if steamvr { *self.steamvr_since.get_or_insert(now) } else { now };
		if !steamvr {
			self.steamvr_since = None;
		}
		let c = &self.rt.calib;
		Facts {
			steamvr,
			steamvr_for: now - since,
			driver,
			hooks: self.vc.drv_hooks,
			seen_sally: self.vc.hook_seen == "sally",
			fit: self.vc.hook_fit,
			sally_devices: self.vc.drv_sally_devices,
			runtime: self.rt.connected(now),
			runtime_for: self.rt.run_time,
			stations: self.rt.stations.len(),
			lh1: c.lh1,
			new_room: c.new_room,
			calibrating: c.running.as_ref().filter(|r| r.kind == "new-room").map(|r| r.progress),
			calibration_failed: c.failed.clone(),
		}
	}

	/// The state as the icon's dot says it (icon.rs), at the host's `now`: asked every turn, the window drawn or not.
	/// A panel pointed at a runtime of its own says whether that runtime answers
	pub fn dot(&mut self, now: f64) -> Dot {
		if self.st.file_target.is_some() {
			return if self.rt.connected(now) { Dot::Tracking } else { Dot::Idle };
		}
		match state(&self.facts(now)) {
			None => Dot::Tracking,
			Some((Tone::Wait, _)) => Dot::Waiting,
			Some((Tone::Stop, _)) => Dot::Stopped,
			Some((Tone::Note, _)) => Dot::Idle,
		}
	}

	/// The state for the bar's line: what it says and the fault's level it is said at (something under way 2,
	/// something that needs the user 3, something to know none). None where there is nothing to say, and for a
	/// panel pointed at a runtime of its own (`--host`, `--port`): that one is not this PC's SteamVR's
	pub(super) fn state_line(&mut self) -> Option<(i32, String)> {
		if self.st.file_target.is_some() {
			return None;
		}
		let (tone, text) = state(&self.facts(self.now))?;
		let level = match tone {
			Tone::Note => 0,
			Tone::Wait => 2,
			Tone::Stop => 3,
		};
		Some((level, text))
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A session from SteamVR's start to sally's tracking shown, in a room sally has not seen: each step's sentence,
	/// and none at the end
	#[test]
	fn a_first_session_is_told_step_by_step() {
		let says =
			|f: &Facts| state(f).map(|(tone, text)| (tone, text.split([':', '.']).next().unwrap_or("").to_string()));
		let mut f = Facts::default();
		assert_eq!(says(&f), Some((Tone::Note, "SteamVR is not running".into())));
		f.steamvr = true;
		f.steamvr_for = 3.0;
		assert_eq!(says(&f), Some((Tone::Wait, "SteamVR is starting".into())));
		f.steamvr_for = 20.0;
		assert_eq!(says(&f), Some((Tone::Stop, "SteamVR is running without the sally driver".into())));
		f.driver = true;
		f.hooks = Some(3);
		f.seen_sally = true;
		assert_eq!(says(&f), Some((Tone::Wait, "The runtime is not answering yet".into())));
		f.runtime = true;
		f.runtime_for = 5.0;
		assert_eq!(says(&f), Some((Tone::Wait, "No tracker or controller has a pose yet".into())));
		f.runtime_for = 40.0;
		assert_eq!(says(&f), Some((Tone::Wait, "No base station seen yet (2".into())));
		f.lh1 = true;
		assert_eq!(says(&f), Some((Tone::Stop, "1".into())));
		f.lh1 = false;
		f.stations = 4;
		f.new_room = Some(75.0);
		assert!(state(&f).unwrap().1.starts_with("New room: its calibration starts in 1:15, "));
		f.calibration_failed = Some(String::new());
		assert_eq!(state(&f).unwrap().0, Tone::Stop);
		assert!(state(&f).unwrap().1.starts_with("Room calibration failed; next try in 1:15. "));
		// with why, as the calibration said it
		f.calibration_failed = Some("only 9 still moments were seen by two placed stations: too few".into());
		assert!(state(&f).unwrap().1.starts_with(
			"Room calibration failed (only 9 still moments were seen by two placed stations: too few); next try in \
			 1:15. "
		));
		f.calibration_failed = None;
		f.new_room = None;
		f.calibrating = Some(0.42);
		assert!(state(&f).unwrap().1.starts_with("Calibrating this room, 42 %: "));
		f.calibrating = None;
		f.sally_devices = 5;
		assert_eq!(says(&f), Some((Tone::Wait, "Lining up with Valve's frame, from devices that move".into())));
		f.fit = true;
		assert_eq!(state(&f), None);
		// the switch on Valve, and a SteamVR sally cannot place its poses in, are said over the rest
		f.seen_sally = false;
		assert_eq!(says(&f), Some((Tone::Note, "The switch is on Valve".into())));
		f.hooks = Some(0);
		assert_eq!(says(&f).unwrap().0, Tone::Stop);
	}
}
