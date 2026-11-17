//! The context lock (survive.c's survive_get_ctx_lock / survive_release_ctx_lock, a semaphore that starts taken): what
//! the runtime's threads share and take in turn. In the C it covers the whole SurviveContext and every driver's state;
//! here a `Mutex` around the runtime (with its drivers) and the two tap drivers' states, which their threads reach
//! through sally-hid's `live::Context` and `replay::Context`. survive-cli's main thread holds it from survive_init on
//! and lets it go only while survive_poll sleeps and while survive_close joins a driver's thread; each driver thread
//! takes it for what it does to the runtime (`sally run`).
//!
//! A std `Mutex` where the C has a semaphore. On Linux the two behave alike: the thread that releases either may take it
//! again before a thread it woke. A Windows semaphore is handed to a thread already waiting on it, where a `Mutex` (an
//! SRW lock) may go back to the thread that let it go if it asks again at once, as the playback thread at factor 0
//! does. That only moves where the main loop's polls get in.

use std::sync::{Mutex, MutexGuard};

use sally_hid::tap::{self, Tap};
use sally_hid::{live, replay};

use crate::runtime::Runtime;

/// What the context lock covers
pub struct Context {
	pub rt: Runtime,
	/// DriverRegHIDTap's state: the live tap's devices (`--hidtap 1`)
	pub live: Option<Tap>,
	/// The replayed recording's devices (`--hidtap-playback`)
	pub playback: Option<Tap>,
}

impl Context {
	/// The runtime survive_init made, no tap started yet
	pub fn new(rt: Runtime) -> Context {
		Context { rt, live: None, playback: None }
	}

	/// sally: the dongles' devices the runtime protocol's `pair` brought (SteamVR's, through its driver), given to the
	/// live tap (`Tap::assign`), stamped as the live reader stamps its reports. A replay's come from its recording
	pub fn apply_pairs(&mut self) {
		if self.rt.pending_pairs.is_empty() {
			return;
		}
		let pairs = std::mem::take(&mut self.rt.pending_pairs);
		let Some(tap) = self.live.as_mut() else { return };
		let t = (sally_os::absolute_time() * 1e6) as u64;
		for (dongle, device) in pairs {
			tap.assign(&mut self.rt, &dongle, &device, t);
		}
	}
}

/// survive_get_ctx_lock (a panic aborts sally, so no lock is ever left poisoned)
pub fn lock(ctx: &Mutex<Context>) -> MutexGuard<'_, Context> {
	ctx.lock().unwrap_or_else(|e| e.into_inner())
}

impl live::Context for Context {
	fn tap(&mut self) -> (&mut Tap, &mut dyn tap::Runtime) {
		(self.live.as_mut().expect("the live tap's state is in the context before its reader starts"), &mut self.rt)
	}
}

impl replay::Context for Context {
	fn tap(&mut self) -> (&mut Tap, &mut dyn tap::Runtime) {
		(self.playback.as_mut().expect("the playback's state is in the context before its thread starts"), &mut self.rt)
	}
}
