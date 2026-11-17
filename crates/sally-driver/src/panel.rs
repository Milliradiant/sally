//! The panel started with SteamVR (`driver.panel_exe`). It is given `--steamvr <this process>`: it opens minimised,
//! steps aside when a panel already runs, and closes once SteamVR has. The driver does nothing else with it: closing
//! its window ends it, and it opens again from the Start menu.

use std::sync::Mutex;

use sally_math::printf::{bsprintf, BArg};

use crate::helpers::log;
use crate::runtime_process::RuntimeProcess;
use crate::settings::g_opt;

static G_PANEL: Mutex<RuntimeProcess> = Mutex::new(RuntimeProcess::NONE);

/// The folder of a file's path (empty: none in it)
fn folder(path: &[u8]) -> &[u8] {
	let cut = path.iter().rposition(|&c| c == b'\\' || c == b'/').unwrap_or(0);
	&path[..cut]
}

pub fn start() {
	let o = g_opt();
	if o.panel_exe.is_empty() {
		return;
	}
	let argv = vec![b"--steamvr".to_vec(), bsprintf("%d", &[BArg::I(std::process::id() as i64)])];
	let mut p = G_PANEL.lock().unwrap_or_else(|e| e.into_inner());
	if p.start_program(&o.panel_exe, folder(&o.panel_exe), b"", &argv) {
		log("sally: the panel started (%s)\n", &[BArg::S(&o.panel_exe)]);
	} else {
		log("sally: the panel did not start (%s)\n", &[BArg::S(&o.panel_exe)]);
	}
}

#[cfg(test)]
mod tests {
	#[test]
	fn the_panels_folder_is_its_paths() {
		assert_eq!(super::folder(br"C:\a b\sally\sally-panel.exe"), br"C:\a b\sally");
		assert_eq!(super::folder(b"/opt/sally/sally-panel"), b"/opt/sally");
		assert_eq!(super::folder(b"sally-panel.exe"), b"");
	}
}
