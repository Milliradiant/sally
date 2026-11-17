//! sally, the runtime and the commands around it:
//! - `sally run`: the runtime, live on the dongles or replaying a tap recording, on its own threads; what the SteamVR
//!   driver starts (docs/sally_driver.md, "The runtime process")
//! - `sally replay`: a tap recording replayed on one thread, deterministically, as the bench replays
//! - `sally calibrate`: a room's stations placed from a recording (docs/sally_design.md 4.2), which the live runtime
//!   also runs by itself (`autocal`)
//! - `sally ctl`: talks to a running runtime over its protocol (docs/sally_runtime_protocol.md)
//! - `sally setup`, `sally off`, `sally stop`, `sally copy-log`: SteamVR set up for sally and back, a leftover runtime
//!   stopped, SteamVR's log copied (`setup`)
//! - `sally convert`: tap recordings made of a USBPcap capture or of openhouse's recordings (`convert`)
//! - `sally probe`: the dongles checked, their config caches seeded (`probe`, Windows)

mod autocal;
mod calibrate;
mod convert;
mod ctl;
mod probe;
mod replay;
mod run;
mod setup;
mod startup;
mod text;

/// The command line from the subcommand on, as main's argv would have it: survive_init reads the options after the
/// program's name, which is the subcommand's here
fn subcommand_argv(name: &[u8], argv: &[Vec<u8>]) -> Vec<Vec<u8>> {
	let mut args = vec![name.to_vec()];
	args.extend_from_slice(&argv[2..]);
	args
}

fn main() {
	// the command line as bytes, as the C's argv has it
	let argv = sally_runtime::config::arguments();
	let code = match argv.get(1).map(Vec::as_slice) {
		Some(b"run") => run::main(&subcommand_argv(b"sally run", &argv)),
		Some(b"replay") => replay::main(&subcommand_argv(b"sally replay", &argv)),
		Some(b"calibrate") => calibrate::main(&subcommand_argv(b"sally calibrate", &argv)),
		Some(b"ctl") => ctl::main(&words(&argv)),
		Some(b"convert") => convert::main(&words(&argv)),
		Some(b"probe") => probe::main(&words(&argv)),
		Some(b"setup") => setup::setup(&words(&argv)),
		Some(b"off") => setup::off(&words(&argv)),
		Some(b"uninstall") => setup::uninstall(&words(&argv)),
		Some(b"stop") => setup::stop(&words(&argv)),
		Some(b"copy-log") => setup::copy_log(&words(&argv)),
		// started by a double click (no command, the console its own): the panel is what that is for
		None if setup::open_panel() => 0,
		_ => {
			eprintln!("usage: sally run [--hidtap 1 | --hidtap-playback <tap recording>] [--<option> <value>]...");
			eprintln!("       sally replay --hidtap-playback <tap recording> [--<option> <value>]...");
			eprintln!("       sally calibrate --hidtap-playback <tap recording> --configfile <config> [--<option> <value>]...");
			eprintln!("       sally ctl [--host HOST] [--port PORT] [--timeout TIMEOUT] [--only ONLY] words...");
			eprintln!("       sally convert <capture.pcap> [<capture.pcap.usbdevs>] > <tap recording>");
			eprintln!(
				"       sally convert [--project <folder>] [--logs <folder>] [--force] [--list] [<recording.lhraw | report.zip>...]"
			);
			eprintln!("       sally probe [-t <seconds>] [--config] [--vid <hex>]");
			eprintln!("       sally setup [--home <folder>] [--no-panel] [--stop-panel] SteamVR set up for sally, the panel started");
			eprintln!(
				"       sally off [--home <folder>]      SteamVR back to Valve's tracking alone (SteamVR closed)"
			);
			eprintln!("       sally uninstall [--home <folder>] sally taken out of SteamVR (SteamVR closed)");
			eprintln!("       sally stop [--home <folder>]     a runtime left running after SteamVR closed, stopped");
			eprintln!("       sally copy-log [--home <folder>] SteamVR's vrserver.txt copied into sally's logs");
			2
		}
	};
	std::process::exit(code);
}

/// The words after the subcommand, as text
fn words(argv: &[Vec<u8>]) -> Vec<String> {
	argv[2..].iter().map(|a| String::from_utf8_lossy(a).into_owned()).collect()
}
