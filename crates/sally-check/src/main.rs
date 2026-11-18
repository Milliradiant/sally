//! `sally-check <folder | zip>`: a session checked again (one that the panel's Check made, or an older sally-check's
//! own recording), its report.html written into its folder and what it found printed. Recording and sharing are the
//! panel's.

use std::path::{Path, PathBuf};
use std::process::exit;

use sally_check::{report, session, zip};

fn main() {
	let args: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
	match args.first() {
		Some(p) if p.as_os_str() != "--help" && p.as_os_str() != "-h" => exit(check(p)),
		_ => {
			println!("sally check {}", env!("CARGO_PKG_VERSION"));
			println!();
			println!("  sally-check <folder | zip>   check a session again (writes report.html into its folder)");
			println!();
			println!("A session is made by the panel's Check tab.");
		}
	}
}

fn check(p: &Path) -> i32 {
	let dir = if p.extension().is_some_and(|e| e.eq_ignore_ascii_case("zip")) {
		let dir = p.with_extension("");
		if session::tap(&dir).is_none() {
			if let Err(e) = zip::unzip(p, &dir) {
				eprintln!("Cannot open {}: {e}", p.display());
				return 1;
			}
		}
		dir
	} else if p.is_file() {
		p.parent().unwrap_or(Path::new(".")).to_path_buf()
	} else {
		p.to_path_buf()
	};
	if session::tap(&dir).is_none() {
		eprintln!("No {} in {}: not a check's session.", session::TAP, dir.display());
		return 1;
	}
	println!("Checking {} ...", dir.display());
	let title = dir.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
	match session::analyze(&dir).and_then(|a| {
		let s = session::Session::load(&dir);
		let r = report::Report::new(&a, &s);
		std::fs::write(dir.join(session::REPORT), r.html(&title))?;
		Ok(r.text())
	}) {
		Ok(text) => {
			println!("\n{text}");
			println!("Report: {}", dir.join(session::REPORT).display());
			0
		}
		Err(e) => {
			eprintln!("The check failed: {e}");
			1
		}
	}
}
