//! A recording shared from the command line, for trying `share` on recordings there are:
//!
//! ```text
//! share <recording> <out folder> [--configs <folder>]... [--room <file>]... [--log <file>] [--steamvr-log <file>]
//!       [--settings <file>]... [--poses <file>] [--from <s>] [--to <s>] [--user <name>] [--home <folder>]
//!       [--pc <name>] [--sally <folder>] [--here <sally's folder>]
//! ```
//! `--from` and `--to` are seconds from the recording's first report. The stand-ins' table is `share_names.txt` in
//! the out folder. `--cut <file>` writes the span as it was recorded there and nothing else (to replay beside the
//! share's). `--here` takes the user's name and folder and the PC's name from the environment, as the panel does.

use std::path::PathBuf;

use sally_check::share::{share, Private, Request};

fn main() {
	let args: Vec<String> = std::env::args().skip(1).collect();
	if args.len() < 2 {
		eprintln!("share <recording> <out folder> [options]: see the source");
		std::process::exit(2);
	}
	let out = PathBuf::from(&args[1]);
	let mut r = Request {
		recording: PathBuf::from(&args[0]),
		names: out.join("share_names.txt"),
		out,
		stamp: "20260101_000000".into(),
		when: "a recording".into(),
		version: env!("CARGO_PKG_VERSION").into(),
		..Request::default()
	};
	let (mut from, mut to) = (None, None);
	let mut cut_only: Option<PathBuf> = None;
	let mut private = Private::default();
	let mut i = 2;
	while i + 1 < args.len() {
		let v = args[i + 1].clone();
		match args[i].as_str() {
			"--configs" => r.config_dirs.push(v.into()),
			"--room" => r.room.push(v.into()),
			"--log" => r.sally_log = Some(v.into()),
			"--steamvr-log" => r.steamvr_log = Some(v.into()),
			"--settings" => r.settings.push(v.into()),
			"--poses" => r.poses = Some(v.into()),
			"--stamp" => r.stamp = v,
			// an older sally-check's session folder: its devices, poses, log and configs
			"--session" => {
				let dir = PathBuf::from(&v);
				r.devices = sally_check::session::Session::load(&dir).devices;
				r.poses = Some(dir.join(sally_check::session::POSES));
				r.steamvr_log = Some(dir.join(sally_check::session::STEAMVR_LOG));
				r.room.push(dir.join("driver_config.json"));
				r.config_dirs.push(dir);
			}
			"--cut" => cut_only = Some(v.into()),
			"--from" => from = v.parse::<f64>().ok(),
			"--to" => to = v.parse::<f64>().ok(),
			"--user" => private.user = v,
			"--home" => private.home = v,
			"--pc" => private.pc = v,
			"--sally" => private.sally = v,
			"--here" => private = Private::here(std::path::Path::new(&v)),
			other => {
				eprintln!("unknown option {other}");
				std::process::exit(2);
			}
		}
		i += 2;
	}
	r.private = private;
	if from.is_some() || to.is_some() {
		let start = std::fs::File::open(&r.recording)
			.ok()
			.and_then(|mut f| sally_hid::pack::span(&mut f).ok().flatten())
			.map_or(0, |(a, _)| a);
		let us = |s: f64| start + (s * 1e6) as u64;
		r.span = Some((from.map_or(0, us), to.map_or(u64::MAX, us)));
	}
	if let Some(to) = cut_only {
		match sally_check::share::recording::cut(&r.recording, r.span, &to) {
			Ok(c) => println!("{} reports, {} dongles", c.reports, c.dongles.len()),
			Err(e) => eprintln!("{e}"),
		}
		return;
	}
	let started = std::time::Instant::now();
	match share(&r) {
		Ok(s) => {
			println!("{}", s.text);
			for n in &s.notes {
				println!("note: {n}");
			}
			println!("{:?}", s.recording);
			println!(
				"{} ({:.1} MB, {:.0} s of recording, made in {:.1} s)",
				s.zip.display(),
				s.bytes as f64 / 1e6,
				s.seconds,
				started.elapsed().as_secs_f64()
			);
		}
		Err(e) => {
			eprintln!("no share: {e}");
			std::process::exit(1);
		}
	}
}
