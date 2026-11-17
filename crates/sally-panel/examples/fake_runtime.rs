//! The fake runtime alone, on a port, for a panel that runs as a program of its own:
//!
//!   fake_runtime [port] [--full]
//!
//! On 127.0.0.1:9100 unless a port is given; `--full` with the whole body's set of twelve devices. It is how the
//! Windows exe is tried under wine (docs/sally_panel.md, "Tests"): the exe is the panel as installed, and this is what
//! it talks to.

use sally_panel::fake_runtime::FakeRuntime;

fn main() {
	let (mut port, mut full) = (9100u16, false);
	for a in std::env::args().skip(1) {
		match a.as_str() {
			"--full" => full = true,
			p => match p.parse() {
				Ok(n) => port = n,
				Err(_) => {
					eprintln!("unknown option {p}");
					std::process::exit(2);
				}
			},
		}
	}
	let runtime = match FakeRuntime::start_with(port, Vec::new(), full) {
		Ok(r) => r,
		Err(e) => {
			eprintln!("no fake runtime on port {port}: {e}");
			std::process::exit(1);
		}
	};
	println!("the fake runtime on 127.0.0.1:{}", runtime.port);
	loop {
		std::thread::sleep(std::time::Duration::from_secs(3600));
	}
}
