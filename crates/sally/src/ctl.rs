//! `sally ctl`: the command line client for the runtime protocol (docs/sally_runtime_protocol.md).
//!
//!   sally ctl hello                       one command, print the reply lines
//!   sally ctl watch diag --only R,F       subscribe and print lines (Ctrl-C to stop)
//!   sally ctl stats 10                    subscribe for N seconds, then count line types per device
//!
//! Its argument parsing follows Python's argparse, which the first client used (each one's exact name or a unique
//! prefix of it, its value after it or after '=', a word that looks like a negative number taken as a word), with
//! argparse's messages; it reads text as Python does (a datagram's bytes decoded with errors replaced, split at
//! Python's line boundaries).

use std::collections::BTreeMap;
use std::io::Write;
use std::net::{Ipv4Addr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use sally_math::printf::{sprintf, Arg};

const USAGE: &str = concat!(
	"usage: sally ctl [-h] [--host HOST] [--port PORT] [--timeout TIMEOUT]\n",
	"                 [--only ONLY]\n",
	"                 words [words ...]\n",
);

const HELP: &str = concat!(
	"\n",
	"sally ctl: command line client for the sally runtime (docs/sally_runtime_protocol.md).\n",
	"\n",
	"  sally ctl hello                       one command, print the reply lines\n",
	"  sally ctl get track-gap-coast\n",
	"  sally ctl set track-gap-coast 0.4\n",
	"  sally ctl params track-gap            every parameter starting with the prefix\n",
	"  sally ctl map lock\n",
	"  sally ctl watch diag --only R,F       subscribe and print lines (Ctrl-C to stop)\n",
	"  sally ctl stats 10                    subscribe for N seconds, then count line types per device\n",
	"\n",
	"Options: --host (default 127.0.0.1), --port (default 9100), --timeout for command replies (default 1 s).\n",
	"\n",
	"positional arguments:\n",
	"  words\n",
	"\n",
	"options:\n",
	"  -h, --help         show this help message and exit\n",
	"  --host HOST\n",
	"  --port PORT\n",
	"  --timeout TIMEOUT\n",
	"  --only ONLY        watch: comma separated line types to print, e.g. R,F\n",
);

/// The command line, as argparse leaves it
struct Args {
	host: String,
	port: u16,
	timeout: f64,
	only: String,
	words: Vec<String>,
}

/// argparse's error: the usage and the message on stderr, exit 2
fn usage_error(msg: &str) -> i32 {
	eprint!("{USAGE}");
	eprintln!("sally ctl: error: {msg}");
	2
}

/// A word that argparse takes for a negative number (a positional, since no option looks like one)
fn negative_number(s: &str) -> bool {
	let Some(rest) = s.strip_prefix('-') else { return false };
	let digits = |t: &str| !t.is_empty() && t.bytes().all(|c| c.is_ascii_digit());
	digits(rest)
		|| rest.strip_prefix('.').is_some_and(digits)
		|| rest.split_once('.').is_some_and(|(a, b)| digits(a) && digits(b))
}

/// The command line after `sally ctl` (Err: the exit code, the help or an error printed)
fn parse(args: &[String]) -> Result<Args, i32> {
	const OPTIONS: [&str; 5] = ["--help", "--host", "--port", "--timeout", "--only"];
	let mut a =
		Args { host: "127.0.0.1".to_string(), port: 9100, timeout: 1.0, only: String::new(), words: Vec::new() };
	let mut i = 0;
	while i < args.len() {
		let arg = &args[i];
		i += 1;
		if arg == "-h" {
			print!("{USAGE}{HELP}");
			return Err(0);
		}
		if !arg.starts_with('-') || arg == "-" || negative_number(arg) {
			a.words.push(arg.clone());
			continue;
		}
		if arg == "--" {
			a.words.extend(args[i..].iter().cloned());
			break;
		}
		let (name, inline) = match arg.split_once('=') {
			Some((n, v)) if n.starts_with("--") => (n, Some(v.to_string())),
			_ => (arg.as_str(), None),
		};
		let matches: Vec<&str> = if OPTIONS.contains(&name) {
			vec![name]
		} else if name.starts_with("--") {
			OPTIONS.iter().copied().filter(|o| o.starts_with(name)).collect()
		} else {
			Vec::new()
		};
		let option = match matches.as_slice() {
			[o] => *o,
			[] => return Err(usage_error(&format!("unrecognized arguments: {arg}"))),
			many => {
				let list = many.join(", ");
				return Err(usage_error(&format!("ambiguous option: {name} could match {list}")));
			}
		};
		if option == "--help" {
			print!("{USAGE}{HELP}");
			return Err(0);
		}
		let value = match inline {
			Some(v) => v,
			None if i < args.len() && (!args[i].starts_with('-') || negative_number(&args[i])) => {
				i += 1;
				args[i - 1].clone()
			}
			None => return Err(usage_error(&format!("argument {option}: expected one argument"))),
		};
		match option {
			"--host" => a.host = value,
			"--port" => match value.trim().parse::<i64>() {
				Ok(p) if (0..=65535).contains(&p) => a.port = p as u16,
				// Python's connect refuses it (OverflowError)
				Ok(_) => {
					eprintln!("sally ctl: port must be 0-65535.");
					return Err(1);
				}
				Err(_) => return Err(usage_error(&format!("argument --port: invalid int value: '{value}'"))),
			},
			"--timeout" => match value.trim().parse::<f64>() {
				Ok(t) => a.timeout = t,
				Err(_) => return Err(usage_error(&format!("argument --timeout: invalid float value: '{value}'"))),
			},
			_ => a.only = value,
		}
	}
	if a.words.is_empty() {
		return Err(usage_error("the following arguments are required: words"));
	}
	Ok(a)
}

/// str.splitlines: the text split at Python's line boundaries (\n, \r, \r\n, \v, \f, \x1c, \x1d, \x1e, \x85,
/// \u2028, \u2029), without a last empty line
fn splitlines(text: &str) -> Vec<&str> {
	let mut out = Vec::new();
	let mut start = 0;
	let mut chars = text.char_indices().peekable();
	while let Some((i, c)) = chars.next() {
		if matches!(c, '\n' | '\r' | '\x0b' | '\x0c' | '\x1c' | '\x1d' | '\x1e' | '\u{85}' | '\u{2028}' | '\u{2029}') {
			out.push(&text[start..i]);
			let mut end = i + c.len_utf8();
			if c == '\r' {
				if let Some(&(j, '\n')) = chars.peek() {
					chars.next();
					end = j + 1;
				}
			}
			start = end;
		}
	}
	if start < text.len() {
		out.push(&text[start..]);
	}
	out
}

/// open_socket: a UDP socket on loopback for a loopback host (else on every interface), connected to the runtime
fn open_socket(host: &str, port: u16) -> std::io::Result<UdpSocket> {
	let local = if host == "127.0.0.1" || host == "localhost" { Ipv4Addr::LOCALHOST } else { Ipv4Addr::UNSPECIFIED };
	let s = UdpSocket::bind((local, 0))?;
	// the socket is IPv4 (AF_INET): the host's IPv4 addresses
	let addrs: Vec<SocketAddr> = (host, port).to_socket_addrs()?.filter(SocketAddr::is_ipv4).collect();
	s.connect(&addrs[..])?;
	Ok(s)
}

/// One datagram as text (bytes that are not UTF-8 replaced), or None when nothing came in time
fn recv_text(s: &UdpSocket, buf: &mut [u8]) -> Option<String> {
	match s.recv(buf) {
		Ok(n) => Some(String::from_utf8_lossy(&buf[..n]).into_owned()),
		// a timeout, or the runtime's port closed (ConnectionRefusedError): nothing this time
		Err(_) => None,
	}
}

fn now() -> f64 {
	SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

/// command: one command with a request id; its reply lines (the last one starts with OK or ERR)
fn command(s: &UdpSocket, line: &str, timeout: f64) -> Vec<String> {
	let rid = format!("#{}", (now() * 1000.0) as i64 % 1000000);
	let _ = s.send(format!("{rid} {line}\n").as_bytes());
	let mut out = Vec::new();
	let deadline = now() + timeout;
	let mut buf = vec![0u8; 65535];
	while now() < deadline {
		let _ = s.set_read_timeout(Some(Duration::from_secs_f64(f64::max(0.01, deadline - now()))));
		let Some(data) = recv_text(s, &mut buf) else { continue };
		for ln in splitlines(&data) {
			// stream lines or another request's reply
			let Some(ln) = ln.strip_prefix(&rid).and_then(|l| l.strip_prefix(' ')) else { continue };
			out.push(ln.to_string());
			if ln.starts_with("OK") || ln.starts_with("ERR") {
				return out;
			}
		}
	}
	out.push("ERR timeout (no runtime on that port?)".to_string());
	out
}

/// stream: subscribe, keep the subscription alive, every stream line to on_line until the seconds pass (None: until
/// interrupted), then unsubscribe
fn stream(s: &UdpSocket, streams: &[String], seconds: Option<f64>, mut on_line: impl FnMut(&str)) {
	let sub = format!("sub {}\n", streams.join(" "));
	let _ = s.send(sub.as_bytes());
	let mut last_sub = now();
	let end = seconds.map(|t| now() + t);
	let _ = s.set_read_timeout(Some(Duration::from_secs_f64(0.2)));
	let mut buf = vec![0u8; 65535];
	while end.is_none_or(|e| now() < e) && !interrupt::caught() {
		if now() - last_sub > 1.0 {
			let _ = s.send(sub.as_bytes());
			last_sub = now();
		}
		let Some(data) = recv_text(s, &mut buf) else { continue };
		for ln in splitlines(&data) {
			if !ln.is_empty() && !ln.starts_with("OK") && !ln.starts_with("ERR") {
				on_line(ln);
			}
		}
	}
	let _ = s.send(b"unsub\n");
}

/// `sally ctl`'s main: `args` are the words after `sally ctl`; the exit code
pub fn main(args: &[String]) -> i32 {
	let a = match parse(args) {
		Ok(a) => a,
		Err(code) => return code,
	};
	let s = match open_socket(&a.host, a.port) {
		Ok(s) => s,
		Err(e) => {
			eprintln!("sally ctl: {}:{}: {e}", a.host, a.port);
			return 1;
		}
	};
	let mut stdout = std::io::stdout();

	if a.words[0] == "watch" {
		let streams = if a.words.len() > 1 { a.words[1..].to_vec() } else { vec!["all".to_string()] };
		let only: Vec<&str> = a.only.split(',').filter(|x| !x.is_empty()).collect();
		// Ctrl-C ends the watch (KeyboardInterrupt), with its unsub
		interrupt::catch();
		stream(&s, &streams, None, |ln| {
			if only.is_empty() || only.contains(&ln.split(' ').next().unwrap_or("")) {
				let _ = writeln!(stdout, "{ln}");
				let _ = stdout.flush();
			}
		});
		return 0;
	}

	if a.words[0] == "stats" {
		let seconds = match a.words.get(1).map(|w| w.trim().parse::<f64>()) {
			None => 5.0,
			Some(Ok(t)) => t,
			Some(Err(_)) => {
				eprintln!("sally ctl: could not convert string to float: '{}'", a.words[1]);
				return 1;
			}
		};
		let mut counts: BTreeMap<(String, String), i64> = BTreeMap::new();
		let mut last: BTreeMap<String, String> = BTreeMap::new();
		stream(&s, &["all".to_string()], Some(seconds), |ln| {
			let f: Vec<&str> = ln.split_whitespace().collect();
			// a line of blanks only (where the Python's f[0] raises): not counted
			if f.is_empty() {
				return;
			}
			// f[0] in "PVSRHT": a substring of it (a one-letter line type, for the lines per device)
			let dev = if "PVSRHT".contains(f[0]) && f.len() > 1 { f[1] } else { "" };
			*counts.entry((f[0].to_string(), dev.to_string())).or_insert(0) += 1;
			last.insert(f[0].to_string(), ln.to_string());
		});
		for ((kind, dev), n) in &counts {
			let rate = *n as f64 / seconds;
			let args = [Arg::S(kind), Arg::S(dev), Arg::I(*n), Arg::F(rate)];
			let _ = writeln!(stdout, "{}", sprintf("%-2s %-8s %7d  %.1f/s", &args));
		}
		for (kind, ln) in &last {
			let _ = writeln!(stdout, "last {kind}: {ln}");
		}
		return if counts.is_empty() { 1 } else { 0 };
	}

	let lines = command(&s, &a.words.join(" "), a.timeout);
	for ln in &lines {
		let _ = writeln!(stdout, "{ln}");
	}
	if lines.last().is_some_and(|l| l.starts_with("OK")) {
		0
	} else {
		1
	}
}

/// Ctrl-C caught: a flag the watch loop reads (SIGINT's handler, or the console's on Windows)
mod interrupt {
	use std::sync::atomic::{AtomicBool, Ordering};

	static CAUGHT: AtomicBool = AtomicBool::new(false);

	#[cfg(unix)]
	extern "C" {
		fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
	}

	#[cfg(unix)]
	extern "C" fn on_signal(_: i32) {
		CAUGHT.store(true, Ordering::SeqCst);
	}

	#[cfg(windows)]
	#[link(name = "kernel32")]
	extern "system" {
		fn SetConsoleCtrlHandler(handler: extern "system" fn(u32) -> i32, add: i32) -> i32;
	}

	#[cfg(windows)]
	extern "system" fn on_ctrl(_: u32) -> i32 {
		CAUGHT.store(true, Ordering::SeqCst);
		1
	}

	/// From now on Ctrl-C sets the flag instead of ending the process
	pub fn catch() {
		// SIGINT is 2 on every unix sally runs on
		// SAFETY: the handler only stores to an atomic
		#[cfg(unix)]
		unsafe {
			signal(2, on_signal);
		}
		// SAFETY: as above
		#[cfg(windows)]
		unsafe {
			SetConsoleCtrlHandler(on_ctrl, 1);
		}
	}

	pub fn caught() -> bool {
		CAUGHT.load(Ordering::SeqCst)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn args(words: &[&str]) -> Vec<String> {
		words.iter().map(|w| w.to_string()).collect()
	}

	#[test]
	fn lines_split_where_python_splits_them() {
		assert_eq!(
			splitlines("a\nb\r\nc\rd\x0be\x0cf\x1cg\u{85}h\u{2028}i\u{2029}j\n"),
			["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"]
		);
		assert_eq!(splitlines("OK hello\n"), ["OK hello"]);
		assert_eq!(splitlines("a\n\nb"), ["a", "", "b"]);
		assert_eq!(splitlines("\n"), [""]);
		assert!(splitlines("").is_empty());
		// \r\n is one boundary, \n\r two
		assert_eq!(splitlines("a\n\rb"), ["a", "", "b"]);
	}

	#[test]
	fn negative_numbers_are_words() {
		for w in ["-1", "-15", "-.5", "-1.5"] {
			assert!(negative_number(w), "{w}");
		}
		for w in ["-", "-x", "--1", "-1.", "-1e3", "-1.2.3", "1"] {
			assert!(!negative_number(w), "{w}");
		}
		let a = parse(&args(&["set", "track-probe-ch", "-1"])).unwrap();
		assert_eq!(a.words, ["set", "track-probe-ch", "-1"]);
		// a negative number is a value too
		let a = parse(&args(&["--timeout", "-1", "hello"])).unwrap();
		assert_eq!(a.timeout, -1.0);
	}

	#[test]
	fn options_as_argparse_reads_them() {
		let a = parse(&args(&["hello"])).unwrap();
		assert_eq!((a.host.as_str(), a.port, a.timeout, a.only.as_str()), ("127.0.0.1", 9100, 1.0, ""));
		// a unique prefix, the value after '=', options after the words
		let a = parse(&args(&["--po", "9360", "--timeout=0.5", "watch", "diag", "--only", "R,F"])).unwrap();
		assert_eq!((a.port, a.timeout, a.only.as_str()), (9360, 0.5, "R,F"));
		assert_eq!(a.words, ["watch", "diag"]);
		let a = parse(&args(&["hello", "--host", "localhost"])).unwrap();
		assert_eq!((a.host.as_str(), a.words.as_slice()), ("localhost", &["hello".to_string()][..]));
		// after "--" everything is a word
		let a = parse(&args(&["--", "--port", "1"])).unwrap();
		assert_eq!(a.words, ["--port", "1"]);
		// argparse's errors exit 2, the help 0, a port out of range 1
		for (words, code) in [
			(&[][..], 2),
			(&["--port"][..], 2),
			(&["--port", "x", "hello"][..], 2),
			(&["--timeout", "x", "hello"][..], 2),
			(&["--h", "hello"][..], 2),
			(&["--bogus", "hello"][..], 2),
			(&["-x", "hello"][..], 2),
			(&["--port", "--host", "hello"][..], 2),
			(&["--help"][..], 0),
			(&["--he", "hello"][..], 0),
			(&["-h"][..], 0),
			(&["--port", "70000", "hello"][..], 1),
		] {
			assert_eq!(parse(&args(words)).err(), Some(code), "{words:?}");
		}
	}
}
