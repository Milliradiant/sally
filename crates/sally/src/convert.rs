//! `sally convert`: recordings made by other means as HID tap recordings (sally-hid's line.rs), for `sally replay`, the
//! panel's Replay tab and the bench. Two converters:
//!
//! ```text
//! sally convert <capture.pcap> [<capture.pcap.usbdevs>] > <capture.hidtap>
//! sally convert [--project <folder>] [--logs <folder>] [--force] [--list] [<recording.lhraw | report.zip>...]
//! ```
//!
//! The first argument's extension says which: a `.pcap` is a USBPcap capture, anything else an openhouse recording
//! (a `.zip` one of its bug reports, with the recordings in it).
//!
//! **A USBPcap capture**: a capture of the dongles' USB traffic, as made before the HID tap. The tap recording goes to
//! stdout, the counts to stderr. Windows captures carry no device serials, so the devices get made-up ones
//! (`<codename><n>`, in the .usbdevs listing's order; the headset `HMD`) and a `D` line each before the first report,
//! which names them in a tap replay as a USB capture replay names them. Its interrupt IN transfers are `I` reports, its
//! SET_REPORTs `W` (feature) and `O` (output), the data of its control IN transfers `F`. Linux usbmon captures are not
//! read.
//!
//! **openhouse's recordings**: a .lhraw holds every HID report the dongles (and a headset on USB) sent, with the host's
//! time; openhouse writes them to %LOCALAPPDATA%\openhouse\recordings, and puts them in its bug report zips. Read from
//! openhouse's description of the format (src/transport/usb/raw_capture.hpp), no code of theirs: "LHRAW001", then
//! records: type 1, an endpoint (u16 ep, u16 vid, u16 pid, u8 interface, u8 kind, str serial, i8 gyro range, i8 accel
//! range, str config json); type 2, a report (f64 host time s, u16 ep, u8 last in batch, u16 len, the bytes, the report
//! id first); a str is a u32 length and its bytes; little endian. Each becomes
//! `<logs>\sallytap_<YYYYMMDD_HHMMSS>.hidtap` (the time from the recording's name, else its file's), with CR LF line
//! ends; one that exists is written again only with --force, and never over a recording sally made (one without this
//! converter's first line, which starts `# openhouse recording`). What the tap needs that the .lhraw says otherwise:
//! - the key: sally keys a wireless device by its dongle's serial, openhouse by the device's (LHR-...). The dongle that
//!   carries each device comes from the newest of sally's recordings that has `# cache <dongle> <device>` lines, else
//!   from the dongles' caches in survive\hidtap\devcfg; a device found in neither keeps its own serial as the key. A
//!   headset on USB is its own serial in both
//! - which device each dongle carries: an `A` line per dongle at the start, and the `# cache` line the panel reads to
//!   copy each device's config into a replay's folder
//! - the configs: the .lhraw carries each device's. A wireless device whose config devcfg has not (someone else's
//!   tracker), or has another one of, gets its config written to `<logs>\steamcfg_<stamp>_openhouse\`, a folder the
//!   panel's replay looks in
//! - the light's mode: not in the recording; sally's replay tells it from the light packets.
//!
//! With no recording named, it converts every report zip in Documents\openhouse reports and every
//! %LOCALAPPDATA%\openhouse\recordings\*.lhraw, each once. The project folder (its logs and survive\hidtap\devcfg) is
//! --project, else the first folder up from this program's or from the current one that has both. --list says what each
//! recording holds and converts nothing.
//!
//! A capture, listing, recording or zip that cannot be read is an error message (a zip's is reported and the run goes
//! on to the next one); a zip entry's time with no stamp in its name is the zip's; a serial that would name a file
//! elsewhere than the configs' folder ("../", a drive) gets no config written or read; a tap recording that cannot be
//! written leaves no `.tmp` behind; a name with a wildcard is matched in its own folder only. The JSON values compared
//! and the messages quoted are as Python's json module has them (`pyjson`), which earlier conversions were made with,
//! but for lone surrogates.

mod pyjson;
mod zip;

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufReader, Read, Write};
use std::path::{Component, Path, PathBuf};

use sally_math::printf::{sprintf, Arg};

use crate::text::{self, TextOut};

pub fn main(args: &[String]) -> i32 {
	match args.first() {
		Some(first) if first.to_ascii_lowercase().ends_with(".pcap") => {
			pcap::main(first, args.get(1).map(String::as_str))
		}
		_ => openhouse::main(args),
	}
}

/// "\x.." / "\u...." / "\U........": a character as Python's repr writes one it cannot print
fn py_escape(c: char) -> String {
	match c as u32 {
		v if v <= 0xff => format!("\\x{v:02x}"),
		v if v <= 0xffff => format!("\\u{v:04x}"),
		v => format!("\\U{v:08x}"),
	}
}

/// UnicodeEncodeError's text where the ascii codec cannot write `text` (None where it can)
fn ascii_error(text: &str) -> Option<String> {
	let chars: Vec<char> = text.chars().collect();
	let start = chars.iter().position(|c| !c.is_ascii())?;
	let end = chars[start..].iter().position(|c| c.is_ascii()).map_or(chars.len(), |n| start + n);
	Some(if end == start + 1 {
		format!(
			"'ascii' codec can't encode character '{}' in position {start}: ordinal not in range(128)",
			py_escape(chars[start])
		)
	} else {
		format!("'ascii' codec can't encode characters in position {start}-{}: ordinal not in range(128)", end - 1)
	})
}

/// bytes.decode('ascii', 'replace'): a byte past 127 is U+FFFD
fn ascii_replace(b: &[u8]) -> String {
	b.iter().map(|&c| if c < 0x80 { c as char } else { '\u{fffd}' }).collect()
}

/// f"{v:04x}": a negative value with its sign before the padding
fn hex4(v: i64) -> String {
	if v < 0 {
		format!("-{:03x}", v.unsigned_abs())
	} else {
		format!("{v:04x}")
	}
}

/// bytes.hex(): the bytes in lower case hex, appended
fn push_hex(out: &mut String, b: &[u8]) {
	const HEX: &[u8; 16] = b"0123456789abcdef";
	out.reserve(2 * b.len());
	for &c in b {
		out.push(HEX[(c >> 4) as usize] as char);
		out.push(HEX[(c & 15) as usize] as char);
	}
}

/// os.path.basename: the part after the last separator ('/', and '\' on Windows)
fn basename(name: &str) -> &str {
	let sep = |c: char| c == '/' || (cfg!(windows) && c == '\\');
	name.rsplit(sep).next().unwrap_or(name)
}

/// A file's modification time (s since 1970), as os.path.getmtime
fn mtime(path: &Path) -> std::io::Result<f64> {
	let m = std::fs::metadata(path)?.modified()?;
	Ok(match m.duration_since(std::time::UNIX_EPOCH) {
		Ok(d) => d.as_secs_f64(),
		Err(e) => -e.duration().as_secs_f64(),
	})
}

/// A file's content as Python's open() in text mode reads it with encoding='utf-8', errors='replace': a byte that is not
/// UTF-8 is U+FFFD, and a CR LF or a lone CR is a newline
fn read_text(path: &Path) -> std::io::Result<String> {
	let b = std::fs::read(path)?;
	Ok(String::from_utf8_lossy(&b).replace("\r\n", "\n").replace('\r', "\n"))
}

/// fnmatch on one name: `*`, `?`, `[...]` (`[!...]` the complement), case-insensitive on Windows
fn fnmatch(pat: &[char], name: &[char]) -> bool {
	let eq = |a: char, b: char| if cfg!(windows) { a.to_lowercase().eq(b.to_lowercase()) } else { a == b };
	match pat.first() {
		None => name.is_empty(),
		Some('*') => (0..=name.len()).any(|i| fnmatch(&pat[1..], &name[i..])),
		Some('?') => !name.is_empty() && fnmatch(&pat[1..], &name[1..]),
		Some('[') => {
			let Some(close) = pat.iter().skip(2).position(|&c| c == ']').map(|i| i + 2) else {
				return !name.is_empty() && eq('[', name[0]) && fnmatch(&pat[1..], &name[1..]);
			};
			let Some(&c) = name.first() else { return false };
			let (neg, set) = match pat[1] {
				'!' => (true, &pat[2..close]),
				_ => (false, &pat[1..close]),
			};
			let mut hit = false;
			let mut i = 0;
			while i < set.len() {
				if i + 2 < set.len() && set[i + 1] == '-' {
					hit |= (set[i]..=set[i + 2]).contains(&c);
					i += 3;
				} else {
					hit |= eq(set[i], c);
					i += 1;
				}
			}
			hit != neg && fnmatch(&pat[close + 1..], &name[1..])
		}
		Some(&p) => !name.is_empty() && eq(p, name[0]) && fnmatch(&pat[1..], &name[1..]),
	}
}

/// glob.glob for a pattern with wildcards in its last part: the names in its folder that match, in the folder's
/// order (a name starting with '.' only where the pattern does)
fn glob(pattern: &Path) -> Vec<PathBuf> {
	let Some(file) = pattern.file_name().map(|f| f.to_string_lossy().chars().collect::<Vec<char>>()) else {
		return Vec::new();
	};
	let dir = match pattern.parent() {
		Some(d) if !d.as_os_str().is_empty() => d.to_path_buf(),
		_ => PathBuf::from("."),
	};
	let Ok(rd) = std::fs::read_dir(&dir) else { return Vec::new() };
	rd.flatten()
		.filter_map(|e| {
			let name: Vec<char> = e.file_name().to_string_lossy().chars().collect();
			let hidden_ok = file.first() == Some(&'.') || name.first() != Some(&'.');
			(hidden_ok && fnmatch(&file, &name)).then(|| match pattern.parent() {
				Some(d) if !d.as_os_str().is_empty() => d.join(e.file_name()),
				_ => PathBuf::from(e.file_name()),
			})
		})
		.collect()
}

/// The USBPcap capture
mod pcap {
	use super::*;

	/// LINKTYPE_USBPCAP
	const USBPCAP_LINKTYPE: u32 = 249;
	const TRANSFER_INTERRUPT: u8 = 1;
	const TRANSFER_CONTROL: u8 = 2;
	const STAGE_SETUP: u8 = 0;
	const STAGE_COMPLETE: u8 = 3;
	/// USBPCAP_BUFFER_PACKET_HEADER: headerLen irpId status function info bus device endpoint transfer dataLength
	const PKT_HDR: usize = 27;

	/// A device of the listing
	#[derive(Clone, Debug, PartialEq)]
	pub struct Dev {
		pub pid: i64,
		pub serial: String,
	}

	/// The listing: each (bus, device address) with its device
	pub type Listing = Vec<((i64, i64), Dev)>;

	/// int(x): a sign, digits (an underscore between two)
	fn py_int(x: &str) -> Result<i64, String> {
		let bad = || format!("invalid literal for int() with base 10: '{x}'");
		let (neg, d) = match x.as_bytes().first() {
			Some(b'-') => (true, &x[1..]),
			Some(b'+') => (false, &x[1..]),
			_ => (false, x),
		};
		if d.is_empty() || d.starts_with('_') || d.ends_with('_') || d.contains("__") {
			return Err(bad());
		}
		let mut v: i64 = 0;
		for c in d.chars().filter(|&c| c != '_') {
			let digit = c.to_digit(10).ok_or_else(bad)?;
			v = v.checked_mul(10).and_then(|v| v.checked_add(digit as i64)).ok_or_else(bad)?;
		}
		Ok(if neg { -v } else { v })
	}

	/// read_usbdevs: "<vid> <pid> <bus> <dev> <codename>" lines, a line with fewer words skipped; each (bus, dev) its
	/// pid and its made-up serial (`<codename><n>`, n counting that codename's lines; the headset `HMD`), in the order
	/// first seen, a later line of the same (bus, dev) replacing it there
	pub fn read_usbdevs(text: &str) -> Result<Listing, String> {
		let mut devs: Listing = Vec::new();
		let mut counts: BTreeMap<String, usize> = BTreeMap::new();
		for line in text.split(['\n', '\r']) {
			let parts: Vec<&str> = line.split_whitespace().collect();
			if parts.len() < 5 {
				continue;
			}
			let _vid = py_int(parts[0])?;
			let (pid, bus, dev) = (py_int(parts[1])?, py_int(parts[2])?, py_int(parts[3])?);
			let codename = parts[4];
			let n = counts.entry(codename.to_string()).or_insert(0);
			let serial = if codename == "HMD" { "HMD".to_string() } else { format!("{codename}{n}") };
			*n += 1;
			let d = Dev { pid, serial };
			match devs.iter_mut().find(|(k, _)| *k == (bus, dev)) {
				Some(slot) => slot.1 = d,
				None => devs.push(((bus, dev), d)),
			}
		}
		Ok(devs)
	}

	/// The counts printed at the end
	#[derive(Default, Debug, PartialEq)]
	pub struct Stats {
		pub i: u64,
		pub f: u64,
		pub w: u64,
		pub o: u64,
		pub skipped: u64,
	}

	/// The capture's records, as pcap_packets reads them: (time in µs, the record's bytes), until one is cut short
	pub struct Packets<R: Read> {
		r: R,
		big: bool,
		nsec: bool,
	}

	impl<R: Read> Packets<R> {
		/// The global header: a pcap of microseconds (either byte order) or of nanoseconds (little endian), its link
		/// type USBPcap's
		pub fn open(mut r: R, path: &str) -> Result<Self, String> {
			let mut gh = [0u8; 24];
			let mut got = 0;
			while got < 24 {
				match r.read(&mut gh[got..]) {
					Ok(0) => break,
					Ok(n) => got += n,
					Err(e) => return Err(format!("{path}: {e}")),
				}
			}
			let not_pcap = || format!("not a pcap file: {path}");
			if got < 4 {
				return Err(not_pcap());
			}
			let (big, nsec) = match u32::from_le_bytes([gh[0], gh[1], gh[2], gh[3]]) {
				0xa1b2_c3d4 => (false, false),
				0xd4c3_b2a1 => (true, false),
				0xa1b2_3c4d => (false, true),
				_ => return Err(not_pcap()),
			};
			if got < 24 {
				return Err(not_pcap());
			}
			let lt = [gh[20], gh[21], gh[22], gh[23]];
			let linktype = if big { u32::from_be_bytes(lt) } else { u32::from_le_bytes(lt) };
			if linktype != USBPCAP_LINKTYPE {
				return Err(format!(
					"linktype {linktype} is not USBPcap ({USBPCAP_LINKTYPE}); Linux usbmon captures are not supported"
				));
			}
			Ok(Packets { r, big, nsec })
		}

		fn read_full(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
			let mut got = 0;
			while got < buf.len() {
				match self.r.read(&mut buf[got..]) {
					Ok(0) => break,
					Ok(n) => got += n,
					Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
					Err(e) => return Err(e),
				}
			}
			Ok(got)
		}

		/// The next record, None at the end (or where the last one is cut short)
		pub fn next_packet(&mut self) -> std::io::Result<Option<(u64, Vec<u8>)>> {
			let mut h = [0u8; 16];
			if self.read_full(&mut h)? < 16 {
				return Ok(None);
			}
			let u = |i: usize| {
				let b = [h[i], h[i + 1], h[i + 2], h[i + 3]];
				if self.big {
					u32::from_be_bytes(b)
				} else {
					u32::from_le_bytes(b)
				}
			};
			let (sec, frac, caplen) = (u(0), u(4), u(8));
			let mut data = vec![0; caplen as usize];
			if self.read_full(&mut data)? < data.len() {
				return Ok(None);
			}
			let t_us = sec as u64 * 1_000_000 + if self.nsec { frac as u64 / 1000 } else { frac as u64 };
			Ok(Some((t_us, data)))
		}
	}

	fn hex(b: &[u8]) -> String {
		let mut s = String::new();
		push_hex(&mut s, b);
		s
	}

	/// The capture's records converted, the lines to `out`: a `D` line per device before the first record, then per
	/// record of a listed device its `R` line, if it has one
	pub fn convert<R: Read, W: Write>(
		packets: &mut Packets<R>, devs: &[((i64, i64), Dev)], out: &mut TextOut<W>,
	) -> Result<Stats, String> {
		let mut stats = Stats::default();
		// irpId -> (bus, dev) of the control IN requests awaiting their completion
		let mut pending_in: BTreeMap<u64, (i64, i64)> = BTreeMap::new();
		let mut first = true;
		let io = |e: std::io::Error| e.to_string();
		while let Some((t_us, pkt)) = packets.next_packet().map_err(io)? {
			if first {
				for (_, d) in devs {
					out.put(&format!("D {t_us} {} {}\n", d.serial, hex4(d.pid))).map_err(io)?;
				}
				first = false;
			}
			if pkt.len() < PKT_HDR {
				continue;
			}
			let u16_at = |i: usize| u16::from_le_bytes([pkt[i], pkt[i + 1]]);
			let u32_at = |i: usize| u32::from_le_bytes([pkt[i], pkt[i + 1], pkt[i + 2], pkt[i + 3]]);
			let hlen = u16_at(0) as usize;
			let irp = u64::from_le_bytes(pkt[2..10].try_into().unwrap_or_default());
			let status = u32_at(10);
			let (bus, dev) = (u16_at(17) as i64, u16_at(19) as i64);
			let (ep, transfer) = (pkt[21], pkt[22]);
			let dlen = u32_at(23) as usize;
			let key = (bus, dev);
			let Some((_, d)) = devs.iter().find(|(k, _)| *k == key) else { continue };
			let (pid, serial) = (hex4(d.pid), &d.serial);
			let payload = &pkt[hlen.min(pkt.len())..hlen.saturating_add(dlen).min(pkt.len())];

			if transfer == TRANSFER_INTERRUPT {
				if ep & 0x80 != 0 && status == 0 && dlen > 0 {
					let e = ep as i64 - 0x81;
					out.put(&format!("R {t_us} {serial} {pid} {e} -1 I {}\n", hex(payload))).map_err(io)?;
					stats.i += 1;
				}
				continue;
			}
			if transfer != TRANSFER_CONTROL || hlen < PKT_HDR + 1 {
				stats.skipped += 1;
				continue;
			}
			// the stage byte follows the header (a record that ends before it: the Python's IndexError)
			let stage = *pkt.get(PKT_HDR).ok_or("index out of range")?;
			// a capture cut at its snap length can cut the setup bytes
			if stage == STAGE_SETUP && dlen >= 8 && payload.len() >= 8 {
				let (request_type, request) = (payload[0], payload[1]);
				let w_value = u16::from_le_bytes([payload[2], payload[3]]);
				let w_index = u16::from_le_bytes([payload[4], payload[5]]);
				if request_type & 0x80 != 0 {
					pending_in.insert(irp, key);
				} else if request_type == 0x21 && request == 0x09 && dlen > 8 {
					// SET_REPORT: USBPcap 1.5 and later put the OUT data right after the 8 setup bytes
					let kind = match w_value >> 8 {
						3 => Some(('W', &mut stats.w)),
						2 => Some(('O', &mut stats.o)),
						_ => None,
					};
					if let Some((k, n)) = kind {
						out.put(&format!("R {t_us} {serial} {pid} {w_index} -1 {k} {}\n", hex(&payload[8..])))
							.map_err(io)?;
						*n += 1;
					}
				}
			} else if stage == STAGE_COMPLETE && pending_in.remove(&irp).is_some() && status == 0 && dlen > 0 {
				out.put(&format!("R {t_us} {serial} {pid} -1 -1 F {}\n", hex(payload))).map_err(io)?;
				stats.f += 1;
			}
		}
		Ok(stats)
	}

	/// `sally convert <capture.pcap> [<listing>]`: the listing is the capture's name with .usbdevs after it unless named
	pub fn main(pcap: &str, usbdevs: Option<&str>) -> i32 {
		let usbdevs = usbdevs.map_or_else(|| format!("{pcap}.usbdevs"), str::to_string);
		let fail = |m: String| {
			text::err(&format!("sally convert: {m}\n"));
			1
		};
		let devs = match std::fs::read(&usbdevs) {
			Ok(b) => match read_usbdevs(&String::from_utf8_lossy(&b)) {
				Ok(d) => d,
				Err(e) => return fail(format!("{usbdevs}: {e}")),
			},
			Err(e) => return fail(format!("{usbdevs}: {e}")),
		};
		let f = match File::open(pcap) {
			Ok(f) => f,
			Err(e) => return fail(format!("{pcap}: {e}")),
		};
		let mut packets = match Packets::open(BufReader::with_capacity(1 << 20, f), pcap) {
			Ok(p) => p,
			Err(e) => return fail(e),
		};
		let mut out = TextOut::new(std::io::stdout().lock());
		let r = convert(&mut packets, &devs, &mut out);
		let flushed = out.flush();
		match r {
			Ok(s) => {
				if let Err(e) = flushed {
					return fail(e.to_string());
				}
				text::err(&format!(
					"sally convert: {{'I': {}, 'F': {}, 'W': {}, 'O': {}, 'skipped': {}}}\n",
					s.i, s.f, s.w, s.o, s.skipped
				));
				0
			}
			Err(e) => fail(e),
		}
	}
}

/// openhouse's recordings
mod openhouse {
	use super::pyjson::{self, Value};
	use super::*;

	/// The first line's start: a tap recording this converter wrote
	pub const MARK: &str = "# openhouse recording";

	/// A wireless device, through its dongle
	const RF_PID: u16 = 0x2102;

	/// A type 1 record
	#[derive(Clone, Debug, PartialEq)]
	pub struct Endpoint {
		pub vid: u16,
		pub pid: u16,
		pub iface: u8,
		pub kind: u8,
		pub serial: String,
		pub config: Vec<u8>,
	}

	/// A report: the host's time (s), its endpoint, its bytes
	pub type Report<'a> = (f64, u16, &'a [u8]);

	/// struct.error's text where `size` bytes at `p` are past the end
	fn short(size: usize, p: usize, n: usize) -> String {
		format!(
			"unpack_from requires a buffer of at least {} bytes for unpacking {size} bytes at offset {p} (actual buffer \
			 size is {n})",
			p + size
		)
	}

	/// read_str: a u32 length and up to that many bytes (fewer where the data ends), and where the next thing starts
	fn read_str(data: &[u8], p: usize) -> Result<(&[u8], usize), String> {
		if p + 4 > data.len() {
			return Err(short(4, p, data.len()));
		}
		let n = u32::from_le_bytes([data[p], data[p + 1], data[p + 2], data[p + 3]]) as usize;
		let end = p + 4 + n;
		Ok((&data[(p + 4).min(data.len())..end.min(data.len())], end))
	}

	/// parse: the endpoints by number (a later record of the same one replacing it) and the reports in the file's order;
	/// a report cut short ends the recording (one stopped mid-write)
	pub fn parse(data: &[u8]) -> Result<(BTreeMap<u16, Endpoint>, Vec<Report<'_>>), String> {
		if !data.starts_with(b"LHRAW001") {
			return Err("not an openhouse recording (no LHRAW001 header)".into());
		}
		let n = data.len();
		let (mut eps, mut reps) = (BTreeMap::new(), Vec::new());
		let mut p = 8;
		while p < n {
			let typ = data[p];
			p += 1;
			match typ {
				1 => {
					if p + 8 > n {
						return Err(short(8, p, n));
					}
					let u16_at = |i: usize| u16::from_le_bytes([data[i], data[i + 1]]);
					let (ep, vid, pid, iface, kind) =
						(u16_at(p), u16_at(p + 2), u16_at(p + 4), data[p + 6], data[p + 7]);
					p += 8;
					let (serial, q) = read_str(data, p)?;
					p = q;
					if p + 2 > n {
						return Err(short(2, p, n));
					}
					p += 2;
					let (config, q) = read_str(data, p)?;
					p = q;
					let serial = ascii_replace(serial);
					eps.insert(ep, Endpoint { vid, pid, iface, kind, serial, config: config.to_vec() });
				}
				2 => {
					if p + 13 > n {
						break;
					}
					let t = f64::from_le_bytes(data[p..p + 8].try_into().unwrap_or_default());
					let ep = u16::from_le_bytes([data[p + 8], data[p + 9]]);
					let len = u16::from_le_bytes([data[p + 11], data[p + 12]]) as usize;
					p += 13;
					if p + len > n {
						break;
					}
					reps.push((t, ep, &data[p..p + len]));
					p += len;
				}
				_ => return Err(format!("unknown record type {typ} at byte {}", p - 1)),
			}
		}
		Ok((eps, reps))
	}

	/// stamp_of: "<YYYYMMDD>_<HHMMSS>" from the first 8 digits, a '-' and 6 digits in the name, else None
	pub fn stamp_in(name: &str) -> Option<String> {
		let c: Vec<char> = basename(name).chars().collect();
		let digits = |a: usize, b: usize| c[a..b].iter().all(|d| d.is_ascii_digit());
		(0..c.len().saturating_sub(14)).find(|&i| digits(i, i + 8) && c[i + 8] == '-' && digits(i + 9, i + 15)).map(
			|i| format!("{}_{}", c[i..i + 8].iter().collect::<String>(), c[i + 9..i + 15].iter().collect::<String>()),
		)
	}

	/// A file's time as a stamp, in local time
	fn stamp_of_time(path: &Path) -> Result<String, String> {
		let t = mtime(path).map_err(|e| format!("{}: {e}", path.display()))?;
		sally_os::localtime::local_time(t.floor() as i64)
			.map(|lt| lt.stamp())
			.ok_or_else(|| format!("{}: no local time for its time", path.display()))
	}

	/// A recording to convert: its stamp, its name as the messages give it, its bytes
	pub struct Source {
		pub stamp: String,
		pub name: String,
		pub data: Vec<u8>,
	}

	/// sources: each recording named, a .lhraw or each .lhraw in a report .zip. A name that cannot be read: its error
	fn sources(path: &Path) -> Vec<Result<Source, String>> {
		let shown = path.display().to_string();
		let fail = |m: String| vec![Err(format!("{shown}: FAILED {m}"))];
		if shown.to_lowercase().ends_with(".zip") {
			let mut f = match File::open(path) {
				Ok(f) => f,
				Err(e) => return fail(e.to_string()),
			};
			let entries = match zip::entries(&mut f) {
				Ok(e) => e,
				Err(e) => return fail(e),
			};
			let zip_name = basename(&shown).to_string();
			entries
				.iter()
				.filter(|e| e.name.to_lowercase().ends_with(".lhraw"))
				.map(|e| {
					let name = format!("{} (in {zip_name})", basename(&e.name));
					let stamp = match stamp_in(&e.name) {
						Some(s) => s,
						None => stamp_of_time(path).map_err(|m| format!("{name}: FAILED {m}"))?,
					};
					let data = zip::read(&mut f, e).map_err(|m| format!("{name}: FAILED {m}"))?;
					Ok(Source { stamp, name, data })
				})
				.collect()
		} else {
			let name = basename(&shown).to_string();
			let stamp = match stamp_in(&shown) {
				Some(s) => s,
				None => match stamp_of_time(path) {
					Ok(s) => s,
					Err(e) => return fail(e),
				},
			};
			match std::fs::read(path) {
				Ok(data) => vec![Ok(Source { stamp, name, data })],
				Err(e) => fail(e.to_string()),
			}
		}
	}

	/// str.split(): at white space (ASCII's, and the separators 0x1c-0x1f)
	fn py_split(s: &str) -> Vec<&str> {
		s.split(|c: char| c.is_ascii_whitespace() || c == '\x0b' || ('\x1c'..='\x1f').contains(&c))
			.filter(|w| !w.is_empty())
			.collect()
	}

	/// str.splitlines() of ASCII text (and U+FFFD)
	fn py_splitlines(s: &str) -> Vec<&str> {
		let mut out = Vec::new();
		let b = s.as_bytes();
		let (mut start, mut i) = (0, 0);
		while i < b.len() {
			if matches!(b[i], b'\n' | b'\r' | 0x0b | 0x0c | 0x1c | 0x1d | 0x1e) {
				out.push(&s[start..i]);
				i += if b[i] == b'\r' && b.get(i + 1) == Some(&b'\n') { 2 } else { 1 };
				start = i;
			} else {
				i += 1;
			}
		}
		if start < b.len() {
			out.push(&s[start..]);
		}
		out
	}

	/// A name ends with `suffix` (as glob matches it: regardless of case on Windows)
	fn ends_with(name: &str, suffix: &str) -> bool {
		if cfg!(windows) {
			name.to_lowercase().ends_with(suffix)
		} else {
			name.ends_with(suffix)
		}
	}

	/// The files of a folder whose names start with `prefix` and end with `suffix` (and do not start with '.'), in the
	/// folder's order: glob.glob(<dir>/<prefix>*<suffix>)
	fn files_like(dir: &Path, prefix: &str, suffix: &str) -> Vec<(String, PathBuf)> {
		let Ok(rd) = std::fs::read_dir(dir) else { return Vec::new() };
		rd.flatten()
			.filter_map(|e| {
				let name = e.file_name().to_string_lossy().into_owned();
				let starts =
					if cfg!(windows) { name.to_lowercase().starts_with(prefix) } else { name.starts_with(prefix) };
				let ok = starts && ends_with(&name, suffix) && name.len() >= prefix.len() + suffix.len();
				(ok && !name.starts_with('.')).then(|| (name, e.path()))
			})
			.collect()
	}

	/// str.upper() of a device serial (or a dongle's, from a recording)
	fn upper(s: &str) -> String {
		s.to_uppercase()
	}

	/// dongles_of: device serial (upper case) -> the dongle that carries it. devcfg's dongle caches (not the devices' own
	/// LHR-*, nor the KN* of old) by their device_serial_number, in name order; then the `# cache` lines of the newest of
	/// sally's own recordings (the 20 newest looked at) that has any
	pub fn dongles_of(logs: &Path, devcfg: &Path) -> Result<BTreeMap<String, String>, String> {
		let mut out = BTreeMap::new();
		let mut cfgs = files_like(devcfg, "", "_config.json");
		cfgs.sort();
		for (name, path) in cfgs {
			let Some(key) = name.get(..name.len() - "_config.json".len()) else { continue };
			if key.starts_with("LHR-") || key.starts_with("KN") {
				continue;
			}
			let Ok(text) = read_text(&path) else { continue };
			let Ok(v) = pyjson::loads_text(&text) else { continue };
			// .get on what is not a dict: the Python's AttributeError, caught
			if !matches!(v, Value::Dict(_)) {
				continue;
			}
			match v.get("device_serial_number") {
				Some(Value::Str(dev)) if !dev.is_empty() => {
					out.insert(upper(dev), key.to_string());
				}
				Some(dev) if dev.truthy() => {
					return Err(format!("'{}' object has no attribute 'upper'", dev.type_name()));
				}
				_ => {}
			}
		}
		// (sally: the recordings of either form, a packed one read as the text it holds)
		let mut taps: Vec<(f64, PathBuf)> = files_like(logs, "sallytap_", ".hidtap")
			.into_iter()
			.chain(files_like(logs, "sallytap_", ".hidtapz"))
			.map(|(_, p)| mtime(&p).map(|t| (t, p)))
			.collect::<std::io::Result<_>>()
			.map_err(|e| e.to_string())?;
		// newest first, ties in the folder's order
		taps.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
		for (_, t) in taps.iter().take(20) {
			let mut head = Vec::new();
			sally_hid::pack::open(t).and_then(|f| f.take(1 << 16).read_to_end(&mut head)).map_err(|e| e.to_string())?;
			let head = ascii_replace(&head);
			if head.starts_with(MARK) {
				continue;
			}
			let mut found = false;
			for l in py_splitlines(&head) {
				let w = py_split(l);
				if w.len() == 4 && w[0] == "#" && w[1] == "cache" {
					out.insert(upper(w[3]), w[2].to_string());
					found = true;
				}
			}
			if found {
				break;
			}
		}
		Ok(out)
	}

	/// A serial that can only name a file in the configs' folder (the recordings come from anyone: one with "../" or a
	/// drive in a serial would name a file anywhere)
	fn plain(serial: &str) -> bool {
		!serial.is_empty() && !serial.contains(['/', '\\', ':', '.', '\0'])
	}

	/// os.path.relpath(path, start), for the messages: lexically, from both made absolute (the full path where they
	/// have no root in common)
	pub fn relpath(path: &Path, start: &Path) -> String {
		let (Ok(p), Ok(s)) = (std::path::absolute(path), std::path::absolute(start)) else {
			return path.display().to_string();
		};
		fn norm(p: &Path) -> Vec<Component<'_>> {
			let mut v: Vec<Component> = Vec::new();
			for c in p.components() {
				match c {
					Component::CurDir => {}
					Component::ParentDir if matches!(v.last(), Some(Component::Normal(_))) => {
						v.pop();
					}
					c => v.push(c),
				}
			}
			v
		}
		let (pc, sc) = (norm(&p), norm(&s));
		if pc.first() != sc.first() {
			return p.display().to_string();
		}
		let common = pc.iter().zip(&sc).take_while(|(a, b)| a == b).count();
		let mut r = PathBuf::new();
		for _ in common..sc.len() {
			r.push("..");
		}
		for c in &pc[common..] {
			r.push(c);
		}
		if r.as_os_str().is_empty() {
			".".into()
		} else {
			r.display().to_string()
		}
	}

	/// round(x) as an int, for `%d`: halves to even; NaN and the infinities fail as int() of them does
	fn py_round(x: f64) -> Result<String, String> {
		if x.is_nan() {
			return Err("cannot convert float NaN to integer".into());
		}
		if x.is_infinite() {
			return Err("cannot convert float infinity to integer".into());
		}
		let r = x.round_ties_even();
		Ok(if r == 0.0 { "0".into() } else { format!("{r:.0}") })
	}

	/// Where the converter reads and writes
	pub struct Places {
		pub project: PathBuf,
		pub logs: PathBuf,
		pub devcfg: PathBuf,
	}

	/// convert: one recording into `<logs>\sallytap_<stamp>.hidtap`; what it did, or why it did not (an Err is the
	/// Python's exception, "FAILED <it>")
	pub fn convert(src: &Source, at: &Places, force: bool, now: &str) -> Result<String, String> {
		let (eps, mut reps) = parse(&src.data)?;
		if reps.is_empty() {
			return Ok("no reports".into());
		}
		let out_name = format!("sallytap_{}.hidtap", src.stamp);
		let out = at.logs.join(&out_name);
		if out.exists() {
			let mut head = Vec::new();
			File::open(&out)
				.and_then(|f| f.take(MARK.len() as u64).read_to_end(&mut head))
				.map_err(|e| e.to_string())?;
			if ascii_replace(&head) != MARK {
				return Ok(format!("not written: {out_name} is a recording sally made"));
			}
			if !force {
				return Ok(format!("already converted: {out_name} (--force writes it again)"));
			}
		}
		let dongle = dongles_of(&at.logs, &at.devcfg)?;
		let mut key: BTreeMap<u16, String> = BTreeMap::new();
		let mut notes: Vec<String> = Vec::new();
		for (&ep, e) in &eps {
			if e.pid == RF_PID {
				let k = match dongle.get(&upper(&e.serial)).filter(|k| !k.is_empty()) {
					Some(k) => k.clone(),
					None => {
						notes.push(format!("{}: no dongle known for it, keyed by its own serial", e.serial));
						e.serial.clone()
					}
				};
				key.insert(ep, k);
			} else {
				key.insert(ep, e.serial.clone());
			}
		}
		// the wireless devices' configs devcfg does not have (the same: every key devcfg's has, openhouse's extra ones
		// left out of it)
		let cdir = at.logs.join(format!("steamcfg_{}_openhouse", src.stamp));
		for e in eps.values() {
			if e.pid != RF_PID || e.config.is_empty() {
				continue;
			}
			let theirs = match pyjson::loads_bytes(&e.config) {
				Ok(v) => v,
				Err(pyjson::Error::Value(m)) => {
					notes.push(format!("{}: its config does not read ({m}): none written", e.serial));
					continue;
				}
				Err(pyjson::Error::Recursion(m)) => return Err(m),
			};
			if !plain(&e.serial) {
				notes.push(format!("{}: its serial names no file: none written", e.serial));
				continue;
			}
			let why = match read_text(&at.devcfg.join(format!("{}_config.json", e.serial))) {
				Err(err) if err.kind() == std::io::ErrorKind::NotFound => "devcfg has no config for it",
				Err(err) => return Err(err.to_string()),
				Ok(text) => {
					let mine = match pyjson::loads_text(&text) {
						Ok(v) => v,
						Err(pyjson::Error::Value(m)) => {
							notes.push(format!("{}: its config does not read ({m}): none written", e.serial));
							continue;
						}
						Err(pyjson::Error::Recursion(m)) => return Err(m),
					};
					let Value::Dict(items) = &mine else {
						return Err(format!("'{}' object has no attribute 'items'", mine.type_name()));
					};
					if !items.is_empty() && !matches!(theirs, Value::Dict(_)) {
						return Err(format!("'{}' object has no attribute 'get'", theirs.type_name()));
					}
					if items.iter().all(|(k, v)| pyjson::py_eq(theirs.get(k).unwrap_or(&Value::Null), v)) {
						continue;
					}
					"devcfg has another config for it"
				}
			};
			std::fs::create_dir_all(&cdir).map_err(|e| e.to_string())?;
			std::fs::write(cdir.join(format!("{}_config.json", e.serial)), &e.config).map_err(|e| e.to_string())?;
			notes.push(format!("{}: {why}; the recording's written to {}", e.serial, relpath(&cdir, &at.project)));
		}
		// by time; reports of one time keep their order
		reps.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or_else(|| a.0.is_nan().cmp(&b.0.is_nan())));
		let t0 = reps[0].0;
		let tmp = at.logs.join(format!("{out_name}.tmp"));
		let written = write_tap(&tmp, src, &eps, &key, &reps, t0, now);
		let n = match written {
			Ok(n) => n,
			Err(e) => {
				let _ = std::fs::remove_file(&tmp);
				return Err(e);
			}
		};
		std::fs::rename(&tmp, &out).map_err(|e| e.to_string())?;
		let span = reps[reps.len() - 1].0 - t0;
		Ok(format!(
			"wrote {out_name}: {n} reports over {} s from {} devices{}",
			sprintf("%.0f", &[Arg::F(span)]),
			eps.len(),
			notes.iter().map(|x| format!("\n    {x}")).collect::<String>()
		))
	}

	/// The tap recording, in CR LF lines of ASCII: the first line, the endpoints, the dongles' `# cache` and `A` lines,
	/// then each report of a known endpoint. The reports written
	fn write_tap(
		path: &Path, src: &Source, eps: &BTreeMap<u16, Endpoint>, key: &BTreeMap<u16, String>, reps: &[Report],
		t0: f64, now: &str,
	) -> Result<u64, String> {
		let f = File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
		let mut out = TextOut::with_crlf(f, true);
		let n = write_lines(&mut out, src, eps, key, reps, t0, now)?;
		out.flush().map_err(|e| e.to_string())?;
		Ok(n)
	}

	/// write_tap's lines, each refused where the ascii codec cannot write it
	fn write_lines(
		out: &mut TextOut<File>, src: &Source, eps: &BTreeMap<u16, Endpoint>, key: &BTreeMap<u16, String>,
		reps: &[Report], t0: f64, now: &str,
	) -> Result<u64, String> {
		let mut put = |line: &str| -> Result<(), String> {
			if !line.is_ascii() {
				return Err(ascii_error(line).unwrap_or_default());
			}
			out.put(line).map_err(|e| e.to_string())
		};
		put(&format!("{MARK} {}, converted by sally convert on {now}\n", src.name))?;
		for (ep, e) in eps {
			put(&format!(
				"# endpoint {ep}: {:04x}:{:04x} interface {} kind {} {} -> {}\n",
				e.vid, e.pid, e.iface, e.kind, e.serial, key[ep]
			))?;
		}
		let carried = || eps.iter().filter(|(ep, e)| e.pid == RF_PID && key[*ep] != e.serial);
		for (ep, e) in carried() {
			put(&format!("# cache {} {}\n", key[ep], e.serial))?;
		}
		let a_time = if carried().next().is_some() { py_round(t0 * 1e6)? } else { String::new() };
		for (ep, e) in carried() {
			put(&format!("A {a_time} {} {}\n", key[ep], e.serial))?;
		}
		let mut n = 0;
		let mut line = String::new();
		for &(t, ep, b) in reps {
			let Some(e) = eps.get(&ep) else { continue };
			line.clear();
			line.push_str(&format!("R {} {} {:04x} {} -1 I ", py_round(t * 1e6)?, key[&ep], e.pid, e.iface));
			push_hex(&mut line, b);
			line.push('\n');
			put(&line)?;
			n += 1;
		}
		Ok(n)
	}

	/// --list: what a recording holds (its reports' span in the file's order)
	fn list(src: &Source) -> Result<String, String> {
		let (eps, reps) = parse(&src.data)?;
		let span = match (reps.first(), reps.last()) {
			(Some(a), Some(b)) => b.0 - a.0,
			_ => 0.0,
		};
		let mut s = format!(
			"{}: {} devices, {} reports over {} s\n",
			src.name,
			eps.len(),
			reps.len(),
			sprintf("%.0f", &[Arg::F(span)])
		);
		for (ep, e) in &eps {
			s.push_str(&format!(
				"  ep {ep} {:04x}:{:04x} if {} kind {} {} config {} bytes\n",
				e.vid,
				e.pid,
				e.iface,
				e.kind,
				e.serial,
				e.config.len()
			));
		}
		Ok(s)
	}

	/// The project folder: the first folder up from this program's, then from the current one, with logs and
	/// survive\hidtap\devcfg in it (the Python's was its scripts folder's parent)
	fn find_project() -> Option<PathBuf> {
		let has = |d: &Path| d.join("logs").is_dir() && d.join("survive").join("hidtap").join("devcfg").is_dir();
		let exe_dir = std::env::current_exe().ok().and_then(|e| e.parent().map(Path::to_path_buf));
		for start in [exe_dir, std::env::current_dir().ok()].into_iter().flatten() {
			if let Some(d) = start.ancestors().find(|d| has(d)) {
				return Some(d.to_path_buf());
			}
		}
		None
	}

	/// The Documents folder (Windows' known folder, so a moved one too)
	#[cfg(windows)]
	fn documents() -> Option<PathBuf> {
		use std::ffi::c_void;
		use std::os::windows::ffi::OsStringExt;
		#[repr(C)]
		struct Guid(u32, u16, u16, [u8; 8]);
		const FOLDERID_DOCUMENTS: Guid =
			Guid(0xfdd3_9ad0, 0x238f, 0x46af, [0xad, 0xb4, 0x6c, 0x85, 0x48, 0x03, 0x69, 0xc7]);
		#[link(name = "shell32")]
		extern "system" {
			fn SHGetKnownFolderPath(rfid: *const Guid, flags: u32, token: *mut c_void, path: *mut *mut u16) -> i32;
		}
		#[link(name = "ole32")]
		extern "system" {
			fn CoTaskMemFree(pv: *mut c_void);
		}
		let mut p: *mut u16 = std::ptr::null_mut();
		// SAFETY: the call writes a string it allocated into `p`, which is freed below whatever it returned
		let hr = unsafe { SHGetKnownFolderPath(&FOLDERID_DOCUMENTS, 0, std::ptr::null_mut(), &mut p) };
		let out = (hr == 0 && !p.is_null()).then(|| {
			// SAFETY: a string of u16 up to its 0, as the call made it
			let len = (0..).take_while(|&i| unsafe { *p.add(i) } != 0).count();
			// SAFETY: the `len` units before the 0
			std::ffi::OsString::from_wide(unsafe { std::slice::from_raw_parts(p, len) }).into()
		});
		// SAFETY: the call's string (null is allowed)
		unsafe { CoTaskMemFree(p.cast()) };
		out
	}

	#[cfg(not(windows))]
	fn documents() -> Option<PathBuf> {
		std::env::var_os("HOME").map(|h| PathBuf::from(h).join("Documents"))
	}

	/// openhouse's recordings on this machine: every report zip in Documents\openhouse reports, then every
	/// %LOCALAPPDATA%\openhouse\recordings\*.lhraw, each in name order
	fn openhouse_recordings() -> Vec<PathBuf> {
		let in_dir = |dir: Option<PathBuf>, ext: &str| -> Vec<PathBuf> {
			let Some(dir) = dir else { return Vec::new() };
			let Ok(rd) = std::fs::read_dir(dir) else { return Vec::new() };
			let mut v: Vec<PathBuf> = rd
				.flatten()
				.filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
				.filter(|e| e.file_name().to_string_lossy().to_lowercase().ends_with(ext))
				.map(|e| e.path())
				.collect();
			v.sort_by_key(|p| p.to_string_lossy().to_lowercase());
			v
		};
		let mut v = in_dir(documents().map(|d| d.join("openhouse reports")), ".zip");
		let recordings =
			std::env::var_os("LOCALAPPDATA").map(|d| PathBuf::from(d).join("openhouse").join("recordings"));
		v.extend(in_dir(recordings, ".lhraw"));
		v
	}

	const USAGE: &str = "usage: sally convert <capture.pcap> [<capture.pcap.usbdevs>] > <capture.hidtap>\n       \
	                     sally convert [--project <folder>] [--logs <folder>] [--force] [--list] [<recording.lhraw | \
	                     report.zip>...]\n";

	pub fn main(args: &[String]) -> i32 {
		let (mut project, mut logs, mut force, mut list_only) = (None, None, false, false);
		let mut named: Vec<String> = Vec::new();
		let mut it = args.iter();
		while let Some(a) = it.next() {
			match a.as_str() {
				"--project" | "--logs" => {
					let Some(v) = it.next() else {
						text::err(&format!("sally convert: {a} needs a folder\n{USAGE}"));
						return 2;
					};
					*(if a == "--project" { &mut project } else { &mut logs }) = Some(PathBuf::from(v));
				}
				"--force" => force = true,
				"--list" => list_only = true,
				"-h" | "--help" => {
					text::out(USAGE);
					return 0;
				}
				_ => named.push(a.clone()),
			}
		}
		let Some(project) =
			project.or_else(find_project).or_else(|| logs.as_ref().and_then(|l| l.parent().map(Path::to_path_buf)))
		else {
			text::err(
				"sally convert: no project folder (one with logs and survive\\hidtap\\devcfg) above this program or the \
				 current folder: name it with --project\n",
			);
			return 1;
		};
		let at = Places {
			logs: logs.unwrap_or_else(|| project.join("logs")),
			devcfg: project.join("survive").join("hidtap").join("devcfg"),
			project,
		};
		let paths: Vec<PathBuf> = if named.is_empty() {
			let found = openhouse_recordings();
			if found.is_empty() {
				text::out("No openhouse recordings found.\n");
				return 0;
			}
			found
		} else {
			named
				.iter()
				.flat_map(|a| {
					let p = PathBuf::from(a);
					let wild = p.file_name().is_some_and(|f| f.to_string_lossy().contains(['*', '?', '[']));
					let g = if wild { glob(&p) } else { Vec::new() };
					if g.is_empty() {
						vec![p]
					} else {
						g
					}
				})
				.collect()
		};
		let lt = sally_os::localtime::local_now().unwrap_or_default();
		let now = format!("{:04}-{:02}-{:02} {:02}:{:02}", lt.year, lt.month, lt.day, lt.hour, lt.minute);
		let mut code = 0;
		for path in &paths {
			for src in sources(path) {
				let src = match src {
					Ok(s) => s,
					Err(m) => {
						text::out(&format!("{m}\n"));
						code = 1;
						continue;
					}
				};
				if list_only {
					match list(&src) {
						Ok(s) => text::out(&s),
						Err(m) => text::out(&format!("{}: FAILED {m}\n", src.name)),
					}
					continue;
				}
				match convert(&src, &at, force, &now) {
					Ok(m) => text::out(&format!("{}: {m}\n", src.name)),
					Err(m) => text::out(&format!("{}: FAILED {m}\n", src.name)),
				}
			}
		}
		code
	}

	#[cfg(test)]
	mod tests {
		use super::*;

		/// A .lhraw of these endpoints and reports
		pub fn lhraw(eps: &[(u16, u16, &str, &str)], reps: &[(f64, u16, &[u8])]) -> Vec<u8> {
			let mut d = b"LHRAW001".to_vec();
			let s = |d: &mut Vec<u8>, b: &[u8]| {
				d.extend((b.len() as u32).to_le_bytes());
				d.extend(b);
			};
			for &(ep, pid, serial, config) in eps {
				d.push(1);
				for v in [ep, 0x28de, pid] {
					d.extend(v.to_le_bytes());
				}
				d.extend([0, if pid == RF_PID { 2 } else { 0 }]);
				s(&mut d, serial.as_bytes());
				d.extend([16u8, 8]);
				s(&mut d, config.as_bytes());
			}
			for &(t, ep, b) in reps {
				d.push(2);
				d.extend(t.to_le_bytes());
				d.extend(ep.to_le_bytes());
				d.push(1);
				d.extend((b.len() as u16).to_le_bytes());
				d.extend(b);
			}
			d
		}

		#[test]
		fn a_recording_parses() {
			let d = lhraw(
				&[(3, RF_PID, "LHR-AAAA0001", "{}"), (0, 0x2300, "LHR-HMD", "")],
				&[(2.5, 3, b"\x23\x01"), (1.0, 0, b"\x01")],
			);
			let (eps, reps) = parse(&d).unwrap();
			assert_eq!(eps.keys().copied().collect::<Vec<_>>(), [0, 3]);
			assert_eq!(eps[&3].serial, "LHR-AAAA0001");
			assert_eq!(eps[&3].config, b"{}");
			assert_eq!(reps, [(2.5, 3, &b"\x23\x01"[..]), (1.0, 0, &b"\x01"[..])]);
			// a report cut short ends it; an endpoint cut short is the Python's struct.error
			assert_eq!(parse(&d[..d.len() - 1]).unwrap().1.len(), 1);
			let cut = &lhraw(&[(3, RF_PID, "LHR-AAAA0001", "{}")], &[])[..14];
			assert_eq!(
				parse(cut).unwrap_err(),
				"unpack_from requires a buffer of at least 17 bytes for unpacking 8 bytes at offset 9 (actual buffer size \
				 is 14)"
			);
			assert_eq!(parse(b"LHRAW001\x07").unwrap_err(), "unknown record type 7 at byte 8");
			assert_eq!(parse(b"LHRAW00").unwrap_err(), "not an openhouse recording (no LHRAW001 header)");
			// a serial's bytes past ASCII
			let (eps, _) = parse(&lhraw(&[(1, RF_PID, "LHR-\u{e9}", "")], &[])).unwrap();
			assert_eq!(eps[&1].serial, "LHR-\u{fffd}\u{fffd}");
		}

		#[test]
		fn stamps_and_rounding() {
			assert_eq!(stamp_in("recording-20261108-190003.lhraw").as_deref(), Some("20261108_190003"));
			assert_eq!(stamp_in("x/123456789-123456.lhraw").as_deref(), Some("23456789_123456"));
			assert_eq!(stamp_in("recording.lhraw"), None);
			assert_eq!(py_round(2.5).unwrap(), "2");
			assert_eq!(py_round(3.5).unwrap(), "4");
			assert_eq!(py_round(-0.4).unwrap(), "0");
			assert_eq!(py_round(1e300).unwrap().len(), 301);
			assert_eq!(py_round(f64::NAN).unwrap_err(), "cannot convert float NaN to integer");
			assert_eq!(
				ascii_error("R 1 ab\u{fffd} x\n").as_deref(),
				Some("'ascii' codec can't encode character '\\ufffd' in position 6: ordinal not in range(128)")
			);
			assert_eq!(
				ascii_error("R 1 \u{fffd}\u{fffd}\n").as_deref(),
				Some("'ascii' codec can't encode characters in position 4-5: ordinal not in range(128)")
			);
			assert_eq!(ascii_error("R 1 x\n"), None);
		}

		#[test]
		fn a_recording_converts() {
			let dir = std::env::temp_dir().join(format!("sally-convert-oh-{}", std::process::id()));
			let _ = std::fs::remove_dir_all(&dir);
			let (logs, devcfg) = (dir.join("logs"), dir.join("survive").join("hidtap").join("devcfg"));
			std::fs::create_dir_all(&logs).unwrap();
			std::fs::create_dir_all(&devcfg).unwrap();
			// a dongle's cache says it carries LHR-AAAA0001, and a recording of sally's that LHR-BBBB0002 is on another
			std::fs::write(devcfg.join("1111111111-1YX_config.json"), "{\"device_serial_number\": \"lhr-aaaa0001\"}")
				.unwrap();
			std::fs::write(devcfg.join("LHR-AAAA0001_config.json"), "{\"a\": 1, \"b\": [2]}").unwrap();
			std::fs::write(
				logs.join("sallytap_20261101_000000.hidtap"),
				"# cache 2222222222-2YX LHR-BBBB0002\r\nR 1 x\r\n",
			)
			.unwrap();
			let data = lhraw(
				&[
					(2, RF_PID, "LHR-AAAA0001", "{\"a\": 1.0, \"b\": [2], \"sensor_type\": \"x\"}"),
					(3, RF_PID, "LHR-BBBB0002", "{\"a\": 2}"),
					(4, RF_PID, "LHR-CCCC0003", "{\"a\": 2"),
					(0, 0x2300, "LHR-HHHH0000", "{}"),
				],
				&[(10.5, 2, b"\x23\xab"), (10.0, 3, b"\x24"), (10.0, 0, b"\x01"), (11.0, 9, b"\x02")],
			);
			let src = Source { stamp: "20261108_190003".into(), name: "r.lhraw".into(), data };
			let at = Places { project: dir.clone(), logs: logs.clone(), devcfg };
			let said = convert(&src, &at, false, "2026-11-09 12:00").unwrap();
			let sep = std::path::MAIN_SEPARATOR;
			assert_eq!(
				said,
				format!(
					"wrote sallytap_20261108_190003.hidtap: 3 reports over 1 s from 4 devices\n    LHR-CCCC0003: no dongle \
					 known for it, keyed by its own serial\n    LHR-BBBB0002: devcfg has no config for it; the recording's \
					 written to logs{sep}steamcfg_20261108_190003_openhouse\n    LHR-CCCC0003: its config does not read \
					 (Expecting ',' delimiter: line 1 column 8 (char 7)): none written"
				)
			);
			let tap = std::fs::read_to_string(logs.join("sallytap_20261108_190003.hidtap")).unwrap();
			assert_eq!(
				tap,
				"# openhouse recording r.lhraw, converted by sally convert on 2026-11-09 12:00\r\n\
				 # endpoint 0: 28de:2300 interface 0 kind 0 LHR-HHHH0000 -> LHR-HHHH0000\r\n\
				 # endpoint 2: 28de:2102 interface 0 kind 2 LHR-AAAA0001 -> 1111111111-1YX\r\n\
				 # endpoint 3: 28de:2102 interface 0 kind 2 LHR-BBBB0002 -> 2222222222-2YX\r\n\
				 # endpoint 4: 28de:2102 interface 0 kind 2 LHR-CCCC0003 -> LHR-CCCC0003\r\n\
				 # cache 1111111111-1YX LHR-AAAA0001\r\n\
				 # cache 2222222222-2YX LHR-BBBB0002\r\n\
				 A 10000000 1111111111-1YX LHR-AAAA0001\r\n\
				 A 10000000 2222222222-2YX LHR-BBBB0002\r\n\
				 R 10000000 2222222222-2YX 2102 0 -1 I 24\r\n\
				 R 10000000 LHR-HHHH0000 2300 0 -1 I 01\r\n\
				 R 10500000 1111111111-1YX 2102 0 -1 I 23ab\r\n"
			);
			assert_eq!(
				std::fs::read(logs.join("steamcfg_20261108_190003_openhouse").join("LHR-BBBB0002_config.json"))
					.unwrap(),
				b"{\"a\": 2}"
			);
			// once only, and never over sally's own
			assert_eq!(
				convert(&src, &at, false, "x").unwrap(),
				"already converted: sallytap_20261108_190003.hidtap (--force writes it again)"
			);
			assert!(convert(&src, &at, true, "x").unwrap().starts_with("wrote"));
			let mine = Source { stamp: "20261101_000000".into(), ..src };
			assert_eq!(
				convert(&mine, &at, true, "x").unwrap(),
				"not written: sallytap_20261101_000000.hidtap is a recording sally made"
			);
			let _ = std::fs::remove_dir_all(&dir);
		}
	}
}

#[cfg(test)]
mod tests {
	use super::pcap::*;
	use super::*;

	#[test]
	fn the_listing_names_devices_as_the_python_did() {
		let devs =
			read_usbdevs("10462 8450 3 8 KN\n10462 8450 3 9 KN\nshort line\n10462 8960 3 2 HMD\r10462 8450 3 8 TR\n")
				.unwrap();
		let named: Vec<_> = devs.iter().map(|(k, d)| (*k, d.pid, d.serial.as_str())).collect();
		assert_eq!(named, [((3, 8), 8450, "TR0"), ((3, 9), 8450, "KN1"), ((3, 2), 8960, "HMD")]);
		assert_eq!(read_usbdevs("1 2 x 4 KN").unwrap_err(), "invalid literal for int() with base 10: 'x'");
		assert_eq!(hex4(-5), "-005");
		assert_eq!(hex4(0x2102), "2102");
		assert_eq!(hex4(0x12345), "12345");
	}

	/// A USBPcap record: its header, then `rest` (the stage byte and the data)
	#[allow(clippy::too_many_arguments)]
	fn record(irp: u64, status: u32, dev: u16, ep: u8, transfer: u8, hlen: u16, rest: &[u8], dlen: u32) -> Vec<u8> {
		let mut p = hlen.to_le_bytes().to_vec();
		p.extend(irp.to_le_bytes());
		p.extend(status.to_le_bytes());
		p.extend([0, 0, 0]);
		p.extend(3u16.to_le_bytes());
		p.extend(dev.to_le_bytes());
		p.extend([ep, transfer]);
		p.extend(dlen.to_le_bytes());
		p.extend(rest);
		p
	}

	fn pcap_of(records: &[(u32, u32, Vec<u8>)]) -> Vec<u8> {
		let mut f = 0xa1b2_c3d4u32.to_le_bytes().to_vec();
		f.extend([2, 0, 4, 0]);
		f.extend([0; 12]);
		f.extend(249u32.to_le_bytes());
		for (sec, usec, r) in records {
			f.extend(sec.to_le_bytes());
			f.extend(usec.to_le_bytes());
			f.extend((r.len() as u32).to_le_bytes());
			f.extend((r.len() as u32).to_le_bytes());
			f.extend(r);
		}
		f
	}

	#[test]
	fn a_capture_converts() {
		let devs = read_usbdevs("10462 8450 3 8 KN\n").unwrap();
		let setup_in = [0xa1, 0x01, 0x10, 0x03, 0, 0, 64, 0];
		let mut set_feature = vec![0x21, 0x09, 0x83, 0x03, 2, 0, 3, 0];
		set_feature.extend([0x83, 1, 2]);
		let mut f_data = vec![STAGE_COMPLETE_BYTE];
		f_data.extend([0x10, 0xaa]);
		let mut setup = vec![0u8];
		setup.extend(setup_in);
		let mut set = vec![0u8];
		set.extend(&set_feature);
		let recs = [
			(1, 5, record(1, 0, 8, 0x81, 1, 27, &[0x23, 0x01], 2)),
			(1, 6, record(2, 0, 9, 0x81, 1, 27, &[0x23], 1)),
			(1, 7, record(3, 0, 8, 0x00, 2, 28, &setup, 8)),
			(1, 8, record(3, 0, 8, 0x80, 2, 28, &f_data, 2)),
			(1, 9, record(4, 0, 8, 0x00, 2, 28, &set, 11)),
			(1, 10, record(5, 0, 8, 0x02, 3, 27, &[], 0)),
			(2, 0, vec![1, 2, 3]),
		];
		let data = pcap_of(&recs);
		let mut packets = Packets::open(&data[..], "x.pcap").unwrap();
		let mut out = TextOut::with_crlf(Vec::new(), false);
		let stats = convert(&mut packets, &devs, &mut out).unwrap();
		assert_eq!(
			String::from_utf8(out.into_inner().unwrap()).unwrap(),
			"D 1000005 KN0 2102\nR 1000005 KN0 2102 0 -1 I 2301\nR 1000008 KN0 2102 -1 -1 F 10aa\n\
			 R 1000009 KN0 2102 2 -1 W 830102\n"
		);
		assert_eq!(stats, Stats { i: 1, f: 1, w: 1, o: 0, skipped: 1 });
		assert!(Packets::open(&b"\xd4\xc3\xb2\xa1"[..], "x").is_err());
		let mut usbmon = pcap_of(&[]);
		usbmon[20] = 220;
		assert_eq!(
			Packets::open(&usbmon[..], "x").err().unwrap(),
			"linktype 220 is not USBPcap (249); Linux usbmon captures are not supported"
		);
	}

	const STAGE_COMPLETE_BYTE: u8 = 3;

	#[test]
	fn names_match_as_glob_does() {
		let m = |p: &str, n: &str| fnmatch(&p.chars().collect::<Vec<_>>(), &n.chars().collect::<Vec<_>>());
		assert!(m("*.lhraw", "recording-1.lhraw"));
		assert!(m("rec?.zip", "rec1.zip"));
		assert!(m("rec[0-9].zip", "rec7.zip"));
		assert!(!m("rec[!0-9].zip", "rec7.zip"));
		assert!(!m("*.zip", "a.lhraw"));
		assert_eq!(basename("a/b/c.lhraw"), "c.lhraw");
	}
}
