//! What a recording leaves out: the bytes of a radio dongle's report that follow its packets.
//!
//! A dongle hands on a 59-byte report of one packet (0x23) or two (0x24), each in a 29-byte slot. A packet says its own
//! length; after it the dongle puts four bytes of its own, and the rest of the slot is what earlier packets left
//! there. The tap gives the parser a 0x23's first slot alone (`tap::true_len`), and the parser reads a packet up to the
//! length it gives. So those bytes are read by nothing, and they are a third of what a packed recording holds
//! (`pack`: the dongle's four follow from nothing). A recording is written without them: zeros stand in their place.
//!
//! "Read by nothing" holds for a packet laid out as its flags say. One that is not (a packet with no payload, one cut
//! short, a kind of the older layout) the parser reads past its end, into those bytes, and what it then reports
//! depends on them. So a packet is cut only where every reading of it stays inside it, whatever state the device's
//! parser is in (`kept`), and a report is left whole from its first packet that is not so. A replay of the recording
//! then reads what the session read, to the bit; `the_parser_reads_nothing_that_is_left_out` holds the parser to it.
//!
//! The runtime has the parser stop every such reading at the packet's own end (`report::Globals::past_end`), so there
//! those bytes are read by nothing at all. The rule here stays as it is: a recording is also read by a parser that
//! reads on (the option's other settings, a build from before it).

use crate::devices::{interface_from_hid, usb_if};
use crate::{line, tap};

const ONE_PACKET: u8 = 0x23;
const TWO_PACKETS: u8 = 0x24;
/// A packet's slot in a report, and a report of two
const SLOT: usize = 29;
const TWO_SLOTS: usize = 1 + 2 * SLOT;

/// How many bytes of its slot a packet is, when no reading of it goes further: the parser's (`report::Parser`'s
/// handle_watchman, by both of its layouts and both light modes) and, for a report's first packet, the tap's look at
/// the light mode (`Tap::sniff_mode`: the flags and the byte after them). `slot` starts at the packet and ends where
/// the parser's buffer has the next thing. None: some reading of this packet goes past it.
///
/// It follows the parser's own steps, and stops where the parser does:
/// [time high] [size] [time low] [flags] [IMU: 13] [status: a marker, then 1 or 2] [input] [light].
fn kept(slot: &[u8], first: bool) -> Option<usize> {
	reading(slot, first, true).ok()
}

/// Why a reading of a packet goes past its end
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Past {
	/// no payload: the flags are the byte after the packet
	NoPayload,
	/// a length that ends outside the slot
	Size,
	/// the older layout's events
	Older,
	/// an IMU sample with 7 to 12 bytes left
	Imu,
	/// a status marker, or a battery's byte, after the end
	Status,
	/// a second-generation input's kind after the end
	Input,
	/// light that starts after the end
	Light,
	/// a light entry with fewer than four bytes left
	Entry,
	/// the tap's look at the byte after a first packet's flags
	Sniff,
}

/// The packet's end where the parser's reading by the newer layout stays inside it, or why it does not: the
/// question `kept` asks, of that one reading alone (the parser's own count of such packets, `report::Globals`)
pub fn past_end(slot: &[u8]) -> Option<Past> {
	reading(slot, false, false).err()
}

/// `kept`, with the reason; `older`: the older layout's reading counts too
fn reading(slot: &[u8], first: bool, older: bool) -> Result<usize, Past> {
	let size = *slot.get(1).ok_or(Past::Size)?;
	// the size counts itself; the flags are read whether or not there is a payload
	let end = 3 + size.wrapping_sub(1) as usize;
	if end < 4 {
		return Err(Past::NoPayload);
	}
	if end > slot.len() {
		return Err(Past::Size);
	}
	let flags = slot[3];
	// a heartbeat: nothing more is read
	if flags == 0xe2 {
		return Ok(end);
	}
	// the older layout's events, read without a look at the length
	if older && flags & 0xe0 == 0xe0 {
		return Err(Past::Older);
	}
	let mut at = 4;
	if flags & 0x80 != 0 && end - at >= 7 {
		// an IMU sample: looked for with 7 bytes left, read as 13
		if end - at < 13 {
			return Err(Past::Imu);
		}
		at += 13;
	}
	if flags & 0x40 != 0 {
		// a status: its marker is read unchecked, and a battery's byte after it
		let marker = *slot.get(at).filter(|_| at < end).ok_or(Past::Status)?;
		at += 1;
		match marker {
			0x80 if at < end => at += 1,
			0x80 => return Err(Past::Status),
			0x20 => at += 1,
			0x30 => at += 2,
			// any other marker ends the packet's reading
			_ => return Ok(end),
		}
	}
	if flags & 0x20 != 0 {
		// the input: each part is checked against the length before it is read, and a short one ends the reading; but
		// a second-generation input's kind is read unchecked
		if at >= end {
			return Ok(end);
		}
		let input = slot[at];
		at += 1;
		if input & 7 != 0 {
			for (bit, bytes) in [(1, 1), (4, 1), (2, 4)] {
				if input & bit != 0 {
					if end - at < bytes {
						return Ok(end);
					}
					at += bytes;
				}
			}
		} else {
			if at >= end {
				return Err(Past::Input);
			}
			let kind = slot[at];
			at += 1;
			if kind != 0xa1 || end - at < 7 {
				return Ok(end);
			}
			at += 7;
		}
	}
	if flags & 0x10 != 0 {
		// the light's length is what is left, as a byte: past the end it would be 200 and more
		if at > end {
			return Err(Past::Light);
		}
		// the tap's look at a first packet of light alone: the byte after the flags
		if first && flags == 0x10 && at == end {
			return Err(Past::Sniff);
		}
		// raw1: a channel byte, or an entry of four bytes read whole (an entry before any channel ends the reading, and
		// so does a channel byte of a kind not known). Raw0's light is read from the end backwards and stays inside
		let mut channel = false;
		while at < end {
			if slot[at] & 1 != 0 {
				if slot[at] & 0x0a != 0 {
					break;
				}
				channel = true;
				at += 1;
			} else {
				if end - at < 4 {
					return Err(Past::Entry);
				}
				if !channel {
					break;
				}
				at += 4;
			}
		}
	}
	Ok(end)
}

/// The bytes of an input report that nothing reads set to zero, for a device with this product id and these HID path
/// numbers (a recording line's): a radio dongle's report after its packets. Any other report is left as it is
pub fn report(pid: u16, mi: i32, col: i32, data: &mut [u8]) {
	let iface = interface_from_hid(pid, mi, col);
	if iface != usb_if::WATCHMAN1 && iface != usb_if::WATCHMAN2 {
		return;
	}
	match data.first() {
		Some(&ONE_PACKET) => {
			// the parser is handed this much of it: past that nothing is read at all
			let seen = tap::true_len(pid, data);
			data[seen..].fill(0);
			if let Some(packet) = kept(&data[1..seen], true) {
				data[1 + packet..seen].fill(0);
			}
		}
		Some(&TWO_PACKETS) if data.len() >= TWO_SLOTS => {
			// a first packet read past its end is read into the second one's slot
			let Some(packet) = kept(&data[1..1 + SLOT], true) else { return };
			data[1 + packet..1 + SLOT].fill(0);
			if let Some(packet) = kept(&data[1 + SLOT..TWO_SLOTS], false) {
				data[1 + SLOT + packet..TWO_SLOTS].fill(0);
			}
		}
		_ => {}
	}
}

/// The line the live tap writes for an input report (`line::input_report`), without what `report` leaves out
pub fn input_report(time_us: u64, serial: &str, pid: u16, mi: i32, col: i32, data: &[u8]) -> Vec<u8> {
	let mut kept = data.to_vec();
	report(pid, mi, col, &mut kept);
	line::input_report(time_us, serial, pid, mi, col, &kept)
}

/// A recording's line with `report` done to the report it carries, where it stands: the hex of each byte left out
/// becomes "00" and nothing else of the line changes. Whether anything did. A line that is not an input report's, as
/// `line::parse` takes it, is left as it is
pub fn line(text: &mut [u8]) -> bool {
	let Some((pid, mi, col, b'I', hex)) = line::report_at(text) else { return false };
	let Some(mut data) = line::decode_hex(&text[hex..]) else { return false };
	let whole = data.clone();
	report(pid, mi, col, &mut data);
	let mut changed = false;
	for (i, _) in data.iter().zip(&whole).enumerate().filter(|(_, (now, was))| now != was) {
		text[hex + 2 * i..hex + 2 * i + 2].copy_from_slice(b"00");
		changed = true;
	}
	changed
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::report::{
		report_buffer, subtype, ButtonEntry, Globals, Host, LightcapElement, LightcapMode, Parser, BUFFER_LEN,
	};

	const DONGLE: u16 = 0x2102;

	/// A host that writes down everything the parser tells it
	struct Seen {
		lh_version: i32,
		subtype: i32,
		last_imu: u64,
		said: Vec<String>,
	}

	impl Host for Seen {
		fn lh_version(&self) -> i32 {
			self.lh_version
		}
		fn last_imu(&self) -> u64 {
			self.last_imu
		}
		fn object_subtype(&self) -> i32 {
			self.subtype
		}
		fn has_config(&self) -> bool {
			true
		}
		fn map_sensor_id(&self, reported: u8) -> u8 {
			reported
		}
		fn raw_imu(&mut self, mask: i32, agm: &[f64; 9], timecode: u32, id: i32) {
			self.last_imu = timecode as u64;
			self.said.push(format!("imu {mask} {agm:?} {timecode} {id}"));
		}
		fn register_runtime(&mut self, time_received_us: u64) {
			self.said.push(format!("runtime {time_received_us}"));
		}
		fn sync(&mut self, channel: u8, timecode: u32, ootx: bool, gen: bool) {
			self.said.push(format!("sync {channel} {timecode} {ootx} {gen}"));
		}
		fn sweep(&mut self, channel: u8, sensor_id: u8, timecode: u32, half_clock: bool) {
			self.said.push(format!("sweep {channel} {sensor_id} {timecode} {half_clock}"));
		}
		fn lightcap(&mut self, le: &LightcapElement) -> bool {
			self.said.push(format!("lightcap {le:?}"));
			true
		}
		fn button(&mut self, entry: &ButtonEntry) {
			self.said.push(format!("button {entry:?}"));
		}
		fn notify_gen2(&mut self, reason: &str) {
			self.said.push(format!("gen2 {reason}"));
		}
		fn warn(&mut self, text: &str) {
			self.said.push(format!("warn {text}"));
		}
		fn info(&mut self, text: &str) {
			self.said.push(format!("info {text}"));
		}
		fn print(&mut self, text: &str) {
			self.said.push(format!("print {text}"));
		}
		fn run_time(&self) -> f64 {
			0.0
		}
		fn colorize(&self, s: &str) -> String {
			s.to_string()
		}
		fn set_imu_scale_modes(&mut self, gyro_scale_mode: i32, acc_scale_mode: i32) {
			self.said.push(format!("scales {gyro_scale_mode} {acc_scale_mode}"));
		}
		fn battery(&mut self, charge: i8, charging: bool) {
			self.said.push(format!("battery {charge} {charging}"));
		}
	}

	struct Rng(u64);

	impl Rng {
		fn next(&mut self) -> u64 {
			self.0 ^= self.0 << 13;
			self.0 ^= self.0 >> 7;
			self.0 ^= self.0 << 17;
			self.0
		}
		fn below(&mut self, n: u64) -> u64 {
			self.next() % n
		}
		fn byte(&mut self) -> u8 {
			self.next() as u8
		}
		fn one_in(&mut self, n: u64) -> bool {
			self.below(n) == 0
		}
		fn of(&mut self, from: &[u8]) -> u8 {
			from[self.below(from.len() as u64) as usize]
		}
	}

	/// A packet as devices send them, or nearly: laid out by its flags, with now and then a part cut short, a size
	/// that says another length than the payload has, or bytes of no kind at all. The slot's rest is the dongle's
	fn slot(rng: &mut Rng) -> [u8; SLOT] {
		let mut s = [0u8; SLOT];
		for b in &mut s {
			*b = rng.byte();
		}
		let flags = match rng.below(12) {
			0..=3 => 0x10,
			4..=5 => 0x90,
			6..=7 => 0x80,
			8 => rng.of(&[0x38, 0x20, 0xa0, 0x34, 0xb4, 0x31, 0x35, 0xb5, 0x30, 0xb0]),
			9 => rng.of(&[0x51, 0xd1, 0xc0, 0x40, 0x70, 0x50, 0xd0, 0xf0, 0x60]),
			10 => rng.of(&[0xe2, 0xe8, 0xf7, 0xe0, 0xf1, 0xe1, 0xe9, 0xf8]),
			_ => rng.byte(),
		};
		let mut p = vec![flags];
		if flags & 0x80 != 0 {
			let n = if rng.one_in(12) { rng.below(14) } else { 13 };
			p.extend((0..n).map(|_| rng.byte()));
		}
		if flags & 0x40 != 0 {
			let marker = if rng.one_in(5) { rng.byte() } else { rng.of(&[0x80, 0x20, 0x30]) };
			p.push(marker);
			let n = if rng.one_in(6) { rng.below(4) } else { [1, 1, 2][(marker as usize >> 4) % 3] };
			p.extend((0..n).map(|_| rng.byte()));
		}
		if flags & 0x20 != 0 {
			let input = if rng.one_in(3) { rng.byte() & 0xf8 } else { rng.byte() };
			p.push(input);
			let n = match input & 7 {
				0 => {
					p.push(if rng.one_in(4) { rng.byte() } else { 0xa1 });
					7
				}
				bits => (bits & 1) as u64 + (bits >> 2 & 1) as u64 + 4 * (bits >> 1 & 1) as u64,
			};
			let n = if rng.one_in(6) { rng.below(9) } else { n };
			p.extend((0..n).map(|_| rng.byte()));
		}
		if flags & 0x10 != 0 || rng.one_in(8) {
			// raw1's light: a channel, then entries; sometimes not quite that
			let mut channel = rng.one_in(10);
			for _ in 0..rng.below(8) {
				if !channel || rng.one_in(6) {
					p.push((rng.byte() & 0xf0) | if rng.one_in(12) { rng.byte() | 1 } else { 1 });
					channel = true;
				} else {
					let entry = (rng.next() as u32 & !1) | if rng.one_in(30) { 1 } else { 0 };
					p.extend(entry.to_le_bytes());
				}
			}
			if rng.one_in(10) {
				p.extend((0..rng.below(4)).map(|_| rng.byte() & 0xfe));
			}
		}
		p.truncate(SLOT - 3);
		s[3..3 + p.len()].copy_from_slice(&p);
		s[1] = match rng.below(16) {
			0 => rng.byte(),
			1 => rng.below(4) as u8,
			2 => (p.len() as u8 + 1).wrapping_add(rng.below(5) as u8).wrapping_sub(2),
			_ => p.len() as u8 + 1,
		};
		s
	}

	fn radio_report(rng: &mut Rng) -> Vec<u8> {
		let mut data = vec![if rng.one_in(4) { TWO_PACKETS } else { ONE_PACKET }];
		data.extend(slot(rng));
		data.extend(slot(rng));
		data
	}

	/// What the tap does with an input report: its look at the light mode, then the parser on the report's true length
	fn read(parser: &mut Parser, host: &mut Seen, g: &mut Globals, pid: u16, time: u64, data: &[u8]) {
		if data.len() >= 6 && (data[0] == ONE_PACKET || data[0] == TWO_PACKETS) && data[4] == 0x10 {
			host.said.push(format!("mode sniffed {}", data[5]));
		}
		let (buffer, len): ([u8; BUFFER_LEN], usize) = report_buffer(&data[..tap::true_len(pid, data)]);
		parser.report(host, g, time, interface_from_hid(pid, -1, -1), &buffer, len);
	}

	/// The guard of the whole module. Reports of every shape go through three parsers side by side, in every state a
	/// device's parser can be in: as they came, with what `report` leaves out as zeros, and with other bytes in those
	/// places. The three are told the same things and end in the same state: so nothing left out was read
	#[test]
	fn the_parser_reads_nothing_that_is_left_out() {
		let modes = [LightcapMode::Raw1, LightcapMode::Raw0, LightcapMode::Unknown, LightcapMode::Raw2];
		let subtypes =
			[subtype::TRACKER, subtype::WAND, subtype::KNUCKLES_R, subtype::TRACKER_GEN2, subtype::INDEX_HMD];
		let mut rng = Rng(0x5a11_7ac4_0001);
		let (mut cut, mut whole, mut reports) = (0u64, 0u64, 0u64);
		for (lh_version, past_end) in [(1, 0), (-1, 0), (0, 0), (1, 2), (-1, 2)] {
			for subtype in subtypes {
				for mode in modes {
					let host = || Seen { lh_version, subtype, last_imu: 0, said: Vec::new() };
					let mut hosts = [host(), host(), host()];
					let mut parser = Parser::new("KN0");
					parser.configured = true;
					parser.lightcap_mode = mode;
					let mut parsers = [parser.clone(), parser.clone(), parser];
					let globals = || Globals { past_end, ..Globals::default() };
					let mut globals = [globals(), globals(), globals()];
					for n in 0..4000u64 {
						// the states a session moves a parser through, and some it only could
						if rng.one_in(400) {
							let mode = modes[rng.below(4) as usize];
							let (wait, seen, ignore) = (rng.below(24) as usize, rng.below(220) as usize, rng.below(3));
							for p in &mut parsers {
								p.lightcap_mode = mode;
								p.time_without_flag = wait;
								p.packets_seen_waiting_for_v2 = seen;
								p.ignore_cnt = ignore as usize;
							}
						}
						let came = radio_report(&mut rng);
						let mut zeros = came.clone();
						report(DONGLE, -1, -1, &mut zeros);
						let other: Vec<u8> =
							came.iter().zip(&zeros).map(|(&c, &z)| if c == z { c } else { rng.byte() }).collect();
						reports += 1;
						match zeros == came {
							true => whole += 1,
							false => cut += 1,
						}
						for (k, data) in [&came, &zeros, &other].into_iter().enumerate() {
							read(&mut parsers[k], &mut hosts[k], &mut globals[k], DONGLE, 1000 * n, data);
						}
						for k in 1..3 {
							assert_eq!(
								hosts[k].said,
								hosts[0].said,
								"lighthouse {lh_version}, subtype {subtype}, {mode:?}, report {n}: {came:02x?} as {:02x?}",
								[&zeros, &other][k - 1]
							);
							assert_eq!(format!("{:?}", parsers[k]), format!("{:?}", parsers[0]), "{came:02x?}");
							assert_eq!(globals[k].raw1_unknown_count, globals[0].raw1_unknown_count);
						}
						for h in &mut hosts {
							h.said.clear();
						}
					}
				}
			}
		}
		// most reports lose something, and the test's reports hold plenty that may not
		assert!(cut * 2 > reports && whole * 20 > reports, "{cut} cut, {whole} whole of {reports}");
	}

	/// What the runtime has the parser do (`Globals::past_end` 2): a reading stops at the packet's own end. Then no
	/// byte after a packet is read, whatever the packet: other bytes there change nothing the parser says or keeps
	#[test]
	fn with_the_readings_stopped_nothing_after_a_packet_is_read() {
		let modes = [LightcapMode::Raw1, LightcapMode::Raw0];
		let mut rng = Rng(0x0dd_bead);
		let (mut odd, mut reports) = (0u64, 0u64);
		for lh_version in [1, -1] {
			for subtype in [subtype::TRACKER_GEN2, subtype::KNUCKLES_R, subtype::INDEX_HMD] {
				for mode in modes {
					let host = || Seen { lh_version, subtype, last_imu: 0, said: Vec::new() };
					let mut hosts = [host(), host()];
					let mut parser = Parser::new("KN0");
					parser.configured = true;
					parser.lightcap_mode = mode;
					let mut parsers = [parser.clone(), parser];
					let stopped = || Globals { past_end: 2, ..Globals::default() };
					let mut globals = [stopped(), stopped()];
					for n in 0..6000u64 {
						let came = radio_report(&mut rng);
						// other bytes after each packet's end, where the packet ends inside its slot (a first packet
						// that says it is longer takes the second one's bytes for its own)
						let mut other = came.clone();
						let end = |slot: &[u8]| 3 + slot[1].wrapping_sub(1) as usize;
						let first = end(&came[1..1 + SLOT]);
						if first <= SLOT {
							for b in &mut other[1 + first..1 + SLOT] {
								*b = rng.byte();
							}
							let second = end(&came[1 + SLOT..]);
							if came[0] == TWO_PACKETS && second <= SLOT {
								for b in &mut other[1 + SLOT + second..] {
									*b = rng.byte();
								}
							}
						}
						// (the tap's own look at the light mode reads a first packet's flags and the byte after them,
						// where they are: not the parser's, and left as they came)
						other[4..6].copy_from_slice(&came[4..6]);
						reports += 1;
						odd += past_end(&came[1..1 + SLOT]).is_some() as u64;
						for (k, data) in [&came, &other].into_iter().enumerate() {
							read(&mut parsers[k], &mut hosts[k], &mut globals[k], DONGLE, 1000 * n, data);
						}
						assert_eq!(hosts[1].said, hosts[0].said, "{lh_version} {subtype} {mode:?} {n}: {came:02x?}");
						assert_eq!(format!("{:?}", parsers[1]), format!("{:?}", parsers[0]), "{came:02x?}");
						assert_eq!(globals[1].past, globals[0].past);
						for h in &mut hosts {
							h.said.clear();
						}
					}
					// the packets a reading would go past are counted where the newer layout's reading is the one used
					let counted = globals[0].past.iter().sum::<u32>();
					assert!(lh_version != 1 || mode != LightcapMode::Raw1 || counted > 0);
				}
			}
		}
		assert!(odd * 20 > reports, "{odd} of {reports}");
	}

	/// Why a packet is not simply cut at the length it gives: one with no payload has its flags read from the dongle's
	/// own bytes, and what the parser then says depends on them
	#[test]
	fn a_packet_read_past_its_end_is_left_whole() {
		// [time] [size 1] [time], then the dongle's four bytes: 0xb3 read as flags asks for an input that is not there
		let mut came = vec![ONE_PACKET, 0xd4, 0x01, 0x25, 0xb3, 0x46, 0x09, 0x5e];
		came.resize(TWO_SLOTS, 0x77);
		assert_eq!(kept(&came[1..1 + SLOT], true), None);
		let mut left = came.clone();
		report(DONGLE, -1, -1, &mut left);
		// (the second slot of a one-packet report is never handed to the parser: that goes)
		assert_eq!(left[..1 + SLOT], came[..1 + SLOT]);
		assert!(left[1 + SLOT..].iter().all(|&b| b == 0));

		let said = |data: &[u8]| {
			let mut host = Seen { lh_version: 1, subtype: subtype::TRACKER_GEN2, last_imu: 0, said: Vec::new() };
			let mut parser = Parser::new("KN0");
			parser.configured = true;
			parser.lightcap_mode = LightcapMode::Raw1;
			read(&mut parser, &mut host, &mut Globals::default(), DONGLE, 0, data);
			host.said
		};
		// the parser's own measurement: counted, and with the reading stopped at the packet's end nothing is said
		let stopped = |past_end: u8| {
			let mut host = Seen { lh_version: 1, subtype: subtype::TRACKER_GEN2, last_imu: 0, said: Vec::new() };
			let mut parser = Parser::new("KN0");
			parser.configured = true;
			parser.lightcap_mode = LightcapMode::Raw1;
			let mut g = Globals { past_end, ..Globals::default() };
			read(&mut parser, &mut host, &mut g, DONGLE, 0, &came);
			(host.said, g.past[Past::NoPayload as usize])
		};
		assert_eq!(past_end(&came[1..1 + SLOT]), Some(Past::NoPayload));
		assert_eq!(stopped(0), (said(&came), 0));
		assert_eq!(stopped(1), (said(&came), 1));
		assert_eq!(stopped(2), (Vec::new(), 1));
		let mut cut_anyway = came.clone();
		cut_anyway[4..1 + SLOT].fill(0);
		assert!(said(&came).iter().any(|s| s.contains("handle_input needed")), "{:?}", said(&came));
		assert_ne!(said(&cut_anyway), said(&came));
		assert_eq!(said(&left), said(&came));
	}

	#[test]
	fn a_packet_is_cut_where_it_ends() {
		// light alone: a channel byte and two entries, 10 bytes of payload; then the dongle's bytes and leftovers
		let mut one = vec![ONE_PACKET, 0x12, 0x0b, 0x34, 0x10, 0x31, 2, 0, 0, 8, 6, 0, 0, 16];
		one.resize(TWO_SLOTS, 0xee);
		let mut cut = one.clone();
		report(DONGLE, -1, -1, &mut cut);
		assert_eq!(cut[..14], one[..14]);
		assert!(cut[14..].iter().all(|&b| b == 0));
		// done twice it is the same, and an older dongle's is cut the same way
		let again = cut.clone();
		report(DONGLE, -1, -1, &mut cut);
		assert_eq!(cut, again);
		let mut older = one.clone();
		report(0x2101, 0, 1, &mut older);
		assert_eq!(older, cut);

		// two packets: an IMU sample and light, then a heartbeat
		let mut two = vec![TWO_PACKETS, 0x12, 0x14, 0x34, 0x90];
		two.extend([7u8; 13]);
		two.extend([0x71, 2, 0, 0, 8]);
		two.resize(1 + SLOT, 0xee);
		two.extend([0x12, 0x02, 0x35, 0xe2]);
		two.resize(TWO_SLOTS, 0xdd);
		let mut cut = two.clone();
		report(DONGLE, -1, -1, &mut cut);
		assert_eq!((&cut[..23], &cut[30..34]), (&two[..23], &two[30..34]));
		assert!(cut[23..30].iter().chain(&cut[34..]).all(|&b| b == 0));
		// a second packet that cannot be cut keeps its slot, the first one's is cut all the same
		let mut odd = two.clone();
		odd[31] = 1;
		let mut cut = odd.clone();
		report(DONGLE, -1, -1, &mut cut);
		assert_eq!((&cut[..23], &cut[30..]), (&odd[..23], &odd[30..]));
		assert!(cut[23..30].iter().all(|&b| b == 0));
		// a first packet that cannot be cut keeps the whole report
		let mut odd = two.clone();
		odd[2] = 1;
		let mut cut = odd.clone();
		report(DONGLE, -1, -1, &mut cut);
		assert_eq!(cut, odd);

		// not a dongle's, not a radio report, not long enough for two: as they are
		for (pid, data) in
			[(0x2300u16, one.clone()), (DONGLE, [&[0x21u8][..], &one[1..]].concat()), (DONGLE, two[..40].to_vec())]
		{
			let mut same = data.clone();
			report(pid, -1, -1, &mut same);
			assert_eq!(same, data);
		}
		let mut empty = Vec::new();
		report(DONGLE, -1, -1, &mut empty);
	}

	/// A line is changed where its report's bytes stand, as the live tap would have written it
	#[test]
	fn a_line_loses_what_its_report_does() {
		let mut rng = Rng(77);
		let (mut changed, mut lines) = (0, 0);
		for _ in 0..3000 {
			let data = radio_report(&mut rng);
			let mut cut = data.clone();
			report(DONGLE, -1, -1, &mut cut);
			for end in ["\n", "\r\n", ""] {
				let mut text = line::input_report(123_456_789, "AAAA000001-1YX", DONGLE, -1, -1, &data);
				text.truncate(text.len() - 1);
				text.extend(end.bytes());
				// (the line the live tap writes with the option on)
				let mut want = input_report(123_456_789, "AAAA000001-1YX", DONGLE, -1, -1, &data);
				assert_eq!(want, line::input_report(123_456_789, "AAAA000001-1YX", DONGLE, -1, -1, &cut));
				want.truncate(want.len() - 1);
				want.extend(end.bytes());
				assert_eq!(line(&mut text), cut != data);
				assert_eq!(text, want);
				changed += (cut != data) as usize;
				lines += 1;
			}
		}
		assert!(changed * 2 > lines);

		// hex in capitals stays so where it is kept; what follows the hex stays
		let mut text = b"R 5 D 2102 -1 -1 I 2312033410AB\r\n".to_vec();
		text.splice(31..31, "CD".repeat(53).bytes());
		let mut want = text.clone();
		want[31..31 + 106].copy_from_slice("00".repeat(53).as_bytes());
		assert!(line(&mut text));
		assert_eq!(String::from_utf8_lossy(&text), String::from_utf8_lossy(&want));

		// other lines are not touched: a feature report, another device's, a comment, a line cut short, bad hex
		let report = "ee".repeat(TWO_SLOTS - 6);
		for other in [
			format!("R 5 D 2102 -1 -1 F 2312023410ab{report}\n"),
			format!("R 5 D 2300 -1 -1 I 2312023410ab{report}\n"),
			format!("# R 5 D 2102 -1 -1 I 2312023410ab{report}\n"),
			format!("A 5 D LHR-00000001 I 2312023410ab{report}\n"),
			"R 5 D 2102 -1\n".to_string(),
			format!("R 5 D 2102 -1 -1 I 2312023410ab{report}x\n"),
			String::new(),
		] {
			let mut text = other.clone().into_bytes();
			assert!(!line(&mut text), "{other}");
			assert_eq!(text, other.as_bytes());
		}
	}
}
