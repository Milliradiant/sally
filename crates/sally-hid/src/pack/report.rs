//! The reports of a packed recording. A report is coded against the last one of its kind from the same device, and
//! what the kinds seen so far are known to hold is coded as what it is:
//!
//! - a radio dongle's report (0x23 or 0x24, 59 bytes): one or two 29-byte packet slots. A packet overwrites the
//!   front of its slot and leaves the rest as the packet before left it, so only the packet is coded: what it
//!   holds, its size, its time, its IMU sample and its light as `imu` and `light` code them, then the four bytes the
//!   dongle puts after it. A recording written without what follows the packets (`crate::strip`) has zeros there,
//!   which the format's second version says in a bit;
//! - a headset's IMU report (0x20, 52 bytes): three samples, of which a new report replaces the oldest;
//! - a headset's light report (0x28, 64 bytes): a length, then light;
//! - anything else: the bytes that changed since the last report of that id and length.
//!
//! None of this is the report parser's business (`crate::report` reads the reports for what they mean); here a wrong
//! idea of a report only costs bits. Every byte of every report is coded, whatever it holds.

use std::collections::HashMap;

use super::imu::{Imu, ImuModels};
use super::light::{apart, Light, LightModels};
use sally_pack::coder::{Io, Number, Prob, Symbol};

/// The longest report a line may carry (`line::MAX_REPORT`)
pub const MAX_REPORT: usize = 1024;

const RADIO_LEN: usize = 59;
const RADIO_ONE: u8 = 0x23;
const RADIO_TWO: u8 = 0x24;
/// A packet slot of a radio report
const SLOT: usize = 29;
/// A packet's payload, at most: the slot less the packet's time and size
const PAYLOAD: usize = SLOT - 3;
/// A packet's flags byte: what its payload holds, in this order
const HEARTBEAT: u8 = 0xe2;
const HAS_IMU: u8 = 0x80;
const HAS_STATUS: u8 = 0x40;
const HAS_INPUT: u8 = 0x20;
const HAS_LIGHT: u8 = 0x10;

const HEADSET_IMU_LEN: usize = 52;
const HEADSET_IMU: u8 = 0x20;
/// A sample of a headset's IMU report: six 16-bit axes, a 32-bit time, a sequence byte
const SAMPLE: usize = 17;
const HEADSET_LIGHT_LEN: usize = 64;
const HEADSET_LIGHT: u8 = 0x28;

/// The kinds of report a device's stream remembers the last of
const SHAPES: usize = 16;

/// A packet's bytes between its IMU sample and its light (battery, buttons, a heartbeat), coded against what a
/// packet with the same flags last held there
struct Plain {
	changed: [Prob; 2],
	byte: Symbol,
}

impl Plain {
	/// `n` more payload bytes from `at`, as many as there are before `end`
	fn code<I: Io>(
		&mut self, io: &mut I, kept: &mut [u8; PAYLOAD], buf: &mut [u8], at: &mut usize, n: usize, end: usize,
	) {
		let mut same = true;
		for i in *at..(*at + n).min(end) {
			let j = i - 3;
			same = !io.bit(&mut self.changed[same as usize], buf[i] != kept[j]);
			if !same {
				kept[j] = kept[j].wrapping_add(self.byte.code(io, j, buf[i].wrapping_sub(kept[j]) as u32) as u8);
			}
			buf[i] = kept[j];
			*at = i + 1;
		}
	}
}

/// The models the reports are coded under, shared by every device
pub struct Models {
	new_len: Prob,
	len: Symbol,
	other_id: Prob,
	third_id: Prob,
	id: Symbol,

	/// the format's second version: a slot may be said to hold zeros after its packet, and a one-packet report's
	/// second slot to hold nothing else
	zeros: bool,
	rest_zero: Prob,
	second_zero: Prob,
	second_slot: Prob,
	no_payload: Prob,
	flags: Symbol,
	size: Symbol,
	packet_time: Number,
	imu_time: Number,
	radio_imu: ImuModels,
	light: LightModels,
	plain: Plain,
	after: Symbol,

	sample_fresh: [Prob; 3],
	sequence: Symbol,
	sample_time: Number,
	headset_imu: ImuModels,
	light_len: Symbol,
	tail: Symbol,
}

impl Models {
	/// The models of a recording of this version of the format (`super::VERSION`, or the first)
	pub fn new(version: u8) -> Models {
		Models {
			zeros: version >= 2,
			rest_zero: Prob::new(),
			second_zero: Prob::new(),
			new_len: Prob::new(),
			len: Symbol::new(11, 1),
			other_id: Prob::new(),
			third_id: Prob::new(),
			id: Symbol::new(8, 1),
			second_slot: Prob::new(),
			no_payload: Prob::new(),
			flags: Symbol::new(8, 512),
			size: Symbol::new(8, 256),
			packet_time: Number::new(1),
			imu_time: Number::new(1),
			radio_imu: ImuModels::new(),
			light: LightModels::new(),
			plain: Plain { changed: [Prob::new(); 2], byte: Symbol::new(8, PAYLOAD) },
			after: Symbol::new(5, 6),
			sample_fresh: [Prob::new(); 3],
			sequence: Symbol::new(8, 1),
			sample_time: Number::new(1),
			headset_imu: ImuModels::new(),
			light_len: Symbol::new(8, 256),
			tail: Symbol::new(7, 1),
		}
	}
}

/// The bytes of a report that changed since the last of its kind, for a kind nothing more is known of
struct Changes {
	reach: Number,
	changed: [Prob; 2],
	byte: Symbol,
}

/// The last report of one id and length
struct Shape {
	len: usize,
	id: u8,
	last: Vec<u8>,
	changes: Option<Box<Changes>>,
}

#[derive(Default)]
struct Radio {
	/// the last packet's flags, and whether the one before it had an IMU sample: samples and light take turns
	flags: u8,
	imu_before: bool,
	/// the last packet's 16-bit time
	time: u16,
	/// the last IMU sample's 24-bit time and its distance from the one before
	imu_time: u32,
	imu_step: u32,
	/// what the payload between the IMU sample and the light last held, by flags
	plain: HashMap<u8, [u8; PAYLOAD]>,
}

#[derive(Default)]
struct Headset {
	/// the sample slot the next report is expected to replace
	next: usize,
	sequence: u8,
	time: u32,
	step: u32,
	light_len: u8,
}

/// One device's reports as the packer and the unpacker both follow them (a device is one name on the recording's
/// lines: a dongle, or one interface of a wired device)
#[derive(Default)]
pub struct Stream {
	len: usize,
	/// the last two report ids, the newest first
	ids: [u8; 2],
	shapes: Vec<Shape>,
	radio: Radio,
	headset: Headset,
	imu: Imu,
	light: Light,
}

/// One report: packing, `data` is the report; unpacking, it is filled with it
pub fn report<I: Io>(io: &mut I, m: &mut Models, s: &mut Stream, data: &mut Vec<u8>) {
	let len = match io.bit(&mut m.new_len, data.len() != s.len) {
		true => m.len.code(io, 0, data.len() as u32) as usize,
		false => s.len,
	};
	s.len = len;
	if !I::PACKING {
		data.clear();
		data.resize(len, 0);
	}
	if len == 0 {
		return;
	}

	// the id: the last one, the one before it, or any
	let id = if !io.bit(&mut m.other_id, data[0] != s.ids[0]) {
		s.ids[0]
	} else {
		let id = match io.bit(&mut m.third_id, data[0] != s.ids[1]) {
			true => m.id.code(io, 0, data[0] as u32) as u8,
			false => s.ids[1],
		};
		s.ids = [id, s.ids[0]];
		id
	};

	// the two radio reports share their slots
	let kind = if len == RADIO_LEN && id == RADIO_TWO { RADIO_ONE } else { id };
	let mut shape = match s.shapes.iter().position(|sh| sh.len == len && sh.id == kind) {
		Some(i) => s.shapes.remove(i),
		None => {
			if s.shapes.len() == SHAPES {
				// the one longest unused goes
				s.shapes.remove(0);
			}
			Shape { len, id: kind, last: vec![0; len], changes: None }
		}
	};
	if !I::PACKING {
		data.copy_from_slice(&shape.last);
	}
	data[0] = id;
	match (len, kind) {
		(RADIO_LEN, RADIO_ONE) => radio(io, m, s, data, &shape.last),
		(HEADSET_IMU_LEN, HEADSET_IMU) => headset_imu(io, m, s, data, &shape.last),
		(HEADSET_LIGHT_LEN, HEADSET_LIGHT) => headset_light(io, m, s, data, &shape.last),
		_ => {
			let c = shape.changes.get_or_insert_with(|| {
				Box::new(Changes { reach: Number::new(1), changed: [Prob::new(); 2], byte: Symbol::new(8, 64) })
			});
			changes(io, c, data, &shape.last);
		}
	}
	shape.last.copy_from_slice(data);
	s.shapes.push(shape);
}

fn radio<I: Io>(io: &mut I, m: &mut Models, s: &mut Stream, data: &mut [u8], last: &[u8]) {
	packet(io, m, s, &mut data[1..1 + SLOT], &last[1..1 + SLOT]);
	let second = 1 + SLOT;
	if data[0] != RADIO_TWO && m.zeros {
		// a 0x23's second slot left out of the recording
		if io.bit(&mut m.second_zero, I::PACKING && data[second..].iter().all(|&b| b == 0)) {
			data[second..].fill(0);
			return;
		}
	}
	// a 0x23's second slot is what the last 0x24 left there, unless reports went missing in between
	let fresh = data[0] == RADIO_TWO || io.bit(&mut m.second_slot, I::PACKING && data[second..] != last[second..]);
	if fresh {
		packet(io, m, s, &mut data[second..], &last[second..]);
	}
}

/// One packet in its slot: `buf` is the slot (unpacking, as the packet before left it), `old` the slot before
fn packet<I: Io>(io: &mut I, m: &mut Models, s: &mut Stream, buf: &mut [u8], old: &[u8]) {
	let r = &mut s.radio;
	// [time high] [size] [time low] [payload: size - 1 bytes, the flags first]. The flags are coded first: what a
	// packet holds says most of how long it is
	let mut flags = 0;
	if io.bit(&mut m.no_payload, buf[1] == 1) {
		buf[1] = 1;
	} else {
		flags = m.flags.code(io, r.flags as usize | (r.imu_before as usize) << 8, buf[3] as u32) as u8;
		buf[3] = flags;
		r.imu_before = r.flags & HAS_IMU != 0;
		r.flags = flags;
		buf[1] = m.size.code(io, flags as usize, buf[1] as u32) as u8;
	}
	let end = (3 + buf[1].wrapping_sub(1) as usize).min(SLOT);
	let mut at = end.min(4);
	// a packet laid out as the flags say: a heartbeat is not
	let mut known = end > 3 && flags != HEARTBEAT;

	let time = (buf[0] as u16) << 8 | buf[2] as u16;
	if known && flags & HAS_IMU != 0 && end - at >= 13 {
		// an IMU sample: a third time byte, then six axes. The samples come at a steady step
		let t = (time as u32) << 8 | buf[4] as u32;
		let predicted = (r.imu_time + r.imu_step) & 0xff_ffff;
		let off = m.imu_time.code(io, 0, apart(t, predicted) as i64);
		let t = (predicted as i64 + off) as u32 & 0xff_ffff;
		buf[0] = (t >> 16) as u8;
		buf[2] = (t >> 8) as u8;
		buf[4] = t as u8;
		let step = t.wrapping_sub(r.imu_time) & 0xff_ffff;
		if step < 1 << 16 {
			r.imu_step = step;
		}
		r.imu_time = t;
		r.time = (t >> 8) as u16;
		for axis in 0..6 {
			let i = 5 + 2 * axis;
			let value = s.imu.code(io, &mut m.radio_imu, axis, i16::from_le_bytes([buf[i], buf[i + 1]]));
			buf[i..i + 2].copy_from_slice(&value.to_le_bytes());
		}
		at = 17;
	} else {
		let off = m.packet_time.code(io, 0, time.wrapping_sub(r.time) as i16 as i64);
		let time = r.time.wrapping_add(off as u16);
		buf[0] = (time >> 8) as u8;
		buf[2] = time as u8;
		r.time = time;
	}

	// between the sample and the light: a status (a marker, then a byte or two by what it is) and the input (its
	// own flags, then a byte per button and trigger and four for the pad; or a kind, and seven bytes of touch)
	let kept = r.plain.entry(flags).or_insert([0; PAYLOAD]);
	if known && flags & HAS_STATUS != 0 {
		let marker = at;
		m.plain.code(io, kept, buf, &mut at, 1, end);
		if at > marker {
			match buf[marker] {
				0x80 | 0x20 => m.plain.code(io, kept, buf, &mut at, 1, end),
				0x30 => m.plain.code(io, kept, buf, &mut at, 2, end),
				_ => known = false,
			}
		}
	}
	if known && flags & HAS_INPUT != 0 && at < end {
		let input = at;
		m.plain.code(io, kept, buf, &mut at, 1, end);
		let input = buf[input];
		if input & 7 != 0 {
			let n = (input & 1) as usize + (input >> 2 & 1) as usize + 4 * (input >> 1 & 1) as usize;
			m.plain.code(io, kept, buf, &mut at, n, end);
		} else if at < end {
			let kind = at;
			m.plain.code(io, kept, buf, &mut at, 1, end);
			match buf[kind] {
				0xa1 => m.plain.code(io, kept, buf, &mut at, 7, end),
				_ => known = false,
			}
		}
	}
	if known && flags & HAS_LIGHT != 0 {
		s.light.code(io, &mut m.light, buf, at, end);
	} else {
		m.plain.code(io, kept, buf, &mut at, PAYLOAD, end);
	}

	// after the payload the dongle writes four bytes of its own (a check of some kind: they follow from nothing in
	// the packet), as far as the slot has room; past them the slot keeps what was there. Where reports went missing
	// in between, more than four bytes differ from what was there: as many as do are coded
	let room = SLOT - end;
	if room > 0 {
		// (the second version: or nothing, in a recording written without them)
		if m.zeros && io.bit(&mut m.rest_zero, I::PACKING && buf[end..].iter().all(|&b| b == 0)) {
			buf[end..].fill(0);
			return;
		}
		let fresh = match I::PACKING {
			true => (end..SLOT).rev().find(|&i| buf[i] != old[i]).map_or(0, |i| i + 1 - end),
			false => 0,
		};
		let fresh = (m.after.code(io, room.min(5), fresh as u32) as usize).min(room);
		for b in &mut buf[end..end + fresh] {
			*b = io.raw(8, *b as u32) as u8;
		}
	}
}

fn headset_imu<I: Io>(io: &mut I, m: &mut Models, s: &mut Stream, data: &mut [u8], last: &[u8]) {
	let h = &mut s.headset;
	// the three samples in the order they are replaced, from the one whose turn it is
	let first = h.next;
	for n in 0..3 {
		let k = (first + n) % 3;
		let at = 1 + SAMPLE * k;
		let fresh = I::PACKING && data[at..at + SAMPLE] != last[at..at + SAMPLE];
		if !io.bit(&mut m.sample_fresh[n], fresh) {
			continue;
		}
		let sequence = m.sequence.code(io, 0, data[at + 16].wrapping_sub(h.sequence) as u32) as u8;
		h.sequence = h.sequence.wrapping_add(sequence);
		data[at + 16] = h.sequence;

		let time = u32::from_le_bytes([data[at + 12], data[at + 13], data[at + 14], data[at + 15]]);
		let predicted = h.time.wrapping_add(h.step);
		let off = m.sample_time.code(io, 0, time.wrapping_sub(predicted) as i32 as i64);
		let time = predicted.wrapping_add(off as u32);
		data[at + 12..at + 16].copy_from_slice(&time.to_le_bytes());
		let step = time.wrapping_sub(h.time);
		if step < 1 << 20 {
			h.step = step;
		}
		h.time = time;

		for axis in 0..6 {
			let i = at + 2 * axis;
			let value = s.imu.code(io, &mut m.headset_imu, axis, i16::from_le_bytes([data[i], data[i + 1]]));
			data[i..i + 2].copy_from_slice(&value.to_le_bytes());
		}
		h.next = (k + 1) % 3;
	}
}

fn headset_light<I: Io>(io: &mut I, m: &mut Models, s: &mut Stream, data: &mut [u8], last: &[u8]) {
	data[1] = m.light_len.code(io, s.headset.light_len as usize, data[1] as u32) as u8;
	s.headset.light_len = data[1];
	let end = (2 + data[1] as usize).min(data.len());
	s.light.code(io, &mut m.light, data, 2, end);
	// past the light the report keeps what was there
	let room = data.len() - end;
	if room > 0 {
		let fresh = match I::PACKING {
			true => (end..data.len()).rev().find(|&i| data[i] != last[i]).map_or(0, |i| i + 1 - end),
			false => 0,
		};
		let fresh = (m.tail.code(io, 0, fresh as u32) as usize).min(room);
		for b in &mut data[end..end + fresh] {
			*b = io.raw(8, *b as u32) as u8;
		}
	}
}

fn changes<I: Io>(io: &mut I, c: &mut Changes, data: &mut [u8], last: &[u8]) {
	// how far the changed bytes reach, then each byte up to there: whether it changed, and by how much
	let reach = match I::PACKING {
		true => (1..data.len()).rev().find(|&i| data[i] != last[i]).unwrap_or(0),
		false => 0,
	};
	let reach = c.reach.code(io, 0, reach as i64).clamp(0, data.len() as i64 - 1) as usize;
	let mut same = true;
	for i in 1..=reach {
		same = !io.bit(&mut c.changed[same as usize], data[i] != last[i]);
		data[i] = match same {
			true => last[i],
			false => last[i].wrapping_add(c.byte.code(io, i.min(63), data[i].wrapping_sub(last[i]) as u32) as u8),
		};
	}
}
