//! The light data of a packed recording (the gen 2 form: a channel byte, then 4-byte entries, a base station's
//! rotation mark or a sensor's hit, each with a 24-bit time on the device's 48 MHz clock).
//!
//! What can be told ahead is when, and with it who. A base station turns at a steady rate, so its next mark comes a
//! rotation after its last; and a sensor a sweep hit one rotation ago is hit again a rotation later, moved by however
//! far the device has turned since, which is about as far as it moved the rotation before. So every sensor has a
//! time at which it is due, a hit is coded as which of the sensors due it is (the one due first, mostly) and how far
//! from its time it came: a few ticks on a device at rest, some dozens on one in motion, where the entry itself is
//! 32 bits. A hit with no such past (a sensor newly lit) is predicted from the hit before it, or from where the
//! sweep it opens began a rotation ago.
//!
//! Nothing here has to be right: every byte is coded whatever it holds, and a wrong guess only costs bits.

use super::coder::{Io, Number, Prob, Symbol};

/// The channels a channel byte can name, and one more for entries that come before any
const CHANNELS: usize = 17;
const SENSORS: usize = 32;
/// The hits kept per sensor and the sweep starts kept per channel
const KEPT: usize = 4;
/// Two hits further apart than this (ticks) are of different sweeps: a sweep crosses a device in a few thousand
/// ticks, and the two sweeps of a rotation are some 300,000 apart
const SWEEP_GAP: i32 = 1 << 15;
/// A rotation takes 887,000 to 959,000 ticks, by channel
const ROTATION: std::ops::Range<i32> = 800_000..1_100_000;
/// How many rotations back a hit or a mark still predicts (the 24-bit time wraps after 17)
const ROTATIONS_BACK: i32 = 16;

const TIME: u32 = 0xff_ffff;

/// `a - b` on the 24-bit clock, as the shorter way round
pub(super) fn apart(a: u32, b: u32) -> i32 {
	((a.wrapping_sub(b) << 8) as i32) >> 8
}

/// `t` moved by `by` ticks on the 24-bit clock
fn moved(t: u32, by: i64) -> u32 {
	(t as i64 + by) as u32 & TIME
}

#[derive(Clone, Copy, Default)]
struct Hit {
	time: u32,
	/// how much later than a rotation after the hit it was predicted from it came, per rotation: the device's motion
	/// as this sensor saw it
	drift: i32,
}

/// Where a hit's prediction came from: the residual's context, as the predictions are not equally good
#[derive(Clone, Copy, PartialEq)]
enum From {
	/// the same sensor, rotations ago
	Sensor = 0,
	/// the entry before it
	Last = 1,
	/// the same sensor, for a hit that opens a sweep
	SensorAhead = 2,
	/// where a sweep began rotations ago, for a hit that opens one
	SweepAhead = 3,
	/// a hit that opens a sweep with nothing to go by
	LastAhead = 4,
	/// the channel's first entry
	Nothing = 5,
}

#[derive(Clone, Copy)]
struct Prediction {
	time: u32,
	from: From,
	/// the kept hit it came from and how many rotations back that was
	hit: Option<(usize, i32)>,
}

/// What a channel expects next of its sensors
struct Expected {
	predictions: [Prediction; SENSORS],
	/// the sensors, the one due first first
	order: [u8; SENSORS],
	/// how many are still to come in the sweep under way
	waiting: usize,
}

/// What is known of one base station's light on one device
#[derive(Clone)]
struct Channel {
	/// the last entry's time
	last: Option<u32>,
	/// the last rotation mark
	mark: Option<u32>,
	/// the rotation's length in 1/16 ticks, once two marks a rotation apart have been seen
	rotation: i32,
	/// each sensor's last hits, the newest first
	hits: [[Hit; KEPT]; SENSORS],
	hit_count: [u8; SENSORS],
	/// where the last sweeps began, the newest first
	starts: [u32; KEPT],
	start_count: u8,
}

impl Channel {
	fn new() -> Channel {
		Channel {
			last: None,
			mark: None,
			rotation: 0,
			hits: [[Hit::default(); KEPT]; SENSORS],
			hit_count: [0; SENSORS],
			starts: [0; KEPT],
			start_count: 0,
		}
	}

	fn rotation(&self) -> i32 {
		self.rotation >> 4
	}

	/// The hit of `sensor` that comes next, given whether it opens a new sweep
	fn predict_hit(&self, sensor: usize, opens: bool) -> Prediction {
		let Some(last) = self.last else { return Prediction { time: 0, from: From::Nothing, hit: None } };
		let rotation = self.rotation();
		let kept = &self.hits[sensor][..self.hit_count[sensor] as usize];
		if !opens {
			// within a sweep: this sensor's newest hit that lands by the last entry, whole rotations on
			if rotation != 0 {
				for (i, h) in kept.iter().enumerate() {
					let n = (apart(last, h.time) + rotation / 2).div_euclid(rotation);
					if !(1..=ROTATIONS_BACK).contains(&n) {
						continue;
					}
					let time = moved(h.time, n as i64 * (rotation + h.drift) as i64);
					if apart(time, last).abs() <= SWEEP_GAP {
						return Prediction { time, from: From::Sensor, hit: Some((i, n)) };
					}
				}
			}
			return Prediction { time: last, from: From::Last, hit: None };
		}
		// a new sweep: the first time after the last entry at which this sensor, or failing that any sweep's start,
		// comes round again
		if rotation != 0 {
			let ahead = |from: u32, drift: i32| -> Option<(i32, u32, i32)> {
				let behind = apart(last, from) + SWEEP_GAP / 2;
				let n = if behind < 0 { 1 } else { behind / rotation + 1 };
				if n > ROTATIONS_BACK {
					return None;
				}
				let time = moved(from, n as i64 * (rotation + drift) as i64);
				let wait = apart(time, last);
				(wait > SWEEP_GAP / 2).then_some((wait, time, n))
			};
			let sensor_next = kept
				.iter()
				.enumerate()
				.filter_map(|(i, h)| ahead(h.time, h.drift).map(|(wait, time, n)| (wait, time, i, n)))
				.min_by_key(|c| c.0);
			let sweep_next = self.starts[..self.start_count as usize]
				.iter()
				.filter_map(|&s| ahead(s, 0).map(|(wait, time, _)| (wait, time)))
				.min_by_key(|c| c.0);
			match (sensor_next, sweep_next) {
				// the sensor's own past, unless a sweep is due well before it: the sensor was not lit in that one
				(Some((wait, time, i, n)), sweep) if sweep.is_none_or(|s| wait <= s.0 + SWEEP_GAP) => {
					return Prediction { time, from: From::SensorAhead, hit: Some((i, n)) };
				}
				(_, Some((_, time))) => return Prediction { time, from: From::SweepAhead, hit: None },
				_ => {}
			}
		}
		Prediction { time: last, from: From::LastAhead, hit: None }
	}

	/// Every sensor's next hit, and the sensors in the order those hits are due: the ones still to come in this sweep
	/// by their times (or, for a hit that opens a sweep, the ones due soonest), then the ones overdue, the ones this
	/// sweep has hit already, and the ones nothing is known of
	fn expected(&self, opens: bool) -> Expected {
		// a hit comes a little before the one listed ahead of it now and then
		const SLACK: i64 = 256;
		const OVERDUE: i64 = 1 << 26;
		const HIT: i64 = 1 << 27;
		const UNKNOWN: i64 = 1 << 28;
		let mut e = Expected {
			predictions: [Prediction { time: 0, from: From::Nothing, hit: None }; SENSORS],
			order: [0; SENSORS],
			waiting: 0,
		};
		let last = self.last.unwrap_or(0);
		let mut keys = [(0i64, 0u8); SENSORS];
		for sensor in 0..SENSORS {
			let p = self.predict_hit(sensor, opens);
			let key = match p.from {
				From::Sensor => {
					let ahead = apart(p.time, last) as i64;
					if apart(last, self.hits[sensor][0].time).abs() <= SWEEP_GAP {
						HIT + ahead.abs()
					} else if ahead >= -SLACK {
						e.waiting += 1;
						ahead + SLACK
					} else {
						OVERDUE - ahead
					}
				}
				From::SensorAhead => apart(p.time, last) as i64,
				_ => UNKNOWN,
			};
			keys[sensor] = (key, sensor as u8);
			e.predictions[sensor] = p;
		}
		keys.sort_unstable();
		for (place, key) in keys.iter().enumerate() {
			e.order[place] = key.1;
		}
		e
	}

	fn add_hit(&mut self, sensor: usize, time: u32, opens: bool, p: &Prediction) {
		let drift = match p.hit {
			Some((i, n)) => {
				let on = moved(self.hits[sensor][i].time, n as i64 * self.rotation() as i64);
				(apart(time, on) / n).clamp(-SWEEP_GAP, SWEEP_GAP)
			}
			None => 0,
		};
		let kept = &mut self.hits[sensor];
		kept.copy_within(0..KEPT - 1, 1);
		kept[0] = Hit { time, drift };
		self.hit_count[sensor] = (self.hit_count[sensor] + 1).min(KEPT as u8);
		if opens {
			self.starts.copy_within(0..KEPT - 1, 1);
			self.starts[0] = time;
			self.start_count = (self.start_count + 1).min(KEPT as u8);
		}
		self.last = Some(time);
	}

	/// The next rotation mark, and whether it is a prediction (1) or only the last time seen (0)
	fn predict_mark(&self) -> (u32, usize) {
		let rotation = self.rotation();
		match (self.mark, self.last) {
			(Some(mark), Some(last)) if rotation != 0 => {
				let n = ((apart(last, mark) + rotation / 2).div_euclid(rotation)).clamp(1, ROTATIONS_BACK + 1);
				(moved(mark, n as i64 * rotation as i64), 1)
			}
			(Some(mark), _) => (mark, 0),
			(None, Some(last)) => (last, 0),
			(None, None) => (0, 0),
		}
	}

	fn add_mark(&mut self, time: u32) {
		if let Some(mark) = self.mark {
			let turn = apart(time, mark);
			if ROTATION.contains(&turn) {
				// the first rotation as it is, then an eighth of the way to each new one: the marks jitter by a few
				// ticks, the rate does not
				self.rotation = match self.rotation {
					0 => turn << 4,
					rotation => rotation + ((turn << 4) - rotation) / 8,
				};
			}
		}
		self.mark = Some(time);
		self.last = Some(time);
	}
}

/// What came before a token in its packet: the token's kind is coded under it
#[derive(Clone, Copy)]
enum Before {
	Start = 0,
	ChannelByte = 1,
	Mark = 2,
	Hit = 3,
}

/// A device's light as the packer and the unpacker both follow it
pub struct Light {
	channels: Vec<Option<Box<Channel>>>,
	/// the channel the entries are on (CHANNELS - 1 before any channel byte)
	channel: usize,
	channel_byte: u8,
	/// the length of the last hit's residual
	length: usize,
}

impl Default for Light {
	fn default() -> Light {
		Light { channels: vec![None; CHANNELS], channel: CHANNELS - 1, channel_byte: 0, length: 0 }
	}
}

/// The classes a hit residual's length falls in, as the next hit's context: a device at rest stays at rest for a while
fn length_class(length: usize) -> usize {
	match length {
		0..=2 => 0,
		3..=4 => 1,
		5..=6 => 2,
		7..=9 => 3,
		10..=13 => 4,
		_ => 5,
	}
}

const LENGTH_CLASSES: usize = 6;

/// The models light is coded under, shared by the devices
pub struct LightModels {
	is_channel: [Prob; 16],
	channel: Symbol,
	is_hit: [Prob; 4],
	opens: [Prob; 6],
	place: Symbol,
	half_clock: Prob,
	hit: Number,
	mark: Number,
	slow_bit: Prob,
	mark_flag: Prob,
	mark_unused: Symbol,
	leftover: Symbol,
}

impl LightModels {
	pub fn new() -> LightModels {
		LightModels {
			is_channel: [Prob::new(); 16],
			channel: Symbol::new(7, 128),
			is_hit: [Prob::new(); 4],
			opens: [Prob::new(); 6],
			place: Symbol::new(5, 8),
			half_clock: Prob::new(),
			hit: Number::new(6 * LENGTH_CLASSES),
			mark: Number::new(2),
			slow_bit: Prob::new(),
			mark_flag: Prob::new(),
			mark_unused: Symbol::new(4, 1),
			leftover: Symbol::new(7, 1),
		}
	}
}

impl Light {
	/// The light data in `buf[from..to]`: packing, read from it; unpacking, written to it
	pub fn code<I: Io>(&mut self, io: &mut I, m: &mut LightModels, buf: &mut [u8], from: usize, to: usize) {
		let mut at = from;
		let mut before = Before::Start;
		while at < to {
			// a channel byte has its lowest bit set, an entry has it clear. A packet's light is channel bytes with
			// whole entries between them, so what is left of it, modulo an entry, says how many channel bytes are
			// still to come
			if io.bit(&mut m.is_channel[before as usize * 4 + (to - at) % 4], buf[at] & 1 != 0) {
				let byte =
					(m.channel.code(io, (self.channel_byte >> 1) as usize, (buf[at] >> 1) as u32) << 1) as u8 | 1;
				buf[at] = byte;
				self.channel_byte = byte;
				self.channel = (byte >> 4) as usize;
				at += 1;
				before = Before::ChannelByte;
				continue;
			}
			if to - at < 4 {
				// not a whole entry: its bytes as they are (the first one's lowest bit is known)
				buf[at] = (m.leftover.code(io, 0, (buf[at] >> 1) as u32) << 1) as u8;
				at += 1;
				while at < to {
					buf[at] = io.raw(8, buf[at] as u32) as u8;
					at += 1;
				}
				break;
			}
			let entry = u32::from_le_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]]);
			let channel = self.channels[self.channel].get_or_insert_with(|| Box::new(Channel::new()));
			let entry = if io.bit(&mut m.is_hit[before as usize], entry & 2 != 0) {
				// a hit: bit 2 the half-clock flag, 24 bits of time, 5 of sensor. Whether it opens a new sweep, then
				// which sensor it is as its place among the sensors by when each is due, then how far off its time
				// it came
				let time = (entry >> 3) & TIME;
				let within = channel.expected(false);
				let waiting = within.waiting;
				let opens = channel.last.is_some_and(|last| apart(time, last).abs() > SWEEP_GAP);
				let context = waiting.min(2) * 2 + matches!(before, Before::Hit) as usize;
				let opens = io.bit(&mut m.opens[context], opens);
				let expected = if opens { channel.expected(true) } else { within };
				let place = expected.order.iter().position(|&s| s as u32 == entry >> 27).unwrap_or(0);
				let place = m.place.code(io, opens as usize * 4 + waiting.min(3), place as u32) as usize;
				let sensor = expected.order[place] as usize;
				let half = io.bit(&mut m.half_clock, entry & 4 != 0);
				let p = expected.predictions[sensor];
				let context = p.from as usize * LENGTH_CLASSES + length_class(self.length);
				let off = m.hit.code(io, context, apart(time, p.time) as i64);
				let time = moved(p.time, off);
				channel.add_hit(sensor, time, opens, &p);
				self.length = Number::length_of(off);
				before = Before::Hit;
				(sensor as u32) << 27 | time << 3 | (half as u32) << 2 | 2
			} else {
				// a rotation mark: 24 bits of time, the slow data bit, a flag, 4 bits not known to be used
				let time = (entry >> 2) & TIME;
				let (predicted, context) = channel.predict_mark();
				let off = m.mark.code(io, context, apart(time, predicted) as i64);
				let time = moved(predicted, off);
				channel.add_mark(time);
				let slow = io.bit(&mut m.slow_bit, entry & (1 << 26) != 0);
				let flag = io.bit(&mut m.mark_flag, entry & (1 << 27) != 0);
				let unused = m.mark_unused.code(io, 0, entry >> 28);
				before = Before::Mark;
				unused << 28 | (flag as u32) << 27 | (slow as u32) << 26 | time << 2
			};
			buf[at..at + 4].copy_from_slice(&entry.to_le_bytes());
			at += 4;
		}
	}
}
