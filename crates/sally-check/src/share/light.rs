//! The base stations' ids, which the light itself carries, rewritten in a recording.
//!
//! A gen2 station sends its info block (its id, its factory calibration, its up direction: OOTX) one bit a rotation,
//! over and over, and a device hands each bit on with the rotation's sync: bit 26 of the sync's entry in its report.
//! So a recording holds every station's id some six times a minute per device, as bits spread over hundreds of
//! reports.
//!
//! Each device's syncs of each station are followed here as the runtime follows them (a skipped rotation counted as
//! a bit not seen, `sally_lighthouse::ootx`'s decoder). A block that arrives whole, its checksum right, says two
//! things: what the station sends, and where in its block every rotation of that stretch of syncs stands, for the
//! blocks follow one another with one bit between. Every sync of the stretch is then given the bit that the same
//! block with the station's stand-in id (and the checksum that goes with it) has at its place. A stretch without a
//! whole block gets ones: nothing can be read from it, by a reader or here.
//!
//! No bit of the original is kept, so nothing of an id is: what is written follows from the stand-in's block and
//! from where the whole blocks stood. A reader of it finds the blocks the original gave it, at their times, for
//! stations named by their stand-ins, and may find a few more: a bit the original had wrong is right in the
//! stand-in's block.

use std::collections::{BTreeSet, HashMap};

use sally_lighthouse::gen2;
use sally_lighthouse::ootx::{OotxDecoder, OotxEvent};

use sally_math::crc::crc32;

/// The bit of a sync's entry that is the station's data bit, and the byte and mask it is in the entry's four
pub const DATA_BIT_BYTE: usize = 3;
pub const DATA_BIT_MASK: u8 = 0x04;

/// One device's syncs of one station
struct Stream {
	last_sync: u32,
	decoder: OotxDecoder,
	/// the rotations of the stretch so far (seen or not), the syncs seen in it (its rotation, its number), and the
	/// whole blocks found in it (the rotation of its first bit, its bits with the stand-in's id)
	rotations: i64,
	seen: Vec<(i64, usize)>,
	blocks: Vec<(i64, Vec<bool>)>,
}

/// Every device's syncs of every station, and what each sync's data bit is to be
pub struct Stations {
	streams: HashMap<(usize, u8), Stream>,
	/// per sync, in the order they were given: the data bit it gets (once `finish` has been called)
	pub bits: Vec<bool>,
	/// the whole blocks found
	pub blocks: u64,
	/// the ids the blocks carried
	pub ids: BTreeSet<u32>,
}

impl Default for Stations {
	fn default() -> Stations {
		Stations::new()
	}
}

/// A block as the bits that carry it: 17 zeros and a one, then its bytes (the length, the payload padded to an
/// even count, the checksum) sixteen bits at a time, the first bit a byte's highest, with a one after every sixteen
/// but the last
pub fn block_bits(payload: &[u8]) -> Vec<bool> {
	let mut bytes = (payload.len() as u16).to_le_bytes().to_vec();
	bytes.extend_from_slice(payload);
	if payload.len() % 2 == 1 {
		bytes.push(0);
	}
	bytes.extend_from_slice(&crc32(payload).to_le_bytes());
	let mut bits = vec![false; 17];
	bits.push(true);
	for (i, pair) in bytes.chunks(2).enumerate() {
		if i > 0 {
			bits.push(true);
		}
		for b in pair {
			bits.extend((0..8).rev().map(|k| b >> k & 1 == 1));
		}
	}
	bits
}

impl Stream {
	/// The stretch's syncs given their bits, and the stretch let go
	fn close(&mut self, bits: &mut [bool]) {
		for &(at, mark) in &self.seen {
			// the nearest whole block says where this rotation stands in the station's block
			let nearest = self.blocks.iter().min_by_key(|(start, block)| (at - start - block.len() as i64 / 2).abs());
			bits[mark] = match nearest {
				Some((start, block)) => {
					let place = (at - start).rem_euclid(block.len() as i64 + 1) as usize;
					block.get(place).copied().unwrap_or(true)
				}
				None => true,
			};
		}
		self.rotations = 0;
		self.seen.clear();
		self.blocks.clear();
	}
}

impl Stations {
	pub fn new() -> Stations {
		Stations { streams: HashMap::new(), bits: Vec::new(), blocks: 0, ids: BTreeSet::new() }
	}

	/// A sync of the device `obj`, as the runtime's survive_default_sync_process takes it: its interval from the
	/// last one checked against the channel's rotor, a bit not seen for each rotation skipped, then its own bit.
	/// `id` gives the id a block is rewritten with for the one it carries. Returns the sync's number
	pub fn sync(
		&mut self, obj: usize, channel: u8, timecode: u32, data: bool, id: &mut dyn FnMut(u32) -> u32,
	) -> usize {
		let mark = self.bits.len();
		self.bits.push(true);
		if channel as usize >= sally_lighthouse::NUM_GEN2_LIGHTHOUSES {
			return mark;
		}
		let s = self.streams.entry((obj, channel)).or_insert_with(|| Stream {
			last_sync: 0,
			decoder: OotxDecoder::new(0.0),
			rotations: 0,
			seen: Vec::new(),
			blocks: Vec::new(),
		});
		let mut skipped = 0;
		if s.last_sync == 0 {
			// the rotor is timed afresh: the rotations since the last sync are not known, and a new stretch starts
			s.close(&mut self.bits);
		} else {
			let interval = gen2::sync_interval(gen2::timecode_difference(timecode, s.last_sync), channel as usize);
			skipped = interval.rotations_since - 1;
			if interval.err > 1.0 {
				if skipped > 10 {
					s.last_sync = 0;
				}
				return mark;
			}
		}
		s.last_sync = timecode;
		for seen in std::iter::repeat_n(false, skipped.max(0) as usize).chain([true]) {
			let at = s.rotations;
			s.rotations += 1;
			if seen {
				s.seen.push((at, mark));
			}
			let event = s.decoder.pump_bit(if seen { data as i8 } else { -1 });
			let Some(OotxEvent::Packet(p)) = event else { continue };
			let (Some(real), true) = (p.data.get(2..6), p.data.len() == p.length as usize) else { continue };
			let real = u32::from_le_bytes([real[0], real[1], real[2], real[3]]);
			let mut payload = p.data.clone();
			payload[2..6].copy_from_slice(&id(real).to_le_bytes());
			let block = block_bits(&payload);
			self.blocks += 1;
			self.ids.insert(real);
			s.blocks.push((at + 1 - block.len() as i64, block));
		}
		mark
	}

	/// The recording is over: every stretch's syncs given their bits
	pub fn finish(&mut self) {
		for s in self.streams.values_mut() {
			s.close(&mut self.bits);
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A station's info block of 43 bytes with `id`
	fn payload(id: u32) -> Vec<u8> {
		let mut p: Vec<u8> = (0..43u8).map(|i| i.wrapping_mul(37).wrapping_add(11)).collect();
		p[2..6].copy_from_slice(&id.to_le_bytes());
		p
	}

	/// A channel's rotation in ticks
	fn period(channel: usize) -> u32 {
		(48_000_000.0 / gen2::FREQ_PER_CHANNEL[channel]) as u32
	}

	/// The ids a decoder reads from the bits of a stream
	fn read(bits: &[Option<bool>]) -> Vec<u32> {
		let mut d = OotxDecoder::new(0.0);
		let mut ids = Vec::new();
		for b in bits {
			if let Some(OotxEvent::Packet(p)) = d.pump_bit(b.map_or(-1, |b| b as i8)) {
				ids.push(u32::from_le_bytes([p.data[2], p.data[3], p.data[4], p.data[5]]));
			}
		}
		ids
	}

	#[test]
	fn a_blocks_bits_are_what_the_decoder_reads() {
		let bits: Vec<Option<bool>> = block_bits(&payload(0xb1c2_d3e4)).into_iter().map(Some).collect();
		assert_eq!(bits.len(), 18 + 25 * 17 - 1);
		assert_eq!(read(&bits), [0xb1c2_d3e4]);
	}

	#[test]
	fn every_sync_of_a_stretch_gets_the_stand_ins_bit_at_its_place() {
		let real = 0xb1c2_d3e4;
		let block = block_bits(&payload(real));
		// the stream: the end of a block, three whole ones (a one between them, as a station sends), the start of one
		let mut stream: Vec<bool> = block[200..].to_vec();
		for _ in 0..3 {
			stream.push(true);
			stream.extend(&block);
		}
		stream.push(true);
		stream.extend(&block[..300]);

		let mut st = Stations::new();
		let period = period(0);
		let mut t = 5_000u32;
		for &b in &stream {
			st.sync(0, 0, t, b, &mut |id| {
				assert_eq!(id, real);
				7
			});
			t = t.wrapping_add(period);
		}
		st.finish();
		assert_eq!(st.blocks, 3);
		assert_eq!(st.ids.iter().copied().collect::<Vec<_>>(), [real]);
		// what is written is the stand-in's own stream, the cut blocks at either end too
		let stand_in = block_bits(&payload(7));
		let mut want: Vec<bool> = stand_in[200..].to_vec();
		for _ in 0..3 {
			want.push(true);
			want.extend(&stand_in);
		}
		want.push(true);
		want.extend(&stand_in[..300]);
		assert_eq!(st.bits, want);
		let out: Vec<Option<bool>> = st.bits.iter().map(|&b| Some(b)).collect();
		assert_eq!(read(&out), [7, 7, 7]);
	}

	#[test]
	fn a_stretch_without_a_whole_block_is_all_ones() {
		let block = block_bits(&payload(0x1234_5678));
		let mut st = Stations::new();
		let mut t = 5_000u32;
		for &b in &block[30..400] {
			st.sync(2, 5, t, b, &mut |_| unreachable!());
			t = t.wrapping_add(period(5));
		}
		st.finish();
		assert_eq!(st.blocks, 0);
		assert!(st.bits.iter().all(|&b| b));
	}

	#[test]
	fn a_rotation_not_seen_is_a_bit_not_seen() {
		let block = block_bits(&payload(5));
		let period = period(3);
		// two blocks; in the second, one rotation of the payload is not seen: the decoder keeps the first's bit there
		let mut st = Stations::new();
		let mut t = 9_000u32;
		let mut n = 0;
		for round in 0..2 {
			for (i, &b) in block.iter().chain([&true]).enumerate() {
				t = t.wrapping_add(period);
				if round == 1 && i == 100 {
					continue;
				}
				st.sync(4, 3, t, b, &mut |_| 9);
				n += 1;
			}
		}
		st.finish();
		assert_eq!(st.blocks, 2);
		assert_eq!(st.bits.len(), n);
		// the bits written are the stand-in's, the rotation not seen passed over
		let stand_in: Vec<bool> = block_bits(&payload(9)).into_iter().chain([true]).collect();
		let mut want = stand_in.clone();
		want.extend(stand_in.iter().enumerate().filter(|(i, _)| *i != 100).map(|(_, b)| *b));
		assert_eq!(st.bits, want);
	}
}
