//! OOTX: the calibration a base station broadcasts one bit per rotation (the ootx bit of each sync), and the gen2 info
//! block it carries. Ported from the C runtime's ootx_decoder.c (libsurvive's decoder, with sally's check on short
//! packets), operation for operation (docs/checking.md).
//!
//! A frame is a preamble (17 zeros and a one), then 16-bit words, each followed by a sync bit (a one): the payload's
//! length, the payload (padded to an even length) and its CRC-32. The bits fill each byte from its top bit down; the
//! length, the CRC and the info block's fields are little-endian, as the C reads them on every machine sally runs on.
//! The decoder looks for a preamble at every bit, so a stream can start over at any time.
//!
//! Where the C calls back (ootx_error_clbk, ootx_packet_clbk, ootx_bad_crc_clbk), `pump_bit` returns the event. A bit
//! gives at most one, and no callback touches the decoder's state (they read only the caller's user and user1), so
//! handing it over once the bit is done is the same: the decoder is the C's with all three callbacks set, as
//! survive_ootx_behavior (survive_process_gen2.c) sets them.
//!
//! The CRC is the C's crc32.c's (the redist copy linked into libsurvive itself), which is zlib's and zip's CRC-32:
//! sally's one copy of it (`sally_math::crc`).
//!
//! One thing cannot be the C's. A length of 0xffff (sixteen ones after a preamble) pads to 0 in the C's uint16_t, so
//! the packet is complete after six bytes and the C runs the CRC over 65535 bytes from a 62-byte buffer, reading past
//! the decoder into the heap. Here the CRC stops at the end of the buffer. The outcome is the C's (a bad CRC, counted,
//! and the buffer reset, unless a checksum matches by chance); only the checksum differs, which in the C is whatever
//! memory follows the decoder.
#![forbid(unsafe_code)]

use sally_math::crc::crc32;

/// OOTX_MAX_BUFF_SIZE: the decoder's buffer, the length field, the payload and its CRC
pub const OOTX_MAX_BUFF_SIZE: usize = 64;

/// sizeof(lighthouse_info_v15_packed): a gen2 station's info block
pub const LIGHTHOUSE_INFO_V15_SIZE: usize = 43;

/// ootx_packet: a complete frame
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OotxPacket {
	/// the length field
	pub length: u16,
	/// the payload, as the C's `data` points to it in the decoder's buffer: the `length` bytes after the length field
	/// (all of them but for the 0xffff length, which gets the 62 the buffer has)
	pub data: Vec<u8>,
	/// the CRC the frame carries
	pub crc32: u32,
}

/// What a bit can give: the C's three callbacks
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OotxEvent {
	/// ootx_error_clbk: the decoder's message, word for word (it says "Preamble found" too)
	Error(&'static str),
	/// ootx_packet_clbk: a packet whose CRC matched
	Packet(OotxPacket),
	/// ootx_bad_crc_clbk: a packet whose CRC did not; `crc` is the one computed over its payload
	BadCrc { packet: OotxPacket, crc: u32 },
}

/// ootx_decoder_context.stats
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OotxStats {
	pub bits_seen: u32,
	pub bad_sync_bits: u32,
	/// the bad CRCs, and the packets too short to hold a station id
	pub bad_crcs: u32,
	pub packets_found: u32,
	/// the payload bytes of the packets found
	pub used_bytes: u32,
	/// the bits written into the buffer, and of those the guessed ones (a -1 leaves the bit as it was)
	pub package_bits: u32,
	pub guess_bits: u32,
	/// the run time the decoder was started at (a float in the C's ootx_init_decoder_context)
	pub started_s: f64,
}

/// ootx_decoder_context, without what Rust has no use for: payload_size (a pointer to the buffer's first two bytes,
/// `payload_size()` here), bits_post_preamble (never used), the caller's user and user1, and the callbacks
#[derive(Clone, Debug)]
pub struct OotxDecoder {
	/// the frame as it arrives: the length field, the payload, its CRC
	pub buffer: [u8; OOTX_MAX_BUFF_SIZE],
	/// the byte being written, and its bits written so far (from the top)
	pub buf_offset: u16,
	pub bits_written: u8,
	/// the last 32 bits, a -1 as detect_preamble guessed it
	pub preamble: u32,
	/// bits since the preamble or the last sync bit: the 17th is a sync bit
	pub bits_processed: u8,
	/// bits since the buffer was reset, and bits in all (neither is read, in the C either)
	pub offset: u16,
	pub total_offset: u16,
	/// 1 while a frame is being written
	pub found_preamble: u8,
	/// set by the caller (survive_ootx_behavior: the ootx-ignore-sync-error option): a zero sync bit is reported and
	/// the frame kept
	pub ignore_sync_bit_error: i32,
	pub stats: OotxStats,
}

impl OotxDecoder {
	/// SV_CALLOC then ootx_init_decoder_context: all zero but the preamble register (all ones, so a preamble needs 17
	/// zeros of its own) and the start time. `start` is a float as in the C, which is passed the run time (a double)
	pub fn new(start: f32) -> Self {
		OotxDecoder {
			buffer: [0; OOTX_MAX_BUFF_SIZE],
			buf_offset: 0,
			bits_written: 0,
			preamble: 0xFFFFFFFF,
			bits_processed: 0,
			offset: 0,
			total_offset: 0,
			found_preamble: 0,
			ignore_sync_bit_error: 0,
			stats: OotxStats { started_s: f64::from(start), ..OotxStats::default() },
		}
	}

	/// *(ctx->payload_size): the length field, the buffer's first two bytes
	pub fn payload_size(&self) -> u16 {
		u16::from_le_bytes([self.buffer[0], self.buffer[1]])
	}

	fn detect_preamble(&mut self, mut dbit: i8) -> bool {
		self.preamble <<= 1;
		// the guess for a -1 is only the preamble register's: the caller goes on with the -1
		if dbit < 0 {
			// Tend to not trigger preamble if we already have one
			if self.found_preamble != 0 {
				dbit = 1;
			} else {
				// If we don't have a preamble; and we just saw a bunch of 0's, trigger a preamble.
				// if we dont have a bunch of 0's, inject a 0 and maybe it's part of the preamble.
				dbit = if (self.preamble & 0x0001ffff) == 0x00000000 { 1 } else { 0 };
			}
		}

		self.preamble |= dbit as u32;
		(self.preamble & 0x0003ffff) == 0x00000001
	}

	fn reset_buffer(&mut self) {
		self.buf_offset = 0;
		self.bits_written = 0;
		self.offset = 0;
		self.found_preamble = 0;
		// *(ctx->payload_size) = 0: the rest of the buffer keeps what it had
		self.buffer[..2].fill(0);
	}

	fn inc_buffer_offset(&mut self) {
		self.buf_offset += 1;

		// the buffer is going to overflow, wrap the buffer and don't write more data until the preamble is found again
		if usize::from(self.buf_offset) >= OOTX_MAX_BUFF_SIZE {
			self.buf_offset = 0;
			self.found_preamble = 0;
		}
	}

	fn write_to_buffer(&mut self, dbit: i8) {
		let current_byte = &mut self.buffer[usize::from(self.buf_offset)];

		// Purposefully leave the value as is if dbit is -1
		let mask: u8 = 1 << (7 - self.bits_written);
		if dbit == 0 {
			*current_byte &= !mask;
		} else if dbit == 1 {
			*current_byte |= mask;
		} else {
			self.stats.guess_bits = self.stats.guess_bits.wrapping_add(1);
		}
		self.stats.package_bits = self.stats.package_bits.wrapping_add(1);

		self.bits_written += 1;
		if self.bits_written > 7 {
			self.bits_written = 0;
			self.inc_buffer_offset();
		}
	}

	/// ootx_pump_bit: one sync's ootx bit, 0 or 1, or -1 for a sync that was not seen (survive_default_sync_process
	/// pumps one for each rotation it skipped). Any other value does what it does in the C
	pub fn pump_bit(&mut self, dbit: i8) -> Option<OotxEvent> {
		self.total_offset = self.total_offset.wrapping_add(1);
		self.offset = self.offset.wrapping_add(1);
		self.bits_processed += 1;
		self.stats.bits_seen = self.stats.bits_seen.wrapping_add(1);
		if self.detect_preamble(dbit) {
			// data stream can start over at any time so we must always look for preamble bits
			self.reset_buffer();
			self.bits_processed = 0;
			self.found_preamble = 1;
			Some(OotxEvent::Error("Preamble found"))
		} else if self.bits_processed > 16 {
			// every 17th bit needs to be dropped (sync bit); a -1 passes as a good one
			let mut event = None;
			if dbit == 0 {
				if self.ignore_sync_bit_error == 0 {
					if self.found_preamble != 0 {
						event = Some(OotxEvent::Error("OOTX Decoder: Bad sync bit"));
						self.stats.bad_sync_bits = self.stats.bad_sync_bits.wrapping_add(1);
					}
					self.reset_buffer();
				} else if self.found_preamble != 0 {
					event = Some(OotxEvent::Error("OOTX Decoder: Ignoring bad sync bit"));
				}
			}
			self.bits_processed = 0;
			event
		} else if self.found_preamble > 0 {
			// only write to buffer if the preamble is found. if the buffer overflows, found_preamble will be cleared
			// and writing will stop. data would be corrupted, so there is no point in continuing
			self.write_to_buffer(dbit);

			let mut padded_length = self.payload_size();
			// extra null byte if odd (0xffff pads to 0: the module's note)
			padded_length = padded_length.wrapping_add(padded_length & 0x01);

			if i32::from(self.buf_offset) >= i32::from(padded_length) + 6 {
				// once we have a complete ootx packet, send it out
				let length = self.payload_size();
				let data = self.buffer[2..(2 + usize::from(length)).min(OOTX_MAX_BUFF_SIZE)].to_vec();
				// memcpy(&op.crc32, op.data + padded_length, 4): the CRC after the padded payload
				let at = 2 + usize::from(padded_length);
				let crc32_field = u32::from_le_bytes([
					self.buffer[at],
					self.buffer[at + 1],
					self.buffer[at + 2],
					self.buffer[at + 3],
				]);
				let op = OotxPacket { length, data, crc32: crc32_field };

				let crc = crc32(&op.data);

				// an all-zero bit run after a preamble decodes as a zero-length packet with a zero CRC, which "passes"
				// (crc32 of nothing is 0). Its station id reads as 0, and it would replace a real, locked station. A
				// real packet carries at least the firmware version and the station id.
				let event = if op.length < 6 {
					self.stats.bad_crcs = self.stats.bad_crcs.wrapping_add(1);
					OotxEvent::Error("OOTX Decoder: ignoring a packet too short to hold a station id")
				} else if crc != op.crc32 {
					self.stats.bad_crcs = self.stats.bad_crcs.wrapping_add(1);
					OotxEvent::BadCrc { packet: op, crc }
				} else {
					self.stats.packets_found = self.stats.packets_found.wrapping_add(1);
					self.stats.used_bytes = self.stats.used_bytes.wrapping_add(u32::from(op.length));
					OotxEvent::Packet(op)
				};

				self.reset_buffer();
				Some(event)
			} else {
				None
			}
		} else {
			None
		}
	}
}

/// lighthouse_info_v15 (ootx_decoder.h): a gen2 station's info block, the payload of its OOTX packet. Each
/// calibration value is a half float there, a float here (float16 is the C's typedef for float)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LighthouseInfoV15 {
	/// firmware version (bits 15..6), protocol version (bits 5..0)
	pub fw_version: u16,
	/// the station's unique id
	pub id: u32,
	/// per rotor (sweep plane): phase, tilt
	pub fcal_phase: [f32; 2],
	pub fcal_tilt: [f32; 2],
	pub sys_unlock_count: u8,
	pub ootx_model: u8,
	pub fcal_curve: [f32; 2],
	/// the station's up direction ("orientation vector")
	pub accel_dir: [i8; 3],
	/// gibbous phase (a normalized angle) and magnitude
	pub fcal_gibphase: [f32; 2],
	pub fcal_gibmag: [f32; 2],
	/// the channel in the low 7 bits
	pub mode_current: u8,
	pub sys_faults: u8,
	pub fcal_ogeephase: [f32; 2],
	pub fcal_ogeemag: [f32; 2],
	pub nonce: u16,
}

impl LighthouseInfoV15 {
	/// init_lighthouse_info_v15: the block from the first 43 bytes of a payload; None if it is shorter (the C reads 43
	/// bytes whatever the length: the gen2 callback checks it first)
	pub fn new(data: &[u8]) -> Option<Self> {
		let data = data.get(..LIGHTHOUSE_INFO_V15_SIZE)?;
		let mut d = Packed { data, idx: 0 };
		// lighthouse_info_v15_packed, field after field in its order (a struct expression evaluates its fields in the
		// order written)
		Some(LighthouseInfoV15 {
			fw_version: d.u16(),
			id: d.u32(),
			fcal_phase: [d.half(), d.half()],
			fcal_tilt: [d.half(), d.half()],
			sys_unlock_count: d.u8(),
			ootx_model: d.u8(),
			fcal_curve: [d.half(), d.half()],
			accel_dir: [d.i8(), d.i8(), d.i8()],
			fcal_gibphase: [d.half(), d.half()],
			fcal_gibmag: [d.half(), d.half()],
			mode_current: d.u8(),
			sys_faults: d.u8(),
			fcal_ogeephase: [d.half(), d.half()],
			fcal_ogeemag: [d.half(), d.half()],
			nonce: d.u16(),
		})
	}
}

/// a walk through a packed block, as get_ptr walks one: each field right after the one before
struct Packed<'a> {
	data: &'a [u8],
	idx: usize,
}

impl Packed<'_> {
	fn bytes<const N: usize>(&mut self) -> [u8; N] {
		let b = std::array::from_fn(|i| self.data[self.idx + i]);
		self.idx += N;
		b
	}

	fn u8(&mut self) -> u8 {
		self.bytes::<1>()[0]
	}

	fn i8(&mut self) -> i8 {
		self.u8() as i8
	}

	fn u16(&mut self) -> u16 {
		u16::from_le_bytes(self.bytes())
	}

	fn u32(&mut self) -> u32 {
		u32::from_le_bytes(self.bytes())
	}

	fn half(&mut self) -> f32 {
		half_to_float(self.u16())
	}
}

/// _half_to_float: an IEEE half (binary16) as a float, by the C's own bit manipulation (a NaN keeps its payload)
fn half_to_float(mut x: u16) -> f32 {
	// sign
	let mut fnum_i: u32 = (u32::from(x) & 0x8000) << 16;

	if (x & 0x7FFF) == 0 {
		// signed zero
		return f32::from_bits(fnum_i);
	}

	if (x & 0x7c00) == 0 {
		// denormalized: only the mantissa, the intrinsic bit advanced forward
		x = (x & 0x3ff) << 1;
		let mut e: u8 = 0;
		// shift until intrinsic bit of mantissa overflows into exponent, increment exponent each time
		while (x & 0x0400) == 0 {
			x <<= 1;
			e += 1;
		}
		// bias exponent to 127, half floats are biased 15 so only need to go 112 more
		fnum_i |= (112 - u32::from(e)) << 23;
		// insert mantissa
		fnum_i |= u32::from(x & 0x3ff) << 13;
		return f32::from_bits(fnum_i);
	}

	if (x & 0x7c00) == 0x7c00 {
		// for infinity, fraction is 0; for NaN, fraction is anything non zero. we could just copy in bits and not
		// shift, but the mantissa of a NaN can have meaning
		fnum_i |= 0x7f800000 | (u32::from(x & 0x3ff) << 13);
		return f32::from_bits(fnum_i);
	}

	fnum_i |= (u32::from(x & 0x7fff) + 0x1c000) << 13;

	f32::from_bits(fnum_i)
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn half_to_float_every_half() {
		for x in 0..=u16::MAX {
			let f = half_to_float(x);
			let sign = if x & 0x8000 != 0 { -1.0 } else { 1.0 };
			let e = i32::from((x >> 10) & 0x1f);
			let m = f64::from(x & 0x3ff);
			if e == 31 && m != 0.0 {
				// a NaN: sign and payload carried over
				assert_eq!(f.to_bits(), u32::from(x & 0x8000) << 16 | 0x7f800000 | u32::from(x & 0x3ff) << 13);
				continue;
			}
			// the value the half stands for, exactly (every half is a float)
			let want = match e {
				0 => sign * m * 2f64.powi(-24),
				31 => sign * f64::INFINITY,
				_ => sign * (1024.0 + m) * 2f64.powi(e - 25),
			};
			assert_eq!(f64::from(f).to_bits(), want.to_bits(), "{x:#06x}");
		}
	}

	/// A real frame: a station on channel 9 as a tracker received it in a recording's sync lines, 17 zeros and the
	/// preamble's one, then 25 words of 16 bits and a sync bit. Its id is a made-up one (1a2b3c4d, and the checksum
	/// that goes with it); every other bit is as received
	const FRAME: [&str; 6] = [
		"000000000000000001",
		"0010101100000000100001000000010101010011010011110010010101100011010100000000000000001",
		"0100011110100000101000101101010101100010010010100110000000100001111111101000001101001",
		"1001001000110100111111111011111111011000110110111010011111000001100100111110110001111",
		"1001011100011110110011010100010011000000001100100110011110010011100100111011110010111",
		"1011000011001101110101110001111111000100110000000010111000010110111110001010101000011",
	];

	#[test]
	fn a_real_packet() {
		let bits: Vec<i8> = FRAME.concat().bytes().map(|c| (c - b'0') as i8).collect();
		let mut d = OotxDecoder::new(7.034201);
		let events = pump(&mut d, &bits);
		assert_eq!(events.len(), 2, "{events:?}");
		assert_eq!(events[0], OotxEvent::Error("Preamble found"));
		let OotxEvent::Packet(p) = &events[1] else { panic!("{events:?}") };
		assert_eq!((p.length, p.crc32), (43, 0xa18ab770));
		let data: Vec<u8> = (0..43)
			.map(|i| {
				let hex = "080a4d3c2b1a000047a045aa8929010fe8349234ff7f636e3e0c3ec7971e9a8900c93c9c3bcbb0cdae3f13";
				u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap()
			})
			.collect();
		assert_eq!(p.data, data);
		assert_eq!((d.stats.bits_seen, d.stats.packets_found, d.stats.used_bytes, d.stats.bad_crcs), (443, 1, 43, 0));

		// every field as the C decodes it (its driver's output for this frame), the floats to the bit
		let v = LighthouseInfoV15::new(&p.data).unwrap();
		assert_eq!((v.fw_version, v.id, v.sys_unlock_count, v.ootx_model), (2568, 0x1a2b3c4d, 1, 15));
		assert_eq!((v.accel_dir, v.mode_current, v.sys_faults, v.nonce), ([-1, 127, 99], 137, 0, 4927));
		assert_eq!(v.mode_current & 0x7f, 9);
		let b = |f: [f32; 2]| f.map(f32::to_bits);
		assert_eq!(b(v.fcal_phase), [0x00000000, 0xbc08e000]);
		assert_eq!(b(v.fcal_tilt), [0xbd48a000, 0x3d312000]);
		assert_eq!(b(v.fcal_curve), [0x3e9d0000, 0x3e924000]);
		assert_eq!(b(v.fcal_gibphase), [0x3fcdc000, 0x3fc18000]);
		assert_eq!(b(v.fcal_gibmag), [0xbaf8e000, 0xbb43c000]);
		assert_eq!(b(v.fcal_ogeephase), [0x3f992000, 0x3f738000]);
		assert_eq!(b(v.fcal_ogeemag), [0xbe196000, 0xbdd9a000]);
		// -0x1.11cp-7 and 0x1.3ap-2 as the C prints them, exactly
		assert_eq!((f64::from(v.fcal_phase[1]), f64::from(v.fcal_curve[0])), (-0.00835418701171875, 0.306640625));
	}

	/// a frame as a station sends it: the preamble, then the length, the payload (padded to even) and the CRC, in
	/// 16-bit words of little-endian bytes sent top bit first, a sync bit after each word
	fn frame(payload: &[u8], crc: u32) -> Vec<i8> {
		let mut bytes = (payload.len() as u16).to_le_bytes().to_vec();
		bytes.extend_from_slice(payload);
		if payload.len() % 2 == 1 {
			bytes.push(0);
		}
		bytes.extend_from_slice(&crc.to_le_bytes());
		let mut bits = vec![0; 17];
		bits.push(1);
		for word in bytes.chunks(2) {
			for b in word {
				bits.extend((0..8).rev().map(|k| ((b >> k) & 1) as i8));
			}
			bits.push(1);
		}
		bits
	}

	fn pump(d: &mut OotxDecoder, bits: &[i8]) -> Vec<OotxEvent> {
		bits.iter().filter_map(|&b| d.pump_bit(b)).collect()
	}

	#[test]
	fn frames_and_their_faults() {
		let payload: Vec<u8> = (0..43u8).map(|i| i.wrapping_mul(37) ^ 0x5a).collect();
		let good = crc32(&payload);
		let packet = OotxPacket { length: 43, data: payload.clone(), crc32: good };

		let mut d = OotxDecoder::new(1.5);
		assert_eq!(
			pump(&mut d, &frame(&payload, good)),
			[OotxEvent::Error("Preamble found"), OotxEvent::Packet(packet)]
		);
		assert_eq!((d.stats.packets_found, d.stats.used_bytes, d.stats.bad_crcs), (1, 43, 0));

		let bad = OotxPacket { length: 43, data: payload.clone(), crc32: good ^ 1 };
		let events = pump(&mut d, &frame(&payload, good ^ 1));
		assert_eq!(events[1], OotxEvent::BadCrc { packet: bad, crc: good });

		// word w of a frame is bits 18 + 17 w .. 18 + 17 w + 15, its sync bit the one after

		// a zero sync bit drops the frame, unless the caller asked to ignore it
		let mut bits = frame(&payload, good);
		bits[18 + 17 * 2 + 16] = 0;
		let events = pump(&mut d, &bits);
		assert_eq!(events, [OotxEvent::Error("Preamble found"), OotxEvent::Error("OOTX Decoder: Bad sync bit")]);
		d.ignore_sync_bit_error = 1;
		let events = pump(&mut d, &bits);
		assert_eq!(events[1], OotxEvent::Error("OOTX Decoder: Ignoring bad sync bit"));
		assert!(matches!(events[2], OotxEvent::Packet(_)));

		// sally's check: no packet shorter than a station id
		let events = pump(&mut d, &frame(&[], 0));
		assert_eq!(events[1], OotxEvent::Error("OOTX Decoder: ignoring a packet too short to hold a station id"));
		assert_eq!((d.stats.packets_found, d.stats.bad_crcs, d.stats.bad_sync_bits), (2, 2, 1));

		// a -1 leaves the bit the buffer had: the frame once, then again with byte 5 (word 2's second byte) unseen
		assert!(matches!(pump(&mut d, &frame(&payload, good))[1], OotxEvent::Packet(_)));
		let mut bits = frame(&payload, good);
		for k in 0..8 {
			bits[18 + 17 * 2 + 8 + k] = -1;
		}
		let events = pump(&mut d, &bits);
		assert!(matches!(events[1], OotxEvent::Packet(_)), "{events:?}");
		// five whole frames of 50 bytes, two stopped after 6
		assert_eq!((d.stats.guess_bits, d.stats.package_bits), (8, 5 * 400 + 2 * 48));
	}
}
