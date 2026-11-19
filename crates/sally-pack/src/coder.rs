//! The packed files' entropy coder: a binary range coder whose probabilities adapt as it goes (the scheme LZMA
//! uses, with wider probabilities that adapt at two rates), and the two shapes every model is built from: a tree of
//! bits for a small symbol, and a signed number coded by its length.
//!
//! A model is written once, against `Io`: the packer's `Encoder` takes each value and writes it, the unpacker's
//! `Decoder` ignores the value it is handed and returns the one it reads. Both walk the same code with the same
//! state, which is what keeps them in step.

/// What a model codes against. `bit` and `raw` take the value to write and return the value written (packing) or
/// read (unpacking)
pub trait Io {
	/// packing (the values handed in are the real ones) or unpacking (they are whatever the caller had)
	const PACKING: bool;
	/// one bit under an adaptive probability
	fn bit(&mut self, p: &mut Prob, bit: bool) -> bool;
	/// `bits` bits (32 at most) as they are: for what no model can shorten
	fn raw(&mut self, bits: u32, value: u32) -> u32;
	/// whether more has been read than there was (unpacking a block that is damaged): what is read from then on is
	/// nothing's, and a long run of it can be left unread
	fn overrun(&self) -> bool {
		false
	}
}

/// The probability that a bit is 0, in 1/65536, as the mean of two estimates: one that follows the last few dozen
/// bits and one that follows the last hundred or so. The mix is closer than either where a source drifts
#[derive(Clone, Copy)]
pub struct Prob {
	fast: u16,
	slow: u16,
}

impl Prob {
	pub const fn new() -> Prob {
		Prob { fast: 1 << 15, slow: 1 << 15 }
	}

	// never 0 and never 65536: the estimates stop short of both ends
	fn zero(&self) -> u32 {
		(self.fast as u32 + self.slow as u32) >> 1
	}

	fn update(&mut self, bit: bool) {
		if bit {
			self.fast -= self.fast >> 5;
			self.slow -= self.slow >> 7;
		} else {
			self.fast += (u16::MAX - self.fast) >> 5;
			self.slow += (u16::MAX - self.slow) >> 7;
		}
	}
}

impl Default for Prob {
	fn default() -> Prob {
		Prob::new()
	}
}

/// `n` probabilities, all even
pub fn probs(n: usize) -> Vec<Prob> {
	vec![Prob::new(); n]
}

const TOP: u32 = 1 << 24;

/// The range coder's writing half. A block is coded from `new` to `finish`, which gives its bytes
pub struct Encoder {
	low: u64,
	range: u32,
	cache: u8,
	cache_size: u64,
	out: Vec<u8>,
}

impl Encoder {
	pub fn new() -> Encoder {
		Encoder { low: 0, range: u32::MAX, cache: 0, cache_size: 1, out: Vec::new() }
	}

	/// The bytes written so far (the block's size, near enough)
	pub fn size(&self) -> usize {
		self.out.len()
	}

	// the top byte of `low` goes out once no carry can reach it any more; a run of 0xff waits for the carry
	fn shift_low(&mut self) {
		if (self.low as u32) < 0xff00_0000 || (self.low >> 32) != 0 {
			let carry = (self.low >> 32) as u8;
			let mut byte = self.cache;
			loop {
				self.out.push(byte.wrapping_add(carry));
				byte = 0xff;
				self.cache_size -= 1;
				if self.cache_size == 0 {
					break;
				}
			}
			self.cache = (self.low >> 24) as u8;
		}
		self.cache_size += 1;
		self.low = (self.low & 0x00ff_ffff) << 8;
	}

	/// The block's bytes, the coder ready for the next block
	pub fn finish(&mut self) -> Vec<u8> {
		for _ in 0..5 {
			self.shift_low();
		}
		let out = std::mem::take(&mut self.out);
		*self = Encoder::new();
		out
	}
}

impl Default for Encoder {
	fn default() -> Encoder {
		Encoder::new()
	}
}

impl Io for Encoder {
	const PACKING: bool = true;

	fn bit(&mut self, p: &mut Prob, bit: bool) -> bool {
		let bound = ((self.range as u64 * p.zero() as u64) >> 16) as u32;
		if bit {
			self.low += bound as u64;
			self.range -= bound;
		} else {
			self.range = bound;
		}
		p.update(bit);
		while self.range < TOP {
			self.range <<= 8;
			self.shift_low();
		}
		bit
	}

	fn raw(&mut self, bits: u32, value: u32) -> u32 {
		for i in (0..bits).rev() {
			self.range >>= 1;
			if (value >> i) & 1 != 0 {
				self.low += self.range as u64;
			}
			while self.range < TOP {
				self.range <<= 8;
				self.shift_low();
			}
		}
		value
	}
}

/// The range coder's reading half, over one block's bytes. Past their end it reads zeros: a block cut short or
/// damaged decodes to something, and the block's checksum is what says so
pub struct Decoder<'a> {
	data: &'a [u8],
	at: usize,
	range: u32,
	code: u32,
}

impl<'a> Decoder<'a> {
	pub fn new(data: &'a [u8]) -> Decoder<'a> {
		let mut d = Decoder { data, at: 0, range: u32::MAX, code: 0 };
		// the encoder's first byte is its empty cache
		for _ in 0..5 {
			d.code = (d.code << 8) | d.next() as u32;
		}
		d
	}

	fn next(&mut self) -> u8 {
		let b = self.data.get(self.at).copied().unwrap_or(0);
		self.at += 1;
		b
	}

	/// Whether the decoder read past the block's bytes: more was asked of it than the block holds
	pub fn overrun(&self) -> bool {
		self.at > self.data.len()
	}
}

impl Io for Decoder<'_> {
	const PACKING: bool = false;

	fn bit(&mut self, p: &mut Prob, _: bool) -> bool {
		let bound = ((self.range as u64 * p.zero() as u64) >> 16) as u32;
		let bit = self.code >= bound;
		if bit {
			self.code -= bound;
			self.range -= bound;
		} else {
			self.range = bound;
		}
		p.update(bit);
		while self.range < TOP {
			self.range <<= 8;
			self.code = (self.code << 8) | self.next() as u32;
		}
		bit
	}

	fn overrun(&self) -> bool {
		Decoder::overrun(self)
	}

	fn raw(&mut self, bits: u32, _: u32) -> u32 {
		let mut value = 0;
		for _ in 0..bits {
			self.range >>= 1;
			let bit = self.code >= self.range;
			if bit {
				self.code -= self.range;
			}
			value = (value << 1) | bit as u32;
			while self.range < TOP {
				self.range <<= 8;
				self.code = (self.code << 8) | self.next() as u32;
			}
		}
		value
	}
}

/// A symbol of `bits` bits, its bits coded from the top, each under the bits before it: `probs` holds `1 << bits`
/// probabilities (one context's worth)
pub fn tree<I: Io>(io: &mut I, probs: &mut [Prob], bits: u32, value: u32) -> u32 {
	let mut node = 1usize;
	for i in (0..bits).rev() {
		let bit = io.bit(&mut probs[node], (value >> i) & 1 != 0);
		node = (node << 1) | bit as usize;
	}
	node as u32 - (1 << bits)
}

/// A symbol coded under one of several contexts: a `tree` per context
pub struct Symbol {
	bits: u32,
	probs: Vec<Prob>,
}

impl Symbol {
	pub fn new(bits: u32, contexts: usize) -> Symbol {
		Symbol { bits, probs: probs(contexts << bits) }
	}

	pub fn code<I: Io>(&mut self, io: &mut I, context: usize, value: u32) -> u32 {
		let at = context << self.bits;
		tree(io, &mut self.probs[at..at + (1 << self.bits)], self.bits, value)
	}
}

/// A signed number that is usually small and now and then is not: how many bits its size takes (0 for zero) as a
/// symbol under the caller's context, its sign, the two bits below its leading one under that length, and the rest as
/// they are. Numbers up to 63 bits
pub struct Number {
	length: Symbol,
	sign: Vec<Prob>,
	top: Vec<Prob>,
}

impl Number {
	pub fn new(contexts: usize) -> Number {
		Number { length: Symbol::new(6, contexts), sign: probs(contexts), top: probs(64 * 4) }
	}

	pub fn code<I: Io>(&mut self, io: &mut I, context: usize, value: i64) -> i64 {
		let size = value.unsigned_abs();
		let length = self.length.code(io, context, 64 - size.leading_zeros());
		if length == 0 {
			return 0;
		}
		let negative = io.bit(&mut self.sign[context], value < 0);
		// the bits under the leading one, from the top: two under a model, the rest as they are
		let mut rest = length - 1;
		let mut out = 1u64;
		let mut node = 1usize;
		while rest > 0 && node < 4 {
			rest -= 1;
			let bit = io.bit(&mut self.top[length as usize * 4 + node], (size >> rest) & 1 != 0);
			out = (out << 1) | bit as u64;
			node = (node << 1) | bit as usize;
		}
		while rest > 0 {
			let n = rest.min(32);
			rest -= n;
			out = (out << n) | io.raw(n, (size >> rest) as u32 & (u32::MAX >> (32 - n))) as u64;
		}
		if negative {
			(out as i64).wrapping_neg()
		} else {
			out as i64
		}
	}

	/// The length `code` gave the last number its context was asked with: a number's size as the next one's context
	pub fn length_of(value: i64) -> usize {
		(64 - value.unsigned_abs().leading_zeros()) as usize
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A small generator of test values (xorshift): the same on every run
	pub(crate) struct Rng(pub u64);

	impl Rng {
		pub fn next(&mut self) -> u64 {
			self.0 ^= self.0 << 13;
			self.0 ^= self.0 >> 7;
			self.0 ^= self.0 << 17;
			self.0
		}
	}

	#[test]
	fn bits_symbols_and_numbers_come_back() {
		let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
		// skewed bits, raw fields of every width, symbols and numbers of every size, mixed as models mix them
		let n = 20000;
		let bits: Vec<bool> = (0..n).map(|_| rng.next().is_multiple_of(10)).collect();
		let raws: Vec<(u32, u32)> = (0..n)
			.map(|_| {
				let w = (rng.next() % 33) as u32;
				(w, if w == 0 { 0 } else { rng.next() as u32 & (u32::MAX >> (32 - w)) })
			})
			.collect();
		let syms: Vec<u32> = (0..n).map(|_| (rng.next() % 32) as u32).collect();
		let nums: Vec<i64> = (0..n)
			.map(|i| match i % 5 {
				0 => 0,
				1 => (rng.next() % 16) as i64 - 8,
				2 => rng.next() as i64 >> (rng.next() % 64),
				3 => i64::MIN + 1,
				_ => i64::MAX,
			})
			.collect();

		let mut enc = Encoder::new();
		let (mut p, mut s, mut num) = (Prob::new(), Symbol::new(5, 3), Number::new(2));
		for i in 0..n {
			enc.bit(&mut p, bits[i]);
			enc.raw(raws[i].0, raws[i].1);
			s.code(&mut enc, i % 3, syms[i]);
			num.code(&mut enc, i % 2, nums[i]);
		}
		let data = enc.finish();

		let mut dec = Decoder::new(&data);
		let (mut p, mut s, mut num) = (Prob::new(), Symbol::new(5, 3), Number::new(2));
		for i in 0..n {
			assert_eq!(dec.bit(&mut p, false), bits[i], "bit {i}");
			assert_eq!(dec.raw(raws[i].0, 0), raws[i].1, "raw {i}");
			assert_eq!(s.code(&mut dec, i % 3, 0), syms[i], "symbol {i}");
			assert_eq!(num.code(&mut dec, i % 2, 0), nums[i], "number {i}");
		}
		assert!(!dec.overrun());
	}

	#[test]
	fn a_carry_runs_through_a_row_of_ff() {
		// near-certain bits that come out the other way: long runs of 0xff with carries into them
		let mut rng = Rng(7);
		let bits: Vec<bool> = (0..200000).map(|_| !rng.next().is_multiple_of(300)).collect();
		let mut enc = Encoder::new();
		let mut p = [Prob::new(), Prob::new()];
		for (i, &b) in bits.iter().enumerate() {
			enc.bit(&mut p[i & 1], b);
		}
		let data = enc.finish();
		// far fewer bytes than bits / 8: the model is in use
		assert!(data.len() < bits.len() / 40, "{}", data.len());
		let mut dec = Decoder::new(&data);
		let mut p = [Prob::new(), Prob::new()];
		for (i, &b) in bits.iter().enumerate() {
			assert_eq!(dec.bit(&mut p[i & 1], false), b, "bit {i}");
		}
	}

	#[test]
	fn the_coder_starts_over_after_a_block() {
		let mut enc = Encoder::new();
		let mut p = Prob::new();
		for i in 0..100 {
			enc.bit(&mut p, i % 3 == 0);
		}
		let first = enc.finish();
		for i in 0..100 {
			enc.bit(&mut p, i % 7 == 0);
		}
		let second = enc.finish();
		// the model carried over, the coder did not
		let mut p = Prob::new();
		let mut dec = Decoder::new(&first);
		for i in 0..100 {
			assert_eq!(dec.bit(&mut p, false), i % 3 == 0);
		}
		let mut dec = Decoder::new(&second);
		for i in 0..100 {
			assert_eq!(dec.bit(&mut p, false), i % 7 == 0);
		}
	}
}
