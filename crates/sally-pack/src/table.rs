//! A table of numbers as text, packed: the driver's pose log (a csv: a line of column names, then a row a frame and
//! device, some ninety numbers each) in a small fraction of its bytes, and back to the same bytes.
//!
//! A row says little that the rows before it did not: a device's pose a frame on is where it was plus how it was
//! moving, a number that belongs to the frame is the same for every device of it, and many stay as they were. A
//! number in such a table is written with a fixed count of decimals, so it is a whole number of its last digit. A
//! packed table holds, for each number, how far it is from the best of four guesses, through the entropy coder
//! (`coder`): the device's last value, that value carried on by its last step, the value in the line before, and
//! what the row's own numbers make it where the column names say they belong together (the last number of a unit
//! quaternion or of a rotation matrix's row, a rotation matrix's third row: `Shape`). Which guess is the best is not
//! written: both sides keep count of how each has done for that column of that device.
//!
//! Nothing is lost: a packed table unpacks to the text's own bytes, whatever they were. The first line (the column
//! names) is kept as its bytes and says how many columns there are and which one names the row's device (`serial`,
//! else `id`); a line with another count of fields is kept as its bytes too, and a field that is not a number as
//! printf's `%d` or `%.Nf` writes one (`nan`, a name, an exponent) is kept as its bytes within its row.
//!
//! The file:
//!
//! ```text
//! header   0x89 "CSVZ" CR LF 0x1a, the format's version (1), three zero bytes
//! block    lines: u32, bytes: u32 (little-endian), the coded lines (`bytes` of them), the CRC-32 of all before it
//!          in the block
//! ...
//! ```
//!
//! A block is whole or it is not there: a table being written, or one whose writer was stopped, reads up to its
//! last whole block. The coder starts over with each block; what it has learned, and what it knows of each device,
//! carries on, so the blocks are read in order from the first.
//!
//! - `PackWriter`: text in whatever pieces, a packed file out
//! - `Unpacker`: a packed file read as the text it holds
//! - `open`: a table of either form as its text, for everything that reads one
//! - `stem`: a table's file name without its end, whichever of the two it has

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Seek, SeekFrom, Write};
use std::path::Path;

use sally_math::crc::crc32;

use crate::coder::{Decoder, Encoder, Io, Number, Prob, Symbol};

/// A packed table's first bytes: no text starts with them, and a copy that changed line ends does not keep them
pub const MAGIC: &[u8; 8] = b"\x89CSVZ\r\n\x1a";
const VERSION: u8 = 1;
const HEADER: usize = 12;
/// A block's own header: its lines, its bytes
const BLOCK_HEADER: usize = 8;

/// The end of a table's name as text, and packed
pub const TEXT_END: &str = ".csv";
pub const PACKED_END: &str = ".csvz";

/// A block ends once it holds this many lines (or when the writer is told to end it: `PackWriter::end`)
const BLOCK_LINES: u32 = 4096;
/// The most a block is read as (a damaged length must not ask for the machine's memory)
const MAX_BLOCK: u32 = 1 << 26;
const MAX_TEXT: i64 = 1 << 26;

/// The most columns a table is packed by; one with more is kept as its bytes
const MAX_COLUMNS: usize = 1024;
/// The devices a table can name; a row of one more is kept as its bytes
const STREAMS: usize = 255;
/// The most decimals, and digits in all, a field is taken for a number with
const MAX_DECIMALS: usize = 15;
const MAX_DIGITS: usize = 18;

/// How a line ends
const CR_LF: u8 = 0;
const LF: u8 = 1;
const NO_END: u8 = 2;

/// The guesses at a number: the device's last value, that value carried on by its last step, the line before's, and
/// what the row's own numbers before it make it (`Shape`)
const GUESSES: usize = 4;
/// The largest size a number of a row is taken at for that last guess (a number read from text is below it)
const LARGEST: i64 = 1_000_000_000_000_000_000;
/// A number's contexts: the size of the misses of late of the guess it is coded against (their bits, up to this
/// many)
const MISSES: usize = 12;

/// One field of a row, between its text and the coder: a number as a whole number of its last digit, or its bytes
#[derive(Clone, Default)]
struct Field {
	number: bool,
	decimals: u8,
	/// "-0.000": the sign of a number that is zero
	minus_zero: bool,
	value: i64,
	word: Vec<u8>,
}

/// One line, between its text and the coder: a row is its device and its fields; any other is its bytes
#[derive(Default)]
struct Line {
	end: u8,
	is_row: bool,
	/// the device's place among those seen, or `STREAMS` for one not seen before (then `key` is its name)
	stream: usize,
	key: Vec<u8>,
	fields: Vec<Field>,
	text: Vec<u8>,
}

/// What is kept of one column of one device
#[derive(Clone, Default)]
struct Cell {
	number: bool,
	decimals: u8,
	minus_zero: bool,
	/// its last value, and the one before
	last: i64,
	before: i64,
	/// how each guess has done here of late: the bits of its misses, the newest weighing most, in 1/128 of a bit
	missed: [u16; GUESSES],
	word: Vec<u8>,
}

/// What a column's number is to the numbers before it in its row, as the column names say (a guess like the others:
/// where it does not hold, it is not used)
#[derive(Clone, Copy)]
enum Shape {
	None,
	/// the last of a unit vector's or a unit quaternion's numbers: the columns this many before it are the others
	/// (`m00,m01,m02`, `qw,qx,qy,qz`)
	Unit(usize),
	/// the third row of a rotation matrix: the cross product of the two rows before it, which start this many
	/// columns before this one
	Cross(usize),
}

/// What the column names say of each column: nine in a row that end in 00, 01, 02, 10, ... 22 after the same start
/// are a rotation matrix, four that end in qw, qx, qy, qz a quaternion
fn shapes(names: &[&[u8]]) -> Vec<Shape> {
	let run = |at: usize, ends: &[&[u8]]| {
		let Some(first) = names[at].strip_suffix(ends[0]) else { return false };
		at + ends.len() <= names.len()
			&& ends.iter().zip(&names[at..]).all(|(end, name)| name.strip_suffix(*end) == Some(first))
	};
	let mut shapes = vec![Shape::None; names.len()];
	for at in 0..names.len() {
		if run(at, &[b"00", b"01", b"02", b"10", b"11", b"12", b"20", b"21", b"22"]) {
			shapes[at + 2] = Shape::Unit(2);
			shapes[at + 5] = Shape::Unit(2);
			for k in 6..9 {
				shapes[at + k] = Shape::Cross(k);
			}
		} else if run(at, &[b"qw", b"qx", b"qy", b"qz"]) {
			shapes[at + 3] = Shape::Unit(3);
		}
	}
	shapes
}

/// `n / d` to the nearest whole number (`d` above zero)
fn nearest(n: i128, d: i128) -> i64 {
	let half = d / 2;
	((if n < 0 { n - half } else { n + half }) / d).clamp(-(LARGEST as i128), LARGEST as i128) as i64
}

/// The square root of `n` to the nearest whole number (0 for one below zero)
fn root(n: i128) -> i64 {
	if n <= 0 {
		return 0;
	}
	// from the floating point root, put right by whole numbers: the same on every machine
	let mut r = (n as f64).sqrt() as i128;
	while r * r > n {
		r -= 1;
	}
	while (r + 1) * (r + 1) <= n {
		r += 1;
	}
	(if n - r * r > r { r + 1 } else { r }) as i64
}

impl Shape {
	/// The guess at the number in column `at`: `row` holds the row's numbers before it, `one` is 1 as the column
	/// writes it (10 to its decimals), `last` the device's last value there (a root's sign)
	fn guess(self, row: &[i64], at: usize, one: i128, last: i64) -> i64 {
		match self {
			Shape::Unit(others) if at >= others => {
				let squares: i128 = row[at - others..at].iter().map(|&v| v as i128 * v as i128).sum();
				let size = root(one * one - squares);
				if last < 0 {
					-size
				} else {
					size
				}
			}
			Shape::Cross(back) if at >= back => {
				let (a, b) = (&row[at - back..at - back + 3], &row[at - back + 3..at - back + 6]);
				let (i, j) = ((back - 5) % 3, (back - 4) % 3);
				nearest(a[i] as i128 * b[j] as i128 - a[j] as i128 * b[i] as i128, one)
			}
			_ => last,
		}
	}
}

/// 10 to the `decimals`
fn one(decimals: u8) -> i128 {
	10i128.pow(decimals.min(MAX_DECIMALS as u8) as u32)
}

/// A column's models, the same for every device
struct Column {
	number: Number,
	minus_zero: [Prob; 2],
	changed: Prob,
	same_word: Prob,
	/// the value in the line before, whichever device's
	above: i64,
	shape: Shape,
}

impl Column {
	fn new(shape: Shape) -> Column {
		Column {
			shape,
			number: Number::new(MISSES),
			minus_zero: [Prob::new(); 2],
			changed: Prob::new(),
			same_word: Prob::new(),
			above: 0,
		}
	}
}

/// What the packer and the unpacker both keep as they go through a table's lines
struct State {
	is_row: [Prob; 2],
	end: Symbol,
	stream: Symbol,
	same_shape: Prob,
	text_len: Number,
	text: Symbol,

	/// the columns, from the first line on (none before it, or when the first line is not a row of names)
	columns: Vec<Column>,
	/// the column that names a row's device
	key: Option<usize>,
	first: bool,
	/// the devices: their names, and their cells by column
	names: Vec<Vec<u8>>,
	streams: Vec<Vec<Cell>>,
	/// the numbers of the row in hand, by column (0 for a field that is not one)
	row: Vec<i64>,
	last_is_row: bool,
	last_end: u8,
	last_stream: usize,
}

fn bits_of(value: i64) -> u32 {
	64 - value.unsigned_abs().leading_zeros()
}

/// The fields of a line: what lies between its commas
fn fields(text: &[u8]) -> impl Iterator<Item = &[u8]> {
	text.split(|&c| c == b',')
}

impl State {
	fn new() -> State {
		State {
			is_row: [Prob::new(); 2],
			end: Symbol::new(2, 4),
			stream: Symbol::new(8, STREAMS + 1),
			same_shape: Prob::new(),
			text_len: Number::new(1),
			text: Symbol::new(8, 1),
			columns: Vec::new(),
			key: None,
			first: true,
			names: Vec::new(),
			streams: Vec::new(),
			row: Vec::new(),
			last_is_row: false,
			last_end: LF,
			last_stream: STREAMS,
		}
	}

	/// Bytes kept as they are: a line that is not a row, a device's name, a field that is not a number
	fn bytes<I: Io>(io: &mut I, text_len: &mut Number, symbol: &mut Symbol, text: &mut Vec<u8>) {
		let len = text_len.code(io, 0, text.len() as i64).clamp(0, MAX_TEXT) as usize;
		if !I::PACKING {
			text.clear();
		}
		// a piece at a time: a damaged block can ask for any length, and once it has given all it holds the rest is
		// left unread (the block is found out by what it holds)
		let mut done = 0;
		while done < len && !io.overrun() {
			let piece = (len - done).min(1024);
			if !I::PACKING {
				text.resize(done + piece, 0);
			}
			for b in &mut text[done..done + piece] {
				*b = symbol.code(io, 0, *b as u32) as u8;
			}
			done += piece;
		}
	}

	/// The first line: how many columns the table has, and which names a row's device
	fn names_line(&mut self, text: &[u8]) {
		let names: Vec<&[u8]> = fields(text).collect();
		if !(2..=MAX_COLUMNS).contains(&names.len()) {
			return;
		}
		self.key = names.iter().position(|n| *n == b"serial").or_else(|| names.iter().position(|n| *n == b"id"));
		self.columns = shapes(&names).into_iter().map(Column::new).collect();
		self.row = vec![0; names.len()];
	}

	fn line<I: Io>(&mut self, io: &mut I, l: &mut Line) {
		if self.columns.is_empty() {
			l.is_row = false;
		} else {
			l.is_row = io.bit(&mut self.is_row[self.last_is_row as usize], l.is_row);
			self.last_is_row = l.is_row;
		}
		l.end = self.end.code(io, self.last_end as usize, l.end as u32) as u8;
		self.last_end = l.end;
		if !l.is_row {
			Self::bytes(io, &mut self.text_len, &mut self.text, &mut l.text);
			if std::mem::take(&mut self.first) {
				self.names_line(&l.text);
			}
			return;
		}

		// the device: one seen before by its place, a new one by its name. Without a column that names it, one device
		let stream = match self.key {
			Some(_) => self.stream.code(io, self.last_stream, l.stream as u32) as usize,
			None if self.streams.is_empty() => STREAMS,
			None => 0,
		};
		let stream = if stream < self.streams.len() {
			stream
		} else {
			if self.key.is_some() {
				Self::bytes(io, &mut self.text_len, &mut self.text, &mut l.key);
			}
			if self.streams.len() < STREAMS {
				// what kind each of its fields is: as the line before's device has them
				let mut cells = match self.streams.get(self.last_stream) {
					Some(above) => above
						.iter()
						.map(|c| Cell { number: c.number, decimals: c.decimals, ..Cell::default() })
						.collect(),
					None => vec![Cell::default(); self.columns.len()],
				};
				cells.resize(self.columns.len(), Cell::default());
				self.names.push(l.key.clone());
				self.streams.push(cells);
			}
			self.streams.len() - 1
		};
		l.stream = stream;
		self.last_stream = stream;
		if !I::PACKING {
			l.fields.resize(self.columns.len(), Field::default());
		}
		let cells = &mut self.streams[stream];

		// what kind each field is: nearly always as in the device's last row
		let same = l.fields.iter().zip(cells.iter()).enumerate().all(|(c, (f, cell))| {
			Some(c) == self.key || (f.number == cell.number && (!f.number || f.decimals == cell.decimals))
		});
		let same = io.bit(&mut self.same_shape, same);
		for (c, ((f, cell), column)) in
			l.fields.iter_mut().zip(cells.iter_mut()).zip(self.columns.iter_mut()).enumerate()
		{
			self.row[c] = 0;
			if Some(c) == self.key {
				continue;
			}
			if !same {
				let changed = f.number != cell.number || (f.number && f.decimals != cell.decimals);
				if io.bit(&mut column.changed, changed) {
					cell.number = io.raw(1, f.number as u32) != 0;
					cell.decimals = io.raw(4, f.decimals as u32) as u8;
				}
			}
			f.number = cell.number;
			f.decimals = cell.decimals;
			if !f.number {
				let same_word = io.bit(&mut column.same_word, f.word == cell.word);
				if same_word {
					if !I::PACKING {
						f.word.clone_from(&cell.word);
					}
				} else {
					Self::bytes(io, &mut self.text_len, &mut self.text, &mut f.word);
					cell.word.clone_from(&f.word);
				}
				continue;
			}

			// the number: how far it is from the guess that has missed the least here
			let step = cell.last.wrapping_sub(cell.before);
			let shaped = column.shape.guess(&self.row, c, one(f.decimals), cell.last);
			let guesses = [cell.last, cell.last.wrapping_add(step), column.above, shaped];
			let mut best = 0;
			for k in 1..GUESSES {
				if cell.missed[k] < cell.missed[best] {
					best = k;
				}
			}
			let context = ((cell.missed[best] + 64) >> 7).min(MISSES as u16 - 1) as usize;
			let miss = column.number.code(io, context, f.value.wrapping_sub(guesses[best]));
			f.value = guesses[best].wrapping_add(miss);
			f.minus_zero = f.value == 0 && io.bit(&mut column.minus_zero[cell.minus_zero as usize], f.minus_zero);
			for (missed, guess) in cell.missed.iter_mut().zip(guesses) {
				let bits = bits_of(f.value.wrapping_sub(guess)) as u16;
				*missed = *missed - (*missed >> 3) + (bits << 4);
			}
			cell.minus_zero = f.minus_zero;
			cell.before = cell.last;
			cell.last = f.value;
			column.above = f.value;
			self.row[c] = f.value.clamp(-LARGEST, LARGEST);
		}
	}
}

/// A field as printf's `%d` or `%.Nf` writes a number: its value as a whole number of its last digit, its decimals,
/// and whether it is a zero with a minus sign. None for anything that would not be written back the same
fn number(text: &[u8]) -> Option<(i64, u8, bool)> {
	let (minus, digits) = match text {
		[b'-', rest @ ..] => (true, rest),
		_ => (false, text),
	};
	let whole = digits.iter().take_while(|c| c.is_ascii_digit()).count();
	if whole == 0 || (whole > 1 && digits[0] == b'0') {
		return None;
	}
	let decimals = match &digits[whole..] {
		[] => 0,
		[b'.', rest @ ..] if !rest.is_empty() && rest.len() <= MAX_DECIMALS && rest.iter().all(u8::is_ascii_digit) => {
			rest.len()
		}
		_ => return None,
	};
	if whole + decimals > MAX_DIGITS {
		return None;
	}
	let mut value = 0i64;
	for &c in digits.iter().filter(|c| c.is_ascii_digit()) {
		value = value * 10 + (c - b'0') as i64;
	}
	Some((if minus { -value } else { value }, decimals as u8, minus && value == 0))
}

/// The number `number` read, as its text
fn write_number(out: &mut Vec<u8>, value: i64, decimals: u8, minus_zero: bool) {
	if value < 0 || minus_zero {
		out.push(b'-');
	}
	// its digits from the last, at least one before the point (19 at most, and 15 decimals: they fit)
	let decimals = (decimals as usize).min(MAX_DECIMALS);
	let mut digits = [b'0'; 20];
	let mut size = value.unsigned_abs();
	let mut at = digits.len();
	loop {
		at -= 1;
		digits[at] = b'0' + (size % 10) as u8;
		size /= 10;
		if size == 0 && digits.len() - at > decimals {
			break;
		}
	}
	let point = digits.len() - decimals;
	out.extend_from_slice(&digits[at..point]);
	if decimals > 0 {
		out.push(b'.');
		out.extend_from_slice(&digits[point..]);
	}
}

/// Lines in, blocks out. `line` takes each line of a table with its newline (the last may have none); `block` gives
/// the lines taken since the last block as a block, and `full` says when to ask for one
pub struct Packer {
	state: State,
	coder: Encoder,
	/// the devices' places (`State::names` the other way round)
	places: HashMap<Vec<u8>, usize>,
	line: Line,
	lines: u32,
}

impl Default for Packer {
	fn default() -> Packer {
		Packer::new()
	}
}

impl Packer {
	pub fn new() -> Packer {
		Packer { state: State::new(), coder: Encoder::new(), places: HashMap::new(), line: Line::default(), lines: 0 }
	}

	/// What a packed table starts with
	pub fn header(&self) -> [u8; HEADER] {
		let mut h = [0; HEADER];
		h[..8].copy_from_slice(MAGIC);
		h[8] = VERSION;
		h
	}

	/// A row of the table, into `self.line`: whether `text` is one (as many fields as the table has columns, its
	/// device one there is a place for)
	fn row(&mut self, text: &[u8]) -> bool {
		let columns = self.state.columns.len();
		if columns == 0 || fields(text).count() != columns {
			return false;
		}
		let l = &mut self.line;
		l.fields.resize(columns, Field::default());
		for (c, (f, text)) in l.fields.iter_mut().zip(fields(text)).enumerate() {
			if Some(c) == self.state.key {
				l.stream = match self.places.get(text) {
					Some(&place) => place,
					None if self.places.len() < STREAMS => {
						self.places.insert(text.to_vec(), self.places.len());
						l.key.clear();
						l.key.extend_from_slice(text);
						STREAMS
					}
					None => return false,
				};
				continue;
			}
			match number(text) {
				Some((value, decimals, minus_zero)) => {
					f.number = true;
					(f.value, f.decimals, f.minus_zero) = (value, decimals, minus_zero);
				}
				None => {
					f.number = false;
					f.word.clear();
					f.word.extend_from_slice(text);
				}
			}
		}
		true
	}

	/// One line of the table: its bytes up to and with its newline, or the table's last bytes without one
	pub fn line(&mut self, line: &[u8]) {
		let (text, end) = match line {
			[text @ .., b'\r', b'\n'] => (text, CR_LF),
			[text @ .., b'\n'] => (text, LF),
			text => (text, NO_END),
		};
		self.line.end = end;
		self.line.is_row = self.row(text);
		if !self.line.is_row {
			self.line.text.clear();
			self.line.text.extend_from_slice(text);
		}
		self.state.line(&mut self.coder, &mut self.line);
		self.lines += 1;
	}

	/// Whether the lines taken since the last block are a block's worth
	pub fn full(&self) -> bool {
		self.lines >= BLOCK_LINES || self.coder.size() >= MAX_BLOCK as usize / 4
	}

	/// The lines taken since the last block, as a block (nothing when there are none)
	pub fn block(&mut self) -> Vec<u8> {
		if self.lines == 0 {
			return Vec::new();
		}
		let coded = self.coder.finish();
		let mut block = Vec::with_capacity(BLOCK_HEADER + coded.len() + 4);
		block.extend_from_slice(&self.lines.to_le_bytes());
		block.extend_from_slice(&(coded.len() as u32).to_le_bytes());
		block.extend_from_slice(&coded);
		block.extend_from_slice(&crc32(&block).to_le_bytes());
		self.lines = 0;
		block
	}
}

/// A packed table written from the table's text, handed over in whatever pieces: each block goes to `out` in one
/// write as soon as it is full, so the file never ends in half a block for longer than a write takes
pub struct PackWriter<W: Write> {
	out: W,
	packer: Packer,
	/// a line begun and not yet ended
	pending: Vec<u8>,
}

impl<W: Write> PackWriter<W> {
	pub fn new(mut out: W) -> io::Result<PackWriter<W>> {
		let packer = Packer::new();
		out.write_all(&packer.header())?;
		Ok(PackWriter { out, packer, pending: Vec::new() })
	}

	fn end_block(&mut self) -> io::Result<()> {
		let block = self.packer.block();
		if block.is_empty() {
			return Ok(());
		}
		self.out.write_all(&block)?;
		self.out.flush()
	}

	/// More of the table's text
	pub fn write(&mut self, mut text: &[u8]) -> io::Result<()> {
		while let Some(i) = text.iter().position(|&c| c == b'\n') {
			let (line, rest) = text.split_at(i + 1);
			if self.pending.is_empty() {
				self.packer.line(line);
			} else {
				self.pending.extend_from_slice(line);
				self.packer.line(&self.pending);
				self.pending.clear();
			}
			text = rest;
			if self.packer.full() {
				self.end_block()?;
			}
		}
		self.pending.extend_from_slice(text);
		Ok(())
	}

	/// What has been handed over written out: a last line without a newline, and the block begun. More text may
	/// still follow (it starts a new line and a new block)
	pub fn end(&mut self) -> io::Result<()> {
		if !self.pending.is_empty() {
			let last = std::mem::take(&mut self.pending);
			self.packer.line(&last);
		}
		self.end_block()
	}

	/// The table's end (`end`), and `out` back
	pub fn finish(mut self) -> io::Result<W> {
		self.end()?;
		Ok(self.out)
	}
}

/// A packed table read as the text it holds. It reads to the last whole block: a table still being written ends
/// there, and so does one that was cut short (`cut` says which it was)
pub struct Unpacker<R: Read> {
	input: R,
	state: State,
	line: Line,
	/// the text of the block read last, and how much of it has been handed on
	text: Vec<u8>,
	given: usize,
	ended: bool,
	cut: bool,
}

/// As much of `buf` as `input` has: less than all of it only at the input's end
fn read_full(input: &mut impl Read, buf: &mut [u8]) -> io::Result<usize> {
	let mut got = 0;
	while got < buf.len() {
		match input.read(&mut buf[got..]) {
			Ok(0) => break,
			Ok(n) => got += n,
			Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
			Err(e) => return Err(e),
		}
	}
	Ok(got)
}

fn damaged(what: &str) -> io::Error {
	io::Error::new(io::ErrorKind::InvalidData, format!("packed table: {what}"))
}

impl<R: Read> Unpacker<R> {
	/// `input` from its first byte: the header is read and checked
	pub fn new(mut input: R) -> io::Result<Unpacker<R>> {
		let mut header = [0; HEADER];
		if read_full(&mut input, &mut header)? != HEADER || &header[..8] != MAGIC {
			return Err(damaged("not one"));
		}
		if header[8] != VERSION {
			return Err(damaged(&format!("version {} (this sally reads version {VERSION})", header[8])));
		}
		Ok(Unpacker {
			input,
			state: State::new(),
			line: Line::default(),
			text: Vec::new(),
			given: 0,
			ended: false,
			cut: false,
		})
	}

	/// Whether the table ended inside a block (its writer was stopped mid-write, or is still writing)
	pub fn cut(&self) -> bool {
		self.cut
	}

	/// The next block into `self.text`; false at the table's end
	fn next_block(&mut self) -> io::Result<bool> {
		self.text.clear();
		self.given = 0;
		let mut head = [0; BLOCK_HEADER];
		let got = read_full(&mut self.input, &mut head)?;
		if got < head.len() {
			self.cut = got != 0;
			return Ok(false);
		}
		let lines = u32::from_le_bytes([head[0], head[1], head[2], head[3]]);
		let bytes = u32::from_le_bytes([head[4], head[5], head[6], head[7]]);
		if bytes > MAX_BLOCK || lines > MAX_BLOCK {
			return Err(damaged("a block's size is not one"));
		}
		let mut block = vec![0; bytes as usize + 4];
		if read_full(&mut self.input, &mut block)? < block.len() {
			self.cut = true;
			return Ok(false);
		}
		let (coded, check) = block.split_at(bytes as usize);
		let mut whole = head.to_vec();
		whole.extend_from_slice(coded);
		if crc32(&whole).to_le_bytes() != check {
			return Err(damaged("a block does not match its checksum"));
		}

		let mut coder = Decoder::new(coded);
		for _ in 0..lines {
			let l = &mut self.line;
			self.state.line(&mut coder, l);
			if l.is_row {
				for (c, f) in l.fields.iter().enumerate() {
					if c > 0 {
						self.text.push(b',');
					}
					if Some(c) == self.state.key {
						self.text.extend_from_slice(&self.state.names[l.stream]);
					} else if f.number {
						write_number(&mut self.text, f.value, f.decimals, f.minus_zero);
					} else {
						self.text.extend_from_slice(&f.word);
					}
				}
			} else {
				self.text.extend_from_slice(&l.text);
			}
			match l.end {
				CR_LF => self.text.extend_from_slice(b"\r\n"),
				LF => self.text.push(b'\n'),
				_ => {}
			}
			if coder.overrun() {
				return Err(damaged("a block holds less than it says"));
			}
		}
		Ok(true)
	}
}

impl<R: Read> Read for Unpacker<R> {
	fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
		while self.given == self.text.len() {
			if self.ended {
				return Ok(0);
			}
			if !self.next_block()? {
				self.ended = true;
				return Ok(0);
			}
		}
		let n = buf.len().min(self.text.len() - self.given);
		buf[..n].copy_from_slice(&self.text[self.given..self.given + n]);
		self.given += n;
		Ok(n)
	}
}

/// Whether a table that starts with `start` is a packed one
pub fn is_packed(start: &[u8]) -> bool {
	start.starts_with(MAGIC)
}

/// Whether `input`, at its start, is a packed table; it is left at its start
pub fn packed<R: Read + Seek>(input: &mut R) -> io::Result<bool> {
	let mut start = [0; 8];
	let got = read_full(input, &mut start)?;
	input.seek(SeekFrom::Start(0))?;
	Ok(is_packed(&start[..got]))
}

/// A table, packed or text, as its text. What its name says does not count: a packed table is known by its first
/// bytes
pub fn open(path: &Path) -> io::Result<Box<dyn BufRead>> {
	let mut file = File::open(path)?;
	Ok(if packed(&mut file)? {
		Box::new(BufReader::with_capacity(1 << 20, Unpacker::new(BufReader::with_capacity(1 << 16, file))?))
	} else {
		Box::new(BufReader::with_capacity(1 << 20, file))
	})
}

/// A table's file name without its end, text or packed: None for a name that has neither
pub fn stem(name: &str) -> Option<&str> {
	name.strip_suffix(PACKED_END).or_else(|| name.strip_suffix(TEXT_END))
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A small generator of test values (xorshift): the same on every run
	struct Rng(u64);

	impl Rng {
		fn next(&mut self) -> u64 {
			self.0 ^= self.0 << 13;
			self.0 ^= self.0 >> 7;
			self.0 ^= self.0 << 17;
			self.0
		}

		/// A number between -1 and 1
		fn unit(&mut self) -> f64 {
			(self.next() >> 11) as f64 / (1u64 << 52) as f64 - 1.0
		}
	}

	/// A made-up device: where it is, how it moves, how it is turned (a quaternion) and how it turns
	struct Body {
		p: [f64; 3],
		v: [f64; 3],
		q: [f64; 4],
		w: [f64; 3],
	}

	fn pack(text: &[u8]) -> Vec<u8> {
		let mut w = PackWriter::new(Vec::new()).unwrap();
		w.write(text).unwrap();
		w.finish().unwrap()
	}

	fn unpack(packed: &[u8]) -> io::Result<Vec<u8>> {
		let mut text = Vec::new();
		Unpacker::new(packed)?.read_to_end(&mut text)?;
		Ok(text)
	}

	fn round_trip(text: &[u8]) -> Vec<u8> {
		let packed = pack(text);
		assert!(
			unpack(&packed).unwrap() == text,
			"{:?} did not come back",
			String::from_utf8_lossy(&text[..text.len().min(200)])
		);
		packed
	}

	/// A pose log as the driver writes one, made up: `devices` devices that turn and drift, a row each a frame, with
	/// the kinds of column the real one has (a time for the frame, a serial, an age, a position and a velocity, a
	/// quaternion, the rotation matrix of it, a flag, a minus zero)
	fn pose_log(rows: usize, devices: usize, seed: u64) -> Vec<u8> {
		let mut rng = Rng(seed | 1);
		let mut text =
			b"t,id,serial,age,x,y,z,vx,vy,vz,spto,sqw,sqx,sqy,sqz,m00,m01,m02,m10,m11,m12,m20,m21,m22,ok\n".to_vec();
		let mut state: Vec<Body> = (0..devices)
			.map(|_| Body {
				p: [rng.unit() * 2.0, rng.unit(), rng.unit() * 2.0],
				v: [0.0; 3],
				q: [1.0, 0.0, 0.0, 0.0],
				w: [rng.unit(), rng.unit(), rng.unit()],
			})
			.collect();
		let mut t = 27438.481719;
		for frame in 0..rows / devices {
			t += 0.011111 + rng.unit() * 0.0004;
			for (d, Body { p, v, q, w }) in state.iter_mut().enumerate() {
				// every second device lies still for a while now and then
				let still = d % 2 == 1 && (frame / 200) % 3 == 0;
				for k in 0..3 {
					if still {
						v[k] = 0.0;
					} else {
						v[k] += rng.unit() * 0.02;
						w[k] += rng.unit() * 0.05;
					}
					p[k] += v[k] * 0.011 + rng.unit() * 0.00002;
				}
				let rate = if still { [0.0; 3] } else { *w };
				let dq = [1.0, rate[0] * 0.0055, rate[1] * 0.0055, rate[2] * 0.0055];
				let mut n = [
					q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3],
					q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
					q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1],
					q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0],
				];
				let size = n.iter().map(|x| x * x).sum::<f64>().sqrt();
				n.iter_mut().for_each(|x| *x /= size);
				*q = n;
				let [qw, qx, qy, qz] = *q;
				let m = [
					1.0 - 2.0 * (qy * qy + qz * qz),
					2.0 * (qx * qy - qz * qw),
					2.0 * (qx * qz + qy * qw),
					2.0 * (qx * qy + qz * qw),
					1.0 - 2.0 * (qx * qx + qz * qz),
					2.0 * (qy * qz - qx * qw),
					2.0 * (qx * qz - qy * qw),
					2.0 * (qy * qz + qx * qw),
					1.0 - 2.0 * (qx * qx + qy * qy),
				];
				let mut row = format!(
					"{t:.6},{},LHR-{:08X},{:.4}",
					d + 1,
					0x0A1B_2C3Du32.wrapping_mul(d as u32 + 3),
					rng.unit().abs() * 0.003
				);
				for x in p.iter() {
					row += &format!(",{x:.5}");
				}
				for x in v.iter() {
					row += &format!(",{x:.4}");
				}
				row += ",-0.0000";
				for x in q.iter().chain(m.iter()) {
					row += &format!(",{x:.6}");
				}
				row += ",1\n";
				text.extend_from_slice(row.as_bytes());
			}
		}
		text
	}

	#[test]
	fn a_number_is_read_as_printf_writes_it_and_written_back_the_same() {
		for (text, value, decimals, minus_zero) in [
			("0", 0, 0, false),
			("7", 7, 0, false),
			("-12", -12, 0, false),
			("0.5", 5, 1, false),
			("-0.0000", 0, 4, true),
			("-0", 0, 0, true),
			("0.000000", 0, 6, false),
			("27438.481719", 27_438_481_719, 6, false),
			("-749.5305", -7_495_305, 4, false),
			("1.000000", 1_000_000, 6, false),
			("-0.000001", -1, 6, false),
			("123456789012345678", 123_456_789_012_345_678, 0, false),
			("123.456789012345678", 123_456_789_012_345_678, 15, false),
		] {
			assert_eq!(number(text.as_bytes()), Some((value, decimals, minus_zero)), "{text}");
			let mut back = Vec::new();
			write_number(&mut back, value, decimals, minus_zero);
			assert_eq!(String::from_utf8(back).unwrap(), text);
		}
		// anything that would not be written back the same is not a number here
		for text in [
			"",
			"-",
			".",
			".5",
			"5.",
			"+5",
			"05",
			"00",
			"-01.5",
			"1e5",
			"1.5e-3",
			"nan",
			"-nan(ind)",
			"inf",
			" 1",
			"1 ",
			"0x10",
			"1.2.3",
			"1,5",
			"1234567890123456789",
			"0.1234567890123456",
			"1234.567890123456789",
			"LHR-0A1B2C3D",
			"--1",
		] {
			assert_eq!(number(text.as_bytes()), None, "{text:?}");
		}
		// and any value at all is written as something (a damaged file's numbers)
		for value in [i64::MIN, i64::MAX, -1, 1] {
			for decimals in [0, 15, 200] {
				let mut out = Vec::new();
				write_number(&mut out, value, decimals, true);
				assert!(!out.is_empty());
			}
		}
	}

	#[test]
	fn roots_and_quotients_are_to_the_nearest() {
		// r - 1/2 <= the root of n < r + 1/2, in whole numbers
		let nearest_root =
			|n: i128, r: i128| (r == 0 || (2 * r - 1) * (2 * r - 1) <= 4 * n) && 4 * n < (2 * r + 1) * (2 * r + 1);
		for n in 0..5000i128 {
			let r = root(n) as i128;
			assert!(nearest_root(n, r), "{n}: {r}");
		}
		let mut rng = Rng(5);
		for _ in 0..20000 {
			let n = (rng.next() as i128) << (rng.next() % 60);
			let r = root(n) as i128;
			assert!(nearest_root(n, r), "{n}: {r}");
		}
		assert_eq!(root(-5), 0);
		assert_eq!(root(1_000_000_000_000_000_000_000_000_000_000), 1_000_000_000_000_000);
		for (n, d, q) in
			[(7, 2, 4), (-7, 2, -4), (6, 4, 2), (-6, 4, -2), (5, 4, 1), (0, 3, 0), (1_000_000, 1_000_000, 1)]
		{
			assert_eq!(nearest(n, d), q, "{n} / {d}");
		}
		assert_eq!(nearest(i128::MAX / 2, 1), LARGEST);
	}

	#[test]
	fn the_column_names_say_what_belongs_together() {
		let names: Vec<&[u8]> =
			"t,sqw,sqx,sqy,sqz,m00,m01,m02,m10,m11,m12,m20,m21,m22,vqw,vqx,vqz,a00,a01,a02,a10,qw,qx,qy,qz"
				.split(',')
				.map(str::as_bytes)
				.collect();
		let got: Vec<String> = shapes(&names)
			.iter()
			.map(|s| match s {
				Shape::None => "-".to_string(),
				Shape::Unit(n) => format!("u{n}"),
				Shape::Cross(n) => format!("c{n}"),
			})
			.collect();
		// a quaternion's last, a matrix's rows; a quaternion short of a name and a matrix cut off are neither
		assert_eq!(got.join(" "), "- - - - u3 - - u2 - - u2 c6 c7 c8 - - - - - - - - - - u3");
	}

	#[test]
	fn a_pose_log_comes_back_and_is_a_fraction_of_its_text() {
		let text = pose_log(40_000, 5, 1);
		let packed = round_trip(&text);
		// the made-up one packs to under a tenth; the row's geometry is in use (without it, a ninth more)
		assert!(packed.len() * 10 < text.len(), "{} of {}", packed.len(), text.len());
		assert!(is_packed(&packed) && !is_packed(&text));

		// the same text in other pieces is the same file but for where its blocks end: a line at a time, ended after
		// each, reads back the same
		let mut w = PackWriter::new(Vec::new()).unwrap();
		for (i, line) in text.split_inclusive(|&c| c == b'\n').enumerate() {
			let (a, b) = line.split_at(line.len() / 2);
			w.write(a).unwrap();
			w.write(b).unwrap();
			if i % 700 == 0 {
				w.end().unwrap();
			}
		}
		assert!(unpack(&w.finish().unwrap()).unwrap() == text);
	}

	/// The packed form is the format: a change to the model that changes a packed table's bytes is a new version of
	/// it, not an edit
	#[test]
	fn the_packed_bytes_are_the_formats() {
		let packed = pack(&pose_log(6_000, 3, 9));
		// every byte counts (FNV-1a; a CRC over blocks that end in their own CRC says their lengths alone)
		let hash =
			packed.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, &b| (h ^ b as u64).wrapping_mul(0x0000_0100_0000_01b3));
		assert_eq!((packed.len(), hash), (94268, 18267796017530769756));
	}

	#[test]
	fn any_text_comes_back() {
		let head = "t,id,serial,a,b,c\n";
		for text in [
			"",
			"\n",
			"no newline",
			"one,line",
			"t,id,serial,a,b,c",
			"t,id,serial,a,b,c\n1.0,1,X,1,2,3",
			"t,id,serial,a,b,c\r\n1.0,1,X,1,2,3\r\n1.1,1,X,1,2,4\n1.2,2,Y,-0.0,nan,\r\n",
			"single\n1\n2\n3\n",
			"a,b\n1,2\n1,2,3\n1\n\n,\n,,\n-0,-0\n0,0\n-0,0\n",
			// a column changes what it holds, and back
			"t,id,serial,a,b,c\n1,1,X,1.5,x,3\n2,1,X,1.50,7,3\n3,1,X,word,7.25,3\n4,1,X,1.5,x,3\n",
			// numbers too long for one, numbers at the ends of what one holds
			"a,b\n999999999999999999,-999999999999999999\n1234567890123456789,0.999999999999999\n-999999999999999999,999999999999999999\n",
			// no column that names the device
			"x,y,z\n0.1,0.2,0.3\n0.2,0.3,0.4\n0.3,0.4,0.5\n",
			// the device named by its id
			"t,id,x\n1,3,0.5\n1,4,0.7\n2,3,0.6\n2,4,0.8\n",
		] {
			round_trip(text.as_bytes());
		}
		// more devices than there are places for: the rows of the ones past them are kept as they are
		let mut many = head.to_string();
		for round in 0..3 {
			for d in 0..STREAMS + 20 {
				many += &format!("{}.5,{d},S{d},{},{}.25,-0.0\n", round, d * round, d + round);
			}
		}
		round_trip(many.as_bytes());

		// lines of anything
		let mut rng = Rng(3);
		for _ in 0..300 {
			let mut text = if rng.next().is_multiple_of(2) { head.as_bytes().to_vec() } else { Vec::new() };
			for _ in 0..rng.next() % 40 {
				for _ in 0..rng.next() % 9 {
					match rng.next() % 6 {
						0 => text.extend_from_slice(format!("{:.4}", rng.unit() * 100.0).as_bytes()),
						1 => text.extend_from_slice(format!("{}", rng.next() as i64 >> (rng.next() % 64)).as_bytes()),
						2 => text.extend_from_slice(b"LHR-0A1B2C3D"),
						3 => text.push(rng.next() as u8),
						4 => text.extend_from_slice(b"-0.000"),
						_ => {}
					}
					text.push(b',');
				}
				text.extend_from_slice(
					[&b"\n"[..], b"\r\n", b"", b"3,1,LHR-0A1B2C3D,1,2,3\n"][(rng.next() % 4) as usize],
				);
			}
			round_trip(&text);
		}
	}

	#[test]
	fn a_table_cut_short_reads_to_its_last_whole_block() {
		let text = pose_log(3 * BLOCK_LINES as usize, 4, 2);
		let packed = pack(&text);
		// where the blocks end
		let mut ends = vec![HEADER];
		while *ends.last().unwrap() < packed.len() {
			let at = *ends.last().unwrap();
			let bytes = u32::from_le_bytes(packed[at + 4..at + 8].try_into().unwrap()) as usize;
			ends.push(at + BLOCK_HEADER + bytes + 4);
		}
		assert!(ends.len() >= 4 && *ends.last().unwrap() == packed.len());
		let lines: Vec<&[u8]> = text.split_inclusive(|&c| c == b'\n').collect();
		for (whole, &end) in ends.iter().enumerate().take(3) {
			for cut in [end, end + 1, end + BLOCK_HEADER, ends[whole + 1] - 1] {
				let mut u = Unpacker::new(&packed[..cut]).unwrap();
				let mut got = Vec::new();
				u.read_to_end(&mut got).unwrap();
				let want: Vec<u8> = lines[..whole * BLOCK_LINES as usize].concat();
				assert!(got == want, "cut at {cut}");
				assert_eq!(u.cut(), cut != end, "cut at {cut}");
			}
		}
		// a table whose header is cut, or is another file's, is not one
		assert!(Unpacker::new(&packed[..HEADER - 1]).is_err());
		assert!(Unpacker::new(&text[..]).is_err());
		let mut newer = packed.clone();
		newer[8] = VERSION + 1;
		assert!(Unpacker::new(&newer[..]).is_err());
	}

	#[test]
	fn a_damaged_block_is_an_error_after_the_blocks_before_it() {
		// three blocks of 300 lines
		let text = pose_log(900, 4, 4);
		let lines: Vec<&[u8]> = text.split_inclusive(|&c| c == b'\n').collect();
		let mut w = PackWriter::new(Vec::new()).unwrap();
		for piece in lines.chunks(300) {
			w.write(&piece.concat()).unwrap();
			w.end().unwrap();
		}
		let packed = w.finish().unwrap();
		let first =
			HEADER + BLOCK_HEADER + u32::from_le_bytes(packed[HEADER + 4..HEADER + 8].try_into().unwrap()) as usize + 4;
		let whole: Vec<u8> = lines[..300].concat();
		let mut rng = Rng(8);
		for _ in 0..200 {
			let mut bad = packed.clone();
			let at = first + (rng.next() as usize % (bad.len() - first));
			bad[at] ^= 1 << (rng.next() % 8);
			let mut u = Unpacker::new(&bad[..]).unwrap();
			let mut got = Vec::new();
			let read = u.read_to_end(&mut got);
			// a changed length reads as a table cut short; anything else as a block that does not match its checksum
			assert!(read.is_err() || u.cut(), "a change at {at} went unseen");
			assert!(got.starts_with(&whole) && text.starts_with(&got), "a change at {at}");
		}
		// what a block decodes to is never trusted: with its checksum made to match, its lines are whatever they are,
		// and the reading neither stops the program nor asks for the machine's memory
		for _ in 0..300 {
			let mut bad = packed.clone();
			let bytes = u32::from_le_bytes(bad[first + 4..first + 8].try_into().unwrap()) as usize;
			let coded = first + BLOCK_HEADER..first + BLOCK_HEADER + bytes;
			for _ in 0..1 + rng.next() % 4 {
				let at = coded.start + (rng.next() as usize % bytes);
				bad[at] = rng.next() as u8;
			}
			let check = crc32(&bad[first..coded.end]).to_le_bytes();
			bad[coded.end..coded.end + 4].copy_from_slice(&check);
			let mut got = Vec::new();
			let _ = Unpacker::new(&bad[..]).unwrap().read_to_end(&mut got);
			assert!(got.starts_with(&whole) && got.len() < 100 * text.len());
		}
	}

	#[test]
	fn a_file_of_either_form_opens_as_its_text() {
		let dir = std::env::temp_dir().join(format!("sally-pack-table-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let text = pose_log(2_000, 2, 6);
		let (plain, packed) =
			(dir.join("sally_poses_20261118_154450.csv"), dir.join("sally_poses_20261118_154450.csvz"));
		std::fs::write(&plain, &text).unwrap();
		std::fs::write(&packed, pack(&text)).unwrap();
		for path in [&plain, &packed] {
			let mut got = Vec::new();
			open(path).unwrap().read_to_end(&mut got).unwrap();
			assert!(got == text, "{}", path.display());
		}
		// by its first bytes, not its name
		std::fs::rename(&packed, &plain).unwrap();
		let mut first = String::new();
		open(&plain).unwrap().read_line(&mut first).unwrap();
		assert!(first.starts_with("t,id,serial,"));
		std::fs::remove_dir_all(&dir).unwrap();

		assert_eq!(stem("sally_poses_20261118_154450.csv"), Some("sally_poses_20261118_154450"));
		assert_eq!(stem("sally_poses_20261118_154450.csvz"), Some("sally_poses_20261118_154450"));
		assert_eq!(stem("sally_poses_20261118_154450.txt"), None);
	}
}
