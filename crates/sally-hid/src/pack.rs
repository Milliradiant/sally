//! Packed tap recordings: the lines of a tap recording (`line`) in about a fourteenth of their bytes, and back to the
//! same bytes.
//!
//! A tap recording is text, one line a report, the report's bytes in hex after the time and the device's name: 162
//! bytes for the 59 a radio dongle sends. Most of those 59 say little: half of them are what the last report left in
//! the dongle's buffer, the IMU sample is the one before it plus sensor noise, and a sensor's light comes when it
//! came a rotation ago, give or take. A packed recording holds what a report adds to what could be told ahead
//! (`report`, `imu`, `light`), through an entropy coder that learns as it goes how likely each answer is (`coder`).
//!
//! Nothing is lost: a packed recording unpacks to the recording's own bytes, whatever they were. A line that is not
//! a report as sally writes it (a comment, a line cut short, another program's spelling) is kept as its bytes, and a
//! report that is not what its kind is taken to be only packs less well. What a recording holds in the first place is
//! the tap's to say: it writes a dongle's reports without the bytes nothing reads (`crate::strip`), which were a
//! third of a packed recording.
//!
//! The file:
//!
//! ```text
//! header   0x89 "HIDTAPZ", the format's version (2; 1 is read too), three zero bytes
//! block    lines: u32, bytes: u32, from: u64, to: u64 (little-endian), the coded lines (`bytes` of them), the
//!          CRC-32 of all before it in the block
//! ...
//! ```
//!
//! `from` and `to` are the earliest and the latest of the block's report times (`from` above `to`: it has none), so
//! a recording's span is read off its blocks without unpacking them (`span`). A block is whole or it is not there: a
//! recording being written, or one whose writer was stopped, reads up to its last whole block. A block ends every half
//! second of the recording's time. The coder starts over with each block; what it has learned, and what it knows
//! of each device, carries on, so the blocks are read in order from the first.
//!
//! - `Packer`: lines in, blocks out; `PackWriter`: text in whatever pieces, a packed file out
//! - `Unpacker`: a packed file read as the text it holds
//! - `open`, `text`: a recording of either form as its text, for everything that reads one
//! - `stem`: a recording's name without its end, whichever of the two it has

mod imu;
mod light;
mod report;

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::Path;

use report::{Models as ReportModels, Stream, MAX_REPORT};
use sally_math::crc::crc32;
use sally_pack::coder::{Decoder, Encoder, Io, Number, Prob, Symbol};

/// A packed recording's first bytes: no text recording starts with them
pub const MAGIC: &[u8; 8] = b"\x89HIDTAPZ";
/// The format's version: the second says where a radio report holds zeros after its packets (`report`), which a
/// recording written without those bytes does throughout. The first is still read
const VERSION: u8 = 2;
const FIRST_VERSION: u8 = 1;
const HEADER: usize = 12;
/// A block's own header: its lines, its bytes, its earliest and latest report times
const BLOCK_HEADER: usize = 24;

/// The end of a tap recording's name as text, and packed
pub const TEXT_END: &str = ".hidtap";
pub const PACKED_END: &str = ".hidtapz";

/// A block ends once it holds this much of the recording's time, this many lines, or this much of its text
const BLOCK_US: u64 = 500_000;
const BLOCK_LINES: u32 = 8192;
const BLOCK_TEXT: usize = 1 << 24;
/// The most a block is read as (a damaged block, or one made up to, must not ask for the machine's memory): its
/// coded bytes, a line's bytes, and the text it unpacks to. A block is written up to `BLOCK_TEXT` (and before that
/// bound was, up to `BLOCK_LINES` of a recording's lines, a few hundred bytes each), so the line that took it there
/// is all it can hold beyond
const MAX_BLOCK: u32 = 1 << 26;
const MAX_TEXT: i64 = 1 << 26;
const MAX_BLOCK_TEXT: usize = BLOCK_TEXT + MAX_TEXT as usize;
/// The longest name a device has (a dongle's or a device's serial, with what follows it on a report's line): the
/// names are kept for the whole recording. A report line with a longer one is kept as its bytes
const MAX_NAME: usize = 255;

/// The devices a recording can name; a line of one more is kept as its bytes
const STREAMS: usize = 255;

/// How a line ends
const CR_LF: u8 = 0;
const LF: u8 = 1;
const NO_END: u8 = 2;

/// One line, between its text and the coder: a report line as sally writes it is its device, its time and its
/// report; any other is its bytes
#[derive(Default)]
struct Line {
	end: u8,
	is_report: bool,
	/// the device's place among those seen, or `STREAMS` for one not seen before (then `name` is its name)
	stream: usize,
	name: Vec<u8>,
	time: u64,
	report: Vec<u8>,
	text: Vec<u8>,
}

/// What the packer and the unpacker both keep as they go through a recording's lines
struct State {
	is_report: [Prob; 2],
	end: Symbol,
	stream: Symbol,
	time: Number,
	text_len: Number,
	text: Symbol,
	reports: ReportModels,

	/// the devices by their names: on a report line, all between the time and the report's hex
	names: Vec<Vec<u8>>,
	streams: Vec<Stream>,
	last_is_report: bool,
	last_end: u8,
	last_stream: usize,
	last_time: u64,
	/// the size of the last step in time: steps come in runs (the reports read together, then the wait for the next)
	last_step: usize,
	/// unpacking: a line held more than one may
	over: bool,
}

impl State {
	fn new(version: u8) -> State {
		State {
			is_report: [Prob::new(); 2],
			end: Symbol::new(2, 4),
			stream: Symbol::new(8, STREAMS + 1),
			time: Number::new(32),
			text_len: Number::new(1),
			text: Symbol::new(8, 1),
			reports: ReportModels::new(version),
			names: Vec::new(),
			streams: Vec::new(),
			last_is_report: true,
			last_end: CR_LF,
			last_stream: STREAMS,
			last_time: 0,
			last_step: 0,
			over: false,
		}
	}

	/// Bytes kept as they are: a line that is not a report (`most`: `MAX_TEXT`), a device's name (`MAX_NAME`)
	fn bytes<I: Io>(&mut self, io: &mut I, text: &mut Vec<u8>, most: usize) {
		let len = self.text_len.code(io, 0, text.len() as i64);
		self.over |= len < 0 || len > most as i64;
		let len = len.clamp(0, most as i64) as usize;
		if !I::PACKING {
			text.clear();
			text.resize(len, 0);
		}
		for b in text.iter_mut() {
			*b = self.text.code(io, 0, *b as u32) as u8;
		}
	}

	fn line<I: Io>(&mut self, io: &mut I, l: &mut Line) {
		l.is_report = io.bit(&mut self.is_report[self.last_is_report as usize], l.is_report);
		self.last_is_report = l.is_report;
		l.end = self.end.code(io, self.last_end as usize, l.end as u32) as u8;
		self.last_end = l.end;
		if !l.is_report {
			return self.bytes(io, &mut l.text, MAX_TEXT as usize);
		}

		let stream = self.stream.code(io, self.last_stream, l.stream as u32) as usize;
		let stream = if stream < self.streams.len() {
			stream
		} else {
			// a device not seen before, by its name
			self.bytes(io, &mut l.name, MAX_NAME);
			if self.streams.len() < STREAMS {
				self.names.push(l.name.clone());
				self.streams.push(Stream::default());
			}
			self.streams.len() - 1
		};
		l.stream = stream;

		let context = self.last_step * 2 + (stream == self.last_stream) as usize;
		let step = self.time.code(io, context, l.time.wrapping_sub(self.last_time) as i64);
		l.time = self.last_time.wrapping_add(step as u64);
		self.last_time = l.time;
		self.last_step = Number::length_of(step).min(15);
		self.last_stream = stream;

		report::report(io, &mut self.reports, &mut self.streams[stream], &mut l.report);
	}
}

/// A number as `%llu` writes it, and what follows it: None for anything a `u64` would not print back the same
fn decimal(text: &[u8]) -> Option<(u64, &[u8])> {
	let digits = text.iter().take_while(|c| c.is_ascii_digit()).count();
	if digits == 0 || (digits > 1 && text[0] == b'0') {
		return None;
	}
	let mut value = 0u64;
	for &c in &text[..digits] {
		value = value.checked_mul(10)?.checked_add((c - b'0') as u64)?;
	}
	Some((value, &text[digits..]))
}

fn hex_value(c: u8) -> Option<u8> {
	match c {
		b'0'..=b'9' => Some(c - b'0'),
		b'a'..=b'f' => Some(c - b'a' + 10),
		_ => None,
	}
}

/// Lines in, blocks out. `line` takes each line of a recording with its newline (the last may have none); `block`
/// gives the lines taken since the last block as a block, and `full` says when to ask for one
pub struct Packer {
	version: u8,
	state: State,
	coder: Encoder,
	/// the names' places (`State::names` the other way round)
	places: HashMap<Vec<u8>, usize>,
	line: Line,
	lines: u32,
	/// the bytes of the block's lines, and how many end a block
	text: usize,
	block_text: usize,
	/// the earliest and the latest of the block's report times
	span: Option<(u64, u64)>,
}

impl Default for Packer {
	fn default() -> Packer {
		Packer::new()
	}
}

impl Packer {
	pub fn new() -> Packer {
		Packer::of_version(VERSION)
	}

	fn of_version(version: u8) -> Packer {
		Packer {
			version,
			state: State::new(version),
			coder: Encoder::new(),
			places: HashMap::new(),
			line: Line::default(),
			lines: 0,
			text: 0,
			block_text: BLOCK_TEXT,
			span: None,
		}
	}

	/// What a packed recording starts with
	pub fn header(&self) -> [u8; HEADER] {
		let mut h = [0; HEADER];
		h[..8].copy_from_slice(MAGIC);
		h[8] = self.version;
		h
	}

	/// `R <time> <name...> <hex>` as sally writes it, into `self.line`: whether `text` is that, to the byte
	fn report_line(&mut self, text: &[u8]) -> bool {
		let Some(rest) = text.strip_prefix(b"R ") else { return false };
		let Some((time, rest)) = decimal(rest) else { return false };
		let Some(rest) = rest.strip_prefix(b" ") else { return false };
		// the name is all up to the last space, with it; the report's hex is what follows
		let name_len = rest.iter().rposition(|&c| c == b' ').map_or(0, |i| i + 1);
		let (name, hex) = rest.split_at(name_len);
		if hex.len() % 2 != 0 || hex.len() > 2 * MAX_REPORT || name.len() > MAX_NAME {
			return false;
		}
		let l = &mut self.line;
		l.report.clear();
		for pair in hex.chunks_exact(2) {
			let (Some(high), Some(low)) = (hex_value(pair[0]), hex_value(pair[1])) else { return false };
			l.report.push(high << 4 | low);
		}
		// a step in time the coder has no number for
		if time.wrapping_sub(self.state.last_time) as i64 == i64::MIN {
			return false;
		}
		l.stream = match self.places.get(name) {
			Some(&place) => place,
			None if self.places.len() < STREAMS => {
				self.places.insert(name.to_vec(), self.places.len());
				l.name.clear();
				l.name.extend_from_slice(name);
				STREAMS
			}
			None => return false,
		};
		l.time = time;
		true
	}

	/// One line of the recording: its bytes up to and with its newline, or the recording's last bytes without one
	pub fn line(&mut self, line: &[u8]) {
		let (text, end) = match line {
			[text @ .., b'\r', b'\n'] => (text, CR_LF),
			[text @ .., b'\n'] => (text, LF),
			text => (text, NO_END),
		};
		self.line.end = end;
		self.line.is_report = self.report_line(text);
		if self.line.is_report {
			let time = self.line.time;
			self.span = Some(self.span.map_or((time, time), |(from, to)| (from.min(time), to.max(time))));
		} else {
			self.line.text.clear();
			self.line.text.extend_from_slice(text);
		}
		self.state.line(&mut self.coder, &mut self.line);
		self.lines += 1;
		self.text += line.len();
	}

	/// Whether the lines taken since the last block are a block's worth
	pub fn full(&self) -> bool {
		self.lines >= BLOCK_LINES
			|| self.text >= self.block_text
			|| self.coder.size() >= MAX_BLOCK as usize / 4
			|| self.span.is_some_and(|(from, to)| to - from >= BLOCK_US)
	}

	/// The lines taken since the last block, as a block (nothing when there are none)
	pub fn block(&mut self) -> Vec<u8> {
		if self.lines == 0 {
			return Vec::new();
		}
		let coded = self.coder.finish();
		let (from, to) = self.span.take().unwrap_or((u64::MAX, 0));
		let mut block = Vec::with_capacity(BLOCK_HEADER + coded.len() + 4);
		block.extend_from_slice(&self.lines.to_le_bytes());
		block.extend_from_slice(&(coded.len() as u32).to_le_bytes());
		block.extend_from_slice(&from.to_le_bytes());
		block.extend_from_slice(&to.to_le_bytes());
		block.extend_from_slice(&coded);
		block.extend_from_slice(&crc32(&block).to_le_bytes());
		self.lines = 0;
		self.text = 0;
		block
	}
}

/// A packed recording written from the recording's text, handed over in whatever pieces: each block goes to `out`
/// in one write as soon as it is full, so the file never ends in half a block for longer than a write takes
pub struct PackWriter<W: Write> {
	out: W,
	packer: Packer,
	/// a line begun and not yet ended
	pending: Vec<u8>,
}

impl<W: Write> PackWriter<W> {
	pub fn new(out: W) -> io::Result<PackWriter<W>> {
		PackWriter::of_version(out, VERSION)
	}

	fn of_version(mut out: W, version: u8) -> io::Result<PackWriter<W>> {
		let packer = Packer::of_version(version);
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

	/// More of the recording's text
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

	/// The recording's end (`end`), and `out` back
	pub fn finish(mut self) -> io::Result<W> {
		self.end()?;
		Ok(self.out)
	}
}

/// A packed recording read as the text it holds. It reads to the last whole block: a recording still being written
/// ends there, and so does one that was cut short (`cut` says which it was)
pub struct Unpacker<R: Read> {
	input: R,
	state: State,
	line: Line,
	/// the text of the block read last, and how much of it has been handed on
	text: Vec<u8>,
	given: usize,
	/// the most text a block is read as
	most: usize,
	ended: bool,
	cut: bool,
}

/// A block header's lines and bytes
fn block_size(head: &[u8; BLOCK_HEADER]) -> (u32, u32) {
	(u32::from_le_bytes([head[0], head[1], head[2], head[3]]), u32::from_le_bytes([head[4], head[5], head[6], head[7]]))
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
	io::Error::new(io::ErrorKind::InvalidData, format!("packed recording: {what}"))
}

impl<R: Read> Unpacker<R> {
	/// `input` from its first byte: the header is read and checked
	pub fn new(mut input: R) -> io::Result<Unpacker<R>> {
		let mut header = [0; HEADER];
		if read_full(&mut input, &mut header)? != HEADER || &header[..8] != MAGIC {
			return Err(damaged("not one"));
		}
		if !(FIRST_VERSION..=VERSION).contains(&header[8]) {
			return Err(damaged(&format!("version {} (this sally reads up to version {VERSION})", header[8])));
		}
		Ok(Unpacker {
			input,
			state: State::new(header[8]),
			line: Line::default(),
			text: Vec::new(),
			given: 0,
			most: MAX_BLOCK_TEXT,
			ended: false,
			cut: false,
		})
	}

	/// Whether the recording ended inside a block (its writer was stopped mid-write, or is still writing)
	pub fn cut(&self) -> bool {
		self.cut
	}

	/// The next block into `self.text`; false at the recording's end
	fn next_block(&mut self) -> io::Result<bool> {
		self.text.clear();
		self.given = 0;
		let mut head = [0; BLOCK_HEADER];
		let got = read_full(&mut self.input, &mut head)?;
		if got < head.len() {
			self.cut = got != 0;
			return Ok(false);
		}
		let (lines, bytes) = block_size(&head);
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
			if l.is_report {
				self.text.extend_from_slice(b"R ");
				self.text.extend_from_slice(l.time.to_string().as_bytes());
				self.text.push(b' ');
				self.text.extend_from_slice(&self.state.names[l.stream]);
				for &b in &l.report {
					self.text.push(b"0123456789abcdef"[(b >> 4) as usize]);
					self.text.push(b"0123456789abcdef"[(b & 15) as usize]);
				}
			} else {
				self.text.extend_from_slice(&l.text);
			}
			match l.end {
				CR_LF => self.text.extend_from_slice(b"\r\n"),
				LF => self.text.push(b'\n'),
				_ => {}
			}
			if self.state.over || self.text.len() > self.most {
				return Err(damaged("a block holds more than one can"));
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

/// Whether a recording that starts with `start` is a packed one
pub fn is_packed(start: &[u8]) -> bool {
	start.starts_with(MAGIC)
}

/// Whether `input`, at its start, is a packed recording; it is left at its start
pub fn packed<R: Read + Seek>(input: &mut R) -> io::Result<bool> {
	let mut start = [0; 8];
	let got = read_full(input, &mut start)?;
	input.seek(SeekFrom::Start(0))?;
	Ok(is_packed(&start[..got]))
}

/// A recording, packed or text, as its text. What its name says does not count: a packed recording is known by its
/// first bytes
pub fn text<R: Read + Seek + Send + 'static>(mut input: R) -> io::Result<Box<dyn Read + Send>> {
	if packed(&mut input)? {
		Ok(Box::new(Unpacker::new(io::BufReader::with_capacity(1 << 16, input))?))
	} else {
		Ok(Box::new(input))
	}
}

/// The recording at `path`, packed or text, as its text
pub fn open(path: &Path) -> io::Result<Box<dyn Read + Send>> {
	text(File::open(path)?)
}

/// How much of a long packed recording's end `span` reads to find its last blocks
const SPAN_END: usize = 1 << 19;

/// A block header's earliest and latest report times: None for a block without a report
fn block_span(head: &[u8]) -> Option<(u64, u64)> {
	let from = u64::from_le_bytes(head[8..16].try_into().ok()?);
	let to = u64::from_le_bytes(head[16..24].try_into().ok()?);
	(from <= to).then_some((from, to))
}

fn widened(span: Option<(u64, u64)>, by: Option<(u64, u64)>) -> Option<(u64, u64)> {
	match (span, by) {
		(Some((a, b)), Some((c, d))) => Some((a.min(c), b.max(d))),
		(span, by) => span.or(by),
	}
}

/// The span of the whole blocks from `at` on, read header by header: of them all, or up to the first with a report
fn span_from<R: Read + Seek>(input: &mut R, mut at: u64, len: u64, first_only: bool) -> io::Result<Option<(u64, u64)>> {
	let mut span = None;
	let mut head = [0; BLOCK_HEADER];
	input.seek(SeekFrom::Start(at))?;
	while read_full(input, &mut head)? == BLOCK_HEADER {
		let next = at + BLOCK_HEADER as u64 + block_size(&head).1 as u64 + 4;
		if next > len {
			break;
		}
		span = widened(span, block_span(&head));
		if first_only && span.is_some() {
			break;
		}
		at = input.seek(SeekFrom::Start(next))?;
	}
	Ok(span)
}

/// The span of the last whole blocks in `end`, a recording's last bytes. Where a block starts in them is not written
/// anywhere: it is the first place whose bytes are a block with its checksum, and the blocks follow each other from
/// there. None when no such block has a report
fn span_of_end(end: &[u8]) -> Option<(u64, u64)> {
	// the block at `at`: where it ends, when all of it is there and matches its checksum
	let block = |at: usize| {
		let head: &[u8; BLOCK_HEADER] = end.get(at..at + BLOCK_HEADER)?.try_into().ok()?;
		let coded = at + BLOCK_HEADER + block_size(head).1 as usize;
		let check = end.get(coded..coded + 4)?;
		(crc32(&end[at..coded]).to_le_bytes() == check).then_some(coded + 4)
	};
	let mut at = (0..end.len()).find(|&at| block(at).is_some())?;
	let mut span = None;
	while let Some(next) = block(at) {
		span = widened(span, block_span(&end[at..]));
		at = next;
	}
	span
}

fn span_reading<R: Read + Seek>(input: &mut R, end: usize) -> io::Result<Option<(u64, u64)>> {
	let len = input.seek(SeekFrom::End(0))?;
	input.seek(SeekFrom::Start(0))?;
	let mut header = [0; HEADER];
	if read_full(input, &mut header)? != HEADER || &header[..8] != MAGIC {
		return Ok(None);
	}
	if len > 2 * end as u64 {
		// a long recording: its first block with a report, and the blocks in its last bytes
		let first = span_from(input, HEADER as u64, len, true)?;
		let mut last = vec![0; end];
		input.seek(SeekFrom::Start(len - end as u64))?;
		if read_full(input, &mut last)? == end {
			if let (Some(first), Some(last)) = (first, span_of_end(&last)) {
				return Ok(widened(Some(first), Some(last)));
			}
		}
	}
	span_from(input, HEADER as u64, len, false)
}

/// The earliest and the latest of a packed recording's report times (the recording's own clock, microseconds), read
/// off its whole blocks' headers: None when it has no report, or is not a packed recording. Of a long recording only
/// the start and the end are read (the times of a recording run on: its first block and its last ones hold the two).
/// `input` is left wherever the reading got to
pub fn span<R: Read + Seek>(input: &mut R) -> io::Result<Option<(u64, u64)>> {
	span_reading(input, SPAN_END)
}

/// A recording's file name without its end, text or packed: None for a name that has neither
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

		fn below(&mut self, n: u64) -> u64 {
			self.next() % n
		}

		fn chance(&mut self, one_in: u64) -> bool {
			self.below(one_in) == 0
		}
	}

	fn hex(bytes: &[u8]) -> String {
		bytes.iter().map(|b| format!("{b:02x}")).collect()
	}

	/// A radio device as a recording shows it: its dongle's two packet slots, an IMU that wanders, and sensors a
	/// base station's two sweeps cross once a rotation
	struct Radio {
		name: String,
		slots: [[u8; 29]; 2],
		time: u32,
		axes: [i16; 6],
		sweep: u32,
		rotation: u32,
	}

	impl Radio {
		fn new(name: &str, rng: &mut Rng) -> Radio {
			Radio {
				name: name.to_string(),
				slots: [[0; 29]; 2],
				time: rng.next() as u32 & 0xff_ffff,
				axes: [10, -3500, 2200, 4, -100, 30],
				sweep: rng.next() as u32 & 0xff_ffff,
				rotation: 0,
			}
		}

		/// A packet into slot `k`: its time and size, its payload, the dongle's four bytes after it
		fn packet(&mut self, k: usize, payload: &[u8], rng: &mut Rng) {
			let slot = &mut self.slots[k];
			slot[0] = (self.time >> 16) as u8;
			slot[1] = payload.len() as u8 + 1;
			slot[2] = (self.time >> 8) as u8;
			let end = (3 + payload.len()).min(29);
			slot[3..end].copy_from_slice(&payload[..end - 3]);
			for b in &mut slot[end..(end + 4).min(29)] {
				*b = rng.next() as u8;
			}
		}

		fn imu(&mut self, rng: &mut Rng) -> Vec<u8> {
			self.time = (self.time + 754) & 0xff_ffff;
			let mut p = vec![self.time as u8];
			for a in &mut self.axes {
				*a = a.wrapping_add(rng.below(41) as i16 - 20);
				p.extend_from_slice(&a.to_le_bytes());
			}
			p
		}

		/// Up to `hits` entries of light on channel 3: a rotation's mark, then sensors a little apart
		fn light(&mut self, hits: usize, rng: &mut Rng) -> Vec<u8> {
			let mut p = vec![0x31];
			if self.rotation.is_multiple_of(2) {
				p.extend_from_slice(&((self.sweep.wrapping_sub(360_000) & 0xff_ffff) << 2 | 1 << 27).to_le_bytes());
			}
			let mut at = self.sweep;
			for sensor in 0..hits as u32 {
				at = (at + 100 + rng.below(20) as u32) & 0xff_ffff;
				p.extend_from_slice(
					&((sensor * 3 % 32) << 27 | at << 3 | (rng.below(2) as u32) << 2 | 2).to_le_bytes(),
				);
			}
			self.rotation += 1;
			// the rotation's second sweep, then the next rotation's first
			self.sweep = (self.sweep + if self.rotation % 2 == 1 { 312_000 } else { 949_000 - 312_000 }) & 0xff_ffff;
			p
		}

		fn report(&mut self, rng: &mut Rng) -> Vec<u8> {
			let two = rng.chance(12);
			for k in 0..1 + two as usize {
				let mut payload = Vec::new();
				match rng.below(40) {
					0 => payload.push(0xe2),
					1 => {}
					2 => {
						// the battery
						payload.extend([0xc0]);
						payload.extend(self.imu(rng));
						payload.extend([0x80, 77]);
					}
					3 => payload.extend([0x20, 0x05, rng.below(4) as u8, rng.below(200) as u8]),
					4 => payload.extend([0x30, 0x00, 0xa1, 2, 9, 9, 9, rng.below(9) as u8, 0, 0, 0x31]),
					5 => payload.extend([0x47, 1, 2, 3]),
					6..=22 => {
						payload.push(0x80);
						payload.extend(self.imu(rng));
					}
					23..=26 => {
						payload.push(0x90);
						payload.extend(self.imu(rng));
						payload.extend(self.light(2, rng));
					}
					_ => {
						payload.push(0x10);
						let hits = 1 + rng.below(6) as usize;
						payload.extend(self.light(hits, rng));
					}
				}
				payload.truncate(26);
				self.packet(k, &payload, rng);
			}
			let mut report = vec![if two { 0x24 } else { 0x23 }];
			report.extend_from_slice(&self.slots[0]);
			report.extend_from_slice(&self.slots[1]);
			report
		}
	}

	/// A recording of `lines` lines as sally writes them, with everything else a recording has been seen to hold
	fn recording(seed: u64, lines: usize) -> Vec<u8> {
		let mut rng = Rng(seed | 1);
		let mut radios: Vec<Radio> =
			(0..4).map(|i| Radio::new(&format!("AAAA00000{i}-{i}YX 2102 -1 -1 I "), &mut rng)).collect();
		let mut headset = [[0u8; 17]; 3];
		let (mut sequence, mut sample_time, mut other) = (0u8, 0x7600_0000u32, [0u8; 65]);
		let mut light = [0u8; 64];
		let mut time = 30_000_000_000u64 + rng.below(1 << 30);
		let mut text = Vec::new();
		let end = if seed.is_multiple_of(3) { "\n" } else { "\r\n" };
		for n in 0..lines {
			time += if rng.chance(6) { 1800 + rng.below(200) } else { rng.below(90) };
			let line = match rng.below(100) {
				0 => "# cache AAAA000001-1YX LHR-00000001".to_string(),
				1 => format!("A {time} AAAA000002-2YX LHR-00000002"),
				2 => format!("D {time} AAAA000003-3YX 2102"),
				// spellings sally does not write, a line cut short, an empty one, bytes that are not text
				3 => match rng.below(9) {
					0 => format!("R {time} AAAA000001-1YX 2102 -1 -1 I 23AB"),
					1 => format!("R 0{time} AAAA000001-1YX 2102 -1 -1 I 23ab"),
					2 => format!("R {time} AAAA000001-1YX 2102 -1 -1 I 23a"),
					3 => format!("R {time}  AAAA000001-1YX 2102 -1 -1 I 23ab"),
					4 => format!("R {time} AAAA000001-1YX 2102 -1 -1 I "),
					5 => "R".to_string(),
					6 => String::new(),
					7 => format!("R {time} AAAA000001-1YX 2102 -1 -1 I 2412\r"),
					_ => {
						(0..rng.below(300)).map(|_| (rng.below(254) as u8 + 1) as char).filter(|&c| c != '\n').collect()
					}
				},
				4 => {
					time = time.wrapping_sub(rng.below(5_000_000));
					format!("R {time} AAAA000001-1YX 2102 -1 -1 F 10{}", hex(&[7; 63]))
				}
				5..=24 => {
					// the headset's IMU: the next of its three samples replaced
					sequence = sequence.wrapping_add(1);
					sample_time = sample_time.wrapping_add(48_000 + rng.below(40) as u32);
					let s = &mut headset[sequence as usize % 3];
					for a in 0..6 {
						s[2 * a..2 * a + 2]
							.copy_from_slice(&(rng.below(60) as i16 - 30 + 100 * a as i16).to_le_bytes());
					}
					s[12..16].copy_from_slice(&sample_time.to_le_bytes());
					s[16] = sequence;
					format!("R {time} LHR-TEST0001 2300 0 -1 I 20{}", hex(&headset.concat()))
				}
				25..=29 => {
					let fresh = radios[0].light(1 + rng.below(12) as usize, &mut rng);
					light[0] = 0x28;
					light[1] = fresh.len() as u8;
					light[2..2 + fresh.len()].copy_from_slice(&fresh);
					format!("R {time} LHR-TEST0001 2300 1 -1 I {}", hex(&light))
				}
				30..=32 => {
					other[13] = rng.next() as u8;
					other[1] = 1;
					format!("R {time} LHR-TEST0001 2300 2 -1 I {}", hex(&other))
				}
				33 => format!("R {time} DEVICE{} 2102 -1 -1 I {}", rng.below(400), hex(&[n as u8; 5])),
				_ => {
					let r = &mut radios[rng.below(4) as usize];
					let report = r.report(&mut rng);
					if rng.chance(50) {
						// a report that never reached the recording: the next one's slots show what it left
						r.report(&mut rng);
					}
					format!("R {time} {}{}", r.name, hex(&report))
				}
			};
			text.extend(line.chars().map(|c| c as u8));
			if n + 1 < lines || seed.is_multiple_of(2) {
				text.extend_from_slice(if rng.chance(40) { b"\n" } else { end.as_bytes() });
			}
		}
		text
	}

	fn pack(text: &[u8], piece: usize) -> Vec<u8> {
		let mut w = PackWriter::new(Vec::new()).unwrap();
		for p in text.chunks(piece) {
			w.write(p).unwrap();
		}
		w.finish().unwrap()
	}

	fn unpack(packed: &[u8]) -> io::Result<Vec<u8>> {
		let mut text = Vec::new();
		Unpacker::new(packed)?.read_to_end(&mut text)?;
		Ok(text)
	}

	#[test]
	fn a_recording_comes_back_to_the_byte() {
		for seed in 1..=6 {
			let text = recording(seed, 6000);
			for piece in [1 << 20, 4096, 61] {
				assert!(unpack(&pack(&text, piece)).unwrap() == text, "seed {seed}, pieces of {piece}");
			}
		}
		// handed over a byte at a time, and nothing at all
		let text = recording(9, 300);
		assert!(unpack(&pack(&text, 1)).unwrap() == text);
		assert_eq!(unpack(&pack(b"", 1)).unwrap(), b"");
		assert_eq!(pack(b"", 1), Packer::new().header());
	}

	#[test]
	fn any_bytes_come_back() {
		let mut rng = Rng(77);
		for round in 0..40 {
			let n = rng.below(5000) as usize;
			let text: Vec<u8> = (0..n)
				.map(|_| match rng.below(30) {
					0 => b'\n',
					1 => b'\r',
					2 => b' ',
					3 => b'R',
					4..=12 => b"0123456789abcdef"[rng.below(16) as usize],
					_ => rng.next() as u8,
				})
				.collect();
			assert!(unpack(&pack(&text, 1 + rng.below(700) as usize)).unwrap() == text, "round {round}");
		}
	}

	#[test]
	fn the_longest_report_and_one_too_long() {
		let longest = format!("R 5 X 2102 -1 -1 I {}\r\n", hex(&[0x5a; MAX_REPORT]));
		let too_long = format!("R 6 X 2102 -1 -1 I {}\r\n", hex(&[0x5a; MAX_REPORT + 1]));
		let text = [longest.as_bytes(), too_long.as_bytes(), longest.as_bytes()].concat();
		assert!(unpack(&pack(&text, 1000)).unwrap() == text);
		// a device's name, likewise: one too long is not a device's (its line is kept as its bytes, and takes no place)
		let named = |n: usize| format!("R 5 {} 23\r\n", "N".repeat(n - 1));
		let text = [named(MAX_NAME), named(MAX_NAME + 1), named(MAX_NAME)].concat();
		let mut w = PackWriter::new(Vec::new()).unwrap();
		w.write(text.as_bytes()).unwrap();
		assert_eq!(w.packer.places.len(), 1);
		assert!(unpack(&w.finish().unwrap()).unwrap() == text.as_bytes());
	}

	#[test]
	fn a_time_no_step_reaches() {
		// a step of 2^63 either way is no number the coder has: the line is kept as its bytes
		let text = format!("R 1 X 2102 -1 -1 I 23\nR {} X 2102 -1 -1 I 23\nR 1 X 2102 -1 -1 I 23\n", 1 + (1u64 << 63));
		assert!(unpack(&pack(text.as_bytes(), 1000)).unwrap() == text.as_bytes());
		let text = format!(
			"R {} X 2102 -1 -1 I 23\nR 0 X 2102 -1 -1 I 23\nR 18446744073709551616 X 2102 -1 -1 I 23\n",
			u64::MAX
		);
		assert!(unpack(&pack(text.as_bytes(), 1000)).unwrap() == text.as_bytes());
	}

	#[test]
	fn it_packs() {
		// a recording of mostly radio reports: a twelfth of its text, and less
		let text = recording(4, 40000);
		let packed = pack(&text, 1 << 16);
		assert!(packed.len() * 12 < text.len(), "{} bytes of text packed to {}", text.len(), packed.len());
	}

	/// Reports of one radio, one every half millisecond from 7 s on
	fn steady(reports: u64) -> Vec<u8> {
		let mut rng = Rng(11);
		let mut radio = Radio::new("AAAA000001-1YX 2102 -1 -1 I ", &mut rng);
		let mut text = String::new();
		for n in 0..reports {
			let report = radio.report(&mut rng);
			text += &format!("R {} {}{}\r\n", 7_000_000 + n * 500, radio.name, hex(&report));
		}
		text.into_bytes()
	}

	/// A long recording's span is read off its start and its end alone, and is the one all its headers give
	#[test]
	fn a_long_recordings_span_is_read_off_its_two_ends() {
		let mut w = PackWriter::new(Vec::new()).unwrap();
		// notes alone before the first report and after the last one: blocks without a report at both ends
		w.write(b"# first\r\n").unwrap();
		w.end_block().unwrap();
		w.write(&steady(20000)).unwrap();
		w.end_block().unwrap();
		w.write(b"# last\r\n").unwrap();
		let packed = w.finish().unwrap();
		let all = Some((7_000_000, 7_000_000 + 19999 * 500));
		assert_eq!(span_from(&mut io::Cursor::new(&packed), HEADER as u64, packed.len() as u64, false).unwrap(), all);
		assert_eq!(span(&mut io::Cursor::new(&packed)).unwrap(), all);
		for end in [40, 700, 5000, 60000, packed.len() / 2 - 1] {
			// whole, and stopped at any byte of its last blocks
			for cut in [0, 1, 5, 29, 300, 4000] {
				let cut = &packed[..packed.len() - cut];
				let walked = span_from(&mut io::Cursor::new(cut), HEADER as u64, cut.len() as u64, false).unwrap();
				assert_eq!(span_reading(&mut io::Cursor::new(cut), end).unwrap(), walked, "{end} {}", cut.len());
				assert!(walked.unwrap().1 > 16_000_000);
			}
		}
		// the end's blocks are found wherever the bytes read start in one, and only where a whole one is among them
		for end in [30000, 30001, 47111, 60000] {
			assert_eq!(span_of_end(&packed[packed.len() - end..]).map(|s| s.1), all.map(|s| s.1), "{end}");
		}
		// (the last block with a report is longer than these)
		for end in [20, 700, 3000] {
			assert_eq!(span_of_end(&packed[packed.len() - end..]), None, "{end}");
		}
	}

	/// How many blocks a packed recording has
	fn blocks_of(packed: &[u8]) -> usize {
		let mut blocks = 0;
		let mut at = HEADER;
		while at < packed.len() {
			let head: [u8; BLOCK_HEADER] = packed[at..at + BLOCK_HEADER].try_into().unwrap();
			at += BLOCK_HEADER + block_size(&head).1 as usize + 4;
			blocks += 1;
		}
		assert_eq!(at, packed.len());
		blocks
	}

	#[test]
	fn a_block_ends_every_half_second_and_at_its_fill_of_text() {
		// ten seconds of reports
		let packed = pack(&steady(20000), 1 << 16);
		assert_eq!(span(&mut io::Cursor::new(&packed)).unwrap(), Some((7_000_000, 7_000_000 + 19999 * 500)));
		assert_eq!(blocks_of(&packed), 20);
		// lines without a time, a few of them long: a block ends with the line that fills it (here at 1000 bytes)
		let text =
			[&b"# a short one\n"[..], &[b'#'; 700], b"\n", &[b'-'; 400], b"\n# next\n", &[b'='; 999], b"\n# last\n"]
				.concat();
		let mut w = PackWriter::new(Vec::new()).unwrap();
		w.packer.block_text = 1000;
		w.write(&text).unwrap();
		let packed = w.finish().unwrap();
		assert_eq!(blocks_of(&packed), 3);
		assert!(unpack(&packed).unwrap() == text);
	}

	#[test]
	fn the_span_is_the_earliest_and_latest_report_times() {
		let text = b"# a comment\r\nR 1000 X 2102 -1 -1 I 23\r\nR 900 X 2102 -1 -1 I 23\r\nA 5 X Y\r\nR 2500 X 2102 -1 -1 I 23\r\n# end\r\n";
		let packed = pack(text, 1000);
		assert_eq!(span(&mut io::Cursor::new(&packed)).unwrap(), Some((900, 2500)));
		// no report, no span; a text recording has none to read off
		assert_eq!(span(&mut io::Cursor::new(pack(b"# only\n", 10))).unwrap(), None);
		assert_eq!(span(&mut io::Cursor::new(&text[..])).unwrap(), None);
		// a block cut short does not count
		let two = [&text[..], &b"R 9000 X 2102 -1 -1 I 23\r\n"[..]].concat();
		let mut w = PackWriter::new(Vec::new()).unwrap();
		w.write(text).unwrap();
		w.end_block().unwrap();
		w.write(&two[text.len()..]).unwrap();
		let packed = w.finish().unwrap();
		assert_eq!(span(&mut io::Cursor::new(&packed)).unwrap(), Some((900, 9000)));
		assert_eq!(span(&mut io::Cursor::new(&packed[..packed.len() - 1])).unwrap(), Some((900, 2500)));
	}

	#[test]
	fn a_recording_cut_short_reads_to_its_last_whole_block() {
		let text = recording(5, 9000);
		let packed = pack(&text, 1 << 16);
		// where each block ends in the file, and in the text
		let mut ends = vec![(HEADER, 0usize)];
		let mut u = Unpacker::new(&packed[..]).unwrap();
		while u.next_block().unwrap() {
			let head: [u8; BLOCK_HEADER] = packed[ends.last().unwrap().0..][..BLOCK_HEADER].try_into().unwrap();
			let last = *ends.last().unwrap();
			ends.push((last.0 + BLOCK_HEADER + block_size(&head).1 as usize + 4, last.1 + u.text.len()));
		}
		assert!(ends.len() > 4 && *ends.last().unwrap() == (packed.len(), text.len()));

		let mut rng = Rng(3);
		for _ in 0..200 {
			let cut = HEADER + rng.below((packed.len() - HEADER + 1) as u64) as usize;
			let whole = ends.iter().rev().find(|e| e.0 <= cut).unwrap();
			let mut u = Unpacker::new(&packed[..cut]).unwrap();
			let mut got = Vec::new();
			u.read_to_end(&mut got).unwrap();
			assert!(got == text[..whole.1], "cut at {cut}");
			assert_eq!(u.cut(), whole.0 != cut, "cut at {cut}");
		}
	}

	#[test]
	fn a_damaged_block_is_an_error_after_the_blocks_before_it() {
		let text = recording(6, 9000);
		let mut packed = pack(&text, 1 << 16);
		let first: [u8; BLOCK_HEADER] = packed[HEADER..][..BLOCK_HEADER].try_into().unwrap();
		let second = HEADER + BLOCK_HEADER + block_size(&first).1 as usize + 4;
		packed[second + BLOCK_HEADER + 40] ^= 0x10;
		let mut u = Unpacker::new(&packed[..]).unwrap();
		let mut got = Vec::new();
		let e = u.read_to_end(&mut got).unwrap_err();
		assert_eq!(e.kind(), io::ErrorKind::InvalidData);
		assert!(e.to_string().contains("checksum"), "{e}");
		assert!(!got.is_empty() && text.starts_with(&got));

		// a block made up to unpack to more than a block holds is not unpacked: the blocks before it are read (here
		// a block of one line, then one of the rest, and a bound between the two)
		let (line, rest) = text.split_at(text.iter().position(|&c| c == b'\n').unwrap() + 1);
		let mut w = PackWriter::new(Vec::new()).unwrap();
		w.write(line).unwrap();
		w.end_block().unwrap();
		w.write(rest).unwrap();
		let packed = w.finish().unwrap();
		let mut u = Unpacker::new(&packed[..]).unwrap();
		u.most = line.len();
		let mut got = Vec::new();
		let e = u.read_to_end(&mut got).unwrap_err();
		assert!(e.kind() == io::ErrorKind::InvalidData && e.to_string().contains("more than"), "{e}");
		assert!(got == line);
		// nor is one that names a device at more length than a name has (the names are kept)
		let mut p = Packer::new();
		p.line = Line {
			is_report: true,
			stream: STREAMS,
			name: vec![b'N'; MAX_NAME + 1],
			report: vec![0x23],
			..Line::default()
		};
		p.state.line(&mut p.coder, &mut p.line);
		p.lines = 1;
		let made = [&p.header()[..], &p.block()].concat();
		let e = unpack(&made).unwrap_err();
		assert!(e.kind() == io::ErrorKind::InvalidData && e.to_string().contains("more than"), "{e}");
	}

	#[test]
	fn only_a_packed_recording_of_this_version_is_read() {
		assert!(Unpacker::new(&b"R 1 X 2102 -1 -1 I 23\n"[..]).is_err());
		assert!(Unpacker::new(&b""[..]).is_err());
		let mut newer = Packer::new().header();
		newer[8] = VERSION + 1;
		let e = Unpacker::new(&newer[..]).err().unwrap();
		assert!(e.to_string().contains(&format!("version {}", VERSION + 1)), "{e}");
		newer[8] = 0;
		assert!(Unpacker::new(&newer[..]).is_err());
	}

	fn pack_as(version: u8, text: &[u8], piece: usize) -> Vec<u8> {
		let mut w = PackWriter::of_version(Vec::new(), version).unwrap();
		for p in text.chunks(piece) {
			w.write(p).unwrap();
		}
		w.finish().unwrap()
	}

	/// A recording as the tap writes it now: its lines without what `strip` leaves out
	fn stripped(text: &[u8]) -> Vec<u8> {
		let mut out = Vec::with_capacity(text.len());
		for l in text.split_inclusive(|&b| b == b'\n') {
			let mut l = l.to_vec();
			crate::strip::line(&mut l);
			out.extend(l);
		}
		out
	}

	/// The format's first version is read as it was written: the recordings packed before the second are. What this
	/// build writes as the first is, to the byte, what the build before the second version wrote
	#[test]
	fn the_first_version_is_still_read() {
		for (text, piece, len, crc) in [
			(recording(5, 3000), 4096, 35950, 0x9910141c_u32),
			(recording(6, 3000), 4096, 35361, 0x86b9478b),
			(recording(9, 20000), 4096, 207179, 0x457895ca),
			(steady(20000), 1 << 16, 188166, 0x14ca62e7),
		] {
			let packed = pack_as(FIRST_VERSION, &text, piece);
			assert_eq!((packed.len(), crc32(&packed)), (len, crc));
			assert_eq!(packed[8], FIRST_VERSION);
			assert!(unpack(&packed).unwrap() == text);
			// and one of the first version read as the second, or the other way, is found out by its blocks
			let mut wrong = pack(&stripped(&text), piece);
			wrong[8] = FIRST_VERSION;
			assert!(unpack(&wrong).map_or(true, |t| t != stripped(&text)));
		}
	}

	/// A recording written without the bytes nothing reads packs to much less, and one with them to what the first
	/// version made of it
	#[test]
	fn zeros_after_the_packets_cost_next_to_nothing() {
		for seed in [5, 6, 9] {
			let whole = recording(seed, 20000);
			let cut = stripped(&whole);
			assert_eq!(cut.len(), whole.len());
			assert!(cut != whole);
			let (first, second, lean) = (pack_as(FIRST_VERSION, &whole, 4096), pack(&whole, 4096), pack(&cut, 4096));
			assert!(unpack(&second).unwrap() == whole);
			assert!(unpack(&lean).unwrap() == cut);
			assert!(second.len() * 200 < first.len() * 201, "{} against {}", second.len(), first.len());
			assert!(lean.len() * 10 < second.len() * 9, "{} against {}", lean.len(), second.len());
			// what was cut once has nothing more to lose
			assert!(stripped(&cut) == cut);
		}
	}

	#[test]
	fn either_form_opens_as_its_text() {
		let text = recording(8, 500);
		let dir = std::env::temp_dir().join(format!("sally-hid-pack-{}", std::process::id()));
		std::fs::create_dir_all(&dir).unwrap();
		let (plain, packed) = (dir.join("a.hidtap"), dir.join("a.hidtapz"));
		std::fs::write(&plain, &text).unwrap();
		std::fs::write(&packed, pack(&text, 1000)).unwrap();
		for path in [&plain, &packed] {
			let mut got = Vec::new();
			open(path).unwrap().read_to_end(&mut got).unwrap();
			assert!(got == text, "{}", path.display());
		}
		// the name does not decide
		std::fs::rename(&packed, dir.join("b.hidtap")).unwrap();
		let mut got = Vec::new();
		open(&dir.join("b.hidtap")).unwrap().read_to_end(&mut got).unwrap();
		assert!(got == text);
		std::fs::remove_dir_all(&dir).unwrap();

		assert_eq!(stem("sallytap_20261108_190003.hidtap"), Some("sallytap_20261108_190003"));
		assert_eq!(stem("sallytap_20261108_190003.hidtapz"), Some("sallytap_20261108_190003"));
		assert_eq!(stem("sallytap_20261108_190003_solver.txt"), None);
	}
}
