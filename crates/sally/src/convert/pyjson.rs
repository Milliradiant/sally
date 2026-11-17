//! Python's json.loads (CPython 3.13's json.decoder and _json.c), which the openhouse conversion reads the device
//! configs as: which documents it takes, what it says about the ones it does not (its messages, with their positions in
//! characters), a document given as bytes decoded as json.loads decodes it (json.detect_encoding, then the codec with
//! errors='surrogatepass', and the codec's own messages), and the values compared as Python's == compares them.
//!
//! Not followed: a lone surrogate in a string (an escape such as \ud800, or one surrogatepass let through) is U+FFFD
//! here, so two strings that differ only there compare equal; Python gives up on a deeply nested document where its
//! own stack does (a RecursionError, about 10 000 levels on Linux), this at 512 (a device config nests a few).

/// A JSON value as Python holds it
#[derive(Clone, Debug)]
pub enum Value {
	Null,
	Bool(bool),
	/// an int, exact: its sign and its digits (no leading zeros; zero is "0", not negative)
	Int(bool, String),
	Float(f64),
	Str(String),
	List(Vec<Value>),
	/// a dict: its keys in the order they were first seen, each with the last value it was given
	Dict(Vec<(String, Value)>),
}

/// What json.loads raised
#[derive(Clone, Debug, PartialEq)]
pub enum Error {
	/// a ValueError (JSONDecodeError and UnicodeDecodeError among them): its text
	Value(String),
	/// a RecursionError: its text (not a ValueError)
	Recursion(String),
}

impl std::fmt::Display for Error {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			Error::Value(m) | Error::Recursion(m) => f.write_str(m),
		}
	}
}

/// The nesting at which this reader gives up, as Python's would on its stack
const MAX_DEPTH: usize = 512;

/// sys.int_info.default_max_str_digits
const MAX_INT_DIGITS: usize = 4300;

impl Value {
	/// type(v).__name__
	pub fn type_name(&self) -> &'static str {
		match self {
			Value::Null => "NoneType",
			Value::Bool(_) => "bool",
			Value::Int(..) => "int",
			Value::Float(_) => "float",
			Value::Str(_) => "str",
			Value::List(_) => "list",
			Value::Dict(_) => "dict",
		}
	}

	/// bool(v)
	pub fn truthy(&self) -> bool {
		match self {
			Value::Null => false,
			Value::Bool(b) => *b,
			Value::Int(_, d) => d != "0",
			Value::Float(f) => *f != 0.0,
			Value::Str(s) => !s.is_empty(),
			Value::List(l) => !l.is_empty(),
			Value::Dict(d) => !d.is_empty(),
		}
	}

	/// dict.get(key) (None where the value is not a dict)
	pub fn get(&self, key: &str) -> Option<&Value> {
		match self {
			Value::Dict(d) => d.iter().find(|(k, _)| k == key).map(|(_, v)| v),
			_ => None,
		}
	}
}

/// `a == b` as Python compares them: an int, a float and a bool by their values, exactly (True is 1); a list or a
/// dict item by item. Inside a list or a dict, Python takes the same object as equal before it compares: json's NaN
/// is one object, so a NaN there equals a NaN, where `a == b` of two NaNs is False
pub fn py_eq(a: &Value, b: &Value) -> bool {
	eq(a, b, false)
}

fn eq(a: &Value, b: &Value, inside: bool) -> bool {
	use Value::*;
	match (a, b) {
		(Null, Null) => true,
		(Str(x), Str(y)) => x == y,
		(List(x), List(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| eq(p, q, true)),
		(Dict(x), Dict(y)) => x.len() == y.len() && x.iter().all(|(k, v)| b.get(k).is_some_and(|w| eq(v, w, true))),
		(Float(x), Float(y)) => x == y || (inside && x.is_nan() && y.is_nan()),
		_ => match (number(a), number(b)) {
			(Some(Num::Int(p, q)), Some(Num::Int(r, s))) => p == r && q == s,
			(Some(Num::Int(neg, digits)), Some(Num::Float(f))) | (Some(Num::Float(f)), Some(Num::Int(neg, digits))) => {
				int_eq_float(neg, digits, f)
			}
			_ => false,
		},
	}
}

enum Num<'a> {
	Int(bool, &'a str),
	Float(f64),
}

fn number(v: &Value) -> Option<Num<'_>> {
	match v {
		Value::Bool(b) => Some(Num::Int(false, if *b { "1" } else { "0" })),
		Value::Int(neg, d) => Some(Num::Int(*neg, d)),
		Value::Float(f) => Some(Num::Float(*f)),
		_ => None,
	}
}

/// An int against a float, exactly: equal only where the float is a whole number of the same value
fn int_eq_float(neg: bool, digits: &str, f: f64) -> bool {
	if !f.is_finite() || f.fract() != 0.0 {
		return false;
	}
	if f == 0.0 {
		return digits == "0";
	}
	// a whole float's decimal digits, exact
	neg == (f < 0.0) && format!("{:.0}", f.abs()) == digits
}

/// json.loads of a str: a document that starts with a BOM refused, then JSONDecoder.decode
pub fn loads_text(text: &str) -> Result<Value, Error> {
	let s: Vec<char> = text.chars().collect();
	if s.first() == Some(&'\u{feff}') {
		return Err(decode_error(&s, "Unexpected UTF-8 BOM (decode using utf-8-sig)", 0));
	}
	decode(&s)
}

/// json.loads of bytes: decoded as json.detect_encoding says, with errors='surrogatepass', then as a str (without
/// the str's BOM check)
pub fn loads_bytes(b: &[u8]) -> Result<Value, Error> {
	let text = decode_bytes(b).map_err(Error::Value)?;
	let s: Vec<char> = text.chars().collect();
	decode(&s)
}

/// json.detect_encoding, then bytes.decode(encoding, 'surrogatepass'): the text, or the UnicodeDecodeError's text
pub fn decode_bytes(b: &[u8]) -> Result<String, String> {
	let starts = |p: &[u8]| b.starts_with(p);
	if starts(&[0, 0, 0xfe, 0xff]) {
		return decode_utf32(b, 4, true);
	}
	if starts(&[0xff, 0xfe, 0, 0]) {
		return decode_utf32(b, 4, false);
	}
	if starts(&[0xfe, 0xff]) {
		return decode_utf16(b, 2, true);
	}
	if starts(&[0xff, 0xfe]) {
		return decode_utf16(b, 2, false);
	}
	if starts(&[0xef, 0xbb, 0xbf]) {
		return decode_utf8(&b[3..]);
	}
	if b.len() >= 4 {
		if b[0] == 0 {
			return if b[1] != 0 { decode_utf16(b, 0, true) } else { decode_utf32(b, 0, true) };
		}
		if b[1] == 0 {
			return if b[2] != 0 || b[3] != 0 { decode_utf16(b, 0, false) } else { decode_utf32(b, 0, false) };
		}
	} else if b.len() == 2 {
		if b[0] == 0 {
			return decode_utf16(b, 0, true);
		}
		if b[1] == 0 {
			return decode_utf16(b, 0, false);
		}
	}
	decode_utf8(b)
}

/// UnicodeDecodeError's text
fn decode_message(codec: &str, b: &[u8], start: usize, end: usize, reason: &str) -> String {
	if end == start + 1 {
		format!("'{codec}' codec can't decode byte 0x{:02x} in position {start}: {reason}", b[start])
	} else {
		format!("'{codec}' codec can't decode bytes in position {start}-{}: {reason}", end - 1)
	}
}

/// A surrogate, which surrogatepass lets through: U+FFFD here
const SURROGATE: char = '\u{fffd}';

/// The UTF-8 codec (CPython's utf8_decode, its error ranges), with surrogatepass: a surrogate's three bytes taken
fn decode_utf8(b: &[u8]) -> Result<String, String> {
	let n = b.len();
	let cont = |c: u8| (0x80..0xc0).contains(&c);
	let mut out = String::with_capacity(n);
	let mut i = 0;
	'chars: while i < n {
		let ch = b[i];
		if ch < 0x80 {
			out.push(ch as char);
			i += 1;
			continue;
		}
		// the error: its range and reason
		let bad: (usize, &str) = 'bad: {
			if ch < 0xc2 {
				break 'bad (1, "invalid start byte");
			}
			if ch < 0xe0 {
				if n - i < 2 {
					break 'bad (n - i, "unexpected end of data");
				}
				if !cont(b[i + 1]) {
					break 'bad (1, "invalid continuation byte");
				}
				out.push(char::from_u32(((ch as u32 & 0x1f) << 6) | (b[i + 1] as u32 & 0x3f)).unwrap_or(SURROGATE));
				i += 2;
				continue 'chars;
			}
			if ch < 0xf0 {
				if n - i < 3 {
					if n - i < 2 {
						break 'bad (n - i, "unexpected end of data");
					}
					let ch2 = b[i + 1];
					if !cont(ch2) || (if ch2 < 0xa0 { ch == 0xe0 } else { ch == 0xed }) {
						break 'bad (1, "invalid continuation byte");
					}
					break 'bad (n - i, "unexpected end of data");
				}
				let (ch2, ch3) = (b[i + 1], b[i + 2]);
				if !cont(ch2) || (ch == 0xe0 && ch2 < 0xa0) || (ch == 0xed && ch2 >= 0xa0) {
					break 'bad (1, "invalid continuation byte");
				}
				if !cont(ch3) {
					break 'bad (2, "invalid continuation byte");
				}
				let c = ((ch as u32 & 0x0f) << 12) | ((ch2 as u32 & 0x3f) << 6) | (ch3 as u32 & 0x3f);
				out.push(char::from_u32(c).unwrap_or(SURROGATE));
				i += 3;
				continue 'chars;
			}
			if ch < 0xf5 {
				if n - i < 4 {
					if n - i < 2 {
						break 'bad (n - i, "unexpected end of data");
					}
					let ch2 = b[i + 1];
					if !cont(ch2) || (if ch2 < 0x90 { ch == 0xf0 } else { ch == 0xf4 }) {
						break 'bad (1, "invalid continuation byte");
					}
					if n - i < 3 {
						break 'bad (n - i, "unexpected end of data");
					}
					if !cont(b[i + 2]) {
						break 'bad (2, "invalid continuation byte");
					}
					break 'bad (n - i, "unexpected end of data");
				}
				let (ch2, ch3, ch4) = (b[i + 1], b[i + 2], b[i + 3]);
				if !cont(ch2) || (ch == 0xf0 && ch2 < 0x90) || (ch == 0xf4 && ch2 >= 0x90) {
					break 'bad (1, "invalid continuation byte");
				}
				if !cont(ch3) {
					break 'bad (2, "invalid continuation byte");
				}
				if !cont(ch4) {
					break 'bad (3, "invalid continuation byte");
				}
				let c = ((ch as u32 & 0x07) << 18)
					| ((ch2 as u32 & 0x3f) << 12)
					| ((ch3 as u32 & 0x3f) << 6)
					| (ch4 as u32 & 0x3f);
				out.push(char::from_u32(c).unwrap_or(SURROGATE));
				i += 4;
				continue 'chars;
			}
			(1, "invalid start byte")
		};
		// surrogatepass: three bytes that encode a surrogate
		if n - i >= 3 && b[i] & 0xf0 == 0xe0 && b[i + 1] & 0xc0 == 0x80 && b[i + 2] & 0xc0 == 0x80 {
			let c = ((b[i] as u32 & 0x0f) << 12) | ((b[i + 1] as u32 & 0x3f) << 6) | (b[i + 2] as u32 & 0x3f);
			if (0xd800..0xe000).contains(&c) {
				out.push(SURROGATE);
				i += 3;
				continue 'chars;
			}
		}
		return Err(decode_message("utf-8", b, i, i + bad.0, bad.1));
	}
	Ok(out)
}

/// The UTF-16 codec from `from` (past a BOM), with surrogatepass: every surrogate taken, paired or not; only a last odd
/// byte fails
fn decode_utf16(b: &[u8], from: usize, big: bool) -> Result<String, String> {
	let codec = if big { "utf-16-be" } else { "utf-16-le" };
	let units = b[from..].chunks_exact(2).map(|u| {
		if big {
			u16::from_be_bytes([u[0], u[1]])
		} else {
			u16::from_le_bytes([u[0], u[1]])
		}
	});
	let out = char::decode_utf16(units).map(|c| c.unwrap_or(SURROGATE)).collect();
	if (b.len() - from) % 2 == 1 {
		return Err(decode_message(codec, b, b.len() - 1, b.len(), "truncated data"));
	}
	Ok(out)
}

/// The UTF-32 codec from `from` (past a BOM), with surrogatepass: a surrogate taken, a code point past U+10FFFF and
/// a last unit cut short failing
fn decode_utf32(b: &[u8], from: usize, big: bool) -> Result<String, String> {
	let codec = if big { "utf-32-be" } else { "utf-32-le" };
	let mut out = String::new();
	let mut i = from;
	while i < b.len() {
		if b.len() - i < 4 {
			return Err(decode_message(codec, b, i, b.len(), "truncated data"));
		}
		let u = [b[i], b[i + 1], b[i + 2], b[i + 3]];
		let c = if big { u32::from_be_bytes(u) } else { u32::from_le_bytes(u) };
		if c > 0x10ffff {
			return Err(decode_message(codec, b, i, i + 4, "code point not in range(0x110000)"));
		}
		out.push(char::from_u32(c).unwrap_or(SURROGATE));
		i += 4;
	}
	Ok(out)
}

/// JSONDecodeError's text: the message, the line and column, the character
fn decode_error(s: &[char], msg: &str, pos: usize) -> Error {
	let before = &s[..pos.min(s.len())];
	let line = before.iter().filter(|&&c| c == '\n').count() + 1;
	let col = match before.iter().rposition(|&c| c == '\n') {
		Some(nl) => pos - nl,
		None => pos + 1,
	};
	Error::Value(format!("{msg}: line {line} column {col} (char {pos})"))
}

/// What the scanner stops with
enum Fail {
	/// StopIteration(idx): "Expecting value" at idx, as JSONDecoder.raw_decode turns it
	Stop(usize),
	/// raise_errmsg
	At(&'static str, usize),
	/// any other ValueError
	Plain(String),
	/// a RecursionError
	Deep(&'static str),
}

/// JSONDecoder.decode: white space, one value, white space, the end
fn decode(s: &[char]) -> Result<Value, Error> {
	let mut p = Scanner { s, depth: 0 };
	let idx = p.ws(0);
	let r = p.scan_once(idx).and_then(|(v, end)| {
		let end = p.ws(end);
		if end != s.len() {
			return Err(Fail::At("Extra data", end));
		}
		Ok(v)
	});
	r.map_err(|f| match f {
		Fail::Stop(i) => decode_error(s, "Expecting value", i),
		Fail::At(m, i) => decode_error(s, m, i),
		Fail::Plain(m) => Error::Value(m),
		Fail::Deep(what) => Error::Recursion(format!("maximum recursion depth exceeded{what}")),
	})
}

/// _json.c's scanner over the characters
struct Scanner<'a> {
	s: &'a [char],
	depth: usize,
}

fn is_ws(c: char) -> bool {
	matches!(c, ' ' | '\t' | '\n' | '\r')
}

impl Scanner<'_> {
	fn ws(&self, mut i: usize) -> usize {
		while i < self.s.len() && is_ws(self.s[i]) {
			i += 1;
		}
		i
	}

	fn at(&self, i: usize) -> Option<char> {
		self.s.get(i).copied()
	}

	/// The characters at `i` are `word`
	fn word(&self, i: usize, word: &str) -> bool {
		let n = word.chars().count();
		i + n <= self.s.len() && self.s[i..i + n].iter().copied().eq(word.chars())
	}

	/// scan_once_unicode
	fn scan_once(&mut self, idx: usize) -> Result<(Value, usize), Fail> {
		let Some(c) = self.at(idx) else { return Err(Fail::Stop(idx)) };
		match c {
			'"' => return self.scanstring(idx + 1).map(|(s, e)| (Value::Str(s), e)),
			'{' | '[' => {
				if self.depth >= MAX_DEPTH {
					return Err(Fail::Deep(if c == '{' {
						" while decoding a JSON object from a unicode string"
					} else {
						" while decoding a JSON array from a unicode string"
					}));
				}
				self.depth += 1;
				let r = if c == '{' { self.object(idx + 1) } else { self.array(idx + 1) };
				self.depth -= 1;
				return r;
			}
			'n' if self.word(idx, "null") => return Ok((Value::Null, idx + 4)),
			't' if self.word(idx, "true") => return Ok((Value::Bool(true), idx + 4)),
			'f' if self.word(idx, "false") => return Ok((Value::Bool(false), idx + 5)),
			'N' if self.word(idx, "NaN") => return Ok((Value::Float(f64::NAN), idx + 3)),
			'I' if self.word(idx, "Infinity") => return Ok((Value::Float(f64::INFINITY), idx + 8)),
			'-' if self.word(idx, "-Infinity") => return Ok((Value::Float(f64::NEG_INFINITY), idx + 9)),
			_ => {}
		}
		self.number(idx)
	}

	/// _parse_object_unicode, from the character after the brace
	fn object(&mut self, mut idx: usize) -> Result<(Value, usize), Fail> {
		let len = self.s.len();
		let mut items: Vec<(String, Value)> = Vec::new();
		let mut index: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
		idx = self.ws(idx);
		if idx >= len || self.s[idx] != '}' {
			loop {
				if idx >= len || self.s[idx] != '"' {
					return Err(Fail::At("Expecting property name enclosed in double quotes", idx));
				}
				let (key, next) = self.scanstring(idx + 1)?;
				idx = self.ws(next);
				if idx >= len || self.s[idx] != ':' {
					return Err(Fail::At("Expecting ':' delimiter", idx));
				}
				idx = self.ws(idx + 1);
				let (val, next) = self.scan_once(idx)?;
				match index.get(&key) {
					Some(&at) => items[at].1 = val,
					None => {
						index.insert(key.clone(), items.len());
						items.push((key, val));
					}
				}
				idx = self.ws(next);
				if idx < len && self.s[idx] == '}' {
					break;
				}
				if idx >= len || self.s[idx] != ',' {
					return Err(Fail::At("Expecting ',' delimiter", idx));
				}
				let comma = idx;
				idx = self.ws(idx + 1);
				if idx < len && self.s[idx] == '}' {
					return Err(Fail::At("Illegal trailing comma before end of object", comma));
				}
			}
		}
		Ok((Value::Dict(items), idx + 1))
	}

	/// _parse_array_unicode, from the character after the bracket
	fn array(&mut self, mut idx: usize) -> Result<(Value, usize), Fail> {
		let len = self.s.len();
		let mut items = Vec::new();
		idx = self.ws(idx);
		if idx >= len || self.s[idx] != ']' {
			loop {
				let (val, next) = self.scan_once(idx)?;
				items.push(val);
				idx = self.ws(next);
				if idx < len && self.s[idx] == ']' {
					break;
				}
				if idx >= len || self.s[idx] != ',' {
					return Err(Fail::At("Expecting ',' delimiter", idx));
				}
				let comma = idx;
				idx = self.ws(idx + 1);
				if idx < len && self.s[idx] == ']' {
					return Err(Fail::At("Illegal trailing comma before end of array", comma));
				}
			}
		}
		if idx >= len || self.s[idx] != ']' {
			return Err(Fail::At("Expecting value", len.saturating_sub(1)));
		}
		Ok((Value::List(items), idx + 1))
	}

	/// _match_number_unicode: an int, or a float where a fraction or an exponent follows
	fn number(&self, start: usize) -> Result<(Value, usize), Fail> {
		let s = self.s;
		let end_idx = s.len() - 1;
		let digit = |i: usize| s.get(i).is_some_and(|c| c.is_ascii_digit());
		let mut idx = start;
		if s[idx] == '-' {
			idx += 1;
			if idx > end_idx {
				return Err(Fail::Stop(start));
			}
		}
		if ('1'..='9').contains(&s[idx]) {
			idx += 1;
			while idx <= end_idx && digit(idx) {
				idx += 1;
			}
		} else if s[idx] == '0' {
			idx += 1;
		} else {
			return Err(Fail::Stop(start));
		}
		let mut is_float = false;
		if idx < end_idx && s[idx] == '.' && digit(idx + 1) {
			is_float = true;
			idx += 2;
			while idx <= end_idx && digit(idx) {
				idx += 1;
			}
		}
		if idx < end_idx && (s[idx] == 'e' || s[idx] == 'E') {
			let e_start = idx;
			idx += 1;
			if idx < end_idx && (s[idx] == '-' || s[idx] == '+') {
				idx += 1;
			}
			while idx <= end_idx && digit(idx) {
				idx += 1;
			}
			if digit(idx - 1) {
				is_float = true;
			} else {
				idx = e_start;
			}
		}
		let text: String = s[start..idx].iter().collect();
		if is_float {
			return Ok((Value::Float(text.parse().unwrap_or(f64::NAN)), idx));
		}
		let (neg, digits) = match text.strip_prefix('-') {
			Some(d) => (true, d),
			None => (false, text.as_str()),
		};
		if digits.len() > MAX_INT_DIGITS {
			return Err(Fail::Plain(format!(
				"Exceeds the limit ({MAX_INT_DIGITS} digits) for integer string conversion: value has {} digits; use \
				 sys.set_int_max_str_digits() to increase the limit",
				digits.len()
			)));
		}
		Ok((Value::Int(neg && digits != "0", digits.to_string()), idx))
	}

	/// scanstring_unicode, strict: from the character after the opening quote to the one after the closing quote
	fn scanstring(&self, mut end: usize) -> Result<(String, usize), Fail> {
		let s = self.s;
		let len = s.len();
		let begin = end - 1;
		let mut out = String::new();
		let hex4 = |at: usize| -> Option<u32> {
			let mut v = 0;
			for c in &s[at..at + 4] {
				v = (v << 4) | c.to_digit(16)?;
			}
			Some(v)
		};
		loop {
			let mut next = end;
			let mut c = None;
			while next < len {
				let d = s[next];
				if d == '"' || d == '\\' {
					c = Some(d);
					break;
				}
				if (d as u32) <= 0x1f {
					return Err(Fail::At("Invalid control character at", next));
				}
				next += 1;
			}
			let Some(c) = c else { return Err(Fail::At("Unterminated string starting at", begin)) };
			out.extend(&s[end..next]);
			next += 1;
			if c == '"' {
				end = next;
				break;
			}
			if next == len {
				return Err(Fail::At("Unterminated string starting at", begin));
			}
			let c = s[next];
			if c != 'u' {
				end = next + 1;
				out.push(match c {
					'"' => '"',
					'\\' => '\\',
					'/' => '/',
					'b' => '\u{8}',
					'f' => '\u{c}',
					'n' => '\n',
					'r' => '\r',
					't' => '\t',
					_ => return Err(Fail::At("Invalid \\escape", end - 2)),
				});
			} else {
				next += 1;
				end = next + 4;
				if end > len {
					return Err(Fail::At("Invalid \\uXXXX escape", next - 1));
				}
				let Some(mut cp) = hex4(next) else { return Err(Fail::At("Invalid \\uXXXX escape", end - 5)) };
				// a surrogate pair
				if (0xd800..0xdc00).contains(&cp) && end + 6 < len && s[end] == '\\' && s[end + 1] == 'u' {
					let Some(low) = hex4(end + 2) else { return Err(Fail::At("Invalid \\uXXXX escape", end + 1)) };
					if (0xdc00..0xe000).contains(&low) {
						cp = 0x10000 + (((cp - 0xd800) << 10) | (low - 0xdc00));
						end += 6;
					}
				}
				out.push(char::from_u32(cp).unwrap_or(SURROGATE));
			}
		}
		Ok((out, end))
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn err(s: &str) -> String {
		match loads_text(s) {
			Err(e) => e.to_string(),
			Ok(v) => format!("ok {v:?}"),
		}
	}

	fn berr(b: &[u8]) -> String {
		match loads_bytes(b) {
			Err(e) => e.to_string(),
			Ok(v) => format!("ok {v:?}"),
		}
	}

	/// Python 3.13's messages for the same documents
	#[test]
	fn its_messages_are_pythons() {
		let cases = [
			("", "Expecting value: line 1 column 1 (char 0)"),
			(" ", "Expecting value: line 1 column 2 (char 1)"),
			("{", "Expecting property name enclosed in double quotes: line 1 column 2 (char 1)"),
			("{\"a\" 1}", "Expecting ':' delimiter: line 1 column 6 (char 5)"),
			("{\"a\":1,}", "Illegal trailing comma before end of object: line 1 column 7 (char 6)"),
			("[1,]", "Illegal trailing comma before end of array: line 1 column 3 (char 2)"),
			("[1 2]", "Expecting ',' delimiter: line 1 column 4 (char 3)"),
			("\"\\x\"", "Invalid \\escape: line 1 column 2 (char 1)"),
			("\"\\u12\"", "Invalid \\uXXXX escape: line 1 column 3 (char 2)"),
			("\"\\u1234", "Unterminated string starting at: line 1 column 1 (char 0)"),
			("\"a\nb\"", "Invalid control character at: line 1 column 3 (char 2)"),
			("{\"a\":tru}", "Expecting value: line 1 column 6 (char 5)"),
			("1 2", "Extra data: line 1 column 3 (char 2)"),
			("-", "Expecting value: line 1 column 1 (char 0)"),
			("01", "Extra data: line 1 column 2 (char 1)"),
			("1e", "Extra data: line 1 column 2 (char 1)"),
			("\u{feff}{}", "Unexpected UTF-8 BOM (decode using utf-8-sig): line 1 column 1 (char 0)"),
			("nan", "Expecting value: line 1 column 1 (char 0)"),
			("[-]", "Expecting value: line 1 column 2 (char 1)"),
			("[1,,2]", "Expecting value: line 1 column 4 (char 3)"),
			("{\"a\":1,,}", "Expecting property name enclosed in double quotes: line 1 column 8 (char 7)"),
			("\"\\ud800\\uzzzz\"", "Invalid \\uXXXX escape: line 1 column 9 (char 8)"),
			("\n\n  [1,\n  2,]", "Illegal trailing comma before end of array: line 4 column 4 (char 11)"),
			("{\"a\":1}\n\n x", "Extra data: line 3 column 2 (char 10)"),
			("[[[]", "Expecting ',' delimiter: line 1 column 5 (char 4)"),
			("{\"a\":[1,{\"b\":}]}", "Expecting value: line 1 column 14 (char 13)"),
			("1E+5x", "Extra data: line 1 column 5 (char 4)"),
		];
		for (doc, want) in cases {
			assert_eq!(err(doc), want, "{doc:?}");
		}
		let long = "9".repeat(4301);
		assert_eq!(
			err(&format!("-{long}")),
			"Exceeds the limit (4300 digits) for integer string conversion: value has 4301 digits; use \
			 sys.set_int_max_str_digits() to increase the limit"
		);
		assert!(matches!(loads_text(&"[".repeat(MAX_DEPTH + 1)), Err(Error::Recursion(_))));
	}

	#[test]
	fn bytes_decode_as_pythons_codecs_do() {
		let cases: [(&[u8], &str); 9] = [
			(b"\xff{}", "'utf-8' codec can't decode byte 0xff in position 0: invalid start byte"),
			(b"{\"a\":\"\xe0\x80\"}", "'utf-8' codec can't decode byte 0xe0 in position 6: invalid continuation byte"),
			(b"{\"a\":\"\xe2\x82", "'utf-8' codec can't decode bytes in position 6-7: unexpected end of data"),
			(b"\xef\xbb\xbf\xff", "'utf-8' codec can't decode byte 0xff in position 0: invalid start byte"),
			(b"\xff\xfe{\x00}", "'utf-16-le' codec can't decode byte 0x7d in position 4: truncated data"),
			(b"{\x00}\x00\x00", "'utf-16-le' codec can't decode byte 0x00 in position 4: truncated data"),
			(
				b"\x00\x00\x00{\x00\x11\x00\x00",
				"'utf-32-be' codec can't decode bytes in position 4-7: code point not in range(0x110000)",
			),
			(b"\xc3", "'utf-8' codec can't decode byte 0xc3 in position 0: unexpected end of data"),
			(b"\"\xf4\x90\x80\x80\"", "'utf-8' codec can't decode byte 0xf4 in position 1: invalid continuation byte"),
		];
		for (doc, want) in cases {
			assert_eq!(berr(doc), want, "{doc:?}");
		}
		// what decodes: a BOM's encodings, surrogates passed
		for doc in [
			&b"\xef\xbb\xbf{\"a\":1}"[..],
			b"{\x00\"\x00a\x00\"\x00:\x001\x00}\x00",
			b"\xff\xfe{\x00}\x00",
			b"\xfe\xff\x00{\x00}",
			b"{\x00\x00\x00}\x00\x00\x00",
			b"\x00\x00\x00{\x00\x00\x00}",
			b"{\"a\":\"\xed\xa0\x80\"}",
		] {
			assert!(loads_bytes(doc).is_ok(), "{doc:?}");
		}
		assert_eq!(berr(b"\xed\xa0A"), "'utf-8' codec can't decode byte 0xed in position 0: invalid continuation byte");
	}

	#[test]
	fn values_compare_as_pythons_do() {
		let v = |s: &str| loads_text(s).unwrap();
		assert!(py_eq(&v("1"), &v("1.0")));
		assert!(py_eq(&v("true"), &v("1")));
		assert!(py_eq(&v("[true]"), &v("[1.0]")));
		assert!(!py_eq(&v("100000000000000000000000"), &v("1e23")));
		assert!(py_eq(&v("99999999999999991611392"), &v("1e23")));
		assert!(py_eq(&v("-0"), &v("0.0")));
		assert!(!py_eq(&v("NaN"), &v("NaN")));
		assert!(py_eq(&v("[NaN]"), &v("[NaN]")));
		assert!(py_eq(&v("{\"a\":NaN}"), &v("{\"a\":NaN}")));
		assert!(py_eq(&v("[1e400]"), &v("[1e401]")));
		assert!(!py_eq(&v("\"1\""), &v("1")));
		assert!(py_eq(&v("{\"a\":1,\"b\":[2]}"), &v("{\"b\":[2.0],\"a\":1}")));
		// a key twice: its last value, where it was first
		let d = v("{\"a\":1,\"b\":2,\"a\":3}");
		assert!(py_eq(d.get("a").unwrap(), &v("3")));
		let Value::Dict(items) = &d else { panic!() };
		assert_eq!(items.iter().map(|(k, _)| k.as_str()).collect::<Vec<_>>(), ["a", "b"]);
		assert_eq!(v("\"\\ud83d\\ude00\\/\"").get("x").map(|_| ()), None);
		let Value::Str(s) = v("\"\\ud83d\\ude00\\/\\n\"") else { panic!() };
		assert_eq!(s, "\u{1f600}/\n");
	}
}
