//! The lenient JSON the config file is written in: the walk over jsmn's tokens (sally-json) that calls the reader back
//! (json_helpers.c's json_run_callbacks), and the writers config_save prints with (json_write_*).
//!
//! It is not strict JSON. The file config_save writes has no braces around its top level and no comma between that
//! level's entries and the "lighthouseN" objects after them; any byte that is not structure starts an unquoted
//! primitive (`true`, `1.5`, `abc`) that runs to the next white space, comma, colon or closing bracket; strings keep
//! their escapes undecoded; and a file jsmn cannot tokenize (an unclosed string or bracket, a bad escape, a control
//! byte in a primitive) is not read at all.

use sally_json::{jsmn_parse, JsmnError, Token, TokenType};

use super::cnum::{self, c_str};

/// json_callbacks: what the walk over the tokens calls. A tag is the key the value stands under, "" for none (an
/// array's elements, and the C's missing stack entry: see json_run_callbacks)
pub(crate) trait JsonCallbacks {
	fn begin_object(&mut self, tag: &[u8]);
	fn end_object(&mut self, tag: &[u8]);
	fn begin_array(&mut self, tag: &[u8]);
	fn end_array(&mut self, tag: &[u8]);
	fn tag_value(&mut self, tag: &[u8], value: &[u8]);
}

/// What a value stands under in the walk (stack_entry_t's key)
#[derive(Clone, Copy)]
enum Key {
	/// the C's stack pointer is NULL (the top level is one object or array)
	Missing,
	/// stack_entry_t with key NULL (an array's element)
	None,
	/// the key's token
	Token(usize),
}

struct Walk {
	/// the text with a NUL written over the byte at every token's end, as json_run_callbacks does before the walk:
	/// every token's text then reads as a C string from its start. A NUL also lands on the byte after a closing
	/// bracket, which cuts a primitive right after it to ""
	text: Vec<u8>,
	tokens: Vec<Token>,
}

impl Walk {
	fn c_str_at(&self, start: i32) -> &[u8] {
		c_str(self.text.get(start.max(0) as usize..).unwrap_or(&[]))
	}

	/// json_stack_tag
	fn tag(&self, key: Key) -> &[u8] {
		match key {
			Key::Missing | Key::None => b"",
			Key::Token(k) => self.c_str_at(self.tokens[k].start),
		}
	}

	/// process_jsontok: calls back for the token at `t` and its children, returns how many tokens it took. An object
	/// takes its size in key / value pairs and an array its size in values, as jsmn counted them: where jsmn counted
	/// wrong (missing commas inside an object) the walk takes other tokens for keys and values, as the C does. `count`
	/// is the tokens left, off by one as in the C (an object passes its children count - j, not count - 1 - j); a token
	/// index past the last token is where the C reads past its token pool, and here takes nothing
	fn process(&self, cbs: &mut impl JsonCallbacks, key: Key, t: usize, count: i64) -> i64 {
		if count == 0 {
			return 0;
		}
		let Some(tok) = self.tokens.get(t) else {
			return 0;
		};
		match tok.kind {
			TokenType::Primitive | TokenType::String => {
				cbs.tag_value(self.tag(key), self.c_str_at(tok.start));
				1
			}
			TokenType::Object => {
				let mut j: i64 = 0;
				cbs.begin_object(self.tag(key));
				for _ in 0..tok.size {
					let k = t as i64 + 1 + j;
					j += 1;
					let value_key = if (k as usize) < self.tokens.len() { Key::Token(k as usize) } else { Key::None };
					j += self.process(cbs, value_key, t + 1 + j as usize, count - j);
				}
				cbs.end_object(self.tag(key));
				j + 1
			}
			TokenType::Array => {
				cbs.begin_array(self.tag(key));
				let mut j: i64 = 0;
				for _ in 0..tok.size {
					j += self.process(cbs, Key::None, t + 1 + j as usize, count - j);
				}
				cbs.end_array(self.tag(key));
				j + 1
			}
		}
	}
}

/// json_run_callbacks: tokenizes the text and walks the tokens, calling `cbs` back. A text jsmn refuses is not walked
/// at all. If the first token is a string or a primitive (the config file's headless top level) the tokens are read as
/// key, value, key, value...; otherwise the first token is walked with no key. The C passes a NULL stack entry there,
/// which its config callbacks dereference (a config file in braces crashes the C); the port gives it the tag ""
pub(crate) fn json_run_callbacks(cbs: &mut impl JsonCallbacks, js: &[u8]) -> Result<usize, JsmnError> {
	let tokens = jsmn_parse(js)?;
	let mut text = js.to_vec();
	// one byte more than the C's copy: a token ending at the very end NULs the byte past the C's buffer
	text.push(0);
	for t in &tokens {
		text[t.end as usize] = 0;
	}
	let items = tokens.len();
	if items == 0 {
		return Ok(0);
	}
	let walk = Walk { text, tokens };
	let no_global = !matches!(walk.tokens[0].kind, TokenType::Array | TokenType::Object);
	if no_global {
		let mut j = 0;
		while j < items {
			let key = Key::Token(j);
			j += 1;
			j += walk.process(cbs, key, j, (items - j) as i64) as usize;
		}
	} else {
		walk.process(cbs, Key::Missing, 0, items as i64);
	}
	Ok(items)
}

/// json_write_uint32: "tag":"<%u>"
pub(crate) fn write_uint32(out: &mut Vec<u8>, tag: &[u8], v: u32) {
	write_str(out, tag, v.to_string().as_bytes());
}

/// json_write_float, handed (float)value: "tag":"<%.12f of the float>"
pub(crate) fn write_float(out: &mut Vec<u8>, tag: &[u8], v: f32) {
	let mut value = Vec::new();
	cnum::fixed(&mut value, cnum::to_double(v), 12);
	write_str(out, tag, &value);
}

/// json_write_str: "tag":"value", the bytes as they are (a quote or a backslash in them is not escaped)
pub(crate) fn write_str(out: &mut Vec<u8>, tag: &[u8], v: &[u8]) {
	out.push(b'"');
	out.extend_from_slice(tag);
	out.extend_from_slice(b"\":\"");
	out.extend_from_slice(v);
	out.push(b'"');
}

/// json_write_double_array: "tag":["<%.12f>","<%.12f>",...]
pub(crate) fn write_double_array(out: &mut Vec<u8>, tag: &[u8], v: &[f64]) {
	out.push(b'"');
	out.extend_from_slice(tag);
	out.extend_from_slice(b"\":[");
	for (i, x) in v.iter().enumerate() {
		if i > 0 {
			out.push(b',');
		}
		out.push(b'"');
		cnum::fixed(out, *x, 12);
		out.push(b'"');
	}
	out.push(b']');
}

#[cfg(test)]
mod tests {
	use super::*;

	#[derive(Default)]
	struct Log(Vec<String>);

	impl JsonCallbacks for Log {
		fn begin_object(&mut self, tag: &[u8]) {
			self.0.push(format!("{{{}", String::from_utf8_lossy(tag)));
		}
		fn end_object(&mut self, tag: &[u8]) {
			self.0.push(format!("}}{}", String::from_utf8_lossy(tag)));
		}
		fn begin_array(&mut self, tag: &[u8]) {
			self.0.push(format!("[{}", String::from_utf8_lossy(tag)));
		}
		fn end_array(&mut self, tag: &[u8]) {
			self.0.push(format!("]{}", String::from_utf8_lossy(tag)));
		}
		fn tag_value(&mut self, tag: &[u8], value: &[u8]) {
			self.0.push(format!("{}={}", String::from_utf8_lossy(tag), String::from_utf8_lossy(value)));
		}
	}

	fn walk(text: &str) -> Result<Vec<String>, JsmnError> {
		let mut log = Log::default();
		json_run_callbacks(&mut log, text.as_bytes())?;
		Ok(log.0)
	}

	#[test]
	fn the_headless_top_level_reads_as_pairs() {
		let got =
			walk("\"v\":\"0\",\n\"x\":1.5\n\"lighthouse0\":{\n\"index\":\"0\",\n\"pose\":[\"1\",\"2\"]\n}\n").unwrap();
		assert_eq!(got, ["v=0", "x=1.5", "{lighthouse0", "index=0", "[pose", "=1", "=2", "]pose", "}lighthouse0"]);
	}

	#[test]
	fn escapes_stay_and_a_bracket_cuts_the_primitive_after_it() {
		assert_eq!(walk(r#""a":"x\"y\u00e9""#).unwrap(), [r#"a=x\"y\u00e9"#]);
		// the inner array's end NULs the '2'
		assert_eq!(walk("\"a\":[[1]2]").unwrap(), ["[a", "[", "=1", "]", "=", "]a"]);
		// at the top level a primitive after a closing bracket is a key, and with nothing after it no value is read
		assert_eq!(walk("\"a\":[1]b\n").unwrap(), ["[a", "=1", "]a"]);
	}

	#[test]
	fn a_text_jsmn_refuses_is_not_walked() {
		assert_eq!(walk("\"a\":\"1\",\"b\":\"2"), Err(JsmnError::Part));
		assert_eq!(walk("\"a\":[1}"), Err(JsmnError::Inval));
		assert_eq!(walk("\"a\":\"\\x\""), Err(JsmnError::Inval));
		assert_eq!(walk("\"a\":\x01"), Err(JsmnError::Inval));
		assert_eq!(walk("\"a\":\"1\"]"), Err(JsmnError::Inval));
	}
}
