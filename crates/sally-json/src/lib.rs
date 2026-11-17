//! jsmn.c: the JSON tokenizer sally's files are read with (the device configs, the config file), as the C
//! build compiles it: neither JSMN_STRICT nor JSMN_PARENT_LINKS is defined (sally's copy, whose token pool grows by
//! realloc, would not compile with either), so it is jsmn's lenient mode. It does not validate: a primitive is
//! anything up to a delimiter (`:`, `,`, `]`, `}`, white space), an object's members need no `:` or `,` between them,
//! a string's escapes are checked but not decoded, and the input ends at its first NUL. A token is a byte range of
//! the input.
#![forbid(unsafe_code)]

/// jsmntype_t, with the C's values (JSMN_UNDEFINED, 0, is never a token's)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TokenType {
	Object = 1,
	Array = 2,
	String = 3,
	Primitive = 4,
}

/// jsmntok_t: where a token's text starts and ends, and how many tokens jsmn counted as its children
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Token {
	pub kind: TokenType,
	pub start: i32,
	pub end: i32,
	pub size: i32,
}

/// enum jsmnerr. JSMN_ERROR_NOMEM cannot happen: the token pool grows as needed (jsmn_init's token_pool_size -1)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JsmnError {
	/// JSMN_ERROR_INVAL: an invalid character
	Inval,
	/// JSMN_ERROR_PART: the text ends inside a string, an object or an array
	Part,
}

/// jsmn_parser
struct Parser {
	pos: usize,
	/// the token the next one counts as a child of (-1 for none)
	toksuper: i64,
	tokens: Vec<Token>,
}

/// jsmn_parse_primitive: an unquoted value runs to white space, ',', ']', '}' or ':' (not strict), or to the end
fn parse_primitive(p: &mut Parser, js: &[u8]) -> Result<(), JsmnError> {
	let start = p.pos;
	while p.pos < js.len() && js[p.pos] != 0 {
		match js[p.pos] {
			b':' | b'\t' | b'\r' | b'\n' | b' ' | b',' | b']' | b'}' => break,
			// the C compares a signed char: a byte from 128 up is below 32 there, invalid either way
			c if !(32..127).contains(&c) => {
				p.pos = start;
				return Err(JsmnError::Inval);
			}
			_ => p.pos += 1,
		}
	}
	p.tokens.push(Token { kind: TokenType::Primitive, start: start as i32, end: p.pos as i32, size: 0 });
	p.pos -= 1;
	Ok(())
}

/// jsmn_parse_string: from the opening quote to the closing one; the escapes are checked, not decoded
fn parse_string(p: &mut Parser, js: &[u8]) -> Result<(), JsmnError> {
	let len = js.len();
	let start = p.pos;
	p.pos += 1;
	while p.pos < len && js[p.pos] != 0 {
		let c = js[p.pos];
		if c == b'"' {
			p.tokens.push(Token { kind: TokenType::String, start: start as i32 + 1, end: p.pos as i32, size: 0 });
			return Ok(());
		}
		if c == b'\\' && p.pos + 1 < len {
			p.pos += 1;
			match js[p.pos] {
				b'"' | b'/' | b'\\' | b'b' | b'f' | b'r' | b'n' | b't' => {}
				// \uXXXX, the four hex digits cut short by the end of the text
				b'u' => {
					p.pos += 1;
					let mut i = 0;
					while i < 4 && p.pos < len && js[p.pos] != 0 {
						if !js[p.pos].is_ascii_hexdigit() {
							p.pos = start;
							return Err(JsmnError::Inval);
						}
						p.pos += 1;
						i += 1;
					}
					p.pos -= 1;
				}
				_ => {
					p.pos = start;
					return Err(JsmnError::Inval);
				}
			}
		}
		p.pos += 1;
	}
	p.pos = start;
	Err(JsmnError::Part)
}

/// jsmn_parse: the tokens of the whole text, in the order they start. A string, a primitive or a bracket counts as a
/// child of `toksuper`: the open object or array, or after a ':' the token just before it (a key). A ',' after a key's
/// value goes back to the innermost open object or array, so without the comma the next key counts as the previous
/// key's child, not the object's (harmless at the top level, which no size describes)
pub fn jsmn_parse(js: &[u8]) -> Result<Vec<Token>, JsmnError> {
	let mut p = Parser { pos: 0, toksuper: -1, tokens: Vec::new() };
	while p.pos < js.len() && js[p.pos] != 0 {
		let c = js[p.pos];
		match c {
			b'{' | b'[' => {
				let kind = if c == b'{' { TokenType::Object } else { TokenType::Array };
				p.tokens.push(Token { kind, start: p.pos as i32, end: -1, size: 0 });
				if p.toksuper != -1 {
					p.tokens[p.toksuper as usize].size += 1;
				}
				p.toksuper = p.tokens.len() as i64 - 1;
			}
			b'}' | b']' => {
				let kind = if c == b'}' { TokenType::Object } else { TokenType::Array };
				let mut i = p.tokens.len() as i64 - 1;
				while i >= 0 {
					let t = &mut p.tokens[i as usize];
					if t.start != -1 && t.end == -1 {
						if t.kind != kind {
							return Err(JsmnError::Inval);
						}
						p.toksuper = -1;
						t.end = p.pos as i32 + 1;
						break;
					}
					i -= 1;
				}
				// "Error if unmatched closing bracket"
				if i == -1 {
					return Err(JsmnError::Inval);
				}
				while i >= 0 {
					let t = &p.tokens[i as usize];
					if t.start != -1 && t.end == -1 {
						p.toksuper = i;
						break;
					}
					i -= 1;
				}
			}
			b'"' => {
				parse_string(&mut p, js)?;
				if p.toksuper != -1 {
					p.tokens[p.toksuper as usize].size += 1;
				}
			}
			b'\t' | b'\r' | b'\n' | b' ' => {}
			b':' => p.toksuper = p.tokens.len() as i64 - 1,
			b',' => {
				if p.toksuper != -1
					&& !matches!(p.tokens[p.toksuper as usize].kind, TokenType::Array | TokenType::Object)
				{
					for i in (0..p.tokens.len()).rev() {
						let t = &p.tokens[i];
						if matches!(t.kind, TokenType::Array | TokenType::Object) && t.start != -1 && t.end == -1 {
							p.toksuper = i as i64;
							break;
						}
					}
				}
			}
			// "In non-strict mode every unquoted value is a primitive"
			_ => {
				parse_primitive(&mut p, js)?;
				if p.toksuper != -1 {
					p.tokens[p.toksuper as usize].size += 1;
				}
			}
		}
		p.pos += 1;
	}
	// "Unmatched opened object or array"
	if p.tokens.iter().any(|t| t.start != -1 && t.end == -1) {
		return Err(JsmnError::Part);
	}
	Ok(p.tokens)
}

impl JsmnError {
	/// The C's value: JSMN_ERROR_INVAL -2, JSMN_ERROR_PART -3
	pub fn code(self) -> i32 {
		match self {
			JsmnError::Inval => -2,
			JsmnError::Part => -3,
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn tok(kind: TokenType, start: i32, end: i32, size: i32) -> Token {
		Token { kind, start, end, size }
	}

	#[test]
	fn tokens_of_an_object() {
		let js = br#"{"a": [1, "x"], "b": {"c": true}}"#;
		let t = jsmn_parse(js).unwrap();
		use TokenType::*;
		assert_eq!(
			t,
			vec![
				tok(Object, 0, 33, 2),
				tok(String, 2, 3, 1),
				tok(Array, 6, 14, 2),
				tok(Primitive, 7, 8, 0),
				tok(String, 11, 12, 0),
				tok(String, 17, 18, 1),
				tok(Object, 21, 32, 1),
				tok(String, 23, 24, 1),
				tok(Primitive, 27, 31, 0),
			]
		);
	}

	#[test]
	fn lenient_mode() {
		use TokenType::*;
		// members without ':' or ',' are all children of the object; a primitive ends at ':'
		let t = jsmn_parse(br#"{"a" "b" c:d}"#).unwrap();
		assert_eq!(t[0], tok(Object, 0, 13, 3));
		assert_eq!(t[3], tok(Primitive, 9, 10, 1));
		assert_eq!(t[4], tok(Primitive, 11, 12, 0));
		// the input ends at a NUL; what follows a closed object is tokens too
		assert_eq!(jsmn_parse(b"{}\0{").unwrap().len(), 1);
		assert_eq!(jsmn_parse(b"{} 12").unwrap()[1], tok(Primitive, 3, 5, 0));
		// escapes are checked, not decoded
		assert_eq!(jsmn_parse(b"[\"\\u00e9\\n\"]").unwrap()[1], tok(String, 2, 10, 0));
	}

	#[test]
	fn errors() {
		use JsmnError::*;
		assert_eq!(jsmn_parse(b"{"), Err(Part));
		assert_eq!(jsmn_parse(b"[1, 2"), Err(Part));
		assert_eq!(jsmn_parse(br#"{"a"#), Err(Part));
		assert_eq!(jsmn_parse(b"}"), Err(Inval));
		assert_eq!(jsmn_parse(b"[}"), Err(Inval));
		assert_eq!(jsmn_parse(br#"["\q"]"#), Err(Inval));
		assert_eq!(jsmn_parse(br#"["\u00g0"]"#), Err(Inval));
		assert_eq!(jsmn_parse(b"[1\x7f]"), Err(Inval));
		assert_eq!(jsmn_parse(b"[\xc3\xa9]"), Err(Inval));
		// a string may hold any byte but NUL; a backslash as the last byte is not an escape
		assert_eq!(jsmn_parse(b"[\"\xc3\xa9\x01\"]").unwrap().len(), 2);
		assert_eq!(jsmn_parse(b"[\"\\"), Err(Part));
		assert_eq!(jsmn_parse(b"").unwrap().len(), 0);
		assert_eq!(jsmn_parse(b"  \n").unwrap().len(), 0);
		assert_eq!((Inval.code(), Part.code()), (-2, -3));
	}
}
