//! Text out as the C's and Python's text-mode streams write it: on Windows every "\n" goes out as "\r\n" (stdout and
//! stderr in text mode, as hidprobe's printf and the Python tools' print wrote them).

use std::io::Write;

/// A stream through the text mode, buffered
pub struct TextOut<W: Write> {
	out: std::io::BufWriter<W>,
	crlf: bool,
}

impl<W: Write> TextOut<W> {
	/// Through this system's text mode (CR LF on Windows)
	pub fn new(w: W) -> Self {
		Self::with_crlf(w, cfg!(windows))
	}

	/// Through a text mode that writes "\n" as "\r\n" (`crlf`) or as it is
	pub fn with_crlf(w: W, crlf: bool) -> Self {
		TextOut { out: std::io::BufWriter::with_capacity(1 << 16, w), crlf }
	}

	pub fn put(&mut self, text: &str) -> std::io::Result<()> {
		if !self.crlf {
			return self.out.write_all(text.as_bytes());
		}
		self.out.write_all(&sally_math::textmode::write(text.as_bytes()))
	}

	pub fn flush(&mut self) -> std::io::Result<()> {
		self.out.flush()
	}

	/// The stream, flushed
	#[cfg(test)]
	pub fn into_inner(self) -> std::io::Result<W> {
		self.out.into_inner().map_err(|e| e.into_error())
	}
}

/// printf / print: the text to stdout through the text mode
pub fn out(text: &str) {
	let mut o = TextOut::new(std::io::stdout().lock());
	let _ = o.put(text);
	let _ = o.flush();
}

/// The same to stderr
pub fn err(text: &str) {
	let mut o = TextOut::new(std::io::stderr().lock());
	let _ = o.put(text);
	let _ = o.flush();
}
