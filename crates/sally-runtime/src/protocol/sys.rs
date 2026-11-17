//! The operating system's calls the protocol makes as the C makes them: inet_addr (the C library's on Linux,
//! Winsock's on Windows), the wall clock wall_epoch_s() reads, and on Windows SIO_UDP_CONNRESET turned off. Declared by
//! hand, the workspace taking no crate for them.

use std::net::{Ipv4Addr, UdpSocket};

#[cfg(not(windows))]
extern "C" {
	fn inet_addr(cp: *const std::ffi::c_char) -> u32;
}

#[cfg(windows)]
mod win {
	use std::ffi::{c_char, c_void};

	/// FILETIME: 100 ns intervals since 1601-01-01, in two halves
	#[repr(C)]
	pub struct FileTime {
		pub low: u32,
		pub high: u32,
	}

	/// _WSAIOW(IOC_VENDOR, 12): IOC_IN | IOC_VENDOR | 12
	pub const SIO_UDP_CONNRESET: u32 = 0x9800_000C;

	#[link(name = "ws2_32")]
	extern "system" {
		pub fn inet_addr(cp: *const c_char) -> u32;
		#[allow(clippy::too_many_arguments)]
		pub fn WSAIoctl(
			s: usize, io_control_code: u32, in_buffer: *const c_void, in_len: u32, out_buffer: *mut c_void,
			out_len: u32, bytes_returned: *mut u32, overlapped: *mut c_void, completion_routine: *mut c_void,
		) -> i32;
	}

	#[link(name = "kernel32")]
	extern "system" {
		pub fn GetSystemTimePreciseAsFileTime(time: *mut FileTime);
	}
}

const INADDR_NONE: u32 = 0xffff_ffff;

/// inet_addr(text): the address a C string names (one to four dotted parts, each decimal, octal or hex), or None for
/// INADDR_NONE, which "255.255.255.255" gives too. The text is a C string: it ends at a NUL
pub fn inet_addr_of(text: &[u8]) -> Option<Ipv4Addr> {
	let end = text.iter().position(|&c| c == 0).unwrap_or(text.len());
	let mut c = text[..end].to_vec();
	c.push(0);
	// SAFETY: a C string the call only reads
	#[cfg(not(windows))]
	let a = unsafe { inet_addr(c.as_ptr().cast()) };
	// SAFETY: as above (Winsock is started by the socket std made before, or the call fails and gives INADDR_NONE)
	#[cfg(windows)]
	let a = unsafe { win::inet_addr(c.as_ptr().cast()) };
	// s_addr is in network byte order: its bytes in memory are the address's
	(a != INADDR_NONE).then(|| Ipv4Addr::from(a.to_ne_bytes()))
}

/// wall_epoch_s(): the wall clock as seconds since the Unix epoch, as the C computes it. Linux: clock_gettime's
/// CLOCK_REALTIME (SystemTime's), tv_sec + tv_nsec * 1e-9
#[cfg(not(windows))]
pub fn wall_epoch() -> f64 {
	let d = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
	d.as_secs() as f64 + d.subsec_nanos() as f64 * 1e-9
}

/// wall_epoch_s(): Windows: GetSystemTimePreciseAsFileTime's 100 ns since 1601-01-01, (t - 116444736000000000) * 1e-7
#[cfg(windows)]
pub fn wall_epoch() -> f64 {
	let mut ft = win::FileTime { low: 0, high: 0 };
	// SAFETY: the call writes the FILETIME it is given
	unsafe { win::GetSystemTimePreciseAsFileTime(&mut ft) };
	let t = ((ft.high as u64) << 32) | ft.low as u64;
	t.wrapping_sub(116444736000000000) as f64 * 1e-7
}

/// set_nonblocking: the socket made non-blocking; on Windows also SIO_UDP_CONNRESET turned off, without which a
/// datagram sent to a subscriber that has gone away makes the next recvfrom fail with WSAECONNRESET, every time, for
/// every other client too (its result is not checked, as in the C)
pub fn set_nonblocking(s: &UdpSocket) -> bool {
	if s.set_nonblocking(true).is_err() {
		return false;
	}
	#[cfg(windows)]
	{
		use std::os::windows::io::AsRawSocket;
		let off: i32 = 0;
		let mut bytes: u32 = 0;
		// SAFETY: the socket is open; the input is the BOOL the call reads, there is no output buffer
		unsafe {
			win::WSAIoctl(
				s.as_raw_socket() as usize,
				win::SIO_UDP_CONNRESET,
				(&off as *const i32).cast(),
				std::mem::size_of::<i32>() as u32,
				std::ptr::null_mut(),
				0,
				&mut bytes,
				std::ptr::null_mut(),
				std::ptr::null_mut(),
			)
		};
	}
	true
}
