//! The sockets the C++ makes, with the calls it makes (BSD sockets on Linux, Winsock on Windows): IPv4 UDP, the
//! options and flags it sets, its sends and receives with their return values. A socket is an i64 here: a file
//! descriptor on Linux, a SOCKET on Windows, -1 (INVALID_SOCKET) for none.
//!
//! The C++'s sockets are made without close-on-exec, so on Linux a runtime the driver starts inherits them as the
//! C++'s does (on Windows the runtime inherits only its log's handle, the handle list's).

use std::ffi::{c_char, c_int, c_void};

pub type Sock = i64;
pub const SOCK_BAD: Sock = -1;

/// struct sockaddr_in (the same 16 bytes on both): the port and the address in network byte order
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SockAddrIn {
	pub sin_family: u16,
	pub sin_port: u16,
	pub sin_addr: u32,
	pub sin_zero: [u8; 8],
}
const _: () = assert!(std::mem::size_of::<SockAddrIn>() == 16);

pub const AF_INET: u16 = 2;
const SOCK_DGRAM: c_int = 2;
const INADDR_LOOPBACK: u32 = 0x7f00_0001;

pub fn htons(x: u16) -> u16 {
	x.to_be()
}

pub fn htonl(x: u32) -> u32 {
	x.to_be()
}

impl SockAddrIn {
	/// sin_family AF_INET, sin_port htons(port), sin_addr htonl(INADDR_LOOPBACK)
	pub fn loopback(port: u16) -> SockAddrIn {
		SockAddrIn { sin_family: AF_INET, sin_port: htons(port), sin_addr: htonl(INADDR_LOOPBACK), sin_zero: [0; 8] }
	}
}

pub use sys::*;

#[cfg(unix)]
mod sys {
	use super::*;

	const SOL_SOCKET: c_int = 1;
	const SO_RCVBUF: c_int = 8;
	const SO_RCVTIMEO: c_int = 20;
	const F_GETFL: c_int = 3;
	const F_SETFL: c_int = 4;
	const O_NONBLOCK: c_int = 0o4000;

	#[repr(C)]
	struct Timeval {
		tv_sec: i64,
		tv_usec: i64,
	}

	mod c {
		use super::*;
		extern "C" {
			pub fn socket(domain: c_int, ty: c_int, protocol: c_int) -> c_int;
			pub fn bind(fd: c_int, addr: *const SockAddrIn, len: u32) -> c_int;
			pub fn setsockopt(fd: c_int, level: c_int, name: c_int, val: *const c_void, len: u32) -> c_int;
			pub fn fcntl(fd: c_int, cmd: c_int, ...) -> c_int;
			pub fn recv(fd: c_int, buf: *mut c_void, len: usize, flags: c_int) -> isize;
			pub fn recvfrom(
				fd: c_int, buf: *mut c_void, len: usize, flags: c_int, addr: *mut SockAddrIn, alen: *mut u32,
			) -> isize;
			pub fn sendto(
				fd: c_int, buf: *const c_void, len: usize, flags: c_int, addr: *const SockAddrIn, alen: u32,
			) -> isize;
			pub fn close(fd: c_int) -> c_int;
			pub fn inet_pton(af: c_int, src: *const c_char, dst: *mut c_void) -> c_int;
		}
	}

	/// The socket's length type (socklen_t)
	pub type SockLen = u32;

	/// socket(AF_INET, SOCK_DGRAM, 0)
	pub fn socket_udp() -> Sock {
		// SAFETY: a plain system call with constant arguments
		unsafe { c::socket(AF_INET as c_int, SOCK_DGRAM, 0) as Sock }
	}

	pub fn bind(s: Sock, a: &SockAddrIn) -> i32 {
		// SAFETY: the address is a sockaddr_in of the size passed
		unsafe { c::bind(s as c_int, a, 16) }
	}

	/// setsockopt(s, SOL_SOCKET, SO_RCVBUF, &bytes, sizeof bytes)
	pub fn set_rcvbuf(s: Sock, bytes: i32) {
		// SAFETY: the option's value is an int of the size passed
		unsafe { c::setsockopt(s as c_int, SOL_SOCKET, SO_RCVBUF, (&bytes as *const i32).cast(), 4) };
	}

	/// The provider's receive timeout: `timeval tv = {0, 50000}; setsockopt(SO_RCVTIMEO)`
	pub fn set_recv_timeout_50ms(s: Sock) {
		let tv = Timeval { tv_sec: 0, tv_usec: 50000 };
		// SAFETY: the option's value is a timeval of the size passed
		unsafe { c::setsockopt(s as c_int, SOL_SOCKET, SO_RCVTIMEO, (&tv as *const Timeval).cast(), 16) };
	}

	/// The command socket made non-blocking: `fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK)`
	pub fn set_nonblocking(s: Sock) {
		// SAFETY: fcntl on the socket's descriptor with an int argument
		unsafe {
			let fl = c::fcntl(s as c_int, F_GETFL, 0 as c_int);
			c::fcntl(s as c_int, F_SETFL, fl | O_NONBLOCK);
		}
	}

	/// recv(s, buf, len, 0), its result as the C++'s int
	pub fn recv(s: Sock, buf: &mut [u8]) -> i32 {
		// SAFETY: recv writes at most buf.len() bytes into buf
		unsafe { c::recv(s as c_int, buf.as_mut_ptr().cast(), buf.len(), 0) as i32 }
	}

	/// recvfrom(s, buf, len, 0, &from, &fl): the result, the sender and its address's length
	pub fn recvfrom(s: Sock, buf: &mut [u8]) -> (i32, SockAddrIn, SockLen) {
		let mut from = SockAddrIn::default();
		let mut fl: SockLen = 16;
		// SAFETY: recvfrom writes at most buf.len() bytes into buf and at most fl bytes into from
		let n = unsafe { c::recvfrom(s as c_int, buf.as_mut_ptr().cast(), buf.len(), 0, &mut from, &mut fl) };
		(n as i32, from, fl)
	}

	pub fn sendto(s: Sock, data: &[u8], to: &SockAddrIn, tolen: SockLen) -> i32 {
		// SAFETY: sendto reads data.len() bytes of data and tolen bytes of the address
		unsafe { c::sendto(s as c_int, data.as_ptr().cast(), data.len(), 0, to, tolen) as i32 }
	}

	/// sock_close (close)
	pub fn close(s: Sock) {
		// SAFETY: closing a descriptor the driver made
		unsafe { c::close(s as c_int) };
	}

	/// inet_pton(AF_INET, host.c_str(), &addr): 1 and the address where the host is a dotted quad, else 0 (and the
	/// address untouched)
	pub fn inet_pton(host: &[u8], addr: &mut u32) -> i32 {
		let mut h = crate::helpers::c_str(host).to_vec();
		h.push(0);
		// SAFETY: a NUL-terminated string in, at most 4 bytes out into the u32
		unsafe { c::inet_pton(AF_INET as c_int, h.as_ptr().cast(), (addr as *mut u32).cast()) }
	}

	/// WSAStartup: none on Linux
	pub fn wsa_startup() {}
}

#[cfg(windows)]
mod sys {
	use super::*;

	const SOL_SOCKET: c_int = 0xffff;
	const SO_RCVBUF: c_int = 0x1002;
	const SO_RCVTIMEO: c_int = 0x1006;
	const FIONBIO: i32 = 0x8004_667eu32 as i32;
	/// _WSAIOW(IOC_VENDOR, 12)
	const SIO_UDP_CONNRESET: u32 = 0x9800_000c;

	mod c {
		use super::*;
		#[link(name = "ws2_32")]
		extern "system" {
			pub fn socket(af: c_int, ty: c_int, protocol: c_int) -> usize;
			pub fn bind(s: usize, name: *const SockAddrIn, namelen: c_int) -> c_int;
			pub fn setsockopt(s: usize, level: c_int, name: c_int, val: *const c_char, len: c_int) -> c_int;
			pub fn ioctlsocket(s: usize, cmd: i32, argp: *mut u32) -> c_int;
			pub fn WSAIoctl(
				s: usize, code: u32, inb: *mut c_void, cbin: u32, outb: *mut c_void, cbout: u32, bytes: *mut u32,
				ov: *mut c_void, cr: *mut c_void,
			) -> c_int;
			pub fn recv(s: usize, buf: *mut c_char, len: c_int, flags: c_int) -> c_int;
			pub fn recvfrom(
				s: usize, buf: *mut c_char, len: c_int, flags: c_int, from: *mut SockAddrIn, fromlen: *mut c_int,
			) -> c_int;
			pub fn sendto(
				s: usize, buf: *const c_char, len: c_int, flags: c_int, to: *const SockAddrIn, tolen: c_int,
			) -> c_int;
			pub fn closesocket(s: usize) -> c_int;
			pub fn inet_pton(family: c_int, src: *const c_char, dst: *mut c_void) -> c_int;
			pub fn WSAStartup(version: u16, data: *mut c_void) -> c_int;
		}
	}

	/// The socket's length type (int)
	pub type SockLen = c_int;

	/// socket(AF_INET, SOCK_DGRAM, 0)
	pub fn socket_udp() -> Sock {
		// SAFETY: a plain Winsock call with constant arguments
		unsafe { c::socket(AF_INET as c_int, SOCK_DGRAM, 0) as isize as Sock }
	}

	pub fn bind(s: Sock, a: &SockAddrIn) -> i32 {
		// SAFETY: the address is a sockaddr_in of the size passed
		unsafe { c::bind(s as usize, a, 16) }
	}

	pub fn set_rcvbuf(s: Sock, bytes: i32) {
		// SAFETY: the option's value is an int of the size passed
		unsafe { c::setsockopt(s as usize, SOL_SOCKET, SO_RCVBUF, (&bytes as *const i32).cast(), 4) };
	}

	/// SIO_UDP_CONNRESET off: a closed port's ICMP reply does not fail the next receive with WSAECONNRESET
	fn udp_connreset_off(s: Sock) {
		let mut off: i32 = 0;
		let mut bytes: u32 = 0;
		// SAFETY: the input is a BOOL of the size passed, no output, the byte count into a local
		unsafe {
			c::WSAIoctl(
				s as usize,
				SIO_UDP_CONNRESET,
				(&mut off as *mut i32).cast(),
				4,
				std::ptr::null_mut(),
				0,
				&mut bytes,
				std::ptr::null_mut(),
				std::ptr::null_mut(),
			)
		};
	}

	/// The provider's receive timeout: `DWORD tv = 50; setsockopt(SO_RCVTIMEO)`, then SIO_UDP_CONNRESET off
	pub fn set_recv_timeout_50ms(s: Sock) {
		let tv: u32 = 50;
		// SAFETY: the option's value is a DWORD of the size passed
		unsafe { c::setsockopt(s as usize, SOL_SOCKET, SO_RCVTIMEO, (&tv as *const u32).cast(), 4) };
		udp_connreset_off(s);
	}

	/// The command socket: `u_long nb = 1; ioctlsocket(FIONBIO)`, then SIO_UDP_CONNRESET off
	pub fn set_nonblocking(s: Sock) {
		let mut nb: u32 = 1;
		// SAFETY: FIONBIO takes a u_long
		unsafe { c::ioctlsocket(s as usize, FIONBIO, &mut nb) };
		udp_connreset_off(s);
	}

	pub fn recv(s: Sock, buf: &mut [u8]) -> i32 {
		// SAFETY: recv writes at most the length passed into buf
		unsafe { c::recv(s as usize, buf.as_mut_ptr().cast(), buf.len() as c_int, 0) }
	}

	pub fn recvfrom(s: Sock, buf: &mut [u8]) -> (i32, SockAddrIn, SockLen) {
		let mut from = SockAddrIn::default();
		let mut fl: SockLen = 16;
		// SAFETY: recvfrom writes at most the length passed into buf and at most fl bytes into from
		let n = unsafe { c::recvfrom(s as usize, buf.as_mut_ptr().cast(), buf.len() as c_int, 0, &mut from, &mut fl) };
		(n, from, fl)
	}

	pub fn sendto(s: Sock, data: &[u8], to: &SockAddrIn, tolen: SockLen) -> i32 {
		// SAFETY: sendto reads the length passed of data and tolen bytes of the address
		unsafe { c::sendto(s as usize, data.as_ptr().cast(), data.len() as c_int, 0, to, tolen) }
	}

	/// sock_close (closesocket)
	pub fn close(s: Sock) {
		// SAFETY: closing a socket the driver made
		unsafe { c::closesocket(s as usize) };
	}

	pub fn inet_pton(host: &[u8], addr: &mut u32) -> i32 {
		let mut h = crate::helpers::c_str(host).to_vec();
		h.push(0);
		// SAFETY: a NUL-terminated string in, at most 4 bytes out into the u32
		unsafe { c::inet_pton(AF_INET as c_int, h.as_ptr().cast(), (addr as *mut u32).cast()) }
	}

	/// WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa)
	pub fn wsa_startup() {
		// WSADATA is 408 bytes on Win64
		let mut wsa = [0u64; 64];
		// SAFETY: WSAStartup writes a WSADATA, which fits in the 512 bytes given
		unsafe { c::WSAStartup(0x0202, wsa.as_mut_ptr().cast()) };
	}
}

/// The size to pass for a sockaddr_in
pub const SOCKADDR_IN_LEN: SockLen = 16;
