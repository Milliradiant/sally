//! The runtime process the driver starts (RuntimeProcess, 0.7): started when SteamVR starts unless one already
//! answers, started again when it exits, sent "quit" and then killed when SteamVR exits.
//!
//! On Windows it is CreateProcessW with CREATE_NO_WINDOW, its stdout and stderr the log file, which is the only handle
//! it inherits (PROC_THREAD_ATTRIBUTE_HANDLE_LIST); on Linux posix_spawn, the log opened on fd 1 and duplicated to 2,
//! in the runtime's folder (the driver's process changes into it for the call and back).

use sally_math::printf::BArg;

use crate::helpers::{expand_time, is_absolute, log, now_seconds, time_stamp, K_SEP};
use crate::settings::g_opt;

/// The process the driver started (none at first)
#[derive(Debug, Default)]
pub struct RuntimeProcess {
	#[cfg(windows)]
	proc: sys::Handle,
	#[cfg(unix)]
	pid: i32,
}

impl RuntimeProcess {
	/// None started
	pub const NONE: RuntimeProcess = RuntimeProcess {
		#[cfg(windows)]
		proc: sys::Handle(std::ptr::null_mut()),
		#[cfg(unix)]
		pid: 0,
	};

	/// Start(dir): runtime_exe in runtime_cwd (each relative to the driver's folder) with the runtime's arguments,
	/// {time} = this start (a restart after a crash writes new files instead of emptying the ones that explain it: the
	/// recording is opened with "w" and the log with CREATE_ALWAYS), and --runtime-port added if they lack it
	pub fn start(&mut self, dir: &[u8]) -> bool {
		let o = g_opt();
		let joined = |rel: &[u8]| -> Vec<u8> {
			let mut p = dir.to_vec();
			p.push(K_SEP);
			p.extend_from_slice(rel);
			p
		};
		let exe = if is_absolute(&o.runtime_exe) { o.runtime_exe.clone() } else { joined(&o.runtime_exe) };
		let cwd = if o.runtime_cwd.is_empty() {
			dir.to_vec()
		} else if is_absolute(&o.runtime_cwd) {
			o.runtime_cwd.clone()
		} else {
			joined(&o.runtime_cwd)
		};
		let stamp = time_stamp();
		let expand = |s: &[u8]| expand_time(s.to_vec(), || stamp.clone());
		let rlog = expand(&o.runtime_log);
		let log_path = if rlog.is_empty() {
			Vec::new()
		} else if is_absolute(&rlog) {
			rlog.clone()
		} else {
			joined(&rlog)
		};
		let mut argv: Vec<Vec<u8>> = o.runtime_args.iter().map(|a| expand(a)).collect();
		let has_port = argv.iter().any(|a| a == b"--runtime-port");
		if !has_port {
			argv.push(b"--runtime-port".to_vec());
			argv.push(o.runtime_port.to_string().into_bytes());
		}
		log(
			"sally: starting the runtime %s in %s (%d arguments, console output %s)\n",
			&[
				BArg::S(&exe),
				BArg::S(&cwd),
				BArg::I(argv.len() as i32 as i64),
				BArg::S(if log_path.is_empty() { b"discarded" } else { &log_path }),
			],
		);
		sys::start(self, &exe, &cwd, &log_path, &argv)
	}

	/// Another program started the same way (the panel): `exe` in `cwd` with `argv`, its
	/// console output to `log_path` (none: discarded); each path as the caller resolved it
	pub fn start_program(&mut self, exe: &[u8], cwd: &[u8], log_path: &[u8], argv: &[Vec<u8>]) -> bool {
		sys::start(self, exe, cwd, log_path, argv)
	}

	/// Killed at once, if it is ours and runs
	pub fn kill(&mut self) {
		if self.started() {
			sys::kill(self);
		}
	}

	/// Started(): a process is ours
	pub fn started(&self) -> bool {
		sys::started(self)
	}

	/// Alive(&code): true while the process runs; false (once) when it has exited, with its exit code (false, and the
	/// code untouched, when there is none)
	pub fn alive(&mut self, code: &mut i32) -> bool {
		sys::alive(self, code)
	}

	/// WaitOrKill(seconds): up to `seconds` for the process to exit by itself, then killed
	pub fn wait_or_kill(&mut self, seconds: f64) {
		let end = now_seconds() + seconds;
		let mut code = 0;
		while now_seconds() < end {
			if !self.alive(&mut code) {
				log("sally: runtime exited (code %d)\n", &[BArg::I(code as i64)]);
				return;
			}
			sally_os::chrono::sleep_for_ms(50);
		}
		if !self.started() {
			return;
		}
		log("sally: runtime did not exit within %.0f s, terminating it\n", &[BArg::F(seconds)]);
		sys::kill(self);
	}
}

#[cfg(unix)]
mod sys {
	use std::ffi::{c_char, c_int, c_void};

	use sally_math::printf::BArg;

	use super::RuntimeProcess;
	use crate::helpers::{c_str, log};

	const O_WRONLY: c_int = 1;
	const O_CREAT: c_int = 0o100;
	const O_TRUNC: c_int = 0o1000;
	const WNOHANG: c_int = 1;
	const SIGKILL: c_int = 9;

	/// glibc's posix_spawn_file_actions_t
	#[repr(C)]
	struct FileActions {
		allocated: c_int,
		used: c_int,
		actions: *mut c_void,
		pad: [c_int; 16],
	}
	const _: () = assert!(std::mem::size_of::<FileActions>() == 80);

	extern "C" {
		static environ: *const *const c_char;
		fn posix_spawn_file_actions_init(fa: *mut FileActions) -> c_int;
		fn posix_spawn_file_actions_addopen(
			fa: *mut FileActions, fd: c_int, path: *const c_char, flags: c_int, mode: u32,
		) -> c_int;
		fn posix_spawn_file_actions_adddup2(fa: *mut FileActions, fd: c_int, newfd: c_int) -> c_int;
		fn posix_spawn_file_actions_destroy(fa: *mut FileActions) -> c_int;
		fn posix_spawn(
			pid: *mut i32, path: *const c_char, fa: *const FileActions, attr: *const c_void, argv: *const *mut c_char,
			envp: *const *const c_char,
		) -> c_int;
		fn getcwd(buf: *mut c_char, size: usize) -> *mut c_char;
		fn chdir(path: *const c_char) -> c_int;
		fn waitpid(pid: i32, status: *mut c_int, options: c_int) -> i32;
		#[link_name = "kill"]
		fn kill_pid(pid: i32, sig: c_int) -> c_int;
	}

	/// A std::string's c_str(): its bytes up to a NUL, and one
	fn cz(s: &[u8]) -> Vec<u8> {
		let mut v = c_str(s).to_vec();
		v.push(0);
		v
	}

	pub fn start(p: &mut RuntimeProcess, exe: &[u8], cwd: &[u8], log_path: &[u8], argv: &[Vec<u8>]) -> bool {
		let exe_z = cz(exe);
		let cwd_z = cz(cwd);
		let out_z = if log_path.is_empty() { cz(b"/dev/null") } else { cz(log_path) };
		let args_z: Vec<Vec<u8>> = argv.iter().map(|a| cz(a)).collect();
		let mut cargv: Vec<*mut c_char> = vec![exe_z.as_ptr() as *mut c_char];
		cargv.extend(args_z.iter().map(|a| a.as_ptr() as *mut c_char));
		cargv.push(std::ptr::null_mut());
		let mut old_cwd = vec![0u8; 4096];
		// SAFETY: the file actions are initialised before use and destroyed after; every string passed is
		// NUL-terminated and outlives the calls; getcwd writes at most 4096 bytes; posix_spawn reads argv (null-ended)
		// and the process's environment and writes the pid
		unsafe {
			let mut fa: FileActions = std::mem::zeroed();
			posix_spawn_file_actions_init(&mut fa);
			posix_spawn_file_actions_addopen(&mut fa, 1, out_z.as_ptr().cast(), O_WRONLY | O_CREAT | O_TRUNC, 0o644);
			posix_spawn_file_actions_adddup2(&mut fa, 1, 2);
			if getcwd(old_cwd.as_mut_ptr().cast(), old_cwd.len()).is_null() {
				old_cwd.clear();
			}
			if chdir(cwd_z.as_ptr().cast()) != 0 {
				log("sally: cannot enter %s\n", &[BArg::S(cwd)]);
			}
			let rc = posix_spawn(&mut p.pid, exe_z.as_ptr().cast(), &fa, std::ptr::null(), cargv.as_ptr(), environ);
			if !old_cwd.is_empty() {
				let back = cz(&old_cwd);
				if chdir(back.as_ptr().cast()) != 0 {
					log("sally: cannot return to %s\n", &[BArg::S(c_str(&old_cwd))]);
				}
			}
			posix_spawn_file_actions_destroy(&mut fa);
			if rc != 0 {
				log("sally: could not start %s (error %d)\n", &[BArg::S(exe), BArg::I(rc as i64)]);
				p.pid = 0;
				return false;
			}
		}
		true
	}

	pub fn started(p: &RuntimeProcess) -> bool {
		p.pid != 0
	}

	pub fn alive(p: &mut RuntimeProcess, code: &mut i32) -> bool {
		if p.pid == 0 {
			return false;
		}
		let mut st: c_int = 0;
		// SAFETY: waitpid on the child the driver started, the status into a local
		let r = unsafe { waitpid(p.pid, &mut st, WNOHANG) };
		if r == 0 {
			return true;
		}
		// WIFEXITED(st) ? WEXITSTATUS(st) : -WTERMSIG(st)
		*code = if st & 0x7f == 0 { (st >> 8) & 0xff } else { -(st & 0x7f) };
		p.pid = 0;
		false
	}

	/// kill(pid, SIGKILL), waitpid(pid, NULL, 0)
	pub fn kill(p: &mut RuntimeProcess) {
		// SAFETY: the signal goes to the child the driver started, which is then reaped
		unsafe {
			kill_pid(p.pid, SIGKILL);
			waitpid(p.pid, std::ptr::null_mut(), 0);
		}
		p.pid = 0;
	}
}

#[cfg(windows)]
mod sys {
	#![allow(non_snake_case, clippy::upper_case_acronyms)]

	use std::ffi::c_void;

	use sally_math::printf::BArg;

	use super::RuntimeProcess;
	use crate::helpers::{log, wide_c_str, widen};

	type HANDLE = *mut c_void;
	const INVALID_HANDLE_VALUE: HANDLE = -1isize as HANDLE;
	const GENERIC_WRITE: u32 = 0x4000_0000;
	const FILE_SHARE_READ: u32 = 0x1;
	const FILE_SHARE_WRITE: u32 = 0x2;
	const CREATE_ALWAYS: u32 = 2;
	const FILE_ATTRIBUTE_NORMAL: u32 = 0x80;
	const STARTF_USESTDHANDLES: u32 = 0x100;
	const PROC_THREAD_ATTRIBUTE_HANDLE_LIST: usize = 0x20002;
	const CREATE_NO_WINDOW: u32 = 0x0800_0000;
	const EXTENDED_STARTUPINFO_PRESENT: u32 = 0x0008_0000;
	const WAIT_OBJECT_0: u32 = 0;

	/// The process handle (none: null)
	#[derive(Debug)]
	pub struct Handle(pub HANDLE);
	impl Default for Handle {
		fn default() -> Handle {
			Handle(std::ptr::null_mut())
		}
	}
	// SAFETY: a process handle may be used from any thread
	unsafe impl Send for Handle {}

	#[repr(C)]
	struct SECURITY_ATTRIBUTES {
		nLength: u32,
		lpSecurityDescriptor: *mut c_void,
		bInheritHandle: i32,
	}

	#[repr(C)]
	struct STARTUPINFOW {
		cb: u32,
		lpReserved: *mut u16,
		lpDesktop: *mut u16,
		lpTitle: *mut u16,
		dwX: u32,
		dwY: u32,
		dwXSize: u32,
		dwYSize: u32,
		dwXCountChars: u32,
		dwYCountChars: u32,
		dwFillAttribute: u32,
		dwFlags: u32,
		wShowWindow: u16,
		cbReserved2: u16,
		lpReserved2: *mut u8,
		hStdInput: HANDLE,
		hStdOutput: HANDLE,
		hStdError: HANDLE,
	}

	#[repr(C)]
	struct STARTUPINFOEXW {
		StartupInfo: STARTUPINFOW,
		lpAttributeList: *mut c_void,
	}
	const _: () = assert!(std::mem::size_of::<STARTUPINFOW>() == 104);
	const _: () = assert!(std::mem::size_of::<STARTUPINFOEXW>() == 112);

	#[repr(C)]
	struct PROCESS_INFORMATION {
		hProcess: HANDLE,
		hThread: HANDLE,
		dwProcessId: u32,
		dwThreadId: u32,
	}

	#[link(name = "kernel32")]
	extern "system" {
		fn CreateFileW(
			name: *const u16, access: u32, share: u32, sa: *mut SECURITY_ATTRIBUTES, disposition: u32, flags: u32,
			template: HANDLE,
		) -> HANDLE;
		fn InitializeProcThreadAttributeList(list: *mut c_void, count: u32, flags: u32, size: *mut usize) -> i32;
		fn UpdateProcThreadAttribute(
			list: *mut c_void, flags: u32, attribute: usize, value: *mut c_void, size: usize, prev: *mut c_void,
			ret: *mut usize,
		) -> i32;
		fn DeleteProcThreadAttributeList(list: *mut c_void);
		fn CreateProcessW(
			app: *const u16, cmd: *mut u16, pa: *mut c_void, ta: *mut c_void, inherit: i32, flags: u32,
			env: *mut c_void, cwd: *const u16, si: *mut STARTUPINFOW, pi: *mut PROCESS_INFORMATION,
		) -> i32;
		fn CloseHandle(h: HANDLE) -> i32;
		fn GetLastError() -> u32;
		fn WaitForSingleObject(h: HANDLE, ms: u32) -> u32;
		fn GetExitCodeProcess(h: HANDLE, code: *mut u32) -> i32;
		fn TerminateProcess(h: HANDLE, code: u32) -> i32;
	}

	/// Quote(a): one argument as CommandLineToArgvW reads it back
	fn quote(a: &[u16]) -> Vec<u16> {
		let special = |c: u16| c == b' ' as u16 || c == b'\t' as u16 || c == b'"' as u16;
		if !a.is_empty() && !a.iter().any(|&c| special(c)) {
			return a.to_vec();
		}
		let bslash = b'\\' as u16;
		let mut o = vec![b'"' as u16];
		let mut bs = 0usize;
		for &c in a {
			if c == bslash {
				bs += 1;
				continue;
			}
			if c == b'"' as u16 {
				o.extend(std::iter::repeat_n(bslash, bs * 2 + 1));
				o.push(c);
			} else {
				o.extend(std::iter::repeat_n(bslash, bs));
				o.push(c);
			}
			bs = 0;
		}
		o.extend(std::iter::repeat_n(bslash, bs * 2));
		o.push(b'"' as u16);
		o
	}

	pub fn start(p: &mut RuntimeProcess, exe: &[u8], cwd: &[u8], log_path: &[u8], argv: &[Vec<u8>]) -> bool {
		let mut cmd = quote(&widen(exe));
		for a in argv {
			cmd.push(b' ' as u16);
			cmd.extend(quote(&widen(a)));
		}
		let out_name = wide_c_str(&widen(if log_path.is_empty() { b"NUL" } else { log_path }));
		let mut sa = SECURITY_ATTRIBUTES {
			nLength: std::mem::size_of::<SECURITY_ATTRIBUTES>() as u32,
			lpSecurityDescriptor: std::ptr::null_mut(),
			bInheritHandle: 1,
		};
		let exe_w = wide_c_str(&widen(exe));
		let cwd_w = wide_c_str(&widen(cwd));
		let mut buf: Vec<u16> = cmd.clone();
		buf.push(0);
		// SAFETY: every string passed is NUL-terminated and outlives the calls; the attribute list lives in attr_buf,
		// sized by the first InitializeProcThreadAttributeList, and is deleted after CreateProcessW; the handle list
		// points at `out`, which outlives it; CreateProcessW may write into the command line buffer, which is ours
		unsafe {
			let mut out = CreateFileW(
				out_name.as_ptr(),
				GENERIC_WRITE,
				FILE_SHARE_READ | FILE_SHARE_WRITE,
				&mut sa,
				CREATE_ALWAYS,
				FILE_ATTRIBUTE_NORMAL,
				std::ptr::null_mut(),
			);
			// The runtime inherits the log handle and nothing else of vrserver's (handle list, not blanket inheritance).
			let mut si: STARTUPINFOEXW = std::mem::zeroed();
			si.StartupInfo.cb = std::mem::size_of::<STARTUPINFOEXW>() as u32;
			let mut attr_buf: Vec<u64> = Vec::new();
			let mut use_list = false;
			if out != INVALID_HANDLE_VALUE {
				si.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
				si.StartupInfo.hStdOutput = out;
				si.StartupInfo.hStdError = out;
				si.StartupInfo.hStdInput = std::ptr::null_mut();
				let mut size: usize = 0;
				InitializeProcThreadAttributeList(std::ptr::null_mut(), 1, 0, &mut size);
				attr_buf.resize(size.div_ceil(8), 0);
				si.lpAttributeList = attr_buf.as_mut_ptr().cast();
				use_list = InitializeProcThreadAttributeList(si.lpAttributeList, 1, 0, &mut size) != 0
					&& UpdateProcThreadAttribute(
						si.lpAttributeList,
						0,
						PROC_THREAD_ATTRIBUTE_HANDLE_LIST,
						(&mut out as *mut HANDLE).cast(),
						std::mem::size_of::<HANDLE>(),
						std::ptr::null_mut(),
						std::ptr::null_mut(),
					) != 0;
				if !use_list {
					si.lpAttributeList = std::ptr::null_mut();
				}
			}
			let mut pi: PROCESS_INFORMATION = std::mem::zeroed();
			let ok = CreateProcessW(
				exe_w.as_ptr(),
				buf.as_mut_ptr(),
				std::ptr::null_mut(),
				std::ptr::null_mut(),
				use_list as i32,
				CREATE_NO_WINDOW | if use_list { EXTENDED_STARTUPINFO_PRESENT } else { 0 },
				std::ptr::null_mut(),
				cwd_w.as_ptr(),
				&mut si.StartupInfo,
				&mut pi,
			);
			if !si.lpAttributeList.is_null() {
				DeleteProcThreadAttributeList(si.lpAttributeList);
			}
			if out != INVALID_HANDLE_VALUE {
				CloseHandle(out);
			}
			if ok == 0 {
				log("sally: could not start %s (error %lu)\n", &[BArg::S(exe), BArg::U(GetLastError() as u64)]);
				return false;
			}
			CloseHandle(pi.hThread);
			p.proc = Handle(pi.hProcess);
		}
		true
	}

	pub fn started(p: &RuntimeProcess) -> bool {
		!p.proc.0.is_null()
	}

	pub fn alive(p: &mut RuntimeProcess, code: &mut i32) -> bool {
		if p.proc.0.is_null() {
			return false;
		}
		// SAFETY: the process handle is ours until closed here
		unsafe {
			if WaitForSingleObject(p.proc.0, 0) != WAIT_OBJECT_0 {
				return true;
			}
			let mut c: u32 = 0;
			GetExitCodeProcess(p.proc.0, &mut c);
			*code = c as i32;
			CloseHandle(p.proc.0);
		}
		p.proc = Handle::default();
		false
	}

	/// TerminateProcess(proc, 1), WaitForSingleObject(proc, 2000), CloseHandle
	pub fn kill(p: &mut RuntimeProcess) {
		// SAFETY: the process handle is ours until closed here
		unsafe {
			TerminateProcess(p.proc.0, 1);
			WaitForSingleObject(p.proc.0, 2000);
			CloseHandle(p.proc.0);
		}
		p.proc = Handle::default();
	}
}
