//! `sally-setup`: sally's installer and uninstaller, one program for a user who runs nothing else (docs/release.md).
//!
//! Run as it is downloaded (sally's files packed after it, `sally_setup::payload`), it installs: after one question
//! it puts sally's files in the user's own folder (%LOCALAPPDATA%\sally: no administrator's rights), has
//! `sally setup` there put the driver on SteamVR's list and write its settings, adds sally to Windows' list of
//! installed apps and to the Start menu, says so, and opens the panel. SteamVR may be running: a file SteamVR holds is set
//! aside under another name and the new one put in its place, and sally starts with SteamVR's next start. Run again,
//! it installs over what is there; what the user set and what sally learned of the room are not among its files and
//! stay.
//!
//! It leaves itself there without its files as uninstall.exe, which Windows' list of apps runs with `--uninstall`:
//! after one question `sally uninstall` takes sally out of SteamVR, then the entry in Windows' list, the Start menu's
//! and the folder go. Started by hand without `--uninstall`, that copy asks the same question.
//!
//!   sally-setup [--to <folder>] [--quiet]      install (`--quiet`: no questions and no windows, for a script)
//!   uninstall --uninstall [--quiet]

#![cfg_attr(windows, windows_subsystem = "windows")]

use std::path::{Path, PathBuf};
use std::process::Command;

use sally_setup::payload;

const VERSION: &str = env!("CARGO_PKG_VERSION");
/// This program without its files, in sally's folder
const UNINSTALLER: &str = "uninstall.exe";

fn main() {
	let mut uninstall = false;
	let mut quiet = false;
	let mut to = None;
	let mut args = std::env::args_os().skip(1);
	while let Some(a) = args.next() {
		match a.to_string_lossy().as_ref() {
			"--uninstall" => uninstall = true,
			"--quiet" => quiet = true,
			"--to" => to = args.next().map(PathBuf::from),
			_ => {}
		}
	}
	let ui = Ui { quiet };
	let done = match Installer::open() {
		Ok(i) if uninstall || i.files.is_none() => i.uninstall(&ui),
		Ok(i) => i.install(&ui, to),
		Err(e) => Err(e),
	};
	if let Err(e) = done {
		ui.fail(&e);
		std::process::exit(1);
	}
}

/// What the user is asked and told: message boxes, or with `--quiet` nothing asked (every answer yes, bar a retry)
/// and the rest on the console
struct Ui {
	quiet: bool,
}

impl Ui {
	fn ask(&self, text: &str) -> bool {
		self.quiet || sys::message(text, sys::Kind::Ask)
	}

	fn retry(&self, text: &str) -> bool {
		!self.quiet && sys::message(text, sys::Kind::Retry)
	}

	fn tell(&self, text: &str) {
		if self.quiet {
			println!("{text}");
		} else {
			sys::message(text, sys::Kind::Tell);
		}
	}

	fn fail(&self, text: &str) {
		if self.quiet {
			eprintln!("{text}");
		} else {
			sys::message(text, sys::Kind::Fail);
		}
	}
}

/// This program as it was started: where it is, its bytes, and where sally's files in them are (none in the
/// uninstaller)
struct Installer {
	exe: PathBuf,
	bytes: Vec<u8>,
	/// the program's own length and the payload's place after it
	files: Option<(usize, usize)>,
}

/// What `sally <command>` said
struct Said {
	ok: bool,
	out: String,
	err: String,
}

impl Installer {
	fn open() -> Result<Installer, String> {
		let exe = std::env::current_exe().map_err(|e| format!("cannot find this program: {e}"))?;
		let bytes = std::fs::read(&exe).map_err(|e| format!("cannot read {}: {e}", exe.display()))?;
		let files = payload::find(&bytes).map(|(program, payload)| (program, payload.len()));
		Ok(Installer { exe, bytes, files })
	}

	fn install(&self, ui: &Ui, to: Option<PathBuf>) -> Result<(), String> {
		let Some((program, len)) = self.files else { return Err("there is nothing to install in this program".into()) };
		let root = match to {
			Some(t) => t,
			None => default_root().ok_or("cannot find your own program folder (%LOCALAPPDATA%)")?,
		};
		let question = if root.join("sally.exe").is_file() {
			format!(
				"sally is installed already. Install version {VERSION} over it?\n\nYour settings and what sally has \
				 learned about your room stay."
			)
		} else {
			format!(
				"Install sally {VERSION}?\n\nsally is a tracking add-on for SteamVR. It goes into\n{}\nNothing is \
				 changed on your trackers, controllers or base stations.",
				root.display()
			)
		};
		if !ui.ask(&question) {
			return Ok(());
		}
		if !steamvr_known() {
			return Err(
				"SteamVR was not found on this PC.\n\nInstall SteamVR, start it once, then run this again.".into()
			);
		}
		let files = payload::unpack(&self.bytes[program..program + len])?;
		std::fs::create_dir_all(&root).map_err(|e| format!("cannot make {}: {e}", root.display()))?;
		sweep(&root, 0);
		for f in &files {
			put(&f.path.split('/').fold(root.clone(), |p, part| p.join(part)), &f.bytes)?;
		}
		put(&root.join(UNINSTALLER), &self.bytes[..program])?;
		// (the panel is started here, not by the setup: a child of the setup would hold the setup's output open)
		let said = sally(&root, &["setup", "--stop-panel", "--no-panel"])?;
		if !said.ok {
			return Err(format!(
				"sally is in its folder, but could not be set up with SteamVR:\n\n{}",
				said.err.trim()
			));
		}
		let size_kb = files.iter().map(|f| f.bytes.len() as u64).sum::<u64>() / 1024;
		let mut notes: Vec<String> =
			said.out.lines().filter_map(|l| l.strip_prefix("note: ")).map(str::to_string).collect();
		if let Err(e) = sys::register(&root, size_kb) {
			notes.push(format!("sally could not be added to Windows' list of apps or the Start menu ({e})."));
		}
		let notes: String = notes.iter().map(|n| format!("\n\n{n}")).collect();
		ui.tell(&format!(
			"sally {VERSION} is installed.\n\nFrom now on sally starts with SteamVR. Its panel, which opens next, says \
			 what sally is doing. The first session in a room takes a few minutes to learn the room.{notes}"
		));
		// (after the message: the panel's window would cover it)
		if !ui.quiet {
			start_panel(&root);
		}
		Ok(())
	}

	fn uninstall(&self, ui: &Ui) -> Result<(), String> {
		let root = self.exe.parent().ok_or("cannot find this program's folder")?.to_path_buf();
		if !root.join("sally.exe").is_file() || !root.join("driver").is_dir() {
			return Err("sally is not installed in this program's folder: nothing was removed.".into());
		}
		if !ui.ask(
			"Remove sally from this PC?\n\nIts settings, what it has learned about your room and its recordings are \
			 removed with it.",
		) {
			return Ok(());
		}
		loop {
			let said = sally(&root, &["uninstall"])?;
			if said.ok {
				break;
			}
			let why = if said.err.contains("SteamVR is running") {
				"SteamVR is running. Close SteamVR completely, then try again.".to_string()
			} else {
				said.err.trim().to_string()
			};
			if !ui.retry(&format!("{why}\n\nsally is still installed.")) {
				return if ui.quiet { Err(why) } else { Ok(()) };
			}
		}
		sys::unregister();
		// all of the folder but this program, which Windows holds while it runs: that, and the folder, go after it
		remove_but(&root, &self.exe);
		ui.tell("sally is removed.");
		remove_later(&root);
		Ok(())
	}
}

/// %LOCALAPPDATA%\sally
fn default_root() -> Option<PathBuf> {
	std::env::var_os("LOCALAPPDATA").filter(|v| !v.is_empty()).map(|l| Path::new(&l).join("sally"))
}

/// Whether SteamVR has run for this user: its path registry is there
fn steamvr_known() -> bool {
	std::env::var_os("LOCALAPPDATA").is_some_and(|l| Path::new(&l).join("openvr").join("openvrpaths.vrpath").is_file())
}

/// A path with something after its file name
fn after(path: &Path, suffix: &str) -> PathBuf {
	let mut name = path.file_name().unwrap_or_default().to_os_string();
	name.push(suffix);
	path.with_file_name(name)
}

/// A file put in its place whole: written beside it, then renamed over it. One that is in use (SteamVR holds the
/// driver, the runtime runs) cannot be written over but can be renamed: it is set aside (`<name>.old-<n>`) and the new
/// one takes its name, for whoever opens it next. What was set aside goes with the next install or uninstall
fn put(path: &Path, bytes: &[u8]) -> Result<(), String> {
	if let Some(dir) = path.parent() {
		std::fs::create_dir_all(dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
	}
	let new = after(path, ".new");
	std::fs::write(&new, bytes).map_err(|e| format!("cannot write {}: {e}", new.display()))?;
	if std::fs::rename(&new, path).is_ok() {
		return Ok(());
	}
	let aside = (0..10_000).map(|n| after(path, &format!(".old-{n}"))).find(|p| !p.exists());
	let moved = aside.is_some_and(|aside| std::fs::rename(path, aside).is_ok());
	match std::fs::rename(&new, path) {
		Ok(()) => Ok(()),
		Err(e) => {
			let _ = std::fs::remove_file(&new);
			let in_use =
				if moved { "" } else { " (it is in use: close SteamVR and sally's panel, then run this again)" };
			Err(format!("cannot replace {}: {e}{in_use}", path.display()))
		}
	}
}

/// What earlier installs set aside or left half written, deleted where it can be (not under the user's own data and
/// logs, where none is)
fn sweep(dir: &Path, depth: usize) {
	let Ok(entries) = std::fs::read_dir(dir) else { return };
	for e in entries.flatten() {
		let path = e.path();
		let name = e.file_name().to_string_lossy().into_owned();
		if path.is_dir() {
			if depth < 6 && !(depth == 0 && (name == "data" || name == "logs")) {
				sweep(&path, depth + 1);
			}
		} else if name.contains(".old-") || name.ends_with(".new") {
			let _ = std::fs::remove_file(&path);
		}
	}
}

/// `sally <command> --home <root> ...` run from sally's folder, without a window, and what it said
fn sally(root: &Path, args: &[&str]) -> Result<Said, String> {
	let exe = root.join(if cfg!(windows) { "sally.exe" } else { "sally" });
	let mut c = Command::new(&exe);
	c.arg(args[0]).arg("--home").arg(root).args(&args[1..]).current_dir(root);
	sys::no_window(&mut c);
	let o = c.output().map_err(|e| format!("cannot run {}: {e}", exe.display()))?;
	Ok(Said {
		ok: o.status.success(),
		out: String::from_utf8_lossy(&o.stdout).into_owned(),
		err: String::from_utf8_lossy(&o.stderr).into_owned(),
	})
}

/// The panel started on its own, with nothing of this program's handed to it
fn start_panel(root: &Path) {
	use std::process::Stdio;
	let exe = root.join(if cfg!(windows) { "sally-panel.exe" } else { "sally-panel" });
	let mut c = Command::new(exe);
	c.current_dir(root).stdin(Stdio::null()).stdout(Stdio::null()).stderr(Stdio::null());
	let _ = c.spawn();
}

/// Everything in the folder deleted but one file
fn remove_but(root: &Path, keep: &Path) {
	let Ok(entries) = std::fs::read_dir(root) else { return };
	for e in entries.flatten() {
		let path = e.path();
		if path.is_dir() {
			let _ = std::fs::remove_dir_all(&path);
		} else if path != keep {
			let _ = std::fs::remove_file(&path);
		}
	}
}

/// The folder, with this program in it, deleted once this program has ended: Windows' command interpreter, without
/// a window, waits two seconds (a ping to itself is its way to wait) and removes it. `/s`: the command is what is
/// between the first quote and the last, the folder's own quotes kept
fn remove_later(root: &Path) {
	#[cfg(windows)]
	{
		use std::os::windows::process::CommandExt;
		let dir = root.display();
		let mut c = Command::new("cmd.exe");
		c.raw_arg(format!("/d /s /c \"ping -n 3 127.0.0.1 >nul & rd /s /q \"{dir}\"\""));
		c.current_dir(std::env::temp_dir());
		sys::no_window(&mut c);
		let _ = c.spawn();
	}
	#[cfg(not(windows))]
	{
		let _ = std::fs::remove_dir_all(root);
	}
}

/// What the installer needs of Windows, declared by hand as the rest of the workspace does: a message box, Windows'
/// list of installed apps (the registry), the Start menu's shortcut (the shell's link object), a program started
/// without a console window. Elsewhere: the console, and nothing to register
#[cfg(windows)]
mod sys {
	use std::ffi::c_void;
	use std::path::Path;
	use std::process::Command;

	use super::{UNINSTALLER, VERSION};

	pub enum Kind {
		Ask,
		Retry,
		Tell,
		Fail,
	}

	const MB_OKCANCEL: u32 = 0x1;
	const MB_RETRYCANCEL: u32 = 0x5;
	const MB_ICONERROR: u32 = 0x10;
	const MB_ICONQUESTION: u32 = 0x20;
	const MB_ICONINFORMATION: u32 = 0x40;
	const MB_SETFOREGROUND: u32 = 0x1_0000;
	const IDOK: i32 = 1;
	const IDRETRY: i32 = 4;

	/// HKEY_CURRENT_USER
	const CURRENT_USER: isize = 0x8000_0001u32 as i32 as isize;
	const KEY_WRITE: u32 = 0x2_0006;
	const REG_SZ: u32 = 1;
	const REG_DWORD: u32 = 4;
	const APP_KEY: &str = "Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\sally";

	const COINIT_APARTMENTTHREADED: u32 = 0x2;
	const CLSCTX_INPROC_SERVER: u32 = 0x1;

	#[repr(C)]
	struct Guid(u32, u16, u16, [u8; 8]);
	const CLSID_SHELL_LINK: Guid = Guid(0x0002_1401, 0, 0, [0xc0, 0, 0, 0, 0, 0, 0, 0x46]);
	const IID_SHELL_LINK_W: Guid = Guid(0x0002_14f9, 0, 0, [0xc0, 0, 0, 0, 0, 0, 0, 0x46]);
	const IID_PERSIST_FILE: Guid = Guid(0x0000_010b, 0, 0, [0xc0, 0, 0, 0, 0, 0, 0, 0x46]);

	#[link(name = "user32")]
	extern "system" {
		fn MessageBoxW(owner: isize, text: *const u16, caption: *const u16, kind: u32) -> i32;
	}

	#[link(name = "advapi32")]
	extern "system" {
		fn RegCreateKeyExW(
			key: isize, sub: *const u16, reserved: u32, class: *const u16, options: u32, access: u32,
			security: *const c_void, out: *mut isize, disposition: *mut u32,
		) -> i32;
		fn RegSetValueExW(key: isize, name: *const u16, reserved: u32, kind: u32, data: *const u8, bytes: u32) -> i32;
		fn RegCloseKey(key: isize) -> i32;
		fn RegDeleteTreeW(key: isize, sub: *const u16) -> i32;
	}

	#[link(name = "ole32")]
	extern "system" {
		fn CoInitializeEx(reserved: *const c_void, model: u32) -> i32;
		fn CoUninitialize();
		fn CoCreateInstance(
			class: *const Guid, outer: *const c_void, context: u32, interface: *const Guid, out: *mut *mut c_void,
		) -> i32;
	}

	fn wide(s: &str) -> Vec<u16> {
		s.encode_utf16().chain(Some(0)).collect()
	}

	/// A message box; for a question, whether the answer was yes (OK, Retry)
	pub fn message(text: &str, kind: Kind) -> bool {
		let flags = MB_SETFOREGROUND
			| match kind {
				Kind::Ask => MB_OKCANCEL | MB_ICONQUESTION,
				Kind::Retry => MB_RETRYCANCEL | MB_ICONERROR,
				Kind::Tell => MB_ICONINFORMATION,
				Kind::Fail => MB_ICONERROR,
			};
		let (text, caption) = (wide(text), wide("sally"));
		// SAFETY: two NUL-terminated UTF-16 strings that outlive the call, and no owner window
		let answer = unsafe { MessageBoxW(0, text.as_ptr(), caption.as_ptr(), flags) };
		answer == IDOK || answer == IDRETRY
	}

	pub fn no_window(c: &mut Command) {
		use std::os::windows::process::CommandExt;
		// CREATE_NO_WINDOW
		c.creation_flags(0x0800_0000);
	}

	/// The Start menu's programs folder, where %APPDATA% is set
	fn start_menu() -> Option<std::path::PathBuf> {
		let roaming = std::env::var_os("APPDATA").filter(|v| !v.is_empty())?;
		Some(Path::new(&roaming).join("Microsoft").join("Windows").join("Start Menu").join("Programs"))
	}

	/// sally in Windows' list of installed apps (this user's), with how to uninstall it, and in the Start menu
	pub fn register(root: &Path, size_kb: u64) -> Result<(), String> {
		let at = |name: &str| root.join(name).to_string_lossy().into_owned();
		let uninstall = format!("\"{}\" --uninstall", at(UNINSTALLER));
		let texts = [
			("DisplayName", "sally".to_string()),
			("DisplayVersion", VERSION.to_string()),
			("Publisher", "sally".to_string()),
			("InstallLocation", root.to_string_lossy().into_owned()),
			("DisplayIcon", at("sally.ico")),
			("UninstallString", uninstall.clone()),
			("QuietUninstallString", format!("{uninstall} --quiet")),
		];
		let numbers = [("NoModify", 1u32), ("NoRepair", 1), ("EstimatedSize", size_kb.min(u32::MAX as u64) as u32)];
		let mut key = 0isize;
		let sub = wide(APP_KEY);
		// SAFETY: NUL-terminated strings and a place for the key, all outliving the calls; each value's bytes and
		// their count are its own; the key is closed
		unsafe {
			let rc = RegCreateKeyExW(
				CURRENT_USER,
				sub.as_ptr(),
				0,
				std::ptr::null(),
				0,
				KEY_WRITE,
				std::ptr::null(),
				&mut key,
				std::ptr::null_mut(),
			);
			if rc != 0 {
				return Err(format!("registry error {rc}"));
			}
			for (name, value) in &texts {
				let (name, value) = (wide(name), wide(value));
				RegSetValueExW(key, name.as_ptr(), 0, REG_SZ, value.as_ptr().cast(), (value.len() * 2) as u32);
			}
			for (name, value) in &numbers {
				let name = wide(name);
				RegSetValueExW(key, name.as_ptr(), 0, REG_DWORD, (&raw const *value).cast(), 4);
			}
			RegCloseKey(key);
		}
		let menu = start_menu().ok_or("no %APPDATA%")?;
		shortcut(&menu.join("sally.lnk"), &root.join("sally-panel.exe"), &root.join("sally.ico"), root)
	}

	pub fn unregister() {
		let sub = wide(APP_KEY);
		// SAFETY: a NUL-terminated string that outlives the call
		unsafe {
			RegDeleteTreeW(CURRENT_USER, sub.as_ptr());
		}
		if let Some(menu) = start_menu() {
			let _ = std::fs::remove_file(menu.join("sally.lnk"));
		}
	}

	/// A method of a COM object, by its place in the object's table
	unsafe fn method(object: *mut c_void, index: usize) -> *const c_void {
		// SAFETY (the caller's): `object` is a live COM object, whose first word points at its table of methods
		unsafe { *(*(object as *const *const *const c_void)).add(index) }
	}

	/// A shortcut file (.lnk) to a program, with its icon and the folder it starts in: the shell's link object
	/// (IShellLinkW) filled in, then saved through its IPersistFile
	fn shortcut(link: &Path, target: &Path, icon: &Path, folder: &Path) -> Result<(), String> {
		type SetText = unsafe extern "system" fn(*mut c_void, *const u16) -> i32;
		type SetIcon = unsafe extern "system" fn(*mut c_void, *const u16, i32) -> i32;
		type Query = unsafe extern "system" fn(*mut c_void, *const Guid, *mut *mut c_void) -> i32;
		type Save = unsafe extern "system" fn(*mut c_void, *const u16, i32) -> i32;
		type Release = unsafe extern "system" fn(*mut c_void) -> u32;
		// IShellLinkW's table: SetDescription 7, SetWorkingDirectory 9, SetIconLocation 17, SetPath 20;
		// IPersistFile's: Save 6; IUnknown's: QueryInterface 0, Release 2
		let path = |p: &Path| wide(&p.to_string_lossy());
		if let Some(dir) = link.parent() {
			std::fs::create_dir_all(dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
		}
		// SAFETY: COM is initialised for this thread and closed again; every object is used only where its making
		// succeeded and released once; each method is called through its own place in the table with the arguments
		// its interface declares; the strings are NUL-terminated and outlive the calls
		unsafe {
			let init = CoInitializeEx(std::ptr::null(), COINIT_APARTMENTTHREADED);
			let mut made = Err("the shell's link object could not be made".to_string());
			let mut shell: *mut c_void = std::ptr::null_mut();
			let rc = CoCreateInstance(
				&CLSID_SHELL_LINK,
				std::ptr::null(),
				CLSCTX_INPROC_SERVER,
				&IID_SHELL_LINK_W,
				&mut shell,
			);
			if rc >= 0 && !shell.is_null() {
				let set = |index: usize, text: &[u16]| {
					std::mem::transmute::<*const c_void, SetText>(method(shell, index))(shell, text.as_ptr())
				};
				set(20, &path(target));
				set(9, &path(folder));
				set(7, &wide("sally's panel: what sally is doing, and its settings"));
				std::mem::transmute::<*const c_void, SetIcon>(method(shell, 17))(shell, path(icon).as_ptr(), 0);
				let mut file: *mut c_void = std::ptr::null_mut();
				let rc =
					std::mem::transmute::<*const c_void, Query>(method(shell, 0))(shell, &IID_PERSIST_FILE, &mut file);
				if rc >= 0 && !file.is_null() {
					let rc = std::mem::transmute::<*const c_void, Save>(method(file, 6))(file, path(link).as_ptr(), 1);
					made = if rc >= 0 { Ok(()) } else { Err(format!("the shortcut could not be saved ({rc:#x})")) };
					std::mem::transmute::<*const c_void, Release>(method(file, 2))(file);
				}
				std::mem::transmute::<*const c_void, Release>(method(shell, 2))(shell);
			}
			if init >= 0 {
				CoUninitialize();
			}
			made
		}
	}
}

#[cfg(not(windows))]
mod sys {
	use std::path::Path;
	use std::process::Command;

	pub enum Kind {
		Ask,
		Retry,
		Tell,
		Fail,
	}

	pub fn message(text: &str, kind: Kind) -> bool {
		eprintln!("{text}");
		matches!(kind, Kind::Ask)
	}

	pub fn no_window(_c: &mut Command) {}

	pub fn register(_root: &Path, _size_kb: u64) -> Result<(), String> {
		Ok(())
	}

	pub fn unregister() {}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A file in use cannot be written over; it is set aside and the new one takes its name. Here nothing holds the
	/// file, so the plain way is what runs; the names are what is checked, and that a sweep clears what was set aside
	#[test]
	fn a_file_is_put_whole_and_what_was_set_aside_is_swept() {
		let dir = std::env::temp_dir().join(format!("sally_setup_put_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		let file = dir.join("driver").join("bin").join("driver_sally.dll");
		put(&file, b"one").unwrap();
		put(&file, b"two").unwrap();
		assert_eq!(std::fs::read(&file).unwrap(), b"two");
		assert_eq!(after(&file, ".old-3").file_name().unwrap(), "driver_sally.dll.old-3");
		// what an install over a running SteamVR leaves, and the user's own files of like names
		std::fs::write(after(&file, ".old-0"), b"old").unwrap();
		std::fs::write(dir.join("sally.exe.new"), b"half").unwrap();
		std::fs::create_dir_all(dir.join("logs")).unwrap();
		std::fs::write(dir.join("logs").join("session.old-1"), b"theirs").unwrap();
		sweep(&dir, 0);
		assert!(!after(&file, ".old-0").exists() && !dir.join("sally.exe.new").exists());
		assert!(dir.join("logs").join("session.old-1").exists() && file.exists());
		remove_but(&dir, &dir.join("logs"));
		assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 0);
		std::fs::remove_dir_all(&dir).unwrap();
	}
}
