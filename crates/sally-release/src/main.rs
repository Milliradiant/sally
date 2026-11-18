//! `sally-release`: the release made from the tree, in one step (docs/release.md).
//!
//!   cargo run -p sally-release -- [--target <triple>] [--out <folder>] [--openvr-api <openvr_api.dll>]
//!
//! 1. The runtime, the driver and the panel are built for the target (Windows unless told otherwise) with the release
//!    profile, in a target folder of their own, with the build machine's paths mapped away (`--remap-path-prefix`): a
//!    Rust program carries the source path of every place that can panic, and those of the dependencies sit under the
//!    builder's home folder, which on a personal machine is the user's name.
//! 2. Each program is read through for those paths, and a Windows one for the toolchain's own DLLs (libunwind.dll:
//!    a program that wants it does not start on a machine without it). Either found is an error: nothing is laid out.
//! 3. THIRD-PARTY-NOTICES.txt is written: the licenses of what sally's own source takes from elsewhere (`licenses/`,
//!    the panel's fonts) and of every crate linked into the three programs, each distinct text once.
//! 4. The folder is laid out as `sally setup` expects to find the project (crates/sally/src/setup.rs), with LICENSE
//!    and the notices beside it, and OpenVR's library beside the panel when it is given (`--openvr-api`: the panel
//!    loads it from there for its overlay and for Valve's poses, and starts without it).
//! 5. The installer is packed beside the folder: sally-setup with the same files after it, in the layout it installs
//!    (`sally_setup::payload`). That one file is the release.
//!
//! It only builds, copies and packs: it neither tags nor publishes anything.
//!
//!   cargo run -p sally-release -- --public <folder> --names <file>
//!
//! writes the tree as the public repository gets it instead (`public`), and builds nothing.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

mod public;

/// The programs a release holds: the package, what cargo names its file (the stem), and whether it is a library
const PROGRAMS: &[(&str, &str, bool)] =
	&[("sally", "sally", false), ("sally-driver", "driver_sally", true), ("sally-panel", "sally-panel", false)];

/// The installer's package (and its file's stem): built and checked with the programs, then packed with them after it
const INSTALLER: &str = "sally-setup";

const WINDOWS: &str = "x86_64-pc-windows-gnullvm";

fn main() {
	if let Err(e) = run(&std::env::args().skip(1).collect::<Vec<_>>()) {
		eprintln!("sally-release: {e}");
		std::process::exit(1);
	}
}

fn run(args: &[String]) -> Result<(), String> {
	let mut target = WINDOWS.to_string();
	let mut out = None;
	let mut openvr_api = None;
	let mut public = None;
	let mut names = None;
	let mut i = 0;
	while i < args.len() {
		match (args[i].as_str(), args.get(i + 1)) {
			("--target", Some(v)) => target = v.clone(),
			("--out", Some(v)) => out = Some(PathBuf::from(v)),
			("--openvr-api", Some(v)) => openvr_api = Some(PathBuf::from(v)),
			("--public", Some(v)) => public = Some(PathBuf::from(v)),
			("--names", Some(v)) => names = Some(PathBuf::from(v)),
			(a, _) => {
				return Err(format!(
					"unknown argument {a} (--target <triple>, --out <folder>, --openvr-api <file>, or --public <folder> \
					 --names <file>)"
				))
			}
		}
		i += 2;
	}
	if let Some(f) = openvr_api.as_ref().filter(|f| !f.is_file()) {
		return Err(format!("{} is not a file", f.display()));
	}
	let root = workspace_root()?;
	if let Some(to) = public {
		let names = names.ok_or("--public wants --names <file>: the names file")?;
		return public::export(&root, &to, &names);
	}
	let version = env!("CARGO_PKG_VERSION");
	let out = out.unwrap_or_else(|| root.join("target").join("dist").join(format!("sally-{version}")));
	let places = Places::of_this_machine(&root)?;

	let build_dir = root.join("target").join("dist-build");
	build(&root, &target, &build_dir, &places)?;
	let built = build_dir.join(&target).join("release");
	let windows = target.contains("windows");
	let mut files = Vec::new();
	for (package, stem, library) in PROGRAMS {
		let name = file_name(stem, *library, windows);
		let path = built.join(&name);
		if !path.is_file() {
			return Err(format!("{package} was built but {} is not there", path.display()));
		}
		files.push((name, path));
	}
	let installer = (file_name(INSTALLER, false, windows), built.join(file_name(INSTALLER, false, windows)));
	if !installer.1.is_file() {
		return Err(format!("{INSTALLER} was built but {} is not there", installer.1.display()));
	}
	let checked: Vec<&(String, PathBuf)> = files.iter().chain(Some(&installer)).collect();

	let mut leaked = 0;
	for (name, path) in &checked {
		let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
		for (needle, at) in leaks(&bytes, &places.needles()) {
			leaked += 1;
			eprintln!("{name}: the build machine's path {needle:?} at byte {at}: {:?}", context(&bytes, at));
		}
	}
	if leaked > 0 {
		return Err(format!("{leaked} of the build machine's paths are in the programs: nothing laid out"));
	}
	println!("no path of this machine in {}", checked.iter().map(|f| f.0.as_str()).collect::<Vec<_>>().join(", "));
	if windows {
		for (name, path) in &checked {
			let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
			let wants = toolchain_dlls(&bytes);
			if !wants.is_empty() {
				return Err(format!(
					"{name} wants {} beside it (the toolchain's, not Windows'): nothing laid out",
					wants.join(", ")
				));
			}
		}
	}

	let notices = notices(&root, &places.cargo_home, &target, version)?;
	lay_out(&root, &out, &files, &notices, windows)?;
	match &openvr_api {
		Some(f) => copy(f, &out.join("tools").join("sally_panel").join(f.file_name().unwrap_or_default()))?,
		None => println!(
			"without --openvr-api the panel has no overlay and no Valve comparison until the file is beside it"
		),
	}
	println!("sally {version} for {target} laid out in {}", out.display());
	let packed = out.with_file_name(&installer.0);
	let size = pack_installer(&root, &packed, &files, &installer.1, &notices, openvr_api.as_deref(), windows)?;
	println!("the installer: {} ({:.1} MB)", packed.display(), size as f64 / 1e6);
	Ok(())
}

/// What cargo names a program's file on the target
fn file_name(stem: &str, library: bool, windows: bool) -> String {
	match (library, windows) {
		(false, true) => format!("{stem}.exe"),
		(false, false) => stem.to_string(),
		(true, true) => format!("{stem}.dll"),
		(true, false) => format!("lib{stem}.so"),
	}
}

fn cargo() -> Command {
	Command::new(std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into()))
}

fn workspace_root() -> Result<PathBuf, String> {
	let o = cargo()
		.args(["locate-project", "--workspace", "--message-format", "plain"])
		.output()
		.map_err(|e| format!("cannot run cargo: {e}"))?;
	let manifest = PathBuf::from(String::from_utf8_lossy(&o.stdout).trim());
	manifest.parent().filter(|_| o.status.success()).map(Path::to_path_buf).ok_or("not in sally's workspace".into())
}

/// The folders of the build machine a program's panic messages would name
struct Places {
	root: PathBuf,
	home: Option<PathBuf>,
	cargo_home: PathBuf,
	rustup_home: Option<PathBuf>,
}

impl Places {
	fn of_this_machine(root: &Path) -> Result<Places, String> {
		let var = |n: &str| std::env::var_os(n).filter(|v| !v.is_empty()).map(PathBuf::from);
		let home = var("HOME").or_else(|| var("USERPROFILE"));
		let cargo_home = var("CARGO_HOME")
			.or_else(|| home.as_ref().map(|h| h.join(".cargo")))
			.ok_or("cannot find cargo's home (CARGO_HOME, or HOME)")?;
		let rustup_home = var("RUSTUP_HOME").or_else(|| home.as_ref().map(|h| h.join(".rustup")));
		Ok(Places { root: root.to_path_buf(), home, cargo_home, rustup_home })
	}

	/// `--remap-path-prefix` for each folder, the widest first: where several match, rustc takes the last
	fn remaps(&self) -> Vec<String> {
		let mut flags = Vec::new();
		let mut add = |from: &Path, to: &str| flags.push(format!("--remap-path-prefix={}={to}", from.display()));
		if let Some(h) = &self.home {
			add(h, "home");
		}
		if let Some(r) = &self.rustup_home {
			add(r, "rustup");
		}
		add(&self.cargo_home, "cargo");
		add(&self.root, "sally");
		flags
	}

	/// What must not be in a program: each folder as a path, with the separator after it (the home folder alone can be
	/// as short as "/root", which is part of other words)
	fn needles(&self) -> Vec<String> {
		let mut n = Vec::new();
		for p in [self.home.as_ref(), self.rustup_home.as_ref(), Some(&self.cargo_home), Some(&self.root)] {
			let Some(p) = p else { continue };
			let text = p.display().to_string();
			for sep in ['/', '\\'] {
				let with = format!("{}{sep}", text.trim_end_matches(['/', '\\']));
				if !n.contains(&with) {
					n.push(with);
				}
			}
		}
		n
	}
}

fn build(root: &Path, target: &str, build_dir: &Path, places: &Places) -> Result<(), String> {
	// RUSTFLAGS as they stand, then the remaps; encoded, so a folder with a space in its name stays one flag
	let mut flags: Vec<String> = match std::env::var("CARGO_ENCODED_RUSTFLAGS") {
		Ok(f) if !f.is_empty() => f.split('\x1f').map(str::to_string).collect(),
		_ => std::env::var("RUSTFLAGS").unwrap_or_default().split_whitespace().map(str::to_string).collect(),
	};
	flags.extend(places.remaps());
	let mut c = cargo();
	c.current_dir(root).args(["build", "--release", "--locked", "--target", target, "--target-dir"]).arg(build_dir);
	for package in PROGRAMS.iter().map(|p| p.0).chain([INSTALLER]) {
		c.args(["-p", package]);
	}
	c.env("CARGO_ENCODED_RUSTFLAGS", flags.join("\x1f")).env_remove("RUSTFLAGS");
	let status = c.status().map_err(|e| format!("cannot run cargo: {e}"))?;
	if status.success() {
		Ok(())
	} else {
		Err("the build failed".into())
	}
}

/// Where each needle is in a program's bytes (every place)
fn leaks(bytes: &[u8], needles: &[String]) -> Vec<(String, usize)> {
	let mut found = Vec::new();
	for n in needles {
		let n8 = n.as_bytes();
		if n8.is_empty() || n8.len() > bytes.len() {
			continue;
		}
		found.extend(bytes.windows(n8.len()).enumerate().filter(|(_, w)| *w == n8).map(|(at, _)| (n.clone(), at)));
	}
	found
}

/// llvm-mingw's own DLLs a Windows program names: a release's programs are whole without them (each crate's build.rs
/// links the unwinder in), and one that is not stops at Windows' "the code execution cannot proceed" on a machine
/// without the toolchain
fn toolchain_dlls(bytes: &[u8]) -> Vec<&'static str> {
	const DLLS: [&str; 3] = ["libunwind.dll", "libc++.dll", "libwinpthread-1.dll"];
	let lower = bytes.to_ascii_lowercase();
	DLLS.into_iter().filter(|d| lower.windows(d.len()).any(|w| w == d.as_bytes())).collect()
}

/// The printable text around a place in a program
fn context(bytes: &[u8], at: usize) -> String {
	let printable = |b: &u8| (0x20..0x7f).contains(b);
	let from = (0..at).rev().take(40).take_while(|&i| printable(&bytes[i])).last().unwrap_or(at);
	let to = (at..bytes.len()).take(100).take_while(|&i| printable(&bytes[i])).last().map_or(at, |i| i + 1);
	String::from_utf8_lossy(&bytes[from..to]).into_owned()
}

/// One crate of `cargo tree --format "{p}|{l}|{r}"`: its name, version, license and repository; None for one of the
/// workspace's own (a path follows its version) and for a line that is not a crate
fn tree_line(line: &str) -> Option<(String, String, String, String)> {
	let mut parts = line.splitn(3, '|');
	let package = parts.next()?.trim().trim_end_matches(" (*)");
	let license = parts.next()?.trim();
	let repository = parts.next().unwrap_or("").trim().trim_end_matches(" (*)");
	let mut words = package.split(' ');
	let name = words.next().filter(|n| !n.is_empty())?;
	let version = words.next()?.strip_prefix('v')?;
	// what follows the version: "(proc-macro)", or the folder of a crate that is not from a registry
	if words.any(|w| w.starts_with("(/") || w.starts_with("(file:") || w.contains(":\\")) {
		return None;
	}
	Some((name.to_string(), version.to_string(), license.to_string(), repository.to_string()))
}

/// Whether a file at a crate's top is a license or a notice, by its name
fn is_license_file(name: &str) -> bool {
	let n = name.to_ascii_lowercase();
	["license", "licence", "copying", "notice", "unlicense", "copyright"].iter().any(|p| n.starts_with(p))
}

/// A crate's folder among cargo's unpacked sources
fn crate_dir(cargo_home: &Path, name: &str, version: &str) -> Option<PathBuf> {
	let src = cargo_home.join("registry").join("src");
	std::fs::read_dir(src).ok()?.flatten().map(|e| e.path().join(format!("{name}-{version}"))).find(|p| p.is_dir())
}

fn read_text(path: &Path) -> Result<String, String> {
	let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
	Ok(String::from_utf8_lossy(&bytes).replace("\r\n", "\n"))
}

/// THIRD-PARTY-NOTICES.txt
fn notices(root: &Path, cargo_home: &Path, target: &str, version: &str) -> Result<String, String> {
	let mut c = cargo();
	c.current_dir(root).args(["tree", "--locked", "--target", target, "-e", "normal", "--prefix", "none"]);
	c.args(["--format", "{p}|{l}|{r}"]);
	for package in PROGRAMS.iter().map(|p| p.0).chain([INSTALLER]) {
		c.args(["-p", package]);
	}
	let o = c.output().map_err(|e| format!("cannot run cargo: {e}"))?;
	if !o.status.success() {
		return Err(format!("cargo tree failed: {}", String::from_utf8_lossy(&o.stderr).trim()));
	}
	let mut crates: BTreeMap<(String, String), (String, String)> = BTreeMap::new();
	for line in String::from_utf8_lossy(&o.stdout).lines() {
		if let Some((name, ver, license, repo)) = tree_line(line) {
			crates.insert((name, ver), (license, repo));
		}
	}
	if crates.is_empty() {
		return Err("cargo tree named no crate".into());
	}

	let rule = "=".repeat(100);
	let mut t = format!(
		"sally {version}: third-party notices\n\nsally is under the MIT license (LICENSE). It holds or is linked with \
		 the work of others, under the licenses below.\n\n{rule}\n1. In sally's own source\n{rule}\n\n\
		 - libsurvive and jsmn (MIT), which sally's runtime is a port of: their notices are in LICENSE.\n\
		 - OpenVR (BSD-3-Clause, Valve Corporation): the driver and the panel declare its interfaces after its headers.\n\
		 - Python (PSF-2.0): `sally convert` reads JSON the way CPython's json module does and `sally ctl` its arguments \
		 the way argparse does, each after CPython's source.\n\
		 - IBM Plex (OFL-1.1) and Phosphor Icons (MIT): the panel's fonts.\n"
	);
	let own = [
		("OpenVR", root.join("licenses").join("OpenVR.txt")),
		("Python", root.join("licenses").join("Python.txt")),
		("IBM Plex", root.join("crates").join("sally-panel").join("fonts").join("OFL-IBM-Plex.txt")),
		("Phosphor Icons", root.join("crates").join("sally-panel").join("fonts").join("MIT-Phosphor.txt")),
	];
	for (what, path) in own {
		t.push_str(&format!(
			"\n{}\n{what}\n{}\n\n{}\n",
			"-".repeat(100),
			"-".repeat(100),
			read_text(&path)?.trim_end()
		));
	}

	t.push_str(&format!("\n{rule}\n2. The Rust crates linked into sally's programs ({})\n{rule}\n\n", crates.len()));
	// the standard library is in every program and in no manifest
	t.push_str(
		"The Rust standard library (MIT OR Apache-2.0, https://github.com/rust-lang/rust) is linked into each program \
		 as well.\n\n",
	);
	// each distinct text once, with the crates it came with
	let mut texts: BTreeMap<String, Vec<String>> = BTreeMap::new();
	let mut without = Vec::new();
	for ((name, ver), (license, repo)) in &crates {
		t.push_str(&format!(
			"{name} {ver}: {license}{}\n",
			if repo.is_empty() { String::new() } else { format!(", {repo}") }
		));
		let dir = crate_dir(cargo_home, name, ver).ok_or(format!("{name} {ver} is not among cargo's sources"))?;
		let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
			.map_err(|e| format!("cannot read {}: {e}", dir.display()))?
			.flatten()
			.filter(|e| e.file_type().is_ok_and(|f| f.is_file()) && is_license_file(&e.file_name().to_string_lossy()))
			.map(|e| e.path())
			.collect();
		files.sort();
		if files.is_empty() {
			without.push(format!("{name} {ver} ({license})"));
		}
		for f in files {
			let text = read_text(&f)?.trim().to_string();
			let file = f.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
			texts.entry(text).or_default().push(format!("{name} {ver} ({file})"));
		}
	}
	if !without.is_empty() {
		t.push_str(&format!(
			"\nPackaged without a license file of their own (the license named is the one their manifest gives):\n  {}\n",
			without.join("\n  ")
		));
	}
	// in the order of the first crate each text belongs to
	let mut by_crate: Vec<(Vec<String>, String)> = texts.into_iter().map(|(text, of)| (of, text)).collect();
	by_crate.sort();
	for (of, text) in by_crate {
		t.push_str(&format!("\n{}\n{}\n{}\n\n{text}\n", "-".repeat(100), of.join("\n"), "-".repeat(100)));
	}
	Ok(t)
}

fn copy(from: &Path, to: &Path) -> Result<(), String> {
	if let Some(dir) = to.parent() {
		std::fs::create_dir_all(dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
	}
	std::fs::copy(from, to).map(|_| ()).map_err(|e| format!("cannot copy {} to {}: {e}", from.display(), to.display()))
}

fn copy_tree(from: &Path, to: &Path) -> Result<(), String> {
	for e in std::fs::read_dir(from).map_err(|e| format!("cannot read {}: {e}", from.display()))?.flatten() {
		let (src, dst) = (e.path(), to.join(e.file_name()));
		if src.is_dir() {
			copy_tree(&src, &dst)?;
		} else {
			copy(&src, &dst)?;
		}
	}
	Ok(())
}

/// The release's folder, as `sally setup` finds its way in it: the runtime in survive\hidtap, the driver's folder
/// (SteamVR's layout) in steamvr_driver\sally, the panel in tools\sally_panel, and logs\ for what sessions write
fn lay_out(root: &Path, out: &Path, files: &[(String, PathBuf)], notices: &str, windows: bool) -> Result<(), String> {
	if out.exists() {
		std::fs::remove_dir_all(out).map_err(|e| format!("cannot clear {}: {e}", out.display()))?;
	}
	let driver = out.join("steamvr_driver").join("sally");
	copy_tree(&root.join("crates").join("sally-driver").join("package"), &driver)?;
	let places = [
		out.join("survive").join("hidtap"),
		driver.join("bin").join(if windows { "win64" } else { "linux64" }),
		out.join("tools").join("sally_panel"),
	];
	for ((name, path), place) in files.iter().zip(&places) {
		copy(path, &place.join(name))?;
	}
	for dir in [out.join("logs"), out.join("survive").join("hidtap").join("devcfg")] {
		std::fs::create_dir_all(&dir).map_err(|e| format!("cannot make {}: {e}", dir.display()))?;
	}
	copy(&root.join("LICENSE"), &out.join("LICENSE.txt"))?;
	std::fs::write(out.join("THIRD-PARTY-NOTICES.txt"), notices)
		.map_err(|e| format!("cannot write the notices in {}: {e}", out.display()))
}

/// Every file under a folder, as the installer's files: each path from `under` on, '/' between folders
fn tree(from: &Path, under: &str, into: &mut Vec<sally_setup::payload::File>) -> Result<(), String> {
	let mut entries: Vec<_> =
		std::fs::read_dir(from).map_err(|e| format!("cannot read {}: {e}", from.display()))?.flatten().collect();
	entries.sort_by_key(|e| e.file_name());
	for e in entries {
		let (src, path) = (e.path(), format!("{under}/{}", e.file_name().to_string_lossy()));
		if src.is_dir() {
			tree(&src, &path, into)?;
		} else {
			let bytes = std::fs::read(&src).map_err(|e| format!("cannot read {}: {e}", src.display()))?;
			into.push(sally_setup::payload::File { path, bytes });
		}
	}
	Ok(())
}

/// The installer: sally-setup (`stub`) with sally's files packed after it (`sally_setup::payload`), in the layout its
/// `sally setup` finds beside itself: sally.exe and the panel (OpenVR's library with it) side by side, the driver's
/// folder as SteamVR wants it in driver/, the icon, LICENSE and the notices. `files`: the runtime, the driver and the
/// panel, in PROGRAMS' order. The installer's size
fn pack_installer(
	root: &Path, to: &Path, files: &[(String, PathBuf)], stub: &Path, notices: &str, openvr_api: Option<&Path>,
	windows: bool,
) -> Result<usize, String> {
	use sally_setup::payload::{pack, File};
	let read = |p: &Path| std::fs::read(p).map_err(|e| format!("cannot read {}: {e}", p.display()));
	let driver_bin = format!("driver/bin/{}/", if windows { "win64" } else { "linux64" });
	let mut packed = Vec::new();
	for ((name, path), place) in files.iter().zip(["", driver_bin.as_str(), ""]) {
		packed.push(File { path: format!("{place}{name}"), bytes: read(path)? });
	}
	tree(&root.join("crates").join("sally-driver").join("package"), "driver", &mut packed)?;
	if let Some(f) = openvr_api {
		packed.push(File { path: f.file_name().unwrap_or_default().to_string_lossy().into_owned(), bytes: read(f)? });
	}
	packed
		.push(File { path: "sally.ico".into(), bytes: read(&root.join("crates").join(INSTALLER).join("sally.ico"))? });
	packed.push(File { path: "LICENSE.txt".into(), bytes: read(&root.join("LICENSE"))? });
	packed.push(File { path: "THIRD-PARTY-NOTICES.txt".into(), bytes: notices.as_bytes().to_vec() });
	let mut out = read(stub)?;
	out.extend(pack(&packed)?);
	std::fs::write(to, &out).map_err(|e| format!("cannot write {}: {e}", to.display()))?;
	Ok(out.len())
}

#[cfg(test)]
mod tests {
	use super::*;

	fn places() -> Places {
		Places {
			root: PathBuf::from("/home/someone/src/sally"),
			home: Some(PathBuf::from("/home/someone")),
			cargo_home: PathBuf::from("/home/someone/.cargo"),
			rustup_home: Some(PathBuf::from("/home/someone/.rustup")),
		}
	}

	#[test]
	fn a_program_that_names_the_toolchains_dll_is_found() {
		assert_eq!(toolchain_dlls(b"\0KERNEL32.dll\0api-ms-win-crt-math-l1-1-0.dll\0"), Vec::<&str>::new());
		assert_eq!(toolchain_dlls(b"\0KERNEL32.dll\0LIBUNWIND.DLL\0"), vec!["libunwind.dll"]);
	}

	/// rustc takes the last remap that matches, so the folders inside the home folder come after it
	#[test]
	fn the_remaps_go_from_the_widest_folder_to_the_narrowest() {
		assert_eq!(
			places().remaps(),
			[
				"--remap-path-prefix=/home/someone=home",
				"--remap-path-prefix=/home/someone/.rustup=rustup",
				"--remap-path-prefix=/home/someone/.cargo=cargo",
				"--remap-path-prefix=/home/someone/src/sally=sally",
			]
		);
	}

	#[test]
	fn a_path_of_the_build_machine_in_a_program_is_found() {
		let needles = places().needles();
		assert!(needles.contains(&"/home/someone/".to_string()) && needles.contains(&"/home/someone\\".to_string()));
		let clean =
			b"\0cargo/registry/src/index.crates.io-1949cf8c6b5b557f/egui-0.33.0/src/ui.rs\0/home/someone-else/x";
		assert!(leaks(clean, &needles).is_empty());
		let dirty =
			b"\x01\x02/home/someone/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/egui-0.33.0/src/ui.rs\0";
		let found = leaks(dirty, &needles);
		// the home folder and cargo's inside it
		assert_eq!(found.iter().map(|f| f.1).collect::<Vec<_>>(), [2, 2]);
		assert!(
			context(dirty, 2).starts_with("/home/someone/.cargo/registry/src/") && context(dirty, 2).ends_with("ui.rs")
		);
		assert!(leaks(b"", &needles).is_empty());
	}

	#[test]
	fn the_dependency_list_gives_the_registrys_crates() {
		let c = |l: &str| tree_line(l).map(|c| (c.0, c.1, c.2));
		assert_eq!(
			c("egui v0.33.0|MIT OR Apache-2.0|https://github.com/emilk/egui"),
			Some(("egui".into(), "0.33.0".into(), "MIT OR Apache-2.0".into()))
		);
		assert_eq!(c("ahash v0.8.12|MIT OR Apache-2.0|https://github.com/tkaitchuck/ahash (*)").unwrap().1, "0.8.12");
		assert_eq!(c("bytemuck_derive v1.10.2 (proc-macro)|Zlib OR Apache-2.0 OR MIT|").unwrap().0, "bytemuck_derive");
		assert_eq!(tree_line("x v1.0.0||").unwrap().3, "");
		// the workspace's own crates, and what is not a crate
		assert_eq!(c("sally-math v0.14.0 (/home/someone/src/sally/crates/sally-math)|MIT|"), None);
		assert_eq!(c("sally-math v0.14.0 (C:\\src\\sally\\crates\\sally-math)|MIT|"), None);
		assert_eq!(c(""), None);
		assert_eq!(c("egui 0.33.0|MIT|"), None);
	}

	#[test]
	fn license_files_by_name() {
		for n in ["LICENSE", "LICENSE-MIT", "license.md", "LICENCE.txt", "COPYING", "NOTICE", "UNLICENSE", "COPYRIGHT"]
		{
			assert!(is_license_file(n), "{n}");
		}
		for n in ["README.md", "Cargo.toml", "CHANGELOG.md", "src"] {
			assert!(!is_license_file(n), "{n}");
		}
	}

	#[test]
	fn the_programs_files_by_target() {
		assert_eq!(file_name("sally", false, true), "sally.exe");
		assert_eq!(file_name("driver_sally", true, true), "driver_sally.dll");
		assert_eq!(file_name("driver_sally", true, false), "libdriver_sally.so");
		assert_eq!(file_name("sally-panel", false, false), "sally-panel");
	}
}
