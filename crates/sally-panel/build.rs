//! sally-panel.exe is installed as one file (with OpenVR's openvr_api.dll beside it), so on Windows llvm-mingw's
//! libunwind (which Rust's standard library links) goes into the program instead of beside it as libunwind.dll: the
//! static library is put in a folder of its own on the linker's search path, ahead of the toolchain's, where `-lunwind`
//! finds it before the DLL's import library. The system DLLs stay DLLs. (sally-check's build.rs, the same.) And the
//! program's icon is linked in: sally's (crates/sally-setup/sally.ico), for Explorer to show.
use std::path::PathBuf;
use std::process::Command;

fn main() {
	println!("cargo:rerun-if-changed=build.rs");
	let windows_gnu = std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows")
		&& std::env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("gnu");
	if !windows_gnu {
		return;
	}
	let target = std::env::var("TARGET").unwrap_or_default();
	let var = format!("CARGO_TARGET_{}_LINKER", target.to_uppercase().replace('-', "_"));
	let linker = std::env::var(var).unwrap_or_else(|_| "x86_64-w64-mingw32-clang".into());
	let found = Command::new(&linker).arg("-print-file-name=libunwind.a").output().ok();
	let lib = found.map(|o| PathBuf::from(String::from_utf8_lossy(&o.stdout).trim()));
	match lib.filter(|l| l.is_absolute() && l.exists()) {
		Some(lib) => {
			let dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR")).join("static");
			std::fs::create_dir_all(&dir).expect("static lib folder");
			std::fs::copy(&lib, dir.join("libunwind.a")).expect("copy libunwind.a");
			println!("cargo:rustc-link-search=native={}", dir.display());
		}
		None => println!(
			"cargo:warning=no static libunwind.a from {linker}: sally-panel.exe will need libunwind.dll beside it"
		),
	}
	// the program's icon, which Explorer shows for the file: sally.ico as its first icon resource, compiled by the
	// toolchain's windres (named as its linker is) and linked into the programs
	let ico = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"))
		.join("../sally-setup/sally.ico");
	println!("cargo:rerun-if-changed={}", ico.display());
	let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
	let (rc, object) = (out.join("icon.rc"), out.join("icon.o"));
	let windres = match linker.rfind("clang") {
		Some(at) => format!("{}windres", &linker[..at]),
		None => "x86_64-w64-mingw32-windres".into(),
	};
	std::fs::write(&rc, format!("1 ICON \"{}\"\n", ico.display().to_string().replace('\\', "/"))).expect("icon.rc");
	let compiled = Command::new(&windres).arg(&rc).args(["-O", "coff", "-o"]).arg(&object).status();
	if compiled.is_ok_and(|s| s.success()) {
		println!("cargo:rustc-link-arg-bins={}", object.display());
	} else {
		println!("cargo:warning=no icon in sally-panel.exe: {windres} did not compile it");
	}
}
