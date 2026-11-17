//! sally.exe is one file wherever it is put (the driver starts it from the folder `sally setup` names), so on Windows
//! llvm-mingw's libunwind (which Rust's standard library links) goes into the program instead of beside it as
//! libunwind.dll: the static library is put in a folder of its own on the linker's search path, ahead of the
//! toolchain's, where `-lunwind` finds it before the DLL's import library. The system DLLs stay DLLs. (sally-panel's
//! build.rs, the same.)
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
		None => {
			println!("cargo:warning=no static libunwind.a from {linker}: sally.exe will need libunwind.dll beside it")
		}
	}
}
