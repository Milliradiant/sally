// The Windows driver is one file in SteamVR's driver folder, with no DLLs of its own beside it, as the C++ driver's
// CMake build made it (`-static`). Rust's precompiled std needs libunwind for its unwinding tables even with panic =
// "abort", and asks for it as a DLL (libunwind.dll); the driver links llvm-mingw's static libunwind.a first, so lld
// takes the unwinder from it and the import library is never used. The system's DLLs and the ucrt's stay as they are.
fn main() {
	let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
	let target_env = std::env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
	if target_os == "windows" && target_env == "gnu" {
		println!("cargo:rustc-link-lib=static:-bundle=unwind");
	}
}
