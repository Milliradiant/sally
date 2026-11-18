//! The sally panel (docs/sally_panel.md): a window that shows what the runtime is doing, live, and compares it against
//! Valve's driver when that runs; the same panel is the SteamVR dashboard overlay. A client of the runtime protocol
//! (docs/sally_runtime_protocol.md) and of OpenVR; it never starts SteamVR.
//!
//! The library is the panel without its window: `ui` (the panel itself, drawn into any egui context), `runtime` (the
//! protocol client), `history` (a device's last minutes), `settings` (sally-panel.cfg), `replay` (a recorded session
//! run again and set beside what SteamVR got), `models` (the devices' 3D models from SteamVR's install), `valve` (the
//! comparison with Valve's driver and the overlay's OpenVR side) over `openvr` (OpenVR's C API, loaded at run time),
//! `offscreen` (a view drawn into a texture: the overlay, its button), and for tests and screenshots `headless`
//! (rendering without a display) and `fake_runtime`. The program (`main.rs`) is the window and the overlay around them.

pub mod check;
pub mod fake_runtime;
#[cfg(target_os = "linux")]
pub mod headless;
pub mod history;
pub mod icon;
pub mod models;
pub mod offscreen;
pub mod openvr;
pub mod overlay_input;
pub mod replay;
pub mod runtime;
pub mod sensor_view;
pub mod settings;
pub mod symbols;
pub mod ui;
pub mod valve;
