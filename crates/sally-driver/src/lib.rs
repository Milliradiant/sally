//! sally's SteamVR driver (driver_sally.dll): the pose override of docs/sally_design.md section 5 (what it does:
//! docs/sally_driver.md).
//!
//! Valve's lighthouse driver stays on and runs the devices (dongles, pairing, input, battery, haptics). The sally
//! runtime (`sally run` with --runtime-port, see docs/sally_runtime_protocol.md) reads the same
//! dongles read-only and solves; this driver subscribes to the runtime's driver stream over loopback UDP and, through a
//! hook on IVRServerDriverHost::TrackedDevicePoseUpdated, swaps Valve's pose for sally's on Valve's own device entries.
//! Valve's pose is the per-device fallback whenever sally has none, and every switch between the two is blended. The
//! driver never touches the hardware and adds no devices of its own.
//!
//! Settings: sally_settings.txt next to the driver (`sally setup` writes it): "driver.<name>=<value>" lines for the
//! driver, and every other non-comment line is one argument for the runtime, which the driver starts when SteamVR
//! starts (driver.runtime_exe set and no runtime answering), restarts if it exits, and stops when SteamVR exits.
//!
//! The modules:
//! - `openvr`: OpenVR's driver interface as the driver uses it (the structs, the vtables, the driver context)
//! - `helpers`: the small helpers (the log, the clocks, the quaternion and string helpers, the driver's folder)
//! - `settings`: sally_settings.txt
//! - `runtime_process`: the runtime the driver starts
//! - `pose_override`: the hook, the frame fit, the fallback and the blend, the panels' lines
//! - `runtime_device`: a device's reports turned into poses (the hand-over)
//! - `output`: the output layer, what SteamVR is shown per device class and device (sally_output.txt, the panel's
//!   `output` commands): the recovery blend, the smoothing, the prediction, the velocity boost, the offsets
//! - `toys`: the output layer's toys, a hand's motion reshaped for play: the punch rail, and the dash punch's share
//!   of it
//! - `body`: dash punch, the whole body slid with a fist on its rail
//! - `provider`: what vrserver calls, the receive thread and the protocol's lines
//! - `net`, `cstdio`: the sockets and the files, through the C library's calls
//!
//! Exact as the runtime's crates are (docs/checking.md): the same floating-point operations in the same order, the C
//! library's sin, cos, acos and sincos, the same selected values as std::nth_element (without a NaN), printf formats
//! through sally-math's printf, and its sscanf, atof and atoi (glibc's).

// The loops over indices and the comparisons (a NaN behaves as written) stay as they are for that exactness, so the
// style lints that would rewrite those are off; anything that would change a result is not.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::manual_memcpy)]

pub mod body;
pub mod cstdio;
pub mod helpers;
pub mod net;
pub mod openvr;
pub mod output;
pub mod panel;
pub mod pose_override;
pub mod provider;
pub mod runtime_device;
pub mod runtime_process;
pub mod sessions;
pub mod settings;
pub mod toys;
pub mod user;

use std::ffi::{c_char, c_void, CStr};

use openvr::{IServerTrackedDeviceProvider_Version, VRInitError_Init_InterfaceNotFound};

/// HmdDriverFactory: the provider for IServerTrackedDeviceProvider_004, nothing (and the error) for anything else
///
/// # Safety
/// `iface` is a NUL-terminated string and `ret` null or a place for the error, as vrserver passes them
#[no_mangle]
pub unsafe extern "C" fn HmdDriverFactory(iface: *const c_char, ret: *mut i32) -> *mut c_void {
	// SAFETY: vrserver's interface name is a NUL-terminated string (a null one is not the provider's)
	if !iface.is_null() && unsafe { CStr::from_ptr(iface) } == IServerTrackedDeviceProvider_Version {
		return &provider::PROVIDER_OBJECT as *const _ as *mut c_void;
	}
	if !ret.is_null() {
		// SAFETY: ret is the caller's place for the error
		unsafe { *ret = VRInitError_Init_InterfaceNotFound };
	}
	std::ptr::null_mut()
}
