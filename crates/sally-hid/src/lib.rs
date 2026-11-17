//! The lighthouse devices' reports, from wherever they come: the HID tap's recordings and the tap on the dongles
//! itself, the table of known devices, and the parser that turns a Vive-family report into IMU samples, light pulses
//! and input (the C runtime's driver_vive.c parser, survive_vive_report.c and driver_hidtap.c),
//! ported to give exactly what the C gives (docs/checking.md).
//!
//! - `tap`: the HID tap's driver, shared by its two inputs: its devices, created and configured as they are seen, their
//!   reports to their parsers; the `Runtime` it feeds; the `--hidtap-record` file
//! - `replay`: a tap recording read into the tap (`--hidtap-playback`), on a thread of its own as the C's is, or on the
//!   caller's
//! - `live`: the dongles read into the tap (`--hidtap 1`, Windows)
//! - `line`: the tap recordings' line format (`R` reports, `D` devices)
//! - `devices`: the known devices and which interface a report belongs to
//! - `report`: the report parser, one `Parser` per device; what it finds goes to a `Host`
//! - `config_stream`: a device's config as it streams out of it, inflated

// The port keeps the C's loops over indices and its comparisons, so the style lints that would rewrite them are off
// here; anything that would change a result is not.
#![allow(clippy::needless_range_loop, clippy::manual_range_contains)]

pub mod config_stream;
pub mod devices;
pub mod line;
pub mod live;
pub mod replay;
pub mod report;
pub mod tap;
