//! The runtime: what libsurvive's C runtime did for sally, in Rust (docs/checking.md). It holds the tracked
//! objects and the base stations, and takes each measurement from a device's parser to the recording: the IMU to the
//! device clock, a station's sync to its rotor and OOTX, a sweep hit to its angle on a plane. Ported from
//! its survive.c, survive_process.c, survive_process_gen2.c, survive_disambiguator.c, survive_recording.c and
//! the parts of survive_default_devices.c that make an object, operation for operation and to the byte of text.
//!
//! - `config`: the config store, its file and the command line; `options`: the options registered in it
//! - `runtime`: the context and its processes; `context`: the context lock, what the runtime's threads share
//! - `tracking`: each device's tracker, hosted by the runtime, and the processes its report goes through;
//!   `learned`: what each tracker learned of its device, carried from one session to the next
//! - `driver`: the drivers (libsurvive's plugins) and the hooks they put in the processes' way; `diagnosis`,
//!   `diagrec`, `map` (the map solve's) and `protocol` (the runtime protocol: the streams and the control channel
//!   over UDP) are four
//! - `object`, `station`: a tracked object, a base station
//! - `tap`: the HID tap feeding the runtime (the tap's runtime and each parser's host)
//! - `recording`, `log`: the text the runtime writes, as the C writes it
//! - `settings`: the options the processes read

// The port keeps the C's loops over indices and its comparisons, so the style lints that would rewrite them are off
// here; anything that would change a result is not.
#![allow(clippy::needless_range_loop, clippy::manual_range_contains)]

pub mod config;
pub mod context;
pub mod diagnosis;
pub mod diagrec;
pub mod driver;
pub mod learned;
pub mod log;
pub mod map;
pub mod object;
pub mod options;
pub mod protocol;
pub mod recording;
pub mod runtime;
pub mod settings;
pub mod station;
pub mod tap;
pub mod tracking;

/// A tracked object: its index in the runtime's objects
pub type ObjectId = usize;

pub use runtime::Runtime;
