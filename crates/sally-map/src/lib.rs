//! sally's live base station map solve, ported from the C runtime's driver_map_solve.c operation for operation and to
//! the byte of text (docs/checking.md; the design: docs/sally_design.md 4.2). All devices solve the station poses
//! together from their raw sweep angles, and the map moves (slowly, its frame held) instead of staying locked:
//!
//! - snapshots: whenever a device moves slowly and two or more stations each give it enough sweep hits within about
//!   one rotor turn, its raw sweep angles are kept with the filter's pose as a seed only (back-facing hits dropped)
//! - a joint solve every map-solve-period s over the snapshots since the last one: Gauss-Newton (Levenberg) over every
//!   snapshot pose and every station pose at once, robust, the Schur complement on the station block
//! - the map remembers: what each solve learned about the stations is kept as an information matrix and enters the
//!   next solve as a prior, fading with map-solve-tau
//! - the frame held (a weak prior to the calibrated map, the rigid motion of the station set removed), the solution
//!   slewed in, a solve that moves a station too far not applied
//! - the map file (map-solve-file): written at exit, read at the next start, by station id
//! - moved stations (map-solve-relocate): a station whose light disagrees with the map while the others agree is left
//!   out, solved on its own (resection) and, once a second batch confirms it, put at its new place at once
//! - the station factory-calibration corrections (station-fcal-file) and the devices' sensor layout corrections
//!   (device-calibration-file), which the C keeps in the same file
//!
//! The modules:
//! - `map_solve`: `MapSolve`, the runtime's entry points (start, the sweep_angle and imupose hooks, close), the hooks'
//!   side of its state
//! - `solving`: one solve of the batch (run_solve), inline or on the solve thread (`MapSolve::start_thread`), which
//!   reaches the hooks' side and the runtime through the context lock (`Locked`)
//! - `host`: `Host`, what it reads from the runtime and does in it (the config store's options included)
//! - `settings`: its bound options, from the config store as it starts or from the variables a runtime binds to them
//! - `solve`, `relocate`, `model`, `linalg`: the solve
//! - `calibrate`: `sally calibrate`'s solve, a room's stations placed from nothing (it uses `solve` and `model`)
//! - `fcal_solve`: the stations' corrections solved with them, for `calibrate`
//! - `map_file`, `corrections`: the files it reads and writes (read through sally-math's `scanf`, the C library's text
//!   in as the C gets it)
//! - `text`: the C library's text out, as the C gets it

// The port keeps the C's loops over indices, its comparisons (NaN behaves as it did) and its order of operations, so
// the style lints that would rewrite those are off here; anything that would change a result is not.
#![allow(
	clippy::needless_range_loop,
	clippy::neg_cmp_op_on_partial_ord,
	clippy::manual_range_contains,
	clippy::manual_clamp,
	clippy::assign_op_pattern
)]

pub mod calibrate;
pub mod corrections;
pub mod fcal_solve;
pub mod host;
pub mod linalg;
pub mod map_file;
pub mod map_solve;
pub mod model;
pub mod relocate;
pub mod settings;
pub mod solve;
pub mod solving;
pub mod text;

#[cfg(test)]
mod tests;

pub use host::{DeviceId, Host, Velocity};
pub use map_solve::{MapSolve, Stats};
pub use settings::{Settings, Source};
pub use solving::Locked;
