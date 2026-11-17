//! sally's tracker: one IMU-driven error-state EKF per device, every light hit a measurement at its own time, starting
//! and recovering on its own (docs/tracking.md). Ported operation for operation from the C tracker it replaced, and
//! held equal to it bit for bit on recorded sessions (docs/checking.md).
//!
//! - `tracker`: the estimator, one `Tracker` per device
//! - `cold_start`: a device's first pose from one station's hits, with no prior (the barycentric SVD)
//! - `host`: what it reads and does outside its own state (the `Host` trait), its settings, and the plain data it
//!   shares with its host
//! - `learned`: what it learns of a device (biases, layout scale), carried from one session to the next
//! - `options`: its options, with their tags, descriptions and defaults
//! - `text`: its lines: printf format strings and their arguments in C types, formatted as glibc's printf does
//!
//! No `unsafe` anywhere in the crate.
#![forbid(unsafe_code)]
// The loops over indices, the comparisons (NaN behaves as written), the copies and the order of operations stay as
// ported, for replays that compute the same bits; the style lints that would rewrite those are off here; anything that
// would change a result is not.
#![allow(
	clippy::needless_range_loop,
	clippy::neg_cmp_op_on_partial_ord,
	clippy::manual_range_contains,
	clippy::manual_memcpy,
	clippy::manual_clamp,
	clippy::field_reassign_with_default
)]

pub mod cold_start;
pub mod host;
pub mod learned;
pub mod options;
pub mod text;
pub mod tracker;
