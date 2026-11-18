//! A session made fit to send to someone else: the recording, the devices' configs, the room's files and the logs
//! with a stand-in for every serial and nothing that names the PC or its user, checked, and zipped.
//!
//! - `names`: the stand-ins, kept per PC
//! - `text`: logs and the room's files scrubbed, and the check that text is clean
//! - `config`: a device's config cut down to what sally reads
//! - `light`: the base stations' ids, which the light itself carries bit by bit, rewritten in a recording
//! - `recording`: a recording cut to a span, with stand-ins and without what nothing reads
//! - `package`: all of it put together, checked and zipped

pub mod config;
pub mod light;
pub mod names;
pub mod package;
pub mod recording;
pub mod text;

pub use names::Names;
pub use package::{share, Request, Shared};
pub use text::Private;
