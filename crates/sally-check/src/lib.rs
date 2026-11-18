//! sally's check: what a session's devices sent, read per device for where tracking trouble comes from (the light,
//! the radio, the device), as a page; and the session made into one file to send to someone else, without what says
//! whose it is (`share`). The panel runs both; `sally-check <folder | zip>` checks a session that came back.
//!
//! - `analysis`: every report through its device's parser, counted per quarter second
//! - `events`: what SteamVR showed going wrong, from its own poses
//! - `report`: the check in words and as a page
//! - `session`: a session's folder, as it is written and read back
//! - `share`: a recording and what belongs to it with stand-ins for every serial, and the check of that
//! - `zip`: the folder as one file

pub mod analysis;
pub mod events;
pub mod report;
pub mod session;
pub mod share;
pub mod zip;
