//! The body behind the trackers, for the virtual trackers (docs/mocap_plan.md): a session's poses as `sally-poses` logs
//! them and where each device had light (`poses`); which device is on which body part, found from the T-pose at the
//! session's start the way VRChat assigns trackers, never by serial or by kind (`roles`); the joints' centres fitted
//! from the motion (`fit`); the IK baseline the virtual trackers are scored against (`ik`); and motion capture (`bvh`)
//! with the tracker rig put on it (`synth`), the training data; and what the model sees and says (`features`).
//!
//! SteamVR's app space throughout: metres, Y up, a headset looking along its -Z.

// small dense algebra and per-side loops read best by index
#![allow(clippy::needless_range_loop)]

pub mod bvh;
pub mod features;
pub mod fit;
pub mod ik;
pub mod poses;
pub mod roles;
pub mod synth;
