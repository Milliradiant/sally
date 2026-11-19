//! What sally's packed files are made with. A packed file holds what its text adds to what could be told ahead,
//! through an entropy coder that learns as it goes, in blocks that are each whole or not there, and unpacks to the
//! text's own bytes.
//!
//! - `coder`: the entropy coder and the shapes a model is built from (a tap recording's packer, in `sally-hid`, is
//!   written against it too)
//! - `table`: a table of numbers as text, a row a line (the driver's pose log), packed and read back

pub mod coder;
pub mod table;
