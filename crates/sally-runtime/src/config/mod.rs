//! libsurvive's config store, its file and its command line, ported from survive_config.c, json_helpers.c,
//! jsmn.c and the config part of survive.c (survive_init_internal's argument loop, survive_process_env,
//! find_correct_config_file), held to the C's behaviour: the same values to the bit, the same text to the byte.
//!
//! One [`Config`] holds what the C keeps in statics and in the SurviveContext: the registry of known options (in
//! registration order, which the runtime fills explicitly with [`Config::register`] where the C's constructors did),
//! the global group (the config file's top level), the temporary group (the command line and the environment, looked
//! up first, never saved) and one group per lighthouse. Lookups keep the C's names: `configf` / `configi` / `configb`
//! / `configs` are survive_configf / i / b / s, with the same flags ([`SC_GET`], [`SC_SET`], [`SC_OVERRIDE`],
//! [`SC_SETCONFIG`]) and the same conversions when a value is read as another type than it was set or registered as.
//!
//! Strings are bytes, as the C's are: tags and values from the file, the command line or the environment are kept
//! and written back as they came, and every string is cut at its first NUL as a C string would be.
//!
//! A variable bound to a tag (survive_attach_config) is a [`Binding`]: a handle the owner keeps and reads, which the
//! store writes into whenever that entry is set, converted as the C converts for the variable's type. It is the C's
//! pointer made safe, not a re-read: a set reaches exactly the variables the C would reach (those bound to that
//! entry), and [`Config::bound_count`] counts them as survive_config_bound_count does. There are no locks (the C's
//! per-group mutex): changing the store takes `&mut`, and a binding can be read from any thread.
//!
//! The order of the startup steps decides the order of the global group's entries, and so the file config_save
//! writes. survive_init_internal, after the options are registered: [`Config::apply_args`]; [`Config::apply_env`];
//! `configs("log", SC_GET, None)` and `configi("record-stdout", SC_GET, 0)`; if "configfile" is not set,
//! [`Config::find_correct_config_file`] on [`PLAYBACK_CONFIG_PREFIX_FIELDS`]; `configi("v", SC_SETCONFIG, 0)` (which
//! makes "v" the first global entry unless the command line set it); [`Config::read`] of "init-configfile" or else of
//! [`Config::file_path`]; then, if "configfile" was not set and `find_correct_config_file` on
//! [`RECORD_CONFIG_PREFIX_FIELDS`] names one, [`Config::save`] with no station's OOTXSet (none is read yet); then the
//! context's three bound floats ("lighthouse-max-update", "lighthouse-max-nudge-distance",
//! "lighthouse-update-velocity"); then [`Config::read_lighthouse`] for each of the 16 stations.
//!
//! Where the C's behaviour is undefined (reading uninitialized memory, past a buffer, or through a NULL pointer), the
//! port picks the nearest defined behaviour and says so at the function. The store reads and prints numbers through one
//! file (cnum.rs), one function per C library function or cast (the floats through sally-math's).

#[cfg(all(test, target_os = "linux"))]
mod against_c;
mod args;
mod cnum;
mod file;
mod json;
mod lighthouse;
mod store;
#[cfg(test)]
mod tests;

pub use args::{arguments, environment, ArgsOutcome, PLAYBACK_CONFIG_PREFIX_FIELDS, RECORD_CONFIG_PREFIX_FIELDS};
pub use file::{write_whole, SaveOutcome, DEFAULT_CONFIG_PATH, FILENAME_MAX};
pub use lighthouse::LighthouseRecord;
pub use store::{
	Binding, Config, ConfigEntry, ConfigGroup, ConfigOption, DetachError, EntryType, OptionDefault, VarType,
	NUM_GEN2_LIGHTHOUSES, SC_GET, SC_OVERRIDE, SC_SET, SC_SETCONFIG,
};
