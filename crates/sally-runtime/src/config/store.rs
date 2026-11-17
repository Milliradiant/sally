//! The store: the registry of known options, the config groups and their entries, and the lookups (survive_config.c).

use std::borrow::Cow;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use super::cnum::{self, c_str};

/// survive_types.h: the most stations a gen2 system has, and so the number of lighthouse groups
pub const NUM_GEN2_LIGHTHOUSES: usize = 16;

/// survive_config_flags (survive.h). SC_GET only reads. Without SC_OVERRIDE a lookup that finds the tag set returns
/// its value and sets nothing; otherwise the value is the default (the registered one unless SC_OVERRIDE, which takes
/// the caller's), written to the temporary group with SC_SET, to the temporary and the global group with SC_SETCONFIG
pub const SC_GET: u8 = 0;
/// "Set, if not present"
pub const SC_SET: u8 = 1;
/// "Set, to new default value"
pub const SC_OVERRIDE: u8 = 2;
/// "Set, both in-memory and config file. Use in conjunction with SC_OVERRIDE."
pub const SC_SETCONFIG: u8 = 4;

/// What a registered option's default is (data_default and the type char of static_conf_t)
#[derive(Clone, Debug, PartialEq)]
pub enum OptionDefault {
	/// 'i': an int
	Int(i32),
	/// 'f': a FLT, a double
	Float(f64),
	/// 'b': a bool
	Bool(bool),
	/// 's': a string, or NULL (None)
	Str(Option<Vec<u8>>),
}

impl OptionDefault {
	/// An 's' default from text
	pub fn text(s: &str) -> OptionDefault {
		OptionDefault::Str(Some(s.as_bytes().to_vec()))
	}

	fn type_char(&self) -> u8 {
		match self {
			OptionDefault::Int(_) => b'i',
			OptionDefault::Float(_) => b'f',
			OptionDefault::Bool(_) => b'b',
			OptionDefault::Str(_) => b's',
		}
	}
}

/// Where the C reads a non-NULL string default as a number (survive_configi / f / b on an 's' option) it reads the
/// string's address: never zero, never the same twice. The port reads this stand-in, nonzero like the address
const STRING_DEFAULT_ADDRESS: u64 = 0x1000;

/// One registered option: static_conf_t (survive_config.c), what survive_config_bind_variable and STATIC_CONFIG_ITEM
/// register
#[derive(Clone, Debug)]
pub struct ConfigOption {
	name: Vec<u8>,
	description: Vec<u8>,
	type_char: u8,
	/// data_default, the C's union { FLT f; int i; char *s; bool b; } as its 8 bytes: an int or a bool fills the low
	/// bytes of a zeroed union, so every lookup that reads another member than the one registered (survive_configf on
	/// an int option reads the int's bits as a double) reads these bits as the C does
	bits: u64,
	/// data_default.s for an 's' option
	text: Option<Vec<u8>>,
}

impl ConfigOption {
	pub fn name(&self) -> &[u8] {
		&self.name
	}

	pub fn description(&self) -> &[u8] {
		&self.description
	}

	/// 'i', 'f', 's' or 'b'
	pub fn type_char(&self) -> u8 {
		self.type_char
	}

	pub fn default_value(&self) -> OptionDefault {
		match self.type_char {
			b'i' => OptionDefault::Int(self.bits as u32 as i32),
			b'f' => OptionDefault::Float(f64::from_bits(self.bits)),
			b'b' => OptionDefault::Bool(self.bits != 0),
			_ => OptionDefault::Str(self.text.clone()),
		}
	}

	/// data_default.i
	fn int_bits(&self) -> u32 {
		self.bits as u32
	}

	/// data_default.f
	fn float_bits(&self) -> f64 {
		f64::from_bits(self.bits)
	}
}

/// cval_type (survive_config.h): what an entry holds. CONFIG_UNKNOWN only exists in the C between creating an entry and
/// setting it
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryType {
	/// CONFIG_FLOAT
	Float = 1,
	/// CONFIG_UINT32
	Uint32 = 2,
	/// CONFIG_STRING
	String = 3,
	/// CONFIG_FLOAT_ARRAY
	FloatArray = 4,
}

/// The type of a variable bound to a config entry: the `type` char of survive_attach_config
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VarType {
	/// 'i': an int32_t (written as the uint32 or the truncated double the entry is set to)
	Int,
	/// 'b': a bool
	Bool,
	/// 'f': a FLT
	Float,
	/// 's': a char buffer, copied once when attached (sally: a set does not reach it)
	Str,
}

#[derive(Debug)]
struct BindingCell {
	var_type: VarType,
	/// the variable's bits: the int's 32, the bool's 0 / 1 or the double's 64
	bits: AtomicU64,
	text: Mutex<Vec<u8>>,
}

/// sally: the writes the bound variables have had, all of them together (`Binding::writes`)
static BINDING_WRITES: AtomicU64 = AtomicU64::new(0);

/// A variable bound to a config entry: survive_attach_config's `var`, made safe. The store holds a clone of the handle
/// in the entry's update list and writes each later config_set_uint32 / config_set_float of that entry into it,
/// converted as the C converts for the variable's type; the owner reads it at any time (from any thread). A handle is
/// identified by its allocation, as the C identifies a variable by its address: attaching the same handle twice to an
/// entry binds it once, and detaching takes it off.
#[derive(Clone, Debug)]
pub struct Binding(Arc<BindingCell>);

impl Binding {
	/// A variable of this type, zero (or empty) until attached
	pub fn of(var_type: VarType) -> Binding {
		Binding(Arc::new(BindingCell { var_type, bits: AtomicU64::new(0), text: Mutex::new(Vec::new()) }))
	}

	pub fn var_type(&self) -> VarType {
		self.0.var_type
	}

	/// sally: how many writes every bound variable together has had so far (any variable's, from any store): while it
	/// stays the same, so do all their values, and a reader that copied them out need not read them again
	pub fn writes() -> u64 {
		BINDING_WRITES.load(Ordering::Acquire)
	}

	/// An 'i' variable
	pub fn int(&self) -> i32 {
		self.0.bits.load(Ordering::Relaxed) as u32 as i32
	}

	/// A 'b' variable
	pub fn bool(&self) -> bool {
		self.0.bits.load(Ordering::Relaxed) != 0
	}

	/// An 'f' variable
	pub fn float(&self) -> f64 {
		f64::from_bits(self.0.bits.load(Ordering::Relaxed))
	}

	/// An 's' variable
	pub fn text(&self) -> Vec<u8> {
		self.0.text.lock().map(|t| t.clone()).unwrap_or_default()
	}

	/// The owner's own write to its 'f' variable, a plain assignment in the C (a driver clamping an option it bound):
	/// the entry keeps its value, and the entry's next set writes over this one
	pub fn set_float(&self, value: f64) {
		debug_assert!(self.0.var_type == VarType::Float, "set_float on a {:?} variable", self.0.var_type);
		self.store_bits(value.to_bits());
	}

	fn same(&self, other: &Binding) -> bool {
		Arc::ptr_eq(&self.0, &other.0)
	}

	fn store_bits(&self, bits: u64) {
		self.0.bits.store(bits, Ordering::Relaxed);
		// after the value: a reader that sees the count moved sees the value it moved for
		BINDING_WRITES.fetch_add(1, Ordering::Release);
	}

	fn store_text(&self, text: &[u8]) {
		if let Ok(mut t) = self.0.text.lock() {
			*t = text.to_vec();
		}
	}

	/// config_set_uint32's write: a bool gets value != 0, a FLT the value, anything but a string the uint32
	fn set_from_uint32(&self, value: u32) {
		match self.0.var_type {
			VarType::Bool => self.store_bits((value != 0) as u64),
			VarType::Float => self.store_bits((value as f64).to_bits()),
			VarType::Int => self.store_bits(value as u64),
			VarType::Str => {}
		}
	}

	/// config_set_float's write: a bool gets value != 0, an int (int32_t)value, anything but a string the FLT
	fn set_from_float(&self, value: f64) {
		match self.0.var_type {
			VarType::Bool => self.store_bits((value != 0.0) as u64),
			VarType::Int => self.store_bits(cnum::int32_from_double(value) as u32 as u64),
			VarType::Float => self.store_bits(value.to_bits()),
			VarType::Str => {}
		}
	}
}

/// One config entry: config_entry (survive_config.h)
#[derive(Clone, Debug)]
pub struct ConfigEntry {
	tag: Vec<u8>,
	kind: EntryType,
	/// The C's union { uint32_t i; FLT f; } as its 8 bytes: config_set_uint32 writes only the low 4, config_set_float
	/// all 8, and config_read_uint32 reads the low 4 whatever the type. A new entry's union is uninitialized memory in
	/// the C (malloc'd or realloc'd, so indeterminate); here it starts at zero
	numeric: u64,
	/// `char *data`: a string's bytes (without the NUL) or a float array's doubles (native byte order). A set of
	/// another type leaves it as it was, as the C does, and survive_configs returns it as a C string whatever it holds
	data: Option<Vec<u8>>,
	/// the float array's length; like data, left alone by a set of another type
	elements: u32,
	/// update_list: the variables bound to this entry
	bindings: Vec<Binding>,
}

impl ConfigEntry {
	fn created(tag: &[u8], kind: EntryType) -> ConfigEntry {
		ConfigEntry { tag: tag.to_vec(), kind, numeric: 0, data: None, elements: 0, bindings: Vec::new() }
	}

	pub fn tag(&self) -> &[u8] {
		&self.tag
	}

	pub fn kind(&self) -> EntryType {
		self.kind
	}

	/// numeric.f
	pub fn float(&self) -> f64 {
		f64::from_bits(self.numeric)
	}

	/// numeric.i
	pub fn uint32(&self) -> u32 {
		self.numeric as u32
	}

	/// The union's 8 bytes
	pub fn numeric_bits(&self) -> u64 {
		self.numeric
	}

	/// `data`, whatever it holds (None for NULL)
	pub fn data(&self) -> Option<&[u8]> {
		self.data.as_deref()
	}

	/// `elements`
	pub fn elements(&self) -> u32 {
		self.elements
	}

	/// The float array (`elements` doubles of `data`)
	pub fn floats(&self) -> Vec<f64> {
		(0..self.elements as usize).map(|i| self.data_double(i)).collect()
	}

	/// The number of variables bound to the entry
	pub fn bound(&self) -> usize {
		self.bindings.len()
	}

	/// ((FLT *)data)[i]. Past the bytes `data` holds (a string's, after a set turned an array into a string) the C
	/// reads past its buffer; here the NUL a C string ends with and zeros after it
	fn data_double(&self, i: usize) -> f64 {
		let mut b = [0u8; 8];
		if let Some(d) = &self.data {
			for (k, byte) in b.iter_mut().enumerate() {
				*byte = d.get(i * 8 + k).copied().unwrap_or(0);
			}
		}
		f64::from_ne_bytes(b)
	}

	/// `data` as a C string (what survive_configs returns, or printf's "%s" of it): up to its first NUL. A float array
	/// with no zero byte has no end in the C (it reads on past the buffer); here it ends with the array
	pub(crate) fn data_c_str(&self) -> Option<Vec<u8>> {
		self.data.as_deref().map(|d| c_str(d).to_vec())
	}

	/// config_entry_as_FLT
	fn as_flt(&self) -> f64 {
		match self.kind {
			EntryType::Float => self.float(),
			EntryType::Uint32 => self.uint32() as f64,
			EntryType::String => cnum::atof(self.data.as_deref().unwrap_or(b"")),
			EntryType::FloatArray => 0.0,
		}
	}

	/// config_entry_as_uint32_t
	fn as_uint32(&self) -> u32 {
		match self.kind {
			EntryType::Float => cnum::uint32_from_roundf(self.float()),
			EntryType::Uint32 => self.uint32(),
			EntryType::String => cnum::strtol(self.data.as_deref().unwrap_or(b""), 0).0 as u32,
			EntryType::FloatArray => 0,
		}
	}
}

/// A config group: config_group (survive_config.h), its entries in the order they were first set (the order
/// config_save writes them in). The C's write_lock is not needed: changing a group takes `&mut`
#[derive(Clone, Debug, Default)]
pub struct ConfigGroup {
	entries: Vec<ConfigEntry>,
}

impl ConfigGroup {
	pub fn entries(&self) -> &[ConfigEntry] {
		&self.entries
	}

	/// find_config_entry
	pub fn find(&self, tag: impl AsRef<[u8]>) -> Option<&ConfigEntry> {
		self.position(c_str(tag.as_ref())).map(|i| &self.entries[i])
	}

	fn position(&self, tag: &[u8]) -> Option<usize> {
		self.entries.iter().position(|e| e.tag == tag)
	}

	/// find_config_entry, or next_unused_entry for a new one (appended)
	fn entry_for(&mut self, tag: &[u8], kind: EntryType) -> &mut ConfigEntry {
		let i = match self.position(tag) {
			Some(i) => i,
			None => {
				self.entries.push(ConfigEntry::created(tag, kind));
				self.entries.len() - 1
			}
		};
		&mut self.entries[i]
	}

	/// config_set_float: the bound variables get the value
	pub fn set_float(&mut self, tag: impl AsRef<[u8]>, value: f64) -> f64 {
		let e = self.entry_for(c_str(tag.as_ref()), EntryType::Float);
		e.numeric = value.to_bits();
		e.kind = EntryType::Float;
		for b in &e.bindings {
			b.set_from_float(value);
		}
		value
	}

	/// config_set_uint32: the bound variables get the value
	pub fn set_uint32(&mut self, tag: impl AsRef<[u8]>, value: u32) -> u32 {
		let e = self.entry_for(c_str(tag.as_ref()), EntryType::Uint32);
		e.numeric = (e.numeric & !0xffff_ffff) | value as u64;
		e.kind = EntryType::Uint32;
		for b in &e.bindings {
			b.set_from_uint32(value);
		}
		value
	}

	/// config_set_str: NULL is stored as "". Bound variables are not written (sally: a string attachment is a buffer of
	/// unknown size the C used to overwrite with a pointer)
	pub fn set_str(&mut self, tag: impl AsRef<[u8]>, value: Option<&[u8]>) {
		let e = self.entry_for(c_str(tag.as_ref()), EntryType::String);
		e.data = Some(c_str(value.unwrap_or(b"")).to_vec());
		e.kind = EntryType::String;
	}

	/// config_set_float_a: the doubles are copied in, `elements` set to their count (the C's count is a uint8_t: every
	/// caller passes at most 255)
	pub fn set_float_array(&mut self, tag: impl AsRef<[u8]>, values: &[f64]) {
		let e = self.entry_for(c_str(tag.as_ref()), EntryType::FloatArray);
		e.data = Some(values.iter().flat_map(|v| v.to_ne_bytes()).collect());
		e.kind = EntryType::FloatArray;
		e.elements = values.len() as u32;
	}

	/// config_read_uint32: numeric.i of the entry, whatever its type; an unset tag is set to `def`
	pub fn read_uint32(&mut self, tag: impl AsRef<[u8]>, def: u32) -> u32 {
		let tag = c_str(tag.as_ref());
		match self.position(tag) {
			Some(i) => self.entries[i].uint32(),
			None => self.set_uint32(tag, def),
		}
	}

	/// config_read_float_array: the entry's first min(values.len(), elements) doubles into `values`, returning
	/// `elements` (a uint16_t in the C). An unset tag is set to `def` (its first values.len() doubles), copied into
	/// `values`; with no default nothing is set and 0 returned
	pub fn read_float_array(&mut self, tag: impl AsRef<[u8]>, values: &mut [f64], def: Option<&[f64]>) -> u16 {
		let tag = c_str(tag.as_ref());
		if let Some(i) = self.position(tag) {
			let e = &self.entries[i];
			for (k, v) in values.iter_mut().enumerate().take(e.elements as usize) {
				*v = e.data_double(k);
			}
			return e.elements as u16;
		}
		let Some(def) = def else {
			return 0;
		};
		let def = &def[..values.len()];
		self.set_float_array(tag, def);
		values.copy_from_slice(def);
		values.len() as u16
	}
}

/// Which group an entry sc_search found is in
#[derive(Clone, Copy, Debug)]
pub(super) enum Found {
	Temporary(usize),
	Global(usize),
}

/// The config store of one runtime: survive_config.c's statics and the SurviveContext fields it works on. The
/// registry (the known options, in registration order), the global group (the config file's top level, written back by
/// config_save), the temporary group (the command line and the environment, never saved, looked up first), and one
/// group per lighthouse (the file's "lighthouseN" objects)
#[derive(Debug, Default)]
pub struct Config {
	options: Vec<ConfigOption>,
	option_index: HashMap<Vec<u8>, usize>,
	/// global_config_values
	pub global: ConfigGroup,
	/// temporary_config_values: "Set per-session, from command-line. Not saved but override global_config_values"
	pub temporary: ConfigGroup,
	/// lh_config
	pub lighthouses: [ConfigGroup; NUM_GEN2_LIGHTHOUSES],
	/// the config reader's static array_data being allocated: an array value has been read, by any config_read
	pub(super) array_value_read: bool,
	/// config_save's static `said`
	pub(super) save_readonly_said: bool,
	/// config_save's static `warnedOnce`
	pub(super) save_open_warned: bool,
}

impl Config {
	/// survive_config_bind_variable: registers an option, kept in registration order. Registering a name again keeps
	/// its place and description and takes the new default, as the C's release build does (its assert is compiled out);
	/// a different type is the C's fatal "Types disagree" (it exits), a panic here
	pub fn register(&mut self, name: &str, description: &str, default: OptionDefault) {
		let type_char = default.type_char();
		let (bits, text) = match default {
			OptionDefault::Int(i) => (i as u32 as u64, None),
			OptionDefault::Float(f) => (f.to_bits(), None),
			OptionDefault::Bool(b) => (b as u64, None),
			OptionDefault::Str(s) => {
				let s = s.map(|s| c_str(&s).to_vec());
				(if s.is_some() { STRING_DEFAULT_ADDRESS } else { 0 }, s)
			}
		};
		let name = c_str(name.as_bytes()).to_vec();
		if let Some(&i) = self.option_index.get(&name) {
			let o = &mut self.options[i];
			assert!(
				o.type_char == type_char,
				"Fatal: Internal error on variable {}.  Types disagree [{}/{}].",
				String::from_utf8_lossy(&name),
				o.type_char as char,
				type_char as char
			);
			o.bits = bits;
			o.text = text;
			return;
		}
		self.option_index.insert(name.clone(), self.options.len());
		self.options.push(ConfigOption {
			name,
			description: c_str(description.as_bytes()).to_vec(),
			type_char,
			bits,
			text,
		});
	}

	/// The registered options in registration order (the C's static_conf_t list, head to tail)
	pub fn options(&self) -> &[ConfigOption] {
		&self.options
	}

	/// An option's name as it is now. The tracker's options were once `core2-*` (the name the tracker was built under):
	/// such a name is read as the `track-*` option it has become, so a command line, a settings file or a bench job
	/// written with one still says what it said. Any other name comes back as it is
	pub fn current_name<'a>(&self, tag: &'a [u8]) -> Cow<'a, [u8]> {
		if let Some(rest) = tag.strip_prefix(b"core2-") {
			let now = [b"track-", rest].concat();
			if self.option_index.contains_key(&now) {
				return Cow::Owned(now);
			}
		}
		Cow::Borrowed(tag)
	}

	/// find_static_conf_t
	pub fn option(&self, tag: impl AsRef<[u8]>) -> Option<&ConfigOption> {
		self.option_index.get(c_str(tag.as_ref())).map(|&i| &self.options[i])
	}

	/// sc_search: the temporary group, then the global one
	pub(super) fn locate(&self, tag: &[u8]) -> Option<Found> {
		match self.temporary.position(tag) {
			Some(i) => Some(Found::Temporary(i)),
			None => self.global.position(tag).map(Found::Global),
		}
	}

	pub(super) fn found(&self, at: Found) -> &ConfigEntry {
		match at {
			Found::Temporary(i) => &self.temporary.entries[i],
			Found::Global(i) => &self.global.entries[i],
		}
	}

	fn found_mut(&mut self, at: Found) -> &mut ConfigEntry {
		match at {
			Found::Temporary(i) => &mut self.temporary.entries[i],
			Found::Global(i) => &mut self.global.entries[i],
		}
	}

	/// sc_search
	pub fn search(&self, tag: impl AsRef<[u8]>) -> Option<&ConfigEntry> {
		self.locate(c_str(tag.as_ref())).map(|at| self.found(at))
	}

	/// survive_configf. Read as a double: a float entry as is, an int entry converted, a string entry by atof, an array
	/// as 0; a registered default is data_default.f, the bits of whatever was registered
	pub fn configf(&mut self, tag: impl AsRef<[u8]>, flags: u8, def: f64) -> f64 {
		let tag = c_str(tag.as_ref());
		if flags & SC_OVERRIDE == 0 {
			if let Some(e) = self.search(tag) {
				return e.as_flt();
			}
		}
		let mut def = def;
		if flags & SC_OVERRIDE == 0 {
			if let Some(o) = self.option(tag) {
				def = o.float_bits();
			}
		}
		// "If override is flagged, or, we can't find the variable, ,continue on."
		if flags & SC_SETCONFIG != 0 {
			self.temporary.set_float(tag, def);
			self.global.set_float(tag, def);
		} else if flags & SC_SET != 0 {
			self.temporary.set_float(tag, def);
		}
		def
	}

	/// survive_configi. Read as a uint32: a float entry by (uint32_t)roundf((float)f), an int entry as is, a string
	/// entry by strtol(s, 0, 0), an array as 0; a registered default is data_default.i
	pub fn configi(&mut self, tag: impl AsRef<[u8]>, flags: u8, def: u32) -> u32 {
		let tag = c_str(tag.as_ref());
		if flags & SC_OVERRIDE == 0 {
			if let Some(e) = self.search(tag) {
				return e.as_uint32();
			}
		}
		let mut def = def;
		if flags & SC_OVERRIDE == 0 {
			if let Some(o) = self.option(tag) {
				def = o.int_bits();
			}
		}
		if flags & SC_SETCONFIG != 0 {
			self.temporary.set_uint32(tag, def);
			self.global.set_uint32(tag, def);
		} else if flags & SC_SET != 0 {
			self.temporary.set_uint32(tag, def);
		}
		def
	}

	/// survive_configb: survive_configi's reading, nonzero is true (a registered default is data_default.i too)
	pub fn configb(&mut self, tag: impl AsRef<[u8]>, flags: u8, def: bool) -> bool {
		let tag = c_str(tag.as_ref());
		if flags & SC_OVERRIDE == 0 {
			if let Some(e) = self.search(tag) {
				return e.as_uint32() != 0;
			}
		}
		let mut def = def;
		if flags & SC_OVERRIDE == 0 {
			if let Some(o) = self.option(tag) {
				def = o.int_bits() != 0;
			}
		}
		if flags & SC_SETCONFIG != 0 {
			self.temporary.set_uint32(tag, def as u32);
			self.global.set_uint32(tag, def as u32);
		} else if flags & SC_SET != 0 {
			self.temporary.set_uint32(tag, def as u32);
		}
		def
	}

	/// survive_configs. A found entry gives its `data` as a C string: a string's text, but None (NULL) for a number
	/// that was never a string, and the leftover text or array bytes of one that was. Not found: an unregistered or 's'
	/// option is set (SC_SET, SC_SETCONFIG) to the default, which is returned, and with neither flag the registered
	/// default (or `def`) is returned; an 'i', 'b' or 'f' option is handed to survive_configi / configb / configf with
	/// atoi(def) / atoi(def) / atof(def), and `def` returned.
	///
	/// Without SC_OVERRIDE the C first makes `def` the registered default read as a string pointer: for an 'i', 'b' or
	/// 'f' option the number's bits, which is NULL when they are zero and points nowhere otherwise (atoi then crashes,
	/// and atof(NULL) always does). The port takes NULL in every one of those cases: the number passed on does not
	/// matter there (without SC_OVERRIDE survive_configi and the others replace it by the registered default), and
	/// None is returned. With SC_OVERRIDE, an 'f' option with a NULL `def` crashes the C too; the port reads it as 0
	pub fn configs(&mut self, tag: impl AsRef<[u8]>, flags: u8, def: Option<&[u8]>) -> Option<Vec<u8>> {
		let tag = c_str(tag.as_ref());
		if flags & SC_OVERRIDE == 0 {
			if let Some(e) = self.search(tag) {
				return e.data_c_str();
			}
		}
		let mut def = def.map(|d| c_str(d).to_vec());
		let mut founddata = def.clone();
		let mut foundtype = 0;
		if let Some(o) = self.option(tag) {
			founddata = if o.type_char == b's' { o.text.clone() } else { None };
			foundtype = o.type_char;
			if flags & SC_OVERRIDE == 0 {
				def = founddata.clone();
			}
		}
		match foundtype {
			0 | b's' => {
				if flags & SC_SETCONFIG != 0 {
					self.temporary.set_str(tag, def.as_deref());
					self.global.set_str(tag, def.as_deref());
				} else if flags & SC_SET != 0 {
					self.temporary.set_str(tag, def.as_deref());
				} else {
					return founddata;
				}
			}
			b'i' => {
				let v = cnum::atoi(def.as_deref().unwrap_or(b"0"));
				self.configi(tag, flags, v as u32);
			}
			b'b' => {
				let v = cnum::atoi(def.as_deref().unwrap_or(b"0"));
				self.configb(tag, flags, v != 0);
			}
			_ => {
				let v = cnum::atof(def.as_deref().unwrap_or(b"0"));
				self.configf(tag, flags, v);
			}
		}
		def
	}

	/// survive_config_is_set: the tag is in the temporary or the global group
	pub fn is_set(&self, tag: impl AsRef<[u8]>) -> bool {
		self.search(tag).is_some()
	}

	/// survive_config_type: the registered type ('i', 'f', 's', 'b'; sally: registration wins, it knows a bool from an
	/// int), else the found entry's ('f', 'i' or 's'), else None (an unknown tag, or an array)
	pub fn value_type(&self, tag: impl AsRef<[u8]>) -> Option<u8> {
		let tag = c_str(tag.as_ref());
		if let Some(o) = self.option(tag) {
			return Some(o.type_char);
		}
		match self.search(tag)?.kind {
			EntryType::Float => Some(b'f'),
			EntryType::Uint32 => Some(b'i'),
			EntryType::String => Some(b's'),
			EntryType::FloatArray => None,
		}
	}

	/// survive_configb(.., SC_GET, ..), which changes nothing
	fn get_bool(&self, tag: &[u8], def: bool) -> bool {
		match (self.search(tag), self.option(tag)) {
			(Some(e), _) => e.as_uint32() != 0,
			(None, Some(o)) => o.int_bits() != 0,
			(None, None) => def,
		}
	}

	/// survive_configf(.., SC_GET, ..)
	fn get_float(&self, tag: &[u8], def: f64) -> f64 {
		match (self.search(tag), self.option(tag)) {
			(Some(e), _) => e.as_flt(),
			(None, Some(o)) => o.float_bits(),
			(None, None) => def,
		}
	}

	/// survive_configi(.., SC_GET, ..)
	fn get_uint32(&self, tag: &[u8], def: u32) -> u32 {
		match (self.search(tag), self.option(tag)) {
			(Some(e), _) => e.as_uint32(),
			(None, Some(o)) => o.int_bits(),
			(None, None) => def,
		}
	}

	/// survive_configs(.., SC_GET, def), which changes nothing: the found entry's data, else the registered 's' default
	/// (None for another type, as survive_configs), else `def`
	pub(super) fn get_str(&self, tag: &[u8], def: Option<&[u8]>) -> Option<Vec<u8>> {
		match (self.search(tag), self.option(tag)) {
			(Some(e), _) => e.data_c_str(),
			(None, Some(o)) => o.text.clone().filter(|_| o.type_char == b's'),
			(None, None) => def.map(|d| c_str(d).to_vec()),
		}
	}

	/// survive_config_as_str: the value as text, as snprintf'd into a buffer of `n` bytes (so cut to n - 1; nothing for
	/// n = 0, where the C leaves the buffer alone). A registered option prints its own type's lookup: "%d" for a bool,
	/// "%i" for an int, "%s" for a string ("(null)" for NULL), "%f" for a float, or "%e" when it is not 0 and within
	/// 0.001 of it (a NaN included). An unregistered tag prints its entry: "%f" of the float cut to a float, "%i",
	/// "%s", or an array's bytes as a string; an unset one `def` (strncpy'd: with n bytes or more it has no NUL in the
	/// C, here it stops at n), or nothing for a NULL `def`
	pub fn as_str(&self, tag: impl AsRef<[u8]>, def: Option<&[u8]>, n: usize) -> Vec<u8> {
		let tag = c_str(tag.as_ref());
		if n == 0 {
			return Vec::new();
		}
		let mut out = Vec::new();
		if let Some(o) = self.option(tag) {
			match o.type_char {
				b'b' => out.extend_from_slice(if self.get_bool(tag, false) { b"1" } else { b"0" }),
				b'f' => {
					let df = self.get_float(tag, 0.0);
					if df.abs() > 0.001 || df == 0.0 {
						cnum::fixed(&mut out, df, 6);
					} else {
						cnum::exp(&mut out, df, 6);
					}
				}
				b'i' => out.extend_from_slice((self.get_uint32(tag, 0) as i32).to_string().as_bytes()),
				_ => out.extend_from_slice(self.get_str(tag, None).as_deref().unwrap_or(b"(null)")),
			}
			out.truncate(n - 1);
			return out;
		}
		let Some(e) = self.search(tag) else {
			if let Some(d) = def {
				let d = c_str(d);
				out.extend_from_slice(&d[..d.len().min(n)]);
			}
			return out;
		};
		match e.kind {
			EntryType::Float => cnum::fixed(&mut out, cnum::to_double(cnum::to_float(e.float())), 6),
			EntryType::Uint32 => out.extend_from_slice((e.uint32() as i32).to_string().as_bytes()),
			EntryType::String | EntryType::FloatArray => out.extend_from_slice(&e.data_c_str().unwrap_or_default()),
		}
		out.truncate(n - 1);
		out
	}

	/// survive_default_to_str: a registered default as survive_config_iterate hands it out, "%15d    " (an int or a
	/// bool), "%15f    " or "%15e    " (a float, as survive_config_as_str chooses), "%15s    " (a string, "(null)" for
	/// NULL), cut to 126 bytes
	pub fn default_text(option: &ConfigOption) -> Vec<u8> {
		let mut value = Vec::new();
		match option.type_char {
			b'b' | b'i' => value.extend_from_slice((option.int_bits() as i32).to_string().as_bytes()),
			b'f' => {
				let df = option.float_bits();
				if df.abs() > 0.001 || df == 0.0 {
					cnum::fixed(&mut value, df, 6);
				} else {
					cnum::exp(&mut value, df, 6);
				}
			}
			_ => value.extend_from_slice(option.text.as_deref().unwrap_or(b"(null)")),
		}
		let mut out = Vec::new();
		cnum::pad(&mut out, &value, 15);
		out.extend_from_slice(b"    ");
		out.truncate(126);
		out
	}

	/// survive_config_iterate: every registered option in registration order, with its default as text
	/// (survive_default_to_str). A recording's OPTION lines are written from it: "OPTION %s %c %s\n" with the name, the
	/// type char and survive_config_as_str(name, "", 128)
	pub fn iterate(&self) -> impl Iterator<Item = (&ConfigOption, Vec<u8>)> {
		self.options.iter().map(|o| (o, Config::default_text(o)))
	}

	/// survive_config_bound_count (sally): the number of variables bound to the entry sc_search finds (a set reaches
	/// them at once; an unbound tag takes effect only where the code reads it again)
	pub fn bound_count(&self, tag: impl AsRef<[u8]>) -> i32 {
		self.search(tag).map_or(0, |e| e.bindings.len() as i32)
	}

	/// survive_config_set_live (sally): sets a value while running, from text, into the group that already holds the
	/// tag (the temporary group shadows the global one in lookups, so a fresh temporary entry would hide the variables
	/// bound to a global one), else the temporary group. A float is read by strtod, an int or a bool by strtol(s, 0, 0)
	/// (a bool also takes "true" / "on" / "false" / "off"), the whole text or it is refused. Returns the number of
	/// bound variables, -1 for an unknown tag, -2 for a bad value. Never writes the config file
	pub fn set_live(&mut self, tag: impl AsRef<[u8]>, value: impl AsRef<[u8]>) -> i32 {
		let tag = c_str(tag.as_ref());
		let value = c_str(value.as_ref());
		let Some(type_char) = self.value_type(tag) else {
			return -1;
		};
		let global = self.temporary.position(tag).is_none() && self.global.position(tag).is_some();
		let group = if global { &mut self.global } else { &mut self.temporary };
		match type_char {
			b'f' => {
				let (v, end) = cnum::strtod(value);
				if end == 0 || end != value.len() {
					return -2;
				}
				group.set_float(tag, v);
			}
			b'i' | b'b' => {
				let v = if type_char == b'b' && (value == b"true" || value == b"on") {
					1
				} else if type_char == b'b' && (value == b"false" || value == b"off") {
					0
				} else {
					let (v, end) = cnum::strtol(value, 0);
					if end == 0 || end != value.len() {
						return -2;
					}
					v
				};
				group.set_uint32(tag, v as u32);
			}
			b's' => group.set_str(tag, Some(value)),
			_ => return -1,
		}
		self.bound_count(tag)
	}

	/// survive_attach_config: binds a variable to the entry sc_search finds, creating it first if the tag is unset (in
	/// the temporary group, with SC_SET: the registered default, or 0). The variable then holds the entry's value read
	/// as its type, and every later config_set_uint32 / config_set_float of that entry (not of another group's entry
	/// for the same tag) writes into it. An 's' variable is copied once, here. Attaching a bound variable again changes
	/// nothing, its value included. (The C logs the value at verbosity 110; logging is the caller's.)
	pub fn attach(&mut self, tag: impl AsRef<[u8]>, var: &Binding) {
		let tag = c_str(tag.as_ref());
		if self.locate(tag).is_none() {
			// "Check to see if there is a static config with this value."
			match var.var_type() {
				VarType::Int => var.store_bits(self.configi(tag, SC_SET, 0) as u64),
				VarType::Bool => var.store_bits(self.configb(tag, SC_SET, false) as u64),
				VarType::Float => var.store_bits(self.configf(tag, SC_SET, 0.0).to_bits()),
				// the C copies strlen(value) bytes, and crashes on a NULL value
				VarType::Str => var.store_text(&self.configs(tag, SC_SET, None).unwrap_or_default()),
			}
		}
		// SC_SET above always leaves an entry ("Configuration item %s not initialized." cannot happen)
		let Some(at) = self.locate(tag) else {
			return;
		};
		let entry = self.found_mut(at);
		if entry.bindings.iter().any(|b| b.same(var)) {
			return;
		}
		entry.bindings.push(var.clone());
		match var.var_type() {
			VarType::Bool => var.store_bits(self.configb(tag, SC_GET, false) as u64),
			VarType::Int => var.store_bits(self.configi(tag, SC_GET, 0) as u64),
			VarType::Float => var.store_bits(self.configf(tag, SC_GET, 0.0).to_bits()),
			// SC_SET, as the C; the entry exists, so it only reads (a number's NULL data crashes the C's strcpy)
			VarType::Str => var.store_text(&self.configs(tag, SC_SET, None).unwrap_or_default()),
		}
	}

	/// survive_detach_config: unbinds the variable from the entry sc_search finds (which may not be the one it was
	/// attached to, if the tag has been set in the temporary group since: the variable then stays bound, as in the C)
	pub fn detach(&mut self, tag: impl AsRef<[u8]>, var: &Binding) -> Result<(), DetachError> {
		let tag = c_str(tag.as_ref());
		let Some(at) = self.locate(tag) else {
			return Err(DetachError::NotInitialized(tag.to_vec()));
		};
		let entry = self.found_mut(at);
		let before = entry.bindings.len();
		entry.bindings.retain(|b| !b.same(var));
		if entry.bindings.len() == before {
			return Err(DetachError::NotAttached(tag.to_vec()));
		}
		Ok(())
	}

	/// survive_config_entry_to_str: an entry's value as survive_print_known_configs prints it, cut to 126 bytes
	fn entry_text(e: &ConfigEntry) -> Vec<u8> {
		let mut out = Vec::new();
		let mut value = Vec::new();
		match e.kind {
			EntryType::Uint32 => value.extend_from_slice((e.uint32() as i32).to_string().as_bytes()),
			EntryType::Float => {
				let df = e.float();
				if df.abs() > 0.001 || df == 0.0 {
					cnum::fixed(&mut value, df, 6);
				} else {
					cnum::exp(&mut value, df, 6);
				}
			}
			EntryType::String => value.extend_from_slice(&e.data_c_str().unwrap_or_default()),
			EntryType::FloatArray => {
				out.extend_from_slice(b"[FA] ");
				cnum::pad(&mut out, &e.tag, 25);
				out.extend_from_slice(b"  \n");
				out.truncate(126);
				return out;
			}
		}
		cnum::pad(&mut out, &value, 15);
		out.extend_from_slice(b"    ");
		out.truncate(126);
		out
	}

	/// survive_print_known_configs: what it prints to stdout, every set tag of the temporary then the global group,
	/// then every registered option not printed yet. Verbose, one line each (" --%-40s%s %-12s     %s\n": the name, the
	/// value or default as text, the type, the description); otherwise "--name " each, and "--no-name " after an int or
	/// bool option whose default is 1
	pub fn known_configs_text(&self, verbose: bool) -> Vec<u8> {
		fn line(out: &mut Vec<u8>, name: &[u8], value: &[u8], type_text: &[u8], description: &[u8]) {
			out.extend_from_slice(b" --");
			out.extend_from_slice(name);
			out.resize(out.len() + 40usize.saturating_sub(name.len()), b' ');
			out.extend_from_slice(value);
			out.push(b' ');
			out.extend_from_slice(type_text);
			out.resize(out.len() + 12usize.saturating_sub(type_text.len()), b' ');
			out.extend_from_slice(b"     ");
			out.extend_from_slice(description);
			out.push(b'\n');
		}
		let mut out = Vec::new();
		let mut printed: Vec<&[u8]> = Vec::new();
		for group in [&self.temporary, &self.global] {
			for e in &group.entries {
				if printed.contains(&e.tag.as_slice()) {
					continue;
				}
				if verbose {
					let type_text: &[u8] = match e.kind {
						EntryType::Float => b":float",
						EntryType::Uint32 => b":int",
						EntryType::String => b":string",
						EntryType::FloatArray => b".",
					};
					let description = self.option(&e.tag).map_or(&b""[..], |o| &o.description);
					line(&mut out, &e.tag, &Config::entry_text(e), type_text, description);
				} else {
					out.extend_from_slice(b"--");
					out.extend_from_slice(&e.tag);
					out.push(b' ');
				}
				printed.push(&e.tag);
			}
		}
		for o in &self.options {
			if printed.contains(&o.name.as_slice()) {
				continue;
			}
			if verbose {
				let type_text: &[u8] = match o.type_char {
					b'f' => b":float",
					b'i' => b":int",
					b's' => b":string",
					_ => b":bool",
				};
				line(&mut out, &o.name, &Config::default_text(o), type_text, &o.description);
			} else {
				out.extend_from_slice(b"--");
				out.extend_from_slice(&o.name);
				out.push(b' ');
				// "Also display '--no-*' formatted options. We only do this for 'i' types since that is how we
				// represent flags internally; and it'd be sorta useless to have them show up for default-off options."
				if matches!(o.type_char, b'i' | b'b') && o.int_bits() == 1 {
					out.extend_from_slice(b"--no-");
					out.extend_from_slice(&o.name);
					out.push(b' ');
				}
			}
		}
		out
	}

	/// survive_print_help_for_parameter: what it prints to stderr for a registered option (None if not registered): a
	/// line drawn over the previous one, "    %s: %s \t\tdefault: %s\t\t(%c)" (cut to 158 bytes) between the terminal's
	/// save and restore cursor codes
	pub fn help_for_parameter(&self, tag: impl AsRef<[u8]>) -> Option<Vec<u8>> {
		let o = self.option(tag)?;
		let value = self.as_str(&o.name, Some(b""), 128);
		let mut help = Vec::new();
		help.extend_from_slice(b"    ");
		help.extend_from_slice(&o.name);
		help.extend_from_slice(b": ");
		help.extend_from_slice(&o.description);
		help.extend_from_slice(b" \t\tdefault: ");
		help.extend_from_slice(&value);
		help.extend_from_slice(b"\t\t(");
		help.push(o.type_char);
		help.push(b')');
		help.truncate(158);
		let mut out = b"\x1b7\x1b[1A\x1b[1000D\x1b[K".to_vec();
		out.extend_from_slice(&help);
		out.extend_from_slice(b"\x1b8");
		Some(out)
	}
}

/// Why survive_detach_config did not detach: the C logs these (an error and a warning) and carries on
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DetachError {
	/// "Configuration item %s not initialized.\n" (SV_GENERAL_ERROR): the tag is not set
	NotInitialized(Vec<u8>),
	/// "Found no config item to detach %s" (SV_WARN): the variable is not bound to the entry found
	NotAttached(Vec<u8>),
}

impl std::fmt::Display for DetachError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			DetachError::NotInitialized(tag) => {
				writeln!(f, "Configuration item {} not initialized.", String::from_utf8_lossy(tag))
			}
			DetachError::NotAttached(tag) => {
				write!(f, "Found no config item to detach {}", String::from_utf8_lossy(tag))
			}
		}
	}
}

impl std::error::Error for DetachError {}
