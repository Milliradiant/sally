//! A device's configuration: what survive_default_devices.c's survive_load_htc_config_format sets on a device from
//! the JSON config Valve's devices carry (its sensors and channel map, its IMU's calibration, the IMU's and the head's
//! poses, its class, serial number and model), starting from the defaults survive_create_device gives a device, with
//! everything the C derives once the JSON is read. Ported operation for operation (docs/checking.md), with its tokenizer
//! (`jsmn`), its numbers read as glibc reads them (sally_math's c_strtod / c_strtof / c_atoi) and its log lines in the
//! C's text.
//!
//! The C's quirks are kept, each explained where it happens: the arrays read by parse_float_array* go through strtof
//! (json_helpers.c tests USE_DOUBLE, which this build never defines), so the IMU's calibration and the poses are
//! single precision, while modelPoints and modelNormals (atof) are double; a string value matches device_class,
//! model_number or model_name when it is a prefix of the C's string (strncmp over the value's length), the empty one
//! included; keys match in any object, at any depth, and "plus_x", "plus_z" and "position" under any key "imu" or
//! "head"; an array's count is a uint8_t (taken modulo 256); an IMU array longer than three runs on into the next
//! field (acc_bias, acc_scale, gyro_bias, gyro_scale lie in that order in the C's struct); has_sensor_locations
//! tests the norm of `sensor_locations[i]`, not `[i * 3]`; "VIVE_Pro 2 MV" sets SURVIVE_OBJECT_TYPE_HMD as the
//! subtype.
//!
//! Where the C's behaviour is undefined on a config (it would read past its tokens or an array, write past an array,
//! dereference a pointer it freed or never set, or overflow SV_WARN's buffer), the port refuses the config
//! (`DeviceConfigError::Undefined`), saying what the C would have done, instead of guessing at what it would have read.

use sally_json::{jsmn_parse, Token, TokenType};
use sally_math::printf::{sprintf, Arg};
use sally_math::{
	apply_pose_to_point, apply_pose_to_pose, c_atof, c_atoi, c_strtof, cross3d, invert_pose, norm3d, quatfrommatrix33,
	quatrotatevector, scale3d, Pose, LINMATHPI,
};

/// SurviveObjectType (survive_types.h)
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObjectType {
	Unknown = 0,
	Hmd = 1,
	Controller = 2,
	Other = 3,
}

/// SurviveObjectSubtype (survive_types.h)
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObjectSubtype {
	Generic = 0,
	ViveHmd = 1,
	IndexHmd = 2,
	Wand = 3,
	KnucklesR = 4,
	KnucklesL = 5,
	Tracker = 6,
	TrackerGen2 = 7,
}

/// The settings the C reads while it loads a config
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ConfigOptions {
	/// imu-acc-bias-units (survive_configi): 1 reads acc_bias in m/s^2, any other value as the upstream guess
	pub imu_acc_bias_units: u32,
	/// ignore-config-imu-bias (survive_configb)
	pub ignore_config_imu_bias: bool,
	/// Whether survive_colorize colorizes the codename in the verbose lines: the C does, except on Windows and when the
	/// runtime logs to a file (--log) or records to stdout
	pub colorize: bool,
}

/// A line the C logs while it loads a config, as its log hook gets it
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Log {
	/// SV_WARN: the log hook at SURVIVE_LOG_LEVEL_WARNING
	Warn(Vec<u8>),
	/// SV_INFO: the log hook at SURVIVE_LOG_LEVEL_INFO
	Info(Vec<u8>),
	/// SV_VERBOSE(level, ...): SV_INFO, when ctx->log_level >= level
	Verbose(i32, Vec<u8>),
	/// SV_GENERAL_ERROR: the report_error hook with SURVIVE_ERROR_GENERAL, then the log hook at SURVIVE_LOG_LEVEL_INFO
	GeneralError(Vec<u8>),
}

/// SURVIVE_ERROR_GENERAL (survive_types.h), what a Log::GeneralError reports
pub const SURVIVE_ERROR_GENERAL: i32 = -1;

/// Why a config was not loaded
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DeviceConfigError {
	/// An empty config: the C returns -1 and logs nothing
	Empty,
	/// jsmn_parse failed (JSMN_ERROR_INVAL -2, JSMN_ERROR_PART -3): the C logs `Failed to parse JSON in HMD
	/// configuration: <error>` and returns -1
	Json(i32),
	/// The first token is not an object: the C logs `Object expected in HMD configuration` and returns -2
	NotAnObject,
	/// The C's behaviour on this config is undefined: what it would do
	Undefined(&'static str),
}

impl DeviceConfigError {
	/// survive_load_htc_config_format's return value; None where the C's behaviour is undefined
	pub fn code(&self) -> Option<i32> {
		match self {
			DeviceConfigError::Empty | DeviceConfigError::Json(_) => Some(-1),
			DeviceConfigError::NotAnObject => Some(-2),
			DeviceConfigError::Undefined(_) => None,
		}
	}

	/// The line the C logs for it
	pub fn log(&self) -> Option<Log> {
		match self {
			DeviceConfigError::Json(r) => Some(sv_info(format!("Failed to parse JSON in HMD configuration: {r}\n"))),
			DeviceConfigError::NotAnObject => Some(sv_info("Object expected in HMD configuration\n")),
			DeviceConfigError::Empty | DeviceConfigError::Undefined(_) => None,
		}
	}
}

impl std::fmt::Display for DeviceConfigError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			DeviceConfigError::Empty => write!(f, "empty device config"),
			DeviceConfigError::Json(r) => write!(f, "device config is not JSON (jsmn error {r})"),
			DeviceConfigError::NotAnObject => write!(f, "device config is not a JSON object"),
			DeviceConfigError::Undefined(what) => write!(f, "device config refused: {what}"),
		}
	}
}

impl std::error::Error for DeviceConfigError {}

/// A device's configuration: every field survive_load_htc_config_format sets on its SurviveObject, from
/// survive_create_device's defaults, in the C's units and frames
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceConfig {
	pub object_type: ObjectType,
	pub object_subtype: ObjectSubtype,
	/// serial_number[16]: device_serial_number's first 16 bytes, zero-padded (with no NUL when it has 16 or more)
	pub serial_number: [u8; 16],
	/// sensor_ct: the points read by whichever of modelPoints and modelNormals came last (up to an error in it)
	pub sensor_ct: usize,
	/// sensor_locations: the C's buffer for 32 sensors (zero past the points read), in the IMU's frame; None: the
	/// config has no modelPoints
	pub sensor_locations: Option<[[f64; 3]; 32]>,
	/// sensor_normals, as sensor_locations (rotated into the IMU's frame); None: no modelNormals
	pub sensor_normals: Option<[[f64; 3]; 32]>,
	/// channel_map: for each of the 32 channels the sensor on it, -1 for none; None: no channelMap
	pub channel_map: Option<[i32; 32]>,
	pub has_sensor_locations: bool,
	pub imu_freq: f64,
	pub head2trackref: Pose,
	pub imu2trackref: Pose,
	pub head2imu: Pose,
	pub raw_acc_scale: f64,
	pub raw_gyro_scale: f64,
	pub acc_bias: [f64; 3],
	pub acc_scale: [f64; 3],
	pub gyro_bias: [f64; 3],
	pub gyro_scale: [f64; 3],
	/// The lines the C logs while it loads the config, in its order
	pub log: Vec<Log>,
}

/// VIVE_DEFAULT_IMU_HZ and HMD_IMU_HZ: the C's float literals
const VIVE_DEFAULT_IMU_HZ: f32 = 250.0;
const HMD_IMU_HZ: f32 = 1000.0;

/// The sensors a device has room for: ParsePoints allocates 32 * 3 doubles, and channel_map 32 ints
const SENSORS: usize = 32;

/// SV_INFO's (and SV_VERBOSE's) buffer: snprintf(stbuff, 1024, ...) keeps 1023 bytes
fn sv_info(text: impl Into<Vec<u8>>) -> Log {
	let mut text = text.into();
	text.truncate(1023);
	Log::Info(text)
}

/// survive_colorize (survive.c): the string between ANSI color codes picked by its hash (survive_hash_str % 8 + 30),
/// unless colorizing is off or the string is longer than its 128-byte buffer leaves room for
fn survive_colorize(s: &[u8], colorize: bool) -> Vec<u8> {
	if !colorize || s.len() > 128 - 18 - 1 {
		return s.to_vec();
	}
	let hash = s.iter().fold(5381u32, |h, &c| (h << 5).wrapping_add(h).wrapping_add(c as u32));
	let mut out = format!("\x1b[0;{}m", hash % 8 + 30).into_bytes();
	out.extend_from_slice(s);
	out.extend_from_slice(b"\x1b[0m");
	out
}

/// model_number_subtypes: a model_number value matches the first key it is a prefix of
struct ModelNumber {
	key: &'static [u8],
	subtype: ObjectSubtype,
	sensor_scale: f64,
}

const fn model(key: &'static [u8], subtype: ObjectSubtype, sensor_scale: f64) -> ModelNumber {
	ModelNumber { key, subtype, sensor_scale }
}

#[rustfmt::skip]
const MODEL_NUMBER_SUBTYPES: [ModelNumber; 21] = [
	model(b"Knuckles Right", ObjectSubtype::KnucklesR, 0.999),
	model(b"Knuckles EV3.0 Right", ObjectSubtype::KnucklesR, 0.999),
	model(b"Knuckles Left", ObjectSubtype::KnucklesL, 0.999),
	model(b"Knuckles EV3.0 Left", ObjectSubtype::KnucklesL, 0.999),
	model(b"Utah MP", ObjectSubtype::IndexHmd, 1.0048),
	model(b"Vive Controller MV", ObjectSubtype::Wand, 1.),
	model(b"Vive. Controller MV", ObjectSubtype::Wand, 1.),
	model(b"VIVE Tracker Pro MV", ObjectSubtype::TrackerGen2, 1.0034),
	model(b"Vive. Tracker MV", ObjectSubtype::Tracker, 1.00585),
	model(b"Vive Tracker MV", ObjectSubtype::Tracker, 1.00585),
	model(b"Vive MV.", ObjectSubtype::ViveHmd, 1.004),
	model(b"Vive MV", ObjectSubtype::ViveHmd, 1.004),
	model(b"Vive. MV", ObjectSubtype::ViveHmd, 1.004),
	model(b"Vive_Pro MV", ObjectSubtype::ViveHmd, 1.0028),
	model(b"VIVE_Pro MV", ObjectSubtype::ViveHmd, 1.0028),
	model(b"VIVE Tracker 3.0 MV", ObjectSubtype::TrackerGen2, 1.0034),
	model(b"REF-HMD", ObjectSubtype::ViveHmd, 1.),
	model(b"Tundra Tracker", ObjectSubtype::TrackerGen2, 1.),
	// the C's table has SURVIVE_OBJECT_TYPE_HMD here, the object type's 1, which as a subtype is VIVE_HMD
	model(b"VIVE_Pro 2 MV", ObjectSubtype::ViveHmd, 1.),
	model(b"VIVE Controller Pro MV", ObjectSubtype::Wand, 1.),
	model(b"Beyond", ObjectSubtype::IndexHmd, 1.),
];

/// The IMU arrays as the C's struct lays them out, acc_bias[3], acc_scale[3], gyro_bias[3], gyro_scale[3] (then the
/// haptic function pointer): the key and the array's offset in that block
const IMU_FIELDS: [(&[u8], usize); 4] = [(b"acc_bias", 0), (b"acc_scale", 3), (b"gyro_bias", 6), (b"gyro_scale", 9)];

/// vive_pose_t: a pose as the config gives it, the frame's origin and its +x and +z axes
#[derive(Clone, Copy, Default)]
struct VivePose {
	position: [f64; 3],
	plus_x: [f64; 3],
	plus_z: [f64; 3],
}

/// The SurviveObject fields the load reads and writes, at survive_create_device's values to start with
struct Object {
	object_type: ObjectType,
	object_subtype: ObjectSubtype,
	serial_number: [u8; 16],
	/// int8_t in the C; never past 32 here (a 33rd point is refused)
	sensor_ct: usize,
	/// SV_CALLOC(sizeof(FLT) * 32 * 3), flat as the C indexes it
	sensor_locations: Option<[f64; 3 * SENSORS]>,
	sensor_normals: Option<[f64; 3 * SENSORS]>,
	channel_map: Option<[i32; SENSORS]>,
	imu_freq: f64,
	head2trackref: Pose,
	imu2trackref: Pose,
	head2imu: Pose,
	raw_acc_scale: f64,
	raw_gyro_scale: f64,
	/// acc_bias, acc_scale, gyro_bias, gyro_scale (IMU_FIELDS)
	imu: [f64; 12],
}

impl Object {
	/// survive_create_device's (and calloc's) values for the fields the load touches
	fn created() -> Object {
		let identity = Pose { pos: [0.0; 3], rot: [1.0, 0.0, 0.0, 0.0] };
		let mut imu = [0.0; 12];
		imu[3..6].copy_from_slice(&[1.0; 3]); // acc_scale
		imu[9..12].copy_from_slice(&[1.0; 3]); // gyro_scale
		Object {
			object_type: ObjectType::Other,
			object_subtype: ObjectSubtype::Generic,
			serial_number: [0; 16],
			sensor_ct: 0,
			sensor_locations: None,
			sensor_normals: None,
			channel_map: None,
			imu_freq: VIVE_DEFAULT_IMU_HZ as f64,
			head2trackref: identity,
			imu2trackref: identity,
			head2imu: Pose::default(),
			raw_acc_scale: 0.0,
			raw_gyro_scale: 0.0,
			imu,
		}
	}
}

/// The JSON and its tokens. Every token read goes through `tok`: the C reads some past its pool on a malformed config
struct Tokens<'a> {
	json: &'a [u8],
	toks: &'a [Token],
}

impl<'a> Tokens<'a> {
	fn tok(&self, i: usize) -> Result<Token, DeviceConfigError> {
		self.toks.get(i).copied().ok_or(DeviceConfigError::Undefined("the C reads past the end of its JSON tokens"))
	}

	/// A token's text: jsmn's tokens lie within the input (an object or array that is never closed is an error), and
	/// none holds a NUL (jsmn stops at one)
	fn text(&self, t: &Token) -> &'a [u8] {
		&self.json[t.start as usize..t.end as usize]
	}

	/// From a token's start to the input's end: what strtof and atoi read, which may run past the token's text
	fn from(&self, t: &Token) -> &'a [u8] {
		&self.json[t.start as usize..]
	}

	/// jsoneq(json, tok, s) == 0: the token is a string whose text is s
	fn jsoneq(&self, t: &Token, s: &[u8]) -> bool {
		t.kind == TokenType::String && self.text(t) == s
	}

	/// parse_float_array_in_place(str, token, out, count): strtof (json_helpers.c tests USE_DOUBLE, never defined in
	/// this build) of each of `count` tokens from `token`, stored as a double, until one is not a number. That one is
	/// stored too (strtof's 0) and it returns 0. `out` is the C's pointer and what follows it in the same struct
	fn parse_float_array_in_place(&self, token: usize, out: &mut [f64], count: u8) -> Result<i32, DeviceConfigError> {
		for i in 0..count as usize {
			let t = self.tok(token + i)?;
			let (f, read) = c_strtof(self.from(&t));
			*out.get_mut(i).ok_or(DeviceConfigError::Undefined(
				"an IMU array runs past gyro_scale: the C writes over the device's haptic function pointer",
			))? = f as f64;
			if read == 0 {
				return Ok(0);
			}
		}
		Ok(count as i32)
	}
}

/// A stack_entry_t as the C reads it: the key of the value being walked, and the key of the value that holds it
/// (stack->previous->key; None at the top object's members)
#[derive(Clone, Copy)]
struct Entry {
	key: usize,
	previous: Option<usize>,
}

/// An object or array being walked: process_jsontok's locals for it (t, count, i, j) and its stack_entry_t's key
struct Frame {
	array: bool,
	t: usize,
	count: i64,
	size: i32,
	i: i32,
	j: i64,
	key: usize,
}

/// scratch_space_t and the device being loaded
struct Load<'a> {
	t: Tokens<'a>,
	so: Object,
	imu_pose: VivePose,
	head: VivePose,
	sensor_scale: f64,
	log: Vec<Log>,
}

impl Load<'_> {
	/// SV_WARN: sprintf into its 1024-byte buffer, so a longer line overflows it (with the fortified build's
	/// __sprintf_chk, the C aborts)
	fn sv_warn(&mut self, text: Vec<u8>) -> Result<(), DeviceConfigError> {
		if text.len() > 1023 {
			return Err(DeviceConfigError::Undefined("a log line overflows SV_WARN's 1024-byte buffer"));
		}
		self.log.push(Log::Warn(text));
		Ok(())
	}

	/// process_jsontok(scratch, d, 0, tokens, r), its recursion run as a loop over a stack of frames (so that a
	/// deeply nested config cannot overflow the thread's stack). Each object's members and array's elements are walked
	/// at t + 1 + j, j counting the tokens before them; an object's key is skipped (j++) and is its value's stack
	/// key, an array element is its own. `count` is the C's: one more than the tokens left (it passes count - j where
	/// count - 1 - j remain), so it reaches 0 only past the pool's end
	fn walk(&mut self) -> Result<(), DeviceConfigError> {
		let mut frames: Vec<Frame> = Vec::new();
		// what the last process_jsontok returned; None while the object or array it was called on is walked
		let mut returned = self.call(&mut frames, 0, self.t.toks.len() as i64)?;
		while let Some(f) = frames.last_mut() {
			if let Some(n) = returned {
				f.j += n;
				f.i += 1;
			}
			if f.i >= f.size {
				returned = Some(f.j + 1);
				frames.pop();
				continue;
			}
			f.key = f.t + 1 + f.j as usize;
			if !f.array {
				f.j += 1; // skip the key
			}
			let (t, count) = (f.t + 1 + f.j as usize, f.count - f.j);
			returned = self.call(&mut frames, t, count)?;
		}
		Ok(())
	}

	/// process_jsontok on token t, inside the objects and arrays on `frames` (the last one's key is its stack entry's):
	/// what it returns for a primitive or a string, or None for an object or array, pushed to be walked
	fn call(&mut self, frames: &mut Vec<Frame>, t: usize, count: i64) -> Result<Option<i64>, DeviceConfigError> {
		if count == 0 {
			return Ok(Some(0));
		}
		let tok = self.t.tok(t)?;
		let stack =
			frames.last().map(|f| Entry { key: f.key, previous: frames.len().checked_sub(2).map(|k| frames[k].key) });
		match tok.kind {
			TokenType::Primitive => Ok(Some(1)),
			TokenType::String => {
				if let Some(stack) = stack {
					self.string_value(stack, &tok)?;
				}
				Ok(Some(1))
			}
			TokenType::Object | TokenType::Array => {
				let array = tok.kind == TokenType::Array;
				if array {
					// an array is never the top token (that is an object), so it always has a stack entry
					let Some(stack) = stack else {
						return Err(DeviceConfigError::Undefined("the C dereferences a NULL stack entry"));
					};
					self.process_jsonarray(stack)?;
				}
				frames.push(Frame { array, t, count, size: tok.size, i: 0, j: 0, key: 0 });
				Ok(None)
			}
		}
	}

	/// process_jsontok's JSMN_STRING case: a string, matched by its stack key (the key it is the value of; an array's
	/// element is its own). The value matches the C's string when it is a prefix of it: strncmp(string, value, value
	/// length), so the empty value matches "controller" and the first model number
	fn string_value(&mut self, stack: Entry, t: &Token) -> Result<(), DeviceConfigError> {
		let key = self.t.tok(stack.key)?;
		let value = self.t.text(t);
		if self.t.jsoneq(&key, b"device_class") {
			if b"controller".starts_with(value) {
				self.so.object_type = ObjectType::Controller;
			} else if b"hmd".starts_with(value) {
				self.so.object_type = ObjectType::Hmd;
			} else if b"generic_tracker".starts_with(value) {
				self.so.object_type = ObjectType::Other;
			}
		} else if self.t.jsoneq(&key, b"device_serial_number") {
			let size = value.len().min(self.so.serial_number.len());
			self.so.serial_number = [0; 16];
			self.so.serial_number[..size].copy_from_slice(&value[..size]);
		} else if self.t.jsoneq(&key, b"model_number") {
			if let Some(m) = MODEL_NUMBER_SUBTYPES.iter().find(|m| m.key.starts_with(value)) {
				self.so.object_subtype = m.subtype;
				self.sensor_scale = m.sensor_scale;
			}
			// the device's subtype, which a model_name or an earlier model_number may have set
			if self.so.object_subtype == ObjectSubtype::Generic {
				let mut line = b"Unknown model_number '".to_vec();
				line.extend_from_slice(value);
				line.extend_from_slice(
					b"'. Please submit an issue with this value describing your device so it can be added to the known \
					list.",
				);
				self.sv_warn(line)?;
			}
		} else if self.t.jsoneq(&key, b"model_name") && b"Vive Tracker MV".starts_with(value) {
			self.so.object_subtype = ObjectSubtype::Tracker;
		}
		Ok(())
	}

	/// process_jsonarray: an array under one of its keys, before its elements are walked. Its return value (-1 when
	/// ParsePoints fails) is ignored by its caller
	fn process_jsonarray(&mut self, stack: Entry) -> Result<(), DeviceConfigError> {
		let tk = stack.key;
		let key = self.t.tok(tk)?;
		if self.t.jsoneq(&key, b"modelPoints") {
			self.parse_points(tk, false)?;
		} else if self.t.jsoneq(&key, b"modelNormals") {
			self.parse_points(tk, true)?;
		} else if self.t.jsoneq(&key, b"channelMap") {
			self.channel_map(tk)?;
		} else if let Some(&(name, offset)) = IMU_FIELDS.iter().find(|(name, _)| self.t.jsoneq(&key, name)) {
			let count = self.t.tok(tk + 1)?.size;
			// parse_float_array_in_place's count is a uint8_t; the array may run on into the fields after this one
			if self.t.parse_float_array_in_place(tk + 2, &mut self.so.imu[offset..], count as u8)? <= 0 {
				let mut line = b"Could not parse ".to_vec();
				line.extend_from_slice(name);
				self.sv_warn(line)?;
			}
		} else if self.t.jsoneq(&key, b"trackref_from_imu") || self.t.jsoneq(&key, b"trackref_from_head") {
			let count = self.t.tok(tk + 1)?.size;
			if count == 7 {
				let mut values = [0.0; 7];
				// parse_float_array returns its count even when a value was not a number, after freeing the values and
				// setting them to NULL, which vive_json_pose_to_survive_pose then reads
				if self.t.parse_float_array_in_place(tk + 2, &mut values, 7)? == 0 {
					return Err(DeviceConfigError::Undefined(
						"a trackref_from_imu / trackref_from_head value is not a number: the C reads its freed (NULL) \
						values",
					));
				}
				let pose = vive_json_pose_to_survive_pose(&values);
				if self.t.jsoneq(&key, b"trackref_from_imu") {
					self.so.imu2trackref = pose;
				} else {
					self.so.head2trackref = pose;
				}
			}
		} else {
			// the context sensitive fields
			self.parse_ctx_sensitive_vive_pose_t(stack, b"imu", false)?;
			self.parse_ctx_sensitive_vive_pose_t(stack, b"head", true)?;
		}
		Ok(())
	}

	/// ParsePoints: modelPoints / modelNormals, [[x, y, z], ...] read as four tokens a point (the point's array, then
	/// its three numbers), each number a primitive of at most 127 bytes read by atof. A token that is not stops it
	/// (SV_GENERAL_ERROR, return 1) with the points read counted and the numbers of the point it stopped in stored.
	/// sensor_ct is shared: whichever of the two is read last sets it
	fn parse_points(&mut self, t: usize, normals: bool) -> Result<i32, DeviceConfigError> {
		let pts = self.t.tok(t + 1)?.size;
		self.so.sensor_ct = 0;
		// a second modelPoints gets a new zeroed buffer: the C's assert(*floats_out == 0) is compiled out (NDEBUG)
		let mut floats = [0.0; 3 * SENSORS];
		let mut result = 0;
		'points: for k in 0..pts as usize {
			for m in 0..3 {
				let tk = self.t.tok(t + 2 + k * 4 + 1 + m)?;
				let elemlen = tk.end - tk.start;
				if tk.kind != TokenType::Primitive || elemlen > 128 - 1 {
					self.log.push(Log::GeneralError(
						format!("Parse error in JSON {} {} {}", tk.kind as i32, elemlen, 128).into_bytes(),
					));
					result = 1;
					break 'points;
				}
				let id = self.so.sensor_ct * 3 + m;
				*floats.get_mut(id).ok_or(DeviceConfigError::Undefined(
					"modelPoints / modelNormals has more than 32 points: the C writes past its 32-sensor buffer",
				))? = c_atof(self.t.text(&tk));
			}
			self.so.sensor_ct += 1;
		}
		*if normals { &mut self.so.sensor_normals } else { &mut self.so.sensor_locations } = Some(floats);
		Ok(result)
	}

	/// channelMap: for each channel, the index of the sensor on it. parse_int_array(..., count) takes count as a
	/// uint8_t and atoi()s each token (stopping at a token with no text, ""); the loop runs over the whole count
	fn channel_map(&mut self, tk: usize) -> Result<(), DeviceConfigError> {
		let count = self.t.tok(tk + 1)?.size;
		let n = count as u8 as usize;
		if n == 0 {
			// parse_int_array returns 0: no channel map (a count of 256, 512, ... included)
			return Ok(());
		}
		let mut values = Vec::with_capacity(n);
		for i in 0..n {
			let v = self.t.tok(tk + 2 + i)?;
			values.push(c_atoi(self.t.from(&v)));
			if v.start == v.end {
				return Err(DeviceConfigError::Undefined(
					"a channelMap entry is an empty string: the C reads its freed (NULL) values",
				));
			}
		}
		let mut map = [-1; SENSORS];
		for i in 0..count {
			let Some(&v) = values.get(i as usize) else {
				return Err(DeviceConfigError::Undefined(
					"channelMap has 256 or more entries: the C reads past the values it read",
				));
			};
			if v >= 0 {
				*map.get_mut(v as usize).ok_or(DeviceConfigError::Undefined(
					"a channelMap entry is 32 or more: the C writes past its 32 channels",
				))? = i;
			}
		}
		// a second channelMap replaces the first (the C leaks it)
		self.so.channel_map = Some(map);
		Ok(())
	}

	/// parse_ctx_sensitive_vive_pose_t: "plus_x", "plus_z" or "position" in an object under the key field_name
	/// ("imu" or "head", whatever holds that object, at any depth), when the array has three entries (the C's
	/// assert(count == 3) is compiled out, so another length is skipped; parse_float_array_in_place's result is ignored)
	fn parse_ctx_sensitive_vive_pose_t(
		&mut self, stack: Entry, field_name: &[u8], head: bool,
	) -> Result<(), DeviceConfigError> {
		let Some(previous) = stack.previous else { return Ok(()) };
		if !self.t.jsoneq(&self.t.tok(previous)?, field_name) {
			return Ok(());
		}
		let tk = self.t.tok(stack.key)?;
		let Some(field) = [&b"plus_x"[..], b"plus_z", b"position"].iter().position(|name| self.t.jsoneq(&tk, name))
		else {
			return Ok(());
		};
		let count = self.t.tok(stack.key + 1)?.size;
		if count == 3 {
			let output = if head { &mut self.head } else { &mut self.imu_pose };
			let vals = match field {
				0 => &mut output.plus_x,
				1 => &mut output.plus_z,
				_ => &mut output.position,
			};
			self.t.parse_float_array_in_place(stack.key + 2, vals, 3)?;
		}
		Ok(())
	}
}

/// vive_json_pose_to_survive_pose: the config's [qx, qy, qz, qw, x, y, z]
fn vive_json_pose_to_survive_pose(values: &[f64; 7]) -> Pose {
	Pose { pos: [values[4], values[5], values[6]], rot: [values[3], values[0], values[1], values[2]] }
}

/// solve_vive_pose: the pose at `position` whose rotation's columns are plus_x, plus_z x plus_x and plus_z; unchanged
/// when plus_x or plus_z is all zeros (not in the config)
fn solve_vive_pose(pose: &mut Pose, vpose: &VivePose) {
	if vpose.plus_x[0] == 0.0 && vpose.plus_x[1] == 0.0 && vpose.plus_x[2] == 0.0 {
		return;
	}
	if vpose.plus_z[0] == 0.0 && vpose.plus_z[1] == 0.0 && vpose.plus_z[2] == 0.0 {
		return;
	}
	pose.pos = vpose.position;
	let plus_y = cross3d(&vpose.plus_z, &vpose.plus_x);
	#[rustfmt::skip]
	let m = [
		vpose.plus_x[0], plus_y[0], vpose.plus_z[0],
		vpose.plus_x[1], plus_y[1], vpose.plus_z[1],
		vpose.plus_x[2], plus_y[2], vpose.plus_z[2],
	];
	pose.rot = quatfrommatrix33(&m);
}

impl DeviceConfig {
	/// survive_load_htc_config_format(so, json, json.len()) on a device survive_create_device just made with this
	/// codename (the C's codename: its bytes up to a NUL)
	pub fn parse(json: &[u8], codename: &str, options: &ConfigOptions) -> Result<DeviceConfig, DeviceConfigError> {
		let codename = codename.as_bytes();
		let codename = &codename[..codename.iter().position(|&c| c == 0).unwrap_or(codename.len())];
		if json.is_empty() {
			return Err(DeviceConfigError::Empty);
		}
		if json.len() > i32::MAX as usize {
			return Err(DeviceConfigError::Undefined("the config is longer than the C's int length"));
		}
		let toks = jsmn_parse(json).map_err(|e| DeviceConfigError::Json(e.code()))?;
		if toks.first().map(|t| t.kind) != Some(TokenType::Object) {
			return Err(DeviceConfigError::NotAnObject);
		}

		let mut load = Load {
			t: Tokens { json, toks: &toks },
			so: Object::created(),
			imu_pose: VivePose::default(),
			head: VivePose::default(),
			sensor_scale: 0.0,
			log: Vec::new(),
		};
		load.so.object_type = ObjectType::Other;
		load.walk()?;
		let Load { mut so, imu_pose, head, sensor_scale, mut log, .. } = load;

		solve_vive_pose(&mut so.imu2trackref, &imu_pose);
		solve_vive_pose(&mut so.head2trackref, &head);

		let trackref2imu = invert_pose(&so.imu2trackref);

		let mut sensors_are_zero = true;
		if so.sensor_ct > 0 {
			let (Some(locations), Some(normals)) = (so.sensor_locations.as_mut(), so.sensor_normals.as_mut()) else {
				return Err(DeviceConfigError::Undefined(
					"the config has modelPoints without modelNormals or the other way round: the C dereferences the \
					missing one's NULL",
				));
			};
			for i in 0..so.sensor_ct {
				// the C's norm3d(&so->sensor_locations[i]): indexed by the sensor, not by i * 3, so it is the norm of
				// the doubles i, i + 1, i + 2, from sensor 1 on partly of sensors already moved into the IMU's frame
				if norm3d(&[locations[i], locations[i + 1], locations[i + 2]]) > 0.001 {
					sensors_are_zero = false;
				}
				let mut p = [locations[i * 3], locations[i * 3 + 1], locations[i * 3 + 2]];
				if sensor_scale != 0.0 {
					p = scale3d(&p, sensor_scale);
				}
				locations[i * 3..i * 3 + 3].copy_from_slice(&apply_pose_to_point(&trackref2imu, &p));
				let n = [normals[i * 3], normals[i * 3 + 1], normals[i * 3 + 2]];
				normals[i * 3..i * 3 + 3].copy_from_slice(&quatrotatevector(&trackref2imu.rot, &n));
			}
		}
		let has_sensor_locations = !sensors_are_zero;

		so.head2imu = apply_pose_to_pose(&trackref2imu, &so.head2trackref);

		let mut acc_bias = [so.imu[0], so.imu[1], so.imu[2]];
		let mut gyro_bias = [so.imu[6], so.imu[7], so.imu[8]];

		// device-specific scaling
		if codename == b"HMD" || so.object_type == ObjectType::Hmd {
			let mut line = codename.to_vec();
			line.extend_from_slice(b" is treated as HMD device");
			log.push(sv_info(line));
			if so.raw_acc_scale == 0.0 {
				so.raw_acc_scale = 1. / 8192.;
			}
			acc_bias = scale3d(&acc_bias, 1. / 1000.);
			so.imu_freq = HMD_IMU_HZ as f64;
			let deg_per_sec = 500.0;
			if so.raw_gyro_scale == 0.0 {
				so.raw_gyro_scale = deg_per_sec / (1 << 15) as f64 * LINMATHPI / 180.;
			}
		} else {
			// the C has two branches here, for the codenames that start "WM" and for the rest; they do the same
			if so.raw_acc_scale == 0.0 {
				so.raw_acc_scale = 2. / 8192.;
			}
			acc_bias = scale3d(&acc_bias, 1. / 1000.);
			let deg_per_sec = 2000.0;
			if so.raw_gyro_scale == 0.0 {
				so.raw_gyro_scale = deg_per_sec / (1 << 15) as f64 * LINMATHPI / 180.;
			}
		}

		if options.imu_acc_bias_units == 1 {
			// sally: undo the division by 1000 above and read acc_bias as m/s^2
			acc_bias = scale3d(&acc_bias, 1000. / 9.80665);
		}
		if options.ignore_config_imu_bias {
			// a scale by 0, so a negative bias becomes -0
			gyro_bias = scale3d(&gyro_bias, 0.0);
			acc_bias = scale3d(&acc_bias, 0.0);
		}
		so.imu[0..3].copy_from_slice(&acc_bias);
		so.imu[6..9].copy_from_slice(&gyro_bias);

		let named = survive_colorize(codename, options.colorize);
		// Point3_format
		let point3 = |v: &[f64]| sprintf("%+le   %+le   %+le", &[Arg::F(v[0]), Arg::F(v[1]), Arg::F(v[2])]);
		for (what, bias, scale) in [(&b" has gyro bias "[..], 6, 9), (b" has acc bias  ", 0, 3)] {
			let mut line = b"Device ".to_vec();
			line.extend_from_slice(&named);
			line.extend_from_slice(what);
			line.extend_from_slice(point3(&so.imu[bias..bias + 3]).as_bytes());
			line.extend_from_slice(b" scale ");
			line.extend_from_slice(point3(&so.imu[scale..scale + 3]).as_bytes());
			line.truncate(1023);
			log.push(Log::Verbose(110, line));
		}
		let mut line = b"Read config for ".to_vec();
		line.extend_from_slice(&named);
		line.truncate(1023);
		log.push(Log::Verbose(50, line));

		let points =
			|p: Option<[f64; 3 * SENSORS]>| p.map(|p| std::array::from_fn(|i| [p[i * 3], p[i * 3 + 1], p[i * 3 + 2]]));
		Ok(DeviceConfig {
			object_type: so.object_type,
			object_subtype: so.object_subtype,
			serial_number: so.serial_number,
			sensor_ct: so.sensor_ct,
			sensor_locations: points(so.sensor_locations),
			sensor_normals: points(so.sensor_normals),
			channel_map: so.channel_map,
			has_sensor_locations,
			imu_freq: so.imu_freq,
			head2trackref: so.head2trackref,
			imu2trackref: so.imu2trackref,
			head2imu: so.head2imu,
			raw_acc_scale: so.raw_acc_scale,
			raw_gyro_scale: so.raw_gyro_scale,
			acc_bias,
			acc_scale: [so.imu[3], so.imu[4], so.imu[5]],
			gyro_bias,
			gyro_scale: [so.imu[9], so.imu[10], so.imu[11]],
			log,
		})
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Two of the configs Valve's devices carry: an Index controller (left) and a VIVE Tracker 3.0
	const KNUCKLES: &[u8] = include_bytes!("testdata/knuckles_left_config.json");
	const TRACKER: &[u8] = include_bytes!("testdata/vive_tracker_3_config.json");

	fn bits3(v: &[f64; 3]) -> [u64; 3] {
		v.map(f64::to_bits)
	}

	fn pose_bits(p: &Pose) -> [u64; 7] {
		[p.pos[0], p.pos[1], p.pos[2], p.rot[0], p.rot[1], p.rot[2], p.rot[3]].map(f64::to_bits)
	}

	/// FNV-1a (64) of the 32 * 3 doubles' bits, each little-endian, in order
	fn fnv(points: &[[f64; 3]; 32]) -> u64 {
		points
			.iter()
			.flatten()
			.flat_map(|v| v.to_bits().to_le_bytes())
			.fold(0xcbf2_9ce4_8422_2325, |h, b| (h ^ b as u64).wrapping_mul(0x0100_0000_01b3))
	}

	fn verbose(level: i32, text: &str) -> Log {
		Log::Verbose(level, text.as_bytes().to_vec())
	}

	fn parse(json: &str) -> DeviceConfig {
		DeviceConfig::parse(json.as_bytes(), "T20", &ConfigOptions::default()).unwrap()
	}

	fn refused(json: &str) -> bool {
		matches!(
			DeviceConfig::parse(json.as_bytes(), "T20", &ConfigOptions::default()),
			Err(DeviceConfigError::Undefined(_))
		)
	}

	// The expected values in the next three tests are the C's: survive_load_htc_config_format on a device from
	// survive_create_device, in the C runtime's libsurvive, every double printed with %a

	#[test]
	fn knuckles_as_the_c_loads_it() {
		let c = DeviceConfig::parse(KNUCKLES, "KN0", &ConfigOptions { colorize: true, ..Default::default() }).unwrap();
		assert_eq!(c.object_type, ObjectType::Controller);
		assert_eq!(c.object_subtype, ObjectSubtype::KnucklesL);
		assert_eq!(&c.serial_number, b"LHR-9365E531\0\0\0\0");
		assert_eq!(c.sensor_ct, 23);
		assert!(c.has_sensor_locations);
		#[rustfmt::skip]
		assert_eq!(c.channel_map, Some([
			-1, 8, 5, 14, 9, 21, 3, 15, 22, 12, 18, -1, 11, 1, 4, 13, 7, 16, 17, 10, 0, 6, 20, 19, 2,
			-1, -1, -1, -1, -1, -1, -1,
		]));
		assert_eq!(c.imu_freq.to_bits(), 0x406f400000000000);
		assert_eq!(c.raw_acc_scale.to_bits(), 0x3f30000000000000);
		assert_eq!(c.raw_gyro_scale.to_bits(), 0x3f51740afa84ad89);
		assert_eq!(bits3(&c.acc_bias), [0xbf1f80fef9db22d1, 0xbf02f83e978d4fdf, 0x3f13a34439581062]);
		// read through strtof: the low 29 bits of a float widened to a double are zero
		assert_eq!(bits3(&c.acc_scale), [0x3ff030c6c0000000, 0x3fef849780000000, 0x3feff8a020000000]);
		assert_eq!(bits3(&c.gyro_bias), [0xbf941cb7a0000000, 0x3f83366f20000000, 0xbf79d177e0000000]);
		assert_eq!(c.gyro_scale, [1.0; 3]);
		#[rustfmt::skip]
		assert_eq!(pose_bits(&c.head2trackref), [
			0xbf8607c420000000, 0x3f964dc220000000, 0xbfa5935fc0000000,
			0x3fef1c8d25cdb627, 0x3fcafdb3a35a319d, 0x3fb8a455d2438e99, 0xbfa06a8a615851e1,
		]);
		#[rustfmt::skip]
		assert_eq!(pose_bits(&c.imu2trackref), [
			0x3f830e8000000000, 0xbfadc833e0000000, 0x3fb7ff8500000000,
			0x3fe683079d0f3178, 0xbfc80bf74f58ac7e, 0xbfe3ea8064dbb431, 0x3fd26086a579f65f,
		]);
		#[rustfmt::skip]
		assert_eq!(pose_bits(&c.head2imu), [
			0xbfabed5b724e92e0, 0x3fc31316f09bca72, 0xbf81e2a797ba6f84,
			0x3fe2684ee0f396da, 0x3fd5ace507c6d6b0, 0x3fe3c901d02a825c, 0xbfda8d6673f75165,
		]);
		assert_eq!(fnv(&c.sensor_locations.unwrap()), 0x35dae3296b01e0bf);
		assert_eq!(fnv(&c.sensor_normals.unwrap()), 0x6cd757b82e70eb3c);
		assert_eq!(
			c.log,
			[
				verbose(
					110,
					"Device \x1b[0;36mKN0\x1b[0m has gyro bias -1.964080e-02   +9.381169e-03   -6.303280e-03 scale \
					+1.000000e+00   +1.000000e+00   +1.000000e+00"
				),
				verbose(
					110,
					"Device \x1b[0;36mKN0\x1b[0m has acc bias  -1.201778e-04   -3.618184e-05   +7.491211e-05 scale \
					+1.011908e+00   +9.849355e-01   +9.990998e-01"
				),
				verbose(50, "Read config for \x1b[0;36mKN0\x1b[0m"),
			]
		);
	}

	#[test]
	fn tracker_as_the_c_loads_it() {
		let c = DeviceConfig::parse(TRACKER, "T20", &ConfigOptions::default()).unwrap();
		assert_eq!(c.object_type, ObjectType::Other);
		assert_eq!(c.object_subtype, ObjectSubtype::TrackerGen2);
		assert_eq!(&c.serial_number, b"LHR-B0D67561\0\0\0\0");
		assert_eq!(c.sensor_ct, 20);
		assert!(c.has_sensor_locations);
		let mut channels = [-1; 32];
		for i in 0..20 {
			channels[i] = i as i32;
		}
		assert_eq!(c.channel_map, Some(channels));
		assert_eq!(c.imu_freq, 250.0);
		assert_eq!(bits3(&c.acc_bias), [0x3ed3e07666666667, 0xbf110e1a3d70a3d7, 0xbf27a89333333333]);
		assert_eq!(bits3(&c.gyro_bias), [0xbf77891e20000000, 0x3f8e646f20000000, 0x3f84d40240000000]);
		assert_eq!(pose_bits(&c.head2trackref), [0, 0, 0xbf589374c0000000, 0, 0x3ff0000000000000, 0, 0]);
		#[rustfmt::skip]
		assert_eq!(pose_bits(&c.imu2trackref), [
			0x3f8c2e6120000000, 0xbf69afb2a0000000, 0x3f79998ee0000000,
			0x3fe6a09e6cfe7797, 0, 0, 0x3fe6a09e495f61a4,
		]);
		#[rustfmt::skip]
		assert_eq!(pose_bits(&c.head2imu), [
			0x3f69afb1ee8ac618, 0x3f8c2e612a1bf6a1, 0xbf7fbe6c10000000,
			0, 0x3fe6a09e784ec6c8, 0xbfe6a09e54afb0c3, 0,
		]);
		assert_eq!(fnv(&c.sensor_locations.unwrap()), 0x990c171685f06aef);
		assert_eq!(fnv(&c.sensor_normals.unwrap()), 0x9592d16282978985);
		assert_eq!(c.log[2], verbose(50, "Read config for T20"));
	}

	#[test]
	fn the_device_branches_and_the_options_as_the_c_has_them() {
		// codename HMD: the HMD branch whatever the device class, and both options
		let o = ConfigOptions { imu_acc_bias_units: 1, ignore_config_imu_bias: true, colorize: true };
		let c = DeviceConfig::parse(KNUCKLES, "HMD", &o).unwrap();
		assert_eq!(c.object_type, ObjectType::Controller);
		assert_eq!(c.imu_freq, 1000.0);
		assert_eq!(c.raw_acc_scale.to_bits(), 0x3f20000000000000);
		assert_eq!(c.raw_gyro_scale.to_bits(), 0x3f31740afa84ad89);
		// a scale by 0: the negative biases are -0
		assert_eq!(bits3(&c.acc_bias), [0x8000000000000000, 0x8000000000000000, 0]);
		assert_eq!(bits3(&c.gyro_bias), [0x8000000000000000, 0, 0x8000000000000000]);
		assert_eq!(c.log[0], Log::Info(b"HMD is treated as HMD device".to_vec()));
		assert_eq!(fnv(&c.sensor_locations.unwrap()), 0x35dae3296b01e0bf);
		// imu-acc-bias-units 1 on a WM codename
		let o = ConfigOptions { imu_acc_bias_units: 1, ..Default::default() };
		let c = DeviceConfig::parse(TRACKER, "WM0", &o).unwrap();
		assert_eq!(c.imu_freq, 250.0);
		assert_eq!(bits3(&c.acc_bias), [0x3f3fab7b158afda4, 0xbf7b2c8a33e4c328, 0xbf92d8fd8b65e204]);
		assert_eq!(fnv(&c.sensor_normals.unwrap()), 0x9592d16282978985);
		// imu-acc-bias-units as survive_configi has it: only 1 counts
		let o = ConfigOptions { imu_acc_bias_units: 2, ..Default::default() };
		assert_eq!(DeviceConfig::parse(TRACKER, "WM0", &o).unwrap().acc_bias, parse_tracker().acc_bias);
	}

	fn parse_tracker() -> DeviceConfig {
		DeviceConfig::parse(TRACKER, "T20", &ConfigOptions::default()).unwrap()
	}

	#[test]
	fn arrays_are_floats_and_points_are_doubles() {
		let c = parse(
			r#"{"imu": {"acc_scale": [0.1, "0.2", 3e-1]}, "modelPoints": [[0.1, 0, 0]], "modelNormals": [[0, 0, 1]]}"#,
		);
		assert_eq!(c.acc_scale, [0.1f32 as f64, 0.2f32 as f64, 0.3f32 as f64]);
		assert_eq!(c.sensor_locations.unwrap()[0][0], 0.1);
	}

	#[test]
	fn strings_match_as_prefixes() {
		assert_eq!(parse(r#"{"device_class": ""}"#).object_type, ObjectType::Controller);
		assert_eq!(parse(r#"{"device_class": "h"}"#).object_type, ObjectType::Hmd);
		assert_eq!(parse(r#"{"device_class": "HMD"}"#).object_type, ObjectType::Other);
		assert_eq!(parse(r#"{"model_number": "Vive"}"#).object_subtype, ObjectSubtype::Wand);
		assert_eq!(parse(r#"{"model_number": ""}"#).object_subtype, ObjectSubtype::KnucklesR);
		assert_eq!(parse(r#"{"model_number": "VIVE_Pro 2 MV"}"#).object_subtype, ObjectSubtype::ViveHmd);
		assert_eq!(parse(r#"{"model_name": "Vive Tracker"}"#).object_subtype, ObjectSubtype::Tracker);
		let c = parse(r#"{"model_number": "Vive Tracker MV extra"}"#);
		assert_eq!(c.object_subtype, ObjectSubtype::Generic);
		assert_eq!(
			c.log[0],
			Log::Warn(
				b"Unknown model_number 'Vive Tracker MV extra'. Please submit an issue with this value describing your \
				device so it can be added to the known list."
					.to_vec()
			)
		);
		// a key matches in any object, at any depth, and an escape in a key is not decoded
		assert_eq!(parse(r#"{"a": {"b": {"device_class": "hmd"}}}"#).object_type, ObjectType::Hmd);
		assert_eq!(parse(r#"{"device\u005fclass": "hmd"}"#).object_type, ObjectType::Other);
		// the warning tests the device's subtype, which an earlier model_name may have set
		let c = parse(r#"{"model_name": "Vive Tracker MV", "model_number": "Nope"}"#);
		assert!(!c.log.iter().any(|l| matches!(l, Log::Warn(_))));
		// the serial number as the C's 16 bytes hold it
		assert_eq!(&parse(r#"{"device_serial_number": "LHR-0123456789ABCDEF"}"#).serial_number, b"LHR-0123456789AB");
	}

	#[test]
	fn poses_under_any_imu_or_head_key() {
		let pose = r#"{"plus_x": [0, 1, 0], "plus_z": [1, 0, 0], "position": [0.5, 0, 0]}"#;
		let c = parse(&format!(r#"{{"x": {{"y": {{"imu": {pose}}}}}}}"#));
		assert_eq!(c.imu2trackref.pos, [0.5, 0.0, 0.0]);
		// three entries or none: plus_x with two is not read, so neither pose is solved
		let c = parse(r#"{"imu": {"plus_x": [0, 1], "plus_z": [1, 0, 0], "position": [0.5, 0, 0]}}"#);
		assert_eq!(c.imu2trackref, Pose { pos: [0.0; 3], rot: [1.0, 0.0, 0.0, 0.0] });
		// a trackref pose is [qx, qy, qz, qw, x, y, z], and the imu one replaces it
		let c = parse(r#"{"trackref_from_head": [0, 0, 1, 0, 1, 2, 3]}"#);
		assert_eq!(c.head2trackref, Pose { pos: [1.0, 2.0, 3.0], rot: [0.0, 0.0, 0.0, 1.0] });
		let c = parse(&format!(r#"{{"imu": {pose}, "trackref_from_imu": [0, 0, 1, 0, 1, 2, 3]}}"#));
		assert_eq!(c.imu2trackref.pos, [0.5, 0.0, 0.0]);
	}

	#[test]
	fn the_imu_arrays_run_on_into_the_next_field() {
		// gyro_scale follows gyro_bias in the C's struct
		let c = parse(r#"{"imu": {"gyro_bias": [1, 2, 3, 4, 5, 6]}}"#);
		assert_eq!((c.gyro_bias, c.gyro_scale), ([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]));
		// the count is a uint8_t: 257 values read one, 256 read none (and warn)
		let c = parse(&format!(r#"{{"imu": {{"gyro_bias": [{}]}}}}"#, ["0.5"; 257].join(",")));
		assert_eq!(c.gyro_bias, [0.5, 0.0, 0.0]);
		let c = parse(&format!(r#"{{"imu": {{"gyro_bias": [{}]}}}}"#, ["0.5"; 256].join(",")));
		assert_eq!(c.log[0], Log::Warn(b"Could not parse gyro_bias".to_vec()));
		// a value that is not a number is stored as strtof's 0, the rest left as they were
		let c = parse(r#"{"imu": {"acc_scale": [2, "x", 3]}}"#);
		assert_eq!(c.acc_scale, [2.0, 0.0, 1.0]);
		assert_eq!(c.log[0], Log::Warn(b"Could not parse acc_scale".to_vec()));
		// past gyro_scale is the device's haptic pointer
		assert!(refused(r#"{"imu": {"gyro_scale": [1, 1, 1, 1]}}"#));
	}

	#[test]
	fn points_and_channels() {
		// has_sensor_locations tests the doubles i, i + 1, i + 2 for sensor i: here 0..3, 1..4, 2..5, all zero, though
		// sensor 2 is half a metre out
		let normals = r#""modelNormals": [[0, 0, 1], [0, 0, 1], [0, 0, 1]]"#;
		let c = parse(&format!(r#"{{"modelPoints": [[0, 0, 0], [0, 0, 0], [0, 0, 0.5]], {normals}}}"#));
		assert_eq!(c.sensor_ct, 3);
		assert!(!c.has_sensor_locations);
		assert_eq!(c.sensor_locations.unwrap()[2], [0.0, 0.0, 0.5]);
		assert!(parse(&format!(r#"{{"modelPoints": [[0, 0, 0], [0.5, 0, 0]], {normals}}}"#)).has_sensor_locations);
		// sensor_ct is whichever list was read last; a point that is not three primitives stops it
		let c = parse(&format!(r#"{{"modelPoints": [[0, 0, 0], [1, 2, 3], ["4", 5, 6]], {normals}}}"#));
		assert_eq!(c.sensor_ct, 3);
		let c = parse(&format!(r#"{{{normals}, "modelPoints": [[0, 0, 0], [1, 2, 3], ["4", 5, 6]]}}"#));
		assert_eq!(c.sensor_ct, 2);
		assert_eq!(c.log[0], Log::GeneralError(b"Parse error in JSON 3 1 128".to_vec()));
		// channels: atoi of each entry, a negative one skipped, the last of a repeated one kept
		let c = parse(r#"{"channelMap": ["3", " 4", -1, 3.9, "0x10", 4294967327]}"#);
		let mut channels = [-1; 32];
		(channels[0], channels[3], channels[4], channels[31]) = (4, 3, 1, 5);
		assert_eq!(c.channel_map, Some(channels));
		assert_eq!(parse(&format!(r#"{{"channelMap": [{}]}}"#, ["1"; 256].join(","))).channel_map, None);
	}

	#[test]
	fn configs_the_c_cannot_read_are_refused() {
		assert!(refused(r#"{"modelPoints": [[0, 0, 0]]}"#));
		assert!(refused(r#"{"modelNormals": [[0, 0, 1]]}"#));
		let points = ["[0, 0, 0]"; 33].join(",");
		assert!(refused(&format!(r#"{{"modelPoints": [{points}], "modelNormals": [{points}]}}"#)));
		assert!(refused(r#"{"channelMap": [0, 32]}"#));
		assert!(refused(r#"{"channelMap": [0, ""]}"#));
		assert!(refused(&format!(r#"{{"channelMap": [{}]}}"#, ["1"; 257].join(","))));
		assert!(refused(r#"{"trackref_from_imu": [0, 0, 0, 1, 0, "x", 0]}"#));
		// SV_WARN's buffer: a 1023-byte line fits, 1024 does not
		assert!(!refused(&format!(r#"{{"model_number": "{}"}}"#, "M".repeat(899))));
		assert!(refused(&format!(r#"{{"model_number": "{}"}}"#, "M".repeat(900))));
		// a key without a value reads past the tokens, unless the C's count stops the walk first
		assert!(refused(r#"{"a" "b" "c"}"#));
		assert_eq!(parse(r#"{"device_class" "hmd"}"#).object_type, ObjectType::Hmd);
	}

	#[test]
	fn errors_before_the_walk() {
		let err = |json: &[u8]| DeviceConfig::parse(json, "T20", &ConfigOptions::default()).unwrap_err();
		assert_eq!((err(b""), err(b"").code(), err(b"").log()), (DeviceConfigError::Empty, Some(-1), None));
		let e = err(b"{\"a\": [1");
		assert_eq!((e.clone(), e.code()), (DeviceConfigError::Json(-3), Some(-1)));
		assert_eq!(e.log(), Some(Log::Info(b"Failed to parse JSON in HMD configuration: -3\n".to_vec())));
		assert_eq!(err(b"{\"a\": \"\\q\"}"), DeviceConfigError::Json(-2));
		let e = err(b"[1]");
		assert_eq!((e.clone(), e.code()), (DeviceConfigError::NotAnObject, Some(-2)));
		assert_eq!(e.log(), Some(Log::Info(b"Object expected in HMD configuration\n".to_vec())));
		assert_eq!(err(b" \n"), DeviceConfigError::NotAnObject);
	}

	#[test]
	fn deep_nesting_does_not_overflow_the_stack() {
		// the C's recursion overflows an 8 MB stack between 6 000 and 8 000 levels; the walk here keeps its own stack
		let depth = 20_000;
		let json = format!(r#"{{"x": {}{}, "device_class": "hmd"}}"#, "[".repeat(depth), "]".repeat(depth));
		assert_eq!(parse(&json).object_type, ObjectType::Hmd);
	}

	#[test]
	fn the_colorized_codename() {
		assert_eq!(survive_colorize(b"KN0", true), b"\x1b[0;36mKN0\x1b[0m");
		assert_eq!(survive_colorize(b"WM0", true), b"\x1b[0;31mWM0\x1b[0m");
		assert_eq!(survive_colorize(b"KN0", false), b"KN0");
		assert_eq!(survive_colorize(&[b'x'; 110], true), [b'x'; 110]);
	}
}
