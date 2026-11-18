//! A device's config cut down to what sally reads of it, its serial a stand-in.
//!
//! A config is the JSON a device carries: its class and model, where its sensors and its IMU sit, the IMU's factory
//! calibration; and things about the unit sally has no use for (when it was built, its firmware's settings, a
//! headset's display). Only the members named in `KEPT` are written, as they stand in the config; the serial's value
//! is the device's stand-in. What is kept still says which unit it is to its maker (no two units have the same
//! calibration), and to no one else.

use super::names::Names;

/// The members of a config that are kept: what sally's parser and the panel read, and what says which kind of device
/// it is
const KEPT: &[&str] = &[
	"device_class",
	"device_pid",
	"device_vid",
	"device_serial_number",
	"manufacturer",
	"model_number",
	"model_name",
	"render_model",
	"resource_root",
	"revision",
	"type",
	"tracked_controller_role",
	"input_profile",
	"head",
	"imu",
	"lighthouse_config",
	"trackref_from_imu",
	"trackref_from_head",
];

fn space(c: u8) -> bool {
	c == b' ' || c == b'\t' || c == b'\r' || c == b'\n'
}

/// The end of the JSON string that starts at `at` (the place after its closing quote)
fn string_end(js: &[u8], at: usize) -> Option<usize> {
	let mut i = at + 1;
	while i < js.len() {
		match js[i] {
			b'\\' => i += 2,
			b'"' => return Some(i + 1),
			_ => i += 1,
		}
	}
	None
}

/// The end of the JSON value that starts at `at`
fn value_end(js: &[u8], at: usize) -> Option<usize> {
	match *js.get(at)? {
		b'"' => string_end(js, at),
		b'{' | b'[' => {
			let mut depth = 0usize;
			let mut i = at;
			while i < js.len() {
				match js[i] {
					b'"' => {
						i = string_end(js, i)?;
						continue;
					}
					b'{' | b'[' => depth += 1,
					b'}' | b']' => {
						depth -= 1;
						if depth == 0 {
							return Some(i + 1);
						}
					}
					_ => {}
				}
				i += 1;
			}
			None
		}
		_ => {
			let mut i = at;
			while i < js.len() && !space(js[i]) && js[i] != b',' && js[i] != b'}' {
				i += 1;
			}
			(i > at).then_some(i)
		}
	}
}

/// The members of the object a config is: each name with its value as it stands. None for anything else
fn members(js: &[u8]) -> Option<Vec<(String, &[u8])>> {
	let mut i = js.iter().position(|&c| c == b'{')? + 1;
	if !js[..i - 1].iter().all(|&c| space(c) || c >= 0x80) {
		return None;
	}
	let mut out = Vec::new();
	loop {
		while i < js.len() && (space(js[i]) || js[i] == b',') {
			i += 1;
		}
		match *js.get(i)? {
			b'}' => return Some(out),
			b'"' => {
				let end = string_end(js, i)?;
				let name = String::from_utf8_lossy(&js[i + 1..end - 1]).into_owned();
				i = end;
				while i < js.len() && space(js[i]) {
					i += 1;
				}
				if *js.get(i)? != b':' {
					return None;
				}
				i += 1;
				while i < js.len() && space(js[i]) {
					i += 1;
				}
				let end = value_end(js, i)?;
				out.push((name, &js[i..end]));
				i = end;
			}
			_ => return None,
		}
	}
}

/// The device's serial in a config, as it is written there
pub fn serial(json: &[u8]) -> Option<String> {
	let m = members(json)?;
	let v = m.iter().find(|(n, _)| n == "device_serial_number")?.1;
	(v.len() >= 2 && v[0] == b'"').then(|| String::from_utf8_lossy(&v[1..v.len() - 1]).into_owned())
}

/// The config with the members kept alone and the device's stand-in as its serial. None for a config that is not a
/// JSON object: it is then left out of the share
pub fn scrub(json: &[u8], names: &mut Names) -> Option<Vec<u8>> {
	let m = members(json)?;
	let mut out = Vec::with_capacity(json.len());
	out.extend_from_slice(b"{\n");
	let mut first = true;
	for (name, value) in m.iter().filter(|(n, _)| KEPT.contains(&n.as_str())) {
		if !first {
			out.extend_from_slice(b",\n");
		}
		first = false;
		out.extend_from_slice(format!("\t\"{name}\": ").as_bytes());
		if name == "device_serial_number" {
			let real = String::from_utf8_lossy(value).trim_matches('"').to_string();
			out.extend_from_slice(format!("\"{}\"", names.serial(&real, "device")).as_bytes());
		} else {
			out.extend_from_slice(value);
		}
	}
	out.extend_from_slice(b"\n}\n");
	Some(out)
}

#[cfg(test)]
mod tests {
	use super::*;

	const CONFIG: &str = r#"{
	"device_class" : "generic_tracker",
	"device_serial_number" : "LHR-1A2B3C4D",
	"firmware_config" : { "button_function_5" : "PA2", "note" : "a } in \" a string" },
	"htcComposeTime" : "2021-06-15 19:05:46.535000",
	"imu" : { "acc_bias" : [ -0.02, 0.5, 1e-3 ], "plus_x" : [ 1, 0, 0 ] },
	"lighthouse_config" : { "channelMap" : [ 0, 1 ], "modelPoints" : [ [ 0, 0.5, 1 ] ] },
	"mb_serial_number" : "FA9B1XX00000",
	"revision" : 3
}"#;

	#[test]
	fn only_what_sally_reads_is_kept() {
		let mut n = Names::default();
		assert_eq!(serial(CONFIG.as_bytes()).as_deref(), Some("LHR-1A2B3C4D"));
		let out = String::from_utf8(scrub(CONFIG.as_bytes(), &mut n).unwrap()).unwrap();
		assert_eq!(
			out,
			"{\n\t\"device_class\": \"generic_tracker\",\n\t\"device_serial_number\": \"LHR-00000001\",\n\t\"imu\": { \
			 \"acc_bias\" : [ -0.02, 0.5, 1e-3 ], \"plus_x\" : [ 1, 0, 0 ] },\n\t\"lighthouse_config\": { \"channelMap\" : \
			 [ 0, 1 ], \"modelPoints\" : [ [ 0, 0.5, 1 ] ] },\n\t\"revision\": 3\n}\n"
		);
		// and it is a config again
		assert_eq!(serial(out.as_bytes()).as_deref(), Some("LHR-00000001"));
	}

	#[test]
	fn what_is_not_an_object_is_not_a_config() {
		let mut n = Names::default();
		for bad in ["", "[1, 2]", "{ \"a\" : ", "{ \"a\" 1 }", "x { }", "{ \"a\" : \"unclosed }"] {
			assert_eq!(scrub(bad.as_bytes(), &mut n), None, "{bad}");
		}
		assert_eq!(scrub(b"\xef\xbb\xbf { }", &mut n).as_deref(), Some(&b"{\n\n}\n"[..]));
	}
}
