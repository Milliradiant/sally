//! SteamVR, as an app sees it, read-only: where it keeps its config and logs, its devices (serial, role, the dongle
//! each is on), and the poses it shows. The client is reached the way openvr_api.dll reaches it (OpenVR's
//! openvr_api_public.cpp), without shipping that DLL: the path registry names the runtime, its vrclient_x64.dll makes
//! the client core, and the core is started as a background app (it never starts SteamVR, and closes with it). The
//! one interface used, IVRSystem, is taken as its C function table (OpenVR's openvr_capi.h, IVRSystem_022, which
//! every SteamVR since 2021 serves).

// the writing half is the recorder's, which is Windows only
#![cfg_attr(not(windows), allow(dead_code))]

// the recorder's, which is Windows only (and the tests')
#[cfg_attr(not(windows), allow(unused_imports))]
pub use sally_lighthouse::steamvr::Paths;

use crate::session::SteamDevice;

/// A rotation matrix (rows) as a quaternion (w, x, y, z)
pub fn quaternion(m: &[[f32; 4]; 3]) -> [f64; 4] {
	let r = |i: usize, j: usize| m[i][j] as f64;
	let tr = r(0, 0) + r(1, 1) + r(2, 2);
	let q = if tr > 0.0 {
		let s = (tr + 1.0).sqrt() * 2.0;
		[0.25 * s, (r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s, (r(1, 0) - r(0, 1)) / s]
	} else if r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2) {
		let s = (1.0 + r(0, 0) - r(1, 1) - r(2, 2)).sqrt() * 2.0;
		[(r(2, 1) - r(1, 2)) / s, 0.25 * s, (r(0, 1) + r(1, 0)) / s, (r(0, 2) + r(2, 0)) / s]
	} else if r(1, 1) > r(2, 2) {
		let s = (1.0 + r(1, 1) - r(0, 0) - r(2, 2)).sqrt() * 2.0;
		[(r(0, 2) - r(2, 0)) / s, (r(0, 1) + r(1, 0)) / s, 0.25 * s, (r(1, 2) + r(2, 1)) / s]
	} else {
		let s = (1.0 + r(2, 2) - r(0, 0) - r(1, 1)).sqrt() * 2.0;
		[(r(1, 0) - r(0, 1)) / s, (r(0, 2) + r(2, 0)) / s, (r(1, 2) + r(2, 1)) / s, 0.25 * s]
	};
	let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt().max(1e-12);
	[q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

#[cfg(windows)]
pub use client::{Client, MAX_DEVICES};

#[cfg(windows)]
mod client {
	use std::ffi::{c_char, c_void, CString};
	use std::os::windows::ffi::OsStrExt;

	use super::{quaternion, Paths};
	use crate::session::{Pose, SteamDevice};

	/// k_unMaxTrackedDeviceCount
	pub const MAX_DEVICES: usize = 64;

	const LOAD_WITH_ALTERED_SEARCH_PATH: u32 = 0x8;
	/// EVRApplicationType: VRApplication_Background (needs SteamVR running, never starts it)
	const APP_BACKGROUND: i32 = 3;
	/// ETrackingUniverseOrigin: TrackingUniverseStanding
	const UNIVERSE_STANDING: i32 = 1;
	/// ETrackedDeviceClass
	const CLASS_HMD: i32 = 1;
	const CLASS_CONTROLLER: i32 = 2;
	const CLASS_TRACKER: i32 = 3;
	const CLASS_STATION: i32 = 4;
	/// ETrackedDeviceProperty
	const PROP_TRACKING_SYSTEM: i32 = 1000;
	const PROP_MODEL: i32 = 1001;
	const PROP_SERIAL: i32 = 1002;
	const PROP_DONGLE: i32 = 1009;
	const PROP_MODE_LABEL: i32 = 4006;

	#[link(name = "kernel32")]
	extern "system" {
		fn LoadLibraryExW(name: *const u16, file: *mut c_void, flags: u32) -> *mut c_void;
		fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
	}

	/// TrackedDevicePose_t
	#[repr(C)]
	#[derive(Clone, Copy, Default)]
	struct TrackedDevicePose {
		m: [[f32; 4]; 3],
		vel: [f32; 3],
		ang: [f32; 3],
		result: i32,
		valid: u8,
		connected: u8,
	}

	/// vr::IVRClientCore's vtable (ivrclientcore.h, IVRClientCore_003)
	#[repr(C)]
	struct ClientCoreVtbl {
		init: unsafe extern "system" fn(*mut ClientCore, i32, *const c_char) -> i32,
		cleanup: unsafe extern "system" fn(*mut ClientCore),
		is_interface_version_valid: unsafe extern "system" fn(*mut ClientCore, *const c_char) -> i32,
		get_generic_interface: unsafe extern "system" fn(*mut ClientCore, *const c_char, *mut i32) -> *mut c_void,
	}

	#[repr(C)]
	struct ClientCore {
		vtbl: *const ClientCoreVtbl,
	}

	/// VR_IVRSystem_FnTable (IVRSystem_022) up to the calls used; the rest are counted, not named
	#[repr(C)]
	struct SystemFns {
		_0_10: [usize; 11],
		get_device_to_absolute_tracking_pose: unsafe extern "system" fn(i32, f32, *mut TrackedDevicePose, u32),
		_12_17: [usize; 6],
		get_controller_role_for_tracked_device_index: unsafe extern "system" fn(u32) -> i32,
		get_tracked_device_class: unsafe extern "system" fn(u32) -> i32,
		_20_26: [usize; 7],
		get_string_tracked_device_property: unsafe extern "system" fn(u32, i32, *mut c_char, u32, *mut i32) -> u32,
	}

	type Factory = unsafe extern "C" fn(*const c_char, *mut i32) -> *mut c_void;

	/// SteamVR's client, started as a background app
	pub struct Client {
		core: *mut ClientCore,
		system: *const SystemFns,
	}

	// The client core and the function table are SteamVR's, made for calls from any thread
	unsafe impl Send for Client {}

	impl Client {
		/// The client, or why not (SteamVR not installed, or not running: the error number is OpenVR's EVRInitError)
		pub fn connect(paths: &Paths) -> Result<Client, String> {
			let runtime = paths.runtime.as_ref().ok_or("SteamVR is not installed (no OpenVR path registry)")?;
			let dll = runtime.join("bin").join("vrclient_x64.dll");
			let wide: Vec<u16> = dll.as_os_str().encode_wide().chain(Some(0)).collect();
			let module = unsafe { LoadLibraryExW(wide.as_ptr(), std::ptr::null_mut(), LOAD_WITH_ALTERED_SEARCH_PATH) };
			if module.is_null() {
				return Err(format!("cannot load {}", dll.display()));
			}
			let factory = unsafe { GetProcAddress(module, c"VRClientCoreFactory".as_ptr()) };
			if factory.is_null() {
				return Err("SteamVR's client has no VRClientCoreFactory".into());
			}
			let factory: Factory = unsafe { std::mem::transmute(factory) };
			let mut rc = 0;
			let core = unsafe { factory(c"IVRClientCore_003".as_ptr(), &mut rc) } as *mut ClientCore;
			if core.is_null() {
				return Err(format!("SteamVR's client core is not available ({rc})"));
			}
			let err = unsafe { ((*(*core).vtbl).init)(core, APP_BACKGROUND, std::ptr::null()) };
			if err != 0 {
				return Err(match err {
					121 => "SteamVR is not running".to_string(),
					e => format!("SteamVR refused the connection (error {e})"),
				});
			}
			let mut e = 0;
			let name = CString::new("FnTable:IVRSystem_022").expect("no NUL");
			let system = unsafe { ((*(*core).vtbl).get_generic_interface)(core, name.as_ptr(), &mut e) };
			if system.is_null() || e != 0 {
				unsafe { ((*(*core).vtbl).cleanup)(core) };
				return Err(format!("SteamVR has no IVRSystem_022 ({e})"));
			}
			Ok(Client { core, system: system as *const SystemFns })
		}

		fn string(&self, i: u32, prop: i32) -> String {
			let mut buf = [0 as c_char; 256];
			let mut e = 0;
			let n = unsafe {
				((*self.system).get_string_tracked_device_property)(i, prop, buf.as_mut_ptr(), buf.len() as u32, &mut e)
			};
			if e != 0 || n == 0 {
				return String::new();
			}
			let bytes: Vec<u8> = buf.iter().take_while(|&&c| c != 0).map(|&c| c as u8).collect();
			String::from_utf8_lossy(&bytes).into_owned()
		}

		/// SteamVR's devices now, by index (None: no device there)
		pub fn devices(&self, paths: &Paths, settings: &str) -> Vec<Option<SteamDevice>> {
			(0..MAX_DEVICES as u32)
				.map(|i| {
					let class = unsafe { ((*self.system).get_tracked_device_class)(i) };
					if class == 0 {
						return None;
					}
					let serial = self.string(i, PROP_SERIAL);
					let role = match class {
						CLASS_CONTROLLER => {
							match unsafe { ((*self.system).get_controller_role_for_tracked_device_index)(i) } {
								1 => "LeftHand".to_string(),
								2 => "RightHand".to_string(),
								_ => String::new(),
							}
						}
						CLASS_TRACKER => paths.tracker_role(settings, &serial).unwrap_or_default(),
						CLASS_HMD => "Head".to_string(),
						_ => String::new(),
					};
					Some(SteamDevice {
						class: match class {
							CLASS_HMD => "hmd",
							CLASS_CONTROLLER => "controller",
							CLASS_TRACKER => "tracker",
							CLASS_STATION => "station",
							_ => "other",
						}
						.to_string(),
						role,
						dongle: self.string(i, PROP_DONGLE),
						model: self.string(i, PROP_MODEL),
						system: self.string(i, PROP_TRACKING_SYSTEM),
						mode: if class == CLASS_STATION { self.string(i, PROP_MODE_LABEL) } else { String::new() },
						serial,
					})
				})
				.collect()
		}

		/// The poses SteamVR shows now, by index, stamped `t_us`
		pub fn poses(&self, t_us: u64) -> [Pose; MAX_DEVICES] {
			let mut raw = [TrackedDevicePose::default(); MAX_DEVICES];
			unsafe {
				((*self.system).get_device_to_absolute_tracking_pose)(
					UNIVERSE_STANDING,
					0.0,
					raw.as_mut_ptr(),
					MAX_DEVICES as u32,
				)
			};
			let mut out = [Pose::default(); MAX_DEVICES];
			for (o, r) in out.iter_mut().zip(raw.iter()) {
				*o = Pose {
					t_us,
					result: r.result,
					valid: r.valid != 0,
					connected: r.connected != 0,
					pos: [r.m[0][3] as f64, r.m[1][3] as f64, r.m[2][3] as f64],
					rot: quaternion(&r.m),
					vel: r.vel.map(|v| v as f64),
					ang: r.ang.map(|v| v as f64),
				};
			}
			out
		}
	}

	impl Drop for Client {
		fn drop(&mut self) {
			unsafe { ((*(*self.core).vtbl).cleanup)(self.core) };
		}
	}

	#[test]
	fn the_pose_struct_is_openvrs_size() {
		assert_eq!(std::mem::size_of::<TrackedDevicePose>(), 80);
		assert_eq!(std::mem::size_of::<SystemFns>(), 28 * 8);
	}
}

/// What SteamVR's devices are, for the session's devices.txt (base stations included, for their channels)
pub fn devices_text(devices: &[SteamDevice]) -> String {
	let mut s = String::from(crate::session::DEVICES_HEADER);
	s.push('\n');
	for d in devices {
		s.push_str(&d.line());
		s.push('\n');
	}
	s
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn tracker_roles_are_found_by_serial() {
		let s = r#"{ "trackers" : { "/devices/htc/vive_trackerLHR-2D779713" : "TrackerRole_Waist", "/devices/htc/vive_trackerLHR-1B80952C": "TrackerRole_LeftFoot" } }"#;
		let p = Paths::default();
		assert_eq!(p.tracker_role(s, "LHR-1B80952C").as_deref(), Some("TrackerRole_LeftFoot"));
		assert_eq!(p.tracker_role(s, "LHR-2D779713").as_deref(), Some("TrackerRole_Waist"));
		assert_eq!(p.tracker_role(s, "LHR-00000000"), None);
	}

	#[test]
	fn a_matrix_becomes_its_quaternion() {
		// 90 degrees about y
		let m = [[0.0, 0.0, 1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]];
		let q = quaternion(&m);
		let h = std::f64::consts::FRAC_1_SQRT_2;
		assert!((q[0] - h).abs() < 1e-6 && (q[2] - h).abs() < 1e-6, "{q:?}");
	}
}
