//! The stand-in vrserver (fake_vrserver.cpp's, in Rust): the interfaces a server driver is given, each an object whose
//! first word is its vtable, the slots in openvr_driver.h's order (OpenVR 2.15.6). They are written here from the
//! header, not taken from the driver's own OpenVR layer, so the slots the driver calls through are checked too; the
//! plain structs (DriverPose_t and the others) are the layer's, whose layouts it asserts against the C++ compilers'.
//!
//! Offered: the driver context, IVRServerDriverHost_006, _005 and _004 (three vtables, as SteamVR has), IVRProperties
//! and IVRDriverLog; and IVRSettings, IVRDriverManager and IVRResources, which vrserver gives every driver and this one
//! never uses: any call into them, or into a slot of the others the driver has no business calling, aborts the test
//! (fake_vrserver's trap). The vtables are on a page of their own, read-only as vrserver's are, so the driver's hook
//! changes the page's protection as it does in SteamVR. Every call is recorded with the session time it came at.

use std::cell::Cell;
use std::collections::BTreeMap;
use std::ffi::{c_char, c_int, c_void, CStr};
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering::SeqCst};
use std::sync::{Mutex, OnceLock};

use driver_sally::openvr::{DriverPose_t, PropertyRead_t, TrackedDevicePose_t};
use sally_math::math3d::{qmul, qrot, Q4, V3};

/// TrackedDevicePoseUpdated(this, id, &pose, size)
pub type PoseFn = unsafe extern "C" fn(*mut c_void, u32, *const DriverPose_t, u32);

/// One pose vrserver got
#[derive(Clone, Copy, Debug)]
pub struct Call {
	pub t: f64,
	/// the interface it came through: 0 = IVRServerDriverHost_006, 1 = _005, 2 = _004
	pub iface: usize,
	pub id: u32,
	pub size: u32,
	pub pose: DriverPose_t,
	/// the pose another driver was handing over on the thread it came on (an index into the session's fed poses);
	/// None for the driver's own (a push)
	pub fed: Option<usize>,
}

thread_local! {
	/// the fed pose (an index) the call in progress on this thread is handing vrserver, if any
	static FEEDING: Cell<Option<usize>> = const { Cell::new(None) };
}

/// The objects, in the vtable page's order: the context, the three hosts (_006, _005, _004), the properties, the log,
/// and the three the driver never uses
const CONTEXT: usize = 0;
const HOSTS: usize = 1;
const SETTINGS: usize = 6;
const DRIVER_MANAGER: usize = 7;
const RESOURCES: usize = 8;
const OBJECTS: usize = 9;
/// Each object's slots
const SLOTS: [usize; OBJECTS] = [2, 12, 12, 12, 4, 1, 16, 16, 16];
/// The versions GetGenericInterface hands each object out for
const VERSIONS: [&str; OBJECTS] = [
	"",
	"IVRServerDriverHost_006",
	"IVRServerDriverHost_005",
	"IVRServerDriverHost_004",
	"IVRProperties_001",
	"IVRDriverLog_001",
	"IVRSettings_003",
	"IVRDriverManager_001",
	"IVRResources_001",
];

/// An interface object: its vtable, then the server it is part of
#[repr(C)]
struct Object {
	vtbl: *const *const c_void,
	server: *const Server,
}

pub struct Server {
	/// the session's clock origin (steady_clock ns)
	t0: i64,
	objects: OnceLock<usize>,
	pub log: Mutex<Vec<(f64, String)>>,
	pub calls: Mutex<Vec<Call>>,
	/// id -> (tracking system, serial); a device without an entry has properties that do not read
	pub props: Mutex<BTreeMap<u32, (&'static str, &'static str)>>,
	/// id -> the dongle Valve's driver names for it (Prop_ConnectedWirelessDongle_String); none: the property does not
	/// read, as for a headset or a base station
	pub dongles: Mutex<BTreeMap<u32, &'static str>>,
	/// property container -> ReadPropertyBatch calls for it
	pub reads: Mutex<BTreeMap<u64, u32>>,
	/// the versions the driver asked GetGenericInterface for
	pub asked: Mutex<Vec<String>>,
	/// TrackedDeviceAdded and WritePropertyBatch calls (the driver adds no devices and writes no properties)
	pub added: AtomicU32,
	pub writes: AtomicU32,
	/// id -> Prop_DeviceClass_Int32 (a device without one has none that reads)
	pub classes: Mutex<BTreeMap<u32, i32>>,
	/// GetRawTrackedDevicePoses' answer: the last pose of each device, as SteamVR has it (world)
	raw: Mutex<[TrackedDevicePose_t; 64]>,
	/// another driver's hook on _006 (the test plays one): its calls, and the slot's function it hands them on to
	pub other_calls: AtomicU32,
	other_next: AtomicUsize,
}

/// The objects' server, and which object this is
///
/// # Safety
/// `this` is one of a server's objects
unsafe fn object(this: *mut c_void) -> (&'static Server, usize) {
	// SAFETY: an object's second word is its server, which lives as long as the process (leaked)
	let server: &'static Server = unsafe { &*(*(this as *const Object)).server };
	let base = *server.objects.get().expect("the objects");
	(server, (this as usize - base) / std::mem::size_of::<Object>())
}

unsafe extern "C" fn get_generic_interface(this: *mut c_void, version: *const c_char, err: *mut i32) -> *mut c_void {
	// SAFETY: vrserver's objects, a version string the driver passes NUL-terminated, and its place for the error
	unsafe {
		let (s, _) = object(this);
		let v = CStr::from_ptr(version).to_string_lossy().into_owned();
		let found = VERSIONS.iter().position(|&x| !x.is_empty() && x == v);
		s.asked.lock().unwrap().push(v);
		if !err.is_null() {
			*err = if found.is_some() { 0 } else { 105 }; // VRInitError_None, VRInitError_Init_InterfaceNotFound
		}
		match found {
			Some(k) => s.object(k),
			None => std::ptr::null_mut(),
		}
	}
}

unsafe extern "C" fn get_driver_handle(_this: *mut c_void) -> u64 {
	1
}

unsafe extern "C" fn tracked_device_added(
	this: *mut c_void, _serial: *const c_char, _class: c_int, _d: *mut c_void,
) -> bool {
	// SAFETY: one of the server's objects
	unsafe {
		let (s, _) = object(this);
		s.added.fetch_add(1, SeqCst);
	}
	// counted and refused: the driver has no device of its own to add
	false
}

unsafe extern "C" fn tracked_device_pose_updated(this: *mut c_void, id: u32, pose: *const DriverPose_t, size: u32) {
	// SAFETY: one of the server's objects, and the caller's pose (of `size` bytes, read only when that is a pose's)
	unsafe {
		let (s, k) = object(this);
		let t = s.now();
		let p = if size as usize == std::mem::size_of::<DriverPose_t>() { *pose } else { DriverPose_t::default() };
		s.calls.lock().unwrap().push(Call { t, iface: k - HOSTS, id, size, pose: p, fed: FEEDING.get() });
		if (id as usize) < 64 {
			s.raw.lock().unwrap()[id as usize] = shown(&p);
		}
	}
}

/// What SteamVR has for a device after a pose: its head's pose in the world (world-from-driver * driver pose *
/// driver-from-head, fake_vrserver's), its velocity in the world, its flags
fn shown(p: &DriverPose_t) -> TrackedDevicePose_t {
	let w = world_head(p);
	let wq = quat(&p.qWorldFromDriverRotation);
	let q = qmul(qmul(wq, quat(&p.qRotation)), quat(&p.qDriverFromHeadRotation));
	let r = matrix(q);
	let v = qrot(wq, V3::new(p.vecVelocity[0], p.vecVelocity[1], p.vecVelocity[2]));
	let mut m = [[0.0f32; 4]; 3];
	for i in 0..3 {
		for j in 0..3 {
			m[i][j] = r[i][j] as f32;
		}
		m[i][3] = w.get(i) as f32;
	}
	TrackedDevicePose_t {
		mDeviceToAbsoluteTracking: driver_sally::openvr::HmdMatrix34_t { m },
		vVelocity: driver_sally::openvr::HmdVector3_t { v: [v.x as f32, v.y as f32, v.z as f32] },
		vAngularVelocity: driver_sally::openvr::HmdVector3_t { v: [0.0; 3] },
		eTrackingResult: p.result,
		bPoseIsValid: p.poseIsValid,
		bDeviceIsConnected: p.deviceIsConnected,
	}
}

pub fn quat(q: &driver_sally::openvr::HmdQuaternion_t) -> Q4 {
	Q4::new(q.w, q.x, q.y, q.z)
}

pub fn v3(a: &[f64; 3]) -> V3 {
	V3::new(a[0], a[1], a[2])
}

/// The head's position in the world: world-from-driver * driver pose * driver-from-head
pub fn world_head(p: &DriverPose_t) -> V3 {
	let head = v3(&p.vecPosition) + qrot(quat(&p.qRotation), v3(&p.vecDriverFromHeadTranslation));
	qrot(quat(&p.qWorldFromDriverRotation), head) + v3(&p.vecWorldFromDriverTranslation)
}

fn matrix(q: Q4) -> [[f64; 3]; 3] {
	let (w, x, y, z) = (q.w, q.x, q.y, q.z);
	[
		[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
		[2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
		[2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
	]
}

unsafe extern "C" fn get_raw_tracked_device_poses(this: *mut c_void, t: f32, out: *mut TrackedDevicePose_t, n: u32) {
	// SAFETY: one of the server's objects, and the driver's array of n poses
	unsafe {
		let (s, _) = object(this);
		let raw = s.raw.lock().unwrap();
		for i in 0..n as usize {
			let mut r = raw.get(i).copied().unwrap_or_default();
			// SteamVR's look ahead: the position moved by the velocity
			for k in 0..3 {
				r.mDeviceToAbsoluteTracking.m[k][3] += r.vVelocity.v[k] * t;
			}
			*out.add(i) = r;
		}
	}
}

unsafe extern "C" fn read_property_batch(this: *mut c_void, c: u64, batch: *mut PropertyRead_t, n: u32) -> i32 {
	// SAFETY: one of the server's objects, and the driver's batch of n reads, each with its buffer and its size
	unsafe {
		let (s, _) = object(this);
		*s.reads.lock().unwrap().entry(c).or_default() += 1;
		let props = s.props.lock().unwrap();
		let dongles = s.dongles.lock().unwrap();
		let classes = s.classes.lock().unwrap();
		for i in 0..n as usize {
			let r = &mut *batch.add(i);
			let id = c.checked_sub(1000).map(|id| id as u32);
			let entry = id.and_then(|id| props.get(&id));
			let dongle = id.and_then(|id| dongles.get(&id));
			// Prop_DeviceClass_Int32
			if r.prop == 1029 {
				match id.and_then(|id| classes.get(&id)) {
					Some(&class) if r.unBufferSize >= 4 => {
						std::ptr::copy_nonoverlapping(class.to_ne_bytes().as_ptr(), r.pvBuffer.cast::<u8>(), 4);
						(r.eError, r.unTag, r.unRequiredBufferSize) = (0, 2, 4); // k_unInt32PropertyTag
					}
					_ => (r.eError, r.unTag, r.unRequiredBufferSize) = (4, 0, 0),
				}
				continue;
			}
			let value: &str = match (entry, dongle, r.prop) {
				(Some((system, _)), _, 1000) => system,  // Prop_TrackingSystemName_String
				(Some((_, serial)), _, 1002) => serial,  // Prop_SerialNumber_String
				(Some(_), Some(dongle), 1009) => dongle, // Prop_ConnectedWirelessDongle_String
				_ => {
					r.eError = 4; // TrackedProp_UnknownProperty
					r.unTag = 0;
					r.unRequiredBufferSize = 0;
					continue;
				}
			};
			r.unTag = 5; // k_unStringPropertyTag
			r.unRequiredBufferSize = value.len() as u32 + 1;
			if (r.unBufferSize as usize) < value.len() + 1 {
				r.eError = 3; // TrackedProp_BufferTooSmall
				continue;
			}
			std::ptr::copy_nonoverlapping(value.as_ptr(), r.pvBuffer.cast::<u8>(), value.len());
			*r.pvBuffer.cast::<u8>().add(value.len()) = 0;
			r.eError = 0;
		}
		0
	}
}

unsafe extern "C" fn write_property_batch(this: *mut c_void, _c: u64, _batch: *mut c_void, _n: u32) -> i32 {
	// SAFETY: one of the server's objects
	unsafe {
		let (s, _) = object(this);
		s.writes.fetch_add(1, SeqCst);
	}
	// counted and refused (TrackedProp_NotYetAvailable): the driver writes no property
	9
}

unsafe extern "C" fn tracked_device_to_property_container(_this: *mut c_void, id: u32) -> u64 {
	1000 + id as u64
}

unsafe extern "C" fn log(this: *mut c_void, msg: *const c_char) {
	// SAFETY: one of the server's objects, and the driver's NUL-terminated message
	unsafe {
		let (s, _) = object(this);
		let m = CStr::from_ptr(msg).to_string_lossy().trim_end_matches(['\n', '\r']).to_string();
		s.log.lock().unwrap().push((s.now(), m));
	}
}

/// What a trap stands for, by its number
const TRAPS: [&str; 13] = [
	"IVRServerDriverHost::VsyncEvent",
	"IVRServerDriverHost::VendorSpecificEvent",
	"IVRServerDriverHost::IsExiting",
	"IVRServerDriverHost::PollNextEvent",
	"IVRServerDriverHost::RequestRestart",
	"IVRServerDriverHost::GetFrameTimings",
	"IVRServerDriverHost::SetDisplayEyeToHead",
	"IVRServerDriverHost::SetDisplayProjectionRaw",
	"IVRServerDriverHost::SetRecommendedRenderTargetSize",
	"IVRProperties::GetPropErrorNameFromEnum",
	"IVRSettings",
	"IVRDriverManager",
	"IVRResources",
];

unsafe extern "C" fn trap<const N: usize>(_this: *mut c_void) {
	println!("TRAP: the driver called {}, which it should not use", TRAPS[N]);
	std::process::abort();
}

/// Another driver's hook on _006's TrackedDevicePoseUpdated (installed over the driver's): counts, hands on
unsafe extern "C" fn other_driver_hook(this: *mut c_void, id: u32, pose: *const DriverPose_t, size: u32) {
	// SAFETY: one of the server's objects; the next function is the slot's as the hook found it, of its type
	unsafe {
		let (s, _) = object(this);
		s.other_calls.fetch_add(1, SeqCst);
		let next: PoseFn = std::mem::transmute::<usize, PoseFn>(s.other_next.load(SeqCst));
		next(this, id, pose, size)
	}
}

extern "C" {
	fn mmap(addr: *mut c_void, len: usize, prot: c_int, flags: c_int, fd: c_int, off: i64) -> *mut c_void;
	fn mprotect(addr: *mut c_void, len: usize, prot: c_int) -> c_int;
}
const PROT_READ: c_int = 1;
const PROT_WRITE: c_int = 2;
const MAP_PRIVATE: c_int = 2;
const MAP_ANONYMOUS: c_int = 0x20;
const PAGE: usize = 4096;

/// A vtable slot written as a driver hooks one: the page made writable, the slot written, the page read-only again
pub fn write_slot(slot: *mut *const c_void, value: *const c_void) {
	let page = (slot as usize & !(PAGE - 1)) as *mut c_void;
	// SAFETY: the slot is on the server's vtable page, which is mapped; written only while writable
	unsafe {
		assert_eq!(mprotect(page, PAGE, PROT_READ | PROT_WRITE), 0);
		slot.write_volatile(value);
		assert_eq!(mprotect(page, PAGE, PROT_READ), 0);
	}
}

impl Server {
	/// A server, its vtables on a fresh page, read-only once filled; it lives as long as the process
	pub fn new(t0: i64) -> &'static Server {
		let s: &'static Server = Box::leak(Box::new(Server {
			t0,
			objects: OnceLock::new(),
			log: Mutex::new(Vec::new()),
			calls: Mutex::new(Vec::new()),
			props: Mutex::new(BTreeMap::new()),
			dongles: Mutex::new(BTreeMap::new()),
			reads: Mutex::new(BTreeMap::new()),
			asked: Mutex::new(Vec::new()),
			added: AtomicU32::new(0),
			writes: AtomicU32::new(0),
			classes: Mutex::new(BTreeMap::new()),
			raw: Mutex::new([TrackedDevicePose_t::default(); 64]),
			other_calls: AtomicU32::new(0),
			other_next: AtomicUsize::new(0),
		}));
		// SAFETY: an anonymous private mapping of one page, written before it is made read-only
		let page = unsafe {
			let p = mmap(std::ptr::null_mut(), PAGE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
			assert!(p as isize != -1, "mmap");
			p.cast::<*const c_void>()
		};
		// IVRServerDriverHost: TrackedDeviceAdded, TrackedDevicePoseUpdated, VsyncEvent, VendorSpecificEvent,
		// IsExiting, PollNextEvent, GetRawTrackedDevicePoses, RequestRestart, GetFrameTimings, SetDisplayEyeToHead,
		// SetDisplayProjectionRaw, SetRecommendedRenderTargetSize
		let host: [*const (); 12] = [
			tracked_device_added as *const (),
			tracked_device_pose_updated as *const (),
			trap::<0> as *const (),
			trap::<1> as *const (),
			trap::<2> as *const (),
			trap::<3> as *const (),
			get_raw_tracked_device_poses as *const (),
			trap::<4> as *const (),
			trap::<5> as *const (),
			trap::<6> as *const (),
			trap::<7> as *const (),
			trap::<8> as *const (),
		];
		let tables: [Vec<*const ()>; OBJECTS] = [
			// IVRDriverContext: GetGenericInterface, GetDriverHandle
			vec![get_generic_interface as *const (), get_driver_handle as *const ()],
			host.to_vec(),
			host.to_vec(),
			host.to_vec(),
			// IVRProperties: ReadPropertyBatch, WritePropertyBatch, GetPropErrorNameFromEnum,
			// TrackedDeviceToPropertyContainer
			vec![
				read_property_batch as *const (),
				write_property_batch as *const (),
				trap::<9> as *const (),
				tracked_device_to_property_container as *const (),
			],
			// IVRDriverLog: Log
			vec![log as *const ()],
			vec![trap::<10> as *const (); SLOTS[SETTINGS]],
			vec![trap::<11> as *const (); SLOTS[DRIVER_MANAGER]],
			vec![trap::<12> as *const (); SLOTS[RESOURCES]],
		];
		let mut at = 0;
		let mut vtbls = [std::ptr::null(); OBJECTS];
		for (k, table) in tables.iter().enumerate() {
			assert_eq!(table.len(), SLOTS[k]);
			// SAFETY: the page holds every table (91 slots of 512)
			unsafe {
				vtbls[k] = page.add(at) as *const *const c_void;
				for (i, &f) in table.iter().enumerate() {
					page.add(at + i).write(f.cast());
				}
			}
			at += table.len();
		}
		// SAFETY: the page, filled
		assert_eq!(unsafe { mprotect(page.cast(), PAGE, PROT_READ) }, 0);
		let objects: &'static [Object; OBJECTS] =
			Box::leak(Box::new(std::array::from_fn(|k| Object { vtbl: vtbls[k], server: s })));
		assert!(s.objects.set(objects.as_ptr() as usize).is_ok(), "the objects, once");
		s
	}

	/// Seconds since the session's clock origin, on the driver's clock (steady_clock)
	pub fn now(&self) -> f64 {
		(sally_os::chrono::steady_clock_now() - self.t0) as f64 / 1e9
	}

	fn object(&self, k: usize) -> *mut c_void {
		(*self.objects.get().expect("the objects") + k * std::mem::size_of::<Object>()) as *mut c_void
	}

	/// The driver context Init gets
	pub fn context(&self) -> *mut c_void {
		self.object(CONTEXT)
	}

	/// The IVRServerDriverHost of interface `iface` (0 = _006, 1 = _005, 2 = _004)
	pub fn host(&self, iface: usize) -> *mut c_void {
		self.object(HOSTS + iface)
	}

	/// Its TrackedDevicePoseUpdated slot (the hook's)
	pub fn pose_slot(&self, iface: usize) -> *mut *const c_void {
		// SAFETY: a host object's vtable has 12 slots; slot 1 is TrackedDevicePoseUpdated
		unsafe { (*(self.host(iface) as *const Object)).vtbl.add(1) as *mut *const c_void }
	}

	/// What the slot holds now
	pub fn slot_value(&self, iface: usize) -> usize {
		// SAFETY: the slot is on the mapped vtable page
		unsafe { self.pose_slot(iface).read_volatile() as usize }
	}

	/// The server's own TrackedDevicePoseUpdated (what the slots held before the driver hooked them)
	pub fn own_pose_fn() -> usize {
		tracked_device_pose_updated as *const () as usize
	}

	/// Another driver hooks _006's slot over whatever holds it (the hook hands calls on to that): its hook's function
	pub fn other_driver_hooks(&self) -> usize {
		let slot = self.pose_slot(0);
		self.other_next.store(self.slot_value(0), SeqCst);
		write_slot(slot, other_driver_hook as *const c_void);
		other_driver_hook as *const () as usize
	}

	/// A driver's pose for device `id`, through interface `iface`'s vtable as another driver calls it (the hook in it
	/// sees it first), `fed` the index of the pose in the session's record
	pub fn report(&self, iface: usize, id: u32, pose: &DriverPose_t, fed: Option<usize>) {
		// SAFETY: the slot holds a function of TrackedDevicePoseUpdated's type (the server's, or a hook installed
		// over it), called with its interface object and a pose that lives through the call
		unsafe {
			let f = std::mem::transmute::<*const c_void, PoseFn>(self.pose_slot(iface).read_volatile());
			FEEDING.set(fed);
			f(self.host(iface), id, pose, std::mem::size_of::<DriverPose_t>() as u32);
			FEEDING.set(None);
		}
	}
}
