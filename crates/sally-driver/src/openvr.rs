//! OpenVR's driver interface as the driver uses it, from openvr_driver.h of OpenVR 2.15.6 (the version the C++ driver
//! was built against): the structs as `#[repr(C)]` (their C++ names and fields, a C++ bool a byte, an enum an i32), the
//! interface classes as vtables of `extern "C"` functions, and the server driver context the header keeps for a
//! driver (COpenVRDriverContext: the interfaces InitServer fetches, VRDriverLog, VRProperties with its string read,
//! VRServerDriverHost).
//!
//! OpenVR's driver interfaces are pure virtual C++ classes without overloads or virtual destructors: an object starts
//! with a pointer to its vtable, which has one slot per method in declaration order, each called with `this` first.
//! That is the same for MSVC (vrserver) and the Itanium ABI llvm-mingw builds with, and on Linux, and `extern "C"` is
//! the platform's calling convention for them (Win64, SysV).

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, clippy::upper_case_acronyms)]

use std::ffi::{c_char, c_void, CStr};
use std::ptr::null_mut;
use std::sync::atomic::{AtomicPtr, Ordering};

pub type EVRInitError = i32;
pub const VRInitError_None: EVRInitError = 0;
pub const VRInitError_Init_InterfaceNotFound: EVRInitError = 105;
pub const VRInitError_Driver_Failed: EVRInitError = 200;

pub type ETrackingResult = i32;
pub const TrackingResult_Running_OK: ETrackingResult = 200;
pub const TrackingResult_Running_OutOfRange: ETrackingResult = 201;

pub type ETrackedPropertyError = i32;
pub const TrackedProp_Success: ETrackedPropertyError = 0;
pub const TrackedProp_WrongDataType: ETrackedPropertyError = 1;
pub const TrackedProp_BufferTooSmall: ETrackedPropertyError = 3;
pub const TrackedProp_CouldNotContactServer: ETrackedPropertyError = 6;

pub type ETrackedDeviceProperty = i32;
pub const Prop_TrackingSystemName_String: ETrackedDeviceProperty = 1000;
pub const Prop_SerialNumber_String: ETrackedDeviceProperty = 1002;
/// the dongle a wireless device talks through, as Valve's lighthouse driver names it (its USB serial, "00A4AE2B50-RYB")
pub const Prop_ConnectedWirelessDongle_String: ETrackedDeviceProperty = 1009;
pub const Prop_DeviceClass_Int32: ETrackedDeviceProperty = 1029;

pub type ETrackedDeviceClass = i32;

pub type PropertyTypeTag_t = u32;
pub const k_unStringPropertyTag: PropertyTypeTag_t = 5;
pub type PropertyContainerHandle_t = u64;
pub type DriverHandle_t = u64;

pub const k_unMaxTrackedDeviceCount: u32 = 64;
/// the headset's device index
pub const k_unTrackedDeviceIndex_Hmd: u32 = 0;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdQuaternion_t {
	pub w: f64,
	pub x: f64,
	pub y: f64,
	pub z: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdMatrix34_t {
	pub m: [[f32; 4]; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdVector3_t {
	pub v: [f32; 3],
}

/// What a driver reports for a device (the header's comments: driver space, metres, seconds)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DriverPose_t {
	pub poseTimeOffset: f64,
	pub qWorldFromDriverRotation: HmdQuaternion_t,
	pub vecWorldFromDriverTranslation: [f64; 3],
	pub qDriverFromHeadRotation: HmdQuaternion_t,
	pub vecDriverFromHeadTranslation: [f64; 3],
	pub vecPosition: [f64; 3],
	pub vecVelocity: [f64; 3],
	pub vecAcceleration: [f64; 3],
	pub qRotation: HmdQuaternion_t,
	pub vecAngularVelocity: [f64; 3],
	pub vecAngularAcceleration: [f64; 3],
	pub result: ETrackingResult,
	pub poseIsValid: u8,
	pub willDriftInYaw: u8,
	pub shouldApplyHeadModel: u8,
	pub deviceIsConnected: u8,
}

/// What SteamVR has for a device (GetRawTrackedDevicePoses)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrackedDevicePose_t {
	pub mDeviceToAbsoluteTracking: HmdMatrix34_t,
	pub vVelocity: HmdVector3_t,
	pub vAngularVelocity: HmdVector3_t,
	pub eTrackingResult: ETrackingResult,
	pub bPoseIsValid: u8,
	pub bDeviceIsConnected: u8,
}

#[repr(C)]
pub struct PropertyRead_t {
	pub prop: ETrackedDeviceProperty,
	pub pvBuffer: *mut c_void,
	pub unBufferSize: u32,
	pub unTag: PropertyTypeTag_t,
	pub unRequiredBufferSize: u32,
	pub eError: ETrackedPropertyError,
}

// the layouts the C++ compilers give them (g++ 13 on Linux, llvm-mingw's clang for Windows: the same)
const _: () = {
	use std::mem::{offset_of, size_of};
	assert!(size_of::<HmdQuaternion_t>() == 32);
	assert!(size_of::<DriverPose_t>() == 280);
	assert!(offset_of!(DriverPose_t, qWorldFromDriverRotation) == 8);
	assert!(offset_of!(DriverPose_t, vecWorldFromDriverTranslation) == 40);
	assert!(offset_of!(DriverPose_t, qDriverFromHeadRotation) == 64);
	assert!(offset_of!(DriverPose_t, vecDriverFromHeadTranslation) == 96);
	assert!(offset_of!(DriverPose_t, vecPosition) == 120);
	assert!(offset_of!(DriverPose_t, vecVelocity) == 144);
	assert!(offset_of!(DriverPose_t, vecAcceleration) == 168);
	assert!(offset_of!(DriverPose_t, qRotation) == 192);
	assert!(offset_of!(DriverPose_t, vecAngularVelocity) == 224);
	assert!(offset_of!(DriverPose_t, vecAngularAcceleration) == 248);
	assert!(offset_of!(DriverPose_t, result) == 272);
	assert!(offset_of!(DriverPose_t, poseIsValid) == 276);
	assert!(offset_of!(DriverPose_t, deviceIsConnected) == 279);
	assert!(size_of::<TrackedDevicePose_t>() == 80);
	assert!(offset_of!(TrackedDevicePose_t, vVelocity) == 48);
	assert!(offset_of!(TrackedDevicePose_t, vAngularVelocity) == 60);
	assert!(offset_of!(TrackedDevicePose_t, eTrackingResult) == 72);
	assert!(offset_of!(TrackedDevicePose_t, bPoseIsValid) == 76);
	assert!(offset_of!(TrackedDevicePose_t, bDeviceIsConnected) == 77);
	assert!(size_of::<PropertyRead_t>() == 32);
	assert!(offset_of!(PropertyRead_t, pvBuffer) == 8);
	assert!(offset_of!(PropertyRead_t, unBufferSize) == 16);
	assert!(offset_of!(PropertyRead_t, unTag) == 20);
	assert!(offset_of!(PropertyRead_t, unRequiredBufferSize) == 24);
	assert!(offset_of!(PropertyRead_t, eError) == 28);
};

/// IVRServerDriverHost::TrackedDevicePoseUpdated's slot, as the hook replaces it: (this, id, &pose, size)
pub type PoseUpdatedFn = unsafe extern "C" fn(*mut c_void, u32, *const DriverPose_t, u32);

/// An interface object: a pointer to its vtable first
#[repr(C)]
pub struct Interface<V> {
	pub vtbl: *const V,
}

#[repr(C)]
pub struct IVRDriverContextVtbl {
	pub GetGenericInterface:
		unsafe extern "C" fn(*mut IVRDriverContext, *const c_char, *mut EVRInitError) -> *mut c_void,
	pub GetDriverHandle: unsafe extern "C" fn(*mut IVRDriverContext) -> DriverHandle_t,
}
pub type IVRDriverContext = Interface<IVRDriverContextVtbl>;

#[repr(C)]
pub struct IVRDriverLogVtbl {
	pub Log: unsafe extern "C" fn(*mut IVRDriverLog, *const c_char),
}
pub type IVRDriverLog = Interface<IVRDriverLogVtbl>;

#[repr(C)]
pub struct IVRPropertiesVtbl {
	pub ReadPropertyBatch: unsafe extern "C" fn(
		*mut IVRProperties,
		PropertyContainerHandle_t,
		*mut PropertyRead_t,
		u32,
	) -> ETrackedPropertyError,
	pub WritePropertyBatch:
		unsafe extern "C" fn(*mut IVRProperties, PropertyContainerHandle_t, *mut c_void, u32) -> ETrackedPropertyError,
	pub GetPropErrorNameFromEnum: unsafe extern "C" fn(*mut IVRProperties, ETrackedPropertyError) -> *const c_char,
	pub TrackedDeviceToPropertyContainer: unsafe extern "C" fn(*mut IVRProperties, u32) -> PropertyContainerHandle_t,
}
pub type IVRProperties = Interface<IVRPropertiesVtbl>;

/// IVRServerDriverHost::TrackedDeviceAdded(this, serial, class, device)
pub type DeviceAddedFn =
	unsafe extern "C" fn(*mut IVRServerDriverHost, *const c_char, ETrackedDeviceClass, *mut c_void) -> bool;

/// IVRServerDriverHost_006 (the slots the driver does not call are only placeholders here)
#[repr(C)]
pub struct IVRServerDriverHostVtbl {
	pub TrackedDeviceAdded: DeviceAddedFn,
	pub TrackedDevicePoseUpdated: PoseUpdatedFn,
	pub VsyncEvent: *const c_void,
	pub VendorSpecificEvent: *const c_void,
	pub IsExiting: *const c_void,
	pub PollNextEvent: *const c_void,
	pub GetRawTrackedDevicePoses: unsafe extern "C" fn(*mut IVRServerDriverHost, f32, *mut TrackedDevicePose_t, u32),
	pub RequestRestart: *const c_void,
	pub GetFrameTimings: *const c_void,
	pub SetDisplayEyeToHead: *const c_void,
	pub SetDisplayProjectionRaw: *const c_void,
	pub SetRecommendedRenderTargetSize: *const c_void,
}
pub type IVRServerDriverHost = Interface<IVRServerDriverHostVtbl>;

/// The interface the driver implements (IServerTrackedDeviceProvider_004)
#[repr(C)]
pub struct IServerTrackedDeviceProviderVtbl {
	pub Init: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider, *mut IVRDriverContext) -> EVRInitError,
	pub Cleanup: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider),
	pub GetInterfaceVersions: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider) -> *const *const c_char,
	pub RunFrame: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider),
	pub ShouldBlockStandbyMode: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider) -> bool,
	pub EnterStandby: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider),
	pub LeaveStandby: unsafe extern "C" fn(*mut IServerTrackedDeviceProvider),
}
pub type IServerTrackedDeviceProvider = Interface<IServerTrackedDeviceProviderVtbl>;

// SAFETY: an interface object the driver makes is a static vtable pointer that nothing writes
unsafe impl<V> Sync for Interface<V> {}

pub const IServerTrackedDeviceProvider_Version: &CStr = c"IServerTrackedDeviceProvider_004";
pub const IVRServerDriverHost_Version: &CStr = c"IVRServerDriverHost_006";
pub const IVRSettings_Version: &CStr = c"IVRSettings_003";
pub const IVRProperties_Version: &CStr = c"IVRProperties_001";
pub const IVRDriverLog_Version: &CStr = c"IVRDriverLog_001";
pub const IVRDriverManager_Version: &CStr = c"IVRDriverManager_001";
pub const IVRResources_Version: &CStr = c"IVRResources_001";

/// k_InterfaceVersions: the interface versions the driver is built against, as GetInterfaceVersions returns them
pub struct InterfaceVersions(pub [*const c_char; 13]);
// SAFETY: pointers to static C strings, never written
unsafe impl Sync for InterfaceVersions {}
pub static k_InterfaceVersions: InterfaceVersions = InterfaceVersions([
	IVRSettings_Version.as_ptr(),
	c"ITrackedDeviceServerDriver_005".as_ptr(),
	c"IVRDisplayComponent_003".as_ptr(),
	c"IVRDriverDirectModeComponent_009".as_ptr(),
	c"IVRCameraComponent_003".as_ptr(),
	IServerTrackedDeviceProvider_Version.as_ptr(),
	c"IVRWatchdogProvider_001".as_ptr(),
	c"IVRVirtualDisplay_002".as_ptr(),
	IVRDriverManager_Version.as_ptr(),
	IVRResources_Version.as_ptr(),
	c"IVRCompositorPluginProvider_001".as_ptr(),
	c"IVRIPCResourceManagerClient_003".as_ptr(),
	std::ptr::null(),
]);

/// pContext->GetGenericInterface(version, &err): the interface (null when not offered) and the error it set (the
/// value passed in when it set none)
///
/// # Safety
/// `ctx` is the IVRDriverContext vrserver handed to Init, still live
pub unsafe fn get_generic_interface(
	ctx: *mut IVRDriverContext, version: &CStr, err_in: EVRInitError,
) -> (*mut c_void, EVRInitError) {
	let mut err = err_in;
	// SAFETY: ctx is the context vrserver handed to Init (live while the driver is), whose vtable starts with
	// GetGenericInterface; the version is a NUL-terminated string and err a valid place for the error
	let p = unsafe { ((*(*ctx).vtbl).GetGenericInterface)(ctx, version.as_ptr(), &mut err) };
	(p, err)
}

/// COpenVRDriverContext and VRDriverContext(): the context Init was given and the interfaces fetched from it, each
/// on first use (InitServer fetches six at once; Clear forgets them)
pub struct DriverContext {
	context: AtomicPtr<IVRDriverContext>,
	server_driver_host: AtomicPtr<c_void>,
	settings: AtomicPtr<c_void>,
	properties: AtomicPtr<c_void>,
	driver_log: AtomicPtr<c_void>,
	driver_manager: AtomicPtr<c_void>,
	resources: AtomicPtr<c_void>,
}

pub static CONTEXT: DriverContext = DriverContext {
	context: AtomicPtr::new(null_mut()),
	server_driver_host: AtomicPtr::new(null_mut()),
	settings: AtomicPtr::new(null_mut()),
	properties: AtomicPtr::new(null_mut()),
	driver_log: AtomicPtr::new(null_mut()),
	driver_manager: AtomicPtr::new(null_mut()),
	resources: AtomicPtr::new(null_mut()),
};

impl DriverContext {
	/// The accessors' pattern: the cached interface, or the context's answer for it, kept. Without a context (after
	/// Cleanup) none: the C++ would call through a null context there and crash
	fn get(&self, cache: &AtomicPtr<c_void>, version: &CStr) -> *mut c_void {
		let p = cache.load(Ordering::Acquire);
		if !p.is_null() {
			return p;
		}
		let ctx = self.context.load(Ordering::Acquire);
		if ctx.is_null() {
			return null_mut();
		}
		// SAFETY: the context is the one Init was given, kept until Cleanup forgets it
		let (p, _) = unsafe { get_generic_interface(ctx, version, VRInitError_None) };
		cache.store(p, Ordering::Release);
		p
	}

	/// Clear(): every interface forgotten
	fn clear(&self) {
		for p in [
			&self.settings,
			&self.properties,
			&self.server_driver_host,
			&self.driver_log,
			&self.driver_manager,
			&self.resources,
		] {
			p.store(null_mut(), Ordering::Release);
		}
	}

	/// InitServerDriverContext(pContext): the context kept, then InitServer: Clear, and VRServerDriverHost,
	/// VRSettings, VRProperties, VRDriverLog, VRDriverManager and VRResources fetched in that order, the first one
	/// missing failing it
	pub fn init_server(&self, ctx: *mut IVRDriverContext) -> EVRInitError {
		self.context.store(ctx, Ordering::Release);
		self.clear();
		if self.get(&self.server_driver_host, IVRServerDriverHost_Version).is_null()
			|| self.get(&self.settings, IVRSettings_Version).is_null()
			|| self.get(&self.properties, IVRProperties_Version).is_null()
			|| self.get(&self.driver_log, IVRDriverLog_Version).is_null()
			|| self.get(&self.driver_manager, IVRDriverManager_Version).is_null()
			|| self.get(&self.resources, IVRResources_Version).is_null()
		{
			return VRInitError_Init_InterfaceNotFound;
		}
		VRInitError_None
	}

	/// CleanupDriverContext(): the context and the interfaces forgotten
	pub fn cleanup(&self) {
		self.context.store(null_mut(), Ordering::Release);
		self.clear();
	}

	fn driver_log(&self) -> *mut IVRDriverLog {
		self.get(&self.driver_log, IVRDriverLog_Version).cast()
	}

	fn properties(&self) -> *mut IVRProperties {
		self.get(&self.properties, IVRProperties_Version).cast()
	}

	fn server_driver_host(&self) -> *mut IVRServerDriverHost {
		self.get(&self.server_driver_host, IVRServerDriverHost_Version).cast()
	}
}

/// `if (VRDriverLog()) VRDriverLog()->Log(msg)`: the message up to its first NUL, as a C string reaches it
pub fn driver_log(msg: &[u8]) {
	let log = CONTEXT.driver_log();
	if log.is_null() {
		return;
	}
	let end = msg.iter().position(|&c| c == 0).unwrap_or(msg.len());
	let mut text = Vec::with_capacity(end + 1);
	text.extend_from_slice(&msg[..end]);
	text.push(0);
	// SAFETY: log is vrserver's IVRDriverLog (from the context, live while the driver is), and text is NUL-terminated
	unsafe { ((*(*log).vtbl).Log)(log, text.as_ptr().cast()) };
}

/// VRProperties()->TrackedDeviceToPropertyContainer(id) (CVRPropertyHelpers forwards it)
pub fn tracked_device_to_property_container(id: u32) -> PropertyContainerHandle_t {
	let props = CONTEXT.properties();
	if props.is_null() {
		return 0;
	}
	// SAFETY: props is vrserver's IVRProperties, live while the driver is
	unsafe { ((*(*props).vtbl).TrackedDeviceToPropertyContainer)(props, id) }
}

/// CVRPropertyHelpers::GetProperty: one PropertyRead_t, its required size, tag and error (what the batch left in
/// them; the C++'s are uninitialised where vrserver writes nothing, 0 here)
fn get_property(
	props: *mut IVRProperties, c: PropertyContainerHandle_t, prop: ETrackedDeviceProperty, buf: &mut [u8],
) -> (u32, PropertyTypeTag_t, ETrackedPropertyError) {
	let mut batch = PropertyRead_t {
		prop,
		pvBuffer: buf.as_mut_ptr().cast(),
		unBufferSize: buf.len() as u32,
		unTag: 0,
		unRequiredBufferSize: 0,
		eError: 0,
	};
	// SAFETY: props is vrserver's IVRProperties; the batch's buffer is `buf`, of the size it says, which vrserver
	// writes at most that much of
	unsafe { ((*(*props).vtbl).ReadPropertyBatch)(props, c, &mut batch, 1) };
	(batch.unRequiredBufferSize, batch.unTag, batch.eError)
}

/// CVRPropertyHelpers::GetStringProperty into a buffer: the required size and the error (WrongDataType for a value
/// that is not a string), the buffer emptied on an error
fn get_string_property_into(
	props: *mut IVRProperties, c: PropertyContainerHandle_t, prop: ETrackedDeviceProperty, buf: &mut [u8],
) -> (u32, ETrackedPropertyError) {
	let (required, tag, mut error) = get_property(props, c, prop, buf);
	if tag != k_unStringPropertyTag && error == TrackedProp_Success {
		error = TrackedProp_WrongDataType;
	}
	if error != TrackedProp_Success && !buf.is_empty() {
		buf[0] = 0;
	}
	(required, error)
}

/// The bytes of a C string in a buffer: up to the first NUL (the whole buffer without one, where the C++ would read
/// on past it)
fn c_string(buf: &[u8]) -> Vec<u8> {
	buf[..buf.iter().position(|&c| c == 0).unwrap_or(buf.len())].to_vec()
}

/// VRProperties()->GetStringProperty(c, prop, &err): the std::string version, a 1024-byte buffer first and one of
/// the required size when that is too small
pub fn get_string_property(
	c: PropertyContainerHandle_t, prop: ETrackedDeviceProperty,
) -> (Vec<u8>, ETrackedPropertyError) {
	let props = CONTEXT.properties();
	if props.is_null() {
		// no context (the C++ would crash calling through it): nothing read
		return (Vec::new(), TrackedProp_CouldNotContactServer);
	}
	let mut buf = [0u8; 1024];
	let (required, mut err) = get_string_property_into(props, c, prop, &mut buf);
	let mut result = Vec::new();
	if err == TrackedProp_Success {
		result = c_string(&buf);
	} else if err == TrackedProp_BufferTooSmall {
		let mut big = vec![0u8; required as usize];
		(_, err) = get_string_property_into(props, c, prop, &mut big);
		result = c_string(&big);
	}
	(result, err)
}

/// VRServerDriverHost()->GetRawTrackedDevicePoses(t, poses, n)
pub fn get_raw_tracked_device_poses(t: f32, poses: &mut [TrackedDevicePose_t]) {
	let host = CONTEXT.server_driver_host();
	if host.is_null() {
		return;
	}
	// SAFETY: host is vrserver's IVRServerDriverHost; it writes at most `poses.len()` poses into the slice
	unsafe { ((*(*host).vtbl).GetRawTrackedDevicePoses)(host, t, poses.as_mut_ptr(), poses.len() as u32) };
}

/// VRServerDriverHost()->TrackedDevicePoseUpdated(id, pose, sizeof pose), through the interface's vtable as any
/// driver's pose goes (the override's hook, if it is in, hands a device not one of Valve's on as it is)
pub fn tracked_device_pose_updated(id: u32, pose: &DriverPose_t) {
	let host = CONTEXT.server_driver_host();
	if host.is_null() {
		return;
	}
	// SAFETY: host is vrserver's IVRServerDriverHost; the pose lives through the call
	unsafe {
		((*(*host).vtbl).TrackedDevicePoseUpdated)(host.cast(), id, pose, std::mem::size_of::<DriverPose_t>() as u32)
	};
}
