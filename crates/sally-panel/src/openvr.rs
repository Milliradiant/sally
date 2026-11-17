//! OpenVR's C API (openvr_capi.h of OpenVR 2.15.6), loaded at run time and never linked, so the panel starts where
//! SteamVR is not installed and never starts it. `Vr` is the calls the comparison and the overlay make (crate::valve),
//! `OpenVr` makes them through the library (crate::valve::fake plays them in tests).
//!
//! - The library: on Windows openvr_api.dll beside the program (the folder of GetModuleFileNameW's path), else by name
//!   (the DLL search order); on Linux, where it serves only tests, ./libopenvr_api.so, then libopenvr_api.so by name.
//!   Once found it stays loaded for the life of the process.
//! - Its entry points, which openvr.h declares (openvr_capi.h has them only under `#if 0`; VR_CALLTYPE is __cdecl on
//!   Windows, `extern "C"` here): VR_InitInternal2 (VR_InitInternal in a library without it), VR_GetGenericInterface
//!   with "FnTable:<version>" for an interface's table of functions, and VR_ShutdownInternal.
//! - The tables: `#[repr(C)]` structs of `Option<unsafe extern "C" fn>`, one entry each in the declaration order of
//!   VR_IVRSystem_FnTable, VR_IVROverlay_FnTable, VR_IVRSettings_FnTable and VR_IVRChaperoneSetup_FnTable, up to the
//!   last entry the panel calls (a table SteamVR hands out goes on after it). The entries are OPENVR_FNTABLE_CALLTYPE,
//!   __stdcall on Windows, which x64 ignores: `extern "C"` is the one convention there, as on Linux (a 32-bit build
//!   would need "system").
//! - The structs and enums as the header has them, under its names: an enum is a C int, with the values of openvr.h's
//!   enumerators (`VREvent_Quit` is openvr_capi.h's EVREventType_VREvent_Quit), and a C bool is one byte (u8, taken
//!   as `!= 0`). VREvent_t is packed to 4 on Linux (the header's `#pragma pack( push, 4 )` there: "originally defined
//!   mis-packed on Linux"), so its data starts at byte 12 there and at 16 on Windows.
//!
//! The layouts are asserted twice: by the `const` block below, and against openvr_capi.h itself by
//! tests/openvr_layout.rs, which hands the same numbers to the C compilers for both platforms (how to run it is at
//! its top).

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, clippy::upper_case_acronyms)]

use std::ffi::{c_char, c_void};
use std::path::PathBuf;
use std::ptr::{null, null_mut};

use sally_math::scanf::c_str;

// ---------------------------------------------------------------- the header's types and the values the panel uses

pub type TrackedDeviceIndex_t = u32;
pub type VROverlayHandle_t = u64;
pub type PropertyTypeTag_t = u32;

pub const k_unMaxTrackedDeviceCount: u32 = 64;

pub const IVRSystem_Version: &str = "IVRSystem_026";
pub const IVROverlay_Version: &str = "IVROverlay_028";
pub const IVRSettings_Version: &str = "IVRSettings_003";
pub const IVRChaperoneSetup_Version: &str = "IVRChaperoneSetup_006";

pub type EVRInitError = i32;
pub const VRInitError_None: EVRInitError = 0;
pub const VRInitError_Init_InterfaceNotFound: EVRInitError = 105;
pub const VRInitError_Init_NotInitialized: EVRInitError = 109;
pub const VRInitError_Init_NoServerForBackgroundApp: EVRInitError = 121;

pub type EVRApplicationType = i32;
pub const VRApplication_Overlay: EVRApplicationType = 2;
pub const VRApplication_Background: EVRApplicationType = 3;

pub type ETrackedDeviceClass = i32;
pub const TrackedDeviceClass_Invalid: ETrackedDeviceClass = 0;
pub const TrackedDeviceClass_HMD: ETrackedDeviceClass = 1;
pub const TrackedDeviceClass_Controller: ETrackedDeviceClass = 2;
pub const TrackedDeviceClass_GenericTracker: ETrackedDeviceClass = 3;
pub const TrackedDeviceClass_TrackingReference: ETrackedDeviceClass = 4;

pub type ETrackedDeviceProperty = i32;
pub const Prop_TrackingSystemName_String: ETrackedDeviceProperty = 1000;
pub const Prop_SerialNumber_String: ETrackedDeviceProperty = 1002;
pub const Prop_ResourceRoot_String: ETrackedDeviceProperty = 1035;
pub const Prop_RegisteredDeviceType_String: ETrackedDeviceProperty = 1036;

pub type ETrackedPropertyError = i32;
pub const TrackedProp_Success: ETrackedPropertyError = 0;

pub type ETrackingUniverseOrigin = i32;
pub const TrackingUniverseRawAndUncalibrated: ETrackingUniverseOrigin = 2;

pub type ETrackingResult = i32;
pub const TrackingResult_Running_OK: ETrackingResult = 200;

/// VREvent_t's eventType is a uint32_t that holds one
pub type EVREventType = u32;
pub const VREvent_MouseMove: EVREventType = 300;
pub const VREvent_MouseButtonDown: EVREventType = 301;
pub const VREvent_MouseButtonUp: EVREventType = 302;
pub const VREvent_FocusLeave: EVREventType = 304;
pub const VREvent_ScrollDiscrete: EVREventType = 305;
pub const VREvent_ScrollSmooth: EVREventType = 309;
pub const VREvent_Quit: EVREventType = 700;
pub const VREvent_KeyboardClosed: EVREventType = 1200;
pub const VREvent_KeyboardDone: EVREventType = 1202;
pub const VREvent_KeyboardClosed_Global: EVREventType = 1204;

/// VREvent_Mouse_t's button is a uint32_t of these bits
pub type EVRMouseButton = u32;
pub const VRMouseButton_Left: EVRMouseButton = 1;
pub const VRMouseButton_Right: EVRMouseButton = 2;
pub const VRMouseButton_Middle: EVRMouseButton = 4;

pub type EVROverlayError = i32;
pub const VROverlayError_None: EVROverlayError = 0;
pub const VROverlayError_KeyInUse: EVROverlayError = 17;
pub const VROverlayError_RequestFailed: EVROverlayError = 23;
pub const VROverlayError_KeyboardAlreadyInUse: EVROverlayError = 26;

pub type VROverlayInputMethod = i32;
pub const VROverlayInputMethod_Mouse: VROverlayInputMethod = 1;

pub type VROverlayFlags = i32;
pub const VROverlayFlags_SendVRSmoothScrollEvents: VROverlayFlags = 131072;
pub const VROverlayFlags_IsPremultiplied: VROverlayFlags = 2097152;

pub type ETextureType = i32;
pub const TextureType_OpenGL: ETextureType = 1;

pub type EColorSpace = i32;
pub const ColorSpace_Gamma: EColorSpace = 1;

pub type EGamepadTextInputMode = i32;
pub const k_EGamepadTextInputModeNormal: EGamepadTextInputMode = 0;

pub type EGamepadTextInputLineMode = i32;
pub const k_EGamepadTextInputLineModeSingleLine: EGamepadTextInputLineMode = 0;

/// ShowKeyboard's unFlags is a uint32_t of these bits
pub type EKeyboardFlags = u32;
pub const KeyboardFlag_Modal: EKeyboardFlags = 2;

pub type EVRSettingsError = i32;
pub const VRSettingsError_None: EVRSettingsError = 0;
pub const VRSettingsError_IPCFailed: EVRSettingsError = 1;
pub const VRSettingsError_UnsetSettingHasNoDefault: EVRSettingsError = 5;

// enums only the tables' other entries take
pub type EVREye = i32;
pub type EVRDistortionChannel = i32;
pub type EDeviceActivityLevel = i32;
pub type ETrackedControllerRole = i32;
pub type EHiddenAreaMeshType = i32;
pub type EVRButtonId = i32;
pub type EVRControllerAxisType = i32;
pub type EVRFirmwareError = i32;
pub type VROverlayTransformType = i32;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdMatrix34_t {
	pub m: [[f32; 4]; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdMatrix44_t {
	pub m: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdVector2_t {
	pub v: [f32; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdVector3_t {
	pub v: [f32; 3],
}

/// A quad of the chaperone's bounds: a wall's four corners
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdQuad_t {
	pub vCorners: [HmdVector3_t; 4],
}

pub type EChaperoneConfigFile = i32;

/// A device's pose (GetDeviceToAbsoluteTrackingPose)
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

/// A texture handed to SetOverlayTexture: for OpenGL the handle is the texture's name
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Texture_t {
	pub handle: *mut c_void,
	pub eType: ETextureType,
	pub eColorSpace: EColorSpace,
}

/// GetHiddenAreaMesh's result, returned by value
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HiddenAreaMesh_t {
	pub pVertexData: *mut HmdVector2_t,
	pub unTriangleCount: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VREvent_Mouse_t {
	pub x: f32,
	pub y: f32,
	pub button: u32,
	pub cursorIndex: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VREvent_Scroll_t {
	pub xdelta: f32,
	pub ydelta: f32,
	pub unused: u32,
	pub viewportscale: f32,
	pub cursorIndex: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VREvent_Keyboard_t {
	pub cNewInput: [c_char; 8],
	pub uUserValue: u64,
	pub overlayHandle: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VREvent_Reserved_t {
	pub reserved0: u64,
	pub reserved1: u64,
	pub reserved2: u64,
	pub reserved3: u64,
	pub reserved4: u64,
	pub reserved5: u64,
}

/// VREvent_Data_t with the members the panel reads, and `reserved`, the largest (48 bytes, as ProgressUpdate's 44
/// padded): the header's other members are other readings of the same bytes, which the size and alignment (8) of
/// these already cover
#[repr(C)]
#[derive(Clone, Copy)]
pub union VREvent_Data_t {
	pub reserved: VREvent_Reserved_t,
	pub mouse: VREvent_Mouse_t,
	pub scroll: VREvent_Scroll_t,
	pub keyboard: VREvent_Keyboard_t,
}

/// An event; packed to 4 on Linux and macOS, as the header has it there
#[cfg_attr(any(target_os = "linux", target_os = "macos"), repr(C, packed(4)))]
#[cfg_attr(not(any(target_os = "linux", target_os = "macos")), repr(C))]
#[derive(Clone, Copy)]
pub struct VREvent_t {
	pub eventType: u32,
	pub trackedDeviceIndex: TrackedDeviceIndex_t,
	pub eventAgeSeconds: f32,
	pub data: VREvent_Data_t,
}

impl VREvent_t {
	/// An event of zeros, every byte of its data written
	pub fn zeroed() -> VREvent_t {
		VREvent_t {
			eventType: 0,
			trackedDeviceIndex: 0,
			eventAgeSeconds: 0.0,
			data: VREvent_Data_t { reserved: VREvent_Reserved_t::default() },
		}
	}
}

/// Structs the tables' other entries take by pointer and the panel never makes: opaque here
macro_rules! opaque {
	($($name:ident),*) => {
		$(
			#[repr(C)]
			pub struct $name {
				_opaque: [u8; 0],
			}
		)*
	};
}
opaque!(
	DistortionCoordinates_t,
	DistortionCoordinate_t,
	VkInstance_T,
	VRControllerState_t,
	VRTextureBounds_t,
	VROverlayProjection_t,
	VROverlayIntersectionParams_t,
	VROverlayIntersectionResults_t,
	VROverlayIntersectionMaskPrimitive_t
);

// ---------------------------------------------------------------- the function tables

/// IVRSystem_026's table, entries 1 to 48 (to AcknowledgeQuit_Exiting) of its 51
#[repr(C)]
pub struct VR_IVRSystem_FnTable {
	pub GetRecommendedRenderTargetSize: Option<unsafe extern "C" fn(pnWidth: *mut u32, pnHeight: *mut u32)>,
	pub GetProjectionMatrix: Option<unsafe extern "C" fn(eEye: EVREye, fNearZ: f32, fFarZ: f32) -> HmdMatrix44_t>,
	pub GetProjectionRaw: Option<
		unsafe extern "C" fn(eEye: EVREye, pfLeft: *mut f32, pfRight: *mut f32, pfTop: *mut f32, pfBottom: *mut f32),
	>,
	pub ComputeDistortion: Option<
		unsafe extern "C" fn(
			eEye: EVREye,
			fU: f32,
			fV: f32,
			pDistortionCoordinates: *mut DistortionCoordinates_t,
		) -> u8,
	>,
	pub ComputeDistortionSet: Option<
		unsafe extern "C" fn(
			eEye: EVREye,
			eChannel: EVRDistortionChannel,
			bAsNormalizedDeviceCoordinates: u8,
			nNumCoordinates: u32,
			pInput: *mut DistortionCoordinate_t,
			pOutput: *mut DistortionCoordinate_t,
		) -> u8,
	>,
	pub GetEyeToHeadTransform: Option<unsafe extern "C" fn(eEye: EVREye) -> HmdMatrix34_t>,
	pub GetTimeSinceLastVsync:
		Option<unsafe extern "C" fn(pfSecondsSinceLastVsync: *mut f32, pulFrameCounter: *mut u64) -> u8>,
	pub GetD3D9AdapterIndex: Option<unsafe extern "C" fn() -> i32>,
	pub GetDXGIOutputInfo: Option<unsafe extern "C" fn(pnAdapterIndex: *mut i32)>,
	pub GetOutputDevice:
		Option<unsafe extern "C" fn(pnDevice: *mut u64, textureType: ETextureType, pInstance: *mut VkInstance_T)>,
	pub IsDisplayOnDesktop: Option<unsafe extern "C" fn() -> u8>,
	pub SetDisplayVisibility: Option<unsafe extern "C" fn(bIsVisibleOnDesktop: u8) -> u8>,
	pub GetDeviceToAbsoluteTrackingPose: Option<
		unsafe extern "C" fn(
			eOrigin: ETrackingUniverseOrigin,
			fPredictedSecondsToPhotonsFromNow: f32,
			pTrackedDevicePoseArray: *mut TrackedDevicePose_t,
			unTrackedDevicePoseArrayCount: u32,
		),
	>,
	pub GetSeatedZeroPoseToStandingAbsoluteTrackingPose: Option<unsafe extern "C" fn() -> HmdMatrix34_t>,
	pub GetRawZeroPoseToStandingAbsoluteTrackingPose: Option<unsafe extern "C" fn() -> HmdMatrix34_t>,
	pub GetSortedTrackedDeviceIndicesOfClass: Option<
		unsafe extern "C" fn(
			eTrackedDeviceClass: ETrackedDeviceClass,
			punTrackedDeviceIndexArray: *mut TrackedDeviceIndex_t,
			unTrackedDeviceIndexArrayCount: u32,
			unRelativeToTrackedDeviceIndex: TrackedDeviceIndex_t,
		) -> u32,
	>,
	pub GetTrackedDeviceActivityLevel:
		Option<unsafe extern "C" fn(unDeviceId: TrackedDeviceIndex_t) -> EDeviceActivityLevel>,
	pub ApplyTransform: Option<
		unsafe extern "C" fn(
			pOutputPose: *mut TrackedDevicePose_t,
			pTrackedDevicePose: *mut TrackedDevicePose_t,
			pTransform: *mut HmdMatrix34_t,
		),
	>,
	pub GetTrackedDeviceIndexForControllerRole:
		Option<unsafe extern "C" fn(unDeviceType: ETrackedControllerRole) -> TrackedDeviceIndex_t>,
	pub GetControllerRoleForTrackedDeviceIndex:
		Option<unsafe extern "C" fn(unDeviceIndex: TrackedDeviceIndex_t) -> ETrackedControllerRole>,
	pub GetTrackedDeviceClass: Option<unsafe extern "C" fn(unDeviceIndex: TrackedDeviceIndex_t) -> ETrackedDeviceClass>,
	pub IsTrackedDeviceConnected: Option<unsafe extern "C" fn(unDeviceIndex: TrackedDeviceIndex_t) -> u8>,
	pub GetBoolTrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			pError: *mut ETrackedPropertyError,
		) -> u8,
	>,
	pub GetFloatTrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			pError: *mut ETrackedPropertyError,
		) -> f32,
	>,
	pub GetInt32TrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			pError: *mut ETrackedPropertyError,
		) -> i32,
	>,
	pub GetUint64TrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			pError: *mut ETrackedPropertyError,
		) -> u64,
	>,
	pub GetMatrix34TrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			pError: *mut ETrackedPropertyError,
		) -> HmdMatrix34_t,
	>,
	pub GetArrayTrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			propType: PropertyTypeTag_t,
			pBuffer: *mut c_void,
			unBufferSize: u32,
			pError: *mut ETrackedPropertyError,
		) -> u32,
	>,
	pub GetStringTrackedDeviceProperty: Option<
		unsafe extern "C" fn(
			unDeviceIndex: TrackedDeviceIndex_t,
			prop: ETrackedDeviceProperty,
			pchValue: *mut c_char,
			unBufferSize: u32,
			pError: *mut ETrackedPropertyError,
		) -> u32,
	>,
	pub GetPropErrorNameFromEnum: Option<unsafe extern "C" fn(error: ETrackedPropertyError) -> *mut c_char>,
	pub PollNextEvent: Option<unsafe extern "C" fn(pEvent: *mut VREvent_t, uncbVREvent: u32) -> u8>,
	pub PollNextEventWithPose: Option<
		unsafe extern "C" fn(
			eOrigin: ETrackingUniverseOrigin,
			pEvent: *mut VREvent_t,
			uncbVREvent: u32,
			pTrackedDevicePose: *mut TrackedDevicePose_t,
		) -> u8,
	>,
	pub PollNextEventWithPoseAndOverlays: Option<
		unsafe extern "C" fn(
			eOrigin: ETrackingUniverseOrigin,
			pEvent: *mut VREvent_t,
			uncbVREvent: u32,
			pTrackedDevicePose: *mut TrackedDevicePose_t,
			pulOverlayHandle: *mut VROverlayHandle_t,
		) -> u8,
	>,
	pub GetEventTypeNameFromEnum: Option<unsafe extern "C" fn(eType: EVREventType) -> *mut c_char>,
	pub GetHiddenAreaMesh: Option<unsafe extern "C" fn(eEye: EVREye, r#type: EHiddenAreaMeshType) -> HiddenAreaMesh_t>,
	pub GetEyeTrackedFoveationCenter:
		Option<unsafe extern "C" fn(pNdcLeft: *mut HmdVector2_t, pNdcRight: *mut HmdVector2_t) -> u8>,
	pub GetEyeTrackedFoveationCenterForProjection:
		Option<unsafe extern "C" fn(pProjMat: *mut HmdMatrix44_t, pNdc: *mut HmdVector2_t) -> u8>,
	pub GetControllerState: Option<
		unsafe extern "C" fn(
			unControllerDeviceIndex: TrackedDeviceIndex_t,
			pControllerState: *mut VRControllerState_t,
			unControllerStateSize: u32,
		) -> u8,
	>,
	pub GetControllerStateWithPose: Option<
		unsafe extern "C" fn(
			eOrigin: ETrackingUniverseOrigin,
			unControllerDeviceIndex: TrackedDeviceIndex_t,
			pControllerState: *mut VRControllerState_t,
			unControllerStateSize: u32,
			pTrackedDevicePose: *mut TrackedDevicePose_t,
		) -> u8,
	>,
	pub TriggerHapticPulse: Option<
		unsafe extern "C" fn(unControllerDeviceIndex: TrackedDeviceIndex_t, unAxisId: u32, usDurationMicroSec: u16),
	>,
	pub GetButtonIdNameFromEnum: Option<unsafe extern "C" fn(eButtonId: EVRButtonId) -> *mut c_char>,
	pub GetControllerAxisTypeNameFromEnum:
		Option<unsafe extern "C" fn(eAxisType: EVRControllerAxisType) -> *mut c_char>,
	pub IsInputAvailable: Option<unsafe extern "C" fn() -> u8>,
	pub IsSteamVRDrawingControllers: Option<unsafe extern "C" fn() -> u8>,
	pub ShouldApplicationPause: Option<unsafe extern "C" fn() -> u8>,
	pub ShouldApplicationReduceRenderingWork: Option<unsafe extern "C" fn() -> u8>,
	pub PerformFirmwareUpdate: Option<unsafe extern "C" fn(unDeviceIndex: TrackedDeviceIndex_t) -> EVRFirmwareError>,
	pub AcknowledgeQuit_Exiting: Option<unsafe extern "C" fn()>,
}

/// IVROverlay_028's table, entries 1 to 78 (to HideKeyboard) of its 82
#[repr(C)]
pub struct VR_IVROverlay_FnTable {
	pub FindOverlay: Option<
		unsafe extern "C" fn(pchOverlayKey: *mut c_char, pOverlayHandle: *mut VROverlayHandle_t) -> EVROverlayError,
	>,
	pub CreateOverlay: Option<
		unsafe extern "C" fn(
			pchOverlayKey: *mut c_char,
			pchOverlayName: *mut c_char,
			pOverlayHandle: *mut VROverlayHandle_t,
		) -> EVROverlayError,
	>,
	pub CreateSubviewOverlay: Option<
		unsafe extern "C" fn(
			parentOverlayHandle: VROverlayHandle_t,
			pchSubviewOverlayKey: *mut c_char,
			pchSubviewOverlayName: *mut c_char,
			pSubviewOverlayHandle: *mut VROverlayHandle_t,
		) -> EVROverlayError,
	>,
	pub DestroyOverlay: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> EVROverlayError>,
	pub GetOverlayKey: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pchValue: *mut c_char,
			unBufferSize: u32,
			pError: *mut EVROverlayError,
		) -> u32,
	>,
	pub GetOverlayName: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pchValue: *mut c_char,
			unBufferSize: u32,
			pError: *mut EVROverlayError,
		) -> u32,
	>,
	pub SetOverlayName:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pchName: *mut c_char) -> EVROverlayError>,
	pub GetOverlayImageData: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pvBuffer: *mut c_void,
			unBufferSize: u32,
			punWidth: *mut u32,
			punHeight: *mut u32,
		) -> EVROverlayError,
	>,
	pub GetOverlayErrorNameFromEnum: Option<unsafe extern "C" fn(error: EVROverlayError) -> *mut c_char>,
	pub SetOverlayRenderingPid:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, unPID: u32) -> EVROverlayError>,
	pub GetOverlayRenderingPid: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> u32>,
	pub SetOverlayFlag: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			eOverlayFlag: VROverlayFlags,
			bEnabled: u8,
		) -> EVROverlayError,
	>,
	pub GetOverlayFlag: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			eOverlayFlag: VROverlayFlags,
			pbEnabled: *mut u8,
		) -> EVROverlayError,
	>,
	pub GetOverlayFlags:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pFlags: *mut u32) -> EVROverlayError>,
	pub SetOverlayColor: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fRed: f32, fGreen: f32, fBlue: f32) -> EVROverlayError,
	>,
	pub GetOverlayColor: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pfRed: *mut f32,
			pfGreen: *mut f32,
			pfBlue: *mut f32,
		) -> EVROverlayError,
	>,
	pub SetOverlayAlpha:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fAlpha: f32) -> EVROverlayError>,
	pub GetOverlayAlpha:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pfAlpha: *mut f32) -> EVROverlayError>,
	pub SetOverlayTexelAspect:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fTexelAspect: f32) -> EVROverlayError>,
	pub GetOverlayTexelAspect:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pfTexelAspect: *mut f32) -> EVROverlayError>,
	pub SetOverlaySortOrder:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, unSortOrder: u32) -> EVROverlayError>,
	pub GetOverlaySortOrder:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, punSortOrder: *mut u32) -> EVROverlayError>,
	pub SetOverlayWidthInMeters:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fWidthInMeters: f32) -> EVROverlayError>,
	pub GetOverlayWidthInMeters:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pfWidthInMeters: *mut f32) -> EVROverlayError>,
	pub SetOverlayCurvature:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fCurvature: f32) -> EVROverlayError>,
	pub GetOverlayCurvature:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pfCurvature: *mut f32) -> EVROverlayError>,
	pub SetOverlayPreCurvePitch:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fRadians: f32) -> EVROverlayError>,
	pub GetOverlayPreCurvePitch:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pfRadians: *mut f32) -> EVROverlayError>,
	pub SetOverlayTextureColorSpace: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, eTextureColorSpace: EColorSpace) -> EVROverlayError,
	>,
	pub GetOverlayTextureColorSpace: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			peTextureColorSpace: *mut EColorSpace,
		) -> EVROverlayError,
	>,
	pub SetOverlayTextureBounds: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pOverlayTextureBounds: *mut VRTextureBounds_t,
		) -> EVROverlayError,
	>,
	pub GetOverlayTextureBounds: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pOverlayTextureBounds: *mut VRTextureBounds_t,
		) -> EVROverlayError,
	>,
	pub GetOverlayTransformType: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			peTransformType: *mut VROverlayTransformType,
		) -> EVROverlayError,
	>,
	pub SetOverlayTransformAbsolute: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			eTrackingOrigin: ETrackingUniverseOrigin,
			pmatTrackingOriginToOverlayTransform: *mut HmdMatrix34_t,
		) -> EVROverlayError,
	>,
	pub GetOverlayTransformAbsolute: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			peTrackingOrigin: *mut ETrackingUniverseOrigin,
			pmatTrackingOriginToOverlayTransform: *mut HmdMatrix34_t,
		) -> EVROverlayError,
	>,
	pub SetOverlayTransformTrackedDeviceRelative: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			unTrackedDevice: TrackedDeviceIndex_t,
			pmatTrackedDeviceToOverlayTransform: *mut HmdMatrix34_t,
		) -> EVROverlayError,
	>,
	pub GetOverlayTransformTrackedDeviceRelative: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			punTrackedDevice: *mut TrackedDeviceIndex_t,
			pmatTrackedDeviceToOverlayTransform: *mut HmdMatrix34_t,
		) -> EVROverlayError,
	>,
	pub SetOverlayTransformTrackedDeviceComponent: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			unDeviceIndex: TrackedDeviceIndex_t,
			pchComponentName: *mut c_char,
		) -> EVROverlayError,
	>,
	pub GetOverlayTransformTrackedDeviceComponent: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			punDeviceIndex: *mut TrackedDeviceIndex_t,
			pchComponentName: *mut c_char,
			unComponentNameSize: u32,
		) -> EVROverlayError,
	>,
	pub SetOverlayTransformCursor: Option<
		unsafe extern "C" fn(ulCursorOverlayHandle: VROverlayHandle_t, pvHotspot: *mut HmdVector2_t) -> EVROverlayError,
	>,
	pub GetOverlayTransformCursor: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pvHotspot: *mut HmdVector2_t) -> EVROverlayError,
	>,
	pub SetOverlayTransformProjection: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			eTrackingOrigin: ETrackingUniverseOrigin,
			pmatTrackingOriginToOverlayTransform: *mut HmdMatrix34_t,
			pProjection: *mut VROverlayProjection_t,
			eEye: EVREye,
		) -> EVROverlayError,
	>,
	pub SetSubviewPosition:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, fX: f32, fY: f32) -> EVROverlayError>,
	pub ShowOverlay: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> EVROverlayError>,
	pub HideOverlay: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> EVROverlayError>,
	pub IsOverlayVisible: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> u8>,
	pub GetTransformForOverlayCoordinates: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			eTrackingOrigin: ETrackingUniverseOrigin,
			coordinatesInOverlay: HmdVector2_t,
			pmatTransform: *mut HmdMatrix34_t,
		) -> EVROverlayError,
	>,
	pub WaitFrameSync: Option<unsafe extern "C" fn(nTimeoutMs: u32) -> EVROverlayError>,
	pub PollNextOverlayEvent: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pEvent: *mut VREvent_t, uncbVREvent: u32) -> u8,
	>,
	pub GetOverlayInputMethod: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			peInputMethod: *mut VROverlayInputMethod,
		) -> EVROverlayError,
	>,
	pub SetOverlayInputMethod: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, eInputMethod: VROverlayInputMethod) -> EVROverlayError,
	>,
	pub GetOverlayMouseScale: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pvecMouseScale: *mut HmdVector2_t) -> EVROverlayError,
	>,
	pub SetOverlayMouseScale: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pvecMouseScale: *mut HmdVector2_t) -> EVROverlayError,
	>,
	pub ComputeOverlayIntersection: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pParams: *mut VROverlayIntersectionParams_t,
			pResults: *mut VROverlayIntersectionResults_t,
		) -> u8,
	>,
	pub IsHoverTargetOverlay: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> u8>,
	pub SetOverlayIntersectionMask: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pMaskPrimitives: *mut VROverlayIntersectionMaskPrimitive_t,
			unNumMaskPrimitives: u32,
			unPrimitiveSize: u32,
		) -> EVROverlayError,
	>,
	pub TriggerLaserMouseHapticVibration: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			fDurationSeconds: f32,
			fFrequency: f32,
			fAmplitude: f32,
		) -> EVROverlayError,
	>,
	pub SetOverlayCursor: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, ulCursorHandle: VROverlayHandle_t) -> EVROverlayError,
	>,
	pub SetOverlayCursorPositionOverride: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pvCursor: *mut HmdVector2_t) -> EVROverlayError,
	>,
	pub ClearOverlayCursorPositionOverride:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> EVROverlayError>,
	pub SetOverlayTexture:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pTexture: *mut Texture_t) -> EVROverlayError>,
	pub ClearOverlayTexture: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> EVROverlayError>,
	pub SetOverlayRaw: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pvBuffer: *mut c_void,
			unWidth: u32,
			unHeight: u32,
			unBytesPerPixel: u32,
		) -> EVROverlayError,
	>,
	pub SetOverlayFromFile:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pchFilePath: *mut c_char) -> EVROverlayError>,
	pub GetOverlayTexture: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pNativeTextureHandle: *mut *mut c_void,
			pNativeTextureRef: *mut c_void,
			pWidth: *mut u32,
			pHeight: *mut u32,
			pNativeFormat: *mut u32,
			pAPIType: *mut ETextureType,
			pColorSpace: *mut EColorSpace,
			pTextureBounds: *mut VRTextureBounds_t,
		) -> EVROverlayError,
	>,
	pub ReleaseNativeOverlayHandle: Option<
		unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, pNativeTextureHandle: *mut c_void) -> EVROverlayError,
	>,
	pub GetOverlayTextureSize: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			pWidth: *mut u32,
			pHeight: *mut u32,
		) -> EVROverlayError,
	>,
	pub CreateDashboardOverlay: Option<
		unsafe extern "C" fn(
			pchOverlayKey: *mut c_char,
			pchOverlayFriendlyName: *mut c_char,
			pMainHandle: *mut VROverlayHandle_t,
			pThumbnailHandle: *mut VROverlayHandle_t,
		) -> EVROverlayError,
	>,
	pub IsDashboardVisible: Option<unsafe extern "C" fn() -> u8>,
	pub IsActiveDashboardOverlay: Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t) -> u8>,
	pub SetDashboardOverlaySceneProcess:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, unProcessId: u32) -> EVROverlayError>,
	pub GetDashboardOverlaySceneProcess:
		Option<unsafe extern "C" fn(ulOverlayHandle: VROverlayHandle_t, punProcessId: *mut u32) -> EVROverlayError>,
	pub ShowDashboard: Option<unsafe extern "C" fn(pchOverlayToShow: *mut c_char)>,
	pub GetPrimaryDashboardDevice: Option<unsafe extern "C" fn() -> TrackedDeviceIndex_t>,
	pub ShowKeyboard: Option<
		unsafe extern "C" fn(
			eInputMode: EGamepadTextInputMode,
			eLineInputMode: EGamepadTextInputLineMode,
			unFlags: u32,
			pchDescription: *mut c_char,
			unCharMax: u32,
			pchExistingText: *mut c_char,
			uUserValue: u64,
		) -> EVROverlayError,
	>,
	pub ShowKeyboardForOverlay: Option<
		unsafe extern "C" fn(
			ulOverlayHandle: VROverlayHandle_t,
			eInputMode: EGamepadTextInputMode,
			eLineInputMode: EGamepadTextInputLineMode,
			unFlags: u32,
			pchDescription: *mut c_char,
			unCharMax: u32,
			pchExistingText: *mut c_char,
			uUserValue: u64,
		) -> EVROverlayError,
	>,
	pub GetKeyboardText: Option<unsafe extern "C" fn(pchText: *mut c_char, cchText: u32) -> u32>,
	pub HideKeyboard: Option<unsafe extern "C" fn()>,
}

/// IVRChaperoneSetup_006's table, entries 1 to 6 (to GetLiveCollisionBoundsInfo) of its 20
#[repr(C)]
pub struct VR_IVRChaperoneSetup_FnTable {
	pub CommitWorkingCopy: Option<unsafe extern "C" fn(configFile: EChaperoneConfigFile) -> u8>,
	pub RevertWorkingCopy: Option<unsafe extern "C" fn()>,
	pub GetWorkingPlayAreaSize: Option<unsafe extern "C" fn(pSizeX: *mut f32, pSizeZ: *mut f32) -> u8>,
	pub GetWorkingPlayAreaRect: Option<unsafe extern "C" fn(rect: *mut HmdQuad_t) -> u8>,
	pub GetWorkingCollisionBoundsInfo:
		Option<unsafe extern "C" fn(pQuadsBuffer: *mut HmdQuad_t, punQuadsCount: *mut u32) -> u8>,
	pub GetLiveCollisionBoundsInfo:
		Option<unsafe extern "C" fn(pQuadsBuffer: *mut HmdQuad_t, punQuadsCount: *mut u32) -> u8>,
}

/// IVRSettings_003's table, entries 1 to 9 (to GetString) of its 11
#[repr(C)]
pub struct VR_IVRSettings_FnTable {
	pub GetSettingsErrorNameFromEnum: Option<unsafe extern "C" fn(eError: EVRSettingsError) -> *mut c_char>,
	pub SetBool: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			bValue: u8,
			peError: *mut EVRSettingsError,
		),
	>,
	pub SetInt32: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			nValue: i32,
			peError: *mut EVRSettingsError,
		),
	>,
	pub SetFloat: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			flValue: f32,
			peError: *mut EVRSettingsError,
		),
	>,
	pub SetString: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			pchValue: *mut c_char,
			peError: *mut EVRSettingsError,
		),
	>,
	pub GetBool: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			peError: *mut EVRSettingsError,
		) -> u8,
	>,
	pub GetInt32: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			peError: *mut EVRSettingsError,
		) -> i32,
	>,
	pub GetFloat: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			peError: *mut EVRSettingsError,
		) -> f32,
	>,
	pub GetString: Option<
		unsafe extern "C" fn(
			pchSection: *mut c_char,
			pchSettingsKey: *mut c_char,
			pchValue: *mut c_char,
			unValueLen: u32,
			peError: *mut EVRSettingsError,
		),
	>,
}

// the layouts the C compilers give openvr_capi.h's own (tests/openvr_layout.rs asserts the same numbers of the header,
// for Linux x64 and Windows x64)
const _: () = {
	use std::mem::{align_of, offset_of, size_of};
	assert!(size_of::<HmdMatrix34_t>() == 48);
	assert!(size_of::<HmdMatrix44_t>() == 64);
	assert!(size_of::<HmdVector2_t>() == 8);
	assert!(size_of::<HmdVector3_t>() == 12);
	assert!(size_of::<TrackedDevicePose_t>() == 80);
	assert!(offset_of!(TrackedDevicePose_t, vVelocity) == 48);
	assert!(offset_of!(TrackedDevicePose_t, vAngularVelocity) == 60);
	assert!(offset_of!(TrackedDevicePose_t, eTrackingResult) == 72);
	assert!(offset_of!(TrackedDevicePose_t, bPoseIsValid) == 76);
	assert!(offset_of!(TrackedDevicePose_t, bDeviceIsConnected) == 77);
	assert!(size_of::<Texture_t>() == 16);
	assert!(offset_of!(Texture_t, eType) == 8);
	assert!(offset_of!(Texture_t, eColorSpace) == 12);
	assert!(size_of::<HiddenAreaMesh_t>() == 16);
	assert!(offset_of!(HiddenAreaMesh_t, unTriangleCount) == 8);
	assert!(size_of::<VREvent_Mouse_t>() == 16);
	assert!(offset_of!(VREvent_Mouse_t, y) == 4);
	assert!(offset_of!(VREvent_Mouse_t, button) == 8);
	assert!(size_of::<VREvent_Scroll_t>() == 20);
	assert!(offset_of!(VREvent_Scroll_t, ydelta) == 4);
	assert!(size_of::<VREvent_Keyboard_t>() == 24);
	assert!(offset_of!(VREvent_Keyboard_t, uUserValue) == 8);
	assert!(offset_of!(VREvent_Keyboard_t, overlayHandle) == 16);
	assert!(size_of::<VREvent_Data_t>() == 48);
	assert!(align_of::<VREvent_Data_t>() == 8);
	assert!(offset_of!(VREvent_t, trackedDeviceIndex) == 4);
	assert!(offset_of!(VREvent_t, eventAgeSeconds) == 8);
	#[cfg(any(target_os = "linux", target_os = "macos"))]
	assert!(size_of::<VREvent_t>() == 60 && align_of::<VREvent_t>() == 4 && offset_of!(VREvent_t, data) == 12);
	#[cfg(not(any(target_os = "linux", target_os = "macos")))]
	assert!(size_of::<VREvent_t>() == 64 && align_of::<VREvent_t>() == 8 && offset_of!(VREvent_t, data) == 16);
	assert!(offset_of!(VR_IVRSystem_FnTable, GetDeviceToAbsoluteTrackingPose) == 96);
	assert!(offset_of!(VR_IVRSystem_FnTable, GetTrackedDeviceClass) == 160);
	assert!(offset_of!(VR_IVRSystem_FnTable, GetStringTrackedDeviceProperty) == 224);
	assert!(offset_of!(VR_IVRSystem_FnTable, PollNextEvent) == 240);
	assert!(offset_of!(VR_IVRSystem_FnTable, GetRawZeroPoseToStandingAbsoluteTrackingPose) == 112);
	assert!(offset_of!(VR_IVRSystem_FnTable, TriggerHapticPulse) == 312);
	assert!(offset_of!(VR_IVRSystem_FnTable, AcknowledgeQuit_Exiting) == 376);
	assert!(size_of::<VR_IVRSystem_FnTable>() == 384);
	assert!(offset_of!(VR_IVROverlay_FnTable, DestroyOverlay) == 24);
	assert!(offset_of!(VR_IVROverlay_FnTable, SetOverlayFlag) == 88);
	assert!(offset_of!(VR_IVROverlay_FnTable, SetOverlayWidthInMeters) == 176);
	assert!(offset_of!(VR_IVROverlay_FnTable, IsOverlayVisible) == 360);
	assert!(offset_of!(VR_IVROverlay_FnTable, PollNextOverlayEvent) == 384);
	assert!(offset_of!(VR_IVROverlay_FnTable, SetOverlayInputMethod) == 400);
	assert!(offset_of!(VR_IVROverlay_FnTable, SetOverlayMouseScale) == 416);
	assert!(offset_of!(VR_IVROverlay_FnTable, SetOverlayTexture) == 480);
	assert!(offset_of!(VR_IVROverlay_FnTable, CreateDashboardOverlay) == 536);
	assert!(offset_of!(VR_IVROverlay_FnTable, ShowKeyboardForOverlay) == 600);
	assert!(offset_of!(VR_IVROverlay_FnTable, GetKeyboardText) == 608);
	assert!(offset_of!(VR_IVROverlay_FnTable, HideKeyboard) == 616);
	assert!(size_of::<VR_IVROverlay_FnTable>() == 624);
	assert!(offset_of!(VR_IVRSettings_FnTable, GetString) == 64);
	assert!(size_of::<VR_IVRSettings_FnTable>() == 72);
	assert!(size_of::<HmdQuad_t>() == 48);
	assert!(offset_of!(VR_IVRChaperoneSetup_FnTable, GetLiveCollisionBoundsInfo) == 40);
	assert!(size_of::<VR_IVRChaperoneSetup_FnTable>() == 48);
};

// ---------------------------------------------------------------- the calls, behind a trait

/// The interfaces the panel takes from SteamVR
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Interface {
	System,
	Overlay,
	Settings,
	ChaperoneSetup,
}

impl Interface {
	/// Its version, as openvr_capi.h has it
	pub fn version(self) -> &'static str {
		match self {
			Interface::System => IVRSystem_Version,
			Interface::Overlay => IVROverlay_Version,
			Interface::Settings => IVRSettings_Version,
			Interface::ChaperoneSetup => IVRChaperoneSetup_Version,
		}
	}
}

/// Why `load` failed
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadError {
	/// no openvr_api library beside the panel, nor by name
	Missing,
	/// one without VR_InitInternal2 and VR_InitInternal, or without VR_GetGenericInterface
	Unusable,
}

/// An event from PollNextEvent or PollNextOverlayEvent: its type and the members of its data valve.cpp reads, each
/// copied from the union's bytes whatever the type is (the C++ read whichever it needed)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VrEvent {
	pub event_type: EVREventType,
	pub mouse: VREvent_Mouse_t,
	pub scroll: VREvent_Scroll_t,
	pub keyboard: VREvent_Keyboard_t,
}

impl VrEvent {
	/// The event's type and the three readings of its data
	pub fn of(ev: &VREvent_t) -> VrEvent {
		// a copy: in the packed struct the union may sit at an address its alignment does not allow a reference to
		let data = ev.data;
		// SAFETY: the members are plain numbers, for which any bytes are a value, and all 48 bytes of the union are
		// initialised (the event was made by VREvent_t::zeroed before SteamVR wrote into it)
		let (mouse, scroll, keyboard) = unsafe { (data.mouse, data.scroll, data.keyboard) };
		VrEvent { event_type: ev.eventType, mouse, scroll, keyboard }
	}
}

/// ShowKeyboardForOverlay's arguments after the overlay
#[derive(Clone, Debug, Default, PartialEq)]
pub struct KeyboardRequest {
	pub input_mode: EGamepadTextInputMode,
	pub line_mode: EGamepadTextInputLineMode,
	pub flags: EKeyboardFlags,
	/// pchDescription and pchExistingText, without their NUL
	pub description: Vec<u8>,
	pub char_max: u32,
	pub existing_text: Vec<u8>,
	/// rides on the keyboard's events as data.keyboard.uUserValue
	pub user_value: u64,
}

/// The OpenVR calls valve.cpp makes, one method each (`OpenVr` makes them through openvr_api; crate::valve::fake
/// plays them for tests). The values are OpenVR's: the enums' codes, a VROverlayHandle_t for an overlay.
pub trait Vr {
	/// Loads the library (the first time: one found stays) and looks its entry points up again
	fn load(&mut self) -> Result<(), LoadError>;
	/// VR_InitInternal2(&err, app, "") (VR_InitInternal(&err, app) in a library without it): the EVRInitError when
	/// it failed
	fn init(&mut self, app: EVRApplicationType) -> Result<(), EVRInitError>;
	/// VR_ShutdownInternal: false, and nothing shut down, in a library without it. The tables fetched are forgotten
	/// either way
	fn shutdown(&mut self) -> bool;
	/// VR_GetGenericInterface("FnTable:<version>", &err): the interface's table, kept for the calls below until
	/// `shutdown`, or the error when SteamVR has none
	fn interface(&mut self, which: Interface) -> Result<(), EVRInitError>;
	/// The interface's table was fetched since the last `shutdown`
	fn has(&self, which: Interface) -> bool;

	// IVRSystem
	fn tracked_device_class(&mut self, index: TrackedDeviceIndex_t) -> ETrackedDeviceClass;
	/// GetStringTrackedDeviceProperty into a 256-byte buffer, as the C++ read it (its error not looked at): the text
	/// up to its NUL, empty when there was none
	fn string_tracked_device_property(&mut self, index: TrackedDeviceIndex_t, prop: ETrackedDeviceProperty) -> String;
	fn device_to_absolute_tracking_pose(
		&mut self, origin: ETrackingUniverseOrigin, predicted_seconds: f32, poses: &mut [TrackedDevicePose_t],
	);
	fn poll_next_event(&mut self) -> Option<VrEvent>;
	fn acknowledge_quit_exiting(&mut self);

	/// GetRawZeroPoseToStandingAbsoluteTrackingPose: OpenVR's raw universe in its standing one, None without the table
	fn raw_zero_to_standing(&mut self) -> Option<HmdMatrix34_t>;
	/// TriggerHapticPulse (legacy input) on a controller
	fn trigger_haptic_pulse(&mut self, index: TrackedDeviceIndex_t, axis: u32, micros: u16);

	// IVRSettings
	/// GetString into a 256-byte buffer: the value, or the EVRSettingsError
	fn settings_string(&mut self, section: &str, key: &str) -> Result<String, EVRSettingsError>;

	// IVRChaperoneSetup
	/// GetLiveCollisionBoundsInfo: the chaperone's walls (in the standing universe), at most 512 of them; None without
	/// the table or when it fails
	fn live_collision_bounds(&mut self) -> Option<Vec<HmdQuad_t>>;

	// IVROverlay
	/// The main overlay's handle and its thumbnail's
	fn create_dashboard_overlay(
		&mut self, key: &str, name: &str,
	) -> Result<(VROverlayHandle_t, VROverlayHandle_t), EVROverlayError>;
	fn destroy_overlay(&mut self, handle: VROverlayHandle_t) -> EVROverlayError;
	fn set_overlay_width_in_meters(&mut self, handle: VROverlayHandle_t, metres: f32) -> EVROverlayError;
	fn set_overlay_input_method(&mut self, handle: VROverlayHandle_t, method: VROverlayInputMethod) -> EVROverlayError;
	fn set_overlay_mouse_scale(&mut self, handle: VROverlayHandle_t, scale: &HmdVector2_t) -> EVROverlayError;
	fn set_overlay_flag(&mut self, handle: VROverlayHandle_t, flag: VROverlayFlags, enabled: bool) -> EVROverlayError;
	fn is_overlay_visible(&mut self, handle: VROverlayHandle_t) -> bool;
	fn poll_next_overlay_event(&mut self, handle: VROverlayHandle_t) -> Option<VrEvent>;
	fn set_overlay_texture(&mut self, handle: VROverlayHandle_t, texture: &Texture_t) -> EVROverlayError;
	fn show_keyboard_for_overlay(&mut self, handle: VROverlayHandle_t, request: &KeyboardRequest) -> EVROverlayError;
	fn hide_keyboard(&mut self);
	/// GetKeyboardText into a 256-byte buffer
	fn keyboard_text(&mut self) -> String;
}

// ---------------------------------------------------------------- the library

// openvr.h's signatures: an init returns the init token, a uint32_t (openvr_capi.h's `#if 0` block, and the C++ panel
// after it, said intptr_t; nothing reads it)
type InitInternal2Fn = unsafe extern "C" fn(
	peError: *mut EVRInitError,
	eApplicationType: EVRApplicationType,
	pStartupInfo: *const c_char,
) -> u32;
type InitInternalFn = unsafe extern "C" fn(peError: *mut EVRInitError, eApplicationType: EVRApplicationType) -> u32;
type ShutdownInternalFn = unsafe extern "C" fn();
type GetGenericInterfaceFn =
	unsafe extern "C" fn(pchInterfaceVersion: *const c_char, peError: *mut EVRInitError) -> *mut c_void;

/// openvr_api as the panel uses it: the library and its entry points, and the tables of the session's interfaces
pub struct OpenVr {
	/// a library of its own instead of the search (`with_library`)
	path: Option<PathBuf>,
	/// the HMODULE or dlopen's handle, null until the library is found; never unloaded
	lib: *mut c_void,
	init_internal2: Option<InitInternal2Fn>,
	init_internal: Option<InitInternalFn>,
	shutdown_internal: Option<ShutdownInternalFn>,
	get_generic_interface: Option<GetGenericInterfaceFn>,
	system: *const VR_IVRSystem_FnTable,
	overlay: *const VR_IVROverlay_FnTable,
	settings: *const VR_IVRSettings_FnTable,
	chaperone_setup: *const VR_IVRChaperoneSetup_FnTable,
}

impl Default for OpenVr {
	fn default() -> OpenVr {
		OpenVr::new()
	}
}

/// A text in a buffer the library wrote: up to its first NUL (the whole buffer without one)
fn c_text(buf: &[u8]) -> String {
	String::from_utf8_lossy(c_str(buf)).into_owned()
}

/// A string as the C++ passed its c_str(): up to its first NUL, with a NUL after
fn c_arg(s: &[u8]) -> Vec<u8> {
	let mut v = c_str(s).to_vec();
	v.push(0);
	v
}

/// The library's symbol `name` (a C string literal) as a function of type `$t`, None when it has no such symbol
macro_rules! entry {
	($lib:expr, $name:literal, $t:ty) => {{
		let p = sys::symbol($lib, $name);
		// SAFETY: openvr_api's symbol of that name is the function openvr.h declares, whose signature is $t; its
		// address is a valid pointer to it while the library is loaded, which is for good
		(!p.is_null()).then(|| unsafe { std::mem::transmute::<*mut c_void, $t>(p) })
	}};
}

impl OpenVr {
	/// Nothing loaded yet: the library is looked for at the first `load`
	pub fn new() -> OpenVr {
		OpenVr {
			path: None,
			lib: null_mut(),
			init_internal2: None,
			init_internal: None,
			shutdown_internal: None,
			get_generic_interface: None,
			system: null(),
			overlay: null(),
			settings: null(),
			chaperone_setup: null(),
		}
	}

	/// The library at `path` instead of the search (a test's stand-in openvr_api)
	pub fn with_library(path: impl Into<PathBuf>) -> OpenVr {
		OpenVr { path: Some(path.into()), ..OpenVr::new() }
	}

	/// An entry of the IVRSystem table, None without the table
	fn sys<F>(&self, entry: impl FnOnce(&VR_IVRSystem_FnTable) -> Option<F>) -> Option<F> {
		// SAFETY: a table that is not null is the one VR_GetGenericInterface returned in this session, which SteamVR
		// keeps until VR_ShutdownInternal (`shutdown` forgets it first); its entries are the header's functions, at
		// the places and of the types declared above (tests/openvr_layout.rs checks the places)
		unsafe { self.system.as_ref() }.and_then(entry)
	}

	/// An entry of the IVROverlay table, None without the table
	fn ovl<F>(&self, entry: impl FnOnce(&VR_IVROverlay_FnTable) -> Option<F>) -> Option<F> {
		// SAFETY: as for `sys`
		unsafe { self.overlay.as_ref() }.and_then(entry)
	}

	/// An entry of the IVRSettings table, None without the table
	fn set<F>(&self, entry: impl FnOnce(&VR_IVRSettings_FnTable) -> Option<F>) -> Option<F> {
		// SAFETY: as for `sys`
		unsafe { self.settings.as_ref() }.and_then(entry)
	}

	/// An entry of the IVRChaperoneSetup table, None without the table
	fn chs<F>(&self, entry: impl FnOnce(&VR_IVRChaperoneSetup_FnTable) -> Option<F>) -> Option<F> {
		// SAFETY: as for `sys`
		unsafe { self.chaperone_setup.as_ref() }.and_then(entry)
	}
}

impl Vr for OpenVr {
	fn load(&mut self) -> Result<(), LoadError> {
		if self.lib.is_null() {
			self.lib = match &self.path {
				Some(p) => sys::load_path(p),
				None => sys::load_library(),
			};
		}
		if self.lib.is_null() {
			return Err(LoadError::Missing);
		}
		let lib = self.lib;
		self.init_internal2 = entry!(lib, c"VR_InitInternal2", InitInternal2Fn);
		self.init_internal = entry!(lib, c"VR_InitInternal", InitInternalFn);
		self.shutdown_internal = entry!(lib, c"VR_ShutdownInternal", ShutdownInternalFn);
		self.get_generic_interface = entry!(lib, c"VR_GetGenericInterface", GetGenericInterfaceFn);
		if (self.init_internal2.is_none() && self.init_internal.is_none()) || self.get_generic_interface.is_none() {
			return Err(LoadError::Unusable);
		}
		Ok(())
	}

	fn init(&mut self, app: EVRApplicationType) -> Result<(), EVRInitError> {
		let mut err = VRInitError_None;
		if let Some(init2) = self.init_internal2 {
			// SAFETY: VR_InitInternal2 writes the error it is given and reads the startup info, a NUL-terminated
			// string (empty, as the C++ passed "")
			unsafe { init2(&mut err, app, c"".as_ptr()) };
		} else if let Some(init) = self.init_internal {
			// SAFETY: VR_InitInternal writes the error it is given
			unsafe { init(&mut err, app) };
		} else {
			return Err(VRInitError_Init_NotInitialized);
		}
		if err == VRInitError_None {
			Ok(())
		} else {
			Err(err)
		}
	}

	fn shutdown(&mut self) -> bool {
		self.system = null();
		self.overlay = null();
		self.settings = null();
		self.chaperone_setup = null();
		match self.shutdown_internal {
			Some(shutdown) => {
				// SAFETY: VR_ShutdownInternal takes nothing; the tables it ends are forgotten above
				unsafe { shutdown() };
				true
			}
			None => false,
		}
	}

	fn interface(&mut self, which: Interface) -> Result<(), EVRInitError> {
		let Some(get) = self.get_generic_interface else { return Err(VRInitError_Init_NotInitialized) };
		let name = c_arg(format!("FnTable:{}", which.version()).as_bytes());
		let mut err = VRInitError_None;
		// SAFETY: VR_GetGenericInterface reads the NUL-terminated name and writes the error it is given
		let table = unsafe { get(name.as_ptr().cast(), &mut err) };
		match which {
			Interface::System => self.system = table.cast(),
			Interface::Overlay => self.overlay = table.cast(),
			Interface::Settings => self.settings = table.cast(),
			Interface::ChaperoneSetup => self.chaperone_setup = table.cast(),
		}
		if table.is_null() {
			Err(err)
		} else {
			Ok(())
		}
	}

	fn has(&self, which: Interface) -> bool {
		match which {
			Interface::System => !self.system.is_null(),
			Interface::Overlay => !self.overlay.is_null(),
			Interface::Settings => !self.settings.is_null(),
			Interface::ChaperoneSetup => !self.chaperone_setup.is_null(),
		}
	}

	fn tracked_device_class(&mut self, index: TrackedDeviceIndex_t) -> ETrackedDeviceClass {
		match self.sys(|t| t.GetTrackedDeviceClass) {
			// SAFETY: IVRSystem's GetTrackedDeviceClass takes an index, any index
			Some(f) => unsafe { f(index) },
			None => TrackedDeviceClass_Invalid,
		}
	}

	fn string_tracked_device_property(&mut self, index: TrackedDeviceIndex_t, prop: ETrackedDeviceProperty) -> String {
		let Some(f) = self.sys(|t| t.GetStringTrackedDeviceProperty) else { return String::new() };
		let mut buf = [0u8; 256];
		let mut err = TrackedProp_Success;
		// SAFETY: it writes at most the buffer's size (given) into the buffer, and the error
		unsafe { f(index, prop, buf.as_mut_ptr().cast(), buf.len() as u32, &mut err) };
		c_text(&buf)
	}

	fn device_to_absolute_tracking_pose(
		&mut self, origin: ETrackingUniverseOrigin, predicted_seconds: f32, poses: &mut [TrackedDevicePose_t],
	) {
		let Some(f) = self.sys(|t| t.GetDeviceToAbsoluteTrackingPose) else { return };
		// SAFETY: it writes at most the given count of poses into the array, which has that many
		unsafe { f(origin, predicted_seconds, poses.as_mut_ptr(), poses.len() as u32) };
	}

	fn poll_next_event(&mut self) -> Option<VrEvent> {
		let f = self.sys(|t| t.PollNextEvent)?;
		let mut ev = VREvent_t::zeroed();
		// SAFETY: it writes at most the given size (the struct's) into the event
		let got = unsafe { f(&mut ev, std::mem::size_of::<VREvent_t>() as u32) };
		(got != 0).then(|| VrEvent::of(&ev))
	}

	fn acknowledge_quit_exiting(&mut self) {
		if let Some(f) = self.sys(|t| t.AcknowledgeQuit_Exiting) {
			// SAFETY: AcknowledgeQuit_Exiting takes nothing
			unsafe { f() };
		}
	}

	fn raw_zero_to_standing(&mut self) -> Option<HmdMatrix34_t> {
		let f = self.sys(|t| t.GetRawZeroPoseToStandingAbsoluteTrackingPose)?;
		// SAFETY: it takes nothing and returns the matrix by value
		Some(unsafe { f() })
	}

	fn trigger_haptic_pulse(&mut self, index: TrackedDeviceIndex_t, axis: u32, micros: u16) {
		if let Some(f) = self.sys(|t| t.TriggerHapticPulse) {
			// SAFETY: it takes an index, an axis and a duration, any of them
			unsafe { f(index, axis, micros) };
		}
	}

	fn live_collision_bounds(&mut self) -> Option<Vec<HmdQuad_t>> {
		let f = self.chs(|t| t.GetLiveCollisionBoundsInfo)?;
		let mut n = 0u32;
		// SAFETY: with a null buffer it writes the count only
		unsafe { f(null_mut(), &mut n) };
		let mut quads = vec![HmdQuad_t::default(); n.min(512) as usize];
		let mut got = quads.len() as u32;
		if got == 0 {
			return Some(quads);
		}
		// SAFETY: it writes at most the count it is given (the buffer's length) into the buffer, and the count
		let ok = unsafe { f(quads.as_mut_ptr(), &mut got) } != 0;
		quads.truncate(got.min(quads.len() as u32) as usize);
		ok.then_some(quads)
	}

	fn settings_string(&mut self, section: &str, key: &str) -> Result<String, EVRSettingsError> {
		let Some(f) = self.set(|t| t.GetString) else { return Err(VRSettingsError_IPCFailed) };
		let mut section = c_arg(section.as_bytes());
		let mut key = c_arg(key.as_bytes());
		let mut buf = [0u8; 256];
		let mut err = VRSettingsError_None;
		// SAFETY: the section and the key are NUL-terminated (the header's char * is a const char * that lost its
		// const; they are the panel's own copies all the same); it writes at most the buffer's size into the buffer,
		// and the error
		unsafe {
			f(section.as_mut_ptr().cast(), key.as_mut_ptr().cast(), buf.as_mut_ptr().cast(), buf.len() as u32, &mut err)
		};
		if err == VRSettingsError_None {
			Ok(c_text(&buf))
		} else {
			Err(err)
		}
	}

	fn create_dashboard_overlay(
		&mut self, key: &str, name: &str,
	) -> Result<(VROverlayHandle_t, VROverlayHandle_t), EVROverlayError> {
		let f = self.ovl(|t| t.CreateDashboardOverlay).ok_or(VROverlayError_RequestFailed)?;
		let mut key = c_arg(key.as_bytes());
		let mut name = c_arg(name.as_bytes());
		let (mut main, mut thumb) = (0, 0);
		// SAFETY: the key and the name are NUL-terminated copies; it writes the two handles it is given
		let e = unsafe { f(key.as_mut_ptr().cast(), name.as_mut_ptr().cast(), &mut main, &mut thumb) };
		if e == VROverlayError_None {
			Ok((main, thumb))
		} else {
			Err(e)
		}
	}

	fn destroy_overlay(&mut self, handle: VROverlayHandle_t) -> EVROverlayError {
		match self.ovl(|t| t.DestroyOverlay) {
			// SAFETY: DestroyOverlay takes a handle, any handle
			Some(f) => unsafe { f(handle) },
			None => VROverlayError_RequestFailed,
		}
	}

	fn set_overlay_width_in_meters(&mut self, handle: VROverlayHandle_t, metres: f32) -> EVROverlayError {
		match self.ovl(|t| t.SetOverlayWidthInMeters) {
			// SAFETY: SetOverlayWidthInMeters takes a handle and a width, no pointer
			Some(f) => unsafe { f(handle, metres) },
			None => VROverlayError_RequestFailed,
		}
	}

	fn set_overlay_input_method(&mut self, handle: VROverlayHandle_t, method: VROverlayInputMethod) -> EVROverlayError {
		match self.ovl(|t| t.SetOverlayInputMethod) {
			// SAFETY: SetOverlayInputMethod takes a handle and an enum's value, no pointer
			Some(f) => unsafe { f(handle, method) },
			None => VROverlayError_RequestFailed,
		}
	}

	fn set_overlay_mouse_scale(&mut self, handle: VROverlayHandle_t, scale: &HmdVector2_t) -> EVROverlayError {
		let Some(f) = self.ovl(|t| t.SetOverlayMouseScale) else { return VROverlayError_RequestFailed };
		let mut scale = *scale;
		// SAFETY: it reads the vector (a copy, for the header's pointer that is not const)
		unsafe { f(handle, &mut scale) }
	}

	fn set_overlay_flag(&mut self, handle: VROverlayHandle_t, flag: VROverlayFlags, enabled: bool) -> EVROverlayError {
		match self.ovl(|t| t.SetOverlayFlag) {
			// SAFETY: SetOverlayFlag takes a handle, a flag and the C bool's byte, no pointer
			Some(f) => unsafe { f(handle, flag, enabled as u8) },
			None => VROverlayError_RequestFailed,
		}
	}

	fn is_overlay_visible(&mut self, handle: VROverlayHandle_t) -> bool {
		match self.ovl(|t| t.IsOverlayVisible) {
			// SAFETY: IsOverlayVisible takes a handle, any handle
			Some(f) => unsafe { f(handle) != 0 },
			None => false,
		}
	}

	fn poll_next_overlay_event(&mut self, handle: VROverlayHandle_t) -> Option<VrEvent> {
		let f = self.ovl(|t| t.PollNextOverlayEvent)?;
		let mut ev = VREvent_t::zeroed();
		// SAFETY: it writes at most the given size (the struct's) into the event
		let got = unsafe { f(handle, &mut ev, std::mem::size_of::<VREvent_t>() as u32) };
		(got != 0).then(|| VrEvent::of(&ev))
	}

	fn set_overlay_texture(&mut self, handle: VROverlayHandle_t, texture: &Texture_t) -> EVROverlayError {
		let Some(f) = self.ovl(|t| t.SetOverlayTexture) else { return VROverlayError_RequestFailed };
		let mut texture = *texture;
		// SAFETY: it reads the Texture_t (a copy, for the header's pointer that is not const); an OpenGL texture's
		// handle is its name, which SteamVR does not dereference
		unsafe { f(handle, &mut texture) }
	}

	fn show_keyboard_for_overlay(&mut self, handle: VROverlayHandle_t, request: &KeyboardRequest) -> EVROverlayError {
		let Some(f) = self.ovl(|t| t.ShowKeyboardForOverlay) else { return VROverlayError_RequestFailed };
		let mut description = c_arg(&request.description);
		let mut text = c_arg(&request.existing_text);
		// SAFETY: the description and the text are NUL-terminated copies; the rest are numbers
		unsafe {
			f(
				handle,
				request.input_mode,
				request.line_mode,
				request.flags,
				description.as_mut_ptr().cast(),
				request.char_max,
				text.as_mut_ptr().cast(),
				request.user_value,
			)
		}
	}

	fn hide_keyboard(&mut self) {
		if let Some(f) = self.ovl(|t| t.HideKeyboard) {
			// SAFETY: HideKeyboard takes nothing
			unsafe { f() };
		}
	}

	fn keyboard_text(&mut self) -> String {
		let Some(f) = self.ovl(|t| t.GetKeyboardText) else { return String::new() };
		let mut buf = [0u8; 256];
		// SAFETY: it writes at most the given count of chars into the buffer, which has that many
		unsafe { f(buf.as_mut_ptr().cast(), buf.len() as u32) };
		c_text(&buf)
	}
}

#[cfg(windows)]
mod sys {
	#![allow(non_snake_case)]

	use std::ffi::{c_char, c_void, CStr};
	use std::os::windows::ffi::OsStrExt;
	use std::path::Path;

	#[link(name = "kernel32")]
	extern "system" {
		fn GetModuleFileNameW(module: *mut c_void, name: *mut u16, size: u32) -> u32;
		fn LoadLibraryW(name: *const u16) -> *mut c_void;
		fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
	}

	const NAME: &str = "openvr_api.dll";

	/// LoadLibraryW of a NUL-terminated wide path or name
	fn load(wide: &[u16]) -> *mut c_void {
		// SAFETY: the name is NUL-terminated; loading openvr_api runs its DllMain, which is made for that
		unsafe { LoadLibraryW(wide.as_ptr()) }
	}

	/// openvr_api.dll beside the program (its path from GetModuleFileNameW, up to the last backslash), else by name
	pub fn load_library() -> *mut c_void {
		// a path as long as Windows makes one (32767 chars and the NUL)
		let mut path = vec![0u16; 32768];
		// SAFETY: a null module is the program's own; it writes at most the buffer's length of chars into it
		let n = unsafe { GetModuleFileNameW(std::ptr::null_mut(), path.as_mut_ptr(), path.len() as u32) } as usize;
		if n > 0 && n < path.len() {
			if let Some(slash) = path[..n].iter().rposition(|&c| c == u16::from(b'\\')) {
				let mut beside = path[..=slash].to_vec();
				beside.extend(NAME.encode_utf16());
				beside.push(0);
				let h = load(&beside);
				if !h.is_null() {
					return h;
				}
			}
		}
		let name: Vec<u16> = NAME.encode_utf16().chain([0]).collect();
		load(&name)
	}

	/// The library at a path of its own
	pub fn load_path(path: &Path) -> *mut c_void {
		let wide: Vec<u16> = path.as_os_str().encode_wide().chain([0]).collect();
		load(&wide)
	}

	pub fn symbol(lib: *mut c_void, name: &CStr) -> *mut c_void {
		// SAFETY: lib is a module LoadLibraryW returned (never freed), the name NUL-terminated
		unsafe { GetProcAddress(lib, name.as_ptr()) }
	}
}

#[cfg(not(windows))]
mod sys {
	use std::ffi::{c_char, c_int, c_void, CStr, CString};
	use std::os::unix::ffi::OsStrExt;
	use std::path::Path;

	const RTLD_NOW: c_int = 2;

	extern "C" {
		fn dlopen(filename: *const c_char, flag: c_int) -> *mut c_void;
		fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
	}

	/// dlopen(name, RTLD_NOW)
	fn load(name: &CStr) -> *mut c_void {
		// SAFETY: the name is NUL-terminated; loading openvr_api runs its initialisers, which are made for that
		unsafe { dlopen(name.as_ptr(), RTLD_NOW) }
	}

	/// ./libopenvr_api.so, else libopenvr_api.so by name
	pub fn load_library() -> *mut c_void {
		let h = load(c"./libopenvr_api.so");
		if !h.is_null() {
			return h;
		}
		load(c"libopenvr_api.so")
	}

	/// The library at a path of its own
	pub fn load_path(path: &Path) -> *mut c_void {
		match CString::new(path.as_os_str().as_bytes()) {
			Ok(p) => load(&p),
			Err(_) => std::ptr::null_mut(),
		}
	}

	pub fn symbol(lib: *mut c_void, name: &CStr) -> *mut c_void {
		// SAFETY: lib is a handle dlopen returned (never closed), the name NUL-terminated
		unsafe { dlsym(lib, name.as_ptr()) }
	}
}

#[cfg(test)]
mod tests;
