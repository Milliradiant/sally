//! The layouts and values src/openvr.rs gives OpenVR's C API, asserted against openvr_capi.h itself (OpenVR v2.15.6)
//! by the C compilers, for Linux x64 and for Windows x64: the same sizes and offsets as openvr.rs's `const` asserts,
//! and every enum value it copied. The header is C, so the assertions are C ([`CHECKS`]), handed to each C compiler
//! that is installed. Nothing of it runs: it compiles, or a _Static_assert names what moved.
//!
//! It needs an OpenVR checkout, so the test is left out unless it is asked for:
//!
//!   OPENVR_HEADERS=<openvr>/headers cargo test -p sally-panel --test openvr_layout -- --ignored
//!
//! clang and gcc check Linux's layout; x86_64-w64-mingw32-clang (llvm-mingw's, on the PATH) checks Windows': there
//! the header's bool is its own `typedef char bool` (MinGW defines __WIN32) and nothing is packed; on Linux bool is
//! _Bool and VREvent_t is packed to 4. -std=c11 keeps `bool` a name the header may typedef (C23 makes it a keyword).

use std::process::Command;

/// The C compilers asked, each for the platform it builds for
const COMPILERS: [(&str, &str); 3] = [("clang", "Linux"), ("gcc", "Linux"), ("x86_64-w64-mingw32-clang", "Windows")];

/// The assertions, as the C compilers read them
const CHECKS: &str = r##"#include <stddef.h>

#include <openvr_capi.h>

#define SIZE(t, n) _Static_assert(sizeof(t) == (n), "sizeof(" #t ")")
#define ALIGN(t, n) _Static_assert(_Alignof(t) == (n), "_Alignof(" #t ")")
#define AT(t, f, n) _Static_assert(offsetof(t, f) == (n), "offsetof(" #t ", " #f ")")
#define VALUE(e, n) _Static_assert((e) == (n), #e)

/* a C bool is one byte, on both */
SIZE(bool, 1);

SIZE(HmdMatrix34_t, 48);
SIZE(HmdMatrix44_t, 64);
SIZE(HmdVector2_t, 8);
SIZE(HmdVector3_t, 12);

SIZE(TrackedDevicePose_t, 80);
AT(TrackedDevicePose_t, vVelocity, 48);
AT(TrackedDevicePose_t, vAngularVelocity, 60);
AT(TrackedDevicePose_t, eTrackingResult, 72);
AT(TrackedDevicePose_t, bPoseIsValid, 76);
AT(TrackedDevicePose_t, bDeviceIsConnected, 77);

SIZE(Texture_t, 16);
AT(Texture_t, eType, 8);
AT(Texture_t, eColorSpace, 12);

SIZE(HiddenAreaMesh_t, 16);
AT(HiddenAreaMesh_t, unTriangleCount, 8);

SIZE(VREvent_Mouse_t, 16);
AT(VREvent_Mouse_t, y, 4);
AT(VREvent_Mouse_t, button, 8);
SIZE(VREvent_Scroll_t, 20);
AT(VREvent_Scroll_t, ydelta, 4);
SIZE(VREvent_Keyboard_t, 24);
AT(VREvent_Keyboard_t, uUserValue, 8);
AT(VREvent_Keyboard_t, overlayHandle, 16);
SIZE(VREvent_Reserved_t, 48);
/* the union's every member, its largest the 48 bytes openvr.rs gives it with `reserved` */
SIZE(VREvent_Data_t, 48);
ALIGN(VREvent_Data_t, 8);

AT(struct VREvent_t, trackedDeviceIndex, 4);
AT(struct VREvent_t, eventAgeSeconds, 8);
#if defined(__linux__) || defined(__APPLE__)
SIZE(struct VREvent_t, 60);
ALIGN(struct VREvent_t, 4);
AT(struct VREvent_t, data, 12);
#else
SIZE(struct VREvent_t, 64);
ALIGN(struct VREvent_t, 8);
AT(struct VREvent_t, data, 16);
#endif

/* the tables, to the last entry openvr.rs declares (the size: that entry's end) */
AT(struct VR_IVRSystem_FnTable, GetDeviceToAbsoluteTrackingPose, 96);
AT(struct VR_IVRSystem_FnTable, GetTrackedDeviceClass, 160);
AT(struct VR_IVRSystem_FnTable, GetStringTrackedDeviceProperty, 224);
AT(struct VR_IVRSystem_FnTable, PollNextEvent, 240);
AT(struct VR_IVRSystem_FnTable, GetRawZeroPoseToStandingAbsoluteTrackingPose, 112);
AT(struct VR_IVRSystem_FnTable, TriggerHapticPulse, 312);
AT(struct VR_IVRSystem_FnTable, AcknowledgeQuit_Exiting, 376);
AT(struct VR_IVROverlay_FnTable, DestroyOverlay, 24);
AT(struct VR_IVROverlay_FnTable, SetOverlayFlag, 88);
AT(struct VR_IVROverlay_FnTable, SetOverlayWidthInMeters, 176);
AT(struct VR_IVROverlay_FnTable, IsOverlayVisible, 360);
AT(struct VR_IVROverlay_FnTable, PollNextOverlayEvent, 384);
AT(struct VR_IVROverlay_FnTable, SetOverlayInputMethod, 400);
AT(struct VR_IVROverlay_FnTable, SetOverlayMouseScale, 416);
AT(struct VR_IVROverlay_FnTable, SetOverlayTexture, 480);
AT(struct VR_IVROverlay_FnTable, CreateDashboardOverlay, 536);
AT(struct VR_IVROverlay_FnTable, ShowKeyboardForOverlay, 600);
AT(struct VR_IVROverlay_FnTable, GetKeyboardText, 608);
AT(struct VR_IVROverlay_FnTable, HideKeyboard, 616);
AT(struct VR_IVRSettings_FnTable, GetString, 64);
SIZE(HmdQuad_t, 48);
AT(struct VR_IVRChaperoneSetup_FnTable, GetLiveCollisionBoundsInfo, 40);
SIZE(EChaperoneConfigFile, 4);

/* the enum values openvr.rs copied (an enum is int-sized, on both; the header's other constants, as
 * k_unMaxTrackedDeviceCount and the interface versions, are static variables, which C cannot assert on) */
SIZE(EVROverlayError, 4);
SIZE(ETrackedDeviceClass, 4);
VALUE(EVRInitError_VRInitError_None, 0);
VALUE(EVRInitError_VRInitError_Init_InterfaceNotFound, 105);
VALUE(EVRInitError_VRInitError_Init_NotInitialized, 109);
VALUE(EVRInitError_VRInitError_Init_NoServerForBackgroundApp, 121);
VALUE(EVRApplicationType_VRApplication_Overlay, 2);
VALUE(EVRApplicationType_VRApplication_Background, 3);
VALUE(ETrackedDeviceClass_TrackedDeviceClass_Invalid, 0);
VALUE(ETrackedDeviceClass_TrackedDeviceClass_HMD, 1);
VALUE(ETrackedDeviceClass_TrackedDeviceClass_Controller, 2);
VALUE(ETrackedDeviceClass_TrackedDeviceClass_GenericTracker, 3);
VALUE(ETrackedDeviceClass_TrackedDeviceClass_TrackingReference, 4);
VALUE(ETrackedDeviceProperty_Prop_TrackingSystemName_String, 1000);
VALUE(ETrackedDeviceProperty_Prop_SerialNumber_String, 1002);
VALUE(ETrackedDeviceProperty_Prop_ResourceRoot_String, 1035);
VALUE(ETrackedDeviceProperty_Prop_RegisteredDeviceType_String, 1036);
VALUE(ETrackedPropertyError_TrackedProp_Success, 0);
VALUE(ETrackingUniverseOrigin_TrackingUniverseRawAndUncalibrated, 2);
VALUE(ETrackingResult_TrackingResult_Running_OK, 200);
VALUE(EVREventType_VREvent_MouseMove, 300);
VALUE(EVREventType_VREvent_MouseButtonDown, 301);
VALUE(EVREventType_VREvent_MouseButtonUp, 302);
VALUE(EVREventType_VREvent_FocusLeave, 304);
VALUE(EVREventType_VREvent_ScrollDiscrete, 305);
VALUE(EVREventType_VREvent_ScrollSmooth, 309);
VALUE(EVREventType_VREvent_Quit, 700);
VALUE(EVREventType_VREvent_KeyboardClosed, 1200);
VALUE(EVREventType_VREvent_KeyboardDone, 1202);
VALUE(EVREventType_VREvent_KeyboardClosed_Global, 1204);
VALUE(EVRMouseButton_VRMouseButton_Left, 1);
VALUE(EVRMouseButton_VRMouseButton_Right, 2);
VALUE(EVRMouseButton_VRMouseButton_Middle, 4);
VALUE(EVROverlayError_VROverlayError_None, 0);
VALUE(EVROverlayError_VROverlayError_KeyInUse, 17);
VALUE(EVROverlayError_VROverlayError_RequestFailed, 23);
VALUE(EVROverlayError_VROverlayError_KeyboardAlreadyInUse, 26);
VALUE(VROverlayInputMethod_Mouse, 1);
VALUE(VROverlayFlags_SendVRSmoothScrollEvents, 131072);
VALUE(VROverlayFlags_IsPremultiplied, 2097152);
VALUE(ETextureType_TextureType_OpenGL, 1);
VALUE(EColorSpace_ColorSpace_Gamma, 1);
VALUE(EGamepadTextInputMode_k_EGamepadTextInputModeNormal, 0);
VALUE(EGamepadTextInputLineMode_k_EGamepadTextInputLineModeSingleLine, 0);
VALUE(EKeyboardFlags_KeyboardFlag_Modal, 2);
VALUE(EVRSettingsError_VRSettingsError_None, 0);
VALUE(EVRSettingsError_VRSettingsError_IPCFailed, 1);
VALUE(EVRSettingsError_VRSettingsError_UnsetSettingHasNoDefault, 5);
"##;

#[test]
#[ignore = "needs OpenVR's headers: OPENVR_HEADERS=<openvr>/headers"]
fn openvr_rs_lays_out_what_the_header_does() {
	let headers = std::env::var_os("OPENVR_HEADERS").expect("OPENVR_HEADERS=<openvr>/headers");
	let dir = std::env::temp_dir().join(format!("sally-openvr-layout-{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let source = dir.join("openvr_layout.c");
	std::fs::write(&source, CHECKS).unwrap();
	let mut checked = Vec::new();
	let mut failed = Vec::new();
	for (compiler, platform) in COMPILERS {
		// a compiler that is not installed checks nothing
		let Ok(out) =
			Command::new(compiler).args(["-std=c11", "-fsyntax-only", "-I"]).arg(&headers).arg(&source).output()
		else {
			continue;
		};
		if out.status.success() {
			checked.push(format!("{compiler} ({platform})"));
		} else {
			failed.push(format!("{compiler} ({platform}):\n{}", String::from_utf8_lossy(&out.stderr)));
		}
	}
	let _ = std::fs::remove_dir_all(&dir);
	assert!(failed.is_empty(), "{}", failed.join("\n"));
	assert!(!checked.is_empty(), "no C compiler found (clang, gcc, x86_64-w64-mingw32-clang)");
	println!("openvr.rs's layouts are the header's by {}", checked.join(", "));
}
