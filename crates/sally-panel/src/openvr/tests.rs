use super::*;
use crate::runtime::Runtime;
use crate::valve::ValveCompare;

#[test]
fn without_a_library_nothing_is_called() {
	let mut vr = OpenVr::with_library("/nonexistent/libopenvr_api.so");
	assert_eq!(vr.load(), Err(LoadError::Missing));
	assert_eq!(vr.init(VRApplication_Background), Err(VRInitError_Init_NotInitialized));
	assert_eq!(vr.interface(Interface::System), Err(VRInitError_Init_NotInitialized));
	assert!(!vr.has(Interface::System) && !vr.has(Interface::Overlay) && !vr.has(Interface::Settings));
	// without a table every call answers nothing
	assert_eq!(vr.tracked_device_class(0), TrackedDeviceClass_Invalid);
	assert_eq!(vr.string_tracked_device_property(0, Prop_SerialNumber_String), "");
	assert!(vr.poll_next_event().is_none());
	assert_eq!(vr.settings_string("trackers", "x"), Err(VRSettingsError_IPCFailed));
	assert_eq!(vr.create_dashboard_overlay("sally.panel", "sally"), Err(VROverlayError_RequestFailed));
	assert!(!vr.is_overlay_visible(1));
	assert!(vr.poll_next_overlay_event(1).is_none());
	assert_eq!(vr.keyboard_text(), "");
	assert!(!vr.shutdown());
}

#[cfg(target_os = "linux")]
#[test]
fn a_library_without_openvrs_entry_points_is_not_usable() {
	// the C library: a shared library wherever the tests run, with nothing of OpenVR's
	let mut vr = OpenVr::with_library("libc.so.6");
	assert_eq!(vr.load(), Err(LoadError::Unusable));
}

#[test]
fn the_panel_without_a_library_says_so() {
	let mut vc = ValveCompare::with_vr(Box::new(OpenVr::with_library("/nonexistent/libopenvr_api.so")));
	vc.driver_port = 0;
	vc.poll(0.0, &mut Runtime::new());
	assert_eq!(vc.status, "openvr_api library not found next to the panel");
	assert_eq!(vc.overlay_status, "no openvr_api library");
}

#[test]
fn an_events_data_is_read_three_ways_from_the_same_bytes() {
	let mut ev = VREvent_t::zeroed();
	ev.eventType = VREvent_KeyboardDone;
	ev.data.keyboard = VREvent_Keyboard_t { cNewInput: [0; 8], uUserValue: 0x1_0000_0007, overlayHandle: 5 };
	let e = VrEvent::of(&ev);
	assert_eq!(e.event_type, VREvent_KeyboardDone);
	assert_eq!((e.keyboard.uUserValue, e.keyboard.overlayHandle), (0x1_0000_0007, 5));
	// mouse.button and cursorIndex are uUserValue's bytes, scroll's unused and viewportscale too
	assert_eq!((e.mouse.button, e.mouse.cursorIndex), (7, 1));
	assert_eq!(e.scroll.unused, 7);
}
