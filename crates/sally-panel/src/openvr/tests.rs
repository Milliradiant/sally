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

/// openvr_api's calls made through this module against the C++ panel's fake library (fake_openvr.cpp, in git: `git show
/// 92d8db1:unported/tools/sally_panel/test/fake_openvr.cpp > fake_openvr.cpp`), compiled against openvr_capi.h: the
/// tables, the poses and the events as the C compiler lays them out meet this module's, through the loader. It needs
/// that library, built and named by SALLY_FAKE_OPENVR:
///
/// ```text
/// g++ -std=c++17 -shared -fPIC -I<openvr>/headers fake_openvr.cpp \
///     -o <dir>/libopenvr_api.so -pthread
/// SALLY_FAKE_OPENVR=<dir>/libopenvr_api.so cargo test -p sally-panel -- --ignored cpp_fake
/// ```
#[test]
#[ignore = "needs the C++ fake built, named by SALLY_FAKE_OPENVR"]
fn the_cpp_fake_through_the_loader() {
	use std::net::UdpSocket;
	use std::time::{Duration, Instant};

	use sally_math::math3d::{compose, len, qangle, qconj, qfrom_mat, qmul, Pose, Q4, V3};

	let path = std::env::var("SALLY_FAKE_OPENVR").expect("SALLY_FAKE_OPENVR: the C++ fake's libopenvr_api.so");
	// the fake takes its devices' poses from a runtime: this test plays it
	let runtime = UdpSocket::bind("127.0.0.1:0").unwrap();
	runtime.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
	std::env::set_var("FAKE_OPENVR_PORT", runtime.local_addr().unwrap().port().to_string());
	std::env::set_var("FAKE_OPENVR_KEYBOARD", "Left shin");
	std::env::remove_var("FAKE_OPENVR_NOISE_MM");

	let mut vr = OpenVr::with_library(path);
	assert_eq!(vr.load(), Ok(()));
	assert_eq!(vr.init(VRApplication_Background), Ok(()));
	for which in [Interface::System, Interface::Overlay, Interface::Settings] {
		assert_eq!(vr.interface(which), Ok(()));
		assert!(vr.has(which));
	}

	// a controller at (0.1, 0.2, 1.0), turned 90 degrees about Z, as a runtime's DEV and Q lines give it
	let mut buf = [0u8; 256];
	let (_, fake) = runtime.recv_from(&mut buf).expect("the fake subscribes");
	let s = 0.5f64.sqrt();
	runtime.send_to(b"#fd DEV KN0 LHR-FAKE 22 tracking 0.004 knuckles_r", fake).unwrap();
	runtime.send_to(format!("Q KN0 LHR-FAKE 1000 1.0 0.1 0.2 1.0 {s} 0 0 {s}").as_bytes(), fake).unwrap();
	let mut poses = [TrackedDevicePose_t::default(); k_unMaxTrackedDeviceCount as usize];
	let t0 = Instant::now();
	loop {
		vr.device_to_absolute_tracking_pose(TrackingUniverseRawAndUncalibrated, 0.0, &mut poses);
		if poses[0].bPoseIsValid != 0 && vr.tracked_device_class(0) == TrackedDeviceClass_Controller {
			break;
		}
		assert!(t0.elapsed() < Duration::from_secs(5), "the fake got no device");
		std::thread::sleep(Duration::from_millis(5));
	}
	assert_eq!(vr.tracked_device_class(1), TrackedDeviceClass_Invalid);
	assert_eq!(vr.string_tracked_device_property(0, Prop_TrackingSystemName_String), "lighthouse");
	assert_eq!(vr.string_tracked_device_property(0, Prop_SerialNumber_String), "LHR-FAKE");
	assert_eq!(vr.string_tracked_device_property(0, Prop_ResourceRoot_String), "");
	let p = &poses[0];
	assert_eq!((p.bDeviceIsConnected, p.eTrackingResult), (1, TrackingResult_Running_OK));
	assert_eq!(poses[1], TrackedDevicePose_t::default());
	// the fake's frame: Z up to Y up, 30 degrees of yaw, shifted
	let device = Pose { p: V3::new(0.1, 0.2, 1.0), q: Q4::new(s, 0.0, 0.0, s) };
	let want = compose(&crate::valve::fake::cpp_fake_frame(), &device);
	let m = &p.mDeviceToAbsoluteTracking.m;
	let at = V3::new(m[0][3] as f64, m[1][3] as f64, m[2][3] as f64);
	assert!(len(at - want.p) < 1e-5, "{at:?} against {:?}", want.p);
	let r = [0, 1, 2].map(|i| [0, 1, 2].map(|j| m[i][j] as f64));
	assert!(qangle(qmul(qconj(qfrom_mat(&r)), want.q)) < 1e-5);

	// no role for a controller, no events
	let key = "/devices/htc/vive_trackerLHR-FAKE";
	assert_eq!(vr.settings_string("trackers", key), Err(VRSettingsError_UnsetSettingHasNoDefault));
	assert!(vr.poll_next_event().is_none());

	// the overlay and the keyboard, whose events come back through VREvent_t
	assert_eq!(vr.create_dashboard_overlay("sally.panel", "sally"), Ok((1, 2)));
	assert_eq!(vr.set_overlay_width_in_meters(1, 2.5), VROverlayError_None);
	assert_eq!(vr.set_overlay_input_method(1, VROverlayInputMethod_Mouse), VROverlayError_None);
	assert_eq!(vr.set_overlay_mouse_scale(1, &HmdVector2_t { v: [2000.0, 1175.0] }), VROverlayError_None);
	assert_eq!(vr.set_overlay_flag(1, VROverlayFlags_SendVRSmoothScrollEvents, true), VROverlayError_None);
	assert!(!vr.is_overlay_visible(1));
	let texture = Texture_t {
		handle: std::ptr::without_provenance_mut(7),
		eType: TextureType_OpenGL,
		eColorSpace: ColorSpace_Gamma,
	};
	assert_eq!(vr.set_overlay_texture(1, &texture), VROverlayError_None);
	let request = KeyboardRequest {
		input_mode: k_EGamepadTextInputModeNormal,
		line_mode: k_EGamepadTextInputLineModeSingleLine,
		flags: KeyboardFlag_Modal,
		description: b"Name for KN0".to_vec(),
		char_max: 32,
		existing_text: b"Right controller".to_vec(),
		user_value: 77,
	};
	assert_eq!(vr.show_keyboard_for_overlay(1, &request), VROverlayError_None);
	let done = vr.poll_next_overlay_event(1).expect("the keyboard's Done");
	assert_eq!(done.event_type, VREvent_KeyboardDone);
	assert_eq!((done.keyboard.uUserValue, done.keyboard.overlayHandle), (77, 1));
	let closed = vr.poll_next_overlay_event(1).expect("the keyboard's Closed");
	assert_eq!((closed.event_type, closed.keyboard.uUserValue), (VREvent_KeyboardClosed, 77));
	assert!(vr.poll_next_overlay_event(1).is_none());
	assert_eq!(vr.keyboard_text(), "Left shin");
	vr.hide_keyboard();
	vr.acknowledge_quit_exiting();
	assert_eq!(vr.destroy_overlay(1), VROverlayError_None);
	assert!(vr.shutdown());
	assert!(!vr.has(Interface::System));
}
