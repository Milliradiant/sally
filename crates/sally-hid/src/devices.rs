//! The known lighthouse devices and the interface a report arrives on (the C runtime's survive_vive_report.c)

/// One kind of device: its USB ids, the codename prefix its objects get, and whether it is tracked (a device without a
/// config file name, the HTC headset's mainboard, gets no object)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeviceType {
	pub vid: u16,
	pub pid: u16,
	pub codename: &'static str,
	pub def_config: Option<&'static str>,
}

/// survive_vive_devices, in its order (the index is the C's `type - survive_vive_devices`, which numbers the codenames)
pub const DEVICES: [DeviceType; 7] = [
	DeviceType { vid: 0x28de, pid: 0x2000, codename: "HMD", def_config: Some("HMD_config.json") },
	DeviceType { vid: 0x0bb4, pid: 0x2c87, codename: "BD", def_config: None },
	DeviceType { vid: 0x28de, pid: 0x2101, codename: "WM", def_config: Some("WM%d_config.json") },
	DeviceType { vid: 0x28de, pid: 0x2022, codename: "TR", def_config: Some("TR%d_config.json") },
	DeviceType { vid: 0x28de, pid: 0x2300, codename: "T2", def_config: Some("T2%d_config.json") },
	DeviceType { vid: 0x28de, pid: 0x2012, codename: "WW", def_config: Some("WW%d_config.json") },
	DeviceType { vid: 0x28de, pid: 0x2102, codename: "KN", def_config: Some("KN%d_config.json") },
];

/// survive_vive_device_lookup: the index into DEVICES
pub fn lookup(vid: u16, pid: u16) -> Option<usize> {
	DEVICES.iter().position(|d| d.vid == vid && d.pid == pid)
}

/// driver_vive.h's USB_IF_t: which part of a device a report belongs to
pub mod usb_if {
	pub const HMD_HEADSET_INFO: i32 = 1;
	pub const HMD_IMU: i32 = 2;
	pub const WATCHMAN1: i32 = 3;
	pub const WATCHMAN2: i32 = 4;
	pub const TRACKER0_IMU: i32 = 5;
	pub const TRACKER_INFO: i32 = 6;
	pub const TRACKER1_IMU: i32 = 7;
	pub const W_WATCHMAN1_IMU: i32 = 8;
	pub const HMD_LIGHTCAP: i32 = 9;
	pub const TRACKER0_LIGHTCAP: i32 = 10;
	pub const TRACKER1_LIGHTCAP: i32 = 11;
	pub const W_WATCHMAN1_LIGHTCAP: i32 = 12;
	pub const HMD_BUTTONS: i32 = 13;
	pub const TRACKER0_BUTTONS: i32 = 14;
	pub const TRACKER1_BUTTONS: i32 = 15;
	pub const W_WATCHMAN1_BUTTONS: i32 = 16;
}

/// survive_vive_report_interface_from_endpoint: the interface of a report on a USB endpoint (0 = none known)
pub fn interface_from_endpoint(pid: u16, endpoint: i32) -> i32 {
	use usb_if::*;
	let id = (pid as i32).wrapping_add(endpoint << 16);
	match id {
		0x812000 => HMD_IMU,
		0x812101 => WATCHMAN1,
		0x812022 => TRACKER0_IMU,
		0x812300 => TRACKER1_IMU,
		0x812012 => W_WATCHMAN1_IMU,
		0x802300 | 0x802101 => TRACKER_INFO,
		0x822000 => HMD_LIGHTCAP,
		0x822101 | 0x812102 => WATCHMAN1,
		0x822022 => TRACKER0_LIGHTCAP,
		0x822300 => TRACKER1_LIGHTCAP,
		0x822012 => W_WATCHMAN1_LIGHTCAP,
		0x832000 => HMD_BUTTONS,
		0x832101 => W_WATCHMAN1_BUTTONS,
		0x832022 => TRACKER0_BUTTONS,
		0x832300 => TRACKER1_LIGHTCAP,
		0x832012 => W_WATCHMAN1_BUTTONS,
		0x842300 => TRACKER1_BUTTONS,
		_ => 0,
	}
}

/// survive_vive_report_interface_from_hid: a Windows HID path's interface number (mi_XX, -1 if the path has none) is
/// endpoint 0x81 + mi on these devices; the collection (colXX) does not change it
pub fn interface_from_hid(pid: u16, mi: i32, _col: i32) -> i32 {
	let mi = if mi < 0 { 0 } else { mi };
	interface_from_endpoint(pid, 0x81 + mi)
}

/// USB_IF_t's names (survive_usb_interface_str)
pub fn interface_str(iface: i32) -> &'static str {
	use usb_if::*;
	match iface {
		HMD_HEADSET_INFO => "USB_IF_HMD_HEADSET_INFO",
		HMD_IMU => "USB_IF_HMD_IMU",
		WATCHMAN1 => "USB_IF_WATCHMAN1",
		WATCHMAN2 => "USB_IF_WATCHMAN2",
		TRACKER0_IMU => "USB_IF_TRACKER0_IMU",
		TRACKER_INFO => "USB_IF_TRACKER_INFO",
		TRACKER1_IMU => "USB_IF_TRACKER1_IMU",
		W_WATCHMAN1_IMU => "USB_IF_W_WATCHMAN1_IMU",
		HMD_LIGHTCAP => "USB_IF_HMD_LIGHTCAP",
		TRACKER0_LIGHTCAP => "USB_IF_TRACKER0_LIGHTCAP",
		TRACKER1_LIGHTCAP => "USB_IF_TRACKER1_LIGHTCAP",
		W_WATCHMAN1_LIGHTCAP => "USB_IF_W_WATCHMAN1_LIGHTCAP",
		HMD_BUTTONS => "USB_IF_HMD_BUTTONS",
		TRACKER0_BUTTONS => "USB_IF_TRACKER0_BUTTONS",
		TRACKER1_BUTTONS => "USB_IF_TRACKER1_BUTTONS",
		W_WATCHMAN1_BUTTONS => "USB_IF_W_WATCHMAN1_BUTTONS",
		_ => "UNKNOWN",
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn dongle_reports_are_watchman() {
		assert_eq!(lookup(0x28de, 0x2102), Some(6));
		assert_eq!(interface_from_hid(0x2102, -1, -1), usb_if::WATCHMAN1);
		assert_eq!(interface_from_hid(0x2101, 0, 1), usb_if::WATCHMAN1);
		assert_eq!(interface_from_endpoint(0x2300, 0x80), usb_if::TRACKER_INFO);
		assert_eq!(interface_from_endpoint(0x2102, 0x85), 0);
	}
}
