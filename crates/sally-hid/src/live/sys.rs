//! The Windows API the live tap calls, declared as windows.h, hidsdi.h, hidpi.h and setupapi.h declare it (x86-64:
//! setupapi's structures with 8-byte packing, which is their natural layout), the ANSI functions where the C calls
//! them; and what `sally probe` (hidprobe.c) calls besides: a device's product string, the process list (tlhelp32.h),
//! a system error's text. Nothing here writes to a device: HidD_GetFeature reads a feature report.

#![allow(non_snake_case, non_camel_case_types, clippy::upper_case_acronyms)]

use std::ffi::c_void;

pub type BOOL = i32;
pub type BOOLEAN = u8;
pub type DWORD = u32;
pub type HANDLE = *mut c_void;
pub type HDEVINFO = *mut c_void;
pub type NTSTATUS = i32;
pub type PHIDP_PREPARSED_DATA = *mut c_void;

pub const FALSE: BOOL = 0;
pub const TRUE: BOOL = 1;
pub const INVALID_HANDLE_VALUE: HANDLE = -1isize as HANDLE;

pub const GENERIC_READ: DWORD = 0x8000_0000;
pub const GENERIC_WRITE: DWORD = 0x4000_0000;
pub const FILE_SHARE_READ: DWORD = 0x1;
pub const FILE_SHARE_WRITE: DWORD = 0x2;
pub const OPEN_EXISTING: DWORD = 3;
pub const FILE_FLAG_OVERLAPPED: DWORD = 0x4000_0000;

pub const ERROR_OPERATION_ABORTED: DWORD = 995;
pub const ERROR_IO_INCOMPLETE: DWORD = 996;
pub const ERROR_IO_PENDING: DWORD = 997;
pub const ERROR_DEVICE_NOT_CONNECTED: DWORD = 1167;

pub const WAIT_OBJECT_0: DWORD = 0;
pub const WAIT_TIMEOUT: DWORD = 0x102;

pub const DIGCF_PRESENT: DWORD = 0x2;
pub const DIGCF_DEVICEINTERFACE: DWORD = 0x10;

/// HIDP_ERROR_CODES(0x0, 0): FACILITY_HID_ERROR_CODE 0x11
pub const HIDP_STATUS_SUCCESS: NTSTATUS = 0x0011_0000;

pub const TH32CS_SNAPPROCESS: DWORD = 0x2;
pub const FORMAT_MESSAGE_IGNORE_INSERTS: DWORD = 0x200;
pub const FORMAT_MESSAGE_FROM_SYSTEM: DWORD = 0x1000;
pub const MAX_PATH: usize = 260;

/// PROCESSENTRY32, the ANSI one (its name a CHAR array)
#[repr(C)]
pub struct PROCESSENTRY32 {
	pub dwSize: DWORD,
	pub cntUsage: DWORD,
	pub th32ProcessID: DWORD,
	pub th32DefaultHeapID: usize,
	pub th32ModuleID: DWORD,
	pub cntThreads: DWORD,
	pub th32ParentProcessID: DWORD,
	pub pcPriClassBase: i32,
	pub dwFlags: DWORD,
	pub szExeFile: [u8; MAX_PATH],
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct GUID {
	pub Data1: u32,
	pub Data2: u16,
	pub Data3: u16,
	pub Data4: [u8; 8],
}

/// OVERLAPPED, its Offset / OffsetHigh union as the two offsets
#[repr(C)]
pub struct OVERLAPPED {
	pub Internal: usize,
	pub InternalHigh: usize,
	pub Offset: DWORD,
	pub OffsetHigh: DWORD,
	pub hEvent: HANDLE,
}

#[repr(C)]
#[derive(Default)]
pub struct HIDD_ATTRIBUTES {
	pub Size: u32,
	pub VendorID: u16,
	pub ProductID: u16,
	pub VersionNumber: u16,
}

#[repr(C)]
#[derive(Default)]
pub struct HIDP_CAPS {
	pub Usage: u16,
	pub UsagePage: u16,
	pub InputReportByteLength: u16,
	pub OutputReportByteLength: u16,
	pub FeatureReportByteLength: u16,
	pub Reserved: [u16; 17],
	pub NumberLinkCollectionNodes: u16,
	pub NumberInputButtonCaps: u16,
	pub NumberInputValueCaps: u16,
	pub NumberInputDataIndices: u16,
	pub NumberOutputButtonCaps: u16,
	pub NumberOutputValueCaps: u16,
	pub NumberOutputDataIndices: u16,
	pub NumberFeatureButtonCaps: u16,
	pub NumberFeatureValueCaps: u16,
	pub NumberFeatureDataIndices: u16,
}

#[repr(C)]
pub struct SP_DEVICE_INTERFACE_DATA {
	pub cbSize: DWORD,
	pub InterfaceClassGuid: GUID,
	pub Flags: DWORD,
	pub Reserved: usize,
}

/// SP_DEVICE_INTERFACE_DETAIL_DATA_A: its size (cbSize) is 8 on x86-64; the path runs on past the struct
#[repr(C)]
pub struct SP_DEVICE_INTERFACE_DETAIL_DATA_A {
	pub cbSize: DWORD,
	pub DevicePath: [u8; 1],
}

#[link(name = "kernel32")]
extern "system" {
	pub fn CreateFileA(
		lpFileName: *const u8, dwDesiredAccess: DWORD, dwShareMode: DWORD, lpSecurityAttributes: *mut c_void,
		dwCreationDisposition: DWORD, dwFlagsAndAttributes: DWORD, hTemplateFile: HANDLE,
	) -> HANDLE;
	pub fn CloseHandle(hObject: HANDLE) -> BOOL;
	pub fn CancelIo(hFile: HANDLE) -> BOOL;
	pub fn ReadFile(
		hFile: HANDLE, lpBuffer: *mut c_void, nNumberOfBytesToRead: DWORD, lpNumberOfBytesRead: *mut DWORD,
		lpOverlapped: *mut OVERLAPPED,
	) -> BOOL;
	pub fn GetOverlappedResult(
		hFile: HANDLE, lpOverlapped: *mut OVERLAPPED, lpNumberOfBytesTransferred: *mut DWORD, bWait: BOOL,
	) -> BOOL;
	pub fn CreateEventA(
		lpEventAttributes: *mut c_void, bManualReset: BOOL, bInitialState: BOOL, lpName: *const u8,
	) -> HANDLE;
	pub fn ResetEvent(hEvent: HANDLE) -> BOOL;
	pub fn WaitForSingleObject(hHandle: HANDLE, dwMilliseconds: DWORD) -> DWORD;
	pub fn WaitForMultipleObjects(
		nCount: DWORD, lpHandles: *const HANDLE, bWaitAll: BOOL, dwMilliseconds: DWORD,
	) -> DWORD;
	pub fn GetLastError() -> DWORD;
	pub fn Sleep(dwMilliseconds: DWORD);
	pub fn CreateToolhelp32Snapshot(dwFlags: DWORD, th32ProcessID: DWORD) -> HANDLE;
	pub fn Process32First(hSnapshot: HANDLE, lppe: *mut PROCESSENTRY32) -> BOOL;
	pub fn Process32Next(hSnapshot: HANDLE, lppe: *mut PROCESSENTRY32) -> BOOL;
	pub fn FormatMessageA(
		dwFlags: DWORD, lpSource: *const c_void, dwMessageId: DWORD, dwLanguageId: DWORD, lpBuffer: *mut u8,
		nSize: DWORD, Arguments: *mut c_void,
	) -> DWORD;
}

#[link(name = "hid")]
extern "system" {
	pub fn HidD_GetHidGuid(HidGuid: *mut GUID);
	pub fn HidD_GetAttributes(HidDeviceObject: HANDLE, Attributes: *mut HIDD_ATTRIBUTES) -> BOOLEAN;
	pub fn HidD_GetSerialNumberString(HidDeviceObject: HANDLE, Buffer: *mut c_void, BufferLength: u32) -> BOOLEAN;
	pub fn HidD_GetProductString(HidDeviceObject: HANDLE, Buffer: *mut c_void, BufferLength: u32) -> BOOLEAN;
	pub fn HidD_GetPreparsedData(HidDeviceObject: HANDLE, PreparsedData: *mut PHIDP_PREPARSED_DATA) -> BOOLEAN;
	pub fn HidD_FreePreparsedData(PreparsedData: PHIDP_PREPARSED_DATA) -> BOOLEAN;
	pub fn HidD_SetNumInputBuffers(HidDeviceObject: HANDLE, NumberBuffers: u32) -> BOOLEAN;
	pub fn HidD_GetFeature(HidDeviceObject: HANDLE, ReportBuffer: *mut c_void, ReportBufferLength: u32) -> BOOLEAN;
	pub fn HidP_GetCaps(PreparsedData: PHIDP_PREPARSED_DATA, Capabilities: *mut HIDP_CAPS) -> NTSTATUS;
}

#[link(name = "setupapi")]
extern "system" {
	pub fn SetupDiGetClassDevsA(
		ClassGuid: *const GUID, Enumerator: *const u8, hwndParent: *mut c_void, Flags: DWORD,
	) -> HDEVINFO;
	pub fn SetupDiEnumDeviceInterfaces(
		DeviceInfoSet: HDEVINFO, DeviceInfoData: *mut c_void, InterfaceClassGuid: *const GUID, MemberIndex: DWORD,
		DeviceInterfaceData: *mut SP_DEVICE_INTERFACE_DATA,
	) -> BOOL;
	pub fn SetupDiGetDeviceInterfaceDetailA(
		DeviceInfoSet: HDEVINFO, DeviceInterfaceData: *mut SP_DEVICE_INTERFACE_DATA,
		DeviceInterfaceDetailData: *mut SP_DEVICE_INTERFACE_DETAIL_DATA_A, DeviceInterfaceDetailDataSize: DWORD,
		RequiredSize: *mut DWORD, DeviceInfoData: *mut c_void,
	) -> BOOL;
	pub fn SetupDiDestroyDeviceInfoList(DeviceInfoSet: HDEVINFO) -> BOOL;
}

// the layouts the headers give on x86-64
const _: () = assert!(std::mem::size_of::<OVERLAPPED>() == 32);
const _: () = assert!(std::mem::size_of::<HIDD_ATTRIBUTES>() == 12);
const _: () = assert!(std::mem::size_of::<HIDP_CAPS>() == 64);
const _: () = assert!(std::mem::size_of::<SP_DEVICE_INTERFACE_DATA>() == 32);
const _: () = assert!(std::mem::size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_A>() == 8);
const _: () = assert!(std::mem::size_of::<PROCESSENTRY32>() == 304);
const _: () = assert!(std::mem::offset_of!(PROCESSENTRY32, szExeFile) == 44);
