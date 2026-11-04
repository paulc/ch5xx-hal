//! GATT server bindings: attribute tables, service registration, client
//! characteristic configuration, notifications, the GAP GATT service (GGS)
//! and the link database.
//!
//! Hand-written from the library headers (see [`super::ffi`]); only the parts
//! used so far are bound.

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, dead_code)]

use core::ffi::c_void;

use super::ffi::{bStatus_t, tmos_event_hdr_t};

/* Connection handles */
pub const INVALID_CONNHANDLE: u16 = 0xFFFF;
pub const LOOPBACK_CONNHANDLE: u16 = 0xFFFE;

/* ATT */

/// Minimum ATT MTU size
pub const ATT_MTU_SIZE: u16 = 23;
pub const ATT_BT_UUID_SIZE: u8 = 2;
pub const ATT_UUID_SIZE: u8 = 16;

/* ATT methods (opcodes) */
pub const ATT_READ_REQ: u8 = 0x0A;
pub const ATT_WRITE_REQ: u8 = 0x12;
pub const ATT_HANDLE_VALUE_NOTI: u8 = 0x1B;
pub const ATT_WRITE_CMD: u8 = 0x52;
/// GATT_MSG_EVENT method sent when the MTU is updated (msg is `attMtuUpdatedEvt_t`)
pub const ATT_MTU_UPDATED_EVENT: u8 = 0x7F;

/* ATT_ERR_CODE_DEFINES */
pub const ATT_ERR_INVALID_HANDLE: u8 = 0x01;
pub const ATT_ERR_READ_NOT_PERMITTED: u8 = 0x02;
pub const ATT_ERR_WRITE_NOT_PERMITTED: u8 = 0x03;
pub const ATT_ERR_INVALID_OFFSET: u8 = 0x07;
pub const ATT_ERR_ATTR_NOT_FOUND: u8 = 0x0A;
pub const ATT_ERR_ATTR_NOT_LONG: u8 = 0x0B;
pub const ATT_ERR_INVALID_VALUE_SIZE: u8 = 0x0D;
pub const ATT_ERR_INSUFFICIENT_RESOURCES: u8 = 0x11;

/* GATT UUIDs */
pub const GATT_PRIMARY_SERVICE_UUID: u16 = 0x2800;
pub const GATT_CHARACTER_UUID: u16 = 0x2803;
pub const GATT_CHAR_USER_DESC_UUID: u16 = 0x2901;
pub const GATT_CLIENT_CHAR_CFG_UUID: u16 = 0x2902;

/// UUIDs as little-endian bytes with a fixed address, for attribute types
pub static PRIMARY_SERVICE_UUID: [u8; 2] = GATT_PRIMARY_SERVICE_UUID.to_le_bytes();
pub static CHARACTER_UUID: [u8; 2] = GATT_CHARACTER_UUID.to_le_bytes();
pub static CHAR_USER_DESC_UUID: [u8; 2] = GATT_CHAR_USER_DESC_UUID.to_le_bytes();
pub static CLIENT_CHAR_CFG_UUID: [u8; 2] = GATT_CLIENT_CHAR_CFG_UUID.to_le_bytes();

/* GATT Attribute Access Permissions */
pub const GATT_PERMIT_READ: u8 = 0x01;
pub const GATT_PERMIT_WRITE: u8 = 0x02;

/* GATT Characteristic Properties */
pub const GATT_PROP_READ: u8 = 0x02;
pub const GATT_PROP_WRITE_NO_RSP: u8 = 0x04;
pub const GATT_PROP_WRITE: u8 = 0x08;
pub const GATT_PROP_NOTIFY: u8 = 0x10;

/* GATT Client Characteristic Configuration bits */
pub const GATT_CLIENT_CFG_NOTIFY: u16 = 0x0001;
pub const GATT_CLIENT_CFG_INDICATE: u16 = 0x0002;

/// Maximum number of connections tracked by a characteristic configuration table
pub const GATT_MAX_NUM_CONN: usize = 4;

pub const GATT_MIN_ENCRYPT_KEY_SIZE: u8 = 7;
pub const GATT_MAX_ENCRYPT_KEY_SIZE: u8 = 16;

/* GAP GATT services (GGS_AddService / GATTServApp_AddService) */
pub const GATT_SERVICE_DEVICE_NAME: u32 = 1 << 0;
pub const GATT_SERVICE_APPEARANCE: u32 = 1 << 1;
pub const GATT_SERVICE_PERI_CONN_PARAM: u32 = 1 << 4;
pub const GATT_SERVICE_CENTRAL_ADDR_RL: u32 = 1 << 5;
/// What the library's header calls "all" (`GATT_SERVICES_DEFS` on CH584/CH585).
#[cfg(any(ch584, ch585))]
pub const GATT_ALL_SERVICES: u32 = GATT_SERVICE_DEVICE_NAME | GATT_SERVICE_APPEARANCE | GATT_SERVICE_PERI_CONN_PARAM;
#[cfg(ch592)]
pub const GATT_ALL_SERVICES: u32 = 0xFFFF_FFFF;

/* GGS parameters (GGS_SetParameter) */
/// uint8_t[GAP_DEVICE_NAME_LEN]
pub const GGS_DEVICE_NAME_ATT: u8 = 0;
/// Maximum device name length, excluding the null terminator
pub const GAP_DEVICE_NAME_LEN: usize = 21;

/* Attribute table */

/// Attribute type: a 2 or 16 byte UUID
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattAttrType_t {
    /// Length of UUID (2 or 16)
    pub len: u8,
    /// Pointer to UUID (little-endian)
    pub uuid: *const u8,
}

// Only ever points to 'static UUIDs
unsafe impl Sync for gattAttrType_t {}

impl gattAttrType_t {
    pub const fn uuid16(uuid: &'static [u8; 2]) -> Self {
        Self {
            len: ATT_BT_UUID_SIZE,
            uuid: uuid.as_ptr(),
        }
    }

    pub const fn uuid128(uuid: &'static [u8; 16]) -> Self {
        Self {
            len: ATT_UUID_SIZE,
            uuid: uuid.as_ptr(),
        }
    }
}

/// Attribute record. Tables of these are registered with
/// `GATTServApp_RegisterService` and must be `'static` (the library keeps a pointer
/// and fills in `handle`).
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattAttribute_t {
    /// Attribute type (UUID)
    pub type_: gattAttrType_t,
    /// GATT_PERMIT_*
    pub permissions: u8,
    /// Assigned by the attribute server
    pub handle: u16,
    /// Attribute value. For a primary service declaration this points to a
    /// `gattAttrType_t` holding the service UUID.
    pub pValue: *mut u8,
}

const _: () = assert!(core::mem::size_of::<gattAttribute_t>() == 16);

impl gattAttribute_t {
    pub const fn new(type_: gattAttrType_t, permissions: u8, pValue: *mut u8) -> Self {
        Self {
            type_,
            permissions,
            handle: 0,
            pValue,
        }
    }
}

/// Read callback: copy up to `maxLen` bytes into `pValue`, set `*pLen`.
/// Returns SUCCESS or an ATT_ERR_* code.
pub type pfnGATTReadAttrCB_t = Option<
    unsafe extern "C" fn(
        connHandle: u16,
        pAttr: *mut gattAttribute_t,
        pValue: *mut u8,
        pLen: *mut u16,
        offset: u16,
        maxLen: u16,
        method: u8,
    ) -> u8,
>;

/// Write callback. Returns SUCCESS or an ATT_ERR_* code.
pub type pfnGATTWriteAttrCB_t = Option<
    unsafe extern "C" fn(
        connHandle: u16,
        pAttr: *mut gattAttribute_t,
        pValue: *mut u8,
        len: u16,
        offset: u16,
        method: u8,
    ) -> u8,
>;

pub type pfnGATTAuthorizeAttrCB_t =
    Option<unsafe extern "C" fn(connHandle: u16, pAttr: *mut gattAttribute_t, opcode: u8) -> bStatus_t>;

/// Service callbacks, registered with `GATTServApp_RegisterService`
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattServiceCBs_t {
    pub pfnReadAttrCB: pfnGATTReadAttrCB_t,
    pub pfnWriteAttrCB: pfnGATTWriteAttrCB_t,
    pub pfnAuthorizeAttrCB: pfnGATTAuthorizeAttrCB_t,
}

/// Client Characteristic Configuration, one entry per connection
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gattCharCfg_t {
    pub connHandle: u16,
    pub value: u8,
}

impl gattCharCfg_t {
    pub const INIT: Self = Self {
        connHandle: INVALID_CONNHANDLE,
        value: 0,
    };
}

/// Handle Value Notification
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attHandleValueNoti_t {
    /// Handle of the attribute (must be first field)
    pub handle: u16,
    pub len: u16,
    /// Value, allocated with `GATT_bm_alloc`
    pub pValue: *mut u8,
}

/* GATT messages (GATT_MSG_EVENT) */

/// ATT_MTU_UPDATED_EVENT
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct attMtuUpdatedEvt_t {
    pub MTU: u16,
}

/// GATT message. Only the members used so far are included; `_align` keeps the
/// union's alignment (and so `gattMsgEvent_t::msg`'s offset) as in C.
#[repr(C)]
#[derive(Copy, Clone)]
pub union gattMsg_t {
    pub mtuEvt: attMtuUpdatedEvt_t,
    _align: *mut u8,
}

/// GATT message event. Only access it through a pointer from the library.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct gattMsgEvent_t {
    /// GATT_MSG_EVENT and status
    pub hdr: tmos_event_hdr_t,
    /// Connection message was received on
    pub connHandle: u16,
    /// Type of message (ATT_* opcode)
    pub method: u8,
    pub msg: gattMsg_t,
}

const _: () = assert!(core::mem::offset_of!(gattMsgEvent_t, msg) == 8);

/* Link database */

pub const LINKDB_STATUS_UPDATE_NEW: u8 = 0;
pub const LINKDB_STATUS_UPDATE_REMOVED: u8 = 1;
pub const LINKDB_STATUS_UPDATE_STATEFLAGS: u8 = 2;

pub const LINK_NOT_CONNECTED: u8 = 0x00;
pub const LINK_CONNECTED: u8 = 0x01;

pub type pfnLinkDBCB_t = Option<unsafe extern "C" fn(connectionHandle: u16, changeType: u8)>;

unsafe extern "C" {
    /* GATT Server App */

    /// Register a service's attribute table and callbacks
    pub fn GATTServApp_RegisterService(
        pAttrs: *mut gattAttribute_t,
        numAttrs: u16,
        encKeySize: u8,
        pServiceCBs: &'static gattServiceCBs_t,
    ) -> bStatus_t;
    /// Add the GATT service (GATT_ALL_SERVICES)
    pub fn GATTServApp_AddService(services: u32) -> bStatus_t;
    /// Initialise a characteristic configuration table for one connection, or all
    /// with INVALID_CONNHANDLE
    pub fn GATTServApp_InitCharCfg(connHandle: u16, charCfgTbl: *mut gattCharCfg_t);
    /// Read a client's characteristic configuration
    pub fn GATTServApp_ReadCharCfg(connHandle: u16, charCfgTbl: *mut gattCharCfg_t) -> u16;
    /// Handle a write to a Client Characteristic Configuration attribute
    pub fn GATTServApp_ProcessCCCWriteReq(
        connHandle: u16,
        pAttr: *mut gattAttribute_t,
        pValue: *mut u8,
        len: u16,
        offset: u16,
        validCfg: u16,
    ) -> bStatus_t;

    /* GATT */

    /// Send a notification. On failure the caller must free `pNoti.pValue` with `GATT_bm_free`.
    pub fn GATT_Notification(connHandle: u16, pNoti: *mut attHandleValueNoti_t, authenticated: u8) -> bStatus_t;
    /// Allocate a message payload (e.g. for a notification)
    pub fn GATT_bm_alloc(connHandle: u16, opcode: u8, size: u16, pSizeAlloc: *mut u16, flag: u8) -> *mut c_void;
    /// Free a message payload. `pMsg` points to the message (e.g. an `attHandleValueNoti_t`).
    pub fn GATT_bm_free(pMsg: *mut c_void, opcode: u8);

    /* GAP GATT Server */

    /// Add the GAP service (GATT_ALL_SERVICES)
    pub fn GGS_AddService(services: u32) -> bStatus_t;
    /// Set a GGS parameter (GGS_*_ATT)
    pub fn GGS_SetParameter(param: u8, len: u8, value: *const c_void) -> bStatus_t;

    /* Link DB */

    /// Register for connection status changes
    pub fn linkDB_Register(pFunc: pfnLinkDBCB_t) -> u8;
    /// Check whether a connection is in `state` (LINK_*)
    pub fn linkDB_State(connectionHandle: u16, state: u8) -> u8;
}
