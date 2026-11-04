//! GAP bindings: advertising/connection parameters, events, and the
//! Broadcaster and Peripheral roles.
//!
//! Hand-written from the library headers (see [`super::ffi`]); only the parts
//! used so far are bound.

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, dead_code)]

use core::ffi::c_void;

use super::ffi::{bStatus_t, tmos_event_hdr_t};

/// Bluetooth device address length
pub const B_ADDR_LEN: usize = 6;

/// Connection handle used when there is no connection
pub const GAP_CONNHANDLE_INIT: u16 = 0xFFFE;

/* GAP parameters (GAP_SetParamValue) */

/// Minimum advertising interval (n * 0.625 mSec)
pub const TGAP_DISC_ADV_INT_MIN: u16 = 3;
/// Maximum advertising interval (n * 0.625 mSec)
pub const TGAP_DISC_ADV_INT_MAX: u16 = 4;
/// Scan request notifications (GAP_SCAN_REQUEST_EVENT). Default 0 (disabled).
pub const TGAP_ADV_SCAN_REQ_NOTIFY: u16 = 30;

/* GAP_ADVERTISEMENT_TYPE_DEFINES */
pub const GAP_ADTYPE_ADV_IND: u8 = 0x00;
pub const GAP_ADTYPE_ADV_SCAN_IND: u8 = 0x02;
pub const GAP_ADTYPE_ADV_NONCONN_IND: u8 = 0x03;

/* GAP_ADTYPE_DEFINES */
pub const GAP_ADTYPE_FLAGS: u8 = 0x01;
pub const GAP_ADTYPE_16BIT_MORE: u8 = 0x02;
pub const GAP_ADTYPE_16BIT_COMPLETE: u8 = 0x03;
pub const GAP_ADTYPE_LOCAL_NAME_SHORT: u8 = 0x08;
pub const GAP_ADTYPE_LOCAL_NAME_COMPLETE: u8 = 0x09;
pub const GAP_ADTYPE_POWER_LEVEL: u8 = 0x0A;
pub const GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE: u8 = 0x12;
pub const GAP_ADTYPE_MANUFACTURER_SPECIFIC: u8 = 0xFF;

/* GAP_ADTYPE_FLAGS_MODES */
pub const GAP_ADTYPE_FLAGS_GENERAL: u8 = 0x02;
pub const GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED: u8 = 0x04;

/* GAPROLE_PROFILE_PARAMETERS (GAPRole_SetParameter) */
pub const GAPROLE_PROFILEROLE: u16 = 0x300;
/// uint8_t
pub const GAPROLE_ADVERT_ENABLED: u16 = 0x305;
pub const GAPROLE_ADVERT_DATA: u16 = 0x306;
pub const GAPROLE_SCAN_RSP_DATA: u16 = 0x307;
/// uint8_t
pub const GAPROLE_ADV_EVENT_TYPE: u16 = 0x308;
/// uint16_t, n * 1.25ms
pub const GAPROLE_MIN_CONN_INTERVAL: u16 = 0x311;
/// uint16_t, n * 1.25ms
pub const GAPROLE_MAX_CONN_INTERVAL: u16 = 0x312;

/* GAP_MSG_EVENT_DEFINES (gapEventHdr_t::opcode) */
pub const GAP_DEVICE_INIT_DONE_EVENT: u8 = 0x00;
pub const GAP_MAKE_DISCOVERABLE_DONE_EVENT: u8 = 0x03;
pub const GAP_END_DISCOVERABLE_DONE_EVENT: u8 = 0x04;
pub const GAP_LINK_ESTABLISHED_EVENT: u8 = 0x05;
pub const GAP_LINK_TERMINATED_EVENT: u8 = 0x06;
pub const GAP_LINK_PARAM_UPDATE_EVENT: u8 = 0x07;
/// A scan request was received (if TGAP_ADV_SCAN_REQ_NOTIFY is set)
pub const GAP_SCAN_REQUEST_EVENT: u8 = 0x19;

/// GAP role state. Bits [3:0] are the advertising state.
pub type gapRole_States_t = u32;

pub const GAPROLE_STATE_ADV_MASK: gapRole_States_t = 0xF;
pub const GAPROLE_INIT: gapRole_States_t = 0;
pub const GAPROLE_STARTED: gapRole_States_t = 1;
pub const GAPROLE_ADVERTISING: gapRole_States_t = 2;
pub const GAPROLE_WAITING: gapRole_States_t = 3;
pub const GAPROLE_CONNECTED: gapRole_States_t = 4;
pub const GAPROLE_CONNECTED_ADV: gapRole_States_t = 5;
pub const GAPROLE_ERROR: gapRole_States_t = 6;

/* GAP events */

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapEventHdr_t {
    /// GAP_MSG_EVENT and status
    pub hdr: tmos_event_hdr_t,
    /// GAP_*_EVENT
    pub opcode: u8,
}

/// GAP_LINK_ESTABLISHED_EVENT
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapEstLinkReqEvent_t {
    pub hdr: tmos_event_hdr_t,
    pub opcode: u8,
    pub devAddrType: u8,
    pub devAddr: [u8; B_ADDR_LEN],
    pub connectionHandle: u16,
    /// Connection formed as Central or Peripheral
    pub connRole: u8,
    /// n * 1.25ms
    pub connInterval: u16,
    pub connLatency: u16,
    /// n * 10ms
    pub connTimeout: u16,
    pub clockAccuracy: u8,
    #[cfg(any(ch584, ch585))]
    pub advHandle: u8,
    #[cfg(any(ch584, ch585))]
    pub syncHandle: u16,
}

const _: () = assert!(core::mem::offset_of!(gapEstLinkReqEvent_t, connInterval) == 14);

/// GAP_LINK_PARAM_UPDATE_EVENT
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapLinkUpdateEvent_t {
    pub hdr: tmos_event_hdr_t,
    pub opcode: u8,
    pub status: u8,
    pub connectionHandle: u16,
    pub connInterval: u16,
    pub connLatency: u16,
    pub connTimeout: u16,
}

/// GAP_LINK_TERMINATED_EVENT
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapTerminateLinkEvent_t {
    pub hdr: tmos_event_hdr_t,
    pub opcode: u8,
    pub connectionHandle: u16,
    /// Termination reason from LL
    pub reason: u8,
    pub connRole: u8,
}

const _: () = assert!(core::mem::offset_of!(gapTerminateLinkEvent_t, reason) == 6);

/// GAP_SCAN_REQUEST_EVENT
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapScanReqReseiveEvent_t {
    pub hdr: tmos_event_hdr_t,
    pub opcode: u8,
    pub advHandle: u8,
    pub scannerAddrType: u8,
    pub scannerAddr: [u8; B_ADDR_LEN],
}

/// GAP role event. Only the members used so far are included, so only access it
/// through a pointer from the library.
#[repr(C)]
#[derive(Copy, Clone)]
pub union gapRoleEvent_t {
    pub gap: gapEventHdr_t,
    pub linkCmpl: gapEstLinkReqEvent_t,
    pub linkUpdate: gapLinkUpdateEvent_t,
    pub linkTerminate: gapTerminateLinkEvent_t,
    pub scanReqEvt: gapScanReqReseiveEvent_t,
}

/* GAP Role callbacks */

pub type gapRolesBroadcasterStateNotify_t = Option<unsafe extern "C" fn(newState: gapRole_States_t)>;
/// Scan request received (argument is a `gapScanRec_t *`, not bound yet)
pub type gapRolesScanReqRecv_t = Option<unsafe extern "C" fn(pEvent: *mut c_void)>;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapRolesBroadcasterCBs_t {
    /// Whenever the device changes state
    pub pfnStateChange: gapRolesBroadcasterStateNotify_t,
    pub pfnScanRecv: gapRolesScanReqRecv_t,
}

pub type gapRolesStateNotify_t = Option<unsafe extern "C" fn(newState: gapRole_States_t, pEvent: *mut gapRoleEvent_t)>;
pub type gapRolesRssiRead_t = Option<unsafe extern "C" fn(connHandle: u16, newRSSI: i8)>;
pub type gapRolesParamUpdateCB_t =
    Option<unsafe extern "C" fn(connHandle: u16, connInterval: u16, connSlaveLatency: u16, connTimeout: u16)>;

/// Peripheral/Central role callbacks
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapRolesCBs_t {
    /// Whenever the device changes state
    pub pfnStateChange: gapRolesStateNotify_t,
    /// When a valid RSSI is read from the controller
    pub pfnRssiRead: gapRolesRssiRead_t,
    /// When the connection parameters are updated
    pub pfnParamUpdate: gapRolesParamUpdateCB_t,
}

/* GAP Bond Manager */

pub type pfnPasscodeCB_t =
    Option<unsafe extern "C" fn(deviceAddr: *mut u8, connectionHandle: u16, uiInputs: u8, uiOutputs: u8)>;
pub type pfnPairStateCB_t = Option<unsafe extern "C" fn(connectionHandle: u16, state: u8, status: u8)>;
pub type pfnOobCB_t =
    Option<unsafe extern "C" fn(deviceAddr: *mut u8, connectionHandle: u16, r_local: *mut u8, c_local: *mut u8)>;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct gapBondCBs_t {
    pub passcodeCB: pfnPasscodeCB_t,
    pub pairStateCB: pfnPairStateCB_t,
    pub oobCB: pfnOobCB_t,
}

/// uint8_t, GAPBOND_PAIRING_MODE_*. Default GAPBOND_PAIRING_MODE_WAIT_FOR_REQ.
pub const GAPBOND_PERI_PAIRING_MODE: u16 = 0x400;
/// uint8_t, default 0 (disabled)
pub const GAPBOND_PERI_BONDING_ENABLED: u16 = 0x405;

pub const GAPBOND_PAIRING_MODE_NO_PAIRING: u8 = 0x00;
pub const GAPBOND_PAIRING_MODE_WAIT_FOR_REQ: u8 = 0x01;
pub const GAPBOND_PAIRING_MODE_INITIATE: u8 = 0x02;

unsafe extern "C" {
    /// Set a GAP parameter (TGAP_*)
    pub fn GAP_SetParamValue(paramID: u16, paramValue: u16) -> bStatus_t;

    /// Set a GAP Role parameter (GAPROLE_*). The value is copied.
    pub fn GAPRole_SetParameter(param: u16, len: u16, pValue: *const c_void) -> bStatus_t;
    /// Get a GAP Role parameter (GAPROLE_*)
    pub fn GAPRole_GetParameter(param: u16, pValue: *mut c_void) -> bStatus_t;
    /// Terminate a connection
    pub fn GAPRole_TerminateLink(connHandle: u16) -> bStatus_t;

    /* Broadcaster role */

    /// Initialize the Broadcaster role
    pub fn GAPRole_BroadcasterInit() -> bStatus_t;
    /// Start the device in Broadcaster role
    pub fn GAPRole_BroadcasterStartDevice(pAppCallbacks: &'static gapRolesBroadcasterCBs_t) -> bStatus_t;
    /// Set the Broadcaster scan request callback
    pub fn GAPRole_BroadcasterSetCB(pAppCallbacks: &'static gapRolesBroadcasterCBs_t);

    /* Peripheral role */

    /// Initialize the Peripheral role
    pub fn GAPRole_PeripheralInit() -> bStatus_t;
    /// Start the device in Peripheral role. GAP messages go to TMOS task `taskid`.
    pub fn GAPRole_PeripheralStartDevice(
        taskid: u8,
        pCB: &'static gapBondCBs_t,
        pAppCallbacks: &'static gapRolesCBs_t,
    ) -> bStatus_t;
    /// Request a connection parameter update (intervals in 1.25ms units, timeout in 10ms units)
    pub fn GAPRole_PeripheralConnParamUpdateReq(
        connHandle: u16,
        connIntervalMin: u16,
        connIntervalMax: u16,
        latency: u16,
        connTimeout: u16,
        taskId: u8,
    ) -> bStatus_t;

    /* Bond manager */

    /// Set a GAP Bond Manager parameter (GAPBOND_*)
    pub fn GAPBondMgr_SetParameter(param: u16, len: u8, pValue: *const c_void) -> bStatus_t;
}
