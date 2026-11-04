//! Bindings to WCH's BLE library: library init and TMOS. GAP and GATT are
//! in [`super::gap`] and [`super::gatt`].
//!
//! Hand-written from `vendor/wch-ble/CH58xBLE_LIB.h` (CH584/CH585) and
//! `CH59xBLE_LIB.h` (CH592), covering what the HAL uses. Names and values
//! follow the C headers, so WCH's manual and EVT examples read alongside.
//! The two libraries share their API; where a struct or constant differs,
//! a cfg picks the die's version.

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, dead_code)]

use core::num::NonZeroU8;

/// `bStatus_t` is a `uint8_t` where 0 is `SUCCESS`. `Result<(), NonZeroU8>`
/// has the same ABI, so library calls can be used with `?`.
pub type bStatus_t = Result<(), NonZeroU8>;

/* Status codes (raw `uint8_t` values, e.g. for GATT callback returns) */
pub const SUCCESS: u8 = 0x00;
pub const FAILURE: u8 = 0x01;
pub const INVALIDPARAMETER: u8 = 0x02;
pub const bleIncorrectMode: u8 = 0x12;
pub const bleMemAllocError: u8 = 0x13;
pub const blePending: u8 = 0x16;

pub type tmosTaskID = u8;
pub type tmosEvents = u16;
pub type tmosTimer = u32;
pub type BOOL = u8;

/// Header version these bindings were written against (checked against
/// `VER_LIB` by [`super::init`]).
#[cfg(any(ch584, ch585))]
pub const VER_FILE: &str = "CH585_BLE_LIB_V1.4";
#[cfg(ch592)]
pub const VER_FILE: &str = "CH59x_BLE_LIB_V1.4";

/* Tx_POWER define (both libraries) */
pub const LL_TX_POWEER_MINUS_20_DBM: u8 = 0x01;
pub const LL_TX_POWEER_MINUS_15_DBM: u8 = 0x03;
pub const LL_TX_POWEER_MINUS_10_DBM: u8 = 0x05;
pub const LL_TX_POWEER_MINUS_8_DBM: u8 = 0x07;
pub const LL_TX_POWEER_MINUS_5_DBM: u8 = 0x0B;
pub const LL_TX_POWEER_MINUS_3_DBM: u8 = 0x0F;
pub const LL_TX_POWEER_MINUS_1_DBM: u8 = 0x13;
pub const LL_TX_POWEER_0_DBM: u8 = 0x15;
pub const LL_TX_POWEER_1_DBM: u8 = 0x1B;
pub const LL_TX_POWEER_2_DBM: u8 = 0x23;
pub const LL_TX_POWEER_3_DBM: u8 = 0x2B;
pub const LL_TX_POWEER_4_DBM: u8 = 0x3B;
#[cfg(ch592)]
pub const LL_TX_POWEER_9_DBM: u8 = 0xBF;

/* ERR_LIB_INIT define (BLE_LibInit return codes) */
pub const ERR_LLE_IRQ_HANDLE: u8 = 0x01;
pub const ERR_MEM_ALLOCATE_SIZE: u8 = 0x02;
pub const ERR_SET_MAC_ADDR: u8 = 0x03;
pub const ERR_GAP_ROLE_CONFIG: u8 = 0x04;
pub const ERR_CONNECT_NUMBER_CONFIG: u8 = 0x05;
pub const ERR_SNV_ADDR_CONFIG: u8 = 0x06;
pub const ERR_CLOCK_SELECT_CONFIG: u8 = 0x07;

/// BLE library config struct, passed to `BLE_LibInit`.
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct bleConfig_t {
    /// Library memory start address
    pub MEMAddr: u32,
    /// Library memory size
    pub MEMLen: u16,
    /// SNV flash start address (0: bonding information is not saved)
    pub SNVAddr: u32,
    /// SNV flash block size (default 256)
    pub SNVBlock: u16,
    /// SNV flash block number (default 1)
    pub SNVNum: u8,
    /// Packets the controller buffers for sending and receiving (default 5).
    /// Must be greater than the number of connections.
    pub BufNumber: u8,
    /// Maximum length of the data of an HCI data packet (default 27;
    /// ATT_MTU = BufMaxLen - 4)
    pub BufMaxLen: u16,
    /// Maximum TX packets in a connection event (default 1)
    pub TxNumEvent: u8,
    /// Maximum RX packets in a connection event (default `BufNumber`)
    pub RxNumEvent: u8,
    /// Transmit power, `LL_TX_POWEER_*`
    pub TxPower: u8,
    /// Connections: the low two bits peripheral, the rest central
    pub ConnectNumber: u8,
    #[cfg(any(ch584, ch585))]
    pub PeripheralNumber: u8,
    #[cfg(any(ch584, ch585))]
    pub CentralNumber: u8,
    /// Wait rf start window (us)
    pub WindowWidening: u8,
    /// Wait event arrive window in one system clock
    pub WaitWindow: u8,
    /// MAC address, little-endian
    pub MacAddr: [u8; 6],
    #[cfg(ch592)]
    pub PeripheralNumber: u8,
    #[cfg(ch592)]
    pub CentralNumber: u8,
    /// Generate a random seed
    pub srandCB: Option<unsafe extern "C" fn() -> u32>,
    /// Called when TMOS is idle, with the clock count of the next event.
    /// Returns 0 if the device slept.
    pub idleCB: Option<unsafe extern "C" fn(time: u32) -> u32>,
    /// Read the temperature (to recalibrate the RF when it changes)
    pub tsCB: Option<unsafe extern "C" fn() -> u16>,
    /// LSI calibration
    pub rcCB: Option<unsafe extern "C" fn()>,
    /// Library status callback
    pub staCB: Option<unsafe extern "C" fn(code: u8, status: u32)>,
    /// Read flash (SNV)
    pub readFlashCB: Option<unsafe extern "C" fn(addr: u32, num: u32, pBuf: *mut u32) -> u32>,
    /// Write flash (SNV)
    pub writeFlashCB: Option<unsafe extern "C" fn(addr: u32, num: u32, pBuf: *mut u32) -> u32>,
}

#[cfg(any(ch584, ch585))]
const _: () = assert!(core::mem::offset_of!(bleConfig_t, MacAddr) == 26);
#[cfg(ch592)]
const _: () = assert!(core::mem::offset_of!(bleConfig_t, MacAddr) == 24);
const _: () = assert!(core::mem::offset_of!(bleConfig_t, srandCB) == 32);
const _: () = assert!(core::mem::size_of::<bleConfig_t>() == 60);

/// TMOS timebase config, passed to `TMOS_TimerInit`.
#[cfg(any(ch584, ch585))]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct bleClockConfig_t {
    /// Returns the current clock count
    pub getClockValue: Option<unsafe extern "C" fn() -> u32>,
    /// Pends the timebase interrupt
    pub SetPendingIRQ: Option<unsafe extern "C" fn()>,
    /// The maximum count value
    pub ClockMaxCount: u32,
    /// The clock frequency (Hz)
    pub ClockFrequency: u16,
    /// The clock accuracy (ppm)
    pub ClockAccuracy: u16,
    /// "resv"
    pub irqEnable: u8,
    /// RF-8K (WCH's proprietary 2.4 GHz mode) only
    pub Clock1Frequency: u32,
    pub getClock1Value: Option<unsafe extern "C" fn() -> u32>,
    pub SetClock1PendingIRQ: Option<unsafe extern "C" fn()>,
    pub SetTign: Option<unsafe extern "C" fn(val: i32)>,
}

/// TMOS timebase config, passed to `TMOS_TimerInit`.
#[cfg(ch592)]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct bleClockConfig_t {
    /// Returns the current clock count
    pub getClockValue: Option<unsafe extern "C" fn() -> u32>,
    /// The maximum count value
    pub ClockMaxCount: u32,
    /// The clock frequency (Hz)
    pub ClockFrequency: u16,
    /// The clock accuracy (ppm)
    pub ClockAccuracy: u16,
    /// "resv"
    pub irqEnable: u8,
    /// Pends the timebase interrupt
    pub SetPendingIRQ: Option<unsafe extern "C" fn()>,
}

/* TMOS */

/// A message is waiting event
pub const SYS_EVENT_MSG: tmosEvents = 0x8000;
/// Task ID isn't setup properly
pub const INVALID_TASK_ID: tmosTaskID = 0xFF;

/// Unit of the TMOS event timer is 625 us
pub const SYSTEM_TIME_MICROSEN: u32 = 625;

/// Convert ms to TMOS timer units (625 us)
pub const fn MS1_TO_SYSTEM_TIME(ms: u32) -> tmosTimer {
    ms * 1000 / SYSTEM_TIME_MICROSEN
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct tmos_event_hdr_t {
    pub event: u8,
    pub status: u8,
}

/// Message event: incoming GATT message (`super::gatt::gattMsgEvent_t`)
pub const GATT_MSG_EVENT: u8 = 0xB0;
/// Message event: incoming GAP message (`super::gap::gapRoleEvent_t`)
pub const GAP_MSG_EVENT: u8 = 0xD0;

pub type pTaskEventHandlerFn = Option<unsafe extern "C" fn(taskID: tmosTaskID, events: tmosEvents) -> tmosEvents>;

unsafe extern "C" {
    /// Library version string, e.g. "CH585_BLE_LIB_V1.4"
    pub static VER_LIB: [u8; 0];

    /* BLE */

    /// Init the BLE library. Returns 0 on success, else `ERR_*`.
    pub fn BLE_LibInit(pCfg: *const bleConfig_t) -> bStatus_t;
    /// BLE register reset and RF calibration
    pub fn BLE_RegInit();
    /// BB interrupt handler
    pub fn BB_IRQLibHandler();
    /// LLE interrupt handler, reached through `g_LLE_IRQLibHandlerLocation`
    pub fn LLE_IRQLibHandler();

    /* TMOS */

    /// TMOS timer initialization. Must be called before using TMOS tasks.
    pub fn TMOS_TimerInit(pClockConfig: *const bleClockConfig_t) -> bStatus_t;
    /// Register a task's event handler. Returns 0xFF on error, else the task id.
    pub fn TMOS_ProcessEventRegister(eventCb: pTaskEventHandlerFn) -> tmosTaskID;
    /// Process system (one pass)
    pub fn TMOS_SystemProcess();
    /// Current system clock (in 625 us units)
    pub fn TMOS_GetSystemClock() -> u32;

    /// Set an event for a task (runs on the next `TMOS_SystemProcess`)
    pub fn tmos_set_event(taskID: tmosTaskID, event: tmosEvents) -> bStatus_t;
    /// Start an event after a time (625 us units). Returns TRUE/FALSE.
    pub fn tmos_start_task(taskID: tmosTaskID, event: tmosEvents, time: tmosTimer) -> BOOL;
    /// Stop a timed event
    pub fn tmos_stop_task(taskID: tmosTaskID, event: tmosEvents) -> bStatus_t;
    /// Receive a message, or NULL if none
    pub fn tmos_msg_receive(taskID: tmosTaskID) -> *mut u8;
    /// Free a received message
    pub fn tmos_msg_deallocate(msg_ptr: *mut u8) -> bStatus_t;
}
