//! Peripheral role helper: GAP setup, advertising and connection events.
//!
//! ```ignore
//! ble::init(p.RTC, ble::Config::default())?;
//!
//! let adv = AdvData::new()
//!     .flags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
//!     .uuid16_list(&[0xFFE0])
//!     .short_name("ch584-per");
//! let scan_rsp = AdvData::new().name("ch584-peripheral");
//! peripheral::start(&peripheral::Config::new("ch584-peripheral", &adv, &scan_rsp), &[&SERVICE])?;
//!
//! // On embassy::Executor TMOS runs by itself; on another executor spawn a
//! // task running ble::run().
//! loop {
//!     match peripheral::next_event().await {
//!         Event::Connected { handle, .. } => {
//!             // Later, to save power: 100-200 ms, latency 4, 6 s timeout.
//!             peripheral::request_conn_params(handle, 80, 160, 4, 600)?;
//!         }
//!         Event::Disconnected { .. } => { /* advertising restarts automatically */ }
//!         _ => {}
//!     }
//! }
//! ```
//!
//! [`start`] does what the EVT `Peripheral_Init` does: initialise the
//! Peripheral role, set advertising and scan response data, advertising and
//! preferred connection intervals, add the GAP/GATT services and the
//! application's services ([`super::gatt_server::register`]), set the device
//! name, and start the device with role callbacks and a TMOS task for GAP/GATT
//! messages. Those callbacks run inside TMOS; they post [`Event`]s to a channel
//! read with [`next_event`], and re-enable advertising after a disconnect.
//!
//! GATT reads, writes and subscriptions are handled by the characteristic
//! callbacks (see [`super::gatt_server`]), not as events.

use core::ffi::c_void;
use core::num::NonZeroU8;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

pub use super::adv::AdvData;
use super::ffi::*;
use super::gap::*;
use super::gatt::*;
use super::gatt_server::{self, Service};
use super::MacAddress;

pub struct Config<'a> {
    /// GAP device name characteristic (at most `GAP_DEVICE_NAME_LEN` bytes)
    pub device_name: &'a str,
    pub adv_data: &'a AdvData,
    pub scan_rsp_data: &'a AdvData,
    /// Advertising interval, 0.625ms units. The current while advertising
    /// goes roughly with the rate: on a CH584F, 50 ms costs about 0.46 mA,
    /// 22 uC an event.
    pub adv_interval: u16,
    /// Preferred connection interval range, 1.25ms units
    pub min_conn_interval: u16,
    pub max_conn_interval: u16,
    /// Report received scan requests as [`Event::ScanRequest`] (for diagnostics)
    pub scan_request_events: bool,
}

impl<'a> Config<'a> {
    /// Defaults (from the EVT): 50ms advertising, 7.5ms-125ms connection interval
    pub fn new(device_name: &'a str, adv_data: &'a AdvData, scan_rsp_data: &'a AdvData) -> Self {
        Self {
            device_name,
            adv_data,
            scan_rsp_data,
            adv_interval: 80,
            min_conn_interval: 6,
            max_conn_interval: 100,
            scan_request_events: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Device name longer than `GAP_DEVICE_NAME_LEN`
    NameTooLong,
    /// `TMOS_ProcessEventRegister` failed
    TaskRegister,
    Gatt(gatt_server::Error),
    /// Library error (`bStatus_t`)
    Ble(NonZeroU8),
}

impl From<NonZeroU8> for Error {
    fn from(e: NonZeroU8) -> Self {
        Error::Ble(e)
    }
}

impl From<gatt_server::Error> for Error {
    fn from(e: gatt_server::Error) -> Self {
        Error::Gatt(e)
    }
}

/// Connection events, read with [`next_event`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Interval in 1.25ms units
    Connected {
        handle: u16,
        interval: u16,
        peer: MacAddress,
    },
    /// Advertising has been restarted
    Disconnected {
        handle: u16,
        reason: u8,
    },
    /// Interval in 1.25ms units, timeout in 10ms units
    ParamUpdate {
        handle: u16,
        interval: u16,
        latency: u16,
        timeout: u16,
    },
    Mtu {
        handle: u16,
        mtu: u16,
    },
    /// A scanner requested the scan response (if `Config::scan_request_events`)
    ScanRequest {
        peer: MacAddress,
    },
}

static EVENTS: Channel<CriticalSectionRawMutex, Event, 8> = Channel::new();

/// Wait for the next connection event
pub async fn next_event() -> Event {
    EVENTS.receive().await
}

fn send_event(event: Event) {
    // Dropped if the application is not reading events
    let _ = EVENTS.try_send(event);
}

/// Set up the Peripheral role with `services` and start advertising. Call after
/// [`super::init`] (and before spawning the task running [`super::run`], if
/// one does).
pub fn start(config: &Config, services: &[&'static Service]) -> Result<(), Error> {
    if config.device_name.len() > GAP_DEVICE_NAME_LEN {
        return Err(Error::NameTooLong);
    }
    let adv = config.adv_data.as_slice();
    let scan_rsp = config.scan_rsp_data.as_slice();
    // GAPRole_SetParameter copies the values
    let enable: u8 = 1;
    let (min_interval, max_interval) = (config.min_conn_interval, config.max_conn_interval);

    unsafe {
        GAPRole_PeripheralInit()?;

        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &raw const enable as *const c_void)?;
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, adv.len() as u16, adv.as_ptr() as *const c_void)?;
        GAPRole_SetParameter(
            GAPROLE_SCAN_RSP_DATA,
            scan_rsp.len() as u16,
            scan_rsp.as_ptr() as *const c_void,
        )?;
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, 2, &raw const min_interval as *const c_void)?;
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, 2, &raw const max_interval as *const c_void)?;

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, config.adv_interval)?;
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, config.adv_interval)?;
        GAP_SetParamValue(TGAP_ADV_SCAN_REQ_NOTIFY, config.scan_request_events as u16)?;

        GGS_AddService(GATT_ALL_SERVICES)?; // GAP service
        GATTServApp_AddService(GATT_ALL_SERVICES)?; // GATT service
        gatt_server::register(services)?;
        let name = config.device_name.as_bytes();
        GGS_SetParameter(GGS_DEVICE_NAME_ATT, name.len() as u8, name.as_ptr() as *const c_void)?;

        // As in the EVT Peripheral_Init (scan request callback unused)
        GAPRole_BroadcasterSetCB(&BROADCASTER_CBS);

        let task_id = TMOS_ProcessEventRegister(Some(tmos_task));
        if task_id == INVALID_TASK_ID {
            return Err(Error::TaskRegister);
        }
        TASK_ID.store(task_id, Ordering::Relaxed);
        GAPRole_PeripheralStartDevice(task_id, &BOND_CBS, &ROLE_CBS)?;
    }
    Ok(())
}

/// The role's TMOS task, for [`request_conn_params`].
static TASK_ID: AtomicU8 = AtomicU8::new(INVALID_TASK_ID);

/// Ask the central of connection `handle` for new connection parameters
/// (`GAPRole_PeripheralConnParamUpdateReq`): an interval between `min` and
/// `max` (1.25 ms units), `latency` (connection events the peripheral may
/// skip when it has nothing to send) and the supervision `timeout` (10 ms
/// units). The central decides; [`Event::ParamUpdate`] reports what it
/// chose. A longer interval and some latency save power on a link that
/// carries little data.
///
/// Apple's accessory guidelines accept: `min` at least 15 ms, `max` at
/// least `min` + 15 ms, `max * (latency + 1)` at most 2 s, `latency` at
/// most 30, `timeout` between 2 s and 6 s and more than three times
/// `max * (latency + 1)`. WCH's EVT asks 6.4 s after connecting, once the
/// central has had time to discover the services at its own interval.
///
/// Call from thread level (a task), after [`start`].
pub fn request_conn_params(handle: u16, min: u16, max: u16, latency: u16, timeout: u16) -> Result<(), Error> {
    let task_id = TASK_ID.load(Ordering::Relaxed);
    unsafe { GAPRole_PeripheralConnParamUpdateReq(handle, min, max, latency, timeout, task_id)? };
    Ok(())
}

static BROADCASTER_CBS: gapRolesBroadcasterCBs_t = gapRolesBroadcasterCBs_t {
    pfnStateChange: None,
    pfnScanRecv: None,
};

static BOND_CBS: gapBondCBs_t = gapBondCBs_t {
    passcodeCB: None,
    pairStateCB: None,
    oobCB: None,
};

static ROLE_CBS: gapRolesCBs_t = gapRolesCBs_t {
    pfnStateChange: Some(state_change),
    pfnRssiRead: None,
    pfnParamUpdate: Some(param_update),
};

unsafe extern "C" fn state_change(new_state: gapRole_States_t, event: *mut gapRoleEvent_t) {
    let event = unsafe { &*event };
    let opcode = unsafe { event.gap.opcode };
    match (new_state & GAPROLE_STATE_ADV_MASK, opcode) {
        (GAPROLE_CONNECTED, GAP_LINK_ESTABLISHED_EVENT) => {
            let link = unsafe { &event.linkCmpl };
            send_event(Event::Connected {
                handle: link.connectionHandle,
                interval: link.connInterval,
                peer: MacAddress(link.devAddr),
            });
        }
        (GAPROLE_ADVERTISING | GAPROLE_WAITING, GAP_LINK_TERMINATED_EVENT) => {
            let link = unsafe { &event.linkTerminate };
            send_event(Event::Disconnected {
                handle: link.connectionHandle,
                reason: link.reason,
            });
            // Advertise again
            let enable: u8 = 1;
            let _ = unsafe { GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &raw const enable as *const c_void) };
        }
        _ => {}
    }
}

unsafe extern "C" fn param_update(handle: u16, interval: u16, latency: u16, timeout: u16) {
    send_event(Event::ParamUpdate {
        handle,
        interval,
        latency,
        timeout,
    });
}

/// TMOS task receiving GAP/GATT messages for the application
unsafe extern "C" fn tmos_task(task_id: tmosTaskID, events: tmosEvents) -> tmosEvents {
    if events & SYS_EVENT_MSG != 0 {
        unsafe {
            let msg = tmos_msg_receive(task_id);
            if !msg.is_null() {
                let hdr = &*(msg as *const tmos_event_hdr_t);
                if hdr.event == GAP_MSG_EVENT {
                    let gap = &*(msg as *const gapRoleEvent_t);
                    if gap.gap.opcode == GAP_SCAN_REQUEST_EVENT {
                        send_event(Event::ScanRequest {
                            peer: MacAddress(gap.scanReqEvt.scannerAddr),
                        });
                    }
                } else if hdr.event == GATT_MSG_EVENT {
                    let gatt = &*(msg as *const gattMsgEvent_t);
                    if gatt.method == ATT_MTU_UPDATED_EVENT {
                        send_event(Event::Mtu {
                            handle: gatt.connHandle,
                            mtu: gatt.msg.mtuEvt.MTU,
                        });
                    }
                }
                let _ = tmos_msg_deallocate(msg);
            }
        }
        return events ^ SYS_EVENT_MSG;
    }
    0
}
