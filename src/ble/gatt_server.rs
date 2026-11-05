//! GATT server helper: declare services and characteristics as Rust statics
//! with read/write/subscribe callbacks, and let this module build the
//! library's attribute table and route its callbacks.
//!
//! ```ignore
//! static DATA: Characteristic = Characteristic::new(Uuid::u16(0xFFE1), GATT_PROP_READ | GATT_PROP_WRITE)
//!     .description("Data")
//!     .on_read(&|_conn, buf| { buf[..2].copy_from_slice(&[1, 2]); Ok(2) })
//!     .on_write(&|_conn, data| { /* store data */ Ok(()) });
//!
//! static COUNTER: Characteristic = Characteristic::new(Uuid::u16(0xFFE2), GATT_PROP_READ | GATT_PROP_NOTIFY)
//!     .on_read(&read_counter)                       // a plain fn works too
//!     .on_subscribe(&|conn, enabled| { /* ... */ });
//!
//! static SERVICE: Service = Service::new(Uuid::u16(0xFFE0), &[&DATA, &COUNTER]);
//!
//! gatt_server::register(&[&SERVICE])?;            // or via ble::peripheral::start
//!
//! // Notifications: from a task per data source, independent of connections
//! loop {
//!     gatt_server::wait_subscribed(&COUNTER).await?;  // optional: pause while nobody listens
//!     Timer::after_secs(1).await;
//!     if !gatt_server::is_subscribed_any(&COUNTER) {
//!         continue;                                   // unsubscribed during the sleep
//!     }
//!     gatt_server::notify_all(&COUNTER, &value)?;     // every subscribed connection
//! }
//! // or to one connection: gatt_server::notify(conn_handle, &COUNTER, &value)?;
//! ```
//!
//! Callbacks run inside TMOS (in the task running [`super::run`]), so keep
//! them short: update state, post to a channel. Writes are limited to a single
//! ATT packet (no long/queued writes), and reads to `offset == 0` (no read
//! blob); values up to the MTU - 1 bytes (22 with the default MTU) can be read.
//!
//! # How the library's GATT server works (the manual process)
//!
//! This module automates what the `ble_peripheral_raw` example does by hand.
//!
//! ## 1. The GATT model
//!
//! A GATT server is a flat list of *attributes*, each with a 16-bit **handle**
//! (its address), a **type** (a UUID saying what kind of attribute it is),
//! **permissions** (read/write) and a **value**. Services and characteristics
//! are not separate objects, just attributes with special type UUIDs, in a
//! fixed order:
//!
//! - `0x2800` *Primary Service* declaration: value is the service UUID
//! - `0x2803` *Characteristic* declaration: value is the properties byte (read,
//!   write, notify...) plus the handle and UUID of the value attribute after it
//! - the characteristic **value** attribute, whose type is the characteristic UUID
//! - optional descriptors after it: `0x2902` CCC (Client Characteristic
//!   Configuration, the client's notify on/off switch), `0x2901` user description
//!
//! ## 2. One row: `gattAttribute_t`
//!
//! ```ignore
//! pub struct gattAttribute_t {
//!     pub type_: gattAttrType_t, // { len: 2 or 16, uuid: *const u8 }, UUID bytes little-endian
//!     pub permissions: u8,       // GATT_PERMIT_READ | GATT_PERMIT_WRITE ...
//!     pub handle: u16,           // 0; filled in by the library at registration
//!     pub pValue: *mut u8,       // meaning depends on the attribute type
//! }
//! ```
//!
//! ## 3. A table for a service with a read/write and a read/notify characteristic
//!
//! | Row | Type UUID | Permissions | `pValue` points to | Served by |
//! |-----|-----------|-------------|--------------------|-----------|
//! | 0 | `0x2800` Primary Service | R | a `gattAttrType_t` holding the service UUID | library |
//! | 1 | `0x2803` Characteristic | R | properties byte (READ\|WRITE) | library |
//! | 2 | characteristic UUID | R/W | value storage | read/write callbacks |
//! | 3 | `0x2901` description | R | description bytes | read callback |
//! | 4 | `0x2803` Characteristic | R | properties byte (READ\|NOTIFY) | library |
//! | 5 | characteristic UUID | R | value storage | read callback |
//! | 6 | `0x2902` CCC | R/W | `gattCharCfg_t[GATT_MAX_NUM_CONN]` (per connection) | callbacks + `GATTServApp_*CharCfg` |
//! | 7 | `0x2901` description | R | description bytes | read callback |
//!
//! - For the service declaration `pValue` points to a `gattAttrType_t`, not raw
//!   bytes (WCH convention, from the EVT `gattprofile.c`).
//! - For a characteristic declaration `pValue` points to just the properties
//!   byte; the library builds the declaration value from it and the *next* row,
//!   so the declaration must come immediately before its value.
//! - The library serves declarations itself; everything else goes to the
//!   service's read/write callbacks. No `pValue` should be null.
//!
//! ## 4. Statics
//!
//! The library keeps pointers into the table and writes each `handle`, so the
//! table and everything it points to must be `'static` (a `static mut`, since
//! it is written and holds raw pointers). Take addresses with `&raw const` /
//! `&raw mut` (no references to `static mut` in edition 2024).
//!
//! ## 5. Registration
//!
//! ```ignore
//! GATTServApp_InitCharCfg(INVALID_CONNHANDLE, ccc_table);  // clear CCC for all connections
//! linkDB_Register(Some(link_status));                       // clear CCC on disconnect
//! GATTServApp_RegisterService(table, num_rows, GATT_MAX_ENCRYPT_KEY_SIZE, &SERVICE_CBS);
//! ```
//!
//! after `GGS_AddService(GATT_ALL_SERVICES)` and `GATTServApp_AddService(...)`
//! (the GAP and GATT services). The library assigns consecutive handles.
//! `SERVICE_CBS` is a `gattServiceCBs_t` with `pfnReadAttrCB`/`pfnWriteAttrCB`.
//!
//! ## 6. Callbacks
//!
//! The library passes a pointer to the row (`*mut gattAttribute_t`); dispatch on
//! its index in the table (UUIDs are ambiguous: two rows may both be `0x2901`).
//!
//! - read `(conn, attr, value, len, offset, max_len, method) -> u8`: copy up to
//!   `max_len` bytes to `value`, set `*len`, return `SUCCESS` or an `ATT_ERR_*`
//!   code. `offset > 0` is a read blob.
//! - write `(conn, attr, value, len, offset, method) -> u8`. For the CCC row,
//!   `GATTServApp_ProcessCCCWriteReq(.., GATT_CLIENT_CFG_NOTIFY)` validates and
//!   stores the client's setting; `GATTServApp_ReadCharCfg` reads it back.
//!
//! ## 7. Notifications
//!
//! Use the handle the library assigned to the value row: allocate the payload
//! with `GATT_bm_alloc(conn, ATT_HANDLE_VALUE_NOTI, len, ..)`, send with
//! `GATT_Notification`, and free it with `GATT_bm_free` only if sending fails.
//! Only send while the client's CCC has notifications enabled.

use core::num::NonZeroU8;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use super::ffi::*;
use super::gatt::*;

/// Maximum attribute rows (each characteristic uses 2-4)
pub const MAX_ATTRIBUTES: usize = 48;
pub const MAX_SERVICES: usize = 4;
/// Maximum characteristics with notify (each needs a CCC table)
pub const MAX_NOTIFY: usize = 8;

/// Read callback: `(conn_handle, buf) -> Ok(len)`. Fill `buf` (sized to what
/// fits in the response) and return the length, or `Err(ATT_ERR_*)`.
pub type ReadFn = dyn Fn(u16, &mut [u8]) -> Result<usize, u8> + Sync;
/// Write callback: `(conn_handle, data)`. Return `Err(ATT_ERR_*)` to reject.
pub type WriteFn = dyn Fn(u16, &[u8]) -> Result<(), u8> + Sync;
/// Called when a client enables or disables notifications: `(conn_handle, enabled)`
pub type SubscribeFn = dyn Fn(u16, bool) + Sync;

/// 16 or 128-bit UUID, stored little-endian as the library expects
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Uuid {
    U16([u8; 2]),
    U128([u8; 16]),
}

impl Uuid {
    pub const fn u16(uuid: u16) -> Self {
        Uuid::U16(uuid.to_le_bytes())
    }

    pub const fn u128(uuid: u128) -> Self {
        Uuid::U128(uuid.to_le_bytes())
    }

    fn attr_type(&'static self) -> gattAttrType_t {
        match self {
            Uuid::U16(bytes) => gattAttrType_t::uuid16(bytes),
            Uuid::U128(bytes) => gattAttrType_t::uuid128(bytes),
        }
    }
}

/// A characteristic: UUID, properties (`GATT_PROP_*`), optional user
/// description, and callbacks. Declare as a `static`.
pub struct Characteristic {
    pub uuid: Uuid,
    /// `GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP | GATT_PROP_NOTIFY`
    pub properties: u8,
    /// User description descriptor (0x2901)
    pub description: Option<&'static str>,
    pub on_read: Option<&'static ReadFn>,
    pub on_write: Option<&'static WriteFn>,
    pub on_subscribe: Option<&'static SubscribeFn>,
}

impl Characteristic {
    pub const fn new(uuid: Uuid, properties: u8) -> Self {
        Self {
            uuid,
            properties,
            description: None,
            on_read: None,
            on_write: None,
            on_subscribe: None,
        }
    }

    pub const fn description(mut self, description: &'static str) -> Self {
        self.description = Some(description);
        self
    }

    pub const fn on_read(mut self, f: &'static ReadFn) -> Self {
        self.on_read = Some(f);
        self
    }

    pub const fn on_write(mut self, f: &'static WriteFn) -> Self {
        self.on_write = Some(f);
        self
    }

    pub const fn on_subscribe(mut self, f: &'static SubscribeFn) -> Self {
        self.on_subscribe = Some(f);
        self
    }
}

/// A primary service. Declare as a `static`.
pub struct Service {
    pub uuid: Uuid,
    pub characteristics: &'static [&'static Characteristic],
}

impl Service {
    pub const fn new(uuid: Uuid, characteristics: &'static [&'static Characteristic]) -> Self {
        Self { uuid, characteristics }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// `register` was called more than once
    AlreadyRegistered,
    /// More than [`MAX_ATTRIBUTES`] rows, [`MAX_SERVICES`] services or
    /// [`MAX_NOTIFY`] notifying characteristics
    TooMany,
    /// The characteristic is not registered or does not support notify
    NotFound,
    /// The client has not enabled notifications
    NotSubscribed,
    /// No buffer available for the notification
    NoMemory,
    /// Library error (`bStatus_t`)
    Ble(NonZeroU8),
}

/// What a table row is, for dispatching callbacks
#[derive(Clone, Copy)]
enum Role {
    /// Served by the library (declarations)
    Fixed,
    Value(&'static Characteristic),
    /// CCC with its index in `Tables::ccc`
    Ccc(&'static Characteristic, usize),
    Description(&'static Characteristic),
}

/// Storage handed to the library; written only by `register` (and the library)
struct Tables {
    rows: [gattAttribute_t; MAX_ATTRIBUTES],
    roles: [Role; MAX_ATTRIBUTES],
    num_rows: usize,
    service_types: [gattAttrType_t; MAX_SERVICES],
    ccc: [[gattCharCfg_t; GATT_MAX_NUM_CONN]; MAX_NOTIFY],
    num_ccc: usize,
}

const EMPTY_ROW: gattAttribute_t = gattAttribute_t::new(
    gattAttrType_t {
        len: 0,
        uuid: ptr::null(),
    },
    0,
    ptr::null_mut(),
);

static mut TABLES: Tables = Tables {
    rows: [EMPTY_ROW; MAX_ATTRIBUTES],
    roles: [Role::Fixed; MAX_ATTRIBUTES],
    num_rows: 0,
    service_types: [gattAttrType_t {
        len: 0,
        uuid: ptr::null(),
    }; MAX_SERVICES],
    ccc: [[gattCharCfg_t::INIT; GATT_MAX_NUM_CONN]; MAX_NOTIFY],
    num_ccc: 0,
};

static REGISTERED: AtomicBool = AtomicBool::new(false);

/// Non-null `pValue` for value rows (the callbacks provide the actual value)
static VALUE_PLACEHOLDER: [u8; 4] = [0; 4];

static SERVICE_CBS: gattServiceCBs_t = gattServiceCBs_t {
    pfnReadAttrCB: Some(read_attr),
    pfnWriteAttrCB: Some(write_attr),
    pfnAuthorizeAttrCB: None,
};

/// Build the attribute tables for `services` and register them with the
/// library. Call once, after `GGS_AddService`/`GATTServApp_AddService`
/// ([`super::peripheral::start`] does this).
pub fn register(services: &[&'static Service]) -> Result<(), Error> {
    if REGISTERED.load(Ordering::Relaxed) {
        return Err(Error::AlreadyRegistered);
    }
    REGISTERED.store(true, Ordering::Relaxed);

    // Only called at init from thread level, before the library uses the tables
    let tables_ptr = &raw mut TABLES;
    let tables = unsafe { &mut *tables_ptr };
    if services.len() > MAX_SERVICES {
        return Err(Error::TooMany);
    }

    for (service_index, service) in services.iter().enumerate() {
        let first_row = tables.num_rows;

        tables.service_types[service_index] = service.uuid.attr_type();
        let service_type = &raw const tables.service_types[service_index] as *mut u8;
        tables.push(&PRIMARY_SERVICE_UUID, GATT_PERMIT_READ, service_type, Role::Fixed)?;

        for &ch in service.characteristics {
            // Declaration: pValue is the properties byte; the value row must follow
            let properties = &raw const ch.properties as *mut u8;
            tables.push(&CHARACTER_UUID, GATT_PERMIT_READ, properties, Role::Fixed)?;

            let mut permissions = 0;
            if ch.properties & GATT_PROP_READ != 0 {
                permissions |= GATT_PERMIT_READ;
            }
            if ch.properties & (GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP) != 0 {
                permissions |= GATT_PERMIT_WRITE;
            }
            tables.push_type(
                ch.uuid.attr_type(),
                permissions,
                &raw const VALUE_PLACEHOLDER as *mut u8,
                Role::Value(ch),
            )?;

            if ch.properties & GATT_PROP_NOTIFY != 0 {
                if tables.num_ccc >= MAX_NOTIFY {
                    return Err(Error::TooMany);
                }
                let index = tables.num_ccc;
                tables.num_ccc += 1;
                let ccc = &raw mut tables.ccc[index] as *mut gattCharCfg_t;
                unsafe { GATTServApp_InitCharCfg(INVALID_CONNHANDLE, ccc) };
                tables.push(
                    &CLIENT_CHAR_CFG_UUID,
                    GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                    ccc as *mut u8,
                    Role::Ccc(ch, index),
                )?;
            }

            if let Some(description) = ch.description {
                tables.push(
                    &CHAR_USER_DESC_UUID,
                    GATT_PERMIT_READ,
                    description.as_ptr() as *mut u8,
                    Role::Description(ch),
                )?;
            }
        }

        let rows = &raw mut tables.rows[first_row];
        let num_rows = (tables.num_rows - first_row) as u16;
        unsafe { GATTServApp_RegisterService(rows, num_rows, GATT_MAX_ENCRYPT_KEY_SIZE, &SERVICE_CBS) }
            .map_err(Error::Ble)?;
    }

    unsafe { linkDB_Register(Some(link_status)) };
    Ok(())
}

impl Tables {
    fn push(&mut self, uuid: &'static [u8; 2], permissions: u8, value: *mut u8, role: Role) -> Result<(), Error> {
        self.push_type(gattAttrType_t::uuid16(uuid), permissions, value, role)
    }

    fn push_type(&mut self, type_: gattAttrType_t, permissions: u8, value: *mut u8, role: Role) -> Result<(), Error> {
        if self.num_rows >= MAX_ATTRIBUTES {
            return Err(Error::TooMany);
        }
        self.rows[self.num_rows] = gattAttribute_t::new(type_, permissions, value);
        self.roles[self.num_rows] = role;
        self.num_rows += 1;
        Ok(())
    }

    /// Role of the row `attr` points to
    fn role(&self, attr: *const gattAttribute_t) -> Option<Role> {
        let base = self.rows.as_ptr() as usize;
        let index = (attr as usize).wrapping_sub(base) / size_of::<gattAttribute_t>();
        (index < self.num_rows).then(|| self.roles[index])
    }

    /// Value row index and CCC table index of a notifying characteristic
    fn find_notify(&self, ch: &'static Characteristic) -> Option<(usize, usize)> {
        let value = (0..self.num_rows).find(|&i| matches!(self.roles[i], Role::Value(c) if ptr::eq(c, ch)))?;
        let ccc = self.roles[..self.num_rows].iter().find_map(|role| match role {
            Role::Ccc(c, index) if ptr::eq(*c, ch) => Some(*index),
            _ => None,
        })?;
        Some((value, ccc))
    }
}

fn tables() -> &'static Tables {
    let tables = &raw const TABLES;
    unsafe { &*tables }
}

fn ccc_table(index: usize) -> *mut gattCharCfg_t {
    unsafe { &raw mut TABLES.ccc[index] as *mut gattCharCfg_t }
}

/// Whether `conn_handle` has enabled notifications for `ch`
pub fn is_subscribed(conn_handle: u16, ch: &'static Characteristic) -> bool {
    match tables().find_notify(ch) {
        Some((_, ccc)) => unsafe { GATTServApp_ReadCharCfg(conn_handle, ccc_table(ccc)) & GATT_CLIENT_CFG_NOTIFY != 0 },
        None => false,
    }
}

/// Send `data` as a notification of `ch` to `conn_handle`. Fails with
/// [`Error::NotSubscribed`] unless the client enabled notifications. `data`
/// must fit in one packet (MTU - 3 bytes: 20 with the default MTU).
pub fn notify(conn_handle: u16, ch: &'static Characteristic, data: &[u8]) -> Result<(), Error> {
    let (value_row, ccc) = tables().find_notify(ch).ok_or(Error::NotFound)?;
    if unsafe { GATTServApp_ReadCharCfg(conn_handle, ccc_table(ccc)) } & GATT_CLIENT_CFG_NOTIFY == 0 {
        return Err(Error::NotSubscribed);
    }
    send_notification(conn_handle, tables().rows[value_row].handle, data)
}

/// Send `data` as a notification of `ch` to every connection that has
/// notifications enabled, and return how many were sent (0 if nobody is
/// subscribed). All subscribers are tried; if any send fails, the first error
/// is returned. `data` must fit in one packet, as for [`notify`].
pub fn notify_all(ch: &'static Characteristic, data: &[u8]) -> Result<usize, Error> {
    let (value_row, ccc) = tables().find_notify(ch).ok_or(Error::NotFound)?;
    let handle = tables().rows[value_row].handle;
    let mut sent = 0;
    let mut first_error = None;
    for conn_handle in subscribers(ccc) {
        match send_notification(conn_handle, handle, data) {
            Ok(()) => sent += 1,
            Err(e) => {
                first_error.get_or_insert(e);
            }
        }
    }
    match first_error {
        Some(e) => Err(e),
        None => Ok(sent),
    }
}

/// Whether any connection has notifications enabled for `ch`
pub fn is_subscribed_any(ch: &'static Characteristic) -> bool {
    match tables().find_notify(ch) {
        Some((_, ccc)) => subscribers(ccc).next().is_some(),
        None => false,
    }
}

/// Wait until at least one connection has notifications enabled for `ch`
/// (returns immediately if one has). E.g. to only produce data while someone
/// is listening. Fails with [`Error::NotFound`] if `ch` is not a registered
/// notifying characteristic.
pub async fn wait_subscribed(ch: &'static Characteristic) -> Result<(), Error> {
    let (_, ccc) = tables().find_notify(ch).ok_or(Error::NotFound)?;
    loop {
        if subscribers(ccc).next().is_some() {
            return Ok(());
        }
        SUBSCRIPTION_CHANGED[ccc].wait().await;
    }
}

/// Signalled when a client configuration changes (CCC write or disconnect),
/// one per CCC table
static SUBSCRIPTION_CHANGED: [Signal<CriticalSectionRawMutex, ()>; MAX_NOTIFY] = [const { Signal::new() }; MAX_NOTIFY];

/// Connections with notifications enabled in CCC table `ccc`. The table has
/// one entry per connection (unused entries are `INVALID_CONNHANDLE`).
fn subscribers(ccc: usize) -> impl Iterator<Item = u16> {
    tables().ccc[ccc]
        .iter()
        .filter(|cfg| cfg.connHandle != INVALID_CONNHANDLE && u16::from(cfg.value) & GATT_CLIENT_CFG_NOTIFY != 0)
        .map(|cfg| cfg.connHandle)
}

/// Allocate, fill and send a notification for attribute `handle`
fn send_notification(conn_handle: u16, handle: u16, data: &[u8]) -> Result<(), Error> {
    unsafe {
        let mut noti = attHandleValueNoti_t {
            handle,
            len: data.len() as u16,
            pValue: ptr::null_mut(),
        };
        noti.pValue = GATT_bm_alloc(conn_handle, ATT_HANDLE_VALUE_NOTI, noti.len, ptr::null_mut(), 0) as *mut u8;
        if noti.pValue.is_null() {
            return Err(Error::NoMemory);
        }
        ptr::copy_nonoverlapping(data.as_ptr(), noti.pValue, data.len());
        let result = GATT_Notification(conn_handle, &mut noti, 0);
        if result.is_err() {
            GATT_bm_free(&raw mut noti as _, ATT_HANDLE_VALUE_NOTI);
        }
        result.map_err(Error::Ble)
    }
}

/// Copy `value` into the read response
fn copy_out(value: &[u8], buf: &mut [u8]) -> usize {
    let len = value.len().min(buf.len());
    buf[..len].copy_from_slice(&value[..len]);
    len
}

unsafe extern "C" fn read_attr(
    conn_handle: u16,
    attr: *mut gattAttribute_t,
    value: *mut u8,
    len: *mut u16,
    offset: u16,
    max_len: u16,
    _method: u8,
) -> u8 {
    if offset > 0 {
        return ATT_ERR_ATTR_NOT_LONG;
    }
    let buf = unsafe { core::slice::from_raw_parts_mut(value, max_len as usize) };
    let result = match tables().role(attr) {
        Some(Role::Value(ch)) => match ch.on_read {
            Some(read) => read(conn_handle, buf).map(|n| n.min(buf.len())),
            None => Err(ATT_ERR_READ_NOT_PERMITTED),
        },
        Some(Role::Description(ch)) => Ok(copy_out(ch.description.unwrap_or("").as_bytes(), buf)),
        Some(Role::Ccc(_, index)) => {
            let cfg = unsafe { GATTServApp_ReadCharCfg(conn_handle, ccc_table(index)) };
            Ok(copy_out(&cfg.to_le_bytes(), buf))
        }
        _ => Err(ATT_ERR_ATTR_NOT_FOUND),
    };
    match result {
        Ok(n) => {
            unsafe { *len = n as u16 };
            SUCCESS
        }
        Err(e) => e,
    }
}

unsafe extern "C" fn write_attr(
    conn_handle: u16,
    attr: *mut gattAttribute_t,
    value: *mut u8,
    len: u16,
    offset: u16,
    _method: u8,
) -> u8 {
    match tables().role(attr) {
        Some(Role::Value(ch)) => {
            if offset > 0 {
                return ATT_ERR_ATTR_NOT_LONG;
            }
            let data = unsafe { core::slice::from_raw_parts(value, len as usize) };
            match ch.on_write {
                Some(write) => write(conn_handle, data).err().unwrap_or(SUCCESS),
                None => ATT_ERR_WRITE_NOT_PERMITTED,
            }
        }
        Some(Role::Ccc(ch, index)) => unsafe {
            if let Err(e) =
                GATTServApp_ProcessCCCWriteReq(conn_handle, attr, value, len, offset, GATT_CLIENT_CFG_NOTIFY)
            {
                return e.get();
            }
            SUBSCRIPTION_CHANGED[index].signal(());
            if let Some(subscribe) = ch.on_subscribe {
                let cfg = GATTServApp_ReadCharCfg(conn_handle, ccc_table(index));
                subscribe(conn_handle, cfg & GATT_CLIENT_CFG_NOTIFY != 0);
            }
            SUCCESS
        },
        _ => ATT_ERR_ATTR_NOT_FOUND,
    }
}

/// Link DB callback: clear the client configurations of a closed connection
unsafe extern "C" fn link_status(conn_handle: u16, change_type: u8) {
    if conn_handle == LOOPBACK_CONNHANDLE {
        return;
    }
    let closed = change_type == LINKDB_STATUS_UPDATE_REMOVED
        || (change_type == LINKDB_STATUS_UPDATE_STATEFLAGS
            && unsafe { linkDB_State(conn_handle, LINK_CONNECTED) } == 0);
    if closed {
        for (index, changed) in SUBSCRIPTION_CHANGED.iter().enumerate().take(tables().num_ccc) {
            unsafe { GATTServApp_InitCharCfg(conn_handle, ccc_table(index)) };
            changed.signal(());
        }
    }
}
