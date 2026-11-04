//! Connectable BLE peripheral with a GATT service, using the
//! `ble::gatt_server` and `ble::peripheral` helpers (WCH's EVT
//! `BLE/Peripheral`, cut down).
//!
//! Service 0xFFE0:
//! - 0xFFE1 "Data": read/write, up to 20 bytes
//! - 0xFFE2 "Counter": read/notify, u32 LE, incremented and notified every
//!   second while a client has notifications enabled (paused otherwise)
//!
//! The characteristic callbacks run inside TMOS: they only update state and
//! signal the main task, which logs on UART0 TX (PB7, 115200 8N1). The
//! counter has its own task, which notifies every subscribed connection
//! (`notify_all`) without tracking connections itself.
//!
//! The HAL's executor runs TMOS and sleeps between radio events, as in
//! `ble_adv`. Connect with a BLE app (e.g. nRF Connect) to
//! "ch584-peripheral".
//!
//! hw-test: manual. It needs a BLE central.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use hal::ble::gap::*;
use hal::ble::gatt::*;
use hal::ble::gatt_server::{self, Characteristic, Service, Uuid};
use hal::ble::peripheral::{self, AdvData, Event};
use hal::ble::{self, MacAddress};
use hal::mode::Blocking;
use hal::peripherals::UART0;
use hal::uart::{self, UartTx};
use panic_halt as _;

const SHORT_NAME: &str = "ch584-per";
const DEVICE_NAME: &str = "ch584-peripheral";
const SERVICE_UUID: u16 = 0xFFE0;
const DATA_MAX_LEN: usize = 20;

/// A value of up to `DATA_MAX_LEN` bytes.
#[derive(Clone, Copy)]
struct Data {
    buf: [u8; DATA_MAX_LEN],
    len: usize,
}

impl Data {
    const fn new() -> Self {
        Self {
            buf: [0; DATA_MAX_LEN],
            len: 0,
        }
    }

    fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/* Application state, shared between the GATT callbacks and the main task */

static DATA: Mutex<CriticalSectionRawMutex, RefCell<Data>> = Mutex::new(RefCell::new(Data::new()));
static COUNTER: AtomicU32 = AtomicU32::new(0);

/// Signals from the GATT callbacks to the main task
static DATA_WRITTEN: Signal<CriticalSectionRawMutex, Data> = Signal::new();
static SUBSCRIBED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// The log, shared by the tasks.
static LOG: Mutex<CriticalSectionRawMutex, RefCell<Option<UartTx<'static, UART0, Blocking>>>> =
    Mutex::new(RefCell::new(None));

/// Log a line. It is sent before this returns: the UART clock stops in
/// sleep.
macro_rules! log {
    ($($t:tt)*) => {
        LOG.lock(|tx| {
            if let Some(tx) = tx.borrow_mut().as_mut() {
                let _ = core::fmt::Write::write_fmt(tx, format_args!($($t)*));
                let _ = tx.blocking_write(b"\r\n");
                let _ = tx.blocking_flush();
            }
        })
    };
}

/* GATT service */

static DATA_CHAR: Characteristic = Characteristic::new(Uuid::u16(0xFFE1), GATT_PROP_READ | GATT_PROP_WRITE)
    .description("Data")
    .on_read(&|_conn, buf| {
        DATA.lock(|data| {
            let data = data.borrow();
            let len = data.len.min(buf.len());
            buf[..len].copy_from_slice(&data.buf[..len]);
            Ok(len)
        })
    })
    .on_write(&|_conn, value| {
        if value.len() > DATA_MAX_LEN {
            return Err(ATT_ERR_INVALID_VALUE_SIZE);
        }
        let mut data = Data::new();
        data.buf[..value.len()].copy_from_slice(value);
        data.len = value.len();
        DATA.lock(|d| *d.borrow_mut() = data);
        DATA_WRITTEN.signal(data);
        Ok(())
    });

static COUNTER_CHAR: Characteristic = Characteristic::new(Uuid::u16(0xFFE2), GATT_PROP_READ | GATT_PROP_NOTIFY)
    .description("Counter")
    .on_read(&|_conn, buf| {
        let value = COUNTER.load(Ordering::Relaxed).to_le_bytes();
        buf[..value.len()].copy_from_slice(&value);
        Ok(value.len())
    })
    .on_subscribe(&|_conn, enabled| SUBSCRIBED.signal(enabled));

static SERVICE: Service = Service::new(Uuid::u16(SERVICE_UUID), &[&DATA_CHAR, &COUNTER_CHAR]);

/// Increment the counter every second while a client is subscribed, and
/// notify all subscribers.
#[embassy_executor::task]
async fn counter_task() -> ! {
    loop {
        // Pause while nobody is listening.
        if let Err(e) = gatt_server::wait_subscribed(&COUNTER_CHAR).await {
            log!("wait_subscribed: {:?}", e);
            loop {}
        }
        Timer::after_secs(1).await;
        // The client may have unsubscribed or gone during the sleep.
        if !gatt_server::is_subscribed_any(&COUNTER_CHAR) {
            continue;
        }
        let counter = COUNTER.load(Ordering::Relaxed).wrapping_add(1);
        COUNTER.store(counter, Ordering::Relaxed);
        if let Err(e) = gatt_server::notify_all(&COUNTER_CHAR, &counter.to_le_bytes()) {
            log!("notify failed: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn stats_task() -> ! {
    loop {
        Timer::after_secs(10).await;
        log!("{}", ble::STATS);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry", executor = "ch5xx_hal::embassy::Executor")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    let tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::default()).unwrap();
    LOG.lock(|log| *log.borrow_mut() = Some(tx));
    log!("BLE peripheral, {}", ble::lib_version());
    // A chance for a debugger to connect before the chip starts sleeping.
    Timer::after_secs(2).await;

    // A random address, so a scanner does not show a cached name.
    let mut config = ble::Config::default();
    config.mac_addr = MacAddress::random();
    log!("MAC {}", config.mac_addr);
    if let Err(e) = ble::init(p.RTC, config) {
        log!("ble::init: {:?}", e);
        loop {}
    }

    let adv = AdvData::new()
        .flags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
        .uuid16_list(&[SERVICE_UUID])
        .short_name(SHORT_NAME);
    let scan_rsp = AdvData::new().name(DEVICE_NAME).conn_interval_range(6, 100);
    let peripheral_config = peripheral::Config::new(DEVICE_NAME, &adv, &scan_rsp);
    if let Err(e) = peripheral::start(&peripheral_config, &[&SERVICE]) {
        log!("peripheral::start: {:?}", e);
        loop {}
    }
    log!("advertising");

    spawner.spawn(counter_task().unwrap());
    spawner.spawn(stats_task().unwrap());

    loop {
        match select3(peripheral::next_event(), DATA_WRITTEN.wait(), SUBSCRIBED.wait()).await {
            Either3::First(event) => match event {
                Event::Connected { handle, interval, peer } => {
                    log!(
                        "connected: handle {:#x}, interval {} x 1.25 ms, peer {}",
                        handle,
                        interval,
                        peer
                    );
                }
                Event::Disconnected { handle, reason } => {
                    log!("disconnected: handle {:#x}, reason {:#x}", handle, reason);
                }
                Event::ParamUpdate { handle, interval, .. } => {
                    log!("param update: handle {:#x}, interval {} x 1.25 ms", handle, interval);
                }
                Event::Mtu { mtu, .. } => log!("MTU {}", mtu),
                Event::ScanRequest { peer } => log!("scan request from {}", peer),
            },
            Either3::Second(data) => log!("data written: {:02x?}", data.as_slice()),
            Either3::Third(enabled) => log!("counter notifications {}", if enabled { "on" } else { "off" }),
        }
    }
}
