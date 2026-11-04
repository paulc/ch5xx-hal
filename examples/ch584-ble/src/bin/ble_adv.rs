//! Low-power BLE advertiser (WCH's EVT `BLE/Broadcaster`): scannable
//! advertising every 100 ms, with WCH manufacturer data and a short name in
//! the advertising packet and the complete name in the scan response. The
//! HAL's executor runs TMOS and sleeps between advertising events.
//!
//! Every 5 s it logs the BLE counters on UART0 TX (PB7, 115200 8N1), and
//! embassy time against the RTC: the two should agree to the crystals'
//! tolerance, however long the chip has slept. Check the advertising with
//! a BLE scanner (a phone app, `blescan`): "ch584-advertiser".
//!
//! For a current reading, every spare pin gets a pull-up and the debug
//! interface goes off after a 2 s pause at reset, as in `sleep_current`.
//! So a debugger can only connect in that pause: power-cycle the board and
//! flash at once.
//!
//! hw-test: manual. It never reports a result.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use hal::ble::adv::AdvData;
use hal::ble::ffi::*;
use hal::ble::gap::*;
use hal::ble::{self, MacAddress};
use hal::debug::set_debug_interface_enabled;
use hal::gpio::{Input, Pull};
use hal::uart::{self, UartTx};
use panic_halt as _;

const SHORT_NAME: &str = "ch584-adv";
const NAME: &str = "ch584-advertiser";
/// Nanjing Qinheng Microelectronics Co., Ltd. (WCH)
const WCH_COMPANY_ID: u16 = 0x07D7;
/// Advertising interval in 0.625 ms units (100 ms)
const ADVERTISING_INTERVAL: u16 = 160;

static mut STATE: gapRole_States_t = 0;

static BROADCASTER_CBS: gapRolesBroadcasterCBs_t = gapRolesBroadcasterCBs_t {
    pfnStateChange: Some(broadcaster_state),
    pfnScanRecv: None,
};

/// Logged by the main loop: a callback must not block on the UART.
unsafe extern "C" fn broadcaster_state(new_state: gapRole_States_t) {
    unsafe { STATE = new_state };
}

/// `Broadcaster_Init` in the EVT.
fn broadcaster_init() -> bStatus_t {
    unsafe {
        GAPRole_BroadcasterInit()?;
        let enable: u8 = 1;
        let event_type: u8 = GAP_ADTYPE_ADV_SCAN_IND;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &raw const enable as _)?;
        GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, 1, &raw const event_type as _)?;
        let adv = AdvData::new()
            .flags(GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
            .manufacturer(WCH_COMPANY_ID, &[0x01])
            .short_name(SHORT_NAME);
        let scan_rsp = AdvData::new().name(NAME).tx_power(0);
        // The library copies the data.
        let (adv, scan_rsp) = (adv.as_slice(), scan_rsp.as_slice());
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, adv.len() as u16, adv.as_ptr() as _)?;
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, scan_rsp.len() as u16, scan_rsp.as_ptr() as _)?;
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, ADVERTISING_INTERVAL)?;
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, ADVERTISING_INTERVAL)?;
        GAPRole_BroadcasterStartDevice(&BROADCASTER_CBS)
    }
}

fn state_name(state: gapRole_States_t) -> &'static str {
    match state & GAPROLE_STATE_ADV_MASK {
        GAPROLE_INIT => "init",
        GAPROLE_STARTED => "started",
        GAPROLE_ADVERTISING => "advertising",
        GAPROLE_WAITING => "waiting",
        GAPROLE_ERROR => "error",
        _ => "?",
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry", executor = "ch5xx_hal::embassy::Executor")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    let mut tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::default()).unwrap();
    // Spare pins get a pull-up: floating inputs draw current in sleep.
    // Input drivers have no drop, so the pulls stay. Not PB7 (the log) or
    // PA10/PA11 (the 32 kHz crystal).
    macro_rules! pull_up {
        ($($pin:ident),*) => { $( let _ = Input::new(p.$pin, Pull::Up); )* };
    }
    pull_up!(PA4, PA5, PA9, PA12, PA13, PA14, PA15);
    pull_up!(PB4, PB5, PB6, PB8, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB22, PB23);

    // The UART clock stops in sleep: finish each line before the executor
    // can sleep.
    macro_rules! log {
        ($($t:tt)*) => {{
            let _ = core::fmt::Write::write_fmt(&mut tx, format_args!($($t)*));
            let _ = tx.blocking_write(b"\r\n");
            let _ = tx.blocking_flush();
        }};
    }

    log!("BLE advertiser (low power), {}", ble::lib_version());
    Timer::after_secs(2).await;
    // The debug interface draws current in sleep: off after the pause, so
    // a debugger can still catch the chip just after a reset.
    set_debug_interface_enabled(false);

    let config = ble::Config::default();
    let mac: MacAddress = config.mac_addr;
    log!("MAC {}", mac);
    if let Err(e) = ble::init(p.RTC, config) {
        log!("ble::init: {:?}", e);
        loop {}
    }
    if let Err(e) = broadcaster_init() {
        log!("broadcaster_init: {:?}", e);
        loop {}
    }

    // Embassy time against the TMOS clock (RTC ticks; it wraps once a day).
    let (t0, c0) = (Instant::now(), ble::clock());
    let mut state = 0xff;
    let mut n = 0u32;
    loop {
        Timer::after_secs(5).await;
        n += 1;
        let s = unsafe { STATE };
        if s != state {
            state = s;
            log!("state {}", state_name(s));
        }
        let ms = t0.elapsed().as_millis();
        let rtc_ms = ble::clock().wrapping_sub(c0) as u64 * 1000 / 32768;
        log!("{}: {} ms, rtc {} ms, {}", n, ms, rtc_ms, ble::STATS);
    }
}
