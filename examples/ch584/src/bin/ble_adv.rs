//! BLE advertiser (WCH's EVT `BLE/Broadcaster`): scannable advertising every
//! 100 ms, with WCH manufacturer data and a short name in the advertising
//! packet and the complete name in the scan response. TMOS runs in the
//! `ble` task; a second task prints the BLE counters every 5 s over SDI
//! print.
//!
//! Build with `--features ble`. Check with a BLE scanner (a phone app,
//! `blescan`): "ch584-advertiser".
//!
//! hw-test: manual. It never reports a result.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::ble::adv::AdvData;
use hal::ble::ffi::*;
use hal::ble::gap::*;
use hal::ble::{self, MacAddress};
use hal::println;
use panic_halt as _;

const SHORT_NAME: &str = "ch584-adv";
const NAME: &str = "ch584-advertiser";
/// Nanjing Qinheng Microelectronics Co., Ltd. (WCH)
const WCH_COMPANY_ID: u16 = 0x07D7;
/// Advertising interval in 0.625 ms units (100 ms)
const ADVERTISING_INTERVAL: u16 = 160;

static BROADCASTER_CBS: gapRolesBroadcasterCBs_t = gapRolesBroadcasterCBs_t {
    pfnStateChange: Some(broadcaster_state),
    pfnScanRecv: None,
};

unsafe extern "C" fn broadcaster_state(new_state: gapRole_States_t) {
    match new_state & GAPROLE_STATE_ADV_MASK {
        GAPROLE_STARTED => println!("initialized"),
        GAPROLE_ADVERTISING => println!("advertising"),
        GAPROLE_WAITING => println!("waiting for advertising"),
        GAPROLE_ERROR => println!("error"),
        _ => println!("broadcaster state {:#x}", new_state),
    }
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

/// PFIC VTF slots: the IRQ number and handler address (bit 0: enabled).
fn print_vtf() {
    for slot in 0..4u32 {
        let id = unsafe { ((0xE000_E050 + slot) as *const u8).read_volatile() };
        let addr = unsafe { ((0xE000_E060 + 4 * slot) as *const u32).read_volatile() };
        println!("VTF{}: irq {} -> {:#010x}", slot, id, addr);
    }
}

#[embassy_executor::task]
async fn ble_task() -> ! {
    ble::run().await
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    println!("BLE advertiser, {}", ble::lib_version());
    let config = ble::Config::default();
    let mac: MacAddress = config.mac_addr;
    println!("MAC {}", mac);
    if let Err(e) = ble::init(p.RTC, config) {
        println!("ble::init: {:?}", e);
        loop {}
    }
    if let Err(e) = broadcaster_init() {
        println!("broadcaster_init: {:?}", e);
        loop {}
    }
    print_vtf();
    spawner.spawn(ble_task().unwrap());
    let mut n = 0u32;
    loop {
        Timer::after_secs(5).await;
        n += 1;
        println!("{}: {}", n, ble::STATS);
        if n == 1 {
            print_vtf();
        }
    }
}
