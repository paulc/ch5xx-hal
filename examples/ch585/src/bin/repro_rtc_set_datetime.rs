//! Repro: protected (safe-access) writes can be dropped, and
//! `Rtc::set_datetime` then spins forever.
//!
//! `set_datetime` writes the day count to `TRIG` and sets `LOAD_HI`, both
//! RWA, then spins until `CNT_DAY` shows the new day. When one of the writes
//! is dropped, the spin never ends. Whether a write inside the safe-access
//! window lands depends on the code layout: on a CH584F the hw-tests `rtc`
//! test hung on its first load in seven of eight runs, while other builds with
//! the same calls never hang, and an instrumented build lost no writes at all.
//! WCH flushes the instruction prefetch (`fence.i`) around every window
//! (`SAFEOPERATE`); the HAL did that only on CH592.
//!
//! In this build a clock switch (`rcc::set_config`) to 78 MHz just before the
//! load makes it happen every time, so the test applies the clock right before
//! each load, at three clocks.
//!
//! The window watchdog turns a hang into a result: it resets the chip when a
//! load takes longer than its period (at least 0.4 s; a load takes about
//! 200 us), and the next boot reports where it hung, kept in
//! `R8_GLOB_RESET_KEEP` across the reset. Power-cycle the board to run it
//! again.
//!
//! No wiring.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use defmt_rtt as _;
use hal::pac;
use hal::rcc::{self, Config};
use hal::rtc::{DateTime, Rtc};
use hal::watchdog::{reset_cause, ResetCause, Watchdog};
use panic_halt as _;
use qingke_rt::entry;

const PRESETS: [(&str, Config); 3] = [
    ("HSE_PLL_78MHZ", Config::HSE_PLL_78MHZ),
    ("HSE_PLL_62_4MHZ", Config::HSE_PLL_62_4MHZ),
    ("HSE_PLL_24MHZ", Config::HSE_PLL_24MHZ),
];
/// Loads at each clock.
const LOADS: u32 = 8;

#[entry]
fn main() -> ! {
    let p = hal::init(Config::HSE_PLL_62_4MHZ.into());
    let keep = pac::SYSTEM_CONTROL.glob_reset_keep();
    let (mut pass, mut fail) = (0u32, 0u32);

    if reset_cause() == ResetCause::Watchdog {
        let k = keep.read().glob_reset_keep() as u32;
        let name = PRESETS.get((k / LOADS) as usize).map_or("?", |p| p.0);
        fail += 1;
        defmt::info!(
            "set_datetime at {}, load {}: hung, the watchdog reset the chip: FAIL",
            name,
            k % LOADS
        );
    } else {
        let mut rtc = Rtc::new(p.RTC);
        let watchdog = Watchdog::new(p.SYSTEM_CONTROL, 0);
        watchdog.enable_reset();
        for (i, (name, config)) in PRESETS.iter().enumerate() {
            let mut loaded = 0;
            for n in 0..LOADS {
                keep.write(|w| w.set_glob_reset_keep((i as u32 * LOADS + n) as u8));
                let set = DateTime {
                    year: 2026,
                    month: 11,
                    day: 8 + (n % 2) as u8,
                    hour: 12,
                    minute: 34,
                    second: 0,
                };
                watchdog.feed(0);
                unsafe { rcc::set_config(*config) };
                if rtc.set_datetime(set).is_ok() && rtc.datetime().day == set.day {
                    loaded += 1;
                }
            }
            unsafe { rcc::set_config(Config::HSE_PLL_62_4MHZ) };
            let ok = loaded == LOADS;
            if ok {
                pass += 1;
            } else {
                fail += 1;
            }
            defmt::info!(
                "set_datetime at {}: {} of {} loads: {}",
                name,
                loaded,
                LOADS,
                if ok { "PASS" } else { "FAIL" }
            );
        }
        watchdog.disable_reset();
    }

    loop {
        defmt::info!("repro_rtc_set_datetime done: {} pass, {} fail", pass, fail);
        qingke::riscv::asm::delay(20_000_000);
    }
}
