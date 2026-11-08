//! Window watchdog and reset-cause test. Results are reported over SDI print.
//!
//! The HAL drives the 8-bit `WDOG` (Fsys / 131072 per count) and has no
//! independent watchdog, so this replaces the original IWDG test.
//!
//! - First boot: the overflow period with reset disabled, the overflow flag
//!   and its clear, then reset enabled and fed every 100 ms for 2 s; then it
//!   stops feeding and prints until the watchdog resets the chip.
//! - Second boot: the reset cause is `Watchdog`; then a software reset.
//! - Third boot: the reset cause is `Software`, and the summary.
//!
//! A marker in `R8_GLOB_RESET_KEEP` (kept across a watchdog or software reset)
//! says which boot this is. Counts from earlier boots are carried in it too.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_time::{Duration, Instant};
use hal::pac;
use hal::watchdog::{self, ResetCause, Watchdog};
use hwtest::check;
use panic_halt as _;
use qingke_rt::entry;

/// Second boot: the high nibble marks the stage, the low nibble carries the
/// first boot's failures (saturating).
const STAGE_WDOG: u8 = 0xa0;
const STAGE_SOFT: u8 = 0x50;

fn wait_ms(ms: u64) {
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_millis(ms) {}
}

/// Milliseconds for `counts` watchdog counts (Fsys / 131072 each).
fn counts_ms(counts: u64) -> u64 {
    counts * 131_072 * 1000 / hal::rcc::clocks().sysclk.0 as u64
}

/// Within 5% of `want`.
fn near(ms: u64, want: u64) -> bool {
    ms.abs_diff(want) * 20 <= want
}

fn keep(v: u8) {
    pac::SYSTEM_CONTROL
        .glob_reset_keep()
        .write(|w| w.set_glob_reset_keep(v));
}

#[entry]
fn main() -> ! {
    let p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();
    let marker = pac::SYSTEM_CONTROL.glob_reset_keep().read().glob_reset_keep();
    let cause = watchdog::reset_cause();
    hwtest::println!("boot: reset cause {:?}, keep {:#04x}", cause, marker);

    let earlier = (marker & 0x0f) as u32;
    match marker & 0xf0 {
        STAGE_WDOG => {
            check!(cause == ResetCause::Watchdog, "reset by the watchdog ({:?})", cause);
            let wdg = Watchdog::new(p.SYSTEM_CONTROL, 0);
            wdg.disable_reset();
            let fail = earlier + hwtest::counts().1;
            keep(STAGE_SOFT | fail.min(15) as u8);
            hwtest::println!("software reset");
            wait_ms(50);
            watchdog::software_reset();
        }
        STAGE_SOFT => {
            keep(0);
            check!(cause == ResetCause::Software, "reset by software ({:?})", cause);
            if earlier > 0 {
                hwtest::println!("earlier boots: {} failed", earlier);
            }
            check!(earlier == 0, "earlier boots passed");
            hwtest::done("wdg")
        }
        _ => {}
    }

    // 256 counts from a reload of 0: 538 ms at 62.4 MHz.
    let wdg = Watchdog::new(p.SYSTEM_CONTROL, 0);
    wdg.disable_reset();
    wdg.clear_overflow();
    check!(!wdg.overflowed(), "not overflowed after new");
    let t0 = Instant::now();
    while !wdg.overflowed() && t0.elapsed() < Duration::from_millis(2000) {}
    let ms = t0.elapsed().as_millis();
    check!(
        near(ms, counts_ms(256)),
        "overflow from reload 0 after ~{} ms ({} ms)",
        counts_ms(256),
        ms
    );
    check!(wdg.overflowed(), "still alive with reset disabled");
    wdg.clear_overflow();
    check!(!wdg.overflowed(), "clear_overflow clears it");

    // Reload 192: 64 counts.
    wdg.feed(192);
    let t0 = Instant::now();
    while !wdg.overflowed() && t0.elapsed() < Duration::from_millis(2000) {}
    let ms = t0.elapsed().as_millis();
    check!(
        near(ms, counts_ms(64)),
        "overflow from reload 192 after ~{} ms ({} ms)",
        counts_ms(64),
        ms
    );
    wdg.feed(192);
    check!(!wdg.overflowed(), "feed clears the overflow");

    // Reset enabled, reload 128 (128 counts, 269 ms at 62.4 MHz), fed every
    // 100 ms.
    wdg.feed(128);
    wdg.enable_reset();
    for _ in 0..20 {
        wait_ms(100);
        wdg.feed(128);
    }
    check!(true, "still alive after 2 s of feeding every 100 ms");

    let fail = hwtest::counts().1;
    keep(STAGE_WDOG | fail.min(15) as u8);
    hwtest::println!("stopped feeding");
    let mut ms = 0;
    loop {
        wait_ms(25);
        ms += 25;
        hwtest::println!("{} ms", ms);
        if ms >= 2000 {
            keep(0);
            check!(false, "watchdog reset within 2 s");
            hwtest::done("wdg")
        }
    }
}
