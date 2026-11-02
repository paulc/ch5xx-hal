//! Independent watchdog test. Results are reported over SDI print.
//!
//! First boot: start the IWDG with a 250 ms timeout, pet it for 2 s, then
//! stop petting and print until it resets the chip. A marker in
//! `R8_GLOB_RESET_KEEP` (kept across a watchdog reset) tells the second boot
//! to report the reset cause.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::pac;
use hal::wdg::IndependentWatchdog;
use panic_halt as _;
use qingke_rt::entry;

const MARKER: u8 = 0xa5;

#[entry]
fn main() -> ! {
    let p = hal::init(hal::Config::default());
    SDIPrint::enable();
    let mut delay = Delay;
    let sc = pac::SYSTEM_CONTROL;
    let keep = sc.glob_reset_keep().read().glob_reset_keep();
    let flag = sc.reset_status().read().reset_flag();
    let iwdg_on = pac::SYS.iwdg_cfg().read().iwdg_en();
    hal::println!(
        "boot: reset flag {:03b}, keep {:#04x}, IWDG running {}",
        flag,
        keep,
        iwdg_on
    );

    if keep == MARKER {
        sc.glob_reset_keep().write(|w| w.set_glob_reset_keep(0));
        // 010 = watchdog timeout reset (WTR).
        let ok = flag == 0b010;
        hal::println!("{} reset by the watchdog", if ok { "PASS" } else { "FAIL" });
        hal::println!("WDG: done");
        let mut wdg = IndependentWatchdog::new(p.IWDG, 1_000_000);
        loop {
            // Keep it fed in case the IWDG survived the reset.
            wdg.pet();
            delay.delay_ms(100);
        }
    }

    let mut wdg = IndependentWatchdog::new(p.IWDG, 250_000);
    let cfg = pac::SYS.iwdg_cfg().read();
    hal::println!("IWDG cfg {:#010x}: pr {} rlr {}", cfg.0, cfg.pr(), cfg.rlr());
    wdg.unleash();
    delay.delay_ms(1);
    let running = pac::SYS.iwdg_cfg().read().iwdg_en();
    hal::println!("{} IWDG running after unleash", if running { "PASS" } else { "FAIL" });

    for _ in 0..20 {
        delay.delay_ms(100);
        wdg.pet();
    }
    hal::println!("PASS still alive after 2 s of petting every 100 ms");

    sc.glob_reset_keep().write(|w| w.set_glob_reset_keep(MARKER));
    hal::println!("stopped petting");
    let mut ms = 0;
    loop {
        delay.delay_ms(25);
        ms += 25;
        hal::println!("{} ms", ms);
        if ms >= 2000 {
            hal::println!("FAIL no watchdog reset after 2 s");
            sc.glob_reset_keep().write(|w| w.set_glob_reset_keep(0));
            loop {}
        }
    }
}
