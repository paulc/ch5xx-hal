//! Print over the 2-wire debug link (WCH "SDI print"), no UART needed.
//!
//! Uses [`ch5xx_hal::debug::SDIPrint`] (see its docs). WCH does not document
//! SDI print for CH59x, but it works (WeAct CH592F board, WCH-LinkE firmware
//! v3.2).
//!
//! wlink 0.1.3 refuses to arm SDI print for `CH59X`: add `RiscvChip::CH59X` to
//! `support_sdi_print()` in wlink's `src/lib.rs`, then
//!
//! ```console
//! wlink --chip CH59X --speed low flash --enable-sdi-print --watch-serial \
//!     target/riscv32imac-unknown-none-elf/release/sdi_print
//! ```
//!
//! `try_println!` is lossy: with nobody draining the console a message is
//! dropped after a short wait instead of hanging, and the LED (PA8 on the
//! WeAct board) keeps blinking either way.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};
use {panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    SDIPrint::enable();

    let mut led = Output::new(p.PA8, Level::High, Speed::Low);
    let mut delay = Delay;
    let mut n: u32 = 0;

    loop {
        hal::try_println!("ch592 sdi print {} (sysclk {} Hz)", n, hal::rcc::clocks().sysclk.0);
        led.toggle();
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
