//! Print over the 2-wire debug link (WCH "SDI print"), no UART needed.
//!
//! Uses [`ch5xx_hal::debug::SDIPrint`] (see its docs). WCH does not document
//! SDI print for CH58x, but it works (WeAct CH584F board, WCH-LinkE firmware
//! v3.2).
//!
//! wlink 0.1.3 refuses to arm SDI print for `CH585`: add `RiscvChip::CH585` to
//! `support_sdi_print()` in wlink's `src/lib.rs`, then
//!
//! ```console
//! wlink --chip CH585 flash --enable-sdi-print --watch-serial \
//!     target/riscv32imc-unknown-none-elf/release/sdi_print
//! ```
//!
//! `try_println!` is lossy: with nobody draining the console a message is
//! dropped after a short wait instead of hanging, and the LED (PB6 on the
//! WeAct board) keeps blinking either way.
//!
//! `sdi_writeln!` writes raw bytes with no formatting. It too gives up when
//! nobody drains the console, and then stays silent until the next
//! `SDIPrint::enable()`.

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

    let mut led = Output::new(p.PB6, Level::High, Speed::Low);
    let mut delay = Delay;
    let mut n: u32 = 0;

    loop {
        hal::try_println!("ch584 sdi print {} (sysclk {} Hz)", n, hal::rcc::clocks().sysclk.0);
        let word = if n % 2 == 0 { b"tick" } else { b"tock" };
        hal::sdi_writeln!(b"raw:", b"no", b"formatting", word);
        led.toggle();
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
