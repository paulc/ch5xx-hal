//! Print over the 2-wire debug link (WCH "SDI print") with the tests' own
//! console (`hwtest::println!`), while the LED (PB6) blinks.
//!
//! ```console
//! wlink --chip CH585 flash --enable-sdi-print --watch-serial \
//!     target/riscv32imc-unknown-none-elf/release/sdi_print
//! ```

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};
use {panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    hwtest::sdi_enable();
    let mut led = Output::new(p.PB6, Level::High, Speed::Low);
    let mut delay = Delay;
    let mut n: u32 = 0;
    loop {
        hwtest::println!("ch584 sdi print {} (sysclk {} Hz)", n, hal::rcc::clocks().sysclk.0);
        led.toggle();
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
