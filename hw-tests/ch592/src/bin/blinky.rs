//! Blink the LED on the WeAct CH592F core board (PA8, active low).
//!
//! On WCH's CH592 EVT board the LEDs are not tied to GPIOs; wire P2-18 (PA8)
//! to P2-22 (LED0), which is also active low.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};
use {panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    // Start with the LED off.
    let mut led = Output::new(p.PA8, Level::High, Speed::Low);
    let mut delay = Delay;
    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
