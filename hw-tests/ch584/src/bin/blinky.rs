//! Blink the LED on the WeAct CH584F core board (PB6, active low).

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};
// The panic handler.
use hwtest as _;
use qingke_rt::entry;

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    // Start with the LED off.
    let mut led = Output::new(p.PB6, Level::High, Speed::Low);
    let mut delay = Delay;
    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
