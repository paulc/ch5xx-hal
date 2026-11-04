//! Blink the LED on the WeAct CH584F core board.
//!
//! The LED is on PB6 and is active low.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};

use {panic_halt as _, qingke_rt::entry};

/// Iteration counter, so progress can be observed with a single memory read
/// (every debugger read halts the core, so sampling `PB_OUT` freezes it).
#[unsafe(no_mangle)]
static mut BLINKS: u32 = 0;

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    // Start with the LED off.
    let mut led = Output::new(p.PB6, Level::High, Speed::Low);

    let mut delay = Delay;

    loop {
        led.toggle();
        unsafe { BLINKS = BLINKS.wrapping_add(1) };
        delay.delay_ms(500);
    }
}
