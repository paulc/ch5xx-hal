//! Blink the two LEDs on the CH592 EVT board.
//!
//! The board's LEDs are not tied to GPIOs; they sit on the extension headers
//! and are active low (anode to 3V3). Wire them up before running:
//!
//! - LED0: jumper P2-20 (PA9) to P2-22 (LED0)
//! - LED1: wire P3-15 (PB13) to P3-21 (LED1)
//!
//! PB15, next to LED1 on P3, is avoided because it is the debug clock (TCK)
//! together with PB14 (TIO).
//!
//! The LEDs are driven in opposite phase so a working build is obvious even
//! with only one wired.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};

use {panic_halt as _, qingke_rt::entry};

/// Iteration counter, so progress can be observed with a single memory read
/// (every debugger read halts the core, so sampling `PA_OUT` freezes it).
#[unsafe(no_mangle)]
static mut BLINKS: u32 = 0;

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut led0 = Output::new(p.PA9, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PB13, Level::High, Speed::Low);

    let mut delay = Delay;

    loop {
        led0.toggle();
        led1.toggle();
        unsafe { BLINKS = BLINKS.wrapping_add(1) };
        delay.delay_ms(500);
    }
}
