//! Blink the LED on the WeAct CH592F core board.
//!
//! The LED (D2) is on PA8 and is active low (anode to 3V3 through R1).
//!
//! On WCH's CH592 EVT board the LEDs are not tied to GPIOs; wire P2-18 (PA8)
//! to P2-22 (LED0), which is also active low.

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

    // Start with the LED off.
    let mut led = Output::new(p.PA8, Level::High, Speed::Low);

    let mut delay = Delay;

    loop {
        led.toggle();
        unsafe { BLINKS = BLINKS.wrapping_add(1) };
        delay.delay_ms(500);
    }
}
