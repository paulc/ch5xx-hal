//! embassy-time on the SysTick time driver.
//!
//! Prints `Instant::now()` every 500 ms from one task while another blinks the
//! LED (PB6) every 100 ms. Compare the host timestamps with the printed
//! instants: both should advance 500 ms per line. Also checks `with_timeout`.
//!
//! Runs at 5.33 MHz (HSI / 3, the reset rate) by default, which exercises the
//! driver's exact (non-integer counts-per-tick) conversion; set `CLOCK` to
//! test another rate. Selecting it through `rcc` rather than keeping the reset
//! setting also sets WCH's flash timing for it, without which the CH584 runs
//! about twice as slowly and `with_timeout` overshoots by 4 ms.
//!
//! Run it at 78 MHz for more than a minute to see the 32-bit SysTick wrap
//! (every 55 s) go by unnoticed.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use hal::debug::SDIPrint;
use hal::gpio::{Level, Output, Speed};
use panic_halt as _;

/// Clock to test at.
const CLOCK: hal::rcc::Config = hal::rcc::Config::new(hal::rcc::Sysclk::Hsi { div: 3 });

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.toggle();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: CLOCK,
        ..Default::default()
    });
    SDIPrint::enable();

    let led = Output::new(p.PB6, Level::High, Speed::Low);
    spawner.spawn(blink(led).unwrap());

    // A future that never completes must time out.
    let t0 = Instant::now();
    let r = with_timeout(Duration::from_millis(50), core::future::pending::<()>()).await;
    let dt = t0.elapsed().as_micros();
    hal::try_println!(
        "with_timeout 50 ms: {} after {} us",
        if r.is_err() && (50_000..52_000).contains(&dt) { "PASS" } else { "FAIL" },
        dt
    );

    let start = Instant::now();
    let mut next = start;
    loop {
        next += Duration::from_millis(500);
        Timer::at(next).await;
        hal::try_println!(
            "sysclk {} Hz, now {} us (late {} us)",
            hal::rcc::clocks().sysclk.0,
            (Instant::now() - start).as_micros(),
            (Instant::now() - next).as_micros()
        );
    }
}
