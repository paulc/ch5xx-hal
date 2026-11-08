//! embassy-time on the SysTick time driver.
//!
//! Checks `with_timeout`, then prints `Instant::now()` every 500 ms from one
//! task while another blinks the LED (PB6) every 100 ms: compare the host
//! timestamps with the printed instants; both should advance 500 ms per line.
//!
//! Runs at 5.33 MHz (HSI / 3, the reset rate) by default, which exercises the
//! driver's non-integer counts-per-tick conversion; set `CLOCK` to test
//! another rate.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use hal::gpio::{Level, Output, Speed};
use hwtest::check;
use panic_halt as _;

/// Clock to test at.
const CLOCK: hal::rcc::Config = hal::rcc::Config::HSI_5_33MHZ;

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.toggle();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(CLOCK.into());
    hwtest::sdi_enable();

    let led = Output::new(p.PB6, Level::High, Speed::Low);
    spawner.spawn(blink(led).unwrap());

    // A future that never completes must time out.
    let t0 = Instant::now();
    let r = with_timeout(Duration::from_millis(50), core::future::pending::<()>()).await;
    let dt = t0.elapsed().as_micros();
    check!(
        r.is_err() && (50_000..52_000).contains(&dt),
        "with_timeout 50 ms: after {} us",
        dt
    );

    // 20 short timers: how late does a 1 ms timer fire?
    let mut worst = 0;
    for _ in 0..20 {
        let t0 = Instant::now();
        Timer::after_millis(1).await;
        worst = worst.max(t0.elapsed().as_micros());
    }
    check!(worst < 1_500, "Timer::after_millis(1) x20: worst {} us", worst);

    let start = Instant::now();
    let mut next = start;
    loop {
        next += Duration::from_millis(500);
        Timer::at(next).await;
        hwtest::println!(
            "sysclk {} Hz, now {} us (late {} us)",
            hal::rcc::clocks().sysclk.0,
            (Instant::now() - start).as_micros(),
            (Instant::now() - next).as_micros()
        );
    }
}
