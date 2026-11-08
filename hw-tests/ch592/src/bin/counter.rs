//! Timer count mode tests, needing no wiring. Results are reported over SDI
//! print.
//!
//! `ExternalCounter`: TMR0 counts edges on its own pin, PA9, while UART1
//! transmits on it. Each 'U' (0x55) has 5 rising and 5 falling edges (8N1,
//! idle high). This HAL's `ExternalCounter` has no `restart`, `elapsed` or
//! `wait`, so a fresh counter is made per case and only the count is checked.
//!
//! `Counter`: TMR1 counts HCLK; its period event (`elapsed`, `wait`) and
//! `restart` are checked against embassy time.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Instant, Timer};
use hal::delay::Delay;
use hal::gpio::Pull;
use hal::timer::counter::{Counter, ExternalCounter};
use hal::timer::{CaptureEdge, PeriodTicks};
use hal::uart::{self, UartTx};
use hal::{bind_interrupts, peripherals};
use hwtest::check;

bind_interrupts!(struct Irqs {
    TMR1 => hal::timer::InterruptHandler<peripherals::TMR1>;
});

/// The end register for a wrap every `period` edges: the counter wraps at
/// the end value minus 2 (the datasheet's "final count value - 2").
fn end(period: u32) -> PeriodTicks {
    PeriodTicks::new(period + 2).unwrap()
}

/// Edges per wrap at the largest end value.
const MAX_EDGES: u32 = hal::timer::MAX_PERIOD - 2;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();
    let mut delay = Delay;

    let uart_config = uart::Config::new(125_000);
    // (edge, period, bytes sent, expected count)
    for (edge, period, bytes, want) in [
        (CaptureEdge::RisingToRising, 1000u32, 4usize, 20u32),
        (CaptureEdge::FallingToFalling, 1000, 4, 20),
        (CaptureEdge::Both, 1000, 4, 40),
        (CaptureEdge::RisingToRising, 8, 4, 4),
        (CaptureEdge::RisingToRising, 3, 4, 2),
        (CaptureEdge::RisingToRising, 2, 4, 0),
        (CaptureEdge::Both, 10, 1, 0),
        (CaptureEdge::RisingToRising, MAX_EDGES, 8, 40),
    ] {
        let cnt = ExternalCounter::new::<0>(p.TMR0.reborrow(), p.PA9.reborrow(), Pull::Up, end(period), edge);
        // The UART takes PA9 as an output after the count input is set up.
        let mut tx =
            UartTx::new_blocking::<0>(p.UART1.reborrow(), unsafe { peripherals::PA9::steal() }, uart_config).unwrap();
        let start = cnt.count();
        tx.blocking_write(&[b'U'; 8][..bytes]).unwrap();
        tx.blocking_flush().unwrap();
        delay.delay_us(20);
        let n = cnt.count();
        check!(
            start == 0 && n == want,
            "external {:?}, period {}, {} x 'U': count {} -> {} (want {})",
            edge,
            period,
            bytes,
            start,
            n,
            want
        );
    }
    hwtest::skip("external restart/elapsed/wait", "not in this HAL's ExternalCounter");

    // Internal counter: 1 ms period at 60 MHz.
    let period = PeriodTicks::from_hz(60_000_000, 1000).unwrap();
    let mut cnt = Counter::new(p.TMR1, period);
    let _ = cnt.elapsed();
    Timer::after_micros(500).await;
    check!(!cnt.elapsed(), "counter: no period event after 0.5 ms");
    Timer::after_micros(1000).await;
    // Both reads before printing: a printed line takes longer than a period.
    let (first, second) = (cnt.elapsed(), cnt.elapsed());
    check!(first, "counter: period event after 1.5 ms");
    check!(!second, "counter: elapsed consumes the event");
    cnt.restart(period);
    let c = cnt.count();
    check!(c < 60_000 / 10, "counter: restart starts from 0 (count {})", c);

    // wait: wakes once a period.
    let t0 = Instant::now();
    for _ in 0..10 {
        cnt.wait(Irqs).await;
    }
    let us = t0.elapsed().as_micros();
    check!((9_000..=10_500).contains(&us), "counter: 10 x wait took {} us", us);

    // wait with the counter stopped must not return.
    // The event is sticky: drop one left from before the stop.
    cnt.stop();
    let _ = cnt.elapsed();
    let r = select(cnt.wait(Irqs), Timer::after_millis(5)).await;
    check!(matches!(r, Either::Second(())), "counter: wait waits while stopped");

    hwtest::done("counter")
}
