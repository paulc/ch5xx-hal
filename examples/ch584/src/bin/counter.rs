//! Timer count mode test, needing no wiring. Results are reported over SDI
//! print.
//!
//! TMR0 counts edges on its own pin, PA9, while UART1 transmits on it: each
//! 'U' (0x55) has 5 rising and 5 falling edges (8N1, idle high).

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::Pull;
use hal::timer::counter::ExternalCounter;
use hal::timer::{CaptureEdge, PeriodTicks};
use hal::uart::{self, UartTx};
use hal::{bind_interrupts, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    TMR0 => hal::timer::InterruptHandler<peripherals::TMR0>;
});

struct Checks {
    pass: u32,
    fail: u32,
}

impl Checks {
    fn check(&mut self, name: &str, ok: bool, value: i64) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        hal::println!("{} {}: {}", if ok { "PASS" } else { "FAIL" }, name, value);
    }
}

/// The end register for a wrap every `period` edges: the counter wraps at
/// the end value minus 2 (the datasheet's "final count value - 2").
fn end(period: u32) -> PeriodTicks {
    PeriodTicks::new(period + 2).unwrap()
}

/// Edges per wrap at the largest end value.
const MAX_EDGES: u32 = hal::timer::MAX_PERIOD - 2;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::HSE_PLL_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut delay = Delay;
    let mut c = Checks { pass: 0, fail: 0 };
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 125_000;

    // (edge, period, bytes sent, expected count, expected wrap)
    for (edge, period, bytes, want, wrap) in [
        (CaptureEdge::RisingToRising, 1000u32, 4usize, 20u32, false),
        (CaptureEdge::FallingToFalling, 1000, 4, 20, false),
        (CaptureEdge::Both, 1000, 4, 40, false),
        (CaptureEdge::RisingToRising, 8, 4, 4, true),
        (CaptureEdge::RisingToRising, 3, 4, 2, true),
        (CaptureEdge::RisingToRising, 2, 4, 0, true),
        (CaptureEdge::Both, 10, 1, 0, true),
        (CaptureEdge::RisingToRising, MAX_EDGES, 8, 40, false),
    ] {
        let mut cnt = ExternalCounter::new::<0>(p.TMR0.reborrow(), p.PA9.reborrow(), Pull::Up, end(period), edge);
        // The UART takes PA9 as an output after the count input is set up.
        let mut tx =
            UartTx::new_blocking::<0>(p.UART1.reborrow(), unsafe { peripherals::PA9::steal() }, uart_config).unwrap();
        cnt.restart();
        let start = cnt.count();
        tx.blocking_write(&[b'U'; 8][..bytes]).unwrap();
        tx.blocking_flush().unwrap();
        delay.delay_us(20);
        let n = cnt.count();
        let wrapped = cnt.elapsed();
        hal::println!(
            "{:?}, period {}, {} x 'U': count {} -> {}, wrapped {}",
            edge,
            period,
            bytes,
            start,
            n,
            wrapped
        );
        c.check(
            "  count and wrap as expected",
            start == 0 && n == want && wrapped == wrap,
            n as i64,
        );
        if wrap {
            c.check("  wrap event consumed", !cnt.elapsed(), 0);
        }
        cnt.restart();
        c.check("  restart: count 0", cnt.count() == 0 && !cnt.elapsed(), cnt.count() as i64);
    }

    // Async: wait wakes on the 10th rising edge (2 bytes).
    {
        let mut cnt = ExternalCounter::new::<0>(
            p.TMR0.reborrow(),
            p.PA9.reborrow(),
            Pull::Up,
            end(10),
            CaptureEdge::RisingToRising,
        );
        let mut tx =
            UartTx::new_blocking::<0>(p.UART1.reborrow(), unsafe { peripherals::PA9::steal() }, uart_config).unwrap();
        cnt.restart();
        let r = select(cnt.wait(Irqs), Timer::after_millis(5)).await;
        c.check("wait waits with no edges", matches!(r, Either::Second(())), 0);
        join(cnt.wait(Irqs), async {
            Timer::after_millis(1).await;
            tx.blocking_write(b"UU").unwrap();
        })
        .await;
        let n = cnt.count();
        c.check("wait wakes on the wrap (count after)", n <= 1, n as i64);
        tx.blocking_flush().unwrap();
        delay.delay_us(20);
        c.check(
            "  and counting goes on: 10 edges, count 0",
            cnt.count() == 0,
            cnt.count() as i64,
        );
    }

    hal::println!("counter: {} passed, {} failed", c.pass, c.fail);
    loop {
        delay.delay_ms(1000);
    }
}
