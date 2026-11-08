//! Async (interrupt-driven) UART test on UART0's alternate pins, RX = PA15 and
//! TX = PA14 (`REMAP` = 1).
//!
//! Without wiring it checks the TX side (`write` and `flush` complete only if
//! the UART0 interrupt is delivered) and the RX side on bytes preloaded into
//! the RX FIFO through the internal loopback. With a jumper from PA14 to PA15
//! it also checks concurrent `write`/`read` through the 8-byte FIFOs and
//! `read_some`. (The internal loopback cannot be used throughout: it
//! disconnects the UART's interrupt.) Results are reported over SDI print.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{with_timeout, Duration, Timer};
use hal::delay::Delay;
use hal::gpio::{Input, Level, Output, Pull, Speed};
use hal::mode::Async;
use hal::uart::{self, Uart, UartRx};
use hal::{bind_interrupts, peripherals};
use hwtest::check;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    UART0 => uart::InterruptHandler<peripherals::UART0>;
});

/// Collect `n` bytes with `read_some`, giving up after 50 ms.
async fn read_some_n(rx: &mut UartRx<'_, peripherals::UART0, Async>, buf: &mut [u8], n: usize) -> usize {
    let mut got = 0;
    while got < n {
        match with_timeout(Duration::from_millis(50), rx.read_some(&mut buf[got..n])).await {
            Ok(Ok(k)) if k > 0 => got += k,
            _ => break,
        }
    }
    got
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
    hwtest::sdi_enable();
    let mut delay = Delay;

    // Is PA14 wired to PA15?
    let jumper = {
        let mut out = Output::new(p.PA14.reborrow(), Level::Low, Speed::Low);
        let inp = Input::new(p.PA15.reborrow(), Pull::Up);
        delay.delay_us(10);
        let low = inp.is_low();
        out.set_high();
        delay.delay_us(10);
        low && inp.is_high()
    };
    hwtest::println!(
        "PA14-PA15 jumper: {}",
        if jumper { "yes" } else { "no (TX-only tests)" }
    );

    let uart = Uart::new::<1>(p.UART0, p.PA15, p.PA14, Irqs, uart::Config::new(1_000_000)).unwrap();
    let (mut tx, mut rx) = uart.split();

    let mut data = [0u8; 64];
    for (i, b) in data.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(37);
    }

    if !jumper {
        check!(tx.write(&data).await.is_ok(), "async write 64 bytes");
        check!(tx.flush().await.is_ok(), "flush");

        // Async RX without a wire: fill the RX FIFO through the internal
        // loopback (which masks the interrupt), then leave loopback and let
        // the interrupt-driven read collect it. 6 bytes is above the RX
        // trigger (4), so this exercises both the RX-ready interrupt and the
        // FIFO timeout.
        let mcr = hal::pac::UART0.mcr();
        mcr.modify(|w| w.set_mcr_loop(true));
        tx.blocking_write(b"loop!\n").ok();
        tx.blocking_flush().ok();
        delay.delay_us(100);
        mcr.modify(|w| w.set_mcr_loop(false));
        let mut buf = [0u8; 32];
        let n = read_some_n(&mut rx, &mut buf, 6).await;
        check!(
            n == 6 && &buf[..6] == b"loop!\n",
            "async read_some (FIFO preload): {} bytes",
            n
        );
        // Nothing more: read_some must keep waiting.
        let r = with_timeout(Duration::from_millis(20), rx.read_some(&mut buf)).await;
        check!(r.is_err(), "read_some waits with nothing to read");
    } else {
        // 64 bytes each way at once: more than the FIFOs hold, so both sides
        // have to wait on interrupts.
        let mut got = [0u8; 64];
        let (w, r) = join(tx.write(&data), rx.read(&mut got)).await;
        check!(w.is_ok() && r.is_ok() && got == data, "async write+read 64 bytes");

        // read_some returns a prefix as soon as bytes arrive.
        let mut buf = [0u8; 32];
        let (_, n) = join(tx.write(b"hello, ch584!"), read_some_n(&mut rx, &mut buf, 13)).await;
        check!(n == 13 && &buf[..13] == b"hello, ch584!", "read_some 13 bytes: {}", n);
        let (_, n) = join(tx.write(b"12345678"), read_some_n(&mut rx, &mut buf, 8)).await;
        check!(n == 8 && &buf[..8] == b"12345678", "read_some 8 bytes: {}", n);

        // A cancelled read_some consumes nothing.
        let r = with_timeout(Duration::from_millis(5), rx.read_some(&mut buf)).await;
        let (_, n) = join(
            async {
                Timer::after_millis(1).await;
                tx.write(b"abc").await
            },
            read_some_n(&mut rx, &mut buf, 3),
        )
        .await;
        check!(
            r.is_err() && n == 3 && &buf[..3] == b"abc",
            "read_some after a cancelled read_some"
        );
        check!(tx.flush().await.is_ok(), "flush");
    }

    hwtest::done("uart_async")
}
