//! Async (interrupt-driven) UART test on UART0's alternate pins, RX = PA15 and
//! TX = PA14 (`REMAP` = 1).
//!
//! Without wiring it checks the TX side (`write` and `flush` complete only if
//! the UART0 interrupt is delivered) and the RX side on bytes preloaded into
//! the RX FIFO through the internal loopback. With a jumper from PA14 to PA15 it also
//! checks concurrent `write`/`read` through the 8-byte FIFOs and
//! `read_until_idle`. (The internal loopback cannot be used here: it
//! disconnects the UART's interrupt.) Results are reported over SDI print.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::{Input, Level, Output, Pull, Speed};
use hal::uart::{self, Uart};
use hal::{bind_interrupts, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    UART0 => uart::InterruptHandler<peripherals::UART0>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
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
    hal::try_println!("PA14-PA15 jumper: {}", if jumper { "yes" } else { "no (TX-only tests)" });

    let mut pass = 0u32;
    let mut fail = 0u32;
    let mut check = |name: &str, ok: bool| {
        hal::try_println!("{}: {}", name, if ok { "PASS" } else { "FAIL" });
        if ok {
            pass += 1
        } else {
            fail += 1
        }
    };

    let mut config = uart::Config::default();
    config.baudrate = 1_000_000;
    let uart = Uart::new::<1>(p.UART0, p.PA15, p.PA14, Irqs, config).unwrap();
    let (mut tx, mut rx) = uart.split();

    let mut data = [0u8; 64];
    for (i, b) in data.iter_mut().enumerate() {
        *b = (i as u8).wrapping_mul(37);
    }

    if !jumper {
        check("async write 64 bytes", tx.write(&data).await.is_ok());
        check("flush", tx.flush().await.is_ok());

        // Async RX without a wire: fill the RX FIFO through the internal
        // loopback (which masks the interrupt), then leave loopback and let
        // the interrupt-driven read collect it. 6 bytes is above the RX
        // trigger (4), so this exercises both the RX-ready interrupt and the
        // FIFO timeout that ends `read_until_idle`.
        let mcr = hal::pac::UART0.mcr();
        mcr.modify(|w| w.set_mcr_loop(true));
        tx.blocking_write(b"loop!\n").ok();
        tx.blocking_flush().ok();
        delay.delay_us(100);
        mcr.modify(|w| w.set_mcr_loop(false));
        let mut buf = [0u8; 32];
        let n = rx.read_until_idle(&mut buf).await.unwrap_or(0);
        check("async read_until_idle (FIFO preload)", n == 6 && &buf[..6] == b"loop!\n");
    } else {
        // 64 bytes each way at once: more than the FIFOs hold, so both sides
        // have to wait on interrupts.
        let mut got = [0u8; 64];
        let (w, r) = join(tx.write(&data), rx.read(&mut got)).await;
        check("async write+read 64 bytes", w.is_ok() && r.is_ok() && got == data);

        // read_until_idle: a 13-byte burst into a 32-byte buffer must return 13.
        let mut buf = [0u8; 32];
        let (_, n) = join(tx.write(b"hello, ch592!"), rx.read_until_idle(&mut buf)).await;
        let n = n.unwrap_or(0);
        check("read_until_idle 13 bytes", n == 13 && &buf[..13] == b"hello, ch592!");

        // A burst that is a multiple of the RX trigger level (8 = 2 x 4).
        let (_, n) = join(tx.write(b"12345678"), rx.read_until_idle(&mut buf)).await;
        let n = n.unwrap_or(0);
        check("read_until_idle 8 bytes", n == 8 && &buf[..8] == b"12345678");

        // Exactly fills the buffer.
        let mut small = [0u8; 5];
        let (_, n) = join(tx.write(b"abcde"), rx.read_until_idle(&mut small)).await;
        check("read_until_idle full buffer", n == Ok(5) && &small == b"abcde");

        check("flush", tx.flush().await.is_ok());
    }

    loop {
        hal::try_println!("uart_async done: {} pass, {} fail", pass, fail);
        delay.delay_ms(1000);
    }
}
