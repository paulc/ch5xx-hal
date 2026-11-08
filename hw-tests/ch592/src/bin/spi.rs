//! SPI0 master test on SCK = PA13, MOSI = PA14, MISO = PA15.
//!
//! Without wiring, MISO is flipped between the internal pull-up and pull-down
//! to check every read path (blocking, interrupt-driven, DMA, full-duplex)
//! sees `0xFF`/`0x00`, and writes are timed to check the SCK divider. With a
//! jumper from PA14 to PA15 (MOSI to MISO) it also checks that transfers read
//! back what they sent. Results are reported over SDI print.
//!
//! In this HAL `write`/`read` are interrupt-driven through the FIFO and DMA
//! is the separate `write_dma`/`read_dma`, which take `&'static mut` RAM
//! buffers (so DMA straight from flash is not possible).

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use hal::delay::Delay;
use hal::dma::StaticByteBuffer;
use hal::gpio::{Input, Level, Output, Pull, Speed};
use hal::spi::{self, Spi};
use hal::time::Hertz;
use hal::{bind_interrupts, pac, peripherals};
use hwtest::check;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

static TX1000: StaticByteBuffer<1000> = StaticByteBuffer::new([0x5a; 1000]);
static TX5000: StaticByteBuffer<5000> = StaticByteBuffer::new([0xa5; 5000]);
static RX40: StaticByteBuffer<40> = StaticByteBuffer::new([0x33; 40]);

fn miso_pull(up: bool) {
    let g = pac::GPIO;
    g.pa_pu().modify(|w| w.set_pa_pu(15, up));
    g.pa_pd_drv().modify(|w| w.set_pa_pd_drv(15, !up));
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();
    let mut delay = Delay;

    let jumper = {
        let mut out = Output::new(p.PA14.reborrow(), Level::Low, Speed::Low);
        let inp = Input::new(p.PA15.reborrow(), Pull::Up);
        delay.delay_us(10);
        let low = inp.is_low();
        out.set_high();
        delay.delay_us(10);
        low && inp.is_high()
    };
    hwtest::println!("PA14-PA15 jumper: {}", if jumper { "yes" } else { "no (pull tests)" });

    let mut spi = Spi::new_async::<0>(p.SPI0, Irqs, p.PA13, p.PA14, p.PA15, spi::Config::new(Hertz::mhz(1))).unwrap();

    // Timing: 1000 bytes at 1 MHz is 8 ms.
    let ram = [0x5au8; 1000];
    let t = Instant::now();
    spi.blocking_write(&ram).unwrap();
    let us = t.elapsed().as_micros();
    check!((7_900..8_600).contains(&us), "blocking write 1000 B: {} us", us);

    let t = Instant::now();
    spi.write(&ram).await.unwrap();
    let us = t.elapsed().as_micros();
    check!((7_900..8_600).contains(&us), "async write 1000 B: {} us", us);

    let t = Instant::now();
    let r = spi.write_dma(TX1000.take().unwrap()).ok().unwrap().wait(Irqs).await;
    let us = t.elapsed().as_micros();
    check!(r.is_ok() && (7_900..8_600).contains(&us), "DMA write 1000 B: {} us", us);
    hwtest::skip("DMA write from flash", "write_dma takes a &'static mut RAM buffer");

    // Past `TOTAL_CNT`'s 12 bits: two chunks.
    let big = [0xa5u8; 5000];
    let t = Instant::now();
    spi.blocking_write(&big).unwrap();
    let us = t.elapsed().as_micros();
    check!((39_500..42_500).contains(&us), "blocking write 5000 B: {} us", us);

    let t = Instant::now();
    let r = spi.write_dma(TX5000.take().unwrap()).ok().unwrap().wait(Irqs).await;
    let us = t.elapsed().as_micros();
    check!(r.is_ok() && (39_500..42_500).contains(&us), "DMA write 5000 B: {} us", us);

    let config = spi::Config::new(Hertz::mhz(15));
    let rate = spi.set_config(config);
    check!(rate.is_ok(), "set_config 15 MHz");
    let mut c1 = config;
    c1.mode = embedded_hal::spi::MODE_1;
    check!(matches!(spi.set_config(c1), Err(spi::ConfigError::Mode)), "MODE_1 rejected");
    check!(
        matches!(spi.set_config(spi::Config::new(Hertz::mhz(40))), Err(spi::ConfigError::Frequency)),
        "40 MHz (above HCLK / 2) rejected"
    );
    spi.set_config(config).unwrap();

    if !jumper {
        let mut rx40: Option<&'static mut [u8]> = Some(RX40.take().unwrap());
        for up in [true, false] {
            miso_pull(up);
            Timer::after_micros(100).await;
            let want = if up { 0xff } else { 0x00 };
            let tag = if up { "pull-up" } else { "pull-down" };

            let mut buf = [0x33u8; 40];
            spi.blocking_read(&mut buf).unwrap();
            check!(buf.iter().all(|&b| b == want), "blocking read ({})", tag);

            let mut buf = [0x33u8; 40];
            spi.read(&mut buf).await.unwrap();
            check!(buf.iter().all(|&b| b == want), "async read ({})", tag);

            let b = rx40.take().unwrap();
            b.fill(0x33);
            let r = spi.read_dma(b).ok().unwrap().wait(Irqs).await;
            let b = match r {
                Ok(b) => b,
                Err((_, b)) => b,
            };
            check!(b.iter().all(|&x| x == want), "DMA read ({})", tag);
            rx40 = Some(b);

            let mut buf = [0x33u8; 40];
            spi.blocking_transfer_in_place(&mut buf).unwrap();
            check!(buf.iter().all(|&b| b == want), "blocking transfer ({})", tag);

            let mut buf = [0x33u8; 40];
            spi.transfer_in_place(&mut buf).await.unwrap();
            check!(buf.iter().all(|&b| b == want), "async transfer ({})", tag);
        }
    } else {
        let mut buf = [0u8; 64];
        for (i, b) in buf.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(29) ^ 0xa5;
        }
        let sent = buf;
        spi.blocking_transfer_in_place(&mut buf).unwrap();
        check!(buf == sent, "loopback transfer_in_place");
        let mut rx = [0u8; 64];
        spi.blocking_transfer(&mut rx, &sent).unwrap();
        check!(rx == sent, "loopback transfer");
        let mut rx = [0u8; 64];
        spi.transfer(&mut rx, &sent).await.unwrap();
        check!(rx == sent, "loopback async transfer");

        let mut config = config;
        config.mode = embedded_hal::spi::MODE_3;
        config.bit_order = spi::BitOrder::LsbFirst;
        spi.set_config(config).unwrap();
        let mut buf = sent;
        spi.transfer_in_place(&mut buf).await.unwrap();
        check!(buf == sent, "loopback MODE_3 LSB first");
    }

    hwtest::done("spi")
}
