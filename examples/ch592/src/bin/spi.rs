//! SPI0 master test on SCK = PA13, MOSI = PA14, MISO = PA15.
//!
//! Without wiring, MISO is flipped between the internal pull-up and pull-down
//! to check every read path (blocking, DMA, full-duplex) sees `0xFF`/`0x00`,
//! and writes are timed to check the SCK divider. With a jumper from PA14 to
//! PA15 (MOSI to MISO) it also checks that transfers read back what they
//! sent. Results are reported over SDI print.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::{Input, Level, Output, Pull, Speed};
use hal::spi::{self, Spi};
use hal::time::Hertz;
use hal::{bind_interrupts, pac, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    SPI0 => spi::InterruptHandler<peripherals::SPI0>;
});

static FLASH_DATA: [u8; 300] = {
    let mut d = [0u8; 300];
    let mut i = 0;
    while i < 300 {
        d[i] = (i * 7) as u8;
        i += 1;
    }
    d
};

fn miso_pull(up: bool) {
    let g = pac::GPIO;
    g.pa_pu().modify(|w| w.set_pa_pu(15, up));
    g.pa_pd_drv().modify(|w| w.set_pa_pd_drv(15, !up));
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
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
    hal::try_println!("PA14-PA15 jumper: {}", if jumper { "yes" } else { "no (pull tests)" });

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

    let mut config = spi::Config::default();
    config.frequency = Hertz::mhz(1);
    let mut spi = Spi::new(p.SPI0, p.PA13, p.PA14, p.PA15, Irqs, config);
    hal::try_println!("SCK {} Hz", spi.get_current_config().frequency.0);

    // Timing: 1000 bytes at 1 MHz is 8 ms.
    let ram = [0x5au8; 1000];
    let t = Instant::now();
    spi.blocking_write(&ram).unwrap();
    let us = t.elapsed().as_micros();
    hal::try_println!("  blocking_write 1000 B: {} us", us);
    check("blocking write timing", (7_900..8_600).contains(&us));
    let t = Instant::now();
    spi.write(&ram).await.unwrap();
    let us = t.elapsed().as_micros();
    hal::try_println!("  DMA write 1000 B: {} us", us);
    check("DMA write timing", (7_900..8_600).contains(&us));
    let t = Instant::now();
    spi.write(&FLASH_DATA).await.unwrap();
    let us = t.elapsed().as_micros();
    hal::try_println!("  DMA write 300 B from flash: {} us", us);
    check("DMA write from flash (bounce)", (2_350..2_900).contains(&us));

    let mut config = spi::Config::default();
    config.frequency = Hertz::mhz(15);
    spi.set_config(&config).unwrap();
    check("MODE_1 rejected", {
        let mut c = config;
        c.mode = spi::MODE_1;
        spi.set_config(&c).is_err()
    });

    if !jumper {
        for up in [true, false] {
            miso_pull(up);
            Timer::after_micros(100).await;
            let want = if up { 0xff } else { 0x00 };
            let mut buf = [0x33u8; 40];
            spi.blocking_read(&mut buf).unwrap();
            check(if up { "blocking read (pull-up)" } else { "blocking read (pull-down)" }, buf.iter().all(|&b| b == want));
            let mut buf = [0x33u8; 40];
            spi.read(&mut buf).await.unwrap();
            check(if up { "DMA read (pull-up)" } else { "DMA read (pull-down)" }, buf.iter().all(|&b| b == want));
            let mut buf = [0x33u8; 40];
            spi.blocking_transfer_in_place(&mut buf).unwrap();
            check(if up { "transfer (pull-up)" } else { "transfer (pull-down)" }, buf.iter().all(|&b| b == want));
        }
    } else {
        let mut buf = [0u8; 64];
        for (i, b) in buf.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(29) ^ 0xa5;
        }
        let sent = buf;
        spi.blocking_transfer_in_place(&mut buf).unwrap();
        check("loopback transfer_in_place", buf == sent);
        let mut rx = [0u8; 64];
        spi.blocking_transfer(&mut rx, &sent).unwrap();
        check("loopback transfer", rx == sent);
        let mut config = config;
        config.mode = spi::MODE_3;
        config.bit_order = spi::BitOrder::LsbFirst;
        spi.set_config(&config).unwrap();
        let mut buf = sent;
        spi.transfer_in_place(&mut buf).await.unwrap();
        check("loopback MODE_3 LSB first", buf == sent);
    }

    loop {
        hal::try_println!("spi done: {} pass, {} fail", pass, fail);
        delay.delay_ms(1000);
    }
}
