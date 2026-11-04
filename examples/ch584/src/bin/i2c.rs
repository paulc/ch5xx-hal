//! Hardware I2C test on SCL = PB13, SDA = PB12, needing no device: results
//! are reported over SDI print.
//!
//! The pins' own pull-ups make the bus. With nobody to acknowledge, every
//! transfer ends in a NACK; while one runs the test samples both pins, which
//! checks that the peripheral drives them. A stuck device is played by
//! swapping a line's pull-up for its pull-down: with the peripheral on, the
//! GPIO output does not reach the pins.
//!
//! Reading and acknowledged writes need a device.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::yield_now;
use embassy_time::Instant;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::i2c::{self, Error, I2c};
use hal::time::Hertz;
use hal::{bind_interrupts, pac, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C0 => i2c::InterruptHandler<peripherals::I2C0>;
});

const SCL: u32 = 13;
const SDA: u32 = 12;

struct Checks {
    pass: u32,
    fail: u32,
}

impl Checks {
    fn check(&mut self, name: &str, ok: bool) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        hal::println!("{} {}", if ok { "PASS" } else { "FAIL" }, name);
    }
}

fn level(n: u32) -> bool {
    pac::GPIO.pb_pin().read().0 & (1 << n) != 0
}

/// Pull PBn down instead of up, or back.
fn hold(n: u32, low: bool) {
    let g = pac::GPIO;
    qingke::riscv::interrupt::free(|| {
        g.pb_pu().modify(|w| if low { w.0 &= !(1 << n) } else { w.0 |= 1 << n });
        g.pb_pd_drv().modify(|w| if low { w.0 |= 1 << n } else { w.0 &= !(1 << n) });
    });
    // Let the pull settle.
    Delay.delay_us(20);
}

/// Count the samples of SCL and SDA found low until `done`, yielding between
/// samples so the transfer moves on. The driver spins between some steps, so
/// samples are sparse: a low at all shows the peripheral drives the pin.
async fn lows(done: &core::cell::Cell<bool>) -> (u32, u32) {
    let (mut scl, mut sda) = (0, 0);
    while !done.get() {
        scl += !level(SCL) as u32;
        sda += !level(SDA) as u32;
        yield_now().await;
    }
    (scl, sda)
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut c = Checks { pass: 0, fail: 0 };

    let mut config = i2c::Config::default();
    config.timeout = core::time::Duration::from_millis(10);

    for (khz, name) in [(100, "100 kHz"), (400, "400 kHz")] {
        let mut i2c = I2c::new(
            p.I2C0.reborrow(),
            p.PB13.reborrow(),
            p.PB12.reborrow(),
            Irqs,
            Hertz::khz(khz),
            config,
        );
        hal::println!("{}:", name);
        c.check("  idle bus is high", level(SCL) && level(SDA));

        let done = core::cell::Cell::new(false);
        let t0 = Instant::now();
        let (r, (scl, sda)) = join(
            async {
                let r = i2c.write(0x55, &[0xaa]).await;
                done.set(true);
                r
            },
            lows(&done),
        )
        .await;
        let us = t0.elapsed().as_micros();
        hal::println!("  write(0x55, [0xaa]) = {:?} in {} us, low samples: SCL {} SDA {}", r, us, scl, sda);
        c.check("  async write, no device: Nack", r == Err(Error::Nack));
        if khz == 100 {
            c.check("  SCL driven low", scl > 0);
            c.check("  SDA driven low", sda > 0);
        }

        let r = i2c.blocking_write(0x55, &[0xaa]);
        hal::println!("  blocking write: {:?}", r);
        c.check("  blocking write: Nack", r == Err(Error::Nack));
        let mut buf = [0u8; 2];
        let r = i2c.blocking_read(0x55, &mut buf);
        hal::println!("  blocking read: {:?}", r);
        c.check("  blocking read: Nack", r == Err(Error::Nack));
        let r = i2c.read(0x55, &mut buf).await;
        hal::println!("  async read: {:?}", r);
        c.check("  async read: Nack", r == Err(Error::Nack));
        let r = i2c.write_read(0x55, &[1], &mut buf).await;
        hal::println!("  async write_read: {:?}", r);
        c.check("  async write_read: Nack", r == Err(Error::Nack));
        c.check("  bus released after a Nack", level(SCL) && level(SDA));
    }

    let mut i2c = I2c::new_blocking(p.I2C0.reborrow(), p.PB13.reborrow(), p.PB12.reborrow(), Hertz::khz(100), config);

    // SDA held low at the start: the peripheral sees the bus busy.
    hold(SDA, true);
    c.check("SDA pulled down reads low", !level(SDA));
    let t0 = Instant::now();
    let r = i2c.blocking_write(0x55, &[]);
    let us = t0.elapsed().as_micros();
    hold(SDA, false);
    hal::println!("SDA held low: {:?} in {} us", r, us);
    c.check("SDA held low: an error other than Nack", r.is_err() && r != Err(Error::Nack));
    let r = i2c.blocking_write(0x55, &[]);
    hal::println!("  then: {:?}", r);
    c.check("  and the bus works again (Nack)", r == Err(Error::Nack));

    // SCL held low: the start never completes.
    hold(SCL, true);
    c.check("SCL pulled down reads low", !level(SCL));
    let t0 = Instant::now();
    let r = i2c.blocking_write(0x55, &[]);
    let us = t0.elapsed().as_micros();
    hold(SCL, false);
    hal::println!("SCL held low: {:?} in {} us", r, us);
    c.check("SCL held low: Timeout", r == Err(Error::Timeout));
    c.check("  after about 10 ms", (9_000..12_000).contains(&us));
    let r = i2c.blocking_write(0x55, &[]);
    hal::println!("  then: {:?}", r);
    c.check("  and the bus works again (Nack)", r == Err(Error::Nack));

    drop(i2c);
    hal::println!("i2c: {} passed, {} failed", c.pass, c.fail);
    loop {
        embassy_time::Timer::after_secs(1).await;
    }
}
