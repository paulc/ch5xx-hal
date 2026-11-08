//! Hardware I2C test on SCL = PB13, SDA = PB12, needing no device: results
//! are reported over SDI print.
//!
//! The pins' own pull-ups make the bus. With nobody to acknowledge, every
//! transfer ends in a NACK; while one runs the test samples both pins, which
//! checks that the peripheral drives them. A stuck device is played by
//! swapping a line's pull-up for its pull-down: with the peripheral on, the
//! GPIO output does not reach the pins.
//!
//! This HAL's I2C needs `Fsys` of 2..=36 MHz (`CTRL2.FREQ`), so the test runs
//! at 32 MHz, and its timeout is a fixed 1 ms. Reading and acknowledged writes
//! need a device.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::yield_now;
use embassy_time::Instant;
use embedded_hal::i2c::NoAcknowledgeSource;
use hal::delay::Delay;
use hal::i2c::{self, Error, I2c};
use hal::time::Hertz;
use hal::{bind_interrupts, pac, peripherals};
use hwtest::check;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C0 => i2c::InterruptHandler<peripherals::I2C0>;
});

const SCL: u32 = 13;
const SDA: u32 = 12;

fn level(n: u32) -> bool {
    pac::GPIO.pb_pin().read().0 & (1 << n) != 0
}

fn nack<T>(r: &Result<T, Error>) -> bool {
    matches!(r, Err(Error::NoAcknowledge(_)))
}

/// Pull PBn down instead of up, or back.
fn hold(n: u32, low: bool) {
    let g = pac::GPIO;
    critical_section::with(|_| {
        g.pb_pu().modify(|w| if low { w.0 &= !(1 << n) } else { w.0 |= 1 << n });
        g.pb_pd_drv()
            .modify(|w| if low { w.0 |= 1 << n } else { w.0 &= !(1 << n) });
    });
    // Let the pull settle.
    Delay.delay_us(20);
}

/// Count the samples of SCL and SDA found low until `done`, yielding between
/// samples so the transfer moves on.
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
    let mut p = hal::init(hal::rcc::Config::HSE_32MHZ.into());
    hwtest::sdi_enable();

    for khz in [100, 400] {
        let mut i2c = I2c::new::<0>(
            p.I2C0.reborrow(),
            Irqs,
            p.PB13.reborrow(),
            p.PB12.reborrow(),
            i2c::Config::new(Hertz::khz(khz)),
        )
        .unwrap();
        hwtest::println!("{} kHz:", khz);
        check!(level(SCL) && level(SDA), "  idle bus is high");

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
        hwtest::println!(
            "  write(0x55, [0xaa]) = {:?} in {} us, low samples: SCL {} SDA {}",
            r,
            us,
            scl,
            sda
        );
        check!(
            r == Err(Error::NoAcknowledge(NoAcknowledgeSource::Address)),
            "  async write, no device: address NACK"
        );
        if khz == 100 {
            check!(scl > 0, "  SCL driven low");
            check!(sda > 0, "  SDA driven low");
        }

        let r = i2c.blocking_write(0x55, &[0xaa]);
        check!(nack(&r), "  blocking write: {:?}", r);
        let mut buf = [0u8; 2];
        let r = i2c.blocking_read(0x55, &mut buf);
        check!(nack(&r), "  blocking read: {:?}", r);
        let r = i2c.read(0x55, &mut buf).await;
        check!(nack(&r), "  async read: {:?}", r);
        let r = i2c.write_read(0x55, &[1], &mut buf).await;
        check!(nack(&r), "  async write_read: {:?}", r);
        check!(level(SCL) && level(SDA), "  bus released after a NACK");

        let r = i2c.blocking_write(0x80, &[]);
        check!(r == Err(Error::Address), "  10-bit address refused: {:?}", r);
    }

    let mut i2c = I2c::new_blocking::<0>(
        p.I2C0.reborrow(),
        p.PB13.reborrow(),
        p.PB12.reborrow(),
        i2c::Config::default(),
    )
    .unwrap();

    // SDA held low at the start: the peripheral sees the bus busy.
    hold(SDA, true);
    check!(!level(SDA), "SDA pulled down reads low");
    let t0 = Instant::now();
    let r = i2c.blocking_write(0x55, &[]);
    let us = t0.elapsed().as_micros();
    hold(SDA, false);
    check!(
        r.is_err() && !nack(&r),
        "SDA held low: {:?} in {} us (an error other than NACK)",
        r,
        us
    );
    let r = i2c.blocking_write(0x55, &[]);
    check!(nack(&r), "  and the bus works again: {:?}", r);

    // SCL held low: the start never completes.
    hold(SCL, true);
    check!(!level(SCL), "SCL pulled down reads low");
    let t0 = Instant::now();
    let r = i2c.blocking_write(0x55, &[]);
    let us = t0.elapsed().as_micros();
    hold(SCL, false);
    check!(r == Err(Error::Timeout), "SCL held low: {:?} in {} us", r, us);
    check!((900..3_000).contains(&us), "  after about 1 ms: {} us", us);
    let r = i2c.blocking_write(0x55, &[]);
    check!(nack(&r), "  and the bus works again: {:?}", r);
    drop(i2c);

    // Above the 36 MHz FREQ limit the driver refuses.
    unsafe { hal::rcc::set_config(hal::rcc::Config::HSE_PLL_62_4MHZ) };
    let r = I2c::new_blocking::<0>(
        p.I2C0.reborrow(),
        p.PB13.reborrow(),
        p.PB12.reborrow(),
        i2c::Config::default(),
    );
    check!(
        matches!(r, Err(i2c::ConfigError::Clock)),
        "Fsys 62.4 MHz refused (ConfigError::Clock)"
    );

    hwtest::done("i2c")
}
