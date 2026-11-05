//! Software I2C with real devices: an AHT20 (0x38) and a BMP280 (0x77), as on
//! the common AHT20+BMP280 breakout. Results are reported over SDI print.
//!
//! **Needs wiring**: SCL to PB13, SDA to PB12, plus 3V3 and GND. The breakout
//! has its own pull-ups; without them set `internal_pullups` and lower the
//! clock. Any other pins work too: change them below.
//!
//! Every 2 s: a bus scan, the BMP280's chip id and temperature (async), and
//! the AHT20's temperature and humidity (blocking, through `embedded-hal`).

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_hal::i2c::I2c;
use hal::debug::SDIPrint;
use hal::soft_i2c::{Config, Error, SoftI2c};
use hal::{bind_interrupts, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    TMR3 => hal::soft_i2c::InterruptHandler<peripherals::TMR3>;
});

const AHT20: u8 = 0x38;
const BMP280: u8 = 0x77;

/// BMP280 temperature in hundredths of a degC, from the datasheet's integer
/// compensation, given the 6 calibration bytes `dig_T1..dig_T3` and the
/// 20-bit raw reading.
fn bmp280_centi_celsius(cal: &[u8; 6], raw: i32) -> i32 {
    let t1 = u16::from_le_bytes([cal[0], cal[1]]) as i32;
    let t2 = i16::from_le_bytes([cal[2], cal[3]]) as i32;
    let t3 = i16::from_le_bytes([cal[4], cal[5]]) as i32;
    let var1 = (((raw >> 3) - (t1 << 1)) * t2) >> 11;
    let var2 = (((((raw >> 4) - t1) * ((raw >> 4) - t1)) >> 12) * t3) >> 14;
    ((var1 + var2) * 5 + 128) >> 8
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::HSE_PLL_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();

    let mut i2c = SoftI2c::new(p.TMR3, p.PB13, p.PB12, Irqs, Config::default());

    loop {
        // Scan.
        let mut found = [0u8; 8];
        let mut n = 0;
        for addr in 0x08..0x78u8 {
            match i2c.write(addr, &[]).await {
                Ok(()) if n < found.len() => {
                    found[n] = addr;
                    n += 1;
                }
                Ok(()) | Err(Error::Nack) => {}
                Err(e) => hal::println!("scan {:#04x}: {:?}", addr, e),
            }
        }
        hal::println!("scan: {:x?}", &found[..n]);

        // BMP280, async: chip id, then one forced measurement.
        let mut id = [0u8];
        match i2c.write_read(BMP280, &[0xd0], &mut id).await {
            Ok(()) => {
                let mut cal = [0u8; 6];
                let mut raw = [0u8; 3];
                let r = async {
                    i2c.write_read(BMP280, &[0x88], &mut cal).await?;
                    // ctrl_meas: temperature x1, pressure skipped, forced mode.
                    i2c.write(BMP280, &[0xf4, 0x21]).await?;
                    Timer::after_millis(10).await;
                    i2c.write_read(BMP280, &[0xfa], &mut raw).await
                }
                .await;
                let adc_t = (raw[0] as i32) << 12 | (raw[1] as i32) << 4 | (raw[2] as i32) >> 4;
                let t = bmp280_centi_celsius(&cal, adc_t);
                hal::println!(
                    "BMP280: id {:#04x} (0x58 expected), {:?}, {}.{:02} degC",
                    id[0],
                    r,
                    t / 100,
                    t % 100
                );
            }
            Err(e) => hal::println!("BMP280: {:?}", e),
        }

        // AHT20, blocking through embedded-hal: calibrate if needed, measure.
        let mut status = [0u8];
        let r = (|| -> Result<(u32, u32), Error> {
            I2c::read(&mut i2c, AHT20, &mut status)?;
            if status[0] & 0x08 == 0 {
                I2c::write(&mut i2c, AHT20, &[0xbe, 0x08, 0x00])?;
                hal::delay::Delay.delay_ms(10);
            }
            I2c::write(&mut i2c, AHT20, &[0xac, 0x33, 0x00])?;
            hal::delay::Delay.delay_ms(80);
            let mut d = [0u8; 7];
            I2c::read(&mut i2c, AHT20, &mut d)?;
            let hum = (d[1] as u32) << 12 | (d[2] as u32) << 4 | (d[3] as u32) >> 4;
            let tmp = (d[3] as u32 & 0x0f) << 16 | (d[4] as u32) << 8 | d[5] as u32;
            Ok((hum, tmp))
        })();
        match r {
            Ok((hum, tmp)) => {
                let rh = hum as u64 * 1000 / (1 << 20);
                let t = (tmp as u64 * 2000 / (1 << 20)) as i32 - 500;
                hal::println!(
                    "AHT20: status {:#04x}, {}.{} %RH, {}.{} degC",
                    status[0],
                    rh / 10,
                    rh % 10,
                    t / 10,
                    (t % 10).abs()
                );
            }
            Err(e) => hal::println!("AHT20: {:?}", e),
        }

        Timer::after_secs(2).await;
    }
}
