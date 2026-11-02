//! I2C bus scan on SCL = PB15, SDA = PB14, reporting over UART1 TX (PA9,
//! 115200 8N1).
//!
//! **Not run by the test suite**: PB14/PB15 are the debug link's TIO/TCK, so
//! this takes the debugger (and SDI print) off the chip. The first 5 s keep
//! the pins alone, so a debugger can still erase the chip after a reset.
//!
//! Wire an I2C device with pull-ups to PB14/PB15 and a USB-serial adapter's
//! RX to PA9.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::i2c::{self, I2c};
use hal::time::Hertz;
use hal::uart::{self, UartTx};
use panic_halt as _;
use qingke_rt::entry;

struct W<'a, 'd>(&'a mut UartTx<'d, hal::peripherals::UART1, hal::mode::Blocking>);

impl core::fmt::Write for W<'_, '_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0.blocking_write(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

#[entry]
fn main() -> ! {
    let p = hal::init(hal::Config::default());
    let mut delay = Delay;
    let mut tx = UartTx::new_blocking::<0>(p.UART1, p.PA9, uart::Config::default()).unwrap();
    let mut w = W(&mut tx);

    let _ = writeln!(w, "i2c_scan: starting in 5 s");
    delay.delay_ms(5000);

    let mut bus = I2c::new_blocking::<0>(p.I2C0, p.PB15, p.PB14, Hertz::khz(100), i2c::Config::default());
    loop {
        let _ = write!(w, "scan:");
        let mut found = 0;
        for addr in 0x08..0x78u8 {
            match bus.blocking_write(addr, &[]) {
                Ok(()) => {
                    let _ = write!(w, " {:#04x}", addr);
                    found += 1;
                }
                Err(i2c::Error::Nack) => {}
                Err(e) => {
                    let _ = write!(w, " {:#04x}:{:?}", addr, e);
                }
            }
        }
        let _ = writeln!(w, " ({} found)", found);
        delay.delay_ms(2000);
    }
}
