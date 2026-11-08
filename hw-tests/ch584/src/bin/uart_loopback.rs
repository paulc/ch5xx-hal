//! Blocking UART self-test over UART0's internal loopback: no wiring needed
//! (PB4 and PB7 are claimed but not driven).
//!
//! The HAL has no `set_config`, so the UART is created anew for each
//! configuration. Results are reported over SDI print.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::uart::{self, Uart};
use hwtest::check;
use {panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let mut p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
    hwtest::sdi_enable();

    for &baud in &[9600u32, 115_200, 1_000_000] {
        let mut uart =
            Uart::new_blocking::<0>(p.UART0.reborrow(), p.PB4.reborrow(), p.PB7.reborrow(), uart::Config::new(baud))
                .unwrap();
        uart.set_internal_loopback(true);

        // Up to a FIFO's worth at a time: nothing drains RX while we write.
        let mut ok = true;
        for round in 0..32u8 {
            let tx = [round, !round, 0x55, 0xaa, round.wrapping_mul(7), 0, 0xff, b'\n'];
            let mut rx = [0u8; 8];
            uart.blocking_write(&tx).unwrap();
            if uart.blocking_read(&mut rx).is_err() || rx != tx {
                ok = false;
                hwtest::println!("  {} baud round {}: tx {:02x?} rx {:02x?}", baud, round, tx, rx);
                break;
            }
        }
        uart.blocking_flush().unwrap();
        check!(ok, "uart0 loopback {} baud", baud);
    }

    // 7E2 framing.
    let mut config = uart::Config::new(115_200);
    config.data_bits = uart::DataBits::DataBits7;
    config.parity = uart::Parity::Even;
    config.stop_bits = uart::StopBits::Stop2;
    let mut uart = Uart::new_blocking::<0>(p.UART0, p.PB4, p.PB7, config).unwrap();
    uart.set_internal_loopback(true);
    let tx = *b"7E2 ok!";
    let mut rx = [0u8; 7];
    uart.blocking_write(&tx).unwrap();
    let ok = uart.blocking_read(&mut rx).is_ok() && rx == tx;
    check!(ok, "uart0 loopback 7E2");

    // The HAL refuses a rate more than 2% off: 115200 from the 5.33 MHz reset
    // clock would be 111.1 kbaud. At 62.4 MHz, 3 Mbaud is 2.6 Mbaud.
    check!(
        matches!(
            Uart::new_blocking::<0>(unsafe { hal::peripherals::UART1::steal() }, unsafe { hal::peripherals::PA8::steal() }, unsafe { hal::peripherals::PA9::steal() }, uart::Config::new(3_000_000)),
            Err(uart::ConfigError::Baudrate)
        ),
        "3 Mbaud refused (ConfigError::Baudrate)"
    );

    hwtest::done("uart_loopback")
}
