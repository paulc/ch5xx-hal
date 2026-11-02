//! Blocking UART self-test over UART0's internal loopback: no wiring needed.
//!
//! Results are reported over SDI print.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::uart::{self, Uart};
use {panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut delay = Delay;

    let mut pass = 0u32;
    let mut fail = 0u32;
    let mut uart = Uart::new_blocking_loopback(p.UART0, uart::Config::default()).unwrap();

    for &baud in &[9600u32, 115_200, 1_000_000] {
        let mut config = uart::Config::default();
        config.baudrate = baud;
        uart.set_config(&config).unwrap();

        // Up to a FIFO's worth at a time: nothing drains RX while we write.
        let mut ok = true;
        for round in 0..32u8 {
            let tx = [round, !round, 0x55, 0xaa, round.wrapping_mul(7), 0, 0xff, b'\n'];
            let mut rx = [0u8; 8];
            uart.blocking_write(&tx).unwrap();
            if uart.blocking_read(&mut rx).is_err() || rx != tx {
                ok = false;
                hal::try_println!("  {} baud round {}: tx {:02x?} rx {:02x?}", baud, round, tx, rx);
                break;
            }
        }
        uart.blocking_flush().unwrap();
        hal::try_println!("uart0 loopback {} baud: {}", baud, if ok { "PASS" } else { "FAIL" });
        if ok {
            pass += 1
        } else {
            fail += 1
        }
    }

    // 7E2 framing.
    let mut config = uart::Config::default();
    config.data_bits = uart::DataBits::DataBits7;
    config.parity = uart::Parity::ParityEven;
    config.stop_bits = uart::StopBits::STOP2;
    uart.set_config(&config).unwrap();
    let tx = *b"7E2 ok!";
    let mut rx = [0u8; 7];
    uart.blocking_write(&tx).unwrap();
    let ok = uart.blocking_read(&mut rx).is_ok() && rx == tx;
    hal::try_println!("uart0 loopback 7E2: {}", if ok { "PASS" } else { "FAIL" });
    if ok {
        pass += 1
    } else {
        fail += 1
    }

    loop {
        hal::try_println!("uart_loopback done: {} pass, {} fail", pass, fail);
        delay.delay_ms(1000);
    }
}
