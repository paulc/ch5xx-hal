//! USB CDC-ACM echo on the board's USB port (D+ = PB11, D- = PB10).
//! Enumeration progress and echoed data are reported over SDI print.
//!
//! On the host the device shows up as a serial port (VID:PID c0de:cafe);
//! anything written to it comes back. On macOS open `/dev/cu.usbmodem*`:
//! the `tty.` node waits for carrier detect, which this class never
//! reports, so nothing is sent. With no host on the USB port the
//! device just waits for a bus reset.
//!
//! hw-test: manual. It needs a host on the USB port.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Handler};
use hal::{bind_interrupts, peripherals, usb};

bind_interrupts!(struct Irqs {
    USB => usb::InterruptHandler<peripherals::USBFS>;
});

/// Reports the device state changes.
struct Events;

impl Handler for Events {
    fn reset(&mut self) {
        hwtest::println!("[{} ms] bus reset", Instant::now().as_millis());
    }
    fn addressed(&mut self, addr: u8) {
        hwtest::println!("[{} ms] address {}", Instant::now().as_millis(), addr);
    }
    fn configured(&mut self, configured: bool) {
        hwtest::println!("[{} ms] configured: {}", Instant::now().as_millis(), configured);
    }
    fn suspended(&mut self, suspended: bool) {
        hwtest::println!("[{} ms] suspended: {}", Instant::now().as_millis(), suspended);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    // The USB PHY runs from the PLL.
    let p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
    hwtest::sdi_enable();
    hwtest::println!("usb_cdc: starting");

    let driver = usb::Driver::new(p.USBFS, Irqs);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("ch5xx-hal hw-tests");
    config.product = Some("CH584 CDC-ACM echo");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();
    let mut events = Events;

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    builder.handler(&mut events);
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let echo = async {
        loop {
            class.wait_connection().await;
            hwtest::println!("[{} ms] port opened", Instant::now().as_millis());
            let _ = echo(&mut class).await;
            hwtest::println!("[{} ms] port closed", Instant::now().as_millis());
        }
    };
    join(usb.run(), echo).await;

    loop {
        Timer::after_secs(1).await;
    }
}

async fn echo<'d, D: embassy_usb::driver::Driver<'d>>(class: &mut CdcAcmClass<'d, D>) -> Result<(), EndpointError> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        match core::str::from_utf8(data) {
            Ok(s) => hwtest::println!("rx {}: {:?}", n, s),
            Err(_) => hwtest::println!("rx {}: {:02x?}", n, data),
        }
        class.write_packet(data).await?;
    }
}
