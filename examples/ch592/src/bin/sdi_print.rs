//! Print over the 2-wire debug link (WCH "SDI print"), no UART needed.
//!
//! CH32 parts print through the debug module's `DATA0`/`DATA1` registers, which
//! WCH-LinkE drains to its virtual serial port. The CH592's `hartinfo` reports
//! those registers memory-mapped at `0xE000_0380` (the same place as on
//! CH32V20x/V30x). WCH does not document SDI print for CH59x, but it works
//! (WeAct CH592F board, WCH-LinkE firmware v3.2).
//!
//! wlink 0.1.3 refuses to arm SDI print for `CH59X`: add `RiscvChip::CH59X` to
//! `support_sdi_print()` in wlink's `src/lib.rs`, then
//!
//! ```console
//! wlink --chip CH59X --speed low flash --enable-sdi-print --watch-serial \
//!     target/riscv32imac-unknown-none-elf/release/sdi_print
//! ```
//!
//! Writes are lossy: with nobody draining `DATA0`, a message is dropped after a
//! short wait instead of hanging, and the LED (PA8 on the WeAct board) keeps
//! blinking either way.

#![no_std]
#![no_main]

use core::fmt::Write;

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};
use panic_halt as _;
use qingke::dm::{DATA0, DATA1};
use qingke_rt::entry;

/// Polls of `DATA0` to wait for the probe before dropping a chunk.
const SPINS: u32 = 20_000;

struct SdiPrint;

impl SdiPrint {
    fn enable() {
        unsafe { core::ptr::write_volatile(DATA0 as *mut u32, 0) };
    }

    fn busy() -> bool {
        unsafe { core::ptr::read_volatile(DATA0 as *const u32) != 0 }
    }
}

impl Write for SdiPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Same framing as ch32-hal's `SDIPrint`: up to 7 bytes per transfer,
        // byte 0 of DATA0 is the length, DATA0 is written last.
        for chunk in s.as_bytes().chunks(7) {
            let mut data = [0u8; 8];
            data[0] = chunk.len() as u8;
            data[1..chunk.len() + 1].copy_from_slice(chunk);

            let mut waited = 0;
            while SdiPrint::busy() {
                waited += 1;
                if waited > SPINS {
                    return Ok(());
                }
            }

            unsafe {
                core::ptr::write_volatile(DATA1 as *mut u32, u32::from_le_bytes(data[4..].try_into().unwrap()));
                core::ptr::write_volatile(DATA0 as *mut u32, u32::from_le_bytes(data[..4].try_into().unwrap()));
            }
        }
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    SdiPrint::enable();

    let mut led = Output::new(p.PA8, Level::High, Speed::Low);
    let mut delay = Delay;
    let mut n: u32 = 0;

    loop {
        let _ = writeln!(
            SdiPrint,
            "ch592 sdi print {} (sysclk {} Hz)",
            n,
            hal::rcc::clocks().sysclk.0
        );
        led.toggle();
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
