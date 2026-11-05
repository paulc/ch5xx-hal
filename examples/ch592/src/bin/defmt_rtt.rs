//! Log to the host with `defmt` over SEGGER RTT.
//!
//! RTT is a RAM-based ring buffer that the debugger reads out of memory, so it
//! needs no UART pin.
//!
//! NOTE: this needs a probe-rs that can attach to CH592, and probe-rs has no
//! CH59x target yet (it also always attaches with the CH32V103 family code,
//! which a CH59x does not answer). It builds, but the default `wlink` runner
//! only flashes it; nothing reads the RTT buffer. Once probe-rs supports the
//! part:
//!
//! ```console
//! probe-rs run --chip CH592 target/riscv32imc-unknown-none-elf/release/defmt_rtt
//! ```

#![no_std]
#![no_main]

use ch5xx_hal as hal;

use defmt_rtt as _;
use {defmt::info, panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let _p = hal::init(Default::default());

    info!("ch5xx-hal defmt/RTT demo");
    info!("sysclk: {} Hz", hal::rcc::clocks().sysclk.0);

    let mut delay = hal::delay::Delay;
    let mut n: u32 = 0;

    loop {
        info!("tick {}", n);
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
