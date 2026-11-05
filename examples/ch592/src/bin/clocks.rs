//! Switch HCLK to 60 MHz from the PLL and report the clock registers over SDI
//! print.
//!
//! The host timestamps on each line are an independent check of the delay
//! timing: lines should arrive every 500 ms.
//!
//! ```console
//! cargo run --release --bin clocks
//! ```

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::{Level, Output, Speed};
use hal::pac;
use hal::rcc;
use {panic_halt as _, qingke_rt::entry};

#[entry]
fn main() -> ! {
    let p = hal::init(hal::Config {
        rcc: rcc::Config::HSE_PLL_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();

    let mut led = Output::new(p.PA8, Level::High, Speed::Low);
    let mut delay = Delay;

    let clk_cfg = pac::SYS.clk_cfg().read().0;
    let flash_cfg = unsafe { core::ptr::read_volatile(0x4000_1807 as *const u8) };
    let pll_config = pac::SYSTEM_CONTROL.pll_config().read().0;
    let variant = unsafe { core::ptr::read_volatile(0x0007_F010 as *const u32) } & 0xff;

    let mut n: u32 = 0;
    loop {
        hal::try_println!(
            "{} sysclk {} Hz, CLK_CFG {:#010x} FLASH_CFG {:#04x} PLL_CONFIG {:#04x} variant {}",
            n,
            rcc::clocks().sysclk.0,
            clk_cfg,
            flash_cfg,
            pll_config,
            variant
        );
        led.toggle();
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
