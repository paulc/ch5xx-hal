//! Switch HCLK to 62.4 MHz from the PLL and report the clock registers over
//! SDI print.
//!
//! The host timestamps on each line are an independent check of the delay
//! timing: lines should arrive every 500 ms. Each line also gives HCLK as
//! measured against the RTC's 32 kHz counter, which shows how far the
//! untrimmed 32 kHz RC is off.
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
use hal::{pac, rcc};
use panic_halt as _;
use qingke_rt::entry;

/// HCLK cycles per 1024 ticks of the RTC's 32 kHz counter, scaled to Hz.
fn measure() -> u32 {
    let rtc = || pac::RTC.cnt_32k().read().0 as u16;
    let systick = || pac::SYSTICK.stk_cntl().read().cntl();
    let start = rtc();
    while rtc() == start {}
    let (r0, s0) = (rtc(), systick());
    while rtc().wrapping_sub(r0) < 1024 {}
    let ticks = systick().wrapping_sub(s0) as u64;
    (ticks * 32_768 / 1024) as u32
}

#[entry]
fn main() -> ! {
    let p = hal::init(hal::Config {
        rcc: rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();

    let mut led = Output::new(p.PB6, Level::High, Speed::Low);
    let mut delay = Delay;

    let clk_cfg = pac::SYS.clk_cfg().read().0;
    let hfck = pac::SYS.hfck_pwr_ctrl().read().0;
    let flash_cfg = unsafe { core::ptr::read_volatile(0x4000_1807 as *const u8) };

    let mut n: u32 = 0;
    loop {
        hal::try_println!(
            "{} sysclk {} Hz (vs 32 kHz RC: {} Hz), CLK_CFG {:#06x} HFCK_PWR_CTRL {:#04x} FLASH_CFG {:#04x}",
            n,
            rcc::clocks().sysclk.0,
            measure(),
            clk_cfg,
            hfck,
            flash_cfg,
        );
        led.toggle();
        n = n.wrapping_add(1);
        delay.delay_ms(500);
    }
}
