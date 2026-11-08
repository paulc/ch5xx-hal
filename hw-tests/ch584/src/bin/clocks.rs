//! Clock tree and `delay` checks.
//!
//! At the clock `init` selects, time `Delay::delay_ms(20)` with TMR1, which
//! counts HCLK; off by more than 2% fails. Then switch through several
//! presets and check `rcc::clocks()`. HCLK is also measured against the RTC's
//! 32 kHz counter and printed, with `CLK_CFG`, `HFCK_PWR_CTRL` and the flash
//! timing byte.
//!
//! Under `time-driver-systick` `delay` uses the embassy timebase. With
//! `time-driver-tmr0` it busy-waits on SysTick at a rate measured against the
//! 32 kHz counter at `init`, so an untrimmed 32 kHz RC shows up as a long or
//! short delay:
//!
//! ```console
//! CARGO_ARGS="--no-default-features --features platform-spin,time-driver-tmr0" \
//!     hw-tests/hw-test.sh hw-tests/ch584 clocks
//! ```

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use hal::delay::Delay;
use hal::timer::counter::Counter;
use hal::timer::PeriodTicks;
use hal::{pac, peripherals, rcc};
use hwtest::check;
use {panic_halt as _, qingke_rt::entry};

/// The longest TMR period: 0.86 s at 78 MHz.
const PERIOD: u32 = (1 << 26) - 1;

fn ticks_between(a: u32, b: u32) -> u32 {
    (b + PERIOD - a) % PERIOD
}

/// HCLK cycles per 1024 ticks of the RTC's 32 kHz counter, scaled to Hz.
fn measure_vs_32k(tmr: &Counter<'_, peripherals::TMR1>) -> u32 {
    let rtc = || pac::RTC.cnt_32k().read().0 as u16;
    let start = rtc();
    while rtc() == start {}
    let (r0, s0) = (rtc(), tmr.count());
    while rtc().wrapping_sub(r0) < 1024 {}
    let ticks = ticks_between(s0, tmr.count()) as u64;
    (ticks * 32_768 / 1024) as u32
}

fn report(name: &str, tmr: &Counter<'_, peripherals::TMR1>, check_delay: bool) {
    let hz = rcc::clocks().sysclk.0;
    let clk_cfg = pac::SYS.clk_cfg().read().0;
    let hfck = pac::SYS.hfck_pwr_ctrl().read().0;
    let flash_cfg = unsafe { core::ptr::read_volatile(0x4000_1807 as *const u8) };
    hwtest::println!(
        "{}: clocks() {} Hz, vs 32 kHz counter {} Hz, delay calibrated to {} Hz, CLK_CFG {:#06x} HFCK_PWR_CTRL {:#04x} FLASH_CFG {:#04x}",
        name,
        hz,
        measure_vs_32k(tmr),
        hal::delay::measured_sysclk().0,
        clk_cfg,
        hfck,
        flash_cfg
    );

    if !check_delay {
        return;
    }
    // TMR1 counts HCLK whatever `delay` thinks, so it is the reference.
    let mut delay = Delay;
    let t0 = tmr.count();
    delay.delay_ms(20);
    let ticks = ticks_between(t0, tmr.count()) as u64;
    let us = ticks * 1_000_000 / hz as u64;
    check!(
        (19_600..=20_400).contains(&us),
        "{}: delay_ms(20) took {} us",
        name,
        us
    );
}

#[entry]
fn main() -> ! {
    let p = hal::init(rcc::Config::HSE_PLL_62_4MHZ.into());
    let tmr = Counter::new(p.TMR1, PeriodTicks::new(PERIOD).unwrap());
    hwtest::sdi_enable();
    check!(rcc::clocks().sysclk.0 == 62_400_000, "init HSE_PLL_62_4MHZ: clocks() {} Hz", rcc::clocks().sysclk.0);
    report("HSE_PLL_62_4MHZ", &tmr, true);

    let presets: [(&str, rcc::Config, u32); 4] = [
        ("HSI_PLL_24MHZ", rcc::Config::HSI_PLL_24MHZ, 24_000_000),
        ("HSE_32MHZ", rcc::Config::HSE_32MHZ, 32_000_000),
        ("HSI_5_33MHZ", rcc::Config::HSI_5_33MHZ, 5_333_333),
        ("HSE_PLL_78MHZ", rcc::Config::HSE_PLL_78MHZ, 78_000_000),
    ];
    for (name, config, hz) in presets {
        let clocks = unsafe { rcc::set_config(config) };
        // Under the SysTick time driver `delay` uses the embassy timebase,
        // which `set_config` does not rebuild (see the HAL docs), so `delay`
        // is only checked at the clock `init` selected.
        check!(clocks.sysclk.0 == hz, "set_config {}: {} Hz", name, clocks.sysclk.0);
        report(name, &tmr, false);
    }

    hwtest::done("clocks")
}
