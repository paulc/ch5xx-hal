//! Repro: `rcc::set_config` leaves the FlashROM timing at its reset value.
//!
//! WCH's `SetSysClock` (`CH58x_sys.c`) sets `XROM_312M_SEL`, `R8_FLASH_SCK`
//! bit 4 and `R8_FLASH_CFG` for the new divider on every clock switch, running
//! from RAM while it does. `set_config` writes none of them, so `FLASH_CFG`
//! stays at its reset value, 0x07, at every clock.
//!
//! At four PLL clocks the test checks the three settings against
//! `SetSysClock`'s, and shows what they cost: the same 512 straight-line
//! instructions run from flash and from RAM, which has no wait states. On a
//! CH584F, flash takes 605% of the RAM time at 62.4 MHz with the reset
//! timing and 310% with WCH's. At 13 MHz WCH keeps 0x07 too, so that clock
//! passes either way.
//!
//! No wiring. The clock changes leave the embassy timebase scaled for the
//! clock at `init`, which cancels out of the flash/RAM ratio.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use defmt_rtt as _;
use embassy_time::Instant;
use hal::rcc::{self, Config};
use panic_halt as _;
use qingke_rt::entry;

const SAFE_MODE_CTRL: *const u8 = 0x4000_1010 as *const u8;
const FLASH_SCK: *const u8 = 0x4000_1805 as *const u8;
const FLASH_CFG: *const u8 = 0x4000_1807 as *const u8;

/// 512 instructions with no branches, so every fetch goes to the memory the
/// function lives in.
macro_rules! block {
    ($name:ident $(, $section:literal)?) => {
        #[inline(never)]
        $(#[link_section = $section])?
        fn $name() {
            unsafe {
                core::arch::asm!(
                    ".rept 512",
                    "addi {r}, {r}, 1",
                    ".endr",
                    r = inout(reg) 0u32 => _,
                    options(nomem, nostack),
                );
            }
        }
    };
}

block!(in_flash);
// `.data` is copied to RAM at startup, like WCH's `__HIGH_CODE`.
block!(in_ram, ".data.repro_flash_timing");

/// Ticks for 2000 runs of `f`.
fn time(f: fn()) -> u64 {
    let t0 = Instant::now();
    for _ in 0..2000 {
        f();
    }
    t0.elapsed().as_ticks()
}

#[entry]
fn main() -> ! {
    let _p = hal::init(Config::HSE_PLL_62_4MHZ.into());

    // `SetSysClock`'s FLASH_SCK bit 4 and FLASH_CFG for each PLL divider.
    let presets: [(&str, Config, bool, u8); 4] = [
        ("HSE_PLL_78MHZ (PLL/4)", Config::HSE_PLL_78MHZ, false, 0x01),
        ("HSE_PLL_62_4MHZ (PLL/5)", Config::HSE_PLL_62_4MHZ, false, 0x01),
        ("HSE_PLL_24MHZ (PLL/13)", Config::HSE_PLL_24MHZ, false, 0x02),
        ("HSE_PLL_13MHZ (PLL/24)", Config::HSE_PLL_13MHZ, false, 0x07),
    ];
    let (mut pass, mut fail) = (0u32, 0u32);
    for (name, config, want_sck4, want_cfg) in presets {
        unsafe { rcc::set_config(config) };
        let (mode, sck, cfg) = unsafe {
            (
                core::ptr::read_volatile(SAFE_MODE_CTRL),
                core::ptr::read_volatile(FLASH_SCK),
                core::ptr::read_volatile(FLASH_CFG),
            )
        };
        let xrom_312m = mode & 1 << 4 != 0;
        let sck4 = sck & 1 << 4 != 0;
        let pct = time(in_flash) * 100 / time(in_ram).max(1);
        let ok = xrom_312m && sck4 == want_sck4 && cfg == want_cfg;
        if ok {
            pass += 1;
        } else {
            fail += 1;
        }
        defmt::info!(
            "{}: FLASH_CFG {:#x} (WCH {:#x}), FLASH_SCK bit 4 {} (WCH {}), XROM_312M_SEL {} (WCH true); flash takes {}% of the RAM time: {}",
            name,
            cfg,
            want_cfg,
            sck4,
            want_sck4,
            xrom_312m,
            pct,
            if ok { "PASS" } else { "FAIL" }
        );
    }
    unsafe { rcc::set_config(Config::HSE_PLL_62_4MHZ) };

    loop {
        defmt::info!("repro_flash_timing done: {} pass, {} fail", pass, fail);
        qingke::riscv::asm::delay(20_000_000);
    }
}
