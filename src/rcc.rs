//! Clock configuration.
//!
//! The CH5xx has no RCC block. `SYS.CLK_CFG` (`0x4000_1008`) selects the system
//! clock source and divider, and `SYS.HFCK_PWR_CTRL` powers the oscillators.
//! This module applies [`Config`] and caches the resulting frequency for the
//! delay driver and for anything else that needs it.
//!
//! # CH592
//!
//! Every HCLK source is derived from the 32 MHz crystal (HSE): either HSE
//! divided by `CLK_PLL_DIV`, or the 480 MHz PLL (HSE × 15) divided by it. The
//! frequency is therefore *computed* from `CLK_CFG`, which is exact; the boot
//! ROM leaves the reset default, HSE / 5 = 6.4 MHz. [`Config::sysclk`] selects
//! another rate, applied with the same sequence (and flash timing) as WCH's
//! `SetSysClock`. The 32 kHz HCLK mode is not offered.
//!
//! # CH585: why the frequency is measured
//!
//! The datasheet's `CLK_CFG` reset value is `0x0003` (`CLK_SYS_MOD = 00`,
//! "divided from 32 MHz"), but the boot ROM does not leave it there: on the
//! CH585 used for bring-up it reads `0x014D`, i.e. `CLK_SYS_MOD = 01`
//! (PLL-sourced) with `CLK_PLL_DIV = 13`. Turning that into a frequency means
//! modelling the PLL, so instead [`crate::delay::Delay::init`] measures the
//! real rate against the independent 32 kHz RTC counter and reports it through
//! [`set_sysclk`]. This port does **not** reconfigure the CH585's PLL.
//!
//! [`DEFAULT_FREQUENCY`] is only a starting value, used if calibration fails.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::time::Hertz;

/// Nominal system frequency, used until the real rate is known.
/// See the module docs.
#[cfg(not(ch592))]
pub const DEFAULT_FREQUENCY: Hertz = Hertz(32_000_000);
/// Reset-default system frequency (HSE / 5).
#[cfg(ch592)]
pub const DEFAULT_FREQUENCY: Hertz = Hertz(6_400_000);

static SYSCLK_HZ: AtomicU32 = AtomicU32::new(DEFAULT_FREQUENCY.0);

/// Clock frequencies, as seen by drivers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Clocks {
    /// System clock (HCLK), which drives the core, SysTick and the peripherals.
    pub sysclk: Hertz,
}

/// The cached clock frequencies.
#[inline]
pub fn clocks() -> Clocks {
    Clocks {
        sysclk: Hertz(SYSCLK_HZ.load(Ordering::Relaxed)),
    }
}

/// Record the measured system clock. Called by [`crate::delay`]'s calibration.
#[allow(unused)]
pub(crate) fn set_sysclk(hz: u32) {
    SYSCLK_HZ.store(hz, Ordering::Relaxed);
}

/// Clock configuration.
///
/// Empty for now: the CH585 is left as the boot ROM configured it.
#[cfg(not(ch592))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config;

/// Apply [`Config`] and refresh the cache.
///
/// # Safety
///
/// Must be called once, before anything reads [`clocks`] or uses [`crate::delay`].
#[cfg(not(ch592))]
pub(crate) unsafe fn init(_config: Config) {
    // The real frequency is measured during delay calibration; leave the
    // nominal value in place until then.
    SYSCLK_HZ.store(DEFAULT_FREQUENCY.0, Ordering::Relaxed);
}

#[cfg(ch592)]
pub use ch592::*;

#[cfg(ch592)]
mod ch592 {
    use core::sync::atomic::Ordering;

    use super::SYSCLK_HZ;
    use crate::pac::SYS;
    use crate::time::Hertz;

    /// HSE (external crystal) frequency. The CH592 needs a 32 MHz crystal.
    pub const HSE_FREQUENCY: Hertz = Hertz(32_000_000);
    /// PLL output: HSE × 15.
    pub const PLL_FREQUENCY: Hertz = Hertz(480_000_000);

    /// System clock (HCLK) source.
    ///
    /// The datasheet allows HCLK of 2–10 MHz or 15–80 MHz; this HAL stops at
    /// 60 MHz (see [`Sysclk::Pll`]).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Sysclk {
        /// HSE (32 MHz) divided by `div`, 2..=32: 16 MHz down to 1 MHz.
        Hse { div: u8 },
        /// PLL (480 MHz) divided by `div`, 8..=32: 60 MHz down to 15 MHz.
        ///
        /// The datasheet allows up to 80 MHz (`div` 6), but WCH's flash timing
        /// only covers 60 MHz: at 80 MHz a CH592F ran garbage, so `div` < 8 is
        /// rejected.
        Pll { div: u8 },
    }

    impl Sysclk {
        /// The HCLK frequency this source produces.
        pub const fn frequency(self) -> Hertz {
            match self {
                Sysclk::Hse { div } => Hertz(HSE_FREQUENCY.0 / div as u32),
                Sysclk::Pll { div } => Hertz(PLL_FREQUENCY.0 / div as u32),
            }
        }
    }

    /// Clock configuration.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct Config {
        /// HCLK source, or `None` to keep what the boot ROM left (6.4 MHz
        /// from reset).
        pub sysclk: Option<Sysclk>,
    }

    impl Config {
        /// 60 MHz from the PLL, the rate WCH's examples and BLE stack use.
        pub const SYSCLK_FREQ_60MHZ: Config = Config::new(Sysclk::Pll { div: 8 });
        /// 48 MHz from the PLL.
        pub const SYSCLK_FREQ_48MHZ: Config = Config::new(Sysclk::Pll { div: 10 });
        /// 32 MHz from the PLL.
        pub const SYSCLK_FREQ_32MHZ: Config = Config::new(Sysclk::Pll { div: 15 });
        /// 24 MHz from the PLL.
        pub const SYSCLK_FREQ_24MHZ: Config = Config::new(Sysclk::Pll { div: 20 });
        /// 16 MHz straight from the crystal (PLL unused).
        pub const SYSCLK_FREQ_16MHZ_HSE: Config = Config::new(Sysclk::Hse { div: 2 });
        /// 8 MHz straight from the crystal.
        pub const SYSCLK_FREQ_8MHZ_HSE: Config = Config::new(Sysclk::Hse { div: 4 });
        /// 6.4 MHz straight from the crystal: the reset default.
        pub const SYSCLK_FREQ_6_4MHZ_HSE: Config = Config::new(Sysclk::Hse { div: 5 });

        /// A configuration that switches HCLK to `sysclk`.
        pub const fn new(sysclk: Sysclk) -> Self {
            Self { sysclk: Some(sysclk) }
        }
    }

    /// Current HCLK frequency, computed from `CLK_CFG`.
    pub fn sysclk_from_registers() -> Hertz {
        let cfg = SYS.clk_cfg().read();
        let div = match cfg.clk_pll_div() {
            0 => 32,
            d => d as u32,
        };
        match cfg.clk_sys_mod() {
            0b01 => Hertz(PLL_FREQUENCY.0 / div),
            0b11 => {
                // CK32K as HCLK: the RC or the 32.768 kHz crystal.
                if SYS.ck32k_config().read().clk_osc32k_xt() {
                    Hertz(32_768)
                } else {
                    Hertz(32_000)
                }
            }
            _ => Hertz(HSE_FREQUENCY.0 / div),
        }
    }

    /// Apply [`Config`] and refresh the cache.
    ///
    /// # Safety
    ///
    /// Must be called once, before anything reads [`super::clocks`] or uses
    /// [`crate::delay`].
    pub(crate) unsafe fn init(config: Config) {
        if let Some(sysclk) = config.sysclk {
            let (mode, div, flash_cfg) = match sysclk {
                Sysclk::Hse { div } => {
                    assert!((2..=32).contains(&div), "HSE divider out of range");
                    (0b00, div, FLASH_CFG_HSE)
                }
                Sysclk::Pll { div } => {
                    assert!((8..=32).contains(&div), "PLL divider out of range (max 60 MHz)");
                    // CH592A (QFN28) parts need one more flash wait setting.
                    let flash_cfg = if is_ch592a() { FLASH_CFG_PLL_CH592A } else { FLASH_CFG_PLL };
                    (0b01, div, flash_cfg)
                }
            };
            // `div` 32 is encoded as 0.
            let div = div & 0x1f;
            critical_section::with(|_| unsafe { set_sysclk_ram(mode, div, flash_cfg) });
        }

        SYSCLK_HZ.store(sysclk_from_registers().0, Ordering::Relaxed);
    }

    /// `R8_FLASH_CFG` (`0x4000_1807`) values from WCH's `SetSysClock`. The
    /// flash controller's registers are not in the metapac yet.
    const FLASH_CFG_HSE: u8 = 0x51;
    const FLASH_CFG_PLL: u8 = 0x52;
    const FLASH_CFG_PLL_CH592A: u8 = 0x53;

    /// Chip variant byte in the ROM config area (`ROM_CFG_VERISON` in the SDK).
    const ROM_CFG_VERSION: *const u32 = 0x0007_F010 as *const u32;
    /// `DEF_CHIP_ID_CH592A`: the QFN28 CH592A/F.
    const CHIP_ID_CH592A: u32 = 9;

    fn is_ch592a() -> bool {
        unsafe { core::ptr::read_volatile(ROM_CFG_VERSION) & 0xff == CHIP_ID_CH592A }
    }

    const SAFE_ACCESS_SIG: *mut u8 = 0x4000_1040 as *mut u8;
    const CLK_SYS_CFG: *mut u32 = 0x4000_1008 as *mut u32;
    const PLL_CONFIG: *mut u8 = 0x4000_104b as *mut u8;
    const FLASH_CFG: *mut u8 = 0x4000_1807 as *mut u8;

    const RB_XT_32M_PWR_EN: u32 = 1 << 18;
    const RB_PLL_PWR_EN: u32 = 1 << 20;

    /// Switch HCLK and the flash timing, following WCH's `SetSysClock`.
    ///
    /// Lives in RAM (`.data` is copied there at startup), like the SDK's
    /// `__HIGH_CODE` version, because it changes the flash access timing
    /// while the clock moves under it. It must therefore not call anything
    /// in flash: raw register pokes only. Run with interrupts disabled.
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.rcc.set_sysclk")]
    unsafe fn set_sysclk_ram(mode: u32, div: u8, flash_cfg: u8) {
        // A macro, not a closure: a closure could be emitted out of line
        // (in flash) in a debug build.
        macro_rules! sam {
            ($($body:tt)*) => {
                core::arch::asm!("fence.i");
                core::ptr::write_volatile(SAFE_ACCESS_SIG, 0x57);
                core::ptr::write_volatile(SAFE_ACCESS_SIG, 0xa8);
                core::arch::asm!("fence.i");
                $($body)*
                core::ptr::write_volatile(SAFE_ACCESS_SIG, 0);
                core::arch::asm!("fence.i");
            };
        }

        unsafe {
            sam! { core::ptr::write_volatile(PLL_CONFIG, core::ptr::read_volatile(PLL_CONFIG) & !(1 << 5)); }

            // Power the PLL up first if it is off, and let it settle (as the
            // SDK's IAP `mySetSysClock` does).
            if core::ptr::read_volatile(CLK_SYS_CFG) & RB_PLL_PWR_EN == 0 {
                sam! { core::ptr::write_volatile(CLK_SYS_CFG, core::ptr::read_volatile(CLK_SYS_CFG) | RB_PLL_PWR_EN); }
                for _ in 0..2000 {
                    core::arch::asm!("nop", "nop");
                }
            }

            sam! {
                core::ptr::write_volatile(CLK_SYS_CFG, (mode << 6) | div as u32 | RB_XT_32M_PWR_EN | RB_PLL_PWR_EN);
                core::arch::asm!("nop", "nop", "nop", "nop");
            }
            sam! { core::ptr::write_volatile(FLASH_CFG, flash_cfg); }
            sam! { core::ptr::write_volatile(PLL_CONFIG, core::ptr::read_volatile(PLL_CONFIG) | (1 << 7)); }
        }
    }
}
