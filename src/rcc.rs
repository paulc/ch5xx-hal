//! Clock configuration.
//!
//! The CH5xx has no RCC block. `SYS.CLK_CFG` (`0x4000_1008`) selects the system
//! clock source and divider, and the oscillators are powered from `SYS` too.
//! This module applies [`Config`] and caches the resulting frequency for the
//! delay driver and for anything else that needs it. The frequency is
//! *computed* from the registers, never measured.
//!
//! # CH592
//!
//! Every HCLK source is derived from the 32 MHz crystal (HSE): either HSE
//! divided by `CLK_PLL_DIV`, or the 480 MHz PLL (HSE x 15) divided by it. The
//! boot ROM leaves the reset default, HSE / 5 = 6.4 MHz. `Config::sysclk`
//! selects another rate, applied with the same sequence (and flash timing) as
//! WCH's `SetSysClock`. The 32 kHz HCLK mode is not offered.
//!
//! # CH584 / CH585
//!
//! HCLK comes from the 32 MHz crystal (HSE), the 16 MHz RC (HSI, +-1%), or the
//! 312 MHz PLL output (half of 624 MHz), which either of them can drive, each
//! divided by `CLK_PLL_DIV`. The reset default is HSI / 3 = 5.33 MHz, and the
//! boot ROM leaves it there (WCH's startup code switches to the PLL before
//! `main`; qingke-rt does not). Again `Config::sysclk` selects another rate,
//! applied as `SetSysClock` does, and the 32 kHz HCLK mode is not offered.
//!
//! A debugger attach can change the clock: wlink sets `CLK_CFG` to `0x014D`
//! (HSI PLL / 13 = 24 MHz) when it connects to a CH585. A program that a
//! debugger reset is unaffected, since [`init`](crate::init) runs after the reset.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::time::Hertz;

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

#[cfg(sys_ch585)]
pub use ch585::*;
#[cfg(sys_ch592)]
pub use ch592::*;

#[cfg(sys_ch592)]
mod ch592 {
    use core::sync::atomic::Ordering;

    use super::SYSCLK_HZ;
    use crate::pac::SYS;
    use crate::time::Hertz;

    /// Reset-default system frequency (HSE / 5).
    pub const DEFAULT_FREQUENCY: Hertz = Hertz(6_400_000);

    /// HSE (external crystal) frequency. The CH592 needs a 32 MHz crystal.
    pub const HSE_FREQUENCY: Hertz = Hertz(32_000_000);
    /// PLL output: HSE x 15.
    pub const PLL_FREQUENCY: Hertz = Hertz(480_000_000);

    /// System clock (HCLK) source.
    ///
    /// The datasheet allows HCLK of 2-10 MHz or 15-80 MHz; this HAL stops at
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
    pub(crate) unsafe fn set_sysclk_ram(mode: u32, div: u8, flash_cfg: u8) {
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

#[cfg(sys_ch585)]
mod ch585 {
    use core::sync::atomic::Ordering;

    use super::SYSCLK_HZ;
    use crate::pac::SYS;
    use crate::time::Hertz;

    /// Reset-default system frequency (HSI / 3).
    pub const DEFAULT_FREQUENCY: Hertz = Hertz(16_000_000 / 3);

    /// HSE (external crystal) frequency. The CH584/CH585 need a 32 MHz crystal.
    pub const HSE_FREQUENCY: Hertz = Hertz(32_000_000);
    /// HSI (internal RC) frequency, +-1%.
    pub const HSI_FREQUENCY: Hertz = Hertz(16_000_000);
    /// The PLL output HCLK divides: half the 624 MHz PLL, from either
    /// oscillator.
    pub const PLL_FREQUENCY: Hertz = Hertz(312_000_000);

    /// System clock (HCLK) source.
    ///
    /// The datasheet allows HCLK of 1-32 MHz from an oscillator and
    /// 9.75-78 MHz from the PLL.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Sysclk {
        /// HSE (32 MHz) divided by `div`, 1..=32: 32 MHz down to 1 MHz.
        Hse { div: u8 },
        /// HSI (16 MHz) divided by `div`, 1..=16: 16 MHz down to 1 MHz.
        Hsi { div: u8 },
        /// The PLL, referenced to HSE, divided by `div`, 4..=32: 78 MHz down
        /// to 9.75 MHz.
        Pll { div: u8 },
        /// The PLL referenced to HSI instead, so as accurate as HSI.
        PllHsi { div: u8 },
    }

    impl Sysclk {
        /// The HCLK frequency this source produces.
        pub const fn frequency(self) -> Hertz {
            match self {
                Sysclk::Hse { div } => Hertz(HSE_FREQUENCY.0 / div as u32),
                Sysclk::Hsi { div } => Hertz(HSI_FREQUENCY.0 / div as u32),
                Sysclk::Pll { div } | Sysclk::PllHsi { div } => Hertz(PLL_FREQUENCY.0 / div as u32),
            }
        }

        /// The `CLK_CFG` value selecting this source, as WCH's `CLK_SOURCE_*`
        /// constants spell it: `CLK_PLL_DIV` (32 encoded as 0), `CLK_SYS_MOD`
        /// (`0b00` divided oscillator, `0b01` divided PLL, `0b10` undivided
        /// oscillator), `XROM_SCLK_SEL` (bit 8: flash clocked from the PLL)
        /// and `OSC32M_SEL` (bit 9: HSE rather than HSI).
        fn clk_cfg(self) -> u16 {
            const XROM_SCLK_SEL: u16 = 1 << 8;
            const OSC32M_SEL: u16 = 1 << 9;
            let osc = |div: u8, hse: bool, max: u8| {
                assert!((1..=max).contains(&div), "oscillator divider out of range");
                let sel = if hse { OSC32M_SEL } else { 0 };
                if div == 1 {
                    sel | XROM_SCLK_SEL | (0b10 << 6)
                } else {
                    sel | (div as u16 & 0x1f)
                }
            };
            let pll = |div: u8, hse: bool| {
                assert!((4..=32).contains(&div), "PLL divider out of range");
                let sel = if hse { OSC32M_SEL } else { 0 };
                sel | XROM_SCLK_SEL | (0b01 << 6) | (div as u16 & 0x1f)
            };
            match self {
                Sysclk::Hse { div } => osc(div, true, 32),
                Sysclk::Hsi { div } => osc(div, false, 16),
                Sysclk::Pll { div } => pll(div, true),
                Sysclk::PllHsi { div } => pll(div, false),
            }
        }
    }

    /// Clock configuration.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct Config {
        /// HCLK source, or `None` to keep what the boot ROM left (5.33 MHz
        /// from reset).
        pub sysclk: Option<Sysclk>,
    }

    impl Config {
        /// 78 MHz from the PLL, the datasheet's maximum.
        pub const SYSCLK_FREQ_78MHZ: Config = Config::new(Sysclk::Pll { div: 4 });
        /// 62.4 MHz from the PLL, the rate WCH's examples and BLE stack use.
        pub const SYSCLK_FREQ_62_4MHZ: Config = Config::new(Sysclk::Pll { div: 5 });
        /// 52 MHz from the PLL.
        pub const SYSCLK_FREQ_52MHZ: Config = Config::new(Sysclk::Pll { div: 6 });
        /// 39 MHz from the PLL.
        pub const SYSCLK_FREQ_39MHZ: Config = Config::new(Sysclk::Pll { div: 8 });
        /// 26 MHz from the PLL.
        pub const SYSCLK_FREQ_26MHZ: Config = Config::new(Sysclk::Pll { div: 12 });
        /// 24 MHz from the PLL.
        pub const SYSCLK_FREQ_24MHZ: Config = Config::new(Sysclk::Pll { div: 13 });
        /// 32 MHz straight from the crystal (PLL unused).
        pub const SYSCLK_FREQ_32MHZ_HSE: Config = Config::new(Sysclk::Hse { div: 1 });
        /// 16 MHz from the crystal.
        pub const SYSCLK_FREQ_16MHZ_HSE: Config = Config::new(Sysclk::Hse { div: 2 });
        /// 16 MHz straight from the RC (no crystal needed).
        pub const SYSCLK_FREQ_16MHZ_HSI: Config = Config::new(Sysclk::Hsi { div: 1 });

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
        let osc = if cfg.osc32m_sel() { HSE_FREQUENCY } else { HSI_FREQUENCY };
        match cfg.clk_sys_mod() {
            0b00 => Hertz(osc.0 / div),
            0b01 => Hertz(PLL_FREQUENCY.0 / div),
            0b10 => osc,
            _ => {
                // CK32K as HCLK: the RC or the 32.768 kHz crystal.
                if SYS.ck32k_config().read().clk_osc32k_xt() {
                    Hertz(32_768)
                } else {
                    Hertz(32_000)
                }
            }
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
            let cfg = sysclk.clk_cfg();
            critical_section::with(|_| unsafe { set_sysclk_ram(cfg) });
        }

        SYSCLK_HZ.store(sysclk_from_registers().0, Ordering::Relaxed);
    }

    const SAFE_MODE_CTRL: *mut u8 = 0x4000_1010 as *mut u8;
    const SAFE_ACCESS_SIG: *mut u8 = 0x4000_1040 as *mut u8;
    const CLK_SYS_CFG: *mut u16 = 0x4000_1008 as *mut u16;
    const HFCK_PWR_CTRL: *mut u8 = 0x4000_100a as *mut u8;
    const XT32M_TUNE: *mut u8 = 0x4000_104e as *mut u8;
    /// The flash controller's registers are not in the metapac yet.
    const FLASH_SCK: *mut u8 = 0x4000_1805 as *mut u8;
    const FLASH_CFG: *mut u8 = 0x4000_1807 as *mut u8;

    const RB_SAFE_AUTO_EN: u8 = 1 << 0;
    const RB_XROM_312M_SEL: u8 = 1 << 4;
    const RB_CLK_RC16M_PON: u8 = 1 << 1;
    const RB_CLK_XT32M_PON: u8 = 1 << 2;
    const RB_CLK_PLL_PON: u8 = 1 << 4;
    const RB_XROM_SCLK_SEL: u16 = 1 << 8;
    const RB_OSC32M_SEL: u16 = 1 << 9;

    /// Switch HCLK and the flash timing to `cfg` (a `CLK_CFG` value),
    /// following WCH's `SetSysClock` step for step.
    ///
    /// Lives in RAM (`.data` is copied there at startup), like the SDK's
    /// `__HIGH_CODE` version, because it changes the flash access timing
    /// while the clock moves under it. It must therefore not call anything
    /// in flash: raw register pokes only. Run with interrupts disabled.
    ///
    /// The oscillator not selected is left running: `SetSysClock` stops it,
    /// but the 1-wire debug interface needs HSI.
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.rcc.set_sysclk")]
    pub(crate) unsafe fn set_sysclk_ram(cfg: u16) {
        use core::ptr::{read_volatile as rd, write_volatile as wr};

        unsafe {
            // Flash clocked at 312 MHz, not 624 MHz, from the PLL. Safe mode
            // closes by itself after about 16 cycles unless `SAFE_AUTO_EN` is
            // cleared, far too soon for this sequence; keep it open until the
            // signature is cleared.
            wr(SAFE_MODE_CTRL, rd(SAFE_MODE_CTRL) | RB_XROM_312M_SEL);
            wr(SAFE_MODE_CTRL, rd(SAFE_MODE_CTRL) & !RB_SAFE_AUTO_EN);
            core::arch::asm!("fence.i");
            wr(SAFE_ACCESS_SIG, 0x57);
            wr(SAFE_ACCESS_SIG, 0xa8);
            core::arch::asm!("fence.i");

            if cfg & RB_OSC32M_SEL != 0 {
                if rd(HFCK_PWR_CTRL) & RB_CLK_XT32M_PON == 0 {
                    // Start the crystal at full bias, and give it time by
                    // running from the 32 kHz clock for a few cycles.
                    let tune = rd(XT32M_TUNE);
                    wr(XT32M_TUNE, tune | 0x03);
                    wr(HFCK_PWR_CTRL, rd(HFCK_PWR_CTRL) | RB_CLK_XT32M_PON);
                    let old = rd(CLK_SYS_CFG);
                    wr(CLK_SYS_CFG, old | 0xc0);
                    core::arch::asm!("nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop");
                    wr(CLK_SYS_CFG, old);
                    wr(XT32M_TUNE, tune);
                }
            } else {
                wr(HFCK_PWR_CTRL, rd(HFCK_PWR_CTRL) | RB_CLK_RC16M_PON);
            }

            let div = cfg & 0x1f;
            let sck = rd(FLASH_SCK) & !(1 << 4);
            if cfg & RB_XROM_SCLK_SEL != 0 {
                wr(HFCK_PWR_CTRL, rd(HFCK_PWR_CTRL) | RB_CLK_PLL_PON);
                let flash_cfg = match div {
                    0 => {
                        wr(FLASH_SCK, sck | (1 << 4));
                        0x07
                    }
                    1..=9 => {
                        wr(FLASH_SCK, sck);
                        0x01
                    }
                    10..=15 => {
                        wr(FLASH_SCK, sck);
                        0x02
                    }
                    _ => {
                        wr(FLASH_SCK, sck);
                        0x07
                    }
                };
                wr(FLASH_CFG, flash_cfg);
            } else {
                wr(FLASH_SCK, sck);
                wr(FLASH_CFG, if div < 8 { 0x51 } else { 0x57 });
            }

            // Through the 32 kHz mode, as `SetSysClock` does.
            wr(CLK_SYS_CFG, cfg | 0xc0);
            wr(CLK_SYS_CFG, cfg);

            wr(SAFE_MODE_CTRL, rd(SAFE_MODE_CTRL) | RB_SAFE_AUTO_EN);
            wr(SAFE_ACCESS_SIG, 0);
            core::arch::asm!("fence.i");
        }
    }
}
