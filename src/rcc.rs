//! Clock configuration.
//!
//! The CH5xx has no RCC block. `SYS.CLK_CFG` (`0x4000_1008`) selects the system
//! clock source and divider, `SYS.HFCK_PWR_CTRL` (`0x4000_100A`) powers the
//! oscillators, and `SYSTEM_CONTROL.XT32M_TUNE` (`0x4000_104E`) trims the 32 MHz
//! crystal. All of them are RWA: writes need the safe-access (SAM) handshake
//! described below.
//!
//! # Tree
//!
//! Unlike the CH32 families there are no AHB/APB prescalers: `HCLK` *is* `Fsys`
//! and drives the core, SysTick and every peripheral.
//!
//! ```text
//!          HSI 16 MHz / HSE 32 MHz
//!                   |
//!      +------------+--------------+
//!      |            |              |
//!   undivided    / 2..=32        PLL: 624 / 2 = 312 MHz
//!   (16/32 MHz)  (1..=16 MHz)       / 4..=32
//!      |            |              |
//!      +------------+--------------+
//!                   Fsys
//! ```
//!
//! The datasheet's ranges (6.1, clock tree diagram): `1..=32 MHz` for the
//! divided/undivided modes and `9.75..=78 MHz` for the PLL mode. The PLL is
//! always 624 MHz (`16 MHz * 39`, or `32 MHz / 2 * 39`), so its source only
//! affects accuracy, not the result.
//!
//! `CLK_SYS_MOD` picks the branch, `CLK_PLL_DIV` divides it, `OSC32M_SEL`
//! selects HSE over HSI, and `XROM_SCLK_SEL` feeds the Flash controller. The
//! 32 kHz mode is deliberately not offered: it requires the whole program in
//! RAM.
//!
//! # Defaults
//!
//! [`Config::default`] is the datasheet's reset value, `CLK_CFG = 0x0003`
//! (HSI 16 MHz / 3 = 5.333 MHz), so adding this module does not change the
//! behaviour of existing programs. The boot ROM powers the PLL but does not
//! route the system clock through it.
//!
//! Every preset mirrors a `CLK_SOURCE_*` value in WCH's C SDK
//! (`CH58x_clk.h`), which is the authoritative encoding table. `HSE_*` and
//! `*_HSE_PLL_*` need the board's 32 MHz crystal; the reference board uses
//! 18 pF load capacitors, which is what the presets select.
//!
//! # Safe access and the switch itself
//!
//! The clock and oscillator registers are RWA. Entering SAM means writing
//! `0x57` then `0xA8` to `SAFE_ACCESS_SIG`; 0x00 closes the window. On CH585 it
//! otherwise closes by itself after about 16 cycles, so the switch clears
//! `SAFE_AUTO_EN` to keep it open, as WCH's `SetSysClock` does.
//!
//! [`set_config`] follows `SetSysClock` step for step, from RAM with
//! interrupts masked: it starts the crystal if it is off, powers the PLL when
//! the flash controller (XROM) is to be clocked from it, sets the flash
//! timing (`FLASH_SCK`, `FLASH_CFG`) for the new divider, and passes through the
//! 32 kHz mode on the way to the new `CLK_CFG`. Running from RAM keeps
//! instruction fetch out of the flash while its clock and timing change. It
//! then reads `CLK_CFG` back to prove the write landed.
//!
//! A debugger attach can change the clock: wlink sets `CLK_CFG` to `0x014D`
//! (HSI PLL / 13 = 24 MHz) when it connects to a CH585. A program that a
//! debugger reset is unaffected, since [`init`](crate::init) runs after the
//! reset.
//!
//! # Frequencies
//!
//! [`clocks`] reports the **nominal** frequency of the configured tree.
//! [`crate::delay`] separately measures the real SysTick rate against the 32 kHz
//! RTC and keeps that number for its busy-waits: for HSE/PLL the nominal value
//! is more accurate than a measurement referenced to an RC oscillator, and for
//! HSI the two agree to within its tolerance.
//!
//! Do not read `CLK_CFG` with `probe-rs read`: that halts the core and the value
//! it reports is not what the running firmware sees. Read it from firmware and
//! report it through the console instead.

use crate::atomic::{AtomicU32, Ordering};
use crate::pac;
use crate::sys::safe_access;
use crate::time::Hertz;

/// Starting value for the cache, until [`set_config`] runs.
///
/// This is the CH585 reset default: HSI 16 MHz divided by 3.
pub const DEFAULT_FREQUENCY: Hertz = Hertz(5_333_333);

static SYSCLK_HZ: AtomicU32 = AtomicU32::new(DEFAULT_FREQUENCY.0);

/// Oscillator at the root of the clock tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Osc {
    /// Internal 16 MHz RC oscillator (HSI). Always available.
    Hsi16,
    /// External 32 MHz crystal (HSE) on X32MI/X32MO. Requires the crystal.
    Hse32,
}

/// How `Fsys` is derived from the oscillator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sys {
    /// Undivided oscillator: 16 MHz on HSI, 32 MHz on HSE (`CLK_SYS_MOD = 10`).
    Direct,
    /// Oscillator divided by `div`, `div` in `2..=32` (`CLK_SYS_MOD = 00`).
    ///
    /// The divider must keep `Fsys` at or above the 1 MHz minimum; for HSI
    /// that limits `div` to `2..=16`.
    Divided(u8),
    /// 312 MHz divided by `div`, `div` in `4..=32` (`CLK_SYS_MOD = 01`).
    ///
    /// That is 9.75 MHz at 32 and 78 MHz at 4. Sources above 78 MHz are outside
    /// the datasheet's range.
    Pll(u8),
}

/// HSE crystal load capacitance, as `XT32M_TUNE.C_LOAD`.
///
/// The datasheet (6.3) maps the four-bit field to `2 * value + 10 pF` for
/// values 0..=7 and `2 * value + 2 pF` for 8..=15. The CH585M-R1-1v1 board
/// fits 18 pF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HseCap {
    /// 2 pF.
    P2 = 8,
    /// 4 pF.
    P4 = 9,
    /// 6 pF.
    P6 = 10,
    /// 8 pF.
    P8 = 11,
    /// 10 pF.
    P10 = 0,
    /// 12 pF.
    P12 = 1,
    /// 14 pF.
    P14 = 2,
    /// 16 pF.
    P16 = 3,
    /// 18 pF.
    P18 = 4,
    /// 20 pF.
    P20 = 5,
    /// 22 pF.
    P22 = 6,
    /// 24 pF.
    P24 = 7,
}

/// Clock configuration.
///
/// Build one from a preset ([`Config::HSI_PLL_24MHZ`] and friends), from
/// [`Config::hsi_nearest`] / [`Config::hse_nearest`], or from
/// [`Config::new`]. The fields stay public so a preset can be adjusted.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Oscillator the tree starts from.
    pub osc: Osc,
    /// Branch taken from `osc`.
    pub sys: Sys,
    /// Crystal load capacitance; only used when `osc` is [`Osc::Hse32`].
    pub hse_cap: HseCap,
}

impl Default for Config {
    /// The datasheet reset value: HSI 16 MHz divided by 3 (5.333 MHz).
    fn default() -> Self {
        Self {
            osc: Osc::Hsi16,
            sys: Sys::Divided(3),
            hse_cap: HseCap::P18,
        }
    }
}

/// Expands the preset table, so each entry stays one line and keeps its docs.
macro_rules! presets {
    ($($(#[$meta:meta])* $name:ident = $osc:ident, $sys:expr;)*) => {
        $(
            $(#[$meta])*
            pub const $name: Self = Self {
                osc: Osc::$osc,
                sys: $sys,
                hse_cap: HseCap::P18,
            };
        )*
    };
}

impl Config {
    /// A configuration for `osc` and `sys`, with the reference board's 18 pF
    /// HSE load capacitance.
    pub const fn new(osc: Osc, sys: Sys) -> Self {
        Self {
            osc,
            sys,
            hse_cap: HseCap::P18,
        }
    }

    /// Replace the HSE load capacitance.
    pub const fn with_hse_cap(self, hse_cap: HseCap) -> Self {
        Self { hse_cap, ..self }
    }

    /// Nominal `Fsys` produced by this configuration.
    ///
    /// This is compile-time arithmetic over the datasheet's tree; it does not
    /// read any register. The value is only meaningful for a configuration
    /// whose divisors are in range (see [`set_config`]).
    pub const fn sysclk(&self) -> Hertz {
        let base = match self.osc {
            Osc::Hsi16 => 16_000_000,
            Osc::Hse32 => 32_000_000,
        };
        match self.sys {
            Sys::Direct => Hertz(base),
            Sys::Divided(div) => Hertz(base / div as u32),
            Sys::Pll(div) => Hertz(312_000_000 / div as u32),
        }
    }

    /// Closest achievable clock that uses HSI only, so no crystal is needed.
    ///
    /// Ties go to the lower frequency. Use [`Config::sysclk`] to see what was
    /// chosen; a request outside 1..=78 MHz is still clamped to the tree.
    pub const fn hsi_nearest(hz: Hertz) -> Self {
        let mut best = 16_000_000;
        let mut sys = Sys::Direct;

        let mut div = 2u8;
        while div <= 16 {
            let freq = 16_000_000 / div as u32;
            if closer(hz.0, freq, best) {
                best = freq;
                sys = Sys::Divided(div);
            }
            div += 1;
        }

        let mut div = 4u8;
        while div <= 32 {
            let freq = 312_000_000 / div as u32;
            if closer(hz.0, freq, best) {
                best = freq;
                sys = Sys::Pll(div);
            }
            div += 1;
        }

        Self {
            osc: Osc::Hsi16,
            sys,
            hse_cap: HseCap::P18,
        }
    }

    /// Closest achievable clock driven by the 32 MHz crystal.
    ///
    /// Same selection rules as [`Config::hsi_nearest`], but divided and
    /// undivided branches use HSE, which reaches 32 MHz.
    pub const fn hse_nearest(hz: Hertz) -> Self {
        let mut best = 32_000_000;
        let mut sys = Sys::Direct;

        let mut div = 2u8;
        while div <= 32 {
            let freq = 32_000_000 / div as u32;
            if closer(hz.0, freq, best) {
                best = freq;
                sys = Sys::Divided(div);
            }
            div += 1;
        }

        let mut div = 4u8;
        while div <= 32 {
            let freq = 312_000_000 / div as u32;
            if closer(hz.0, freq, best) {
                best = freq;
                sys = Sys::Pll(div);
            }
            div += 1;
        }

        Self {
            osc: Osc::Hse32,
            sys,
            hse_cap: HseCap::P18,
        }
    }

    presets! {
        /// 16 MHz, HSI undivided (`CLK_SOURCE_HSI_16MHz`).
        HSI_16MHZ = Hsi16, Sys::Direct;

        /// 8 MHz, HSI / 2 (`CLK_SOURCE_HSI_8MHz`).
        HSI_8MHZ = Hsi16, Sys::Divided(2);

        /// 5.333 MHz, HSI / 3 (`CLK_SOURCE_HSI_5_3MHz`). The reset default.
        HSI_5_33MHZ = Hsi16, Sys::Divided(3);

        /// 4 MHz, HSI / 4 (`CLK_SOURCE_HSI_4MHz`).
        HSI_4MHZ = Hsi16, Sys::Divided(4);

        /// 2 MHz, HSI / 8 (`CLK_SOURCE_HSI_2MHz`).
        HSI_2MHZ = Hsi16, Sys::Divided(8);

        /// 1 MHz, HSI / 16 (`CLK_SOURCE_HSI_1MHz`).
        HSI_1MHZ = Hsi16, Sys::Divided(16);

        /// 32 MHz, HSE undivided (`CLK_SOURCE_HSE_32MHz`). Needs the crystal.
        HSE_32MHZ = Hse32, Sys::Direct;

        /// 16 MHz, HSE / 2 (`CLK_SOURCE_HSE_16MHz`). Needs the crystal.
        HSE_16MHZ = Hse32, Sys::Divided(2);

        /// 8 MHz, HSE / 4 (`CLK_SOURCE_HSE_8MHz`). Needs the crystal.
        HSE_8MHZ = Hse32, Sys::Divided(4);

        /// 6.4 MHz, HSE / 5 (`CLK_SOURCE_HSE_6_4MHz`). Needs the crystal.
        HSE_6_4MHZ = Hse32, Sys::Divided(5);

        /// 4 MHz, HSE / 8 (`CLK_SOURCE_HSE_4MHz`). Needs the crystal.
        HSE_4MHZ = Hse32, Sys::Divided(8);

        /// 2 MHz, HSE / 16 (`CLK_SOURCE_HSE_2MHz`). Needs the crystal.
        HSE_2MHZ = Hse32, Sys::Divided(16);

        /// 78 MHz, PLL / 4 (`CLK_SOURCE_HSI_PLL_78MHz`).
        HSI_PLL_78MHZ = Hsi16, Sys::Pll(4);

        /// 62.4 MHz, PLL / 5 (`CLK_SOURCE_HSI_PLL_62_4MHz`).
        HSI_PLL_62_4MHZ = Hsi16, Sys::Pll(5);

        /// 52 MHz, PLL / 6 (`CLK_SOURCE_HSI_PLL_52MHz`).
        HSI_PLL_52MHZ = Hsi16, Sys::Pll(6);

        /// 39 MHz, PLL / 8 (`CLK_SOURCE_HSI_PLL_39MHz`).
        HSI_PLL_39MHZ = Hsi16, Sys::Pll(8);

        /// 26 MHz, PLL / 12 (`CLK_SOURCE_HSI_PLL_26MHz`).
        HSI_PLL_26MHZ = Hsi16, Sys::Pll(12);

        /// 24 MHz, PLL / 13 (`CLK_SOURCE_HSI_PLL_24MHz`).
        ///
        /// 115200 baud becomes `DL = 26` = 115385 (+0.16%), inside the
        /// datasheet's 0.5% transmit budget, so this is the cheapest way to get
        /// a usable 115200 on a board without a crystal.
        HSI_PLL_24MHZ = Hsi16, Sys::Pll(13);

        /// 19.5 MHz, PLL / 16 (`CLK_SOURCE_HSI_PLL_19_5MHz`).
        HSI_PLL_19_5MHZ = Hsi16, Sys::Pll(16);

        /// 13 MHz, PLL / 24 (`CLK_SOURCE_HSI_PLL_13MHz`).
        HSI_PLL_13MHZ = Hsi16, Sys::Pll(24);

        /// 78 MHz, PLL / 4 from the crystal (`CLK_SOURCE_HSE_PLL_78MHz`).
        HSE_PLL_78MHZ = Hse32, Sys::Pll(4);

        /// 62.4 MHz, PLL / 5 from the crystal (`CLK_SOURCE_HSE_PLL_62_4MHz`).
        HSE_PLL_62_4MHZ = Hse32, Sys::Pll(5);

        /// 52 MHz, PLL / 6 from the crystal (`CLK_SOURCE_HSE_PLL_52MHz`).
        HSE_PLL_52MHZ = Hse32, Sys::Pll(6);

        /// 39 MHz, PLL / 8 from the crystal (`CLK_SOURCE_HSE_PLL_39MHz`).
        HSE_PLL_39MHZ = Hse32, Sys::Pll(8);

        /// 26 MHz, PLL / 12 from the crystal (`CLK_SOURCE_HSE_PLL_26MHz`).
        HSE_PLL_26MHZ = Hse32, Sys::Pll(12);

        /// 24 MHz, PLL / 13 from the crystal (`CLK_SOURCE_HSE_PLL_24MHz`).
        HSE_PLL_24MHZ = Hse32, Sys::Pll(13);

        /// 19.5 MHz, PLL / 16 from the crystal (`CLK_SOURCE_HSE_PLL_19_5MHz`).
        HSE_PLL_19_5MHZ = Hse32, Sys::Pll(16);

        /// 13 MHz, PLL / 24 from the crystal (`CLK_SOURCE_HSE_PLL_13MHz`).
        HSE_PLL_13MHZ = Hse32, Sys::Pll(24);
    }
}

/// Absolute difference, usable in `const fn`.
const fn abs_diff(a: u32, b: u32) -> u32 {
    if a > b {
        a - b
    } else {
        b - a
    }
}

/// Whether `cand` is a better match for `requested` than `best`.
///
/// Ties prefer the lower frequency: an undershoot leaves more of the UART
/// receiver's tolerance margin than an overshoot in the same direction.
const fn closer(requested: u32, cand: u32, best: u32) -> bool {
    let cand_err = abs_diff(cand, requested);
    let best_err = abs_diff(best, requested);
    cand_err < best_err || (cand_err == best_err && cand < best)
}

/// Clock frequencies, as seen by drivers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Clocks {
    /// System clock, which drives the core, SysTick and every peripheral.
    pub sysclk: Hertz,
}

/// The cached clock frequencies.
#[inline]
pub fn clocks() -> Clocks {
    Clocks {
        sysclk: Hertz(SYSCLK_HZ.load(Ordering::Relaxed)),
    }
}

/// Apply `config` to the hardware and update [`clocks`].
///
/// `ch5xx_hal::init` calls this before the delay calibration and the time
/// driver, which is the supported order. Only the clock tree and the
/// [`clocks`] cache are touched: nothing derived from the old frequency is
/// rebuilt. After a later call, re-run whatever depends on `Fsys`:
/// `crate::delay::recalibrate` (where the SysTick time driver is not selected)
/// for busy-waits, the embassy time driver (only during a fresh startup), UART
/// dividers, and any other peripheral configured from [`clocks`].
///
/// A crystal that is off is started at full bias first, as `SetSysClock`
/// does; reset leaves it running. The oscillator not selected is left
/// running too: `SetSysClock` stops it, but the 2-wire debug interface needs
/// HSI.
///
/// # Panics
///
/// Panics if the configuration cannot be represented by the hardware:
/// `Divided` outside `2..=32`, `Pll` outside `4..=32`, or a divided `Fsys`
/// below the 1 MHz minimum. Also panics if the register write does not read
/// back as written.
///
/// # Safety
///
/// `Fsys` changes underneath everything running, leaving the derived state
/// described above stale. The oscillator source change itself is safe because
/// this function keeps the PLL running (`PLL_GATE_DISS`): XROM is clocked from
/// the PLL in direct and PLL modes. Do not reproduce the sequence elsewhere
/// without that bit.
/// Stop and quiesce active Timer/DMA/NFC sessions before changing clocks, then
/// rebuild their timing state. Dedicated NFC clock mode does not remove its
/// system-clock precondition or make a live session safe across this change.
pub unsafe fn set_config(config: Config) -> Clocks {
    validate(&config);

    let sysclk = config.sysclk();
    let before = pac::SYS.clk_cfg().read();

    let (sys_mod, div) = match config.sys {
        Sys::Direct => (0b10, 0),
        Sys::Divided(div) => (0b00, div),
        Sys::Pll(div) => (0b01, div),
    };
    // WCH's CLK_SOURCE_* table sets XROM to the PLL for both direct and PLL
    // modes, and to the 2x oscillator clock for divided modes. Mirror it.
    let xrom_sclk = matches!(config.sys, Sys::Direct | Sys::Pll(_));
    let osc32m_sel = config.osc == Osc::Hse32;

    // Changing the PLL source stops the PLL for 30..40 us unless told not to.
    // Both direct and PLL modes clock the Flash controller (XROM) from the
    // PLL, so every source change must keep it running, not only changes made
    // while `Fsys` itself comes from the PLL.
    let pll_gate_diss = before.osc32m_sel() != osc32m_sel;

    if osc32m_sel {
        safe_access(|| {
            pac::SYSTEM_CONTROL
                .xt32m_tune()
                .modify(|w| w.set_xt32m_c_load(config.hse_cap as u8));
        });
    }

    let mut cfg = pac::sys::regs::ClkCfg::default();
    cfg.set_clk_pll_div(div);
    cfg.set_clk_sys_mod(sys_mod);
    cfg.set_xrom_sclk_sel(xrom_sclk);
    cfg.set_osc32m_sel(osc32m_sel);
    cfg.set_pll_gate_diss(pll_gate_diss);
    cfg.set_pll_gate_time(false);
    critical_section::with(|_| unsafe { set_sysclk_ram(cfg.0) });

    // The oscillator multiplexer needs 1 us to settle after OSC32M_SEL changes
    // (datasheet 6.3). No timer is available at this point, so spin a fixed
    // count that is conservative even at the 1 MHz minimum.
    for _ in 0..64 {
        core::hint::spin_loop();
    }

    // A SAM window that expired early drops the write silently, so prove it.
    let after = pac::SYS.clk_cfg().read();
    assert_eq!(after.clk_sys_mod(), sys_mod, "rcc: CLK_SYS_MOD did not stick");
    // The 5-bit field encodes 32 as 0.
    assert_eq!(after.clk_pll_div(), div & 0x1f, "rcc: CLK_PLL_DIV did not stick");
    assert_eq!(after.osc32m_sel(), osc32m_sel, "rcc: OSC32M_SEL did not stick");
    assert_eq!(after.xrom_sclk_sel(), xrom_sclk, "rcc: XROM_SCLK_SEL did not stick");

    SYSCLK_HZ.store(sysclk.0, Ordering::Relaxed);
    Clocks { sysclk }
}

/// Reject configurations the clock tree cannot produce.
fn validate(config: &Config) {
    match config.sys {
        Sys::Direct => {}
        Sys::Divided(div) => {
            assert!((2..=32).contains(&div), "rcc: CLK_PLL_DIV must be 2..=32");
            let base = match config.osc {
                Osc::Hsi16 => 16_000_000,
                Osc::Hse32 => 32_000_000,
            };
            assert!(base / div as u32 >= 1_000_000, "rcc: Fsys below the 1 MHz minimum");
        }
        Sys::Pll(div) => assert!((4..=32).contains(&div), "rcc: PLL divider must be 4..=32"),
    }
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
#[inline(never)]
#[unsafe(link_section = ".data.ch5xx_hal.rcc.set_sysclk")]
unsafe fn set_sysclk_ram(cfg: u16) {
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
