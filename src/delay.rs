//! Busy-wait delays on the core SysTick counter.
//!
//! CH5xx's SysTick is the WCH variant: `STK_CTLR` (`STE` bit 0, `STCLK` bit 2,
//! `MODE` bit 4), `STK_SR.CNTIF`, and the 32-bit `STK_CNTL` / `STK_CMPLR` pair.
//! There is no free-running 64-bit counter.
//!
//! The counter is started **once** by [`crate::init`] and left running; each
//! delay only reads `STK_CNT` and compares it against a target. The embassy
//! time driver shares the same counter.
//!
//! # Counter rate
//!
//! SysTick counts HCLK, which [`crate::rcc`] computes from `CLK_CFG`. That is
//! exact from the crystal, and +-1% from the CH584/CH585's 16 MHz RC.

use core::sync::atomic::Ordering;

use crate::pac::SYSTICK;

/// A delay backed by the SysTick counter.
///
/// Requires [`crate::init`] to have run, so the counter is started and its
/// rate is known.
pub struct Delay;

/// SysTick ticks per second, measured against the RTC at init.
static TICKS_PER_SEC: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(crate::rcc::DEFAULT_FREQUENCY.0);

/// The free-running SysTick count, at [`ticks_per_sec`] ticks per second.
#[allow(unused)]
#[inline]
pub(crate) fn ticks() -> u32 {
    SYSTICK.stk_cntl().read().cntl()
}

/// SysTick ticks per second.
#[allow(unused)]
#[inline]
pub(crate) fn ticks_per_sec() -> u32 {
    TICKS_PER_SEC.load(Ordering::Relaxed)
}

impl Delay {
    /// Start SysTick and measure its rate.
    ///
    /// # Safety
    ///
    /// Must run before the embassy time driver starts, and nothing else may
    /// reset or reload the counter: the time driver shares it, using only the
    /// compare register.
    pub(crate) unsafe fn init() {
        // Free-running, up-count, HCLK source (`STCLK` set; clearing it
        // would select HCLK/8). `STE` starts it.
        SYSTICK.stk_sr().modify(|w| w.set_cntif(false));
        SYSTICK.stk_cntl().write(|w| w.set_cntl(0));
        SYSTICK.stk_cnth().write(|w| w.set_cnth(0));
        SYSTICK.stk_ctlr().modify(|w| {
            w.set_mode(false); // up-count
            w.set_stclk(true); // source: HCLK (not HCLK/8)
            w.set_stie(false); // poll, never interrupt
            w.set_ste(true);
        });

        // Not calibrated against the 32 kHz RC, which is less accurate than
        // either high-frequency source until it is trimmed (~5% off on CH592,
        // ~9% on CH584).
        TICKS_PER_SEC.store(crate::rcc::clocks().sysclk.0, Ordering::Relaxed);
    }

    /// Busy-wait for `us` microseconds.
    ///
    /// Panics in debug builds if called before [`crate::init`].
    pub fn delay_us(&mut self, us: u32) {
        let hz = TICKS_PER_SEC.load(Ordering::Relaxed);
        debug_assert!(hz != 0, "delay used before `ch5xx_hal::init`");

        // `us * hz / 1_000_000` in 64 bits: `us` up to ~4e6 here, `hz` ~3e7, so
        // the product needs more than 32 bits.
        let mut remaining = (us as u64 * hz as u64 / 1_000_000) as u32;

        // The counter is 32 bits, so chase it in chunks to keep the
        // wrapping comparison meaningful.
        while remaining > 0 {
            let chunk = remaining.min(u32::MAX / 2);
            remaining -= chunk;

            let start = SYSTICK.stk_cntl().read().cntl();
            while SYSTICK.stk_cntl().read().cntl().wrapping_sub(start) < chunk {
                core::hint::spin_loop();
            }
        }
    }

    /// Busy-wait for `ms` milliseconds.
    pub fn delay_ms(&mut self, ms: u32) {
        // Split so a large `ms` cannot overflow the microsecond multiply.
        let mut ms = ms;
        while ms > 4_000 {
            self.delay_us(4_000_000);
            ms -= 4_000;
        }
        self.delay_us(ms * 1_000);
    }
}

pub(crate) unsafe fn init() {
    unsafe { Delay::init() }
}
