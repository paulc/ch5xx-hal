//! Independent watchdog
//!
//! Follows the STM32 HALs' `IndependentWatchdog`: [`IndependentWatchdog::new`]
//! sets the timeout, [`start`](IndependentWatchdog::start) starts it and
//! [`feed`](IndependentWatchdog::feed) reloads it. Like the STM32 IWDG it counts
//! the internal 32 kHz RC (LSI) down from a 12-bit reload through a /4../512
//! prescaler, so timeouts run from 125 us to about 65 s, to the LSI's
//! accuracy. Once started it cannot be stopped.

use crate::sys::safe_access;
use crate::{pac, peripherals, Peri};

/// Nominal LSI frequency.
const LSI_HZ: u64 = 32_000;

const KEY_RELOAD: u16 = 0xaaaa;
const KEY_UNLOCK: u16 = 0x5555;
const KEY_START: u16 = 0xcccc;

/// IWDG driver.
pub struct IndependentWatchdog<'d, T: Instance> {
    _wdg: Peri<'d, T>,
}

impl<'d, T: Instance> IndependentWatchdog<'d, T> {
    /// Set the timeout, in microseconds. The watchdog does not run until
    /// [`start`](Self::start).
    ///
    /// # Panics
    ///
    /// If the timeout is outside 125 us..=65.5 s.
    pub fn new(wdg: Peri<'d, T>, timeout_us: u32) -> Self {
        // Find the smallest prescaler (/4 << pr) whose reload fits 12 bits.
        let ticks = timeout_us as u64 * LSI_HZ / 1_000_000;
        let pr = (0..8u8)
            .find(|&pr| ticks.div_ceil(4 << pr) <= 0xfff)
            .unwrap_or_else(|| panic!("watchdog timeout too long"));
        let rl = (ticks / (4 << pr)) as u16;
        if rl == 0 {
            panic!("watchdog timeout too short");
        }

        // The IWDG counts the LSI, so keep it powered.
        safe_access(|| pac::SYS.ck32k_config().modify(|w| w.set_clk_int32k_pon(true)));

        let sys = pac::SYS;
        sys.iwdg_kr().write(|w| w.set_iwdg_kr(KEY_UNLOCK));
        sys.iwdg_cfg().modify(|w| {
            w.set_pr(pr);
            w.set_rlr(rl);
        });
        sys.iwdg_kr().write(|w| w.set_iwdg_kr(KEY_RELOAD));

        Self { _wdg: wdg }
    }

    /// Start the watchdog.
    pub fn start(&mut self) {
        pac::SYS.iwdg_kr().write(|w| w.set_iwdg_kr(KEY_START));
    }

    /// Reload the counter.
    pub fn feed(&mut self) {
        pac::SYS.iwdg_kr().write(|w| w.set_iwdg_kr(KEY_RELOAD));
    }
}

trait SealedInstance {}

/// IWDG instance.
#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + 'static + Send {}

impl SealedInstance for peripherals::IWDG {}
impl Instance for peripherals::IWDG {}
