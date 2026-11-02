//! The time driver for the Embassy framework.
//!
//! Ported from ch32-hal's SysTick driver. CH592 (QingKe V4C) has the same
//! 64-bit SysTick counter/compare as the CH32V20x. Not enabled for CH585, whose
//! SysTick has not been checked.

#[cfg(ch592)]
#[path = "time_driver_systick.rs"]
pub mod time_driver_impl;

/// Initialize the Embassy time driver.
///
/// System global clocks and [`crate::delay`] must be initialized before calling
/// this function: the driver shares the delay's free-running SysTick counter.
///
/// # Safety
///
/// This function should be called only once.
pub unsafe fn init() {
    #[cfg(ch592)]
    critical_section::with(|cs| time_driver_impl::init(cs));
}

// No driver for this chip yet; keep the optional dependencies "used".
#[cfg(not(ch592))]
use {embassy_time_driver as _, embassy_time_queue_utils as _};
