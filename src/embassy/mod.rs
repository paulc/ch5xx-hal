//! The time driver for the Embassy framework, and (feature `executor`) a
//! low-power executor.
//!
//! Ported from ch32-hal's SysTick driver. CH592 (QingKe V4C) has the same
//! 64-bit SysTick counter/compare as the CH32V20x; CH584/CH585 (V3C) have a
//! 32-bit one, which the driver extends.

#[cfg(any(systick_ch583, systick_ch585))]
#[path = "time_driver_systick.rs"]
pub mod time_driver_impl;

#[cfg(all(feature = "executor", any(systick_ch583, systick_ch585)))]
mod executor;
#[cfg(all(feature = "executor", any(systick_ch583, systick_ch585)))]
pub use executor::Executor;

/// Initialize the Embassy time driver.
///
/// System global clocks and [`crate::delay`] must be initialized before calling
/// this function: the driver shares the delay's free-running SysTick counter.
///
/// # Safety
///
/// This function should be called only once.
pub unsafe fn init() {
    #[cfg(any(systick_ch583, systick_ch585))]
    critical_section::with(|cs| time_driver_impl::init(cs));
}

// No driver for this chip yet; keep the optional dependencies "used".
#[cfg(not(any(systick_ch583, systick_ch585)))]
use {embassy_time_driver as _, embassy_time_queue_utils as _};
