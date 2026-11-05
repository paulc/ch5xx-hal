//! Embassy integration.
//!
//! One time driver is compiled in, chosen by a feature:
//!
//! | Feature | Driver | Takes |
//! | --- | --- | --- |
//! | `time-driver-systick` | `time_driver_systick.rs` | the core SysTick counter |
//! | `time-driver-tmr0` … `time-driver-tmr3` | `time_driver_tmr.rs` | that TMR instance |
//!
//! `time-driver-systick` is the lowest-latency option and is what the examples
//! use. It reprograms SysTick for its own reload model, which
//! [`crate::delay::Delay`] copes with by reading this driver's timebase rather
//! than the raw counter, so `delay` keeps working either way.
//!
//! Picking a `time-driver-tmr*` feature instead frees SysTick for anything else
//! that wants a free-running counter, and keeps `delay` on it too. TMR0 is the
//! least contended instance.
//!
//! # Ownership
//!
//! A TMR driver's peripheral is removed from [`crate::Peripherals`] by
//! `build.rs`, so [`crate::init`] can never hand the same instance to a second
//! user. SysTick is core-internal and was never in that struct.
//!
//! # Ticks per second
//!
//! [`embassy_time_driver::TICK_HZ`] is fixed at build time by embassy's
//! `tick-*` feature, while the CH585's system clock is measured at runtime.
//! The driver converts between the two using the calibrated rate from
//! [`crate::rcc::clocks`], so any `tick-hz-*` feature works; a rate that does
//! not divide the clock evenly loses sub-tick precision but never runs
//! backwards.
//!
//! # Executor
//!
//! The `executor` feature adds [`Executor`], a thread-mode executor that
//! waits in `wfi` when idle and, with `ble`, runs TMOS and sleeps between BLE
//! events. It replaces embassy-executor's `platform-*` executors.

#[cfg(any(
    feature = "time-driver-systick",
    feature = "time-driver-tmr0",
    feature = "time-driver-tmr1",
    feature = "time-driver-tmr2",
    feature = "time-driver-tmr3"
))]
mod reload_math;
mod reload_timer;

#[cfg(not(any(
    feature = "time-driver-systick",
    feature = "time-driver-tmr0",
    feature = "time-driver-tmr1",
    feature = "time-driver-tmr2",
    feature = "time-driver-tmr3"
)))]
compile_error!(
    "the `embassy` feature needs exactly one time driver: enable `time-driver-systick` (the default) \
     or one of `time-driver-tmr0` … `time-driver-tmr3`"
);

#[cfg(all(
    feature = "time-driver-systick",
    any(
        feature = "time-driver-tmr0",
        feature = "time-driver-tmr1",
        feature = "time-driver-tmr2",
        feature = "time-driver-tmr3"
    )
))]
compile_error!("enable only one time driver: `time-driver-systick` or a single `time-driver-tmr*`");

// Enumerating the pairs is the only stable way to say "at most one of these
// four"; `cfg` has no counting.
#[cfg(any(
    all(feature = "time-driver-tmr0", feature = "time-driver-tmr1"),
    all(feature = "time-driver-tmr0", feature = "time-driver-tmr2"),
    all(feature = "time-driver-tmr0", feature = "time-driver-tmr3"),
    all(feature = "time-driver-tmr1", feature = "time-driver-tmr2"),
    all(feature = "time-driver-tmr1", feature = "time-driver-tmr3"),
    all(feature = "time-driver-tmr2", feature = "time-driver-tmr3"),
))]
compile_error!("enable only one `time-driver-tmr*` feature");

#[cfg(feature = "executor")]
mod executor;
#[cfg(feature = "executor")]
pub use executor::Executor;
#[cfg(all(feature = "executor", feature = "ble"))]
pub(crate) use executor::pended;

#[cfg(feature = "time-driver-systick")]
#[path = "time_driver_systick.rs"]
pub mod time_driver_impl;

// `not(time-driver-systick)` only suppresses a duplicate `time_driver_impl` when
// both kinds are enabled — that case is already reported by the `compile_error!`
// above, and letting this module through too would bury it under `E0428`.
#[cfg(all(
    not(feature = "time-driver-systick"),
    any(
        feature = "time-driver-tmr0",
        feature = "time-driver-tmr1",
        feature = "time-driver-tmr2",
        feature = "time-driver-tmr3"
    )
))]
#[path = "time_driver_tmr.rs"]
pub mod time_driver_impl;

/// Priority given to the time driver's interrupt.
///
/// **On V3C only bits `[7:5]` of `PFIC_IPRIORx` are implemented**, and the
/// *QingKeV3 Processor Manual* states that the lower the value, the higher the
/// priority. So there are eight usable levels, 0…7, and `P8 = 0x80` is level 4:
/// mid-range, able to preempt ordinary work rather than the least urgent.
///
/// `qingke::interrupt::Priority` names 16 steps of `0x10`, but on this core each
/// pair (P0/P1, P2/P3, …) maps to the same hardware level. Do not read a 16-step
/// ordering into it; only the top three bits survive.
///
/// Preemption additionally requires nesting to be enabled: `PFIC_CFGR.NESTCTRL`
/// controls it and `CSR 0xBC1.NEST_LVL` bounds the depth. qingke-rt's V3C
/// startup sets `NEST_LVL = 1`, i.e. one level.
pub(crate) const TIME_DRIVER_PRIORITY: qingke::interrupt::Priority = qingke::interrupt::Priority::P8;
