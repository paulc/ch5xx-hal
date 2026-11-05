//! HAL for WCH's CH5xx BLE microcontrollers.
//!
//! Ported from `ch32-hal`, adjusted for the CH5xx register model:
//!
//! - **No RCC.** Clocks, power and the pin mux live in `SYS` /
//!   `System_control` at `0x4000_1000`. [`rcc`] configures the clock tree and
//!   caches the resulting frequency.
//! - **No AFIO.** Peripheral signals are selected with the `SYS.PIN_ALTERNATE`
//!   "alternate pin enable" bits.
//! - **One GPIO block**, `pac::GPIO`, with per-port register groups
//!   (`pa_out`, `pb_dir`, …) instead of one peripheral per port.
//!
//! [`rcc`] is a plain module rather than a driver directory because the tree is
//! a single path with no peripheral prescalers: there is one frequency to
//! report.

#![no_std]
#![allow(static_mut_refs)]
#![allow(unexpected_cfgs)]

// The QingKe V3C core implements RV32IMBC: no A extension. `amoadd.w` and
// `amoswap.w` trap on real CH585 silicon (measured; see the `embassy` module
// docs), so the standard target's atomics cannot be used. Every atomic in this
// crate goes through `portable-atomic`, which falls back to a critical
// section. Plain loads/stores still compile to `lw`/`sw`.
extern crate portable_atomic as atomic;

pub use ch5xx_metapac as pac;
pub(crate) use embassy_hal_internal::{impl_peripheral, peripherals_definition, peripherals_struct};
pub use embassy_hal_internal::{Peri, PeripheralType};
#[cfg(feature = "rt")]
pub use qingke_rt::{entry, interrupt};

// Must come first: later modules use these macros.
include!(concat!(env!("OUT_DIR"), "/_macros.rs"));

pub(crate) mod internal;

mod macros;

pub mod time;

/// Operating modes for peripherals.
pub mod mode {
    trait SealedMode {}

    /// Operating mode for a peripheral.
    #[allow(private_bounds)]
    pub trait Mode: SealedMode {}

    macro_rules! impl_mode {
        ($name:ident) => {
            impl SealedMode for $name {}
            impl Mode for $name {}
        };
    }

    /// Blocking mode.
    pub struct Blocking;

    impl_mode!(Blocking);

    /// Async mode, driven by the peripheral interrupt.
    #[cfg(feature = "embassy")]
    pub struct Async;

    #[cfg(feature = "embassy")]
    impl_mode!(Async);
}

pub mod rcc;

pub mod debug;
pub mod prelude;

mod interrupt_ext;

// The `interrupt` module, generated because the interrupt list is per-chip.
include!(concat!(env!("OUT_DIR"), "/_interrupts.rs"));

// Must come last, so the `impl_foo!` macros above are all in scope.
pub(crate) mod _generated {
    #![allow(dead_code)]
    #![allow(unused_imports)]
    #![allow(non_snake_case)]
    #![allow(missing_docs)]

    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

pub use crate::_generated::{peripherals, Peripherals};

pub(crate) mod sys;

pub mod delay;
pub mod dma;
pub mod gpio;
pub mod timer;
pub mod uart;

/// Embassy integration (time driver). Requires the `embassy` feature.
#[cfg(feature = "embassy")]
pub mod embassy;

pub struct Config {
    /// System clock tree applied before any driver starts.
    pub rcc: rcc::Config,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rcc: Default::default(),
        }
    }
}

impl From<rcc::Config> for Config {
    /// Boot with this clock tree and the default for everything else.
    fn from(rcc: rcc::Config) -> Self {
        Self { rcc }
    }
}

/// Initialize the HAL and return the peripheral singletons.
///
/// Panics if called twice.
pub fn init(config: Config) -> Peripherals {
    // First, so a second call panics before anything is touched.
    let p = Peripherals::take();

    unsafe {
        // Apply the clock tree first: SysTick counts Fsys, so the calibration
        // and the time driver below must see the final clock.
        rcc::set_config(config.rcc);
        // `delay` measures the real SysTick rate against the RTC, when the
        // 32 kHz clock is the crystal, and keeps it for its busy-waits.
        // `rcc::clocks()` keeps reporting the nominal frequency the tree was
        // configured to.
        delay::init();
        // The time driver starts its counter and enables its own interrupt. It
        // reads the frequency from `rcc::clocks()`, so it must run after the
        // calibration above.
        #[cfg(feature = "embassy")]
        embassy::time_driver_impl::init();
    }

    // With or without a time driver: after `init()` interrupts work.
    #[cfg(feature = "rt")]
    unsafe {
        enable_interrupts()
    };

    critical_section::with(|cs| unsafe {
        gpio::init(cs);
    });

    p
}

/// Enable global interrupts and make them wake the core from `wfi`.
///
/// On V3C the authoritative register is `gintenr` (CSR 0x800), which `corecfgr`'s
/// `IE_REMAP_EN` maps onto `mstatus.MIE`/`MPIE`. qingke-rt's V3C startup leaves
/// it clear, so an interrupt handler never runs until this is written. WCH's own
/// `startup_CH585.S` does the equivalent with `mstatus = 0x88`.
///
/// `SEVONPEND` (bit 4) matters just as much: QingKe executes `wfi` as `wfe`
/// whenever `PFIC_SCTLR.WFITOWFE` is set, and `qingke-rt` sets that. A `wfe`
/// wakes on an *event*, not on a pending interrupt, so an alarm raised while the
/// executor sleeps inside a critical section (`mstatus.MIE` clear) posts nothing
/// and the core stays asleep until some unrelated wake-up. Measured cost: 8-25 ms
/// per alarm, i.e. a 500 ms `Timer` completing in 510-627 ms. With `SEVONPEND`
/// the same timer is 501.9 ms. An interrupt that becomes pending between the
/// executor's check and its `wfi` latches the event, so it is not lost.
///
/// Done in [`init`] whatever the features: a program with its own interrupt
/// handlers and no time driver needs both too.
#[cfg(feature = "rt")]
unsafe fn enable_interrupts() {
    // `PFIC.sctlr()` also carries `SETEVENT` (bit 5), which the PAC marks as
    // write-only. A `modify()` would read that bit back and write it, so the
    // field is set directly instead: read, OR in just this bit, write.
    critical_section::with(|_| {
        let mut sctlr = pac::PFIC.sctlr().read();
        sctlr.set_sevonpend(true);
        pac::PFIC.sctlr().write_value(sctlr);
    });

    // QingKe V4C (CH592) comes out of qingke-rt's startup with them on.
    #[cfg(any(ch584, ch585))]
    unsafe {
        qingke::register::gintenr::write(0x08)
    };
}

#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[$crate::interrupt]
            unsafe fn $irq() {
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
            )*
        )*
    };
}
