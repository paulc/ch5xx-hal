//! HAL for WCH's CH5xx BLE microcontrollers.
//!
//! Ported from `ch32-hal`, adjusted for the CH5xx register model:
//!
//! - **No RCC.** Clocks, power, clock gating and the pin mux live in `SYS` /
//!   `System_control` at `0x4000_1000`. [`rcc`] sets the system clock (CH592)
//!   or measures it (CH585) and reports it.
//! - **No AFIO.** Peripheral signals move to their alternate pins with the
//!   `SYS.PIN_ALTERNATE` bits; drivers take the choice as a `REMAP` const
//!   generic, as in ch32-hal.
//! - **One GPIO block**, `pac::GPIO`, with per-port register groups
//!   (`pa_out`, `pb_dir`, ...) instead of one peripheral per port.
//!
//! # Drivers
//!
//! All chips: [`gpio`], [`delay`], [`rcc`], [`debug`] (SDI print works on
//! CH592 only).
//!
//! CH592: `adc`, `embassy` (time driver), `exti`, `i2c`, `rtc`, `spi`,
//! `timer` (PWM, input capture with DMA), `uart`, `usb`
//! (`embassy-usb` device) and `wdg`. Where the peripheral exists in ch32-hal
//! the API follows it; blocking and async (`embedded-hal`,
//! `embedded-hal-async`, `embedded-io(-async)`) variants are provided where
//! the hardware allows.
//!
//! # Features
//!
//! - `ch585` / `ch592`: the chip (exactly one).
//! - `rt` (default): `qingke-rt` runtime and interrupt vectors.
//! - `embassy`: the `embassy-time` driver on SysTick (CH592).
//! - `exti`: built-in GPIO interrupt handlers for `exti::ExtiInput` (CH592).
//! - `memory-x`: emit `memory.x` for the selected chip.

#![no_std]
#![allow(static_mut_refs)]
#![allow(unexpected_cfgs)]

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
    /// Async mode.
    pub struct Async;

    impl_mode!(Blocking);
    impl_mode!(Async);
}

pub mod rcc;

pub mod debug;
pub mod prelude;

mod peripheral;
pub use peripheral::Peripheral;

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

#[cfg(ch592)]
pub mod adc;

#[cfg(feature = "embassy")]
pub mod embassy;

pub mod delay;
#[cfg(ch592)]
pub mod exti;
// Only `exti` and `spi` use it so far, and those are CH592-only.
#[cfg(not(ch592))]
use embedded_hal_async as _;
pub mod gpio;
#[cfg(ch592)]
pub mod i2c;
#[cfg(ch592)]
pub mod rtc;
#[cfg(ch592)]
pub mod spi;
#[cfg(ch592)]
pub mod timer;
pub mod uart;
#[cfg(ch592)]
pub mod usb;
#[cfg(ch592)]
pub mod wdg;

pub struct Config {
    pub rcc: rcc::Config,
}

impl Default for Config {
    fn default() -> Self {
        Self { rcc: Default::default() }
    }
}

/// Initialize the HAL and return the peripheral singletons.
///
/// Panics if called twice.
pub fn init(config: Config) -> Peripherals {
    // First, so a second call panics before anything is touched.
    let p = Peripherals::take();

    unsafe {
        rcc::init(config.rcc);
        delay::init();

        #[cfg(feature = "embassy")]
        embassy::init();
    }

    critical_section::with(|cs| unsafe {
        gpio::init(cs);
        #[cfg(all(ch592, feature = "exti"))]
        exti::init(cs);
    });

    p
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
