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
//! CH592: `adc`, `embassy` (time driver), `exti`, `i2c`, `rtc`, `soft_i2c`
//! (bit-bang I2C on any pins), `spi`, `timer` (PWM, input capture with DMA,
//! edge counter), `uart`, `usb` (`embassy-usb` device) and `wdg`. Where the
//! peripheral exists in ch32-hal the API follows it; blocking and async
//! (`embedded-hal`, `embedded-hal-async`, `embedded-io(-async)`) variants are
//! provided where the hardware allows.
//!
//! # Features
//!
//! - the part, exactly one: `ch584m`, `ch584x`, `ch584f`, `ch585m`,
//!   `ch585f`, `ch585c`, `ch585d`, `ch592x`, `ch592f`, `ch592a`, `ch592d`.
//!   The package decides which pins exist.
//! - `rt` (default): `qingke-rt` runtime and interrupt vectors.
//! - `embassy`: the `embassy-time` driver on SysTick.
//! - `exti`: built-in GPIO interrupt handlers for `exti::ExtiInput`.
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

#[cfg(any(adc_ch592, adc_ch585))]
pub mod adc;

#[cfg(feature = "embassy")]
pub mod embassy;

pub mod delay;
#[cfg(any(gpio_ch592, gpio_ch585))]
pub mod exti;
pub mod gpio;
#[cfg(any(sys_ch592, sys_ch585))]
pub mod i2c;
#[cfg(led_ch585)]
pub mod ledc;
#[cfg(any(sys_ch592, sys_ch585))]
pub mod rtc;
#[cfg(any(tmr_ch592, tmr_ch585))]
pub mod soft_i2c;
#[cfg(any(spi_ch592, spi_ch585))]
pub mod spi;
#[cfg(any(tmr_ch592, tmr_ch585))]
pub mod timer;
pub mod uart;
#[cfg(any(usb_ch592, usbfs_ch585))]
pub mod usb;
#[cfg(any(sys_ch592, sys_ch585))]
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

    #[cfg(feature = "rt")]
    unsafe {
        wake_on_pending_interrupts()
    };

    critical_section::with(|cs| unsafe {
        gpio::init(cs);
        #[cfg(feature = "exti")]
        exti::init(cs);
    });

    #[cfg(all(feature = "rt", any(ch584, ch585)))]
    unsafe {
        enable_interrupts_v3c()
    };

    p
}

/// Turn interrupts on, which qingke-rt 0.8 fails to do on QingKe V3C.
///
/// qingke-rt reaches `main` through `mret` with `mstatus.MPIE` set, so `MIE`
/// should come up set. But the last step of its `_setup_interrupts` takes a
/// critical section (`wfi_to_wfe`), and on V3C that is `csrrc 0x800, 0x88`,
/// which clears `MPIE` as well, and is only undone if `MIE` was set: it was
/// not, so `main` starts with interrupts off (`mstatus` 0x1800, checked on a
/// CH584). qingke-rt's V4 path calls `gintenr::set_enable()` first; the V3C
/// path does not.
#[cfg(all(feature = "rt", any(ch584, ch585)))]
unsafe fn enable_interrupts_v3c() {
    unsafe { qingke::register::gintenr::set_enable() };
}

/// Let a pending interrupt end a `wfi` even while interrupts are masked.
///
/// qingke-rt turns on `PFIC_SCTLR.WFITOWFE` at startup, so `wfi` waits for an
/// event, because QingKe's own `wfi` does not wake with `mstatus.MIE` clear,
/// and embassy's `platform-riscv32` executor sleeps inside a critical
/// section. But an interrupt is an event for WFE only once
/// `PFIC_SCTLR.SEVONPEND` is set too: without it, an executor sleeping in
/// that critical section never wakes (checked on CH592; likely
/// ch32-rs/ch32-hal#201 too). An interrupt that becomes pending between the
/// executor's check and its `wfi` latches the event, so it is not lost.
#[cfg(feature = "rt")]
unsafe fn wake_on_pending_interrupts() {
    const PFIC_SCTLR: *mut u32 = 0xE000_ED10 as *mut u32;
    const SEVONPEND: u32 = 1 << 4;
    critical_section::with(|_| unsafe { PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() | SEVONPEND) });
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
