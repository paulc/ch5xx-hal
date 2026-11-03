//! Timers (CH592)
//!
//! CH5xx has two kinds of PWM, neither like the CH32 advanced/general timers:
//!
//! - `TMR0`..`TMR3`: 26-bit timers counting HCLK, each with **one** pin that is
//!   either its PWM output or its capture input. [`simple_pwm::SimplePwm`]
//!   drives the PWM output and [`input_capture::InputCapture`] measures the
//!   input (TMR1/TMR2 also by DMA).
//! - `PWMX`: eight 8-bit PWM channels (PWM4-PWM11) on one shared counter and
//!   prescaler, or six 16-bit ones (PWM4-PWM9). See [`pwmx`].
//!
//! # Timer pins (CH592)
//!
//! | REMAP | TMR0 | TMR1 | TMR2 | TMR3 |
//! |-------|------|------|------|------|
//! | 0     | PA9  | PA10 | PA11 | PB22 |
//! | 1     | PB23 | PB10 | PB11 | -    |
//!
//! PA10/PA11 are the 32 kHz crystal pins and PB10/PB11 the USB pins. PB23 is
//! also the RST# input: with `CFG_RESET_EN` set in the user config it is not
//! usable as TMR0's pin.

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_sync::waitqueue::AtomicWaker;

use crate::{interrupt, pac, peripherals};

pub mod input_capture;
pub mod pwmx;
pub mod simple_pwm;

/// Output polarity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputPolarity {
    /// Idle low, high for the duty part of the period.
    #[default]
    ActiveHigh,
    /// Idle high, low for the duty part of the period.
    ActiveLow,
}

pub(crate) trait SealedInstance: crate::peripheral::RemapPeripheral {
    /// The registers common to all four timers (TMR1/TMR2 add DMA).
    fn regs() -> pac::tmr::TmrNodma;
    /// Ungate the timer's clock.
    fn enable();
    /// Woken by the timer interrupt.
    fn waker() -> &'static AtomicWaker;
}

/// A `TMRx` timer.
#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + 'static + Send {
    /// Interrupt for this timer.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

pub(crate) trait SealedDmaInstance {
    /// The full register block, with the DMA registers.
    fn dma_regs() -> pac::tmr::TmrFull;
}

/// A `TMRx` timer with DMA: TMR1 and TMR2.
#[allow(private_bounds)]
pub trait DmaInstance: Instance + SealedDmaInstance {}

pin_trait!(TimerPin, Instance);

/// Timer interrupt handler, for the async [`input_capture::InputCapture`].
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Mask everything; the driver re-enables what it waits for.
        T::regs().inter_en().write(|_| {});
        compiler_fence(Ordering::SeqCst);
        T::waker().wake();
    }
}

mod ch592 {
    use super::*;
    use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};

    macro_rules! impl_timer {
        ($inst:ident, $gate:ident, $remap:expr) => {
            impl SealedInstance for peripherals::$inst {
                fn regs() -> pac::tmr::TmrNodma {
                    unsafe { pac::tmr::TmrNodma::from_ptr(pac::$inst.as_ptr()) }
                }

                fn enable() {
                    set_clock_enabled(ClockGate::$gate, true);
                }

                fn waker() -> &'static AtomicWaker {
                    static WAKER: AtomicWaker = AtomicWaker::new();
                    &WAKER
                }
            }

            impl Instance for peripherals::$inst {
                type Interrupt = crate::interrupt::typelevel::$inst;
            }

            impl crate::peripheral::RemapPeripheral for peripherals::$inst {
                fn set_remap(remap: u8) {
                    let group: Option<RemapGroup> = $remap;
                    match group {
                        Some(g) => set_remap(g, remap),
                        None => assert!(remap == 0),
                    }
                }
            }
        };
    }

    impl_timer!(TMR0, Tmr0, Some(RemapGroup::Tmr0));
    impl_timer!(TMR1, Tmr1, Some(RemapGroup::Tmr1));
    impl_timer!(TMR2, Tmr2, Some(RemapGroup::Tmr2));
    impl_timer!(TMR3, Tmr3, None);

    macro_rules! impl_dma_timer {
        ($inst:ident) => {
            impl SealedDmaInstance for peripherals::$inst {
                fn dma_regs() -> pac::tmr::TmrFull {
                    pac::$inst
                }
            }

            impl DmaInstance for peripherals::$inst {}
        };
    }

    impl_dma_timer!(TMR1);
    impl_dma_timer!(TMR2);

    pin_trait_impl!(crate::timer::TimerPin, TMR0, PA9, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR0, PB23, 1);
    pin_trait_impl!(crate::timer::TimerPin, TMR1, PA10, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR1, PB10, 1);
    pin_trait_impl!(crate::timer::TimerPin, TMR2, PA11, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR2, PB11, 1);
    pin_trait_impl!(crate::timer::TimerPin, TMR3, PB22, 0);
}
