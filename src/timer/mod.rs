//! Timers
//!
//! CH5xx has two kinds of PWM, neither like the CH32 advanced/general timers:
//!
//! - `TMR0`..`TMR3`: 26-bit timers counting HCLK, each with **one** pin that is
//!   either its PWM output or its capture input. [`simple_pwm::SimplePwm`]
//!   drives the PWM output, [`input_capture::InputCapture`] measures the
//!   input (TMR1/TMR2 also by DMA) and [`counter::Counter`] counts its edges.
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
//! # Timer pins (CH584/CH585)
//!
//! | REMAP | TMR0 | TMR1 | TMR2 | TMR3 |
//! |-------|------|------|------|------|
//! | 0     | PA9  | PA10 | PA11 | PB22 |
//! | 1     | PB23 | PB10 | PB11 | PA2  |
//!
//! All four have DMA there, against TMR1 and TMR2 on CH592. TMR3 is as the
//! datasheet's pin table has it (PB22 "TMR3", PA2 "TMR3_"), and as checked on
//! a CH584 at PB22; CH585SFR.h's comment on `RB_PIN_TMR3` swaps them.
//!
//! PA10/PA11 are the 32 kHz crystal pins and PB10/PB11 the USB pins. PB23 is
//! also the RST# input: with `CFG_RESET_EN` set in the user config it is not
//! usable as TMR0's pin.

use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_sync::waitqueue::AtomicWaker;

use crate::{interrupt, pac, peripherals};

pub mod counter;
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

/// The registers common to all four timers: on CH592, TMR1/TMR2 add DMA.
#[cfg(tmr_ch592)]
pub(crate) type Regs = pac::tmr::TmrNodma;
/// The registers common to all four timers, DMA included.
#[cfg(tmr_ch585)]
pub(crate) type Regs = pac::tmr::TmrFull;

pub(crate) trait SealedInstance: crate::peripheral::RemapPeripheral {
    /// The registers common to all four timers.
    fn regs() -> Regs;
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

/// A `TMRx` timer with DMA: TMR1 and TMR2 on CH592, all four on CH584/CH585.
#[allow(private_bounds)]
pub trait DmaInstance: Instance + SealedDmaInstance {}

pin_trait!(TimerPin, Instance);

/// Timer interrupt handler, for the async [`input_capture::InputCapture`]
/// and [`counter::Counter`].
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

#[cfg(ch592)]
mod ch592 {
    use super::*;
    use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};

    macro_rules! impl_timer {
        ($inst:ident, $gate:ident, $remap:expr) => {
            impl SealedInstance for peripherals::$inst {
                fn regs() -> Regs {
                    unsafe { Regs::from_ptr(pac::$inst.as_ptr()) }
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

#[cfg(any(ch584, ch585))]
mod ch585 {
    use super::*;
    use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};

    macro_rules! impl_timer {
        ($inst:ident, $gate:ident, $remap:ident) => {
            impl SealedInstance for peripherals::$inst {
                fn regs() -> Regs {
                    pac::$inst
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
                    set_remap(RemapGroup::$remap, remap)
                }
            }

            impl SealedDmaInstance for peripherals::$inst {
                fn dma_regs() -> pac::tmr::TmrFull {
                    pac::$inst
                }
            }

            impl DmaInstance for peripherals::$inst {}
        };
    }

    impl_timer!(TMR0, Tmr0, Tmr0);
    impl_timer!(TMR1, Tmr1, Tmr1);
    impl_timer!(TMR2, Tmr2, Tmr2);
    impl_timer!(TMR3, Tmr3, Tmr3);

    // The datasheet's pin table (see the module docs for TMR3).
    pin_trait_impl!(crate::timer::TimerPin, TMR0, PA9, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR0, PB23, 1);
    pin_trait_impl!(crate::timer::TimerPin, TMR1, PA10, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR1, PB10, 1);
    pin_trait_impl!(crate::timer::TimerPin, TMR2, PA11, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR2, PB11, 1);
    pin_trait_impl!(crate::timer::TimerPin, TMR3, PB22, 0);
    pin_trait_impl!(crate::timer::TimerPin, TMR3, PA2, 1);
}
