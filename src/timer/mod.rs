//! CH585 timers: one counter/FIFO per instance, not CH32 multi-channel TIMs.
//!
//! Normal counting runs at Fsys without a prescaler. TMR0/3 additionally have
//! the SDK's NFC clock mode. DMA belongs to the timer, not a separate channel.
//! Clock changes and deep sleep require stopping and rebuilding active drivers.

use crate::{interrupt, pac, peripherals};

pub mod counter;
pub mod input_capture;
pub mod low_level;
pub mod simple_pwm;
mod types;
pub use types::*;

#[allow(private_bounds)]
pub trait Instance: crate::PeripheralType + SealedInstance + Send + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

/// Timer instances supporting the SDK's dedicated NFC clock (CH585).
pub trait NfcClockInstance: Instance {}

/// Timer instances with local DMA: all four on CH585, TMR1 and TMR2 on CH592.
pub trait DmaInstance: Instance {}

pub(crate) trait SealedInstance {
    /// The instance has the DMA registers.
    const HAS_DMA: bool;
    /// The registers, as the full block: TMR0/TMR3 on CH592 lack only the
    /// DMA ones, which `HAS_DMA` guards.
    fn regs() -> pac::tmr::TmrFull;
    unsafe fn set_remap(remap: bool);
    #[cfg(feature = "embassy")]
    fn state() -> &'static State;
}

#[cfg(feature = "embassy")]
pub(crate) struct State {
    pub events: crate::atomic::AtomicU8,
    pub rx_finite: crate::atomic::AtomicBool,
    pub waker: embassy_sync::waitqueue::AtomicWaker,
}

macro_rules! instance {
    ($name:ident, no_remap, dma: $dma:tt) => {
        instance!(@impl $name, dma: $dma, |remap| assert!(!remap, "no remap for this timer"));
    };
    ($name:ident, $set:ident, dma: $dma:tt) => {
        instance!(@impl $name, dma: $dma, |remap| pac::SYS.pin_alternate().modify(|w| w.$set(remap)));
    };
    (@impl $name:ident, dma: $dma:tt, $remap:expr) => {
        impl SealedInstance for peripherals::$name {
            const HAS_DMA: bool = $dma;
            fn regs() -> pac::tmr::TmrFull {
                unsafe { pac::tmr::TmrFull::from_ptr(pac::$name.as_ptr()) }
            }
            unsafe fn set_remap(remap: bool) {
                let f: fn(bool) = $remap;
                f(remap)
            }
            #[cfg(feature = "embassy")]
            fn state() -> &'static State {
                static STATE: State = State {
                    events: crate::atomic::AtomicU8::new(0),
                    rx_finite: crate::atomic::AtomicBool::new(false),
                    waker: embassy_sync::waitqueue::AtomicWaker::new(),
                };
                &STATE
            }
        }
        impl Instance for peripherals::$name {
            type Interrupt = interrupt::typelevel::$name;
        }
        instance!(@dma $name, $dma);
    };
    (@dma $name:ident, true) => {
        impl DmaInstance for peripherals::$name {}
    };
    (@dma $name:ident, false) => {};
}

#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr0")))]
instance!(TMR0, set_pin_tmr0, dma: true);
#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr1")))]
instance!(TMR1, set_pin_tmr1, dma: true);
#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr2")))]
instance!(TMR2, set_pin_tmr2, dma: true);
#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr3")))]
instance!(TMR3, set_pin_tmr3, dma: true);
#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr0")))]
impl NfcClockInstance for peripherals::TMR0 {}
#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr3")))]
impl NfcClockInstance for peripherals::TMR3 {}

#[cfg(all(tmr_ch592, not(feature = "time-driver-tmr0")))]
instance!(TMR0, set_pin_tmr0, dma: false);
#[cfg(all(tmr_ch592, not(feature = "time-driver-tmr1")))]
instance!(TMR1, set_pin_tmr1, dma: true);
#[cfg(all(tmr_ch592, not(feature = "time-driver-tmr2")))]
instance!(TMR2, set_pin_tmr2, dma: true);
#[cfg(all(tmr_ch592, not(feature = "time-driver-tmr3")))]
instance!(TMR3, no_remap, dma: false);

mod sealed {
    pub trait Pin {}
}
/// External TMR input/output pin with a checked remap selection.
#[allow(private_bounds)]
pub trait TimerPin<T: Instance, const REMAP: u8>: crate::gpio::Pin + sealed::Pin {}
macro_rules! pin {
    ($tmr:ident, $remap:literal, $pin:ident) => {
        if_pin!($pin,
            impl sealed::Pin for peripherals::$pin {}
            impl TimerPin<peripherals::$tmr, $remap> for peripherals::$pin {}
        );
    };
}
#[cfg(not(feature = "time-driver-tmr0"))]
pin!(TMR0, 0, PA9);
#[cfg(not(feature = "time-driver-tmr0"))]
pin!(TMR0, 1, PB23);
#[cfg(not(feature = "time-driver-tmr1"))]
pin!(TMR1, 0, PA10);
#[cfg(not(feature = "time-driver-tmr1"))]
pin!(TMR1, 1, PB10);
#[cfg(not(feature = "time-driver-tmr2"))]
pin!(TMR2, 0, PA11);
#[cfg(not(feature = "time-driver-tmr2"))]
pin!(TMR2, 1, PB11);
#[cfg(not(feature = "time-driver-tmr3"))]
pin!(TMR3, 0, PB22);
#[cfg(all(tmr_ch585, not(feature = "time-driver-tmr3")))]
pin!(TMR3, 1, PA2);

/// One handler for the instance's currently enabled timer events.
///
/// Masks reported sources rather than repeatedly clearing a level event. In
/// particular DMA_END must not be acknowledged as an ordinary W1C event while
/// NOW equals END in non-loop mode. No borrowed pointers are stored in this ISR.
#[cfg(feature = "embassy")]
pub struct InterruptHandler<T: Instance>(core::marker::PhantomData<T>);

#[cfg(feature = "embassy")]
impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let events = r.int_flag().read().0 & r.inter_en().read().0;
        r.inter_en().modify(|w| w.0 &= !events);
        if T::HAS_DMA && events & Events::DMA_END.bits() != 0 && T::state().rx_finite.load(crate::atomic::Ordering::Relaxed) {
            // Do not let an exhausted finite capture fill/overflow the FIFO
            // while its task is waiting to resume. TX must instead drain.
            r.ctrl_mod().modify(|w| w.set_count_en(false));
            r.ctrl_dma().write(|w| w.0 = 0);
            crate::dma::fence();
        }
        T::state().events.fetch_or(events, crate::atomic::Ordering::Relaxed);
        T::state().waker.wake();
    }
}

impl embedded_hal::pwm::Error for Error {
    fn kind(&self) -> embedded_hal::pwm::ErrorKind {
        embedded_hal::pwm::ErrorKind::Other
    }
}
