//! GPIO pin interrupts, with ch32-hal's `ExtiInput` API.
//!
//! CH5xx has no EXTI block or EXTI channels: every GPIO pin has its own
//! interrupt enable, mode (edge/level) and flag bit in the GPIO block, and the
//! two ports raise one interrupt each (`GPIO_A`, `GPIO_B`). So
//! [`ExtiInput::new`] takes just the pin.
//!
//! - Polarity is the pin's `OUT` latch (1 = high/rising, 0 = low/falling),
//!   unused while the pin is an input.
//! - Any-edge arms the edge away from the current level. The datasheet's
//!   `INT_EDGE_TYPE` ("non-polarity sensitive") should do both edges, but on a
//!   CH592F it still fired on the `OUT`-selected edge only, so it is kept
//!   clear.
//! - The PB interrupt registers are 16 bits wide: PB22 and PB23 use bits 8 and
//!   9 (CH592 bonds no PB8/PB9).
//!
//! The `GPIO_A`/`GPIO_B` handlers are defined by this module when the `exti`
//! feature is on, as in ch32-hal.

use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{AnyPin, Input, Level, Pin as GpioPin, Port, Pull};
use crate::{pac, Peri};

const NEW_AW: AtomicWaker = AtomicWaker::new();
/// One waker per interrupt bit: PA0-15, then PB's 16 bits.
static EXTI_WAKERS: [AtomicWaker; 32] = [NEW_AW; 32];

/// Interrupt bit of pin `n` in its port's 16-bit interrupt registers.
#[inline]
fn int_bit(port: Port, n: u8) -> u8 {
    match port {
        Port::A => n,
        // PB22/PB23 → bits 8/9 (see the module docs).
        Port::B if n >= 16 => n - 14,
        Port::B => n,
    }
}

#[inline]
fn waker_index(port: Port, bit: u8) -> usize {
    match port {
        Port::A => bit as usize,
        Port::B => 16 + bit as usize,
    }
}

/// Service a port interrupt: clear and disarm the pins that fired, wake them.
///
/// # Safety
///
/// Call only from the port's interrupt handler.
pub unsafe fn on_irq(port: Port) {
    let g = pac::GPIO;
    let fired = critical_section::with(|_| match port {
        Port::A => {
            let fired = g.pa_int_if().read().0 & g.pa_int_en().read().0;
            g.pa_int_en().modify(|w| w.0 &= !fired);
            g.pa_int_if().write(|w| w.0 = fired);
            fired
        }
        Port::B => {
            let fired = g.pb_int_if().read().0 & g.pb_int_en().read().0;
            g.pb_int_en().modify(|w| w.0 &= !fired);
            g.pb_int_if().write(|w| w.0 = fired);
            fired
        }
    });

    let mut bits = fired;
    while bits != 0 {
        let b = bits.trailing_zeros() as u8;
        bits &= !(1 << b);
        EXTI_WAKERS[waker_index(port, b)].wake();
    }
}

/// EXTI input driver
pub struct ExtiInput<'d> {
    pin: Input<'d>,
}

impl<'d> Unpin for ExtiInput<'d> {}

impl<'d> ExtiInput<'d> {
    /// Create an interrupt-capable input on `pin`.
    pub fn new<T: GpioPin>(pin: Peri<'d, T>, pull: Pull) -> Self {
        Self {
            pin: Input::new(pin, pull),
        }
    }

    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    fn port_pin(&self) -> (Port, u8) {
        let p = &self.pin.pin.pin;
        (p.port(), p.pin())
    }

    pub async fn wait_for_high<'a>(&'a mut self) {
        let (port, n) = self.port_pin();
        let fut = ExtiInputFuture::new(port, n, Trigger::High);
        if self.is_high() {
            return;
        }
        fut.await
    }

    pub async fn wait_for_low<'a>(&'a mut self) {
        let (port, n) = self.port_pin();
        let fut = ExtiInputFuture::new(port, n, Trigger::Low);
        if self.is_low() {
            return;
        }
        fut.await
    }

    pub async fn wait_for_rising_edge<'a>(&'a mut self) {
        let (port, n) = self.port_pin();
        ExtiInputFuture::new(port, n, Trigger::Rising).await
    }

    pub async fn wait_for_falling_edge<'a>(&'a mut self) {
        let (port, n) = self.port_pin();
        ExtiInputFuture::new(port, n, Trigger::Falling).await
    }

    pub async fn wait_for_any_edge<'a>(&'a mut self) {
        let (port, n) = self.port_pin();
        let high = self.is_high();
        let fut = ExtiInputFuture::new(port, n, if high { Trigger::Falling } else { Trigger::Rising });
        // The level may have changed before the edge was armed.
        if self.is_high() != high {
            return;
        }
        fut.await
    }
}

impl<'d> embedded_hal::digital::ErrorType for ExtiInput<'d> {
    type Error = core::convert::Infallible;
}

impl<'d> embedded_hal::digital::InputPin for ExtiInput<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_async::digital::Wait for ExtiInput<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Trigger {
    High,
    Low,
    Rising,
    Falling,
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct ExtiInputFuture<'a> {
    port: Port,
    bit: u8,
    phantom: PhantomData<&'a mut AnyPin>,
}

impl<'a> ExtiInputFuture<'a> {
    fn new(port: Port, n: u8, trigger: Trigger) -> Self {
        let bit = int_bit(port, n);
        let mask = 1u16 << bit;
        let edge = !matches!(trigger, Trigger::High | Trigger::Low);
        let high = matches!(trigger, Trigger::High | Trigger::Rising);

        critical_section::with(|_| {
            let g = pac::GPIO;
            macro_rules! arm {
                ($mode:ident, $edge_type:ident, $set:ident, $set_f:ident, $clr:ident, $clr_f:ident, $if_:ident, $en:ident) => {{
                    g.$mode().modify(|w| if edge { w.0 |= mask } else { w.0 &= !mask });
                    g.$edge_type().modify(|w| w.0 &= !mask);
                    // Polarity lives in the (unused, for an input) OUT latch.
                    if high {
                        g.$set().write(|w| w.$set_f(n as usize, true));
                    } else {
                        g.$clr().write(|w| w.$clr_f(n as usize, true));
                    }
                    g.$if_().write(|w| w.0 = mask);
                    g.$en().modify(|w| w.0 |= mask);
                }};
            }
            match port {
                Port::A => arm!(pa_int_mode, pa_int_edge_type, pa_set, set_pa_set, pa_clr, set_pa_clr, pa_int_if, pa_int_en),
                Port::B => arm!(pb_int_mode, pb_int_edge_type, pb_set, set_pb_set, pb_clr, set_pb_clr, pb_int_if, pb_int_en),
            }
        });

        Self {
            port,
            bit,
            phantom: PhantomData,
        }
    }

    fn armed(&self) -> bool {
        let g = pac::GPIO;
        let en = match self.port {
            Port::A => g.pa_int_en().read().0,
            Port::B => g.pb_int_en().read().0,
        };
        en & (1 << self.bit) != 0
    }
}

impl<'a> Drop for ExtiInputFuture<'a> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            let g = pac::GPIO;
            let mask = 1u16 << self.bit;
            match self.port {
                Port::A => g.pa_int_en().modify(|w| w.0 &= !mask),
                Port::B => g.pb_int_en().modify(|w| w.0 &= !mask),
            }
        });
    }
}

impl<'a> Future for ExtiInputFuture<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        EXTI_WAKERS[waker_index(self.port, self.bit)].register(cx.waker());

        if !self.armed() {
            // The interrupt handler disarmed it: the event happened.
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

#[cfg(feature = "exti")]
mod irq_impl {
    use qingke_rt::interrupt;

    use super::*;

    #[interrupt]
    fn GPIO_A() {
        unsafe { on_irq(Port::A) };
    }

    #[interrupt]
    fn GPIO_B() {
        unsafe { on_irq(Port::B) };
    }
}

/// Enable the port interrupts. Pins stay disarmed until a future arms them.
#[cfg(feature = "exti")]
pub(crate) unsafe fn init(_cs: critical_section::CriticalSection) {
    use crate::interrupt::InterruptExt;

    for irq in [pac::Interrupt::GPIO_A, pac::Interrupt::GPIO_B] {
        irq.unpend();
        unsafe { irq.enable() };
    }
}
