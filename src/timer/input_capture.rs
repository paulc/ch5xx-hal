//! Interval capture with mode-dependent timeout/level metadata.

use super::low_level::Timer;
use super::{CaptureEdge, CaptureWord, Error, Events, Instance, PeriodTicks, TimerPin};
use crate::gpio::{Flex, Pull};
use crate::{dma, Peri};

pub struct InputCapture<'d, T: Instance> {
    pub(crate) inner: Timer<'d, T>,
    _pin: Flex<'d>,
}
impl<'d, T: Instance> InputCapture<'d, T> {
    /// # Panics
    /// Timeout must fit the 25-bit captured interval field. Use
    /// `PeriodTicks::new_capture` to validate it without a constructor panic.
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, REMAP>>,
        pull: Pull,
        timeout: PeriodTicks,
        edge: CaptureEdge,
    ) -> Self {
        assert!(
            timeout.get() <= super::MAX_CAPTURE_PERIOD,
            "capture timeout exceeds 25-bit interval field"
        );
        let mut pin = Flex::new(pin);
        pin.set_as_input(pull);
        critical_section::with(|_| unsafe { T::set_remap(REMAP != 0) });
        let mut inner = Timer::new(peri);
        inner.configure_capture(timeout, edge);
        inner.start();
        Self { inner, _pin: pin }
    }
    /// Start capturing afresh: empty the FIFO and clear the events.
    pub fn restart(&mut self) {
        let mode = self.inner.mode;
        let period = self.inner.period;
        unsafe {
            self.inner.configure_raw(mode, period);
        }
        self.inner.start();
    }
    pub fn try_capture(&mut self) -> Result<Option<CaptureWord>, Error> {
        if self.inner.events().contains(Events::FIFO_ERROR) {
            return Err(Error::FifoOverrun);
        }
        Ok(unsafe { self.inner.pop_capture() })
    }
    pub fn read<'a>(
        &'a mut self,
        buffer: &'static mut [u32],
    ) -> Result<dma::Transfer<'a, 'd, T>, (dma::Error, &'static mut [u32])> 
    where
        T: super::DmaInstance,
    {
        if let Err(e) = dma::Descriptor::new(buffer.as_ptr() as usize, buffer.len()) {
            return Err((e, buffer));
        }
        let mode = self.inner.mode;
        let period = self.inner.period;
        unsafe {
            self.inner.configure_raw(mode, period);
        }
        unsafe {
            dma::Transfer::new(
                &mut self.inner,
                buffer,
                dma::Dir::PeripheralToMemory,
                dma::RunMode::Once,
            )
        }
    }
    /// Circular capture does not expose live slices or promise lossless laps.
    pub fn read_circular<'a>(
        &'a mut self,
        buffer: &'static mut [u32],
    ) -> Result<dma::Transfer<'a, 'd, T>, (dma::Error, &'static mut [u32])> 
    where
        T: super::DmaInstance,
    {
        if let Err(e) = dma::Descriptor::new(buffer.as_ptr() as usize, buffer.len()) {
            return Err((e, buffer));
        }
        let mode = self.inner.mode;
        let period = self.inner.period;
        unsafe {
            self.inner.configure_raw(mode, period);
        }
        unsafe {
            dma::Transfer::new(
                &mut self.inner,
                buffer,
                dma::Dir::PeripheralToMemory,
                dma::RunMode::Circular,
            )
        }
    }
    #[cfg(feature = "embassy")]
    pub fn bind_interrupt(
        &mut self,
        irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, super::InterruptHandler<T>>,
    ) {
        self.inner.bind_interrupt(irq);
    }
}

/// A measurement from [`Decoder`], in ticks (HCLK cycles).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Capture {
    /// [`CaptureEdge::Both`]: the input was high for this long.
    High(u32),
    /// [`CaptureEdge::Both`]: the input was low for this long.
    Low(u32),
    /// The same-edge modes: from one rising (or falling) edge to the next.
    Period(u32),
    /// The input did not change for the timeout. The timeout's ticks are
    /// added to the next measurement.
    Timeout,
}

/// Turns capture words into whole measurements.
///
/// The timer starts timing again after each timeout, so a level or period
/// longer than the timeout arrives as one or more timeout words and then
/// the remainder. The decoder adds them up (saturating at `u32::MAX`),
/// reporting each timeout as [`Capture::Timeout`] on the way. In
/// [`CaptureEdge::Both`] mode bit 25 is the level, so a timeout shows only
/// as a word of exactly the timeout's ticks; in the same-edge modes it is
/// [`CaptureWord::timeout_fragment`].
///
/// The first measurement after the timer starts began part-way, and is
/// dropped; so is the next one after [`Decoder::resync`], which a FIFO
/// overrun calls for (the FIFO then holds a broken stream). Checked on
/// CH584F and CH592F.
#[derive(Clone, Copy, Debug)]
pub struct Decoder {
    both: bool,
    timeout: u32,
    /// Timeout ticks waiting to join the next measurement.
    pending: u32,
    /// Drop the next measurement (it started part-way).
    skip: bool,
}

impl Decoder {
    /// A decoder for a timer capturing `edge` with `timeout`, as passed to
    /// [`InputCapture::new`].
    pub const fn new(edge: CaptureEdge, timeout: PeriodTicks) -> Self {
        Self {
            both: matches!(edge, CaptureEdge::Both),
            timeout: timeout.get(),
            pending: 0,
            skip: true,
        }
    }

    /// Start afresh: the next measurement is partial.
    pub fn resync(&mut self) {
        self.pending = 0;
        self.skip = true;
    }

    /// Decode one word; `None` if it gives no result (a dropped partial
    /// measurement).
    #[inline(always)]
    pub fn push(&mut self, word: CaptureWord) -> Option<Capture> {
        let ticks = word.ticks();
        let timeout = if self.both {
            ticks == self.timeout
        } else {
            word.timeout_fragment()
        };
        let total = self.pending.saturating_add(ticks);
        if timeout {
            self.pending = total;
            return Some(Capture::Timeout);
        }
        self.pending = 0;
        if self.skip {
            self.skip = false;
            return None;
        }
        Some(if !self.both {
            Capture::Period(total)
        } else if word.high_level() {
            Capture::High(total)
        } else {
            Capture::Low(total)
        })
    }
}
