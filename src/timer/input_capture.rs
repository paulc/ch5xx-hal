//! Input capture on a `TMRx` timer.
//!
//! The API follows ch32-hal's [`InputCaptureMode`] and embassy's
//! `InputCapture`, reshaped for the CH5xx timer, which measures intervals
//! rather than latching a free-running counter:
//!
//! - [`InputCaptureMode::Rising`] / [`InputCaptureMode::Falling`] measure the
//!   period, rising edge to rising edge (or falling to falling).
//! - [`InputCaptureMode::BothEdges`] measures each level: the time from one
//!   edge to the next, and whether the input was high or low.
//!
//! Measurements are in timer ticks (HCLK cycles, see
//! [`InputCapture::tick_frequency`]) and queue in the timer's 8-entry FIFO.
//!
//! # Timeouts
//!
//! When the input does not change for [`Config::timeout`] ticks, the
//! hardware queues a timeout and starts timing again. [`InputCapture::read`]
//! reports each one as [`Error::Timeout`] and adds its ticks to the next
//! measurement, so measurements are complete however long they are (up to
//! `u32::MAX` ticks, where they saturate).
//!
//! # Lost measurements
//!
//! The first measurement after [`InputCapture::new`] or
//! [`InputCapture::restart`] starts part-way and is dropped. When the FIFO
//! is full, each new measurement overwrites the oldest unread one, so
//! measurements are lost and the queue is out of order: the next read
//! reports [`Error::Overrun`], empties the FIFO and drops the next (partial)
//! measurement too. Reading keeps up with about one measurement per 500 HCLK
//! cycles (tested at 60 MHz, from flash); faster signals overrun.
//!
//! Pins are in the [module table](super#timer-pins-ch592).

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use super::{Instance, TimerPin};
use crate::gpio::{AfType, AnyPin, Pull};
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, Peri};

/// Largest [`Config::timeout`]: the FIFO holds 25 bits of ticks.
pub const MAX_TIMEOUT: u32 = (1 << 25) - 1;

/// FIFO depth.
const FIFO_SIZE: u8 = 8;

/// Bit 25 of a FIFO entry: the level (`BothEdges`) or a timeout (`Rising`,
/// `Falling`).
const FLAG: u32 = 1 << 25;

/// Which edges a measurement runs between.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InputCaptureMode {
    /// Rising edge to rising edge: the period.
    Rising,
    /// Falling edge to falling edge: the period.
    Falling,
    /// Each edge to the next: the length of each high and low level.
    BothEdges,
}

/// Input capture configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub mode: InputCaptureMode,
    /// Ticks without an edge before the hardware reports a timeout,
    /// `1..=MAX_TIMEOUT`. The default is [`MAX_TIMEOUT`], 0.56 s at 60 MHz.
    pub timeout: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: InputCaptureMode::Rising,
            timeout: MAX_TIMEOUT,
        }
    }
}

/// A measurement, in timer ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Capture {
    /// `Rising` / `Falling`: one period.
    Period(u32),
    /// `BothEdges`: the input was high for this long.
    High(u32),
    /// `BothEdges`: the input was low for this long.
    Low(u32),
}

impl Capture {
    /// The measured time, whatever kind of measurement it is.
    pub fn ticks(self) -> u32 {
        match self {
            Self::Period(t) | Self::High(t) | Self::Low(t) => t,
        }
    }
}

/// Input capture error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// No edge for [`Config::timeout`] ticks. The time counts towards the
    /// next measurement.
    Timeout,
    /// The FIFO overflowed and measurements were lost.
    Overrun,
}

/// The timer's pin, as a capture input.
pub struct CapturePin<'d, T> {
    _pin: Peri<'d, AnyPin>,
    phantom: PhantomData<T>,
}

impl<'d, T: Instance> CapturePin<'d, T> {
    /// Take `pin` as `T`'s capture input, selecting its `REMAP`.
    pub fn new<const REMAP: u8>(pin: Peri<'d, impl TimerPin<T, REMAP>>, pull: Pull) -> Self {
        apply_remap!();
        crate::gpio::SealedPin::set_as_af(&*pin, AfType::input(pull));
        Self {
            _pin: pin.into(),
            phantom: PhantomData,
        }
    }
}

/// Timer interrupt handler, for the async [`InputCapture`].
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Mask everything; `read` re-enables what it waits for.
        T::regs().inter_en().write(|_| {});
        compiler_fence(Ordering::SeqCst);
        T::waker().wake();
    }
}

/// Input capture driver on a `TMRx` timer.
pub struct InputCapture<'d, T: Instance, M: Mode> {
    _peri: Peri<'d, T>,
    _pin: CapturePin<'d, T>,
    config: Config,
    /// Timeout ticks waiting to join the next measurement.
    pending: u32,
    /// Drop the next measurement (it started part-way).
    skip: bool,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance> InputCapture<'d, T, Blocking> {
    /// Start capturing, for polling with [`nb_read`](Self::nb_read) and
    /// [`blocking_read`](Self::blocking_read).
    ///
    /// # Panics
    ///
    /// If `config.timeout` is outside `1..=MAX_TIMEOUT`.
    pub fn new_blocking(tim: Peri<'d, T>, pin: CapturePin<'d, T>, config: Config) -> Self {
        Self::new_inner(tim, pin, config)
    }
}

impl<'d, T: Instance> InputCapture<'d, T, Async> {
    /// Start capturing, with async [`read`](Self::read) as well.
    ///
    /// # Panics
    ///
    /// If `config.timeout` is outside `1..=MAX_TIMEOUT`.
    pub fn new(
        tim: Peri<'d, T>,
        pin: CapturePin<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let this = Self::new_inner(tim, pin, config);
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        this
    }

    /// Wait for the next measurement.
    pub async fn read(&mut self) -> Result<Capture, Error> {
        loop {
            if let Ok(r) = self.poll() {
                return r;
            }
            poll_fn(|cx| {
                T::waker().register(cx.waker());
                let r = T::regs();
                // Clear the flags before checking the FIFO, so an entry that
                // lands in between still raises the interrupt.
                r.int_flag().write(|w| {
                    w.set_if_data_act(true);
                    w.set_if_cyc_end(true);
                });
                if Self::has_data() {
                    return Poll::Ready(());
                }
                r.inter_en().write(|w| {
                    w.set_ie_data_act(true);
                    w.set_ie_cyc_end(true);
                    w.set_ie_fifo_ov(true);
                });
                if Self::has_data() {
                    r.inter_en().write(|_| {});
                    return Poll::Ready(());
                }
                Poll::Pending
            })
            .await;
        }
    }
}

impl<'d, T: Instance, M: Mode> InputCapture<'d, T, M> {
    fn new_inner(tim: Peri<'d, T>, pin: CapturePin<'d, T>, config: Config) -> Self {
        if !(1..=MAX_TIMEOUT).contains(&config.timeout) {
            panic!("capture timeout out of range");
        }
        T::enable();
        let mut this = Self {
            _peri: tim,
            _pin: pin,
            config,
            pending: 0,
            skip: true,
            _phantom: PhantomData,
        };
        this.restart();
        this
    }

    /// Empty the FIFO and start timing afresh. The next measurement is
    /// dropped, as after [`new`](InputCapture::new).
    pub fn restart(&mut self) {
        let r = T::regs();
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| w.set_all_clear(true));
        r.pwm_mod().write(|_| {});
        // Writing `CNT_END` also clears the count.
        r.cnt_end().write(|w| w.set_cnt_end(self.config.timeout));
        r.int_flag().write(|w| w.0 = 0xff);
        r.ctrl_mod().write(|w| {
            w.set_mode_in(true);
            w.set_cap_edge(match self.config.mode {
                InputCaptureMode::BothEdges => 0b01,
                InputCaptureMode::Falling => 0b10,
                InputCaptureMode::Rising => 0b11,
            });
            w.set_count_en(true);
        });
        self.pending = 0;
        self.skip = true;
    }

    /// The tick rate: HCLK.
    pub fn tick_frequency(&self) -> Hertz {
        crate::rcc::clocks().sysclk
    }

    /// The configuration in use.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// The next measurement if there is one.
    pub fn nb_read(&mut self) -> Result<Capture, nb::Error<Error>> {
        match self.poll() {
            Ok(r) => r.map_err(nb::Error::Other),
            Err(()) => Err(nb::Error::WouldBlock),
        }
    }

    /// Wait for the next measurement, polling.
    pub fn blocking_read(&mut self) -> Result<Capture, Error> {
        loop {
            if let Ok(r) = self.poll() {
                return r;
            }
        }
    }

    fn has_data() -> bool {
        T::regs().fifo_count().read().fifo_count() > 0
    }

    /// Decode FIFO entries until one gives a result; `Err(())` if the FIFO
    /// runs dry first.
    fn poll(&mut self) -> Result<Result<Capture, Error>, ()> {
        let r = T::regs();
        loop {
            let count = r.fifo_count().read().fifo_count();
            if count == 0 {
                return Err(());
            }
            // An overflow leaves the FIFO full until the next read, so the
            // flag needs checking only then.
            if count >= FIFO_SIZE && r.int_flag().read().if_fifo_ov() {
                // Entries were overwritten: drop what is left.
                while r.fifo_count().read().fifo_count() > 0 {
                    r.fifo().read();
                }
                r.int_flag().write(|w| w.set_if_fifo_ov(true));
                self.pending = 0;
                self.skip = true;
                return Ok(Err(Error::Overrun));
            }
            let entry = r.fifo().read().fifo();
            let ticks = entry & MAX_TIMEOUT;
            let flag = entry & FLAG != 0;
            // In `BothEdges` mode the flag is the level, so a timeout shows
            // only as a full `CNT_END` of ticks.
            let timeout = match self.config.mode {
                InputCaptureMode::BothEdges => ticks == self.config.timeout,
                _ => flag,
            };
            let total = self.pending.saturating_add(ticks);
            if timeout {
                self.pending = total;
                return Ok(Err(Error::Timeout));
            }
            self.pending = 0;
            if self.skip {
                self.skip = false;
                continue;
            }
            return Ok(Ok(match self.config.mode {
                InputCaptureMode::BothEdges if flag => Capture::High(total),
                InputCaptureMode::BothEdges => Capture::Low(total),
                _ => Capture::Period(total),
            }));
        }
    }
}

impl<'d, T: Instance, M: Mode> Drop for InputCapture<'d, T, M> {
    fn drop(&mut self) {
        let r = T::regs();
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| w.set_all_clear(true));
    }
}
