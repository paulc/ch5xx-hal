//! Edge counter on a `TMRx` timer ("count mode").
//!
//! The timer counts edges on its pin (rising, falling or both) from 0 up to
//! [`Config::period`] - 1, then wraps to 0 and raises its wrap flag. The pin
//! is taken as a [`CapturePin`].
//!
//! Pins are in the [module tables](super).

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

pub use super::input_capture::CapturePin;
use super::Instance;
pub use super::InterruptHandler;
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::{interrupt, Peri};

/// Largest [`Config::period`]: `CNT_END` is 26 bits and the counter wraps
/// at `CNT_END - 2` (checked on CH592; the datasheet says "final count
/// value - 2").
pub const MAX_PERIOD: u32 = (1 << 26) - 3;

/// Which edges count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Edge {
    Rising,
    Falling,
    BothEdges,
}

/// Counter configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub edge: Edge,
    /// Edges per wrap, `2..=MAX_PERIOD`. The default is [`MAX_PERIOD`].
    pub period: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            edge: Edge::Rising,
            period: MAX_PERIOD,
        }
    }
}

/// Edge counter on a `TMRx` timer.
pub struct Counter<'d, T: Instance, M: Mode> {
    _peri: Peri<'d, T>,
    _pin: CapturePin<'d, T>,
    config: Config,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance> Counter<'d, T, Blocking> {
    /// Start counting from 0.
    ///
    /// # Panics
    ///
    /// If `config.period` is outside `2..=MAX_PERIOD`.
    pub fn new_blocking(tim: Peri<'d, T>, pin: CapturePin<'d, T>, config: Config) -> Self {
        Self::new_inner(tim, pin, config)
    }
}

impl<'d, T: Instance> Counter<'d, T, Async> {
    /// Start counting from 0, with async [`wait_for_wrap`](Self::wait_for_wrap).
    ///
    /// # Panics
    ///
    /// If `config.period` is outside `2..=MAX_PERIOD`.
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

    /// Wait for the count to wrap, or return at once if it has wrapped since
    /// the flag was last cleared; clears the flag. Several wraps in between
    /// count as one.
    pub async fn wait_for_wrap(&mut self) {
        poll_fn(|cx| {
            T::waker().register(cx.waker());
            let r = T::regs();
            if self.has_wrapped() {
                self.clear_wrapped();
                return Poll::Ready(());
            }
            r.inter_en().write(|w| w.set_ie_cyc_end(true));
            if self.has_wrapped() {
                r.inter_en().write(|_| {});
                self.clear_wrapped();
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await
    }
}

impl<'d, T: Instance, M: Mode> Counter<'d, T, M> {
    fn new_inner(tim: Peri<'d, T>, pin: CapturePin<'d, T>, config: Config) -> Self {
        if !(2..=MAX_PERIOD).contains(&config.period) {
            panic!("counter period out of range");
        }
        T::enable();
        let mut this = Self {
            _peri: tim,
            _pin: pin,
            config,
            _phantom: PhantomData,
        };
        this.reset();
        this
    }

    /// Set the count to 0 and clear the wrap flag.
    pub fn reset(&mut self) {
        let r = T::regs();
        let cap_edge = match self.config.edge {
            Edge::BothEdges => 0b01,
            Edge::Falling => 0b10,
            Edge::Rising => 0b11,
        };
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| {
            w.set_all_clear(true);
            w.set_mode_in(true);
            w.set_cap_count(true);
            w.set_cap_edge(cap_edge);
        });
        #[cfg(tmr_ch592)]
        r.pwm_mod().write(|_| {});
        r.cnt_end().write(|w| w.set_cnt_end(self.config.period + 2));
        r.int_flag().write(|w| w.0 = 0xff);
        r.ctrl_mod().write(|w| {
            w.set_mode_in(true);
            w.set_cap_count(true);
            w.set_cap_edge(cap_edge);
            w.set_count_en(true);
        });
    }

    /// Edges counted since the last wrap or reset, `0..period`.
    pub fn count(&self) -> u32 {
        T::regs().count().read().count()
    }

    /// The configuration in use.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Whether the count has wrapped since the flag was last cleared.
    pub fn has_wrapped(&self) -> bool {
        T::regs().int_flag().read().if_cyc_end()
    }

    pub fn clear_wrapped(&mut self) {
        T::regs().int_flag().write(|w| w.set_if_cyc_end(true));
    }
}

impl<'d, T: Instance, M: Mode> Drop for Counter<'d, T, M> {
    fn drop(&mut self) {
        let r = T::regs();
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| w.set_all_clear(true));
    }
}
