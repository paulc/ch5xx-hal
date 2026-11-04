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
//! measurement too. Blocking reads keep up with about one measurement per
//! 500 HCLK cycles (tested at 60 MHz, from flash); faster signals overrun.
//! Async reads also wake a task per measurement: allow about 600 cycles. At
//! 480 they keep up or fall behind depending on where the code lands in
//! flash, so an unrelated change elsewhere in the program can tip them.
//!
//! # DMA (TMR1/TMR2 on CH592, all four on CH584/CH585)
//!
//! [`InputCapture::read_dma`] moves raw FIFO entries into a RAM buffer, and
//! [`InputCapture::decode`] turns them into measurements. It keeps up with
//! levels a few tens of HCLK cycles long (tested with levels of 30 cycles on
//! average), where reading the FIFO needs about 500. The DMA takes the entries already queued first, and
//! capture carries on into the FIFO afterwards, so DMA and FIFO reads can be
//! mixed: they form one stream.
//!
//! Pins are in the [module tables](super).

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;

pub use super::InterruptHandler;
use super::{DmaInstance, Instance, TimerPin};
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

/// The RAM the timer DMA reaches: its address registers hold 15 bits on
/// CH592.
#[cfg(tmr_ch592)]
const DMA_WINDOW: core::ops::Range<usize> = 0x2000_0000..0x2000_8000;
/// The RAM the timer DMA reaches: 17 bits of address on CH584/CH585, so all
/// of it.
#[cfg(tmr_ch585)]
const DMA_WINDOW: core::ops::Range<usize> = 0x2000_0000..0x2002_0000;
/// `DMA_BEG`/`DMA_END`/`DMA_NOW` as written: byte addresses, word aligned.
#[cfg(tmr_ch592)]
type DmaAddr = u16;
#[cfg(tmr_ch585)]
type DmaAddr = u32;
const DMA_ADDR_MASK: usize = DMA_WINDOW.end - DMA_WINDOW.start - 4;

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

/// The timer's pin, as a capture (or count) input.
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

/// Turns FIFO entries into measurements.
struct Decoder {
    config: Config,
    /// Timeout ticks waiting to join the next measurement.
    pending: u32,
    /// Drop the next measurement (it started part-way).
    skip: bool,
}

impl Decoder {
    /// Start afresh: the next measurement is partial.
    fn resync(&mut self) {
        self.pending = 0;
        self.skip = true;
    }

    /// Decode one entry; `None` if it gives no result (a dropped partial
    /// measurement).
    #[inline(always)]
    fn push(&mut self, entry: u32) -> Option<Result<Capture, Error>> {
        let ticks = entry & MAX_TIMEOUT;
        let flag = entry & FLAG != 0;
        // In `BothEdges` mode the flag is the level, so a timeout shows only
        // as a full `CNT_END` of ticks.
        let timeout = match self.config.mode {
            InputCaptureMode::BothEdges => ticks == self.config.timeout,
            _ => flag,
        };
        let total = self.pending.saturating_add(ticks);
        if timeout {
            self.pending = total;
            return Some(Err(Error::Timeout));
        }
        self.pending = 0;
        if self.skip {
            self.skip = false;
            return None;
        }
        Some(Ok(match self.config.mode {
            InputCaptureMode::BothEdges if flag => Capture::High(total),
            InputCaptureMode::BothEdges => Capture::Low(total),
            _ => Capture::Period(total),
        }))
    }
}

/// Input capture driver on a `TMRx` timer.
pub struct InputCapture<'d, T: Instance, M: Mode> {
    _peri: Peri<'d, T>,
    _pin: CapturePin<'d, T>,
    decoder: Decoder,
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

impl<'d, T: DmaInstance> InputCapture<'d, T, Async> {
    /// Fill `buf` with the next `buf.len()` raw FIFO entries by DMA, waiting
    /// on the timer interrupt; then [`decode`](Self::decode) them.
    ///
    /// Dropping the future stops the DMA; the measurements it had moved are
    /// lost and the next one is dropped.
    ///
    /// # Panics
    ///
    /// If `buf` is empty or outside the RAM the DMA reaches (the first 32 KiB
    /// on CH592, all of it on CH584/CH585), for instance in flash.
    pub async fn read_dma(&mut self, buf: &mut [u32]) -> Result<(), Error> {
        self.check_overrun(T::regs().fifo_count().read().fifo_count())?;
        let end = start_dma::<T>(buf);
        let decoder = &mut self.decoder;
        let guard = OnDrop::new(|| {
            stop_dma::<T>();
            decoder.resync();
        });
        poll_fn(|cx| {
            T::waker().register(cx.waker());
            let r = T::dma_regs();
            if dma_done::<T>(end) {
                return Poll::Ready(());
            }
            r.inter_en().write(|w| w.set_ie_dma_end(true));
            if dma_done::<T>(end) {
                r.inter_en().write(|_| {});
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;
        guard.defuse();
        stop_dma::<T>();
        Ok(())
    }
}

impl<'d, T: DmaInstance, M: Mode> InputCapture<'d, T, M> {
    /// Fill `buf` with the next `buf.len()` raw FIFO entries by DMA,
    /// polling; then [`decode`](Self::decode) them.
    ///
    /// # Panics
    ///
    /// As [`read_dma`](InputCapture::read_dma).
    pub fn blocking_read_dma(&mut self, buf: &mut [u32]) -> Result<(), Error> {
        self.check_overrun(T::regs().fifo_count().read().fifo_count())?;
        let end = start_dma::<T>(buf);
        while !dma_done::<T>(end) {}
        stop_dma::<T>();
        Ok(())
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
            decoder: Decoder {
                config,
                pending: 0,
                skip: true,
            },
            _phantom: PhantomData,
        };
        this.restart();
        this
    }

    /// Empty the FIFO and start timing afresh. The next measurement is
    /// dropped, as after [`new`](InputCapture::new).
    pub fn restart(&mut self) {
        let r = T::regs();
        let mode = self.decoder.config.mode;
        let cap_edge = match mode {
            InputCaptureMode::BothEdges => 0b01,
            InputCaptureMode::Falling => 0b10,
            InputCaptureMode::Rising => 0b11,
        };
        r.inter_en().write(|_| {});
        // Select capture mode while the timer is held clear: a DMA (TMR1,
        // TMR2) enabled in PWM mode reads from RAM instead and, with the FIFO
        // held clear, stalls the bus (seen on CH592).
        r.ctrl_mod().write(|w| {
            w.set_all_clear(true);
            w.set_mode_in(true);
            w.set_cap_edge(cap_edge);
        });
        #[cfg(tmr_ch592)]
        r.pwm_mod().write(|_| {});
        // Writing `CNT_END` also clears the count.
        r.cnt_end().write(|w| w.set_cnt_end(self.decoder.config.timeout));
        r.int_flag().write(|w| w.0 = 0xff);
        r.ctrl_mod().write(|w| {
            w.set_mode_in(true);
            w.set_cap_edge(cap_edge);
            w.set_count_en(true);
        });
        self.decoder.resync();
    }

    /// The tick rate: HCLK.
    pub fn tick_frequency(&self) -> Hertz {
        crate::rcc::clocks().sysclk
    }

    /// The configuration in use.
    pub fn config(&self) -> &Config {
        &self.decoder.config
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

    /// Decode raw entries from [`read_dma`](InputCapture::read_dma), in the
    /// order they were read and each buffer once: timeouts and the partial
    /// measurement carry over between buffers and the FIFO reads.
    pub fn decode<'a>(&'a mut self, raw: &'a [u32]) -> impl Iterator<Item = Result<Capture, Error>> + 'a {
        let decoder = &mut self.decoder;
        raw.iter().filter_map(move |&entry| decoder.push(entry))
    }

    fn has_data() -> bool {
        T::regs().fifo_count().read().fifo_count() > 0
    }

    /// If the FIFO overflowed, empty it and report it. An overflow leaves
    /// the FIFO full until the next read, so `count` (the FIFO count) tells
    /// whether the flag needs checking.
    fn check_overrun(&mut self, count: u8) -> Result<(), Error> {
        let r = T::regs();
        if count >= FIFO_SIZE && r.int_flag().read().if_fifo_ov() {
            // Entries were overwritten: drop what is left.
            while r.fifo_count().read().fifo_count() > 0 {
                r.fifo().read();
            }
            r.int_flag().write(|w| w.set_if_fifo_ov(true));
            self.decoder.resync();
            return Err(Error::Overrun);
        }
        Ok(())
    }

    /// Decode FIFO entries until one gives a result; `Err(())` if the FIFO
    /// runs dry first. This is the polling hot path: one count read per
    /// entry.
    fn poll(&mut self) -> Result<Result<Capture, Error>, ()> {
        let r = T::regs();
        loop {
            let count = r.fifo_count().read().fifo_count();
            if count == 0 {
                return Err(());
            }
            if let Err(e) = self.check_overrun(count) {
                return Ok(Err(e));
            }
            let entry = r.fifo().read().fifo();
            if let Some(result) = self.decoder.push(entry) {
                return Ok(result);
            }
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

/// Point the DMA at `buf` and start it; returns the `DMA_END` value.
fn start_dma<T: DmaInstance>(buf: &mut [u32]) -> DmaAddr {
    let start = buf.as_mut_ptr() as usize;
    let end = start + 4 * buf.len();
    if buf.is_empty() || !DMA_WINDOW.contains(&start) || end > DMA_WINDOW.end {
        panic!("capture DMA buffer must be in RAM the timer DMA reaches");
    }
    let r = T::dma_regs();
    r.ctrl_dma().write(|_| {});
    r.int_flag().write(|w| w.set_if_dma_end(true));
    // The address fields count words from bit 2: write byte addresses whole.
    r.dma_beg().write(|w| w.0 = (start & DMA_ADDR_MASK) as DmaAddr);
    r.dma_end().write(|w| w.0 = (end & DMA_ADDR_MASK) as DmaAddr);
    compiler_fence(Ordering::SeqCst);
    r.ctrl_dma().write(|w| w.set_dma_enable(true));
    (end & DMA_ADDR_MASK) as DmaAddr
}

fn dma_done<T: DmaInstance>(end: DmaAddr) -> bool {
    T::dma_regs().dma_now().read().0 == end
}

/// Stop the DMA and clear its end flag (moving `DMA_END` off `DMA_NOW`
/// first, as the datasheet says).
fn stop_dma<T: DmaInstance>() {
    let r = T::dma_regs();
    r.ctrl_dma().write(|_| {});
    compiler_fence(Ordering::SeqCst);
    r.inter_en().modify(|w| w.set_ie_dma_end(false));
    let now = r.dma_now().read().0;
    r.dma_end().write(|w| w.0 = ((now as usize + 0x100) & DMA_ADDR_MASK) as DmaAddr);
    r.int_flag().write(|w| w.set_if_dma_end(true));
}
