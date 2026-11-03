//! Analog to Digital Converter (CH592)
//!
//! A 12-bit SAR ADC shared with the touch-key block. Unlike the CH32 ADC it
//! has no sample-time setting; what it does have shapes the API:
//!
//! - A PGA in front of the converter ([`Gain`], 1/4 to 16x). The input range
//!   is centred on the internal reference VINTA (Vref, about 1.05 V), not on
//!   0 V: single-ended, `Vin = Vref * (ADC / (2048 * gain) + 1 - 1 / gain)`.
//!   [`Adc::to_millivolts`] does that sum.
//! - An optional input buffer ([`Config::buffer`]), recommended unless the
//!   source impedance is below 1 kohm.
//! - Differential pairs AIN0/AIN2 and AIN1/AIN3
//!   ([`Adc::convert_differential`]).
//! - Two internal channels, the battery voltage ([`Vbat`]) and the
//!   temperature sensor ([`Temperature`]). Each needs its own front-end
//!   setting, which [`Adc::convert`] applies by itself.
//!
//! The API follows ch32-hal's `adc` module: [`Adc::new`], [`Adc::convert`]
//! taking `&mut` a channel (a pin, or an internal channel from
//! [`Adc::enable_temperature`]/[`Adc::enable_vbat`]), and [`AnyAdcChannel`].
//!
//! On top of that:
//!
//! - [`Adc::new_async`] adds [`Adc::read`], a conversion that waits for the
//!   end-of-conversion interrupt.
//! - [`Adc::blocking_read_dma`] (and, async, [`Adc::read_dma`]) fill a
//!   buffer with conversions of one channel, paced back to back or by the
//!   ADC's interval timer ([`Pace`]), the DMA storing each result.
//! - [`Adc::start_ring`] converts without end into a circular buffer, read
//!   through [`RingBufferedAdc`].
//!
//! # Channels (CH592)
//!
//! | AIN | 0   | 1   | 2    | 3    | 4    | 5    | 8   | 9   | 10  | 11  | 12  | 13  |
//! |-----|-----|-----|------|------|------|------|-----|-----|-----|-----|-----|-----|
//! | Pin | PA4 | PA5 | PA12 | PA13 | PA14 | PA15 | PB0 | PB6 | PA6 | PA7 | PA8 | PA9 |
//!
//! Channel 14 is VBAT and 15 the temperature sensor; 6 and 7 do not exist.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_sync::waitqueue::AtomicWaker;
use portable_atomic::AtomicU32;

use crate::gpio::SealedPin as _;
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::{interrupt, pac, peripherals, Peri};

/// Largest raw conversion result.
pub const ADC_MAX: u32 = (1 << 12) - 1;

/// Nominal internal reference VINTA, in millivolts (1.05 V +/- 15 mV).
pub const VREF_MV: u32 = 1050;

/// Raw result for 0 V across a differential pair.
const MID: u16 = 0x800;

const CH_VBAT: u8 = 14;
const CH_TEMP: u8 = 15;

/// Temperature-sensor calibration word in the info flash: the raw reading at
/// the upper half's temperature (0 = 25 degC) in the lower half.
const ROM_CFG_TMP_25C: *const u32 = 0x0007_f014 as *const u32;

/// ADC clock, from the 32 MHz clock. A conversion takes about 17 ADC clocks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleClock {
    /// 32 MHz / 10 = 3.2 MHz
    #[default]
    Div10,
    /// 32 MHz / 4 = 8 MHz
    Div4,
    /// 32 MHz / 6 = 5.33 MHz
    Div6,
    /// 32 MHz / 8 = 4 MHz
    Div8,
}

impl SampleClock {
    fn bits(self) -> u8 {
        match self {
            Self::Div10 => 0b00,
            Self::Div4 => 0b01,
            Self::Div6 => 0b10,
            Self::Div8 => 0b11,
        }
    }
}

/// PGA gain.
///
/// Single-ended input ranges (Vref = 1.05 V), with the datasheet's
/// high-accuracy range in brackets:
///
/// | Gain      | Range            | Accurate         |
/// |-----------|------------------|------------------|
/// | `Quarter` | -0.2 V to VIO33  | (2.9 V to VIO33) |
/// | `Half`    | -0.2 V to 3.15 V | (1.9 V to 3 V)   |
/// | `X1`      | 0 V to 2.1 V     | (0 V to 2 V)     |
/// | `X2`      | 0.53 V to 1.58 V | (0.6 V to 1.5 V) |
/// | `X4`      | 0.79 V to 1.31 V |                  |
/// | `X8`      | 0.92 V to 1.18 V |                  |
/// | `X16`     | 0.98 V to 1.12 V |                  |
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gain {
    /// -12 dB
    Quarter,
    /// -6 dB
    Half,
    /// 0 dB
    #[default]
    X1,
    /// 6 dB
    X2,
    /// 12 dB
    X4,
    /// 18 dB
    X8,
    /// 24 dB
    X16,
}

impl Gain {
    /// `(RB_ADC_PGA_GAIN[1:0], RB_ADC_PGA_GAIN[2])`. 2x has two encodings;
    /// the temperature sensor needs `011b`, so that is the one used.
    fn bits(self) -> (u8, bool) {
        match self {
            Self::Quarter => (0b00, false),
            Self::Half => (0b01, false),
            Self::X1 => (0b10, false),
            Self::X2 => (0b11, false),
            Self::X4 => (0b01, true),
            Self::X8 => (0b10, true),
            Self::X16 => (0b11, true),
        }
    }

    /// 1 / gain as `p / q`.
    fn inverse(self) -> (i32, i32) {
        match self {
            Self::Quarter => (4, 1),
            Self::Half => (2, 1),
            Self::X1 => (1, 1),
            Self::X2 => (1, 2),
            Self::X4 => (1, 4),
            Self::X8 => (1, 8),
            Self::X16 => (1, 16),
        }
    }
}

/// How [`Adc::read_dma`] spaces its conversions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pace {
    /// Back to back (`RB_ADC_CONT_EN`): a conversion every 18 ADC clocks,
    /// 5.6 us at the default [`SampleClock`] (measured on CH592).
    Continuous,
    /// One conversion every `n * 16` system clocks, `n` in `1..=256`
    /// (`R8_ADC_AUTO_CYCLE`, `RB_ADC_AUTO_EN`). The first conversion comes
    /// one interval after the start. An interval shorter than a conversion
    /// gives back-to-back conversions.
    Timed(u16),
}

impl Pace {
    /// [`Pace::Timed`] for the nearest interval to `hz` conversions per
    /// second at the current system clock, if it is in range.
    pub fn from_hz(hz: u32) -> Option<Self> {
        if hz == 0 {
            return None;
        }
        let sysclk = crate::rcc::clocks().sysclk.0;
        let n = (sysclk + 8 * hz) / (16 * hz);
        (1..=256).contains(&n).then_some(Self::Timed(n as u16))
    }
}

/// The DMA lapped a [`RingBufferedAdc`]'s reader and results were lost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OverrunError;

/// ADC configuration.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub struct Config {
    pub clock: SampleClock,
    /// Gain for external channels. The internal channels use their own.
    pub gain: Gain,
    /// Input buffer for single-ended external channels. Differential
    /// conversions always run without it, as the datasheet recommends.
    pub buffer: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: SampleClock::default(),
            gain: Gain::default(),
            buffer: true,
        }
    }
}

/// RAM the ADC DMA can reach: its address registers hold bits 14:1.
const DMA_WINDOW: core::ops::Range<usize> = 0x2000_0000..0x2000_8000;

/// ADC driver.
pub struct Adc<'d, T: Instance, M: Mode = Blocking> {
    _peri: Peri<'d, T>,
    config: Config,
    /// `(CFG, CHANNEL)` of the last conversion, to spot a front-end change.
    last: Option<(u8, u8)>,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance> Adc<'d, T, Blocking> {
    /// Power the ADC up with `config`. Turns the touch-key block off, which
    /// shares the converter.
    pub fn new(adc: Peri<'d, T>, config: Config) -> Self {
        Self::new_inner(adc, config)
    }
}

impl<'d, T: Instance> Adc<'d, T, Async> {
    /// [`Adc::new`], with the interrupt for [`Adc::read`], [`Adc::read_dma`]
    /// and [`Adc::start_ring`].
    ///
    /// The interrupt stays masked in the PFIC except while something waits
    /// on it (and while a ring runs): with no ADC interrupt enable set, every
    /// finished conversion asserts it until `R8_ADC_CONVERT` is written.
    pub fn new_async(
        adc: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(adc, config)
    }

    /// [`Adc::convert`], waiting for the end-of-conversion interrupt rather
    /// than spinning. A conversion takes only about 17 ADC clocks, so this
    /// pays off mainly at the slower [`SampleClock`]s or to let a
    /// low-priority task yield.
    pub async fn read(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        let gain2 = self.select(channel);
        let r = T::regs();
        r.convert().write(|w| {
            w.set_pga_gain2(gain2);
            w.set_start(true);
        });
        // Starting cleared the end-of-conversion flags, so IE_EOC asserts the
        // interrupt only once this conversion ends.
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            if !r.convert().read().start() {
                return Poll::Ready(());
            }
            r.ctrl_dma().write(|w| w.set_ie_eoc(true));
            unmask::<T>();
            if !r.convert().read().start() {
                r.ctrl_dma().write(|_| {});
                T::Interrupt::disable();
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;
        r.data().read().data()
    }

    /// Fill `buf` with conversions of `channel` spaced by `pace`, the DMA
    /// storing each result. Dropping the future stops the conversions.
    ///
    /// The ADC DMA moves results only while the core is awake (checked on
    /// CH592: one that ends while the core sleeps waits for a wake-up, and the
    /// timed conversions behind it with it). So rather than sleep until an
    /// interrupt, this keeps its task scheduled, as `yield_now` does, until
    /// the buffer is full: other tasks still run, and an executor that would
    /// sleep (`platform-riscv32`) stays awake until then.
    ///
    /// # Panics
    ///
    /// As [`Adc::blocking_read_dma`].
    pub async fn read_dma(&mut self, channel: &mut impl AdcChannel<T>, buf: &mut [u16], pace: Pace) {
        self.select(channel);
        let r = T::regs();
        let guard = OnDrop::new(|| stop_dma(r));
        start_dma(r, buf, pace, false);
        poll_fn(|cx| {
            if r.dma_if().read().if_dma_end() {
                return Poll::Ready(());
            }
            cx.waker().wake_by_ref();
            Poll::Pending
        })
        .await;
        drop(guard);
        compiler_fence(Ordering::SeqCst);
    }

    /// Convert `channel` without end, spaced by `pace`, the DMA writing round
    /// and round `buf`. Read the results through the returned
    /// [`RingBufferedAdc`]; dropping it stops the conversions.
    ///
    /// The front end is set up as [`Adc::convert`] sets it. The interrupt
    /// stays unmasked while the ring runs, to count the DMA's laps of `buf`.
    ///
    /// As for [`Adc::read_dma`], the DMA moves results only while the core is
    /// awake: with an executor that sleeps (`platform-riscv32`) conversions
    /// pause while it sleeps, and [`RingBufferedAdc::read`] keeps its task
    /// scheduled while it waits.
    ///
    /// # Panics
    ///
    /// As [`Adc::blocking_read_dma`].
    pub fn start_ring<'a>(
        &'a mut self,
        channel: &mut impl AdcChannel<T>,
        buf: &'a mut [u16],
        pace: Pace,
    ) -> RingBufferedAdc<'a, 'd, T> {
        self.select(channel);
        let r = T::regs();
        stop_dma(r);
        T::state().wraps.store(0, Ordering::Relaxed);
        start_dma(r, buf, pace, true);
        unmask::<T>();
        RingBufferedAdc {
            _adc: self,
            buf: buf.as_ptr(),
            len: buf.len(),
            beg: buf.as_ptr() as u16,
            pos: 0,
            lap: 0,
            _buf: PhantomData,
        }
    }
}

impl<'d, T: Instance, M: Mode> Adc<'d, T, M> {
    fn new_inner(adc: Peri<'d, T>, config: Config) -> Self {
        pac::TOUCHKEY.cfg().modify(|w| w.set_pwr_on(false));
        let r = T::regs();
        r.ctrl_dma().write(|_| {});
        r.cfg().write(|w| {
            w.set_power_on(true);
            w.set_clk_div(config.clock.bits());
        });
        Self {
            _peri: adc,
            config,
            last: None,
            _phantom: PhantomData,
        }
    }

    /// Change the gain for external channels.
    pub fn set_gain(&mut self, gain: Gain) {
        self.config.gain = gain;
    }

    pub fn gain(&self) -> Gain {
        self.config.gain
    }

    /// Power the temperature sensor and hand out its channel.
    pub fn enable_temperature(&mut self) -> Temperature {
        T::regs().tem_sensor().write(|w| w.set_tem_sen_pwr_on(true));
        Temperature
    }

    /// Hand out the battery-voltage channel.
    pub fn enable_vbat(&mut self) -> Vbat {
        Vbat
    }

    /// Convert one channel and return the raw 12-bit result.
    ///
    /// The front end follows the channel: VBAT runs at 1/4 gain with the
    /// buffer, the temperature sensor differentially at 2x without it, and
    /// external channels single-ended with [`Config::gain`] and
    /// [`Config::buffer`].
    pub fn convert(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        let gain2 = self.select(channel);
        Self::start_and_wait(gain2)
    }

    /// Fill `buf` with conversions of `channel` spaced by `pace`, the DMA
    /// storing each result, spinning until it is full.
    ///
    /// The front end is set up as [`Adc::convert`] sets it.
    ///
    /// # Panics
    ///
    /// If `buf` is empty or not in the first 32 KiB of RAM (all of RAM on
    /// CH592), or `pace` is `Timed(n)` with `n` outside `1..=256`.
    pub fn blocking_read_dma(&mut self, channel: &mut impl AdcChannel<T>, buf: &mut [u16], pace: Pace) {
        self.select(channel);
        let r = T::regs();
        start_dma(r, buf, pace, false);
        while !r.dma_if().read().if_dma_end() {}
        stop_dma(r);
        compiler_fence(Ordering::SeqCst);
    }

    /// Route `channel` to the converter with its front end; returns the
    /// `RB_ADC_PGA_GAIN2` bit to start conversions with.
    fn select(&mut self, channel: &mut impl AdcChannel<T>) -> bool {
        channel.setup();
        let ch = channel.channel();
        let (gain, buffer, diff) = match ch {
            CH_VBAT => (Gain::Quarter, true, false),
            CH_TEMP => (Gain::X2, false, true),
            _ => (self.config.gain, self.config.buffer, false),
        };
        self.front_end(ch, gain, buffer, diff)
    }

    /// Convert a differential pair: AIN0 - AIN2 or AIN1 - AIN3.
    ///
    /// 0x800 means 0 V; see [`Adc::to_millivolts_differential`].
    ///
    /// # Panics
    ///
    /// If `(pos, neg)` is not one of those pairs.
    pub fn convert_differential(&mut self, pos: &mut impl AdcChannel<T>, neg: &mut impl AdcChannel<T>) -> u16 {
        let (p, n) = (pos.channel(), neg.channel());
        if !matches!((p, n), (0, 2) | (1, 3)) {
            panic!("not a differential pair");
        }
        pos.setup();
        neg.setup();
        let gain2 = self.front_end(p, self.config.gain, false, true);
        Self::start_and_wait(gain2)
    }

    fn front_end(&mut self, ch: u8, gain: Gain, buffer: bool, diff: bool) -> bool {
        let r = T::regs();
        let (gain10, gain2) = gain.bits();
        let mut cfg = pac::adc::regs::Cfg(0);
        cfg.set_power_on(true);
        cfg.set_buf_en(buffer);
        cfg.set_diff_en(diff);
        cfg.set_pga_gain10(gain10);
        cfg.set_clk_div(self.config.clock.bits());

        // Only touch the front end when it changes: rewriting it costs about a
        // conversion's time. The datasheet suggests throwing away the first
        // result after a change.
        if self.last != Some((cfg.0, ch)) {
            self.last = Some((cfg.0, ch));
            r.channel().write(|w| w.set_ch_inx(ch));
            r.cfg().write_value(cfg);
            r.convert().write(|w| w.set_pga_gain2(gain2));
            Self::start_and_wait(gain2);
        }
        gain2
    }

    fn start_and_wait(gain2: bool) -> u16 {
        let r = T::regs();
        r.convert().write(|w| {
            w.set_pga_gain2(gain2);
            w.set_start(true);
        });
        while r.convert().read().start() {}
        r.data().read().data()
    }

    /// Measure the converter's offset, as WCH's `ADC_DataCalib_Rough`: add
    /// the result to raw single-ended readings of external channels.
    ///
    /// Runs on the current channel with the current front end, so convert
    /// the channel of interest once first.
    pub fn calibrate(&mut self) -> i16 {
        let r = T::regs();
        let cfg = r.cfg().read();
        let gain2 = r.convert().read().pga_gain2();
        r.cfg().modify(|w| {
            w.set_ofs_test(true);
            w.set_diff_en(false);
        });
        // In test mode the result comes out inverted.
        Self::start_and_wait(gain2);
        let mut sum = 0u32;
        for _ in 0..16 {
            sum += (!Self::start_and_wait(gain2) & ADC_MAX as u16) as u32;
        }
        r.cfg().write_value(cfg);
        self.last = None;
        MID as i16 - ((sum + 8) >> 4) as i16
    }

    /// Convert a raw single-ended reading taken at the current gain to
    /// millivolts, assuming Vref = [`VREF_MV`].
    pub fn to_millivolts(&self, raw: u16) -> i32 {
        single_ended_millivolts(raw, self.config.gain)
    }

    /// Convert a raw differential reading taken at the current gain to
    /// millivolts (positive minus negative input), assuming Vref = [`VREF_MV`].
    pub fn to_millivolts_differential(&self, raw: u16) -> i32 {
        let (p, q) = self.config.gain.inverse();
        VREF_MV as i32 * p * (raw as i32 - MID as i32) / (MID as i32 * q)
    }
}

impl<'d, T: Instance, M: Mode> Drop for Adc<'d, T, M> {
    fn drop(&mut self) {
        let r = T::regs();
        T::Interrupt::disable();
        stop_dma(r);
        r.tem_sensor().write(|_| {});
        r.cfg().modify(|w| w.set_power_on(false));
    }
}

/// Point the DMA at `buf` and start conversions, going round `buf` with
/// `ring`. The DMA-end interrupt is enabled, as the DMA-end flag needs it,
/// and so that [`InterruptHandler`] can stop the conversions in time (only
/// `Async` drivers unmask the interrupt in the PFIC).
fn start_dma(r: pac::adc::Adc, buf: &mut [u16], pace: Pace, ring: bool) {
    let start = buf.as_mut_ptr() as usize;
    let end = start + 2 * buf.len();
    if buf.is_empty() || !DMA_WINDOW.contains(&start) || end >= DMA_WINDOW.end {
        panic!("ADC DMA buffer must be in the first 32 KiB of RAM");
    }
    let cycle = match pace {
        Pace::Continuous => 0,
        Pace::Timed(n) if (1..=256).contains(&n) => (256 - n) as u8,
        Pace::Timed(_) => panic!("ADC pace out of range"),
    };
    r.ctrl_dma().write(|_| {});
    r.dma_if().write(|w| {
        w.set_if_dma_end(true);
        w.set_if_end(true);
    });
    r.auto_cycle().write(|w| w.set_auto_cycle(cycle));
    // The address fields hold bits 14:1: write byte addresses whole.
    r.dma_beg().write(|w| w.0 = start as u16);
    r.dma_end().write(|w| w.0 = end as u16);
    compiler_fence(Ordering::SeqCst);
    r.ctrl_dma().write(|w| {
        w.set_dma_enable(true);
        w.set_dma_loop(ring);
        w.set_ie_dma_end(true);
        match pace {
            Pace::Continuous => w.set_cont_en(true),
            Pace::Timed(_) => w.set_auto_en(true),
        }
    });
    // Back-to-back conversions wait for a first start; timed ones start
    // themselves after one interval.
    if pace == Pace::Continuous {
        r.convert().modify(|w| w.set_start(true));
    }
}

/// Stop conversions and the DMA, and clear their flags.
fn stop_dma(r: pac::adc::Adc) {
    r.ctrl_dma().write(|_| {});
    r.dma_if().write(|w| {
        w.set_if_dma_end(true);
        w.set_if_end(true);
    });
}

/// Results streaming into a circular buffer, from [`Adc::start_ring`].
///
/// The reader must keep up: once the DMA has gone a whole buffer ahead, the
/// next read returns [`OverrunError`] and reading resumes at the newest
/// result.
pub struct RingBufferedAdc<'a, 'd, T: Instance> {
    _adc: &'a mut Adc<'d, T, Async>,
    buf: *const u16,
    len: usize,
    /// `DMA_BEG`, the low half of `buf`'s address.
    beg: u16,
    /// Next index to read.
    pos: usize,
    /// Laps the DMA is ahead of the reader's lap: 0 or 1 unless overrun.
    lap: usize,
    _buf: PhantomData<&'a mut [u16]>,
}

impl<'a, 'd, T: Instance> RingBufferedAdc<'a, 'd, T> {
    /// Copy the results that have arrived since the last read into `out`,
    /// as many as fit, without waiting; returns how many.
    pub fn try_read(&mut self, out: &mut [u16]) -> Result<usize, OverrunError> {
        let ahead = self.ahead()?;
        let n = ahead.min(out.len());
        let start = self.pos;
        for (i, o) in out[..n].iter_mut().enumerate() {
            let j = (start + i) % self.len;
            *o = unsafe { self.buf.add(j).read_volatile() };
        }
        self.pos += n;
        if self.pos >= self.len {
            self.pos -= self.len;
            self.lap -= 1;
        }
        // The DMA must not have caught up with what was just copied.
        let ahead = self.ahead()?;
        if ahead + n > self.len {
            self.resync();
            return Err(OverrunError);
        }
        Ok(n)
    }

    /// Fill `out`, spinning until enough results have arrived.
    pub fn blocking_read(&mut self, out: &mut [u16]) -> Result<(), OverrunError> {
        let mut done = 0;
        while done < out.len() {
            done += self.try_read(&mut out[done..])?;
        }
        Ok(())
    }

    /// Results waiting to be read, or the overrun.
    fn ahead(&mut self) -> Result<usize, OverrunError> {
        let (laps, now) = position::<T>(self.beg);
        self.lap += laps as usize;
        let ahead = (self.lap * self.len + now).wrapping_sub(self.pos);
        if self.lap > 1 || ahead > self.len {
            self.resync();
            return Err(OverrunError);
        }
        Ok(ahead)
    }

    /// Carry on from the newest result.
    fn resync(&mut self) {
        let (_, now) = position::<T>(self.beg);
        self.pos = now;
        self.lap = 0;
    }

    /// Copy the results that have arrived into `out`, as many as fit,
    /// waiting until there is at least one; returns how many.
    ///
    /// The task stays scheduled while it waits, as for [`Adc::read_dma`].
    pub async fn read(&mut self, out: &mut [u16]) -> Result<usize, OverrunError> {
        if out.is_empty() {
            return Ok(0);
        }
        poll_fn(|cx| match self.try_read(out) {
            Ok(0) => {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
            res => Poll::Ready(res),
        })
        .await
    }
}

impl<'a, 'd, T: Instance> Drop for RingBufferedAdc<'a, 'd, T> {
    fn drop(&mut self) {
        T::Interrupt::disable();
        stop_dma(T::regs());
        compiler_fence(Ordering::SeqCst);
    }
}

/// Unmask the interrupt, dropping a request left from before the enables
/// were set (a level source asserts it again at once).
fn unmask<T: Instance>() {
    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };
}

/// DMA laps since the last call and the index it writes next, read
/// consistently.
fn position<T: Instance>(beg: u16) -> (u32, usize) {
    let r = T::regs();
    critical_section::with(|_| loop {
        // DMA_NOW and the flag change together at the wrap: read the flag on
        // both sides and retry if it changed in between.
        let f1 = r.dma_if().read().if_dma_end();
        let now = r.dma_now().read().0;
        let f2 = r.dma_if().read().if_dma_end();
        if f1 == f2 {
            if f1 {
                r.dma_if().write(|w| w.set_if_dma_end(true));
            }
            let laps = T::state().wraps.swap(0, Ordering::Relaxed) + f1 as u32;
            break (laps, (now.wrapping_sub(beg) / 2) as usize);
        }
    })
}

/// `Vin = Vref * (raw * p + 2048 * (q - p)) / (2048 * q)` for 1 / gain = p / q.
fn single_ended_millivolts(raw: u16, gain: Gain) -> i32 {
    let (p, q) = gain.inverse();
    VREF_MV as i32 * (raw as i32 * p + MID as i32 * (q - p)) / (MID as i32 * q)
}

/// Internal temperature sensor channel, from [`Adc::enable_temperature`].
#[non_exhaustive]
pub struct Temperature;

impl Temperature {
    /// Convert a raw reading to thousandths of a degree Celsius, with the
    /// factory calibration word, as WCH's `adc_to_temperature_celsius`
    /// (2.83 LSB/degC).
    pub fn to_millicelsius(raw: u16) -> i32 {
        let c25 = unsafe { ROM_CFG_TMP_25C.read_volatile() };
        let at = match c25 >> 16 {
            0 => 25,
            t => t as i32,
        };
        at * 1000 + (raw as i32 - (c25 & 0xffff) as i32) * 100_000 / 283
    }
}

/// Battery-voltage channel, from [`Adc::enable_vbat`].
#[non_exhaustive]
pub struct Vbat;

impl Vbat {
    /// Convert a raw reading to millivolts. VBAT is read at 1/4 gain.
    pub fn to_millivolts(raw: u16) -> u32 {
        single_ended_millivolts(raw, Gain::Quarter).max(0) as u32
    }
}

// ========
// ## Channels

pub(crate) trait SealedAdcChannel<T> {
    /// Prepare the channel's pin, if it has one.
    fn setup(&mut self) {}

    fn channel(&self) -> u8;
}

/// ADC channel: a pin or an internal channel.
#[allow(private_bounds)]
pub trait AdcChannel<T>: SealedAdcChannel<T> + Sized {
    /// Erase the channel's type.
    fn degrade_adc(mut self) -> AnyAdcChannel<T> {
        self.setup();
        AnyAdcChannel {
            channel: self.channel(),
            _phantom: PhantomData,
        }
    }
}

/// A type-erased ADC channel.
///
/// Its pin stays an analog input and is not reset when this is dropped.
pub struct AnyAdcChannel<T> {
    channel: u8,
    _phantom: PhantomData<T>,
}

impl<T: Instance> AdcChannel<T> for AnyAdcChannel<T> {}
impl<T: Instance> SealedAdcChannel<T> for AnyAdcChannel<T> {
    fn channel(&self) -> u8 {
        self.channel
    }
}

impl<T: Instance> AdcChannel<T> for Temperature {}
impl<T: Instance> SealedAdcChannel<T> for Temperature {
    fn channel(&self) -> u8 {
        CH_TEMP
    }
}

impl<T: Instance> AdcChannel<T> for Vbat {}
impl<T: Instance> SealedAdcChannel<T> for Vbat {
    fn channel(&self) -> u8 {
        CH_VBAT
    }
}

macro_rules! impl_adc_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::adc::AdcChannel<peripherals::$inst> for Peri<'_, peripherals::$pin> {}
        impl crate::adc::SealedAdcChannel<peripherals::$inst> for Peri<'_, peripherals::$pin> {
            fn setup(&mut self) {
                self.set_as_analog();
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

// ========
// ## Interrupt

/// ADC interrupt handler, for [`Adc::new_async`].
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Sources (checked on CH592): with RB_ADC_IE_EOC, RB_ADC_IF_END_ADC
        // (level); with RB_ADC_IE_DMA_END alone, a pulse when the DMA reaches
        // DMA_END; with neither, RB_ADC_IF_EOC (level, cleared only by
        // writing R8_ADC_CONVERT).
        let r = T::regs();
        let ctrl = r.ctrl_dma().read();
        let dma_end = r.dma_if().read().if_dma_end();
        if ctrl.dma_loop() {
            // A ring: count its wraps, and leave the interrupt unmasked for
            // the next one.
            if dma_end {
                T::state().wraps.fetch_add(1, Ordering::Relaxed);
                r.dma_if().write(|w| w.set_if_dma_end(true));
            }
        } else {
            // One conversion (`Adc::read`) has ended.
            r.ctrl_dma().modify(|w| w.set_ie_eoc(false));
            T::Interrupt::disable();
            T::state().waker.wake();
        }
    }
}

// ========
// ## Peripheral traits

struct State {
    waker: AtomicWaker,
    /// Ring wraps counted by the interrupt handler.
    wraps: AtomicU32,
}

trait SealedInstance {
    fn regs() -> pac::adc::Adc;
    fn state() -> &'static State;
}

/// ADC instance.
#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + 'static + Send {
    /// Interrupt for this ADC.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl SealedInstance for peripherals::ADC {
    fn regs() -> pac::adc::Adc {
        pac::ADC
    }

    fn state() -> &'static State {
        static STATE: State = State {
            waker: AtomicWaker::new(),
            wraps: AtomicU32::new(0),
        };
        &STATE
    }
}

impl Instance for peripherals::ADC {
    type Interrupt = crate::interrupt::typelevel::ADC;
}

impl_adc_pin!(ADC, PA4, 0);
impl_adc_pin!(ADC, PA5, 1);
impl_adc_pin!(ADC, PA12, 2);
impl_adc_pin!(ADC, PA13, 3);
impl_adc_pin!(ADC, PA14, 4);
impl_adc_pin!(ADC, PA15, 5);
impl_adc_pin!(ADC, PB0, 8);
impl_adc_pin!(ADC, PB6, 9);
impl_adc_pin!(ADC, PA6, 10);
impl_adc_pin!(ADC, PA7, 11);
impl_adc_pin!(ADC, PA8, 12);
impl_adc_pin!(ADC, PA9, 13);
