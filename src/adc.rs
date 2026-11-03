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
//! # Channels (CH592)
//!
//! | AIN | 0   | 1   | 2    | 3    | 4    | 5    | 8   | 9   | 10  | 11  | 12  | 13  |
//! |-----|-----|-----|------|------|------|------|-----|-----|-----|-----|-----|-----|
//! | Pin | PA4 | PA5 | PA12 | PA13 | PA14 | PA15 | PB0 | PB6 | PA6 | PA7 | PA8 | PA9 |
//!
//! Channel 14 is VBAT and 15 the temperature sensor; 6 and 7 do not exist.

use core::marker::PhantomData;

use crate::gpio::SealedPin as _;
use crate::{pac, peripherals, Peri};

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

/// ADC driver.
pub struct Adc<'d, T: Instance> {
    _peri: Peri<'d, T>,
    config: Config,
    /// `(CFG, CHANNEL)` of the last conversion, to spot a front-end change.
    last: Option<(u8, u8)>,
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Power the ADC up with `config`. Turns the touch-key block off, which
    /// shares the converter.
    pub fn new(adc: Peri<'d, T>, config: Config) -> Self {
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
        channel.setup();
        let ch = channel.channel();
        let (gain, buffer, diff) = match ch {
            CH_VBAT => (Gain::Quarter, true, false),
            CH_TEMP => (Gain::X2, false, true),
            _ => (self.config.gain, self.config.buffer, false),
        };
        self.run(ch, gain, buffer, diff)
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
        self.run(p, self.config.gain, false, true)
    }

    fn run(&mut self, ch: u8, gain: Gain, buffer: bool, diff: bool) -> u16 {
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
        Self::start_and_wait(gain2)
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

impl<'d, T: Instance> Drop for Adc<'d, T> {
    fn drop(&mut self) {
        let r = T::regs();
        r.tem_sensor().write(|_| {});
        r.cfg().modify(|w| w.set_power_on(false));
    }
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
// ## Peripheral traits

trait SealedInstance {
    fn regs() -> pac::adc::Adc;
}

/// ADC instance.
#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + 'static + Send {}

impl SealedInstance for peripherals::ADC {
    fn regs() -> pac::adc::Adc {
        pac::ADC
    }
}

impl Instance for peripherals::ADC {}

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
