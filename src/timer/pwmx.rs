//! PWMX: the PWM4-PWM11 block.
//!
//! One counter and prescaler drive all channels: eight 8-bit channels
//! (PWM4-PWM11) with a 256/255, 128/127 or 64/63-clock period, or six 16-bit
//! channels (PWM4-PWM9) with any period up to 65536 clocks ([`Period`]).
//! Each channel has its own duty, polarity and output enable. While a
//! channel's output is disabled its pin is a GPIO output at the inactive
//! level.
//!
//! [`Pwmx::new`] sets the period and frequency; the `chN` methods hand out a
//! [`PwmxChannel`] per pin, in the spirit of ch32-hal's
//! `PwmPin::new_chN::<REMAP>`.
//!
//! # Pins (CH592)
//!
//! | REMAP | PWM4 | PWM5 | PWM6 | PWM7 | PWM8 | PWM9 | PWM10 | PWM11 |
//! |-------|------|------|------|------|------|------|-------|-------|
//! | 0     | PA12 | PA13 | PB0  | PB4  | PB6  | PB7  | PB14  | PB23  |
//! | 1     | PA6  | PA7  | -    | -    | -    | -    | -     | -     |
//!
//! # Pins (CH584/CH585)
//!
//! | REMAP | PWM4 | PWM5 | PWM6 | PWM7 | PWM8 | PWM9 | PWM10 | PWM11 |
//! |-------|------|------|------|------|------|------|-------|-------|
//! | 0     | PA12 | PA13 | PB0  | PB4  | PB6  | PB7  | PB14  | PB23  |
//! | 1     | PA6  | PA7  | -    | PB1  | PB2  | PB3  | -     | -     |
//!
//! The remapped channels share one remap bit: taking one of them moves the
//! others too.
//!
//! PB14 is the debug link's TIO, and PB23 the RST# input when `CFG_RESET_EN`
//! is set in the user config. CH592F bonds none of PB0, PB6, PA6 and PA7.

use core::marker::PhantomData;

use super::OutputPolarity;
use crate::gpio::{AfType, AnyPin, Speed};
use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};
use crate::time::Hertz;
use crate::{pac, peripherals, Peri};

/// PWM period, in prescaled clocks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Period {
    /// 8-bit duty.
    Cycles256,
    /// 8-bit duty; a duty of 255 is 100%.
    Cycles255,
    /// 7-bit duty.
    Cycles128,
    /// 7-bit duty; a duty of 127 is 100%.
    Cycles127,
    /// 6-bit duty.
    Cycles64,
    /// 6-bit duty; a duty of 63 is 100%.
    Cycles63,
    /// 16-bit duty, `2..=65536` clocks. Only PWM4-PWM9 work in this mode.
    Wide(u32),
}

impl Period {
    /// Clocks per period.
    pub fn cycles(self) -> u32 {
        match self {
            Self::Cycles256 => 256,
            Self::Cycles255 => 255,
            Self::Cycles128 => 128,
            Self::Cycles127 => 127,
            Self::Cycles64 => 64,
            Self::Cycles63 => 63,
            Self::Wide(n) => n,
        }
    }

    /// `(RB_PWM_CYC_MOD, RB_PWM_CYCLE_SEL)`.
    fn bits(self) -> (u8, bool) {
        match self {
            Self::Cycles256 => (0, false),
            Self::Cycles255 => (0, true),
            Self::Cycles128 => (1, false),
            Self::Cycles127 => (1, true),
            Self::Cycles64 => (2, false),
            Self::Cycles63 => (2, true),
            Self::Wide(_) => (3, false),
        }
    }

    fn is_wide(self) -> bool {
        matches!(self, Self::Wide(_))
    }

    /// Largest value the data register takes.
    fn max_data(self) -> u32 {
        match self {
            Self::Cycles256 | Self::Cycles255 => 0xff,
            Self::Cycles128 | Self::Cycles127 => 0x7f,
            Self::Cycles64 | Self::Cycles63 => 0x3f,
            Self::Wide(_) => 0xffff,
        }
    }
}

/// PWMX driver.
pub struct Pwmx<'d> {
    _peri: Peri<'d, peripherals::PWMX>,
    period: Period,
}

impl<'d> Pwmx<'d> {
    /// Set up the block with `period` clocks per PWM period, at `freq`.
    ///
    /// # Panics
    ///
    /// If a [`Period::Wide`] count is outside `2..=65536`.
    pub fn new(peri: Peri<'d, peripherals::PWMX>, period: Period, freq: Hertz) -> Self {
        set_clock_enabled(ClockGate::Pwmx, true);
        let r = pac::PWMX;
        r.out_en().write(|_| {});
        r.polar().write(|_| {});
        r.int_ctrl().write(|w| w.set_if_cyc(true));

        if let Period::Wide(n) = period {
            if !(2..=65536).contains(&n) {
                panic!("PWMX period out of range");
            }
            r.reg_cycle().write(|w| w.set_cyc_value((n - 1) as u16));
        }
        let (cyc_mod, cycle_sel) = period.bits();
        r.config().write(|w| {
            w.set_cyc_mod(cyc_mod);
            w.set_cycle_sel(cycle_sel);
        });

        let mut this = Self { _peri: peri, period };
        this.set_frequency(freq);
        this
    }

    /// Set the PWM frequency: the prescaler becomes HCLK / (`freq` * period),
    /// rounded, within `1..=255`. See [`Pwmx::frequency`] for the result.
    pub fn set_frequency(&mut self, freq: Hertz) {
        let clk = crate::rcc::clocks().sysclk.0 as u64;
        let per = freq.0 as u64 * self.period.cycles() as u64;
        let div = if per == 0 { 255 } else { (clk + per / 2) / per };
        pac::PWMX
            .clock_div()
            .write(|w| w.set_clock_div(div.clamp(1, 255) as u8));
    }

    /// The actual PWM frequency.
    pub fn frequency(&self) -> Hertz {
        let div = pac::PWMX.clock_div().read().clock_div() as u32;
        Hertz(crate::rcc::clocks().sysclk.0 / div / self.period.cycles())
    }

    /// The duty that gives a 100% duty cycle: the period in clocks.
    pub fn get_max_duty(&self) -> u32 {
        self.period.cycles()
    }

    fn channel<'a>(&'a self, ch: u8, pin: Peri<'a, AnyPin>) -> PwmxChannel<'a> {
        if self.period.is_wide() && ch > 9 {
            panic!("PWM10/PWM11 have no 16-bit mode");
        }
        crate::gpio::SealedPin::set_as_af(&*pin, AfType::output(Speed::Low));
        let mut c = PwmxChannel {
            pin,
            ch,
            period: self.period,
            phantom: PhantomData,
        };
        c.set_polarity(OutputPolarity::ActiveHigh);
        c.set_duty(0);
        c
    }
}

macro_rules! channel_fns {
    ($($fn_name:ident, $trait:ident, $ch:expr, $remap:expr;)*) => {
        impl<'d> Pwmx<'d> {
            $(
                #[doc = concat!("Take PWM", stringify!($ch), "'s pin, selecting its `REMAP`. The output starts disabled.")]
                pub fn $fn_name<'a, const REMAP: u8>(&'a self, pin: Peri<'a, impl $trait<REMAP>>) -> PwmxChannel<'a> {
                    if $remap {
                        set_remap(RemapGroup::Pwmx, REMAP);
                    }
                    self.channel($ch, pin.into())
                }
            )*
        }

        $(
            #[doc = concat!("A PWM", stringify!($ch), " pin.")]
            pub trait $trait<const REMAP: u8 = 0>: crate::gpio::Pin {}
        )*
    };
}

// Only PWM4/PWM5 (and PWM7-PWM9 on CH584/CH585) have alternate pins; the
// others must not touch the shared remap bit.
channel_fns! {
    ch4, Pwm4Pin, 4, true;
    ch5, Pwm5Pin, 5, true;
    ch6, Pwm6Pin, 6, false;
    ch7, Pwm7Pin, 7, cfg!(any(ch584, ch585));
    ch8, Pwm8Pin, 8, cfg!(any(ch584, ch585));
    ch9, Pwm9Pin, 9, cfg!(any(ch584, ch585));
    ch10, Pwm10Pin, 10, false;
    ch11, Pwm11Pin, 11, false;
}

/// One PWMX channel.
pub struct PwmxChannel<'a> {
    pin: Peri<'a, AnyPin>,
    ch: u8,
    period: Period,
    phantom: PhantomData<&'a Pwmx<'a>>,
}

impl<'a> PwmxChannel<'a> {
    fn bit(&self) -> u8 {
        1 << (self.ch - 4)
    }

    /// Enable the output.
    pub fn enable(&mut self) {
        let b = self.bit();
        critical_section::with(|_| pac::PWMX.out_en().modify(|w| w.0 |= b));
    }

    /// Disable the output; the pin goes to the idle level.
    pub fn disable(&mut self) {
        let b = self.bit();
        critical_section::with(|_| pac::PWMX.out_en().modify(|w| w.0 &= !b));
    }

    pub fn is_enabled(&self) -> bool {
        pac::PWMX.out_en().read().0 & self.bit() != 0
    }

    pub fn set_polarity(&mut self, polarity: OutputPolarity) {
        let b = self.bit();
        let active_low = polarity == OutputPolarity::ActiveLow;
        critical_section::with(|_| {
            pac::PWMX
                .polar()
                .modify(|w| if active_low { w.0 |= b } else { w.0 &= !b })
        });
        // The idle level, for while the output is disabled.
        if active_low {
            crate::gpio::SealedPin::set_high(&*self.pin);
        } else {
            crate::gpio::SealedPin::set_low(&*self.pin);
        }
    }

    /// The duty that gives a 100% duty cycle: the period in clocks.
    pub fn get_max_duty(&self) -> u32 {
        self.period.cycles()
    }

    /// Set the duty, `0..=get_max_duty()`. Clamped to the data register's
    /// width, so the 256-, 128- and 64-clock periods stop one clock short of
    /// 100%.
    pub fn set_duty(&mut self, duty: u32) {
        let duty = duty.min(self.period.max_data());
        let base = pac::PWMX.as_ptr() as *mut u8;
        let n = (self.ch - 4) as usize;
        unsafe {
            if self.period.is_wide() {
                // 16-bit data: PWM4-7 at 0x04.., PWM8/9 in R32_PWM_REG_DATA8.
                let off = if n < 4 { 0x04 + 2 * n } else { 0x10 + 2 * (n - 4) };
                (base.add(off) as *mut u16).write_volatile(duty as u16);
            } else {
                base.add(0x04 + n).write_volatile(duty as u8);
            }
        }
    }
}

impl<'a> Drop for PwmxChannel<'a> {
    fn drop(&mut self) {
        self.disable();
    }
}

impl<'a> embedded_hal::pwm::ErrorType for PwmxChannel<'a> {
    type Error = core::convert::Infallible;
}

impl<'a> embedded_hal::pwm::SetDutyCycle for PwmxChannel<'a> {
    fn max_duty_cycle(&self) -> u16 {
        self.period.cycles().min(u16::MAX as u32) as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let max = self.max_duty_cycle() as u32;
        self.set_duty(duty as u32 * self.period.cycles() / max);
        Ok(())
    }
}

macro_rules! impl_pwmx_pin {
    ($trait:ident, $pin:ident, $remap:expr) => {
        if_pin!($pin, impl $trait<$remap> for peripherals::$pin {});
    };
}

impl_pwmx_pin!(Pwm4Pin, PA12, 0);
impl_pwmx_pin!(Pwm4Pin, PA6, 1);
impl_pwmx_pin!(Pwm5Pin, PA13, 0);
impl_pwmx_pin!(Pwm5Pin, PA7, 1);
impl_pwmx_pin!(Pwm6Pin, PB0, 0);
impl_pwmx_pin!(Pwm7Pin, PB4, 0);
impl_pwmx_pin!(Pwm8Pin, PB6, 0);
impl_pwmx_pin!(Pwm9Pin, PB7, 0);
impl_pwmx_pin!(Pwm10Pin, PB14, 0);
impl_pwmx_pin!(Pwm11Pin, PB23, 0);
#[cfg(any(ch584, ch585))]
impl_pwmx_pin!(Pwm7Pin, PB1, 1);
#[cfg(any(ch584, ch585))]
impl_pwmx_pin!(Pwm8Pin, PB2, 1);
#[cfg(any(ch584, ch585))]
impl_pwmx_pin!(Pwm9Pin, PB3, 1);
