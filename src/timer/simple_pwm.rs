//! PWM output on a `TMRx` timer.
//!
//! Follows ch32-hal's `SimplePwm`, cut down to the one channel a CH5xx timer
//! has: [`PwmPin::new`] takes the pin with its `REMAP`, and the channel
//! arguments of `enable`/`set_duty`/... are gone.
//!
//! The period is `CNT_END` HCLK cycles (up to 2^26) and the duty comes from
//! the timer's 8-entry FIFO: each value is used for one period, and the last
//! one is kept once the FIFO runs dry. [`SimplePwm::set_duty`] queues a value,
//! so it takes effect at the start of the next period; if eight are already
//! queued it waits for room (at most one period) rather than drop the value.
//!
//! While the output is disabled the pin is a GPIO output at the inactive
//! level.

use core::marker::PhantomData;

use super::{Instance, OutputPolarity, TimerPin};
use crate::gpio::{AfType, AnyPin, Speed};
use crate::time::Hertz;
use crate::Peri;

/// Largest period, in timer clocks.
const MAX_PERIOD: u32 = (1 << 26) - 1;

/// Duty FIFO depth.
const FIFO_SIZE: u8 = 8;

/// The timer's pin, as a PWM output.
pub struct PwmPin<'d, T> {
    pin: Peri<'d, AnyPin>,
    phantom: PhantomData<T>,
}

impl<'d, T: Instance> PwmPin<'d, T> {
    /// Take `pin` as `T`'s PWM output, selecting its `REMAP`.
    pub fn new<const REMAP: u8>(pin: Peri<'d, impl TimerPin<T, REMAP>>) -> Self {
        apply_remap!();
        crate::gpio::SealedPin::set_as_af(&*pin, AfType::output(Speed::Low));
        crate::gpio::SealedPin::set_low(&*pin);
        Self {
            pin: pin.into(),
            phantom: PhantomData,
        }
    }
}

/// PWM driver on a `TMRx` timer.
pub struct SimplePwm<'d, T: Instance> {
    _peri: Peri<'d, T>,
    pin: PwmPin<'d, T>,
    period: u32,
    duty: u32,
}

impl<'d, T: Instance> SimplePwm<'d, T> {
    /// Set up PWM at `freq` with a 0 duty cycle, output disabled.
    pub fn new(tim: Peri<'d, T>, pin: PwmPin<'d, T>, freq: Hertz) -> Self {
        T::enable();
        let r = T::regs();
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| w.set_all_clear(true));
        r.pwm_mod().write(|_| {});

        let mut this = Self {
            _peri: tim,
            pin,
            period: 0,
            duty: 0,
        };
        this.set_frequency(freq);
        r.ctrl_mod().write(|_| {});
        r.fifo().write(|w| w.set_fifo(0));
        r.ctrl_mod().modify(|w| w.set_count_en(true));
        this
    }

    /// Enable the output.
    pub fn enable(&mut self) {
        T::regs().ctrl_mod().modify(|w| w.set_out_en(true));
    }

    /// Disable the output; the pin goes to the idle level.
    pub fn disable(&mut self) {
        T::regs().ctrl_mod().modify(|w| w.set_out_en(false));
    }

    pub fn is_enabled(&self) -> bool {
        T::regs().ctrl_mod().read().out_en()
    }

    /// Set the PWM frequency. The duty cycle is kept as a fraction.
    ///
    /// Restarts the counter (writing `CNT_END` clears it).
    ///
    /// # Panics
    ///
    /// If `freq` is 0 or above HCLK / 2.
    pub fn set_frequency(&mut self, freq: Hertz) {
        let clk = crate::rcc::clocks().sysclk.0;
        if freq.0 == 0 || freq.0 > clk / 2 {
            panic!("PWM frequency out of range");
        }
        let period = (clk / freq.0).min(MAX_PERIOD);
        let duty = if self.period == 0 {
            0
        } else {
            (self.duty as u64 * period as u64 / self.period as u64) as u32
        };
        self.period = period;
        T::regs().cnt_end().write(|w| w.set_cnt_end(period));
        self.set_duty(duty);
    }

    /// The duty value that gives a 100% duty cycle: the period in timer
    /// clocks.
    pub fn get_max_duty(&self) -> u32 {
        self.period
    }

    /// Set the duty, `0..=get_max_duty()`. Larger values are clamped.
    pub fn set_duty(&mut self, duty: u32) {
        let duty = duty.min(self.period);
        self.duty = duty;
        let r = T::regs();
        // A write to a full FIFO is dropped.
        while r.fifo_count().read().fifo_count() >= FIFO_SIZE {}
        r.fifo().write(|w| w.set_fifo(duty));
    }

    /// The duty last set.
    pub fn get_duty(&self) -> u32 {
        self.duty
    }

    pub fn set_polarity(&mut self, polarity: OutputPolarity) {
        let active_low = polarity == OutputPolarity::ActiveLow;
        T::regs().ctrl_mod().modify(|w| w.set_out_polar(active_low));
        // The idle level, for while the output is disabled.
        if active_low {
            crate::gpio::SealedPin::set_high(&*self.pin.pin);
        } else {
            crate::gpio::SealedPin::set_low(&*self.pin.pin);
        }
    }
}

impl<'d, T: Instance> Drop for SimplePwm<'d, T> {
    fn drop(&mut self) {
        T::regs().ctrl_mod().write(|w| w.set_all_clear(true));
    }
}

impl<'d, T: Instance> embedded_hal::pwm::ErrorType for SimplePwm<'d, T> {
    type Error = core::convert::Infallible;
}

/// Duty is scaled when the period is above `u16::MAX`.
impl<'d, T: Instance> embedded_hal::pwm::SetDutyCycle for SimplePwm<'d, T> {
    fn max_duty_cycle(&self) -> u16 {
        self.period.min(u16::MAX as u32) as u16
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let max = self.max_duty_cycle() as u32;
        let duty = (duty as u64 * self.period as u64 / max as u64) as u32;
        self.set_duty(duty);
        Ok(())
    }
}
