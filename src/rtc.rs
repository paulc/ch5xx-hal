//! Real-time clock (CH592)
//!
//! The CH5xx RTC is a free-running counter clocked by CK32K (the internal
//! 32 kHz RC, or a 32.768 kHz crystal on PA10/PA11), not a calendar:
//!
//! - `CNT_32K` counts 0..=65535 in 2 s, `CNT_2S` counts 2 s units within a day
//!   (0..43200) and `CNT_DAY` counts days (14 bits). [`Rtc::now`] joins them
//!   into ticks of 1/32768 s, wrapping after 16384 days.
//! - An alarm ("trigger mode") matches the time of day, to the tick.
//! - A periodic flag ("timing mode") fires every 1/8 s to 16 s.
//!
//! The counter keeps running across resets other than power-on, and
//! [`Rtc::new`] leaves it alone; [`Rtc::set_ticks`] sets it.
//!
//! Both flags raise the `RTC` interrupt; this driver only polls them.

use crate::sys::safe_access;
use crate::{pac, peripherals, Peri};

/// Counter ticks per second.
pub const TICKS_PER_SECOND: u64 = 32_768;

/// Counter ticks per day.
pub const TICKS_PER_DAY: u64 = 86_400 * TICKS_PER_SECOND;

/// Days the counter holds before wrapping.
const DAYS: u64 = 1 << 14;

/// CK32K source.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// Internal 32 kHz RC (LSI).
    #[default]
    Lsi,
    /// External 32.768 kHz crystal (LSE) on PA10/PA11, which must be left as
    /// floating inputs. Selecting [`ClockSource::Lsi`] stops it again.
    Lse,
}

/// RTC configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Default)]
pub struct Config {
    pub clock_source: ClockSource,
}

/// Period of the periodic flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Periodic {
    Ms125,
    Ms250,
    Ms500,
    S1,
    S2,
    S4,
    S8,
    S16,
}

/// Error from [`Rtc::new`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The 32.768 kHz crystal did not start; the RTC stays on the LSI.
    LseNotRunning,
}

/// RTC driver.
pub struct Rtc<'d> {
    _peri: Peri<'d, peripherals::RTC>,
}

impl<'d> Rtc<'d> {
    /// Select the clock source. The counter keeps its value.
    ///
    /// Starting the crystal blocks for about 500 ms.
    pub fn new(peri: Peri<'d, peripherals::RTC>, config: Config) -> Result<Self, Error> {
        let this = Self { _peri: peri };
        let sys = pac::SYS;
        match config.clock_source {
            ClockSource::Lsi => {
                safe_access(|| {
                    sys.ck32k_config().modify(|w| w.set_clk_int32k_pon(true));
                });
                safe_access(|| {
                    sys.ck32k_config().modify(|w| w.set_clk_osc32k_xt(false));
                });
                // Stop the crystal oscillator, which holds PA10/PA11 while on.
                crate::delay::Delay.delay_us(100);
                safe_access(|| {
                    sys.ck32k_config().modify(|w| w.set_clk_xt32k_pon(false));
                });
            }
            ClockSource::Lse => {
                // Start at the highest bias current, as the datasheet
                // suggests, and drop to the rated current once it runs.
                safe_access(|| sys.xt32k_tune().modify(|w| w.set_xt32k_i_tune(0b11)));
                safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_xt32k_pon(true)));
                crate::delay::Delay.delay_ms(500);
                safe_access(|| sys.xt32k_tune().modify(|w| w.set_xt32k_i_tune(0b01)));
                safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_osc32k_xt(true)));
                crate::delay::Delay.delay_us(100);
                if !this.clock_running() {
                    safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_osc32k_xt(false)));
                    safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_xt32k_pon(false)));
                    return Err(Error::LseNotRunning);
                }
            }
        }
        Ok(this)
    }

    /// The selected clock source.
    pub fn clock_source(&self) -> ClockSource {
        if pac::SYS.ck32k_config().read().clk_osc32k_xt() {
            ClockSource::Lse
        } else {
            ClockSource::Lsi
        }
    }

    /// Whether CK32K toggles: watch its pin status for a few 32 kHz periods.
    fn clock_running(&self) -> bool {
        let cfg = pac::SYS.ck32k_config();
        let first = cfg.read()._32k_clk_pin();
        let mut d = crate::delay::Delay;
        for _ in 0..200 {
            d.delay_us(1);
            if cfg.read()._32k_clk_pin() != first {
                return true;
            }
        }
        false
    }

    /// Time of day, `CNT_2S:CNT_32K`, read twice for a consistent value.
    fn time_of_day() -> u32 {
        let p = (pac::RTC.as_ptr() as *const u8).wrapping_add(0x38) as *const u32;
        loop {
            let a = unsafe { p.read_volatile() };
            if a == unsafe { p.read_volatile() } {
                return a;
            }
        }
    }

    /// The counter, in ticks of 1/32768 s.
    pub fn now(&self) -> u64 {
        let r = pac::RTC;
        loop {
            let day = r.cnt_day().read().cnt_day() as u64;
            let tod = Self::time_of_day() as u64;
            // A day boundary between the reads shows as a changed day.
            if r.cnt_day().read().cnt_day() as u64 == day {
                return day * TICKS_PER_DAY + tod;
            }
        }
    }

    /// Set the counter, in ticks of 1/32768 s (modulo 16384 days).
    pub fn set_ticks(&mut self, ticks: u64) {
        let ticks = ticks % (DAYS * TICKS_PER_DAY);
        let day = (ticks / TICKS_PER_DAY) as u32;
        let tod = (ticks % TICKS_PER_DAY) as u32;
        let r = pac::RTC;

        // Load just after a CK32K rising edge, as WCH's `RTC_InitTime`.
        let cfg = pac::SYS.ck32k_config();
        while cfg.read()._32k_clk_pin() {}
        while !cfg.read()._32k_clk_pin() {}

        safe_access(|| {
            r.trig().write(|w| w.set_trig(day));
            r.mode_ctrl().modify(|w| w.set_load_hi(true));
        });
        while r.cnt_day().read().cnt_day() as u32 != day {}
        safe_access(|| {
            r.trig().write(|w| w.set_trig(tod));
            r.mode_ctrl().modify(|w| w.set_load_lo(true));
        });
    }

    /// Raise the alarm flag when the time of day reaches that of `at`
    /// (ticks, as [`Rtc::now`]). The day is not compared.
    pub fn set_alarm(&mut self, at: u64) {
        let tod = (at % TICKS_PER_DAY) as u32;
        let r = pac::RTC;
        safe_access(|| {
            r.trig().write(|w| w.set_trig(tod));
            r.mode_ctrl().modify(|w| w.set_trig_en(true));
        });
        self.clear_alarm();
    }

    /// Stop the alarm.
    pub fn disable_alarm(&mut self) {
        safe_access(|| pac::RTC.mode_ctrl().modify(|w| w.set_trig_en(false)));
    }

    pub fn is_alarm_pending(&self) -> bool {
        pac::RTC.flag_ctrl().read().trig_flag()
    }

    pub fn clear_alarm(&mut self) {
        pac::RTC.flag_ctrl().write(|w| w.set_trig_clr(true));
    }

    /// Start (or with `None`, stop) the periodic flag.
    pub fn set_periodic(&mut self, period: Option<Periodic>) {
        let r = pac::RTC;
        safe_access(|| r.mode_ctrl().modify(|w| w.set_tmr_en(false)));
        if let Some(p) = period {
            safe_access(|| {
                r.mode_ctrl().modify(|w| {
                    w.set_tmr_mode(p as u8);
                    w.set_tmr_en(true);
                })
            });
            self.clear_periodic();
        }
    }

    pub fn is_periodic_pending(&self) -> bool {
        pac::RTC.flag_ctrl().read().tmr_flag()
    }

    pub fn clear_periodic(&mut self) {
        pac::RTC.flag_ctrl().write(|w| w.set_tmr_clr(true));
    }
}
