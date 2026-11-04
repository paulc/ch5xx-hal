//! SysTick-based time driver.
//!
//! The counter is the one [`crate::delay`] starts: up-counting at HCLK, never
//! reset or reloaded. This driver only adds the compare interrupt.
//!
//! On CH592 the counter and compare are 64 bits. On CH584/CH585 they are 32
//! bits, wrapping every 55 s at 78 MHz, so the driver extends the count in
//! software, as embassy-stm32 does for its 16-bit timers: `period` counts
//! half-wraps, and the compare is always armed for the next half-wrap or the
//! next alarm, whichever comes first, so the interrupt keeps `period` current.
//! The compare fires on `CNT == CMP` only (checked on a CH584), so a compare
//! value just passed does not fire until the counter comes round again; the
//! arming code checks for that.

use core::cell::RefCell;
#[cfg(systick_ch585)]
use core::sync::atomic::compiler_fence;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::{Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;
use qingke::interrupt::Priority;
use qingke_rt::interrupt;

use crate::pac;

#[interrupt(core)]
fn SysTick() {
    DRIVER.on_interrupt();
}

pub struct SystickDriver {
    /// `ticks = cnt * num / den`: `TICK_HZ / HCLK` reduced by their GCD, so
    /// the conversion is exact even when HCLK is not a whole multiple of
    /// `TICK_HZ` (6.4 MHz against 1 MHz is 5/32), using only 64-bit division.
    num: AtomicU32,
    den: AtomicU32,
    /// `log2(den)` when `den` is a power of two (every HSE-divided HCLK, and
    /// 6.4 MHz), else `u32::MAX`: lets the hot path shift instead of divide.
    den_shift: AtomicU32,
    /// 32-bit counter only: half-wraps so far. Even while the counter's top
    /// bit is clear, odd while it is set, give or take an interrupt that has
    /// yet to run.
    #[cfg(systick_ch585)]
    period: AtomicU32,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    num: AtomicU32::new(1),
    den: AtomicU32::new(1), // avoid div by zero
    den_shift: AtomicU32::new(0),
    #[cfg(systick_ch585)]
    period: AtomicU32::new(0),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

const fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

impl SystickDriver {
    fn init(&'static self, _cs: critical_section::CriticalSection) {
        let r = pac::SYSTICK;
        let hz = crate::rcc::clocks().sysclk.0 as u64;

        let g = gcd(TICK_HZ, hz);
        self.num.store((TICK_HZ / g) as u32, Ordering::Relaxed);
        self.den.store((hz / g) as u32, Ordering::Relaxed);
        let den = hz / g;
        self.den_shift.store(
            if den.is_power_of_two() { den.trailing_zeros() } else { u32::MAX },
            Ordering::Relaxed,
        );

        // The delay driver already runs the counter: up-count, no reload,
        // HCLK. Only make sure the compare cannot fire before the first alarm.
        r.stk_ctlr().modify(|w| {
            w.set_stie(false);
            w.set_stre(false);
            w.set_mode(false);
            w.set_stclk(true);
            w.set_ste(true);
        });
        #[cfg(systick_ch583)]
        {
            r.stk_cmphr().write(|w| w.set_cmph(u32::MAX));
            r.stk_cmplr().write(|w| w.set_cmpl(u32::MAX));
            r.stk_sr().write(|w| w.set_cntif(false));
        }
        // The counter already ran since `delay` started it, but for less than
        // half a wrap, so `period` 0 is right. Arm the first half-wrap.
        #[cfg(systick_ch585)]
        while !self.set_alarm(_cs, u64::MAX) {}
    }

    fn on_interrupt(&self) {
        let r = pac::SYSTICK;
        // Count value compare flag
        r.stk_sr().write(|w| w.set_cntif(false)); // clear IF

        critical_section::with(|cs| {
            #[cfg(systick_ch585)]
            self.update_period(cs);
            self.trigger_alarm(cs);
        });
    }

    #[cfg(systick_ch583)]
    #[inline]
    fn raw_cnt(&self) -> u64 {
        let r = pac::SYSTICK;
        loop {
            let high = r.stk_cnth().read().cnth();
            let low = r.stk_cntl().read().cntl();
            if r.stk_cnth().read().cnth() == high {
                return (u64::from(high) << 32) | u64::from(low);
            }
        }
    }

    /// The counter, extended to 64 bits by `period`. Correct as long as
    /// `period` lags the counter by less than half a wrap, which the
    /// half-wrap compare keeps true (embassy-stm32's `calc_now`).
    #[cfg(systick_ch585)]
    #[inline]
    fn raw_cnt(&self) -> u64 {
        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let cnt = pac::SYSTICK.stk_cntl().read().cntl();
        ((period as u64) << 31) + (cnt ^ ((period & 1) << 31)) as u64
    }

    /// Count the half-wraps the counter has made since `period` last moved.
    #[cfg(systick_ch585)]
    fn update_period(&self, _cs: CriticalSection) {
        let period = self.period.load(Ordering::Relaxed);
        let cnt = pac::SYSTICK.stk_cntl().read().cntl();
        if cnt >> 31 != period & 1 {
            self.period.store(period.wrapping_add(1), Ordering::Relaxed);
        }
    }

    fn cnt_to_ticks(&self, cnt: u64) -> u64 {
        let num = self.num.load(Ordering::Relaxed) as u64;
        let shift = self.den_shift.load(Ordering::Relaxed);
        if shift != u32::MAX {
            // Split so `cnt * num` cannot overflow.
            let mask = (1u64 << shift) - 1;
            return (cnt >> shift) * num + (((cnt & mask) * num) >> shift);
        }
        let den = self.den.load(Ordering::Relaxed) as u64;
        if num == 1 {
            return cnt / den;
        }
        (cnt / den) * num + (cnt % den) * num / den
    }

    /// The first count at or after `ticks`.
    fn ticks_to_cnt(&self, ticks: u64) -> u64 {
        let num = self.num.load(Ordering::Relaxed) as u64;
        let den = self.den.load(Ordering::Relaxed) as u64;
        if num == 1 {
            return ticks.saturating_mul(den);
        }
        (ticks / num).saturating_mul(den).saturating_add(((ticks % num) * den).div_ceil(num))
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    #[cfg(systick_ch583)]
    fn set_alarm(&self, _cs: CriticalSection, next_alarm_ticks: u64) -> bool {
        let r = pac::SYSTICK;

        if next_alarm_ticks == u64::MAX {
            // Nothing queued: disarm.
            r.stk_ctlr().modify(|w| w.set_stie(false));
            r.stk_sr().write(|w| w.set_cntif(false));
            return true;
        }

        let next_alarm_cnt = self.ticks_to_cnt(next_alarm_ticks);
        if next_alarm_cnt <= self.raw_cnt() {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            return false;
        }

        r.stk_cmphr().write(|w| w.set_cmph((next_alarm_cnt >> 32) as u32));
        r.stk_cmplr().write(|w| w.set_cmpl(next_alarm_cnt as u32));
        r.stk_sr().write(|w| w.set_cntif(false));
        r.stk_ctlr().modify(|w| w.set_stie(true));

        if next_alarm_cnt <= self.raw_cnt() {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            r.stk_ctlr().modify(|w| w.set_stie(false));
            r.stk_sr().write(|w| w.set_cntif(false));
            return false;
        }

        true
    }

    /// Arm the compare for the alarm or the next half-wrap, whichever comes
    /// first. Returns `false` if the alarm has already passed; the half-wrap
    /// is still armed then.
    #[cfg(systick_ch585)]
    fn set_alarm(&self, cs: CriticalSection, next_alarm_ticks: u64) -> bool {
        let r = pac::SYSTICK;

        let alarm_cnt = if next_alarm_ticks == u64::MAX {
            u64::MAX
        } else {
            self.ticks_to_cnt(next_alarm_ticks)
        };

        loop {
            self.update_period(cs);
            let half_wrap = (self.period.load(Ordering::Relaxed) as u64 + 1) << 31;
            let target = alarm_cnt.min(half_wrap);
            r.stk_cmplr().write(|w| w.set_cmpl(target as u32));
            r.stk_sr().write(|w| w.set_cntif(false));
            r.stk_ctlr().modify(|w| w.set_stie(true));

            // Equality compare: a target the counter reached before (or while)
            // it was written will not fire until the next wrap. A match that
            // happened after the write left `CNTIF` set, and the interrupt
            // will run.
            if self.raw_cnt() < target || r.stk_sr().read().cntif() {
                return true;
            }
            if target == alarm_cnt {
                // The alarm has passed; still arm the half-wrap.
                let half_wrap = (self.period.load(Ordering::Relaxed) as u64 + 1) << 31;
                r.stk_cmplr().write(|w| w.set_cmpl(half_wrap as u32));
                if self.raw_cnt() >= half_wrap {
                    // Passed it too while handling the alarm: go round again.
                    self.update_period(cs);
                    continue;
                }
                return false;
            }
            // Missed the half-wrap itself: `update_period` above picks it up.
        }
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        self.cnt_to_ticks(self.raw_cnt())
    }

    fn schedule_wake(&self, ticks: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(ticks, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs);
    use qingke_rt::CoreInterrupt;

    // enable interrupt
    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15 as u8);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}
