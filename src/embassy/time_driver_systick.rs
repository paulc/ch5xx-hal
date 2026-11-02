//! SysTick-based time driver.
//!
//! The counter is the one [`crate::delay`] starts: 64-bit, up-counting at HCLK,
//! never reset or reloaded. This driver only adds the compare interrupt.

use core::cell::RefCell;
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
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    num: AtomicU32::new(1),
    den: AtomicU32::new(1), // avoid div by zero
    den_shift: AtomicU32::new(0),
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
        r.stk_cmphr().write(|w| w.set_cmph(u32::MAX));
        r.stk_cmplr().write(|w| w.set_cmpl(u32::MAX));
        r.stk_sr().write(|w| w.set_cntif(false));
    }

    fn on_interrupt(&self) {
        let r = pac::SYSTICK;
        // Count value compare flag
        r.stk_sr().write(|w| w.set_cntif(false)); // clear IF

        critical_section::with(|cs| {
            self.trigger_alarm(cs);
        });
    }

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
