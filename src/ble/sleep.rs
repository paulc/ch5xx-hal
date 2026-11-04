//! Sleep between BLE events, with [`crate::embassy::Executor`].
//!
//! The executor runs TMOS whenever it has polled its tasks
//! (`executor_idle`). When TMOS has nothing to do it calls the library's
//! idle callback (`idle`) with the clock count of its next event, having
//! powered the radio down; the callback's return value says whether the
//! chip slept (0: the library re-initialises the radio) or not (the library
//! powers it back up). As WCH's `CH58x_LowPower` (`HAL/SLEEP.c`), `idle`
//! sleeps until shortly before that event, or before the next embassy timer
//! if that comes first:
//!
//! - It sleeps only if no task is ready and the wake-up is at least
//!   [`WAKE_EARLY`] + [`SLEEP_MIN`] away. The RTC trigger ends the sleep
//!   [`WAKE_EARLY`] early, for the 32 MHz crystal to restart (which
//!   [`pwr::sleep`] waits out on the HSI). What is left until the event
//!   passes in the short wait below.
//! - Otherwise it waits in `wfi` for an interrupt: the RTC trigger at the
//!   TMOS event, the SysTick alarm, the radio, a peripheral. So it does
//!   while sleep is off ([`set_enabled`]).
//!
//! SysTick stops in sleep, and [`pwr::sleep`] puts it back as it was. The
//! callback then moves it on by the time slept, as the RTC measures it, so
//! embassy time keeps step with the RTC across sleeps. It reads both
//! counters straight after an RTC tick before and after the sleep, so the
//! RTC's 30.5 us resolution does not add up.
//!
//! The RTC wake-up is on from [`super::init`]: a later
//! [`pwr::configure_wake`] must keep `rtc` set. A UART transmission must
//! finish (`blocking_flush`) before the executor can sleep: the UART clock
//! stops in sleep.

use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::CriticalSection;
use embassy_time_driver::TICK_HZ;

use super::ffi::TMOS_SystemProcess;
use super::{clock, count, CLOCK_HZ, RTC_MAX_COUNT, STATS};
use crate::embassy::time_driver_impl as time_driver;
use crate::interrupt::{Interrupt, InterruptExt};
use crate::pac;
use crate::pwr::{self, SleepConfig};
use crate::sys::safe_access;

const fn us_to_rtc(us: u32) -> u32 {
    (us as u64 * CLOCK_HZ as u64 / 1_000_000) as u32
}

/// How long before an event the sleep ends: [`pwr::sleep`] runs 1.6 ms on
/// the HSI while the crystal restarts (`WAKE_UP_RTC_MAX_TIME` in WCH's
/// `SLEEP.h`), plus time to spare for the wake-up and the radio re-init.
pub const WAKE_EARLY: u32 = us_to_rtc(2000);
/// The shortest sleep worth taking (`SLEEP_RTC_MIN_TIME`).
pub const SLEEP_MIN: u32 = us_to_rtc(1000);

/// [`set_enabled`].
static ENABLED: AtomicBool = AtomicBool::new(true);
/// Set while the executor runs TMOS: only then may [`idle`] sleep.
static SLEEP_ALLOWED: AtomicBool = AtomicBool::new(false);
/// Set at the end of [`super::init`].
pub(super) static STARTED: AtomicBool = AtomicBool::new(false);

/// Turn on the RTC wake-up and trigger, as WCH's `HAL_SleepInit`.
pub(super) fn init() {
    safe_access(|| {
        pac::SYS.slp_wake_ctrl().modify(|w| {
            w.set_slp_rtc_wake(true);
            w.set_wake_ev_mode(true);
        })
    });
    safe_access(|| pac::RTC.mode_ctrl().modify(|w| w.set_trig_en(true)));
    pac::RTC.flag_ctrl().write(|w| w.set_trig_clr(true));
    unsafe { Interrupt::RTC.enable() };
}

/// Allow the sleep mode between events (the default), or only `wfi`: while
/// a debugger is attached, say, which loses the chip in sleep, or while a
/// peripheral that stops in sleep (a UART, USB) is busy.
pub fn set_enabled(enabled: bool) {
    ENABLED.store(enabled, Ordering::Relaxed);
}

/// The executor's idle step: one TMOS pass, which may sleep. Returns false
/// before [`super::init`], when the executor waits by itself.
pub(crate) fn executor_idle() -> bool {
    if !STARTED.load(Ordering::Relaxed) {
        return false;
    }
    SLEEP_ALLOWED.store(true, Ordering::Relaxed);
    unsafe { TMOS_SystemProcess() };
    SLEEP_ALLOWED.store(false, Ordering::Relaxed);
    count(&STATS.tmos_passes);
    true
}

/// `a - b` on the TMOS clock, which wraps at [`RTC_MAX_COUNT`].
fn diff(a: u32, b: u32) -> u32 {
    if a >= b {
        a - b
    } else {
        a + (RTC_MAX_COUNT - b)
    }
}

/// `a + b` on the TMOS clock.
fn add(a: u32, b: u32) -> u32 {
    diff(a, RTC_MAX_COUNT - b)
}

/// Whether `t` is due: at most half a day ahead counts as the future.
fn is_due(t: u32, now: u32) -> bool {
    diff(t, now) > RTC_MAX_COUNT / 2
}

/// The next RTC tick: read straight after the count changes.
fn next_tick() -> u32 {
    let t = clock();
    loop {
        let n = clock();
        if n != t {
            return n;
        }
    }
}

fn set_trigger(t: u32) {
    safe_access(|| pac::RTC.trig().write(|w| w.set_trig(t)));
    pac::RTC.flag_ctrl().write(|w| w.set_trig_clr(true));
}

/// `idleCB`: wait or sleep until `time` (TMOS clock), see the
/// [module docs](self). Returns 0 if the chip slept.
pub(super) unsafe extern "C" fn idle(time: u32) -> u32 {
    if !SLEEP_ALLOWED.load(Ordering::Relaxed) {
        return 1;
    }
    critical_section::with(|cs| idle_cs(cs, time))
}

fn idle_cs(cs: CriticalSection, time: u32) -> u32 {
    if crate::embassy::pended() {
        return 1;
    }
    // Alarms already due wake their tasks here.
    let alarm = time_driver::next_alarm(cs);
    if crate::embassy::pended() {
        return 1;
    }

    let now = clock();
    if is_due(time, now) {
        return 1;
    }
    let mut target = time;
    let mut left = diff(time, now);
    if alarm != u64::MAX {
        let ticks = alarm.saturating_sub(embassy_time_driver::now());
        let rtc = ticks.saturating_mul(CLOCK_HZ as u64) / TICK_HZ;
        if rtc < left as u64 {
            left = rtc as u32;
            target = add(now, left);
        }
    }

    if left < WAKE_EARLY + SLEEP_MIN || !ENABLED.load(Ordering::Relaxed) {
        // The trigger compares for equality: one about to pass would not
        // fire, so leave the last tick to the next pass.
        if diff(time, now) > 1 {
            set_trigger(time);
            unsafe { core::arch::asm!("wfi") };
            count(&STATS.waits);
        }
        return 1;
    }

    let t0 = next_tick();
    let c0 = time_driver::count(cs);
    if is_due(target, t0) || diff(target, t0) < WAKE_EARLY + SLEEP_MIN {
        return 1;
    }
    set_trigger(diff(target, WAKE_EARLY));
    let mut config = SleepConfig::default();
    config.usb_ble = true;
    pwr::sleep(config);
    let t1 = next_tick();

    let hz = crate::rcc::clocks().sysclk.0 as u64;
    let slept = diff(t1, t0) as u64;
    time_driver::set_count(cs, c0 + slept * hz / CLOCK_HZ as u64);
    count(&STATS.sleeps);
    0
}
