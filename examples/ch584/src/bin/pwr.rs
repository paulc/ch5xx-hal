//! Power management test: battery monitor, DC-DC (off unless `DCDC` is
//! set), then idle, halt and sleep woken by the RTC alarm, sleep woken by
//! the KEY button (PB22) or a 10 s timeout, and finally shutdown, which wakes
//! as a reset after 1 s and reports from there.
//!
//! Results go to UART0 TX on PB7 (115200 8N1), not SDI print, which stops at
//! the first halt: wire PB7 to WCH-LinkE RX and use a runner without
//! `--enable-sdi-print`, which then forwards the UART. The first boot waits
//! 2 s before printing, to give a serial monitor time to attach.
//!
//! hw-test: manual. It enters the low-power modes, which drop the debug
//! link, so `hw-test.sh` runs it only when named. The key sleep waits up to
//! 10 s: allow `SECS=60`.

#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Instant, Timer};
use hal::exti::ExtiInput;
use hal::gpio::Pull;
use hal::mode::Blocking;
use hal::peripherals::UART0;
use hal::pwr::{
    self, BatteryMonitor, LowPowerLevel, PreciseLevel, ResetReason, ShutdownConfig, SleepConfig, WakeConfig,
};
use hal::rtc::{Config, Rtc, TICKS_PER_SECOND};
use hal::uart::{self, UartTx};
use panic_halt as _;

/// Set once the board is known to have the DC-DC inductor (VSW to VDCID).
/// Without it the chip is expected to reset when DC-DC takes over: the next
/// boot reports that and leaves DC-DC alone.
const DCDC: bool = true;

/// The output.
static mut TX: Option<UartTx<'static, UART0, Blocking>> = None;

/// Print a line to the UART.
fn out(args: core::fmt::Arguments) {
    if let Some(tx) = unsafe { &mut *(&raw mut TX) } {
        let _ = core::fmt::Write::write_fmt(tx, args);
        let _ = tx.blocking_write(b"\r\n");
    }
}

macro_rules! log {
    ($($t:tt)*) => { out(format_args!($($t)*)) };
}

/// Let the UART finish before a low-power mode stops its clock.
fn flush() {
    if let Some(tx) = unsafe { &mut *(&raw mut TX) } {
        let _ = tx.blocking_flush();
    }
}

struct Checks {
    pass: u32,
    fail: u32,
}

impl Checks {
    fn check(&mut self, name: &str, ok: bool) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        log!("{} {}", if ok { "PASS" } else { "FAIL" }, name);
    }
}

/// Kept across the shutdown in the main SRAM (`.uninit` is not cleared).
#[repr(C)]
struct Kept {
    magic: u32,
    pass: u32,
    fail: u32,
    /// RTC time at shutdown.
    at: u64,
}

const MAGIC: u32 = 0x5157_5244;

#[unsafe(link_section = ".uninit.pwr_test")]
static mut KEPT: MaybeUninit<Kept> = MaybeUninit::uninit();

fn kept() -> *mut Kept {
    (&raw mut KEPT).cast()
}

/// Set while DC-DC is on, so a reset in that time is noticed.
#[unsafe(link_section = ".uninit.pwr_test")]
static mut DCDC_TRYING: MaybeUninit<u32> = MaybeUninit::uninit();

const DCDC_MAGIC: u32 = 0x4443_4443;

fn dcdc_trying(on: bool) {
    unsafe { (&raw mut DCDC_TRYING).cast::<u32>().write_volatile(if on { DCDC_MAGIC } else { 0 }) };
}

/// Busy for `ms`, for a supply current reading.
fn busy(ms: u64) {
    let t0 = Instant::now();
    while t0.elapsed().as_millis() < ms {}
}

/// Data the sleeps must keep.
static mut PATTERN: [u32; 64] = [0; 64];

fn fill_pattern() {
    let p = &raw mut PATTERN;
    for i in 0..64 {
        unsafe { (*p)[i] = 0xa5a5_0000 ^ (i as u32 * 0x0101_0101) };
    }
}

fn pattern_ok() -> bool {
    let p = &raw const PATTERN;
    (0..64).all(|i| unsafe { (*p)[i] } == 0xa5a5_0000 ^ (i as u32 * 0x0101_0101))
}

fn ms(ticks: u64) -> u64 {
    ticks * 1000 / TICKS_PER_SECOND
}

/// Set the RTC alarm `ms` ahead and return the time it was set.
fn alarm_in(rtc: &mut Rtc<'_>, ms: u64) -> u64 {
    let now = rtc.now();
    rtc.set_alarm(now + ms * TICKS_PER_SECOND / 1000);
    now
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    let tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::default()).unwrap();
    unsafe { *(&raw mut TX) = Some(tx) };
    let mut c = Checks { pass: 0, fail: 0 };
    let mut rtc = Rtc::new(p.RTC, Config::default()).unwrap();

    let reason = pwr::reset_reason();
    let dcdc_reset = unsafe { (&raw const DCDC_TRYING).cast::<u32>().read_volatile() } == DCDC_MAGIC;
    dcdc_trying(false);
    let k = kept();
    let second_boot = unsafe { (*k).magic } == MAGIC;
    if !second_boot {
        Timer::after_secs(2).await;
    }
    log!("reset reason: {:?}", reason);
    if second_boot {
        // Second boot: after the shutdown.
        unsafe { (*k).magic = 0 };
        c.pass = unsafe { (*k).pass };
        c.fail = unsafe { (*k).fail };
        let slept = ms(rtc.now() - unsafe { (*k).at });
        log!("back from shutdown after {} ms", slept);
        c.check(
            "shutdown wakes as a shutdown-wake reset",
            reason == ResetReason::ShutdownWake,
        );
        c.check("main SRAM kept through shutdown", true);
        c.check("RTC alarm wakes shutdown after ~1 s", (900..1500).contains(&slept));
        rtc.clear_alarm();
        rtc.disable_alarm();
        finish(&c).await
    }

    let sysclk = hal::rcc::clocks().sysclk;
    log!(
        "sysclk {} Hz, DC-DC supported {}, enabled {}, POWER_PLAN {:#06x}",
        sysclk.0,
        pwr::dcdc_supported(),
        pwr::is_dcdc_enabled(),
        hal::pac::SYS.power_plan().read().0
    );

    // Battery monitor, on a 3.3 V supply.
    pwr::set_battery_monitor(BatteryMonitor::Precise(PreciseLevel::V2_55));
    Timer::after_millis(1).await;
    let s = pwr::battery_status();
    log!("precise 2.55 V: {:?}", s);
    c.check("precise detector: 3.3 V is not low", !s.low && !s.lower);
    pwr::set_battery_monitor(BatteryMonitor::LowPower(LowPowerLevel::V2_4));
    Timer::after_millis(1).await;
    let s = pwr::battery_status();
    log!("low-power 2.4 V: {:?}", s);
    c.check("low-power monitor: 3.3 V is not low", !s.low);
    pwr::set_battery_monitor(BatteryMonitor::Off);

    if dcdc_reset {
        c.check("no reset while on DC-DC (is the inductor fitted?)", false);
    } else if DCDC && !matches!(reason, ResetReason::Software | ResetReason::Pin) {
        // A power-on reset may be a supply glitch from the last try.
        log!("DC-DC skipped after a {:?} reset: reset the chip to try it", reason);
    } else if DCDC {
        // Compare the supply current in the two (WCH: about 60% on DC-DC).
        log!("busy 5 s on the LDO");
        busy(5000);
        log!("busy 5 s on DC-DC");
        flush();
        dcdc_trying(true);
        let r = pwr::enable_dcdc();
        let on = pwr::is_dcdc_enabled();
        busy(5000);
        let still = pwr::is_dcdc_enabled();
        pwr::disable_dcdc();
        dcdc_trying(false);
        log!("enable_dcdc: {:?}", r);
        c.check("DC-DC enables", r.is_ok() && on);
        c.check("still running on DC-DC after 5 s", still);
        c.check("DC-DC disables", !pwr::is_dcdc_enabled());
    } else {
        log!("DC-DC skipped (set DCDC once the inductor is confirmed)");
    }

    let mut wake = WakeConfig::default();
    wake.rtc = true;
    pwr::configure_wake(wake);
    fill_pattern();

    // Idle: clocks run, so SysTick sees the whole wait.
    let t0 = Instant::now();
    alarm_in(&mut rtc, 100);
    let mut n = 0;
    while !rtc.is_alarm_pending() && n < 100 {
        flush();
        pwr::idle();
        n += 1;
    }
    let e = t0.elapsed().as_millis();
    rtc.clear_alarm();
    log!("idle: {} waits, {} ms", n, e);
    c.check(
        "idle ends on the RTC alarm after ~100 ms",
        (95..130).contains(&e) && n < 100,
    );

    // Halt: HCLK stops, so SysTick misses the wait; the RTC does not.
    let t0 = Instant::now();
    let r0 = alarm_in(&mut rtc, 200);
    flush();
    pwr::halt();
    let (e, r) = (t0.elapsed().as_millis(), ms(rtc.now() - r0));
    let pending = rtc.is_alarm_pending();
    rtc.clear_alarm();
    log!("halt: RTC {} ms, SysTick {} ms, alarm {}", r, e, pending);
    c.check(
        "halt ends on the RTC alarm after ~200 ms",
        pending && (190..260).contains(&r),
    );
    c.check("SysTick stopped in halt", e < 50);
    c.check("HCLK restored after halt", hal::rcc::sysclk_from_registers() == sysclk);

    // Sleep, flash off.
    let t0 = Instant::now();
    let r0 = alarm_in(&mut rtc, 200);
    flush();
    pwr::sleep(SleepConfig::default());
    let (e, r) = (t0.elapsed().as_millis(), ms(rtc.now() - r0));
    let pending = rtc.is_alarm_pending();
    rtc.clear_alarm();
    log!("sleep: RTC {} ms, SysTick {} ms, alarm {}", r, e, pending);
    c.check(
        "sleep ends on the RTC alarm after ~200 ms",
        pending && (190..260).contains(&r),
    );
    c.check("SysTick stopped in sleep", e < 50);
    c.check("HCLK restored after sleep", hal::rcc::sysclk_from_registers() == sysclk);
    c.check("SRAM kept through sleep", pattern_ok());

    // Sleep, flash kept powered.
    let r0 = alarm_in(&mut rtc, 100);
    let mut keep = SleepConfig::default();
    keep.flash = true;
    flush();
    pwr::sleep(keep);
    let r = ms(rtc.now() - r0);
    let pending = rtc.is_alarm_pending();
    rtc.clear_alarm();
    log!("sleep (flash on): RTC {} ms, alarm {}", r, pending);
    c.check(
        "sleep with flash on ends after ~100 ms",
        pending && (90..160).contains(&r),
    );

    // Sleep until KEY (PB22, pulled up, low when pressed) or 10 s.
    let mut key = ExtiInput::new(p.PB22, Pull::Up);
    wake.gpio = true;
    pwr::configure_wake(wake);
    log!("press KEY within 10 s to wake the sleep (or wait)");
    Timer::after_millis(10).await;
    let r0 = alarm_in(&mut rtc, 10_000);
    // `select` polls the edge wait first, which arms the pin, then sleeps.
    // The sleep always finishes first: the pin interrupt that woke it runs
    // as it returns.
    flush();
    let r = select(key.wait_for_falling_edge(), async {
        pwr::sleep(SleepConfig::default())
    })
    .await;
    let slept = ms(rtc.now() - r0);
    let by_rtc = rtc.is_alarm_pending();
    rtc.clear_alarm();
    rtc.disable_alarm();
    log!(
        "key sleep: {} ms, woken by {}",
        slept,
        if by_rtc { "RTC (no key press)" } else { "KEY" }
    );
    c.check(
        "key sleep finishes before the edge wait",
        matches!(r, Either::Second(())),
    );
    if by_rtc {
        c.check(
            "RTC timeout ends the key sleep after ~10 s",
            (9_900..10_500).contains(&slept),
        );
    } else {
        c.check("KEY wakes the sleep before the timeout", slept < 9_900);
    }
    drop(key);
    wake.gpio = false;
    pwr::configure_wake(wake);

    // Shutdown: wakes as a reset, reported by the next boot.
    log!("shutdown for 1 s");
    Timer::after_millis(10).await;
    let at = alarm_in(&mut rtc, 1000);
    unsafe {
        k.write(Kept {
            magic: MAGIC,
            pass: c.pass,
            fail: c.fail,
            at,
        })
    };
    let mut sd = ShutdownConfig::default();
    sd.main_ram = true;
    flush();
    pwr::shutdown(sd)
}

async fn finish(c: &Checks) -> ! {
    log!("PWR: {} passed, {} failed", c.pass, c.fail);
    loop {
        Timer::after_secs(1).await;
    }
}
