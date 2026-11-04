//! Power management test: battery monitor, DC-DC (off unless `DCDC` is
//! set), then idle, halt and sleep woken by the RTC alarm, sleep woken by
//! the KEY button (PB22) or a 10 s timeout, and finally shutdown, which wakes
//! as a reset after 1 s and reports from there. Results are reported over SDI
//! print.
//!
//! hw-test: manual. It enters the low-power modes, which may drop the debug
//! link, so `hw-test.sh` runs it only when named. The key sleep waits up to
//! 10 s: allow `SECS=60`.

#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Instant, Timer};
use hal::debug::SDIPrint;
use hal::exti::ExtiInput;
use hal::gpio::Pull;
use hal::pwr::{
    self, BatteryMonitor, LowPowerLevel, PreciseLevel, ResetReason, ShutdownConfig, SleepConfig, WakeConfig,
};
use hal::rtc::{Config, Rtc, TICKS_PER_SECOND};
use panic_halt as _;

/// Set once the board is known to have the DC-DC inductor (VSW to VDCID).
const DCDC: bool = false;

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
        hal::println!("{} {}", if ok { "PASS" } else { "FAIL" }, name);
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
        rcc: hal::rcc::Config::SYSCLK_FREQ_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut c = Checks { pass: 0, fail: 0 };
    let mut rtc = Rtc::new(p.RTC, Config::default()).unwrap();

    let reason = pwr::reset_reason();
    hal::println!("reset reason: {:?}", reason);
    let k = kept();
    if unsafe { (*k).magic } == MAGIC {
        // Second boot: after the shutdown.
        unsafe { (*k).magic = 0 };
        c.pass = unsafe { (*k).pass };
        c.fail = unsafe { (*k).fail };
        let slept = ms(rtc.now() - unsafe { (*k).at });
        hal::println!("back from shutdown after {} ms", slept);
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
    hal::println!(
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
    hal::println!("precise 2.55 V: {:?}", s);
    c.check("precise detector: 3.3 V is not low", !s.low && !s.lower);
    pwr::set_battery_monitor(BatteryMonitor::LowPower(LowPowerLevel::V2_4));
    Timer::after_millis(1).await;
    let s = pwr::battery_status();
    hal::println!("low-power 2.4 V: {:?}", s);
    c.check("low-power monitor: 3.3 V is not low", !s.low);
    pwr::set_battery_monitor(BatteryMonitor::Off);

    if DCDC {
        let r = pwr::enable_dcdc();
        hal::println!("enable_dcdc: {:?}", r);
        c.check("DC-DC enables", r.is_ok() && pwr::is_dcdc_enabled());
        // Measure the supply current now (WCH: about 60% of the LDO's).
        let t0 = Instant::now();
        while t0.elapsed().as_millis() < 3000 {}
        c.check("still running on DC-DC", pwr::is_dcdc_enabled());
        pwr::disable_dcdc();
        c.check("DC-DC disables", !pwr::is_dcdc_enabled());
    } else {
        hal::println!("DC-DC skipped (set DCDC once the inductor is confirmed)");
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
        pwr::idle();
        n += 1;
    }
    let e = t0.elapsed().as_millis();
    rtc.clear_alarm();
    hal::println!("idle: {} waits, {} ms", n, e);
    c.check(
        "idle ends on the RTC alarm after ~100 ms",
        (95..130).contains(&e) && n < 100,
    );

    // Halt: HCLK stops, so SysTick misses the wait; the RTC does not.
    let t0 = Instant::now();
    let r0 = alarm_in(&mut rtc, 200);
    pwr::halt();
    let (e, r) = (t0.elapsed().as_millis(), ms(rtc.now() - r0));
    let pending = rtc.is_alarm_pending();
    rtc.clear_alarm();
    hal::println!("halt: RTC {} ms, SysTick {} ms, alarm {}", r, e, pending);
    c.check(
        "halt ends on the RTC alarm after ~200 ms",
        pending && (190..260).contains(&r),
    );
    c.check("SysTick stopped in halt", e < 50);
    c.check("HCLK restored after halt", hal::rcc::sysclk_from_registers() == sysclk);

    // Sleep, flash off.
    let t0 = Instant::now();
    let r0 = alarm_in(&mut rtc, 200);
    pwr::sleep(SleepConfig::default());
    let (e, r) = (t0.elapsed().as_millis(), ms(rtc.now() - r0));
    let pending = rtc.is_alarm_pending();
    rtc.clear_alarm();
    hal::println!("sleep: RTC {} ms, SysTick {} ms, alarm {}", r, e, pending);
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
    pwr::sleep(keep);
    let r = ms(rtc.now() - r0);
    let pending = rtc.is_alarm_pending();
    rtc.clear_alarm();
    hal::println!("sleep (flash on): RTC {} ms, alarm {}", r, pending);
    c.check(
        "sleep with flash on ends after ~100 ms",
        pending && (90..160).contains(&r),
    );

    // Sleep until KEY (PB22, pulled up, low when pressed) or 10 s.
    let mut key = ExtiInput::new(p.PB22, Pull::Up);
    wake.gpio = true;
    pwr::configure_wake(wake);
    hal::println!("press KEY within 10 s to wake the sleep (or wait)");
    Timer::after_millis(10).await;
    let r0 = alarm_in(&mut rtc, 10_000);
    // `select` polls the edge wait first, which arms the pin, then sleeps.
    // The sleep always finishes first: the pin interrupt that woke it runs
    // as it returns.
    let r = select(key.wait_for_falling_edge(), async {
        pwr::sleep(SleepConfig::default())
    })
    .await;
    let slept = ms(rtc.now() - r0);
    let by_rtc = rtc.is_alarm_pending();
    rtc.clear_alarm();
    rtc.disable_alarm();
    hal::println!(
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
    hal::println!("shutdown for 1 s");
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
    pwr::shutdown(sd)
}

async fn finish(c: &Checks) -> ! {
    hal::println!("PWR: {} passed, {} failed", c.pass, c.fail);
    loop {
        Timer::after_secs(1).await;
    }
}
