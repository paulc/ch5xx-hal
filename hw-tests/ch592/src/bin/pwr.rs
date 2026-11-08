//! Power management test: the battery detector, halt and sleep woken by the
//! RTC alarm, then sleep woken by the KEY button (PB22) or a 10 s timeout.
//!
//! Results go to UART0 TX on PB7 (115200 8N1), not SDI print, which stops at
//! the first halt: wire PB7 to WCH-LinkE RX and use a runner without
//! `--enable-sdi-print`, which then forwards the UART. The test waits 2 s
//! before printing, to give a serial monitor time to attach.
//!
//! The HAL has `power::halt` and `power::sleep` only, so the original's idle,
//! DC-DC, low-power battery monitor and shutdown checks are skipped.
//!
//! hw-test: manual. It enters the low-power modes, which drop the debug
//! link, so `hw-test.sh` runs it only when named. The key sleep waits up to
//! 10 s: allow `SECS=60`.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Instant, Timer};
use hal::gpio::{Input, Pull};
use hal::mode::Blocking;
use hal::peripherals::{RTC, UART0};
use hal::power::{self, Retention, WakeDelay, WakeSources};
use hal::rtc::Rtc;
use hal::uart::{self, UartTx};
use hal::watchdog::{Battery, Level};
use panic_halt as _;

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
        log!("{}: {}", name, if ok { "PASS" } else { "FAIL" });
    }
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

/// Milliseconds of RTC cycles.
fn ms(cycles: u32) -> u32 {
    (cycles as u64 * 1000 / 32_768) as u32
}

/// Set the RTC alarm `ms` ahead and return the cycle count it was set at.
fn alarm_in(rtc: &mut Rtc<'_, RTC>, ms: u32) -> u32 {
    rtc.clear_trigger();
    let now = rtc.cycle_count();
    rtc.set_alarm_in(ms * 32_768 / 1000);
    now
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    let tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::new(115_200)).unwrap();
    unsafe { *(&raw mut TX) = Some(tx) };
    let mut c = Checks { pass: 0, fail: 0 };
    hwtest::select_lsi();
    let mut rtc = Rtc::new(p.RTC);

    Timer::after_secs(2).await;
    log!("reset cause: {:?}", hal::watchdog::reset_cause());

    // The clock setting in force, to compare after each mode.
    let clk_cfg = hal::pac::SYS.clk_cfg().read().0;
    log!(
        "sysclk {} Hz, POWER_PLAN {:#06x}",
        hal::rcc::clocks().sysclk.0,
        hal::pac::SYS.power_plan().read().0
    );

    // Battery detector, on a 3.3 V supply.
    {
        let bat = Battery::new(p.SYS, Level::V2_5);
        Timer::after_millis(1).await;
        let s = bat.status();
        log!("detector 2.5 V: {:?}", s);
        c.check("battery detector: 3.3 V is not low", !s.low && !s.lower);
    }
    log!("low-power battery monitor: SKIP (no API)");
    log!("DC-DC: SKIP (no API)");
    log!("idle: SKIP (no API)");

    fill_pattern();

    // Halt: HCLK stops, so SysTick misses the wait; the RTC does not.
    let t0 = Instant::now();
    let r0 = alarm_in(&mut rtc, 200);
    flush();
    power::halt(WakeSources::RTC, WakeDelay::Long);
    let (e, r) = (t0.elapsed().as_millis(), ms(rtc.cycle_count().wrapping_sub(r0)));
    let pending = rtc.trigger_pending();
    rtc.clear_trigger();
    log!("halt: RTC {} ms, SysTick {} ms, alarm {}", r, e, pending);
    c.check("halt ends on the RTC alarm after ~200 ms", pending && (190..260).contains(&r));
    c.check("SysTick stopped in halt", e < 50);
    c.check("HCLK restored after halt", hal::pac::SYS.clk_cfg().read().0 == clk_cfg);

    // A second halt: the event register must not end it at once.
    let r0 = alarm_in(&mut rtc, 100);
    flush();
    power::halt(WakeSources::RTC, WakeDelay::Long);
    let r = ms(rtc.cycle_count().wrapping_sub(r0));
    rtc.clear_trigger();
    log!("second halt: RTC {} ms", r);
    c.check("second halt also lasts ~100 ms", (90..160).contains(&r));

    // Sleep, both SRAM blocks kept.
    let ram = Retention::RAM2K | Retention::RAM24K;
    let t0 = Instant::now();
    let r0 = alarm_in(&mut rtc, 200);
    flush();
    power::sleep(ram, WakeSources::RTC, WakeDelay::Long);
    let (e, r) = (t0.elapsed().as_millis(), ms(rtc.cycle_count().wrapping_sub(r0)));
    let pending = rtc.trigger_pending();
    rtc.clear_trigger();
    log!("sleep: RTC {} ms, SysTick {} ms, alarm {}", r, e, pending);
    c.check("sleep ends on the RTC alarm after ~200 ms", pending && (190..260).contains(&r));
    c.check("SysTick stopped in sleep", e < 50);
    let t1 = Instant::now();
    Timer::after_millis(10).await;
    let e = t1.elapsed().as_millis();
    c.check("SysTick runs again after sleep", (9..20).contains(&e));
    c.check("HCLK restored after sleep", hal::pac::SYS.clk_cfg().read().0 == clk_cfg);
    c.check("SRAM kept through sleep", pattern_ok());

    // Sleep until KEY (PB22, pulled up, low when pressed) or 10 s.
    let mut key = Input::new(p.PB22, Pull::Up);
    log!("press KEY within 10 s to wake the sleep (or wait)");
    Timer::after_millis(10).await;
    let r0 = alarm_in(&mut rtc, 10_000);
    // `select` polls the edge wait first, which arms the pin, then sleeps.
    // The sleep always finishes first: the pin interrupt that woke it runs
    // as it returns.
    flush();
    let r = select(key.wait_for_falling_edge(), async {
        power::sleep(ram, WakeSources::RTC | WakeSources::GPIO, WakeDelay::Long)
    })
    .await;
    let slept = ms(rtc.cycle_count().wrapping_sub(r0));
    let by_rtc = rtc.trigger_pending();
    rtc.clear_trigger();
    rtc.disable_alarm();
    log!(
        "key sleep: {} ms, woken by {}",
        slept,
        if by_rtc { "RTC (no key press)" } else { "KEY" }
    );
    c.check("key sleep finishes before the edge wait", matches!(r, Either::Second(())));
    if by_rtc {
        c.check("RTC timeout ends the key sleep after ~10 s", (9_900..10_500).contains(&slept));
    } else {
        c.check("KEY wakes the sleep before the timeout", slept < 9_900);
    }
    drop(key);

    log!("shutdown: SKIP (no API)");
    log!("pwr done: {} pass, {} fail", c.pass, c.fail);
    loop {
        Timer::after_secs(1).await;
        log!("pwr done: {} pass, {} fail", c.pass, c.fail);
    }
}
