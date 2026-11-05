//! RTC test: counter rate against the HSE-derived SysTick, LSI calibration,
//! setting the
//! counter across a day boundary, the alarm and the periodic flag, then an
//! attempt to switch to a 32.768 kHz crystal on PA10/PA11 (reported, not
//! required). Results are reported over SDI print.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use hal::debug::SDIPrint;
use hal::rtc::{ClockSource, Config, Periodic, Rtc, TICKS_PER_DAY, TICKS_PER_SECOND};
use panic_halt as _;

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

/// RTC ticks per second, measured over `ms` of SysTick time.
async fn rate(rtc: &Rtc<'_>, ms: u64) -> u64 {
    let (r0, t0) = (rtc.now(), Instant::now());
    Timer::after_millis(ms).await;
    let (r1, t1) = (rtc.now(), Instant::now());
    (r1 - r0) * 1_000_000 / (t1 - t0).as_micros()
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    // The PLL runs from the HSE: the reset clock (HSI/3) is an RC, no
    // reference for the LSI.
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::HSE_PLL_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut c = Checks { pass: 0, fail: 0 };

    let mut rtc = Rtc::new(p.RTC.reborrow(), Config::default()).unwrap();
    c.check("clock source is LSI", rtc.clock_source() == ClockSource::Lsi);
    hal::println!("RTC at boot: {} ticks", rtc.now());

    let hz = rate(&rtc, 1000).await;
    hal::println!("LSI: {} Hz", hz);
    // Uncalibrated, the LSI is a few percent off (29.6 kHz on one CH584F).
    // The trim survives resets, so an earlier calibration may still apply.
    c.check("LSI within 20% of 32768 Hz", hz.abs_diff(32_768) < 6_554);

    let tune0 = hal::pac::SYS.int32k_tune().read().int32k_tune();
    let t0 = Instant::now();
    let r = rtc.calibrate_lsi(32_768);
    let ms = t0.elapsed().as_millis();
    let tune1 = hal::pac::SYS.int32k_tune().read().int32k_tune();
    hal::println!(
        "calibrate_lsi: {:?} in {} ms, tune {:#06x} -> {:#06x}",
        r,
        ms,
        tune0,
        tune1
    );
    c.check(
        "calibrate_lsi reports under 500 ppm",
        matches!(r, Ok(e) if e.abs() < 500),
    );
    let hz = rate(&rtc, 1000).await;
    hal::println!("LSI after calibration: {} Hz", hz);
    c.check("calibrated LSI within 0.1% of 32768 Hz", hz.abs_diff(32_768) < 33);

    // Set just before the end of day 5 and watch it roll over.
    let t = 6 * TICKS_PER_DAY - TICKS_PER_SECOND / 4;
    rtc.set_ticks(t);
    let r = rtc.now();
    hal::println!("set {} read {}", t, r);
    c.check("set_ticks reads back within 2 ms", r >= t && r - t < 64);
    let mut last = rtc.now();
    let mut monotonic = true;
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(500) {
        let n = rtc.now();
        monotonic &= n >= last;
        last = n;
    }
    hal::println!("after rollover: {} (day {})", last, last / TICKS_PER_DAY);
    c.check("counter rolls into day 6", last / TICKS_PER_DAY == 6);
    c.check("now() monotonic across the day boundary", monotonic);

    // Alarm 300 ms ahead.
    rtc.set_alarm(rtc.now() + TICKS_PER_SECOND * 3 / 10);
    let t0 = Instant::now();
    c.check("alarm not pending at once", !rtc.is_alarm_pending());
    while !rtc.is_alarm_pending() && t0.elapsed() < Duration::from_secs(2) {}
    let ms = t0.elapsed().as_millis();
    hal::println!("alarm after {} ms", ms);
    c.check("alarm fires after ~300 ms", (270..330).contains(&ms));
    rtc.clear_alarm();
    c.check("alarm clears", !rtc.is_alarm_pending());
    rtc.disable_alarm();

    // Periodic flag at 125 ms: count it for 1 s.
    rtc.set_periodic(Some(Periodic::Ms125));
    let t0 = Instant::now();
    let mut n = 0;
    while t0.elapsed() < Duration::from_millis(1000) {
        if rtc.is_periodic_pending() {
            rtc.clear_periodic();
            n += 1;
        }
    }
    rtc.set_periodic(None);
    hal::println!("periodic flags in 1 s: {}", n);
    c.check("periodic 125 ms fires 7..=9 times a second", (7..=9).contains(&n));

    // LSE, if the board has a crystal.
    drop(rtc);
    let mut lse = Config::default();
    lse.clock_source = ClockSource::Lse;
    match Rtc::new(p.RTC.reborrow(), lse) {
        Ok(rtc) => {
            let hz = rate(&rtc, 1000).await;
            hal::println!("LSE running: {} Hz", hz);
            c.check("LSE within 0.1% of 32768 Hz", hz.abs_diff(32_768) < 33);
            drop(rtc);
            let rtc = Rtc::new(p.RTC.reborrow(), Config::default()).unwrap();
            c.check("back on LSI", rtc.clock_source() == ClockSource::Lsi);
        }
        Err(e) => {
            hal::println!("no LSE: {:?}", e);
            let rtc = Rtc::new(p.RTC.reborrow(), Config::default()).unwrap();
            c.check(
                "still on LSI and counting",
                rtc.clock_source() == ClockSource::Lsi && rate(&rtc, 100).await > 30_000,
            );
        }
    }

    hal::println!("RTC: {} passed, {} failed", c.pass, c.fail);
    loop {
        Timer::after_secs(1).await;
    }
}
