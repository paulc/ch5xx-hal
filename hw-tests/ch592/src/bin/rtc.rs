//! RTC test: counter rate against the HSE-derived SysTick, the calendar
//! (round trip, day, month, year and leap-day rollover, refused values), the
//! alarm and the interval timer. Results are reported over SDI print.
//!
//! The HAL's `Rtc` does not choose the 32 kHz source or calibrate the RC, so
//! the test selects the RC itself and the original's calibration and crystal
//! checks are skipped.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_time::{Duration, Instant};
use hal::peripherals::RTC;
use hal::rtc::{DateTime, Error, Period, Rtc};
use hwtest::check;
use qingke_rt::entry;

/// 32 kHz cycles per second, measured over `ms` of SysTick time.
fn rate(rtc: &Rtc<'_, RTC>, ms: u64) -> u64 {
    let (r0, t0) = (rtc.cycle_count(), Instant::now());
    while t0.elapsed() < Duration::from_millis(ms) {}
    let (r1, t1) = (rtc.cycle_count(), Instant::now());
    r1.wrapping_sub(r0) as u64 * 1_000_000 / (t1 - t0).as_micros()
}

fn dt(year: u16, month: u8, day: u8, hour: u8, minute: u8, second: u8) -> DateTime {
    DateTime {
        year,
        month,
        day,
        hour,
        minute,
        second,
    }
}

/// Set `from`, wait 1.2 s, and check the calendar reads `to`.
fn rollover(rtc: &mut Rtc<'_, RTC>, name: &str, from: DateTime, to: DateTime) {
    let r = rtc.set_datetime(from);
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_millis(1200) {}
    let now = rtc.datetime();
    check!(r.is_ok() && now == to, "{}: {:?}", name, now);
}

#[entry]
fn main() -> ! {
    // The PLL runs from the HSE: the reset clock (HSI/3) is an RC, no
    // reference for the LSI.
    let mut p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();
    hwtest::select_lsi();

    let mut rtc = Rtc::new(p.RTC.reborrow());
    hwtest::println!("RTC at boot: {:?}, {} cycles", rtc.datetime(), rtc.cycle_count());

    let hz = rate(&rtc, 1000);
    hwtest::println!("LSI: {} Hz", hz);
    // Uncalibrated, the LSI is a few percent off (30.7 kHz on one CH592F).
    check!(hz.abs_diff(32_768) < 6_554, "LSI within 20% of 32768 Hz ({})", hz);
    hwtest::skip("calibrate_lsi", "no RC calibration API");

    // Round trip, a second before midnight so the read lands on the set value.
    let set = dt(2026, 11, 8, 12, 34, 56);
    let r = rtc.set_datetime(set);
    let back = rtc.datetime();
    check!(r.is_ok() && back == set, "set_datetime reads back ({:?})", back);

    rollover(
        &mut rtc,
        "day rollover",
        dt(2026, 11, 8, 23, 59, 59),
        dt(2026, 11, 9, 0, 0, 0),
    );
    rollover(
        &mut rtc,
        "month rollover",
        dt(2026, 4, 30, 23, 59, 59),
        dt(2026, 5, 1, 0, 0, 0),
    );
    rollover(
        &mut rtc,
        "year rollover",
        dt(2026, 12, 31, 23, 59, 59),
        dt(2027, 1, 1, 0, 0, 0),
    );
    rollover(
        &mut rtc,
        "leap day",
        dt(2028, 2, 28, 23, 59, 59),
        dt(2028, 2, 29, 0, 0, 0),
    );
    rollover(
        &mut rtc,
        "no leap day",
        dt(2027, 2, 28, 23, 59, 59),
        dt(2027, 3, 1, 0, 0, 0),
    );

    // The calendar must keep counting: odd seconds are in the 32 kHz half.
    let a = rtc.datetime();
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_millis(3000) {}
    let b = rtc.datetime();
    let secs = |d: DateTime| d.hour as i32 * 3600 + d.minute as i32 * 60 + d.second as i32;
    let step = secs(b) - secs(a);
    check!((2..=4).contains(&step), "3 s later the calendar moved {} s", step);

    for (name, bad) in [
        ("30 February", dt(2026, 2, 30, 0, 0, 0)),
        ("29 February 2027", dt(2027, 2, 29, 0, 0, 0)),
        ("month 13", dt(2026, 13, 1, 0, 0, 0)),
        ("hour 24", dt(2026, 1, 1, 24, 0, 0)),
        ("year 2019", dt(2019, 12, 31, 0, 0, 0)),
        ("2064-11-09", dt(2064, 11, 9, 0, 0, 0)),
    ] {
        check!(rtc.set_datetime(bad) == Err(Error::DateTime), "{} refused", name);
    }
    let last = dt(2064, 11, 8, 23, 59, 58);
    let r = rtc.set_datetime(last);
    check!(r.is_ok() && rtc.datetime() == last, "2064-11-08 23:59:58 accepted");

    // Alarm 300 ms ahead.
    rtc.set_datetime(set).unwrap();
    rtc.clear_trigger();
    rtc.set_alarm_in(32_768 * 3 / 10);
    let t0 = Instant::now();
    check!(!rtc.trigger_pending(), "alarm not pending at once");
    while !rtc.trigger_pending() && t0.elapsed() < Duration::from_secs(2) {}
    let ms = t0.elapsed().as_millis();
    check!((270..330).contains(&ms), "alarm fires after ~300 ms ({} ms)", ms);
    rtc.clear_trigger();
    check!(!rtc.trigger_pending(), "alarm clears");
    rtc.disable_alarm();

    // Interval timer at 125 ms: count it for 1 s.
    rtc.start_timer(Period::Ms125);
    rtc.clear_timer();
    let t0 = Instant::now();
    let mut n = 0;
    while t0.elapsed() < Duration::from_millis(1000) {
        if rtc.timer_pending() {
            rtc.clear_timer();
            n += 1;
        }
    }
    rtc.stop_timer();
    check!(
        (7..=9).contains(&n),
        "interval 125 ms fires 7..=9 times a second ({})",
        n
    );

    hwtest::skip("LSE on PA10/PA11", "no clock-source API");

    hwtest::done("rtc")
}
