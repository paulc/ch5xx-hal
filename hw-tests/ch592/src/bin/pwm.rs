//! PWM test for `TMRx` (`SimplePwm`) and `PWMX`, needing no wiring: each
//! output's period and high time are measured by timing its own pin's edges
//! against SysTick. Results are reported over SDI print.
//!
//! Pins: TMR0 PA9, TMR1 PA10, TMR2 PA11, TMR3 PB22 (the KEY button: don't
//! press it while this runs), PWM4 PA12, PWM5 PA13, PWM7 PB4, PWM9 PB7.
//!
//! This HAL's `pwm::Pwm::channel` borrows the block mutably for the channel's
//! lifetime, so only one PWMX channel can be live at a time: the channels are
//! checked one after another. It has no 16-bit cycle mode.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embedded_hal::pwm::SetDutyCycle;
use hal::delay::Delay;
use hal::pac;
use hal::pwm::{self, Cycle, Pwm, Polarity};
use hal::timer::simple_pwm::SimplePwm;
use hal::timer::{OutputPolarity, PeriodTicks, Repeat};
use hwtest::check;
use panic_halt as _;

#[derive(Clone, Copy)]
enum P {
    A(usize),
    B(usize),
}

fn level(p: P) -> bool {
    match p {
        P::A(n) => pac::GPIO.pa_pin().read().pa_pin(n),
        P::B(n) => pac::GPIO.pb_pin().read().pb_pin(n),
    }
}

/// The SysTick count. The embassy time driver reloads it at `STK_CMPLR`,
/// which stays put while no embassy timer is pending, so take differences
/// with [`cycles`].
fn now() -> u32 {
    pac::SYSTICK.stk_cntl().read().cntl()
}

/// HCLK cycles from `t0` to `t1`, across at most one reload.
fn cycles(t0: u32, t1: u32) -> u32 {
    if t1 >= t0 {
        t1 - t0
    } else {
        t1.wrapping_sub(t0).wrapping_add(pac::SYSTICK.stk_cmplr().read().cmpl().wrapping_add(1))
    }
}

/// Wait for the pin to reach `high`, up to `timeout` cycles.
fn wait(p: P, high: bool, timeout: u32) -> Option<u32> {
    let t0 = now();
    loop {
        if level(p) == high {
            return Some(now());
        }
        if cycles(t0, now()) > timeout {
            return None;
        }
    }
}

/// Average (period, high time) in HCLK cycles over 8 periods, or the
/// constant level if the pin does not toggle.
fn measure(p: P) -> Result<(u32, u32), bool> {
    const TO: u32 = 6_000_000; // 100 ms at 60 MHz
    let (mut per, mut hi) = (0, 0);
    wait(p, false, TO).ok_or(true)?;
    let mut rise = wait(p, true, TO).ok_or(false)?;
    for _ in 0..8 {
        let fall = wait(p, false, TO).ok_or(true)?;
        let next = wait(p, true, TO).ok_or(false)?;
        hi += cycles(rise, fall);
        per += cycles(rise, next);
        rise = next;
    }
    Ok((per / 8, hi / 8))
}

/// Check the pin's period and duty (in permille) to within 1.5%.
///
/// `set_duty` restarts the timer, and the period it restarts in is not at
/// the new duty, so let 2 ms (two periods at 1 kHz) pass first.
fn check_pwm(name: &str, p: P, period: u32, permille: u32) {
    Delay.delay_ms(2);
    match measure(p) {
        Ok((per, hi)) => {
            let d = hi * 1000 / per;
            let per_ok = per.abs_diff(period) * 1000 <= period * 15;
            check!(
                per_ok && d.abs_diff(permille) <= 15,
                "{}: period {} (want {}), duty {} permille (want {})",
                name,
                per,
                period,
                d,
                permille
            );
        }
        Err(level) => {
            check!(false, "{}: constant {}", name, if level { "high" } else { "low" });
        }
    }
}

/// Check the pin stays at `high`.
fn check_constant(name: &str, p: P, high: bool) {
    let r = measure(p);
    if let Ok((per, hi)) = r {
        hwtest::println!("  {}: toggling, period {} high {}", name, per, hi);
    }
    hwtest::check(name, r == Err(high));
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();
    // The 32 kHz crystal holds PA10/PA11 while it runs.
    hwtest::select_lsi();
    let mut delay = Delay;
    let hclk = hal::rcc::clocks().sysclk.0;
    hwtest::println!("HCLK {} Hz", hclk);

    // ---- TMR0 on PA9
    {
        let period = PeriodTicks::from_hz(hclk, 10_000).unwrap();
        let mut pwm = SimplePwm::new::<0>(
            p.TMR0.reborrow(),
            p.PA9.reborrow(),
            period,
            OutputPolarity::ActiveHigh,
            Repeat::Once,
        );
        let max = pwm.period().get();
        hwtest::check("TMR0 period = HCLK / 10 kHz", max == hclk / 10_000);
        pwm.set_duty(max / 4).unwrap();
        check_pwm("TMR0 10 kHz 25%", P::A(9), max, 250);
        pwm.set_duty(max * 3 / 4).unwrap();
        check_pwm("TMR0 10 kHz 75%", P::A(9), max, 750);
        pwm.set_duty(0).unwrap();
        delay.delay_ms(2);
        check_constant("TMR0 duty 0 is low", P::A(9), false);
        pwm.set_duty(max).unwrap();
        delay.delay_ms(2);
        check_constant("TMR0 duty = period is high", P::A(9), true);
        hwtest::check("TMR0 duty above the period refused", pwm.set_duty(max + 1).is_err());
        // Many updates in a row; the last should win.
        for i in 0..20 {
            pwm.set_duty(max * i / 40).unwrap();
        }
        pwm.set_duty(max / 10).unwrap();
        delay.delay_ms(2);
        check_pwm("TMR0 after 20 updates: 10%", P::A(9), max, 100);
        pwm.set_duty_cycle_percent(30).unwrap();
        delay.delay_ms(2);
        check_pwm("TMR0 SetDutyCycle 30%", P::A(9), max, 300);
        pwm.disable();
        delay.delay_ms(2);
        check_constant("TMR0 disabled idles low", P::A(9), false);
    }
    {
        let period = PeriodTicks::from_hz(hclk, 1000).unwrap();
        let mut pwm = SimplePwm::new::<0>(
            p.TMR0.reborrow(),
            p.PA9.reborrow(),
            period,
            OutputPolarity::ActiveLow,
            Repeat::Once,
        );
        pwm.set_duty(period.get() * 3 / 4).unwrap();
        check_pwm("TMR0 1 kHz active low 75% -> high 25%", P::A(9), hclk / 1000, 250);
    }

    // ---- TMR1-3
    let (pol, rep) = (OutputPolarity::ActiveHigh, Repeat::Once);
    {
        let period = PeriodTicks::from_hz(hclk, 20_000).unwrap();
        let mut pwm = SimplePwm::new::<0>(p.TMR1.reborrow(), p.PA10.reborrow(), period, pol, rep);
        pwm.set_duty(period.get() / 2).unwrap();
        check_pwm("TMR1 20 kHz 50% on PA10", P::A(10), hclk / 20_000, 500);
    }
    {
        let period = PeriodTicks::from_hz(hclk, 20_000).unwrap();
        let mut pwm = SimplePwm::new::<0>(p.TMR2.reborrow(), p.PA11.reborrow(), period, pol, rep);
        pwm.set_duty(period.get() / 5).unwrap();
        check_pwm("TMR2 20 kHz 20% on PA11", P::A(11), hclk / 20_000, 200);
    }
    {
        let period = PeriodTicks::from_hz(hclk, 5_000).unwrap();
        let mut pwm = SimplePwm::new::<0>(p.TMR3.reborrow(), p.PB22.reborrow(), period, pol, rep);
        pwm.set_duty(period.get() * 4 / 5).unwrap();
        check_pwm("TMR3 5 kHz 80% on PB22", P::B(22), hclk / 5_000, 800);
    }

    // ---- PWMX, 8-bit: HCLK / 24 / 256 = 10.16 kHz.
    {
        let mut pwmx = Pwm::new(p.PWMX.reborrow(), pwm::Config::new(24, Cycle::Clocks256)).unwrap();
        let div = pac::PWMX.clock_div().read().clock_div() as u32;
        hwtest::check("PWMX div = 24", div == 24);
        let per = div * 256;
        {
            let mut ch = pwmx.channel(p.PA12.reborrow(), Polarity::ActiveHigh);
            ch.set_duty(64);
            check_pwm("PWM4 25%", P::A(12), per, 250);
        }
        {
            let mut ch = pwmx.channel(p.PA13.reborrow(), Polarity::ActiveHigh);
            ch.set_duty(128);
            check_pwm("PWM5 50%", P::A(13), per, 500);
            ch.set_duty(0);
            delay.delay_ms(1);
            check_constant("PWM5 duty 0 is low", P::A(13), false);
        }
        {
            let mut ch = pwmx.channel(p.PB4.reborrow(), Polarity::ActiveLow);
            ch.set_duty(192);
            check_pwm("PWM7 active low 75% -> high 25%", P::B(4), per, 250);
        }
        {
            let mut ch = pwmx.channel(p.PB7.reborrow(), Polarity::ActiveHigh);
            ch.set_duty(32);
            check_pwm("PWM9 12.5%", P::B(7), per, 125);
            ch.set_duty_cycle_fraction(1, 2).unwrap();
            check_pwm("PWM9 SetDutyCycle 1/2", P::B(7), per, 500);
            ch.set_duty(255);
            check_pwm("PWM9 duty 255 of 256", P::B(7), per, 996);
        }
        // Dropping the channel stops the output; the pin is left a pulled-up
        // input.
        check_constant("PWM9 dropped: pin idles (pull-up)", P::B(7), true);
    }
    // ---- PWMX, 63-clock cycle: HCLK / 10 / 63 = 99 kHz.
    {
        let mut pwmx = Pwm::new(p.PWMX.reborrow(), pwm::Config::new(10, Cycle::Clocks63)).unwrap();
        let mut ch = pwmx.channel(p.PA12.reborrow(), Polarity::ActiveHigh);
        ch.set_duty(21);
        check_pwm("PWM4 63-clock cycle, 21/63", P::A(12), 630, 333);
    }
    hwtest::check(
        "PWMX divisor 0 refused",
        matches!(Pwm::new(p.PWMX.reborrow(), pwm::Config::new(0, Cycle::Clocks256)), Err(pwm::ConfigError::Divisor)),
    );
    hwtest::skip("PWMX 16-bit cycle", "not in this HAL");
    hwtest::skip("PWMX several channels at once", "Pwm::channel borrows the block mutably");

    hwtest::done("pwm")
}
