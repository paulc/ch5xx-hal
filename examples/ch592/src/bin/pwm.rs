//! PWM test for `TMRx` (`SimplePwm`) and `PWMX`, needing no wiring: each
//! output's period and high time are measured by timing its own pin's edges
//! against SysTick. Results are reported over SDI print.
//!
//! Pins: TMR0 PA9, TMR1 PA10, TMR2 PA11, TMR3 PB22 (the KEY button: don't
//! press it while this runs), PWM4 PA12, PWM5 PA13, PWM7 PB4, PWM9 PB7.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embedded_hal::pwm::SetDutyCycle;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::pac;
use hal::time::Hertz;
use hal::timer::pwmx::{Period, Pwmx};
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use hal::timer::OutputPolarity;
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

fn now() -> u32 {
    pac::SYSTICK.stk_cntl().read().cntl()
}

/// Wait for the pin to reach `high`, up to `timeout` cycles.
fn wait(p: P, high: bool, timeout: u32) -> Option<u32> {
    let t0 = now();
    loop {
        if level(p) == high {
            return Some(now());
        }
        if now().wrapping_sub(t0) > timeout {
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
        hi += fall.wrapping_sub(rise);
        per += next.wrapping_sub(rise);
        rise = next;
    }
    Ok((per / 8, hi / 8))
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
        hal::println!("{} {}", if ok { "PASS" } else { "FAIL" }, name);
    }

    /// Check the pin's period and duty (in permille) to within 1.5%.
    fn pwm(&mut self, name: &str, p: P, period: u32, permille: u32) {
        match measure(p) {
            Ok((per, hi)) => {
                let d = hi * 1000 / per;
                hal::println!(
                    "  {}: period {} (want {}), duty {}‰ (want {}‰)",
                    name,
                    per,
                    period,
                    d,
                    permille
                );
                let per_ok = per.abs_diff(period) * 1000 <= period * 15;
                self.check(name, per_ok && d.abs_diff(permille) <= 15);
            }
            Err(level) => {
                hal::println!("  {}: constant {}", name, if level { "high" } else { "low" });
                self.check(name, false);
            }
        }
    }

    /// Check the pin stays at `high`.
    fn constant(&mut self, name: &str, p: P, high: bool) {
        let r = measure(p);
        if let Ok((per, hi)) = r {
            hal::println!("  {}: toggling, period {} high {}", name, per, hi);
        }
        self.check(name, r == Err(high));
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_60MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut delay = Delay;
    let mut c = Checks { pass: 0, fail: 0 };
    let hclk = hal::rcc::clocks().sysclk.0;
    hal::println!("HCLK {} Hz", hclk);

    // ---- TMR0 on PA9
    {
        let pin = PwmPin::new::<0>(p.PA9.reborrow());
        let mut pwm = SimplePwm::new(p.TMR0.reborrow(), pin, Hertz::khz(10));
        let max = pwm.get_max_duty();
        c.check("TMR0 max duty = HCLK / 10 kHz", max == hclk / 10_000);
        pwm.set_duty(max / 4);
        pwm.enable();
        c.pwm("TMR0 10 kHz 25%", P::A(9), max, 250);
        pwm.set_duty(max * 3 / 4);
        c.pwm("TMR0 10 kHz 75%", P::A(9), max, 750);
        pwm.set_polarity(OutputPolarity::ActiveLow);
        c.pwm("TMR0 active low 75% -> high 25%", P::A(9), max, 250);
        pwm.set_polarity(OutputPolarity::ActiveHigh);
        pwm.set_frequency(Hertz::khz(1));
        c.pwm("TMR0 1 kHz keeps 75%", P::A(9), hclk / 1000, 750);
        pwm.set_duty(0);
        delay.delay_ms(2);
        c.constant("TMR0 duty 0 is low", P::A(9), false);
        pwm.set_duty(pwm.get_max_duty());
        delay.delay_ms(2);
        c.constant("TMR0 duty max is high", P::A(9), true);
        // Overfill the 8-entry FIFO; the last value should win.
        for i in 0..20 {
            pwm.set_duty(pwm.get_max_duty() * i / 40);
        }
        pwm.set_duty(pwm.get_max_duty() / 10);
        delay.delay_ms(40);
        c.pwm("TMR0 after FIFO overfill: 10%", P::A(9), hclk / 1000, 100);
        pwm.set_duty_cycle_percent(30).unwrap();
        delay.delay_ms(10);
        c.pwm("TMR0 SetDutyCycle 30%", P::A(9), hclk / 1000, 300);
        pwm.disable();
        delay.delay_ms(2);
        c.constant("TMR0 disabled idles low", P::A(9), false);
    }

    // ---- TMR1-3
    {
        let pin = PwmPin::new::<0>(p.PA10.reborrow());
        let mut pwm = SimplePwm::new(p.TMR1.reborrow(), pin, Hertz::khz(20));
        pwm.set_duty(pwm.get_max_duty() / 2);
        pwm.enable();
        c.pwm("TMR1 20 kHz 50% on PA10", P::A(10), hclk / 20_000, 500);
    }
    {
        let pin = PwmPin::new::<0>(p.PA11.reborrow());
        let mut pwm = SimplePwm::new(p.TMR2.reborrow(), pin, Hertz::khz(20));
        pwm.set_duty(pwm.get_max_duty() / 5);
        pwm.enable();
        c.pwm("TMR2 20 kHz 20% on PA11", P::A(11), hclk / 20_000, 200);
    }
    {
        let pin = PwmPin::new::<0>(p.PB22.reborrow());
        let mut pwm = SimplePwm::new(p.TMR3.reborrow(), pin, Hertz::khz(5));
        pwm.set_duty(pwm.get_max_duty() * 4 / 5);
        pwm.enable();
        c.pwm("TMR3 5 kHz 80% on PB22", P::B(22), hclk / 5_000, 800);
    }

    // ---- PWMX, 8-bit
    {
        let pwmx = Pwmx::new(p.PWMX.reborrow(), Period::Cycles256, Hertz::khz(10));
        let f = pwmx.frequency().0;
        let div = pac::PWMX.clock_div().read().clock_div() as u32;
        hal::println!("PWMX 8-bit: {} Hz, div {}", f, div);
        c.check("PWMX 8-bit div = 23", div == 23);
        let per = div * 256;
        let mut ch4 = pwmx.ch4::<0>(p.PA12.reborrow());
        let mut ch5 = pwmx.ch5::<0>(p.PA13.reborrow());
        let mut ch7 = pwmx.ch7::<0>(p.PB4.reborrow());
        let mut ch9 = pwmx.ch9::<0>(p.PB7.reborrow());
        ch4.set_duty(64);
        ch5.set_duty(128);
        ch7.set_duty(192);
        ch7.set_polarity(OutputPolarity::ActiveLow);
        ch9.set_duty(32);
        for ch in [&mut ch4, &mut ch5, &mut ch7, &mut ch9] {
            ch.enable();
        }
        c.pwm("PWM4 25%", P::A(12), per, 250);
        c.pwm("PWM5 50%", P::A(13), per, 500);
        c.pwm("PWM7 active low 75% -> high 25%", P::B(4), per, 250);
        c.pwm("PWM9 12.5%", P::B(7), per, 125);
        ch9.set_duty_cycle_fraction(1, 2).unwrap();
        c.pwm("PWM9 SetDutyCycle 1/2", P::B(7), per, 500);
        ch5.set_duty(0);
        delay.delay_ms(1);
        c.constant("PWM5 duty 0 is low", P::A(13), false);
        ch4.disable();
        delay.delay_ms(1);
        c.constant("PWM4 disabled idles low", P::A(12), false);
    }

    // ---- PWMX, 16-bit
    {
        let pwmx = Pwmx::new(p.PWMX.reborrow(), Period::Wide(6000), Hertz::khz(1));
        let div = pac::PWMX.clock_div().read().clock_div() as u32;
        hal::println!("PWMX 16-bit: {} Hz, div {}", pwmx.frequency().0, div);
        c.check("PWMX 16-bit div = 10", div == 10);
        let per = div * 6000;
        let mut ch4 = pwmx.ch4::<0>(p.PA12.reborrow());
        let mut ch5 = pwmx.ch5::<0>(p.PA13.reborrow());
        let mut ch7 = pwmx.ch7::<0>(p.PB4.reborrow());
        let mut ch9 = pwmx.ch9::<0>(p.PB7.reborrow());
        ch4.set_duty(1500);
        ch5.set_duty(3000);
        ch7.set_duty(600);
        ch9.set_duty(4500);
        for ch in [&mut ch4, &mut ch5, &mut ch7, &mut ch9] {
            ch.enable();
        }
        c.pwm("PWM4 16-bit 25%", P::A(12), per, 250);
        c.pwm("PWM5 16-bit 50%", P::A(13), per, 500);
        c.pwm("PWM7 16-bit 10%", P::B(4), per, 100);
        c.pwm("PWM9 16-bit 75%", P::B(7), per, 750);
    }

    hal::println!("PWM: {} passed, {} failed", c.pass, c.fail);
    loop {
        delay.delay_ms(1000);
    }
}
