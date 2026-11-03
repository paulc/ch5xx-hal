//! ADC test, needing no wiring. Results are reported over SDI print.
//!
//! - Temperature sensor and VBAT, which should read room temperature and the
//!   supply voltage.
//! - AIN12 = PA8 (the LED) driven high and low as an output.
//! - AIN5 = PA15 floating, pulled up and down.
//! - The AIN0 (PA4) / AIN2 (PA12) differential pair, pulled opposite ways.
//! - The offset calibration, conversion time, and that an analog pin's digital
//!   input is off and comes back when the pin is made an `Input`.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Instant;
use hal::adc::{Adc, AdcChannel, Config, Gain, Temperature, Vbat};
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::{Input, Pull};
use hal::pac;
use panic_halt as _;

struct Checks {
    pass: u32,
    fail: u32,
}

impl Checks {
    fn check(&mut self, name: &str, ok: bool, value: i32) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        hal::println!("{} {}: {}", if ok { "PASS" } else { "FAIL" }, name, value);
    }
}

/// Set PAn's pull without touching its direction.
fn pull(n: usize, p: Pull) {
    let g = pac::GPIO;
    g.pa_pu().modify(|w| w.set_pa_pu(n, p == Pull::Up));
    g.pa_pd_drv().modify(|w| w.set_pa_pd_drv(n, p == Pull::Down));
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

    let mut adc = Adc::new(p.ADC, Config::default());

    // Internal channels.
    let mut ts = adc.enable_temperature();
    delay.delay_us(100);
    let mut sum = 0u32;
    for _ in 0..16 {
        sum += adc.convert(&mut ts) as u32;
    }
    let raw = (sum / 16) as u16;
    let c25 = unsafe { core::ptr::read_volatile(0x0007_f014 as *const u32) };
    hal::println!("TS raw {} cal {:#010x}", raw, c25);
    let mc = Temperature::to_millicelsius(raw);
    c.check("temperature mdegC in 5..50 degC", (5_000..50_000).contains(&mc), mc);

    let mut vbat = adc.enable_vbat();
    let raw = adc.convert(&mut vbat);
    let mv = Vbat::to_millivolts(raw) as i32;
    hal::println!("VBAT raw {}", raw);
    c.check("VBAT mV in 2700..3700", (2700..3700).contains(&mv), mv);

    // PA8 as a driven output, read back through AIN12. The channel is
    // degraded first so converting does not make the pin an input again.
    let mut led = p.PA8.reborrow().degrade_adc();
    let g = pac::GPIO;
    g.pa_dir().modify(|w| w.set_pa_dir(8, true));
    for (gain, name) in [(Gain::Quarter, "1/4"), (Gain::Half, "1/2"), (Gain::X1, "1x")] {
        adc.set_gain(gain);
        g.pa_set().write(|w| w.set_pa_set(8, true));
        let hi = adc.convert(&mut led);
        g.pa_clr().write(|w| w.set_pa_clr(8, true));
        let lo = adc.convert(&mut led);
        hal::println!(
            "PA8 {}: high raw {} = {} mV, low raw {} = {} mV",
            name,
            hi,
            adc.to_millivolts(hi),
            lo,
            adc.to_millivolts(lo)
        );
        match gain {
            Gain::Quarter => {
                let mv = adc.to_millivolts(hi);
                c.check("PA8 high at 1/4, mV in 2900..3700", (2900..3700).contains(&mv), mv);
            }
            Gain::X1 => {
                c.check("PA8 high at 1x saturates", hi >= 4090, hi as i32);
                let mv = adc.to_millivolts(lo);
                c.check("PA8 low at 1x, mV < 30", mv < 30, mv);
            }
            _ => {}
        }
    }
    g.pa_set().write(|w| w.set_pa_set(8, true));
    g.pa_dir().modify(|w| w.set_pa_dir(8, false));

    // PA15 floating, then pulled; the pull is set after the channel's setup.
    adc.set_gain(Gain::X1);
    let mut ain5 = p.PA15.reborrow().degrade_adc();
    pull(15, Pull::Up);
    delay.delay_ms(1);
    let up = adc.convert(&mut ain5);
    let dig = g.pa_pin().read().pa_pin(15);
    c.check("PA15 pulled up reads full scale", up >= 4090, up as i32);
    c.check("PA15 analog: digital input off", !dig, dig as i32);
    pull(15, Pull::Down);
    delay.delay_ms(1);
    let down = adc.convert(&mut ain5);
    c.check("PA15 pulled down reads ~0", down < 40, down as i32);

    let off = adc.calibrate();
    c.check("offset calibration |off| < 64", off.abs() < 64, off as i32);

    // Conversion time at the default 3.2 MHz ADC clock: ~17 clocks each, plus
    // the driver's register writes.
    let t0 = Instant::now();
    for _ in 0..1000 {
        core::hint::black_box(adc.convert(&mut ain5));
    }
    let us = t0.elapsed().as_micros() as i32;
    c.check("1000 conversions in 6..15 ms (us)", (6_000..15_000).contains(&us), us);

    // Differential AIN0 (PA4) - AIN2 (PA12).
    let mut ain0 = p.PA4.reborrow().degrade_adc();
    let mut ain2 = p.PA12.reborrow().degrade_adc();
    pull(4, Pull::Up);
    pull(12, Pull::Down);
    delay.delay_ms(1);
    let pos = adc.convert_differential(&mut ain0, &mut ain2);
    pull(4, Pull::Down);
    pull(12, Pull::Up);
    delay.delay_ms(1);
    let neg = adc.convert_differential(&mut ain0, &mut ain2);
    pull(4, Pull::Down);
    pull(12, Pull::Down);
    delay.delay_ms(1);
    let zero = adc.convert_differential(&mut ain0, &mut ain2);
    hal::println!(
        "diff +: {} ({} mV), -: {} ({} mV), 0: {} ({} mV)",
        pos,
        adc.to_millivolts_differential(pos),
        neg,
        adc.to_millivolts_differential(neg),
        zero,
        adc.to_millivolts_differential(zero)
    );
    c.check("diff AIN0 high, AIN2 low saturates high", pos >= 4090, pos as i32);
    c.check("diff AIN0 low, AIN2 high saturates low", neg < 8, neg as i32);
    c.check("diff both low near 0x800", (0x7c0..0x840).contains(&zero), zero as i32);

    // An Input brings the digital input back.
    drop(ain5);
    let inp = Input::new(p.PA15.reborrow(), Pull::Up);
    delay.delay_us(10);
    c.check(
        "PA15 as Input with pull-up reads high",
        inp.is_high(),
        inp.is_high() as i32,
    );
    drop(inp);
    pull(4, Pull::None);
    pull(12, Pull::None);

    hal::println!("ADC: {} passed, {} failed", c.pass, c.fail);
    loop {
        delay.delay_ms(1000);
    }
}
