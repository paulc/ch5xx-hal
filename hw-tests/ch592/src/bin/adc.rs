//! ADC test, needing no wiring. Results are reported over SDI print.
//!
//! - Temperature sensor and battery, which should read room temperature and
//!   the supply voltage.
//! - AIN5 = PA15 driven high and low as an output, at 1/4 and unity gain.
//! - AIN5 floating, pulled up and down; its digital input is off while the
//!   `Adc` holds it and back once it is dropped.
//! - The offset calibration and the conversion time.
//!
//! The ADC runs at 4 MHz (32 MHz / 8, `SampleClock::Divide4`). The original
//! test read PA8 (AIN12, the LED); upstream has no AIN12/AIN13 (PA8/PA9), so
//! PA15 stands in. CH592 has no scan sequencer.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_time::Instant;
use hal::adc::{Adc, Config, Gain, SampleClock};
use hal::gpio::{Input, Pull};
use hal::pac;
use hwtest::check;
use {panic_halt as _, qingke_rt::entry};

const VREF_MV: i32 = 1050;
const MID: i32 = 0x800;

/// Millivolts from a single-ended sample, as the vendor's
/// `ADC_VoltConverSignalPGA_*`.
fn millivolts(raw: u16, gain: Gain) -> i32 {
    let (p, q) = match gain {
        Gain::Div4 => (4, 1),
        Gain::Div2 => (2, 1),
        Gain::Unity => (1, 1),
        Gain::X2 => (1, 2),
        Gain::X4 => (1, 4),
        Gain::X8 => (1, 8),
        Gain::X16 => (1, 16),
    };
    VREF_MV * (raw as i32 * p + MID * (q - p)) / (MID * q)
}

fn config(gain: Gain) -> Config {
    let mut c = Config::default();
    c.gain = gain;
    c.sample_clock = SampleClock::Divide4;
    c
}

/// Set PAn's pull without touching its direction.
fn pull(n: usize, p: Pull) {
    let g = pac::GPIO;
    g.pa_pu().modify(|w| w.set_pa_pu(n, p == Pull::Up));
    g.pa_pd_drv().modify(|w| w.set_pa_pd_drv(n, p == Pull::Down));
}

fn delay_us(us: u32) {
    let t0 = Instant::now();
    while t0.elapsed().as_micros() < us as u64 {}
}

#[entry]
fn main() -> ! {
    let mut p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();
    let g = pac::GPIO;

    // Internal channels.
    {
        let mut adc = Adc::new_temperature(p.ADC.reborrow());
        delay_us(100);
        let mut sum = 0u32;
        let mut ok = true;
        for _ in 0..16 {
            match adc.read() {
                Ok(v) => sum += v as u32,
                Err(_) => ok = false,
            }
        }
        let raw = (sum / 16) as u16;
        let deg = adc.temperature(raw);
        let cal = unsafe { core::ptr::read_volatile(0x0007_f014 as *const u32) };
        hwtest::println!("TS raw {} cal {:#010x}", raw, cal);
        check!(ok && (5..50).contains(&deg), "temperature in 5..50 degC ({})", deg);
    }
    {
        let mut adc = Adc::new_battery(p.ADC.reborrow(), config(Gain::Div4));
        let raw = adc.read();
        let mv = raw.map(|r| millivolts(r, Gain::Div4)).unwrap_or(-1);
        hwtest::println!("VBAT raw {:?}", raw);
        check!((2700..3700).contains(&mv), "VBAT in 2700..3700 mV ({})", mv);
    }

    // PA15 as a driven output, read back through AIN1. `new_pin` floats the
    // pad, so the direction is set after it.
    for gain in [Gain::Div4, Gain::Div2, Gain::Unity] {
        let mut adc = Adc::new_pin(p.ADC.reborrow(), p.PA15.reborrow(), config(gain));
        g.pa_dir().modify(|w| w.set_pa_dir(15, true));
        g.pa_set().write(|w| w.set_pa_set(15, true));
        let hi = adc.read().unwrap_or(0);
        g.pa_clr().write(|w| w.set_pa_clr(15, true));
        let lo = adc.read().unwrap_or(0xffff);
        g.pa_dir().modify(|w| w.set_pa_dir(15, false));
        hwtest::println!(
            "PA15 {:?}: high raw {} = {} mV, low raw {} = {} mV",
            gain,
            hi,
            millivolts(hi, gain),
            lo,
            millivolts(lo, gain)
        );
        match gain {
            Gain::Div4 => {
                let mv = millivolts(hi, gain);
                check!((2900..3700).contains(&mv), "PA15 high at 1/4, mV in 2900..3700 ({})", mv);
            }
            Gain::Unity => {
                check!(hi >= 4090, "PA15 high at unity saturates ({})", hi);
                let mv = millivolts(lo, gain);
                check!(mv < 30, "PA15 low at unity, mV < 30 ({})", mv);
            }
            _ => {}
        }
    }

    // PA15 floating, then pulled; the pull is set after `new_pin`.
    {
        let mut adc = Adc::new_pin(p.ADC.reborrow(), p.PA15.reborrow(), config(Gain::Unity));
        pull(15, Pull::Up);
        delay_us(1000);
        let up = adc.read().unwrap_or(0);
        let dig = g.pa_pin().read().pa_pin(15);
        check!(up >= 4090, "PA15 pulled up reads full scale ({})", up);
        check!(!dig, "PA15 analog: digital input off");
        pull(15, Pull::Down);
        delay_us(1000);
        let down = adc.read().unwrap_or(0xffff);
        check!(down < 40, "PA15 pulled down reads ~0 ({})", down);

        let off = adc.calibrate();
        check!(
            matches!(off, Ok(o) if o.abs() < 64),
            "offset calibration |off| < 64 ({:?})",
            off
        );

        // `read` is two conversions, about 4.5 us each at 4 MHz, plus the
        // polling.
        let t0 = Instant::now();
        for _ in 0..1000 {
            core::hint::black_box(adc.read().ok());
        }
        let us = t0.elapsed().as_micros() as i32;
        check!((8_000..30_000).contains(&us), "1000 reads in 8..30 ms ({} us)", us);
        pull(15, Pull::None);
    }

    // Dropping the `Adc` brings the digital input back.
    {
        let inp = Input::new(p.PA15.reborrow(), Pull::Up);
        delay_us(10);
        check!(inp.is_high(), "PA15 as Input with pull-up reads high");
    }

    hwtest::skip("PA8/AIN12, PA9/AIN13, PB0/AIN8, PB6/AIN9", "not ADC pins upstream");
    hwtest::skip("scan", "no scan sequencer on CH592");
    hwtest::skip("differential", "no differential API");
    hwtest::skip("async read, async DMA, ring buffer, paced DMA", "polled API only");

    hwtest::done("adc")
}
