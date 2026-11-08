//! ADC test, needing no wiring. Results are reported over SDI print.
//!
//! - Temperature sensor and battery, which should read room temperature and
//!   the supply voltage.
//! - AIN1 = PA5 driven high and low as an output, at 1/4 and unity gain.
//! - AIN1 floating, pulled up and down; its digital input is off while the
//!   `Adc` holds it and back once it is dropped.
//! - The offset calibration and the conversion time.
//! - A scan of AIN0 (PA4), AIN1 (PA5) and AIN2 (PA12), pulled up, down and
//!   up, by DMA: one pass and sixteen, and a buffer that is not whole passes.
//!
//! The ADC runs at 4 MHz (32 MHz / 8, `SampleClock::Divide4`). The original
//! test read PA9 (AIN13); upstream has no AIN12/AIN13 (PA8/PA9), so PA5 stands
//! in.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_time::Instant;
use hal::adc::{Adc, Config, Error, Gain, SampleClock, Scan};
use hal::dma::StaticSampleBuffer;
use hal::gpio::{Input, Pull};
use hal::pac;
use hwtest::check;
use {panic_halt as _, qingke_rt::entry};

static ONE_PASS: StaticSampleBuffer<3> = StaticSampleBuffer::new([0; 3]);
static PASSES: StaticSampleBuffer<48> = StaticSampleBuffer::new([0; 48]);
static ODD: StaticSampleBuffer<4> = StaticSampleBuffer::new([0; 4]);

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
    let mut p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
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

    // PA5 as a driven output, read back through AIN1. `new_pin` floats the
    // pad, so the direction is set after it.
    for gain in [Gain::Div4, Gain::Div2, Gain::Unity] {
        let mut adc = Adc::new_pin(p.ADC.reborrow(), p.PA5.reborrow(), config(gain));
        g.pa_dir().modify(|w| w.set_pa_dir(5, true));
        g.pa_set().write(|w| w.set_pa_set(5, true));
        let hi = adc.read().unwrap_or(0);
        g.pa_clr().write(|w| w.set_pa_clr(5, true));
        let lo = adc.read().unwrap_or(0xffff);
        g.pa_dir().modify(|w| w.set_pa_dir(5, false));
        hwtest::println!(
            "PA5 {:?}: high raw {} = {} mV, low raw {} = {} mV",
            gain,
            hi,
            millivolts(hi, gain),
            lo,
            millivolts(lo, gain)
        );
        match gain {
            Gain::Div4 => {
                let mv = millivolts(hi, gain);
                check!((2900..3700).contains(&mv), "PA5 high at 1/4, mV in 2900..3700 ({})", mv);
            }
            Gain::Unity => {
                check!(hi >= 4090, "PA5 high at unity saturates ({})", hi);
                let mv = millivolts(lo, gain);
                check!(mv < 30, "PA5 low at unity, mV < 30 ({})", mv);
            }
            _ => {}
        }
    }

    // PA5 floating, then pulled; the pull is set after `new_pin`.
    {
        let mut adc = Adc::new_pin(p.ADC.reborrow(), p.PA5.reborrow(), config(Gain::Unity));
        pull(5, Pull::Up);
        delay_us(1000);
        let up = adc.read().unwrap_or(0);
        let dig = g.pa_pin().read().pa_pin(5);
        check!(up >= 4090, "PA5 pulled up reads full scale ({})", up);
        check!(!dig, "PA5 analog: digital input off");
        pull(5, Pull::Down);
        delay_us(1000);
        let down = adc.read().unwrap_or(0xffff);
        check!(down < 40, "PA5 pulled down reads ~0 ({})", down);

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
        pull(5, Pull::None);
    }

    // Dropping the `Adc` brings the digital input back.
    {
        let inp = Input::new(p.PA5.reborrow(), Pull::Up);
        delay_us(10);
        check!(inp.is_high(), "PA5 as Input with pull-up reads high");
    }

    // Scan, PA4 up, PA5 down, PA12 up. `Scan::new` floats the pads, so the
    // pulls are set after it.
    {
        let mut scan = Scan::new(
            p.ADC.reborrow(),
            (p.PA4.reborrow(), p.PA5.reborrow(), p.PA12.reborrow()),
            config(Gain::Unity),
        );
        pull(4, Pull::Up);
        pull(5, Pull::Down);
        pull(12, Pull::Up);
        delay_us(1000);
        let pattern = |buf: &[u16]| {
            buf.chunks(3)
                .all(|s| s[0] >= 4090 && s[1] < 40 && s[2] >= 4090)
        };

        let buf = ONE_PASS.take().unwrap();
        buf.fill(0xffff);
        let r = scan.read(buf).map_err(|(e, _)| e).and_then(|t| t.blocking_wait().map_err(|(e, _)| e));
        match r {
            Ok(buf) => check!(pattern(buf), "scan one pass: up, down, up ({:?})", buf),
            Err(e) => check!(false, "scan one pass ({:?})", e),
        };

        let buf = PASSES.take().unwrap();
        buf.fill(0xffff);
        let t0 = Instant::now();
        let r = scan.read(buf).map_err(|(e, _)| e).and_then(|t| t.blocking_wait().map_err(|(e, _)| e));
        let us = t0.elapsed().as_micros() as i32;
        match r {
            Ok(buf) => {
                check!(pattern(buf), "scan 16 passes: up, down, up");
                if !pattern(buf) {
                    hwtest::println!("  {:?}", buf);
                }
            }
            Err(e) => {
                check!(false, "scan 16 passes ({:?})", e);
            }
        }
        // 48 conversions of about 18 ADC clocks (7-clock sample window).
        check!((150..2_000).contains(&us), "  48 samples in 150..2000 us ({})", us);

        let r = scan.read(ODD.take().unwrap());
        check!(
            matches!(r, Err((Error::Buffer, _))),
            "scan into 4 samples for 3 channels: Error::Buffer"
        );

        pull(4, Pull::None);
        pull(5, Pull::None);
        pull(12, Pull::None);
    }

    hwtest::skip("PA9/AIN13 and PA8/AIN12", "not ADC pins upstream");
    hwtest::skip("differential", "no differential API");
    hwtest::skip("async read, async DMA, ring buffer, paced DMA", "polled API only");

    hwtest::done("adc")
}
