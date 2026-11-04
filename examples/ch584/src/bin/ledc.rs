//! LEDC test on LEDC = PA4 and LED4 = PA5, the only LEDC pins the CH584F
//! bonds. Needs no wiring: results are reported over SDI print.
//!
//! While a DMA write runs, a loop in RAM samples both pins, and the lane-4
//! bits are decoded from them at the rising clock edges. That checks the data
//! format (8 lanes, both bit orders, inverted) and the bounce buffer. Other
//! lane counts, the bit rate, DMA chunking, dropping a write and the blocking
//! driver are checked by counting edges and timing.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::{Instant, Timer};
use hal::debug::SDIPrint;
use hal::gpio::{Input, Pull};
use hal::ledc::{self, BitOrder, DataPins, Lanes, Ledc};
use hal::time::Hertz;
use hal::{bind_interrupts, pac};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    LED => ledc::InterruptHandler;
});

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

const N: usize = 1024;
static mut SAMPLES: [u8; N] = [0; N];

/// Record each change of (PA4, PA5) as `clk | data << 1`, until `quiet`
/// polls pass without one after the first change. In RAM, so flash wait
/// states do not slow it.
#[unsafe(link_section = ".highcode.ledc_test.sample")]
#[inline(never)]
fn sample(quiet: u32) -> usize {
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(SAMPLES) };
    let mut n = 0;
    let mut last = 0xff;
    let mut idle = 0;
    let mut polls = 0u32;
    loop {
        let v = ((pac::GPIO.pa_pin().read().0 >> 4) & 3) as u8;
        if v != last {
            if n < N {
                buf[n] = v;
                n += 1;
            }
            last = v;
            idle = 0;
        } else {
            idle += 1;
            if n > 1 && idle > quiet {
                break;
            }
        }
        polls += 1;
        // Nothing happened at all: give up.
        if n <= 1 && polls > 2_000_000 {
            break;
        }
    }
    n
}

/// Lane-4 bits from the samples: the data level at each rising clock edge
/// (the data change on falling edges). Returns (bits, clock edges).
fn decode(n: usize, bits: &mut [u8]) -> (usize, usize) {
    let buf = unsafe { &*core::ptr::addr_of!(SAMPLES) };
    let (mut k, mut edges) = (0, 0);
    for i in 1..n {
        let (prev, v) = (buf[i - 1], buf[i]);
        if (prev ^ v) & 1 != 0 {
            edges += 1;
            if v & 1 == 1 && k < bits.len() {
                bits[k] = (v >> 1) & 1;
                k += 1;
            }
        }
    }
    (k, edges)
}

/// Words that put `lane4` on lane 4 with 8 lanes, and ones on every other
/// lane, so leaks from them show.
fn words_for(lane4: &[u8], out: &mut [u32]) -> usize {
    for (i, &b) in lane4.iter().enumerate() {
        out[2 * i] = 0xffff_ffff;
        out[2 * i + 1] = 0xffff_ff00 | b as u32;
    }
    2 * lane4.len()
}

fn expected(lane4: &[u8], msb_first: bool, invert: bool, out: &mut [u8]) -> usize {
    let mut k = 0;
    for &b in lane4 {
        for i in 0..8 {
            let bit = if msb_first { (b >> (7 - i)) & 1 } else { (b >> i) & 1 };
            out[k] = bit ^ invert as u8;
            k += 1;
        }
    }
    k
}

fn show(bits: &[u8]) -> heapless_str::Bits<'_> {
    heapless_str::Bits(bits)
}

mod heapless_str {
    pub struct Bits<'a>(pub &'a [u8]);
    impl core::fmt::Display for Bits<'_> {
        fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
            for &b in self.0 {
                f.write_str(if b != 0 { "1" } else { "0" })?;
            }
            Ok(())
        }
    }
}

/// Lane-4 bytes. The first is a lead-in: the sampler starts a few bit times
/// after the DMA, so up to 8 leading bits may be missed.
const LANE4: [u8; 7] = [0x00, 0xa5, 0x3c, 0x01, 0x80, 0xff, 0x00];

/// Do the decoded bits match `want`, allowing for missed lead-in bits?
fn matches_tail(got: &[u8], want: &[u8]) -> bool {
    got.len() <= want.len() && got.len() + 8 >= want.len() && got == &want[want.len() - got.len()..]
}

static FLASH_WORDS: [u32; 14] = {
    let mut w = [0u32; 14];
    let mut i = 0;
    while i < 7 {
        w[2 * i] = 0xffff_ffff;
        w[2 * i + 1] = 0xffff_ff00 | LANE4[i] as u32;
        i += 1;
    }
    w
};

static mut BIG: [u32; 5000] = [0; 5000];

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut c = Checks { pass: 0, fail: 0 };

    // The slowest bit rate: HCLK / 254 = 245.7 kHz, about 16 samples a bit.
    let mut config = ledc::Config::default();
    config.lanes = Lanes::Eight;
    config.frequency = Hertz::khz(1);
    let mut led = Ledc::new(
        p.LED.reborrow(),
        p.PA4.reborrow(),
        DataPins::new().lane(p.PA5.reborrow()),
        Irqs,
        config,
    );
    let f = led.get_current_config().frequency.0;
    hal::println!("bit rate {} Hz", f);
    c.check("slowest bit rate is HCLK / 254", f == 62_400_000 / 254);
    c.check("idle: clock high", pac::GPIO.pa_pin().read().0 & (1 << 4) != 0);

    let mut words = [0u32; 16];
    let nw = words_for(&LANE4, &mut words);
    let mut want = [0u8; 64];
    let mut got = [0u8; 96];

    // Lane-4 data, 8 lanes, both bit orders and inverted.
    for (name, order, invert) in [
        ("8 lanes, MSB first", BitOrder::MsbFirst, false),
        ("8 lanes, LSB first", BitOrder::LsbFirst, false),
        ("8 lanes, MSB first, inverted", BitOrder::MsbFirst, true),
    ] {
        config.bit_order = order;
        config.invert = invert;
        led.set_config(&config);
        let ne = expected(&LANE4, order == BitOrder::MsbFirst, invert, &mut want);
        let (_, n) = join(led.write(&words[..nw]), async { sample(400) }).await;
        let (k, edges) = decode(n, &mut got);
        hal::println!("{}: {} edges, lane 4 {}", name, edges, show(&got[..k]));
        c.check(name, matches_tail(&got[..k], &want[..ne]) && (2 * k - 1..=2 * k).contains(&edges));
    }
    config.bit_order = BitOrder::MsbFirst;
    config.invert = false;
    led.set_config(&config);

    // From flash, through the bounce buffer.
    let ne = expected(&LANE4, true, false, &mut want);
    let (_, n) = join(led.write(&FLASH_WORDS), async { sample(400) }).await;
    let (k, _) = decode(n, &mut got);
    c.check("write from flash (bounce buffer)", matches_tail(&got[..k], &want[..ne]));

    // Other lane counts: 3 words of ones are 96, 48 or 24 bit times (less
    // up to 8 missed at the start); lane 4 is not used and stays low. Without
    // a lane-4 pin, as those modes leave it out, and pulled down so the check
    // does not read a floating pad. (Input has no drop; Ledc sets the pin up
    // again when it next takes it.)
    let big = unsafe { &mut *core::ptr::addr_of_mut!(BIG) };
    for (i, w) in big.iter_mut().enumerate() {
        *w = (i as u32).wrapping_mul(0x9e37_79b9);
    }
    for (lanes, bits) in [(Lanes::One, 96), (Lanes::Two, 48), (Lanes::Four, 24)] {
        drop(led);
        let _ = Input::new(p.PA5.reborrow(), Pull::Down);
        config.lanes = lanes;
        led = Ledc::new(p.LED.reborrow(), p.PA4.reborrow(), DataPins::new(), Irqs, config);
        let ones = [0xffff_ffffu32; 3];
        let (_, n) = join(led.write(&ones), async { sample(400) }).await;
        let (k, _) = decode(n, &mut got);
        let lane4_ones = got[..k].iter().filter(|&&b| b != 0).count();
        hal::println!("{:?}: {} bit times", lanes, k);
        c.check("  bit times per 3 words", k <= bits && k + 8 >= bits);
        c.check("  LED4 unused, low", lane4_ones == 0);
    }

    // Bit rate, with 1 lane so the DMA always keeps up: 200 words are 6400
    // bit times; 2 MHz requested gives HCLK / 32 = 1.95 MHz, 3282 us.
    drop(led);
    config.lanes = Lanes::One;
    config.frequency = Hertz::mhz(2);
    led = Ledc::new(p.LED.reborrow(), p.PA4.reborrow(), DataPins::new(), Irqs, config);
    let f = led.get_current_config().frequency.0;
    let t0 = Instant::now();
    led.write(&big[..200]).await;
    let us = t0.elapsed().as_micros();
    hal::println!("2 MHz requested: {} Hz, 6400 bit times in {} us", f, us);
    c.check("2 MHz rounds down to HCLK / 32", f == 1_950_000);
    c.check("  6400 bit times take 3282 us (-0 +60 us)", (3282..=3342).contains(&us));

    // Chunking: 5000 words is more than one DMA run (4094 words). At HCLK / 2
    // that is 160 000 bit times, 5128 us.
    config.frequency = Hertz::mhz(100);
    led.set_config(&config);
    let f = led.get_current_config().frequency.0;
    let t0 = Instant::now();
    led.write(&big[..]).await;
    let us = t0.elapsed().as_micros();
    hal::println!("5000 words at {} Hz in {} us", f, us);
    c.check("fastest rate is HCLK / 2", f == 31_200_000);
    // The output pauses while the executor wakes to start the second run.
    c.check("  5000 words in two DMA runs: 5128 us (-0 +120 us)", (5128..=5248).contains(&us));

    drop(led);
    config.lanes = Lanes::Eight;
    let mut led = Ledc::new(
        p.LED.reborrow(),
        p.PA4.reborrow(),
        DataPins::new().lane(p.PA5.reborrow()),
        Irqs,
        config,
    );

    // With 8 lanes a word lasts only 4 bit times: at HCLK / 2 the DMA falls
    // behind and the clock pauses. Report how much.
    let t0 = Instant::now();
    led.write(&big[..1000]).await;
    let us = t0.elapsed().as_micros();
    hal::println!("8 lanes, 1000 words at HCLK / 2: {} us (128 us if never starved)", us);
    c.check("  completes", !led.is_busy());

    // Empty write.
    let t0 = Instant::now();
    led.write(&[]).await;
    c.check("empty write returns at once", t0.elapsed().as_micros() < 50);

    // Dropping a write stops the output at once.
    config.frequency = Hertz::khz(1);
    led.set_config(&config);
    let r = select(led.write(&big[..]), Timer::after_millis(2)).await;
    let busy = led.is_busy();
    let clk_high = pac::GPIO.pa_pin().read().0 & (1 << 4) != 0;
    hal::println!("after the drop: busy {}, clock high {}, status {:03x}", busy, clk_high, pac::LED.status().read().0);
    c.check("dropped write: the timer won", matches!(r, Either::Second(())));
    c.check("  not busy", !busy);
    c.check("  clock back high", clk_high);
    let n = sample(400);
    c.check("  no clock edges afterwards", n <= 1);
    let ne = expected(&LANE4, true, false, &mut want);
    let (_, n) = join(led.write(&words[..nw]), async { sample(400) }).await;
    let (k, _) = decode(n, &mut got);
    c.check("  next write intact", matches_tail(&got[..k], &want[..ne]));
    drop(led);

    // Blocking driver: 14 words with 8 lanes are 56 bit times, 228 us.
    let mut led = Ledc::new_blocking(p.LED, p.PA4, DataPins::new().lane(p.PA5), config);
    let t0 = Instant::now();
    led.blocking_write(&words[..nw]);
    let us = t0.elapsed().as_micros();
    hal::println!("blocking write of 56 bit times: {} us", us);
    c.check("blocking write returns when done (228-260 us)", (228..=260).contains(&us));
    c.check("  not busy, clock high", !led.is_busy() && pac::GPIO.pa_pin().read().0 & (1 << 4) != 0);
    let t0 = Instant::now();
    led.blocking_write(&big[..1000]);
    let us = t0.elapsed().as_micros();
    c.check("blocking write of 4000 bit times: 16 282 us (-0 +40 us)", (16_282..=16_322).contains(&us));
    hal::println!("  ({} us)", us);
    drop(led);

    hal::println!("ledc: {} passed, {} failed", c.pass, c.fail);
    loop {
        Timer::after_secs(1).await;
    }
}
