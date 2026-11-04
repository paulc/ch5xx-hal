//! Software I2C test, needing no device: results are reported over SDI print.
//!
//! TMR3 clocks the bus. SCL is PA10 and SDA PA11 (the 32 kHz crystal pins,
//! with the crystal off), with the
//! pins' pull-ups, and TMR1/TMR2 capture both lines by DMA, so the test can
//! check the waveform of a transfer nobody acknowledges:
//!
//! - start, address 0x55 + write (0xAA), NACK, stop, at 100 kHz and 50 kHz;
//! - [`Error::Bus`] with SDA held low, [`Error::Timeout`] with SCL held low,
//!   and a transfer waiting for SCL to be released at the start;
//! - with the PA14-PA15 jumper, SCL on PA14 stretched by PA15 in the middle
//!   of the address byte.
//!
//! Reading and acknowledged writes need a device: see `soft_i2c_sensors`.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3};
use embassy_time::{Instant, Timer};
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::{Flex, Pull};
use hal::mode::Async;
use hal::soft_i2c::{self, Error, SoftI2c};
use hal::time::Hertz;
use hal::timer::input_capture::{Capture, CapturePin, Config, InputCapture, InputCaptureMode};
use hal::{bind_interrupts, pac, peripherals, Peri};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    TMR1 => hal::timer::InterruptHandler<peripherals::TMR1>;
    TMR2 => hal::timer::InterruptHandler<peripherals::TMR2>;
    TMR3 => hal::soft_i2c::InterruptHandler<peripherals::TMR3>;
});

struct Checks {
    pass: u32,
    fail: u32,
}

impl Checks {
    fn check(&mut self, name: &str, ok: bool, value: i64) {
        if ok {
            self.pass += 1;
        } else {
            self.fail += 1;
        }
        hal::println!("{} {}: {}", if ok { "PASS" } else { "FAIL" }, name, value);
    }
}

/// Levels of a line, in steps (half SCL periods), from a capture DMA buffer.
struct Levels {
    v: [(bool, u32); 32],
    n: usize,
}

impl Levels {
    fn from<T: hal::timer::Instance, M: hal::mode::Mode>(
        cap: &mut InputCapture<'_, T, M>,
        raw: &[u32],
        step: u32,
    ) -> Self {
        let mut l = Levels {
            v: [(false, 0); 32],
            n: 0,
        };
        for m in cap.decode(raw) {
            let (high, t) = match m {
                Ok(Capture::High(t)) => (true, t),
                Ok(Capture::Low(t)) => (false, t),
                _ => continue,
            };
            if l.n < l.v.len() {
                l.v[l.n] = (high, (t + step / 2) / step);
                l.n += 1;
            }
        }
        l
    }

    fn get(&self) -> &[(bool, u32)] {
        &self.v[..self.n]
    }

    fn print(&self, name: &str) {
        let mut s = [0u8; 64];
        let mut k = 0;
        for &(high, steps) in self.get() {
            if k + 4 > s.len() {
                break;
            }
            s[k] = if high { b'H' } else { b'L' };
            let d = steps.min(99);
            if d >= 10 {
                s[k + 1] = b'0' + (d / 10) as u8;
                k += 1;
            }
            s[k + 1] = b'0' + (d % 10) as u8;
            s[k + 2] = b' ';
            k += 3;
        }
        hal::println!("  {}: {}", name, core::str::from_utf8(&s[..k]).unwrap());
    }
}

/// SDA for start, 0xAA (0x55 + write), NACK, stop: low and high steps after
/// the idle level.
const SDA_0X55: [(bool, u32); 11] = [
    (false, 1),
    (true, 2),
    (false, 2),
    (true, 2),
    (false, 2),
    (true, 2),
    (false, 2),
    (true, 2),
    (false, 2),
    (true, 2),
    (false, 2),
];

/// SCL: nine clocks and the one before the stop, a step each.
fn scl_ok(l: &Levels) -> bool {
    let v = l.get();
    v.len() == 19
        && v.iter()
            .enumerate()
            .all(|(i, &(high, steps))| high == (i % 2 == 1) && steps == 1)
}

fn capture_config() -> Config {
    let mut c = Config::default();
    c.mode = InputCaptureMode::BothEdges;
    c.timeout = 62_400; // 1 ms: fills the rest of the buffer after the transfer.
    c
}

fn i2c_config(hz: u32, timeout_us: u32) -> soft_i2c::Config {
    let mut c = soft_i2c::Config::default();
    c.frequency = Hertz(hz);
    c.timeout_us = timeout_us;
    c.internal_pullups = true;
    c
}

/// Hold PAn low through its direction bit, as a device would. Done only
/// while the driver leaves the pins alone (idle, or waiting on SCL).
fn hold_pa(n: u32, low: bool) {
    pac::GPIO
        .pa_dir()
        .modify(|w| if low { w.0 |= 1 << n } else { w.0 &= !(1 << n) });
}

/// Wait for SCL (PA14) to fall `falls` times, then hold it low through PA15
/// (jumpered to it) for `hold` SysTick cycles, as a device stretching the
/// clock. In RAM, as the driver's step: the edges are only 5 us apart.
/// Gives up after 1 ms.
#[inline(never)]
#[unsafe(link_section = ".highcode.soft_i2c_example")]
fn stretch_after(falls: u32, hold: u32) -> bool {
    let g = pac::GPIO;
    let cnt = || pac::SYSTICK.stk_cntl().read().cntl();
    // The driver's interrupt also writes PA_DIR.
    let pa15 = |low: bool| {
        qingke::riscv::interrupt::free(|| {
            g.pa_dir()
                .modify(|w| if low { w.0 |= 1 << 15 } else { w.0 &= !(1 << 15) })
        })
    };
    let t0 = cnt();
    let (mut n, mut last) = (0, true);
    while n < falls {
        let now = g.pa_pin().read().0 & (1 << 14) != 0;
        if last && !now {
            n += 1;
        }
        last = now;
        if cnt().wrapping_sub(t0) > 62_400 {
            return false;
        }
    }
    pa15(true);
    let t1 = cnt();
    while cnt().wrapping_sub(t1) < hold {}
    pa15(false);
    true
}

fn new_i2c(tim: Peri<'_, peripherals::TMR3>, hz: u32, timeout_us: u32) -> SoftI2c<'_, peripherals::TMR3, Async> {
    SoftI2c::new(
        tim,
        unsafe { peripherals::PA10::steal() },
        unsafe { peripherals::PA11::steal() },
        Irqs,
        i2c_config(hz, timeout_us),
    )
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut delay = Delay;
    let mut c = Checks { pass: 0, fail: 0 };
    let sysclk = hal::rcc::clocks().sysclk.0;

    let scl_pin = CapturePin::new::<0>(p.PA10.reborrow(), Pull::Up);
    let sda_pin = CapturePin::new::<0>(p.PA11.reborrow(), Pull::Up);
    let mut cap_scl = InputCapture::new(p.TMR1.reborrow(), scl_pin, Irqs, capture_config());
    let mut cap_sda = InputCapture::new(p.TMR2.reborrow(), sda_pin, Irqs, capture_config());

    // Waveform of an unacknowledged address, async, at 100 and 50 kHz.
    for hz in [100_000u32, 50_000] {
        let mut i2c = new_i2c(p.TMR3.reborrow(), hz, 25_000);
        let step = sysclk / (2 * hz);
        let mut raw_scl = [0u32; 32];
        let mut raw_sda = [0u32; 24];
        cap_scl.restart();
        cap_sda.restart();
        let t0 = Instant::now();
        let (a, b, r) = join3(cap_scl.read_dma(&mut raw_scl), cap_sda.read_dma(&mut raw_sda), async {
            let r = i2c.write(0x55, &[]).await;
            (r, t0.elapsed().as_micros())
        })
        .await;
        let (r, us) = r;
        hal::println!(
            "{} kHz: write(0x55, []) = {:?} in {} us (captures {:?} {:?})",
            hz / 1000,
            r,
            us,
            a,
            b
        );
        let scl = Levels::from(&mut cap_scl, &raw_scl, step);
        let sda = Levels::from(&mut cap_sda, &raw_sda, step);
        scl.print("SCL");
        sda.print("SDA");
        c.check("  Nack", r == Err(Error::Nack), 0);
        c.check("  SCL: 10 low steps, 9 high", scl_ok(&scl), scl.n as i64);
        let tail = sda.get();
        let ok = tail.len() >= SDA_0X55.len() && tail[tail.len() - SDA_0X55.len()..] == SDA_0X55;
        c.check("  SDA: start, 0xAA, released ACK, stop", ok, sda.n as i64);
    }

    let mut i2c = new_i2c(p.TMR3.reborrow(), 100_000, 2_000);

    // Blocking.
    let t0 = Instant::now();
    let r = i2c.blocking_write(0x55, &[0x12, 0x34]);
    let us = t0.elapsed().as_micros() as i64;
    c.check("blocking write, no device: Nack", r == Err(Error::Nack), 0);
    c.check("  in 100..160 us (22 steps)", (100..160).contains(&us), us);
    let mut buf = [0u8; 2];
    let r = i2c.blocking_write_read(0x55, &[0x00], &mut buf);
    c.check("blocking write_read, no device: Nack", r == Err(Error::Nack), 0);
    let r = i2c.blocking_read(0x55, &mut []);
    c.check("empty read: ZeroLengthTransfer", r == Err(Error::ZeroLengthTransfer), 0);
    let r = i2c.blocking_transaction(0x55, &mut []);
    c.check("no operations: Ok", r.is_ok(), 0);

    // SDA held low at the start.
    hold_pa(11, true);
    let r = i2c.blocking_write(0x55, &[]);
    hold_pa(11, false);
    c.check("SDA held low: Bus", r == Err(Error::Bus), 0);
    let r = i2c.write(0x55, &[]).await;
    c.check("  and the bus works again", r == Err(Error::Nack), 0);

    // SCL held low throughout: timeout after 2 ms, async, with the CPU left
    // to count.
    hold_pa(10, true);
    let t0 = Instant::now();
    let mut spins = 0u32;
    let (r, ()) = join(i2c.write(0x55, &[]), async {
        while t0.elapsed().as_micros() < 1_500 {
            spins += 1;
        }
    })
    .await;
    let us = t0.elapsed().as_micros() as i64;
    hold_pa(10, false);
    let t1 = Instant::now();
    let mut idle = 0u32;
    while t1.elapsed().as_micros() < 1_500 {
        idle += 1;
    }
    hal::println!(
        "  CPU left while the interrupt waits on SCL: {}% ({} vs {} spins)",
        spins as u64 * 100 / idle as u64,
        spins,
        idle
    );
    c.check("SCL held low: Timeout", r == Err(Error::Timeout), 0);
    c.check("  after 2000..2300 us", (2_000..2_300).contains(&us), us);

    // SCL held low for 500 us at the start, then released.
    hold_pa(10, true);
    let t0 = Instant::now();
    let (r, ()) = join(i2c.write(0x55, &[]), async {
        Timer::after_micros(500).await;
        hold_pa(10, false);
    })
    .await;
    let us = t0.elapsed().as_micros() as i64;
    c.check("SCL released after 500 us: then Nack", r == Err(Error::Nack), 0);
    c.check("  in 600..900 us", (600..900).contains(&us), us);
    drop(i2c);

    // SCL on PA14, stretched by PA15 (jumpered) after its third fall.
    {
        let mut i2c = SoftI2c::new(
            p.TMR3.reborrow(),
            p.PA14.reborrow(),
            unsafe { peripherals::PA11::steal() },
            Irqs,
            i2c_config(100_000, 25_000),
        );
        // PA15 is released (an input, latch low) except while it stretches.
        let mut dev = Flex::new(p.PA15.reborrow());
        dev.set_low();
        dev.set_as_input(Pull::None);
        let step = sysclk / 200_000;
        let mut raw_sda = [0u32; 24];
        cap_sda.restart();
        let mut stretched = false;
        let (b, (r, us), ()) = join3(
            cap_sda.read_dma(&mut raw_sda),
            async {
                let t0 = Instant::now();
                let r = i2c.write(0x55, &[]).await;
                (r, t0.elapsed().as_micros() as i64)
            },
            async {
                stretched = stretch_after(3, sysclk / 5_000);
            },
        )
        .await;
        let sda = Levels::from(&mut cap_sda, &raw_sda, step);
        hal::println!("stretch: {:?} in {} us (capture {:?})", r, us, b);
        sda.print("SDA");
        c.check(
            "SCL stretched 200 us mid-byte: Nack",
            stretched && r == Err(Error::Nack),
            0,
        );
        c.check("  in 300..380 us", (300..380).contains(&us), us);
        // The stretch lengthens one level; the bits stay the same.
        let tail = sda.get();
        let ok = tail.len() >= SDA_0X55.len()
            && tail[tail.len() - SDA_0X55.len()..]
                .iter()
                .zip(SDA_0X55.iter())
                .all(|(a, b)| a.0 == b.0 && (a.1 == b.1 || a.1 > 30));
        c.check("  SDA: the same bits", ok, sda.n as i64);
    }

    hal::println!("soft_i2c: {} passed, {} failed", c.pass, c.fail);
    loop {
        delay.delay_ms(1000);
    }
}
