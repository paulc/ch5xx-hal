//! Timer input capture test, needing no wiring. Results are reported over SDI
//! print.
//!
//! - TMR0 captures its own pin, PA9, while UART1 transmits on it: a stream of
//!   'U' (0x55) is a square wave of one bit per level, 8 x `DL` HCLK cycles,
//!   so each measurement has an exact expected value. This covers the three
//!   modes, measurements longer than the timeout and FIFO overrun. Polling
//!   runs at 125000 baud, a level every 480 HCLK cycles.
//! - TMR3 (PB22, pulled up on the WeAct CH584F board, but the output
//!   overrides that), TMR1 (PB10) and TMR2 (PB11) capture their pin while the
//!   test drives it as a GPIO output, which checks their pin routing and the
//!   timeout of an input held low. PB10/PB11 are the USB pins; the pulses are
//!   far shorter than a USB host's attach debounce.
//! - DMA (TMR1, TMR2): an async DMA keeps up with a GPIO toggle loop whose
//!   levels are far too short for polling, a blocking DMA collects a slow
//!   signal, and dropping the transfer stops the DMA.
//!
//! This HAL returns raw capture words and has no `InputCapture::restart`, so
//! the test decodes the words itself (`Decoder` below) and makes a fresh
//! capture where it would restart one.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use hal::delay::Delay;
use hal::dma::StaticBuffer;
use hal::gpio::Pull;
use hal::timer::input_capture::InputCapture;
use hal::timer::{CaptureEdge, CaptureWord, Error, PeriodTicks};
use hal::uart::{self, UartTx};
use hal::{bind_interrupts, pac, peripherals};
use hwtest::check;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    TMR1 => hal::timer::InterruptHandler<peripherals::TMR1>;
});

const N: usize = 100;

static RAW: StaticBuffer<48> = StaticBuffer::new([0; 48]);
static RAW8: StaticBuffer<8> = StaticBuffer::new([0; 8]);

/// A measurement, in ticks (HCLK cycles).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Capture {
    /// `CaptureEdge::Both`: the input was high for this long.
    High(u32),
    /// `CaptureEdge::Both`: the input was low for this long.
    Low(u32),
    /// The same-edge modes: from one rising (or falling) edge to the next.
    Period(u32),
    /// The input did not change for the timeout. The timeout's ticks are
    /// added to the next measurement.
    Timeout,
}

/// Turns capture words into whole measurements: a level or period longer
/// than the timeout arrives as timeout words and then the remainder, which
/// are added up. In `Both` mode a timeout is a word of exactly the timeout's
/// ticks; in the same-edge modes bit 25 marks it. The first measurement after
/// the timer starts began part-way, and is dropped.
struct Decoder {
    both: bool,
    timeout: u32,
    pending: u32,
    skip: bool,
}

impl Decoder {
    fn new(edge: CaptureEdge, timeout: PeriodTicks) -> Self {
        Self {
            both: matches!(edge, CaptureEdge::Both),
            timeout: timeout.get(),
            pending: 0,
            skip: true,
        }
    }

    fn push(&mut self, word: CaptureWord) -> Option<Capture> {
        let ticks = word.ticks();
        let timeout = if self.both {
            ticks == self.timeout
        } else {
            word.timeout_fragment()
        };
        let total = self.pending.saturating_add(ticks);
        if timeout {
            self.pending = total;
            return Some(Capture::Timeout);
        }
        self.pending = 0;
        if self.skip {
            self.skip = false;
            return None;
        }
        Some(if !self.both {
            Capture::Period(total)
        } else if word.high_level() {
            Capture::High(total)
        } else {
            Capture::Low(total)
        })
    }
}

/// Report a check with a value, as `name: value: PASS/FAIL`.
fn c_check(name: &str, ok: bool, value: i64) {
    check!(ok, "{}: {}", name, value);
}

fn uart_config(baudrate: u32) -> uart::Config {
    uart::Config::new(baudrate)
}

/// HCLK cycles per bit at `baudrate`, as the UART divides (`DIV` = 1).
fn bit_ticks(baudrate: u32) -> u32 {
    let fsys = hal::rcc::clocks().sysclk.0 as u64;
    let dl = (fsys * 10 / 8 / baudrate as u64 + 5) / 10;
    8 * dl as u32
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

fn timeout(ticks: u32) -> PeriodTicks {
    PeriodTicks::new_capture(ticks).unwrap()
}

/// Read up to `out.len()` results, polling, with a 20 ms limit overall.
fn collect<T: hal::timer::Instance>(
    cap: &mut InputCapture<'_, T>,
    dec: &mut Decoder,
    out: &mut [Result<Capture, Error>],
) -> usize {
    let t0 = now();
    let limit = hal::rcc::clocks().sysclk.0 / 50;
    let mut n = 0;
    while n < out.len() && cycles(t0, now()) < limit {
        match cap.try_capture() {
            Ok(Some(w)) => match dec.push(w) {
                Some(c) => out[n] = Ok(c),
                None => continue,
            },
            Ok(None) => continue,
            Err(e) => out[n] = Err(e),
        }
        n += 1;
    }
    n
}

/// Poll for the next result, up to 20 ms.
fn next<T: hal::timer::Instance>(cap: &mut InputCapture<'_, T>, dec: &mut Decoder) -> Option<Result<Capture, Error>> {
    let mut r = [Ok(Capture::Timeout)];
    (collect(cap, dec, &mut r) == 1).then_some(r[0])
}

/// Whether `got` (skipping timeouts) alternates `High`/`Low` (or is all
/// `Period`) of `want` ticks, within 2; and the worst deviation.
fn all_near(got: &[Result<Capture, Error>], both: bool, want: u32) -> (bool, i64) {
    let mut ok = true;
    let mut worst = 0i64;
    let mut last_high = None;
    let mut seen = 0;
    for r in got {
        let (high, ticks) = match r {
            Ok(Capture::High(t)) => (Some(true), *t),
            Ok(Capture::Low(t)) => (Some(false), *t),
            Ok(Capture::Period(t)) => (None, *t),
            Ok(Capture::Timeout) => continue,
            Err(_) => return (false, -1),
        };
        seen += 1;
        let dev = ticks as i64 - want as i64;
        if dev.abs() > worst.abs() {
            worst = dev;
        }
        ok &= dev.abs() <= 2;
        if both {
            ok &= high.is_some() && last_high != high;
            last_high = high;
        } else {
            ok &= high.is_none();
        }
    }
    (ok && seen > 0, worst)
}

fn dump_raw(raw: &[u32]) {
    for v in raw {
        hwtest::println!("    {:#010x}", v);
    }
}

fn dump(got: &[Result<Capture, Error>]) {
    for r in got {
        hwtest::println!("    {:?}", r);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
    hwtest::sdi_enable();
    let mut delay = Delay;
    let mut got = [Ok(Capture::Timeout); N];
    let burst = [b'U'; 8];
    hwtest::println!(
        "RST# on PB23 enabled in the user config: {}",
        pac::SYSTEM_CONTROL.glob_cfg_info().read().cfg_reset_en()
    );

    // ---- TMR0 on PA9, driven by UART1 TX.
    for (edge, baud, want_bits) in [
        (CaptureEdge::Both, 125_000u32, 1u32),
        (CaptureEdge::RisingToRising, 125_000, 2),
        (CaptureEdge::FallingToFalling, 125_000, 2),
    ] {
        let mut cap = InputCapture::new::<0>(p.TMR0.reborrow(), p.PA9.reborrow(), Pull::Up, timeout(48_000), edge);
        // The UART takes PA9 as an output after the capture input is set up.
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(baud),
        )
        .unwrap();
        let mut dec = Decoder::new(edge, timeout(48_000));
        tx.blocking_write(&burst).unwrap();
        let n = collect(&mut cap, &mut dec, &mut got);
        let want = want_bits * bit_ticks(baud);
        let (ok, worst) = all_near(&got[..n], edge == CaptureEdge::Both, want);
        hwtest::println!("{:?} at {} baud, want {} ticks, {} results", edge, baud, want, n);
        c_check("  each measurement within 2 ticks (worst deviation)", ok, worst);
        if !ok {
            dump(&got[..n]);
        }
        // The line idles high after the burst.
        c_check(
            "  ends in timeouts",
            n > 0 && got[n - 1] == Ok(Capture::Timeout),
            n as i64,
        );
    }

    // Levels longer than the timeout: 2400 baud is 26000 ticks a bit, 5.4
    // timeouts of 4800.
    for (edge, want_bits) in [(CaptureEdge::Both, 1u32), (CaptureEdge::RisingToRising, 2)] {
        let mut cap = InputCapture::new::<0>(p.TMR0.reborrow(), p.PA9.reborrow(), Pull::Up, timeout(4_800), edge);
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(2400),
        )
        .unwrap();
        let mut dec = Decoder::new(edge, timeout(4_800));
        tx.blocking_write(&burst[..2]).unwrap();
        let n = collect(&mut cap, &mut dec, &mut got);
        let want = want_bits * bit_ticks(2400);
        let (ok, worst) = all_near(&got[..n], edge == CaptureEdge::Both, want);
        let timeouts = got[..n].iter().filter(|r| **r == Ok(Capture::Timeout)).count();
        hwtest::println!("{:?} at 2400 baud, timeout 4800, want {} ticks, {} results", edge, want, n);
        c_check("  timeouts merged into each measurement (worst deviation)", ok, worst);
        c_check("  timeouts reported", timeouts >= 5 * 3, timeouts as i64);
        if !ok {
            dump(&got[..n]);
        }
    }

    // Overrun: let 8 bytes' worth of levels pile up unread.
    {
        let edge = CaptureEdge::Both;
        let mut cap = InputCapture::new::<0>(p.TMR0.reborrow(), p.PA9.reborrow(), Pull::Up, timeout(48_000), edge);
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(125_000),
        )
        .unwrap();
        tx.blocking_write(&burst).unwrap();
        delay.delay_us(700);
        c_check("overrun reported", cap.try_capture() == Err(Error::FifoOverrun), 0);
        let mut dec = Decoder::new(edge, timeout(48_000));
        let n = collect(&mut cap, &mut dec, &mut got[..4]);
        c_check(
            "a new capture clears it; then timeouts only (the burst is over)",
            n > 0 && got[..n].iter().all(|r| *r == Ok(Capture::Timeout)),
            n as i64,
        );
    }

    // ---- TMR3 (PB22), TMR1 (PB10) and TMR2 (PB11), driven as GPIO outputs.
    let g = pac::GPIO;
    let drive = |bit: usize, high: bool| {
        if high {
            g.pb_out().modify(|w| w.0 |= 1 << bit);
        } else {
            g.pb_clr().write(|w| w.0 = 1 << bit);
        }
    };
    macro_rules! gpio_test {
        ($name:literal, $tim:ident, $pin:ident, $remap:literal, $bit:literal) => {{
            let edge = CaptureEdge::Both;
            let mut cap =
                InputCapture::new::<$remap>(p.$tim.reborrow(), p.$pin.reborrow(), Pull::None, timeout(60_000), edge);
            drive($bit, false);
            g.pb_dir().modify(|w| w.0 |= 1 << $bit);
            let mut dec = Decoder::new(edge, timeout(60_000));
            for i in 0..6 {
                delay.delay_us(50);
                drive($bit, i % 2 == 0);
            }
            drive($bit, false);
            let n = collect(&mut cap, &mut dec, &mut got[..5]);
            let ok = n == 5
                && got[..5].iter().enumerate().all(|(i, r)| match r {
                    // 50 us is 3120 ticks, plus the loop.
                    Ok(Capture::High(t)) => i % 2 == 0 && (3000..4000).contains(t),
                    Ok(Capture::Low(t)) => i % 2 == 1 && (3000..4000).contains(t),
                    _ => false,
                });
            c_check(concat!($name, ": 5 levels of 50 us"), ok, n as i64);
            if !ok {
                dump(&got[..n]);
            }
            // Held low: a timeout every 1 ms.
            let r = next(&mut cap, &mut dec);
            c_check(
                concat!($name, ": timeout with the input held low"),
                r == Some(Ok(Capture::Timeout)),
                0,
            );
            if r != Some(Ok(Capture::Timeout)) {
                hwtest::println!("    {:?}", r);
            }
            g.pb_dir().modify(|w| w.0 &= !(1 << $bit));
        }};
    }
    gpio_test!("TMR3 PB22", TMR3, PB22, 0, 22);
    gpio_test!("TMR1 PB10", TMR1, PB10, 1, 10);
    gpio_test!("TMR2 PB11", TMR2, PB11, 1, 11);

    // Rising mode, input held low: the timeout flag, not a level.
    {
        let edge = CaptureEdge::RisingToRising;
        let mut cap = InputCapture::new::<0>(p.TMR3.reborrow(), p.PB22.reborrow(), Pull::None, timeout(60_000), edge);
        drive(22, false);
        g.pb_dir().modify(|w| w.0 |= 1 << 22);
        let mut dec = Decoder::new(edge, timeout(60_000));
        let r = next(&mut cap, &mut dec);
        c_check("Rising, input held low: timeout", r == Some(Ok(Capture::Timeout)), 0);
        g.pb_dir().modify(|w| w.0 &= !(1 << 22));
    }

    // ---- DMA. A toggle loop with no delay: levels of a few cycles each.
    let toggle_fast = |bit: usize, n: u32| {
        for _ in 0..n {
            g.pb_out().modify(|w| w.0 |= 1 << bit);
            g.pb_clr().write(|w| w.0 = 1 << bit);
        }
    };
    {
        let edge = CaptureEdge::Both;
        let mut cap = InputCapture::new::<1>(p.TMR1.reborrow(), p.PB10.reborrow(), Pull::None, timeout(60_000), edge);
        drive(10, false);
        g.pb_dir().modify(|w| w.0 |= 1 << 10);
        // 40 edges, then 1 ms timeouts until the buffer is full.
        let t0 = now();
        let mut t = cap.read(RAW.take().unwrap()).ok().unwrap();
        let (r, ()) = join(t.wait(Irqs), async { toggle_fast(10, 20) }).await;
        let dt = cycles(t0, now());
        let raw = t.stop();
        let mut dec = Decoder::new(edge, timeout(60_000));
        let mut levels = 0u32;
        let mut sum = 0u32;
        let mut alternating = true;
        let mut last_high = None;
        let mut timeouts = 0;
        for m in raw.iter().filter_map(|&w| dec.push(CaptureWord(w))) {
            match m {
                Capture::High(t) | Capture::Low(t) => {
                    let high = matches!(m, Capture::High(_));
                    alternating &= last_high != Some(high);
                    last_high = Some(high);
                    levels += 1;
                    sum += t;
                }
                Capture::Timeout => timeouts += 1,
                Capture::Period(_) => alternating = false,
            }
        }
        hwtest::println!(
            "DMA TMR1: {} levels totalling {} ticks, {} timeouts, {} ticks overall",
            levels,
            sum,
            timeouts,
            dt
        );
        c_check(
            "DMA keeps up with 40 fast edges: 39 alternating levels",
            r.is_ok() && levels == 39 && alternating,
            levels as i64,
        );
        c_check("  then timeouts fill the buffer", timeouts == 48 - 40, timeouts as i64);
        c_check(
            "  shortest level a few cycles (average ticks)",
            sum / 39 < 100,
            (sum / 39) as i64,
        );
        if levels != 39 {
            dump_raw(raw);
        }

        // The same signal, polled: overruns.
        drop(cap);
        let mut cap = InputCapture::new::<1>(p.TMR1.reborrow(), p.PB10.reborrow(), Pull::None, timeout(60_000), edge);
        toggle_fast(10, 20);
        let mut dec = Decoder::new(edge, timeout(60_000));
        let n = collect(&mut cap, &mut dec, &mut got[..20]);
        c_check(
            "polling the same signal overruns",
            got[..n].iter().any(|r| *r == Err(Error::FifoOverrun)),
            n as i64,
        );

        // Dropping the transfer stops the DMA (no edges: it would take 48 ms).
        let mut t = cap.read(raw).ok().unwrap();
        let r = select(t.wait(Irqs), Timer::after_millis(2)).await;
        drop(t);
        let stopped = !pac::TMR1.ctrl_dma().read().dma_enable();
        c_check(
            "dropped transfer stops the DMA",
            matches!(r, Either::Second(())) && stopped,
            0,
        );
        g.pb_dir().modify(|w| w.0 &= !(1 << 10));
    }
    {
        // Blocking DMA on TMR2: 5 levels of 50 us, then timeouts.
        let edge = CaptureEdge::Both;
        let mut cap = InputCapture::new::<1>(p.TMR2.reborrow(), p.PB11.reborrow(), Pull::None, timeout(60_000), edge);
        drive(11, false);
        g.pb_dir().modify(|w| w.0 |= 1 << 11);
        let t = cap.read(RAW8.take().unwrap()).ok().unwrap();
        for i in 0..6 {
            delay.delay_us(50);
            drive(11, i % 2 == 0);
        }
        drive(11, false);
        let r = t.blocking_wait();
        let ok0 = r.is_ok();
        let raw = match r {
            Ok(b) => b,
            Err((_, b)) => b,
        };
        let mut dec = Decoder::new(edge, timeout(60_000));
        let mut n = 0;
        let mut ok = ok0;
        for (i, m) in raw.iter().filter_map(|&w| dec.push(CaptureWord(w))).enumerate() {
            n += 1;
            ok &= match (i, m) {
                (0 | 2 | 4, Capture::High(t)) | (1 | 3, Capture::Low(t)) => (3000..4000).contains(&t),
                (5 | 6, Capture::Timeout) => true,
                _ => false,
            };
        }
        c_check("blocking DMA on TMR2: 5 levels, then timeouts", ok && n == 7, n);
        if !ok {
            dump_raw(raw);
        }
        g.pb_dir().modify(|w| w.0 &= !(1 << 11));
    }

    hwtest::done("capture")
}
