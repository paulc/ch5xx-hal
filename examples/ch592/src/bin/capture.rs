//! Timer input capture test, needing no wiring. Results are reported over SDI
//! print.
//!
//! - TMR0 captures its own pin, PA9, while UART1 transmits on it: a stream of
//!   'U' (0x55) is a square wave of one bit per level, 8 x `DL` HCLK cycles,
//!   so each measurement has an exact expected value. This covers the three
//!   modes, blocking and async reads, timeouts, measurements longer than the
//!   timeout and FIFO overrun. 125000 baud is a level every 480 HCLK cycles;
//!   at 250000 (240 cycles) reads from flash no longer keep up.
//! - TMR3 (PB22, the KEY button: don't press it while this runs), TMR1 (PB10)
//!   and TMR2 (PB11) capture their pin while the test drives it as a GPIO
//!   output, which checks their pin routing and the timeout of an input held
//!   low. PB10/PB11 are the USB pins; the pulses are far shorter than a USB
//!   host's attach debounce.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::gpio::Pull;
use hal::mode::Mode;
use hal::timer::input_capture::{Capture, CapturePin, Config, Error, InputCapture, InputCaptureMode};
use hal::uart::{self, UartTx};
use hal::{bind_interrupts, pac, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    TMR0 => hal::timer::input_capture::InterruptHandler<peripherals::TMR0>;
});

const N: usize = 100;

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

fn config(mode: InputCaptureMode, timeout: u32) -> Config {
    let mut c = Config::default();
    c.mode = mode;
    c.timeout = timeout;
    c
}

fn uart_config(baudrate: u32) -> uart::Config {
    let mut c = uart::Config::default();
    c.baudrate = baudrate;
    c
}

/// HCLK cycles per bit at `baudrate`, as the UART divides (`DIV` = 1).
fn bit_ticks(baudrate: u32) -> u32 {
    let fsys = hal::rcc::clocks().sysclk.0 as u64;
    let dl = (fsys * 10 / 8 / baudrate as u64 + 5) / 10;
    8 * dl as u32
}

/// SysTick, which counts HCLK.
fn now() -> u32 {
    pac::SYSTICK.stk_cntl().read().cntl()
}

/// Read up to `out.len()` results, polling, with a 20 ms limit overall.
fn collect<T: hal::timer::Instance, M: Mode>(
    cap: &mut InputCapture<'_, T, M>,
    out: &mut [Result<Capture, Error>],
) -> usize {
    let t0 = now();
    let limit = hal::rcc::clocks().sysclk.0 / 50;
    let mut n = 0;
    while n < out.len() && now().wrapping_sub(t0) < limit {
        match cap.nb_read() {
            Ok(c) => out[n] = Ok(c),
            Err(nb::Error::Other(e)) => out[n] = Err(e),
            Err(nb::Error::WouldBlock) => continue,
        }
        n += 1;
    }
    n
}

/// Whether `got` (skipping timeouts) alternates `High`/`Low` (or is all
/// `Period`) of `want` ticks, within 2; and the worst deviation.
fn all_near(got: &[Result<Capture, Error>], mode: InputCaptureMode, want: u32) -> (bool, i64) {
    let mut ok = true;
    let mut worst = 0i64;
    let mut last_high = None;
    let mut seen = 0;
    for r in got {
        let c = match r {
            Ok(c) => *c,
            Err(Error::Timeout) => continue,
            Err(_) => return (false, -1),
        };
        seen += 1;
        let dev = c.ticks() as i64 - want as i64;
        if dev.abs() > worst.abs() {
            worst = dev;
        }
        ok &= dev.abs() <= 2;
        match (mode, c) {
            (InputCaptureMode::BothEdges, Capture::High(_)) => {
                ok &= last_high != Some(true);
                last_high = Some(true);
            }
            (InputCaptureMode::BothEdges, Capture::Low(_)) => {
                ok &= last_high != Some(false);
                last_high = Some(false);
            }
            (InputCaptureMode::BothEdges, _) => ok = false,
            (_, Capture::Period(_)) => {}
            _ => ok = false,
        }
    }
    (ok && seen > 0, worst)
}

fn dump(got: &[Result<Capture, Error>]) {
    for r in got {
        hal::println!("    {:?}", r);
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
    let mut got = [Ok(Capture::Period(0)); N];
    let burst = [b'U'; 8];
    hal::println!(
        "RST# on PB23 enabled in the user config: {}",
        pac::SYSTEM_CONTROL.glob_cfg_info().read().cfg_reset_en()
    );

    // ---- TMR0 on PA9, driven by UART1 TX.
    for (mode, baud, want_bits) in [
        (InputCaptureMode::BothEdges, 125_000u32, 1u32),
        (InputCaptureMode::Rising, 125_000, 2),
        (InputCaptureMode::Falling, 125_000, 2),
    ] {
        let pin = CapturePin::new::<0>(p.PA9.reborrow(), Pull::Up);
        let mut cap = InputCapture::new_blocking(p.TMR0.reborrow(), pin, config(mode, 48_000));
        // The UART takes PA9 as an output after the capture input is set up.
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(baud),
        )
        .unwrap();
        cap.restart();
        tx.blocking_write(&burst).unwrap();
        let n = collect(&mut cap, &mut got);
        let want = want_bits * bit_ticks(baud);
        let (ok, worst) = all_near(&got[..n], mode, want);
        hal::println!("{:?} at {} baud, want {} ticks, {} results", mode, baud, want, n);
        c.check("  each measurement within 2 ticks (worst deviation)", ok, worst);
        if !ok {
            dump(&got[..n]);
        }
        // The line idles high after the burst.
        c.check(
            "  ends in timeouts",
            matches!(got[n - 1], Err(Error::Timeout)),
            n as i64,
        );
    }

    // Levels longer than the timeout: 2400 baud is 25000 ticks a bit, 5.2
    // timeouts of 4800.
    for (mode, want_bits) in [(InputCaptureMode::BothEdges, 1u32), (InputCaptureMode::Rising, 2)] {
        let pin = CapturePin::new::<0>(p.PA9.reborrow(), Pull::Up);
        let mut cap = InputCapture::new_blocking(p.TMR0.reborrow(), pin, config(mode, 4_800));
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(2400),
        )
        .unwrap();
        cap.restart();
        tx.blocking_write(&burst[..2]).unwrap();
        let n = collect(&mut cap, &mut got);
        let want = want_bits * bit_ticks(2400);
        let (ok, worst) = all_near(&got[..n], mode, want);
        let timeouts = got[..n].iter().filter(|r| matches!(r, Err(Error::Timeout))).count();
        hal::println!(
            "{:?} at 2400 baud, timeout 4800, want {} ticks, {} results",
            mode,
            want,
            n
        );
        c.check("  timeouts merged into each measurement (worst deviation)", ok, worst);
        c.check("  timeouts reported", timeouts >= 5 * 3, timeouts as i64);
        if !ok {
            dump(&got[..n]);
        }
    }

    // Overrun: let 8 bytes' worth of levels pile up unread.
    {
        let pin = CapturePin::new::<0>(p.PA9.reborrow(), Pull::Up);
        let mut cap = InputCapture::new_blocking(p.TMR0.reborrow(), pin, config(InputCaptureMode::BothEdges, 48_000));
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(125_000),
        )
        .unwrap();
        cap.restart();
        tx.blocking_write(&burst).unwrap();
        delay.delay_us(700);
        let first = cap.blocking_read();
        c.check("overrun reported", first == Err(Error::Overrun), 0);
        let n = collect(&mut cap, &mut got);
        c.check(
            "then timeouts only (the burst is over)",
            n > 0 && got[..n].iter().all(|r| *r == Err(Error::Timeout)),
            n as i64,
        );
    }

    // Async read.
    {
        let pin = CapturePin::new::<0>(p.PA9.reborrow(), Pull::Up);
        let mut cap = InputCapture::new(
            p.TMR0.reborrow(),
            pin,
            Irqs,
            config(InputCaptureMode::BothEdges, 48_000),
        );
        let mut tx = UartTx::new_blocking::<0>(
            p.UART1.reborrow(),
            unsafe { peripherals::PA9::steal() },
            uart_config(125_000),
        )
        .unwrap();
        cap.restart();
        tx.blocking_write(&burst).unwrap();
        let mut n = 0;
        while n < N {
            got[n] = cap.read().await;
            n += 1;
            if got[n - 1] == Err(Error::Timeout) {
                break;
            }
        }
        let (ok, worst) = all_near(&got[..n], InputCaptureMode::BothEdges, bit_ticks(125_000));
        hal::println!("async BothEdges at 125000 baud, {} results", n);
        c.check("  each measurement within 2 ticks (worst deviation)", ok, worst);
        c.check("  79 levels then a timeout", n == 80, n as i64);
        if !ok {
            dump(&got[..n]);
        }
        // A timeout wakes `read` with no edges at all. (Restart: timeouts
        // piled up while the results printed.)
        cap.restart();
        let t0 = now();
        let r = cap.read().await;
        let us = now().wrapping_sub(t0) as u64 * 1_000_000 / hal::rcc::clocks().sysclk.0 as u64;
        c.check(
            "  idle line: read wakes on the timeout (us)",
            r == Err(Error::Timeout) && us < 2000,
            us as i64,
        );
        if r != Err(Error::Timeout) {
            hal::println!("    {:?}", r);
        }
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
            let pin = CapturePin::new::<$remap>(p.$pin.reborrow(), Pull::None);
            let mut cap =
                InputCapture::new_blocking(p.$tim.reborrow(), pin, config(InputCaptureMode::BothEdges, 60_000));
            drive($bit, false);
            g.pb_dir().modify(|w| w.0 |= 1 << $bit);
            cap.restart();
            for i in 0..6 {
                delay.delay_us(50);
                drive($bit, i % 2 == 0);
            }
            drive($bit, false);
            let n = collect(&mut cap, &mut got[..5]);
            let ok = n == 5
                && got[..5].iter().enumerate().all(|(i, r)| match r {
                    // 50 us is 3000 ticks, plus the loop.
                    Ok(Capture::High(t)) => i % 2 == 0 && (3000..4000).contains(t),
                    Ok(Capture::Low(t)) => i % 2 == 1 && (3000..4000).contains(t),
                    _ => false,
                });
            c.check(concat!($name, ": 5 levels of 50 us"), ok, n as i64);
            if !ok {
                dump(&got[..n]);
            }
            // Held low: a timeout every 1 ms.
            let r = cap.blocking_read();
            c.check(
                concat!($name, ": timeout with the input held low"),
                r == Err(Error::Timeout),
                0,
            );
            if r != Err(Error::Timeout) {
                hal::println!("    {:?}", r);
            }
            g.pb_dir().modify(|w| w.0 &= !(1 << $bit));
        }};
    }
    gpio_test!("TMR3 PB22", TMR3, PB22, 0, 22);
    gpio_test!("TMR1 PB10", TMR1, PB10, 1, 10);
    gpio_test!("TMR2 PB11", TMR2, PB11, 1, 11);

    // Rising mode, input held low: the timeout flag, not a level.
    {
        let pin = CapturePin::new::<0>(p.PB22.reborrow(), Pull::None);
        let mut cap = InputCapture::new_blocking(p.TMR3.reborrow(), pin, config(InputCaptureMode::Rising, 60_000));
        drive(22, false);
        g.pb_dir().modify(|w| w.0 |= 1 << 22);
        cap.restart();
        let r = cap.blocking_read();
        c.check("Rising, input held low: timeout", r == Err(Error::Timeout), 0);
        g.pb_dir().modify(|w| w.0 &= !(1 << 22));
    }

    hal::println!("capture: {} passed, {} failed", c.pass, c.fail);
    loop {
        delay.delay_ms(1000);
    }
}
