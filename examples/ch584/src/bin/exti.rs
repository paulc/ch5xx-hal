//! GPIO interrupts with `ExtiInput`, self-stimulated: no wiring needed.
//!
//! PA15, PB13 and PB8 are flipped between the internal pull-up and pull-down
//! from a second task, which makes real edges on the pads. Every `wait_for_*`
//! variant is checked on each. Results are reported over SDI print.
//!
//! PB8 shares interrupt bit 8 with PB22, which is not tested: on the WeAct
//! CH584F it is pulled up too strongly for the internal pull-down (it reads
//! high throughout), and its partner PB23 is the reset pin. PA15 may be
//! jumpered to PA14 (the loopback tests' jumper): PA14 is left a floating
//! input, so the pulls still move it. If a pin is wired to something on your
//! board, take it out of `pins`.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::exti::ExtiInput;
use hal::gpio::{AnyPin, Pull};
use hal::{pac, Peri};
use panic_halt as _;

/// Flip a pin between pull-up (`high`) and pull-down.
fn set_pull(port_b: bool, n: usize, high: bool) {
    let g = pac::GPIO;
    if port_b {
        g.pb_pu().modify(|w| w.set_pb_pu(n, high));
        g.pb_pd_drv().modify(|w| w.set_pb_pd_drv(n, !high));
    } else {
        g.pa_pu().modify(|w| w.set_pa_pu(n, high));
        g.pa_pd_drv().modify(|w| w.set_pa_pd_drv(n, !high));
    }
}

/// Run `wait`, flipping the pull to `to_high` 2 ms in. Returns whether `wait`
/// finished (within 50 ms).
async fn edge_test<F: core::future::Future>(pin: (bool, usize), from_high: bool, to_high: bool, wait: F) -> bool {
    set_pull(pin.0, pin.1, from_high);
    Timer::after_millis(2).await;
    let stim = async {
        Timer::after_millis(2).await;
        set_pull(pin.0, pin.1, to_high);
        Timer::after_millis(50).await;
    };
    matches!(select(wait, stim).await, Either::First(_))
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::HSE_PLL_62_4MHZ,
        ..Default::default()
    });
    SDIPrint::enable();
    let mut delay = Delay;

    let mut pass = 0u32;
    let mut fail = 0u32;
    let mut check = |name: &str, ok: bool| {
        hal::try_println!("{}: {}", name, if ok { "PASS" } else { "FAIL" });
        if ok {
            pass += 1
        } else {
            fail += 1
        }
    };

    let pins: [(&str, (bool, usize), Peri<'static, AnyPin>); 3] = [
        ("PA15", (false, 15), p.PA15.into()),
        ("PB13", (true, 13), p.PB13.into()),
        ("PB8", (true, 8), p.PB8.into()),
    ];

    for (name, id, pin) in pins {
        let mut pin = ExtiInput::new(pin, Pull::Up);

        // Pulls actually move the pad?
        set_pull(id.0, id.1, false);
        Timer::after_millis(2).await;
        let low = pin.is_low();
        set_pull(id.0, id.1, true);
        Timer::after_millis(2).await;
        let high = pin.is_high();
        check(name, low && high);

        let r = edge_test(id, true, false, pin.wait_for_falling_edge()).await;
        hal::try_println!("  falling");
        check(name, r);
        let r = edge_test(id, false, true, pin.wait_for_rising_edge()).await;
        hal::try_println!("  rising");
        check(name, r);
        let r = edge_test(id, true, false, pin.wait_for_any_edge()).await;
        let r2 = edge_test(id, false, true, pin.wait_for_any_edge()).await;
        hal::try_println!("  any edge: falling {} rising {}", r, r2);
        check(name, r && r2);
        let r = edge_test(id, false, true, pin.wait_for_high()).await;
        hal::try_println!("  high");
        check(name, r);
        let r = edge_test(id, true, false, pin.wait_for_low()).await;
        hal::try_println!("  low");
        check(name, r);
        // No edge: must time out, not fire spuriously.
        let r = edge_test(id, true, true, pin.wait_for_falling_edge()).await;
        hal::try_println!("  no spurious edge");
        check(name, !r);
    }

    loop {
        hal::try_println!("exti done: {} pass, {} fail", pass, fail);
        delay.delay_ms(1000);
    }
}
