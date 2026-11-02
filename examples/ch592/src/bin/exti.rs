//! GPIO interrupts with `ExtiInput`, self-stimulated: no wiring needed.
//!
//! PA15 and PB13 are left unconnected and flipped between the internal
//! pull-up and pull-down from a second task, which makes real edges on the
//! pads. Every `wait_for_*` variant is checked on both ports. Results are
//! reported over SDI print.
//!
//! If either pin is wired to something on your board, change `PA_PIN`/`PB_PIN`.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::exti::ExtiInput;
use hal::gpio::Pull;
use hal::pac;
use panic_halt as _;

const PA_PIN: usize = 15;
const PB_PIN: usize = 13;

/// Flip `port`'s pin between pull-up (`high`) and pull-down.
fn set_pull(port_b: bool, high: bool) {
    let g = pac::GPIO;
    if port_b {
        g.pb_pu().modify(|w| w.set_pb_pu(PB_PIN, high));
        g.pb_pd_drv().modify(|w| w.set_pb_pd_drv(PB_PIN, !high));
    } else {
        g.pa_pu().modify(|w| w.set_pa_pu(PA_PIN, high));
        g.pa_pd_drv().modify(|w| w.set_pa_pd_drv(PA_PIN, !high));
    }
}

/// Run `wait`, flipping the pull to `to_high` 2 ms in. Returns whether `wait`
/// finished (within 50 ms).
async fn edge_test<F: core::future::Future>(port_b: bool, from_high: bool, to_high: bool, wait: F) -> bool {
    set_pull(port_b, from_high);
    Timer::after_millis(2).await;
    let stim = async {
        Timer::after_millis(2).await;
        set_pull(port_b, to_high);
        Timer::after_millis(50).await;
    };
    matches!(select(wait, stim).await, Either::First(_))
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_60MHZ,
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

    let mut pa = ExtiInput::new(p.PA15, Pull::Up);
    let mut pb = ExtiInput::new(p.PB13, Pull::Up);

    for (name, port_b) in [("PA15", false), ("PB13", true)] {
        let pin = if port_b { &mut pb } else { &mut pa };

        // Pulls actually move the pad?
        set_pull(port_b, false);
        Timer::after_millis(2).await;
        let low = pin.is_low();
        set_pull(port_b, true);
        Timer::after_millis(2).await;
        let high = pin.is_high();
        check(name, low && high);

        let r = edge_test(port_b, true, false, pin.wait_for_falling_edge()).await;
        hal::try_println!("  falling");
        check(name, r);
        let r = edge_test(port_b, false, true, pin.wait_for_rising_edge()).await;
        hal::try_println!("  rising");
        check(name, r);
        let r = edge_test(port_b, true, false, pin.wait_for_any_edge()).await;
        let r2 = edge_test(port_b, false, true, pin.wait_for_any_edge()).await;
        hal::try_println!("  any edge: falling {} rising {}", r, r2);
        check(name, r && r2);
        let r = edge_test(port_b, false, true, pin.wait_for_high()).await;
        hal::try_println!("  high");
        check(name, r);
        let r = edge_test(port_b, true, false, pin.wait_for_low()).await;
        hal::try_println!("  low");
        check(name, r);
        // No edge: must time out, not fire spuriously.
        let r = edge_test(port_b, true, true, pin.wait_for_falling_edge()).await;
        hal::try_println!("  no spurious edge");
        check(name, !r);
    }

    loop {
        hal::try_println!("exti done: {} pass, {} fail", pass, fail);
        delay.delay_ms(1000);
    }
}
