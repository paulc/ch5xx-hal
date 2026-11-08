//! GPIO port-interrupt waits (`gpio::Input::wait_for_*`), self-stimulated: no
//! wiring needed.
//!
//! PA15 and PB13 are flipped between the internal pull-up and pull-down from
//! a second future, which makes real edges on the pads. Every `wait_for_*`
//! variant is checked on each. Results are reported over SDI print.
//!
//! PB22 (the KEY button, pulled up on the board: don't press it while this
//! runs) is driven as a push-pull output instead, which overrides the
//! pull-up; the input still sees the pad. CH592 folds the PB22/PB23
//! interrupts onto bits 8/9 (`R16_PIN_ALTERNATE.RB_PIN_INTX`), so this checks
//! the HAL routes them. PB23 is the reset pin and is not tested.
//!
//! `wait_for_any_edge` panics in this HAL (one pin stores one polarity), so it
//! is reported as skipped. PA15 may be jumpered to PA14 (the loopback tests'
//! jumper): PA14 is left a floating input, so the pulls still move it.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use hal::gpio::{AnyPin, Input, Pull};
use hal::{pac, Peri};
use hwtest::check;
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
    let p = hal::init(hal::rcc::Config::PLL_60MHZ.into());
    hwtest::sdi_enable();

    let pins: [(&str, (bool, usize), Peri<'static, AnyPin>); 2] = [
        ("PA15", (false, 15), p.PA15.into()),
        ("PB13", (true, 13), p.PB13.into()),
    ];
    for (name, id, pin) in pins {
        let mut pin = Input::new(pin, Pull::Up);

        // Pulls actually move the pad?
        set_pull(id.0, id.1, false);
        Timer::after_millis(2).await;
        let low = pin.is_low();
        set_pull(id.0, id.1, true);
        Timer::after_millis(2).await;
        let high = pin.is_high();
        check!(low && high, "{} pulls move the pad", name);

        let r = edge_test(id, true, false, pin.wait_for_falling_edge()).await;
        check!(r, "{} falling edge", name);
        let r = edge_test(id, false, true, pin.wait_for_rising_edge()).await;
        check!(r, "{} rising edge", name);
        hwtest::skip("any edge", "wait_for_any_edge panics in this HAL");
        let r = edge_test(id, false, true, pin.wait_for_high()).await;
        check!(r, "{} high level", name);
        let r = edge_test(id, true, false, pin.wait_for_low()).await;
        check!(r, "{} low level", name);
        // Already at the level: must return at once.
        set_pull(id.0, id.1, true);
        Timer::after_millis(2).await;
        let r = edge_test(id, true, true, pin.wait_for_high()).await;
        check!(r, "{} high level already met", name);
        // No edge: must time out, not fire spuriously.
        let r = edge_test(id, true, true, pin.wait_for_falling_edge()).await;
        check!(!r, "{} no spurious edge", name);
        // A cancelled wait must not leave the next one confused.
        let r = edge_test(id, true, false, pin.wait_for_falling_edge()).await;
        check!(r, "{} falling edge after a cancelled wait", name);
    }

    // PB22, driven: the output register sets the level, 2 ms in.
    let mut key = Input::new(p.PB22, Pull::Up);
    let g = pac::GPIO;
    let drive = |high: bool| {
        if high {
            g.pb_out().modify(|w| w.set_pb_out(22, true));
        } else {
            g.pb_clr().write(|w| w.set_pb_clr(22, true));
        }
    };
    drive(true);
    g.pb_dir().modify(|w| w.set_pb_dir(22, true));
    Timer::after_millis(2).await;
    let high = key.is_high();
    drive(false);
    Timer::after_millis(2).await;
    let low = key.is_low();
    check!(high && low, "PB22 driven: input follows the pad");
    for (name, from, to) in [("falling", true, false), ("rising", false, true)] {
        drive(from);
        Timer::after_millis(2).await;
        let stim = async {
            Timer::after_millis(2).await;
            drive(to);
            Timer::after_millis(50).await;
        };
        let wait = async {
            if to {
                key.wait_for_rising_edge().await
            } else {
                key.wait_for_falling_edge().await
            }
        };
        let r = matches!(select(wait, stim).await, Either::First(_));
        check!(r, "PB22 {} edge (interrupt bit 8)", name);
    }
    g.pb_dir().modify(|w| w.set_pb_dir(22, false));

    hwtest::done("exti")
}
