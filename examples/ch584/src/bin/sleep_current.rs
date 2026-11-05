//! Supply current in halt, sleep and shutdown, for a meter: every spare pin
//! gets a pull-up (floating inputs draw current), then the chip halts for
//! 10 s, sleeps for 10 s and shuts down for 10 s, woken by the RTC. The
//! shutdown wakes as a reset, which starts it all again. Each step is logged
//! on UART0 TX (PB7, 115200 8N1) as it starts.
//!
//! Only PB7 (the log) is left alone. PB14/PB15 belong to the debug interface
//! while it is on; it is switched off for each low-power step and back on
//! after it, so their pull-ups count only then and a debugger can still
//! connect in the 2 s pause after a reset. PA10/PA11 are the unused 32 kHz
//! crystal's (the RTC runs from the LSI).
//!
//! Power the WeAct CH584F board from its 5 V pin: fed from the WCH-LinkE's
//! 3.3 V instead, its regulator (ME6231) draws about 0.55 mA backwards.
//! From 5 V it reads 194 uA in halt, 9 uA in sleep and 3 uA in shutdown,
//! as WCH's PM example does.
//!
//! hw-test: manual. It enters the low-power modes.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::debug::set_debug_interface_enabled;
use hal::gpio::{Input, Pull};
use hal::pwr::{self, ShutdownConfig, SleepConfig, WakeConfig};
use hal::rtc::{Config, Rtc, TICKS_PER_SECOND};
use hal::uart::{self, UartTx};
use panic_halt as _;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::HSE_PLL_62_4MHZ,
        ..Default::default()
    });
    let mut tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::default()).unwrap();
    let mut rtc = Rtc::new(p.RTC, Config::default()).unwrap();

    // Input drivers have no drop, so the pulls stay.
    macro_rules! pull_up {
        ($($pin:ident),*) => { $( let _ = Input::new(p.$pin, Pull::Up); )* };
    }
    pull_up!(PA4, PA5, PA9, PA10, PA11, PA12, PA13, PA14, PA15);
    pull_up!(PB4, PB5, PB6, PB8, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB22, PB23);

    let mut wake = WakeConfig::default();
    wake.rtc = true;
    pwr::configure_wake(wake);

    macro_rules! log {
        ($($t:tt)*) => {{
            let _ = core::fmt::Write::write_fmt(&mut tx, format_args!($($t)*));
            let _ = tx.blocking_write(b"\r\n");
            let _ = tx.blocking_flush();
        }};
    }

    // Time for a serial monitor to attach.
    Timer::after_secs(2).await;
    log!("reset reason: {:?}", pwr::reset_reason());
    log!("halt 10 s");
    rtc.set_alarm(rtc.now() + 10 * TICKS_PER_SECOND);
    set_debug_interface_enabled(false);
    pwr::halt();
    set_debug_interface_enabled(true);
    rtc.clear_alarm();

    log!("sleep 10 s");
    rtc.set_alarm(rtc.now() + 10 * TICKS_PER_SECOND);
    set_debug_interface_enabled(false);
    pwr::sleep(SleepConfig::default());
    set_debug_interface_enabled(true);
    rtc.clear_alarm();

    log!("shutdown 10 s");
    rtc.set_alarm(rtc.now() + 10 * TICKS_PER_SECOND);
    set_debug_interface_enabled(false);
    pwr::shutdown(ShutdownConfig::default())
}
