//! Supply current in halt and sleep, for a meter: every spare pin gets a
//! pull-up (floating inputs draw current), then the chip halts for 10 s and
//! sleeps for 10 s, woken by the RTC, and starts again. Each step is logged
//! on UART0 TX (PB7, 115200 8N1) as it starts.
//!
//! Only PB7 (the log) is left alone. PB14/PB15 belong to the debug interface
//! while it is on; the test switches it off (`SAFE_DEBUG_CTRL.DEBUG_DIS`,
//! which the HAL has no call for) for each low-power step and back on after
//! it, so their pull-ups count only then and a debugger can still connect in
//! the 2 s pause. PA10/PA11 are the unused 32 kHz crystal's (the RTC runs
//! from the LSI).
//!
//! Power the WeAct CH584F board from its 5 V pin: fed from the WCH-LinkE's
//! 3.3 V instead, its regulator (ME6231) draws about 0.55 mA backwards. With
//! the original HAL it read 194 uA in halt and 9 uA in sleep (flash off),
//! as WCH's PM example does. This HAL keeps the flash powered in sleep and
//! the clock tree at full speed in halt, so expect more. There is no
//! shutdown call, so the original's shutdown step is left out.
//!
//! hw-test: manual. It enters the low-power modes.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Input, Pull};
use hal::power::{self, Retention, WakeDelay, WakeSources};
use hal::rtc::Rtc;
use hal::uart::{self, UartTx};

fn set_debug_interface_enabled(enabled: bool) {
    hwtest::safe_access(|| hal::pac::SYS.safe_debug_ctrl().write(|w| w.set_debug_dis(!enabled)));
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
    let mut tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::new(115_200)).unwrap();
    hwtest::select_lsi();
    let mut rtc = Rtc::new(p.RTC);

    // Input drivers have no drop, so the pulls stay.
    macro_rules! pull_up {
        ($($pin:ident),*) => { $( let _ = Input::new(p.$pin, Pull::Up); )* };
    }
    pull_up!(PA4, PA5, PA9, PA10, PA11, PA12, PA13, PA14, PA15);
    pull_up!(PB4, PB5, PB6, PB8, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB22, PB23);

    macro_rules! log {
        ($($t:tt)*) => {{
            let _ = core::fmt::Write::write_fmt(&mut tx, format_args!($($t)*));
            let _ = tx.blocking_write(b"\r\n");
            let _ = tx.blocking_flush();
        }};
    }

    let ram = Retention::RAM32K | Retention::RAM96K;
    loop {
        // Time for a serial monitor or debugger to attach.
        Timer::after_secs(2).await;
        log!("halt 10 s");
        rtc.clear_trigger();
        rtc.set_alarm_in(10 * 32_768);
        set_debug_interface_enabled(false);
        power::halt(WakeSources::RTC, WakeDelay::Long);
        set_debug_interface_enabled(true);
        rtc.clear_trigger();

        log!("sleep 10 s");
        rtc.set_alarm_in(10 * 32_768);
        set_debug_interface_enabled(false);
        power::sleep(ram, WakeSources::RTC, WakeDelay::Long);
        set_debug_interface_enabled(true);
        rtc.clear_trigger();
    }
}
