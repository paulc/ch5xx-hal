//! Sleep between BLE events: advertises (every 100 ms) for 20 s on the
//! HAL's low-power executor, timing embassy timers against the RTC meanwhile,
//! then turns sleep off and reports:
//!
//! - the chip slept between advertising events, and advertising went on;
//! - it was asleep (or in the halt that ends a sleep) 95% of the time;
//! - embassy time kept step with the RTC (within 100 ppm);
//! - each timer, measured on the RTC, took what it asked for (within
//!   0.5 ms).
//!
//! The log is on UART0 TX (PB7, 115200 8N1): SDI print stops at the first
//! sleep (the debugger has to re-arm it). The results also stay in
//! registers, for `wlink regs` once it is done (about 25 s after the
//! reset; wlink's halt overwrites a0-a4): a6 0x600d when done, a7 checks
//! passed, s2 checks failed, s3 sleeps, s4 LLE interrupts, s5 embassy
//! minus RTC time (us), s6 the worst timer error (us), s7 the time asleep
//! (per mille).
//!
//! hw-test: manual. Its result is on the UART.

#![no_std]
#![no_main]

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use hal::ble::adv::AdvData;
use hal::ble::ffi::*;
use hal::ble::gap::*;
use hal::ble::{self, sleep};
use hal::uart::{self, UartTx};
use panic_halt as _;

/// Advertising interval in 0.625 ms units (100 ms)
const ADVERTISING_INTERVAL: u16 = 160;
const TIMERS: usize = 12;

static BROADCASTER_CBS: gapRolesBroadcasterCBs_t = gapRolesBroadcasterCBs_t {
    pfnStateChange: Some(broadcaster_state),
    pfnScanRecv: None,
};

static mut STATE: gapRole_States_t = 0;

unsafe extern "C" fn broadcaster_state(new_state: gapRole_States_t) {
    unsafe { STATE = new_state };
}

fn broadcaster_init() -> bStatus_t {
    unsafe {
        GAPRole_BroadcasterInit()?;
        let enable: u8 = 1;
        let event_type: u8 = GAP_ADTYPE_ADV_NONCONN_IND;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 1, &raw const enable as _)?;
        GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, 1, &raw const event_type as _)?;
        let adv = AdvData::new()
            .flags(GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
            .short_name("ch584-sleep");
        let adv = adv.as_slice();
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, adv.len() as u16, adv.as_ptr() as _)?;
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, ADVERTISING_INTERVAL)?;
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, ADVERTISING_INTERVAL)?;
        GAPRole_BroadcasterStartDevice(&BROADCASTER_CBS)
    }
}

/// RTC ticks to microseconds.
fn rtc_us(ticks: u32) -> i64 {
    ticks as i64 * 1_000_000 / 32768
}

fn load(c: &core::sync::atomic::AtomicU32) -> u32 {
    c.load(core::sync::atomic::Ordering::Relaxed)
}

#[embassy_executor::main(entry = "qingke_rt::entry", executor = "ch5xx_hal::embassy::Executor")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::HSE_PLL_62_4MHZ,
        ..Default::default()
    });
    let mut tx = UartTx::new_blocking::<0>(p.UART0, p.PB7, uart::Config::default()).unwrap();
    // The UART clock stops in sleep: finish each line before the executor
    // can sleep.
    macro_rules! println {
        ($($t:tt)*) => {{
            let _ = core::fmt::Write::write_fmt(&mut tx, format_args!($($t)*));
            let _ = tx.blocking_write(b"\r\n");
            let _ = tx.blocking_flush();
        }};
    }
    println!("ble_sleep, {}", ble::lib_version());

    let mut pass = 0;
    let mut fail = 0;
    macro_rules! check {
        ($ok:expr, $($t:tt)*) => {{
            if $ok {
                pass += 1;
                println!("PASS {}", format_args!($($t)*));
            } else {
                fail += 1;
                println!("FAIL {}", format_args!($($t)*));
            }
        }};
    }

    let init = ble::init(p.RTC, ble::Config::default());
    check!(init.is_ok(), "ble::init {:?}", init);
    let adv = broadcaster_init();
    check!(adv.is_ok(), "broadcaster_init {:?}", adv);
    // Let advertising start before sleeping.
    sleep::set_enabled(false);
    Timer::after_millis(500).await;
    check!(
        unsafe { STATE } & GAPROLE_STATE_ADV_MASK == GAPROLE_ADVERTISING,
        "advertising"
    );
    println!("sleeping for about 20 s");

    let stats = &ble::STATS;
    let (sleeps0, lle0, slept0) = (load(&stats.sleeps), stats.lle_irqs(), load(&stats.slept));
    let (t0, c0) = (Instant::now(), ble::clock());
    sleep::set_enabled(true);

    // Timers of different lengths, each measured on the RTC.
    let mut asked = [0u32; TIMERS];
    let mut took = [0i64; TIMERS];
    for i in 0..TIMERS {
        asked[i] = 1000 + 97 * i as u32;
        let c = ble::clock();
        Timer::after(Duration::from_millis(asked[i] as u64)).await;
        took[i] = rtc_us(ble::clock().wrapping_sub(c));
    }
    let elapsed = t0.elapsed().as_micros() as i64;
    let rtc = rtc_us(ble::clock().wrapping_sub(c0));
    let (sleeps, lle) = (load(&stats.sleeps) - sleeps0, stats.lle_irqs() - lle0);
    // Per mille of the time asleep (or in the halt that ends a sleep).
    let asleep = rtc_us(load(&stats.slept) - slept0) * 1000 / rtc;

    sleep::set_enabled(false);
    println!("awake: {}", stats);

    let secs = rtc / 1_000_000;
    // 10 advertising events a second, of which most give a sleep (one
    // that a timer is due in the middle of does not); each has a few LLE
    // interrupts.
    check!(sleeps as i64 >= 5 * secs, "{} sleeps in {} s", sleeps, secs);
    check!(lle as i64 >= 20 * secs, "{} LLE interrupts in {} s", lle, secs);
    // On a CH584F: 988 (about 1.2 ms awake per advertising event).
    check!(asleep >= 950, "asleep {} per mille of the time", asleep);
    check!(
        unsafe { STATE } & GAPROLE_STATE_ADV_MASK == GAPROLE_ADVERTISING,
        "still advertising"
    );
    let drift = elapsed - rtc;
    check!(
        drift.abs() <= rtc / 10_000,
        "embassy {} us, rtc {} us: {} us apart",
        elapsed,
        rtc,
        drift
    );
    let mut worst = 0i64;
    for i in 0..TIMERS {
        let err = took[i] - asked[i] as i64 * 1000;
        if err.abs() > worst.abs() {
            worst = err;
        }
        check!(
            (-500..=500).contains(&err),
            "timer {} ms took {} us on the RTC ({:+} us)",
            asked[i],
            took[i],
            err
        );
    }
    println!("{} passed, {} failed", pass, fail);
    // For `wlink regs`: interrupts off (BLE stops), spin.
    unsafe {
        core::arch::asm!(
            "csrci mstatus, 8",
            "1: j 1b",
            in("a6") 0x600d,
            in("a7") pass,
            in("s2") fail,
            in("s3") sleeps,
            in("s4") lle,
            in("s5") drift as i32,
            in("s6") worst as i32,
            in("s7") asleep as i32,
            options(noreturn),
        )
    }
}
