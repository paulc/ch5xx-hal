//! Wakes from a task racing wakes from an interrupt. Results are reported over
//! SDI print; no wiring needed.
//!
//! Two tasks wake each other as fast as they can (as a channel between tasks
//! does), and a TMR0 interrupt wakes a third task. Each wake queues the task
//! in the executor's run queue. On CH592 built for `riscv32imac` that is an
//! LR/SC loop that an interrupt can undo, losing a task. The QingKe V3C
//! core has no A extension, so here the queue always takes a critical
//! section: this checks that no task is lost and every phase passes, with
//! either executor platform.

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::sync::atomic::Ordering::SeqCst;
use core::sync::atomic::{AtomicBool, AtomicU32};
use core::task::{Poll, Waker};

use ch5xx_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::interrupt::typelevel::Interrupt;
use hwtest::check;

/// TMR0 periods (sysclk cycles at 62.4 MHz): 10 us, 100 us, 1 ms.
const PERIODS: [u32; 3] = [624, 6_240, 62_400];
const SECS: u32 = 10;

struct Slot(UnsafeCell<Option<Waker>>);
unsafe impl Sync for Slot {}
impl Slot {
    const fn new() -> Self {
        Self(UnsafeCell::new(None))
    }
    fn put(&self, w: &Waker) {
        critical_section(|| unsafe { *self.0.get() = Some(w.clone()) });
    }
    fn wake(&self) {
        if let Some(w) = critical_section(|| unsafe { (*self.0.get()).take() }) {
            w.wake();
        }
    }
}

fn critical_section<R>(f: impl FnOnce() -> R) -> R {
    qingke::riscv::interrupt::free(f)
}

/// An increment that needs no atomic read-modify-write (none on riscv32imc).
fn inc(a: &AtomicU32) {
    critical_section(|| a.store(a.load(SeqCst) + 1, SeqCst));
}

static TURN: [AtomicBool; 2] = [AtomicBool::new(true), AtomicBool::new(false)];
static PONG: [Slot; 2] = [Slot::new(), Slot::new()];
static ROUNDS: AtomicU32 = AtomicU32::new(0);
static TICKS: AtomicU32 = AtomicU32::new(0);
static PHASE: AtomicU32 = AtomicU32::new(0);
static TICK: [Slot; PERIODS.len()] = [const { Slot::new() }; PERIODS.len()];
static SEEN: [AtomicU32; PERIODS.len()] = [const { AtomicU32::new(0) }; PERIODS.len()];

#[hal::interrupt]
fn TMR0() {
    hal::pac::TMR0.int_flag().write(|w| w.0 = 0xff);
    inc(&TICKS);
    TICK[PHASE.load(SeqCst) as usize].wake();
}

#[embassy_executor::task(pool_size = 2)]
async fn ping_pong(me: usize) {
    loop {
        poll_fn(|cx| {
            PONG[me].put(cx.waker());
            let mine = critical_section(|| {
                let mine = TURN[me].load(SeqCst);
                TURN[me].store(false, SeqCst);
                mine
            });
            if mine {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        if me == 0 {
            inc(&ROUNDS);
        }
        TURN[1 - me].store(true, SeqCst);
        PONG[1 - me].wake(); // a wake from a task
    }
}

/// Counts the TMR0 wakes of one phase.
#[embassy_executor::task(pool_size = 3)]
async fn ticker(phase: usize) {
    loop {
        let last = TICKS.load(SeqCst);
        poll_fn(|cx| {
            TICK[phase].put(cx.waker());
            if TICKS.load(SeqCst) != last {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        inc(&SEEN[phase]);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let _p = hal::init(hal::rcc::Config::HSE_PLL_62_4MHZ.into());
    hwtest::sdi_enable();
    spawner.spawn(ping_pong(0).unwrap());
    spawner.spawn(ping_pong(1).unwrap());

    let t = hal::pac::TMR0;
    for (phase, &period) in PERIODS.iter().enumerate() {
        t.ctrl_mod().write(|w| w.set_all_clear(true));
        t.ctrl_mod().write(|w| w.0 = 0);
        t.cnt_end().write(|w| w.set_cnt_end(period));
        t.int_flag().write(|w| w.0 = 0xff);
        t.inter_en().write(|w| w.set_ie_cyc_end(true));
        PHASE.store(phase as u32, SeqCst);
        spawner.spawn(ticker(phase).unwrap());
        unsafe { hal::interrupt::typelevel::TMR0::enable() };
        let (t0, r0) = (TICKS.load(SeqCst), ROUNDS.load(SeqCst));
        t.ctrl_mod().write(|w| w.set_count_en(true));
        Timer::after_secs(SECS as u64 - 1).await;
        let before = SEEN[phase].load(SeqCst);
        Timer::after_secs(1).await;
        let (ticks, rounds) = (TICKS.load(SeqCst) - t0, ROUNDS.load(SeqCst) - r0);
        let seen = SEEN[phase].load(SeqCst);
        // Ticks closer together than a poll share one wake; a lost task stops
        // counting for good.
        hwtest::println!(
            "{} us interrupt for {} s: {} ticks, ticker woken {} times, {} ping-pong rounds",
            period as u64 * 1_000_000 / hal::rcc::clocks().sysclk.0 as u64,
            SECS,
            ticks,
            seen,
            rounds
        );
        check!(
            seen > before,
            "  ticker still woken in the last second ({})",
            seen - before
        );
        check!(rounds > 1000, "  ping-pong still running ({})", rounds);
    }
    hal::interrupt::typelevel::TMR0::disable();
    t.ctrl_mod().write(|w| w.set_all_clear(true));
    hwtest::done("wake_race")
}
