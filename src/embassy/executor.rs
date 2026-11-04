//! The low-power executor, [`Executor`].

use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::{raw, Spawner};

/// Set by the pender: a task was woken since the executor last polled.
static PENDED: AtomicBool = AtomicBool::new(false);

#[unsafe(export_name = "__pender")]
fn __pender(_context: *mut ()) {
    PENDED.store(true, Ordering::SeqCst);
}

/// A thread-mode embassy executor that sleeps when idle.
///
/// Like embassy's `platform-riscv32` executor it waits in `wfi` when no task
/// is ready.
///
/// Use it in place of a `platform-*` executor, which this one replaces (both
/// define `__pender`):
///
/// ```ignore
/// #[embassy_executor::main(entry = "qingke_rt::entry", executor = "ch5xx_hal::embassy::Executor")]
/// async fn main(spawner: Spawner) -> ! { ... }
/// ```
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    /// Create a new Executor.
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(core::ptr::null_mut()),
            not_send: PhantomData,
        }
    }

    /// Run the executor: `init` spawns the first task(s), then the executor
    /// polls them forever. Needs a `&'static mut self`, which
    /// `#[embassy_executor::main]` provides.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            // A wake from here on, by a task or an interrupt, keeps the wait
            // below from starting and brings the loop round again.
            PENDED.store(false, Ordering::SeqCst);
            unsafe { self.inner.poll() };

            critical_section::with(|_| {
                // A pending interrupt ends the wait even with interrupts
                // masked (`SEVONPEND`, set by `crate::init`), and is taken
                // when the critical section ends.
                if !PENDED.load(Ordering::SeqCst) {
                    unsafe { core::arch::asm!("wfi") };
                }
            });
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
