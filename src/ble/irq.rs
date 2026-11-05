//! BLE interrupt glue.
//!
//! The library routes BB and LLE through two of the PFIC's VTF slots to its
//! own handlers (`BB_IRQLibFunction`, `LLE_IRQLibFunction`), set up in
//! `BLE_LibInit` and again after each sleep. Those handlers end in `mret`
//! and leave saving the caller-saved registers to the hardware stack
//! (`INTSYSCR.HWSTKEN`, which qingke-rt sets).
//!
//! Should a VTF slot be off, the vector table takes over, as in WCH's
//! startup file: `BLEB` calls `BB_IRQLibHandler`, and `BLEL` and the
//! machine-mode ecall go to the library's own `LLE_IRQHandler` and
//! `Ecall_M_Mode_Handler` (its `ble_task_scheduler.S`), which defer
//! controller work to thread level and back. Those two must be reached
//! with `sp` and every register as they were, so the trampolines below only
//! count (on the stack, popped again) and jump.
//!
//! `RTC`: the library pends it (`SetPendingIRQ`) when TMOS has work.

use core::sync::atomic::AtomicU32;

use crate::interrupt::{Interrupt, InterruptExt, Priority};
use crate::pac;

/// Address of `LLE_IRQLibHandler`, read by the library's `LLE_IRQHandler`
/// (and checked by `BLE_LibInit`).
#[unsafe(no_mangle)]
static mut g_LLE_IRQLibHandlerLocation: u32 = 0;

core::arch::global_asm!(
    r#"
    .section .highcode.ble_vectors,"ax",@progbits
    .global BLEL
    .global MachineEnvCall
    .align 2
BLEL:
    addi sp, sp, -8
    sw t0, 0(sp)
    sw t1, 4(sp)
    la t0, {lle}
    lw t1, 0(t0)
    addi t1, t1, 1
    sw t1, 0(t0)
    lw t0, 0(sp)
    lw t1, 4(sp)
    addi sp, sp, 8
    j LLE_IRQHandler
    .align 2
MachineEnvCall:
    addi sp, sp, -8
    sw t0, 0(sp)
    sw t1, 4(sp)
    la t0, {deferred}
    lw t1, 0(t0)
    addi t1, t1, 1
    sw t1, 0(t0)
    lw t0, 0(sp)
    lw t1, 4(sp)
    addi sp, sp, 8
    j Ecall_M_Mode_Handler
    "#,
    lle = sym LLE_IRQS,
    deferred = sym LLE_DEFERRED,
);

/// LLE interrupts through the vector table.
pub(super) static LLE_IRQS: AtomicU32 = AtomicU32::new(0);
/// Controller work deferred from the LLE interrupt and finished (ecalls).
pub(super) static LLE_DEFERRED: AtomicU32 = AtomicU32::new(0);

#[qingke_rt::interrupt]
fn BLEB() {
    unsafe { super::ffi::BB_IRQLibHandler() };
}

#[qingke_rt::interrupt]
fn RTC() {
    pac::RTC.flag_ctrl().write(|w| {
        w.set_tmr_clr(true);
        w.set_trig_clr(true);
    });
    super::count(&super::STATS.rtc_irqs);
}

/// `SetPendingIRQ` for `TMOS_TimerInit`.
#[unsafe(link_section = ".highcode.ble_pend_rtc")]
pub(super) unsafe extern "C" fn pend_rtc() {
    Interrupt::RTC.pend();
}

/// Set up before `BLE_LibInit`, as `CH58x_BLEInit`.
pub(super) unsafe fn init() {
    unsafe {
        g_LLE_IRQLibHandlerLocation = super::ffi::LLE_IRQLibHandler as *const () as u32;
        // Lowest priority, as the EVT.
        Interrupt::BLEL.set_priority(Priority::P15);
    }
}
