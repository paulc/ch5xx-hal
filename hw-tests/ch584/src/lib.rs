//! Helpers for the CH584 hardware tests (see `hw-tests/README.md`).

#![no_std]

/// `DATA0` of the QingKe V3C debug module (`hartinfo.dataaddr` = 0x340).
pub const SDI_DATA0: usize = 0xE000_0340;

include!("../../common/hwtest.rs");

use ch5xx_hal::pac;

/// Run `f` in safe-access mode, which "RWA" registers need for writes.
///
/// CH584's window is about 16 HCLK cycles (datasheet 4.2), so `SAFE_AUTO_EN`
/// is cleared to keep it open until the signature is cleared, as WCH's
/// `SetSysClock` does; `fence.i` around the signature, as WCH's `SAFEOPERATE`.
pub fn safe_access<R>(f: impl FnOnce() -> R) -> R {
    critical_section::with(|_| unsafe {
        let sc = pac::SYSTEM_CONTROL;
        pac::SYS.safe_mode_ctrl().modify(|w| w.set_safe_auto_en(false));
        core::arch::asm!("fence.i");
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0x57));
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0xa8));
        core::arch::asm!("fence.i");
        let r = f();
        pac::SYS.safe_mode_ctrl().modify(|w| w.set_safe_auto_en(true));
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0));
        core::arch::asm!("fence.i");
        r
    })
}
