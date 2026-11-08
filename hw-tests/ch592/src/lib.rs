//! Helpers for the CH592 hardware tests (see `hw-tests/README.md`).

#![no_std]

/// `DATA0` of the QingKe V4C debug module (`hartinfo.dataaddr` = 0x380).
pub const SDI_DATA0: usize = 0xE000_0380;

include!("../../common/hwtest.rs");

use ch5xx_hal::pac;

/// Run `f` in safe-access mode, which "RWA" registers need for writes.
///
/// CH592's window is about 112 HCLK cycles (datasheet 4.2) and it has no
/// `SAFE_AUTO_EN`, so `f` must finish its writes inside it; `fence.i` around
/// the signature, as WCH's `SAFEOPERATE`.
pub fn safe_access<R>(f: impl FnOnce() -> R) -> R {
    critical_section::with(|_| unsafe {
        let sc = pac::SYSTEM_CONTROL;
        core::arch::asm!("fence.i");
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0x57));
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0xa8));
        core::arch::asm!("fence.i");
        let r = f();
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0));
        core::arch::asm!("fence.i");
        r
    })
}
