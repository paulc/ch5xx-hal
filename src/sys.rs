//! `SYS` helpers shared by the drivers: safe-access mode, peripheral clock
//! gates and the pin-remap register.
//!
//! These stand in for ch32-hal's RCC enable bits and AFIO remap: on CH5xx both
//! live in the `SYS` block at `0x4000_1000`.

use crate::pac;

/// Run `f` in safe-access mode (SAM), which "RWA"/"WA" registers need for
/// writes.
///
/// Writing `0x57` then `0xA8` to `R8_SAFE_ACCESS_SIG` opens a window of about
/// 112 HCLK cycles on CH592; `f` must finish its writes inside it. On
/// CH584/CH585 the window is only about 16 cycles, so `SAFE_AUTO_EN` is
/// cleared to keep it open until the signature is cleared, as WCH's
/// `SetSysClock` does. Interrupts are disabled throughout, as in WCH's
/// `sys_safe_access_enable()`.
#[allow(unused)]
pub(crate) fn safe_access<R>(f: impl FnOnce() -> R) -> R {
    // `fence.i` around the signature, as WCH's `SAFEOPERATE`.
    critical_section::with(|_| unsafe {
        let sc = pac::SYSTEM_CONTROL;
        #[cfg(sys_ch585)]
        pac::SYS.safe_mode_ctrl().modify(|w| w.set_safe_auto_en(false));
        core::arch::asm!("fence.i");
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0x57));
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0xa8));
        core::arch::asm!("fence.i");
        let r = f();
        #[cfg(sys_ch585)]
        pac::SYS.safe_mode_ctrl().modify(|w| w.set_safe_auto_en(true));
        sc.safe_access_sig().write(|w| w.set_safe_access_sig(0));
        core::arch::asm!("fence.i");
        r
    })
}

/// Peripheral clock gates (`R8_SLP_CLK_OFF0/1`, 1 = clock off). All are on
/// after reset; drivers turn theirs back on in case the application gated it.
#[cfg(any(sys_ch592, sys_ch585))]
#[allow(unused)]
#[derive(Clone, Copy)]
pub(crate) enum ClockGate {
    Tmr0,
    Tmr1,
    Tmr2,
    Tmr3,
    Uart0,
    Uart1,
    Uart2,
    Uart3,
    Spi0,
    Pwmx,
    I2c,
    Usb,
    Lcd,
}

#[cfg(any(sys_ch592, sys_ch585))]
#[allow(unused)]
pub(crate) fn set_clock_enabled(gate: ClockGate, on: bool) {
    let off = !on;
    safe_access(|| match gate {
        ClockGate::Tmr0 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_tmr0(off)),
        ClockGate::Tmr1 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_tmr1(off)),
        ClockGate::Tmr2 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_tmr2(off)),
        ClockGate::Tmr3 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_tmr3(off)),
        ClockGate::Uart0 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_uart0(off)),
        ClockGate::Uart1 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_uart1(off)),
        ClockGate::Uart2 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_uart2(off)),
        ClockGate::Uart3 => pac::SYS.slp_clk_off0().modify(|w| w.set_slp_clk_uart3(off)),
        ClockGate::Spi0 => pac::SYS.slp_clk_off1().modify(|w| w.set_slp_clk_spi0(off)),
        ClockGate::Pwmx => pac::SYS.slp_clk_off1().modify(|w| w.set_slp_clk_pwmx(off)),
        ClockGate::I2c => pac::SYS.slp_clk_off1().modify(|w| w.set_slp_clk_i2c(off)),
        ClockGate::Usb => pac::SYS.slp_clk_off1().modify(|w| w.set_slp_clk_usb(off)),
        ClockGate::Lcd => pac::SYS.slp_clk_off1().modify(|w| w.set_slp_clk_lcd(off)),
    })
}

/// Pin-remap groups in `R16_PIN_ALTERNATE`: 0 selects the default pins, 1 the
/// alternate ("`_`"-suffixed) pins. Not a SAM register.
#[cfg(any(sys_ch592, sys_ch585))]
#[allow(unused)]
#[derive(Clone, Copy)]
pub(crate) enum RemapGroup {
    Tmr0,
    Tmr1,
    Tmr2,
    #[cfg(sys_ch585)]
    Tmr3,
    Uart0,
    Uart1,
    Uart2,
    #[cfg(sys_ch585)]
    Uart3,
    Spi0,
    Pwmx,
    #[cfg(sys_ch585)]
    I2c,
}

#[cfg(any(sys_ch592, sys_ch585))]
#[allow(unused)]
pub(crate) fn set_remap(group: RemapGroup, remap: u8) {
    let alt = match remap {
        0 => false,
        1 => true,
        _ => panic!("invalid remap"),
    };
    critical_section::with(|_| {
        pac::SYS.pin_alternate().modify(|w| match group {
            RemapGroup::Tmr0 => w.set_pin_tmr0(alt),
            RemapGroup::Tmr1 => w.set_pin_tmr1(alt),
            RemapGroup::Tmr2 => w.set_pin_tmr2(alt),
            #[cfg(sys_ch585)]
            RemapGroup::Tmr3 => w.set_pin_tmr3(alt),
            RemapGroup::Uart0 => w.set_pin_uart0(alt),
            RemapGroup::Uart1 => w.set_pin_uart1(alt),
            RemapGroup::Uart2 => w.set_pin_uart2(alt),
            #[cfg(sys_ch585)]
            RemapGroup::Uart3 => w.set_pin_uart3(alt),
            RemapGroup::Spi0 => w.set_pin_spi0(alt),
            RemapGroup::Pwmx => w.set_pin_pwmx(alt),
            #[cfg(sys_ch585)]
            RemapGroup::I2c => w.set_pin_i2c(alt),
        })
    })
}
