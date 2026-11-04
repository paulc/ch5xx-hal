//! Power management: the DC-DC converter, the low-power modes and their
//! wake-up sources, and the battery voltage monitor, after WCH's
//! `CH58x_pwr.c` / `CH59x_pwr.c`.
//!
//! # DC-DC
//!
//! After reset the core supply comes from an internal LDO. [`enable_dcdc`]
//! moves it to the DC-DC converter, which WCH rates at about 60% of the LDO's
//! current. The converter needs an inductor between `VSW` and `VDCID` (see
//! the datasheet's pin table); without one, enabling it browns the chip out.
//! Parts whose factory configuration says they have no DC-DC are refused, as
//! WCH's `PWR_DCDCCfg` does.
//!
//! # Low-power modes
//!
//! | Mode         | Core                 | Clocks              | Ended by           | Then    |
//! |--------------|----------------------|---------------------|--------------------|---------|
//! | [`idle`]     | stopped              | running             | any interrupt      | returns |
//! | [`halt`]     | stopped              | HSE and PLL stopped | a wake-up source   | returns |
//! | [`sleep`]    | held by the ULP LDO  | stopped             | a wake-up source   | returns |
//! | [`shutdown`] | off                  | stopped             | a wake-up source   | resets  |
//!
//! The wake-up sources ([`configure_wake`]) are the GPIO pin interrupts (an
//! [`ExtiInput`](crate::exti::ExtiInput) with a wait in progress), the RTC
//! alarm and periodic flags, USB and the battery monitor. Datasheet currents
//! for CH584/CH585 (CH592): idle about 1.3 mA (1.1 mA), halt 0.2-1 mA
//! (0.18-1 mA), sleep 2.6-10 uA (0.8-2.5 uA), shutdown 0.65-5.5 uA
//! (0.3-1.3 uA), depending on what stays powered.
//!
//! Each mode runs from RAM with interrupts masked, as WCH's `__HIGH_CODE`
//! `LowPower_*` functions do. It parks the flash, slows HCLK where WCH does
//! (to 4 MHz in halt and sleep on CH584/CH585), waits, and puts the flash and
//! clock back before returning. The interrupt that ended the wait is taken
//! only then, so its handler may live in flash. An enabled interrupt that is
//! already pending ends the wait at once, so a mode can return early: check
//! what woke the chip and go back to sleep if need be. Clear the flag that
//! did (an RTC flag, say), or the next wait may end at once too.
//!
//! HCLK stops in halt and sleep, and SysTick with it: [`crate::delay`] and the
//! embassy time driver lose that time.

use crate::pac;
use crate::sys::safe_access;

/// Error from [`enable_dcdc`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The factory configuration marks this part as having no DC-DC.
    DcdcNotSupported,
}

/// Hardware configuration word in the factory area (`ROM_CFG_ADR_HW`). WCH
/// reads it through its ISP library (`CMD_GET_ROM_INFO`); the area is
/// memory-mapped too, as the SDK's direct reads of `ROM_CFG_CHIP_ID` and
/// `ROM_CFG_TMP_25C` rely on.
const ROM_CFG_ADR_HW: *const u32 = 0x0007_f00c as *const u32;
/// Set in [`ROM_CFG_ADR_HW`] on parts without DC-DC.
const HW_NO_DCDC: u32 = 1 << 13;

/// Whether the factory configuration allows the DC-DC converter. It says
/// nothing about the board: the inductor must be fitted as well.
pub fn dcdc_supported() -> bool {
    let hw = unsafe { ROM_CFG_ADR_HW.read_volatile() };
    hw & HW_NO_DCDC == 0
}

/// Switch the core supply to the DC-DC converter, as WCH's `PWR_DCDCCfg`:
/// the auxiliary charge pump and the converter's bias first, then, 10 us
/// later, the converter itself. Needs [`crate::init`] (for the delay).
///
/// Only with the inductor fitted (see the [module docs](self)).
pub fn enable_dcdc() -> Result<(), Error> {
    if !dcdc_supported() {
        return Err(Error::DcdcNotSupported);
    }
    if is_dcdc_enabled() {
        return Ok(());
    }
    let sys = pac::SYS;
    safe_access(|| {
        sys.aux_power_adj().modify(|w| w.set_dcdc_charge(true));
        sys.power_plan().modify(|w| w.set_pwr_dcdc_pre(true));
    });
    crate::delay::Delay.delay_us(10);
    safe_access(|| sys.power_plan().modify(|w| w.set_pwr_dcdc_en(true)));
    Ok(())
}

/// Switch the core supply back to the LDO.
pub fn disable_dcdc() {
    let sys = pac::SYS;
    safe_access(|| {
        sys.aux_power_adj().modify(|w| w.set_dcdc_charge(false));
        sys.power_plan().modify(|w| {
            w.set_pwr_dcdc_en(false);
            w.set_pwr_dcdc_pre(false);
        });
    });
}

/// Whether the core runs from the DC-DC converter.
pub fn is_dcdc_enabled() -> bool {
    pac::SYS.power_plan().read().pwr_dcdc_en()
}

/// The events that end [`halt`], [`sleep`] and [`shutdown`] (`R8_SLP_WAKE_CTRL`).
/// [`idle`] ends on any interrupt and ignores this.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct WakeConfig {
    /// A GPIO pin interrupt: a pin armed by an
    /// [`ExtiInput`](crate::exti::ExtiInput) wait that is in progress.
    pub gpio: bool,
    /// The RTC alarm or periodic flag
    /// ([`Rtc::set_alarm`](crate::rtc::Rtc::set_alarm),
    /// [`Rtc::set_periodic`](crate::rtc::Rtc::set_periodic)).
    pub rtc: bool,
    /// Full-speed USB bus activity.
    pub usb: bool,
    /// High-speed USB bus activity.
    #[cfg(sys_ch585)]
    pub usb_hs: bool,
    /// The battery monitor's low flag (see [`set_battery_monitor`]).
    pub battery: bool,
}

/// Select the wake-up sources, as WCH's `PWR_PeriphWakeUpCfg` with its
/// default (short) wake-up delay: 520 cycles plus the oscillator start-up.
/// Wake-up events are latched, so a short pulse is enough.
pub fn configure_wake(config: WakeConfig) {
    let sys = pac::SYS;
    safe_access(|| {
        sys.slp_wake_ctrl().write(|w| {
            w.set_slp_gpio_wake(config.gpio);
            w.set_slp_rtc_wake(config.rtc);
            w.set_slp_usb_wake(config.usb);
            #[cfg(sys_ch585)]
            w.set_slp_usb2_wake(config.usb_hs);
            w.set_slp_bat_wake(config.battery);
            w.set_wake_ev_mode(true);
        });
        sys.slp_power_ctrl().modify(|w| w.set_wake_dly_mod(WAKE_DLY_SHORT));
    });
}

/// `RB_WAKE_DLY_MOD`: 520 cycles plus the oscillator start-up.
const WAKE_DLY_SHORT: u8 = 0b01;

/// Battery voltage monitor configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BatteryMonitor {
    Off,
    /// The precise detector (about 210 uA on CH584/CH585, 80 uA on CH592),
    /// for use while running: [`BatteryStatus::low`] below the level,
    /// [`BatteryStatus::lower`] 0.25 V below that. The low-power modes turn
    /// it off while they wait.
    Precise(PreciseLevel),
    /// The low-power monitor (about 1 uA), which keeps running in the
    /// low-power modes and can wake the chip ([`WakeConfig::battery`]):
    /// [`BatteryStatus::low`] below the level.
    LowPower(LowPowerLevel),
}

/// Low-voltage level of the precise detector; the "lower" level is 0.25 V
/// below it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PreciseLevel {
    V1_95,
    V2_15,
    V2_35,
    V2_55,
}

/// Level of the low-power monitor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LowPowerLevel {
    V1_7,
    V1_8,
    V1_9,
    V2_0,
    V2_1,
    V2_2,
    V2_3,
    V2_4,
}

/// Battery monitor result (`R8_BAT_STATUS`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryStatus {
    /// Below the configured level.
    pub low: bool,
    /// Below the precise detector's lower level.
    pub lower: bool,
}

/// Configure the battery voltage monitor, as WCH's `PowerMonitor` (without
/// its interrupt enables: with both on, a low battery raises an NMI).
pub fn set_battery_monitor(monitor: BatteryMonitor) {
    let sys = pac::SYS;
    match monitor {
        BatteryMonitor::Off => safe_access(|| sys.bat_det_ctrl().write(|w| w.0 = 0)),
        BatteryMonitor::Precise(level) => safe_access(|| {
            sys.bat_det_ctrl().write(|w| w.set_bat_det_en(true));
            sys.bat_det_cfg().write(|w| w.set_bat_low_vth(level as u8));
        }),
        BatteryMonitor::LowPower(level) => {
            let level = level as u8;
            safe_access(|| {
                // `BAT_DET_EN` is `BAT_LOW_VTHX` (the level's top bit) once
                // `BAT_MON_EN` is set.
                sys.bat_det_ctrl().write(|w| w.0 = BAT_MON_EN | (level >> 2));
                sys.bat_det_cfg().write(|w| w.set_bat_low_vth(level & 0b11));
            })
        }
    }
}

/// `RB_BAT_MON_EN`.
const BAT_MON_EN: u8 = 1 << 1;
/// `RB_BAT_DET_EN`.
const BAT_DET_EN: u8 = 1 << 0;

/// The battery monitor's flags.
pub fn battery_status() -> BatteryStatus {
    let s = pac::SYS.bat_status().read();
    BatteryStatus {
        low: s.bat_stat_low(),
        lower: s.bat_stat_lower(),
    }
}

/// Cause of the last reset (`RB_RESET_FLAG`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetReason {
    /// A software reset (also the debugger's).
    Software,
    /// Power-on.
    PowerOn,
    /// The watchdog.
    Watchdog,
    /// The reset pin.
    Pin,
    /// A wake-up from [`shutdown`].
    ShutdownWake,
    /// Power-on reset while in sleep or shutdown.
    PowerLossInSleep,
}

/// What caused the last reset.
pub fn reset_reason() -> ResetReason {
    match pac::SYSTEM_CONTROL.reset_status().read().reset_flag() {
        0b000 => ResetReason::Software,
        0b001 => ResetReason::PowerOn,
        0b010 => ResetReason::Watchdog,
        0b011 => ResetReason::Pin,
        0b101 => ResetReason::ShutdownWake,
        _ => ResetReason::PowerLossInSleep,
    }
}

/// What stays powered in [`sleep`], besides the core, the peripherals and
/// all the SRAM, which the program needs to carry on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SleepConfig {
    /// Keep the flash powered (`RB_PWR_XROM`). Otherwise [`sleep`] waits
    /// for it after the wake-up (300 us on CH584/CH585) before returning.
    pub flash: bool,
    /// Keep the USB and BLE configuration powered (`RB_PWR_EXTEND`).
    pub usb_ble: bool,
}

/// What stays powered in [`shutdown`]. The reset that follows the wake-up
/// clears `.bss` and `.data`; data meant to survive belongs in `.uninit`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ShutdownConfig {
    /// The low SRAM block from `0x2000_0000` (`RB_PWR_RAM96K`: 96 KB on
    /// CH584/CH585; `RB_PWR_RAM24K`: 24 KB on CH592).
    pub main_ram: bool,
    /// The high SRAM block (`RB_PWR_RAM32K`: 32 KB from `0x2001_8000` on
    /// CH585; `RB_PWR_RAM2K`: 2 KB from `0x2000_6000` on CH592).
    pub retention_ram: bool,
}

/// Stop the core until an interrupt, with the flash in standby, as WCH's
/// `LowPower_Idle`. Clocks and peripherals keep running.
pub fn idle() {
    critical_section::with(|_| unsafe { idle_ram() })
}

/// Stop the core and the high-frequency clocks until a wake-up source
/// ([`configure_wake`]) fires, as WCH's `LowPower_Halt`. HCLK is restored
/// before returning.
pub fn halt() {
    critical_section::with(|_| unsafe { chip::halt_ram() })
}

/// Power down everything but the PMU, the core, the peripherals and the SRAM
/// (and what `config` keeps) until a wake-up source ([`configure_wake`])
/// fires, as WCH's `LowPower_Sleep`. The program then carries on. SysTick
/// loses its registers in sleep; they are put back, so it stands still
/// across the sleep as it does in [`halt`].
pub fn sleep(config: SleepConfig) {
    let mut plan = PWR_RAM_MAIN | PWR_RAM_RETENTION;
    if config.flash {
        plan |= PWR_XROM;
    }
    if config.usb_ble {
        plan |= PWR_EXTEND;
    }
    let us = chip::wake_us(config.flash);
    let cycles = (crate::rcc::clocks().sysclk.0 as u64 * us as u64 / 1_000_000) as u32;
    critical_section::with(|_| unsafe {
        let systick = SysTickState::save();
        chip::sleep_ram(plan, cycles);
        systick.restore();
    })
}

/// The SysTick registers, which sleep clears.
struct SysTickState {
    ctlr: u32,
    cntl: u32,
    #[cfg(systick_ch583)]
    cnth: u32,
    cmplr: u32,
    #[cfg(systick_ch583)]
    cmphr: u32,
}

impl SysTickState {
    fn save() -> Self {
        let r = pac::SYSTICK;
        Self {
            ctlr: r.stk_ctlr().read().0,
            cntl: r.stk_cntl().read().0,
            #[cfg(systick_ch583)]
            cnth: r.stk_cnth().read().0,
            cmplr: r.stk_cmplr().read().0,
            #[cfg(systick_ch583)]
            cmphr: r.stk_cmphr().read().0,
        }
    }

    /// Put the count and compare values back with the counter stopped, then
    /// restart it.
    fn restore(&self) {
        let r = pac::SYSTICK;
        r.stk_ctlr().write_value(pac::systick::regs::StkCtlr(0));
        r.stk_cntl().write_value(pac::systick::regs::StkCntl(self.cntl));
        #[cfg(systick_ch583)]
        r.stk_cnth().write_value(pac::systick::regs::StkCnth(self.cnth));
        r.stk_cmplr().write_value(pac::systick::regs::StkCmplr(self.cmplr));
        #[cfg(systick_ch583)]
        r.stk_cmphr().write_value(pac::systick::regs::StkCmphr(self.cmphr));
        r.stk_ctlr().write_value(pac::systick::regs::StkCtlr(self.ctlr));
    }
}

/// Power down everything but the PMU, the RTC clock and what `config`
/// keeps, as WCH's `LowPower_Shutdown`. A wake-up source ([`configure_wake`])
/// resets the chip ([`ResetReason::ShutdownWake`]); this never returns.
///
/// It turns the DC-DC converter off. If the chip does not go down (an
/// interrupt was pending), it resets at once.
pub fn shutdown(config: ShutdownConfig) -> ! {
    let mut plan = 0;
    if config.main_ram {
        plan |= PWR_RAM_MAIN;
    }
    if config.retention_ram {
        plan |= PWR_RAM_RETENTION;
    }
    critical_section::with(|_| unsafe { chip::shutdown_ram(plan) });
    loop {}
}

// `R16_POWER_PLAN` bits.
const PWR_XROM: u16 = 1 << 0;
/// `RB_PWR_RAM32K` (CH584/CH585), `RB_PWR_RAM2K` (CH592).
const PWR_RAM_RETENTION: u16 = 1 << 1;
const PWR_CORE: u16 = 1 << 2;
const PWR_EXTEND: u16 = 1 << 3;
/// `RB_PWR_RAM96K` (CH584/CH585), `RB_PWR_RAM24K` (CH592).
const PWR_RAM_MAIN: u16 = 1 << 4;
const PWR_DCDC_EN: u16 = 1 << 9;
const PWR_DCDC_PRE: u16 = 1 << 10;
/// `RB_PWR_MUST_0010`: bits 14:11 must be written as 0010b.
const PWR_MUST: u16 = 0b0010 << 11;
const PWR_PLAN_EN: u16 = 1 << 15;

// Registers for the RAM routines, which must not call into flash: raw
// pointers only (`read_volatile`/`write_volatile` are always inlined), as
// `rcc`'s `set_sysclk_ram`.
#[cfg(sys_ch585)]
const CLK_SYS_CFG: *mut u16 = 0x4000_1008 as *mut u16;
const SLP_POWER_CTRL: *mut u8 = 0x4000_100f as *mut u8;
const POWER_PLAN: *mut u16 = 0x4000_1020 as *mut u16;
const BAT_DET_CTRL: *mut u8 = 0x4000_1024 as *mut u8;
const XT32K_TUNE: *mut u8 = 0x4000_102e as *mut u8;
const SAFE_ACCESS_SIG: *mut u8 = 0x4000_1040 as *mut u8;
const GLOB_ROM_CFG: *mut u8 = 0x4000_1044 as *mut u8;
const RST_WDOG_CTRL: *mut u8 = 0x4000_1046 as *mut u8;
const XT32M_TUNE: *mut u8 = 0x4000_104e as *mut u8;
// The flash controller's registers are not in the metapac yet.
const FLASH_DATA: *mut u8 = 0x4000_1804 as *mut u8;
#[cfg(sys_ch585)]
const FLASH_SCK: *mut u8 = 0x4000_1805 as *mut u8;
const FLASH_CTRL: *mut u8 = 0x4000_1806 as *mut u8;
#[cfg(sys_ch585)]
const FLASH_CFG: *mut u8 = 0x4000_1807 as *mut u8;
const PFIC_ISR0: *const u32 = 0xe000_e000 as *const u32;
const PFIC_ISR1: *const u32 = 0xe000_e004 as *const u32;
const PFIC_IPR0: *const u32 = 0xe000_e020 as *const u32;
const PFIC_IPR1: *const u32 = 0xe000_e024 as *const u32;
const PFIC_SCTLR: *mut u32 = 0xe000_ed10 as *mut u32;

const SCTLR_SLEEPDEEP: u32 = 1 << 2;
const SCTLR_WFITOWFE: u32 = 1 << 3;
const SCTLR_SEVONPEND: u32 = 1 << 4;
const SCTLR_SETEVENT: u32 = 1 << 5;

/// `R8_SLP_POWER_CTRL.RB_WAKE_DLY_MOD`.
#[cfg(sys_ch585)]
const SLP_WAKE_DLY_MOD: u8 = 0b11;
/// `RB_RAM_RET_LV` (CH592), `RB_RAM_RETENT_LV` = 01 (CH584/CH585): the RAM
/// retention supply at low voltage.
const SLP_RAM_RET_LV: u8 = 1 << 6;
/// `RB_SOFTWARE_RESET`.
const RST_SOFTWARE: u8 = 1 << 0;

/// Write `$body` in safe-access mode. A macro, not a closure: a closure could
/// be emitted out of line (in flash) in a debug build. On CH584/CH585 the
/// window stays open until the signature is cleared only while
/// `SAFE_AUTO_EN` is clear (see [`begin`]).
macro_rules! sam {
    ($($body:tt)*) => {
        core::arch::asm!("fence.i");
        wr(SAFE_ACCESS_SIG, 0x57);
        wr(SAFE_ACCESS_SIG, 0xa8);
        core::arch::asm!("fence.i");
        $($body)*
        wr(SAFE_ACCESS_SIG, 0);
        core::arch::asm!("fence.i");
    };
}

#[inline(always)]
unsafe fn rd<T: Copy>(p: *const T) -> T {
    unsafe { core::ptr::read_volatile(p) }
}

#[inline(always)]
unsafe fn wr<T>(p: *mut T, v: T) {
    unsafe { core::ptr::write_volatile(p, v) }
}

/// Start a RAM routine: keep safe-access windows open (CH584/CH585), and
/// clear the core's event latch, which any interrupt since the last wait has
/// set and which would end the coming wait at once (`SEV` then `WFE`, as
/// WCH's `__WFE`). `wfi` waits for an event (`WFITOWFE`, which qingke-rt
/// sets), and a pending interrupt is one (`SEVONPEND`, which
/// [`crate::init`] sets).
#[inline(always)]
unsafe fn begin() {
    unsafe {
        #[cfg(sys_ch585)]
        chip::hold_safe_access(true);
        let s = rd(PFIC_SCTLR) & !(SCTLR_SLEEPDEEP | SCTLR_SETEVENT);
        wr(PFIC_SCTLR, s | SCTLR_WFITOWFE | SCTLR_SEVONPEND);
        wr(PFIC_SCTLR, s | SCTLR_WFITOWFE | SCTLR_SEVONPEND | SCTLR_SETEVENT);
        core::arch::asm!("wfi");
    }
}

/// End a RAM routine: back to plain sleep for the next `wfi` (an executor's).
#[inline(always)]
unsafe fn end() {
    unsafe {
        deep_sleep(false);
        #[cfg(sys_ch585)]
        chip::hold_safe_access(false);
    }
}

/// `SLEEPDEEP`: whether `wfi` enters halt/sleep/shutdown rather than idle.
#[inline(always)]
unsafe fn deep_sleep(on: bool) {
    unsafe {
        let s = rd(PFIC_SCTLR) & !(SCTLR_SLEEPDEEP | SCTLR_SETEVENT);
        wr(PFIC_SCTLR, if on { s | SCTLR_SLEEPDEEP } else { s });
    }
}

/// Wait for an event, unless an enabled interrupt is pending already: its
/// event may have been consumed by [`begin`].
#[inline(always)]
unsafe fn wait() {
    unsafe {
        let pending = (rd(PFIC_ISR0) & rd(PFIC_IPR0)) | (rd(PFIC_ISR1) & rd(PFIC_IPR1));
        if pending == 0 {
            core::arch::asm!("wfi", "nop", "nop");
        }
    }
}

/// Busy-wait at least `cycles` HCLK cycles (an iteration takes more than
/// one). Not SysTick: it does not count straight after a sleep.
#[inline(always)]
unsafe fn spin(cycles: u32) {
    let mut n = cycles;
    while n != 0 {
        unsafe { core::arch::asm!("nop") };
        n -= 1;
    }
}

/// Send one command byte to the flash chip, as `FLASH_ROM_BEG` /
/// `FLASH_ROM_END` in WCH's ISP library (`libISP585.a`/`libISP592.a`, which
/// this HAL does not link).
#[inline(always)]
unsafe fn flash_command(cmd: u8) {
    unsafe {
        wr(FLASH_CTRL, 0);
        wr(FLASH_CTRL, 5);
        core::arch::asm!("nop");
        wr(FLASH_DATA, cmd);
        while rd(FLASH_CTRL) & 0x80 != 0 {}
        wr(FLASH_CTRL, 0);
    }
}

/// Take the flash out of its continuous-read mode (`0xFF`), and with `reset`
/// reset it too (`0x66`, `0x99`): the ISP library's `FLASH_EEPROM_CMD`
/// preamble, then its `CMD_FLASH_ROM_SW_RESET` (WCH's `FLASH_ROM_SW_RESET`).
#[inline(always)]
unsafe fn flash_park(reset: bool) {
    unsafe {
        // `RB_ROM_CTRL_EN` and the write enables, as the ISP library does.
        sam! { wr(GLOB_ROM_CFG, rd(GLOB_ROM_CFG) | 0xe0); }
        wr(FLASH_CTRL, 4);
        flash_command(0xff);
        if reset {
            flash_command(0x66);
            flash_command(0x99);
        }
        sam! { wr(GLOB_ROM_CFG, rd(GLOB_ROM_CFG) & 0x10); }
    }
}

/// Keep the low-power battery monitor; turn the precise detector (up to
/// 210 uA) off.
#[inline(always)]
fn bat_det_while_waiting(ctrl: u8) -> u8 {
    if ctrl & BAT_MON_EN != 0 {
        ctrl
    } else {
        ctrl & !BAT_DET_EN
    }
}

/// [`idle`], in RAM: WCH's `LowPower_Idle`.
#[inline(never)]
#[unsafe(link_section = ".data.ch5xx_hal.pwr.idle")]
unsafe fn idle_ram() {
    unsafe {
        begin();
        flash_park(true);
        // "Flash off", as `LowPower_Idle`.
        wr(FLASH_CTRL, 4);
        wait();
        end();
    }
}

#[cfg(sys_ch585)]
mod chip {
    use super::*;

    /// The flash needs 300 us after a sleep before code runs from it
    /// (`LowPower_Sleep`).
    const FLASH_WAKE_US: u32 = 300;
    /// The crystal needs about 1.6 ms after a sleep, which WCH's PM example
    /// waits out on the HSI (as `PM_LowPower_Sleep`).
    const HSE_WAKE_US: u32 = 1600;

    /// How long [`sleep_ram`] runs on the HSI after the wake-up, in
    /// microseconds of the clock in use now.
    pub(super) fn wake_us(flash_kept: bool) -> u32 {
        if unsafe { rd(CLK_SYS_CFG) } & RB_OSC32M_SEL != 0 {
            HSE_WAKE_US
        } else if flash_kept {
            0
        } else {
            FLASH_WAKE_US
        }
    }

    const HFCK_PWR_CTRL: *mut u8 = 0x4000_100a as *mut u8;
    const SAFE_MODE_CTRL: *mut u8 = 0x4000_1010 as *mut u8;

    const RB_SAFE_AUTO_EN: u8 = 1 << 0;
    const RB_CLK_RC16M_PON: u8 = 1 << 1;
    const RB_CLK_XT32M_KEEP: u8 = 1 << 3;
    const RB_XROM_SCLK_SEL: u16 = 1 << 8;
    const RB_OSC32M_SEL: u16 = 1 << 9;
    /// WCH's `CLK_SOURCE_HSE_4MHz` and `CLK_SOURCE_HSI_4MHz`.
    const CLK_HSE_4MHZ: u16 = RB_OSC32M_SEL | 8;
    const CLK_HSI_4MHZ: u16 = 4;
    /// WCH's `CLK_SOURCE_HSI_PLL_13MHz`.
    const CLK_HSI_PLL_13MHZ: u16 = RB_XROM_SCLK_SEL | (0b01 << 6) | 24;

    /// Keep safe-access windows open until the signature is cleared (clear
    /// `SAFE_AUTO_EN`), or not, as `rcc`'s `set_sysclk_ram`.
    #[inline(always)]
    pub(super) unsafe fn hold_safe_access(hold: bool) {
        unsafe {
            let v = rd(SAFE_MODE_CTRL);
            wr(
                SAFE_MODE_CTRL,
                if hold {
                    v & !RB_SAFE_AUTO_EN
                } else {
                    v | RB_SAFE_AUTO_EN
                },
            );
        }
    }

    /// Run HCLK at 4 MHz from the oscillator it uses, with matching flash
    /// timing, short wake-up delay: the common part of `LowPower_Halt` and
    /// `LowPower_Sleep`.
    #[inline(always)]
    unsafe fn slow_clock(clk: u16, sck: u8) {
        unsafe {
            sam! {
                wr(SLP_POWER_CTRL, (rd(SLP_POWER_CTRL) & !SLP_WAKE_DLY_MOD) | WAKE_DLY_SHORT);
                wr(FLASH_CFG, 0x57);
                wr(FLASH_SCK, sck & !(1 << 4));
                wr(CLK_SYS_CFG, if clk & RB_OSC32M_SEL != 0 { CLK_HSE_4MHZ } else { CLK_HSI_4MHZ });
            }
        }
    }

    /// The PLL from HSI needs a moment after the wake-up (`LowPower_Halt`).
    #[inline(always)]
    unsafe fn settle(clk: u16) {
        if clk & RB_OSC32M_SEL == 0 && clk & RB_XROM_SCLK_SEL != 0 {
            // A counter, not a range: `Range::next` is not inlined in a debug
            // build.
            let mut n = 40u32;
            while n != 0 {
                unsafe { core::arch::asm!("nop") };
                n -= 1;
            }
        }
    }

    /// [`halt`](super::halt), in RAM: WCH's `LowPower_Halt`. The crystal
    /// bias and the battery detector are put back afterwards, which WCH
    /// leaves to the caller.
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.pwr.halt")]
    pub(super) unsafe fn halt_ram() {
        unsafe {
            begin();
            let clk = rd(CLK_SYS_CFG);
            let cfg = rd(FLASH_CFG);
            let sck = rd(FLASH_SCK);
            let tune = rd(XT32M_TUNE);
            let bat = rd(BAT_DET_CTRL);
            flash_park(true);
            wr(FLASH_CTRL, 4);
            // The crystal at 150% bias, to restart quickly, unless it is
            // kept running.
            let halt_tune = if rd(HFCK_PWR_CTRL) & RB_CLK_XT32M_KEEP == 0 {
                (tune & 0xfc) | 0x03
            } else {
                tune
            };
            sam! {
                wr(BAT_DET_CTRL, bat_det_while_waiting(bat));
                wr(XT32M_TUNE, halt_tune);
            }
            slow_clock(clk, sck);
            deep_sleep(true);
            wait();
            settle(clk);
            sam! {
                wr(FLASH_CFG, cfg);
                wr(FLASH_SCK, sck);
                wr(CLK_SYS_CFG, clk);
                wr(XT32M_TUNE, tune);
                wr(BAT_DET_CTRL, bat);
            }
            end();
        }
    }

    /// [`sleep`](super::sleep), in RAM: WCH's `LowPower_Sleep`, wrapped as
    /// its PM example's `PM_LowPower_Sleep`: HCLK moves from the HSE to the
    /// HSI first (the same rate, from the PLL), and stays there for
    /// `wake_cycles` after the wake-up, while the crystal restarts and the
    /// flash powers up. Not done: WCH's reset of the USBHS registers after a
    /// sleep with `RB_PWR_EXTEND`, which only matters to a USBHS driver (this
    /// HAL has none).
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.pwr.sleep")]
    pub(super) unsafe fn sleep_ram(keep: u16, wake_cycles: u32) {
        unsafe {
            begin();
            let clk = rd(CLK_SYS_CFG);
            let hfck = rd(HFCK_PWR_CTRL);
            let cfg = rd(FLASH_CFG);
            let sck = rd(FLASH_SCK);
            let tune = rd(XT32M_TUNE);
            let bat = rd(BAT_DET_CTRL);
            // The crystal stops in sleep and is not ready when the core
            // wakes, so run from the HSI around it.
            let hsi_clk = clk & !RB_OSC32M_SEL;
            sam! {
                wr(HFCK_PWR_CTRL, hfck | RB_CLK_RC16M_PON);
                wr(CLK_SYS_CFG, hsi_clk);
            }
            sam! {
                wr(BAT_DET_CTRL, bat_det_while_waiting(bat));
                wr(XT32M_TUNE, (tune & 0xfc) | 0x03);
            }
            deep_sleep(true);
            let plan = (rd(POWER_PLAN) & (PWR_DCDC_EN | PWR_DCDC_PRE)) | PWR_PLAN_EN | PWR_CORE | PWR_MUST | keep;
            sam! { wr(POWER_PLAN, plan); }
            slow_clock(hsi_clk, sck);
            wait();
            settle(hsi_clk);
            sam! {
                wr(FLASH_CFG, cfg);
                wr(FLASH_SCK, sck);
                wr(CLK_SYS_CFG, hsi_clk);
            }
            sam! { wr(POWER_PLAN, rd(POWER_PLAN) & !PWR_PLAN_EN); }
            spin(wake_cycles);
            sam! {
                wr(CLK_SYS_CFG, clk);
                wr(HFCK_PWR_CTRL, hfck);
                wr(XT32M_TUNE, tune);
                wr(BAT_DET_CTRL, bat);
            }
            end();
        }
    }

    /// [`shutdown`](super::shutdown), in RAM: WCH's `LowPower_Shutdown`. WCH
    /// also sets `R16_INT32K_TUNE` to 0xFFFF before its fallback reset; this
    /// keeps the LSI trim.
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.pwr.shutdown")]
    pub(super) unsafe fn shutdown_ram(keep: u16) -> ! {
        unsafe {
            begin();
            flash_park(true);
            crate::rcc::set_sysclk_ram(CLK_HSI_PLL_13MHZ);
            // `set_sysclk_ram` lets safe-access windows close by themselves.
            hold_safe_access(true);
            let bat = rd(BAT_DET_CTRL);
            sam! {
                wr(BAT_DET_CTRL, bat_det_while_waiting(bat));
                // The 32 kHz crystal at its rated bias.
                wr(XT32K_TUNE, (rd(XT32K_TUNE) & 0xfc) | 0x01);
            }
            deep_sleep(true);
            sam! { wr(SLP_POWER_CTRL, rd(SLP_POWER_CTRL) | SLP_RAM_RET_LV); }
            sam! { wr(POWER_PLAN, PWR_PLAN_EN | keep); }
            wait();
            sam! { wr(RST_WDOG_CTRL, rd(RST_WDOG_CTRL) | RST_SOFTWARE); }
            loop {}
        }
    }
}

#[cfg(sys_ch592)]
mod chip {
    use super::*;

    /// WCH's `LowPower_Sleep` waits 20 us after restoring the PLL.
    const PLL_WAKE_US: u32 = 20;

    /// How long [`sleep_ram`] waits after the wake-up, in microseconds.
    pub(super) fn wake_us(_flash_kept: bool) -> u32 {
        PLL_WAKE_US
    }

    const PLL_CONFIG: *mut u8 = 0x4000_104b as *mut u8;
    /// `R8_PLL_CONFIG` bit 5, which WCH sets around halt and sleep
    /// (undocumented).
    const PLL_SLEEP: u8 = 1 << 5;
    /// `RB_XT_PRE_EN`: start the HSE early before an RTC wake-up (unused).
    const PWR_XT_PRE_EN: u16 = 1 << 14;
    /// WCH's `FLASH_CFG` for HCLK from the HSE (as `rcc`).
    const FLASH_CFG_HSE: u8 = 0x51;

    /// [`halt`](super::halt), in RAM: WCH's `LowPower_Halt`. The crystal
    /// biases and the battery detector are put back afterwards, which WCH
    /// leaves to the caller.
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.pwr.halt")]
    pub(super) unsafe fn halt_ram() {
        unsafe {
            begin();
            flash_park(true);
            wr(FLASH_CTRL, 4);
            let tune32k = rd(XT32K_TUNE);
            let tune = rd(XT32M_TUNE);
            let bat = rd(BAT_DET_CTRL);
            sam! { wr(BAT_DET_CTRL, bat_det_while_waiting(bat)); }
            sam! {
                // 32 kHz crystal at its rated bias, 32 MHz crystal at 150%.
                wr(XT32K_TUNE, (tune32k & 0xfc) | 0x01);
                wr(XT32M_TUNE, (tune & 0xfc) | 0x03);
            }
            sam! { wr(PLL_CONFIG, rd(PLL_CONFIG) | PLL_SLEEP); }
            deep_sleep(true);
            wait();
            sam! { wr(PLL_CONFIG, rd(PLL_CONFIG) & !PLL_SLEEP); }
            sam! {
                wr(XT32K_TUNE, tune32k);
                wr(XT32M_TUNE, tune);
                wr(BAT_DET_CTRL, bat);
            }
            end();
        }
    }

    /// [`sleep`](super::sleep), in RAM: WCH's `LowPower_Sleep` without the
    /// HSE early start (`RB_XT_PRE_EN`). WCH reads the MAC address first,
    /// for the flash command's preamble only (it ends the continuous read).
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.pwr.sleep")]
    pub(super) unsafe fn sleep_ram(keep: u16, wake_cycles: u32) {
        unsafe {
            begin();
            flash_park(false);
            let tune = rd(XT32M_TUNE);
            let bat = rd(BAT_DET_CTRL);
            sam! { wr(BAT_DET_CTRL, bat_det_while_waiting(bat)); }
            sam! { wr(XT32M_TUNE, (tune & 0xfc) | 0x03); }
            sam! { wr(POWER_PLAN, rd(POWER_PLAN) & !PWR_XT_PRE_EN); }
            deep_sleep(true);
            let plan = (rd(POWER_PLAN) & (PWR_DCDC_EN | PWR_DCDC_PRE)) | PWR_PLAN_EN | PWR_CORE | PWR_MUST | keep;
            sam! {
                wr(SLP_POWER_CTRL, rd(SLP_POWER_CTRL) | SLP_RAM_RET_LV);
                wr(POWER_PLAN, plan);
            }
            sam! { wr(PLL_CONFIG, rd(PLL_CONFIG) | PLL_SLEEP); }
            wait();
            sam! { wr(POWER_PLAN, rd(POWER_PLAN) & !PWR_PLAN_EN); }
            sam! { wr(PLL_CONFIG, rd(PLL_CONFIG) & !PLL_SLEEP); }
            spin(wake_cycles);
            sam! {
                wr(XT32M_TUNE, tune);
                wr(BAT_DET_CTRL, bat);
            }
            end();
        }
    }

    /// [`shutdown`](super::shutdown), in RAM: WCH's `LowPower_Shutdown`. WCH
    /// also sets `R16_INT32K_TUNE` to 0xFFFF before its fallback reset; this
    /// keeps the LSI trim.
    #[inline(never)]
    #[unsafe(link_section = ".data.ch5xx_hal.pwr.shutdown")]
    pub(super) unsafe fn shutdown_ram(keep: u16) -> ! {
        unsafe {
            begin();
            flash_park(true);
            let bat = rd(BAT_DET_CTRL);
            sam! { wr(BAT_DET_CTRL, bat_det_while_waiting(bat)); }
            sam! {
                wr(XT32K_TUNE, (rd(XT32K_TUNE) & 0xfc) | 0x01);
                wr(XT32M_TUNE, (rd(XT32M_TUNE) & 0xfc) | 0x03);
            }
            // HSE / 5 = 6.4 MHz, as WCH.
            crate::rcc::set_sysclk_ram(0b00, 5, FLASH_CFG_HSE);
            deep_sleep(true);
            sam! { wr(SLP_POWER_CTRL, rd(SLP_POWER_CTRL) | SLP_RAM_RET_LV); }
            sam! { wr(POWER_PLAN, PWR_PLAN_EN | keep); }
            wait();
            flash_park(true);
            sam! { wr(RST_WDOG_CTRL, rd(RST_WDOG_CTRL) | RST_SOFTWARE); }
            loop {}
        }
    }
}
