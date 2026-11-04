//! Debug output over the 2-wire debug link ("SDI print").
//!
//! The debugger drains the debug module's `DATA0`/`DATA1` registers to
//! WCH-LinkE's virtual serial port: up to 7 bytes per transfer, byte 0 of
//! `DATA0` holding the length, `DATA0` written last.
//!
//! - **CH592** (QingKe V4C) maps them at `0xE000_0380`, like CH32V20x/V30x.
//! - **CH584/CH585** (QingKe V3C) map them at `0xE000_0340`
//!   (`hartinfo.dataaddr` = 0x340, as `wlink --chip CH585 status` reports).
//!
//! WCH documents SDI print for neither, but it works on both
//! (`wlink --enable-sdi-print --watch-serial`). wlink 0.1.3 refuses to arm it
//! for `CH59X` and `CH585`: add them to `support_sdi_print()` in wlink's
//! `src/lib.rs`.
//!
//! Ported from ch32-hal's `debug` module; see also
//! <https://github.com/openwch/ch32v003/blob/main/EVT/EXAM/SDI_Printf/SDI_Printf/Debug/debug.c>

/// Console over the debug link.
pub struct SDIPrint;

/// Iterations [`SDIPrint::write_str_lossy`] tolerates per chunk. Kept small
/// because each iteration is a *debug-module* read, which is far slower than a
/// normal load, and deliberately *not* remembered across calls: a console that
/// attaches later must still receive output, so each message pays its own budget.
const SPINS: u32 = 20_000;

/// Polls [`sdi_write`] allows per chunk before deciding no debugger is
/// draining the console. Paid once: the verdict sticks until
/// [`SDIPrint::enable`].
const SPIN_LIMIT: u32 = 100_000;

/// Cleared by [`sdi_write`] when the debugger stops draining the console.
static SDI_ALIVE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(true);

mod imp {
    #[cfg(ch592)]
    use qingke::dm::{DATA0, DATA1};
    // qingke has no `dm` constants for V3C yet.
    #[cfg(any(ch584, ch585))]
    const DATA0: u32 = 0xE000_0340;
    #[cfg(any(ch584, ch585))]
    const DATA1: u32 = DATA0 + 4;

    const DEBUG_DATA0_ADDRESS: *mut u32 = DATA0 as *mut u32;
    const DEBUG_DATA1_ADDRESS: *mut u32 = DATA1 as *mut u32;

    pub(super) fn enable() {
        unsafe {
            core::ptr::write_volatile(DEBUG_DATA0_ADDRESS, 0);
            qingke::riscv::asm::delay(100000);
        }
    }

    #[inline]
    pub(super) fn is_busy() -> bool {
        unsafe { core::ptr::read_volatile(DEBUG_DATA0_ADDRESS) != 0 }
    }

    /// Hand one chunk of at most 7 bytes to the debugger.
    #[inline]
    fn put(chunk: &[u8]) {
        let mut data = [0u8; 8];
        data[1..chunk.len() + 1].copy_from_slice(chunk);
        data[0] = chunk.len() as u8;

        // data1 is the last 4 bytes of data
        let data1 = u32::from_le_bytes(data[4..].try_into().unwrap());
        let data0 = u32::from_le_bytes(data[..4].try_into().unwrap());

        unsafe {
            core::ptr::write_volatile(DEBUG_DATA1_ADDRESS, data1);
            core::ptr::write_volatile(DEBUG_DATA0_ADDRESS, data0);
        }
    }

    /// Wait up to `limit` polls for the previous chunk to drain.
    #[inline]
    fn wait(limit: u32) -> bool {
        let mut waited = 0u32;
        while is_busy() {
            waited += 1;
            if waited > limit {
                return false;
            }
        }
        true
    }

    pub(super) fn write(s: &str, spins: Option<u32>) -> core::fmt::Result {
        for chunk in s.as_bytes().chunks(7) {
            match spins {
                Some(limit) => {
                    if !wait(limit) {
                        return Ok(());
                    }
                }
                None => while is_busy() {},
            }
            put(chunk);
        }

        Ok(())
    }

    pub(super) fn write_raw(bytes: &[u8]) {
        if !super::SDI_ALIVE.load(Relaxed) {
            return;
        }
        for chunk in bytes.chunks(7) {
            if !wait(super::SPIN_LIMIT) {
                super::SDI_ALIVE.store(false, Relaxed);
                return;
            }
            put(chunk);
        }
    }

    use core::sync::atomic::Ordering::Relaxed;
}

impl SDIPrint {
    /// Arm the console. Call once before printing. Also re-arms
    /// [`sdi_write`] after it has given up.
    pub fn enable() {
        SDI_ALIVE.store(true, core::sync::atomic::Ordering::Relaxed);
        imp::enable()
    }

    /// Is the previous chunk still waiting for the debugger?
    #[inline]
    pub fn is_busy() -> bool {
        imp::is_busy()
    }

    /// Write, giving up (and dropping the rest of the message) once a chunk
    /// waits too long.
    ///
    /// SDI print is only drained while the debugger has it armed
    /// (`wlink --enable-sdi-print`). With it disarmed the blocking
    /// [`core::fmt::Write`] impl spins forever on the second chunk.
    pub fn write_str_lossy(s: &str) -> core::fmt::Result {
        imp::write(s, Some(SPINS))
    }
}

/// Write raw bytes to the console, with no formatting.
///
/// Never blocks for long: if a chunk is still waiting for the debugger after
/// 100 000 polls, the rest is dropped and every later
/// call returns at once, until [`SDIPrint::enable`] re-arms it. So a debugger
/// that attaches after that sees nothing from `sdi_write`; the
/// [`crate::println!`] family is unaffected.
pub fn sdi_write(bytes: &[u8]) {
    imp::write_raw(bytes)
}

/// Writer that routes formatting through [`SDIPrint::write_str_lossy`], so a
/// `try_println!` never blocks even though `SDIPrint`'s own [`core::fmt::Write`]
/// impl does.
struct LossyWriter;

impl core::fmt::Write for LossyWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        SDIPrint::write_str_lossy(s)
    }
}

/// Format `args` into the lossy writer and terminate the line.
pub fn write_fmt_lossy(args: core::fmt::Arguments) -> core::fmt::Result {
    use core::fmt::Write as _;

    let mut writer = LossyWriter;
    core::fmt::write(&mut writer, args)?;
    writer.write_str("\n")
}

impl core::fmt::Write for SDIPrint {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        imp::write(s, None)
    }
}

/// `println!` over SDI that drops the message rather than spinning when the
/// console is not armed. Use it in control loops; use [`crate::println!`] when the
/// console is known to be attached.
#[macro_export]
macro_rules! try_println {
    ($($arg:tt)*) => {
        {
            let _ = $crate::debug::write_fmt_lossy(format_args!($($arg)*));
        }
    };
}

/// Write each argument (`&[u8]`) with [`debug::sdi_write`](crate::debug::sdi_write),
/// each followed by a space, then a newline. No formatting, so it costs
/// little code and time:
///
/// ```ignore
/// sdi_writeln!(b"adc", hex);
/// ```
#[macro_export]
macro_rules! sdi_writeln {
    ($($arg:expr),* $(,)?) => {
        {
            $(
                $crate::debug::sdi_write($arg);
                $crate::debug::sdi_write(b" ");
            )*
            $crate::debug::sdi_write(b"\n");
        }
    };
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::writeln;

            writeln!(&mut $crate::debug::SDIPrint, $($arg)*).unwrap();
        }
    }
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            use core::write;

            write!(&mut $crate::debug::SDIPrint, $($arg)*).unwrap();
        }
    }
}

/// Turn the 2-wire debug interface off (or back on) at run time, freeing
/// PB14/PB15 (TIO/TCK) for GPIO and peripherals.
///
/// With `CFG_DEBUG_EN` set in the user config, those pins are the debug
/// link's alone (datasheet pin descriptions).
///
/// - CH592: `R16_PIN_ALTERNATE` bit 13 releases them until the next reset.
///   The datasheet says only parts whose lot number's fifth digit is 1 or
///   more implement the bit; on older parts it does nothing and the pins stay
///   with the debugger.
/// - CH584/CH585: `R8_SAFE_DEBUG_CTRL.RB_DEBUG_DIS`, a safe-access write.
///   Not yet tried on hardware.
///
/// While it is off, the debugger cannot attach and SDI print goes nowhere.
/// Reset the chip to get it back; leave a delay before calling this so the
/// debugger can still catch the chip after a reset.
pub fn set_debug_interface_enabled(enabled: bool) {
    #[cfg(ch592)]
    critical_section::with(|_| crate::pac::SYS.pin_alternate().modify(|w| w.set_debug_en(!enabled)));
    #[cfg(any(ch584, ch585))]
    crate::sys::safe_access(|| crate::pac::SYS.safe_debug_ctrl().write(|w| w.set_debug_dis(!enabled)));
}

/// Whether the user config enables the 2-wire debug interface
/// (`CFG_DEBUG_EN`), regardless of [`set_debug_interface_enabled`].
pub fn debug_interface_configured() -> bool {
    crate::pac::SYSTEM_CONTROL.glob_cfg_info().read().cfg_debug_en()
}
