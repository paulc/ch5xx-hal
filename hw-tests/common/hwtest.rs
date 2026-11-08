// Shared by the hardware-test crates: SDI print and PASS/FAIL bookkeeping.
//
// Each crate's `lib.rs` includes this file and defines `SDI_DATA0`, the
// address of the debug module's `DATA0` word on its core.
//
// SDI print: the debugger drains `DATA0`/`DATA1` to WCH-LinkE's virtual
// serial port, up to 7 bytes per transfer, byte 0 of `DATA0` holding the
// length and `DATA0` written last (`wlink --enable-sdi-print
// --watch-serial`). The HAL's own `debug::SDIPrint` only drives it on CH592,
// so the tests carry their own. Writes give up after a bounded wait, so a
// test still runs with no debugger draining the console.

use portable_atomic::{AtomicU32, Ordering};

const DATA0: *mut u32 = crate::SDI_DATA0 as *mut u32;
const DATA1: *mut u32 = (crate::SDI_DATA0 + 4) as *mut u32;

/// Polls a chunk may wait for the debugger before the message is dropped.
const SPINS: u32 = 20_000;

/// Arm the console. Call once, before printing.
pub fn sdi_enable() {
    unsafe {
        core::ptr::write_volatile(DATA0, 0);
        qingke::riscv::asm::delay(100_000);
    }
}

fn busy() -> bool {
    unsafe { core::ptr::read_volatile(DATA0) != 0 }
}

/// Write `s`, dropping the rest of it once a chunk waits too long.
pub fn sdi_write(s: &str) {
    for chunk in s.as_bytes().chunks(7) {
        let mut waited = 0u32;
        while busy() {
            waited += 1;
            if waited > SPINS {
                return;
            }
        }
        let mut data = [0u8; 8];
        data[1..chunk.len() + 1].copy_from_slice(chunk);
        data[0] = chunk.len() as u8;
        unsafe {
            core::ptr::write_volatile(DATA1, u32::from_le_bytes(data[4..].try_into().unwrap()));
            core::ptr::write_volatile(DATA0, u32::from_le_bytes(data[..4].try_into().unwrap()));
        }
    }
}

struct Sdi;

impl core::fmt::Write for Sdi {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        sdi_write(s);
        Ok(())
    }
}

/// Format `args` to the console and end the line.
pub fn write_line(args: core::fmt::Arguments) {
    let _ = core::fmt::write(&mut Sdi, args);
    sdi_write("\n");
}

/// `println!` over SDI print; never blocks for long.
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {
        $crate::write_line(format_args!($($arg)*))
    };
}

static PASS: AtomicU32 = AtomicU32::new(0);
static FAIL: AtomicU32 = AtomicU32::new(0);

/// Report one check as `name: PASS` or `name: FAIL` and count it.
pub fn check(name: &str, ok: bool) -> bool {
    write_line(format_args!("{}: {}", name, if ok { "PASS" } else { "FAIL" }));
    if ok {
        PASS.fetch_add(1, Ordering::Relaxed);
    } else {
        FAIL.fetch_add(1, Ordering::Relaxed);
    }
    ok
}

/// Report a check whose detail the caller formats, e.g. a measured value.
pub fn check_fmt(ok: bool, args: core::fmt::Arguments) -> bool {
    let _ = core::fmt::write(&mut Sdi, args);
    sdi_write(if ok { ": PASS\n" } else { ": FAIL\n" });
    if ok {
        PASS.fetch_add(1, Ordering::Relaxed);
    } else {
        FAIL.fetch_add(1, Ordering::Relaxed);
    }
    ok
}

/// `check_fmt` with `format_args!` syntax: `check!(ok, "spi {} Hz", hz)`.
#[macro_export]
macro_rules! check {
    ($ok:expr, $($arg:tt)*) => {
        $crate::check_fmt($ok, format_args!($($arg)*))
    };
}

/// Note a check that cannot run on this HAL, so the summary says why.
pub fn skip(name: &str, why: &str) {
    write_line(format_args!("{}: SKIP ({})", name, why));
}

/// The counts so far.
pub fn counts() -> (u32, u32) {
    (PASS.load(Ordering::Relaxed), FAIL.load(Ordering::Relaxed))
}

/// Print the summary line `hw-test.sh` waits for, once a second, forever.
pub fn done(test: &str) -> ! {
    let (pass, fail) = counts();
    loop {
        write_line(format_args!("{} done: {} pass, {} fail", test, pass, fail));
        qingke::riscv::asm::delay(20_000_000);
    }
}

/// Clock the 32 kHz domain from the internal RC and stop the crystal.
///
/// The crystal oscillator holds PA10/PA11 while it runs, and a reset leaves it
/// running (a BLE or RTC run can leave it on). The HAL's `Rtc` does not select
/// the 32 kHz source, so tests that use those pins call this first.
pub fn select_lsi() {
    let sys = ch5xx_hal::pac::SYS;
    crate::safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_int32k_pon(true)));
    crate::safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_osc32k_xt(false)));
    qingke::riscv::asm::delay(10_000);
    crate::safe_access(|| sys.ck32k_config().modify(|w| w.set_clk_xt32k_pon(false)));
}

/// Report a panic as a failed check and keep printing a summary, so a panic
/// shows up in `hw-test.sh`'s table instead of as a test that never finished.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    FAIL.fetch_add(1, Ordering::Relaxed);
    let (pass, fail) = counts();
    loop {
        write_line(format_args!("panic: {}: FAIL", info));
        write_line(format_args!("panic done: {} pass, {} fail", pass, fail));
        qingke::riscv::asm::delay(20_000_000);
    }
}

/// The low word of the core's cycle counter (`cycle`, CSR 0xC00). It does not
/// count on CH584/CH585 (`clocks` checks it), so the delays below do not use it.
pub fn cycles() -> u32 {
    let c: u32;
    unsafe { core::arch::asm!("csrr {0}, 0xC00", out(reg) c) };
    c
}

/// `qingke::riscv::asm::delay` loops per millisecond, measured on first use.
static LOOPS_PER_MS: AtomicU32 = AtomicU32::new(0);

/// Busy-wait `us` microseconds on a loop calibrated once against embassy time
/// over 10 ms, so a short wait costs no timebase reads (`Delay` and `Instant`
/// add tens of microseconds each).
pub fn delay_us(us: u32) {
    let mut per_ms = LOOPS_PER_MS.load(Ordering::Relaxed);
    if per_ms == 0 {
        let t0 = embassy_time::Instant::now();
        let mut chunks = 0u32;
        while t0.elapsed().as_micros() < 10_000 {
            qingke::riscv::asm::delay(1000);
            chunks += 1;
        }
        per_ms = (chunks * 1000 / 10).max(1);
        LOOPS_PER_MS.store(per_ms, Ordering::Relaxed);
    }
    qingke::riscv::asm::delay((us as u64 * per_ms as u64 / 1000) as u32);
}
