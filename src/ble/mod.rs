//! BLE, with WCH's BLE library (`vendor/wch-ble`).
//!
//! The library holds the BLE controller and host, scheduled by WCH's TMOS (a
//! cooperative event scheduler). The application calls
//! `TMOS_SystemProcess()` over and over. Two ways to do that here:
//!
//! - With the `executor` feature, [`crate::embassy::Executor`] runs TMOS
//!   each time it has polled its tasks, and sleeps between BLE events when
//!   no task is ready (see [`sleep`]). This is the low-power way.
//! - With any other executor, spawn [`run`] as a task: it runs TMOS,
//!   yielding between passes so other tasks run too. The chip never sleeps.
//!
//! The library is not re-entrant: call it from thread level only, that is
//! from embassy tasks or TMOS callbacks (which run inside TMOS). All tasks
//! share one executor, so calls never overlap. The radio interrupts (BB,
//! LLE) go straight to the library: it points two of the PFIC's
//! vector-table-free (VTF) slots at its own handlers, which rely on the
//! hardware register stack that qingke-rt turns on.
//!
//! The TMOS timebase is the RTC (on the 32.768 kHz crystal by default, see
//! [`Config::clock`]): its time-of-day count, `CNT_2S:CNT_32K`, which wraps
//! once a day. So [`init`] takes the RTC, and the [`rtc`] driver
//! cannot be used alongside.
//!
//! # Linking
//!
//! Binaries add `-Tble.x` after `-Tlink.x` (in `build.rs`:
//! `println!("cargo:rustc-link-arg-bins=-Tble.x")`). It places what
//! qingke-rt's `link.x` does not know about: the library's `.highcode_lp`
//! (time-critical code, in RAM) and its C common symbols. Without it the
//! link fails with an undefined `__ble_highcode_lp_start`.
//!
//! Setup follows WCH's EVT (`EXAM/BLE/HAL`): `CH58x_BLEInit`, `HAL_Init`,
//! `HAL_TimeInit`.

pub mod adv;
pub mod ffi;
pub mod gap;
pub mod gatt;
pub mod gatt_server;
mod irq;
pub mod peripheral;
#[cfg(feature = "executor")]
pub mod sleep;
use core::ffi::CStr;
use core::mem::MaybeUninit;
use core::num::NonZeroU8;
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "executor")]
pub(crate) use sleep::executor_idle;

use self::ffi::*;
use crate::rtc::{self, ClockSource, Rtc};
use crate::sys::safe_access;
use crate::{pac, peripherals, Peri};

/// Library memory (`BLE_MEMHEAP_SIZE` in the EVT, which needs at least 4 KB).
const HEAP_SIZE: usize = 6 * 1024;

#[repr(C, align(4))]
struct BleHeap([MaybeUninit<u8>; HEAP_SIZE]);
static mut BLE_HEAP: BleHeap = BleHeap([MaybeUninit::uninit(); HEAP_SIZE]);

// Library buffer configuration (EVT defaults).
const BLE_BUFF_MAX_LEN: u16 = 27;
const BLE_BUFF_NUM: u8 = 5;
const BLE_TX_NUM_EVENT: u8 = 1;

/// HAL task event: periodic RF calibration.
const HAL_REG_INIT_EVENT: tmosEvents = 0x2000;
const BLE_CALIBRATION_PERIOD_MS: u32 = 120_000;

/// The TMOS clock: the RTC's time of day, `CNT_2S:CNT_32K`, which counts
/// 2 s units in its top half and wraps after 43200 of them (`RTC_MAX_COUNT`).
const RTC_MAX_COUNT: u32 = 0xA8C0_0000;
/// TMOS clock rate. On the LSI, [`init`] trims it to this rate too.
const CLOCK_HZ: u16 = 32_768;

/// Event counters, for diagnostics; prints as
/// `tmos=.. lle=.. deferred=.. rtc_irq=.. sleeps=.. slept=.. waits=.. clock=..`.
///
/// - `tmos`: `TMOS_SystemProcess` calls from [`run`] or the executor: how
///   often TMOS got the CPU.
/// - `lle`: link-layer interrupts (radio events: advertising, connection
///   events).
/// - `deferred`: link-layer work the LLE interrupt handed to thread level,
///   finished.
/// - `rtc_irq`: RTC interrupts (the library pends it to have TMOS run).
/// - `sleeps`: times the chip slept between events ([`sleep`]).
/// - `slept`: time spent in those sleeps (and the halts that end them), in
///   1/32768 s; printed in ms.
/// - `waits`: times TMOS waited in `wfi`, the next event being too close to
///   sleep.
/// - `clock`: the TMOS clock, the RTC time of day in 1/32768 s.
pub struct Stats {
    pub tmos_passes: AtomicU32,
    pub rtc_irqs: AtomicU32,
    pub sleeps: AtomicU32,
    pub slept: AtomicU32,
    pub waits: AtomicU32,
}

pub static STATS: Stats = Stats {
    tmos_passes: AtomicU32::new(0),
    rtc_irqs: AtomicU32::new(0),
    sleeps: AtomicU32::new(0),
    slept: AtomicU32::new(0),
    waits: AtomicU32::new(0),
};

impl Stats {
    /// Link-layer interrupts so far (`lle` above).
    pub fn lle_irqs(&self) -> u32 {
        irq::LLE_IRQS.load(Ordering::Relaxed)
    }

    /// Deferred link-layer work finished so far (`deferred` above).
    pub fn lle_deferred(&self) -> u32 {
        irq::LLE_DEFERRED.load(Ordering::Relaxed)
    }
}

/// Increment a counter. Each has a single writer, so a load and a store do.
fn count(counter: &AtomicU32) {
    counter.store(counter.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
}

impl core::fmt::Display for Stats {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "tmos={} lle={} deferred={} rtc_irq={} sleeps={} slept={}ms waits={} clock={}",
            self.tmos_passes.load(Ordering::Relaxed),
            self.lle_irqs(),
            self.lle_deferred(),
            self.rtc_irqs.load(Ordering::Relaxed),
            self.sleeps.load(Ordering::Relaxed),
            self.slept.load(Ordering::Relaxed) as u64 * 1000 / CLOCK_HZ as u64,
            self.waits.load(Ordering::Relaxed),
            clock(),
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// `VER_LIB` does not match the header the bindings were written from.
    LibVersion,
    /// The RTC clock did not start.
    Rtc(rtc::Error),
    /// `BLE_LibInit` failed, see `ffi::ERR_*`.
    LibInit(NonZeroU8),
    /// `TMOS_TimerInit` failed.
    TimerInit(NonZeroU8),
    /// `TMOS_ProcessEventRegister` failed.
    TaskRegister,
}

/// BLE configuration.
#[non_exhaustive]
pub struct Config {
    pub mac_addr: MacAddress,
    /// `ffi::LL_TX_POWEER_*`.
    pub tx_power: u8,
    /// Maximum connections as peripheral (0-3).
    pub peripheral_connections: u8,
    /// Maximum connections as central.
    pub central_connections: u8,
    /// The TMOS clock: the 32.768 kHz crystal (the default; WCH requires it
    /// for the central role), or the LSI, trimmed to 32768 Hz at init and
    /// every 2 minutes after.
    pub clock: ClockSource,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            mac_addr: MacAddress::factory(),
            tx_power: LL_TX_POWEER_0_DBM,
            peripheral_connections: 1,
            central_connections: 0,
            clock: ClockSource::Lse,
        }
    }
}

/// The RTC, held for the library.
static mut RTC: MaybeUninit<Rtc<'static>> = MaybeUninit::uninit();

/// Start the RTC clock and initialise the library and TMOS.
///
/// Call after [`crate::init`], then set up a GAP role (and spawn [`run`],
/// unless on [`crate::embassy::Executor`]).
/// Starting the crystal takes about 500 ms.
pub fn init(rtc: Peri<'static, peripherals::RTC>, config: Config) -> Result<(), Error> {
    unsafe { load_sections() };

    if !lib_version().starts_with(VER_FILE) {
        return Err(Error::LibVersion);
    }

    let mut rtc = Rtc::new(
        rtc,
        rtc::Config {
            clock_source: config.clock,
        },
    )
    .map_err(Error::Rtc)?;
    let lse = config.clock == ClockSource::Lse;
    if !lse {
        let _ = rtc.calibrate_lsi(CLOCK_HZ as u32);
    }
    unsafe { (*(&raw mut RTC)).write(rtc) };

    // 32 MHz crystal load capacitance, as the EVT (`HSECap_18p`).
    // `R8_XT32M_TUNE` is not in the metapac yet.
    #[cfg(any(ch584, ch585))]
    {
        const XT32M_TUNE: *mut u8 = 0x4000_104e as *mut u8;
        safe_access(|| unsafe { XT32M_TUNE.write_volatile((XT32M_TUNE.read_volatile() & 0x0f) | (4 << 4)) });
    }

    let mut cfg: bleConfig_t = unsafe { core::mem::zeroed() };
    cfg.MEMAddr = &raw mut BLE_HEAP as u32;
    cfg.MEMLen = HEAP_SIZE as u16;
    cfg.BufMaxLen = BLE_BUFF_MAX_LEN;
    cfg.BufNumber = BLE_BUFF_NUM;
    cfg.TxNumEvent = BLE_TX_NUM_EVENT;
    cfg.TxPower = config.tx_power;
    // No SNV (SNVAddr, readFlashCB, writeFlashCB): bonds are not kept.
    cfg.ConnectNumber = (config.peripheral_connections & 3) | (config.central_connections << 2);
    cfg.MacAddr = config.mac_addr.0;
    cfg.srandCB = Some(srand);
    // No temperature sampling (tsCB) or LSI calibration on temperature
    // changes (rcCB); the calibration task below runs every 2 minutes.
    #[cfg(feature = "executor")]
    {
        cfg.idleCB = Some(sleep::idle);
    }

    unsafe {
        irq::init();
        BLE_LibInit(&cfg).map_err(Error::LibInit)?;

        // HAL_Init: the HAL task, the timebase and the periodic calibration.
        let hal_task = TMOS_ProcessEventRegister(Some(hal_process_event));
        if hal_task == INVALID_TASK_ID {
            return Err(Error::TaskRegister);
        }
        HAL_LSI.store(!lse as u32, Ordering::Relaxed);

        let mut clock: bleClockConfig_t = core::mem::zeroed();
        clock.getClockValue = Some(get_clock_value);
        clock.SetPendingIRQ = Some(irq::pend_rtc);
        clock.ClockMaxCount = RTC_MAX_COUNT;
        clock.ClockFrequency = CLOCK_HZ;
        clock.ClockAccuracy = if lse { 50 } else { 1000 };
        TMOS_TimerInit(&clock).map_err(Error::TimerInit)?;

        // First calibration after 500 ms, as the EVT.
        tmos_start_task(hal_task, HAL_REG_INIT_EVENT, 800);
    }

    #[cfg(feature = "executor")]
    {
        sleep::init();
        sleep::STARTED.store(true, Ordering::Relaxed);
    }

    Ok(())
}

/// Copy `.highcode_lp` to RAM and zero the library's common symbols (see
/// `ble.x`).
unsafe fn load_sections() {
    unsafe extern "C" {
        static __ble_highcode_lp_lma: u32;
        static mut __ble_highcode_lp_start: u32;
        static mut __ble_highcode_lp_end: u32;
        static mut __ble_common_start: u32;
        static mut __ble_common_end: u32;
    }
    unsafe {
        let mut src = &raw const __ble_highcode_lp_lma;
        let mut dst = &raw mut __ble_highcode_lp_start;
        while dst < &raw mut __ble_highcode_lp_end {
            dst.write_volatile(src.read_volatile());
            src = src.add(1);
            dst = dst.add(1);
        }
        let mut p = &raw mut __ble_common_start;
        while p < &raw mut __ble_common_end {
            p.write_volatile(0);
            p = p.add(1);
        }
        // The copied code is fetched from RAM through the instruction path.
        core::arch::asm!("fence.i");
    }
}

/// Run TMOS forever, yielding to other tasks between passes. Spawn this in
/// an embassy task after [`init`] and the GAP role setup. Not with
/// [`crate::embassy::Executor`], which runs TMOS itself.
pub async fn run() -> ! {
    loop {
        unsafe { TMOS_SystemProcess() };
        count(&STATS.tmos_passes);
        YieldNow(false).await;
    }
}

/// Ready on the second poll, after waking itself on the first.
struct YieldNow(bool);

impl core::future::Future for YieldNow {
    type Output = ();
    fn poll(mut self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<()> {
        if self.0 {
            core::task::Poll::Ready(())
        } else {
            self.0 = true;
            cx.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    }
}

/// Library version, e.g. "CH585_BLE_LIB_V1.4".
pub fn lib_version() -> &'static str {
    unsafe { CStr::from_ptr(VER_LIB.as_ptr().cast()) }
        .to_str()
        .unwrap_or("?")
}

/// The TMOS clock: the RTC time of day, `CNT_2S:CNT_32K`, in 1/32768 s.
#[inline(always)]
pub fn clock() -> u32 {
    let p = (pac::RTC.as_ptr() as *const u8).wrapping_add(0x38) as *const u32;
    loop {
        let a = unsafe { p.read_volatile() };
        if a == unsafe { p.read_volatile() } {
            return a;
        }
    }
}

/// `getClockValue` for `TMOS_TimerInit`. Called from the radio interrupts.
#[unsafe(link_section = ".highcode.ble_get_clock_value")]
unsafe extern "C" fn get_clock_value() -> u32 {
    clock()
}

/// `srandCB`: random seed.
unsafe extern "C" fn srand() -> u32 {
    crate::delay::ticks() ^ clock()
}

/// Whether the RTC runs from the LSI (for the calibration task).
static HAL_LSI: AtomicU32 = AtomicU32::new(0);

/// The HAL TMOS task (`HAL_ProcessEvent` in the EVT).
unsafe extern "C" fn hal_process_event(task_id: tmosTaskID, events: tmosEvents) -> tmosEvents {
    unsafe {
        if events & SYS_EVENT_MSG != 0 {
            let msg = tmos_msg_receive(task_id);
            if !msg.is_null() {
                let _ = tmos_msg_deallocate(msg);
            }
            return events ^ SYS_EVENT_MSG;
        }
        if events & HAL_REG_INIT_EVENT != 0 {
            // Recalibrate the RF (under 10 ms).
            BLE_RegInit();
            if HAL_LSI.load(Ordering::Relaxed) != 0 {
                let _ = (*(&raw mut RTC)).assume_init_mut().calibrate_lsi(CLOCK_HZ as u32);
            } else {
                // The crystal's rated bias current, as the EVT.
                safe_access(|| pac::SYS.xt32k_tune().modify(|w| w.set_xt32k_i_tune(0b01)));
            }
            tmos_start_task(
                task_id,
                HAL_REG_INIT_EVENT,
                MS1_TO_SYSTEM_TIME(BLE_CALIBRATION_PERIOD_MS),
            );
            return events ^ HAL_REG_INIT_EVENT;
        }
    }
    0
}

/// MAC address in the library's order, LSB first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MacAddress(pub [u8; 6]);

impl MacAddress {
    /// The factory address, from the ROM configuration area
    /// (`ROM_CFG_MAC_ADDR`, which WCH's `GetMACAddress` reads).
    pub fn factory() -> Self {
        let p = 0x0007_F018 as *const u8;
        let mut a = [0u8; 6];
        for (i, b) in a.iter_mut().enumerate() {
            *b = unsafe { p.add(i).read_volatile() };
        }
        MacAddress(a)
    }

    /// A pseudo-random address, e.g. so a scanner does not show a name it
    /// cached for the factory address.
    ///
    /// Mixes the factory address with the RTC and SysTick counts; not
    /// cryptographic. The top two bits are set as for a static random
    /// address, though the library still uses it as its public address.
    pub fn random() -> Self {
        let f = Self::factory().0;
        let mut seed = u32::from_le_bytes([f[0], f[1], f[2], f[3]])
            ^ (u32::from(f[4]) << 8 | u32::from(f[5]))
            ^ clock()
            ^ crate::delay::ticks();
        let mut next = || {
            // splitmix32
            seed = seed.wrapping_add(0x9E37_79B9);
            let mut z = seed;
            z = (z ^ (z >> 16)).wrapping_mul(0x85EB_CA6B);
            z = (z ^ (z >> 13)).wrapping_mul(0xC2B2_AE35);
            z ^ (z >> 16)
        };
        let low = next().to_le_bytes();
        let high = next().to_le_bytes();
        MacAddress([low[0], low[1], low[2], low[3], high[0], high[1] | 0xC0])
    }

    /// From the usual written order, MSB first.
    pub const fn from_msb(addr: [u8; 6]) -> Self {
        MacAddress([addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]])
    }
}

impl core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let a = self.0;
        write!(
            f,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            a[5], a[4], a[3], a[2], a[1], a[0]
        )
    }
}
