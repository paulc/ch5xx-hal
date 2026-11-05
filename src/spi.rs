//! Serial Peripheral Interface (master)
//!
//! The CH5xx SPI0 differs from the CH32 SPI in ways that shape this driver:
//!
//! - The FIFO runs one way at a time (`FIFO_DIR`): out for writes, in for
//!   reads, with `TOTAL_CNT` bytes clocked automatically. Full-duplex
//!   transfers go byte by byte through `BUFFER`, which latches MISO whichever
//!   way the FIFO points (checked on CH592).
//! - The DMA is built in, but its address registers are 16 bits (17 on
//!   CH584/CH585): it reaches RAM only. Async writes from flash go through a
//!   small RAM bounce buffer.
//! - `TOTAL_CNT` holds 16 bits on CH592 and 12 on CH584/CH585, so longer
//!   transfers run in chunks.
//! - Only SPI modes 0 and 3 exist (`MST_SCK_MOD`), and only 8-bit words.
//! - SCK = HCLK / `CLOCK_DIV`, `CLOCK_DIV` >= 2.
//!
//! The API follows ch32-hal's `spi` module: [`Spi`] with `new_blocking`/`new`
//! (and `_rxonly`/`_txonly` variants) taking a `REMAP` const, [`Config`], and
//! the embedded-hal (async) `SpiBus` traits. Chip select is left to the user,
//! as in ch32-hal.
//!
//! # Pins (CH592, CH584/CH585)
//!
//! | REMAP | SCK  | MOSI | MISO |
//! |-------|------|------|------|
//! | 0     | PA13 | PA14 | PA15 |
//! | 1     | PB13 | PB14 | PB15 |
//!
//! REMAP 1 puts MOSI/MISO on PB14/PB15, the debug link's TIO/TCK: using it
//! takes the debugger off the chip. CH585 has a second, master-only SPI1
//! without DMA, which this driver does not cover; CH584 lacks it.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

use crate::gpio::{AfType, AnyPin, Pull, SealedPin as _, Speed};
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri};

/// FIFO depth.
const FIFO_SIZE: u8 = 8;

/// SPI Error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Overrun.
    Overrun,
}

/// Bit order.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BitOrder {
    LsbFirst,
    MsbFirst,
}

/// SPI configuration.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// [`MODE_0`] or [`MODE_3`]; the hardware has no modes 1 and 2.
    pub mode: Mode,
    pub bit_order: BitOrder,
    /// Requested SCK frequency; rounded down to HCLK / n, n >= 2.
    pub frequency: Hertz,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
            frequency: Hertz::hz(1_000_000),
        }
    }
}

/// Interrupt handler (async mode).
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        // Only `CNT_END` is ever enabled: the transfer is done.
        if r.int_flag().read().if_cnt_end() {
            r.inter_en().write(|w| w.0 = 0);
            compiler_fence(Ordering::SeqCst);
            T::state().waker.wake();
        }
    }
}

/// SPI driver.
pub struct Spi<'d, T: Instance, M: PeriMode> {
    _peri: Peri<'d, T>,
    sck: Option<Peri<'d, AnyPin>>,
    mosi: Option<Peri<'d, AnyPin>>,
    miso: Option<Peri<'d, AnyPin>>,
    current_config: Config,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance, M: PeriMode> Spi<'d, T, M> {
    fn new_inner(
        peri: Peri<'d, T>,
        sck: Option<Peri<'d, AnyPin>>,
        mosi: Option<Peri<'d, AnyPin>>,
        miso: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Self {
        T::enable();
        let r = T::regs();

        r.ctrl_mod().write(|w| w.set_all_clear(true));
        r.ctrl_mod().write(|w| {
            w.set_mode_slave(false);
            w.set_sck_oe(sck.is_some());
            w.set_mosi_oe(mosi.is_some());
            w.set_miso_oe(false);
        });
        r.ctrl_cfg().write(|w| {
            // BUFFER/FIFO accesses clear `IF_BYTE_END` automatically.
            w.set_auto_if(true);
            w.set_dma_enable(false);
        });
        r.inter_en().write(|w| w.0 = 0);

        let mut this = Self {
            _peri: peri,
            sck,
            mosi,
            miso,
            current_config: config,
            _phantom: PhantomData,
        };
        this.set_config(&config).expect("unsupported SPI mode");
        this
    }

    /// Reconfigure the SPI. Fails for SPI modes 1 and 2.
    pub fn set_config(&mut self, config: &Config) -> Result<(), ()> {
        let mode3 = match config.mode {
            MODE_0 => false,
            MODE_3 => true,
            _ => return Err(()),
        };
        let r = T::regs();
        let hclk = T::frequency().0;
        let div = hclk.div_ceil(config.frequency.0.max(1)).clamp(2, 255);

        r.clock_div().write(|w| w.set_clock_div(div as u8));
        r.ctrl_mod().modify(|w| w.set_mst_sck_mod(mode3));
        r.ctrl_cfg().modify(|w| {
            w.set_bit_order(config.bit_order == BitOrder::LsbFirst);
            // WCH sets the input delay at the fastest divider.
            w.set_mst_dly_en(div == 2);
        });
        self.current_config = *config;
        Ok(())
    }

    /// The configuration in use. `frequency` is the actual SCK rate.
    pub fn get_current_config(&self) -> Config {
        let div = T::regs().clock_div().read().clock_div() as u32;
        Config {
            frequency: Hertz(T::frequency().0 / div.max(2)),
            ..self.current_config
        }
    }

    /// Blocking write; received bits are discarded.
    pub fn blocking_write(&mut self, words: &[u8]) -> Result<(), Error> {
        let r = T::regs();
        for chunk in words.chunks(MAX_CNT) {
            r.ctrl_mod().modify(|w| w.set_fifo_dir(false));
            r.total_cnt().write(|w| w.set_total_cnt(chunk.len() as u16));
            r.int_flag().write(|w| w.set_if_cnt_end(true));
            for &b in chunk {
                while r.fifo_count().read().fifo_count() >= FIFO_SIZE {}
                r.fifo().write(|w| w.set_fifo(b));
            }
            // TOTAL_CNT reaches 0 when the last byte is clocked out.
            while r.total_cnt().read().total_cnt() != 0 {}
            while !r.int_flag().read().free() {}
        }
        Ok(())
    }

    /// Blocking read. What MOSI sends meanwhile is up to the hardware.
    pub fn blocking_read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        let r = T::regs();
        for chunk in words.chunks_mut(MAX_CNT) {
            r.ctrl_mod().modify(|w| w.set_fifo_dir(true));
            r.total_cnt().write(|w| w.set_total_cnt(chunk.len() as u16));
            r.int_flag().write(|w| w.set_if_cnt_end(true));
            for b in chunk {
                while r.fifo_count().read().fifo_count() == 0 {}
                *b = r.fifo().read().fifo();
            }
        }
        r.ctrl_mod().modify(|w| w.set_fifo_dir(false));
        Ok(())
    }

    /// Blocking in-place full-duplex transfer.
    pub fn blocking_transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
        let r = T::regs();
        r.ctrl_mod().modify(|w| w.set_fifo_dir(false));
        for b in words {
            *b = transfer_byte(r, *b);
        }
        Ok(())
    }

    /// Blocking full-duplex transfer. If the buffers differ in length, the
    /// shorter `write` is padded with `0x00` and extra reads are discarded.
    pub fn blocking_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let r = T::regs();
        r.ctrl_mod().modify(|w| w.set_fifo_dir(false));
        let len = read.len().max(write.len());
        for i in 0..len {
            let rx = transfer_byte(r, write.get(i).copied().unwrap_or(0));
            if let Some(b) = read.get_mut(i) {
                *b = rx;
            }
        }
        Ok(())
    }
}

#[inline]
fn transfer_byte(r: pac::spi::SpiFull, b: u8) -> u8 {
    r.buffer().write(|w| w.set_buffer(b));
    while !r.int_flag().read().free() {}
    r.buffer().read().buffer()
}

impl<'d, T: Instance> Spi<'d, T, Blocking> {
    /// Create a new SPI driver.
    pub fn new_blocking<const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T, REMAP>>,
        mosi: Peri<'d, impl MosiPin<T, REMAP>>,
        miso: Peri<'d, impl MisoPin<T, REMAP>>,
        config: Config,
    ) -> Self {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(Speed::High)),
            new_pin!(mosi, AfType::output(Speed::High)),
            new_pin!(miso, AfType::input(Pull::None)),
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_blocking_rxonly<const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T, REMAP>>,
        miso: Peri<'d, impl MisoPin<T, REMAP>>,
        config: Config,
    ) -> Self {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(Speed::High)),
            None,
            new_pin!(miso, AfType::input(Pull::None)),
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_blocking_txonly<const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T, REMAP>>,
        mosi: Peri<'d, impl MosiPin<T, REMAP>>,
        config: Config,
    ) -> Self {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(Speed::High)),
            new_pin!(mosi, AfType::output(Speed::High)),
            None,
            config,
        )
    }
}

impl<'d, T: Instance> Spi<'d, T, Async> {
    /// Create a new SPI driver.
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T, REMAP>>,
        mosi: Peri<'d, impl MosiPin<T, REMAP>>,
        miso: Peri<'d, impl MisoPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        apply_remap!();
        let this = Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(Speed::High)),
            new_pin!(mosi, AfType::output(Speed::High)),
            new_pin!(miso, AfType::input(Pull::None)),
            config,
        );
        enable_interrupt::<T>();
        this
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_rxonly<const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T, REMAP>>,
        miso: Peri<'d, impl MisoPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        apply_remap!();
        let this = Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(Speed::High)),
            None,
            new_pin!(miso, AfType::input(Pull::None)),
            config,
        );
        enable_interrupt::<T>();
        this
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_txonly<const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, impl SckPin<T, REMAP>>,
        mosi: Peri<'d, impl MosiPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        apply_remap!();
        let this = Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(Speed::High)),
            new_pin!(mosi, AfType::output(Speed::High)),
            None,
            config,
        );
        enable_interrupt::<T>();
        this
    }

    /// DMA write. `data` outside RAM is sent through a bounce buffer.
    pub async fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        if in_dma_ram(data.as_ptr(), data.len()) {
            for chunk in data.chunks(MAX_CNT) {
                dma::<T>(chunk.as_ptr() as u32, chunk.len(), false).await;
            }
        } else {
            let mut bounce = [0u8; 64];
            for chunk in data.chunks(bounce.len()) {
                bounce[..chunk.len()].copy_from_slice(chunk);
                dma::<T>(bounce.as_ptr() as u32, chunk.len(), false).await;
            }
        }
        Ok(())
    }

    /// DMA read. What MOSI sends meanwhile is up to the hardware.
    pub async fn read(&mut self, data: &mut [u8]) -> Result<(), Error> {
        // `&mut` data is always in RAM.
        for chunk in data.chunks_mut(MAX_CNT) {
            dma::<T>(chunk.as_mut_ptr() as u32, chunk.len(), true).await;
        }
        T::regs().ctrl_mod().modify(|w| w.set_fifo_dir(false));
        Ok(())
    }

    /// Full-duplex transfer. The FIFO runs one way at a time, so this is the
    /// blocking byte-by-byte transfer; it does not yield.
    pub async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        self.blocking_transfer(read, write)
    }

    /// In-place full-duplex transfer; see [`Self::transfer`].
    pub async fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.blocking_transfer_in_place(data)
    }
}

/// Bytes one `TOTAL_CNT` run moves.
#[cfg(spi_ch592)]
const MAX_CNT: usize = u16::MAX as usize;
/// Bytes one `TOTAL_CNT` run moves: WCH's driver allows up to 4095.
#[cfg(spi_ch585)]
const MAX_CNT: usize = 0xfff;

/// The addresses the DMA reaches: its address registers are 16 bits on
/// CH592, 17 on CH584/CH585, offsets into the RAM at 0x2000_0000.
#[cfg(spi_ch592)]
const DMA_WINDOW: core::ops::Range<usize> = 0x2000_0000..0x2001_0000;
#[cfg(spi_ch585)]
const DMA_WINDOW: core::ops::Range<usize> = 0x2000_0000..0x2002_0000;

/// Can the SPI DMA reach `[ptr, ptr + len)`?
fn in_dma_ram(ptr: *const u8, len: usize) -> bool {
    let start = ptr as usize;
    start >= DMA_WINDOW.start && start + len <= DMA_WINDOW.end
}

/// Run one DMA transfer of `len` bytes at RAM address `addr`, `rx` = into RAM.
async fn dma<T: Instance>(addr: u32, len: usize, rx: bool) {
    let r = T::regs();
    let s = T::state();

    r.ctrl_cfg().modify(|w| w.set_dma_enable(false));
    r.ctrl_mod().modify(|w| w.set_fifo_dir(rx));
    r.dma_beg().write(|w| w.set_dma_beg(addr as _));
    r.dma_end().write(|w| w.set_dma_end((addr as usize + len) as _));
    // Count first: `IF_CNT_END` is set again at once while the count is 0.
    r.total_cnt().write(|w| w.set_total_cnt(len as u16));
    r.int_flag().write(|w| {
        w.set_if_cnt_end(true);
        w.set_if_dma_end(true);
    });
    compiler_fence(Ordering::SeqCst);
    r.inter_en().write(|w| w.set_ie_cnt_end(true));
    r.ctrl_cfg().modify(|w| w.set_dma_enable(true));

    // Stop the DMA if the future is dropped mid-transfer.
    let _guard = crate::internal::drop::OnDrop::new(|| {
        r.inter_en().write(|w| w.0 = 0);
        r.ctrl_cfg().modify(|w| w.set_dma_enable(false));
        r.total_cnt().write(|w| w.set_total_cnt(0));
    });

    poll_fn(|cx| {
        s.waker.register(cx.waker());
        if r.int_flag().read().if_cnt_end() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;

    if !rx {
        // The last byte is still shifting out when the count ends.
        while !r.int_flag().read().free() {}
    }
    compiler_fence(Ordering::SeqCst);
}

fn enable_interrupt<T: Instance>() {
    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };
}

impl<'d, T: Instance, M: PeriMode> Drop for Spi<'d, T, M> {
    fn drop(&mut self) {
        let r = T::regs();
        r.inter_en().write(|w| w.0 = 0);
        r.ctrl_cfg().modify(|w| w.set_dma_enable(false));
        r.ctrl_mod().write(|w| w.set_all_clear(true));
        for pin in [&self.sck, &self.mosi, &self.miso].into_iter().flatten() {
            pin.set_as_disconnected();
        }
    }
}

// ========
// ## embedded-hal

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match *self {
            Self::Overrun => embedded_hal::spi::ErrorKind::Overrun,
        }
    }
}

impl<'d, T: Instance, M: PeriMode> embedded_hal::spi::ErrorType for Spi<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: PeriMode> embedded_hal::spi::SpiBus<u8> for Spi<'d, T, M> {
    fn flush(&mut self) -> Result<(), Self::Error> {
        while !T::regs().int_flag().read().free() {}
        Ok(())
    }

    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(words)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(words)
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_transfer(read, write)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_transfer_in_place(words)
    }
}

impl<'d, T: Instance> embedded_hal_async::spi::SpiBus<u8> for Spi<'d, T, Async> {
    async fn flush(&mut self) -> Result<(), Self::Error> {
        while !T::regs().int_flag().read().free() {}
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write(words).await
    }

    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.read(words).await
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer_in_place(words).await
    }
}

// ========
// ## Peripheral traits

struct State {
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedInstance: crate::sys::RemapPeripheral {
    fn regs() -> pac::spi::SpiFull;
    fn state() -> &'static State;
    /// Ungate the peripheral clock.
    fn enable();

    fn frequency() -> Hertz {
        crate::rcc::clocks().sysclk
    }
}

/// SPI instance.
#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(SckPin, Instance);
pin_trait!(MosiPin, Instance);
pin_trait!(MisoPin, Instance);

/// SPI0: the same pins and remap on CH592 and CH584/CH585.
mod spi0 {
    use super::*;
    use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};

    impl SealedInstance for peripherals::SPI0 {
        fn regs() -> pac::spi::SpiFull {
            pac::SPI0
        }

        fn state() -> &'static State {
            static STATE: State = State::new();
            &STATE
        }

        fn enable() {
            set_clock_enabled(ClockGate::Spi0, true);
        }
    }

    impl Instance for peripherals::SPI0 {
        type Interrupt = crate::interrupt::typelevel::SPI0;
    }

    impl crate::sys::RemapPeripheral for peripherals::SPI0 {
        fn set_remap(remap: u8) {
            set_remap(RemapGroup::Spi0, remap)
        }
    }

    pin_trait_impl!(crate::spi::SckPin, SPI0, PA13, 0);
    pin_trait_impl!(crate::spi::MosiPin, SPI0, PA14, 0);
    pin_trait_impl!(crate::spi::MisoPin, SPI0, PA15, 0);
    pin_trait_impl!(crate::spi::SckPin, SPI0, PB13, 1);
    pin_trait_impl!(crate::spi::MosiPin, SPI0, PB14, 1);
    pin_trait_impl!(crate::spi::MisoPin, SPI0, PB15, 1);
}
