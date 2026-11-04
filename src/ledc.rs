//! LED screen controller (LEDC), CH584/CH585
//!
//! A clock (LEDC) and 1, 2, 4 or 8 data lines (LED0-LED7) shifting out a
//! buffer, for LED dot-matrix panels built from shift registers. Data comes
//! from a 2-word FIFO, written by the CPU ([`Ledc::blocking_write`]) or by the
//! block's own DMA ([`Ledc::write`]).
//!
//! # Data format
//!
//! The data are a byte stream, little-endian within each `u32` word: byte 0
//! is bits 0-7 of `words[0]`. With N lanes the stream goes out in groups of N
//! bytes, all lanes together: lane k sends byte k of each group over 8 bit
//! times, MSB or LSB first ([`Config::bit_order`]). So one word lasts 32, 16,
//! 8 or 4 bit times with 1, 2, 4 or 8 lanes.
//!
//! With 8 lanes the block consumes the FIFO in word pairs: an odd word count
//! is rejected.
//!
//! # Timing
//!
//! A bit time is HCLK / `R8_LED_CLOCK_DIV` ([`Config::frequency`]), and the
//! clock pin runs one period per bit: it idles high, the data change on its
//! falling edge and are stable at its rising edge, as in SPI mode 3. The
//! clock pauses (high) whenever the FIFO runs dry, so a writer that falls
//! behind stretches the output but does not corrupt it.
//!
//! # Checked on a CH584 (WeAct CH584F)
//!
//! The datasheet's chapter on this block is short, and in places wrong. The
//! behaviour above was measured by sampling the pins; the CH584F bonds only
//! LEDC (PA4) and LED4 (PA5), so the lane mapping was seen on lane 4 in
//! 8-lane mode and the rest follows from the FIFO count:
//!
//! - `RB_LED_BIT_ORDER` = 1 sends the MSB first, 0 the LSB first (the
//!   datasheet and `CH585SFR.h` say the opposite).
//! - Mode 3 (`RB_LED_CHAN_MOD` = 11) drives all 8 lanes, as the datasheet
//!   says; the WCH EVT's "LED0/1 plus auxiliary buffer" description is wrong.
//! - `R16_LED_DMA_LEN` counts 32-bit words, not 16-bit ones. A length of 0
//!   never completes.
//! - Divider values 2-254 give exactly that many HCLK cycles per bit. 255
//!   is odd: one clock period of 255 HCLK carries two bits, the data changing
//!   on both edges. 0 gives 128 HCLK per bit, and 1 stalls the block. The
//!   driver keeps to 2-254.
//! - `RB_LED_IF_DMA_END` is set when the DMA has fetched the last word,
//!   `RB_LED_IF_DMA_INT` when the last bit is out. `RB_LED_DMA_EN` raises the
//!   interrupt on the first, `RB_IE_SEND_END` on the second.
//! - `RB_LED_OUT_EN` gates the whole block, not only the pins.
//!
//! # Pins
//!
//! | LEDC | LED0 | LED1 | LED2 | LED3 | LED4 | LED5 | LED6 | LED7 |
//! |------|------|------|------|------|------|------|------|------|
//! | PA4  | PA0  | PA1  | PA2  | PA3  | PA5  | PA6  | PA7  | PA8  |
//!
//! There is no remap. Lanes left without a pin still shift, unseen.
//!
//! WCH lists the LED screen interface for the QFN48 parts (CH58xM/X) only.
//! The QFN32 CH584F/CH585F have the same block but bond only LEDC and LED4,
//! so one lane, in 8-lane mode; that works on a CH584F (`examples/ch584`'s
//! `ledc` test), but WCH does not support it.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{AfType, AnyPin, SealedPin as _, Speed};
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri};

/// FIFO depth in bytes.
const FIFO_SIZE: u8 = 8;

/// Words one DMA run moves (`R16_LED_DMA_LEN` is 12 bits). Even, for 8 lanes.
const MAX_LEN: usize = 0xffe;

/// The addresses the DMA reaches: `R32_LED_DMA_BEG` holds bits 16:2 of an
/// offset into the RAM at 0x2000_0000.
const DMA_WINDOW: core::ops::Range<usize> = 0x2000_0000..0x2002_0000;

/// Number of data lanes (`RB_LED_CHAN_MOD`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Lanes {
    /// LED0.
    One,
    /// LED0-LED1.
    Two,
    /// LED0-LED3.
    Four,
    /// LED0-LED7.
    Eight,
}

impl Lanes {
    /// The number of lanes.
    pub const fn count(self) -> usize {
        match self {
            Self::One => 1,
            Self::Two => 2,
            Self::Four => 4,
            Self::Eight => 8,
        }
    }

    fn chan_mod(self) -> u8 {
        self as u8
    }
}

/// Bit order within each lane's byte.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    LsbFirst,
    MsbFirst,
}

/// LEDC configuration.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    pub lanes: Lanes,
    pub bit_order: BitOrder,
    /// Requested bit rate per lane, which is also the clock pin's frequency;
    /// rounded down to HCLK / n, 2 <= n <= 254.
    pub frequency: Hertz,
    /// Invert the data lines (`RB_LED_OUT_POLAR`). The clock is not inverted.
    pub invert: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            lanes: Lanes::One,
            bit_order: BitOrder::MsbFirst,
            frequency: Hertz::hz(1_000_000),
            invert: false,
        }
    }
}

/// The data lines, by lane: `DataPins::new().lane(p.PA0).lane(p.PA1)`.
pub struct DataPins<'d> {
    pins: [Option<Peri<'d, AnyPin>>; 8],
}

impl<'d> DataPins<'d> {
    /// No data lines yet.
    pub fn new() -> Self {
        Self {
            pins: [const { None }; 8],
        }
    }

    /// Add the pin for lane `L` (`LED<L>`).
    pub fn lane<const L: u8>(mut self, pin: Peri<'d, impl DataPin<L>>) -> Self {
        self.pins[L as usize] = Some(pin.into());
        self
    }

    /// The highest lane with a pin, plus one.
    fn used(&self) -> usize {
        self.pins.iter().rposition(Option::is_some).map_or(0, |i| i + 1)
    }
}

impl<'d> Default for DataPins<'d> {
    fn default() -> Self {
        Self::new()
    }
}

/// Interrupt handler (async mode).
pub struct InterruptHandler {
    _private: (),
}

impl interrupt::typelevel::Handler<interrupt::typelevel::LED> for InterruptHandler {
    unsafe fn on_interrupt() {
        let r = regs();
        let s = r.status().read();
        // `DMA_EN` interrupts when the DMA has fetched the last word, which
        // only matters as a flag to clear; the transfer ends at `IF_DMA_INT`.
        if s.if_dma_end() {
            r.status().write(|w| w.set_if_dma_end(true));
        }
        if s.if_dma_int() {
            r.ctrl_mod1().modify(|w| w.set_ie_send_end(false));
            compiler_fence(Ordering::SeqCst);
            STATE.waker.wake();
        }
    }
}

struct State {
    waker: AtomicWaker,
}

static STATE: State = State {
    waker: AtomicWaker::new(),
};

fn regs() -> pac::led::Led {
    pac::LED
}

/// LEDC driver.
pub struct Ledc<'d, M: PeriMode> {
    _peri: Peri<'d, peripherals::LED>,
    clk: Peri<'d, AnyPin>,
    data: DataPins<'d>,
    config: Config,
    _phantom: PhantomData<M>,
}

impl<'d, M: PeriMode> Ledc<'d, M> {
    fn new_inner(
        peri: Peri<'d, peripherals::LED>,
        clk: Peri<'d, impl ClkPin>,
        data: DataPins<'d>,
        config: Config,
    ) -> Self {
        crate::sys::set_clock_enabled(crate::sys::ClockGate::Led, true);
        let r = regs();
        r.ctrl_mod().write(|w| w.set_all_clear(true));
        r.ctrl_mod().write(|w| w.0 = 0);
        r.ctrl_mod1().write(|w| w.0 = 0);
        clear_flags();

        clk.set_as_af(AfType::output(Speed::High));
        for pin in data.pins.iter().flatten() {
            pin.set_as_af(AfType::output(Speed::High));
        }

        let mut this = Self {
            _peri: peri,
            clk: clk.into(),
            data,
            config,
            _phantom: PhantomData,
        };
        this.set_config(&config);
        this
    }

    /// Reconfigure. Panics if a data pin is on a lane `config.lanes` leaves
    /// out.
    pub fn set_config(&mut self, config: &Config) {
        assert!(
            self.data.used() <= config.lanes.count(),
            "LEDC data pin on a lane beyond config.lanes"
        );
        let r = regs();
        let div = kernel_clock().0.div_ceil(config.frequency.0.max(1)).clamp(2, 254);
        r.clock_div().write(|w| w.set_clock_div(div as u8));
        r.ctrl_mod().write(|w| {
            w.set_chan_mod(config.lanes.chan_mod());
            w.set_bit_order(config.bit_order == BitOrder::MsbFirst);
            w.set_out_polar(config.invert);
            w.set_out_en(true);
        });
        self.config = *config;
    }

    /// The configuration in use. `frequency` is the actual bit rate.
    pub fn get_current_config(&self) -> Config {
        let div = regs().clock_div().read().clock_div() as u32;
        Config {
            frequency: Hertz(kernel_clock().0 / div.max(2)),
            ..self.config
        }
    }

    /// Is a transfer still shifting out?
    pub fn is_busy(&self) -> bool {
        let s = regs().status().read();
        s.fifo_count() != 0 || !s.load_fail()
    }

    /// Blocking write through the FIFO. Returns once the last bit is out.
    ///
    /// With 8 lanes at high bit rates the CPU cannot keep the FIFO fed: the
    /// clock then pauses between words.
    pub fn blocking_write(&mut self, words: &[u32]) {
        self.check_len(words.len());
        let r = regs();
        for &w in words {
            while r.status().read().fifo_count() > FIFO_SIZE - 4 {}
            r.fifo().write(|f| f.set_fifo(w));
        }
        self.blocking_flush();
    }

    /// Wait until the last bit is out.
    pub fn blocking_flush(&mut self) {
        let r = regs();
        // `LOAD_FAIL` stays set for a moment after a FIFO write, until the
        // shifter takes the data: wait for the FIFO to drain first.
        while r.status().read().fifo_count() != 0 {}
        while !r.status().read().load_fail() {}
    }

    fn check_len(&self, len: usize) {
        assert!(
            self.config.lanes != Lanes::Eight || len % 2 == 0,
            "LEDC with 8 lanes needs an even number of words"
        );
    }
}

/// The clock the bit rate divides: HCLK.
fn kernel_clock() -> Hertz {
    crate::rcc::clocks().sysclk
}

impl<'d> Ledc<'d, Blocking> {
    /// Create a new LEDC driver.
    pub fn new_blocking(
        peri: Peri<'d, peripherals::LED>,
        clk: Peri<'d, impl ClkPin>,
        data: DataPins<'d>,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, clk, data, config)
    }
}

impl<'d> Ledc<'d, Async> {
    /// Create a new LEDC driver.
    pub fn new(
        peri: Peri<'d, peripherals::LED>,
        clk: Peri<'d, impl ClkPin>,
        data: DataPins<'d>,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::LED, InterruptHandler> + 'd,
        config: Config,
    ) -> Self {
        let this = Self::new_inner(peri, clk, data, config);
        interrupt::typelevel::LED::unpend();
        unsafe { interrupt::typelevel::LED::enable() };
        this
    }

    /// DMA write. Returns once the last bit is out. `words` outside RAM are
    /// sent through a bounce buffer.
    pub async fn write(&mut self, words: &[u32]) {
        self.check_len(words.len());
        let start = words.as_ptr() as usize;
        if start >= DMA_WINDOW.start && start + words.len() * 4 <= DMA_WINDOW.end {
            for chunk in words.chunks(MAX_LEN) {
                dma(chunk.as_ptr() as u32, chunk.len()).await;
            }
        } else {
            let mut bounce = [0u32; 16];
            for chunk in words.chunks(bounce.len()) {
                bounce[..chunk.len()].copy_from_slice(chunk);
                dma(bounce.as_ptr() as u32, chunk.len()).await;
            }
        }
    }
}

fn clear_flags() {
    regs().status().write(|w| {
        w.set_if_fifo(true);
        w.set_if_dma_end(true);
        w.set_if_dma_int(true);
    });
}

/// Run one DMA transfer of `len` words at RAM address `addr`.
async fn dma(addr: u32, len: usize) {
    if len == 0 {
        // A zero length never completes.
        return;
    }
    let r = regs();

    clear_flags();
    r.dma_beg().write(|w| w.set_dma_beg(((addr & 0x1_ffff) >> 2) as u16));
    r.dma_len().write(|w| w.set_dma_length(len as u16));
    compiler_fence(Ordering::SeqCst);
    r.ctrl_mod1().modify(|w| w.set_ie_send_end(true));
    r.ctrl_mod().modify(|w| w.set_dma_en(true));

    // Stop if the future is dropped mid-transfer: `ALL_CLEAR` empties the
    // FIFO and halts the shifter, where clearing `DMA_EN` alone would let the
    // FIFO drain.
    let _guard = crate::internal::drop::OnDrop::new(|| {
        r.ctrl_mod1().modify(|w| w.set_ie_send_end(false));
        abort();
    });

    poll_fn(|cx| {
        STATE.waker.register(cx.waker());
        if r.status().read().if_dma_int() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;

    _guard.defuse();
    r.ctrl_mod().modify(|w| w.set_dma_en(false));
    clear_flags();
    compiler_fence(Ordering::SeqCst);
}

/// Stop the output: empty the FIFO and halt the shifter.
fn abort() {
    let r = regs();
    r.ctrl_mod().modify(|w| {
        w.set_dma_en(false);
        w.set_all_clear(true);
    });
    r.ctrl_mod().modify(|w| w.set_all_clear(false));
    // The shifter stops at the end of the bit in progress, at most 254 HCLK
    // away, with the clock back high: wait for that, but not forever.
    for _ in 0..256 {
        if r.status().read().load_fail() {
            break;
        }
    }
    clear_flags();
}

impl<'d, M: PeriMode> Drop for Ledc<'d, M> {
    fn drop(&mut self) {
        let r = regs();
        r.ctrl_mod1().write(|w| w.0 = 0);
        abort();
        r.ctrl_mod().write(|w| w.0 = 0);
        self.clk.set_as_disconnected();
        for pin in self.data.pins.iter().flatten() {
            pin.set_as_disconnected();
        }
    }
}

// ========
// ## Pin traits

/// The LEDC clock pin.
pub trait ClkPin: crate::gpio::Pin {}

/// The data pin for lane `LANE` (`LED<LANE>`).
pub trait DataPin<const LANE: u8>: crate::gpio::Pin {}

impl ClkPin for peripherals::PA4 {}
impl DataPin<0> for peripherals::PA0 {}
impl DataPin<1> for peripherals::PA1 {}
impl DataPin<2> for peripherals::PA2 {}
impl DataPin<3> for peripherals::PA3 {}
impl DataPin<4> for peripherals::PA5 {}
impl DataPin<5> for peripherals::PA6 {}
impl DataPin<6> for peripherals::PA7 {}
impl DataPin<7> for peripherals::PA8 {}
