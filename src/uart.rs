//! UART - Universal Asynchronous Receiver Transmitter
//!
//! The CH5xx UARTs are 16550-style: an 8-byte FIFO in each direction, a
//! divisor-latch baud generator (`DIV`, `DL`) and the usual `IER`/`IIR`/`LCR`/
//! `LSR` registers. There is no DMA, so the async API is interrupt driven,
//! moving up to a FIFO's worth of bytes per interrupt.
//!
//! The API follows ch32-hal's `usart` module: [`Uart`], [`UartTx`] and
//! [`UartRx`], `new_blocking`/`new` constructors with a `REMAP` const for the
//! pin mapping, and [`Config`].
//!
//! UART0 is the full UART (modem pins, half duplex, internal loopback);
//! UART1-3 are the "no modem" variant at the same register offsets.
//! Internal loopback ([`Uart::new_blocking_loopback`]) disconnects the
//! interrupt, so it only works blocking.
//!
//! # Pins (CH592)
//!
//! | UART  | REMAP = 0 (RX/TX) | REMAP = 1 (RX/TX) |
//! |-------|-------------------|-------------------|
//! | UART0 | PB4 / PB7         | PA15 / PA14       |
//! | UART1 | PA8 / PA9         | PB12 / PB13       |
//! | UART2 | PB22 / PB23       | PA6 / PA7         |
//! | UART3 | PA4 / PA5         | -                 |
//!
//! # Pins (CH584/CH585)
//!
//! | UART  | REMAP = 0 (RX/TX) | REMAP = 1 (RX/TX) |
//! |-------|-------------------|-------------------|
//! | UART0 | PB4 / PB7         | PA15 / PA14       |
//! | UART1 | PA8 / PA9         | PB12 / PB13       |
//! | UART2 | PA6 / PA7         | PB22 / PB23       |
//! | UART3 | PA4 / PA5         | PB20 / PB21       |

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use portable_atomic::AtomicBool;

use crate::gpio::{AfType, AnyPin, Pull, SealedPin as _, Speed};
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri};

/// FIFO depth in each direction.
const FIFO_SIZE: usize = 8;

/// `IIR` interrupt identification codes (`IIR & 0x0F`).
const IIR_LINE_STAT: u8 = 0x06;
const IIR_RECV_RDY: u8 = 0x04;
const IIR_RECV_TOUT: u8 = 0x0C;
const IIR_THR_EMPTY: u8 = 0x02;

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();

        // Reading IIR acknowledges a THR-empty interrupt. The others are
        // level conditions (FIFO level, line status, RX timeout): mask them
        // here and let the waiting future service the FIFO and re-enable.
        let iir = r.iir().read().0 & 0x0f;
        match iir {
            IIR_RECV_TOUT => {
                s.rx_idle.store(true, Ordering::Relaxed);
                r.ier().modify(|w| {
                    w.set_ier_recv_rdy(false);
                    w.set_ier_line_stat(false);
                });
            }
            IIR_RECV_RDY | IIR_LINE_STAT => r.ier().modify(|w| {
                w.set_ier_recv_rdy(false);
                w.set_ier_line_stat(false);
            }),
            IIR_THR_EMPTY => r.ier().modify(|w| w.set_ier_thr_empty(false)),
            _ => return,
        }

        compiler_fence(Ordering::SeqCst);
        s.rx_waker.wake();
        s.tx_waker.wake();
    }
}

/// Number of data bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataBits {
    DataBits5 = 0,
    DataBits6 = 1,
    DataBits7 = 2,
    DataBits8 = 3,
}

/// Parity.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
    /// Parity bit always 1.
    ParityMark,
    /// Parity bit always 0.
    ParitySpace,
}

/// Number of stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    #[doc = "2 stop bits"]
    STOP2,
}

/// UART configuration.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,

    /// If true: on a read-like method, if there is a latent error pending,
    /// the read will abort and the error will be reported and cleared
    ///
    /// If false: the error is ignored and cleared
    pub detect_previous_overrun: bool,
}

impl Default for Config {
    /// 115200 8N1
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
            detect_previous_overrun: false,
        }
    }
}

/// UART error.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    /// Break condition received
    Break,
}

/// Configuration error.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    BaudrateTooLow,
    BaudrateTooHigh,
    /// Loopback was requested on a UART that does not support it.
    LoopbackUnsupported,
}

// ========
// ## Tx Part

/// Tx-only UART Driver.
pub struct UartTx<'d, T: Instance, M: Mode> {
    _phantom: PhantomData<(&'d mut T, M)>,
    tx: Option<Peri<'d, AnyPin>>,
}

impl<'d, T: Instance, M: Mode> UartTx<'d, T, M> {
    /// Reconfigure the UART.
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(config)
    }

    fn new_inner(_peri: Peri<'d, T>, tx: Option<Peri<'d, AnyPin>>, config: Config) -> Result<Self, ConfigError> {
        T::enable();
        configure::<T>(&config)?;

        Ok(Self {
            _phantom: PhantomData,
            tx,
        })
    }

    /// Perform a blocking UART write
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let r = T::regs();
        for chunk in buffer.chunks(FIFO_SIZE) {
            while !r.lsr().read().lsr_tx_fifo_emp() {}
            for &c in chunk {
                r.thr().write(|w| w.set_thr(c));
            }
        }
        Ok(())
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        while !T::regs().lsr().read().lsr_tx_all_emp() {}
        Ok(())
    }
}

impl<'d, T: Instance> UartTx<'d, T, Async> {
    /// Create a new tx-only UART.
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        let this = Self::new_inner(peri, new_pin!(tx, AfType::output(Speed::High)), config)?;
        enable_interrupt::<T>();
        Ok(this)
    }

    /// Asynchronous UART write.
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        write_async::<T>(buffer).await
    }

    /// Wait until transmission complete.
    pub async fn flush(&mut self) -> Result<(), Error> {
        flush_async::<T>().await
    }
}

impl<'d, T: Instance> UartTx<'d, T, Blocking> {
    /// Create a new blocking tx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking<const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(tx, AfType::output(Speed::High)), config)
    }
}

impl<'d, T: Instance, M: Mode> Drop for UartTx<'d, T, M> {
    fn drop(&mut self) {
        if let Some(pin) = &self.tx {
            pin.set_as_disconnected();
        }
    }
}

// ========
// ## Rx Part

/// Rx-only UART Driver.
pub struct UartRx<'d, T: Instance, M: Mode> {
    _phantom: PhantomData<(&'d mut T, M)>,
    rx: Option<Peri<'d, AnyPin>>,
    detect_previous_overrun: bool,
}

impl<'d, T: Instance, M: Mode> UartRx<'d, T, M> {
    /// Reconfigure the UART.
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.detect_previous_overrun = config.detect_previous_overrun;
        reconfigure::<T>(config)
    }

    fn new_inner(_peri: Peri<'d, T>, rx: Option<Peri<'d, AnyPin>>, config: Config) -> Result<Self, ConfigError> {
        T::enable();
        configure::<T>(&config)?;

        Ok(Self {
            _phantom: PhantomData,
            rx,
            detect_previous_overrun: config.detect_previous_overrun,
        })
    }

    /// Read a single `u8` or return `WouldBlock`.
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let r = T::regs();
        let lsr = r.lsr().read();
        check_errors(lsr).map_err(nb::Error::Other)?;
        if lsr.lsr_data_rdy() {
            Ok(r.rbr().read().rbr())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Perform a blocking read into `buffer`.
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = T::regs();
        if !self.detect_previous_overrun {
            // Reading LSR clears the latched error flags.
            let _ = r.lsr().read();
        }
        for b in buffer {
            loop {
                let lsr = r.lsr().read();
                check_errors(lsr)?;
                if lsr.lsr_data_rdy() {
                    *b = r.rbr().read().rbr();
                    break;
                }
            }
        }
        Ok(())
    }
}

impl<'d, T: Instance> UartRx<'d, T, Async> {
    /// Create a new rx-only UART.
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        let this = Self::new_inner(peri, new_pin!(rx, AfType::input(Pull::Up)), config)?;
        enable_interrupt::<T>();
        Ok(this)
    }

    /// Asynchronously fill `buffer`.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        read_async::<T>(buffer, false, self.detect_previous_overrun).await.map(|_| ())
    }

    /// Read into `buffer` until it is full or the line goes idle (the RX FIFO
    /// timeout: about 4 character times with no new data) after at least one
    /// byte. Returns the number of bytes read.
    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        read_async::<T>(buffer, true, self.detect_previous_overrun).await
    }
}

impl<'d, T: Instance> UartRx<'d, T, Blocking> {
    /// Create a new rx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(rx, AfType::input(Pull::Up)), config)
    }
}

impl<'d, T: Instance, M: Mode> Drop for UartRx<'d, T, M> {
    fn drop(&mut self) {
        if let Some(pin) = &self.rx {
            pin.set_as_disconnected();
        }
    }
}

// ========
// ## Bidirectional

/// Bidirectional UART Driver, which acts as a combination of [`UartTx`] and [`UartRx`].
pub struct Uart<'d, T: Instance, M: Mode> {
    tx: UartTx<'d, T, M>,
    rx: UartRx<'d, T, M>,
}

impl<'d, T: Instance, M: Mode> Uart<'d, T, M> {
    /// Reconfigure the UART.
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.rx.set_config(config)
    }

    fn new_inner(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        tx: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable();
        configure::<T>(&config)?;

        Ok(Self {
            tx: UartTx {
                _phantom: PhantomData,
                tx,
            },
            rx: UartRx {
                _phantom: PhantomData,
                rx,
                detect_previous_overrun: config.detect_previous_overrun,
            },
        })
    }

    /// Perform a blocking write
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// Read a single `u8` or return `WouldBlock`
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the Uart into a transmitter and receiver, which is
    /// particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, T, M>, UartRx<'d, T, M>) {
        (self.tx, self.rx)
    }
}

impl<'d, T: Instance> Uart<'d, T, Async> {
    /// Create a new bidirectional UART.
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        let this = Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::Up)),
            new_pin!(tx, AfType::output(Speed::High)),
            config,
        )?;
        enable_interrupt::<T>();
        Ok(this)
    }

    /// Asynchronous write.
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.write(buffer).await
    }

    /// Wait until transmission complete.
    pub async fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush().await
    }

    /// Asynchronously fill `buffer`.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.read(buffer).await
    }

    /// See [`UartRx::read_until_idle`].
    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_until_idle(buffer).await
    }
}

impl<'d, T: Instance> Uart<'d, T, Blocking> {
    /// Create a new blocking bidirectional UART.
    pub fn new_blocking<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::Up)),
            new_pin!(tx, AfType::output(Speed::High)),
            config,
        )
    }

    /// Create a blocking UART that talks to itself through the internal
    /// loopback (UART0 only), without claiming any pins. For self-tests.
    ///
    /// Blocking only: loopback also loops `MCR.OUT2` back, which disconnects
    /// the UART's interrupt request (measured on CH592: `IIR` reports nothing
    /// and the PFIC never sees the interrupt).
    pub fn new_blocking_loopback(peri: Peri<'d, T>, config: Config) -> Result<Self, ConfigError> {
        let this = Self::new_inner(peri, None, None, config)?;
        set_loopback::<T>()?;
        Ok(this)
    }
}

// ========
// ## Shared implementation

/// Route TX to RX inside the chip (`MCR.LOOP`). The setting survives
/// [`Uart::set_config`], which does not touch `MCR`.
fn set_loopback<T: Instance>() -> Result<(), ConfigError> {
    if !T::HAS_LOOPBACK {
        return Err(ConfigError::LoopbackUnsupported);
    }
    T::regs().mcr().modify(|w| w.set_mcr_loop(true));
    Ok(())
}

fn enable_interrupt<T: Instance>() {
    // `MCR.OUT2` gates the UART's interrupt request to the PFIC.
    T::regs().mcr().modify(|w| w.set_mcr_int_oe(true));
    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };
}

fn reconfigure<T: Instance>(config: &Config) -> Result<(), ConfigError> {
    let irq = T::Interrupt::is_enabled();
    T::Interrupt::disable();
    let res = configure::<T>(config);
    if irq {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
    }
    res
}

fn configure<T: Instance>(config: &Config) -> Result<(), ConfigError> {
    let r = T::regs();

    // Baud rate = Fsys * 2 / DIV / 16 / DL. With DIV = 1 (WCH's choice, the
    // highest resolution) that is Fsys / 8 / DL, rounded to nearest.
    let fsys = T::frequency().0 as u64;
    let baud = config.baudrate as u64;
    if baud == 0 {
        return Err(ConfigError::BaudrateTooLow);
    }
    let dl = (fsys * 10 / 8 / baud + 5) / 10;
    if dl == 0 {
        return Err(ConfigError::BaudrateTooHigh);
    }
    if dl > u16::MAX as u64 {
        return Err(ConfigError::BaudrateTooLow);
    }

    // Quiesce: no interrupts, FIFOs flushed. `TXD_EN` (which connects the
    // transmitter to the TX pin) stays set: clearing it pulls the line low,
    // which the receiver latches as a break plus a NUL byte.
    r.ier().write(|w| w.set_ier_txd_en(true));

    r.div().write(|w| w.set_div(1));
    r.dl().write(|w| w.set_dl(dl as u16));
    r.fcr().write(|w| {
        // RX trigger at 4 bytes.
        w.set_fcr_fifo_trig(0b10);
        w.set_fcr_tx_fifo_clr(true);
        w.set_fcr_rx_fifo_clr(true);
        w.set_fcr_fifo_en(true);
    });
    r.lcr().write(|w| {
        w.set_lcr_word_sz(config.data_bits as u8);
        w.set_lcr_stop_bit(config.stop_bits == StopBits::STOP2);
        w.set_lcr_par_en(config.parity != Parity::ParityNone);
        w.set_lcr_par_mod(match config.parity {
            Parity::ParityOdd | Parity::ParityNone => 0,
            Parity::ParityEven => 1,
            Parity::ParityMark => 2,
            Parity::ParitySpace => 3,
        });
    });
    // Discard anything latched while reconfiguring.
    let _ = r.lsr().read();
    while r.lsr().read().lsr_data_rdy() {
        let _ = r.rbr().read();
    }

    Ok(())
}

fn check_errors(lsr: pac::uart::regs::Lsr) -> Result<(), Error> {
    if lsr.lsr_over_err() {
        Err(Error::Overrun)
    } else if lsr.lsr_par_err() {
        Err(Error::Parity)
    } else if lsr.lsr_frame_err() {
        Err(Error::Framing)
    } else if lsr.lsr_break_err() {
        Err(Error::Break)
    } else {
        Ok(())
    }
}

async fn write_async<T: Instance>(buffer: &[u8]) -> Result<(), Error> {
    let r = T::regs();
    let s = T::state();
    let mut rest = buffer;

    while !rest.is_empty() {
        poll_fn(|cx| {
            s.tx_waker.register(cx.waker());
            if r.lsr().read().lsr_tx_fifo_emp() {
                Poll::Ready(())
            } else {
                r.ier().modify(|w| w.set_ier_thr_empty(true));
                Poll::Pending
            }
        })
        .await;

        let n = rest.len().min(FIFO_SIZE);
        for &c in &rest[..n] {
            r.thr().write(|w| w.set_thr(c));
        }
        rest = &rest[n..];
    }
    Ok(())
}

async fn flush_async<T: Instance>() -> Result<(), Error> {
    let r = T::regs();
    let s = T::state();

    // Wait for the FIFO to drain, then for the last character to leave the
    // shift register (there is no interrupt for that: at most one character
    // time of spinning).
    poll_fn(|cx| {
        s.tx_waker.register(cx.waker());
        if r.lsr().read().lsr_tx_fifo_emp() {
            Poll::Ready(())
        } else {
            r.ier().modify(|w| w.set_ier_thr_empty(true));
            Poll::Pending
        }
    })
    .await;
    while !r.lsr().read().lsr_tx_all_emp() {}
    Ok(())
}

/// Fill `buffer`, or with `until_idle` stop early at an RX FIFO timeout.
///
/// The 16550 raises its timeout interrupt only while data sits in the FIFO,
/// so in `until_idle` mode each wakeup leaves the last byte behind: if the line
/// then goes quiet, that byte triggers the timeout that ends the read.
async fn read_async<T: Instance>(buffer: &mut [u8], until_idle: bool, detect_previous_overrun: bool) -> Result<usize, Error> {
    let r = T::regs();
    let s = T::state();

    if buffer.is_empty() {
        return Ok(0);
    }
    if !detect_previous_overrun {
        let _ = r.lsr().read();
    }
    s.rx_idle.store(false, Ordering::Relaxed);

    let _disable = crate::internal::drop::OnDrop::new(|| {
        r.ier().modify(|w| {
            w.set_ier_recv_rdy(false);
            w.set_ier_line_stat(false);
        })
    });

    let mut n = 0;
    poll_fn(|cx| {
        s.rx_waker.register(cx.waker());

        let idle = s.rx_idle.swap(false, Ordering::Relaxed);
        loop {
            let lsr = r.lsr().read();
            if let Err(e) = check_errors(lsr) {
                return Poll::Ready(Err(e));
            }
            if !lsr.lsr_data_rdy() {
                break;
            }
            // Keep one byte back for the timeout, unless the timeout already
            // fired or this byte completes the buffer.
            if until_idle && !idle && n + 1 < buffer.len() && r.rfc().read().rfc() <= 1 {
                break;
            }
            buffer[n] = r.rbr().read().rbr();
            n += 1;
            if n == buffer.len() {
                return Poll::Ready(Ok(n));
            }
        }
        if until_idle && idle && n > 0 {
            return Poll::Ready(Ok(n));
        }

        r.ier().modify(|w| {
            w.set_ier_recv_rdy(true);
            w.set_ier_line_stat(true);
        });
        Poll::Pending
    })
    .await
}

impl<'d, T: Instance> core::fmt::Write for UartTx<'d, T, Blocking> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

impl<'d, T: Instance> core::fmt::Write for Uart<'d, T, Blocking> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

// ========
// ## embedded-io

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::Framing => "Framing Error",
            Self::Overrun => "RX Buffer Overrun",
            Self::Parity => "Parity Check Error",
            Self::Break => "Break Received",
        };
        write!(f, "{}", message)
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<'d, T: Instance, M: Mode> embedded_io::ErrorType for Uart<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_io::ErrorType for UartTx<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_io::ErrorType for UartRx<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_io::Write for UartTx<'d, T, M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<'d, T: Instance, M: Mode> embedded_io::Write for Uart<'d, T, M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<'d, T: Instance> embedded_io_async::Write for UartTx<'d, T, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        UartTx::write(self, buf).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        UartTx::flush(self).await
    }
}

impl<'d, T: Instance> embedded_io_async::Write for Uart<'d, T, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Uart::write(self, buf).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Uart::flush(self).await
    }
}

impl<'d, T: Instance> embedded_io_async::Read for UartRx<'d, T, Async> {
    /// Returns once at least one byte has arrived and the line went idle, or
    /// `buf` is full.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_until_idle(buf).await
    }
}

impl<'d, T: Instance> embedded_io_async::Read for Uart<'d, T, Async> {
    /// Returns once at least one byte has arrived and the line went idle, or
    /// `buf` is full.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_until_idle(buf).await
    }
}

// ========
// ## embedded-hal-nb

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            Error::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Error::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Error::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Error::Break => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::ErrorType for Uart<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::Read for Uart<'d, T, M> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.nb_read()
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::Write for Uart<'d, T, M> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        let r = T::regs();
        if r.lsr().read().lsr_tx_fifo_emp() || r.tfc().read().tfc() < FIFO_SIZE as u8 {
            r.thr().write(|w| w.set_thr(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if T::regs().lsr().read().lsr_tx_all_emp() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

// ========
// ## Peripheral traits

struct State {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    /// Set by the interrupt handler on an RX FIFO timeout.
    rx_idle: AtomicBool,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            rx_idle: AtomicBool::new(false),
        }
    }
}

trait SealedInstance: crate::peripheral::RemapPeripheral {
    /// Internal loopback (`MCR.LOOP`) is implemented.
    const HAS_LOOPBACK: bool;

    /// Registers, as the "no modem" block every UART shares.
    fn regs() -> pac::uart::UartNomodem;
    fn state() -> &'static State;
    /// Ungate the peripheral clock.
    fn enable();

    fn frequency() -> Hertz {
        crate::rcc::clocks().sysclk
    }
}

/// UART instance.
#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);

macro_rules! impl_instance {
    ($inst:ident, loopback: $lb:expr, $($chip:tt)*) => {
        impl SealedInstance for peripherals::$inst {
            const HAS_LOOPBACK: bool = $lb;

            fn regs() -> pac::uart::UartNomodem {
                // UART0 is the full block on some chips; the offsets match.
                unsafe { pac::uart::UartNomodem::from_ptr(pac::$inst.as_ptr()) }
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }

            $($chip)*
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$inst;
        }
    };
}

#[cfg(ch592)]
mod ch592 {
    use super::*;
    use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};

    macro_rules! ch592_instance {
        ($inst:ident, $gate:ident, $remap:expr, loopback: $lb:expr) => {
            impl_instance!($inst, loopback: $lb,
                fn enable() {
                    set_clock_enabled(ClockGate::$gate, true);
                }
            );

            impl crate::peripheral::RemapPeripheral for peripherals::$inst {
                fn set_remap(remap: u8) {
                    let group: Option<RemapGroup> = $remap;
                    match group {
                        Some(g) => set_remap(g, remap),
                        None => assert!(remap == 0, "no remap for this UART"),
                    }
                }
            }
        };
    }

    ch592_instance!(UART0, Uart0, Some(RemapGroup::Uart0), loopback: true);
    ch592_instance!(UART1, Uart1, Some(RemapGroup::Uart1), loopback: false);
    ch592_instance!(UART2, Uart2, Some(RemapGroup::Uart2), loopback: false);
    ch592_instance!(UART3, Uart3, None, loopback: false);

    pin_trait_impl!(crate::uart::RxPin, UART0, PB4, 0);
    pin_trait_impl!(crate::uart::TxPin, UART0, PB7, 0);
    pin_trait_impl!(crate::uart::RxPin, UART0, PA15, 1);
    pin_trait_impl!(crate::uart::TxPin, UART0, PA14, 1);

    pin_trait_impl!(crate::uart::RxPin, UART1, PA8, 0);
    pin_trait_impl!(crate::uart::TxPin, UART1, PA9, 0);
    pin_trait_impl!(crate::uart::RxPin, UART1, PB12, 1);
    pin_trait_impl!(crate::uart::TxPin, UART1, PB13, 1);

    pin_trait_impl!(crate::uart::RxPin, UART2, PB22, 0);
    pin_trait_impl!(crate::uart::TxPin, UART2, PB23, 0);
    pin_trait_impl!(crate::uart::RxPin, UART2, PA6, 1);
    pin_trait_impl!(crate::uart::TxPin, UART2, PA7, 1);

    pin_trait_impl!(crate::uart::RxPin, UART3, PA4, 0);
    pin_trait_impl!(crate::uart::TxPin, UART3, PA5, 0);
}

#[cfg(any(ch584, ch585))]
mod ch585 {
    use super::*;
    use crate::sys::{set_clock_enabled, set_remap, ClockGate, RemapGroup};

    macro_rules! ch585_instance {
        ($inst:ident, $gate:ident, $remap:ident, loopback: $lb:expr) => {
            impl_instance!($inst, loopback: $lb,
                fn enable() {
                    set_clock_enabled(ClockGate::$gate, true);
                }
            );

            impl crate::peripheral::RemapPeripheral for peripherals::$inst {
                fn set_remap(remap: u8) {
                    set_remap(RemapGroup::$remap, remap)
                }
            }
        };
    }

    ch585_instance!(UART0, Uart0, Uart0, loopback: true);
    ch585_instance!(UART1, Uart1, Uart1, loopback: false);
    ch585_instance!(UART2, Uart2, Uart2, loopback: false);
    ch585_instance!(UART3, Uart3, Uart3, loopback: false);

    // CH585SFR.h, R16_PIN_ALTERNATE. Unlike CH592, UART2 defaults to PA6/PA7.
    pin_trait_impl!(crate::uart::RxPin, UART0, PB4, 0);
    pin_trait_impl!(crate::uart::TxPin, UART0, PB7, 0);
    pin_trait_impl!(crate::uart::RxPin, UART0, PA15, 1);
    pin_trait_impl!(crate::uart::TxPin, UART0, PA14, 1);

    pin_trait_impl!(crate::uart::RxPin, UART1, PA8, 0);
    pin_trait_impl!(crate::uart::TxPin, UART1, PA9, 0);
    pin_trait_impl!(crate::uart::RxPin, UART1, PB12, 1);
    pin_trait_impl!(crate::uart::TxPin, UART1, PB13, 1);

    pin_trait_impl!(crate::uart::RxPin, UART2, PA6, 0);
    pin_trait_impl!(crate::uart::TxPin, UART2, PA7, 0);
    pin_trait_impl!(crate::uart::RxPin, UART2, PB22, 1);
    pin_trait_impl!(crate::uart::TxPin, UART2, PB23, 1);

    pin_trait_impl!(crate::uart::RxPin, UART3, PA4, 0);
    pin_trait_impl!(crate::uart::TxPin, UART3, PA5, 0);
    pin_trait_impl!(crate::uart::RxPin, UART3, PB20, 1);
    pin_trait_impl!(crate::uart::TxPin, UART3, PB21, 1);
}
