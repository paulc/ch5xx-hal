//! Inter-Integrated-Circuit (I2C), master
//!
//! The CH5xx I2C is the same STM32F1-style block as the CH32's, so this is
//! ch32-hal's driver with the CH5xx differences:
//!
//! - No DMA: the async methods move one byte per interrupt.
//! - One interrupt (`I2C0`) for events and errors, so one
//!   [`InterruptHandler`].
//! - No open-drain GPIO mode: the pins are pull-up inputs and the
//!   peripheral drives them open-drain itself.
//! - [`Config::timeout`] is a `core::time::Duration` counted on SysTick, so
//!   the blocking methods time out without the `embassy` feature.
//!
//! # Pins (CH592)
//!
//! SCL PB15 and SDA PB14, the debug link's TCK and TIO. With the debug
//! interface enabled in the user config the pins belong to it, so creating the
//! driver turns the interface off ([`crate::debug::set_debug_interface_enabled`])
//! and dropping it turns it back on. That takes the debugger (and SDI
//! print) off the chip, and needs a part whose lot number supports it; on
//! older parts disable `CFG_DEBUG_EN` with the ISP tool instead.
//!
//! # Pins (CH584/CH585)
//!
//! | REMAP | SCL  | SDA  |
//! |-------|------|------|
//! | 0     | PB13 | PB12 |
//! | 1     | PB21 | PB20 |
//!
//! CH584F/CH585F bond neither PB20 nor PB21.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::i2c::Operation;

use crate::gpio::{AfType, AnyPin, Pull};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::pac::i2c::regs::Star1;
use crate::sys::{set_clock_enabled, ClockGate};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri};

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Wake the task on every interrupt and mask them until it re-arms.
        T::state().waker.wake();
        critical_section::with(|_| {
            T::regs().ctrl2().modify(|w| {
                w.set_itevten(false);
                w.set_iterren(false);
                w.set_itbufen(false);
            });
        });
    }
}

/// I2C error.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error. (BERR)
    Bus,
    /// Arbitration lost (ARLO)
    Arbitration,
    /// ACK not received (either to the address or to a data byte) (AF)
    Nack,
    /// Timeout
    Timeout,
    /// CRC error
    Crc,
    /// Overrun error (OVR)
    Overrun,
    /// Zero-length transfers are not allowed.
    ZeroLengthTransfer,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for Error {}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Duty {
    Duty2_1 = 0,
    Duty16_9 = 1,
}

/// I2C config
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// Timeout of each blocking operation.
    pub timeout: core::time::Duration,
    pub duty: Duty,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            timeout: core::time::Duration::from_millis(1000),
            duty: Duty::Duty2_1,
        }
    }
}

/// A deadline on the SysTick counter.
#[derive(Clone, Copy)]
struct Timeout {
    start: u32,
    ticks: u32,
}

impl Timeout {
    fn new(d: core::time::Duration) -> Self {
        let hz = crate::delay::ticks_per_sec() as u64;
        // Wrapping comparisons need the span below 2^31 ticks.
        let ticks = (d.as_micros() as u64 * hz / 1_000_000).min(i32::MAX as u64) as u32;
        Self {
            start: crate::delay::ticks(),
            ticks,
        }
    }

    fn check(self) -> Result<(), Error> {
        if crate::delay::ticks().wrapping_sub(self.start) > self.ticks {
            Err(Error::Timeout)
        } else {
            Ok(())
        }
    }
}

/// I2C driver.
pub struct I2c<'d, T: Instance, M: Mode> {
    _peri: Peri<'d, T>,
    _scl: Peri<'d, AnyPin>,
    _sda: Peri<'d, AnyPin>,
    timeout: core::time::Duration,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance> I2c<'d, T, Async> {
    /// Create a new I2C driver.
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        freq: Hertz,
        config: Config,
    ) -> Self {
        apply_remap!();
        let this = Self::new_inner(peri, scl.into(), sda.into(), freq, config);
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        this
    }
}

impl<'d, T: Instance> I2c<'d, T, Blocking> {
    /// Create a new blocking I2C driver.
    pub fn new_blocking<const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        freq: Hertz,
        config: Config,
    ) -> Self {
        apply_remap!();
        Self::new_inner(peri, scl.into(), sda.into(), freq, config)
    }
}

impl<'d, T: Instance, M: Mode> I2c<'d, T, M> {
    fn new_inner(peri: Peri<'d, T>, scl: Peri<'d, AnyPin>, sda: Peri<'d, AnyPin>, freq: Hertz, config: Config) -> Self {
        T::enable();
        #[cfg(ch592)]
        crate::debug::set_debug_interface_enabled(false);

        // The peripheral switches its pins to open-drain output when it
        // drives them.
        let af = AfType::input(Pull::Up);
        crate::gpio::SealedPin::set_as_af(&*scl, af);
        crate::gpio::SealedPin::set_as_af(&*sda, af);

        let mut this = Self {
            _peri: peri,
            _scl: scl,
            _sda: sda,
            timeout: config.timeout,
            _phantom: PhantomData,
        };

        this.init(freq, config);

        this
    }

    fn timeout(&self) -> Timeout {
        Timeout::new(self.timeout)
    }

    // init as master mode
    fn init(&mut self, freq: Hertz, config: Config) {
        let regs = T::regs();

        regs.ctrl1().modify(|w| w.set_pe(false)); // disable i2c

        // soft reset
        regs.ctrl1().modify(|w| w.set_swrst(true));
        regs.ctrl1().modify(|w| w.set_swrst(false));

        let freq_in = crate::rcc::clocks().sysclk.0;
        // The datasheet allows 2..=36 MHz here; like WCH's `I2C_Init`, write
        // HCLK in MHz whatever it is.
        let freq_range = (freq_in / 1_000_000).min(0x3f);

        regs.ctrl2().modify(|w| w.set_freq(freq_range as u8)); // set i2c clock in

        let i2c_clk = freq.0;
        if i2c_clk <= 100_000 {
            let tmp = freq_in / (i2c_clk * 2);
            let tmp = u32::max(tmp, 0x04);

            regs.rtr().write(|w| w.set_trise((freq_range + 1).min(0x3f) as u8));
            regs.ckcfgr().write(|w| {
                w.set_ccr(tmp as _);
            });
        } else {
            // high speed, use duty cycle
            let tmp = if config.duty == Duty::Duty2_1 {
                freq_in / (i2c_clk * 3)
            } else {
                freq_in / (i2c_clk * 25)
            };
            let tmp = u32::max(tmp, 0x01);

            regs.rtr().write(|w| w.set_trise((freq_range * 300 / 1000 + 1) as u8));
            regs.ckcfgr().write(|w| {
                w.set_f_s(true);
                w.set_duty(config.duty as u8 != 0);
                w.set_ccr(tmp as u16);
            });
        }

        regs.ctrl1().modify(|w| w.set_pe(true));
    }

    fn generate_stop() {
        T::regs().ctrl1().modify(|w| w.set_stop(true));
    }

    /// Wait until a previously requested STOP has been generated and HW has
    /// cleared CTRL1.STOP, so the next START's read-modify-write cannot write
    /// STOP back (ch32-hal#221).
    ///
    /// A STOP requested once the peripheral has already let go of an idle bus
    /// is never generated, and would hold off every later START: clear it.
    /// Seen at 400 kHz after a NACK, on CH584 with only the pins' pull-ups.
    fn wait_stop_finished(timeout: Timeout) -> Result<(), Error> {
        let regs = T::regs();
        while regs.ctrl1().read().stop() {
            if !regs.star2().read().busy() {
                regs.ctrl1().modify(|w| w.set_stop(false));
                break;
            }
            timeout.check()?;
        }
        Ok(())
    }

    /// Reset the peripheral, keeping its clock setup. After a timeout it may
    /// hold a START that never goes out, or see the bus busy for good (a START
    /// requested while SCL was held low), so the next transfer would time out
    /// too. Done at the next START rather than at the timeout: while the
    /// fault lasts, a reset sees the bus busy again.
    fn reset() {
        let regs = T::regs();
        let (freq, ckcfgr, rtr) = (regs.ctrl2().read().freq(), regs.ckcfgr().read(), regs.rtr().read());
        regs.ctrl1().modify(|w| w.set_swrst(true));
        regs.ctrl1().modify(|w| w.set_swrst(false));
        regs.ctrl2().modify(|w| w.set_freq(freq));
        regs.rtr().write_value(rtr);
        regs.ckcfgr().write_value(ckcfgr);
        regs.ctrl1().modify(|w| w.set_pe(true));
    }

    /// Pass `r` on; after a timeout, have the next START reset the
    /// peripheral first.
    fn recover<R>(r: Result<R, Error>) -> Result<R, Error> {
        if matches!(r, Err(Error::Timeout)) {
            T::state().reset_pending.store(true, Ordering::Relaxed);
        }
        r
    }

    /// Issue a START (optionally with ACK) only after any pending STOP finishes.
    fn launch_start(ack: bool, timeout: Timeout) -> Result<(), Error> {
        let state = T::state();
        if state.reset_pending.load(Ordering::Relaxed) {
            state.reset_pending.store(false, Ordering::Relaxed);
            Self::reset();
        }
        Self::wait_stop_finished(timeout)?;
        T::regs().ctrl1().modify(|reg| {
            reg.set_start(true);
            if ack {
                reg.set_ack(true);
            }
        });
        Ok(())
    }

    fn check_and_clear_error_flags() -> Result<Star1, Error> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let star1 = T::regs().star1().read();

        if star1.timeout() {
            T::regs().star1().modify(|w| w.set_timeout(false));
            // Release the bus; otherwise the master can keep SCL low (ch32-hal#222).
            Self::generate_stop();
            return Err(Error::Timeout);
        }

        if star1.pecerr() {
            T::regs().star1().modify(|w| w.set_pecerr(false));
            return Err(Error::Crc);
        }

        if star1.ovr() {
            T::regs().star1().modify(|w| w.set_ovr(false));
            return Err(Error::Overrun);
        }

        if star1.af() {
            T::regs().star1().modify(|w| w.set_af(false));
            // Without STOP the peripheral stays in master mode with SCL held
            // low (ch32-hal#222).
            Self::generate_stop();
            return Err(Error::Nack);
        }

        if star1.arlo() {
            T::regs().star1().modify(|w| w.set_arlo(false));
            Self::generate_stop();
            return Err(Error::Arbitration);
        }

        // The STM32F1 errata say BERR may be detected wrongly, and recommend
        // ignoring and clearing it.
        if star1.berr() {
            T::regs().star1().modify(|w| w.set_berr(false));
        }

        Ok(star1)
    }

    /// Check we, not another master, generated the START just seen.
    fn check_start_ours() -> Result<(), Error> {
        if T::regs().ctrl1().read().start() || !T::regs().star2().read().msl() {
            Self::generate_stop();
            return Err(Error::Arbitration);
        }
        Ok(())
    }

    fn write_bytes(&mut self, addr: u8, bytes: &[u8], timeout: Timeout, frame: FrameOptions) -> Result<(), Error> {
        if frame.send_start() {
            // Send a START condition
            Self::launch_start(false, timeout)?;

            // Wait until START condition was generated
            while !Self::check_and_clear_error_flags()?.sb() {
                timeout.check()?;
            }

            Self::check_start_ours()?;

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar(addr << 1));

            // Wait for the address to be acknowledged. If a NACK occurs, the
            // ADDR bit will never be set.
            while !Self::check_and_clear_error_flags()?.addr() {
                timeout.check()?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();
        }

        // Send bytes
        for c in bytes {
            self.send_byte(*c, timeout)?;
        }

        if frame.send_stop() {
            // Send a STOP condition
            Self::generate_stop();
        }

        // Fallthrough is success
        Ok(())
    }

    fn send_byte(&self, byte: u8, timeout: Timeout) -> Result<(), Error> {
        // Wait until we're ready for sending
        while !Self::check_and_clear_error_flags()?.tx_e() {
            timeout.check()?;
        }

        // Push out a byte of data
        T::regs().datar().write(|reg| reg.set_datar(byte));

        // Wait until byte is transferred
        while !Self::check_and_clear_error_flags()?.btf() {
            timeout.check()?;
        }

        Ok(())
    }

    fn recv_byte(&self, timeout: Timeout) -> Result<u8, Error> {
        while !Self::check_and_clear_error_flags()?.rx_ne() {
            timeout.check()?;
        }

        let value = T::regs().datar().read().datar();
        Ok(value)
    }

    fn blocking_read_timeout(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        timeout: Timeout,
        frame: FrameOptions,
    ) -> Result<(), Error> {
        let Some((last, buffer)) = buffer.split_last_mut() else {
            return Err(Error::Overrun);
        };

        if frame.send_start() {
            // Send a START condition and set ACK bit
            Self::launch_start(true, timeout)?;

            // Wait until START condition was generated
            while !Self::check_and_clear_error_flags()?.sb() {
                timeout.check()?;
            }

            Self::check_start_ours()?;

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar((addr << 1) + 1));

            // Wait for the address to be acknowledged
            while !Self::check_and_clear_error_flags()?.addr() {
                timeout.check()?;
            }

            // A single byte's NACK must be set before ADDR is cleared.
            if buffer.is_empty() && frame.send_nack() {
                T::regs().ctrl1().modify(|w| w.set_ack(false));
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();
        }

        // Receive bytes into buffer
        for c in buffer {
            *c = self.recv_byte(timeout)?;
        }

        // Prepare to send NACK then STOP after next byte
        T::regs().ctrl1().modify(|reg| {
            if frame.send_nack() {
                reg.set_ack(false);
            }
            if frame.send_stop() {
                reg.set_stop(true);
            }
        });

        // Receive last byte
        *last = self.recv_byte(timeout)?;

        // Fallthrough is success
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read(&mut self, addr: u8, read: &mut [u8]) -> Result<(), Error> {
        let timeout = self.timeout();
        Self::recover(self.blocking_read_timeout(addr, read, timeout, FrameOptions::FirstAndLastFrame))
    }

    /// Blocking write.
    pub fn blocking_write(&mut self, addr: u8, write: &[u8]) -> Result<(), Error> {
        let timeout = self.timeout();
        Self::recover(self.write_bytes(addr, write, timeout, FrameOptions::FirstAndLastFrame))
    }

    /// Blocking write, restart, read.
    pub fn blocking_write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // Check empty read buffer before starting transaction. Otherwise, we would not generate the
        // stop condition below.
        if read.is_empty() {
            return Err(Error::Overrun);
        }

        let timeout = self.timeout();

        Self::recover(
            self.write_bytes(addr, write, timeout, FrameOptions::FirstFrame)
                .and_then(|()| self.blocking_read_timeout(addr, read, timeout, FrameOptions::FirstAndLastFrame)),
        )
    }

    /// Blocking transaction with operations.
    ///
    /// Consecutive operations of same type are merged. See [transaction contract] for details.
    ///
    /// [transaction contract]: embedded_hal::i2c::I2c::transaction
    pub fn blocking_transaction(&mut self, addr: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let timeout = self.timeout();

        for (op, frame) in operation_frames(operations)? {
            Self::recover(match op {
                Operation::Read(read) => self.blocking_read_timeout(addr, read, timeout, frame),
                Operation::Write(write) => self.write_bytes(addr, write, timeout, frame),
            })?;
        }

        Ok(())
    }
}

// ======== Async

impl<'d, T: Instance> I2c<'d, T, Async> {
    /// Wait for `done` on STAR1, sleeping on the interrupt. `buf` also
    /// enables the TxE/RxNE interrupt.
    async fn wait(done: impl Fn(Star1) -> bool, buf: bool) -> Result<Star1, Error> {
        let state = T::state();
        poll_fn(|cx| {
            state.waker.register(cx.waker());

            match Self::check_and_clear_error_flags() {
                Err(e) => Poll::Ready(Err(e)),
                Ok(sr1) if done(sr1) => Poll::Ready(Ok(sr1)),
                Ok(_) => {
                    // When pending, (re-)enable interrupts to wake us up.
                    critical_section::with(|_| {
                        T::regs().ctrl2().modify(|w| {
                            w.set_iterren(true);
                            w.set_itevten(true);
                            w.set_itbufen(buf);
                        })
                    });
                    Poll::Pending
                }
            }
        })
        .await
    }

    fn on_drop() -> OnDrop<impl FnOnce()> {
        // Disable the interrupts and release the bus if the future is
        // cancelled or fails mid-transfer.
        OnDrop::new(|| {
            critical_section::with(|_| {
                T::regs().ctrl2().modify(|w| {
                    w.set_iterren(false);
                    w.set_itevten(false);
                    w.set_itbufen(false);
                })
            });
            Self::generate_stop();
        })
    }

    async fn write_frame(&mut self, address: u8, write: &[u8], frame: FrameOptions) -> Result<(), Error> {
        let on_drop = Self::on_drop();

        if frame.send_start() {
            // Send a START condition
            Self::launch_start(false, self.timeout())?;

            // Wait until START condition was generated
            Self::wait(|sr1| sr1.sb(), false).await?;

            Self::check_start_ours()?;

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar(address << 1));

            // Wait for the address to be acknowledged
            Self::wait(|sr1| sr1.addr(), false).await?;

            // Clear condition by reading SR2
            T::regs().star2().read();
        }

        for &byte in write {
            Self::wait(|sr1| sr1.tx_e(), true).await?;
            T::regs().datar().write(|reg| reg.set_datar(byte));
        }

        if frame.send_stop() {
            // Wait for the last byte to go out before the STOP.
            Self::wait(|sr1| sr1.btf(), false).await?;
            Self::generate_stop();
        }

        on_drop.defuse();

        // Fallthrough is success
        Ok(())
    }

    /// Write.
    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        Self::recover(self.write_frame(address, write, FrameOptions::FirstAndLastFrame).await)
    }

    /// Read.
    pub async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        Self::recover(self.read_frame(address, buffer, FrameOptions::FirstAndLastFrame).await)
    }

    async fn read_frame(&mut self, address: u8, buffer: &mut [u8], frame: FrameOptions) -> Result<(), Error> {
        let Some((last, buffer)) = buffer.split_last_mut() else {
            return Err(Error::Overrun);
        };

        let on_drop = Self::on_drop();

        if frame.send_start() {
            // Send a START condition and set ACK bit
            Self::launch_start(true, self.timeout())?;

            // Wait until START condition was generated
            Self::wait(|sr1| sr1.sb(), false).await?;

            Self::check_start_ours()?;

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar((address << 1) + 1));

            // Wait for the address to be acknowledged
            Self::wait(|sr1| sr1.addr(), false).await?;

            // A single byte's NACK must be set before ADDR is cleared.
            if buffer.is_empty() && frame.send_nack() {
                T::regs().ctrl1().modify(|w| w.set_ack(false));
            }

            // Clear condition by reading SR2
            T::regs().star2().read();
        }

        for c in buffer {
            Self::wait(|sr1| sr1.rx_ne(), true).await?;
            *c = T::regs().datar().read().datar();
        }

        // Prepare to send NACK then STOP after next byte
        T::regs().ctrl1().modify(|reg| {
            if frame.send_nack() {
                reg.set_ack(false);
            }
            if frame.send_stop() {
                reg.set_stop(true);
            }
        });

        Self::wait(|sr1| sr1.rx_ne(), true).await?;
        *last = T::regs().datar().read().datar();

        on_drop.defuse();

        // Fallthrough is success
        Ok(())
    }

    /// Write, restart, read.
    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // Check empty read buffer before starting transaction. Otherwise, we would not generate the
        // stop condition below.
        if read.is_empty() {
            return Err(Error::Overrun);
        }

        Self::recover(self.write_frame(address, write, FrameOptions::FirstFrame).await)?;
        Self::recover(self.read_frame(address, read, FrameOptions::FirstAndLastFrame).await)
    }

    /// Transaction with operations.
    ///
    /// Consecutive operations of same type are merged. See [transaction contract] for details.
    ///
    /// [transaction contract]: embedded_hal::i2c::I2c::transaction
    pub async fn transaction(&mut self, addr: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        for (op, frame) in operation_frames(operations)? {
            Self::recover(match op {
                Operation::Read(read) => self.read_frame(addr, read, frame).await,
                Operation::Write(write) => self.write_frame(addr, write, frame).await,
            })?;
        }

        Ok(())
    }
}

// ======== Common

impl<'d, T: Instance, M: Mode> Drop for I2c<'d, T, M> {
    fn drop(&mut self) {
        T::regs().ctrl1().modify(|w| w.set_pe(false));
        crate::gpio::SealedPin::set_as_disconnected(&*self._scl);
        crate::gpio::SealedPin::set_as_disconnected(&*self._sda);
        #[cfg(ch592)]
        crate::debug::set_debug_interface_enabled(true);
    }
}

struct State {
    waker: AtomicWaker,
    /// A transfer timed out: reset the peripheral before the next START.
    reset_pending: AtomicBool,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            reset_pending: AtomicBool::new(false),
        }
    }
}

trait SealedInstance: crate::sys::RemapPeripheral {
    fn regs() -> pac::i2c::I2c;
    fn state() -> &'static State;
    /// Ungate the clock.
    fn enable();
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + embassy_hal_internal::PeripheralType + 'static + Send {
    /// Interrupt for this instance (events and errors).
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl SealedInstance for peripherals::I2C0 {
    fn regs() -> pac::i2c::I2c {
        pac::I2C0
    }

    fn state() -> &'static State {
        static STATE: State = State::new();
        &STATE
    }

    fn enable() {
        set_clock_enabled(ClockGate::I2c, true);
    }
}

impl Instance for peripherals::I2C0 {
    type Interrupt = crate::interrupt::typelevel::I2C0;
}

/// I2C0 has no alternate pins on CH592.
#[cfg(ch592)]
impl crate::sys::RemapPeripheral for peripherals::I2C0 {
    fn set_remap(remap: u8) {
        assert!(remap == 0);
    }
}

#[cfg(any(ch584, ch585))]
impl crate::sys::RemapPeripheral for peripherals::I2C0 {
    fn set_remap(remap: u8) {
        crate::sys::set_remap(crate::sys::RemapGroup::I2c, remap)
    }
}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);

#[cfg(ch592)]
pin_trait_impl!(crate::i2c::SclPin, I2C0, PB15, 0);
#[cfg(ch592)]
pin_trait_impl!(crate::i2c::SdaPin, I2C0, PB14, 0);

#[cfg(any(ch584, ch585))]
pin_trait_impl!(crate::i2c::SclPin, I2C0, PB13, 0);
#[cfg(any(ch584, ch585))]
pin_trait_impl!(crate::i2c::SdaPin, I2C0, PB12, 0);
#[cfg(any(ch584, ch585))]
pin_trait_impl!(crate::i2c::SclPin, I2C0, PB21, 1);
#[cfg(any(ch584, ch585))]
pin_trait_impl!(crate::i2c::SdaPin, I2C0, PB20, 1);

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match *self {
            Self::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Self::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::Timeout => embedded_hal::i2c::ErrorKind::Other,
            Self::Crc => embedded_hal::i2c::ErrorKind::Other,
            Self::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            Self::ZeroLengthTransfer => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::ErrorType for I2c<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::I2c for I2c<'d, T, M> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, write)
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, write, read)
    }

    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.blocking_transaction(address, operations)
    }
}

impl<'d, T: Instance> embedded_hal_async::i2c::I2c for I2c<'d, T, Async> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.write_read(address, write, read).await
    }

    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.transaction(address, operations).await
    }
}

/// Frame type in I2C transaction.
///
/// This tells each method what kind of framing to use, to generate a (repeated) start condition (ST
/// or SR), and/or a stop condition (SP). For read operations, this also controls whether to send an
/// ACK or NACK after the last byte received.
///
/// For write operations, the following options are identical because they differ only in the (N)ACK
/// treatment relevant for read operations:
///
/// - `FirstFrame` and `FirstAndNextFrame`
/// - `NextFrame` and `LastFrameNoStop`
///
/// Abbreviations used below:
///
/// - `ST` = start condition
/// - `SR` = repeated start condition
/// - `SP` = stop condition
/// - `ACK`/`NACK` = last byte in read operation
#[derive(Copy, Clone)]
#[allow(dead_code)]
enum FrameOptions {
    /// `[ST/SR]+[NACK]+[SP]` First frame (of this type) in transaction and also last frame overall.
    FirstAndLastFrame,
    /// `[ST/SR]+[NACK]` First frame of this type in transaction, last frame in a read operation but
    /// not the last frame overall.
    FirstFrame,
    /// `[ST/SR]+[ACK]` First frame of this type in transaction, neither last frame overall nor last
    /// frame in a read operation.
    FirstAndNextFrame,
    /// `[ACK]` Middle frame in a read operation (neither first nor last).
    NextFrame,
    /// `[NACK]+[SP]` Last frame overall in this transaction but not the first frame.
    LastFrame,
    /// `[NACK]` Last frame in a read operation but not last frame overall in this transaction.
    LastFrameNoStop,
}

#[allow(dead_code)]
impl FrameOptions {
    /// Sends start or repeated start condition before transfer.
    fn send_start(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::FirstAndNextFrame => true,
            Self::NextFrame | Self::LastFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends stop condition after transfer.
    fn send_stop(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::LastFrame => true,
            Self::FirstFrame | Self::FirstAndNextFrame | Self::NextFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends NACK after last byte received, indicating end of read operation.
    fn send_nack(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::LastFrame | Self::LastFrameNoStop => true,
            Self::FirstAndNextFrame | Self::NextFrame => false,
        }
    }
}

/// Iterates over operations in transaction.
///
/// Returns necessary frame options for each operation to uphold the [transaction contract] and have
/// the right start/stop/(N)ACK conditions on the wire.
///
/// [transaction contract]: embedded_hal::i2c::I2c::transaction
fn operation_frames<'a, 'b: 'a>(
    operations: &'a mut [Operation<'b>],
) -> Result<impl IntoIterator<Item = (&'a mut Operation<'b>, FrameOptions)>, Error> {
    use core::iter;

    use embedded_hal::i2c::Operation::{Read, Write};

    // Check empty read buffer before starting transaction. Otherwise, we would risk halting with an
    // error in the middle of the transaction.
    if operations.iter().any(|op| match op {
        Read(read) => read.is_empty(),
        Write(_) => false,
    }) {
        return Err(Error::Overrun);
    }

    let mut operations = operations.iter_mut().peekable();

    let mut next_first_frame = true;

    Ok(iter::from_fn(move || {
        let op = operations.next()?;

        // Is `op` first frame of its type?
        let first_frame = next_first_frame;
        let next_op = operations.peek();

        // Get appropriate frame options as combination of the following properties:
        //
        // - For each first operation of its type, generate a (repeated) start condition.
        // - For the last operation overall in the entire transaction, generate a stop condition.
        // - For read operations, check the next operation: if it is also a read operation, we merge
        //   these and send ACK for all bytes in the current operation; send NACK only for the final
        //   read operation's last byte (before write or end of entire transaction) to indicate last
        //   byte read and release the bus for transmission of the bus master's next byte (or stop).
        //
        // We check the third property unconditionally, i.e. even for write opeartions. This is okay
        // because the resulting frame options are identical for write operations.
        let frame = match (first_frame, next_op) {
            (true, None) => FrameOptions::FirstAndLastFrame,
            (true, Some(Read(_))) => FrameOptions::FirstAndNextFrame,
            (true, Some(Write(_))) => FrameOptions::FirstFrame,
            //
            (false, None) => FrameOptions::LastFrame,
            (false, Some(Read(_))) => FrameOptions::NextFrame,
            (false, Some(Write(_))) => FrameOptions::LastFrameNoStop,
        };

        // Pre-calculate if `next_op` is the first operation of its type. We do this here and not at
        // the beginning of the loop because we hand out `op` as iterator value and cannot access it
        // anymore in the next iteration.
        next_first_frame = match (&op, next_op) {
            (_, None) => false,
            (Read(_), Some(Write(_))) | (Write(_), Some(Read(_))) => true,
            (Read(_), Some(Read(_))) | (Write(_), Some(Write(_))) => false,
        };

        Some((op, frame))
    }))
}
