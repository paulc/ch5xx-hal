//! Software ("bit-bang") I2C master on any two GPIOs, clocked by a `TMRx`
//! timer.
//!
//! The hardware [`crate::i2c`] has fixed pins: on CH592 only PB14/PB15, the
//! 2-wire debug link's. This driver runs on any pair of pins instead, up to
//! about 100 kHz.
//!
//! The timer ticks twice per SCL period and each tick runs one step of a
//! state machine, which moves SCL or SDA and samples SDA. The async driver
//! steps it in the timer interrupt; the blocking one polls the timer.
//!
//! - Open drain is emulated: a line is pulled low as an output (its latch is
//!   kept at 0) and released as an input. The bus needs pull-ups: external
//!   ones, or for a short bus at a low clock the pins' own
//!   ([`Config::internal_pullups`]).
//! - Clock stretching is honoured, for up to [`Config::timeout_us`] per wait.
//! - Single master: arbitration is not checked, beyond finding the bus held
//!   low at a start ([`Error::Bus`]).
//! - Cost: two timer interrupts per bit while transferring. Enable the HAL's
//!   `highcode` feature: it runs interrupt entry and the step from RAM. From
//!   flash a step outlasts a 100 kHz half period and the bus runs at about
//!   half the set clock (50 kHz still works). With it, at 100 kHz and a
//!   60 MHz system clock, the interrupts take 40-45% of the CPU while waiting
//!   on a stretched SCL (measured), and more while moving bits.
//! - The interrupt handler changes the pins' direction register
//!   read-modify-write; other code doing that must do it in a critical
//!   section, as [`crate::gpio`] does.
//!
//! Implements the `embedded-hal` and `embedded-hal-async` I2C traits, with
//! the hardware driver's [`Error`].

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embedded_hal::i2c::Operation;

use crate::gpio::{Flex, Pin, Port, Pull};
pub use crate::i2c::Error;
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, timer, Peri};

/// Driver configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// SCL frequency. The default is 100 kHz; much above that the interrupt
    /// handler cannot keep up.
    pub frequency: Hertz,
    /// Longest wait for a device stretching SCL (or for SCL to rise), in
    /// microseconds, before [`Error::Timeout`]. The default is 25 ms.
    pub timeout_us: u32,
    /// Turn on the pins' pull-ups (tens of kohms: only for a short bus at a
    /// low clock). The default is off.
    pub internal_pullups: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Hertz::khz(100),
            timeout_us: 25_000,
            internal_pullups: false,
        }
    }
}

/// Software I2C master.
pub struct SoftI2c<'d, T: Instance, M: Mode> {
    _tim: Peri<'d, T>,
    _scl: Flex<'d>,
    _sda: Flex<'d>,
    scl: Line,
    sda: Line,
    /// Timer ticks per step (half an SCL period).
    period: u32,
    /// Steps a stretch may last.
    timeout: u32,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance> SoftI2c<'d, T, Blocking> {
    /// Create the driver; it runs the state machine by polling `tim`.
    ///
    /// # Panics
    ///
    /// If `config.frequency` is 0 or above a quarter of the system clock.
    pub fn new_blocking(tim: Peri<'d, T>, scl: Peri<'d, impl Pin>, sda: Peri<'d, impl Pin>, config: Config) -> Self {
        Self::new_inner(tim, scl, sda, config)
    }
}

impl<'d, T: Instance> SoftI2c<'d, T, Async> {
    /// Create the driver; the async methods run the state machine in `tim`'s
    /// interrupt (the blocking ones still poll).
    ///
    /// # Panics
    ///
    /// As [`SoftI2c::new_blocking`].
    pub fn new(
        tim: Peri<'d, T>,
        scl: Peri<'d, impl Pin>,
        sda: Peri<'d, impl Pin>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let this = Self::new_inner(tim, scl, sda, config);
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        this
    }

    /// Run `operations` on the device at `address` (7-bit), as
    /// [`embedded_hal_async::i2c::I2c::transaction`]. Dropping the future
    /// part way releases both lines, leaving the device mid-transfer.
    pub async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        if !self.setup(address, operations)? {
            return Ok(());
        }
        let r = T::regs();
        let guard = OnDrop::new(|| self.stop());
        r.inter_en().write(|w| w.set_ie_cyc_end(true));
        let result = poll_fn(|cx| {
            T::waker().register(cx.waker());
            let m = T::machine().0.get();
            // Written by the interrupt handler.
            if unsafe { core::ptr::addr_of!((*m).st).read_volatile() } == St::Done {
                compiler_fence(Ordering::SeqCst);
                Poll::Ready(unsafe { (*m).result })
            } else {
                Poll::Pending
            }
        })
        .await;
        drop(guard);
        result
    }

    /// Read `read.len()` bytes from the device at `address`.
    pub async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Read(read)]).await
    }

    /// Write `write` to the device at `address`. An empty `write` only sends
    /// the address, to see whether the device acknowledges it.
    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write)]).await
    }

    /// Write `write`, then, after a repeated start, read `read.len()` bytes.
    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(write), Operation::Read(read)])
            .await
    }
}

impl<'d, T: Instance, M: Mode> SoftI2c<'d, T, M> {
    fn new_inner(tim: Peri<'d, T>, scl: Peri<'d, impl Pin>, sda: Peri<'d, impl Pin>, config: Config) -> Self {
        let sysclk = crate::rcc::clocks().sysclk.0;
        let hz = config.frequency.0;
        if hz == 0 || hz > sysclk / 4 {
            panic!("soft I2C frequency out of range");
        }
        T::enable();
        T::regs().inter_en().write(|_| {});
        T::regs().ctrl_mod().write(|w| w.set_all_clear(true));

        let (scl_line, sda_line) = (Line::new(scl.port(), scl.pin()), Line::new(sda.port(), sda.pin()));
        let pull = if config.internal_pullups { Pull::Up } else { Pull::None };
        let mut scl = Flex::new(scl);
        let mut sda = Flex::new(sda);
        for pin in [&mut scl, &mut sda] {
            // Released (input); pulled low by turning the output on with its
            // latch at 0.
            pin.set_low();
            pin.set_as_input(pull);
        }
        let steps_per_sec = 2 * hz as u64;
        Self {
            _tim: tim,
            _scl: scl,
            _sda: sda,
            scl: scl_line,
            sda: sda_line,
            period: (sysclk + hz) / (2 * hz),
            timeout: (config.timeout_us as u64 * steps_per_sec / 1_000_000).max(2) as u32,
            _phantom: PhantomData,
        }
    }

    /// Run `operations` on the device at `address` (7-bit), as
    /// [`embedded_hal::i2c::I2c::transaction`]: a start, then for each run of
    /// operations of the same kind the address and the bytes, a repeated
    /// start between runs, and a stop.
    ///
    /// An empty write is allowed (with only empty writes, just the address is
    /// sent); an empty read is [`Error::ZeroLengthTransfer`].
    pub fn blocking_transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        if !self.setup(address, operations)? {
            return Ok(());
        }
        let r = T::regs();
        let m = T::machine().0.get();
        loop {
            while !r.int_flag().read().if_cyc_end() {}
            r.int_flag().write(|w| w.set_if_cyc_end(true));
            // A critical section, as GPIO direction changes elsewhere expect.
            if critical_section::with(|_| unsafe { (*m).step() }) {
                break;
            }
        }
        self.stop();
        unsafe { (*m).result }
    }

    /// Read `read.len()` bytes from the device at `address`.
    pub fn blocking_read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Error> {
        self.blocking_transaction(address, &mut [Operation::Read(read)])
    }

    /// Write `write` to the device at `address`. An empty `write` only sends
    /// the address, to see whether the device acknowledges it.
    pub fn blocking_write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.blocking_transaction(address, &mut [Operation::Write(write)])
    }

    /// Write `write`, then, after a repeated start, read `read.len()` bytes.
    pub fn blocking_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.blocking_transaction(address, &mut [Operation::Write(write), Operation::Read(read)])
    }

    /// Load the state machine and start the timer. `Ok(false)`: nothing to
    /// send.
    fn setup(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<bool, Error> {
        if operations
            .iter()
            .any(|o| matches!(o, Operation::Read(b) if b.is_empty()))
        {
            return Err(Error::ZeroLengthTransfer);
        }
        if operations.is_empty() {
            return Ok(false);
        }
        let read = operations
            .iter()
            .find(|o| !matches!(o, Operation::Write(b) if b.is_empty()))
            .is_some_and(|o| matches!(o, Operation::Read(_)));
        let r = T::regs();
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| w.set_all_clear(true));
        // The timer is stopped and its interrupt masked: the handler is not
        // using the machine.
        unsafe {
            *T::machine().0.get() = Machine {
                scl: self.scl,
                sda: self.sda,
                addr: address,
                ops: operations as *mut [Operation<'_>] as *mut [Operation<'static>],
                op: 0,
                idx: 0,
                read,
                st: St::Start,
                frame: Frame::Addr,
                byte: 0,
                bit: 0,
                waited: 0,
                timeout: self.timeout,
                result: Ok(()),
            };
        }
        compiler_fence(Ordering::SeqCst);
        r.cnt_end().write(|w| w.set_cnt_end(self.period));
        r.int_flag().write(|w| w.0 = 0xff);
        r.ctrl_mod().write(|w| w.set_count_en(true));
        Ok(true)
    }

    /// Stop the timer and release both lines.
    fn stop(&self) {
        let r = T::regs();
        r.inter_en().write(|_| {});
        r.ctrl_mod().write(|w| w.set_all_clear(true));
        compiler_fence(Ordering::SeqCst);
        critical_section::with(|_| {
            let m = unsafe { &mut *T::machine().0.get() };
            m.st = St::Done;
            self.scl.release();
            self.sda.release();
        });
    }
}

impl<'d, T: Instance, M: Mode> Drop for SoftI2c<'d, T, M> {
    fn drop(&mut self) {
        self.stop();
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::ErrorType for SoftI2c<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::I2c for SoftI2c<'d, T, M> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.blocking_transaction(address, operations)
    }
}

impl<'d, T: Instance> embedded_hal_async::i2c::I2c for SoftI2c<'d, T, Async> {
    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        SoftI2c::transaction(self, address, operations).await
    }
}

// ========
// ## State machine

/// A bus line, driven open drain through its direction bit.
#[derive(Clone, Copy)]
struct Line {
    port: Port,
    mask: u32,
}

impl Line {
    const NONE: Self = Self { port: Port::A, mask: 0 };

    fn new(port: Port, pin: u8) -> Self {
        Self { port, mask: 1 << pin }
    }

    #[inline(always)]
    fn low(self) {
        let g = pac::GPIO;
        match self.port {
            Port::A => g.pa_dir().modify(|w| w.0 |= self.mask),
            Port::B => g.pb_dir().modify(|w| w.0 |= self.mask),
        }
    }

    #[inline(always)]
    fn release(self) {
        let g = pac::GPIO;
        match self.port {
            Port::A => g.pa_dir().modify(|w| w.0 &= !self.mask),
            Port::B => g.pb_dir().modify(|w| w.0 &= !self.mask),
        }
    }

    #[inline(always)]
    fn is_high(self) -> bool {
        let g = pac::GPIO;
        let v = match self.port {
            Port::A => g.pa_pin().read().0,
            Port::B => g.pb_pin().read().0,
        };
        v & self.mask != 0
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum St {
    /// SDA falls with SCL high: a (repeated) start.
    Start,
    /// SCL falls, after sampling SDA; SDA takes the next bit.
    Low,
    /// SCL is released.
    High,
    /// SCL released before a repeated start.
    RestartHigh,
    /// SCL released before a stop.
    StopHigh,
    /// SDA rises with SCL high: the stop.
    StopEnd,
    Done,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Frame {
    Addr,
    Tx,
    Rx,
}

enum Next {
    Byte,
    Restart,
    Stop,
}

struct Machine {
    scl: Line,
    sda: Line,
    addr: u8,
    ops: *mut [Operation<'static>],
    /// Cursor: the next byte is `ops[op]`'s `idx`th.
    op: usize,
    idx: usize,
    /// Direction of the current address.
    read: bool,
    st: St,
    frame: Frame,
    /// Shift register.
    byte: u8,
    /// Bit of the frame, 0..=8 (8 is the acknowledge), 9 once done.
    bit: u8,
    waited: u32,
    timeout: u32,
    result: Result<(), Error>,
}

impl Machine {
    const IDLE: Self = Self {
        scl: Line::NONE,
        sda: Line::NONE,
        addr: 0,
        ops: core::ptr::slice_from_raw_parts_mut(core::ptr::null_mut(), 0),
        op: 0,
        idx: 0,
        read: false,
        st: St::Done,
        frame: Frame::Addr,
        byte: 0,
        bit: 0,
        waited: 0,
        timeout: 0,
        result: Ok(()),
    };

    /// Run one step (half an SCL period); `true` once the transfer is over.
    ///
    /// In `.highcode`: RAM with `qingke-rt`'s `highcode` feature, which runs
    /// it several times faster than flash does.
    #[inline(never)]
    #[unsafe(link_section = ".highcode.ch5xx_hal.soft_i2c.step")]
    fn step(&mut self) -> bool {
        let (scl, sda) = (self.scl, self.sda);
        match self.st {
            St::Start => {
                if !scl.is_high() {
                    return self.stretch();
                }
                self.waited = 0;
                if !sda.is_high() {
                    return self.abort(Error::Bus);
                }
                sda.low();
                self.frame = Frame::Addr;
                self.byte = self.addr << 1 | self.read as u8;
                self.bit = 0;
                self.st = St::Low;
            }
            St::Low => {
                if !scl.is_high() {
                    return self.stretch();
                }
                self.waited = 0;
                if self.bit > 0 {
                    // What the device put on SDA for the previous bit.
                    let b = sda.is_high();
                    match self.frame {
                        Frame::Rx if self.bit <= 8 => self.byte = self.byte << 1 | b as u8,
                        Frame::Addr | Frame::Tx if self.bit == 9 && b => {
                            self.result = Err(Error::Nack);
                            self.stop_low();
                            return false;
                        }
                        _ => {}
                    }
                }
                if self.bit == 9 {
                    match self.next() {
                        Next::Byte => self.begin_byte(),
                        Next::Restart => {
                            scl.low();
                            sda.release();
                            self.st = St::RestartHigh;
                            return false;
                        }
                        Next::Stop => {
                            self.stop_low();
                            return false;
                        }
                    }
                }
                scl.low();
                match (self.frame, self.bit) {
                    (Frame::Rx, 0..=7) => sda.release(),
                    (Frame::Rx, _) => {
                        // Acknowledge unless this ends the run of reads.
                        let ops = unsafe { &mut *self.ops };
                        if let Operation::Read(b) = &mut ops[self.op] {
                            b[self.idx] = self.byte;
                        }
                        self.idx += 1;
                        if matches!(self.next(), Next::Byte) {
                            sda.low()
                        } else {
                            sda.release()
                        }
                    }
                    (_, 0..=7) => {
                        if self.byte & (0x80 >> self.bit) != 0 {
                            sda.release()
                        } else {
                            sda.low()
                        }
                    }
                    // The device acknowledges.
                    _ => sda.release(),
                }
                self.st = St::High;
            }
            St::High => {
                scl.release();
                self.bit += 1;
                self.st = St::Low;
            }
            St::RestartHigh => {
                scl.release();
                self.read = !self.read;
                self.st = St::Start;
            }
            St::StopHigh => {
                scl.release();
                self.st = St::StopEnd;
            }
            St::StopEnd => {
                if !scl.is_high() {
                    return self.stretch();
                }
                sda.release();
                self.st = St::Done;
                return true;
            }
            St::Done => return true,
        }
        false
    }

    /// SCL low, then SDA low, ready for a stop.
    #[inline(always)]
    fn stop_low(&mut self) {
        self.scl.low();
        self.sda.low();
        self.st = St::StopHigh;
    }

    /// SCL is held low: wait, up to the timeout.
    #[inline(always)]
    fn stretch(&mut self) -> bool {
        self.waited += 1;
        if self.waited > self.timeout {
            return self.abort(Error::Timeout);
        }
        false
    }

    #[inline(always)]
    fn abort(&mut self, e: Error) -> bool {
        self.result = Err(e);
        self.scl.release();
        self.sda.release();
        self.st = St::Done;
        true
    }

    /// What comes at the cursor, skipping finished operations.
    #[inline(always)]
    fn next(&mut self) -> Next {
        let ops = unsafe { &*self.ops };
        while let Some(o) = ops.get(self.op) {
            let (len, read) = match o {
                Operation::Write(b) => (b.len(), false),
                Operation::Read(b) => (b.len(), true),
            };
            if self.idx < len {
                return if read == self.read { Next::Byte } else { Next::Restart };
            }
            self.op += 1;
            self.idx = 0;
        }
        Next::Stop
    }

    /// Start the frame for the byte at the cursor.
    #[inline(always)]
    fn begin_byte(&mut self) {
        let ops = unsafe { &*self.ops };
        self.bit = 0;
        match &ops[self.op] {
            Operation::Write(b) => {
                self.frame = Frame::Tx;
                self.byte = b[self.idx];
                self.idx += 1;
            }
            Operation::Read(_) => {
                self.frame = Frame::Rx;
                self.byte = 0;
            }
        }
    }
}

// ========
// ## Interrupt

/// Timer interrupt handler for the async [`SoftI2c`]; bind it instead of
/// [`timer::InterruptHandler`].
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        r.int_flag().write(|w| w.set_if_cyc_end(true));
        let m = unsafe { &mut *T::machine().0.get() };
        if m.step() {
            r.inter_en().write(|_| {});
            r.ctrl_mod().write(|w| w.set_all_clear(true));
            compiler_fence(Ordering::SeqCst);
            T::waker().wake();
        }
    }
}

// ========
// ## Peripheral traits

/// The state machine of the driver on one timer. It is used by one side at a
/// time: the driver while the timer is stopped or its interrupt masked, the
/// interrupt handler otherwise.
struct Slot(UnsafeCell<Machine>);

unsafe impl Sync for Slot {}

trait SealedInstance {
    fn machine() -> &'static Slot;
}

/// A timer that can clock a [`SoftI2c`]: any `TMRx`.
#[allow(private_bounds)]
pub trait Instance: timer::Instance + SealedInstance {}

macro_rules! impl_instance {
    ($inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn machine() -> &'static Slot {
                static SLOT: Slot = Slot(UnsafeCell::new(Machine::IDLE));
                &SLOT
            }
        }

        impl Instance for peripherals::$inst {}
    };
}

impl_instance!(TMR0);
impl_instance!(TMR1);
impl_instance!(TMR2);
impl_instance!(TMR3);
