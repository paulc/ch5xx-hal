//! USB full-speed device driver for [`embassy-usb`](https://docs.rs/embassy-usb)
//! (CH592).
//!
//! The shape follows ch32-hal's `otg_fs` driver; the interrupt handling does
//! not:
//!
//! - The interrupt handler services every token itself, as WCH's SDK does:
//!   it NAKs the endpoint, flips its data toggle, records the result and
//!   clears `UIF_TRANSFER`. With `RB_UC_INT_BUSY` set the SIE NAKs all
//!   traffic while that flag is set, so leaving it to a task that may not be
//!   polling would stall the whole bus.
//! - `SETUP` is detected by `RB_UIS_SETUP_ACT` (the token field reads "idle"
//!   for it) and copied out of the shared EP0 buffer at once.
//! - The packet buffers are internal (1 KiB of RAM, 4-byte aligned) rather
//!   than passed in, because the hardware fixes where each direction of
//!   an endpoint lives relative to its `UEPn_DMA` address.
//!
//! Endpoints 1–7, IN and OUT, up to 64 bytes. Bulk and interrupt only:
//! isochronous endpoints and double buffering are not supported. A `write`
//! or `read` future dropped before it completes leaves its packet armed.
//!
//! Tested: enumeration, control transfers with data both ways, and bulk
//! OUT/IN as a CDC-ACM echo on a macOS host. Not yet tested: stall
//! handling, remote wakeup.
//!
//! The USB PHY needs the 48 MHz clock from the PLL, so run the system clock
//! from the PLL (e.g. `SYSCLK_FREQ_60MHZ`).
//!
//! ```ignore
//! bind_interrupts!(struct Irqs {
//!     USB => usb::InterruptHandler<peripherals::USB>;
//! });
//! let driver = usb::Driver::new(p.USB, Irqs, p.PB11, p.PB10);
//! ```

use core::cell::Cell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Poll;

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};

use crate::gpio::{AfType, Pull};
use crate::interrupt::typelevel::Interrupt as _;
use crate::sys::{set_clock_enabled, ClockGate};
use crate::{interrupt, pac, peripherals, Peri};

/// Endpoints 0–7. The 8–15 extension (`R32_EPX_MODE`) is not used.
const EP_COUNT: usize = 8;
const MAX_PACKET_SIZE: u16 = 64;

// MASK_UIS_TOKEN.
const TOKEN_OUT: u8 = 0b00;
const TOKEN_IN: u8 = 0b10;
const TOKEN_IDLE: u8 = 0b11;

// MASK_UEP_T_RES / MASK_UEP_R_RES.
const RES_ACK: u8 = 0b00;
const RES_NAK: u8 = 0b10;
const RES_STALL: u8 = 0b11;

/// `R16_UEPn_DMA` holds a 16-bit RAM address.
#[repr(C, align(4))]
struct Buffers {
    /// EP0 (shared by IN and OUT), then EP4's OUT and IN buffers.
    ep0_4: [u8; 192],
    /// EP1–3 and EP5–7: OUT at +0, IN at +64.
    ep: [[u8; 128]; 6],
}

static mut BUFFERS: Buffers = Buffers {
    ep0_4: [0; 192],
    ep: [[0; 128]; 6],
};

struct State {
    bus_waker: AtomicWaker,
    ep_wakers: [AtomicWaker; EP_COUNT],
    /// `EVENT_*` bits, set by the interrupt handler.
    bus_events: AtomicU32,
    /// A SETUP packet is waiting in `setup`.
    setup_ready: AtomicBool,
    setup: Mutex<Cell<[u8; 8]>>,
    /// Per-endpoint bits (bit n = EPn) for each direction.
    in_done: AtomicU32,
    out_done: AtomicU32,
    /// A transfer is armed and waiting for the host.
    in_armed: AtomicU32,
    out_armed: AtomicU32,
    in_enabled: AtomicU32,
    out_enabled: AtomicU32,
    /// `R8_USB_RX_LEN` of the last OUT per endpoint.
    rx_len: [AtomicU32; EP_COUNT],
}

const EVENT_RESET: u32 = 1 << 0;
const EVENT_SUSPEND: u32 = 1 << 1;
const EVENT_RESUME: u32 = 1 << 2;

#[allow(clippy::declare_interior_mutable_const)]
const NEW_WAKER: AtomicWaker = AtomicWaker::new();
#[allow(clippy::declare_interior_mutable_const)]
const NEW_LEN: AtomicU32 = AtomicU32::new(0);

static STATE: State = State {
    bus_waker: AtomicWaker::new(),
    ep_wakers: [NEW_WAKER; EP_COUNT],
    bus_events: AtomicU32::new(0),
    setup_ready: AtomicBool::new(false),
    setup: Mutex::new(Cell::new([0; 8])),
    in_done: AtomicU32::new(0),
    out_done: AtomicU32::new(0),
    in_armed: AtomicU32::new(0),
    out_armed: AtomicU32::new(0),
    in_enabled: AtomicU32::new(0),
    out_enabled: AtomicU32::new(0),
    rx_len: [NEW_LEN; EP_COUNT],
};

type EpCtrl = pac::common::Reg<pac::usb::regs::Uep0Ctrl, pac::common::RW>;
type EpTLen = pac::common::Reg<pac::usb::regs::Uep0TLen, pac::common::RW>;
type EpDma = pac::common::Reg<pac::usb::regs::Uep0Dma, pac::common::RW>;

/// `R8_UEPn_CTRL`. All eight share one layout.
fn ep_ctrl(ep: usize) -> EpCtrl {
    let off = if ep < 5 { 0x22 + 4 * ep } else { 0x52 + 4 * ep };
    unsafe { EpCtrl::from_ptr((pac::USB.as_ptr() as *mut u8).add(off) as _) }
}

/// `R8_UEPn_T_LEN`, just below `R8_UEPn_CTRL`.
fn ep_t_len(ep: usize) -> EpTLen {
    let off = if ep < 5 { 0x20 + 4 * ep } else { 0x50 + 4 * ep };
    unsafe { EpTLen::from_ptr((pac::USB.as_ptr() as *mut u8).add(off) as _) }
}

/// `R16_UEPn_DMA`. EP4 has none: its buffers follow EP0's.
fn ep_dma(ep: usize) -> EpDma {
    let off = match ep {
        0..=3 => 0x10 + 4 * ep,
        5..=7 => 0x40 + 4 * ep,
        _ => unreachable!(),
    };
    unsafe { EpDma::from_ptr((pac::USB.as_ptr() as *mut u8).add(off) as _) }
}

/// Start of the DMA region for `ep` (EP0 and EP4 share one).
fn ep_base(ep: usize) -> *mut u8 {
    let bufs = core::ptr::addr_of_mut!(BUFFERS);
    unsafe {
        match ep {
            0 | 4 => core::ptr::addr_of_mut!((*bufs).ep0_4) as *mut u8,
            1..=3 => core::ptr::addr_of_mut!((*bufs).ep[ep - 1]) as *mut u8,
            _ => core::ptr::addr_of_mut!((*bufs).ep[ep - 2]) as *mut u8,
        }
    }
}

fn mod_bits(ep: usize) -> (bool, bool) {
    let r = pac::USB;
    match ep {
        1 => {
            let m = r.uep4_1_mod().read();
            (m.uep1_rx_en(), m.uep1_tx_en())
        }
        2 => {
            let m = r.uep2_3_mod().read();
            (m.uep2_rx_en(), m.uep2_tx_en())
        }
        3 => {
            let m = r.uep2_3_mod().read();
            (m.uep3_rx_en(), m.uep3_tx_en())
        }
        4 => {
            let m = r.uep4_1_mod().read();
            (m.uep4_rx_en(), m.uep4_tx_en())
        }
        5 => {
            let m = r.uep567_mod().read();
            (m.uep5_rx_en(), m.uep5_tx_en())
        }
        6 => {
            let m = r.uep567_mod().read();
            (m.uep6_rx_en(), m.uep6_tx_en())
        }
        7 => {
            let m = r.uep567_mod().read();
            (m.uep7_rx_en(), m.uep7_tx_en())
        }
        _ => (true, true),
    }
}

/// Where the hardware reads IN data for `ep`. It depends on which
/// directions are enabled (tables 17-3 to 17-5): the regions are packed,
/// so with OUT disabled the IN buffer moves down by 64 bytes.
fn in_buf(ep: usize) -> *mut u8 {
    let (rx, _) = mod_bits(ep);
    let off = match ep {
        0 => 0,
        4 => {
            if rx {
                128
            } else {
                64
            }
        }
        _ => {
            if rx {
                64
            } else {
                0
            }
        }
    };
    unsafe { ep_base(ep).add(off) }
}

/// Where the hardware writes OUT data for `ep`.
fn out_buf(ep: usize) -> *mut u8 {
    unsafe { ep_base(ep).add(if ep == 4 { 64 } else { 0 }) }
}

fn copy_from_buf(src: *const u8, dst: &mut [u8]) {
    for (i, b) in dst.iter_mut().enumerate() {
        *b = unsafe { core::ptr::read_volatile(src.add(i)) };
    }
}

fn copy_to_buf(src: &[u8], dst: *mut u8) {
    for (i, b) in src.iter().enumerate() {
        unsafe { core::ptr::write_volatile(dst.add(i), *b) };
    }
}

/// USB interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = pac::USB;
        let fg = r.int_fg().read();

        if fg.uif_transfer() {
            let st = r.int_st().read();
            let token = st.mask_uis_token();
            if token != TOKEN_IDLE {
                let ep = st.mask_uis_h_res__mask_uis_endp() as usize;
                if ep < EP_COUNT {
                    let bit = 1 << ep;
                    match token {
                        TOKEN_IN => {
                            ep_ctrl(ep).modify(|w| {
                                w.set_mask_uep_t_res(RES_NAK);
                                w.set_uep_t_tog(!w.uep_t_tog());
                            });
                            STATE.in_armed.fetch_and(!bit, Ordering::Relaxed);
                            STATE.in_done.fetch_or(bit, Ordering::Release);
                        }
                        // An OUT with the wrong toggle is a retransmission of
                        // a packet we already have: it is ACKed and dropped.
                        TOKEN_OUT if st.uis_tog_ok() => {
                            STATE.rx_len[ep].store(r.rx_len().read().0 as u32, Ordering::Relaxed);
                            ep_ctrl(ep).modify(|w| {
                                w.set_mask_uep_r_res(RES_NAK);
                                w.set_uep_r_tog(!w.uep_r_tog());
                            });
                            STATE.out_armed.fetch_and(!bit, Ordering::Relaxed);
                            STATE.out_done.fetch_or(bit, Ordering::Release);
                        }
                        _ => {}
                    }
                    STATE.ep_wakers[ep].wake();
                }
                // Clear the token before looking at SETUP (datasheet 17.2.1).
                r.int_fg().write(|w| w.set_uif_transfer(true));
            }

            if st.uis_setup_act() || r.int_st().read().uis_setup_act() {
                let mut setup = [0u8; 8];
                copy_from_buf(ep_base(0), &mut setup);
                // Both data and status stages start with DATA1; the driver
                // arms whichever comes next.
                ep_ctrl(0).write(|w| {
                    w.set_uep_r_tog(true);
                    w.set_uep_t_tog(true);
                    w.set_mask_uep_r_res(RES_NAK);
                    w.set_mask_uep_t_res(RES_NAK);
                });
                STATE.in_armed.fetch_and(!1, Ordering::Relaxed);
                STATE.out_armed.fetch_and(!1, Ordering::Relaxed);
                STATE.in_done.fetch_and(!1, Ordering::Relaxed);
                STATE.out_done.fetch_and(!1, Ordering::Relaxed);
                critical_section::with(|cs| STATE.setup.borrow(cs).set(setup));
                STATE.setup_ready.store(true, Ordering::Release);
                STATE.ep_wakers[0].wake();
                r.int_fg().write(|w| w.set_uif_transfer(true));
            }
        }

        if fg.uif_bus_rst__uif_detect() {
            bus_reset();
            STATE.bus_events.fetch_or(EVENT_RESET, Ordering::Release);
            r.int_fg().write(|w| w.set_uif_bus_rst__uif_detect(true));
            STATE.bus_waker.wake();
            for w in &STATE.ep_wakers {
                w.wake();
            }
        }

        if fg.uif_suspend() {
            // One flag for both edges: MIS_ST says which.
            let (set, clear) = if r.mis_st().read().ums_suspend() {
                (EVENT_SUSPEND, EVENT_RESUME)
            } else {
                (EVENT_RESUME, EVENT_SUSPEND)
            };
            STATE.bus_events.fetch_and(!clear, Ordering::Relaxed);
            STATE.bus_events.fetch_or(set, Ordering::Release);
            r.int_fg().write(|w| w.set_uif_suspend(true));
            STATE.bus_waker.wake();
        }

        if fg.uif_fifo_ov() || fg.uif_hst_sof() {
            r.int_fg().write(|w| {
                w.set_uif_fifo_ov(true);
                w.set_uif_hst_sof(true);
            });
        }
    }
}

/// Back to the default state: address 0, every endpoint but EP0 disabled.
fn bus_reset() {
    let r = pac::USB;
    r.dev_ad().write(|w| w.set_mask_usb_addr(0));
    r.uep4_1_mod().write(|_| {});
    r.uep2_3_mod().write(|_| {});
    r.uep567_mod().write(|_| {});
    for ep in 0..EP_COUNT {
        ep_ctrl(ep).write(|w| {
            w.set_mask_uep_r_res(RES_NAK);
            w.set_mask_uep_t_res(RES_NAK);
        });
    }
    STATE.in_enabled.store(1, Ordering::Relaxed);
    STATE.out_enabled.store(1, Ordering::Relaxed);
    STATE.in_armed.store(0, Ordering::Relaxed);
    STATE.out_armed.store(0, Ordering::Relaxed);
    STATE.in_done.store(0, Ordering::Relaxed);
    STATE.out_done.store(0, Ordering::Relaxed);
    STATE.setup_ready.store(false, Ordering::Relaxed);
}

/// USB device driver.
pub struct Driver<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    in_used: u8,
    out_used: u8,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Create the driver on D+ = PB11, D- = PB10.
    pub fn new(
        _usb: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl DpPin<T>>,
        dm: Peri<'d, impl DmPin<T>>,
    ) -> Self {
        // The PHY takes the pins over once `RB_PIN_USB_IE` is set; keep the
        // GPIO side passive.
        crate::gpio::SealedPin::set_as_af(&*dp, AfType::input(Pull::None));
        crate::gpio::SealedPin::set_as_af(&*dm, AfType::input(Pull::None));
        T::enable();

        Self {
            _phantom: PhantomData,
            in_used: 1,
            out_used: 1,
        }
    }

    fn alloc<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, T, D>, EndpointAllocError> {
        if ep_type == EndpointType::Isochronous || ep_type == EndpointType::Control {
            return Err(EndpointAllocError);
        }
        if max_packet_size > MAX_PACKET_SIZE {
            return Err(EndpointAllocError);
        }
        let used = match D::DIR {
            Direction::In => &mut self.in_used,
            Direction::Out => &mut self.out_used,
        };
        let index = match ep_addr {
            Some(addr) => {
                let i = addr.index();
                if addr.direction() != D::DIR || i == 0 || i >= EP_COUNT || *used & (1 << i) != 0 {
                    return Err(EndpointAllocError);
                }
                i
            }
            None => (1..EP_COUNT)
                .find(|i| *used & (1 << i) == 0)
                .ok_or(EndpointAllocError)?,
        };
        *used |= 1 << index;

        Ok(Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::DIR),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size <= MAX_PACKET_SIZE);
        let r = pac::USB;

        T::Interrupt::disable();
        r.ctrl().write(|w| {
            w.set_uc_clr_all(true);
            w.set_uc_reset_sie(true);
        });
        crate::delay::Delay.delay_us(10);
        r.ctrl().write(|_| {});

        for ep in [0, 1, 2, 3, 5, 6, 7] {
            ep_dma(ep).write_value(pac::usb::regs::Uep0Dma(ep_base(ep) as u16));
        }
        bus_reset();
        STATE.bus_events.store(0, Ordering::Relaxed);

        r.int_fg().write_value(pac::usb::regs::IntFg(0xff));
        r.int_en().write(|w| {
            w.set_uie_bus_rst__uie_detect(true);
            w.set_uie_transfer(true);
            w.set_uie_suspend(true);
        });
        // Device mode with DMA, auto-NAK while UIF_TRANSFER is set. The
        // pull-up stays off until `Bus::enable`.
        r.ctrl().write(|w| {
            w.set_uc_dma_en(true);
            w.set_uc_int_busy(true);
        });

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        (
            Bus {
                _phantom: PhantomData,
                inited: false,
            },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: control_max_packet_size,
            },
        )
    }
}

/// USB bus.
pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    inited: bool,
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn enable(&mut self) {
        let r = pac::USB;
        critical_section::with(|_| pac::SYS.pin_analog_ie().modify(|w| w.set_pin_usb_ie(true)));
        r.udev_ctrl().write(|w| {
            // The pull-downs are for host mode.
            w.set_ud_pd_dis(true);
            w.set_ud_port_en(true);
        });
        r.ctrl().modify(|w| w.set_uc_dev_pu_en(true));
    }

    async fn disable(&mut self) {
        let r = pac::USB;
        r.ctrl().modify(|w| w.set_uc_dev_pu_en(false));
        r.udev_ctrl().write(|w| w.set_ud_pd_dis(true));
        critical_section::with(|_| pac::SYS.pin_analog_ie().modify(|w| w.set_pin_usb_ie(false)));
    }

    async fn poll(&mut self) -> Event {
        // No VBUS sense on this chip: assume power.
        if !self.inited {
            self.inited = true;
            return Event::PowerDetected;
        }

        poll_fn(|cx| {
            STATE.bus_waker.register(cx.waker());
            let ev = STATE.bus_events.load(Ordering::Acquire);
            if ev & EVENT_RESET != 0 {
                STATE.bus_events.fetch_and(!EVENT_RESET, Ordering::Relaxed);
                Poll::Ready(Event::Reset)
            } else if ev & EVENT_SUSPEND != 0 {
                STATE.bus_events.fetch_and(!EVENT_SUSPEND, Ordering::Relaxed);
                Poll::Ready(Event::Suspend)
            } else if ev & EVENT_RESUME != 0 {
                STATE.bus_events.fetch_and(!EVENT_RESUME, Ordering::Relaxed);
                Poll::Ready(Event::Resume)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let ep = ep_addr.index();
        if ep == 0 || ep >= EP_COUNT {
            return;
        }
        let r = pac::USB;
        let bit = 1 << ep;
        let out = ep_addr.direction() == Direction::Out;

        critical_section::with(|_| {
            match (ep, out) {
                (1, true) => r.uep4_1_mod().modify(|w| w.set_uep1_rx_en(enabled)),
                (1, false) => r.uep4_1_mod().modify(|w| w.set_uep1_tx_en(enabled)),
                (2, true) => r.uep2_3_mod().modify(|w| w.set_uep2_rx_en(enabled)),
                (2, false) => r.uep2_3_mod().modify(|w| w.set_uep2_tx_en(enabled)),
                (3, true) => r.uep2_3_mod().modify(|w| w.set_uep3_rx_en(enabled)),
                (3, false) => r.uep2_3_mod().modify(|w| w.set_uep3_tx_en(enabled)),
                (4, true) => r.uep4_1_mod().modify(|w| w.set_uep4_rx_en(enabled)),
                (4, false) => r.uep4_1_mod().modify(|w| w.set_uep4_tx_en(enabled)),
                (5, true) => r.uep567_mod().modify(|w| w.set_uep5_rx_en(enabled)),
                (5, false) => r.uep567_mod().modify(|w| w.set_uep5_tx_en(enabled)),
                (6, true) => r.uep567_mod().modify(|w| w.set_uep6_rx_en(enabled)),
                (6, false) => r.uep567_mod().modify(|w| w.set_uep6_tx_en(enabled)),
                (7, true) => r.uep567_mod().modify(|w| w.set_uep7_rx_en(enabled)),
                (7, false) => r.uep567_mod().modify(|w| w.set_uep7_tx_en(enabled)),
                _ => unreachable!(),
            }
            // (Re)configuring an endpoint resets its toggle to DATA0.
            ep_ctrl(ep).modify(|w| {
                if out {
                    w.set_mask_uep_r_res(RES_NAK);
                    w.set_uep_r_tog(false);
                } else {
                    w.set_mask_uep_t_res(RES_NAK);
                    w.set_uep_t_tog(false);
                }
            });
            let (en, armed) = if out {
                (&STATE.out_enabled, &STATE.out_armed)
            } else {
                (&STATE.in_enabled, &STATE.in_armed)
            };
            armed.fetch_and(!bit, Ordering::Relaxed);
            if enabled {
                en.fetch_or(bit, Ordering::Relaxed);
            } else {
                en.fetch_and(!bit, Ordering::Relaxed);
            }
        });
        STATE.ep_wakers[ep].wake();
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let ep = ep_addr.index();
        if ep >= EP_COUNT {
            return;
        }
        let bit = 1 << ep;
        let out = ep_addr.direction() == Direction::Out;
        critical_section::with(|_| {
            let armed = if out { &STATE.out_armed } else { &STATE.in_armed };
            // Clearing a halt resets the toggle and restores a transfer
            // that was armed before the stall.
            let res = if stalled {
                RES_STALL
            } else if armed.load(Ordering::Relaxed) & bit != 0 {
                RES_ACK
            } else {
                RES_NAK
            };
            ep_ctrl(ep).modify(|w| {
                if out {
                    w.set_mask_uep_r_res(res);
                    if !stalled {
                        w.set_uep_r_tog(false);
                    }
                } else {
                    w.set_mask_uep_t_res(res);
                    if !stalled {
                        w.set_uep_t_tog(false);
                    }
                }
            });
        });
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let ep = ep_addr.index();
        if ep >= EP_COUNT {
            return false;
        }
        let c = ep_ctrl(ep).read();
        let res = match ep_addr.direction() {
            Direction::Out => c.mask_uep_r_res(),
            Direction::In => c.mask_uep_t_res(),
        };
        res == RES_STALL
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        // WCH's SDK signals resume (K state) by briefly switching the port
        // to low speed, which swaps the pull-up over to D-.
        let r = pac::USB;
        r.udev_ctrl().modify(|w| w.set_ud_low_speed(true));
        crate::delay::Delay.delay_ms(2);
        r.udev_ctrl().modify(|w| w.set_ud_low_speed(false));
        Ok(())
    }
}

/// Endpoint direction marker.
#[allow(private_bounds)]
pub trait Dir: SealedDir {}
trait SealedDir {
    const DIR: Direction;
}

/// Marker for IN endpoints.
pub enum In {}
impl SealedDir for In {
    const DIR: Direction = Direction::In;
}
impl Dir for In {}

/// Marker for OUT endpoints.
pub enum Out {}
impl SealedDir for Out {
    const DIR: Direction = Direction::Out;
}
impl Dir for Out {}

/// USB endpoint.
pub struct Endpoint<'d, T: Instance, D: Dir> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
}

impl<'d, T: Instance, D: Dir> Endpoint<'d, T, D> {
    fn index(&self) -> usize {
        self.info.addr.index()
    }

    fn is_enabled(&self) -> bool {
        let en = match D::DIR {
            Direction::In => &STATE.in_enabled,
            Direction::Out => &STATE.out_enabled,
        };
        en.load(Ordering::Relaxed) & (1 << self.index()) != 0
    }
}

impl<'d, T: Instance, D: Dir> embassy_usb_driver::Endpoint for Endpoint<'d, T, D> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        poll_fn(|cx| {
            STATE.ep_wakers[self.index()].register(cx.waker());
            if self.is_enabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        let ep = self.index();
        let bit = 1 << ep;
        critical_section::with(|_| {
            if !self.is_enabled() {
                return Err(EndpointError::Disabled);
            }
            copy_to_buf(buf, in_buf(ep));
            ep_t_len(ep).write(|w| w.set_uep0_t_len(buf.len() as u8));
            STATE.in_done.fetch_and(!bit, Ordering::Relaxed);
            STATE.in_armed.fetch_or(bit, Ordering::Relaxed);
            ep_ctrl(ep).modify(|w| {
                // A stalled endpoint keeps its stall; the transfer goes out
                // once the halt is cleared.
                if w.mask_uep_t_res() != RES_STALL {
                    w.set_mask_uep_t_res(RES_ACK);
                }
            });
            Ok(())
        })?;

        poll_fn(|cx| {
            STATE.ep_wakers[ep].register(cx.waker());
            if STATE.in_done.load(Ordering::Acquire) & bit != 0 {
                Poll::Ready(Ok(()))
            } else if !self.is_enabled() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let ep = self.index();
        let bit = 1 << ep;
        critical_section::with(|_| {
            if !self.is_enabled() {
                return Err(EndpointError::Disabled);
            }
            STATE.out_done.fetch_and(!bit, Ordering::Relaxed);
            STATE.out_armed.fetch_or(bit, Ordering::Relaxed);
            ep_ctrl(ep).modify(|w| {
                if w.mask_uep_r_res() != RES_STALL {
                    w.set_mask_uep_r_res(RES_ACK);
                }
            });
            Ok(())
        })?;

        poll_fn(|cx| {
            STATE.ep_wakers[ep].register(cx.waker());
            if STATE.out_done.load(Ordering::Acquire) & bit != 0 {
                let len = STATE.rx_len[ep].load(Ordering::Relaxed) as usize;
                if len > buf.len() {
                    return Poll::Ready(Err(EndpointError::BufferOverflow));
                }
                copy_from_buf(out_buf(ep), &mut buf[..len]);
                Poll::Ready(Ok(len))
            } else if !self.is_enabled() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

/// USB control pipe (EP0).
pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

impl<'d, T: Instance> ControlPipe<'d, T> {
    /// Arm EP0 for one IN packet (`Some`) or one OUT packet (`None`) and wait
    /// for it. A new SETUP aborts the wait.
    async fn transfer(&mut self, data: Option<&[u8]>) -> Result<(), EndpointError> {
        let (done, armed) = if data.is_some() {
            (&STATE.in_done, &STATE.in_armed)
        } else {
            (&STATE.out_done, &STATE.out_armed)
        };
        critical_section::with(|_| {
            if STATE.setup_ready.load(Ordering::Relaxed) {
                return Err(EndpointError::Disabled);
            }
            done.fetch_and(!1, Ordering::Relaxed);
            armed.fetch_or(1, Ordering::Relaxed);
            match data {
                Some(data) => {
                    copy_to_buf(data, ep_base(0));
                    ep_t_len(0).write(|w| w.set_uep0_t_len(data.len() as u8));
                    ep_ctrl(0).modify(|w| w.set_mask_uep_t_res(RES_ACK));
                }
                None => ep_ctrl(0).modify(|w| w.set_mask_uep_r_res(RES_ACK)),
            }
            Ok(())
        })?;

        poll_fn(|cx| {
            STATE.ep_wakers[0].register(cx.waker());
            if STATE.setup_ready.load(Ordering::Acquire) {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if done.load(Ordering::Acquire) & 1 != 0 {
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance> embassy_usb_driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            STATE.ep_wakers[0].register(cx.waker());
            critical_section::with(|cs| {
                if STATE.setup_ready.load(Ordering::Acquire) {
                    STATE.setup_ready.store(false, Ordering::Relaxed);
                    Poll::Ready(STATE.setup.borrow(cs).get())
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        // The toggle starts at DATA1 after SETUP and the handler flips it per
        // packet.
        self.transfer(None).await?;
        let len = STATE.rx_len[0].load(Ordering::Relaxed) as usize;
        if len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        copy_from_buf(ep_base(0), &mut buf[..len]);
        Ok(len)
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > self.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        self.transfer(Some(data)).await?;
        if last {
            // ACK the zero-length status OUT (DATA1) whenever it comes; the
            // handler NAKs EP0 again after it.
            critical_section::with(|_| {
                if !STATE.setup_ready.load(Ordering::Relaxed) {
                    STATE.out_done.fetch_and(!1, Ordering::Relaxed);
                    STATE.out_armed.fetch_or(1, Ordering::Relaxed);
                    ep_ctrl(0).modify(|w| {
                        w.set_uep_r_tog(true);
                        w.set_mask_uep_r_res(RES_ACK);
                    });
                }
            });
        }
        Ok(())
    }

    async fn accept(&mut self) {
        // Zero-length status IN, always DATA1.
        critical_section::with(|_| ep_ctrl(0).modify(|w| w.set_uep_t_tog(true)));
        let _ = self.transfer(Some(&[])).await;
    }

    async fn reject(&mut self) {
        // The next SETUP clears the stall.
        ep_ctrl(0).modify(|w| {
            w.set_mask_uep_r_res(RES_STALL);
            w.set_mask_uep_t_res(RES_STALL);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        // The new address applies after the status stage.
        self.accept().await;
        pac::USB.dev_ad().modify(|w| w.set_mask_usb_addr(addr));
    }
}

trait SealedInstance {
    /// Ungate the clock.
    fn enable();
}

/// USB peripheral instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + embassy_hal_internal::PeripheralType + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl SealedInstance for peripherals::USB {
    fn enable() {
        set_clock_enabled(ClockGate::Usb, true);
    }
}

impl Instance for peripherals::USB {
    type Interrupt = crate::interrupt::typelevel::USB;
}

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

pin_trait_impl!(crate::usb::DpPin, USB, PB11, 0);
pin_trait_impl!(crate::usb::DmPin, USB, PB10, 0);
