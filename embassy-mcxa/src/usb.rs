//! USB full-speed device driver for the `USB0` controller.
//!
//! The controller locates each endpoint's transfers through a Buffer Descriptor
//! Table (BDT) in system RAM. Every endpoint direction has an "even" and "odd"
//! buffer descriptor (BD) and the controller alternates between them after
//! each completed transaction. This driver keeps at most one BD armed per
//! endpoint direction and tracks which bank the controller will use next.
//!
//! Packets are staged through per-endpoint buffers owned by the driver, so a
//! transfer future can be dropped while the controller still owns a BD.
//!
//! Endpoint 0 OUT is armed without data toggle synchronization so that it can
//! always accept a SETUP packet (DATA0) as well as data/status OUT packets
//! (DATA1). After a SETUP token the controller sets `CTL[TXSUSPENDTOKENBUSY]`
//! and NAKs further tokens until software clears it in [`ControlPipe::setup`].

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering, compiler_fence};
use core::task::Poll;

use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver as driver;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};

use crate::clocks::periph_helpers::{Div4, UsbClockSel, UsbConfig};
use crate::clocks::{ClockError, Gate, PoweredClock, WakeGuard, enable_and_reset};
use crate::interrupt::typelevel;
use crate::interrupt::typelevel::{Handler, Interrupt};
use crate::pac;
use crate::pac::usb::{
    Bdtpage1, Bdtpage2, Bdtpage3, Dppullupnonotg, Endpt, Epctldis, Errstat, Istat, Odd, Pde, Susp, Tx, Usbensofen,
    Usbreset,
};

const EP_COUNT: usize = 16;
const EP_MEMORY_SIZE: usize = 1024;
const MAX_PACKET_SIZE: u16 = 64;

// ISTAT/INTEN bit positions (identical in both registers).
const INT_USBRST: u8 = 1 << 0;
const INT_ERROR: u8 = 1 << 1;
const INT_TOKDNE: u8 = 1 << 3;
const INT_SLEEP: u8 = 1 << 4;
const INT_RESUME: u8 = 1 << 5;
const INT_STALL: u8 = 1 << 7;

// Buffer descriptor control word. Bits 2..=5 hold the token PID when the
// controller hands the BD back to software.
const BD_OWN: u32 = 1 << 7;
const BD_DATA1: u32 = 1 << 6;
const BD_DTS: u32 = 1 << 3;
const BD_STALL: u32 = 1 << 2;
const BD_BC_SHIFT: u32 = 16;
const BD_BC_MASK: u32 = 0x3ff;

const PID_SETUP: u32 = 0x0d;

const EVENT_RESET: u8 = 1 << 0;
const EVENT_SUSPEND: u8 = 1 << 1;
const EVENT_RESUME: u8 = 1 << 2;

#[repr(C, align(512))]
struct Bdt(UnsafeCell<[[u32; 2]; EP_COUNT * 4]>);

// SAFETY: every access goes through volatile reads/writes of single words.
unsafe impl Sync for Bdt {}

static BDT: Bdt = Bdt(UnsafeCell::new([[0; 2]; EP_COUNT * 4]));

#[repr(C, align(4))]
struct EpMemory(UnsafeCell<[u8; EP_MEMORY_SIZE]>);

// SAFETY: each region of the buffer belongs to exactly one endpoint direction,
// and is only touched while the controller does not own that endpoint's BD.
unsafe impl Sync for EpMemory {}

static EP_MEMORY: EpMemory = EpMemory(UnsafeCell::new([0; EP_MEMORY_SIZE]));

struct EpState {
    waker: AtomicWaker,
    /// Address of this endpoint's packet buffer in [`EP_MEMORY`].
    buf: AtomicU32,
    max_packet_size: AtomicU16,
    /// Length of the last completed transaction.
    len: AtomicU16,
    /// Bank (even/odd BD) the controller will use for the next transaction.
    odd: AtomicBool,
    /// Data toggle for the next transaction.
    data1: AtomicBool,
    /// A BD is owned by the controller.
    armed: AtomicBool,
    /// A transaction completed and has not been consumed yet.
    done: AtomicBool,
    stalled: AtomicBool,
    enabled: AtomicBool,
}

impl EpState {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            buf: AtomicU32::new(0),
            max_packet_size: AtomicU16::new(0),
            len: AtomicU16::new(0),
            odd: AtomicBool::new(false),
            data1: AtomicBool::new(false),
            armed: AtomicBool::new(false),
            done: AtomicBool::new(false),
            stalled: AtomicBool::new(false),
            enabled: AtomicBool::new(false),
        }
    }

    fn buf(&self) -> *mut u8 {
        self.buf.load(Ordering::Relaxed) as *mut u8
    }
}

static EP_IN: [EpState; EP_COUNT] = [const { EpState::new() }; EP_COUNT];
static EP_OUT: [EpState; EP_COUNT] = [const { EpState::new() }; EP_COUNT];

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static BUS_EVENTS: AtomicU8 = AtomicU8::new(0);

static SETUP_READY: AtomicBool = AtomicBool::new(false);
static SETUP_LO: AtomicU32 = AtomicU32::new(0);
static SETUP_HI: AtomicU32 = AtomicU32::new(0);

fn ep_state(index: usize, dir: Direction) -> &'static EpState {
    match dir {
        Direction::In => &EP_IN[index],
        Direction::Out => &EP_OUT[index],
    }
}

fn bd_index(index: usize, dir: Direction, odd: bool) -> usize {
    index * 4 + (matches!(dir, Direction::In) as usize) * 2 + odd as usize
}

fn bd_ptr(index: usize, dir: Direction, odd: bool) -> *mut [u32; 2] {
    // SAFETY: bd_index is always in range for EP_COUNT endpoints.
    unsafe { (BDT.0.get() as *mut [u32; 2]).add(bd_index(index, dir, odd)) }
}

fn bd_read_ctrl(index: usize, dir: Direction, odd: bool) -> u32 {
    // SAFETY: the pointer is valid and aligned; the controller may write this
    // word concurrently, so it is accessed volatilely.
    unsafe { core::ptr::read_volatile(bd_ptr(index, dir, odd) as *const u32) }
}

fn bd_write(index: usize, dir: Direction, odd: bool, ctrl: u32, addr: u32) {
    let bd = bd_ptr(index, dir, odd) as *mut u32;
    // SAFETY: the pointer is valid and aligned. The address word is written
    // before the control word so the controller never sees OWN with a stale
    // buffer address.
    unsafe {
        core::ptr::write_volatile(bd.add(1), addr);
        compiler_fence(Ordering::SeqCst);
        cortex_m::asm::dsb();
        core::ptr::write_volatile(bd, ctrl);
    }
}

/// Hand the next BD of an endpoint direction to the controller.
///
/// For OUT endpoints `len` is the buffer capacity, for IN endpoints it is the
/// number of bytes to send.
fn arm(index: usize, dir: Direction, len: usize) {
    let st = ep_state(index, dir);
    let mut ctrl = BD_OWN | ((len as u32 & BD_BC_MASK) << BD_BC_SHIFT);
    if !(index == 0 && matches!(dir, Direction::Out)) {
        ctrl |= BD_DTS;
        if st.data1.load(Ordering::Relaxed) {
            ctrl |= BD_DATA1;
        }
    }
    st.done.store(false, Ordering::Relaxed);
    st.armed.store(true, Ordering::Relaxed);
    bd_write(index, dir, st.odd.load(Ordering::Relaxed), ctrl, st.buf.load(Ordering::Relaxed));
}

fn arm_ep0_out() {
    arm(0, Direction::Out, EP_OUT[0].max_packet_size.load(Ordering::Relaxed) as usize);
}

fn clear_bds(index: usize, dir: Direction) {
    bd_write(index, dir, false, 0, 0);
    bd_write(index, dir, true, 0, 0);
}

fn wake_all() {
    for st in EP_IN.iter().chain(EP_OUT.iter()) {
        st.waker.wake();
    }
}

fn bus_reset(regs: pac::usb::Usb) {
    regs.addr().write(|w| w.set_addr(0));
    regs.ctl().modify(|w| w.set_oddrst(true));

    for index in 0..EP_COUNT {
        if index != 0 {
            regs.endpoint(index).endpt().write_value(Endpt(0));
        }
        for dir in [Direction::Out, Direction::In] {
            let st = ep_state(index, dir);
            clear_bds(index, dir);
            st.odd.store(false, Ordering::Relaxed);
            st.data1.store(false, Ordering::Relaxed);
            st.armed.store(false, Ordering::Relaxed);
            st.done.store(false, Ordering::Relaxed);
            st.stalled.store(false, Ordering::Relaxed);
            st.enabled.store(index == 0, Ordering::Relaxed);
        }
    }

    regs.ctl().modify(|w| w.set_oddrst(false));
    regs.endpoint(0).endpt().write(|w| {
        w.set_ephshk(true);
        w.set_eptxen(true);
        w.set_eprxen(true);
    });

    SETUP_READY.store(false, Ordering::Relaxed);
    arm_ep0_out();
    wake_all();
}

fn handle_token(regs: pac::usb::Usb) {
    let stat = regs.stat().read();
    let index = stat.endp() as usize;
    let dir = match stat.tx() {
        Tx::TxTransaction => Direction::In,
        Tx::RxTransaction => Direction::Out,
    };
    let odd = stat.odd() == Odd::OddBank;
    let ctrl = bd_read_ctrl(index, dir, odd);
    let pid = (ctrl >> 2) & 0xf;
    let len = ((ctrl >> BD_BC_SHIFT) & BD_BC_MASK) as u16;

    #[cfg(feature = "defmt")]
    defmt::trace!(
        "usb: token ep{} {} odd={} pid={:#x} len={}",
        index,
        matches!(dir, Direction::In),
        odd,
        pid,
        len
    );

    let st = ep_state(index, dir);
    st.odd.store(!odd, Ordering::Relaxed);
    st.armed.store(false, Ordering::Relaxed);

    if index == 0 && matches!(dir, Direction::Out) {
        if pid == PID_SETUP {
            // SAFETY: the BD pointed at this endpoint's buffer, which holds at
            // least 8 bytes, and the controller has released it.
            let words = unsafe { core::ptr::read_unaligned(st.buf() as *const [u32; 2]) };
            SETUP_LO.store(words[0], Ordering::Relaxed);
            SETUP_HI.store(words[1], Ordering::Relaxed);

            // A SETUP aborts any control transfer still in progress. The
            // data or status stage that follows always starts with DATA1.
            let ep0_in = &EP_IN[0];
            clear_bds(0, Direction::In);
            ep0_in.armed.store(false, Ordering::Relaxed);
            ep0_in.done.store(false, Ordering::Relaxed);
            ep0_in.data1.store(true, Ordering::Relaxed);
            ep0_in.waker.wake();

            st.done.store(false, Ordering::Relaxed);
            arm_ep0_out();
            SETUP_READY.store(true, Ordering::Release);
        } else if len == 0 {
            // Status stage of an IN control transfer.
            arm_ep0_out();
        } else {
            // Left unarmed until `data_out` copies the packet out.
            st.len.store(len, Ordering::Relaxed);
            st.done.store(true, Ordering::Release);
        }
    } else {
        st.data1.store(!st.data1.load(Ordering::Relaxed), Ordering::Relaxed);
        st.len.store(len, Ordering::Relaxed);
        st.done.store(true, Ordering::Release);
    }
    st.waker.wake();
}

/// USB0 interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let istat = regs.istat().read().0;

        if istat & INT_USBRST != 0 {
            bus_reset(regs);
            regs.istat().write_value(Istat(0xff));
            BUS_EVENTS.fetch_or(EVENT_RESET, Ordering::Release);
            BUS_WAKER.wake();
            return;
        }

        if istat & INT_ERROR != 0 {
            #[cfg(feature = "defmt")]
            defmt::trace!("usb: error {:#x}", regs.errstat().read().0);
            regs.errstat().write_value(Errstat(0xff));
        }

        // STAT is a four-entry FIFO that advances when TOKDNE is cleared.
        while regs.istat().read().0 & INT_TOKDNE != 0 {
            handle_token(regs);
            regs.istat().write_value(Istat(INT_TOKDNE));
        }

        if istat & INT_SLEEP != 0 {
            regs.inten().modify(|w| w.0 |= INT_RESUME);
            BUS_EVENTS.fetch_or(EVENT_SUSPEND, Ordering::Release);
            BUS_WAKER.wake();
        }

        if istat & INT_RESUME != 0 {
            regs.inten().modify(|w| w.0 &= !INT_RESUME);
            BUS_EVENTS.fetch_or(EVENT_RESUME, Ordering::Release);
            BUS_WAKER.wake();
        }

        regs.istat().write_value(Istat(istat & !INT_TOKDNE));
    }
}

/// USB driver configuration.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Config {
    /// Source of the 48MHz USB functional clock.
    pub clock_source: UsbClockSel,
    /// Divider applied to `clock_source`.
    pub clock_div: Div4,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: UsbClockSel::Pll1Clk,
            clock_div: Div4::no_div(),
        }
    }
}

/// USB full-speed device driver.
pub struct Driver<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    ep_mem_used: usize,
    in_used: u16,
    out_used: u16,
    wake_guard: Option<WakeGuard>,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Create a new USB driver.
    ///
    /// The device is bus powered as far as the driver is concerned: it reports
    /// [`Event::PowerDetected`] immediately rather than monitoring VBUS.
    pub fn new(
        _peri: Peri<'d, T>,
        _irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, ClockError> {
        let parts = unsafe {
            enable_and_reset::<T>(&UsbConfig {
                power: PoweredClock::NormalEnabledDeepSleepDisabled,
                source: config.clock_source,
                div: config.clock_div,
            })?
        };

        let regs = T::regs();

        regs.usbtrc0().modify(|w| w.set_usbreset(Usbreset::ForceHardReset));
        while regs.usbtrc0().read().usbreset() == Usbreset::ForceHardReset {}

        let bdt = BDT.0.get() as u32;
        regs.bdtpage1().write_value(Bdtpage1((bdt >> 8) as u8));
        regs.bdtpage2().write_value(Bdtpage2((bdt >> 16) as u8));
        regs.bdtpage3().write_value(Bdtpage3((bdt >> 24) as u8));

        for index in 0..EP_COUNT {
            regs.endpoint(index).endpt().write_value(Endpt(0));
            clear_bds(index, Direction::Out);
            clear_bds(index, Direction::In);
        }

        regs.usbctrl().write(|w| {
            w.set_susp(Susp::XcvrNotSuspend);
            w.set_pde(Pde::EnPulldowns);
        });
        regs.ctl().write(|w| w.set_usbensofen(Usbensofen::EnUsbSof));

        regs.istat().write_value(Istat(0xff));
        regs.errstat().write_value(Errstat(0xff));

        Ok(Self {
            _phantom: PhantomData,
            ep_mem_used: 0,
            in_used: 0,
            out_used: 0,
            wake_guard: parts.wake_guard,
        })
    }

    fn alloc_buffer(&mut self, max_packet_size: u16) -> Result<u32, EndpointAllocError> {
        let size = (max_packet_size as usize).next_multiple_of(4);
        if self.ep_mem_used + size > EP_MEMORY_SIZE {
            return Err(EndpointAllocError);
        }
        let addr = EP_MEMORY.0.get() as u32 + self.ep_mem_used as u32;
        self.ep_mem_used += size;
        Ok(addr)
    }

    fn alloc_endpoint(
        &mut self,
        dir: Direction,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<EndpointInfo, EndpointAllocError> {
        if max_packet_size > MAX_PACKET_SIZE || ep_type == EndpointType::Isochronous {
            return Err(EndpointAllocError);
        }

        let used = match dir {
            Direction::In => self.in_used,
            Direction::Out => self.out_used,
        };
        let index = match ep_addr {
            Some(addr) if addr.index() == 0 || addr.index() >= EP_COUNT || used & (1 << addr.index()) != 0 => {
                return Err(EndpointAllocError);
            }
            Some(addr) => addr.index(),
            None => (1..EP_COUNT).find(|i| used & (1 << i) == 0).ok_or(EndpointAllocError)?,
        };

        let buf = self.alloc_buffer(max_packet_size)?;
        let st = ep_state(index, dir);
        st.buf.store(buf, Ordering::Relaxed);
        st.max_packet_size.store(max_packet_size, Ordering::Relaxed);

        match dir {
            Direction::In => self.in_used |= 1 << index,
            Direction::Out => self.out_used |= 1 << index,
        }

        Ok(EndpointInfo {
            addr: EndpointAddress::from_parts(index, dir),
            ep_type,
            max_packet_size,
            interval_ms,
        })
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = EndpointOut<'d, T>;
    type EndpointIn = EndpointIn<'d, T>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        Ok(EndpointOut {
            info: self.alloc_endpoint(Direction::Out, ep_type, ep_addr, max_packet_size, interval_ms)?,
            _phantom: PhantomData,
        })
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        Ok(EndpointIn {
            info: self.alloc_endpoint(Direction::In, ep_type, ep_addr, max_packet_size, interval_ms)?,
            _phantom: PhantomData,
        })
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        // embassy-usb only asks for 8, 16, 32 or 64 bytes, which all fit.
        let mps = control_max_packet_size.min(MAX_PACKET_SIZE);
        for dir in [Direction::Out, Direction::In] {
            let buf = self
                .alloc_buffer(mps.max(8))
                .expect("EP0 buffers are allocated before any other endpoint memory is exhausted");
            let st = ep_state(0, dir);
            st.buf.store(buf, Ordering::Relaxed);
            st.max_packet_size.store(mps, Ordering::Relaxed);
        }

        let regs = T::regs();
        critical_section::with(|_| bus_reset(regs));
        regs.inten()
            .write_value(pac::usb::Inten(INT_USBRST | INT_ERROR | INT_TOKDNE | INT_SLEEP | INT_STALL));
        regs.erren().write_value(pac::usb::Erren(0xff));

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        (
            Bus {
                _phantom: PhantomData,
                power_reported: false,
                _wake_guard: self.wake_guard,
            },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: mps,
            },
        )
    }
}

/// USB bus.
pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    power_reported: bool,
    _wake_guard: Option<WakeGuard>,
}

impl<'d, T: Instance> driver::Bus for Bus<'d, T> {
    async fn enable(&mut self) {
        T::regs()
            .control()
            .write(|w| w.set_dppullupnonotg(Dppullupnonotg::EnDeviceDpPu));
    }

    async fn disable(&mut self) {
        T::regs()
            .control()
            .write(|w| w.set_dppullupnonotg(Dppullupnonotg::DisDeviceDpPu));
    }

    async fn poll(&mut self) -> Event {
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            if !self.power_reported {
                self.power_reported = true;
                return Poll::Ready(Event::PowerDetected);
            }

            let events = BUS_EVENTS.load(Ordering::Acquire);
            for (flag, event) in [
                (EVENT_RESET, Event::Reset),
                (EVENT_SUSPEND, Event::Suspend),
                (EVENT_RESUME, Event::Resume),
            ] {
                if events & flag != 0 {
                    BUS_EVENTS.fetch_and(!flag, Ordering::AcqRel);
                    return Poll::Ready(event);
                }
            }

            Poll::Pending
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let index = ep_addr.index();
        if index == 0 {
            return;
        }
        let dir = ep_addr.direction();
        let st = ep_state(index, dir);
        let regs = T::regs();

        critical_section::with(|_| {
            if enabled {
                st.data1.store(false, Ordering::Relaxed);
                st.stalled.store(false, Ordering::Relaxed);
                st.armed.store(false, Ordering::Relaxed);
                st.done.store(false, Ordering::Relaxed);
            } else {
                // The bank pointer only advances on completed transactions, so
                // `odd` stays in sync when an armed BD is withdrawn here.
                clear_bds(index, dir);
                st.armed.store(false, Ordering::Relaxed);
                st.done.store(false, Ordering::Relaxed);
            }

            regs.endpoint(index).endpt().modify(|w| {
                w.set_ephshk(true);
                w.set_epctldis(Epctldis::Disable);
                match dir {
                    Direction::In => w.set_eptxen(enabled),
                    Direction::Out => w.set_eprxen(enabled),
                }
            });

            st.enabled.store(enabled, Ordering::Release);
        });
        st.waker.wake();
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let index = ep_addr.index();
        let dir = ep_addr.direction();
        let st = ep_state(index, dir);
        let regs = T::regs();

        critical_section::with(|_| {
            let odd = st.odd.load(Ordering::Relaxed);
            if stalled {
                bd_write(index, dir, odd, BD_OWN | BD_STALL, st.buf.load(Ordering::Relaxed));
                st.armed.store(true, Ordering::Relaxed);
            } else {
                bd_write(index, dir, odd, 0, 0);
                st.armed.store(false, Ordering::Relaxed);
                st.data1.store(false, Ordering::Relaxed);
                regs.endpoint(index).endpt().modify(|w| w.set_epstall(false));
            }
            st.stalled.store(stalled, Ordering::Release);
        });
        st.waker.wake();
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        ep_state(ep_addr.index(), ep_addr.direction())
            .stalled
            .load(Ordering::Acquire)
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

async fn wait_enabled(st: &EpState) {
    poll_fn(|cx| {
        st.waker.register(cx.waker());
        if st.enabled.load(Ordering::Acquire) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await
}

/// USB OUT endpoint.
pub struct EndpointOut<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    info: EndpointInfo,
}

impl<'d, T: Instance> driver::Endpoint for EndpointOut<'d, T> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        wait_enabled(&EP_OUT[self.info.addr.index()]).await
    }
}

impl<'d, T: Instance> driver::EndpointOut for EndpointOut<'d, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();
        let st = &EP_OUT[index];
        let mps = self.info.max_packet_size as usize;

        poll_fn(|cx| {
            st.waker.register(cx.waker());
            critical_section::with(|_| {
                if !st.enabled.load(Ordering::Acquire) {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if st.done.load(Ordering::Acquire) {
                    st.done.store(false, Ordering::Relaxed);
                    let len = st.len.load(Ordering::Relaxed) as usize;
                    if len > buf.len() {
                        return Poll::Ready(Err(EndpointError::BufferOverflow));
                    }
                    // SAFETY: the controller released this buffer and nothing
                    // re-arms it until the copy is done.
                    unsafe { core::ptr::copy_nonoverlapping(st.buf(), buf.as_mut_ptr(), len) };
                    return Poll::Ready(Ok(len));
                }
                if !st.armed.load(Ordering::Relaxed) && !st.stalled.load(Ordering::Relaxed) {
                    arm(index, Direction::Out, mps);
                }
                Poll::Pending
            })
        })
        .await
    }
}

/// USB IN endpoint.
pub struct EndpointIn<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    info: EndpointInfo,
}

impl<'d, T: Instance> driver::Endpoint for EndpointIn<'d, T> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        wait_enabled(&EP_IN[self.info.addr.index()]).await
    }
}

impl<'d, T: Instance> driver::EndpointIn for EndpointIn<'d, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        let index = self.info.addr.index();
        let st = &EP_IN[index];

        // Wait for any previous (possibly abandoned) transaction to finish.
        poll_fn(|cx| {
            st.waker.register(cx.waker());
            critical_section::with(|_| {
                if !st.enabled.load(Ordering::Acquire) {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if st.armed.load(Ordering::Relaxed) || st.stalled.load(Ordering::Relaxed) {
                    return Poll::Pending;
                }
                // SAFETY: the controller does not own this endpoint's buffer.
                unsafe { core::ptr::copy_nonoverlapping(buf.as_ptr(), st.buf(), buf.len()) };
                arm(index, Direction::In, buf.len());
                Poll::Ready(Ok(()))
            })
        })
        .await?;

        poll_fn(|cx| {
            st.waker.register(cx.waker());
            if !st.enabled.load(Ordering::Acquire) {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if st.done.swap(false, Ordering::AcqRel) {
                return Poll::Ready(Ok(()));
            }
            Poll::Pending
        })
        .await
    }
}

/// USB control pipe (endpoint 0).
pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

impl<'d, T: Instance> ControlPipe<'d, T> {
    /// Send a packet on EP0 IN and wait until the host has collected it.
    async fn send(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let st = &EP_IN[0];
        critical_section::with(|_| {
            // SAFETY: EP0 IN is only armed from this pipe, and never while a
            // previous packet is still owned by the controller.
            unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), st.buf(), data.len()) };
            arm(0, Direction::In, data.len());
        });

        poll_fn(|cx| {
            st.waker.register(cx.waker());
            if st.done.swap(false, Ordering::AcqRel) {
                return Poll::Ready(Ok(()));
            }
            if !st.armed.load(Ordering::Acquire) {
                // Withdrawn by a new SETUP or a bus reset.
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            Poll::Pending
        })
        .await
    }
}

impl<'d, T: Instance> driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        let setup = poll_fn(|cx| {
            EP_OUT[0].waker.register(cx.waker());
            if SETUP_READY.swap(false, Ordering::AcqRel) {
                let lo = SETUP_LO.load(Ordering::Relaxed).to_ne_bytes();
                let hi = SETUP_HI.load(Ordering::Relaxed).to_ne_bytes();
                let mut setup = [0; 8];
                setup[..4].copy_from_slice(&lo);
                setup[4..].copy_from_slice(&hi);
                Poll::Ready(setup)
            } else {
                Poll::Pending
            }
        })
        .await;

        // EP0 OUT was re-armed when the SETUP arrived, so the controller can
        // resume token processing.
        T::regs().ctl().modify(|w| w.set_txsuspendtokenbusy(false));
        setup
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        let st = &EP_OUT[0];
        poll_fn(|cx| {
            st.waker.register(cx.waker());
            critical_section::with(|_| {
                if SETUP_READY.load(Ordering::Acquire) {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if !st.done.load(Ordering::Acquire) {
                    return Poll::Pending;
                }
                st.done.store(false, Ordering::Relaxed);
                let len = st.len.load(Ordering::Relaxed) as usize;
                let res = if len > buf.len() {
                    Err(EndpointError::BufferOverflow)
                } else {
                    // SAFETY: EP0 OUT was left unarmed when this packet arrived.
                    unsafe { core::ptr::copy_nonoverlapping(st.buf(), buf.as_mut_ptr(), len) };
                    Ok(len)
                };
                arm_ep0_out();
                Poll::Ready(res)
            })
        })
        .await
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, _last: bool) -> Result<(), EndpointError> {
        if data.len() > self.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        // The OUT status stage that follows the last packet is absorbed by the
        // interrupt handler, which keeps EP0 OUT armed for zero-length packets.
        self.send(data).await
    }

    async fn accept(&mut self) {
        EP_IN[0].data1.store(true, Ordering::Relaxed);
        // An error means a new SETUP or a bus reset superseded this status
        // stage, which embassy-usb handles on its next `setup()` call.
        let _ = self.send(&[]).await;
    }

    async fn reject(&mut self) {
        let st = &EP_IN[0];
        critical_section::with(|_| {
            // The STALL BD is never consumed by the controller; it is withdrawn
            // when the next SETUP arrives.
            bd_write(
                0,
                Direction::In,
                st.odd.load(Ordering::Relaxed),
                BD_OWN | BD_STALL,
                st.buf.load(Ordering::Relaxed),
            );
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        T::regs().addr().write(|w| w.set_addr(addr));
    }
}

pub(crate) trait SealedInstance: Gate<MrccPeriphConfig = UsbConfig> {
    fn regs() -> pac::usb::Usb;
}

/// USB instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this USB instance.
    type Interrupt: typelevel::Interrupt;
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_usb_instance {
    ($n:literal) => {
        paste::paste! {
            impl $crate::usb::SealedInstance for $crate::peripherals::[<USB $n>] {
                fn regs() -> $crate::pac::usb::Usb {
                    $crate::pac::[<USB $n>]
                }
            }

            impl $crate::usb::Instance for $crate::peripherals::[<USB $n>] {
                type Interrupt = $crate::interrupt::typelevel::[<USB $n>];
            }
        }
    };
}
