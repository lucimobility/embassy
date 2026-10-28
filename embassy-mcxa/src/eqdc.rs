//! Enhanced Quadrature Decoder (eQDC)
//!
//! The eQDC has no dedicated pins. Its phase and index inputs are selected
//! through INPUTMUX from the chip's `TRIG_IN` signals, so encoder signals must
//! be wired to pins that offer a `TRIG_IN` alternate function.

use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};

use crate::clocks::periph_helpers::NoConfig;
use crate::clocks::{ClockError, Gate, disable, enable, enable_and_reset};
use crate::gpio::{GpioPin, SealedPin};
use crate::pac;
use crate::pac::port::Mux;
use crate::peripherals::INPUTMUX0;

/// INPUTMUX source value of `TRIG_IN0` for the QDC input select registers.
const TRIG_IN0_INP: u8 = 0x18;

/// eQDC configuration.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Input filter sample period, in peripheral (bus) clock cycles. `0`
    /// bypasses the filter.
    pub filter_period: u8,
    /// Number of consecutive matching samples, minus 3, an input must hold
    /// before a level change is accepted. Must be in `0..=7`.
    pub filter_count: u8,
    /// Count in the opposite direction.
    pub reverse: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            filter_period: 0,
            filter_count: 0,
            reverse: false,
        }
    }
}

/// Raw levels of the decoder inputs after filtering, see [`Eqdc::inputs`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Inputs {
    /// Phase A
    pub phase_a: bool,
    /// Phase B
    pub phase_b: bool,
    /// Index
    pub index: bool,
    /// Last counting direction was positive.
    pub up: bool,
}

/// Quadrature decoder driver, counting in quadrature (4x) mode.
pub struct Eqdc<'d> {
    regs: pac::eqdc::Eqdc,
    _phantom: PhantomData<&'d mut ()>,
}

impl<'d> Eqdc<'d> {
    /// Create a decoder from phase A and phase B inputs.
    pub fn new<T: Instance>(
        _peri: Peri<'d, T>,
        phase_a: Peri<'d, impl TrigInPin>,
        phase_b: Peri<'d, impl TrigInPin>,
        config: Config,
    ) -> Result<Self, ClockError> {
        Self::new_inner::<T>(phase_a.trig_in(), phase_b.trig_in(), None, config, || {
            phase_a.mux();
            phase_b.mux();
        })
    }

    /// Create a decoder from phase A, phase B and index inputs.
    ///
    /// The index input is routed to the decoder, but does not reset the
    /// position counter.
    pub fn new_with_index<T: Instance>(
        _peri: Peri<'d, T>,
        phase_a: Peri<'d, impl TrigInPin>,
        phase_b: Peri<'d, impl TrigInPin>,
        index: Peri<'d, impl TrigInPin>,
        config: Config,
    ) -> Result<Self, ClockError> {
        Self::new_inner::<T>(phase_a.trig_in(), phase_b.trig_in(), Some(index.trig_in()), config, || {
            phase_a.mux();
            phase_b.mux();
            index.mux();
        })
    }

    fn new_inner<T: Instance>(
        phase_a: u8,
        phase_b: u8,
        index: Option<u8>,
        config: Config,
        mux_pins: impl FnOnce(),
    ) -> Result<Self, ClockError> {
        // The eQDC runs from the bus clock, there is nothing to configure.
        _ = unsafe { enable_and_reset::<T>(&NoConfig)? };

        mux_pins();
        T::route_inputs(phase_a, phase_b, index);

        let regs = T::regs();

        // FILT_PER must pass through zero when changing between non-zero values.
        regs.filt().write(|w| w.set_filt_per(0));
        if config.filter_period != 0 {
            regs.filt().write(|w| {
                w.set_filt_cnt(config.filter_count.min(7));
                w.set_filt_per(config.filter_period);
            });
        }

        // Quadrature mode (PH1 = 0, OPMODE = 0), index does not initialize
        // the position (XIP = 0).
        regs.ctrl2().write(|_| {});
        regs.ctrl().write(|w| w.set_rev(config.reverse));

        let mut eqdc = Self {
            regs,
            _phantom: PhantomData,
        };
        eqdc.set_position(0);
        Ok(eqdc)
    }

    /// Current position counter value.
    pub fn position(&self) -> i32 {
        // Reading UPOS latches LPOS into LPOSH, so the halves are coherent.
        let upper = self.regs.upos().read().pos();
        let lower = self.regs.lposh().read().lposh();
        (((upper as u32) << 16) | lower as u32) as i32
    }

    /// Set the position counter.
    pub fn set_position(&mut self, position: i32) {
        let position = position as u32;
        self.regs.uinit().write(|w| w.set_init((position >> 16) as u16));
        self.regs.linit().write(|w| w.set_init(position as u16));

        // UINIT/LINIT are buffered: with CTRL2[LDMOD] = 0 they are loaded
        // immediately once LDOK is set, after which LDOK self-clears.
        self.regs.ctrl().modify(|w| w.set_ldok(true));
        while self.regs.ctrl().read().ldok() {}

        self.regs.ctrl().modify(|w| w.set_swip(true));
    }

    /// Number of index pulses seen, counted up or down with direction.
    pub fn revolutions(&self) -> i16 {
        self.regs.rev().read().rev() as i16
    }

    /// Filtered input levels, useful for checking the encoder wiring.
    pub fn inputs(&self) -> Inputs {
        let imr = self.regs.imr().read();
        Inputs {
            phase_a: imr.fpha(),
            phase_b: imr.fphb(),
            index: imr.find_pre(),
            up: imr.dir(),
        }
    }
}

/// A pin that can be routed to the eQDC through a `TRIG_IN` signal.
#[allow(private_bounds)]
pub trait TrigInPin: GpioPin + SealedTrigInPin + PeripheralType {}

pub(crate) trait SealedTrigInPin: SealedPin {
    /// Index `n` of the pin's `TRIG_INn` signal.
    const TRIG_IN: u8;

    fn trig_in(&self) -> u8 {
        TRIG_IN0_INP + Self::TRIG_IN
    }

    fn mux(&self) {
        self.set_pull(crate::gpio::Pull::Disabled);
        self.set_function(Mux::Mux1);
        self.set_enable_input_buffer(true);
    }
}

macro_rules! impl_trig_in_pin {
    ($pin:ident, $n:literal) => {
        impl SealedTrigInPin for crate::peripherals::$pin {
            const TRIG_IN: u8 = $n;
        }
        impl TrigInPin for crate::peripherals::$pin {}
    };
}

// MCXA2xx pinmux table: every TRIG_INn signal is on ALT1.
impl_trig_in_pin!(P1_0, 0);
impl_trig_in_pin!(P1_1, 1);
impl_trig_in_pin!(P1_6, 2);
impl_trig_in_pin!(P1_13, 3);
#[cfg(feature = "sosc-as-gpio")]
impl_trig_in_pin!(P1_31, 4);
impl_trig_in_pin!(P2_0, 6);
impl_trig_in_pin!(P2_1, 7);
impl_trig_in_pin!(P2_2, 6);
impl_trig_in_pin!(P2_3, 7);
impl_trig_in_pin!(P2_7, 5);
impl_trig_in_pin!(P2_9, 4);
impl_trig_in_pin!(P2_11, 4);
impl_trig_in_pin!(P2_13, 8);
impl_trig_in_pin!(P2_17, 9);
impl_trig_in_pin!(P2_20, 8);
impl_trig_in_pin!(P2_21, 9);
impl_trig_in_pin!(P2_26, 5);
impl_trig_in_pin!(P3_0, 0);
impl_trig_in_pin!(P3_1, 1);
impl_trig_in_pin!(P3_7, 2);
impl_trig_in_pin!(P3_8, 3);
impl_trig_in_pin!(P3_9, 4);
impl_trig_in_pin!(P3_10, 5);
impl_trig_in_pin!(P3_11, 6);
impl_trig_in_pin!(P3_24, 11);
impl_trig_in_pin!(P3_26, 10);
impl_trig_in_pin!(P3_28, 11);
impl_trig_in_pin!(P3_31, 10);
impl_trig_in_pin!(P4_0, 5);
impl_trig_in_pin!(P4_6, 4);
impl_trig_in_pin!(P4_7, 5);

pub(crate) trait SealedInstance: Gate<MrccPeriphConfig = NoConfig> {
    fn regs() -> pac::eqdc::Eqdc;

    fn route_inputs(phase_a: u8, phase_b: u8, index: Option<u8>);
}

/// eQDC instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {}

pub(crate) fn write_inputmux(f: impl FnOnce(pac::inputmux::Inputmux)) {
    critical_section::with(|_| {
        let _ = unsafe { enable::<INPUTMUX0>(&NoConfig) };
        f(pac::INPUTMUX0);
        unsafe { disable::<INPUTMUX0>() };
    });
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_eqdc_instance {
    ($n:literal) => {
        paste::paste! {
            impl $crate::eqdc::SealedInstance for $crate::peripherals::[<EQDC $n>] {
                fn regs() -> $crate::pac::eqdc::Eqdc {
                    $crate::pac::[<EQDC $n>]
                }

                fn route_inputs(phase_a: u8, phase_b: u8, index: Option<u8>) {
                    $crate::eqdc::write_inputmux(|im| {
                        im.[<qdc $n _phasea>]().write(|w| w.set_inp(phase_a));
                        im.[<qdc $n _phaseb>]().write(|w| w.set_inp(phase_b));
                        // 0x7f leaves the input unconnected.
                        im.[<qdc $n _index>]().write(|w| w.set_inp(index.unwrap_or(0x7f)));
                    });
                }
            }

            impl $crate::eqdc::Instance for $crate::peripherals::[<EQDC $n>] {}
        }
    };
}
