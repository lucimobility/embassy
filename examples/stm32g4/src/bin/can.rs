#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::num::{NonZeroU16, NonZeroU8};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::config::NominalBitTiming;
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
// use embassy_stm32::can::bxcan::filter::Mask32;
// use embassy_stm32::can::fdcan::{Fifo, Frame, StandardId};
use embassy_stm32::can::{
    Fdcan, FrameFormat, IT0InterruptHandler, IT1InterruptHandler, Id, StandardId, TxFrame, TxFrameHeader,
};
// use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::peripherals::FDCAN2;
use embassy_time::Instant;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => IT1InterruptHandler<FDCAN2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut p = embassy_stm32::init(Default::default());

    // The next two lines are a workaround for testing without transceiver.
    // To synchronise to the bus the RX input needs to see a high level.
    // Use `mem::forget()` to release the borrow on the pin but keep the
    // pull-up resistor enabled.
    let rx_pin = Input::new(&mut p.PB5, Pull::Up);
    core::mem::forget(rx_pin);

    let can = Fdcan::new(p.FDCAN2, p.PB5, p.PB6, Irqs);

    can.borrow_mut()
        .set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

    // info!("Config: {:?}", can.borrow().get_config().);
    let nominal_bit_125kbps = NominalBitTiming {
        prescaler: NonZeroU16::new(4).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };
    can.borrow_mut().set_nominal_bit_timing(nominal_bit_125kbps);

    // info!("Config: {:?}", can.borrow().get_config().interrupt_line_config);

    // Think this is replaced by the above.
    // can.as_mut()
    //     .modify_filters()
    //     .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    // Think this is replaced by into_external_loopback_mode()
    // can.as_mut()
    //     .modify_config()
    //     .set_loopback(true) // Receive own frames
    //     .set_silent(true)
    //     .leave_disabled();

    // TODO I think this needs to be configured somewhere - nominal bit timing?
    // can.set_bitrate(1_000_000);

    // can.enable().await;

    info!("Done configurign");

    let mut can = can.into_external_loopback_mode();

    let mut i: u8 = 0;
    loop {
        info!("Writing frame {}", i);
        // let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), [i]);
        let tx_frame_header = TxFrameHeader {
            len: 1,
            frame_format: FrameFormat::Fdcan,
            id: Id::Standard(StandardId::new(i as _).unwrap()),
            bit_rate_switching: false,
            marker: None,
        };
        let tx_frame = TxFrame::new(tx_frame_header, &[i]).unwrap();
        let tx_ts = Instant::now();
        can.write(&tx_frame).await;

        info!("Wrote frame {}", i);

        let rx_frame = can.read().await.unwrap();

        // We can measure loopback latency by using receive timestamp in the `Envelope`.
        // Our frame is ~55 bits long (exlcuding bit stuffing), so at 1mbps loopback delay is at least 55 us.
        // When measured with `tick-hz-1_000_000` actual latency is 80~83 us, giving a combined hardware and software
        // overhead of ~25 us. Note that CPU frequency can greatly affect the result.
        let latency = Instant::from_ticks(rx_frame.header.time_stamp as u64).saturating_duration_since(tx_ts);

        info!(
            "loopback frame {=u8}, latency: {} us",
            rx_frame.data()[0],
            latency.as_micros()
        );
        i += 1;
    }
}
