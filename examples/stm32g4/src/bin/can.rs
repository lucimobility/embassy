#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
// use embassy_stm32::can::bxcan::filter::Mask32;
// use embassy_stm32::can::fdcan::{Fifo, Frame, StandardId};
use embassy_stm32::can::{Fdcan, IT0InterruptHandler, IT1InterruptHandler, StandardId};
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

    let mut can = Fdcan::new(p.FDCAN2, p.PB5, p.PB6, Irqs);

    can.set_standard_filter(StandardFilterSlot::_0, StandardFilter::accept_all_into_fifo0());

    can.as_mut()
        .modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    can.as_mut()
        .modify_config()
        .set_loopback(true) // Receive own frames
        .set_silent(true)
        .leave_disabled();

    can.set_bitrate(1_000_000);

    can.enable().await;

    let mut i: u8 = 0;
    loop {
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), [i]);
        let tx_ts = Instant::now();
        can.write(&tx_frame).await;

        let envelope = can.read().await.unwrap();

        // We can measure loopback latency by using receive timestamp in the `Envelope`.
        // Our frame is ~55 bits long (exlcuding bit stuffing), so at 1mbps loopback delay is at least 55 us.
        // When measured with `tick-hz-1_000_000` actual latency is 80~83 us, giving a combined hardware and software
        // overhead of ~25 us. Note that CPU frequency can greatly affect the result.
        let latency = envelope.ts.saturating_duration_since(tx_ts);

        info!(
            "loopback frame {=u8}, latency: {} us",
            unwrap!(envelope.frame.data())[0],
            latency.as_micros()
        );
        i += 1;
    }
}
