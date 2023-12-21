#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::*;
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::Timer;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs2 {
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
});

bind_interrupts!(struct Irqs3 {
    FDCAN3_IT0 => can::IT0InterruptHandler<FDCAN3>;
    FDCAN3_IT1 => can::IT1InterruptHandler<FDCAN3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Config::default();

    let mut peripherals = embassy_stm32::init(config);

    // Swap which line is commented to test FDCAN2 and FDCAN3 separately.
    let can = can::Fdcan::new(peripherals.FDCAN2, peripherals.PB5, peripherals.PB6, Irqs2);
    // let can = can::Fdcan::new(peripherals.FDCAN3, peripherals.PA8, peripherals.PA15, Irqs3);

    // 125k bps
    let bit_timing = can::config::NominalBitTiming {
        sync_jump_width: 1.try_into().unwrap(),
        prescaler: 8.try_into().unwrap(),
        seg1: 13.try_into().unwrap(),
        seg2: 2.try_into().unwrap(),
    };
    can.can.borrow_mut().set_nominal_bit_timing(bit_timing);

    info!("Configured");

    // PB12 & PB7 are the standby pins on CAN transceiver
    let _can_standby2 = Output::new(&mut peripherals.PB7, Level::Low, Speed::Low);
    let _can_standby3 = Output::new(&mut peripherals.PB12, Level::Low, Speed::Low);

    let mut can = can.into_external_loopback_mode();
    // let mut can = can.into_normal_mode();

    let mut i = 0;
    loop {
        let frame = can::TxFrame::new(
            can::TxFrameHeader {
                len: 1,
                frame_format: can::FrameFormat::Standard,
                id: can::StandardId::new(0x123).unwrap().into(),
                bit_rate_switching: false,
                marker: None,
            },
            &[i],
        )
        .unwrap();
        info!("Writing frame");
        _ = can.write(&frame).await;


        match can.read().await {
            Ok(rx_frame) => info!("Rx: {}", rx_frame.data()[0]),
            Err(_err) => error!("Error in frame"),
        }

        Timer::after_millis(250).await;

        i += 1;
    }
}
