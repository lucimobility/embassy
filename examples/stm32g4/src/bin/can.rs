#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cmp::Ordering;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::*;
use embassy_stm32::{bind_interrupts, can, Config};
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
    let can3 = can::Fdcan::new(peripherals.FDCAN3, peripherals.PA8, peripherals.PA15, Irqs3);

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

    let mut i: u8 = 0;

    let wakeup_frame = can::TxFrame::new(
        can::TxFrameHeader {
            len: 1,
            frame_format: can::FrameFormat::Standard,
            id: can::StandardId::new(0xF).unwrap().into(),
            bit_rate_switching: false,
            marker: None,
        },
        &[1],
    )
    .unwrap();

    let pm_hb_frame = can::TxFrame::new(
        can::TxFrameHeader {
            len: 1,
            frame_format: can::FrameFormat::Standard,
            id: can::ExtendedId::new(0x0C140000).unwrap().into(),
            bit_rate_switching: false,
            marker: None,
        },
        &[0xC0],
    )
    .unwrap();

    let jsm_heartbeat_frame = can::TxFrame::new(
        can::TxFrameHeader {
            len: 7,
            frame_format: can::FrameFormat::Standard,
            id: can::ExtendedId::new(0x03C30F0F).unwrap().into(),
            bit_rate_switching: false,
            marker: None,
        },
        &[0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87, 0x87],
    )
    .unwrap();
    // info!("Writing frame");
    // _ = can.write(&frame).await;

    let info_frame = can::TxFrame::new(
        can::TxFrameHeader {
            len: 7,
            frame_format: can::FrameFormat::Standard,
            id: can::StandardId::new(0x123).unwrap().into(),
            bit_rate_switching: false,
            marker: None,
        },
        &[0, 5, 1, 43, 23, 12, 5],
    )
    .unwrap();

    let info_request_frame = can::TxFrame::new(
        can::TxFrameHeader {
            len: 0,
            frame_format: can::FrameFormat::Standard,
            id: can::StandardId::new(0x321).unwrap().into(),
            bit_rate_switching: false,
            marker: None,
        },
        &[],
    )
    .unwrap();

    loop {
        // let frame = can::TxFrame::new(
        //     can::TxFrameHeader {
        //         len: 7,
        //         frame_format: can::FrameFormat::Standard,
        //         id: can::StandardId::new(0x123).unwrap().into(),
        //         bit_rate_switching: false,
        //         marker: None,
        //     },
        //     &[0, 5, 0, 253,  23, 12, 5],
        // )

        // .unwrap();
        info!("Writing frame");

        match i.cmp(&3) {
            Ordering::Less => _ = can.write(&wakeup_frame).await,
            Ordering::Equal => {
                _ = can.write(&pm_hb_frame).await;
                // Timer::after_millis(70).await;
                // _ = can.write(&info_frame).await;
                // _ = can.write(&info_request_frame).await;

                // match can.read().await {
                //     Ok(rx_frame) => info!(
                //         "Rx: {} {} {} {} {} {} {}",
                //         rx_frame.data()[0],
                //         rx_frame.data()[1],
                //         rx_frame.data()[2],
                //         rx_frame.data()[3],
                //         rx_frame.data()[4],
                //         rx_frame.data()[5],
                //         rx_frame.data()[6]
                //     ),
                //     Err(err) => error!("Error in frame {}", err),
                // }
            }
            Ordering::Greater => _ = can.write(&jsm_heartbeat_frame).await,
        }

        match can.read().await {
            Ok(rx_frame) => info!("Rx: {}", rx_frame.data()[0]),
            Err(_err) => error!("Error in frame"),
        }

        Timer::after_millis(70).await;

        i = i.saturating_add(1);
    }
}
