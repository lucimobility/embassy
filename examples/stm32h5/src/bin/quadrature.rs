#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use embassy_stm32::timer::qei::{Qei, QeiPin};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PB0, Level::High, Speed::Low);

    let mut a = QeiPin::new_ch1(p.PB4);
    let mut b = QeiPin::new_ch2(p.PB5);

    let mut quadrature = Qei::new(p.TIM3, a, b);

    loop {
        // info!("high");
        led.set_high();
        Timer::after_millis(200).await;

        // info!("low");
        led.set_low();
        Timer::after_millis(200).await;

        let direction = match quadrature.read_direction() {
            embassy_stm32::timer::qei::Direction::Upcounting => "Up",
            embassy_stm32::timer::qei::Direction::Downcounting => "Down",
        };

        info!("Count: {}, Direction: {}", quadrature.count(), direction);
    }
}
