#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::adc::{Adc, AdcConfig, SampleTime};
use embassy_stm32::dac::{DacChannel, Value};
use embassy_stm32::gpio::Level::Low;
use embassy_stm32::gpio::Output;
use embassy_stm32::gpio::Speed::High;
use embassy_stm32::pac::adccommon::vals::Presc;
use embassy_stm32::pac::vrefbuf::vals::{Hiz, Vrs};
use embassy_stm32::vrefbuf::VoltageReferenceBuffer;
use embassy_time::{Instant, Timer};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL25,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV4), // SPI1 cksel defaults to pll1_q
            divr: None,
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL25,
            divp: None,
            divq: None,
            divr: Some(PllDiv::DIV4), // 100mhz
        });
        config.rcc.sys = Sysclk::PLL1_P; // 200 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV1; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.adcdacsel = mux::Adcdacsel::PLL2_R;
    }
    let mut p = embassy_stm32::init(config);

    info!("Hello World!");

    let _vref = VoltageReferenceBuffer::new(p.VREFBUF, Vrs::VREF0, Hiz::CONNECTED);

    let mut dac = DacChannel::new_blocking(p.DAC1, p.PA4);

    let mut adc_config = AdcConfig::default();

    adc_config.prescaler = Some(Presc::DIV256);

    let mut adc = Adc::new_with_config(p.ADC1, adc_config);

    dac.set(Value::Bit12Right(3070));

    let mut vrefint_channel = adc.enable_vrefint();

    let mut gpio = Output::new(p.PC6, Low, High);

    loop {
        let start = Instant::now();
        gpio.set_high();
        // 17us with no divider
        let vrefint = adc.blocking_read(&mut vrefint_channel, SampleTime::CYCLES640_5);
        let measured = adc.blocking_read(&mut p.PC0, SampleTime::CYCLES640_5);
        gpio.set_low();
        let stop = Instant::now();

        info!("ADC reads took {} ns", (stop - start).as_nanos());
        info!("vrefint: {}", vrefint);
        info!("measured: {}", measured);
        Timer::after_millis(500).await;
    }
}
