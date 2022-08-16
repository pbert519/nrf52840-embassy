#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod bmp280;
mod neopixel;
mod sht30;

use crate::bmp280::*;
use crate::sht30::*;

use embassy_util::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    Forever
};
use embassy_executor::{
    time::{Duration, Timer},
};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::{
    gpio::{Input, Level, Output, OutputDrive, Pin, Pull},
    gpiote::{InputChannel, InputChannelPolarity},
    interrupt,
    pdm::Pdm,
    peripherals::{GPIOTE_CH0, TWISPI0, TWISPI1},
    spim::Spim,
    twim::Twim,
    qspi::*,
};

use defmt::{info, unwrap};
use defmt_rtt as _; // global logger
use panic_probe as _;

static EXECUTOR: Forever<embassy_executor::executor::Executor> = Forever::new();
static I2C_DRIVER: Forever<Mutex<CriticalSectionRawMutex, Twim<TWISPI0>>> = Forever::new();

const _ADDRESS_APDS9960: u8 = 0x39; // 57
const _ADDRESS_LSM6DS33: u8 = 0x6A; // 106
const _ADDRESS_LIS3MDL: u8 = 0x1C; // 28

// lsm6ds22 int P1.11 -> GPIO Input
// apds int P1.00 -> GPIO Input
// qspi flash
// usb

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut p = embassy_nrf::init(Default::default());

    // configure gpio
    let led_d13 = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
    let led2 = Output::new(p.P1_10.degrade(), Level::Low, OutputDrive::Standard);
    let switch = InputChannel::new(
        p.GPIOTE_CH0,
        Input::new(p.P1_02.degrade(), Pull::Up),
        InputChannelPolarity::HiToLo,
    );
    // configure adc
    let adc_config = embassy_nrf::saadc::Config::default();
    let channel_config = embassy_nrf::saadc::ChannelConfig::single_ended(&mut p.P0_29);
    let adc = embassy_nrf::saadc::Saadc::new(
        p.SAADC,
        interrupt::take!(SAADC),
        adc_config,
        [channel_config],
    );

    // configure twi
    let twim_config = embassy_nrf::twim::Config::default();
    let twim_irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
    let mut twim = Twim::new(p.TWISPI0, twim_irq, p.P0_12, p.P0_11, twim_config);

    // scan i2c devices
    let mut buf = [0u8, 8];
    for x in 0..127 {
        let res = twim.blocking_read(x, &mut buf);
        if res == Ok(()) {
            info!("Found device on address {}", x);
        }
    }

    // initalize shared bus for i2c to use it in multiple tasks
    let i2c_driver = I2C_DRIVER.put(Mutex::new(twim));

    // the mic samples with 16 kHz
    let pdm_config = embassy_nrf::pdm::Config::default();
    let pdm_irq = interrupt::take!(PDM);
    let pdm = Pdm::new(p.PDM, pdm_irq, p.P0_01, p.P0_00, pdm_config);

    // qspi flash interface
    let mut qspi_config = embassy_nrf::qspi::Config::default();
    //qspi_config.read_opcode = ReadOpcode::READ4IO;
    //qspi_config.write_opcode = WriteOpcode::PP4IO;
    //qspi_config.write_page_size = WritePageSize::_256BYTES;
    let qspi_irq = interrupt::take!(QSPI);
    let qspi: Qspi<'_, _,16777216> = Qspi::new(p.QSPI, qspi_irq, p.P0_19, p.P0_20, p.P0_17, p.P0_22, p.P0_23, p.P0_21, qspi_config);

    // spi
    let mut spi_config = embassy_nrf::spim::Config::default();
    spi_config.frequency = embassy_nrf::spim::Frequency::M2;
    let spi_irq = interrupt::take!(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1);
    let spi = Spim::new_txonly(p.TWISPI1, spi_irq, p.P0_15, p.P0_16, spi_config);

    info!("Starting executor");
    let executor = EXECUTOR.put(embassy_executor::executor::Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(blink(led_d13)));
        unwrap!(spawner.spawn(button(led2, switch)));
        unwrap!(spawner.spawn(sht30(i2c_driver)));
        unwrap!(spawner.spawn(bmp280(i2c_driver)));
        unwrap!(spawner.spawn(sample_adc(adc)));
        unwrap!(spawner.spawn(sample_mic(pdm)));
        unwrap!(spawner.spawn(neopixel(spi)));
        // unwrap!(spawner.spawn(display(i2c_driver)));
    })
}

#[embassy_executor::task]
async fn display(twim_mutex: &'static embassy_util::blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::RefCell<Twim<'static, TWISPI0>>>) {
    let twim_dev = embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice::new(twim_mutex);
    
    let mut display = ssd1306::Ssd1306::new(
        ssd1306::I2CDisplayInterface::new(twim_dev),
        ssd1306::size::DisplaySize128x32,
        ssd1306::rotation::DisplayRotation::Rotate0,
    ).into_buffered_graphics_mode();
    ssd1306::prelude::DisplayConfig::init(&mut display);
    let text_style = embedded_graphics::mono_font::MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
        .text_color(embedded_graphics::pixelcolor::BinaryColor::On)
        .build();
    embedded_graphics::Drawable::draw(&embedded_graphics::text::Text::with_baseline("Hello world!", embedded_graphics::prelude::Point::zero(), text_style, embedded_graphics::text::Baseline::Top), &mut display)
        .unwrap();
    display.flush().unwrap();
}

#[embassy_executor::task]
async fn sample_mic(mut pdm: Pdm<'static>) {
    const SAMPLES: usize = 2048;
    const BASE_FREQUENCY: f32 = 16000.0 / SAMPLES as f32;

    // some time to stabilize the microphon
    Timer::after(Duration::from_millis(1000)).await;

    loop {
        let mut buf = [0i16; SAMPLES];

        pdm.sample(&mut buf).await.unwrap();

        let mut fbuf = [0.0f32; SAMPLES];
        let mut sum: u64 = 0;
        for i in 100..fbuf.len() {
            fbuf[i] = buf[i] as f32;
            sum += buf[i].abs() as u64;
        }
        let loudness = sum / SAMPLES as u64;

        let mut spectrum = microfft::real::rfft_2048(&mut fbuf);
        spectrum[0].im = 0.0;

        let mut amplitudes = [0u32; SAMPLES / 2];
        let mut max_index = 0;
        for i in 0..amplitudes.len() {
            let amp = spectrum[i].l1_norm() as u32;
            amplitudes[i] = amp;

            if amp > amplitudes[max_index] {
                max_index = i;
            }
        }

        defmt::info!("Sound level: {:?}", loudness);
        defmt::info!("Main Frequency: {:?} with Amplitude: {:?}", max_index as f32 * BASE_FREQUENCY, amplitudes[max_index]);

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn sample_adc(mut adc: embassy_nrf::saadc::Saadc<'static, 1>) {
    loop {
        let mut buf = [0; 1];
        adc.sample(&mut buf).await;
        let voltage = (buf[0] as f32 / 16384.0) * 2.0 * 3.3;
        defmt::info!("VDD is {}", voltage);
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn blink(mut led_d13: Output<'static, embassy_nrf::gpio::AnyPin>) {
    loop {
        led_d13.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led_d13.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}

#[embassy_executor::task]
async fn button(
    mut led: Output<'static, embassy_nrf::gpio::AnyPin>,
    input: InputChannel<'static, GPIOTE_CH0, embassy_nrf::gpio::AnyPin>,
) {
    loop {
        input.wait().await;
        led.set_high();
        input.wait().await;
        led.set_low();
    }
}

#[embassy_executor::task]
async fn sht30(twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>) {
    let twim_dev = I2cDevice::new(twim_mutex);

    let mut sht30 = SHT30::new(twim_dev);

    loop {
        sht30.measure().await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn bmp280(twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>) {
    let twim_dev = I2cDevice::new(twim_mutex);

    if let Ok(mut bmp280) = BMP280::new(twim_dev).await {
        loop {
            bmp280.measure().await;
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}

#[embassy_executor::task]
async fn neopixel(spi: Spim<'static, TWISPI1>) {
    let mut neopixel = neopixel::Neopixel::new(spi);

    let mut counter: u8 = 0;
    loop {
        neopixel.set_pixel(0, 0, 0).await;
        if counter >= 100 {
            counter = 0;
        }
        Timer::after(Duration::from_millis(10));
    }
}
