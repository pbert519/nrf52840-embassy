#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod bmp280;
mod board_config;
mod log_storage;
mod neopixel;
mod sht30;

use crate::bmp280::*;
use crate::sht30::*;

use defmt::Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_nrf::{
    gpio::Output,
    gpiote::InputChannel,
    pdm::Pdm,
    peripherals::{GPIOTE_CH0, SPI3, TWISPI0, TWISPI1},
    spim::Spim,
    twim::Twim,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use defmt::{info, unwrap};
use defmt_rtt as _; // global logger
use panic_probe as _;

static SHARED_TWIM: StaticCell<Mutex<CriticalSectionRawMutex, Twim<TWISPI0>>> = StaticCell::new();

#[derive(bincode::Encode, bincode::Decode, PartialEq, Debug, Format)]
pub struct LogData {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let b = board_config::Board::configure(p);

    let mut twim = b.twim;
    // scan i2c devices
    let mut buf = [0u8, 8];
    for x in 0..127 {
        let res = twim.blocking_read(x, &mut buf);
        if res == Ok(()) {
            info!("Found device on address {}", x);
        }
    }

    // test qspi storage
    test_storage(b.qspi);

    let shared_twim = SHARED_TWIM.init(Mutex::new(twim));

    unwrap!(spawner.spawn(blink(b.led_d13)));
    unwrap!(spawner.spawn(button(b.led2, b.switch)));
    unwrap!(spawner.spawn(sample_adc(b.adc)));
    unwrap!(spawner.spawn(sample_mic(b.pdm)));
    unwrap!(spawner.spawn(neopixel(b.spi)));
    unwrap!(spawner.spawn(sht30(shared_twim)));
    unwrap!(spawner.spawn(bmp280(shared_twim)));

    unwrap!(spawner.spawn(display(b.twim_disp)));
}

fn test_storage(
    qspi: embassy_nrf::qspi::Qspi<
        'static,
        embassy_nrf::peripherals::QSPI,
        { board_config::EXTERNAL_FLASH_SIZE },
    >,
) {
    let mut storage = log_storage::LogStorage::<_, LogData>::new(qspi);
    storage
        .add_entry(LogData {
            temperature: 11.1,
            humidity: 22.2,
            pressure: 33.3,
        })
        .unwrap();
    storage
        .add_entry(LogData {
            temperature: 111.1,
            humidity: 122.2,
            pressure: 133.3,
        })
        .unwrap();
    storage
        .add_entry(LogData {
            temperature: 211.1,
            humidity: 222.2,
            pressure: 233.3,
        })
        .unwrap();
    info!("Number of entries: {}", storage.len());
    let data = storage.at(storage.len() - 1).unwrap();
    info!("Last LogEntry: {:?}", &data);
}

#[embassy_executor::task]
async fn display(twim_dev: Twim<'static, TWISPI1>) {
    let mut display = ssd1306::Ssd1306::new(
        ssd1306::I2CDisplayInterface::new(twim_dev),
        ssd1306::size::DisplaySize128x32,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    ssd1306::prelude::DisplayConfig::init(&mut display).unwrap();
    let text_style = embedded_graphics::mono_font::MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_10X20)
        .text_color(embedded_graphics::pixelcolor::BinaryColor::On)
        .build();
    embedded_graphics::Drawable::draw(
        &embedded_graphics::text::Text::with_baseline(
            "Hello world!",
            embedded_graphics::prelude::Point::zero(),
            text_style,
            embedded_graphics::text::Baseline::Top,
        ),
        &mut display,
    )
    .unwrap();
    display.flush().unwrap();
}

#[embassy_executor::task]
async fn sample_mic(mut pdm: Pdm<'static>) {
    const SAMPLES: usize = 2048;
    const BASE_FREQUENCY: f32 = 16000.0 / SAMPLES as f32;
    pdm.start().await;
    // some time to stabilize the microphon
    Timer::after(Duration::from_millis(1000)).await;

    loop {
        let mut buf = [0i16; SAMPLES];

        pdm.sample(&mut buf).await.unwrap();

        let mut fbuf = [0.0f32; SAMPLES];
        let mut sum: u64 = 0;
        for i in 100..fbuf.len() {
            fbuf[i] = buf[i] as f32;
            sum += buf[i].unsigned_abs() as u64;
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
        defmt::info!(
            "Main Frequency: {:?} with Amplitude: {:?}",
            max_index as f32 * BASE_FREQUENCY,
            amplitudes[max_index]
        );

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
        if let Ok(measurement) = sht30.measure().await {
            defmt::info!(
                "Temperature: {} and Humidity: {}",
                measurement.temp,
                measurement.humidity
            );
        } else {
            defmt::info!("SHT Error, retry in 100ms");
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn bmp280(twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>) {
    let twim_dev = I2cDevice::new(twim_mutex);

    if let Ok(mut bmp280) = BMP280::new(twim_dev).await {
        loop {
            if let Ok(measurment) = bmp280.measure().await {
                defmt::info!(
                    "Temperature {} and pressure {}",
                    measurment.temp,
                    measurment.pressure / 100.0
                );
            } else {
                defmt::info!("Error reading BMP280");
            }
            Timer::after(Duration::from_millis(1000)).await;
        }
    } else {
        defmt::info!("Could not obtain BMP280 configuration, abort BMP280 task");
    }
}

#[embassy_executor::task]
async fn neopixel(spi: Spim<'static, SPI3>) {
    let mut neopixel = neopixel::Neopixel::new(spi);

    let mut counter: u8 = 0;
    loop {
        neopixel.set_pixel(counter, 0, 0).await;
        if counter >= 100 {
            counter = 0;
        }
        counter += 10;
        Timer::after(Duration::from_millis(100)).await;
    }
}
