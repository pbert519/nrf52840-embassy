use crate::{
    driver::{bmp280::*, sht30::*},
    message_hub::TemperatureHumidityMeasurement,
};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::{peripherals::TWISPI0, twim::Twim};
use embassy_sync::pubsub::DynImmediatePublisher;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn sample_battery_voltage(
    mut adc: embassy_nrf::saadc::Saadc<'static, 1>,
    batt_pub: DynImmediatePublisher<'static, f32>,
) {
    loop {
        let mut buf = [0; 1];
        adc.sample(&mut buf).await;
        let voltage = (buf[0] as f32 / 16384.0) * 2.0 * 3.3;
        batt_pub.publish_immediate(voltage);
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
pub async fn sht30(
    twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>,
    sht30_pub: DynImmediatePublisher<'static, TemperatureHumidityMeasurement>,
) {
    let twim_dev = I2cDevice::new(twim_mutex);

    let mut sht30 = SHT30::new(twim_dev);

    loop {
        if let Ok(measurement) = sht30.measure().await {
            sht30_pub.publish_immediate(TemperatureHumidityMeasurement {
                temperature: measurement.temp,
                humidity: measurement.humidity,
            });
        } else {
            defmt::warn!("SHT Error, retry in 100ms");
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
pub async fn bmp280(
    twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>,
    press_pub: DynImmediatePublisher<'static, f32>,
) {
    let twim_dev = I2cDevice::new(twim_mutex);

    if let Ok(mut bmp280) = BMP280::new(twim_dev).await {
        loop {
            if let Ok(measurement) = bmp280.measure().await {
                press_pub.publish_immediate(measurement.pressure)
            } else {
                defmt::warn!("Error reading BMP280");
            }
            Timer::after(Duration::from_millis(1000)).await;
        }
    } else {
        defmt::warn!("Could not obtain BMP280 configuration, abort BMP280 task");
    }
}
