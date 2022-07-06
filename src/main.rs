#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod bmp280;

use crate::bmp280::*;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_nrf::{
    gpio::{Input, Level, Output, OutputDrive, Pin, Pull},
    gpiote::{InputChannel, InputChannelPolarity},
    interrupt,
    peripherals::{GPIOTE_CH0, TWISPI0},
    twim::Twim,
};

use defmt::{info, unwrap};
use defmt_rtt as _; // global logger
use panic_probe as _;

static EXECUTOR: Forever<embassy::executor::Executor> = Forever::new();

const ADDRESS_SHT30: u8 = 0x44; // 68
const ADDRESS_BMP280: u8 = 0x77; //119
const _ADDRESS_APDS9960: u8 = 0x39; // 57
const _ADDRESS_LSM6DS33: u8 = 0x6A; // 106
const _ADDRESS_LIS3MDL: u8 = 0x1C; // 28

// neopixel P0.16  -> SPI
// x switch P1.02    -> GPIO Input
// x vdiv P0.29 / AIN5 -> Analog In
// lsm6ds22 int P1.11 -> GPIO Input
// apds int P1.00 -> GPIO Input
// pdm_dat P0.00
// pdm_clk P0.01

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

    info!("Starting executor");
    let executor = EXECUTOR.put(embassy::executor::Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(blink(led_d13)));
        unwrap!(spawner.spawn(button(led2, switch)));
        unwrap!(spawner.spawn(i2c_devices(twim)));
        unwrap!(spawner.spawn(sample_adc(adc)));
    });
}

#[embassy::task]
async fn sample_adc(mut adc: embassy_nrf::saadc::Saadc<'static, 1>) {
    loop {
        let mut buf = [0; 1];
        adc.sample(&mut buf).await;
        let voltage = (buf[0] as f32 / 16384.0) * 2.0 * 3.3;
        defmt::info!("VDD is {}", voltage);
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy::task]
async fn blink(mut led_d13: Output<'static, embassy_nrf::gpio::AnyPin>) {
    loop {
        led_d13.set_high();
        Timer::after(Duration::from_millis(300)).await;
        led_d13.set_low();
        Timer::after(Duration::from_millis(300)).await;
    }
}

#[embassy::task]
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

#[embassy::task]
async fn i2c_devices(mut twim: Twim<'static, TWISPI0>) {
    // read BMP280 calibration
    let mut calibration: [u8; 26] = [0; 26];
    let address: [u8; 1] = [0x88];
    if let Err(_res) = twim
        .write_read(ADDRESS_BMP280, &address, &mut calibration)
        .await
    {
        info!("Could not obtain BMP280 configuration, abort");
        return;
    }
    let config = BMP280Config::from_buffer(&calibration);

    loop {
        sht30(&mut twim).await;
        bmp280(&mut twim, &config).await;
        Timer::after(Duration::from_millis(900)).await;
    }
}

async fn sht30(twim: &mut Twim<'static, TWISPI0>) {
    let buf: [u8; 2] = [0x24, 0x00];
    if let Err(_res) = twim.blocking_write(ADDRESS_SHT30, &buf) {
        defmt::info!("Error write SHT I2C Command, retry in 100ms");
        Timer::after(Duration::from_millis(100)).await;
        return;
    }
    // measurement duration is maximal 15ms
    Timer::after(Duration::from_millis(20)).await;
    let mut ibuf: [u8; 6] = [0; 6];
    // use trait over method
    if let Ok(_res) = twim.read(ADDRESS_SHT30, &mut ibuf).await {
        //defmt::info!("Got buffer {}", ibuf);
        let raw_temp = u16::from_le_bytes([ibuf[1], ibuf[0]]) as f32;
        let temp: f32 = -45.0 + 175.0 * raw_temp / 65535.0;
        let raw_humidity = u16::from_le_bytes([ibuf[4], ibuf[3]]) as f32;
        let humidity: f32 = 100.0 * raw_humidity / 65535.0;
        defmt::info!("Temperature: {} and Humidity: {}", temp, humidity);
    } else {
        defmt::info!("Error while reading SHT30");
    }
}

async fn bmp280(twim: &mut Twim<'static, TWISPI0>, config: &BMP280Config) {
    // start measurement
    let start_buf: [u8; 2] = [0xF4, (0x01 << 5) | (0x01 << 2) | 0x01];
    if let Err(_res) = twim.blocking_write(ADDRESS_BMP280, &start_buf) {
        defmt::info!("Error write BMP280 I2C Command, retry in 100ms");
        Timer::after(Duration::from_millis(100)).await;
        return;
    }

    // wait till measurement is finished
    Timer::after(Duration::from_millis(20)).await;
    // read measurements from sensor
    let mut ibuf: [u8; 6] = [0; 6];
    let address: [u8; 1] = [0xF7];
    if let Ok(_res) = twim.write_read(ADDRESS_BMP280, &address, &mut ibuf).await {
        let pressure_raw: i32 =
            ((ibuf[0] as i32) << (12)) | ((ibuf[1] as i32) << 4) | ((ibuf[2] >> 4) as i32);
        let temperature_raw: i32 =
            ((ibuf[3] as i32) << (12)) | ((ibuf[4] as i32) << 4) | ((ibuf[5] >> 4) as i32);
        let (temperature, t_fine) = bmp280_compensate_temp(temperature_raw, config);
        let pressure = bmp280_compensate_pressure(pressure_raw, t_fine, config);
        info!(
            "Temperature {} and pressure {}",
            temperature,
            pressure / 100.0
        );
    } else {
        info!("Error reading BMP280");
    }
}
