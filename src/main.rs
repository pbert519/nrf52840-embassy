#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod bmp280;
mod sht30;

use crate::bmp280::*;
use crate::sht30::*;

use embassy::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    time::{Duration, Timer},
    util::Forever,
};
use embassy_embedded_hal::shared_bus::i2c::I2cBusDevice;
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
static I2C_DRIVER: Forever<Mutex<CriticalSectionRawMutex, Twim<TWISPI0>>> = Forever::new();

const _ADDRESS_APDS9960: u8 = 0x39; // 57
const _ADDRESS_LSM6DS33: u8 = 0x6A; // 106
const _ADDRESS_LIS3MDL: u8 = 0x1C; // 28

// neopixel P0.16  -> SPI
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

    // initalize shared bus for i2c to use it in multiple tasks
    let i2c_driver = I2C_DRIVER.put(Mutex::new(twim));

    info!("Starting executor");
    let executor = EXECUTOR.put(embassy::executor::Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(blink(led_d13)));
        unwrap!(spawner.spawn(button(led2, switch)));
        unwrap!(spawner.spawn(sht30(i2c_driver)));
        unwrap!(spawner.spawn(bmp280(i2c_driver)));
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
async fn sht30(twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>) {
    let twim_dev = I2cBusDevice::new(twim_mutex);

    let mut sht30 = SHT30::new(twim_dev);

    loop {
        sht30.measure().await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy::task]
async fn bmp280(twim_mutex: &'static Mutex<CriticalSectionRawMutex, Twim<'static, TWISPI0>>) {
    let twim_dev = I2cBusDevice::new(twim_mutex);

    if let Ok(mut bmp280) = BMP280::new(twim_dev).await {
        loop {
            bmp280.measure().await;
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}
