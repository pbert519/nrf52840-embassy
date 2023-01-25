#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(default_alloc_error_handler)]

mod board_config;
mod display;
mod driver;
mod message_hub;
mod sensors;
mod softdevice;

use crate::driver::{log_storage::*, neopixel::*};
use display::*;
use message_hub::*;
use sensors::*;

#[macro_use]
extern crate alloc;

use defmt::Format;
use embassy_executor::Spawner;
use embassy_nrf::{
    gpio::Output,
    gpiote::InputChannel,
    pdm::Pdm,
    peripherals::{GPIOTE_CH0, SPI3, TWISPI0},
    spim::Spim,
    twim::Twim,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use defmt_rtt as _; // global logger
use panic_probe as _;

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

static SHARED_TWIM: static_cell::StaticCell<Mutex<CriticalSectionRawMutex, Twim<TWISPI0>>> =
    static_cell::StaticCell::new();
static MEASSGE_HUB: static_cell::StaticCell<MessageHub> = static_cell::StaticCell::new();

#[derive(bincode::Encode, bincode::Decode, PartialEq, Debug, Format)]
pub struct LogData {
    pub temperature: f32,
    pub humidity: f32,
    pub pressure: f32,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let b = board_config::Board::init();
    let shared_twim = SHARED_TWIM.init(Mutex::new(b.twim));
    let message_hub = MEASSGE_HUB.init(MessageHub::new());

    // Initialize the allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    }
    // scan i2c devices
    defmt::info!("Scan I2C Devices on interface TWISPI0");
    let mut buf = [0u8, 8];
    for x in 0..127 {
        let res = shared_twim.lock().await.blocking_read(x, &mut buf);
        if res == Ok(()) {
            defmt::info!("Found device on address {}", x);
        }
    }

    // test qspi storage
    test_storage(b.qspi);

    // softdevice
    let sd = softdevice::configure_softdevice();
    let server = softdevice::Server::new(sd).unwrap();

    // Spin-up all tasks
    spawner.spawn(softdevice::softdevice_task(sd)).unwrap();
    spawner
        .spawn(softdevice::gatt_server_task(
            sd,
            server,
            message_hub.subscriber(),
        ))
        .unwrap();
    spawner.spawn(blink(b.led_d13)).unwrap();
    spawner.spawn(button(b.led2, b.switch)).unwrap();
    spawner
        .spawn(sample_battery_voltage(
            b.adc,
            message_hub.battery_voltage.dyn_immediate_publisher(),
        ))
        .unwrap();
    spawner.spawn(sample_mic(b.pdm)).unwrap();
    spawner.spawn(neopixel(b.spi)).unwrap();
    spawner
        .spawn(sht30(
            shared_twim,
            message_hub.temperature_humidity.dyn_immediate_publisher(),
        ))
        .unwrap();
    spawner
        .spawn(bmp280(
            shared_twim,
            message_hub.pressure.dyn_immediate_publisher(),
        ))
        .unwrap();

    spawner
        .spawn(display(b.twim_disp, message_hub.subscriber()))
        .unwrap();
}

fn test_storage(
    qspi: embassy_nrf::qspi::Qspi<
        'static,
        embassy_nrf::peripherals::QSPI,
        { board_config::EXTERNAL_FLASH_SIZE },
    >,
) {
    let mut storage = LogStorage::<_, LogData>::new(qspi);
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
    defmt::info!("Number of entries: {}", storage.len());
    let data = storage.at(storage.len() - 1).unwrap();
    defmt::info!("Last LogEntry: {:?}", &data);
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
async fn neopixel(spi: Spim<'static, SPI3>) {
    let mut neopixel = Neopixel::new(spi);

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
