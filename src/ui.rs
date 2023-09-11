use alloc::{boxed::Box, rc::Rc, vec::Vec};
use embassy_futures::{join::join4, select::select3};
use embassy_nrf::{
    gpiote::InputChannel,
    peripherals::{GPIOTE_CH0, TWISPI1},
    twim::Twim,
};
use embassy_time::Timer;
use slint::{platform::{
    software_renderer::{MinimalSoftwareWindow, TargetPixel},
    Key, Platform,
}, ModelRc, VecModel};
use ssd1306::prelude::*;

use crate::message_hub::MessageSubscriber;

slint::include_modules!();

struct UiPlatform {
    window: Rc<MinimalSoftwareWindow>,
}

impl Platform for UiPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(embassy_time::Instant::now().as_micros())
    }
}

#[derive(Clone, Copy, defmt::Format)]
struct BinaryPixel(bool);
impl TargetPixel for BinaryPixel {
    fn blend(&mut self, color: slint::platform::software_renderer::PremultipliedRgbaColor) {
        let on = color.red > 0 || color.blue > 0 || color.green > 0;
        self.0 = on;
    }

    fn from_rgb(red: u8, green: u8, blue: u8) -> Self {
        Self(red > 0 || green > 0 || blue > 0)
    }
}

#[embassy_executor::task]
pub async fn run_ui(
    twim_dev: Twim<'static, TWISPI1>,
    mut msg_sub: MessageSubscriber,
    input: InputChannel<'static, GPIOTE_CH0, embassy_nrf::gpio::AnyPin>,
) {
    let mut display = ssd1306::Ssd1306::new(
        ssd1306::I2CDisplayInterface::new(twim_dev),
        ssd1306::size::DisplaySize128x32,
        ssd1306::rotation::DisplayRotation::Rotate0,
    );
    display.init().unwrap();
    display
        .set_draw_area((0, 0), (DISPLAY_WIDTH as u8, DISPLAY_HEIGHT as u8))
        .unwrap();
    display.clear().unwrap();

    let window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(Box::new(UiPlatform {
        window: window.clone(),
    }))
    .unwrap();

    let ui = Top::new().unwrap();

    const DISPLAY_WIDTH: usize = 128;
    const DISPLAY_HEIGHT: usize = 32;

    window.set_size(slint::PhysicalSize::new(
        DISPLAY_WIDTH as u32,
        DISPLAY_HEIGHT as u32,
    ));
    let mut buffer = [BinaryPixel(false); DISPLAY_WIDTH * DISPLAY_HEIGHT];
    let mut raw_buffer = [0u8; DISPLAY_HEIGHT * DISPLAY_WIDTH / 8];

    defmt::warn!("Start ui loop");

    let mut next_wakeup = embassy_time::Duration::from_millis(1);

    loop {
        slint::platform::update_timers_and_animations();


        let data_fut =  join4(
            msg_sub.temp_humidity_sub.next_message_pure(),
            msg_sub.pressure_sub.next_message_pure(),
            msg_sub.battery_voltage_sub.next_message_pure(),
            msg_sub.mic_sub.next_message_pure(),
        );

        match select3(Timer::after(next_wakeup), input.wait(), data_fut).await {
            embassy_futures::select::Either3::First(_) => (),
            embassy_futures::select::Either3::Second(_) => {
                defmt::info!("Key pressed");
                window.dispatch_event(slint::platform::WindowEvent::KeyPressed {
                    text: Key::Return.into(),
                });
            }
            embassy_futures::select::Either3::Third((temp_humidity, pressure, battery_voltage, mic_data)) => {
                ui.set_values(ValueScreenModel {
                    left_bottom: slint::format!("T: {:2.1}°C", temp_humidity.temperature),
                    left_top: slint::format!("RH: {:2.1}%", temp_humidity.humidity),
                    right_bottom: slint::format!("B: {:1.2}V", battery_voltage),
                    right_top: slint::format!("P: {:.0}hPa", pressure / 100.0),
                });
                let data: Vec<i32> = mic_data.into_iter().map(|element| (element / 100) as i32).collect();
                ui.set_spectrum(ModelRc::new(VecModel::from(data)));
            }
        }

        window.draw_if_needed(|renderer| {
            renderer.render(&mut buffer, DISPLAY_WIDTH);

            for y in 0..DISPLAY_HEIGHT {
                for x in 0..DISPLAY_WIDTH {
                    let value: u8 = buffer[x + y * DISPLAY_WIDTH].0.into();

                    let raw = raw_buffer[y / 8 * DISPLAY_WIDTH + x];
                    let bit = y % 8;
                    raw_buffer[y / 8 * DISPLAY_WIDTH + x] = (raw & ((1 << bit) - 1)) | value << bit;
                }
            }
            display.draw(&raw_buffer).unwrap();
        });

        next_wakeup = if !window.has_active_animations() {
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                embassy_time::Duration::from_micros(duration.as_micros() as u64)
            } else {
                embassy_time::Duration::from_millis(1000)
            }
        } else {
            embassy_time::Duration::from_micros(0)
        }
    }
}
