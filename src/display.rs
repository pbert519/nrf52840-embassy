use crate::MessageSubscriber;
use embassy_futures::join::join3;
use embassy_nrf::{peripherals::TWISPI1, twim::Twim};
use embedded_graphics::{
    geometry::AnchorPoint,
    mono_font::{iso_8859_1::FONT_6X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

#[embassy_executor::task]
pub async fn display(twim_dev: Twim<'static, TWISPI1>, mut msg_sub: MessageSubscriber) {
    let mut display = ssd1306::Ssd1306::new(
        ssd1306::I2CDisplayInterface::new(twim_dev),
        ssd1306::size::DisplaySize128x32,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    ssd1306::prelude::DisplayConfig::init(&mut display).unwrap();
    let bounding_box = display.bounding_box();

    let character_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X13)
        .text_color(BinaryColor::On)
        .build();

    loop {
        let (temp_humidity, pressure, battery_voltage) = join3(
            msg_sub.temp_humidity_sub.next_message_pure(),
            msg_sub.pressure_sub.next_message_pure(),
            msg_sub.battery_voltage_sub.next_message_pure(),
        )
        .await;

        display.clear();

        Text::with_baseline(
            &format!("T: {:2.1}°C", temp_humidity.temperature),
            bounding_box.top_left,
            character_style,
            embedded_graphics::text::Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline(
            &format!("RH: {:2.1}%", temp_humidity.humidity),
            bounding_box.anchor_point(AnchorPoint::BottomLeft),
            character_style,
            embedded_graphics::text::Baseline::Bottom,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline(
            &format!("B: {:1.2}V", battery_voltage),
            bounding_box.anchor_point(AnchorPoint::TopCenter),
            character_style,
            embedded_graphics::text::Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline(
            &format!("P: {:.0}hPa", pressure / 100.0),
            bounding_box.anchor_point(AnchorPoint::BottomCenter),
            character_style,
            embedded_graphics::text::Baseline::Bottom,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();
    }
}
