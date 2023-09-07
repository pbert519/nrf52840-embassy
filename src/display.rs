use core::fmt::Debug;

use crate::MessageSubscriber;
use embassy_futures::{join::join3, select::select};
use embassy_nrf::{
    gpiote::InputChannel,
    peripherals::{GPIOTE_CH0, TWISPI1},
    twim::Twim,
};
use embedded_graphics::{
    geometry::AnchorPoint,
    mono_font::{iso_8859_1::FONT_6X13, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::Text,
};

enum DisplayMode {
    EnviormentData,
    MicData,
}

#[embassy_executor::task]
pub async fn display(
    twim_dev: Twim<'static, TWISPI1>,
    mut msg_sub: MessageSubscriber,
    input: InputChannel<'static, GPIOTE_CH0, embassy_nrf::gpio::AnyPin>,
) {
    let mut display = ssd1306::Ssd1306::new(
        ssd1306::I2CDisplayInterface::new(twim_dev),
        ssd1306::size::DisplaySize128x32,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    ssd1306::prelude::DisplayConfig::init(&mut display).unwrap();

    let mut mode = DisplayMode::EnviormentData;

    loop {
        let disp_fut = async {
            loop {
                let _ = display.clear(BinaryColor::Off);

                match mode {
                    DisplayMode::EnviormentData => {
                        display_enviorment_data(&mut display, &mut msg_sub).await
                    }

                    DisplayMode::MicData => display_mic_data(&mut display, &mut msg_sub).await,
                };

                display.flush().unwrap();
            }
        };

        select(disp_fut, input.wait()).await;

        mode = match mode {
            DisplayMode::EnviormentData => DisplayMode::MicData,
            DisplayMode::MicData => DisplayMode::EnviormentData,
        }
    }
}

async fn display_enviorment_data<DISP>(display: &mut DISP, msg_sub: &mut MessageSubscriber)
where
    DISP: embedded_graphics::draw_target::DrawTarget<Color = BinaryColor>,
    DISP::Error: Debug,
{
    let (temp_humidity, pressure, battery_voltage) = join3(
        msg_sub.temp_humidity_sub.next_message_pure(),
        msg_sub.pressure_sub.next_message_pure(),
        msg_sub.battery_voltage_sub.next_message_pure(),
    )
    .await;

    let bounding_box = display.bounding_box();

    let character_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X13)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        &format!("T: {:2.1}°C", temp_humidity.temperature),
        bounding_box.top_left,
        character_style,
        embedded_graphics::text::Baseline::Top,
    )
    .draw(display)
    .unwrap();

    Text::with_baseline(
        &format!("RH: {:2.1}%", temp_humidity.humidity),
        bounding_box.anchor_point(AnchorPoint::BottomLeft),
        character_style,
        embedded_graphics::text::Baseline::Bottom,
    )
    .draw(display)
    .unwrap();

    Text::with_baseline(
        &format!("B: {:1.2}V", battery_voltage),
        bounding_box.anchor_point(AnchorPoint::TopCenter),
        character_style,
        embedded_graphics::text::Baseline::Top,
    )
    .draw(display)
    .unwrap();

    Text::with_baseline(
        &format!("P: {:.0}hPa", pressure / 100.0),
        bounding_box.anchor_point(AnchorPoint::BottomCenter),
        character_style,
        embedded_graphics::text::Baseline::Bottom,
    )
    .draw(display)
    .unwrap();
}

async fn display_mic_data<DISP>(display: &mut DISP, msg_sub: &mut MessageSubscriber)
where
    DISP: embedded_graphics::draw_target::DrawTarget<Color = BinaryColor>,
    DISP::Error: Debug,
{
    let amps = msg_sub.mic_sub.next_message_pure().await;

    for (pos, amp) in amps.into_iter().enumerate() {
        let height = (amp / 100) as i32;
        let pos = pos as i32;
        Line::new(Point { x: pos, y: 0 }, Point { x: pos, y: height })
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display)
            .unwrap();
    }
}
