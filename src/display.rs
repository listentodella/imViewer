// #![no_std]
// #![no_main]

use crate::LcdResources;
use core::fmt::Write;
use display_interface_spi::SPIInterface;
use embassy_stm32::spi;
use embassy_stm32::time::mhz;
use embassy_stm32::{
    dma::NoDma,
    gpio::{AnyPin, Level, Output, Speed},
};
use embassy_time::Delay;
use embedded_graphics::image::{ImageRawLE, *};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::Text;
use heapless::String;
use st7789::ST7789;

// use crate::fmt::{error, info};
use crate::IMU_CHANNEL;

#[embassy_executor::task]
pub async fn display_task(r: LcdResources) {
    let mut subscriber = IMU_CHANNEL.subscriber().unwrap();
    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(100);
    let cs = Output::new(r.cs, Level::Low, Speed::Low);
    let dc = Output::new(r.dc, Level::Low, Speed::Low);
    let bl = Output::new(r.bl, Level::Low, Speed::Low);

    let spi = spi::Spi::new_txonly(
        r.spi, r.sck, r.mosi, NoDma, //r.txdma,
        NoDma, spi_config,
    );

    let di = SPIInterface::new(spi, dc, cs);
    let mut lcd = ST7789::new(di, None::<Output<AnyPin>>, Some(bl), 240, 320);

    let mut delay = Delay;
    lcd.init(&mut delay).unwrap();
    lcd.set_orientation(st7789::Orientation::Landscape).unwrap();
    lcd.set_backlight(st7789::BacklightState::On, &mut delay)
        .unwrap();
    lcd.clear(Rgb565::BLACK).unwrap();

    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(0, 0));
    ferris.draw(&mut lcd).unwrap();

    let large_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    let mut_style = MonoTextStyle::new(&FONT_10X20, Rgb565::YELLOW);
    let rectangle = Rectangle::new(Point::new(0, 100), Size::new(240, 100));

    let mut filter = 0u64;
    let mut text: String<32> = String::new();
    loop {
        let imu = subscriber.next_message_pure().await;
        // trace!("imu: {},", imu);
        filter += 1;
        if filter % 500 == 0 {
            lcd.fill_solid(&rectangle, Rgb565::BLACK).unwrap();
            // Draw the first text at (20, 30) using the small character style.
            let fid_pos = Text::new("frameid: ", Point::new(0, 120), large_style)
                .draw(&mut lcd)
                .unwrap();
            // Draw the second text after the first text using the large character style.
            let ts_pos = Text::new("timestamp: ", Point::new(0, 140), large_style)
                .draw(&mut lcd)
                .unwrap();
            let acc_pos = Text::new("acc: ", Point::new(0, 160), large_style)
                .draw(&mut lcd)
                .unwrap();
            let gyr_pos = Text::new("gyr: ", Point::new(0, 180), large_style)
                .draw(&mut lcd)
                .unwrap();
            let tmp_pos = Text::new("temp: ", Point::new(0, 200), large_style)
                .draw(&mut lcd)
                .unwrap();

            text.clear();
            text.write_fmt(format_args!("{}", imu.frame_id)).unwrap();
            let _ = Text::new(text.as_str(), fid_pos, mut_style)
                .draw(&mut lcd)
                .unwrap();

            text.clear();
            text.write_fmt(format_args!("{}", imu.timestamp)).unwrap();
            let _ = Text::new(text.as_str(), ts_pos, mut_style)
                .draw(&mut lcd)
                .unwrap();

            text.clear();
            text.write_fmt(format_args!(
                "{:.3},{:.3},{:.3}",
                imu.data[0], imu.data[1], imu.data[2]
            ))
            .unwrap();
            let _ = Text::new(text.as_str(), acc_pos, mut_style)
                .draw(&mut lcd)
                .unwrap();

            text.clear();
            text.write_fmt(format_args!(
                "{:.3},{:.3},{:.3}",
                imu.data[3], imu.data[4], imu.data[5]
            ))
            .unwrap();
            let _ = Text::new(text.as_str(), gyr_pos, mut_style)
                .draw(&mut lcd)
                .unwrap();

            text.clear();
            text.write_fmt(format_args!("{:.3}", imu.data[6])).unwrap();
            let _ = Text::new(text.as_str(), tmp_pos, mut_style)
                .draw(&mut lcd)
                .unwrap();
        }
    }
}
