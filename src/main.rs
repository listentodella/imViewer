#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::time::mhz;
use embassy_stm32::{
    dma::NoDma,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed},
};
use embassy_stm32::{spi, Config};
use fmt::{info, error};

// use embassy_stm32::usb::{Driver, Instance};
// use embassy_stm32::{bind_interrupts, peripherals, usb};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};

use display_interface_spi::SPIInterface;
use embassy_time::Duration;
use embassy_time::{Delay, Timer};
use embedded_graphics::image::{ImageRawLE, *};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::{Rgb565, Rgb888};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::{Alignment, LineHeight, Text, TextStyleBuilder};
use heapless::String;
use st7789::ST7789;


#[embassy_executor::task]
async fn blinky(pin: AnyPin) {
    let mut led = Output::new(pin, Level::High, Speed::Low);

    loop {
        led.toggle();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: mhz(25),
            mode: HseMode::Oscillator,
        });
        config.rcc.hsi = None;
        config.rcc.csi = false;

        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL160,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV4),
            //divr: None,
            divr: Some(PllDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        // config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    let p = embassy_stm32::init(config);

    spawner.spawn(blinky(p.PC13.degrade())).unwrap();

    // spawner.spawn(display_task(lcd)).unwrap();
    spawner.spawn(display_task()).unwrap();

    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(16);
    // PB7-CS, PB8-int1, PB9-int2
    let spi = spi::Spi::new(
        p.SPI3, p.PB3, p.PB5, p.PB4, p.DMA1_CH3, p.DMA1_CH4, spi_config,
    );
    let cs = Output::new(p.PB7, Level::High, Speed::High);
    let int1 = Input::new(p.PB8, Pull::Down);
    let int1 = embassy_stm32::exti::ExtiInput::new(int1, p.EXTI8);
    spawner.spawn(imu_task(spi, cs, int1)).unwrap();

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn imu_task(
    mut spi: spi::Spi<'static, peripherals::SPI3, peripherals::DMA1_CH3, peripherals::DMA1_CH4>,
    mut cs: Output<'static, peripherals::PB7>,
    mut int1: embassy_stm32::exti::ExtiInput<'static, peripherals::PB8>,
) {
    let buf = [0x11u8, 0x01];
    cs.set_low();
    spi.write(&buf).await.unwrap();
    cs.set_high();
    Timer::after_millis(200).await;

    cs.set_low();
    let buf = [0x75u8 | 0x80];
    let mut chipid = [0x00u8];
    spi.write(&buf).await.unwrap();
    spi.read(&mut chipid).await.unwrap();
    cs.set_high();
    if chipid[0] == 0x47u8 {
        info!("icm42688 detected!");
    } else {
        error!("unknown chip detected: {}", chipid[0]);
        return;
    }

    cs.set_low();
    let buf = [0x14u8, 0x03];
    spi.write(&buf).await.unwrap();

    let buf = [0x65u8, 0x08];
    spi.write(&buf).await.unwrap();

    let buf = [0x4eu8, 0x0f];
    spi.write(&buf).await.unwrap();

    cs.set_high();

    let combine = |msb: u8, lsb: u8| ((msb as u16) << 8 | lsb as u16) as i16;

    loop {
        let addr = [0x1du8 | 0x80];
        let mut buf = [0x00u8; 14];
        int1.wait_for_rising_edge().await;

        cs.set_low();
        spi.write(&addr).await.unwrap();
        spi.read(&mut buf).await.unwrap();
        cs.set_high();

        let tmp = combine(buf[0], buf[1]);
        let acc = [
            combine(buf[2], buf[3]),
            combine(buf[4], buf[5]),
            combine(buf[6], buf[7]),
        ];
        let gyr = [
            combine(buf[8], buf[9]),
            combine(buf[10], buf[11]),
            combine(buf[12], buf[13]),
        ];
        let temp = tmp as f32 / 132.48f32 + 25.0f32;
        let acc = [
            acc[0] as f32 * 16f32 * 9.81f32 / 32768f32,
            acc[1] as f32 * 16f32 * 9.81f32 / 32768f32,
            acc[2] as f32 * 16f32 * 9.81f32 / 32768f32,
        ];
        let gyr = [
            gyr[0] as f32 * 2000f32 / 32768.0f32,
            gyr[1] as f32 * 2000f32 / 32768.0f32,
            gyr[2] as f32 * 2000f32 / 32768.0f32,
        ];
        let imu = [acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], temp];
        // info!("{}", imu);
        // Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn display_task() {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(24);
    let spi = unsafe { peripherals::SPI4::steal() };
    let cs = Output::new(
        unsafe { peripherals::PE11::steal() },
        Level::Low,
        Speed::Low,
    );
    let dc = Output::new(
        unsafe { peripherals::PE15::steal() },
        Level::Low,
        Speed::Low,
    );
    let bl = Output::new(
        unsafe { peripherals::PD15::steal() },
        Level::Low,
        Speed::Low,
    );

    let spi = unsafe {
        spi::Spi::new_txonly(
            spi,
            peripherals::PE12::steal(),
            peripherals::PE14::steal(),
            peripherals::DMA1_CH3::steal(),
            NoDma,
            spi_config,
        )
    };

    let di = SPIInterface::new(spi, dc, cs);
    let mut lcd = ST7789::new(di, None::<Output<AnyPin>>, Some(bl), 240, 320);

    let mut delay = Delay;
    lcd.init(&mut delay).unwrap();
    lcd.set_orientation(st7789::Orientation::Landscape).unwrap();
    lcd.set_backlight(st7789::BacklightState::On, &mut delay)
        .unwrap();
    lcd.clear(Rgb565::BLACK).unwrap();




}
