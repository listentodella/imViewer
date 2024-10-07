// #![no_std]
// #![no_main]

use crate::fmt::{error, info};
use crate::ImuDataType;
use crate::IMU_CHANNEL;
use crate::ImuResources;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::spi;
use embassy_stm32::time::mhz;
use embassy_time::{Instant, Timer};

#[embassy_executor::task]
pub async fn imu_task(r: ImuResources) {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(16);
    let mut spi = spi::Spi::new(r.spi, r.sck, r.mosi, r.miso, r.txdma, r.rxdma, spi_config);
    let mut cs = Output::new(r.cs, Level::High, Speed::High);
    let int1 = Input::new(r.int1, Pull::Down);
    let mut int1 = embassy_stm32::exti::ExtiInput::new(int1, r.exti);

    let mut frame_id = 0u64;
    let publisher = IMU_CHANNEL.publisher().unwrap();
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
    let buf = [0x64u8, 0x00];
    spi.write(&buf).await.unwrap();
    cs.set_high();

    cs.set_low();
    let buf = [0x14u8, 0x03];
    spi.write(&buf).await.unwrap();
    cs.set_high();

    cs.set_low();
    let buf = [0x65u8, 0x08];
    spi.write(&buf).await.unwrap();
    cs.set_high();

    cs.set_low();
    let buf = [0x4eu8, 0x0f];
    spi.write(&buf).await.unwrap();
    cs.set_high();

    // cs.set_low();
    // let buf = [0x14u8 | 0x80];
    // let mut cfg = [0x00u8];
    // spi.write(&buf).await.unwrap();
    // spi.read(&mut cfg).await.unwrap();
    // info!("0x14 cfg: 0x{:02x}", cfg[0]);
    // cs.set_high();

    // cs.set_low();
    // let buf = [0x64u8 | 0x80];
    // let mut cfg = [0x00u8];
    // spi.write(&buf).await.unwrap();
    // spi.read(&mut cfg).await.unwrap();
    // info!("0x64 cfg: 0x{:02x}", cfg[0]);
    // cs.set_high();

    // cs.set_low();
    // let buf = [0x65u8 | 0x80];
    // let mut cfg = [0x00u8];
    // spi.write(&buf).await.unwrap();
    // spi.read(&mut cfg).await.unwrap();
    // info!("0x65 cfg: 0x{:02x}", cfg[0]);
    // cs.set_high();

    // cs.set_low();
    // let buf = [0x4eu8 | 0x80];
    // let mut cfg = [0x00u8];
    // spi.write(&buf).await.unwrap();
    // spi.read(&mut cfg).await.unwrap();
    // info!("0x4e cfg: 0x{:02x}", cfg[0]);
    // cs.set_high();

    let combine = |msb: u8, lsb: u8| ((msb as u16) << 8 | lsb as u16) as i16;

    loop {
        let addr = [0x1du8 | 0x80];
        let mut buf = [0x00u8; 14];
        int1.wait_for_rising_edge().await;
        frame_id += 1;

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

        let imu = ImuDataType {
            frame_id,
            timestamp: Instant::now().as_micros(),
            data: [acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], temp],
        };
        // this ensures that all subscribers will not miss the data in the queue
        // but it may block some subscribers once one of subscribers reads lately
        // publisher.publish(imu).await;
        // this ensures that any subscriber can recv the data without waiting for other subscribers
        publisher.publish_immediate(imu);
    }
}
