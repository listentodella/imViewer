#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Pin,AnyPin, Level, Output, Speed};
use embassy_time::{Duration, Timer};
use fmt::info;

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
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(blinky(p.PC13.degrade())).unwrap();

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
