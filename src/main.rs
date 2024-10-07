#![no_std]
#![no_main]

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};
use defmt::info;

// use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Speed};
use embassy_stm32::time::mhz;
use embassy_stm32::Config;

use imViewer::display::display_task;
use imViewer::imu::imu_task;
use imViewer::usb::usb_task;
use imViewer::{split_resources, AssignedResources, ImuResources, LcdResources, UsbResources};

// use embassy_futures::join::join;
// use embassy_stm32::{bind_interrupts, peripherals, usb_otg};

use embassy_time::Duration;
use embassy_time::Timer;

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
    let r = split_resources!(p);

    spawner.spawn(blinky(p.PC13.degrade())).unwrap();
    spawner.spawn(usb_task(r.usb)).unwrap();
    spawner.spawn(display_task(r.lcd)).unwrap();
    spawner.spawn(imu_task(r.imu)).unwrap();

    loop {
        info!("Hello Imu!");
        Timer::after(Duration::from_millis(1000)).await;
    }
}
