#![no_std]
// #![no_main]

pub mod fmt;
pub use crate::fmt::*;
use assign_resources::assign_resources;
use embassy_stm32::{bind_interrupts, peripherals, usb_otg};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, pubsub::PubSubChannel};

pub mod display;
pub mod imu;
pub mod usb;

assign_resources! {
    usb: UsbResources {
        dp: PA12,
        dm: PA11,
        usb: USB_OTG_FS,
    }
    lcd: LcdResources {
        cs: PE11,
        sck: PE12,
        mosi: PE14,
        //txdma: DMA1_CH3,
        dc: PE15,
        bl: PD15,
        spi: SPI4,
    }
    imu: ImuResources {
        spi: SPI3,
        sck: PB3,
        mosi: PB5,
        miso: PB4,
        txdma: DMA1_CH3,
        rxdma: DMA1_CH4,
        cs: PB7,
        int1: PB8,
        exti:EXTI8
    }
}

bind_interrupts!(struct Irqs {
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

// if packed, it will cannot be used for Format
// #[repr(C, packed)]
#[repr(C)]
#[derive(Clone, Copy, defmt::Format)]
pub struct ImuDataType {
    pub frame_id: u64,
    pub timestamp: u64,
    pub data: [f32; 7],
}
//type ImuDataType = [f32;7];
pub static IMU_CHANNEL: PubSubChannel<ThreadModeRawMutex, ImuDataType, 32, 2, 1> =
    PubSubChannel::new();
