// #![no_std]
// #![no_main]

use crate::UsbResources;
use embassy_futures::join::join;

use embassy_stm32::usb_otg::{Config, Driver, Instance};
use embassy_time::Timer;
#[cfg(feature = "cdc")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
#[cfg(feature = "hid")]
use embassy_usb::class::hid::{HidWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};

use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

use crate::fmt::{error, info, warn};
use crate::ImuDataType;
use crate::Irqs;
use crate::IMU_CHANNEL;

#[embassy_executor::task]
pub async fn usb_task(r: UsbResources) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = false;

    let driver = Driver::new_fs(r.usb, Irqs, r.dp, r.dm, &mut ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    #[cfg(feature = "hid")]
    let mut request_handler = MyRequestHandler {};

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    #[cfg(feature = "hid")]
    {
        // Create classes on the builder.
        let config = embassy_usb::class::hid::Config {
            report_descriptor: MouseReport::desc(),
            request_handler: Some(&mut request_handler),
            poll_ms: 60,
            max_packet_size: 8,
        };

        let mut writer = HidWriter::<_, 5>::new(&mut builder, &mut state, config);

        // Build the builder.
        let mut usb = builder.build();

        // Run the USB device.
        let usb_fut = usb.run();

        // Do stuff with the class!
        let hid_fut = async {
            let mut y: i8 = 100;
            loop {
                Timer::after_millis(100).await;

                y = -y;
                let report = MouseReport {
                    buttons: 0,
                    x: 0,
                    y,
                    wheel: 0,
                    pan: 0,
                };
                match writer.write_serialize(&report).await {
                    Ok(()) => {}
                    Err(e) => warn!("Failed to send report: {:?}", e),
                }
            }
        };

        // Run everything concurrently.
        // If we had made everything `'static` above instead, we could do this using separate tasks instead.
        join(usb_fut, hid_fut).await;
    }

    // Create classes on the builder.
    #[cfg(feature = "cdc")]
    {
        let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

        // Build the builder.
        let mut usb = builder.build();

        // Run the USB device.
        let usb_fut = usb.run();

        // Do stuff with the class!
        let echo_fut = async {
            loop {
                class.wait_connection().await;
                info!("Connected");
                let _ = echo(&mut class).await;
                info!("Disconnected");
            }
        };

        // Run everything concurrently.
        // If we had made everything `'static` above instead, we could do this using separate tasks instead.
        join(usb_fut, echo_fut).await;
    }
}

#[cfg(feature = "cdc")]
struct Disconnected {}

#[cfg(feature = "cdc")]
impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[cfg(feature = "cdc")]
async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut subscriber = IMU_CHANNEL.subscriber().unwrap();
    loop {
        let imu = subscriber.next_message_pure().await;
        let data = unsafe {
            core::mem::transmute::<&ImuDataType, &[u8; core::mem::size_of::<ImuDataType>()]>(&imu)
        };
        class.write_packet(data).await?;
    }
}

#[cfg(feature = "hid")]
struct MyRequestHandler {}

#[cfg(feature = "hid")]
impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}
