use core::cell::RefCell;

use fugit::{HertzU32, MicrosDurationU32};
use usb_device::UsbError;

#[cfg(feature = "debug-to-cli")]
use panic_persist::get_panic_message_bytes;

pub type UsbSerialCell<'a> = RefCell<
    usbd_serial::SerialPort<
        'a,
        rp2040_hal::usb::UsbBus,
        [u8; USB_SERIAL_RX_SZ],
        [u8; USB_SERIAL_TX_SZ],
    >,
>;

/// USB Serial cli update frequency.
const CLI_FREQUENCY: HertzU32 = HertzU32::kHz(1);
pub const USB_SERIAL_RX_SZ: usize = 64;
pub const USB_SERIAL_TX_SZ: usize = 1024;

/// CLI task
///
/// Handles simple request from the usb-serial interface to help manage and debug the firmware.
/// see [`cli::update`] for a full description of the supported commands.
pub async fn cli_app<'a>(
    timer: &rp2040_hal::Timer,
    usb_serial: &UsbSerialCell<'a>,
    #[cfg(feature = "debug-to-cli")] mut consumer: defmt_bbq::DefmtConsumer,
) {
    const CLI_PERIOD: MicrosDurationU32 = CLI_FREQUENCY.into_duration();

    let mut next_scan_at = timer.get_counter() + CLI_PERIOD;
    loop {
        next_scan_at = crate::utils_time::wait_until(timer, next_scan_at).await + CLI_PERIOD;

        update(
            usb_serial,
            #[cfg(feature = "debug-to-cli")]
            &mut consumer,
        );
    }
}

fn read_from_usb(usb_serial: &UsbSerialCell, buf: &mut [u8]) -> Result<usize, UsbError> {
    let serial = &mut *usb_serial.borrow_mut();

    match serial.read(buf) {
        Ok(len) => Ok(len),
        Err(UsbError::WouldBlock) => Ok(0),
        Err(e) => Err(e),
    }
}

pub fn update(
    usb_serial: &UsbSerialCell,
    #[cfg(feature = "debug-to-cli")] consumer: &mut defmt_bbq::DefmtConsumer,
) {
    let mut buf = [0; 8];

    let Ok(len) = read_from_usb(usb_serial, &mut buf) else {
        panic!("Failed to read from usb serial port");
    };

    for b in &buf[..len] {
        match b {
            b'r' => cortex_m::peripheral::SCB::sys_reset(),
            b'd' => {
                #[cfg(feature = "debug-to-cli")]
                {
                    let msg = get_panic_message_bytes().unwrap_or(b"No panic message available.");
                    match core::str::from_utf8(msg) {
                        Ok(msg) => defmt::info!("{}", msg),
                        Err(_) => defmt::info!("Corrupted panic message"),
                    }
                }
                #[cfg(feature = "debug-to-probe")]
                defmt::info!("Please refer to the rtt output of the debug probe.");
            }
            b'u' => rp2040_hal::rom_data::reset_to_usb_boot(0, 0),
            b's' => {
                defmt::info!("This is the {} half", crate::read_side());
            }
            _ => {}
        }
    }

    #[cfg(feature = "debug-to-cli")]
    if let Ok(grant) = consumer.read() {
        let serial = &mut *usb_serial.borrow_mut();
        if let Ok(len) = serial.write(grant.buf()) {
            grant.release(len);
            { /* Ignore error: we have no way to recover from that. */ }
        }
    }
}
