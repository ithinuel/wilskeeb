use crate::{Source, UsbSerialCell};
use usb_device::UsbError;

#[cfg(feature = "debug-to-cli")]
use panic_persist::get_panic_message_bytes;

#[cfg(not(feature = "debug"))]
use crate::defmt;

fn read_from_usb(usb_serial: &UsbSerialCell, buf: &mut [u8]) -> Result<usize, UsbError> {
    let serial = &mut *usb_serial.borrow_mut();

    match serial.read(buf) {
        Ok(len) => (Ok(len)),
        Err(UsbError::WouldBlock) => Ok(0),
        Err(e) => (Err(e)),
    }
}

pub fn update(
    usb_serial: &UsbSerialCell,
    source: Source,
    #[cfg(feature = "debug-to-cli")] consumer: &mut defmt_bbq::DefmtConsumer,
) {
    let mut buf = [0; 8];

    let len = read_from_usb(usb_serial, &mut buf).expect("Failed to read from usb serial port");

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
                #[cfg(all(feature = "debug", not(feature = "debug-to-cli")))]
                {
                    defmt::info!("Using panic_probe! Refer to defmt traces.");
                }
            }
            b'u' => rp2040_hal::rom_data::reset_to_usb_boot(0, 0),
            b's' => {
                defmt::info!("This is the {} half", source);
            }
            _ => {}
        }
    }

    #[cfg(feature = "debug-to-cli")]
    if let Ok(grant) = consumer.read() {
        let serial = &mut *usb_serial.borrow_mut();
        match serial.write(grant.buf()) {
            Ok(len) => {
                grant.release(len);
            }
            _ => { /* Ignore error: we have no way to recover from that. */ }
        }
    }
}
