use arrayvec::ArrayVec;
use core::{
    cell::{Cell, RefCell},
    sync::atomic::Ordering,
};
use frunk::hlist::{HCons, HNil};
use fugit::ExtU32;
use keyberon::layout::{CustomEvent, Layout};
use rp2040_hal::{timer::Timer, usb::UsbBus};

use usb_device::{
    bus::UsbBusAllocator,
    class::UsbClass as _,
    device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbRev, UsbVidPid},
};
use usbd_human_interface_device::{device::DeviceClass, prelude::*, UsbHidError};
cfg_if::cfg_if! {
    if #[cfg(feature = "nkro")] {
        pub use usbd_human_interface_device::device::keyboard::{
            NKROBootKeyboard as Keyboard, NKROBootKeyboardConfig as KeyboardConfig,
        };
    } else {
        pub use usbd_human_interface_device::device::keyboard::{
            BootKeyboard as Keyboard, BootKeyboardConfig as KeyboardConfig,
        };
    }
}

use super::{layout, layout::CustomAction, utils_time, BlackBoard, Source};

type InterfaceList<'a> = HCons<Keyboard<'a, UsbBus>, HNil>;
type MyUsbHidClass<'a> = UsbHidClass<'a, UsbBus, InterfaceList<'a>>;

/// USB VID/PID for a generic keyboard from <https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt>
const VID_PID: UsbVidPid = UsbVidPid(0x16c0, 0x27db);
const DEVICE_RELEASE: u16 = 0x0100;

/// USB polling task
///
/// Handles:
/// - Processing the usb stack and associated protocols.
/// - Draining the keyboard event stack from the blackboard and feeds it into the layout
/// - Ticks the layout & generates HID reports
pub(crate) async fn usb_app<'a>(
    board: &BlackBoard,
    to_usb: &RefCell<crate::ToUSBStack>,
    timer: &Timer,
    usb_bus: &UsbBusAllocator<UsbBus>,
    mut keyboard_hid: MyUsbHidClass<'_>,
    #[cfg(feature = "cli")] usb_serial: &crate::cli::UsbSerialCell<'a>,
) {
    let BlackBoard { is_main_half, .. } = board;

    let Ok(usb_builder) = UsbDeviceBuilder::new(usb_bus, VID_PID)
        .supports_remote_wakeup(true)
        .device_class(0)
        .device_sub_class(0)
        .device_protocol(0)
        .usb_rev(UsbRev::Usb200)
        .device_release(DEVICE_RELEASE)
        .strings(&[StringDescriptors::default()
            .manufacturer("Ithinuel.me")
            .product("WilsKeeb")
            .serial_number("Dev Environment")])
        .and_then(|builder| builder.max_packet_size_0(64))
        .and_then(|builder| builder.max_power(500))
    else {
        panic!("Failed to configure UsbDeviceBuilder");
    };
    #[cfg(feature = "cli")]
    let usb_builder = usb_builder.composite_with_iads();
    let mut usb_dev = usb_builder.build();

    let mut layout = Layout::new(&layout::LAYERS);
    let mut timestamp = timer.get_counter_low();
    let mut previous_state = usb_dev.state();
    loop {
        utils_time::wait_for(timer, 10.micros()).await;

        #[cfg(feature = "cli")]
        if let Ok(mut usb_serial) = usb_serial.try_borrow_mut() {
            if usb_dev.poll(&mut [&mut keyboard_hid, &mut *usb_serial]) {
                keyboard_hid.poll();
                usb_serial.poll();
            }
        }
        #[cfg(not(feature = "cli"))]
        if usb_dev.poll(&mut [&mut keyboard_hid]) {
            keyboard_hid.poll();
        }

        let usb_state = usb_dev.state();
        if usb_state != previous_state {
            previous_state = usb_state;
            defmt::info!("USBState: {:?}", usb_state);

            board.usb_state.store(usb_state);
        }

        let _ = keyboard_hid.device().read_report();

        // if 1ms since last update
        let now = timer.get_counter_low();

        if is_main_half.load(Ordering::Relaxed) && now.wrapping_sub(timestamp) >= 1_000 {
            timestamp = now;

            // Process events from keypress
            for (source, mut event) in to_usb.borrow_mut().drain(..) {
                if source == Source::Right {
                    event = event.transform(|row, col| (row, 13 - col));
                }
                layout.event(event);
            }
            if let CustomEvent::Press(CustomAction::Bootldr) = layout.tick() {
                rp2040_hal::rom_data::reset_to_usb_boot(0, 0)
            }

            // Collect key presses.
            let keycodes: ArrayVec<_, 70> = layout.keycodes().collect();
            let has_keycodes = !keycodes.is_empty();

            // Setup the report for the control channel
            match keyboard_hid.device().write_report(keycodes) {
                Err(UsbHidError::WouldBlock) | Err(UsbHidError::Duplicate) | Ok(_) => {}
                Err(e) => panic!("Failed to write keyboard report: {:?}", e),
            }
            match keyboard_hid.device().tick() {
                Err(UsbHidError::WouldBlock) | Ok(_) => {}
                Err(e) => panic!("Failed to process keyboard tick: {:?}", e),
            }

            // Wake the host.
            if !has_keycodes
                && usb_state == UsbDeviceState::Suspend
                && usb_dev.remote_wakeup_enabled()
            {
                usb_dev.bus().remote_wakeup();
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, defmt::Format)]
pub struct AtomicUsbDeviceState(Cell<UsbDeviceState>);
impl AtomicUsbDeviceState {
    pub const fn new(state: UsbDeviceState) -> Self {
        AtomicUsbDeviceState(Cell::new(state))
    }
    pub fn load(&self) -> UsbDeviceState {
        self.0.get()
    }
    pub fn store(&self, state: UsbDeviceState) {
        self.0.replace(state);
    }
}
unsafe impl core::marker::Sync for AtomicUsbDeviceState {}
static_assertions::const_assert_eq!(core::mem::size_of::<UsbDeviceState>(), 1,);
