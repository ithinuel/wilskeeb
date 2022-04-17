use keyberon::{
    hid::{HidClass, ReportType},
    layout::CustomEvent,
};
use usb_device::class_prelude::UsbBusAllocator;

use crate::layout::CustomAction;

pub type MediaHIDClass<'a, B> = HidClass<'a, B, MediaDevice>;

#[rustfmt::skip]
const MEDIA_DESCRIPTOR: &[u8] = &[
    0x05, 0x0C, // Usage page: Consumer Device
    0x09, 0x01, // Usage: Consumer Control
    0xA1, 0x01, // Collection: Application
    0x05, 0x0C, //   Usage page: Consumer Device
    0x09, 0xE9, //   Usage: Volume Up
    0x09, 0xEA, //   Usage: Volume Down
    0x09, 0xE2, //   Usage: Mute
    0x09, 0xB5, //   Usage: Scan Next Track
    0x09, 0xB6, //   Usage: Scan Prev Track
    0x09, 0xB1, //   Usage: Pause
    0x09, 0xB0, //   Usage: Play
    0x15, 0x00, //   Logical minimum: 0
    0x25, 0x01, //   Logical maximum: 1
    0x75, 0x01, //   Report size: 1
    0x95, 0x07, //   Report count: 7
    0x81, 0x02, //   Input (Data, Var, Abs, NoWrap, Linear, PrefState, NoNull, NonVolatile, BitField)
    0x75, 0x01, //   Report size: 1
    0x95, 0x01, //   Report count: 1
    0x81, 0x03, //   Input (Const, Var, Abs, NoWrap, Linear, PrefState, NoNull, NonVolatile, BitField)
    0xC0        // End Collection
];

pub fn new_class<'alloc, 'state, B>(
    alloc: &'alloc UsbBusAllocator<B>,
    ui: &'state crate::ui::State,
) -> MediaHIDClass<'alloc, B>
where
    B: usb_device::bus::UsbBus,
{
    MediaHIDClass::new(MediaDevice::new(ui), alloc)
}

pub struct MediaReport(u8);
impl MediaReport {
    pub fn as_bytes(&self) -> &[u8] {
        core::slice::from_ref(&self.0)
    }
}

pub struct MediaDevice(u8);
impl MediaDevice {
    fn new(_ui: &crate::ui::State) -> Self {
        MediaDevice(0)
    }

    pub fn report(&self) -> MediaReport {
        MediaReport(self.0)
    }

    pub fn update(&mut self, event: CustomEvent<CustomAction>) {
        //const PLAY_MASK: u8 = 1 << 6;
        const PREV_MASK: u8 = 1 << 4;
        const NEXT_MASK: u8 = 1 << 3;

        use CustomEvent::*;
        match event {
            //Press(CustomAction::PlayPause) => self.0 |= PLAY_MASK,
            //Release(CustomAction::PlayPause) => self.0 &= !PLAY_MASK,
            Press(CustomAction::PrevTrack) => self.0 |= PREV_MASK,
            Release(CustomAction::PrevTrack) => self.0 &= !PREV_MASK,
            Press(CustomAction::NextTrack) => self.0 |= NEXT_MASK,
            Release(CustomAction::NextTrack) => self.0 &= !NEXT_MASK,
            _ => {}
        }
    }
}
impl keyberon::hid::HidDevice for MediaDevice {
    fn subclass(&self) -> keyberon::hid::Subclass {
        keyberon::hid::Subclass::None
    }

    fn protocol(&self) -> keyberon::hid::Protocol {
        keyberon::hid::Protocol::None
    }

    fn max_packet_size(&self) -> u16 {
        1
    }

    fn report_descriptor(&self) -> &[u8] {
        MEDIA_DESCRIPTOR
    }

    fn set_report(
        &mut self,
        _report_type: ReportType,
        _report_id: u8,
        _data: &[u8],
    ) -> Result<(), ()> {
        Ok(())
        //Err(())
    }

    fn get_report(&mut self, report_type: ReportType, _report_id: u8) -> Result<&[u8], ()> {
        match report_type {
            ReportType::Input => Ok(core::slice::from_ref(&self.0)),
            _ => Err(()),
        }
    }
}
