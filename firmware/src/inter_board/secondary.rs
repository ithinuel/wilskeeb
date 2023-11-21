use fugit::ExtU32;
use rp2040_hal::{i2c::peripheral::I2CEvent, pac};

use crate::TimerInstant;

use super::Error;

type I2CPeriph = rp2040_hal::i2c::peripheral::I2CPeripheralEventIterator<pac::I2C0, super::Pins>;

pub struct Secondary {
    i2c: I2CPeriph,
    transaction_start: bool,
    ptr: u8,
    start_ts: Option<TimerInstant>,
    configured: bool,
    timestamp: TimerInstant,
}
#[cfg(feature = "debug")]
impl defmt::Format for Secondary {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Secondary {{ tx_start: {}, ptr: {}, active: {}, configured: {} }}",
            self.transaction_start,
            self.ptr,
            self.start_ts,
            self.configured
        )
    }
}

impl Secondary {
    pub fn new(
        i2c_block: pac::I2C0,
        (sda, scl): super::Pins,
        resets: &mut pac::RESETS,
        timestamp: TimerInstant,
    ) -> Self {
        let i2c = rp2040_hal::i2c::I2C::new_peripheral_event_iterator(
            i2c_block,
            sda,
            scl,
            resets,
            super::ADDRESS,
        );
        Self {
            i2c,
            transaction_start: false,
            ptr: 0,
            start_ts: None,
            configured: false,
            timestamp,
        }
    }

    pub fn serve(
        &mut self,
        scanned: &core::cell::RefCell<crate::ScannedEventStack>,
        timestamp: TimerInstant,
    ) -> Result<(), Error> {
        // if last request was more than 1sec ago => drop configured state
        if timestamp >= (self.timestamp + 100.millis()) && self.configured {
            defmt::info!("Inter: timed out, configured state lost");
            self.configured = false;
        }

        // serve any pending request.
        let evt = self.i2c.next();
        #[cfg(feature = "debug")]
        if let Some(evt) = &evt {
            defmt::info!("Inter: {}", defmt::Debug2Format(evt));
        }
        match evt {
            None => {
                if let Some(ts) = self.start_ts {
                    if timestamp >= (ts + super::TRANSACTION_TIMEOUT) {
                        defmt::info!("Inter: {}: transaction timeout", timestamp);

                        self.start_ts = None;
                        self.transaction_start = false;
                        self.configured = false;

                        return Err(Error::Timeout);
                    }
                } else if timestamp >= (self.timestamp + super::COMM_TIMEOUT) {
                    return Err(Error::BusIdle);
                }
            }
            Some(_e @ (I2CEvent::Start | I2CEvent::Restart)) => {
                self.start_ts = Some(timestamp);
                self.transaction_start = true;
            }
            Some(I2CEvent::TransferWrite) => {
                let mut byte = 0;
                if self.i2c.read(core::slice::from_mut(&mut byte)) == 1 {
                    if self.transaction_start {
                        self.ptr = byte;
                        self.transaction_start = false;
                    } else {
                        match self.ptr {
                            0x80 => {
                                self.configured = byte == 1;
                                defmt::info!(
                                    "Inter: {}configured",
                                    if self.configured { "" } else { "not " }
                                )
                            }
                            0x81 => {
                                let mut scanned = scanned.borrow_mut();
                                defmt::info!(
                                    "Inter: write: {:x}: {} ({})",
                                    self.ptr,
                                    byte,
                                    scanned.len()
                                );
                                let drain = usize::from(byte).min(scanned.len());
                                scanned.drain(0..drain);
                            }
                            0x82 => {
                                defmt::info!("Inter: setting UI (not yet implemented)");
                                /* TODO: impl ui */
                            }
                            _ => {
                                defmt::info!("Inter: Out of range. Ignored");
                            }
                        }
                        if (0x80..=0x81).contains(&self.ptr) {
                            self.ptr += 1;
                        }
                    }
                }
            }
            Some(I2CEvent::TransferRead) => {
                match self.ptr {
                    0x00..=0x10 => {
                        let v: arrayvec::ArrayVec<u8, 16> = scanned
                            .borrow()
                            .iter()
                            .cloned()
                            .map(|event| {
                                let (row, col) = event.coord();
                                (row * super::COLUMN_COUNT + col + 1)
                                    | if event.is_press() { 0x80 } else { 0 }
                            })
                            .chain(core::iter::repeat(0))
                            .take(16)
                            .collect();
                        defmt::info!("Inter: read: {:x}: {:x}", self.ptr, v.as_slice());
                        self.i2c.write(v.as_slice())
                    }
                    0x80 => {
                        defmt::info!("Inter: read: {:x}: {}", self.ptr, self.configured);
                        self.i2c.write(&[if self.configured { 1 } else { 0 }])
                    }
                    _ => {
                        defmt::info!("Inter: read: {:x}: out of bound", self.ptr);
                        self.i2c.write(&[0])
                    }
                };
            }
            Some(I2CEvent::Stop) => {
                self.timestamp = timestamp;
                self.start_ts = None;
            }
        }
        Ok(())
    }

    pub fn release(self, resets: &mut pac::RESETS) -> (pac::I2C0, super::Pins) {
        let Secondary { i2c, .. } = self;
        i2c.free(resets)
    }
}
