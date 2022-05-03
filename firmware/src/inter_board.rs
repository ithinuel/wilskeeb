use core::cell::RefCell;

use arrayvec::ArrayVec;
use defmt::info;
use embedded_hal_async::i2c::I2c;
use embedded_time::{duration::Extensions, rate::Hertz};
use futures::FutureExt;
use rp2040_async_i2c::AsyncI2C;
use rp2040_hal::{
    gpio::{bank0, FunctionI2C, Pin},
    i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator},
    pac::{self, I2C0},
    timer::Timer,
};

use super::{ScannedEventStack, Source};
use crate::{async_utils, ToUSBStack};

type Pins<Mode> = (Pin<bank0::Gpio0, Mode>, Pin<bank0::Gpio1, Mode>);
type I2CPeriph = I2CPeripheralEventIterator<I2C0, Pins<FunctionI2C>>;

/// I2C address for inter keyboard communication.
const ADDRESS: u16 = 0x055;
const INTER_BOARD_FREQ: Hertz = Hertz(50_000);
const TRANSACTION_TIMEOUT: u32 = 10_000;
const COMM_TIMEOUT: u32 = 50_000;

pub struct Main {
    i2c: AsyncI2C<I2C0, Pins<FunctionI2C>>,
    attached: bool,
}
impl defmt::Format for Main {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Main {{…}}")
    }
}

pub struct Secondary {
    i2c: I2CPeriph,
    transaction_start: bool,
    ptr: u8,
    start_ts: Option<u32>,
    configured: bool,
    timestamp: u32,
}
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
pub enum Error {
    BusIdle,
    Timeout,
    BusError(rp2040_hal::i2c::Error),
    QueueFull,
}
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        use embedded_hal::i2c::Error as _;
        use rp2040_hal::i2c::Error;
        match self {
            Self::BusIdle => defmt::write!(fmt, "BusIdle"),
            Self::Timeout => defmt::write!(fmt, "Timeout"),
            Self::QueueFull => defmt::write!(fmt, "QueueFull"),
            Self::BusError(err) => match err {
                Error::Abort(_) => {
                    defmt::write!(fmt, "BusError(Abort({}))", defmt::Debug2Format(&err.kind()))
                }
                err => defmt::write!(fmt, "BusError({})", err),
            },
        }
    }
}
impl From<rp2040_hal::i2c::Error> for Error {
    fn from(err: rp2040_hal::i2c::Error) -> Self {
        Error::BusError(err)
    }
}

struct Timeout;

impl Main {
    pub fn new(
        i2c_block: pac::I2C0,
        (sda, scl): Pins<FunctionI2C>,
        resets: &mut pac::RESETS,
        system_clock_freq: Hertz,
    ) -> Self {
        let i2c = rp2040_async_i2c::AsyncI2C::new(
            i2c_block,
            sda,
            scl,
            INTER_BOARD_FREQ,
            resets,
            system_clock_freq,
        );
        Main {
            i2c,
            attached: false,
        }
    }

    pub async fn poll(
        &mut self,
        to_usb: &RefCell<ToUSBStack<32>>,
        timer: &Timer,
        source: Source,
    ) -> Result<(), Error> {
        if !self.attached {
            let mut count = 0u32;
            loop {
                info!("new main: attempt {}", count);
                let res = futures::select_biased! {
                    res = self.i2c.write(ADDRESS, &[0x80, 0x01]).fuse() => Ok(res),
                    _ = async_utils::wait_for(timer, 1_000.microseconds()).fuse() => Err(Timeout)
                };
                // try to configure secondary
                break match res {
                    Ok(Ok(_)) => {
                        self.attached = true;
                        Ok(())
                    }
                    // if left side: retry
                    Ok(Err(_)) if count < 5 => {
                        count += 1;
                        let delay = if source == Source::Right {
                            10_000.microseconds()
                        } else {
                            5_000.microseconds()
                        };
                        super::async_utils::wait_for(timer, delay).await;
                        continue;
                    }
                    Ok(_) | Err(Timeout) => Err(Error::Timeout),
                };
            }
        } else {
            let mut health = 5;

            let _timestamp = timer.get_counter_low();
            let mut keypresses = [0; 16];
            loop {
                match futures::select_biased! {
                    res = self.i2c.write_read(ADDRESS, &[0x00], &mut keypresses).fuse() => Ok(res),
                    _ = async_utils::wait_for(timer, 5_000.microseconds()).fuse() => Err(Timeout)
                } {
                    Ok(res) => break res?,
                    Err(Timeout) if health > 0 => health -= 1,
                    Err(Timeout) => return Err(Error::Timeout),
                }
            }

            use keyberon::layout::Event;
            let mut to_usb = to_usb.borrow_mut();
            let before = to_usb.len();
            // we don't really care about errors, that might just be because of events buffer full
            // We don't have recovery procedures anyway
            keypresses
                .into_iter()
                .take_while(|&b| b != 0)
                .map(|evt| {
                    let evt = evt - 1;
                    let pressed = (evt & 0x80) == 0x80;
                    let (row, col) = num_integer::div_rem(evt & 0x7F, 14);
                    let conv = if pressed {
                        Event::Press(row, col)
                    } else {
                        Event::Release(row, col)
                    };
                    info!(
                        "{}: {:x} => {}",
                        _timestamp,
                        evt,
                        defmt::Debug2Format(&conv)
                    );
                    conv
                })
                .try_for_each(|evt| to_usb.push_back((source, evt)))
                .map_err(|_| Error::QueueFull)?;
            let cnt = to_usb.len() - before;
            drop(to_usb);

            // TODO: actually read that from the UI
            let led = 0;
            // TODO: failure here suck as this means we may get the same event twice and we don't want
            // that.
            let mut frame: ArrayVec<u8, 3> = ArrayVec::new();
            if cnt != 0 {
                frame.push(0x81);
                frame.push(cnt as u8);
            } else {
                frame.push(0x82);
            };
            frame.push(led);

            health = 5;
            loop {
                match futures::select_biased! {
                    res = self.i2c.write(ADDRESS, &frame).fuse() => Ok(res),
                    _ = async_utils::wait_for(timer, 7_000.microseconds()).fuse() => Err(Timeout)
                } {
                    Ok(res) => break res?,
                    Err(Timeout) if health > 0 => health -= 1,
                    Err(Timeout) => return Err(Error::Timeout),
                }
            }

            Ok(())
        }
    }

    pub fn release(self, resets: &mut pac::RESETS) -> (pac::I2C0, Pins<FunctionI2C>) {
        let Main { i2c, .. } = self;
        i2c.free(resets)
    }
}

impl Secondary {
    pub fn new(
        i2c_block: pac::I2C0,
        (sda, scl): Pins<FunctionI2C>,
        resets: &mut pac::RESETS,
        timestamp: u32,
    ) -> Self {
        let i2c = rp2040_hal::i2c::I2C::new_peripheral_event_iterator(
            i2c_block, sda, scl, resets, ADDRESS,
        );
        info!("{}: new secondary", timestamp);
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
        scanned: &RefCell<ScannedEventStack<16>>,
        timestamp: u32,
    ) -> Result<(), Error> {
        // if last request was more than 1sec ago => drop configured state
        if timestamp.wrapping_sub(self.timestamp) >= 100_000 {
            if self.configured {
                info!("{}: timed out, configured state lost", timestamp);
                self.configured = false;
            }
        }

        // serve any pending request.
        let evt = self.i2c.next();
        if let Some(evt) = &evt {
            info!("{}: {}", timestamp, defmt::Debug2Format(evt));
        }
        match evt {
            None => {
                if let Some(ts) = self.start_ts {
                    if timestamp.wrapping_sub(ts) >= TRANSACTION_TIMEOUT {
                        info!("{}: transaction timeout", timestamp);

                        self.start_ts = None;
                        self.transaction_start = false;
                        self.configured = false;

                        return Err(Error::Timeout);
                    }
                } else if timestamp.wrapping_sub(self.timestamp) >= COMM_TIMEOUT {
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
                                info!(
                                    "{}: {}configured",
                                    timestamp,
                                    if self.configured { "" } else { "not " }
                                )
                            }
                            0x81 => {
                                let mut scanned = scanned.borrow_mut();
                                info!(
                                    "{}: write: {:x}: {} ({})",
                                    timestamp,
                                    self.ptr,
                                    byte,
                                    scanned.len()
                                );
                                let drain = usize::from(byte).min(scanned.len());
                                scanned.drain(0..drain);
                            }
                            0x82 => {
                                info!("{}: setting UI (not yet implemented)", timestamp);
                                /* TODO: impl ui */
                            }
                            _ => {
                                info!("{}: Out of range. Ignored", timestamp);
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
                        let v: ArrayVec<u8, 16> = scanned
                            .borrow()
                            .iter()
                            .cloned()
                            .map(|event| {
                                let (row, col) = event.coord();
                                (row * 14 + col + 1) | if event.is_press() { 0x80 } else { 0 }
                            })
                            .chain(core::iter::repeat(0))
                            .take(16)
                            .collect();
                        info!("{}: read: {:x}: {:x}", timestamp, self.ptr, v.as_slice());
                        self.i2c.write(v.as_slice())
                    }
                    0x80 => {
                        info!("{}: read: {:x}: {}", timestamp, self.ptr, self.configured);
                        self.i2c.write(&[if self.configured { 1 } else { 0 }])
                    }
                    _ => {
                        info!("{}: read: {:x}: out of bound", timestamp, self.ptr);
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

    pub fn release(self, resets: &mut pac::RESETS) -> (pac::I2C0, Pins<FunctionI2C>) {
        let Secondary { i2c, .. } = self;
        i2c.free(resets)
    }
}
