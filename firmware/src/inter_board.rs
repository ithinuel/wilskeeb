//! Secondary register's map:
//!
//!
//! Read:
//! read on 0x00-0x7F do not change PTR.
//! read on 0x80-0xFF do increment PTR. 0xFF rolls over to 0x80.
//!
//! Write:
//! The first by written is always "PTR"
//!
//! 0x00: Event FIFO
//! 0x80: 7-1 reserved
//!       0   configured
//! 0x81: 7-0 led status

use core::cell::RefCell;

use fugit::{ExtU32, HertzU32, MicrosDurationU32};
#[cfg(feature = "debug")]
use keyberon::layout::Event;
use rp2040_hal::{
    clocks::SystemClock,
    gpio::{bank0, FunctionI2C, Pin},
    pac,
    timer::Timer,
};
use usb_device::device::UsbDeviceState;

#[cfg(not(feature = "debug"))]
use crate::defmt;
use crate::{read_side, utils_async, ScannedEventStack, Source, TimerInstant, ToUSBStack};

use self::{main::Main, secondary::Secondary};
mod main;
mod secondary;

type Pins = (
    Pin<bank0::Gpio0, FunctionI2C>,
    Pin<bank0::Gpio1, FunctionI2C>,
);

/// I2C address for inter keyboard communication.
const ADDRESS: u16 = 0x055;
const INTER_BOARD_FREQ: HertzU32 = HertzU32::from_raw(50_000);
const TRANSACTION_TIMEOUT: MicrosDurationU32 = MicrosDurationU32::millis(10);
const COMM_TIMEOUT: MicrosDurationU32 = MicrosDurationU32::millis(50);
const COLUMN_COUNT: u8 = 14;

#[cfg(feature = "debug")]
pub struct EventWrapper(pub Event);
#[cfg(feature = "debug")]
impl defmt::Format for EventWrapper {
    fn format(&self, fmt: defmt::Formatter) {
        let (op, r, c) = match self.0 {
            Event::Press(r, c) => (defmt::intern!("Press"), r, c),
            Event::Release(r, c) => (defmt::intern!("Release"), r, c),
        };
        defmt::write!(fmt, "{}({}, {})", op, r, c)
    }
}

pub enum Error {
    BusIdle,
    Timeout,
    BusError(rp2040_hal::i2c::Error),
    QueueFull,
}
#[cfg(feature = "debug")]
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        use embedded_hal_async::i2c::Error as _;
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

enum State {
    Main(Main),
    Secondary(Secondary),
}
impl From<Main> for State {
    fn from(val: Main) -> Self {
        State::Main(val)
    }
}
impl From<Secondary> for State {
    fn from(val: Secondary) -> Self {
        State::Secondary(val)
    }
}
pub struct InterBoard<'clk> {
    state: State,
    resets: pac::RESETS,
    system_clock: &'clk SystemClock,
    side: Source,
}
impl<'clk> InterBoard<'clk> {
    const CONFIGURED: [UsbDeviceState; 2] = [UsbDeviceState::Configured, UsbDeviceState::Suspend];

    pub fn new(
        system_clock: &'clk SystemClock,
        i2c_block: pac::I2C0,
        (sda, scl): Pins,
        mut resets: pac::RESETS,
        timestamp: TimerInstant,
    ) -> Self {
        Self {
            state: Secondary::new(i2c_block, (sda, scl), &mut resets, timestamp).into(),
            resets,
            system_clock,
            side: read_side(),
        }
    }
    pub async fn poll(
        self,
        usb_state: UsbDeviceState,
        timer: &Timer,
        scanned: &RefCell<ScannedEventStack>,
        to_usb: &RefCell<ToUSBStack>,
    ) -> InterBoard<'clk> {
        let Self {
            mut state,
            mut resets,
            system_clock,
            side,
        } = self;
        let timestamp = timer.get_counter();
        state = match state {
            State::Main(mut main) => {
                if !Self::CONFIGURED.contains(&usb_state) {
                    let (i2c_block, pins) = main.release(&mut resets);
                    defmt::info!("Inter: USB nolonger configured, switching back to secondary");
                    Secondary::new(i2c_block, pins, &mut resets, timestamp).into()
                } else {
                    let (state, delay) = match main.poll(to_usb, timer, !side).await {
                        Ok(_) => (main.into(), 1_000.micros()),
                        Err(_e) => {
                            defmt::info!("Inter: Main::poll error: {}", _e);
                            let (i2c_block, pins) = main.release(&mut resets);
                            (
                                Secondary::new(i2c_block, pins, &mut resets, timestamp).into(),
                                500.micros(),
                            )
                        }
                    };

                    // scope the borrows
                    {
                        let mut scanned = scanned.borrow_mut();
                        let mut to_usb = to_usb.borrow_mut();
                        while let Some(evt) = scanned.pop_front() {
                            if to_usb.push_back((side, evt)).is_err() {
                                defmt::info!("Inter: Main: to_usb push failed");
                                break;
                            }
                        }
                    }

                    utils_async::wait_for(timer, delay).await;
                    state
                }
            }
            State::Secondary(mut secondary) => {
                match secondary.serve(scanned, timestamp) {
                    Err(Error::BusIdle) if usb_state == UsbDeviceState::Configured => {
                        defmt::info!("Inter: USB Configured");

                        let (i2c_block, pins) = secondary.release(&mut resets);
                        Main::new(i2c_block, pins, &mut resets, system_clock).into()
                    }
                    // If the bus becomes idle (either during a transaction or outside of a
                    // transaction) then reset the bus to clear any unexpected clock stretching.
                    Err(Error::BusIdle) | Err(Error::Timeout) => {
                        let (i2c_block, pins) = secondary.release(&mut resets);
                        utils_async::wait_for(timer, 1_000.micros()).await;
                        Secondary::new(i2c_block, pins, &mut resets, timer.get_counter()).into()
                    }
                    _ => {
                        // lets be nice
                        utils_async::wait_for(timer, 10.micros()).await;
                        secondary.into()
                    }
                }
            }
        };
        Self {
            state,
            resets,
            system_clock,
            side,
        }
    }
    pub fn is_main(&self) -> bool {
        matches!(self.state, State::Main(_))
    }
}
