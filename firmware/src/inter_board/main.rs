use embedded_hal_async::i2c::I2c;
use fugit::ExtU32;
use futures::FutureExt;
use rp2040_hal::{
    i2c::{Async, Controller},
    pac, Clock,
};

use crate::{
    inter_board::{ADDRESS, COLUMN_COUNT},
    utils_time, Source,
};

use super::{Error, Pins, INTER_BOARD_FREQ};

use super::EventWrapper;

struct Timeout;
pub struct Main {
    i2c: rp2040_hal::i2c::I2C<pac::I2C0, Pins, Controller<Async>>,
    attached: bool,
}
#[cfg(feature = "debug")]
impl defmt::Format for Main {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Main {{…}}")
    }
}

impl Main {
    pub fn new(
        i2c_block: pac::I2C0,
        (sda, scl): Pins,
        resets: &mut pac::RESETS,
        system_clock: &rp2040_hal::clocks::SystemClock,
    ) -> Self {
        let mut i2c = rp2040_hal::i2c::I2C::new_async_controller(
            i2c_block,
            sda,
            scl,
            INTER_BOARD_FREQ,
            resets,
            system_clock.freq(),
        );
        i2c.set_on_pending(crate::utils_async::i2c0_on_pending);
        i2c.set_on_cancel(crate::utils_async::i2c0_on_cancel);
        unsafe {
            rp2040_hal::pac::NVIC::unpend(rp2040_hal::pac::Interrupt::I2C0_IRQ);
            rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::I2C0_IRQ);
        }
        Main {
            i2c,
            attached: false,
        }
    }

    pub async fn poll(
        &mut self,
        to_usb: &core::cell::RefCell<crate::ToUSBStack>,
        timer: &rp2040_hal::Timer,
        source: Source,
    ) -> Result<(), Error> {
        if !self.attached {
            let mut count = 0u32;
            loop {
                defmt::info!("new main: attempt {}", count);
                let res = futures::select_biased! {
                    res = self.i2c.write(ADDRESS, &[0x80, 0x01]).fuse() => Ok(res),
                    _ = utils_time::wait_for(timer, 1_000.micros()).fuse() => Err(Timeout)
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
                            10_000.micros()
                        } else {
                            5_000.micros()
                        };
                        super::utils_time::wait_for(timer, delay).await;
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
                    _ = utils_time::wait_for(timer, 5_000.micros()).fuse() => Err(Timeout)
                } {
                    Ok(res) => break res?,
                    Err(Timeout) if health > 0 => health -= 1,
                    Err(Timeout) => return Err(Error::Timeout),
                }
            }

            // scope the borrows
            let cnt = {
                let mut to_usb = to_usb.borrow_mut();
                let before = to_usb.len();
                // we don't really care about errors, that might just be because of events buffer full
                // We don't have recovery procedures anyway
                keypresses
                    .into_iter()
                    .take_while(|&b| b != 0)
                    .map(|evt| {
                        let evt = evt - 1;
                        let (pressed, key) = (evt & 0x80, evt & 0x7F);
                        // there's no concerns for speed here, u8 are already super optimised.
                        let (row, col) = (key / COLUMN_COUNT, key % COLUMN_COUNT);

                        use keyberon::layout::Event;
                        let conv = if pressed == 0 {
                            Event::Release(row, col)
                        } else {
                            Event::Press(row, col)
                        };
                        defmt::info!("Inter: {}", EventWrapper(conv));
                        conv
                    })
                    .try_for_each(|evt| to_usb.push_back((source, evt)))
                    .map_err(|_| Error::QueueFull)?;
                to_usb.len() - before
            };

            // TODO: actually read that from the UI
            let led = 0;
            // TODO: failure here suck as this means we may get the same event twice and we don't want
            // that.
            let mut frame = arrayvec::ArrayVec::<u8, 3>::new();
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
                    _ = utils_time::wait_for(timer, 7_000.micros()).fuse() => Err(Timeout)
                } {
                    Ok(res) => break res?,
                    Err(Timeout) if health > 0 => health -= 1,
                    Err(Timeout) => return Err(Error::Timeout),
                }
            }

            Ok(())
        }
    }

    pub fn release(self, resets: &mut pac::RESETS) -> (pac::I2C0, Pins) {
        let Main { i2c, .. } = self;
        i2c.free(resets)
    }
}
