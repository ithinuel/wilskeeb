use embedded_hal::digital::{blocking::OutputPin, PinState};
use rp2040_hal::gpio::{bank0, Output, Pin, PullUpInput, Readable};

const ROW_MASK: u32 = 0b11_1111_0100;
#[derive(Debug, PartialEq, Eq, Default, Clone, Copy)]
pub struct Row(u32);

#[derive(Debug, PartialEq, Eq, Clone)]
pub struct MatrixState([Row; 5]);
impl Default for MatrixState {
    fn default() -> Self {
        Self([Row(ROW_MASK); 5])
    }
}

impl<'a> IntoIterator for &'a MatrixState {
    type IntoIter = impl Iterator<Item = &'a Row>;
    type Item = &'a Row;
    fn into_iter(self) -> Self::IntoIter {
        self.0.iter()
    }
}
impl<'a> IntoIterator for &'a Row {
    type IntoIter = impl Iterator<Item = &'a bool>;
    type Item = &'a bool;
    fn into_iter(self) -> Self::IntoIter {
        core::iter::once(2).chain(4..10).map(move |idx| {
            let mask = 1 << idx;
            if (self.0 & mask) != mask {
                &true
            } else {
                &false
            }
        })
    }
}
type Cols = (
    Pin<bank0::Gpio2, PullUpInput>,
    Pin<bank0::Gpio4, PullUpInput>,
    Pin<bank0::Gpio5, PullUpInput>,
    Pin<bank0::Gpio6, PullUpInput>,
    Pin<bank0::Gpio7, PullUpInput>,
    Pin<bank0::Gpio8, PullUpInput>,
    Pin<bank0::Gpio9, PullUpInput>,
);
type Rows = (
    Pin<bank0::Gpio28, Output<Readable>>,
    Pin<bank0::Gpio21, Output<Readable>>,
    Pin<bank0::Gpio23, Output<Readable>>,
    Pin<bank0::Gpio20, Output<Readable>>,
    Pin<bank0::Gpio22, Output<Readable>>,
);
pub struct Matrix {
    _cols: Cols,
    rows: Rows,
}
impl Matrix {
    pub fn new(cols: Cols, rows: Rows) -> Self {
        let mut res = Self { _cols: cols, rows };
        res.clear();
        res
    }
    fn set(&mut self, idx: usize, state: PinState) {
        let _ = match idx {
            0 => self.rows.0.set_state(state),
            1 => self.rows.1.set_state(state),
            2 => self.rows.2.set_state(state),
            3 => self.rows.3.set_state(state),
            4 => self.rows.4.set_state(state),
            _ => unreachable!(),
        };
    }
    pub fn clear(&mut self) {
        for idx in 0..5 {
            self.set(idx, PinState::High);
        }
    }
    pub fn get(&mut self) -> MatrixState {
        let mut keys = MatrixState::default();

        // SAFETY: We're only doing atomic read and only interested in the pins we own
        let sio = unsafe { &*rp2040_hal::pac::SIO::ptr() };
        for (idx, key) in keys.0.iter_mut().enumerate() {
            self.set(idx, PinState::Low);

            // experimentally measured to generate a ~1200ns delay between 2 iterations.
            cortex_m::asm::delay(40);

            key.0 = sio.gpio_in.read().bits() & ROW_MASK;

            self.set(idx, PinState::High);
        }
        keys
    }
}
