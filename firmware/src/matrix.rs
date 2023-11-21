use embedded_hal::digital::{OutputPin, PinState};
use rp2040_hal::gpio::{bank0, FunctionSioInput, FunctionSioOutput, Pin, PullNone, PullUp};

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
    Pin<bank0::Gpio2, FunctionSioInput, PullUp>,
    Pin<bank0::Gpio4, FunctionSioInput, PullUp>,
    Pin<bank0::Gpio5, FunctionSioInput, PullUp>,
    Pin<bank0::Gpio6, FunctionSioInput, PullUp>,
    Pin<bank0::Gpio7, FunctionSioInput, PullUp>,
    Pin<bank0::Gpio8, FunctionSioInput, PullUp>,
    Pin<bank0::Gpio9, FunctionSioInput, PullUp>,
);
type Rows = (
    Pin<bank0::Gpio28, FunctionSioOutput, PullNone>,
    Pin<bank0::Gpio21, FunctionSioOutput, PullNone>,
    Pin<bank0::Gpio23, FunctionSioOutput, PullNone>,
    Pin<bank0::Gpio20, FunctionSioOutput, PullNone>,
    Pin<bank0::Gpio22, FunctionSioOutput, PullNone>,
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
    fn get_row(&mut self, idx: usize) -> u32 {
        self.set(idx, PinState::Low);

        // experimentally measured to generate a ~1200ns delay between 2 iterations.
        cortex_m::asm::delay(40);

        let column = rp2040_hal::sio::Sio::read_bank0() & ROW_MASK;

        self.set(idx, PinState::High);

        column
    }
    pub fn clear(&mut self) {
        for idx in 0..5 {
            self.set(idx, PinState::High);
        }
    }
    pub fn get(&mut self) -> MatrixState {
        let mut keys = MatrixState::default();

        // SAFETY: We're only doing atomic read and only interested in the pins we own
        for (idx, key) in keys.0.iter_mut().enumerate() {
            key.0 = self.get_row(idx);
        }
        keys
    }
    pub fn is_boot_loader_key_pressed(&mut self) -> bool {
        // matrix: 0, 0 => gpio2
        const MASK: u32 = 1 << 2;
        (self.get_row(0) & MASK) != MASK
    }
}
