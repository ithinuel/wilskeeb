//! # UI content
//!
//! - attachment status
//!   - Main configured − USB (up arrow)
//!   - Secondary configured − Side (arrow to the right/left)
//!   - not configured
//! - current layer ()

use core::cell::RefCell;

use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};

struct FrameBuffer([u8; 1024]);

impl OriginDimensions for FrameBuffer {
    fn size(&self) -> Size {
        Size::new(64, 128)
    }
}
impl DrawTarget for FrameBuffer {
    type Color = BinaryColor;

    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for p in pixels {
            let id = (p.0.x * 128 + p.0.y) as usize;
            let (idx, bit) = (id / 8, id % 8);
            let mask = 1 << bit;

            match p.1 {
                BinaryColor::Off => self.0[idx] &= !mask,
                BinaryColor::On => self.0[idx] |= mask,
            }
        }
        Ok(())
    }
}

pub struct StateInner {}
pub struct State(RefCell<StateInner>);

impl State {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self(RefCell::new(StateInner {}))
    }
}
impl keyberon::keyboard::Leds for &State {
    fn caps_lock(&mut self, _status: bool) {}

    fn num_lock(&mut self, _status: bool) {}

    fn scroll_lock(&mut self, _status: bool) {}

    fn compose(&mut self, _status: bool) {}

    fn kana(&mut self, _status: bool) {}
}
