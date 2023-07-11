use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::Drawable;

pub const _GLYPHS: [u8; 1024] = {
    let bmp = include_bytes!("../../../vendor/sh1107-rs/assets/glyphs.bmp");

    // Eliminate bmp header
    // Transpose & flip image

    let mut g = [0u8; 1024];
    let mut page = 0;
    while page < 16 {
        let mut col = 0;
        while col < 64 {
            g[page * 64 + col] = bmp[130 + (63 - col) * 16 + (15 - page)];
            col += 1;
        }
        page += 1;
    }
    g
};

pub trait Widget<E> {
    /// Return true when focus shall return to parent
    fn update(&mut self, event: E) -> bool;
}

struct Slider;
impl<E> Widget<E> for Slider {
    fn update(&mut self, _event: E) -> bool {
        todo!()
    }
}
impl Drawable for Slider {
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, _target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        todo!()
    }
}

pub enum CarouselEvent {
    Next,
    Select,
    Return,
}
struct Carousel {
    entered: bool,
    selected: usize,
    slider: Slider,
}
impl Drawable for Carousel {
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, _target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        todo!()
    }
}
impl<E: Into<Option<CarouselEvent>>> Widget<E> for Carousel {
    fn update(&mut self, event: E) -> bool {
        if self.entered {
            self.entered = match self.selected {
                0 => self.slider.update(event),
                _ => unreachable!(),
            }
        } else {
            match event.into() {
                None => {}
                Some(CarouselEvent::Next) => self.selected = (self.selected + 1) % 1,
                Some(CarouselEvent::Select) => self.entered = true,
                Some(CarouselEvent::Return) => return true,
            }
        }
        false
    }
}
