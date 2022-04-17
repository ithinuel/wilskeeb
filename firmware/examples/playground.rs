use core::marker::PhantomData;
use std::cell::RefCell;
trait PixelColor: Copy + PartialEq<Self> {}
trait DrawTarget {
    type Color: PixelColor;
    type Error;
}
trait Drawable {
    type Color: PixelColor;
    type Output;
    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, <D as DrawTarget>::Error>
    where
        D: DrawTarget<Color = Self::Color>;
}

#[derive(Copy, PartialEq, Clone)]
struct BinaryColor;
impl PixelColor for BinaryColor {}

trait Widget: Drawable {
    type Event;
    /// Returns true if the widget keeps the focus.
    fn update(&mut self, event: Self::Event) -> bool;
}

enum CarouselEvent {
    Next,
    Select,
}
struct Carousel<T: Widget, const SZ: usize> {
    el: [T; SZ],
    selected: usize,
    entered: bool,
}
impl<T: Widget, const SZ: usize> Drawable for Carousel<T, SZ> {
    type Color = BinaryColor;
    type Output = ();
    fn draw<D>(&self, _target: &mut D) -> Result<Self::Output, <D as DrawTarget>::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        todo!()
    }
}
impl<EV: Into<CarouselEvent>, T: Widget<Event = EV>, const SZ: usize> Widget for Carousel<T, SZ> {
    type Event = EV;
    fn update(&mut self, event: Self::Event) -> bool {
        if self.entered {
            self.entered = self.el[self.selected].update(event);
        } else {
            match event.into() {
                CarouselEvent::Next => {
                    self.selected = (self.selected + 1) % (SZ + 1);
                }
                CarouselEvent::Select => {
                    if self.selected == SZ {
                        self.selected = 0;
                        return false;
                    } else {
                        self.entered = !self.entered;
                    }
                }
            }
        }
        true
    }
}
struct Slider<F, T> {
    f: F,
    _t: PhantomData<T>,
    v: u32,
}
impl<F, T> Drawable for Slider<F, T> {
    type Color = BinaryColor;
    type Output = ();
    fn draw<Dt>(&self, _target: &mut Dt) -> Result<Self::Output, <Dt as DrawTarget>::Error>
    where
        Dt: DrawTarget<Color = Self::Color>,
    {
        todo!()
    }
}
impl<F, T> Widget for Slider<F, T>
where
    F: FnMut(u32, T) -> u32,
{
    type Event = T;
    fn update(&mut self, event: Self::Event) -> bool {
        let new = (self.f)(self.v, event).clamp(0, 100);
        let update = new == self.v;
        if update {
            self.v = new;
        }
        update
    }
}

fn main() {
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    enum UIEvent {
        Next,
        Select,
    }
    impl Into<CarouselEvent> for UIEvent {
        fn into(self) -> CarouselEvent {
            match self {
                UIEvent::Next => CarouselEvent::Next,
                UIEvent::Select => CarouselEvent::Select
            }
        }
    }

    let cell = RefCell::new(0);
    let mut ui = Carousel {
        el: [Slider {
            f: |data: u32, event: UIEvent| -> u32 {
                if event == UIEvent::Select {
                    *cell.borrow_mut() = data;
                    data
                } else if data == 100 {
                    0
                } else {
                    data.saturating_add(10)
                }
            },
            _t: PhantomData,
            v: 50,
        }],
        selected: 0,
        entered: false,
    };
    ui.update(UIEvent::Select); // enter slider
    ui.update(UIEvent::Next); // grow slider
    ui.update(UIEvent::Select); // exit slider
    ui.update(UIEvent::Next); // move to "return"
    ui.update(UIEvent::Select); // do return (nop)

    ui.update(UIEvent::Select); // enter slider
    ui.update(UIEvent::Next); // grow slider
    ui.update(UIEvent::Select); // exit slider
}

