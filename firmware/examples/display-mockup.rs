//! # Example: Hello world
//!
//! A simple hello world example displaying some primitive shapes and some text underneath.

use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::{raw::BigEndian, BinaryColor},
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
        StrokeAlignment, StyledDrawable, Triangle,
    },
    text::{Alignment, Text},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
};

mod gui {
    struct Carousel {
        a: (),
    }
}

const IMAGE: [u8; 1024] = {
    let mut img = [0; 1024];
    let source = include_bytes!("./assets/source.bmp");
    let header_len = 62;

    let mut line = 0;
    while line < 128 {
        let mut col = 0;
        let max_col = 64 / 8;
        while col < max_col {
            img[line * max_col + col] = source[header_len + (127 - line) * max_col + col];
            col += 1;
        }
        line += 1;
    }
    img
};

const IMAGE2: [u8; 1024] = {
    let mut img = [0; 1024];
    let source = include_bytes!("./assets/source2.bmp");
    let header_len = 62;

    /* bmp are drawn from the bottom left. Lets flip it vertically */
    let mut line = 0;
    while line < 128 {
        let mut col = 0;
        let max_col = 64 / 8;
        while col < max_col {
            img[line * max_col + col] = source[header_len + (127 - line) * max_col + col];
            col += 1;
        }
        line += 1;
    }
    img
};

fn main() -> Result<(), std::convert::Infallible> {
    // Create a new simulator display with 128x64 pixels.
    let mut display: SimulatorDisplay<BinaryColor> = SimulatorDisplay::new(Size::new(64, 128));

    // Create styles used by the drawing operations.
    let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    //let thick_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    //let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    //let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    //let yoffset = 14;

    let sprites = ImageRaw::<_, BigEndian>::new(&IMAGE2, 64);
    let sprite_map = [
        ((0, 0), (16, 16)),
        ((16, 0), (16, 16)),
        ((32, 0), (16, 16)),
        ((0, 16), (16, 16)),
        ((16, 16), (16, 16)),
        ((32, 16), (16, 16)),
        ((3, 32), (58, 32)),
        ((3, 64), (58, 32)),
        ((3, 96), (58, 32)),
    ];
    let sprites = sprite_map
        .into_iter()
        .map(|(point, size)| sprites.sub_image(&Rectangle::new(point.into(), size.into())))
        .collect::<Vec<_>>();

    let link_pt = Point::new(3, 3);
    let mode_pt = Point::new(3, 56);

    let usb = &sprites[0];
    let usb_i2cl = &sprites[1];
    let not_attached = &sprites[3];
    let mode_abc = &sprites[6];
    let mode_123 = &sprites[8];

    // Draw a 3px wide outline around the display.
    RoundedRectangle::with_equal_corners(
        display.bounding_box().offset(1).offset(-2),
        Size::new(6, 6),
    )
    .into_styled(border_stroke)
    .draw(&mut display)?;

    Rectangle::new(Point::new(3, 19), Size::new(58, 1)).draw_styled(&thin_stroke, &mut display)?;

    Image::new(usb_i2cl, link_pt).draw(&mut display)?;
    Image::new(mode_abc, mode_pt).draw(&mut display)?;

    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .scale(1)
        //.pixel_spacing(0)
        //.theme(BinaryColorTheme::OledBlue)
        //.scale(2)
        //.pixel_spacing(1)
        .build();
    let mut window = Window::new("Demo", &output_settings);
    window.update(&display);

    let mut cnt = 0;
    loop {
        let evt = window.events().next();
        if let Some(evt) = evt {
            match evt {
                SimulatorEvent::KeyDown { .. } => {
                    match cnt {
                        0 => {
                            Image::new(usb, link_pt).draw(&mut display)?;
                            Image::new(mode_123, mode_pt).draw(&mut display)?
                        }
                        1 => {
                            Image::new(usb_i2cl, link_pt).draw(&mut display)?;
                            Image::new(mode_abc, mode_pt).draw(&mut display)?
                        }
                        _ => (),
                    }
                    window.update(&display);
                    cnt += 1;
                    cnt %= 2;
                }
                SimulatorEvent::Quit => break,
                _ => (),
            }
        }
    }

    Ok(())
}
