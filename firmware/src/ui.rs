//! # UI content
//!
//! - attachment status
//!   - Main configured − USB (up arrow)
//!   - Secondary configured − Side (arrow to the right/left)
//!   - not configured
//! - current layer ()

use adafruit_featherwing_oled128x64::{BufferedDisplay, DisplayState, ValidBus};
use fugit::{ExtU32, HertzU32, RateExtU32};
use sparkfun_pro_micro_rp2040::hal;
use usb_device::device::UsbDeviceState;

use embedded_graphics::{
    geometry::Dimensions,
    mono_font::{ascii::FONT_6X13_ITALIC, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::{Point, Size},
    primitives::{
        Primitive, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
        StrokeAlignment, StyledDrawable,
    },
    text::{Alignment, Text},
    Drawable,
};
use embedded_layout::prelude::*;

use hal::{
    gpio::{bank0, FunctionNull, Pin, PullDown},
    pac, pio, Timer,
};

#[cfg(not(feature = "debug"))]
use crate::defmt;
use crate::ABlackBoard;
use crate::{utils_async, ABBInner};

mod sh1107_wrapper;
mod widget;

pub const ADDRESS: embedded_hal::i2c::SevenBitAddress = 0x3C;

type Display<T> = BufferedDisplay<T, { ADDRESS }>;
enum UIStateMachine {
    Home,
    Idle,
    Suspended,
}
impl UIStateMachine {
    fn draw_rounded_frame<T: ValidBus>(display: &mut BufferedDisplay<T, { ADDRESS }>) {
        let display_area = display.bounding_box();
        let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

        // Frame
        let border_stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .stroke_alignment(StrokeAlignment::Inside)
            .build();
        let _ = RoundedRectangle::with_equal_corners(
            display_area.offset(1).offset(-2),
            Size::new(6, 6),
        )
        .into_styled(border_stroke)
        .draw(display);

        // Top horizontal bar
        let _ =
            Rectangle::new(Point::new(3, 19), Size::new(58, 1)).draw_styled(&thin_stroke, display);
    }

    fn get_text<'a>(
        txt: &'a str,
        display_area: Rectangle,
    ) -> Text<'a, MonoTextStyle<'static, BinaryColor>> {
        let text_style = MonoTextStyle::new(&FONT_6X13_ITALIC, BinaryColor::On);
        Text::with_alignment(txt, Point::zero(), text_style, Alignment::Center)
            // align text to the center of the display
            .align_to(&display_area, horizontal::Center, vertical::Center)
    }

    async fn startup<T: ValidBus>(
        timer: &Timer,
        display: &mut BufferedDisplay<T, { ADDRESS }>,
        aboard: &ABBInner,
    ) -> Self {
        Self::draw_rounded_frame(display);

        let display_area = display.bounding_box();
        let mut text = Self::get_text("Hey\nHandsome!", display_area);
        let _ = text.draw(display);

        let _ = display.flush().await;
        utils_async::wait_for(timer, 2000.millis()).await;
        text.character_style.text_color = Some(BinaryColor::Off);
        let _ = text.draw(display);

        let mut state = UIStateMachine::Idle;
        state.to_home(display, aboard).await;
        state
    }

    async fn to_home<T: ValidBus>(&mut self, _display: &mut Display<T>, _aboard: &ABBInner) {
        // TODO: Draw home

        // display:
        // - USB status
        // - Interboard status
        // - Scroll Lock
        // - Caps Lock

        *self = UIStateMachine::Home;
    }

    async fn to_idle<T: ValidBus>(&mut self, display: &mut Display<T>) {
        //assert!(matches!(self, UIStateMachine::Home));
        let _ = display.set_state(DisplayState::Off).await;
        *self = UIStateMachine::Idle;
    }

    async fn to_suspend<T: ValidBus>(&mut self, display: &mut Display<T>, timer: &Timer) {
        let display_area = display.bounding_box();
        let mut text = Self::get_text("Nat Nat!\n<3", display_area);
        let _ = text.draw(display);

        let _ = display.flush().await;
        let _ = display.set_state(DisplayState::On).await;
        utils_async::wait_for(timer, 1000.millis()).await;
        let _ = display.set_state(DisplayState::Off).await;

        text.character_style.text_color = Some(BinaryColor::Off);
        let _ = text.draw(display);
        *self = UIStateMachine::Suspended;
    }

    async fn wakeup<T: ValidBus>(
        &mut self,
        display: &mut Display<T>,
        timer: &Timer,
        aboard: &ABBInner,
    ) {
        Self::draw_rounded_frame(display);

        let display_area = display.bounding_box();
        let mut text = Self::get_text("Hi\nthere my\nlovely!", display_area);
        let _ = text.draw(display);

        let _ = display.flush().await;
        let _ = display.set_state(DisplayState::On).await;
        utils_async::wait_for(timer, 2000.millis()).await;
        let _ = display.set_state(DisplayState::Off).await;

        text.character_style.text_color = Some(BinaryColor::Off);
        let _ = text.draw(display);

        self.to_home(display, aboard).await
    }

    async fn update<T: ValidBus>(
        &mut self,
        timer: &Timer,
        display: &mut Display<T>,
        aboard: &ABBInner,
    ) {
        match self {
            UIStateMachine::Home => self.to_idle(display).await,
            UIStateMachine::Idle if aboard.usb_state == UsbDeviceState::Suspend => {
                self.to_suspend(display, timer).await
            }
            UIStateMachine::Suspended if aboard.usb_state != UsbDeviceState::Suspend => {
                self.wakeup(display, timer, aboard).await
            }
            _ => {}
        }
    }
}

/// User Interface task
///
/// Handles:
/// - On pro-micro board's neopixel
/// - LED1 neopixel strip
/// - OLED Display
///
/// Reads status from the black board & updates the UI accordingly.
#[allow(clippy::type_complexity)]
pub(crate) async fn ui_app(
    aboard: &ABlackBoard,
    timer: &Timer,
    _leds: (
        pio::PIO<pac::PIO0>,
        pio::UninitStateMachine<pio::PIO0SM0>,
        Pin<bank0::Gpio25, FunctionNull, PullDown>,
        pio::UninitStateMachine<pio::PIO0SM1>,
        Pin<bank0::Gpio3, FunctionNull, PullDown>,
    ),
    oled: (
        pio::PIO<pac::PIO1>,
        pio::UninitStateMachine<pio::PIO1SM0>,
        Pin<bank0::Gpio16, FunctionNull, PullDown>,
        Pin<bank0::Gpio17, FunctionNull, PullDown>,
    ),
    system_clock_freq: HertzU32,
) {
    let (mut pio1, pio1sm0, sda, scl) = oled;
    let mut oled_display = sh1107_wrapper::I2CPeriph(rp2040_async_i2c::pio::I2C::new(
        &mut pio1,
        sda,
        scl,
        pio1sm0,
        400_000.Hz(),
        system_clock_freq,
    ));
    oled_display.set_waker_setter(utils_async::pio1_waker_setter);

    critical_section::with(move |_| unsafe {
        pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_0);
    });

    let mut display: BufferedDisplay<_, { ADDRESS }>;

    utils_async::wait_for(timer, 500.millis()).await;
    loop {
        match BufferedDisplay::new(oled_display).await {
            Ok(d) => {
                display = d;
                break;
            }
            Err((oled, err)) => {
                defmt::error!("Display startup failed with: {}", defmt::Debug2Format(&err));
                utils_async::wait_for(timer, 10.millis()).await;
                oled_display = oled;
            }
        }
    }

    let _ = display.flip_horizontal(true).await;
    let _ = display.flip_vertical(true).await;
    let _ = display.set_contrast(0).await;
    let _ = display.set_state(DisplayState::On).await;
    defmt::info!("Display On!");

    let mut state = UIStateMachine::startup(
        timer,
        &mut display,
        &critical_section::with(|cs| aboard.borrow_ref(cs).clone()),
    )
    .await;

    //let carousel =
    //    embedded_graphics_widgets::carousel::Carousel::new(CarouselElements { slider: Slider });

    //let (mut pio0, pio0sm0, neopixel, pio1sm1, strip) = leds;
    //let mut neopixel = ws2812_pio::Ws2812::new(
    //    neopixel.into_mode(),
    //    &mut pio0,
    //    pio0sm0,
    //    system_clock_freq,
    //    timer.count_down(),
    //);
    //let _led_strip = ws2812_pio::Ws2812::new(
    //    strip.into_mode(),
    //    &mut pio0,
    //    pio1sm1,
    //    system_clock_freq,
    //    timer.count_down(),
    //);
    //let mut timestamp = timer.get_counter();
    //for wheel_pos in (0..=255).cycle() {
    //    timestamp = utils_async::wait_until(timer, timestamp + 40_000.micros()).await;
    //    neopixel
    //        .write(brightness(once(wheel(wheel_pos)), 25))
    //        .expect("Failed to set neopixel's color");
    //}
    let mut timestamp = timer.get_counter();
    loop {
        let aboard = critical_section::with(|cs| aboard.borrow_ref(cs).clone());
        state.update(timer, &mut display, &aboard).await;
        timestamp = utils_async::wait_until(timer, timestamp + 40.millis()).await;
    }
}
