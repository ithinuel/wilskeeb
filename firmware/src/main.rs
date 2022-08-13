#![doc = include_str!("../../README.md")]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(all(feature = "debug-to-probe", feature = "debug-to-cli"))]
compile_error!(
    "Only one feature of \"debug-to-probe\" or \"debug-to-cli\" must be enabled for this create"
);

use core::{
    cell::RefCell,
    convert::TryInto,
    iter::once,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

#[cfg(not(any(feature = "debug-to-probe", feature = "debug-to-cli")))]
use panic_reset as _;

#[cfg(feature = "debug-to-probe")]
use panic_probe as _;

#[cfg(feature = "debug-to-probe")]
use defmt_rtt as _;

use embedded_time::{
    duration::{Extensions as _, Microseconds},
    rate::{Extensions as _, Hertz},
};
use rp2040_hal::{
    clocks::Clock,
    gpio::{bank0, Pin, PinId, Pins, PullDownDisabled},
    pac,
    pio::{self, PIOExt},
    sio::Sio,
    timer::Timer,
    usb::UsbBus,
    watchdog::Watchdog,
};

use usb_device::{
    bus::UsbBusAllocator,
    class::UsbClass as _,
    device::{UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
};
use usbd_human_interface_device::{
    device::keyboard::NKROBootKeyboardInterface,
    hid_class::{UsbHidClass, UsbHidClassBuilder},
    page::Keyboard,
    UsbHidError,
};
#[cfg(feature = "cli")]
use usbd_serial::SerialPort;

use arraydeque::ArrayDeque;
use arrayvec::ArrayVec;
use keyberon::{
    debounce::Debouncer,
    layout::{CustomEvent, Event, Layout},
};
use num_enum::FromPrimitive;
use smart_leds::{brightness, SmartLedsWrite, RGB8};

#[cfg(not(feature = "debug"))]
mod defmt {
    #[macro_export]
    macro_rules! info {
        ($($_:tt)*) => {{}};
    }
    #[macro_export]
    macro_rules! timestamp {
        ($($_:tt)*) => {};
    }

    // macros are exported at the root of the crate so pull them back here
    pub use super::{info, timestamp};
}

#[cfg(feature = "cli")]
mod cli;
mod inter_board;
mod layout;
mod matrix;
//mod ui;
mod utils_async;
mod utils_time;

use inter_board::{Error, Main, Secondary};
use layout::CustomAction;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "debug", derive(defmt::Format))]
pub enum Source {
    Left,
    Right,
}
impl core::ops::Not for Source {
    type Output = Source;

    fn not(self) -> Self::Output {
        match self {
            Source::Left => Source::Right,
            Source::Right => Source::Left,
        }
    }
}

type ScannedEventStack<const SIZE: usize> =
    arraydeque::ArrayDeque<[Event; SIZE], arraydeque::behavior::Saturating>;
type ToUSBStack<const SIZE: usize> =
    arraydeque::ArrayDeque<[(Source, Event); SIZE], arraydeque::behavior::Saturating>;
type MyUsbHidClass<'a> = UsbHidClass<
    UsbBus,
    frunk::hlist::HCons<
        NKROBootKeyboardInterface<'a, rp2040_hal::usb::UsbBus, utils_time::MyClock<'a>>,
        frunk::hlist::HNil,
    >,
>;
#[cfg(feature = "cli")]
type UsbSerialCell<'a> = RefCell<SerialPort<'a, UsbBus, [u8; 64], [u8; USB_SERIAL_TX_SZ]>>;
struct BlackBoard {
    side: Source,
    /// The use of that field is a bit ugly but `UsbDeviceState` is `repr(u8)` so it should be ok.
    usb_state: AtomicU8,
    is_main_half: AtomicBool,
    scanned: RefCell<ScannedEventStack<16>>,
    to_usb: RefCell<ToUSBStack<32>>,
}

/// USB VID/PID for a generic keyboard from <https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt>
const VID_PID: UsbVidPid = UsbVidPid(0x16c0, 0x27db);
const DEVICE_RELEASE: u16 = 0x0100;
/// USB Serial cli update frequency.
#[cfg(feature = "cli")]
const CLI_FREQUENCY: Hertz<u64> = Hertz(1_000);
/// Matrix scan frequency.
const SCAN_FREQUENCY: Hertz<u64> = Hertz(5_000);
/// Key debounce period.
const DEBOUNCE_PERIOD: Microseconds<u64> = Microseconds(1_000);

#[cfg(feature = "cli")]
const USB_SERIAL_TX_SZ: usize = 1024;

/// RP2040's second stage bootloader.
///
/// This bootloader configures the QSPI and the XIP controllers.
#[used]
#[link_section = ".boot2"]
static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

defmt::timestamp!("{=u32:us}", {
    // NOTE(interrupt-safe) single instruction volatile read operation
    unsafe { pac::Peripherals::steal().TIMER.timerawl.read().bits() }
});

/// Detects which keyboard half we are running.
fn read_side<P: PinId>(pin: Pin<P, PullDownDisabled>) -> Source {
    use embedded_hal::digital::blocking::InputPin;
    let pin = pin.into_pull_up_input(); // setup pull-up input
    let is_right = pin.is_high().unwrap_or_else(|_| unreachable!()); // Cannot fail
    pin.into_pull_down_disabled(); // reset the pin to its default status.

    if is_right {
        Source::Right
    } else {
        Source::Left
    }
}

/// Key board scan task
///
/// Handles:
/// - Scans the matrix
/// - Debounces matrix' state
/// - Stacks the events on the blackboard
async fn scan_app(board: &BlackBoard, timer: &Timer, mut matrix: matrix::Matrix) {
    const SCAN_PERIOD: Microseconds<u64> = Microseconds(1_000_000 / SCAN_FREQUENCY.0);

    let BlackBoard { scanned, .. } = board;

    let mut debouncer = Debouncer::new(
        Default::default(),
        Default::default(),
        (DEBOUNCE_PERIOD / SCAN_PERIOD.0)
            .0
            .try_into()
            .expect("Debounce period must fit a u16"),
    );

    let mut now = Microseconds(timer.get_counter());
    loop {
        now = utils_async::wait_until(timer, now + SCAN_PERIOD).await;
        let mut scanned = scanned.borrow_mut();
        for event in debouncer.events(matrix.get()) {
            // TODO: if for some reason the queue isn't processed.
            // shall we drop the oldest or ignore the most recent?

            defmt::info!(" Scan: {}", inter_board::EventWrapper(event));
            let _ = scanned.push_back(event);
        }
        drop(scanned);
    }
}

/// Inter-Keyboard half communication task
///
/// Handles:
/// - negociation of main/secondary keyboard half
/// - when acting as main:
///   - reads events from secondary and stacks them on the blackboard;
///   - sends ui updates;
/// - when acting as secondary: serves requests comming from main.
async fn inter_board_app(
    board: &BlackBoard,
    system_clock_freq: Hertz,
    i2c_block: pac::I2C0,
    sda: Pin<bank0::Gpio0, PullDownDisabled>,
    scl: Pin<bank0::Gpio1, PullDownDisabled>,
    mut resets: pac::RESETS,
    timer: &Timer,
) {
    use either::Either;

    let BlackBoard {
        side,
        usb_state,
        scanned,
        to_usb,
        is_main_half,
        ..
    } = board;
    let side = *side;
    // Secondary register's map:
    //
    //
    // Read:
    // read on 0x00-0x7F do not change PTR.
    // read on 0x80-0xFF do increment PTR. 0xFF rolls over to 0x80.
    //
    // Write:
    // The first by written is always "PTR"
    //
    // 0x00: Event FIFO
    // 0x80: 7-1 reserved
    //       0   configured
    // 0x81: 7-0 led status
    let (mut sda, mut scl) = (sda.into_mode(), scl.into_mode());
    scl.set_slew_rate(rp2040_hal::gpio::OutputSlewRate::Slow);
    sda.set_slew_rate(rp2040_hal::gpio::OutputSlewRate::Slow);
    // Fix odd capacitor-like signal behavior
    scl.set_drive_strength(rp2040_hal::gpio::OutputDriveStrength::TwoMilliAmps);
    sda.set_drive_strength(rp2040_hal::gpio::OutputDriveStrength::TwoMilliAmps);
    let mut state: Either<Secondary, Main> = Either::Left(Secondary::new(
        i2c_block,
        (sda, scl),
        &mut resets,
        timer.get_counter_low(),
    ));

    let configured = [
        (UsbDeviceState::Configured as u8),
        (UsbDeviceState::Suspend as u8),
    ];

    // wait for usb configured state or an i2c sync packet
    loop {
        let usb_state = usb_state.load(Ordering::Relaxed);
        let timestamp = timer.get_counter_low();
        state = match state {
            Either::Right(mut main) => {
                if !configured.contains(&usb_state) {
                    let (i2c_block, pins) = main.release(&mut resets);
                    is_main_half.store(false, Ordering::Relaxed);
                    defmt::info!("Inter: USB nolonger configured, switching back to secondary");
                    Either::Left(Secondary::new(i2c_block, pins, &mut resets, timestamp))
                } else {
                    let (state, delay) = match main.poll(to_usb, timer, !side).await {
                        Ok(_) => (Either::Right(main), 1_000.microseconds()),
                        Err(_e) => {
                            defmt::info!("Inter: Main::poll error: {}", _e);
                            let (i2c_block, pins) = main.release(&mut resets);
                            is_main_half.store(false, Ordering::Relaxed);
                            (
                                Either::Left(Secondary::new(
                                    i2c_block,
                                    pins,
                                    &mut resets,
                                    timestamp,
                                )),
                                500.microseconds(),
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
            Either::Left(mut secondary) => {
                let timestamp = timer.get_counter_low();
                match secondary.serve(scanned, timestamp) {
                    Err(Error::BusIdle) if usb_state == (UsbDeviceState::Configured as u8) => {
                        defmt::info!("Inter: USB Configured");

                        let (i2c_block, pins) = secondary.release(&mut resets);
                        is_main_half.store(true, Ordering::Relaxed);
                        Either::Right(Main::new(i2c_block, pins, &mut resets, system_clock_freq))
                    }
                    // If the bus becomes idle (either during a transaction or outside of a
                    // transaction) then reset the bus to clear any unexpected clock stretching.
                    Err(Error::BusIdle) | Err(Error::Timeout) => {
                        let (i2c_block, pins) = secondary.release(&mut resets);
                        utils_async::wait_for(timer, 1_000.microseconds()).await;
                        Either::Left(Secondary::new(
                            i2c_block,
                            pins,
                            &mut resets,
                            timer.get_counter_low(),
                        ))
                    }
                    _ => {
                        // lets be nice
                        utils_async::_yield().await;
                        Either::Left(secondary)
                    }
                }
            }
        }
    }
}

/// USB polling task
///
/// Handles:
/// - Processing the usb stack and associated protocols.
/// - Draining the keyboard event stack from the blackboard and feeds it into the layout
/// - Ticks the layout & generates HID reports
async fn usb_app<'a>(
    board: &BlackBoard,
    timer: &Timer,
    usb_bus: &UsbBusAllocator<UsbBus>,
    mut keyboard_hid: MyUsbHidClass<'_>,
    #[cfg(feature = "cli")] usb_serial: &UsbSerialCell<'a>,
) {
    let BlackBoard {
        to_usb,
        is_main_half,
        ..
    } = board;
    let builder = UsbDeviceBuilder::new(usb_bus, VID_PID)
        .manufacturer("Ithinuel.me")
        .product("WilsKeeb")
        .serial_number(env!("CARGO_PKG_VERSION"))
        .supports_remote_wakeup(true)
        .device_class(0)
        .device_sub_class(0)
        .device_protocol(0)
        .max_packet_size_0(64)
        .max_power(500)
        .device_release(DEVICE_RELEASE);
    #[cfg(feature = "cli")]
    let builder = builder.composite_with_iads();
    let mut usb_dev = builder.build();

    let mut layout = Layout::new(&layout::LAYERS);
    let mut timestamp = timer.get_counter_low();
    let mut state = usb_dev.state();
    loop {
        utils_async::_yield().await;
        let new_state = usb_dev.state();
        if state != new_state {
            defmt::info!(
                "{} {}",
                defmt::Debug2Format(&state),
                defmt::Debug2Format(&new_state)
            );
            state = new_state;
        }

        #[cfg(feature = "cli")]
        if let Ok(mut usb_serial) = usb_serial.try_borrow_mut() {
            if usb_dev.poll(&mut [&mut keyboard_hid, &mut *usb_serial]) {
                keyboard_hid.poll();
                usb_serial.poll();
            }
        }
        #[cfg(not(feature = "cli"))]
        if usb_dev.poll(&mut [&mut keyboard_hid]) {
            keyboard_hid.poll();
        }

        let usb_state = usb_dev.state();
        board.usb_state.store(usb_state as u8, Ordering::Relaxed);

        // if 1ms since last update
        let now = timer.get_counter_low();

        if is_main_half.load(Ordering::Relaxed) && now.wrapping_sub(timestamp) >= 1_000 {
            timestamp = now;

            // Process events from keypress
            for (source, mut event) in to_usb.borrow_mut().drain(..) {
                if source == Source::Right {
                    event = event.transform(|row, col| (row, 13 - col));
                }
                layout.event(event);
            }
            if let CustomEvent::Press(CustomAction::Bootldr) = layout.tick() {
                rp2040_hal::rom_data::reset_to_usb_boot(0, 0)
            }

            // Collect key presses.
            let keycodes: ArrayVec<_, 70> = layout
                .keycodes()
                .map(|k| Keyboard::from_primitive(k as u8))
                .collect();

            // Setup the report for the control channel
            match keyboard_hid.interface().write_report(&keycodes) {
                Err(UsbHidError::WouldBlock) | Err(UsbHidError::Duplicate) | Ok(_) => {}
                Err(e) => panic!("Failed to write keyboard report: {:?}", e),
            }
            match keyboard_hid.interface().tick() {
                Err(UsbHidError::WouldBlock) | Ok(_) => {}
                Err(e) => panic!("Failed to process keyboard tick: {:?}", e),
            }

            // Wake the host.
            if !keycodes.is_empty()
                && usb_state == UsbDeviceState::Suspend
                && usb_dev.remote_wakeup_enabled()
            {
                usb_dev.bus().remote_wakeup();
            }
        }
    }
}

/// CLI task
///
/// Handles simple request from the usb-serial interface to help manage and debug the firmware.
/// see [`cli::update`] for a full description of the supported commands.
#[cfg(feature = "debug-to-cli")]
async fn cli_app<'a>(
    timer: &Timer,
    usb_serial: &UsbSerialCell<'a>,
    source: Source,
    mut consumer: defmt_bbq::DefmtConsumer,
) {
    const CLI_PERIOD: Microseconds<u64> = Microseconds(1_000_000 / CLI_FREQUENCY.0);

    let mut next_scan_at = Microseconds(timer.get_counter()) + CLI_PERIOD;
    loop {
        next_scan_at = wait_until(timer, next_scan_at).await + CLI_PERIOD;

        cli::update(usb_serial, source, &mut consumer);
    }
}
#[cfg(all(feature = "cli", not(feature = "debug-to-cli")))]
async fn cli_app<'a>(timer: &Timer, usb_serial: &UsbSerialCell<'a>, source: Source) {
    const CLI_PERIOD: Microseconds<u64> = Microseconds(1_000_000 / CLI_FREQUENCY.0);

    let mut next_scan_at = Microseconds(timer.get_counter()) + CLI_PERIOD;
    loop {
        next_scan_at = wait_until(timer, next_scan_at).await + CLI_PERIOD;

        cli::update(usb_serial, source);
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
async fn ui_app(
    _board: &BlackBoard,
    timer: &Timer,
    leds: (
        pio::PIO<pac::PIO0>,
        pio::UninitStateMachine<pio::PIO0SM0>,
        Pin<bank0::Gpio25, PullDownDisabled>,
        pio::UninitStateMachine<pio::PIO0SM1>,
        Pin<bank0::Gpio3, PullDownDisabled>,
    ),
    oled: (
        pio::PIO<pac::PIO1>,
        pio::UninitStateMachine<pio::PIO1SM0>,
        Pin<bank0::Gpio16, PullDownDisabled>,
        Pin<bank0::Gpio17, PullDownDisabled>,
    ),
    system_clock_freq: Hertz,
) {
    let (mut pio0, pio0sm0, sda, scl) = oled;
    let _oled_display = i2c_pio::I2C::new(
        &mut pio0,
        sda.into_pull_up_disabled(),
        scl.into_pull_up_disabled(),
        pio0sm0,
        400_000.Hz(),
        system_clock_freq,
    );
    let (mut pio1, pio1sm0, neopixel, pio1sm1, strip) = leds;
    let mut neopixel = ws2812_pio::Ws2812::new(
        neopixel.into_mode(),
        &mut pio1,
        pio1sm0,
        system_clock_freq,
        timer.count_down(),
    );
    let _led_strip = ws2812_pio::Ws2812::new(
        strip.into_mode(),
        &mut pio1,
        pio1sm1,
        system_clock_freq,
        timer.count_down(),
    );
    let mut timestamp = Microseconds(timer.get_counter());
    for wheel_pos in (0..=255).cycle() {
        let now = utils_async::wait_until(timer, timestamp + 40_000.microseconds()).await;

        neopixel
            .write(brightness(once(wheel(wheel_pos)), 25))
            .expect("Failed to set neopixel's color");

        timestamp = now;
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    #[allow(unused_unsafe)]
    unsafe {
        // prime the spinlock used to sync both cores by atomic-polyfill
        (*pac::SIO::ptr()).spinlock[31].write_with_zero(|w| w.bits(1));
    }

    #[cfg(feature = "debug-to-cli")]
    let consumer = defmt_bbq::init().unwrap();

    // GPIO0-1: SDA/SCL: inter-chip com (i2c0)
    // GPIO2: col0
    // GPIO3: RGB data                  (pio0-sm1)
    // GPIO4-9: col 1-6
    // GPIO16-17: SDA/SCL: oled display (pio1)
    // GPIO28, 21, 23, 20, 22: row0-4
    // GPIO25: WS2812 onboard addressable rgb (pio0-sm0)
    // GPIO29: right_nleft: high = right half

    let side = read_side(pins.gpio29);

    let runtime = nostd_async::Runtime::new();

    let board = BlackBoard {
        side,
        usb_state: AtomicU8::new(UsbDeviceState::Default as u8),
        is_main_half: AtomicBool::new(false),
        scanned: RefCell::new(ArrayDeque::new()),
        to_usb: RefCell::new(ArrayDeque::new()),
    };

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let theclock = utils_time::MyClock(&timer);
    let keyboard_hid = UsbHidClassBuilder::new()
        .add_interface(NKROBootKeyboardInterface::default_config(&theclock))
        .build(&usb_bus);

    // This needs to be the last class to be defined.
    #[cfg(feature = "cli")]
    let usb_serial = RefCell::new(SerialPort::new_with_store(
        &usb_bus,
        [0; 64],
        [0; USB_SERIAL_TX_SZ],
    ));

    let (pio0, pio0sm0, pio0sm1, ..) = pac.PIO0.split(&mut pac.RESETS);
    let (pio1, pio1sm0, ..) = pac.PIO1.split(&mut pac.RESETS);

    let matrix = matrix::Matrix::new(
        (
            pins.gpio2.into_pull_up_input(),
            pins.gpio4.into_pull_up_input(),
            pins.gpio5.into_pull_up_input(),
            pins.gpio6.into_pull_up_input(),
            pins.gpio7.into_pull_up_input(),
            pins.gpio8.into_pull_up_input(),
            pins.gpio9.into_pull_up_input(),
        ),
        (
            pins.gpio28.into_readable_output(),
            pins.gpio21.into_readable_output(),
            pins.gpio23.into_readable_output(),
            pins.gpio20.into_readable_output(),
            pins.gpio22.into_readable_output(),
        ),
    );

    let system_clock_freq = clocks.system_clock.freq();
    let neopixel = pins.gpio25;
    let led_strip = pins.gpio3;
    let (oled_sda, oled_scl) = (pins.gpio16, pins.gpio17);
    let (resets, i2c, sda, scl) = (pac.RESETS, pac.I2C0, pins.gpio0, pins.gpio1);

    use nostd_async::Task;
    let mut inter_board_task = Task::new(inter_board_app(
        &board,
        system_clock_freq,
        i2c,
        sda,
        scl,
        resets,
        &timer,
    ));
    let mut scan_task = Task::new(scan_app(&board, &timer, matrix));
    let mut usb_task = Task::new(usb_app(
        &board,
        &timer,
        &usb_bus,
        keyboard_hid,
        #[cfg(feature = "cli")]
        &usb_serial,
    ));
    #[cfg(feature = "cli")]
    let mut cli_task = Task::new(cli_app(
        &timer,
        &usb_serial,
        side,
        #[cfg(feature = "debug-to-cli")]
        consumer,
    ));
    let mut ui_task = Task::new(ui_app(
        &board,
        &timer,
        (pio0, pio0sm0, neopixel, pio0sm1, led_strip),
        (pio1, pio1sm0, oled_sda, oled_scl),
        system_clock_freq,
    ));

    let _inter_board_hndl = inter_board_task.spawn(&runtime);
    let _scan_hndl = scan_task.spawn(&runtime);
    let usb_hndl = usb_task.spawn(&runtime);
    #[cfg(feature = "cli")]
    let _cli_hndl = cli_task.spawn(&runtime);
    let _ui_hndl = ui_task.spawn(&runtime);
    usb_hndl.join();

    unreachable!("The USB task shall never end");
}
