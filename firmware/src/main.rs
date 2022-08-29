#![doc = include_str!("../../README.md")]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(any(
    all(feature = "debug-to-probe", feature = "debug-to-cli"),
    all(
        feature = "debug",
        not(any(feature = "debug-to-probe", feature = "debug-to-cli"))
    )
))]
compile_error!(
    "Only one feature of \"debug-to-probe\" or \"debug-to-cli\" must be enabled for this create"
);

use core::{
    cell::RefCell,
    iter::once,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

#[cfg(feature = "debug-to-probe")]
use panic_probe as _;
#[cfg(not(any(feature = "debug-to-probe", feature = "debug-to-cli")))]
use panic_reset as _;

#[cfg(feature = "debug-to-probe")]
use defmt_rtt as _;

use fugit::{ExtU32, HertzU32, MicrosDurationU32, RateExtU32, TimerInstantU64};

use rp2040_hal::{
    gpio::{bank0, Pin, Pins, PullDownDisabled},
    pac,
    pio::{self, PIOExt},
    sio::Sio,
    timer::Timer,
    usb::UsbBus,
    watchdog::Watchdog,
    Clock,
};

use frunk::hlist::{HCons, HNil};
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

use arraydeque::{behavior::Saturating, ArrayDeque};
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
    macro_rules! trace {
        ($($_:tt)*) => {{}};
    }
    #[macro_export]
    macro_rules! debug {
        ($($_:tt)*) => {{}};
    }
    #[macro_export]
    macro_rules! info {
        ($($_:tt)*) => {{}};
    }
    #[macro_export]
    macro_rules! error {
        ($($_:tt)*) => {{}};
    }
    #[macro_export]
    macro_rules! timestamp {
        ($($_:tt)*) => {};
    }

    // macros are exported at the root of the crate so pull them back here
    pub use super::{debug, error, info, timestamp, trace};
}

#[cfg(feature = "cli")]
mod cli;
mod inter_board;
mod layout;
mod matrix;
//mod ui;
mod utils_async;
mod utils_time;

use layout::CustomAction;
use utils_time::MyClock;

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

const SCANNED_STACK_SZ: usize = 16;
const TO_USB_STACK_SZ: usize = 32;

type ScannedEventStack = ArrayDeque<[Event; SCANNED_STACK_SZ], Saturating>;
type ToUSBStack = ArrayDeque<[(Source, Event); TO_USB_STACK_SZ], Saturating>;
type InterfaceList<'a> = HCons<NKROBootKeyboardInterface<'a, UsbBus, MyClock<'a>>, HNil>;
type MyUsbHidClass<'a> = UsbHidClass<UsbBus, InterfaceList<'a>>;
#[cfg(feature = "cli")]
type UsbSerialCell<'a> =
    RefCell<SerialPort<'a, UsbBus, [u8; USB_SERIAL_RX_SZ], [u8; USB_SERIAL_TX_SZ]>>;
type TimerInstant = TimerInstantU64<1_000_000>;

struct BlackBoard {
    /// The use of that field is a bit ugly but `UsbDeviceState` is `repr(u8)` so it should be ok.
    usb_state: AtomicU8,
    is_main_half: AtomicBool,
    scanned: RefCell<ScannedEventStack>,
    to_usb: RefCell<ToUSBStack>,
}

/// USB VID/PID for a generic keyboard from <https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt>
const VID_PID: UsbVidPid = UsbVidPid(0x16c0, 0x27db);
const DEVICE_RELEASE: u16 = 0x0100;
/// USB Serial cli update frequency.
#[cfg(feature = "cli")]
const CLI_FREQUENCY: HertzU32 = HertzU32::kHz(1);
/// Matrix scan frequency.
const SCAN_FREQUENCY: HertzU32 = HertzU32::kHz(5);
/// Time required for a key press to stop bouncing.
const DEBOUNCE_PERIOD: MicrosDurationU32 = MicrosDurationU32::millis(5);
/// Maximum async task count on the system
#[cfg(feature = "cli")]
const TASK_COUNT: usize = 5;
#[cfg(not(feature = "cli"))]
const TASK_COUNT: usize = 6;

static_assertions::const_assert!(DEBOUNCE_PERIOD.ticks() <= (u16::max_value() as u32));
static_assertions::const_assert!(matches!(
    DEBOUNCE_PERIOD.const_partial_cmp(SCAN_FREQUENCY.into_duration::<1, 1_000_000>()),
    Some(core::cmp::Ordering::Greater | core::cmp::Ordering::Equal)
));

#[cfg(feature = "cli")]
const USB_SERIAL_RX_SZ: usize = 64;
#[cfg(feature = "cli")]
const USB_SERIAL_TX_SZ: usize = 1024;

static IS_RIGHT: AtomicBool = AtomicBool::new(false);

defmt::timestamp!("{=u32:us}", {
    // NOTE(interrupt-safe) single instruction volatile read operation
    unsafe { pac::Peripherals::steal().TIMER.timerawl.read().bits() }
});

/// Detects which keyboard half we are running.
fn read_side() -> Source {
    if IS_RIGHT.load(Ordering::Relaxed) {
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
    const SCAN_PERIOD: MicrosDurationU32 = SCAN_FREQUENCY.into_duration();
    const DEBOUNCE_TICKS: u16 = (DEBOUNCE_PERIOD.ticks() / SCAN_PERIOD.ticks()) as u16;

    let BlackBoard { scanned, .. } = board;

    let mut debouncer = Debouncer::new(Default::default(), Default::default(), DEBOUNCE_TICKS);

    let mut now = timer.get_counter();
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
    system_clock: &rp2040_hal::clocks::SystemClock,
    i2c_block: pac::I2C0,
    sda: Pin<bank0::Gpio0, PullDownDisabled>,
    scl: Pin<bank0::Gpio1, PullDownDisabled>,
    resets: pac::RESETS,
    timer: &Timer,
) {
    let BlackBoard {
        usb_state,
        scanned,
        to_usb,
        is_main_half,
        ..
    } = board;

    let (mut sda, mut scl) = (sda.into_mode(), scl.into_mode());
    scl.set_slew_rate(rp2040_hal::gpio::OutputSlewRate::Slow);
    sda.set_slew_rate(rp2040_hal::gpio::OutputSlewRate::Slow);
    // Fix odd capacitor-like signal behavior
    scl.set_drive_strength(rp2040_hal::gpio::OutputDriveStrength::TwoMilliAmps);
    sda.set_drive_strength(rp2040_hal::gpio::OutputDriveStrength::TwoMilliAmps);

    let mut inter_board = inter_board::InterBoard::new(
        system_clock,
        i2c_block,
        (sda, scl),
        resets,
        timer.get_counter(),
    );

    // wait for usb configured state or an i2c sync packet
    loop {
        let usb_state = unsafe { core::mem::transmute(usb_state.load(Ordering::Relaxed)) };
        inter_board = inter_board.poll(usb_state, timer, scanned, to_usb).await;

        is_main_half.store(inter_board.is_main(), Ordering::Relaxed);
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
        .serial_number("Dev Environment")
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
    loop {
        utils_async::wait_for(timer, 10.micros()).await;

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

        let _ = keyboard_hid.interface().read_report();

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
    mut consumer: defmt_bbq::DefmtConsumer,
) {
    const CLI_PERIOD: MicrosDurationU32 = CLI_FREQUENCY.into_duration();

    let mut next_scan_at = timer.get_counter() + CLI_PERIOD;
    loop {
        next_scan_at = utils_async::wait_until(timer, next_scan_at).await + CLI_PERIOD;

        cli::update(usb_serial, &mut consumer);
    }
}
#[cfg(all(feature = "cli", not(feature = "debug-to-cli")))]
async fn cli_app<'a>(timer: &Timer, usb_serial: &UsbSerialCell<'a>) {
    const CLI_PERIOD: MicrosDurationU32 = CLI_FREQUENCY.into_duration();

    let mut next_scan_at = timer.get_counter() + CLI_PERIOD;
    loop {
        next_scan_at = utils_async::wait_until(timer, next_scan_at).await + CLI_PERIOD;

        cli::update(usb_serial);
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
    system_clock: &rp2040_hal::clocks::SystemClock,
) {
    let (mut pio0, pio0sm0, sda, scl) = oled;
    let _oled_display = i2c_pio::I2C::new(
        &mut pio0,
        sda.into_pull_up_disabled(),
        scl.into_pull_up_disabled(),
        pio0sm0,
        400_000.Hz(),
        system_clock.freq(),
    );
    let (mut pio1, pio1sm0, neopixel, pio1sm1, strip) = leds;
    let mut neopixel = ws2812_pio::Ws2812::new(
        neopixel.into_mode(),
        &mut pio1,
        pio1sm0,
        system_clock.freq(),
        timer.count_down(),
    );
    let _led_strip = ws2812_pio::Ws2812::new(
        strip.into_mode(),
        &mut pio1,
        pio1sm1,
        system_clock.freq(),
        timer.count_down(),
    );
    let mut timestamp = timer.get_counter();
    for wheel_pos in (0..=255).cycle() {
        defmt::info!("ui task heart beat");
        timestamp = utils_async::wait_until(timer, timestamp + 40_000.micros()).await;

        neopixel
            .write(brightness(once(wheel(wheel_pos)), 25))
            .expect("Failed to set neopixel's color");
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

#[sparkfun_pro_micro_rp2040::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    //pac.CLOCKS.sleep_en0.modify(|_, w| {
    //    w.clk_sys_pio0()
    //        .set_bit()
    //        .clk_sys_pio1()
    //        .set_bit()
    //        .clk_sys_jtag()
    //        .set_bit()
    //        .clk_sys_pll_usb()
    //        .set_bit()
    //        .clk_sys_i2c0()
    //        .set_bit()
    //});
    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        sparkfun_pro_micro_rp2040::XOSC_CRYSTAL_FREQ,
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

    use embedded_hal::digital::blocking::InputPin;
    let pin = pins.gpio29.into_pull_up_input(); // setup pull-up input
    let is_right = pin.is_high().unwrap_or_else(|_| unreachable!()); // Cannot fail
    pin.into_pull_down_disabled(); // reset the pin to its default status.
    IS_RIGHT.store(is_right, Ordering::Relaxed);

    let runtime = nostd_async::Runtime::new();

    let board = BlackBoard {
        usb_state: AtomicU8::new(UsbDeviceState::Default as u8),
        is_main_half: AtomicBool::new(false),
        scanned: RefCell::new(ArrayDeque::new()),
        to_usb: RefCell::new(ArrayDeque::new()),
    };

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let alarm0 = timer.alarm_0().unwrap_or_else(|| unreachable!());
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let theclock = MyClock(&timer);
    let keyboard_hid = UsbHidClassBuilder::new()
        .add_interface(NKROBootKeyboardInterface::default_config(&theclock))
        .build(&usb_bus);

    // This needs to be the last class to be defined.
    #[cfg(feature = "cli")]
    let usb_serial = RefCell::new(SerialPort::new_with_store(
        &usb_bus,
        [0; USB_SERIAL_RX_SZ],
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

    let system_clock_freq = &clocks.system_clock;
    let neopixel = pins.gpio25;
    let led_strip = pins.gpio3;
    let (oled_sda, oled_scl) = (pins.gpio16, pins.gpio17);
    let (resets, i2c, sda, scl) = (pac.RESETS, pac.I2C0, pins.gpio0, pins.gpio1);

    use nostd_async::Task;
    utils_async::init(alarm0);

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

    unsafe {
        rp2040_hal::pac::NVIC::unpend(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
        rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
    }

    usb_hndl.join();

    unreachable!("The USB task shall never end");
}
