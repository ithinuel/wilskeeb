#![doc = include_str!("../../README.md")]
#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

cfg_if::cfg_if! {
    if #[cfg(all(feature = "default", feature = "debug"))] {
            compile_error!("Default features must be disabled while building with debug support");
    } else if #[cfg(feature = "default")] {
        use panic_reset as _;
    } else if #[cfg(not(feature = "debug"))] {
        compile_error!("Either default or one of the debug features should be enabled.");
    }
}

#[cfg(feature = "debug")]
cfg_if::cfg_if! {
    if #[cfg(any(
            all(feature = "debug-to-probe", feature = "debug-to-cli"),
            not(any(feature = "debug-to-probe", feature = "debug-to-cli"))
    ))] {
        compile_error!(
            "Exacly only one feature of \"debug-to-probe\" or \"debug-to-cli\" must be enabled for this create when no-default-features is set"
        );
    } else if #[cfg(feature = "debug-to-probe")] {
        use panic_probe as _;
        use defmt_rtt as _;
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "nkro")] {
        use usbd_human_interface_device::device::keyboard::{
            NKROBootKeyboard as Keyboard, NKROBootKeyboardConfig as KeyboardConfig,
        };
    } else {
        use usbd_human_interface_device::device::keyboard::{
            BootKeyboard as Keyboard, BootKeyboardConfig as KeyboardConfig,
        };
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "cli")] {
        use usbd_serial::SerialPort;
        mod cli;
        type UsbSerialCell<'a> =
            RefCell<SerialPort<'a, UsbBus, [u8; USB_SERIAL_RX_SZ], [u8; USB_SERIAL_TX_SZ]>>;

        /// USB Serial cli update frequency.
        const CLI_FREQUENCY: HertzU32 = HertzU32::kHz(1);
        const USB_SERIAL_RX_SZ: usize = 64;
        const USB_SERIAL_TX_SZ: usize = 1024;
    }
}

use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

use critical_section::Mutex;
use fugit::{ExtU32, HertzU32, MicrosDurationU32, TimerInstantU64};

use rp2040_hal::{
    gpio::{bank0, FunctionNull, Pin, Pins, PullDown},
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    timer::Timer,
    usb::UsbBus,
    watchdog::Watchdog,
};

use frunk::hlist::{HCons, HNil};
use usb_device::{
    bus::UsbBusAllocator,
    class::UsbClass as _,
    device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbRev, UsbVidPid},
};
use usbd_human_interface_device::{device::DeviceClass, prelude::*, UsbHidError};

use arraydeque::{behavior::Saturating, ArrayDeque};
use arrayvec::ArrayVec;
use keyberon::{
    debounce::Debouncer,
    layout::{CustomEvent, Event, Layout},
};

mod inter_board;
mod layout;
mod matrix;
mod ui;
mod utils_async;

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

const SCANNED_STACK_SZ: usize = 16;
const TO_USB_STACK_SZ: usize = 32;

type ScannedEventStack = ArrayDeque<Event, SCANNED_STACK_SZ, Saturating>;
type ToUSBStack = ArrayDeque<(Source, Event), TO_USB_STACK_SZ, Saturating>;
type InterfaceList<'a> = HCons<Keyboard<'a, UsbBus>, HNil>;
type MyUsbHidClass<'a> = UsbHidClass<'a, UsbBus, InterfaceList<'a>>;
type TimerInstant = TimerInstantU64<1_000_000>;

#[derive(Default)]
struct BlackBoard {
    /// The use of that field is a bit ugly but `UsbDeviceState` is `repr(u8)` so it should be ok.
    usb_state: AtomicU8,
    is_main_half: AtomicBool,
    scanned: RefCell<ScannedEventStack>,
    to_usb: RefCell<ToUSBStack>,
}

/// Atomic BlackBoard
type ABlackBoard = Mutex<RefCell<ABBInner>>;

#[derive(Debug, Clone)]
#[cfg_attr(feature = "debug", derive(defmt::Format))]
struct ABBInner {
    usb_state: UsbDeviceState,
}
impl ABBInner {
    const fn new() -> Self {
        Self {
            usb_state: UsbDeviceState::Default,
        }
    }
}

/// USB VID/PID for a generic keyboard from <https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt>
const VID_PID: UsbVidPid = UsbVidPid(0x16c0, 0x27db);
const DEVICE_RELEASE: u16 = 0x0100;
/// Matrix scan frequency.
const SCAN_FREQUENCY: HertzU32 = HertzU32::kHz(5);
/// Time required for a key press to stop bouncing.
const DEBOUNCE_PERIOD: MicrosDurationU32 = MicrosDurationU32::millis(5);
/// Maximum async task count on the system
//const TASK_COUNT: usize = if cfg!(feature = "cli") { 6 } else { 5 };
const TASK_COUNT: usize = 32;

static_assertions::const_assert!(DEBOUNCE_PERIOD.ticks() <= (u16::max_value() as u32));
static_assertions::const_assert!(matches!(
    DEBOUNCE_PERIOD.const_partial_cmp(SCAN_FREQUENCY.into_duration::<1, 1_000_000>()),
    Some(core::cmp::Ordering::Greater | core::cmp::Ordering::Equal)
));

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
    sda: Pin<bank0::Gpio0, FunctionNull, PullDown>,
    scl: Pin<bank0::Gpio1, FunctionNull, PullDown>,
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

    let (mut sda, mut scl) = (sda.reconfigure(), scl.reconfigure());
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
    aboard: &ABlackBoard,
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
    let builder = || {
        let builder = UsbDeviceBuilder::new(usb_bus, VID_PID)
            .supports_remote_wakeup(true)
            .device_class(0)
            .device_sub_class(0)
            .device_protocol(0)
            .usb_rev(UsbRev::Usb200)
            .device_release(DEVICE_RELEASE)
            .strings(&[StringDescriptors::default()
                .manufacturer("Ithinuel.me")
                .product("WilsKeeb")
                .serial_number("Dev Environment")])
            .and_then(|builder| builder.max_packet_size_0(64))
            .and_then(|builder| builder.max_power(500));
        #[cfg(feature = "cli")]
        let builder = builder.map(|builder| builder.composite_with_iads());
        builder
    };
    let mut usb_dev = builder()
        .expect("Failed to configure UsbDeviceBuilder")
        .build();

    let mut layout = Layout::new(&layout::LAYERS);
    let mut timestamp = timer.get_counter_low();
    let mut previous_state = usb_dev.state();
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
        if usb_state != previous_state {
            previous_state = usb_state;
            defmt::info!("USBState: {:?}", defmt::Debug2Format(&usb_state));

            board.usb_state.store(usb_state as u8, Ordering::Relaxed);
            critical_section::with(|cs| {
                aboard.borrow_ref_mut(cs).usb_state = usb_state;
            });
        }

        let _ = keyboard_hid.device().read_report();

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
            let keycodes: ArrayVec<_, 70> = layout.keycodes().collect();
            let has_keycodes = !keycodes.is_empty();

            // Setup the report for the control channel
            match keyboard_hid.device().write_report(keycodes) {
                Err(UsbHidError::WouldBlock) | Err(UsbHidError::Duplicate) | Ok(_) => {}
                Err(e) => panic!("Failed to write keyboard report: {:?}", e),
            }
            match keyboard_hid.device().tick() {
                Err(UsbHidError::WouldBlock) | Ok(_) => {}
                Err(e) => panic!("Failed to process keyboard tick: {:?}", e),
            }

            // Wake the host.
            if !has_keycodes
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
#[cfg(feature = "cli")]
async fn cli_app<'a>(
    timer: &Timer,
    usb_serial: &UsbSerialCell<'a>,
    #[cfg(feature = "debug-to-cli")] mut consumer: defmt_bbq::DefmtConsumer,
) {
    const CLI_PERIOD: MicrosDurationU32 = CLI_FREQUENCY.into_duration();

    let mut next_scan_at = timer.get_counter() + CLI_PERIOD;
    loop {
        next_scan_at = utils_async::wait_until(timer, next_scan_at).await + CLI_PERIOD;

        cli::update(
            usb_serial,
            #[cfg(feature = "debug-to-cli")]
            &mut consumer,
        );
    }
}

#[sparkfun_pro_micro_rp2040::entry]
fn main() -> ! {
    static mut CORE1_STACK: Stack<40960> = Stack::new();
    static mut TIMER: Option<Timer> = None;
    static ASBB: ABlackBoard = Mutex::new(RefCell::new(ABBInner::new()));
    let aboard = &ASBB;

    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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

    use embedded_hal::digital::InputPin;
    let pin = pins.gpio29.into_pull_up_input(); // setup pull-up input
                                                // give a little bit of time for the pull-up to do its job.
    cortex_m::asm::delay(500);
    let is_right = pin.is_high().unwrap_or_else(|_| unreachable!()); // Cannot fail
    pin.into_pull_down_disabled(); // reset the pin to its default status.
    defmt::info!("I am the {} side", if is_right { "right" } else { "left" });
    IS_RIGHT.store(is_right, Ordering::Relaxed);

    let board = Default::default();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let alarm0 = timer.alarm_0().unwrap_or_else(|| unreachable!());
    *TIMER = Some(timer);
    let timer = TIMER.as_ref().unwrap();

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let keyboard_hid = UsbHidClassBuilder::new()
        .add_device(KeyboardConfig::default())
        .build(&usb_bus);

    // This needs to be the last class to be defined.
    #[cfg(feature = "cli")]
    let usb_serial = RefCell::new(SerialPort::new_with_store(
        &usb_bus,
        [0; USB_SERIAL_RX_SZ],
        [0; USB_SERIAL_TX_SZ],
    ));

    use rp2040_hal::pio::PIOExt;
    let (pio0, pio0sm0, pio0sm1, ..) = pac.PIO0.split(&mut pac.RESETS);
    let (pio1, pio1sm0, ..) = pac.PIO1.split(&mut pac.RESETS);

    let mut matrix = matrix::Matrix::new(
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
            pins.gpio28.reconfigure(),
            pins.gpio21.reconfigure(),
            pins.gpio23.reconfigure(),
            pins.gpio20.reconfigure(),
            pins.gpio22.reconfigure(),
        ),
    );

    if matrix.is_boot_loader_key_pressed() {
        rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
        unreachable!()
    }

    let neopixel = pins.gpio25;
    let led_strip = pins.gpio3;
    let (oled_sda, oled_scl) = (pins.gpio16, pins.gpio17);
    let (resets, i2c, sda, scl) = (pac.RESETS, pac.I2C0, pins.gpio0, pins.gpio1);

    use nostd_async::Task;
    utils_async::init(alarm0);

    let mut inter_board_task = Task::new(inter_board_app(
        &board,
        &clocks.system_clock,
        i2c,
        sda,
        scl,
        resets,
        timer,
    ));
    let mut scan_task = Task::new(scan_app(&board, timer, matrix));
    let mut usb_task = Task::new(usb_app(
        &board,
        &aboard,
        timer,
        &usb_bus,
        keyboard_hid,
        #[cfg(feature = "cli")]
        &usb_serial,
    ));
    #[cfg(feature = "cli")]
    let mut cli_task = Task::new(cli_app(
        timer,
        &usb_serial,
        #[cfg(feature = "debug-to-cli")]
        consumer,
    ));
    use rp2040_hal::Clock;
    let system_clock_freq = clocks.system_clock.freq();

    // delegate ui to second core
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];
    core1
        .spawn(&mut CORE1_STACK.mem, move || {
            defmt::trace!("Hi from the second core!");
            let mut runtime = nostd_async::Runtime::new();
            let mut ui_task = Task::new(ui::ui_app(
                aboard,
                timer,
                (pio0, pio0sm0, neopixel, pio0sm1, led_strip),
                (pio1, pio1sm0, oled_sda, oled_scl),
                system_clock_freq,
            ));

            let ui_hndl = ui_task.spawn(&mut runtime);
            ui_hndl.join();
            unreachable!("The UI task shall never end");
        })
        .expect("Unable to start second core's thread.");

    let runtime = nostd_async::Runtime::new();
    let _inter_board_hndl = inter_board_task.spawn(&runtime);
    let _scan_hndl = scan_task.spawn(&runtime);
    let usb_hndl = usb_task.spawn(&runtime);
    #[cfg(feature = "cli")]
    let _cli_hndl = cli_task.spawn(&runtime);

    unsafe {
        rp2040_hal::pac::NVIC::unpend(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
        rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
    }

    usb_hndl.join();

    unreachable!("The USB task shall never end");
}
