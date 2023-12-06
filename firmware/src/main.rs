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

use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, Ordering},
};

use fugit::{HertzU32, MicrosDurationU32};
use rp2040_hal::{
    gpio::{bank0, FunctionNull, Pin, Pins, PullDown},
    pac,
    sio::Sio,
    timer::{Instant, Timer},
    usb::UsbBus,
    watchdog::Watchdog,
};

use usb::AtomicUsbDeviceState;
use usb_device::{bus::UsbBusAllocator, device::UsbDeviceState};
use usbd_human_interface_device::usb_class::UsbHidClassBuilder;

use arraydeque::{behavior::Saturating, ArrayDeque};
use keyberon::{debounce::Debouncer, layout::Event};

mod inter_board;
mod layout;
mod matrix;
mod utils_async;
mod utils_time;

#[cfg(feature = "cli")]
mod cli;
#[cfg(feature = "ui")]
mod ui;
mod usb;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Source {
    Left,
    Right,
}

const SCANNED_STACK_SZ: usize = 16;
const TO_USB_STACK_SZ: usize = 32;

type ScannedEventStack = ArrayDeque<Event, SCANNED_STACK_SZ, Saturating>;
type ToUSBStack = ArrayDeque<(Source, Event), TO_USB_STACK_SZ, Saturating>;

/// this is async safe provided:
/// - All the futures run on the same core
/// - no CAS operation is done on the AtomicBool and AtomicU8 (only plain load/store).
struct BlackBoard {
    /// The use of that field is a bit ugly but `UsbDeviceState` is `repr(u8)` so it should be ok.
    usb_state: usb::AtomicUsbDeviceState,
    is_main_half: AtomicBool,
}
impl BlackBoard {
    const fn new() -> Self {
        BlackBoard {
            usb_state: AtomicUsbDeviceState::new(UsbDeviceState::Default),
            is_main_half: AtomicBool::new(false),
        }
    }
}

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
async fn scan_app(scanned: &RefCell<ScannedEventStack>, timer: &Timer, mut matrix: matrix::Matrix) {
    const SCAN_PERIOD: MicrosDurationU32 = SCAN_FREQUENCY.into_duration();
    const DEBOUNCE_TICKS: u16 = (DEBOUNCE_PERIOD.ticks() / SCAN_PERIOD.ticks()) as u16;

    let mut debouncer = Debouncer::new(Default::default(), Default::default(), DEBOUNCE_TICKS);

    let mut now = timer.get_counter();
    loop {
        now = utils_time::wait_until(timer, now + SCAN_PERIOD).await;
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
    (scanned, to_usb): (&RefCell<ScannedEventStack>, &RefCell<ToUSBStack>),
    system_clock: &rp2040_hal::clocks::SystemClock,
    i2c_block: pac::I2C0,
    sda: Pin<bank0::Gpio0, FunctionNull, PullDown>,
    scl: Pin<bank0::Gpio1, FunctionNull, PullDown>,
    resets: pac::RESETS,
    timer: &Timer,
) {
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
        //board.usb_state
        let usb_state = board.usb_state.load();
        inter_board = inter_board.poll(usb_state, timer, scanned, to_usb).await;

        board
            .is_main_half
            .store(inter_board.is_main(), Ordering::Relaxed);
    }
}

#[sparkfun_pro_micro_rp2040::entry]
fn main() -> ! {
    #[cfg(feature = "ui")]
    static mut CORE1_STACK: rp2040_hal::multicore::Stack<40960> =
        rp2040_hal::multicore::Stack::new();
    cfg_if::cfg_if!(if #[cfg(feature = "ui")] {
        static BOARD: BlackBoard = BlackBoard::new();
        let board = &BOARD;
    } else {
        let board = BlackBoard::new();
        let board = &board;
    });

    let scanned = Default::default();
    let to_usb = Default::default();

    let mut pac = pac::Peripherals::take().unwrap_or_else(|| unreachable!());
    let _core = pac::CorePeripherals::take().unwrap_or_else(|| unreachable!());
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    #[allow(unused_mut)]
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
    .unwrap_or_else(|| unreachable!());

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    #[cfg(feature = "debug-to-cli")]
    let consumer = defmt_bbq::init().unwrap_or_else(|_| unreachable!());

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

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let alarm0 = timer.alarm_0().unwrap_or_else(|| unreachable!());

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let keyboard_hid = UsbHidClassBuilder::new()
        .add_device(usb::KeyboardConfig::default())
        .build(&usb_bus);

    // This needs to be the last class to be defined.
    #[cfg(feature = "cli")]
    let usb_serial = RefCell::new(usbd_serial::SerialPort::new_with_store(
        &usb_bus,
        [0; cli::USB_SERIAL_RX_SZ],
        [0; cli::USB_SERIAL_TX_SZ],
    ));

    #[allow(unused_mut)]
    let (mut resets, i2c, sda, scl) = (pac.RESETS, pac.I2C0, pins.gpio0, pins.gpio1);

    #[cfg(feature = "ui")]
    let (
        (pio0, pio0sm0, pio0sm1, ..),
        (pio1, pio1sm0, ..),
        (neopixel, led_strip),
        (oled_sda, oled_scl),
    ) = {
        use rp2040_hal::pio::PIOExt;
        (
            pac.PIO0.split(&mut resets),
            pac.PIO1.split(&mut resets),
            (pins.gpio25, pins.gpio3),
            (pins.gpio16, pins.gpio17),
        )
    };

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

    use nostd_async::Task;
    utils_time::init(alarm0, timer);

    let mut inter_board_task = Task::new(inter_board_app(
        &board,
        (&scanned, &to_usb),
        &clocks.system_clock,
        i2c,
        sda,
        scl,
        resets,
        &timer,
    ));
    let mut scan_task = Task::new(scan_app(&scanned, &timer, matrix));
    let mut usb_task = Task::new(usb::usb_app(
        &board,
        &to_usb,
        &timer,
        &usb_bus,
        keyboard_hid,
        #[cfg(feature = "cli")]
        &usb_serial,
    ));
    #[cfg(feature = "cli")]
    let mut cli_task = Task::new(cli::cli_app(
        &timer,
        &usb_serial,
        #[cfg(feature = "debug-to-cli")]
        consumer,
    ));

    // delegate ui to second core
    #[cfg(feature = "ui")]
    {
        use rp2040_hal::{multicore::Multicore, Clock};
        let system_clock_freq = clocks.system_clock.freq();

        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
        let core1 = &mut mc.cores()[1];
        let ref_usb_state = &board.usb_state;
        if let Err(_) = core1.spawn(&mut CORE1_STACK.mem, move || {
            defmt::trace!("Hi from the second core!");
            let mut runtime = nostd_async::Runtime::new();
            let mut ui_task = Task::new(ui::ui_app(
                &ref_usb_state,
                &timer,
                (pio0, pio0sm0, neopixel, pio0sm1, led_strip),
                (pio1, pio1sm0, oled_sda, oled_scl),
                system_clock_freq,
            ));

            let ui_hndl = ui_task.spawn(&mut runtime);
            ui_hndl.join();
            unreachable!("The UI task shall never end");
        }) {
            panic!("Unable to start second core's thread.");
        }
    }

    let runtime = nostd_async::Runtime::new();
    let _inter_board_hndl = inter_board_task.spawn(&runtime);
    let _scan_hndl = scan_task.spawn(&runtime);
    let usb_hndl = usb_task.spawn(&runtime);
    #[cfg(feature = "cli")]
    let _cli_hndl = cli_task.spawn(&runtime);

    usb_hndl.join();

    unreachable!("The USB task shall never end");
}
