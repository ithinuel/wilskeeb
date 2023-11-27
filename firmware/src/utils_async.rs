use core::{cell::RefCell, task::Waker};
use critical_section::Mutex;
use rp2040_hal::sio::CoreId;
use rp2040_hal::pac::interrupt;

macro_rules! waker_handler {
    ($name:ident, $setter_name:ident, $core:path, $irq_name:ident, $clear_irq:tt) => {
        static $name: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
        pub fn $setter_name(waker: Waker) {
            assert_eq!(rp2040_hal::sio::Sio::core(), $core);
            assert!(cortex_m::register::primask::read().is_inactive());
            critical_section::with(|cs| {
                *$name.borrow_ref_mut(cs) = Some(waker);
            });
        }
        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $irq_name() {
            assert_eq!(rp2040_hal::sio::Sio::core(), $core);
            $clear_irq
            critical_section::with(|cs| $name.borrow_ref_mut(cs).take().map(Waker::wake));
        }
    };
}

// Safety: Only accessed from core 0 and nostd_async runs tasks from within an interrupt::free.
waker_handler!(I2C0_IRQ, i2c0_waker_setter, CoreId::Core0, I2C0_IRQ, {
    let i2c0 = unsafe { &rp2040_hal::pac::Peripherals::steal().I2C0 };
    i2c0.ic_intr_mask.modify(|_, w| {
        w.m_tx_abrt()
            .enabled()
            .m_tx_empty()
            .enabled()
            .m_rx_full()
            .enabled()
    });
});

// Safety: Only accessed from core 1 and nostd_async runs tasks from within an interrupt::free (on
// the other core :o).
waker_handler!(PIO1_WAKER, pio1_waker_setter, CoreId::Core1, PIO1_IRQ_0, {
    let pio = &rp2040_hal::pac::Peripherals::steal().PIO1;
    pio.sm_irq[0].irq_inte.modify(|_, w| {
        w.sm0()
            .clear_bit()
            .sm0_txnfull()
            .clear_bit()
            .sm0_rxnempty()
            .clear_bit()
    });
});
