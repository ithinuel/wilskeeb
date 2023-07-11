use arraydeque::ArrayDeque;
use core::{
    cell::RefCell,
    task::{Poll, Waker},
};
use critical_section::Mutex;
use fugit::MicrosDurationU32;
use rp2040_hal::{
    pac::interrupt,
    sio::CoreId,
    timer::{Alarm, Alarm0, Timer},
};

use crate::TimerInstant;

type WakerVec = ArrayDeque<(TimerInstant, Waker, CoreId), { super::TASK_COUNT }>;

/// any us duration lower than that will be "async busy looped";
const CUTOFF_DELAY: MicrosDurationU32 = MicrosDurationU32::micros(500);

static TIMER_WAKER: Mutex<RefCell<Option<(Alarm0, WakerVec)>>> = Mutex::new(RefCell::new(None));
pub fn init(alarm: Alarm0) {
    critical_section::with(|cs| {
        *TIMER_WAKER.borrow_ref_mut(cs) = Some((alarm, ArrayDeque::new()));
    });
}

pub async fn wait_for(timer: &Timer, delay: MicrosDurationU32) {
    let timestamp = timer.get_counter() + delay;
    wait_until(timer, timestamp).await;
}

pub async fn wait_until(timer: &Timer, timestamp: TimerInstant) -> TimerInstant {
    let now = timer.get_counter();
    if timestamp >= (now + CUTOFF_DELAY) {
        let mut started = false;

        futures::future::poll_fn(move |cx| {
            if !started {
                critical_section::with(|cs| {
                    if let Some((alarm, waker_vec)) = TIMER_WAKER.borrow_ref_mut(cs).as_mut() {
                        let core = rp2040_hal::sio::Sio::core();
                        // walk vec to insert at the right location
                        let idx = waker_vec
                            .iter()
                            .enumerate()
                            .find_map(|(idx, &(d, _, _))| (d >= timestamp).then_some(idx))
                            .unwrap_or_else(|| waker_vec.len());

                        waker_vec
                            .insert(idx, (timestamp, cx.waker().clone(), core))
                            .expect("Unable to add waker to the wait list");

                        if idx == 0 {
                            alarm.clear_interrupt();
                            alarm.enable_interrupt();
                            alarm.schedule_at(timestamp).unwrap();
                        }
                    } else {
                        unreachable!()
                    }
                });

                started = true;
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }

    futures::future::poll_fn(|cx| {
        cx.waker().wake_by_ref();
        if timer.get_counter() < timestamp {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    })
    .await;
    timer.get_counter()
}

#[interrupt]
#[allow(non_snake_case)]
fn TIMER_IRQ_0() {
    fn get_stollen_counter() -> TimerInstant {
        let timer = unsafe { &rp2040_hal::pac::Peripherals::steal().TIMER };
        let mut hi0 = timer.timerawh.read().bits();
        let ts = loop {
            let low = timer.timerawl.read().bits();
            let hi1 = timer.timerawh.read().bits();
            if hi0 == hi1 {
                break (u64::from(hi0) << 32) | u64::from(low);
            }
            hi0 = hi1;
        };
        TimerInstant::from_ticks(ts)
    }

    critical_section::with(|cs| {
        if let Some((alarm, waker_vec)) = TIMER_WAKER.borrow_ref_mut(cs).as_mut() {
            alarm.clear_interrupt();

            let now = get_stollen_counter();

            let mut next_stop = None;
            let mut wake_core1 = false;
            let (_, waker, core) = waker_vec.pop_front().unwrap_or_else(|| unreachable!());
            waker.wake();
            wake_core1 |= matches!(core, CoreId::Core1);
            while let Some((timestamp, next_waker, core)) = waker_vec.pop_front() {
                if timestamp >= (now + CUTOFF_DELAY) {
                    next_stop = Some(timestamp);

                    waker_vec
                        .push_front((timestamp, next_waker, core))
                        .expect("Failed to add waker back to the wait list");
                    break;
                }

                next_waker.wake();
                wake_core1 |= matches!(core, CoreId::Core1);
            }
            if wake_core1 {
                cortex_m::asm::sev();
            }
            if let Some(timestamp) = next_stop {
                alarm.enable_interrupt();
                alarm.schedule_at(timestamp).unwrap();
            } else {
                alarm.disable_interrupt();
            }
        }
    });
}

static mut I2C0_WAKER: Option<Waker> = None;
pub fn i2c0_waker_setter(waker: Waker) {
    assert_eq!(rp2040_hal::sio::Sio::core(), CoreId::Core0);
    assert!(cortex_m::register::primask::read().is_inactive());
    unsafe {
        I2C0_WAKER = Some(waker);
    }
}

#[interrupt]
#[allow(non_snake_case)]
unsafe fn I2C0_IRQ() {
    assert_eq!(rp2040_hal::sio::Sio::core(), CoreId::Core0);
    let i2c0 = unsafe { &rp2040_hal::pac::Peripherals::steal().I2C0 };
    i2c0.ic_intr_mask.modify(|_, w| {
        w.m_tx_abrt()
            .enabled()
            .m_tx_empty()
            .enabled()
            .m_rx_full()
            .enabled()
    });
    cortex_m::interrupt::free(|_| unsafe { I2C0_WAKER.take().map(Waker::wake) });
}

// Safety: Only accessed from core 1 and nostd_async runs tasks from within an interrupt::free.
static mut PIO1_WAKER: Option<Waker> = None;
pub fn pio1_waker_setter(waker: Waker) {
    assert_eq!(rp2040_hal::sio::Sio::core(), CoreId::Core1);
    assert!(cortex_m::register::primask::read().is_inactive());
    unsafe {
        PIO1_WAKER = Some(waker);
    }
}

#[interrupt]
#[allow(non_snake_case)]
unsafe fn PIO1_IRQ_0() {
    assert_eq!(rp2040_hal::sio::Sio::core(), CoreId::Core1);
    let pio = &rp2040_hal::pac::Peripherals::steal().PIO1;
    pio.sm_irq[0].irq_inte.modify(|_, w| {
        w.sm0()
            .clear_bit()
            .sm0_txnfull()
            .clear_bit()
            .sm0_rxnempty()
            .clear_bit()
    });
    cortex_m::interrupt::free(|_| PIO1_WAKER.take().map(Waker::wake));
}
