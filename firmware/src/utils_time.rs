use core::{
    cell::RefCell,
    task::{Context, Poll, Waker},
};

use arraydeque::ArrayDeque;
use critical_section::Mutex;
use fugit::MicrosDurationU32;
use itertools::Itertools;
use rp2040_hal::{
    pac::interrupt,
    sio::CoreId,
    timer::{Alarm, Alarm0, Timer},
};

use crate::Instant;
use crate::TASK_COUNT;

type WakerVec = ArrayDeque<(Instant, Waker, CoreId), { TASK_COUNT }>;

/// any us duration lower than that will be "async busy looped";
const CUTOFF_DELAY: MicrosDurationU32 = MicrosDurationU32::micros(500);

static TIMER_WAKER: Mutex<RefCell<Option<(Alarm0, WakerVec)>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<rp2040_hal::Timer>>> = Mutex::new(RefCell::new(None));
pub fn init(alarm: Alarm0, timer: Timer) {
    critical_section::with(|cs| {
        *TIMER_WAKER.borrow_ref_mut(cs) = Some((alarm, ArrayDeque::new()));
        *TIMER.borrow_ref_mut(cs) = Some(timer);
    });
    unsafe {
        rp2040_hal::pac::NVIC::unpend(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
        rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::TIMER_IRQ_0);
    }
}

fn setup_alarm(alarm: &mut Alarm0, timestamp: Instant) {
    alarm.clear_interrupt();
    alarm.enable_interrupt();
    if let Err(_) = alarm.schedule_at(timestamp) {
        panic!("Failed to schedule next alarm");
    }
}
fn enqueue_waker(cx: &Context, timestamp: Instant) {
    let core = rp2040_hal::sio::Sio::core();

    critical_section::with(|cs| {
        let mut tuple = TIMER_WAKER.borrow_ref_mut(cs);
        let Some((alarm, waker_vec)) = tuple.as_mut() else {
            unreachable!()
        };

        // figure where we should sit in the queue.
        let idx = waker_vec
            .iter()
            .find_position(|&&(d, _, _)| d >= timestamp)
            .map(|(pos, _)| pos)
            .unwrap_or_else(|| waker_vec.len());

        // Add us in there.
        if let Err(_) = waker_vec.insert(idx, (timestamp, cx.waker().clone(), core)) {
            panic!("Unable to add waker to the wait list");
        }

        // if we're first in the queue, setup the alarm
        if idx == 0 {
            setup_alarm(alarm, timestamp);
        }
    });
}
async fn shcedule_for(timestamp: Instant) {
    let mut queued = false;

    futures::future::poll_fn(move |cx| {
        // if we already queued this delay, that means we just woke up.
        if queued {
            Poll::Ready(())
        } else {
            // otherwise:
            enqueue_waker(cx, timestamp);
            queued = true;
            Poll::Pending
        }
    })
    .await
}
async fn poll_fn_until(timer: &Timer, timestamp: Instant) {
    futures::future::poll_fn(|cx| {
        cx.waker().wake_by_ref();
        if timer.get_counter() < timestamp {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    })
    .await
}

pub async fn wait_for(timer: &Timer, delay: MicrosDurationU32) {
    let timestamp = timer.get_counter() + delay;
    wait_until(timer, timestamp).await;
}

pub async fn wait_until(timer: &Timer, timestamp: Instant) -> Instant {
    let now = timer.get_counter();
    if timestamp >= (now + CUTOFF_DELAY) {
        shcedule_for(timestamp).await;
    }
    poll_fn_until(timer, timestamp).await;

    // just in case we woke a tad too early. Should not exceed CUTOFF_DELAY.
    timer.get_counter()
}

#[interrupt]
#[allow(non_snake_case)]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        let mut tuple = TIMER_WAKER.borrow_ref_mut(cs);
        let mut timer = TIMER.borrow_ref_mut(cs);
        let (Some(timer), Some((alarm, waker_vec))) = (timer.as_mut(), tuple.as_mut()) else {
            unreachable!()
        };

        let now = timer.get_counter();

        let mut wake_core1 = false;
        let count = waker_vec
            .iter()
            .take_while(|&&(timestamp, _, _)| timestamp < (now + CUTOFF_DELAY))
            .inspect(|(_timestamp, waker, core)| {
                waker.wake_by_ref();
                wake_core1 |= matches!(core, CoreId::Core1);
            })
            .count();
        waker_vec.drain(0..count);

        if wake_core1 {
            cortex_m::asm::sev();
        }

        if let Some(&(timestamp, _, _)) = waker_vec.front() {
            setup_alarm(alarm, timestamp);
        } else {
            alarm.disable_interrupt();
        }
    });
}
