use rp2040_hal::timer::Timer;

use embedded_time::duration::Microseconds;

pub async fn _yield() {
    let mut once = false;
    futures::future::poll_fn(move |cx| {
        cx.waker().wake_by_ref();
        if !once {
            once = true;
            core::task::Poll::Pending
        } else {
            core::task::Poll::Ready(())
        }
    })
    .await
}

pub async fn wait_for<T: Into<Microseconds<u32>>>(timer: &Timer, delay: T) {
    let delay = delay.into();
    let start = timer.get_counter_low();

    // prevent task queue starvation.
    _yield().await;
    while timer.get_counter_low().wrapping_sub(start) <= delay.0 {
        _yield().await
    }
}

pub async fn wait_until(timer: &Timer, end: Microseconds<u64>) -> Microseconds<u64> {
    // prevent task queue starvation.
    _yield().await;
    loop {
        let now = timer.get_counter();
        if now >= end.0 {
            return Microseconds(now);
        }
        _yield().await
    }
}

#[allow(dead_code)]
pub struct Mutex<T>(core::cell::RefCell<T>);
impl<T> Mutex<T> {
    #[allow(dead_code)]
    pub async fn lock(&self) -> core::cell::RefMut<'_, T> {
        loop {
            match self.0.try_borrow_mut() {
                Ok(borrow) => break borrow,
                Err(_) => _yield().await,
            }
        }
    }
}
