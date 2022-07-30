use embedded_time::{clock::Error, rate::Fraction, Clock, Instant};
use rp2040_hal::timer::Timer;

pub struct MyClock<'a>(pub &'a Timer);

impl<'a> Clock for MyClock<'a> {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1_000_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        Ok(Instant::new(self.0.get_counter_low()))
    }
}
