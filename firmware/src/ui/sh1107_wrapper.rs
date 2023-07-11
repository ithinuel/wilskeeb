use core::ops::{Deref, DerefMut};
use embedded_hal::i2c::{ErrorType, SevenBitAddress};
use hal::{
    gpio::{bank0, FunctionNull, Pin, PullDown},
    pac::PIO1,
    pio::SM0,
};
use rp2040_async_i2c::pio::I2C;
use sparkfun_pro_micro_rp2040::hal;
pub(crate) type I2CPeriphInner<'pio> = I2C<
    'pio,
    PIO1,
    SM0,
    Pin<bank0::Gpio16, FunctionNull, PullDown>,
    Pin<bank0::Gpio17, FunctionNull, PullDown>,
>;

pub struct I2CPeriph<'pio>(pub I2CPeriphInner<'pio>);

impl<'pio> Deref for I2CPeriph<'pio> {
    type Target = I2CPeriphInner<'pio>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for I2CPeriph<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<'pio> ErrorType for I2CPeriph<'pio> {
    type Error = <I2CPeriphInner<'pio> as ErrorType>::Error;
}

impl sh1107::WriteIter<SevenBitAddress> for I2CPeriph<'_> {
    async fn write_iter<'a, U>(
        &'a mut self,
        address: SevenBitAddress,
        bytes: U,
    ) -> Result<(), embedded_hal::i2c::ErrorKind>
    where
        U: IntoIterator<Item = u8> + 'a,
    {
        self.0.write_iter(address, bytes).await
    }
}
