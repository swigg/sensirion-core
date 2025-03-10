use bytes::{BufMut, BytesMut};
use core::{fmt::Debug, time::Duration};
use embedded_hal::i2c::SevenBitAddress;

use crate::Error;

///
pub trait SensirionI2c<I2C, A, D> {
    ///
    fn i2c(&mut self) -> &mut I2C;

    ///
    fn address(&mut self) -> A;

    ///
    fn delay(&mut self) -> &mut D;
}

impl<I2C, D> dyn SensirionI2c<I2C, SevenBitAddress, D>
where
    I2C: embedded_hal_async::i2c::I2c + Debug,
    D: embedded_hal_async::delay::DelayNs,
{
    async fn read_command<'a, T: Copy + Into<Duration> + Into<u16> + Debug>(
        &mut self,
        command: T,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error<I2C::Error>> {
        self.read_command_with_args(command, None, buffer).await
    }

    async fn read_command_with_args<'a, T: Copy + Into<Duration> + Into<u16> + Debug>(
        &mut self,
        command: T,
        args: Option<&[u16]>,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error<I2C::Error>> {
        self.write_command_with_args(command, args).await?;
        let command_code: u16 = command.into();
        let address = self.address();

        let delay_duration: Duration = command.into();
        #[cfg(feature = "log")]
        log::trace!(
            "Waiting {:?} after sending {{address: {:#x?}, command: {:?}[{}]}}",
            &delay_duration,
            self.address(),
            command,
            u16::from(command_code)
                .to_be_bytes()
                .iter()
                .map(|b| alloc::format!("{:#x?}", b))
                .collect::<alloc::vec::Vec<_>>()
                .join(", "),
        );
        self.delay().delay_ns(delay_duration.as_nanos() as u32);

        sensirion_i2c::i2c_async::read_words_with_crc(&mut self.i2c(), address, buffer)
            .await
            .map_err(Error::from)?;

        #[cfg(feature = "log")]
        log::trace!(
            "Read from bus {{address: {:#x?}, command: {:?}[{}], response: [{}]}}",
            self.address(),
            command,
            u16::from(command_code)
                .to_be_bytes()
                .iter()
                .map(|b| alloc::format!("{:#x?}", b))
                .collect::<alloc::vec::Vec<_>>()
                .join(", "),
            &buffer[..]
                .iter()
                .map(|b| alloc::format!("{:#x?}", b))
                .collect::<alloc::vec::Vec<_>>()
                .join(", "),
        );

        Ok(buffer)
    }

    async fn write_command<T: Copy + Into<Duration> + Into<u16> + Debug>(
        &mut self,
        command: T,
    ) -> Result<(), Error<I2C::Error>> {
        let command_code: u16 = command.into();
        let address = self.address();

        #[cfg(feature = "log")]
        log::trace!(
            "Writing to bus {{address: {:#x?}, command: {:?}[{}]}}",
            self.address(),
            command,
            u16::from(command_code)
                .to_be_bytes()
                .iter()
                .map(|b| alloc::format!("{:#x?}", b))
                .collect::<alloc::vec::Vec<_>>()
                .join(", "),
        );
        sensirion_i2c::i2c_async::write_command_u16(&mut self.i2c(), address, command_code)
            .await
            .map_err(sensirion_i2c::i2c::Error::<I2C>::I2cWrite)?;
        Ok(())
    }

    async fn write_command_with_args<T: Copy + Into<Duration> + Into<u16> + Debug>(
        &mut self,
        command: T,
        args: Option<&[u16]>,
    ) -> Result<(), Error<I2C::Error>> {
        let mut buffer = BytesMut::with_capacity(8);
        let command_code: u16 = command.into();
        let address = self.address();

        buffer.put_u16(command.into());
        if let Some(args) = args {
            args.iter().for_each(|arg| {
                buffer.put_u16(*arg);
                buffer.put_u8(sensirion_i2c::crc8::calculate(&(*arg).to_be_bytes()[..]))
            });

            #[cfg(feature = "log")]
            log::trace!(
                "Writing to bus {{address: {:#x?}, command: {:?}[{}], args: [{}]}}",
                self.address(),
                command,
                u16::from(command_code)
                    .to_be_bytes()
                    .iter()
                    .map(|b| alloc::format!("{:#x?}", b))
                    .collect::<alloc::vec::Vec<_>>()
                    .join(", "),
                &buffer[2..]
                    .iter()
                    .map(|b| alloc::format!("{:#x?}", b))
                    .collect::<alloc::vec::Vec<_>>()
                    .join(", "),
            );
        } else {
            #[cfg(feature = "log")]
            log::trace!(
                "Writing to bus {{address: {:#x?}, command: {:?}[{}]}}",
                self.address(),
                command,
                u16::from(command_code)
                    .to_be_bytes()
                    .iter()
                    .map(|b| alloc::format!("{:#x?}", b))
                    .collect::<alloc::vec::Vec<_>>()
                    .join(", "),
            );
        }

        self.i2c()
            .write(address, &buffer[..])
            .await
            .map_err(sensirion_i2c::i2c_async::Error::<I2C>::I2cWrite)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use core::ops::AsyncFnOnce;

    use super::*;
    use crate::AddressPin;
    use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Transaction};
    use measurements::{Humidity, Temperature};

    #[derive(Debug, Default)]
    pub struct TestDriver<I2C, A, D> {
        i2c: I2C,
        address: A,
        delay: D,
    }

    impl<I2C, D> SensirionI2c<I2C, SevenBitAddress, D> for TestDriver<I2C, SevenBitAddress, D> {
        fn i2c(&mut self) -> &mut I2C {
            &mut self.i2c
        }

        fn address(&mut self) -> SevenBitAddress {
            self.address
        }

        fn delay(&mut self) -> &mut D {
            &mut self.delay
        }
    }

    fn create_i2c<F>(expectations: &[Transaction], action: F)
    where
        F: AsyncFnOnce(
            TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                SevenBitAddress,
                NoopDelay,
            >,
        ),
    {
        let mut i2c_mock = embedded_hal_mock::eh1::i2c::Mock::new(expectations);
        let test_driver = create_device(&mut i2c_mock);
        futures::executor::block_on(action(test_driver));
        i2c_mock.done();
    }

    fn create_device(
        i2c: &mut embedded_hal_mock::common::Generic<Transaction>,
    ) -> TestDriver<&mut embedded_hal_mock::common::Generic<Transaction>, u8, NoopDelay> {
        TestDriver {
            i2c,
            address: 0x4,
            delay: NoopDelay::default(),
        }
    }

    #[test]
    fn test_new() {
        create_i2c(&[], async |test_driver| {
            log::info!("here {}", test_driver.address);
        });
    }
}
