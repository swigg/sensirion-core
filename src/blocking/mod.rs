use bytes::{BufMut, BytesMut};
use core::{fmt::Debug, time::Duration};
use embedded_hal::i2c::SevenBitAddress;

use crate::Error;

/// Defines an blocking interface for communication with Sensirion I2C sensors.
///
/// This trait encapsulates the necessary methods for interacting with Sensirion
/// sensors over an I2C bus using blocking operations. It provides a standardized
/// way to read from and write to these sensors, handling low-level communication details
/// and error management.
pub trait SensirionI2c<I2C, D>
where
    I2C: embedded_hal::i2c::I2c,
    D: embedded_hal::delay::DelayNs,
{
    /// Returns a mutable reference to the I2C controller.
    ///
    /// This method provides access to the underlying I2C interface,
    /// allowing for low-level communication with the device.
    fn i2c(&mut self) -> &mut I2C;

    /// Retrieves the 7-bit I2C address of the device.
    ///
    /// This method returns the unique address used to communicate
    /// with the Sensirion sensor on the I2C bus.
    fn address(&mut self) -> SevenBitAddress;

    /// Returns a mutable reference to the delay implementation.
    ///
    /// This method provides access to the delay mechanism used for
    /// timing operations in communication with the sensor.
    fn delay(&mut self) -> &mut D;

    /// Reads data from the sensor using a specified command.
    ///
    /// This method sends a command to the sensor and reads the response into the provided buffer.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to send to the sensor.
    /// * `buffer` - A mutable slice to store the response from the sensor.
    ///
    /// # Returns
    ///
    /// A result containing a mutable slice with the read data or an error.
    fn read_command<'a, T: Copy + Into<Duration> + Into<u16> + Debug>(
        &mut self,
        command: T,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error<I2C::Error>> {
        self.read_command_with_args(command, None, buffer)
    }

    /// Reads data from the sensor using a specified command and optional arguments.
    ///
    /// This method sends a command with optional arguments to the sensor and reads the response into the provided buffer.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to send to the sensor.
    /// * `args` - Optional arguments to send with the command.
    /// * `buffer` - A mutable slice to store the response from the sensor.
    ///
    /// # Returns
    ///
    /// A result containing a mutable slice with the read data or an error.
    fn read_command_with_args<'a, T: Copy + Into<Duration> + Into<u16> + Debug>(
        &mut self,
        command: T,
        args: Option<&[u16]>,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error<I2C::Error>> {
        self.write_command_with_args(command, args)?;
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

        sensirion_i2c::i2c::read_words_with_crc(&mut self.i2c(), address, buffer)
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

    /// Writes a command to the sensor.
    ///
    /// This method sends a command to the sensor without any additional arguments.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to send to the sensor.
    ///
    /// # Returns
    ///
    /// A result indicating success or an error if the write operation failed.
    fn write_command<T: Copy + Into<Duration> + Into<u16> + Debug>(
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
        sensirion_i2c::i2c::write_command_u16(&mut self.i2c(), address, command_code)
            .map_err(sensirion_i2c::i2c::Error::<I2C>::I2cWrite)?;
        Ok(())
    }

    /// Writes a command with optional arguments to the sensor.
    ///
    /// This method sends a command along with optional arguments to the sensor.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to send to the sensor.
    /// * `args` - Optional arguments to send with the command.
    ///
    /// # Returns
    ///
    /// A result indicating success or an error if the write operation failed.
    fn write_command_with_args<T: Copy + Into<Duration> + Into<u16> + Debug>(
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
            .map_err(sensirion_i2c::i2c::Error::<I2C>::I2cWrite)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use core::u16;
    const DEFAULT_BLOCKING_ADDRESS: SevenBitAddress = 4;
    const MINIMUM_DELAY: u64 = 1;

    use super::*;
    use alloc::vec::Vec;
    use assert_matches::assert_matches;
    use bytes::Buf;
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::{
        delay::NoopDelay,
        i2c::{Mock, Transaction},
    };

    #[derive(Debug, Default)]
    pub struct TestDriver<I2C, D>
    where
        I2C: embedded_hal::i2c::I2c<SevenBitAddress>,
    {
        i2c: I2C,
        address: SevenBitAddress,
        delay: D,
    }

    impl<I2C, D> SensirionI2c<I2C, D> for TestDriver<I2C, D>
    where
        I2C: embedded_hal::i2c::I2c,
        D: embedded_hal::delay::DelayNs,
    {
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

    #[derive(Debug, Clone, Copy)]
    enum Command {
        CommandOne,
    }

    impl From<Command> for u16 {
        fn from(value: Command) -> Self {
            match value {
                Command::CommandOne => 0x44,
            }
        }
    }

    impl From<Command> for Duration {
        fn from(value: Command) -> Self {
            match value {
                Command::CommandOne => Duration::from_millis(MINIMUM_DELAY),
            }
        }
    }

    fn create_i2c<T: FnOnce(TestDriver<&mut Mock, NoopDelay>)>(
        expectations: &[Transaction],
        action: T,
    ) {
        let mut i2c_mock = embedded_hal_mock::eh1::i2c::Mock::new(expectations);
        let test_driver = create_device(&mut i2c_mock);
        action(test_driver);
        i2c_mock.done();
    }

    fn create_device(i2c: &mut Mock) -> TestDriver<&mut Mock, NoopDelay> {
        TestDriver {
            i2c,
            address: DEFAULT_BLOCKING_ADDRESS,
            delay: NoopDelay::default(),
        }
    }

    #[test]
    fn test_new() {
        create_i2c(&[], |test_driver| {
            #[cfg(feature = "log")]
            log::info!("here {}", test_driver.address);
        });
    }

    #[test]
    fn test_read_command_success() {
        let mut response = BytesMut::with_capacity(3);
        response.put_u16(0x22);
        response.put_u8(sensirion_i2c::crc8::calculate(&response[0..2]));

        let expectations = [
            Transaction::write(
                DEFAULT_BLOCKING_ADDRESS,
                u16::from(Command::CommandOne).to_be_bytes().to_vec(),
            ),
            Transaction::read(DEFAULT_BLOCKING_ADDRESS, response.to_vec()),
        ];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert!(
                    test_driver
                        .read_command(Command::CommandOne, &mut buffer)
                        .is_ok()
                );
            },
        );
    }

    #[test]
    fn test_read_command_fail_crc() {
        let mut response = BytesMut::with_capacity(3);
        response.put_u16(0x22);
        response.put_u8(0);

        let expectations = [
            Transaction::write(
                DEFAULT_BLOCKING_ADDRESS,
                u16::from(Command::CommandOne).to_be_bytes().to_vec(),
            ),
            Transaction::read(DEFAULT_BLOCKING_ADDRESS, response.to_vec()),
        ];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert_matches!(
                    test_driver.read_command(Command::CommandOne, &mut buffer),
                    Err(Error::Crc)
                )
            },
        );
    }

    #[test]
    fn test_read_command_fail_nack() {
        let mut response = BytesMut::with_capacity(3);
        response.put_u16(0x22);
        response.put_u8(0);

        let expectations = [Transaction::write(
            DEFAULT_BLOCKING_ADDRESS,
            u16::from(Command::CommandOne).to_be_bytes().to_vec(),
        )
        .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert_matches!(
                    test_driver.read_command(Command::CommandOne, &mut buffer),
                    Err(Error::I2c(ErrorKind::NoAcknowledge(
                        NoAcknowledgeSource::Address
                    )))
                )
            },
        );
    }

    #[test]
    fn test_read_command_with_args_success() {
        let mut response = BytesMut::with_capacity(3);
        response.put_u16(0x22);
        response.put_u8(sensirion_i2c::crc8::calculate(&response[0..2]));

        let mut expected = BytesMut::with_capacity(4);
        // command
        expected.put_u16(Command::CommandOne.into());
        // args
        expected.put_u16(0x22);
        expected.put_u8(sensirion_i2c::crc8::calculate(&&expected[2..4]));

        let expectations = [
            Transaction::write(DEFAULT_BLOCKING_ADDRESS, expected.to_vec()),
            Transaction::read(DEFAULT_BLOCKING_ADDRESS, response.to_vec()),
        ];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert!(
                    test_driver
                        .read_command_with_args(
                            Command::CommandOne,
                            Some(
                                &expected[2..4]
                                    .chunks(2)
                                    .map(|mut c| c.get_u16())
                                    .collect::<Vec<u16>>()
                            ),
                            &mut buffer
                        )
                        .is_ok()
                );
            },
        );
    }

    #[test]
    fn test_read_command_with_args_fail_crc() {
        let mut response = BytesMut::with_capacity(3);
        response.put_u16(0x22);
        response.put_u8(0);

        let mut expected = BytesMut::with_capacity(4);
        // command
        expected.put_u16(Command::CommandOne.into());
        // args
        expected.put_u16(0x22);
        expected.put_u8(sensirion_i2c::crc8::calculate(&&expected[2..4]));

        let expectations = [
            Transaction::write(DEFAULT_BLOCKING_ADDRESS, expected.to_vec()),
            Transaction::read(DEFAULT_BLOCKING_ADDRESS, response.to_vec()),
        ];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert_matches!(
                    test_driver.read_command_with_args(
                        Command::CommandOne,
                        Some(
                            &expected[2..4]
                                .chunks(2)
                                .map(|mut c| c.get_u16())
                                .collect::<Vec<u16>>()
                        ),
                        &mut buffer
                    ),
                    Err(Error::Crc)
                )
            },
        );
    }

    #[test]
    fn test_read_command_with_args_fail_nack() {
        let mut expected = BytesMut::with_capacity(4);
        // command
        expected.put_u16(Command::CommandOne.into());
        // args
        expected.put_u16(0x22);
        expected.put_u8(sensirion_i2c::crc8::calculate(&&expected[2..4]));

        let expectations = [
            Transaction::write(DEFAULT_BLOCKING_ADDRESS, expected.to_vec())
                .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)),
        ];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert_matches!(
                    test_driver.read_command_with_args(
                        Command::CommandOne,
                        Some(
                            &expected[2..4]
                                .chunks(2)
                                .map(|mut c| c.get_u16())
                                .collect::<Vec<u16>>()
                        ),
                        &mut buffer
                    ),
                    Err(Error::I2c(ErrorKind::NoAcknowledge(
                        NoAcknowledgeSource::Address
                    )))
                )
            },
        );
    }

    #[test]
    fn test_write_command_success() {
        let expectations = [Transaction::write(
            DEFAULT_BLOCKING_ADDRESS,
            u16::from(Command::CommandOne).to_be_bytes().to_vec(),
        )];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert!(test_driver.write_command(Command::CommandOne).is_ok());
            },
        );
    }

    #[test]
    fn test_write_command_fail_nack() {
        let expectations = [Transaction::write(
            DEFAULT_BLOCKING_ADDRESS,
            u16::from(Command::CommandOne).to_be_bytes().to_vec(),
        )
        .with_error(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert_matches!(
                    test_driver.write_command(Command::CommandOne),
                    Err(Error::I2c(ErrorKind::NoAcknowledge(
                        NoAcknowledgeSource::Address
                    )))
                );
            },
        );
    }

    #[test]
    fn test_write_command_fail_other() {
        let expectations = [Transaction::write(
            DEFAULT_BLOCKING_ADDRESS,
            u16::from(Command::CommandOne).to_be_bytes().to_vec(),
        )
        .with_error(ErrorKind::Other)];

        create_i2c(
            &expectations,
            |mut test_driver: TestDriver<
                &mut embedded_hal_mock::common::Generic<Transaction>,
                NoopDelay,
            >| {
                let mut buffer = BytesMut::with_capacity(3);
                buffer.resize(3, 0);
                assert_matches!(
                    test_driver.write_command(Command::CommandOne),
                    Err(Error::I2c(ErrorKind::Other))
                );
            },
        );
    }
}
