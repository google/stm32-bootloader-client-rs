// Copyright 2022 The stm32-bootloader-client-rs Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

//! Communicates with the STM32 factory bootloader over i2c. See AN4221 for
//! details of how the factory bootloader works.

#![cfg_attr(not(any(test, feature = "std")), no_std)]

use core::ops::Deref;
use core::ops::DerefMut;
use embedded_hal::blocking::i2c::Read;
use embedded_hal::blocking::i2c::Write;

const BOOTLOADER_ACK: u8 = 0x79;
const BOOTLOADER_NACK: u8 = 0x1f;
const BOOTLOADER_BUSY: u8 = 0x76;

/// Configuration for communication with stm32 system bootloader.
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct Config {
    /// See AN2606 for the i2c address for the specific chip you're talking to.
    /// The i2c address will also depend on which i2c bus on the chip you're
    /// connected to.
    i2c_address: u8,

    /// The maximum number of milliseconds that a full flash erase can take.
    /// Search the datasheet for your specific STM32 for "mass erase time". If
    /// in doubt, round up.
    pub mass_erase_max_ms: u32,
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Command {
    GetVersion = 0x01,
    GetId = 0x02,
    ReadMemory = 0x11,
    Go = 0x21,
    WriteMemory = 0x31,
    Erase = 0x44,
}

const SPECIAL_ERASE_ALL: [u8; 2] = [0xff, 0xff];

pub const MAX_READ_WRITE_SIZE: usize = 128;

type Result<T, E = Error> = core::result::Result<T, E>;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    TransportError,
    Nack,
    NackFromCommand(u8),
    Busy,
    UnexpectedResponse,
    InvalidArgument,
    VerifyFailedAtAddress(u32),
    EraseFailed,
}

pub struct Stm32<'a, I2c: Write + Read, DelayMs> {
    dev: MaybeOwned<'a, I2c>,
    delay: MaybeOwned<'a, DelayMs>,
    config: Config,
}

#[derive(Debug, Clone)]
pub struct Progress {
    pub bytes_complete: usize,
    pub bytes_total: usize,
}

#[cfg(feature = "std")]
impl<E, I2c> Stm32<'static, I2c, StdDelay>
where
    E: core::fmt::Debug,
    I2c: Write<Error = E> + Read<Error = E>,
{
    pub fn new(dev: I2c, config: Config) -> Stm32<'static, I2c, StdDelay> {
        Self {
            dev: MaybeOwned::Owned(dev),
            delay: MaybeOwned::Owned(StdDelay),
            config,
        }
    }
}

#[cfg(feature = "std")]
impl<'a, E, I2c> Stm32<'a, I2c, StdDelay>
where
    E: core::fmt::Debug,
    I2c: Write<Error = E> + Read<Error = E>,
{
    /// Construct a new instance where we only borrow the I2C implementation.
    /// This is useful if you have other things on the I2C bus that want to
    /// communicate with so don't want to give up ownership of the I2C bus.
    pub fn borrowed(dev: &'a mut I2c, config: Config) -> Stm32<'a, I2c, StdDelay> {
        Self {
            dev: MaybeOwned::Borrowed(dev),
            delay: MaybeOwned::Owned(StdDelay),
            config,
        }
    }
}

impl<E, I2c, Delay> Stm32<'static, I2c, Delay>
where
    E: core::fmt::Debug,
    I2c: Write<Error = E> + Read<Error = E>,
    Delay: embedded_hal::blocking::delay::DelayMs<u32>,
{
    /// Constructs a new instance with a custom delay implementation.
    pub fn new_with_delay(dev: I2c, delay: Delay, config: Config) -> Stm32<'static, I2c, Delay> {
        Self {
            dev: MaybeOwned::Owned(dev),
            delay: MaybeOwned::Owned(delay),
            config,
        }
    }
}

impl<'a, E, I2c, Delay> Stm32<'a, I2c, Delay>
where
    E: core::fmt::Debug,
    I2c: Write<Error = E> + Read<Error = E>,
    Delay: embedded_hal::blocking::delay::DelayMs<u32>,
{
    /// Borrows both the I2C implementation and a custom delay.
    pub fn borrowed_with_delay(
        dev: &'a mut I2c,
        delay: &'a mut Delay,
        config: Config,
    ) -> Stm32<'a, I2c, Delay> {
        Self {
            dev: MaybeOwned::Borrowed(dev),
            delay: MaybeOwned::Borrowed(delay),
            config,
        }
    }

    pub fn get_chip_id(&mut self) -> Result<u16> {
        self.send_command(Command::GetId)?;
        // For STM32, the first byte will always be a 1 and the payload will
        // always be 3 bytes.
        let mut buffer = [0u8; 3];
        self.read(&mut buffer)?;
        self.get_ack_for_command(Command::GetId)?;
        Ok(u16::from_be_bytes([buffer[1], buffer[2]]))
    }

    /// Reads memory starting from `address`, putting the result into `out`.
    pub fn read_memory(&mut self, address: u32, out: &mut [u8]) -> Result<()> {
        if out.len() > MAX_READ_WRITE_SIZE {
            return Err(Error::InvalidArgument);
        }
        self.send_command(Command::ReadMemory)?;
        self.send_address(address)?;
        let mut buffer = [0u8; 2];
        buffer[0] = (out.len() - 1) as u8;
        buffer[1] = checksum(&buffer[0..1]);
        self.write(&buffer)?;
        self.get_ack_for_command(Command::ReadMemory)?;
        self.read(out)?;

        Ok(())
    }

    /// Writes `data` at `address`. Maximum write size is 256 bytes.
    pub fn write_memory(&mut self, address: u32, data: &[u8]) -> Result<()> {
        if data.len() > MAX_READ_WRITE_SIZE {
            return Err(Error::InvalidArgument);
        }
        self.send_command(Command::WriteMemory)?;
        self.send_address(address)?;

        let mut buffer = [0u8; MAX_READ_WRITE_SIZE + 2];
        buffer[0] = (data.len() - 1) as u8;
        buffer[1..1 + data.len()].copy_from_slice(data);
        buffer[1 + data.len()] = checksum(&buffer[0..1 + data.len()]);
        self.write(&buffer[..data.len() + 2])?;

        self.get_ack_for_command(Command::WriteMemory)
    }

    /// Writes `bytes` to `address`, calling `progress_cb` after each block.
    pub fn write_bulk(
        &mut self,
        mut address: u32,
        bytes: &[u8],
        mut progress_cb: impl FnMut(Progress),
    ) -> Result<()> {
        let mut complete = 0;
        for chunk in bytes.chunks(MAX_READ_WRITE_SIZE) {
            // Write-memory sometimes gets a NACK, so allow a single retry.
            if self.write_memory(address, chunk).is_err() {
                self.write_memory(address, chunk)?;
            }
            complete += chunk.len();
            address += chunk.len() as u32;
            progress_cb(Progress {
                bytes_complete: complete,
                bytes_total: bytes.len(),
            });
        }
        Ok(())
    }

    /// Verifies that memory at `address` is equal to `bytes`, calling
    /// `progress_cb` to report progress.
    pub fn verify(
        &mut self,
        mut address: u32,
        bytes: &[u8],
        mut progress_cb: impl FnMut(Progress),
    ) -> Result<()> {
        let mut read_back_buffer = [0; MAX_READ_WRITE_SIZE];
        let mut complete = 0;
        for chunk in bytes.chunks(MAX_READ_WRITE_SIZE) {
            let read_back = &mut read_back_buffer[..chunk.len()];
            self.read_memory(address, read_back)?;
            for (offset, (expected, actual)) in chunk.iter().zip(read_back.iter()).enumerate() {
                if expected != actual {
                    return Err(Error::VerifyFailedAtAddress(address + offset as u32));
                }
            }
            complete += chunk.len();
            address += chunk.len() as u32;
            progress_cb(Progress {
                bytes_complete: complete,
                bytes_total: bytes.len(),
            });
        }
        Ok(())
    }

    /// Erase the flash of the STM32.
    pub fn erase_flash(&mut self) -> Result<()> {
        self.send_command(Command::Erase)?;
        let mut buffer = [0u8; 3];
        buffer[0..2].copy_from_slice(&SPECIAL_ERASE_ALL);
        buffer[2] = checksum(&buffer[..2]);
        self.write(&buffer)?;
        self.delay.delay_ms(self.config.mass_erase_max_ms);
        self.get_ack().map_err(|_| Error::EraseFailed)
    }

    /// Returns the version number of the bootloader.
    pub fn get_bootloader_version(&mut self) -> Result<u8> {
        self.send_command(Command::GetVersion)?;
        let mut buffer = [0];
        self.read(&mut buffer)?;
        self.get_ack_for_command(Command::GetVersion)?;
        Ok(buffer[0])
    }

    /// Exit system bootloader by jumping to the reset vector specified in the
    /// vector table at `address`.
    pub fn go(&mut self, address: u32) -> Result<()> {
        self.send_command(Command::Go)?;
        self.send_address(address)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        self.dev
            .write(self.config.i2c_address, bytes)
            .map_err(|error| {
                log_error(&error);
                Error::TransportError
            })
    }

    fn read(&mut self, out: &mut [u8]) -> Result<()> {
        self.dev
            .read(self.config.i2c_address, out)
            .map_err(|error| {
                log_error(&error);
                Error::TransportError
            })
    }

    fn read_with_timeout(&mut self, out: &mut [u8]) -> Result<()> {
        // TODO: Implement timeout mechanism
        const MAX_ATTEMPTS: u32 = 10000;
        let mut attempts = 0;
        loop {
            attempts += 1;
            let result = self.read(out);
            if result.is_ok() || attempts == MAX_ATTEMPTS {
                return result;
            }
        }
    }

    fn send_command(&mut self, command: Command) -> Result<()> {
        let command_u8 = command as u8;
        self.write(&[command_u8, !command_u8])?;
        self.get_ack_for_command(command)
    }

    fn get_ack(&mut self) -> Result<()> {
        let mut response = [0u8; 1];
        self.read_with_timeout(&mut response)?;
        match response[0] {
            BOOTLOADER_ACK => Ok(()),
            BOOTLOADER_NACK => Err(Error::Nack),
            BOOTLOADER_BUSY => Err(Error::Busy),
            _ => Err(Error::UnexpectedResponse),
        }
    }

    fn get_ack_for_command(&mut self, command: Command) -> Result<()> {
        self.get_ack().map_err(|error| match error {
            Error::Nack => Error::NackFromCommand(command as u8),
            x => x,
        })
    }

    fn send_address(&mut self, address: u32) -> Result<()> {
        let mut buffer = [0u8; 5];
        buffer[0..4].copy_from_slice(&address.to_be_bytes());
        buffer[4] = checksum(&buffer[0..4]);
        self.write(&buffer)?;
        self.get_ack()
    }
}

/// Wraps either an owned value, or a mutable reference. This sounds a little
/// bit like std::borrow::Cow, but is actually quite different in that (a) it
/// doesn't rely on std, (b) it mutates via either variant without having to
/// switch to an owned variant and (c) it works with types that we can't, or
/// don't want to clone.
enum MaybeOwned<'a, T> {
    Borrowed(&'a mut T),
    Owned(T),
}

impl<'a, T> Deref for MaybeOwned<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        match self {
            MaybeOwned::Borrowed(x) => *x,
            MaybeOwned::Owned(x) => x,
        }
    }
}

impl<'a, T> DerefMut for MaybeOwned<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            MaybeOwned::Borrowed(x) => *x,
            MaybeOwned::Owned(x) => x,
        }
    }
}

/// An implementation of the embedded-hal DelayMs trait that works when std is
/// available.
#[cfg(feature = "std")]
pub struct StdDelay;

#[cfg(feature = "std")]
impl embedded_hal::blocking::delay::DelayMs<u32> for StdDelay {
    fn delay_ms(&mut self, ms: u32) {
        std::thread::sleep(std::time::Duration::from_millis(ms.into()));
    }
}

fn checksum(bytes: &[u8]) -> u8 {
    let initial = if bytes.len() == 1 { 0xff } else { 0 };
    bytes
        .iter()
        .fold(initial, |checksum, value| checksum ^ value)
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::TransportError => write!(f, "Transport error"),
            Error::Nack => write!(f, "NACK"),
            Error::NackFromCommand(command) => write!(f, "Nack from command {}", command),
            Error::Busy => write!(f, "Busy"),
            Error::UnexpectedResponse => write!(f, "Unexpected response"),
            Error::InvalidArgument => write!(f, "Invalid argument"),
            Error::VerifyFailedAtAddress(address) => {
                write!(f, "Verify failed at address {:x}", address)
            }
            Error::EraseFailed => write!(f, "Erase failed"),
        }
    }
}

impl Config {
    pub const fn i2c_address(i2c_address: u8) -> Self {
        Self {
            i2c_address,
            // A moderately conservative default. stm32g071 has 40.1ms.
            // stm32l452 has 24.59ms
            mass_erase_max_ms: 200,
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

fn log_error<E: core::fmt::Debug>(_error: &E) {
    #[cfg(feature = "defmt")]
    {
        defmt::error!("I2C error: {:?}", defmt::Debug2Format(_error));
    }
    #[cfg(feature = "log")]
    {
        log::error!("I2C error: {:?}", _error);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::i2c;

    const I2C_ADDRESS: u8 = 0x51;
    const CONFIG: Config = Config::i2c_address(I2C_ADDRESS);

    fn mock_write_with_checksum(bytes: &[u8]) -> i2c::Transaction {
        let mut with_checksum = Vec::with_capacity(bytes.len() + 1);
        with_checksum.extend_from_slice(bytes);
        with_checksum.push(checksum(bytes));
        i2c::Transaction::write(I2C_ADDRESS, with_checksum)
    }

    fn mock_read(bytes: &[u8]) -> i2c::Transaction {
        i2c::Transaction::read(I2C_ADDRESS, bytes.to_owned())
    }

    struct Delay;

    impl embedded_hal::blocking::delay::DelayMs<u32> for Delay {
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_get_chip_id() {
        let expectations = [
            mock_write_with_checksum(&[Command::GetId as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[0, 0xab, 0xcd]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        assert_eq!(stm32.get_chip_id().unwrap(), 0xabcd);
        i2c.done();
    }

    #[test]
    fn test_get_bootloader_version() {
        let expectations = [
            mock_write_with_checksum(&[Command::GetVersion as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[0xef]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        assert_eq!(stm32.get_bootloader_version().unwrap(), 0xef);
        i2c.done();
    }

    #[test]
    fn test_read_memory() {
        let expectations = [
            mock_write_with_checksum(&[Command::ReadMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x02]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&[0xab, 0xcd, 0xef]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        let mut out = [0u8; 3];
        stm32.read_memory(0x12345678, &mut out).unwrap();
        assert_eq!(&out, &[0xab, 0xcd, 0xef]);
        i2c.done();
    }

    #[test]
    fn test_write_memory() {
        let expectations = [
            mock_write_with_checksum(&[Command::WriteMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x03, 0xab, 0xcd, 0xef, 0x12]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        stm32
            .write_memory(0x12345678, &[0xab, 0xcd, 0xef, 0x12])
            .unwrap();
        i2c.done();
    }

    #[test]
    fn test_go() {
        let expectations = [
            mock_write_with_checksum(&[Command::Go as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        stm32.go(0x12345678).unwrap();
        i2c.done();
    }

    #[test]
    fn test_write_bulk() {
        let to_write: Vec<u8> = (0..200).collect();
        let mut write1 = vec![127u8];
        write1.extend_from_slice(&to_write[..128]);
        let mut write2 = vec![(to_write.len() - 128 - 1) as u8];
        write2.extend_from_slice(&to_write[128..]);
        let expectations = [
            mock_write_with_checksum(&[Command::WriteMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&write1),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[Command::WriteMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0xf8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&write2),
            mock_read(&[BOOTLOADER_ACK]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        let mut callback_count = 0;
        stm32
            .write_bulk(0x12345678, &to_write, |_| {
                callback_count += 1;
            })
            .unwrap();
        assert_eq!(callback_count, 2);
        i2c.done();
    }

    #[test]
    fn test_verify() {
        let to_verify: Vec<u8> = (0..200).collect();
        let expectations = [
            mock_write_with_checksum(&[Command::ReadMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0x78]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[127]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&to_verify[..128]),
            mock_write_with_checksum(&[Command::ReadMemory as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[0x12, 0x34, 0x56, 0xf8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_write_with_checksum(&[(to_verify.len() - 128 - 1) as u8]),
            mock_read(&[BOOTLOADER_ACK]),
            mock_read(&to_verify[128..]),
        ];
        let mut i2c = i2c::Mock::new(&expectations);
        let mut delay = Delay;
        let mut stm32 = Stm32::borrowed_with_delay(&mut i2c, &mut delay, CONFIG);
        let mut callback_count = 0;
        stm32
            .verify(0x12345678, &to_verify, |_| {
                callback_count += 1;
            })
            .unwrap();
        assert_eq!(callback_count, 2);
        i2c.done();
    }
}
