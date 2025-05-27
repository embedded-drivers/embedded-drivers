//! Driver for SHT20.

#![no_std]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

pub const ADDRESS: u8 = 0x40;

/// Commands for SHT20
pub mod cmds {
    pub const TEMP_HOLD: u8 = 0xE3;
    pub const HUM_HOLD: u8 = 0xE5;
    pub const TEMP_NOHOLD: u8 = 0xF3;
    pub const HUM_NOHOLD: u8 = 0xF5;
    pub const WRITE_REG: u8 = 0xE6;
    pub const READ_REG: u8 = 0xE7;
    pub const SOFT_RESET: u8 = 0xFE;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    CRC,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Bus(error)
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, Default)]
pub enum Resolution {
    #[default]
    RH12_T14 = 0b0000_0000,
    RH8_T12 = 0b0000_0001,
    RH10_T13 = 0b1000_0000,
    RH11_T11 = 0b1000_0001,
}

pub struct SHT20<I2C: embedded_hal_async::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    resolution: Resolution,
}

impl<I2C: embedded_hal_async::i2c::I2c> SHT20<I2C> {
    pub fn new_primary(i2c: I2C) -> Self {
        Self {
            addr: ADDRESS,
            i2c,
            resolution: Resolution::RH12_T14,
        }
    }

    pub async fn set_resolution(&mut self, resolution: Resolution) -> Result<(), Error<I2C::Error>> {
        self.resolution = resolution;

        let mut bits = resolution as u8;
        bits = bits | 0b10; // disable on-chip heater, enable OTP reload

        self.i2c.write(self.addr, &[cmds::WRITE_REG, bits]).await?;

        Ok(())
    }

    pub async fn read_temperature(&mut self, mut delay: impl DelayNs) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.i2c.write(self.addr, &[cmds::TEMP_HOLD]).await?;

        delay.delay_ms(100).await;

        self.i2c.read(self.addr, &mut buf).await?;

        let raw = u16::from_be_bytes([buf[0], buf[1]]);
        let raw = raw & !0x0003;

        let temp = -46.85 + 175.72 * (raw as f32 / 65536.0);
        Ok(temp)
    }

    pub async fn read_humidity(&mut self, mut delay: impl DelayNs) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.i2c.write(self.addr, &[cmds::HUM_HOLD]).await?;
        delay.delay_ms(40).await;
        self.i2c.read(self.addr, &mut buf).await?;

        let raw = u16::from_be_bytes([buf[0], buf[1]]);
        let raw = raw & !0x0003;

        let hum = -6.0 + 125.0 * (raw as f32 / 65536.0);
        Ok(hum)
    }
}
