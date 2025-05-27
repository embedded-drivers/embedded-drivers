use embedded_hal::delay::DelayNs;

use crate::{cmds, Error, Resolution, ADDRESS};

pub struct SHT20<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    resolution: Resolution,
}

impl<I2C: embedded_hal::i2c::I2c> SHT20<I2C> {
    pub fn new_primary(i2c: I2C) -> Self {
        Self {
            addr: ADDRESS,
            i2c,
            resolution: Resolution::RH12_T14,
        }
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.resolution = resolution;

        let mut bits = resolution as u8;
        bits = bits | 0b10; // disable on-chip heater, enable OTP reload

        self.i2c.write(self.addr, &[cmds::WRITE_REG, bits]).unwrap();
    }

    pub fn read_temperature(&mut self, mut delay: impl DelayNs) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.i2c.write(self.addr, &[cmds::TEMP_HOLD])?;

        delay.delay_ms(100);

        self.i2c.read(self.addr, &mut buf)?;

        let raw = u16::from_be_bytes([buf[0], buf[1]]);
        let raw = raw & !0x0003;

        let temp = -46.85 + 175.72 * (raw as f32 / 65536.0);
        Ok(temp)
    }

    pub fn read_humidity(&mut self, mut delay: impl DelayNs) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 3];
        self.i2c.write(self.addr, &[cmds::HUM_HOLD])?;
        delay.delay_ms(40);
        self.i2c.read(self.addr, &mut buf)?;

        let raw = u16::from_be_bytes([buf[0], buf[1]]);
        let raw = raw & !0x0003;

        let hum = -6.0 + 125.0 * (raw as f32 / 65536.0);
        Ok(hum)
    }
}
