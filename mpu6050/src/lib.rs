//! Driver for MPU6050.

#![cfg_attr(not(test), no_std)]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

pub const PRIMARY_ADDRESS: u8 = 0x68;
pub const SECONDARY_ADDRESS: u8 = 0x69;

/// `PWR_MGMT_1` value that leaves sleep and selects the X-gyro PLL clock.
///
/// Register map RM-MPU-6000A-00, section 4.30: `CLKSEL = 0` is the internal
/// 8 MHz oscillator and `CLKSEL = 1` is "PLL with X axis gyroscope reference",
/// and "it is highly recommended that the device be configured to use one of
/// the gyroscopes (or an external clock source) as the clock reference for
/// improved stability". The product specification (section 6.6) quantifies that:
/// the internal oscillator has -5%/+5% initial tolerance and -15%/+10% over
/// temperature, against +/-1% for the gyro PLL. That error propagates straight
/// into the sample rate, so `0x00` is not merely a preference.
///
/// Bit 6 (SLEEP) is 0, so this also takes the part out of sleep.
pub const PWR_MGMT_1_CLKSEL_PLL_X_GYRO: u8 = 0x01;

/// How long `init` waits after wake-up, in milliseconds.
///
/// The product specification (section 6.1) lists the gyroscope start-up time as
/// "ZRO Settling (from power-on) to +/-1 dps of Final" = 30 ms typical, and
/// section 7.15 notes that the MEMS oscillators have to stabilise before they
/// are selected as the clock source. The sensor registers read as zero until the
/// first conversion has completed, so without this wait a caller's first read
/// returns zeros.
///
/// Note the datasheet gives this as a **typical** value and for power-on rather
/// than wake-from-sleep; 30 ms is the closest documented figure and is used as
/// a floor, not a guarantee.
pub const WAKE_UP_DELAY_MS: u32 = 30;

pub mod regs {
    pub const XG_OFFS_TC: u8 = 0x00; //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
    pub const YG_OFFS_TC: u8 = 0x01; //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
    pub const ZG_OFFS_TC: u8 = 0x02; //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
    pub const X_FINE_GAIN: u8 = 0x03; //[7:0] X_FINE_GAIN
    pub const Y_FINE_GAIN: u8 = 0x04; //[7:0] Y_FINE_GAIN
    pub const Z_FINE_GAIN: u8 = 0x05; //[7:0] Z_FINE_GAIN
    pub const XA_OFFS_H: u8 = 0x06; //[15:0] XA_OFFS
    pub const XA_OFFS_L_TC: u8 = 0x07;
    pub const YA_OFFS_H: u8 = 0x08; //[15:0] YA_OFFS
    pub const YA_OFFS_L_TC: u8 = 0x09;
    pub const ZA_OFFS_H: u8 = 0x0A; //[15:0] ZA_OFFS
    pub const ZA_OFFS_L_TC: u8 = 0x0B;
    pub const XG_OFFS_USRH: u8 = 0x13; //[15:0] XG_OFFS_USR
    pub const XG_OFFS_USRL: u8 = 0x14;
    pub const YG_OFFS_USRH: u8 = 0x15; //[15:0] YG_OFFS_USR
    pub const YG_OFFS_USRL: u8 = 0x16;
    pub const ZG_OFFS_USRH: u8 = 0x17; //[15:0] ZG_OFFS_USR
    pub const ZG_OFFS_USRL: u8 = 0x18;
    pub const SMPLRT_DIV: u8 = 0x19;
    pub const CONFIG: u8 = 0x1A;
    pub const GYRO_CONFIG: u8 = 0x1B;
    pub const ACCEL_CONFIG: u8 = 0x1C;
    pub const FF_THR: u8 = 0x1D;
    pub const FF_DUR: u8 = 0x1E;
    pub const MOT_THR: u8 = 0x1F;
    pub const MOT_DUR: u8 = 0x20;
    pub const ZRMOT_THR: u8 = 0x21;
    pub const ZRMOT_DUR: u8 = 0x22;
    pub const FIFO_EN: u8 = 0x23;
    pub const I2C_MST_CTRL: u8 = 0x24;
    pub const I2C_SLV0_ADDR: u8 = 0x25;
    pub const I2C_SLV0_REG: u8 = 0x26;
    pub const I2C_SLV0_CTRL: u8 = 0x27;
    pub const I2C_SLV1_ADDR: u8 = 0x28;
    pub const I2C_SLV1_REG: u8 = 0x29;
    pub const I2C_SLV1_CTRL: u8 = 0x2A;
    pub const I2C_SLV2_ADDR: u8 = 0x2B;
    pub const I2C_SLV2_REG: u8 = 0x2C;
    pub const I2C_SLV2_CTRL: u8 = 0x2D;
    pub const I2C_SLV3_ADDR: u8 = 0x2E;
    pub const I2C_SLV3_REG: u8 = 0x2F;
    pub const I2C_SLV3_CTRL: u8 = 0x30;
    pub const I2C_SLV4_ADDR: u8 = 0x31;
    pub const I2C_SLV4_REG: u8 = 0x32;
    pub const I2C_SLV4_DO: u8 = 0x33;
    pub const I2C_SLV4_CTRL: u8 = 0x34;
    pub const I2C_SLV4_DI: u8 = 0x35;
    pub const I2C_MST_STATUS: u8 = 0x36;
    pub const INT_PIN_CFG: u8 = 0x37;
    pub const INT_ENABLE: u8 = 0x38;
    pub const DMP_INT_STATUS: u8 = 0x39;
    pub const INT_STATUS: u8 = 0x3A;
    pub const ACCEL_XOUT_H: u8 = 0x3B;
    pub const ACCEL_XOUT_L: u8 = 0x3C;
    pub const ACCEL_YOUT_H: u8 = 0x3D;
    pub const ACCEL_YOUT_L: u8 = 0x3E;
    pub const ACCEL_ZOUT_H: u8 = 0x3F;
    pub const ACCEL_ZOUT_L: u8 = 0x40;
    pub const TEMP_OUT_H: u8 = 0x41;
    pub const TEMP_OUT_L: u8 = 0x42;
    pub const GYRO_XOUT_H: u8 = 0x43;
    pub const GYRO_XOUT_L: u8 = 0x44;
    pub const GYRO_YOUT_H: u8 = 0x45;
    pub const GYRO_YOUT_L: u8 = 0x46;
    pub const GYRO_ZOUT_H: u8 = 0x47;
    pub const GYRO_ZOUT_L: u8 = 0x48;
    pub const EXT_SENS_DATA_00: u8 = 0x49;
    pub const EXT_SENS_DATA_01: u8 = 0x4A;
    pub const EXT_SENS_DATA_02: u8 = 0x4B;
    pub const EXT_SENS_DATA_03: u8 = 0x4C;
    pub const EXT_SENS_DATA_04: u8 = 0x4D;
    pub const EXT_SENS_DATA_05: u8 = 0x4E;
    pub const EXT_SENS_DATA_06: u8 = 0x4F;
    pub const EXT_SENS_DATA_07: u8 = 0x50;
    pub const EXT_SENS_DATA_08: u8 = 0x51;
    pub const EXT_SENS_DATA_09: u8 = 0x52;
    pub const EXT_SENS_DATA_10: u8 = 0x53;
    pub const EXT_SENS_DATA_11: u8 = 0x54;
    pub const EXT_SENS_DATA_12: u8 = 0x55;
    pub const EXT_SENS_DATA_13: u8 = 0x56;
    pub const EXT_SENS_DATA_14: u8 = 0x57;
    pub const EXT_SENS_DATA_15: u8 = 0x58;
    pub const EXT_SENS_DATA_16: u8 = 0x59;
    pub const EXT_SENS_DATA_17: u8 = 0x5A;
    pub const EXT_SENS_DATA_18: u8 = 0x5B;
    pub const EXT_SENS_DATA_19: u8 = 0x5C;
    pub const EXT_SENS_DATA_20: u8 = 0x5D;
    pub const EXT_SENS_DATA_21: u8 = 0x5E;
    pub const EXT_SENS_DATA_22: u8 = 0x5F;
    pub const EXT_SENS_DATA_23: u8 = 0x60;
    pub const MOT_DETECT_STATUS: u8 = 0x61;
    pub const I2C_SLV0_DO: u8 = 0x63;
    pub const I2C_SLV1_DO: u8 = 0x64;
    pub const I2C_SLV2_DO: u8 = 0x65;
    pub const I2C_SLV3_DO: u8 = 0x66;
    pub const I2C_MST_DELAY_CTRL: u8 = 0x67;
    pub const SIGNAL_PATH_RESET: u8 = 0x68;
    pub const MOT_DETECT_CTRL: u8 = 0x69;
    pub const USER_CTRL: u8 = 0x6A;
    pub const PWR_MGMT_1: u8 = 0x6B;
    pub const PWR_MGMT_2: u8 = 0x6C;
    pub const BANK_SEL: u8 = 0x6D;
    pub const MEM_START_ADDR: u8 = 0x6E;
    pub const MEM_R_W: u8 = 0x6F;
    pub const DMP_CFG_1: u8 = 0x70;
    pub const DMP_CFG_2: u8 = 0x71;
    pub const FIFO_COUNTH: u8 = 0x72;
    pub const FIFO_COUNTL: u8 = 0x73;
    pub const FIFO_R_W: u8 = 0x74;
    pub const WHO_AM_I: u8 = 0x75;
}

pub mod consts {
    pub const DEV_ID_MPU6050: u8 = 0x68;
    pub const DEV_ID_MPU6500: u8 = 0x70;
    pub const DEV_ID_MPU9250: u8 = 0x71;
    pub const DEV_ID_MPU9255: u8 = 0x73;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    InvalidDevice,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

/// DLPF_CFG
/// Digital low pass filter configuration
#[derive(Debug, Clone, Copy, Default)]
pub enum DlpfCfg {
    /// 260Hz,
    Hz260 = 0,
    /// 184Hz
    #[default]
    Hz184 = 1,
    /// 94Hz
    Hz94 = 2,
    /// 44Hz
    Hz44 = 3,
    /// 21Hz
    Hz21 = 4,
    /// 10Hz
    Hz10 = 5,
    /// 5Hz
    Hz5 = 6,
}

/// FS_SEL
#[derive(Debug, Clone, Copy, Default)]
pub enum GyroRange {
    /// ±250 degrees per second
    Deg250 = 0,
    /// ±500 degrees per second
    Deg500 = 1,
    /// ±1000 degrees per second
    #[default]
    Deg1000 = 2,
    /// ±2000 degrees per second
    Deg2000 = 3,
}

impl GyroRange {
    #[inline]
    fn lsb_sensitivity(&self) -> f32 {
        match *self {
            GyroRange::Deg250 => 131.0,
            GyroRange::Deg500 => 65.5,
            GyroRange::Deg1000 => 32.8,
            GyroRange::Deg2000 => 16.4,
        }
    }

    /// `FS_SEL` field value for `GYRO_CONFIG`.
    ///
    /// `FS_SEL` lives in bits 4:3, so the discriminant has to be shifted.
    /// Writing it unshifted leaves the chip on its default range while the
    /// conversion maths assumes the requested one, silently scaling every gyro
    /// reading (Deg1000 was 4x too large).
    #[inline]
    pub const fn config_bits(self) -> u8 {
        (self as u8) << 3
    }
}

/// AFS_SEL
/// Full scale range for the accelerometer
#[derive(Debug, Clone, Copy, Default)]
pub enum AccelRange {
    /// ±2g
    #[default]
    G2 = 0,
    /// ±4g
    G4 = 1,
    /// ±8g
    G8 = 2,
    /// ±16g
    G16 = 3,
}

impl AccelRange {
    #[inline]
    fn lsb_sensitivity(&self) -> f32 {
        match *self {
            AccelRange::G2 => 16384.0,
            AccelRange::G4 => 8192.0,
            AccelRange::G8 => 4096.0,
            AccelRange::G16 => 2048.0,
        }
    }

    /// `AFS_SEL` field value for `ACCEL_CONFIG`, which also lives in bits 4:3.
    #[inline]
    pub const fn config_bits(self) -> u8 {
        (self as u8) << 3
    }
}

#[derive(Debug, Default)]
pub struct Config {
    pub lpf: DlpfCfg,
    pub gyro_range: GyroRange,
    pub accel_range: AccelRange,
}

pub struct MPU6050<I2C: embedded_hal_async::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    gyro_range: GyroRange,
    accel_range: AccelRange,
}

impl<I2C: embedded_hal_async::i2c::I2c> MPU6050<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            addr,
            i2c,
            gyro_range: GyroRange::Deg1000,
            accel_range: AccelRange::G2,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, PRIMARY_ADDRESS)
    }

    /// Validate the part, wake it and wait for the gyro to start up.
    ///
    /// `PWR_MGMT_1` is written as [`PWR_MGMT_1_CLKSEL_PLL_X_GYRO`] (leave sleep,
    /// PLL with X-gyro reference) rather than `0x00`, which would leave the part
    /// on its internal 8 MHz oscillator. `init` then waits
    /// [`WAKE_UP_DELAY_MS`] before returning, because the sensor registers read
    /// zero until the first conversion completes.
    pub async fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let who_am_i = self.read_reg(regs::WHO_AM_I).await?;
        if who_am_i != consts::DEV_ID_MPU6050
            && who_am_i != consts::DEV_ID_MPU6500
            && who_am_i != consts::DEV_ID_MPU9250
            && who_am_i != consts::DEV_ID_MPU9255
        {
            return Err(Error::InvalidDevice);
        }

        // Exit sleep mode and select the X-gyro PLL as the clock source.
        self.write_reg(regs::PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL_X_GYRO).await?;

        // LPF
        self.write_reg(regs::CONFIG, config.lpf as u8).await?;

        // gyro ADC scale
        self.write_reg(regs::GYRO_CONFIG, config.gyro_range.config_bits())
            .await?;

        // accel ADC scale
        self.write_reg(regs::ACCEL_CONFIG, config.accel_range.config_bits())
            .await?;

        self.gyro_range = config.gyro_range;
        self.accel_range = config.accel_range;

        // The sensor registers hold zero until the gyro has started up.
        delay.delay_ms(WAKE_UP_DELAY_MS).await;

        Ok(())
    }

    pub async fn read_raw_accel(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(regs::ACCEL_XOUT_H, &mut buf).await?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    pub async fn read_raw_gyro(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(regs::GYRO_XOUT_H, &mut buf).await?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Read accelerometer data in g
    pub async fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_accel().await?;
        let lsb_sensitivity = self.accel_range.lsb_sensitivity();
        Ok((
            x as f32 / lsb_sensitivity,
            y as f32 / lsb_sensitivity,
            z as f32 / lsb_sensitivity,
        ))
    }

    /// Read gyroscope data in degrees per second
    pub async fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_gyro().await?;
        let lsb_sensitivity = self.gyro_range.lsb_sensitivity();
        Ok((
            x as f32 / lsb_sensitivity,
            y as f32 / lsb_sensitivity,
            z as f32 / lsb_sensitivity,
        ))
    }

    /// Read temperature in degrees Celsius
    pub async fn read_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_regs(regs::TEMP_OUT_H, &mut buf).await?;

        let temp = i16::from_be_bytes([buf[0], buf[1]]);
        Ok((temp as f32) / 340.0 + 36.53)
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    pub async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf).await?;
        Ok(())
    }

    pub async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value]).await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gyro_range_lands_in_fs_sel() {
        // Regression test: these used to be written unshifted, so every range
        // except Deg250 silently left the chip on +/-250 deg/s.
        assert_eq!(GyroRange::Deg250.config_bits(), 0b0000_0000);
        assert_eq!(GyroRange::Deg500.config_bits(), 0b0000_1000);
        assert_eq!(GyroRange::Deg1000.config_bits(), 0b0001_0000);
        assert_eq!(GyroRange::Deg2000.config_bits(), 0b0001_1000);

        for range in [
            GyroRange::Deg250,
            GyroRange::Deg500,
            GyroRange::Deg1000,
            GyroRange::Deg2000,
        ] {
            let fs_sel = (range.config_bits() >> 3) & 0b11;
            assert_eq!(fs_sel, range as u8, "FS_SEL must encode the requested range");
        }
    }

    #[test]
    fn accel_range_lands_in_afs_sel() {
        assert_eq!(AccelRange::G2.config_bits(), 0b0000_0000);
        assert_eq!(AccelRange::G4.config_bits(), 0b0000_1000);
        assert_eq!(AccelRange::G8.config_bits(), 0b0001_0000);
        assert_eq!(AccelRange::G16.config_bits(), 0b0001_1000);

        for range in [AccelRange::G2, AccelRange::G4, AccelRange::G8, AccelRange::G16] {
            let afs_sel = (range.config_bits() >> 3) & 0b11;
            assert_eq!(afs_sel, range as u8, "AFS_SEL must encode the requested range");
        }
    }
}
