//! Driver for HMC5883L.

#![no_std]

pub mod blocking;

pub const ADDRESS: u8 = 0x1E;

pub mod regs {
    pub const CONFIG_A: u8 = 0x00;
    pub const CONFIG_B: u8 = 0x01;
    pub const MODE: u8 = 0x02;
    pub const DATA_X_MSB: u8 = 0x03;
    pub const DATA_X_LSB: u8 = 0x04;
    pub const DATA_Z_MSB: u8 = 0x05;
    pub const DATA_Z_LSB: u8 = 0x06;
    pub const DATA_Y_MSB: u8 = 0x07;
    pub const DATA_Y_LSB: u8 = 0x08;
    pub const STATUS: u8 = 0x09;
    pub const IDENT_A: u8 = 0x0A;
    pub const IDENT_B: u8 = 0x0B;
    pub const IDENT_C: u8 = 0x0C;
}

/// Select number of samples averaged (1 to 8) per measurement output
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Samples {
    Samples1 = 0b00, // 1 sample (Default)
    Samples2 = 0b01, // 2 samples
    Samples4 = 0b10, // 4 samples
    Samples8 = 0b11, // 8 samples
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Rate {
    Hz0_75 = 0b000,
    Hz1_5 = 0b001,
    Hz3 = 0b010,
    Hz7_5 = 0b011,
    Hz15 = 0b100,
    Hz30 = 0b101,
    Hz75 = 0b110,
}

/// Gain settings for the HMC5883L magnetometer.
///
/// This enum represents the gain settings for the HMC5883L magnetometer.
/// Each variant corresponds to a specific gain value, which affects the
/// sensor's field range, gain in LSB/Gauss, and digital resolution in mG/LSB.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Gain {
    /// ±0.88 Ga, 1370 LSB/Gauss, 0.73 mG/LSB
    Gain1370 = 0b000,
    /// ±1.3 Ga, 1090 LSB/Gauss, 0.92 mG/LSB (default)
    Gain1090 = 0b001,
    /// ±1.9 Ga, 820 LSB/Gauss, 1.22 mG/LSB
    Gain820 = 0b010,
    /// ±2.5 Ga, 660 LSB/Gauss, 1.52 mG/LSB
    Gain660 = 0b011,
    /// ±4.0 Ga, 440 LSB/Gauss, 2.27 mG/LSB
    Gain440 = 0b100,
    /// ±4.7 Ga, 390 LSB/Gauss, 2.56 mG/LSB
    Gain390 = 0b101,
    /// ±5.6 Ga, 330 LSB/Gauss, 3.03 mG/LSB
    Gain330 = 0b110,
    /// ±8.1 Ga, 230 LSB/Gauss, 4.35 mG/LSB
    Gain230 = 0b111,
}

impl Gain {
    fn resolution(&self) -> f32 {
        match self {
            Gain::Gain1370 => 0.73,
            Gain::Gain1090 => 0.92,
            Gain::Gain820 => 1.22,
            Gain::Gain660 => 1.52,
            Gain::Gain440 => 2.27,
            Gain::Gain390 => 2.56,
            Gain::Gain330 => 3.03,
            Gain::Gain230 => 4.35,
        }
    }
}

/// Configuration settings for the HMC5883L magnetometer.
///
/// This struct holds the data rate and gain settings for the magnetometer.
/// The `data_rate` field determines the data output rate of the magnetometer,
/// and the `gain` field determines the gain setting of the magnetometer.
#[derive(Clone, Copy)]
pub struct Config {
    pub data_rate: Rate,
    pub gain: Gain,
    pub samples: Samples,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            data_rate: Rate::Hz15,
            gain: Gain::Gain1090,
            samples: Samples::Samples8,
        }
    }
}

#[derive(Debug)]
pub enum Error<IE> {
    Bus(IE),
    InvalidDevice,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

pub struct HMC5883L<I2C: embedded_hal_async::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    gain: Gain,
}

impl<I2C: embedded_hal_async::i2c::I2c> HMC5883L<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            gain: Gain::Gain1090,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    pub async fn init(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        let id_a = self.read_reg(regs::IDENT_A).await?;
        let id_b = self.read_reg(regs::IDENT_B).await?;
        let id_c = self.read_reg(regs::IDENT_C).await?;

        if id_a != 0x48 || id_b != 0x34 || id_c != 0x33 {
            return Err(Error::InvalidDevice);
        }

        let mut cra = 0; // Initialize CRA with all bits set to 0
        cra |= (config.samples as u8) << 5; // Set MA (CRA6 to CRA5) based on config.samples
        cra |= (config.data_rate as u8) << 2; // Set DO (CRA4 to CRA2) based on config.data_rate
        self.write_reg(regs::CONFIG_A, cra).await?;

        let crb = (config.gain as u8) << 5; // Set GN (CRB7 to CRB5) based on config.gain
        self.write_reg(regs::CONFIG_B, crb).await?;

        self.write_reg(regs::MODE, 0x00).await?; // Continuous-measurement mode

        self.gain = config.gain;

        Ok(())
    }

    pub async fn read_raw_measurement(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from DATA_X_MSB
        self.i2c.write_read(self.addr, &[regs::DATA_X_MSB], &mut buf).await?;
        self.i2c.write(self.addr, &[regs::DATA_X_MSB]).await?;

        let x = ((buf[0] as i16) << 8) | buf[1] as i16;
        let y = ((buf[4] as i16) << 8) | buf[5] as i16;
        let z = ((buf[2] as i16) << 8) | buf[3] as i16;

        Ok((x, y, z))
    }

    pub async fn read_measurement(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_measurement().await?;

        let resolution = self.gain.resolution();

        let x_norm = x as f32 * resolution;
        let y_norm = y as f32 * resolution;
        let z_norm = z as f32 * resolution;

        Ok((x_norm, y_norm, z_norm))
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    // Add this method to write to registers
    pub async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.addr, &[reg, value]).await?;
        Ok(())
    }
}
