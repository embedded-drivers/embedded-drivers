//! Driver for L3G4200D.
#![cfg_attr(not(test), no_std)]

use embedded_hal_async::delay::DelayNs;

pub mod blocking;

pub mod regs {
    pub const WHO_AM_I: u8 = 0x0F;

    pub const CTRL_REG1: u8 = 0x20;
    pub const CTRL_REG2: u8 = 0x21;
    pub const CTRL_REG3: u8 = 0x22;
    pub const CTRL_REG4: u8 = 0x23;
    pub const CTRL_REG5: u8 = 0x24;
    pub const REFERENCE: u8 = 0x25;
    pub const OUT_TEMP: u8 = 0x26;
    pub const STATUS_REG: u8 = 0x27;

    pub const OUT_X_L: u8 = 0x28;
    pub const OUT_X_H: u8 = 0x29;
    pub const OUT_Y_L: u8 = 0x2A;
    pub const OUT_Y_H: u8 = 0x2B;
    pub const OUT_Z_L: u8 = 0x2C;
    pub const OUT_Z_H: u8 = 0x2D;

    /// `OUT_X_L` with the address auto-increment bit set, for the six-byte
    /// X/Y/Z burst read.
    pub const OUT_X_L_AUTO_INCREMENT: u8 = OUT_X_L | 0x80;

    pub const FIFO_CTRL_REG: u8 = 0x2E;
    pub const FIFO_SRC_REG: u8 = 0x2F;

    pub const INT1_CFG: u8 = 0x30;
    pub const INT1_SRC: u8 = 0x31;
    pub const INT1_THS_XH: u8 = 0x32;
    pub const INT1_THS_XL: u8 = 0x33;
    pub const INT1_THS_YH: u8 = 0x34;
    pub const INT1_THS_YL: u8 = 0x35;
    pub const INT1_THS_ZH: u8 = 0x36;
    pub const INT1_THS_ZL: u8 = 0x37;
    pub const INT1_DURATION: u8 = 0x38;

    /// `STATUS_REG` bit 3: a new X, Y, Z sample is available.
    ///
    /// L3G4200D datasheet (Doc ID 17116 Rev 3), Table 41/42: `ZYXDA`,
    /// "X, Y, Z-axis new data available ... 1: a new set of data is available".
    pub const STATUS_ZYXDA: u8 = 0x08;
}

/// `CTRL_REG1` value with PD set and all three axes enabled.
///
/// `PD` is an **active-low power-down** bit, not a power-up bit: the L3G4200D
/// datasheet (Table 21) says "PD: Power down mode enable. Default value: 0
/// (0: power down mode, 1: normal mode or sleep mode)". The reset value of
/// `CTRL_REG1` is `0x07`, i.e. the part powers up powered down with all axes
/// enabled. Clearing this bit would power the gyro down and every reading would
/// be zero, so the value must stay `0b0000_1111`.
pub const CTRL_REG1_NORMAL_MODE: u8 = 0x0F;

/// How long `init` waits for the first sample, in milliseconds.
///
/// The datasheet does not state a turn-on time (it only notes that sleep mode
/// turns on faster than power-down mode), so this is bounded by the output data
/// rate instead: the slowest rate this driver offers is 100 Hz, i.e. a 10 ms
/// sample period, making 100 ms ten sample periods.
pub(crate) const DATA_READY_TIMEOUT_MS: u32 = 100;

pub const ADDRESS: u8 = 0x69;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Scale {
    Dps2000 = 0b10,
    Dps500 = 0b01,
    Dps250 = 0b00,
}

impl Scale {
    /// Sensitivity in **quarter milli-degrees per second per LSB**.
    ///
    /// L3G4200D datasheet (Doc ID 17116 Rev 3), Table 3 gives 8.75 / 17.5 / 70
    /// mdps per digit. None of those are whole numbers, so the exact integer
    /// conversion works in quarters (35 / 70 / 280) and divides once at the end.
    /// This is the single source of truth for both conversions below.
    pub const fn mdps_per_digit_x4(self) -> i32 {
        match self {
            Scale::Dps250 => 35,
            Scale::Dps500 => 70,
            Scale::Dps2000 => 280,
        }
    }

    /// Sensitivity in degrees per second per LSB, derived from the integer form.
    pub fn dps_per_digit(self) -> f32 {
        self.mdps_per_digit_x4() as f32 / 4000.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DataRate {
    Hz800Bw110 = 0b1111,
    Hz800Bw50 = 0b1110,
    Hz800Bw35 = 0b1101,
    Hz800Bw30 = 0b1100,
    Hz400Bw110 = 0b1011,
    Hz400Bw50 = 0b1010,
    Hz400Bw25 = 0b1001,
    Hz400Bw20 = 0b1000,
    Hz200Bw70 = 0b0111,
    Hz200Bw50 = 0b0110,
    Hz200Bw25 = 0b0101,
    Hz200Bw12_5 = 0b0100,
    Hz100Bw25 = 0b0001,
    Hz100Bw12_5 = 0b0000,
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    InvalidDevice,
    /// The device did not report a new sample within the driver's polling
    /// limit. `init` waits for the first sample and returns this instead of
    /// handing a caller data that was never measured.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

/// Configuration options for the L3G4200D
pub struct Config {
    /// Full scale selection
    pub scale: Scale,
    /// Output data rate & bandwidth
    pub data_rate: DataRate,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            scale: Scale::Dps2000,
            data_rate: DataRate::Hz400Bw50,
        }
    }
}

pub struct L3G4200D<I2C: embedded_hal_async::i2c::I2c> {
    i2c: I2C,
    addr: u8,
    /// The configured full scale. Stored rather than a precomputed float so the
    /// integer conversion ([`Self::read_gyro_mdps`]) stays exact.
    scale: Scale,
}

impl<I2C: embedded_hal_async::i2c::I2c> L3G4200D<I2C> {
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            // CTRL_REG4 resets to FS = 00, which is +/-250 dps, so this is what
            // the part is actually doing before `init` reprograms it.
            scale: Scale::Dps250,
        }
    }

    pub fn new_primary(i2c: I2C) -> Self {
        Self::new(i2c, ADDRESS)
    }

    /// Configure the device and wait for the first sample.
    ///
    /// The output registers reset to zero and only hold a measurement once the
    /// gyro has produced one, so a read issued straight after `init` would
    /// return `(0, 0, 0)`. `init` therefore polls `STATUS_REG.ZYXDA` (datasheet
    /// Table 41/42) before returning. Once running, the device refreshes the
    /// registers at the configured output data rate, so later reads need no
    /// additional wait.
    pub async fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        if self.read_reg(regs::WHO_AM_I).await? != 0xD3 {
            return Err(Error::InvalidDevice);
        }

        // Enable all axis and setup normal mode + Output Data Range & Bandwidth
        let mut reg1 = CTRL_REG1_NORMAL_MODE; // PD = 1 (normal mode), all axes enabled
        reg1 |= (config.data_rate as u8) << 4; // Set output data rate & bandwidth
        self.write_reg(regs::CTRL_REG1, reg1).await?;

        // Disable high pass filter
        self.write_reg(regs::CTRL_REG2, 0x00).await?;

        // Generate data ready interrupt on INT2
        self.write_reg(regs::CTRL_REG3, 0x08).await?;

        // Set full scale selection in continuous mode
        self.write_reg(regs::CTRL_REG4, (config.scale as u8) << 4).await?;

        self.scale = config.scale;

        // Boot in normal mode, disable FIFO, HPF disabled
        self.write_reg(regs::CTRL_REG5, 0x00).await?;

        self.wait_for_data_ready(&mut delay).await?;

        Ok(())
    }

    /// Poll `STATUS_REG.ZYXDA` until the first sample lands, or give up.
    async fn wait_for_data_ready(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<I2C::Error>> {
        for _ in 0..=DATA_READY_TIMEOUT_MS {
            if self.read_reg(regs::STATUS_REG).await? & regs::STATUS_ZYXDA != 0 {
                return Ok(());
            }
            delay.delay_ms(1).await;
        }

        Err(Error::Timeout)
    }

    /// The raw output registers, in **LSBs**, unconverted.
    ///
    /// One LSB is 8.75, 17.5 or 70 milli-degrees per second depending on the
    /// configured [`Scale`]. Prefer [`Self::read_gyro_mdps`] or
    /// [`Self::read_gyro`] unless you specifically want the register values.
    pub async fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from OUT_X_L register (0x28 | 0x80 for auto-increment)
        self.i2c
            .write_read(self.addr, &[regs::OUT_X_L_AUTO_INCREMENT], &mut buf)
            .await?;

        // Combine high and low bytes into 16-bit integers
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Angular rate in **degrees per second**.
    ///
    /// This is the convenient form, and it matches `edrv-adxl345`'s `read_accel`,
    /// which also returns `f32`. On a soft-float target - the examples in this
    /// workspace run on `riscv32imc`, which has no FPU - the arithmetic is
    /// emulated, but that is cheap next to *formatting* a float, which is what
    /// actually costs about 12.5 KB of flash.
    ///
    /// Use [`Self::read_gyro_mdps`] to stay off floating point entirely.
    pub async fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw().await?;
        let sensitivity = self.scale.dps_per_digit();

        Ok((x as f32 * sensitivity, y as f32 * sensitivity, z as f32 * sensitivity))
    }

    /// Angular rate in **milli-degrees per second**, with no floating point.
    ///
    /// The datasheet sensitivities (8.75 / 17.5 / 70 mdps per LSB) are not whole
    /// numbers, so the conversion is done in quarters and divided once:
    /// `raw * (mdps_per_digit * 4) / 4`. That single division truncates toward
    /// zero, leaving under **1 mdps** of error - three orders of magnitude finer
    /// than the whole-degree resolution this replaced, and far below the part's
    /// own noise.
    ///
    /// Full scale is 2 000 000 mdps at +/-2000 dps, so the result fits `i32` with
    /// room to spare.
    pub async fn read_gyro_mdps(&mut self) -> Result<(i32, i32, i32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw().await?;
        let factor = self.scale.mdps_per_digit_x4();

        Ok((
            (x as i32 * factor) / 4,
            (y as i32 * factor) / 4,
            (z as i32 * factor) / 4,
        ))
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0];
        self.i2c.write_read(self.addr, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    // Add this new method to write to registers
    pub async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value]).await?;
        Ok(())
    }
}
