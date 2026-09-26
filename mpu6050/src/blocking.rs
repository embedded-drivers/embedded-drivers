use embedded_hal::delay::DelayNs;

use crate::{
    AccelRange, Config, Error, GyroRange, PRIMARY_ADDRESS, PWR_MGMT_1_CLKSEL_PLL_X_GYRO, WAKE_UP_DELAY_MS, consts, regs,
};

pub struct MPU6050<I2C: embedded_hal::i2c::I2c> {
    addr: u8,
    i2c: I2C,
    gyro_range: GyroRange,
    accel_range: AccelRange,
}

impl<I2C: embedded_hal::i2c::I2c> MPU6050<I2C> {
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
    pub fn init(&mut self, config: Config, mut delay: impl DelayNs) -> Result<(), Error<I2C::Error>> {
        let who_am_i = self.read_reg(regs::WHO_AM_I)?;
        if who_am_i != consts::DEV_ID_MPU6050
            && who_am_i != consts::DEV_ID_MPU6500
            && who_am_i != consts::DEV_ID_MPU9250
            && who_am_i != consts::DEV_ID_MPU9255
        {
            return Err(Error::InvalidDevice);
        }

        // Exit sleep mode and select the X-gyro PLL as the clock source.
        self.write_reg(regs::PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL_X_GYRO)?;

        // LPF
        self.write_reg(regs::CONFIG, config.lpf as u8)?;

        // gyro ADC scale
        self.write_reg(regs::GYRO_CONFIG, config.gyro_range.config_bits())?;

        // accel ADC scale
        self.write_reg(regs::ACCEL_CONFIG, config.accel_range.config_bits())?;

        self.gyro_range = config.gyro_range;
        self.accel_range = config.accel_range;

        // The sensor registers hold zero until the gyro has started up.
        delay.delay_ms(WAKE_UP_DELAY_MS);

        Ok(())
    }

    pub fn read_raw_accel(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(regs::ACCEL_XOUT_H, &mut buf)?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    pub fn read_raw_gyro(&mut self) -> Result<(i16, i16, i16), Error<I2C::Error>> {
        let mut buf = [0u8; 6];
        self.read_regs(regs::GYRO_XOUT_H, &mut buf)?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);

        Ok((x, y, z))
    }

    /// Read accelerometer data in g
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_accel()?;
        let lsb_sensitivity = self.accel_range.lsb_sensitivity();
        Ok((
            x as f32 / lsb_sensitivity,
            y as f32 / lsb_sensitivity,
            z as f32 / lsb_sensitivity,
        ))
    }

    /// Read gyroscope data in degrees per second
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let (x, y, z) = self.read_raw_gyro()?;
        let lsb_sensitivity = self.gyro_range.lsb_sensitivity();
        Ok((
            x as f32 / lsb_sensitivity,
            y as f32 / lsb_sensitivity,
            z as f32 / lsb_sensitivity,
        ))
    }

    /// Read temperature in degrees Celsius
    pub fn read_temperature(&mut self) -> Result<f32, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.read_regs(regs::TEMP_OUT_H, &mut buf)?;

        let temp = i16::from_be_bytes([buf[0], buf[1]]);
        Ok((temp as f32) / 340.0 + 36.53)
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    pub fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.addr, &[reg], buf)?;
        Ok(())
    }

    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.addr, &[reg, value])?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use core::cell::{Cell, RefCell};
    use std::rc::Rc;

    use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};

    use super::*;

    #[derive(Debug)]
    struct MockError;

    impl embedded_hal::i2c::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    /// A shared fake clock in milliseconds.
    #[derive(Clone, Default)]
    struct Clock(Rc<Cell<u32>>);

    impl Clock {
        fn now(&self) -> u32 {
            self.0.get()
        }

        fn advance_ms(&self, ms: u32) {
            self.0.set(self.0.get() + ms);
        }
    }

    struct FakeDelay(Clock);

    impl DelayNs for FakeDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.0.advance_ms(ns.div_ceil(1_000_000));
        }
    }

    /// Big-endian X/Y/Z samples: 0x0100, 0x0200, 0x0300.
    const REAL_SAMPLE: [u8; 6] = [0x01, 0x00, 0x02, 0x00, 0x03, 0x00];

    struct Shared {
        clock: Clock,
        /// When the part was taken out of sleep.
        awake_since: Cell<Option<u32>>,
        /// Last value written to `PWR_MGMT_1`.
        pwr_mgmt_1: Cell<u8>,
        /// How long the gyro needs to start up.
        start_up_ms: u32,
        writes: RefCell<Vec<(u8, u8)>>,
    }

    impl Shared {
        fn new(clock: Clock, start_up_ms: u32) -> Rc<Self> {
            Rc::new(Self {
                clock,
                awake_since: Cell::new(None),
                pwr_mgmt_1: Cell::new(0),
                start_up_ms,
                writes: RefCell::new(Vec::new()),
            })
        }

        fn data_ready(&self) -> bool {
            self.awake_since
                .get()
                .is_some_and(|since| self.clock.now().saturating_sub(since) >= self.start_up_ms)
        }
    }

    /// An MPU6050 whose sensor registers read zero until the gyro has started.
    struct DeviceModel(Rc<Shared>);

    impl ErrorType for DeviceModel {
        type Error = MockError;
    }

    impl I2c for DeviceModel {
        fn read(&mut self, _address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
            read.fill(0);
            Ok(())
        }

        fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
            if let [reg, value] = write {
                self.0.writes.borrow_mut().push((*reg, *value));
                if *reg == regs::PWR_MGMT_1 {
                    self.0.pwr_mgmt_1.set(*value);
                    if value & 0x40 == 0 {
                        self.0.awake_since.set(Some(self.0.clock.now()));
                    } else {
                        self.0.awake_since.set(None);
                    }
                }
            }
            Ok(())
        }

        fn write_read(&mut self, _address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            let reg = write[0];
            if reg == regs::ACCEL_XOUT_H || reg == regs::GYRO_XOUT_H {
                read.fill(0);
                if self.0.data_ready() {
                    read.copy_from_slice(&REAL_SAMPLE);
                }
            } else if reg == regs::WHO_AM_I {
                read.fill(0);
                read[0] = consts::DEV_ID_MPU6050;
            } else if reg == regs::TEMP_OUT_H {
                read.fill(0);
                if self.0.data_ready() {
                    read[0] = 0x01;
                    read[1] = 0x00;
                }
            } else {
                read.fill(0);
            }
            Ok(())
        }

        fn transaction(&mut self, _address: u8, _operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
            unimplemented!("the driver does not use transactions")
        }
    }

    /// `init` must leave sleep with a stable clock source and wait for the gyro
    /// to start before returning.
    ///
    /// Before this fix `init` wrote `PWR_MGMT_1 = 0x00` and returned immediately:
    /// the part was awake but on its internal 8 MHz oscillator, and the first
    /// read of the sensor registers returned zeros.
    #[test]
    fn init_waits_for_the_gyro_to_start() {
        let clock = Clock::default();
        let shared = Shared::new(clock.clone(), WAKE_UP_DELAY_MS);
        let mut sensor = MPU6050::new(DeviceModel(shared.clone()), PRIMARY_ADDRESS);

        sensor
            .init(Config::default(), FakeDelay(clock.clone()))
            .expect("init should succeed");

        assert_eq!(
            shared.pwr_mgmt_1.get(),
            PWR_MGMT_1_CLKSEL_PLL_X_GYRO,
            "PWR_MGMT_1 must clear SLEEP and select the X-gyro PLL, not the internal oscillator"
        );
        assert_eq!(shared.pwr_mgmt_1.get() & 0x40, 0, "SLEEP must be cleared");
        assert_eq!(shared.pwr_mgmt_1.get() & 0x07, 1, "CLKSEL must be 1 (PLL with X gyro)");

        assert!(
            shared.clock.now() >= WAKE_UP_DELAY_MS,
            "init returned after only {} ms",
            shared.clock.now()
        );

        let (gx, gy, gz) = sensor.read_raw_gyro().expect("read should succeed");
        assert_eq!(
            (gx, gy, gz),
            (0x0100, 0x0200, 0x0300),
            "read_raw_gyro returned the pre-conversion zeros"
        );
        let (ax, ay, az) = sensor.read_raw_accel().expect("read should succeed");
        assert_eq!((ax, ay, az), (0x0100, 0x0200, 0x0300));
    }
}
