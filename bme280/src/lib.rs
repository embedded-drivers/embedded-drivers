//! Driver for BME280 and BMP280.
//!
//! The only difference between the two sensors is that the BME280 also includes a humidity sensor.

#![no_std]

pub const ADDRESS: u8 = 0x76;

pub const CHIP_ID_BME280: u8 = 0x60;
pub const CHIP_ID_BMP280: u8 = 0x58;

pub mod blocking;

pub mod regs {
    pub const TEMP_XLSB: u8 = 0xFC;
    pub const TEMP_LSB: u8 = 0xFB;
    pub const TEMP_MSB: u8 = 0xFA;

    pub const PRESS_XLSB: u8 = 0xF9;
    pub const PRESS_LSB: u8 = 0xF8;
    pub const PRESS_MSB: u8 = 0xF7;

    pub const HUM_XLSB: u8 = 0xFD;
    pub const HUM_LSB: u8 = 0xFE;
    pub const HUM_MSB: u8 = 0xFD;

    pub const CHIP_ID: u8 = 0xD0;

    pub const CALIB_00: u8 = 0x88;

    pub const CONFIG: u8 = 0xF5;
    pub const CTRL_MEAS: u8 = 0xF4;
    pub const CTRL_HUM: u8 = 0xF2;

    pub const STATUS: u8 = 0xF3;
    pub const RESET: u8 = 0xE0;
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    CalibrationDataError,
    ConversionError,
    InvalidDevice,
    UnsupportedMeasurement,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Bus(error)
    }
}

#[derive(Debug)]
pub struct CalibrationData {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
    // BME280 only
    pub dig_h1: u8,
    pub dig_h2: i16,
    pub dig_h3: u8,
    pub dig_h4: i16,
    pub dig_h5: i16,
    pub dig_h6: i8,
}

impl CalibrationData {
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }

    fn from_raw(raw: &[u8]) -> Self {
        CalibrationData {
            dig_t1: u16::from_le_bytes([raw[0], raw[1]]),
            dig_t2: i16::from_le_bytes([raw[2], raw[3]]),
            dig_t3: i16::from_le_bytes([raw[4], raw[5]]),
            dig_p1: u16::from_le_bytes([raw[6], raw[7]]),
            dig_p2: i16::from_le_bytes([raw[8], raw[9]]),
            dig_p3: i16::from_le_bytes([raw[10], raw[11]]),
            dig_p4: i16::from_le_bytes([raw[12], raw[13]]),
            dig_p5: i16::from_le_bytes([raw[14], raw[15]]),
            dig_p6: i16::from_le_bytes([raw[16], raw[17]]),
            dig_p7: i16::from_le_bytes([raw[18], raw[19]]),
            dig_p8: i16::from_le_bytes([raw[20], raw[21]]),
            dig_p9: i16::from_le_bytes([raw[22], raw[23]]),
            // BME280 only
            dig_h1: raw[25],
            dig_h2: i16::from_le_bytes([raw[26], raw[27]]),
            dig_h3: raw[28],
            dig_h4: i16::from_le_bytes([raw[29], raw[30] & 0x0F]),
            dig_h5: i16::from_le_bytes([raw[30] >> 4, raw[31]]),
            dig_h6: raw[32] as i8,
        }
    }
}

#[derive(Debug)]
pub struct Measurements {
    pub temperature_centidegree: i32,
    pub pressure_centipascal: u32,
    pub humidity_centipercent: u32,
}
