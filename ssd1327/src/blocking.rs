//! Blocking mode driver for SSD1327 OLED display
//!
//! # Examples
//!
//! This snippet shows how the driver is wired up in an application, using that
//! application's HAL, so it is not compiled as a doctest.
//!
//! ```ignore
//! use embedded_graphics::framebuffer::Framebuffer;
//!
//! let dc = Output::new(r.dc, Level::Low, Default::default());
//! let cs = Output::new(r.cs, Level::High, Default::default());
//!
//! let mut config = hal::spi::Config::default();
//! config.frequency = Hertz::mhz(20);
//! let spi = hal::spi::Spi::new_blocking_txonly(r.spi1, r.sclk, r.mosi, config);
//!
//! let spi_bus = embedded_hal_bus::util::AtomicCell::new(spi);
//! let spi_dev = embedded_hal_bus::spi::AtomicDevice::new(&spi_bus, cs, Delay).unwrap();
//!
//! let mut ssd1327 = edrv_ssd1327::blocking::SSD1327::new(spi_dev, dc);
//!
//! ssd1327.init().unwrap();
//! let mut fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { 128 * 128 / 2 }>::new();
//!
//! // Draw something on the framebuffer
//!
//! ssd1327.write_framebuffer(fb.data()).unwrap();
//! ```

use embedded_hal::digital::OutputPin;

use crate::{Error, HEIGHT, WIDTH, cmds};

/// SSD1327 driver, blocking mode
///
/// Framebuffer format:
///
/// ```
/// use embedded_graphics::framebuffer::{buffer_size, Framebuffer};
/// use embedded_graphics::pixelcolor::{raw::LittleEndian, Gray4};
///
/// let _fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { buffer_size::<Gray4>(128, 128) }>::new();
/// // or, equivalently, 4 bits per pixel:
/// let _fb = Framebuffer::<Gray4, _, LittleEndian, 128, 128, { 128 * 128 / 2 }>::new();
/// ```
pub struct SSD1327<SPI: embedded_hal::spi::SpiDevice, DC: OutputPin> {
    spi: SPI,
    dc: DC,
}

impl<SPI: embedded_hal::spi::SpiDevice, DC: OutputPin> SSD1327<SPI, DC> {
    pub fn new(spi: SPI, dc: DC) -> Self {
        Self { spi, dc }
    }

    pub fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write_command(&[cmds::SET_DISPLAY_OFF])?;
        // 0x3F
        self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, WIDTH / 8 * 4 - 1])?;
        // 0x7F
        self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, HEIGHT - 1])?;
        self.write_command(&[cmds::SET_CONTRAST_CURRENT, 0x80])?;

        // address remap
        self.write_command(&[cmds::SET_REMAP, 0x51])?;

        self.write_command(&[cmds::SET_DISPLAY_START_LINE, 0x00])?;
        self.write_command(&[cmds::SET_DISPLAY_OFFSET, 0x00])?;

        self.write_command(&[cmds::SET_MULTIPLEX_RATIO, 0x7F])?;
        self.write_command(&[cmds::SET_PHASE_LENGTH, 0x11])?; // gray scale tune

        // gamma setting
        // 0xb8: SET_GRAY_SCALE_TABLE, [u8; 15]
        // 0xb9: SET_DEFAULT_LINEAR_GRAY_SCALE_TABLE
        //self.send_cmd_data(0xb8, &[1,2,30,40,5,6,7,8,9,10,11,12,13,14,0b11111])?;
        self.write_command(&[cmds::SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE])?;

        self.write_command(&[cmds::SET_FRONT_CLOCK_DIVIDER, 0x00])?;
        self.write_command(&[cmds::FUNCTION_SELECTION_A, 0x01])?;
        self.write_command(&[cmds::SET_SECOND_PRECHARGE_PERIOD, 0x08])?;
        self.write_command(&[cmds::SET_VCOMH_VOLTAGE, 0x0f])?;
        self.write_command(&[cmds::SET_PRECHARGE_VOLTAGE, 0x08])?;
        self.write_command(&[cmds::FUNCTION_SELECTION_B, 0x62])?;
        self.write_command(&[cmds::SET_COMMAND_LOCK, 0x12])?;
        self.write_command(&[cmds::SET_DISPLAY_MODE])?; // display mode normal
        self.write_command(&[cmds::SET_DISPLAY_ON])?;

        self.write_command(&[cmds::DEACTIVATE_SCROLL])?;

        Ok(())
    }

    pub fn write_framebuffer(&mut self, fb: &[u8]) -> Result<(), Error<SPI::Error>> {
        // self.write_command(&[cmds::SET_COLUMN_ADDRESS, 0x00, 0x3F])?;
        // self.write_command(&[cmds::SET_ROW_ADDRESS, 0x00, 0x7F])?;
        self.write_data(fb)?;
        Ok(())
    }

    pub fn clear(&mut self, color_byte: u8) -> Result<(), Error<SPI::Error>> {
        let buf = [color_byte; 128 * 128 / 2];
        self.write_framebuffer(&buf)?;
        Ok(())
    }

    fn write_command(&mut self, cmd: &[u8]) -> Result<(), Error<SPI::Error>> {
        let _ = self.dc.set_low();
        self.spi.write(cmd)?;
        Ok(())
    }

    fn write_data(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        let _ = self.dc.set_high();
        self.spi.write(data)?;
        let _ = self.dc.set_low();
        Ok(())
    }
}
