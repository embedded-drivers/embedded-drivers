use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::*;
use embedded_graphics_core::primitives::Rectangle;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

use super::{cmds, DisplaySpec, Error};

pub struct ST7735<SPEC, SPI, DC> {
    spi: SPI,
    dc: DC,
    _marker: core::marker::PhantomData<SPEC>,
}

impl<SPEC: DisplaySpec, SPI: embedded_hal::spi::SpiDevice, DC: OutputPin> ST7735<SPEC, SPI, DC> {
    pub fn new(spi: SPI, dc: DC) -> Self {
        Self {
            spi,
            dc,
            _marker: core::marker::PhantomData,
        }
    }

    pub fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<SPI::Error>> {
        self.send_command(cmds::SWRESET)?;

        delay.delay_ms(20);
        self.send_command(cmds::SLPOUT)?;
        delay.delay_ms(200);

        self.send_command_data(cmds::FRMCTR1, &[0x01, 0x2C, 0x2D])?;
        self.send_command_data(cmds::FRMCTR2, &[0x01, 0x2C, 0x2D])?;
        self.send_command_data(cmds::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])?;
        self.send_command_data(cmds::INVCTR, &[0x07])?;
        self.send_command_data(cmds::PWCTR1, &[0xA2, 0x02, 0x84])?;
        self.send_command_data(cmds::PWCTR2, &[0xC5])?;
        self.send_command_data(cmds::PWCTR3, &[0x0A, 0x00])?;
        self.send_command_data(cmds::PWCTR4, &[0x8A, 0x2A])?;
        self.send_command_data(cmds::PWCTR5, &[0x8A, 0xEE])?;
        self.send_command_data(cmds::VMCTR1, &[0x0E])?;

        if SPEC::INVERTED {
            self.send_command(cmds::INVON)?;
        } else {
            self.send_command(cmds::INVOFF)?;
        }

        // BITS:
        // MY, MX, MV, ML, RGB, MH, _D1, _D0
        self.send_command_data(cmds::MADCTL, &[0b0110_10_00])?;

        self.send_command_data(cmds::COLMOD, &[0x05])?; // 16-bit/pixel
        self.send_command(cmds::DISPON)?;

        delay.delay_ms(100);

        Ok(())
    }

    #[inline]
    fn set_update_window(&mut self, x: u16, y: u16, w: u16, h: u16) -> Result<(), Error<SPI::Error>> {
        let ox = SPEC::OFFSETX + x;
        let oy = SPEC::OFFSETY + y;

        self.send_command_data(
            cmds::CASET,
            &[
                (ox >> 8) as u8,
                (ox & 0xFF) as u8,
                ((ox + w - 1) >> 8) as u8,
                ((ox + w - 1) & 0xFF) as u8,
            ],
        )?;

        self.send_command_data(
            cmds::RASET,
            &[
                (oy >> 8) as u8,
                (oy & 0xFF) as u8,
                ((oy + h - 1) >> 8) as u8,
                ((oy + h - 1) & 0xFF) as u8,
            ],
        )?;
        Ok(())
    }

    pub fn write_framebuffer(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.set_update_window(0, 0, SPEC::WIDTH, SPEC::HEIGHT)?;
        self.send_command_data(cmds::RAMWR, data)?;
        Ok(())
    }

    pub fn write_pixel(&mut self, x: u16, y: u16, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.set_update_window(x, y, 1, 1)?;
        self.send_command_data(cmds::RAMWR, data)?;
        Ok(())
    }

    fn send_command(&mut self, cmd: u8) -> Result<(), Error<SPI::Error>> {
        let _ = self.dc.set_low();
        self.spi.write(&[cmd])?;
        Ok(())
    }

    fn send_data(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        let _ = self.dc.set_high();
        self.spi.write(data)?;
        Ok(())
    }

    fn send_command_data(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.send_command(cmd)?;
        self.send_data(data)?;
        Ok(())
    }
}

impl<SPEC: DisplaySpec, SPI: embedded_hal::spi::SpiDevice, DC: OutputPin> OriginDimensions for ST7735<SPEC, SPI, DC> {
    fn size(&self) -> Size {
        Size::new(SPEC::WIDTH as _, SPEC::HEIGHT as _)
    }
}

impl<SPEC: DisplaySpec, SPI: embedded_hal::spi::SpiDevice, DC: OutputPin> DrawTarget for ST7735<SPEC, SPI, DC> {
    type Color = Rgb565;
    type Error = Error<SPI::Error>;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let x = pixel.0.x as u16;
            let y = pixel.0.y as u16;
            let color = pixel.1;

            self.write_pixel(x, y, color.to_be_bytes().as_ref())?;
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.set_update_window(0, 0, SPEC::WIDTH, SPEC::HEIGHT)?;

        self.send_command(cmds::RAMWR)?;
        for _ in 0..(SPEC::WIDTH * SPEC::HEIGHT) {
            self.send_data(color.to_be_bytes().as_ref())?;
        }
        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.set_update_window(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
        )?;

        self.send_command(cmds::RAMWR)?;
        for color in colors {
            self.send_data(color.to_be_bytes().as_ref())?;
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.set_update_window(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
        )?;

        self.send_command(cmds::RAMWR)?;
        for _ in 0..(area.size.width * area.size.height) {
            self.send_data(color.to_be_bytes().as_ref())?;
        }
        Ok(())
    }
}
