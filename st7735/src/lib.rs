//! Driver for ST7735.

#![no_std]

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::*;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

pub mod blocking;

pub mod cmds {
    pub const NOP: u8 = 0x00;
    pub const SWRESET: u8 = 0x01;
    pub const RDDID: u8 = 0x04;
    pub const RDDST: u8 = 0x09;
    pub const SLPIN: u8 = 0x10;
    pub const SLPOUT: u8 = 0x11;
    pub const PTLON: u8 = 0x12;
    pub const NORON: u8 = 0x13;
    pub const INVOFF: u8 = 0x20;
    pub const INVON: u8 = 0x21;
    pub const DISPOFF: u8 = 0x28;
    pub const DISPON: u8 = 0x29;
    pub const CASET: u8 = 0x2A;
    pub const RASET: u8 = 0x2B;
    pub const RAMWR: u8 = 0x2C;
    pub const RAMRD: u8 = 0x2E;
    pub const PTLAR: u8 = 0x30;
    pub const COLMOD: u8 = 0x3A;
    pub const MADCTL: u8 = 0x36;
    pub const FRMCTR1: u8 = 0xB1;
    pub const FRMCTR2: u8 = 0xB2;
    pub const FRMCTR3: u8 = 0xB3;
    pub const INVCTR: u8 = 0xB4;
    pub const DISSET5: u8 = 0xB6;
    pub const PWCTR1: u8 = 0xC0;
    pub const PWCTR2: u8 = 0xC1;
    pub const PWCTR3: u8 = 0xC2;
    pub const PWCTR4: u8 = 0xC3;
    pub const PWCTR5: u8 = 0xC4;
    pub const VMCTR1: u8 = 0xC5;
    pub const RDID1: u8 = 0xDA;
    pub const RDID2: u8 = 0xDB;
    pub const RDID3: u8 = 0xDC;
    pub const RDID4: u8 = 0xDD;
    pub const PWCTR6: u8 = 0xFC;
    pub const GMCTRP1: u8 = 0xE0;
    pub const GMCTRN1: u8 = 0xE1;
}

pub trait DisplaySpec {
    const WIDTH: u16;
    const HEIGHT: u16;
    const OFFSETX: u16 = 0;
    const OFFSETY: u16 = 0;

    const INVERTED: bool = false;
}

/// Common 0.96" 160x80 display
pub struct Display160x80Type1;
impl DisplaySpec for Display160x80Type1 {
    const WIDTH: u16 = 160;
    const HEIGHT: u16 = 80;
    const OFFSETX: u16 = 0;
    const OFFSETY: u16 = 24;
}

/// Common 0.96" 160x80 display, type 2
pub struct Display160x80Type2;
impl DisplaySpec for Display160x80Type2 {
    const WIDTH: u16 = 160;
    const HEIGHT: u16 = 80;
    const OFFSETX: u16 = 1;
    const OFFSETY: u16 = 26;
    const INVERTED: bool = true;
}

#[derive(Debug)]
pub enum Error<BusError> {
    Bus(BusError),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

pub struct ST7735<SPEC, SPI, DC> {
    spi: SPI,
    dc: DC,
    _marker: core::marker::PhantomData<SPEC>,
}

impl<SPEC: DisplaySpec, SPI: embedded_hal_async::spi::SpiDevice, DC: OutputPin> ST7735<SPEC, SPI, DC> {
    pub fn new(spi: SPI, dc: DC) -> Self {
        Self {
            spi,
            dc,
            _marker: core::marker::PhantomData,
        }
    }

    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<SPI::Error>> {
        self.send_command(cmds::SWRESET).await?;
        delay.delay_ms(20);
        self.send_command(cmds::SLPOUT).await?;
        delay.delay_ms(200);

        self.send_command_data(cmds::FRMCTR1, &[0x01, 0x2C, 0x2D]).await?;
        self.send_command_data(cmds::FRMCTR2, &[0x01, 0x2C, 0x2D]).await?;
        self.send_command_data(cmds::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])
            .await?;
        self.send_command_data(cmds::INVCTR, &[0x07]).await?;
        self.send_command_data(cmds::PWCTR1, &[0xA2, 0x02, 0x84]).await?;
        self.send_command_data(cmds::PWCTR2, &[0xC5]).await?;
        self.send_command_data(cmds::PWCTR3, &[0x0A, 0x00]).await?;
        self.send_command_data(cmds::PWCTR4, &[0x8A, 0x2A]).await?;
        self.send_command_data(cmds::PWCTR5, &[0x8A, 0xEE]).await?;
        self.send_command_data(cmds::VMCTR1, &[0x0E]).await?;

        if SPEC::INVERTED {
            self.send_command(cmds::INVON).await?;
        } else {
            self.send_command(cmds::INVOFF).await?;
        }

        // BITS:
        // MY, MX, MV, ML, RGB, MH, D1, D0
        self.send_command_data(cmds::MADCTL, &[0b0110_10_00]).await?;

        self.send_command_data(cmds::COLMOD, &[0x05]).await?; // 16-bit/pixel
        self.send_command(cmds::DISPON).await?;

        delay.delay_ms(100);

        Ok(())
    }

    #[inline]
    async fn set_update_window(&mut self, x: u16, y: u16, w: u16, h: u16) -> Result<(), Error<SPI::Error>> {
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
        )
        .await?;

        self.send_command_data(
            cmds::RASET,
            &[
                (oy >> 8) as u8,
                (oy & 0xFF) as u8,
                ((oy + h - 1) >> 8) as u8,
                ((oy + h - 1) & 0xFF) as u8,
            ],
        )
        .await?;
        Ok(())
    }

    pub async fn write_framebuffer(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.set_update_window(0, 0, SPEC::WIDTH, SPEC::HEIGHT).await?;
        self.send_command_data(cmds::RAMWR, data).await?;
        Ok(())
    }

    pub async fn write_window_framebuffer(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        self.set_update_window(x, y, w, h).await?;
        self.send_command_data(cmds::RAMWR, data).await?;
        Ok(())
    }

    pub async fn write_pixel(&mut self, x: u16, y: u16, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.set_update_window(x, y, 1, 1).await?;

        self.send_command_data(cmds::RAMWR, data).await?;
        Ok(())
    }

    async fn send_command(&mut self, cmd: u8) -> Result<(), Error<SPI::Error>> {
        let _ = self.dc.set_low(); // ignore any io errors
        self.spi.write(&[cmd]).await?;
        Ok(())
    }

    async fn send_data(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        let _ = self.dc.set_high(); // ignore any io errors
        self.spi.write(data).await?;
        Ok(())
    }

    async fn send_command_data(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.send_command(cmd).await?;
        self.send_data(data).await?;
        Ok(())
    }

    pub async fn clear(&mut self, color: Rgb565) -> Result<(), Error<SPI::Error>> {
        self.set_update_window(0, 0, SPEC::WIDTH, SPEC::HEIGHT).await?;

        self.send_command(cmds::RAMWR).await?;
        for _ in 0..(SPEC::WIDTH * SPEC::HEIGHT) {
            self.send_data(color.to_be_bytes().as_ref()).await?;
        }
        Ok(())
    }
}
