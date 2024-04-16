#![no_std]

use embedded_graphics_core::{draw_target::DrawTarget, geometry::Dimensions};
use embedded_hal::spi::Operation;

#[repr(u8)]
pub enum Commands {
    DCramDataWrite = 0x20,
    DGramDataClear = 0x10,
    CGramDataWrite = 0x40,
    SetDisplayTiming = 0xe0,
    SetDimmingData = 0xe4,
    SetDisplayOn = 0xe8,
    SetDisplayOff = 0xea,
    SetStandByMode = 0xec,
}
impl Into<u8> for Commands {
    fn into(self) -> u8 {
        self as u8
    }
}

const fn reverse_bits(x: u8) -> u8 {
    ((x & 0b00000001) << 7)
        | ((x & 0b00000010) << 5)
        | ((x & 0b00000100) << 3)
        | ((x & 0b00001000) << 1)
        | ((x & 0b00010000) >> 1)
        | ((x & 0b00100000) >> 3)
        | ((x & 0b01000000) >> 5)
        | ((x & 0b10000000) >> 7)
}

pub type Result<T> = core::result::Result<T, display_interface::DisplayError>;

pub struct Vfd8md06inkm<'d, SPI, R, E, D> {
    digits: usize,
    spi: SPI,
    reset: R,
    enabled: E,
    delay: D,
    buffer: Option<&'d mut [u8]>,
}
impl<'d, SPI, R, E, D> Vfd8md06inkm<'d, SPI, R, E, D> {
    pub fn new(
        digits: usize,
        spi: SPI,
        reset: R,
        enabled: E,
        delay: D,
        buffer: Option<&'d mut [u8]>,
    ) -> Self {
        Self {
            digits,
            spi,
            reset,
            enabled,
            delay,
            buffer,
        }
    }
    pub fn digits(&self) -> usize {
        if let Some(buffer) = &self.buffer {
            (buffer.len() / 5).min(self.digits)
        } else {
            self.digits
        }
    }
    pub fn trans_delay_ns() -> u32 {
        // 4_000
        // 0
        // 1_000_000
        // 4_000_000
        100_000
        // 100_000_000
    }
    pub fn clear_buffer(&mut self) {
        if let Some(buffer) = self.buffer.as_mut() {
            buffer.iter_mut().for_each(|b| *b = 0);
        }
    }
}

impl<'d, SPI, R, E, D> Vfd8md06inkm<'d, SPI, R, E, D>
where
    SPI: embedded_hal::spi::SpiDevice,
    R: embedded_hal::digital::OutputPin,
    E: embedded_hal::digital::OutputPin,
    D: embedded_hal::delay::DelayNs,
{
    pub fn write_reg<C>(&mut self, cmd: C, data: u8) -> Result<()>
    where
        C: Into<u8>,
    {
        self.spi
            .transaction(&mut [
                Operation::Write(&[reverse_bits(cmd.into()), reverse_bits(data)]),
                Operation::DelayNs(Self::trans_delay_ns()),
            ])
            .map_err(|_| display_interface::DisplayError::BusWriteError)?;
        Ok(())
    }
    pub fn write_data<C>(&mut self, cmd: C, data: &[u8]) -> Result<()>
    where
        C: Into<u8>,
    {
        let mut buf = [0u8; 16];
        buf.iter_mut()
            .zip([cmd.into()].iter().chain(data.iter()))
            .for_each(|(b, d)| {
                *b = reverse_bits(*d);
            });
        self.spi
            .transaction(&mut [
                Operation::Write(&buf[..data.len() + 1]),
                Operation::DelayNs(Self::trans_delay_ns()),
            ])
            .map_err(|_| display_interface::DisplayError::BusWriteError)?;
        Ok(())
    }
    pub fn init(&mut self) -> Result<()> {
        self.enabled
            .set_high()
            .map_err(|_| display_interface::DisplayError::RSError)?;
        self.reset
            .set_low()
            .map_err(|_| display_interface::DisplayError::RSError)?;
        self.delay.delay_ms(10);
        self.reset
            .set_high()
            .map_err(|_| display_interface::DisplayError::RSError)?;
        self.delay.delay_ms(1);

        use Commands::*;
        self.write_reg(SetDisplayTiming, self.digits() as u8 - 1)?;
        self.write_reg(SetDimmingData, 31)?;
        self.write_reg(SetDisplayOn, 0)?;
        Ok(())
    }
    pub fn clear(&mut self) -> Result<()> {
        for d in 0..self.digits() {
            let addr = d as u8 + Commands::DCramDataWrite as u8;
            self.write_reg(addr, Commands::DGramDataClear.into())?;
        }
        Ok(())
    }
    pub fn write_str(&mut self, start: u8, s: &str) -> Result<()> {
        if start > self.digits() as u8 {
            return Err(display_interface::DisplayError::OutOfBoundsError);
        }
        let s = &s.as_bytes()[..(s.len().min(self.digits() - start as usize))];
        let addr = start + Commands::DCramDataWrite as u8;
        self.write_data(addr, s)?;
        Ok(())
    }
    pub fn flush(&mut self) -> Result<()> {
        if self.buffer.is_some() {
            let digits = self.digits();
            for i in 0..digits {
                let addr = i as u8 + Commands::CGramDataWrite as u8;
                let mut buf = [0u8; 5];
                buf.copy_from_slice(&self.buffer.as_ref().unwrap()[(i * 5)..(i * 5) + 5]);
                self.write_data(addr, &buf)?;
            }
            let indexes: [u8; 16] = core::array::from_fn(|i| i as u8);
            self.write_data(Commands::DCramDataWrite, &indexes[..digits])?;
            Ok(())
        } else {
            Err(display_interface::DisplayError::DataFormatNotImplemented)
        }
    }
}

impl<'d, SPI, R, E, D> Vfd8md06inkm<'d, SPI, R, E, D>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    R: embedded_hal::digital::OutputPin,
    E: embedded_hal::digital::OutputPin,
    D: embedded_hal::delay::DelayNs,
{
    pub async fn write_reg_async<C>(&mut self, cmd: C, data: u8) -> Result<()>
    where
        C: Into<u8>,
    {
        self.spi
            .transaction(&mut [
                Operation::Write(&[reverse_bits(cmd.into()), reverse_bits(data)]),
                Operation::DelayNs(Self::trans_delay_ns()),
            ])
            .await
            .map_err(|_| display_interface::DisplayError::BusWriteError)?;
        Ok(())
    }
    pub async fn write_data_async<C>(&mut self, cmd: C, data: &[u8]) -> Result<()>
    where
        C: Into<u8>,
    {
        let mut buf = [0u8; 16];
        buf.iter_mut()
            .zip([cmd.into()].iter().chain(data.iter()))
            .for_each(|(b, d)| {
                *b = reverse_bits(*d);
            });
        self.spi
            .transaction(&mut [
                Operation::Write(&buf[..data.len() + 1]),
                Operation::DelayNs(Self::trans_delay_ns()),
            ])
            .await
            .map_err(|_| display_interface::DisplayError::BusWriteError)?;
        Ok(())
    }
    pub async fn init_async(&mut self) -> Result<()> {
        self.enabled
            .set_high()
            .map_err(|_| display_interface::DisplayError::RSError)?;
        self.reset
            .set_low()
            .map_err(|_| display_interface::DisplayError::RSError)?;
        self.delay.delay_ms(10);
        self.reset
            .set_high()
            .map_err(|_| display_interface::DisplayError::RSError)?;
        self.delay.delay_ms(1);

        use Commands::*;
        self.write_reg_async(SetDisplayTiming, self.digits() as u8 - 1)
            .await?;
        self.write_reg_async(SetDimmingData, 31).await?;
        self.write_reg_async(SetDisplayOn, 0).await?;
        Ok(())
    }
    pub async fn clear_async(&mut self) -> Result<()> {
        for d in 0..self.digits() {
            let addr = d as u8 + Commands::DCramDataWrite as u8;
            self.write_reg_async(addr, Commands::DGramDataClear.into())
                .await?;
        }
        Ok(())
    }
    pub async fn write_str_async(&mut self, start: u8, s: &str) -> Result<()> {
        if start > self.digits() as u8 {
            return Err(display_interface::DisplayError::OutOfBoundsError);
        }
        let s = &s.as_bytes()[..(s.len().min(self.digits() - start as usize))];
        let addr = start + Commands::DCramDataWrite as u8;
        self.write_data_async(addr, s).await?;
        Ok(())
    }
    pub async fn flush_async(&mut self) -> Result<()> {
        if self.buffer.is_some() {
            let digits = self.digits();
            for i in 0..digits {
                let addr = i as u8 + Commands::CGramDataWrite as u8;
                let mut buf = [0u8; 5];
                buf.copy_from_slice(&self.buffer.as_ref().unwrap()[(i * 5)..(i * 5) + 5]);
                self.write_data_async(addr, &buf).await?;
            }
            let indexes: [u8; 16] = core::array::from_fn(|i| i as u8);
            self.write_data_async(Commands::DCramDataWrite, &indexes[..digits])
                .await?;
            Ok(())
        } else {
            Err(display_interface::DisplayError::DataFormatNotImplemented)
        }
    }
}

#[cfg(feature = "graphics")]
impl<'d, SPI, R, E, D> Dimensions for Vfd8md06inkm<'d, SPI, R, E, D> {
    fn bounding_box(&self) -> embedded_graphics_core::primitives::Rectangle {
        embedded_graphics_core::primitives::Rectangle::new(
            embedded_graphics_core::geometry::Point::zero(),
            embedded_graphics_core::geometry::Size::new(self.digits() as u32 * 5, 7),
        )
    }
}

#[cfg(feature = "graphics")]
impl<'d, SPI, R, E, D> DrawTarget for Vfd8md06inkm<'d, SPI, R, E, D>
where
    SPI: embedded_hal::spi::SpiDevice,
    R: embedded_hal::digital::OutputPin,
    E: embedded_hal::digital::OutputPin,
    D: embedded_hal::delay::DelayNs,
{
    type Error = display_interface::DisplayError;
    type Color = embedded_graphics_core::pixelcolor::BinaryColor;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<()>
    where
        I: IntoIterator<Item = embedded_graphics_core::Pixel<Self::Color>>,
    {
        // write to buffer then flush
        if let Some(buffer) = self.buffer.as_mut() {
            for pixel in pixels {
                let x = pixel.0.x as usize;
                let y = pixel.0.y as usize;
                let b = &mut buffer[x];
                if y < 7 {
                    if pixel.1.is_on() {
                        *b |= 1 << y;
                    } else {
                        *b &= !(1 << y);
                    }
                }
            }
            // self.buffer.iter_mut().for_each(|b| *b = 0x55);
            self.flush()?;
        }
        Ok(())
    }
}
