#![no_std]

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
    spi: SPI,
    reset: R,
    enabled: E,
    delay: D,
    buffer: &'d mut [u8],
}

impl<'d, SPI, R, E, D> Vfd8md06inkm<'d, SPI, R, E, D>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    R: embedded_hal::digital::OutputPin,
    E: embedded_hal::digital::OutputPin,
    D: embedded_hal::delay::DelayNs,
{
    pub fn new(spi: SPI, reset: R, enabled: E, delay: D, buffer: &'d mut [u8]) -> Self {
        Self {
            spi,
            reset,
            enabled,
            delay,
            buffer,
        }
    }
    // fn trans_delay() -> u32 {
    //     // 40_000
    //     0
    // }
    pub fn digits(&self) -> usize {
        self.buffer.len() / 5
    }
    pub async fn write_reg<C>(&mut self, cmd: C, data: u8) -> Result<()>
    where
        C: Into<u8>,
    {
        self.spi
            .transaction(&mut [
                Operation::Write(&[reverse_bits(cmd.into()), reverse_bits(data)]),
                // Operation::DelayNs(Self::trans_delay()),
            ])
            .await
            .map_err(|_| display_interface::DisplayError::BusWriteError)?;
        Ok(())
    }
    pub async fn write_data<C>(&mut self, cmd: C, data: &[u8]) -> Result<()>
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
                // Operation::DelayNs(Self::trans_delay()),
            ])
            .await
            .map_err(|_| display_interface::DisplayError::BusWriteError)?;
        Ok(())
    }
    pub async fn init(&mut self) -> Result<()> {
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
        self.write_reg(SetDisplayTiming, self.digits() as u8 - 1)
            .await?;
        self.write_reg(SetDimmingData, 31).await?;
        self.write_reg(SetDisplayOn, 0).await?;
        Ok(())
    }
    pub async fn clear(&mut self) -> Result<()> {
        for d in 0..self.digits() {
            let addr = d as u8 + Commands::DCramDataWrite as u8;
            self.write_reg(addr, Commands::DGramDataClear.into())
                .await?;
        }
        Ok(())
    }
    pub async fn write_str(&mut self, start: u8, s: &str) -> Result<()> {
        if start > self.digits() as u8 {
            return Err(display_interface::DisplayError::OutOfBoundsError);
        }
        let s = &s.as_bytes()[..(s.len().min(self.digits() - start as usize))];
        let addr = start + Commands::DCramDataWrite as u8;
        self.write_data(addr, s).await?;
        Ok(())
    }
}
