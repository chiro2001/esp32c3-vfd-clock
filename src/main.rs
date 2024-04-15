#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::spi::master::Spi;
use esp_hal::spi::SpiMode;
use esp_hal::IO;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay};
use esp_hal::{systimer::SystemTimer, Rng};
use esp_wifi::{initialize, EspWifiInitFor};
use log::*;

struct SoftSpi<SCK, MOSI, CS, DELAY> {
    sck: SCK,
    mosi: MOSI,
    cs: CS,
    delay: DELAY,
}
#[derive(Debug)]
enum SoftSpiError {
    WriteError,
}

impl<SCK, MOSI, CS, DELAY> SoftSpi<SCK, MOSI, CS, DELAY>
where
    SCK: OutputPin,
    MOSI: OutputPin,
    CS: OutputPin,
    DELAY: DelayNs,
{
    fn new(sck: SCK, mosi: MOSI, cs: CS, delay: DELAY) -> Self {
        Self {
            sck,
            mosi,
            cs,
            delay,
        }
    }
    fn write_u8(&mut self, data: u8) {
        self.delay.delay_us(10);
        for i in 0..8 {
            if data & (1 << i) != 0 {
                self.mosi.set_high().unwrap();
            } else {
                self.mosi.set_low().unwrap();
            }
            self.sck.set_high().unwrap();
            self.delay.delay_us(10);
            self.sck.set_low().unwrap();
            self.delay.delay_us(10);
        }
        self.delay.delay_us(10);
        self.mosi.set_high().unwrap();
        // self.mosi.set_low().unwrap();
    }
    fn write(&mut self, data: &[u8]) -> Result<(), SoftSpiError> {
        self.cs.set_low().unwrap();
        self.delay.delay_us(20);
        for d in data {
            self.write_u8(*d);
            self.delay.delay_us(20);
        }
        self.cs.set_high().unwrap();
        self.delay.delay_us(20);
        Ok(())
    }
    fn write_str(&mut self, data: &str) -> Result<(), SoftSpiError> {
        let bytes = data.as_bytes();
        let mut addr = 0x20;
        for b in bytes {
            self.write(&[addr, *b])?;
            addr += 1;
        }
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();
    info!("Launched");
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let _init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio2.into_push_pull_output();
    let mosi = io.pins.gpio3.into_push_pull_output();
    let mut res = io.pins.gpio6.into_push_pull_output();
    let mut en = io.pins.gpio5.into_push_pull_output();
    let cs = io.pins.gpio7.into_push_pull_output();

    let mut spi = Spi::new(
        peripherals.SPI2,
        fugit::HertzU32::kHz(100),
        SpiMode::Mode0,
        &clocks,
    )
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_cs(cs)
    ;
    // let mut spi = SoftSpi::new(sclk, mosi, cs, delay);

    OutputPin::set_high(&mut en).unwrap();

    #[rustfmt::skip]
    let init = [
        [0xe0 as u8, 0x07], 
        [0xe4, (32 - 1)], 
        [0xe8, 0], 
        [0xec, 0],
        [0x20, 0x10],
        [0x21, 0x10],
        [0x22, 0x10],
        [0x23, 0x10],
        [0x24, 0x10],
        [0x25, 0x10],
        [0x26, 0x10],
        [0x27, 0x10],
    ];

    loop {
        OutputPin::set_high(&mut res).unwrap();
        DelayNs::delay_ms(&mut delay, 1);
        OutputPin::set_low(&mut res).unwrap();
        DelayNs::delay_ms(&mut delay, 10);
        OutputPin::set_high(&mut res).unwrap();
        DelayNs::delay_ms(&mut delay, 1);

        for cmd in init.as_ref().iter() {
            // spi.write(cmd).unwrap();
            spi.write(&[reverse_bits(cmd[0]), reverse_bits(cmd[1])])
                .unwrap();
            DelayNs::delay_us(&mut delay, 40);
        }

        DelayNs::delay_ms(&mut delay, 10);

        // spi.write_str("TEST").unwrap();
        let s = b"TEST";
        s.iter().enumerate().for_each(|(i, c)| {
            // spi.write(&[0x20 + i as u8, *c]).unwrap();
            spi.write(&[reverse_bits(0x20 + i as u8), reverse_bits(*c)])
                .unwrap();
            DelayNs::delay_us(&mut delay, 40);
        });

        DelayNs::delay_ms(&mut delay, 1000);
    }
}

// const fn reverse_bits(x: u8) -> u8 {
//     x
// }

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
