#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::embassy;
use esp_hal::spi::master::Spi;
use esp_hal::spi::SpiMode;
use esp_hal::timer::TimerGroup;
use esp_hal::IO;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay};
use esp_hal::{systimer::SystemTimer, Rng};
use esp_wifi::{initialize, EspWifiInitFor};
use log::*;

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

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
    let res = io.pins.gpio6.into_push_pull_output();
    let en = io.pins.gpio5.into_push_pull_output();
    let cs = io.pins.gpio7.into_push_pull_output();

    let spi = Spi::new(
        peripherals.SPI2,
        fugit::HertzU32::kHz(100),
        SpiMode::Mode0,
        &clocks,
    )
    .with_sck(sclk)
    .with_mosi(mosi);

    let spi_mutex = NoopMutex::new(RefCell::new(spi));
    let spi_device = SpiDevice::new(&spi_mutex, cs);
    let mut buffer = [0u8; 8 * 5];
    let mut vfd = vfd_8md06inkm::Vfd8md06inkm::new(spi_device, res, en, delay, &mut buffer);
    vfd.init().unwrap();
    // vfd.clear().unwrap();
    loop {
        // vfd.init().unwrap();
        // vfd.clear().unwrap();
        vfd.write_str(0, "TEST").unwrap();
        DelayNs::delay_ms(&mut delay, 1000);
    }
}
