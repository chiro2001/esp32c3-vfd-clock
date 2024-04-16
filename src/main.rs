#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::dma::{Dma, DmaPriority};
use esp_hal::spi::master::prelude::*;
use esp_hal::spi::master::Spi;
use esp_hal::spi::SpiMode;
use esp_hal::timer::TimerGroup;
use esp_hal::IO;
use esp_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay};
use esp_hal::{dma_descriptors, embassy};
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

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(512);

    let spi = Spi::new(
        peripherals.SPI2,
        fugit::HertzU32::kHz(1),
        SpiMode::Mode0,
        &clocks,
    )
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    let spi_mutex =
        embassy_sync::mutex::Mutex::<embassy_sync::blocking_mutex::raw::NoopRawMutex, _>::new(spi);
    let spi_device = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&spi_mutex, cs);
    let mut vfd = vfd_8md06inkm::Vfd8md06inkm::new(8, spi_device, res, en, delay, None);
    let mut cnt = 0;
    loop {
        vfd.init_async().await.unwrap();
        let mut buf = [0u8; 16];
        let s = format_no_std::show(&mut buf, format_args!("[{}]", cnt)).unwrap();
        vfd.write_str_async(0, s).await.unwrap();
        cnt += 1;
        DelayNs::delay_ms(&mut delay, 100);
    }
}
