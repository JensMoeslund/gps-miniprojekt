#![no_main]
#![no_std]
// #![deny(warnings)]
// #![feature(type_alias_impl_trait)]

use gps_miniprojekt as _; // global logger + panicking-behavior
use gps_miniprojekt::gnss::GnssLocation;
use gps_miniprojekt::ms5611::Ms5611Sample;
use gps_miniprojekt::SensorData;
use gps_miniprojekt::kalman::{KalmanFilter, Matrix8};

use fugit::ExtU32;

const BUF_LEN: usize = 20;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART3,SPI1,ETH,ETH_WKUP])]
mod app {

    use core::{mem::size_of, u8};

    use super::*;
    // use alloc::string::ToString;
    use defmt::info;
    use gps_miniprojekt::{kalman, ms5611::{self, Osr}};
    use nalgebra::Matrix6;
    use nmea0183::Parser;
    use stm32f4xx_hal::{
        i2c::I2c,
        pac::{I2C1, USART1},
        serial::Serial,
    };

    use rtic_monotonics::{systick::Systick, Monotonic};
    use stm32f4xx_hal::prelude::*;

    const DELTA_T: f32 = 1.0;
    const DELTA_T_MS: u32 = (DELTA_T * 1000.0) as u32;

    // Holds the shared resources (used by multiple tasks)
    // Needed even if we don't use it
    #[shared]
    struct Shared {
        gnss_data: GnssLocation,
        pressure_data: Ms5611Sample,
    }

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        serial: Serial<USART1>,
        ch_sender: rtic_sync::channel::Sender<'static, char, BUF_LEN>,
        ch_receiver: rtic_sync::channel::Receiver<'static, char, BUF_LEN>,
        ms5611: ms5611::Ms5611<I2c<I2C1>>,
        nmea_parser: Parser,
        kalman: KalmanFilter,
    }

    // The init function is called in the beginning of the program
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // panic!("test");
        info!("init");
        defmt::info!("size ={}", size_of::<nmea0183::Parser>());

        // Device specific peripherals
        let mut _device: stm32f4xx_hal::pac::Peripherals = ctx.device;
        let mut _core: cortex_m::Peripherals = ctx.core;
        let usart2 = _device.USART1;
        let clocks = gps_miniprojekt::configure_clock!(_device, _core, 84.MHz());

        // Set up the LED. On the Nucleo-F446RE it's connected to pin PA5.
        let gpioa = _device.GPIOA.split();

        // Set up the USART1 peripheral for the Nmea Reciever
        let tx = gpioa.pa9.into_alternate::<7>();
        let rx = gpioa.pa10.into_alternate::<7>();

        let (ch_sender, ch_receiver) = rtic_sync::make_channel!(char, BUF_LEN);
        let mut serial = usart2
            .serial(
                (tx, rx),
                stm32f4xx_hal::serial::config::Config::default().baudrate(9600.bps()),
                &clocks,
            )
            .unwrap();
        // Create the I2C device:
        let gpiob = _device.GPIOB.split();

        let scl = gpiob.pb8;
        let sda = gpiob.pb9;

        let i2c1 = _device.I2C1;
        let i2c = i2c1.i2c((scl, sda), 100.kHz(), &clocks);
        // Create the MS5611 device:
        let ms5611 =
            ms5611::Ms5611::new(i2c, 0x77.into(), Osr::Opt1024.into()).unwrap_or_else(|e| {
                defmt::panic!("Error: {:?}", defmt::Debug2Format(&e));
            });
        // defmt::info!("{}",size_of::<Result<ms5611::Ms5611<I2c<I2C1>>>>());
        let nmea_parser = Parser::new();

        // Create Kalman Filter
        let kalman = KalmanFilter::new(
            Matrix6::identity(),
            Matrix8::identity(),
            DELTA_T,
        );

        // usart2.bwrite_all("Hello, world!\n".as_bytes()).unwrap();
        serial.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);

        nmea_handler::spawn().unwrap();
        alt_sampler::spawn().unwrap();
        estimate::spawn().unwrap();
        defmt::info!("Init done!");
        (
            Shared {
                gnss_data: GnssLocation::default(),
                pressure_data: Ms5611Sample::default(),
            },
            Local {
                serial,
                ch_sender,
                ch_receiver,
                ms5611,
                nmea_parser,
                kalman,
            },
        )
    }

    // The idle function is called when there is nothing else to do
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(binds = USART1, priority=10, local = [serial,ch_sender])]
    fn usart_input_handler(ctx: usart_input_handler::Context) {
        let serial = ctx.local.serial;
        let queue = ctx.local.ch_sender;
        while let Ok(byte) = serial.read() {
            queue.try_send(byte as char).unwrap();
        }
    }

    #[task(priority = 2, local = [ch_receiver,nmea_parser], shared = [gnss_data])]
    async fn nmea_handler(ctx: nmea_handler::Context) {
        defmt::debug!("NMEA handler start");
        let queue = ctx.local.ch_receiver;
        let nmea_parser = ctx.local.nmea_parser;
        let mut gnss_data = ctx.shared.gnss_data;
        loop {
            // await here automatically defers the task after receiving a byte
            while let Ok(byte) = queue.recv().await {
                if let Some(result) = nmea_parser.parse_from_byte(byte as u8) {
                    match result {
                        Ok(result) => {
                            gnss_data.lock(|d| d.update(result)).unwrap_or_else(|e| {
                                // Update error is not critical
                                defmt::trace!("{}", e);
                            });
                        }
                        Err(e) => {
                            // Parse error should be logged
                            defmt::warn!("{}", e)
                        }
                    }
                }
            }
        }
    }

    // Task for printing the location and pressure data together:
    #[task(priority = 1, local=[kalman], shared = [gnss_data, pressure_data])]
    async fn estimate(ctx: estimate::Context) {
        let mut gnss_data = ctx.shared.gnss_data;
        let mut pressure_data = ctx.shared.pressure_data;
        let kalman = ctx.local.kalman;
        let mut observation = kalman::ObservationVector::default();
        loop {
            let t = Systick::now();
            let data = SensorData {
                ms5611_data: pressure_data.lock(|d| *d),
                gnss_location: gnss_data.lock(|d| *d),
            };
            observation.update(data, DELTA_T);
            defmt::info!("z: {}", observation);
            match kalman.update(&observation) {
                Ok(_) => {
                    defmt::info!("x: {}", kalman.predict());
                }
                Err(e) => {
                    defmt::warn!("{}", e);
                }
            }
            
            Systick::delay_until(t + DELTA_T_MS.millis()).await;

        }
    }

    #[task(priority = 2,local = [ms5611], shared = [pressure_data])]
    async fn alt_sampler(ctx: alt_sampler::Context) {
        // panic!("Sampler");
        let ms5611 = ctx.local.ms5611;
        // let mut nmea_struct = ctx.shared.nmea_struct;
        let mut pressure_data = ctx.shared.pressure_data;

        loop {
            let sample = ms5611.read_sample().await.unwrap();
            let t = Systick::now();
            pressure_data.lock(|d| {
                *d = sample;
            });
            Systick::delay_until(t + DELTA_T_MS.millis()).await;
        }
    }
}
