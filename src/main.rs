#![no_main]
#![no_std]
// #![deny(warnings)]
// #![feature(type_alias_impl_trait)]

use gps_miniprojekt as _; // global logger + panicking-behavior
use gps_miniprojekt::ms5611::Ms5611Sample;
use gps_miniprojekt::{GnssLocation, NmeaReciever};
use nmea0183::datetime::DateTime;


struct SensorData {
    ms5611_data: Ms5611Sample,
    gnss_location: GnssLocation,
    // timestamp: DateTime,
}
impl SensorData {
    pub fn new() -> Self {
        todo!();
    }

}


const BUF_LEN: usize = 20; 
impl defmt::Format for SensorData {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "{},{},{},{},{}",
            self.ms5611_data.temperature_c,
            self.ms5611_data.pressure_mbar,
            self.gnss_location.latitude,
            self.gnss_location.longitude,
            self.gnss_location.altitude 
        );
    }
}

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART3,SPI1,ETH,ETH_WKUP])]
mod app {

    use core::{mem::size_of, u8};

    use super::*;
    // use alloc::string::ToString;
    use defmt::info;
    use gps_miniprojekt::{
        ms5611::{self, Osr},
        Error::SentenceNotFinished,
    };
    use heapless::String;
    use nmea0183::{Parser, ParseResult};
    use stm32f4xx_hal::{
        i2c::I2c,
        pac::{dma1::st::par, I2C1, USART1},
        serial::Serial,
    };

    use rtic_monotonics::{systick::Systick, Monotonic};
    use stm32f4xx_hal::prelude::*;
    // Holds the shared resources (used by multiple tasks)
    // Needed even if we don't use it
    #[shared]
    struct Shared {
        cmd_buffer: heapless::String<82>,
        gnss_data: GnssLocation,
        pressure_data: Ms5611Sample,


    }

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        serial: Serial<USART1>,
        nmea_reciever: NmeaReciever,
        sender: rtic_sync::channel::Sender<'static, char, BUF_LEN>,
        receiver: rtic_sync::channel::Receiver<'static, char, BUF_LEN>,
        ms5611: ms5611::Ms5611<I2c<I2C1>>,
        parser: Parser,
    }

    // The init function is called in the beginning of the program
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // panic!("test");
        info!("init");
        defmt::info!("size ={}",size_of::<nmea0183::Parser>());

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

        let (sender, receiver) = rtic_sync::make_channel!(char, BUF_LEN);
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
        let ms5611 = ms5611::Ms5611::new(i2c, 0x77.into(), Osr::Opt1024.into()).unwrap_or_else(|e| {
            defmt::error!("Error: {:?}", defmt::Debug2Format(&e));
            panic!();
        });
        // defmt::info!("{}",size_of::<Result<ms5611::Ms5611<I2c<I2C1>>>>());
        let parser = Parser::new();
        

        // usart2.bwrite_all("Hello, world!\n".as_bytes()).unwrap();
        serial.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);
        
        nmea_handler::spawn().unwrap();
        alt_sampler::spawn().unwrap();
        print_data::spawn().unwrap();
        defmt::info!("Init done!");

        // gga_saver::spawn().ok();
        (
            Shared {
                cmd_buffer: String::new(),
                gnss_data: GnssLocation::default(),
                pressure_data: Ms5611Sample::default(),
                // nmea_struct: Nmea::default(),
            },
            Local {
                serial,
                nmea_reciever: NmeaReciever::new(),
                sender,
                receiver,
                ms5611,
                parser,
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

    #[task(binds = USART1, priority=10, local = [serial,sender])]
    fn usart_input_handler(ctx: usart_input_handler::Context) {
        let serial = ctx.local.serial;
        let sender = ctx.local.sender;
        while let Ok(byte) = serial.read() {
            sender.try_send(byte as char).unwrap();
        }
    }

    #[task(priority = 2, local = [nmea_reciever,receiver,parser], shared = [cmd_buffer, gnss_data])]
    async fn nmea_handler(ctx: nmea_handler::Context) {
        defmt::debug!("NMEA handler start");
        let mut cmd_buf = ctx.shared.cmd_buffer;
        let queue = ctx.local.receiver;
        let parser = ctx.local.parser;
        let mut gnss_data = ctx.shared.gnss_data;
        loop {
            // await here automatically defers the task after receiving a byte
            while let Ok(byte) = queue.recv().await {
                if let Some(result) = parser.parse_from_byte(byte as u8) {
                    match result {
                        Ok(sentence) => {
                            gnss_data.lock(|d| {
                                if let Ok(loc) = GnssLocation::try_from(sentence) {
                                    *d = loc;
                                };
                            })
                        }, // Some other sentences..
                        Err(e) => {defmt::error!("{}",e)} // Got parse error
                    }
                }
            }
        }
    }

    // #[task(priority = 3, shared = [ cmd_buffer])]
    // async fn nmea_decode(ctx: nmea_decode::Context) {
    //     let mut buf = ctx.shared.cmd_buffer;
    //     // let mut nmea_struct = ctx.shared.nmea_struct;

    //     defmt::debug!("NMEA decode");
    //     let local_sentence = buf.lock(|b| {
    //         let sentence = b.clone();
    //         b.clear();
    //         sentence
    //     });
    //     defmt::trace!("NMEA: {:?}", local_sentence.as_str());

    //     // nmea_struct.lock(
    //         // |nmea_struct| match nmea_struct.parse(local_sentence.as_str()) {
    //         //     Ok(sentence_type) => {
    //         //         defmt::debug!("NMEA sentence type: {:?}", sentence_type.as_str());
    //         //     }
    //         //     Err(e) => {
    //         //         // This is very expensive, but we're not expecting many errors
    //         //         defmt::error!("Error: {}", defmt::Display2Format(&e));
    //         //     }
    //         // },
    //     // );
    // }

    // Task for printing the location and pressure data together: 
    #[task(priority = 3, shared = [gnss_data, pressure_data])]
    async fn print_data(ctx: print_data::Context) {
        let mut gnss_data = ctx.shared.gnss_data;
        let mut pressure_data = ctx.shared.pressure_data;
        loop {
            let t = Systick::now();
            let data = SensorData {
                ms5611_data: pressure_data.lock(|d| *d),
                gnss_location: gnss_data.lock(|d| *d),
            };
            defmt::info!("{}",data);
            Systick::delay_until(t + 1.secs()).await;    
        }

    }

    // The task functions are called by the scheduler
    // #[task(shared = [nmea_struct], priority = 1)]
    // async fn gga_saver(ctx: gga_saver::Context) {
    //     let mut nmea_struct = ctx.shared.nmea_struct;

    //     loop {
    //         let t = Systick::now();
    //         defmt::info!(
    //             "{:?}",
    //             nmea_struct.lock(|nmea_struct| GnssLocation::from(nmea_struct))
    //         );
    //         Systick::delay_until(t + 1.secs()).await;
    //     }
    // }

 
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
            Systick::delay_until(t + 1.secs()).await;
        }
    }
}
