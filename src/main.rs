#![no_main]
#![no_std]
// #![deny(warnings)]
#![feature(type_alias_impl_trait)]

use gps_miniprojekt as _; // global logger + panicking-behavior
use gps_miniprojekt::NmeaReciever;
use rtic_monotonics::{systick::Systick, Monotonic};
use stm32f4xx_hal::{
    gpio::{gpioa::PA5, Output, PushPull},
    prelude::*,
};

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART3,SPI1])]
mod app {

    use core::u8;

    use super::*;
    // use alloc::string::ToString;
    use defmt::info;
    use gps_miniprojekt::Error::SentenceNotFinished;
    use heapless::String;
    use nmea::Nmea;
    use stm32f4xx_hal::{pac::USART1, serial::Serial};

    // Holds the shared resources (used by multiple tasks)
    // Needed even if we don't use it
    #[shared]
    struct Shared {
        cmd_buffer: heapless::String<82>,
        nmea_struct: nmea::Nmea,
    }

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        usart2: Serial<USART1>,
        nmea_reciever: NmeaReciever,
        sender: rtic_sync::channel::Sender<'static, char, 10>,
        receiver: rtic_sync::channel::Receiver<'static, char, 10>,
    }

    // The init function is called in the beginning of the program
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        info!("init");

        // Device specific peripherals
        let mut _device: stm32f4xx_hal::pac::Peripherals = ctx.device;
        let mut _core: cortex_m::Peripherals = ctx.core;
        let usart2 = _device.USART1;
        let clocks = gps_miniprojekt::configure_clock!(_device, _core, 84.MHz());

        // Set up the LED. On the Nucleo-F446RE it's connected to pin PA5.
        let gpioa = _device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

        // Set up the USART1 peripheral for the Nmea Reciever
        let tx = gpioa.pa9.into_alternate::<7>();
        let rx = gpioa.pa10.into_alternate::<7>();

        let (sender, receiver) = rtic_sync::make_channel!(char, 10);
        let mut usart2 = usart2
            .serial(
                (tx, rx),
                stm32f4xx_hal::serial::config::Config::default().baudrate(9600.bps()),
                &clocks,
            )
            .unwrap();
        // usart2.bwrite_all("Hello, world!\n".as_bytes()).unwrap();
        usart2.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);

        defmt::info!("Init done!");
        nmea_handler::spawn().ok();
        // blink::spawn().ok();
        (
            Shared {
                cmd_buffer: String::new(),
                nmea_struct: Nmea::default(),
            },
            Local {
                led,
                usart2,
                nmea_reciever: NmeaReciever::new(),
                sender,
                receiver,
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

    #[task(binds = USART1, priority=10, local = [usart2,sender])]
    fn usart_input_handler(ctx: usart_input_handler::Context) {
        let usart2 = ctx.local.usart2;
        let sender = ctx.local.sender;
        while let Ok(byte) = usart2.read() {
            sender.try_send(byte as char).unwrap();
        }
    }

    #[task(priority = 3, local = [nmea_reciever,receiver], shared = [cmd_buffer])]
    async fn nmea_handler(ctx: nmea_handler::Context) {
        let reciever = ctx.local.nmea_reciever;
        let mut cmd_buf = ctx.shared.cmd_buffer;
        let consumer = ctx.local.receiver;
        loop {
            // await here automatically defers the task after receiving a byte
            while let Ok(byte) = consumer.recv().await {
                match reciever.handle_byte(byte as char) {
                    Ok(sentence) => {
                        defmt::info!("Sentence: {:?}", sentence.as_str());
                        cmd_buf.lock(|b| b.push_str(sentence.as_str()).unwrap());
                        nmea_decode::spawn().ok();
                    }
                    Err(SentenceNotFinished) => continue,
                    Err(e) => {
                        defmt::error!("Error: {}", e);
                        break;
                    }
                }
            }
        }
    }

    #[task(priority = 1, shared = [nmea_struct, cmd_buffer])]
    async fn nmea_decode(ctx: nmea_decode::Context) {
        let mut buf = ctx.shared.cmd_buffer;
        let mut nmea_struct = ctx.shared.nmea_struct;

        defmt::info!("NMEA decode");
        let local_sentence = buf.lock(|b| {
            let sentence = b.clone();
            b.clear();
            sentence
        });
        defmt::debug!("NMEA: {:?}", local_sentence.as_str());

        nmea_struct.lock(
            |nmea_struct| match nmea_struct.parse(local_sentence.as_str()) {
                Ok(sentence_type) => {
                    defmt::info!("NMEA sentence type: {:?}", sentence_type.as_str());
                }
                Err(e) => {
                    // This is very expensive, but we're not expecting many errors
                    defmt::error!("Error: {}", defmt::Display2Format(&e));
                }
            },
        )
    }

    // The task functions are called by the scheduler
    #[task(local = [led], priority = 1)]
    async fn blink(ctx: blink::Context) {
        loop {
            let t = Systick::now();
            ctx.local.led.toggle();
            defmt::info!("Blink!");
            Systick::delay_until(t + 1.secs()).await;
        }
    }
}
