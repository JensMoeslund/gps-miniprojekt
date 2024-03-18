#![no_main]
#![no_std]
#![deny(warnings)]
#![feature(type_alias_impl_trait)]

use gps_miniprojekt as _; // global logger + panicking-behavior
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
    use stm32f4xx_hal::{pac::USART1, serial::Serial};

    // Holds the shared resources (used by multiple tasks)
    // Needed even if we don't use it
    #[shared]
    struct Shared {
        cmd_buffer: heapless::String<82>,
    }

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        usart2: Serial<USART1>,
        usart_input_buf: heapless::String<82>,
        nmea_struct: nmea::Nmea,
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

        let usart_input_buf = heapless::String::new();
        let cmd_buffer = heapless::String::new();
        let nmea_struct = nmea::Nmea::default();

        // Set up the LED. On the Nucleo-F446RE it's connected to pin PA5.
        let gpioa = _device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

        let tx = gpioa.pa9.into_alternate::<7>();
        let rx = gpioa.pa10.into_alternate::<7>();

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
        blink::spawn().ok();
        (
            Shared { cmd_buffer },
            Local {
                led,
                usart2,
                usart_input_buf,
                nmea_struct,
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

    #[task(binds = USART1, priority = 2, local = [usart2,usart_input_buf],shared = [cmd_buffer])]
    fn usart2(ctx: usart2::Context) {
        // defmt::debug!("USART2 interrupt");
        let usart2 = ctx.local.usart2;
        let usart_input_buf = ctx.local.usart_input_buf;
        let mut cmd_buf = ctx.shared.cmd_buffer;
        while let Ok(byte) = usart2.read() {
            match byte as char {
                '\r' | '\n' => {
                    defmt::debug!("Received: {}", usart_input_buf.as_str());
                    cmd_buf.lock(|b| b.push_str(usart_input_buf.as_str()).unwrap());
                    usart_input_buf.clear();
                    nmea_decode::spawn().ok();
                }
                _ => {
                    usart_input_buf.push(byte as char).unwrap();
                }
            }
        }
        // let (mut tx, mut rx) = usart2.split();
        // let byte = rx.read().unwrap();
        // tx.write(byte).unwrap();
    }

    #[task(priority = 2,local=[nmea_struct],shared = [cmd_buffer])]
    async fn nmea_decode(ctx: nmea_decode::Context) {
        let mut buf = ctx.shared.cmd_buffer;
        let _nmea_struct = ctx.local.nmea_struct;

        defmt::info!("NMEA decode");
        // let mut teststring;
        // buf.lock(|b| teststring = b.clone());
        // defmt::debug!("NMEA: {:?}",teststring.as_str());
        // buf.lock(|b| defmt::debug!("NMEA: {:?}",b.as_str()));
        let mut test = buf.lock(|b| b.clone());
        defmt::debug!("NMEA: {:?}", test.as_str());
        // match nmea_struct.parse(test.as_str()) {
        //     Ok(_) => {
        //         defmt::debug!("NMEA: {:?}", nmea_struct.latitude.unwrap());
        //     }
        //     Err(_) => return ,
        // }
        test.clear();
        // nmea_struct.parse(test.as_str()).unwrap();
        // test.unwrap();
        buf.lock(|b| b.clear());
        // defmt::debug!("Latitude: {:?}",nmea_struct.latitude.unwrap());
        // (buf.lock(|b| &b.pop().unwrap())).unwrap();
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
