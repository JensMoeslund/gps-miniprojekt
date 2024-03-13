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

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART3])]
mod app {

    use core::u8;

    use super::*;
    use defmt::info;
    use stm32f4xx_hal::{pac::USART1, serial::Serial};

    // Holds the shared resources (used by multiple tasks)
    // Needed even if we don't use it
    #[shared]
    struct Shared {}

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        usart2: Serial<USART1>,
        buf: heapless::String<82>,
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

        let buf = heapless::String::new();

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
        (Shared {}, Local { led, usart2, buf })
    }

    // The idle function is called when there is nothing else to do
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(binds = USART1, priority = 2, local = [usart2,buf])]
    fn usart2(ctx: usart2::Context) {
        // defmt::debug!("USART2 interrupt");
        let usart2 = ctx.local.usart2;
        let buf = ctx.local.buf;
        while let Ok(byte) = usart2.read() {
            match byte as char {
                '\r'|'\n' => {
                    defmt::debug!("Received: {}", buf.as_str());
                    buf.clear();
                }
                _ => {
                    buf.push(byte as char).unwrap();
                }
            }
        }
        // let (mut tx, mut rx) = usart2.split();
        // let byte = rx.read().unwrap();
        // tx.write(byte).unwrap();
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
