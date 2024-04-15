#![no_main]
#![no_std]
// #![deny(warnings)]
#![feature(type_alias_impl_trait)]

use gps_miniprojekt as _; // global logger + panicking-behavior
use gps_miniprojekt::ms5611::Ms5611Reg;
use rtic_monotonics::{systick::Systick, Monotonic};
use stm32f4xx_hal::{
    gpio::{gpioa::PA5, gpiob::PB8, gpiob::PB9, Output, PushPull},
    prelude::*,
};
use byteorder::{ByteOrder, BigEndian};


#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART3,SPI1])]
mod app {
    use core::u8;

    use super::*;
    use byteorder::LittleEndian;
    // use alloc::string::ToString;
    use defmt::info;
    use gps_miniprojekt::ms5611;
    use gps_miniprojekt::ms5611::Osr;
    use stm32f4xx_hal::i2c::I2c;
    use stm32f4xx_hal::pac::I2C1;
    #[shared]
    struct Shared {}

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        ms5611: ms5611::Ms5611<I2c<I2C1>>,
    }
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        info!("init");

        let mut _device: stm32f4xx_hal::pac::Peripherals = ctx.device;
        let mut _core: cortex_m::Peripherals = ctx.core;
        let i2c1 = _device.I2C1;
        let clocks = gps_miniprojekt::configure_clock!(_device, _core, 84.MHz());

        let gpiob = _device.GPIOB.split();

        let scl = gpiob.pb8;
        let sda = gpiob.pb9;

        let gpioa = _device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();
        let i2c = i2c1.i2c(
            (scl, sda),100.kHz(),
            &clocks,
        );
        let mut ms5611 = ms5611::Ms5611::new(i2c, 0x77.into(),Osr::Opt1024.into()).unwrap();
        blink::spawn().ok();
        (Shared {}, Local { led, ms5611 })
    }
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(local = [led,ms5611], priority = 1)]
    async fn blink(ctx: blink::Context) {
        let ms5611 = ctx.local.ms5611;
        let mut buf = [0u8; 3];
        // Start by resetting to get the calibration data
        ms5611.reset().await.unwrap();


        loop {
            let t = Systick::now();
            ctx.local.led.toggle();
            defmt::info!("Blink!");

            let meas = ms5611.read_sample().await.unwrap();
            defmt::info!("Measurement: {}", meas);
            // Systick::delay_until(t + 1.secs()).await;
        }
    }
}
