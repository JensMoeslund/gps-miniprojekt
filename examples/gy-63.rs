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
    // use alloc::string::ToString;
    use defmt::info;
    #[shared]
    struct Shared {}

    // Holds the local resources (used by a single task)
    // Needed even if we don't use it
    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        i2c: stm32f4xx_hal::i2c::I2c<stm32f4xx_hal::pac::I2C1>,
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
        blink::spawn().ok();
        (Shared {}, Local { led, i2c })
    }
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(local = [led,i2c], priority = 1)]
    async fn blink(ctx: blink::Context) {
        let i2c = ctx.local.i2c;
        let mut buf = [0u8; 3];
        loop {
            let t = Systick::now();
            ctx.local.led.toggle();
            defmt::info!("Blink!");
            i2c.write(0x77,&[Ms5611Reg::D1.addr()]).unwrap();
            Systick::delay(10.millis().into()).await;
            i2c.write(0x77,&[Ms5611Reg::AdcRead.addr()]).unwrap();

            i2c.read(0x77, &mut buf).unwrap();
            defmt::info!("{:?}",BigEndian::read_i24(&buf));
            Systick::delay_until(t + 1.secs()).await;
        }
    }
}
