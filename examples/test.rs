//! examples/executor-size.rs

#![no_main]
#![no_std]
// #![deny(warnings)]
#![deny(unsafe_code)]
// #![deny(missing_docs)]

use gps_miniprojekt as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART3,SPI1])]
mod app {
    use defmt;

    #[shared]
    struct Shared {
        bo: [u8; 10],
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init, total executor size = {}", cx.executors_size);
        let bo = [0u8;10];
        foo::spawn().ok();
        bar::spawn().ok();
        baz::spawn().ok();

        (Shared {bo}, Local {})
    }

    #[task(priority = 1, shared = [bo])]
    async fn foo(_cx: foo::Context) {
        let bob = [0u8;10];
    }

    #[task]
    async fn bar(_cx: bar::Context) {}

    #[task]
    async fn baz(_cx: baz::Context) {
        loop {
            continue;
        };
    }
}