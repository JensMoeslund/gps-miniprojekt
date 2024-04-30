#![no_std]

use panic_probe as _; // panic handler
use fugit as _;
use defmt_brtt as _; // global logger
pub mod ms5611;
pub mod gnss;

#[macro_export]
macro_rules! configure_clock {
    ($device:ident, $core:ident, $sysclk:expr) => {{
        // Set up the system clock.
        let rcc = $device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk($sysclk).freeze();

        defmt::debug!("AHB1 clock: {} Hz", clocks.hclk().to_Hz());
        defmt::debug!("APB1 clock: {} Hz", clocks.pclk1().to_Hz());

        // enable tracing and the cycle counter for the monotonic timer
        $core.DCB.enable_trace();
        $core.DWT.enable_cycle_counter();

        // Set up the monotonic timer
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        rtic_monotonics::systick::Systick::start(
            $core.SYST,
            clocks.sysclk().to_Hz(),
            systick_mono_token,
        );

        // TODO: Change this to print date and time instead of uptime
        defmt::timestamp!("{=u64:us}", {
            // This will overflow after 584868 years :)
            let t: u64 = rtic_monotonics::systick::Systick::now()
                .duration_since_epoch()
                .to_micros();
            t
        });
        clocks
    }};
}
