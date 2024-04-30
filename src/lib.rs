#![no_std]

use defmt::Format;
use panic_probe as _; // panic handler
use fugit as _;
use defmt_brtt as _; // defmt logger
pub mod ms5611;
pub mod gnss;
pub mod kalman;

pub enum Error {
    InvalidSentence,
    SentenceWithoutPositonData,
    KalmanNotInvertible,
}

impl Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Error::SentenceWithoutPositonData => {
                defmt::write!(fmt, "NMEA sentence does not contain position data")
            }
            Error::InvalidSentence => {
                defmt::write!(fmt, "Invalid NMEA sentence")
            }
            Error::KalmanNotInvertible => {
                defmt::write!(fmt, "Kalman filter matrix not invertible")
            }
        }
    }
}

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
