#![no_std]

use defmt::Format;
use defmt_brtt as _; // global logger
use fugit as _;
// use nmea::Nmea;
use nmea0183 as nmea;
// time units
use panic_probe as _; // panic handler
use stm32f4xx_hal as _; // memory layout // time abstractions
pub mod ms5611;
#[derive(Debug)]
pub enum Error {
    InvalidSentence,

    SentenceWithoutPositonData,
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
        }
    }
}

#[derive(Format, Default, Clone, Copy)]
pub struct GnssLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: Option<f32>,
    pub speed: Option<f32>,
    pub course: Option<f32>,
}

impl TryFrom<nmea::ParseResult> for GnssLocation {
    type Error = Error;
    fn try_from(value: nmea::ParseResult) -> Result<Self, Self::Error> {
        match value {
            nmea::ParseResult::RMC(Some(rmc)) => Ok(Self::from(rmc)),
            nmea::ParseResult::GGA(Some(gga)) => Ok(Self::from(gga)),
            nmea::ParseResult::GLL(Some(gll)) => Ok(Self::from(gll)),
            nmea::ParseResult::RMC(None) => Err(Error::InvalidSentence),
            nmea::ParseResult::GGA(None) => Err(Error::InvalidSentence),
            nmea::ParseResult::GLL(None) => Err(Error::InvalidSentence),
            _ => Err(Error::SentenceWithoutPositonData),
        }
    }
}

impl GnssLocation {
    pub fn update(&mut self, value: nmea::ParseResult) -> Result<(), Error> {
        let location = GnssLocation::try_from(value)?;
        self.latitude = location.latitude;
        self.longitude = location.longitude;
        self.altitude = location.altitude.or(self.altitude);
        self.speed = location.speed.or(self.speed);
        self.course = location.course.or(self.course);
        Ok(())
    }
}

impl From<nmea::RMC> for GnssLocation {
    fn from(rmc: nmea::RMC) -> Self {
        GnssLocation {
            latitude: rmc.latitude.as_f64(),
            longitude: rmc.longitude.as_f64(),
            altitude: None,
            speed: Some(rmc.speed.as_mps()),
            course: rmc.course.map(|f| f.degrees),
        }
    }
}
impl From<nmea::GGA> for GnssLocation {
    fn from(gga: nmea::GGA) -> Self {
        GnssLocation {
            latitude: gga.latitude.as_f64(),
            longitude: gga.longitude.as_f64(),
            altitude: Some(gga.altitude.meters),
            speed: None,
            course: None,
        }
    }
}
impl From<nmea::GLL> for GnssLocation {
    fn from(gll: nmea::GLL) -> Self {
        GnssLocation {
            latitude: gll.latitude.as_f64(),
            longitude: gll.longitude.as_f64(),
            altitude: None,
            speed: None,
            course: None,
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
