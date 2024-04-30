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
use heapless::String;
#[derive(Debug)]
pub enum Error {
    InvalidTransition {
        from: NmeaRecieverState,
        to: NmeaRecieverState,
    },

    BufferFull {
        byte: char,
    },

    SentenceNotStarted {
        byte: char,
    },

    SentenceWithoutPositonData,

    InvalidSentence,
    
    // Used as a return type for the handle_byte method to support while let loops
    SentenceNotFinished,
}

impl Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Error::InvalidTransition { from, to } => {
                defmt::write!(
                    fmt,
                    "Invalid reciever state transition from {:?} to {:?}",
                    from,
                    to
                )
            }
            Error::BufferFull { byte } => {
                defmt::write!(fmt, "Buffer full, cannot push byte {}", byte)
            }
            Error::SentenceNotStarted { byte } => {
                defmt::write!(fmt, "Sentence not started with {}", byte)
            }
            Error::SentenceNotFinished => {
                defmt::write!(fmt, "NMEA sentence not yet finished (continue reading)")
            }
            Error::SentenceWithoutPositonData => {
                defmt::write!(fmt, "NMEA sentence does not contain position data")
            },
            Error::InvalidSentence => {
                defmt::write!(fmt, "Invalid NMEA sentence")
            },
        }
    }
}

// Reciever finite state machine
// The reciever can only go forward in state, and can only go back to the Clear state.
// an invalid transition will result in returning an Error::InvalidTransition.
#[derive(Format, Debug, Clone, Copy)]
pub enum NmeaRecieverState {
    Clear,         // Nothing recieved yet
    StartRecieved, // Start of sentence recieved
    Recieving,     // Recieving a sentence
    CrRecieved,    // Carriage return recieved
    LfRecieved,    // Line feed recieved (end of sentence)
}

//
impl NmeaRecieverState {
    pub fn new() -> Self {
        NmeaRecieverState::Clear
    }

    fn clear(&mut self) {
        *self = NmeaRecieverState::Clear;
    }
    fn start_recieved(&mut self) -> Result<(), Error> {
        match self {
            NmeaRecieverState::Clear => {
                *self = NmeaRecieverState::StartRecieved;
                Ok(())
            }
            _ => Err(Error::InvalidTransition {
                from: *self,
                to: NmeaRecieverState::StartRecieved,
            }),
        }
    }

    fn recieving(&mut self) -> Result<(), Error> {
        match self {
            NmeaRecieverState::StartRecieved => {
                *self = NmeaRecieverState::Recieving;
                Ok(())
            }
            _ => Err(Error::InvalidTransition {
                from: *self,
                to: NmeaRecieverState::Recieving,
            }),
        }
    }

    fn cr_recieved(&mut self) -> Result<(), Error> {
        match self {
            NmeaRecieverState::Recieving => {
                *self = NmeaRecieverState::CrRecieved;
                Ok(())
            }
            _ => Err(Error::InvalidTransition {
                from: *self,
                to: NmeaRecieverState::CrRecieved,
            }),
        }
    }

    fn lf_recieved(&mut self) -> Result<(), Error> {
        match self {
            NmeaRecieverState::CrRecieved => {
                *self = NmeaRecieverState::LfRecieved;
                Ok(())
            }
            _ => Err(Error::InvalidTransition {
                from: *self,
                to: NmeaRecieverState::LfRecieved,
            }),
        }
    }
}

const BUF_LEN: usize = 82; // 82 is the maximum length of a NMEA sentence

pub struct NmeaReciever {
    state: NmeaRecieverState,
    buffer: String<BUF_LEN>,
}

impl NmeaReciever {
    pub fn new() -> Self {
        NmeaReciever {
            state: NmeaRecieverState::Clear,
            buffer: String::new(),
        }
    }

    fn clear(&mut self) {
        self.state.clear();
        self.buffer.clear();
    }
    fn start_recieved(&mut self) -> Result<(), Error> {
        self.state.start_recieved()
    }

    fn recieving(&mut self) -> Result<(), Error> {
        self.state.recieving()
    }

    fn cr_recieved(&mut self) -> Result<(), Error> {
        self.state.cr_recieved()
    }

    fn lf_recieved(&mut self) -> Result<(), Error> {
        self.state.lf_recieved()
    }

    fn push(&mut self, byte: char) -> Result<(), Error> {
        self.buffer
            .push(byte)
            .map_err(|_| Error::BufferFull { byte })
    }

    pub fn handle_byte(&mut self, byte: char) -> Result<String<BUF_LEN>, Error> {
        match self.state {
            NmeaRecieverState::Clear => match byte {
                '$' => {
                    self.start_recieved()?;
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
                _ => {
                    self.clear();
                    Err(Error::SentenceNotStarted { byte })
                }
            },
            NmeaRecieverState::StartRecieved => match byte {
                '$' => {
                    self.clear();
                    self.start_recieved()?;
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
                _ => {
                    self.recieving()?;
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
            },
            NmeaRecieverState::Recieving => match byte {
                '\r' => {
                    self.cr_recieved()?;
                    // self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
                '$' => {
                    self.clear();
                    self.start_recieved()?;
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
                _ => {
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
            },
            NmeaRecieverState::CrRecieved => match byte {
                '\n' => {
                    self.lf_recieved()?;
                    // self.push(byte)?;
                    let sentence = self.buffer.clone();
                    self.clear();
                    Ok(sentence)
                }
                '$' => {
                    self.clear();
                    self.start_recieved()?;
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
                _ => {
                    self.push(byte)?;
                    Err(Error::SentenceNotFinished)
                }
            },
            NmeaRecieverState::LfRecieved => {
                // This should never happen, but we have to handle it
                self.clear();
                Err(Error::InvalidTransition {
                    from: NmeaRecieverState::LfRecieved,
                    to: NmeaRecieverState::Clear,
                })
            }
        }
    }
}
#[derive(Format,Default,Clone, Copy)]
pub struct GnssLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: Option<f32>,
}

impl TryFrom<nmea::ParseResult> for GnssLocation {
    type Error = Error;
    fn try_from(value: nmea::ParseResult) -> Result<Self,Self::Error> {
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
impl From<nmea::RMC> for GnssLocation {
    fn from(rmc: nmea::RMC) -> Self {
        GnssLocation {
            latitude: rmc.latitude.as_f64(),
            longitude: rmc.longitude.as_f64(),
            altitude: None,
        }
    }

}
impl From<nmea::GGA> for GnssLocation {
    fn from(gga: nmea::GGA) -> Self {
        GnssLocation {
            latitude: gga.latitude.as_f64(),
            longitude: gga.longitude.as_f64(),
            altitude: Some(gga.altitude.meters),
        }
    }
    
}
impl From<nmea::GLL> for GnssLocation {
    fn from(gll: nmea::GLL) -> Self {
        GnssLocation {
            latitude: gll.latitude.as_f64(),
            longitude: gll.longitude.as_f64(),
            altitude: None,
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
