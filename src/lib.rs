#![no_std]

use defmt_brtt as _; // global logger
use fugit as _; // time units
use panic_probe as _; // panic handler
use snafu::prelude::*;
use stm32f4xx_hal as _; // memory layout // time abstractions
use defmt::Format;

use heapless::String;
#[derive(Format, Debug, Snafu)]
pub enum Error {
    #[snafu(display("Invalid reciever state transition from {:?} to {:?}", from, to))]
    InvalidTransition {
        from: NmeaRecieverState,
        to: NmeaRecieverState,
    },

    #[snafu(display("Buffer full, cannot push byte {}", byte))]
    BufferFull { byte: char },

    #[snafu(display("Invalid utf8 string"))]
    InvalidUtf8String,

    #[snafu(display("NMEA sentence not yet finished"))]
    // Used as a return type for the handle_byte method to support while let loops
    SentenceNotFinished,
}

// Reciever finite state machine
// The reciever can only go forward in state, and can only go back to the Clear state.
// an invalid transition will result in returning an Error::InvalidTransition.
#[derive(Format, Debug, Clone, Copy)]
pub enum NmeaRecieverState {
    Clear,      // Nothing recieved yet
    Recieving,  // Recieving a sentence
    CrRecieved, // Carriage return recieved
    LfRecieved, // Line feed recieved (end of sentence)
}

//
impl NmeaRecieverState {
    pub fn new() -> Self {
        NmeaRecieverState::Clear
    }

    fn clear(&mut self) {
        *self = NmeaRecieverState::Clear;
    }

    fn recieving(&mut self) -> Result<(), Error> {
        match self {
            NmeaRecieverState::Clear => {
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
            NmeaRecieverState::Clear => {
                self.recieving()?;
                self.push(byte)?;
                Err(Error::SentenceNotFinished)
            }
            NmeaRecieverState::Recieving => match byte {
                '\r' => {
                    self.cr_recieved()?;
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
                    self.push(byte)?;
                    let sentence = self.buffer.clone();
                    self.clear();
                    Ok(sentence)
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
