use defmt::Format;
use nmea0183 as nmea;
use super::Error;

#[derive(Debug)]


#[derive(Format, Clone, Copy)]
pub struct GnssLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: Option<f32>,
    pub speed: Option<f32>,
    pub course: Option<f32>,
}

impl Default for GnssLocation {
    fn default() -> Self {
        Self {
            latitude: 1.0,
            longitude: 1.0,
            altitude: Some(1.0),
            speed: Some(1.0),
            course: Some(1.0),
        }
    }
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