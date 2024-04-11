use embedded_hal::i2c;
use byteorder::{BigEndian, ByteOrder};
use rtic_monotonics::{systick::Systick, Monotonic};
use stm32f4xx_hal::hal_02::blocking::i2c::{Read, Write};
use fugit as _;
pub enum Ms5611Reg {
    Reset,
    /// Digital pressure value
    D1,
    /// Digital temperature value
    D2,
    /// AdcRead command returns 24-bit result.
    AdcRead,
    /// Prom command returns 16-bit result.
    Prom,
}

impl Ms5611Reg {
    pub fn addr(&self) -> u8 {
        match *self {
            Ms5611Reg::Reset => 0x1e,
            Ms5611Reg::D1 => 0x40,
            Ms5611Reg::D2 => 0x50,
            Ms5611Reg::AdcRead => 0x00,
            // Valid from 0xa0 to 0xae
            Ms5611Reg::Prom => 0xa0,
        }
    }
}
/// Oversampling ratio
/// See datasheet for more information.
pub enum Osr {
    Opt256,
    Opt512,
    Opt1024,
    Opt2048,
    Opt4096,
}

impl Osr {
    pub fn get_delay(&self) -> u32 {
        match *self {
            Osr::Opt256 => 1,
            Osr::Opt512 => 2,
            Osr::Opt1024 => 3,
            Osr::Opt2048 => 5,
            Osr::Opt4096 => 10,
        }
    }

    pub fn addr_modifier(&self) -> u8 {
        match *self {
            Osr::Opt256 => 0,
            Osr::Opt512 => 2,
            Osr::Opt1024 => 4,
            Osr::Opt2048 => 6,
            Osr::Opt4096 => 8,
        }
    }
}
/// Output from the MS5611.
#[derive(Debug)]
pub struct Ms5611Sample {
    /// Pressure measured in millibars.
    pub pressure_mbar: f32,
    /// Temperature in celsius.
    pub temperature_c: f32,
}
/// Factory calibrated data in device's ROM.
#[derive(Debug)]
struct Prom {
    /// From datasheet, C1.
    pub pressure_sensitivity: u16,
    /// From datasheet, C2.
    pub pressure_offset: u16,
    /// From datasheet, C3.
    pub temp_coef_pressure_sensitivity: u16,
    /// From datasheet, C4.
    pub temp_coef_pressure_offset: u16,
    /// From datasheet, C5.
    pub temp_ref: u16,
    /// From datasheet, C6.
    pub temp_coef_temp: u16,
}

pub struct Ms5611<I2C> 
where I2C: Read + Write
{
    address: u16,
    i2c: I2C,
    prom: Prom

}


impl Ms5611<I2C>{
        /// If i2c_addr is unspecified, 0x77 is used.
    /// The addr of the device is 0x77 if CSB is low / 0x76 if CSB is high.
    pub fn new(i2c_bus: I2C, i2c_addr: Option<u16>)
            -> Result<Ms5611, stm32f4xx_hal::i2c::Error> {
                let default_prom = Prom {
                    pressure_sensitivity: 0,
                    pressure_offset: 0,
                    temp_coef_pressure_sensitivity: 0,
                    temp_coef_pressure_offset: 0,
                    temp_ref: 0,
                    temp_coef_temp: 0,
                };
        let mut ms = Ms5611 {
                    address:  i2c_addr.unwrap_or(0x77),
                    i2c: i2c_bus,
                    prom: default_prom,
                };
        Self::read_prom(&mut ms)?;

        Ok(ms)
    }

    /// Triggers a hardware reset of the device.
    pub fn reset(&mut self) -> Result<(), stm32f4xx_hal::i2c::Error> {
        self.i2c.write(&[Ms5611Reg::Reset.addr()])?;
        // Haven't tested for the lower time bound necessary for the chip to
        // start functioning again. But, it does require some amount of sleep.
        Systick::delay(50_u32.millis().into());
        Ok(())
    }

    fn read_prom(&mut self) -> Result<(), stm32f4xx_hal::i2c::Error> {
        let mut crc_check = 0u16;

        // This is the CRC scheme in the MS5611 AN520 (Application Note)
        fn crc_accumulate_byte(crc_check: &mut u16, byte: u8) {
            *crc_check ^= byte as u16;
            for _ in 0..8 {
                if (*crc_check & 0x8000) > 0 {
                    *crc_check = (*crc_check << 1) ^ 0x3000;
                } else {
                    *crc_check = *crc_check << 1;
                }
            }
        }

        fn crc_accumulate_buf2(crc_check: &mut u16, buf: &[u8]) {
            crc_accumulate_byte(crc_check,buf[0]);
            crc_accumulate_byte(crc_check,buf[1]);
        }

        let mut buf: [u8; 2] = [0u8; 2];
        // Address reserved for manufacturer. We need it for the CRC.
        self.i2c.write(&[Ms5611Reg::Prom.addr()])?;
        self.i2c.read(&mut buf)?;
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 2])?;
        self.i2c.read(&mut buf)?;
        let pressure_sensitivity = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 4])?;
        self.i2c.read(&mut buf)?;
        let pressure_offset = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 6])?;
        self.i2c.read(&mut buf)?;
        let temp_coef_pressure_sensitivity = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 8])?;
        self.i2c.read(&mut buf)?;
        let temp_coef_pressure_offset = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 10])?;
        self.i2c.read(&mut buf)?;
        let temp_ref = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 12])?;
        self.i2c.read(&mut buf)?;
        let temp_coef_temp = BigEndian::read_u16(&mut buf);
        crc_accumulate_buf2(&mut crc_check, &buf);

        self.i2c.write(&[Ms5611Reg::Prom.addr() + 14])?;
        self.i2c.read(&mut buf)?;
        // CRC is only last 4 bits
        let crc = BigEndian::read_u16(&mut buf) & 0x000f;
        crc_accumulate_byte(&mut crc_check, buf[0]);
        crc_accumulate_byte(&mut crc_check, 0);

        crc_check = crc_check >> 12;

        if crc != crc_check {
            panic!("PROM CRC did not match: {} != {}", crc, crc_check);
        }

        self.prom = Prom {
            pressure_sensitivity,
            pressure_offset,
            temp_coef_pressure_sensitivity,
            temp_coef_pressure_offset,
            temp_ref,
            temp_coef_temp,
        };
        return Ok(());
    }


    pub async fn read_sample(&mut self, osr: Osr) -> Result<Ms5611Sample,stm32f4xx_hal::i2c::Error> {
        let mut buf = [0u8; 3];
        // Read the digital pressure value
        self.i2c.write(0x77,&[Ms5611Reg::D1.addr()+osr.addr_modifier()]).unwrap();
        Systick::delay(osr.get_delay().millis().into()).await;
        self.i2c.write(0x77,&[Ms5611Reg::AdcRead.addr()]).unwrap();
        self.i2c.read(0x77, &mut buf).unwrap();
        let mut d1 = BigEndian::read_u24(&buf);

        // Read the digital temperature value
        self.i2c.write(0x77,&[Ms5611Reg::D2.addr()+osr.addr_modifier()]).unwrap();
        Systick::delay(osr.get_delay().millis().into()).await;
        self.i2c.write(0x77,&[Ms5611Reg::AdcRead.addr()]).unwrap();
        self.i2c.read(0x77, &mut buf).unwrap();
        let mut d2 = BigEndian::read_u24(&buf) as i64;
        // defmt::info!("{:?}",d2);
            
            
        let dt = d2 - ((self.prom.temp_ref as i64) << 8);
        let mut temp: i32 = 2000 + (((dt as i64 * (self.prom.temp_coef_temp as i64)) >> 23) as i32);
        
        
        let mut offset: i64 = ((self.prom.pressure_offset as i64) << 16) + ((dt * (self.prom.temp_coef_pressure_offset as i64)) >> 7);
        let mut sens: i64 = ((self.prom.pressure_sensitivity as i64) << 15) + ((dt * (self.prom.temp_coef_pressure_sensitivity as i64)) >> 8);


        let mut t2 = 0i32;
        let mut off2 = 0i64;
        let mut sens2 = 0i64;
        //
        // Second order temperature compensation
        //

        // Low temperature (< 20C)
        if temp < 2000 {
            t2 = ((dt * dt) >> 31) as i32;
            off2 = ((5 * (temp - 2000).pow(2)) >> 1) as i64;
            sens2 = off2 >> 1;
        }

        // Very low temperature (< -15)
        if temp < -1500 {
            off2 += 7 * (temp as i64 + 1500).pow(2);
            sens2 += ((11 * (temp as i64 + 1500).pow(2)) >> 1) as i64;
            }

        temp -= t2;
        offset -= off2;
        sens -= sens2;
        let pressure: i32 = (((((d1 as i64) * sens) >> 21) - offset) >> 15) as i32;

        return Ok(Ms5611Sample {
            pressure_mbar: pressure as f32/100.0,
            temperature_c: temp as f32/100.0,
        });
    }
}

