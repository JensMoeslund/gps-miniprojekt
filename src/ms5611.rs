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
impl Ms5611{
    pub fn read_sample(&mut self, osr: Osr) -> Result<Ms5611Sample,stm32f4xx_hal::i2c::Error> {
            
            // Read the digital pressure value
            i2c.write(0x77,&[Ms5611Reg::D1.addr()+osr.addr_modifier()]).unwrap();
            Systick::delay(osr.get_delay().millis().into()).await;
            i2c.write(0x77,&[Ms5611Reg::AdcRead.addr()]).unwrap();
            i2c.read(0x77, &mut buf).unwrap();
            let mut d1 = BigEndian::read_u24(&buf);

            // Read the digital temperature value
            i2c.write(0x77,&[Ms5611Reg::D2.addr()+osr.addr_modifier()]).unwrap();
            Systick::delay(osr.get_delay().millis().into()).await;
            i2c.write(0x77,&[Ms5611Reg::AdcRead.addr()]).unwrap();
            i2c.read(0x77, &mut buf).unwrap();
            let mut d2 = BigEndian::read_u24(&buf) as i64;
            // defmt::info!("{:?}",d2);
            
            
            // let mut dt:i32 = d2 as i32 - ((c[4] as i32)*256);
            let dt = d2 - ((c[5] as i64) << 8);
            // defmt::info!("{:?}",dt);
            // let mut test = [0u8;8];
            // let mut temp = BigEndian::write_i64_into(&[(2000 + dt as i64 *(c[4] as i64)/2^23)],&mut test);
            let mut temp: i32 = 2000 + (((dt as i64 * (c[6] as i64)) >> 23) as i32);
            
            
            let mut offset: i64 = ((c[2] as i64) << 16) + ((dt * (c[4] as i64)) >> 7);
            let mut sens: i64 = ((c[1] as i64) << 15) + ((dt * (c[3] as i64)) >> 8);


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
                temperature_c: temperature as f32/100.0,
            });
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
