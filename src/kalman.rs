use super::Error;
use super::SensorData;
use defmt::Format;
use na::{Matrix6, Vector6};
use nalgebra as na;
use nalgebra::ComplexField;

// The nalgebra crate provides only up to 6D vectors and matrices.
/// A stack-allocated, 8-dimensional column vector.
type Vector8<T> = na::Matrix<T, na::U8, na::U1, na::ArrayStorage<T, 8, 1>>;
/// A stack-allocated, column-major, 8x8 square matrix.
pub type Matrix8<T> = na::Matrix<T, na::U8, na::U8, na::ArrayStorage<T, 8, 8>>;
/// A stack-allocated, column-major, 8x6 matrix.
type Matrix8x6<T> = na::Matrix<T, na::U8, na::U6, na::ArrayStorage<T, 8, 6>>;
/// A stack-allocated, column-major, 6x8 matrix.
type Matrix6x8<T> = na::Matrix<T, na::U6, na::U8, na::ArrayStorage<T, 6, 8>>;

// newtype pattern
#[derive(Debug, Default)]
pub struct StateVector(Vector6<f32>);

impl StateVector {
    pub fn lat(&self) -> &f32 {
        &self.0[0]
    }

    pub fn lon(&self) -> &f32 {
        &self.0[1]
    }

    pub fn alt(&self) -> &f32 {
        &self.0[2]
    }

    pub fn v_lat(&self) -> &f32 {
        &self.0[3]
    }

    pub fn v_lon(&self) -> &f32 {
        &self.0[4]
    }

    pub fn v_alt(&self) -> &f32 {
        &self.0[5]
    }
}

impl Default for StateVector {
    fn default() -> Self {
        Self(Vector6::from_element(1.0))
    }
}

impl Format for StateVector {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "{{'lat': {}, 'lon': {}, 'alt': {}, 'v_lat': {}, 'v_lon': {}, 'v_alt': {}}}",
            self.lat(),
            self.lon(),
            self.alt(),
            self.v_lat(),
            self.v_lon(),
            self.v_alt(),
        );
    }
}

#[derive(Debug, Default)]
pub struct ObservationVector(Vector8<f32>);

impl ObservationVector {
    pub fn lat(&self) -> &f32 {
        &self.0[0]
    }

    pub fn lon(&self) -> &f32 {
        &self.0[1]
    }

    pub fn alt_gnss(&self) -> &f32 {
        &self.0[2]
    }

    pub fn v_alt_gnss(&self) -> &f32 {
        &self.0[3]
    }

    pub fn alt_pres(&self) -> &f32 {
        &self.0[4]
    }

    pub fn v_alt_pres(&self) -> &f32 {
        &self.0[5]
    }

    pub fn v_lat(&self) -> &f32 {
        &self.0[6]
    }

    pub fn v_lon(&self) -> &f32 {
        &self.0[7]
    }
}

impl Default for ObservationVector {
    fn default() -> Self {
        Self(Vector8::from_element(1.0))
    }
}

impl Format for ObservationVector {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{{'lat': {}, 'lon': {}, 'alt_gnss': {}, 'v_alt_gnss': {}, 'alt_pres': {}, 'v_alt_pres': {}, 'v_lat': {}, 'v_lon': {}}}",
            self.lat(),
            self.lon(),
            self.alt_gnss(),
            self.v_alt_gnss(),
            self.alt_pres(),
            self.v_alt_pres(),
            self.v_lat(),
            self.v_lon(),
        );
    }
}

impl ObservationVector {
    fn new(
        lat: f32,
        lon: f32,
        alt_gnss: f32,
        v_alt_gnss: f32,
        alt_pres: f32,
        v_alt_pres: f32,
        v_lat: f32,
        v_lon: f32,
    ) -> Self {
        Self(Vector8::from_column_slice(&[
            lat, lon, alt_gnss, v_alt_gnss, alt_pres, v_alt_pres, v_lat, v_lon,
        ]))
    }

    pub fn update(&mut self, data: SensorData, delta_t: f32) -> () {
        *self = {
            const R_0: f32 = 8.3144598;
            const P_0: f32 = 101325.0;
            const T_0: f32 = 288.15;
            const G: f32 = 9.80665;
            const M: f32 = 0.0289644;

            // Shortcut to the GNSS data
            let gnss = data.gnss_location;
            let course_rad = gnss.course.unwrap_or(0.0).to_radians();

            let alt_gnss = gnss.altitude.unwrap_or(0.0);
            let alt_pres = -(((data.ms5611_data.pressure_pa() / P_0).ln() * T_0 * R_0) / (G * M));

            let v_lat = course_rad.sin() * gnss.speed.unwrap_or(0.0);
            let v_lon = course_rad.cos() * gnss.speed.unwrap_or(0.0);

            ObservationVector::new(
                gnss.latitude as f32,
                gnss.longitude as f32,
                alt_gnss,
                self.alt_gnss() - alt_gnss / delta_t,
                alt_pres,
                self.alt_pres() - alt_pres / delta_t,
                v_lat,
                v_lon,
            )
        }
    }
}

// Kalman filter struct
#[derive(Debug, Default)]
pub struct KalmanFilter {
    // the state vector
    pub x: StateVector,

    // the observation vector
    pub z_tilde: ObservationVector,

    // the state transition matrix
    pub a: Matrix6<f32>,

    // the output gain matrix
    pub h: Matrix8x6<f32>,

    // the process noise covariance matrix
    pub q: Matrix6<f32>,

    // the measurement noise covariance matrix
    pub r: Matrix8<f32>,

    // the a priori covariance matrix
    pub p_m: Matrix6<f32>,

    // the a posteriori covariance matrix
    pub p: Matrix6<f32>,

    // the kalman gain matrix
    pub k: Matrix6x8<f32>,
}

impl KalmanFilter {
    pub fn new(q: Matrix6<f32>, r: Matrix8<f32>, delta_t: f32) -> Self {
        KalmanFilter {
            q,
            r,
            h: {
                let mut h = Matrix8x6::zeros();
                h[(0, 0)] = 1.0; //[1, 0, 0, 0, 0, 0],
                h[(1, 1)] = 1.0; //[0, 1, 0, 0, 0, 0],
                h[(2, 2)] = 1.0; //[0, 0, 1, 0, 0, 0],
                h[(3, 5)] = 1.0; //[0, 0, 0, 0, 0, 1],
                h[(4, 2)] = 1.0; //[0, 0, 1, 0, 0, 0],
                h[(5, 5)] = 1.0; //[0, 0, 0, 0, 0, 1],
                h[(6, 3)] = 1.0; //[0, 0, 0, 1, 0, 0],
                h[(7, 4)] = 1.0; //[0, 0, 0, 0, 1, 0],
                h
            },
            a: {
                let mut a = Matrix6::identity();
                a[(0, 3)] = delta_t;
                a[(1, 4)] = delta_t;
                a[(2, 5)] = delta_t;
                a
            },
            ..Default::default()
        }
    }

    // Add a measurement to the filter
    pub fn update(&mut self, z: &ObservationVector) -> Result<(), Error> {
        // measurement error
        self.z_tilde.0 = z.0 - &self.h * &self.x.0;

        // P * H^T (used twice)
        let pht = &self.p_m * &self.h.transpose();

        // System uncertainty
        let s = &self.h * &pht + &self.r;
        let si = s.try_inverse().ok_or(Error::KalmanNotInvertible)?;

        // kalman gain
        self.k = &pht * &si;

        // update state estimate
        self.x.0 += &self.k * &self.z_tilde.0;

        // I - K * H (used twice)
        let ikh = Matrix6::identity() - &self.k * &self.h;

        // update covariance estimate
        self.p = &ikh * &self.p_m * &ikh.transpose() + &self.k * &self.r * &self.k.transpose();

        Ok(())
    }

    pub fn predict(&mut self) -> &StateVector {
        // Predict the state
        self.x.0 = &self.a * &self.x.0;

        // Predict the covariance
        self.p_m = &self.a * &self.p * &self.a.transpose() + &self.q;

        &self.x
    }
}
