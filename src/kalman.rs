use super::Error;
use na::{Matrix6, Vector6};
use nalgebra as na;
use num_traits::float::FloatCore;
// The nalgebra crate provides only up to 6D vectors and matrices.

/// A stack-allocated, 7-dimensional column vector.
type Vector7<T> = na::Matrix<T, na::U7, na::U1, na::ArrayStorage<T, 7, 1>>;
/// A stack-allocated, column-major, 7x7 square matrix.
type Matrix7<T> = na::Matrix<T, na::U7, na::U7, na::ArrayStorage<T, 7, 7>>;
/// A stack-allocated, column-major, 7x6 matrix.
type Matrix7x6<T> = na::Matrix<T, na::U7, na::U6, na::ArrayStorage<T, 7, 6>>;
/// A stack-allocated, column-major, 6x7 matrix.
type Matrix6x7<T> = na::Matrix<T, na::U6, na::U7, na::ArrayStorage<T, 6, 7>>;

// newtype pattern
#[derive(Debug, Default)]
pub struct StateVector(Vector6<f32>);

impl StateVector {
    pub fn lat(self) -> f32 {
        self.0[0]
    }

    pub fn lon(self) -> f32 {
        self.0[1]
    }

    pub fn alt(self) -> f32 {
        self.0[2]
    }

    pub fn v_lat(self) -> f32 {
        self.0[3]
    }

    pub fn v_lon(self) -> f32 {
        self.0[4]
    }

    pub fn v_alt(self) -> f32 {
        self.0[5]
    }
}

#[derive(Debug, Default)]
pub struct ObservationVector(Vector7<f32>);

impl ObservationVector {
    pub fn lat(self) -> f32 {
        self.0[0]
    }

    pub fn lon(self) -> f32 {
        self.0[1]
    }

    pub fn alt(self) -> f32 {
        self.0[2]
    }

    pub fn v_alt(self) -> f32 {
        self.0[3]
    }

    pub fn pres(self) -> f32 {
        self.0[4]
    }

    pub fn speed(self) -> f32 {
        self.0[5]
    }

    pub fn course(self) -> f32 {
        self.0[6]
    }
}

// Noise variance structure
pub struct Qnoise(StateVector);

// Default noise variance is 1.0
impl Default for Qnoise {
    fn default() -> Self {
        Self(StateVector(Vector6::from_element(1.0)))
    }
}

pub struct Rnoise(ObservationVector);

impl Default for Rnoise {
    fn default() -> Self {
        Self(ObservationVector(Vector7::from_element(1.0)))
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
    pub h: Matrix7x6<f32>,

    // the process noise covariance matrix
    pub q: Matrix6<f32>,

    // the measurement noise covariance matrix
    pub r: Matrix7<f32>,

    // the a priori covariance matrix
    pub p_m: Matrix6<f32>,

    // the a posteriori covariance matrix
    pub p: Matrix6<f32>,

    // the kalman gain matrix
    pub k: Matrix6x7<f32>,
}

impl KalmanFilter {
    pub fn new(q: Qnoise, r: Rnoise) -> Self {
        KalmanFilter {
            q: Matrix6::from_diagonal(&q.0 .0),
            r: Matrix7::from_diagonal(&r.0 .0),
            ..Default::default()
        }
    }

    // Add a measurement to the filter
    pub fn update(
        &mut self,
        z: &ObservationVector,
        r: Option<&Matrix7<f32>>,
        h: Option<&Matrix7x6<f32>>,
    ) -> Result<(), Error> {
        // If new linearization matrices are provided, use them
        self.h = *h.unwrap_or(&self.h);
        self.r = *r.unwrap_or(&self.r);

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

    pub fn predict(&mut self, a: Option<&Matrix6<f32>>, q: Option<&Matrix6<f32>>) {
        // If new linearization matrices are provided, use them
        self.a = *a.unwrap_or(&self.a);
        self.q = *q.unwrap_or(&self.q);

        // Predict the state
        self.x.0 = &self.a * &self.x.0;

        // Predict the covariance
        self.p_m = &self.a * &self.p * &self.a.transpose() + &self.q;
    }

    pub fn linearize_h(&mut self) {
        const R_STAR: f32 = 8.3144598;
        const T: f32 = 288.15;
        const G: f32 = 9.80665;
        const M: f32 = 0.0289644;
        const P: f32 = 101325.0;

        // convienence
        let x = &self.x;

        // Move this to default implementation
        *self.h.get_mut((4, 2)).unwrap() = -(R_STAR * T) / P * G * M;

        // TODO Måske skal der trækkes 2pi fra her
        *self.h.get_mut((5, 3)).unwrap() =
            -x.v_lon() / (x.v_lat().powi(2) * (x.v_lon().powi(2) / x.v_lat().powi(2) + 1.0));

        *self.h.get_mut((5, 4)).unwrap() =
            1.0 / (x.v_lat() * (x.v_lon().powi(2) / x.v_lat().powi(2) + 1.0));

        *self.h.get_mut((6, 3)).unwrap() =
            x.v_lat() / (x.v_lat().powi(2) + x.v_lon().powi(2)).sqrt();

        *self.h.get_mut((6, 4)).unwrap() =
            -x.v_lon() / (x.v_lat().powi(2) + x.v_lon().powi(2)).sqrt();
    }
}
