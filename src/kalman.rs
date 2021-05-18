extern crate nalgebra as na;

use na::dimension::Dim;
use na::{DefaultAllocator, OVector, RealField, OMatrix};
use na::{Matrix2x1, Matrix2, Matrix1x2, Matrix1};
use num_traits::identities::One;

#[derive(Eq, PartialEq, Clone)]
pub struct KalmanState<T: RealField, N: Dim> where
    DefaultAllocator: na::allocator::Allocator<T, N>,
    DefaultAllocator: na::allocator::Allocator<T, N, N>
{
    pub state: OVector<T, N>,
    pub covariance: OMatrix<T, N, N>
}


pub struct KalmanFilter<T: RealField, N: Dim, K: Dim> where
    DefaultAllocator: na::allocator::Allocator<T, N, N>,
    DefaultAllocator: na::allocator::Allocator<T, K, N>,
    DefaultAllocator: na::allocator::Allocator<T, K, K>,
{
    pub motion_model: OMatrix<T, N, N>,
    pub motion_error_model: OMatrix<T, N, N>,
    pub measurement_model: OMatrix<T, K, N>,
    pub measurement_error_model :OMatrix<T, K, K>,
}

impl<T: RealField, N: Dim, K: Dim> KalmanFilter<T, N, K> where
    DefaultAllocator: na::allocator::Allocator<T, N, N>,
    DefaultAllocator: na::allocator::Allocator<T, K, N>,
    DefaultAllocator: na::allocator::Allocator<T, N, K>,
    DefaultAllocator: na::allocator::Allocator<T, K, K>,
    DefaultAllocator: na::allocator::Allocator<T, N>,
    DefaultAllocator: na::allocator::Allocator<T, K>,
    DefaultAllocator: na::allocator::Allocator<T, <K as na::DimMin<K>>::Output, K>,
    DefaultAllocator: na::allocator::Allocator<T, K, <K as na::DimMin<K>>::Output>,
    DefaultAllocator: na::allocator::Allocator<T, <K as na::DimMin<K>>::Output>,
    DefaultAllocator: na::allocator::Allocator<T, <<K as na::DimMin<K>>::Output as na::DimSub<na::Const<1_usize>>>::Output>,
    K: na::DimMin<K> + na::DimName,
    N: na::DimName,
    <K as na::DimMin<K>>::Output: na::DimSub<na::Const<1_usize>>
{
    pub fn filter_state(&self, measurement: OVector<T, K>,
                    previous_state: KalmanState<T, N>, eps:T) -> KalmanState<T, N> {
        // Prediction
        let state  =  &self.motion_model * &previous_state.state;
        let covariance = &self.motion_model *
            &previous_state.covariance * &self.motion_model.transpose() + &self.motion_error_model;

        // Filtration
        let error_covariance: OMatrix<T, K, K> = &self.measurement_model * &covariance *
            &self.measurement_model.transpose() + &self.measurement_error_model;

        let inverse_error_covariance:OMatrix<T, K, K> =
            match error_covariance.pseudo_inverse(eps)
        {
            Ok(x) => x,
            Err(..) => OMatrix::one(),
        };

        let kalman_gain = &covariance * &self.measurement_model.transpose() *
            inverse_error_covariance;

        let final_state = &state + &kalman_gain * (measurement - &self.measurement_model * &state);
        let final_covariance: OMatrix<T, N, N> = (OMatrix::one() as OMatrix<T, N, N> -
            kalman_gain * &self.measurement_model) * &covariance;
        return KalmanState{state: final_state, covariance: final_covariance};
    }

    pub fn filter_states(&self, measurements: Vec<OVector<T, K>>,
                    initial_state: KalmanState<T, N>, eps:T) -> Vec<KalmanState<T, N>> {
        let mut current_state = initial_state;
        let mut states: Vec<KalmanState<T, N>> = Vec::new();
        for measurement in measurements {
            current_state = self.filter_state(measurement, current_state, eps);
            states.push(current_state.clone())
        }
        states
    }
}

impl<T: std::fmt::Debug + RealField, N: Dim> std::fmt::Debug for KalmanState<T, N> where
    DefaultAllocator: na::allocator::Allocator<T, N, N>,
    DefaultAllocator: na::allocator::Allocator<T, N>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "State {:?} with covariance {:?}", &self.state, &self.covariance)
    }
}

pub fn make_simple_kalman_filter(time_delta: f64, motion_noise: f64,
                                 measurement_noise: f64) -> KalmanFilter<f64, na::U2, na::U1> {
    let motion_model: Matrix2<f64> = Matrix2::from_vec(vec![1., time_delta, 0., 1.]);

    let state_motion_error_model: Matrix2x1<f64> = Matrix2x1::from_vec(vec![time_delta * time_delta / 2., time_delta]);
    let motion_error_model = state_motion_error_model *
        state_motion_error_model.transpose() * motion_noise * motion_noise;

    let measurement_model: Matrix1x2<f64> = Matrix1x2::from_vec(vec![1.0, 0.0]);
    let measurement_error_model: Matrix1<f64> = Matrix1::from_vec(vec![measurement_noise * measurement_noise]);
    KalmanFilter{
        motion_model, motion_error_model, measurement_model, measurement_error_model
    }
}