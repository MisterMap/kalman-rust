extern crate nalgebra as na;

use na::{SMatrix, SVector};

use na::dimension::Dim;
use na::{DefaultAllocator, OVector, RealField, Unit, Vector2, Vector3, OMatrix};
use nalgebra::Field;
use num_traits::Float;

use std::fmt::Display;

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
    DefaultAllocator: na::allocator::Allocator<T, K, K>,
    DefaultAllocator: na::allocator::Allocator<T, N>,
    DefaultAllocator: na::allocator::Allocator<T, K>
{
    pub fn filter_state(&self, measurement: OVector<T, K>,
                    previous_state: KalmanState<T, N>) -> KalmanState<T, N> {
        let state  =  &self.motion_model * &previous_state.state;
        return KalmanState{state: previous_state.state, covariance: previous_state.covariance};
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