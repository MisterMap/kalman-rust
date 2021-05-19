mod kalman;

extern crate nalgebra as na;

use na::{SMatrix, Vector2, OMatrix, Matrix2, DefaultAllocator, Dim, RealField, Vector1};
use num_traits::identities::{Zero, One};
use na::allocator::Allocator;
use rand::Rng;

fn mul<T: RealField, N: Dim>(v1: OMatrix<T, N, N>, v2: OMatrix<T, N, N>) -> OMatrix<T, N, N> where
    T: RealField,
    N: Dim,
    DefaultAllocator: Allocator<T, N, N>, {
    v1 * v2
}

fn main() {
    let kalman_filter = kalman::make_simple_kalman_filter(0.1, 1.0, 1.0);
    let initial_state: kalman::KalmanState<f64, na::U2> = kalman::KalmanState{
        state: Vector2::from_vec(vec![0., 0.]),
        covariance: Matrix2::from_vec(vec![1., 0., 0., 1.])
    };
    let mut rng = rand::thread_rng();
    let measurement_vectors: Vec<Vector1<f64>> = (0..500)
        .map(|_| -> Vector1<f64>{Vector1::from_vec(vec![rng.gen_range(0., 20.)])})
        .collect();
    let mut filtred_states: Vec<kalman::KalmanState<f64, na::U2>> = Vec::new();
    for i in 1..1000000{
        let  filtred_states = kalman_filter.filter_states(&measurement_vectors,
        initial_state.clone(), 1e-8);
    }
    println!{"Done"};
}