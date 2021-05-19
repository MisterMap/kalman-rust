mod kalman;

extern crate nalgebra as na;

use na::{SMatrix, Vector2, OMatrix, Matrix2, DefaultAllocator, Dim, RealField};
use num_traits::identities::{Zero, One};
use na::allocator::Allocator;

fn mul<T: RealField, N: Dim>(v1: OMatrix<T, N, N>, v2: OMatrix<T, N, N>) -> OMatrix<T, N, N> where
    T: RealField,
    N: Dim,
    DefaultAllocator: Allocator<T, N, N>, {
    v1 * v2
}

fn main() {
    // let vector = SVector<f64, 2>::new();
    let state = Vector2::from_vec(vec![1.0, 2.0]);
    let covariance = Matrix2::from_vec(vec![1.0, 0.1, 0., 1.]);
    let kalman_state = kalman::KalmanState{state, covariance};
    let a : SMatrix<f32, 4, 4> = SMatrix::zero();
    let b : SMatrix<f32, 4, 4> = SMatrix::one();
    let c = mul(b, a);
    println!("{:?}", kalman_state);
    println!("{:?}", c);
}