mod kalman;

extern crate nalgebra as na;

use na::{SMatrix, SVector, Vector3, Vector, Vector2, Matrix2, Matrix2x3, Matrix4, OMatrix, DefaultAllocator, Dim, RealField};
use num_traits::identities::Zero;
use num_traits::Float;
type Matrix2x3f = SMatrix<f32, 2, 3>;
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
    let covariance = Matrix2::from_vec(vec![1.0, 0., 0., 1.]);
    let kalman_state = kalman::KalmanState{state, covariance};
    let a : SMatrix<f32, 4, 4> = SMatrix::zero();
    let b : SMatrix<f32, 4, 4> = SMatrix::zero();
    let c = mul(b, a);
    print!("{:?}", kalman_state);
    // let v2 = Vector<f32, Const<10>>;
    // let v = Vector3::new(1, 2, 3);
    // kalman::KalmanState{state: nalgebra::SVector<f64, 2>{}}
}