#![feature(test)]

extern crate kalman_rust as kalman;
extern crate nalgebra as na;
extern crate test;

use test::{Bencher};
use kalman::{KalmanState, KalmanFilter, make_simple_kalman_filter};
use na::{Vector2, Vector1, Matrix2, Matrix1x2, Matrix1};
use rand::Rng;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_filter() {
        let state = kalman::KalmanState{
            state: Vector2::from_vec(vec![1.0, 1.0]),
            covariance: Matrix2::from_vec(vec![2.0, 0., 0., 2.])};
        let measurement = Vector1::from_vec(vec![1.]);
        let kalman_filter = kalman::KalmanFilter{
            motion_model: Matrix2::from_vec(vec![1.0, 0., 0., 1.]),
            motion_error_model: Matrix2::from_vec(vec![1.0, 0., 0., 1.]),
            measurement_model: Matrix1x2::from_vec(vec![1.0, 0.]),
            measurement_error_model: Matrix1::from_vec(vec![1.0]),
        };
        let result = kalman_filter.filter_state(&measurement, &state, 1e-8);
        let wanted_result = kalman::KalmanState{
            state: Vector2::from_vec(vec![1.0, 1.0]),
            covariance: Matrix2::from_vec(vec![0.75, 0., 0., 3.])};
        assert_eq!(wanted_result, result);
    }

    #[bench]
    fn bench_xor_1000_ints(b: &mut Bencher) {
        let kalman_filter = kalman::make_simple_kalman_filter(0.1, 1.0, 1.0);
        let initial_state: kalman::KalmanState<f64, na::U2> = kalman::KalmanState{
            state: Vector2::from_vec(vec![0., 0.]),
            covariance: Matrix2::from_vec(vec![1., 0., 0., 1.])
        };
        let mut rng = rand::thread_rng();
        let measurement_vectors: Vec<Vector1<f64>> = (0..500)
            .map(|_| -> Vector1<f64>{Vector1::from_vec(vec![rng.gen_range(0., 20.)])})
            .collect();
        b.iter(|| {
            let n = test::black_box(10000);

            let mut filtred_states1: Vec<kalman::KalmanState<f64, na::U2>> = Vec::new();
            for i in 1..n{
              let  filtred_states1 = kalman_filter.filter_states(&measurement_vectors,
                initial_state.clone(), 1e-8);
            }
        })
    }
}