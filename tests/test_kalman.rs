extern crate kalman_rust;
extern crate nalgebra as na;

use kalman_rust::{KalmanState, KalmanFilter};
use na::{Vector2, Vector1, Matrix2, Matrix1x2, Matrix1};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_filter() {
        let state = kalman_rust::KalmanState{
            state: Vector2::from_vec(vec![1.0, 1.0]),
            covariance: Matrix2::from_vec(vec![2.0, 0., 0., 2.])};
        let measurement = Vector1::from_vec(vec![1.]);
        let kalman_filter = kalman_rust::KalmanFilter{
            motion_model: Matrix2::from_vec(vec![1.0, 0., 0., 1.]),
            motion_error_model: Matrix2::from_vec(vec![1.0, 0., 0., 1.]),
            measurement_model: Matrix1x2::from_vec(vec![1.0, 0.]),
            measurement_error_model: Matrix1::from_vec(vec![1.0]),
        };
        let result = kalman_filter.filter_state(measurement, state, 1e-8);
        let wanted_result = kalman_rust::KalmanState{
            state: Vector2::from_vec(vec![1.0, 1.0]),
            covariance: Matrix2::from_vec(vec![0.75, 0., 0., 3.])};
        assert_eq!(wanted_result, result);
    }
}