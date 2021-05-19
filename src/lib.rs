pub mod kalman;
pub use kalman::{KalmanState, KalmanFilter};

struct RandomMotionParameters {
    initial_state : ndarray::Array1<f64>,
    motion_noise : f64,
    measurement_noise : f64
}

use rand::distributions::{Normal, Distribution};

fn generate_data(point_count: i64, parameters : RandomMotionParameters, time_delta : f64)
    -> (Vec<ndarray::Array1<f64>>, Vec<ndarray::Array1<f64>>) {
    let state : ndarray::Array1<f64> = parameters.initial_state;
    let motion_distribution = Normal::new(0., parameters.motion_noise);
    let measurement_distribution = Normal::new(0., parameters.motion_noise);
    let mut measurements: Vec<ndarray::Array1<f64>> = Vec::new();
    let mut states: Vec<ndarray::Array1<f64>> = Vec::new();
    for _ in 0..point_count {
        let acceleration = motion_distribution.sample(&mut rand::thread_rng());
        let state: ndarray::Array1<f64> = ndarray::arr1(&[
            state[0] + state[1] * time_delta + acceleration * time_delta.powi(2) / 2.,
            state[1] + acceleration * time_delta
        ]);
        let measurement:ndarray::Array1<f64> = ndarray::arr1(&[
            measurement_distribution.sample(&mut rand::thread_rng())
        ]);
        measurements.push(measurement.clone());
        states.push(state.clone());
    }
    (measurements, states)
}
