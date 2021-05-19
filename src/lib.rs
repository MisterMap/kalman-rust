// function generate_data(point_count, parameters::RandomMotionParameters{T}, time_delta::T) where {T}
//     state = parameters.initial_state
//     motion_distribution = Normal(0, parameters.motion_noise)
//     measurement_distribution = Normal(0, parameters.measurement_noise)
//     measurements = SVector{1, T}[]
//     states = SVector{2, T}[]
//     for _ in 1:point_count
//         acceleration = rand(motion_distribution)
//         state = SVector{2}(state[1] + state[2] * time_delta + acceleration * time_delta * time_delta / 2,
//          state[2] + acceleration * time_delta)
//         measurement = SVector{1}(state[1] + rand(measurement_distribution))
//         push!(measurements, measurement)
//         push!(states, state)
//     end
//     measurements, states
// end


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

// fn some() {
//     let time_step = 0.1;
//     let points_count=  500;
//     let times:Vec<f64> =  (0..points_count).map(|x:i32|->f64{x as f64 * time_step}).collect();
//     let initial_state : ndarray::Array1<f64> = ndarray::arr1(&[0., 0.]);
//     let parameters = RandomMotionParameters{
//         initial_state: ndarray::arr1(&[0., 0.]),
//         measurement_noise: 1.,
//         motion_noise: 1.
//     };
//     let (measurements, states):(Vec<ndarray::Array1<f64>>, Vec<ndarray::Array1<f64>>) = generate_data(points_count, RandomMotionParameters(1., 1., SVector{2}(0., 0.)), time_delta);
// }
