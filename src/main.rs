use std::path::Path;
use crate::solver::{backward_pass, corner_speed_limits, forward_pass, lap_time, CarParams, export_velocity_csv};
use crate::track::{build_spline, chord_length_params, export_curvature_csv, load_track_geometry, resample, smooth_curvature, Point};

mod track;
mod solver;

fn main() {
    let track_geometry: Vec<Point> = load_track_geometry(Path::new("data/track_geometry.csv")).unwrap();
    let t = chord_length_params(&track_geometry);
    let segments = build_spline(&track_geometry, &t);
    let (resampled, raw_curvature) = resample(&segments, 1.0);
    let curvature = smooth_curvature(&raw_curvature, 35);
    export_curvature_csv(&raw_curvature, &curvature, 1.0, Path::new("data/curvature_debug.csv")).unwrap();

    println!("Input point count: {}, resampled point count: {}, total track length: {}", track_geometry.len(), resampled.len(), resampled.len() as f64 * 1.0);
    println!("Minimum curvature: {}, maximum curvature: {}, average curvature: {}", curvature.iter().cloned().fold(f64::INFINITY, f64::min), curvature.iter().cloned().fold(f64::NEG_INFINITY, f64::max), curvature.iter().sum::<f64>() / curvature.len() as f64);

    let parameters = CarParams {
        mu_lat: 1.63,
        mu_lon: 1.5,
        a_brake: 40.0,
        c_l: 0.008,
        c_d: 0.0015,
        p_engine: 921.0,
    };
    // backward_pass/forward_pass are open (linear) sweeps: they never connect index n-1
    // back to index 0, even though the track itself is a closed loop. Solve over several
    // concatenated laps instead, so the artificial "cold start" at the very first sample
    // decays before the final lap, then keep only that final, periodic lap.
    let n = curvature.len();
    let laps = 3;
    let curvature_padded: Vec<f64> = curvature.iter().cycle().take(n * laps).cloned().collect();

    let corner_lims_padded = corner_speed_limits(&curvature_padded, &parameters);
    let mut v = corner_lims_padded.clone();
    loop {
        let v_back = backward_pass(&curvature_padded, &v, &parameters, 1.0);
        let v_fwd = forward_pass(&curvature_padded, &v_back, &parameters, 1.0);
        let max_change = v.iter().zip(&v_fwd).map(|(a, b)| (a - b).abs()).fold(0.0_f64, f64::max);
        v = v_fwd;
        if max_change < 1e-3 { break; }
    }

    let v_final = v[(laps - 1) * n..laps * n].to_vec();
    let v_prev_lap = &v[(laps - 2) * n..(laps - 1) * n];
    let seam_diff = v_final.iter().zip(v_prev_lap).map(|(a, b)| (a - b).abs()).fold(0.0_f64, f64::max);
    println!("Max diff between final lap and previous lap (periodicity check): {}", seam_diff);

    println!("Minimum velocity: {}, maximum velocity: {}, average velocity: {}", corner_lims_padded[..n].iter().cloned().fold(f64::INFINITY, f64::min), corner_lims_padded[..n].iter().cloned().fold(f64::NEG_INFINITY, f64::max), v_final.iter().sum::<f64>() / n as f64);

    let lap_time = lap_time(&v_final, 1.0);
    let minutes = (lap_time / 60.0) as u32;
    let seconds = lap_time % 60.0;
    println!("Lap time: {}:{:06.3}", minutes, seconds);

    export_velocity_csv(&v_final, 1.0, Path::new("data/simulated_lap.csv")).unwrap();
}