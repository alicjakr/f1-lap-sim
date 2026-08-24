use std::env;
use std::path::Path;
use crate::solver::{backward_pass, corner_speed_limits, forward_pass, lap_time, CarParams, export_velocity_csv};
use crate::track::{export_curvature_csv, fit_periodic_bspline, load_track_geometry, resample, total_length};

mod track;
mod solver;

fn main() {
    // Track slug, e.g. "singapore" or "suzuka" — matches the <slug>_ prefix that
    // python_scripts/export_track.py writes, so both tracks' data can coexist under data/.
    let track = env::args().nth(1).unwrap_or_else(|| "singapore".to_string());
    let geometry_path = format!("data/{}_track_geometry.csv", track);
    let curvature_debug_path = format!("data/{}_curvature_debug.csv", track);
    let simulated_lap_path = format!("data/{}_simulated_lap.csv", track);

    let (track_geometry, t) = load_track_geometry(Path::new(&geometry_path)).unwrap();
    let length = total_length(&track_geometry, &t);
    // One control point roughly every 25m: far fewer than the raw GPS point count, so the
    // fitted curve is structurally incapable of reproducing point-to-point GPS noise.
    let control_points = (length / 12.0).round() as usize;
    let smoother = fit_periodic_bspline(&track_geometry, &t, length, control_points, 1.0);
    let (resampled, curvature) = resample(&smoother, 1.0);
    export_curvature_csv(&curvature, 1.0, Path::new(&curvature_debug_path)).unwrap();

    println!("Input point count: {}, control points: {}, resampled point count: {}, total track length: {}", track_geometry.len(), control_points, resampled.len(), resampled.len() as f64 * 1.0);
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

    export_velocity_csv(&v_final, 1.0, Path::new(&simulated_lap_path)).unwrap();
}