use std::env;
use std::path::Path;

use f1_lap_sim::{api, optimal, solver, track};

fn main() {
    // Track slug, e.g. "singapore" or "suzuka" — matches the <slug>_ prefix that
    // python_scripts/export_track.py writes, so both tracks' data can coexist under data/.
    let track_slug = env::args().nth(1).unwrap_or_else(|| "singapore".to_string());
    // "racingline" (default, primary solver) or "optimal" (fixed-line fallback and
    // racingline's correctness baseline) -- see README.md for the full CLI reference.
    let mode = env::args().nth(2).unwrap_or_else(|| "racingline".to_string());
    // Only used in "optimal"/"racingline" modes: target collocation-point spacing in meters.
    let spacing: f64 = env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(25.0);
    let curvature_debug_path = format!("data/{}_curvature_debug.csv", track_slug);
    let simulated_lap_optimal_path = format!("data/{}_simulated_lap_optimal.csv", track_slug);
    let simulated_lap_racingline_path = format!("data/{}_simulated_lap_racingline.csv", track_slug);
    let racingline_xy_path = format!("data/{}_racingline_path.csv", track_slug);
    let drs_zones_path = format!("data/{}_drs_zones.csv", track_slug);

    let geometry = api::load_track_curvature(&track_slug).unwrap_or_else(|e| panic!("{}", e));
    let (resampled, curvature) = (geometry.resampled, geometry.curvature);
    let ds = api::RESAMPLE_DS_M;
    track::export_curvature_csv(&curvature, ds, Path::new(&curvature_debug_path)).unwrap();

    println!("Input point count: {}, control points: {}, resampled point count: {}, total track length: {}", geometry.input_points, geometry.control_points, resampled.len(), resampled.len() as f64 * ds);
    println!("Minimum curvature: {}, maximum curvature: {}, average curvature: {}", curvature.iter().cloned().fold(f64::INFINITY, f64::min), curvature.iter().cloned().fold(f64::NEG_INFINITY, f64::max), curvature.iter().sum::<f64>() / curvature.len() as f64);

    // Car params (mass/tire/engine + per-circuit aero) -- see api::load_car_params and
    // README.md/DISCUSSION.md for sourcing.
    let parameters = api::load_car_params(&track_slug);

    // Per-point DRS-open flag along the same driven lap the geometry above came from (see
    // export_track.py's drs_zones.csv) -- missing for a track with no DRS export (e.g. one
    // exported before this feature existed), in which case every point falls back to
    // params.c_d (DRS always closed).
    let (drs_s, drs_open) = track::load_drs_zones(Path::new(&drs_zones_path)).unwrap_or_else(|_| (Vec::new(), Vec::new()));
    if mode == "optimal" {
        let (v_final, obj_lap_time, ds_coarse) = optimal::solve_min_time(&curvature, ds, &parameters, spacing, &drs_s, &drs_open)
            .unwrap_or_else(|e| panic!("{}", e));
        let minutes = (obj_lap_time / 60.0) as u32;
        let seconds = obj_lap_time % 60.0;
        println!("Lap time (optimal, objective value): {}:{:06.3}", minutes, seconds);
        let lap_time_check = solver::lap_time(&v_final, ds_coarse);
        println!("Lap time (recomputed from returned velocity profile): {:.3}s", lap_time_check);
        solver::export_velocity_csv(&v_final, ds_coarse, Path::new(&simulated_lap_optimal_path)).unwrap();
        return;
    }

    if mode == "racingline" {
        let result = api::solve_racing_line_for_track(&track_slug, spacing).unwrap_or_else(|e| panic!("{}", e));

        let minutes = (result.lap_time_s / 60.0) as u32;
        let seconds = result.lap_time_s % 60.0;
        println!("Lap time (racing line): {}:{:06.3}", minutes, seconds);
        println!("Lateral offset used: mean |n| = {:.2} m, max |n| = {:.2} m", result.mean_abs_n, result.max_abs_n);

        // n=0 is a feasible point of this same problem (it's exactly the fixed-line
        // problem), so the racing line can never come out slower -- a useful built-in
        // correctness check, not just a comparison.
        println!(
            "Fixed-line optimal lap time for comparison: {:.3}s (racing line should be <= this; delta = {:.3}s)",
            result.fixed_line_time_s, result.lap_time_s - result.fixed_line_time_s
        );

        let ds_coarse = result.s[1] - result.s[0];
        let velocity_ms: Vec<f64> = result.speed_kmh.iter().map(|v| v / 3.6).collect();
        solver::export_velocity_csv(&velocity_ms, ds_coarse, Path::new(&simulated_lap_racingline_path)).unwrap();

        track::export_racing_line_csv(
            &result.x_ref, &result.y_ref, &result.x_line, &result.y_line,
            &result.x_left, &result.y_left, &result.x_right, &result.y_right,
            &result.n_profile, &velocity_ms, ds_coarse, Path::new(&racingline_xy_path),
        ).unwrap();
        println!("Exported racing-line X/Y path -> {}", racingline_xy_path);

        return;
    }

    panic!("unrecognized mode {:?} -- expected \"optimal\" or \"racingline\"", mode);
}
