use std::env;
use std::path::Path;
use crate::optimal::{solve_min_time, solve_racing_line};
use crate::solver::{backward_pass, corner_speed_limits, forward_pass, lap_time, CarParams, export_velocity_csv};
use crate::track::{export_curvature_csv, fit_periodic_bspline, load_boundaries, load_track_geometry, resample, total_length};

mod track;
mod solver;
mod optimal;

fn main() {
    // Track slug, e.g. "singapore" or "suzuka" — matches the <slug>_ prefix that
    // python_scripts/export_track.py writes, so both tracks' data can coexist under data/.
    let track = env::args().nth(1).unwrap_or_else(|| "singapore".to_string());
    // "twopass" (default) runs the existing greedy sweep; "optimal" runs the minimum-time
    // collocation solver in optimal.rs instead, on a coarser grid (see solve_min_time);
    // "racingline" runs that same solver with a free lateral offset bounded by track width
    // (see solve_racing_line), requiring data/<slug>_track_boundaries.csv to already exist
    // (python_scripts/export_osm_boundaries.py).
    let mode = env::args().nth(2).unwrap_or_else(|| "twopass".to_string());
    // Only used in "optimal"/"racingline" modes: target collocation-point spacing in meters.
    let spacing: f64 = env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(25.0);
    let geometry_path = format!("data/{}_track_geometry.csv", track);
    let curvature_debug_path = format!("data/{}_curvature_debug.csv", track);
    let simulated_lap_path = format!("data/{}_simulated_lap.csv", track);
    let simulated_lap_optimal_path = format!("data/{}_simulated_lap_optimal.csv", track);
    let simulated_lap_racingline_path = format!("data/{}_simulated_lap_racingline.csv", track);
    let boundaries_path = format!("data/{}_track_boundaries.csv", track);

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

    // mu_lat/mu_lon/p_engine are tire and powertrain properties, not wing-angle choices, so
    // they stay fixed across tracks. mass 734 kg (2018 min car+driver weight, near-empty
    // quali fuel); mu_lat/mu_lon from published tire-only (aero-excluded) friction estimates
    // (1.4-1.8 / 1.5-1.6); p_engine = (625 kW ICE + 120 kW MGU-K peak) / 734 kg, no ERS
    // energy budget modeled (known idealization: real cars can't sustain this continuously,
    // only ~33s/lap of MGU-K boost). Braking capacity is derived purely from the friction
    // ellipse (mu_lon*g_eff), not a separate flat floor -- an earlier a_brake*0.3 floor was
    // dropped once the minimum-time solver (optimal.rs) showed it let the two-pass sweep
    // brake harder than the pure ellipse allows, particularly at corner entry where lateral
    // load is highest; it was a two-pass crutch, not real physics, so removed from both
    // solvers rather than kept as an inconsistency between them.
    //
    // c_l/c_d are wing-level choices real teams change per circuit, so they're derived
    // per track below rather than as one universal figure. Published Cl/Cd for two known
    // reference points (Monaco, max downforce: 2.89; Monza, min downforce: 2.98) show the
    // L/D ratio stays roughly constant (~2.9) across downforce levels -- what actually
    // changes is the absolute Cd. Three tiers, Cd scaled from Monaco/Monza's own historical
    // Cd (1.08 / 0.68) with Cl = Cd*2.9, A = 1.4 m², rho = 1.225 kg/m^3, c_x = 0.5*rho*Cx*A/m.
    const HIGH_DOWNFORCE: (f64, f64) = (0.0036, 0.0012); // Cd=1.05, Cl=3.05
    const MED_DOWNFORCE: (f64, f64) = (0.0029, 0.0010);  // Cd=0.85, Cl=2.47
    const LOW_DOWNFORCE: (f64, f64) = (0.0024, 0.00082); // Cd=0.70, Cl=2.03

    // 2018 calendar (21 rounds), tiered by circuit character: tight/technical -> high,
    // long-straight power circuits -> low, everything else -> medium. Monaco/Hungaroring/
    // Singapore/Monza/Spa/Baku are well-sourced as tier extremes; most "medium" placements
    // are standard paddock classification rather than individually re-derived. Mexico is a
    // special case folded into "medium" as an approximation: real air density at 2240m
    // altitude is ~77% of sea level, which this model doesn't account for separately from
    // the wing-level choice captured here.
    let (c_l, c_d) = match track.as_str() {
        "monaco" | "hungaroring" | "singapore" | "catalunya" => HIGH_DOWNFORCE,
        "baku" | "montreal" | "redbullring" | "spa" | "monza" => LOW_DOWNFORCE,
        "melbourne" | "bahrain" | "shanghai" | "paulricard" | "silverstone" | "hockenheim"
        | "sochi" | "suzuka" | "cota" | "mexico" | "interlagos" | "yasmarina" => MED_DOWNFORCE,
        _ => MED_DOWNFORCE, // fallback for unrecognized slugs
    };
    let parameters = CarParams {
        mu_lat: 1.6,
        mu_lon: 1.55,
        c_l,
        c_d,
        p_engine: 1015.0,
    };
    if mode == "optimal" {
        let (v_final, obj_lap_time, ds_coarse) = solve_min_time(&curvature, 1.0, &parameters, spacing);
        let minutes = (obj_lap_time / 60.0) as u32;
        let seconds = obj_lap_time % 60.0;
        println!("Lap time (optimal, objective value): {}:{:06.3}", minutes, seconds);
        let lap_time_check = lap_time(&v_final, ds_coarse);
        println!("Lap time (recomputed from returned velocity profile): {:.3}s", lap_time_check);
        export_velocity_csv(&v_final, ds_coarse, Path::new(&simulated_lap_optimal_path)).unwrap();
        return;
    }

    if mode == "racingline" {
        let (bound_s, n_left, n_right) = load_boundaries(Path::new(&boundaries_path)).unwrap();
        let (v_final, n_profile, racing_line_time, ds_coarse) =
            solve_racing_line(&curvature, 1.0, &parameters, spacing, &bound_s, &n_left, &n_right);
        let minutes = (racing_line_time / 60.0) as u32;
        let seconds = racing_line_time % 60.0;
        println!("Lap time (racing line): {}:{:06.3}", minutes, seconds);

        let mean_abs_n = n_profile.iter().map(|n| n.abs()).sum::<f64>() / n_profile.len() as f64;
        let max_abs_n = n_profile.iter().cloned().fold(0.0_f64, |acc, n| acc.max(n.abs()));
        println!("Lateral offset used: mean |n| = {:.2} m, max |n| = {:.2} m", mean_abs_n, max_abs_n);

        // n=0 is a feasible point of this same problem (it's exactly the fixed-line
        // problem), so the racing line can never come out slower -- a useful built-in
        // correctness check, not just a comparison.
        let (_, fixed_line_time, _) = solve_min_time(&curvature, 1.0, &parameters, spacing);
        println!(
            "Fixed-line optimal lap time for comparison: {:.3}s (racing line should be <= this; delta = {:.3}s)",
            fixed_line_time, racing_line_time - fixed_line_time
        );

        export_velocity_csv(&v_final, ds_coarse, Path::new(&simulated_lap_racingline_path)).unwrap();
        return;
    }

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