use std::env;
use std::path::Path;
use crate::optimal::{coarsen_stride, resample_bounds, solve_min_time, solve_racing_line};
use crate::solver::{backward_pass, corner_speed_limits, forward_pass, lap_time, CarParams, export_velocity_csv};
use crate::track::{export_curvature_csv, export_racing_line_csv, fit_periodic_bspline, load_boundaries, load_drs_zones, load_track_geometry, offset_line, resample, total_length};

mod track;
mod solver;
mod optimal;

fn main() {
    // Track slug, e.g. "singapore" or "suzuka" — matches the <slug>_ prefix that
    // python_scripts/export_track.py writes, so both tracks' data can coexist under data/.
    let track = env::args().nth(1).unwrap_or_else(|| "singapore".to_string());
    // "racingline" (default, and the primary solver) runs the minimum-time collocation
    // solver in optimal.rs with a free lateral offset bounded by track width (see
    // solve_racing_line), requiring data/<slug>_track_boundaries.csv to already exist
    // (python_scripts/export_osm_boundaries.py). "optimal" runs the same collocation
    // solver pinned to the fixed FastF1 driven line (see solve_min_time) -- kept as the
    // fallback for a track with no boundary data, and as racingline's own correctness-check
    // baseline (n=0 is a feasible point of the racing-line problem, so its lap time must be
    // <= the fixed-line one; see the "racingline" branch below). The original two-pass
    // greedy sweep (a bang-bang always-brake/accelerate-at-the-ellipse-edge heuristic) has
    // been removed -- solve_min_time's real optimization strictly supersedes it.
    let mode = env::args().nth(2).unwrap_or_else(|| "racingline".to_string());
    // Only used in "optimal"/"racingline" modes: target collocation-point spacing in meters.
    let spacing: f64 = env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(25.0);
    let geometry_path = format!("data/{}_track_geometry.csv", track);
    let curvature_debug_path = format!("data/{}_curvature_debug.csv", track);
    let simulated_lap_optimal_path = format!("data/{}_simulated_lap_optimal.csv", track);
    let simulated_lap_racingline_path = format!("data/{}_simulated_lap_racingline.csv", track);
    let racingline_xy_path = format!("data/{}_racingline_path.csv", track);
    let boundaries_path = format!("data/{}_track_boundaries.csv", track);
    let drs_zones_path = format!("data/{}_drs_zones.csv", track);

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

    // DRS (rear wing flap) cuts drag by roughly 10-15% while open, per published estimates;
    // it never affects cornering since it's closed again before the braking zone. Applied as
    // a flat reduction on whichever downforce tier's Cd is already chosen above, in the
    // per-point drag profile built from data/<slug>_drs_zones.csv (see optimal::resample_drs)
    // -- not a fourth downforce tier of its own.
    const DRS_DRAG_REDUCTION: f64 = 0.88;

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
        c_d_drs: c_d * DRS_DRAG_REDUCTION,
        p_engine: 1015.0,
    };

    // Per-point DRS-open flag along the same driven lap the geometry above came from (see
    // export_track.py's drs_zones.csv) -- missing for a track with no DRS export (e.g. one
    // exported before this feature existed), in which case every point falls back to
    // params.c_d (DRS always closed).
    let (drs_s, drs_open) = load_drs_zones(Path::new(&drs_zones_path)).unwrap_or_else(|_| (Vec::new(), Vec::new()));
    if mode == "optimal" {
        let (v_final, obj_lap_time, ds_coarse) = solve_min_time(&curvature, 1.0, &parameters, spacing, &drs_s, &drs_open);
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
            solve_racing_line(&curvature, 1.0, &parameters, spacing, &bound_s, &n_left, &n_right, &drs_s, &drs_open);
        let minutes = (racing_line_time / 60.0) as u32;
        let seconds = racing_line_time % 60.0;
        println!("Lap time (racing line): {}:{:06.3}", minutes, seconds);

        let mean_abs_n = n_profile.iter().map(|n| n.abs()).sum::<f64>() / n_profile.len() as f64;
        let max_abs_n = n_profile.iter().cloned().fold(0.0_f64, |acc, n| acc.max(n.abs()));
        println!("Lateral offset used: mean |n| = {:.2} m, max |n| = {:.2} m", mean_abs_n, max_abs_n);

        // n=0 is a feasible point of this same problem (it's exactly the fixed-line
        // problem), so the racing line can never come out slower -- a useful built-in
        // correctness check, not just a comparison.
        let (_, fixed_line_time, _) = solve_min_time(&curvature, 1.0, &parameters, spacing, &drs_s, &drs_open);
        println!(
            "Fixed-line optimal lap time for comparison: {:.3}s (racing line should be <= this; delta = {:.3}s)",
            fixed_line_time, racing_line_time - fixed_line_time
        );

        export_velocity_csv(&v_final, ds_coarse, Path::new(&simulated_lap_racingline_path)).unwrap();

        // X/Y path for visualization: subsample the fine-resolution (ds=1m) resampled
        // centerline with the same stride solve_racing_line used to build its coarse
        // curvature grid, so index i here lines up with n_profile[i] exactly.
        let stride = coarsen_stride(1.0, spacing);
        let coarse_points: Vec<_> = resampled.iter().step_by(stride).cloned().collect();
        let x_ref: Vec<f64> = coarse_points.iter().map(|p| p.x).collect();
        let y_ref: Vec<f64> = coarse_points.iter().map(|p| p.y).collect();
        let (x_line, y_line) = offset_line(&coarse_points, &n_profile);

        let target_s: Vec<f64> = (0..coarse_points.len()).map(|i| i as f64 * ds_coarse).collect();
        let (n_left_coarse, n_right_coarse) = resample_bounds(&bound_s, &n_left, &n_right, &target_s);
        let (x_left, y_left) = offset_line(&coarse_points, &n_left_coarse);
        let (x_right, y_right) = offset_line(&coarse_points, &n_right_coarse);

        export_racing_line_csv(
            &x_ref, &y_ref, &x_line, &y_line, &x_left, &y_left, &x_right, &y_right,
            &n_profile, &v_final, ds_coarse, Path::new(&racingline_xy_path),
        ).unwrap();
        println!("Exported racing-line X/Y path -> {}", racingline_xy_path);

        return;
    }

    panic!("unrecognized mode {:?} -- expected \"optimal\" or \"racingline\"", mode);
}