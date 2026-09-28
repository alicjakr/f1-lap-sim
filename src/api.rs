// Shared pipeline behind both binaries: main.rs (CLI) and bin/web.rs (web server). Keeps
// the racing-line solve callable in one place so neither binary duplicates the physics.

use std::fs;
use std::path::Path;

use serde::Serialize;

use crate::optimal::{coarsen_periodic, resample_bounds, solve_min_time, solve_racing_line, SolveInputs};
use crate::solver::CarParams;
use crate::track::{
    fit_periodic_bspline, load_aero_params, load_boundaries, load_drs_zones,
    load_reference_lap, load_track_geometry, offset_line, resample, total_length, Point,
};

// (c_l, c_d), per unit mass -- Cl/Cd of 3.05/1.05, 2.47/0.85 and 2.03/0.70 respectively.
const HIGH_DOWNFORCE: (f64, f64) = (0.0036, 0.0012);
const MED_DOWNFORCE: (f64, f64) = (0.0029, 0.0010);
const LOW_DOWNFORCE: (f64, f64) = (0.0024, 0.00082);
const DRS_DRAG_REDUCTION: f64 = 0.88;

// Real 2018 qualifying pole time per track, in seconds: the outright fastest lap in Q by
// any driver (session.laps.pick_fastest()), fetched once via FastF1. A historical fact
// rather than derived data, so it lives here as a table instead of a data/ export -- the
// web UI needs it without a network round-trip.
//
// The same figures are fetched live by python_scripts/validate_pole_times.py, which is the
// authority: if the two ever disagree, FastF1 has corrected something upstream and these
// values are the stale copy.
const POLE_TIME_2018: &[(&str, f64)] = &[
    ("bahrain", 87.958),
    ("baku", 101.498),
    ("catalunya", 76.173),
    ("cota", 92.237),
    ("hockenheim", 71.212),
    ("hungaroring", 76.666),
    ("interlagos", 67.281),
    ("monaco", 70.810),
    ("montreal", 70.764),
    ("monza", 79.119),
    ("paulricard", 90.029),
    ("redbullring", 63.130),
    ("shanghai", 91.095),
    ("silverstone", 85.892),
    ("singapore", 96.015),
    ("spa", 101.501),
    ("suzuka", 87.760),
    ("yasmarina", 94.794),
];

pub fn pole_time_2018(track: &str) -> Option<f64> {
    POLE_TIME_2018.iter().find(|(slug, _)| *slug == track).map(|(_, t)| *t)
}

// Tracks with a boundaries CSV present but known-bad data, documented in
// results/DISCUSSION.md's "6 known limitations" table -- kept solvable rather than hidden
// (the CLI already ran them before this UI existed), but flagged so the UI can warn.
const KNOWN_DATA_ISSUES: &[(&str, &str)] = &[
    (
        "bahrain",
        "OSM boundary data is unusable here (the relation bundles multiple real track layouts, e.g. car vs. motorcycle circuit; registration residual 35-63 m). Racing line and lap time are not reliable.",
    ),
    (
        "catalunya",
        "OSM boundary data is unusable here (the relation bundles multiple real track layouts, e.g. car vs. motorcycle circuit; registration residual 35-63 m). Racing line and lap time are not reliable.",
    ),
    (
        "singapore",
        "A real corner's geometry exceeds the solver's steering-rate model limit here -- Ipopt may report success with an unreliable racing line.",
    ),
];

pub fn known_data_issue(track: &str) -> Option<&'static str> {
    KNOWN_DATA_ISSUES.iter().find(|(slug, _)| *slug == track).map(|(_, msg)| *msg)
}

#[derive(Serialize)]
pub struct TrackInfo {
    pub slug: String,
    pub known_issue: Option<&'static str>,
}

// mu_lat/mu_lon/p_engine are tire/powertrain properties, fixed across tracks (see
// README.md/DISCUSSION.md for sourcing). c_l/c_d are wing-level, per-circuit choices:
// prefer data/<slug>_aero_params.csv (python_scripts/derive_downforce.py) over the coarse
// 3-tier classification below.
pub fn load_car_params(track: &str) -> CarParams {
    let aero_params_path = format!("data/{}_aero_params.csv", track);
    let (c_l, c_d) = match track {
        "monaco" | "hungaroring" | "singapore" | "catalunya" => HIGH_DOWNFORCE,
        "baku" | "montreal" | "redbullring" | "spa" | "monza" => LOW_DOWNFORCE,
        _ => MED_DOWNFORCE,
    };
    let (c_l, c_d) = load_aero_params(Path::new(&aero_params_path)).unwrap_or((c_l, c_d));
    CarParams {
        mu_lat: 1.6,
        mu_lon: 1.55,
        c_l,
        c_d,
        c_d_drs: c_d * DRS_DRAG_REDUCTION,
        p_engine: 1015.0,
    }
}

/// Resample step for the fitted centerline; the solvers coarsen from this.
pub const RESAMPLE_DS_M: f64 = 1.0;

/// B-spline knot spacing and roughness penalty for the centerline fit. Checked by
/// split-half reproducibility (fit the even-numbered raw samples and the odd-numbered ones
/// separately, compare the two curvature profiles): these values sit at 2-4% disagreement
/// with physically sensible minimum corner radii, and corner curvature is insensitive to
/// the exact choice across the whole stable range. Lighter smoothing at knots this close to
/// the raw sample spacing (~6-9 m) is *not* stable -- it predicts held-out positions better
/// while turning the second derivative into noise (0.3 m "corners", and solves that fail
/// outright), which is the trap position-based cross-validation walks into.
const CONTROL_POINT_SPACING_M: f64 = 12.0;
const SMOOTHING_LAMBDA: f64 = 1.0;

pub struct TrackCurvature {
    pub resampled: Vec<Point>,
    pub curvature: Vec<f64>,
    pub input_points: usize,
    pub control_points: usize,
}

/// Loads a track's driven centerline and fits/resamples it into the curvature array both
/// solvers run on. Shared so the CLI and the web server fit the same curve the same way.
pub fn load_track_curvature(track: &str) -> Result<TrackCurvature, String> {
    let geometry_path = format!("data/{}_track_geometry.csv", track);
    let (track_geometry, t) = load_track_geometry(Path::new(&geometry_path))
        .map_err(|e| format!("failed to load track geometry for {:?}: {}", track, e))?;
    let length = total_length(&track_geometry, &t);
    let control_points = (length / CONTROL_POINT_SPACING_M).round() as usize;
    let smoother =
        fit_periodic_bspline(&track_geometry, &t, length, control_points, SMOOTHING_LAMBDA);
    let (resampled, curvature) = resample(&smoother, RESAMPLE_DS_M);
    Ok(TrackCurvature {
        resampled,
        curvature,
        input_points: track_geometry.len(),
        control_points,
    })
}

/// Peak curvature rate the real car actually achieved on this track, |dkappa/dt| in
/// 1/(m*s), measured on the solver's own grid from the reference lap's curvature and speed.
/// A point mass can re-aim itself instantly -- nothing in the grip limit stops it changing
/// path curvature between one collocation point and the next -- so a free line exploits
/// flicks no car could drive. This is the cheapest physical bound on that: the car may not
/// change curvature faster than the real one demonstrably did here. The 99th percentile
/// rather than the max, since the max is a single telemetry spike. Returns None when the
/// track has no reference lap, in which case the limit simply isn't applied.
pub fn steering_rate_limit(track: &str, coarse_curvature: &[f64], ds: f64) -> Option<f64> {
    let (ref_s, ref_v) = load_reference_lap(Path::new(&format!(
        "data/{}_reference_lap.csv",
        track
    )))
    .ok()?;
    if ref_s.len() < 2 {
        return None;
    }
    let n = coarse_curvature.len();
    let mut rates: Vec<f64> = (0..n)
        .map(|i| {
            let d_kappa = (coarse_curvature[(i + 1) % n] - coarse_curvature[i]).abs() / ds;
            let s = i as f64 * ds;
            let j = ref_s.partition_point(|&rs| rs <= s).clamp(1, ref_s.len() - 1);
            let (s_lo, s_hi) = (ref_s[j - 1], ref_s[j]);
            let f = if s_hi > s_lo { (s - s_lo) / (s_hi - s_lo) } else { 0.0 };
            d_kappa * (ref_v[j - 1] + f * (ref_v[j] - ref_v[j - 1]))
        })
        .collect();
    rates.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Some(rates[(n as f64 * 0.99) as usize])
}

#[derive(Serialize)]
pub struct RacingLineResult {
    pub track: String,
    pub spacing: f64,
    pub lap_time_s: f64,
    pub fixed_line_time_s: f64,
    pub real_lap_time_2018_s: Option<f64>,
    pub mean_abs_n: f64,
    pub max_abs_n: f64,
    pub s: Vec<f64>,
    pub x_ref: Vec<f64>,
    pub y_ref: Vec<f64>,
    pub x_line: Vec<f64>,
    pub y_line: Vec<f64>,
    pub x_left: Vec<f64>,
    pub y_left: Vec<f64>,
    pub x_right: Vec<f64>,
    pub y_right: Vec<f64>,
    pub n_profile: Vec<f64>,
    pub speed_kmh: Vec<f64>,
}

/// Runs the full racing-line pipeline for one track (geometry, boundaries, free-lateral-
/// offset solve, fixed-line comparison) -- the same steps main.rs's "racingline" mode runs,
/// extracted so both the CLI and the web server call the exact same code.
pub fn solve_racing_line_for_track(track: &str, spacing: f64) -> Result<RacingLineResult, String> {
    let boundaries_path = format!("data/{}_track_boundaries.csv", track);
    let drs_zones_path = format!("data/{}_drs_zones.csv", track);

    let geometry = load_track_curvature(track)?;
    let (resampled, curvature) = (geometry.resampled, geometry.curvature);

    let (bound_s, n_left, n_right) = load_boundaries(Path::new(&boundaries_path))
        .map_err(|e| format!("failed to load track boundaries for {:?}: {}", track, e))?;

    let params = load_car_params(track);
    let (drs_s, drs_open) =
        load_drs_zones(Path::new(&drs_zones_path)).unwrap_or_else(|_| (Vec::new(), Vec::new()));

    let (coarse_curvature, coarse_ds) = coarsen_periodic(&curvature, RESAMPLE_DS_M, spacing);
    let omega_max = steering_rate_limit(track, &coarse_curvature, coarse_ds);

    let inputs = SolveInputs {
        curvature: &curvature,
        full_ds: RESAMPLE_DS_M,
        params: &params,
        spacing,
        drs_s: &drs_s,
        drs_open: &drs_open,
        omega_max,
    };
    let (v_final, n_profile, lap_time_s, ds_coarse) =
        solve_racing_line(&inputs, &bound_s, &n_left, &n_right)
            .map_err(|e| format!("racing-line solve for {:?}: {}", track, e))?;

    let mean_abs_n = n_profile.iter().map(|n| n.abs()).sum::<f64>() / n_profile.len() as f64;
    let max_abs_n = n_profile.iter().cloned().fold(0.0_f64, |acc, n| acc.max(n.abs()));

    let (_, fixed_line_time_s, _) = solve_min_time(&inputs)
        .map_err(|e| format!("fixed-line solve for {:?}: {}", track, e))?;

    // Same grid the solver used, so index i here lines up with n_profile[i].
    let xs: Vec<f64> = resampled.iter().map(|p| p.x).collect();
    let ys: Vec<f64> = resampled.iter().map(|p| p.y).collect();
    let (x_ref, _) = coarsen_periodic(&xs, RESAMPLE_DS_M, spacing);
    let (y_ref, _) = coarsen_periodic(&ys, RESAMPLE_DS_M, spacing);
    let coarse_points: Vec<Point> = x_ref.iter().zip(&y_ref).map(|(&x, &y)| Point { x, y }).collect();
    let (x_line, y_line) = offset_line(&coarse_points, &n_profile);

    let target_s: Vec<f64> = (0..coarse_points.len()).map(|i| i as f64 * ds_coarse).collect();
    let (n_left_coarse, n_right_coarse) = resample_bounds(&bound_s, &n_left, &n_right, &target_s);
    let (x_left, y_left) = offset_line(&coarse_points, &n_left_coarse);
    let (x_right, y_right) = offset_line(&coarse_points, &n_right_coarse);

    let speed_kmh: Vec<f64> = v_final.iter().map(|v| v * 3.6).collect();
    let real_lap_time_2018_s = pole_time_2018(track);

    Ok(RacingLineResult {
        track: track.to_string(),
        spacing,
        lap_time_s,
        fixed_line_time_s,
        real_lap_time_2018_s,
        mean_abs_n,
        max_abs_n,
        s: target_s,
        x_ref,
        y_ref,
        x_line,
        y_line,
        x_left,
        y_left,
        x_right,
        y_right,
        n_profile,
        speed_kmh,
    })
}

/// Track slugs with racing-line-capable data (a boundaries CSV present under data/),
/// scanned rather than hardcoded so a newly-exported track shows up with no code change.
/// Each carries known_issue (see KNOWN_DATA_ISSUES) so the UI can warn on a track whose
/// boundary data is documented as unreliable, rather than silently hiding it.
pub fn available_tracks() -> Vec<TrackInfo> {
    let mut tracks: Vec<TrackInfo> = fs::read_dir("data")
        .into_iter()
        .flatten()
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| {
            let name = entry.file_name().into_string().ok()?;
            let slug = name.strip_suffix("_track_boundaries.csv")?.to_string();
            let known_issue = known_data_issue(&slug);
            Some(TrackInfo { slug, known_issue })
        })
        .collect();
    tracks.sort_by(|a, b| a.slug.cmp(&b.slug));
    tracks
}
