// Shared pipeline behind both binaries: main.rs (CLI) and bin/web.rs (web server). Keeps
// the racing-line solve callable in one place so neither binary duplicates the physics.

use std::fs;
use std::path::Path;

use serde::Serialize;

use crate::optimal::{coarsen_stride, resample_bounds, solve_min_time, solve_racing_line};
use crate::solver::CarParams;
use crate::track::{
    fit_periodic_bspline, load_aero_params, load_boundaries, load_drs_zones,
    load_track_geometry, offset_line, resample, total_length,
};

const HIGH_DOWNFORCE: (f64, f64) = (0.0036, 0.0012); // Cd=1.05, Cl=3.05
const MED_DOWNFORCE: (f64, f64) = (0.0029, 0.0010); // Cd=0.85, Cl=2.47
const LOW_DOWNFORCE: (f64, f64) = (0.0024, 0.00082); // Cd=0.70, Cl=2.03
const DRS_DRAG_REDUCTION: f64 = 0.88;

// Real 2018 qualifying pole time per track (session.laps.pick_fastest() in Q, fetched via
// FastF1 -- same source/methodology as python_scripts/validate_pole_times.py), in seconds.
// Historical fact, not derived data, so it's a static table rather than a data/ export.
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
    let geometry_path = format!("data/{}_track_geometry.csv", track);
    let boundaries_path = format!("data/{}_track_boundaries.csv", track);
    let drs_zones_path = format!("data/{}_drs_zones.csv", track);

    let (track_geometry, t) = load_track_geometry(Path::new(&geometry_path))
        .map_err(|e| format!("failed to load track geometry for {:?}: {}", track, e))?;
    let length = total_length(&track_geometry, &t);
    let control_points = (length / 12.0).round() as usize;
    let smoother = fit_periodic_bspline(&track_geometry, &t, length, control_points, 1.0);
    let (resampled, curvature) = resample(&smoother, 1.0);

    let (bound_s, n_left, n_right) = load_boundaries(Path::new(&boundaries_path))
        .map_err(|e| format!("failed to load track boundaries for {:?}: {}", track, e))?;

    let params = load_car_params(track);
    let (drs_s, drs_open) =
        load_drs_zones(Path::new(&drs_zones_path)).unwrap_or_else(|_| (Vec::new(), Vec::new()));

    let (v_final, n_profile, lap_time_s, ds_coarse) = solve_racing_line(
        &curvature, 1.0, &params, spacing, &bound_s, &n_left, &n_right, &drs_s, &drs_open,
    );

    let mean_abs_n = n_profile.iter().map(|n| n.abs()).sum::<f64>() / n_profile.len() as f64;
    let max_abs_n = n_profile.iter().cloned().fold(0.0_f64, |acc, n| acc.max(n.abs()));

    let (_, fixed_line_time_s, _) =
        solve_min_time(&curvature, 1.0, &params, spacing, &drs_s, &drs_open);

    let stride = coarsen_stride(1.0, spacing);
    let coarse_points: Vec<_> = resampled.iter().step_by(stride).cloned().collect();
    let x_ref: Vec<f64> = coarse_points.iter().map(|p| p.x).collect();
    let y_ref: Vec<f64> = coarse_points.iter().map(|p| p.y).collect();
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
