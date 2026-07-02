use std::path::Path;
use crate::track::{build_spline, chord_length_params, load_track_geometry, resample, Point};

mod track;
mod solver;

fn main() {
    let track_geometry: Vec<Point> = load_track_geometry(Path::new("data/track_geometry.csv")).unwrap();
    let t = chord_length_params(&track_geometry);
    let segments = build_spline(&track_geometry, &t);
    let (resampled, curvature) = resample(&segments, 1.0);

    println!("Input point count: {}, resampled point count: {}, total track length: {}", track_geometry.len(), resampled.len(), resampled.len() as f64 * 1.0);
    println!("Minimum curvature: {}, maximum curvature: {}, average curvature: {}", curvature.iter().cloned().fold(f64::INFINITY, f64::min), curvature.iter().cloned().fold(f64::NEG_INFINITY, f64::max), curvature.iter().sum::<f64>() / curvature.len() as f64);
}
