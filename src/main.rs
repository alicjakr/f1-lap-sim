use std::path::Path;
use crate::track::{compute_arc_lengths, compute_curvature, load_track_geometry, resample, Point};

mod track;

fn main() {
    let track_geometry: Vec<Point> = load_track_geometry(Path::new("data/track_geometry.csv")).unwrap();
    let arc_lengths = compute_arc_lengths(&track_geometry);
    let total_length = *arc_lengths.last().unwrap();
    let resampled = resample(&track_geometry, arc_lengths, 1.0);
    let curvature = compute_curvature(&resampled, 1.0);

    println!("Input point count: {}, resampled point count: {}, total track length: {}", track_geometry.len(), resampled.len(), total_length);
    println!("Minimum curvature: {}, maximum curvature: {}, average curvature: {}", curvature.iter().cloned().fold(f64::INFINITY, f64::min), curvature.iter().cloned().fold(f64::NEG_INFINITY, f64::max), curvature.iter().sum::<f64>() / curvature.len() as f64);
}
