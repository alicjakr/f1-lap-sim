use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

pub struct Point {
    x: f64,
    y: f64,
}


pub fn load_track_geometry(path: &Path) -> Result<Vec<Point>, Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut points: Vec<Point> = Vec::new();

    let mut lines = reader.lines();
    lines.next(); // skip header

    for line in lines {
        let line = line?;
        let mut cols = line.split(',');
        let x = cols.next().ok_or("missing x")?.parse::<f64>()?;
        let y = cols.next().ok_or("missing y")?.parse::<f64>()?;
        points.push(Point {x, y});
    }

    Ok(points)

}


pub fn compute_arc_lengths(points: &[Point]) -> Vec<f64> {
    let mut distances = Vec::with_capacity(points.len());
    distances.push(0.0);

    // sum of Euclidean distances
    for i in 1..points.len() {
        distances.push(distances[i-1] + f64::hypot(points[i].x - points[i-1].x, points[i].y - points[i-1].y));
    }

    distances
}


pub fn resample(points: &[Point], arc_lengths: Vec<f64>, ds: f64) -> Vec<Point> {
    let mut coordinates: Vec<Point > = Vec::new();
    let mut j = 0;
    let n_output = (arc_lengths.last().unwrap() / ds).floor() as usize;

    for i in 0..n_output {
        let t = i as f64 * ds;
        while arc_lengths[j+1] < t || arc_lengths[j+1] == arc_lengths[j] {
            j += 1;
        }
        let alpha = (t - arc_lengths[j]) / (arc_lengths[j+1] - arc_lengths[j]);

        let x = points[j].x + alpha * (points[j+1].x - points[j].x);
        let y = points[j].y + alpha * (points[j+1].y - points[j].y);

        coordinates.push( Point {x, y} );
    }

    coordinates
}


pub fn compute_curvature(points: &[Point], ds: f64) -> Vec<f64> {
    let mut kappa: Vec<f64> = Vec::with_capacity(points.len());

    for i in 1..points.len()-1 {
        // first derivatives
        let f_x = (points[i+1].x - points[i-1].x) / (2.0 * ds);
        let f_y = (points[i+1].y - points[i-1].y) / (2.0 * ds);

        // second derivatives
        let d_x = (points[i+1].x - 2.0 * points[i].x + points[i-1].x) / ds.powi(2);
        let d_y = (points[i+1].y - 2.0 * points[i].y + points[i-1].y) / ds.powi(2);

        kappa.push((f_x*d_y - f_y*d_x) / (f_x.powi(2) + f_y.powi(2)).powf(1.5));
    }

    kappa.insert(0, kappa[0]);
    kappa.push(*kappa.last().unwrap());
    kappa
}