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


pub fn chord_length_params(points: &[Point]) -> Vec<f64> {
    let mut distances = Vec::with_capacity(points.len());
    distances.push(0.0);

    // sum of Euclidean distances
    for i in 1..points.len() {
        distances.push(distances[i - 1] + f64::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y));
    }

    distances
}


pub fn thomas(a: &[f64], b: &[f64], c: &[f64], r: &[f64]) -> Vec<f64> {
    let mut b_clone = b.to_vec();
    let mut r_clone = r.to_vec();
    let n = a.len() + 1;

    // forward sweep
    for i in 1..n {
        let w = a[i-1] / b_clone[i-1];
        b_clone[i] -= w * c[i-1];
        r_clone[i] -= w * r_clone[i-1];
    }

    // back substitution
    let mut x = vec![0.0; n];
    x[n-1] = r_clone[n-1] / b_clone[n-1];

    for i in (0..n-1).rev() {
        x[i] = (r_clone[i] - c[i] * x[i+1]) / b_clone[i];
    }

    x
}


pub fn cyclic_thomas(a: &[f64], b: &[f64], c: &[f64], r: &[f64], alpha: f64, beta: f64) -> Vec<f64> {
    let n = a.len() + 1;
    let gamma = -b[0];
    let mut b_prime = b.to_vec();
    b_prime[0] = b[0] - gamma;
    b_prime[n-1] = b[n-1] - alpha * beta / gamma;

    let mut u = vec![0.0; n];
    u[0] = gamma;
    u[n-1] = alpha;

    let y = thomas(a, &b_prime, c, r);
    let q = thomas(a, &b_prime, c, &u);
    let factor = (y[0] + (beta/gamma)*y[n-1]) / (1.0 + q[0] + (beta/gamma)*q[n-1]);

    let mut x = vec![0.0; n];

    for i in 0..n {
        x[i] = y[i] - factor * q[i];
    }

    x
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