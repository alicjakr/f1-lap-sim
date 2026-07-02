use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

const G: f64 = 9.81;

pub struct CarParams {
    pub mu_lat: f64,     // lateral friction coefficient
    pub mu_lon: f64,     // longitudinal friction coefficient (accel + braking)
    pub a_max: f64,      // peak longitudinal acceleration (m/s²)
    pub a_brake: f64,    // peak braking deceleration (m/s²)
    pub v_top: f64,      // hard speed cap (m/s)
}


pub fn load_top_speed(path: &Path) -> Result<f64, Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut top_speed: f64 = 0.0;

    let mut lines = reader.lines();
    lines.next();

    for line in lines {
        let line = line?;
        let mut cols = line.split(',');
        cols.next();
        let speed: f64 = cols.next().ok_or("missing speed")?.parse()?;
        if speed > top_speed {
            top_speed = speed;
        }
    }

    Ok(top_speed / 3.6)
}


pub fn corner_speed_limits(curvature: &[f64], params: &CarParams) -> Vec<f64> {
    let mut limits: Vec<f64> = vec![0.0; curvature.len()];

    for i in 0..curvature.len() {
        limits[i] = (params.mu_lat * G / curvature[i].abs()).sqrt().min(params.v_top)
    }

    limits
}

pub fn backward_pass(v_corner: &[f64], params: &CarParams, ds: f64) -> Vec<f64> {
    let mut v = v_corner.to_vec();

    for _ in 0..2 {
        for i in(0..v_corner.len()-1).rev() {
            // speed at i can't be so high that you can't brake down to v[i+1] within ds metres
            v[i] = v[i].min((v[i+1].powi(2) + 2.0 * params.a_brake * ds).sqrt())
        }
    }

    v
}

pub fn forward_pass(v_backward: &[f64], params: &CarParams, ds: f64) -> Vec<f64> {
    let mut v = v_backward.to_vec();

    for _ in 0..2 {
        for i in 1..v_backward.len() {
            v[i] = v[i].min((v[i-1].powi(2) + 2.0 * params.a_max * ds).sqrt())
        }
    }

    v
}


pub fn lap_time(velocity: &[f64], ds: f64) -> f64 {
    velocity.iter().map(|v| ds / v).sum()
}