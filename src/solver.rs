use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

const G: f64 = 9.81;

pub struct CarParams {
    pub mu_lat: f64,     // lateral friction coefficient
    pub mu_lon: f64,     // longitudinal friction coefficient (accel + braking)
    pub a_brake: f64,    // peak braking deceleration (m/s²)
    pub v_top: f64,      // hard speed cap (m/s)
    pub c_l: f64,  // downforce coefficient per unit mass (m⁻¹)
    pub c_d: f64,  // drag coefficient per unit mass (m⁻¹)
    pub p_engine: f64,
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
        let denom: f64 = curvature[i].abs() - params.mu_lat * params.c_l;
        if denom <= 0.0 {
            limits[i] = params.v_top;
        } else {
            limits[i] = (params.mu_lat * G / denom).sqrt().min(params.v_top);
        }
    }

    limits
}

pub fn backward_pass(curvature: &[f64], v_corner: &[f64], params: &CarParams, ds: f64) -> Vec<f64> {
    let mut v = v_corner.to_vec();

    for _ in 0..2 {
        for i in(0..v_corner.len()-1).rev() {
            let g_eff = G + params.c_l * v[i].powi(2);
            let a_lat = v[i].powi(2) * curvature[i].abs();
            let ratio = (a_lat / (params.mu_lat * g_eff)).min(1.0);
            let a_lon = params.mu_lon * g_eff * (1.0 - ratio.powi(2)).sqrt();
            // fixed minimum braking regardless of ratio
            let a_brake_eff = (a_lon.max(params.a_brake * 0.3) + params.c_d * v[i+1].powi(2)).max(0.0);            // speed at i can't be so high that you can't brake down to v[i+1] within ds metres
            v[i] = v[i].min((v[i+1].powi(2) + 2.0 * a_brake_eff * ds).sqrt())
        }
    }

    v
}

pub fn forward_pass(curvature: &[f64], v_backward: &[f64], params: &CarParams, ds: f64) -> Vec<f64> {
    let mut v = v_backward.to_vec();

    for _ in 0..2 {
        for i in 1..v_backward.len() {
            let g_eff = G + params.c_l * v[i-1].powi(2);
            let a_lat = v[i-1].powi(2) * curvature[i-1].abs();
            let ratio = (a_lat / (params.mu_lat * g_eff)).min(1.0);
            // at least 10% of traction capacity at all times
            let a_lon = (params.mu_lon * g_eff * (1.0 - ratio.powi(2)).sqrt()).max(params.mu_lon * g_eff * 0.1);
            let a_available = (a_lon.min(params.p_engine / v[i-1]) - params.c_d * v[i-1].powi(2)).max(0.0);

            v[i] = v[i].min((v[i-1].powi(2) + 2.0 * a_available * ds).sqrt())
        }
    }

    v
}


pub fn lap_time(velocity: &[f64], ds: f64) -> f64 {
    velocity.iter().map(|v| ds / v).sum()
}


pub fn export_velocity_csv(velocity: &[f64], ds: f64, path: &Path) -> Result<(), Box<dyn Error>> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writeln!(writer, "Distance,Speed")?;

    for i in 0..velocity.len() {
        let distance = i as f64 * ds;
        let speed = velocity[i] * 3.6;
        writeln!(writer, "{},{}", distance, speed)?;
    }

    Ok(())
}