use std::error::Error;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

const G: f64 = 9.81;

#[derive(Clone)]
pub struct CarParams {
    pub mu_lat: f64,     // lateral friction coefficient
    pub mu_lon: f64,     // longitudinal friction coefficient (accel + braking)
    pub c_l: f64,        // downforce coefficient per unit mass (m⁻¹)
    pub c_d: f64,        // drag coefficient per unit mass (m⁻¹), DRS closed
    pub c_d_drs: f64,    // drag coefficient per unit mass (m⁻¹), DRS open (lower than c_d)
    pub p_engine: f64,   // engine power per unit mass (W/kg)
}


// Top speed the car can sustain in a straight line, from its own power/drag balance
// (a_available = P/v - c_d*v² = 0), rather than read off a real lap's telemetry.
pub fn top_speed(params: &CarParams) -> f64 {
    (params.p_engine / params.c_d).cbrt()
}

// Top speed with DRS open (lower drag, so higher than top_speed above) -- used to size the
// optimal.rs solvers' velocity upper bound so a DRS zone's real speed potential isn't
// clipped by a bound sized off the DRS-closed drag alone.
pub fn top_speed_drs(params: &CarParams) -> f64 {
    (params.p_engine / params.c_d_drs).cbrt()
}


pub fn corner_speed_limits(curvature: &[f64], params: &CarParams) -> Vec<f64> {
    let v_top = top_speed(params);
    let mut limits: Vec<f64> = vec![0.0; curvature.len()];

    for i in 0..curvature.len() {
        let denom: f64 = curvature[i].abs() - params.mu_lat * params.c_l;
        if denom <= 0.0 {
            limits[i] = v_top;
        } else {
            limits[i] = (params.mu_lat * G / denom).sqrt().min(v_top);
        }
    }

    limits
}

pub fn backward_pass(curvature: &[f64], v_corner: &[f64], params: &CarParams, ds: f64) -> Vec<f64> {
    let mut v = v_corner.to_vec();

    for i in (0..v_corner.len()-1).rev() {
        let g_eff = G + params.c_l * v[i].powi(2);
        let a_lat = v[i].powi(2) * curvature[i].abs();
        let ratio = (a_lat / (params.mu_lat * g_eff)).min(1.0);
        let a_lon = params.mu_lon * g_eff * (1.0 - ratio.powi(2)).sqrt();
        let a_brake_eff = (a_lon + params.c_d * v[i+1].powi(2)).max(0.0);
        v[i] = v[i].min((v[i+1].powi(2) + 2.0 * a_brake_eff * ds).sqrt())
    }

    v
}

pub fn forward_pass(curvature: &[f64], v_backward: &[f64], params: &CarParams, ds: f64) -> Vec<f64> {
    let mut v = v_backward.to_vec();

    for i in 1..v_backward.len() {
        let g_eff = G + params.c_l * v[i-1].powi(2);
        let a_lat = v[i-1].powi(2) * curvature[i-1].abs();
        let ratio = (a_lat / (params.mu_lat * g_eff)).min(1.0);
        let a_lon = params.mu_lon * g_eff * (1.0 - ratio.powi(2)).sqrt();
        let a_available = (a_lon.min(params.p_engine / v[i-1]) - params.c_d * v[i-1].powi(2)).max(0.0);
        v[i] = v[i].min((v[i-1].powi(2) + 2.0 * a_available * ds).sqrt())
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