use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

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
