use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

pub struct Point {
    x: f64,
    y: f64,
}

pub struct SplineSegment {
    pub ax: f64,
    pub bx: f64,
    pub cx: f64,
    pub dx: f64,               // cubic coefficients for x
    pub ay: f64,
    pub by: f64,
    pub cy: f64,
    pub dy: f64,               // cubic coefficients for y
    pub t0: f64,               // parameter value at segment start
    pub h: f64,                // segment length in parameter space
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


pub fn build_spline(points: &[Point], t: &[f64]) -> Vec<SplineSegment> {
    let n = points.len();
    let mut b = vec![0.0; n];
    let mut h = vec![0.0; n];
    let total_chord_length = t[n-1] + f64::hypot(points[0].x - points[n-1].x, points[0].y - points[n-1].y);

    for i in 0..n-1 {
        h[i] = t[i+1] - t[i];
    }
    h[n-1] = total_chord_length - t[n-1];

    b[0] = 2.0 * (h[n-1] + h[0]);

    for i in 1..n {
        b[i] = 2.0 * (h[i-1] + h[i]);
    }

    let mut a = vec![0.0; n-1];
    let mut c = vec![0.0; n-1];

    for i in 0..n-1 {
        a[i] = h[i];
        c[i] = h[i];
    }

    let alpha = h[n-1];
    let beta = h[n-1];

    let mut r_x = vec![0.0; n];
    let mut r_y = vec![0.0; n];

    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = (i+1) % n;
        r_x[i] = 6.0 *  ((points[next].x - points[i].x) / h[i] - (points[i].x - points[prev].x) / h[prev]);
        r_y[i] = 6.0 *  ((points[next].y - points[i].y) / h[i] - (points[i].y - points[prev].y) / h[prev]);
    }

    let m_x = cyclic_thomas(&a, &b, &c, &r_x, alpha, beta);
    let m_y = cyclic_thomas(&a, &b, &c, &r_y, alpha, beta);

    let mut segments: Vec<SplineSegment> = Vec::new();
    for i in 0..n {
        let next = (i+1) % n;
        segments.push(SplineSegment {
            ax: points[i].x,
            bx: (points[next].x - points[i].x) / h[i] - h[i] * (2.0 * m_x[i] + m_x[next]) / 6.0,
            cx: m_x[i] / 2.0,
            dx: (m_x[next] - m_x[i]) / (6.0 * h[i]),
            ay: points[i].y,
            by: (points[next].y - points[i].y) / h[i] - h[i] * (2.0 * m_y[i] + m_y[next]) / 6.0,
            cy: m_y[i] / 2.0,
            dy: (m_y[next] - m_y[i]) / (6.0 * h[i]),
            t0: t[i],
            h: h[i],
        });
    }

    segments
}


fn eval_spline(segments: &[SplineSegment], t: f64) -> Point {
    let i = segments.partition_point(|seg| seg.t0 <= t).saturating_sub(1);
    let s = t - segments[i].t0;

    let x =  segments[i].ax + segments[i].bx * s + segments[i].cx * s.powi(2) + segments[i].dx * s.powi(3);
    let y =  segments[i].ay + segments[i].by * s + segments[i].cy * s.powi(2) + segments[i].dy * s.powi(3);

    Point { x, y }
}

fn eval_spline_deriv(segments: &[SplineSegment], t: f64) -> (f64, f64) {
    let i = segments.partition_point(|seg| seg.t0 <= t).saturating_sub(1);
    let s = t - segments[i].t0;

    let dx = segments[i].bx + 2.0 * segments[i].cx * s + 3.0 * segments[i].dx * s.powi(2);
    let dy = segments[i].by + 2.0 * segments[i].cy * s + 3.0 * segments[i].dy * s.powi(2);

    (dx, dy)
}


fn build_arc_length_table(segments: &[SplineSegment], steps_per_segment: usize) -> Vec<(f64, f64)> {
    let mut table: Vec<(f64, f64)> = Vec::with_capacity(segments.len() * steps_per_segment + 1);
    table.push((segments[0].t0, 0.0));

    let mut cumulative_s: f64 = 0.0;

    for seg in segments {
        let width = seg.h / steps_per_segment as f64;
        for i in 0..steps_per_segment {
            let t_a = seg.t0 + i as f64 * width;
            let t_b = t_a + width;
            let t_m = (t_a + t_b) / 2.0;
            let (da_x, da_y) = eval_spline_deriv(segments, t_a);
            let (db_x, db_y) = eval_spline_deriv(segments, t_b);
            let (dm_x, dm_y) = eval_spline_deriv(segments, t_m);

            let speed_a = f64::hypot(da_x, da_y);
            let speed_b = f64::hypot(db_x, db_y);
            let speed_m = f64::hypot(dm_x, dm_y);

            let arc = (t_b - t_a) / 6.0 * (speed_a + 4.0 * speed_m + speed_b);
            cumulative_s += arc;
            table.push((t_b, cumulative_s));
        }
    }

    table
}


pub fn eval_spline_curvature(segments: &[SplineSegment], t: f64) -> f64 {
    let i = segments.partition_point(|seg| seg.t0 <= t).saturating_sub(1);
    let seg = &segments[i];
    let s = t - segments[i].t0;

    let f_x = seg.bx + 2.0 * seg.cx * s + 3.0 * seg.dx * s.powi(2);
    let f_y = seg.by + 2.0 * seg.cy * s + 3.0 * seg.dy * s.powi(2);
    let d_x = 2.0 * seg.cx + 6.0 * seg.dx * s;
    let d_y = 2.0 * seg.cy + 6.0 * seg.dy * s;
    let cur = (f_x * d_y - f_y * d_x) / (f_x.powi(2) + f_y.powi(2)).powf(1.5);

    cur
}

pub fn resample(segments: &[SplineSegment], ds: f64) -> (Vec<Point>, Vec<f64>) {
    let table = build_arc_length_table(segments, 20);
    let total_arc_length = table.last().unwrap().1;
    let n_output = (total_arc_length / ds).floor() as usize;

    let mut coordinates: Vec<Point > = Vec::new();
    let mut curvatures: Vec<f64> = Vec::new();

    for i in 0..n_output {
        let s_i = i as f64 * ds;

        let i_table = table.partition_point(|(_, s)| *s <= s_i).saturating_sub(1);

        let (t_lo, s_lo) = table[i_table];
        let (t_hi, s_hi) = table[i_table + 1];
        let t_star = t_lo + (s_i - s_lo) / (s_hi - s_lo) * (t_hi - t_lo);

        coordinates.push(eval_spline(segments, t_star));
        curvatures.push(eval_spline_curvature(segments, t_star));
    }

    (coordinates, curvatures)
}