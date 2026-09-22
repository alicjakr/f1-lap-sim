use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

#[derive(Clone, Copy)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}


// Returns (points, distances). Distance is FastF1's own telemetry channel, not a
// chord length recomputed from X/Y: at high speed, position samples can go stale or
// repeat between GPS fixes, so summing Euclidean distance between consecutive points
// systematically undercounts true distance (badly and non-uniformly on fast tracks).
// FastF1's Distance channel is integrated from Speed instead, so it doesn't have that
// problem, and it's what reference_lap.csv's own distance axis is built from too —
// using it here keeps both files on the same distance axis.
pub fn load_track_geometry(path: &Path) -> Result<(Vec<Point>, Vec<f64>), Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut points: Vec<Point> = Vec::new();
    let mut distances: Vec<f64> = Vec::new();

    let mut lines = reader.lines();
    lines.next(); // skip header

    for line in lines {
        let line = line?;
        let mut cols = line.split(',');
        let d = cols.next().ok_or("missing distance")?.parse::<f64>()?;
        let x = cols.next().ok_or("missing x")?.parse::<f64>()?;
        let y = cols.next().ok_or("missing y")?.parse::<f64>()?;
        distances.push(d);
        points.push(Point {x, y});
    }

    Ok((points, distances))
}


// Total arc length of the closed loop: FastF1's own distance up to the last sample,
// plus a short closing segment back to the first point. That closing chord is only
// trustworthy if the source lap's position telemetry is clean end-to-end — see the
// LAP_NUMBER note in export_track.py; a lap with a stale/gapped position channel can
// make this chord wildly wrong (hundreds of meters) rather than a small correction.
pub fn total_length(points: &[Point], t: &[f64]) -> f64 {
    let n = points.len();
    t[n - 1] + f64::hypot(points[0].x - points[n - 1].x, points[0].y - points[n - 1].y)
}


// --- Periodic cubic P-spline geometry smoothing ---
//
// The raw centerline is noisy GPS telemetry. Interpolating a spline exactly through
// every noisy point amplifies that noise into curvature (large, spurious curvature
// spikes at sharp local perturbations). Instead, fit a *smoothing* spline: far fewer
// control points than data points (m << n), found by penalized least squares, so the
// fitted curve can't reproduce point-to-point noise in the first place. The track is a
// closed loop, so both the basis and the roughness penalty wrap around periodically —
// otherwise the start/finish line would show the same kind of seam artifact the
// velocity solver had before it was made to run over padded laps.

// Uniform cubic B-spline blending weights and derivatives, for local parameter u in [0,1).
fn bspline_basis(u: f64) -> [f64; 4] {
    let u2 = u * u;
    let u3 = u2 * u;
    [
        (1.0 - u).powi(3) / 6.0,
        (3.0 * u3 - 6.0 * u2 + 4.0) / 6.0,
        (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) / 6.0,
        u3 / 6.0,
    ]
}

fn bspline_basis_d1(u: f64) -> [f64; 4] {
    [
        -(1.0 - u).powi(2) / 2.0,
        (3.0 * u * u - 4.0 * u) / 2.0,
        (-3.0 * u * u + 2.0 * u + 1.0) / 2.0,
        u * u / 2.0,
    ]
}

fn bspline_basis_d2(u: f64) -> [f64; 4] {
    [1.0 - u, 3.0 * u - 2.0, -3.0 * u + 1.0, u]
}


pub struct PeriodicBSpline {
    cx: Vec<f64>,
    cy: Vec<f64>,
    m: usize,
    h: f64,       // uniform knot spacing = length / m
    length: f64,  // total arc length of the closed loop
}

impl PeriodicBSpline {
    fn segment_and_u(&self, t: f64) -> (usize, f64) {
        let tm = t.rem_euclid(self.length);
        let raw = tm / self.h;
        let seg = (raw.floor() as usize).min(self.m - 1);
        (seg, raw - seg as f64)
    }

    // Control point index, wrapped periodically around the m control points.
    fn ctrl(&self, idx: isize) -> (f64, f64) {
        let m = self.m as isize;
        let j = ((idx % m) + m) % m;
        (self.cx[j as usize], self.cy[j as usize])
    }

    fn blend(&self, seg: usize, weights: [f64; 4]) -> (f64, f64) {
        let mut x = 0.0;
        let mut y = 0.0;
        for k in 0..4 {
            let (cx, cy) = self.ctrl(seg as isize - 1 + k as isize);
            x += weights[k] * cx;
            y += weights[k] * cy;
        }
        (x, y)
    }

    pub fn eval(&self, t: f64) -> (f64, f64) {
        let (seg, u) = self.segment_and_u(t);
        self.blend(seg, bspline_basis(u))
    }

    pub fn eval_deriv(&self, t: f64) -> (f64, f64) {
        let (seg, u) = self.segment_and_u(t);
        let (dx, dy) = self.blend(seg, bspline_basis_d1(u));
        (dx / self.h, dy / self.h)
    }

    pub fn eval_deriv2(&self, t: f64) -> (f64, f64) {
        let (seg, u) = self.segment_and_u(t);
        let (ddx, ddy) = self.blend(seg, bspline_basis_d2(u));
        (ddx / (self.h * self.h), ddy / (self.h * self.h))
    }

    pub fn curvature(&self, t: f64) -> f64 {
        let (dx, dy) = self.eval_deriv(t);
        let (ddx, ddy) = self.eval_deriv2(t);
        (dx * ddy - dy * ddx) / (dx * dx + dy * dy).powf(1.5)
    }
}


// The normal matrix (B^T B + lambda D^T D) is symmetric positive definite and banded: a
// data point's basis touches 4 consecutive control points and the roughness penalty 3, so
// every entry lies within |i-j| <= BAND (mod m). Stored as one short row per control point
// with periodic wraparound. Dense Gaussian elimination would be O(m^3), which rules out the
// closely-spaced knots a P-spline needs to resolve a tight corner.
const BAND: usize = 3;

struct PeriodicBandMatrix {
    m: usize,
    rows: Vec<[f64; 2 * BAND + 1]>,
}

impl PeriodicBandMatrix {
    fn new(m: usize) -> Self {
        PeriodicBandMatrix { m, rows: vec![[0.0; 2 * BAND + 1]; m] }
    }

    fn add(&mut self, i: usize, j: usize, v: f64) {
        let m = self.m as isize;
        let mut d = j as isize - i as isize;
        if d > m / 2 {
            d -= m;
        } else if d < -m / 2 {
            d += m;
        }
        self.rows[i][(d + BAND as isize) as usize] += v;
    }

    fn mul(&self, x: &[f64], out: &mut [f64]) {
        for i in 0..self.m {
            let mut acc = 0.0;
            for (k, coeff) in self.rows[i].iter().enumerate() {
                acc += coeff * x[(i + self.m + k - BAND) % self.m];
            }
            out[i] = acc;
        }
    }
}

// Jacobi-preconditioned conjugate gradient.
fn cg_solve(a: &PeriodicBandMatrix, rhs: &[f64]) -> Vec<f64> {
    let m = rhs.len();
    let inv_diag: Vec<f64> = (0..m).map(|i| 1.0 / a.rows[i][BAND]).collect();
    let dot = |u: &[f64], v: &[f64]| u.iter().zip(v).map(|(a, b)| a * b).sum::<f64>();

    let mut x = vec![0.0; m];
    let mut r = rhs.to_vec();
    let mut z: Vec<f64> = r.iter().zip(&inv_diag).map(|(ri, d)| ri * d).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    let mut ap = vec![0.0; m];
    let tol = 1e-12 * dot(rhs, rhs).sqrt().max(1.0);

    for _ in 0..20 * m {
        a.mul(&p, &mut ap);
        let pap = dot(&p, &ap);
        if pap <= 0.0 {
            break;
        }
        let alpha = rz / pap;
        for i in 0..m {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }
        if dot(&r, &r).sqrt() < tol {
            break;
        }
        for i in 0..m {
            z[i] = r[i] * inv_diag[i];
        }
        let rz_next = dot(&r, &z);
        let beta = rz_next / rz;
        rz = rz_next;
        for i in 0..m {
            p[i] = z[i] + beta * p[i];
        }
    }
    x
}


// Fit a periodic P-spline with m control points to n (>> m) raw points, minimizing
// squared residual plus lambda times a periodic second-difference roughness penalty
// on the control polygon. Solved via the normal equations (B^T B + lambda D^T D) c = B^T y,
// accumulated directly per data point rather than ever forming the dense n x m design
// matrix B.
pub fn fit_periodic_bspline(points: &[Point], t: &[f64], length: f64, m: usize, lambda: f64) -> PeriodicBSpline {
    let h = length / m as f64;

    let mut btb = PeriodicBandMatrix::new(m);
    let mut btx = vec![0.0; m];
    let mut bty = vec![0.0; m];

    for (i, p) in points.iter().enumerate() {
        let tm = t[i].rem_euclid(length);
        let raw = tm / h;
        let seg = (raw.floor() as usize).min(m - 1);
        let u = raw - seg as f64;
        let b = bspline_basis(u);
        let cols = [
            ((seg as isize - 1).rem_euclid(m as isize)) as usize,
            seg,
            (seg + 1) % m,
            (seg + 2) % m,
        ];

        for a in 0..4 {
            btx[cols[a]] += b[a] * p.x;
            bty[cols[a]] += b[a] * p.y;
            for bb in 0..4 {
                btb.add(cols[a], cols[bb], b[a] * b[bb]);
            }
        }
    }

    // Periodic roughness penalty: (Dc)_i = c[i-1] - 2c[i] + c[i+1], wrapped mod m.
    for i in 0..m {
        let idxs = [(i + m - 1) % m, i, (i + 1) % m];
        let coeffs = [1.0, -2.0, 1.0];
        for a in 0..3 {
            for b in 0..3 {
                btb.add(idxs[a], idxs[b], lambda * coeffs[a] * coeffs[b]);
            }
        }
    }

    let cx = cg_solve(&btb, &btx);
    let cy = cg_solve(&btb, &bty);

    PeriodicBSpline { cx, cy, m, h, length }
}


fn build_arc_length_table(spline: &PeriodicBSpline, steps_per_interval: usize) -> Vec<(f64, f64)> {
    let total_steps = spline.m * steps_per_interval;
    let width = spline.length / total_steps as f64;
    let mut table: Vec<(f64, f64)> = Vec::with_capacity(total_steps + 1);
    table.push((0.0, 0.0));

    let mut cumulative_s: f64 = 0.0;
    for i in 0..total_steps {
        let t_a = i as f64 * width;
        let t_b = t_a + width;
        let t_m = (t_a + t_b) / 2.0;

        let (da_x, da_y) = spline.eval_deriv(t_a);
        let (db_x, db_y) = spline.eval_deriv(t_b);
        let (dm_x, dm_y) = spline.eval_deriv(t_m);

        let speed_a = f64::hypot(da_x, da_y);
        let speed_b = f64::hypot(db_x, db_y);
        let speed_m = f64::hypot(dm_x, dm_y);

        let arc = width / 6.0 * (speed_a + 4.0 * speed_m + speed_b);
        cumulative_s += arc;
        table.push((t_b, cumulative_s));
    }

    table
}


pub fn resample(spline: &PeriodicBSpline, ds: f64) -> (Vec<Point>, Vec<f64>) {
    let table = build_arc_length_table(spline, 20);
    let total_arc_length = table.last().unwrap().1;
    let n_output = (total_arc_length / ds).floor() as usize;

    let mut coordinates: Vec<Point> = Vec::new();
    let mut curvatures: Vec<f64> = Vec::new();

    for i in 0..n_output {
        let s_i = i as f64 * ds;

        let i_table = table.partition_point(|(_, s)| *s <= s_i).saturating_sub(1);

        let (t_lo, s_lo) = table[i_table];
        let (t_hi, s_hi) = table[i_table + 1];
        let t_star = t_lo + (s_i - s_lo) / (s_hi - s_lo) * (t_hi - t_lo);

        let (x, y) = spline.eval(t_star);
        coordinates.push(Point { x, y });
        curvatures.push(spline.curvature(t_star));
    }

    (coordinates, curvatures)
}


// Half the width of a 2018 car (2.0 m regulation maximum), subtracted from each side's
// boundary offset in load_boundaries below.
const CAR_HALF_WIDTH_M: f64 = 1.0;

// Reads python_scripts/export_osm_boundaries.py's output: s,x,y,n_left,n_right. The x,y
// columns are dropped -- they're the same reference line already loaded via
// load_track_geometry, so only the arc-length axis and the two offset bounds are needed.
// That `s` axis is FastF1's raw Distance channel, the same one load_track_geometry uses --
// NOT the periodic B-spline's own arc-length parameterization that resample() produces, so
// callers matching this against a curvature array need to interpolate, not index directly.
// The raw offsets are narrowed by the car's half width and re-centered onto the driven line
// (see the body) before being handed to the solver.
pub fn load_boundaries(path: &Path) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>), Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut s: Vec<f64> = Vec::new();
    let mut n_left: Vec<f64> = Vec::new();
    let mut n_right: Vec<f64> = Vec::new();

    let mut lines = reader.lines();
    lines.next(); // skip header

    for line in lines {
        let line = line?;
        let mut cols = line.split(',');
        let s_i = cols.next().ok_or("missing s")?.parse::<f64>()?;
        cols.next().ok_or("missing x")?; // x, unused
        cols.next().ok_or("missing y")?; // y, unused
        let left = cols.next().ok_or("missing n_left")?.parse::<f64>()?;
        let right = cols.next().ok_or("missing n_right")?.parse::<f64>()?;
        let half_width = (left - right) / 2.0;
        let center = (left + right) / 2.0;
        // The car's own width has to fit inside the paved surface: its centerline can reach
        // the edge minus half a car, not the edge itself.
        let usable = (half_width - CAR_HALF_WIDTH_M).max(0.0);
        // `center` is where the OSM road centerline sits relative to the driven line, so it
        // carries the ICP registration error (up to ~16 m on Monaco, see DISCUSSION.md) on
        // top of the real racing-line-vs-road-center offset. A driven lap was physically on
        // the track, so an offset that puts the driven line outside the usable width is
        // registration error, not geometry -- clamp it back to just touching the edge rather
        // than handing the solver a corridor wider than the real track.
        let center = center.clamp(-usable, usable);
        s.push(s_i);
        n_left.push(center + usable);
        n_right.push(center - usable);
    }

    Ok((s, n_left, n_right))
}


// Reads python_scripts/export_track.py's reference_lap.csv: Distance, Speed (m, km/h) --
// the real driven lap's speed trace, on the same FastF1 Distance axis as the geometry.
pub fn load_reference_lap(path: &Path) -> Result<(Vec<f64>, Vec<f64>), Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut s: Vec<f64> = Vec::new();
    let mut speed: Vec<f64> = Vec::new();
    let mut lines = reader.lines();
    lines.next(); // skip header
    for line in lines {
        let line = line?;
        let mut cols = line.split(',');
        s.push(cols.next().ok_or("missing distance")?.parse::<f64>()?);
        speed.push(cols.next().ok_or("missing speed")?.parse::<f64>()? / 3.6);
    }
    Ok((s, speed))
}


// Reads python_scripts/derive_downforce.py's output: one row, c_l,c_d -- per-track aero
// coefficients derived from that track's own real telemetry (apex lateral acceleration,
// top speed), in place of api.rs's 3-tier HIGH/MED/LOW_DOWNFORCE guess.
pub fn load_aero_params(path: &Path) -> Result<(f64, f64), Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut lines = reader.lines();
    lines.next(); // skip header
    let line = lines.next().ok_or("empty aero_params.csv")??;
    let mut cols = line.split(',');
    let c_l = cols.next().ok_or("missing c_l")?.parse::<f64>()?;
    let c_d = cols.next().ok_or("missing c_d")?.parse::<f64>()?;
    Ok((c_l, c_d))
}


// Reads python_scripts/export_track.py's drs_zones.csv: s,drs_open. Same raw FastF1
// Distance axis as load_boundaries (not resample()'s own arc-length parameterization), so
// callers matching this against a curvature array need to interpolate via
// optimal::resample_drs, not index directly.
pub fn load_drs_zones(path: &Path) -> Result<(Vec<f64>, Vec<bool>), Box<dyn Error>> {
    let reader = BufReader::new(File::open(path)?);
    let mut s: Vec<f64> = Vec::new();
    let mut drs_open: Vec<bool> = Vec::new();

    let mut lines = reader.lines();
    lines.next(); // skip header

    for line in lines {
        let line = line?;
        let mut cols = line.split(',');
        let s_i = cols.next().ok_or("missing s")?.parse::<f64>()?;
        let open = cols.next().ok_or("missing drs_open")?.trim();
        s.push(s_i);
        drs_open.push(open.eq_ignore_ascii_case("true"));
    }

    Ok((s, drs_open))
}


// Offsets a closed-loop point sequence by a per-point lateral distance along its own
// local left-normal direction (n>0 is left of driving direction, matching
// python_scripts/export_osm_boundaries.py's convention). Used for visualization only --
// the tangent is a central difference between neighbors (periodic wraparound), not the
// analytic spline derivative, since by this point resample()'s per-point spline
// parameter t is no longer available and a finite-difference tangent is accurate enough
// for plotting.
pub fn offset_line(points: &[Point], n_profile: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let m = points.len();
    let mut x_out = Vec::with_capacity(m);
    let mut y_out = Vec::with_capacity(m);
    for i in 0..m {
        let prev = (i + m - 1) % m;
        let next = (i + 1) % m;
        let tx = points[next].x - points[prev].x;
        let ty = points[next].y - points[prev].y;
        let mag = f64::hypot(tx, ty);
        let (tx, ty) = (tx / mag, ty / mag);
        let (nx, ny) = (-ty, tx);
        x_out.push(points[i].x + n_profile[i] * nx);
        y_out.push(points[i].y + n_profile[i] * ny);
    }
    (x_out, y_out)
}

pub fn export_racing_line_csv(
    x_ref: &[f64],
    y_ref: &[f64],
    x_line: &[f64],
    y_line: &[f64],
    x_left: &[f64],
    y_left: &[f64],
    x_right: &[f64],
    y_right: &[f64],
    n_profile: &[f64],
    velocity: &[f64],
    ds: f64,
    path: &Path,
) -> Result<(), Box<dyn Error>> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writeln!(writer, "s,x_ref,y_ref,x_line,y_line,x_left,y_left,x_right,y_right,n,speed_kmh")?;

    for i in 0..x_ref.len() {
        let s = i as f64 * ds;
        writeln!(
            writer,
            "{},{},{},{},{},{},{},{},{},{},{}",
            s, x_ref[i], y_ref[i], x_line[i], y_line[i],
            x_left[i], y_left[i], x_right[i], y_right[i],
            n_profile[i], velocity[i] * 3.6,
        )?;
    }

    Ok(())
}

pub fn export_curvature_csv(curvature: &[f64], ds: f64, path: &Path) -> Result<(), Box<dyn Error>> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    writeln!(writer, "Distance,Curvature")?;

    for (i, k) in curvature.iter().enumerate() {
        let distance = i as f64 * ds;
        writeln!(writer, "{},{}", distance, k)?;
    }

    Ok(())
}
