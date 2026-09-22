// Minimum-time velocity profile via direct collocation, solved with Ipopt. See
// results/DISCUSSION.md for the full rationale (vs. the two-pass sweep, and why analytic
// Hessians replaced Ipopt's L-BFGS approximation).
//
// State/control: x(s) = v(s)^2 (avoids a 1/v singularity, affine dynamics), u(s) = a_lon(s),
// signed: positive = traction, negative = brake.
//   dx/ds = 2u
// trapezoidal collocation over n points around a closed (periodic) lap:
//   x[i+1] - x[i] - (u[i] + u[i+1])*ds = 0   (indices wrap mod n)
//
// Path constraints per point: friction ellipse and traction-power ceiling:
//   (x*kappa)^2 / (mu_lat*g_eff)^2 + w^2 / (mu_lon*g_eff)^2 <= 1,   g_eff = G + c_l*x
//   u <= p_engine/sqrt(x) - c_d*x
// where w = u + c_d*x is the longitudinal force the *tires* must provide: u is the net
// acceleration (drag included, as the dynamics and power ceiling above require), so drag
// has to be added back to get the tire's share -- it helps under braking and costs under
// acceleration.
//
// Objective: minimize sum(ds / v[i]), the same lap-time quantity solver::lap_time measures.
//
// Analytic Hessian: the dynamics defect is linear in (x, u), contributing nothing. Only the
// objective and ellipse/power constraints need second derivatives. With g = G + c_l*x,
// K = kappa, L = mu_lat, M = mu_lon:
//   d2f/dx2   = 0.75*ds*x^-2.5                                        (objective, diagonal)
//   d2E/dx2   = 2*K^2*G/(L^2*g^3) - 6*c_l*K^2*x*G/(L^2*g^4)
//               + (2/M^2)*[(c_d^2*g - c_d*c_l*w)/g^3 - 3*c_l*(c_d*g*w - c_l*w^2)/g^4]
//   d2E/du2   = 2/(M^2*g^2)
//   d2E/dxdu  = 2*c_d/(M^2*g^2) - 4*c_l*w/(M^2*g^3)
//   d2P/dx2   = -0.75*p_engine*x^-2.5
// (d2P/du2 = d2P/dxdu = 0 -- power is linear in u.) Sparsity: 3 entries per point (x
// diagonal, u diagonal, x-u cross term).

use ipopt::*;

use crate::solver::{backward_pass, corner_speed_limits, forward_pass, top_speed_drs, CarParams};

const G: f64 = 9.81;

/// Coarse periodic grid over a full-resolution lap of `n_full` samples spaced `full_ds`
/// apart: `m` points spaced exactly `L/m`, so the wrap-around segment back to the start is
/// the same length as every other one (an integer stride leaves it short while the
/// collocation still treats it as a full ds, overstating the lap length by up to one ds).
pub fn coarse_grid(n_full: usize, full_ds: f64, target_spacing: f64) -> (usize, f64) {
    let length = n_full as f64 * full_ds;
    let m = ((length / target_spacing).round() as usize).max(3);
    (m, length / m as f64)
}

/// Periodic linear interpolation of a full-resolution array (sample i at s = i*full_ds)
/// onto coarse_grid's points. Returns the coarse array and its ds.
pub fn coarsen_periodic(values: &[f64], full_ds: f64, target_spacing: f64) -> (Vec<f64>, f64) {
    let n = values.len();
    let (m, ds) = coarse_grid(n, full_ds, target_spacing);
    let coarse = (0..m)
        .map(|k| {
            let pos = k as f64 * ds / full_ds;
            let i = pos.floor() as usize;
            let frac = pos - i as f64;
            values[i % n] * (1.0 - frac) + values[(i + 1) % n] * frac
        })
        .collect();
    (coarse, ds)
}

fn check_status(status: SolveStatus) -> Result<(), String> {
    match status {
        SolveStatus::SolveSucceeded | SolveStatus::SolvedToAcceptableLevel => Ok(()),
        other => Err(format!("Ipopt did not converge: {:?}", other)),
    }
}

struct MinTimeProblem {
    curvature: Vec<f64>,
    params: CarParams,
    c_d: Vec<f64>, // per-point drag coefficient: params.c_d_drs where DRS is open, else params.c_d
    ds: f64,
    n: usize,
    x_min: f64,
    x_max: Vec<f64>, // per point: the global cap, tightened by the steering-rate limit
    u_bound: f64,
    initial_x: Vec<f64>,
    initial_u: Vec<f64>,
}

impl MinTimeProblem {
    fn next(&self, i: usize) -> usize {
        (i + 1) % self.n
    }

    fn g_eff(&self, x_i: f64) -> f64 {
        G + self.params.c_l * x_i
    }
}

impl BasicProblem for MinTimeProblem {
    fn num_variables(&self) -> usize {
        2 * self.n
    }

    fn bounds(&self, x_l: &mut [Number], x_u: &mut [Number]) -> bool {
        for i in 0..self.n {
            x_l[i] = self.x_min;
            x_u[i] = self.x_max[i];
            x_l[self.n + i] = -self.u_bound;
            x_u[self.n + i] = self.u_bound;
        }
        true
    }

    fn initial_point(&self, x: &mut [Number]) -> bool {
        x[0..self.n].copy_from_slice(&self.initial_x);
        x[self.n..2 * self.n].copy_from_slice(&self.initial_u);
        true
    }

    fn objective(&self, x: &[Number], _new_x: bool, obj: &mut Number) -> bool {
        *obj = (0..self.n).map(|i| self.ds / x[i].sqrt()).sum();
        true
    }

    fn objective_grad(&self, x: &[Number], _new_x: bool, grad_f: &mut [Number]) -> bool {
        for i in 0..self.n {
            grad_f[i] = -0.5 * self.ds * x[i].powf(-1.5);
            grad_f[self.n + i] = 0.0;
        }
        true
    }
}

impl ConstrainedProblem for MinTimeProblem {
    fn num_constraints(&self) -> usize {
        3 * self.n
    }

    fn num_constraint_jacobian_non_zeros(&self) -> usize {
        8 * self.n
    }

    fn constraint_bounds(&self, g_l: &mut [Number], g_u: &mut [Number]) -> bool {
        for i in 0..self.n {
            g_l[i] = 0.0; // dynamics defect: equality
            g_u[i] = 0.0;
            g_l[self.n + i] = -1e20; // friction ellipse: <= 1
            g_u[self.n + i] = 1.0;
            g_l[2 * self.n + i] = -1e20; // power ceiling: <= 0
            g_u[2 * self.n + i] = 0.0;
        }
        true
    }

    fn constraint(&self, x: &[Number], _new_x: bool, g: &mut [Number]) -> bool {
        let (xs, us) = x.split_at(self.n);
        for i in 0..self.n {
            let ip1 = self.next(i);
            g[i] = xs[ip1] - xs[i] - (us[i] + us[ip1]) * self.ds;
        }
        for i in 0..self.n {
            let g_eff = self.g_eff(xs[i]);
            let a_lat = xs[i] * self.curvature[i];
            let w = us[i] + self.c_d[i] * xs[i]; // tire longitudinal force, drag added back
            g[self.n + i] = (a_lat / (self.params.mu_lat * g_eff)).powi(2)
                + (w / (self.params.mu_lon * g_eff)).powi(2);
        }
        for i in 0..self.n {
            g[2 * self.n + i] =
                us[i] - self.params.p_engine / xs[i].sqrt() + self.c_d[i] * xs[i];
        }
        true
    }

    fn constraint_jacobian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        let mut k = 0;
        // dynamics defect row i: d/dx_i = -1, d/dx_{i+1} = 1, d/du_i = -ds, d/du_{i+1} = -ds
        for i in 0..self.n {
            let ip1 = self.next(i);
            let row = i as Index;
            irow[k] = row;
            jcol[k] = i as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = ip1 as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = (self.n + i) as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = (self.n + ip1) as Index;
            k += 1;
        }
        // ellipse row n+i: depends on x_i, u_i only
        for i in 0..self.n {
            let row = (self.n + i) as Index;
            irow[k] = row;
            jcol[k] = i as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = (self.n + i) as Index;
            k += 1;
        }
        // power row 2n+i: depends on x_i, u_i only
        for i in 0..self.n {
            let row = (2 * self.n + i) as Index;
            irow[k] = row;
            jcol[k] = i as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = (self.n + i) as Index;
            k += 1;
        }
        true
    }

    fn constraint_jacobian_values(&self, x: &[Number], _new_x: bool, vals: &mut [Number]) -> bool {
        let (xs, us) = x.split_at(self.n);
        let mut k = 0;
        for _ in 0..self.n {
            vals[k] = -1.0;
            k += 1;
            vals[k] = 1.0;
            k += 1;
            vals[k] = -self.ds;
            k += 1;
            vals[k] = -self.ds;
            k += 1;
        }
        // Ellipse E(x,u) = K^2*x^2/(L^2*g_eff^2) + w^2/(M^2*g_eff^2), g_eff = G + c_l*x,
        // w = u + c_d*x. The lateral term's dE/dx uses d/dx[x^2/g_eff^2] = 2x*G/g_eff^3 (the
        // c_l terms cancel: g_eff - x*c_l = G).
        for i in 0..self.n {
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let c_d = self.c_d[i];
            let kap = self.curvature[i];
            let g_eff = self.g_eff(xs[i]);
            let w = us[i] + c_d * xs[i];
            let de_dx = 2.0 * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(3))
                + 2.0 * w * (c_d * g_eff - w * c_l) / (m.powi(2) * g_eff.powi(3));
            let de_du = 2.0 * w / (m.powi(2) * g_eff.powi(2));
            vals[k] = de_dx;
            k += 1;
            vals[k] = de_du;
            k += 1;
        }
        // Power P(x,u) = u - p_engine/sqrt(x) + c_d*x
        for i in 0..self.n {
            let dp_dx = 0.5 * self.params.p_engine * xs[i].powf(-1.5) + self.c_d[i];
            vals[k] = dp_dx;
            k += 1;
            vals[k] = 1.0;
            k += 1;
        }
        true
    }

    // Lower-triangular sparsity: per point i, the x_i/u_i block contributes (i,i), (n+i,n+i),
    // and (n+i,i) -- row=n+i >= col=i always holds since n>i for every i in 0..n. See the
    // module doc comment for the derivatives.
    fn num_hessian_non_zeros(&self) -> usize {
        3 * self.n
    }

    fn hessian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        for i in 0..self.n {
            let k = 3 * i;
            irow[k] = i as Index;
            jcol[k] = i as Index; // (x_i, x_i)
            irow[k + 1] = (self.n + i) as Index;
            jcol[k + 1] = (self.n + i) as Index; // (u_i, u_i)
            irow[k + 2] = (self.n + i) as Index;
            jcol[k + 2] = i as Index; // (u_i, x_i)
        }
        true
    }

    fn hessian_values(
        &self,
        x: &[Number],
        _new_x: bool,
        obj_factor: Number,
        lambda: &[Number],
        vals: &mut [Number],
    ) -> bool {
        let (xs, us) = x.split_at(self.n);
        for i in 0..self.n {
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let c_d = self.c_d[i];
            let kap = self.curvature[i];
            let g_eff = self.g_eff(xs[i]);
            let w = us[i] + c_d * xs[i];
            let lambda_ellipse = lambda[self.n + i];
            let lambda_power = lambda[2 * self.n + i];

            let d2f_dx2 = 0.75 * self.ds * xs[i].powf(-2.5);
            let d2e_dx2 = 2.0 * kap.powi(2) * G / (l.powi(2) * g_eff.powi(3))
                - 6.0 * c_l * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(4))
                + 2.0
                    * ((c_d * c_d * g_eff - c_d * c_l * w) / g_eff.powi(3)
                        - 3.0 * c_l * (c_d * g_eff * w - c_l * w * w) / g_eff.powi(4))
                    / m.powi(2);
            let d2p_dx2 = -0.75 * self.params.p_engine * xs[i].powf(-2.5);
            let d2e_du2 = 2.0 / (m.powi(2) * g_eff.powi(2));
            let d2e_dxdu = 2.0 * c_d / (m.powi(2) * g_eff.powi(2))
                - 4.0 * c_l * w / (m.powi(2) * g_eff.powi(3));

            let k = 3 * i;
            vals[k] = obj_factor * d2f_dx2 + lambda_ellipse * d2e_dx2 + lambda_power * d2p_dx2;
            vals[k + 1] = lambda_ellipse * d2e_du2;
            vals[k + 2] = lambda_ellipse * d2e_dxdu;
        }
        true
    }
}

/// Solves the minimum-time velocity profile over one closed lap on a coarse grid
/// (~target_spacing meters between collocation points, subsampled from the full-resolution
/// curvature array). drs_s/drs_open are python_scripts/export_track.py's drs_zones.csv,
/// resampled here onto the coarse grid via resample_drs -- pass empty slices for a track
/// with no DRS data (every point then uses params.c_d, i.e. DRS always closed).
/// Returns (velocity profile, lap time, coarse ds), or Err if Ipopt didn't converge.
pub fn solve_min_time(
    curvature_full: &[f64],
    full_ds: f64,
    params: &CarParams,
    target_spacing: f64,
    drs_s: &[f64],
    drs_open: &[bool],
    omega_max: Option<f64>,
) -> Result<(Vec<f64>, f64, f64), String> {
    let (curvature, ds) = coarsen_periodic(curvature_full, full_ds, target_spacing);
    let n = curvature.len();

    // Warm start from the existing two-pass sweep on the same coarse grid (not periodic,
    // just a reasonable initial guess -- the NLP's own periodicity constraint resolves the
    // seam that the padded-laps trick used to paper over). DRS isn't modeled in this warm
    // start (uses the constant DRS-closed params.c_d) -- it only needs to be a reasonable
    // starting guess, not itself correct.
    let corner_lims = corner_speed_limits(&curvature, params);
    let v_back = backward_pass(&curvature, &corner_lims, params, ds);
    let v_init = forward_pass(&curvature, &v_back, params, ds);

    let x_min = 5.0 * 5.0; // 5 m/s floor, avoids sqrt(0) and a stationary car mid-lap
    // Sized off the DRS-open (lower-drag, higher-top-speed) case so a DRS zone's real speed
    // potential isn't clipped by a bound sized off the DRS-closed drag alone.
    let x_max = top_speed_drs(params).powi(2);
    // Steering-rate limit (see api::steering_rate_limit). kappa is fixed on this problem, so
    // "curvature can't change faster than omega_max per second" is just a speed cap at points
    // where the reference line's curvature changes quickly. Never tightened below x_min --
    // a cap under the floor would make the problem infeasible rather than slow.
    let x_max: Vec<f64> = (0..n)
        .map(|i| {
            let d_kappa = (curvature[(i + 1) % n] - curvature[i]).abs() / ds;
            match omega_max {
                Some(w) if d_kappa > 1e-12 => x_max.min((w / d_kappa).powi(2)).max(x_min),
                _ => x_max,
            }
        })
        .collect();
    let initial_x: Vec<f64> = v_init
        .iter()
        .enumerate()
        .map(|(i, v)| v.powi(2).clamp(x_min, x_max[i]))
        .collect();
    let initial_u: Vec<f64> = (0..n)
        .map(|i| {
            let ip1 = (i + 1) % n;
            (initial_x[ip1] - initial_x[i]) / (2.0 * ds)
        })
        .collect();
    let u_bound = 10.0 * G; // loose box; the ellipse/power constraints bind first

    let target_s: Vec<f64> = (0..n).map(|i| i as f64 * ds).collect();
    let c_d: Vec<f64> = if drs_s.is_empty() {
        vec![params.c_d; n]
    } else {
        resample_drs(drs_s, drs_open, &target_s)
            .iter()
            .map(|&open| if open { params.c_d_drs } else { params.c_d })
            .collect()
    };

    let problem = MinTimeProblem {
        curvature,
        params: params.clone(),
        c_d,
        ds,
        n,
        x_min,
        x_max,
        u_bound,
        initial_x,
        initial_u,
    };

    let mut ipopt = Ipopt::new(problem).unwrap();
    // "exact" is Ipopt's default, but set explicitly rather than relying on that implicitly
    // -- MinTimeProblem's hessian_values() only makes sense if this stays "exact".
    ipopt.set_option("hessian_approximation", "exact");
    ipopt.set_option("mu_strategy", "adaptive");
    ipopt.set_option("tol", 1e-6);
    ipopt.set_option("max_iter", 3000);
    ipopt.set_option("sb", "yes");
    ipopt.set_option("print_level", 5);

    let SolveResult {
        solver_data: SolverDataMut { solution, .. },
        status,
        objective_value,
        ..
    } = ipopt.solve();

    println!("Ipopt status: {:?}", status);
    check_status(status)?;

    let velocity: Vec<f64> = solution.primal_variables[0..n].iter().map(|x| x.sqrt()).collect();
    Ok((velocity, objective_value, ds))
}

// --- Racing-line optimization: free lateral offset on top of the fixed-line solver above ---
// Adds lateral offset n(s) (bounded by track width, data/<slug>_track_boundaries.csv),
// heading xi(s) relative to the track tangent, and path curvature kappa(s) as a free
// control (replacing MinTimeProblem's fixed curvature array). Heading-based curvilinear
// formulation (Perantoni & Limebeer). With h(s) = 1 - n(s)*kappa_ref(s) (path-length
// stretch factor) and the small-angle approximation tan(xi)=xi, cos(xi)=1 (xi is bounded as
// a validity rail, not a physical limit):
//   dn/ds = h*xi
//   dxi/ds = kappa*h - kappa_ref
//   dt/ds = h/v
//   dx/ds = 2*u*h        (the car covers h*ds of path per ds of reference line)
//   a_lat = x*kappa
//
// Staggered grid: x, u, n, kappa live on the nodes, xi on the segments (xi[i] is the heading
// over segment i -> i+1):
//   n-defect[i]  = n[i+1] - n[i] - ds*xi[i]*(h[i] + h[i+1])/2
//   xi-defect[i] = xi[i] - xi[i-1] - ds*(kappa[i]*h[i] - kappa_ref[i])
// so xi is fixed by n's first differences and kappa by xi's. With all three on the nodes
// (plain trapezoidal), an alternating xi/kappa pattern satisfies both defects at any
// amplitude while letting kappa undercut kappa_ref at speed-limiting nodes -- an exact
// odd-even null space the solver exploits for a spurious lap-time gain. The staggered grid
// removes that mode structurally, so no regularization is needed.
//
// Variables, 5n: [x, u, n, xi, kappa] at offsets 0, n, 2n, 3n, 4n.
// Constraints, 6n: [x-defect, n-defect, xi-defect, ellipse, power, steering-rate] at row
// offsets 0, n, 2n, 3n, 4n, 5n. The steering-rate rows bound (kappa[i+1]-kappa[i])^2 * x[i]
// -- i.e. |dkappa/ds|*v, the rate the car re-aims itself in *time* (see
// api::steering_rate_limit). Without it a point mass changes direction instantly and the
// free line exploits flicks no car could drive. Power is unchanged from MinTimeProblem; the ellipse is the same functional form
// with kappa in place of the fixed curvature; the x-defect carries the same h as dt/ds does,
// so speed changes are integrated over the path actually driven rather than over the
// reference line. n=0, xi=0, kappa=kappa_ref reproduces MinTimeProblem exactly.
//
// Hessian terms beyond MinTimeProblem's (K=kappa_i, L=mu_lat, g=g_eff):
//   d2E/dK2                      = 2*x^2/(L^2*g^2)
//   d2E/dKdx                     = 4*x*K/(L^2*g^2) - 4*x^2*K*c_l/(L^2*g^3)
//   d2f/dxdn                     = 0.5*ds*kappa_ref_i*x^-1.5
//   d2(n-defect[i])/dxi_i dn_i     = ds*kappa_ref_i/2
//   d2(n-defect[i])/dxi_i dn_{i+1} = ds*kappa_ref_{i+1}/2
//   d2(xi-defect[i])/dK_i dn_i     = ds*kappa_ref_i
//   d2(x-defect)/du_i dn_i         = ds*kappa_ref_i (from the defects both starting and
//                                    ending at i -- see hessian_values)
//   d2(rate[i])/dK_i2 = d2(rate[i])/dK_{i+1}2 = 2*x_i,  d2/dK_i dK_{i+1} = -2*x_i
//   d2(rate[i])/dK_i dx_i = -2*(K_{i+1}-K_i),  d2/dK_{i+1} dx_i = 2*(K_{i+1}-K_i)

/// Linearly interpolates n_left(s)/n_right(s) from the boundary CSV's own arc-length axis
/// (FastF1's raw Distance channel, via track::load_boundaries) onto the solver's coarse
/// grid `target_s[i] = i*ds`. That axis isn't numerically identical to the curvature
/// array's own arc-length parameterization (the periodic B-spline's, via track::resample)
/// -- both measure the same physical lap distance, just via slightly different paths, and
/// treating them as equivalent here is an accepted approximation (see track::load_boundaries).
pub fn resample_bounds(bound_s: &[f64], n_left: &[f64], n_right: &[f64], target_s: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let total = *bound_s.last().unwrap();
    let mut out_left = Vec::with_capacity(target_s.len());
    let mut out_right = Vec::with_capacity(target_s.len());
    for &s in target_s {
        let s = s.rem_euclid(total);
        let idx = bound_s
            .partition_point(|&bs| bs <= s)
            .saturating_sub(1)
            .min(bound_s.len() - 2);
        let (s_lo, s_hi) = (bound_s[idx], bound_s[idx + 1]);
        let frac = if s_hi > s_lo { (s - s_lo) / (s_hi - s_lo) } else { 0.0 };
        out_left.push(n_left[idx] + frac * (n_left[idx + 1] - n_left[idx]));
        out_right.push(n_right[idx] + frac * (n_right[idx + 1] - n_right[idx]));
    }
    (out_left, out_right)
}

/// Nearest-sample resample of a boolean DRS-open signal (python_scripts/export_track.py's
/// drs_zones.csv, on the same raw FastF1 Distance axis as load_boundaries -- see that
/// function's comment) onto the solver's coarse grid `target_s[i] = i*ds`. Nearest-sample
/// rather than resample_bounds's linear interpolation since there's no sensible
/// "in-between" value for a boolean -- DRS zones (hundreds of meters) are comfortably
/// larger than one coarse grid cell, so nearest-sample doesn't lose a zone entirely.
pub fn resample_drs(drs_s: &[f64], drs_open: &[bool], target_s: &[f64]) -> Vec<bool> {
    let total = *drs_s.last().unwrap();
    let mut out = Vec::with_capacity(target_s.len());
    for &s in target_s {
        let s = s.rem_euclid(total);
        let idx = drs_s.partition_point(|&ds| ds <= s).saturating_sub(1).min(drs_s.len() - 2);
        let (s_lo, s_hi) = (drs_s[idx], drs_s[idx + 1]);
        let nearest = if s_hi > s_lo && (s - s_lo) > (s_hi - s) { idx + 1 } else { idx };
        out.push(drs_open[nearest]);
    }
    out
}

struct RacingLineProblem {
    curvature: Vec<f64>, // kappa_ref
    params: CarParams,
    c_d: Vec<f64>, // per-point drag coefficient: params.c_d_drs where DRS is open, else params.c_d
    ds: f64,
    n: usize,
    x_min: f64,
    x_max: f64,
    u_bound: f64,
    xi_bound: f64,
    kappa_bound: f64,
    // (omega_max * ds)^2: bounds (kappa[i+1] - kappa[i])^2 * x[i], i.e. |dkappa/ds|*v.
    rate_bound_sq: f64,
    n_left: Vec<f64>,
    n_right: Vec<f64>,
    initial_x: Vec<f64>,
    initial_u: Vec<f64>,
}

impl RacingLineProblem {
    fn next(&self, i: usize) -> usize {
        (i + 1) % self.n
    }

    fn prev(&self, i: usize) -> usize {
        (i + self.n - 1) % self.n
    }

    fn g_eff(&self, x_i: f64) -> f64 {
        G + self.params.c_l * x_i
    }

    fn h(&self, n_i: f64, i: usize) -> f64 {
        1.0 - n_i * self.curvature[i]
    }
}

impl BasicProblem for RacingLineProblem {
    fn num_variables(&self) -> usize {
        5 * self.n
    }

    fn bounds(&self, x_l: &mut [Number], x_u: &mut [Number]) -> bool {
        let n = self.n;
        for i in 0..n {
            x_l[i] = self.x_min;
            x_u[i] = self.x_max;
            x_l[n + i] = -self.u_bound;
            x_u[n + i] = self.u_bound;
            x_l[2 * n + i] = self.n_right[i];
            x_u[2 * n + i] = self.n_left[i];
            x_l[3 * n + i] = -self.xi_bound;
            x_u[3 * n + i] = self.xi_bound;
            x_l[4 * n + i] = -self.kappa_bound;
            x_u[4 * n + i] = self.kappa_bound;
        }
        true
    }

    fn initial_point(&self, x: &mut [Number]) -> bool {
        let n = self.n;
        x[0..n].copy_from_slice(&self.initial_x);
        x[n..2 * n].copy_from_slice(&self.initial_u);
        for i in 0..n {
            x[2 * n + i] = 0.0;
            x[3 * n + i] = 0.0;
            x[4 * n + i] = self.curvature[i];
        }
        true
    }

    fn objective(&self, x: &[Number], _new_x: bool, obj: &mut Number) -> bool {
        let n = self.n;
        *obj = (0..n)
            .map(|i| self.ds * self.h(x[2 * n + i], i) / x[i].sqrt())
            .sum();
        true
    }

    fn objective_grad(&self, x: &[Number], _new_x: bool, grad_f: &mut [Number]) -> bool {
        let n = self.n;
        for i in 0..n {
            let h_i = self.h(x[2 * n + i], i);
            grad_f[i] = -0.5 * self.ds * h_i * x[i].powf(-1.5);
            grad_f[n + i] = 0.0;
            grad_f[2 * n + i] = -self.ds * self.curvature[i] / x[i].sqrt();
            grad_f[3 * n + i] = 0.0;
            grad_f[4 * n + i] = 0.0;
        }
        true
    }
}

impl ConstrainedProblem for RacingLineProblem {
    fn num_constraints(&self) -> usize {
        6 * self.n
    }

    fn num_constraint_jacobian_non_zeros(&self) -> usize {
        21 * self.n
    }

    fn constraint_bounds(&self, g_l: &mut [Number], g_u: &mut [Number]) -> bool {
        let n = self.n;
        for i in 0..n {
            g_l[i] = 0.0; // x-defect: equality
            g_u[i] = 0.0;
            g_l[n + i] = 0.0; // n-defect: equality
            g_u[n + i] = 0.0;
            g_l[2 * n + i] = 0.0; // xi-defect: equality
            g_u[2 * n + i] = 0.0;
            g_l[3 * n + i] = -1e20; // friction ellipse: <= 1
            g_u[3 * n + i] = 1.0;
            g_l[4 * n + i] = -1e20; // power ceiling: <= 0
            g_u[4 * n + i] = 0.0;
            g_l[5 * n + i] = -1e20; // steering rate: (dkappa)^2 * x <= (omega*ds)^2
            g_u[5 * n + i] = self.rate_bound_sq;
        }
        true
    }

    fn constraint(&self, x: &[Number], _new_x: bool, g: &mut [Number]) -> bool {
        let n = self.n;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let ns = &x[2 * n..3 * n];
        let xis = &x[3 * n..4 * n];
        let kappas = &x[4 * n..5 * n];

        for i in 0..n {
            let ip1 = self.next(i);
            g[i] = xs[ip1] - xs[i]
                - self.ds * (us[i] * self.h(ns[i], i) + us[ip1] * self.h(ns[ip1], ip1));
        }
        for i in 0..n {
            let ip1 = self.next(i);
            let h_mid = 0.5 * (self.h(ns[i], i) + self.h(ns[ip1], ip1));
            g[n + i] = ns[ip1] - ns[i] - self.ds * xis[i] * h_mid;
        }
        for i in 0..n {
            let im1 = self.prev(i);
            g[2 * n + i] = xis[i] - xis[im1]
                - self.ds * (kappas[i] * self.h(ns[i], i) - self.curvature[i]);
        }
        for i in 0..n {
            let g_eff = self.g_eff(xs[i]);
            let a_lat = xs[i] * kappas[i];
            let w = us[i] + self.c_d[i] * xs[i]; // tire longitudinal force, drag added back
            g[3 * n + i] = (a_lat / (self.params.mu_lat * g_eff)).powi(2)
                + (w / (self.params.mu_lon * g_eff)).powi(2);
        }
        for i in 0..n {
            g[4 * n + i] =
                us[i] - self.params.p_engine / xs[i].sqrt() + self.c_d[i] * xs[i];
        }
        for i in 0..n {
            let d = kappas[self.next(i)] - kappas[i];
            g[5 * n + i] = d * d * xs[i];
        }
        true
    }

    fn constraint_jacobian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        let n = self.n;
        let mut k = 0;

        // x-defect row i: d/dx_i, d/dx_{i+1}, d/du_i, d/du_{i+1}, d/dn_i, d/dn_{i+1}
        for i in 0..n {
            let ip1 = self.next(i);
            let row = i as Index;
            irow[k] = row; jcol[k] = i as Index; k += 1;
            irow[k] = row; jcol[k] = ip1 as Index; k += 1;
            irow[k] = row; jcol[k] = (n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (n + ip1) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + ip1) as Index; k += 1;
        }
        // n-defect row n+i: d/dn_i, d/dn_{i+1}, d/dxi_i
        for i in 0..n {
            let ip1 = self.next(i);
            let row = (n + i) as Index;
            irow[k] = row; jcol[k] = (2 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + ip1) as Index; k += 1;
            irow[k] = row; jcol[k] = (3 * n + i) as Index; k += 1;
        }
        // xi-defect row 2n+i: d/dxi_i, d/dxi_{i-1}, d/dkappa_i, d/dn_i
        for i in 0..n {
            let im1 = self.prev(i);
            let row = (2 * n + i) as Index;
            irow[k] = row; jcol[k] = (3 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (3 * n + im1) as Index; k += 1;
            irow[k] = row; jcol[k] = (4 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + i) as Index; k += 1;
        }
        // ellipse row 3n+i: d/dx_i, d/du_i, d/dkappa_i
        for i in 0..n {
            let row = (3 * n + i) as Index;
            irow[k] = row; jcol[k] = i as Index; k += 1;
            irow[k] = row; jcol[k] = (n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (4 * n + i) as Index; k += 1;
        }
        // power row 4n+i: d/dx_i, d/du_i
        for i in 0..n {
            let row = (4 * n + i) as Index;
            irow[k] = row; jcol[k] = i as Index; k += 1;
            irow[k] = row; jcol[k] = (n + i) as Index; k += 1;
        }
        // steering-rate row 5n+i: d/dkappa_i, d/dkappa_{i+1}, d/dx_i
        for i in 0..n {
            let row = (5 * n + i) as Index;
            irow[k] = row; jcol[k] = (4 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (4 * n + self.next(i)) as Index; k += 1;
            irow[k] = row; jcol[k] = i as Index; k += 1;
        }
        true
    }

    fn constraint_jacobian_values(&self, x: &[Number], _new_x: bool, vals: &mut [Number]) -> bool {
        let n = self.n;
        let ds = self.ds;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let ns = &x[2 * n..3 * n];
        let xis = &x[3 * n..4 * n];
        let kappas = &x[4 * n..5 * n];
        let mut k = 0;

        for i in 0..n {
            let ip1 = self.next(i);
            vals[k] = -1.0; k += 1;
            vals[k] = 1.0; k += 1;
            vals[k] = -ds * self.h(ns[i], i); k += 1;
            vals[k] = -ds * self.h(ns[ip1], ip1); k += 1;
            vals[k] = ds * us[i] * self.curvature[i]; k += 1;
            vals[k] = ds * us[ip1] * self.curvature[ip1]; k += 1;
        }
        for i in 0..n {
            let ip1 = self.next(i);
            let h_mid = 0.5 * (self.h(ns[i], i) + self.h(ns[ip1], ip1));
            vals[k] = -1.0 + 0.5 * ds * xis[i] * self.curvature[i]; k += 1;
            vals[k] = 1.0 + 0.5 * ds * xis[i] * self.curvature[ip1]; k += 1;
            vals[k] = -ds * h_mid; k += 1;
        }
        for i in 0..n {
            vals[k] = 1.0; k += 1;
            vals[k] = -1.0; k += 1;
            vals[k] = -ds * self.h(ns[i], i); k += 1;
            vals[k] = ds * kappas[i] * self.curvature[i]; k += 1;
        }
        // Ellipse E(x,u,kappa) = kappa^2*x^2/(L^2*g^2) + w^2/(M^2*g^2), g = G + c_l*x,
        // w = u + c_d*x (see the module comment).
        for i in 0..n {
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let c_d = self.c_d[i];
            let kap = kappas[i];
            let g_eff = self.g_eff(xs[i]);
            let w = us[i] + c_d * xs[i];
            let de_dx = 2.0 * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(3))
                + 2.0 * w * (c_d * g_eff - w * c_l) / (m.powi(2) * g_eff.powi(3));
            let de_du = 2.0 * w / (m.powi(2) * g_eff.powi(2));
            let de_dkappa = 2.0 * xs[i].powi(2) * kap / (l.powi(2) * g_eff.powi(2));
            vals[k] = de_dx; k += 1;
            vals[k] = de_du; k += 1;
            vals[k] = de_dkappa; k += 1;
        }
        for i in 0..n {
            let dp_dx = 0.5 * self.params.p_engine * xs[i].powf(-1.5) + self.c_d[i];
            vals[k] = dp_dx; k += 1;
            vals[k] = 1.0; k += 1;
        }
        for i in 0..n {
            let d = kappas[self.next(i)] - kappas[i];
            vals[k] = -2.0 * d * xs[i]; k += 1;
            vals[k] = 2.0 * d * xs[i]; k += 1;
            vals[k] = d * d; k += 1;
        }
        true
    }

    // Lower-triangular sparsity, 9 entries per node i -- every pair is local to node i except
    // (xi_i, n_{i+1}), and row >= col holds for all of them since the variable blocks are
    // ordered x < u < n < xi < kappa.
    fn num_hessian_non_zeros(&self) -> usize {
        12 * self.n
    }

    fn hessian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        let n = self.n;
        for i in 0..n {
            let ip1 = self.next(i);
            let k = 12 * i;
            irow[k] = i as Index; jcol[k] = i as Index; // (x_i, x_i)
            irow[k + 1] = (n + i) as Index; jcol[k + 1] = (n + i) as Index; // (u_i, u_i)
            irow[k + 2] = (n + i) as Index; jcol[k + 2] = i as Index; // (u_i, x_i)
            irow[k + 3] = (4 * n + i) as Index; jcol[k + 3] = (4 * n + i) as Index; // (kappa_i, kappa_i)
            irow[k + 4] = (4 * n + i) as Index; jcol[k + 4] = i as Index; // (kappa_i, x_i)
            irow[k + 5] = (2 * n + i) as Index; jcol[k + 5] = i as Index; // (n_i, x_i)
            irow[k + 6] = (3 * n + i) as Index; jcol[k + 6] = (2 * n + i) as Index; // (xi_i, n_i)
            irow[k + 7] = (3 * n + i) as Index; jcol[k + 7] = (2 * n + ip1) as Index; // (xi_i, n_{i+1})
            irow[k + 8] = (4 * n + i) as Index; jcol[k + 8] = (2 * n + i) as Index; // (kappa_i, n_i)
            irow[k + 9] = (2 * n + i) as Index; jcol[k + 9] = (n + i) as Index; // (n_i, u_i)
            // steering rate: (kappa_{i+1}, kappa_i) -- ordered by variable index so row >= col
            // holds across the wraparound -- and (kappa_i, x_{i-1}) from the defect ending at i.
            let (kap_a, kap_b) = (4 * n + i, 4 * n + ip1);
            irow[k + 10] = kap_a.max(kap_b) as Index; jcol[k + 10] = kap_a.min(kap_b) as Index;
            irow[k + 11] = (4 * n + i) as Index; jcol[k + 11] = self.prev(i) as Index;
        }
        true
    }

    fn hessian_values(
        &self,
        x: &[Number],
        _new_x: bool,
        obj_factor: Number,
        lambda: &[Number],
        vals: &mut [Number],
    ) -> bool {
        let n = self.n;
        let ds = self.ds;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let kappas = &x[4 * n..5 * n];

        for i in 0..n {
            let ip1 = self.next(i);
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let c_d = self.c_d[i];
            let kap = kappas[i];
            let g_eff = self.g_eff(xs[i]);
            let w = us[i] + c_d * xs[i];
            let h_i = self.h(x[2 * n + i], i);

            let prev_i = self.prev(i);
            let lambda_x_defect = lambda[i];
            let lambda_x_defect_prev = lambda[prev_i];
            let lambda_rate = lambda[5 * n + i];
            let lambda_rate_prev = lambda[5 * n + prev_i];
            let d_i = kappas[ip1] - kappas[i];
            let d_prev = kappas[i] - kappas[prev_i];
            let lambda_n_defect = lambda[n + i];
            let lambda_xi_defect = lambda[2 * n + i];
            let lambda_ellipse = lambda[3 * n + i];
            let lambda_power = lambda[4 * n + i];

            let d2f_dx2 = h_i * 0.75 * ds * xs[i].powf(-2.5);
            let d2e_dx2 = 2.0 * kap.powi(2) * G / (l.powi(2) * g_eff.powi(3))
                - 6.0 * c_l * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(4))
                + 2.0
                    * ((c_d * c_d * g_eff - c_d * c_l * w) / g_eff.powi(3)
                        - 3.0 * c_l * (c_d * g_eff * w - c_l * w * w) / g_eff.powi(4))
                    / m.powi(2);
            let d2p_dx2 = -0.75 * self.params.p_engine * xs[i].powf(-2.5);
            let d2e_du2 = 2.0 / (m.powi(2) * g_eff.powi(2));
            let d2e_dxdu = 2.0 * c_d / (m.powi(2) * g_eff.powi(2))
                - 4.0 * c_l * w / (m.powi(2) * g_eff.powi(3));
            let d2e_dkappa2 = 2.0 * xs[i].powi(2) / (l.powi(2) * g_eff.powi(2));
            let d2e_dkappadx = 4.0 * xs[i] * kap / (l.powi(2) * g_eff.powi(2))
                - 4.0 * xs[i].powi(2) * kap * c_l / (l.powi(2) * g_eff.powi(3));
            let d2f_dxdn = 0.5 * ds * self.curvature[i] * xs[i].powf(-1.5);

            let k = 12 * i;
            vals[k] = obj_factor * d2f_dx2 + lambda_ellipse * d2e_dx2 + lambda_power * d2p_dx2;
            vals[k + 1] = lambda_ellipse * d2e_du2;
            vals[k + 2] = lambda_ellipse * d2e_dxdu;
            vals[k + 3] = lambda_ellipse * d2e_dkappa2
                + 2.0 * (lambda_rate * xs[i] + lambda_rate_prev * xs[prev_i]);
            vals[k + 4] = lambda_ellipse * d2e_dkappadx - 2.0 * lambda_rate * d_i;
            vals[k + 5] = obj_factor * d2f_dxdn;
            vals[k + 6] = lambda_n_defect * 0.5 * ds * self.curvature[i];
            vals[k + 7] = lambda_n_defect * 0.5 * ds * self.curvature[ip1];
            vals[k + 8] = lambda_xi_defect * ds * self.curvature[i];
            // u_i appears in the x-defect starting at i and the one ending at i.
            vals[k + 9] =
                (lambda_x_defect + lambda_x_defect_prev) * ds * self.curvature[i];
            vals[k + 10] = -2.0 * lambda_rate * xs[i];
            vals[k + 11] = 2.0 * lambda_rate_prev * d_prev;
        }
        true
    }
}

/// Solves the minimum-time problem with a free lateral offset (racing line) on the same
/// coarse grid solve_min_time uses, plus track-boundary bounds on that offset. drs_s/
/// drs_open are python_scripts/export_track.py's drs_zones.csv (see solve_min_time) -- pass
/// empty slices for a track with no DRS data. Returns (velocity profile, lateral offset
/// profile, lap time, coarse ds), or Err if Ipopt didn't converge.
pub fn solve_racing_line(
    curvature_full: &[f64],
    full_ds: f64,
    params: &CarParams,
    target_spacing: f64,
    bound_s: &[f64],
    n_left_full: &[f64],
    n_right_full: &[f64],
    drs_s: &[f64],
    drs_open: &[bool],
    omega_max: Option<f64>,
) -> Result<(Vec<f64>, Vec<f64>, f64, f64), String> {
    let (curvature, ds) = coarsen_periodic(curvature_full, full_ds, target_spacing);
    let n = curvature.len();

    let target_s: Vec<f64> = (0..n).map(|i| i as f64 * ds).collect();
    let (n_left, n_right) = resample_bounds(bound_s, n_left_full, n_right_full, &target_s);
    let c_d: Vec<f64> = if drs_s.is_empty() {
        vec![params.c_d; n]
    } else {
        resample_drs(drs_s, drs_open, &target_s)
            .iter()
            .map(|&open| if open { params.c_d_drs } else { params.c_d })
            .collect()
    };

    // Same warm start as solve_min_time -- n=0, xi=0, kappa=kappa_ref is the exact
    // fixed-line problem, so the racing-line solver starts from a known-feasible point.
    let corner_lims = corner_speed_limits(&curvature, params);
    let v_back = backward_pass(&curvature, &corner_lims, params, ds);
    let v_init = forward_pass(&curvature, &v_back, params, ds);

    let x_min = 5.0 * 5.0;
    let x_max = top_speed_drs(params).powi(2);
    let initial_x: Vec<f64> = v_init.iter().map(|v| v.powi(2).clamp(x_min, x_max)).collect();
    let initial_u: Vec<f64> = (0..n)
        .map(|i| {
            let ip1 = (i + 1) % n;
            (initial_x[ip1] - initial_x[i]) / (2.0 * ds)
        })
        .collect();
    let u_bound = 10.0 * G;
    let xi_bound = 0.3; // rad (~17 deg), numerical safety rail on the small-angle approximation, not a physical limit
    let max_abs_kappa = curvature.iter().cloned().fold(0.0_f64, |acc, k| acc.max(k.abs()));
    let kappa_bound = 3.0 * max_abs_kappa + 0.05; // loose box; the ellipse constraint binds first, as with u_bound
    let curvature_ref = curvature.clone(); // kept for the pure-lap-time recompute below, after `curvature` moves into `problem`

    let problem = RacingLineProblem {
        curvature,
        params: params.clone(),
        c_d,
        ds,
        n,
        x_min,
        x_max,
        u_bound,
        xi_bound,
        kappa_bound,
        rate_bound_sq: omega_max.map(|w| (w * ds).powi(2)).unwrap_or(1e20),
        n_left,
        n_right,
        initial_x,
        initial_u,
    };

    let mut ipopt = Ipopt::new(problem).unwrap();
    ipopt.set_option("hessian_approximation", "exact");
    ipopt.set_option("mu_strategy", "adaptive");
    ipopt.set_option("tol", 1e-6);
    ipopt.set_option("max_iter", 3000);
    ipopt.set_option("sb", "yes");
    ipopt.set_option("print_level", 5);

    let SolveResult {
        solver_data: SolverDataMut { solution, .. },
        status,
        ..
    } = ipopt.solve();

    println!("Ipopt status: {:?}", status);
    check_status(status)?;

    let velocity: Vec<f64> = solution.primal_variables[0..n].iter().map(|x| x.sqrt()).collect();
    let n_profile: Vec<f64> = solution.primal_variables[2 * n..3 * n].to_vec();
    let lap_time: f64 = (0..n)
        .map(|i| ds * (1.0 - n_profile[i] * curvature_ref[i]) / velocity[i])
        .sum();
    Ok((velocity, n_profile, lap_time, ds))
}
