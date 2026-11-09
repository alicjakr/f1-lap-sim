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

use std::collections::{BTreeSet, HashMap};

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

/// Everything both solvers need about one track and car. Grouped rather than passed
/// positionally so the two entry points stay readable, and so a new modelling input (an
/// energy budget, say) is one field rather than another argument at every call site.
pub struct SolveInputs<'a> {
    /// Curvature of the fitted centerline at full resolution, sampled every `full_ds`.
    pub curvature: &'a [f64],
    pub full_ds: f64,
    pub params: &'a CarParams,
    /// Target spacing between collocation points; the solvers coarsen to it.
    pub spacing: f64,
    /// export_track.py's drs_zones.csv. Empty slices mean "no DRS data": every point then
    /// uses the DRS-closed drag.
    pub drs_s: &'a [f64],
    pub drs_open: &'a [bool],
    /// Steering-rate limit, |dkappa/dt| in 1/(m*s) -- see api::steering_rate_limit. None
    /// leaves the path free to change direction as fast as the grid allows.
    pub omega_max: Option<f64>,
    /// MGU-K deployment allowed per lap, in J/kg. The solver chooses *where* to spend it;
    /// None means unlimited, i.e. peak power all lap.
    pub ers_budget: Option<f64>,
}

/// Deployment to start the solve from: the constant power that spends exactly the budget
/// over the warm-start lap (energy = power * time, so p = E / T), so Ipopt begins inside
/// the budget instead of three times over it.
fn initial_deployment(v_init: &[f64], ds: f64, budget: f64, p_mguk: f64) -> f64 {
    if !budget.is_finite() {
        return p_mguk;
    }
    let lap_time: f64 = v_init.iter().map(|v| ds / v).sum();
    (budget / lap_time).min(p_mguk)
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
    ers_budget: f64, // J/kg per lap; f64::INFINITY when unlimited
    initial_x: Vec<f64>,
    initial_u: Vec<f64>,
    initial_pk: f64,
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
        3 * self.n
    }

    fn bounds(&self, x_l: &mut [Number], x_u: &mut [Number]) -> bool {
        let n = self.n;
        for i in 0..n {
            x_l[i] = self.x_min;
            x_u[i] = self.x_max[i];
            x_l[n + i] = -self.u_bound;
            x_u[n + i] = self.u_bound;
            x_l[2 * n + i] = 0.0; // MGU-K deployment, W/kg
            x_u[2 * n + i] = self.params.p_mguk;
        }
        true
    }

    fn initial_point(&self, x: &mut [Number]) -> bool {
        let n = self.n;
        x[0..n].copy_from_slice(&self.initial_x);
        x[n..2 * n].copy_from_slice(&self.initial_u);
        x[2 * n..3 * n].fill(self.initial_pk);
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
            grad_f[2 * self.n + i] = 0.0;
        }
        true
    }
}

impl ConstrainedProblem for MinTimeProblem {
    // The last row is the lap's energy budget: one constraint over every point, not one
    // per point.
    fn num_constraints(&self) -> usize {
        3 * self.n + 1
    }

    fn num_constraint_jacobian_non_zeros(&self) -> usize {
        11 * self.n
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
        g_l[3 * self.n] = -1e20; // energy deployed over the lap: <= budget
        g_u[3 * self.n] = self.ers_budget;
        true
    }

    fn constraint(&self, x: &[Number], _new_x: bool, g: &mut [Number]) -> bool {
        let n = self.n;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let pks = &x[2 * n..3 * n];
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
                us[i] - (self.params.p_ice + pks[i]) / xs[i].sqrt() + self.c_d[i] * xs[i];
        }
        // Energy = power * time, and time over one step is ds/v, so deployment costs
        // ds*p_k/sqrt(x) per point.
        g[3 * n] = (0..n).map(|i| self.ds * pks[i] / xs[i].sqrt()).sum();
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
        // power row 2n+i: x_i, u_i, and the deployment that raises the ceiling
        for i in 0..self.n {
            let row = (2 * self.n + i) as Index;
            irow[k] = row;
            jcol[k] = i as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = (self.n + i) as Index;
            k += 1;
            irow[k] = row;
            jcol[k] = (2 * self.n + i) as Index;
            k += 1;
        }
        // budget row: every x_i and every p_k
        for i in 0..self.n {
            irow[k] = (3 * self.n) as Index;
            jcol[k] = i as Index;
            k += 1;
            irow[k] = (3 * self.n) as Index;
            jcol[k] = (2 * self.n + i) as Index;
            k += 1;
        }
        true
    }

    fn constraint_jacobian_values(&self, x: &[Number], _new_x: bool, vals: &mut [Number]) -> bool {
        let n = self.n;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let pks = &x[2 * n..3 * n];
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
        // Power P(x,u,p_k) = u - (p_ice + p_k)/sqrt(x) + c_d*x
        for i in 0..self.n {
            let dp_dx = 0.5 * (self.params.p_ice + pks[i]) * xs[i].powf(-1.5) + self.c_d[i];
            vals[k] = dp_dx;
            k += 1;
            vals[k] = 1.0;
            k += 1;
            vals[k] = -xs[i].powf(-0.5);
            k += 1;
        }
        // Budget B = sum(ds*p_k/sqrt(x))
        for i in 0..n {
            vals[k] = -0.5 * self.ds * pks[i] * xs[i].powf(-1.5);
            k += 1;
            vals[k] = self.ds * xs[i].powf(-0.5);
            k += 1;
        }
        true
    }

    // Lower-triangular sparsity: per point i, the x_i/u_i block contributes (i,i), (n+i,n+i),
    // and (n+i,i) -- row=n+i >= col=i always holds since n>i for every i in 0..n. See the
    // module doc comment for the derivatives.
    fn num_hessian_non_zeros(&self) -> usize {
        4 * self.n
    }

    fn hessian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        for i in 0..self.n {
            let k = 4 * i;
            irow[k] = i as Index;
            jcol[k] = i as Index; // (x_i, x_i)
            irow[k + 1] = (self.n + i) as Index;
            jcol[k + 1] = (self.n + i) as Index; // (u_i, u_i)
            irow[k + 2] = (self.n + i) as Index;
            jcol[k + 2] = i as Index; // (u_i, x_i)
            irow[k + 3] = (2 * self.n + i) as Index;
            jcol[k + 3] = i as Index; // (p_k_i, x_i)
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
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let pks = &x[2 * n..3 * n];
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
            let lambda_budget = lambda[3 * n];

            let d2f_dx2 = 0.75 * self.ds * xs[i].powf(-2.5);
            let d2e_dx2 = 2.0 * kap.powi(2) * G / (l.powi(2) * g_eff.powi(3))
                - 6.0 * c_l * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(4))
                + 2.0
                    * ((c_d * c_d * g_eff - c_d * c_l * w) / g_eff.powi(3)
                        - 3.0 * c_l * (c_d * g_eff * w - c_l * w * w) / g_eff.powi(4))
                    / m.powi(2);
            let d2p_dx2 = -0.75 * (self.params.p_ice + pks[i]) * xs[i].powf(-2.5);
            let d2e_du2 = 2.0 / (m.powi(2) * g_eff.powi(2));
            let d2e_dxdu = 2.0 * c_d / (m.powi(2) * g_eff.powi(2))
                - 4.0 * c_l * w / (m.powi(2) * g_eff.powi(3));
            // Budget row: d2B/dx2 = 0.75*ds*p_k*x^-2.5, d2B/dp_k dx = -0.5*ds*x^-1.5.
            let d2b_dx2 = 0.75 * self.ds * pks[i] * xs[i].powf(-2.5);

            let k = 4 * i;
            vals[k] = obj_factor * d2f_dx2
                + lambda_ellipse * d2e_dx2
                + lambda_power * d2p_dx2
                + lambda_budget * d2b_dx2;
            vals[k + 1] = lambda_ellipse * d2e_du2;
            vals[k + 2] = lambda_ellipse * d2e_dxdu;
            vals[k + 3] = lambda_power * 0.5 * xs[i].powf(-1.5)
                - lambda_budget * 0.5 * self.ds * xs[i].powf(-1.5);
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
pub fn solve_min_time(inputs: &SolveInputs) -> Result<(Vec<f64>, f64, f64), String> {
    let SolveInputs { curvature: curvature_full, full_ds, params, spacing: target_spacing,
                      drs_s, drs_open, omega_max, ers_budget } = *inputs;
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
        ers_budget: ers_budget.unwrap_or(f64::INFINITY),
        initial_pk: initial_deployment(&v_init, ds, ers_budget.unwrap_or(f64::INFINITY), params.p_mguk),
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
// formulation (Perantoni & Limebeer), exact in xi -- no small-angle approximation. With
// h(s) = 1 - n(s)*kappa_ref(s) the path stretch from sitting off the reference line, the car
// drives Q = h/cos(xi) of path per unit of reference-line length ds:
//   dn/ds = h*tan(xi)          (dn/dpath = sin(xi))
//   dxi/ds = kappa*Q - kappa_ref
//   dt/ds = Q/v
//   dx/ds = 2*u*Q
//   a_lat = x*kappa
// Earlier versions used tan(xi) = xi and cos(xi) = 1, which needed xi capped at 0.3 rad to be
// defensible and then sat on that cap on every circuit: the cap alone moved lap time by
// several seconds (DISCUSSION.md V12). The remaining bound on xi is only a domain guard --
// tan and 1/cos blow up at 90 degrees -- and the tests assert the solution never reaches it.
//
// Staggered grid: x, u, n, kappa live on the nodes, xi on the segments (xi[i] is the heading
// over segment i -> i+1), and so does Q:  Q_i = h_mid_i / cos(xi_i),  h_mid_i = mean of h at
// nodes i and i+1.
//   x-defect[i]  = x[i+1] - x[i] - ds*Q_i*(u[i] + u[i+1])
//   n-defect[i]  = n[i+1] - n[i] - ds*h_mid_i*tan(xi_i)
//   xi-defect[i] = xi[i] - xi[i-1] - ds*(kappa[i]*(Q_{i-1} + Q_i)/2 - kappa_ref[i])
// so xi is fixed by n's first differences and kappa by xi's. (The xi-defect integrates kappa
// over the path of the dual cell centred on node i, whose length is ds*(Q_{i-1}+Q_i)/2.) With
// all three on the nodes (plain trapezoidal), an alternating xi/kappa pattern satisfies both
// defects at any amplitude while letting kappa undercut kappa_ref at speed-limiting nodes --
// an exact odd-even null space the solver exploits for a spurious lap-time gain. The
// staggered grid removes that mode structurally, so no regularization is needed.
//
// Lap time and energy are trapezoidal in 1/v with Q constant per segment:
//   T = sum_i ds*Q_i*(1/v_i + 1/v_{i+1})/2,   E = sum_i ds*Q_i*(p_i/v_i + p_{i+1}/v_{i+1})/2
// so the energy budget now counts the path actually driven, not the reference line.
//
// Variables, 6n: [x, u, n, xi, kappa, p_k] at offsets 0, n, 2n, 3n, 4n, 5n, where p_k is the
// MGU-K deployment control (W/kg).
// Constraints, 6n+1: [x-defect, n-defect, xi-defect, ellipse, power, steering-rate] at row
// offsets 0, n, 2n, 3n, 4n, 5n, plus the lap's ERS energy budget as a single row at 6n -- the
// only constraint here that is not pointwise.
//
// The steering-rate rows bound (kappa[i+1]-kappa[i])^2 * x[i], i.e. |dkappa/ds|*v, the rate
// the car re-aims itself in *time* (see api::steering_rate_limit). Without it a point mass
// changes direction instantly and the free line exploits flicks no car could drive. Power is
// unchanged from MinTimeProblem; the ellipse is the same functional form with kappa in place
// of the fixed curvature.
// n=0, xi=0, kappa=kappa_ref reproduces MinTimeProblem exactly (Q = 1).
//
// Hessian: see RacingLineProblem::hessian_terms. Everything Q scales is gathered into one
// weight per segment, so the path-stretch dependence is derived once rather than per
// constraint; the pattern comes from that same routine.

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
    ers_budget: f64, // J/kg per lap; f64::INFINITY when unlimited
    initial_pk: f64,
    n_left: Vec<f64>,
    n_right: Vec<f64>,
    initial_x: Vec<f64>,
    initial_u: Vec<f64>,
    // Lower-triangular Hessian pattern, found once by running hessian_terms on a dummy point.
    // Many terms land on the same (row, col), so values are accumulated through this index.
    hess_pairs: Vec<(usize, usize)>,
    hess_index: HashMap<(usize, usize), usize>,
}

/// One segment i -> i+1 of the staggered grid: the path-stretch factor Q = h/cos(xi) (path
/// length driven per unit of reference-line length) and its first derivatives.
#[derive(Clone, Copy)]
struct Seg {
    h: f64,    // mean of 1 - n*kappa_ref over the segment's two nodes
    s: f64,    // sin(xi)
    c: f64,    // cos(xi)
    q: f64,    // h / c
    q_ni: f64, // dQ/dn_i
    q_nj: f64, // dQ/dn_{i+1}
    q_xi: f64, // dQ/dxi_i
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

    fn seg(&self, i: usize, ns: &[f64], xis: &[f64]) -> Seg {
        let j = self.next(i);
        let (ki, kj) = (self.curvature[i], self.curvature[j]);
        let h = 1.0 - 0.5 * (ns[i] * ki + ns[j] * kj);
        let (s, c) = xis[i].sin_cos();
        Seg { h, s, c, q: h / c, q_ni: -0.5 * ki / c, q_nj: -0.5 * kj / c, q_xi: h * s / (c * c) }
    }

    fn segs(&self, x: &[f64]) -> Vec<Seg> {
        let n = self.n;
        (0..n).map(|i| self.seg(i, &x[2 * n..3 * n], &x[3 * n..4 * n])).collect()
    }

    /// Every second-derivative term of the Lagrangian obj_factor*f + sum(lambda*g), emitted
    /// as (var, var, value). The same routine builds the sparsity pattern (on a dummy point)
    /// and the values, so the two cannot disagree about which entries exist.
    ///
    /// Variable blocks: x, u, n, xi, kappa, p_k at offsets 0, n, 2n, 3n, 4n, 5n.
    ///
    /// The path-stretch factor Q_i = h_i/cos(xi_i) of each segment multiplies a weight V_i
    /// that gathers everything Q_i scales: the lap-time and energy sums, the speed defect, and
    /// the heading defects of the two nodes either side. So that part of the Lagrangian is
    /// L_Q = sum_i Q_i*V_i, whose Hessian is
    ///   Q_ab*V  +  Q_a*V_b  +  Q_b*V_a  +  Q*V_ab
    /// over Q's variables (n_i, n_{i+1}, xi_i) and V's (x, u, p_k, kappa at i and i+1).
    fn hessian_terms(
        &self,
        x: &[f64],
        obj_factor: f64,
        lambda: &[f64],
        emit: &mut dyn FnMut(usize, usize, f64),
    ) {
        let n = self.n;
        let ds = self.ds;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let kappas = &x[4 * n..5 * n];
        let pks = &x[5 * n..6 * n];
        let segs = self.segs(x);
        let l_bud = lambda[6 * n];
        let (ix, iu, i_n, ixi, ik, ip) = (
            |i: usize| i,
            |i: usize| n + i,
            |i: usize| 2 * n + i,
            |i: usize| 3 * n + i,
            |i: usize| 4 * n + i,
            |i: usize| 5 * n + i,
        );

        // Terms that stay local to a node: friction ellipse, power ceiling, steering rate.
        for i in 0..n {
            let prev_i = self.prev(i);
            let ip1 = self.next(i);
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let c_d = self.c_d[i];
            let kap = kappas[i];
            let g_eff = self.g_eff(xs[i]);
            let w = us[i] + c_d * xs[i];
            let lambda_ellipse = lambda[3 * n + i];
            let lambda_power = lambda[4 * n + i];
            let lambda_rate = lambda[5 * n + i];
            let lambda_rate_prev = lambda[5 * n + prev_i];
            let d_i = kappas[ip1] - kappas[i];
            let d_prev = kappas[i] - kappas[prev_i];

            let d2e_dx2 = 2.0 * kap.powi(2) * G / (l.powi(2) * g_eff.powi(3))
                - 6.0 * c_l * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(4))
                + 2.0
                    * ((c_d * c_d * g_eff - c_d * c_l * w) / g_eff.powi(3)
                        - 3.0 * c_l * (c_d * g_eff * w - c_l * w * w) / g_eff.powi(4))
                    / m.powi(2);
            let d2p_dx2 = -0.75 * (self.params.p_ice + pks[i]) * xs[i].powf(-2.5);
            let d2e_du2 = 2.0 / (m.powi(2) * g_eff.powi(2));
            let d2e_dxdu = 2.0 * c_d / (m.powi(2) * g_eff.powi(2))
                - 4.0 * c_l * w / (m.powi(2) * g_eff.powi(3));
            let d2e_dkappa2 = 2.0 * xs[i].powi(2) / (l.powi(2) * g_eff.powi(2));
            let d2e_dkappadx = 4.0 * xs[i] * kap / (l.powi(2) * g_eff.powi(2))
                - 4.0 * xs[i].powi(2) * kap * c_l / (l.powi(2) * g_eff.powi(3));

            emit(ix(i), ix(i), lambda_ellipse * d2e_dx2 + lambda_power * d2p_dx2);
            emit(iu(i), iu(i), lambda_ellipse * d2e_du2);
            emit(iu(i), ix(i), lambda_ellipse * d2e_dxdu);
            emit(
                ik(i),
                ik(i),
                lambda_ellipse * d2e_dkappa2 + 2.0 * (lambda_rate * xs[i] + lambda_rate_prev * xs[prev_i]),
            );
            emit(ik(i), ix(i), lambda_ellipse * d2e_dkappadx - 2.0 * lambda_rate * d_i);
            emit(ik(ip1), ik(i), -2.0 * lambda_rate * xs[i]);
            emit(ik(i), ix(prev_i), 2.0 * lambda_rate_prev * d_prev);
            emit(ip(i), ix(i), lambda_power * 0.5 * xs[i].powf(-1.5));
        }

        // Terms that run through a segment.
        for i in 0..n {
            let j = self.next(i);
            let sg = segs[i];
            let (ki, kj) = (self.curvature[i], self.curvature[j]);
            let (s, c) = (sg.s, sg.c);
            let l_x = lambda[i];
            let l_off = lambda[n + i];
            let l_head_i = lambda[2 * n + i];
            let l_head_j = lambda[2 * n + j];

            // Offset defect: n_{i+1} - n_i - ds*h*tan(xi).
            emit(ixi(i), i_n(i), l_off * ds * ki / (2.0 * c * c));
            emit(ixi(i), i_n(j), l_off * ds * kj / (2.0 * c * c));
            emit(ixi(i), ixi(i), -l_off * ds * sg.h * 2.0 * s / (c * c * c));

            // V_i: the weight on Q_i.
            let (ci, cj) = (obj_factor + l_bud * pks[i], obj_factor + l_bud * pks[j]);
            let v = 0.5 * ds * (ci * xs[i].powf(-0.5) + cj * xs[j].powf(-0.5))
                - l_x * ds * (us[i] + us[j])
                - 0.5 * ds * (l_head_i * kappas[i] + l_head_j * kappas[j]);
            let v_xi = -0.25 * ds * ci * xs[i].powf(-1.5);
            let v_xj = -0.25 * ds * cj * xs[j].powf(-1.5);
            let v_pi = 0.5 * ds * l_bud * xs[i].powf(-0.5);
            let v_pj = 0.5 * ds * l_bud * xs[j].powf(-0.5);
            let v_u = -ds * l_x;
            let v_ki = -0.5 * ds * l_head_i;
            let v_kj = -0.5 * ds * l_head_j;

            let q_vars = [(i_n(i), sg.q_ni), (i_n(j), sg.q_nj), (ixi(i), sg.q_xi)];
            let v_vars = [
                (ix(i), v_xi),
                (ix(j), v_xj),
                (iu(i), v_u),
                (iu(j), v_u),
                (ip(i), v_pi),
                (ip(j), v_pj),
                (ik(i), v_ki),
                (ik(j), v_kj),
            ];
            for &(a, qa) in &q_vars {
                for &(b, vb) in &v_vars {
                    emit(a, b, qa * vb);
                }
            }
            emit(ixi(i), i_n(i), -0.5 * ki * s / (c * c) * v);
            emit(ixi(i), i_n(j), -0.5 * kj * s / (c * c) * v);
            emit(ixi(i), ixi(i), sg.h * (1.0 + s * s) / (c * c * c) * v);

            emit(ix(i), ix(i), sg.q * 0.375 * ds * ci * xs[i].powf(-2.5));
            emit(ip(i), ix(i), sg.q * -0.25 * ds * l_bud * xs[i].powf(-1.5));
            emit(ix(j), ix(j), sg.q * 0.375 * ds * cj * xs[j].powf(-2.5));
            emit(ip(j), ix(j), sg.q * -0.25 * ds * l_bud * xs[j].powf(-1.5));
        }
    }

    fn with_hessian_pattern(mut self) -> Self {
        let n = self.n;
        // Any strictly positive point will do: only which (row, col) pairs get emitted matters.
        let mut x = vec![0.0; 6 * n];
        x[0..n].copy_from_slice(&self.initial_x);
        x[n..2 * n].copy_from_slice(&self.initial_u);
        x[4 * n..5 * n].copy_from_slice(&self.curvature);
        x[5 * n..6 * n].fill(self.initial_pk);
        let lambda = vec![1.0; 6 * n + 1];
        let mut pairs = BTreeSet::new();
        self.hessian_terms(&x, 1.0, &lambda, &mut |a, b, _| {
            pairs.insert((a.max(b), a.min(b)));
        });
        self.hess_pairs = pairs.into_iter().collect();
        self.hess_index = self.hess_pairs.iter().enumerate().map(|(k, &p)| (p, k)).collect();
        self
    }
}

impl BasicProblem for RacingLineProblem {
    fn num_variables(&self) -> usize {
        6 * self.n
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
            x_l[5 * n + i] = 0.0; // MGU-K deployment, W/kg
            x_u[5 * n + i] = self.params.p_mguk;
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
            x[5 * n + i] = self.initial_pk;
        }
        true
    }

    // Lap time: sum over segments of Q_i*ds * mean(1/v) -- trapezoidal in 1/v, with the path
    // stretch Q_i constant on the segment.
    fn objective(&self, x: &[Number], _new_x: bool, obj: &mut Number) -> bool {
        let n = self.n;
        let segs = self.segs(x);
        *obj = (0..n)
            .map(|i| {
                let j = self.next(i);
                self.ds * segs[i].q * 0.5 * (x[i].powf(-0.5) + x[j].powf(-0.5))
            })
            .sum();
        true
    }

    fn objective_grad(&self, x: &[Number], _new_x: bool, grad_f: &mut [Number]) -> bool {
        let n = self.n;
        let segs = self.segs(x);
        grad_f.fill(0.0);
        for i in 0..n {
            let j = self.next(i);
            let sg = segs[i];
            let wf = 0.5 * (x[i].powf(-0.5) + x[j].powf(-0.5));
            grad_f[i] += -0.25 * self.ds * sg.q * x[i].powf(-1.5);
            grad_f[j] += -0.25 * self.ds * sg.q * x[j].powf(-1.5);
            grad_f[2 * n + i] += self.ds * sg.q_ni * wf;
            grad_f[2 * n + j] += self.ds * sg.q_nj * wf;
            grad_f[3 * n + i] += self.ds * sg.q_xi * wf;
        }
        true
    }
}

impl ConstrainedProblem for RacingLineProblem {
    // The last row is the lap's energy budget: one constraint over every point.
    fn num_constraints(&self) -> usize {
        6 * self.n + 1
    }

    // x-defect 7, offset defect 3, heading defect 6, ellipse 3, power 3, steering rate 3 per
    // node, plus the budget row's four entries per node.
    fn num_constraint_jacobian_non_zeros(&self) -> usize {
        29 * self.n
    }

    fn constraint_bounds(&self, g_l: &mut [Number], g_u: &mut [Number]) -> bool {
        let n = self.n;
        for i in 0..n {
            g_l[i] = 0.0; // x-defect: equality
            g_u[i] = 0.0;
            g_l[n + i] = 0.0; // offset defect: equality
            g_u[n + i] = 0.0;
            g_l[2 * n + i] = 0.0; // heading defect: equality
            g_u[2 * n + i] = 0.0;
            g_l[3 * n + i] = -1e20; // friction ellipse: <= 1
            g_u[3 * n + i] = 1.0;
            g_l[4 * n + i] = -1e20; // power ceiling: <= 0
            g_u[4 * n + i] = 0.0;
            g_l[5 * n + i] = -1e20; // steering rate: (dkappa)^2 * x <= (omega*ds)^2
            g_u[5 * n + i] = self.rate_bound_sq;
        }
        g_l[6 * n] = -1e20; // energy deployed over the lap: <= budget
        g_u[6 * n] = self.ers_budget;
        true
    }

    fn constraint(&self, x: &[Number], _new_x: bool, g: &mut [Number]) -> bool {
        let n = self.n;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let xis = &x[3 * n..4 * n];
        let kappas = &x[4 * n..5 * n];
        let pks = &x[5 * n..6 * n];
        let segs = self.segs(x);
        let ds = self.ds;

        for i in 0..n {
            let j = self.next(i);
            g[i] = xs[j] - xs[i] - ds * segs[i].q * (us[i] + us[j]);
        }
        for i in 0..n {
            let j = self.next(i);
            let ns = &x[2 * n..3 * n];
            g[n + i] = ns[j] - ns[i] - ds * segs[i].h * xis[i].tan();
        }
        for i in 0..n {
            let m = self.prev(i);
            g[2 * n + i] = xis[i] - xis[m]
                - ds * (kappas[i] * 0.5 * (segs[m].q + segs[i].q) - self.curvature[i]);
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
                us[i] - (self.params.p_ice + pks[i]) / xs[i].sqrt() + self.c_d[i] * xs[i];
        }
        for i in 0..n {
            let d = kappas[self.next(i)] - kappas[i];
            g[5 * n + i] = d * d * xs[i];
        }
        // Energy = integral of power over time, dt = Q*ds/v: trapezoidal like the lap time.
        g[6 * n] = (0..n)
            .map(|i| {
                let j = self.next(i);
                ds * segs[i].q
                    * 0.5
                    * (pks[i] * xs[i].powf(-0.5) + pks[j] * xs[j].powf(-0.5))
            })
            .sum();
        true
    }

    fn constraint_jacobian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        let n = self.n;
        let mut k = 0;
        let mut put = |row: usize, col: usize| {
            irow[k] = row as Index;
            jcol[k] = col as Index;
            k += 1;
        };

        // x-defect row i: x_i, x_{i+1}, u_i, u_{i+1}, n_i, n_{i+1}, xi_i
        for i in 0..n {
            let j = self.next(i);
            for col in [i, j, n + i, n + j, 2 * n + i, 2 * n + j, 3 * n + i] {
                put(i, col);
            }
        }
        // offset-defect row n+i: n_i, n_{i+1}, xi_i
        for i in 0..n {
            let j = self.next(i);
            for col in [2 * n + i, 2 * n + j, 3 * n + i] {
                put(n + i, col);
            }
        }
        // heading-defect row 2n+i: xi_i, xi_{i-1}, kappa_i, n_{i-1}, n_i, n_{i+1}
        for i in 0..n {
            let m = self.prev(i);
            let j = self.next(i);
            for col in [3 * n + i, 3 * n + m, 4 * n + i, 2 * n + m, 2 * n + i, 2 * n + j] {
                put(2 * n + i, col);
            }
        }
        // ellipse row 3n+i: x_i, u_i, kappa_i
        for i in 0..n {
            for col in [i, n + i, 4 * n + i] {
                put(3 * n + i, col);
            }
        }
        // power row 4n+i: x_i, u_i, p_k_i
        for i in 0..n {
            for col in [i, n + i, 5 * n + i] {
                put(4 * n + i, col);
            }
        }
        // steering-rate row 5n+i: kappa_i, kappa_{i+1}, x_i
        for i in 0..n {
            for col in [4 * n + i, 4 * n + self.next(i), i] {
                put(5 * n + i, col);
            }
        }
        // budget row: x_i, p_k_i, n_i, xi_i for every node
        for i in 0..n {
            for col in [i, 5 * n + i, 2 * n + i, 3 * n + i] {
                put(6 * n, col);
            }
        }
        true
    }

    fn constraint_jacobian_values(&self, x: &[Number], _new_x: bool, vals: &mut [Number]) -> bool {
        let n = self.n;
        let ds = self.ds;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let kappas = &x[4 * n..5 * n];
        let pks = &x[5 * n..6 * n];
        let segs = self.segs(x);
        let mut k = 0;
        let mut put = |v: f64| {
            vals[k] = v;
            k += 1;
        };

        for i in 0..n {
            let j = self.next(i);
            let sg = segs[i];
            let usum = us[i] + us[j];
            put(-1.0);
            put(1.0);
            put(-ds * sg.q);
            put(-ds * sg.q);
            put(-ds * sg.q_ni * usum);
            put(-ds * sg.q_nj * usum);
            put(-ds * sg.q_xi * usum);
        }
        for (i, sg) in segs.iter().enumerate() {
            let t = sg.s / sg.c;
            put(-1.0 + 0.5 * ds * self.curvature[i] * t);
            put(1.0 + 0.5 * ds * self.curvature[self.next(i)] * t);
            put(-ds * sg.h / (sg.c * sg.c));
        }
        for i in 0..n {
            let (si, sm) = (segs[i], segs[self.prev(i)]);
            put(1.0 - 0.5 * ds * kappas[i] * si.q_xi);
            put(-1.0 - 0.5 * ds * kappas[i] * sm.q_xi);
            put(-0.5 * ds * (sm.q + si.q));
            put(-0.5 * ds * kappas[i] * sm.q_ni);
            put(-0.5 * ds * kappas[i] * (sm.q_nj + si.q_ni));
            put(-0.5 * ds * kappas[i] * si.q_nj);
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
            put(2.0 * kap.powi(2) * xs[i] * G / (l.powi(2) * g_eff.powi(3))
                + 2.0 * w * (c_d * g_eff - w * c_l) / (m.powi(2) * g_eff.powi(3)));
            put(2.0 * w / (m.powi(2) * g_eff.powi(2)));
            put(2.0 * xs[i].powi(2) * kap / (l.powi(2) * g_eff.powi(2)));
        }
        for i in 0..n {
            put(0.5 * (self.params.p_ice + pks[i]) * xs[i].powf(-1.5) + self.c_d[i]);
            put(1.0);
            put(-xs[i].powf(-0.5));
        }
        for i in 0..n {
            let d = kappas[self.next(i)] - kappas[i];
            put(-2.0 * d * xs[i]);
            put(2.0 * d * xs[i]);
            put(d * d);
        }
        // Budget B = sum_i ds*Q_i*W_i, W_i = mean over the segment's nodes of p_k/sqrt(x).
        let w_seg: Vec<f64> = (0..n)
            .map(|i| {
                let j = self.next(i);
                0.5 * (pks[i] * xs[i].powf(-0.5) + pks[j] * xs[j].powf(-0.5))
            })
            .collect();
        for i in 0..n {
            let m = self.prev(i);
            let q_sum = segs[m].q + segs[i].q;
            put(-0.25 * ds * pks[i] * xs[i].powf(-1.5) * q_sum);
            put(0.5 * ds * xs[i].powf(-0.5) * q_sum);
            put(ds * (segs[i].q_ni * w_seg[i] + segs[m].q_nj * w_seg[m]));
            put(ds * segs[i].q_xi * w_seg[i]);
        }
        true
    }

    fn num_hessian_non_zeros(&self) -> usize {
        self.hess_pairs.len()
    }

    fn hessian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        for (k, &(r, c)) in self.hess_pairs.iter().enumerate() {
            irow[k] = r as Index;
            jcol[k] = c as Index;
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
        vals.fill(0.0);
        let index = &self.hess_index;
        self.hessian_terms(x, obj_factor, lambda, &mut |a, b, v| {
            vals[index[&(a.max(b), a.min(b))]] += v;
        });
        true
    }
}

/// Solves the minimum-time problem with a free lateral offset (racing line) on the same
/// coarse grid solve_min_time uses, plus track-boundary bounds on that offset. drs_s/
/// drs_open are python_scripts/export_track.py's drs_zones.csv (see solve_min_time) -- pass
/// empty slices for a track with no DRS data. Returns (velocity profile, lateral offset
/// profile, lap time, coarse ds), or Err if Ipopt didn't converge.
pub fn solve_racing_line(
    inputs: &SolveInputs,
    bound_s: &[f64],
    n_left_full: &[f64],
    n_right_full: &[f64],
) -> Result<(Vec<f64>, Vec<f64>, f64, f64), String> {
    let SolveInputs { curvature: curvature_full, full_ds, params, spacing: target_spacing,
                      drs_s, drs_open, omega_max, ers_budget } = *inputs;
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
    // Domain guard only: tan(xi) and 1/cos(xi) are singular at 90 degrees. 1.2 rad (69 deg) is
    // far beyond any heading a racing line takes; the tests assert the solution stays clear of it.
    let xi_bound = 1.2;
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
        ers_budget: ers_budget.unwrap_or(f64::INFINITY),
        initial_pk: initial_deployment(&v_init, ds, ers_budget.unwrap_or(f64::INFINITY), params.p_mguk),
        n_left,
        n_right,
        initial_x,
        initial_u,
        hess_pairs: Vec::new(),
        hess_index: HashMap::new(),
    }
    .with_hessian_pattern();

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
    // Same trapezoid as the objective: Q_i*ds per segment, mean of 1/v over its two nodes.
    let xi_profile = &solution.primal_variables[3 * n..4 * n];
    let lap_time: f64 = (0..n)
        .map(|i| {
            let j = (i + 1) % n;
            let h = 1.0 - 0.5 * (n_profile[i] * curvature_ref[i] + n_profile[j] * curvature_ref[j]);
            ds * h / xi_profile[i].cos() * 0.5 * (1.0 / velocity[i] + 1.0 / velocity[j])
        })
        .sum();
    Ok((velocity, n_profile, lap_time, ds))
}

#[cfg(test)]
mod derivative_tests {
    //! The racing-line problem's hand-derived Jacobian and Hessian against central finite
    //! differences, at a point away from every symmetry (nonzero offset, heading, curvature
    //! deviation and deployment, varying DRS). Ipopt's own derivative_test checks the same
    //! thing but only when someone remembers to switch it on and read the output.
    use super::*;

    const N: usize = 14;

    /// Small deterministic generator in [lo, hi); no dependency needed for a test fixture.
    fn lcg(state: &mut u64, lo: f64, hi: f64) -> f64 {
        *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        lo + (hi - lo) * ((*state >> 11) as f64 / (1u64 << 53) as f64)
    }

    fn problem() -> (RacingLineProblem, Vec<f64>, Vec<f64>) {
        let params = CarParams {
            mu_lat: 1.6,
            mu_lon: 1.55,
            c_l: 0.006,
            c_d: 0.0015,
            c_d_drs: 0.0013,
            p_ice: 625_000.0 / 734.0,
            p_mguk: 120_000.0 / 734.0,
        };
        let mut st = 7u64;
        let curvature: Vec<f64> = (0..N)
            .map(|i| 0.012 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).sin() + 0.004)
            .collect();
        let c_d: Vec<f64> = (0..N).map(|i| if i % 3 == 0 { params.c_d_drs } else { params.c_d }).collect();
        let mut x = vec![0.0; 6 * N];
        for i in 0..N {
            x[i] = lcg(&mut st, 300.0, 900.0);
            x[N + i] = lcg(&mut st, -5.0, 5.0);
            x[2 * N + i] = lcg(&mut st, -2.0, 2.0);
            x[3 * N + i] = lcg(&mut st, -0.3, 0.3);
            x[4 * N + i] = curvature[i] + lcg(&mut st, -0.004, 0.004);
            x[5 * N + i] = lcg(&mut st, 5.0, 150.0);
        }
        let lambda: Vec<f64> = (0..6 * N + 1).map(|_| lcg(&mut st, 0.2, 1.5)).collect();
        let p = RacingLineProblem {
            curvature,
            params,
            c_d,
            ds: 5.0,
            n: N,
            x_min: 25.0,
            x_max: 10_000.0,
            u_bound: 98.0,
            xi_bound: 1.2,
            kappa_bound: 0.3,
            rate_bound_sq: 1e20,
            ers_budget: 5000.0,
            initial_pk: 50.0,
            n_left: vec![3.0; N],
            n_right: vec![-3.0; N],
            initial_x: x[0..N].to_vec(),
            initial_u: x[N..2 * N].to_vec(),
            hess_pairs: Vec::new(),
            hess_index: HashMap::new(),
        }
        .with_hessian_pattern();
        (p, x, lambda)
    }

    fn jacobian_dense(p: &RacingLineProblem, x: &[f64]) -> Vec<Vec<f64>> {
        let m = p.num_constraints();
        let nnz = p.num_constraint_jacobian_non_zeros();
        let (mut ir, mut jc) = (vec![0; nnz], vec![0; nnz]);
        p.constraint_jacobian_indices(&mut ir, &mut jc);
        let mut v = vec![0.0; nnz];
        p.constraint_jacobian_values(x, true, &mut v);
        let mut d = vec![vec![0.0; 6 * N]; m];
        for k in 0..nnz {
            d[ir[k] as usize][jc[k] as usize] += v[k];
        }
        d
    }

    fn gradient_of_lagrangian(p: &RacingLineProblem, x: &[f64], obj: f64, lambda: &[f64]) -> Vec<f64> {
        let mut g = vec![0.0; 6 * N];
        p.objective_grad(x, true, &mut g);
        let mut out: Vec<f64> = g.iter().map(|v| obj * v).collect();
        for (row, lam) in jacobian_dense(p, x).iter().zip(lambda) {
            for (o, r) in out.iter_mut().zip(row) {
                *o += lam * r;
            }
        }
        out
    }

    fn worst(a: f64, b: f64) -> f64 {
        (a - b).abs() / (1.0 + a.abs().max(b.abs()))
    }

    #[test]
    fn jacobian_and_objective_gradient_match_finite_differences() {
        let (p, x, _) = problem();
        let eps = 1e-6;
        let m = p.num_constraints();
        let jac = jacobian_dense(&p, &x);
        let mut grad = vec![0.0; 6 * N];
        p.objective_grad(&x, true, &mut grad);

        let mut worst_jac = 0.0_f64;
        let mut worst_grad = 0.0_f64;
        for c in 0..6 * N {
            let (mut xp, mut xm) = (x.clone(), x.clone());
            xp[c] += eps;
            xm[c] -= eps;
            let (mut gp, mut gm) = (vec![0.0; m], vec![0.0; m]);
            p.constraint(&xp, true, &mut gp);
            p.constraint(&xm, true, &mut gm);
            for r in 0..m {
                worst_jac = worst_jac.max(worst(jac[r][c], (gp[r] - gm[r]) / (2.0 * eps)));
            }
            let (mut fp, mut fm) = (0.0, 0.0);
            p.objective(&xp, true, &mut fp);
            p.objective(&xm, true, &mut fm);
            worst_grad = worst_grad.max(worst(grad[c], (fp - fm) / (2.0 * eps)));
        }
        assert!(worst_jac < 1e-6, "Jacobian off by {worst_jac:.3e} from finite differences");
        assert!(worst_grad < 1e-6, "objective gradient off by {worst_grad:.3e}");
    }

    #[test]
    fn hessian_matches_finite_differences_everywhere() {
        let (p, x, lambda) = problem();
        let obj = 0.7;
        let eps = 1e-5;
        let nv = 6 * N;

        let nnz = p.num_hessian_non_zeros();
        let (mut ir, mut jc) = (vec![0; nnz], vec![0; nnz]);
        p.hessian_indices(&mut ir, &mut jc);
        let mut v = vec![0.0; nnz];
        p.hessian_values(&x, true, obj, &lambda, &mut v);
        let mut analytic = vec![vec![0.0; nv]; nv];
        for k in 0..nnz {
            let (r, c) = (ir[k] as usize, jc[k] as usize);
            assert!(r >= c, "entry ({r},{c}) is above the diagonal");
            analytic[r][c] += v[k];
            if r != c {
                analytic[c][r] += v[k];
            }
        }

        // Dense finite-difference Hessian: any true nonzero missing from the pattern shows up
        // here as a mismatch against the analytic zero.
        let mut worst_h = 0.0_f64;
        let mut at = (0, 0);
        for c in 0..nv {
            let (mut xp, mut xm) = (x.clone(), x.clone());
            xp[c] += eps;
            xm[c] -= eps;
            let gp = gradient_of_lagrangian(&p, &xp, obj, &lambda);
            let gm = gradient_of_lagrangian(&p, &xm, obj, &lambda);
            for r in 0..nv {
                let err = worst(analytic[r][c], (gp[r] - gm[r]) / (2.0 * eps));
                if err > worst_h {
                    worst_h = err;
                    at = (r, c);
                }
            }
        }
        assert!(worst_h < 1e-5, "Hessian off by {worst_h:.3e} at {at:?} (blocks x,u,n,xi,kappa,p of {N})");
    }
}
