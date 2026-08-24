// Minimum-time velocity profile via direct collocation, solved with Ipopt.
//
// Replaces the two-pass sweep's bang-bang assumption (always brake/accelerate at the
// friction ellipse's edge) with a real optimization: the solver is free to choose any
// point inside the ellipse at every point along the lap, so genuine trail braking
// (blending lateral and longitudinal grip through a corner) can emerge on its own
// instead of being hard-coded. The racing line (X/Y path) is still fixed -- lateral
// acceleration is pinned by whatever v the solver picks (a_lat = v^2 * kappa), not a
// free control -- so the only real decision variable is longitudinal acceleration.
//
// State/control: x(s) = v(s)^2 (avoids a 1/v singularity in the dynamics and makes them
// affine in the control), u(s) = a_lon(s), signed: positive = traction, negative = brake.
//   dx/ds = 2u
// discretized with trapezoidal collocation over n points around a closed (periodic) lap:
//   x[i+1] - x[i] - (u[i] + u[i+1])*ds = 0   (indices wrap mod n)
//
// Path constraints per point: friction ellipse (replaces the old a_brake floor entirely --
// that floor only existed to patch the two-pass sweep's inability to blend lat/lon grip,
// which is exactly the limitation this solver removes) and the traction-power ceiling:
//   (x*kappa)^2 / (mu_lat*g_eff)^2 + u^2 / (mu_lon*g_eff)^2 <= 1,   g_eff = G + c_l*x
//   u <= p_engine/sqrt(x) - c_d*x
//
// Objective: minimize sum(ds / v[i]), the same lap-time quantity solver::lap_time measures.
//
// First working version deliberately ran on a coarse grid (~25 m spacing) with Ipopt's
// limited-memory (L-BFGS) Hessian approximation instead of analytic second derivatives.
// That worked up to ~500 variables but stalled well short of full 1 m resolution (~10000
// variables) -- L-BFGS's fixed-size memory window captures proportionally less curvature
// information as problem size grows, and iteration counts were already scaling much faster
// than linearly (131 -> 700 -> 1713 iterations for n = 202 -> 336 -> 504) before it broke
// down entirely. Replaced with the analytic Hessian below to reach full resolution.
//
// The dynamics defect is linear in (x, u), so it contributes nothing to the Hessian. Only
// the objective and the ellipse/power constraints need second derivatives. With
// g = G + c_l*x, K = kappa, L = mu_lat, M = mu_lon:
//   d2f/dx2   = 0.75*ds*x^-2.5                                        (objective, diagonal)
//   d2E/dx2   = -6*c_l*(K^2*x*G/L^2 - c_l*u^2/M^2)/g^4 + 2*K^2*G/(L^2*g^3)
//   d2E/du2   = 2/(M^2*g^2)
//   d2E/dxdu  = -4*c_l*u/(M^2*g^3)
//   d2P/dx2   = -0.75*p_engine*x^-2.5
// (d2P/du2 = d2P/dxdu = 0 -- power is linear in u.) So the Hessian sparsity is unchanged
// from before: 3 entries per point (x diagonal, u diagonal, x-u cross term).

use ipopt::*;

use crate::solver::{backward_pass, corner_speed_limits, forward_pass, top_speed, CarParams};

const G: f64 = 9.81;

/// Subsamples a full-resolution (ds=1m) curvature array down to roughly `target_spacing`
/// meters between points, returning the coarse curvature array and its new ds.
fn coarsen(curvature: &[f64], full_ds: f64, target_spacing: f64) -> (Vec<f64>, f64) {
    let stride = (target_spacing / full_ds).round().max(1.0) as usize;
    let coarse: Vec<f64> = curvature.iter().step_by(stride).cloned().collect();
    (coarse, full_ds * stride as f64)
}

struct MinTimeProblem {
    curvature: Vec<f64>,
    params: CarParams,
    ds: f64,
    n: usize,
    x_min: f64,
    x_max: f64,
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
            x_u[i] = self.x_max;
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
            g[self.n + i] = (a_lat / (self.params.mu_lat * g_eff)).powi(2)
                + (us[i] / (self.params.mu_lon * g_eff)).powi(2);
        }
        for i in 0..self.n {
            g[2 * self.n + i] =
                us[i] - self.params.p_engine / xs[i].sqrt() + self.params.c_d * xs[i];
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
        // Ellipse E(x,u) = K^2*x^2/(L^2*g_eff^2) + u^2/(M^2*g_eff^2), g_eff = G + c_l*x.
        // dE/dx simplifies to (2/g_eff^3) * [K^2*x*G/L^2 - c_l*u^2/M^2] since
        // d/dx[x^2/g_eff^2] = 2x*G/g_eff^3 (the c_l terms cancel: g_eff - x*c_l = G).
        for i in 0..self.n {
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let kap = self.curvature[i];
            let g_eff = self.g_eff(xs[i]);
            let de_dx = (2.0 / g_eff.powi(3))
                * (kap.powi(2) * xs[i] * G / l.powi(2) - c_l * us[i].powi(2) / m.powi(2));
            let de_du = 2.0 * us[i] / (m.powi(2) * g_eff.powi(2));
            vals[k] = de_dx;
            k += 1;
            vals[k] = de_du;
            k += 1;
        }
        // Power P(x,u) = u - p_engine/sqrt(x) + c_d*x
        for i in 0..self.n {
            let dp_dx = 0.5 * self.params.p_engine * xs[i].powf(-1.5) + self.params.c_d;
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
            let kap = self.curvature[i];
            let g_eff = self.g_eff(xs[i]);
            let lambda_ellipse = lambda[self.n + i];
            let lambda_power = lambda[2 * self.n + i];

            let d2f_dx2 = 0.75 * self.ds * xs[i].powf(-2.5);
            let d2e_dx2 = -6.0 * c_l * (kap.powi(2) * xs[i] * G / l.powi(2) - c_l * us[i].powi(2) / m.powi(2))
                / g_eff.powi(4)
                + 2.0 * kap.powi(2) * G / (l.powi(2) * g_eff.powi(3));
            let d2p_dx2 = -0.75 * self.params.p_engine * xs[i].powf(-2.5);
            let d2e_du2 = 2.0 / (m.powi(2) * g_eff.powi(2));
            let d2e_dxdu = -4.0 * c_l * us[i] / (m.powi(2) * g_eff.powi(3));

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
/// curvature array). Returns (velocity profile, lap time, coarse ds).
pub fn solve_min_time(
    curvature_full: &[f64],
    full_ds: f64,
    params: &CarParams,
    target_spacing: f64,
) -> (Vec<f64>, f64, f64) {
    let (curvature, ds) = coarsen(curvature_full, full_ds, target_spacing);
    let n = curvature.len();

    // Warm start from the existing two-pass sweep on the same coarse grid (not periodic,
    // just a reasonable initial guess -- the NLP's own periodicity constraint resolves the
    // seam that the padded-laps trick used to paper over).
    let corner_lims = corner_speed_limits(&curvature, params);
    let v_back = backward_pass(&curvature, &corner_lims, params, ds);
    let v_init = forward_pass(&curvature, &v_back, params, ds);

    let x_min = 5.0 * 5.0; // 5 m/s floor, avoids sqrt(0) and a stationary car mid-lap
    let x_max = top_speed(params).powi(2);
    let initial_x: Vec<f64> = v_init.iter().map(|v| v.powi(2).clamp(x_min, x_max)).collect();
    let initial_u: Vec<f64> = (0..n)
        .map(|i| {
            let ip1 = (i + 1) % n;
            (initial_x[ip1] - initial_x[i]) / (2.0 * ds)
        })
        .collect();
    let u_bound = 10.0 * G; // loose box; the ellipse/power constraints bind first

    let problem = MinTimeProblem {
        curvature,
        params: CarParams {
            mu_lat: params.mu_lat,
            mu_lon: params.mu_lon,
            c_l: params.c_l,
            c_d: params.c_d,
            p_engine: params.p_engine,
        },
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

    let velocity: Vec<f64> = solution.primal_variables[0..n].iter().map(|x| x.sqrt()).collect();
    (velocity, objective_value, ds)
}
