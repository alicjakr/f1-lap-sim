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

// Regularization weight on xi's point-to-point differences in RacingLineProblem's
// objective (eps*sum((xi_i-xi_{i+1})^2)). Without it, the xi/kappa chain has an exact
// null-space direction -- an alternating xi_i=+A, xi_{i+1}=-A pattern has a trapezoidal
// segment-average of exactly zero regardless of A or ds, so it satisfies the n-defect
// equation for free at any amplitude, while still letting kappa deviate from kappa_ref
// through the xi-defect equation and shape the ellipse constraint favorably. Confirmed
// empirically: with track width pinned to ~0 (forcing n=0), the solver found a lap time
// ~1.4s faster than the fixed-line solver by riding exactly this mode -- a numerical
// artifact, not real physics, since n=0 forces kappa=kappa_ref exactly in the continuous
// problem.
//
// A first attempt penalized raw magnitude (eps*xi_i^2) instead of the difference above.
// That requires a weight large enough to also crush genuine racing-line behavior --
// confirmed empirically on real (non-degenerate) track width, where it produced multi-
// thousand-second "lap times" because a real racing line's legitimate curvature deviation
// from kappa_ref got penalized just as harshly as the spurious checkerboard noise, since
// both can have similar *magnitude*. Penalizing the point-to-point *difference* instead
// targets what actually makes the checkerboard mode exploitable (period-2 alternation
// means a maximal difference of 2A between neighbors) while barely touching a genuine
// racing line's curvature, which changes gradually from point to point.
//
// Calibrated against the pinned-n regression check (data/<slug>_track_boundaries.csv with
// n_left/n_right forced to ~0, so the racing-line solver should reproduce MinTimeProblem's
// lap time exactly). At this weight the remaining gap is ~0.5s at 25m spacing and shrinks
// to ~0.1s at 10m spacing -- confirmed as ordinary trapezoidal discretization error between
// the two problems' slightly different objective/dynamics forms, not a residual exploit:
// raising either weight by 100x past this point changes the gap and the xi/kappa deviation
// magnitudes by less than 10%, the signature of a saturated (not still-being-ridden)
// null-space direction. See KAPPA_REG_WEIGHT below for why it needs its own weight rather
// than sharing this one.
const XI_REG_WEIGHT: f64 = 2000.0;

// Same problem, one level down: with xi's differences regularized smooth, kappa can still
// checkerboard around kappa_ref (kappa_i-kappa_ref_i=+B, kappa_{i+1}-kappa_ref_{i+1}=-B
// still averages to zero in the xi-defect) since kappa never appears in the objective
// either. Same point-to-point-difference shape as XI_REG_WEIGHT; needs its own weight
// (rather than reusing XI_REG_WEIGHT) because kappa has more leverage on the ellipse
// constraint (a_lat = x*kappa directly) than xi does, so the same weight suppresses it far
// less per unit -- confirmed empirically: tightening the n-pin in the regression check by
// 100x shrank xi's residual deviation by ~17x but kappa's by only ~3x. Calibrated the same
// way as XI_REG_WEIGHT.
const KAPPA_REG_WEIGHT: f64 = 2000.0;

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

// --- Racing-line optimization: free lateral offset on top of the fixed-line solver above ---
//
// MinTimeProblem pins the car to the FastF1 driven centerline; only speed is free. Real
// drivers use track width to straighten corners (wide entry -> apex at the inside -> wide
// exit), which raises the effective corner radius and lets them carry more speed -- that's
// the actual "racing line", and RacingLineProblem below adds it as a genuine decision.
//
// State/control adds three arrays to MinTimeProblem's (x, u): lateral offset n(s) (bounded
// by track width from data/<slug>_track_boundaries.csv), heading ξ(s) relative to the
// track tangent, and path curvature κ(s) -- which becomes a free control here, in place of
// the fixed curvature array MinTimeProblem uses directly in its ellipse constraint.
//
// This is the standard heading-based curvilinear formulation from minimum-time lap-sim
// literature (e.g. Perantoni & Limebeer). It was chosen over two alternatives:
//   - kappa_path = kappa_ref/(1-n*kappa_ref) with n alone: rejected, since it can't
//     represent the entry-apex-exit S-shape that's the actual source of a racing line's
//     speed gain -- the benefit comes from how n *changes* across a corner, not just being
//     offset at a point.
//   - the full second-order Frenet curvature formula (needs n''): rejected, since it needs
//     a 3-point stencil and would break the 2-point trapezoidal coupling this whole module
//     is built around.
// The heading state ξ avoids n'' entirely while keeping that same 2-point-per-segment
// structure -- just with three more state/control arrays per point instead of one.
//
// With h(s) = 1 - n(s)*kappa_ref(s) (the path-length stretch factor) and the small-angle
// approximation tan(xi)=xi, cos(xi)=1 (valid for the realistic heading deviations a racing
// line produces; xi is bounded as a validity rail, not a physical limit):
//   dn/ds = h*xi
//   dxi/ds = kappa*h - kappa_ref
//   dt/ds = h/v          (replaces MinTimeProblem's implicit 1/v, i.e. h=1)
//   a_lat = v^2*kappa = x*kappa      (kappa now free, was the fixed curvature array)
//
// Variables, 5n total (n=0 replaces MinTimeProblem's fixed-line case exactly, since n=0,
// xi=0, kappa=kappa_ref makes every new term collapse to MinTimeProblem's): [x(n), u(n),
// n(n), xi(n), kappa(n)] at offsets 0, n, 2n, 3n, 4n.
//
// Constraints, 5n total: [x-defect(n), n-defect(n), xi-defect(n), ellipse(n), power(n)] at
// row offsets 0, n, 2n, 3n, 4n. x-defect and power are unchanged from MinTimeProblem;
// ellipse is the same functional form with kappa replacing the fixed curvature array.
//
// n-defect[i] = n[i+1] - n[i] - ds/2*(h_i*xi_i + h_{i+1}*xi_{i+1})
// xi-defect[i] = xi[i+1] - xi[i] - ds/2*((kappa_i*h_i - kappa_ref_i) + (kappa_{i+1}*h_{i+1} - kappa_ref_{i+1}))
//
// Both are bilinear in their own-index variables (n*xi, kappa*n), so unlike MinTimeProblem
// (where the linear x-defect contributes nothing to the Hessian), these two contribute new
// Hessian terms -- but only at each defect's own two endpoints, never across non-adjacent
// points, so the Hessian sparsity pattern stays "one block per point" as before, just a
// bigger block. New per-point Hessian pairs (K=kappa_i, L=mu_lat, M=mu_lon, g=g_eff):
//   d2E/dK2   = 2*x^2/(L^2*g^2)                                          (ellipse)
//   d2E/dKdx  = 4*x*K/(L^2*g^2) - 4*x^2*K*c_l/(L^2*g^3)                  (ellipse)
//   d2f/dxdn  = 0.5*ds*kappa_ref_i*x^-1.5                                (objective)
//   d2(n-defect)/dn_i dxi_i = ds/2*kappa_ref_i     (per defect touching point i, accumulate
//   d2(xi-defect)/dn_i dkappa_i = ds/2*kappa_ref_i  both the defect starting at i AND the
//                                                    one ending at i -- see hessian_values)

/// Linearly interpolates n_left(s)/n_right(s) from the boundary CSV's own arc-length axis
/// (FastF1's raw Distance channel, via track::load_boundaries) onto the solver's coarse
/// grid `target_s[i] = i*ds`. That axis isn't numerically identical to the curvature
/// array's own arc-length parameterization (the periodic B-spline's, via track::resample)
/// -- both measure the same physical lap distance, just via slightly different paths, and
/// treating them as equivalent here is an accepted approximation (see track::load_boundaries).
fn resample_bounds(bound_s: &[f64], n_left: &[f64], n_right: &[f64], target_s: &[f64]) -> (Vec<f64>, Vec<f64>) {
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

struct RacingLineProblem {
    curvature: Vec<f64>, // kappa_ref
    params: CarParams,
    ds: f64,
    n: usize,
    x_min: f64,
    x_max: f64,
    u_bound: f64,
    xi_bound: f64,
    kappa_bound: f64,
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
        let xis = &x[3 * n..4 * n];
        let kappas = &x[4 * n..5 * n];
        let lap_time: f64 = (0..n)
            .map(|i| self.ds * self.h(x[2 * n + i], i) / x[i].sqrt())
            .sum();
        let reg: f64 = (0..n)
            .map(|i| {
                let ip1 = self.next(i);
                XI_REG_WEIGHT * (xis[i] - xis[ip1]).powi(2)
                    + KAPPA_REG_WEIGHT * (kappas[i] - kappas[ip1]).powi(2)
            })
            .sum();
        *obj = lap_time + reg;
        true
    }

    fn objective_grad(&self, x: &[Number], _new_x: bool, grad_f: &mut [Number]) -> bool {
        let n = self.n;
        let xis = &x[3 * n..4 * n];
        let kappas = &x[4 * n..5 * n];
        for i in 0..n {
            let h_i = self.h(x[2 * n + i], i);
            grad_f[i] = -0.5 * self.ds * h_i * x[i].powf(-1.5);
            grad_f[n + i] = 0.0;
            grad_f[2 * n + i] = -self.ds * self.curvature[i] / x[i].sqrt();
            grad_f[3 * n + i] = 0.0;
            grad_f[4 * n + i] = 0.0;
        }
        // d/d(xi_j) of sum_i eps*(xi_i-xi_{i+1})^2 picks up a term from segment i=j (as the
        // left endpoint) and from segment i=prev(j) (as the right endpoint) -- accumulate
        // both into point j's slot rather than point i's, since this is a per-segment sum.
        for i in 0..n {
            let ip1 = self.next(i);
            let d_xi = 2.0 * XI_REG_WEIGHT * (xis[i] - xis[ip1]);
            grad_f[3 * n + i] += d_xi;
            grad_f[3 * n + ip1] -= d_xi;
            let d_kappa = 2.0 * KAPPA_REG_WEIGHT * (kappas[i] - kappas[ip1]);
            grad_f[4 * n + i] += d_kappa;
            grad_f[4 * n + ip1] -= d_kappa;
        }
        true
    }
}

impl ConstrainedProblem for RacingLineProblem {
    fn num_constraints(&self) -> usize {
        5 * self.n
    }

    fn num_constraint_jacobian_non_zeros(&self) -> usize {
        19 * self.n
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
            g[i] = xs[ip1] - xs[i] - (us[i] + us[ip1]) * self.ds;
        }
        for i in 0..n {
            let ip1 = self.next(i);
            let h_i = self.h(ns[i], i);
            let h_ip1 = self.h(ns[ip1], ip1);
            g[n + i] = ns[ip1] - ns[i] - self.ds / 2.0 * (h_i * xis[i] + h_ip1 * xis[ip1]);
        }
        for i in 0..n {
            let ip1 = self.next(i);
            let h_i = self.h(ns[i], i);
            let h_ip1 = self.h(ns[ip1], ip1);
            g[2 * n + i] = xis[ip1] - xis[i]
                - self.ds / 2.0
                    * ((kappas[i] * h_i - self.curvature[i]) + (kappas[ip1] * h_ip1 - self.curvature[ip1]));
        }
        for i in 0..n {
            let g_eff = self.g_eff(xs[i]);
            let a_lat = xs[i] * kappas[i];
            g[3 * n + i] = (a_lat / (self.params.mu_lat * g_eff)).powi(2)
                + (us[i] / (self.params.mu_lon * g_eff)).powi(2);
        }
        for i in 0..n {
            g[4 * n + i] =
                us[i] - self.params.p_engine / xs[i].sqrt() + self.params.c_d * xs[i];
        }
        true
    }

    fn constraint_jacobian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        let n = self.n;
        let mut k = 0;

        // x-defect row i: d/dx_i, d/dx_{i+1}, d/du_i, d/du_{i+1}
        for i in 0..n {
            let ip1 = self.next(i);
            let row = i as Index;
            irow[k] = row; jcol[k] = i as Index; k += 1;
            irow[k] = row; jcol[k] = ip1 as Index; k += 1;
            irow[k] = row; jcol[k] = (n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (n + ip1) as Index; k += 1;
        }
        // n-defect row n+i: d/dn_i, d/dxi_i, d/dn_{i+1}, d/dxi_{i+1}
        for i in 0..n {
            let ip1 = self.next(i);
            let row = (n + i) as Index;
            irow[k] = row; jcol[k] = (2 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (3 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + ip1) as Index; k += 1;
            irow[k] = row; jcol[k] = (3 * n + ip1) as Index; k += 1;
        }
        // xi-defect row 2n+i: d/dxi_i, d/dxi_{i+1}, d/dn_i, d/dkappa_i, d/dn_{i+1}, d/dkappa_{i+1}
        for i in 0..n {
            let ip1 = self.next(i);
            let row = (2 * n + i) as Index;
            irow[k] = row; jcol[k] = (3 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (3 * n + ip1) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (4 * n + i) as Index; k += 1;
            irow[k] = row; jcol[k] = (2 * n + ip1) as Index; k += 1;
            irow[k] = row; jcol[k] = (4 * n + ip1) as Index; k += 1;
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
        true
    }

    fn constraint_jacobian_values(&self, x: &[Number], _new_x: bool, vals: &mut [Number]) -> bool {
        let n = self.n;
        let xs = &x[0..n];
        let us = &x[n..2 * n];
        let ns = &x[2 * n..3 * n];
        let xis = &x[3 * n..4 * n];
        let kappas = &x[4 * n..5 * n];
        let mut k = 0;

        for _ in 0..n {
            vals[k] = -1.0; k += 1;
            vals[k] = 1.0; k += 1;
            vals[k] = -self.ds; k += 1;
            vals[k] = -self.ds; k += 1;
        }
        for i in 0..n {
            let ip1 = self.next(i);
            let h_i = self.h(ns[i], i);
            let h_ip1 = self.h(ns[ip1], ip1);
            vals[k] = -1.0 + self.ds / 2.0 * self.curvature[i] * xis[i]; k += 1;
            vals[k] = -self.ds / 2.0 * h_i; k += 1;
            vals[k] = 1.0 + self.ds / 2.0 * self.curvature[ip1] * xis[ip1]; k += 1;
            vals[k] = -self.ds / 2.0 * h_ip1; k += 1;
        }
        for i in 0..n {
            let ip1 = self.next(i);
            let h_i = self.h(ns[i], i);
            let h_ip1 = self.h(ns[ip1], ip1);
            vals[k] = -1.0; k += 1;
            vals[k] = 1.0; k += 1;
            vals[k] = self.ds / 2.0 * kappas[i] * self.curvature[i]; k += 1;
            vals[k] = -self.ds / 2.0 * h_i; k += 1;
            vals[k] = self.ds / 2.0 * kappas[ip1] * self.curvature[ip1]; k += 1;
            vals[k] = -self.ds / 2.0 * h_ip1; k += 1;
        }
        // Ellipse E(x,u,kappa) = kappa^2*x^2/(L^2*g^2) + u^2/(M^2*g^2), g = G + c_l*x.
        for i in 0..n {
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let kap = kappas[i];
            let g_eff = self.g_eff(xs[i]);
            let de_dx = (2.0 / g_eff.powi(3))
                * (kap.powi(2) * xs[i] * G / l.powi(2) - c_l * us[i].powi(2) / m.powi(2));
            let de_du = 2.0 * us[i] / (m.powi(2) * g_eff.powi(2));
            let de_dkappa = 2.0 * xs[i].powi(2) * kap / (l.powi(2) * g_eff.powi(2));
            vals[k] = de_dx; k += 1;
            vals[k] = de_du; k += 1;
            vals[k] = de_dkappa; k += 1;
        }
        for i in 0..n {
            let dp_dx = 0.5 * self.params.p_engine * xs[i].powf(-1.5) + self.params.c_d;
            vals[k] = dp_dx; k += 1;
            vals[k] = 1.0; k += 1;
        }
        true
    }

    // Lower-triangular sparsity, 11 entries per point i (see module doc comment for the new
    // pairs beyond MinTimeProblem's (x,x)/(u,u)/(u,x)): the two new dynamics defects are
    // bilinear in their own-index variables, so every new pair stays local to one point,
    // same as the existing ellipse/power/objective terms. (xi_i,xi_i) and (kappa_i,kappa_i)
    // are the regularization terms' diagonal contribution (see XI_REG_WEIGHT); the last two
    // entries are the regularization terms' cross-point coupling between i and next(i) --
    // the one place this problem needs an off-diagonal (i, i+1) block.
    fn num_hessian_non_zeros(&self) -> usize {
        11 * self.n
    }

    fn hessian_indices(&self, irow: &mut [Index], jcol: &mut [Index]) -> bool {
        let n = self.n;
        for i in 0..n {
            let ip1 = self.next(i);
            let k = 11 * i;
            irow[k] = i as Index; jcol[k] = i as Index; // (x_i, x_i)
            irow[k + 1] = (n + i) as Index; jcol[k + 1] = (n + i) as Index; // (u_i, u_i)
            irow[k + 2] = (n + i) as Index; jcol[k + 2] = i as Index; // (u_i, x_i)
            irow[k + 3] = (4 * n + i) as Index; jcol[k + 3] = (4 * n + i) as Index; // (kappa_i, kappa_i)
            irow[k + 4] = (4 * n + i) as Index; jcol[k + 4] = i as Index; // (kappa_i, x_i)
            irow[k + 5] = (2 * n + i) as Index; jcol[k + 5] = i as Index; // (n_i, x_i)
            irow[k + 6] = (3 * n + i) as Index; jcol[k + 6] = (2 * n + i) as Index; // (xi_i, n_i)
            irow[k + 7] = (4 * n + i) as Index; jcol[k + 7] = (2 * n + i) as Index; // (kappa_i, n_i)
            irow[k + 8] = (3 * n + i) as Index; jcol[k + 8] = (3 * n + i) as Index; // (xi_i, xi_i)
            // (xi_i, xi_{i+1}) cross term -- order by variable index so row >= col holds
            // even across the wraparound segment (i=n-1, ip1=0, where xi_0's index is
            // smaller than xi_{n-1}'s).
            let (xi_a, xi_b) = (3 * n + i, 3 * n + ip1);
            irow[k + 9] = xi_a.max(xi_b) as Index; jcol[k + 9] = xi_a.min(xi_b) as Index;
            let (kap_a, kap_b) = (4 * n + i, 4 * n + ip1);
            irow[k + 10] = kap_a.max(kap_b) as Index; jcol[k + 10] = kap_a.min(kap_b) as Index;
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
        let kappas = &x[4 * n..5 * n];

        for i in 0..n {
            let prev_i = self.prev(i);
            let l = self.params.mu_lat;
            let m = self.params.mu_lon;
            let c_l = self.params.c_l;
            let kap = kappas[i];
            let g_eff = self.g_eff(xs[i]);
            let h_i = self.h(x[2 * n + i], i);

            let lambda_n_defect = lambda[n + i];
            let lambda_xi_defect = lambda[2 * n + i];
            let lambda_ellipse = lambda[3 * n + i];
            let lambda_power = lambda[4 * n + i];
            let lambda_n_defect_prev = lambda[n + prev_i];
            let lambda_xi_defect_prev = lambda[2 * n + prev_i];

            let d2f_dx2 = h_i * 0.75 * self.ds * xs[i].powf(-2.5);
            let d2e_dx2 = -6.0 * c_l * (kap.powi(2) * xs[i] * G / l.powi(2) - c_l * us[i].powi(2) / m.powi(2))
                / g_eff.powi(4)
                + 2.0 * kap.powi(2) * G / (l.powi(2) * g_eff.powi(3));
            let d2p_dx2 = -0.75 * self.params.p_engine * xs[i].powf(-2.5);
            let d2e_du2 = 2.0 / (m.powi(2) * g_eff.powi(2));
            let d2e_dxdu = -4.0 * c_l * us[i] / (m.powi(2) * g_eff.powi(3));
            let d2e_dkappa2 = 2.0 * xs[i].powi(2) / (l.powi(2) * g_eff.powi(2));
            let d2e_dkappadx = 4.0 * xs[i] * kap / (l.powi(2) * g_eff.powi(2))
                - 4.0 * xs[i].powi(2) * kap * c_l / (l.powi(2) * g_eff.powi(3));
            let d2f_dxdn = 0.5 * self.ds * self.curvature[i] * xs[i].powf(-1.5);
            let d2ndefect_dndxi = self.ds / 2.0 * self.curvature[i] * (lambda_n_defect + lambda_n_defect_prev);
            let d2xidefect_dndkappa = self.ds / 2.0 * self.curvature[i] * (lambda_xi_defect + lambda_xi_defect_prev);

            let k = 11 * i;
            // Diagonal regularization coefficients are 4*eps, not 2*eps: each point i picks
            // up a factor of 2 from being the left endpoint of segment i's (xi_i-xi_{i+1})^2
            // term AND another factor of 2 from being the right endpoint of segment
            // prev(i)'s -- see the objective_grad comment for the same accounting.
            vals[k] = obj_factor * d2f_dx2 + lambda_ellipse * d2e_dx2 + lambda_power * d2p_dx2;
            vals[k + 1] = lambda_ellipse * d2e_du2;
            vals[k + 2] = lambda_ellipse * d2e_dxdu;
            vals[k + 3] = lambda_ellipse * d2e_dkappa2 + obj_factor * 4.0 * KAPPA_REG_WEIGHT;
            vals[k + 4] = lambda_ellipse * d2e_dkappadx;
            vals[k + 5] = obj_factor * d2f_dxdn;
            vals[k + 6] = d2ndefect_dndxi;
            vals[k + 7] = d2xidefect_dndkappa;
            vals[k + 8] = obj_factor * 4.0 * XI_REG_WEIGHT;
            vals[k + 9] = obj_factor * -2.0 * XI_REG_WEIGHT;
            vals[k + 10] = obj_factor * -2.0 * KAPPA_REG_WEIGHT;
        }
        true
    }
}

/// Solves the minimum-time problem with a free lateral offset (racing line) on the same
/// coarse grid solve_min_time uses, plus track-boundary bounds on that offset. Returns
/// (velocity profile, lateral offset profile, lap time, coarse ds).
pub fn solve_racing_line(
    curvature_full: &[f64],
    full_ds: f64,
    params: &CarParams,
    target_spacing: f64,
    bound_s: &[f64],
    n_left_full: &[f64],
    n_right_full: &[f64],
) -> (Vec<f64>, Vec<f64>, f64, f64) {
    let (curvature, ds) = coarsen(curvature_full, full_ds, target_spacing);
    let n = curvature.len();

    let target_s: Vec<f64> = (0..n).map(|i| i as f64 * ds).collect();
    let (n_left, n_right) = resample_bounds(bound_s, n_left_full, n_right_full, &target_s);

    // Same warm start as solve_min_time -- n=0, xi=0, kappa=kappa_ref is the exact
    // fixed-line problem, so the racing-line solver starts from a known-feasible point.
    let corner_lims = corner_speed_limits(&curvature, params);
    let v_back = backward_pass(&curvature, &corner_lims, params, ds);
    let v_init = forward_pass(&curvature, &v_back, params, ds);

    let x_min = 5.0 * 5.0;
    let x_max = top_speed(params).powi(2);
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
        xi_bound,
        kappa_bound,
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

    let velocity: Vec<f64> = solution.primal_variables[0..n].iter().map(|x| x.sqrt()).collect();
    let n_profile: Vec<f64> = solution.primal_variables[2 * n..3 * n].to_vec();
    // Ipopt's own objective_value includes the xi/kappa regularization terms (see
    // XI_REG_WEIGHT), which are a solver-internal device to suppress a spurious null-space
    // mode, not part of the actual lap time -- recompute the true value (matching
    // MinTimeProblem's objective exactly when n=0) from the returned solution instead of
    // returning that raw value.
    let lap_time: f64 = (0..n)
        .map(|i| ds * (1.0 - n_profile[i] * curvature_ref[i]) / velocity[i])
        .sum();
    (velocity, n_profile, lap_time, ds)
}
