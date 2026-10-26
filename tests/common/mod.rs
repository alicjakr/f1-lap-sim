//! Shared fixtures for the integration tests.
//!
//! Everything here is synthetic on purpose. `data/` is gitignored, so a fresh clone (and CI)
//! has no real circuits; more importantly, a synthetic track has an answer that can be
//! computed independently of the solver, which a real one does not.

use f1_lap_sim::solver::CarParams;

pub const G: f64 = 9.81; // must match the solver's own constant

/// A representative 2018 car. Values follow api::load_car_params' fixed tyre/powertrain
/// terms and a mid-downforce aero setup; the tests depend on the *relationships*, not on
/// these exact numbers.
pub fn car() -> CarParams {
    CarParams {
        mu_lat: 1.6,
        mu_lon: 1.55,
        c_l: 0.006,
        c_d: 0.0015,
        c_d_drs: 0.0015 * 0.88,
        p_ice: 625_000.0 / 734.0,
        p_mguk: 120_000.0 / 734.0,
    }
}

/// Curvature of a constant-radius track, sampled every metre.
///
/// A circle is the one closed track whose optimal lap is known without a solver: the speed
/// is constant, so every defect is satisfied exactly and the answer reduces to a scalar
/// equation (see `steady_state_x`).
pub fn circle_curvature(radius_m: f64) -> Vec<f64> {
    let n = (2.0 * std::f64::consts::PI * radius_m).round() as usize;
    vec![1.0 / radius_m; n]
}

/// The speed-squared a car settles at on a constant-radius corner, found by bisection on the
/// friction ellipse rather than by the solver.
///
/// At constant speed the longitudinal demand is exactly drag, so with `g_eff = G + c_l*x`:
///
///   (x*kappa / (mu_lat*g_eff))^2 + (c_d*x / (mu_lon*g_eff))^2 = 1
///
/// Both terms rise monotonically with x, so the root is unique and bisection is safe. Note
/// this is *not* solver::corner_speed_limits, which ignores the drag term -- the point of the
/// test is to check the solver against an independent calculation, not against itself.
pub fn steady_state_x(curvature: f64, p: &CarParams) -> f64 {
    let ellipse = |x: f64| {
        let g_eff = G + p.c_l * x;
        let lat = x * curvature / (p.mu_lat * g_eff);
        let lon = p.c_d * x / (p.mu_lon * g_eff);
        lat * lat + lon * lon - 1.0
    };
    let (mut lo, mut hi) = (1.0_f64, 1.0e6_f64);
    assert!(ellipse(lo) < 0.0, "lower bracket is already over the limit");
    assert!(ellipse(hi) > 0.0, "corner is never grip-limited for this car; pick a tighter radius");
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if ellipse(mid) > 0.0 { hi = mid } else { lo = mid }
    }
    0.5 * (lo + hi)
}

/// Lateral and longitudinal loads as a fraction of the friction ellipse, at one point.
pub fn ellipse_usage(x: f64, u: f64, curvature: f64, c_d: f64, p: &CarParams) -> f64 {
    let g_eff = G + p.c_l * x;
    let lat = x * curvature / (p.mu_lat * g_eff);
    let lon = (u + c_d * x) / (p.mu_lon * g_eff);
    lat * lat + lon * lon
}
