//! Tier 1: closed forms and invariants for the minimum-time solvers.
//!
//! These assert *relationships* rather than remembered numbers.

mod common;

use common::{car, circle_curvature, ellipse_usage, steady_state_x, G};
use f1_lap_sim::optimal::{solve_min_time, solve_racing_line, SolveInputs};

/// Radius tight enough that the corner is grip-limited rather than power-limited:
/// `corner_speed_limits` only bites while `kappa > mu_lat * c_l`, i.e. R < ~104 m here.
const RADIUS_M: f64 = 50.0;
const SPACING_M: f64 = 5.0;

fn inputs<'a>(curvature: &'a [f64], params: &'a f1_lap_sim::solver::CarParams) -> SolveInputs<'a> {
    SolveInputs {
        curvature,
        full_ds: 1.0,
        params,
        spacing: SPACING_M,
        drs_s: &[],
        drs_open: &[],
        omega_max: None, // the steering-rate limit must not bind on a constant-radius track
        ers_budget: None,
    }
}

/// The solver's answer on a constant-radius track, against the ellipse root computed by
/// bisection. This exercises the whole chain -- collocation, periodic wrap, friction
/// ellipse, drag, power ceiling and the objective -- against a calculation that shares no
/// code with it.
#[test]
fn constant_radius_matches_the_ellipse_root() {
    let p = car();
    let curvature = circle_curvature(RADIUS_M);
    let (velocity, lap_time, ds) = solve_min_time(&inputs(&curvature, &p)).expect("solve failed");

    let expected_v = steady_state_x(1.0 / RADIUS_M, &p).sqrt();
    let length = velocity.len() as f64 * ds;
    let expected_t = length / expected_v;

    for (i, &v) in velocity.iter().enumerate() {
        assert!(
            (v - expected_v).abs() / expected_v < 1e-3,
            "point {i}: solver {v:.6} m/s vs ellipse root {expected_v:.6} m/s"
        );
    }
    assert!(
        (lap_time - expected_t).abs() / expected_t < 1e-3,
        "lap time {lap_time:.6}s vs {expected_t:.6}s over {length:.1} m"
    );
}

/// With no room to move, the racing-line problem *is* the fixed-line problem: n = 0, xi = 0,
/// kappa = kappa_ref is a feasible point of one and the whole of the other. Any disagreement
/// means a bound, a defect or a derivative differs between the two formulations (found for ERS budget).
#[test]
fn zero_width_reduces_to_the_fixed_line() {
    let p = car();
    let curvature = circle_curvature(RADIUS_M);
    let inp = inputs(&curvature, &p);
    let n_full = curvature.len();
    let bound_s: Vec<f64> = (0..n_full).map(|i| i as f64).collect();
    let zero = vec![0.0; n_full];

    let (_, fixed_time, _) = solve_min_time(&inp).expect("fixed-line solve failed");
    let (_, _, racing_time, _) =
        solve_racing_line(&inp, &bound_s, &zero, &zero).expect("racing-line solve failed");

    assert!(
        (racing_time - fixed_time).abs() < 1e-6,
        "zero-width racing line {racing_time:.9}s != fixed line {fixed_time:.9}s"
    );
}

/// Widening the corridor can only ever add feasible paths, so the lap time must not rise.
/// A sign error in the bounds, or offsets swapped between n_left and n_right, breaks this.
#[test]
fn wider_corridor_is_never_slower() {
    let p = car();
    let curvature = circle_curvature(RADIUS_M);
    let inp = inputs(&curvature, &p);
    let n_full = curvature.len();
    let bound_s: Vec<f64> = (0..n_full).map(|i| i as f64).collect();

    let mut previous = f64::INFINITY;
    for half_width in [0.0, 1.0, 2.0, 4.0] {
        let left = vec![half_width; n_full];
        let right = vec![-half_width; n_full];
        let (_, _, t, _) =
            solve_racing_line(&inp, &bound_s, &left, &right).expect("racing-line solve failed");
        assert!(
            t <= previous + 1e-6,
            "half-width {half_width} m gave {t:.6}s, slower than the narrower corridor's {previous:.6}s"
        );
        previous = t;
    }
}

/// Same inputs, same answer. Cross-run comparisons in results/validation_log.csv are only
/// meaningful if this holds.
#[test]
fn solving_twice_gives_the_same_answer() {
    let p = car();
    let curvature = circle_curvature(RADIUS_M);
    let inp = inputs(&curvature, &p);

    let (v1, t1, _) = solve_min_time(&inp).expect("first solve failed");
    let (v2, t2, _) = solve_min_time(&inp).expect("second solve failed");

    assert_eq!(t1.to_bits(), t2.to_bits(), "lap time differed between identical solves");
    assert_eq!(v1.len(), v2.len());
    for (i, (a, b)) in v1.iter().zip(&v2).enumerate() {
        assert_eq!(a.to_bits(), b.to_bits(), "velocity differed at point {i}");
    }
}

/// More deployable energy can only help, and none can only hurt. Checks two limiting cases.
#[test]
fn energy_budget_orders_the_lap_times() {
    let p = car();
    // A straight needs power, where a constant-radius corner is grip-limited and barely
    // touches the budget, so use a track with a long flat-out section.
    let mut curvature = circle_curvature(RADIUS_M);
    let n = curvature.len();
    for k in curvature.iter_mut().take(n / 2) {
        *k = 0.0;
    }

    let solve = |budget: Option<f64>| {
        let inp = SolveInputs { ers_budget: budget, ..inputs(&curvature, &p) };
        solve_min_time(&inp).expect("solve failed").1
    };

    let none = solve(Some(0.0));
    let real = solve(Some(4_000_000.0 / 734.0));
    let unlimited = solve(None);

    assert!(real <= none + 1e-6, "a real budget ({real:.4}s) was slower than no budget ({none:.4}s)");
    assert!(
        unlimited <= real + 1e-6,
        "an unlimited budget ({unlimited:.4}s) was slower than a capped one ({real:.4}s)"
    );
}

/// A converged status is not the same as a feasible answer. This evaluates the constraints
/// on the profile the solver actually returned, rather than trusting its exit code.
#[test]
fn returned_profile_satisfies_the_constraints() {
    let p = car();
    let curvature = circle_curvature(RADIUS_M);
    let (velocity, _, ds) = solve_min_time(&inputs(&curvature, &p)).expect("solve failed");
    let n = velocity.len();
    let coarse_kappa = 1.0 / RADIUS_M; // constant, so coarsening leaves it unchanged

    for i in 0..n {
        let x = velocity[i] * velocity[i];
        let x_next = velocity[(i + 1) % n] * velocity[(i + 1) % n];
        // the x-defect integrates dx/ds = 2*u*h, and h = 1 on the fixed line
        let u = (x_next - x) / (2.0 * ds);

        let usage = ellipse_usage(x, u, coarse_kappa, p.c_d, &p);
        assert!(usage <= 1.0 + 1e-6, "point {i}: friction ellipse at {usage:.9} > 1");

        let available = p.p_peak() / x.sqrt() - p.c_d * x;
        assert!(u <= available + 1e-6, "point {i}: demands {u:.6} m/s^2, engine gives {available:.6}");
        assert!(x > 0.0 && (G + p.c_l * x).is_finite(), "point {i}: non-physical speed {x}");
    }
}

/// The steering-rate row bounds |dkappa/ds|*v, so tightening it can only remove paths.
/// Nothing above reaches this constraint: a constant-radius track has dkappa/ds = 0 by
/// construction, and the fixed-line solver has no such row at all (its path is given).
#[test]
fn tighter_steering_rate_is_never_faster() {
    let p = car();
    let curvature = common::two_arc_curvature(45.0, 80.0);
    let n_full = curvature.len();
    let bound_s: Vec<f64> = (0..n_full).map(|i| i as f64).collect();
    let left = vec![3.0; n_full];
    let right = vec![-3.0; n_full];

    let solve = |omega: Option<f64>| {
        let inp = SolveInputs { omega_max: omega, ..inputs(&curvature, &p) };
        solve_racing_line(&inp, &bound_s, &left, &right).expect("solve failed").2
    };

    // These values are chosen to BIND. A cap of 0.05 or even 0.01 leaves this track's lap
    // time bit-identical to no cap at all, so a test using them would pass just as happily
    // with the constraint deleted. The strict inequalities below are the point of the test.
    let free = solve(None);
    let loose = solve(Some(0.004));
    let tight = solve(Some(0.003));

    assert!(
        loose > free + 0.05,
        "the rate cap did not bind: 0.004 gave {loose:.4}s against {free:.4}s uncapped"
    );
    assert!(
        tight > loose + 0.05,
        "tightening the cap did nothing: 0.003 gave {tight:.4}s against 0.004's {loose:.4}s"
    );
}

/// Opening DRS lowers drag, which cannot make the lap slower. This is the only test that
/// reaches the c_d -> c_d_drs switching path and resample_drs.
#[test]
fn drs_never_costs_time() {
    let p = car();
    let curvature = common::corner_and_straight(60.0, 600.0);
    let n_full = curvature.len();

    // DRS open over the second half of the straight, where the car is quickest.
    let drs_s: Vec<f64> = (0..n_full).map(|i| i as f64).collect();
    let drs_open: Vec<bool> = (0..n_full).map(|i| curvature[i] == 0.0 && i > n_full * 3 / 4).collect();
    assert!(drs_open.iter().any(|&o| o), "fixture opened DRS nowhere");

    let closed = solve_min_time(&inputs(&curvature, &p)).expect("solve failed").1;
    let open = solve_min_time(&SolveInputs {
        drs_s: &drs_s,
        drs_open: &drs_open,
        ..inputs(&curvature, &p)
    })
    .expect("solve failed")
    .1;

    // Strict, so that a c_d -> c_d_drs switch that silently never fires fails here rather
    // than passing on an inequality it satisfies by doing nothing.
    assert!(
        open < closed - 1e-3,
        "opening DRS changed nothing: {open:.4}s against {closed:.4}s closed"
    );
}

/// The racing-line solution's own constraints, checked on what it returned. The fixed-line
/// feasibility test above cannot reach these: the corridor bounds, the heading bound and the
/// path curvature only exist in this formulation.
#[test]
fn racing_line_solution_stays_inside_its_bounds() {
    // Mirrors xi_bound in optimal.rs::solve_racing_line. If that constant moves, this should
    // fail and be updated deliberately, load bearing rather than a formality.
    const XI_BOUND: f64 = 0.3;

    let p = car();
    let curvature = common::two_arc_curvature(45.0, 80.0);
    let n_full = curvature.len();
    let bound_s: Vec<f64> = (0..n_full).map(|i| i as f64).collect();
    let (half_left, half_right) = (3.0_f64, -3.0_f64);
    let left = vec![half_left; n_full];
    let right = vec![half_right; n_full];

    let inp = inputs(&curvature, &p);
    let (velocity, n_profile, _, ds) =
        solve_racing_line(&inp, &bound_s, &left, &right).expect("solve failed");

    let (kappa_ref, _) = f1_lap_sim::optimal::coarsen_periodic(&curvature, 1.0, SPACING_M);
    assert_eq!(kappa_ref.len(), n_profile.len());

    let xi = common::heading_from_offsets(&n_profile, &kappa_ref, ds);
    let kappa = common::curvature_from_heading(&xi, &n_profile, &kappa_ref, ds);

    for i in 0..n_profile.len() {
        assert!(
            n_profile[i] >= half_right - 1e-6 && n_profile[i] <= half_left + 1e-6,
            "point {i}: offset {:.6} m is outside the corridor", n_profile[i]
        );
        assert!(
            xi[i].abs() <= XI_BOUND + 1e-6,
            "point {i}: heading {:.6} rad exceeds the {XI_BOUND} rad bound", xi[i]
        );
        assert!(velocity[i] > 0.0, "point {i}: non-physical speed {:.6}", velocity[i]);

        // Necessary condition from the ellipse: the lateral term alone cannot exceed it.
        let x = velocity[i] * velocity[i];
        let lat = (x * kappa[i]).abs() / (p.mu_lat * (G + p.c_l * x));
        assert!(lat <= 1.0 + 1e-4, "point {i}: lateral load at {lat:.6} of the ellipse");
    }
}
