# Validation: Simulated vs Reference Lap

**Reference**: Lewis Hamilton, Singapore GP 2018 Qualifying, pole lap (1:36.015)
**Simulator**: point-mass model, periodic cubic spline track, two-pass velocity solver

---

## Model progression (v1)

The simulator was developed iteratively. Each stage added a physical component and reduced the lap time error.

| Model | Lap time | Delta | Error |
|---|---|---|---|
| Reference (HAM 2018 Q) | 1:36.015 | — | — |
| Baseline (friction circle only) | 2:06.834 | +30.8 s | +32% |
| + Aero (c_l = 0.004) | 2:02.728 | +26.7 s | +28% |
| + Aero (c_l = 0.008) | 1:58.004 | +22.0 s | +23% |
| + Power curve + curvature smoothing | 1:42.969 | +6.95 s | +7% |

---

## Stage 1 — Baseline: friction circle only

Corner speed limit: `v_max = sqrt(μ_lat · g / κ)` with `μ_lat = 1.5`, no aero, flat `a_max = 15 m/s²`.

![Baseline comparison](comparison_baseline.png)

The macro-level track structure is reproduced correctly — speed peaks and valleys appear at the right distances throughout the lap. The dominant failure is corner minimum speeds: the simulator drops to 35–50 km/h where the reference stays at 90–150 km/h. On the straights, both traces reach the same top speed (~322 km/h), as `v_top` is calibrated directly from the reference lap. The 32% lap time error is almost entirely explained by corners being too slow.

---

## Stage 2 — Aerodynamic downforce and drag (c_l = 0.004)

Downforce increases the effective normal force: `N = mg + c_l·v²`, raising the corner speed limit to `v_max = sqrt(μ_lat·g / (κ − μ_lat·c_l))`. Drag reduces available acceleration: `a_net = a_available − c_d·v²`.

![Aero c_l=0.004](comparison_aero_c04.png)

With `c_l = 0.004`, the improvement is modest (~4 seconds). At low corner speeds (10–15 m/s), the downforce contribution `c_l·v²` is small relative to `g`, so the corner speed limit barely changes. The aero effect is speed-dependent — it only matters significantly in medium- and high-speed corners.

---

## Stage 3 — Increased downforce (c_l = 0.008)

Singapore is one of the highest-downforce circuits on the calendar. Doubling `c_l` to 0.008 gives a more realistic representation of the 2018 car's aerodynamic package.

![Aero c_l=0.008](comparison_aero_c08.png)

Corner exits in the first half of the lap (0–2000 m) now track the reference closely. The second half remains worse — the technical section around 2500–4000 m has many closely-spaced corners where combined braking and cornering (not captured by our independent-axis model) is the limiting factor.

---

## Stage 4 — Power curve and curvature smoothing (final model)

The forward pass now uses a traction-limited and power-limited acceleration model:

```
a_traction  = μ_lon · (g + c_l·v²)
a_power     = P_specific / v
a_available = min(a_traction, a_power) − c_d·v²
```

This replaces the flat `a_max` cap. The key gain is that traction couples with downforce — at 100 km/h, effective traction is `μ_lon·(g + c_l·v²) ≈ 24 m/s²` vs the previous flat 15 m/s². Power limits only kick in near top speed (~300 km/h).

Curvature smoothing (10 m moving average) reduces narrow spikes from GPS noise in the input centerline data.

![Final model](comparison_final.png)

Corner exit acceleration now closely matches the reference across most of the lap. The remaining divergence is concentrated in the second half (2500–5000 m), which is the friction ellipse gap — combined braking and cornering reduce available grip in both axes simultaneously, which the independent-axis model cannot capture.

---

## Remaining sources of divergence

### Friction ellipse coupling

The model treats lateral and longitudinal grip as fully independent. In reality, the friction ellipse constraint means braking while cornering (turn-in) and accelerating while cornering (exit) reduce available grip in both directions. Implementing this would require coupling `μ_lat` and `μ_lon` through an ellipse constraint, changing corner speed limits and the two-pass propagation simultaneously.

### Curvature noise

Some speed dips in the simulated trace are not real corners but curvature spikes from GPS noise in the track centerline. The current post-hoc smoothing (10 m moving average on curvature) reduces but does not eliminate them. The correct fix is a smoothing spline fit to the raw GPS geometry, which does not enforce exact interpolation through every noisy point. This was attempted during development but the interpolating spline used here amplifies rather than suppresses geometry noise when the input points are moved, so curvature smoothing remains a known approximation.

### Tire degradation

Not modelled. On a qualifying lap (fresh tires, maximum grip), this is a negligible effect — qualifying is the most appropriate lap to simulate with a constant-grip model.

---

## Conclusion (v1)

The final v1 model (1:42.969) is within 7 seconds of the reference pole lap (1:36.015), a 7% error, using only mechanical grip, a simplified aero model, and a power curve. The track structure is reproduced correctly throughout. The primary remaining gap is the friction ellipse, which would require a more coupled solver. The model is a sound foundation for sensitivity analysis: lap time response to changes in `c_l`, `μ_lat`, or `P_specific` can be read directly from a parameter sweep.

---

---

# V2 — Friction Ellipse

## What changed

The two-pass solver now couples lateral and longitudinal grip through the friction ellipse constraint:

**(a_lat / μ_lat·g_eff)² + (a_lon / μ_lon·g_eff)² ≤ 1**

At each point, the lateral load already carried (`a_lat = v²·|κ|`) reduces the remaining longitudinal budget:

```
a_lon = μ_lon · g_eff · √(1 − (a_lat / μ_lat·g_eff)²)
```

This affects both passes: forward pass (acceleration out of corners) and backward pass (braking into corners). `a_brake` is retained as a flat soft floor (`a_brake × 0.3`), representing that real tire friction ellipses are not perfectly sharp — some longitudinal capacity remains even at high lateral load, regardless of how saturated the lateral grip is.

## Results

| Model | Lap time | Delta | Error |
|---|---|---|---|
| Reference (HAM 2018 Q) | 1:36.015 | — | — |
| V1 final (independent axes) | 1:42.969 | +6.95 s | +7% |
| V2 (friction ellipse) | 1:47.671 | +11.7 s | +12% |

![V2 friction ellipse](comparison_v2_ellipse.png)

## Discussion

The friction ellipse increases the lap time by ~5 seconds relative to v1. This is physically correct: the v1 model allowed full braking while cornering and full acceleration while cornering, which overstated performance in the combined-load zones (turn-in and exit). The ellipse correctly penalises those zones.

The reference lap (1:36) is driven by Hamilton, who optimally uses the friction ellipse — trail braking into corners, blending lateral and longitudinal grip continuously through the corner. The two-pass algorithm cannot replicate this: it assumes a discrete sequence of brake → corner → accelerate. A skilled driver effectively "extends" the friction budget by never being fully at the lateral or longitudinal limit exclusively.

The remaining 12% error therefore has two components:
1. **Physics gap** — parameters and model structure (same issues as v1: curvature noise, no tire model)
2. **Algorithmic gap** — the two-pass cannot recover the time a real driver gains by optimally blending lat/lon grip across the full corner arc; this requires an optimal control formulation (minimum-time problem) rather than a greedy two-pass sweep

The algorithmic gap is fundamental to the two-pass approach and cannot be closed without replacing the solver.

---

# V3 — Recalibration and Periodic Boundary Fix

## What changed: geometry and parameter fixes

Three fixes were made on top of V2:

1. **Removed the interpolating B-spline geometry smoother** that had briefly been introduced to address the curvature-noise problem described above. As already noted in that section, an *interpolating* spline amplifies rather than suppresses noise when input points are perturbed — this was confirmed in practice and reverted. Curvature smoothing remains the simpler post-hoc moving average.
2. **Restored the correct track length.** A bug in the arc-length/resampling path had been producing an incorrect total track length; fixing it brings the resampled length to 5057 m, close to FastF1's own reported lap distance (5026 m, ~0.6% higher — plausibly a geometric-path-length vs. telemetry-distance discrepancy).
3. **Recalibrated car parameters**, including reducing `p_engine` from 1200 to 921 W/kg — the latter corresponds to a real hybrid power unit's specific power (~735 kW combined ICE+MGU-K over ~800 kg race weight), rather than an arbitrary round number.

## Results after recalibration

| Model | Lap time | Delta | Error |
|---|---|---|---|
| Reference (HAM 2018 Q) | 1:36.015 | — | — |
| V2 (friction ellipse) | 1:47.671 | +11.7 s | +12% |
| V3 recalibrated | 1:39.476 | +3.46 s | +3.6% |

The corner-by-corner shape now tracks the reference closely across nearly the entire lap, not just the aggregate lap time — confirmed by plotting the simulated and reference speed traces against distance rather than relying on the total-time figure alone.

## Periodic boundary fix (start/finish line)

Plotting the recalibrated trace revealed one clear artifact: the first ~300 m sat flat at `v_top` (322 km/h) while the reference lap is still accelerating out of the final corner before the line, from ~270 km/h.

**Root cause:** the track model (`track.rs`) is periodic — the spline and curvature array represent a closed loop — but the velocity solver (`solver.rs`) is not. `backward_pass` only ever sets `v[i]` from `v[i+1]`, and `forward_pass` only ever sets `v[i]` from `v[i-1]`; neither touches the boundary between the last sample and the first. So index 0 had no upstream constraint from the corner that actually precedes it on the real track (which is off the end of the array), and was left at the bare corner-speed limit computed from local curvature alone. No amount of outer-loop iteration could fix this, since the connection between the last and first sample simply doesn't exist in an open sweep — it isn't a convergence problem, it's a missing constraint.

**Fix:** `main.rs` now concatenates the curvature array with itself three times (`curvature.iter().cycle().take(n * laps)`) and runs the existing, unmodified two-pass solver over the padded array. By the third copy, the artificial cold-start transient from the very first sample has been overwritten — pass after pass — by the real upstream braking constraint carried over from the previous copy. Only the final lap (`v[(laps-1)*n .. laps*n]`) is kept for lap-time integration and CSV export. A periodicity check (max difference between the final lap and the one before it) printed exactly `0`, confirming the solution has converged to a genuinely periodic steady state and that 3 laps was more than sufficient padding (a single extra lap was already enough in principle, given the artifact only extended ~260 m — consistent with the deceleration distance from `v_top` into the first braking zone).

## Results after the boundary fix

| Model | Lap time | Delta | Error |
|---|---|---|---|
| Reference (HAM 2018 Q) | 1:36.015 | — | — |
| V3 recalibrated (open boundary) | 1:39.476 | +3.46 s | +3.6% |
| V3 final (periodic boundary) | 1:40.060 | +4.05 s | +4.2% |

![V3 periodic boundary fix](comparison_v3_periodic.png)

## Discussion

The periodic fix makes the lap time *worse* (+0.58 s) but more honest: the previous 3.6% figure was partly inflated by a solver artifact that let the car begin the lap already at top speed, for free, rather than carrying speed over realistically from the corner before the line. This is the correct trade — a result that looks better only because of a boundary bug is not a result worth keeping.

At 4.2% error, this remains the best-validated model so far, and the remaining gap is consistent with what V1 and V2 already identified: curvature noise from the moving-average smoother, and the algorithmic gap between the two-pass sweep and a real driver's continuous blending of lateral and longitudinal grip through the friction ellipse (see V2 discussion above). Both remain open items rather than newly discovered issues.
