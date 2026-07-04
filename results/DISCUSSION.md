# Validation: Simulated vs Reference Lap

**Reference**: Lewis Hamilton, Singapore GP 2018 Qualifying, pole lap (1:36.015)
**Simulator**: point-mass model, periodic cubic spline track, two-pass velocity solver

---

## Model progression

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

## Conclusion

The final model (1:42.969) is within 7 seconds of the reference pole lap (1:36.015), a 7% error, using only mechanical grip, a simplified aero model, and a power curve. The track structure is reproduced correctly throughout. The primary remaining gap is the friction ellipse, which would require a more coupled solver. The model is a sound foundation for sensitivity analysis: lap time response to changes in `c_l`, `μ_lat`, or `P_specific` can be read directly from a parameter sweep.
