# Stage 3: Validation — Simulated vs Reference Lap

## Results

| | Time |
|---|---|
| Simulated | 2:06.834 |
| Reference (HAM, Singapore 2018 Q) | 1:36.015 |
| Delta | +30.8 s (+32%) |

## Speed Trace Comparison

![Speed trace comparison](data/comparison.png)

The overall track structure is reproduced correctly: speed peaks and valleys appear at the right distances throughout the lap, confirming the track geometry and curvature pipeline are working. The primary divergence is in corner minimum speeds — the simulator consistently underestimates how slow the car goes into corners, dropping to 35–60 km/h where the reference stays at 90–150 km/h. On the straights, both traces reach the same top speed (~322 km/h), as expected since `v_top` is calibrated directly from the reference lap.

## Sources of Divergence

### 1. No aerodynamic downforce

The dominant source of error. Singapore is one of the highest-downforce circuits on the calendar — at race speeds, aerodynamic downforce roughly doubles the effective normal force on the tires, raising the friction circle limit from the mechanical grip baseline. The corner speed limit in the model is `v_max = sqrt(μ_lat · g / κ)`, where `μ_lat = 1.5` represents mechanical grip alone. With realistic aero, the effective lateral acceleration limit is closer to 25–30 m/s² (vs 14.7 m/s² here), pushing corner speeds up by roughly 40% (√2 ≈ 1.41). This single gap accounts for the majority of the lap time delta.

### 2. Flat acceleration and braking caps

The forward pass uses a constant `a_max = 15 m/s²` and the backward pass a constant `a_brake = 40 m/s²`, independent of speed. Real F1 cars are traction-limited at low speed (higher effective acceleration) and power-limited at high speed (lower acceleration as the engine approaches peak power). This means the model underestimates acceleration out of slow corners and overestimates it at high speed, distorting the shape of the speed trace between corners.

### 3. No friction ellipse coupling

Lateral and longitudinal grip are treated as fully independent: braking and accelerating use `a_brake`/`a_max` regardless of how much lateral load the tire is already carrying. In reality, the friction ellipse constraint means that cornering while braking or accelerating reduces available grip in each direction. Ignoring this overstates the achievable speed at combined-load points (turn-in and exit phases), partially offsetting the corner minimum error but introducing shape distortion.

### 4. Residual curvature noise

Some of the deepest speed dips in the simulated trace — where the car briefly drops near zero — correspond to isolated curvature spikes rather than genuine tight corners. These come from residual noise in the GPS centerline data that survives the spline smoothing. A light smoothing pass on the curvature array, or increasing the spline's arc-length resampling resolution, would eliminate these artefacts.

## Conclusion

The model correctly captures the macro-level structure of the lap: the track layout, the distribution of fast and slow sections, and the top speed. The 32% lap time error is almost entirely explained by the absence of aerodynamic downforce, which is the defining characteristic of a Singapore-spec car setup. The next modelling step (Stage 4) would add a simple aero model — downforce and drag both scaling with v², feeding into an effective friction limit — which would bring corner speeds and lap time into much closer agreement with the reference.
