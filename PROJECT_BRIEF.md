# Project Brief: Physics-Based Lap Time Simulator (Rust)

## Purpose of this document

This is a planning and context document for an ongoing summer project. It is meant to be read by Claude Code at the start of each work session so that implementation help stays consistent with the overall plan, even across many small, detailed prompts spread out over time.

**How this project will be worked on:** progressively, one detailed prompt at a time, stage by stage. This document exists so that context doesn't have to be re-explained every session — Claude Code should treat it as the source of truth for scope, sequencing, and rationale, and should flag if a requested change conflicts with the plan below rather than silently going along with scope creep.

---

## Goal

Build a point-mass lap time simulator in Rust that predicts a car's speed profile around a race track, validated against real Formula 1 telemetry data (via FastF1). The end goal is a working artifact with a clear validation story — useful both as a coding-skills project and as preparatory work for a possible motorsport-simulation engineering thesis.

**Definition of done (v1):** given a track's curvature profile and basic car parameters, the simulator outputs a predicted speed-vs-distance trace and total lap time, and this is compared directly against a real lap pulled from FastF1 for the same track, with a written/plotted discussion of where and why the model diverges from reality.

---

## Author background (relevant context for Claude Code)

- Strong in Rust; comfortable with systems-level code, less interested in hand-holding on basic syntax.
- Comfortable with Python for data work (FastF1, pandas) — Python is used only as a *data preparation* step, not for the simulator itself.
- CS engineering student; has already worked through the conceptual side of point-mass car models, velocity-pass algorithms, and track curvature representation.
- Prefers working iteratively and in detail — small, focused prompts rather than "build the whole thing." Wants to understand each piece, not just get working code.
- Has previously been flagged (in other contexts) for satisfying test harnesses without satisfying the underlying conceptual intent — for this project, correctness should mean "the physics is right," not just "the code compiles and runs."

---

## Why Rust + Python split

- **Rust**: the simulator core (track model, velocity solver, lap time integration). This is the part meant to be a serious systems/numerical-code exercise.
- **Python (one-time/offline step)**: pulling real telemetry from FastF1 and exporting it to CSV/JSON for Rust to consume. Not meant to grow into a second parallel implementation — Python stays a thin data-export layer.

---

## Stages

Each stage should be tackled as its own sequence of detailed prompts. Do not jump ahead to a later stage's implementation until the current stage has a working, validated result.

### Stage 1 — Track model

- Represent a track as a sequence of points along its centerline, with arc-length `s` and local curvature `κ(s)`.
- Source the centerline from real data: extract X/Y position channel from a real FastF1 lap (Python export step), then smooth/resample it in Rust (or in the export step — to be decided when we get there).
- Output: a `κ(s)` function/array usable by Stage 2.
- Open questions to resolve during implementation: how to handle noisy GPS-like position data, what smoothing method to use, how finely to discretize `s`.

### Stage 2 — Velocity profile solver (two-pass algorithm)

- Forward pass: maximum acceleration achievable at each point, limited by available traction/engine power.
- Backward pass: latest possible braking into each corner, limited by max corner speed from the friction circle constraint: `v_max = sqrt(μ·g / κ)`.
- Combine: take the pointwise minimum of the two passes.
- Core numerical/Rust practice piece — iterative array-based computation, careful handling of curvature singularities (straights where κ ≈ 0).

### Stage 3 — Lap time + validation

- Integrate the velocity profile over track distance to get total lap time.
- Compare the simulated speed-vs-distance trace against the real FastF1 lap for the same track.
- Produce a plot (likely exported to CSV/JSON from Rust, plotted in Python/matplotlib for convenience) and a short written discussion of divergence: where the model overestimates/underestimates speed, and why (missing aero, missing tire degradation, simplified friction model, etc.)
- This validation discussion is a first-class deliverable, not an afterthought — it's the part that's actually useful for thesis framing.

### Stage 4 (stretch) — Refined car model

- Add a simple aero model: downforce and drag both scale with v², downforce modifies the effective friction circle.
- Add gear/engine power limits to the forward pass (more realistic acceleration curve than a flat max).
- Sensitivity analysis: e.g., lap time delta per kg of downforce, per % of mechanical grip — framed as the kind of question a thesis promotor would find compelling.

---

## Explicitly out of scope for v1

- Tire degradation modeling over a stint (mentioned only as a *known limitation* to discuss in Stage 3's validation writeup, not something to implement).
- Multi-car race simulation, strategy, pit stops — that's a separate project idea, not part of this one.
- Building a GUI or web frontend. CLI + file output is sufficient.
- Reimplementing FastF1's functionality in Rust. Python stays the data-export layer (see "Why Rust + Python split" above).

---

## Working style for Claude Code sessions

- Expect prompts to arrive piecemeal — e.g. "let's implement the curvature smoothing function" rather than "build Stage 1." Treat each prompt as advancing one piece of the stage it belongs to.
- When something in a prompt seems to skip ahead of the current stage or contradict the stated scope, flag it rather than just complying.
- Favor correctness of the underlying physics/numerics over merely passing whatever ad hoc check is requested — if a requested implementation would technically "work" but get the physics wrong (e.g., ignoring the friction circle constraint, mishandling κ ≈ 0 on straights), say so.
- Real validation data (from FastF1) should be used wherever possible rather than synthetic/toy data, since the validation comparison in Stage 3 is a core deliverable, not a nice-to-have.

---

## Useful background already established (not to redo)

- Point-mass car model and velocity-pass algorithm: conceptually scoped already (see Stage 2).
- Track curvature representation: conceptually scoped already (see Stage 1).
- FastF1 has already been identified as the data source for validation.
- A related prior project, `Formula-One-Regulations-Analysis` (Python/Dash, on GitHub), already does F1 telemetry analysis and could be a source of reusable FastF1-handling code/snippets for the export step, though it is a separate codebase and not part of this repo.

---

## Future direction: universal model (post-v1)

Not current scope — v1 is still single-track (Singapore), single-driver (Hamilton 2018 Q), and the stages above take priority. This section exists so the idea isn't lost: once v1 is validated and Stage 4's sensitivity analysis is done, a natural next question is whether the model generalizes beyond the one lap it's been tuned against, rather than only reproducing it. Surfaced during a v1 accuracy review (found while checking why the model matched Hamilton's lap as well as it did — `v_top` turned out to be read directly from his telemetry rather than derived from the car's own power/drag balance; fixed, see below). Roughly in priority order:

1. **Generalization test.** Run the current fixed parameter set (`μ_lat`, `μ_lon`, `c_l`, `c_d`, `p_engine`, `a_brake`) against a second, independent track/driver lap, unretuned. This is the cheapest way to find out whether the current ~4% error reflects real model accuracy or a Singapore-specific fit — do this before investing in anything else below.
2. **Use `data/track_boundaries.csv`.** Already exported by `python_scripts/export_boundaries.py` but never read by the Rust simulator. Needed if the goal becomes finding an optimal racing line per track rather than replaying whatever line the source telemetry happened to drive.
3. **Curvature smoothing is numerically backwards.** Currently: interpolate exactly through noisy GPS points, differentiate twice (noise-amplifying), then smooth the result after the fact. A smoothing (not interpolating) spline fit to the raw X/Y — before differentiating — would matter more once feeding in arbitrary tracks of varying GPS quality, not just one hand-tuned circuit.
4. **Parameters are curve-fit, not derived.** `μ_lat=1.63` and the ad hoc brake/traction floor constants (`a_brake × 0.3`, `μ_lon·g_eff × 0.1`) were tuned to match this one lap. Fine for a single-track validation; a universal model needs these to mean something physically, not just happen to work for Singapore.
5. **2D-only, no elevation/banking.** Irrelevant for flat Singapore; will matter for tracks with significant elevation change (Spa, Silverstone, etc.).
6. **Two-pass solver is bang-bang, not optimal.** The friction ellipse couples lateral/longitudinal grip *magnitude* at each point, but both passes always brake/accelerate at the maximum the ellipse allows — there's no modeling of a driver choosing to brake below the limit to carry more speed into an apex (real trail braking). Closing this gap means replacing the two-pass sweep with a minimum-time optimal control formulation (direct collocation / NLP over the whole lap). Biggest lift on this list — do it last, once track/line/parameters are already track-independent, so there's something worth optimizing precisely.
