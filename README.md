# f1-lap-sim

A physics-based F1 lap time simulator. A Rust minimum-time optimal-control solver finds
the fastest velocity profile — and optionally the fastest line within the track's real
width — around a track, given real track geometry and boundaries.

There are two modes, and the difference between them matters. **`racingline` is the more
complete model**: it optimizes the path as well as the speed, and contains the fixed-line
case exactly (a zero lateral offset). **`optimal` is the narrower one**: it pins the car to
the lap a driver actually drove and optimizes only the speed profile.

The narrower mode is the validated one, because everything it needs was measured: **+5.5%
slower than real 2018 pole times** across all 21 circuits of the 2018 calendar. The more
complete mode currently reads **~10.0% faster** than any real lap — not because the
formulation is wrong, but because its extra freedom is under-constrained. Its corridor now
uses satellite-measured track widths, which made the racing line faster. The remaining suspects are a sub-grid weave and a point mass
that changes direction for free. Until that is settled its lap time is a bound, not a
prediction. See V9 in `results/DISCUSSION.md` for the evidence behind both numbers.

## What it does and doesn't model

**Works:**

- **Speed optimization on a known path.** Given the line a driver actually drove, it finds
  the fastest way to drive it — where to brake, how hard, when to get back on power. This
  is the validated part: +5.5% mean vs. real 2018 pole times across all 21 circuits.
- **Combined grip.** Braking and cornering share one friction budget (a friction ellipse),
  so the car can't brake at full force mid-corner.
- **Aerodynamics per circuit.** Downforce and drag are derived from each track's own
  telemetry rather than guessed, and derived from apex speeds and top speed — not from lap
  time, so checking the result against lap times isn't circular. The derivation is also the
  single largest remaining source of error — see below.
- **Power, drag and DRS**, including DRS only where the real car had it open.
- **The hybrid's energy budget.** The ICE runs all lap; the MGU-K's 4 MJ (about 33 s at
  full deployment) is a budget the solver chooses where to spend.
- **A steering-rate limit**, so the path can't change direction faster than the real car did.
- **The solve itself.** Minimum-time optimal control over the whole lap at once, with
  hand-derived derivatives checked against Ipopt's own finite-difference checker, and a
  start/finish line that wraps properly. All 21 circuits converge in about a second.

**Doesn't work / not modelled:**

- **Picking the line is not trustworthy.** `racingline` mode reads ~10.0% faster than any
  real lap. Measured track widths neither fixed this nor caused it; the leading suspects are
  the sub-grid artifact below and the point-mass model's free direction changes.
- **The car is a point mass.** No yaw inertia, no weight transfer, no tire slip angles.
  Measured to be worth under a second on lap time, but it's why a freely chosen line can
  find time a real car couldn't.
- **A sub-grid artifact.** Give the car 1 cm of lateral freedom and it still finds 0.3–1.1 s
  by weaving, because the grip limit is only enforced at the solver's sample points.
- **The downforce coefficient is noisily estimated.** `c_l` is a median over 10–25 apex
  samples whose spread is 2.8–3.9×, and it alone explains about half the variance in the
  fixed-line gap (r = −0.68): ±0.001 in `c_l` is worth ∓1.9 percentage points of lap time.
  Sochi is the worst case at +17.6%.
- **Harvesting isn't modelled.** The MGU-K's 4 MJ per lap is granted rather than earned
  under braking, and deployment can be spent anywhere in the lap.
- **No tire wear, fuel burn, track evolution, elevation or banking.** Reasonable for a
  single qualifying lap, less so for anything else.
- **4 circuits have no measured track width.** Baku and Singapore fall back to OpenStreetMap
  lane counts — the width of the public road, not of the circuit between the barriers — and
  Monaco and Paul Ricard to a flat 7 m. Their racing lines are the least trustworthy here.

## Open questions

Things worth solving, roughly in order of how much they'd change the results:

1. **Estimate `c_l` properly.** It is a median over 10–25 apex samples with a 2.8–3.9×
   spread, drawn from a pool contaminated by curvature-fit artifacts, and it accounts for
   roughly half the remaining fixed-line error. The fix is a robust regression of `a_lat`
   against `v²` over the whole corner population — hundreds of points, not sixteen. Filtering
   the bad samples out does *not* work; V9 explains why.
2. **The weave.** Enforcing the grip limit between sample points, not just at them, would
   close part of the racing-line gap; the rest is the point-mass assumption.
3. **Sochi and Singapore keep a ~+6.5 pp residual** once `c_l` is accounted for — the two
   largest of the 21, against a residual stdev of 3.6 pp. Cause unknown.
4. **A real vehicle model** (yaw dynamics, load transfer, tire slip). Measured as worth
   ≤1 s, so it's for correctness rather than accuracy.
5. **Harvesting.** Deployment is budgeted (V8), but the budget is granted rather than
   recovered under braking, and it can be spent anywhere in the lap.
6. **Four circuits still have no measured width** (see above), so their corridors remain
   assumptions rather than measurements.
7. **Everything rests on one lap per track, and it isn't the lap being scored against.**
   Geometry comes from HAM's lap; the benchmark is the fastest lap by any driver, which at
   Bahrain, Mexico and Sochi in 2018 was someone else. Geometry quality also varies
   (Monaco's tunnel, telemetry dropouts).

The pipeline is split in two:

- **Rust** (`src/`) — the simulator core: track-curve fitting, the minimum-time
  collocation solver (Ipopt), lap-time integration. This is the part that matters.
- **Python** (`python_scripts/`) — a thin, one-time data-export layer: pulls real
  telemetry (FastF1) and track boundary geometry (OpenStreetMap) to CSV for Rust to
  consume. Not a second implementation of the physics.

See `results/DISCUSSION.md` for the full engineering narrative (V1 through V9): what was
tried, what was rejected and why, and how the model's accuracy evolved.

## Setup

**Rust / Ipopt** — requires Homebrew's `ipopt`:

```
brew install ipopt
./scripts/setup_ipopt.sh   # one-time; works around ipopt-sys's coin/ header path
cargo build --release
```

**Python** — used only to export data, not to run the simulator:

```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## CLI

```
cargo run --release -- <track> [mode] [spacing]
```

- **`<track>`** — track slug (e.g. `monaco`, `suzuka`, `singapore`), matching the
  `<slug>_` prefix under `data/` that the Python export scripts write. Default:
  `singapore`.
- **`[mode]`** — `racingline` (default) or `optimal`:
  - `racingline` — finds a free racing line: both the velocity profile *and* a lateral
    offset within the track's real width (from `data/<slug>_track_boundaries.csv`), so the
    solver can straighten corners the way a real driver does. Requires that boundaries file
    to exist. Treat its lap time as an upper bound rather than a prediction (see above).
  - `optimal` — the same minimum-time collocation solver, but pinned to the fixed
    FastF1-driven centerline (no lateral freedom). This is the validated mode (+5.5% vs.
    real pole times). Also serves as `racingline`'s own correctness baseline: since a zero
    lateral offset is always a feasible racing-line solution, `racingline`'s lap time can
    never come out slower than `optimal`'s — printed automatically as a sanity check.
- **`[spacing]`** — target collocation-point spacing in meters. Default: `5.0`. Lap time is
  still converging above that: 25 m reads 1–2 s optimistic because widely spaced samples
  skip curvature peaks, while 5 m is within ~0.1 s of a full 1 m solve. Coarser values are
  faster but flattering; finer ones cost time without changing the answer.

Examples:

```
cargo run --release -- monaco                      # racingline mode, 5m spacing
cargo run --release -- suzuka optimal 5             # fixed-line baseline
cargo run --release -- redbullring racingline 25    # coarser, faster, optimistic
```

### Output

Printed to stdout: input/resampled point counts, curvature stats, the solved lap time,
and (in `racingline` mode) the mean/max lateral offset used and the delta against the
fixed-line baseline.

Written to `data/`:
- `<slug>_racingline_path.csv` — X/Y racing line, boundaries, and speed. Read by
  `plot_racing_line.py`.
- `<slug>_curvature_debug.csv` — resampled curvature profile
- `<slug>_simulated_lap_optimal.csv` / `<slug>_simulated_lap_racingline.csv` — velocity
  vs. distance

The last two are debug output: nothing in the repo reads them, they're written on every
run for inspecting a solve by hand. The curvature profile in particular is what you want
when a track's lap time looks wrong — it's the input the whole solve rests on.

### Required per-track input data (from `data/`)

Produced by the Python export scripts below, keyed by the same `<slug>_` prefix:

| File | Produced by | Required for |
|---|---|---|
| `<slug>_track_geometry.csv` | `export_track.py` | both modes |
| `<slug>_reference_lap.csv` | `export_track.py` | the steering-rate limit and `derive_downforce.py` (skipped if missing) |
| `<slug>_track_boundaries.csv` | `export_tum_boundaries.py` (17 circuits) or `export_osm_boundaries.py` (the other 4) | `racingline` mode |
| `<slug>_drs_zones.csv` | `export_track.py` | DRS modeling (falls back to DRS-closed if missing) |
| `<slug>_aero_params.csv` | `derive_downforce.py` | per-track aero (warns and falls back to a coarse 3-tier default if missing). **Tracked in git**, unlike the rest of `data/` |

## Web UI

An interactive alternative to the CLI: pick a track, hit Solve, and explore the racing
line and speed trace in the browser (live solve — it runs the same `solve_racing_line`
pipeline as `cargo run -- <track> racingline`, not a replay of precomputed data).

```
cargo run --release --bin web
```

Then open `http://127.0.0.1:3000`. Requires the same `data/` inputs as CLI `racingline`
mode (see the table above) — the track selector only lists tracks that have a
`<slug>_track_boundaries.csv`.

## Python data-export scripts

All run from the repo root with the venv active. Each has a fuller usage docstring at
the top of its file.

```
python python_scripts/export_track.py --track Suzuka --lap-number 2
python python_scripts/export_tum_boundaries.py --track suzuka   # measured widths; preferred
python python_scripts/export_osm_boundaries.py --track suzuka --relation-name "Suzuka"
python python_scripts/derive_downforce.py --track suzuka
python python_scripts/validate_pole_times.py --track suzuka   # or no --track for all 21
python python_scripts/check_solver.py --save baseline.json    # offline solver regression check
python python_scripts/plot_racing_line.py --track suzuka
```

### Regenerating `data/` from scratch

`data/` is gitignored, so a fresh clone has no track data. These are the per-track export
recipes, recovered from the session history that produced the current files — treat the
relation-name search strings as *starting points*: OSM renames things, and the script fails
loudly (no match, or "Ambiguous match ... narrow the search string") rather than silently
producing something wrong. `--inspect-tags` is the quickest way to confirm a name resolves.

Boundaries come from two sources. For the 17 circuits in the TUM racetrack-database the
command takes no per-track arguments at all — `export_tum_boundaries.py --track <slug>` for
each of bahrain, catalunya, cota, hockenheim, hungaroring, interlagos, melbourne, mexico,
montreal, monza, redbullring, shanghai, silverstone, sochi, spa, suzuka, yasmarina. It caches
the source CSVs under `cache/tum_tracks/`, so re-runs need no network, and it is deterministic
(a re-export is byte-identical). Only the remaining 4 circuits need the OSM path and its
per-track search strings, which is what the table below records.

For `export_track.py` only three lap numbers were recorded — `Monza --lap-number 2`,
`Singapore --lap-number 3`, `Suzuka --lap-number 2` — and the rest used the default fastest
lap (check the endpoint gap it prints; see its docstring for why that matters). Note that
the output slug is just `--track` lowercased, so the value used for multi-word circuits must
have been a single token to produce slugs like `paulricard`; the exact strings weren't
recorded. Every track also needs `derive_downforce.py --track <slug>` afterwards.

**The 4 circuits that need the OSM path:**

| slug | `export_osm_boundaries.py --track <slug> ...` |
|---|---|
| baku | `--relation-name "Baku City Circuit"` |
| monaco | `--relation-name "Circuit de Monaco"` |
| paulricard | `--relation-name "Paul Ricard"` |
| singapore | `--relation-name "Marina Bay"` |

**The 17 that no longer need it**, with the strings that were used before
`export_tum_boundaries.py` replaced them — kept only because the TUM database could change
or drop a circuit:

| slug | former `export_osm_boundaries.py` arguments |
|---|---|
| bahrain | `--relation-name "Bahrain International Circuit"` ¹ |
| catalunya | `--relation-name "Catalunya GP FIA"` ¹ |
| cota | `--relation-name "Circuit of the Americas"` |
| hockenheim | `--relation-name "Hockenheim"` |
| hungaroring | `--relation-name "Hungaroring"` |
| interlagos | `--relation-name "Carlos Pace"` |
| melbourne | ² |
| mexico | no OSM relation found ² |
| montreal | `--relation-name "Gilles Villeneuve"` |
| monza | `--relation-name "Monza"` |
| redbullring | `--relation-name "Red Bull Ring" --jump-threshold 10 --outlier-threshold 6 --hysteresis-high 8 --hysteresis-low 5` |
| shanghai | `--relation-name "Shanghai"` |
| silverstone | `--relation-name "Silverstone Grand Prix"` |
| sochi | no OSM relation found ² |
| spa | `--relation-name "Spa-Francorchamps"` |
| suzuka | `--relation-name "Suzuka"` (verified 2026-09; 41 ways, no width tags) |
| yasmarina | `--relation-name "Yas Marina" --jump-threshold 13 --outlier-threshold 8 --hysteresis-high 13 --hysteresis-low 8` |

¹ The OSM relation resolves but bundles several real track layouts, giving a 35–63 m
registration residual. Unusable; the TUM data is what made these two work.
² These had no usable OSM boundaries at all, and are among the 6 circuits the TUM widths
brought into the working set. See `results/DISCUSSION.md` V9.

If the default Overpass instance refuses connections, pass
`--overpass-url https://overpass.kumi.systems/api/interpreter`.

- **`export_track.py`** — pulls a real FastF1 lap (default: 2018 Q, HAM) and exports
  centerline geometry, reference speed trace, and DRS zones. `--lap-number` matters:
  the fastest lap isn't always the cleanest telemetry (see its docstring).
- **`export_tum_boundaries.py`** — the preferred boundary source: downloads the TUM
  racetrack-database's satellite-measured track widths, registers the centerline against the
  FastF1 driven line via ICP, and writes the same format as the OSM path. Covers 17 of the 21
  circuits. Nothing is pushed back to that repository; the CSVs are cached under `cache/`.
- **`export_osm_boundaries.py`** — the fallback for the 4 circuits the TUM database doesn't
  cover. Pulls track-edge geometry from OpenStreetMap and registers it the same way. OSM has
  no width data, so width is imputed from `lanes` tags where they exist (Baku, Singapore) and
  otherwise falls back to a flat 7 m (Monaco, Paul Ricard).
- **`derive_downforce.py`** — derives per-track aero coefficients (`c_l`, `c_d`) from
  that track's own real telemetry (apex lateral acceleration, top speed), rather than a
  coarse 3-way classification. This closed most of the gap to real pole times, and its
  remaining noise is now the largest single source of error (see Open questions).
- **`validate_pole_times.py`** — runs the release solver against real 2018 qualifying pole
  times (fetched live via FastF1) across all 21 circuits, reporting both modes: the fixed
  line is the validated number, the racing line is shown for context. Every run appends one
  row per track to `results/validation_log.csv` (with the commit, and whether the tree was
  dirty) so runs can be compared per track later; `--no-log` skips that.
- **`check_solver.py`** — offline regression check: pinned-width invariants, convergence
  across every working track, and a diff against a saved run. Run it after changing the
  solver; it exits non-zero if something broke.
- **`plot_racing_line.py`** — plots a solved racing line against the track boundaries and
  reference centerline, colored by speed. Requires the corresponding `racingline` CLI run
  to have been done first.
