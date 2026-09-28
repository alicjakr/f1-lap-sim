# f1-lap-sim

A physics-based F1 lap time simulator. A Rust minimum-time optimal-control solver finds
the fastest velocity profile — and optionally the fastest line within the track's real
width — around a track, given real track geometry and boundaries.

There are two modes, and the difference between them matters. **`racingline` is the more
complete model**: it optimizes the path as well as the speed, and contains the fixed-line
case exactly (a zero lateral offset). **`optimal` is the narrower one**: it pins the car to
the lap a driver actually drove and optimizes only the speed profile.

The narrower mode is the validated one, because everything it needs was measured: **+4.6%
slower than real 2018 pole times** across 15 tracks. The more complete mode currently reads
**~7.6% faster** than any real lap — not because the formulation is wrong, but because its
extra freedom is under-constrained. OSM has no real track widths, and the corridor it does
get is centred on a lap that was already a racing line, so the solver re-spends width the
driver had already used. Give it real track edges and a car that resists changing direction,
and it should become the better predictor; until then its lap time is a bound, not a
prediction. See V7 and V8 in `results/DISCUSSION.md` for the audit behind both numbers.

## What it does and doesn't model

**Works:**

- **Speed optimization on a known path.** Given the line a driver actually drove, it finds
  the fastest way to drive it — where to brake, how hard, when to get back on power. This
  is the validated part: +4.6% mean vs. real 2018 pole times across 15 tracks.
- **Combined grip.** Braking and cornering share one friction budget (a friction ellipse),
  so the car can't brake at full force mid-corner.
- **Aerodynamics per circuit.** Downforce and drag are derived from each track's own
  telemetry rather than guessed, and derived from apex speeds and top speed — not from lap
  time, so checking the result against lap times isn't circular.
- **Power, drag and DRS**, including DRS only where the real car had it open.
- **The hybrid's energy budget.** The ICE runs all lap; the MGU-K's 4 MJ (about 33 s at
  full deployment) is a budget the solver chooses where to spend.
- **A steering-rate limit**, so the path can't change direction faster than the real car did.
- **The solve itself.** Minimum-time optimal control over the whole lap at once, with
  hand-derived derivatives checked against Ipopt's own finite-difference checker, and a
  start/finish line that wraps properly. All 15 tracks converge in about a second.

**Doesn't work / not modelled:**

- **Picking the line is not trustworthy.** `racingline` mode reads ~7.6% faster than any
  real lap. Two reasons: OSM has no real track widths (every track falls back to a 7 m
  guess), and the corridor is centred on a lap that was already a racing line, so the
  solver re-spends width the driver had already used.
- **The car is a point mass.** No yaw inertia, no weight transfer, no tire slip angles.
  Measured to be worth under a second on lap time, but it's why a freely chosen line can
  find time a real car couldn't.
- **A sub-grid artifact.** Give the car 1 cm of lateral freedom and it still finds 0.3–1.1 s
  by weaving, because the grip limit is only enforced at the solver's sample points.
- **Harvesting isn't modelled.** The MGU-K's 4 MJ per lap is granted rather than earned
  under braking, and deployment can be spent anywhere in the lap.
- **No tire wear, fuel burn, track evolution, elevation or banking.** Reasonable for a
  single qualifying lap, less so for anything else.
- **6 of the 21 2018 circuits don't work**, all for data reasons (see V6 in `DISCUSSION.md`).

## Open questions

Things worth solving, roughly in order of how much they'd change the results:

1. **Where does the track edge actually lie?** The corridor needs real widths and, more
   importantly, real asymmetry — at an apex the inside edge is *at* the driven line, not
   2.5 m beyond it. OSM doesn't carry this (confirmed for Suzuka: 41 ways, no width tags),
   so it needs another source. This is the whole racing-line gap.
2. **The weave.** Enforcing the grip limit between sample points, not just at them, would
   close part of it; the rest is the point-mass assumption.
3. **Harvesting.** Deployment is now budgeted (V8), but the budget is granted rather than
   recovered under braking, and it can be spent anywhere in the lap.
4. **A real vehicle model** (yaw dynamics, load transfer, tire slip). Measured as worth
   ≤1 s, so it's for correctness rather than accuracy.
5. **Thin aero evidence on some tracks.** Monza derives its downforce from 2 corners;
   `derive_downforce.py` warns below 10.
6. **Everything rests on one lap per track.** The geometry is one driver's GPS trace, and
   its quality varies (Monaco's tunnel, telemetry dropouts).

The pipeline is split in two:

- **Rust** (`src/`) — the simulator core: track-curve fitting, the minimum-time
  collocation solver (Ipopt), lap-time integration. This is the part that matters.
- **Python** (`python_scripts/`) — a thin, one-time data-export layer: pulls real
  telemetry (FastF1) and track boundary geometry (OpenStreetMap) to CSV for Rust to
  consume. Not a second implementation of the physics.

See `results/DISCUSSION.md` for the full engineering narrative (V1 through V8): what was
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
    FastF1-driven centerline (no lateral freedom). This is the validated mode (+4.6% vs.
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
| `<slug>_track_boundaries.csv` | `export_osm_boundaries.py` | `racingline` mode |
| `<slug>_drs_zones.csv` | `export_track.py` | DRS modeling (falls back to DRS-closed if missing) |
| `<slug>_aero_params.csv` | `derive_downforce.py` | per-track aero (falls back to a coarse 3-tier default if missing) |

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
python python_scripts/export_osm_boundaries.py --track suzuka --relation-name "Suzuka"
python python_scripts/derive_downforce.py --track suzuka
python python_scripts/validate_pole_times.py --track suzuka   # or no --track for all 15
python python_scripts/check_solver.py --save baseline.json    # offline solver regression check
python python_scripts/plot_racing_line.py --track suzuka
```

### Regenerating `data/` from scratch

`data/` is gitignored, so a fresh clone has no track data. These are the per-track export
recipes, recovered from the session history that produced the current files — treat the
relation-name search strings as *starting points*: OSM renames things, and the script fails
loudly (no match, or "Ambiguous match ... narrow the search string") rather than silently
producing something wrong. `--inspect-tags` is the quickest way to confirm a name resolves.

The boundary step is the one with per-track arguments, so it's the one recorded here. For
`export_track.py` only three lap numbers were recorded — `Monza --lap-number 2`,
`Singapore --lap-number 3`, `Suzuka --lap-number 2` — and the rest used the default fastest
lap (check the endpoint gap it prints; see its docstring for why that matters). Note that
the output slug is just `--track` lowercased, so the value used for multi-word circuits must
have been a single token to produce slugs like `paulricard`; the exact strings weren't
recorded. Every track also needs `derive_downforce.py --track <slug>` afterwards.

| slug | `export_osm_boundaries.py --track <slug> ...` |
|---|---|
| bahrain | `--relation-name "Bahrain International Circuit"` ¹ |
| baku | `--relation-name "Baku City Circuit"` |
| catalunya | `--relation-name "Catalunya GP FIA"` ¹ |
| cota | `--relation-name "Circuit of the Americas"` |
| hockenheim | `--relation-name "Hockenheim"` |
| hungaroring | `--relation-name "Hungaroring"` |
| interlagos | `--relation-name "Carlos Pace"` |
| mexico | no OSM relation found ² |
| monaco | `--relation-name "Circuit de Monaco"` |
| montreal | `--relation-name "Gilles Villeneuve"` |
| monza | `--relation-name "Monza"` |
| paulricard | `--relation-name "Paul Ricard"` |
| redbullring | `--relation-name "Red Bull Ring" --jump-threshold 10 --outlier-threshold 6 --hysteresis-high 8 --hysteresis-low 5` |
| shanghai | `--relation-name "Shanghai"` |
| silverstone | `--relation-name "Silverstone Grand Prix"` |
| singapore | `--relation-name "Marina Bay"` ² |
| sochi | no OSM relation found ² |
| spa | `--relation-name "Spa-Francorchamps"` |
| suzuka | `--relation-name "Suzuka"` (verified 2026-09; 41 ways, no width tags) |
| yasmarina | `--relation-name "Yas Marina" --jump-threshold 13 --outlier-threshold 8 --hysteresis-high 13 --hysteresis-low 8` |

¹ Boundary data unusable even when the relation resolves — the relation bundles several
real track layouts (registration residual 35–63 m). Kept here for completeness; these
tracks are excluded from the working set.
² Not in the working 15: Mexico and Sochi have no OSM circuit relation at all, and
Singapore's geometry exceeds the solver's steering-rate/small-angle limit. See
`results/DISCUSSION.md` V6.

If the default Overpass instance refuses connections, pass
`--overpass-url https://overpass.kumi.systems/api/interpreter`.

- **`export_track.py`** — pulls a real FastF1 lap (default: 2018 Q, HAM) and exports
  centerline geometry, reference speed trace, and DRS zones. `--lap-number` matters:
  the fastest lap isn't always the cleanest telemetry (see its docstring).
- **`export_osm_boundaries.py`** — pulls real track-edge geometry from OpenStreetMap and
  registers it against the FastF1 driven line via ICP, producing left/right boundary
  offsets for the racing-line solver. Prefer `--relation-name` (a named OSM circuit
  relation); `--bbox` is a weaker fallback for circuits with no relation. Works for 15 of
  21 2018-calendar tracks — see the script's docstring for the rest.
- **`derive_downforce.py`** — derives per-track aero coefficients (`c_l`, `c_d`) from
  that track's own real telemetry (apex lateral acceleration, top speed), rather than a
  coarse 3-way classification. This closed most of the gap to real pole times.
- **`validate_pole_times.py`** — runs the release solver against real 2018 qualifying pole
  times (fetched live via FastF1) across all working tracks, reporting both modes: the
  fixed line is the validated number, the racing line is shown for context.
- **`check_solver.py`** — offline regression check: pinned-width invariants, convergence
  across every working track, and a diff against a saved run. Run it after changing the
  solver; it exits non-zero if something broke.
- **`plot_racing_line.py`** — plots a solved racing line against the track boundaries and
  reference centerline, colored by speed. Requires the corresponding `racingline` CLI run
  to have been done first.
