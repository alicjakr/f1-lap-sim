# f1-lap-sim

A physics-based F1 lap time simulator. A Rust minimum-time optimal-control solver finds
the fastest velocity profile — and optionally the fastest line within the track's real
width — around a track, given real track geometry and boundaries.

Validated against real 2018 F1 qualifying pole times across 15 tracks: the **fixed-line**
solver (`optimal` mode) comes out **+3.8% slower than pole on average**. The **free racing
line** (`racingline` mode, the default) currently comes out ~8.5% *faster* than pole, which
is not a lap-time prediction — it's an upper bound on what line choice alone could buy a
point-mass car, which pays nothing for changing direction beyond its grip limit. See V7 in
`results/DISCUSSION.md` for the audit behind both numbers and what a fix would take.

The pipeline is split in two:

- **Rust** (`src/`) — the simulator core: track-curve fitting, the minimum-time
  collocation solver (Ipopt), lap-time integration. This is the part that matters.
- **Python** (`python_scripts/`) — a thin, one-time data-export layer: pulls real
  telemetry (FastF1) and track boundary geometry (OpenStreetMap) to CSV for Rust to
  consume. Not a second implementation of the physics.

See `results/DISCUSSION.md` for the full engineering narrative (V1 through V7): what was
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
    FastF1-driven centerline (no lateral freedom). This is the validated mode (+3.8% vs.
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
- `<slug>_curvature_debug.csv` — resampled curvature profile
- `<slug>_simulated_lap_optimal.csv` / `<slug>_simulated_lap_racingline.csv` — velocity
  vs. distance
- `<slug>_racingline_path.csv` — X/Y racing line, boundaries, and speed (for plotting)

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
- **`validate_pole_times.py`** — runs the release solver against real 2018 qualifying
  pole times (fetched live via FastF1) across all working tracks and reports the gap.
- **`plot_racing_line.py`** — plots a solved racing line against the track boundaries and
  reference centerline, colored by speed. Requires the corresponding `racingline` CLI run
  to have been done first.
