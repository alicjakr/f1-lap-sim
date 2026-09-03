"""
Export track geometry and reference speed trace from a FastF1 lap to CSV.

Outputs (per track/driver, so multiple tracks can coexist under data/):
  data/<slug>_track_geometry.csv   — Distance, X, Y columns (meters)
  data/<slug>_reference_lap.csv    — Distance, Speed columns (meters, km/h)
  data/<slug>_drs_zones.csv        — Distance, drs_open columns (meters, bool)

Distance is FastF1's own telemetry channel (integrated from Speed), not recomputed
from X/Y. On fast tracks the position (X/Y) samples can go stale/repeat between GPS
fixes at high speed, so summing Euclidean distance between consecutive X/Y points
systematically undercounts true distance — badly enough on a fast circuit like Suzuka
(~11% short, non-uniformly) to misalign the geometry against reference_lap.csv's
Distance-based axis. FastF1's own Distance channel doesn't have that problem, since
it's integrated from Speed rather than resampled position.

--lap isn't safe to leave at "fastest" by default — pick_fastest() only optimizes for
lap time, blind to telemetry quality. At Suzuka, HAM's fastest 2018 Q lap (lap 8,
1:27.760) has a genuine ~630m gap between its first and last X/Y position samples (a
real position-telemetry dropout, not a computation bug), despite FastF1 marking it
IsAccurate=True (that flag is based on sector-time consistency, not raw position
quality). Lap 2 (1:28.702, only 0.94s slower) has a clean 1.2m endpoint gap. Check a
candidate lap's endpoint gap (printed below) before trusting it, then pass --lap
explicitly once you've picked a good one.

Examples:
  python export_track.py --track Singapore --lap-number 3   # HAM 2018 Q pole lap
  python export_track.py --track Suzuka --lap-number 2       # HAM 2018 Q, dropout-free lap
"""

import argparse

import fastf1
import pandas as pd

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--year", type=int, default=2018)
parser.add_argument("--track", default="Singapore", help="FastF1 session location, e.g. Singapore, Suzuka")
parser.add_argument("--session-type", default="Q")
parser.add_argument("--driver", default="HAM")
parser.add_argument("--lap-number", type=int, default=None, help="Explicit lap number; default picks the fastest lap (verify its endpoint gap first)")
args = parser.parse_args()

slug = args.track.lower()

fastf1.Cache.enable_cache("cache")

session = fastf1.get_session(args.year, args.track, args.session_type)
session.load(telemetry=True, weather=False, messages=False)

driver_laps = session.laps.pick_drivers(args.driver)
if args.lap_number is not None:
    lap = driver_laps[driver_laps["LapNumber"] == args.lap_number].iloc[0]
else:
    lap = driver_laps.pick_fastest()
tel = lap.get_telemetry()

endpoint_gap = ((tel["X"].iloc[0] - tel["X"].iloc[-1]) ** 2 + (tel["Y"].iloc[0] - tel["Y"].iloc[-1]) ** 2) ** 0.5 / 10.0
print(f"Lap {lap['LapNumber']} endpoint gap: {endpoint_gap:.1f} m (large values indicate a position-telemetry dropout; see module docstring)")

# track_geometry.csv — X/Y position along the driven line, indexed by FastF1's own
# Distance channel rather than a recomputed chord length.
geometry = tel[["Distance", "X", "Y"]].dropna().reset_index(drop=True)
geometry["X"] = geometry["X"] / 10.0
geometry["Y"] = geometry["Y"] / 10.0
geometry_path = f"data/{slug}_track_geometry.csv"
geometry.to_csv(geometry_path, index=False)

# reference_lap.csv — distance and speed for Stage 3 validation
reference = tel[["Distance", "Speed"]].dropna().reset_index(drop=True)
reference_path = f"data/{slug}_reference_lap.csv"
reference.to_csv(reference_path, index=False)

# drs_zones.csv — per-point DRS-open flag along this same driven lap, used as a proxy for
# the real FIA zone boundaries (exact zone geometry isn't in FastF1's data; this driver's
# actual DRS usage on a competitive quali lap tracks the real zone closely). FastF1's DRS
# channel: 0/1 = off, 8 = detected/eligible (not yet activated), 10/12/14 = active.
drs = tel[["Distance", "DRS"]].dropna().reset_index(drop=True)
drs["drs_open"] = drs["DRS"] >= 10
drs = drs[["Distance", "drs_open"]]
drs_path = f"data/{slug}_drs_zones.csv"
drs.to_csv(drs_path, index=False)

print(f"Exported {len(geometry)} geometry points → {geometry_path}")
print(f"Exported {len(reference)} reference points → {reference_path}")
print(f"Exported {len(drs)} DRS-zone points ({int(drs['drs_open'].sum())} open) → {drs_path}")
print(f"Lap: {args.year} {args.track} {args.session_type}, driver {args.driver}, lap {lap['LapNumber']}")
print(f"Lap time: {lap['LapTime']}")
print(f"Track length (approx): {tel['Distance'].max():.0f} m")
