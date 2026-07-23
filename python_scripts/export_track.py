"""
Export track geometry and reference speed trace from a FastF1 lap to CSV.

Outputs:
  data/track_geometry.csv   — Distance, X, Y columns (meters)
  data/reference_lap.csv    — Distance, Speed columns (meters, km/h)

Distance is FastF1's own telemetry channel (integrated from Speed), not recomputed
from X/Y. On fast tracks the position (X/Y) samples can go stale/repeat between GPS
fixes at high speed, so summing Euclidean distance between consecutive X/Y points
systematically undercounts true distance — badly enough on a fast circuit like Suzuka
(~11% short, non-uniformly) to misalign the geometry against reference_lap.csv's
Distance-based axis. FastF1's own Distance channel doesn't have that problem, since
it's integrated from Speed rather than resampled position.

Session: 2018 Singapore GP Qualifying, HAM (pole lap)

LAP_NUMBER: pick_fastest() isn't safe to trust blindly — it only optimizes for lap
time, blind to telemetry quality. At Suzuka, HAM's fastest lap (lap 8, 1:27.760) has
a genuine ~630m gap between its first and last X/Y position samples (a real position-
telemetry dropout, not a computation bug), despite FastF1 marking it IsAccurate=True
(that flag is based on sector-time consistency, not raw position quality). Lap 2
(1:28.702, only 0.94s slower) has a clean 1.2m endpoint gap. Check a candidate lap's
endpoint gap before trusting it; set LAP_NUMBER explicitly once you've picked a good one.
"""

import fastf1
import pandas as pd

SESSION_YEAR = 2018
SESSION_NAME = "Singapore"
SESSION_TYPE = "Q"
DRIVER = "HAM"
LAP_NUMBER = None  # None = fastest lap; see LAP_NUMBER note above for why that's not used here

fastf1.Cache.enable_cache("cache")

session = fastf1.get_session(SESSION_YEAR, SESSION_NAME, SESSION_TYPE)
session.load(telemetry=True, weather=False, messages=False)

driver_laps = session.laps.pick_drivers(DRIVER)
if LAP_NUMBER is not None:
    lap = driver_laps[driver_laps["LapNumber"] == LAP_NUMBER].iloc[0]
else:
    lap = driver_laps.pick_fastest()
tel = lap.get_telemetry()

# track_geometry.csv — X/Y position along the driven line, indexed by FastF1's own
# Distance channel rather than a recomputed chord length.
geometry = tel[["Distance", "X", "Y"]].dropna().reset_index(drop=True)
geometry["X"] = geometry["X"] / 10.0
geometry["Y"] = geometry["Y"] / 10.0
geometry.to_csv("data/track_geometry.csv", index=False)

# reference_lap.csv — distance and speed for Stage 3 validation
reference = tel[["Distance", "Speed"]].dropna().reset_index(drop=True)
reference.to_csv("data/reference_lap.csv", index=False)

print(f"Exported {len(geometry)} geometry points → data/track_geometry.csv")
print(f"Exported {len(reference)} reference points → data/reference_lap.csv")
print(f"Lap: {SESSION_YEAR} {SESSION_NAME} {SESSION_TYPE}, driver {DRIVER}, lap {lap['LapNumber']}")
print(f"Lap time: {lap['LapTime']}")
print(f"Track length (approx): {tel['Distance'].max():.0f} m")
