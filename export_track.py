"""
Export track geometry and reference speed trace from a FastF1 lap to CSV.

Outputs:
  data/track_geometry.csv   — X, Y columns (meters, FastF1 track-local 2D projection)
  data/reference_lap.csv    — Distance, Speed columns (meters, km/h)

Session: 2018 Singapore GP Qualifying, HAM (pole lap)
"""

import os
import fastf1
import pandas as pd

SESSION_YEAR = 2018
SESSION_NAME = "Singapore"
SESSION_TYPE = "Q"
DRIVER = "HAM"

os.makedirs("data", exist_ok=True)
os.makedirs("cache", exist_ok=True)

fastf1.Cache.enable_cache("cache")

session = fastf1.get_session(SESSION_YEAR, SESSION_NAME, SESSION_TYPE)
session.load(telemetry=True, weather=False, messages=False)

lap = session.laps.pick_drivers(DRIVER).pick_fastest()
tel = lap.get_telemetry()

# track_geometry.csv — sampled X/Y position along the driven line
geometry = tel[["X", "Y"]].dropna().reset_index(drop=True) / 10.0
geometry.to_csv("data/track_geometry.csv", index=False)

# reference_lap.csv — distance and speed for Stage 3 validation
reference = tel[["Distance", "Speed"]].dropna().reset_index(drop=True)
reference.to_csv("data/reference_lap.csv", index=False)

print(f"Exported {len(geometry)} geometry points → data/track_geometry.csv")
print(f"Exported {len(reference)} reference points → data/reference_lap.csv")
print(f"Lap: {SESSION_YEAR} {SESSION_NAME} {SESSION_TYPE}, driver {DRIVER}, lap {lap['LapNumber']}")
print(f"Lap time: {lap['LapTime']}")
print(f"Track length (approx): {tel['Distance'].max():.0f} m")
