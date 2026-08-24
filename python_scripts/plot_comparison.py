import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--track", default="singapore", help="Track slug, matches the <slug>_ prefix used by export_track.py and main.rs")
args = parser.parse_args()
slug = args.track.lower()

sim = pd.read_csv(f"data/{slug}_simulated_lap.csv")
ref = pd.read_csv(f"data/{slug}_reference_lap.csv")

plt.plot(ref["Distance"], ref["Speed"], label=f"Reference (HAM 2018, {args.track})")
plt.plot(sim["Distance"], sim["Speed"], label="Simulated (two-pass)")

# Only present once solve_min_time (optimal.rs) has been run for this track; skipped
# otherwise rather than erroring, since not every track will have this yet.
optimal_path = Path(f"data/{slug}_simulated_lap_optimal.csv")
if optimal_path.exists():
    optimal = pd.read_csv(optimal_path)
    plt.plot(optimal["Distance"], optimal["Speed"], label="Simulated (optimal control)")

plt.xlabel("Distance [m]")
plt.ylabel("Speed [km/h]")
plt.legend()
plt.savefig(f"data/{slug}_comparison.png", dpi=150)
