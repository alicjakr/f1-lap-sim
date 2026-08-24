import argparse

import pandas as pd
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--track", default="singapore", help="Track slug, matches the <slug>_ prefix used by export_track.py and main.rs")
args = parser.parse_args()
slug = args.track.lower()

sim = pd.read_csv(f"data/{slug}_simulated_lap.csv")
ref = pd.read_csv(f"data/{slug}_reference_lap.csv")

plt.plot(ref["Distance"], ref["Speed"], label=f"Reference (HAM 2018, {args.track})")
plt.plot(sim["Distance"], sim["Speed"], label="Simulated")
plt.xlabel("Distance [m]")
plt.ylabel("Speed [km/h]")
plt.legend()
plt.savefig(f"data/{slug}_comparison.png", dpi=150)
