"""
Plot the racing line (X/Y) found by `cargo run -- <track> racingline <spacing>` against the
track boundaries and the FastF1 reference centerline, colored by speed.

Requires data/<slug>_racingline_path.csv (written by main.rs's "racingline" mode).

Usage:
  python plot_racing_line.py --track monaco
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--track", default="monaco", help="Track slug, matches the <slug>_ prefix used by main.rs")
args = parser.parse_args()
slug = args.track.lower()

df = pd.read_csv(f"data/{slug}_racingline_path.csv")
# Close the loop for plotting -- the CSV itself doesn't repeat the start point.
df = pd.concat([df, df.iloc[[0]]], ignore_index=True)

fig, ax = plt.subplots(figsize=(10, 10))

ax.plot(df["x_left"], df["y_left"], color="black", linewidth=1, label="Track edge")
ax.plot(df["x_right"], df["y_right"], color="black", linewidth=1)
ax.plot(df["x_ref"], df["y_ref"], color="gray", linewidth=0.8, linestyle="--", label="FastF1 reference line")

points = df[["x_line", "y_line"]].values.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, cmap="viridis", linewidth=2.5)
lc.set_array(df["speed_kmh"].values[:-1])
line = ax.add_collection(lc)
fig.colorbar(line, ax=ax, label="Speed [km/h]", shrink=0.7)

ax.set_aspect("equal")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title(f"Racing line: {args.track}")
ax.legend(loc="upper right")

out_path = f"data/{slug}_racing_line.png"
fig.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"Saved {out_path}")
