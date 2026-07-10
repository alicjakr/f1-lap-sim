"""
Export track boundary limits from multi-driver position data.

For each point along the HAM centerline, collects all car positions
from all qualifying laps and projects them onto the local track normal.
The 2nd and 98th percentiles of that lateral spread give the right and
left boundary estimates (cars never quite reach the walls, so percentiles
give a more stable estimate than absolute min/max).

Output: data/track_boundaries.csv
  s       — arc length along centerline (m)
  x, y    — centerline position (m)
  n_left  — max lateral offset to the left, positive (m)
  n_right — max lateral offset to the right, negative (m)

Convention: n > 0 is left of the driving direction, n < 0 is right.
The racing line optimizer uses: n_right[i] <= n[i] <= n_left[i].
"""

import fastf1
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

fastf1.Cache.enable_cache("cache")

session = fastf1.get_session(2018, "Singapore", "Q")
session.load(telemetry=True, weather=False, messages=False)

# --- Centerline from HAM fastest lap ---
ham_lap = session.laps.pick_drivers("HAM").pick_fastest()
ham_tel = ham_lap.get_telemetry()
center = ham_tel[["X", "Y"]].dropna().reset_index(drop=True).values / 10.0
cx, cy = center[:, 0], center[:, 1]
n_pts = len(cx)

# Arc-length parameterization
diffs = np.hypot(np.diff(cx), np.diff(cy))
s = np.concatenate([[0.0], np.cumsum(diffs)])

# Unit tangent vectors (central differences)
tx = np.gradient(cx, s)
ty = np.gradient(cy, s)
mag = np.hypot(tx, ty)
tx, ty = tx / mag, ty / mag

# Unit normal: 90° CCW from tangent = left of driving direction
nx, ny = -ty, tx

# --- Collect all car positions from pos_data ---
all_x, all_y = [], []
for drv in session.drivers:
    if drv not in session.pos_data:
        continue
    xy = session.pos_data[drv][["X", "Y"]].dropna().values / 10.0
    all_x.append(xy[:, 0])
    all_y.append(xy[:, 1])

all_x = np.concatenate(all_x)
all_y = np.concatenate(all_y)
print(f"Collected {len(all_x):,} position samples across {len(session.drivers)} drivers")

# --- For each car position, find nearest centerline point ---
tree = cKDTree(np.column_stack([cx, cy]))
_, nearest = tree.query(np.column_stack([all_x, all_y]))

# Lateral offset: project displacement onto normal at nearest centerline point
dx = all_x - cx[nearest]
dy = all_y - cy[nearest]
lateral = dx * nx[nearest] + dy * ny[nearest]

# --- Per centerline point: collect lateral offsets and take percentiles ---
buckets = [[] for _ in range(n_pts)]
for pos_i, c_i in enumerate(nearest):
    buckets[c_i].append(lateral[pos_i])

n_left = np.zeros(n_pts)
n_right = np.zeros(n_pts)
fallback_count = 0

for i, bucket in enumerate(buckets):
    if len(bucket) < 10:
        n_left[i] = 6.0
        n_right[i] = -6.0
        fallback_count += 1
    else:
        arr = np.array(bucket)
        n_left[i] = float(np.percentile(arr, 98))
        n_right[i] = float(np.percentile(arr, 2))

if fallback_count:
    print(f"Warning: {fallback_count} points used fallback width (too few samples)")

# --- Export ---
df = pd.DataFrame({
    "s":       s,
    "x":       cx,
    "y":       cy,
    "n_left":  n_left,
    "n_right": n_right,
})
df.to_csv("data/track_boundaries.csv", index=False)

avg_width = (n_left - n_right).mean()
print(f"Exported {len(df)} boundary samples → data/track_boundaries.csv")
print(f"Average track width: {avg_width:.1f} m  "
      f"(left {n_left.mean():.1f} m, right {abs(n_right.mean()):.1f} m)")
